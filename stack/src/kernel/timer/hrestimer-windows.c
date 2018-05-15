/**
********************************************************************************
\file   hrestimer-windows.c

\brief  High-resolution timer module for Windows

This module is the target specific implementation of the high-resolution
timer module for Windows.

\ingroup module_hrestimer
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2012, SYSTEC electronic GmbH
Copyright (c) 2016, B&R Industrial Automation GmbH
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the copyright holders nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
------------------------------------------------------------------------------*/

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <common/oplkinc.h>
#include <kernel/hrestimer.h>

//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#define TIMER_COUNT             2           ///< number of high-resolution timers

#define HRTIMER_HDL_EVENT       0
#define HRTIMER_HDL_TIMER0      1
#define HRTIMER_HDL_TIMER1      2
#define HRTIMER_HDL_COUNT       3

/* macros for timer handles */
#define TIMERHDL_MASK           0x0FFFFFFF
#define TIMERHDL_SHIFT          28
#define HDL_TO_IDX(hdl)         ((hdl >> TIMERHDL_SHIFT) - 1)
#define HDL_INIT(idx)           ((idx + 1) << TIMERHDL_SHIFT)
#define HDL_INC(hdl)            (((hdl + 1) & TIMERHDL_MASK) | (hdl & ~TIMERHDL_MASK))

//------------------------------------------------------------------------------
// module global vars
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// global function prototypes
//------------------------------------------------------------------------------

//============================================================================//
//          P R I V A T E   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------

/**
\brief  High-resolution timer information structure

The structure contains all necessary information for a high-resolution timer.
*/
typedef struct
{
    tTimerEventArg      eventArg;           ///< Event argument
    tTimerkCallback     pfnCallback;        ///< Pointer to timer callback function
    LARGE_INTEGER       dueTime;            ///< Due time for continuous timers, otherwise 0
} tHresTimerInfo;

/**
\brief  High-resolution timer instance

The structure defines a high-resolution timer module instance.
*/
typedef struct
{
    tHresTimerInfo      aTimerInfo[TIMER_COUNT];    ///< Array with timer information for a set of timers
    HANDLE              threadHandle;               ///< Handle of the hrtimer worker thread
    HANDLE              aHandle[HRTIMER_HDL_COUNT]; ///< Array of event handles of the hrtimer
} tHresTimerInstance;

// Function prototypes for undocumented ntdll.dll functions
typedef LONG (NTAPI* tNtQueryTimerResolution)(OUT PULONG MinimumResolution,
                                              OUT PULONG MaximumResolution,
                                              OUT PULONG CurrentResolution);
typedef LONG (NTAPI* tNtSetTimerResolution)(IN ULONG DesiredResolution,
                                            IN BOOLEAN SetResolution,
                                            OUT PULONG CurrentResolution);

//------------------------------------------------------------------------------
// module local vars
//------------------------------------------------------------------------------
static tHresTimerInstance       hresTimerInstance_l;
static HINSTANCE                hInstLibNtDll_l;            ///< Instance handle of the loaded NT kernel DLL

// Function pointers to undocumented ntdll.dll functions
static tNtQueryTimerResolution  NtQueryTimerResolution;
static tNtSetTimerResolution    NtSetTimerResolution;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static tOplkError   setMaxTimerResolution(void);
static void         restoreTimerResolution(void);
static void         callTimerCb(UINT index_p);
static DWORD WINAPI timerThread(LPVOID pArgument_p);

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief    Initialize high-resolution timer module

The function initializes the high-resolution timer module

\return Returns a tOplkError error code.

\ingroup module_hrestimer
*/
//------------------------------------------------------------------------------
tOplkError hrestimer_init(void)
{
    tOplkError      ret = kErrorOk;
    DWORD           threadId;

    // Set the system timer to maximum resolution
    ret = setMaxTimerResolution();
    if (ret != kErrorOk)
        return ret;

    // Clear the module instance structure
    OPLK_MEMSET(&hresTimerInstance_l, 0, sizeof(hresTimerInstance_l));

    // Create two unnamed waitable timers
    hresTimerInstance_l.aHandle[HRTIMER_HDL_TIMER0] = CreateWaitableTimer(NULL, FALSE, NULL);
    if (hresTimerInstance_l.aHandle[HRTIMER_HDL_TIMER0] == NULL)
    {
        DEBUG_LVL_ERROR_TRACE("CreateWaitableTimer failed (%d)\n", GetLastError());
        return kErrorNoResource;
    }

    hresTimerInstance_l.aHandle[HRTIMER_HDL_TIMER1] = CreateWaitableTimer(NULL, FALSE, NULL);
    if (hresTimerInstance_l.aHandle[HRTIMER_HDL_TIMER1] == NULL)
    {
        DEBUG_LVL_ERROR_TRACE("CreateWaitableTimer failed (%d)\n", GetLastError());
        return kErrorNoResource;
    }

    // Create event for signaling shutdown
    hresTimerInstance_l.aHandle[HRTIMER_HDL_EVENT] = CreateEvent(NULL, FALSE, FALSE, NULL);

    // Create the thread to begin execution on its own
    hresTimerInstance_l.threadHandle = CreateThread(NULL, 0, timerThread, NULL, 0, &threadId);
    if (hresTimerInstance_l.threadHandle == NULL)
         return kErrorNoResource;

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief    Shut down high-resolution timer module

The function shuts down the high-resolution timer module.

\return Returns a tOplkError error code.

\ingroup module_hrestimer
*/
//------------------------------------------------------------------------------
tOplkError hrestimer_exit(void)
{
    tOplkError      ret = kErrorOk;

    // Signal shutdown to the thread and wait for it to terminate
    SetEvent(hresTimerInstance_l.aHandle[HRTIMER_HDL_EVENT]);
    WaitForSingleObject(hresTimerInstance_l.threadHandle, INFINITE);

    // Close the thread and the signal event handle
    CloseHandle(hresTimerInstance_l.threadHandle);
    CloseHandle(hresTimerInstance_l.aHandle[HRTIMER_HDL_EVENT]);

    // Close the timer handles
    CloseHandle(hresTimerInstance_l.aHandle[HRTIMER_HDL_TIMER0]);
    CloseHandle(hresTimerInstance_l.aHandle[HRTIMER_HDL_TIMER1]);

    // Clear instance structure
    OPLK_MEMSET(&hresTimerInstance_l, 0, sizeof(hresTimerInstance_l));

    // Restore the standard timer resolution
    restoreTimerResolution();

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief    Modify a high-resolution timer

The function modifies the timeout of the timer with the specified handle.
If the handle to which the pointer points to is zero, the timer must be created
first. If it is not possible to stop the old timer, this function always assures
that the old timer does not trigger the callback function with the same handle
as the new timer. That means the callback function must check the passed handle
with the one returned by this function. If these are unequal, the call can be
discarded.

\param[in,out]  pTimerHdl_p         Pointer to timer handle.
\param[in]      time_p              Relative timeout in [ns].
\param[in]      pfnCallback_p       Callback function, which is called when timer expires.
                                    (The function is called mutually exclusive with
                                    the Edrv callback functions (Rx and Tx)).
\param[in]      argument_p          User-specific argument.
\param[in]      fContinue_p         If TRUE, the callback function will be called continuously.
                                    Otherwise, it is a one-shot timer.

\return Returns a tOplkError error code.

\ingroup module_hrestimer
*/
//------------------------------------------------------------------------------
tOplkError hrestimer_modifyTimer(tTimerHdl* pTimerHdl_p,
                                 ULONGLONG time_p,
                                 tTimerkCallback pfnCallback_p,
                                 ULONG argument_p,
                                 BOOL fContinue_p)
{
    tOplkError          ret = kErrorOk;
    BOOL                fRet;
    UINT                index;
    tHresTimerInfo*     pTimerInfo;
    HANDLE              hTimer;
    LARGE_INTEGER       dueTime;

    // check pointer to handle
    if (pTimerHdl_p == NULL)
    {
        DEBUG_LVL_ERROR_TRACE("%s() Invalid timer handle\n", __func__);
        return kErrorTimerInvalidHandle;
    }

    if (*pTimerHdl_p == 0)
    {   // no timer created yet -> search free timer info structure
        pTimerInfo = &hresTimerInstance_l.aTimerInfo[0];
        for (index = 0; index < TIMER_COUNT; index++, pTimerInfo++)
        {
            if (pTimerInfo->pfnCallback == NULL)
            {   // free structure found
                break;
            }
        }

        if (index >= TIMER_COUNT)
        {   // no free structure found
            DEBUG_LVL_ERROR_TRACE("%s() Invalid timer index: %d\n", __func__, index);
            return kErrorTimerNoTimerCreated;
        }
        pTimerInfo->eventArg.timerHdl.handle = HDL_INIT(index);
    }
    else
    {
        index = (UINT)HDL_TO_IDX(*pTimerHdl_p);
        if (index >= TIMER_COUNT)
        {   // invalid handle
            DEBUG_LVL_ERROR_TRACE("%s() Invalid timer index: %d\n", __func__, index);
            return kErrorTimerInvalidHandle;
        }
        pTimerInfo = &hresTimerInstance_l.aTimerInfo[index];
    }

    // increment timer handle (if timer expires right after this statement,
    // the user would detect an unknown timer handle and discard it)
    pTimerInfo->eventArg.timerHdl.handle = HDL_INC(pTimerInfo->eventArg.timerHdl.handle);

    // calculate duetime [100 ns] (negative value = relative time)
    dueTime.QuadPart = (LONGLONG)time_p / -100LL;
    if (dueTime.QuadPart > -10000LL)
    {   // duetime is less than 1 ms
        dueTime.QuadPart = -10000LL;
    }

    if (fContinue_p != FALSE)
    {   // continuous timer
        pTimerInfo->dueTime = dueTime;
    }
    else
    {   // one-shot timer
        pTimerInfo->dueTime.QuadPart = 0LL;
    }

    pTimerInfo->eventArg.argument.value = argument_p;
    pTimerInfo->pfnCallback = pfnCallback_p;

    *pTimerHdl_p = pTimerInfo->eventArg.timerHdl.handle;

    // Configure timer
    hTimer = hresTimerInstance_l.aHandle[index + HRTIMER_HDL_TIMER0];
    fRet = SetWaitableTimer(hTimer, &dueTime, 0L, NULL, NULL, 0);
    if (!fRet)
    {
        DEBUG_LVL_ERROR_TRACE("SetWaitableTimer failed (%d)\n", GetLastError());
        return kErrorTimerNoTimerCreated;
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief    Delete a high-resolution timer

The function deletes a created high-resolution timer. The timer is specified
by its timer handle. After deleting, the handle is reset to zero.

\param[in,out]  pTimerHdl_p         Pointer to timer handle.

\return Returns a tOplkError error code.

\ingroup module_hrestimer
*/
//------------------------------------------------------------------------------
tOplkError hrestimer_deleteTimer(tTimerHdl* pTimerHdl_p)
{
    tOplkError          ret = kErrorOk;
    UINT                index;
    tHresTimerInfo*     pTimerInfo;
    HANDLE              hTimer;

    DEBUG_LVL_TIMERH_TRACE("%s() Deleting timer: %lx\n", __func__, *pTimerHdl_p);

    if (pTimerHdl_p == NULL)
        return kErrorTimerInvalidHandle;

    if (*pTimerHdl_p == 0)
    {   // no timer created yet
        return ret;
    }
    else
    {
        index = (UINT)HDL_TO_IDX(*pTimerHdl_p);
        if (index >= TIMER_COUNT)
        {   // invalid handle
            return kErrorTimerInvalidHandle;
        }
        pTimerInfo = &hresTimerInstance_l.aTimerInfo[index];
        if (pTimerInfo->eventArg.timerHdl.handle != *pTimerHdl_p)
        {   // invalid handle
            return ret;
        }
    }

    pTimerInfo->pfnCallback = NULL;
    *pTimerHdl_p = 0;

    // Cancel timer
    hTimer = hresTimerInstance_l.aHandle[index + HRTIMER_HDL_TIMER0];
    CancelWaitableTimer(hTimer);
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Control external synchronization interrupt

This function enables/disables the external synchronization interrupt. If the
external synchronization interrupt is not supported, the call is ignored.

\param[in]      fEnable_p           Flag determines if sync should be enabled or disabled.

\ingroup module_hrestimer
*/
//------------------------------------------------------------------------------
void hrestimer_controlExtSyncIrq(BOOL fEnable_p)
{
    UNUSED_PARAMETER(fEnable_p);
}

//------------------------------------------------------------------------------
/**
\brief  Set external synchronization interrupt time

This function sets the time when the external synchronization interrupt shall
be triggered to synchronize the host processor. If the external synchronization
interrupt is not supported, the call is ignored.

\param[in]      time_p              Time when the sync shall be triggered

\ingroup module_hrestimer
*/
//------------------------------------------------------------------------------
void hrestimer_setExtSyncIrqTime(tTimestamp time_p)
{
    UNUSED_PARAMETER(time_p);
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief  Set maximum timer resolution

This function sets the system timer to the maximum possible resolution.

\return Returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError setMaxTimerResolution(void)
{
    tOplkError ret = kErrorOk;
    LONG       winRet = 0;
    ULONG      min = ~0UL;
    ULONG      max = ~0UL;
    ULONG      current = ~0UL;

    // Load ntdll.dll
    hInstLibNtDll_l = LoadLibrary("ntdll.dll");
    if (hInstLibNtDll_l == NULL)
    {
        DEBUG_LVL_ERROR_TRACE("LoadLibrary(ntdll.dll) failed (%d)\n", GetLastError());
        return kErrorNoResource;
    }

    // Load address of NtQueryTimerResolution() function
    NtQueryTimerResolution = (tNtQueryTimerResolution)GetProcAddress(hInstLibNtDll_l, "NtQueryTimerResolution");
    if (NtQueryTimerResolution == NULL)
    {
        DEBUG_LVL_ERROR_TRACE("GetProcAddress(NtQueryTimerResolution) failed (%d)\n", GetLastError());
        return kErrorNoResource;
    }

    // Load address of NtSetTimerResolution() function
    NtSetTimerResolution = (tNtSetTimerResolution)GetProcAddress(hInstLibNtDll_l, "NtSetTimerResolution");
    if (NtSetTimerResolution == NULL)
    {
        DEBUG_LVL_ERROR_TRACE("GetProcAddress(NtSetTimerResolution) failed (%d)\n", GetLastError());
        return kErrorNoResource;
    }

    // Query current timer resolution
    NtQueryTimerResolution(&min, &max, &current);
    DEBUG_LVL_ERROR_TRACE("TimerResolution Min = %lu, Max = %lu, Cur = %lu\n", min, max, current);

    // Set timer resolution to maximum
    winRet = NtSetTimerResolution(max, TRUE, &current);
    DEBUG_LVL_ERROR_TRACE("NtSetTimerResolution returned %ld, current resolution = %lu\n", winRet, current);

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Restore standard timer resolution

This function resets the system timer resolution to the standard value.
*/
//------------------------------------------------------------------------------
static void restoreTimerResolution(void)
{
    LONG  winRet = 0;
    ULONG current = ~0UL;

    // Reset timer resolution to old value
    winRet = NtSetTimerResolution(0, FALSE, &current);
    DEBUG_LVL_TIMERH_TRACE("NtSetTimerResolution returned %ld, current resolution = %lu\n", winRet, current);

    // Free library NTDLL.DLL
    FreeLibrary(hInstLibNtDll_l);
    hInstLibNtDll_l = NULL;
}

//------------------------------------------------------------------------------
/**
\brief    Call a timer callback function

The function implements the timer callback function. It is called when a timer
expires.

\param[in]      index_p             Index of timer (0 or 1)
*/
//------------------------------------------------------------------------------
static void callTimerCb(UINT index_p)
{
    tHresTimerInfo* pTimerInfo;

    // Get the timer info according to the index
    pTimerInfo = &hresTimerInstance_l.aTimerInfo[index_p];

    // Check if the timer is a periodic timer
    if (pTimerInfo->dueTime.QuadPart != 0)
    {
        HANDLE  hTimer;
        BOOL    fRet;

        // Set up the timer again
        hTimer = hresTimerInstance_l.aHandle[index_p + HRTIMER_HDL_TIMER0];
        fRet = SetWaitableTimer(hTimer, &pTimerInfo->dueTime, 0L, NULL, NULL, 0);
        if (!fRet)
        {
            DEBUG_LVL_ERROR_TRACE("SetWaitableTimer failed (%d)\n", GetLastError());
            return;
        }
    }

    // If a callback function is given, call it
    if (pTimerInfo->pfnCallback != NULL)
    {
        pTimerInfo->pfnCallback(&pTimerInfo->eventArg);
    }
}

//------------------------------------------------------------------------------
/**
\brief  hrestimer worker thread

This function implements the hrestimer worker thread. It is responsible to handle
timer events.

\param[in,out]  pArgument_p         Thread argument (unused!)

\return The function returns a thread return code
*/
//------------------------------------------------------------------------------
static DWORD WINAPI timerThread(LPVOID pArgument_p)
{
    UINT32               waitRet;

    UNUSED_PARAMETER(pArgument_p);

    // Increase thread priority
    SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_TIME_CRITICAL);

    // Loop forever until thread is canceled
    while (TRUE)
    {
        // Wait for events
        waitRet = WaitForMultipleObjects(HRTIMER_HDL_COUNT, hresTimerInstance_l.aHandle, FALSE, INFINITE);
        switch (waitRet)
        {
            case WAIT_OBJECT_0 + HRTIMER_HDL_EVENT:
                // Shutdown was signaled
                return 0;

            case WAIT_OBJECT_0 + HRTIMER_HDL_TIMER0:
                // Timer 0 triggered
                callTimerCb(0);
                break;

            case WAIT_OBJECT_0 + HRTIMER_HDL_TIMER1:
                // Timer 1 triggered
                callTimerCb(1);
                break;

            case WAIT_FAILED:
            default:
                DEBUG_LVL_ERROR_TRACE("WaitForMultipleObjects failed (%d)\n", GetLastError());
                break;
        }
    }

    return 0;
}

/// \}
