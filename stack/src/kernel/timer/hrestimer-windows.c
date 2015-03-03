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
Copyright (c) 2015, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
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

#define TIMER_COUNT                 2
#define TIMERHDL_MASK               0x0FFFFFFF
#define TIMERHDL_SHIFT              28

#define HRTIMER_HDL_EVENT     0
#define HRTIMER_HDL_TIMER0    1
#define HRTIMER_HDL_TIMER1    2
#define HRTIMER_HDL_COUNT     3

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
    LARGE_INTEGER       dueTime;            ///< Duetime for continuous timers, otherwise 0
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
    HINSTANCE           hInstLibNtDll;              ///< Instance handle of the loaded NT kernel DLL
} tHresTimerInstance;


// function types from NTDLL.DLL
typedef LONG (NTAPI* NTQUERYTIMERRESOLUTION)(OUT PULONG MinimumResolution,
                                             OUT PULONG MaximumResolution,
                                             OUT PULONG CurrentResolution);
typedef LONG (NTAPI* NTSETTIMERRESOLUTION)(IN ULONG DesiredResolution,
                                           IN BOOLEAN SetResolution,
                                           OUT PULONG CurrentResolution);

//------------------------------------------------------------------------------
// module local vars
//------------------------------------------------------------------------------
static tHresTimerInstance   hresTimerInstance_l;
// function pointers to NTDLL.DLL
NTQUERYTIMERRESOLUTION      NtQueryTimerResolution;
NTSETTIMERRESOLUTION        NtSetTimerResolution;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static void callTimerCb(UINT index_p);
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
    LONG            winRet = 0;
    ULONG           min = ~0UL;
    ULONG           max = ~0UL;
    ULONG           current = ~0UL;
    DWORD           threadId;

    OPLK_MEMSET(&hresTimerInstance_l, 0, sizeof(hresTimerInstance_l));

    // load NTDLL.DLL
    hresTimerInstance_l.hInstLibNtDll = LoadLibrary("ntdll.dll");
    if (hresTimerInstance_l.hInstLibNtDll == NULL)
    {
        DEBUG_LVL_ERROR_TRACE("LoadLibrary(ntdll.dll) failed (%d)\n", GetLastError());
        return kErrorNoResource;
    }

    // load proc address of NtQueryTimerResolution
    NtQueryTimerResolution = (NTQUERYTIMERRESOLUTION)GetProcAddress(hresTimerInstance_l.hInstLibNtDll,
                                                                    "NtQueryTimerResolution");
    if (NtQueryTimerResolution == NULL)
    {
        DEBUG_LVL_ERROR_TRACE("GetProcAddress(NtQueryTimerResolution) failed (%d)\n", GetLastError());
        return kErrorNoResource;
    }

    // load proc address of NtSetTimerResolution
    NtSetTimerResolution = (NTSETTIMERRESOLUTION)GetProcAddress(hresTimerInstance_l.hInstLibNtDll,
                                                                "NtSetTimerResolution");
    if (NtSetTimerResolution == NULL)
    {
        DEBUG_LVL_ERROR_TRACE("GetProcAddress(NtSetTimerResolution) failed (%d)\n", GetLastError());
        return kErrorNoResource;
    }

    // query actual timer resolution
    NtQueryTimerResolution(&min, &max, &current);
    DEBUG_LVL_ERROR_TRACE("TimerResolution Min = %lu, Max = %lu, Cur = %lu\n", min, max, current);

    // set timer resolution to maximum
    winRet = NtSetTimerResolution(max, TRUE, &current);
    DEBUG_LVL_ERROR_TRACE("NtSetTimerResolution returnd %ld, current resolution = %lu\n", winRet, current);


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

    // create event for signalling shutdown
    hresTimerInstance_l.aHandle[HRTIMER_HDL_EVENT] = CreateEvent(NULL, FALSE, FALSE, NULL);

    // Create the thread to begin execution on its own
    hresTimerInstance_l.threadHandle = CreateThread(NULL, 0, timerThread, &hresTimerInstance_l, 0, &threadId);
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
    LONG            winRet = 0;
    ULONG           current = ~0UL;

    // signal shutdown to the thread
    SetEvent(hresTimerInstance_l.aHandle[HRTIMER_HDL_EVENT]);
    WaitForSingleObject(hresTimerInstance_l.threadHandle, INFINITE);
    CloseHandle(hresTimerInstance_l.threadHandle);

    CloseHandle(hresTimerInstance_l.aHandle[HRTIMER_HDL_EVENT]);
    CloseHandle(hresTimerInstance_l.aHandle[HRTIMER_HDL_TIMER0]);
    CloseHandle(hresTimerInstance_l.aHandle[HRTIMER_HDL_TIMER1]);


    // set timer resolution to old value
    winRet = NtSetTimerResolution(0, FALSE, &current);
    DEBUG_LVL_TIMERH_TRACE("NtSetTimerResolution returnd %ld, current resolution = %lu\n", winRet, current);

    // free library NTDLL.DLL
    FreeLibrary(hresTimerInstance_l.hInstLibNtDll);
    hresTimerInstance_l.hInstLibNtDll = NULL;

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

\param  pTimerHdl_p     Pointer to timer handle.
\param  time_p          Relative timeout in [ns].
\param  pfnCallback_p   Callback function, which is called when timer expires.
                        (The function is called mutually exclusive with the Edrv
                        callback functions (Rx and Tx)).
\param  argument_p      User-specific argument.
\param  fContinue_p     If TRUE, the callback function will be called continuously.
                        Otherwise, it is a one-shot timer.

\return Returns a tOplkError error code.

\ingroup module_hrestimer
*/
//------------------------------------------------------------------------------
tOplkError hrestimer_modifyTimer(tTimerHdl* pTimerHdl_p, ULONGLONG time_p,
                                 tTimerkCallback pfnCallback_p, ULONG argument_p,
                                 BOOL fContinue_p)
{
    tOplkError                  ret = kErrorOk;
    BOOL                        fRet;
    UINT                        index;
    tHresTimerInfo*             pTimerInfo;
    HANDLE                      hTimer;
    LARGE_INTEGER               dueTime;

    if (pTimerHdl_p == NULL)
        return kErrorTimerInvalidHandle;

    if (*pTimerHdl_p == 0)
    {   // no timer created yet - search free timer info structure
        pTimerInfo = &hresTimerInstance_l.aTimerInfo[0];
        for (index = 0; index < TIMER_COUNT; index++, pTimerInfo++)
        {
            if (pTimerInfo->pfnCallback == NULL)
                break;      // free structure found
        }
        if (index >= TIMER_COUNT)
            return kErrorTimerNoTimerCreated;
    }
    else
    {
        index = (UINT)(*pTimerHdl_p >> TIMERHDL_SHIFT) - 1;
        if (index >= TIMER_COUNT)
        {   // invalid handle
            return kErrorTimerInvalidHandle;
        }
        pTimerInfo = &hresTimerInstance_l.aTimerInfo[index];
    }

    // increment timer handle (if timer expires right after this statement,
    // the user would detect an unknown timer handle and discard it)
    pTimerInfo->eventArg.timerHdl = ((pTimerInfo->eventArg.timerHdl + 1) & TIMERHDL_MASK) |
                                    ((index + 1) << TIMERHDL_SHIFT);

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

    *pTimerHdl_p = pTimerInfo->eventArg.timerHdl;

    // configure timer
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

The function deletes an created high-resolution timer. The timer is specified
by its timer handle. After deleting, the handle is reset to zero.

\param  pTimerHdl_p     Pointer to timer handle.

\return Returns a tOplkError error code.

\ingroup module_hrestimer
*/
//------------------------------------------------------------------------------
tOplkError hrestimer_deleteTimer(tTimerHdl* pTimerHdl_p)
{
    tOplkError                  ret = kErrorOk;
    UINT                        index;
    tHresTimerInfo*             pTimerInfo;
    HANDLE                      hTimer;

    if (pTimerHdl_p == NULL)
        return kErrorTimerInvalidHandle;

    if (*pTimerHdl_p == 0)
    {
        return ret;      // no timer created yet
    }
    else
    {
        index = (UINT)(*pTimerHdl_p >> TIMERHDL_SHIFT) - 1;
        if (index >= TIMER_COUNT)
        {
            return kErrorTimerInvalidHandle;
        }
        pTimerInfo = &hresTimerInstance_l.aTimerInfo[index];
        if (pTimerInfo->eventArg.timerHdl != *pTimerHdl_p)
        {   // invalid handle
            return ret;
        }
    }

    pTimerInfo->pfnCallback = NULL;

    *pTimerHdl_p = 0;

    // cancel timer
    hTimer = hresTimerInstance_l.aHandle[index + HRTIMER_HDL_TIMER0];
    CancelWaitableTimer(hTimer);
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief    Call a timer callback function

The function implements the timer callback function. It is called when a timer
expires.

\param  index_p     Index of timer (0 or 1)
*/
//------------------------------------------------------------------------------
static void callTimerCb(UINT index_p)
{
    tHresTimerInfo* pTimerInfo;

    if (index_p > TIMER_COUNT)
        return;     // invalid handle

    pTimerInfo = &hresTimerInstance_l.aTimerInfo[index_p];

    if (pTimerInfo->dueTime.QuadPart != 0)
    {   // periodic timer
        HANDLE  hTimer;
        BOOL    fRet;

        // configure timer
        hTimer = hresTimerInstance_l.aHandle[index_p + HRTIMER_HDL_TIMER0];

        fRet = SetWaitableTimer(hTimer, &pTimerInfo->dueTime, 0L, NULL, NULL, 0);
        if (!fRet)
        {
            DEBUG_LVL_ERROR_TRACE("SetWaitableTimer failed (%d)\n", GetLastError());
            return;
        }
    }

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

\param  pArgument_p    Thread argument

\return The function returns a thread return code
*/
//------------------------------------------------------------------------------
static DWORD WINAPI timerThread(LPVOID pArgument_p)
{
    tHresTimerInstance*  pInstance = (tHresTimerInstance*)pArgument_p;
    UINT32               waitRet;

    // increase priority
    SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_TIME_CRITICAL);

    for (;;)
    {
        // Wait for events
        waitRet = WaitForMultipleObjects(HRTIMER_HDL_COUNT, pInstance->aHandle, FALSE, INFINITE);
        switch (waitRet)
        {
            case WAIT_OBJECT_0 + HRTIMER_HDL_EVENT:
            {   // shutdown was signalled
                return 0;
            }

            case WAIT_OBJECT_0 + HRTIMER_HDL_TIMER0:
            {   // timer 0 triggered
                callTimerCb(0);
                break;
            }

            case WAIT_OBJECT_0 + HRTIMER_HDL_TIMER1:
            {   // timer 1 triggered
                callTimerCb(1);
                break;
            }

            default:
            case WAIT_FAILED:
            {
                DEBUG_LVL_ERROR_TRACE("WaitForMultipleObjects failed (%d)\n", GetLastError());
                break;
            }
        }
    }
    return 0;
}

/// \}
