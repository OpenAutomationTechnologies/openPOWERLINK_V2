/**
********************************************************************************
\file   timer-generic.c

\brief  Implementation of user timer module using a generic timer list

This file contains an implementation of the user timer module which uses a
generic timer list. It is used for Windows and non OS targets.

\ingroup module_timeru
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2016, B&R Industrial Automation GmbH
Copyright (c) 2013, SYSTEC electronic GmbH
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
#include <user/timeru.h>
#include <user/eventu.h>
#include <common/target.h>

#include <limits.h>

//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// module global vars
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// global function prototypes
//------------------------------------------------------------------------------

//============================================================================//
//            P R I V A T E   D E F I N I T I O N S                           //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

#define TIMERU_TIMER_LIST       0
#define TIMERU_FREE_LIST        1

#if ((TARGET_SYSTEM == _WIN32_) || (TARGET_SYSTEM == _WINCE_))
#define TIMERU_EVENT_SHUTDOWN   0   // smaller index has higher priority
#define TIMERU_EVENT_WAKEUP     1
#endif

//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------
typedef struct _tTimerEntry
{
    struct _tTimerEntry*    pNext;            // this must be the first element
    DWORD                   timeoutInMs;      // timeout in [ms]
    tTimerArg               timerArg;
} tTimerEntry;

typedef struct
{
    tTimerEntry*            pEntries;         // pointer to array of all timer entries
    tTimerEntry*            pTimerListFirst;
    tTimerEntry*            pFreeListFirst;
    UINT32                  startTimeInMs;    // start time when the first timeout in list is based on

    UINT                    freeEntries;
    UINT                    minFreeEntries;   // minimum number of free entries
                                              // used to check if TIMERU_MAX_ENTRIES is large enough
#if ((TARGET_SYSTEM == _WIN32_) || (TARGET_SYSTEM == _WINCE_))
    CRITICAL_SECTION        aCriticalSections[2];
    HANDLE                  hProcessThread;
    HANDLE                  ahEvents[2];      // WakeUp and ShutDown event handles
#endif
} tTimeruInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tTimeruInstance timeruInstance_l;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static void         enterCriticalSection(int nType_p);
static void         leaveCriticalSection(int nType_p);

#if ((TARGET_SYSTEM == _WIN32_) || (TARGET_SYSTEM == _WINCE_))
static DWORD WINAPI processThread(LPVOID parameter_p);
#endif

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Initialize user timers

The function initializes the user timer module.

\return The function returns a tOplkError error code.

\ingroup module_timeru
*/
//------------------------------------------------------------------------------
tOplkError timeru_init(void)
{
    int nIdx;

    // reset instance structure
    OPLK_MEMSET(&timeruInstance_l, 0, sizeof(timeruInstance_l));

    timeruInstance_l.pEntries = OPLK_MALLOC(sizeof(tTimerEntry) * TIMERU_MAX_ENTRIES);
    if (timeruInstance_l.pEntries == NULL)
        return kErrorNoResource;

    timeruInstance_l.pTimerListFirst = NULL;

    // fill free timer list
    for (nIdx = 0; nIdx < TIMERU_MAX_ENTRIES-1; nIdx++)
        timeruInstance_l.pEntries[nIdx].pNext = &timeruInstance_l.pEntries[nIdx + 1];

    timeruInstance_l.pEntries[TIMERU_MAX_ENTRIES-1].pNext = NULL;

    timeruInstance_l.pFreeListFirst = timeruInstance_l.pEntries;
    timeruInstance_l.freeEntries = TIMERU_MAX_ENTRIES;
    timeruInstance_l.minFreeEntries = TIMERU_MAX_ENTRIES;

    // set start time to a value which is in any case less or equal than getTickCount()
    // -> the only solution = 0
    timeruInstance_l.startTimeInMs = 0;

#if ((TARGET_SYSTEM == _WIN32_) || (TARGET_SYSTEM == _WINCE_))
    InitializeCriticalSection(&timeruInstance_l.aCriticalSections[TIMERU_TIMER_LIST]);
    InitializeCriticalSection(&timeruInstance_l.aCriticalSections[TIMERU_FREE_LIST]);

    timeruInstance_l.ahEvents[TIMERU_EVENT_WAKEUP]   = CreateEvent(NULL, FALSE, FALSE, NULL);
    timeruInstance_l.ahEvents[TIMERU_EVENT_SHUTDOWN] = CreateEvent(NULL, FALSE, FALSE, NULL);
    if ((timeruInstance_l.ahEvents[TIMERU_EVENT_WAKEUP] == NULL) ||
        (timeruInstance_l.ahEvents[TIMERU_EVENT_SHUTDOWN] == NULL))
    {
        return kErrorTimerThreadError;
    }

    timeruInstance_l.hProcessThread = CreateThread(NULL, 0, processThread, NULL, 0, NULL);
    if (timeruInstance_l.hProcessThread == NULL)
        return kErrorTimerThreadError;
#endif

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Shutdown user timer

The function shuts down the user timer instance.

\return The function returns a tOplkError error code.

\ingroup module_timeru
*/
//------------------------------------------------------------------------------
tOplkError timeru_exit(void)
{
#if ((TARGET_SYSTEM == _WIN32_) || (TARGET_SYSTEM == _WINCE_))
    SetEvent(timeruInstance_l.ahEvents[TIMERU_EVENT_SHUTDOWN]);

    WaitForSingleObject(timeruInstance_l.hProcessThread, INFINITE);

    CloseHandle(timeruInstance_l.hProcessThread);
    CloseHandle(timeruInstance_l.ahEvents[TIMERU_EVENT_SHUTDOWN]);
    CloseHandle(timeruInstance_l.ahEvents[TIMERU_EVENT_WAKEUP]);

    DeleteCriticalSection(&timeruInstance_l.aCriticalSections[TIMERU_TIMER_LIST]);
    DeleteCriticalSection(&timeruInstance_l.aCriticalSections[TIMERU_FREE_LIST]);
#endif

    OPLK_FREE(timeruInstance_l.pEntries);

    timeruInstance_l.pEntries = NULL;
    timeruInstance_l.pFreeListFirst = NULL;
    timeruInstance_l.pTimerListFirst = NULL;
    timeruInstance_l.freeEntries = 0;

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  User timer process function

This function must be called repeatedly from within the application. It checks
whether a timer has expired.

\return The function returns a tOplkError error code.

\ingroup module_timeru
*/
//------------------------------------------------------------------------------
tOplkError timeru_process(void)
{
    tTimerEntry*        pTimerEntry;
    UINT32              timeoutInMs;
    tEvent              event;
    tTimerEventArg      timerEventArg;
    tOplkError          ret = kErrorOk;

    enterCriticalSection(TIMERU_TIMER_LIST);
    // calculate elapsed time since start time
    timeoutInMs = target_getTickCount() - timeruInstance_l.startTimeInMs;

    // observe first timer entry in timer list
    pTimerEntry = timeruInstance_l.pTimerListFirst;
    if (pTimerEntry != NULL)
    {
        if (timeoutInMs >= pTimerEntry->timeoutInMs)
        {   // timeout elapsed - remove entry from timer list
            timeruInstance_l.pTimerListFirst = pTimerEntry->pNext;
            // adjust start time
            timeruInstance_l.startTimeInMs += pTimerEntry->timeoutInMs;
        }
        else
            pTimerEntry = NULL;
    }
    leaveCriticalSection(TIMERU_TIMER_LIST);

    if (pTimerEntry != NULL)
    {
        // call event function
        timerEventArg.timerHdl.handle = (tTimerHdl)pTimerEntry;
        OPLK_MEMCPY(&timerEventArg.argument, &pTimerEntry->timerArg.argument, sizeof(timerEventArg.argument));

        event.eventSink = pTimerEntry->timerArg.eventSink;
        event.eventType = kEventTypeTimer;
        OPLK_MEMSET(&event.netTime, 0x00, sizeof(tNetTime));
        event.eventArg.pEventArg = &timerEventArg;
        event.eventArgSize = sizeof(timerEventArg);

        ret = eventu_postEvent(&event);
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Create and set a timer

This function creates a timer, sets up the timeout and saves the
corresponding timer handle.

\param[out]     pTimerHdl_p         Pointer to store the timer handle.
\param[in]      timeInMs_p          Timeout in milliseconds.
\param[in]      pArgument_p         Pointer to user definable argument for timer.

\return The function returns a tOplkError error code.

\ingroup module_timeru
*/
//------------------------------------------------------------------------------
tOplkError timeru_setTimer(tTimerHdl* pTimerHdl_p,
                           ULONG timeInMs_p,
                           const tTimerArg* pArgument_p)
{
    tTimerEntry*    pNewEntry;
    tTimerEntry**   ppEntry;

    // check pointer to handle
    if (pTimerHdl_p == NULL)
        return kErrorTimerInvalidHandle;

    // We can only handle timeouts less than the maximum 32 bit value minus
    // the timeru_process() frequency. On Windows the timeru_process() is called
    // by the timer thread which wakes up when the last timeout is elapsed.
    // Therefore we can only handle timeouts less than UINT_MAX / 2.
    if (timeInMs_p > (UINT_MAX >> 1))
        return kErrorTimerNoTimerCreated;

    // fetch entry from free timer list
    enterCriticalSection(TIMERU_FREE_LIST);
    pNewEntry = timeruInstance_l.pFreeListFirst;
    if (pNewEntry != NULL)
    {
        timeruInstance_l.pFreeListFirst = pNewEntry->pNext;
        timeruInstance_l.freeEntries--;
        if (timeruInstance_l.minFreeEntries > timeruInstance_l.freeEntries)
            timeruInstance_l.minFreeEntries = timeruInstance_l.freeEntries;
    }
    leaveCriticalSection(TIMERU_FREE_LIST);

    if (pNewEntry == NULL)
    {   // sorry, no free entry
        return kErrorTimerNoTimerCreated;
    }

    *pTimerHdl_p = (tTimerHdl)pNewEntry;
    OPLK_MEMCPY(&pNewEntry->timerArg, pArgument_p, sizeof(tTimerArg));

    // insert timer entry in timer list
    enterCriticalSection(TIMERU_TIMER_LIST);
    // calculate timeout based on start time
    pNewEntry->timeoutInMs = (target_getTickCount() - timeruInstance_l.startTimeInMs) + timeInMs_p;

    ppEntry = &timeruInstance_l.pTimerListFirst;
    while (*ppEntry != NULL)
    {
        if ((*ppEntry)->timeoutInMs > pNewEntry->timeoutInMs)
        {
            (*ppEntry)->timeoutInMs -= pNewEntry->timeoutInMs;
            break;
        }
        pNewEntry->timeoutInMs -= (*ppEntry)->timeoutInMs;
        ppEntry = &(*ppEntry)->pNext;
    }
    // insert before **ppEntry
    pNewEntry->pNext = *ppEntry;
    *ppEntry = pNewEntry;
    leaveCriticalSection(TIMERU_TIMER_LIST);

#if ((TARGET_SYSTEM == _WIN32_) || (TARGET_SYSTEM == _WINCE_))
    if (ppEntry == &timeruInstance_l.pTimerListFirst)
        SetEvent(timeruInstance_l.ahEvents[TIMERU_EVENT_WAKEUP]);
#endif

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Modifies an existing timer

This function modifies an existing timer. If the timer was not yet created
it creates the timer and stores the new timer handle at \p pTimerHdl_p.

\param[in,out]  pTimerHdl_p         Pointer to store the timer handle.
\param[in]      timeInMs_p          Timeout in milliseconds.
\param[in]      pArgument_p         Pointer to user definable argument for timer.

\return The function returns a tOplkError error code.

\ingroup module_timeru
*/
//------------------------------------------------------------------------------
tOplkError timeru_modifyTimer(tTimerHdl* pTimerHdl_p,
                              ULONG timeInMs_p,
                              const tTimerArg* pArgument_p)
{
    tOplkError  ret;

    ret = timeru_deleteTimer(pTimerHdl_p);
    if (ret != kErrorOk)
        return ret;

    ret = timeru_setTimer(pTimerHdl_p, timeInMs_p, pArgument_p);

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Delete a timer

This function deletes an existing timer.

\param[in,out]  pTimerHdl_p         Pointer to timer handle of timer to delete.

\return The function returns a tOplkError error code.
\retval kErrorTimerInvalidHandle    An invalid timer handle was specified.
\retval kErrorOk                    The timer is deleted.

\ingroup module_timeru
*/
//------------------------------------------------------------------------------
tOplkError timeru_deleteTimer(tTimerHdl* pTimerHdl_p)
{
    tTimerEntry*    pTimerEntry;
    tTimerEntry**   ppEntry;

    // check pointer to handle
    if (pTimerHdl_p == NULL)
        return kErrorTimerInvalidHandle;

    // check handle itself, i.e. was the handle initialized before
    if (*pTimerHdl_p == 0)
        return kErrorOk;

    pTimerEntry = (tTimerEntry*)*pTimerHdl_p;

    // remove timer entry from timer list
    enterCriticalSection(TIMERU_TIMER_LIST);
    ppEntry = &timeruInstance_l.pTimerListFirst;
    while (*ppEntry != NULL)
    {
        if (*ppEntry == pTimerEntry)
        {
            *ppEntry = pTimerEntry->pNext;
            if (*ppEntry != NULL)
                (*ppEntry)->timeoutInMs += pTimerEntry->timeoutInMs;
            break;
        }

        ppEntry = &(*ppEntry)->pNext;
    }
    leaveCriticalSection(TIMERU_TIMER_LIST);

    // insert in free list
    enterCriticalSection(TIMERU_FREE_LIST);
    pTimerEntry->pNext = timeruInstance_l.pFreeListFirst;
    timeruInstance_l.pFreeListFirst = pTimerEntry;
    timeruInstance_l.freeEntries++;
    leaveCriticalSection(TIMERU_FREE_LIST);

    // set handle invalid
    *pTimerHdl_p = 0;

    return kErrorOk;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief  Enter critical section

This function enters a critical section of the timer module.

\param[in]      nType_p             Type of critical section to enter.
*/
//------------------------------------------------------------------------------
static void enterCriticalSection(int nType_p)
{
#if (TARGET_SYSTEM == _NO_OS_)
    UNUSED_PARAMETER(nType_p);

    target_enableGlobalInterrupt(FALSE);
#elif ((TARGET_SYSTEM == _WIN32_) || (TARGET_SYSTEM == _WINCE_))
    EnterCriticalSection(&timeruInstance_l.aCriticalSections[nType_p]);
#endif
}

//------------------------------------------------------------------------------
/**
\brief  Leave critical section

This function leaves a critical section of the timer module.

\param[in]      nType_p             Type of critical section to leave.
*/
//------------------------------------------------------------------------------
static void leaveCriticalSection(int nType_p)
{
#if (TARGET_SYSTEM == _NO_OS_)
    UNUSED_PARAMETER(nType_p);

    target_enableGlobalInterrupt(TRUE);
#elif ((TARGET_SYSTEM == _WIN32_) || (TARGET_SYSTEM == _WINCE_))
    LeaveCriticalSection(&timeruInstance_l.aCriticalSections[nType_p]);
#endif
}

#if ((TARGET_SYSTEM == _WIN32_) || (TARGET_SYSTEM == _WINCE_))
//------------------------------------------------------------------------------
/**
\brief  Timer thread function

This function implements the thread which is handling the timer events on
Windows.

\param[in,out]  parameter_p         Thread function parameter (not used)

\return The function returns the thread error code.
*/
//------------------------------------------------------------------------------
static DWORD WINAPI processThread(LPVOID parameter_p)
{
    tTimerEntry*    pTimerEntry;
    UINT32          timeoutInMs;
    UINT32          waitResult;
    tOplkError      ret;

    UNUSED_PARAMETER(parameter_p);

    for (;;)
    {
        ret = timeru_process();
        if (ret != kErrorOk)
        {
            // Error
        }

        // calculate time until the next timer event
        enterCriticalSection(TIMERU_TIMER_LIST);
        pTimerEntry = timeruInstance_l.pTimerListFirst;
        if (pTimerEntry == NULL)
        {   // timer list is empty
            timeoutInMs = INFINITE;
        }
        else
        {
            timeoutInMs = target_getTickCount() - timeruInstance_l.startTimeInMs;
            if (timeoutInMs > pTimerEntry->timeoutInMs)
            {   // timeout elapsed
                timeoutInMs = 0;
            }
            else
            {   // adjust timeout with elapsed time since start time
                timeoutInMs = pTimerEntry->timeoutInMs - timeoutInMs;
            }
        }
        leaveCriticalSection(TIMERU_TIMER_LIST);

        waitResult = WaitForMultipleObjects(2, timeruInstance_l.ahEvents, FALSE, timeoutInMs);
        switch (waitResult)
        {
            case (WAIT_OBJECT_0 + TIMERU_EVENT_SHUTDOWN):
                goto Exit;
                break;

            case (WAIT_OBJECT_0 + TIMERU_EVENT_WAKEUP):
            case WAIT_TIMEOUT:
                break;

            default:
                // Error
                break;
        }
    }

Exit:
    return 0;
}

#endif

/// \}
