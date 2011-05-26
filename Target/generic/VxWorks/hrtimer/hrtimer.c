/*******************************************************************************
  File:         hrtimer.c

  (c) Bernecker + Rainer Ges.m.b.H.,  B&R Strasse 1, 5142 Eggelsberg, Austria
      www.br-automation.com

  Project:      openPOWERLINK

  Description:

    hrtimer contains the implementation of the internal functions of the high-
    resolution timer library.

  License:

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    1. Redistributions of source code must retain the above copyright
       notice, this list of conditions and the following disclaimer.

    2. Redistributions in binary form must reproduce the above copyright
       notice, this list of conditions and the following disclaimer in the
       documentation and/or other materials provided with the distribution.

    3. Neither the name of Bernecker + Rainer Ges.m.b.H nor the names of its
       contributors may be used to endorse or promote products derived
       from this software without prior written permission. For written
       permission, please contact office@br-automation.com.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    POSSIBILITY OF SUCH DAMAGE.

    Severability Clause:

        If a provision of this License is or becomes illegal, invalid or
        unenforceable in any jurisdiction, that shall not affect:
        1. the validity or enforceability in that jurisdiction of any other
           provision of this License; or
        2. the validity or enforceability in other jurisdictions of that or
           any other provision of this License.

*******************************************************************************/

//============================================================================//
// Includes                                                                   //
//============================================================================//
#include "hrtimer.h"
#include "hpetTimer.h"

#include <semLib.h>
#include <taskLib.h>
#include <logLib.h>

//============================================================================//
// Global variables                                                           //
//============================================================================//
// Main data structure of hrtimer library
tHrtimerData timerData_g;

//============================================================================//
// Functions                                                                  //
//============================================================================//

//------------------------------------------------------------------------------
//
// Function: addToList
//
// Description:
//    Add a timer to the timer list.
//
// Parameters:
//    pTimer_p =  Pointer to the timer to be inserted
//
//    pTimerId_p =  Pointer to store the timer Id of the inserted timer
//
// Return:    int
//------------------------------------------------------------------------------
void addToList(tHrtimer* pTimer_p, timer_t* pTimerId_p)
{
    tHrtimer    *pTimer;

    /* get the pointer to the last timer */
    pTimer = timerData_g.m_pLastTimer;

    /* if it is the first timer adjust both timer pointers */
    if (pTimer == NULL)
    {
        timerData_g.m_pLastTimer = pTimer_p;
        timerData_g.m_pFirstTimer = pTimer_p;
        pTimer_p->m_pNext = NULL;
        pTimer_p->m_pPrev = NULL;
    }
    else
    {
        pTimer->m_pNext = pTimer_p;
        timerData_g.m_pLastTimer = pTimer_p;
        pTimer_p->m_pPrev = pTimer;
        pTimer_p->m_pNext = NULL;
    }
    *pTimerId_p = (timer_t)pTimer_p;

}

//------------------------------------------------------------------------------
//
// Function: removeFromList
//
// Description:
//    Removes a timer from the timer list.
//
// Parameters:
//    pTimer_p
//
// Return:    void
//------------------------------------------------------------------------------
void removeFromList(tHrtimer* pTimer_p)
{
    /* check if it is the last one */
    if (pTimer_p->m_pNext == NULL)
    {
        timerData_g.m_pLastTimer = pTimer_p->m_pPrev;
    }
    else
    {
        pTimer_p->m_pNext->m_pPrev = pTimer_p->m_pPrev;
    }

    /* check if it is the first one */
    if (pTimer_p->m_pPrev == NULL)
    {
        timerData_g.m_pFirstTimer = pTimer_p->m_pNext;
    }
    else
    {
        pTimer_p->m_pPrev->m_pNext = pTimer_p->m_pNext;
    }
}

//------------------------------------------------------------------------------
//
// Function: insertInArmedList
//
// Description:
//    Insert a new timer into the armed list.
//
// Parameters:
//    pInsertionPoint_p =  Insertion point for new timer
//
//    pTimer_p =  Pointer to the timer instance which will be inserted
//
// Return:    void
//------------------------------------------------------------------------------
void insertInArmedList(tHrtimer* pInsertionPoint_p, tHrtimer* pTimer_p)
{
    tHrtimer            *pNext;

    /* check if we have to insert at first place */
    if (pInsertionPoint_p == NULL)
    {
        if (timerData_g.m_pArmedTimers == NULL)
        {
            /* we are the only one */
            timerData_g.m_pArmedTimers = pTimer_p;
            pTimer_p->m_pPrevArmed = NULL;
            pTimer_p->m_pNextArmed = NULL;
        }
        else
        {
            pTimer_p->m_pNextArmed = timerData_g.m_pArmedTimers;
            pTimer_p->m_pNextArmed->m_pPrevArmed = pTimer_p;
            pTimer_p->m_pPrevArmed = NULL;
            timerData_g.m_pArmedTimers = pTimer_p;
        }
    }
    else
    {
        /* check if we are the last one */
        if (pInsertionPoint_p->m_pNextArmed == NULL)
        {
            pInsertionPoint_p->m_pNextArmed = pTimer_p;
            pTimer_p->m_pPrevArmed = pInsertionPoint_p;
            pTimer_p->m_pNextArmed = NULL;
        }
        else
        {
            pNext = pInsertionPoint_p->m_pNextArmed;
            pInsertionPoint_p->m_pNextArmed = pTimer_p;
            pTimer_p->m_pPrevArmed = pInsertionPoint_p;
            pTimer_p->m_pNextArmed = pNext;
            pNext->m_pPrevArmed = pTimer_p;
        }
    }
}

//------------------------------------------------------------------------------
//
// Function: removeFromArmedList
//
// Description:
//    Remove a timer from the armed list. The timer to be removed is specified
//    by pTimer.
//
// Parameters:
//    pTimer_p =  Pointer to the timer to be removed from the armed list
//
// Return:    void
//------------------------------------------------------------------------------
void removeFromArmedList(tHrtimer* pTimer_p)
{
    /* check if it is the last one */
    if (pTimer_p->m_pNextArmed != NULL)
    {
        pTimer_p->m_pNextArmed->m_pPrevArmed = pTimer_p->m_pPrevArmed;
    }

    /* check if it is the first one */
    if (pTimer_p->m_pPrevArmed != NULL)
    {
        pTimer_p->m_pPrevArmed->m_pNextArmed = pTimer_p->m_pNextArmed;
    }
    else
    {
        timerData_g.m_pArmedTimers = pTimer_p->m_pNextArmed;
    }
    pTimer_p->m_pNextArmed = NULL;
    pTimer_p->m_pPrevArmed = NULL;
}

//------------------------------------------------------------------------------
//
// Function: searchArmedPosition
//
// Description:
//    The function searches for the position to insert the timer in the armed
//    list.
//
// Parameters:
//    pTimeout =  Timeout to search for
//
//    pInsertionPoint_p =  Pointer of timer to insert
//
// Return:    void
//------------------------------------------------------------------------------
void searchArmedPosition(struct timespec * pTimeout, tHrtimer**
                         pInsertionPoint_p)
{
    tHrtimer    *pTimer;
    tHrtimer    *pPrevTimer;
    BOOL        fFound = FALSE;
    int         iResult;

    pTimer = timerData_g.m_pArmedTimers;

    if (pTimer == NULL)
    {
        *pInsertionPoint_p = NULL;
        return;
    }

    while (pTimer != NULL)
    {
        /* compare timeout value */
        iResult = hrtimer_compareTimespec(pTimeout, &pTimer->m_nextTimeout);
        if (iResult < 0)
        {
            /* our timeout is smaller, therefore insert it after previous */
            *pInsertionPoint_p = pTimer->m_pPrevArmed;
            fFound = TRUE;
            break;
        }
        pPrevTimer = pTimer;
        pTimer = pTimer->m_pNextArmed;
    }

    /* if no insertion point is found we have to append it at last position */
    if (!fFound)
    {
        *pInsertionPoint_p = pPrevTimer;
    }
}

//------------------------------------------------------------------------------
//
// Function: handleTimerEvent
//
// Description:
//    Handle a timer event
//
// Parameters:
//    none
//
// Return:    void
//------------------------------------------------------------------------------
void handleTimerEvent(void)
{
    tHrtimer*           pTimer;
    tHrtimer*           pInsertionPoint;
    long long           llDiffNs;
    unsigned long long  ullCurTimeNs, ullCompareTimeNs, ullTimeoutNs;

    pTimer = timerData_g.m_pArmedTimers;

    /* A timer expired, therefore it shouldn't be NULL! */
    if (pTimer == NULL)
    {
        return;
    }

    /* loop through armed list and signal all timers with equal timeout */
    do
    {
        /* remove timer from armed list */
        removeFromArmedList(pTimer);
        pTimer->m_state = kHrtimerStateInactive;

        /* if signalling type is semaphore we give the semaphore of the timer */
        if (pTimer->m_signalling.sigType == kHrtimerSigSem)
        {
            semGive (pTimer->m_signalling.sigParam.m_signalSem.m_semaphore);
        }

        /* if signalling type is callback we add the timer to the callback list */
        if (pTimer->m_signalling.sigType == kHrtimerSigCallback)
        {
            if (timerData_g.m_pFirstCallbackTimer == NULL)
            {
                timerData_g.m_pFirstCallbackTimer = pTimer;
                timerData_g.m_pLastCallbackTimer = pTimer;
                pTimer->m_pNextCallback = NULL;
            }
            else
            {
                timerData_g.m_pLastCallbackTimer->m_pNextCallback = pTimer;
                timerData_g.m_pLastCallbackTimer = pTimer;
                pTimer->m_pNextCallback = NULL;
            }
            semGive(timerData_g.m_signalSem);
        }

        /* if signalling type is message queue we add a message to the queue */
        if (pTimer->m_signalling.sigType == kHrtimerSigMsgQueue)
        {
            msgQSend(pTimer->m_signalling.sigParam.m_signalMq.m_msgQueue,
                     (char *)&pTimer->m_signalling.sigParam.m_signalMq.m_sigData,
                     sizeof(unsigned long),
                     NO_WAIT, MSG_PRI_NORMAL);
        }

        /* if signalling type is interrupt callback we directly call a callback
         * function now!
         * NOTE: Usage of functions in callback is limited as we are in
         * interrupt context. Timers are blocked while we are executing
         * the callback! */
        if (pTimer->m_signalling.sigType == kHrtimerSigIntCb)
        {
            pTimer->m_signalling.sigParam.m_signalCallback.m_pfnCallback(
                    pTimer->m_signalling.sigParam.m_signalCallback.m_arg);
        }

        /* check if it is a continuous timer */
        if ((pTimer->m_timeout.it_interval.tv_sec != 0) ||
            (pTimer->m_timeout.it_interval.tv_nsec != 0))
        {
            /* calculate next timeout and reinsert it in armed list */
            pTimer->m_nextTimeout = hrtimer_addTimespec (pTimer->m_nextTimeout,
                                    pTimer->m_timeout.it_interval);
            searchArmedPosition(&pTimer->m_nextTimeout, &pInsertionPoint);
            insertInArmedList(pInsertionPoint, pTimer);
            pTimer->m_state = kHrtimerStateActive;
        }

        /*  get pointer to next timer which will expire.
         *  As we removed the current timer, the armed list points
         *  to the next timer now. */
        pTimer = timerData_g.m_pArmedTimers;
        if (pTimer == NULL)
        {
            break;
        }

        /* If the time difference between two timers is very small we can
         * loose a timer event. Therfore we read the timer value and handle
         * all timers which have expired in the meantime or expire in the next
         * two microseconds.
         */
        ullTimeoutNs = pTimer->m_nextTimeout.tv_sec * NSEC_PER_SEC +
                    pTimer->m_nextTimeout.tv_nsec;
        timerdev_readClock(&ullCurTimeNs);
        ullCompareTimeNs = ullCurTimeNs + TIMERDEV_MIN_TIMEOUT;
        llDiffNs =  ullTimeoutNs - ullCompareTimeNs;
        if (llDiffNs > 0)
        {
            /* We finished handling timers, schedule first timer in armed list */
            timerdev_arm(ullTimeoutNs - ullCurTimeNs);
            pTimer->m_state = kHrtimerStateScheduled;
        }
    }while ((llDiffNs <= 0));
}

//------------------------------------------------------------------------------
//
// Function: handleCallback
//
// Description:
//    This function handles the timer callbacks. It will be executed in the
//    callback timer task. If a timer event is signaled the callback functions
//    of all timers in the callback list will be called.
//
// Parameters:
//    iArg_p =  Argument for task function
//
// Return:    int
//------------------------------------------------------------------------------
int handleCallback(int iArg_p)
{
    BOOL                fStop = FALSE;
    tHrTimerCbFuncPtr   pfnCbFunc;
    void  *             pCbArg;
    BOOL                fReady;
    int                 iLockKey;
    tHrtimer            *pTimer;

    while (!fStop)
    {
        /* wait for signaling semaphore */
        semTake(timerData_g.m_signalSem, WAIT_FOREVER);

        /* check flag if we have to handle an event or have to stop */
        if (timerData_g.m_fStopTimerTask == FALSE)
        {

            fReady = FALSE;
            /* check callback list for timer callbacks */
            do
            {
                iLockKey = intLock();

                if (timerData_g.m_pFirstCallbackTimer == NULL)
                {
                    fReady = TRUE;
                    intUnlock(iLockKey);
                }
                else
                {
                    pfnCbFunc = timerData_g.m_pFirstCallbackTimer->m_signalling.
                             sigParam.m_signalCallback.m_pfnCallback;
                    pCbArg = timerData_g.m_pFirstCallbackTimer->m_signalling.
                             sigParam.m_signalCallback.m_arg;

                    /* remove timer from callback list */
                    if (timerData_g.m_pFirstCallbackTimer ==
                                    timerData_g.m_pLastCallbackTimer)
                    {
                        timerData_g.m_pFirstCallbackTimer = NULL;
                        timerData_g.m_pLastCallbackTimer = NULL;
                    }
                    else
                    {
                        pTimer = timerData_g.m_pFirstCallbackTimer;
                        timerData_g.m_pFirstCallbackTimer = pTimer->m_pNextCallback;
                        pTimer->m_pNextCallback = NULL;
                    }

                    /* give semaphore before starting callback so that timers
                     * are not blocked while callback is executed. */
                    intUnlock(iLockKey);

                    /* now we can call the callback function */
                    if (pfnCbFunc != NULL)
                    {
                        pfnCbFunc(pCbArg);
                    }
                }
            }while (!fReady);
        }
        else
        {
            fStop = TRUE;
        }
    }

    /* Show that task exited */
    timerData_g.m_fStopTimerTask = FALSE;
    return 0;
}

//------------------------------------------------------------------------------
//
// Function: disarmTimer
//
// Description:
//    The function disarms a timer. If the timer was scheduled, the timer
//    device will be disarmed. Then the timer will be removed from the armed
//    list. If it was scheduled it returns TRUE otherwise it returns FALSE.
//
// Parameters:
//    pTimer_p =  Pointer to timer which should be disarmed.
//
// Return:    BOOL
//------------------------------------------------------------------------------
BOOL disarmTimer(tHrtimer * pTimer_p)
{
    tHrtimer    *pTimer, *pPrevTimer;
    BOOL    fWasScheduled = FALSE;

    if (pTimer_p == NULL)
    {
        return FALSE;
    }

    if (pTimer_p->m_state == kHrtimerStateScheduled)
    {
        timerdev_disarm();
        pTimer_p->m_state = kHrtimerStateActive;
        fWasScheduled = TRUE;
    }

    if (pTimer_p->m_state == kHrtimerStateActive)
    {
        /* remove timer from armed list */
        removeFromArmedList(pTimer_p);
        pTimer_p->m_state = kHrtimerStateInactive;
    }

    /* search if we are in callback list and remove */
    pPrevTimer = NULL;
    pTimer = timerData_g.m_pFirstCallbackTimer;
    while (pTimer != NULL)
    {
        if (pTimer == pTimer_p)
        {
            if (pPrevTimer != NULL)
            {
                pPrevTimer->m_pNextCallback = pTimer->m_pNextCallback;
            }
            else
            {
                timerData_g.m_pFirstCallbackTimer = pTimer->m_pNextCallback;
            }
            if (pTimer->m_pNextCallback == NULL)
            {
                timerData_g.m_pLastCallbackTimer = pPrevTimer;
            }
            break;
        }
        pPrevTimer = pTimer;
        pTimer = pTimer->m_pNextCallback;
    }

    return  fWasScheduled;
}

//------------------------------------------------------------------------------
//
// Function: armTimerList
//
// Description:
//    This functions arms the timer list. It arms the timer device with the
//    timeout of the first armed timer in the list.
//
// Parameters:
//
//
// Return:    void
//------------------------------------------------------------------------------
int armTimerList(void)
{
    int                    iRet = kHrtimerReturnOk;
    unsigned long long     ullCurTimeNs, ullTimeoutNs, ullDiffNs;

    /* check if there are timers in the armed list */
    if (timerData_g.m_pArmedTimers != NULL)
    {
        if (timerData_g.m_pArmedTimers->m_state != kHrtimerStateScheduled)
        {
            /* arm the first timer and update its state */
            ullTimeoutNs = timerData_g.m_pArmedTimers->m_nextTimeout.tv_sec * NSEC_PER_SEC
                    + timerData_g.m_pArmedTimers->m_nextTimeout.tv_nsec;
            timerdev_readClock(&ullCurTimeNs);
            ullDiffNs = ullTimeoutNs - ullCurTimeNs;
            if (ullDiffNs > TIMERDEV_MIN_TIMEOUT)
            {
                timerdev_arm(ullDiffNs);
                timerData_g.m_pArmedTimers->m_state = kHrtimerStateScheduled;
            }
            else
            {
                /* timeout too small to schedule timer, therefore handle
                 * timer imediately. As we are not calling handleTimerEvent()
                 * from interrupt context we have to ensure that no one else
                 * interrupts us while handling timers! Because armTimerList()
                 * will only be called whithin locked context, this is
                 * ensured.
                 */
                handleTimerEvent();
            }
        }
    }
    return iRet;
}

