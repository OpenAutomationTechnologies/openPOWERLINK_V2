/*******************************************************************************
  File:         hrtimerLib.c

  (c) Bernecker + Rainer Ges.m.b.H.,  B&R Strasse 1, 5142 Eggelsberg, Austria
      www.br-automation.com

  Project:      openPOWERLINK

  Description:

    hrtimerLib implements the user interface to the high-resolution timer
    library.

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
#include <stdlib.h>
#include <semLib.h>
#include <taskLib.h>
#include <sysLib.h>

#include "hrtimerLib.h"
#include "hrtimer.h"
#include "hpetTimer.h"

//============================================================================//
// Function declarations                                                      //
//============================================================================//

//============================================================================//
// Functions                                                                  //
//============================================================================//

//------------------------------------------------------------------------------
//
// Function: hrtimer_init
//
// Description:
//    Initialize the high-resolution timer library. It must be called before
//    any other call to the library.
//    It returns kHrtimerReturnOk if the timer library was successfully
//    initialized, otherwise an error code is returned.
//
// Parameters:
//    iCbPrio_p =  Priority of timer callback task
//
//    iCbStackSize_p =  Stack size of timer callback task
//
// Return:    int
//------------------------------------------------------------------------------
int hrtimer_init(int iCbPrio_p, int iCbStackSize_p)
{

    /* initialize timer data structure */
    memset (&timerData_g, 0x00, sizeof(timerData_g));

    /* create semaphore for callback signaling */
    if ((timerData_g.m_signalSem = semBCreate(SEM_Q_FIFO, SEM_EMPTY)) == NULL)
    {
        EPL_DBGLVL_ERROR_TRACE1(
         "%s() Couldn't create binary semaphore for timer signalling!\n",
         __func__);
        return kHrtimerReturnError;
    }

    /* create task for callback signaling */
    timerData_g.m_fStopTimerTask = FALSE;
    if ((timerData_g.m_timerTask = taskSpawn("tHrtimerCb",
                                             iCbPrio_p,
                                             0,
                                             iCbStackSize_p,
                                             handleCallback,
                                             0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
                                == ERROR)
    {
        EPL_DBGLVL_ERROR_TRACE1 ("%s() Couldn't create timer task!\n",
                                 __func__);
        semDelete(timerData_g.m_signalSem);
        return kHrtimerReturnError;
    }

    /* initialize timer device */
    timerdev_init();

    /* install interrupt handler */
    timerdev_registerInterruptHandler(handleTimerEvent, 0);

    /* show that lib is initialized */
    timerData_g.m_fInitialized = TRUE;

#ifdef TIMER_DEV_HPET
    /*
     * We setup a first timer which ensures that timeout will be read
     * within the timer period to detect overruns and implement a
     * 64 bit timer.
     *
     * This is specific for the HPET timer implementation and is
     * therefore surrounded by a preprocessor macro!
     */
    {
    timer_t             ovTimer;
    tHrtimerSig         ovSignalling;
    struct itimerspec   ovPeriod;

    ovSignalling.sigType = kHrtimerSigIntCb;
    ovSignalling.sigParam.m_signalCallback.m_pfnCallback =
                                       (tHrTimerCbFuncPtr)timerdev_readClock;
    ovSignalling.sigParam.m_signalCallback.m_arg = NULL;

    /* Overrun period is 299.966 seconds for Intel Core2Duo.
     * We use 100 s for the timer. */
    ovPeriod.it_value.tv_sec = 100;
    ovPeriod.it_value.tv_nsec = 0;
    ovPeriod.it_interval.tv_sec = 100;
    ovPeriod.it_interval.tv_nsec = 0;

    hrtimer_create(0, &ovSignalling, &ovTimer);
    hrtimer_settime(ovTimer, 0, &ovPeriod, NULL);
    }
#endif

    return kHrtimerReturnOk;
}

//------------------------------------------------------------------------------
//
// Function: hrtimer_shutdown
//
// Description:
//    Shutdown high-resolution timers.
//
// Parameters:
//    none
//
// Return:    void
//------------------------------------------------------------------------------
void hrtimer_shutdown(void)
{
    tHrtimer*       pTimer;
    int             iLockKey;

    if (!timerData_g.m_fInitialized)
    {
        return;
    }

    iLockKey = intLock();

    /* stop, remove and free timers */
    while ((pTimer = timerData_g.m_pFirstTimer) != NULL)
    {
        /* disarm the timer if it was armed */
        disarmTimer(pTimer);

        /* remove timer from list */
        removeFromList(pTimer);

        /* free timer memory */
        free(pTimer);
    }

    /* disable timer device */
    timerdev_shutdown();

    /* stop timer task */
    timerData_g.m_fStopTimerTask = TRUE;
    semGive (timerData_g.m_signalSem);
    while (timerData_g.m_fStopTimerTask == TRUE)
    {
        taskDelay(sysClkRateGet() / 2);
    }

    timerData_g.m_fInitialized = FALSE;

    intUnlock(iLockKey);

    /* delete semaphores */
    semDelete(timerData_g.m_signalSem);
}

//------------------------------------------------------------------------------
//
// Function: hrtimer_create
//
// Description:
//    Creates a new timer instance.  timer_create() creates a new timer
//    instance and append it in the timers list.  It returns the ID of the
//    generated timer.
//
// Parameters:
//    clockid =  The clockid argument specifies the clock to be used for
//    measuring time. This argument is ignored at the moment!
//
//    sigevent =  Pointer to timer signaling structure
//
//    pTimerId_p =  Pointer to store timerID
//
// Return:    int
//------------------------------------------------------------------------------
int hrtimer_create(clockid_t clockid, tHrtimerSig* sigevent, timer_t *
                   pTimerId_p)
{
    tHrtimer        *pTimer;
    int             iLockKey;

    if (!timerData_g.m_fInitialized)
    {
        return kHrtimerReturnNotInit;
    }

    /* allocate memory for timer */
    if ((pTimer = malloc(sizeof(tHrtimer))) == NULL)
    {
        return kHrtimerReturnError;
    }
    memset (pTimer, 0x00, sizeof(tHrtimer));

    /* Store signalling parameters */
    pTimer->m_signalling = *sigevent;

    iLockKey = intLock();

    /* add timer to timer list */
    addToList(pTimer, &pTimer->m_iTimerId);

    *pTimerId_p = pTimer->m_iTimerId;

    intUnlock(iLockKey);

    return kHrtimerReturnOk;
}

//------------------------------------------------------------------------------
//
// Function: hrtimer_delete
//
// Description:
//    Deletes an existing timer. The timer to delete is specifed by \p timerid.
//
// Parameters:
//    timerid =  ID of timer to delete
//
// Return:    int
//------------------------------------------------------------------------------
int hrtimer_delete(timer_t timerid)
{
    tHrtimer*           pTimer = (tHrtimer*)timerid;
    int                 iLockKey;

    if (!timerData_g.m_fInitialized)
    {
        return kHrtimerReturnNotInit;
    }

    iLockKey = intLock();
    /* disarm timer if it was armed. If it was scheduled we have
     * to reschedule the next timer */
    if (disarmTimer(pTimer))
    {
        armTimerList();
    }

    /* remove timer from list */
    removeFromList(pTimer);

    intUnlock(iLockKey);

    /* free timer memory */
    free(pTimer);

    return kHrtimerReturnOk;
}

//------------------------------------------------------------------------------
//
// Function: hrtimer_clock_gettime
//
// Description:
//    Get the current clock used by the high-resolution timers. Returns
//    kHrtimerReturnOk if clock was successfully read.
//
// Parameters:
//    clk_id =  The clk_id argument is the identifier of the particular clock
//    on which to act. It is ignored at the moment!
//
//    tp =  tp points to a struct timespec where the function stores the read
//    clock value.
//
// Return:    int
//------------------------------------------------------------------------------
int hrtimer_clock_gettime(clockid_t clk_id, struct timespec* tp)
{
    unsigned long long ullNsec;

    if (!timerData_g.m_fInitialized)
    {
        return kHrtimerReturnNotInit;
    }

    timerdev_readClock(&ullNsec);
    NSEC_TO_TIMESPEC(ullNsec, tp);

    return kHrtimerReturnOk;
}

//------------------------------------------------------------------------------
//
// Function: hrtimer_clock_getres
//
// Description:
//    Get the clock resolution of the high-resolution timers
//
// Parameters:
//    clk_id =  Id of the clock to be used. Ignored at the moment!
//
//    res =  Pointer to store the clock resolution.
//
// Return:    int
//------------------------------------------------------------------------------
int hrtimer_clock_getres(clockid_t clk_id, struct timespec* res)
{
    if (!timerData_g.m_fInitialized)
    {
        return kHrtimerReturnNotInit;
    }

    // todo to be implemented

    return kHrtimerReturnOk;
}

//------------------------------------------------------------------------------
//
// Function: hrtimer_gettime
//
// Description:
//    Get the current timeout value of the timer
//
// Parameters:
//    timerid =  Specifiy the timer to get the time from
//
//    curr_value =  Pointer to store current timer value
//
// Return:    int
//------------------------------------------------------------------------------
int hrtimer_gettime(timer_t timerid, struct itimerspec* curr_value)
{
    tHrtimer *          pTimer;
    int                 iLockKey;

    if (!timerData_g.m_fInitialized)
    {
        return kHrtimerReturnNotInit;
    }
    pTimer = (tHrtimer *)timerid;

    iLockKey = intLock();
    *curr_value = pTimer->m_timeout;
    intUnlock(iLockKey);

    return kHrtimerReturnOk;
}

//------------------------------------------------------------------------------
//
// Function: hrtimer_nanosleep
//
// Description:
//    Sleep
//
// Parameters:
//    req = time in nanoseconds to sleep
//
// Return:    void
//------------------------------------------------------------------------------
void hrtimer_nanosleep(struct timespec* req)
{
    if (!timerData_g.m_fInitialized)
    {
        return;
    }

    //todo to be implemented
}

//------------------------------------------------------------------------------
//
// Function: hrtimer_settime
//
// Description:
//    Set the timeout value of a timer. If both subfields of new_value-
//    >it_value are zero the timer will be disarmd. If they are nonzero the
//    timer is armed and setup for the given time.
//
// Parameters:
//    timerid =  timerid specifies the id of the timer to set the time
//
//    flags =  Argument is ignored at the moment!
//
//    new_value =  Specifies the new timer value to set
//
//    old_value =  If not NULL, the old value of the timer will be stored at
//    this location
//
// Return:    int
//------------------------------------------------------------------------------
int hrtimer_settime(timer_t timerid, int flags, struct itimerspec* new_value,
                     struct itimerspec* old_value)
{
    tHrtimer            *pTimer;
    tHrtimer            *pInsertionPoint;
    unsigned long long  ullCurTimeNs, ullTimeoutNs, ullIntervalNs;
    int                 iLockKey;
    int                 iRet = kHrtimerReturnOk;

    if (!timerData_g.m_fInitialized)
    {
        return kHrtimerReturnNotInit;
    }

    pTimer = (tHrtimer *)timerid;

    iLockKey = intLock();

    /* disarm the timer if it was armed */
    disarmTimer(pTimer);

    /* return old timer value */
    if (old_value != NULL)
    {
        *old_value = pTimer->m_timeout;
    }

    /* set new timer value */
    pTimer->m_timeout = *new_value;

    /* check if timer should be armed by checking if the timeout is zero */
    if ((pTimer->m_timeout.it_value.tv_sec != 0) ||
        (pTimer->m_timeout.it_value.tv_nsec != 0))
    {
        ullTimeoutNs = TIMESPEC_TO_NSEC(pTimer->m_timeout.it_value);
        ullIntervalNs = TIMESPEC_TO_NSEC(pTimer->m_timeout.it_interval);
        timerdev_readClock(&ullCurTimeNs);

        /* check if absolute or relative time was specified */
        if ((flags & TIMER_ABSTIME) != 0)
        {
            /* calculate relative timeout to check range */
            ullTimeoutNs = ullTimeoutNs - ullCurTimeNs;
        }

        if ((ullTimeoutNs < TIMERDEV_MIN_TIMEOUT) ||
            (ullTimeoutNs > TIMERDEV_MAX_TIMEOUT) ||
            ((ullIntervalNs != 0) && (ullIntervalNs < TIMERDEV_MIN_TIMEOUT)) ||
            (ullIntervalNs > TIMERDEV_MAX_TIMEOUT))
        {
            intUnlock(iLockKey);
            return kHrtimerReturnInvalidTimeout;
        }

        /* if relative time is specified we have to calculate the absolute
         * timeout to store in the timer structure. */
        if ((flags & TIMER_ABSTIME) == 0)
        {
            ullTimeoutNs = ullTimeoutNs+ ullCurTimeNs;
            NSEC_TO_TIMESPEC(ullTimeoutNs, &pTimer->m_timeout.it_value);
        }

        pTimer->m_nextTimeout = pTimer->m_timeout.it_value;

        searchArmedPosition(&pTimer->m_nextTimeout, &pInsertionPoint);
        insertInArmedList(pInsertionPoint, pTimer);
        pTimer->m_state = kHrtimerStateActive;
        /* if it will be inserted at first point we have to change the state of
         * the previous armed timer. */
        if ((pInsertionPoint == NULL) && (pTimer->m_pNextArmed != NULL))
        {
            pTimer->m_pNextArmed->m_state = kHrtimerStateActive;
        }
    }
    else
    {
        pTimer->m_nextTimeout = pTimer->m_timeout.it_value;
    }

    /* reschedule armed list */
    iRet = armTimerList();

    intUnlock(iLockKey);

    return iRet;
}

//------------------------------------------------------------------------------
//
// Function: hrtimer_setCallback
//
// Description:
//    The function sets the callback function for a timer which uses the
//    signalling types kHrtimerCallback or kHrtimerIntCb.
//
// Parameters:
//    fpCallback_p =  Pointer to callback function
//
//    iArg_p =  Argument for callback function
//
// Return:    int
//------------------------------------------------------------------------------
int hrtimer_setCallback(timer_t timerid, tHrTimerCbFuncPtr fpCallback_p,
                        void * pArg_p)
{
    tHrtimer        *pTimer;
    int             iLockKey;
    int             iRet = kHrtimerReturnOk;

    if (!timerData_g.m_fInitialized)
    {
        return kHrtimerReturnNotInit;
    }

    pTimer = (tHrtimer *)timerid;

    iLockKey = intLock();

    if ((pTimer->m_signalling.sigType != kHrtimerSigCallback) &&
        (pTimer->m_signalling.sigType != kHrtimerSigIntCb))
    {
        iRet = kHrtimerReturnError;
    }
    else
    {
        pTimer->m_signalling.sigParam.m_signalCallback.m_pfnCallback =
                fpCallback_p;
        pTimer->m_signalling.sigParam.m_signalCallback.m_arg = pArg_p;
    }

    intUnlock(iLockKey);

    return iRet;
}

//------------------------------------------------------------------------------
//
// Function: hrtimer_compareTimespec
//
// Description:
//    The function compares two timespec values. It returns 0 if the values are
//    equal, a negative value if the first is smaller than the second and a
//    positive value if the first is bigger than the second.
//
// Parameters:
//    pValue1_p =  Pointer to first timespec value
//
//    pValue2_p =  Pointer to second timespec value
//
// Return:    int
//------------------------------------------------------------------------------
int hrtimer_compareTimespec(const struct timespec * pValue1_p, const struct
                            timespec * pValue2_p)
{
    long long  llDiff;

    if (pValue1_p->tv_sec < pValue2_p->tv_sec)
    {
        return -1;
    }

    if (pValue1_p->tv_sec > pValue2_p->tv_sec)
    {
        return 1;
    }

    llDiff = pValue1_p->tv_nsec - pValue2_p->tv_nsec;
    if (llDiff > 0)
    {
        llDiff = 1;
    }
    else
    {
        if (llDiff < 0)
        {
            llDiff = -1;
        }
    }
    return llDiff;
}

//------------------------------------------------------------------------------
//
// Function: hrtimer_setTimespec
//
// Description:
//    Setup a normalized timespec value from seconds and nanoseconds
//
// Parameters:
//    pTs_p =  Pointer to store normalized timespec value
//
//    sec_p =  Seconds
//
//    nsec_p =  Nanoseconds
//
// Return:    void
//------------------------------------------------------------------------------
void hrtimer_setTimespec(struct timespec * pTs_p, time_t sec_p, long long nsec_p)
{
    while (nsec_p >= NSEC_PER_SEC)
    {
        /*The following asm() prevents the compiler from
         * optimising this loop. */
        asm("" : "+rm"(nsec_p));
        nsec_p -= NSEC_PER_SEC;
        ++sec_p;
    }
    while (nsec_p < 0)
    {
        asm("" : "+rm"(nsec_p));
        nsec_p += NSEC_PER_SEC;
        --sec_p;
    }
    pTs_p->tv_sec = sec_p;
    pTs_p->tv_nsec = nsec_p;
}

//------------------------------------------------------------------------------
//
// Function: hrtimer_subTimespec
//
// Description:
//    Subtract two timespec values
//
// Parameters:
//    value1_p =  First value
//
//    value2_p =  Second value
//
// Return:    struct timespec
//------------------------------------------------------------------------------
struct timespec hrtimer_subTimespec(struct timespec value1_p,
                                    struct timespec value2_p)
{
    struct timespec     delta;
    hrtimer_setTimespec(&delta, value1_p.tv_sec - value2_p.tv_sec,
                value1_p.tv_nsec - value2_p.tv_nsec);
    return delta;
}

//------------------------------------------------------------------------------
//
// Function: hrtimer_addTimespec
//
// Description:
//    Add two timespec values
//
// Parameters:
//    value1_p =  First value
//
//    value2_p =  Second value
//
// Return:    struct timespec
//------------------------------------------------------------------------------
struct timespec hrtimer_addTimespec(struct timespec value1_p,
                                    struct timespec value2_p)
{
    struct timespec     sum;
    hrtimer_setTimespec(&sum, value1_p.tv_sec + value2_p.tv_sec,
                value1_p.tv_nsec + value2_p.tv_nsec);
    return sum;
}

#ifdef    INCLUDE_HRTIMER_SHOW
//------------------------------------------------------------------------------
//
// Function: hrtimer_showArmed
//
// Description:
//    Show information about armed timers.
//
// Parameters:
//    none
//
// Return:    void
//------------------------------------------------------------------------------
void hrtimer_showArmed(void)
{
    tHrtimer            *pTimer;
    struct timespec     curTime;
    int                 iLockKey;

    if (!timerData_g.m_fInitialized)
    {
        printf("High resolution timer are not initialized!\n");
        return;
    }

    printf ("--- Armed Timer Info ---\n");
    iLockKey = intLock();

    hrtimer_clock_gettime(0, &curTime);
    printf("Current time: %d:%d s:ns\n", curTime.tv_sec, curTime.tv_nsec);

    pTimer = timerData_g.m_pArmedTimers;
    while (pTimer != NULL)
    {
        printf ("----- Timer ID %08x -----\n", pTimer->m_iTimerId);
        printf ("Next timeout: %d:%d s:ns\n", pTimer->m_nextTimeout.tv_sec,
                pTimer->m_nextTimeout.tv_nsec);
        printf ("Timeout/Intervall: %d:%d s:ns - %d:%d s:ns\n",
                pTimer->m_timeout.it_value.tv_sec,
                pTimer->m_timeout.it_value.tv_nsec,
                pTimer->m_timeout.it_interval.tv_sec,
                pTimer->m_timeout.it_interval.tv_nsec);
        if (pTimer->m_signalling.sigType == kHrtimerSigCallback)
        {
            printf ("Signalling Type: Callback Function:%p, Argument:%08x\n",
                pTimer->m_signalling.sigParam.m_signalCallback.m_pfnCallback,
                pTimer->m_signalling.sigParam.m_signalCallback.m_arg);
        }
        if (pTimer->m_signalling.sigType == kHrtimerSigIntCb)
        {
            printf ("Signalling Type: Interrupt Callback Function:%p, Argument:%08x\n",
                pTimer->m_signalling.sigParam.m_signalCallback.m_pfnCallback,
                pTimer->m_signalling.sigParam.m_signalCallback.m_arg);
        }
        if (pTimer->m_signalling.sigType == kHrtimerSigSem)
        {
            printf ("Signalling Type: Semaphore: %08x\n",
                pTimer->m_signalling.sigParam.m_signalSem.m_semaphore);
        }
        if (pTimer->m_signalling.sigType == kHrtimerSigMsgQueue)
        {
            printf ("Signalling Type: Message Queue: %08x Data:%08x\n",
                pTimer->m_signalling.sigParam.m_signalMq.m_msgQueue,
                pTimer->m_signalling.sigParam.m_signalMq.m_sigData);
        }
        pTimer = pTimer->m_pNextArmed;
    }
    printf ("\n");

    intUnlock(iLockKey);

    return;
}

//------------------------------------------------------------------------------
//
// Function: hrtimer_show
//
// Description:
//    Show high resolution timer info
//
// Parameters:
//    none
//
// Return:    void
//------------------------------------------------------------------------------
void hrtimer_show(void)
{
    tHrtimer            *pTimer;
    struct timespec     curTime;
    int                 iLockKey;

    if (!timerData_g.m_fInitialized)
    {
        printf("High resolution timer are not initialized!\n");
        return;
    }

    printf ("--- High Resolution Timer Info ---\n");

    iLockKey = intLock();

    hrtimer_clock_gettime(0, &curTime);
    printf("Current time: %d:%d s:ns\n", curTime.tv_sec, curTime.tv_nsec);

    pTimer = timerData_g.m_pFirstTimer;
    while (pTimer != NULL)
    {
        printf ("----- Timer ID %08x -----\n", pTimer->m_iTimerId);
        printf ("Next timeout: %d:%d s:ns\n", pTimer->m_nextTimeout.tv_sec,
                pTimer->m_nextTimeout.tv_nsec);
        printf ("Timeout/Intervall: %d:%d s:ns - %d:%d s:ns\n",
                pTimer->m_timeout.it_value.tv_sec,
                pTimer->m_timeout.it_value.tv_nsec,
                pTimer->m_timeout.it_interval.tv_sec,
                pTimer->m_timeout.it_interval.tv_nsec);
        if (pTimer->m_signalling.sigType == kHrtimerSigCallback)
        {
            printf ("Signalling Type: Callback Function:%p, Argument:%08x\n",
                pTimer->m_signalling.sigParam.m_signalCallback.m_pfnCallback,
                pTimer->m_signalling.sigParam.m_signalCallback.m_arg);
        }
        if (pTimer->m_signalling.sigType == kHrtimerSigIntCb)
        {
            printf ("Signalling Type: Interrupt Callback Function:%p, Argument:%08x\n",
                pTimer->m_signalling.sigParam.m_signalCallback.m_pfnCallback,
                pTimer->m_signalling.sigParam.m_signalCallback.m_arg);
        }
        if (pTimer->m_signalling.sigType == kHrtimerSigSem)
        {
            printf ("Signalling Type: Semaphore: %08x\n",
                pTimer->m_signalling.sigParam.m_signalSem.m_semaphore);
        }
        if (pTimer->m_signalling.sigType == kHrtimerSigMsgQueue)
        {
            printf ("Signalling Type: Message Queue: %08x Data:%08x\n",
                pTimer->m_signalling.sigParam.m_signalMq.m_msgQueue,
                pTimer->m_signalling.sigParam.m_signalMq.m_sigData);
        }
        printf ("Timer State: %d\n", pTimer->m_state);

        pTimer = pTimer->m_pNext;
    }
    printf ("\n");

    intUnlock(iLockKey);

    return;
}
#endif /* INCLUDE_HRTIMER_SHOW */
