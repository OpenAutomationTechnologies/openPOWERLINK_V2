/**
********************************************************************************
\file   hrestimer-posix_clocknanosleep.c

\brief  High-resolution timer module for Linux using Posix clock_nanosleep

This module is the target specific implementation of the high-resolution
timer module for Linux userspace. It uses Posix clock_nanosleep function for its
implementation.

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
#include <common/ftracedebug.h>
#include <kernel/hrestimer.h>

#include <signal.h>
#include <semaphore.h>
#include <time.h>
#include <unistd.h>
#include <sys/timerfd.h>
#include <pthread.h>
#include <sys/syscall.h>

//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#undef  HIGH_RESK_TIMER_LATENCY_DEBUG   ///< enable/disable latency debugging

#define TIMER_COUNT           2         ///< number of high-resolution timers
#define TIMER_MIN_VAL_SINGLE  20000     ///< minimum timer intervall for single timeouts
#define TIMER_MIN_VAL_CYCLE   100000    ///< minimum timer intervall for continuous timeouts

/* macros for timer handles */
#define TIMERHDL_MASK         0x0FFFFFFF
#define TIMERHDL_SHIFT        28
#define HDL_TO_IDX(Hdl)       ((Hdl >> TIMERHDL_SHIFT) - 1)
#define HDL_INIT(Idx)         ((Idx + 1) << TIMERHDL_SHIFT)
#define HDL_INC(Hdl)          (((Hdl + 1) & TIMERHDL_MASK) | (Hdl & ~TIMERHDL_MASK))

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
    struct timespec     startTime;          ///< Timestamp of timer start
    ULONGLONG           time;               ///< Timer period in nanoseconds
    pthread_t           timerThreadId;      ///< Handle of timer thread
    sem_t               syncSem;            ///< Thread synchronisation semaphore
    BOOL                fTerminate;         ///< Thread termination flag
    BOOL                fContinue;          ///< Flag determines if timer will be restarted continuously
#ifdef HIGH_RESK_TIMER_LATENCY_DEBUG
    /* additional variables for latency debugging */
    LONG                maxLatency;         ///< Minimum measured timer latency
    LONG                minLatency;         ///< Maximum measured timer latency
#endif
} tHresTimerInfo;

/**
\brief  High-resolution timer instance

The structure defines a high-resolution timer module instance.
*/
typedef struct
{
    tHresTimerInfo  aTimerInfo[TIMER_COUNT];    ///< Array with timer information for a set of timers
} tHresTimerInstance;

//------------------------------------------------------------------------------
// module local vars
//------------------------------------------------------------------------------
static tHresTimerInstance    hresTimerInstance_l;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static void* timerThread(void* pArgument_p);
static inline void timespec_add(struct timespec* time1_p, ULONGLONG time_p,
                                struct timespec* result_p);

#ifdef HIGH_RESK_TIMER_LATENCY_DEBUG
static inline void timespec_sub(struct timespec* time1_p, struct timespec* time2_p,
                                struct timespec* result_p);
#endif


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
    tOplkError                  ret = kErrorOk;
    UINT                        index;
    struct sched_param          schedParam;
    tHresTimerInfo*             pTimerInfo;

    OPLK_MEMSET(&hresTimerInstance_l, 0, sizeof(hresTimerInstance_l));

    /* Initialize timer threads for all usable timers. */
    for (index = 0; index < TIMER_COUNT; index++)
    {
        pTimerInfo = &hresTimerInstance_l.aTimerInfo[index];
        pTimerInfo->fTerminate = FALSE;

#ifdef HIGH_RESK_TIMER_LATENCY_DEBUG
        pTimerInfo->maxLatency = 0;
        pTimerInfo->minLatency = 999999999;
#endif

        if (sem_init(&pTimerInfo->syncSem, 0, 0) != 0)
        {
            DEBUG_LVL_ERROR_TRACE("%s() Couldn't init semaphore!\n", __func__);
            return kErrorNoResource;
        }

        if (pthread_create(&pTimerInfo->timerThreadId, NULL, timerThread, pTimerInfo) != 0)
        {
            sem_destroy(&pTimerInfo->syncSem);
            return kErrorNoResource;
        }

        schedParam.sched_priority = CONFIG_THREAD_PRIORITY_HIGH;
        if (pthread_setschedparam(pTimerInfo->timerThreadId, SCHED_FIFO, &schedParam) != 0)
        {
            DEBUG_LVL_ERROR_TRACE("%s() Couldn't set thread scheduling parameters!\n", __func__);
            sem_destroy(&pTimerInfo->syncSem);
            pthread_cancel(pTimerInfo->timerThreadId);
            return kErrorNoResource;
        }

#if (defined(__GLIBC__) && __GLIBC__ >= 2 && __GLIBC_MINOR__ >= 12)
        pthread_setname_np(pTimerInfo->timerThreadId, "oplk-hrtimer");
#endif
    }

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
    tHresTimerInfo*         pTimerInfo;
    tOplkError              ret = kErrorOk;
    UINT                    index;

    for (index = 0; index < TIMER_COUNT; index++)
    {
        pTimerInfo = &hresTimerInstance_l.aTimerInfo[index];

        pTimerInfo->eventArg.timerHdl = 0;

        /* send exit signal to thread */
        pTimerInfo->fContinue = 0;
        pTimerInfo->fTerminate = TRUE;
        sem_post(&pTimerInfo->syncSem);

        /* wait until thread terminates */
        pthread_join(pTimerInfo->timerThreadId, NULL);

        /* clean up */
        pTimerInfo->pfnCallback = NULL;
        sem_destroy(&pTimerInfo->syncSem);
    }

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
\param  argument_p      User-specific argument
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
    tOplkError              ret = kErrorOk;
    UINT                    index;
    tHresTimerInfo*         pTimerInfo;

    DEBUG_LVL_TIMERH_TRACE("%s() pTimerHdl_p=%08x/%08x\n",
                            __func__, (unsigned int)pTimerHdl_p, (unsigned int)*pTimerHdl_p);

    if (pTimerHdl_p == NULL)
        return kErrorTimerInvalidHandle;

    if (*pTimerHdl_p == 0)
    {   // no timer created yet
        // search free timer info structure
        pTimerInfo = &hresTimerInstance_l.aTimerInfo[0];
        for (index = 0; index < TIMER_COUNT; index++, pTimerInfo++)
        {
            if (pTimerInfo->eventArg.timerHdl == 0)
            {   // free structure found
                break;
            }
        }
        if (index >= TIMER_COUNT)
        {   // no free structure found
            return kErrorTimerNoTimerCreated;
        }

        pTimerInfo->eventArg.timerHdl = HDL_INIT(index);
    }
    else
    {
        index = HDL_TO_IDX(*pTimerHdl_p);
        if (index >= TIMER_COUNT)
        {   // invalid handle
            return kErrorTimerInvalidHandle;
        }

        pTimerInfo = &hresTimerInstance_l.aTimerInfo[index];
    }

    // increase too small time values
    if (fContinue_p != FALSE)
    {
        if (time_p < TIMER_MIN_VAL_CYCLE)
            time_p = TIMER_MIN_VAL_CYCLE;
    }
    else
    {
        if (time_p < TIMER_MIN_VAL_SINGLE)
            time_p = TIMER_MIN_VAL_SINGLE;
    }

    /* increment timer handle
     * (if timer expires right after this statement, the user
     * would detect an unknown timer handle and discard it) */
    pTimerInfo->eventArg.timerHdl = HDL_INC(pTimerInfo->eventArg.timerHdl);
    *pTimerHdl_p = pTimerInfo->eventArg.timerHdl;

    /* initialize timer info */
    pTimerInfo->eventArg.argument.value = argument_p;
    pTimerInfo->pfnCallback = pfnCallback_p;
    pTimerInfo->fContinue   = fContinue_p;
    pTimerInfo->time        = time_p;

    clock_gettime(CLOCK_MONOTONIC, &pTimerInfo->startTime);  // get current time
    sem_post(&pTimerInfo->syncSem); /* signal timer start to thread */

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
    tOplkError              ret = kErrorOk;
    UINT                    index;
    tHresTimerInfo*         pTimerInfo;

    // check pointer to handle
    if (pTimerHdl_p == NULL)
        return kErrorTimerInvalidHandle;

    if (*pTimerHdl_p == 0)
    {   // no timer created yet
        return ret;
    }
    else
    {
        index = HDL_TO_IDX(*pTimerHdl_p);
        if (index >= TIMER_COUNT)
        {   // invalid handle
            return kErrorTimerInvalidHandle;
        }
        pTimerInfo = &hresTimerInstance_l.aTimerInfo[index];
        if (pTimerInfo->eventArg.timerHdl != *pTimerHdl_p)
        {   // invalid handle
            return ret;
        }
    }

    pTimerInfo->fContinue = FALSE;
    *pTimerHdl_p = 0;
    pTimerInfo->eventArg.timerHdl = 0;
    pTimerInfo->pfnCallback = NULL;

    return ret;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief    Timer thread function

The function provides the main function of the timer thread. It waits for a
timer start signal. When it is received it reads the high-resolution timer
information structure and calculates the timeout value. It sleeps by calling
clock_nanosleep() until the timeout is reached. When the timeout is reached
the callback function registered in the timer info structure is called. If the
flag m_fContinue is set the thread loops until the timer is deleted.

\param  pArgument_p     Thread parameter. It contains the pointer to the timer
                        info structure.

\return Returns a void* as specified by the pthread interface but it is not used!
*/
//------------------------------------------------------------------------------
static void* timerThread(void* pArgument_p)
{
    INT                         iRet;
    tHresTimerInfo*             pTimerInfo;
    struct timespec             startTime, timeout;
    ULONGLONG                   period;
    tTimerHdl                   timerHdl;
#ifdef HIGH_RESK_TIMER_LATENCY_DEBUG
    struct timespec             debugtime, curTime;
#endif

    DEBUG_LVL_TIMERH_TRACE("%s(): ThreadId:%ld\n", __func__, syscall(SYS_gettid));
    DEBUG_LVL_TIMERH_TRACE("%s(): timer:%lx\n", __func__, (unsigned long)pArgument_p);

    /* thread parameter contains the address of the timer information structure */
    pTimerInfo = (tHresTimerInfo*)pArgument_p;

    /* loop forever until thread will be canceled */
    while (1)
    {
        /* wait for semaphore which signals a timer start */
        sem_wait(&pTimerInfo->syncSem);

        /* check if thread should terminate */
        if (pTimerInfo->fTerminate)
        {
            DEBUG_LVL_TIMERH_TRACE("%s() Exiting signal received!\n", __func__);
            break;
        }
        else
        {
            /* save timer information into local variables */
            startTime = pTimerInfo->startTime;
            timerHdl = pTimerInfo->eventArg.timerHdl;
            period = pTimerInfo->time;

            /* calculate the timeout value for the timer cycle */
            timespec_add(&startTime, period, &timeout);

#ifdef HIGH_RESK_TIMER_LATENCY_DEBUG
            clock_gettime(CLOCK_MONOTONIC, &curTime);
#endif
            do
            {
                /* sleep until timeout */
                iRet = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &timeout, NULL);
                if (iRet < 0)
                {
                    DEBUG_LVL_ERROR_TRACE("%s(): Error in clock_nanosleep!\n",
                                          __func__);
                    /* todo how to signal that timeout wasn't correct? */
                }
                FTRACE_MARKER("HighReskTimer(%d) expired (%d ns)",
                              (int)pArgument_p, period);

#ifdef HIGH_RESK_TIMER_LATENCY_DEBUG
                clock_gettime(CLOCK_MONOTONIC, &curTime);
                timespec_sub(&timeout, &curTime, &debugtime);
                FTRACE_MARKER("%s latency=%ld:%ld", __func__, debugtime.tv_sec,
                              debugtime.tv_nsec);
                if (debugtime.tv_nsec > pTimerInfo->maxLatency)
                {
                    DEBUG_LVL_TIMERH_TRACE("%s() Timer elapsed: max latency=%ld ns\n",
                                           __func__, debugtime.tv_nsec);
                    pTimerInfo->maxLatency = debugtime.tv_nsec;
                }
                if (timeout.tv_nsec < pTimerInfo->minLatency)
                {
                    DEBUG_LVL_TIMERH_TRACE("%s() Timer elapsed: min latency=%ld ns\n",
                                           __func__, debugtime.tv_nsec);
                    pTimerInfo->minLatency = debugtime.tv_nsec;
                }
#endif

                /* check if timer handle is valid */
                if (timerHdl == pTimerInfo->eventArg.timerHdl)
                {
                    /* call callback function */
                    if (pTimerInfo->pfnCallback != NULL)
                    {
                        pTimerInfo->pfnCallback(&pTimerInfo->eventArg);
                    }
                }

                /* check if timer handle is still valid. Could be modified in callback! */
                if (timerHdl == pTimerInfo->eventArg.timerHdl)
                {
                    if (pTimerInfo->fContinue)
                    {
                        /* calculate timeout value for next timer cycle */
                        timespec_add(&timeout, period, &timeout);
                    }
                }
            } while ((pTimerInfo->fContinue) &&
                     (timerHdl == pTimerInfo->eventArg.timerHdl));
        }
    }
    return NULL;
}

//------------------------------------------------------------------------------
/**
\brief    Add offset to timespec value

The function adds a time offset in nanoseconds to a timespec value.

\param  time1_p     Pointer to timespec to which the offset should be added.
\param  offset_p    Offset in nanoseconds to add.
\param  result_p    Pointer to store the result of the calculation.
*/
//------------------------------------------------------------------------------
static inline void timespec_add(struct timespec* time1_p, ULONGLONG offset_p,
                                struct timespec* result_p)
{
    result_p->tv_sec = time1_p->tv_sec;
    if (offset_p >= 1000000000L)
    {
        result_p->tv_sec++;
        offset_p = offset_p % 1000000000L;
    }
    result_p->tv_nsec = time1_p->tv_nsec + offset_p;
    if (result_p->tv_nsec >= 1000000000L)
    {
        result_p->tv_sec++;
        result_p->tv_nsec = result_p->tv_nsec - 1000000000L;
    }
}

#ifdef HIGH_RESK_TIMER_LATENCY_DEBUG
//------------------------------------------------------------------------------
/**
\brief    Subtract two timespec values

The function subtracts two timespec values.

\param  time1_p     Pointer to first timespec value from which to substract the
                    second one.
\param  time2_p     Pointer to second timespec value which will be substracted.
\param  result_p    Pointer to store the result of the calculation.
*/
//------------------------------------------------------------------------------
static inline void timespec_sub(struct timespec* time1_p, struct timespec* time2_p,
                                struct timespec* result_p)
{
    if (time2_p->tv_nsec > time1_p->tv_nsec)
    {
        result_p->tv_nsec = time2_p->tv_nsec - time1_p->tv_nsec;
        result_p->tv_sec = time2_p->tv_sec - time1_p->tv_sec;
    }
    else
    {
        result_p->tv_nsec = 1000000000L + time2_p->tv_nsec - time1_p->tv_nsec;
        result_p->tv_sec = time2_p->tv_sec - time1_p->tv_sec - 1;
    }
}
#endif

/// \}
