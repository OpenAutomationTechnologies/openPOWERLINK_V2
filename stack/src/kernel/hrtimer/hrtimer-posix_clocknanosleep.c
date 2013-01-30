/*******************************************************************************

  File:         EplTimerHighReskposixClockNanosleep.c

  (c) Bernecker + Rainer Industrie-Elektronik Ges.m.b.H.
      B&R Strasse 1, 5142 Eggelsberg, Austria
      www.br-automation.com

  (c) SYSTEC electronic GmbH, D-07973 Greiz, August-Bebel-Str. 29
      www.systec-electronic.com

  Project:      openPOWERLINK

  Author:       Josef Baumgartner

  Description:  Linux user space implementation of high resolution timers
                using clock_nanosleep

  TimerHighReskLinuxUser.c contains the high-resolution timer implementation
  for Linux user space using the clock_nanosleep function and pthreads.

  For each timer a own thread will be created. A timer will be started by
  initializing its timer structure and wake up the thread through a semaphore.
  The thread reads the timer information and sleeps by calling clock_nanosleep()
  until the timeout is reached. Then the registered callback function is
  called.

  License:

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    1. Redistributions of source code must retain the above copyright
       notice, this list of conditions and the following disclaimer.

    2. Redistributions in binary form must reproduce the above copyright
       notice, this list of conditions and the following disclaimer in the
       documentation and/or other materials provided with the distribution.

    3. Neither the name of copyright holders nor the names of its
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

//=========================================================================//
// Includes                                                                //
//=========================================================================//
#include "EplInc.h"
#include "kernel/EplTimerHighResk.h"
#include "Benchmark.h"

#include <signal.h>
#include <semaphore.h>
#include <time.h>
#include <unistd.h>
#include <sys/timerfd.h>
#include <pthread.h>
#include <sys/syscall.h>

//=========================================================================//
// Definitions                                                             //
//=========================================================================//
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

//=========================================================================//
// Type definitions                                                             //
//=========================================================================//

/*******************************************************************************
* \brief       high-resolution timer information structure
*
* tEplTimerHighReskTimerInfo defines the structure which contains all
* necessary information for a high-resolution timer.
*******************************************************************************/
typedef struct
{
    tEplTimerEventArg   m_EventArg;
    tEplTimerkCallback  m_pfnCallback;          ///< pointer to timer callback function
    struct timespec     m_startTime;            ///< timestamp of timer start
    ULONGLONG           m_ullTime;              ///< timer period in nanoseconds
    pthread_t           m_timerThread;          ///< handle of timer thread
    sem_t               m_syncSem;              ///< thread synchronisation semaphore
    BOOL                m_fTerminate;           ///< thread termination flag
    BOOL                m_fContinuously;        ///< flag determines if timer will be restarted continuously
#ifdef HIGH_RESK_TIMER_LATENCY_DEBUG
    /* additional variables for latency debugging */
    LONG                m_maxLatency;           ///< minimum measured timer latency
    LONG                m_minLatency;           ///< maximum measured timer latency
#endif
} tEplTimerHighReskTimerInfo;


/*******************************************************************************
* \brief       high-resolution timer instance
*
* tEplTimerHighReskInstance contains all data of a high-resolution timer
* instance.
*******************************************************************************/
typedef struct
{
    tEplTimerHighReskTimerInfo  m_aTimerInfo[TIMER_COUNT];
} tEplTimerHighReskInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tEplTimerHighReskInstance    EplTimerHighReskInstance_l;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static void * EplTimerHighReskProcessThread(void *pArgument_p);
static inline void timespec_add (struct timespec *time1_p,
                                 unsigned long long ullTime_p, struct timespec *result_p);

#ifdef HIGH_RESK_TIMER_LATENCY_DEBUG
static inline void timespec_sub (struct timespec *time1_p,
                                 struct timespec *time2_p, struct timespec *result_p);
#endif

//=========================================================================//
//                                                                         //
//          P U B L I C   F U N C T I O N S                                //
//                                                                         //
//=========================================================================//

//---------------------------------------------------------------------------
// Function:    EplTimerHighReskInit()
//
// Description: initializes the high resolution timer module.
//
// Parameters:  void
//
// Return:      tEplKernel      = error code
//---------------------------------------------------------------------------
tEplKernel PUBLIC EplTimerHighReskInit(void)
{
    tEplKernel  Ret;

    Ret = EplTimerHighReskAddInstance();

    return Ret;
}

//---------------------------------------------------------------------------
// Function:    EplTimerHighReskAddInstance()
//
// Description: initializes the high resolution timer module.
//
// Parameters:  void
//
// Return:      tEplKernel      = error code
//---------------------------------------------------------------------------
tEplKernel PUBLIC EplTimerHighReskAddInstance(void)
{
    tEplKernel                   Ret;
    UINT                         uiIndex;
    struct sched_param           schedParam;
    tEplTimerHighReskTimerInfo*  pTimerInfo;

    Ret = kEplSuccessful;

    EPL_MEMSET(&EplTimerHighReskInstance_l, 0, sizeof (EplTimerHighReskInstance_l));

    /* Initialize timer threads for all usable timers. */
    for (uiIndex = 0; uiIndex < TIMER_COUNT; uiIndex++)
    {
        pTimerInfo = &EplTimerHighReskInstance_l.m_aTimerInfo[uiIndex];
        pTimerInfo->m_fTerminate = FALSE;

#ifdef HIGH_RESK_TIMER_LATENCY_DEBUG
        pTimerInfo->m_maxLatency = 0;
        pTimerInfo->m_minLatency = 999999999;
#endif

        if (sem_init(&pTimerInfo->m_syncSem, 0, 0) != 0)
        {
            EPL_DBGLVL_ERROR_TRACE("%s() Couldn't init semaphore!\n", __func__);
            Ret = kEplNoResource;
            goto Exit;
        }

        if (pthread_create(&pTimerInfo->m_timerThread, NULL, EplTimerHighReskProcessThread,
                           (void *)uiIndex) != 0)
        {
            Ret = kEplNoResource;
            sem_destroy(&pTimerInfo->m_syncSem);
            goto Exit;
        }

        schedParam.__sched_priority = EPL_THREAD_PRIORITY_HIGH;
        if (pthread_setschedparam(pTimerInfo->m_timerThread, SCHED_FIFO, &schedParam) != 0)
        {
            EPL_DBGLVL_ERROR_TRACE("%s() Couldn't set thread scheduling parameters!\n", __func__);
            Ret = kEplNoResource;
            sem_destroy(&pTimerInfo->m_syncSem);
            pthread_cancel(pTimerInfo->m_timerThread);
            goto Exit;
        }
    }

Exit:
    return Ret;
}

//---------------------------------------------------------------------------
// Function:    EplTimerHighReskDelInstance()
//
// Description: shuts down the high resolution timer module.
//
// Parameters:  void
//
// Return:      tEplKernel      = error code
//---------------------------------------------------------------------------
tEplKernel PUBLIC EplTimerHighReskDelInstance(void)
{
    tEplTimerHighReskTimerInfo* pTimerInfo;
    tEplKernel                  Ret;
    UINT                        uiIndex;

    Ret = kEplSuccessful;

    for (uiIndex = 0; uiIndex < TIMER_COUNT; uiIndex++)
    {
        pTimerInfo = &EplTimerHighReskInstance_l.m_aTimerInfo[uiIndex];

        pTimerInfo->m_EventArg.m_TimerHdl = 0;

        /* send exit signal to thread */
        pTimerInfo->m_fContinuously = 0;
        pTimerInfo->m_fTerminate = TRUE;
        sem_post(&pTimerInfo->m_syncSem);

        /* wait until thread terminates */
        pthread_join(pTimerInfo->m_timerThread, NULL);

        /* clean up */
        pTimerInfo->m_pfnCallback = NULL;
        sem_destroy(&pTimerInfo->m_syncSem);
    }

    return Ret;
}

//---------------------------------------------------------------------------
// Function:    EplTimerHighReskModifyTimerNs()
//
// Description: modifies the timeout of the timer with the specified handle.
//              If the handle the pointer points to is zero, the timer must
//              be created first.
//              If it is not possible to stop the old timer,
//              this function always assures that the old timer does not
//              trigger the callback function with the same handle as the new
//              timer. That means the callback function must check the passed
//              handle with the one returned by this function. If these are
//              unequal, the call can be discarded.
//
// Parameters:  pTimerHdl_p     = pointer to timer handle
//              ullTimeNs_p     = relative timeout in [ns]
//              pfnCallback_p   = callback function, which is called mutual
//                                exclusive with the Edrv callback functions
//                                (Rx and Tx).
//              ulArgument_p    = user-specific argument
//              fContinuously_p = if TRUE, callback function will be called
//                                continuously;
//                                otherwise, it is a oneshot timer.
//
// Return:      tEplKernel      = error code
//---------------------------------------------------------------------------
tEplKernel PUBLIC EplTimerHighReskModifyTimerNs(tEplTimerHdl*     pTimerHdl_p,
                                    ULONGLONG           ullTimeNs_p,
                                    tEplTimerkCallback  pfnCallback_p,
                                    ULONG               ulArgument_p,
                                    BOOL                fContinuously_p)
{
    tEplKernel                   Ret;
    UINT                         uiIndex;
    tEplTimerHighReskTimerInfo*  pTimerInfo;

    /*
    EPL_DBGLVL_TIMERH_TRACE("%s() pTimerHdl_p=%08x/%08x\n",
      __func__, (unsigned int)pTimerHdl_p,(unsigned int)*pTimerHdl_p);
    */

    Ret = kEplSuccessful;

    // check pointer to handle
    if(pTimerHdl_p == NULL)
    {
        Ret = kEplTimerInvalidHandle;
        goto Exit;
    }

    if (*pTimerHdl_p == 0)
    {   // no timer created yet
        // search free timer info structure
        pTimerInfo = &EplTimerHighReskInstance_l.m_aTimerInfo[0];
        for (uiIndex = 0; uiIndex < TIMER_COUNT; uiIndex++, pTimerInfo++)
        {
            if (pTimerInfo->m_EventArg.m_TimerHdl == 0)
            {   // free structure found
                break;
            }
        }
        if (uiIndex >= TIMER_COUNT)
        {   // no free structure found
            Ret = kEplTimerNoTimerCreated;
            goto Exit;
        }

        pTimerInfo->m_EventArg.m_TimerHdl = HDL_INIT(uiIndex);
    }
    else
    {
        uiIndex = HDL_TO_IDX(*pTimerHdl_p);
        if (uiIndex >= TIMER_COUNT)
        {   // invalid handle
            Ret = kEplTimerInvalidHandle;
            goto Exit;
        }

        pTimerInfo = &EplTimerHighReskInstance_l.m_aTimerInfo[uiIndex];
    }

    // increase too small time values
    if (fContinuously_p != FALSE)
    {
        if (ullTimeNs_p < TIMER_MIN_VAL_CYCLE)
        {
            ullTimeNs_p = TIMER_MIN_VAL_CYCLE;
        }
    }
    else
    {
        if (ullTimeNs_p < TIMER_MIN_VAL_SINGLE)
        {
            ullTimeNs_p = TIMER_MIN_VAL_SINGLE;
        }
    }

    /*
     * increment timer handle
     * (if timer expires right after this statement, the user
     * would detect an unknown timer handle and discard it)
     */
    pTimerInfo->m_EventArg.m_TimerHdl = HDL_INC(pTimerInfo->m_EventArg.m_TimerHdl);
    *pTimerHdl_p = pTimerInfo->m_EventArg.m_TimerHdl;

    /* initialize timer info */
    pTimerInfo->m_EventArg.m_Arg.m_dwVal = ulArgument_p;
    pTimerInfo->m_pfnCallback      = pfnCallback_p;
    pTimerInfo->m_fContinuously    = fContinuously_p;
    pTimerInfo->m_ullTime          = ullTimeNs_p;

    clock_gettime(CLOCK_MONOTONIC, &pTimerInfo->m_startTime);  // get current time

    /* signal timer start to thread */
    sem_post(&pTimerInfo->m_syncSem);

Exit:
    return Ret;
}

//---------------------------------------------------------------------------
// Function:    EplTimerHighReskDeleteTimer()
//
// Description: deletes the timer with the specified handle. Afterward the
//              handle is set to zero.
//
// Parameters:  pTimerHdl_p     = pointer to timer handle
//
// Return:      tEplKernel      = error code
//
// State:       not tested
//---------------------------------------------------------------------------
tEplKernel PUBLIC EplTimerHighReskDeleteTimer(tEplTimerHdl* pTimerHdl_p)
{
    tEplKernel                  Ret = kEplSuccessful;
    unsigned int                uiIndex;
    tEplTimerHighReskTimerInfo* pTimerInfo;

    // check pointer to handle
    if(pTimerHdl_p == NULL)
    {
        Ret = kEplTimerInvalidHandle;
        goto Exit;
    }

    if (*pTimerHdl_p == 0)
    {   // no timer created yet
        goto Exit;
    }
    else
    {
        uiIndex = HDL_TO_IDX(*pTimerHdl_p);
        if (uiIndex >= TIMER_COUNT)
        {   // invalid handle
            Ret = kEplTimerInvalidHandle;
            goto Exit;
        }
        pTimerInfo = &EplTimerHighReskInstance_l.m_aTimerInfo[uiIndex];
        if (pTimerInfo->m_EventArg.m_TimerHdl != *pTimerHdl_p)
        {   // invalid handle
            goto Exit;
        }
    }

    pTimerInfo->m_fContinuously = FALSE;
    *pTimerHdl_p = 0;
    pTimerInfo->m_EventArg.m_TimerHdl = 0;
    pTimerInfo->m_pfnCallback = NULL;

Exit:
    return Ret;
}

//=========================================================================//
//                                                                         //
//          P R I V A T E   F U N C T I O N S                              //
//                                                                         //
//=========================================================================//

//---------------------------------------------------------------------------
// Function:    EplTimerHighReskProcessThread()
//
// Description: Main function of the high-resolution timer thread.
//
//              EplTimerHighReskProcessThread() waits for a timer start
//              signal. When it is received it reads the high-resolution
//              timer information structure and calculates the timeout value.
//              It sleeps by calling clock_nanosleep() until the timeout is
//              reached. When the timeout is reached the callback function
//              registered in the timer info structure is called. If the
//              flag m_fContinuously is set the thread loops until the
//              timer is deleted.
//
// Parameters:  pArgument_p *   = pointer to timer info structure
//
// Return:      void *          = return value is specified by the pthread
//                                interface but is not used!
//---------------------------------------------------------------------------
static void * EplTimerHighReskProcessThread(void *pArgument_p)
{
    INT                                 iRet;
    tEplTimerHighReskTimerInfo          *pTimerInfo;
    struct timespec                     startTime, curTime, timeout;
    ULONGLONG                           ullPeriod;
    tEplTimerHdl                        timerHdl;
#ifdef HIGH_RESK_TIMER_LATENCY_DEBUG
    struct timespec                     debugtime;
#endif

    EPL_DBGLVL_TIMERH_TRACE("%s(): ThreadId:%ld\n", __func__, syscall(SYS_gettid));
    EPL_DBGLVL_TIMERH_TRACE("%s(): timer:%lx\n", __func__, (unsigned long)pArgument_p);

    /* thread parameter contains the address of the timer information structure */
    pTimerInfo = &EplTimerHighReskInstance_l.m_aTimerInfo[(int)pArgument_p];

    /* loop forever until thread will be canceled */
    while (1)
    {
        /* wait for semaphore which signals a timer start */
        sem_wait(&pTimerInfo->m_syncSem);

        /* check if thread should terminate */
        if (pTimerInfo->m_fTerminate)
        {
            EPL_DBGLVL_TIMERH_TRACE("%s() Exiting signal received!\n", __func__);
            break;
        }
        else
        {
            /* save timer information into local variables */
            startTime = pTimerInfo->m_startTime;
            timerHdl = pTimerInfo->m_EventArg.m_TimerHdl;
            ullPeriod = pTimerInfo->m_ullTime;

            /* calculate the timeout value for the timer cycle */
            timespec_add(&startTime, ullPeriod, &timeout);

#ifdef HIGH_RESK_TIMER_LATENCY_DEBUG
            clock_gettime(CLOCK_MONOTONIC, &curTime);
#endif
            do
            {
                /* sleep until timeout */
                iRet = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &timeout, NULL);
                if (iRet < 0)
                {
                    EPL_DBGLVL_ERROR_TRACE("%s(): Error in clock_nanosleep!\n",
                                            __func__);
                    /* todo how to signal that timeout wasn't correct? */
                }
                FTRACE_MARKER("HighReskTimer(%d) expired (%d ns)",
                              (int)pArgument_p, ullPeriod);

#ifdef HIGH_RESK_TIMER_LATENCY_DEBUG
                clock_gettime(CLOCK_MONOTONIC, &curTime);
                timespec_sub(&timeout, &curTime, &debugtime);
                FTRACE_MARKER("%s latency=%ld:%ld", __func__, debugtime.tv_sec,
                              debugtime.tv_nsec);
                if (debugtime.tv_nsec > pTimerInfo->m_maxLatency)
                {
                    EPL_DBGLVL_TIMERH_TRACE("%s() Timer elapsed: max latency=%ld ns\n",
                                             __func__, debugtime.tv_nsec);
                    pTimerInfo->m_maxLatency = debugtime.tv_nsec;
                }
                if (timeout.tv_nsec < pTimerInfo->m_minLatency)
                {
                    EPL_DBGLVL_TIMERH_TRACE("%s() Timer elapsed: min latency=%ld ns\n",
                                             __func__, debugtime.tv_nsec);
                    pTimerInfo->m_minLatency = debugtime.tv_nsec;
                }
#endif

                /* check if timer handle is valid */
                if (timerHdl == pTimerInfo->m_EventArg.m_TimerHdl)
                {
                    /* call callback function */
                    if (pTimerInfo->m_pfnCallback != NULL)
                    {
                        pTimerInfo->m_pfnCallback(&pTimerInfo->m_EventArg);
                    }
                }

                /* check if timer handle is still valid. Could be modified in callback! */
                if (timerHdl == pTimerInfo->m_EventArg.m_TimerHdl)
                {
                    if (pTimerInfo->m_fContinuously)
                    {
                        /* calculate timeout value for next timer cycle */
                        timespec_add(&timeout, ullPeriod, &timeout);
                    }
                }
            } while ((pTimerInfo->m_fContinuously) &&
                     (timerHdl == pTimerInfo->m_EventArg.m_TimerHdl));
        }
    }
    return NULL;
}


//---------------------------------------------------------------------------
// Function:    timespec_add()
//
// Description: add an offset to a timespec value
//
// Parameters:  time1_p *       = pointer to time where offset should be
//                                added to
//              ullTime_p       = time offset in nanoseconds
//              result_p *      = pointer to store result of time calculation
//
// Return:      void            = N/A
//---------------------------------------------------------------------------
static inline void timespec_add (struct timespec *time1_p, unsigned long long ullTime_p,
                             struct timespec *result_p)
{
    result_p->tv_sec = time1_p->tv_sec;
    if (ullTime_p >= 1000000000L)
    {
        result_p->tv_sec++;
        ullTime_p = ullTime_p % 1000000000L;
    }
    result_p->tv_nsec = time1_p->tv_nsec + ullTime_p;
    if (result_p->tv_nsec >= 1000000000L)
    {
        result_p->tv_sec++;
        result_p->tv_nsec = result_p->tv_nsec - 1000000000L;
    }
}

#ifdef HIGH_RESK_TIMER_LATENCY_DEBUG
//---------------------------------------------------------------------------
// Function:    timespec_sub()
//
// Description: subtrace too timespec values
//
// Parameters:  time1_p *       = pointer to first time value
//              time2_p *       = pointer to time value which will be substracted
//              result_p *      = pointer to store result of time calculation
//
// Return:      void            = N/A
//---------------------------------------------------------------------------
static inline void timespec_sub (struct timespec *time1_p, struct timespec *time2_p,
                                 struct timespec *result_p)
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
