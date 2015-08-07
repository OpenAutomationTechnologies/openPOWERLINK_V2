/**
********************************************************************************
\file   hrestimer-posix.c

\brief  High-resolution timer module for Linux using Posix timer functions

This module is the target specific implementation of the high-resolution
timer module for Linux userspace. It uses Posix timer functions for its
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
#include <kernel/hrestimer.h>

#include <time.h>
#include <unistd.h>
#include <pthread.h>
#include <signal.h>
#include <sys/syscall.h>

//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

#define TIMER_COUNT           2            ///< number of high-resolution timers
#define TIMER_MIN_VAL_SINGLE  20000        ///< minimum timer intervall for single timeouts
#define TIMER_MIN_VAL_CYCLE   100000       ///< minimum timer intervall for continuous timeouts

#define SIGHIGHRES           SIGRTMIN + 1

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
    tTimerEventArg      eventArg;       ///< Event argument
    tTimerkCallback     pfnCallback;    ///< Pointer to timer callback function
    timer_t             timer;          ///< timer_t struct of this timer
} tHresTimerInfo;

/**
\brief  High-resolution timer instance

The structure defines a high-resolution timer module instance.
*/
typedef struct
{
    tHresTimerInfo      aTimerInfo[TIMER_COUNT];    ///< Array with timer information for a set of timers
    pthread_t           threadId;                   ///< Timer thread Id
} tHresTimerInstance;

//------------------------------------------------------------------------------
// module local vars
//------------------------------------------------------------------------------
static tHresTimerInstance       hresTimerInstance_l;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static void* timerThread(void* pParm_p);

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
    tOplkError              ret = kErrorOk;
    UINT                    index;
    struct sched_param      schedParam;
    tHresTimerInfo*         pTimerInfo;
    struct sigevent         sev;

    OPLK_MEMSET(&hresTimerInstance_l, 0, sizeof(hresTimerInstance_l));

    /* Initialize timer threads for all usable timers. */
    for (index = 0; index < TIMER_COUNT; index++)
    {
        pTimerInfo = &hresTimerInstance_l.aTimerInfo[index];

        sev.sigev_notify = SIGEV_SIGNAL;
        sev.sigev_signo = SIGHIGHRES;
        sev.sigev_value.sival_ptr = pTimerInfo;

        if (timer_create(CLOCK_MONOTONIC, &sev, &pTimerInfo->timer) != 0)
        {
            return kErrorNoResource;
        }
    }

    if (pthread_create(&hresTimerInstance_l.threadId, NULL,
                       timerThread, NULL) != 0)
    {
        return kErrorNoResource;
    }

    schedParam.sched_priority = CONFIG_THREAD_PRIORITY_HIGH;
    if (pthread_setschedparam(hresTimerInstance_l.threadId, SCHED_FIFO, &schedParam) != 0)
    {
        DEBUG_LVL_ERROR_TRACE("%s() Couldn't set thread scheduling parameters!\n", __func__);
        pthread_cancel(hresTimerInstance_l.threadId);
        return kErrorNoResource;
    }

#if (defined(__GLIBC__) && __GLIBC__ >= 2 && __GLIBC_MINOR__ >= 12)
    pthread_setname_np(hresTimerInstance_l.threadId, "oplk-hrtimer");
#endif

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
        timer_delete(pTimerInfo->timer);
        pTimerInfo->eventArg.timerHdl = 0;
        pTimerInfo->pfnCallback = NULL;
    }

    /* send exit signal to thread */
    pthread_cancel(hresTimerInstance_l.threadId);
    /* wait until thread terminates */
    DEBUG_LVL_TIMERH_TRACE("%s() Waiting for thread to exit...\n", __func__);

    pthread_join(hresTimerInstance_l.threadId, NULL);
    DEBUG_LVL_TIMERH_TRACE("%s() Thread exited!\n", __func__);

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
    struct itimerspec       RelTime;

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
            if (pTimerInfo->eventArg.timerHdl == 0)
            {   // free structure found
                break;
            }
        }
        if (index >= TIMER_COUNT)
        {   // no free structure found
            DEBUG_LVL_ERROR_TRACE("%s() Invalid timer index:%d\n", __func__, index);
            return kErrorTimerNoTimerCreated;
        }
        pTimerInfo->eventArg.timerHdl = HDL_INIT(index);
    }
    else
    {
        index = HDL_TO_IDX(*pTimerHdl_p);
        if (index >= TIMER_COUNT)
        {   // invalid handle
            DEBUG_LVL_ERROR_TRACE("%s() Invalid timer index:%d\n", __func__, index);
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

    if (time_p >= 1000000000L)
    {
        RelTime.it_value.tv_sec = (time_p / 1000000000L);
        RelTime.it_value.tv_nsec = (time_p % 1000000000);
    }
    else
    {
        RelTime.it_value.tv_sec = 0;
        RelTime.it_value.tv_nsec = time_p;
    }

    if (fContinue_p)
    {
        RelTime.it_interval.tv_nsec = RelTime.it_value.tv_nsec;
        RelTime.it_interval.tv_sec = RelTime.it_value.tv_sec;
    }
    else
    {
        RelTime.it_interval.tv_nsec = 0;
        RelTime.it_interval.tv_sec = 0;
    }

    DEBUG_LVL_TIMERH_TRACE("%s() timer:%lx timeout=%ld:%ld\n", __func__,
                            pTimerInfo->eventArg.timerHdl,
                            RelTime.it_value.tv_sec, RelTime.it_value.tv_nsec);

    timer_settime(pTimerInfo->timer, 0, &RelTime, NULL);

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
    struct itimerspec           relTime;

    DEBUG_LVL_TIMERH_TRACE("%s() Deleting timer:%lx\n", __func__, *pTimerHdl_p);

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

    // values of 0 disarms the timer
    relTime.it_value.tv_sec = 0;
    relTime.it_value.tv_nsec = 0;
    timer_settime(pTimerInfo->timer, 0, &relTime, NULL);

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

The function provides the main function of the timer thread.

\param  pParm_p     Thread parameter (unused!)

\return Returns a void* as specified by the pthread interface but it is not used!
*/
//------------------------------------------------------------------------------
static void* timerThread(void* pParm_p)
{
    INT                                 iRet;
    tHresTimerInfo*                     pTimerInfo;
    sigset_t                            awaitedSignal;
    siginfo_t                           signalInfo;

    UNUSED_PARAMETER(pParm_p);

    DEBUG_LVL_TIMERH_TRACE("%s(): ThreadId:%ld\n", __func__, syscall(SYS_gettid));

    sigemptyset(&awaitedSignal);
    sigaddset(&awaitedSignal, SIGHIGHRES);
    pthread_sigmask(SIG_BLOCK, &awaitedSignal, NULL);

    /* loop forever until thread will be canceled */
    while (1)
    {
        if ((iRet = sigwaitinfo(&awaitedSignal, &signalInfo)) > 0)
        {
            pTimerInfo = (tHresTimerInfo*)signalInfo.si_value.sival_ptr;
            /* call callback function */
            if (pTimerInfo->pfnCallback != NULL)
            {
                pTimerInfo->pfnCallback(&pTimerInfo->eventArg);
            }
        }
    }

    DEBUG_LVL_TIMERH_TRACE("%s() Exiting!\n", __func__);
    return NULL;
}

/// \}
