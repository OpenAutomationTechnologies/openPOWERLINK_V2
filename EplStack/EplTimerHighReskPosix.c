/*******************************************************************************

  File:         TimerHighReskLinuxUser.c

  (c) Bernecker + Rainer Ges.m.b.H.,  B&R Strasse 1, 5142 Eggelsberg, Austria
      www.br-automation.com

  (c) SYSTEC electronic GmbH, D-07973 Greiz, August-Bebel-Str. 29
      www.systec-electronic.com

  Project:      openPOWERLINK

  Author:       Josef Baumgartner

  Description:  Linux user space implementation of high resolution timers
                using POSIX timer functions

  TimerHighReskLinuxUser.c contains the high-resolution timer implementation
  for Linux user space using the POSIX timer function and pthreads.

  NOTE:
  The high resolution timer uses the real time signal SIGRTMIN + 1. The
  signal must be unblocked by the main thread to be handled by the
  sigwaitinfo() call.!

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

//=========================================================================//
// Includes                                                                //
//=========================================================================//
#include "EplInc.h"
#include "kernel/EplTimerHighResk.h"
#include "Benchmark.h"

#include <time.h>
#include <unistd.h>
#include <pthread.h>
#include <signal.h>
#include <sys/syscall.h>

//=========================================================================//
// Definitions                                                             //
//=========================================================================//
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

//=========================================================================//
// Type definitions                                                        //
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
    tEplTimerkCallback  m_pfnCallback;  ///< pointer to timer callback function
    timer_t             m_timer;
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
    pthread_t                   m_thread;
} tEplTimerHighReskInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tEplTimerHighReskInstance    EplTimerHighReskInstance_l;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static void * EplTimerHighReskProcessThread(void *pArgument_p);

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
    struct sigevent              sev;

    Ret = kEplSuccessful;

    EPL_MEMSET(&EplTimerHighReskInstance_l, 0, sizeof (EplTimerHighReskInstance_l));

    /* Initialize timer threads for all usable timers. */
    for (uiIndex = 0; uiIndex < TIMER_COUNT; uiIndex++)
    {
        pTimerInfo = &EplTimerHighReskInstance_l.m_aTimerInfo[uiIndex];

        sev.sigev_notify = SIGEV_SIGNAL;
        sev.sigev_signo = SIGHIGHRES;
        sev.sigev_value.sival_ptr = pTimerInfo;

        if (timer_create(CLOCK_MONOTONIC, &sev, &pTimerInfo->m_timer) != 0)
        {
            Ret = kEplNoResource;
            goto Exit;
        }
    }

    if (pthread_create(&EplTimerHighReskInstance_l.m_thread, NULL,
                       EplTimerHighReskProcessThread, NULL) != 0)
    {
        Ret = kEplNoResource;
        goto Exit;
    }

    schedParam.__sched_priority = EPL_THREAD_PRIORITY_HIGH;
    if (pthread_setschedparam(EplTimerHighReskInstance_l.m_thread, SCHED_FIFO, &schedParam) != 0)
    {
        EPL_DBGLVL_ERROR_TRACE("%s() Couldn't set thread scheduling parameters!\n", __func__);
        Ret = kEplNoResource;
        pthread_cancel(EplTimerHighReskInstance_l.m_thread);
        goto Exit;
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
        timer_delete(pTimerInfo->m_timer);
        pTimerInfo->m_EventArg.m_TimerHdl = 0;
        pTimerInfo->m_pfnCallback = NULL;
    }

    /* send exit signal to thread */
    pthread_cancel(EplTimerHighReskInstance_l.m_thread);
    /* wait until thread terminates */
    EPL_DBGLVL_TIMERH_TRACE("%s() Waiting for thread to exit...\n", __func__);

    pthread_join(EplTimerHighReskInstance_l.m_thread, NULL);
    EPL_DBGLVL_TIMERH_TRACE("%s() Thread exited!\n", __func__);

    return Ret;
}

//---------------------------------------------------------------------------
// Function:    EplTimerHighReskModifyTimerNs()
//
// Description: modifies the timeout of the timer with the specified handle.
//              If the handle the pointer points to is zero, the timer must
//              be created first.
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
    struct itimerspec            RelTime;


    Ret = kEplSuccessful;

    // check pointer to handle
    if(pTimerHdl_p == NULL)
    {
        EPL_DBGLVL_ERROR_TRACE("%s() Invalid timer handle\n", __func__);
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
            EPL_DBGLVL_ERROR_TRACE("%s() Invalid timer index:%d\n", __func__, uiIndex);
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
            EPL_DBGLVL_ERROR_TRACE("%s() Invalid timer index:%d\n", __func__, uiIndex);
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

    if (ullTimeNs_p >= 1000000000L)
    {
        RelTime.it_value.tv_sec = (ullTimeNs_p / 1000000000L);
        RelTime.it_value.tv_nsec = (ullTimeNs_p % 1000000000) ;
    }
    else
    {
        RelTime.it_value.tv_sec = 0;
        RelTime.it_value.tv_nsec = ullTimeNs_p;
    }

    if (fContinuously_p)
    {
        RelTime.it_interval.tv_nsec = RelTime.it_value.tv_nsec;
        RelTime.it_interval.tv_sec = RelTime.it_value.tv_sec;
    }
    else
    {
        RelTime.it_interval.tv_nsec = 0;
        RelTime.it_interval.tv_sec = 0;
    }

    /*
    EPL_DBGLVL_TIMERH_TRACE("%s() timer:%lx timeout=%ld:%ld\n", __func__,
           pTimerInfo->m_EventArg.m_TimerHdl,
           RelTime.it_value.tv_sec, RelTime.it_value.tv_nsec);
           */

    timer_settime(pTimerInfo->m_timer, 0, &RelTime, NULL);

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
    UINT                        uiIndex;
    tEplTimerHighReskTimerInfo* pTimerInfo;
    struct itimerspec           RelTime;

    EPL_DBGLVL_TIMERH_TRACE("%s() Deleting timer:%lx\n", __func__, *pTimerHdl_p);

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

    // values of 0 disarms the timer
    RelTime.it_value.tv_sec = 0;
    RelTime.it_value.tv_nsec = 0;
    timer_settime(pTimerInfo->m_timer, 0, &RelTime, NULL);

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
//              EplTimerHighReskProcessThread() waits until the expiration
//              of a high resolution timer is signaled. It reads the pointer
//              to the timer information structure from the signaling info
//              and calls the corresponding callback function.
//
// Parameters:  pParm_p *       = thread parameter (unused!)
//
// Return:      void *          = return value is specified by the pthread
//                                interface but is not used!
//---------------------------------------------------------------------------
static void * EplTimerHighReskProcessThread(void *pParm_p __attribute((unused)))
{
    INT                                 iRet;
    tEplTimerHighReskTimerInfo          *pTimerInfo;
    sigset_t                            awaitedSignal;
    siginfo_t                           signalInfo;

    EPL_DBGLVL_TIMERH_TRACE("%s(): ThreadId:%ld\n", __func__, syscall(SYS_gettid));

    sigemptyset(&awaitedSignal);
    sigaddset(&awaitedSignal, SIGHIGHRES);
    pthread_sigmask(SIG_BLOCK, &awaitedSignal, NULL);

    /* loop forever until thread will be canceled */
    while (1)
    {
        if ((iRet = sigwaitinfo(&awaitedSignal, &signalInfo)) > 0)
        {
            pTimerInfo = (tEplTimerHighReskTimerInfo *)signalInfo.si_value.sival_ptr;
            /* call callback function */
            if (pTimerInfo->m_pfnCallback != NULL)
            {
                pTimerInfo->m_pfnCallback(&pTimerInfo->m_EventArg);
            }
        }
    }

    EPL_DBGLVL_TIMERH_TRACE("%s() Exiting!\n", __func__);

    return NULL;
}

