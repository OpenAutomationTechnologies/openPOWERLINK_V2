/****************************************************************************
  File:         EplTimeruLinuxUser.c

  (c) Bernecker + Rainer Ges.m.b.H.,  B&R Strasse 1, 5142 Eggelsberg, Austria
      www.br-automation.com

  (c) SYSTEC electronic GmbH, D-07973 Greiz, August-Bebel-Str. 29
      www.systec-electronic.com

  Project:      openPOWERLINK

  Description:  Linux Pthread based user space implementation of
                EPL user timer module

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
****************************************************************************/

#include "user/EplTimeru.h"

#include <stdio.h>
#include <unistd.h>
#include <sys/timerfd.h>
#include <pthread.h>
#include <sys/syscall.h>
#include <semaphore.h>

#include <signal.h>

/***************************************************************************/
/*                                                                         */
/*          G L O B A L   D E F I N I T I O N S                            */
/*                                                                         */
/***************************************************************************/
//---------------------------------------------------------------------------
// const defines
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
// local types
//---------------------------------------------------------------------------

typedef struct EplTimeruData tEplTimeruData;

struct EplTimeruData
{
    timer_t             m_Timer;
    tEplTimerArg        TimerArgument;
    tEplTimeruData      *m_pNextTimer;
    tEplTimeruData      *m_pPrevTimer;
};

typedef struct
{
    pthread_t           m_hProcessThread;
    pthread_mutex_t     m_Mutex;
    tEplTimeruData      *m_pFirstTimer;
    tEplTimeruData      *m_pLastTimer;
    tEplTimeruData      *m_pCurrentTimer;
} tEplTimeruInstance;

//---------------------------------------------------------------------------
// module global vars
//---------------------------------------------------------------------------
static tEplTimeruInstance EplTimeruInstance_g;

//---------------------------------------------------------------------------
// local function prototypes
//---------------------------------------------------------------------------
static void PUBLIC EplTimeruCbMs(ULONG ulParameter_p);
static void * EplTimeruProcessThread(void *pArgument_p);
static void EplTimeruLinuxUserAddTimer(tEplTimeruData *pData_p);
static void EplTimeruLinuxUserRemoveTimer(tEplTimeruData *pData_p);
static void EplTimeruResetTimerList(void);
static tEplTimeruData * EplTimeruGetNextTimer(void);

/***************************************************************************/
/*                                                                         */
/*     C L A S S  <Epl Userspace-Timermodule for Linux User Space>         */
/*                                                                         */
/***************************************************************************/
//
// Description: Epl Userspace-Timermodule for Linux User Space
//
/***************************************************************************/

//=========================================================================//
//                                                                         //
//          P U B L I C   F U N C T I O N S                                //
//                                                                         //
//=========================================================================//

//---------------------------------------------------------------------------
// Function:    EplTimeruInit
//
// Description: function inits first instance
//
// Parameters:  void
//
// Returns:     tEplKernel  = errorcode
//---------------------------------------------------------------------------
tEplKernel PUBLIC EplTimeruInit()
{
    tEplKernel  Ret;

    Ret = EplTimeruAddInstance();

    return Ret;
}

//---------------------------------------------------------------------------
// Function:    EplTimeruAddInstance
//
// Description: function inits additional instance
//
// Parameters:  void
//
// Returns:     tEplKernel  = errorcode
//---------------------------------------------------------------------------
tEplKernel PUBLIC EplTimeruAddInstance()
{
    tEplKernel                  Ret;
    struct sched_param          schedParam;
    INT                         iRetVal;

    // reset instance structure
    EplTimeruInstance_g.m_hProcessThread = 0;
    EplTimeruInstance_g.m_pFirstTimer = NULL;
    EplTimeruInstance_g.m_pLastTimer = NULL;

    if (pthread_mutex_init(&EplTimeruInstance_g.m_Mutex, NULL) != 0)
    {
        EPL_DBGLVL_ERROR_TRACE1("%s() couldn't init mutex!\n", __func__);
        Ret = kEplNoResource;
        goto Exit;
    }

    if ((iRetVal = pthread_create(&EplTimeruInstance_g.m_hProcessThread, NULL,
                       EplTimeruProcessThread,  &EplTimeruInstance_g)) != 0)
    {
        EPL_DBGLVL_ERROR_TRACE2("%s() couldn't create timer thread! (%d)\n",
                                __func__, iRetVal);
        Ret = kEplNoResource;
        pthread_mutex_destroy(&EplTimeruInstance_g.m_Mutex);
        goto Exit;
    }

    schedParam.__sched_priority = EPL_THREAD_PRIORITY_LOW;
    if (pthread_setschedparam(EplTimeruInstance_g.m_hProcessThread, SCHED_RR,
                              &schedParam) != 0)
    {
        EPL_DBGLVL_ERROR_TRACE1("%s() couldn't set thread scheduling parameters!\n",
                                __func__);
    }

    Ret = kEplSuccessful;

Exit:
    return Ret;
}

//---------------------------------------------------------------------------
// Function:    EplTimeruDelInstance
//
// Description: function deletes instance
//
// Parameters:  void
//
// Returns:     tEplKernel  = errorcode
//---------------------------------------------------------------------------
tEplKernel PUBLIC EplTimeruDelInstance(void)
{
    tEplKernel          Ret;
    tEplTimeruData*     pTimer;

    Ret = kEplSuccessful;

    /* cancel thread */
    pthread_cancel(EplTimeruInstance_g.m_hProcessThread);
    EPL_DBGLVL_TIMERU_TRACE1("%s() Waiting for thread to exit...\n", __func__);

    /* wait for thread to terminate */
    pthread_join(EplTimeruInstance_g.m_hProcessThread, NULL);
    EPL_DBGLVL_TIMERU_TRACE1("%s()Thread exited\n", __func__);

    /* free up timer list */
    EplTimeruResetTimerList();
    while ((pTimer = EplTimeruGetNextTimer()) != NULL)
    {
        EplTimeruLinuxUserRemoveTimer(pTimer);
        EPL_FREE(pTimer);
    }

    pthread_mutex_destroy(&EplTimeruInstance_g.m_Mutex);

    EplTimeruInstance_g.m_pFirstTimer = NULL;
    EplTimeruInstance_g.m_pLastTimer = NULL;

    return Ret;
}

//---------------------------------------------------------------------------
// Function:    EplTimeruProcess
//
// Description: This function is called repeatedly from within the main
//              loop of the application. It checks whether the first timer
//              entry has been elapsed.
//
//              We don't need it because we are using threads!
//
// Parameters:  none
//
// Returns:     tEplKernel  = errorcode
//---------------------------------------------------------------------------

tEplKernel PUBLIC EplTimeruProcess()
{
    return kEplSuccessful;
}

//---------------------------------------------------------------------------
// Function:    EplTimeruSetTimerMs
//
// Description: function creates a timer and returns the corresponding handle
//
// Parameters:  pTimerHdl_p = pointer to a buffer to fill in the handle
//              ulTime_p    = time for timer in ms
//              Argument_p  = argument for timer
//
// Returns:     tEplKernel  = errorcode
//---------------------------------------------------------------------------
tEplKernel PUBLIC EplTimeruSetTimerMs(tEplTimerHdl*     pTimerHdl_p,
                                      ULONG             ulTime_p,
                                      tEplTimerArg      Argument_p)
{
    tEplKernel          Ret = kEplSuccessful;
    tEplTimeruData*     pData;
    struct itimerspec   RelTime;
    struct itimerspec   CurTime;
    struct sigevent sev;

    // check pointer to handle
    if(pTimerHdl_p == NULL)
    {
        Ret = kEplTimerInvalidHandle;
        goto Exit;
    }

    pData = (tEplTimeruData*) EPL_MALLOC(sizeof (tEplTimeruData));
    if (pData == NULL)
    {
        Ret = kEplNoResource;
        goto Exit;
    }

    EPL_MEMCPY(&pData->TimerArgument, &Argument_p, sizeof(tEplTimerArg));

    EplTimeruLinuxUserAddTimer(pData);

    sev.sigev_notify = SIGEV_SIGNAL;
    sev.sigev_signo = SIGRTMIN;
    sev.sigev_value.sival_ptr = &pData->m_Timer;
    if (timer_create(CLOCK_MONOTONIC, &sev, &pData->m_Timer) == -1)
    {
        EPL_DBGLVL_ERROR_TRACE1("%s() Error creating timer!\n", __func__);
        EPL_FREE(pData);
        Ret = kEplNoResource;
        goto Exit;
    }

    if (ulTime_p >= 1000)
    {
        RelTime.it_value.tv_sec = (ulTime_p / 1000);
        RelTime.it_value.tv_nsec = (ulTime_p % 1000) * 1000000;
    }
    else
    {
        RelTime.it_value.tv_sec = 0;
        RelTime.it_value.tv_nsec = ulTime_p * 1000000;
    }

    /*
    EPL_DBGLVL_TIMERU_TRACE4("%s() Set timer: %p, ulTime_p=%ld\n",
                             __func__, (void *)pData, ulTime_p);
    */

    RelTime.it_interval.tv_sec = 0;
    RelTime.it_interval.tv_nsec = 0;

    if (timer_settime(pData->m_Timer, 0, &RelTime, &CurTime) < 0)
    {
        EPL_DBGLVL_ERROR_TRACE1("%s() Error timer_settime!\n", __func__);
        Ret = kEplTimerNoTimerCreated;
        goto Exit;
    }

    *pTimerHdl_p = (tEplTimerHdl) pData;

Exit:
    return Ret;
}

//---------------------------------------------------------------------------
// Function:    EplTimeruModifyTimerMs
//
// Description: function changes a timer and returns the corresponding handle
//
// Parameters:  pTimerHdl_p = pointer to a buffer to fill in the handle
//              ulTime_p    = time for timer in ms
//              Argument_p  = argument for timer
//
// Returns:     tEplKernel  = errorcode
//---------------------------------------------------------------------------
tEplKernel PUBLIC EplTimeruModifyTimerMs(tEplTimerHdl*     pTimerHdl_p,
                                        ULONG              ulTime_p,
                                        tEplTimerArg       Argument_p)
{
    tEplKernel          Ret = kEplSuccessful;
    tEplTimeruData*     pData;
    struct itimerspec   RelTime, CurTime;

    // check pointer to handle
    if(pTimerHdl_p == NULL)
    {
        Ret = kEplTimerInvalidHandle;
        goto Exit;
    }

    // check handle itself, i.e. was the handle initialized before
    if (*pTimerHdl_p == 0)
    {
        Ret = EplTimeruSetTimerMs(pTimerHdl_p, ulTime_p, Argument_p);
        goto Exit;
    }
    pData = (tEplTimeruData*) *pTimerHdl_p;

    if (ulTime_p >= 1000)
    {
        RelTime.it_value.tv_sec = (ulTime_p / 1000);
        RelTime.it_value.tv_nsec = (ulTime_p % 1000) * 1000000;
    }
    else
    {
        RelTime.it_value.tv_sec = 0;
        RelTime.it_value.tv_nsec = ulTime_p * 1000000;
    }

    /*
    EPL_DBGLVL_TIMERU_TRACE3("%s() Modify timer:%08x ulTime_p=%ld\n",
                             __func__, *pTimerHdl_p, ulTime_p);
    */

    RelTime.it_interval.tv_sec = 0;
    RelTime.it_interval.tv_nsec = 0;
    if (timer_settime(pData->m_Timer, 0, &RelTime, &CurTime) != 0)
    {
        EPL_DBGLVL_ERROR_TRACE1("%s() Error timer_settime!\n", __func__);
        Ret = kEplTimerNoTimerCreated;
        goto Exit;
    }

    // copy the TimerArg after the timer is restarted,
    // so that a timer occurred immediately before timer_settime
    // won't use the new TimerArg and
    // therefore the old timer cannot be distinguished from the new one.
    // But if the new timer is too fast, it may get lost.
    EPL_MEMCPY(&pData->TimerArgument, &Argument_p, sizeof(tEplTimerArg));

Exit:
    return Ret;
}

//---------------------------------------------------------------------------
// Function:    EplTimeruDeleteTimer
//
// Description: function deletes a timer
//
// Parameters:  pTimerHdl_p = pointer to a buffer to fill in the handle
//
// Returns:     tEplKernel  = errorcode
//---------------------------------------------------------------------------
tEplKernel PUBLIC EplTimeruDeleteTimer(tEplTimerHdl* pTimerHdl_p)
{
    tEplKernel          Ret = kEplSuccessful;
    tEplTimeruData*     pData;

    // check pointer to handle
    if(pTimerHdl_p == NULL)
    {
        Ret = kEplTimerInvalidHandle;
        goto Exit;
    }

    // check handle itself, i.e. was the handle initialized before
    if (*pTimerHdl_p == 0)
    {
        Ret = kEplSuccessful;
        goto Exit;
    }
    pData = (tEplTimeruData*) *pTimerHdl_p;

    timer_delete (pData->m_Timer);
    EplTimeruLinuxUserRemoveTimer(pData);
    EPL_FREE(pData);

    // uninitialize handle
    *pTimerHdl_p = 0;

Exit:
    return Ret;

}

//---------------------------------------------------------------------------
// Function:    EplTimeruIsTimerActive
//
// Description: checks if the timer referenced by the handle is currently
//              active.
//
// Parameters:  TimerHdl_p  = handle of the timer to check
//
// Returns:     BOOL        = TRUE, if active;
//                            FALSE, otherwise
//---------------------------------------------------------------------------
BOOL PUBLIC EplTimeruIsTimerActive(tEplTimerHdl TimerHdl_p)
{
    BOOL                fActive = TRUE;
    tEplTimeruData*     pData;
    struct itimerspec   remaining;

    // check handle itself, i.e. was the handle initialized before
    if (TimerHdl_p == 0)
    {   // timer was not created yet, so it is not active
        goto Exit;
    }
    pData = (tEplTimeruData*) TimerHdl_p;

    // check if timer is running
    timer_gettime(pData->m_Timer, &remaining);

    if ((remaining.it_value.tv_sec == 0) &&
        (remaining.it_value.tv_nsec == 0))
    {
        fActive = FALSE;
    }
    else
    {
        fActive = TRUE;
    }
Exit:
    return fActive;
}

//=========================================================================//
//                                                                         //
//          P R I V A T E   F U N C T I O N S                              //
//                                                                         //
//=========================================================================//

//---------------------------------------------------------------------------
// Function:    EplTimeruProcessThread()
//
// Description: Main function of the user timer thread.
//
//              EplTimeruProcessThread() waits until the expiration
//              of a user timer is signaled. It reads the pointer
//              to the timer information structure from the signaling info
//              and calls the corresponding callback function.
//
// Parameters:  pParm_p *       = thread parameter (unused!)
//
// Return:      void *          = return value is specified by the pthread
//                                interface but is not used!
//---------------------------------------------------------------------------
static void * EplTimeruProcessThread(void *pArgument_p __attribute((unused)))
{
    int             iRet;
    tEplTimeruData  *pTimer;
    sigset_t        awaitedSignal;
    siginfo_t       signalInfo;

    EPL_DBGLVL_TIMERU_TRACE2("%s() ThreadId:%d\n", __func__, syscall(SYS_gettid));

    sigemptyset(&awaitedSignal);
    sigaddset(&awaitedSignal, SIGRTMIN);
    pthread_sigmask(SIG_BLOCK, &awaitedSignal, NULL);

    /* loop forever until thread will be canceled */
    while (1)
    {
        if ((iRet = sigwaitinfo(&awaitedSignal, &signalInfo)) > 0)
        {
            pTimer = (tEplTimeruData *)signalInfo.si_value.sival_ptr;
            /* call callback function of timer */
            EplTimeruCbMs((unsigned long)pTimer);
        }
    }

    EPL_DBGLVL_TIMERU_TRACE1("%s() Exiting!\n", __func__);
    return NULL;
}

//---------------------------------------------------------------------------
// Function:    EplTimeruCbMs
//
// Description: function to process timer
//
//
//
// Parameters:  lpParameter = pointer to structur of type tEplTimeruData
//
//
// Returns:     (none)
//---------------------------------------------------------------------------
static void PUBLIC EplTimeruCbMs(ULONG ulParameter_p)
{
    tEplKernel          Ret = kEplSuccessful;
    tEplTimeruData*     pData;
    tEplEvent           EplEvent;
    tEplTimerEventArg   TimerEventArg;

    pData = (tEplTimeruData*) ulParameter_p;

    // call event function
    TimerEventArg.m_TimerHdl = (tEplTimerHdl)pData;
    EPL_MEMCPY(&TimerEventArg.m_Arg, &pData->TimerArgument.m_Arg,
               sizeof (TimerEventArg.m_Arg));

    EplEvent.m_EventSink = pData->TimerArgument.m_EventSink;
    EplEvent.m_EventType = kEplEventTypeTimer;
    EPL_MEMSET(&EplEvent.m_NetTime, 0x00, sizeof(tEplNetTime));
    EplEvent.m_pArg = &TimerEventArg;
    EplEvent.m_uiSize = sizeof(TimerEventArg);

    Ret = EplEventuPost(&EplEvent);
}

//------------------------------------------------------------------------------
// Function:    EplTimeruLinuxUserAddTimer
//
// Description: Adds a user timer into the timer list
//
// Parameters:  pData_p =               pointer to the timer structure
//
// Return:      N/A
//------------------------------------------------------------------------------
static void EplTimeruLinuxUserAddTimer(tEplTimeruData *pData_p)
{
    tEplTimeruData              *pTimerData;

    pthread_mutex_lock(&EplTimeruInstance_g.m_Mutex);

    if (EplTimeruInstance_g.m_pFirstTimer == NULL)
    {
        EplTimeruInstance_g.m_pFirstTimer = pData_p;
        EplTimeruInstance_g.m_pLastTimer = pData_p;

        pData_p->m_pPrevTimer = NULL;
        pData_p->m_pNextTimer = NULL;
    }
    else
    {
        pTimerData = EplTimeruInstance_g.m_pLastTimer;
        pTimerData->m_pNextTimer = pData_p;
        pData_p->m_pPrevTimer = pTimerData;
        pData_p->m_pNextTimer = NULL;
        EplTimeruInstance_g.m_pLastTimer = pData_p;
    }

    pthread_mutex_unlock(&EplTimeruInstance_g.m_Mutex);
}

//------------------------------------------------------------------------------
// Function:    EplTimeruLinuxUserRemoveTimer
//
// Description: Remove a user timer from the timer list
//
// Parameters:  pData_p =               pointer to timer structure
//
// Return:      N/A
//------------------------------------------------------------------------------
static void EplTimeruLinuxUserRemoveTimer(tEplTimeruData *pData_p)
{
    tEplTimeruData              *pTimerData;

    pthread_mutex_lock(&EplTimeruInstance_g.m_Mutex);

    if (pData_p->m_pPrevTimer == NULL)          // first one
    {
        EplTimeruInstance_g.m_pFirstTimer = pData_p->m_pNextTimer;
        pTimerData = pData_p->m_pNextTimer;
        if (pTimerData != NULL)
        {
            pTimerData->m_pPrevTimer = NULL;
        }
    }
    else if (pData_p->m_pNextTimer == NULL)     // last one
    {
        EplTimeruInstance_g.m_pLastTimer = pData_p->m_pPrevTimer;
        pTimerData = pData_p->m_pPrevTimer;
        pTimerData->m_pNextTimer = NULL;
    }
    else
    {
        pData_p->m_pPrevTimer->m_pNextTimer = pData_p->m_pNextTimer;
        pData_p->m_pNextTimer->m_pPrevTimer = pData_p->m_pPrevTimer;
    }

    pthread_mutex_unlock(&EplTimeruInstance_g.m_Mutex);
}

//------------------------------------------------------------------------------
// Function:    EplTimeruResetTimerList
//
// Description: Reset the timer list pointer
//
// Parameters:  N/A
//
// Return:      N/A
//------------------------------------------------------------------------------
static void EplTimeruResetTimerList(void)
{
    EplTimeruInstance_g.m_pCurrentTimer = EplTimeruInstance_g.m_pFirstTimer;
}

//------------------------------------------------------------------------------
// Function:    EplTimeruGetNextTimer
//
// Description: Get the next timer from the timer list
//
// Parameters:  N/A
//
// Return:      returns pointer to the timer structure
//------------------------------------------------------------------------------
static tEplTimeruData * EplTimeruGetNextTimer(void)
{
    tEplTimeruData *pTimer;

    pTimer = EplTimeruInstance_g.m_pCurrentTimer;
    if (pTimer != NULL)
    {
        EplTimeruInstance_g.m_pCurrentTimer = pTimer->m_pNextTimer;
    }
    return pTimer;
}
