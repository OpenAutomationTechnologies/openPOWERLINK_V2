/****************************************************************************
  File:         EplTimeruVxWorks.c

  (c) 2011, Bernecker + Rainer Ges.m.b.H., B&R Strasse 1, A-5142 Eggelsberg
            http://www.br-automation.com

  (c) SYSTEC electronic GmbH, D-07973 Greiz, August-Bebel-Str. 29
      www.systec-electronic.com

  Project:      openPOWERLINK

  Description:  source file for openPOWERLINK userspace timermodule
                implementation for VxWorks RTOS

  License:

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    1. Redistributions of source code must retain the above copyright
       notice, this list of conditions and the following disclaimer.

    2. Redistributions in binary form must reproduce the above copyright
       notice, this list of conditions and the following disclaimer in the
       documentation and/or other materials provided with the distribution.

    3. Neither the name of the copyright holders nor the names of its
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

#include "EplInc.h"
#include "user/EplTimeru.h"

#include <semLib.h>
#include <taskLib.h>
#include <sysLib.h>
#include <timers.h>
#include "hrtimerLib.h"


/***************************************************************************/
/*                                                                         */
/*                                                                         */
/*          G L O B A L   D E F I N I T I O N S                            */
/*                                                                         */
/*                                                                         */
/***************************************************************************/

//---------------------------------------------------------------------------
// const defines
//---------------------------------------------------------------------------
#define	TIMERU_MAX_MSGS				20

//---------------------------------------------------------------------------
// local types
//---------------------------------------------------------------------------


typedef struct EplTimeruData tEplTimeruData;

struct EplTimeruData
{
    timer_t             m_timer;
    tEplTimerArg        m_timerArg;
    tEplTimeruData      *m_pNextTimer;
    tEplTimeruData      *m_pPrevTimer;
};

typedef struct
{
    int                 m_taskId;
    SEM_ID              m_mutex;
    MSG_Q_ID            m_msgQueue;
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
static void EplTimeruAddTimer(tEplTimeruData *pData_p);
static void EplTimeruRemoveTimer(tEplTimeruData *pData_p);
static void EplTimeruResetTimerList(void);
static tEplTimeruData * EplTimeruGetNextTimer(void);
static void EplTimeruCbMs(ULONG ulParameter_p);
static void EplTimeruProcessTask (void);


/***************************************************************************/
/*                                                                         */
/*                                                                         */
/*          C L A S S  <Epl Userspace-Timermodule NoOS>                    */
/*                                                                         */
/*                                                                         */
/***************************************************************************/
//
// Description: Epl Userspace-Timermodule Implementation for use without
//              any operating system
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
// Description: function init first instance
//
// Parameters:
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
// Description: function init additional instance
//
// Parameters:
//
// Returns:     tEplKernel  = errorcode
//---------------------------------------------------------------------------
tEplKernel PUBLIC EplTimeruAddInstance(void)
{
    tEplKernel Ret;

    Ret = kEplSuccessful;

    // reset instance structure
    EPL_MEMSET(&EplTimeruInstance_g, 0, sizeof (EplTimeruInstance_g));

    /* initialize message queue */
    if ((EplTimeruInstance_g.m_msgQueue = msgQCreate(TIMERU_MAX_MSGS,
                                                     sizeof(unsigned long),
                                                     MSG_Q_FIFO)) == NULL)
    {
        Ret = kEplTimerThreadError;
        goto Exit;
    }

    /* initialize mutexe for synchronisation */
    if ((EplTimeruInstance_g.m_mutex =
         semMCreate (SEM_Q_PRIORITY | SEM_DELETE_SAFE |
                     SEM_INVERSION_SAFE)) == NULL)
    {
        Ret = kEplTimerThreadError;
        goto Exit;
    }

    /* create user timer task */
    if ((EplTimeruInstance_g.m_taskId =
         taskSpawn("tTimerEplu", EPL_TASK_PRIORITY_UTIMER, 0, EPL_TASK_STACK_SIZE,
                   (FUNCPTR)EplTimeruProcessTask,
                   0, 0, 0, 0, 0, 0, 0, 0, 0, 0)) == ERROR)
    {
        Ret = kEplTimerThreadError;
        goto Exit;
    }

Exit:
    return Ret;

}

//---------------------------------------------------------------------------
// Function:    EplTimeruDelInstance
//
// Description: function deletes instance
//
// Parameters:
//
// Returns:     tEplKernel  = errorcode
//---------------------------------------------------------------------------
tEplKernel PUBLIC EplTimeruDelInstance(void)
{
    tEplKernel          Ret;
    unsigned long       msg;
    tEplTimeruData 		*pTimer;

    Ret = kEplSuccessful;

    /* send message to timer task to signal shutdown */
    msg = 0;
    msgQSend (EplTimeruInstance_g.m_msgQueue, (char *)&msg, sizeof(unsigned long),
              NO_WAIT, MSG_PRI_NORMAL);

    /* wait for timer task to end */
    while (taskIdVerify(EplTimeruInstance_g.m_taskId) == OK)
    	taskDelay(sysClkRateGet());

    /* free up timer list */
    EplTimeruResetTimerList();
    while ((pTimer = EplTimeruGetNextTimer()) != NULL)
    {
        hrtimer_delete (pTimer->m_timer);
        EplTimeruRemoveTimer(pTimer);
        EPL_FREE(pTimer);
    }

    /* cleanup resources */
    semDelete (EplTimeruInstance_g.m_mutex);
    msgQDelete (EplTimeruInstance_g.m_msgQueue);

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
// Parameters:  none
//
// Returns:     tEplKernel  = errorcode
//---------------------------------------------------------------------------
tEplKernel PUBLIC EplTimeruProcess(void)
{
    return kEplSuccessful;
}

//---------------------------------------------------------------------------
// Function:    EplTimeruSetTimerMs
//
// Description: function creates a timer and returns a handle to the pointer
//
// Parameters:  pTimerHdl_p = pointer to a buffer to fill in the handle
//              ulTimeMs_p  = time for timer in ms
//              Argument_p  = argument for timer
//
// Returns:     tEplKernel  = errorcode
//---------------------------------------------------------------------------
tEplKernel PUBLIC EplTimeruSetTimerMs(  tEplTimerHdl*   pTimerHdl_p,
                                        ULONG           ulTime_p,
                                        tEplTimerArg    Argument_p)
{
    tEplKernel          Ret = kEplSuccessful;
    tEplTimeruData*     pData;
    struct itimerspec   RelTime;
    tHrtimerSig			sig;

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
        printf ("error allocating user timer memory!\n");
        goto Exit;
    }

    EPL_MEMCPY(&pData->m_timerArg, &Argument_p, sizeof(tEplTimerArg));

    EplTimeruAddTimer(pData);

    sig.sigType = kHrtimerSigMsgQueue;
    sig.sigParam.m_signalMq.m_msgQueue = EplTimeruInstance_g.m_msgQueue;
    sig.sigParam.m_signalMq.m_sigData = (unsigned long)pData;

    if (hrtimer_create(CLOCK_MONOTONIC, &sig, &pData->m_timer) != 0)
    {
        EPL_DBGLVL_ERROR_TRACE("%s() Error hrtimer_create!\n", __func__);
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

    RelTime.it_interval.tv_sec = 0;
    RelTime.it_interval.tv_nsec = 0;

    EPL_DBGLVL_TIMERU_TRACE("%s() Set timer:%08x ulTime_p=%ld\n",
                             __func__, *pData, ulTime_p);

    if (hrtimer_settime(pData->m_timer, 0, &RelTime, NULL) < 0)
    {
        EPL_DBGLVL_ERROR_TRACE("%s() Error hrtimer_settime!\n", __func__);
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
tEplKernel PUBLIC EplTimeruModifyTimerMs(tEplTimerHdl*    pTimerHdl_p,
                                        ULONG		      ulTime_p,
                                        tEplTimerArg      Argument_p)
{
    tEplKernel          Ret = kEplSuccessful;
    tEplTimeruData*     pData;
    struct itimerspec   RelTime;

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

    EPL_DBGLVL_TIMERU_TRACE("%s() Modify timer:%08x ulTime_p=%ld\n",
                             __func__, *pTimerHdl_p, ulTime_p);

    RelTime.it_interval.tv_sec = 0;
    RelTime.it_interval.tv_nsec = 0;

    if (hrtimer_settime(pData->m_timer, 0, &RelTime, NULL) != 0)
    {
        EPL_DBGLVL_ERROR_TRACE("%s() Error timer_settime!\n", __func__);
        Ret = kEplTimerNoTimerCreated;
        goto Exit;
    }

    // copy the TimerArg after the timer is restarted,
    // so that a timer occured immediately before hrtimer_settime
    // won't use the new TimerArg and
    // therefore the old timer cannot be distinguished from the new one.
    // But if the new timer is too fast, it may get lost.
    EPL_MEMCPY(&pData->m_timerArg, &Argument_p, sizeof(tEplTimerArg));

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
    tEplKernel      Ret;
    tEplTimeruData*     pData;

    Ret         = kEplSuccessful;

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

    hrtimer_delete (pData->m_timer);
    EplTimeruRemoveTimer(pData);
    EPL_FREE(pData);

    // uninitialize handle
    *pTimerHdl_p = 0;

Exit:
    return Ret;

}

//=========================================================================//
//                                                                         //
//          P R I V A T E   F U N C T I O N S                              //
//                                                                         //
//=========================================================================//

//---------------------------------------------------------------------------
//  Task for processing the timer list
//---------------------------------------------------------------------------
static void EplTimeruProcessTask (void)
{
    unsigned long   ulTimer;

    while (TRUE)
    {
    	ulTimer = 0;
        msgQReceive(EplTimeruInstance_g.m_msgQueue, (char *)&ulTimer,
                    sizeof(unsigned long), WAIT_FOREVER);

        if (ulTimer != 0)
        {
            /* call callback function of timer */
            EplTimeruCbMs(ulTimer);
        }
        else
        {
            break;
        }
    }

    return;
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
static void EplTimeruCbMs(ULONG ulParameter_p)
{
    tEplKernel          Ret = kEplSuccessful;
    tEplTimeruData*     pData;
    tEplEvent           EplEvent;
    tEplTimerEventArg   TimerEventArg;

    pData = (tEplTimeruData*) ulParameter_p;

    // call event function
    TimerEventArg.m_TimerHdl = (tEplTimerHdl)pData;
    EPL_MEMCPY(&TimerEventArg.m_Arg, &pData->m_timerArg.m_Arg,
               sizeof (TimerEventArg.m_Arg));

    EplEvent.m_EventSink = pData->m_timerArg.m_EventSink;
    EplEvent.m_EventType = kEplEventTypeTimer;
    EPL_MEMSET(&EplEvent.m_NetTime, 0x00, sizeof(tEplNetTime));
    EplEvent.m_pArg = &TimerEventArg;
    EplEvent.m_uiSize = sizeof(TimerEventArg);

    Ret = EplEventuPost(&EplEvent);
}

//------------------------------------------------------------------------------
// Function:    EplTimeruAddTimer
//
// Description: Adds a user timer into the timer list
//
// Parameters:  pData_p =               pointer to the timer structure
//
// Return:      N/A
//------------------------------------------------------------------------------
static void EplTimeruAddTimer(tEplTimeruData *pData_p)
{
    tEplTimeruData              *pTimerData;

    semTake(EplTimeruInstance_g.m_mutex, WAIT_FOREVER);

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

    semGive(EplTimeruInstance_g.m_mutex);
}

//------------------------------------------------------------------------------
// Function:    EplTimeruRemoveTimer
//
// Description: Remove a user timer from the timer list
//
// Parameters:  pData_p =               pointer to timer structure
//
// Return:      N/A
//------------------------------------------------------------------------------
static void EplTimeruRemoveTimer(tEplTimeruData *pData_p)
{
    tEplTimeruData              *pTimerData;

    semTake(EplTimeruInstance_g.m_mutex, WAIT_FOREVER);

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

    semGive(EplTimeruInstance_g.m_mutex);
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
