/**
********************************************************************************
\file   timer-vxworks.c

\brief  Implementation of user timer module for VxWorks

This file contains the implementation of the user timer module for VxWorks.

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

#include <semLib.h>
#include <taskLib.h>
#include <sysLib.h>
#include <timers.h>
#include <hrtimerLib.h>

//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#define TIMERU_MAX_MSGS         20

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

//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------

typedef struct sTimeruData tTimeruData;

struct sTimeruData
{
    timer_t             timer;
    tTimerArg           timerArg;
    tTimeruData*        pNextTimer;
    tTimeruData*        pPrevTimer;
};

typedef struct
{
    int                 taskId;
    SEM_ID              mutex;
    MSG_Q_ID            msgQueue;
    tTimeruData*        pFirstTimer;
    tTimeruData*        pLastTimer;
    tTimeruData*        pCurrentTimer;
} tTimeruInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tTimeruInstance timeruInstance_l;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static void         addTimer(tTimeruData* pData_p);
static void         removeTimer(tTimeruData* pData_p);
static void         resetTimerList(void);
static tTimeruData* getNextTimer(void);
static void         cbTimer(const tTimeruData* pData_p);
static void         processTask(void);

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
    OPLK_MEMSET(&timeruInstance_l, 0, sizeof(timeruInstance_l));

    /* initialize message queue */
    if ((timeruInstance_l.msgQueue = msgQCreate(TIMERU_MAX_MSGS,
                                                sizeof(unsigned long),
                                                MSG_Q_FIFO)) == NULL)
        return kErrorTimerThreadError;

    /* initialize mutex for synchronization */
    timeruInstance_l.mutex = semMCreate(SEM_Q_PRIORITY | SEM_DELETE_SAFE | SEM_INVERSION_SAFE);
    if (timeruInstance_l.mutex == NULL)
        return kErrorTimerThreadError;

    /* create user timer task */
    timeruInstance_l.taskId = taskSpawn("tTimeruPlk",
                                        EPL_TASK_PRIORITY_UTIMER,
                                        0,
                                        EPL_TASK_STACK_SIZE,
                                        (FUNCPTR)processTask,
                                        0,
                                        0,
                                        0,
                                        0,
                                        0,
                                        0,
                                        0,
                                        0,
                                        0,
                                        0);
    if (timeruInstance_l.taskId == ERROR)
        return kErrorTimerThreadError;

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
    ULONG           msg;
    tTimeruData*    pTimer;

    /* send message to timer task to signal shutdown */
    msg = 0;
    msgQSend(timeruInstance_l.msgQueue,
             (char*)&msg,
             sizeof(ULONG),
             NO_WAIT,
             MSG_PRI_NORMAL);

    /* wait for timer task to end */
    while (taskIdVerify(timeruInstance_l.taskId) == OK)
        taskDelay(sysClkRateGet());

    /* free up timer list */
    resetTimerList();
    while ((pTimer = getNextTimer()) != NULL)
    {
        hrtimer_delete (pTimer->timer);
        removeTimer(pTimer);
        OPLK_FREE(pTimer);
    }

    /* cleanup resources */
    semDelete(timeruInstance_l.mutex);
    msgQDelete(timeruInstance_l.msgQueue);

    timeruInstance_l.pFirstTimer = NULL;
    timeruInstance_l.pLastTimer = NULL;

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  User timer process function

This function must be called repeatedly from within the application. It checks
whether a timer has expired.

\note The function is not used in the VxWorks implementation!

\return The function returns a tOplkError error code.

\ingroup module_timeru
*/
//------------------------------------------------------------------------------
tOplkError timeru_process(void)
{
    return kErrorOk;
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
    tTimeruData*        pData;
    struct itimerspec   relTime;
    tHrtimerSig         sig;

    if (pTimerHdl_p == NULL)
        return kErrorTimerInvalidHandle;

    pData = (tTimeruData*)OPLK_MALLOC(sizeof(tTimeruData));
    if (pData == NULL)
    {
        DEBUG_LVL_ERROR_TRACE("Error allocating user timer memory!\n");
        return kErrorNoResource;
    }

    OPLK_MEMCPY(&pData->timerArg, pArgument_p, sizeof(tTimerArg));

    addTimer(pData);

    sig.sigType = kHrtimerSigMsgQueue;
    sig.sigParam.m_signalMq.msgQueue = timeruInstance_l.msgQueue;
    sig.sigParam.m_signalMq.m_sigData = (ULONG)pData;

    if (hrtimer_create(CLOCK_MONOTONIC, &sig, &pData->timer) != 0)
    {
        DEBUG_LVL_ERROR_TRACE("%s() Error hrtimer_create!\n", __func__);
        return kErrorNoResource;
    }

    if (timeInMs_p >= 1000)
    {
        relTime.it_value.tv_sec = (timeInMs_p / 1000);
        relTime.it_value.tv_nsec = (timeInMs_p % 1000) * 1000000;
    }
    else
    {
        relTime.it_value.tv_sec = 0;
        relTime.it_value.tv_nsec = timeInMs_p * 1000000;
    }

    relTime.it_interval.tv_sec = 0;
    relTime.it_interval.tv_nsec = 0;

    DEBUG_LVL_TIMERU_TRACE("%s() Set timer:%08x timeInMs_p=%ld\n",
                           __func__,
                           *pData,
                           timeInMs_p);

    if (hrtimer_settime(pData->timer, 0, &relTime, NULL) < 0)
    {
        DEBUG_LVL_ERROR_TRACE("%s() Error hrtimer_settime!\n", __func__);
        return kErrorTimerNoTimerCreated;
    }

    *pTimerHdl_p = (tTimerHdl)pData;

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
    tTimeruData*        pData;
    struct itimerspec   relTime;

    if (pTimerHdl_p == NULL)
        return kErrorTimerInvalidHandle;

    // check handle itself, i.e. was the handle initialized before
    if (*pTimerHdl_p == 0)
        return timeru_setTimer(pTimerHdl_p, timeInMs_p, pArgument_p);

    pData = (tTimeruData*)*pTimerHdl_p;

    if (timeInMs_p >= 1000)
    {
        relTime.it_value.tv_sec = (timeInMs_p / 1000);
        relTime.it_value.tv_nsec = (timeInMs_p % 1000) * 1000000;
    }
    else
    {
        relTime.it_value.tv_sec = 0;
        relTime.it_value.tv_nsec = timeInMs_p * 1000000;
    }

    DEBUG_LVL_TIMERU_TRACE("%s() Modify timer:%08x timeInMs_p=%ld\n",
                           __func__,
                           *pTimerHdl_p,
                           timeInMs_p);

    relTime.it_interval.tv_sec = 0;
    relTime.it_interval.tv_nsec = 0;

    if (hrtimer_settime(pData->timer, 0, &relTime, NULL) != 0)
    {
        DEBUG_LVL_ERROR_TRACE("%s() Error timer_settime!\n", __func__);
        return kErrorTimerNoTimerCreated;
    }

    // copy the TimerArg after the timer is restarted,
    // so that a timer occurred immediately before hrtimer_settime
    // won't use the new timerArg and
    // therefore the old timer cannot be distinguished from the new one.
    // But if the new timer is too fast, it may get lost.
    OPLK_MEMCPY(&pData->timerArg, pArgument_p, sizeof(tTimerArg));

    return kErrorOk;
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
    tTimeruData*    pData;

    // check pointer to handle
    if (pTimerHdl_p == NULL)
        return kErrorTimerInvalidHandle;

    // check handle itself, i.e. was the handle initialized before
    if (*pTimerHdl_p == 0)
        return kErrorOk;

    pData = (tTimeruData*)*pTimerHdl_p;

    hrtimer_delete(pData->timer);
    removeTimer(pData);
    OPLK_FREE(pData);

    // uninitialize handle
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
\brief  Timer task function

This function implements the timer task function which will be started as task
and is responsible for processing expired timers. The task will be woken up
if a timer message is available in the message queue.

*/
//------------------------------------------------------------------------------
static void processTask(void)
{
    tTimeruData*    timer;

    while (TRUE)
    {
        timer = NULL;
        msgQReceive(timeruInstance_l.msgQueue, (char*)&timer, sizeof(ULONG), WAIT_FOREVER);

        if (timer != NULL)
            cbTimer(timer);
        else
            break;
    }
}

//------------------------------------------------------------------------------
/**
\brief  Timer callback function

This function is registered if a timer is started and therefore will be called
by the timer when it expires.

\param[in]      pData_p             The user defined parameter supplied when starting
                                    the timer.
*/
//------------------------------------------------------------------------------
static void cbTimer(const tTimeruData* pData_p)
{
    tOplkError      ret;
    tEvent          event;
    tTimerEventArg  timerEventArg;

    // call event function
    timerEventArg.timerHdl.handle = (tTimerHdl)pData_p;
    OPLK_MEMCPY(&timerEventArg.argument, &pData_p->timerArg.argument, sizeof(timerEventArg.argument));

    event.eventSink = pData_p->timerArg.eventSink;
    event.eventType = kEventTypeTimer;
    OPLK_MEMSET(&event.netTime, 0x00, sizeof(tNetTime));
    event.eventArg.pEventArg = &timerEventArg;
    event.eventArgSize = sizeof(timerEventArg);

    ret = eventu_postEvent(&event);
}

//------------------------------------------------------------------------------
/**
\brief  Add a timer to the timer list

This function adds a new timer to the timer list.

\param[in,out]  pData_p             Pointer to the timer structure.
*/
//------------------------------------------------------------------------------
static void addTimer(tTimeruData* pData_p)
{
    tTimeruData*    pTimerData;

    semTake(timeruInstance_l.mutex, WAIT_FOREVER);

    if (timeruInstance_l.pFirstTimer == NULL)
    {
        timeruInstance_l.pFirstTimer = pData_p;
        timeruInstance_l.pLastTimer = pData_p;
        pData_p->pPrevTimer = NULL;
        pData_p->pNextTimer = NULL;
    }
    else
    {
        pTimerData = timeruInstance_l.pLastTimer;
        pTimerData->pNextTimer = pData_p;
        pData_p->pPrevTimer = pTimerData;
        pData_p->pNextTimer = NULL;
        timeruInstance_l.pLastTimer = pData_p;
    }

    semGive(timeruInstance_l.mutex);
}

//------------------------------------------------------------------------------
/**
\brief  Remove a timer from the timer list

This function removes a new timer from the timer list.

\param[in,out]  pData_p             Pointer to the timer structure.
*/
//------------------------------------------------------------------------------
static void removeTimer(tTimeruData* pData_p)
{
    tTimeruData*    pTimerData;

    semTake(timeruInstance_l.mutex, WAIT_FOREVER);

    if (pData_p->pPrevTimer == NULL)          // first one
    {
        timeruInstance_l.pFirstTimer = pData_p->pNextTimer;
        pTimerData = pData_p->pNextTimer;
        if (pTimerData != NULL)
            pTimerData->pPrevTimer = NULL;
    }
    else if (pData_p->pNextTimer == NULL)     // last one
    {
        timeruInstance_l.pLastTimer = pData_p->pPrevTimer;
        pTimerData = pData_p->pPrevTimer;
        pTimerData->pNextTimer = NULL;
    }
    else
    {
        pData_p->pPrevTimer->pNextTimer = pData_p->pNextTimer;
        pData_p->pNextTimer->pPrevTimer = pData_p->pPrevTimer;
    }

    semGive(timeruInstance_l.mutex);
}

//------------------------------------------------------------------------------
/**
\brief  Reset the timer list

This function resets the timer list.
*/
//------------------------------------------------------------------------------
static void resetTimerList(void)
{
    timeruInstance_l.pCurrentTimer = timeruInstance_l.pFirstTimer;
}

//------------------------------------------------------------------------------
/**
\brief  Get next timer from the list

This function gets the next timer from the timer list.

\return     The function returns a pointer to the next timer in the timer list.
*/
//------------------------------------------------------------------------------
static tTimeruData* getNextTimer(void)
{
    tTimeruData*    pTimer;

    pTimer = timeruInstance_l.pCurrentTimer;
    if (pTimer != NULL)
        timeruInstance_l.pCurrentTimer = pTimer->pNextTimer;

    return pTimer;
}

/// \}
