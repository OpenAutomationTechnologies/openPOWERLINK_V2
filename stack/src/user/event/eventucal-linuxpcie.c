/**
********************************************************************************
\file   eventucal-linuxpcie.c

\brief  User event CAL module for openPOWERLINK PCIe driver on Linux

This file implements the user event CAL module implementation and uses ioctl
calls on Linux to communicate with the openPOWERLINK PCIe driver.

\ingroup module_eventucal
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2014, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
Copyright (c) 2015, Kalycito Infotech Private Limited
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
#include <user/eventucal.h>
#include <user/eventucalintf.h>
#include <user/eventu.h>
#include <user/ctrlucal.h>
#include <common/target.h>
#include <common/driver.h>

#include <pthread.h>
#include <sys/ioctl.h>

//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

#define USER_EVENT_THREAD_PRIORITY    20

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

/**
\brief User event CAL instance type

The structure contains all necessary information needed by the user event
CAL module.
*/
typedef struct
{
    OPLK_FILE_HANDLE    fd;                     ///< File descriptor for the kernel PCIe driver.
    pthread_t           kernelEventThreadId;    ///< K2U event processing thread Id.
    pthread_t           userEventThreadId;      ///< UInt event processing thread Id.
    pthread_mutex_t     userEventMutex;         ///< Mutex for accessing pending user event counter.
    pthread_cond_t      userEventCondition;     ///< Conditional variable for signalling UInt event.
    OPLK_ATOMIC_T       userEventCount;         ///< Pending user event counter.
    BOOL                fStopKernelThread;      ///< Flag to start stop K2U event thread.
    BOOL                fStopUserThread;        ///< Flag to start stop UInt event thread.
    BOOL                fInitialized;           ///< Flag indicate the valid state of this module.
} tEventuCalInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tEventuCalInstance    instance_l;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
void                signalUserEvent(void);
static void*        eventKThread(void* pArg_p);
static void*        eventUThread(void* pArg_p);
static tOplkError   postEvent(tEvent* pEvent_p);

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief    Initialize user event CAL module

The function initializes the user event CAL module. Depending on the
configuration, it gets the function pointer interface of the used queue
implementations and calls the appropriate init functions.

\return The function returns a tOplkError error code.
\retval kErrorOk                Function executes correctly
\retval other error codes       An error occurred

\ingroup module_eventucal
*/
//------------------------------------------------------------------------------
tOplkError eventucal_init(void)
{
    tOplkError                  ret = kErrorOk;
    struct sched_param          schedParam;
    const pthread_mutex_t       userEventMutex = PTHREAD_MUTEX_INITIALIZER;
    const pthread_cond_t        userEventCondition = PTHREAD_COND_INITIALIZER;

    OPLK_MEMSET(&instance_l, 0, sizeof(tEventuCalInstance));

    instance_l.fd = ctrlucal_getFd();
    instance_l.fStopKernelThread = FALSE;
    instance_l.fStopUserThread = FALSE;

    OPLK_MEMCPY(&instance_l.userEventMutex, &userEventMutex, sizeof(pthread_mutex_t));
    OPLK_MEMCPY(&instance_l.userEventCondition, &userEventCondition, sizeof(pthread_cond_t));
    instance_l.userEventCount = 0;

    if (eventucal_initQueueCircbuf(kEventQueueUInt) != kErrorOk)
        goto Exit;

    if (eventucal_setSignalingCircbuf(kEventQueueUInt, signalUserEvent) != kErrorOk)
        goto Exit;

    // Create thread for signalling new user data from kernel
    if (pthread_create(&instance_l.kernelEventThreadId, NULL, eventKThread, NULL) != 0)
        goto Exit;

    schedParam.__sched_priority = USER_EVENT_THREAD_PRIORITY;
    if (pthread_setschedparam(instance_l.kernelEventThreadId, SCHED_FIFO, &schedParam) != 0)
    {
        DEBUG_LVL_ERROR_TRACE("%s(): couldn't set K2U thread scheduling parameters! %d\n",
                              __func__, schedParam.__sched_priority);
    }

#if (defined(__GLIBC__) && __GLIBC__ >= 2 && __GLIBC_MINOR__ >= 12)
    pthread_setname_np(instance_l.kernelEventThreadId, "oplk-eventu");
#endif

    // Create thread for signalling new user internal data
    if (pthread_create(&instance_l.userEventThreadId, NULL, eventUThread, NULL) != 0)
        goto Exit;

    schedParam.__sched_priority = USER_EVENT_THREAD_PRIORITY;
    if (pthread_setschedparam(instance_l.userEventThreadId, SCHED_FIFO, &schedParam) != 0)
    {
        DEBUG_LVL_ERROR_TRACE("%s(): couldn't set UInt thread scheduling parameters! %d\n",
                              __func__, schedParam.__sched_priority);
    }

#if (defined(__GLIBC__) && __GLIBC__ >= 2 && __GLIBC_MINOR__ >= 12)
    pthread_setname_np(instance_l.userEventThreadId, "oplk-eventuint");
#endif

    return kErrorOk;

Exit:
    eventucal_exit();
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief    Clean up user event CAL module

The function cleans up the user event CAL module. For cleanup it calls the exit
functions of the queue implementations for each used queue.

\return The function returns a tOplkError error code.
\retval kErrorOk                Function executes correctly
\retval other error codes       An error occurred

\ingroup module_eventucal
*/
//------------------------------------------------------------------------------
tOplkError eventucal_exit(void)
{
    UINT    timeout = 0;

    if (instance_l.kernelEventThreadId != 0)
    {
        instance_l.fStopKernelThread = TRUE;
        while (instance_l.fStopKernelThread == TRUE)
        {
            target_msleep(10);
            if (timeout++ > 1000)
            {
                DEBUG_LVL_ERROR_TRACE("Kernel-User Event Thread is not terminating, continue shutdown...!\n");
                break;
            }
        }
    }

    timeout = 0;
    if (instance_l.userEventThreadId != 0)
    {
        instance_l.fStopUserThread = TRUE;
        pthread_cond_signal(&instance_l.userEventCondition);
        while (instance_l.fStopUserThread == TRUE)
        {
            target_msleep(10);
            if (timeout++ > 1000)
            {
                DEBUG_LVL_ERROR_TRACE("UInt Event Thread is not terminating, continue shutdown...!\n");
                break;
            }
        }
    }

    eventucal_exitQueueCircbuf(kEventQueueUInt);
    instance_l.fd = (OPLK_FILE_HANDLE)0;
    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief    Post user event

This function is called from the generic user event post function in the
event handler and posts an user event to the UInt queue using circular buffer.

\param  pEvent_p                Event to be posted.

\return The function returns a tOplkError error code.
\retval kErrorOk                Function executes correctly
\retval other error codes       An error occurred

\ingroup module_eventucal
*/
//------------------------------------------------------------------------------
tOplkError eventucal_postUserEvent(tEvent* pEvent_p)
{
    tOplkError    ret = kErrorOk;

    ret = eventucal_postEventCircbuf(kEventQueueUInt, pEvent_p);
    if (ret != kErrorOk)
    {
        DEBUG_LVL_ERROR_TRACE("User internal event could not be posted!!\n");
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief    Post kernel event

This function is called from the generic user event post function in the
event handler and posts an user event to the U2K queue using the ioctl
interface.

\param  pEvent_p                Event to be posted.

\return The function returns a tOplkError error code.
\retval kErrorOk                Function executes correctly
\retval other error codes       An error occurred

\ingroup module_eventucal
*/
//------------------------------------------------------------------------------
tOplkError eventucal_postKernelEvent(tEvent* pEvent_p)
{
    return postEvent(pEvent_p);
}

//------------------------------------------------------------------------------
/**
\brief  Process function of user CAL module

This function will be called by the system process function.

\ingroup module_eventucal
*/
//------------------------------------------------------------------------------
void eventucal_process(void)
{
    // Nothing to do, because we use threads
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief    Post event

This function posts an user event to the U2K queue via the PCIe interface
driver using ioctl interface.

\param  pEvent_p                Event to be posted.

\return The function returns a tOplkError error code.
\retval kErrorOk                Function executes correctly
\retval other error codes       An error occurred
*/
//------------------------------------------------------------------------------
static tOplkError postEvent(tEvent* pEvent_p)
{
    int    ioctlret;

    /* TRACE("%s() Event type:%s(%d) sink:%s(%d) size:%d!\n", __func__,
             debugstr_getEventTypeStr(pEvent_p->eventType), pEvent_p->eventType,
             debugstr_getEventSinkStr(pEvent_p->eventSink), pEvent_p->eventSink,
             pEvent_p->eventArgSize); */

    ioctlret = ioctl(instance_l.fd, PLK_CMD_POST_EVENT, pEvent_p);
    if (ioctlret != 0)
        return kErrorNoResource;

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief    Event thread function

This function implements the K2U event thread. The thread uses the ioctl
interface to the PCIe driver to wait for an event to be posted from the PCP.

\param  pArg_p              Thread argument.

*/
//------------------------------------------------------------------------------
static void* eventKThread(void* pArg_p)
{
    tEvent*     pEvent;
    INT         ret;
    char        eventBuf[sizeof(tEvent) + MAX_EVENT_ARG_SIZE];

    UNUSED_PARAMETER(pArg_p);

    pEvent = (tEvent*)eventBuf;

    while (!instance_l.fStopKernelThread)
    {
        ret = ioctl(instance_l.fd, PLK_CMD_GET_EVENT, eventBuf);
        if (ret == 0)
        {
            /*
                DEBUG_LVL_EVENTK_TRACE("%s() User: got event type:%d(%s) sink:%d(%s)\n", __func__,
                    pEvent->eventType, debugstr_getEventTypeStr(pEvent->eventType),
                    pEvent->eventSink, debugstr_getEventSinkStr(pEvent->eventSink));*/
            if (pEvent->eventArgSize != 0)
                pEvent->eventArg.pEventArg = (char*)pEvent + sizeof(tEvent);

            ret = eventu_process(pEvent);
        }
        else
        {
            // Ignore errors from kernel
            DEBUG_LVL_EVENTK_TRACE("Error in retrieving kernel to user event!!\n");
        }

    }

    instance_l.fStopKernelThread = FALSE;

    return NULL;
}

//------------------------------------------------------------------------------
/**
\brief    Event thread function

This function implements the UInt event thread. The thread waits for an user
event to be posted to the circular buffer and then processes it, once signalled.

\param  pArg_p              Thread argument.

*/
//------------------------------------------------------------------------------
static void* eventUThread(void* pArg_p)
{
    UNUSED_PARAMETER(pArg_p);

    while (!instance_l.fStopUserThread)
    {
        pthread_mutex_lock(&instance_l.userEventMutex);
        instance_l.userEventCount = 0;  // Reset the pending event count
        while ((instance_l.userEventCount <= 0) && (!instance_l.fStopUserThread))
            pthread_cond_wait(&instance_l.userEventCondition, &instance_l.userEventMutex);

        pthread_mutex_unlock(&instance_l.userEventMutex);
        if (instance_l.fStopUserThread)
            break;

        while (eventucal_getEventCountCircbuf(kEventQueueUInt) > 0)
        {
            eventucal_processEventCircbuf(kEventQueueUInt);
        }
    }

    instance_l.fStopUserThread = FALSE;

    return NULL;
}

//------------------------------------------------------------------------------
/**
\brief  Signal a user event

This function signals that a user event was posted. It will be registered in
the circular buffer library as signal callback function.
*/
//------------------------------------------------------------------------------
void signalUserEvent(void)
{
    pthread_mutex_lock(&instance_l.userEventMutex);
    instance_l.userEventCount++;
    pthread_mutex_unlock(&instance_l.userEventMutex);
    pthread_cond_signal(&instance_l.userEventCondition);
}

/// \}
