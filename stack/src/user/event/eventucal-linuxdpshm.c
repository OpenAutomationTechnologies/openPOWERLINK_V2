/**
********************************************************************************
\file   eventucal-linuxdpshm.c

\brief  User event CAL module for openPOWERLINK PCIe driver on Linux

This file implements the user event CAL module implementation and uses ioctl
calls on Linux to communicate with the openPOWERLINK PCIe driver.

\ingroup module_eventucal
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2016, B&R Industrial Automation GmbH
Copyright (c) 2017, Kalycito Infotech Private Limited
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
#include <oplk/debugstr.h>

#include <pthread.h>
#include <sys/ioctl.h>
#include <time.h>
#include <fcntl.h>
#include <semaphore.h>


//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

#define KERNEL_EVENT_FETCH_THREAD_PRIORITY      20  // Priority of the kernel event fetch thread
#define EVENT_PROCESS_THREAD_PRIORITY           30  // Priority of the process event thread; higher than fetch thread to avoid event drops

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
    pthread_t           kernelEventThreadId;    ///< K2U event fetching thread Id.
    pthread_t           processEventThreadId;   ///< Event processing thread Id.
    BOOL                fStopKernelThread;      ///< Flag to start stop K2U event thread.
    BOOL                fStopProcessThread;     ///< Flag to start stop UInt event thread.
    BOOL                fInitialized;           ///< Flag indicate the valid state of this module.
    sem_t*              semUserData;            ///< Semaphore for signaling pending events to be processed.
} tEventuCalInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tEventuCalInstance    instance_l;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static void         signalUIntEvent(void);
static void*        k2uEventFetchThread(void* pArg_p);
static void*        eventProcessThread(void* pArg_p);
static tOplkError   postEvent(const tEvent* pEvent_p);

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
\retval kErrorOk                    Function executes correctly
\retval other error codes           An error occurred

\ingroup module_eventucal
*/
//------------------------------------------------------------------------------
tOplkError eventucal_init(void)
{
    tOplkError          ret = kErrorOk;
    struct sched_param  schedParam;

    OPLK_MEMSET(&instance_l, 0, sizeof(tEventuCalInstance));

    instance_l.fd = ctrlucal_getFd();
    instance_l.fStopKernelThread = FALSE;
    instance_l.fStopProcessThread = FALSE;

    sem_unlink("/semUserEvent");    // Deinitialize any existing instance of the semaphore

    if ((instance_l.semUserData = sem_open("/semUserEvent", O_CREAT | O_RDWR, S_IRWXG, 0)) == SEM_FAILED)
        goto Exit;

    if (eventucal_initQueueCircbuf(kEventQueueUInt) != kErrorOk)
        goto Exit;

    if (eventucal_setSignalingCircbuf(kEventQueueUInt, signalUIntEvent) != kErrorOk)
        goto Exit;

    // Create thread for fetching new user data from kernel
    if (pthread_create(&instance_l.kernelEventThreadId, NULL, k2uEventFetchThread, NULL) != 0)
        goto Exit;

    schedParam.sched_priority = KERNEL_EVENT_FETCH_THREAD_PRIORITY;
    if (pthread_setschedparam(instance_l.kernelEventThreadId, SCHED_FIFO, &schedParam) != 0)
    {
        DEBUG_LVL_ERROR_TRACE("%s(): couldn't set K2U thread scheduling parameters! %d\n",
                              __func__,
                              schedParam.sched_priority);
    }

#if (defined(__GLIBC__) && (__GLIBC__ >= 2) && (__GLIBC_MINOR__ >= 12))
    pthread_setname_np(instance_l.kernelEventThreadId, "oplk-eventufetch");
#endif

    // Create thread for processing pending user data
    if (pthread_create(&instance_l.processEventThreadId, NULL, eventProcessThread, NULL) != 0)
        goto Exit;

    schedParam.sched_priority = EVENT_PROCESS_THREAD_PRIORITY;
    if (pthread_setschedparam(instance_l.processEventThreadId, SCHED_FIFO, &schedParam) != 0)
    {
        DEBUG_LVL_ERROR_TRACE("%s(): couldn't set event process thread scheduling parameters! %d\n",
                              __func__,
                              schedParam.sched_priority);
    }

#if (defined(__GLIBC__) && (__GLIBC__ >= 2) && (__GLIBC_MINOR__ >= 12))
    pthread_setname_np(instance_l.processEventThreadId, "oplk-eventuprocess");
#endif
    instance_l.fInitialized = TRUE;
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
\retval kErrorOk                    Function executes correctly
\retval other error codes           An error occurred

\ingroup module_eventucal
*/
//------------------------------------------------------------------------------
tOplkError eventucal_exit(void)
{
    UINT    timeout = 0;

    if (instance_l.kernelEventThreadId != 0)
    {
        instance_l.fStopKernelThread = TRUE;
        while (instance_l.fStopKernelThread != FALSE)
        {
            target_msleep(10);
            if (timeout++ > 1000)
            {
                DEBUG_LVL_ERROR_TRACE("%s(): Kernel-user event thread is not terminating, continue shutdown...!\n",
                                      __func__);
                break;
            }
        }
    }

    timeout = 0;
    if (instance_l.processEventThreadId != 0)
    {
        instance_l.fStopProcessThread = TRUE;
        while (instance_l.fStopProcessThread != FALSE)
        {
            target_msleep(10);
            if (timeout++ > 1000)
            {
                DEBUG_LVL_ERROR_TRACE("%s(): Process event thread is not terminating, continue shutdown...!\n",
                                      __func__);
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

\param[in]      pEvent_p            Event to be posted.

\return The function returns a tOplkError error code.
\retval kErrorOk                    Function executes correctly
\retval other error codes           An error occurred

\ingroup module_eventucal
*/
//------------------------------------------------------------------------------
tOplkError eventucal_postUserEvent(const tEvent* pEvent_p)
{
    tOplkError  ret;

    // Check parameter validity
    ASSERT(pEvent_p != NULL);

    ret = eventucal_postEventCircbuf(kEventQueueUInt, pEvent_p);

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief    Post kernel event

This function is called from the generic user event post function in the
event handler and posts an user event to the U2K queue using the ioctl
interface.

\param[in]      pEvent_p            Event to be posted.

\return The function returns a tOplkError error code.
\retval kErrorOk                    Function executes correctly
\retval other error codes           An error occurred

\ingroup module_eventucal
*/
//------------------------------------------------------------------------------
tOplkError eventucal_postKernelEvent(const tEvent* pEvent_p)
{
    // Check parameter validity
    ASSERT(pEvent_p != NULL);

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
    // Nothing to do in the loop call, because we use threads
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

\param[in]      pEvent_p            Event to be posted.

\return The function returns a tOplkError error code.
\retval kErrorOk                    Function executes correctly
\retval other error codes           An error occurred
*/
//------------------------------------------------------------------------------
static tOplkError postEvent(const tEvent* pEvent_p)
{
    int    ioctlret;

    ioctlret = ioctl(instance_l.fd, PLK_CMD_POST_EVENT, pEvent_p);
    if (ioctlret != 0)
        return kErrorNoResource;

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief    Event thread function

This function implements the K2U event fetch thread. The thread uses the ioctl
interface to the PCIe driver to wait for an event to be posted from the PCP.
Once an event has been retrieved, this function posts the event into the UInt
event queue to be processed by process event thread.

\param[in,out]  pArg_p              Thread argument.

\return The function returns a NULL pointer.
*/
//------------------------------------------------------------------------------
static void* k2uEventFetchThread(void* pArg_p)
{
    UINT8   aEventBuf[sizeof(tEvent) + MAX_EVENT_ARG_SIZE];
    tEvent* pEvent = (tEvent*)aEventBuf;
    INT     ret;

    UNUSED_PARAMETER(pArg_p);

    while (!instance_l.fStopKernelThread)
    {
        ret = ioctl(instance_l.fd, PLK_CMD_GET_EVENT, pEvent);
        if (ret == 0)
        {
            DEBUG_LVL_EVENTU_TRACE("%s() User: got event type:%d(%s) sink:%d(%s)\n",
                                   __func__,
                                   pEvent->eventType,
                                   debugstr_getEventTypeStr(pEvent->eventType),
                                   pEvent->eventSink,
                                   debugstr_getEventSinkStr(pEvent->eventSink));

            if (pEvent->eventArgSize == 0)
                pEvent->eventArg.pEventArg = NULL;
            else
                pEvent->eventArg.pEventArg = (UINT8*)pEvent + sizeof(tEvent);

            if (eventucal_postEventCircbuf(kEventQueueUInt, pEvent) != kErrorOk)
            {
                DEBUG_LVL_ERROR_TRACE("%s(): K2U event is dropped!!\n", __func__);
            }
        }
        else
        {
            // Ignore errors from kernel
            DEBUG_LVL_EVENTU_TRACE("%s(): Error in retrieving kernel to user event!!\n", __func__);
        }
    }

    instance_l.fStopKernelThread = FALSE;

    return NULL;
}

//------------------------------------------------------------------------------
/**
\brief    Event thread function

This function implements the event processing thread. The thread waits for a user
or kernel event to be fetched and then processes it, once signaled.

\param[in,out]  pArg_p              Thread argument.

\return The function returns a NULL pointer.
*/
//------------------------------------------------------------------------------
static void* eventProcessThread(void* pArg_p)
{
    struct timespec curTime;
    struct timespec timeout;

    UNUSED_PARAMETER(pArg_p);

    while (!instance_l.fStopProcessThread)
    {
        clock_gettime(CLOCK_REALTIME, &curTime);
        timeout.tv_sec = 0;
        timeout.tv_nsec = 50000 * 1000;
        TIMESPECADD(&timeout, &curTime);

        if (sem_timedwait(instance_l.semUserData, &timeout) == 0)
        {
            if (eventucal_getEventCountCircbuf(kEventQueueUInt) > 0)
                eventucal_processEventCircbuf(kEventQueueUInt);
        }
    }

    instance_l.fStopProcessThread = FALSE;

    return NULL;
}

//------------------------------------------------------------------------------
/**
\brief  Signal a user internal event

This function signals that a user event was posted. It will be registered in
the circular buffer library as signal callback function.
*/
//------------------------------------------------------------------------------
static void signalUIntEvent(void)
{
    sem_post(instance_l.semUserData);
}

/// \}
