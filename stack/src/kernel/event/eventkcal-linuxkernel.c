/**
********************************************************************************
\file   eventkcal-linuxkernel.c

\brief  Kernel event CAL module for Linux kernelspace

This file implements the kernel event handler CAL module for the Linux
kernelspace platform. It uses the circular buffer interface for all event queues.

\see eventkcalintf-circbuf.c

\ingroup module_eventkcal
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2016, B&R Industrial Automation GmbH
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
#include <kernel/eventkcal.h>
#include <kernel/eventkcalintf.h>
#include <oplk/debugstr.h>

#include <linux/kthread.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/wait.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/cpumask.h>
#include <linux/uaccess.h>
#include <asm/uaccess.h>
#include <asm/atomic.h>

//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

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
\brief Kernel event CAL instance type

The structure contains all necessary information needed by the kernel event
CAL module.
*/
typedef struct
{
    struct task_struct*     threadId;
    wait_queue_head_t       kernelWaitQueue;
    wait_queue_head_t       userWaitQueue;
    atomic_t                userEventCount;
    atomic_t                kernelEventCount;
    BOOL                    fThreadIsRunning;
    BOOL                    fInitialized;
    BYTE                    aUintRxBuffer[sizeof(tEvent) + MAX_EVENT_ARG_SIZE];
    BYTE                    aK2URxBuffer[sizeof(tEvent) + MAX_EVENT_ARG_SIZE];
} tEventkCalInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tEventkCalInstance   instance_l;             ///< Instance variable of kernel event CAL module


//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static int  eventThread(void* arg);
static void signalUserEvent(void);
static void signalKernelEvent(void);

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief    Initialize kernel event CAL module

The function initializes the kernel event CAL module. Depending on the
configuration it gets the function pointer interface of the used queue
implementations and calls the appropriate init functions.

\return The function returns a tOplkError error code.
\retval kErrorOk                    Function executes correctly
\retval other error codes           An error occurred

\ingroup module_eventkcal
*/
//------------------------------------------------------------------------------
tOplkError eventkcal_init(void)
{
    OPLK_MEMSET(&instance_l, 0, sizeof(tEventkCalInstance));

    init_waitqueue_head(&instance_l.kernelWaitQueue);
    init_waitqueue_head(&instance_l.userWaitQueue);
    atomic_set(&instance_l.kernelEventCount, 0);
    atomic_set(&instance_l.userEventCount, 0);

    if (eventkcal_initQueueCircbuf(kEventQueueK2U) != kErrorOk)
        goto Exit;

    if (eventkcal_initQueueCircbuf(kEventQueueU2K) != kErrorOk)
        goto Exit;

    if (eventkcal_initQueueCircbuf(kEventQueueKInt) != kErrorOk)
        goto Exit;

    if (eventkcal_initQueueCircbuf(kEventQueueUInt) != kErrorOk)
        goto Exit;

    eventkcal_setSignalingCircbuf(kEventQueueK2U, signalUserEvent);

    eventkcal_setSignalingCircbuf(kEventQueueU2K, signalKernelEvent);

    eventkcal_setSignalingCircbuf(kEventQueueUInt, signalUserEvent);

    eventkcal_setSignalingCircbuf(kEventQueueKInt, signalKernelEvent);

    instance_l.threadId = kthread_run(eventThread, NULL, "EventkThread");

    if (IS_ERR(instance_l.threadId))
    {
        instance_l.threadId = NULL;
        goto Exit;
    }

    set_cpus_allowed_ptr(instance_l.threadId, cpumask_of(1));

    instance_l.fInitialized = TRUE;

    return kErrorOk;

Exit:
    DEBUG_LVL_ERROR_TRACE("%s() Initialization error!\n", __func__);
    eventkcal_exitQueueCircbuf(kEventQueueK2U);
    eventkcal_exitQueueCircbuf(kEventQueueU2K);
    eventkcal_exitQueueCircbuf(kEventQueueKInt);
    eventkcal_exitQueueCircbuf(kEventQueueUInt);

    return kErrorNoResource;
}

//------------------------------------------------------------------------------
/**
\brief    Clean up kernel event CAL module

The function cleans up the kernel event CAL module. For cleanup it calls the exit
functions of the queue implementations for each used queue.

\return The function returns a tOplkError error code.
\retval kErrorOk                    Function executes correctly
\retval other error codes           An error occurred

\ingroup module_eventkcal
*/
//------------------------------------------------------------------------------
tOplkError eventkcal_exit(void)
{
    UINT    i = 0;

    instance_l.fInitialized = FALSE;

    if (instance_l.threadId)
        kthread_stop(instance_l.threadId);

    instance_l.threadId = NULL;

    while (instance_l.fThreadIsRunning)
    {
        msleep(10);
        if (i++ > 1000)
        {
            DEBUG_LVL_EVENTK_TRACE("Event thread is not terminating, continue shutdown...!\n");
            break;
        }
    }

    eventkcal_exitQueueCircbuf(kEventQueueK2U);
    eventkcal_exitQueueCircbuf(kEventQueueU2K);
    eventkcal_exitQueueCircbuf(kEventQueueUInt);
    eventkcal_exitQueueCircbuf(kEventQueueKInt);

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief    Post kernel event

This function posts a event to a queue. It is called from the generic kernel
event post function in the event handler. Depending on the sink the appropriate
queue post function is called.

\param[in]      pEvent_p            Event to be posted.

\return The function returns a tOplkError error code.
\retval kErrorOk                    Function executes correctly
\retval other error codes           An error occurred

\ingroup module_eventkcal
*/
//------------------------------------------------------------------------------
tOplkError eventkcal_postUserEvent(const tEvent* pEvent_p)
{
    tOplkError  ret;

    DEBUG_LVL_EVENTK_TRACE("K2U  type:%s(%d) sink:%s(%d) size:%d!\n",
                           debugstr_getEventTypeStr(pEvent_p->eventType),
                           pEvent_p->eventType,
                           debugstr_getEventSinkStr(pEvent_p->eventSink),
                           pEvent_p->eventSink,
                           pEvent_p->eventArgSize);

    if (instance_l.fInitialized)
        ret = eventkcal_postEventCircbuf(kEventQueueK2U, pEvent_p);
    else
        ret = kErrorIllegalInstance;

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief    Post kernel event

This function posts an event to a queue. It is called from the generic kernel
event post function in the event handler. Depending on the sink the appropriate
queue post function is called.

\param[in]      pEvent_p            Event to be posted.

\return The function returns a tOplkError error code.
\retval kErrorOk                    Function executes correctly
\retval other error codes           An error occurred

\ingroup module_eventkcal
*/
//------------------------------------------------------------------------------
tOplkError eventkcal_postKernelEvent(const tEvent* pEvent_p)
{
    tOplkError  ret;

    DEBUG_LVL_EVENTK_TRACE("KINT  type:%s(%d) sink:%s(%d) size:%d!\n",
                           debugstr_getEventTypeStr(pEvent_p->eventType),
                           pEvent_p->eventType,
                           debugstr_getEventSinkStr(pEvent_p->eventSink),
                           pEvent_p->eventSink,
                           pEvent_p->eventArgSize);

    if (instance_l.fInitialized)
        ret = eventkcal_postEventCircbuf(kEventQueueKInt, pEvent_p);
    else
        ret = kErrorIllegalInstance;

    return ret;
}


//------------------------------------------------------------------------------
/**
\brief  Process function of kernel CAL module

This function will be called by the systems process function.

\ingroup module_eventkcal
*/
//------------------------------------------------------------------------------
void eventkcal_process(void)
{
    // Nothing to do, because we use threads
}

//------------------------------------------------------------------------------
/**
\brief    Post event from user

This function posts a event from the user layer to a queue.

\param[in]      arg                 Ioctl argument. Contains the event to post.

\return The function returns Linux error code.

\ingroup module_eventkcal
*/
//------------------------------------------------------------------------------
int eventkcal_postEventFromUser(ULONG arg)
{
    tOplkError  ret = kErrorOk;
    tEvent      event;
    UINT8*      pArg = NULL;
    int         order = 0;

    if (!instance_l.fInitialized)
        return -EIO;

    if (copy_from_user(&event, (const void __user *)arg, sizeof(tEvent)))
        return -EFAULT;

    if (event.eventArgSize != 0)
    {
        order = get_order(event.eventArgSize);
        pArg = (UINT8*)__get_free_pages(GFP_KERNEL, order);

        if (!pArg)
            return -EIO;

        DEBUG_LVL_EVENTK_TRACE("%s() allocated %d Bytes at %p\n", __func__, event.eventArgSize, pArg);
        if (copy_from_user(pArg, (const void __user *)event.eventArg.pEventArg, event.eventArgSize))
        {
            free_pages((ULONG)pArg, order);
            return -EFAULT;
        }
        event.eventArg.pEventArg = pArg;
    }

    switch (event.eventSink)
    {
        case kEventSinkSync:
        case kEventSinkNmtk:
        case kEventSinkDllk:
        case kEventSinkDllkCal:
        case kEventSinkPdok:
        case kEventSinkPdokCal:
        case kEventSinkErrk:
            DEBUG_LVL_EVENTK_TRACE("U2K  type:%s(%d) sink:%s(%d) size:%d!\n",
                                   debugstr_getEventTypeStr(event.eventType),
                                   event.eventType,
                                   debugstr_getEventSinkStr(event.eventSink),
                                   event.eventSink,
                                   event.eventArgSize);
            ret = eventkcal_postEventCircbuf(kEventQueueU2K, &event);
            break;

        case kEventSinkNmtMnu:
        case kEventSinkNmtu:
        case kEventSinkSdoAsySeq:
        case kEventSinkApi:
        case kEventSinkDlluCal:
        case kEventSinkErru:
            DEBUG_LVL_EVENTK_TRACE("UINT type:%s(%d) sink:%s(%d) size:%d!\n",
                                   debugstr_getEventTypeStr(event.eventType),
                                   event.eventType,
                                   debugstr_getEventSinkStr(event.eventSink),
                                   event.eventSink,
                                   event.eventArgSize);
            ret = eventkcal_postEventCircbuf(kEventQueueUInt, &event);
            break;

        default:
            ret = -EIO;
            break;
    }

    if (event.eventArgSize != 0)
        free_pages((ULONG)pArg, order);

    return 0;
}

//------------------------------------------------------------------------------
/**
\brief    Get an event for the user layer

This function waits for events to the user.

\param[in]      arg                 Ioctl argument. Contains the received event.

\return The function returns Linux error code.

\ingroup module_eventkcal
*/
//------------------------------------------------------------------------------
int eventkcal_getEventForUser(ULONG arg)
{
    tOplkError  error;
    int         ret;
    size_t      readSize;
    int         timeout = 500 * HZ / 1000;

    if (!instance_l.fInitialized)
        return -EIO;

    ret = wait_event_interruptible_timeout(instance_l.userWaitQueue,
                                           (atomic_read(&instance_l.userEventCount) > 0),
                                           timeout);
    if (ret == 0)
    {
        DEBUG_LVL_EVENTK_TRACE("%s() timeout!\n", __func__);
        return -ERESTARTSYS;
    }

    if (ret == -ERESTARTSYS)
    {
        DEBUG_LVL_EVENTK_TRACE("%s() interrupted\n", __func__);
        return ret;
    }

    if (!instance_l.fInitialized)
        return -EIO;

    if (eventkcal_getEventCountCircbuf(kEventQueueK2U) > 0)
    {
        atomic_dec(&instance_l.userEventCount);

        error = eventkcal_getEventCircbuf(kEventQueueK2U, instance_l.aK2URxBuffer, &readSize);
        if (error != kErrorOk)
        {
            DEBUG_LVL_ERROR_TRACE("%s() Error reading K2U events %d!\n", __func__, error);
            return -EIO;
        }

        DEBUG_LVL_EVENTK_TRACE("%s() copy kernel event to user: %d Bytes\n", __func__, readSize);
        if (copy_to_user((void __user *)arg, instance_l.aK2URxBuffer, readSize))
            return -EFAULT;

        return 0;
    }
    else
    {

        if (eventkcal_getEventCountCircbuf(kEventQueueUInt) > 0)
        {
            atomic_dec(&instance_l.userEventCount);

            error = eventkcal_getEventCircbuf(kEventQueueUInt, instance_l.aUintRxBuffer, &readSize);
            if (error != kErrorOk)
            {
                DEBUG_LVL_ERROR_TRACE("%s() Error reading UINT events %d!\n", __func__, error);
                return -EIO;
            }

            DEBUG_LVL_EVENTK_TRACE("%s() copy user event to user: %d Bytes\n", __func__, readSize);
            if (copy_to_user((void __user *)arg, instance_l.aUintRxBuffer, readSize))
                return -EFAULT;

            return 0;
        }
    }

    return -ERESTARTSYS;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief  Event handler thread function

This function contains the main function for the event handler thread.

\param[in,out]  arg                 Thread parameter. Not used!

\return The function returns the thread exit code.
*/
//------------------------------------------------------------------------------
static int eventThread(void* arg)
{
    int     timeout = 500 * HZ / 1000;
    int     result;
#if (defined(CONFIG_PREEMPT_RT_FULL) || defined(CONFIG_PREEMPT_RT))
    struct sched_param  rt_prio;
#endif

    set_user_nice(current, -20);

#if (defined(CONFIG_PREEMPT_RT_FULL) || defined(CONFIG_PREEMPT_RT))
        rt_prio.sched_priority = 79;
        sched_setscheduler(current, SCHED_FIFO, &rt_prio);
#endif

    instance_l.fThreadIsRunning = TRUE;
    while (!kthread_should_stop())
    {
        result = wait_event_interruptible_timeout(instance_l.kernelWaitQueue,
                                                  (atomic_read(&instance_l.kernelEventCount) > 0),
                                                  timeout);

        if (kthread_should_stop())
            break;

        if (result == 0)
            continue;

        /* first handle all kernel internal events --> higher priority! */
        while (eventkcal_getEventCountCircbuf(kEventQueueKInt) > 0)
        {
            eventkcal_processEventCircbuf(kEventQueueKInt);
            atomic_dec(&instance_l.kernelEventCount);
        }

        if (eventkcal_getEventCountCircbuf(kEventQueueU2K) > 0)
        {
            eventkcal_processEventCircbuf(kEventQueueU2K);
            atomic_dec(&instance_l.kernelEventCount);
        }
    }

    instance_l.fThreadIsRunning = FALSE;

    return 0;
}

//------------------------------------------------------------------------------
/**
\brief  Signal a user event

This function signals that a user event was posted. It will be registered in
the circular buffer library as signal callback function
*/
//------------------------------------------------------------------------------
static void signalUserEvent(void)
{
    atomic_inc(&instance_l.userEventCount);
    wake_up_interruptible(&instance_l.userWaitQueue);
}

//------------------------------------------------------------------------------
/**
\brief  Signal a kernel event

This function signals that a kernel event was posted. It will be registered in
the circular buffer library as signal callback function
*/
//------------------------------------------------------------------------------
static void signalKernelEvent(void)
{
    atomic_inc(&instance_l.kernelEventCount);
    wake_up_interruptible(&instance_l.kernelWaitQueue);
}

/// \}
