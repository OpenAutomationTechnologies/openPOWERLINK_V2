/**
********************************************************************************
\file   eventucal-linux.c

\brief  User event CAL module for Linux userspace

This file implements the user event handler CAL module for the Linux
userspace platform. It uses the circular buffer interface for all event queues.

\see eventucalintf-circbuf.c

\ingroup module_eventucal
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2012, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
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
#include <oplk/oplkinc.h>
#include <oplk/oplk.h>
#include <oplk/debugstr.h>

#include <user/eventu.h>
#include <user/eventucal.h>
#include <user/eventucalintf.h>

#include <time.h>
#include <fcntl.h>
#include <pthread.h>
#include <semaphore.h>
#include <linux/errno.h>
#include <common/target.h>

//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#define USER_EVENT_THREAD_PRIORITY        45

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
    pthread_t               threadId;
    BOOL                    fStopThread;
    sem_t*                  semUserData;
    sem_t*                  semKernelData;
    BOOL                    fInitialized;
} tEventuCalInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tEventuCalInstance       instance_l;             ///< Instance variable of kernel event CAL module

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static void* eventThread(void *arg);
static void signalUserEvent(void);
static void signalKernelEvent(void);

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief    Initialize architecture specific stuff of user event CAL module

The function initializes the architecture specific stuff of the user event
CAL module.

\return The function returns a tOplkError error code.
\retval kErrorOk          If function executes correctly
\retval other error codes       If an error occurred

\ingroup module_eventucal
*/
//------------------------------------------------------------------------------
tOplkError eventucal_init (void)
{
    struct sched_param  schedParam;

    OPLK_MEMSET(&instance_l, 0, sizeof(tEventuCalInstance));

    if ((instance_l.semUserData = sem_open("/semUserEvent", O_RDWR)) == SEM_FAILED)
        goto Exit;

    if ((instance_l.semKernelData = sem_open("/semKernelEvent", O_RDWR)) == SEM_FAILED)
        goto Exit;

    if (eventucal_initQueueCircbuf(kEventQueueK2U) != kErrorOk)
        goto Exit;

    if (eventucal_initQueueCircbuf(kEventQueueU2K) != kErrorOk)
        goto Exit;

    eventucal_setSignalingCircbuf(kEventQueueU2K, signalKernelEvent);

    if (eventucal_initQueueCircbuf(kEventQueueUInt) != kErrorOk)
        goto Exit;

    eventucal_setSignalingCircbuf(kEventQueueUInt, signalUserEvent);

    instance_l.fStopThread = FALSE;
    if (pthread_create(&instance_l.threadId, NULL, eventThread, (void*)&instance_l) != 0)
        goto Exit;

    schedParam.__sched_priority = USER_EVENT_THREAD_PRIORITY;
    if (pthread_setschedparam(instance_l.threadId, SCHED_FIFO, &schedParam) != 0)
    {
        DEBUG_LVL_ERROR_TRACE("%s(): couldn't set thread scheduling parameters! %d\n",
               __func__, schedParam.__sched_priority);
    }
    instance_l.fInitialized = TRUE;
    return kErrorOk;

Exit:
    if (instance_l.semUserData != SEM_FAILED)
        sem_close(instance_l.semUserData);

    if (instance_l.semKernelData != SEM_FAILED)
        sem_close(instance_l.semKernelData);

    eventucal_exitQueueCircbuf(kEventQueueK2U);
    eventucal_exitQueueCircbuf(kEventQueueU2K);
    eventucal_exitQueueCircbuf(kEventQueueUInt);

    return kErrorNoResource;
}

//------------------------------------------------------------------------------
/**
\brief    Cleanup kernel event CAL module

The function cleans up the kernel event CAL module. For cleanup it calls the exit
functions of the queue implementations for each used queue.

\return The function returns a tOplkError error code.
\retval kErrorOk          If function executes correctly
\retval other error codes       If an error occurred

\ingroup module_eventucal
*/
//------------------------------------------------------------------------------
tOplkError eventucal_exit (void)
{
    UINT            i = 0;

    if (instance_l.fInitialized == TRUE)
    {
        instance_l.fStopThread = TRUE;
        while (instance_l.fStopThread == TRUE)
        {
            target_msleep(10);
            if (i++ > 100)
            {
                TRACE("Event Thread is not terminating, continue shutdown...!\n");
                break;
            }
        }

        eventucal_exitQueueCircbuf(kEventQueueK2U);
        eventucal_exitQueueCircbuf(kEventQueueU2K);
        eventucal_exitQueueCircbuf(kEventQueueUInt);

        sem_close(instance_l.semUserData);
        sem_close(instance_l.semKernelData);
    }
    instance_l.fInitialized = FALSE;

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief    Post kernel event

This function posts a event to a queue. It is called from the generic kernel
event post function in the event handler. Depending on the sink the appropriate
queue post function is called.

\param  pEvent_p                Event to be posted.

\return The function returns a tOplkError error code.
\retval kErrorOk          If function executes correctly
\retval other error codes       If an error occurred

\ingroup module_eventucal
*/
//------------------------------------------------------------------------------
tOplkError eventucal_postKernelEvent (tEvent *pEvent_p)
{
    tOplkError      ret;
    /*TRACE("U2K type:%s(%d) sink:%s(%d) size:%d!\n",
                   debugstr_getEventTypeStr(pEvent_p->eventType), pEvent_p->eventType,
                   debugstr_getEventSinkStr(pEvent_p->eventSink), pEvent_p->eventSink,
                   pEvent_p->eventArgSize);*/
    ret = eventucal_postEventCircbuf(kEventQueueU2K, pEvent_p);
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief    Post user event

This function posts a event to a queue. It is called from the generic kernel
event post function in the event handler. Depending on the sink the appropriate
queue post function is called.

\param  pEvent_p                Event to be posted.

\return The function returns a tOplkError error code.
\retval kErrorOk          If function executes correctly
\retval other error codes       If an error occurred

\ingroup module_eventucal
*/
//------------------------------------------------------------------------------
tOplkError eventucal_postUserEvent (tEvent *pEvent_p)
{
    tOplkError      ret;

    /*TRACE("UINT  type:%s(%d) sink:%s(%d) size:%d!\n",
                   debugstr_getEventTypeStr(pEvent_p->eventType), pEvent_p->eventType,
                   debugstr_getEventSinkStr(pEvent_p->eventSink), pEvent_p->eventSink,
                   pEvent_p->eventArgSize);*/
    ret = eventucal_postEventCircbuf(kEventQueueUInt, pEvent_p);

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Process function of user CAL module

This function will be called by the systems process function.

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
\brief  Event handler thread function

This function contains the main function for the event handler thread.

\param  arg                     Thread parameter. Not used!

\return The function returns the thread exit code.
*/
//------------------------------------------------------------------------------
static void* eventThread(void *arg)
{
    struct timespec         curTime, timeout;
    tEventuCalInstance*     pInstance = (tEventuCalInstance*)arg;

    while (!pInstance->fStopThread)
    {
        clock_gettime(CLOCK_REALTIME, &curTime);
        timeout.tv_sec = 0;
        timeout.tv_nsec = 50000 * 1000;
        TIMESPECADD(&timeout, &curTime);

        if (sem_timedwait(pInstance->semUserData, &timeout) == 0)
        {
            /* first handle all user to kernel events --> higher priority! */
            if (eventucal_getEventCountCircbuf(kEventQueueK2U) > 0)
            {
                eventucal_processEventCircbuf(kEventQueueK2U);
            }
            else
            {
                if (eventucal_getEventCountCircbuf(kEventQueueUInt) > 0)
                {
                    eventucal_processEventCircbuf(kEventQueueUInt);
                }
            }
        }
    }
    pInstance->fStopThread = FALSE;

    return 0;
}

//------------------------------------------------------------------------------
/**
\brief  Signal a user event

This function signals that a user event was posted. It will be registered in
the circular buffer library as signal callback function
*/
//------------------------------------------------------------------------------
void signalUserEvent(void)
{
    sem_post(instance_l.semUserData);
}

//------------------------------------------------------------------------------
/**
\brief  Signal a kernel event

This function signals that a kernel event was posted. It will be registered in
the circular buffer library as signal callback function
*/
//------------------------------------------------------------------------------
void signalKernelEvent(void)
{
    sem_post(instance_l.semKernelData);
}

/// \}
