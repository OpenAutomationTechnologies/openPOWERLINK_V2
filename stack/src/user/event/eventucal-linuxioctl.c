/**
********************************************************************************
\file   eventucal-linuxioctl.c

\brief  User event CAL module using ioctl calls on Linux

This user event CAL module implementation uses ioctl calls on Linux.

\brief  User event CAL module for Linux user/kernelspace

This file implements the user event handler CAL module for the Linux
userspace platform. It uses the ioctl() calls to communicate with a kernel
CAL module running in Linux kernelspace.

\ingroup module_eventucal
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2014, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
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
#include <user/eventu.h>
#include <user/ctrlucal.h>
#include <common/target.h>
#include <common/driver.h>
//#include <oplk/debugstr.h>

#include <pthread.h>
#include <sys/ioctl.h>

//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

#define USER_EVENT_THREAD_PRIORITY      20

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
    int                 fd;
    pthread_t           threadId;
    BOOL                fStopThread;
} tEventuCalInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tEventuCalInstance    instance_l;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static void* eventThread(void* arg_p);
static tOplkError postEvent(tEvent* pEvent_p);

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief    Initialize user event CAL module

The function initializes the user event CAL module. Depending on the
configuration it gets the function pointer interface of the used queue
implementations and calls the appropriate init functions.

\return The function returns a tOplkError error code.
\retval kErrorOk                Function executes correctly
\retval other error codes       An error occurred

\ingroup module_eventucal
*/
//------------------------------------------------------------------------------
tOplkError eventucal_init(void)
{
    tOplkError          ret = kErrorOk;
    struct sched_param  schedParam;

    OPLK_MEMSET(&instance_l, 0, sizeof(tEventuCalInstance));

    instance_l.fd = ctrlucal_getFd();
    instance_l.fStopThread = FALSE;

    //create thread for signaling new data
    if (pthread_create(&instance_l.threadId, NULL, eventThread, NULL) != 0)
    {
        goto Exit;
    }
    schedParam.sched_priority = USER_EVENT_THREAD_PRIORITY;
    if (pthread_setschedparam(instance_l.threadId, SCHED_FIFO, &schedParam) != 0)
    {
        DEBUG_LVL_ERROR_TRACE("%s(): couldn't set thread scheduling parameters! %d\n",
                              __func__, schedParam.sched_priority);
    }

#if (defined(__GLIBC__) && __GLIBC__ >= 2 && __GLIBC_MINOR__ >= 12)
    pthread_setname_np(instance_l.threadId, "oplk-eventu");
#endif

Exit:
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
    UINT            i = 0;

    if (instance_l.threadId != 0)
    {
        instance_l.fStopThread = TRUE;
        while (instance_l.fStopThread == TRUE)
        {
            target_msleep(10);
            if (i++ > 1000)
            {
                TRACE("Event Thread is not terminating, continue shutdown...!\n");
                break;
            }
        }
    }

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief    Post user event

This function posts an event to a queue. It is called from the generic user
event post function in the event handler. Depending on the sink the appropriate
queue post function is called.

\param  pEvent_p                Event to be posted.

\return The function returns a tOplkError error code.
\retval kErrorOk                Function executes correctly
\retval other error codes       An error occurred

\ingroup module_eventucal
*/
//------------------------------------------------------------------------------
tOplkError eventucal_postUserEvent(tEvent* pEvent_p)
{
    return postEvent(pEvent_p);
}

//------------------------------------------------------------------------------
/**
\brief    Post kernel event

This function posts an event to a queue. It is called from the generic user
event post function in the event handler. Depending on the sink the appropriate
queue post function is called.

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

This function posts an event to a queue. It is called from the generic user
event post function in the event handler. Depending on the sink the appropriate
queue post function is called.

\param  pEvent_p                Event to be posted.

\return The function returns a tOplkError error code.
\retval kErrorOk                Function executes correctly
\retval other error codes       An error occurred
*/
//------------------------------------------------------------------------------
static tOplkError postEvent(tEvent* pEvent_p)
{
    int             ioctlret;

    /*TRACE("%s() Event type:%s(%d) sink:%s(%d) size:%d!\n", __func__,
           debugstr_getEventTypeStr(pEvent_p->eventType), pEvent_p->eventType,
           debugstr_getEventSinkStr(pEvent_p->eventSink), pEvent_p->eventSink,
           pEvent_p->eventArgSize);*/

    ioctlret = ioctl(instance_l.fd, PLK_CMD_POST_EVENT, pEvent_p);
    if (ioctlret != 0)
        return kErrorNoResource;

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief    Event thread function

This function implements the event thread.

\param  arg_p                Thread argument.

*/
//------------------------------------------------------------------------------
static void* eventThread(void* arg_p)
{
    tEvent*     pEvent;
    int         ret;
    char        eventBuf[sizeof(tEvent) + MAX_EVENT_ARG_SIZE];

    UNUSED_PARAMETER(arg_p);

    pEvent = (tEvent*)eventBuf;

    while (!instance_l.fStopThread)
    {
        ret = ioctl(instance_l.fd, PLK_CMD_GET_EVENT, eventBuf);
        if (ret == 0)
        {
            /*TRACE("%s() User: got event type:%d(%s) sink:%d(%s)\n", __func__,
                    pEvent->eventType, debugstr_getEventTypeStr(pEvent->eventType),
                    pEvent->eventSink, debugstr_getEventSinkStr(pEvent->eventSink));*/
            if (pEvent->eventArgSize != 0)
                pEvent->pEventArg = (char*)pEvent + sizeof(tEvent);

            ret = eventu_process(pEvent);
        }
        /*else
            TRACE("%s() ret = %d\n", __func__, ret);*/
    }
    instance_l.fStopThread = FALSE;

    return NULL;
}

///\}
