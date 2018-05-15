/**
********************************************************************************
\file   eventkcal-linux.c

\brief  Kernel event CAL module for Linux userspace

This file implements the kernel event handler CAL module for the Linux
userspace platform. It uses the circular buffer interface for all event queues.

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
#include <common/target.h>

#include <time.h>
#include <fcntl.h>
#include <pthread.h>
#include <semaphore.h>
#include <common/target.h>

//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#define KERNEL_EVENT_THREAD_PRIORITY        55

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
} tEventkCalInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tEventkCalInstance   instance_l;             ///< Instance variable of kernel event CAL module

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static void* eventThread(void* arg);
static void  signalKernelEvent(void);
static void  signalUserEvent(void);

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief    Initialize kernel event CAL module

The function initializes the kernel event CAL module on Linux.

\return The function returns a tOplkError error code.
\retval kErrorOk                    Function executes correctly
\retval other error codes           An error occurred

\ingroup module_eventkcal
*/
//------------------------------------------------------------------------------
tOplkError eventkcal_init(void)
{
    struct sched_param  schedParam;

    OPLK_MEMSET(&instance_l, 0, sizeof(tEventkCalInstance));

    sem_unlink("/semUserEvent");
    sem_unlink("/semKernelEvent");

    if ((instance_l.semUserData = sem_open("/semUserEvent", O_CREAT | O_RDWR, S_IRWXG, 0)) == SEM_FAILED)
        goto Exit;

    if ((instance_l.semKernelData = sem_open("/semKernelEvent", O_CREAT | O_RDWR, S_IRWXG, 0)) == SEM_FAILED)
        goto Exit;

    if (eventkcal_initQueueCircbuf(kEventQueueK2U) != kErrorOk)
        goto Exit;

    if (eventkcal_initQueueCircbuf(kEventQueueU2K) != kErrorOk)
        goto Exit;

    if (eventkcal_initQueueCircbuf(kEventQueueKInt) != kErrorOk)
        goto Exit;

    eventkcal_setSignalingCircbuf(kEventQueueK2U, signalUserEvent);

    eventkcal_setSignalingCircbuf(kEventQueueKInt, signalKernelEvent);

    instance_l.fStopThread = FALSE;
    if (pthread_create(&instance_l.threadId, NULL, eventThread, (void*)&instance_l) != 0)
        goto Exit;

    schedParam.sched_priority = KERNEL_EVENT_THREAD_PRIORITY;
    if (pthread_setschedparam(instance_l.threadId, SCHED_FIFO, &schedParam) != 0)
    {
        DEBUG_LVL_ERROR_TRACE("%s(): couldn't set thread scheduling parameters! %d\n",
                              __func__,
                              schedParam.sched_priority);
    }

#if (defined(__GLIBC__) && __GLIBC__ >= 2 && __GLIBC_MINOR__ >= 12)
    pthread_setname_np(instance_l.threadId, "oplk-eventk");
#endif

    instance_l.fInitialized = TRUE;
    return kErrorOk;

Exit:
    if (instance_l.semUserData != SEM_FAILED)
        sem_close(instance_l.semUserData);

    if (instance_l.semKernelData != SEM_FAILED)
        sem_close(instance_l.semKernelData);

    eventkcal_exitQueueCircbuf(kEventQueueK2U);
    eventkcal_exitQueueCircbuf(kEventQueueU2K);
    eventkcal_exitQueueCircbuf(kEventQueueKInt);

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

    if (instance_l.fInitialized != FALSE)
    {
        instance_l.fStopThread = TRUE;
        while (instance_l.fStopThread != FALSE)
        {
            target_msleep(10);
            if (i++ > 100)
            {
                DEBUG_LVL_EVENTK_TRACE("Event thread is not terminating, continue shutdown...!\n");
                break;
            }
        }

        eventkcal_exitQueueCircbuf(kEventQueueK2U);
        eventkcal_exitQueueCircbuf(kEventQueueU2K);
        eventkcal_exitQueueCircbuf(kEventQueueKInt);

        sem_close(instance_l.semUserData);
        sem_close(instance_l.semKernelData);

        sem_unlink("/semUserEvent");
        sem_unlink("/semKernelEvent");

    }
    instance_l.fInitialized = FALSE;

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief    Post kernel event

This function posts a event to the kernel queue.

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

    ret = eventkcal_postEventCircbuf(kEventQueueKInt, pEvent_p);

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief    Post user event

This function posts a event to the user queue.

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

    ret = eventkcal_postEventCircbuf(kEventQueueK2U, pEvent_p);

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
static void* eventThread(void* arg)
{
    struct timespec         curTime, timeout;
    tEventkCalInstance*     pInstance = (tEventkCalInstance*)arg;

    while (!pInstance->fStopThread)
    {
        clock_gettime(CLOCK_REALTIME, &curTime);
        timeout.tv_sec = 0;
        timeout.tv_nsec = 50000 * 1000;
        TIMESPECADD(&timeout, &curTime);

        if (sem_timedwait(pInstance->semKernelData, &timeout) == 0)
        {
            /* first handle kernel internal events --> higher priority! */
            if (eventkcal_getEventCountCircbuf(kEventQueueKInt) > 0)
            {
                eventkcal_processEventCircbuf(kEventQueueKInt);
            }
            else
            {
                if (eventkcal_getEventCountCircbuf(kEventQueueU2K) > 0)
                {
                    eventkcal_processEventCircbuf(kEventQueueU2K);
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
static void signalUserEvent(void)
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
static void signalKernelEvent(void)
{
    sem_post(instance_l.semKernelData);
}

/// \}
