/**
********************************************************************************
\file   eventkcal-linux.c

\brief  Kernel event CAL module using circular buffers on Linux

This kernel event CAL module implementation uses circular buffers on Linux.

\ingroup module_eventkcal
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
#include <EplInc.h>
#include <Epl.h>
#include <eventcal.h>
#include <kernel/eventkcal.h>
#include <kernel/eventkcalintf.h>

#include <time.h>
#include <fcntl.h>
#include <pthread.h>
#include <semaphore.h>
#include <linux/errno.h>

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
static void* eventThread(void *arg);
static void signalKernelEvent(void);
static void signalUserEvent(void);

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief    Initialize kernel event CAL module

The function initializes the kernel event CAL module on Linux.

\return The function returns a tEplKernel error code.
\retval kEplSuccessful          If function executes correctly
\retval other error codes       If an error occurred

\ingroup module_eventkcal
*/
//------------------------------------------------------------------------------
tEplKernel eventkcal_init (void)
{
    struct sched_param  schedParam;

    EPL_MEMSET(&instance_l, 0, sizeof(tEventkCalInstance));

    if ((instance_l.semUserData = sem_open("/semUserEvent", O_CREAT | O_RDWR, S_IRWXG, 0)) == SEM_FAILED)
        goto Exit;

    if ((instance_l.semKernelData = sem_open("/semKernelEvent", O_CREAT | O_RDWR, S_IRWXG, 0)) == SEM_FAILED)
        goto Exit;

    if (eventkcal_initQueueCircbuf(kEventQueueK2U) != kEplSuccessful)
        goto Exit;

    if (eventkcal_initQueueCircbuf(kEventQueueU2K) != kEplSuccessful)
        goto Exit;

    if (eventkcal_initQueueCircbuf(kEventQueueKInt) != kEplSuccessful)
        goto Exit;

    eventkcal_setSignalingCircbuf(kEventQueueK2U, signalUserEvent);

    eventkcal_setSignalingCircbuf(kEventQueueKInt, signalKernelEvent);

    instance_l.fStopThread = FALSE;
    if (pthread_create(&instance_l.threadId, NULL, eventThread, (void*)&instance_l) != 0)
        goto Exit;

    schedParam.__sched_priority = KERNEL_EVENT_THREAD_PRIORITY;
    if (pthread_setschedparam(instance_l.threadId, SCHED_FIFO, &schedParam) != 0)
    {
        EPL_DBGLVL_ERROR_TRACE("%s(): couldn't set thread scheduling parameters! %d\n",
               __func__, schedParam.__sched_priority);
    }

    instance_l.fInitialized = TRUE;
    return kEplSuccessful;

Exit:
    if (instance_l.semUserData != SEM_FAILED)
        sem_close(instance_l.semUserData);

    if (instance_l.semKernelData != SEM_FAILED)
        sem_close(instance_l.semKernelData);

    eventkcal_exitQueueCircbuf(kEventQueueK2U);
    eventkcal_exitQueueCircbuf(kEventQueueU2K);
    eventkcal_exitQueueCircbuf(kEventQueueKInt);

    return kEplNoResource;
}


//------------------------------------------------------------------------------
/**
\brief    Cleanup kernel event CAL module

The function cleans up the kernel event CAL module. For cleanup it calls the exit
functions of the queue implementations for each used queue.

\return The function returns a tEplKernel error code.
\retval kEplSuccessful          If function executes correctly
\retval other error codes       If an error occurred

\ingroup module_eventkcal
*/
//------------------------------------------------------------------------------
tEplKernel eventkcal_exit (void)
{
    UINT             i = 0;

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

        eventkcal_exitQueueCircbuf(kEventQueueK2U);
        eventkcal_exitQueueCircbuf(kEventQueueU2K);
        eventkcal_exitQueueCircbuf(kEventQueueKInt);

        sem_close(instance_l.semUserData);
        sem_close(instance_l.semKernelData);
    }
    instance_l.fInitialized = FALSE;

    return kEplSuccessful;
}

//------------------------------------------------------------------------------
/**
\brief    Post kernel event

This function posts a event to the kernel queue.

\param  pEvent_p                Event to be posted.

\return The function returns a tEplKernel error code.
\retval kEplSuccessful          If function executes correctly
\retval other error codes       If an error occurred

\ingroup module_eventkcal
*/
//------------------------------------------------------------------------------
tEplKernel eventkcal_postKernelEvent (tEplEvent *pEvent_p)
{
    tEplKernel      ret = kEplSuccessful;

    ret = eventkcal_postEventCircbuf(kEventQueueKInt, pEvent_p);

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief    Post user event

This function posts a event to the user queue.

\param  pEvent_p                Event to be posted.

\return The function returns a tEplKernel error code.
\retval kEplSuccessful          If function executes correctly
\retval other error codes       If an error occurred

\ingroup module_eventkcal
*/
//------------------------------------------------------------------------------
tEplKernel eventkcal_postUserEvent (tEplEvent *pEvent_p)
{
    tEplKernel      ret = kEplSuccessful;

    ret = eventkcal_postEventCircbuf(kEventQueueK2U, pEvent_p);

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Process function of kernel CAL module

This function will be called by the systems process function.
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

\param  arg                     Thread parameter. Not used!

\return The function returns the thread exit code.
*/
//------------------------------------------------------------------------------
static void * eventThread(void *arg)
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
