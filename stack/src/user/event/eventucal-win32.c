/**
********************************************************************************
\file   eventucal-win32.c

\brief  User event CAL module for Windows

This file implements the user event handler CAL module for the Windows
platform. It uses the circular buffer interface for all event queues.

\see eventucalintf-circbuf.c

\ingroup module_eventucal
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
#include <user/eventucal.h>
#include <user/eventucalintf.h>

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
\brief User event CAL instance type

The structure contains all necessary information needed by the user event
CAL module.
*/
typedef struct
{
    HANDLE                  threadHandle;
    HANDLE                  semUserData;
    HANDLE                  semKernelData;
    BOOL                    fInitialized;
    BOOL                    fStopThread;
} tEventuCalInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
tEventuCalInstance          instance_l;             ///< Instance variable of user event CAL module

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static DWORD WINAPI eventThread(LPVOID arg);
static void         signalUserEvent(void);
static void         signalKernelEvent(void);

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

\ingroup module_eventucal
*/
//------------------------------------------------------------------------------
tOplkError eventucal_init(void)
{
    OPLK_MEMSET(&instance_l, 0, sizeof(tEventuCalInstance));

    if ((instance_l.semUserData = CreateSemaphore(NULL, 0, 100, "Local\\semUserEvent")) == NULL)
        goto Exit;

    if ((instance_l.semKernelData = CreateSemaphore(NULL, 0, 100, "Local\\semKernelEvent")) == NULL)
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
    if ((instance_l.threadHandle = CreateThread(NULL,
                                                0,
                                                eventThread,
                                                (LPVOID)&instance_l,
                                                0,
                                                NULL)) == NULL)
    {
        DEBUG_LVL_ERROR_TRACE("%s() CreateThread fails! Error:%ld\n",
                              __func__,
                              GetLastError());
        goto Exit;
    }


    // jba set thread priority!!!


    instance_l.fInitialized = TRUE;
    return kErrorOk;

Exit:
    if (instance_l.semUserData != NULL)
        CloseHandle(instance_l.semUserData);

    if (instance_l.semKernelData != NULL)
        CloseHandle(instance_l.semKernelData);

    eventucal_exitQueueCircbuf(kEventQueueK2U);
    eventucal_exitQueueCircbuf(kEventQueueU2K);
    eventucal_exitQueueCircbuf(kEventQueueUInt);

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

\ingroup module_eventucal
*/
//------------------------------------------------------------------------------
tOplkError eventucal_exit(void)
{
    if (instance_l.fInitialized != FALSE)
    {
        instance_l.fStopThread = TRUE;

        // jba wait for thread to exit!

        eventucal_exitQueueCircbuf(kEventQueueK2U);
        eventucal_exitQueueCircbuf(kEventQueueU2K);
        eventucal_exitQueueCircbuf(kEventQueueUInt);

        CloseHandle(instance_l.semUserData);
        CloseHandle(instance_l.semKernelData);
    }
    instance_l.fInitialized = FALSE;

    return kErrorOk;
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

\ingroup module_eventucal
*/
//------------------------------------------------------------------------------
tOplkError eventucal_postKernelEvent(const tEvent* pEvent_p)
{
    tOplkError  ret;

    // Check parameter validity
    ASSERT(pEvent_p != NULL);

    ret = eventucal_postEventCircbuf(kEventQueueU2K, pEvent_p);

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief    Post user event

This function posts an event to a queue. It is called from the generic kernel
event post function in the event handler. Depending on the sink the appropriate
queue post function is called.

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

\param[in,out]  arg                 Thread parameter. Used to access the module instance.

\return The function returns the thread exit code.
*/
//------------------------------------------------------------------------------
static DWORD WINAPI eventThread(LPVOID arg)
{
    tEventuCalInstance*     pInstance = (tEventuCalInstance*)arg;
    DWORD                   waitResult;

    DEBUG_LVL_EVENTU_TRACE("%s(): User event thread %d waiting for events...\n",
                           __func__,
                           GetCurrentThreadId());

    while (!pInstance->fStopThread)
    {
        waitResult = WaitForSingleObject(pInstance->semUserData, 5000);
        switch (waitResult)
        {
            case WAIT_OBJECT_0:
                DEBUG_LVL_EVENTU_TRACE("%s(): Received user event!\n", __func__);

                /* first handle all kernel to user events --> higher priority! */
                if (eventucal_getEventCountCircbuf(kEventQueueK2U) > 0)
                    eventucal_processEventCircbuf(kEventQueueK2U);
                else
                {
                    if (eventucal_getEventCountCircbuf(kEventQueueUInt) > 0)
                        eventucal_processEventCircbuf(kEventQueueUInt);
                }
                break;

            case WAIT_TIMEOUT:
                break;

            default:
                DEBUG_LVL_ERROR_TRACE("%s() Semaphore wait unknown error! Error:%ld\n",
                                      __func__,
                                      GetLastError());
                break;
        }
    }

    DEBUG_LVL_EVENTU_TRACE("%s(): User event thread is exiting!\n", __func__);
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
    ReleaseSemaphore(instance_l.semUserData, 1, NULL);
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
    ReleaseSemaphore(instance_l.semKernelData, 1, NULL);
}

/// \}
