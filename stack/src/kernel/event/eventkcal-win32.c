/**
********************************************************************************
\file   eventkcal-win32.c

\brief  Kernel event CAL module for Windows

This file implements the kernel event handler CAL module for the Windows
platform. It uses the circular buffer library for all event queues.

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
    HANDLE                  threadHandle;
    HANDLE                  semUserData;
    HANDLE                  semKernelData;
    BOOL                    fInitialized;
    BOOL                    fStopThread;
} tEventkCalInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tEventkCalInstance   instance_l;             ///< Instance variable of kernel event CAL module

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static DWORD WINAPI eventThread(LPVOID arg);
static void signalKernelEvent(void);
static void signalUserEvent(void);

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief    Initialize kernel event CAL module

The function initializes the kernel event CAL module on Windows.

\return The function returns a tOplkError error code.
\retval kErrorOk                    Function executes correctly
\retval other error codes           An error occurred

\ingroup module_eventkcal
*/
//------------------------------------------------------------------------------
tOplkError eventkcal_init(void)
{
    OPLK_MEMSET(&instance_l, 0, sizeof(tEventkCalInstance));

    if ((instance_l.semUserData = CreateSemaphore(NULL, 0, 100, "Local\\semUserEvent")) == NULL)
        goto Exit;

    if ((instance_l.semKernelData = CreateSemaphore(NULL, 0, 100, "Local\\semKernelEvent")) == NULL)
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
    instance_l.threadHandle = CreateThread(NULL,
                                           0,
                                           eventThread,
                                           (LPVOID)&instance_l,
                                           0,
                                           NULL);
    if (instance_l.threadHandle == NULL)
        goto Exit;

    SetThreadPriority(instance_l.threadHandle, THREAD_PRIORITY_ABOVE_NORMAL);

    instance_l.fInitialized = TRUE;
    return kErrorOk;

Exit:

    if (instance_l.semKernelData != NULL)
        CloseHandle(instance_l.semKernelData);

    if (instance_l.semUserData != NULL)
        CloseHandle(instance_l.semUserData);

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
    if (instance_l.fInitialized != FALSE)
    {
        instance_l.fStopThread = TRUE;

        // jba wait for thread to exit!

        eventkcal_exitQueueCircbuf(kEventQueueK2U);
        eventkcal_exitQueueCircbuf(kEventQueueU2K);
        eventkcal_exitQueueCircbuf(kEventQueueKInt);

        CloseHandle(instance_l.semKernelData);
        CloseHandle(instance_l.semUserData);
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

    // Check parameter validity
    ASSERT(pEvent_p != NULL);

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

    // Check parameter validity
    ASSERT(pEvent_p != NULL);

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

\param[in]      arg                 Thread parameter. Used to get the instance structure.

\return The function returns the thread exit code.
*/
//------------------------------------------------------------------------------
static DWORD WINAPI eventThread(LPVOID arg)
{
    const tEventkCalInstance*   pInstance = (const tEventkCalInstance*)arg;
    DWORD                       waitResult;

    DEBUG_LVL_EVENTK_TRACE("Kernel event thread %d waiting for events...\n", GetCurrentThreadId());
    while (!pInstance->fStopThread)
    {
        waitResult = WaitForSingleObject(pInstance->semKernelData, 100);
        switch (waitResult)
        {
            case WAIT_OBJECT_0:
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
                break;

            case WAIT_TIMEOUT:
                DEBUG_LVL_ERROR_TRACE("kernel event timeout!\n");
                break;

            default:
                DEBUG_LVL_ERROR_TRACE("%s() Semaphore wait unknown error! Error:%ld\n",
                                      __func__,
                                      GetLastError());
                break;
        }
    }

    DEBUG_LVL_EVENTK_TRACE("Kernel event thread is exiting!\n");
    return 0;
}

//------------------------------------------------------------------------------
/**
\brief  Signal a user event

This function signals that a user event was posted. It will be registered in
the circular buffer library as signal callback function.
*/
//------------------------------------------------------------------------------
static void signalUserEvent(void)
{
    ReleaseSemaphore(instance_l.semUserData, 1, NULL);
}

//------------------------------------------------------------------------------
/**
\brief  Signal a kernel event

This function signals that a kernel event was posted. It will be registered in
the circular buffer library as signal callback function.
*/
//------------------------------------------------------------------------------
static void signalKernelEvent(void)
{
    ReleaseSemaphore(instance_l.semKernelData, 1, NULL);
}

/// \}
