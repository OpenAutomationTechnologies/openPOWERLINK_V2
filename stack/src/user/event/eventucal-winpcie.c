/**
********************************************************************************
\file   eventucal-winpcie.c

\brief  User event CAL module for PCIe interface on Windows

This file implements the event CAL module for Windows userspace platform which uses
IOCTLs to communicate with the openPOWERLINK kernel layer running on an external PCIe.

The event user module uses the circular buffer library interface for the creation
and management of event queues. Separate user and kernel thread routines are used.
The kernel thread receives events from the kernel layer and posts them into the
user-internal event queue. The user thread processes all events received from
the user-internal event queue.

\ingroup module_eventucal
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2015, Kalycito Infotech Private Limited
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
#include <user/eventucal.h>
#include <user/eventucalintf.h>
#include <common/target.h>

#include <user/ctrlucal.h>
#include <common/driver.h>
#include <user/eventu.h>

//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#define DEVICE_CLOSE_IO    995

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
    OPLK_FILE_HANDLE    hSendfileHandle;               ///< Handle to driver for sending events
    HANDLE              hRcvfileHandle;                ///< Handle to driver for receiving events
    HANDLE              hEventProcThread;              ///< User event thread handle
    HANDLE              hKernelThread;                 ///< Kernel event thread handle
    HANDLE              hSemUserData;                  ///< Semaphore to synchronize main and user thread access
    BOOL                fStopKernelThread;             ///< Flag to synchronize kernel thread exit
    BOOL                fStopUserThread;               ///< Flag to synchronize user thread exit
    BOOL                fInitialized;                  ///< Flag to indicate module initialization
} tEventuCalInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tEventuCalInstance    instance_l;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static void         signalUserEvent(void);
static DWORD WINAPI eventProcess(LPVOID pArg_p);
static DWORD WINAPI kernelEventThread(LPVOID pArg_p);
static tOplkError   postEvent(const tEvent* pEvent_p);

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
\retval kErrorOk                    Function executed correctly
\retval other error codes           An error occurred

\ingroup module_eventucal
*/
//------------------------------------------------------------------------------
tOplkError eventucal_init(void)
{
    OPLK_MEMSET(&instance_l, 0, sizeof(tEventuCalInstance));

    instance_l.hSendfileHandle = ctrlucal_getFd();
    instance_l.fStopKernelThread = FALSE;
    instance_l.fStopUserThread = FALSE;

    if ((instance_l.hSemUserData = CreateSemaphore(NULL, 0, 100, "Local\\semUserEvent")) == NULL)
        goto Exit;

    if (eventucal_initQueueCircbuf(kEventQueueUInt) != kErrorOk)
        goto Exit;

    eventucal_setSignalingCircbuf(kEventQueueUInt, signalUserEvent);

    instance_l.hEventProcThread = CreateThread(NULL,         // Default security attributes
                                               0,            // Use Default stack size
                                               eventProcess, // Thread routine
                                               NULL,         // Argument to the thread routine
                                               0,            // Use default creation flags
                                               NULL          // Returned thread Id
                                               );

    if (instance_l.hEventProcThread == NULL)
    {
        DEBUG_LVL_ERROR_TRACE("%s() Failed to create event Process thread with error: 0x%X\n",
                              __func__,
                              GetLastError());
        goto Exit;
    }

    instance_l.hKernelThread = CreateThread(NULL,               // Default security attributes
                                            0,                  // Use Default stack size
                                            kernelEventThread,  // Thread routine
                                            NULL,               // Argument to the thread routine
                                            0,                  // Use default creation flags
                                            NULL                // Returned thread Id
                                            );

    if (instance_l.hKernelThread == NULL)
    {
        DEBUG_LVL_ERROR_TRACE("%s() Failed to create kernel thread with error: 0x%X\n",
                              __func__,
                              GetLastError());
        goto Exit;
    }

    instance_l.fInitialized = TRUE;
    return kErrorOk;

Exit:
    eventucal_exit();

    return kErrorNoResource;

}

//------------------------------------------------------------------------------
/**
\brief    Clean up user event CAL module

The function cleans up the user event CAL module. For clean-up it calls the exit
functions of the queue implementations for each used queue.

\return The function returns a tOplkError error code.
\retval kErrorOk                    Function executes correctly
\retval other error codes           An error occurred

\ingroup module_eventucal
*/
//------------------------------------------------------------------------------
tOplkError eventucal_exit(void)
{
    DWORD   waitResult;

    if (instance_l.fInitialized)
    {
        instance_l.fStopKernelThread = TRUE;
        instance_l.fStopUserThread = TRUE;

        ReleaseSemaphore(instance_l.hSemUserData, 1, NULL);

        waitResult = WaitForSingleObject(instance_l.hKernelThread, 10000);
        if (waitResult == WAIT_TIMEOUT)
        {
            DEBUG_LVL_EVENTU_TRACE("%s(): Kernel event thread is not terminating, continue shutdown...!\n",
                                   __func__);
        }

        waitResult = WaitForSingleObject(instance_l.hEventProcThread, 10000);
        if (waitResult == WAIT_TIMEOUT)
        {
            DEBUG_LVL_EVENTU_TRACE("%s(): User event thread is not terminating, continue shutdown...!\n",
                                   __func__);
        }
    }

    eventucal_exitQueueCircbuf(kEventQueueUInt);

    if (instance_l.hSemUserData != NULL)
        CloseHandle(instance_l.hSemUserData);

    if (instance_l.hKernelThread != NULL)
        CloseHandle(instance_l.hKernelThread);

    if (instance_l.hEventProcThread != NULL)
        CloseHandle(instance_l.hEventProcThread);

    instance_l.fInitialized = FALSE;

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief    Post user event

This function posts an event to the user internal queue. It is called from the
generic user event post function in the event handler.

\param[in]      pEvent_p            Event to be posted.

\return The function returns a tOplkError error code.
\retval kErrorOk                    Function executes correctly
\retval other error codes           An error occurred

\ingroup module_eventucal
*/
//------------------------------------------------------------------------------
tOplkError eventucal_postUserEvent(const tEvent* pEvent_p)
{
    // Check parameter validity
    ASSERT(pEvent_p != NULL);

    return eventucal_postEventCircbuf(kEventQueueUInt, pEvent_p);
}

//------------------------------------------------------------------------------
/**
\brief    Post kernel event

This function posts an event to the kernel stack. It is called from the
generic user event post function in the event handler.

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

This function posts an event to the user to kernel queue. The function uses IOCTL
to forward the event to the kernel driver which then writes it into the
user to kernel queue.

\param[in]      pEvent_p            Event to be posted.

\return The function returns a tOplkError error code.
\retval kErrorOk                    Function executes correctly
\retval other error codes           An error occurred
*/
//------------------------------------------------------------------------------
static tOplkError postEvent(const tEvent* pEvent_p)
{
    UINT8       aEventBuf[sizeof(tEvent) + MAX_EVENT_ARG_SIZE];
    UINT32      eventBufSize = sizeof(tEvent) + pEvent_p->eventArgSize;
    ULONG       bytesReturned;
    BOOL        fIoctlRet;

    OPLK_MEMCPY(aEventBuf, pEvent_p, sizeof(tEvent));

    if (pEvent_p->eventArgSize != 0)
    {
        OPLK_MEMCPY((aEventBuf + sizeof(tEvent)),
                    pEvent_p->eventArg.pEventArg,
                    pEvent_p->eventArgSize);
    }

    fIoctlRet = DeviceIoControl(instance_l.hSendfileHandle,
                                PLK_CMD_POST_EVENT,
                                aEventBuf,
                                eventBufSize,
                                0,
                                0,
                                &bytesReturned,
                                NULL);
    if (!fIoctlRet || (bytesReturned == 0))
    {
        DEBUG_LVL_ERROR_TRACE("%s() Failed to post event: error (0x%X)\n",
                              __func__,
                              GetLastError());
        return kErrorNoResource;
    }

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  User event handler thread function

This function contains the main function for the user event handler thread.

\param[in,out]  pArg_p              Thread parameter. Not used!

\return The function returns the thread exit code.

*/
//------------------------------------------------------------------------------
static DWORD WINAPI eventProcess(LPVOID pArg_p)
{
    DWORD   waitResult;

    UNUSED_PARAMETER(pArg_p);

    if (!SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_TIME_CRITICAL))
    {
        DEBUG_LVL_ERROR_TRACE("%s(): Failed to boost thread priority with error: 0x%X\n",
                              __func__,
                              GetLastError());
        DEBUG_LVL_ERROR_TRACE("%s(): The thread will execute with normal priority\n",
                              __func__);
    }

    DEBUG_LVL_EVENTU_TRACE("%s(): User event thread %d waiting for events...\n",
                           __func__,
                           GetCurrentThreadId());

    while (!instance_l.fStopUserThread)
    {
        waitResult = WaitForSingleObject(instance_l.hSemUserData, 5000);
        switch (waitResult)
        {
            case WAIT_OBJECT_0:
                if (eventucal_getEventCountCircbuf(kEventQueueUInt) > 0)
                    eventucal_processEventCircbuf(kEventQueueUInt);
                break;

            case WAIT_TIMEOUT:
                break;

            default:
                DEBUG_LVL_ERROR_TRACE("%s(): Semaphore wait unknown error! Error:(0x%X)\n",
                                      __func__,
                                      GetLastError());
                if (GetLastError() == ERROR_INVALID_HANDLE)
                    instance_l.fStopUserThread = TRUE;
                break;
        }
    }

    instance_l.fStopUserThread = FALSE;
    DEBUG_LVL_EVENTU_TRACE("%s(): User event thread is exiting!\n", __func__);
    return 0;

}

//------------------------------------------------------------------------------
/**
\brief  Kernel event handler thread function

This function contains the main function for the kernel event handler thread.

\param[in,out]  pArg_p              Thread parameter. Not used!

\return The function returns the thread exit code.

*/
//------------------------------------------------------------------------------
static DWORD WINAPI kernelEventThread(LPVOID pArg_p)
{
    UINT8       aEventBuf[sizeof(tEvent) + MAX_EVENT_ARG_SIZE];
    tEvent*     pEvent = (tEvent*)aEventBuf;
    BOOL        fIoctlRet;
    UINT32      eventBufSize = sizeof(aEventBuf);
    ULONG       bytesReturned;
    UINT        errNum = 0;

    UNUSED_PARAMETER(pArg_p);

    if (!SetThreadPriority(GetCurrentThread(), (THREAD_PRIORITY_TIME_CRITICAL - 1)))
    {
        DEBUG_LVL_ERROR_TRACE("%s(): Failed to boost thread priority with error: 0x%X\n",
                              __func__,
                              GetLastError());
        DEBUG_LVL_ERROR_TRACE("%s(): The thread will execute with normal priority\n",
                              __func__);
    }

    instance_l.hRcvfileHandle = CreateFile(PLK_DEV_FILE,                        // Name of the NT "device" to open
                                           GENERIC_READ | GENERIC_WRITE,        // Access rights requested
                                           FILE_SHARE_READ | FILE_SHARE_WRITE,  // Share access - NONE
                                           NULL,                                // Security attributes - not used!
                                           OPEN_EXISTING,                       // Device must exist to open it.
                                           FILE_ATTRIBUTE_NORMAL,               // Open for overlapped I/O
                                           NULL);

    if (instance_l.hRcvfileHandle == INVALID_HANDLE_VALUE)
    {
        errNum = GetLastError();

        if (!((errNum == ERROR_FILE_NOT_FOUND) || (errNum == ERROR_PATH_NOT_FOUND)))
        {
            DEBUG_LVL_ERROR_TRACE("%s() createFile failed!  ERROR_FILE_NOT_FOUND = %d\n",
                                  __func__,
                                  errNum);
        }
        else
        {
            DEBUG_LVL_ERROR_TRACE("%s() createFile failed with error = %d\n",
                                  __func__,
                                  errNum);
        }
        return kErrorNoResource;
    }

    while (!instance_l.fStopKernelThread)
    {
        target_msleep(1);
        fIoctlRet = DeviceIoControl(instance_l.hRcvfileHandle,
                                    PLK_CMD_GET_EVENT,
                                    NULL,
                                    0,
                                    aEventBuf,
                                    eventBufSize,
                                    &bytesReturned,
                                    NULL);
        if (!fIoctlRet)
        {
            if (GetLastError() == DEVICE_CLOSE_IO)
            {
                DEBUG_LVL_EVENTU_TRACE("Closing event thread\n");
            }
            else
            {
                DEBUG_LVL_ERROR_TRACE("%s():Error in DeviceIoControl : %d\n", GetLastError());
            }
            break;
        }

        if (bytesReturned > 0)
        {
            if (pEvent->eventArgSize == 0)
                pEvent->eventArg.pEventArg = NULL;
            else
                pEvent->eventArg.pEventArg = (void*)((UINT8*)pEvent + sizeof(tEvent));

            eventucal_postEventCircbuf(kEventQueueUInt, pEvent);
        }
    }

    CloseHandle(instance_l.hRcvfileHandle);
    instance_l.fStopKernelThread = FALSE;

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
    if (!ReleaseSemaphore(instance_l.hSemUserData, 1, NULL))
    {
        DEBUG_LVL_ERROR_TRACE("%s() Failed to signal user event (0x%X)\n",
                              __func__,
                              GetLastError());
    }
}

/// \}
