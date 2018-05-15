/**
********************************************************************************
\file   eventucal-winioctl.c

\brief  User event CAL module for IOCTL interface on Windows

This file implements the user event CAL module for Windows kernel interface
which uses Windows IOCTL calls for communication.

The event user module uses the circular buffer library interface for the creation
and management for event queues. Separate thread routines to process events
are added to process events in background.

\ingroup module_eventucal
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2016, B&R Industrial Automation GmbH
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
#include <oplk/debugstr.h>
#include <user/eventucal.h>
#include <user/eventu.h>
#include <common/target.h>

#include <user/ctrlucal.h>
#include <common/driver.h>

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
    HANDLE    hSendFileHandle;          ///< Handle to driver for sending events
    HANDLE    hRcvFileHandle;           ///< Handle to driver for receiving events
    HANDLE    hThreadHandle;            ///< Handle to the event thread
    BOOL      fStopThread;              ///< Flag to identify thread exit
    DWORD     threadId;                 ///< Thread ID of the event thread
} tEventuCalInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tEventuCalInstance    instance_l;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static DWORD WINAPI eventThread(void* pArg_p);
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
\retval kErrorOk                    Function executes correctly
\retval other error codes           An error occurred

\ingroup module_eventucal
*/
//------------------------------------------------------------------------------
tOplkError eventucal_init(void)
{
    OPLK_MEMSET(&instance_l, 0, sizeof(tEventuCalInstance));

    instance_l.hSendFileHandle = ctrlucal_getFd();
    instance_l.fStopThread = FALSE;

    instance_l.hThreadHandle = CreateThread(NULL,                 // Default security attributes
                                           0,                     // Use Default stack size
                                           eventThread,           // Thread routine
                                           NULL,                  // Argument to the thread routine
                                           0,                     // Use default creation flags
                                           &instance_l.threadId   // Returned thread Id
                                           );

    if (instance_l.hThreadHandle == NULL)
    {
        DEBUG_LVL_ERROR_TRACE("%s() Failed to create event thread with error: 0x%X\n",
                              __func__,
                              GetLastError());
        return kErrorNoResource;
    }

    if (!SetThreadPriority(instance_l.hThreadHandle, THREAD_PRIORITY_TIME_CRITICAL))
    {
        DEBUG_LVL_ERROR_TRACE("%s() Failed to boost thread priority with error: 0x%X\n",
                              __func__,
                              GetLastError());
        return kErrorNoResource;
    }

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief    Clean up user event CAL module

The function cleans up the user event CAL module. For cleanup it calls the exit
functions of the queue implementations for each used queue.

\note The queues reside in the kernel layer and would be cleaned during
      the exit of file interface.

\return The function returns a tOplkError error code.
\retval kErrorOk                    Function executes correctly
\retval other error codes           An error occurred

\ingroup module_eventucal
*/
//------------------------------------------------------------------------------
tOplkError eventucal_exit(void)
{
    UINT    i = 0;

    instance_l.fStopThread = TRUE;
    while (instance_l.fStopThread != FALSE)
    {
        target_msleep(10);
        if (i++ > 1000)
        {
            DEBUG_LVL_EVENTU_TRACE("%s(): Event thread is not terminating, continue shutdown...!\n",
                                   __func__);
            break;
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

    return postEvent(pEvent_p);
}

//------------------------------------------------------------------------------
/**
\brief    Post kernel event

This function posts an event to a queue. It is called from the generic user
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

This function posts an event to a queue. It is called from the generic user
event post function in the event handler. Depending on the sink the appropriate
queue post function is called.

\param[in]      pEvent_p            Event to be posted.

\return The function returns a tOplkError error code.
\retval kErrorOk                    Function executes correctly
\retval other error codes           An error occurred
*/
//------------------------------------------------------------------------------
static tOplkError postEvent(const tEvent* pEvent_p)
{
    UINT8     eventBuf[sizeof(tEvent) + MAX_EVENT_ARG_SIZE];
    DWORD     eventBufSize = (DWORD)(sizeof(tEvent) + pEvent_p->eventArgSize);
    ULONG     bytesReturned;

    OPLK_MEMCPY(eventBuf, pEvent_p, sizeof(tEvent));
    OPLK_MEMCPY(eventBuf + sizeof(tEvent),
                pEvent_p->eventArg.pEventArg,
                pEvent_p->eventArgSize);

    if (!DeviceIoControl(instance_l.hSendFileHandle,
                         PLK_CMD_POST_EVENT,
                         eventBuf,
                         eventBufSize,
                         0,
                         0,
                         &bytesReturned,
                         NULL))
        return kErrorNoResource;

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief    Event thread function

This function implements the event thread. The thread uses IOCTLs to fetch the
user-internal and kernel-to-user events from kernel layer. The events fetched
are forwarded to the eventu module for processing.

\param[in,out]  pArg_p              Thread argument.

\return Returns system error code.
\retval 0, if function completed without errors.
\retval system error code, if function failed.

*/
//------------------------------------------------------------------------------
static DWORD WINAPI eventThread(void* pArg_p)
{
    tEvent*   pEvent;
    BOOL      ret;
    UINT8     eventBuf[sizeof(tEvent) + MAX_EVENT_ARG_SIZE];
    DWORD     eventBufSize = sizeof(tEvent) + MAX_EVENT_ARG_SIZE;
    ULONG     bytesReturned;
    UINT      errNum = 0;

    UNUSED_PARAMETER(pArg_p);

    instance_l.hRcvFileHandle = CreateFile(PLK_DEV_FILE,                        // Name of the NT "device" to open
                                           GENERIC_READ | GENERIC_WRITE,        // Access rights requested
                                           FILE_SHARE_READ | FILE_SHARE_WRITE,  // Share access - NONE
                                           NULL,                                // Security attributes - not used!
                                           OPEN_EXISTING,                       // Device must exist to open it.
                                           FILE_ATTRIBUTE_NORMAL,               // Open for overlapped I/O
                                           NULL);

    if (instance_l.hRcvFileHandle == INVALID_HANDLE_VALUE)
    {
        errNum = GetLastError();

        if (!(errNum == ERROR_FILE_NOT_FOUND ||
              errNum == ERROR_PATH_NOT_FOUND))
        {
            DEBUG_LVL_ERROR_TRACE("%s() createFile failed!  ERROR_FILE_NOT_FOUND = %d\n",
                                  __func__,
                                  errNum);
        }
        else
        {
            DEBUG_LVL_ERROR_TRACE("%s() createFile failed with error %d\n",
                                  __func__,
                                  errNum);
        }

        return kErrorNoResource;
    }

    pEvent = (tEvent*)eventBuf;

    while (!instance_l.fStopThread)
    {
        ret = DeviceIoControl(instance_l.hRcvFileHandle,
                              PLK_CMD_GET_EVENT,
                              NULL,
                              0,
                              eventBuf,
                              eventBufSize,
                              &bytesReturned,
                              NULL);
        if (!ret)
        {
            if (DEVICE_CLOSE_IO == GetLastError())
            {
                DEBUG_LVL_EVENTU_TRACE("%s(): Closing event thread\n", __func__);
            }
            else
            {
                DEBUG_LVL_ERROR_TRACE("%s(): Error in DeviceIoControl: %d\n",
                                      __func__,
                                      GetLastError());
            }
            break;
        }

        if (bytesReturned != 0)
        {
            DEBUG_LVL_EVENTU_TRACE("%s() User: got event type:%d(%s) sink:%d(%s)\n",
                                   __func__,
                                   pEvent->eventType,
                                   debugstr_getEventTypeStr(pEvent->eventType),
                                   pEvent->eventSink,
                                   debugstr_getEventSinkStr(pEvent->eventSink));
            if (pEvent->eventArgSize != 0)
                pEvent->eventArg.pEventArg = (UINT8*)pEvent + sizeof(tEvent);

            ret = eventu_process(pEvent);
        }
        else
        {
            DEBUG_LVL_EVENTU_TRACE("%s() ret = %d %d\n",
                                   __func__,
                                   ret,
                                   bytesReturned);
        }
    }

    CloseHandle(instance_l.hRcvFileHandle);
    instance_l.fStopThread = FALSE;

    return 0;
}

/// \}
