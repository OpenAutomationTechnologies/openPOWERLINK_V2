/**
********************************************************************************
\file   system-windows.c

\brief  System specific functions for Windows

The file implements the system specific functions for Windows used by the
openPOWERLINK demo applications.

\ingroup module_app_common
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2016, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
Copyright (c) 2013, SYSTEC electronic GmbH
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
#define _WIN32_WINNT            0x0501  // Windows version must be at least Windows XP
#define WIN32_LEAN_AND_MEAN             // Do not use extended Win32 API functions
#include <Windows.h>

#include "system.h"

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
#if defined(CONFIG_USE_SYNCTHREAD)
/**
\brief  Local instance for synchronization thread

This structure contains local variables used by the synchronization thread.
*/
typedef struct
{
    HANDLE      hSyncThreadHandle;      ///< Synchronization thread handle
    tSyncCb     pfnSyncCb;              ///< Pointer to synchronization callback routine
    BOOL        fThreadExit;            ///< Flag to communicate with main thread
} tSyncThreadInstance;
#endif

/**
\brief  Local instance for firmware manager thread

This structure contains local variables used by the firmware manager thread.
*/
typedef struct
{
    HANDLE                      hThreadHandle;  ///< Firmware manager thread handle
    tFirmwareManagerThreadCb    pfnFwmCb;       ///< Pointer to firmware manager thread callback routine
    BOOL                        fThreadExit;    ///< Flag to communicate with main thread
    unsigned int                interval;       ///< Firmware manager thread call interval
} tFirmwareManagerThreadInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
#if defined(CONFIG_USE_SYNCTHREAD)
static tSyncThreadInstance  syncThreadInstance_l;
#endif

static tFirmwareManagerThreadInstance   fwmThreadInstance_l;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
#if defined(CONFIG_USE_SYNCTHREAD)
static DWORD WINAPI syncThread(LPVOID pArg_p);
#endif
static DWORD WINAPI fwmThread(LPVOID pArg_p);

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Initialize system

The function initializes important stuff on the system for openPOWERLINK to
work correctly.

\return The function returns 0 if the initialization has been successful,
        otherwise -1.

\ingroup module_app_common
*/
//------------------------------------------------------------------------------
int system_init(void)
{
    // activate realtime priority class
    SetPriorityClass(GetCurrentProcess(), REALTIME_PRIORITY_CLASS);
    // lower the priority of this thread
    SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_IDLE);

#if defined(CONFIG_USE_SYNCTHREAD)
    syncThreadInstance_l.fThreadExit = FALSE;
#endif

    return 0;
}

//------------------------------------------------------------------------------
/**
\brief  Shutdown system

The function shuts down the system.

\ingroup module_app_common
*/
//------------------------------------------------------------------------------
void system_exit(void)
{
#if defined(CONFIG_USE_SYNCTHREAD)
    CloseHandle(syncThreadInstance_l.hSyncThreadHandle);
#endif
}

/**
\brief  Determines whether a termination signal has been received

The function can be used by the application to react on termination request.
On Windows, this function is only implemented as a stub.


\ingroup module_app_common
*/
//------------------------------------------------------------------------------
BOOL system_getTermSignalState(void)
{
    return FALSE;
}

//------------------------------------------------------------------------------
/**
\brief Sleep for the specified number of milliseconds

The function makes the calling thread sleep until the number of specified
milliseconds have elapsed.

\param[in]      milliSeconds_p      Number of milliseconds to sleep

\ingroup module_app_common
*/
//------------------------------------------------------------------------------
void system_msleep(unsigned int milliSeconds_p)
{
    Sleep(milliSeconds_p);
}

#if defined(CONFIG_USE_SYNCTHREAD)
//------------------------------------------------------------------------------
/**
\brief  Start synchronous data thread

The function starts the thread used for synchronous data handling.

\param[in]      pfnSync_p           Pointer to sync callback function

\note   Currently not implemented for Windows!

\ingroup module_app_common
*/
//------------------------------------------------------------------------------
void system_startSyncThread(tSyncCb pfnSync_p)
{
    // Currently threads are not used on Windows
    syncThreadInstance_l.pfnSyncCb = pfnSync_p;

    syncThreadInstance_l.hSyncThreadHandle = CreateThread(NULL,          // Default security attributes
                                                          0,             // Use Default stack size
                                                          syncThread,    // Thread routine
                                                          NULL,          // Argument to the thread routine
                                                          0,             // Use default creation flags
                                                          NULL           // Returned thread Id
                                                          );
}

//------------------------------------------------------------------------------
/**
\brief  Stop synchronous data thread

The function stops the thread used for synchronous data handling.

\ingroup module_app_common
*/
//------------------------------------------------------------------------------
void system_stopSyncThread(void)
{
    CancelIoEx(syncThreadInstance_l.hSyncThreadHandle, NULL);

    // Signal to stop the thread and wait for the thread to terminate
    syncThreadInstance_l.fThreadExit = TRUE;
    WaitForSingleObject(syncThreadInstance_l.hSyncThreadHandle, 1000);
}
#endif

//------------------------------------------------------------------------------
/**
\brief  Start firmware manager thread

The function starts the thread used by the firmware manager.

\param[in]      pfnFwmThreadCb_p    Pointer to firmware manager callback
\param[in]      intervalSec_p       Thread execution interval in seconds

\ingroup module_app_common
*/
//------------------------------------------------------------------------------
void system_startFirmwareManagerThread(tFirmwareManagerThreadCb pfnFwmThreadCb_p,
                                       unsigned int intervalSec_p)
{
    fwmThreadInstance_l.pfnFwmCb = pfnFwmThreadCb_p;
    fwmThreadInstance_l.interval = intervalSec_p * 1000;
    fwmThreadInstance_l.fThreadExit = FALSE;

    fwmThreadInstance_l.hThreadHandle = CreateThread(NULL,          // Default security attributes
                                                     0,             // Use Default stack size
                                                     fwmThread,     // Thread routine
                                                     NULL,          // Argument to the thread routine
                                                     0,             // Use default creation flags
                                                     NULL           // Returned thread Id
                                                      );
}

//------------------------------------------------------------------------------
/**
\brief  Stop firmware manager thread

The function stops the thread used by the firmware manager.

\ingroup module_app_common
*/
//------------------------------------------------------------------------------
void system_stopFirmwareManagerThread(void)
{
    //TODO: Call CancelIoEx with thread handle as soon as XP support is dropped

    // Signal to stop the thread and wait for the thread to terminate
    fwmThreadInstance_l.fThreadExit = TRUE;
    WaitForSingleObject(fwmThreadInstance_l.hThreadHandle, 1000);
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

#if defined(CONFIG_USE_SYNCTHREAD)
//------------------------------------------------------------------------------
/**
\brief  Synchronous application thread

This function implements the synchronous application thread.

\param[in,out]  pArg_p              Thread parameter. Not used!

\return The function returns the thread exit code.
*/
//------------------------------------------------------------------------------
static DWORD WINAPI syncThread(LPVOID pArg_p)
{
    tOplkError  ret;

    UNUSED_PARAMETER(pArg_p);

    while (!syncThreadInstance_l.fThreadExit)
    {
        if (syncThreadInstance_l.pfnSyncCb)
        {
            ret = syncThreadInstance_l.pfnSyncCb();
            if (ret != kErrorOk)
                break;
        }
        else
            break;
    }

    printf("Exiting sync thread\n");

    return 0;
}
#endif

//------------------------------------------------------------------------------
/**
\brief  Firmware manager thread

This function implements the firmware manager thread.

\param[in,out]  pArg_p              Thread parameter. Not used!

\return The function returns the thread exit code.
*/
//------------------------------------------------------------------------------
static DWORD WINAPI fwmThread(LPVOID pArg_p)
{
    UNUSED_PARAMETER(pArg_p);

    while (!fwmThreadInstance_l.fThreadExit)
    {
        fwmThreadInstance_l.pfnFwmCb();

        Sleep(fwmThreadInstance_l.interval);
    }

    return 0;
}

/// \}
