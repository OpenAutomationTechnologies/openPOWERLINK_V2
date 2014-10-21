/**
********************************************************************************
\file   system-windows.c

\brief  Sytem specific functions for Windows

The file implements the system specific funtions for Windows used by the
openPOWERLINK demo applications.

\ingroup module_app_common
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2014, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
Copyright (c) 2013, SYSTEC electronic GmbH
Copyright (c) 2013, Kalycito Infotech Private Ltd.All rights reserved.
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
#define _WIN32_WINNT 0x0501     // Windows version must be at least Windows XP
#define WIN32_LEAN_AND_MEAN     // Do not use extended Win32 API functions
#include <Windows.h>

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

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Initialize system

The function initializes important stuff on the system for openPOWERLINK to
work correctly.

\ingroup module_app_common
*/
//------------------------------------------------------------------------------
int initSystem(void)
{
    // activate realtime priority class
    SetPriorityClass(GetCurrentProcess(), REALTIME_PRIORITY_CLASS);
    // lower the priority of this thread
    SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_IDLE);

    return 0;
}

//------------------------------------------------------------------------------
/**
\brief  Shutdown system

The function shuts-down the system.

\ingroup module_app_common
*/
//------------------------------------------------------------------------------
void shutdownSystem(void)
{

}

#if defined(CONFIG_USE_SYNCTHREAD)
//------------------------------------------------------------------------------
/**
\brief  Start synchronous data thread

The function starts the thread used for synchronous data handling.

\param  pfnSync_p           Pointer to sync callback function

\note   Currently not implemented for Windows!

\ingroup module_app_common
*/
//------------------------------------------------------------------------------
void startSyncThread(tSyncCb pfnSync_p)
{
    // Currently threads are not used on Windows
    UNUSED_PARAMETER(pfnSync_p);
}

//------------------------------------------------------------------------------
/**
\brief  Stop synchronous data thread

The function stops the thread used for synchronous data handling.

\ingroup module_app_common
*/
//------------------------------------------------------------------------------
void stopSyncThread(void)
{
    // Currently threads are not used on Windows
}
#endif


//------------------------------------------------------------------------------
/**
\brief  Return true if a termination signal has been received

The function can be used by the application to react on termination request.
On Windows, this function only implemented as a stub.

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

\param  milliSeconds_p      Number of milliseconds to sleep

\ingroup module_target
*/
//------------------------------------------------------------------------------
void msleep(unsigned int milliSeconds_p)
{
    Sleep(milliSeconds_p);
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

///\}

