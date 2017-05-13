/**
********************************************************************************
\file   timesyncucal-winioctl.c

\brief  Sync implementation for the user CAL timesync module using Windows IOCTL

This files implements the user CAL timesync module for synchronization using
Windows IOCTL for communication between user and kernel layer of the stack.

\ingroup module_timesyncucal
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2015, Kalycito Infotech Private Limited
Copyright (c) 2016, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
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
#include <user/timesyncucal.h>
#include <user/ctrlucal.h>

#include <common/driver.h>

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
\brief Local instance for PDO synchronization module

The structure holds the global parameters for PDO synchronization module.
*/
typedef struct
{
    OPLK_FILE_HANDLE    hGlobalFileHandle;        ///< Global file handle to POWERLINK driver
    HANDLE              hSyncFileHandle;          ///< File handle to driver for synchronization
    BOOL                fIntialized;              ///< Flag to mark module initialization
} tTimesyncuCalInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tTimesyncuCalInstance timesyncuInstance_l;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Initialize user CAL timesync module

The function initializes the user CAL timesync module

\param[in]      pfnSyncCb_p         Function that is called in case of sync event

\return The function returns a tOplkError error code.

\ingroup module_timesyncucal
*/
//------------------------------------------------------------------------------
tOplkError timesyncucal_init(tSyncCb pfnSyncCb_p)
{
    UINT    errNum;

    UNUSED_PARAMETER(pfnSyncCb_p);

    timesyncuInstance_l.hGlobalFileHandle = ctrlucal_getFd();
    timesyncuInstance_l.hSyncFileHandle = CreateFile(PLK_DEV_FILE,                        // Name of the NT "device" to open
                                                     GENERIC_READ | GENERIC_WRITE,        // Access rights requested
                                                     FILE_SHARE_READ | FILE_SHARE_WRITE,  // Share access - NONE
                                                     NULL,                                // Security attributes - not used!
                                                     OPEN_EXISTING,                       // Device must exist to open it.
                                                     FILE_ATTRIBUTE_NORMAL,               // Open for overlapped I/O
                                                     NULL);

    if (timesyncuInstance_l.hSyncFileHandle == INVALID_HANDLE_VALUE)
    {
        errNum = GetLastError();

        if ((errNum != ERROR_FILE_NOT_FOUND) &&
            (errNum != ERROR_PATH_NOT_FOUND))
        {
            DEBUG_LVL_ERROR_TRACE("%s() createFile failed!  ERROR_FILE_NOT_FOUND = %d\n",
                                  __func__,
                                  errNum);
            return kErrorNoResource;
        }
    }

    timesyncuInstance_l.fIntialized = TRUE;
    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Clean up user CAL timesync module

The function cleans up the user CAL timesync module

\ingroup module_timesyncucal
*/
//------------------------------------------------------------------------------
void timesyncucal_exit(void)
{
    ULONG    bytesReturned;

    // Clean resources acquired in kernel driver for synchronization.
    if (!DeviceIoControl(timesyncuInstance_l.hGlobalFileHandle,
                         PLK_CMD_CLEAN,
                         0,
                         0,
                         0,
                         0,
                         &bytesReturned,
                         NULL))
    {
        DEBUG_LVL_ERROR_TRACE("%s():Error in IOCTL %d\n", __func__, GetLastError());
    }

    CloseHandle(timesyncuInstance_l.hSyncFileHandle);

    timesyncuInstance_l.fIntialized = FALSE;
}

//------------------------------------------------------------------------------
/**
\brief  Wait for a sync event

The function waits for a sync event.

\param[in]      timeout_p           Specifies a timeout in microseconds. If 0 it waits
                                    forever.

\return The function returns a tOplkError error code.
\retval kErrorOk                    Successfully received sync event
\retval kErrorGeneralError          Error while waiting on sync event

\ingroup module_timesyncucal
*/
//------------------------------------------------------------------------------
tOplkError timesyncucal_waitSyncEvent(ULONG timeout_p)
{
    ULONG    bytesReturned;

    if (!timesyncuInstance_l.fIntialized)
        return kErrorNoResource;

    if (!DeviceIoControl(timesyncuInstance_l.hSyncFileHandle,
                         PLK_CMD_TIMESYNC_SYNC,
                         &timeout_p,
                         sizeof(ULONG),
                         0,
                         0,
                         &bytesReturned,
                         NULL))
    {
        DEBUG_LVL_ERROR_TRACE("%s():Error in IOCTL %d\n", __func__, GetLastError());
        return kErrorGeneralError;
    }

    return kErrorOk;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

/// \}
