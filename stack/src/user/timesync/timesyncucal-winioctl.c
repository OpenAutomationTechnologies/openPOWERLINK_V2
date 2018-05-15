/**
********************************************************************************
\file   timesyncucal-winioctl.c

\brief  Sync implementation for the user CAL timesync module using Windows IOCTL

This files implements the user CAL timesync module for synchronization using
Windows IOCTL for communication between user and kernel layer of the stack. In
addition, SoC timestamp forwarding feature implementation is done by creating
a shared memory for the user and kernel.

\ingroup module_timesyncucal
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2017, Kalycito Infotech Private Limited
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
    OPLK_FILE_HANDLE       hGlobalFileHandle;        ///< Global file handle to POWERLINK driver
    HANDLE                 hSyncFileHandle;          ///< File handle to driver for synchronization
    BOOL                   fIntialized;              ///< Flag to mark module initialization
#if defined(CONFIG_INCLUDE_SOC_TIME_FORWARD)
    tTimesyncSharedMemory* pSharedMemory;            ///< Pointer to timesync shared memory
#endif
} tTimesyncuCalInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tTimesyncuCalInstance timesyncuInstance_l;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
#if defined(CONFIG_INCLUDE_SOC_TIME_FORWARD)
static tOplkError getTimesyncShm(void);
static tOplkError releaseTimesyncShm(void);
#endif

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

#if defined(CONFIG_INCLUDE_SOC_TIME_FORWARD)
    if (getTimesyncShm() != kErrorOk)
    {
        DEBUG_LVL_ERROR_TRACE("%s() Couldn't get timesync shared memory!\n",
                              __func__);
        return kErrorNoResource;
    }
#endif

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

#if defined(CONFIG_INCLUDE_SOC_TIME_FORWARD)
    if (releaseTimesyncShm() != kErrorOk)
    {
        DEBUG_LVL_ERROR_TRACE("%s() SoC Timestamp shm could not be released\n",
                              __func__);
    }
#endif

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

#if defined(CONFIG_INCLUDE_SOC_TIME_FORWARD)
//------------------------------------------------------------------------------
/**
\brief  Get timesync shared memory

The function returns the reference to the timesync shared memory.

\return The function returns a pointer to the timesync shared memory.

\ingroup module_timesyncucal
*/
//------------------------------------------------------------------------------
tTimesyncSharedMemory* timesyncucal_getSharedMemory(void)
{
    return timesyncuInstance_l.pSharedMemory;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief  Get timesync shared memory

The function gets the available SoC timesync shared memory using
Windows IOCTL and remaps the shared memory into user space.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError getTimesyncShm(void)
{
    BOOL        fIoctlRet = FALSE;
    ULONG       bytesReturned = 0;
    tSocMem     inSocTimesyncMem;
    tSocMem     outSocTimesyncMem;
    tOplkError  ret = kErrorOk;
#if defined(CONFIG_PCIE)
    void*       pSocTimesyncMemOut;
#endif

    if (timesyncuInstance_l.hGlobalFileHandle == NULL)
        return kErrorNoResource;

    inSocTimesyncMem.socMemSize = sizeof(tTimesyncSharedMemory);

    fIoctlRet = DeviceIoControl(timesyncuInstance_l.hGlobalFileHandle,
                                PLK_CMD_SOC_GET_MEM,
                                &inSocTimesyncMem,
                                sizeof(tSocMem),
                                &outSocTimesyncMem,
                                sizeof(tSocMem),
                                &bytesReturned,
                                NULL);

    if (!fIoctlRet || (bytesReturned == 0) ||
        (outSocTimesyncMem.socMemOffset == 0))
    {
        DEBUG_LVL_ERROR_TRACE("%s(): Error in IOCTL %d\n",
                              __func__,
                              GetLastError());
        timesyncuInstance_l.pSharedMemory = NULL;
        return kErrorNoResource;
    }

#if defined(CONFIG_PCIE)
    ret = ctrlucal_getMappedMem(outSocTimesyncMem.socMemOffset,
                                outSocTimesyncMem.socMemSize,
                                &pSocTimesyncMemOut);

    if ((ret != kErrorOk) || (pSocTimesyncMemOut == NULL))
    {
        DEBUG_LVL_ERROR_TRACE("%s() Couldn't get timesync shared memory.\n",
                              __func__);
        return ret;
    }

    timesyncuInstance_l.pSharedMemory = pSocTimesyncMemOut;
#else
    if (outSocTimesyncMem.socMemSize > inSocTimesyncMem.socMemSize)
        return kErrorNoResource;

    timesyncuInstance_l.pSharedMemory = (void*)outSocTimesyncMem.socMemOffset;
#endif

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Release timesync shared memory

The function releases the remapped timesync shared memory.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError releaseTimesyncShm(void)
{
    if (timesyncuInstance_l.hGlobalFileHandle == NULL)
        return kErrorNoResource;

    if (timesyncuInstance_l.pSharedMemory != NULL)
        timesyncuInstance_l.pSharedMemory = NULL;

    return kErrorOk;
}
#endif

/// \}
