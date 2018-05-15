/**
********************************************************************************
\file   timesyncucal-ioctl.c

\brief  Sync implementation for the user CAL timesync module using Linux ioctl

This file contains a sync implementation for the user CAL timesync module. It
uses a Linux ioctl call for synchronisation. In addition SoC timestamp
forwarding feature implementation is done by creating a shared memory for the
user and kernel.

\ingroup module_timesyncucal
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2016, B&R Industrial Automation GmbH
Copyright (c) 2017, Kalycito Infotech Private Limited
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
#include <user/timesyncucal.h>
#include <user/ctrlucal.h>
#include <common/driver.h>

#include <fcntl.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <time.h>

#if defined(CONFIG_INCLUDE_SOC_TIME_FORWARD)
#include <unistd.h>
#include <sys/mman.h>
#endif

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
\brief Instance for user timesync module

This structure contains all necessary information needed by the timesync user
CAL module for Linux ioctl design.
*/
typedef struct
{
    OPLK_FILE_HANDLE        fd;               ///< File descriptor
#if defined(CONFIG_INCLUDE_SOC_TIME_FORWARD)
    tTimesyncSharedMemory*  pSharedMemory;    ///< Shared timesync structure
    size_t                  memSize;          ///< Size of the timesync shared memory
#endif
}tTimesynckcalInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tTimesynckcalInstance instance_l;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
#if defined(CONFIG_INCLUDE_SOC_TIME_FORWARD)
static tOplkError getTimeSyncSharedMem(void);
static tOplkError releaseTimeSyncSharedMem(void);
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
    UNUSED_PARAMETER(pfnSyncCb_p);

    instance_l.fd = ctrlucal_getFd();
#if defined(CONFIG_INCLUDE_SOC_TIME_FORWARD)
    instance_l.memSize = sizeof(tTimesyncSharedMemory);
    if (getTimeSyncSharedMem() != kErrorOk)
    {
        DEBUG_LVL_ERROR_TRACE("%s() Could not get timesync shared memory\n",
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
#if defined(CONFIG_INCLUDE_SOC_TIME_FORWARD)
    tOplkError ret;

    ret = releaseTimeSyncSharedMem();
    if (ret != kErrorOk)
    {
        DEBUG_LVL_ERROR_TRACE("%s() SoC Timestamp shm could not be released\n",
                              __func__);
    }
#endif
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
    int ret;

    ret = ioctl(instance_l.fd, PLK_CMD_TIMESYNC_SYNC, timeout_p);
    if (ret == 0)
        return kErrorOk;

    return kErrorGeneralError;
}

#if defined(CONFIG_INCLUDE_SOC_TIME_FORWARD)
//------------------------------------------------------------------------------
/**
\brief  Get timesync shared memory

The function returns the reference to the timesync shared memory base.

\return The function returns a pointer to the timesync shared memory base.

\ingroup module_timesyncucal
*/
//------------------------------------------------------------------------------
tTimesyncSharedMemory* timesyncucal_getSharedMemory(void)
{
    return instance_l.pSharedMemory;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief  Get timesync shared memory

The function remaps the timesync shared memory into user space memory using the
mmap function.

\return The function returns a tOplkError error code.
\retval kErrorOk                    mmap successful
\retval kErrorNoResource            mmap failed
*/
//------------------------------------------------------------------------------
static tOplkError getTimeSyncSharedMem(void)
{
    instance_l.pSharedMemory = mmap(NULL,                                   // Map at any address in vma
                                    instance_l.memSize + 2 * sysconf(_SC_PAGE_SIZE),
                                    PROT_READ | PROT_WRITE,                 // Map as read and write memory
                                    MAP_SHARED,                             // Map as shared memory
                                    instance_l.fd,                          // File descriptor
                                    sysconf(_SC_PAGE_SIZE));

    if (instance_l.pSharedMemory == MAP_FAILED)
    {
        DEBUG_LVL_ERROR_TRACE("%s() mmap failed!\n", __func__);
        instance_l.pSharedMemory = NULL;
        return kErrorNoResource;
    }

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Release timesync shared memory

The function releases the remapped timesync shared memory into user memory.

\return The function returns a tOplkError error code.
\retval kErrorOk                    munmap successful
\retval kErrorGeneralError          munmap failed
*/
//------------------------------------------------------------------------------
static tOplkError releaseTimeSyncSharedMem(void)
{
    if (instance_l.pSharedMemory != NULL)
    {
        if (munmap(instance_l.pSharedMemory, instance_l.memSize) != 0)
        {
            DEBUG_LVL_ERROR_TRACE("%s() munmap failed\n", __func__);
            return kErrorNoResource;
        }

        instance_l.pSharedMemory = NULL;
        instance_l.memSize = 0;
    }

    return kErrorOk;
}
#endif

/// \}
