/**
********************************************************************************
\file   timesyncucal-linuxdpshm.c

\brief  Sync implementation for the user CAL timesync module using Linux ioctl

This file contains a sync implementation for the user CAL timesync module. It
uses a Linux ioctl call for synchronisation. In addition SoC timestamp
forwarding feature implementation is done by creating a shared memory for the
user and kernel.

\ingroup module_timesyncucal
*******************************************************************************/

/*------------------------------------------------------------------------------
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
#if defined(CONFIG_INCLUDE_SOC_TIME_FORWARD)
#include <common/memmap.h>
#endif

#include <fcntl.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <time.h>

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
\brief Memory instance for user timesync module

The structure contains all necessary information needed by the timesync CAL
module for dual processor design.
*/
typedef struct
{
    OPLK_FILE_HANDLE        fd;             ///< File descriptor for POWERLINK device
#if defined(CONFIG_INCLUDE_SOC_TIME_FORWARD)
    tTimesyncSharedMemory*  pSharedMemory;  ///< Pointer to shared memory
#endif
}tTimesyncucalInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tTimesyncucalInstance instance_l;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
#if defined(CONFIG_INCLUDE_SOC_TIME_FORWARD)
static tOplkError getTimeSyncShm(void);
static void       releaseTimeSyncShm(void);
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
\retval kErrorOk  Timesyncucal initialization successful
\retval other     Timesyncucal initialization fails

\ingroup module_timesyncucal
*/
//------------------------------------------------------------------------------
tOplkError timesyncucal_init(tSyncCb pfnSyncCb_p)
{
#if defined(CONFIG_INCLUDE_SOC_TIME_FORWARD)
    tOplkError ret;
#endif

    UNUSED_PARAMETER(pfnSyncCb_p);

    OPLK_MEMSET(&instance_l, 0, sizeof(tTimesyncucalInstance));
    instance_l.fd = ctrlucal_getFd();

#if defined(CONFIG_INCLUDE_SOC_TIME_FORWARD)
    ret = getTimeSyncShm();
    if (ret != kErrorOk)
        return ret;
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
    releaseTimeSyncShm();
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
    if (ret != 0)
        return kErrorGeneralError;

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

This function gets the available SoC timesync shared memory using ioctl and
maps the shared memory using memmap.

\return The function returns a tOplkError error code.
\retval kErrorOk          Get timesync shared memory is successful
\retval kErrorNoResource  Kernel or User timesync shared memory is null
*/
//------------------------------------------------------------------------------
static tOplkError getTimeSyncShm(void)
{
    ULONG* pSocTimesyncMem = NULL;
    int    ret;

    // Gets the timesync shared memory kernel address
    ret = ioctl(instance_l.fd, PLK_CMD_TIMESYNC_MAP_OFFSET, &pSocTimesyncMem);

    if (ret < 0)
    {
       DEBUG_LVL_ERROR_TRACE("%s(): IOCTL failed! Couldn't get timesync shared memory\n",
                             __func__);
       return kErrorGeneralError;
    }

    if (pSocTimesyncMem == NULL)
    {
        DEBUG_LVL_ERROR_TRACE("%s(): Timesync shared memory is NULL\n", __func__);
        return kErrorNoResource;
    }

    // Maps the kernel layer buffer address into user application virtual address space
    instance_l.pSharedMemory = memmap_mapKernelBuffer(pSocTimesyncMem,
                                                      sizeof(tTimesyncSharedMemory));

    if (instance_l.pSharedMemory == NULL)
    {
        DEBUG_LVL_ERROR_TRACE("%s(): Couldn't map timesync shared memory\n",
                              __func__);
        return kErrorNoResource;
    }

    return kErrorOk;
}
//------------------------------------------------------------------------------
/**
\brief  Release timesync shared memory

This function releases the unmapped timesync shared memory.

*/
//------------------------------------------------------------------------------
static void releaseTimeSyncShm(void)
{
    if (instance_l.pSharedMemory != NULL)
    {
        // Unmaps timesync shared memory
        memmap_unmapKernelBuffer(instance_l.pSharedMemory);
        instance_l.pSharedMemory = NULL;
    }
}
#endif
/// \}
