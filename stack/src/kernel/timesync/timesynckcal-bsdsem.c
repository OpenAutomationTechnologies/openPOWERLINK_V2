/**
********************************************************************************
\file   timesynckcal-bsdsem.c

\brief  CAL kernel timesync module using BSD semaphores

This file contains an implementation for the kernel CAL timesync module which
uses BSD semaphores for synchronization.

The sync module is responsible to synchronize the user layer.

\ingroup module_timesynckcal
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2016, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
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
#include <common/timesync.h>
#include <kernel/timesynckcal.h>

#include <fcntl.h>
#include <sys/stat.h>
#include <semaphore.h>

#if defined(CONFIG_INCLUDE_SOC_TIME_FORWARD)
#include <sys/mman.h>
#include <unistd.h>
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

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static sem_t*           syncSem_l;

#if defined(CONFIG_INCLUDE_SOC_TIME_FORWARD)
static int              fd_l;
tTimesyncSharedMemory*  timesynckcal_sharedMemory;
static size_t           memSize_p = sizeof(tTimesyncSharedMemory);
#endif

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Initialize kernel CAL timesync module

The function initializes the kernel CAL timesync module.

\return The function returns a tOplkError error code.

\ingroup module_timesynckcal
*/
//------------------------------------------------------------------------------
tOplkError timesynckcal_init(void)
{
    sem_unlink(TIMESYNC_SYNC_BSDSEM);

    syncSem_l = sem_open(TIMESYNC_SYNC_BSDSEM, O_CREAT, S_IRWXG, 1);
    if (syncSem_l == SEM_FAILED)
    {
        DEBUG_LVL_ERROR_TRACE("%s() creating sem failed!\n", __func__);
        return kErrorNoResource;
    }

#if defined(CONFIG_INCLUDE_SOC_TIME_FORWARD)
    // Initialize shared memory for SOC timestamp
    fd_l = shm_open(SOC_TIMESTAMP_SHM_BSDSEM, O_CREAT | O_RDWR , 0);

    if (fd_l < 0)
    {
        return kErrorNoResource;
    }

    // Allocate shared memory for SOC timestamp
    if (ftruncate(fd_l, memSize_p) < 0)
    {
        return kErrorNoResource;
    }

    timesynckcal_sharedMemory = (void*)mmap(NULL,
                                            memSize_p,              // Memory size
                                            PROT_READ | PROT_WRITE, // Map as read and write memory
                                            MAP_SHARED,             // Map as shared memory
                                            fd_l,                   // File descriptor
                                            0);

    // Check for valid memory mapping
    if (timesynckcal_sharedMemory == MAP_FAILED)
    {
        DEBUG_LVL_ERROR_TRACE("%s() mmap failed!\n", __func__);
        timesynckcal_sharedMemory = NULL;
        return kErrorNoResource;
    }

#endif

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Clean up CAL timesync module

The function cleans up the CAL timesync module

\ingroup module_timesynckcal
*/
//------------------------------------------------------------------------------
void timesynckcal_exit(void)
{
    sem_close(syncSem_l);
    sem_unlink(TIMESYNC_SYNC_BSDSEM);

#if defined(CONFIG_INCLUDE_SOC_TIME_FORWARD)
    // Unmap SOC timestamp shared memory
    if (munmap(timesynckcal_sharedMemory, memSize_p) != 0)
    {
        DEBUG_LVL_ERROR_TRACE("%s() munmap failed!\n", __func__);
    }

    // Uninitiaize SOC timestamp shared memory
    shm_unlink(SOC_TIMESTAMP_SHM_BSDSEM);
#endif
}

//------------------------------------------------------------------------------
/**
\brief  Send a sync event

The function sends a sync event.

\return The function returns a tOplkError error code.

\ingroup module_timesynckcal
*/
//------------------------------------------------------------------------------
tOplkError timesynckcal_sendSyncEvent(void)
{
    sem_post(syncSem_l);

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Enable sync events

The function enables sync events.

\param[in]      fEnable_p           Enable/disable sync event

\return The function returns a tOplkError error code.

\ingroup module_timesynckcal
*/
//------------------------------------------------------------------------------
tOplkError timesynckcal_controlSync(BOOL fEnable_p)
{
    UNUSED_PARAMETER(fEnable_p);

    return kErrorOk;
}

#if defined(CONFIG_INCLUDE_SOC_TIME_FORWARD)
//------------------------------------------------------------------------------
/**
\brief  Get timesync shared memory

The function returns the reference to the timesync shared memory.

\return The function returns a pointer to the timesync shared memory.

\ingroup module_timesynckcal
*/
//------------------------------------------------------------------------------
tTimesyncSharedMemory* timesynckcal_getSharedMemory(void)
{
    return timesynckcal_sharedMemory;
}
#endif

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

/// \}
