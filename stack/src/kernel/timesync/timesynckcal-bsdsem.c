/**
********************************************************************************
\file   timesynckcal-bsdsem.c

\brief  CAL kernel timesync module using BSD semaphores

This file contains an implementation for the kernel CAL timesync module which
uses BSD semaphores for synchronization and a shared memory (shm) to transfer
time stamps.

The sync module is responsible to synchronize the user layer.

\ingroup module_timesynckcal
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
/**
\brief Memory instance for kernel timesync module

This structure contains all necessary information needed by the timesync CAL
module for BSD semaphores.
*/
typedef struct
{
    sem_t*                    syncSem;                   ///< Semaphore for synchronization between kernel and user tiemsync modules.
#if defined(CONFIG_INCLUDE_SOC_TIME_FORWARD)
    OPLK_FILE_HANDLE          fd;                        ///< File descriptor for POWERLINK device.
    size_t                    memSize;                   ///< Memory size of SoC timestamp shared memory.
    tTimesyncSharedMemory*    pSharedMemory;             ///< Pointer to SoC timestamp shared memory.
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
static tOplkError createTimestampShm(void);
static tOplkError destroyTimestampShm(void);
#endif

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
#if defined(CONFIG_INCLUDE_SOC_TIME_FORWARD)
    tOplkError  ret;
#endif

    OPLK_MEMSET(&instance_l, 0, sizeof(tTimesynckcalInstance));
    sem_unlink(TIMESYNC_SYNC_BSDSEM);

    instance_l.syncSem = sem_open(TIMESYNC_SYNC_BSDSEM, O_CREAT, S_IRWXG, 1);
    if (instance_l.syncSem == SEM_FAILED)
    {
        DEBUG_LVL_ERROR_TRACE("%s() creating sem failed!\n", __func__);
        return kErrorNoResource;
    }

#if defined(CONFIG_INCLUDE_SOC_TIME_FORWARD)
    ret = createTimestampShm();
    if (ret != kErrorOk)
    {
        // Close the semaphore
        sem_close(instance_l.syncSem);
        // Unlink the semaphore
        sem_unlink(TIMESYNC_SYNC_BSDSEM);
        return ret;
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
#if defined(CONFIG_INCLUDE_SOC_TIME_FORWARD)
    tOplkError ret;

    ret = destroyTimestampShm();
    if (ret != kErrorOk)
    {
        DEBUG_LVL_ERROR_TRACE("%s() timesync shared memory exit failed", __func__);
    }
#endif

    sem_close(instance_l.syncSem);
    sem_unlink(TIMESYNC_SYNC_BSDSEM);
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
    sem_post(instance_l.syncSem);

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
    return instance_l.pSharedMemory;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief  Create timesync shared memory

This function initializes and allocates the shared memory for SoC timestamp.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError createTimestampShm(void)
{
    instance_l.memSize = sizeof(tTimesyncSharedMemory);

    // Initialize shared memory for SoC timestamp
    instance_l.fd = shm_open(TIMESYNC_TIMESTAMP_SHM, O_CREAT | O_RDWR, 0);

    if (instance_l.fd < 0)
    {
        DEBUG_LVL_ERROR_TRACE("%s() Initialization of shared memory failed!\n",
                              __func__);
        return kErrorNoResource;
    }

    // Allocate shared memory for SoC timestamp
    if (ftruncate(instance_l.fd, instance_l.memSize) < 0)
    {
        DEBUG_LVL_ERROR_TRACE("%s() Allocation of shared memory failed!\n",
                              __func__);
        // Unlink the timesync shared memory
        shm_unlink(TIMESYNC_TIMESTAMP_SHM);
        return kErrorNoResource;
    }

    instance_l.pSharedMemory = (void*)mmap(NULL,
                                           instance_l.memSize,     // Memory size
                                           PROT_READ | PROT_WRITE, // Map as read and write memory
                                           MAP_SHARED,             // Map as shared memory
                                           instance_l.fd,          // File descriptor for POWERLINK device
                                           0);

    // Check for valid memory mapping
    if (instance_l.pSharedMemory == MAP_FAILED)
    {
        DEBUG_LVL_ERROR_TRACE("%s() timesync shared memory mmap failed!\n",
                              __func__);
        instance_l.pSharedMemory = NULL;
        //Unlink the timesync shared memory
        shm_unlink(TIMESYNC_TIMESTAMP_SHM);
        return kErrorNoResource;
    }

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Destroy timesync shared memory

This function unmaps and unlinks the SoC timestamp shared memory.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError destroyTimestampShm(void)
{
    if (instance_l.pSharedMemory != NULL)
    {
        // Unmap timesync shared memory
        if (munmap(instance_l.pSharedMemory, instance_l.memSize) != 0)
        {
            DEBUG_LVL_ERROR_TRACE("%s() timesync shared memory munmap failed!\n",
                                  __func__);

            return kErrorNoResource;
        }

        instance_l.pSharedMemory = NULL;
        instance_l.memSize = 0;
    }

    // Unlink timesync shared memory
    shm_unlink(TIMESYNC_TIMESTAMP_SHM);

    return kErrorOk;
}
#endif

/// \}
