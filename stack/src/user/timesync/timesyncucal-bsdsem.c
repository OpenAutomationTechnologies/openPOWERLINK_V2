/**
********************************************************************************
\file   timesyncucal-bsdsem.c

\brief  Sync implementation for the user CAL timesync module using BSD semaphores

This file contains a sync implementation for the user CAL timesync module. It
uses BSD semaphores for synchronization and a shared memory (shm) to transfer
time stamps.

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
#include <common/timesync.h>
#include <user/timesyncucal.h>

#include <errno.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <semaphore.h>
#include <time.h>

#if defined(CONFIG_INCLUDE_SOC_TIME_FORWARD)
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
\brief Memory instance for user timesync module

This structure contains all necessary information needed by the timesync CAL
module for BSD semaphores.
*/
typedef struct
{
    sem_t*                    syncSem;                   ///< Semaphore for synchronization between kernel and user tiemsync modules
#if defined(CONFIG_INCLUDE_SOC_TIME_FORWARD)
    OPLK_FILE_HANDLE          fd;                        ///< File descriptor for POWERLINK device
    size_t                    memSize;                   ///< Memory size of SoC timestamp shared memory
    tTimesyncSharedMemory*    pSharedMemory;             ///< Pointer to SoC timestamp shared memory
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
static tOplkError createTimestampShm(void);
static tOplkError destroyTimestampShm(void);
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
#if defined(CONFIG_INCLUDE_SOC_TIME_FORWARD)
    tOplkError  ret;
#endif

    UNUSED_PARAMETER(pfnSyncCb_p);
    OPLK_MEMSET(&instance_l, 0, sizeof(tTimesyncucalInstance));

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
\brief  Clean up user CAL timesync module

The function cleans up the user CAL timesync module

\ingroup module_timesyncucal
*/
//------------------------------------------------------------------------------
void timesyncucal_exit(void)
{
#if defined(CONFIG_INCLUDE_SOC_TIME_FORWARD)
    tOplkError ret;

    ret = destroyTimestampShm();
    if (ret != kErrorOk)
    {
        DEBUG_LVL_ERROR_TRACE("%s() exit failed", __func__);
    }
#endif

    sem_close(instance_l.syncSem);

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
    int                 semRet;
    struct timespec     currentTime;
    struct timespec     semTimeout;

    if (timeout_p != 0)
    {
        if (timeout_p >= 1000000)
        {
            semTimeout.tv_sec = (timeout_p / 1000000);
            semTimeout.tv_nsec = (timeout_p % 1000000) * 1000;
        }
        else
        {
            semTimeout.tv_sec = 0;
            semTimeout.tv_nsec = timeout_p * 1000;
        }
        clock_gettime(CLOCK_REALTIME, &currentTime);
        TIMESPECADD(&semTimeout, &currentTime);
        semRet = sem_timedwait(instance_l.syncSem, &semTimeout);
    }
    else
    {
        semRet = sem_wait(instance_l.syncSem);
    }

    if (semRet == 0)
        return kErrorOk;
    else
        return kErrorGeneralError;
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
\brief  Create timesync shared memory

This function initializes and allocates the shared memory for SoC timestamp.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError createTimestampShm(void)
{
    instance_l.memSize = sizeof(tTimesyncSharedMemory);

    // Initialize shared memory for SOC timestamp
    instance_l.fd = shm_open(TIMESYNC_TIMESTAMP_SHM, O_RDWR, 0);

    if (instance_l.fd < 0)
    {
        DEBUG_LVL_ERROR_TRACE("%s() Initialization of shared memory failed!\n",
                              __func__);
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
        //Unlink timesync shared memory
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
    // Unmap timesync shared memory
    if (instance_l.pSharedMemory != NULL)
    {
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
