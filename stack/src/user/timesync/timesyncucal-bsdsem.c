/**
********************************************************************************
\file   timesyncucal-bsdsem.c

\brief  Sync implementation for the user CAL timesync module using BSD semaphores

This file contains a sync implementation for the user CAL timesync module. It
uses BSD semaphores for synchronisation.

\ingroup module_timesyncucal
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

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static sem_t*           syncSem_l;

#if defined(CONFIG_INCLUDE_SOC_TIME_FORWARD)
static int              fd_l;
tTimesyncSharedMemory*  timesyncucal_sharedMemory;
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

    syncSem_l = sem_open(TIMESYNC_SYNC_BSDSEM, O_CREAT, S_IRWXG, 1);
    if (syncSem_l == SEM_FAILED)
    {
        DEBUG_LVL_ERROR_TRACE("%s() creating sem failed!\n", __func__);
        return kErrorNoResource;
    }

#if defined(CONFIG_INCLUDE_SOC_TIME_FORWARD)
    // Initialize shared memory for SOC timestamp
    fd_l = shm_open(SOC_TIMESTAMP_SHM_BSDSEM, O_RDWR, 0);

    if (fd_l < 0)
    {
        return kErrorNoResource;
    }

    timesyncucal_sharedMemory = (void*)mmap(NULL,
                                            memSize_p,              // Memory size
                                            PROT_READ | PROT_WRITE, // Map as read and write memory
                                            MAP_SHARED,             // Map as shared memory
                                            fd_l,                   // File descriptor
                                            0);

    // Check for valid memory mapping
    if (timesyncucal_sharedMemory == MAP_FAILED)
    {
        DEBUG_LVL_ERROR_TRACE("%s() mmap failed!\n", __func__);
        timesyncucal_sharedMemory = NULL;
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
    sem_close(syncSem_l);

#if defined(CONFIG_INCLUDE_SOC_TIME_FORWARD)
    // Unmap SOC timestamp shared memory
    if (munmap(timesyncucal_sharedMemory, memSize_p) != 0)
    {
        DEBUG_LVL_ERROR_TRACE("%s() munmap failed!\n", __func__);
    }

    // Uninitialize SOC timestamp shared memory
    shm_unlink(SOC_TIMESTAMP_SHM_BSDSEM);
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
        semRet = sem_timedwait(syncSem_l, &semTimeout);
    }
    else
    {
        semRet = sem_wait(syncSem_l);
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
    return timesyncucal_sharedMemory;
}
#endif

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

/// \}
