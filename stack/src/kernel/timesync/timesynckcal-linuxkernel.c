/**
********************************************************************************
\file   timesynckcal-linuxkernel.c

\brief  CAL kernel timesync module using the openPOWERLINK Linux kernel driver

This file contains an implementation for the kernel CAL timesync module which
uses the openPOWERLINK Linux kernel driver interface. In addition SoC timestamp
forwarding feature implementation is done by creating a shared memory for the
user and kernel.

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
#include <kernel/timesynckcal.h>

#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/wait.h>

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
\brief Instance for kernel timesync module

This structure contains all necessary information needed by the timesync CAL
module for Linux kernel module.
*/
typedef struct
{
    wait_queue_head_t       syncWaitQueue;   ///< Wait queue for time sync event
    BOOL                    fSync;           ///< Flag for wait sync event
    BOOL                    fInitialized;    ///< Flag for timesynckcal module initialization
#if defined(CONFIG_INCLUDE_SOC_TIME_FORWARD)
    tTimesyncSharedMemory*  pSharedMemory;   ///< Shared timesync structure
    size_t                  memSize;         ///< Size of the timesync shared memory
#endif
} tTimesynckCalInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tTimesynckCalInstance instance_l; ///< Instance variable of kernel timesync module

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
#if defined(CONFIG_INCLUDE_SOC_TIME_FORWARD)
static tOplkError allocateSocMem(void** ppSocMem_p, size_t memSize_p);
static tOplkError freeSocMem(void* pMem_p, size_t memSize_p);
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
    void*  pMem;
#endif

    OPLK_MEMSET(&instance_l, 0, sizeof(tTimesynckCalInstance));

    init_waitqueue_head(&instance_l.syncWaitQueue);
    instance_l.fInitialized = TRUE;

#if defined(CONFIG_INCLUDE_SOC_TIME_FORWARD)
    instance_l.memSize = sizeof(tTimesyncSharedMemory);
    if (instance_l.pSharedMemory != NULL)
        freeSocMem(instance_l.pSharedMemory, instance_l.memSize);

    if (allocateSocMem(&pMem, instance_l.memSize) != kErrorOk)
        return kErrorNoResource;

    instance_l.pSharedMemory = (tTimesyncSharedMemory*)pMem;
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
    instance_l.fInitialized = FALSE;
#if defined(CONFIG_INCLUDE_SOC_TIME_FORWARD)
    if (instance_l.pSharedMemory != NULL)
       {
            freeSocMem(instance_l.pSharedMemory, instance_l.memSize);
            instance_l.pSharedMemory = NULL;
       }
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
    if (instance_l.fInitialized)
    {
        instance_l.fSync = TRUE;
        wake_up_interruptible(&instance_l.syncWaitQueue);
    }

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Wait for a sync event

The function waits for a sync event

\return The function returns a tOplkError error code.

\ingroup module_timesynckcal
*/
//------------------------------------------------------------------------------
tOplkError timesynckcal_waitSyncEvent(void)
{
    int ret;
    int timeout = 1000 * HZ / 1000;

    if (!instance_l.fInitialized)
        return kErrorNoResource;

    ret = wait_event_interruptible_timeout(instance_l.syncWaitQueue,
                                           (instance_l.fSync != FALSE),
                                           timeout);
    if (ret == 0)
        return kErrorRetry;

    instance_l.fSync = FALSE;

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
\brief  Free timesync shared memory

The function frees shared memory which was allocated in the kernel layer for
transferring the SoC timestamp.

\param[in,out]  pMem_p              Pointer to the shared memory base
\param[in]      memSize_p           Size of timesync shared memory

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError freeSocMem(void* pMem_p, size_t memSize_p)
{
    ULONG   order;

    // Check parameter validity
    ASSERT(pMem_p != NULL);

    order = get_order(memSize_p);
    free_pages((ULONG)pMem_p, order);

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Allocate timesync shared memory

The function allocates shared memory required to transfer the SoC timestamp from
the kernel layer to the user layer.

\param[out]     ppSocMem_p          Pointer to store the timesync shared memory base address.
\param[in]      memSize_p           Size of timesync shared memory

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError allocateSocMem(void** ppSocMem_p, size_t memSize_p)
{
    ULONG   order;

    // Check parameter validity
    ASSERT(ppSocMem_p != NULL);

    order = get_order(memSize_p);
    *ppSocMem_p = (void*)__get_free_pages(GFP_KERNEL, order);
    if (*ppSocMem_p == NULL)
    {
        DEBUG_LVL_ERROR_TRACE("%s() Memory for SoC could not be created\n",
                              __func__);
        return kErrorNoResource;
    }

    return kErrorOk;
}
#endif
/// \}
