/**
********************************************************************************
\file   timesynckcal-linuxdpshm.c

\brief  CAL kernel timesync module using the openPOWERLINK Linux kernel driver

This file contains an implementation for the kernel CAL timesync module which
uses the openPOWERLINK Linux kernel driver interface. In addition SoC timestamp
forwarding feature implementation is done by creating a shared memory for the
user and kernel.

The sync module is responsible to synchronize the user layer.

\ingroup module_timesynckcal
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
#include <kernel/timesynckcal.h>

#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/wait.h>

#if defined(CONFIG_INCLUDE_SOC_TIME_FORWARD)
#include <drvintf.h>
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

The structure contains all necessary information needed by the timesync CAL
module for dual processor design.
*/
typedef struct
{
    wait_queue_head_t       syncWaitQueue;   ///< Wait queue for time sync event
    BOOL                    fSync;           ///< Flag for wait sync event
    BOOL                    fInitialized;    ///< Flag for timesynckcal module initialization
#if defined(CONFIG_INCLUDE_SOC_TIME_FORWARD)
    tTimesyncSharedMemory*  pSharedMemory;   ///< Shared timesync structure
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
static tOplkError initTimeSyncShm(void);
static void       exitTimeSyncShm(void);
#endif

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Initialize kernel CAL timesync module

The function initializes the kernel CAL timesync module.

\return The function returns a tOplkError error code.
\retval kErrorOk  Timesynckcal initialization successful
\retval other     Timesynckcal initialization fails

\ingroup module_timesynckcal
*/
//------------------------------------------------------------------------------
tOplkError timesynckcal_init(void)
{
#if defined(CONFIG_INCLUDE_SOC_TIME_FORWARD)
    tOplkError ret;
#endif

    OPLK_MEMSET(&instance_l, 0, sizeof(tTimesynckCalInstance));

    init_waitqueue_head(&instance_l.syncWaitQueue);
    instance_l.fInitialized = TRUE;

#if defined(CONFIG_INCLUDE_SOC_TIME_FORWARD)
    ret = initTimeSyncShm();
    if (ret != kErrorOk)
        return ret;
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
    exitTimeSyncShm();
#endif
}

//------------------------------------------------------------------------------
/**
\brief  Send a sync event

The function sends a sync event.

\return The function returns a tOplkError error code.
\retval kErrorOk  Send sync event successful

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
\retval kErrorOk          Wait sync event successful
\retval kErrorRetry       wait_event_interruptible_timeout returns NULL
\retval kErrorNoResource  Flag for timesynckcal module initialization is not set

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
                                           instance_l.fSync,
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
\retval kErrorOk Successfully enables sync events

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
\retval Returns a timesync shared memory pointer

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
\brief  Initialize timesync shared memory

This function initializes the shared memory buffer for timesync module.

\return The function returns a tOplkError error code.
\retval kErrorOk          Timesync shared memory initialization successful
\retval kErrorNoResource  Received timesync shared memory is NULL
\retval other             Timesync initialization fails
*/
//------------------------------------------------------------------------------
static tOplkError initTimeSyncShm(void)
{
    tOplkError ret;

    // Initiates the shared memory for the timesync module
    ret = drvintf_initTimesyncShm();
    if (ret != kErrorOk)
    {
        DEBUG_LVL_ERROR_TRACE("%s(): Timesync initialization failed (0x%X)\n",
                              ret);
        return ret;
    }

    // Receives the address of timesync shared memory to be passed to user layer
    instance_l.pSharedMemory = drvintf_getTimesyncShm();

    if (instance_l.pSharedMemory == NULL)
    {
        DEBUG_LVL_ERROR_TRACE("%s(): Timesync shared memory is NULL\n",
                              __func__);
        return kErrorNoResource;
    }

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Exit timesync shared memory

This function unmaps the SoC timesync shared memory.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static void exitTimeSyncShm(void)
{
    if (instance_l.pSharedMemory != NULL)
    {
        // Frees the timesync shared memory
        if (drvintf_exitTimesyncShm() != kErrorOk)
            DEBUG_LVL_ERROR_TRACE("%s(): Timesync shared memory exit failed",
                                  __func__);

        instance_l.pSharedMemory = NULL;
    }
}
#endif

/// \}
