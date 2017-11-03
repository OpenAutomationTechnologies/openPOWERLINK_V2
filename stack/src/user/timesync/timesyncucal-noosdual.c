/**
********************************************************************************
\file   timesyncucal-noosdual.c

\brief  Dual Processor CAL user timesync module

This file contains the dualprocshm sync implementation for the user timesync CAL
module.

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
#include <common/target.h>

#include <dualprocshm.h>

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
#define TARGET_SYNC_INTERRUPT_ID            1
#define DUALPROCSHM_BUFF_ID_TIMESYNC        14

#define DUALPROCSHM_ADDR_READ_TIMEOUT_MS    1000

//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------
/**
\brief Memory instance for kernel timesync module

The structure contains all necessary information needed by the timesync CAL
module for no-OS dual processor design.
*/
typedef struct
{
    tDualprocDrvInstance    pDrvInstance;   ///< Pointer to dual processor driver instance
#if defined(CONFIG_INCLUDE_SOC_TIME_FORWARD)
    tTimesyncSharedMemory*  pSharedMemory;  ///< Pointer to shared memory
#endif
    tSyncCb                 pfnSyncCb;      ///< Synchronization callback
} tTimesynckcalInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tTimesynckcalInstance    instance_l;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static void irqSyncCb(void);

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
    tDualprocReturn dualRet;
#if defined(CONFIG_INCLUDE_SOC_TIME_FORWARD)
    void*           pBuffer;
    size_t          memSize = sizeof(tTimesyncSharedMemory);
    INT             loopCount = 0;
#endif

    OPLK_MEMSET(&instance_l, 0, sizeof(instance_l));

    instance_l.pDrvInstance = dualprocshm_getLocalProcDrvInst();
    if (instance_l.pDrvInstance == NULL)
    {
        DEBUG_LVL_ERROR_TRACE("%s() couldn't get Host dual proc driver instance\n",
                              __func__);
        return kErrorNoResource;
    }

    dualRet = dualprocshm_registerHandler(instance_l.pDrvInstance, TARGET_SYNC_INTERRUPT_ID, irqSyncCb);
    if (dualRet != kDualprocSuccessful)
    {
        DEBUG_LVL_ERROR_TRACE("%s: Enable irq not possible!\n", __func__);
        return kErrorNoResource;
    }

    instance_l.pfnSyncCb = pfnSyncCb_p;

#if defined(CONFIG_INCLUDE_SOC_TIME_FORWARD)
    for (loopCount = 0; loopCount < DUALPROCSHM_ADDR_READ_TIMEOUT_MS; loopCount++)
    {
        dualRet = dualprocshm_getMemory(instance_l.pDrvInstance,
                                        DUALPROCSHM_BUFF_ID_TIMESYNC,
                                        &pBuffer,
                                        &memSize,
                                        FALSE);

        if (dualRet == kDualprocSuccessful)
            break;

        target_msleep(1);
    }

    if (loopCount == DUALPROCSHM_ADDR_READ_TIMEOUT_MS)
    {
        DEBUG_LVL_ERROR_TRACE("%s() couldn't allocate timesync buffer (%d)\n",
                              __func__,
                              dualRet);
        return kErrorNoResource;
    }

    instance_l.pSharedMemory = (tTimesyncSharedMemory*)pBuffer;
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
    instance_l.pfnSyncCb = NULL;
#if defined(CONFIG_INCLUDE_SOC_TIME_FORWARD)
    instance_l.pSharedMemory = NULL;
#endif

    if (instance_l.pDrvInstance != NULL)
    {
        dualprocshm_registerHandler(instance_l.pDrvInstance, TARGET_SYNC_INTERRUPT_ID, NULL);
        dualprocshm_freeMemory(instance_l.pDrvInstance, DUALPROCSHM_BUFF_ID_TIMESYNC, FALSE);
    }
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
    UNUSED_PARAMETER(timeout_p);

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
#endif

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{
//------------------------------------------------------------------------------
/**
\brief  Synchronization callback

This function is called for synchronization by the dualprocshm instance.

*/
//------------------------------------------------------------------------------
static void irqSyncCb(void)
{
    if (instance_l.pfnSyncCb != NULL)
        instance_l.pfnSyncCb();
}

/// \}
