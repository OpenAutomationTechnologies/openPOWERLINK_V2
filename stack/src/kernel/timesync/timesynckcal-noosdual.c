/**
********************************************************************************
\file   timesynckcal-noosdual.c

\brief  Dual Processor CAL kernel timesync module

This file contains an implementation for the kernel CAL timesync module which
uses the dualprocshm interrupt feature for synchronization.

The sync module is responsible to synchronize the user layer.

\ingroup module_timesynckcal
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2014, Kalycito Infotech Private Limited
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
#include <kernel/timesynckcal.h>

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
#define TARGET_SYNC_INTERRUPT_ID        1
#define DUALPROCSHM_BUFF_ID_TIMESYNC    14

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
} tTimesynckcalInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tTimesynckcalInstance    instance_l;

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
    tDualprocReturn dualRet;
#if defined(CONFIG_INCLUDE_SOC_TIME_FORWARD)
    void*           pBuffer;
    size_t          memSize = sizeof(tTimesyncSharedMemory);
#endif

    instance_l.pDrvInstance = dualprocshm_getLocalProcDrvInst();
    if (instance_l.pDrvInstance == NULL)
    {
        DEBUG_LVL_ERROR_TRACE("%s() couldn't get PCP dual proc driver instance\n",
                              __func__);
        return kErrorNoResource;
    }

    dualRet = dualprocshm_enableIrq(instance_l.pDrvInstance, TARGET_SYNC_INTERRUPT_ID, FALSE);
    if (dualRet != kDualprocSuccessful)
        return kErrorNoResource;

#if defined(CONFIG_INCLUDE_SOC_TIME_FORWARD)
    dualRet = dualprocshm_getMemory(instance_l.pDrvInstance,
                                    DUALPROCSHM_BUFF_ID_TIMESYNC,
                                    &pBuffer,
                                    &memSize,
                                    TRUE);
    if (dualRet != kDualprocSuccessful)
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
\brief  Clean up CAL timesync module

The function cleans up the CAL timesync module

\ingroup module_timesynckcal
*/
//------------------------------------------------------------------------------
void timesynckcal_exit(void)
{
#if defined(CONFIG_INCLUDE_SOC_TIME_FORWARD)
    instance_l.pSharedMemory = NULL;
#endif

    if (instance_l.pDrvInstance != NULL)
    {
        dualprocshm_enableIrq(instance_l.pDrvInstance, TARGET_SYNC_INTERRUPT_ID, FALSE);
        dualprocshm_freeMemory(instance_l.pDrvInstance, DUALPROCSHM_BUFF_ID_TIMESYNC, TRUE);
    }
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
    tDualprocReturn         dualRet;

    if (instance_l.pDrvInstance == NULL)
    {
        DEBUG_LVL_ERROR_TRACE("%s() couldn't get PCP dual proc driver instance\n",
                              __func__);
        return kErrorNoResource;
    }

    dualRet = dualprocshm_setIrq(instance_l.pDrvInstance, TARGET_SYNC_INTERRUPT_ID, TRUE);
    if (dualRet != kDualprocSuccessful)
        return kErrorNoResource;

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
    tDualprocReturn dualRet;

    if (instance_l.pDrvInstance == NULL)
    {
        DEBUG_LVL_ERROR_TRACE("%s() couldn't get PCP dual proc driver instance\n",
                              __func__);
        return kErrorNoResource;
    }

    dualRet = dualprocshm_enableIrq(instance_l.pDrvInstance, TARGET_SYNC_INTERRUPT_ID, fEnable_p);
    if (dualRet != kDualprocSuccessful)
        return kErrorNoResource;

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
#endif

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

/// \}
