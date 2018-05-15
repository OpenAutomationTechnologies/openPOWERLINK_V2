/**
********************************************************************************
\file   timesyncucal-hostif.c

\brief  Host interface sync implementation for the user CAL timesync module

This file contains the hostif sync implementation for the user CAL timesync module.

\ingroup module_timesyncucal
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2017, B&R Industrial Automation GmbH
Copyright (c) 2017, Kalycito Infotech Private Limited.
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

#include <hostiflib.h>

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
module for host interface.
*/
typedef struct
{
    tHostifInstance         pHifInstance;   ///< Host interface instance
    tSyncCb                 pfnSyncCb;      ///< Synchronization callback
#if defined(CONFIG_INCLUDE_SOC_TIME_FORWARD)
    tTimesyncSharedMemory*  pSharedMemory;  ///< Shared memory
#endif
} tTimesyncucalInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tTimesyncucalInstance instance_l;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static void hostifIrqSyncCb(void* pArg_p);

#if defined(CONFIG_INCLUDE_SOC_TIME_FORWARD)
static void* getSharedMemory(tHostifInstance pHifInstance_p);
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
    tHostifReturn   hifRet;

    OPLK_MEMSET(&instance_l, 0, sizeof(tTimesyncucalInstance));

    instance_l.pHifInstance = hostif_getInstance(0);

    if (instance_l.pHifInstance == NULL)
    {
        DEBUG_LVL_ERROR_TRACE("%s: Could not find hostif instance!\n", __func__);
        return kErrorNoResource;
    }

    hifRet = hostif_irqRegHdl(instance_l.pHifInstance, kHostifIrqSrcSync, hostifIrqSyncCb);
    if (hifRet != kHostifSuccessful)
    {
        DEBUG_LVL_ERROR_TRACE("%s: Enable irq not possible!\n", __func__);
        return kErrorNoResource;
    }

    instance_l.pfnSyncCb = pfnSyncCb_p;

#if defined(CONFIG_INCLUDE_SOC_TIME_FORWARD)
    instance_l.pSharedMemory = (tTimesyncSharedMemory*)getSharedMemory(instance_l.pHifInstance);
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
    tHostifReturn   hifRet;
    tHostifInstance pHifInstance = hostif_getInstance(0);

    if (pHifInstance == NULL)
    {
        DEBUG_LVL_ERROR_TRACE("%s: Could not find hostif instance!\n", __func__);
        return;
    }

    hifRet = hostif_irqRegHdl(pHifInstance, kHostifIrqSrcSync, NULL);
    if (hifRet != kHostifSuccessful)
    {
        DEBUG_LVL_ERROR_TRACE("%s: Disable irq not possible (%d)!\n", __func__, hifRet);
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
\brief  Synchronization callback called by Host Interface library

\param[in,out]  pArg_p              Argument pointer provides hostif instance
*/
//------------------------------------------------------------------------------
static void hostifIrqSyncCb(void* pArg_p)
{
    UNUSED_PARAMETER(pArg_p);

    if (instance_l.pfnSyncCb != NULL)
        instance_l.pfnSyncCb();
}

#if defined(CONFIG_INCLUDE_SOC_TIME_FORWARD)
//------------------------------------------------------------------------------
/**
\brief  Get shared memory

\param[in]      pHifInstance_p      Host interface instance

\return The function returns a pointer to the shared memory.
\retval NULL    No shared memory available!
\retval !NULL   Pointer to shared memory
*/
//------------------------------------------------------------------------------
static void* getSharedMemory(tHostifInstance pHifInstance_p)
{
    tHostifReturn   hifRet;
    void*           pBase;
    size_t          span;

    hifRet = hostif_getBuf(pHifInstance_p, kHostifInstIdTimesync, &pBase, &span);
    if (hifRet != kHostifSuccessful)
    {
        DEBUG_LVL_ERROR_TRACE("%s() Could not get buffer from host interface (%d)\n",
                              __func__,
                              hifRet);
        return NULL;
    }

    if (span < sizeof(tTimesyncSharedMemory))
    {
        DEBUG_LVL_ERROR_TRACE("%s() Time Synchronization Buffer too small (shall be: %lu)\n",
                              __func__,
                              (ULONG)sizeof(tTimesyncSharedMemory));
        return NULL;
    }

    return pBase;
}
#endif

/// \}
