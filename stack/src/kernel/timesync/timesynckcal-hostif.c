/**
********************************************************************************
\file   timesynckcal-hostif.c

\brief  Host interface CAL kernel timesync module

The sync module is responsible to synchronize the user layer.

\ingroup module_timesynckcal
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2017, B&R Industrial Automation GmbH
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
\brief Memory instance for kernel timesync module

This structure contains all necessary information needed by the timesync CAL
module for host interface.
*/
typedef struct
{
    tHostifInstance         pHifInstance;   ///< Host interface instance
#if defined(CONFIG_INCLUDE_SOC_TIME_FORWARD)
    tTimesyncSharedMemory*  pSharedMemory;  ///< Shared memory
#endif
} tTimesynckcalInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tTimesynckcalInstance instance_l;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static tOplkError enableSyncIrq(BOOL fEnable_p);

#if defined(CONFIG_INCLUDE_SOC_TIME_FORWARD)
static void* getSharedMemory(tHostifInstance pHifInstance_p);
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
    OPLK_MEMSET(&instance_l, 0, sizeof(tTimesynckcalInstance));

    instance_l.pHifInstance = hostif_getInstance(0);

    if (instance_l.pHifInstance == NULL)
    {
        DEBUG_LVL_ERROR_TRACE("%s: Could not find hostif instance!\n", __func__);
        return kErrorNoResource;
    }

#if defined(CONFIG_INCLUDE_SOC_TIME_FORWARD)
    instance_l.pSharedMemory = (tTimesyncSharedMemory*)getSharedMemory(instance_l.pHifInstance);
#endif

    return enableSyncIrq(FALSE);
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
    if (instance_l.pHifInstance == NULL)
    {
        DEBUG_LVL_ERROR_TRACE("%s: Could not find hostif instance!\n", __func__);
        return;
    }

    enableSyncIrq(FALSE);

#if defined(CONFIG_INCLUDE_SOC_TIME_FORWARD)
    instance_l.pSharedMemory = NULL;
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
    if (instance_l.pHifInstance == NULL)
    {
        DEBUG_LVL_ERROR_TRACE("%s: Could not find hostif instance!\n", __func__);
        return kErrorNoResource;
    }

    return enableSyncIrq(fEnable_p);
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

//------------------------------------------------------------------------------
/**
\brief  Enable sync interrupt source in host interface IP-Core

\param[in]      fEnable_p           enable/disable sync interrupt source

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError enableSyncIrq(BOOL fEnable_p)
{
    tHostifReturn hifRet;

    hifRet = hostif_irqSourceEnable(instance_l.pHifInstance, kHostifIrqSrcSync, fEnable_p);
    if (hifRet != kHostifSuccessful)
    {
        DEBUG_LVL_ERROR_TRACE("%s irq not possible (%d)!\n",
                              fEnable_p ? "enable" : "disable",
                              hifRet);
        return kErrorNoResource;
    }

    return kErrorOk;
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

    OPLK_MEMSET(pBase, 0, span);

    return pBase;
}
#endif

/// \}
