/**
********************************************************************************
\file   timesyncu.c

\brief  User timesync module

This file contains the main implementation of the user timesync module.

\ingroup module_timesyncu
*******************************************************************************/

/*------------------------------------------------------------------------------
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
#include <common/oplkinc.h>
#include <common/target.h>
#include <user/timesyncu.h>
#include <user/timesyncucal.h>

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
#if defined(CONFIG_INCLUDE_SOC_TIME_FORWARD)
static tTimesyncSharedMemory*   pSharedMemory_l = NULL;
#endif

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
#if defined(CONFIG_INCLUDE_SOC_TIME_FORWARD)
static tTimesyncSocTime* getSocTime(void);
#endif

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Initialize user timesync module

The function initializes the user timesync module.

\param[in]      pfnSyncCb_p         Function that is called in case of sync event

\return The function returns a tOplkError error code.

\ingroup module_timesyncu
*/
//------------------------------------------------------------------------------
tOplkError timesyncu_init(tSyncCb pfnSyncCb_p)
{
    tOplkError  ret;

    ret = timesyncucal_init(pfnSyncCb_p);
    if (ret != kErrorOk)
        return ret;

#if defined(CONFIG_INCLUDE_SOC_TIME_FORWARD)
    pSharedMemory_l = timesyncucal_getSharedMemory();
    if (pSharedMemory_l == NULL)
        return kErrorNoResource;
#endif

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Cleanup timesync module

The function cleans up the timesync module.

\ingroup module_timesyncu
*/
//------------------------------------------------------------------------------
void timesyncu_exit(void)
{
    timesyncucal_exit();
#if defined(CONFIG_INCLUDE_SOC_TIME_FORWARD)
    pSharedMemory_l = NULL;
#endif
}

#if defined(CONFIG_INCLUDE_SOC_TIME_FORWARD)
//------------------------------------------------------------------------------
/**
\brief  Get SoC time

The function obtains the SoC time information.

\param[out]     pSocTime_p          Pointer to memory to store SoC time information

\return The function returns a tOplkError error code.

\ingroup module_timesyncu
*/
//------------------------------------------------------------------------------
tOplkError timesyncu_getSocTime(tOplkApiSocTimeInfo* pSocTime_p)
{
    tTimesyncSocTime*   pSocTime;

    if (pSocTime_p == NULL)
        return kErrorNoResource;

    // Check if the shared memory is available
    if (pSharedMemory_l == NULL)
    {
        DEBUG_LVL_ERROR_TRACE("%s Pointer to shared memory is invalid!\n",
                              __func__);
        return kErrorNoResource;
    }

    pSocTime = getSocTime();
    if (pSocTime != NULL)
    {
        // Assign timesync to API structure members
        pSocTime_p->fValidRelTime = (pSocTime->fRelTimeValid != 0);
        pSocTime_p->relTime = pSocTime->relTime;
        pSocTime_p->netTime = pSocTime->netTime;
    }

    return kErrorOk;
}
#endif

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

#if defined(CONFIG_INCLUDE_SOC_TIME_FORWARD)
//------------------------------------------------------------------------------
/**
\brief  Obtain SoC time from triple buffer

The function returns the pointer to the SoC time information buffer in the
triple buffer.

\return The function returns a pointer to SoC time information buffer.
*/
//------------------------------------------------------------------------------
static tTimesyncSocTime* getSocTime(void)
{
    tTimesyncSocTimeTripleBuf*  pTripleBuf;
    OPLK_ATOMIC_T               readBuf;

    pTripleBuf = &pSharedMemory_l->socTime;

    OPLK_DCACHE_INVALIDATE(pTripleBuf, sizeof(*pTripleBuf));

    if (pTripleBuf->newData)
    {
        // Switch triple buffer because there is new data available!
        readBuf = pTripleBuf->read;
        OPLK_ATOMIC_EXCHANGE(&pTripleBuf->clean, readBuf, pTripleBuf->read);

        pTripleBuf->newData = 0;

        OPLK_DCACHE_FLUSH(&pTripleBuf->clean, sizeof(pTripleBuf->clean));
        OPLK_DCACHE_FLUSH(&pTripleBuf->read, sizeof(pTripleBuf->read));
        OPLK_DCACHE_FLUSH(&pTripleBuf->newData, sizeof(pTripleBuf->newData));
    }

    readBuf = pTripleBuf->read;

    // Return reference to read buffer
    return &pTripleBuf->aTripleBuf[readBuf];
}
#endif

/// \}
