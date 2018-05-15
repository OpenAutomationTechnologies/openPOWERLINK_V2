/**
********************************************************************************
\file   timesynck.c

\brief  Kernel timesync module

This file contains the main implementation of the kernel timesync module.

\ingroup module_timesynck
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
#include <common/target.h>
#include <kernel/timesynck.h>
#include <kernel/timesynckcal.h>

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
\brief Timesync instance

The following structure defines the instance variable of the kernel timesync module.
*/
typedef struct
{
    UINT32                  syncEventCycle;     ///< Synchronization event cycle
#if defined(CONFIG_INCLUDE_SOC_TIME_FORWARD)
    tTimesyncSharedMemory*  pSharedMemory;      ///< Time sync shared memory
#endif
} tTimesynckInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tTimesynckInstance   timesynckInstance_l;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Initialize kernel timesync module

The function initializes the kernel timesync module.

\return The function returns a tOplkError error code.

\ingroup module_timesynck
*/
//------------------------------------------------------------------------------
tOplkError timesynck_init(void)
{
    tOplkError  ret;

    OPLK_MEMSET(&timesynckInstance_l, 0, sizeof(timesynckInstance_l));

    timesynckInstance_l.syncEventCycle = 1; // Default every cycle

    ret = timesynckcal_init();
    if (ret != kErrorOk)
        return ret;

#if defined(CONFIG_INCLUDE_SOC_TIME_FORWARD)
    timesynckInstance_l.pSharedMemory = timesynckcal_getSharedMemory();
    if (timesynckInstance_l.pSharedMemory == NULL)
        return kErrorNoResource;

    // Initialize shared memory
    OPLK_MEMSET(timesynckInstance_l.pSharedMemory, 0, sizeof(*timesynckInstance_l.pSharedMemory));

    // Initialize triple buffer
    timesynckInstance_l.pSharedMemory->kernelToUserSocTime.clean = 0;
    timesynckInstance_l.pSharedMemory->kernelToUserSocTime.read = 1;
    timesynckInstance_l.pSharedMemory->kernelToUserSocTime.write = 2;

    OPLK_DCACHE_FLUSH(timesynckInstance_l.pSharedMemory, sizeof(*timesynckInstance_l.pSharedMemory));
#endif

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Cleanup timesync module

The function cleans up the timesync module.

\ingroup module_timesynck
*/
//------------------------------------------------------------------------------
void timesynck_exit(void)
{
    timesynckcal_exit();
#if defined(CONFIG_INCLUDE_SOC_TIME_FORWARD)
    timesynckInstance_l.pSharedMemory = NULL;
#endif
}

//------------------------------------------------------------------------------
/**
\brief  Set cycle time to timesync module

The function sets the POWERLINK cycle time to the timesync module

\param[in]      cycleLen_p          POWERLINK Cycle time [us]
\param[in]      minSyncTime_p       Minimum period for sending sync event [us]

\return The function returns a tOplkError error code.

\ingroup module_timesynck
*/
//------------------------------------------------------------------------------
tOplkError timesynck_setCycleTime(UINT32 cycleLen_p, UINT32 minSyncTime_p)
{
    tOplkError  ret = kErrorOk;

    if ((cycleLen_p == 0) || (minSyncTime_p == 0))
    {
        // - Handle a cycle time of 0 (avoids div by 0)
        // - Handle not configured minimum sync period
        timesynckInstance_l.syncEventCycle = 1;
    }
    else
    {
        // Calculate synchronization event cycle
        timesynckInstance_l.syncEventCycle = ((minSyncTime_p + cycleLen_p - 1) / cycleLen_p);
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Send sync event

The function sends a synchronization event to the user layer.

\return The function returns a tOplkError error code.

\ingroup module_timesynck
*/
//------------------------------------------------------------------------------
tOplkError timesynck_sendSyncEvent(void)
{
    tOplkError      ret = kErrorOk;
    static UINT32   cycleCnt = 0;

    if ((++cycleCnt == timesynckInstance_l.syncEventCycle))
    {
        ret = timesynckcal_sendSyncEvent();

        cycleCnt = 0;
    }
    else if (cycleCnt > timesynckInstance_l.syncEventCycle)
    {
        cycleCnt = 0;
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Process events for timesync

The function processes events intended for the kernel timesync module.

\param[in]      pEvent_p            Pointer to event

\return The function returns a tOplkError error code.

\ingroup module_timesynck
*/
//------------------------------------------------------------------------------
tOplkError timesynck_process(const tEvent* pEvent_p)
{
    tOplkError ret;

    // Check parameter validity
    ASSERT(pEvent_p != NULL);

    switch (pEvent_p->eventType)
    {
        case kEventTypeTimesynckControl:
            ret = timesynckcal_controlSync(*((const BOOL*)pEvent_p->eventArg.pEventArg));
            break;

        default:
            ret = kErrorInvalidEvent;
            break;
    }

    return ret;
}

#if defined(CONFIG_INCLUDE_SOC_TIME_FORWARD)
//------------------------------------------------------------------------------
/**
\brief  Set SoC time

The function sets the given SoC time to the timesync module.

\param[in]      pSocTime_p          Pointer to SoC time information structure

\return The function returns a tOplkError error code.

\ingroup module_timesynck
*/
//------------------------------------------------------------------------------
tOplkError timesynck_setSocTime(const tTimesyncSocTime* pSocTime_p)
{
    tTimesyncSocTimeTripleBuf*  pTripleBuf;
    OPLK_ATOMIC_T               writeBuf;
    tTimesyncSocTime*           pBuffer;

    // Check parameter validity
    ASSERT(pSocTime_p != NULL);

    if (timesynckInstance_l.pSharedMemory == NULL)
    {
        // Looks like the CAL has no SoC time forward support, but feature is
        // enabled!
        DEBUG_LVL_ERROR_TRACE("%s Pointer to shared memory is invalid!\n",
                              __func__);
        return kErrorNoResource;
    }

    pTripleBuf = &timesynckInstance_l.pSharedMemory->kernelToUserSocTime;

    OPLK_DCACHE_INVALIDATE(pTripleBuf, sizeof(*pTripleBuf));

    writeBuf = pTripleBuf->write;
    pBuffer = &pTripleBuf->aTripleBuf[writeBuf];

    OPLK_MEMCPY(pBuffer, pSocTime_p, sizeof(*pBuffer));

    OPLK_ATOMIC_EXCHANGE(&pTripleBuf->clean, writeBuf, pTripleBuf->write);

    pTripleBuf->newData = 1;

    OPLK_DCACHE_FLUSH(&pTripleBuf->clean, sizeof(pTripleBuf->clean));
    OPLK_DCACHE_FLUSH(&pTripleBuf->write, sizeof(pTripleBuf->write));
    OPLK_DCACHE_FLUSH(&pTripleBuf->newData, sizeof(pTripleBuf->newData));

    return kErrorOk;
}

#if defined(CONFIG_INCLUDE_NMT_MN)
//------------------------------------------------------------------------------
/**
\brief  Get net time from user layer

The function returns the net time set by the timesyncu module.

\param[out]      pNetTime_p         Pointer to NETTIME time information structure.
\param[out]      pNewData_p         Pointer to flag which is set to indicate new
                                    net time from user layer.

\return The function returns a tOplkError error code.
\retval kErrorNoResource   Pointer to shared memory is invalid.
\retval kErrorOk           The net time value was successfully returned.

\ingroup module_timesynck
*/
//------------------------------------------------------------------------------
tOplkError timesynck_getNetTime(tNetTime* pNetTime_p, BOOL* pNewData_p)
{
    OPLK_ATOMIC_T readBuf;

    if ((timesynckInstance_l.pSharedMemory == NULL) || (pNetTime_p == NULL) ||
        (pNewData_p == NULL))
    {
        DEBUG_LVL_ERROR_TRACE("%s Pointer to memory is invalid!\n",
                              __func__);
        return kErrorNoResource;
    }

    if (timesynckInstance_l.pSharedMemory->userToKernelSocTime.newData)
    {
        *pNewData_p = TRUE;
        readBuf = timesynckInstance_l.pSharedMemory->userToKernelSocTime.read;

        OPLK_ATOMIC_EXCHANGE(&timesynckInstance_l.pSharedMemory->userToKernelSocTime.clean,
                             readBuf,
                             timesynckInstance_l.pSharedMemory->userToKernelSocTime.read);
        // Once the read from buffer is done, reset the newData
        timesynckInstance_l.pSharedMemory->userToKernelSocTime.newData = 0;

        // Flush data cache for variables changed in this function
        OPLK_DCACHE_FLUSH(&timesynckInstance_l.pSharedMemory->userToKernelSocTime.read,
                          sizeof(OPLK_ATOMIC_T));
        OPLK_DCACHE_FLUSH(&timesynckInstance_l.pSharedMemory->userToKernelSocTime.newData,
                          sizeof(UINT8));

        pNetTime_p->nsec = timesynckInstance_l.pSharedMemory->userToKernelSocTime.aTripleBuf[0].netTime.nsec;
        pNetTime_p->sec = timesynckInstance_l.pSharedMemory->userToKernelSocTime.aTripleBuf[0].netTime.sec;
    }
    else
        *pNewData_p = FALSE;

    return kErrorOk;
}
#endif

#endif

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

/// \}
