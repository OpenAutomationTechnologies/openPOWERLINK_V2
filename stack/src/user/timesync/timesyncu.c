/**
********************************************************************************
\file   timesyncu.c

\brief  User timesync module

This file contains the main implementation of the user timesync module.

\ingroup module_timesyncu
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
/**
\brief Memory instance for user timesync module

This structure contains all necessary information needed by the user timesync
module.
*/
typedef struct
{
    tSyncCb                pfnSyncCb;           ///< Synchronization callback.
#if defined(CONFIG_INCLUDE_SOC_TIME_FORWARD)
    tTimesyncSharedMemory* pSharedMemory;       ///< Pointer to SoC timestamp shared memory.
#if defined(CONFIG_INCLUDE_NMT_MN)
    BOOL                   fFirstSyncEventDone; ///< Flag to indicate first sync event.
#endif /* defined(CONFIG_INCLUDE_NMT_MN) */
#endif /* defined(CONFIG_INCLUDE_SOC_TIME_FORWARD) */
}tTimesyncuInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tTimesyncuInstance instance_l;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
#if defined(CONFIG_INCLUDE_SOC_TIME_FORWARD)
static tTimesyncSocTime* getSocTime(void);
#if defined(CONFIG_INCLUDE_NMT_MN)
static tOplkError setNetTime(void);
#endif /* defined(CONFIG_INCLUDE_NMT_MN) */
#endif /* defined(CONFIG_INCLUDE_SOC_TIME_FORWARD) */
static tOplkError syncCb(void);

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

    OPLK_MEMSET(&instance_l, 0, sizeof(tTimesyncuInstance));
#if defined(CONFIG_INCLUDE_SOC_TIME_FORWARD)
    instance_l.pSharedMemory = NULL;
#if defined(CONFIG_INCLUDE_NMT_MN)
    instance_l.fFirstSyncEventDone = FALSE;
#endif
#endif
    // Store the pointer function locally for synchronization callback
    instance_l.pfnSyncCb = pfnSyncCb_p;

    ret = timesyncucal_init(syncCb);
    if (ret != kErrorOk)
        return ret;

#if defined(CONFIG_INCLUDE_SOC_TIME_FORWARD)
    instance_l.pSharedMemory = timesyncucal_getSharedMemory();
    if (instance_l.pSharedMemory == NULL)
        return kErrorNoResource;
#endif

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Wait for a sync event and set the SoC time to kernel

The function waits for a sync event and sets the SoC time to the kernel at the first
sync event.

\param[in]      timeout_p           Specifies a timeout in microseconds. If 0 it waits
                                    forever.

\return The function returns a tOplkError error code.

\ingroup module_timesyncu
*/
//------------------------------------------------------------------------------
tOplkError timesyncu_waitSyncEvent(ULONG timeout_p)
{
    tOplkError  ret;

    ret = timesyncucal_waitSyncEvent(timeout_p);

    if (ret != kErrorOk)
        return ret;

#if (defined(CONFIG_INCLUDE_SOC_TIME_FORWARD) && defined(CONFIG_INCLUDE_NMT_MN))
        if (!instance_l.fFirstSyncEventDone)
        {   // Set MN net time at first sync event
            ret = setNetTime();
            if (ret != kErrorOk)
                return ret;

            instance_l.fFirstSyncEventDone = TRUE;
        }
#endif

    return kErrorOk;
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
    instance_l.pSharedMemory = NULL;
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
    if (instance_l.pSharedMemory == NULL)
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

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

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

    pTripleBuf = &instance_l.pSharedMemory->kernelToUserSocTime;

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

#if defined(CONFIG_INCLUDE_NMT_MN)
//------------------------------------------------------------------------------
/**
\brief  Set SoC net time to kernel

The function sets the network time to kernel using the UserToKernelSocTime shared
buffer.

\return The function returns a tOplkError code.
*/
//------------------------------------------------------------------------------
static tOplkError setNetTime(void)
{
    OPLK_ATOMIC_T  writeBuf;
    tOplkError     ret = kErrorOk;
    BOOL           fValidSystemTime = FALSE;

    writeBuf = instance_l.pSharedMemory->userToKernelSocTime.write;
    // Get system clock time
    ret = target_getSystemTime(&instance_l.pSharedMemory->userToKernelSocTime.aTripleBuf[writeBuf].netTime,
                               &fValidSystemTime);

    if (ret != kErrorOk)
    {
        DEBUG_LVL_ERROR_TRACE("%s Failed to get current system time!\n",
                              __func__);
        return ret;
    }

    if (fValidSystemTime)
    {
        OPLK_ATOMIC_EXCHANGE(&instance_l.pSharedMemory->userToKernelSocTime.clean,
                             writeBuf,
                             instance_l.pSharedMemory->userToKernelSocTime.write);
        // Set the newData flag to indicate that a new data is available in the shared buffer
        instance_l.pSharedMemory->userToKernelSocTime.newData = 1;

        // Flush data cache for variables changed in this function
        OPLK_DCACHE_FLUSH(&instance_l.pSharedMemory->userToKernelSocTime.write, sizeof(OPLK_ATOMIC_T));
        OPLK_DCACHE_FLUSH(&instance_l.pSharedMemory->userToKernelSocTime.newData, sizeof(UINT8));
    }

    return ret;
}
#endif
#endif

//------------------------------------------------------------------------------
/**
\brief  Synchronization callback

This function is called for synchronization by the user CAL module and sets
the net time at first synchronization event.

\return The function returns a tOplkError code.
*/
//------------------------------------------------------------------------------
static tOplkError syncCb(void)
{
    tOplkError ret = kErrorOk;

    if (instance_l.pfnSyncCb != NULL)
    {
        ret = instance_l.pfnSyncCb();
        if (ret != kErrorOk)
            return ret;

#if (defined(CONFIG_INCLUDE_SOC_TIME_FORWARD) && defined(CONFIG_INCLUDE_NMT_MN))
        if (!instance_l.fFirstSyncEventDone)
        {
            // Set MN net time at first sync event
            ret = setNetTime();
            if (ret != kErrorOk)
            {
                DEBUG_LVL_ERROR_TRACE("%s Failed to set network time!\n",
                                      __func__);
                return ret;
            }

            instance_l.fFirstSyncEventDone = TRUE;
        }
#endif
    }

    return ret;
}

/// \}
