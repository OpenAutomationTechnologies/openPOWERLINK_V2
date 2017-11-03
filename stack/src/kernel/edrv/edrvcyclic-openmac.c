/**
********************************************************************************
\file   edrvcyclic-openmac.c

\brief  Implementation of openMAC Cyclic Ethernet driver

This file contains the implementation of the openMAC Cyclic Ethernet driver.
It implements time-triggered transmission of frames necessary for MN.

\ingroup module_edrv
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2013, SYSTEC electronic GmbH
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
#include <kernel/edrvcyclic.h>
#include <kernel/edrv.h>
#include <kernel/hrestimer.h>
#include <oplk/benchmark.h>

#include <target/openmac.h>
#include <omethlib.h>

//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

#if (CONFIG_TIMER_USE_HIGHRES == FALSE)
#error "EdrvCyclic needs CONFIG_TIMER_USE_HIGHRES = TRUE"
#endif

//define to shift timer interrupt before cycle
#ifndef CONFIG_EDRVCYC_NEG_SHIFT_US
#define CONFIG_EDRVCYC_NEG_SHIFT_US        100U //us (timer irq before next cycle)
#endif

#if (CONFIG_EDRVCYC_NEG_SHIFT_US < 50U)
#error "Set EDRVCYC_NEG_SHIFT larger 50 us!"
#endif

#if (CONFIG_EDRV_CYCLIC_USE_DIAGNOSTICS != FALSE)
#warning "edrvcyclic diagnostics is not supported by openMAC!"
#endif

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

// Defines necessary for calculating correct Tx time (oplkdefs.h - THX!)
#define EDRVCYC_BYTETIME_NS         (C_DLL_T_BITTIME * 8U)
#define EDRVCYC_IPG_NS              C_DLL_T_IFG
#define EDRVCYC_PREAMB_SIZE         (C_DLL_T_PREAMBLE / C_DLL_T_BITTIME / 8U)

#define EDRVCYC_MIN_NEG_SHIFT_US    20U

//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------
/**
\brief Structure describing an instance of the cyclic Edrv

This structure describes an instance of the cyclic Ethernet driver.
*/
typedef struct
{
    tEdrvTxBuffer**     apTxBufferList;         ///< Pointer to the Tx buffer list
    UINT                maxTxBufferCount;       ///< Maximum Tx buffer count
    UINT                currrentTxBufferList;   ///< Current Tx buffer list
    UINT                currentTxBufferEntry;   ///< Current Tx buffer entry
    UINT32              cycleLengthUs;          ///< Cycle time (us)
    UINT32              syncEventCycle;         ///< Synchronization event cycle
    tTimerHdl           timerHdlCycle;          ///< Handle of the cycle timer
    tEdrvCyclicCbSync   pfnSyncCb;              ///< Function pointer to the sync callback function
    tEdrvCyclicCbError  pfnErrorCb;             ///< Function pointer to the error callback function
    tTimestamp          nextCycleTime;          ///< Timestamp of the start of the next cycle
    tTimestamp          lastIsrEntryTime;       ///< Timestamp when the ISR was entered previously
    tTimestamp          lastIsrExitTime;        ///< Timestamp when the ISR was exit previously
    BOOL                fNextCycleTimeValid;    ///< Flag indicating whether the value in nextCycleTime is valid
    BOOL                fLastIsrTimeValid;      ///< Flag indicating whether the values in lastIsrEntryTime and
                                                /**< lastIsrExitTime are valid */
} tEdrvCyclicInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tEdrvCyclicInstance instance_l;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static tOplkError timerHdlCycleCb(const tTimerEventArg* pEventArg_p) SECTION_EDRVCYC_TIMER_CB;
static tOplkError processTxBufferList(void);
static void       handleExtSync(UINT32 nextSocTime_p);

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Cyclic Ethernet driver initialization

This function initializes the cyclic Ethernet driver.

\return The function returns a tOplkError error code.

\ingroup module_edrv
*/
//------------------------------------------------------------------------------
tOplkError edrvcyclic_init(void)
{
    // Clear instance structure
    OPLK_MEMSET(&instance_l, 0, sizeof(instance_l));

    instance_l.syncEventCycle = 1; // Default every cycle

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Shut down cyclic Ethernet driver

This function shuts down the cyclic Ethernet driver.

\return The function returns a tOplkError error code.

\ingroup module_edrv
*/
//------------------------------------------------------------------------------
tOplkError edrvcyclic_exit(void)
{
    if (instance_l.apTxBufferList != NULL)
    {
        OPLK_FREE(instance_l.apTxBufferList);
        instance_l.apTxBufferList = NULL;
        instance_l.maxTxBufferCount = 0;
    }

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Set maximum size of Tx buffer list

This function determines the maximum size of the cyclic Tx buffer list.

\param[in]      maxListSize_p       Maximum Tx buffer list size

\return The function returns a tOplkError error code.

\ingroup module_edrv
*/
//------------------------------------------------------------------------------
tOplkError edrvcyclic_setMaxTxBufferListSize(UINT maxListSize_p)
{
    tOplkError ret = kErrorOk;

    if (instance_l.maxTxBufferCount != maxListSize_p)
    {
        instance_l.maxTxBufferCount = maxListSize_p;
        if (instance_l.apTxBufferList != NULL)
        {
            OPLK_FREE(instance_l.apTxBufferList);
            instance_l.apTxBufferList = NULL;
        }

        instance_l.apTxBufferList = (tEdrvTxBuffer**)OPLK_MALLOC(sizeof(*instance_l.apTxBufferList) * maxListSize_p * 2);
        if (instance_l.apTxBufferList != NULL)
        {
            instance_l.currrentTxBufferList = 0;

            OPLK_MEMSET(instance_l.apTxBufferList, 0, sizeof(*instance_l.apTxBufferList) * maxListSize_p * 2);
        }
        else
            ret = kErrorEdrvNoFreeBufEntry;
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Set next Tx buffer list

This function forwards the next cycle Tx buffer list to the cyclic Edrv.

\param[in]      ppTxBuffer_p        Pointer to next cycle Tx buffer list
\param[in]      txBufferCount_p     Tx buffer list count

\return The function returns a tOplkError error code.

\ingroup module_edrv
*/
//------------------------------------------------------------------------------
tOplkError edrvcyclic_setNextTxBufferList(tEdrvTxBuffer* const* ppTxBuffer_p,
                                          UINT txBufferCount_p)
{
    tOplkError  ret = kErrorOk;
    UINT        nextTxBufferList;

    // Check parameter validity
    ASSERT(ppTxBuffer_p != NULL);

    nextTxBufferList = instance_l.currrentTxBufferList ^ instance_l.maxTxBufferCount;

    // Check if next list is free
    if (instance_l.apTxBufferList[nextTxBufferList] != NULL)
    {
        ret = kErrorEdrvNextTxListNotEmpty;
        goto Exit;
    }

    if (txBufferCount_p == 0 || txBufferCount_p > instance_l.maxTxBufferCount)
    {
        ret = kErrorEdrvInvalidParam;
        goto Exit;
    }

    // Check if last entry in list equals a NULL pointer
    if (ppTxBuffer_p[txBufferCount_p - 1] != NULL)
    {
        ret = kErrorEdrvInvalidParam;
        goto Exit;
    }

    OPLK_MEMCPY(&instance_l.apTxBufferList[nextTxBufferList], ppTxBuffer_p, sizeof(*ppTxBuffer_p) * txBufferCount_p);

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Set cycle time

This function sets the cycle time controlled by the cyclic Edrv.

\param[in]      cycleTimeUs_p       Cycle time [us]
\param[in]      minSyncTime_p       Minimum period for sending sync events to the api [us]

\return The function returns a tOplkError error code.

\ingroup module_edrv
*/
//------------------------------------------------------------------------------
tOplkError edrvcyclic_setCycleTime(UINT32 cycleTimeUs_p, UINT32 minSyncTime_p)
{
    instance_l.cycleLengthUs = cycleTimeUs_p;

    if ((cycleTimeUs_p == 0) || (minSyncTime_p == 0))
    {
        // - Handle a cycle time of 0 (avoids div by 0)
        // - Handle not configured minimum sync period
        instance_l.syncEventCycle = 1;
    }
    else
    {
        // Calculate synchronization event cycle
        instance_l.syncEventCycle = ((minSyncTime_p + cycleTimeUs_p -1 ) / cycleTimeUs_p);
    }

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Start cycle

This function starts the cycles.

\param  fContinuousMode_p   If TRUE, the timer will be called continuously
                            Otherwise, it is a one-shot timer

\return The function returns a tOplkError error code.

\ingroup module_edrv
*/
//------------------------------------------------------------------------------
tOplkError edrvcyclic_startCycle(BOOL fContinuousMode_p)
{
    tOplkError  ret = kErrorOk;

    UNUSED_PARAMETER(fContinuousMode_p);

    if (instance_l.cycleLengthUs == 0)
    {
        ret = kErrorEdrvInvalidCycleLen;
        goto Exit;
    }

    // Clear Tx buffer list
    instance_l.currrentTxBufferList = 0;
    instance_l.currentTxBufferEntry = 0;
    OPLK_MEMSET(instance_l.apTxBufferList, 0, sizeof(*instance_l.apTxBufferList) * instance_l.maxTxBufferCount * 2);

    ret = hrestimer_modifyTimer(&instance_l.timerHdlCycle,
            instance_l.cycleLengthUs * 1000ULL, timerHdlCycleCb, 0L, FALSE);

    // Reset valid flags
    instance_l.fNextCycleTimeValid = FALSE;
    instance_l.fLastIsrTimeValid = FALSE;

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Stop cycle

This function stops the cycles.

\param[in]      fKeepCycle_p        If TRUE, just stop transmission (i.e. slot timer),
                                    but keep cycle timer running.

\return The function returns a tOplkError error code.

\ingroup module_edrv
*/
//------------------------------------------------------------------------------
tOplkError edrvcyclic_stopCycle(BOOL fKeepCycle_p)
{
    UNUSED_PARAMETER(fKeepCycle_p);

    return hrestimer_deleteTimer(&instance_l.timerHdlCycle);
}

//------------------------------------------------------------------------------
/**
\brief  Register synchronization callback

This function registers the synchronization callback.

\param[in]      pfnCbSync_p         Function pointer called at the configured
                                    synchronization point

\return The function returns a tOplkError error code.

\ingroup module_edrv
*/
//------------------------------------------------------------------------------
tOplkError edrvcyclic_regSyncHandler(tEdrvCyclicCbSync pfnCbSync_p)
{
    instance_l.pfnSyncCb = pfnCbSync_p;

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Register error callback

This function registers the error callback.

\param[in]      pfnCbError_p        Function pointer called in case of a cycle processing error

\return The function returns a tOplkError error code.

\ingroup module_edrv
*/
//------------------------------------------------------------------------------
tOplkError edrvcyclic_regErrorHandler(tEdrvCyclicCbError pfnCbError_p)
{
    instance_l.pfnErrorCb = pfnCbError_p;

    return kErrorOk;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief  Cycle timer callback

This function is called by the timer module. It starts the next cycle.

\param[in]      pEventArg_p         Timer event argument

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError timerHdlCycleCb(const tTimerEventArg* pEventArg_p)
{
    tOplkError  ret = kErrorOk;
    tTimestamp  entryTime;

    entryTime.timeStamp = OPENMAC_TIMERGETTIMEVALUE();

    if (pEventArg_p->timerHdl.handle != instance_l.timerHdlCycle)
    {
        // Zombie callback - just exit
        goto Exit;
    }

    if (instance_l.apTxBufferList[instance_l.currentTxBufferEntry] != NULL)
    {
        ret = kErrorEdrvTxListNotFinishedYet;
        goto Exit;
    }

    instance_l.apTxBufferList[instance_l.currrentTxBufferList] = NULL;

    // Enter new cycle -> switch Tx buffer list
    instance_l.currrentTxBufferList ^= instance_l.maxTxBufferCount;
    instance_l.currentTxBufferEntry = instance_l.currrentTxBufferList;

    if (instance_l.apTxBufferList[instance_l.currentTxBufferEntry] == NULL)
    {
        ret = kErrorEdrvCurTxListEmpty;
        goto Exit;
    }

    BENCHMARK_MOD_01_SET(0);

    ret = processTxBufferList();
    if (ret != kErrorOk)
    {
        goto Exit;
    }

    BENCHMARK_MOD_01_RESET(0);

    if (instance_l.pfnSyncCb != NULL)
    {
        BENCHMARK_MOD_01_SET(0);
        ret = instance_l.pfnSyncCb();
        BENCHMARK_MOD_01_RESET(0);
    }

Exit:
    if (ret == kErrorOk)
    {
        instance_l.lastIsrEntryTime.timeStamp = entryTime.timeStamp;
        instance_l.lastIsrExitTime.timeStamp = OPENMAC_TIMERGETTIMEVALUE();
        instance_l.fLastIsrTimeValid = TRUE;
    }
    else
    {
        if (instance_l.pfnErrorCb != NULL)
        {
            ret = instance_l.pfnErrorCb(ret, NULL);
        }
    }

    BENCHMARK_MOD_01_RESET(0);

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Process Tx buffer list

This function processes the provided Tx buffer list. It forwards the Tx buffer
descriptors to the Ethernet driver.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError processTxBufferList(void)
{
    tOplkError      ret = kErrorOk;
    tEdrvTxBuffer*  pTxBuffer = NULL;
    UINT32          absoluteTime; // Absolute time accumulator
    UINT            txBufferCnt = 0; // Tx buffer count
    UINT32          nextOffsetNs = 0; // Next earliest Tx time
    UINT32          earliestRxOffsetNs = 0; // Earliest time Rx frame expected
    UINT32          negShift; // Negative time shift
    tTimestamp      nextIrqTime; // Next timer IRQ

    if (!instance_l.fNextCycleTimeValid)
    {
        // Use current time + negative shift to set a valid next cycle
        instance_l.nextCycleTime.timeStamp = OPENMAC_TIMERGETTIMEVALUE() +
                                             OMETH_US_2_TICKS(CONFIG_EDRVCYC_NEG_SHIFT_US);

        instance_l.fNextCycleTimeValid = TRUE;
    }

    handleExtSync(instance_l.nextCycleTime.timeStamp);

    // Initialize time accumulator to SoC Tx time
    absoluteTime = instance_l.nextCycleTime.timeStamp;

    // Process Tx buffer list
    while ((pTxBuffer = instance_l.apTxBufferList[instance_l.currentTxBufferEntry]) != NULL)
    {
        // Compare Tx buffer time offset with next offset (considers IPG and last packet length)
        // Note: Otherwise openMAC is confused if time-trig Tx starts within other time-trig Tx!
        if ((txBufferCnt > 0) && (nextOffsetNs > pTxBuffer->timeOffsetNs))
        {
            absoluteTime += OMETH_NS_2_TICKS(nextOffsetNs);
        }
        else
        {
            absoluteTime += OMETH_NS_2_TICKS(pTxBuffer->timeOffsetNs);
        }

        // set the absolute Tx start time, and the fLaunchTimeValid = TRUE, to
        // use time triggered send
        pTxBuffer->launchTime.ticks = absoluteTime;
        pTxBuffer->fLaunchTimeValid = TRUE; // Enables time triggered send

        ret = edrv_sendTxBuffer(pTxBuffer);
        if (ret != kErrorOk)
        {
            goto Exit;
        }

        // set fLaunchTimeValid flag to FALSE
        // -> If the Tx buffer is reused as manual Tx, edrv_sendTxBuffer will send it normally!
        pTxBuffer->fLaunchTimeValid = FALSE;

        nextOffsetNs = (EDRVCYC_PREAMB_SIZE + EDRV_ETH_CRC_SIZE) * EDRVCYC_BYTETIME_NS;

        if (pTxBuffer->txFrameSize < 60UL)
            nextOffsetNs += (60 * EDRVCYC_BYTETIME_NS); // Consider padding!
        else
            nextOffsetNs += (pTxBuffer->txFrameSize * EDRVCYC_BYTETIME_NS);

        nextOffsetNs += EDRVCYC_IPG_NS;

        if (txBufferCnt == 1)
        {
            // SoC and first PReq (or PResMN) sent
            earliestRxOffsetNs = nextOffsetNs;
        }

        // Switch to next Tx buffer
        instance_l.currentTxBufferEntry++;
        txBufferCnt++;
    }

    // Process negative shift time
    if (instance_l.fLastIsrTimeValid)
    {
        // Get duration of ISR processing
        negShift = instance_l.lastIsrExitTime.timeStamp -
                   instance_l.lastIsrEntryTime.timeStamp;

        if (negShift > OMETH_NS_2_TICKS(earliestRxOffsetNs))
        {
            // Reduce shift so that ISR is done when first Rx frames are expected
            negShift -= OMETH_NS_2_TICKS(earliestRxOffsetNs);
        }

        if (negShift < OMETH_US_2_TICKS(EDRVCYC_MIN_NEG_SHIFT_US))
        {
            // Clip shift to minimum possible shift
            negShift = OMETH_US_2_TICKS(EDRVCYC_MIN_NEG_SHIFT_US);
        }
    }
    else
    {
        // ISR processing time not valid, use default shift value.
        negShift = OMETH_US_2_TICKS(CONFIG_EDRVCYC_NEG_SHIFT_US);
    }

    // Calculate next cycle and set up timer interrupt
    instance_l.nextCycleTime.timeStamp += OMETH_US_2_TICKS(instance_l.cycleLengthUs);
    nextIrqTime.timeStamp = instance_l.nextCycleTime.timeStamp - negShift;

    ret = hrestimer_setAbsoluteTimer(&instance_l.timerHdlCycle, nextIrqTime,
                                     timerHdlCycleCb, 0L);
    if (ret != kErrorOk)
    {
        DEBUG_LVL_ERROR_TRACE("%s: hrestimer_setAbsoluteTimer ret=0x%X\n", __func__, ret);
        goto Exit;
    }

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Handle external synchronization

This function handles the external synchronization interrupt.

\param[in]      nextSocTime_p       Next SoC send time

*/
//------------------------------------------------------------------------------
static void handleExtSync(UINT32 nextSocTime_p)
{
    tTimestamp      extSyncTime;
    static UINT32   extCycleCnt = 0;

    if (++extCycleCnt == instance_l.syncEventCycle)
    {
        // Forward next SoC time to timer module
        extSyncTime.timeStamp = nextSocTime_p;
        hrestimer_setExtSyncIrqTime(extSyncTime);

        extCycleCnt = 0;
    }
    else if (extCycleCnt > instance_l.syncEventCycle)
    {
        extCycleCnt = 0;
    }
}

/// \}
