/**
********************************************************************************
\file   edrvcyclic-openmac.c

\brief  Implementation of openMAC Cyclic Ethernet driver

This file contains the implementation of the openMAC Cyclic Ethernet driver.

\ingroup module_edrv
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2013, SYSTEC electronic GmbH
Copyright (c) 2013, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
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
#include <global.h>
#include <Epl.h>
#include <kernel/EplTimerHighResk.h>

#include <edrv.h>
#include <openmac.h>
#include <omethlib.h>

#include <Benchmark.h>
#include <Debug.h>

//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

#if EPL_TIMER_USE_HIGHRES == FALSE
#error "EdrvCyclic needs EPL_TIMER_USE_HIGHRES = TRUE"
#endif

//define to shift timer interrupt before cycle
#ifndef EDRVCYC_NEG_SHIFT_US
#define EDRVCYC_NEG_SHIFT_US        100U //us (timer irq before next cycle)
#endif

#if (EDRVCYC_NEG_SHIFT_US < 50U)
#error "Set EDRVCYC_NEG_SHIFT larger 50 us!"
#endif

#if EDRV_CYCLIC_USE_DIAGNOSTICS != FALSE
#warning "EdrvCyclic Diagnostics is not supported by openMAC!"
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
//defines for cycle preparation call
#define EDRVCYC_POS_SHIFT_US        50U //us (duration of ProcessTxBufferList)
#define EDRVCYC_MIN_SHIFT_US        10U //us (minimum time of ISR call before SoC)

//define for negative shift filter
#define EDRVCYC_NEGSHFT_FLT_SPAN    8U

//define necessary for calculating correct TX time (EplDef.h - THX!)
#define EDRVCYC_BYTETIME_NS     (EPL_C_DLL_T_BITTIME * 8U)
#define EDRVCYC_IPG_NS          EPL_C_DLL_T_IFG
#define EDRVCYC_PREAMB_SIZE     (EPL_C_DLL_T_PREAMBLE / EPL_C_DLL_T_BITTIME / 8U)

//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------
typedef struct
{
    tEdrvTxBuffer**     apTxBufferList;
    UINT                maxTxBufferCount;
    UINT                currrentTxBufferList;
    UINT                currentTxBufferEntry;
    UINT32              cycleLengthUs;
    tEplTimerHdl        timerHdlCycle;
    tEdrvCyclicCbSync   pfnSyncCb;
    tEdrvCyclicCbError  pfnErrorCb;
    UINT32              nextCycleTime;
    BOOL                fNextCycleTimeValid;
    UINT32              aTxProcFilter[EDRVCYC_NEGSHFT_FLT_SPAN];
    UINT                txProcFilterIndex;
    UINT32              txProcFilterResult;
} tEdrvCyclicInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tEdrvCyclicInstance instance_l;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static tEplKernel timerHdlCycleCb(tEplTimerEventArg* pEventArg_p) SECTION_EDRVCYC_TIMER_CB;
static tEplKernel processTxBufferList(void);
static tEplKernel processCycleViolation(UINT32 nextTimerIrqNs_p);

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Cyclic Ethernet driver initialization

This function initializes the Cyclic Ethernet driver.

\return The function returns a tEplKernel error code.

\ingroup module_edrv
*/
//------------------------------------------------------------------------------
tEplKernel edrvcyclic_init(void)
{
    // clear instance structure
    EPL_MEMSET(&instance_l, 0, sizeof (instance_l));

    return kEplSuccessful;
}

//------------------------------------------------------------------------------
/**
\brief  Cyclic Ethernet driver shutdown

This function shuts down the Cyclic Ethernet driver.

\return The function returns a tEplKernel error code.

\ingroup module_edrv
*/
//------------------------------------------------------------------------------
tEplKernel edrvcyclic_shutdown(void)
{
    if (instance_l.apTxBufferList != NULL)
    {
        EPL_FREE(instance_l.apTxBufferList);
        instance_l.apTxBufferList = NULL;
        instance_l.maxTxBufferCount = 0;
    }

    return kEplSuccessful;
}

//------------------------------------------------------------------------------
/**
\brief  Set maximum Tx buffer entries

This function sets the maximum number of Tx buffer list entries.

\param  maxListEntries_p    Maximum Tx buffer list entries requested by dllk

\return The function returns a tEplKernel error code.

\ingroup module_edrv
*/
//------------------------------------------------------------------------------
tEplKernel edrvcyclic_setMaxTxBufferListSize(UINT maxListEntries_p)
{
    tEplKernel ret = kEplSuccessful;

    if (instance_l.maxTxBufferCount != maxListEntries_p)
    {
        instance_l.maxTxBufferCount = maxListEntries_p;
        if (instance_l.apTxBufferList != NULL)
        {
            EPL_FREE(instance_l.apTxBufferList);
            instance_l.apTxBufferList = NULL;
        }

        instance_l.apTxBufferList = EPL_MALLOC(sizeof (*instance_l.apTxBufferList) * maxListEntries_p * 2);
        if (instance_l.apTxBufferList == NULL)
        {
            ret = kEplEdrvNoFreeBufEntry;
        }

        instance_l.currrentTxBufferList = 0;

        EPL_MEMSET(instance_l.apTxBufferList, 0, sizeof (*instance_l.apTxBufferList) * maxListEntries_p * 2);
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Set next Tx buffer list

This function is called the exchange the Tx buffer list to be processed next.

\param  apTxBuffer_p        Pointer to the next Tx buffer list
\param  txBufferCount_p     Number of Tx buffer list entries

\return The function returns a tEplKernel error code.

\ingroup module_edrv
*/
//------------------------------------------------------------------------------
tEplKernel edrvcyclic_setNextTxBufferList(tEdrvTxBuffer** apTxBuffer_p, UINT txBufferCount_p)
{
    tEplKernel  ret = kEplSuccessful;
    UINT        nextTxBufferList;

    nextTxBufferList = instance_l.currrentTxBufferList ^ instance_l.maxTxBufferCount;

    // check if next list is free
    if (instance_l.apTxBufferList[nextTxBufferList] != NULL)
    {
        ret = kEplEdrvNextTxListNotEmpty;
        goto Exit;
    }

    if (txBufferCount_p == 0 || txBufferCount_p > instance_l.maxTxBufferCount)
    {
        ret = kEplEdrvInvalidParam;
        goto Exit;
    }

    // check if last entry in list equals a NULL pointer
    if (apTxBuffer_p[txBufferCount_p - 1] != NULL)
    {
        ret = kEplEdrvInvalidParam;
        goto Exit;
    }

    EPL_MEMCPY(&instance_l.apTxBufferList[nextTxBufferList], apTxBuffer_p, sizeof (*apTxBuffer_p) * txBufferCount_p);

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Set POWERLINK cycle time

This function sets the POWERLINK cycle time.

\param  cycleLengthUs_p     POWERLINK cycle time [us]

\return The function returns a tEplKernel error code.

\ingroup module_edrv
*/
//------------------------------------------------------------------------------
tEplKernel edrvcyclic_setCycleTime (UINT32 cycleLengthUs_p)
{
    instance_l.cycleLengthUs = cycleLengthUs_p;

    return kEplSuccessful;
}

//------------------------------------------------------------------------------
/**
\brief  Start cyclic Ethernet driver

This function starts the cyclic Ethernet driver module.

\return The function returns a tEplKernel error code.

\ingroup module_edrv
*/
//------------------------------------------------------------------------------
tEplKernel edrvcyclic_startCycle (void)
{
    tEplKernel  ret = kEplSuccessful;
    INT         i;

    if (instance_l.cycleLengthUs == 0)
    {
        ret = kEplEdrvInvalidCycleLen;
        goto Exit;
    }

    //set initial time value for TX Process duration
    instance_l.txProcFilterResult = OMETH_US_2_TICKS(EDRVCYC_POS_SHIFT_US);

    //initialize the filter
    for(i=0; i<EDRVCYC_NEGSHFT_FLT_SPAN; i++)
    {
        instance_l.aTxProcFilter[i] = OMETH_US_2_TICKS(EDRVCYC_POS_SHIFT_US);
    }
    instance_l.txProcFilterIndex = 0;

    // clear Tx buffer list
    instance_l.currrentTxBufferList = 0;
    instance_l.currentTxBufferEntry = 0;
    EPL_MEMSET(instance_l.apTxBufferList, 0, sizeof (*instance_l.apTxBufferList) * instance_l.maxTxBufferCount * 2);

    ret = EplTimerHighReskModifyTimerNs(&instance_l.timerHdlCycle,
            instance_l.cycleLengthUs * 1000ULL, timerHdlCycleCb, 0L, FALSE);

    //the next cycle value is not valid!
    instance_l.fNextCycleTimeValid = FALSE;

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Stop cyclic Ethernet driver

This function stops the cyclic Ethernet driver module.

\return The function returns a tEplKernel error code.

\ingroup module_edrv
*/
//------------------------------------------------------------------------------
tEplKernel edrvcyclic_stopCycle (void)
{
    return EplTimerHighReskDeleteTimer(&instance_l.timerHdlCycle);
}

//------------------------------------------------------------------------------
/**
\brief  Register sync callback handler

This function registers the sync callback handler from the dllk.

\param  pfnCbSync_p     Sync callback function pointer

\return The function returns a tEplKernel error code.

\ingroup module_edrv
*/
//------------------------------------------------------------------------------
tEplKernel edrvcyclic_regSyncHandler (tEdrvCyclicCbSync pfnCbSync_p)
{
    instance_l.pfnSyncCb = pfnCbSync_p;

    return kEplSuccessful;
}

//------------------------------------------------------------------------------
/**
\brief  Register error callback handler

This function registers the error callback handler from the dllk.

\param  pfnCbError_p    Error callback function pointer

\return The function returns a tEplKernel error code.

\ingroup module_edrv
*/
//------------------------------------------------------------------------------
tEplKernel edrvcyclic_regErrorHandler (tEdrvCyclicCbError pfnCbError_p)
{
    instance_l.pfnErrorCb = pfnCbError_p;

    return kEplSuccessful;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief  Cyclic timer callback

This function is called by the cyclic timer. It starts the next cycle.

\param  pEventArg_p     Timer event argument

\return The function returns the allocated packet buffer's descriptor.
*/
//------------------------------------------------------------------------------
static tEplKernel timerHdlCycleCb(tEplTimerEventArg* pEventArg_p)
{
    tEplKernel  ret = kEplSuccessful;
    UINT32      macTimeDiff;
    UINT32      macTime1;
    UINT32      macTime2;
    UINT32      filterAccumulate;
    INT         i;

    if (pEventArg_p->m_TimerHdl != instance_l.timerHdlCycle)
    {   // zombie callback
        // just exit
        goto Exit;
    }

    if (instance_l.apTxBufferList[instance_l.currentTxBufferEntry] != NULL)
    {
        ret = kEplEdrvTxListNotFinishedYet;
        goto Exit;
    }

    instance_l.apTxBufferList[instance_l.currrentTxBufferList] = NULL;

    // enter new cycle -> switch Tx buffer list
    instance_l.currrentTxBufferList ^= instance_l.maxTxBufferCount;
    instance_l.currentTxBufferEntry = instance_l.currrentTxBufferList;

    if (instance_l.apTxBufferList[instance_l.currentTxBufferEntry] == NULL)
    {
        ret = kEplEdrvCurTxListEmpty;
        goto Exit;
    }

    BENCHMARK_MOD_01_SET(0);

    //get timer tick before calling TX Process
    macTime1 = openmac_timerGetTimeValue(HWTIMER_SYNC);

    ret = processTxBufferList();

    if (ret != kEplSuccessful)
    {
        goto Exit;
    }

    //get timer tick after calling TX Process
    macTime2 = openmac_timerGetTimeValue(HWTIMER_SYNC);

    //obtain absolute difference
    macTimeDiff = macTime2 - macTime1;

    //do filtering
    // add to filter
    instance_l.aTxProcFilter[instance_l.txProcFilterIndex] = macTimeDiff;

    // increment filter index for next entry
    instance_l.txProcFilterIndex++;
    if( instance_l.txProcFilterIndex >= EDRVCYC_NEGSHFT_FLT_SPAN )
    {
        instance_l.txProcFilterIndex = 0;
    }

    // sum all entries
    filterAccumulate = 0U;
    for(i=0; i<EDRVCYC_NEGSHFT_FLT_SPAN; i++)
    {
        filterAccumulate += instance_l.aTxProcFilter[i];
    }

    // store average to instance
    instance_l.txProcFilterResult = filterAccumulate / EDRVCYC_NEGSHFT_FLT_SPAN;

    BENCHMARK_MOD_01_RESET(0);

    if (instance_l.pfnSyncCb != NULL)
    {
        BENCHMARK_MOD_01_SET(0);
        ret = instance_l.pfnSyncCb();
        BENCHMARK_MOD_01_RESET(0);
    }

Exit:
    BENCHMARK_MOD_01_RESET(0);
    if (ret != kEplSuccessful)
    {

        if (instance_l.pfnErrorCb != NULL)
        {
            ret = instance_l.pfnErrorCb(ret, NULL);
        }
    }
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Process Tx buffer list

This function processes the provided Tx buffer list. It forwards the Tx buffer
descriptors to the Ethernet driver.

\return The function returns the allocated packet buffer's descriptor.
*/
//------------------------------------------------------------------------------
static tEplKernel processTxBufferList(void)
{
    tEplKernel      ret = kEplSuccessful;
    tEdrvTxBuffer*  pTxBuffer = NULL;
    UINT32          absoluteTime; //absolute time accumulator
    BOOL            fFirstPkt = TRUE; //flag to identify first packet
    UINT32          macTimeFctCall = openmac_timerGetTimeValue(HWTIMER_SYNC); //MAC TIME AT FUNCTION CALL
    UINT32          cycleMin = 0; //absolute minimum cycle time
    UINT32          cycleMax = 0; //absolute maximum cycle time
    UINT32          nextOffsetNs = 0; //next earliest tx time
    UINT32          nextTimerIrqNs = instance_l.cycleLengthUs * 1000UL; //time of next timer irq

    if(instance_l.fNextCycleTimeValid == FALSE)
    {
        //use current time + negative shift to set a valid next cycle
        instance_l.nextCycleTime = macTimeFctCall + OMETH_US_2_TICKS(EDRVCYC_NEG_SHIFT_US);

        //next timer IRQ correction
        nextTimerIrqNs -= OMETH_TICKS_2_NS(instance_l.txProcFilterResult);

        instance_l.fNextCycleTimeValid = TRUE;
    }
    else
    {
        //cycle is valid, compensate next timer irq
        //calculate time difference of Next SoC - approx. entry of this function
        UINT32 diffToNextSoc;
        UINT32 diffToNextSocNs;

        diffToNextSoc = instance_l.nextCycleTime - macTimeFctCall;

        if(diffToNextSoc & 0x80000000UL)
        {
            diffToNextSoc = macTimeFctCall - instance_l.nextCycleTime;
            diffToNextSocNs = OMETH_TICKS_2_NS(diffToNextSoc);
            nextTimerIrqNs -= (EDRVCYC_NEG_SHIFT_US * 1000UL + diffToNextSocNs);
            ret = processCycleViolation(nextTimerIrqNs);
            goto CycleDone;
        }

        //substract TX buffer list processing from cycle time
        nextTimerIrqNs -= OMETH_TICKS_2_NS(instance_l.txProcFilterResult);

        diffToNextSocNs = OMETH_TICKS_2_NS(diffToNextSoc);

        if(diffToNextSocNs > EDRVCYC_NEG_SHIFT_US * 1000UL)
        {
            //time difference is larger negative shift
            nextTimerIrqNs += (diffToNextSocNs - EDRVCYC_NEG_SHIFT_US * 1000UL);
        }
        else if(diffToNextSocNs > EDRVCYC_MIN_SHIFT_US * 1000UL)
        {
            //time difference is shorter than negative shift but larger than minimum
            nextTimerIrqNs -= (EDRVCYC_NEG_SHIFT_US * 1000UL - diffToNextSocNs);
        }
        else
        {
            //time difference is too short => cycle violation!
            nextTimerIrqNs -= (EDRVCYC_NEG_SHIFT_US * 1000UL - diffToNextSocNs);
            ret = processCycleViolation(nextTimerIrqNs);
            goto CycleDone;
        }
    }

    //set accumulator for cycle window calculation
    absoluteTime = instance_l.nextCycleTime; //get cycle start time

    //set limits to verify if time triggered tx is within cycle
    // note: otherwise openMAC would be confused
    cycleMin = instance_l.nextCycleTime; //minimum limit
    cycleMax = instance_l.nextCycleTime + OMETH_US_2_TICKS(instance_l.cycleLengthUs); //maximum limit

    //loop through TX buffer list
    while((pTxBuffer = instance_l.apTxBufferList[instance_l.currentTxBufferEntry]) != NULL)
    {
        //compare TX buffer time offset with next offset (considers IPG and last packet length)
        //note: otherwise openMAC is confused if time-trig TX starts within other time-trig TX!
        if(fFirstPkt == FALSE && nextOffsetNs > pTxBuffer->timeOffsetNs)
        {
            absoluteTime += OMETH_NS_2_TICKS(nextOffsetNs); //accumulate offset
        }
        else
        {
            absoluteTime += OMETH_NS_2_TICKS(pTxBuffer->timeOffsetNs); //accumulate offset
        }

        fFirstPkt = FALSE; //first packet is surely out...

        //verify if packet TX start time is within cycle window
        if( (absoluteTime - cycleMin) > (cycleMax - cycleMin) )
        {
            //packet is outside of the window
            ret = processCycleViolation(nextTimerIrqNs);
            goto CycleDone;
        }

        //pTxBuffer->timeOffsetNs = absoluteTime | 1; //lowest bit enables time triggered send
        //set the absolute TX start time, and OR the lowest bit to give Edrv a hint
        pTxBuffer->timeOffsetAbs = absoluteTime | 1; //lowest bit enables time triggered send

        ret = edrv_sendTxBuffer(pTxBuffer);
        if (ret != kEplSuccessful)
        {
            goto Exit;
        }

        //set the absolute TX start time to zero
        // -> If the TX buffer is reused as manual TX, edrv_sendTxBuffer is not confused!
        pTxBuffer->timeOffsetAbs = 0;

        //calculate the length of the sent packet, add IPG and thus know the next earliest TX time
        {
            UINT32 udwLength = pTxBuffer->txFrameSize;

            //consider padding!
            if( udwLength < 60UL )
            {
                udwLength = 60UL;
            }

            // ( pre + header + payload + padding + crc ) * 80ns/byte + 960ns = next TX time
            nextOffsetNs = EDRVCYC_BYTETIME_NS * (EDRVCYC_PREAMB_SIZE + EDRV_ETH_CRC_SIZE + udwLength) + EDRVCYC_IPG_NS;
        }

        //switch to next TX buffer
        instance_l.currentTxBufferEntry++;
    }

    //set up next timer interrupt
    ret = EplTimerHighReskModifyTimerNs(&instance_l.timerHdlCycle, nextTimerIrqNs,
            timerHdlCycleCb, 0L, FALSE);

    if(ret != kEplSuccessful)
    {
        PRINTF("%s: EplTimerHighReskModifyTimerNs ret=0x%X\n", __func__, ret);
        goto Exit;
    }

CycleDone:
    //calculate next cycle
    instance_l.nextCycleTime += OMETH_US_2_TICKS(instance_l.cycleLengthUs);

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Process cycle time violation

This function processes a cycle time violation. It skips the Tx packets of the
current cycle and sets up the timer handle for the next cycle.

\param  nextTimerIrqNs_p    Irq time of next cycle [ns]

\return The function returns the allocated packet buffer's descriptor.
*/
//------------------------------------------------------------------------------
static tEplKernel processCycleViolation(UINT32 nextTimerIrqNs_p)
{
    tEplKernel ret;

    //set up next timer interrupt
    ret = EplTimerHighReskModifyTimerNs(&instance_l.timerHdlCycle, nextTimerIrqNs_p,
            timerHdlCycleCb, 0L, FALSE);

    if (ret != kEplSuccessful)
    {
        PRINTF("%s: EplTimerHighReskModifyTimerNs ret=0x%X\n", __func__, ret);
        goto Exit;
    }

    //post error to generate EPL_DLL_ERR_MN_CYCTIMEEXCEED in EplDllkCbCyclicError()
    ret = kEplEdrvTxListNotFinishedYet;
Exit:
    return ret;
}

///\}
