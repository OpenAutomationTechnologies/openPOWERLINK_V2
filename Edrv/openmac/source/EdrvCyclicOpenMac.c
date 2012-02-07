/****************************************************************************

  (c) SYSTEC electronic GmbH, D-07973 Greiz, August-Bebel-Str. 29
      www.systec-electronic.com
  (c) Bernecker + Rainer Industrie-Elektronik Ges.m.b.H.
      A-5142 Eggelsberg, B&R Strasse 1
      www.br-automation.com


  Project:      openPOWERLINK

  Description:  Part of Ethernet driver to implement time-triggered transmission.
                It is necessary to implement Managing Nodes.
                Uses hardware acceleration of openMAC

  License:

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    1. Redistributions of source code must retain the above copyright
       notice, this list of conditions and the following disclaimer.

    2. Redistributions in binary form must reproduce the above copyright
       notice, this list of conditions and the following disclaimer in the
       documentation and/or other materials provided with the distribution.

    3. Neither the name of SYSTEC electronic GmbH nor the names of its
       contributors may be used to endorse or promote products derived
       from this software without prior written permission. For written
       permission, please contact info@systec-electronic.com.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    POSSIBILITY OF SUCH DAMAGE.

    Severability Clause:

        If a provision of this License is or becomes illegal, invalid or
        unenforceable in any jurisdiction, that shall not affect:
        1. the validity or enforceability in that jurisdiction of any other
           provision of this License; or
        2. the validity or enforceability in other jurisdictions of that or
           any other provision of this License.

  -------------------------------------------------------------------------
                $RCSfile$

                $Author$

                $Revision$  $Date$

                $State$

                Build Environment:
                    GCC V3.4

****************************************************************************/

#include "global.h"
#include "EplInc.h"
#include "edrv.h"
#include "kernel/EplTimerHighResk.h"
#include "Benchmark.h"


#if EPL_TIMER_USE_HIGHRES == FALSE
#error "EdrvCyclic needs EPL_TIMER_USE_HIGHRES = TRUE"
#endif

#include "system.h"
#include "omethlib.h"
#include <io.h>
#ifdef __POWERLINK
#define EDRVCYC_TIMER_BASE          (void*)POWERLINK_0_MAC_CMP_BASE
#elif defined(__OPENMAC)
#define EDRVCYC_TIMER_BASE          (void*)OPENMAC_0_CMP_BASE
#else
#error "Configuration unknown!"
#endif

#define EDRV_GET_MAC_TIME()            IORD_32DIRECT(EDRVCYC_TIMER_BASE, 0)

//define to shift timer interrupt before cycle
#define EDRVCYC_POS_SHIFT_US        50U //us (duration of ProcessTxBufferList)
#define EDRVCYC_NEG_SHIFT_US        100U //us (timer irq before next cycle)

#if (EDRVCYC_NEG_SHIFT_US < 50U)
#error "Set EDRVCYC_NEG_SHIFT larger 50 us!"
#endif

#if (EDRVCYC_POS_SHIFT_US < 30U)
#error "Set EDRVCYC_NEG_SHIFT larger 30 us!"
#endif

//define for negative shift filter
#define EDRVCYC_NEGSHFT_FLT_SPAN    8U

//define necessary for calculating correct TX time (EplDef.h - THX!)
#define EDRVCYC_BYTETIME_NS            (EPL_C_DLL_T_BITTIME * 8U)
#define EDRVCYC_IPG_NS                EPL_C_DLL_T_IFG
#define EDRVCYC_PREAMB_SIZE            (EPL_C_DLL_T_PREAMBLE / EPL_C_DLL_T_BITTIME / 8U)

#ifdef __SOC_CHECKER
#define SOC_CHECKER
#define SOC_CHECKER_JITTER_NS        100U
#include "socChecker.h"
#endif

//---------------------------------------------------------------------------
// const defines
//---------------------------------------------------------------------------

#if EDRV_CYCLIC_USE_DIAGNOSTICS != FALSE
#warning "EdrvCyclic Diagnostics is not supported by openMAC!"
#endif


//---------------------------------------------------------------------------
// local types
//---------------------------------------------------------------------------

typedef struct
{
    tEdrvTxBuffer**     m_paTxBufferList;
    unsigned int        m_uiMaxTxBufferCount;
    unsigned int        m_uiCurTxBufferList;
    unsigned int        m_uiCurTxBufferEntry;
    DWORD               m_dwCycleLenUs;
    tEplTimerHdl        m_TimerHdlCycle;
    tEplTimerHdl        m_TimerHdlSlot;
    tEdrvCyclicCbSync   m_pfnCbSync;
    tEdrvCyclicCbError  m_pfnCbError;

    DWORD               m_dwNextCycleTime;
    BOOL                m_fNextCycleValid;

    DWORD               m_aTxProcFlt[EDRVCYC_NEGSHFT_FLT_SPAN];
    unsigned int        m_uiTxProcFltIndex;
    DWORD               m_dwTxProcDur;

#ifdef SOC_CHECKER
    BOOL                m_dwSocCheckerValid;
#endif

} tEdrvCyclicInstance;



//---------------------------------------------------------------------------
// local function prototypes
//---------------------------------------------------------------------------


static tEplKernel EdrvCyclicCbTimerCycle(tEplTimerEventArg* pEventArg_p);

static tEplKernel EdrvCyclicProcessTxBufferList(void);



//---------------------------------------------------------------------------
// module global vars
//---------------------------------------------------------------------------

static tEdrvCyclicInstance EdrvCyclicInstance_l;




//---------------------------------------------------------------------------
//
// Function:    EdrvCyclicInit
//
// Description: initialize EdrvCyclic module
//
// Parameters:  void
//
// Returns:     Errorcode           = kEplSuccessful
//                                  = kEplNoResource
//
// State:
//
//---------------------------------------------------------------------------

tEplKernel EdrvCyclicInit()
{
tEplKernel  Ret;

    Ret = kEplSuccessful;

    // clear instance structure
    EPL_MEMSET(&EdrvCyclicInstance_l, 0, sizeof (EdrvCyclicInstance_l));

//Exit:
    return Ret;

}


//---------------------------------------------------------------------------
//
// Function:    EdrvCyclicShutdown
//
// Description: Shutdown EdrvCyclic module
//
// Parameters:  void
//
// Returns:     Errorcode   = kEplSuccessful
//
// State:
//
//---------------------------------------------------------------------------

tEplKernel EdrvCyclicShutdown(void)
{
    if (EdrvCyclicInstance_l.m_paTxBufferList != NULL)
    {
        EPL_FREE(EdrvCyclicInstance_l.m_paTxBufferList);
        EdrvCyclicInstance_l.m_paTxBufferList = NULL;
        EdrvCyclicInstance_l.m_uiMaxTxBufferCount = 0;
    }

    return kEplSuccessful;
}


//---------------------------------------------------------------------------
//
// Function:    EdrvCyclicSetMaxTxBufferListSize
//
// Description: Sets the maximum number of TxBuffer list entries.
//
// Parameters:  uiMaxListSize_p = maximum number of TxBuffer list entries.
//
// Returns:     Errorcode       = kEplSuccessful
//
// State:
//
//---------------------------------------------------------------------------

tEplKernel EdrvCyclicSetMaxTxBufferListSize(unsigned int uiMaxListSize_p)
{
tEplKernel  Ret = kEplSuccessful;

    if (EdrvCyclicInstance_l.m_uiMaxTxBufferCount != uiMaxListSize_p)
    {
        EdrvCyclicInstance_l.m_uiMaxTxBufferCount = uiMaxListSize_p;
        if (EdrvCyclicInstance_l.m_paTxBufferList != NULL)
        {
            EPL_FREE(EdrvCyclicInstance_l.m_paTxBufferList);
            EdrvCyclicInstance_l.m_paTxBufferList = NULL;
        }

        EdrvCyclicInstance_l.m_paTxBufferList = EPL_MALLOC(sizeof (*EdrvCyclicInstance_l.m_paTxBufferList) * uiMaxListSize_p * 2);
        if (EdrvCyclicInstance_l.m_paTxBufferList == NULL)
        {
            Ret = kEplEdrvNoFreeBufEntry;
        }

        EdrvCyclicInstance_l.m_uiCurTxBufferList = 0;

        EPL_MEMSET(EdrvCyclicInstance_l.m_paTxBufferList, 0, sizeof (*EdrvCyclicInstance_l.m_paTxBufferList) * uiMaxListSize_p * 2);
    }

    return Ret;
}


//---------------------------------------------------------------------------
//
// Function:    EdrvCyclicSetNextTxBufferList
//
// Description: Sets the next TxBuffer list.
//
// Parameters:  apTxBuffer_p
//              uiTxBufferCount_p
//
// Returns:     Errorcode       = kEplSuccessful
//
// State:
//
//---------------------------------------------------------------------------

tEplKernel EdrvCyclicSetNextTxBufferList(tEdrvTxBuffer** apTxBuffer_p, unsigned int uiTxBufferCount_p)
{
tEplKernel  Ret = kEplSuccessful;
unsigned int    uiNextTxBufferList;

    uiNextTxBufferList = EdrvCyclicInstance_l.m_uiCurTxBufferList ^ EdrvCyclicInstance_l.m_uiMaxTxBufferCount;

    // check if next list is free
    if (EdrvCyclicInstance_l.m_paTxBufferList[uiNextTxBufferList] != NULL)
    {
        Ret = kEplEdrvNextTxListNotEmpty;
        goto Exit;
    }

    if ((uiTxBufferCount_p == 0)
        || (uiTxBufferCount_p > EdrvCyclicInstance_l.m_uiMaxTxBufferCount))
    {
        Ret = kEplEdrvInvalidParam;
        goto Exit;
    }

    // check if last entry in list equals a NULL pointer
    if (apTxBuffer_p[uiTxBufferCount_p - 1] != NULL)
    {
        Ret = kEplEdrvInvalidParam;
        goto Exit;
    }

    //BENCHMARK_MOD_01_TOGGLE(3);

    EPL_MEMCPY(&EdrvCyclicInstance_l.m_paTxBufferList[uiNextTxBufferList], apTxBuffer_p, sizeof (*apTxBuffer_p) * uiTxBufferCount_p);

Exit:
    return Ret;
}


//---------------------------------------------------------------------------
//
// Function:    EdrvCyclicSetCycleLenUs()
//
// Description:
//
// Parameters:
//
// Return:      tEplKernel      = error code
//
// State:       not tested
//
//---------------------------------------------------------------------------

tEplKernel EdrvCyclicSetCycleLenUs (DWORD dwCycleLenUs_p)
{
tEplKernel      Ret = kEplSuccessful;

    EdrvCyclicInstance_l.m_dwCycleLenUs = dwCycleLenUs_p;

    return Ret;

}


//---------------------------------------------------------------------------
//
// Function:    EdrvCyclicStartCycle()
//
// Description:
//
// Parameters:
//
// Return:      tEplKernel      = error code
//
// State:       not tested
//
//---------------------------------------------------------------------------

tEplKernel EdrvCyclicStartCycle (void)
{
tEplKernel      Ret = kEplSuccessful;
int                i;

    if (EdrvCyclicInstance_l.m_dwCycleLenUs == 0)
    {
        Ret = kEplEdrvInvalidCycleLen;
        goto Exit;
    }

    //set initial time value for TX Process duration
    EdrvCyclicInstance_l.m_dwTxProcDur = OMETH_US_2_TICKS(EDRVCYC_POS_SHIFT_US);

    //initialize the filter
    for(i=0; i<EDRVCYC_NEGSHFT_FLT_SPAN; i++)
    {
        EdrvCyclicInstance_l.m_aTxProcFlt[i] = OMETH_US_2_TICKS(EDRVCYC_POS_SHIFT_US);
    }
    EdrvCyclicInstance_l.m_uiTxProcFltIndex = 0;

    // clear Tx buffer list
    EdrvCyclicInstance_l.m_uiCurTxBufferList = 0;
    EdrvCyclicInstance_l.m_uiCurTxBufferEntry = 0;
    EPL_MEMSET(EdrvCyclicInstance_l.m_paTxBufferList, 0,
        sizeof (*EdrvCyclicInstance_l.m_paTxBufferList) * EdrvCyclicInstance_l.m_uiMaxTxBufferCount * 2);

    Ret = EplTimerHighReskModifyTimerNs(&EdrvCyclicInstance_l.m_TimerHdlCycle,
        EdrvCyclicInstance_l.m_dwCycleLenUs * 1000ULL,
        EdrvCyclicCbTimerCycle,
        0L,
        FALSE);

    //the next cycle value is not valid!
    EdrvCyclicInstance_l.m_fNextCycleValid = FALSE;

#ifdef SOC_CHECKER
    EdrvCyclicInstance_l.m_dwSocCheckerValid = FALSE;
    socchecker_init();
#endif

Exit:
    return Ret;

}


//---------------------------------------------------------------------------
//
// Function:    EdrvCyclicStopCycle()
//
// Description:
//
// Parameters:
//
// Return:      tEplKernel      = error code
//
// State:       not tested
//
//---------------------------------------------------------------------------

tEplKernel EdrvCyclicStopCycle (void)
{
tEplKernel      Ret = kEplSuccessful;

    Ret = EplTimerHighReskDeleteTimer(&EdrvCyclicInstance_l.m_TimerHdlCycle);
    //Ret = EplTimerHighReskDeleteTimer(&EdrvCyclicInstance_l.m_TimerHdlSlot);

    return Ret;

}


//---------------------------------------------------------------------------
//
// Function:    EdrvCyclicRegSyncHandler()
//
// Description: registers handler for synchronized periodic call back
//
// Parameters:  pfnTimerSynckCbSync_p   = pointer to callback function,
//                                        which will be called in interrupt context.
//
// Return:      tEplKernel      = error code
//
// State:       not tested
//
//---------------------------------------------------------------------------

tEplKernel EdrvCyclicRegSyncHandler (tEdrvCyclicCbSync pfnCbSync_p)
{
tEplKernel      Ret = kEplSuccessful;


    EdrvCyclicInstance_l.m_pfnCbSync = pfnCbSync_p;

    return Ret;

}



//---------------------------------------------------------------------------
//
// Function:    EdrvCyclicRegErrorHandler()
//
// Description: registers handler for error events
//
// Parameters:  pfnTimerSynckCbSync_p   = pointer to callback function,
//                                        which will be called in interrupt context.
//
// Return:      tEplKernel      = error code
//
// State:       not tested
//
//---------------------------------------------------------------------------

tEplKernel EdrvCyclicRegErrorHandler (tEdrvCyclicCbError pfnCbError_p)
{
tEplKernel      Ret = kEplSuccessful;


    EdrvCyclicInstance_l.m_pfnCbError = pfnCbError_p;

    return Ret;

}


//=========================================================================//
//                                                                         //
//          P R I V A T E   F U N C T I O N S                              //
//                                                                         //
//=========================================================================//


//---------------------------------------------------------------------------
//
// Function:    EdrvCyclicCbTimerCycle()
//
// Description: called by timer module. It starts the next cycle.
//
// Parameters:  pEventArg_p             = timer event argument
//
// Returns:     tEplKernel              = error code
//
//
// State:
//
//---------------------------------------------------------------------------

//static tEplKernel PUBLIC EdrvCyclicCbTimerCycle(tEplTimerEventArg* pEventArg_p)
static tEplKernel EdrvCyclicCbTimerCycle(tEplTimerEventArg* pEventArg_p)
{
tEplKernel      Ret = kEplSuccessful;
DWORD            dwMacTimeDiff, dwMacTime1, dwMacTime2; //necessary for negative shift filter
DWORD            dwFltAccu; //used for filter calculation (accumulation)
int                i; //used for for loop

    if (pEventArg_p->m_TimerHdl != EdrvCyclicInstance_l.m_TimerHdlCycle)
    {   // zombie callback
        // just exit
        goto Exit;
    }

    if (EdrvCyclicInstance_l.m_paTxBufferList[EdrvCyclicInstance_l.m_uiCurTxBufferEntry] != NULL)
    {
        Ret = kEplEdrvTxListNotFinishedYet;
        goto Exit;
    }

    EdrvCyclicInstance_l.m_paTxBufferList[EdrvCyclicInstance_l.m_uiCurTxBufferList] = NULL;

    // enter new cycle -> switch Tx buffer list
    EdrvCyclicInstance_l.m_uiCurTxBufferList ^= EdrvCyclicInstance_l.m_uiMaxTxBufferCount;
    EdrvCyclicInstance_l.m_uiCurTxBufferEntry = EdrvCyclicInstance_l.m_uiCurTxBufferList;

    if (EdrvCyclicInstance_l.m_paTxBufferList[EdrvCyclicInstance_l.m_uiCurTxBufferEntry] == NULL)
    {
        Ret = kEplEdrvCurTxListEmpty;
        goto Exit;
    }

    BENCHMARK_MOD_01_SET(2);

    //get timer tick before calling TX Process
    dwMacTime1 = EDRV_GET_MAC_TIME();

    Ret = EdrvCyclicProcessTxBufferList();

    if (Ret != kEplSuccessful)
    {
        goto Exit;
    }

    //get timer tick after calling TX Process
    dwMacTime2 = EDRV_GET_MAC_TIME();

    //obtain absolute difference
    dwMacTimeDiff = dwMacTime2 - dwMacTime1;

    //do filtering
    // add to filter
    EdrvCyclicInstance_l.m_aTxProcFlt[EdrvCyclicInstance_l.m_uiTxProcFltIndex] = dwMacTimeDiff;

    // increment filter index for next entry
    EdrvCyclicInstance_l.m_uiTxProcFltIndex++;
    if( EdrvCyclicInstance_l.m_uiTxProcFltIndex >= EDRVCYC_NEGSHFT_FLT_SPAN )
    {
        EdrvCyclicInstance_l.m_uiTxProcFltIndex = 0;
    }

    // sum all entries
    dwFltAccu = 0U;
    for(i=0; i<EDRVCYC_NEGSHFT_FLT_SPAN; i++)
    {
        dwFltAccu += EdrvCyclicInstance_l.m_aTxProcFlt[i];
    }

    // store average to instance
    EdrvCyclicInstance_l.m_dwTxProcDur = dwFltAccu / EDRVCYC_NEGSHFT_FLT_SPAN;

    BENCHMARK_MOD_01_RESET(2);

    if (EdrvCyclicInstance_l.m_pfnCbSync != NULL)
    {
        BENCHMARK_MOD_01_SET(2);
        Ret = EdrvCyclicInstance_l.m_pfnCbSync();
        BENCHMARK_MOD_01_RESET(2);
    }

Exit:
    if (Ret != kEplSuccessful)
    {

        if (EdrvCyclicInstance_l.m_pfnCbError != NULL)
        {
            Ret = EdrvCyclicInstance_l.m_pfnCbError(Ret, NULL);
        }
    }
    return Ret;
}


//---------------------------------------------------------------------------
//
// Function:    EdrvCyclicProcessTxBufferList()
//
// Description: processes the Tx buffer list.
//
// Parameters:  void
//
// Returns:     tEplKernel              = error code
//
//
// State:
//
//---------------------------------------------------------------------------

static tEplKernel EdrvCyclicProcessTxBufferList(void)
{
tEplKernel      Ret = kEplSuccessful;
tEdrvTxBuffer*  pTxBuffer = NULL;
DWORD            udwAbsTime; //absolute time accumulator
BOOL             fFirstPkt = TRUE; //flag to identify first packet
DWORD            udwMacTimeEntry = EDRV_GET_MAC_TIME(); //MAC TIME AT FUNCTION CALL
//DWORD            udwMacTimePlusOffset = udwMacTimeEntry + EdrvCyclicInstance_l.m_dwTxProcDur; //plus offset
DWORD            udwCycleMin = 0; //absolute minimum cycle time
DWORD            udwCycleMax = 0; //absolute maximum cycle time
DWORD            udwNextOffNs = 0; //next earliest tx time
DWORD            udwNextTimerIrqNs = EdrvCyclicInstance_l.m_dwCycleLenUs * 1000UL; //time of next timer irq

    //BENCHMARK_MOD_01_SET(0);

#ifdef SOC_CHECKER
    //reset SoC Checker before SoC Tx
    socchecker_rst();
#endif

    if( EdrvCyclicInstance_l.m_fNextCycleValid == FALSE )
    {
        //use current time + negative shift to set a valid next cycle
        EdrvCyclicInstance_l.m_dwNextCycleTime = udwMacTimeEntry + OMETH_US_2_TICKS(EDRVCYC_NEG_SHIFT_US);

        //next timer IRQ correction
        udwNextTimerIrqNs -= OMETH_TICKS_2_NS(EdrvCyclicInstance_l.m_dwTxProcDur);

#ifndef SOC_CHECKER
        EdrvCyclicInstance_l.m_fNextCycleValid = TRUE; //very first call is done
#endif
    }
    else
    {
        //cycle is valid, compensate next timer irq

        //calculate time difference of Next SoC - approx. entry of this function
        DWORD udwDiff;
        DWORD udwDiffNs;

        udwDiff = EdrvCyclicInstance_l.m_dwNextCycleTime - udwMacTimeEntry;

        if( udwDiff & 0x80000000UL )
        {
            PRINTF0("\nTX of first cycle packet is in the past!\n");
            PRINTF4(" %s entry: 0x%08X\n SoC TX time: 0x%08X\n Diff: 0x%08X\n",
                    __func__,
                    (int)udwMacTimeEntry,
                    (int)EdrvCyclicInstance_l.m_dwNextCycleTime,
                    (int)udwDiff);

            Ret = kEplEdrvNoFreeBufEntry;
            goto Exit;
        }

        //substract TX buffer list processing from cycle time
        udwNextTimerIrqNs -= OMETH_TICKS_2_NS(EdrvCyclicInstance_l.m_dwTxProcDur);

        udwDiffNs = OMETH_TICKS_2_NS(udwDiff);

        if( udwDiffNs > (EDRVCYC_NEG_SHIFT_US * 1000UL) )
        {
            //time difference is larger negative shift
            udwNextTimerIrqNs += (udwDiffNs - EDRVCYC_NEG_SHIFT_US * 1000UL);
        }
        else
        {
            //time difference is shorter than negative shift
            udwNextTimerIrqNs -= (EDRVCYC_NEG_SHIFT_US * 1000UL - udwDiffNs);
        }
    }

    //set accumulator for cycle window calculation
    udwAbsTime = EdrvCyclicInstance_l.m_dwNextCycleTime; //get cycle start time

    //set limits to verify if time triggered tx is within cycle
    // note: otherwise openMAC would be confused
    udwCycleMin = EdrvCyclicInstance_l.m_dwNextCycleTime; //minimum limit
    udwCycleMax = EdrvCyclicInstance_l.m_dwNextCycleTime + \
            OMETH_US_2_TICKS(EdrvCyclicInstance_l.m_dwCycleLenUs); //maximum limit

    //loop through TX buffer list
    while ((pTxBuffer = EdrvCyclicInstance_l.m_paTxBufferList[EdrvCyclicInstance_l.m_uiCurTxBufferEntry]) != NULL)
    {
        //compare TX buffer time offset with next offset (considers IPG and last packet length)
        //note: otherwise openMAC is confused if time-trig TX starts within other time-trig TX!
        if( (fFirstPkt == FALSE) && (udwNextOffNs > pTxBuffer->m_dwTimeOffsetNs) )
        {
            udwAbsTime += OMETH_NS_2_TICKS(udwNextOffNs); //accumulate offset
        }
        else
        {
            udwAbsTime += OMETH_NS_2_TICKS(pTxBuffer->m_dwTimeOffsetNs); //accumulate offset
        }
        fFirstPkt = FALSE; //first packet is surely out...

        //verify if packet TX start time is within cycle window
        if( (udwAbsTime - udwCycleMin) > (udwCycleMax - udwCycleMin) )
        {
            //packet is outside of the window
            BENCHMARK_MOD_01_TOGGLE(7);
            BENCHMARK_MOD_01_TOGGLE(7);
            BENCHMARK_MOD_01_TOGGLE(7);
            BENCHMARK_MOD_01_TOGGLE(7);

            PRINTF2("\n%s: TimeOffsetUs = %i\n", __func__, (int)pTxBuffer->m_dwTimeOffsetNs/1000U);
            PRINTF3(" min=0x%08X max=0x%08X abs=0x%08X\n", (int)udwCycleMin, (int)udwCycleMax, (int)udwAbsTime);

            Ret = kEplEdrvNoFreeBufEntry;
            goto Exit;
        }

        //pTxBuffer->m_dwTimeOffsetNs = udwAbsTime | 1; //lowest bit enables time triggered send
        //set the absolute TX start time, and OR the lowest bit to give Edrv a hint
        pTxBuffer->m_dwTimeOffsetAbsTk = udwAbsTime | 1; //lowest bit enables time triggered send

        Ret = EdrvSendTxMsg(pTxBuffer);
        if (Ret != kEplSuccessful)
        {
            int i;

            PRINTF2("\n%s: EdrvSendTxMsg ret=0x%X\n", __func__, Ret);
            for(i=0; i<8; i++)
            {
                PRINTF1("%i ", (int)EdrvCyclicInstance_l.m_aTxProcFlt[i]);
            }
            PRINTF1("\n %i\n", (int)EdrvCyclicInstance_l.m_dwTxProcDur);
            goto Exit;
        }

        //set the absolute TX start time to zero
        // -> If the TX buffer is reused as manual TX, EdrvSendTxMsg is not confused!
        pTxBuffer->m_dwTimeOffsetAbsTk = 0;

        //calculate the length of the sent packet, add IPG and thus know the next earliest TX time
        {
            DWORD udwLength = pTxBuffer->m_uiTxMsgLen;

            //consider padding!
            if( udwLength < 60UL )
            {
                udwLength = 60UL;
            }

            // ( pre + header + payload + padding + crc ) * 80ns/byte + 960ns = next TX time
            udwNextOffNs = EDRVCYC_BYTETIME_NS * (EDRVCYC_PREAMB_SIZE + ETH_CRC_SIZE + udwLength) + EDRVCYC_IPG_NS;
        }

        //switch to next TX buffer
        EdrvCyclicInstance_l.m_uiCurTxBufferEntry++;
    }

    //so, we're done with this cycle

    //calculate next cycle
    EdrvCyclicInstance_l.m_dwNextCycleTime += OMETH_US_2_TICKS(EdrvCyclicInstance_l.m_dwCycleLenUs);

    //set up next timer interrupt
    Ret = EplTimerHighReskModifyTimerNs(&EdrvCyclicInstance_l.m_TimerHdlCycle,
        udwNextTimerIrqNs,
        EdrvCyclicCbTimerCycle,
        0L,
        FALSE);

#ifdef SOC_CHECKER
    if(EdrvCyclicInstance_l.m_fNextCycleValid == TRUE)
    {
        if(EdrvCyclicInstance_l.m_dwSocCheckerValid == TRUE)
        {
            DWORD dwSocCheckCnt;
            DWORD dwSocUpperLimit = OMETH_NS_2_TICKS(EdrvCyclicInstance_l.m_dwCycleLenUs * 1000UL + SOC_CHECKER_JITTER_NS/2U);
            DWORD dwSocLowerLimit = OMETH_NS_2_TICKS(EdrvCyclicInstance_l.m_dwCycleLenUs * 1000UL - SOC_CHECKER_JITTER_NS/2U);

            //get SoC Checker tick value
            socchecker_read(&dwSocCheckCnt);

            //verify if the cycle time is within the limit
            if( (dwSocLowerLimit <= dwSocCheckCnt) && (dwSocCheckCnt <= dwSocUpperLimit) )
            {
                socchecker_okay();
            }
            else
            {
                socchecker_error();
                BENCHMARK_MOD_01_TOGGLE(7);
                BENCHMARK_MOD_01_TOGGLE(7);
                BENCHMARK_MOD_01_TOGGLE(7);
            }
        }
        EdrvCyclicInstance_l.m_dwSocCheckerValid = TRUE;
    }

    EdrvCyclicInstance_l.m_fNextCycleValid = TRUE; //very first call is done
#endif

    if (Ret != kEplSuccessful)
    {
        PRINTF2("%s: EplTimerHighReskModifyTimerNs ret=0x%X\n", __func__, Ret);
        goto Exit;
    }

Exit:
    if (Ret != kEplSuccessful)
    {
        BENCHMARK_MOD_01_TOGGLE(7);

        if (EdrvCyclicInstance_l.m_pfnCbError != NULL)
        {
            Ret = EdrvCyclicInstance_l.m_pfnCbError(Ret, pTxBuffer);
        }
    }
    //BENCHMARK_MOD_01_RESET(0);
    return Ret;
}

