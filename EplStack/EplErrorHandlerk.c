/****************************************************************************

  (c) SYSTEC electronic GmbH, D-07973 Greiz, August-Bebel-Str. 29
      www.systec-electronic.com

  Project:      openPOWERLINK

  Description:  source file for error handler module

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

  -------------------------------------------------------------------------

  Revision History:

  2006/10/02 d.k.:   start of the implementation

****************************************************************************/

#include "kernel/EplErrorHandlerk.h"
#include "EplNmt.h"
#include "kernel/EplEventk.h"
#include "kernel/EplObdk.h"         // function prototyps of the EplOBD-Modul
#include "kernel/EplDllk.h"
#include "Benchmark.h"

#if(((EPL_MODULE_INTEGRATION) & (EPL_MODULE_DLLK)) != 0)

#if(((EPL_MODULE_INTEGRATION) & (EPL_MODULE_OBDK)) == 0)
#error "EPL ErrorHandler module needs EPL module OBDK!"
#endif


/***************************************************************************/
/*                                                                         */
/*                                                                         */
/*          G L O B A L   D E F I N I T I O N S                            */
/*                                                                         */
/*                                                                         */
/***************************************************************************/

//---------------------------------------------------------------------------
// const defines
//---------------------------------------------------------------------------

#define EPL_ERRORHANDLERK_CN_LOSS_PRES_EVENT_NONE   0   // not occurred
#define EPL_ERRORHANDLERK_CN_LOSS_PRES_EVENT_OCC    1   // occurred
#define EPL_ERRORHANDLERK_CN_LOSS_PRES_EVENT_THR    2   // threshold exceeded


//---------------------------------------------------------------------------
// local types
//---------------------------------------------------------------------------

typedef struct
{
    DWORD           m_dwCumulativeCnt;  // subindex 1
    DWORD           m_dwThresholdCnt;   // subindex 2
    DWORD           m_dwThreshold;      // subindex 3

} tEplErrorHandlerkErrorCounter;

typedef struct
{
    tEplErrorHandlerkErrorCounter   m_CnLossSoc;    // object 0x1C0B
    tEplErrorHandlerkErrorCounter   m_CnLossPreq;   // object 0x1C0D
    tEplErrorHandlerkErrorCounter   m_CnCrcErr;     // object 0x1C0F
    unsigned long                   m_ulDllErrorEvents;

#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)
    tEplErrorHandlerkErrorCounter   m_MnCrcErr;                     // object 0x1C00
    tEplErrorHandlerkErrorCounter   m_MnCycTimeExceed;              // object 0x1C02
    DWORD                           m_adwMnCnLossPresCumCnt[254];   // object 0x1C07
    DWORD                           m_adwMnCnLossPresThrCnt[254];   // object 0x1C08
    DWORD                           m_adwMnCnLossPresThreshold[254];// object 0x1C09
    BYTE                            m_abMnCnLossPresEvent[254];
#endif

} tEplErrorHandlerkInstance;

//---------------------------------------------------------------------------
// module global vars
//---------------------------------------------------------------------------

static tEplErrorHandlerkInstance EplErrorHandlerkInstance_g;

//---------------------------------------------------------------------------
// local function prototypes
//---------------------------------------------------------------------------

// posts history entry events
static tEplKernel PUBLIC EplErrorHandlerkPostHistoryEntry(tEplErrHistoryEntry* pHistoryEntry_p);

static tEplKernel EplErrorHandlerkLinkErrorCounter(
                                tEplErrorHandlerkErrorCounter* pErrorCounter_p,
                                unsigned int uiIndex_p);

#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)
static tEplKernel EplErrorHandlerkLinkArray(
                                DWORD*      pdwValue_p,
                                unsigned int uiValueCount_p,
                                unsigned int uiIndex_p);
#endif

/***************************************************************************/
/*                                                                         */
/*                                                                         */
/*          C L A S S  <Epl-Kernelspace-Error-Handler>                     */
/*                                                                         */
/*                                                                         */
/***************************************************************************/
//
// Description:
//
//
/***************************************************************************/

//=========================================================================//
//                                                                         //
//          P U B L I C   F U N C T I O N S                                //
//                                                                         //
//=========================================================================//

//---------------------------------------------------------------------------
//
// Function:    EplErrorHandlerkInit
//
// Description: function initialize the first instance
//
//
//
// Parameters:
//
//
// Returns:      tEpKernel  = errorcode
//
//
// State:
//
//---------------------------------------------------------------------------
tEplKernel PUBLIC EplErrorHandlerkInit(void)
{
tEplKernel Ret;


    Ret = EplErrorHandlerkAddInstance();


return Ret;

}


//---------------------------------------------------------------------------
//
// Function:    EplErrorHandlerkAddInstance
//
// Description: function add one more instance
//
//
//
// Parameters:
//
//
// Returns:      tEpKernel  = errorcode
//
//
// State:
//
//---------------------------------------------------------------------------
tEplKernel PUBLIC EplErrorHandlerkAddInstance(void)
{
tEplKernel      Ret;

    Ret = kEplSuccessful;

    // reset only event variable,
    // all other instance members are reset by OD or may keep their current value
    // d.k.: this is necessary for the cumulative counters, which shall not be reset
    EplErrorHandlerkInstance_g.m_ulDllErrorEvents = 0;

    // link counters to OD
    // $$$ d.k. if OD resides in userspace, fetch pointer to shared memory,
    //          which shall have the same structure as the instance (needs to be declared globally).
    //          Other idea: error counter shall belong to the process image
    //          (reset of counters by SDO write are a little bit tricky).

    Ret = EplErrorHandlerkLinkErrorCounter(
            &EplErrorHandlerkInstance_g.m_CnLossSoc,
            0x1C0B);
    if (Ret != kEplSuccessful)
    {
        goto Exit;
    }

    Ret = EplErrorHandlerkLinkErrorCounter(
            &EplErrorHandlerkInstance_g.m_CnLossPreq,
            0x1C0D);
    // ignore return code, because object 0x1C0D is conditional

    Ret = EplErrorHandlerkLinkErrorCounter(
            &EplErrorHandlerkInstance_g.m_CnCrcErr,
            0x1C0F);
    if (Ret != kEplSuccessful)
    {
        goto Exit;
    }

#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)
    Ret = EplErrorHandlerkLinkErrorCounter(
            &EplErrorHandlerkInstance_g.m_MnCrcErr,
            0x1C00);
    if (Ret != kEplSuccessful)
    {
        goto Exit;
    }

    Ret = EplErrorHandlerkLinkErrorCounter(
            &EplErrorHandlerkInstance_g.m_MnCycTimeExceed,
            0x1C02);
    if (Ret != kEplSuccessful)
    {
        goto Exit;
    }

    Ret = EplErrorHandlerkLinkArray(
            EplErrorHandlerkInstance_g.m_adwMnCnLossPresCumCnt,
            tabentries(EplErrorHandlerkInstance_g.m_adwMnCnLossPresCumCnt),
            0x1C07);
    if (Ret != kEplSuccessful)
    {
        goto Exit;
    }

    Ret = EplErrorHandlerkLinkArray(
            EplErrorHandlerkInstance_g.m_adwMnCnLossPresThrCnt,
            tabentries(EplErrorHandlerkInstance_g.m_adwMnCnLossPresThrCnt),
            0x1C08);
    if (Ret != kEplSuccessful)
    {
        goto Exit;
    }

    Ret = EplErrorHandlerkLinkArray(
            EplErrorHandlerkInstance_g.m_adwMnCnLossPresThreshold,
            tabentries(EplErrorHandlerkInstance_g.m_adwMnCnLossPresThreshold),
            0x1C09);
    if (Ret != kEplSuccessful)
    {
        goto Exit;
    }

#endif

Exit:
    return Ret;

}

//---------------------------------------------------------------------------
//
// Function:    EplErrorHandlerkDelInstance
//
// Description: function delete instance an free the bufferstructure
//
//
//
// Parameters:
//
//
// Returns:      tEpKernel  = errorcode
//
//
// State:
//
//---------------------------------------------------------------------------
tEplKernel PUBLIC EplErrorHandlerkDelInstance()
{
tEplKernel      Ret;

    Ret = kEplSuccessful;


return Ret;

}

//---------------------------------------------------------------------------
//
// Function:    EplErrorHandlerkProcess
//
// Description: processes error events from DLL
//
// Parameters:  pEvent_p = pointer to event-structure from buffer
//
// Returns:     tEpKernel  = errorcode
//
// State:
//
//---------------------------------------------------------------------------
tEplKernel PUBLIC EplErrorHandlerkProcess(tEplEvent* pEvent_p)
{
tEplKernel              Ret;
unsigned long           ulDllErrorEvents;
tEplEvent               Event;
tEplNmtEvent            NmtEvent;

    Ret = kEplSuccessful;


    // check m_EventType
    switch(pEvent_p->m_EventType)
    {
        case kEplEventTypeDllError:
        {
        tEplErrorHandlerkEvent* pErrHandlerEvent = (tEplErrorHandlerkEvent*)pEvent_p->m_pArg;
        tEplErrHistoryEntry     HistoryEntry = {0};

            ulDllErrorEvents = pErrHandlerEvent->m_ulDllErrorEvents;

            HistoryEntry.m_wEntryType = EPL_ERR_ENTRYTYPE_MODE_OCCURRED | EPL_ERR_ENTRYTYPE_PROF_EPL | EPL_ERR_ENTRYTYPE_HISTORY;

            // check the several error events
            if ((ulDllErrorEvents & EPL_DLL_ERR_CN_LOSS_SOC) != 0)
            {   // loss of SoC event occurred
                // increment cumulative counter by 1
                EplErrorHandlerkInstance_g.m_CnLossSoc.m_dwCumulativeCnt++;

                if (EplErrorHandlerkInstance_g.m_CnLossSoc.m_dwThreshold > 0)
                {
                    // increment threshold counter by 8
                    EplErrorHandlerkInstance_g.m_CnLossSoc.m_dwThresholdCnt += 8;
                    if (EplErrorHandlerkInstance_g.m_CnLossSoc.m_dwThresholdCnt
                        >= EplErrorHandlerkInstance_g.m_CnLossSoc.m_dwThreshold)
                    {   // threshold is reached
                        // generate error history entry E_DLL_LOSS_SOC_TH
                        HistoryEntry.m_wErrorCode = EPL_E_DLL_LOSS_SOC_TH;
                        HistoryEntry.m_TimeStamp = pEvent_p->m_NetTime;
                        Ret = EplErrorHandlerkPostHistoryEntry(&HistoryEntry);
                        if (Ret != kEplSuccessful)
                        {
                            goto Exit;
                        }

                        BENCHMARK_MOD_02_TOGGLE(7);

                        // post event to NMT state machine
                        NmtEvent = kEplNmtEventNmtCycleError;
                        Event.m_EventSink = kEplEventSinkNmtk;
                        Event.m_EventType = kEplEventTypeNmtEvent;
                        Event.m_pArg = &NmtEvent;
                        Event.m_uiSize = sizeof (NmtEvent);
                        Ret = EplEventkPost(&Event);
                    }
                    EplErrorHandlerkInstance_g.m_ulDllErrorEvents |=
                        EPL_DLL_ERR_CN_LOSS_SOC;
                }
            }

            if ((ulDllErrorEvents & EPL_DLL_ERR_CN_LOSS_PREQ) != 0)
            {   // loss of PReq event occurred
                // increment cumulative counter by 1
                EplErrorHandlerkInstance_g.m_CnLossPreq.m_dwCumulativeCnt++;

                if (EplErrorHandlerkInstance_g.m_CnLossPreq.m_dwThreshold > 0)
                {
                    // increment threshold counter by 8
                    EplErrorHandlerkInstance_g.m_CnLossPreq.m_dwThresholdCnt += 8;
                    if (EplErrorHandlerkInstance_g.m_CnLossPreq.m_dwThresholdCnt
                        >= EplErrorHandlerkInstance_g.m_CnLossPreq.m_dwThreshold)
                    {   // threshold is reached
                        // generate error history entry E_DLL_LOSS_PREQ_TH
                        HistoryEntry.m_wErrorCode = EPL_E_DLL_LOSS_PREQ_TH;
                        HistoryEntry.m_TimeStamp = pEvent_p->m_NetTime;
                        Ret = EplErrorHandlerkPostHistoryEntry(&HistoryEntry);
                        if (Ret != kEplSuccessful)
                        {
                            goto Exit;
                        }

                        BENCHMARK_MOD_02_TOGGLE(7);

                        // post event to NMT state machine
                        NmtEvent = kEplNmtEventNmtCycleError;
                        Event.m_EventSink = kEplEventSinkNmtk;
                        Event.m_EventType = kEplEventTypeNmtEvent;
                        Event.m_pArg = &NmtEvent;
                        Event.m_uiSize = sizeof (NmtEvent);
                        Ret = EplEventkPost(&Event);
                    }
                }
            }

            if ((EplErrorHandlerkInstance_g.m_CnLossPreq.m_dwThresholdCnt > 0)
                && ((ulDllErrorEvents & EPL_DLL_ERR_CN_RECVD_PREQ) != 0))
            {   // PReq correctly received
                // decrement threshold counter by 1
                EplErrorHandlerkInstance_g.m_CnLossPreq.m_dwThresholdCnt--;
            }

            if ((ulDllErrorEvents & EPL_DLL_ERR_CN_CRC) != 0)
            {   // CRC error event occurred
                // increment cumulative counter by 1
                EplErrorHandlerkInstance_g.m_CnCrcErr.m_dwCumulativeCnt++;

                if (EplErrorHandlerkInstance_g.m_CnCrcErr.m_dwThreshold > 0)
                {
                    // increment threshold counter by 8
                    EplErrorHandlerkInstance_g.m_CnCrcErr.m_dwThresholdCnt += 8;
                    if (EplErrorHandlerkInstance_g.m_CnCrcErr.m_dwThresholdCnt
                        >= EplErrorHandlerkInstance_g.m_CnCrcErr.m_dwThreshold)
                    {   // threshold is reached
                        // generate error history entry E_DLL_CRC_TH
                        HistoryEntry.m_wErrorCode = EPL_E_DLL_CRC_TH;
                        HistoryEntry.m_TimeStamp = pEvent_p->m_NetTime;
                        Ret = EplErrorHandlerkPostHistoryEntry(&HistoryEntry);
                        if (Ret != kEplSuccessful)
                        {
                            goto Exit;
                        }

                        BENCHMARK_MOD_02_TOGGLE(7);

                        // post event to NMT state machine
                        NmtEvent = kEplNmtEventNmtCycleError;
                        Event.m_EventSink = kEplEventSinkNmtk;
                        Event.m_EventType = kEplEventTypeNmtEvent;
                        Event.m_pArg = &NmtEvent;
                        Event.m_uiSize = sizeof (NmtEvent);
                        Ret = EplEventkPost(&Event);
                    }
                    EplErrorHandlerkInstance_g.m_ulDllErrorEvents |=
                        EPL_DLL_ERR_CN_CRC;
                }
            }

            if ((ulDllErrorEvents & EPL_DLL_ERR_INVALID_FORMAT) != 0)
            {   // invalid format error occurred (only direct reaction)
                // generate error history entry E_DLL_INVALID_FORMAT
                HistoryEntry.m_wErrorCode = EPL_E_DLL_INVALID_FORMAT;
                HistoryEntry.m_TimeStamp = pEvent_p->m_NetTime;
                AmiSetByteToLe(&HistoryEntry.m_abAddInfo[0], (BYTE) pErrHandlerEvent->m_uiNodeId);
                Ret = EplErrorHandlerkPostHistoryEntry(&HistoryEntry);
                if (Ret != kEplSuccessful)
                {
                    goto Exit;
                }

                BENCHMARK_MOD_02_TOGGLE(7);

#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)
                if (pErrHandlerEvent->m_NmtState >= kEplNmtMsNotActive)
                {   // MN is active
                    if (pErrHandlerEvent->m_uiNodeId != 0)
                    {
                    tEplHeartbeatEvent  HeartbeatEvent;
                    tEplDllNodeOpParam  NodeOpParam;

                        NodeOpParam.m_OpNodeType = kEplDllNodeOpTypeIsochronous;
                        NodeOpParam.m_uiNodeId = pErrHandlerEvent->m_uiNodeId;

                        // remove node from isochronous phase
                        Ret = EplDllkDeleteNode(&NodeOpParam);

                        // inform NmtMnu module about state change, which shall send NMT command ResetNode to this CN
                        HeartbeatEvent.m_uiNodeId = pErrHandlerEvent->m_uiNodeId;
                        HeartbeatEvent.m_NmtState = kEplNmtCsNotActive;
                        HeartbeatEvent.m_wErrorCode = EPL_E_DLL_INVALID_FORMAT;
                        Event.m_EventSink = kEplEventSinkNmtMnu;
                        Event.m_EventType = kEplEventTypeHeartbeat;
                        Event.m_uiSize = sizeof (HeartbeatEvent);
                        Event.m_pArg = &HeartbeatEvent;
                        Ret = EplEventkPost(&Event);
                    }
                    // $$$ and else should lead to InternComError
                }
                else
#endif
                {   // CN is active
                    // post event to NMT state machine
                    NmtEvent = kEplNmtEventInternComError;
                    Event.m_EventSink = kEplEventSinkNmtk;
                    Event.m_EventType = kEplEventTypeNmtEvent;
                    Event.m_pArg = &NmtEvent;
                    Event.m_uiSize = sizeof (NmtEvent);
                    Ret = EplEventkPost(&Event);
                }
            }

#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)
            if ((ulDllErrorEvents & EPL_DLL_ERR_MN_CRC) != 0)
            {   // CRC error event occurred
                // increment cumulative counter by 1
                EplErrorHandlerkInstance_g.m_MnCrcErr.m_dwCumulativeCnt++;

                if (EplErrorHandlerkInstance_g.m_MnCrcErr.m_dwThreshold > 0)
                {
                    // increment threshold counter by 8
                    EplErrorHandlerkInstance_g.m_MnCrcErr.m_dwThresholdCnt += 8;
                    if (EplErrorHandlerkInstance_g.m_MnCrcErr.m_dwThresholdCnt
                        >= EplErrorHandlerkInstance_g.m_MnCrcErr.m_dwThreshold)
                    {   // threshold is reached
                        // generate error history entry E_DLL_CRC_TH
                        HistoryEntry.m_wErrorCode = EPL_E_DLL_CRC_TH;
                        HistoryEntry.m_TimeStamp = pEvent_p->m_NetTime;
                        Ret = EplErrorHandlerkPostHistoryEntry(&HistoryEntry);
                        if (Ret != kEplSuccessful)
                        {
                            goto Exit;
                        }

                        // post event to NMT state machine
                        NmtEvent = kEplNmtEventNmtCycleError;
                        Event.m_EventSink = kEplEventSinkNmtk;
                        Event.m_EventType = kEplEventTypeNmtEvent;
                        Event.m_pArg = &NmtEvent;
                        Event.m_uiSize = sizeof (NmtEvent);
                        Ret = EplEventkPost(&Event);
                    }
                    EplErrorHandlerkInstance_g.m_ulDllErrorEvents |=
                        EPL_DLL_ERR_MN_CRC;
                }
            }

            if ((ulDllErrorEvents & EPL_DLL_ERR_MN_CYCTIMEEXCEED) != 0)
            {   // cycle time exceeded event occurred
                // increment cumulative counter by 1
                EplErrorHandlerkInstance_g.m_MnCycTimeExceed.m_dwCumulativeCnt++;

                if (EplErrorHandlerkInstance_g.m_MnCycTimeExceed.m_dwThreshold > 0)
                {
                    // increment threshold counter by 8
                    EplErrorHandlerkInstance_g.m_MnCycTimeExceed.m_dwThresholdCnt += 8;
                    if (EplErrorHandlerkInstance_g.m_MnCycTimeExceed.m_dwThresholdCnt
                        >= EplErrorHandlerkInstance_g.m_MnCycTimeExceed.m_dwThreshold)
                    {   // threshold is reached
                        // generate error history entry E_DLL_CYCLE_EXCEED_TH
                        HistoryEntry.m_wErrorCode = EPL_E_DLL_CYCLE_EXCEED_TH;
                        HistoryEntry.m_TimeStamp = pEvent_p->m_NetTime;
                        AmiSetWordToLe(&HistoryEntry.m_abAddInfo[0], (WORD) pErrHandlerEvent->m_EplError);
                        Ret = EplErrorHandlerkPostHistoryEntry(&HistoryEntry);
                        if (Ret != kEplSuccessful)
                        {
                            goto Exit;
                        }

                        // post event to NMT state machine
                        NmtEvent = kEplNmtEventNmtCycleError;
                        Event.m_EventSink = kEplEventSinkNmtk;
                        Event.m_EventType = kEplEventTypeNmtEvent;
                        Event.m_pArg = &NmtEvent;
                        Event.m_uiSize = sizeof (NmtEvent);
                        Ret = EplEventkPost(&Event);
                    }
                    else
                    {   // generate error history entry E_DLL_CYCLE_EXCEED
                        HistoryEntry.m_wErrorCode = EPL_E_DLL_CYCLE_EXCEED;
                        HistoryEntry.m_TimeStamp = pEvent_p->m_NetTime;
                        AmiSetWordToLe(&HistoryEntry.m_abAddInfo[0], (WORD) pErrHandlerEvent->m_EplError);
                        Ret = EplErrorHandlerkPostHistoryEntry(&HistoryEntry);
                        if (Ret != kEplSuccessful)
                        {
                            goto Exit;
                        }
                    }

                    EplErrorHandlerkInstance_g.m_ulDllErrorEvents |=
                        EPL_DLL_ERR_MN_CYCTIMEEXCEED;
                }
            }

            if ((ulDllErrorEvents & EPL_DLL_ERR_MN_CN_LOSS_PRES) != 0)
            {   // CN loss PRes event occurred
            unsigned int uiNodeId;

                uiNodeId = pErrHandlerEvent->m_uiNodeId - 1;
                if (uiNodeId < tabentries(EplErrorHandlerkInstance_g.m_adwMnCnLossPresCumCnt))
                {
                    if  (EplErrorHandlerkInstance_g.m_abMnCnLossPresEvent[uiNodeId] == EPL_ERRORHANDLERK_CN_LOSS_PRES_EVENT_NONE)
                    {
                        // increment cumulative counter by 1
                        EplErrorHandlerkInstance_g.m_adwMnCnLossPresCumCnt[uiNodeId]++;

                        if (EplErrorHandlerkInstance_g.m_adwMnCnLossPresThreshold[uiNodeId] > 0)
                        {
                            // increment threshold counter by 8
                            EplErrorHandlerkInstance_g.m_adwMnCnLossPresThrCnt[uiNodeId] += 8;
                            if (EplErrorHandlerkInstance_g.m_adwMnCnLossPresThrCnt[uiNodeId]
                                >= EplErrorHandlerkInstance_g.m_adwMnCnLossPresThreshold[uiNodeId])
                            {   // threshold is reached
                            tEplHeartbeatEvent  HeartbeatEvent;
                            tEplDllNodeOpParam  NodeOpParam;

                                EplErrorHandlerkInstance_g.m_abMnCnLossPresEvent[uiNodeId] = EPL_ERRORHANDLERK_CN_LOSS_PRES_EVENT_THR;

                                NodeOpParam.m_OpNodeType = kEplDllNodeOpTypeIsochronous;
                                NodeOpParam.m_uiNodeId = pErrHandlerEvent->m_uiNodeId;

                                // generate error history entry E_DLL_LOSS_PRES_TH
                                HistoryEntry.m_wErrorCode = EPL_E_DLL_LOSS_PRES_TH;
                                HistoryEntry.m_TimeStamp = pEvent_p->m_NetTime;
                                AmiSetByteToLe(&HistoryEntry.m_abAddInfo[0], (BYTE) pErrHandlerEvent->m_uiNodeId);
                                Ret = EplErrorHandlerkPostHistoryEntry(&HistoryEntry);
                                if (Ret != kEplSuccessful)
                                {
                                    goto Exit;
                                }

                                // remove node from isochronous phase
                                Ret = EplDllkDeleteNode(&NodeOpParam);

                                // inform NmtMnu module about state change, which shall send NMT command ResetNode to this CN
                                HeartbeatEvent.m_uiNodeId = pErrHandlerEvent->m_uiNodeId;
                                HeartbeatEvent.m_NmtState = kEplNmtCsNotActive;
                                HeartbeatEvent.m_wErrorCode = EPL_E_DLL_LOSS_PRES_TH;
                                Event.m_EventSink = kEplEventSinkNmtMnu;
                                Event.m_EventType = kEplEventTypeHeartbeat;
                                Event.m_uiSize = sizeof (HeartbeatEvent);
                                Event.m_pArg = &HeartbeatEvent;
                                Ret = EplEventkPost(&Event);
                            }
                            else
                            {
                                EplErrorHandlerkInstance_g.m_abMnCnLossPresEvent[uiNodeId] = EPL_ERRORHANDLERK_CN_LOSS_PRES_EVENT_OCC;
                            }
                        }
                    }
                }
            }
#endif

            break;
        }

        // unknown type
        default:
        {
            Ret = kEplInvalidEvent;
            break;
        }

    } // end of switch(pEvent_p->m_EventType)

Exit:
    return Ret;

}


//---------------------------------------------------------------------------
//
// Function:    EplErrorHandlerkCycleFinished
//
// Description: cycle has been finished, so threshold counters can be decremented (called by DLL)
//
// Parameters:  fMN_p       = TRUE if local node is MN
//
// Returns:     tEpKernel  = errorcode
//
// State:
//
//---------------------------------------------------------------------------
tEplKernel PUBLIC EplErrorHandlerkCycleFinished(BOOL fMN_p)
{
tEplKernel              Ret = kEplSuccessful;

#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)
    if (fMN_p != FALSE)
    {   // local node is MN -> decrement MN threshold counters
    BYTE*           pbCnNodeId;
    unsigned int    uiNodeId;

        Ret = EplDllkGetCurrentCnNodeIdList(&pbCnNodeId);
        if (Ret != kEplSuccessful)
        {
            goto Exit;
        }
        // iterate through node info structure list
        while (*pbCnNodeId != EPL_C_ADR_INVALID)
        {
            uiNodeId = *pbCnNodeId - 1;
            if (uiNodeId < tabentries(EplErrorHandlerkInstance_g.m_adwMnCnLossPresCumCnt))
            {
                if  (EplErrorHandlerkInstance_g.m_abMnCnLossPresEvent[uiNodeId] == EPL_ERRORHANDLERK_CN_LOSS_PRES_EVENT_NONE)
                {
                    if (EplErrorHandlerkInstance_g.m_adwMnCnLossPresThrCnt[uiNodeId] > 0)
                    {
                        EplErrorHandlerkInstance_g.m_adwMnCnLossPresThrCnt[uiNodeId]--;
                    }
                }
                else if  (EplErrorHandlerkInstance_g.m_abMnCnLossPresEvent[uiNodeId] == EPL_ERRORHANDLERK_CN_LOSS_PRES_EVENT_OCC)
                {
                    EplErrorHandlerkInstance_g.m_abMnCnLossPresEvent[uiNodeId] = EPL_ERRORHANDLERK_CN_LOSS_PRES_EVENT_NONE;
                }
            }
            pbCnNodeId++;
        }

        if ((EplErrorHandlerkInstance_g.m_ulDllErrorEvents & EPL_DLL_ERR_MN_CRC) == 0)
        {   // decrement CRC threshold counter, because it didn't occur last cycle
            if (EplErrorHandlerkInstance_g.m_MnCrcErr.m_dwThresholdCnt > 0)
            {
                EplErrorHandlerkInstance_g.m_MnCrcErr.m_dwThresholdCnt--;
            }
        }

        if ((EplErrorHandlerkInstance_g.m_ulDllErrorEvents & EPL_DLL_ERR_MN_CYCTIMEEXCEED) == 0)
        {   // decrement cycle exceed threshold counter, because it didn't occur last cycle
            if (EplErrorHandlerkInstance_g.m_MnCycTimeExceed.m_dwThresholdCnt > 0)
            {
                EplErrorHandlerkInstance_g.m_MnCycTimeExceed.m_dwThresholdCnt--;
            }
        }
    }
    else
#endif
    {   // local node is CN -> decrement CN threshold counters

        if ((EplErrorHandlerkInstance_g.m_ulDllErrorEvents & EPL_DLL_ERR_CN_LOSS_SOC) == 0)
        {   // decrement loss of SoC threshold counter, because it didn't occur last cycle
            if (EplErrorHandlerkInstance_g.m_CnLossSoc.m_dwThresholdCnt > 0)
            {
                EplErrorHandlerkInstance_g.m_CnLossSoc.m_dwThresholdCnt--;
            }
        }

        if ((EplErrorHandlerkInstance_g.m_ulDllErrorEvents & EPL_DLL_ERR_CN_CRC) == 0)
        {   // decrement CRC threshold counter, because it didn't occur last cycle
            if (EplErrorHandlerkInstance_g.m_CnCrcErr.m_dwThresholdCnt > 0)
            {
                EplErrorHandlerkInstance_g.m_CnCrcErr.m_dwThresholdCnt--;
            }
        }
    }

    // reset error events
    EplErrorHandlerkInstance_g.m_ulDllErrorEvents = 0L;

#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)
Exit:
#endif
    return Ret;

}


//---------------------------------------------------------------------------
//
// Function:    EplErrorHandlerkResetCnError
//
// Description: resets the error flag for the specified CN
//
// Parameters:  uiNodeId_p  = node-ID of CN
//
// Returns:     tEpKernel   = errorcode
//
// State:
//
//---------------------------------------------------------------------------

#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)
tEplKernel PUBLIC EplErrorHandlerkResetCnError(unsigned int uiNodeId_p)
{
tEplKernel              Ret = kEplSuccessful;

    uiNodeId_p--;
    if (uiNodeId_p < tabentries(EplErrorHandlerkInstance_g.m_adwMnCnLossPresCumCnt))
    {
        EplErrorHandlerkInstance_g.m_abMnCnLossPresEvent[uiNodeId_p] = EPL_ERRORHANDLERK_CN_LOSS_PRES_EVENT_NONE;
    }

    return Ret;

}
#endif


//---------------------------------------------------------------------------
//
// Function:    EplErrorHandlerkPostError
//
// Description: posts error events to error handler (called by DLL)
//
// Parameters:  pDllEvent_p = pointer to event-structure
//
// Returns:     tEpKernel  = errorcode
//
// State:
//
//---------------------------------------------------------------------------
tEplKernel PUBLIC EplErrorHandlerkPostError(tEplErrorHandlerkEvent* pDllEvent_p)
{
tEplKernel              Ret;
tEplEvent               Event;

    Event.m_EventSink = kEplEventSinkErrk;
    Event.m_EventType = kEplEventTypeDllError;
    Event.m_uiSize = sizeof (*pDllEvent_p);
    Event.m_pArg = pDllEvent_p;
    Ret = EplEventkPost(&Event);

    return Ret;

}


//=========================================================================//
//                                                                         //
//          P R I V A T E   F U N C T I O N S                              //
//                                                                         //
//=========================================================================//

//---------------------------------------------------------------------------
//
// Function:    EplErrorHandlerkPostHistoryEntry
//
// Description: posts history entry events to API layer
//
// Parameters:  pHistoryEntry_p     = pointer to history entry structure
//
// Returns:     tEpKernel  = errorcode
//
// State:
//
//---------------------------------------------------------------------------
static tEplKernel PUBLIC EplErrorHandlerkPostHistoryEntry(tEplErrHistoryEntry* pHistoryEntry_p)
{
tEplKernel              Ret;
tEplEvent               Event;

    Event.m_EventSink = kEplEventSinkApi;
    Event.m_EventType = kEplEventTypeHistoryEntry;
    Event.m_uiSize = sizeof (*pHistoryEntry_p);
    Event.m_pArg = pHistoryEntry_p;
    Ret = EplEventkPost(&Event);

    return Ret;

}


//---------------------------------------------------------------------------
//
// Function:    EplErrorHandlerkLinkErrorCounter
//
// Description: link specified error counter structure to OD entry
//
// Parameters:  pErrorCounter_p         = pointer to error counter structure
//              uiIndex_p               = OD index
//
// Returns:     tEplKernel              = error code
//
//
// State:
//
//---------------------------------------------------------------------------

static tEplKernel EplErrorHandlerkLinkErrorCounter(
                                tEplErrorHandlerkErrorCounter* pErrorCounter_p,
                                unsigned int uiIndex_p)
{
tEplKernel      Ret = kEplSuccessful;
tEplVarParam        VarParam;

    VarParam.m_pData = &pErrorCounter_p->m_dwCumulativeCnt;
    VarParam.m_Size = sizeof(DWORD);
    VarParam.m_uiIndex = uiIndex_p;
    VarParam.m_uiSubindex = 0x01;
    VarParam.m_ValidFlag = kVarValidAll;
    Ret = EplObdDefineVar(&VarParam);
    if (Ret != kEplSuccessful)
    {
        goto Exit;
    }

    VarParam.m_pData = &pErrorCounter_p->m_dwThresholdCnt;
    VarParam.m_Size = sizeof(DWORD);
    VarParam.m_uiIndex = uiIndex_p;
    VarParam.m_uiSubindex = 0x02;
    VarParam.m_ValidFlag = kVarValidAll;
    Ret = EplObdDefineVar(&VarParam);
    if (Ret != kEplSuccessful)
    {
        goto Exit;
    }

    VarParam.m_pData = &pErrorCounter_p->m_dwThreshold;
    VarParam.m_Size = sizeof(DWORD);
    VarParam.m_uiIndex = uiIndex_p;
    VarParam.m_uiSubindex = 0x03;
    VarParam.m_ValidFlag = kVarValidAll;
    Ret = EplObdDefineVar(&VarParam);
    if (Ret != kEplSuccessful)
    {
        goto Exit;
    }

Exit:
    return Ret;
}

//---------------------------------------------------------------------------
//
// Function:    EplErrorHandlerkLinkErrorCounter
//
// Description: link specified error counter structure to OD entry
//
// Parameters:  pErrorCounter_p         = pointer to error counter structure
//              uiIndex_p               = OD index
//
// Returns:     tEplKernel              = error code
//
//
// State:
//
//---------------------------------------------------------------------------

#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)
static tEplKernel EplErrorHandlerkLinkArray(
                                DWORD*      pdwValue_p,
                                unsigned int uiValueCount_p,
                                unsigned int uiIndex_p)
{
tEplKernel      Ret = kEplSuccessful;
tEplVarParam    VarParam;
tEplObdSize     EntrySize;
BYTE            bIndexEntries;

    EntrySize = (tEplObdSize)  sizeof(bIndexEntries);
    Ret = EplObdReadEntry (
                            uiIndex_p,
                            0x00,
                            (void GENERIC*) &bIndexEntries,
                            &EntrySize );

    if ((Ret != kEplSuccessful) || (bIndexEntries == 0x00))
    {
        // Object doesn't exist or invalid entry number
        Ret = kEplObdIndexNotExist;
        goto Exit;
    }

    if (bIndexEntries < uiValueCount_p)
    {
        uiValueCount_p = bIndexEntries;
    }

    VarParam.m_Size = sizeof(DWORD);
    VarParam.m_uiIndex = uiIndex_p;
    VarParam.m_ValidFlag = kVarValidAll;

    for (VarParam.m_uiSubindex = 0x01; VarParam.m_uiSubindex <= uiValueCount_p; VarParam.m_uiSubindex++)
    {
        VarParam.m_pData = pdwValue_p;
        Ret = EplObdDefineVar(&VarParam);
        if (Ret != kEplSuccessful)
        {
            goto Exit;
        }
        pdwValue_p++;
    }

Exit:
    return Ret;
}
#endif //(((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)


#endif //(((EPL_MODULE_INTEGRATION) & (EPL_MODULE_DLLK)) != 0)

// EOF

