/**
********************************************************************************
\file   dllkframe.c

\brief  Frame processing functions of kernel DLL module

This file contains the frame processing functions of the kernel DLL module.

\ingroup module_dllk
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2013, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
Copyright (c) 2013, SYSTEC electronic GmbH
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
#include "dllk-internal.h"

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

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static tEplKernel processReceivedPreq(tFrameInfo* pFrameInfo_p, tNmtState nmtState_p,
                                      tEdrvReleaseRxBuffer* pReleaseRxBuffer_p);
static tEplKernel processReceivedPres(tFrameInfo* pFrameInfo_p, tNmtState nmtState_p,
                                      tNmtEvent* pNmtEvent_p, tEdrvReleaseRxBuffer* pReleaseRxBuffer_p);
static tEplKernel processReceivedSoc(tEdrvRxBuffer* pRxBuffer_p, tNmtState nmtState_p);
static tEplKernel processReceivedSoa(tEdrvRxBuffer* pRxBuffer_p, tNmtState nmtState_p);
static tEplKernel processReceivedAsnd(tFrameInfo* pFrameInfo_p, tEdrvRxBuffer* pRxBuffer_p,
                                      tNmtState nmtState_p);
static tEplKernel forwardRpdo(tFrameInfo * pFrameInfo_p);

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief  Callback function for received frame

The function implements the callback function to process a received frame.

\param  pRxBuffer_p         Pointer to received frame.

\return The function returns a tEdrvReleaseRxBuffer flag to determine if the
        buffer could be released immediately
\retval kEdrvReleaseRxBufferImmediately     Buffer could be released immediately.
\retval kEdrvReleaseRxBufferLater           Buffer must be released later.
*/
//------------------------------------------------------------------------------
tEdrvReleaseRxBuffer dllk_processFrameReceived(tEdrvRxBuffer * pRxBuffer_p)
{
    tEdrvReleaseRxBuffer    releaseRxBuffer = kEdrvReleaseRxBufferImmediately;
    tEplKernel              ret             = kEplSuccessful;
    tNmtState               nmtState;
    tNmtEvent               nmtEvent        = kNmtEventNoEvent;
    tEplEvent               event;
    tEplFrame*              pFrame;
    tFrameInfo              frameInfo;
    tEplMsgType             msgType;
    TGT_DLLK_DECLARE_FLAGS

    TGT_DLLK_ENTER_CRITICAL_SECTION()

    BENCHMARK_MOD_02_SET(3);
    nmtState = dllkInstance_g.nmtState;
    if (nmtState <= kNmtGsResetConfiguration)
        goto Exit;

    pFrame = (tEplFrame *) pRxBuffer_p->m_pbBuffer;
#if EDRV_EARLY_RX_INT != FALSE
    switch (pRxBuffer_p->m_BufferInFrame)
    {
        case kEdrvBufferFirstInFrame:
        {
            tEdrvTxBuffer*  pTxBuffer = NULL;

            msgType = (tEplMsgType)AmiGetByteFromLe(&pFrame->m_le_bMessageType);
            if (msgType == kEplMsgTypePreq)
            {
                if (dllkInstance_g.dllState == kDllCsWaitPreq)
                {   // PReq expected and actually received
                    // d.k.: The condition above is sufficent, because EPL cycle is active
                    //       and no non-EPL frame shall be received in isochronous phase.
                    // start transmission PRes
                    // $$$ What if Tx buffer is invalid?
                    pTxBuffer = &dllkInstance_g.pTxBuffer[DLLK_TXFRAME_PRES +
                                                          dllkInstance_g.curTxBufferOffsetCycle];
#if (EPL_DLL_PRES_READY_AFTER_SOA != FALSE) || (EPL_DLL_PRES_READY_AFTER_SOC != FALSE)
                    Ret = EdrvTxMsgStart(pTxBuffer);
#else
                    pTxFrame = (tEplFrame *) pTxBuffer->m_pbBuffer;
                    // update frame (NMT state, RD, RS, PR, MS, EN flags)
                    AmiSetByteToLe(&pTxFrame->m_Data.m_Pres.m_le_bNmtStatus, (BYTE) nmtState);
                    AmiSetByteToLe(&pTxFrame->m_Data.m_Pres.m_le_bFlag2, dllkInstance_g.flag2);
                    if (nmtState != kNmtCsOperational)
                    {   // mark PDO as invalid in NMT state Op
                        // $$$ reset only RD flag; set other flags appropriately
                        AmiSetByteToLe(&pTxFrame->m_Data.m_Pres.m_le_bFlag1, 0);
                    }
                    // $$$ make function that updates Pres, StatusRes
                    // send PRes frame
                    Ret = EdrvSendTxMsg(pTxBuffer);
#endif
                }
            }
            goto Exit;
            break;
        }

        case kEdrvBufferMiddleInFrame:
            goto Exit;
            break;

        case kEdrvBufferLastInFrame:
            break;
    }
#endif

    frameInfo.pFrame = pFrame;
    frameInfo.frameSize = pRxBuffer_p->m_uiRxMsgLen;

    if (AmiGetWordFromBe(&pFrame->m_be_wEtherType) != EPL_C_DLL_ETHERTYPE_EPL)
    {   // non-EPL frame
        //TRACE("cbFrameReceived: pfnCbAsync=0x%p SrcMAC=0x%llx\n", dllkInstance_g.pfnCbAsync, AmiGetQword48FromBe(pFrame->m_be_abSrcMac));
        if (dllkInstance_g.pfnCbAsync != NULL)
        {   // handler for async frames is registered
            dllkInstance_g.pfnCbAsync(&frameInfo);
        }
        goto Exit;
    }

    msgType = (tEplMsgType)AmiGetByteFromLe(&pFrame->m_le_bMessageType);
    switch (msgType)
    {
        case kEplMsgTypePreq:
            if (AmiGetByteFromLe(&pFrame->m_le_bDstNodeId) != dllkInstance_g.dllConfigParam.nodeId)
            {   // this PReq is not intended for us
                goto Exit;
            }
            nmtEvent = kNmtEventDllCePreq;
            ret = processReceivedPreq(&frameInfo, nmtState, &releaseRxBuffer);
            if (ret != kEplSuccessful)
                goto Exit;
            break;

        case kEplMsgTypePres:
            ret = processReceivedPres(&frameInfo, nmtState, &nmtEvent, &releaseRxBuffer);
            if (ret != kEplSuccessful)
                goto Exit;
            break;

        case kEplMsgTypeSoc:
            nmtEvent = kNmtEventDllCeSoc;
            ret = processReceivedSoc(pRxBuffer_p, nmtState);
            if (ret != kEplSuccessful)
                goto Exit;
            break;

        case kEplMsgTypeSoa:
            nmtEvent = kNmtEventDllCeSoa;
            ret = processReceivedSoa(pRxBuffer_p, nmtState);
            if (ret != kEplSuccessful)
                goto Exit;
            break;

        case kEplMsgTypeAsnd:
            nmtEvent = kNmtEventDllCeAsnd;
            ret = processReceivedAsnd(&frameInfo, pRxBuffer_p, nmtState);
            if (ret != kEplSuccessful)
                goto Exit;
            break;

        default:
            break;
    }

    if (nmtEvent != kNmtEventNoEvent)
    {   // event for DLL and NMT state machine generated
        ret = dllk_changeState(nmtEvent, nmtState);
        if (ret != kEplSuccessful)
            goto Exit;

        if ((nmtEvent != kNmtEventDllCeAsnd) &&
            ((nmtState <= kNmtCsPreOperational1) || (nmtEvent != kNmtEventDllCePres)))
        {   // NMT state machine is not interested in ASnd frames and PRes frames when not CsNotActive or CsPreOp1
            // inform NMT module
            event.m_EventSink = kEplEventSinkNmtk;
            event.m_EventType = kEplEventTypeNmtEvent;
            event.m_uiSize = sizeof (nmtEvent);
            event.m_pArg = &nmtEvent;
            ret = eventk_postEvent(&event);
        }
    }

Exit:
    if (ret != kEplSuccessful)
    {
        UINT32      arg;

        BENCHMARK_MOD_02_TOGGLE(7);
        arg = dllkInstance_g.dllState | (nmtEvent << 8);
        // Error event for API layer
        ret = eventk_postError(kEplEventSourceDllk, ret, sizeof(arg), &arg);
    }
    BENCHMARK_MOD_02_RESET(3);
    TGT_DLLK_LEAVE_CRITICAL_SECTION()

    return releaseRxBuffer;
}

//------------------------------------------------------------------------------
/**
\brief  Callback function for transmitted NMT request frame

The function implements the callback function which is called when a NMT request
frame was transmitted.

\param  pTxBuffer_p         Pointer to TxBuffer structure of transmitted frame.

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
void dllk_processTransmittedNmtReq(tEdrvTxBuffer * pTxBuffer_p)
{
    tEplKernel              ret = kEplSuccessful;
    tEplEvent               event;
    tDllAsyncReqPriority    priority;
    tNmtState               nmtState;
    UINT                    handle = DLLK_TXFRAME_NMTREQ;
    UINT32                  arg;

    TGT_DLLK_DECLARE_FLAGS

    TGT_DLLK_ENTER_CRITICAL_SECTION()

    nmtState = dllkInstance_g.nmtState;
    if (nmtState <= kNmtGsResetConfiguration)
        goto Exit;

#if defined(CONFIG_INCLUDE_NMT_MN)
    if (nmtState >= kNmtMsNotActive)
    {
        tEplFrame*      pTxFrame;

        // check if this frame is a NMT command,
        // then forward this frame back to NmtMnu module,
        // because it needs the time, when this frame is
        // actually sent, to start the timer for monitoring
        // the NMT state change.
        pTxFrame = (tEplFrame *) pTxBuffer_p->m_pbBuffer;
        if ((AmiGetByteFromLe(&pTxFrame->m_le_bMessageType) == (UINT8) kEplMsgTypeAsnd) &&
            (AmiGetByteFromLe(&pTxFrame->m_Data.m_Asnd.m_le_bServiceId) == (UINT8) kDllAsndNmtCommand))
        {   // post event directly to NmtMnu module
            event.m_EventSink = kEplEventSinkNmtMnu;
            event.m_EventType = kEplEventTypeNmtMnuNmtCmdSent;
            event.m_uiSize = pTxBuffer_p->m_uiTxMsgLen;
            event.m_pArg = pTxFrame;
            //PRINTF("%s TxB=%p, TxF=%p, s=%u\n", __func__, pTxBuffer_p, event.m_pArg, event.m_uiSize);
            ret = eventk_postEvent(&event);
            if (ret != kEplSuccessful)
                goto Exit;
        }
    }
#endif

    // frame from NMT request FIFO sent
    // mark Tx-buffer as empty
    pTxBuffer_p->m_uiTxMsgLen = DLLK_BUFLEN_EMPTY;

    // post event to DLL
    priority = kDllAsyncReqPrioNmt;
    event.m_EventSink = kEplEventSinkDllk;
    event.m_EventType = kEplEventTypeDllkFillTx;
    EPL_MEMSET(&event.m_NetTime, 0x00, sizeof(event.m_NetTime));
    event.m_pArg = &priority;
    event.m_uiSize = sizeof(priority);
    ret = eventk_postEvent(&event);
    if (ret != kEplSuccessful)
        goto Exit;

Exit:
    if (ret != kEplSuccessful)
    {
        BENCHMARK_MOD_02_TOGGLE(7);
        arg = dllkInstance_g.dllState | (handle << 16);
        ret = eventk_postError(kEplEventSourceDllk, ret, sizeof(arg), &arg);
    }

    TGT_DLLK_LEAVE_CRITICAL_SECTION()
    return;
}

//------------------------------------------------------------------------------
/**
\brief  Callback function for transmitted Non POWERLINK frame

The function implements the callback function which is called when a NON
POWERLINK frame was transmitted.

\param  pTxBuffer_p         Pointer to TxBuffer structure of transmitted frame.

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
void dllk_processTransmittedNonEpl(tEdrvTxBuffer * pTxBuffer_p)
{
    tEplKernel              ret = kEplSuccessful;
    tEplEvent               event;
    tDllAsyncReqPriority    priority;
    tNmtState               nmtState;
    UINT                    handle = DLLK_TXFRAME_NONEPL;
    UINT32                  arg;

    TGT_DLLK_DECLARE_FLAGS

    TGT_DLLK_ENTER_CRITICAL_SECTION()

    nmtState = dllkInstance_g.nmtState;
    if (nmtState <= kNmtGsResetConfiguration)
        goto Exit;

    // frame from generic priority FIFO sent
    // mark Tx-buffer as empty
    pTxBuffer_p->m_uiTxMsgLen = DLLK_BUFLEN_EMPTY;

    // post event to DLL
    priority = kDllAsyncReqPrioGeneric;
    event.m_EventSink = kEplEventSinkDllk;
    event.m_EventType = kEplEventTypeDllkFillTx;
    EPL_MEMSET(&event.m_NetTime, 0x00, sizeof(event.m_NetTime));
    event.m_pArg = &priority;
    event.m_uiSize = sizeof(priority);
    ret = eventk_postEvent(&event);

Exit:
    if (ret != kEplSuccessful)
    {
        BENCHMARK_MOD_02_TOGGLE(7);
        arg = dllkInstance_g.dllState | (handle << 16);
        ret = eventk_postError(kEplEventSourceDllk, ret, sizeof(arg), &arg);
    }
    TGT_DLLK_LEAVE_CRITICAL_SECTION()
    return;
}

#if defined(CONFIG_INCLUDE_NMT_MN)
//------------------------------------------------------------------------------
/**
\brief  Callback function for transmitted SoC frame

The function implements the callback function which is called when a SoC
frame was transmitted.

\param  pTxBuffer_p         Pointer to TxBuffer structure of transmitted frame.

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
void dllk_processTransmittedSoc(tEdrvTxBuffer * pTxBuffer_p)
{
    tEplKernel      ret = kEplSuccessful;
    tNmtState       nmtState;
    UINT            handle = DLLK_TXFRAME_SOC;
    UINT32          arg;

    TGT_DLLK_DECLARE_FLAGS

    UNUSED_PARAMETER(pTxBuffer_p);

    TGT_DLLK_ENTER_CRITICAL_SECTION()

    nmtState = dllkInstance_g.nmtState;
    if (nmtState <= kNmtGsResetConfiguration)
        goto Exit;

    // SoC frame sent
    ret = dllk_changeState(kNmtEventDllMeAsndTimeout, nmtState);
    if (ret != kEplSuccessful)
        goto Exit;

Exit:
    if (ret != kEplSuccessful)
    {
        BENCHMARK_MOD_02_TOGGLE(7);
        arg = dllkInstance_g.dllState | (handle << 16);
        ret = eventk_postError(kEplEventSourceDllk, ret, sizeof(arg), &arg);
    }
    TGT_DLLK_LEAVE_CRITICAL_SECTION()
    return;
}
#endif

#if defined(CONFIG_INCLUDE_NMT_MN)
//------------------------------------------------------------------------------
/**
\brief  Callback function for transmitted SoA frame

The function implements the callback function which is called when a SoA
frame was transmitted.

\param  pTxBuffer_p         Pointer to TxBuffer structure of transmitted frame.

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
void dllk_processTransmittedSoa(tEdrvTxBuffer * pTxBuffer_p)
{
    tEplKernel      ret = kEplSuccessful;
    tNmtState       nmtState;
    UINT            handle = DLLK_TXFRAME_SOA;
    UINT32          arg;

    TGT_DLLK_DECLARE_FLAGS

    UNUSED_PARAMETER(pTxBuffer_p);

    TGT_DLLK_ENTER_CRITICAL_SECTION()

    nmtState = dllkInstance_g.nmtState;
    if (nmtState <= kNmtGsResetConfiguration)
        goto Exit;

    // SoA frame sent
    // check if we are invited
    // old handling only in PreOp1
    if ((dllkInstance_g.dllState == kDllMsNonCyclic) &&
        (dllkInstance_g.aLastTargetNodeId[dllkInstance_g.curLastSoaReq] == dllkInstance_g.dllConfigParam.nodeId))
    {
        switch (dllkInstance_g.aLastReqServiceId[dllkInstance_g.curLastSoaReq])
        {
            case kDllReqServiceStatus:
                if (dllkInstance_g.pTxBuffer[DLLK_TXFRAME_STATUSRES + dllkInstance_g.curTxBufferOffsetStatusRes].m_pbBuffer != NULL)
                {   // StatusRes does exist
                    // send StatusRes
                    ret = EdrvSendTxMsg(&dllkInstance_g.pTxBuffer[DLLK_TXFRAME_STATUSRES + dllkInstance_g.curTxBufferOffsetStatusRes]);
                    if (ret != kEplSuccessful)
                        goto Exit;
                    TGT_DBG_SIGNAL_TRACE_POINT(8);
                }
                break;

            case kDllReqServiceIdent:
                if (dllkInstance_g.pTxBuffer[DLLK_TXFRAME_IDENTRES + dllkInstance_g.curTxBufferOffsetIdentRes].m_pbBuffer != NULL)
                {   // IdentRes does exist
                    // send IdentRes
                    ret = EdrvSendTxMsg(&dllkInstance_g.pTxBuffer[DLLK_TXFRAME_IDENTRES + dllkInstance_g.curTxBufferOffsetIdentRes]);
                    if (ret != kEplSuccessful)
                        goto Exit;
                    TGT_DBG_SIGNAL_TRACE_POINT(7);
                }
                break;

            case kDllReqServiceNmtRequest:
                if (dllkInstance_g.pTxBuffer[DLLK_TXFRAME_NMTREQ + dllkInstance_g.curTxBufferOffsetNmtReq].m_pbBuffer != NULL)
                {   // NmtRequest does exist
                    // check if frame is not empty and not being filled
                    if (dllkInstance_g.pTxBuffer[DLLK_TXFRAME_NMTREQ + dllkInstance_g.curTxBufferOffsetNmtReq].m_uiTxMsgLen > DLLK_BUFLEN_FILLING)
                    {
                        // send NmtRequest
                        ret = EdrvSendTxMsg(&dllkInstance_g.pTxBuffer[DLLK_TXFRAME_NMTREQ + dllkInstance_g.curTxBufferOffsetNmtReq]);
                        if (ret != kEplSuccessful)
                            goto Exit;
                        dllkInstance_g.curTxBufferOffsetNmtReq ^= 1;
                    }
                }
                break;

            case kDllReqServiceUnspecified:
                if (dllkInstance_g.pTxBuffer[DLLK_TXFRAME_NONEPL + dllkInstance_g.curTxBufferOffsetNonEpl].m_pbBuffer != NULL)
                {   // non-EPL frame does exist
                    // check if frame is not empty and not being filled
                    if (dllkInstance_g.pTxBuffer[DLLK_TXFRAME_NONEPL + dllkInstance_g.curTxBufferOffsetNonEpl].m_uiTxMsgLen > DLLK_BUFLEN_FILLING)
                    {
                        // send non-EPL frame
                        ret = EdrvSendTxMsg(&dllkInstance_g.pTxBuffer[DLLK_TXFRAME_NONEPL + dllkInstance_g.curTxBufferOffsetNonEpl]);
                        if (ret != kEplSuccessful)
                            goto Exit;

                        dllkInstance_g.curTxBufferOffsetNonEpl ^= 1;
                    }
                }
                break;

            default:
                break;
        }

        // ASnd frame was sent, remove the request
        dllkInstance_g.aLastReqServiceId[dllkInstance_g.curLastSoaReq] = kDllReqServiceNo;
    }

    // reprogram timer in PREOP1
#if EPL_TIMER_USE_HIGHRES != FALSE
    if ((dllkInstance_g.dllState == kDllMsNonCyclic) &&
        (dllkInstance_g.dllConfigParam.asyncSlotTimeout != 0))
    {
        ret = EplTimerHighReskModifyTimerNs(&dllkInstance_g.timerHdlCycle,
                                            dllkInstance_g.dllConfigParam.asyncSlotTimeout,
                                            dllk_cbMnTimerCycle, 0L, FALSE);
        if (ret != kEplSuccessful)
            goto Exit;

        // forward event to ErrorHandler and DLLk module
        ret = dllk_postEvent(kEplEventTypeDllkCycleFinish);
        if (ret != kEplSuccessful)
            goto Exit;
    }
#endif

    if ((dllkInstance_g.dllState > kDllMsNonCyclic) &&
        (dllkInstance_g.dllConfigParam.syncNodeId > EPL_C_ADR_SYNC_ON_SOC) &&
        (dllkInstance_g.fSyncProcessed == FALSE))
    {   // cyclic state is active, so preprocessing is necessary
        dllkInstance_g.fSyncProcessed = TRUE;
        ret = dllk_postEvent(kEplEventTypeSync);
        if (ret != kEplSuccessful)
            goto Exit;
    }

Exit:
    if (ret != kEplSuccessful)
    {
        BENCHMARK_MOD_02_TOGGLE(7);
        arg = dllkInstance_g.dllState | (handle << 16);
        ret = eventk_postError(kEplEventSourceDllk, ret, sizeof(arg), &arg);
    }
    TGT_DLLK_LEAVE_CRITICAL_SECTION()
    return;
}
#endif

//------------------------------------------------------------------------------
/**
\brief  Update IdentResponse frame

The function updates a IdentResponse frame with the specified information.

\param  pTxBuffer_p         Pointer to TX buffer of frame.
\param  nmtState_p          NMT state of node.

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
tEplKernel dllk_updateFrameIdentRes(tEdrvTxBuffer* pTxBuffer_p, tNmtState nmtState_p)
{
    tEplKernel      ret = kEplSuccessful;
    tEplFrame*      pTxFrame;

    pTxFrame = (tEplFrame *) pTxBuffer_p->m_pbBuffer;

    // update frame (NMT state, RD, RS, PR flags)
    AmiSetByteToLe(&pTxFrame->m_Data.m_Asnd.m_Payload.m_IdentResponse.m_le_bNmtStatus, (UINT8)nmtState_p);
    AmiSetByteToLe(&pTxFrame->m_Data.m_Asnd.m_Payload.m_IdentResponse.m_le_bFlag2, dllkInstance_g.flag2);

#if (EDRV_AUTO_RESPONSE != FALSE)
    if (nmtState_p < kNmtMsNotActive)
    {
        ret = EdrvUpdateTxMsgBuffer(pTxBuffer_p);
    }
#endif

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Update StatusResponse frame

The function updates a StatusResponse frame with the specified information.

\param  pTxBuffer_p         Pointer to TX buffer of frame.
\param  nmtState_p          NMT state of node.

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
tEplKernel dllk_updateFrameStatusRes(tEdrvTxBuffer* pTxBuffer_p, tNmtState nmtState_p)
{
    tEplKernel      ret = kEplSuccessful;
    tEplFrame*      pTxFrame;

    pTxFrame = (tEplFrame *) pTxBuffer_p->m_pbBuffer;

    // update frame (NMT state, RD, RS, PR, EC, EN flags)
    AmiSetByteToLe(&pTxFrame->m_Data.m_Asnd.m_Payload.m_StatusResponse.m_le_bNmtStatus, (UINT8)nmtState_p);
    AmiSetByteToLe(&pTxFrame->m_Data.m_Asnd.m_Payload.m_StatusResponse.m_le_bFlag2, dllkInstance_g.flag2);
    AmiSetByteToLe(&pTxFrame->m_Data.m_Asnd.m_Payload.m_StatusResponse.m_le_bFlag1, dllkInstance_g.flag1);

#if (EDRV_AUTO_RESPONSE != FALSE)
    if (nmtState_p < kNmtMsNotActive)
    {
        ret = EdrvUpdateTxMsgBuffer(pTxBuffer_p);
    }
#endif

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Update PRes frame

The function updates a PRes frame with the specified information.

\param  pTxBuffer_p         Pointer to TX buffer of frame.
\param  nmtState_p          NMT state of node.

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
tEplKernel dllk_updateFramePres(tEdrvTxBuffer* pTxBuffer_p, tNmtState nmtState_p)
{
    tEplKernel      ret = kEplSuccessful;
    tEplFrame*      pTxFrame;
    UINT8           flag1;

    pTxFrame = (tEplFrame *) pTxBuffer_p->m_pbBuffer;

    // update frame (NMT state, RD, RS, PR, MS, EN flags)
    AmiSetByteToLe(&pTxFrame->m_Data.m_Pres.m_le_bNmtStatus, (BYTE) nmtState_p);
    AmiSetByteToLe(&pTxFrame->m_Data.m_Pres.m_le_bFlag2, dllkInstance_g.flag2);

    // get RD flag
    flag1 = AmiGetByteFromLe(&pTxFrame->m_Data.m_Pres.m_le_bFlag1) & EPL_FRAME_FLAG1_RD;

    if ( (dllkInstance_g.dllConfigParam.multipleCycleCnt > 0) &&
         (dllkInstance_g.mnFlag1 & EPL_FRAME_FLAG1_MS) ) // MS flag set in PReq
    {   // set MS flag, because PRes will be sent multiplexed with other CNs
        flag1 |= EPL_FRAME_FLAG1_MS;
    }

    // add EN flag from Error signaling module
    flag1 |= dllkInstance_g.flag1 & EPL_FRAME_FLAG1_EN;

    if (nmtState_p != kNmtCsOperational)
    {   // mark PDO as invalid in all NMT states but OPERATIONAL - reset only RD flag
        flag1 &= ~EPL_FRAME_FLAG1_RD;
    }
    AmiSetByteToLe(&pTxFrame->m_Data.m_Pres.m_le_bFlag1, flag1);        // update frame (flag1)

#if (EDRV_AUTO_RESPONSE != FALSE)
//    if (NmtState_p < kNmtMsNotActive)
    {   // currently, this function is only called on CN
        ret = EdrvUpdateTxMsgBuffer(pTxBuffer_p);
    }
#endif

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Check frame

The function checks a frame and sets the missing information.

\param  pFrame_p            Pointer to frame.
\param  frameSize_p         Size of the frame

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
tEplKernel dllk_checkFrame(tEplFrame * pFrame_p, UINT frameSize_p)
{
    tEplMsgType     MsgType;
    UINT16          etherType;

    UNUSED_PARAMETER(frameSize_p);

    if (pFrame_p != NULL)
    {
        // check SrcMAC
        if (AmiGetQword48FromBe(pFrame_p->m_be_abSrcMac) == 0)
        {
            // source MAC address
            EPL_MEMCPY(&pFrame_p->m_be_abSrcMac[0], &dllkInstance_g.aLocalMac[0], 6);
        }

        // check ethertype
        etherType = AmiGetWordFromBe(&pFrame_p->m_be_wEtherType);
        if (etherType == 0)
        {
            // assume EPL frame
            etherType = EPL_C_DLL_ETHERTYPE_EPL;
            AmiSetWordToBe(&pFrame_p->m_be_wEtherType, etherType);
        }

        if (etherType == EPL_C_DLL_ETHERTYPE_EPL)
        {
            // source node ID
            AmiSetByteToLe(&pFrame_p->m_le_bSrcNodeId, (BYTE) dllkInstance_g.dllConfigParam.nodeId);

            // check message type
            MsgType = AmiGetByteFromLe(&pFrame_p->m_le_bMessageType);
            if (MsgType == 0)
            {
                MsgType = kEplMsgTypeAsnd;
                AmiSetByteToLe(&pFrame_p->m_le_bMessageType, (BYTE) MsgType);
            }

            if (MsgType == kEplMsgTypeAsnd)
            {
                // destination MAC address
                AmiSetQword48ToBe(&pFrame_p->m_be_abDstMac[0], EPL_C_DLL_MULTICAST_ASND);
            }
        }
    }

    return kEplSuccessful;
}

#if defined (CONFIG_INCLUDE_NMT_MN)
//------------------------------------------------------------------------------
/**
\brief  Update and transmit SoA

The function updates and transmits a SoA.

\param  nmtState_p              Current NMT state.
\param  pDllStateProposed_p     Proposed DLL state.
\param  fEnableInvitation_p     Enable invitation for asynchronous phase. It
                                will be disabled for EPL_C_DLL_PREOP1_START_CYCLES
                                SoAs.^

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
tEplKernel dllk_mnSendSoa(tNmtState nmtState_p, tDllState* pDllStateProposed_p, BOOL fEnableInvitation_p)
{
    tEplKernel      ret = kEplSuccessful;
    tEdrvTxBuffer  *pTxBuffer = NULL;

    *pDllStateProposed_p = kDllMsNonCyclic;

    pTxBuffer = &dllkInstance_g.pTxBuffer[DLLK_TXFRAME_SOA];
    if (pTxBuffer->m_pbBuffer != NULL)
    {   // SoA does exist
        ret = dllk_updateFrameSoa(pTxBuffer, nmtState_p, fEnableInvitation_p, dllkInstance_g.curLastSoaReq);
        if (ret != kEplSuccessful)
            return ret;

        if (dllkInstance_g.aLastReqServiceId[dllkInstance_g.curLastSoaReq] != kDllReqServiceNo)
        {   // asynchronous phase will be assigned to one node
            if (dllkInstance_g.aLastTargetNodeId[dllkInstance_g.curLastSoaReq] ==
                                  dllkInstance_g.dllConfigParam.nodeId)
            {   // d.k. DLL state WaitAsndTrig is not helpful;
                //      so just step over to WaitSocTrig,
                //      because own ASnd is sent automatically in CbFrameTransmitted() after SoA.
                //*pDllStateProposed_p = kDllMsWaitAsndTrig;
                *pDllStateProposed_p = kDllMsWaitSocTrig;
            }
            else
            {   // assignment to CN
                *pDllStateProposed_p = kDllMsWaitAsnd;
            }
        }
        else
        {   // no assignment of asynchronous phase
            *pDllStateProposed_p = kDllMsWaitSocTrig;
        }

        // send SoA frame
        ret = EdrvSendTxMsg(pTxBuffer);
    }
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Update SoA frame

The function updates a SoA frame.

\param  pTxBuffer_p             Pointer to TX buffer of frame.
\param  nmtState_p              NMT state of the local node.
\param  fEnableInvitation_p     Enable the invitation for asynchronous phase.
                                It will be disabled for the first EPL_C_DLL_PREOP1_START_CYCLES
                                SoAs
\param  curReq_p                Index of current request.

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
tEplKernel dllk_updateFrameSoa(tEdrvTxBuffer* pTxBuffer_p, tNmtState nmtState_p,
                               BOOL fEnableInvitation_p, UINT8 curReq_p)
{
    tEplKernel          ret = kEplSuccessful;
    tEplFrame*          pTxFrame;
    tDllkNodeInfo*      pNodeInfo;

    pTxFrame = (tEplFrame *) pTxBuffer_p->m_pbBuffer;

    if (fEnableInvitation_p != FALSE)
    {   // fetch target of asynchronous phase
        if (dllkInstance_g.flag2 == 0)
        {   // own queues are empty
            dllkInstance_g.aLastReqServiceId[curReq_p] = kDllReqServiceNo;
        }
        else if (((tDllAsyncReqPriority) (dllkInstance_g.flag2 >> EPL_FRAME_FLAG2_PR_SHIFT)) == kDllAsyncReqPrioNmt)
        {   // frames in own NMT request queue available
            dllkInstance_g.aLastReqServiceId[curReq_p] = kDllReqServiceNmtRequest;
        }
        else
        {
            dllkInstance_g.aLastReqServiceId[curReq_p] = kDllReqServiceUnspecified;
        }
        ret = dllkcal_getSoaRequest(&dllkInstance_g.aLastReqServiceId[curReq_p],
                                           &dllkInstance_g.aLastTargetNodeId[curReq_p],
                                           &pTxFrame->m_Data.m_Soa.m_Payload);
        if (ret != kEplSuccessful)
            return ret;

        if (dllkInstance_g.aLastReqServiceId[curReq_p] != kDllReqServiceNo)
        {   // asynchronous phase will be assigned to one node
            if (dllkInstance_g.aLastTargetNodeId[curReq_p] == EPL_C_ADR_INVALID)
            {   // exchange invalid node ID with local node ID
                dllkInstance_g.aLastTargetNodeId[curReq_p] = dllkInstance_g.dllConfigParam.nodeId;
            }

            pNodeInfo = dllk_getNodeInfo(dllkInstance_g.aLastTargetNodeId[curReq_p]);
            if (pNodeInfo == NULL)
            {   // no node info structure available
                ret = kEplDllNoNodeInfo;
                return ret;
            }

            // update frame (EA, ER flags)
            AmiSetByteToLe(&pTxFrame->m_Data.m_Soa.m_le_bFlag1,
                pNodeInfo->soaFlag1 & (EPL_FRAME_FLAG1_EA | EPL_FRAME_FLAG1_ER));
        }
        else
        {   // no assignment of asynchronous phase
            dllkInstance_g.aLastTargetNodeId[curReq_p] = EPL_C_ADR_INVALID;
        }
    }
    else
    {   // invite nobody
        dllkInstance_g.aLastReqServiceId[curReq_p] = kDllReqServiceNo;
        dllkInstance_g.aLastTargetNodeId[curReq_p] = EPL_C_ADR_INVALID;
    }

    // update frame (target)
    AmiSetByteToLe(&pTxFrame->m_Data.m_Soa.m_le_bReqServiceId,
                   (UINT8) dllkInstance_g.aLastReqServiceId[curReq_p]);
    AmiSetByteToLe(&pTxFrame->m_Data.m_Soa.m_le_bReqServiceTarget,
                   (UINT8) dllkInstance_g.aLastTargetNodeId[curReq_p]);
    // update frame (NMT state)
    AmiSetByteToLe(&pTxFrame->m_Data.m_Soa.m_le_bNmtStatus, (BYTE) nmtState_p);

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Pass empty ASnd frame to receive FIFO.

The function passes an empty ASnd frame to the receive FIFO. It will be called
only for frames with registered AsndServiceIds (only kDllAsndFilterAny).

\param  reqServiceId_p  Requested service ID.
\param  nodeId_p        Node ID.


\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
tEplKernel dllk_asyncFrameNotReceived(tDllReqServiceId reqServiceId_p, UINT nodeId_p)
{
    tEplKernel      Ret = kEplSuccessful;
    BYTE            abBuffer[18];
    tEplFrame*      pFrame = (tEplFrame*) abBuffer;
    tFrameInfo      FrameInfo;

    // check if previous SoA invitation was not answered
    switch (reqServiceId_p)
    {
        case kDllReqServiceIdent:
        case kDllReqServiceStatus:
#if (EPL_DLL_PRES_CHAINING_MN != FALSE)
        case kDllReqServiceSync:
#endif
            // ASnd service registered?
            if (dllkInstance_g.aAsndFilter[reqServiceId_p] == kDllAsndFilterAny)
            {   // ASnd service ID is registered
                AmiSetByteToLe(&pFrame->m_le_bSrcNodeId, (BYTE) nodeId_p);
                AmiSetByteToLe(&pFrame->m_le_bMessageType, (BYTE) kEplMsgTypeAsnd);
                AmiSetByteToLe(&pFrame->m_Data.m_Asnd.m_le_bServiceId, (BYTE) reqServiceId_p);

                FrameInfo.pFrame = pFrame;
                FrameInfo.frameSize = 18;   // empty non existing ASnd frame
                // forward frame via async receive FIFO to userspace
                Ret = dllkcal_asyncFrameReceived(&FrameInfo);
            }
            break;

        default:
            // no invitation issued or it was successfully answered or it is uninteresting
            break;
    }
    return Ret;
}
#endif

//------------------------------------------------------------------------------
/**
\brief  Create TX frame

The function creates the buffer for a TX frame and registers it to the ethernet
driver.

\param  pHandle_p           Handle to the last allocated frame buffer used for
                            faster search for PReq buffers. The function stores
                            the handle to the new frame buffer at this location.
\param  pFrameSize_p        Pointer to the size of the frame. The function stores
                            the size of the new frame at this locaten. It is
                            always equal or larget than the requested size. If
                            that is not possible an error will be generated.
\param  msgType_p           The message type of the frame.
\param  serviceId_p         The service ID in case of an ASnd frame. Otherwise
                            kDllAsndNotDefined.

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
tEplKernel dllk_createTxFrame (UINT* pHandle_p, UINT* pFrameSize_p,
                               tEplMsgType msgType_p, tDllAsndServiceId serviceId_p)
{
    tEplKernel      ret = kEplSuccessful;
    tEplFrame*      pTxFrame;
    UINT            handle = *pHandle_p;
    tEdrvTxBuffer*  pTxBuffer = NULL;
    UINT            nIndex = 0;

    switch (msgType_p)
    {
        case kEplMsgTypeAsnd:
            // search for fixed Tx buffers
            switch (serviceId_p)
            {
                case kDllAsndIdentResponse:
                    handle = DLLK_TXFRAME_IDENTRES;
                    break;

                case kDllAsndStatusResponse:
                    handle = DLLK_TXFRAME_STATUSRES;
                    break;

                case kDllAsndNmtRequest:
                case kDllAsndNmtCommand:
                    handle = DLLK_TXFRAME_NMTREQ;
                    break;

#if EPL_DLL_PRES_CHAINING_CN != FALSE
                case kDllAsndSyncResponse:
                    handle = DLLK_TXFRAME_SYNCRES;
                    break;
#endif

                case kDllAsndNotDefined:
                    ret = kEplDllInvalidParam;
                    goto Exit;
                    break;

                case kDllAsndSdo:
#if (EPL_DLL_PRES_CHAINING_CN == FALSE) && (EPL_DLL_PRES_CHAINING_MN != FALSE)
                case kDllAsndSyncResponse:
#endif
                    ret = kEplEdrvBufNotExisting;
                    goto Exit;
                    break;
            }
            break;

        case kEplMsgTypeNonEpl:
            handle = DLLK_TXFRAME_NONEPL;
            break;

        case kEplMsgTypePres:
            handle = DLLK_TXFRAME_PRES;
            break;

#if defined(CONFIG_INCLUDE_NMT_MN)
        case kEplMsgTypeSoc:
            handle = DLLK_TXFRAME_SOC;
            break;

        case kEplMsgTypeSoa:
            handle = DLLK_TXFRAME_SOA;
            break;

        case kEplMsgTypePreq:
          // look for free entry
            if ((handle < DLLK_TXFRAME_PREQ) || (handle >= dllkInstance_g.maxTxFrames))
            {   // start with first PReq buffer
                handle = DLLK_TXFRAME_PREQ;
            }

            // otherwise start with last allocated handle
            pTxBuffer = &dllkInstance_g.pTxBuffer[handle];
            for (; handle < dllkInstance_g.maxTxFrames; handle++, pTxBuffer++)
            {
                if (pTxBuffer->m_pbBuffer == NULL)
                {   // free entry found
                    break;
                }
            }

            if (pTxBuffer->m_pbBuffer != NULL)
            {
                ret = kEplEdrvNoFreeBufEntry;
                goto Exit;
            }
            break;

#else
        default:
            ret = kEplEdrvBufNotExisting;
            goto Exit;
            break;
#endif
    }

    *pHandle_p = handle;

    for ( ; nIndex < 2; nIndex++, handle++)
    {
        // test if requested entry is free
        pTxBuffer = &dllkInstance_g.pTxBuffer[handle];
        if (pTxBuffer->m_pbBuffer != NULL)
        {   // entry is not free
            ret = kEplEdrvNoFreeBufEntry;
            goto Exit;
        }

        // setup Tx buffer
        pTxBuffer->m_uiMaxBufferLen = *pFrameSize_p;

        ret = EdrvAllocTxMsgBuffer(pTxBuffer);
        if (ret != kEplSuccessful)
        {   // error occurred while registering Tx frame
            goto Exit;
        }

        // because buffer size may be larger than requested/ memorize real length of frame
        pTxBuffer->m_uiTxMsgLen = *pFrameSize_p;
        // initialize time offset
        pTxBuffer->m_dwTimeOffsetNs = 0;
        // fill whole frame with 0
        EPL_MEMSET(pTxBuffer->m_pbBuffer, 0, pTxBuffer->m_uiMaxBufferLen);
        pTxFrame = (tEplFrame *) pTxBuffer->m_pbBuffer;

        if (msgType_p != kEplMsgTypeNonEpl)
        {   // fill out Frame only if it is an EPL frame
            AmiSetWordToBe(&pTxFrame->m_be_wEtherType, EPL_C_DLL_ETHERTYPE_EPL);
            AmiSetByteToLe(&pTxFrame->m_le_bSrcNodeId, (BYTE) dllkInstance_g.dllConfigParam.nodeId);
            EPL_MEMCPY(&pTxFrame->m_be_abSrcMac[0], &dllkInstance_g.aLocalMac[0], 6);
            switch (msgType_p)
            {
                case kEplMsgTypeAsnd:
                    // destination MAC address
                    AmiSetQword48ToBe(&pTxFrame->m_be_abDstMac[0], EPL_C_DLL_MULTICAST_ASND);
                    // destination node ID
                    switch (serviceId_p)
                    {
                        case kDllAsndIdentResponse:
                            AmiSetByteToLe(&pTxFrame->m_Data.m_Asnd.m_Payload.m_IdentResponse.m_le_bEplProfileVersion,
                                           (UINT8) EPL_SPEC_VERSION);
                            AmiSetDwordToLe(&pTxFrame->m_Data.m_Asnd.m_Payload.m_IdentResponse.m_le_dwFeatureFlags,
                                            dllkInstance_g.dllConfigParam.featureFlags);
                            AmiSetWordToLe(&pTxFrame->m_Data.m_Asnd.m_Payload.m_IdentResponse.m_le_wMtu,
                                           (UINT16) dllkInstance_g.dllConfigParam.asyncMtu);
                            AmiSetWordToLe(&pTxFrame->m_Data.m_Asnd.m_Payload.m_IdentResponse.m_le_wPollInSize,
                                           (UINT16)dllkInstance_g.dllConfigParam.preqActPayloadLimit);
                            AmiSetWordToLe(&pTxFrame->m_Data.m_Asnd.m_Payload.m_IdentResponse.m_le_wPollOutSize,
                                           (UINT16)dllkInstance_g.dllConfigParam.presActPayloadLimit);
                            AmiSetDwordToLe(&pTxFrame->m_Data.m_Asnd.m_Payload.m_IdentResponse.m_le_dwResponseTime,
                                            dllkInstance_g.dllConfigParam.presMaxLatency);
                            AmiSetDwordToLe(&pTxFrame->m_Data.m_Asnd.m_Payload.m_IdentResponse.m_le_dwDeviceType,
                                            dllkInstance_g.dllIdentParam.deviceType);
                            AmiSetDwordToLe(&pTxFrame->m_Data.m_Asnd.m_Payload.m_IdentResponse.m_le_dwVendorId,
                                            dllkInstance_g.dllIdentParam.vendorId);
                            AmiSetDwordToLe(&pTxFrame->m_Data.m_Asnd.m_Payload.m_IdentResponse.m_le_dwProductCode,
                                            dllkInstance_g.dllIdentParam.productCode);
                            AmiSetDwordToLe(&pTxFrame->m_Data.m_Asnd.m_Payload.m_IdentResponse.m_le_dwRevisionNumber,
                                            dllkInstance_g.dllIdentParam.revisionNumber);
                            AmiSetDwordToLe(&pTxFrame->m_Data.m_Asnd.m_Payload.m_IdentResponse.m_le_dwSerialNumber,
                                            dllkInstance_g.dllIdentParam.serialNumber);
                            AmiSetQword64ToLe(&pTxFrame->m_Data.m_Asnd.m_Payload.m_IdentResponse.m_le_qwVendorSpecificExt1,
                                              dllkInstance_g.dllIdentParam.vendorSpecificExt1);
                            AmiSetDwordToLe(&pTxFrame->m_Data.m_Asnd.m_Payload.m_IdentResponse.m_le_dwVerifyConfigurationDate,
                                            dllkInstance_g.dllIdentParam.verifyConfigurationDate);
                            AmiSetDwordToLe(&pTxFrame->m_Data.m_Asnd.m_Payload.m_IdentResponse.m_le_dwVerifyConfigurationTime,
                                            dllkInstance_g.dllIdentParam.verifyConfigurationTime);
                            AmiSetDwordToLe(&pTxFrame->m_Data.m_Asnd.m_Payload.m_IdentResponse.m_le_dwApplicationSwDate,
                                            dllkInstance_g.dllIdentParam.applicationSwDate);
                            AmiSetDwordToLe(&pTxFrame->m_Data.m_Asnd.m_Payload.m_IdentResponse.m_le_dwApplicationSwTime,
                                            dllkInstance_g.dllIdentParam.applicationSwTime);
                            AmiSetDwordToLe(&pTxFrame->m_Data.m_Asnd.m_Payload.m_IdentResponse.m_le_dwIpAddress,
                                            dllkInstance_g.dllIdentParam.ipAddress);
                            AmiSetDwordToLe(&pTxFrame->m_Data.m_Asnd.m_Payload.m_IdentResponse.m_le_dwSubnetMask,
                                            dllkInstance_g.dllIdentParam.subnetMask);
                            AmiSetDwordToLe(&pTxFrame->m_Data.m_Asnd.m_Payload.m_IdentResponse.m_le_dwDefaultGateway,
                                            dllkInstance_g.dllIdentParam.defaultGateway);
                            EPL_MEMCPY(&pTxFrame->m_Data.m_Asnd.m_Payload.m_IdentResponse.m_le_sHostname[0],
                                       &dllkInstance_g.dllIdentParam.sHostname[0],
                                       sizeof (dllkInstance_g.dllIdentParam.sHostname));
                            EPL_MEMCPY(&pTxFrame->m_Data.m_Asnd.m_Payload.m_IdentResponse.m_le_abVendorSpecificExt2[0],
                                       &dllkInstance_g.dllIdentParam.aVendorSpecificExt2[0],
                                       sizeof (dllkInstance_g.dllIdentParam.aVendorSpecificExt2));
                            // fall-through

                        case kDllAsndStatusResponse:
                            // IdentResponses and StatusResponses are Broadcast
                            AmiSetByteToLe(&pTxFrame->m_le_bDstNodeId, (BYTE) EPL_C_ADR_BROADCAST);
                            break;

#if EPL_DLL_PRES_CHAINING_CN != FALSE
                        case kDllAsndSyncResponse:
                            // SyncRes destination node ID is MN node ID
                            AmiSetByteToLe(&pTxFrame->m_le_bDstNodeId, (BYTE) EPL_C_ADR_MN_DEF_NODE_ID);
                            AmiSetDwordToLe(&pTxFrame->m_Data.m_Asnd.m_Payload.m_SyncResponse.m_le_dwLatency,
                                            dllkInstance_g.dllConfigParam.syncResLatency);
                            // SyncStatus: PResMode disabled / PResTimeFirst and PResTimeSecond invalid
                            // AmiSetDwordToLe(&pTxFrame->m_Data.m_Asnd.m_Payload.m_SyncResponse.m_le_dwSyncStatus, 0);
                            // init SyncNodeNumber
                            // AmiSetDwordToLe(&pTxFrame->m_Data.m_Asnd.m_Payload.m_SyncResponse.m_le_dwSyncNodeNumber, 0);
                            // init SyncDelay
                            // AmiSetDwordToLe(&pTxFrame->m_Data.m_Asnd.m_Payload.m_SyncResponse.m_le_dwSyncDelay, 0);
                            break;
#endif

                        default:
                            break;
                    }

                    // ASnd Service ID
                    AmiSetByteToLe(&pTxFrame->m_Data.m_Asnd.m_le_bServiceId, serviceId_p);
                    break;

                case kEplMsgTypePres:
                    AmiSetQword48ToBe(&pTxFrame->m_be_abDstMac[0], EPL_C_DLL_MULTICAST_PRES);
                    AmiSetByteToLe(&pTxFrame->m_le_bDstNodeId, (BYTE) EPL_C_ADR_BROADCAST);
                    break;

#if defined(CONFIG_INCLUDE_NMT_MN)
                case kEplMsgTypeSoc:
                    AmiSetQword48ToBe(&pTxFrame->m_be_abDstMac[0], EPL_C_DLL_MULTICAST_SOC);
                    AmiSetByteToLe(&pTxFrame->m_le_bDstNodeId, (BYTE) EPL_C_ADR_BROADCAST);
                    break;

                case kEplMsgTypeSoa:
                    AmiSetQword48ToBe(&pTxFrame->m_be_abDstMac[0], EPL_C_DLL_MULTICAST_SOA);
                    AmiSetByteToLe(&pTxFrame->m_le_bDstNodeId, (BYTE) EPL_C_ADR_BROADCAST);
                    AmiSetByteToLe(&pTxFrame->m_Data.m_Soa.m_le_bEplVersion, (BYTE) EPL_SPEC_VERSION);
                    break;

                case kEplMsgTypePreq:
                    break;
#endif

                default:
                    break;
            }

            // EPL message type
            AmiSetByteToLe(&pTxFrame->m_le_bMessageType, (BYTE) msgType_p);
        }
    }

    *pFrameSize_p = pTxBuffer->m_uiMaxBufferLen;

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Delete TX frame

The function deletes the buffer for a TX frame and frees it in the ethernet
driver.

\param  handle_p            Handle to the frame buffer.

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
tEplKernel dllk_deleteTxFrame (UINT handle_p)
{
    tEplKernel      ret = kEplSuccessful;
    tEdrvTxBuffer*  pTxBuffer = NULL;
    UINT            nIndex = 0;

    if (handle_p >= dllkInstance_g.maxTxFrames)
    {   // handle is not valid
        return kEplDllIllegalHdl;
    }

    for ( ; nIndex < 2; nIndex++, handle_p++)
    {
        pTxBuffer = &dllkInstance_g.pTxBuffer[handle_p];

        // mark buffer as free so that frame will not be send in future anymore
        // $$$ d.k. What's up with running transmissions?
        pTxBuffer->m_uiTxMsgLen = DLLK_BUFLEN_EMPTY;

        ret = EdrvReleaseTxMsgBuffer(pTxBuffer);
        if (ret != kEplSuccessful)
        {   // error occurred while releasing Tx frame
            return ret;
        }

        pTxBuffer->m_pbBuffer = NULL;
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Process TPDO frame

The function forwards the specified TPDO for processing to the registered
callback function (i.e. to the PDO module).

\param  pFrameInfo_p        Pointer to frame information.
\param  fReadyFlag_p        Ready flag.

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
tEplKernel dllk_processTpdo(tFrameInfo * pFrameInfo_p, BOOL fReadyFlag_p)
{
    tEplKernel      ret = kEplSuccessful;

    if (dllkInstance_g.pfnCbProcessTpdo != NULL)
    {
        ret = dllkInstance_g.pfnCbProcessTpdo(pFrameInfo_p, fReadyFlag_p);
    }
    return ret;
}

//----------------------------------------------------------------------------//
//                L O C A L   F U N C T I O N S                               //
//----------------------------------------------------------------------------//

//------------------------------------------------------------------------------
/**
\brief  Process received PReq frame.

The function processes a received PReq frame.

\param  pFrameInfo_p        Pointer to frame information.
\param  nmtState_p          NMT state of the local node.
\param  pReleaseRxBuffer_p  Pointer to buffer release flag. The function must
                            set this flag to determine if the RxBuffer could be
                            released immediately.

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
static tEplKernel processReceivedPreq(tFrameInfo* pFrameInfo_p, tNmtState nmtState_p,
                                      tEdrvReleaseRxBuffer* pReleaseRxBuffer_p)
{
    tEplKernel      ret = kEplSuccessful;
    tEplFrame*      pFrame;
    BYTE            bFlag1;

    pFrame = pFrameInfo_p->pFrame;

    if (nmtState_p >= kNmtMsNotActive)
    {   // MN is active -> wrong msg type
        goto Exit;
    }

#if EDRV_EARLY_RX_INT == FALSE
    if (nmtState_p >= kNmtCsPreOperational2)
    {   // respond to and process PReq frames only in PreOp2, ReadyToOp and Op

#if (EDRV_AUTO_RESPONSE == FALSE)
        tEdrvTxBuffer*  pTxBuffer = NULL;

        // Auto-response is disabled
        // Does PRes exist?
        pTxBuffer = &dllkInstance_g.pTxBuffer[DLLK_TXFRAME_PRES + dllkInstance_g.curTxBufferOffsetCycle];
        if (pTxBuffer->m_pbBuffer != NULL)
        {   // PRes does exist -> send PRes frame
#if (EPL_DLL_PRES_READY_AFTER_SOA != FALSE) || (EPL_DLL_PRES_READY_AFTER_SOC != FALSE)
            EdrvTxMsgStart(pTxBuffer);
#else
            if ((ret = EdrvSendTxMsg(pTxBuffer)) != kEplSuccessful)
                goto Exit;
#endif
        }
#endif
#endif

        // update only EA and MS flag
        bFlag1 = AmiGetByteFromLe(&pFrame->m_Data.m_Preq.m_le_bFlag1);

        dllkInstance_g.mnFlag1 = (dllkInstance_g.mnFlag1 &
                                  ~(EPL_FRAME_FLAG1_EA | EPL_FRAME_FLAG1_MS)) |             // preserve all flags except EA and MS
                                  (bFlag1 & (EPL_FRAME_FLAG1_EA | EPL_FRAME_FLAG1_MS));     // set EA and MS flag

        // inform PDO module
#if defined(CONFIG_INCLUDE_PDOK)
        if (nmtState_p >= kNmtCsReadyToOperate)
        {   // inform PDO module only in ReadyToOp and Op
            if (nmtState_p != kNmtCsOperational)
            {
                // reset RD flag and all other flags, but that does not matter, because they were processed above
                AmiSetByteToLe(&pFrame->m_Data.m_Preq.m_le_bFlag1, 0);
            }

            // compares real frame size and PDO size
            if (((UINT) (AmiGetWordFromLe(&pFrame->m_Data.m_Preq.m_le_wSize) + EPL_FRAME_OFFSET_PDO_PAYLOAD) > pFrameInfo_p->frameSize) ||
                         (pFrameInfo_p->frameSize > (dllkInstance_g.dllConfigParam.preqActPayloadLimit + EPL_FRAME_OFFSET_PDO_PAYLOAD)))
            {   // format error
                tErrHndkEvent  dllEvent;

                dllEvent.m_ulDllErrorEvents = EPL_DLL_ERR_INVALID_FORMAT;
                dllEvent.m_uiNodeId = AmiGetByteFromLe(&pFrame->m_le_bSrcNodeId);
                dllEvent.m_NmtState = nmtState_p;
                errhndk_postError(&dllEvent);
                goto Exit;
            }

            // forward PReq frame as RPDO to PDO module
            ret = forwardRpdo(pFrameInfo_p);
            if (ret == kEplReject)
            {
                *pReleaseRxBuffer_p = kEdrvReleaseRxBufferLater;
                ret = kEplSuccessful;
            }
            else if (ret != kEplSuccessful)
            {
                goto Exit;
            }
        }
#endif

#if EDRV_EARLY_RX_INT == FALSE
        // $$$ inform emergency protocol handling (error signaling module) about flags
    }
#endif

    // reset cycle counter
    dllkInstance_g.cycleCount = 0;

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Process received PRes frame

The function processes a received PRes frame.

\param  pFrameInfo_p        Pointer to frame information.
\param  nmtState_p          NMT state of the local node.
\param  pNmtEvent_p         Pointer to store NMT event.
\param  pReleaseRxBuffer_p  Pointer to buffer release flag. The function must
                            set this flag to determine if the RxBuffer could be
                            released immediately.

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
static tEplKernel processReceivedPres(tFrameInfo* pFrameInfo_p, tNmtState nmtState_p,
                                      tNmtEvent* pNmtEvent_p, tEdrvReleaseRxBuffer* pReleaseRxBuffer_p)
{
    tEplKernel      ret = kEplSuccessful;
    tEplFrame*      pFrame;
    UINT            nodeId;

#if EPL_NMT_MAX_NODE_ID > 0
    tDllkNodeInfo*  pIntNodeInfo = NULL;
#endif

    pFrame = pFrameInfo_p->pFrame;
    nodeId = AmiGetByteFromLe(&pFrame->m_le_bSrcNodeId);

#if EPL_DLL_PRES_CHAINING_CN != FALSE
    if ((dllkInstance_g.fPrcEnabled != FALSE) && (nodeId == EPL_C_ADR_MN_DEF_NODE_ID))
    {   // handle PResMN as PReq for PRes Chaining
        *pNmtEvent_p = kNmtEventDllCePreq;
    }
    else
#endif
    {
        *pNmtEvent_p = kNmtEventDllCePres;
    }

    if ((nmtState_p >= kNmtCsPreOperational2) && (nmtState_p <= kNmtCsOperational))
    {   // process PRes frames only in PreOp2, ReadyToOp and Op of CN
#if EPL_NMT_MAX_NODE_ID > 0
        pIntNodeInfo = dllk_getNodeInfo(nodeId);
        if (pIntNodeInfo == NULL)
        {   // no node info structure available
            ret = kEplDllNoNodeInfo;
            goto Exit;
        }
#endif
    }

#if defined(CONFIG_INCLUDE_NMT_MN)
    else
        if (dllkInstance_g.dllState > kDllMsNonCyclic)
        {   // or process PRes frames in MsWaitPres
        tHeartbeatEvent     heartbeatEvent;
        BYTE                flag1;
        BYTE                nextNodeIndex = dllkInstance_g.curNodeIndex;
        BYTE*               pCnNodeId = &dllkInstance_g.aCnNodeIdList[dllkInstance_g.curTxBufferOffsetCycle][nextNodeIndex];

#if EPL_DLL_PRES_CHAINING_MN != FALSE
        BOOL    fPrcSlotFinished = FALSE;
#endif

        while (*pCnNodeId != EPL_C_ADR_INVALID)
        {
            if (*pCnNodeId == nodeId)
            {   // CN found in list
                nextNodeIndex = nextNodeIndex - dllkInstance_g.curNodeIndex;
                dllkInstance_g.curNodeIndex += nextNodeIndex + 1;

                for (pCnNodeId-- ; nextNodeIndex > 0; nextNodeIndex--, pCnNodeId--)
                {   // issue error for each CN in list between last and current
                    if ((ret = dllk_issueLossOfPres(*pCnNodeId)) != kEplSuccessful)
                        goto Exit;
                }

                pIntNodeInfo = dllk_getNodeInfo(nodeId);

                break;
            }
#if EPL_DLL_PRES_CHAINING_MN != FALSE
            else if (*pCnNodeId == EPL_C_ADR_BROADCAST)
            {   // PRC slot finished
                fPrcSlotFinished = TRUE;
            }
#endif
            pCnNodeId++;
            nextNodeIndex++;
        }
        if (pIntNodeInfo == NULL)
        {   // ignore PRes, because it is from wrong CN
            *pNmtEvent_p = kNmtEventNoEvent;
            goto Exit;
        }

#if EPL_DLL_PRES_CHAINING_MN != FALSE
        if (fPrcSlotFinished != FALSE)
        {
            dllkInstance_g.fPrcSlotFinished = TRUE;

            if ((dllkInstance_g.dllConfigParam.syncNodeId > EPL_C_ADR_SYNC_ON_SOC)
                && (dllkInstance_g.fSyncProcessed == FALSE)
                && (dllkInstance_g.dllConfigParam.fSyncOnPrcNode != FALSE))
            {
                dllkInstance_g.fSyncProcessed = TRUE;
                if ((ret = dllk_postEvent(kEplEventTypeSync)) != kEplSuccessful)
                    goto Exit;
            }
        }
        else
#endif

        if ((dllkInstance_g.dllConfigParam.syncNodeId > EPL_C_ADR_SYNC_ON_SOC)
            && (dllkInstance_g.fSyncProcessed == FALSE)
#if EPL_DLL_PRES_CHAINING_MN != FALSE
            && (dllkInstance_g.dllConfigParam.fSyncOnPrcNode != dllkInstance_g.fPrcSlotFinished)
#endif
            && (nodeId > dllkInstance_g.dllConfigParam.syncNodeId))
        {
            dllkInstance_g.fSyncProcessed = TRUE;
            if ((ret = dllk_postEvent(kEplEventTypeSync)) != kEplSuccessful)
                goto Exit;
        }

        // forward Flag2 to asynchronous scheduler
        flag1 = AmiGetByteFromLe(&pFrame->m_Data.m_Asnd.m_Payload.m_StatusResponse.m_le_bFlag2);
        ret = dllkcal_setAsyncPendingRequests(nodeId,
            ((tDllAsyncReqPriority) ((flag1 & EPL_FRAME_FLAG2_PR) >> EPL_FRAME_FLAG2_PR_SHIFT)),
            (flag1 & EPL_FRAME_FLAG2_RS));
        if (ret != kEplSuccessful)
            goto Exit;

        // check NMT state of CN
        heartbeatEvent.errorCode = EPL_E_NO_ERROR;
        heartbeatEvent.nmtState = (tNmtState) (AmiGetByteFromLe(&pFrame->m_Data.m_Pres.m_le_bNmtStatus) | NMT_TYPE_CS);

        if (pIntNodeInfo->nmtState != heartbeatEvent.nmtState)
        {   // NMT state of CN has changed -> post event to NmtMnu module
            tEplEvent   event;

            if (pIntNodeInfo->fSoftDelete == FALSE)
            {   // normal isochronous CN
                heartbeatEvent.nodeId = nodeId;
                event.m_EventSink = kEplEventSinkNmtMnu;
                event.m_EventType = kEplEventTypeHeartbeat;
                event.m_uiSize = sizeof (heartbeatEvent);
                event.m_pArg = &heartbeatEvent;
            }
            else
            {   // CN shall be deleted softly, so remove it now, without issuing any error
                tDllNodeOpParam     nodeOpParam;

                nodeOpParam.opNodeType = kDllNodeOpTypeIsochronous;
                nodeOpParam.nodeId = pIntNodeInfo->nodeId;

                event.m_EventSink = kEplEventSinkDllkCal;
                event.m_EventType = kEplEventTypeDllkDelNode;
                // $$$ d.k. set Event.m_NetTime to current time
                event.m_uiSize = sizeof (nodeOpParam);
                event.m_pArg = &nodeOpParam;
            }

            if ((ret = eventk_postEvent(&event)) != kEplSuccessful)
                goto Exit;

            // save current NMT state of CN in internal node structure
            pIntNodeInfo->nmtState = heartbeatEvent.nmtState;
        }
    }
#endif
    else
    {   // ignore PRes, because it was received in wrong NMT state
        // but execute changeState() and post event to NMT module
        goto Exit;
    }

    // inform PDO module
#if defined(CONFIG_INCLUDE_PDOK)
    if (( nmtState_p != kNmtCsPreOperational2) && (nmtState_p != kNmtMsPreOperational2))
    {   // inform PDO module only in ReadyToOp and Op
        // compare real frame size and PDO size?
        WORD wPresPayloadSize = AmiGetWordFromLe(&pFrame->m_Data.m_Pres.m_le_wSize);

        if (((UINT) (wPresPayloadSize + EPL_FRAME_OFFSET_PDO_PAYLOAD) > pFrameInfo_p->frameSize)
#if EPL_NMT_MAX_NODE_ID > 0
            || (wPresPayloadSize > pIntNodeInfo->presPayloadLimit)
            || ((nmtState_p >= kNmtMsNotActive)
                && (pFrameInfo_p->frameSize >
                    (UINT) (pIntNodeInfo->presPayloadLimit + EPL_FRAME_OFFSET_PDO_PAYLOAD)))
#endif
            )
        {   // format error
        tErrHndkEvent  DllEvent;

#if EPL_NMT_MAX_NODE_ID > 0
            if (pIntNodeInfo->presPayloadLimit > 0)
#endif
            {   // This PRes frame was expected, but it is too large
                // otherwise it will be silently ignored
                DllEvent.m_ulDllErrorEvents = EPL_DLL_ERR_INVALID_FORMAT;
                DllEvent.m_uiNodeId = nodeId;
                DllEvent.m_NmtState = nmtState_p;
                ret = errhndk_postError(&DllEvent);
                if (ret != kEplSuccessful)
                    goto Exit;
            }
            goto Exit;
        }
        if ((nmtState_p != kNmtCsOperational)
            && (nmtState_p != kNmtMsOperational))
        {
            // reset RD flag and all other flags, but that does not matter, because they were processed above
            AmiSetByteToLe(&pFrame->m_Data.m_Pres.m_le_bFlag1, 0);
        }
        ret = forwardRpdo(pFrameInfo_p);
        if (ret == kEplReject)
        {
            *pReleaseRxBuffer_p = kEdrvReleaseRxBufferLater;
            ret = kEplSuccessful;
        }
        else if (ret != kEplSuccessful)
        {
            goto Exit;
        }
    }
#endif

#if defined(CONFIG_INCLUDE_NMT_MN)
    if ((dllkInstance_g.dllState > kDllMsNonCyclic) &&
        (dllkInstance_g.dllConfigParam.syncNodeId > EPL_C_ADR_SYNC_ON_SOC) &&
        (dllkInstance_g.fSyncProcessed == FALSE))
    {   // check if Sync event needs to be triggered
        if (
#if EPL_DLL_PRES_CHAINING_MN != FALSE
            (dllkInstance_g.dllConfigParam.fSyncOnPrcNode != dllkInstance_g.fPrcSlotFinished) &&
#endif
            (nodeId == dllkInstance_g.dllConfigParam.syncNodeId))
        {
            dllkInstance_g.fSyncProcessed = TRUE;
            ret = dllk_postEvent(kEplEventTypeSync);
        }
    }
#endif

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Process received SoC frame

The function processes a received SoC frame.

\param  pRxBuffer_p         Pointer to RxBuffer structure of received frame.
\param  nmtState_p          NMT state of the local node.

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
static tEplKernel processReceivedSoc(tEdrvRxBuffer* pRxBuffer_p, tNmtState nmtState_p)
{
    tEplKernel      ret = kEplSuccessful;
#if EPL_DLL_PRES_READY_AFTER_SOC != FALSE
    tEdrvTxBuffer*  pTxBuffer = NULL;
#endif

#if (EPL_DLL_PROCESS_SYNC != EPL_DLL_PROCESS_SYNC_ON_TIMER)
    UNUSED_PARAMETER(pRxBuffer_p);
#endif

    if (nmtState_p >= kNmtMsNotActive)
    {   // MN is active -> wrong msg type
        return ret;
    }

#if EPL_DLL_PRES_READY_AFTER_SOC != FALSE
    // post PRes to transmit FIFO of the ethernet controller, but don't start
    // transmission over bus
    pTxBuffer = &dllkInstance_g.pTxBuffer[DLLK_TXFRAME_PRES + dllkInstance_g.curTxBufferOffsetCycle];
    if (pTxBuffer->m_pbBuffer != NULL)          // Does PRes exist?
    {   // PRes does exist -> mark PRes frame as ready for transmission
        ret = EdrvTxMsgReady(pTxBuffer);
        if (ret != kEplSuccessful)
            return ret;
    }
#endif

    if (nmtState_p >= kNmtCsStopped)
    {   // SoC frames only in Stopped, PreOp2, ReadyToOp and Operational

#if (EPL_DLL_PROCESS_SYNC == EPL_DLL_PROCESS_SYNC_ON_SOC)
        // trigger synchronous task
        if ((ret = dllk_postEvent(kEplEventTypeSync)) != kEplSuccessful)
            return ret;
#elif (EPL_DLL_PROCESS_SYNC == EPL_DLL_PROCESS_SYNC_ON_TIMER)
        ret = EplTimerSynckTriggerAtTimeStamp(pRxBuffer_p->m_pTgtTimeStamp);
        if (ret != kEplSuccessful)
            return ret;
#endif

        // update cycle counter
        if (dllkInstance_g.dllConfigParam.multipleCycleCnt > 0)
        {   // multiplexed cycle active
            dllkInstance_g.cycleCount = (dllkInstance_g.cycleCount + 1) %
                            dllkInstance_g.dllConfigParam.multipleCycleCnt;
        }
    }

    // reprogram timer
#if EPL_TIMER_USE_HIGHRES != FALSE
    if (dllkInstance_g.frameTimeout != 0)
    {
        EplTimerHighReskModifyTimerNs(&dllkInstance_g.timerHdlCycle, dllkInstance_g.frameTimeout,
                                      dllk_cbCnTimer, 0L, FALSE);
    }
#endif
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Process received SoA frame

The function processes a received SoA frame.

\param  pRxBuffer_p         Pointer to RxBuffer structure of received frame.
\param  nmtState_p          NMT state of the local node.

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
static tEplKernel processReceivedSoa(tEdrvRxBuffer* pRxBuffer_p, tNmtState nmtState_p)
{
    tEplKernel          ret = kEplSuccessful;
    tEplFrame*          pFrame;
    tEdrvTxBuffer*      pTxBuffer = NULL;
    tDllReqServiceId reqServiceId;
    UINT                nodeId;
    UINT8               flag1;

    pFrame = (tEplFrame *)pRxBuffer_p->m_pbBuffer;

    if (nmtState_p >= kNmtMsNotActive)
    {   // MN is active -> wrong msg type
        goto Exit;
    }

    pTxBuffer = NULL;

    if ((nmtState_p & NMT_SUPERSTATE_MASK) != NMT_CS_PLKMODE)
    {   // do not respond, if NMT state is < PreOp1 (i.e. not EPL_MODE)
        goto Exit;
    }

    // check TargetNodeId
    nodeId = AmiGetByteFromLe(&pFrame->m_Data.m_Soa.m_le_bReqServiceTarget);
    if (nodeId == dllkInstance_g.dllConfigParam.nodeId)
    {   // local node is the target of the current request

        // check ServiceId
        reqServiceId = (tDllReqServiceId) AmiGetByteFromLe(&pFrame->m_Data.m_Soa.m_le_bReqServiceId);
        switch (reqServiceId)
        {
            case kDllReqServiceStatus:
                // StatusRequest
#if (EDRV_AUTO_RESPONSE == FALSE)
                // Auto-response is not available
                pTxBuffer = &dllkInstance_g.pTxBuffer[DLLK_TXFRAME_STATUSRES + dllkInstance_g.curTxBufferOffsetStatusRes];
                if (pTxBuffer->m_pbBuffer != NULL)
                {   // StatusRes does exist
                    // send StatusRes
                    ret = EdrvSendTxMsg(pTxBuffer);
                    if (ret != kEplSuccessful)
                        goto Exit;

                    TGT_DBG_SIGNAL_TRACE_POINT(8);
                }
                else
                {   // no frame transmitted
                    pTxBuffer = NULL;
                }
#endif

                // update error signaling
                flag1 = AmiGetByteFromLe(&pFrame->m_Data.m_Soa.m_le_bFlag1);
                if (((flag1 ^ dllkInstance_g.mnFlag1) & EPL_FRAME_FLAG1_ER) != 0)
                {   // exception reset flag was changed by MN
                    // assume same state for EC in next cycle (clear all other bits)
                    if ((flag1 & EPL_FRAME_FLAG1_ER) != 0)
                    {
                        // set EC and reset rest
                        dllkInstance_g.flag1 = EPL_FRAME_FLAG1_EC;
                    }
                    else
                    {
                        // reset entire flag 1 (including EC and EN)
                        dllkInstance_g.flag1 = 0;
                    }

                    // signal update of StatusRes
                    ret = dllk_postEvent(kEplEventTypeDllkFlag1);
                    if (ret != kEplSuccessful)
                        goto Exit;

                }
                // update (only) EA and ER flag from MN for Status request response cycle
                // $$$ d.k. only in PreOp1 and when async-only or not accessed isochronously
                dllkInstance_g.mnFlag1 =
                        (dllkInstance_g.mnFlag1 & ~(EPL_FRAME_FLAG1_EA | EPL_FRAME_FLAG1_ER)) // preserve all flags except EA and ER
                        | (flag1 & (EPL_FRAME_FLAG1_EA | EPL_FRAME_FLAG1_ER));                     // set EA and ER flag
                goto Exit;
                break;

            case kDllReqServiceIdent:
               // IdentRequest
#if (EDRV_AUTO_RESPONSE == FALSE)
                // Auto-response is not available
                pTxBuffer = &dllkInstance_g.pTxBuffer[DLLK_TXFRAME_IDENTRES + dllkInstance_g.curTxBufferOffsetIdentRes];
                if (pTxBuffer->m_pbBuffer != NULL)
                {   // IdentRes does exist
                    // send IdentRes
                    ret = EdrvSendTxMsg(pTxBuffer);
                    if (ret != kEplSuccessful)
                        goto Exit;

                    TGT_DBG_SIGNAL_TRACE_POINT(7);
                }
                else
                {   // no frame transmitted
                    pTxBuffer = NULL;
                }
#endif
                goto Exit;
                break;

            case kDllReqServiceNmtRequest:
                // NmtRequest
#if (EDRV_AUTO_RESPONSE == FALSE)
                // Auto-response is not available
                pTxBuffer = &dllkInstance_g.pTxBuffer[DLLK_TXFRAME_NMTREQ + dllkInstance_g.curTxBufferOffsetNmtReq];
                if (pTxBuffer->m_pbBuffer != NULL)
                {   // NmtRequest does exist
                    // check if frame is not empty and not being filled
                    if (pTxBuffer->m_uiTxMsgLen > DLLK_BUFLEN_FILLING)
                    {
                        // send NmtRequest
                        ret = EdrvSendTxMsg(pTxBuffer);
                        if (ret != kEplSuccessful)
                            goto Exit;

                        // decrement RS in Flag 2
                        // The real update will be done later on event FillTx,
                        // but for now it assures that a quite good value gets via the SoA event into the next PRes.
                        if ((dllkInstance_g.flag2 & EPL_FRAME_FLAG2_RS) != 0)
                        {
                            dllkInstance_g.flag2--;
                        }
                    }
                    else
                    {   // no frame transmitted
                        pTxBuffer = NULL;
                    }
                }
                else
                {   // no frame transmitted
                    pTxBuffer = NULL;
                }
#endif
                goto Exit;
                break;

#if (EPL_DLL_PRES_CHAINING_CN != FALSE) || (EPL_DLL_PRES_CHAINING_MN != FALSE)
            case kDllReqServiceSync:
                {
                // SyncRequest
#if EPL_DLL_PRES_CHAINING_CN != FALSE
                UINT32      syncControl;
                tEplFrame*  pTxFrameSyncRes;
                tDllkPrcCycleTiming  PrcCycleTiming;

                pTxFrameSyncRes = (tEplFrame *) dllkInstance_g.pTxBuffer[DLLK_TXFRAME_SYNCRES].m_pbBuffer;
                syncControl = AmiGetDwordFromLe(&pFrame->m_Data.m_Soa.m_Payload.m_SyncRequest.m_le_dwSyncControl);
                if (syncControl & EPL_SYNC_DEST_MAC_ADDRESS_VALID)
                {
                    if (EPL_MEMCMP(&pFrame->m_Data.m_Soa.m_Payload.m_SyncRequest.m_be_abDestMacAddress,
                                   &dllkInstance_g.aLocalMac, 6) != 0)
                    {   // DestMacAddress valid but unequal to own MAC address -> SyncReq is ignored
                        goto Exit;
                    }
                }

                PrcCycleTiming.pResTimeFirstNs = AmiGetDwordFromLe(&pFrame->m_Data.m_Soa.m_Payload.m_SyncRequest.m_le_dwPResTimeFirst);

                if ((syncControl & EPL_SYNC_PRES_TIME_FIRST_VALID) &&
                    (dllkInstance_g.prcPResTimeFirst != PrcCycleTiming.pResTimeFirstNs))
                {
                    dllkInstance_g.prcPResTimeFirst = PrcCycleTiming.pResTimeFirstNs;

                    dllkInstance_g.pTxBuffer[DLLK_TXFRAME_PRES].m_dwTimeOffsetNs = PrcCycleTiming.pResTimeFirstNs;
                    ret = EdrvChangeFilter(dllkInstance_g.aFilter, DLLK_FILTER_COUNT,
                                           DLLK_FILTER_PREQ,
                                           EDRV_FILTER_CHANGE_AUTO_RESPONSE_DELAY);
                    if (ret != kEplSuccessful)
                        goto Exit;

                    AmiSetDwordToLe(&pTxFrameSyncRes->m_Data.m_Asnd.m_Payload.m_SyncResponse.m_le_dwPResTimeFirst,
                                    PrcCycleTiming.pResTimeFirstNs);
                    AmiSetDwordToLe(&pTxFrameSyncRes->m_Data.m_Asnd.m_Payload.m_SyncResponse.m_le_dwSyncStatus,
                                    AmiGetDwordFromLe(&pTxFrameSyncRes->m_Data.m_Asnd.m_Payload.m_SyncResponse.m_le_dwSyncStatus)
                                    | EPL_SYNC_PRES_TIME_FIRST_VALID);
                    // update SyncRes Tx buffer in Edrv
                    ret = EdrvUpdateTxMsgBuffer(&dllkInstance_g.pTxBuffer[DLLK_TXFRAME_SYNCRES]);
                    if (ret != kEplSuccessful)
                        goto Exit;
                }

                if (syncControl & EPL_SYNC_PRES_FALL_BACK_TIMEOUT_VALID)
                {
                    dllkInstance_g.prcPResFallBackTimeout = AmiGetDwordFromLe(&pFrame->m_Data.m_Soa.m_Payload.m_SyncRequest.m_le_dwPResFallBackTimeout);

#if (EPL_DLL_PROCESS_SYNC == EPL_DLL_PROCESS_SYNC_ON_TIMER)
                    if (dllkInstance_g.fPrcEnabled != FALSE)
                    {
                        ret = EplTimerSynckSetLossOfSyncTolerance2Ns(dllkInstance_g.prcPResFallBackTimeout);
                    }
#endif
                }

                if (syncControl & EPL_SYNC_PRES_MODE_RESET)
                {
                    // PResModeReset overrules PResModeSet
                    syncControl &= ~EPL_SYNC_PRES_MODE_SET;

                    ret = dllk_presChainingDisable();
                    if (ret != kEplSuccessful)
                        goto Exit;
                }
                else if (syncControl & EPL_SYNC_PRES_MODE_SET)
                {   // PRes Chaining is Enabled
                    ret = dllk_presChainingEnable();
                    if (ret != kEplSuccessful)
                        goto Exit;
                }

                PrcCycleTiming.syncControl = syncControl & (EPL_SYNC_PRES_TIME_FIRST_VALID
                                                              | EPL_SYNC_PRES_TIME_SECOND_VALID
                                                              | EPL_SYNC_SYNC_MN_DELAY_FIRST_VALID
                                                              | EPL_SYNC_SYNC_MN_DELAY_SECOND_VALID
                                                              | EPL_SYNC_PRES_MODE_RESET
                                                              | EPL_SYNC_PRES_MODE_SET);

                if (PrcCycleTiming.syncControl != 0)
                {
                    PrcCycleTiming.pResTimeSecondNs = AmiGetDwordFromLe(&pFrame->m_Data.m_Soa.m_Payload.m_SyncRequest.m_le_dwPResTimeSecond);
                    PrcCycleTiming.syncMNDelayFirstNs = AmiGetDwordFromLe(&pFrame->m_Data.m_Soa.m_Payload.m_SyncRequest.m_le_dwSyncMnDelayFirst);
                    PrcCycleTiming.syncMNDelaySecondNs = AmiGetDwordFromLe(&pFrame->m_Data.m_Soa.m_Payload.m_SyncRequest.m_le_dwSyncMnDelaySecond);
                    // $$$ m.u.: CbUpdatePrcCycleTiming
                }
#endif
                goto Exit;
                }
                break;
#endif

            case kDllReqServiceUnspecified:
                // unspecified invite
#if (EDRV_AUTO_RESPONSE == FALSE)
                // Auto-response is not available
                pTxBuffer = &dllkInstance_g.pTxBuffer[DLLK_TXFRAME_NONEPL + dllkInstance_g.curTxBufferOffsetNonEpl];
                if (pTxBuffer->m_pbBuffer != NULL)
                {   // non-EPL frame does exist
                    // check if frame is not empty and not being filled
                    if (pTxBuffer->m_uiTxMsgLen > DLLK_BUFLEN_FILLING)
                    {
                        // send non-EPL frame
                        ret = EdrvSendTxMsg(pTxBuffer);
                        if (ret != kEplSuccessful)
                            goto Exit;

                        // decrement RS in Flag 2
                        // The real update will be done later on event FillTx,
                        // but for now it assures that a quite good value gets via the SoA event into the next PRes.
                        if ((dllkInstance_g.flag2 & EPL_FRAME_FLAG2_RS) != 0)
                        {
                            dllkInstance_g.flag2--;
                        }
                    }
                    else
                    {   // no frame transmitted
                        pTxBuffer = NULL;
                    }
                }
                else
                {   // no frame transmitted
                    pTxBuffer = NULL;
                }
#endif
                break;

            case kDllReqServiceNo:
                // no async service requested -> do nothing
                goto Exit;
                break;
        }
    }

#if EPL_DLL_PRES_CHAINING_CN != FALSE
    else
    {   // other node is the target of the current request
        // check ServiceId
        reqServiceId = (tDllReqServiceId) AmiGetByteFromLe(&pFrame->m_Data.m_Soa.m_le_bReqServiceId);
        if (reqServiceId == kDllReqServiceSync)
        {   // SyncRequest -> store node ID and TimeStamp
            dllkInstance_g.syncReqPrevNodeId = nodeId;
            EplTgtTimeStampCopy(dllkInstance_g.pSyncReqPrevTimeStamp, pRxBuffer_p->m_pTgtTimeStamp);
        }
    }
#endif

#if EPL_DLL_PRES_READY_AFTER_SOA != FALSE
    if (pTxBuffer == NULL)
    {   // signal process function readiness of PRes frame
        ret = postEvent(kEplEventTypeDllkPresReady);
        if (ret != kEplSuccessful)
            goto Exit;
    }
#endif
    // $$$ put SrcNodeId, NMT state and NetTime as HeartbeatEvent into eventqueue
    // $$$ inform emergency protocol handling about flags

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Process received ASnd frame

The function processes a received ASnd frame.

\param  pFrameInfo_p        Pointer to frame information.
\param  pRxBuffer_p         Pointer to RxBuffer structure of received frame.
\param  nmtState_p          NMT state of the local node.

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
static tEplKernel processReceivedAsnd(tFrameInfo* pFrameInfo_p, tEdrvRxBuffer* pRxBuffer_p,
                                      tNmtState nmtState_p)
{
    tEplKernel      ret = kEplSuccessful;
    tEplFrame*      pFrame;
    UINT            asndServiceId;
    UINT            nodeId;

#if defined(CONFIG_INCLUDE_NMT_MN)
    UINT8           flag1;
#endif

    UNUSED_PARAMETER(nmtState_p);
#if EPL_DLL_PRES_CHAINING_CN == FALSE
    UNUSED_PARAMETER(pRxBuffer_p);
#endif

    pFrame = pFrameInfo_p->pFrame;

    // ASnd service registered?
    asndServiceId = (UINT)AmiGetByteFromLe(&pFrame->m_Data.m_Asnd.m_le_bServiceId);

#if defined(CONFIG_INCLUDE_NMT_MN)
    if (dllkInstance_g.dllState >= kDllMsNonCyclic)
    {
        switch ((tDllAsndServiceId) asndServiceId)
        {
            case kDllAsndStatusResponse:
            case kDllAsndIdentResponse:
#if (EPL_DLL_PRES_CHAINING_MN != FALSE)
            case kDllAsndSyncResponse:
#endif
                nodeId = AmiGetByteFromLe(&pFrame->m_le_bSrcNodeId);
                if ((dllkInstance_g.aLastReqServiceId[dllkInstance_g.curLastSoaReq] == ((tDllReqServiceId) asndServiceId)) &&
                    (nodeId == dllkInstance_g.aLastTargetNodeId[dllkInstance_g.curLastSoaReq]))
                {   // mark request as responded
                    dllkInstance_g.aLastReqServiceId[dllkInstance_g.curLastSoaReq] = kDllReqServiceNo;
                }

                if (((tDllAsndServiceId) asndServiceId) == kDllAsndIdentResponse)
                {   // memorize MAC address of CN for PReq
                    tDllkNodeInfo*   pIntNodeInfo;

                    pIntNodeInfo = dllk_getNodeInfo(nodeId);
                    if (pIntNodeInfo == NULL)
                    {   // no node info structure available
                        ret = kEplDllNoNodeInfo;
                        goto Exit;
                    }
                    else
                    {
                        EPL_MEMCPY(pIntNodeInfo->aMacAddr, pFrame->m_be_abSrcMac, 6);
                    }
                }
#if (EPL_DLL_PRES_CHAINING_MN != FALSE)
                else if (((tDllAsndServiceId) asndServiceId) == kDllAsndSyncResponse)
                {
                    break;
                }
#endif
                // forward Flag2 to asynchronous scheduler
                flag1 = AmiGetByteFromLe(&pFrame->m_Data.m_Asnd.m_Payload.m_StatusResponse.m_le_bFlag2);
                ret = dllkcal_setAsyncPendingRequests(nodeId,
                    ((tDllAsyncReqPriority) ((flag1 & EPL_FRAME_FLAG2_PR) >> EPL_FRAME_FLAG2_PR_SHIFT)),
                    (flag1 & EPL_FRAME_FLAG2_RS));
                if (ret != kEplSuccessful)
                    goto Exit;
                break;

            default:
                break;
        }
    }
#endif

    if (asndServiceId < DLL_MAX_ASND_SERVICE_ID)
    {   // ASnd service ID is valid

#if EPL_DLL_PRES_CHAINING_CN != FALSE
        if (asndServiceId == kDllAsndSyncResponse)
        {
            tEplFrame*  pTxFrameSyncRes;

            pTxFrameSyncRes = (tEplFrame *) dllkInstance_g.pTxBuffer[DLLK_TXFRAME_SYNCRES].m_pbBuffer;
            nodeId = (UINT) AmiGetByteFromLe(&pFrame->m_le_bSrcNodeId);

            if (nodeId == dllkInstance_g.syncReqPrevNodeId)
            {
                UINT32      syncDelayNs;
                syncDelayNs = EplTgtTimeStampTimeDiffNs(dllkInstance_g.pSyncReqPrevTimeStamp,
                                                        pRxBuffer_p->m_pTgtTimeStamp) -
                                                        // Transmission time for SyncReq frame
                                                        (EPL_C_DLL_T_MIN_FRAME + EPL_C_DLL_T_PREAMBLE);

                // update SyncRes frame (SyncDelay and SyncNodeNumber)
                AmiSetDwordToLe(&pTxFrameSyncRes->m_Data.m_Asnd.m_Payload.m_SyncResponse.m_le_dwSyncDelay, syncDelayNs);
                AmiSetDwordToLe(&pTxFrameSyncRes->m_Data.m_Asnd.m_Payload.m_SyncResponse.m_le_dwSyncNodeNumber, (UINT32) nodeId);
                // $$$ m.u.: CbUpdateRelativeLatencyDiff
            }
            else
            {
                AmiSetDwordToLe(&pTxFrameSyncRes->m_Data.m_Asnd.m_Payload.m_SyncResponse.m_le_dwSyncDelay, 0);
                AmiSetDwordToLe(&pTxFrameSyncRes->m_Data.m_Asnd.m_Payload.m_SyncResponse.m_le_dwSyncNodeNumber, (UINT32) 0);
            }

            // update Tx buffer in Edrv
            ret = EdrvUpdateTxMsgBuffer(&dllkInstance_g.pTxBuffer[DLLK_TXFRAME_SYNCRES]);
            if (ret != kEplSuccessful)
                goto Exit;

            // reset stored node ID
            dllkInstance_g.syncReqPrevNodeId = 0;
        }
#endif

        if (dllkInstance_g.aAsndFilter[asndServiceId] == kDllAsndFilterAny)
        {   // ASnd service ID is registered
            // forward frame via async receive FIFO to userspace
            ret = dllkcal_asyncFrameReceived(pFrameInfo_p);
            if (ret != kEplSuccessful)
                goto Exit;
        }
        else if (dllkInstance_g.aAsndFilter[asndServiceId] == kDllAsndFilterLocal)
        {   // ASnd service ID is registered, but only local node ID or broadcasts
            // shall be forwarded
            nodeId = AmiGetByteFromLe(&pFrame->m_le_bDstNodeId);
            if ((nodeId == dllkInstance_g.dllConfigParam.nodeId) || (nodeId == EPL_C_ADR_BROADCAST))
            {   // ASnd frame is intended for us
                // forward frame via async receive FIFO to userspace
                ret = dllkcal_asyncFrameReceived(pFrameInfo_p);
                if (ret != kEplSuccessful)
                    goto Exit;
            }
        }
    }

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Forward RPDO frame

The function is called if PREs or PReq frame was received. It posts the frame to
the event queue. It is called in states NMT_CS_READY_TO_OPERATE and
NMT_CS_OPERATIONAL. The passed PDO needs to be valid.

\param  pFrameInfo_p        Pointer to frame information.

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
static tEplKernel forwardRpdo(tFrameInfo * pFrameInfo_p)
{
    tEplKernel      ret = kEplSuccessful;

    if (dllkInstance_g.pfnCbProcessRpdo != NULL)
    {
        ret = dllkInstance_g.pfnCbProcessRpdo(pFrameInfo_p);
    }
    return ret;
}

///\}

