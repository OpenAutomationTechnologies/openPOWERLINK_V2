/**
********************************************************************************
\file   dllkframe.c

\brief  Frame processing functions of kernel DLL module

This file contains the frame processing functions of the kernel DLL module.

\ingroup module_dllk
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2018, B&R Industrial Automation GmbH
Copyright (c) 2015, SYSTEC electronic GmbH
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
#include <stddef.h>
#include "dllkframe.h"
#include "dllknode.h"
#include "dllkstatemachine.h"

#include <kernel/dllktgt.h>
#include <kernel/dllkfilter.h>
#include <kernel/dllkcal.h>
#include <kernel/eventk.h>
#include <kernel/errhndk.h>
#include <kernel/edrvcyclic.h>

#if (CONFIG_TIMER_USE_HIGHRES != FALSE)
#include <kernel/hrestimer.h>
#endif

#if (CONFIG_DLL_PROCESS_SYNC == DLL_PROCESS_SYNC_ON_TIMER)
#include <kernel/synctimer.h>
#endif

#include <kernel/timesynck.h>

#include <common/ami.h>
#include <oplk/benchmark.h>

#include "dllk-internal.h"

#if (CONFIG_DLL_PRES_CHAINING_CN != FALSE)
#include <kernel/timestamp.h>
#endif

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
static tOplkError processReceivedPreq(tFrameInfo* pFrameInfo_p,
                                      tNmtState nmtState_p,
                                      tEdrvReleaseRxBuffer* pReleaseRxBuffer_p);
static tOplkError processReceivedPres(const tFrameInfo* pFrameInfo_p,
                                      tNmtState nmtState_p,
                                      tNmtEvent* pNmtEvent_p,
                                      tEdrvReleaseRxBuffer* pReleaseRxBuffer_p);
static tOplkError processReceivedSoc(const tEdrvRxBuffer* pRxBuffer_p,
                                     tNmtState nmtState_p);

#if defined(CONFIG_INCLUDE_NMT_RMN)
static tOplkError processReceivedAmni(const tEdrvRxBuffer* pRxBuffer_p,
                                      tNmtState nmtState_p);
#endif

static tOplkError processReceivedSoa(const tEdrvRxBuffer* pRxBuffer_p,
                                     tNmtState nmtState_p);
static tOplkError processReceivedAsnd(tFrameInfo* pFrameInfo_p,
                                      const tEdrvRxBuffer* pRxBuffer_p,
                                      tNmtState nmtState_p,
                                      tEdrvReleaseRxBuffer* pReleaseRxBuffer_p);
static INLINE tOplkError forwardRpdo(const tFrameInfo* pFrameInfo_p);
static INLINE void       postInvalidFormatError(UINT nodeId_p, tNmtState nmtState_p);
static BOOL       presFrameFormatIsInvalid(const tFrameInfo* pFrameInfo_p,
                                           const tDllkNodeInfo* pIntNodeInfo_p,
                                           tNmtState nodeNmtState_p);

#if defined(CONFIG_INCLUDE_NMT_MN)
static tOplkError checkAndSetSyncEvent(BOOL fPrcSlotFinished_p, UINT nodeId_p);
static tOplkError updateNode(tDllkNodeInfo* pIntNodeInfo_p,
                             UINT nodeId_p,
                             tNmtState nodeNmtState_p,
                             const UINT8* pMacAddr_p);
static tOplkError searchNodeInfo(UINT nodeId_p,
                                 tDllkNodeInfo** ppIntNodeInfo_p,
                                 BOOL* pfPrcSlotFinished_p);
static void       handleErrorSignaling(const tPlkFrame* pFrame_p, UINT nodeId_p);
#endif

#if (CONFIG_TIMER_USE_HIGHRES != FALSE)
static tOplkError cbCnTimer(const tTimerEventArg* pEventArg_p);
#endif

#if (CONFIG_EDRV_AUTO_RESPONSE != FALSE)
static tOplkError enableRxFilter(UINT filterEntry_p, BOOL fEnable_p);
#endif

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

\param[in]      pRxBuffer_p         Pointer to received frame.

\return The function returns a tEdrvReleaseRxBuffer flag to determine if the
        buffer could be released immediately
\retval kEdrvReleaseRxBufferImmediately     Buffer could be released immediately.
\retval kEdrvReleaseRxBufferLater           Buffer must be released later.
*/
//------------------------------------------------------------------------------
tEdrvReleaseRxBuffer dllkframe_processFrameReceived(tEdrvRxBuffer* pRxBuffer_p)
{
    tEdrvReleaseRxBuffer    releaseRxBuffer = kEdrvReleaseRxBufferImmediately;
    tOplkError              ret = kErrorOk;
    tNmtState               nmtState;
    tNmtEvent               nmtEvent = kNmtEventNoEvent;
    tEvent                  event;
    tPlkFrame*              pFrame;
    tFrameInfo              frameInfo;
    tMsgType                msgType;
    TGT_DLLK_DECLARE_FLAGS

    TGT_DLLK_ENTER_CRITICAL_SECTION()

    BENCHMARK_MOD_02_SET(3);
    nmtState = dllkInstance_g.nmtState;
    if (nmtState <= kNmtGsResetConfiguration)
        goto Exit;

    pFrame = (tPlkFrame*)pRxBuffer_p->pBuffer;

    frameInfo.frame.pBuffer = pFrame;
    frameInfo.frameSize = (UINT)pRxBuffer_p->rxFrameSize;

    if (ami_getUint16Be(&pFrame->etherType) != C_DLL_ETHERTYPE_EPL)
    {   // non-POWERLINK frame
        DEBUG_LVL_DLL_TRACE("%s(): pfnCbAsync=0x%p SrcMAC=0x%llx\n",
                            __func__,
                            dllkInstance_g.pfnCbAsync,
                            ami_getUint48Be(pFrame->aSrcMac));

        if (dllkInstance_g.pfnCbAsync != NULL)
        {   // handler for async frames is registered
            ret = dllkInstance_g.pfnCbAsync(&frameInfo, &releaseRxBuffer);
        }
        goto Exit;
    }

    msgType = (tMsgType)ami_getUint8Le(&pFrame->messageType);
    switch (msgType)
    {
        case kMsgTypePreq:
            if (ami_getUint8Le(&pFrame->dstNodeId) != dllkInstance_g.dllConfigParam.nodeId)
            {   // this PReq is not intended for us
                goto Exit;
            }
            nmtEvent = kNmtEventDllCePreq;
            ret = processReceivedPreq(&frameInfo, nmtState, &releaseRxBuffer);
            if (ret != kErrorOk)
                goto Exit;
            break;

        case kMsgTypePres:
            ret = processReceivedPres(&frameInfo, nmtState, &nmtEvent, &releaseRxBuffer);
            if (ret != kErrorOk)
                goto Exit;
            break;

        case kMsgTypeSoc:
            nmtEvent = kNmtEventDllCeSoc;
            ret = processReceivedSoc(pRxBuffer_p, nmtState);
            if (ret != kErrorOk)
                goto Exit;
            break;

#if defined(CONFIG_INCLUDE_NMT_RMN)
        case kMsgTypeAmni:
            nmtEvent = kNmtEventDllReAmni;
            ret = processReceivedAmni(pRxBuffer_p, nmtState);
            if (ret != kErrorOk)
                goto Exit;
            break;
#endif

#if defined(CONFIG_INCLUDE_MASND)
        case kMsgTypeAInv:
            nmtEvent = kNmtEventDllCeAInv;
            ret = processReceivedSoa(pRxBuffer_p, nmtState);
            if (ret != kErrorOk)
                goto Exit;
            break;
#endif

        case kMsgTypeSoa:
            nmtEvent = kNmtEventDllCeSoa;
            ret = processReceivedSoa(pRxBuffer_p, nmtState);
            if (ret != kErrorOk)
                goto Exit;
            break;

        case kMsgTypeAsnd:
            nmtEvent = kNmtEventDllCeAsnd;
            ret = processReceivedAsnd(&frameInfo, pRxBuffer_p, nmtState, &releaseRxBuffer);
            if (ret != kErrorOk)
                goto Exit;
            break;

        default:
            break;
    }

    if (nmtEvent != kNmtEventNoEvent)
    {   // event for DLL and NMT state machine generated
        ret = dllkstatemachine_changeState(nmtEvent, nmtState);
        if (ret != kErrorOk)
            goto Exit;

        if (((nmtEvent != kNmtEventDllCeAsnd) && (nmtEvent != kNmtEventDllCeAInv)) &&
            ((nmtState <= kNmtCsPreOperational1) || (nmtEvent != kNmtEventDllCePres)))
        {   // NMT state machine is not interested in ASnd frames and PRes frames when not CsNotActive or CsPreOp1
            // inform NMT module
            event.eventSink = kEventSinkNmtk;
            event.eventType = kEventTypeNmtEvent;
            event.eventArgSize = sizeof(nmtEvent);
            event.eventArg.pEventArg = &nmtEvent;
            ret = eventk_postEvent(&event);
        }
    }

Exit:
    if (ret != kErrorOk)
    {
        UINT32 arg;

        BENCHMARK_MOD_02_TOGGLE(7);
        arg = dllkInstance_g.dllState | (nmtEvent << 8);
        // Error event for API layer
        eventk_postError(kEventSourceDllk, ret, sizeof(arg), &arg);
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

\param[in]      pTxBuffer_p         Pointer to TxBuffer structure of transmitted frame.

*/
//------------------------------------------------------------------------------
void dllkframe_processTransmittedNmtReq(tEdrvTxBuffer* pTxBuffer_p)
{
    tOplkError              ret = kErrorOk;
    tEvent                  event;
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
    if (NMT_IF_ACTIVE_MN(nmtState))
    {
        tPlkFrame* pTxFrame;

        // check if this frame is a NMT command,
        // then forward this frame back to NmtMnu module,
        // because it needs the time, when this frame is
        // actually sent, to start the timer for monitoring
        // the NMT state change.
        pTxFrame = (tPlkFrame*)pTxBuffer_p->pBuffer;
        if ((ami_getUint8Le(&pTxFrame->messageType) == (UINT8)kMsgTypeAsnd) &&
            (ami_getUint8Le(&pTxFrame->data.asnd.serviceId) == (UINT8)kDllAsndNmtCommand))
        {   // post event directly to NmtMnu module
            event.eventSink = kEventSinkNmtMnu;
            event.eventType = kEventTypeNmtMnuNmtCmdSent;
            event.eventArgSize = (UINT)pTxBuffer_p->txFrameSize;
            event.eventArg.pEventArg = pTxFrame;
            //PRINTF("%s TxB=%p, TxF=%p, s=%u\n", __func__, pTxBuffer_p, event.eventArg, event.eventArgSize);
            ret = eventk_postEvent(&event);
            if (ret != kErrorOk)
                goto Exit;
        }
    }
#endif

    // frame from NMT request FIFO sent
    // mark Tx-buffer as empty
    dllkInstance_g.aTxBufferStateNmtReq[pTxBuffer_p - &dllkInstance_g.pTxBuffer[handle]] = kDllkTxBufEmpty;

#if (CONFIG_EDRV_AUTO_RESPONSE != FALSE)
    // Disable the filter to avoid retransmission of the same frame
    if ((dllkInstance_g.nmtState & (NMT_TYPE_MASK | NMT_SUPERSTATE_MASK)) == (NMT_TYPE_CS | NMT_CS_PLKMODE))
    {
        ret = enableRxFilter(DLLK_FILTER_SOA_NMTREQ, FALSE);
        if (ret != kErrorOk)
            goto Exit;
    }

    // decrement RS in Flag 2
    if ((dllkInstance_g.flag2 & PLK_FRAME_FLAG2_RS) != 0)
    {
        dllkInstance_g.flag2--;
        dllkInstance_g.updateTxFrame = DLLK_UPDATE_BOTH;
    }

    ret = dllkframe_updateFrameAsyncRes(nmtState);
    if (ret != kErrorOk)
        goto Exit;
#endif

    // post event to DLL
    priority = kDllAsyncReqPrioNmt;
    event.eventSink = kEventSinkDllk;
    event.eventType = kEventTypeDllkFillTx;
    OPLK_MEMSET(&event.netTime, 0x00, sizeof(event.netTime));
    event.eventArg.pEventArg = &priority;
    event.eventArgSize = sizeof(priority);
    ret = eventk_postEvent(&event);
    if (ret != kErrorOk)
        goto Exit;

Exit:
    if (ret != kErrorOk)
    {
        BENCHMARK_MOD_02_TOGGLE(7);
        arg = dllkInstance_g.dllState | (handle << 16);
        eventk_postError(kEventSourceDllk, ret, sizeof(arg), &arg);
    }

    TGT_DLLK_LEAVE_CRITICAL_SECTION()
}

//------------------------------------------------------------------------------
/**
\brief  Callback function for transmitted non-POWERLINK frame

The function implements the callback function which is called when a non-
POWERLINK frame was transmitted.

\param[in]      pTxBuffer_p         Pointer to TxBuffer structure of transmitted frame.

*/
//------------------------------------------------------------------------------
void dllkframe_processTransmittedNonPlk(tEdrvTxBuffer* pTxBuffer_p)
{
    tOplkError              ret = kErrorOk;
    tEvent                  event;
    tDllAsyncReqPriority    priority;
    tNmtState               nmtState;
    UINT                    handle = DLLK_TXFRAME_NONPLK;
    UINT32                  arg;

    TGT_DLLK_DECLARE_FLAGS

    TGT_DLLK_ENTER_CRITICAL_SECTION()

    nmtState = dllkInstance_g.nmtState;
    if (nmtState <= kNmtGsResetConfiguration)
        goto Exit;

    // frame from generic priority FIFO sent
    // mark Tx-buffer as empty
   dllkInstance_g.aTxBufferStateNonPlk[pTxBuffer_p - &dllkInstance_g.pTxBuffer[handle]] = kDllkTxBufEmpty;

#if (CONFIG_EDRV_AUTO_RESPONSE != FALSE)
    // Disable the filter to avoid retransmission of the same frame
    if ((dllkInstance_g.nmtState & (NMT_TYPE_MASK | NMT_SUPERSTATE_MASK)) == (NMT_TYPE_CS | NMT_CS_PLKMODE))
    {
        ret = enableRxFilter(DLLK_FILTER_SOA_NONPLK, FALSE);
        if (ret != kErrorOk)
            goto Exit;
    }

    // decrement RS in Flag 2
    if ((dllkInstance_g.flag2 & PLK_FRAME_FLAG2_RS) != 0)
    {
        dllkInstance_g.flag2--;
        dllkInstance_g.updateTxFrame = DLLK_UPDATE_BOTH;
    }

    ret = dllkframe_updateFrameAsyncRes(nmtState);
    if (ret != kErrorOk)
        goto Exit;
#endif

    // post event to DLL
    priority = kDllAsyncReqPrioGeneric;
    event.eventSink = kEventSinkDllk;
    event.eventType = kEventTypeDllkFillTx;
    OPLK_MEMSET(&event.netTime, 0x00, sizeof(event.netTime));
    event.eventArg.pEventArg = &priority;
    event.eventArgSize = sizeof(priority);
    ret = eventk_postEvent(&event);

Exit:
    if (ret != kErrorOk)
    {
        BENCHMARK_MOD_02_TOGGLE(7);
        arg = dllkInstance_g.dllState | (handle << 16);
        eventk_postError(kEventSourceDllk, ret, sizeof(arg), &arg);
    }

    TGT_DLLK_LEAVE_CRITICAL_SECTION()
}

//------------------------------------------------------------------------------
/**
\brief  Update IdentResponse frame

The function updates an IdentResponse frame with the specified information.

\param[in]      pTxBuffer_p         Pointer to TX buffer of frame.
\param[in]      nmtState_p          NMT state of node.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
tOplkError dllkframe_updateFrameIdentRes(tEdrvTxBuffer* pTxBuffer_p,
                                         tNmtState nmtState_p)
{
    tOplkError  ret = kErrorOk;
    tPlkFrame*  pTxFrame;

    pTxFrame = (tPlkFrame*)pTxBuffer_p->pBuffer;

    // update frame (NMT state, RD, RS, PR flags)
    ami_setUint8Le(&pTxFrame->data.asnd.payload.identResponse.nmtStatus, (UINT8)nmtState_p);
    ami_setUint8Le(&pTxFrame->data.asnd.payload.identResponse.flag2, dllkInstance_g.flag2);

#if (CONFIG_EDRV_AUTO_RESPONSE != FALSE)
    if (NMT_IF_CN_OR_RMN(nmtState_p))
        ret = edrv_updateTxBuffer(pTxBuffer_p);
#endif

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Update StatusResponse frame

The function updates a StatusResponse frame with the specified information.

\param[in]      pTxBuffer_p         Pointer to TX buffer of frame.
\param[in]      nmtState_p          NMT state of node.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
tOplkError dllkframe_updateFrameStatusRes(tEdrvTxBuffer* pTxBuffer_p,
                                          tNmtState nmtState_p)
{
    tOplkError  ret = kErrorOk;
    tPlkFrame*  pTxFrame;

    pTxFrame = (tPlkFrame*)pTxBuffer_p->pBuffer;

    // update frame (NMT state, RD, RS, PR, EC, EN flags)
    ami_setUint8Le(&pTxFrame->data.asnd.payload.statusResponse.nmtStatus, (UINT8)nmtState_p);
    ami_setUint8Le(&pTxFrame->data.asnd.payload.statusResponse.flag2, dllkInstance_g.flag2);
    ami_setUint8Le(&pTxFrame->data.asnd.payload.statusResponse.flag1, dllkInstance_g.flag1);

#if (CONFIG_EDRV_AUTO_RESPONSE != FALSE)
    if (NMT_IF_CN_OR_RMN(nmtState_p))
        ret = edrv_updateTxBuffer(pTxBuffer_p);
#endif

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Update PRes frame

The function updates a PRes frame with the specified information.

\param[in]      pTxBuffer_p         Pointer to TX buffer of frame.
\param[in]      nmtState_p          NMT state of node.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
tOplkError dllkframe_updateFramePres(tEdrvTxBuffer* pTxBuffer_p,
                                     tNmtState nmtState_p)
{
    tOplkError  ret = kErrorOk;
    tPlkFrame*  pTxFrame;
    UINT8       flag1;

    pTxFrame = (tPlkFrame*)pTxBuffer_p->pBuffer;

    // update frame (NMT state, RD, RS, PR, MS, EN flags)
    ami_setUint8Le(&pTxFrame->data.pres.nmtStatus, (UINT8)nmtState_p);
    ami_setUint8Le(&pTxFrame->data.pres.flag2, dllkInstance_g.flag2);

    // get RD flag
    flag1 = ami_getUint8Le(&pTxFrame->data.pres.flag1) & PLK_FRAME_FLAG1_RD;

    if ((dllkInstance_g.dllConfigParam.multipleCycleCnt > 0) &&
        (dllkInstance_g.mnFlag1 & PLK_FRAME_FLAG1_MS))  // MS flag set in PReq
    {   // set MS flag, because PRes will be sent multiplexed with other CNs
        flag1 |= PLK_FRAME_FLAG1_MS;
    }

    // add EN flag from error signaling module
    flag1 |= dllkInstance_g.flag1 & PLK_FRAME_FLAG1_EN;

    if (nmtState_p != kNmtCsOperational)
    {   // mark PDO as invalid in all NMT states but OPERATIONAL - reset only RD flag
        flag1 &= ~PLK_FRAME_FLAG1_RD;
    }
    ami_setUint8Le(&pTxFrame->data.pres.flag1, flag1);        // update frame (flag1)

#if (CONFIG_EDRV_AUTO_RESPONSE != FALSE)
//    if (NmtState_p < kNmtMsNotActive)
    {   // currently, this function is only called on CN
        ret = edrv_updateTxBuffer(pTxBuffer_p);
    }
#endif

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Update CN asynchronous response frames

The function updates the CN asynchronous response frames Status Response and
Ident Response depending on dllkInstance_g.updateTxFrame.

\param[in]      nmtState_p          NMT state of the node.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
tOplkError dllkframe_updateFrameAsyncRes(tNmtState nmtState_p)
{
    tOplkError      ret = kErrorOk;
    tEdrvTxBuffer*  pTxBuffer;

    switch (dllkInstance_g.updateTxFrame)
    {
        case DLLK_UPDATE_BOTH:
            dllkInstance_g.curTxBufferOffsetIdentRes ^= 1;
            pTxBuffer = &dllkInstance_g.pTxBuffer[DLLK_TXFRAME_IDENTRES +
                                                  dllkInstance_g.curTxBufferOffsetIdentRes];
            if (pTxBuffer->pBuffer != NULL)
            {   // IdentRes does exist
                ret = dllkframe_updateFrameIdentRes(pTxBuffer, nmtState_p);
                if (ret != kErrorOk)
                    return ret;
            }
            // fall-through

        case DLLK_UPDATE_STATUS:
            dllkInstance_g.curTxBufferOffsetStatusRes ^= 1;
            pTxBuffer = &dllkInstance_g.pTxBuffer[DLLK_TXFRAME_STATUSRES +
                                                  dllkInstance_g.curTxBufferOffsetStatusRes];
            if (pTxBuffer->pBuffer != NULL)
            {   // StatusRes does exist
                ret = dllkframe_updateFrameStatusRes(pTxBuffer, nmtState_p);
                if (ret != kErrorOk)
                    return ret;
            }

            // reset signal variable
            dllkInstance_g.updateTxFrame = DLLK_UPDATE_NONE;
            break;

        default:
            break;
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Check frame

The function checks a frame and sets the missing information.
It sets fields in the Ethernet and POWERLINK header parts.

\param[in]      pFrame_p            Pointer to frame.
\param[in]      frameSize_p         Size of the frame

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
tOplkError dllkframe_checkFrame(tPlkFrame* pFrame_p, size_t frameSize_p)
{
    tOplkError  ret = kErrorOk;
    tMsgType    msgType;
    UINT16      etherType;

    // Check if accessed memory is valid before accessing it!
    if ((pFrame_p != NULL) && (frameSize_p > offsetof(tPlkFrame, srcNodeId)))
    {
        // check SrcMAC
        if (ami_getUint48Be(pFrame_p->aSrcMac) == 0)
        {
            // source MAC address
            OPLK_MEMCPY(&pFrame_p->aSrcMac[0], edrv_getMacAddr(), 6);
        }

        // check ethertype
        etherType = ami_getUint16Be(&pFrame_p->etherType);
        if (etherType == 0)
        {
            // assume POWERLINK frame
            etherType = C_DLL_ETHERTYPE_EPL;
            ami_setUint16Be(&pFrame_p->etherType, etherType);
        }

        if (etherType == C_DLL_ETHERTYPE_EPL)
        {
            // source node ID
            ami_setUint8Le(&pFrame_p->srcNodeId, dllkInstance_g.dllConfigParam.nodeId);

            // check message type
            msgType = (tMsgType)ami_getUint8Le(&pFrame_p->messageType);
            if (msgType == 0)
            {
                msgType = kMsgTypeAsnd;
                ami_setUint8Le(&pFrame_p->messageType, (UINT8)msgType);
            }

            if (msgType == kMsgTypeAsnd)
            {
                // destination MAC address
                ami_setUint48Be(&pFrame_p->aDstMac[0], C_DLL_MULTICAST_ASND);
            }
        }
    }
    else
        ret = kErrorDllTxFrameInvalid;

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Create TX frame

The function creates the buffer for a TX frame and registers it to the Ethernet
driver.

\param[in,out]  pHandle_p           Handle to the last allocated frame buffer used for
                                    faster search for PReq buffers. The function stores
                                    the handle to the new frame buffer at this location.
\param[in,out]  pFrameSize_p        Pointer to the size of the frame. The function stores
                                    the size of the new frame at this location. It is
                                    always equal or larger than the requested size. If
                                    that is not possible an error will be generated.
\param[in]      msgType_p           The message type of the frame.
\param[in]      serviceId_p         The service ID in case of an ASnd frame. Otherwise
                                    kDllAsndNotDefined.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
tOplkError dllkframe_createTxFrame(UINT* pHandle_p,
                                   size_t* pFrameSize_p,
                                   tMsgType msgType_p,
                                   tDllAsndServiceId serviceId_p)
{
    tOplkError      ret = kErrorOk;
    tPlkFrame*      pTxFrame;
    UINT            handle = *pHandle_p;
    tEdrvTxBuffer*  pTxBuffer = NULL;
    UINT            nIndex = 0;

    switch (msgType_p)
    {
        case kMsgTypeAsnd:
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

#if (CONFIG_DLL_PRES_CHAINING_CN != FALSE)
                case kDllAsndSyncResponse:
                    handle = DLLK_TXFRAME_SYNCRES;
                    break;
#endif

                case kDllAsndNotDefined:
                    ret = kErrorDllInvalidParam;
                    goto Exit;

                case kDllAsndSdo:
#if ((CONFIG_DLL_PRES_CHAINING_CN == FALSE) && defined(CONFIG_INCLUDE_NMT_MN))
                case kDllAsndSyncResponse:
#endif
                    ret = kErrorEdrvBufNotExisting;
                    goto Exit;
            }
            break;

        case kMsgTypeNonPowerlink:
            handle = DLLK_TXFRAME_NONPLK;
            break;

        case kMsgTypePres:
            handle = DLLK_TXFRAME_PRES;
            break;

#if defined(CONFIG_INCLUDE_NMT_RMN)
        case kMsgTypeAmni:
            handle = DLLK_TXFRAME_AMNI;
            break;
#endif

#if defined(CONFIG_INCLUDE_NMT_MN)
        case kMsgTypeSoc:
            handle = DLLK_TXFRAME_SOC;
            break;

        case kMsgTypeSoa:
            handle = DLLK_TXFRAME_SOA;
            break;

        case kMsgTypePreq:
          // look for free entry
            if ((handle < DLLK_TXFRAME_PREQ) || (handle >= dllkInstance_g.maxTxFrames))
            {   // start with first PReq buffer
                handle = DLLK_TXFRAME_PREQ;
            }

            // otherwise start with last allocated handle
            pTxBuffer = &dllkInstance_g.pTxBuffer[handle];
            for (; handle < dllkInstance_g.maxTxFrames; handle++, pTxBuffer++)
            {
                if (pTxBuffer->pBuffer == NULL)
                {   // free entry found
                    break;
                }
            }

            if (pTxBuffer->pBuffer != NULL)
            {
                ret = kErrorEdrvNoFreeBufEntry;
                goto Exit;
            }
            break;

#endif

        default:
            ret = kErrorEdrvBufNotExisting;
            goto Exit;
    }

    *pHandle_p = handle;

    for (; nIndex < 2; nIndex++, handle++)
    {
        // test if requested entry is free
        pTxBuffer = &dllkInstance_g.pTxBuffer[handle];
        if (pTxBuffer->pBuffer != NULL)
        {   // entry is not free
            ret = kErrorEdrvNoFreeBufEntry;
            goto Exit;
        }

        // setup Tx buffer
        pTxBuffer->maxBufferSize = *pFrameSize_p;

        ret = edrv_allocTxBuffer(pTxBuffer);
        if (ret != kErrorOk)
        {   // error occurred while registering Tx frame
            goto Exit;
        }

        // because buffer size may be larger than requested, save real length of frame
        pTxBuffer->txFrameSize = *pFrameSize_p;
        // initialize time offset
        pTxBuffer->timeOffsetNs = 0;
        // fill whole frame with 0
        OPLK_MEMSET(pTxBuffer->pBuffer, 0, pTxBuffer->maxBufferSize);
        pTxFrame = (tPlkFrame*)pTxBuffer->pBuffer;

        if (msgType_p != kMsgTypeNonPowerlink)
        {   // fill out Frame only if it is a POWERLINK frame
            ami_setUint16Be(&pTxFrame->etherType, C_DLL_ETHERTYPE_EPL);
            ami_setUint8Le(&pTxFrame->srcNodeId, dllkInstance_g.dllConfigParam.nodeId);
            OPLK_MEMCPY(&pTxFrame->aSrcMac[0], edrv_getMacAddr(), 6);

            switch (msgType_p)
            {
                case kMsgTypeAsnd:
                    // destination MAC address
                    ami_setUint48Be(&pTxFrame->aDstMac[0], C_DLL_MULTICAST_ASND);
                    // destination node ID
                    switch (serviceId_p)
                    {
                        case kDllAsndIdentResponse:
                            ami_setUint8Le(&pTxFrame->data.asnd.payload.identResponse.powerlinkProfileVersion,
                                           (UINT8)PLK_SPEC_VERSION);
                            ami_setUint32Le(&pTxFrame->data.asnd.payload.identResponse.featureFlagsLe,
                                            dllkInstance_g.dllConfigParam.featureFlags);
                            ami_setUint16Le(&pTxFrame->data.asnd.payload.identResponse.mtuLe,
                                            dllkInstance_g.dllConfigParam.asyncMtu);
                            ami_setUint16Le(&pTxFrame->data.asnd.payload.identResponse.pollInSizeLe,
                                            dllkInstance_g.dllConfigParam.preqActPayloadLimit);
                            ami_setUint16Le(&pTxFrame->data.asnd.payload.identResponse.pollOutSizeLe,
                                            dllkInstance_g.dllConfigParam.presActPayloadLimit);
                            ami_setUint32Le(&pTxFrame->data.asnd.payload.identResponse.responseTimeLe,
                                            dllkInstance_g.dllConfigParam.presMaxLatency);
                            ami_setUint32Le(&pTxFrame->data.asnd.payload.identResponse.deviceTypeLe,
                                            dllkInstance_g.dllIdentParam.deviceType);
                            ami_setUint32Le(&pTxFrame->data.asnd.payload.identResponse.vendorIdLe,
                                            dllkInstance_g.dllIdentParam.vendorId);
                            ami_setUint32Le(&pTxFrame->data.asnd.payload.identResponse.productCodeLe,
                                            dllkInstance_g.dllIdentParam.productCode);
                            ami_setUint32Le(&pTxFrame->data.asnd.payload.identResponse.revisionNumberLe,
                                            dllkInstance_g.dllIdentParam.revisionNumber);
                            ami_setUint32Le(&pTxFrame->data.asnd.payload.identResponse.serialNumberLe,
                                            dllkInstance_g.dllIdentParam.serialNumber);
                            ami_setUint64Le(&pTxFrame->data.asnd.payload.identResponse.vendorSpecificExt1Le,
                                            dllkInstance_g.dllIdentParam.vendorSpecificExt1);
                            ami_setUint32Le(&pTxFrame->data.asnd.payload.identResponse.verifyConfigurationDateLe,
                                            dllkInstance_g.dllIdentParam.verifyConfigurationDate);
                            ami_setUint32Le(&pTxFrame->data.asnd.payload.identResponse.verifyConfigurationTimeLe,
                                            dllkInstance_g.dllIdentParam.verifyConfigurationTime);
                            ami_setUint32Le(&pTxFrame->data.asnd.payload.identResponse.applicationSwDateLe,
                                            dllkInstance_g.dllIdentParam.applicationSwDate);
                            ami_setUint32Le(&pTxFrame->data.asnd.payload.identResponse.applicationSwTimeLe,
                                            dllkInstance_g.dllIdentParam.applicationSwTime);
                            ami_setUint32Le(&pTxFrame->data.asnd.payload.identResponse.ipAddressLe,
                                            dllkInstance_g.dllIdentParam.ipAddress);
                            ami_setUint32Le(&pTxFrame->data.asnd.payload.identResponse.subnetMaskLe,
                                            dllkInstance_g.dllIdentParam.subnetMask);
                            ami_setUint32Le(&pTxFrame->data.asnd.payload.identResponse.defaultGatewayLe,
                                            dllkInstance_g.dllIdentParam.defaultGateway);
                            OPLK_MEMCPY(&pTxFrame->data.asnd.payload.identResponse.sHostName[0],
                                        &dllkInstance_g.dllIdentParam.sHostname[0],
                                        sizeof(dllkInstance_g.dllIdentParam.sHostname));
                            OPLK_MEMCPY(&pTxFrame->data.asnd.payload.identResponse.aVendorSpecificExt2[0],
                                        &dllkInstance_g.dllIdentParam.aVendorSpecificExt2[0],
                                        sizeof(dllkInstance_g.dllIdentParam.aVendorSpecificExt2));
                            // fall-through

                        case kDllAsndStatusResponse:
                            // IdentResponses and StatusResponses are broadcast
                            ami_setUint8Le(&pTxFrame->dstNodeId, (UINT8)C_ADR_BROADCAST);
                            break;

#if (CONFIG_DLL_PRES_CHAINING_CN != FALSE)
                        case kDllAsndSyncResponse:
                            // SyncRes destination node ID is MN node ID
                            ami_setUint8Le(&pTxFrame->dstNodeId, (UINT8)C_ADR_MN_DEF_NODE_ID);
                            ami_setUint32Le(&pTxFrame->data.asnd.payload.syncResponse.latencyLe,
                                            dllkInstance_g.dllConfigParam.syncResLatency);
                            // SyncStatus: PResMode disabled / PResTimeFirst and PResTimeSecond invalid
                            // ami_setUint32Le(&pTxFrame->data.asnd.payload.syncResponse.syncStatusLe, 0);
                            // init SyncNodeNumber
                            // ami_setUint32Le(&pTxFrame->data.asnd.payload.syncResponse.syncNodeNumberLe, 0);
                            // init SyncDelay
                            // ami_setUint32Le(&pTxFrame->data.asnd.payload.syncResponse.syncDelayLe, 0);
                            break;
#endif

                        default:
                            break;
                    }

                    // ASnd Service ID
                    ami_setUint8Le(&pTxFrame->data.asnd.serviceId, serviceId_p);
                    break;

                case kMsgTypePres:
                    ami_setUint48Be(&pTxFrame->aDstMac[0], C_DLL_MULTICAST_PRES);
                    ami_setUint8Le(&pTxFrame->dstNodeId, (UINT8)C_ADR_BROADCAST);
                    break;

#if defined(CONFIG_INCLUDE_NMT_RMN)
                case kMsgTypeAmni:
                    ami_setUint48Be(&pTxFrame->aDstMac[0], C_DLL_MULTICAST_AMNI);
                    ami_setUint8Le(&pTxFrame->dstNodeId, (UINT8)C_ADR_BROADCAST);
                    break;
#endif

#if defined(CONFIG_INCLUDE_NMT_MN)
                case kMsgTypeSoc:
                    ami_setUint8Le(&pTxFrame->srcNodeId, (UINT8)C_ADR_MN_DEF_NODE_ID);
                    ami_setUint48Be(&pTxFrame->aDstMac[0], C_DLL_MULTICAST_SOC);
                    ami_setUint8Le(&pTxFrame->dstNodeId, (UINT8)C_ADR_BROADCAST);
                    break;

                case kMsgTypeSoa:
                    ami_setUint8Le(&pTxFrame->srcNodeId, (UINT8)C_ADR_MN_DEF_NODE_ID);
                    ami_setUint48Be(&pTxFrame->aDstMac[0], C_DLL_MULTICAST_SOA);
                    ami_setUint8Le(&pTxFrame->dstNodeId, (UINT8)C_ADR_BROADCAST);
                    ami_setUint8Le(&pTxFrame->data.soa.powerlinkVersion, (UINT8)PLK_SPEC_VERSION);
#if defined(CONFIG_INCLUDE_NMT_RMN)
                    if (dllkInstance_g.fRedundancy)
                        ami_setUint8Le(&pTxFrame->data.soa.flag3, (UINT8)PLK_FRAME_FLAG3_MR);
#endif
                    break;

                case kMsgTypePreq:
                    ami_setUint8Le(&pTxFrame->srcNodeId, (UINT8)C_ADR_MN_DEF_NODE_ID);
                    break;
#endif

                default:
                    break;
            }

            // POWERLINK message type
            ami_setUint8Le(&pTxFrame->messageType, (UINT8)msgType_p);
        }
    }

    *pFrameSize_p = pTxBuffer->maxBufferSize;

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Delete TX frame

The function deletes the buffer for a TX frame and frees it in the Ethernet
driver.

\param[in]      handle_p            Handle of the frame buffer.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
tOplkError dllkframe_deleteTxFrame(UINT handle_p)
{
    tOplkError      ret = kErrorOk;
    tEdrvTxBuffer*  pTxBuffer = NULL;
    UINT            nIndex = 0;

    if (handle_p >= dllkInstance_g.maxTxFrames)
    {   // handle is not valid
        return kErrorDllIllegalHdl;
    }

    for (; nIndex < 2; nIndex++, handle_p++)
    {
        pTxBuffer = &dllkInstance_g.pTxBuffer[handle_p];

        ret = edrv_freeTxBuffer(pTxBuffer);
        if (ret != kErrorOk)
        {   // error occurred while releasing Tx frame
            return ret;
        }

        pTxBuffer->pBuffer = NULL;
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Process TPDO frame

The function forwards the specified TPDO for processing to the registered
callback function (i.e. to the PDO module).

\param[in]      pFrameInfo_p        Pointer to frame information.
\param[in]      fReadyFlag_p        Ready flag.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
tOplkError dllkframe_processTpdo(tFrameInfo* pFrameInfo_p, BOOL fReadyFlag_p)
{
    tOplkError ret = kErrorOk;

    if (dllkInstance_g.pfnCbProcessTpdo != NULL)
        ret = dllkInstance_g.pfnCbProcessTpdo(pFrameInfo_p, fReadyFlag_p);

    return ret;
}

#if defined(CONFIG_INCLUDE_NMT_MN)
//------------------------------------------------------------------------------
/**
\brief  Callback function for transmitted SoC frame

The function implements the callback function which is called when a SoC
frame was transmitted.

\param[in]      pTxBuffer_p         Pointer to TxBuffer structure of transmitted frame.

*/
//------------------------------------------------------------------------------
void dllkframe_processTransmittedSoc(tEdrvTxBuffer* pTxBuffer_p)
{
    tOplkError  ret = kErrorOk;
    tNmtState   nmtState;
    UINT        handle = DLLK_TXFRAME_SOC;
    UINT32      arg;

    TGT_DLLK_DECLARE_FLAGS

    UNUSED_PARAMETER(pTxBuffer_p);

    TGT_DLLK_ENTER_CRITICAL_SECTION()

    nmtState = dllkInstance_g.nmtState;
    if (nmtState <= kNmtGsResetConfiguration)
        goto Exit;

    // SoC frame sent
    ret = dllkstatemachine_changeState(kNmtEventDllMeAsndTimeout, nmtState);
    if (ret != kErrorOk)
        goto Exit;

Exit:
    if (ret != kErrorOk)
    {
        BENCHMARK_MOD_02_TOGGLE(7);
        arg = dllkInstance_g.dllState | (handle << 16);
        eventk_postError(kEventSourceDllk, ret, sizeof(arg), &arg);
    }

    TGT_DLLK_LEAVE_CRITICAL_SECTION()
}

//------------------------------------------------------------------------------
/**
\brief  Callback function for transmitted SoA frame

The function implements the callback function which is called when a SoA
frame was transmitted.

\param[in]      pTxBuffer_p         Pointer to TxBuffer structure of transmitted frame.

*/
//------------------------------------------------------------------------------
void dllkframe_processTransmittedSoa(tEdrvTxBuffer* pTxBuffer_p)
{
    tOplkError  ret = kErrorOk;
    tNmtState   nmtState;
    UINT        handle = DLLK_TXFRAME_SOA;
    UINT32      arg;

    TGT_DLLK_DECLARE_FLAGS

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
                if (dllkInstance_g.pTxBuffer[DLLK_TXFRAME_STATUSRES + dllkInstance_g.curTxBufferOffsetStatusRes].pBuffer != NULL)
                {   // StatusRes does exist
                    // send StatusRes
                    ret = edrv_sendTxBuffer(&dllkInstance_g.pTxBuffer[DLLK_TXFRAME_STATUSRES + dllkInstance_g.curTxBufferOffsetStatusRes]);
                    if (ret != kErrorOk)
                        goto Exit;
                    TGT_DBG_SIGNAL_TRACE_POINT(8);
                }
                break;

            case kDllReqServiceIdent:
                if (dllkInstance_g.pTxBuffer[DLLK_TXFRAME_IDENTRES + dllkInstance_g.curTxBufferOffsetIdentRes].pBuffer != NULL)
                {   // IdentRes does exist
                    // send IdentRes
                    ret = edrv_sendTxBuffer(&dllkInstance_g.pTxBuffer[DLLK_TXFRAME_IDENTRES + dllkInstance_g.curTxBufferOffsetIdentRes]);
                    if (ret != kErrorOk)
                        goto Exit;
                    TGT_DBG_SIGNAL_TRACE_POINT(7);
                }
                break;

            case kDllReqServiceNmtRequest:
                if (dllkInstance_g.pTxBuffer[DLLK_TXFRAME_NMTREQ + dllkInstance_g.curTxBufferOffsetNmtReq].pBuffer != NULL)
                {   // NmtRequest does exist
                    // check if frame is not empty and not being filled
                    if (dllkInstance_g.aTxBufferStateNmtReq[dllkInstance_g.curTxBufferOffsetNmtReq] == kDllkTxBufReady)
                    {
                        // send NmtRequest
                        dllkInstance_g.aTxBufferStateNmtReq[dllkInstance_g.curTxBufferOffsetNmtReq] = kDllkTxBufSending;
                        ret = edrv_sendTxBuffer(&dllkInstance_g.pTxBuffer[DLLK_TXFRAME_NMTREQ + dllkInstance_g.curTxBufferOffsetNmtReq]);
                        if (ret != kErrorOk)
                            goto Exit;
                        dllkInstance_g.curTxBufferOffsetNmtReq ^= 1;
                    }
                }
                break;

            case kDllReqServiceUnspecified:
                if (dllkInstance_g.pTxBuffer[DLLK_TXFRAME_NONPLK + dllkInstance_g.curTxBufferOffsetNonPlk].pBuffer != NULL)
                {   // non-POWERLINK frame does exist
                    // check if frame is not empty and not being filled
                    if (dllkInstance_g.aTxBufferStateNonPlk[dllkInstance_g.curTxBufferOffsetNonPlk] == kDllkTxBufReady)
                    {
                        // send non-POWERLINK frame
                        dllkInstance_g.aTxBufferStateNonPlk[dllkInstance_g.curTxBufferOffsetNonPlk] = kDllkTxBufSending;
                        ret = edrv_sendTxBuffer(&dllkInstance_g.pTxBuffer[DLLK_TXFRAME_NONPLK + dllkInstance_g.curTxBufferOffsetNonPlk]);
                        if (ret != kErrorOk)
                            goto Exit;

                        dllkInstance_g.curTxBufferOffsetNonPlk ^= 1;
                    }
                }
                break;

            default:
                break;
        }

        // ASnd frame was sent, remove the request
        dllkInstance_g.aLastReqServiceId[dllkInstance_g.curLastSoaReq] = kDllReqServiceNo;
    }
    else
    {
        tPlkFrame*          pFrame = (tPlkFrame*)pTxBuffer_p->pBuffer;
        UINT                nodeId = ami_getUint8Le(&(pFrame->data.soa.reqServiceTarget));
        tDllReqServiceId    serviceId = ami_getUint8Le(&(pFrame->data.soa.reqServiceId));

        ret = dllkcal_ackAsyncRequest(nodeId, serviceId);
        if (ret != kErrorOk)
            goto Exit;
    }

    // reprogram timer in PREOP1
#if (CONFIG_TIMER_USE_HIGHRES != FALSE)
    if ((dllkInstance_g.dllState == kDllMsNonCyclic) &&
        (dllkInstance_g.dllConfigParam.asyncSlotTimeout != 0))
    {
        ret = hrestimer_modifyTimer(&dllkInstance_g.timerHdlCycle,
                                    dllkInstance_g.dllConfigParam.asyncSlotTimeout,
                                    dllkframe_cbMnTimerCycle, 0L, FALSE);
        if (ret != kErrorOk)
            goto Exit;

        // forward event to ErrorHandler and DLLk module
        ret = dllk_postEvent(kEventTypeDllkCycleFinish);
        if (ret != kErrorOk)
            goto Exit;
    }
#endif

    if ((dllkInstance_g.dllState > kDllMsNonCyclic) &&
        (dllkInstance_g.dllConfigParam.syncNodeId > C_ADR_SYNC_ON_SOC) &&
        (dllkInstance_g.fSyncProcessed == FALSE))
    {   // cyclic state is active, so preprocessing is necessary
        dllkInstance_g.fSyncProcessed = TRUE;
        ret = dllk_postEvent(kEventTypeSync);
        if (ret != kErrorOk)
            goto Exit;
    }

Exit:
    if (ret != kErrorOk)
    {
        BENCHMARK_MOD_02_TOGGLE(7);
        arg = dllkInstance_g.dllState | (handle << 16);
        eventk_postError(kEventSourceDllk, ret, sizeof(arg), &arg);
    }

    TGT_DLLK_LEAVE_CRITICAL_SECTION()
}

//------------------------------------------------------------------------------
/**
\brief  Update and transmit SoA

The function updates and transmits an SoA.

\param[in]      nmtState_p          Current NMT state.
\param[out]     pDllStateProposed_p Proposed DLL state.
\param[in]      fEnableInvitation_p Enable invitation for asynchronous phase. It
                                    will be disabled for C_DLL_PREOP1_START_CYCLES
                                    SoAs.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
tOplkError dllkframe_mnSendSoa(tNmtState nmtState_p,
                               tDllState* pDllStateProposed_p,
                               BOOL fEnableInvitation_p)
{
    tOplkError      ret = kErrorOk;
    tEdrvTxBuffer*  pTxBuffer;

    *pDllStateProposed_p = kDllMsNonCyclic;

    pTxBuffer = &dllkInstance_g.pTxBuffer[DLLK_TXFRAME_SOA];
    if (pTxBuffer->pBuffer != NULL)
    {   // SoA does exist
        ret = dllkframe_updateFrameSoa(pTxBuffer,
                                       nmtState_p,
                                       fEnableInvitation_p,
                                       dllkInstance_g.curLastSoaReq);
        if (ret != kErrorOk)
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
        ret = edrv_sendTxBuffer(pTxBuffer);
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Update SoA frame

The function updates an SoA frame.

\param[in]      pTxBuffer_p         Pointer to TX buffer of frame.
\param[in]      nmtState_p          NMT state of the local node.
\param[in]      fEnableInvitation_p Enable the invitation for asynchronous phase.
                                    It will be disabled for the first C_DLL_PREOP1_START_CYCLES
                                    SoAs.
\param[in]      curReq_p            Index of current request.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
tOplkError dllkframe_updateFrameSoa(tEdrvTxBuffer* pTxBuffer_p,
                                    tNmtState nmtState_p,
                                    BOOL fEnableInvitation_p,
                                    UINT8 curReq_p)
{
    tOplkError          ret = kErrorOk;
    tPlkFrame*          pTxFrame;
    tDllkNodeInfo*      pNodeInfo;
    tDllkTxBufState*    pTxBufferState = NULL;

    pTxFrame = (tPlkFrame*)pTxBuffer_p->pBuffer;

    if (fEnableInvitation_p != FALSE)
    {   // fetch target of asynchronous phase
        if (dllkInstance_g.flag2 == 0)
        {   // own queues are empty
            dllkInstance_g.aLastReqServiceId[curReq_p] = kDllReqServiceNo;
        }
        else if (((tDllAsyncReqPriority)(dllkInstance_g.flag2 >> PLK_FRAME_FLAG2_PR_SHIFT)) == kDllAsyncReqPrioNmt)
        {   // frames in own NMT request queue available
            dllkInstance_g.aLastReqServiceId[curReq_p] = kDllReqServiceNmtRequest;
        }
        else
        {
            dllkInstance_g.aLastReqServiceId[curReq_p] = kDllReqServiceUnspecified;
        }

        ret = dllkcal_getSoaRequest(&dllkInstance_g.aLastReqServiceId[curReq_p],
                                    &dllkInstance_g.aLastTargetNodeId[curReq_p],
                                    &pTxFrame->data.soa.payload);
        if (ret != kErrorOk)
            return ret;

        if (dllkInstance_g.aLastReqServiceId[curReq_p] != kDllReqServiceNo)
        {   // asynchronous phase will be assigned to one node
            if (dllkInstance_g.aLastTargetNodeId[curReq_p] == C_ADR_INVALID)
            {
                UINT            selectTxBuffer;
                tEdrvTxBuffer*  pTxBuffer;

                // exchange invalid node ID with local node ID
                dllkInstance_g.aLastTargetNodeId[curReq_p] = dllkInstance_g.dllConfigParam.nodeId;

                // Select the Tx buffer that should hold the async Tx message
                switch (dllkInstance_g.aLastReqServiceId[curReq_p])
                {
                    case kDllReqServiceStatus:
                        selectTxBuffer = DLLK_TXFRAME_STATUSRES + dllkInstance_g.curTxBufferOffsetStatusRes;
                        pTxBuffer = &dllkInstance_g.pTxBuffer[selectTxBuffer];
                        break;

                    case kDllReqServiceIdent:
                        selectTxBuffer = DLLK_TXFRAME_IDENTRES + dllkInstance_g.curTxBufferOffsetIdentRes;
                        pTxBuffer = &dllkInstance_g.pTxBuffer[selectTxBuffer];
                        break;

                    case kDllReqServiceNmtRequest:
                        selectTxBuffer = DLLK_TXFRAME_NMTREQ + dllkInstance_g.curTxBufferOffsetNmtReq;
                        pTxBuffer = &dllkInstance_g.pTxBuffer[selectTxBuffer];
                        pTxBufferState = &dllkInstance_g.aTxBufferStateNmtReq[dllkInstance_g.curTxBufferOffsetNmtReq];
                        break;

                    case kDllReqServiceUnspecified:
                        selectTxBuffer = DLLK_TXFRAME_NONPLK + dllkInstance_g.curTxBufferOffsetNonPlk;
                        pTxBuffer = &dllkInstance_g.pTxBuffer[selectTxBuffer];
                        pTxBufferState = &dllkInstance_g.aTxBufferStateNonPlk[dllkInstance_g.curTxBufferOffsetNonPlk];
                        break;

                    default:
                        pTxBuffer = NULL;
                        break;
                }

                // If the async Tx buffer is not ready discard the request
                if ((pTxBuffer == NULL) ||
                    ((pTxBufferState != NULL) && (*pTxBufferState != kDllkTxBufReady)))
                {
                    dllkInstance_g.aLastReqServiceId[curReq_p] = kDllReqServiceNo;
                    dllkInstance_g.aLastTargetNodeId[curReq_p] = C_ADR_INVALID;
                    return ret;
                }
            }

            pNodeInfo = dllknode_getNodeInfo(dllkInstance_g.aLastTargetNodeId[curReq_p]);
            if (pNodeInfo == NULL)
            {   // no node info structure available
                ret = kErrorDllNoNodeInfo;
                return ret;
            }

            // Update EA/ER flag only for StatusReq
            if (dllkInstance_g.aLastReqServiceId[curReq_p] == kDllReqServiceStatus)
            {
                ami_setUint8Le(&pTxFrame->data.soa.flag1,
                               pNodeInfo->soaFlag1 & (PLK_FRAME_FLAG1_EA | PLK_FRAME_FLAG1_ER));
            }
            else
            {
                ami_setUint8Le(&pTxFrame->data.soa.flag1, 0);
            }
        }
        else
        {   // no assignment of asynchronous phase
            dllkInstance_g.aLastTargetNodeId[curReq_p] = C_ADR_INVALID;
        }
    }
    else
    {   // invite nobody
        dllkInstance_g.aLastReqServiceId[curReq_p] = kDllReqServiceNo;
        dllkInstance_g.aLastTargetNodeId[curReq_p] = C_ADR_INVALID;
    }

    // update frame (target)
    ami_setUint8Le(&pTxFrame->data.soa.reqServiceId,
                   (UINT8)dllkInstance_g.aLastReqServiceId[curReq_p]);
    ami_setUint8Le(&pTxFrame->data.soa.reqServiceTarget,
                   (UINT8)dllkInstance_g.aLastTargetNodeId[curReq_p]);
    // update frame (NMT state)
    ami_setUint8Le(&pTxFrame->data.soa.nmtStatus, (UINT8)nmtState_p);

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Pass empty ASnd frame to receive FIFO.

The function passes an empty ASnd frame to the receive FIFO. It will be called
only for frames with registered AsndServiceIds (only kDllAsndFilterAny).

\param[in]      reqServiceId_p      Requested service ID.
\param[in]      nodeId_p            Node ID.


\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
tOplkError dllkframe_asyncFrameNotReceived(tDllReqServiceId reqServiceId_p,
                                           UINT nodeId_p)
{
    tOplkError      ret = kErrorOk;
    tDllAsndNotRx   asndNotRx;
    tEvent          event;

    // check if previous SoA invitation was not answered
    switch (reqServiceId_p)
    {
        case kDllReqServiceIdent:
        case kDllReqServiceStatus:
        case kDllReqServiceSync:
            // ASnd service registered?
            if (dllkInstance_g.aAsndFilter[reqServiceId_p] == kDllAsndFilterAny)
            {   // ASnd service ID is registered
                asndNotRx.nodeId = (UINT8)nodeId_p;
                asndNotRx.serviceId = (UINT8)reqServiceId_p;

                event.eventSink = kEventSinkDlluCal;
                event.eventType = kEventTypeAsndNotRx;
                event.eventArg.pEventArg = &asndNotRx;
                event.eventArgSize = sizeof(tDllAsndNotRx);

                // Post event with dummy frame
                ret = eventk_postEvent(&event);
            }
            break;

        default:
            // no invitation issued or it was successfully answered or it is uninteresting
            break;
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  MN timer callback function

This function is called by the timer module. It triggers the SoC for an MN.

\param[in]      pEventArg_p         Pointer to timer event argument.

\return The function returns a pointer to the node Information of the node.
*/
//------------------------------------------------------------------------------
tOplkError dllkframe_cbMnTimerCycle(const tTimerEventArg* pEventArg_p)
{
    tOplkError  ret = kErrorOk;
    tNmtState   nmtState;
    UINT32      arg;

    TGT_DLLK_DECLARE_FLAGS;

    TGT_DLLK_ENTER_CRITICAL_SECTION();

#if (CONFIG_TIMER_USE_HIGHRES != FALSE)
    if (pEventArg_p->timerHdl.handle != dllkInstance_g.timerHdlCycle)
    {   // zombie callback - just exit
        goto Exit;
    }
#endif

    nmtState = dllkInstance_g.nmtState;
    if (nmtState <= kNmtGsResetConfiguration)
        goto Exit;

    ret = dllkstatemachine_changeState(kNmtEventDllMeSocTrig, nmtState);

Exit:
    if (ret != kErrorOk)
    {
        BENCHMARK_MOD_02_TOGGLE(7);
        arg = dllkInstance_g.dllState | (kNmtEventDllMeSocTrig << 8);
        // Error event for API layer
        ret = eventk_postError(kEventSourceDllk, ret, sizeof(arg), &arg);
    }
    TGT_DLLK_LEAVE_CRITICAL_SECTION();

    return ret;
}

#endif

#if (CONFIG_DLL_PRES_CHAINING_CN != FALSE)
//------------------------------------------------------------------------------
/**
\brief  Enable PRes chaining mode

This function enables the PRes chaining mode.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
tOplkError dllkframe_presChainingEnable(void)
{
    tOplkError  ret = kErrorOk;
    tPlkFrame*  pTxFrameSyncRes;

    if (dllkInstance_g.fPrcEnabled == FALSE)
    {
        // relocate PReq filter to PResMN
        ami_setUint48Be(&dllkInstance_g.aFilter[DLLK_FILTER_PREQ].aFilterValue[0],
                        C_DLL_MULTICAST_PRES);
        ami_setUint8Be(&dllkInstance_g.aFilter[DLLK_FILTER_PREQ].aFilterValue[14],
                       kMsgTypePres);
        ami_setUint8Be(&dllkInstance_g.aFilter[DLLK_FILTER_PREQ].aFilterValue[15],
                       C_ADR_BROADCAST); // Set Destination Node ID to C_ADR_BROADCAST

        dllkInstance_g.pTxBuffer[DLLK_TXFRAME_PRES].timeOffsetNs = dllkInstance_g.prcPResTimeFirst;

        ret = edrv_changeRxFilter(dllkInstance_g.aFilter,
                                  DLLK_FILTER_COUNT,
                                  DLLK_FILTER_PREQ,
                                  EDRV_FILTER_CHANGE_VALUE | EDRV_FILTER_CHANGE_AUTO_RESPONSE_DELAY);
        if (ret != kErrorOk)
            return ret;

        dllkInstance_g.fPrcEnabled = TRUE;
        pTxFrameSyncRes = (tPlkFrame*)dllkInstance_g.pTxBuffer[DLLK_TXFRAME_SYNCRES].pBuffer;

        ami_setUint32Le(&pTxFrameSyncRes->data.asnd.payload.syncResponse.syncStatusLe,
                        ami_getUint32Le(&pTxFrameSyncRes->data.asnd.payload.syncResponse.syncStatusLe) |
                            PLK_SYNC_PRES_MODE_SET);
        // update SyncRes Tx buffer in Edrv
        ret = edrv_updateTxBuffer(&dllkInstance_g.pTxBuffer[DLLK_TXFRAME_SYNCRES]);
        if (ret != kErrorOk)
            return ret;

#if (CONFIG_DLL_PROCESS_SYNC == DLL_PROCESS_SYNC_ON_TIMER)
        ret = synctimer_setLossOfSyncTolerance2(dllkInstance_g.prcPResFallBackTimeout);
#endif
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Disable PRes chaining mode

This function disables the PRes chaining mode, thus restoring PReq/PRes mode.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
tOplkError dllkframe_presChainingDisable(void)
{
    tOplkError  ret = kErrorOk;
    tPlkFrame*  pTxFrameSyncRes;

    if (dllkInstance_g.fPrcEnabled != FALSE)
    {   // relocate PReq filter from PResMN to PReq
        OPLK_MEMCPY(&dllkInstance_g.aFilter[DLLK_FILTER_PREQ].aFilterValue[0],
                    edrv_getMacAddr(),
                    6);
        ami_setUint8Be(&dllkInstance_g.aFilter[DLLK_FILTER_PREQ].aFilterValue[14],
                       kMsgTypePreq);
        ami_setUint8Be(&dllkInstance_g.aFilter[DLLK_FILTER_PREQ].aFilterValue[15],
                       dllkInstance_g.dllConfigParam.nodeId); // Set Dst Node ID

        // disable auto-response delay
        dllkInstance_g.pTxBuffer[DLLK_TXFRAME_PRES].timeOffsetNs = 0;
        dllkInstance_g.fPrcEnabled = FALSE;

        ret = edrv_changeRxFilter(dllkInstance_g.aFilter,
                                  DLLK_FILTER_COUNT,
                                  DLLK_FILTER_PREQ,
                                  EDRV_FILTER_CHANGE_VALUE | EDRV_FILTER_CHANGE_AUTO_RESPONSE_DELAY);
        if (ret != kErrorOk)
            return ret;

        pTxFrameSyncRes = (tPlkFrame*)dllkInstance_g.pTxBuffer[DLLK_TXFRAME_SYNCRES].pBuffer;

        ami_setUint32Le(&pTxFrameSyncRes->data.asnd.payload.syncResponse.syncStatusLe,
                        ami_getUint32Le(&pTxFrameSyncRes->data.asnd.payload.syncResponse.syncStatusLe)
                            & ~PLK_SYNC_PRES_MODE_SET);
        // update SyncRes Tx buffer in Edrv
        ret = edrv_updateTxBuffer(&dllkInstance_g.pTxBuffer[DLLK_TXFRAME_SYNCRES]);
        if (ret != kErrorOk)
            return ret;

#if (CONFIG_DLL_PROCESS_SYNC == DLL_PROCESS_SYNC_ON_TIMER)
        ret = synctimer_setLossOfSyncTolerance2(0);
#endif
    }

    return ret;
}

#endif // #if (CONFIG_DLL_PRES_CHAINING_CN != FALSE)


//----------------------------------------------------------------------------//
//                L O C A L   F U N C T I O N S                               //
//----------------------------------------------------------------------------//

//------------------------------------------------------------------------------
/**
\brief  Post invalid frame error event

The function posts an invalid frame error event.

\param[in]      nodeId_p            Node ID of node which sent an invalid frame.
\param[in]      nmtState_p          NMT state of the local node
*/
//------------------------------------------------------------------------------
static void postInvalidFormatError(UINT nodeId_p, tNmtState nmtState_p)
{
    tEventDllError  dllEvent;

    dllEvent.dllErrorEvents = DLL_ERR_INVALID_FORMAT;
    dllEvent.nodeId = nodeId_p;
    dllEvent.nmtState = nmtState_p;

    errhndk_postError(&dllEvent);
}

//------------------------------------------------------------------------------
/**
\brief  Process received PReq frame.

The function processes a received PReq frame.

\param[in]      pFrameInfo_p        Pointer to frame information.
\param[in]      nmtState_p          NMT state of the local node.
\param[out]     pReleaseRxBuffer_p  Pointer to buffer release flag. The function must
                                    set this flag to determine if the RxBuffer could be
                                    released immediately.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError processReceivedPreq(tFrameInfo* pFrameInfo_p,
                                      tNmtState nmtState_p,
                                      tEdrvReleaseRxBuffer* pReleaseRxBuffer_p)
{
    tOplkError  ret = kErrorOk;
    tPlkFrame*  pFrame;
    UINT8       flag1;

    pFrame = pFrameInfo_p->frame.pBuffer;

    if (!NMT_IF_ACTIVE_CN(nmtState_p))
    {   // MN is active -> wrong msg type
        goto Exit;
    }

    if (nmtState_p >= kNmtCsPreOperational2)
    {   // respond to and process PReq frames only in PreOp2, ReadyToOp and Op

#if (CONFIG_EDRV_AUTO_RESPONSE == FALSE)
        tEdrvTxBuffer*  pTxBuffer = NULL;

        // Auto-response is disabled
        // Does PRes exist?
        pTxBuffer = &dllkInstance_g.pTxBuffer[DLLK_TXFRAME_PRES + dllkInstance_g.curTxBufferOffsetCycle];
        if (pTxBuffer->pBuffer != NULL)
        {   // PRes does exist -> send PRes frame
            ret = edrv_sendTxBuffer(pTxBuffer);
            if (ret != kErrorOk)
                goto Exit;
        }
#endif

        // update only EA and MS flag
        flag1 = ami_getUint8Le(&pFrame->data.preq.flag1);

        dllkInstance_g.mnFlag1 = (dllkInstance_g.mnFlag1 &
                                      ~(PLK_FRAME_FLAG1_EA | PLK_FRAME_FLAG1_MS)) |             // preserve all flags except EA and MS
                                      (flag1 & (PLK_FRAME_FLAG1_EA | PLK_FRAME_FLAG1_MS));      // set EA and MS flag

        // inform PDO module
#if defined(CONFIG_INCLUDE_PDO)
        if (nmtState_p >= kNmtCsReadyToOperate)
        {   // inform PDO module only in ReadyToOp and Op
            UINT16 preqPayloadSize = ami_getUint16Le(&pFrame->data.preq.sizeLe);

            if (nmtState_p != kNmtCsOperational)
            {
                // reset RD flag and all other flags, but that does not matter, because they were processed above
                ami_setUint8Le(&pFrame->data.preq.flag1, 0);
            }

            // compares real frame size and PDO size
            if (((UINT)(preqPayloadSize + PLK_FRAME_OFFSET_PDO_PAYLOAD) > pFrameInfo_p->frameSize) ||
                (preqPayloadSize > dllkInstance_g.dllConfigParam.preqActPayloadLimit))
            {   // format error
                postInvalidFormatError(ami_getUint8Le(&pFrame->srcNodeId), nmtState_p);
                goto Exit;
            }

            // forward PReq frame as RPDO to PDO module
            ret = forwardRpdo(pFrameInfo_p);
            if (ret == kErrorReject)
            {
                *pReleaseRxBuffer_p = kEdrvReleaseRxBufferLater;
                ret = kErrorOk;
            }
            else if (ret != kErrorOk)
            {
                goto Exit;
            }
        }
#endif

        // $$$ inform emergency protocol handling (error signaling module) about flags
    }

    // reset cycle counter
    dllkInstance_g.cycleCount = 0;

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Check for an invalid PRes frame

The function checks if a PRes frame is invalid.

\param[in]      pFrameInfo_p        Pointer to frame information.
\param[in]      pIntNodeInfo_p      Pointer to node information.
\param[in]      nodeNmtState_p      NMT state of the node which sent the frame.

\return The function returns TRUE if the frame is invalid and FALSE if it is
        valid.
*/
//------------------------------------------------------------------------------
static BOOL presFrameFormatIsInvalid(const tFrameInfo* pFrameInfo_p,
                                     const tDllkNodeInfo* pIntNodeInfo_p,
                                     tNmtState nodeNmtState_p)
{
    const tPlkFrame*    pFrame = pFrameInfo_p->frame.pBuffer;
    size_t              frameSize = pFrameInfo_p->frameSize;
    size_t              payloadSize = ami_getUint16Le(&pFrame->data.pres.sizeLe);

#if (NMT_MAX_NODE_ID > 0)
    size_t              payloadLimit = pIntNodeInfo_p->presPayloadLimit;
#else
    UNUSED_PARAMETER(pIntNodeInfo_p);
    UNUSED_PARAMETER(nodeNmtState_p);
#endif

    // Check if frame is too small for contained payload size
    if ((payloadSize + PLK_FRAME_OFFSET_PDO_PAYLOAD) > frameSize)
        return TRUE;

    // Additional checks for MN and CN which can handle cross-traffic
#if (NMT_MAX_NODE_ID > 0)
    if ((nodeNmtState_p & 0xFF) > (kNmtCsPreOperational2 & 0xFF))
    {
        if ((payloadSize > payloadLimit) ||                                     // payload limit is exceeded
            (frameSize > (payloadLimit + PLK_FRAME_OFFSET_PDO_PAYLOAD)))        // frame is too big for payload limit
            return TRUE;
    }
#endif

    return FALSE;
}

//------------------------------------------------------------------------------
/**
\brief  Process received PRes frame

The function processes a received PRes frame.

\param[in]      pFrameInfo_p        Pointer to frame information.
\param[in]      nmtState_p          NMT state of the local node.
\param[out]     pNmtEvent_p         Pointer to store NMT event.
\param[out]     pReleaseRxBuffer_p  Pointer to buffer release flag. The function must
                                    set this flag to determine if the RxBuffer could be
                                    released immediately.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError processReceivedPres(const tFrameInfo* pFrameInfo_p,
                                      tNmtState nmtState_p,
                                      tNmtEvent* pNmtEvent_p,
                                      tEdrvReleaseRxBuffer* pReleaseRxBuffer_p)
{
    tOplkError          ret = kErrorOk;
    const tPlkFrame*    pFrame;
    UINT                nodeId;
    tDllkNodeInfo*      pIntNodeInfo = NULL;
    tNmtState           nodeNmtState;

#if (defined(CONFIG_INCLUDE_NMT_MN) && defined(CONFIG_INCLUDE_PRES_FORWARD))
    tDllkPresFw*        pPresFw;
#endif

    pFrame = pFrameInfo_p->frame.pBuffer;
    nodeId = ami_getUint8Le(&pFrame->srcNodeId);

#if (defined(CONFIG_INCLUDE_NMT_MN) && defined(CONFIG_INCLUDE_PRES_FORWARD))
    // Check if PRes frame should be forwarded to API layer

    pPresFw = &dllkInstance_g.aPresForward[nodeId - 1];
    if (pPresFw->numRequests != pPresFw->numResponse)
    {
        tEvent                  event;
        tDllEventReceivedPres   presEvent;

        OPLK_MEMSET(&presEvent, 0x00, sizeof(presEvent));

        presEvent.nodeId = (UINT16)nodeId;
        presEvent.frameSize = (UINT16)pFrameInfo_p->frameSize;

        // If Presp frames are received which are larger than the buffer, they are cut off
        // (the application will most probably just be interested in the frame-header anyway).
        OPLK_MEMCPY(&presEvent.frameBuf,
                    pFrame,
                    min(sizeof(presEvent.frameBuf), (size_t)pFrameInfo_p->frameSize));

        event.eventSink = kEventSinkApi;
        event.eventType = kEventTypeReceivedPres;
        event.eventArgSize = sizeof(presEvent);
        event.eventArg.pEventArg = &presEvent;

        ret = eventk_postEvent(&event);
        pPresFw->numResponse++;
    }
#endif

    if (nodeId == C_ADR_MN_DEF_NODE_ID)
        nodeNmtState = (tNmtState)(ami_getUint8Le(&pFrame->data.pres.nmtStatus) | NMT_TYPE_MS);
    else
        nodeNmtState = (tNmtState)(ami_getUint8Le(&pFrame->data.pres.nmtStatus) | NMT_TYPE_CS);

#if (CONFIG_DLL_PRES_CHAINING_CN != FALSE)
    // handle PResMN as PReq for PRes Chaining
    if ((dllkInstance_g.fPrcEnabled != FALSE) && (nodeId == C_ADR_MN_DEF_NODE_ID))
        *pNmtEvent_p = kNmtEventDllCePreq;      // handle PResMN as PReq for PRes Chaining
    else
        *pNmtEvent_p = kNmtEventDllCePres;
#else
    *pNmtEvent_p = kNmtEventDllCePres;
#endif

    if ((nmtState_p >= kNmtCsPreOperational2) && (nmtState_p <= kNmtCsOperational))
    {   // process PRes frames only in PreOp2, ReadyToOp and Op of CN
#if (NMT_MAX_NODE_ID > 0)
        pIntNodeInfo = dllknode_getNodeInfo(nodeId);
        if (pIntNodeInfo == NULL)
        {   // no node info structure available
            return kErrorDllNoNodeInfo;
        }
#if defined(CONFIG_INCLUDE_NMT_RMN)
        if (dllkInstance_g.fRedundancy)
        {
            if ((ret = updateNode(pIntNodeInfo, nodeId, nodeNmtState, pFrame->aSrcMac)) != kErrorOk)
                return ret;
        }
#endif
#endif
    }
    else
#if defined(CONFIG_INCLUDE_NMT_MN)
    {
        if (dllkInstance_g.dllState > kDllMsNonCyclic)
        {   // or process PRes frames in MsWaitPres
            UINT8                   flag2;
            BOOL                    fPrcSlotFinished = FALSE;
            tDllAsyncReqPriority    asyncReqPrio;

            ret = searchNodeInfo(nodeId, &pIntNodeInfo, &fPrcSlotFinished);
            if (ret != kErrorOk)
                return ret;

            if (pIntNodeInfo == NULL)
            {   // ignore PRes, because it is from wrong CN
                *pNmtEvent_p = kNmtEventNoEvent;
                return ret;
            }

            ret = checkAndSetSyncEvent(fPrcSlotFinished, nodeId);
            if (ret != kErrorOk)
                return ret;

            // forward Flag2 to asynchronous scheduler
            flag2 = ami_getUint8Le(&pFrame->data.asnd.payload.statusResponse.flag2);
            asyncReqPrio = (tDllAsyncReqPriority)((flag2 & PLK_FRAME_FLAG2_PR) >> PLK_FRAME_FLAG2_PR_SHIFT);
            ret = dllkcal_setAsyncPendingRequests(nodeId, asyncReqPrio, flag2 & PLK_FRAME_FLAG2_RS);
            if (ret != kErrorOk)
                return ret;

            ret = updateNode(pIntNodeInfo, nodeId, nodeNmtState, NULL);
            if (ret != kErrorOk)
                return ret;
        }
        else
        {   // ignore PRes, because it was received in wrong NMT state
            // but execute changeState() and post event to NMT module
            return ret;
        }
    }
#else
    {   // ignore PRes, because it was received in wrong NMT state
        // but execute changeState() and post event to NMT module
        return ret;
    }
#endif

#if defined(CONFIG_INCLUDE_PDO)
    // At this point we know that we are in a cyclic state due to the checks above!
    if ((nmtState_p != kNmtCsPreOperational2) && (nmtState_p != kNmtMsPreOperational2))
    {
#if (NMT_MAX_NODE_ID > 0)
        // This check can be skipped when no cross traffic is supported
        if (pIntNodeInfo == NULL)
        {
            ret = kErrorDllNoNodeInfo;
            return ret;
        }
#endif
        // So we are in ReadyToOp or Operational after the check and can inform the PDO module now.
        if (presFrameFormatIsInvalid(pFrameInfo_p, pIntNodeInfo, nodeNmtState))
        {
#if (NMT_MAX_NODE_ID > 0)
            // Check if the configuration defines the received node as irrelevant
            if (pIntNodeInfo->presPayloadLimit > 0)
#endif
            {
                postInvalidFormatError(nodeId, nmtState_p);
            }

            return ret;
        }

        ret = forwardRpdo(pFrameInfo_p);
        if (ret != kErrorOk)
        {
            if (ret == kErrorReject)
            {
                *pReleaseRxBuffer_p = kEdrvReleaseRxBufferLater;
                ret = kErrorOk;
            }
            else
            {
                return ret;
            }
        }
    }
#endif

#if defined(CONFIG_INCLUDE_NMT_MN)
    // check if Sync event needs to be triggered
    if ((dllkInstance_g.dllState > kDllMsNonCyclic) &&
        (dllkInstance_g.dllConfigParam.syncNodeId > C_ADR_SYNC_ON_SOC) &&
        (dllkInstance_g.fSyncProcessed == FALSE))
    {
        if ((dllkInstance_g.dllConfigParam.fSyncOnPrcNode != dllkInstance_g.fPrcSlotFinished) &&
            (nodeId == dllkInstance_g.dllConfigParam.syncNodeId))
        {
            dllkInstance_g.fSyncProcessed = TRUE;
            ret = dllk_postEvent(kEventTypeSync);
        }
    }
#endif

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Process received SoC frame

The function processes a received SoC frame.

\param[in]      pRxBuffer_p         Pointer to RxBuffer structure of received frame.
\param[in]      nmtState_p          NMT state of the local node.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError processReceivedSoc(const tEdrvRxBuffer* pRxBuffer_p,
                                     tNmtState nmtState_p)
{
    tOplkError          ret = kErrorOk;
    const tPlkFrame*    pFrame;
    UINT64              relTime;

#if (CONFIG_DLL_PROCESS_SYNC != DLL_PROCESS_SYNC_ON_TIMER)
    UNUSED_PARAMETER(pRxBuffer_p);
#endif

    if (!NMT_IF_ACTIVE_CN(nmtState_p))
    {   // MN is active -> wrong msg type
        return ret;
    }

    pFrame = (const tPlkFrame*)pRxBuffer_p->pBuffer;

    if (nmtState_p >= kNmtCsStopped)
    {   // SoC frames only in Stopped, PreOp2, ReadyToOp and Operational

#if (CONFIG_DLL_PROCESS_SYNC == DLL_PROCESS_SYNC_ON_SOC)
        // trigger synchronous task
        ret = dllk_postEvent(kEventTypeSync);
        if (ret != kErrorOk)
            return ret;
#elif (CONFIG_DLL_PROCESS_SYNC == DLL_PROCESS_SYNC_ON_TIMER)
        ret = synctimer_syncTriggerAtTimeStamp(pRxBuffer_p->pRxTimeStamp);
        if (ret != kErrorOk)
            return ret;
#endif

        // update cycle counter
        if (dllkInstance_g.dllConfigParam.multipleCycleCnt > 0)
        {   // multiplexed cycle active
            dllkInstance_g.cycleCount = (dllkInstance_g.cycleCount + 1) %
                                            dllkInstance_g.dllConfigParam.multipleCycleCnt;
        }

        // Get SoC time stamps
        relTime = ami_getUint64Le(&pFrame->data.soc.relativeTimeLe);
        dllkInstance_g.socTime.netTime.nsec = ami_getUint32Le(&pFrame->data.soc.netTimeLe.nsec);
        dllkInstance_g.socTime.netTime.sec = ami_getUint32Le(&pFrame->data.soc.netTimeLe.sec);

        // Validate locally stored SoC time
        if (!dllkInstance_g.socTime.fRelTimeValid)
        {
            // From the first change in the SoC time stamp it is considered valid
            if (dllkInstance_g.socTime.relTime != relTime)
                dllkInstance_g.socTime.fRelTimeValid = TRUE;
        }

        dllkInstance_g.socTime.relTime = relTime;

#if defined(CONFIG_INCLUDE_SOC_TIME_FORWARD)
        // Forward Soc time stamp to timesync modules
        ret = timesynck_setSocTime(&dllkInstance_g.socTime);
        if (ret != kErrorOk)
            return ret;
#endif
    }

    // reprogram timer
#if (CONFIG_TIMER_USE_HIGHRES != FALSE)
#if defined(CONFIG_INCLUDE_NMT_RMN)
    if (dllkInstance_g.fRedundancy)
    {
        if (nmtState_p == kNmtCsOperational)
        {
            hrestimer_modifyTimer(&dllkInstance_g.timerHdlSwitchOver,
                                  dllkInstance_g.dllConfigParam.switchOverTimeMn * 1000ULL,
                                  dllk_cbTimerSwitchOver, 0L, FALSE);
        }
        else
        {
            hrestimer_modifyTimer(&dllkInstance_g.timerHdlSwitchOver,
                                  dllkInstance_g.dllConfigParam.delayedSwitchOverTimeMn * 1000ULL,
                                  dllk_cbTimerSwitchOver, 0L, FALSE);
        }

        ret = edrvcyclic_startCycle(FALSE);
        if (ret != kErrorOk)
            return ret;
    }
    else
#endif
    if (dllkInstance_g.frameTimeout != 0)
    {
        hrestimer_modifyTimer(&dllkInstance_g.timerHdlCycle,
                              dllkInstance_g.frameTimeout,
                              cbCnTimer,
                              0L,
                              FALSE);
    }
#endif
    return ret;
}

#if defined(CONFIG_INCLUDE_NMT_RMN)
//------------------------------------------------------------------------------
/**
\brief  Process received AMNI frame

The function processes a received AMNI frame.

\param[in]      pRxBuffer_p         Pointer to RxBuffer structure of received frame.
\param[in]      nmtState_p          NMT state of the local node.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError processReceivedAmni(const tEdrvRxBuffer* pRxBuffer_p,
                                      tNmtState nmtState_p)
{
    tOplkError          ret = kErrorOk;
    const tPlkFrame*    pFrame;
    tEvent              event;
    UINT                nodeId;

    pFrame = (const tPlkFrame*)pRxBuffer_p->pBuffer;
    nodeId = ami_getUint8Le(&pFrame->srcNodeId);

    event.eventSink = kEventSinkNmtMnu;
    event.eventType = kEventTypeReceivedAmni;
    event.eventArgSize = sizeof(nodeId);
    event.eventArg.pEventArg = &nodeId;
    ret = eventk_postEvent(&event);

    if (!NMT_IF_ACTIVE_CN(nmtState_p))
    {   // not a Standby Managing Node
        return ret;
    }

    // reprogram timer
    if (dllkInstance_g.fRedundancy)
    {
        ULONGLONG   switchOverTime;

        switch (nmtState_p)
        {
            case kNmtCsPreOperational1:
                switchOverTime = dllkInstance_g.dllConfigParam.reducedSwitchOverTimeMn * 1000ULL;
                break;

            case kNmtCsOperational:
                switchOverTime = dllkInstance_g.dllConfigParam.switchOverTimeMn * 1000ULL;
                break;

            default:
                switchOverTime = dllkInstance_g.dllConfigParam.delayedSwitchOverTimeMn * 1000ULL;
                break;
        }

        hrestimer_modifyTimer(&dllkInstance_g.timerHdlSwitchOver,
                              switchOverTime,
                              dllk_cbTimerSwitchOver,
                              0L,
                              FALSE);
    }

    return ret;
}
#endif

//------------------------------------------------------------------------------
/**
\brief  Process received SoA frame

The function processes a received SoA frame.

\param[in]      pRxBuffer_p         Pointer to RxBuffer structure of received frame.
\param[in]      nmtState_p          NMT state of the local node.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError processReceivedSoa(const tEdrvRxBuffer* pRxBuffer_p,
                                     tNmtState nmtState_p)
{
    tOplkError          ret = kErrorOk;
    const tPlkFrame*    pFrame;
#if (CONFIG_EDRV_AUTO_RESPONSE == FALSE)
    tEdrvTxBuffer*      pTxBuffer = NULL;
    tDllkTxBufState*    pTxBufferState = NULL;
#endif
    tDllReqServiceId    reqServiceId;
    UINT8               nodeId;
    UINT8               flag1;

    pFrame = (const tPlkFrame*)pRxBuffer_p->pBuffer;

    if (!NMT_IF_ACTIVE_CN(nmtState_p))
    {   // do not respond,
        // if NMT state is < PreOp1 (i.e. not in a POWERLINK mode)
        // or node is MN
        goto Exit;
    }

    // check TargetNodeId
    nodeId = ami_getUint8Le(&pFrame->data.soa.reqServiceTarget);
    if (nodeId == dllkInstance_g.dllConfigParam.nodeId)
    {   // local node is the target of the current request

        // check ServiceId
        reqServiceId = (tDllReqServiceId)ami_getUint8Le(&pFrame->data.soa.reqServiceId);
        switch (reqServiceId)
        {
            case kDllReqServiceStatus:
                // StatusRequest
#if (CONFIG_EDRV_AUTO_RESPONSE == FALSE)
                // Auto-response is not available
                pTxBuffer = &dllkInstance_g.pTxBuffer[DLLK_TXFRAME_STATUSRES + dllkInstance_g.curTxBufferOffsetStatusRes];
                if (pTxBuffer->pBuffer != NULL)
                {   // StatusRes does exist
                    // send StatusRes
                    ret = edrv_sendTxBuffer(pTxBuffer);
                    if (ret != kErrorOk)
                        goto Exit;

                    TGT_DBG_SIGNAL_TRACE_POINT(8);
                }
                else
                {   // no frame transmitted
                    pTxBuffer = NULL;
                }
#endif

                // update error signaling
                flag1 = ami_getUint8Le(&pFrame->data.soa.flag1);
                if (((flag1 ^ dllkInstance_g.mnFlag1) & PLK_FRAME_FLAG1_ER) != 0)
                {   // exception reset flag was changed by MN
                    // assume same state for EC in next cycle (clear all other bits)
                    if ((flag1 & PLK_FRAME_FLAG1_ER) != 0)
                    {
                        // set EC and reset rest
                        dllkInstance_g.flag1 = PLK_FRAME_FLAG1_EC;
                    }
                    else
                    {
                        // reset entire flag 1 (including EC and EN)
                        dllkInstance_g.flag1 = 0;
                    }

                    // signal update of StatusRes
                    ret = dllk_postEvent(kEventTypeDllkFlag1);
                    if (ret != kErrorOk)
                        goto Exit;

                }
                // update (only) EA and ER flag from MN for Status request response cycle
                // $$$ d.k. only in PreOp1 and when async-only or not accessed isochronously
                dllkInstance_g.mnFlag1 =
                    (dllkInstance_g.mnFlag1 & ~(PLK_FRAME_FLAG1_EA | PLK_FRAME_FLAG1_ER)) | // preserve all flags except EA and ER
                    (flag1 & (PLK_FRAME_FLAG1_EA | PLK_FRAME_FLAG1_ER));                    // set EA and ER flag
                goto Exit;

            case kDllReqServiceIdent:
               // IdentRequest
#if (CONFIG_EDRV_AUTO_RESPONSE == FALSE)
                // Auto-response is not available
                pTxBuffer = &dllkInstance_g.pTxBuffer[DLLK_TXFRAME_IDENTRES + dllkInstance_g.curTxBufferOffsetIdentRes];
                if (pTxBuffer->pBuffer != NULL)
                {   // IdentRes does exist
                    // send IdentRes
                    ret = edrv_sendTxBuffer(pTxBuffer);
                    if (ret != kErrorOk)
                        goto Exit;

                    TGT_DBG_SIGNAL_TRACE_POINT(7);
                }
                else
                {   // no frame transmitted
                    pTxBuffer = NULL;
                }
#endif
                goto Exit;

            case kDllReqServiceNmtRequest:
                // NmtRequest
#if (CONFIG_EDRV_AUTO_RESPONSE == FALSE)
                // Auto-response is not available
                pTxBuffer = &dllkInstance_g.pTxBuffer[DLLK_TXFRAME_NMTREQ + dllkInstance_g.curTxBufferOffsetNmtReq];
                pTxBufferState = &dllkInstance_g.aTxBufferStateNmtReq[dllkInstance_g.curTxBufferOffsetNmtReq];
                if (pTxBuffer->pBuffer != NULL)
                {   // NmtRequest does exist
                    // check if frame is not empty and not being filled
                    if (*pTxBufferState == kDllkTxBufReady)
                    {
                        // send NmtRequest
                        *pTxBufferState = kDllkTxBufSending;
                        ret = edrv_sendTxBuffer(pTxBuffer);
                        if (ret != kErrorOk)
                            goto Exit;

                        // decrement RS in Flag 2
                        // The real update will be done later on event FillTx,
                        // but for now it assures that a quite good value gets via the SoA event into the next PRes.
                        if ((dllkInstance_g.flag2 & PLK_FRAME_FLAG2_RS) != 0)
                            dllkInstance_g.flag2--;
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

#if ((CONFIG_DLL_PRES_CHAINING_CN != FALSE) || defined(CONFIG_INCLUDE_NMT_MN))
            case kDllReqServiceSync:
            {
                // SyncRequest
#if (CONFIG_DLL_PRES_CHAINING_CN != FALSE)
                UINT32              syncControl;
                tPlkFrame*          pTxFrameSyncRes;
                tDllkPrcCycleTiming prcCycleTiming;

                pTxFrameSyncRes = (tPlkFrame*)dllkInstance_g.pTxBuffer[DLLK_TXFRAME_SYNCRES].pBuffer;
                syncControl = ami_getUint32Le(&pFrame->data.soa.payload.syncRequest.syncControlLe);
                if (syncControl & PLK_SYNC_DEST_MAC_ADDRESS_VALID)
                {
                    if (OPLK_MEMCMP(&pFrame->data.soa.payload.syncRequest.aDestMacAddress,
                                    edrv_getMacAddr(), 6) != 0)
                    {   // DestMacAddress valid but unequal to own MAC address -> SyncReq is ignored
                        goto Exit;
                    }
                }

                prcCycleTiming.pResTimeFirstNs = ami_getUint32Le(&pFrame->data.soa.payload.syncRequest.presTimeFirstLe);

                if (syncControl & PLK_SYNC_PRES_TIME_FIRST_VALID)
                {
                    dllkInstance_g.prcPResTimeFirst = prcCycleTiming.pResTimeFirstNs;

                    dllkInstance_g.pTxBuffer[DLLK_TXFRAME_PRES].timeOffsetNs = prcCycleTiming.pResTimeFirstNs;
                    ret = edrv_changeRxFilter(dllkInstance_g.aFilter,
                                              DLLK_FILTER_COUNT,
                                              DLLK_FILTER_PREQ,
                                              EDRV_FILTER_CHANGE_AUTO_RESPONSE_DELAY);
                    if (ret != kErrorOk)
                        goto Exit;

                    ami_setUint32Le(&pTxFrameSyncRes->data.asnd.payload.syncResponse.presTimeFirstLe,
                                    prcCycleTiming.pResTimeFirstNs);
                    ami_setUint32Le(&pTxFrameSyncRes->data.asnd.payload.syncResponse.syncStatusLe,
                                    ami_getUint32Le(&pTxFrameSyncRes->data.asnd.payload.syncResponse.syncStatusLe) |
                                        PLK_SYNC_PRES_TIME_FIRST_VALID);
                    // update SyncRes Tx buffer in Edrv
                    ret = edrv_updateTxBuffer(&dllkInstance_g.pTxBuffer[DLLK_TXFRAME_SYNCRES]);
                    if (ret != kErrorOk)
                        goto Exit;
                }

                if (syncControl & PLK_SYNC_PRES_FALL_BACK_TIMEOUT_VALID)
                {
                    dllkInstance_g.prcPResFallBackTimeout = ami_getUint32Le(&pFrame->data.soa.payload.syncRequest.presFallBackTimeoutLe);

#if (CONFIG_DLL_PROCESS_SYNC == DLL_PROCESS_SYNC_ON_TIMER)
                    if (dllkInstance_g.fPrcEnabled != FALSE)
                        ret = synctimer_setLossOfSyncTolerance2(dllkInstance_g.prcPResFallBackTimeout);
#endif
                }

                if (syncControl & PLK_SYNC_PRES_MODE_RESET)
                {
                    // PResModeReset overrules PResModeSet
                    syncControl &= ~PLK_SYNC_PRES_MODE_SET;

                    ret = dllkframe_presChainingDisable();
                    if (ret != kErrorOk)
                        goto Exit;
                }
                else if (syncControl & PLK_SYNC_PRES_MODE_SET)
                {   // PRes Chaining is Enabled
                    ret = dllkframe_presChainingEnable();
                    if (ret != kErrorOk)
                        goto Exit;
                }

                prcCycleTiming.syncControl = syncControl & (PLK_SYNC_PRES_TIME_FIRST_VALID |
                                                            PLK_SYNC_PRES_TIME_SECOND_VALID |
                                                            PLK_SYNC_SYNC_MN_DELAY_FIRST_VALID |
                                                            PLK_SYNC_SYNC_MN_DELAY_SECOND_VALID |
                                                            PLK_SYNC_PRES_MODE_RESET |
                                                            PLK_SYNC_PRES_MODE_SET);

                if (prcCycleTiming.syncControl != 0)
                {
                    prcCycleTiming.pResTimeSecondNs = ami_getUint32Le(&pFrame->data.soa.payload.syncRequest.presTimeSecondLe);
                    prcCycleTiming.syncMNDelayFirstNs = ami_getUint32Le(&pFrame->data.soa.payload.syncRequest.syncMnDelayFirstLe);
                    prcCycleTiming.syncMNDelaySecondNs = ami_getUint32Le(&pFrame->data.soa.payload.syncRequest.syncMnDelaySecondLe);
                    // $$$ m.u.: CbUpdatePrcCycleTiming
                }
#endif
                goto Exit;
            }
#endif

            case kDllReqServiceUnspecified:
                // unspecified invite
#if (CONFIG_EDRV_AUTO_RESPONSE == FALSE)
                // Auto-response is not available
                pTxBuffer = &dllkInstance_g.pTxBuffer[DLLK_TXFRAME_NONPLK + dllkInstance_g.curTxBufferOffsetNonPlk];
                pTxBufferState = &dllkInstance_g.aTxBufferStateNonPlk[dllkInstance_g.curTxBufferOffsetNonPlk];
                if (pTxBuffer->pBuffer != NULL)
                {   // non-POWERLINK frame does exist
                    // check if frame is not empty and not being filled
                    if (*pTxBufferState == kDllkTxBufReady)
                    {
                        // send non-POWERLINK frame
                        *pTxBufferState = kDllkTxBufSending;
                        ret = edrv_sendTxBuffer(pTxBuffer);
                        if (ret != kErrorOk)
                            goto Exit;

                        // decrement RS in Flag 2
                        // The real update will be done later on event FillTx,
                        // but for now it assures that a quite good value gets via the SoA event into the next PRes.
                        if ((dllkInstance_g.flag2 & PLK_FRAME_FLAG2_RS) != 0)
                            dllkInstance_g.flag2--;
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
        }
    }

#if (CONFIG_DLL_PRES_CHAINING_CN != FALSE)
    else
    {   // other node is the target of the current request
        // check ServiceId
        reqServiceId = (tDllReqServiceId)ami_getUint8Le(&pFrame->data.soa.reqServiceId);
        if (reqServiceId == kDllReqServiceSync)
        {   // SyncRequest -> store node ID and TimeStamp
            dllkInstance_g.syncReqPrevNodeId = nodeId;
            dllkInstance_g.syncReqPrevTimeStamp = *pRxBuffer_p->pRxTimeStamp;
        }
    }
#endif

    // $$$ put SrcNodeId, NMT state and NetTime as HeartbeatEvent into eventqueue
    // $$$ inform emergency protocol handling about flags

#if defined(CONFIG_INCLUDE_NMT_RMN)
    if ((dllkInstance_g.fRedundancy) &&
        (nmtState_p == kNmtCsPreOperational1))
    {
        hrestimer_modifyTimer(&dllkInstance_g.timerHdlSwitchOver,
                              dllkInstance_g.dllConfigParam.reducedSwitchOverTimeMn * 1000ULL,
                              dllk_cbTimerSwitchOver,
                              0L,
                              FALSE);
    }
#endif

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Process received ASnd frame

The function processes a received ASnd frame.

\param[in]      pFrameInfo_p        Pointer to frame information.
\param[in]      pRxBuffer_p         Pointer to RxBuffer structure of received frame.
\param[in]      nmtState_p          NMT state of the local node.
\param[out]     pReleaseRxBuffer_p  Pointer to buffer release flag. The function must
                                    set this flag to determine if the RxBuffer could be
                                    released immediately.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError processReceivedAsnd(tFrameInfo* pFrameInfo_p,
                                      const tEdrvRxBuffer* pRxBuffer_p,
                                      tNmtState nmtState_p,
                                      tEdrvReleaseRxBuffer* pReleaseRxBuffer_p)
{
    tOplkError          ret = kErrorOk;
    const tPlkFrame*    pFrame;
    tDllAsndServiceId   asndServiceId;
    UINT                nodeId;

#if defined(CONFIG_INCLUDE_NMT_MN)
    UINT8               flag1;
#endif

    UNUSED_PARAMETER(nmtState_p);
#if (CONFIG_DLL_PRES_CHAINING_CN == FALSE)
    UNUSED_PARAMETER(pRxBuffer_p);
#endif

    pFrame = pFrameInfo_p->frame.pBuffer;

    // ASnd service registered?
    asndServiceId = (tDllAsndServiceId)ami_getUint8Le(&pFrame->data.asnd.serviceId);

#if defined(CONFIG_INCLUDE_NMT_MN)
    if (dllkInstance_g.dllState >= kDllMsNonCyclic)
    {
        switch (asndServiceId)
        {
            case kDllAsndStatusResponse:
            case kDllAsndIdentResponse:
            case kDllAsndSyncResponse:
                nodeId = ami_getUint8Le(&pFrame->srcNodeId);
                if ((dllkInstance_g.aLastReqServiceId[dllkInstance_g.curLastSoaReq] == asndServiceId) &&
                    (nodeId == dllkInstance_g.aLastTargetNodeId[dllkInstance_g.curLastSoaReq]))
                {   // mark request as responded
                    dllkInstance_g.aLastReqServiceId[dllkInstance_g.curLastSoaReq] = kDllReqServiceNo;
                }

                if (asndServiceId == kDllAsndIdentResponse)
                {   // save MAC address of CN for PReq
                    tDllkNodeInfo*   pIntNodeInfo;

                    pIntNodeInfo = dllknode_getNodeInfo(nodeId);
                    if (pIntNodeInfo == NULL)
                    {   // no node info structure available
                        ret = kErrorDllNoNodeInfo;
                        goto Exit;
                    }
                    else
                    {
                        OPLK_MEMCPY(pIntNodeInfo->aMacAddr, pFrame->aSrcMac, 6);
                    }
                }
                if (asndServiceId == kDllAsndStatusResponse)
                {
                    handleErrorSignaling(pFrame, nodeId);
                }
                else if (asndServiceId == kDllAsndSyncResponse)
                {
                    break;
                }

                // forward Flag2 to asynchronous scheduler
                flag1 = ami_getUint8Le(&pFrame->data.asnd.payload.statusResponse.flag2);
                ret = dllkcal_setAsyncPendingRequests(
                          nodeId,
                          ((tDllAsyncReqPriority)((flag1 & PLK_FRAME_FLAG2_PR) >> PLK_FRAME_FLAG2_PR_SHIFT)),
                          (flag1 & PLK_FRAME_FLAG2_RS));
                if (ret != kErrorOk)
                    goto Exit;
                break;

            default:
                break;
        }
    }
#endif

    if ((UINT)asndServiceId < DLL_MAX_ASND_SERVICE_ID)
    {   // ASnd service ID is valid

#if (CONFIG_DLL_PRES_CHAINING_CN != FALSE)
        if (asndServiceId == kDllAsndSyncResponse)
        {
            tPlkFrame*  pTxFrameSyncRes;

            pTxFrameSyncRes = (tPlkFrame*)dllkInstance_g.pTxBuffer[DLLK_TXFRAME_SYNCRES].pBuffer;
            nodeId = (UINT)ami_getUint8Le(&pFrame->srcNodeId);

            if (nodeId == dllkInstance_g.syncReqPrevNodeId)
            {
                UINT32  syncDelayNs;

                syncDelayNs = timestamp_calcTimeDiff(&dllkInstance_g.syncReqPrevTimeStamp,
                                                     pRxBuffer_p->pRxTimeStamp) -
                                  // Transmission time for SyncReq frame
                                  (C_DLL_T_MIN_FRAME + C_DLL_T_PREAMBLE);

                // update SyncRes frame (SyncDelay and SyncNodeNumber)
                ami_setUint32Le(&pTxFrameSyncRes->data.asnd.payload.syncResponse.syncDelayLe, syncDelayNs);
                ami_setUint32Le(&pTxFrameSyncRes->data.asnd.payload.syncResponse.syncNodeNumberLe, (UINT32)nodeId);
                // $$$ m.u.: CbUpdateRelativeLatencyDiff
            }
            else
            {
                ami_setUint32Le(&pTxFrameSyncRes->data.asnd.payload.syncResponse.syncDelayLe, 0);
                ami_setUint32Le(&pTxFrameSyncRes->data.asnd.payload.syncResponse.syncNodeNumberLe, 0);
            }

            // update Tx buffer in Edrv
            ret = edrv_updateTxBuffer(&dllkInstance_g.pTxBuffer[DLLK_TXFRAME_SYNCRES]);
            if (ret != kErrorOk)
                goto Exit;

            // reset stored node ID
            dllkInstance_g.syncReqPrevNodeId = 0;
        }
#endif
        nodeId = ami_getUint8Le(&pFrame->dstNodeId);

        // Processing of NMT commands in kernel layer
        if ((nodeId == dllkInstance_g.dllConfigParam.nodeId) || (nodeId == C_ADR_BROADCAST))
        {
            if (asndServiceId == kDllAsndNmtCommand)
            {
                // Forward NMT command
                ret = dllkcal_nmtCmdReceived(&pFrame->data.asnd.payload.nmtCommandService);
                if (ret == kErrorReject)
                {
                    DEBUG_LVL_ERROR_TRACE("%s kErrorReject is not allowed in this situation!",
                                          __func__);
                    ret = kErrorInvalidOperation;
                }

                if (ret != kErrorOk)
                    goto Exit;
            }
        }

        if (dllkInstance_g.aAsndFilter[(UINT)asndServiceId] == kDllAsndFilterAny)
        {   // ASnd service ID is registered
            // forward frame via async receive FIFO to userspace
            ret = dllkcal_asyncFrameReceived(pFrameInfo_p);
            if (ret == kErrorReject)
            {
                *pReleaseRxBuffer_p = kEdrvReleaseRxBufferLater;
                ret = kErrorOk;
            }
            else if (ret != kErrorOk)
                goto Exit;
        }
        else if (dllkInstance_g.aAsndFilter[(UINT)asndServiceId] == kDllAsndFilterLocal)
        {   // ASnd service ID is registered, but only local node ID or broadcasts
            // shall be forwarded
            if ((nodeId == dllkInstance_g.dllConfigParam.nodeId) || (nodeId == C_ADR_BROADCAST))
            {   // ASnd frame is intended for us
                // forward frame via async receive FIFO to userspace
                ret = dllkcal_asyncFrameReceived(pFrameInfo_p);
                if (ret == kErrorReject)
                {
                    *pReleaseRxBuffer_p = kEdrvReleaseRxBufferLater;
                    ret = kErrorOk;
                }
                else if (ret != kErrorOk)
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

The function is called if PRes or PReq frame was received. It posts the frame to
the event queue. It is called in states NMT_CS_READY_TO_OPERATE and
NMT_CS_OPERATIONAL. The passed PDO needs to be valid.

\param[in]      pFrameInfo_p        Pointer to frame information.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError forwardRpdo(const tFrameInfo* pFrameInfo_p)
{
    tOplkError ret = kErrorOk;

    if (dllkInstance_g.pfnCbProcessRpdo != NULL)
        ret = dllkInstance_g.pfnCbProcessRpdo(pFrameInfo_p);

    return ret;
}

#if defined(CONFIG_INCLUDE_NMT_MN)
//------------------------------------------------------------------------------
/**
\brief Check and set the sync event

The function checks if the conditions for setting a sync event are fulfilled
and sends the event if necessary.

\param[in]      fPrcSlotFinished_p  Flag determines if poll response chaining slot is
                                    finished.
\param[in]      nodeId_p            Node ID for which to do the check.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError checkAndSetSyncEvent(BOOL fPrcSlotFinished_p, UINT nodeId_p)
{
    tOplkError  ret = kErrorOk;

    if (fPrcSlotFinished_p)
    {
        dllkInstance_g.fPrcSlotFinished = TRUE;

        if ((dllkInstance_g.dllConfigParam.syncNodeId > C_ADR_SYNC_ON_SOC) &&
            (dllkInstance_g.fSyncProcessed == FALSE) &&
            (dllkInstance_g.dllConfigParam.fSyncOnPrcNode != FALSE))
        {
            dllkInstance_g.fSyncProcessed = TRUE;
            ret = dllk_postEvent(kEventTypeSync);
        }
    }
    else
    {
        if ((dllkInstance_g.dllConfigParam.syncNodeId > C_ADR_SYNC_ON_SOC) &&
            (dllkInstance_g.fSyncProcessed == FALSE) &&
            (dllkInstance_g.dllConfigParam.fSyncOnPrcNode != dllkInstance_g.fPrcSlotFinished) &&
            (nodeId_p > dllkInstance_g.dllConfigParam.syncNodeId))
        {
            dllkInstance_g.fSyncProcessed = TRUE;
            ret = dllk_postEvent(kEventTypeSync);
        }
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief Update a node

The function updates a node by sending the appropriate event to the NMT or
DLLK module.

\param[in,out]  pIntNodeInfo_p      Pointer to the node information of the node to
                                    be updated.
\param[in]      nodeId_p            Node ID of the node to be updated.
\param[in]      nodeNmtState_p      NMT state of the node to be updated.
\param[in]      pMacAddr_p          Pointer to MAC address of node, if it shall be
                                    stored in node information

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError updateNode(tDllkNodeInfo* pIntNodeInfo_p,
                             UINT nodeId_p,
                             tNmtState nodeNmtState_p,
                             const UINT8* pMacAddr_p)
{
    tOplkError          ret = kErrorOk;
    tHeartbeatEvent     heartbeatEvent;
    tDllNodeOpParam     nodeOpParam;
    tEvent              event;

    // check NMT state of CN
    heartbeatEvent.errorCode = E_NO_ERROR;
    heartbeatEvent.nmtState = nodeNmtState_p;

    if (pIntNodeInfo_p->nmtState != heartbeatEvent.nmtState)
    {   // NMT state of CN has changed -> post event to NmtMnu module
        if (!pIntNodeInfo_p->fSoftDelete)
        {   // normal isochronous CN
            if (pMacAddr_p != NULL)
                OPLK_MEMCPY(pIntNodeInfo_p->aMacAddr, pMacAddr_p, 6);

            heartbeatEvent.nodeId = nodeId_p;
            event.eventSink = kEventSinkNmtMnu;
            event.eventType = kEventTypeHeartbeat;
            event.eventArgSize = sizeof(heartbeatEvent);
            event.eventArg.pEventArg = &heartbeatEvent;
        }
        else
        {   // CN shall be deleted softly, so remove it now without issuing any error
            nodeOpParam.opNodeType = kDllNodeOpTypeIsochronous;
            nodeOpParam.nodeId = pIntNodeInfo_p->nodeId;

            event.eventSink = kEventSinkDllkCal;
            event.eventType = kEventTypeDllkDelNode;
            // $$$ d.k. set Event.netTime to current time
            event.eventArgSize = sizeof(nodeOpParam);
            event.eventArg.pEventArg = &nodeOpParam;
        }

        ret = eventk_postEvent(&event);
        if (ret != kErrorOk)
            return ret;

        // save current NMT state of CN in internal node structure
        pIntNodeInfo_p->nmtState = heartbeatEvent.nmtState;
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief Search for Node Info and handle Loss of PRes

The function searches the node information for the node from which we received
the PRes frame. For all nodes between the one we received the last PRes and
this node we issue a loss of PRes. The node information of the node will
be stored at \p ppIntNodeInfo_p.

\param[in]      nodeId_p            Node ID of node to search.
\param[out]     ppIntNodeInfo_p     Location to store the pointer to the node information.
\param[out]     pfPrcSlotFinished_p Pointer to store the flag for a finished poll response
                                    chaining slot.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError searchNodeInfo(UINT nodeId_p,
                                 tDllkNodeInfo** ppIntNodeInfo_p,
                                 BOOL* pfPrcSlotFinished_p)
{
    tOplkError      ret = kErrorOk;
    UINT8           nextNodeIndex = dllkInstance_g.curNodeIndex;
    UINT8*          pCnNodeId = &dllkInstance_g.aCnNodeIdList[dllkInstance_g.curTxBufferOffsetCycle][nextNodeIndex];
    tDllkNodeInfo*  pIntNodeInfo = NULL;

    while (*pCnNodeId != C_ADR_INVALID)
    {
        if (*pCnNodeId == nodeId_p)
        {   // CN found in list
            nextNodeIndex = nextNodeIndex - dllkInstance_g.curNodeIndex;
            dllkInstance_g.curNodeIndex += nextNodeIndex + 1;

            // issue error for each CN in list between last and current
            for (pCnNodeId-- ; nextNodeIndex > 0; nextNodeIndex--, pCnNodeId--)
            {
                ret = dllknode_issueLossOfPres(*pCnNodeId);
                if (ret != kErrorOk)
                    return ret;
            }

            pIntNodeInfo = dllknode_getNodeInfo(nodeId_p);
            break;
        }
        else
        {
            if (*pCnNodeId == C_ADR_BROADCAST)
                *pfPrcSlotFinished_p = TRUE;        // PRC slot finished
        }

        pCnNodeId++;
        nextNodeIndex++;
    }

    *ppIntNodeInfo_p = pIntNodeInfo;

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Handle MN error signaling

The function handles the error signaling on an MN.

\param[in]      pFrame_p            Pointer to received ASnd Frame
\param[in]      nodeId_p            Node ID of CN to handle error signaling
*/
//------------------------------------------------------------------------------
static void handleErrorSignaling(const tPlkFrame* pFrame_p, UINT nodeId_p)
{
    tOplkError          ret = kErrorOk;
    BOOL                fSendRequest = FALSE;
    tEvent              event;
    tDllCalIssueRequest issueReq;
    BOOL                fEcIsSet;
    tDllkNodeInfo*      pIntNodeInfo;

    pIntNodeInfo = dllknode_getNodeInfo(nodeId_p);
    if (pIntNodeInfo == NULL)
        return;         // we have no node info therefore error signaling cannot be handled

    // extract EC flag for error signaling
    fEcIsSet = ami_getUint8Le(&pFrame_p->data.asnd.payload.statusResponse.flag1) & PLK_FRAME_FLAG1_EC;

    switch (pIntNodeInfo->errSigState)
    {
        case STATE_MN_ERRSIG_INIT_WAIT_EC1:

            DEBUG_LVL_DLL_TRACE("Node:%d ERRSIG_INIT_WAIT_EC1 Cnt:%d  EC:%d\n",
                                pIntNodeInfo->nodeId,
                                pIntNodeInfo->errSigReqCnt,
                                (fEcIsSet != 0));
            fSendRequest = TRUE;
            if (fEcIsSet)
            {
                if (pIntNodeInfo->errSigReqCnt > 0) // Ensure at least one StatusReq with ER=1 is sent
                {
                    DEBUG_LVL_DLL_TRACE("       --> WAIT_EC0\n");
                    pIntNodeInfo->soaFlag1 &= ~PLK_FRAME_FLAG1_ER;
                    pIntNodeInfo->errSigState = STATE_MN_ERRSIG_INIT_WAIT_EC0;
                    pIntNodeInfo->errSigReqCnt = 0;
                }
                else
                {
                    pIntNodeInfo->errSigReqCnt++;
                }
            }
            else
            {
                pIntNodeInfo->errSigReqCnt++;
            }
            break;

        case STATE_MN_ERRSIG_INIT_WAIT_EC0:
            DEBUG_LVL_DLL_TRACE("Node:%d ERRSIG_INIT_WAIT_EC0 Cnt:%d EC:%d\n",
                                pIntNodeInfo->nodeId,
                                pIntNodeInfo->errSigReqCnt,
                                (fEcIsSet != 0));
            if (fEcIsSet)
            {
                fSendRequest = TRUE;
                // if the CN does not react it could be that it was reset
                // and waits for starting error signaling by ER=1
                if (pIntNodeInfo->errSigReqCnt > ERRSIG_WAIT_EC0_TIMEOUT_CNT)
                {
                    // restart error signaling initialization
                    pIntNodeInfo->errSigState = STATE_MN_ERRSIG_INIT_WAIT_EC1;
                    pIntNodeInfo->soaFlag1 |= PLK_FRAME_FLAG1_ER;
                    DEBUG_LVL_DLL_TRACE("       --> ERRSIG_INIT_WAIT_EC1\n");
                }
                else
                {
                    pIntNodeInfo->errSigReqCnt++;   // try a second time to set ER = 0
                }
            }
            else
            {
                // CN responded with EC=0, we are ready
                pIntNodeInfo->errSigState = STATE_MN_ERRSIG_INIT_READY;
                DEBUG_LVL_DLL_TRACE("       --> ERRSIG_INIT_READY\n");
            }
            break;

        case STATE_MN_ERRSIG_INIT_READY:
            DEBUG_LVL_DLL_TRACE("Node:%d ERRSIG_INIT_READY EC:%d\n",
                                pIntNodeInfo->nodeId,
                                (fEcIsSet != 0));
            // If EC=1 CN must be reset so we have to restart error
            // signaling initialization
            if (fEcIsSet)
            {
                pIntNodeInfo->errSigState = STATE_MN_ERRSIG_INIT_WAIT_EC1;
                pIntNodeInfo->soaFlag1 |= PLK_FRAME_FLAG1_ER;
                fSendRequest = TRUE;
                DEBUG_LVL_DLL_TRACE("       --> ERRSIG_INIT_WAIT_EC1\n");
            }
            break;

        default:
            // we never come here
            break;
    }

    if (fSendRequest)
    {
        DEBUG_LVL_DLL_TRACE("       --> Send StatusRequest: ER:%d\n",
                            ((pIntNodeInfo->soaFlag1 & PLK_FRAME_FLAG1_ER) != 0));
        event.eventSink = kEventSinkDllkCal;
        event.eventType = kEventTypeDllkIssueReq;
        issueReq.service = kDllReqServiceStatus;
        issueReq.nodeId = pIntNodeInfo->nodeId;
        issueReq.soaFlag1 = pIntNodeInfo->soaFlag1;
        event.eventArg.pEventArg = &issueReq;
        event.eventArgSize = sizeof(tDllCalIssueRequest);

        ret = eventk_postEvent(&event);
        if (ret != kErrorOk)
        {
            DEBUG_LVL_EVENTK_TRACE("%s() Couldn't post kEventTypeDllkIssueReq!\n", __func__);
        }
    }
}

#endif

#if (CONFIG_TIMER_USE_HIGHRES != FALSE)
//------------------------------------------------------------------------------
/**
\brief  CN timer callback function

This function is called by the timer module. It monitors the POWERLINK cycle
when running as CN.

\param[in]      pEventArg_p         Pointer to timer event argument.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError cbCnTimer(const tTimerEventArg* pEventArg_p)
{
    tOplkError  ret = kErrorOk;
    tNmtState   nmtState;
    UINT32      arg;

    TGT_DLLK_DECLARE_FLAGS;

    TGT_DLLK_ENTER_CRITICAL_SECTION();

    if (pEventArg_p->timerHdl.handle != dllkInstance_g.timerHdlCycle)
    {   // zombie callback - just exit
        goto Exit;
    }

    nmtState = dllkInstance_g.nmtState;
    if (nmtState <= kNmtGsResetConfiguration)
        goto Exit;

    ret = dllkstatemachine_changeState(kNmtEventDllCeFrameTimeout, nmtState);
    if (ret != kErrorOk)
        goto Exit;

    // restart the timer to detect further loss of SoC
    ret = hrestimer_modifyTimer(&dllkInstance_g.timerHdlCycle,
                                dllkInstance_g.dllConfigParam.cycleLen * 1000ULL,
                                cbCnTimer,
                                0L,
                                FALSE);

    if (ret != kErrorOk)
        goto Exit;

Exit:
    if (ret != kErrorOk)
    {
        BENCHMARK_MOD_02_TOGGLE(7);
        arg = dllkInstance_g.dllState | (kNmtEventDllCeFrameTimeout << 8);
        // Error event for API layer
        ret = eventk_postError(kEventSourceDllk, ret, sizeof(arg), &arg);
    }

    TGT_DLLK_LEAVE_CRITICAL_SECTION();

    return ret;
}
#endif

#if (CONFIG_EDRV_AUTO_RESPONSE != FALSE)
//------------------------------------------------------------------------------
/**
\brief  Enable Rx filter

This function enables the given Rx filter.

\param[in]      filterEntry_p       Filter to be enabled/disabled
\param[in]      fEnable_p           Enable the filter with TRUE
                                    Disable the filter with FALSE

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError enableRxFilter(UINT filterEntry_p, BOOL fEnable_p)
{
    tOplkError ret = kErrorOk;

    // disable corresponding Rx filter
    dllkInstance_g.aFilter[filterEntry_p].fEnable = fEnable_p;
    ret = edrv_changeRxFilter(dllkInstance_g.aFilter,
                              DLLK_FILTER_COUNT,
                              filterEntry_p,
                              EDRV_FILTER_CHANGE_STATE);

    return ret;
}
#endif

/// \}
