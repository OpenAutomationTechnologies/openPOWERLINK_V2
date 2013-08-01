/**
********************************************************************************
\file   dllkevent.c

\brief  Event handling functions of kernel DLL module

This file contains the event handling functions of the kernel DLL module.

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
#include <kernel/EplDllk.h>
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
static tEplKernel processNmtStateChange(tNmtState newNmtState_p, tNmtState OldNmtState_p);
static tEplKernel processNmtEvent(tEplEvent * pEvent_p);
static tEplKernel processCycleFinish(tNmtState nmtState_p) SECTION_DLLK_PROCESS_CYCFIN;
static tEplKernel processSync(tNmtState nmtState_p) SECTION_DLLK_PROCESS_SYNC;
static tEplKernel processSyncCn(tNmtState nmtState_p, BOOL fReadyFlag_p) SECTION_DLLK_PROCESS_SYNC;
#if defined(CONFIG_INCLUDE_NMT_MN)
static tEplKernel processSyncMn(tNmtState nmtState_p, BOOL fReadyFlag_p) SECTION_DLLK_PROCESS_SYNC;
static tEplKernel processStartReducedCycle(void);
#endif
#if EPL_DLL_PRES_READY_AFTER_SOA != FALSE
static tEplKernel processPresReady(tNmtState nmtState_p);
#endif
static tEplKernel processFillTx(tEplDllAsyncReqPriority asyncReqPriority_p, tNmtState nmtState_p);

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Process an event

The function processes internal events and does work that cannot be done in
interrupt context.

\param  pEvent_p        Pointer to event which should be processed.

\return The function returns a tEplKernel error code.

\ingroup module_dllk
*/
//------------------------------------------------------------------------------
tEplKernel dllk_process(tEplEvent* pEvent_p)
{
    tEplKernel              ret = kEplSuccessful;
    tEventNmtStateChange*   pNmtStateChange;

    switch (pEvent_p->m_EventType)
    {
        case kEplEventTypeNmtStateChange:
            pNmtStateChange = (tEventNmtStateChange*) pEvent_p->m_pArg;
            ret = processNmtStateChange(pNmtStateChange->newNmtState,
                                        pNmtStateChange->oldNmtState);
            break;

        case kEplEventTypeNmtEvent:
            ret = processNmtEvent(pEvent_p);
            break;

        case kEplEventTypeDllkFillTx:
            ret = processFillTx(*((tEplDllAsyncReqPriority*)pEvent_p->m_pArg),
                                dllkInstance_g.nmtState);
            break;

        case kEplEventTypeDllkFlag1:
            // trigger update of StatusRes on SoA, because Flag 1 was changed
            if (dllkInstance_g.updateTxFrame == DLLK_UPDATE_NONE)
                dllkInstance_g.updateTxFrame = DLLK_UPDATE_STATUS;
            break;

        case kEplEventTypeDllkCycleFinish:
            ret = processCycleFinish(dllkInstance_g.nmtState);
            break;

        case kEplEventTypeSync:
            ret = processSync(dllkInstance_g.nmtState);
            break;

#if defined(CONFIG_INCLUDE_NMT_MN)
        case kEplEventTypeDllkStartReducedCycle:
            ret = processStartReducedCycle();
            break;
#endif

#if EPL_DLL_PRES_READY_AFTER_SOA != FALSE
        case kEplEventTypeDllkPresReady:
            ret = processPresReady(dllkInstance_g.nmtState);
            break;
#endif

        default:
            ret = kEplInvalidEvent;
            ASSERTMSG(ret != kEplInvalidEvent, "EplDllkProcess(): unhandled event type!\n");
            break;
    }

    return ret;
}

#if defined(CONFIG_INCLUDE_NMT_MN)
//------------------------------------------------------------------------------
/**
\brief  Issue loss of PRes

The function forwards a loss of PRes event to the error handler module.

\param  nodeId_p            Node ID of CN from which no PRes fram was received.

\return The function returns a tEplKernel error code.

\ingroup module_dllk
*/
//------------------------------------------------------------------------------
tEplKernel dllk_issueLossOfPres(UINT nodeId_p)
{
    tEplKernel          ret = kEplSuccessful;
    tDllkNodeInfo*      pIntNodeInfo;
    tEplEvent           event;
    tEplDllNodeOpParam  nodeOpParam;

    pIntNodeInfo = dllk_getNodeInfo(nodeId_p);
    if (pIntNodeInfo != NULL)
    {
        if (pIntNodeInfo->fSoftDelete == FALSE)
        {   // normal isochronous CN
            tErrHndkEvent  dllEvent;

            dllEvent.m_ulDllErrorEvents = EPL_DLL_ERR_MN_CN_LOSS_PRES;
            dllEvent.m_uiNodeId = pIntNodeInfo->nodeId;
            ret = errhndk_postError(&dllEvent);
            if (ret != kEplSuccessful)
                return ret;
        }
        else
        {   // CN shall be deleted softly, so remove it now, without issuing any error
            nodeOpParam.m_OpNodeType = kEplDllNodeOpTypeIsochronous;
            nodeOpParam.m_uiNodeId = pIntNodeInfo->nodeId;

            event.m_EventSink = kEplEventSinkDllkCal;
            event.m_EventType = kEplEventTypeDllkDelNode;
            // $$$ d.k. set Event.m_NetTime to current time
            event.m_uiSize = sizeof (nodeOpParam);
            event.m_pArg = &nodeOpParam;
            eventk_postEvent(&event);
        }
    }
    return ret;
}
#endif

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief  Post an event

The function posts the specified event type to itself.

\param  eventType_p             Event type to post.

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
tEplKernel dllk_postEvent(tEplEventType eventType_p)
{
    tEplKernel              ret;
    tEplEvent               event;

    event.m_EventSink = kEplEventSinkDllk;
    event.m_EventType = eventType_p;
    event.m_uiSize = 0;
    event.m_pArg = NULL;
    ret = eventk_postEvent(&event);
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Control PDOK CAL sync function

This function controls the kernel PDO CAL sync function. It enables/disables
the sync function by sending the appropriate event.

\param  fEnable_p       Flag determines if sync should be enabled or disabled.

\return The function returns a pointer to the node Information of the node
*/
//------------------------------------------------------------------------------
tEplKernel controlPdokcalSync (BOOL fEnable_p)
{
    tEplEvent event;
    BOOL fEnable = fEnable_p;

    event.m_EventSink = kEplEventSinkPdokCal;
    event.m_EventType = kEplEventTypePdokControlSync;
    event.m_pArg = &fEnable;
    event.m_uiSize = sizeof(fEnable);

    return eventk_postEvent(&event);
}

//----------------------------------------------------------------------------//
//                L O C A L   F U N C T I O N S                               //
//----------------------------------------------------------------------------//

//------------------------------------------------------------------------------
/**
\brief  Process NMT state change event

The function processes a NMT state change event.

\param  newNmtState_p           New NMT state of the local node.
\param  oldNmtState_p           Previous NMT state of the local node.

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
static tEplKernel processNmtStateChange(tNmtState newNmtState_p, tNmtState oldNmtState_p)
{
    tEplKernel      ret = kEplSuccessful;

    switch (newNmtState_p)
    {
        case kNmtGsOff:
        case kNmtGsInitialising:
            dllkInstance_g.relativeTime  = 0;
            // set EC flag in Flag 1, so the MN can detect a reboot and
            // will initialize the Error Signaling.
            dllkInstance_g.flag1 = EPL_FRAME_FLAG1_EC;
            dllkInstance_g.nmtState = newNmtState_p;
            if (oldNmtState_p > kNmtGsResetConfiguration)
            {
                ret = dllk_cleanupLocalNode(oldNmtState_p);      // deinitialize DLL and destroy frames
            }
            break;

        case kNmtGsResetApplication:
        case kNmtGsResetCommunication:
        case kNmtGsResetConfiguration:
            // at first, update NMT state in instance structure to disable frame processing
            dllkInstance_g.nmtState = newNmtState_p;
            if (oldNmtState_p > kNmtGsResetConfiguration)
            {
                ret = dllk_cleanupLocalNode(oldNmtState_p);      // deinitialize DLL and destroy frames

            }
            break;

        // node listens for EPL-Frames and check timeout
        case kNmtMsNotActive:
        case kNmtCsNotActive:
            if (oldNmtState_p <= kNmtGsResetConfiguration)
            {
                // setup DLL and create frames
                ret = dllk_setupLocalNode(newNmtState_p);
            }
            break;

        // node processes only async frames
        case kNmtCsPreOperational1:
#if EPL_TIMER_USE_HIGHRES != FALSE
            if ((ret = EplTimerHighReskDeleteTimer(&dllkInstance_g.timerHdlCycle)) != kEplSuccessful)
                return ret;
#endif
            /// deactivate sync generation
            if ((ret = controlPdokcalSync(FALSE)) != kEplSuccessful)
                return ret;

#if (EPL_DLL_PROCESS_SYNC == EPL_DLL_PROCESS_SYNC_ON_TIMER)
            if ((ret = EplTimerSynckStopSync()) != kEplSuccessful)
                return ret;
#endif

#if EPL_DLL_PRES_CHAINING_CN != FALSE
            if ((ret = dllk_presChainingDisable()) != kEplSuccessful)
                return ret;
#endif

            // update IdentRes and StatusRes
            ret = dllk_updateFrameStatusRes(&dllkInstance_g.pTxBuffer[DLLK_TXFRAME_STATUSRES + dllkInstance_g.curTxBufferOffsetStatusRes],
                                       newNmtState_p);
            if (ret != kEplSuccessful)
                return ret;

            ret = dllk_updateFrameIdentRes(&dllkInstance_g.pTxBuffer[DLLK_TXFRAME_IDENTRES + dllkInstance_g.curTxBufferOffsetIdentRes],
                                      newNmtState_p);
            if (ret != kEplSuccessful)
                return ret;

            // enable IdentRes and StatusRes
#if (EDRV_AUTO_RESPONSE != FALSE)
            // enable corresponding Rx filter
            dllkInstance_g.aFilter[DLLK_FILTER_SOA_STATREQ].m_fEnable = TRUE;
            ret = EdrvChangeFilter(dllkInstance_g.aFilter, DLLK_FILTER_COUNT,
                                   DLLK_FILTER_SOA_STATREQ, EDRV_FILTER_CHANGE_STATE);
            if (ret != kEplSuccessful)
                return ret;

            // enable corresponding Rx filter
            dllkInstance_g.aFilter[DLLK_FILTER_SOA_IDREQ].m_fEnable = TRUE;
            ret = EdrvChangeFilter(dllkInstance_g.aFilter, DLLK_FILTER_COUNT,
                                   DLLK_FILTER_SOA_IDREQ, EDRV_FILTER_CHANGE_STATE);
            if (ret != kEplSuccessful)
                return ret;

#if EPL_DLL_PRES_CHAINING_CN != FALSE
            // enable SyncReq Rx filter
            dllkInstance_g.aFilter[DLLK_FILTER_SOA_SYNCREQ].m_fEnable = TRUE;
            ret = EdrvChangeFilter(dllkInstance_g.aFilter, DLLK_FILTER_COUNT,
                                   DLLK_FILTER_SOA_SYNCREQ, EDRV_FILTER_CHANGE_STATE);
            if (ret != kEplSuccessful)
                return ret;
#endif
#endif

            // update PRes (for sudden changes to PreOp2)
            ret = dllk_updateFramePres(&dllkInstance_g.pTxBuffer[DLLK_TXFRAME_PRES + (dllkInstance_g.curTxBufferOffsetCycle ^ 1)],
                                  newNmtState_p);
            if (ret != kEplSuccessful)
                return ret;

            ret = dllk_updateFramePres(&dllkInstance_g.pTxBuffer[DLLK_TXFRAME_PRES + dllkInstance_g.curTxBufferOffsetCycle],
                                  newNmtState_p);
            if (ret != kEplSuccessful)
                return ret;

            // enable PRes (for sudden changes to PreOp2)
#if (EDRV_AUTO_RESPONSE != FALSE)
            // enable corresponding Rx filter
            dllkInstance_g.aFilter[DLLK_FILTER_PREQ].m_fEnable = TRUE;
            dllkInstance_g.aFilter[DLLK_FILTER_PREQ].m_pTxBuffer = &dllkInstance_g.pTxBuffer[DLLK_TXFRAME_PRES];
            ret = EdrvChangeFilter(dllkInstance_g.aFilter, DLLK_FILTER_COUNT,
                                   DLLK_FILTER_PREQ, EDRV_FILTER_CHANGE_STATE | EDRV_FILTER_CHANGE_AUTO_RESPONSE);
            if (ret != kEplSuccessful)
                return ret;
#endif
            break;

        // node processes isochronous and asynchronous frames
        case kNmtCsPreOperational2:
            // signal update of IdentRes and StatusRes on SoA
            dllkInstance_g.updateTxFrame = DLLK_UPDATE_BOTH;

            // enable PRes (necessary if coming from Stopped)
#if (EDRV_AUTO_RESPONSE != FALSE)
            // enable corresponding Rx filter
            dllkInstance_g.aFilter[DLLK_FILTER_PREQ].m_fEnable = TRUE;
            dllkInstance_g.aFilter[DLLK_FILTER_PREQ].m_pTxBuffer = &dllkInstance_g.pTxBuffer[DLLK_TXFRAME_PRES];
            ret = EdrvChangeFilter(dllkInstance_g.aFilter, DLLK_FILTER_COUNT,
                                   DLLK_FILTER_PREQ, EDRV_FILTER_CHANGE_STATE | EDRV_FILTER_CHANGE_AUTO_RESPONSE);
            if (ret != kEplSuccessful)
                return ret;
#endif
            break;

#if defined (CONFIG_INCLUDE_NMT_MN)
        case kNmtMsPreOperational1:
#if EPL_TIMER_USE_HIGHRES != FALSE
            ret = EplTimerHighReskDeleteTimer(&dllkInstance_g.timerHdlCycle);
            if (ret != kEplSuccessful)
                return ret;
#endif
            /// deactivate sync generation
            if ((ret = controlPdokcalSync(FALSE)) != kEplSuccessful)
                return ret;

            ret = EdrvCyclicStopCycle();
            if (ret != kEplSuccessful)
                return ret;

            // update IdentRes and StatusRes
            ret = dllk_updateFrameIdentRes(&dllkInstance_g.pTxBuffer[DLLK_TXFRAME_IDENTRES +
                                                                dllkInstance_g.curTxBufferOffsetIdentRes],
                                      newNmtState_p);
            if (ret != kEplSuccessful)
                return ret;

            ret = dllk_updateFrameStatusRes(&dllkInstance_g.pTxBuffer[DLLK_TXFRAME_STATUSRES +
                                                                 dllkInstance_g.curTxBufferOffsetStatusRes],
                                       newNmtState_p);
            break;

        case kNmtMsReadyToOperate:
            /// activate sync generation
            if ((ret = controlPdokcalSync(TRUE)) != kEplSuccessful)
                return ret;
            break;

        case kNmtMsPreOperational2:
        case kNmtMsOperational:
            // signal update of IdentRes and StatusRes on SoA
            dllkInstance_g.updateTxFrame = DLLK_UPDATE_BOTH;
            break;

#endif

        case kNmtCsReadyToOperate:
            /// activate sync generation
            if ((ret = controlPdokcalSync(TRUE)) != kEplSuccessful)
                return ret;
            break;

        case kNmtCsOperational:
            // signal update of IdentRes and StatusRes on SoA
            dllkInstance_g.updateTxFrame = DLLK_UPDATE_BOTH;
            break;

        // node stopped by MN
        case kNmtCsStopped:
            // signal update of IdentRes and StatusRes on SoA
            dllkInstance_g.updateTxFrame = DLLK_UPDATE_BOTH;

#if (EDRV_AUTO_RESPONSE != FALSE)
            // disable auto-response for PRes filter
            dllkInstance_g.aFilter[DLLK_FILTER_PREQ].m_pTxBuffer = NULL;
            ret = EdrvChangeFilter(dllkInstance_g.aFilter, DLLK_FILTER_COUNT,
                                   DLLK_FILTER_PREQ, EDRV_FILTER_CHANGE_AUTO_RESPONSE);
            if (ret != kEplSuccessful)
                return ret;
#endif

            // update PRes
            ret = dllk_updateFramePres(&dllkInstance_g.pTxBuffer[DLLK_TXFRAME_PRES +
                                                            (dllkInstance_g.curTxBufferOffsetCycle ^ 1)],
                                         newNmtState_p);
            if (ret != kEplSuccessful)
                return ret;

            ret = dllk_updateFramePres(&dllkInstance_g.pTxBuffer[DLLK_TXFRAME_PRES +
                                                            dllkInstance_g.curTxBufferOffsetCycle],
                                         newNmtState_p);
            if (ret != kEplSuccessful)
                return ret;
            break;

        // no EPL cycle -> normal ethernet communication
        case kNmtMsBasicEthernet:
        case kNmtCsBasicEthernet:
            // Fill Async Tx Buffer, because state BasicEthernet was entered
            ret = processFillTx(kEplDllAsyncReqPrioGeneric, newNmtState_p);
            if (ret != kEplSuccessful)
                return ret;
            break;

        default:
            return kEplNmtInvalidState;
            break;

    }

    // update NMT state in instance structure. This is done after updating all
    // Tx frames, so no frame will be transmitted by callback function, when it
    // is not up to date yet.
    dllkInstance_g.nmtState = newNmtState_p;

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Process NMT event

The function processes a NMT event.

\param  pEvent_p                Event to process.

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
static tEplKernel processNmtEvent(tEplEvent * pEvent_p)
{
    tEplKernel      Ret = kEplSuccessful;
    tNmtEvent*      pNmtEvent;
    tNmtState       NmtState;

    pNmtEvent = (tNmtEvent*) pEvent_p->m_pArg;

    switch (*pNmtEvent)
    {
        case kNmtEventDllCeSoa:
            // do preprocessing for next cycle
            NmtState = dllkInstance_g.nmtState;
#if (EPL_DLL_PROCESS_SYNC == EPL_DLL_PROCESS_SYNC_ON_SOA)
            if (dllkInstance_g.dllState != kDllGsInit)
            {   // cyclic state is active, so preprocessing is necessary
                Ret = processSync(NmtState);
            }
//            BENCHMARK_MOD_02_TOGGLE(7);
#endif
            Ret = processCycleFinish(NmtState);
            break;

        default:
            break;
    }

    return Ret;
}

//------------------------------------------------------------------------------
/**
\brief  Process fill TX event

The function processes the fill TX event.

\param  asyncReqPriority_p      Priority of asynchronous request.
\param  nmtState_p              NMT state of local node.

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
static tEplKernel processFillTx(tEplDllAsyncReqPriority asyncReqPriority_p, tNmtState nmtState_p)
{
    tEplKernel      ret = kEplSuccessful;
    tEplFrame*      pTxFrame;
    tEdrvTxBuffer*  pTxBuffer;
    UINT            frameSize;
    UINT            frameCount;
    UINT            nextTxBufferOffset;
#if (EDRV_AUTO_RESPONSE != FALSE)
    UINT            filterEntry;
#endif

    // fill TxBuffer of specified priority with new frame if empty
    pTxFrame = NULL;
    switch (asyncReqPriority_p)
    {
        case kEplDllAsyncReqPrioNmt:    // NMT request priority
            nextTxBufferOffset = dllkInstance_g.curTxBufferOffsetNmtReq;
            pTxBuffer = &dllkInstance_g.pTxBuffer[DLLK_TXFRAME_NMTREQ + nextTxBufferOffset];
#if (EDRV_AUTO_RESPONSE != FALSE)
        filterEntry = DLLK_FILTER_SOA_NMTREQ;
#endif
        break;

        default:    // generic priority
            nextTxBufferOffset = dllkInstance_g.curTxBufferOffsetNonEpl;
            pTxBuffer = &dllkInstance_g.pTxBuffer[DLLK_TXFRAME_NONEPL + nextTxBufferOffset];
#if (EDRV_AUTO_RESPONSE != FALSE)
        filterEntry = DLLK_FILTER_SOA_NONEPL;
#endif
        break;
    }

    if (pTxBuffer->m_pbBuffer != NULL)
    {   // NmtRequest or non-EPL frame does exist
        // check if frame is empty and not being filled
        if (pTxBuffer->m_uiTxMsgLen == DLLK_BUFLEN_EMPTY)
        {
            pTxBuffer->m_uiTxMsgLen = DLLK_BUFLEN_FILLING;      // mark Tx buffer as filling is in process
            frameSize = pTxBuffer->m_uiMaxBufferLen;            // set max buffer size as input parameter

            // copy frame from shared loop buffer to Tx buffer
            ret = dllkcal_getAsyncTxFrame(pTxBuffer->m_pbBuffer, &frameSize, asyncReqPriority_p);
            if (ret == kEplSuccessful)
            {
                pTxFrame = (tEplFrame *) pTxBuffer->m_pbBuffer;
                ret = dllk_checkFrame(pTxFrame, frameSize);

                pTxBuffer->m_uiTxMsgLen = frameSize;    // set buffer valid

#if (EDRV_AUTO_RESPONSE != FALSE)
                if ((nmtState_p & (NMT_TYPE_MASK | NMT_SUPERSTATE_MASK)) == (NMT_TYPE_CS | NMT_CS_PLKMODE))
                {
                    ret = EdrvUpdateTxMsgBuffer(pTxBuffer);

                    // enable corresponding Rx filter
                    dllkInstance_g.aFilter[filterEntry].m_fEnable = TRUE;
                    ret = EdrvChangeFilter(dllkInstance_g.aFilter, DLLK_FILTER_COUNT,
                                           filterEntry, EDRV_FILTER_CHANGE_STATE);
                    if (ret != kEplSuccessful)
                        goto Exit;
                }
#endif
            }
            else if (ret == kEplDllAsyncTxBufferEmpty)
            {   // empty Tx buffer is not a real problem so just ignore it
                ret = kEplSuccessful;
                pTxBuffer->m_uiTxMsgLen = DLLK_BUFLEN_EMPTY;    // mark Tx buffer as empty

#if (EDRV_AUTO_RESPONSE != FALSE)
                if ((nmtState_p & (NMT_TYPE_MASK | NMT_SUPERSTATE_MASK)) == (NMT_TYPE_CS | NMT_CS_PLKMODE))
                {
                    // disable corresponding Rx filter
                    dllkInstance_g.aFilter[filterEntry].m_fEnable = FALSE;
                    ret = EdrvChangeFilter(dllkInstance_g.aFilter, DLLK_FILTER_COUNT,
                                           filterEntry, EDRV_FILTER_CHANGE_STATE);
                    if (ret != kEplSuccessful)
                        goto Exit;
                }
#endif
            }
            else
            {
                goto Exit;
            }
        }
    }

    if ((nmtState_p == kNmtCsBasicEthernet) || (nmtState_p == kNmtMsBasicEthernet))
    {   // send frame immediately
        if (pTxFrame != NULL)
        {   // frame is present - padding is done by Edrv or ethernet controller
            ret = EdrvSendTxMsg(pTxBuffer);
        }
        else
        {   // no frame moved to TxBuffer

            // check if TxBuffers contain unsent frames
            if (dllkInstance_g.pTxBuffer[DLLK_TXFRAME_NMTREQ +
                                         dllkInstance_g.curTxBufferOffsetNmtReq].m_uiTxMsgLen > DLLK_BUFLEN_EMPTY)
            {   // NMT request Tx buffer contains a frame
                ret = EdrvSendTxMsg(&dllkInstance_g.pTxBuffer[DLLK_TXFRAME_NMTREQ +
                                                              dllkInstance_g.curTxBufferOffsetNmtReq]);
            }
            else if (dllkInstance_g.pTxBuffer[DLLK_TXFRAME_NONEPL +
                                              dllkInstance_g.curTxBufferOffsetNonEpl].m_uiTxMsgLen > DLLK_BUFLEN_EMPTY)
            {   // non-EPL Tx buffer contains a frame
                ret = EdrvSendTxMsg(&dllkInstance_g.pTxBuffer[DLLK_TXFRAME_NONEPL +
                                                              dllkInstance_g.curTxBufferOffsetNonEpl]);
            }
            if (ret == kEplInvalidOperation)
            {   // ignore error if caused by already active transmission
                ret = kEplSuccessful;
            }
        }
        dllkInstance_g.flag2 = 0;               // reset PRes flag 2
    }
    else
    {
        // update Flag 2 (PR, RS)
        ret = dllkcal_getAsyncTxCount(&asyncReqPriority_p, &frameCount);
        if (asyncReqPriority_p == kEplDllAsyncReqPrioNmt)
        {   // non-empty FIFO with hightest priority is for NMT requests
            if (dllkInstance_g.pTxBuffer[DLLK_TXFRAME_NMTREQ +
                                         dllkInstance_g.curTxBufferOffsetNmtReq].m_uiTxMsgLen > DLLK_BUFLEN_EMPTY)
            {   // NMT request Tx buffer contains a frame
                // add one more frame
                frameCount++;
            }
        }
        else
        {   // non-empty FIFO with highest priority is for generic frames
            if (dllkInstance_g.pTxBuffer[DLLK_TXFRAME_NMTREQ +
                                         dllkInstance_g.curTxBufferOffsetNmtReq].m_uiTxMsgLen > DLLK_BUFLEN_EMPTY)
            {   // NMT request Tx buffer contains a frame
                // use NMT request FIFO, because of higher priority
                frameCount = 1;
                asyncReqPriority_p = kEplDllAsyncReqPrioNmt;
            }
            else if (dllkInstance_g.pTxBuffer[DLLK_TXFRAME_NONEPL +
                                              dllkInstance_g.curTxBufferOffsetNonEpl].m_uiTxMsgLen > DLLK_BUFLEN_EMPTY)
            {   // non-EPL Tx buffer contains a frame
                // use NMT request FIFO, because of higher priority
                // add one more frame
                frameCount++;
            }
        }

        if (frameCount > 7)
        {   // limit frame request to send counter to 7
            frameCount = 7;
        }
        if (frameCount > 0)
        {
            dllkInstance_g.flag2 = (UINT8) (((asyncReqPriority_p << EPL_FRAME_FLAG2_PR_SHIFT) &
                                             EPL_FRAME_FLAG2_PR) | (frameCount & EPL_FRAME_FLAG2_RS));
        }
        else
        {
            dllkInstance_g.flag2 = 0;
        }
        dllkInstance_g.updateTxFrame = DLLK_UPDATE_BOTH;
    }

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Process cycle finish event

The function processes a cycle finish event.

\param  nmtState_p              NMT state of the node.

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
static tEplKernel processCycleFinish(tNmtState nmtState_p)
{
    tEplKernel      ret = kEplReject;
    tEdrvTxBuffer*  pTxBuffer;

    switch (dllkInstance_g.updateTxFrame)
    {
        case DLLK_UPDATE_BOTH:
            dllkInstance_g.curTxBufferOffsetIdentRes ^= 1;
            pTxBuffer = &dllkInstance_g.pTxBuffer[DLLK_TXFRAME_IDENTRES +
                                                  dllkInstance_g.curTxBufferOffsetIdentRes];
            if (pTxBuffer->m_pbBuffer != NULL)
            {   // IdentRes does exist
                if ((ret = dllk_updateFrameIdentRes(pTxBuffer, nmtState_p)) != kEplSuccessful)
                    return ret;
            }
            // fall-through

        case DLLK_UPDATE_STATUS:
            dllkInstance_g.curTxBufferOffsetStatusRes ^= 1;
            pTxBuffer = &dllkInstance_g.pTxBuffer[DLLK_TXFRAME_STATUSRES +
                                                  dllkInstance_g.curTxBufferOffsetStatusRes];
            if (pTxBuffer->m_pbBuffer != NULL)
            {   // StatusRes does exist
                if ((ret = dllk_updateFrameStatusRes(pTxBuffer, nmtState_p)) != kEplSuccessful)
                    return ret;
            }

            // reset signal variable
            dllkInstance_g.updateTxFrame = DLLK_UPDATE_NONE;
            break;

        default:
            break;
    }

    ret = errhndk_decrementCounters((nmtState_p >= kNmtMsNotActive));

#if defined(CONFIG_INCLUDE_NMT_MN)
    if (dllkInstance_g.dllState > kDllMsNonCyclic)
    {
        if (dllkInstance_g.dllConfigParam.m_uiSyncNodeId == EPL_C_ADR_SYNC_ON_SOC)
        {   // cyclic state is active, so preprocessing is necessary
            ret = processSync(nmtState_p);
        }
    }
#endif

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Process sync event

The function processes the sync event.

\param  nmtState_p              NMT state of the node.

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
static tEplKernel processSync(tNmtState nmtState_p)
{
    tEplKernel      ret = kEplReject;
    BOOL            fReadyFlag = FALSE;

    if (dllkInstance_g.pfnCbSync != NULL)
    {
        ret = dllkInstance_g.pfnCbSync();
        if (ret == kEplReject)
            fReadyFlag = FALSE;
        else if (ret == kEplSuccessful)
            fReadyFlag = TRUE;
        else
            return ret;
    }

    // do cycle preparation
#if defined(CONFIG_INCLUDE_NMT_MN)
    if (nmtState_p >= kNmtMsNotActive)
    {   // local node is MN
        ret = processSyncMn(nmtState_p, fReadyFlag);
    }
    else
    {   // local node is CN
        ret = processSyncCn(nmtState_p, fReadyFlag);
    }
#else
    // local could only be CN as MN part is not compiled in
    ret = processSyncCn(nmtState_p, fReadyFlag);
#endif

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Process sync event on CN

The function processes the sync event on a CN.

\param  nmtState_p              NMT state of the node.
\param  fReadyFlag_p            Status of the ready flag.

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
static tEplKernel processSyncCn(tNmtState nmtState_p, BOOL fReadyFlag_p)
{
    tEplKernel          ret = kEplSuccessful;
    tEplFrame*          pTxFrame;
    tEdrvTxBuffer*      pTxBuffer;
    tEplFrameInfo       FrameInfo;
    UINT                nextTxBufferOffset = dllkInstance_g.curTxBufferOffsetCycle ^ 1;

    // local node is CN, update only the PRes
    pTxBuffer = &dllkInstance_g.pTxBuffer[DLLK_TXFRAME_PRES + nextTxBufferOffset];
    if (pTxBuffer->m_pbBuffer != NULL)
    {   // PRes does exist
        pTxFrame = (tEplFrame *) pTxBuffer->m_pbBuffer;

        if (nmtState_p != kNmtCsOperational)
            fReadyFlag_p = FALSE;

        FrameInfo.m_pFrame = pTxFrame;
        FrameInfo.m_uiFrameSize = pTxBuffer->m_uiTxMsgLen;
        ret = dllk_processTpdo(&FrameInfo, fReadyFlag_p);
        if (ret != kEplSuccessful)
            return ret;

//      BENCHMARK_MOD_02_TOGGLE(7);

        ret = dllk_updateFramePres(pTxBuffer, nmtState_p);
        if (ret != kEplSuccessful)
            return ret;

        // switch to next cycle
        dllkInstance_g.curTxBufferOffsetCycle = (UINT8)nextTxBufferOffset;
    }

    return ret;
}

#if defined(CONFIG_INCLUDE_NMT_MN)
//------------------------------------------------------------------------------
/**
\brief  Process sync event on MN

The function processes the sync event on a MN.

\param  nmtState_p              NMT state of the node.
\param  fReadyFlag_p            Status of the ready flag.

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
static tEplKernel processSyncMn(tNmtState nmtState_p, BOOL fReadyFlag_p)
{
    tEplKernel          ret = kEplSuccessful;
    tEplFrame*          pTxFrame;
    tEdrvTxBuffer*      pTxBuffer;
    UINT                index = 0;
    UINT32              nextTimeOffsetNs = 0;
    UINT                nextTxBufferOffset = dllkInstance_g.curTxBufferOffsetCycle ^ 1;

    pTxBuffer = &dllkInstance_g.pTxBuffer[DLLK_TXFRAME_SOC + nextTxBufferOffset];
    pTxBuffer->m_dwTimeOffsetNs = nextTimeOffsetNs;
    pTxFrame = (tEplFrame *)pTxBuffer->m_pbBuffer;

    // Set SoC relative time
    AmiSetQword64ToLe( &pTxFrame->m_Data.m_Soc.m_le_RelativeTime, dllkInstance_g.relativeTime);
    dllkInstance_g.relativeTime += dllkInstance_g.dllConfigParam.m_dwCycleLen;

    if (dllkInstance_g.ppTxBufferList == NULL)
        return ret;

    dllkInstance_g.ppTxBufferList[index] = pTxBuffer;
    index++;

    ret = dllk_setupSyncPhase(nmtState_p, fReadyFlag_p, nextTxBufferOffset, &nextTimeOffsetNs, &index);
    if (ret != kEplSuccessful)
        return ret;

    dllk_setupAsyncPhase(nmtState_p, nextTxBufferOffset, nextTimeOffsetNs, &index);

    // set last list element to NULL
    dllkInstance_g.ppTxBufferList[index] = NULL;
    index++;

    ret = EdrvCyclicSetNextTxBufferList(dllkInstance_g.ppTxBufferList, index);

    return ret;
}
#endif

#if EPL_DLL_PRES_READY_AFTER_SOA != FALSE
//------------------------------------------------------------------------------
/**
\brief  Process PRes ready event

The function processes the PRes Ready event.

\param  nmtState_p              NMT state of local node.

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
static tEplKernel processPresReady(tNmtState nmtState_p)
{
    tEplKernel          ret = kEplSuccessful;
    tEplFrame*          pTxFrame;

    // post PRes to transmit FIFO
    if (nmtState_p != kNmtCsBasicEthernet)
    {
        // Does PRes exist?
        if (dllkInstance_g.pTxBuffer[DLLK_TXFRAME_PRES +
                                     dllkInstance_g.curTxBufferOffsetCycle].m_pbBuffer != NULL)
        {   // PRes does exist
            pTxFrame = (tEplFrame *) dllkInstance_g.pTxBuffer[DLLK_TXFRAME_PRES +
                                                              dllkInstance_g.curTxBufferOffsetCycle].m_pbBuffer;
            // update frame (NMT state, RD, RS, PR, MS, EN flags)
            if (nmtState_p < kNmtCsPreOperational2)
            {   // NMT state is not PreOp2, ReadyToOp or Op
                // fake NMT state PreOp2, because PRes will be sent only in PreOp2 or greater
                nmtState_p = kNmtCsPreOperational2;
            }
            AmiSetByteToLe(&pTxFrame->m_Data.m_Pres.m_le_bNmtStatus, (UINT8) nmtState_p);
            AmiSetByteToLe(&pTxFrame->m_Data.m_Pres.m_le_bFlag2, dllkInstance_g.flag2);
            if (nmtState_p != kNmtCsOperational)
            {   // mark PDO as invalid in all NMT state but Op
                // $$$ reset only RD flag; set other flags appropriately
                AmiSetByteToLe(&pTxFrame->m_Data.m_Pres.m_le_bFlag1, 0);
            }
            // $$$ make function that updates Pres, StatusRes
            // mark PRes frame as ready for transmission
            ret = EdrvTxMsgReady(&dllkInstance_g.pTxBuffer[DLLK_TXFRAME_PRES +
                                                           dllkInstance_g.curTxBufferOffsetCycle]);
        }
    }
    return ret;
}
#endif

#if defined(CONFIG_INCLUDE_NMT_MN)
//------------------------------------------------------------------------------
/**
\brief  Process StartReducedCycle Event

The function processes the StartReducedCycle event.

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
static tEplKernel processStartReducedCycle(void)
{
    tEplKernel      ret = kEplSuccessful;

    // start the reduced cycle by programming the cycle timer
    // it is issued by NMT MN module, when PreOp1 is entered

    // clear the asynchronous queues
    ret = dllkcal_clearAsyncQueues();

    // reset cycle counter (every time a SoA is triggered in PreOp1 the counter is incremented
    // and when it reaches EPL_C_DLL_PREOP1_START_CYCLES the SoA may contain invitations)
    dllkInstance_g.cycleCount = 0;

    // remove any CN from isochronous phase
    while (dllkInstance_g.pFirstNodeInfo != NULL)
    {
        ret = dllk_deleteNodeIsochronous(dllkInstance_g.pFirstNodeInfo);
        if (ret != kEplSuccessful)
            goto Exit;
    }

#if EPL_DLL_PRES_CHAINING_MN != FALSE
    while (dllkInstance_g.pFirstPrcNodeInfo != NULL)
    {
        ret = dllk_deleteNodeIsochronous(dllkInstance_g.pFirstPrcNodeInfo);
        if (ret != kEplSuccessful)
            goto Exit;
    }
#endif

    // change state to NonCyclic,
    // hence changeState() will not ignore the next call
    dllkInstance_g.dllState = kDllMsNonCyclic;

#if EPL_TIMER_USE_HIGHRES != FALSE
    if (dllkInstance_g.dllConfigParam.m_dwAsyncSlotTimeout != 0)
    {
        ret = EplTimerHighReskModifyTimerNs(&dllkInstance_g.timerHdlCycle,
                                            dllkInstance_g.dllConfigParam.m_dwAsyncSlotTimeout,
                                            dllk_cbMnTimerCycle, 0L, FALSE);
    }
#endif

    dllkInstance_g.curLastSoaReq = 0;
    dllkInstance_g.curTxBufferOffsetCycle = 0;

Exit:
    return ret;
}
#endif

///\}

