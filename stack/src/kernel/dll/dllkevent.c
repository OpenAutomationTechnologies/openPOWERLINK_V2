/**
********************************************************************************
\file   dllkevent.c

\brief  Event handling functions of the kernel DLL module

This file contains the event handling functions of the kernel DLL module.

\ingroup module_dllk
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2015, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
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
#include "dllk-internal.h"
#include "dllkframe.h"
#include "dllknode.h"

#include <kernel/dllkcal.h>
#include <kernel/eventk.h>
#include <kernel/errhndk.h>
#include <common/ami.h>

#if CONFIG_TIMER_USE_HIGHRES != FALSE
#include <kernel/hrestimer.h>
#endif

#if (CONFIG_DLL_PROCESS_SYNC == DLL_PROCESS_SYNC_ON_TIMER)
#include <kernel/synctimer.h>
#endif

#if defined(CONFIG_INCLUDE_NMT_MN)
#include <kernel/edrvcyclic.h>
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
static tOplkError controlPdokcalSync(BOOL fEnable_p);

static tOplkError processNmtStateChange(tNmtState newNmtState_p, tNmtState OldNmtState_p, tNmtEvent nmtEvent_p);
static tOplkError processNmtEvent(tEvent* pEvent_p);
static tOplkError processCycleFinish(tNmtState nmtState_p) SECTION_DLLK_PROCESS_CYCFIN;
static tOplkError processSync(tNmtState nmtState_p) SECTION_DLLK_PROCESS_SYNC;
static tOplkError processSyncCn(tNmtState nmtState_p, BOOL fReadyFlag_p) SECTION_DLLK_PROCESS_SYNC;
#if defined(CONFIG_INCLUDE_NMT_MN)
static tOplkError processSyncMn(tNmtState nmtState_p, BOOL fReadyFlag_p) SECTION_DLLK_PROCESS_SYNC;
static tOplkError processStartReducedCycle(void);
#endif
#if CONFIG_DLL_PRES_READY_AFTER_SOA != FALSE
static tOplkError processPresReady(tNmtState nmtState_p);
#endif
static tOplkError processFillTx(tDllAsyncReqPriority asyncReqPriority_p, tNmtState nmtState_p);

#if defined(CONFIG_INCLUDE_NMT_MN) && defined(CONFIG_INCLUDE_PRES_FORWARD)
// Request forwarding of Pres frames (for conformance test)
static tOplkError requestPresForward(UINT node_p);
#endif

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Process an event

The function processes internal events and does work that cannot be done in
interrupt context.

\param  pEvent_p        Pointer to event which should be processed.

\return The function returns a tOplkError error code.

\ingroup module_dllk
*/
//------------------------------------------------------------------------------
tOplkError dllk_process(tEvent* pEvent_p)
{
    tOplkError              ret = kErrorOk;
    tEventNmtStateChange*   pNmtStateChange;

    switch (pEvent_p->eventType)
    {
        case kEventTypeNmtStateChange:
            pNmtStateChange = (tEventNmtStateChange*)pEvent_p->pEventArg;
            ret = processNmtStateChange(pNmtStateChange->newNmtState,
                                        pNmtStateChange->oldNmtState,
                                        pNmtStateChange->nmtEvent);
            break;

        case kEventTypeNmtEvent:
            ret = processNmtEvent(pEvent_p);
            break;

        case kEventTypeDllkFillTx:
            ret = processFillTx(*((tDllAsyncReqPriority*)pEvent_p->pEventArg),
                                dllkInstance_g.nmtState);
            break;

        case kEventTypeDllkFlag1:
            // trigger update of StatusRes on SoA, because Flag 1 was changed
            if (dllkInstance_g.updateTxFrame == DLLK_UPDATE_NONE)
                dllkInstance_g.updateTxFrame = DLLK_UPDATE_STATUS;
            break;

        case kEventTypeDllkCycleFinish:
            ret = processCycleFinish(dllkInstance_g.nmtState);
            break;

        case kEventTypeSync:
            ret = processSync(dllkInstance_g.nmtState);
            break;

#if defined(CONFIG_INCLUDE_NMT_MN)
        case kEventTypeDllkStartReducedCycle:
            ret = processStartReducedCycle();
            break;

#if defined(CONFIG_INCLUDE_PRES_FORWARD)
        case kEventTypeRequPresForward:
            ret = requestPresForward(*((UINT*)pEvent_p->pEventArg));
            break;
#endif

#endif

#if CONFIG_DLL_PRES_READY_AFTER_SOA != FALSE
        case kEventTypeDllkPresReady:
            ret = processPresReady(dllkInstance_g.nmtState);
            break;
#endif


        default:
            ret = kErrorInvalidEvent;
            ASSERTMSG(ret != kErrorInvalidEvent, "dllk_process(): unhandled event type!\n");
            break;
    }

    return ret;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief  Control PDOK CAL sync function

This function controls the kernel PDO CAL sync function. It enables/disables
the sync function by sending the appropriate event.

\param  fEnable_p       Flag determines if sync should be enabled or disabled.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError controlPdokcalSync(BOOL fEnable_p)
{
    tEvent event;
    BOOL fEnable = fEnable_p;

    event.eventSink = kEventSinkPdokCal;
    event.eventType = kEventTypePdokControlSync;
    event.pEventArg = &fEnable;
    event.eventArgSize = sizeof(fEnable);

    return eventk_postEvent(&event);
}

//------------------------------------------------------------------------------
/**
\brief  Process NMT state change event

The function processes a NMT state change event.

\param  newNmtState_p           New NMT state of the local node.
\param  oldNmtState_p           Previous NMT state of the local node.
\param  nmtEvent_p              NMT event which caused the state change.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError processNmtStateChange(tNmtState newNmtState_p,
                    tNmtState oldNmtState_p, tNmtEvent nmtEvent_p)
{
    tOplkError      ret = kErrorOk;

    UNUSED_PARAMETER(nmtEvent_p);

    switch (newNmtState_p)
    {
        case kNmtGsOff:
        case kNmtGsInitialising:
            dllkInstance_g.relativeTime = 0;
            // set EC flag in Flag 1, so the MN can detect a reboot and
            // will initialize the Error Signaling.
            dllkInstance_g.flag1 = PLK_FRAME_FLAG1_EC;
            dllkInstance_g.nmtState = newNmtState_p;
            if (oldNmtState_p > kNmtGsResetConfiguration)
            {
                ret = dllknode_cleanupLocalNode(oldNmtState_p);      // deinitialize DLL and destroy frames
            }
            break;

        case kNmtGsResetApplication:
        case kNmtGsResetCommunication:
        case kNmtGsResetConfiguration:
            // at first, update NMT state in instance structure to disable frame processing
            dllkInstance_g.nmtState = newNmtState_p;
            if (oldNmtState_p > kNmtGsResetConfiguration)
            {
                ret = dllknode_cleanupLocalNode(oldNmtState_p);      // deinitialize DLL and destroy frames

            }
            break;

        // node listens for POWERLINK frames and check timeout
        case kNmtMsNotActive:
        case kNmtCsNotActive:
        case kNmtRmsNotActive:
            if (oldNmtState_p <= kNmtGsResetConfiguration)
            {
                // setup DLL and create frames
                ret = dllknode_setupLocalNode(newNmtState_p);
            }
            break;

        // node processes only async frames
        case kNmtCsPreOperational1:
#if CONFIG_TIMER_USE_HIGHRES != FALSE
            if ((ret = hrestimer_deleteTimer(&dllkInstance_g.timerHdlCycle)) != kErrorOk)
                return ret;
#endif

#if defined(CONFIG_INCLUDE_NMT_RMN)
            if (dllkInstance_g.fRedundancy)
            {
                ret = edrvcyclic_stopCycle(FALSE);
                if (ret != kErrorOk)
                    return ret;

                hrestimer_modifyTimer(&dllkInstance_g.timerHdlSwitchOver,
                                      dllkInstance_g.dllConfigParam.reducedSwitchOverMN_us * 1000ULL,
                                      dllk_cbTimerSwitchOver, 0L, FALSE);
            }
#endif

            // deactivate sync generation
            if ((ret = controlPdokcalSync(FALSE)) != kErrorOk)
                return ret;

#if (CONFIG_DLL_PROCESS_SYNC == DLL_PROCESS_SYNC_ON_TIMER)
            if ((ret = synctimer_stopSync()) != kErrorOk)
                return ret;
#endif

#if CONFIG_DLL_PRES_CHAINING_CN != FALSE
            if ((ret = dllkframe_presChainingDisable()) != kErrorOk)
                return ret;
#endif

            // update IdentRes and StatusRes
            ret = dllkframe_updateFrameStatusRes(&dllkInstance_g.pTxBuffer[DLLK_TXFRAME_STATUSRES +
                                                     dllkInstance_g.curTxBufferOffsetStatusRes],
                                                 newNmtState_p);
            if (ret != kErrorOk)
                return ret;

            ret = dllkframe_updateFrameIdentRes(&dllkInstance_g.pTxBuffer[DLLK_TXFRAME_IDENTRES +
                                                    dllkInstance_g.curTxBufferOffsetIdentRes],
                                                newNmtState_p);
            if (ret != kErrorOk)
                return ret;

            // enable IdentRes and StatusRes
#if (CONFIG_EDRV_AUTO_RESPONSE != FALSE)
            // enable corresponding Rx filter
            dllkInstance_g.aFilter[DLLK_FILTER_SOA_STATREQ].fEnable = TRUE;
            ret = edrv_changeRxFilter(dllkInstance_g.aFilter, DLLK_FILTER_COUNT,
                                   DLLK_FILTER_SOA_STATREQ, EDRV_FILTER_CHANGE_STATE);
            if (ret != kErrorOk)
                return ret;

            // enable corresponding Rx filter
            dllkInstance_g.aFilter[DLLK_FILTER_SOA_IDREQ].fEnable = TRUE;
            ret = edrv_changeRxFilter(dllkInstance_g.aFilter, DLLK_FILTER_COUNT,
                                   DLLK_FILTER_SOA_IDREQ, EDRV_FILTER_CHANGE_STATE);
            if (ret != kErrorOk)
                return ret;

#if CONFIG_DLL_PRES_CHAINING_CN != FALSE
            // enable SyncReq Rx filter
            dllkInstance_g.aFilter[DLLK_FILTER_SOA_SYNCREQ].fEnable = TRUE;
            ret = edrv_changeRxFilter(dllkInstance_g.aFilter, DLLK_FILTER_COUNT,
                                   DLLK_FILTER_SOA_SYNCREQ, EDRV_FILTER_CHANGE_STATE);
            if (ret != kErrorOk)
                return ret;
#endif
#endif

            // update PRes (for sudden changes to PreOp2)
            ret = dllkframe_updateFramePres(&dllkInstance_g.pTxBuffer[DLLK_TXFRAME_PRES +
                                                (dllkInstance_g.curTxBufferOffsetCycle ^ 1)],
                                            newNmtState_p);
            if (ret != kErrorOk)
                return ret;

            ret = dllkframe_updateFramePres(&dllkInstance_g.pTxBuffer[DLLK_TXFRAME_PRES +
                                                dllkInstance_g.curTxBufferOffsetCycle],
                                            newNmtState_p);
            if (ret != kErrorOk)
                return ret;

            // enable PRes (for sudden changes to PreOp2)
#if (CONFIG_EDRV_AUTO_RESPONSE != FALSE)
            // enable corresponding Rx filter
            dllkInstance_g.aFilter[DLLK_FILTER_PREQ].fEnable = TRUE;
            dllkInstance_g.aFilter[DLLK_FILTER_PREQ].pTxBuffer = &dllkInstance_g.pTxBuffer[DLLK_TXFRAME_PRES];
            ret = edrv_changeRxFilter(dllkInstance_g.aFilter, DLLK_FILTER_COUNT,
                                   DLLK_FILTER_PREQ, EDRV_FILTER_CHANGE_STATE | EDRV_FILTER_CHANGE_AUTO_RESPONSE);
            if (ret != kErrorOk)
                return ret;
#endif
            break;

        // node processes isochronous and asynchronous frames
        case kNmtCsPreOperational2:
            // signal update of IdentRes and StatusRes on SoA
            dllkInstance_g.updateTxFrame = DLLK_UPDATE_BOTH;

            // enable PRes (necessary if coming from Stopped)
#if (CONFIG_EDRV_AUTO_RESPONSE != FALSE)
            // enable corresponding Rx filter
            dllkInstance_g.aFilter[DLLK_FILTER_PREQ].fEnable = TRUE;
            dllkInstance_g.aFilter[DLLK_FILTER_PREQ].pTxBuffer = &dllkInstance_g.pTxBuffer[DLLK_TXFRAME_PRES];
            ret = edrv_changeRxFilter(dllkInstance_g.aFilter, DLLK_FILTER_COUNT,
                                   DLLK_FILTER_PREQ, EDRV_FILTER_CHANGE_STATE | EDRV_FILTER_CHANGE_AUTO_RESPONSE);
            if (ret != kErrorOk)
                return ret;
#endif
            break;

#if defined (CONFIG_INCLUDE_NMT_MN)
        case kNmtMsPreOperational1:
#if CONFIG_TIMER_USE_HIGHRES != FALSE
            ret = hrestimer_deleteTimer(&dllkInstance_g.timerHdlCycle);
            if (ret != kErrorOk)
                return ret;
#endif
            /// deactivate sync generation
            if ((ret = controlPdokcalSync(FALSE)) != kErrorOk)
                return ret;

            ret = edrvcyclic_stopCycle(FALSE);
            if (ret != kErrorOk)
                return ret;

#if defined(CONFIG_INCLUDE_NMT_RMN)
            if (dllkInstance_g.fRedundancy)
            {
                if ((ret = hrestimer_deleteTimer(&dllkInstance_g.timerHdlSwitchOver)) != kErrorOk)
                    return ret;

                if (oldNmtState_p == kNmtRmsNotActive)
                {   // send AMNI
                    ret = edrv_sendTxBuffer(&dllkInstance_g.pTxBuffer[DLLK_TXFRAME_AMNI]);
                    if (ret != kErrorOk)
                        return ret;
                }

                // initialize cycle counter
                if (dllkInstance_g.dllConfigParam.fAsyncOnly == FALSE)
                {
                    dllkInstance_g.cycleCount = 0;
                }
                else
                {   // it is an async-only CN -> fool changeState() to think that PRes was not expected
                    dllkInstance_g.cycleCount = 1;
                }
            }
#endif

            // update IdentRes and StatusRes
            ret = dllkframe_updateFrameIdentRes(&dllkInstance_g.pTxBuffer[DLLK_TXFRAME_IDENTRES +
                                                    dllkInstance_g.curTxBufferOffsetIdentRes],
                                                newNmtState_p);
            if (ret != kErrorOk)
                return ret;

            ret = dllkframe_updateFrameStatusRes(&dllkInstance_g.pTxBuffer[DLLK_TXFRAME_STATUSRES +
                                                     dllkInstance_g.curTxBufferOffsetStatusRes],
                                                 newNmtState_p);
            break;

        case kNmtMsReadyToOperate:
            /// activate sync generation
            if ((ret = controlPdokcalSync(TRUE)) != kErrorOk)
                return ret;
            break;

        case kNmtMsPreOperational2:
        case kNmtMsOperational:
            // signal update of IdentRes and StatusRes on SoA
            dllkInstance_g.updateTxFrame = DLLK_UPDATE_BOTH;

#if defined(CONFIG_INCLUDE_NMT_RMN)
            if (dllkInstance_g.fRedundancy && (oldNmtState_p == kNmtCsOperational))
            {
                dllkInstance_g.dllState = kDllMsWaitSocTrig;
                if ((ret = hrestimer_deleteTimer(&dllkInstance_g.timerHdlSwitchOver)) != kErrorOk)
                    return ret;

                dllkInstance_g.relativeTime += dllkInstance_g.dllConfigParam.cycleLen;
                // initialize SoAReq number for ProcessSync (cycle preparation)
                dllkInstance_g.syncLastSoaReq = dllkInstance_g.curLastSoaReq;
                // trigger synchronous task for cycle preparation
                dllkInstance_g.fSyncProcessed = TRUE;
                ret = dllk_postEvent(kEventTypeSync);
            }
#endif
            break;

#endif

        case kNmtCsReadyToOperate:
            /// activate sync generation
            if ((ret = controlPdokcalSync(TRUE)) != kErrorOk)
                return ret;
            // signal update of IdentRes and StatusRes on SoA
            dllkInstance_g.updateTxFrame = DLLK_UPDATE_BOTH;
            break;

        case kNmtCsOperational:
            // signal update of IdentRes and StatusRes on SoA
            dllkInstance_g.updateTxFrame = DLLK_UPDATE_BOTH;
#if defined(CONFIG_INCLUDE_NMT_RMN)
            if (dllkInstance_g.fRedundancy && (oldNmtState_p == kNmtMsOperational))
            {
                dllkInstance_g.dllState = kDllCsWaitSoc;
                ret = edrvcyclic_stopCycle(TRUE);
                if (ret != kErrorOk)
                    return ret;

                hrestimer_modifyTimer(&dllkInstance_g.timerHdlSwitchOver,
                                      dllkInstance_g.dllConfigParam.switchOverMN_us * 1000ULL,
                                      dllk_cbTimerSwitchOver, 0L, FALSE);

                if ((nmtEvent_p == kNmtEventGoToStandby)
                    || (nmtEvent_p == kNmtEventGoToStandbyDelayed))
                {   // save event, so cbCyclicError can start switch-over timeout
                    // appropriately
                    dllkInstance_g.nmtEventGoToStandby = nmtEvent_p;
                }
            }
#endif
            break;

        // node stopped by MN
        case kNmtCsStopped:
            // signal update of IdentRes and StatusRes on SoA
            dllkInstance_g.updateTxFrame = DLLK_UPDATE_BOTH;

#if (CONFIG_EDRV_AUTO_RESPONSE != FALSE)
            // disable auto-response for PRes filter
            dllkInstance_g.aFilter[DLLK_FILTER_PREQ].pTxBuffer = NULL;
            ret = edrv_changeRxFilter(dllkInstance_g.aFilter, DLLK_FILTER_COUNT,
                                   DLLK_FILTER_PREQ, EDRV_FILTER_CHANGE_AUTO_RESPONSE);
            if (ret != kErrorOk)
                return ret;
#endif

            // update PRes
            ret = dllkframe_updateFramePres(&dllkInstance_g.pTxBuffer[DLLK_TXFRAME_PRES +
                                                (dllkInstance_g.curTxBufferOffsetCycle ^ 1)],
                                            newNmtState_p);
            if (ret != kErrorOk)
                return ret;

            ret = dllkframe_updateFramePres(&dllkInstance_g.pTxBuffer[DLLK_TXFRAME_PRES +
                                                dllkInstance_g.curTxBufferOffsetCycle],
                                            newNmtState_p);
            if (ret != kErrorOk)
                return ret;
            break;

        // no POWERLINK cycle -> normal ethernet communication
        case kNmtMsBasicEthernet:
        case kNmtCsBasicEthernet:
            // Fill Async Tx Buffer, because state BasicEthernet was entered
            ret = processFillTx(kDllAsyncReqPrioGeneric, newNmtState_p);
            if (ret != kErrorOk)
                return ret;
            break;

        default:
            return kErrorNmtInvalidState;
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

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError processNmtEvent(tEvent* pEvent_p)
{
    tOplkError      Ret = kErrorOk;
    tNmtEvent*      pNmtEvent;
    tNmtState       NmtState;

    pNmtEvent = (tNmtEvent*)pEvent_p->pEventArg;

    switch (*pNmtEvent)
    {
        case kNmtEventDllCeSoa:
            // do preprocessing for next cycle
            NmtState = dllkInstance_g.nmtState;
#if (CONFIG_DLL_PROCESS_SYNC == DLL_PROCESS_SYNC_ON_SOA)
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

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError processFillTx(tDllAsyncReqPriority asyncReqPriority_p, tNmtState nmtState_p)
{
    tOplkError      ret = kErrorOk;
    tPlkFrame *     pTxFrame;
    tEdrvTxBuffer*  pTxBuffer;
    UINT            frameSize;
    UINT            frameCount;
    UINT            nextTxBufferOffset;
    tDllkTxBufState* pTxBufferState = NULL;
#if (CONFIG_EDRV_AUTO_RESPONSE != FALSE)
    UINT            filterEntry;
#endif

    // fill TxBuffer of specified priority with new frame if empty
    pTxFrame = NULL;
    switch (asyncReqPriority_p)
    {
        case kDllAsyncReqPrioNmt:    // NMT request priority
            nextTxBufferOffset = dllkInstance_g.curTxBufferOffsetNmtReq;
            pTxBuffer = &dllkInstance_g.pTxBuffer[DLLK_TXFRAME_NMTREQ + nextTxBufferOffset];
            pTxBufferState = &dllkInstance_g.aTxBufferStateNmtReq[nextTxBufferOffset];
#if (CONFIG_EDRV_AUTO_RESPONSE != FALSE)
            filterEntry = DLLK_FILTER_SOA_NMTREQ;
#endif
            break;

        default:    // generic priority
            nextTxBufferOffset = dllkInstance_g.curTxBufferOffsetNonPlk;
            pTxBuffer = &dllkInstance_g.pTxBuffer[DLLK_TXFRAME_NONPLK + nextTxBufferOffset];
            pTxBufferState = &dllkInstance_g.aTxBufferStateNonPlk[nextTxBufferOffset];
#if (CONFIG_EDRV_AUTO_RESPONSE != FALSE)
            filterEntry = DLLK_FILTER_SOA_NONPLK;
#endif
            break;
    }

    if (pTxBuffer->pBuffer != NULL)
    {   // NmtRequest or non-POWERLINK frame does exist
        // check if frame is empty and not being filled
        if (*pTxBufferState == kDllkTxBufEmpty)
        {
            *pTxBufferState = kDllkTxBufFilling;              // mark Tx buffer as filling is in process
            frameSize = pTxBuffer->maxBufferSize;            // set max buffer size as input parameter

            // copy frame from shared loop buffer to Tx buffer
            ret = dllkcal_getAsyncTxFrame(pTxBuffer->pBuffer, &frameSize, asyncReqPriority_p);
            if (ret == kErrorOk)
            {
                pTxFrame = (tPlkFrame*)pTxBuffer->pBuffer;
                ret = dllkframe_checkFrame(pTxFrame, frameSize);
                if (ret != kErrorOk)
                    goto Exit;

                pTxBuffer->txFrameSize = frameSize;    // set buffer valid
                *pTxBufferState = kDllkTxBufReady;

#if (CONFIG_EDRV_AUTO_RESPONSE != FALSE)
                if ((nmtState_p & (NMT_TYPE_MASK | NMT_SUPERSTATE_MASK)) == (NMT_TYPE_CS | NMT_CS_PLKMODE))
                {
                    ret = edrv_updateTxBuffer(pTxBuffer);

                    // enable corresponding Rx filter
                    dllkInstance_g.aFilter[filterEntry].fEnable = TRUE;
                    ret = edrv_changeRxFilter(dllkInstance_g.aFilter, DLLK_FILTER_COUNT,
                                           filterEntry, EDRV_FILTER_CHANGE_STATE);
                    if (ret != kErrorOk)
                        goto Exit;
                }
#endif
            }
            else if (ret == kErrorDllAsyncTxBufferEmpty)
            {   // empty Tx buffer is not a real problem so just ignore it
                ret = kErrorOk;

                *pTxBufferState = kDllkTxBufEmpty;    // mark Tx buffer as empty

#if (CONFIG_EDRV_AUTO_RESPONSE != FALSE)
                if ((nmtState_p & (NMT_TYPE_MASK | NMT_SUPERSTATE_MASK)) == (NMT_TYPE_CS | NMT_CS_PLKMODE))
                {
                    // disable corresponding Rx filter
                    dllkInstance_g.aFilter[filterEntry].fEnable = FALSE;
                    ret = edrv_changeRxFilter(dllkInstance_g.aFilter, DLLK_FILTER_COUNT,
                                           filterEntry, EDRV_FILTER_CHANGE_STATE);
                    if (ret != kErrorOk)
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
            *pTxBufferState = kDllkTxBufSending;
            ret = edrv_sendTxBuffer(pTxBuffer);
        }
        else
        {   // no frame moved to TxBuffer

            // check if TxBuffers contain unsent frames
            if (dllkInstance_g.aTxBufferStateNmtReq[dllkInstance_g.curTxBufferOffsetNmtReq] == kDllkTxBufReady)
            {   // NMT request Tx buffer contains a frame
                dllkInstance_g.aTxBufferStateNmtReq[dllkInstance_g.curTxBufferOffsetNmtReq] = kDllkTxBufSending;
                ret = edrv_sendTxBuffer(&dllkInstance_g.pTxBuffer[DLLK_TXFRAME_NMTREQ +
                                                              dllkInstance_g.curTxBufferOffsetNmtReq]);
            }
            else if (dllkInstance_g.aTxBufferStateNonPlk[dllkInstance_g.curTxBufferOffsetNonPlk] == kDllkTxBufReady)
            {   // non-POWERLINK Tx buffer contains a frame
                dllkInstance_g.aTxBufferStateNonPlk[dllkInstance_g.curTxBufferOffsetNonPlk] = kDllkTxBufSending;
                ret = edrv_sendTxBuffer(&dllkInstance_g.pTxBuffer[DLLK_TXFRAME_NONPLK +
                                                              dllkInstance_g.curTxBufferOffsetNonPlk]);
            }
            if (ret == kErrorInvalidOperation)
            {   // ignore error if caused by already active transmission
                ret = kErrorOk;
            }
        }
        dllkInstance_g.flag2 = 0;               // reset PRes flag 2
    }
    else
    {
        // update Flag 2 (PR, RS)
        ret = dllkcal_getAsyncTxCount(&asyncReqPriority_p, &frameCount);
        if (asyncReqPriority_p == kDllAsyncReqPrioNmt)
        {   // non-empty FIFO with hightest priority is for NMT requests
            if (dllkInstance_g.aTxBufferStateNmtReq[dllkInstance_g.curTxBufferOffsetNmtReq] == kDllkTxBufReady)
            {   // NMT request Tx buffer contains a frame
                // add one more frame
                frameCount++;
            }
        }
        else
        {   // non-empty FIFO with highest priority is for generic frames
            if (dllkInstance_g.aTxBufferStateNmtReq[dllkInstance_g.curTxBufferOffsetNmtReq] == kDllkTxBufReady)
            {   // NMT request Tx buffer contains a frame
                // use NMT request FIFO, because of higher priority
                frameCount = 1;
                asyncReqPriority_p = kDllAsyncReqPrioNmt;
            }
            else if (dllkInstance_g.aTxBufferStateNonPlk[dllkInstance_g.curTxBufferOffsetNonPlk] == kDllkTxBufReady)
            {   // non-POWERLINK Tx buffer contains a frame
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
            dllkInstance_g.flag2 = (UINT8)(((asyncReqPriority_p << PLK_FRAME_FLAG2_PR_SHIFT) &
                                             PLK_FRAME_FLAG2_PR) | (frameCount & PLK_FRAME_FLAG2_RS));
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

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError processCycleFinish(tNmtState nmtState_p)
{
    tOplkError      ret = kErrorOk;

    ret = dllkframe_updateFrameAsyncRes(nmtState_p);
    if (ret != kErrorOk)
        goto Exit;

    ret = errhndk_decrementCounters(NMT_IF_ACTIVE_MN(nmtState_p));

#if defined(CONFIG_INCLUDE_NMT_MN)
    if (dllkInstance_g.dllState > kDllMsNonCyclic)
    {
        if (dllkInstance_g.dllConfigParam.syncNodeId == C_ADR_SYNC_ON_SOC)
        {   // cyclic state is active, so preprocessing is necessary
            ret = processSync(nmtState_p);
        }
    }
#endif
Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Process sync event

The function processes the sync event.

\param  nmtState_p              NMT state of the node.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError processSync(tNmtState nmtState_p)
{
    tOplkError      ret = kErrorReject;
    BOOL            fReadyFlag = FALSE;

    if (dllkInstance_g.pfnCbSync != NULL)
    {
        ret = dllkInstance_g.pfnCbSync();
        if (ret == kErrorReject)
            fReadyFlag = FALSE;
        else if (ret == kErrorOk)
            fReadyFlag = TRUE;
        else
            return ret;
    }

    // do cycle preparation
#if defined(CONFIG_INCLUDE_NMT_MN)
    if (NMT_IF_ACTIVE_MN(nmtState_p))
    {   // local node is MN
        ret = processSyncMn(nmtState_p, fReadyFlag);
    }
    else
    {   // local node is CN
        ret = processSyncCn(nmtState_p, fReadyFlag);
    }
#else
    // local can only be CN as MN part is not compiled in
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

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError processSyncCn(tNmtState nmtState_p, BOOL fReadyFlag_p)
{
    tOplkError          ret = kErrorOk;
    tPlkFrame *         pTxFrame;
    tEdrvTxBuffer*      pTxBuffer;
    tFrameInfo          FrameInfo;
    UINT                nextTxBufferOffset = dllkInstance_g.curTxBufferOffsetCycle ^ 1;

    // local node is CN, update only the PRes
    pTxBuffer = &dllkInstance_g.pTxBuffer[DLLK_TXFRAME_PRES + nextTxBufferOffset];
    if (pTxBuffer->pBuffer != NULL)
    {   // PRes does exist
        pTxFrame = (tPlkFrame*)pTxBuffer->pBuffer;

        if (nmtState_p != kNmtCsOperational)
            fReadyFlag_p = FALSE;

        FrameInfo.pFrame = pTxFrame;
        FrameInfo.frameSize = pTxBuffer->txFrameSize;
        ret = dllkframe_processTpdo(&FrameInfo, fReadyFlag_p);
        if (ret != kErrorOk)
            return ret;

//      BENCHMARK_MOD_02_TOGGLE(7);

        ret = dllkframe_updateFramePres(pTxBuffer, nmtState_p);
        if (ret != kErrorOk)
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

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError processSyncMn(tNmtState nmtState_p, BOOL fReadyFlag_p)
{
    tOplkError          ret = kErrorOk;
    tPlkFrame *         pTxFrame;
    tEdrvTxBuffer*      pTxBuffer;
    UINT                index = 0;
    UINT32              nextTimeOffsetNs = 0;
    UINT                nextTxBufferOffset = dllkInstance_g.curTxBufferOffsetCycle ^ 1;

    pTxBuffer = &dllkInstance_g.pTxBuffer[DLLK_TXFRAME_SOC + nextTxBufferOffset];
    pTxBuffer->timeOffsetNs = nextTimeOffsetNs;
    pTxFrame = (tPlkFrame*)pTxBuffer->pBuffer;

    // Set SoC relative time
    ami_setUint64Le(&pTxFrame->data.soc.relativeTimeLe, dllkInstance_g.relativeTime);
    dllkInstance_g.relativeTime += dllkInstance_g.dllConfigParam.cycleLen;

    // Update SOC Prescaler Flag
    ami_setUint8Le(&pTxFrame->data.soc.flag1, dllkInstance_g.mnFlag1 & (PLK_FRAME_FLAG1_PS | PLK_FRAME_FLAG1_MC));

    if (dllkInstance_g.ppTxBufferList == NULL)
        return ret;

    dllkInstance_g.ppTxBufferList[index] = pTxBuffer;
    index++;

    ret = dllknode_setupSyncPhase(nmtState_p, fReadyFlag_p, nextTxBufferOffset, &nextTimeOffsetNs, &index);
    if (ret != kErrorOk)
        return ret;

    dllknode_setupAsyncPhase(nmtState_p, nextTxBufferOffset, nextTimeOffsetNs, &index);

    // set last list element to NULL
    dllkInstance_g.ppTxBufferList[index] = NULL;
    index++;

    ret = edrvcyclic_setNextTxBufferList(dllkInstance_g.ppTxBufferList, index);

    return ret;
}
#endif

#if CONFIG_DLL_PRES_READY_AFTER_SOA != FALSE
//------------------------------------------------------------------------------
/**
\brief  Process PRes ready event

The function processes the PRes Ready event.

\param  nmtState_p              NMT state of local node.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError processPresReady(tNmtState nmtState_p)
{
    tOplkError          ret = kErrorOk;
    tPlkFrame*          pTxFrame;

    // post PRes to transmit FIFO
    if (nmtState_p != kNmtCsBasicEthernet)
    {
        // Does PRes exist?
        if (dllkInstance_g.pTxBuffer[DLLK_TXFRAME_PRES +
                                     dllkInstance_g.curTxBufferOffsetCycle].pBuffer != NULL)
        {   // PRes does exist
            pTxFrame = (tPlkFrame*)dllkInstance_g.pTxBuffer[DLLK_TXFRAME_PRES +
                                                            dllkInstance_g.curTxBufferOffsetCycle].pBuffer;
            // update frame (NMT state, RD, RS, PR, MS, EN flags)
            if (nmtState_p < kNmtCsPreOperational2)
            {   // NMT state is not PreOp2, ReadyToOp or Op
                // fake NMT state PreOp2, because PRes will be sent only in PreOp2 or greater
                nmtState_p = kNmtCsPreOperational2;
            }
            ami_setUint8Le(&pTxFrame->data.pres.nmtStatus, (UINT8)nmtState_p);
            ami_setUint8Le(&pTxFrame->data.pres.flag2, dllkInstance_g.flag2);
            if (nmtState_p != kNmtCsOperational)
            {   // mark PDO as invalid in all NMT state but Op
                // $$$ reset only RD flag; set other flags appropriately
                ami_setUint8Le(&pTxFrame->data.pres.flag1, 0);
            }
            // $$$ make function that updates Pres, StatusRes
            // mark PRes frame as ready for transmission
            ret = edrv_setTxBufferReady(&dllkInstance_g.pTxBuffer[DLLK_TXFRAME_PRES +
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

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError processStartReducedCycle(void)
{
    tOplkError      ret = kErrorOk;

    // start the reduced cycle by programming the cycle timer
    // it is issued by NMT MN module, when PreOp1 is entered

    // clear the asynchronous queues
    ret = dllkcal_clearAsyncQueues();

    // reset cycle counter (every time a SoA is triggered in PreOp1 the counter is incremented
    // and when it reaches C_DLL_PREOP1_START_CYCLES the SoA may contain invitations)
    dllkInstance_g.cycleCount = 0;

    // remove any CN from isochronous phase
    while (dllkInstance_g.pFirstNodeInfo != NULL)
    {
        ret = dllknode_deleteNodeIsochronous(dllkInstance_g.pFirstNodeInfo);
        if (ret != kErrorOk)
            goto Exit;
    }

    while (dllkInstance_g.pFirstPrcNodeInfo != NULL)
    {
        ret = dllknode_deleteNodeIsochronous(dllkInstance_g.pFirstPrcNodeInfo);
        if (ret != kErrorOk)
            goto Exit;
    }

    // change state to NonCyclic,
    // hence changeState() will not ignore the next call
    dllkInstance_g.dllState = kDllMsNonCyclic;

#if CONFIG_TIMER_USE_HIGHRES != FALSE
    if (dllkInstance_g.dllConfigParam.asyncSlotTimeout != 0)
    {
        ret = hrestimer_modifyTimer(&dllkInstance_g.timerHdlCycle,
                                    dllkInstance_g.dllConfigParam.asyncSlotTimeout,
                                    dllkframe_cbMnTimerCycle, 0L, FALSE);
    }
#endif

    dllkInstance_g.curLastSoaReq = 0;
    dllkInstance_g.curTxBufferOffsetCycle = 0;

Exit:
    return ret;
}

#if defined(CONFIG_INCLUDE_PRES_FORWARD)
//------------------------------------------------------------------------------
/**
\brief  Request of forwarding PRes frames to the application

The function requests the DLL to forward a received PRes frame to the
application.

\param  node_p          Node ID of node to request PRes forwarding.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError requestPresForward(UINT node_p)
{
    tOplkError      ret = kErrorOk;

    if ((node_p > 0) && (node_p <= NMT_MAX_NODE_ID))
        dllkInstance_g.aPresForward[node_p - 1].numRequests++;
    else
        ret = kErrorInvalidNodeId;

    return ret;
}
#endif

#endif

/// \}
