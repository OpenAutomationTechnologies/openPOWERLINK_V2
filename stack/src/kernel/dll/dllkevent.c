/**
********************************************************************************
\file   dllkevent.c

\brief  Event handling functions of the kernel DLL module

This file contains the event handling functions of the kernel DLL module.

\ingroup module_dllk
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2017, Kalycito Infotech Private Limited
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
#include "dllk-internal.h"
#include "dllkframe.h"
#include "dllknode.h"

#include <kernel/dllkcal.h>
#include <kernel/eventk.h>
#include <kernel/errhndk.h>
#include <common/ami.h>

#include <kernel/timesynck.h>

#if (CONFIG_TIMER_USE_HIGHRES != FALSE)
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
#if (defined(CONFIG_INCLUDE_NMT_MN) && defined(CONFIG_INCLUDE_SOC_TIME_FORWARD))
#define DLLK_SEC_TO_NSEC_FACTOR 1000000000U
#endif

//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static tOplkError controlTimeSync(BOOL fEnable_p);

static tOplkError processNmtStateChange(tNmtState newNmtState_p, tNmtState oldNmtState_p, tNmtEvent nmtEvent_p);
static tOplkError processNmtEvent(const tEvent* pEvent_p);
static tOplkError processCycleFinish(tNmtState nmtState_p) SECTION_DLLK_PROCESS_CYCFIN;
static tOplkError processSync(tNmtState nmtState_p) SECTION_DLLK_PROCESS_SYNC;
static tOplkError processSyncCn(tNmtState nmtState_p, BOOL fReadyFlag_p) SECTION_DLLK_PROCESS_SYNC;
#if defined(CONFIG_INCLUDE_NMT_MN)
static tOplkError processSyncMn(tNmtState nmtState_p, BOOL fReadyFlag_p) SECTION_DLLK_PROCESS_SYNC;
static tOplkError processStartReducedCycle(void);
#endif
static tOplkError processFillTx(tDllAsyncReqPriority asyncReqPriority_p, tNmtState nmtState_p);

#if (defined(CONFIG_INCLUDE_NMT_MN) && defined(CONFIG_INCLUDE_PRES_FORWARD))
// Request forwarding of Pres frames (for conformance test)
static tOplkError requestPresForward(UINT node_p);
#endif

#if (defined(CONFIG_INCLUDE_NMT_MN) && defined(CONFIG_INCLUDE_SOC_TIME_FORWARD))
static void addNetTime(const tNetTime* pInputNetTime_p,
                       const tNetTime cycleLength_p,
                       tNetTime* pResultantNetTime_p);
#endif

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Process an event

The function processes internal events and does work that cannot be done in
interrupt context.

\param[in]      pEvent_p            Pointer to event which should be processed.

\return The function returns a tOplkError error code.

\ingroup module_dllk
*/
//------------------------------------------------------------------------------
tOplkError dllk_process(const tEvent* pEvent_p)
{
    tOplkError                  ret = kErrorOk;
    const tEventNmtStateChange* pNmtStateChange;

    switch (pEvent_p->eventType)
    {
        case kEventTypeNmtStateChange:
            pNmtStateChange = (const tEventNmtStateChange*)pEvent_p->eventArg.pEventArg;
            ret = processNmtStateChange(pNmtStateChange->newNmtState,
                                        pNmtStateChange->oldNmtState,
                                        pNmtStateChange->nmtEvent);
            break;

        case kEventTypeNmtEvent:
            ret = processNmtEvent(pEvent_p);
            break;

        case kEventTypeDllkFillTx:
            ret = processFillTx(*((const tDllAsyncReqPriority*)pEvent_p->eventArg.pEventArg),
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
            ret = requestPresForward(*((const UINT*)pEvent_p->eventArg.pEventArg));
            break;
#endif

#endif

        default:
            TRACE("%s(): unhandled event type!\n", __func__);
#if !defined(NDEBUG)
            // Severe error -> stop execution here
            for (;;);
#else
            ret = kErrorInvalidEvent;
            break;
#endif
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
\brief  Control CAL timesync function

This function controls the kernel CAL timesync function. It enables/disables
the sync function by sending the appropriate event.

\param[in]      fEnable_p           Flag determines if sync should be enabled or disabled.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError controlTimeSync(BOOL fEnable_p)
{
    tOplkError  ret = kErrorOk;
    tEvent      event;
    BOOL        fEnable = fEnable_p;

    event.eventSink = kEventSinkTimesynck;
    event.eventType = kEventTypeTimesynckControl;
    event.eventArg.pEventArg = &fEnable;
    event.eventArgSize = sizeof(fEnable);

    ret = eventk_postEvent(&event);

#if (CONFIG_DLL_PROCESS_SYNC == DLL_PROCESS_SYNC_ON_TIMER)
    if (ret == kErrorOk)
    {
        // Activate/deactivate external synchronization interrupt
        synctimer_controlExtSyncIrq(fEnable);
    }
#endif

#if (CONFIG_TIMER_USE_HIGHRES != FALSE)
    if (ret == kErrorOk)
    {
        // Activate/deactivate external synchronization interrupt
        hrestimer_controlExtSyncIrq(fEnable);
    }
#endif

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Process NMT state change event

The function processes a NMT state change event.

\param[in]      newNmtState_p       New NMT state of the local node.
\param[in]      oldNmtState_p       Previous NMT state of the local node.
\param[in]      nmtEvent_p          NMT event which caused the state change.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError processNmtStateChange(tNmtState newNmtState_p,
                                        tNmtState oldNmtState_p,
                                        tNmtEvent nmtEvent_p)
{
    tOplkError  ret = kErrorOk;

#if !defined(CONFIG_INCLUDE_NMT_RMN)
    UNUSED_PARAMETER(nmtEvent_p);
#endif

    switch (newNmtState_p)
    {
        case kNmtGsOff:
        case kNmtGsInitialising:
            dllkInstance_g.socTime.relTime = 0;
#if (defined(CONFIG_INCLUDE_NMT_MN) && defined(CONFIG_INCLUDE_SOC_TIME_FORWARD))
            dllkInstance_g.fIncrementNetTime = FALSE;  // Reset increment net time flag
#endif
            // set EC flag in Flag 1, so the MN can detect a reboot and
            // will initialize the Error Signaling.
            dllkInstance_g.flag1 = PLK_FRAME_FLAG1_EC;
            dllkInstance_g.nmtState = newNmtState_p;
            if (oldNmtState_p > kNmtGsResetConfiguration)
                ret = dllknode_cleanupLocalNode(oldNmtState_p);      // de-initialize DLL and destroy frames
            break;

        case kNmtGsResetApplication:
        case kNmtGsResetCommunication:
        case kNmtGsResetConfiguration:
            // at first, update NMT state in instance structure to disable frame processing
            dllkInstance_g.nmtState = newNmtState_p;
            if (oldNmtState_p > kNmtGsResetConfiguration)
                ret = dllknode_cleanupLocalNode(oldNmtState_p);      // de-initialize DLL and destroy frames
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
#if (CONFIG_TIMER_USE_HIGHRES != FALSE)
            ret = hrestimer_deleteTimer(&dllkInstance_g.timerHdlCycle);
            if (ret != kErrorOk)
                return ret;
#endif

#if defined(CONFIG_INCLUDE_NMT_RMN)
            if (dllkInstance_g.fRedundancy)
            {
                ret = edrvcyclic_stopCycle(FALSE);
                if (ret != kErrorOk)
                    return ret;

                hrestimer_modifyTimer(&dllkInstance_g.timerHdlSwitchOver,
                                      dllkInstance_g.dllConfigParam.reducedSwitchOverTimeMn * 1000ULL,
                                      dllk_cbTimerSwitchOver,
                                      0L,
                                      FALSE);
            }
#endif

            // deactivate sync generation
            ret = controlTimeSync(FALSE);
            if (ret != kErrorOk)
                return ret;

#if (CONFIG_DLL_PROCESS_SYNC == DLL_PROCESS_SYNC_ON_TIMER)
            ret = synctimer_stopSync();
            if (ret != kErrorOk)
                return ret;
#endif

#if (CONFIG_DLL_PRES_CHAINING_CN != FALSE)
            ret = dllkframe_presChainingDisable();
            if (ret != kErrorOk)
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
            ret = edrv_changeRxFilter(dllkInstance_g.aFilter,
                                      DLLK_FILTER_COUNT,
                                      DLLK_FILTER_SOA_STATREQ,
                                      EDRV_FILTER_CHANGE_STATE);
            if (ret != kErrorOk)
                return ret;

            // enable corresponding Rx filter
            dllkInstance_g.aFilter[DLLK_FILTER_SOA_IDREQ].fEnable = TRUE;
            ret = edrv_changeRxFilter(dllkInstance_g.aFilter,
                                      DLLK_FILTER_COUNT,
                                      DLLK_FILTER_SOA_IDREQ,
                                      EDRV_FILTER_CHANGE_STATE);
            if (ret != kErrorOk)
                return ret;

#if (CONFIG_DLL_PRES_CHAINING_CN != FALSE)
            // enable SyncReq Rx filter
            dllkInstance_g.aFilter[DLLK_FILTER_SOA_SYNCREQ].fEnable = TRUE;
            ret = edrv_changeRxFilter(dllkInstance_g.aFilter,
                                      DLLK_FILTER_COUNT,
                                      DLLK_FILTER_SOA_SYNCREQ,
                                      EDRV_FILTER_CHANGE_STATE);
            if (ret != kErrorOk)
                return ret;
#endif
#endif

            // update PRes (for sudden changes to PreOp2)
            ret = dllkframe_updateFramePres(&dllkInstance_g.pTxBuffer[DLLK_TXFRAME_PRES +
                                                (dllkInstance_g.curTxBufferOffsetCycle ^ 1)],
                                            kNmtCsPreOperational2);
            if (ret != kErrorOk)
                return ret;

            ret = dllkframe_updateFramePres(&dllkInstance_g.pTxBuffer[DLLK_TXFRAME_PRES +
                                                dllkInstance_g.curTxBufferOffsetCycle],
                                            kNmtCsPreOperational2);
            if (ret != kErrorOk)
                return ret;

            // enable PRes (for sudden changes to PreOp2)
#if (CONFIG_EDRV_AUTO_RESPONSE != FALSE)
            // enable corresponding Rx filter
            dllkInstance_g.aFilter[DLLK_FILTER_PREQ].fEnable = TRUE;
            dllkInstance_g.aFilter[DLLK_FILTER_PREQ].pTxBuffer = &dllkInstance_g.pTxBuffer[DLLK_TXFRAME_PRES];
            ret = edrv_changeRxFilter(dllkInstance_g.aFilter,
                                      DLLK_FILTER_COUNT,
                                      DLLK_FILTER_PREQ,
                                      EDRV_FILTER_CHANGE_STATE | EDRV_FILTER_CHANGE_AUTO_RESPONSE);
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
            ret = edrv_changeRxFilter(dllkInstance_g.aFilter,
                                      DLLK_FILTER_COUNT,
                                      DLLK_FILTER_PREQ,
                                      EDRV_FILTER_CHANGE_STATE | EDRV_FILTER_CHANGE_AUTO_RESPONSE);
            if (ret != kErrorOk)
                return ret;
#endif
            break;

#if defined(CONFIG_INCLUDE_NMT_MN)
        case kNmtMsPreOperational1:
#if (CONFIG_TIMER_USE_HIGHRES != FALSE)
            ret = hrestimer_deleteTimer(&dllkInstance_g.timerHdlCycle);
            if (ret != kErrorOk)
                return ret;
#endif
            /// deactivate sync generation
            ret = controlTimeSync(FALSE);
            if (ret != kErrorOk)
                return ret;

            ret = edrvcyclic_stopCycle(FALSE);
            if (ret != kErrorOk)
                return ret;

#if defined(CONFIG_INCLUDE_NMT_RMN)
            if (dllkInstance_g.fRedundancy)
            {
                ret = hrestimer_deleteTimer(&dllkInstance_g.timerHdlSwitchOver);
                if (ret != kErrorOk)
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
            ret = controlTimeSync(TRUE);
            if (ret != kErrorOk)
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
                ret = hrestimer_deleteTimer(&dllkInstance_g.timerHdlSwitchOver);
                if (ret != kErrorOk)
                    return ret;

                // start cycle timer to send frames
                ret = edrvcyclic_startCycle(TRUE);
                if (ret != kErrorOk)
                {
                    return ret;
                }

                dllkInstance_g.socTime.relTime += dllkInstance_g.dllConfigParam.cycleLen;
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
            ret = controlTimeSync(TRUE);
            if (ret != kErrorOk)
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
                                      dllkInstance_g.dllConfigParam.switchOverTimeMn * 1000ULL,
                                      dllk_cbTimerSwitchOver,
                                      0L,
                                      FALSE);

                if ((nmtEvent_p == kNmtEventGoToStandby) || (nmtEvent_p == kNmtEventGoToStandbyDelayed))
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
            ret = edrv_changeRxFilter(dllkInstance_g.aFilter,
                                      DLLK_FILTER_COUNT,
                                      DLLK_FILTER_PREQ,
                                      EDRV_FILTER_CHANGE_AUTO_RESPONSE);
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

        // no POWERLINK cycle -> normal Ethernet communication
        case kNmtMsBasicEthernet:
        case kNmtCsBasicEthernet:
            // Fill Async Tx Buffer, because state BasicEthernet was entered
            ret = processFillTx(kDllAsyncReqPrioGeneric, newNmtState_p);
            if (ret != kErrorOk)
                return ret;
            break;

        default:
            return kErrorNmtInvalidState;
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

\param[in]      pEvent_p            Event to process.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError processNmtEvent(const tEvent* pEvent_p)
{
    tOplkError          ret = kErrorOk;
    const tNmtEvent*    pNmtEvent;
    tNmtState           nmtState;

    pNmtEvent = (const tNmtEvent*)pEvent_p->eventArg.pEventArg;

    switch (*pNmtEvent)
    {
        case kNmtEventDllCeSoa:
            // do preprocessing for next cycle
            nmtState = dllkInstance_g.nmtState;
#if (CONFIG_DLL_PROCESS_SYNC == DLL_PROCESS_SYNC_ON_SOA)
            if (dllkInstance_g.dllState != kDllGsInit)
            {   // cyclic state is active, so preprocessing is necessary
                ret = processSync(nmtState);
            }
//            BENCHMARK_MOD_02_TOGGLE(7);
#endif
            ret = processCycleFinish(nmtState);
            break;

        default:
            break;
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Process fill TX event

The function processes the fill TX event.

\param[in]      asyncReqPriority_p  Priority of asynchronous request.
\param[in]      nmtState_p          NMT state of local node.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError processFillTx(tDllAsyncReqPriority asyncReqPriority_p, tNmtState nmtState_p)
{
    tOplkError          ret = kErrorOk;
    tPlkFrame *         pTxFrame;
    tEdrvTxBuffer*      pTxBuffer;
    size_t              frameSize;
    UINT                frameCount;
    UINT                nextTxBufferOffset;
    tDllkTxBufState*    pTxBufferState = NULL;
#if (CONFIG_EDRV_AUTO_RESPONSE != FALSE)
    UINT                filterEntry;
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
            frameSize = pTxBuffer->maxBufferSize;             // set max buffer size as input parameter

            // copy frame from shared loop buffer to Tx buffer
            ret = dllkcal_getAsyncTxFrame(pTxBuffer->pBuffer, &frameSize, asyncReqPriority_p);
            if (ret == kErrorOk)
            {
                pTxFrame = (tPlkFrame*)pTxBuffer->pBuffer;
                ret = dllkframe_checkFrame(pTxFrame, (size_t)frameSize);
                if (ret != kErrorOk)
                    goto Exit;

                if (frameSize < C_DLL_MIN_ETH_FRAME)
                {
                    // Zero the frame buffer until minimum frame size to avoid
                    // relicts in padding area. The async Tx buffers are always
                    // allocated with maximum size, so we can zero safely.
                    OPLK_MEMSET((UINT8*)pTxFrame + frameSize,
                                0,
                                C_DLL_MIN_ETH_FRAME - frameSize);

                    frameSize = C_DLL_MIN_ETH_FRAME;
                }

                pTxBuffer->txFrameSize = (UINT)frameSize;    // set buffer valid
                *pTxBufferState = kDllkTxBufReady;

#if (CONFIG_EDRV_AUTO_RESPONSE != FALSE)
                if ((nmtState_p & (NMT_TYPE_MASK | NMT_SUPERSTATE_MASK)) == (NMT_TYPE_CS | NMT_CS_PLKMODE))
                {
                    ret = edrv_updateTxBuffer(pTxBuffer);

                    // enable corresponding Rx filter
                    dllkInstance_g.aFilter[filterEntry].fEnable = TRUE;
                    ret = edrv_changeRxFilter(dllkInstance_g.aFilter,
                                              DLLK_FILTER_COUNT,
                                              filterEntry,
                                              EDRV_FILTER_CHANGE_STATE);
                    if (ret != kErrorOk)
                        goto Exit;
                }
                else if ((nmtState_p & NMT_STATE_XX_MASK) < NMT_STATE_XX_PRE_OPERATIONAL_1)
                {
                    // frame is silently dropped for POWERLINK states < PreOp1
                    // to avoid deadlocks and higher layer confusions
                    *pTxBufferState = kDllkTxBufEmpty;
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
                    ret = edrv_changeRxFilter(dllkInstance_g.aFilter,
                                              DLLK_FILTER_COUNT,
                                              filterEntry,
                                              EDRV_FILTER_CHANGE_STATE);
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
        {   // frame is present - padding is done by Edrv or Ethernet controller
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
        {   // non-empty FIFO with highest priority is for NMT requests
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
                                             PLK_FRAME_FLAG2_PR) |
                                           (frameCount & PLK_FRAME_FLAG2_RS));
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

\param[in]      nmtState_p          NMT state of the node.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError processCycleFinish(tNmtState nmtState_p)
{
    tOplkError  ret = kErrorOk;

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

\param[in]      nmtState_p          NMT state of the node.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError processSync(tNmtState nmtState_p)
{
    tOplkError  ret = kErrorReject;
    BOOL        fReadyFlag = FALSE;

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

\param[in]      nmtState_p          NMT state of the node.
\param[in]      fReadyFlag_p        Status of the ready flag.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError processSyncCn(tNmtState nmtState_p, BOOL fReadyFlag_p)
{
    tOplkError      ret = kErrorOk;
    tPlkFrame*      pTxFrame;
    tEdrvTxBuffer*  pTxBuffer;
    tFrameInfo      frameInfo;
    UINT            nextTxBufferOffset = dllkInstance_g.curTxBufferOffsetCycle ^ 1;

    // local node is CN, update only the PRes
    pTxBuffer = &dllkInstance_g.pTxBuffer[DLLK_TXFRAME_PRES + nextTxBufferOffset];
    if (pTxBuffer->pBuffer != NULL)
    {   // PRes does exist
        pTxFrame = (tPlkFrame*)pTxBuffer->pBuffer;

        if (nmtState_p != kNmtCsOperational)
            fReadyFlag_p = FALSE;

        frameInfo.frame.pBuffer = pTxFrame;
        frameInfo.frameSize = (UINT)pTxBuffer->txFrameSize;
        ret = dllkframe_processTpdo(&frameInfo, fReadyFlag_p);
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

\param[in]      nmtState_p          NMT state of the node.
\param[in]      fReadyFlag_p        Status of the ready flag.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError processSyncMn(tNmtState nmtState_p, BOOL fReadyFlag_p)
{
    tOplkError      ret = kErrorOk;
    tPlkFrame*      pTxFrame;
    tEdrvTxBuffer*  pTxBuffer;
    UINT            index = 0;
    UINT32          nextTimeOffsetNs = 0;
    UINT            nextTxBufferOffset = dllkInstance_g.curTxBufferOffsetCycle ^ 1;
#if defined(CONFIG_INCLUDE_SOC_TIME_FORWARD)
    tNetTime*       pNetTime = &dllkInstance_g.socTime.netTime;   // Pointer to net time value
    BOOL            fNewData = FALSE;                             // Flag to indicate if new net time data is received from user layer
#endif

    pTxBuffer = &dllkInstance_g.pTxBuffer[DLLK_TXFRAME_SOC + nextTxBufferOffset];
    pTxBuffer->timeOffsetNs = nextTimeOffsetNs;
    pTxFrame = (tPlkFrame*)pTxBuffer->pBuffer;

#if defined(CONFIG_INCLUDE_SOC_TIME_FORWARD)
    // Forward SoC time information to timesync module. Note that this SoC time
    // info is sent after the current cycle is completed (due to double buffers)!
    ret = timesynck_setSocTime(&dllkInstance_g.socTime);
    if (ret != kErrorOk)
        return ret;

    // Get the net time information from userToKernel shared buffer
    ret = timesynck_getNetTime(pNetTime, &fNewData);
    if (ret != kErrorOk)
        return ret;

    if (fNewData)
    {
        // Set the flag to increment the net time value
        dllkInstance_g.fIncrementNetTime = TRUE;
    }

    // Increment net time if dllkInstance_g.fIncrementNetTime is TRUE
    if (dllkInstance_g.fIncrementNetTime)
        addNetTime(pNetTime, dllkInstance_g.cycleLength, pNetTime);

    // Set SoC net time
    ami_setUint32Le(&pTxFrame->data.soc.netTimeLe.nsec, dllkInstance_g.socTime.netTime.nsec);
    ami_setUint32Le(&pTxFrame->data.soc.netTimeLe.sec, dllkInstance_g.socTime.netTime.sec);
#endif

    // Set SoC relative time
    ami_setUint64Le(&pTxFrame->data.soc.relativeTimeLe, dllkInstance_g.socTime.relTime);
    dllkInstance_g.socTime.relTime += dllkInstance_g.dllConfigParam.cycleLen;

    if (!dllkInstance_g.socTime.fRelTimeValid)
    {
        // SoC time information is valid from now on...
        dllkInstance_g.socTime.fRelTimeValid = TRUE;
    }

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
    tOplkError  ret = kErrorOk;

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
                                    dllkframe_cbMnTimerCycle,
                                    0L,
                                    FALSE);
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

\param[in]      node_p              Node ID of node to request PRes forwarding.

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

#if (defined(CONFIG_INCLUDE_SOC_TIME_FORWARD) && defined(CONFIG_INCLUDE_NMT_MN))
//------------------------------------------------------------------------------
/**
\brief  Add net time value

This function increments the net time value with respect to cycle length and
returns the updated net time value.

\param[in]     pInputNetTime_p      Input pointer to net time structure.
\param[in]     cycleLength_p        Cycle length in tNetTime format.
\param[out]    pResultantNetTime_p  Resultant pointer to net time structure.

*/
//------------------------------------------------------------------------------
static void addNetTime(const tNetTime* pInputNetTime_p,
                       const tNetTime cycleLength_p,
                       tNetTime* pResultantNetTime_p)
{
    UINT32 netTimeNsec = 0;

    netTimeNsec = pInputNetTime_p->nsec + cycleLength_p.nsec;

    // Update the seconds value
    pResultantNetTime_p->sec = pInputNetTime_p->sec + cycleLength_p.sec +
                               (netTimeNsec / DLLK_SEC_TO_NSEC_FACTOR);
    // Update the nanoseconds value
    pResultantNetTime_p->nsec = netTimeNsec % DLLK_SEC_TO_NSEC_FACTOR;
}
#endif

/// \}
