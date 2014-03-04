/**
********************************************************************************
\file   dllk-internal.h

\brief  Internal definitions for DLL kernel module files

This file contains internal definitions used by the DLL kernel implementation
files.

*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2013, SYSTEC electronic GmbH
Copyright (c) 2014, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
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

#ifndef _INC_dllk_internal_H_
#define _INC_dllk_internal_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------

#include <kernel/dllk.h>
#include <kernel/dllkcal.h>
#include <kernel/dllkfilter.h>
#include <kernel/eventk.h>
#include <kernel/errhndk.h>
#include <oplk/nmt.h>
#include <kernel/edrv.h>
#include <oplk/benchmark.h>

#if CONFIG_TIMER_USE_HIGHRES != FALSE
#include <kernel/hrestimer.h>
#endif

#include "kernel/dllktgt.h"

#if (CONFIG_DLL_PROCESS_SYNC == DLL_PROCESS_SYNC_ON_TIMER)
#include <kernel/synctimer.h>
#endif


//------------------------------------------------------------------------------
// check for correct compilation options
//------------------------------------------------------------------------------
#if (CONFIG_DLL_PRES_READY_AFTER_SOA != FALSE) && (CONFIG_DLL_PRES_READY_AFTER_SOC != FALSE)
#error "DLLK: select only one of CONFIG_DLL_PRES_READY_AFTER_SOA and CONFIG_DLL_PRES_READY_AFTER_SOC."
#endif

#if ((CONFIG_DLL_PRES_READY_AFTER_SOA != FALSE) || (CONFIG_DLL_PRES_READY_AFTER_SOC != FALSE)) &&  defined(CONFIG_INCLUDE_NMT_MN)
#error "DLLK: currently, CONFIG_DLL_PRES_READY_AFTER_* is not supported if CONFIG_INCLUDE_NMT_MN is enabled."
#endif

#if (CONFIG_EDRV_FAST_TXFRAMES == FALSE) && ((CONFIG_DLL_PRES_READY_AFTER_SOA != FALSE) || (CONFIG_DLL_PRES_READY_AFTER_SOC != FALSE))
#error "DLLK: CONFIG_DLL_PRES_READY_AFTER_* is enabled, but not CONFIG_EDRV_FAST_TXFRAMES."
#endif

#if defined(CONFIG_INCLUDE_NMT_MN) && (CONFIG_DLL_PRES_FILTER_COUNT == 0)
#error "MN support needs CONFIG_DLL_PRES_FILTER_COUNT != 0"
#endif

#if (CONFIG_DLL_PRES_CHAINING_CN != FALSE) && (CONFIG_EDRV_AUTO_RESPONSE_DELAY == FALSE)
#error "Ethernet driver support for auto-response delay is required for PRes Chaining."
#endif

#if (CONFIG_DLL_PRES_CHAINING_CN != FALSE) && (CONFIG_DLL_PROCESS_SYNC != DLL_PROCESS_SYNC_ON_TIMER)
#error "PRes Chaining CN support requires CONFIG_DLL_PROCESS_SYNC == DLL_PROCESS_SYNC_ON_TIMER."
#endif

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

// TracePoint support for realtime-debugging
#ifdef _DBG_TRACE_POINTS_
void  TgtDbgSignalTracePoint (BYTE bTracePointNumber_p);
void  TgtDbgPostTraceValue (DWORD dwTraceValue_p);
#define TGT_DBG_SIGNAL_TRACE_POINT(p)   TgtDbgSignalTracePoint(p)
#define TGT_DBG_POST_TRACE_VALUE(v)     TgtDbgPostTraceValue(v)
#else
#define TGT_DBG_SIGNAL_TRACE_POINT(p)
#define TGT_DBG_POST_TRACE_VALUE(v)
#endif

#define DLLK_DBG_POST_TRACE_VALUE(Event_p, uiNodeId_p, wErrorCode_p) \
    TGT_DBG_POST_TRACE_VALUE((kEventSinkDllk << 28) | (Event_p << 24) \
                             | (uiNodeId_p << 16) | wErrorCode_p)

// defines for indexes of tDllkInstance.pTxBuffer
#define DLLK_TXFRAME_IDENTRES       0   // IdentResponse on CN / MN
#define DLLK_TXFRAME_STATUSRES      2   // StatusResponse on CN / MN
#define DLLK_TXFRAME_NMTREQ         4   // NMT Request from FIFO on CN / MN

#if CONFIG_DLL_PRES_CHAINING_CN != FALSE
#define DLLK_TXFRAME_SYNCRES      6   // SyncResponse on CN
#define DLLK_TXFRAME_NONPLK       8   // non-POWERLINK frame from FIFO on CN / MN
#else
#define DLLK_TXFRAME_NONPLK       6   // non-POWERLINK frame from FIFO on CN / MN
#endif

#define DLLK_TXFRAME_PRES           (DLLK_TXFRAME_NONPLK + 2) // PRes on CN / MN

#if defined(CONFIG_INCLUDE_NMT_MN)
  #define DLLK_TXFRAME_SOC          (DLLK_TXFRAME_PRES + 2)   // SoC on MN
  #define DLLK_TXFRAME_SOA          (DLLK_TXFRAME_SOC + 2)    // SoA on MN
  #define DLLK_TXFRAME_PREQ         (DLLK_TXFRAME_SOA + 2)    // PReq on MN
  #define DLLK_TXFRAME_COUNT        (DLLK_TXFRAME_PREQ + (2 * (D_NMT_MaxCNNumber_U8 + 2)))
                                    // on MN: 7 + MaxPReq of regular CNs + 1 Diag + 1 Router
#else
  #define DLLK_TXFRAME_COUNT        (DLLK_TXFRAME_PRES + 2)
#endif

#define DLLK_SOAREQ_COUNT               3

// defines for tEdrvTxBuffer.m_uiTxBufLen
#define DLLK_BUFLEN_EMPTY               0   // buffer is empty
#define DLLK_BUFLEN_FILLING             1   // just the buffer is being filled
#define DLLK_BUFLEN_MIN                 60  // minimum ethernet frame length

// defines for tDllkInstance.updateTxFrame
#define DLLK_UPDATE_NONE                0   // no update necessary
#define DLLK_UPDATE_STATUS              1   // StatusRes needs update
#define DLLK_UPDATE_BOTH                2   // IdentRes and StatusRes need update

// defines for tDllkNodeInfo.presFilterFlags
#define DLLK_FILTER_FLAG_PDO            0x01    // PRes needed for RPDO
#define DLLK_FILTER_FLAG_HB             0x02    // PRes needed for Heartbeat Consumer

//------------------------------------------------------------------------------
// typedef
//------------------------------------------------------------------------------

/**
 * \brief Structure for handling the report of a loss of SoC to the error handler
 *
 * A loss of SoC shall be reported only once per cycle to the error handler.
 * This structure controls the status of the reported errors in each cycle.
 */
typedef struct
{
    BOOL      fLossReported;        ///< A loss of SoC was already reported in this cycle
    BOOL      fTimeoutOccurred;     ///< The sync interrupt occurred after a report of a loss of SoC
} tDllLossSocStatus;

typedef struct
{
    tNmtState               nmtState;
    UINT64                  relativeTime;
    UINT8                   aLocalMac[6];
    tEdrvTxBuffer*          pTxBuffer;                      // Buffers for Tx-Frames
    UINT                    maxTxFrames;
    UINT8                   flag1;                          // Flag 1 with EN, EC for PRes, StatusRes
    UINT8                   mnFlag1;                        // Flag 1 with MS, EA, ER from PReq, SoA of MN
    UINT8                   flag2;                          // Flag 2 with PR and RS for PRes, StatusRes, IdentRes
    UINT8                   updateTxFrame;
    UINT                    usedPresFilterCount;
    tDllConfigParam         dllConfigParam;
    tDllIdentParam          dllIdentParam;
    tDllState               dllState;
    tDllkCbProcessRpdo      pfnCbProcessRpdo;
    tDllkCbProcessTpdo      pfnCbProcessTpdo;
    tDllkCbAsync            pfnCbAsync;
    tSyncCb                 pfnCbSync;
    tDllAsndFilter          aAsndFilter[DLL_MAX_ASND_SERVICE_ID];
    tEdrvFilter             aFilter[DLLK_FILTER_COUNT];
#if NMT_MAX_NODE_ID > 0
    tDllkNodeInfo           aNodeInfo[NMT_MAX_NODE_ID];
#endif
    UINT8                   curTxBufferOffsetIdentRes;
    UINT8                   curTxBufferOffsetStatusRes;
    UINT8                   curTxBufferOffsetNmtReq;
    UINT8                   curTxBufferOffsetNonPlk;
    UINT8                   curTxBufferOffsetCycle;         // PRes, SoC, SoA, PReq
#if CONFIG_DLL_PRES_CHAINING_CN != FALSE
    UINT8                   curTxBufferOffsetSyncRes;
#endif

#if defined(CONFIG_INCLUDE_NMT_MN)
    tDllkNodeInfo*          pFirstNodeInfo;
    UINT8                   aCnNodeIdList[2][NMT_MAX_NODE_ID];
    UINT8                   curNodeIndex;
    tEdrvTxBuffer**         ppTxBufferList;
    UINT8                   syncLastSoaReq;
    tDllReqServiceId        aLastReqServiceId[DLLK_SOAREQ_COUNT];
    UINT                    aLastTargetNodeId[DLLK_SOAREQ_COUNT];
    UINT8                   curLastSoaReq;
    BOOL                    fSyncProcessed;
    BOOL                    fPrcSlotFinished;
    tDllkNodeInfo*          pFirstPrcNodeInfo;
#endif

#if CONFIG_TIMER_USE_HIGHRES != FALSE
    tTimerHdl               timerHdlCycle;                  // used for POWERLINK cycle monitoring on CN and generation on MN
#if defined(CONFIG_INCLUDE_NMT_MN)
    tTimerHdl               timerHdlResponse;               // used for CN response monitoring
#endif
#endif

    UINT                    prescaleCycleCount;             // cycle counter for toggling PS bit in MN SOC
    UINT                    cycleCount;                     // cycle counter (needed for multiplexed cycle support)
    UINT64                  frameTimeout;                   // frame timeout (cycle length + loss of frame tolerance)

    tDllLossSocStatus       lossSocStatus;

#if CONFIG_DLL_PRES_CHAINING_CN != FALSE
    UINT                    syncReqPrevNodeId;
    tTimestamp              syncReqPrevTimeStamp;
    BOOL                    fPrcEnabled;
    UINT32                  prcPResTimeFirst;
    UINT32                  prcPResFallBackTimeout;
#endif
} tDllkInstance;

//------------------------------------------------------------------------------
// global variable declarations
//------------------------------------------------------------------------------
extern tDllkInstance        dllkInstance_g;
TGT_DLLK_DECLARE_CRITICAL_SECTION

//------------------------------------------------------------------------------
// function prototypes
//------------------------------------------------------------------------------

#ifdef __cplusplus
extern "C" {
#endif

//------------------------------------------------------------------------------
/* Frame processing functions (dllkframe.c) */
tEdrvReleaseRxBuffer dllk_processFrameReceived(tEdrvRxBuffer * pRxBuffer_p) SECTION_DLLK_FRAME_RCVD_CB;
void       dllk_processTransmittedNmtReq(tEdrvTxBuffer * pTxBuffer_p) SECTION_DLLK_PROCESS_TX_NMT;
void       dllk_processTransmittedNonPlk(tEdrvTxBuffer* pTxBuffer_p) SECTION_DLLK_PROCESS_TX_NPLK;
#if defined(CONFIG_INCLUDE_NMT_MN)
void       dllk_processTransmittedSoc(tEdrvTxBuffer * pTxBuffer_p) SECTION_DLLK_PROCESS_TX_SOC;
void       dllk_processTransmittedSoa(tEdrvTxBuffer * pTxBuffer_p) SECTION_DLLK_PROCESS_TX_SOA;
#endif
tOplkError dllk_updateFrameIdentRes(tEdrvTxBuffer* pTxBuffer_p, tNmtState nmtState_p);
tOplkError dllk_updateFrameStatusRes(tEdrvTxBuffer* pTxBuffer_p, tNmtState NmtState_p);
tOplkError dllk_updateFramePres(tEdrvTxBuffer* pTxBuffer_p, tNmtState nmtState_p);
tOplkError dllk_checkFrame(tPlkFrame * pFrame_p, UINT frameSize_p);
tOplkError dllk_createTxFrame(UINT* pHandle_p, UINT* pFrameSize_p,
                              tMsgType msgType_p, tDllAsndServiceId serviceId_p);
tOplkError dllk_deleteTxFrame(UINT handle_p);
tOplkError dllk_processTpdo(tFrameInfo* pFrameInfo_p, BOOL fReadyFlag_p);
#if defined(CONFIG_INCLUDE_NMT_MN)
tOplkError dllk_mnSendSoa(tNmtState nmtState_p, tDllState* pDllStateProposed_p,
                          BOOL fEnableInvitation_p);
tOplkError dllk_updateFrameSoa(tEdrvTxBuffer* pTxBuffer_p, tNmtState NmtState_p,
                               BOOL fEnableInvitation_p, BYTE curReq_p) SECTION_DLLK_FRAME_UPDATE_SOA;
tOplkError dllk_asyncFrameNotReceived(tDllReqServiceId reqServiceId_p, UINT nodeId_p) SECTION_DLLK_FRAME_ASYNC_NRX;
#endif

//------------------------------------------------------------------------------
/* event functions (dllkevent.c) */
tOplkError dllk_postEvent(tEventType EventType_p);
tOplkError controlPdokcalSync(BOOL fEnable_p);
#if defined(CONFIG_INCLUDE_NMT_MN)
tOplkError dllk_issueLossOfPres(UINT nodeId_p);
#endif

//------------------------------------------------------------------------------
/* DLL filter functions (dllkfilter.c) */
void       dllk_setupAsndFilter(tEdrvFilter* pFilter_p);
void       dllk_setupSocFilter(tEdrvFilter* pFilter_p);
void       dllk_setupSoaFilter(tEdrvFilter* pFilter_p);
void       dllk_setupSoaIdentReqFilter(tEdrvFilter* pFilter_p, UINT nodeId_p, tEdrvTxBuffer *pBuffer_p);
void       dllk_setupSoaStatusReqFilter(tEdrvFilter* pFilter_p, UINT nodeId_p, tEdrvTxBuffer *pBuffer_p);
void       dllk_setupSoaNmtReqFilter(tEdrvFilter* pFilter_p, UINT nodeId_p, tEdrvTxBuffer *pBuffer_p);
#if CONFIG_DLL_PRES_CHAINING_CN != FALSE
void       dllk_setupSoaSyncReqFilter(tEdrvFilter* pFilter_p, UINT nodeId_p, tEdrvTxBuffer *pBuffer_p);
#endif
void       dllk_setupSoaUnspecReqFilter(tEdrvFilter* pFilter_p, UINT nodeId_p, tEdrvTxBuffer *pBuffer_p);
void       dllk_setupPresFilter(tEdrvFilter* pFilter_p, BOOL fEnable_p);
void       dllk_setupPreqFilter(tEdrvFilter* pFilter_p, UINT nodeId_p, tEdrvTxBuffer *pBuffer_p, UINT8* pMacAdrs_p);
#if NMT_MAX_NODE_ID > 0
tOplkError dllk_addNodeFilter(tDllkNodeInfo* pIntNodeInfo_p, tDllNodeOpType NodeOpType_p, BOOL fUpdateEdrv_p);
tOplkError dllk_deleteNodeFilter(tDllkNodeInfo* pIntNodeInfo_p, tDllNodeOpType nodeOpType_p, BOOL fUpdateEdrv_p);
#endif

//------------------------------------------------------------------------------
/* DLL state machine functions (dllkstatemachine.c) */
tOplkError dllk_changeState(tNmtEvent nmtEvent_p, tNmtState nmtState_p) SECTION_DLLK_CHANGE_STATE;

//------------------------------------------------------------------------------
/* node functions */
tOplkError dllk_cleanupLocalNode(tNmtState oldNmtState_p);
tOplkError dllk_setupLocalNode(tNmtState nmtState_p);
tOplkError dllk_setupLocalNodeCn(void);
#if defined(CONFIG_INCLUDE_NMT_MN)
tOplkError dllk_setupLocalNodeMn(void);
tOplkError dllk_addNodeIsochronous(tDllkNodeInfo* pIntNodeInfo_p);
tOplkError dllk_deleteNodeIsochronous(tDllkNodeInfo* pIntNodeInfo_p);
tOplkError dllk_setupAsyncPhase(tNmtState nmtState_p, UINT nextTxBufferOffset_p,
                                UINT32 nextTimeOffsetNs_p, UINT* pIndex_p) SECTION_DLLK_PROCESS_SYNC;
tOplkError dllk_setupSyncPhase(tNmtState nmtState_p, BOOL fReadyFlag_p, UINT nextTxBufferOffset_p,
                               UINT32* pNextTimeOffsetNs_p, UINT* pIndex_p) SECTION_DLLK_PROCESS_SYNC;
#endif
#if NMT_MAX_NODE_ID > 0
tDllkNodeInfo* dllk_getNodeInfo(UINT uiNodeId_p);
#endif

//------------------------------------------------------------------------------
/* Cycle/Sync Callback functions */
#if defined(CONFIG_INCLUDE_NMT_MN)
tOplkError dllk_cbCyclicError(tOplkError errorCode_p, tEdrvTxBuffer * pTxBuffer_p);
tOplkError dllk_cbMnSyncHandler(void) SECTION_DLLK_MN_SYNC_CB;
tOplkError dllk_cbMnTimerCycle(tTimerEventArg* pEventArg_p);
#endif
#if CONFIG_TIMER_USE_HIGHRES != FALSE
tOplkError dllk_cbCnTimer(tTimerEventArg* pEventArg_p);
#endif
#if (CONFIG_DLL_PROCESS_SYNC == DLL_PROCESS_SYNC_ON_TIMER)
tOplkError dllk_cbCnTimerSync(void);
tOplkError dllk_cbCnLossOfSync(void);
#endif

//------------------------------------------------------------------------------
/* PRes Chaining functions */
#if CONFIG_DLL_PRES_CHAINING_CN == TRUE
tOplkError dllk_presChainingEnable (void);
tOplkError dllk_presChainingDisable (void);
#if (CONFIG_DLL_PROCESS_SYNC == DLL_PROCESS_SYNC_ON_TIMER)
tOplkError dllk_cbCnPresFallbackTimeout(void);
#endif
#endif

#ifdef __cplusplus
}
#endif


#endif /* _INC_dllk_internal_H_ */

