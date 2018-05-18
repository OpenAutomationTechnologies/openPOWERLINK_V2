/**
********************************************************************************
\file   dllk-internal.h

\brief  Internal definitions for DLL kernel module files

This file contains internal definitions used by the DLL kernel implementation
files.

*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2015, SYSTEC electronic GmbH
Copyright (c) 2018, B&R Industrial Automation GmbH
Copyright (c) 2017, Kalycito Infotech Private Limited
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
#include <common/oplkinc.h>

#include <oplk/nmt.h>
#include <oplk/dll.h>
#include <kernel/edrv.h>
#include <kernel/dllk.h>
#include <kernel/dllkfilter.h>
#include <kernel/dllktgt.h>
#include <common/timer.h>
#include <common/timesync.h>

//------------------------------------------------------------------------------
// check for correct compilation options
//------------------------------------------------------------------------------

#if (defined(CONFIG_INCLUDE_NMT_MN) && (CONFIG_DLL_PRES_FILTER_COUNT == 0))
#error "MN support needs CONFIG_DLL_PRES_FILTER_COUNT != 0"
#endif

#if ((CONFIG_DLL_PRES_CHAINING_CN != FALSE) && (CONFIG_EDRV_AUTO_RESPONSE_DELAY == FALSE))
#error "Ethernet driver support for auto-response delay is required for PRes Chaining."
#endif

#if ((CONFIG_DLL_PRES_CHAINING_CN != FALSE) && (CONFIG_DLL_PROCESS_SYNC != DLL_PROCESS_SYNC_ON_TIMER))
#error "PRes Chaining CN support requires CONFIG_DLL_PROCESS_SYNC == DLL_PROCESS_SYNC_ON_TIMER."
#endif

#if (defined(CONFIG_INCLUDE_NMT_RMN) && CONFIG_TIMER_USE_HIGHRES == FALSE)
#error "RMN support needs CONFIG_TIMER_USE_HIGHRES != FALSE"
#endif

#if (defined(CONFIG_INCLUDE_NMT_RMN) && !defined(CONFIG_INCLUDE_NMT_MN))
#error "RMN support needs CONFIG_INCLUDE_NMT_MN"
#endif

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

// TracePoint support for realtime-debugging
#ifdef _DBG_TRACE_POINTS_
void target_signalTracePoint(UINT8 tracePointNumber_p);
#define TGT_DBG_SIGNAL_TRACE_POINT(p)   target_signalTracePoint(p)
#else
#define TGT_DBG_SIGNAL_TRACE_POINT(p)
#endif

// defines for indexes of tDllkInstance.pTxBuffer
#define DLLK_TXFRAME_IDENTRES       0   // IdentResponse on CN / MN
#define DLLK_TXFRAME_STATUSRES      2   // StatusResponse on CN / MN
#define DLLK_TXFRAME_NMTREQ         4   // NMT Request from FIFO on CN / MN

#if (CONFIG_DLL_PRES_CHAINING_CN != FALSE)
#define DLLK_TXFRAME_SYNCRES        6   // SyncResponse on CN
#define DLLK_TXFRAME_NONPLK         8   // non-POWERLINK frame from FIFO on CN / MN
#else
#define DLLK_TXFRAME_NONPLK         6   // non-POWERLINK frame from FIFO on CN / MN
#endif

#define DLLK_TXFRAME_PRES           (DLLK_TXFRAME_NONPLK + 2) // PRes on CN / MN

#if defined(CONFIG_INCLUDE_NMT_MN)
#define DLLK_TXFRAME_SOC            (DLLK_TXFRAME_PRES + 2)   // SoC on MN
#define DLLK_TXFRAME_SOA            (DLLK_TXFRAME_SOC + 2)    // SoA on MN
#define DLLK_TXFRAME_PREQ           (DLLK_TXFRAME_SOA + 2)    // PReq on MN
#if defined(CONFIG_INCLUDE_NMT_RMN)
#define DLLK_TXFRAME_AMNI           (DLLK_TXFRAME_PREQ + 2)   // AMNI on MN
#define DLLK_TXFRAME_COUNT          (DLLK_TXFRAME_AMNI + (2 * (NMT_MAX_NODE_ID + 2)))
                                    // on MN: 7 + MaxPReq of regular CNs + 1 Diag + 1 Router
#else
#define DLLK_TXFRAME_COUNT          (DLLK_TXFRAME_PREQ + (2 * (D_NMT_MaxCNNumber_U8 + 2)))
#endif
#else
#define DLLK_TXFRAME_COUNT          (DLLK_TXFRAME_PRES + 2)
#endif

#define DLLK_SOAREQ_COUNT           3

// defines for tDllkInstance.updateTxFrame
#define DLLK_UPDATE_NONE            0       // no update necessary
#define DLLK_UPDATE_STATUS          1       // StatusRes needs update
#define DLLK_UPDATE_BOTH            2       // IdentRes and StatusRes need update

// defines for tDllkNodeInfo.presFilterFlags
#define DLLK_FILTER_FLAG_PDO        0x01    // PRes needed for RPDO
#define DLLK_FILTER_FLAG_HB         0x02    // PRes needed for Heartbeat Consumer

//------------------------------------------------------------------------------
// typedef
//------------------------------------------------------------------------------

typedef enum
{
    kDllkTxBufEmpty      = 0,    // Tx buffer is empty
    kDllkTxBufFilling,           // just the buffer is being filled
    kDllkTxBufSending,           // the buffer is being transmitted
    kDllkTxBufReady,             // the buffer is ready for transmission
} eDllkTxBufState;

typedef UINT32 tDllkTxBufState;

/**
\brief Structure for handling the report of a loss of SoC to the error handler

A loss of SoC shall be reported only once per cycle to the error handler.
This structure controls the status of the reported errors in each cycle.
*/
typedef struct
{
    BOOL      fLossReported;        ///< A loss of SoC was already reported in this cycle
    BOOL      fTimeoutOccurred;     ///< The sync interrupt occurred after a report of a loss of SoC
} tDllLossSocStatus;

/**
\brief Structure for handling PRes forwarding to application

The structure is used by the PRes forwarding function to the application which
can be used for diagnosis purpose.
*/
typedef struct
{
    UINT8       numRequests;        ///< Forward request, incremented by call from app
    UINT8       numResponse;        ///< Forward response, incremented by Dllk layer
} tDllkPresFw;

/**
\brief Structure containing the DLLk instance information

This structure contains the data of a DLLk instance.
*/
typedef struct
{
    tNmtState               nmtState;                               ///< Current NMT state
    tTimesyncSocTime        socTime;                                ///< Current SoC Time
    tEdrvTxBuffer*          pTxBuffer;                              ///< Buffers for TX frames
    UINT                    maxTxFrames;                            ///< Max TX frames
    UINT8                   flag1;                                  ///< Flag 1 with EN, EC for PRes, StatusRes
    UINT8                   mnFlag1;                                ///< Flag 1 with MS, EA, ER from PReq, SoA of MN
    UINT8                   flag2;                                  ///< Flag 2 with PR and RS for PRes, StatusRes, IdentRes
    UINT8                   updateTxFrame;                          ///< Update TX frame
    UINT                    usedPresFilterCount;                    ///< Count of used PRes filters
    tDllConfigParam         dllConfigParam;                         ///< DLL configuration parameters
    tDllIdentParam          dllIdentParam;                          ///< DLL ident parameters
    tDllState               dllState;                               ///< Current DLL state
    tDllkCbProcessRpdo      pfnCbProcessRpdo;                       ///< Pointer to the RPDO process callback function
    tDllkCbProcessTpdo      pfnCbProcessTpdo;                       ///< Pointer to the TPDO process callback function
    tDllkCbAsync            pfnCbAsync;                             ///< Pointer to the asynchronous callback function
    tSyncCb                 pfnCbSync;                              ///< Pointer to the synchronous callback function
    tDllAsndFilter          aAsndFilter[DLL_MAX_ASND_SERVICE_ID];   ///< Array of ASnd filters
    tEdrvFilter             aFilter[DLLK_FILTER_COUNT];             ///< Array of Ethernet driver filters
#if (NMT_MAX_NODE_ID > 0)
    tDllkNodeInfo           aNodeInfo[NMT_MAX_NODE_ID];             ///< Array of node information structures
#endif
    UINT8                   curTxBufferOffsetIdentRes;              ///< Current TX buffer offset for IdentResponse frames
    UINT8                   curTxBufferOffsetStatusRes;             ///< Current TX buffer offset for StatusResponse frames
    UINT8                   curTxBufferOffsetNmtReq;                ///< Current TX buffer offset for NMT-priority frames
    UINT8                   curTxBufferOffsetNonPlk;                ///< Current TX buffer offset for non-POWERLINK frames
    UINT8                   curTxBufferOffsetCycle;                 ///< Current TX buffer offset for PRes, SoC, SoA, PReq
#if (CONFIG_DLL_PRES_CHAINING_CN != FALSE)
    UINT8                   curTxBufferOffsetSyncRes;               ///< Current TX buffer offset for SyncResponse frames
#endif
    tDllkTxBufState         aTxBufferStateNmtReq[2];
    tDllkTxBufState         aTxBufferStateNonPlk[2];
#if defined(CONFIG_INCLUDE_NMT_MN)
    tDllkNodeInfo*          pFirstNodeInfo;                         ///< Pointer to the first node information structure
    UINT8                   aCnNodeIdList[2][NMT_MAX_NODE_ID];      ///< Double-buffered node ID list
    UINT8                   curNodeIndex;                           ///< Current node index
    tEdrvTxBuffer**         ppTxBufferList;                         ///< Pointer to the TX buffer list
    UINT8                   syncLastSoaReq;                         ///< Sync last SoA request
    tDllReqServiceId        aLastReqServiceId[DLLK_SOAREQ_COUNT];   ///< Array of last requested service IDs
    UINT                    aLastTargetNodeId[DLLK_SOAREQ_COUNT];   ///< Array of last target node IDs
    UINT8                   curLastSoaReq;                          ///< Current last SoA request
    BOOL                    fSyncProcessed;                         ///< Sync is processed
    BOOL                    fPrcSlotFinished;                       ///< PRC slot is finished
    tDllkNodeInfo*          pFirstPrcNodeInfo;                      ///< Pointer to the first PRC node information structure
#endif

#if (CONFIG_TIMER_USE_HIGHRES != FALSE)
    tTimerHdl               timerHdlCycle;                          ///< Timer handle used for POWERLINK cycle monitoring on CN and generation on MN
#if defined(CONFIG_INCLUDE_NMT_RMN)
    tTimerHdl               timerHdlSwitchOver;                     ///< Timer used for monitoring of missing SoC/SoA/AMNI (Redundancy)
#endif
#if defined(CONFIG_INCLUDE_NMT_MN)
    tTimerHdl               timerHdlResponse;                       ///< Timer handle used for CN response monitoring
#endif
#endif

    UINT16                  prescaleCycleCount;                     ///< Cycle counter for toggling PS bit in MN SOC
    UINT                    cycleCount;                             ///< Cycle counter (needed for multiplexed cycle support)
    UINT64                  frameTimeout;                           ///< Frame timeout (cycle length + loss of frame tolerance)

    tDllLossSocStatus       lossSocStatus;                          ///< Loss of SoC status

#if (CONFIG_DLL_PRES_CHAINING_CN != FALSE)
    UINT                    syncReqPrevNodeId;                      ///< Node ID of the previous SyncRequest
    tTimestamp              syncReqPrevTimeStamp;                   ///< Timestamp of the previous SyncRequest
    BOOL                    fPrcEnabled;                            ///< PRC is enabled
    UINT32                  prcPResTimeFirst;                       ///< PRes time on the first communication path
    UINT32                  prcPResFallBackTimeout;                 ///< Timeout to fall back to PReq/PRes mode on the first communication path
#endif

#if (defined(CONFIG_INCLUDE_NMT_MN) && defined(CONFIG_INCLUDE_PRES_FORWARD))
    tDllkPresFw             aPresForward[NMT_MAX_NODE_ID];
#endif
#if defined(CONFIG_INCLUDE_NMT_RMN)
    BOOL                    fRedundancy;                            ///< Managing Node Redundancy is enabled
    tNmtEvent               nmtEventGoToStandby;                    ///< NMT command GoToStandby has been requested
#endif
#if (defined(CONFIG_INCLUDE_NMT_MN) && defined(CONFIG_INCLUDE_SOC_TIME_FORWARD))
    BOOL                    fIncrementNetTime;                      ///< Flag to increment net time
    tNetTime                cycleLength;                            ///< Cycle length in nano seconds and seconds
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
extern "C"
{
#endif

//------------------------------------------------------------------------------
/* Helper functions */
tOplkError dllk_postEvent(tEventType eventType_p);

#if defined(CONFIG_INCLUDE_NMT_RMN)
tOplkError dllk_cbTimerSwitchOver(const tTimerEventArg* pEventArg_p);
#endif

#ifdef __cplusplus
}
#endif

#endif /* _INC_dllk_internal_H_ */
