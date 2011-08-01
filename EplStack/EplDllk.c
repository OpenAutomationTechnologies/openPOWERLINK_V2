/****************************************************************************

  (c) SYSTEC electronic GmbH, D-07973 Greiz, August-Bebel-Str. 29
      www.systec-electronic.com

  Project:      openPOWERLINK

  Description:  source file for kernel DLL module

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

  2006/06/12 d.k.:   start of the implementation, version 1.00

****************************************************************************/

#include "kernel/EplDllk.h"
#include "kernel/EplDllkCal.h"
#include "kernel/EplEventk.h"
#include "kernel/EplErrorHandlerk.h"
#include "EplNmt.h"
#include "edrv.h"
#include "Benchmark.h"

//#if EPL_TIMER_USE_HIGHRES != FALSE
#include "kernel/EplTimerHighResk.h"
//#endif


#if(((EPL_MODULE_INTEGRATION) & (EPL_MODULE_DLLK)) != 0)

#include "kernel/EplDllkTgt.h"

#if (EPL_DLL_PROCESS_SYNC == EPL_DLL_PROCESS_SYNC_ON_TIMER)
#include "kernel/EplTimerSynck.h"
#endif

#if (EPL_DLL_PRES_READY_AFTER_SOA != FALSE) && (EPL_DLL_PRES_READY_AFTER_SOC != FALSE)
#error "EPL module DLLK: select only one of EPL_DLL_PRES_READY_AFTER_SOA and EPL_DLL_PRES_READY_AFTER_SOC."
#endif

#if ((EPL_DLL_PRES_READY_AFTER_SOA != FALSE) || (EPL_DLL_PRES_READY_AFTER_SOC != FALSE)) \
    && (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) == 0)
#error "EPL module DLLK: currently, EPL_DLL_PRES_READY_AFTER_* is not supported if EPL_MODULE_NMT_MN is enabled."
#endif

#if (EDRV_FAST_TXFRAMES == FALSE) && \
    ((EPL_DLL_PRES_READY_AFTER_SOA != FALSE) || (EPL_DLL_PRES_READY_AFTER_SOC != FALSE))
#error "EPL module DLLK: EPL_DLL_PRES_READY_AFTER_* is enabled, but not EDRV_FAST_TXFRAMES."
#endif

#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0) \
    && (EPL_DLL_PRES_FILTER_COUNT == 0)
#error "MN support needs EPL_DLL_PRES_FILTER_COUNT != 0"
#endif

#if (EPL_DLL_PRES_CHAINING_CN != FALSE) && (EDRV_AUTO_RESPONSE_DELAY == FALSE)
#error "Ethernet driver support for auto-response delay is required for PRes Chaining."
#endif

#if (EPL_DLL_PRES_CHAINING_CN != FALSE) && (EPL_DLL_PROCESS_SYNC != EPL_DLL_PROCESS_SYNC_ON_TIMER)
#error "PRes Chaining CN support requires EPL_DLL_PROCESS_SYNC == EPL_DLL_PROCESS_SYNC_ON_TIMER."
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

// TracePoint support for realtime-debugging
#ifdef _DBG_TRACE_POINTS_
    void  PUBLIC  TgtDbgSignalTracePoint (BYTE bTracePointNumber_p);
    void  PUBLIC  TgtDbgPostTraceValue (DWORD dwTraceValue_p);
    #define TGT_DBG_SIGNAL_TRACE_POINT(p)   TgtDbgSignalTracePoint(p)
    #define TGT_DBG_POST_TRACE_VALUE(v)     TgtDbgPostTraceValue(v)
#else
    #define TGT_DBG_SIGNAL_TRACE_POINT(p)
    #define TGT_DBG_POST_TRACE_VALUE(v)
#endif
#define EPL_DLLK_DBG_POST_TRACE_VALUE(Event_p, uiNodeId_p, wErrorCode_p) \
    TGT_DBG_POST_TRACE_VALUE((kEplEventSinkDllk << 28) | (Event_p << 24) \
                             | (uiNodeId_p << 16) | wErrorCode_p)


/***************************************************************************/
/*                                                                         */
/*                                                                         */
/*          C L A S S  EplDllk                                             */
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
//          P R I V A T E   D E F I N I T I O N S                          //
//                                                                         //
//=========================================================================//

//---------------------------------------------------------------------------
// const defines
//---------------------------------------------------------------------------

// the EdrvCyclic module is recommended for MN implementations;
// disabling the EdrvCyclic module is deprecated
#ifndef EPL_DLL_DISABLE_EDRV_CYCLIC
#define EPL_DLL_DISABLE_EDRV_CYCLIC     FALSE
#endif


// defines for indexes of tEplDllInstance.m_pTxFrameInfo
#define EPL_DLLK_TXFRAME_IDENTRES   0   // IdentResponse on CN / MN
#define EPL_DLLK_TXFRAME_STATUSRES  2   // StatusResponse on CN / MN
#define EPL_DLLK_TXFRAME_NMTREQ     4   // NMT Request from FIFO on CN / MN
#if EPL_DLL_PRES_CHAINING_CN != FALSE
  #define EPL_DLLK_TXFRAME_SYNCRES  6   // SyncResponse on CN
  #define EPL_DLLK_TXFRAME_NONEPL   8   // non-EPL frame from FIFO on CN / MN
#else
  #define EPL_DLLK_TXFRAME_NONEPL   6   // non-EPL frame from FIFO on CN / MN
#endif
#define EPL_DLLK_TXFRAME_PRES       (EPL_DLLK_TXFRAME_NONEPL + 2) // PRes on CN / MN
#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)
  #define EPL_DLLK_TXFRAME_SOC      (EPL_DLLK_TXFRAME_PRES + 2)   // SoC on MN
  #define EPL_DLLK_TXFRAME_SOA      (EPL_DLLK_TXFRAME_SOC + 2)    // SoA on MN
  #define EPL_DLLK_TXFRAME_PREQ     (EPL_DLLK_TXFRAME_SOA + 2)    // PReq on MN
  #define EPL_DLLK_TXFRAME_COUNT    (EPL_DLLK_TXFRAME_PREQ + (2 * (EPL_D_NMT_MaxCNNumber_U8 + 2)))
                                    // on MN: 7 + MaxPReq of regular CNs + 1 Diag + 1 Router
#else
  #define EPL_DLLK_TXFRAME_COUNT    (EPL_DLLK_TXFRAME_PRES + 2)
#endif


#define EPL_DLLK_FILTER_PREQ          0
#define EPL_DLLK_FILTER_SOA_IDREQ     1
#define EPL_DLLK_FILTER_SOA_STATREQ   2
#define EPL_DLLK_FILTER_SOA_NMTREQ    3
#if EPL_DLL_PRES_CHAINING_CN != FALSE
  #define EPL_DLLK_FILTER_SOA_SYNCREQ 4
  #define EPL_DLLK_FILTER_SOA_NONEPL  5
#else
  #define EPL_DLLK_FILTER_SOA_NONEPL  4
#endif

#define EPL_DLLK_FILTER_SOA           (EPL_DLLK_FILTER_SOA_NONEPL + 1)
#define EPL_DLLK_FILTER_SOC           (EPL_DLLK_FILTER_SOA + 1)
#define EPL_DLLK_FILTER_ASND          (EPL_DLLK_FILTER_SOC + 1)
#define EPL_DLLK_FILTER_PRES          (EPL_DLLK_FILTER_ASND + 1)

#if EPL_DLL_PRES_FILTER_COUNT < 0
  #define EPL_DLLK_FILTER_COUNT       (EPL_DLLK_FILTER_PRES + 1)
#else
  #define EPL_DLLK_FILTER_COUNT       (EPL_DLLK_FILTER_PRES + EPL_DLL_PRES_FILTER_COUNT)
#endif

#define EPL_DLLK_SOAREQ_COUNT       3

// defines for tEdrvTxBuffer.m_uiTxBufLen
#define EPL_DLLK_BUFLEN_EMPTY       0   // buffer is empty
#define EPL_DLLK_BUFLEN_FILLING     1   // just the buffer is being filled
#define EPL_DLLK_BUFLEN_MIN         60  // minimum ethernet frame length

// defines for tEplDllkInstance.m_bUpdateTxFrame
#define EPL_DLLK_UPDATE_NONE        0   // no update necessary
#define EPL_DLLK_UPDATE_STATUS      1   // StatusRes needs update
#define EPL_DLLK_UPDATE_BOTH        2   // IdentRes and StatusRes need update

// defines for tEplDllNodeInfo.m_bPresFilterFlags
#define EPL_DLLK_FILTER_FLAG_PDO    0x01    // PRes needed for RPDO
#define EPL_DLLK_FILTER_FLAG_HB     0x02    // PRes needed for Heartbeat Consumer


//---------------------------------------------------------------------------
// local types
//---------------------------------------------------------------------------

typedef enum
{
    kEplDllGsInit           = 0x00, // MN/CN: initialisation (< PreOp2)
    kEplDllCsWaitPreq       = 0x01, // CN: wait for PReq frame
    kEplDllCsWaitSoc        = 0x02, // CN: wait for SoC frame
    kEplDllCsWaitSoa        = 0x03, // CN: wait for SoA frame
    kEplDllMsNonCyclic      = 0x04, // MN: reduced EPL cycle (PreOp1)
    kEplDllMsWaitSocTrig    = 0x05, // MN: wait for SoC trigger (cycle timer)
    kEplDllMsWaitPreqTrig   = 0x06, // MN: wait for (first) PReq trigger (WaitSoCPReq_U32)
    kEplDllMsWaitPres       = 0x07, // MN: wait for PRes frame from CN
    kEplDllMsWaitSoaTrig    = 0x08, // MN: wait for SoA trigger (PRes transmitted)
    kEplDllMsWaitAsndTrig   = 0x09, // MN: wait for ASnd trigger (SoA transmitted)
    kEplDllMsWaitAsnd       = 0x0A, // MN: wait for ASnd frame if SoA contained invitation

} tEplDllState;


typedef struct
{
    tEplNmtState        m_NmtState;

    BYTE                m_be_abLocalMac[6];
    tEdrvTxBuffer*      m_pTxBuffer;        // Buffers for Tx-Frames
    unsigned int        m_uiMaxTxFrames;
    BYTE                m_bFlag1;           // Flag 1 with EN, EC for PRes, StatusRes
    BYTE                m_bMnFlag1;         // Flag 1 with EA, ER from PReq, SoA of MN
    BYTE                m_bFlag2;           // Flag 2 with PR and RS for PRes, StatusRes, IdentRes
    BYTE                m_bUpdateTxFrame;
    unsigned int        m_uiUsedPresFilterCount;
    tEplDllConfigParam  m_DllConfigParam;
    tEplDllIdentParam   m_DllIdentParam;
    tEplDllState        m_DllState;
    tEplDllkCbProcessRpdo   m_pfnCbProcessRpdo;
    tEplDllkCbProcessTpdo   m_pfnCbProcessTpdo;
    tEplDllkCbAsync     m_pfnCbAsync;
    tEplSyncCb          m_pfnCbSync;
    tEplDllAsndFilter   m_aAsndFilter[EPL_DLL_MAX_ASND_SERVICE_ID];

    tEdrvFilter         m_aFilter[EPL_DLLK_FILTER_COUNT];

    tEplDllkNodeInfo    m_aNodeInfo[EPL_NMT_MAX_NODE_ID];

    BYTE                m_bCurTxBufferOffsetIdentRes;
    BYTE                m_bCurTxBufferOffsetStatusRes;
    BYTE                m_bCurTxBufferOffsetNmtReq;
    BYTE                m_bCurTxBufferOffsetNonEpl;
    BYTE                m_bCurTxBufferOffsetCycle;      // PRes, SoC, SoA, PReq
#if EPL_DLL_PRES_CHAINING_CN != FALSE
    BYTE                m_bCurTxBufferOffsetSyncRes;
#endif
#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)
    tEplDllkNodeInfo*   m_pFirstNodeInfo;
    BYTE                m_aabCnNodeIdList[2][EPL_NMT_MAX_NODE_ID];
#if (EPL_DLL_DISABLE_EDRV_CYCLIC != FALSE)
    tEplDllkNodeInfo*   m_pCurNodeInfo;
#else
    BYTE                m_bCurNodeIndex;
    tEdrvTxBuffer**     m_ppTxBufferList;
    BYTE                m_bSyncLastSoaReq;
#endif
    tEplDllReqServiceId m_aLastReqServiceId[EPL_DLLK_SOAREQ_COUNT];
    unsigned int        m_auiLastTargetNodeId[EPL_DLLK_SOAREQ_COUNT];
    BYTE                m_bCurLastSoaReq;
    BOOL                m_fSyncProcessed;
#if EPL_DLL_PRES_CHAINING_MN != FALSE
    BOOL                m_fPrcSlotFinished;
    tEplDllkNodeInfo*   m_pFirstPrcNodeInfo;
#endif
#endif

#if EPL_TIMER_USE_HIGHRES != FALSE
    tEplTimerHdl        m_TimerHdlCycle;    // used for EPL cycle monitoring on CN and generation on MN
#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)
    tEplTimerHdl        m_TimerHdlResponse; // used for CN response monitoring
#endif // (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)
#endif

    unsigned int        m_uiCycleCount;     // cycle counter (needed for multiplexed cycle support)
    unsigned long long  m_ullFrameTimeout;  // frame timeout (cycle length + loss of frame tolerance)

#if EPL_DLL_PRES_CHAINING_CN != FALSE
    unsigned int        m_uiSyncReqPrevNodeId;
    tEplTgtTimeStamp*   m_pSyncReqPrevTimeStamp;

    BOOL                m_fPrcEnabled;
    DWORD               m_dwPrcPResTimeFirst;
    DWORD               m_dwPrcPResFallBackTimeout;
#endif

} tEplDllkInstance;


//---------------------------------------------------------------------------
// local vars
//---------------------------------------------------------------------------

// if no dynamic memory allocation shall be used
// define structures statically
static tEplDllkInstance     EplDllkInstance_g;

static tEdrvTxBuffer        aEplDllkTxBuffer_l[EPL_DLLK_TXFRAME_COUNT];

TGT_DLLK_DECLARE_CRITICAL_SECTION;


//---------------------------------------------------------------------------
// local function prototypes
//---------------------------------------------------------------------------

// process the create event
static tEplKernel EplDllkProcessCreate(tEplNmtState NmtState_p);

// process the destroy event
static tEplKernel EplDllkProcessDestroy(tEplNmtState OldNmtState_p);

// process NMT state changes
static tEplKernel EplDllkProcessNmtStateChange(tEplNmtState NewNmtState_p, tEplNmtState OldNmtState_p);

// process the NMT event
static tEplKernel EplDllkProcessNmtEvent(tEplEvent * pEvent_p);

// process the CycleFinish event
static tEplKernel EplDllkProcessCycleFinish(tEplNmtState NmtState_p);

// process the Sync event
static tEplKernel EplDllkProcessSync(tEplNmtState NmtState_p);

// process the FillTx event
static tEplKernel EplDllkProcessFillTx(tEplDllAsyncReqPriority AsyncReqPriority_p, tEplNmtState NmtState_p);

// change DLL state on event
static tEplKernel EplDllkChangeState(tEplNmtEvent NmtEvent_p, tEplNmtState NmtState_p);

// called from EdrvInterruptHandler()
static tEdrvReleaseRxBuffer EplDllkCbFrameReceived(tEdrvRxBuffer * pRxBuffer_p);

static tEplKernel EplDllkProcessReceivedPreq(tEplFrameInfo* pFrameInfo_p, tEplNmtState NmtState_p, tEdrvReleaseRxBuffer* pReleaseRxBuffer_p);
static tEplKernel EplDllkProcessReceivedPres(tEplFrameInfo* pFrameInfo_p, tEplNmtState NmtState_p, tEplNmtEvent* pNmtEvent_p, tEdrvReleaseRxBuffer* pReleaseRxBuffer_p);
static tEplKernel EplDllkProcessReceivedSoc(tEdrvRxBuffer* pRxBuffer_p, tEplNmtState NmtState_p);
static tEplKernel EplDllkProcessReceivedSoa(tEdrvRxBuffer* pRxBuffer_p, tEplNmtState NmtState_p);
static tEplKernel EplDllkProcessReceivedAsnd(tEplFrameInfo* pFrameInfo_p, tEdrvRxBuffer* pRxBuffer_p, tEplNmtState NmtState_p);

// called from EdrvInterruptHandler()
static void EplDllkCbTransmittedNmtReq(tEdrvTxBuffer * pTxBuffer_p);
static void EplDllkCbTransmittedNonEpl(tEdrvTxBuffer * pTxBuffer_p);
#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)
static void EplDllkCbTransmittedSoc(tEdrvTxBuffer * pTxBuffer_p);
static void EplDllkCbTransmittedSoa(tEdrvTxBuffer * pTxBuffer_p);
#if EPL_DLL_DISABLE_EDRV_CYCLIC == FALSE
static tEplKernel EplDllkCbCyclicError(tEplKernel ErrorCode_p, tEdrvTxBuffer * pTxBuffer_p);
static tEplKernel EplDllkCbMnSyncHandler(void);
#else
static void EplDllkCbTransmittedPreq(tEdrvTxBuffer * pTxBuffer_p);
#endif
#endif

// post event to DLL itself
static tEplKernel EplDllkPostEvent(tEplEventType EventType_p);

// forward RPDO to callback function
static tEplKernel EplDllkForwardRpdo(tEplFrameInfo * pFrameInfo_p);

// forward TPDO to callback function
static tEplKernel EplDllkProcessTpdo(tEplFrameInfo * pFrameInfo_p, BOOL fReadyFlag_p);

// update IdentRes frame
static tEplKernel EplDllkUpdateFrameIdentRes(tEdrvTxBuffer* pTxBuffer_p, tEplNmtState NmtState_p);

// update StatusRes frame
static tEplKernel EplDllkUpdateFrameStatusRes(tEdrvTxBuffer* pTxBuffer_p, tEplNmtState NmtState_p);

// update PRes frame
static tEplKernel EplDllkUpdateFramePres(tEdrvTxBuffer* pTxBuffer_p, tEplNmtState NmtState_p);

// creates the buffer for a Tx frame and registers it to the ethernet driver
static tEplKernel EplDllkCreateTxFrame(unsigned int * puiHandle_p,
                                unsigned int * puiFrameSize_p,
                                tEplMsgType MsgType_p,
                                tEplDllAsndServiceId ServiceId_p);

static tEplKernel EplDllkDeleteTxFrame(unsigned int uiHandle_p);

// check frame and set missing information
static tEplKernel EplDllkCheckFrame(tEplFrame * pFrame_p, unsigned int uiFrameSize_p);

// called by high resolution timer module to monitor EPL cycle as CN
#if EPL_TIMER_USE_HIGHRES != FALSE
static tEplKernel PUBLIC EplDllkCbCnTimer(tEplTimerEventArg* pEventArg_p);
#endif

#if (EPL_DLL_PROCESS_SYNC == EPL_DLL_PROCESS_SYNC_ON_TIMER)
static tEplKernel PUBLIC EplDllkCbCnTimerSync(void);

static tEplKernel PUBLIC EplDllkCbCnLossOfSync(void);
#endif


#if EPL_NMT_MAX_NODE_ID > 0

// MN: returns internal node info structure
static tEplDllkNodeInfo* EplDllkGetNodeInfo(unsigned int uiNodeId_p);

static tEplKernel EplDllkAddNodeFilter(tEplDllkNodeInfo* pIntNodeInfo_p, tEplDllNodeOpType NodeOpType_p, BOOL fUpdateEdrv_p);

static tEplKernel EplDllkDeleteNodeFilter(tEplDllkNodeInfo* pIntNodeInfo_p, tEplDllNodeOpType NodeOpType_p, BOOL fUpdateEdrv_p);

#endif


#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)

static tEplKernel EplDllkAddNodeIsochronous(tEplDllkNodeInfo* pIntNodeInfo_p);

static tEplKernel EplDllkDeleteNodeIsochronous(tEplDllkNodeInfo* pIntNodeInfo_p);

static tEplKernel EplDllkIssueLossOfPres(unsigned int uiNodeId_p);

// process StartReducedCycle event
static tEplKernel EplDllkProcessStartReducedCycle(tEplEvent * pEvent_p);

// update SoA frame
static tEplKernel EplDllkUpdateFrameSoa(tEdrvTxBuffer* pTxBuffer_p, tEplNmtState NmtState_p, BOOL fEnableInvitation_p, BYTE bCurReq_p);

// transmit SoA
static tEplKernel EplDllkMnSendSoa(tEplNmtState NmtState_p,
                                   tEplDllState* pDllStateProposed_p,
                                   BOOL fEnableInvitation_p);

#if EPL_DLL_DISABLE_EDRV_CYCLIC != FALSE
static tEplKernel EplDllkMnSendSoc(void);

static tEplKernel EplDllkMnSendPreq(tEplNmtState NmtState_p,
                                    tEplDllState* pDllStateProposed_p);
#endif

static tEplKernel EplDllkAsyncFrameNotReceived(tEplDllReqServiceId ReqServiceId_p, unsigned int uiNodeId_p);

static tEplKernel PUBLIC EplDllkCbMnTimerCycle(tEplTimerEventArg* pEventArg_p);

#if EPL_DLL_DISABLE_EDRV_CYCLIC != FALSE
static tEplKernel PUBLIC EplDllkCbMnTimerResponse(tEplTimerEventArg* pEventArg_p);
#endif

#endif


#if EPL_DLL_PRES_CHAINING_CN != FALSE

static tEplKernel EplDllkPResChainingEnable (void);
static tEplKernel EplDllkPResChainingDisable (void);

#if (EPL_DLL_PROCESS_SYNC == EPL_DLL_PROCESS_SYNC_ON_TIMER)
static tEplKernel PUBLIC EplDllkCbCnPResFallBackTimeout(void);
#endif

#endif


//=========================================================================//
//                                                                         //
//          P U B L I C   F U N C T I O N S                                //
//                                                                         //
//=========================================================================//

//---------------------------------------------------------------------------
//
// Function:    EplDllkAddInstance()
//
// Description: add and initialize new instance of EPL stack
//
// Parameters:  pInitParam_p            = initialisation parameters like MAC address
//
// Returns:     tEplKernel              = error code
//
//
// State:
//
//---------------------------------------------------------------------------

tEplKernel EplDllkAddInstance(tEplDllkInitParam * pInitParam_p)
{
tEplKernel      Ret = kEplSuccessful;
unsigned int    uiIndex;
tEdrvInitParam  EdrvInitParam;

    // reset instance structure
    EPL_MEMSET(&EplDllkInstance_g, 0, sizeof (EplDllkInstance_g));

#if EPL_TIMER_USE_HIGHRES != FALSE
    Ret = EplTimerHighReskInit();
    if (Ret != kEplSuccessful)
    {   // error occurred while initializing high resolution timer module
        goto Exit;
    }
#endif

#if (EPL_DLL_PROCESS_SYNC == EPL_DLL_PROCESS_SYNC_ON_TIMER)
    Ret = EplTimerSynckAddInstance();
    if (Ret != kEplSuccessful)
    {   // error occurred while initializing sync timer module
        goto Exit;
    }

    Ret = EplTimerSynckRegSyncHandler(EplDllkCbCnTimerSync);
    if (Ret != kEplSuccessful)
    {
        goto Exit;
    }

    Ret = EplTimerSynckRegLossOfSyncHandler(EplDllkCbCnLossOfSync);
    if (Ret != kEplSuccessful)
    {
        goto Exit;
    }

#if EPL_DLL_PRES_CHAINING_CN != FALSE
    Ret = EplTimerSynckRegLossOfSyncHandler2(EplDllkCbCnPResFallBackTimeout);
    if (Ret != kEplSuccessful)
    {
        goto Exit;
    }
#endif

    Ret = EplTimerSynckSetSyncShiftUs(EPL_DLL_SOC_SYNC_SHIFT_US);
    if (Ret != kEplSuccessful)
    {
        goto Exit;
    }

#endif

    // if dynamic memory allocation available
    // allocate instance structure

    // initialize and link pointers in instance structure to frame tables
    EplDllkInstance_g.m_pTxBuffer = aEplDllkTxBuffer_l;
    EplDllkInstance_g.m_uiMaxTxFrames = sizeof (aEplDllkTxBuffer_l) / sizeof (tEdrvTxBuffer);

    // initialize state
    EplDllkInstance_g.m_DllState = kEplDllGsInit;

#if EPL_NMT_MAX_NODE_ID > 0
    // set up node info structure
    for (uiIndex = 0; uiIndex < tabentries (EplDllkInstance_g.m_aNodeInfo); uiIndex++)
    {
        EplDllkInstance_g.m_aNodeInfo[uiIndex].m_uiNodeId = uiIndex + 1;
    }
#endif

    // initialize Edrv
    EPL_MEMCPY(EdrvInitParam.m_abMyMacAddr, pInitParam_p->m_be_abLocalMac, 6);
    EdrvInitParam.m_HwParam = pInitParam_p->m_HwParam;
    EdrvInitParam.m_pfnRxHandler = EplDllkCbFrameReceived;
//    EdrvInitParam.m_pfnTxHandler = EplDllkCbFrameTransmitted;
    Ret = EdrvInit(&EdrvInitParam);
    if (Ret != kEplSuccessful)
    {   // error occurred while initializing ethernet driver
        goto Exit;
    }

    // copy local MAC address from Ethernet driver back to local instance structure
    // because Ethernet driver may have read it from controller EEPROM
    EPL_MEMCPY(EplDllkInstance_g.m_be_abLocalMac, EdrvInitParam.m_abMyMacAddr, 6);
    EPL_MEMCPY(pInitParam_p->m_be_abLocalMac, EdrvInitParam.m_abMyMacAddr, 6);

    // initialize TxBuffer array
    for (uiIndex = 0; uiIndex < EplDllkInstance_g.m_uiMaxTxFrames; uiIndex++)
    {
        EplDllkInstance_g.m_pTxBuffer[uiIndex].m_pbBuffer = NULL;
    }

#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)
#if (EPL_DLL_DISABLE_EDRV_CYCLIC == FALSE)
    Ret = EdrvCyclicInit();
    if (Ret != kEplSuccessful)
    {
        goto Exit;
    }

    Ret = EdrvCyclicRegErrorHandler(EplDllkCbCyclicError);
    if (Ret != kEplSuccessful)
    {
        goto Exit;
    }

#endif
#endif

#if EPL_DLL_PRES_CHAINING_CN != FALSE
    EplDllkInstance_g.m_pSyncReqPrevTimeStamp = EplTgtTimeStampAlloc();
#endif

Exit:
    return Ret;
}


//---------------------------------------------------------------------------
//
// Function:    EplDllkDelInstance()
//
// Description: deletes an instance of EPL stack
//
// Parameters:  (none)
//
// Returns:     tEplKernel              = error code
//
//
// State:
//
//---------------------------------------------------------------------------

tEplKernel EplDllkDelInstance(void)
{
tEplKernel      Ret = kEplSuccessful;

    // reset state
    EplDllkInstance_g.m_DllState = kEplDllGsInit;

#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)
#if (EPL_DLL_DISABLE_EDRV_CYCLIC == FALSE)
    Ret = EdrvCyclicShutdown();
#endif
#endif

#if (EPL_DLL_PROCESS_SYNC == EPL_DLL_PROCESS_SYNC_ON_TIMER)
    Ret = EplTimerSynckDelInstance();
#endif

#if EPL_TIMER_USE_HIGHRES != FALSE
    Ret = EplTimerHighReskDelInstance();
#endif

#if EPL_DLL_PRES_CHAINING_CN != FALSE
    EplTgtTimeStampFree(EplDllkInstance_g.m_pSyncReqPrevTimeStamp);
#endif

    Ret = EdrvShutdown();

    return Ret;
}


//---------------------------------------------------------------------------
//
// Function:    EplDllkProcess
//
// Description: process the passed event
//
// Parameters:  pEvent_p                = event to be processed
//
// Returns:     tEplKernel              = error code
//
//
// State:
//
//---------------------------------------------------------------------------

tEplKernel EplDllkProcess(tEplEvent * pEvent_p)
{
tEplKernel                  Ret = kEplSuccessful;
tEplEventNmtStateChange*    pNmtStateChange;
#if EPL_DLL_PRES_READY_AFTER_SOA != FALSE
tEplFrame      *pTxFrame;
tEplNmtState    NmtState;
#endif


    switch (pEvent_p->m_EventType)
    {
        case kEplEventTypeNmtStateChange:
        {

            pNmtStateChange = (tEplEventNmtStateChange*) pEvent_p->m_pArg;

            Ret = EplDllkProcessNmtStateChange(pNmtStateChange->m_NewNmtState, pNmtStateChange->m_OldNmtState);
            break;
        }

        case kEplEventTypeNmtEvent:
        {

            Ret = EplDllkProcessNmtEvent(pEvent_p);

            break;
        }

        case kEplEventTypeDllkFillTx:
        {

            Ret = EplDllkProcessFillTx(*((tEplDllAsyncReqPriority*) pEvent_p->m_pArg), EplDllkInstance_g.m_NmtState);

            break;
        }

        case kEplEventTypeDllkFlag1:
        {   // trigger update of StatusRes on SoA, because Flag 1 was changed
            if (EplDllkInstance_g.m_bUpdateTxFrame == EPL_DLLK_UPDATE_NONE)
            {
                EplDllkInstance_g.m_bUpdateTxFrame = EPL_DLLK_UPDATE_STATUS;
            }

            break;
        }

        case kEplEventTypeDllkCycleFinish:
        {

            Ret = EplDllkProcessCycleFinish(EplDllkInstance_g.m_NmtState);

            break;
        }

        case kEplEventTypeSync:
        {

            Ret = EplDllkProcessSync(EplDllkInstance_g.m_NmtState);

            break;
        }

#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)
        case kEplEventTypeDllkStartReducedCycle:
        {

            Ret = EplDllkProcessStartReducedCycle(pEvent_p);

            break;
        }
#endif

#if EPL_DLL_PRES_READY_AFTER_SOA != FALSE
        case kEplEventTypeDllkPresReady:
        {
            // post PRes to transmit FIFO

            NmtState = EplDllkInstance_g.m_NmtState;

            if (NmtState != kEplNmtCsBasicEthernet)
            {
                // Does PRes exist?
                if (EplDllkInstance_g.m_pTxBuffer[EPL_DLLK_TXFRAME_PRES + EplDllkInstance_g.m_bCurTxBufferOffsetCycle].m_pbBuffer != NULL)
                {   // PRes does exist
                    pTxFrame = (tEplFrame *) EplDllkInstance_g.m_pTxBuffer[EPL_DLLK_TXFRAME_PRES + EplDllkInstance_g.m_bCurTxBufferOffsetCycle].m_pbBuffer;
                    // update frame (NMT state, RD, RS, PR, MS, EN flags)
                    if (NmtState < kEplNmtCsPreOperational2)
                    {   // NMT state is not PreOp2, ReadyToOp or Op
                        // fake NMT state PreOp2, because PRes will be sent only in PreOp2 or greater
                        NmtState = kEplNmtCsPreOperational2;
                    }
                    AmiSetByteToLe(&pTxFrame->m_Data.m_Pres.m_le_bNmtStatus, (BYTE) NmtState);
                    AmiSetByteToLe(&pTxFrame->m_Data.m_Pres.m_le_bFlag2, EplDllkInstance_g.m_bFlag2);
                    if (NmtState != kEplNmtCsOperational)
                    {   // mark PDO as invalid in all NMT state but Op
                        // $$$ reset only RD flag; set other flags appropriately
                        AmiSetByteToLe(&pTxFrame->m_Data.m_Pres.m_le_bFlag1, 0);
                    }
                    // $$$ make function that updates Pres, StatusRes
                    // mark PRes frame as ready for transmission
                    Ret = EdrvTxMsgReady(&EplDllkInstance_g.m_pTxBuffer[EPL_DLLK_TXFRAME_PRES + EplDllkInstance_g.m_bCurTxBufferOffsetCycle]);
                }
            }

            break;
        }
#endif
        default:
        {
            Ret = kEplInvalidEvent;
            ASSERTMSG(Ret != kEplInvalidEvent, "EplDllkProcess(): unhandled event type!\n");
            break;
        }
    }

//Exit:
    return Ret;
}


//---------------------------------------------------------------------------
//
// Function:    EplDllkConfig
//
// Description: configure parameters of DLL
//
// Parameters:  pDllConfigParam_p       = configuration parameters
//
// Returns:     tEplKernel              = error code
//
//
// State:
//
//---------------------------------------------------------------------------

tEplKernel EplDllkConfig(tEplDllConfigParam * pDllConfigParam_p)
{
tEplKernel      Ret = kEplSuccessful;
tEplNmtState    NmtState;

    NmtState = EplDllkInstance_g.m_NmtState;

    if (NmtState > kEplNmtGsResetConfiguration)
    {   // configuration updates are only allowed in state DLL_GS_INIT, except LossOfFrameTolerance,
        // because all other parameters are "valid on reset".
        EplDllkInstance_g.m_DllConfigParam.m_dwLossOfFrameTolerance = pDllConfigParam_p->m_dwLossOfFrameTolerance;

#if (EPL_DLL_PROCESS_SYNC == EPL_DLL_PROCESS_SYNC_ON_TIMER)
        Ret = EplTimerSynckSetLossOfSyncToleranceNs(pDllConfigParam_p->m_dwLossOfFrameTolerance);
#endif
    }
    else
    {   // copy entire configuration to local storage,
        // because we are in state DLL_GS_INIT
        EPL_MEMCPY (&EplDllkInstance_g.m_DllConfigParam, pDllConfigParam_p,
            (pDllConfigParam_p->m_uiSizeOfStruct < sizeof (tEplDllConfigParam) ?
            pDllConfigParam_p->m_uiSizeOfStruct : sizeof (tEplDllConfigParam)));
    }

    if (NmtState < kEplNmtMsNotActive)
    {   // CN or NMT reset states are active,
        // so we can calculate the frame timeout.
        // MN calculates on kEplEventTypeDllkCreate, its own frame timeout.
        if ((EplDllkInstance_g.m_DllConfigParam.m_dwCycleLen != 0)
            && (EplDllkInstance_g.m_DllConfigParam.m_dwLossOfFrameTolerance != 0))
        {   // monitor EPL cycle, calculate frame timeout
            EplDllkInstance_g.m_ullFrameTimeout = (1000LL
                * ((unsigned long long) EplDllkInstance_g.m_DllConfigParam.m_dwCycleLen))
                + ((unsigned long long) EplDllkInstance_g.m_DllConfigParam.m_dwLossOfFrameTolerance);
        }
        else
        {
            EplDllkInstance_g.m_ullFrameTimeout = 0LL;
        }
    }

    if (EplDllkInstance_g.m_DllConfigParam.m_fAsyncOnly != FALSE)
    {   // it is configured as async-only CN
        // disable multiplexed cycle, that m_uiCycleCount will not be incremented spuriously on SoC
        EplDllkInstance_g.m_DllConfigParam.m_uiMultiplCycleCnt = 0;
    }

//Exit:
    return Ret;
}

//---------------------------------------------------------------------------
//
// Function:    EplDllkSetIdentity
//
// Description: configure identity of local node for IdentResponse
//
// Parameters:  pDllIdentParam_p        = identity
//
// Returns:     tEplKernel              = error code
//
//
// State:
//
//---------------------------------------------------------------------------

tEplKernel EplDllkSetIdentity(tEplDllIdentParam * pDllIdentParam_p)
{
tEplKernel      Ret = kEplSuccessful;

    EPL_MEMCPY (&EplDllkInstance_g.m_DllIdentParam, pDllIdentParam_p,
        (pDllIdentParam_p->m_uiSizeOfStruct < sizeof (tEplDllIdentParam) ?
        pDllIdentParam_p->m_uiSizeOfStruct : sizeof (tEplDllIdentParam)));

    // $$$ if IdentResponse frame exists update it

    return Ret;
}


//---------------------------------------------------------------------------
//
// Function:    EplDllkRegAsyncHandler
//
// Description: registers handler for non-EPL frames (used by virtual Ethernet driver)
//
// Parameters:  pfnDllkCbAsync_p        = pointer to callback function,
//                                        which will be called in interrupt context
//                                        normally.
//
// Returns:     tEplKernel              = error code
//
//
// State:
//
//---------------------------------------------------------------------------

tEplKernel EplDllkRegAsyncHandler(tEplDllkCbAsync pfnDllkCbAsync_p)
{
tEplKernel  Ret = kEplSuccessful;

    if (EplDllkInstance_g.m_pfnCbAsync == NULL)
    {   // no handler registered yet
        EplDllkInstance_g.m_pfnCbAsync = pfnDllkCbAsync_p;
    }
    else
    {   // handler already registered
        Ret = kEplDllCbAsyncRegistered;
    }

    return Ret;
}


//---------------------------------------------------------------------------
//
// Function:    EplDllkDeregAsyncHandler
//
// Description: deregisters handler for non-EPL frames
//
// Parameters:  pfnDllkCbAsync_p        = pointer to callback function
//
// Returns:     tEplKernel              = error code
//
//
// State:
//
//---------------------------------------------------------------------------

tEplKernel EplDllkDeregAsyncHandler(tEplDllkCbAsync pfnDllkCbAsync_p)
{
tEplKernel  Ret = kEplSuccessful;

    if (EplDllkInstance_g.m_pfnCbAsync == pfnDllkCbAsync_p)
    {   // same handler is registered
        // deregister it
        EplDllkInstance_g.m_pfnCbAsync = NULL;
    }
    else
    {   // wrong handler or no handler registered
        Ret = kEplDllCbAsyncRegistered;
    }

    return Ret;
}


//---------------------------------------------------------------------------
//
// Function:    EplDllkRegSyncHandler
//
// Description: registers handler for Sync event
//
// Parameters:  pfnCbSync_p             = pointer to callback function,
//                                        which will be called in event context.
//
// Returns:     tEplSyncCb              = old callback function
//
//
// State:
//
//---------------------------------------------------------------------------

tEplSyncCb EplDllkRegSyncHandler(tEplSyncCb pfnCbSync_p)
{
tEplSyncCb  pfnCbOld;

    pfnCbOld = EplDllkInstance_g.m_pfnCbSync;
    EplDllkInstance_g.m_pfnCbSync = pfnCbSync_p;

    return pfnCbOld;
}


//---------------------------------------------------------------------------
//
// Function:    EplDllkRegRpdoHandler
//
// Description: registers handler for RPDOs (used by PDO module)
//
// Parameters:  pfnDllkCbProcessRpdo_p  = pointer to callback function,
//                                        which will be called in interrupt context.
//
// Returns:     tEplKernel              = error code
//
//
// State:
//
//---------------------------------------------------------------------------

tEplKernel EplDllkRegRpdoHandler(tEplDllkCbProcessRpdo pfnDllkCbProcessRpdo_p)
{
tEplKernel  Ret = kEplSuccessful;

    EplDllkInstance_g.m_pfnCbProcessRpdo = pfnDllkCbProcessRpdo_p;

    return Ret;
}


//---------------------------------------------------------------------------
//
// Function:    EplDllkRegTpdoHandler
//
// Description: registers handler for TPDOs (used by PDO module)
//
// Parameters:  pfnDllkCbProcessTpdo_p  = pointer to callback function,
//                                        which will be called in context of kernel part event queue.
//
// Returns:     tEplKernel              = error code
//
//
// State:
//
//---------------------------------------------------------------------------

tEplKernel EplDllkRegTpdoHandler(tEplDllkCbProcessTpdo pfnDllkCbProcessTpdo_p)
{
tEplKernel  Ret = kEplSuccessful;

    EplDllkInstance_g.m_pfnCbProcessTpdo = pfnDllkCbProcessTpdo_p;

    return Ret;
}


//---------------------------------------------------------------------------
//
// Function:    EplDllkSetAsndServiceIdFilter()
//
// Description: sets the specified node ID filter for the specified
//              AsndServiceId. It registers C_DLL_MULTICAST_ASND in ethernet
//              driver if any AsndServiceId is open.
//
// Parameters:  ServiceId_p             = ASnd Service ID
//              Filter_p                = node ID filter
//
// Returns:     tEplKernel              = error code
//
//
// State:
//
//---------------------------------------------------------------------------

tEplKernel EplDllkSetAsndServiceIdFilter(tEplDllAsndServiceId ServiceId_p, tEplDllAsndFilter Filter_p)
{
tEplKernel  Ret = kEplSuccessful;

    if (ServiceId_p < tabentries (EplDllkInstance_g.m_aAsndFilter))
    {
        EplDllkInstance_g.m_aAsndFilter[ServiceId_p] = Filter_p;
    }
    else
    {
        Ret = kEplDllInvalidAsndServiceId;
    }

    return Ret;
}


//---------------------------------------------------------------------------
//
// Function:    EplDllkReleaseRxFrame()
//
// Description: Releases the rx buffer for the specified rx frame in Edrv
//
// Parameters:  pFrame_p                = pointer to frame
//              uiFrameSize_p           = size of frame
//
// Returns:     tEplKernel              = error code
//
//
// State:
//
//---------------------------------------------------------------------------
#if EPL_DLL_DISABLE_DEFERRED_RXFRAME_RELEASE == FALSE
tEplKernel EplDllkReleaseRxFrame(tEplFrame* pFrame_p, unsigned int uiFrameSize_p)
{
tEplKernel      Ret;
tEdrvRxBuffer   RxBuffer;

    RxBuffer.m_pbBuffer   = (BYTE*) pFrame_p;
    RxBuffer.m_uiRxMsgLen = uiFrameSize_p;

    Ret = EdrvReleaseRxBuffer(&RxBuffer);

    return Ret;
}
#endif


#if EPL_NMT_MAX_NODE_ID > 0

//---------------------------------------------------------------------------
//
// Function:    EplDllkConfigNode()
//
// Description: configures the specified node (e.g. payload limits and timeouts).
//
// Parameters:  pNodeInfo_p             = pointer of node info structure
//
// Returns:     tEplKernel              = error code
//
//
// State:
//
//---------------------------------------------------------------------------

tEplKernel EplDllkConfigNode(tEplDllNodeInfo * pNodeInfo_p)
{
tEplKernel          Ret = kEplSuccessful;
tEplDllkNodeInfo*   pIntNodeInfo;
tEplNmtState        NmtState;

    NmtState = EplDllkInstance_g.m_NmtState;

    if ((NmtState > kEplNmtGsResetConfiguration)
        && (pNodeInfo_p->m_uiNodeId != EplDllkInstance_g.m_DllConfigParam.m_uiNodeId))
    {   // configuration updates are only allowed in reset states
        Ret = kEplInvalidOperation;
        goto Exit;
    }

    pIntNodeInfo = EplDllkGetNodeInfo(pNodeInfo_p->m_uiNodeId);
    if (pIntNodeInfo == NULL)
    {   // no node info structure available
        Ret = kEplDllNoNodeInfo;
        goto Exit;
    }

    // copy node configuration
    if (pNodeInfo_p->m_wPresPayloadLimit > EplDllkInstance_g.m_DllConfigParam.m_uiIsochrRxMaxPayload)
    {
        pIntNodeInfo->m_wPresPayloadLimit = (WORD) EplDllkInstance_g.m_DllConfigParam.m_uiIsochrRxMaxPayload;
    }
    else
    {
        pIntNodeInfo->m_wPresPayloadLimit = pNodeInfo_p->m_wPresPayloadLimit;
    }

#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)
    pIntNodeInfo->m_dwPresTimeoutNs = pNodeInfo_p->m_dwPresTimeoutNs;
    if (pNodeInfo_p->m_wPreqPayloadLimit > EplDllkInstance_g.m_DllConfigParam.m_uiIsochrTxMaxPayload)
    {
        pIntNodeInfo->m_wPreqPayloadLimit = (WORD) EplDllkInstance_g.m_DllConfigParam.m_uiIsochrTxMaxPayload;
    }
    else
    {
        pIntNodeInfo->m_wPreqPayloadLimit = pNodeInfo_p->m_wPreqPayloadLimit;
    }

    // initialize elements of internal node info structure
    pIntNodeInfo->m_bSoaFlag1 = 0;
    pIntNodeInfo->m_fSoftDelete = FALSE;
    pIntNodeInfo->m_ulDllErrorEvents = 0L;
    pIntNodeInfo->m_NmtState = kEplNmtCsNotActive;
#endif

Exit:
    return Ret;
}


//---------------------------------------------------------------------------
//
// Function:    EplDllkAddNode()
//
// Description: adds the specified node to the isochronous phase.
//
// Parameters:  pNodeInfo_p             = pointer of node info structure
//
// Returns:     tEplKernel              = error code
//
//
// State:
//
//---------------------------------------------------------------------------

tEplKernel EplDllkAddNode(tEplDllNodeOpParam* pNodeOpParam_p)
{
tEplKernel          Ret = kEplSuccessful;
tEplDllkNodeInfo*   pIntNodeInfo;
tEplNmtState        NmtState;

    NmtState = EplDllkInstance_g.m_NmtState;

    pIntNodeInfo = EplDllkGetNodeInfo(pNodeOpParam_p->m_uiNodeId);
    if (pIntNodeInfo == NULL)
    {   // no node info structure available
        Ret = kEplDllNoNodeInfo;
        goto Exit;
    }

    EPL_DLLK_DBG_POST_TRACE_VALUE(kEplEventTypeDllkAddNode,
                                  pNodeOpParam_p->m_uiNodeId,
                                  0);

    switch (pNodeOpParam_p->m_OpNodeType)
    {
#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)
        case kEplDllNodeOpTypeIsochronous:
        {
            if (NmtState >= kEplNmtMsNotActive)
            {
                Ret = EplDllkAddNodeIsochronous(pIntNodeInfo);
            }
            else
            {
                Ret = kEplDllInvalidParam;
            }

            break;
        }
#endif

        case kEplDllNodeOpTypeFilterPdo:
        case kEplDllNodeOpTypeFilterHeartbeat:
        {
        BOOL    fUpdateEdrv = FALSE;

            if ((NmtState >= kEplNmtCsNotActive) && (NmtState < kEplNmtMsNotActive))
            {
                fUpdateEdrv = TRUE;
            }
            Ret = EplDllkAddNodeFilter(pIntNodeInfo, pNodeOpParam_p->m_OpNodeType, fUpdateEdrv);
            break;
        }

        default:
        {
            Ret = kEplDllInvalidParam;
            break;
        }
    }

Exit:
    return Ret;
}


//---------------------------------------------------------------------------
//
// Function:    EplDllkDeleteNode()
//
// Description: removes the specified node from the isochronous phase.
//
// Parameters:  uiNodeId_p              = node ID
//
// Returns:     tEplKernel              = error code
//
//
// State:
//
//---------------------------------------------------------------------------

tEplKernel EplDllkDeleteNode(tEplDllNodeOpParam* pNodeOpParam_p)
{
tEplKernel          Ret = kEplSuccessful;
tEplDllkNodeInfo*   pIntNodeInfo;
tEplNmtState        NmtState;

    NmtState = EplDllkInstance_g.m_NmtState;

    if (pNodeOpParam_p->m_uiNodeId == EPL_C_ADR_BROADCAST)
    {
        switch (pNodeOpParam_p->m_OpNodeType)
        {
            case kEplDllNodeOpTypeFilterPdo:
            case kEplDllNodeOpTypeFilterHeartbeat:
            {
            BOOL            fUpdateEdrv = FALSE;
            unsigned int    uiIndex;

                if ((NmtState >= kEplNmtCsNotActive) && (NmtState < kEplNmtMsNotActive))
                {
                    fUpdateEdrv = TRUE;
                }

                for (uiIndex = 0, pIntNodeInfo = &EplDllkInstance_g.m_aNodeInfo[0];
                     uiIndex < tabentries (EplDllkInstance_g.m_aNodeInfo);
                     uiIndex++, pIntNodeInfo++)
                {
                    Ret = EplDllkDeleteNodeFilter(pIntNodeInfo, pNodeOpParam_p->m_OpNodeType, fUpdateEdrv);
                }
                break;
            }

            default:
            {
                Ret = kEplDllInvalidParam;
                break;
            }
        }
        goto Exit;
    }

    pIntNodeInfo = EplDllkGetNodeInfo(pNodeOpParam_p->m_uiNodeId);
    if (pIntNodeInfo == NULL)
    {   // no node info structure available
        Ret = kEplDllNoNodeInfo;
        goto Exit;
    }

    EPL_DLLK_DBG_POST_TRACE_VALUE(kEplEventTypeDllkDelNode,
                                  pNodeOpParam_p->m_uiNodeId,
                                  0);

    switch (pNodeOpParam_p->m_OpNodeType)
    {
#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)
        case kEplDllNodeOpTypeIsochronous:
        {
            if (NmtState >= kEplNmtMsNotActive)
            {
                Ret = EplDllkDeleteNodeIsochronous(pIntNodeInfo);
            }
            else
            {
                Ret = kEplDllInvalidParam;
            }

            break;
        }

        case kEplDllNodeOpTypeSoftDelete:
        {
            pIntNodeInfo->m_fSoftDelete = TRUE;

            break;
        }
#endif

        case kEplDllNodeOpTypeFilterPdo:
        case kEplDllNodeOpTypeFilterHeartbeat:
        {
        BOOL    fUpdateEdrv = FALSE;

            if ((NmtState >= kEplNmtCsNotActive) && (NmtState < kEplNmtMsNotActive))
            {
                fUpdateEdrv = TRUE;
            }
            Ret = EplDllkDeleteNodeFilter(pIntNodeInfo, pNodeOpParam_p->m_OpNodeType, fUpdateEdrv);
            break;
        }

        default:
        {
            Ret = kEplDllInvalidParam;
            break;
        }
    }

Exit:
    return Ret;
}

#endif // EPL_NMT_MAX_NODE_ID > 0


#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)

//---------------------------------------------------------------------------
//
// Function:    EplDllkSetFlag1OfNode()
//
// Description: sets Flag1 (for PReq and SoA) of the specified node ID.
//
// Parameters:  uiNodeId_p              = node ID
//              bSoaFlag1_p             = flag1
//
// Returns:     tEplKernel              = error code
//
//
// State:
//
//---------------------------------------------------------------------------

tEplKernel EplDllkSetFlag1OfNode(unsigned int uiNodeId_p, BYTE bSoaFlag1_p)
{
tEplKernel          Ret = kEplSuccessful;
tEplDllkNodeInfo*   pNodeInfo;

    pNodeInfo = EplDllkGetNodeInfo(uiNodeId_p);
    if (pNodeInfo == NULL)
    {   // no node info structure available
        Ret = kEplDllNoNodeInfo;
        goto Exit;
    }

    // store flag1 in internal node info structure
    pNodeInfo->m_bSoaFlag1 = bSoaFlag1_p;

Exit:
    return Ret;
}


//---------------------------------------------------------------------------
//
// Function:    EplDllkGetCurrentCnNodeIdList()
//
// Description: returns first info structure of first node in isochronous phase.
//              It is only useful for ErrorHandlerk module.
//
// Parameters:  ppbCnNodeIdList_p       = pointer to array of BYTE with node-ID list;
//                                        array is terminated by value 0
//
// Returns:     tEplKernel              = error code
//
//
// State:
//
//---------------------------------------------------------------------------

tEplKernel EplDllkGetCurrentCnNodeIdList(BYTE** ppbCnNodeIdList_p)
{
tEplKernel          Ret = kEplSuccessful;

    *ppbCnNodeIdList_p = &EplDllkInstance_g.m_aabCnNodeIdList[EplDllkInstance_g.m_bCurTxBufferOffsetCycle ^ 1][0];

    return Ret;
}


//---------------------------------------------------------------------------
//
// Function:    EplDllkGetCnMacAddress()
//
// Description: returns the MAC address of the specified node.
//
// Parameters:  uiNodeId_p              = [IN] node-ID of CN
//              pb_be_CnMacAddress_p    = [OUT] MAC address of the specified CN
//
// Returns:     tEplKernel              = error code
//
//
// State:
//
//---------------------------------------------------------------------------

#if EPL_DLL_PRES_CHAINING_MN != FALSE
tEplKernel EplDllkGetCnMacAddress(unsigned int uiNodeId_p, BYTE* pb_be_CnMacAddress_p)
{
tEplKernel          Ret = kEplSuccessful;
tEplDllkNodeInfo*   pNodeInfo;

    pNodeInfo = EplDllkGetNodeInfo(uiNodeId_p);
    if (pNodeInfo == NULL)
    {   // no node info structure available
        Ret = kEplDllNoNodeInfo;
        goto Exit;
    }

    EPL_MEMCPY(pb_be_CnMacAddress_p, pNodeInfo->m_be_abMacAddr, 6);

Exit:
    return Ret;
}
#endif

#endif // (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)


//=========================================================================//
//                                                                         //
//          P R I V A T E   F U N C T I O N S                              //
//                                                                         //
//=========================================================================//


//---------------------------------------------------------------------------
//
// Function:    EplDllkProcessCreate
//
// Description: process the create event
//
// Parameters:  NmtState_p              = new NMT state
//
// Returns:     tEplKernel              = error code
//
//
// State:
//
//---------------------------------------------------------------------------

static tEplKernel EplDllkProcessCreate(tEplNmtState NmtState_p)
{
tEplKernel      Ret = kEplSuccessful;
unsigned int    uiHandle;
unsigned int    uiFrameSize;
BYTE            abMulticastMac[6];
unsigned int        uiIndex;
tEplDllkNodeInfo*   pIntNodeInfo;

    // initialize flags for PRes and StatusRes (leave Flag 1 unchanged)
    EplDllkInstance_g.m_bMnFlag1 = 0;
    EplDllkInstance_g.m_bFlag2 = 0;

#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)
    // initialize linked node list
    EplDllkInstance_g.m_pFirstNodeInfo = NULL;
#if EPL_DLL_PRES_CHAINING_MN != FALSE
    EplDllkInstance_g.m_pFirstPrcNodeInfo = NULL;
#endif
#endif

    // register TxFrames in Edrv

    // IdentResponse
    uiFrameSize = EPL_C_DLL_MINSIZE_IDENTRES;
    Ret = EplDllkCreateTxFrame(&uiHandle, &uiFrameSize, kEplMsgTypeAsnd, kEplDllAsndIdentResponse);
    if (Ret != kEplSuccessful)
    {   // error occurred while registering Tx frame
        goto Exit;
    }

    // StatusResponse
    uiFrameSize = EPL_C_DLL_MINSIZE_STATUSRES;
    Ret = EplDllkCreateTxFrame(&uiHandle, &uiFrameSize, kEplMsgTypeAsnd, kEplDllAsndStatusResponse);
    if (Ret != kEplSuccessful)
    {   // error occurred while registering Tx frame
        goto Exit;
    }

#if EPL_DLL_PRES_CHAINING_CN != FALSE
    // SyncResponse
    uiFrameSize = EPL_C_DLL_MINSIZE_SYNCRES;
    Ret = EplDllkCreateTxFrame(&uiHandle, &uiFrameSize, kEplMsgTypeAsnd, kEplDllAsndSyncResponse);
    if (Ret != kEplSuccessful)
    {   // error occurred while registering Tx frame
        goto Exit;
    }
#endif

    // PRes
    if ((EplDllkInstance_g.m_DllConfigParam.m_fAsyncOnly == FALSE)
        && (EplDllkInstance_g.m_DllConfigParam.m_uiPresActPayloadLimit >= 36))
    {   // it is not configured as async-only CN,
        // so take part in isochronous phase and register PRes frame
        uiFrameSize = EplDllkInstance_g.m_DllConfigParam.m_uiPresActPayloadLimit + EPL_FRAME_OFFSET_PDO_PAYLOAD;
        Ret = EplDllkCreateTxFrame(&uiHandle, &uiFrameSize, kEplMsgTypePres, kEplDllAsndNotDefined);
        if (Ret != kEplSuccessful)
        {   // error occurred while registering Tx frame
            goto Exit;
        }

        // reset cycle counter
        EplDllkInstance_g.m_uiCycleCount = 0;
    }
    else
    {   // it is an async-only CN
        // fool EplDllkChangeState() to think that PRes was not expected
        EplDllkInstance_g.m_uiCycleCount = 1;
    }

    // NMT request
    uiFrameSize = EPL_C_IP_MAX_MTU;
    Ret = EplDllkCreateTxFrame(&uiHandle, &uiFrameSize, kEplMsgTypeAsnd, kEplDllAsndNmtRequest);
    if (Ret != kEplSuccessful)
    {   // error occurred while registering Tx frame
        goto Exit;
    }
    // mark Tx buffer as empty
    EplDllkInstance_g.m_pTxBuffer[uiHandle].m_uiTxMsgLen = EPL_DLLK_BUFLEN_EMPTY;
    EplDllkInstance_g.m_pTxBuffer[uiHandle].m_pfnTxHandler = EplDllkCbTransmittedNmtReq;
    uiHandle++;
    EplDllkInstance_g.m_pTxBuffer[uiHandle].m_uiTxMsgLen = EPL_DLLK_BUFLEN_EMPTY;
    EplDllkInstance_g.m_pTxBuffer[uiHandle].m_pfnTxHandler = EplDllkCbTransmittedNmtReq;

    // non-EPL frame
    uiFrameSize = EPL_C_IP_MAX_MTU;
    Ret = EplDllkCreateTxFrame(&uiHandle, &uiFrameSize, kEplMsgTypeNonEpl, kEplDllAsndNotDefined);
    if (Ret != kEplSuccessful)
    {   // error occurred while registering Tx frame
        goto Exit;
    }
    // mark Tx buffer as empty
    EplDllkInstance_g.m_pTxBuffer[uiHandle].m_uiTxMsgLen = EPL_DLLK_BUFLEN_EMPTY;
    EplDllkInstance_g.m_pTxBuffer[uiHandle].m_pfnTxHandler = EplDllkCbTransmittedNonEpl;
    uiHandle++;
    EplDllkInstance_g.m_pTxBuffer[uiHandle].m_uiTxMsgLen = EPL_DLLK_BUFLEN_EMPTY;
    EplDllkInstance_g.m_pTxBuffer[uiHandle].m_pfnTxHandler = EplDllkCbTransmittedNonEpl;


    // setup filter structure for Edrv
    EPL_MEMSET(EplDllkInstance_g.m_aFilter, 0, sizeof (EplDllkInstance_g.m_aFilter));

    // setup ASnd filter
    AmiSetQword48ToBe(&EplDllkInstance_g.m_aFilter[EPL_DLLK_FILTER_ASND].m_abFilterValue[0],
                      EPL_C_DLL_MULTICAST_ASND);
    AmiSetQword48ToBe(&EplDllkInstance_g.m_aFilter[EPL_DLLK_FILTER_ASND].m_abFilterMask[0],
                      EPL_DLL_MACADDR_MASK);
    EplDllkInstance_g.m_aFilter[EPL_DLLK_FILTER_ASND].m_fEnable = TRUE;

    // setup SoC filter
    AmiSetQword48ToBe(&EplDllkInstance_g.m_aFilter[EPL_DLLK_FILTER_SOC].m_abFilterValue[0],
                      EPL_C_DLL_MULTICAST_SOC);
    AmiSetQword48ToBe(&EplDllkInstance_g.m_aFilter[EPL_DLLK_FILTER_SOC].m_abFilterMask[0],
                      EPL_DLL_MACADDR_MASK);
    EplDllkInstance_g.m_aFilter[EPL_DLLK_FILTER_SOC].m_fEnable = TRUE;

    // setup SoA filter
    AmiSetQword48ToBe(&EplDllkInstance_g.m_aFilter[EPL_DLLK_FILTER_SOA].m_abFilterValue[0],
                      EPL_C_DLL_MULTICAST_SOA);
    AmiSetQword48ToBe(&EplDllkInstance_g.m_aFilter[EPL_DLLK_FILTER_SOA].m_abFilterMask[0],
                      EPL_DLL_MACADDR_MASK);
    EplDllkInstance_g.m_aFilter[EPL_DLLK_FILTER_SOA].m_fEnable = TRUE;

    // setup SoA/IdentReq filter
    AmiSetQword48ToBe(&EplDllkInstance_g.m_aFilter[EPL_DLLK_FILTER_SOA_IDREQ].m_abFilterValue[0],
                      EPL_C_DLL_MULTICAST_SOA);
    AmiSetQword48ToBe(&EplDllkInstance_g.m_aFilter[EPL_DLLK_FILTER_SOA_IDREQ].m_abFilterMask[0],
                      EPL_DLL_MACADDR_MASK);
    AmiSetWordToBe(&EplDllkInstance_g.m_aFilter[EPL_DLLK_FILTER_SOA_IDREQ].m_abFilterValue[12],
                   EPL_C_DLL_ETHERTYPE_EPL);
    AmiSetWordToBe(&EplDllkInstance_g.m_aFilter[EPL_DLLK_FILTER_SOA_IDREQ].m_abFilterMask[12],
                   0xFFFF);
    AmiSetByteToBe(&EplDllkInstance_g.m_aFilter[EPL_DLLK_FILTER_SOA_IDREQ].m_abFilterValue[14],
                   kEplMsgTypeSoa);
    AmiSetByteToBe(&EplDllkInstance_g.m_aFilter[EPL_DLLK_FILTER_SOA_IDREQ].m_abFilterMask[14],
                   0xFF);
    AmiSetByteToBe(&EplDllkInstance_g.m_aFilter[EPL_DLLK_FILTER_SOA_IDREQ].m_abFilterValue[20],
                   kEplDllReqServiceIdent);
    AmiSetByteToBe(&EplDllkInstance_g.m_aFilter[EPL_DLLK_FILTER_SOA_IDREQ].m_abFilterMask[20],
                   0xFF);
    AmiSetByteToBe(&EplDllkInstance_g.m_aFilter[EPL_DLLK_FILTER_SOA_IDREQ].m_abFilterValue[21],
                   (BYTE) EplDllkInstance_g.m_DllConfigParam.m_uiNodeId);
    AmiSetByteToBe(&EplDllkInstance_g.m_aFilter[EPL_DLLK_FILTER_SOA_IDREQ].m_abFilterMask[21],
                   0xFF);
    EplDllkInstance_g.m_aFilter[EPL_DLLK_FILTER_SOA_IDREQ].m_pTxBuffer = &EplDllkInstance_g.m_pTxBuffer[EPL_DLLK_TXFRAME_IDENTRES];
    EplDllkInstance_g.m_aFilter[EPL_DLLK_FILTER_SOA_IDREQ].m_fEnable = FALSE;

    // setup SoA/StatusReq filter
    AmiSetQword48ToBe(&EplDllkInstance_g.m_aFilter[EPL_DLLK_FILTER_SOA_STATREQ].m_abFilterValue[0],
                      EPL_C_DLL_MULTICAST_SOA);
    AmiSetQword48ToBe(&EplDllkInstance_g.m_aFilter[EPL_DLLK_FILTER_SOA_STATREQ].m_abFilterMask[0],
                      EPL_DLL_MACADDR_MASK);
    AmiSetWordToBe(&EplDllkInstance_g.m_aFilter[EPL_DLLK_FILTER_SOA_STATREQ].m_abFilterValue[12],
                   EPL_C_DLL_ETHERTYPE_EPL);
    AmiSetWordToBe(&EplDllkInstance_g.m_aFilter[EPL_DLLK_FILTER_SOA_STATREQ].m_abFilterMask[12],
                   0xFFFF);
    AmiSetByteToBe(&EplDllkInstance_g.m_aFilter[EPL_DLLK_FILTER_SOA_STATREQ].m_abFilterValue[14],
                   kEplMsgTypeSoa);
    AmiSetByteToBe(&EplDllkInstance_g.m_aFilter[EPL_DLLK_FILTER_SOA_STATREQ].m_abFilterMask[14],
                   0xFF);
    AmiSetByteToBe(&EplDllkInstance_g.m_aFilter[EPL_DLLK_FILTER_SOA_STATREQ].m_abFilterValue[20],
                   kEplDllReqServiceStatus);
    AmiSetByteToBe(&EplDllkInstance_g.m_aFilter[EPL_DLLK_FILTER_SOA_STATREQ].m_abFilterMask[20],
                   0xFF);
    AmiSetByteToBe(&EplDllkInstance_g.m_aFilter[EPL_DLLK_FILTER_SOA_STATREQ].m_abFilterValue[21],
                   (BYTE) EplDllkInstance_g.m_DllConfigParam.m_uiNodeId);
    AmiSetByteToBe(&EplDllkInstance_g.m_aFilter[EPL_DLLK_FILTER_SOA_STATREQ].m_abFilterMask[21],
                   0xFF);
    EplDllkInstance_g.m_aFilter[EPL_DLLK_FILTER_SOA_STATREQ].m_pTxBuffer = &EplDllkInstance_g.m_pTxBuffer[EPL_DLLK_TXFRAME_STATUSRES];
    EplDllkInstance_g.m_aFilter[EPL_DLLK_FILTER_SOA_STATREQ].m_fEnable = FALSE;

    // setup SoA/NmtReq filter
    AmiSetQword48ToBe(&EplDllkInstance_g.m_aFilter[EPL_DLLK_FILTER_SOA_NMTREQ].m_abFilterValue[0],
                      EPL_C_DLL_MULTICAST_SOA);
    AmiSetQword48ToBe(&EplDllkInstance_g.m_aFilter[EPL_DLLK_FILTER_SOA_NMTREQ].m_abFilterMask[0],
                      EPL_DLL_MACADDR_MASK);
    AmiSetWordToBe(&EplDllkInstance_g.m_aFilter[EPL_DLLK_FILTER_SOA_NMTREQ].m_abFilterValue[12],
                   EPL_C_DLL_ETHERTYPE_EPL);
    AmiSetWordToBe(&EplDllkInstance_g.m_aFilter[EPL_DLLK_FILTER_SOA_NMTREQ].m_abFilterMask[12],
                   0xFFFF);
    AmiSetByteToBe(&EplDllkInstance_g.m_aFilter[EPL_DLLK_FILTER_SOA_NMTREQ].m_abFilterValue[14],
                   kEplMsgTypeSoa);
    AmiSetByteToBe(&EplDllkInstance_g.m_aFilter[EPL_DLLK_FILTER_SOA_NMTREQ].m_abFilterMask[14],
                   0xFF);
    AmiSetByteToBe(&EplDllkInstance_g.m_aFilter[EPL_DLLK_FILTER_SOA_NMTREQ].m_abFilterValue[20],
                   kEplDllReqServiceNmtRequest);
    AmiSetByteToBe(&EplDllkInstance_g.m_aFilter[EPL_DLLK_FILTER_SOA_NMTREQ].m_abFilterMask[20],
                   0xFF);
    AmiSetByteToBe(&EplDllkInstance_g.m_aFilter[EPL_DLLK_FILTER_SOA_NMTREQ].m_abFilterValue[21],
                   (BYTE) EplDllkInstance_g.m_DllConfigParam.m_uiNodeId);
    AmiSetByteToBe(&EplDllkInstance_g.m_aFilter[EPL_DLLK_FILTER_SOA_NMTREQ].m_abFilterMask[21],
                   0xFF);
    EplDllkInstance_g.m_aFilter[EPL_DLLK_FILTER_SOA_NMTREQ].m_pTxBuffer = &EplDllkInstance_g.m_pTxBuffer[EPL_DLLK_TXFRAME_NMTREQ];
    EplDllkInstance_g.m_aFilter[EPL_DLLK_FILTER_SOA_NMTREQ].m_fEnable = FALSE;

#if EPL_DLL_PRES_CHAINING_CN != FALSE
    // setup SoA/SyncReq filter
    AmiSetQword48ToBe(&EplDllkInstance_g.m_aFilter[EPL_DLLK_FILTER_SOA_SYNCREQ].m_abFilterValue[0],
                      EPL_C_DLL_MULTICAST_SOA);
    AmiSetQword48ToBe(&EplDllkInstance_g.m_aFilter[EPL_DLLK_FILTER_SOA_SYNCREQ].m_abFilterMask[0],
                      EPL_DLL_MACADDR_MASK);
    AmiSetWordToBe(&EplDllkInstance_g.m_aFilter[EPL_DLLK_FILTER_SOA_SYNCREQ].m_abFilterValue[12],
                   EPL_C_DLL_ETHERTYPE_EPL);
    AmiSetWordToBe(&EplDllkInstance_g.m_aFilter[EPL_DLLK_FILTER_SOA_SYNCREQ].m_abFilterMask[12],
                   0xFFFF);
    AmiSetByteToBe(&EplDllkInstance_g.m_aFilter[EPL_DLLK_FILTER_SOA_SYNCREQ].m_abFilterValue[14],
                   kEplMsgTypeSoa);
    AmiSetByteToBe(&EplDllkInstance_g.m_aFilter[EPL_DLLK_FILTER_SOA_SYNCREQ].m_abFilterMask[14],
                   0xFF);
    AmiSetByteToBe(&EplDllkInstance_g.m_aFilter[EPL_DLLK_FILTER_SOA_SYNCREQ].m_abFilterValue[20],
                   kEplDllReqServiceSync);
    AmiSetByteToBe(&EplDllkInstance_g.m_aFilter[EPL_DLLK_FILTER_SOA_SYNCREQ].m_abFilterMask[20],
                   0xFF);
    AmiSetByteToBe(&EplDllkInstance_g.m_aFilter[EPL_DLLK_FILTER_SOA_SYNCREQ].m_abFilterValue[21],
                   (BYTE) EplDllkInstance_g.m_DllConfigParam.m_uiNodeId);
    AmiSetByteToBe(&EplDllkInstance_g.m_aFilter[EPL_DLLK_FILTER_SOA_SYNCREQ].m_abFilterMask[21],
                   0xFF);
    EplDllkInstance_g.m_aFilter[EPL_DLLK_FILTER_SOA_SYNCREQ].m_pTxBuffer = &EplDllkInstance_g.m_pTxBuffer[EPL_DLLK_TXFRAME_SYNCRES];
    EplDllkInstance_g.m_aFilter[EPL_DLLK_FILTER_SOA_SYNCREQ].m_fEnable = FALSE;
#endif

    // setup SoA/UnspecifiedReq filter
    AmiSetQword48ToBe(&EplDllkInstance_g.m_aFilter[EPL_DLLK_FILTER_SOA_NONEPL].m_abFilterValue[0],
                      EPL_C_DLL_MULTICAST_SOA);
    AmiSetQword48ToBe(&EplDllkInstance_g.m_aFilter[EPL_DLLK_FILTER_SOA_NONEPL].m_abFilterMask[0],
                      EPL_DLL_MACADDR_MASK);
    AmiSetWordToBe(&EplDllkInstance_g.m_aFilter[EPL_DLLK_FILTER_SOA_NONEPL].m_abFilterValue[12],
                   EPL_C_DLL_ETHERTYPE_EPL);
    AmiSetWordToBe(&EplDllkInstance_g.m_aFilter[EPL_DLLK_FILTER_SOA_NONEPL].m_abFilterMask[12],
                   0xFFFF);
    AmiSetByteToBe(&EplDllkInstance_g.m_aFilter[EPL_DLLK_FILTER_SOA_NONEPL].m_abFilterValue[14],
                   kEplMsgTypeSoa);
    AmiSetByteToBe(&EplDllkInstance_g.m_aFilter[EPL_DLLK_FILTER_SOA_NONEPL].m_abFilterMask[14],
                   0xFF);
    AmiSetByteToBe(&EplDllkInstance_g.m_aFilter[EPL_DLLK_FILTER_SOA_NONEPL].m_abFilterValue[20],
                   kEplDllReqServiceUnspecified);
    AmiSetByteToBe(&EplDllkInstance_g.m_aFilter[EPL_DLLK_FILTER_SOA_NONEPL].m_abFilterMask[20],
                   0xFF);
    AmiSetByteToBe(&EplDllkInstance_g.m_aFilter[EPL_DLLK_FILTER_SOA_NONEPL].m_abFilterValue[21],
                   (BYTE) EplDllkInstance_g.m_DllConfigParam.m_uiNodeId);
    AmiSetByteToBe(&EplDllkInstance_g.m_aFilter[EPL_DLLK_FILTER_SOA_NONEPL].m_abFilterMask[21],
                   0xFF);
    EplDllkInstance_g.m_aFilter[EPL_DLLK_FILTER_SOA_NONEPL].m_pTxBuffer = &EplDllkInstance_g.m_pTxBuffer[EPL_DLLK_TXFRAME_NONEPL];
    EplDllkInstance_g.m_aFilter[EPL_DLLK_FILTER_SOA_NONEPL].m_fEnable = FALSE;


    // register multicast MACs in ethernet driver
    AmiSetQword48ToBe(&abMulticastMac[0], EPL_C_DLL_MULTICAST_SOC);
    Ret = EdrvDefineRxMacAddrEntry(abMulticastMac);
    AmiSetQword48ToBe(&abMulticastMac[0], EPL_C_DLL_MULTICAST_SOA);
    Ret = EdrvDefineRxMacAddrEntry(abMulticastMac);
    AmiSetQword48ToBe(&abMulticastMac[0], EPL_C_DLL_MULTICAST_PRES);
    Ret = EdrvDefineRxMacAddrEntry(abMulticastMac);
    AmiSetQword48ToBe(&abMulticastMac[0], EPL_C_DLL_MULTICAST_ASND);
    Ret = EdrvDefineRxMacAddrEntry(abMulticastMac);

#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)
    if (NmtState_p >= kEplNmtMsNotActive)
    {   // local node is MN
    unsigned int    uiCount = 0;

        // SoC
        uiFrameSize = EPL_C_DLL_MINSIZE_SOC;
        Ret = EplDllkCreateTxFrame(&uiHandle, &uiFrameSize, kEplMsgTypeSoc, kEplDllAsndNotDefined);
        if (Ret != kEplSuccessful)
        {   // error occurred while registering Tx frame
            goto Exit;
        }
        EplDllkInstance_g.m_pTxBuffer[uiHandle].m_pfnTxHandler = EplDllkCbTransmittedSoc;
        uiHandle++;
        EplDllkInstance_g.m_pTxBuffer[uiHandle].m_pfnTxHandler = EplDllkCbTransmittedSoc;

        // SoA
        uiFrameSize = EPL_C_DLL_MINSIZE_SOA;
        Ret = EplDllkCreateTxFrame(&uiHandle, &uiFrameSize, kEplMsgTypeSoa, kEplDllAsndNotDefined);
        if (Ret != kEplSuccessful)
        {   // error occurred while registering Tx frame
            goto Exit;
        }
        EplDllkInstance_g.m_pTxBuffer[uiHandle].m_pfnTxHandler = EplDllkCbTransmittedSoa;
        uiHandle++;
        EplDllkInstance_g.m_pTxBuffer[uiHandle].m_pfnTxHandler = EplDllkCbTransmittedSoa;

        for (uiIndex = 0, pIntNodeInfo = &EplDllkInstance_g.m_aNodeInfo[0];
             uiIndex < tabentries (EplDllkInstance_g.m_aNodeInfo);
             uiIndex++, pIntNodeInfo++)
        {
            if (pIntNodeInfo->m_wPreqPayloadLimit > 0)
            {   // create PReq frame for this node
                uiCount++;

                uiFrameSize = pIntNodeInfo->m_wPreqPayloadLimit + EPL_FRAME_OFFSET_PDO_PAYLOAD;
                Ret = EplDllkCreateTxFrame(&uiHandle, &uiFrameSize, kEplMsgTypePreq, kEplDllAsndNotDefined);
                if (Ret != kEplSuccessful)
                {
                    goto Exit;
                }
#if (EPL_DLL_DISABLE_EDRV_CYCLIC != FALSE)
                EplDllkInstance_g.m_pTxBuffer[uiHandle].m_pfnTxHandler = EplDllkCbTransmittedPreq;
                uiHandle++;
                EplDllkInstance_g.m_pTxBuffer[uiHandle].m_pfnTxHandler = EplDllkCbTransmittedPreq;
#endif
                pIntNodeInfo->m_pPreqTxBuffer = &EplDllkInstance_g.m_pTxBuffer[uiHandle];

            }
        }

#if (EPL_DLL_DISABLE_EDRV_CYCLIC == FALSE)
        // alloc TxBuffer pointer list
        uiCount += 5;   // SoC, PResMN, SoA, ASnd, NULL
        EplDllkInstance_g.m_ppTxBufferList = EPL_MALLOC(sizeof (tEdrvTxBuffer*) * uiCount);
        if (EplDllkInstance_g.m_ppTxBufferList == NULL)
        {
            Ret = kEplDllOutOfMemory;
            goto Exit;
        }

        Ret = EdrvCyclicSetMaxTxBufferListSize(uiCount);
        if (Ret != kEplSuccessful)
        {
            goto Exit;
        }

        Ret = EdrvCyclicSetCycleLenUs(EplDllkInstance_g.m_DllConfigParam.m_dwCycleLen);
        if (Ret != kEplSuccessful)
        {
            goto Exit;
        }

        Ret = EdrvCyclicRegSyncHandler(EplDllkCbMnSyncHandler);
        if (Ret != kEplSuccessful)
        {
            goto Exit;
        }
#endif

        // calculate cycle length
        EplDllkInstance_g.m_ullFrameTimeout = 1000LL
            * ((unsigned long long) EplDllkInstance_g.m_DllConfigParam.m_dwCycleLen);

        // setup PRes filter
        AmiSetQword48ToBe(&EplDllkInstance_g.m_aFilter[EPL_DLLK_FILTER_PRES].m_abFilterValue[0],
                          EPL_C_DLL_MULTICAST_PRES);
        AmiSetQword48ToBe(&EplDllkInstance_g.m_aFilter[EPL_DLLK_FILTER_PRES].m_abFilterMask[0],
                          EPL_DLL_MACADDR_MASK);
        AmiSetWordToBe(&EplDllkInstance_g.m_aFilter[EPL_DLLK_FILTER_PRES].m_abFilterValue[12],
                       EPL_C_DLL_ETHERTYPE_EPL);
        AmiSetWordToBe(&EplDllkInstance_g.m_aFilter[EPL_DLLK_FILTER_PRES].m_abFilterMask[12],
                       0xFFFF);
        AmiSetByteToBe(&EplDllkInstance_g.m_aFilter[EPL_DLLK_FILTER_PRES].m_abFilterValue[14],
                       kEplMsgTypePres);
        AmiSetByteToBe(&EplDllkInstance_g.m_aFilter[EPL_DLLK_FILTER_PRES].m_abFilterMask[14],
                       0xFF);
        EplDllkInstance_g.m_aFilter[EPL_DLLK_FILTER_PRES].m_fEnable = TRUE;

    }
    else
#endif // (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)
    {   // local node is CN

        // setup PReq filter
        EPL_MEMCPY(&EplDllkInstance_g.m_aFilter[EPL_DLLK_FILTER_PREQ].m_abFilterValue[0],
                   &EplDllkInstance_g.m_be_abLocalMac[0], 6);
        AmiSetQword48ToBe(&EplDllkInstance_g.m_aFilter[EPL_DLLK_FILTER_PREQ].m_abFilterMask[0],
                          EPL_DLL_MACADDR_MASK);
        AmiSetWordToBe(&EplDllkInstance_g.m_aFilter[EPL_DLLK_FILTER_PREQ].m_abFilterValue[12],
                       EPL_C_DLL_ETHERTYPE_EPL);
        AmiSetWordToBe(&EplDllkInstance_g.m_aFilter[EPL_DLLK_FILTER_PREQ].m_abFilterMask[12],
                       0xFFFF);
        AmiSetByteToBe(&EplDllkInstance_g.m_aFilter[EPL_DLLK_FILTER_PREQ].m_abFilterValue[14],
                       kEplMsgTypePreq);
        AmiSetByteToBe(&EplDllkInstance_g.m_aFilter[EPL_DLLK_FILTER_PREQ].m_abFilterMask[14],
                       0xFF);
        AmiSetByteToBe(&EplDllkInstance_g.m_aFilter[EPL_DLLK_FILTER_PREQ].m_abFilterValue[15],
                       (BYTE) EplDllkInstance_g.m_DllConfigParam.m_uiNodeId);
        AmiSetByteToBe(&EplDllkInstance_g.m_aFilter[EPL_DLLK_FILTER_PREQ].m_abFilterMask[15],
                       0xFF);
        AmiSetByteToBe(&EplDllkInstance_g.m_aFilter[EPL_DLLK_FILTER_PREQ].m_abFilterValue[16],
                       EPL_C_ADR_MN_DEF_NODE_ID);
        AmiSetByteToBe(&EplDllkInstance_g.m_aFilter[EPL_DLLK_FILTER_PREQ].m_abFilterMask[16],
                       0xFF);
        EplDllkInstance_g.m_aFilter[EPL_DLLK_FILTER_PREQ].m_pTxBuffer = &EplDllkInstance_g.m_pTxBuffer[EPL_DLLK_TXFRAME_PRES];
        EplDllkInstance_g.m_aFilter[EPL_DLLK_FILTER_PREQ].m_fEnable = FALSE;

        // setup PRes filter
#if EPL_DLL_PRES_FILTER_COUNT < 0
        AmiSetQword48ToBe(&EplDllkInstance_g.m_aFilter[EPL_DLLK_FILTER_PRES].m_abFilterValue[0],
                          EPL_C_DLL_MULTICAST_PRES);
        AmiSetQword48ToBe(&EplDllkInstance_g.m_aFilter[EPL_DLLK_FILTER_PRES].m_abFilterMask[0],
                          EPL_DLL_MACADDR_MASK);
        AmiSetWordToBe(&EplDllkInstance_g.m_aFilter[EPL_DLLK_FILTER_PRES].m_abFilterValue[12],
                       EPL_C_DLL_ETHERTYPE_EPL);
        AmiSetWordToBe(&EplDllkInstance_g.m_aFilter[EPL_DLLK_FILTER_PRES].m_abFilterMask[12],
                       0xFFFF);
        AmiSetByteToBe(&EplDllkInstance_g.m_aFilter[EPL_DLLK_FILTER_PRES].m_abFilterValue[14],
                       kEplMsgTypePres);
        AmiSetByteToBe(&EplDllkInstance_g.m_aFilter[EPL_DLLK_FILTER_PRES].m_abFilterMask[14],
                       0xFF);
        if (EplDllkInstance_g.m_uiUsedPresFilterCount > 0)
        {
            EplDllkInstance_g.m_aFilter[EPL_DLLK_FILTER_PRES].m_fEnable = TRUE;
        }
        else
        {
            EplDllkInstance_g.m_aFilter[EPL_DLLK_FILTER_PRES].m_fEnable = FALSE;
        }

#else
        for (uiHandle = EPL_DLLK_FILTER_PRES; uiHandle < EPL_DLLK_FILTER_COUNT; uiHandle++)
        {
            AmiSetQword48ToBe(&EplDllkInstance_g.m_aFilter[uiHandle].m_abFilterValue[0],
                              EPL_C_DLL_MULTICAST_PRES);
            AmiSetQword48ToBe(&EplDllkInstance_g.m_aFilter[uiHandle].m_abFilterMask[0],
                              EPL_DLL_MACADDR_MASK);
            AmiSetWordToBe(&EplDllkInstance_g.m_aFilter[uiHandle].m_abFilterValue[12],
                           EPL_C_DLL_ETHERTYPE_EPL);
            AmiSetWordToBe(&EplDllkInstance_g.m_aFilter[uiHandle].m_abFilterMask[12],
                           0xFFFF);
            AmiSetByteToBe(&EplDllkInstance_g.m_aFilter[uiHandle].m_abFilterValue[14],
                           kEplMsgTypePres);
            AmiSetByteToBe(&EplDllkInstance_g.m_aFilter[uiHandle].m_abFilterMask[14],
                           0xFF);
            AmiSetByteToBe(&EplDllkInstance_g.m_aFilter[uiHandle].m_abFilterMask[16],
                           0xFF);
            EplDllkInstance_g.m_aFilter[uiHandle].m_fEnable = FALSE;
        }

        uiHandle = EPL_DLLK_FILTER_PRES;
        for (uiIndex = 0, pIntNodeInfo = &EplDllkInstance_g.m_aNodeInfo[0];
             uiIndex < tabentries (EplDllkInstance_g.m_aNodeInfo);
             uiIndex++, pIntNodeInfo++)
        {
            if ((pIntNodeInfo->m_bPresFilterFlags & (EPL_DLLK_FILTER_FLAG_PDO | EPL_DLLK_FILTER_FLAG_HB)) != 0)
            {
                AmiSetByteToBe(&EplDllkInstance_g.m_aFilter[uiHandle].m_abFilterValue[16],
                               pIntNodeInfo->m_uiNodeId);
                EplDllkInstance_g.m_aFilter[uiHandle].m_fEnable = TRUE;

                uiHandle++;
                if (uiHandle >= EPL_DLLK_FILTER_COUNT)
                {
                    break;
                }
            }
        }
#endif

#if (EPL_DLL_PROCESS_SYNC == EPL_DLL_PROCESS_SYNC_ON_TIMER)
        Ret = EplTimerSynckSetCycleLenUs(EplDllkInstance_g.m_DllConfigParam.m_dwCycleLen);
        if (Ret != kEplSuccessful)
        {
            goto Exit;
        }
        Ret = EplTimerSynckSetLossOfSyncToleranceNs(EplDllkInstance_g.m_DllConfigParam.m_dwLossOfFrameTolerance);
        if (Ret != kEplSuccessful)
        {
            goto Exit;
        }
#endif

#if EPL_DLL_PRES_CHAINING_CN != FALSE
        EplDllkInstance_g.m_fPrcEnabled         = FALSE;
        EplDllkInstance_g.m_uiSyncReqPrevNodeId = 0;
#endif
    }

    // clear all asynchronous buffers
    Ret = EplDllkCalAsyncClearBuffer();
    if (Ret != kEplSuccessful)
    {
        goto Exit;
    }

    // set filters in Edrv
    Ret = EdrvChangeFilter(EplDllkInstance_g.m_aFilter, EPL_DLLK_FILTER_COUNT, EPL_DLLK_FILTER_COUNT, 0);

Exit:
    return Ret;
}


//---------------------------------------------------------------------------
//
// Function:    EplDllkProcessDestroy
//
// Description: process the destroy event
//
// Parameters:  OldNmtState_p           = previous NMT state
//
// Returns:     tEplKernel              = error code
//
//
// State:
//
//---------------------------------------------------------------------------

static tEplKernel EplDllkProcessDestroy(tEplNmtState OldNmtState_p)
{
tEplKernel      Ret = kEplSuccessful;
BYTE            abMulticastMac[6];
unsigned int    uiIndex;
#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)
unsigned int    uiHandle;
#endif

    // remove all filters from Edrv
    Ret = EdrvChangeFilter(NULL, 0, 0, 0);

#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)
#if (EPL_DLL_DISABLE_EDRV_CYCLIC == FALSE)
    // destroy all data structures
    EPL_FREE(EplDllkInstance_g.m_ppTxBufferList);
    EplDllkInstance_g.m_ppTxBufferList = NULL;
#endif
#endif

    // delete timer
#if EPL_TIMER_USE_HIGHRES != FALSE
    Ret = EplTimerHighReskDeleteTimer(&EplDllkInstance_g.m_TimerHdlCycle);
    if (Ret != kEplSuccessful)
    {
        goto Exit;
    }
#endif

#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)
#if (EPL_DLL_DISABLE_EDRV_CYCLIC == FALSE)
    Ret = EdrvCyclicStopCycle();
    if (Ret != kEplSuccessful)
    {
        goto Exit;
    }

    Ret = EdrvCyclicRegSyncHandler(NULL);
    if (Ret != kEplSuccessful)
    {
        goto Exit;
    }
#endif
#endif

#if (EPL_DLL_PROCESS_SYNC == EPL_DLL_PROCESS_SYNC_ON_TIMER)
    Ret = EplTimerSynckStopSync();
    if (Ret != kEplSuccessful)
    {
        goto Exit;
    }
#endif

    // delete Tx frames
    Ret = EplDllkDeleteTxFrame(EPL_DLLK_TXFRAME_IDENTRES);
    if (Ret != kEplSuccessful)
    {   // error occurred while deregistering Tx frame
        goto Exit;
    }

    Ret = EplDllkDeleteTxFrame(EPL_DLLK_TXFRAME_STATUSRES);
    if (Ret != kEplSuccessful)
    {   // error occurred while deregistering Tx frame
        goto Exit;
    }

    Ret = EplDllkDeleteTxFrame(EPL_DLLK_TXFRAME_PRES);
    if (Ret != kEplSuccessful)
    {   // error occurred while deregistering Tx frame
        goto Exit;
    }

    Ret = EplDllkDeleteTxFrame(EPL_DLLK_TXFRAME_NMTREQ);
    if (Ret != kEplSuccessful)
    {   // error occurred while deregistering Tx frame
        goto Exit;
    }

#if EPL_DLL_PRES_CHAINING_CN != FALSE
    Ret = EplDllkDeleteTxFrame(EPL_DLLK_TXFRAME_SYNCRES);
    if (Ret != kEplSuccessful)
    {   // error occurred while deregistering Tx frame
        goto Exit;
    }
#endif

    Ret = EplDllkDeleteTxFrame(EPL_DLLK_TXFRAME_NONEPL);
    if (Ret != kEplSuccessful)
    {   // error occurred while deregistering Tx frame
        goto Exit;
    }

#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)
    if (OldNmtState_p >= kEplNmtMsNotActive)
    {   // local node was MN

        Ret = EplDllkDeleteTxFrame(EPL_DLLK_TXFRAME_SOC);
        if (Ret != kEplSuccessful)
        {   // error occurred while deregistering Tx frame
            goto Exit;
        }

        Ret = EplDllkDeleteTxFrame(EPL_DLLK_TXFRAME_SOA);
        if (Ret != kEplSuccessful)
        {   // error occurred while deregistering Tx frame
            goto Exit;
        }

        for (uiIndex = 0; uiIndex < tabentries (EplDllkInstance_g.m_aNodeInfo); uiIndex++)
        {
            if (EplDllkInstance_g.m_aNodeInfo[uiIndex].m_pPreqTxBuffer != NULL)
            {
                uiHandle = (unsigned int) (EplDllkInstance_g.m_aNodeInfo[uiIndex].m_pPreqTxBuffer - EplDllkInstance_g.m_pTxBuffer);
                EplDllkInstance_g.m_aNodeInfo[uiIndex].m_pPreqTxBuffer = NULL;
                if (uiHandle != EPL_DLLK_TXFRAME_PRES)
                {
                    Ret = EplDllkDeleteTxFrame(uiHandle);
                    if (Ret != kEplSuccessful)
                    {   // error occurred while deregistering Tx frame
                        goto Exit;
                    }
                }
            }

            // disable PReq and PRes for this node
            EplDllkInstance_g.m_aNodeInfo[uiIndex].m_wPreqPayloadLimit = 0;
            EplDllkInstance_g.m_aNodeInfo[uiIndex].m_wPresPayloadLimit = 0;
        }
    }
    else
#endif // (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)
    {
        for (uiIndex = 0; uiIndex < tabentries (EplDllkInstance_g.m_aNodeInfo); uiIndex++)
        {
            // disable PReq and PRes for this node
            EplDllkInstance_g.m_aNodeInfo[uiIndex].m_wPresPayloadLimit = 0;
#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)
            EplDllkInstance_g.m_aNodeInfo[uiIndex].m_wPreqPayloadLimit = 0;
#endif
        }
    }

    // deregister multicast MACs in ethernet driver
    AmiSetQword48ToBe(&abMulticastMac[0], EPL_C_DLL_MULTICAST_SOC);
    Ret = EdrvUndefineRxMacAddrEntry(abMulticastMac);
    AmiSetQword48ToBe(&abMulticastMac[0], EPL_C_DLL_MULTICAST_SOA);
    Ret = EdrvUndefineRxMacAddrEntry(abMulticastMac);
    AmiSetQword48ToBe(&abMulticastMac[0], EPL_C_DLL_MULTICAST_PRES);
    Ret = EdrvUndefineRxMacAddrEntry(abMulticastMac);
    AmiSetQword48ToBe(&abMulticastMac[0], EPL_C_DLL_MULTICAST_ASND);
    Ret = EdrvUndefineRxMacAddrEntry(abMulticastMac);

Exit:
    return Ret;
}


//---------------------------------------------------------------------------
//
// Function:    EplDllkProcessNmtStateChange
//
// Description: process the NMT event
//
// Parameters:  NewNmtState_p           = new NMT state
//
// Returns:     tEplKernel              = error code
//
//
// State:
//
//---------------------------------------------------------------------------

static tEplKernel EplDllkProcessNmtStateChange(tEplNmtState NewNmtState_p, tEplNmtState OldNmtState_p)
{
tEplKernel      Ret = kEplSuccessful;

    switch (NewNmtState_p)
    {
        case kEplNmtGsOff:
        case kEplNmtGsInitialising:
        {
            // set EC flag in Flag 1, so the MN can detect a reboot and
            // will initialize the Error Signaling.
            EplDllkInstance_g.m_bFlag1 = EPL_FRAME_FLAG1_EC;

            // fall-through
        }

        case kEplNmtGsResetApplication:
        case kEplNmtGsResetCommunication:
        case kEplNmtGsResetConfiguration:
        {
            // at first, update NMT state in instance structure to disable frame processing
            EplDllkInstance_g.m_NmtState = NewNmtState_p;

            if (OldNmtState_p > kEplNmtGsResetConfiguration)
            {
                // deinitialize DLL and destroy frames
                Ret = EplDllkProcessDestroy(OldNmtState_p);
            }
            break;
        }

        // node listens for EPL-Frames and check timeout
        case kEplNmtMsNotActive:
        case kEplNmtCsNotActive:
        {
            if (OldNmtState_p <= kEplNmtGsResetConfiguration)
            {
                // setup DLL and create frames
                Ret = EplDllkProcessCreate(NewNmtState_p);
            }
            break;
        }

        // node processes only async frames
        case kEplNmtCsPreOperational1:
        {
#if EPL_TIMER_USE_HIGHRES != FALSE
            Ret = EplTimerHighReskDeleteTimer(&EplDllkInstance_g.m_TimerHdlCycle);
            if (Ret != kEplSuccessful)
            {
                goto Exit;
            }
#endif

#if (EPL_DLL_PROCESS_SYNC == EPL_DLL_PROCESS_SYNC_ON_TIMER)
            Ret = EplTimerSynckStopSync();
            if (Ret != kEplSuccessful)
            {
                goto Exit;
            }
#endif

#if EPL_DLL_PRES_CHAINING_CN != FALSE
            Ret = EplDllkPResChainingDisable();
            if (Ret != kEplSuccessful)
            {
                goto Exit;
            }
#endif

            // update IdentRes and StatusRes
            Ret = EplDllkUpdateFrameStatusRes(&EplDllkInstance_g.m_pTxBuffer[EPL_DLLK_TXFRAME_STATUSRES + EplDllkInstance_g.m_bCurTxBufferOffsetStatusRes],
                                              NewNmtState_p);
            if (Ret != kEplSuccessful)
            {
                goto Exit;
            }

            Ret = EplDllkUpdateFrameIdentRes(&EplDllkInstance_g.m_pTxBuffer[EPL_DLLK_TXFRAME_IDENTRES + EplDllkInstance_g.m_bCurTxBufferOffsetIdentRes],
                                             NewNmtState_p);
            if (Ret != kEplSuccessful)
            {
                goto Exit;
            }

            // enable IdentRes and StatusRes
#if (EDRV_AUTO_RESPONSE != FALSE)
            // enable corresponding Rx filter
            EplDllkInstance_g.m_aFilter[EPL_DLLK_FILTER_SOA_STATREQ].m_fEnable = TRUE;
            Ret = EdrvChangeFilter(EplDllkInstance_g.m_aFilter,
                                   EPL_DLLK_FILTER_COUNT,
                                   EPL_DLLK_FILTER_SOA_STATREQ,
                                   EDRV_FILTER_CHANGE_STATE);
            if (Ret != kEplSuccessful)
            {
                goto Exit;
            }

            // enable corresponding Rx filter
            EplDllkInstance_g.m_aFilter[EPL_DLLK_FILTER_SOA_IDREQ].m_fEnable = TRUE;
            Ret = EdrvChangeFilter(EplDllkInstance_g.m_aFilter,
                                   EPL_DLLK_FILTER_COUNT,
                                   EPL_DLLK_FILTER_SOA_IDREQ,
                                   EDRV_FILTER_CHANGE_STATE);
            if (Ret != kEplSuccessful)
            {
                goto Exit;
            }

#if EPL_DLL_PRES_CHAINING_CN != FALSE
            // enable SyncReq Rx filter
            EplDllkInstance_g.m_aFilter[EPL_DLLK_FILTER_SOA_SYNCREQ].m_fEnable = TRUE;
            Ret = EdrvChangeFilter(EplDllkInstance_g.m_aFilter,
                                   EPL_DLLK_FILTER_COUNT,
                                   EPL_DLLK_FILTER_SOA_SYNCREQ,
                                   EDRV_FILTER_CHANGE_STATE);
            if (Ret != kEplSuccessful)
            {
                goto Exit;
            }
#endif
#endif

            // update PRes (for sudden changes to PreOp2)
            Ret = EplDllkUpdateFramePres(&EplDllkInstance_g.m_pTxBuffer[EPL_DLLK_TXFRAME_PRES + (EplDllkInstance_g.m_bCurTxBufferOffsetCycle ^ 1)],
                                         NewNmtState_p);
            if (Ret != kEplSuccessful)
            {
                goto Exit;
            }
            Ret = EplDllkUpdateFramePres(&EplDllkInstance_g.m_pTxBuffer[EPL_DLLK_TXFRAME_PRES + EplDllkInstance_g.m_bCurTxBufferOffsetCycle],
                                         NewNmtState_p);
            if (Ret != kEplSuccessful)
            {
                goto Exit;
            }

            // enable PRes (for sudden changes to PreOp2)
#if (EDRV_AUTO_RESPONSE != FALSE)
            // enable corresponding Rx filter
            EplDllkInstance_g.m_aFilter[EPL_DLLK_FILTER_PREQ].m_fEnable = TRUE;
            EplDllkInstance_g.m_aFilter[EPL_DLLK_FILTER_PREQ].m_pTxBuffer = &EplDllkInstance_g.m_pTxBuffer[EPL_DLLK_TXFRAME_PRES];
            Ret = EdrvChangeFilter(EplDllkInstance_g.m_aFilter,
                                   EPL_DLLK_FILTER_COUNT,
                                   EPL_DLLK_FILTER_PREQ,
                                   EDRV_FILTER_CHANGE_STATE | EDRV_FILTER_CHANGE_AUTO_RESPONSE);
            if (Ret != kEplSuccessful)
            {
                goto Exit;
            }
#endif

            break;
        }

        // node processes isochronous and asynchronous frames
        case kEplNmtCsPreOperational2:
        {
            // signal update of IdentRes and StatusRes on SoA
            EplDllkInstance_g.m_bUpdateTxFrame = EPL_DLLK_UPDATE_BOTH;

            // enable PRes (necessary if coming from Stopped)
#if (EDRV_AUTO_RESPONSE != FALSE)
            // enable corresponding Rx filter
            EplDllkInstance_g.m_aFilter[EPL_DLLK_FILTER_PREQ].m_fEnable = TRUE;
            EplDllkInstance_g.m_aFilter[EPL_DLLK_FILTER_PREQ].m_pTxBuffer = &EplDllkInstance_g.m_pTxBuffer[EPL_DLLK_TXFRAME_PRES];
            Ret = EdrvChangeFilter(EplDllkInstance_g.m_aFilter,
                                   EPL_DLLK_FILTER_COUNT,
                                   EPL_DLLK_FILTER_PREQ,
                                   EDRV_FILTER_CHANGE_STATE | EDRV_FILTER_CHANGE_AUTO_RESPONSE);
            if (Ret != kEplSuccessful)
            {
                goto Exit;
            }
#endif
            break;
        }

#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)
        case kEplNmtMsPreOperational1:
        {
#if EPL_TIMER_USE_HIGHRES != FALSE
            Ret = EplTimerHighReskDeleteTimer(&EplDllkInstance_g.m_TimerHdlCycle);
            if (Ret != kEplSuccessful)
            {
                goto Exit;
            }
#endif

#if (EPL_DLL_DISABLE_EDRV_CYCLIC == FALSE)
            Ret = EdrvCyclicStopCycle();
            if (Ret != kEplSuccessful)
            {
                goto Exit;
            }
#endif

            // update IdentRes and StatusRes
            Ret = EplDllkUpdateFrameIdentRes(&EplDllkInstance_g.m_pTxBuffer[EPL_DLLK_TXFRAME_IDENTRES + EplDllkInstance_g.m_bCurTxBufferOffsetIdentRes],
                                             NewNmtState_p);
            if (Ret != kEplSuccessful)
            {
                goto Exit;
            }
            Ret = EplDllkUpdateFrameStatusRes(&EplDllkInstance_g.m_pTxBuffer[EPL_DLLK_TXFRAME_STATUSRES + EplDllkInstance_g.m_bCurTxBufferOffsetStatusRes],
                                              NewNmtState_p);

            break;
        }

        case kEplNmtMsPreOperational2:
        case kEplNmtMsReadyToOperate:
        case kEplNmtMsOperational:
#endif // (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)

        case kEplNmtCsOperational:
        case kEplNmtCsReadyToOperate:
        {
            // signal update of IdentRes and StatusRes on SoA
            EplDllkInstance_g.m_bUpdateTxFrame = EPL_DLLK_UPDATE_BOTH;
            break;
        }

        // node stopped by MN
        case kEplNmtCsStopped:
        {
            // signal update of IdentRes and StatusRes on SoA
            EplDllkInstance_g.m_bUpdateTxFrame = EPL_DLLK_UPDATE_BOTH;

#if (EDRV_AUTO_RESPONSE != FALSE)
            // disable auto-response for PRes filter
            EplDllkInstance_g.m_aFilter[EPL_DLLK_FILTER_PREQ].m_pTxBuffer = NULL;
            Ret = EdrvChangeFilter(EplDllkInstance_g.m_aFilter,
                                   EPL_DLLK_FILTER_COUNT,
                                   EPL_DLLK_FILTER_PREQ,
                                   EDRV_FILTER_CHANGE_AUTO_RESPONSE);
            if (Ret != kEplSuccessful)
            {
                goto Exit;
            }
#endif

            // update PRes
            Ret = EplDllkUpdateFramePres(&EplDllkInstance_g.m_pTxBuffer[EPL_DLLK_TXFRAME_PRES + (EplDllkInstance_g.m_bCurTxBufferOffsetCycle ^ 1)],
                                         NewNmtState_p);
            if (Ret != kEplSuccessful)
            {
                goto Exit;
            }
            Ret = EplDllkUpdateFramePres(&EplDllkInstance_g.m_pTxBuffer[EPL_DLLK_TXFRAME_PRES + EplDllkInstance_g.m_bCurTxBufferOffsetCycle],
                                         NewNmtState_p);
            if (Ret != kEplSuccessful)
            {
                goto Exit;
            }
            break;
        }

        // no EPL cycle
        // -> normal ethernet communication
        case kEplNmtMsBasicEthernet:
        case kEplNmtCsBasicEthernet:
        {
            // Fill Async Tx Buffer, because state BasicEthernet was entered
            Ret = EplDllkProcessFillTx(kEplDllAsyncReqPrioGeneric, NewNmtState_p);
            if (Ret != kEplSuccessful)
            {
                goto Exit;
            }

            break;
        }

        default:
        {
            Ret = kEplNmtInvalidState;
            goto Exit;
        }
    }

    // update NMT state in instance structure
    // This is done after updating all Tx frames,
    // so no frame will be transmitted by callback function, when it is not up to date yet.
    EplDllkInstance_g.m_NmtState = NewNmtState_p;

Exit:
    return Ret;
}


//---------------------------------------------------------------------------
//
// Function:    EplDllkProcessNmtEvent
//
// Description: process the NMT event
//
// Parameters:  pEvent_p                = event to be processed
//
// Returns:     tEplKernel              = error code
//
//
// State:
//
//---------------------------------------------------------------------------

static tEplKernel EplDllkProcessNmtEvent(tEplEvent * pEvent_p)
{
tEplKernel      Ret = kEplSuccessful;
tEplNmtEvent*   pNmtEvent;
tEplNmtState    NmtState;

    pNmtEvent = (tEplNmtEvent*) pEvent_p->m_pArg;

    switch (*pNmtEvent)
    {
        case kEplNmtEventDllCeSoa:
        {   // do preprocessing for next cycle

            NmtState = EplDllkInstance_g.m_NmtState;

#if (EPL_DLL_PROCESS_SYNC == EPL_DLL_PROCESS_SYNC_ON_SOA)
            if (EplDllkInstance_g.m_DllState != kEplDllGsInit)
            {   // cyclic state is active, so preprocessing is necessary

                Ret = EplDllkProcessSync(NmtState);
            }
//            BENCHMARK_MOD_02_TOGGLE(7);
#endif

            Ret = EplDllkProcessCycleFinish(NmtState);

            break;
        }

        default:
        {
            break;
        }
    }

//Exit:
    return Ret;
}


//---------------------------------------------------------------------------
//
// Function:    EplDllkProcessCycleFinish
//
// Description: process the CycleFinish event
//
// Parameters:  NmtState_p              = current NMT state
//
// Returns:     tEplKernel              = error code
//
//
// State:
//
//---------------------------------------------------------------------------

static tEplKernel EplDllkProcessCycleFinish(tEplNmtState NmtState_p)
{
tEplKernel      Ret = kEplReject;
tEdrvTxBuffer*  pTxBuffer;

    switch (EplDllkInstance_g.m_bUpdateTxFrame)
    {
        case EPL_DLLK_UPDATE_BOTH:
        {
            EplDllkInstance_g.m_bCurTxBufferOffsetIdentRes ^= 1;
            pTxBuffer = &EplDllkInstance_g.m_pTxBuffer[EPL_DLLK_TXFRAME_IDENTRES + EplDllkInstance_g.m_bCurTxBufferOffsetIdentRes];
            if (pTxBuffer->m_pbBuffer != NULL)
            {   // IdentRes does exist

                Ret = EplDllkUpdateFrameIdentRes(pTxBuffer, NmtState_p);
                if (Ret != kEplSuccessful)
                {
                    goto Exit;
                }
            }

            // fall-through
        }

        case EPL_DLLK_UPDATE_STATUS:
        {
            EplDllkInstance_g.m_bCurTxBufferOffsetStatusRes ^= 1;
            pTxBuffer = &EplDllkInstance_g.m_pTxBuffer[EPL_DLLK_TXFRAME_STATUSRES + EplDllkInstance_g.m_bCurTxBufferOffsetStatusRes];
            if (pTxBuffer->m_pbBuffer != NULL)
            {   // StatusRes does exist

                Ret = EplDllkUpdateFrameStatusRes(pTxBuffer, NmtState_p);
                if (Ret != kEplSuccessful)
                {
                    goto Exit;
                }
            }

            // reset signal variable
            EplDllkInstance_g.m_bUpdateTxFrame = EPL_DLLK_UPDATE_NONE;
            break;
        }

        default:
        {
            break;
        }
    }

    Ret = EplErrorHandlerkCycleFinished((NmtState_p >= kEplNmtMsNotActive));

#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)
#if EPL_DLL_DISABLE_EDRV_CYCLIC == FALSE
    if (EplDllkInstance_g.m_DllState > kEplDllMsNonCyclic)
    {
        if (EplDllkInstance_g.m_DllConfigParam.m_uiSyncNodeId == EPL_C_ADR_SYNC_ON_SOC)
        {   // cyclic state is active, so preprocessing is necessary

            Ret = EplDllkProcessSync(NmtState_p);
        }
    }
#endif
#endif

Exit:
    return Ret;
}


//---------------------------------------------------------------------------
//
// Function:    EplDllkProcessSync
//
// Description: process the Sync event
//
// Parameters:  NmtState_p              = current NMT state
//
// Returns:     tEplKernel              = error code
//
//
// State:
//
//---------------------------------------------------------------------------

static tEplKernel EplDllkProcessSync(tEplNmtState NmtState_p)
{
tEplKernel      Ret = kEplReject;
BOOL            fReadyFlag = FALSE;
tEplFrame*      pTxFrame;
tEdrvTxBuffer*  pTxBuffer;
tEplFrameInfo   FrameInfo;
unsigned int    uiNextTxBufferOffset = EplDllkInstance_g.m_bCurTxBufferOffsetCycle ^ 1;

    if (EplDllkInstance_g.m_pfnCbSync != NULL)
    {
        Ret = EplDllkInstance_g.m_pfnCbSync();
        if (Ret == kEplReject)
        {
            fReadyFlag = FALSE;
        }
        else if (Ret == kEplSuccessful)
        {
            fReadyFlag = TRUE;
        }
        else
        {
            goto Exit;
        }
    }

    // do cycle preparation

#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)

    if (NmtState_p >= kEplNmtMsNotActive)
    {   // local node is MN
    tEplDllkNodeInfo*   pIntNodeInfo;
    BYTE                bFlag1;
    BYTE*               pbCnNodeId;
#if (EPL_DLL_DISABLE_EDRV_CYCLIC == FALSE)
    unsigned int        uiIndex = 0;
    DWORD               dwNextTimeOffsetNs = 0;
    DWORD               dwAccFrameLenNs = 0;

        pTxBuffer = &EplDllkInstance_g.m_pTxBuffer[EPL_DLLK_TXFRAME_SOC + uiNextTxBufferOffset];
        pTxBuffer->m_dwTimeOffsetNs = dwNextTimeOffsetNs;
        if (EplDllkInstance_g.m_ppTxBufferList == NULL)
        {
            goto Exit;
        }
        EplDllkInstance_g.m_ppTxBufferList[uiIndex] = pTxBuffer;
        uiIndex++;

        // calculate WaitSoCPReq delay
        if (EplDllkInstance_g.m_DllConfigParam.m_dwWaitSocPreq != 0)
        {
            dwNextTimeOffsetNs = EplDllkInstance_g.m_DllConfigParam.m_dwWaitSocPreq
                                + EPL_C_DLL_T_PREAMBLE + EPL_C_DLL_T_MIN_FRAME + EPL_C_DLL_T_IFG;
        }
        else
        {
            dwAccFrameLenNs = EPL_C_DLL_T_PREAMBLE + EPL_C_DLL_T_MIN_FRAME + EPL_C_DLL_T_IFG;
        }
#endif

        pbCnNodeId = &EplDllkInstance_g.m_aabCnNodeIdList[uiNextTxBufferOffset][0];

        if (NmtState_p != kEplNmtMsOperational)
        {
            fReadyFlag = FALSE;
        }

        pIntNodeInfo = EplDllkInstance_g.m_pFirstNodeInfo;
        while (pIntNodeInfo != NULL)
        {
            pTxBuffer = &pIntNodeInfo->m_pPreqTxBuffer[uiNextTxBufferOffset];
            if ((pTxBuffer != NULL) && (pTxBuffer->m_pbBuffer != NULL))
            {   // PReq does exist
                pTxFrame = (tEplFrame *) pTxBuffer->m_pbBuffer;

                bFlag1 = pIntNodeInfo->m_bSoaFlag1 & EPL_FRAME_FLAG1_EA;

                // $$$ d.k. set EPL_FRAME_FLAG1_MS if necessary
                // update frame (Flag1)
                AmiSetByteToLe(&pTxFrame->m_Data.m_Preq.m_le_bFlag1, bFlag1);

                // process TPDO
                FrameInfo.m_pFrame = pTxFrame;
                FrameInfo.m_uiFrameSize = pTxBuffer->m_uiTxMsgLen;
                Ret = EplDllkProcessTpdo(&FrameInfo, fReadyFlag);
                if (Ret != kEplSuccessful)
                {
                    goto Exit;
                }

#if (EPL_DLL_DISABLE_EDRV_CYCLIC == FALSE)
                pTxBuffer->m_dwTimeOffsetNs = dwNextTimeOffsetNs;
                EplDllkInstance_g.m_ppTxBufferList[uiIndex] = pTxBuffer;
                uiIndex++;
#endif

                if (pTxBuffer == &EplDllkInstance_g.m_pTxBuffer[EPL_DLLK_TXFRAME_PRES + uiNextTxBufferOffset])
                {   // PRes of MN will be sent
                    // update NMT state
                    AmiSetByteToLe(&pTxFrame->m_Data.m_Pres.m_le_bNmtStatus, (BYTE) NmtState_p);
#if (EPL_DLL_DISABLE_EDRV_CYCLIC == FALSE)
#if EPL_DLL_PRES_CHAINING_MN != FALSE
                    dwNextTimeOffsetNs = pIntNodeInfo->m_dwPresTimeoutNs;
#else
                    dwNextTimeOffsetNs = 0;
#endif
#else
                    pTxBuffer->m_pfnTxHandler = EplDllkCbTransmittedPres;
#endif

#if EPL_DLL_PRES_CHAINING_MN != FALSE
                    {
                    tEplDllkNodeInfo*   pIntPrcNodeInfo;

                        pIntPrcNodeInfo = EplDllkInstance_g.m_pFirstPrcNodeInfo;
                        while (pIntPrcNodeInfo != NULL)
                        {
                            *pbCnNodeId = (BYTE) pIntPrcNodeInfo->m_uiNodeId;
                            pbCnNodeId++;
#if (EPL_DLL_DISABLE_EDRV_CYCLIC == FALSE)
                            dwNextTimeOffsetNs = pIntNodeInfo->m_dwPresTimeoutNs;
#endif

                            pIntPrcNodeInfo = pIntPrcNodeInfo->m_pNextNodeInfo;
                        }

                        *pbCnNodeId = EPL_C_ADR_BROADCAST;    // mark this entry as PRC slot finished
                        pbCnNodeId++;
                    }
#endif

                }
                else
                {   // PReq to CN
                    *pbCnNodeId = (BYTE) pIntNodeInfo->m_uiNodeId;
                    pbCnNodeId++;
#if (EPL_DLL_DISABLE_EDRV_CYCLIC == FALSE)
                    dwNextTimeOffsetNs = pIntNodeInfo->m_dwPresTimeoutNs;
#endif
                }

#if (EPL_DLL_DISABLE_EDRV_CYCLIC == FALSE)
                if (dwNextTimeOffsetNs == 0)
                {   // add SoC frame length
                    dwAccFrameLenNs += EPL_C_DLL_T_PREAMBLE + (pTxBuffer->m_uiTxMsgLen * EPL_C_DLL_T_BITTIME) + EPL_C_DLL_T_IFG;
                }
                else
                {
                    dwNextTimeOffsetNs += dwAccFrameLenNs;
                    dwAccFrameLenNs = 0;
                }
#endif
            }

            pIntNodeInfo = pIntNodeInfo->m_pNextNodeInfo;
        }

        *pbCnNodeId = EPL_C_ADR_INVALID;    // mark last entry in node-ID list

#if (EPL_DLL_DISABLE_EDRV_CYCLIC == FALSE)
        pTxBuffer = &EplDllkInstance_g.m_pTxBuffer[EPL_DLLK_TXFRAME_SOA + uiNextTxBufferOffset];
        pTxBuffer->m_dwTimeOffsetNs = dwNextTimeOffsetNs;
        // switch SoAReq buffer
        EplDllkInstance_g.m_bSyncLastSoaReq++;
        if (EplDllkInstance_g.m_bSyncLastSoaReq >= EPL_DLLK_SOAREQ_COUNT)
        {
            EplDllkInstance_g.m_bSyncLastSoaReq = 0;
        }

        // $$$ d.k. fEnableInvitation_p = ((NmtState_p != kEplNmtMsPreOperational1) || (EplDllkInstance_g.m_uiCycleCount >= EPL_C_DLL_PREOP1_START_CYCLES))
        //          currently, EplDllkProcessSync is not called in PreOp1
        Ret = EplDllkUpdateFrameSoa(pTxBuffer, NmtState_p, TRUE, EplDllkInstance_g.m_bSyncLastSoaReq);
        EplDllkInstance_g.m_ppTxBufferList[uiIndex] = pTxBuffer;
        uiIndex++;

        // check if we are invited in SoA
        if (EplDllkInstance_g.m_auiLastTargetNodeId[EplDllkInstance_g.m_bSyncLastSoaReq] == EplDllkInstance_g.m_DllConfigParam.m_uiNodeId)
        {
            switch (EplDllkInstance_g.m_aLastReqServiceId[EplDllkInstance_g.m_bSyncLastSoaReq])
            {
                case kEplDllReqServiceStatus:
                {   // StatusRequest
                    pTxBuffer = &EplDllkInstance_g.m_pTxBuffer[EPL_DLLK_TXFRAME_STATUSRES + EplDllkInstance_g.m_bCurTxBufferOffsetStatusRes];
                    if (pTxBuffer->m_pbBuffer != NULL)
                    {   // StatusRes does exist
                        EplDllkInstance_g.m_ppTxBufferList[uiIndex] = pTxBuffer;
                        uiIndex++;

                        TGT_DBG_SIGNAL_TRACE_POINT(8);
                    }

                    break;
                }

                case kEplDllReqServiceIdent:
                {   // IdentRequest
                    pTxBuffer = &EplDllkInstance_g.m_pTxBuffer[EPL_DLLK_TXFRAME_IDENTRES + EplDllkInstance_g.m_bCurTxBufferOffsetIdentRes];
                    if (pTxBuffer->m_pbBuffer != NULL)
                    {   // IdentRes does exist
                        EplDllkInstance_g.m_ppTxBufferList[uiIndex] = pTxBuffer;
                        uiIndex++;

                        TGT_DBG_SIGNAL_TRACE_POINT(7);
                    }

                    break;
                }

                case kEplDllReqServiceNmtRequest:
                {   // NmtRequest
                    pTxBuffer = &EplDllkInstance_g.m_pTxBuffer[EPL_DLLK_TXFRAME_NMTREQ + EplDllkInstance_g.m_bCurTxBufferOffsetNmtReq];
                    if (pTxBuffer->m_pbBuffer != NULL)
                    {   // NmtRequest does exist
                        // check if frame is not empty and not being filled
                        if (pTxBuffer->m_uiTxMsgLen > EPL_DLLK_BUFLEN_FILLING)
                        {
                            EplDllkInstance_g.m_ppTxBufferList[uiIndex] = pTxBuffer;
                            uiIndex++;
                            EplDllkInstance_g.m_bCurTxBufferOffsetNmtReq ^= 1;
                        }
                    }

                    break;
                }

                case kEplDllReqServiceUnspecified:
                {   // unspecified invite
                    pTxBuffer = &EplDllkInstance_g.m_pTxBuffer[EPL_DLLK_TXFRAME_NONEPL + EplDllkInstance_g.m_bCurTxBufferOffsetNonEpl];
                    if (pTxBuffer->m_pbBuffer != NULL)
                    {   // non-EPL frame does exist
                        // check if frame is not empty and not being filled
                        if (pTxBuffer->m_uiTxMsgLen > EPL_DLLK_BUFLEN_FILLING)
                        {
                            EplDllkInstance_g.m_ppTxBufferList[uiIndex] = pTxBuffer;
                            uiIndex++;
                            EplDllkInstance_g.m_bCurTxBufferOffsetNonEpl ^= 1;
                        }
                    }

                    break;
                }

                default:
                {
                    break;
                }
            }

            // ASnd frame will be sent, remove the request
            EplDllkInstance_g.m_aLastReqServiceId[EplDllkInstance_g.m_bSyncLastSoaReq] = kEplDllReqServiceNo;
        }

        // set last list element to NULL
        EplDllkInstance_g.m_ppTxBufferList[uiIndex] = NULL;
        uiIndex++;

        Ret = EdrvCyclicSetNextTxBufferList(EplDllkInstance_g.m_ppTxBufferList, uiIndex);
#endif
    }
    else
#endif // (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)

    {   // local node is CN, update only the PRes

        pTxBuffer = &EplDllkInstance_g.m_pTxBuffer[EPL_DLLK_TXFRAME_PRES + uiNextTxBufferOffset];
        if (pTxBuffer->m_pbBuffer != NULL)
        {   // PRes does exist
            pTxFrame = (tEplFrame *) pTxBuffer->m_pbBuffer;

            if (NmtState_p != kEplNmtCsOperational)
            {
                fReadyFlag = FALSE;
            }

            // process TPDO
            FrameInfo.m_pFrame = pTxFrame;
            FrameInfo.m_uiFrameSize = pTxBuffer->m_uiTxMsgLen;
            Ret = EplDllkProcessTpdo(&FrameInfo, fReadyFlag);
            if (Ret != kEplSuccessful)
            {
                goto Exit;
            }

//                        BENCHMARK_MOD_02_TOGGLE(7);

            Ret = EplDllkUpdateFramePres(pTxBuffer, NmtState_p);
            if (Ret != kEplSuccessful)
            {
                goto Exit;
            }

            // switch to next cycle
            EplDllkInstance_g.m_bCurTxBufferOffsetCycle = (BYTE) uiNextTxBufferOffset;
        }

    }

Exit:
    return Ret;
}
//---------------------------------------------------------------------------
//
// Function:    EplDllkProcessFillTx
//
// Description: process the FillTx event
//
// Parameters:  AsyncReqPriority_p      = priority of asynchronous request
//
// Returns:     tEplKernel              = error code
//
//
// State:
//
//---------------------------------------------------------------------------

static tEplKernel EplDllkProcessFillTx(tEplDllAsyncReqPriority AsyncReqPriority_p, tEplNmtState NmtState_p)
{
tEplKernel      Ret = kEplSuccessful;
tEplFrame*      pTxFrame;
tEdrvTxBuffer*  pTxBuffer;
unsigned int    uiFrameSize;
unsigned int    uiFrameCount;
unsigned int    uiNextTxBufferOffset;

    // fill TxBuffer of specified priority with new frame if empty

    pTxFrame = NULL;
    switch (AsyncReqPriority_p)
    {
        case kEplDllAsyncReqPrioNmt:    // NMT request priority
        {
            uiNextTxBufferOffset = EplDllkInstance_g.m_bCurTxBufferOffsetNmtReq;
            pTxBuffer = &EplDllkInstance_g.m_pTxBuffer[EPL_DLLK_TXFRAME_NMTREQ + uiNextTxBufferOffset];
            if (pTxBuffer->m_pbBuffer != NULL)
            {   // NmtRequest does exist
                // check if frame is empty and not being filled
                if (pTxBuffer->m_uiTxMsgLen == EPL_DLLK_BUFLEN_EMPTY)
                {
                    // mark Tx buffer as filling is in process
                    pTxBuffer->m_uiTxMsgLen = EPL_DLLK_BUFLEN_FILLING;
                    // set max buffer size as input parameter
                    uiFrameSize = pTxBuffer->m_uiMaxBufferLen;
                    // copy frame from shared loop buffer to Tx buffer
                    Ret = EplDllkCalAsyncGetTxFrame(
                        pTxBuffer->m_pbBuffer, &uiFrameSize, AsyncReqPriority_p);
                    if (Ret == kEplSuccessful)
                    {
                        pTxFrame = (tEplFrame *) pTxBuffer->m_pbBuffer;
                        Ret = EplDllkCheckFrame(pTxFrame, uiFrameSize);

                        // set buffer valid
                        pTxBuffer->m_uiTxMsgLen = uiFrameSize;

#if (EDRV_AUTO_RESPONSE != FALSE)
                        if ((NmtState_p & (EPL_NMT_TYPE_MASK | EPL_NMT_SUPERSTATE_MASK)) == (EPL_NMT_TYPE_CS | EPL_NMT_CS_EPLMODE))
                        {
                            // update Tx buffer in Edrv
                            Ret = EdrvUpdateTxMsgBuffer(pTxBuffer);

                            // enable corresponding Rx filter
                            EplDllkInstance_g.m_aFilter[EPL_DLLK_FILTER_SOA_NMTREQ].m_fEnable = TRUE;
                            Ret = EdrvChangeFilter(EplDllkInstance_g.m_aFilter,
                                                   EPL_DLLK_FILTER_COUNT,
                                                   EPL_DLLK_FILTER_SOA_NMTREQ,
                                                   EDRV_FILTER_CHANGE_STATE);
                            if (Ret != kEplSuccessful)
                            {
                                goto Exit;
                            }
                        }
#endif
                    }
                    else if (Ret == kEplDllAsyncTxBufferEmpty)
                    {   // empty Tx buffer is not a real problem
                        // so just ignore it
                        Ret = kEplSuccessful;
                        // mark Tx buffer as empty
                        pTxBuffer->m_uiTxMsgLen = EPL_DLLK_BUFLEN_EMPTY;

#if (EDRV_AUTO_RESPONSE != FALSE)
                        if ((NmtState_p & (EPL_NMT_TYPE_MASK | EPL_NMT_SUPERSTATE_MASK)) == (EPL_NMT_TYPE_CS | EPL_NMT_CS_EPLMODE))
                        {
                            // disable corresponding Rx filter
                            EplDllkInstance_g.m_aFilter[EPL_DLLK_FILTER_SOA_NMTREQ].m_fEnable = FALSE;
                            Ret = EdrvChangeFilter(EplDllkInstance_g.m_aFilter,
                                                   EPL_DLLK_FILTER_COUNT,
                                                   EPL_DLLK_FILTER_SOA_NMTREQ,
                                                   EDRV_FILTER_CHANGE_STATE);
                            if (Ret != kEplSuccessful)
                            {
                                goto Exit;
                            }
                        }
#endif
                    }
                }
            }
            break;
        }

        default:    // generic priority
        {
            uiNextTxBufferOffset = EplDllkInstance_g.m_bCurTxBufferOffsetNonEpl;
            pTxBuffer = &EplDllkInstance_g.m_pTxBuffer[EPL_DLLK_TXFRAME_NONEPL + uiNextTxBufferOffset];
            if (pTxBuffer->m_pbBuffer != NULL)
            {   // non-EPL frame does exist
                // check if frame is empty and not being filled
                if (pTxBuffer->m_uiTxMsgLen == EPL_DLLK_BUFLEN_EMPTY)
                {
                    // mark Tx buffer as filling is in process
                    pTxBuffer->m_uiTxMsgLen = EPL_DLLK_BUFLEN_FILLING;
                    // set max buffer size as input parameter
                    uiFrameSize = pTxBuffer->m_uiMaxBufferLen;
                    // copy frame from shared loop buffer to Tx buffer
                    Ret = EplDllkCalAsyncGetTxFrame(
                        pTxBuffer->m_pbBuffer, &uiFrameSize, AsyncReqPriority_p);
                    if (Ret == kEplSuccessful)
                    {
                        pTxFrame = (tEplFrame *) pTxBuffer->m_pbBuffer;
                        Ret = EplDllkCheckFrame(pTxFrame, uiFrameSize);

                        // set buffer valid
                        pTxBuffer->m_uiTxMsgLen = uiFrameSize;

#if (EDRV_AUTO_RESPONSE != FALSE)
                        if ((NmtState_p & (EPL_NMT_TYPE_MASK | EPL_NMT_SUPERSTATE_MASK)) == (EPL_NMT_TYPE_CS | EPL_NMT_CS_EPLMODE))
                        {
                            // update Tx buffer in Edrv
                            Ret = EdrvUpdateTxMsgBuffer(pTxBuffer);

                            // enable corresponding Rx filter
                            EplDllkInstance_g.m_aFilter[EPL_DLLK_FILTER_SOA_NONEPL].m_fEnable = TRUE;
                            Ret = EdrvChangeFilter(EplDllkInstance_g.m_aFilter,
                                                   EPL_DLLK_FILTER_COUNT,
                                                   EPL_DLLK_FILTER_SOA_NONEPL,
                                                   EDRV_FILTER_CHANGE_STATE);
                            if (Ret != kEplSuccessful)
                            {
                                goto Exit;
                            }
                        }
#endif
                    }
                    else if (Ret == kEplDllAsyncTxBufferEmpty)
                    {   // empty Tx buffer is not a real problem
                        // so just ignore it
                        Ret = kEplSuccessful;
                        // mark Tx buffer as empty
                        pTxBuffer->m_uiTxMsgLen = EPL_DLLK_BUFLEN_EMPTY;

#if (EDRV_AUTO_RESPONSE != FALSE)
                        if ((NmtState_p & (EPL_NMT_TYPE_MASK | EPL_NMT_SUPERSTATE_MASK)) == (EPL_NMT_TYPE_CS | EPL_NMT_CS_EPLMODE))
                        {
                            // disable corresponding Rx filter
                            EplDllkInstance_g.m_aFilter[EPL_DLLK_FILTER_SOA_NONEPL].m_fEnable = FALSE;
                            Ret = EdrvChangeFilter(EplDllkInstance_g.m_aFilter,
                                                   EPL_DLLK_FILTER_COUNT,
                                                   EPL_DLLK_FILTER_SOA_NONEPL,
                                                   EDRV_FILTER_CHANGE_STATE);
                            if (Ret != kEplSuccessful)
                            {
                                goto Exit;
                            }
                        }
#endif
                    }
                }
            }
            break;
        }
    }

    if ((NmtState_p == kEplNmtCsBasicEthernet) || (NmtState_p == kEplNmtMsBasicEthernet))
    {   // send frame immediately
        if (pTxFrame != NULL)
        {   // frame is present
            // padding is done by Edrv or ethernet controller
            Ret = EdrvSendTxMsg(pTxBuffer);
        }
        else
        {   // no frame moved to TxBuffer
            // check if TxBuffers contain unsent frames
            if (EplDllkInstance_g.m_pTxBuffer[EPL_DLLK_TXFRAME_NMTREQ + EplDllkInstance_g.m_bCurTxBufferOffsetNmtReq].m_uiTxMsgLen > EPL_DLLK_BUFLEN_EMPTY)
            {   // NMT request Tx buffer contains a frame
                Ret = EdrvSendTxMsg(
                        &EplDllkInstance_g.m_pTxBuffer[EPL_DLLK_TXFRAME_NMTREQ + EplDllkInstance_g.m_bCurTxBufferOffsetNmtReq]);
            }
            else if (EplDllkInstance_g.m_pTxBuffer[EPL_DLLK_TXFRAME_NONEPL + EplDllkInstance_g.m_bCurTxBufferOffsetNonEpl].m_uiTxMsgLen > EPL_DLLK_BUFLEN_EMPTY)
            {   // non-EPL Tx buffer contains a frame
                Ret = EdrvSendTxMsg(
                        &EplDllkInstance_g.m_pTxBuffer[EPL_DLLK_TXFRAME_NONEPL + EplDllkInstance_g.m_bCurTxBufferOffsetNonEpl]);
            }
            if (Ret == kEplInvalidOperation)
            {   // ignore error if caused by already active transmission
                Ret = kEplSuccessful;
            }
        }
        // reset PRes flag 2
        EplDllkInstance_g.m_bFlag2 = 0;
    }
    else
    {
        // update Flag 2 (PR, RS)
        Ret = EplDllkCalAsyncGetTxCount(&AsyncReqPriority_p, &uiFrameCount);
        if (AsyncReqPriority_p == kEplDllAsyncReqPrioNmt)
        {   // non-empty FIFO with hightest priority is for NMT requests
            if (EplDllkInstance_g.m_pTxBuffer[EPL_DLLK_TXFRAME_NMTREQ + EplDllkInstance_g.m_bCurTxBufferOffsetNmtReq].m_uiTxMsgLen > EPL_DLLK_BUFLEN_EMPTY)
            {   // NMT request Tx buffer contains a frame
                // add one more frame
                uiFrameCount++;
            }
        }
        else
        {   // non-empty FIFO with highest priority is for generic frames
            if (EplDllkInstance_g.m_pTxBuffer[EPL_DLLK_TXFRAME_NMTREQ + EplDllkInstance_g.m_bCurTxBufferOffsetNmtReq].m_uiTxMsgLen > EPL_DLLK_BUFLEN_EMPTY)
            {   // NMT request Tx buffer contains a frame
                // use NMT request FIFO, because of higher priority
                uiFrameCount = 1;
                AsyncReqPriority_p = kEplDllAsyncReqPrioNmt;
            }
            else if (EplDllkInstance_g.m_pTxBuffer[EPL_DLLK_TXFRAME_NONEPL + EplDllkInstance_g.m_bCurTxBufferOffsetNonEpl].m_uiTxMsgLen > EPL_DLLK_BUFLEN_EMPTY)
            {   // non-EPL Tx buffer contains a frame
                // use NMT request FIFO, because of higher priority
                // add one more frame
                uiFrameCount++;
            }
        }

        if (uiFrameCount > 7)
        {   // limit frame request to send counter to 7
            uiFrameCount = 7;
        }
        if (uiFrameCount > 0)
        {
            EplDllkInstance_g.m_bFlag2 =
                (BYTE) (((AsyncReqPriority_p << EPL_FRAME_FLAG2_PR_SHIFT) & EPL_FRAME_FLAG2_PR)
                | (uiFrameCount & EPL_FRAME_FLAG2_RS));
        }
        else
        {
            EplDllkInstance_g.m_bFlag2 = 0;
        }
        EplDllkInstance_g.m_bUpdateTxFrame = EPL_DLLK_UPDATE_BOTH;
    }

#if (EDRV_AUTO_RESPONSE != FALSE)
Exit:
#endif
    return Ret;
}


//---------------------------------------------------------------------------
//
// Function:    EplDllkProcessStartReducedCycle
//
// Description: process the StartReducedCycle event
//
// Parameters:  pEvent_p                = event to be processed
//
// Returns:     tEplKernel              = error code
//
//
// State:
//
//---------------------------------------------------------------------------

#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)
static tEplKernel EplDllkProcessStartReducedCycle(tEplEvent * pEvent_p)
{
tEplKernel      Ret = kEplSuccessful;

    UNUSED_PARAMETER(pEvent_p);

    // start the reduced cycle by programming the cycle timer
    // it is issued by NMT MN module, when PreOp1 is entered

    // clear the asynchronous queues
    Ret = EplDllkCalAsyncClearQueues();

    // reset cycle counter (everytime a SoA is triggerd in PreOp1 the counter is incremented
    // and when it reaches EPL_C_DLL_PREOP1_START_CYCLES the SoA may contain invitations)
    EplDllkInstance_g.m_uiCycleCount = 0;

    // remove any CN from isochronous phase
    while (EplDllkInstance_g.m_pFirstNodeInfo != NULL)
    {
        Ret = EplDllkDeleteNodeIsochronous(EplDllkInstance_g.m_pFirstNodeInfo);
        if (Ret != kEplSuccessful)
        {
            goto Exit;
        }
    }

#if EPL_DLL_PRES_CHAINING_MN != FALSE
    while (EplDllkInstance_g.m_pFirstPrcNodeInfo != NULL)
    {
        Ret = EplDllkDeleteNodeIsochronous(EplDllkInstance_g.m_pFirstPrcNodeInfo);
        if (Ret != kEplSuccessful)
        {
            goto Exit;
        }
    }
#endif

    // change state to NonCyclic,
    // hence EplDllkChangeState() will not ignore the next call
    EplDllkInstance_g.m_DllState = kEplDllMsNonCyclic;

#if EPL_TIMER_USE_HIGHRES != FALSE
    if (EplDllkInstance_g.m_DllConfigParam.m_dwAsyncSlotTimeout != 0)
    {
        Ret = EplTimerHighReskModifyTimerNs(&EplDllkInstance_g.m_TimerHdlCycle,
            EplDllkInstance_g.m_DllConfigParam.m_dwAsyncSlotTimeout,
            EplDllkCbMnTimerCycle,
            0L,
            FALSE);
    }
#endif

    EplDllkInstance_g.m_bCurLastSoaReq = 0;
    EplDllkInstance_g.m_bCurTxBufferOffsetCycle = 0;

Exit:
    return Ret;
}
#endif // (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)


//---------------------------------------------------------------------------
//
// Function:    EplDllkChangeState
//
// Description: change DLL state on event and diagnose some communication errors
//
// Parameters:  NmtEvent_p              = DLL event (wrapped in NMT event)
//
// Returns:     tEplKernel              = error code
//
//
// State:
//
//---------------------------------------------------------------------------

static tEplKernel EplDllkChangeState(tEplNmtEvent NmtEvent_p, tEplNmtState NmtState_p)
{
tEplKernel              Ret = kEplSuccessful;
tEplErrorHandlerkEvent  DllEvent;

    DllEvent.m_ulDllErrorEvents = 0;
    DllEvent.m_uiNodeId = 0;
    DllEvent.m_NmtState = NmtState_p;
    DllEvent.m_EplError = kEplSuccessful;

    switch (NmtState_p)
    {
        case kEplNmtGsOff:
        case kEplNmtGsInitialising:
        case kEplNmtGsResetApplication:
        case kEplNmtGsResetCommunication:
        case kEplNmtGsResetConfiguration:
        case kEplNmtCsBasicEthernet:
            // enter DLL_GS_INIT
            EplDllkInstance_g.m_DllState = kEplDllGsInit;
            break;

        case kEplNmtCsNotActive:
        case kEplNmtCsPreOperational1:
            // reduced EPL cycle is active
            if (NmtEvent_p == kEplNmtEventDllCeSoc)
            {   // SoC received
                // enter DLL_CS_WAIT_PREQ
                EplDllkInstance_g.m_DllState = kEplDllCsWaitPreq;
            }
            else
            {
                // enter DLL_GS_INIT
                EplDllkInstance_g.m_DllState = kEplDllGsInit;
            }
            break;

        case kEplNmtCsPreOperational2:
        case kEplNmtCsReadyToOperate:
        case kEplNmtCsOperational:
            // full EPL cycle is active

            switch (EplDllkInstance_g.m_DllState)
            {
                case kEplDllCsWaitPreq:
                    switch (NmtEvent_p)
                    {
                            // DLL_CT2
                        case kEplNmtEventDllCePreq:
                            // enter DLL_CS_WAIT_SOA
                            DllEvent.m_ulDllErrorEvents |= EPL_DLL_ERR_CN_RECVD_PREQ;
                            EplDllkInstance_g.m_DllState = kEplDllCsWaitSoa;
                            break;

                            // DLL_CT8
                        case kEplNmtEventDllCeFrameTimeout:
                            if (NmtState_p == kEplNmtCsPreOperational2)
                            {   // ignore frame timeout in PreOp2,
                                // because the previously configured cycle len
                                // may be wrong.
                                // 2008/10/15 d.k. If it would not be ignored,
                                // we would go cyclically to PreOp1 and on next
                                // SoC back to PreOp2.
                                break;
                            }

                            // report DLL_CEV_LOSS_SOC and DLL_CEV_LOSS_SOA
                            DllEvent.m_ulDllErrorEvents |= EPL_DLL_ERR_CN_LOSS_SOA | EPL_DLL_ERR_CN_LOSS_SOC;

                            // enter DLL_CS_WAIT_SOC
                            EplDllkInstance_g.m_DllState = kEplDllCsWaitSoc;
                            break;

                        case kEplNmtEventDllCeSoa:
                            // check if multiplexed and PReq should have been received in this cycle
                            // and if >= NMT_CS_READY_TO_OPERATE
                            if ((EplDllkInstance_g.m_uiCycleCount == 0)
                                && (NmtState_p >= kEplNmtCsReadyToOperate))
                            {   // report DLL_CEV_LOSS_OF_PREQ
                                DllEvent.m_ulDllErrorEvents |= EPL_DLL_ERR_CN_LOSS_PREQ;
                            }

                            // enter DLL_CS_WAIT_SOC
                            EplDllkInstance_g.m_DllState = kEplDllCsWaitSoc;
                            break;

                            // DLL_CT7
                        case kEplNmtEventDllCeSoc:
                        case kEplNmtEventDllCeAsnd:
                            // report DLL_CEV_LOSS_SOA
                            DllEvent.m_ulDllErrorEvents |= EPL_DLL_ERR_CN_LOSS_SOA;

                        case kEplNmtEventDllCePres:
                        default:
                            // remain in this state
                            break;
                    }
                    break;

                case kEplDllCsWaitSoc:
                    switch (NmtEvent_p)
                    {
                            // DLL_CT1
                        case kEplNmtEventDllCeSoc:
                            // start of cycle and isochronous phase
                            // enter DLL_CS_WAIT_PREQ
                            EplDllkInstance_g.m_DllState = kEplDllCsWaitPreq;
                            break;

                            // DLL_CT4
//                        case kEplNmtEventDllCePres:
                        case kEplNmtEventDllCeFrameTimeout:
                            if (NmtState_p == kEplNmtCsPreOperational2)
                            {   // ignore frame timeout in PreOp2,
                                // because the previously configured cycle len
                                // may be wrong.
                                // 2008/10/15 d.k. If it would not be ignored,
                                // we would go cyclically to PreOp1 and on next
                                // SoC back to PreOp2.
                                break;
                            }

                            // fall through

                        case kEplNmtEventDllCePreq:
                        case kEplNmtEventDllCeSoa:
                            // report DLL_CEV_LOSS_SOC
                            DllEvent.m_ulDllErrorEvents |= EPL_DLL_ERR_CN_LOSS_SOC;

                        case kEplNmtEventDllCeAsnd:
                        default:
                            // remain in this state
                            break;
                    }
                    break;

                case kEplDllCsWaitSoa:
                    switch (NmtEvent_p)
                    {
                        case kEplNmtEventDllCeFrameTimeout:
                            // DLL_CT3
                            if (NmtState_p == kEplNmtCsPreOperational2)
                            {   // ignore frame timeout in PreOp2,
                                // because the previously configured cycle len
                                // may be wrong.
                                // 2008/10/15 d.k. If it would not be ignored,
                                // we would go cyclically to PreOp1 and on next
                                // SoC back to PreOp2.
                                break;
                            }

                            // fall through

                        case kEplNmtEventDllCePreq:
                            // report DLL_CEV_LOSS_SOC and DLL_CEV_LOSS_SOA
                            DllEvent.m_ulDllErrorEvents |= EPL_DLL_ERR_CN_LOSS_SOA | EPL_DLL_ERR_CN_LOSS_SOC;

                        case kEplNmtEventDllCeSoa:
                            // enter DLL_CS_WAIT_SOC
                            EplDllkInstance_g.m_DllState = kEplDllCsWaitSoc;
                            break;

                            // DLL_CT9
                        case kEplNmtEventDllCeSoc:
                            // report DLL_CEV_LOSS_SOA
                            DllEvent.m_ulDllErrorEvents |= EPL_DLL_ERR_CN_LOSS_SOA;

                            // enter DLL_CS_WAIT_PREQ
                            EplDllkInstance_g.m_DllState = kEplDllCsWaitPreq;
                            break;

                            // DLL_CT10
                        case kEplNmtEventDllCeAsnd:
                            // report DLL_CEV_LOSS_SOA
                            DllEvent.m_ulDllErrorEvents |= EPL_DLL_ERR_CN_LOSS_SOA;

                        case kEplNmtEventDllCePres:
                        default:
                            // remain in this state
                            break;
                    }
                    break;

                case kEplDllGsInit:
                    // enter DLL_CS_WAIT_PREQ
                    EplDllkInstance_g.m_DllState = kEplDllCsWaitPreq;
                    break;

                default:
                    break;
            }
            break;

        case kEplNmtCsStopped:
            // full EPL cycle is active, but without PReq/PRes

            switch (EplDllkInstance_g.m_DllState)
            {
                case kEplDllCsWaitPreq:
                    switch (NmtEvent_p)
                    {
                            // DLL_CT2
                        case kEplNmtEventDllCePreq:
                            // enter DLL_CS_WAIT_SOA
                            EplDllkInstance_g.m_DllState = kEplDllCsWaitSoa;
                            break;

                            // DLL_CT8
                        case kEplNmtEventDllCeFrameTimeout:
                            // report DLL_CEV_LOSS_SOC and DLL_CEV_LOSS_SOA
                            DllEvent.m_ulDllErrorEvents |= EPL_DLL_ERR_CN_LOSS_SOA | EPL_DLL_ERR_CN_LOSS_SOC;

                        case kEplNmtEventDllCeSoa:
                            // NMT_CS_STOPPED active
                            // it is Ok if no PReq was received

                            // enter DLL_CS_WAIT_SOC
                            EplDllkInstance_g.m_DllState = kEplDllCsWaitSoc;
                            break;

                            // DLL_CT7
                        case kEplNmtEventDllCeSoc:
                        case kEplNmtEventDllCeAsnd:
                            // report DLL_CEV_LOSS_SOA
                            DllEvent.m_ulDllErrorEvents |= EPL_DLL_ERR_CN_LOSS_SOA;

                        case kEplNmtEventDllCePres:
                        default:
                            // remain in this state
                            break;
                    }
                    break;

                case kEplDllCsWaitSoc:
                    switch (NmtEvent_p)
                    {
                            // DLL_CT1
                        case kEplNmtEventDllCeSoc:
                            // start of cycle and isochronous phase
                            // enter DLL_CS_WAIT_SOA
                            EplDllkInstance_g.m_DllState = kEplDllCsWaitSoa;
                            break;

                            // DLL_CT4
//                        case kEplNmtEventDllCePres:
                        case kEplNmtEventDllCePreq:
                        case kEplNmtEventDllCeSoa:
                        case kEplNmtEventDllCeFrameTimeout:
                            // report DLL_CEV_LOSS_SOC
                            DllEvent.m_ulDllErrorEvents |= EPL_DLL_ERR_CN_LOSS_SOC;

                        case kEplNmtEventDllCeAsnd:
                        default:
                            // remain in this state
                            break;
                    }
                    break;

                case kEplDllCsWaitSoa:
                    switch (NmtEvent_p)
                    {
                            // DLL_CT3
                        case kEplNmtEventDllCeFrameTimeout:
                            // report DLL_CEV_LOSS_SOC and DLL_CEV_LOSS_SOA
                            DllEvent.m_ulDllErrorEvents |= EPL_DLL_ERR_CN_LOSS_SOA | EPL_DLL_ERR_CN_LOSS_SOC;

                        case kEplNmtEventDllCeSoa:
                            // enter DLL_CS_WAIT_SOC
                            EplDllkInstance_g.m_DllState = kEplDllCsWaitSoc;
                            break;

                            // DLL_CT9
                        case kEplNmtEventDllCeSoc:
                            // report DLL_CEV_LOSS_SOA
                            DllEvent.m_ulDllErrorEvents |= EPL_DLL_ERR_CN_LOSS_SOA;
                            // remain in DLL_CS_WAIT_SOA
                            break;

                            // DLL_CT10
                        case kEplNmtEventDllCeAsnd:
                            // report DLL_CEV_LOSS_SOA
                            DllEvent.m_ulDllErrorEvents |= EPL_DLL_ERR_CN_LOSS_SOA;

                        case kEplNmtEventDllCePreq:
                            // NMT_CS_STOPPED active and we do not expect any PReq
                            // so just ignore it
                        case kEplNmtEventDllCePres:
                        default:
                            // remain in this state
                            break;
                    }
                    break;

                case kEplDllGsInit:
                default:
                    // enter DLL_CS_WAIT_PREQ
                    EplDllkInstance_g.m_DllState = kEplDllCsWaitSoa;
                    break;
            }
            break;

#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)
        case kEplNmtMsNotActive:
        case kEplNmtMsBasicEthernet:
            break;

        case kEplNmtMsPreOperational1:
            // reduced EPL cycle is active
            if (EplDllkInstance_g.m_DllState != kEplDllMsNonCyclic)
            {   // stop cycle timer
#if EPL_DLL_DISABLE_EDRV_CYCLIC != FALSE
#if EPL_TIMER_USE_HIGHRES != FALSE
                Ret = EplTimerHighReskDeleteTimer(&EplDllkInstance_g.m_TimerHdlCycle);
                if (Ret != kEplSuccessful)
                {
                    goto Exit;
                }
#endif
#else
                Ret = EdrvCyclicStopCycle();
                if (Ret != kEplSuccessful)
                {
                    goto Exit;
                }
#endif
                EplDllkInstance_g.m_DllState = kEplDllMsNonCyclic;

                // stop further processing,
                // because it will be restarted by NMT MN module
                break;
            }

            switch (NmtEvent_p)
            {
                case kEplNmtEventDllMeSocTrig:
                case kEplNmtEventDllCeAsnd:
                {   // because of reduced EPL cycle SoA shall be triggered, not SoC
                tEplDllState    DummyDllState;

                    Ret = EplDllkAsyncFrameNotReceived(EplDllkInstance_g.m_aLastReqServiceId[EplDllkInstance_g.m_bCurLastSoaReq],
                                                       EplDllkInstance_g.m_auiLastTargetNodeId[EplDllkInstance_g.m_bCurLastSoaReq]);
                    if (Ret != kEplSuccessful)
                    {
                        goto Exit;
                    }

                    // $$$ d.k. only continue with sending of the SoA, if the received ASnd was the requested one
                    //          or the transmission of the previous SoA has already finished.
                    //          If we receive multiple ASnd after a SoA, we will get kEplInvalidOperation in SendSoa()
                    //          otherwise.

                    // go ahead and send SoA
                    Ret = EplDllkMnSendSoa(NmtState_p,
                                           &DummyDllState,
                                           (EplDllkInstance_g.m_uiCycleCount >= EPL_C_DLL_PREOP1_START_CYCLES));

                    // increment cycle counter to detect if EPL_C_DLL_PREOP1_START_CYCLES empty cycles are elapsed
                    EplDllkInstance_g.m_uiCycleCount++;

                    Ret = kEplSuccessful;
/*
                    if (Ret != kEplSuccessful)
                    {
                        goto Exit;
                    }
*/
                    // reprogramming of timer will be done in CbFrameTransmitted()
                    break;
                }

                default:
                    break;
            }
            break;

        case kEplNmtMsPreOperational2:
        case kEplNmtMsReadyToOperate:
        case kEplNmtMsOperational:
        {

            // full EPL cycle is active
            switch (NmtEvent_p)
            {
                case kEplNmtEventDllMeSocTrig:
                {
                    // update cycle counter
                    if (EplDllkInstance_g.m_DllConfigParam.m_uiMultiplCycleCnt > 0)
                    {   // multiplexed cycle active
                        EplDllkInstance_g.m_uiCycleCount = (EplDllkInstance_g.m_uiCycleCount + 1) % EplDllkInstance_g.m_DllConfigParam.m_uiMultiplCycleCnt;
                        // $$$ check multiplexed cycle restart
                        //     -> toggle MC flag
                        //     -> change node linked list
                    }
                    else
                    {   // non-multiplexed cycle active
#if EPL_DLL_DISABLE_EDRV_CYCLIC != FALSE
                        // start with first node in isochronous phase
                        EplDllkInstance_g.m_pCurNodeInfo = NULL;
#endif
                    }

                    switch (EplDllkInstance_g.m_DllState)
                    {
                        case kEplDllMsNonCyclic:
                        {   // start continuous cycle timer

                            // ASndTimeout is checked on next SoC Tx callback function

#if EPL_DLL_DISABLE_EDRV_CYCLIC == FALSE

                            Ret = EplTimerHighReskDeleteTimer(&EplDllkInstance_g.m_TimerHdlCycle);
                            if (Ret != kEplSuccessful)
                            {
                                goto Exit;
                            }

                            Ret = EdrvCyclicStartCycle();
                            if (Ret != kEplSuccessful)
                            {
                                goto Exit;
                            }

                            // new DLL state
                            EplDllkInstance_g.m_DllState = kEplDllMsWaitSocTrig;

                            // initialize SoAReq number for ProcessSync (cycle preparation)
                            EplDllkInstance_g.m_bSyncLastSoaReq = EplDllkInstance_g.m_bCurLastSoaReq;

                            // forward dummy SoA event to DLLk, ErrorHandler and PDO module
                            // to trigger preparation of first cycle
                            Ret = EplDllkPostEvent(kEplEventTypeSync);
                            if (Ret != kEplSuccessful)
                            {
                                goto Exit;
                            }

#else
#if EPL_TIMER_USE_HIGHRES != FALSE
                            Ret = EplTimerHighReskModifyTimerNs(&EplDllkInstance_g.m_TimerHdlCycle,
                                EplDllkInstance_g.m_ullFrameTimeout,
                                EplDllkCbMnTimerCycle,
                                0L,
                                TRUE);
                            if (Ret != kEplSuccessful)
                            {
                                goto Exit;
                            }
#endif

                            // forward dummy SoA event to DLLk, ErrorHandler and PDO module
                            // to trigger preparation of first cycle
                            Ret = EplDllkPostEvent(kEplEventTypeDllkCycleFinish);
                            if (Ret != kEplSuccessful)
                            {
                                goto Exit;
                            }

                            // new DLL state
                            EplDllkInstance_g.m_DllState = kEplDllMsWaitSocTrig;

                            // do not continue with sending SoC,
                            // because the first cycle needs to be prepared by the DLL
                            break;
                        }

                        case kEplDllMsWaitAsnd:
                        case kEplDllMsWaitSocTrig:
                        {
                            // send SoC
                            Ret = EplDllkMnSendSoc();
                            if (Ret != kEplSuccessful)
                            {
                                goto Exit;
                            }

                            // if m_LastReqServiceId is still valid,
                            // SoA was not correctly answered
                            // and user part has to be informed
                            Ret = EplDllkAsyncFrameNotReceived(EplDllkInstance_g.m_aLastReqServiceId[EplDllkInstance_g.m_bCurLastSoaReq],
                                                               EplDllkInstance_g.m_auiLastTargetNodeId[EplDllkInstance_g.m_bCurLastSoaReq]);
                            if (Ret != kEplSuccessful)
                            {
                                goto Exit;
                            }

                            // new DLL state
                            EplDllkInstance_g.m_DllState = kEplDllMsWaitPreqTrig;

                            // start WaitSoCPReq Timer in CbFrameTransmitted()
#endif
                            break;
                        }

                        default:
                        {   // wrong DLL state / cycle time exceeded
                            DllEvent.m_ulDllErrorEvents |= EPL_DLL_ERR_MN_CYCTIMEEXCEED;
                            EplDllkInstance_g.m_DllState = kEplDllMsWaitSocTrig;
                            break;
                        }
                    }
                    break;
                }

#if EPL_DLL_DISABLE_EDRV_CYCLIC != FALSE
                case kEplNmtEventDllMePresTimeout:
                {

                    switch (EplDllkInstance_g.m_DllState)
                    {
                        case kEplDllMsWaitPres:
                        {   // PRes not received

                            if (EplDllkInstance_g.m_pCurNodeInfo->m_fSoftDelete == FALSE)
                            {   // normal isochronous CN
                                DllEvent.m_ulDllErrorEvents |= EPL_DLL_ERR_MN_CN_LOSS_PRES;
                                DllEvent.m_uiNodeId = EplDllkInstance_g.m_pCurNodeInfo->m_uiNodeId;
                            }
                            else
                            {   // CN shall be deleted softly, so remove it now, without issuing any error
                            tEplDllNodeOpParam  NodeOpParam;

                                NodeOpParam.m_OpNodeType = kEplDllNodeOpTypeIsochronous;
                                NodeOpParam.m_uiNodeId = EplDllkInstance_g.m_pCurNodeInfo->m_uiNodeId;

                                Event.m_EventSink = kEplEventSinkDllkCal;
                                Event.m_EventType = kEplEventTypeDllkDelNode;
                                // $$$ d.k. set Event.m_NetTime to current time
                                Event.m_uiSize = sizeof (NodeOpParam);
                                Event.m_pArg = &NodeOpParam;
                                Ret = EplEventkPost(&Event);
                                if (Ret != kEplSuccessful)
                                {
                                    goto Exit;
                                }
                            }

                            // continue with sending next PReq
                        }

                        case kEplDllMsWaitPreqTrig:
                        {
                            // send next PReq
                            Ret = EplDllkMnSendPreq(NmtState_p, &EplDllkInstance_g.m_DllState);
                            if (Ret != kEplSuccessful)
                            {
                                goto Exit;
                            }

                            break;
                        }

                        default:
                        {   // wrong DLL state
                            break;
                        }
                    }

                    break;
                }

                case kEplNmtEventDllCePres:
                {

                    switch (EplDllkInstance_g.m_DllState)
                    {
                        case kEplDllMsWaitPres:
                        {   // PRes received
                            // send next PReq
                            Ret = EplDllkMnSendPreq(NmtState_p, &EplDllkInstance_g.m_DllState);
                            if (Ret != kEplSuccessful)
                            {
                                goto Exit;
                            }

                            break;
                        }

                        default:
                        {   // wrong DLL state
                            break;
                        }
                    }

                    break;
                }

                case kEplNmtEventDllMeSoaTrig:
                {

                    switch (EplDllkInstance_g.m_DllState)
                    {
                        case kEplDllMsWaitSoaTrig:
                        {   // MN PRes sent
                            // send SoA
                            Ret = EplDllkMnSendSoa(NmtState_p, &EplDllkInstance_g.m_DllState, TRUE);
                            if (Ret != kEplSuccessful)
                            {
                                goto Exit;
                            }

                            break;
                        }

                        default:
                        {   // wrong DLL state
                            break;
                        }
                    }

                    break;
                }

                case kEplNmtEventDllCeAsnd:
                {   // ASnd has been received, but it may be not the requested one
                    if (EplDllkInstance_g.m_DllState == kEplDllMsWaitAsnd)
                    {
                        EplDllkInstance_g.m_DllState = kEplDllMsWaitSocTrig;
                    }
                    break;
                }
#else
                case kEplNmtEventDllMeAsndTimeout:
                {   // SoC has been sent, so ASnd should have been received
                    // report if SoA was correctly answered
                    Ret = EplDllkAsyncFrameNotReceived(EplDllkInstance_g.m_aLastReqServiceId[EplDllkInstance_g.m_bCurLastSoaReq],
                                                       EplDllkInstance_g.m_auiLastTargetNodeId[EplDllkInstance_g.m_bCurLastSoaReq]);
                    // switch SoAReq buffer
                    EplDllkInstance_g.m_bCurLastSoaReq++;
                    if (EplDllkInstance_g.m_bCurLastSoaReq >= EPL_DLLK_SOAREQ_COUNT)
                    {
                        EplDllkInstance_g.m_bCurLastSoaReq = 0;
                    }
                    break;
                }
#endif

                default:
                    break;
            }
            break;
        }
#endif // (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)

        default:
            break;
    }

    if (DllEvent.m_ulDllErrorEvents != 0)
    {   // error event set -> post it to error handler
        Ret = EplErrorHandlerkPostError(&DllEvent);
    }

#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)
Exit:
#endif
    return Ret;
}

//---------------------------------------------------------------------------
//
// Function:    EplDllkCbFrameReceived()
//
// Description: called from EdrvInterruptHandler()
//
// Parameters:  pRxBuffer_p             = receive buffer structure
//
// Returns:     (none)
//
//
// State:
//
//---------------------------------------------------------------------------

static tEdrvReleaseRxBuffer EplDllkCbFrameReceived(tEdrvRxBuffer * pRxBuffer_p)
{
tEdrvReleaseRxBuffer    ReleaseRxBuffer = kEdrvReleaseRxBufferImmediately;
tEplKernel              Ret             = kEplSuccessful;
tEplNmtState            NmtState;
tEplNmtEvent            NmtEvent        = kEplNmtEventNoEvent;
tEplEvent               Event;
tEplFrame*              pFrame;
tEplFrameInfo           FrameInfo;
tEplMsgType             MsgType;

TGT_DLLK_DECLARE_FLAGS

    TGT_DLLK_ENTER_CRITICAL_SECTION();

    BENCHMARK_MOD_02_SET(3);
    NmtState = EplDllkInstance_g.m_NmtState;

    if (NmtState <= kEplNmtGsResetConfiguration)
    {
        goto Exit;
    }

    pFrame = (tEplFrame *) pRxBuffer_p->m_pbBuffer;

#if EDRV_EARLY_RX_INT != FALSE
    switch (pRxBuffer_p->m_BufferInFrame)
    {
        case kEdrvBufferFirstInFrame:
        {
        tEdrvTxBuffer*  pTxBuffer = NULL;

            MsgType = (tEplMsgType)AmiGetByteFromLe(&pFrame->m_le_bMessageType);
            if (MsgType == kEplMsgTypePreq)
            {
            if (EplDllkInstance_g.m_DllState == kEplDllCsWaitPreq)
            {   // PReq expected and actually received
                // d.k.: The condition above is sufficent, because EPL cycle is active
                //       and no non-EPL frame shall be received in isochronous phase.
                // start transmission PRes
                // $$$ What if Tx buffer is invalid?
                pTxBuffer = &EplDllkInstance_g.m_pTxBuffer[EPL_DLLK_TXFRAME_PRES + EplDllkInstance_g.m_bCurTxBufferOffsetCycle];
#if (EPL_DLL_PRES_READY_AFTER_SOA != FALSE) || (EPL_DLL_PRES_READY_AFTER_SOC != FALSE)
                Ret = EdrvTxMsgStart(pTxBuffer);
#else
                pTxFrame = (tEplFrame *) pTxBuffer->m_pbBuffer;
                // update frame (NMT state, RD, RS, PR, MS, EN flags)
                AmiSetByteToLe(&pTxFrame->m_Data.m_Pres.m_le_bNmtStatus, (BYTE) NmtState);
                AmiSetByteToLe(&pTxFrame->m_Data.m_Pres.m_le_bFlag2, EplDllkInstance_g.m_bFlag2);
                if (NmtState != kEplNmtCsOperational)
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
        }

        case kEdrvBufferMiddleInFrame:
        {
            goto Exit;
        }

        case kEdrvBufferLastInFrame:
        {
            break;
        }
    }
#endif

    FrameInfo.m_pFrame = pFrame;
    FrameInfo.m_uiFrameSize = pRxBuffer_p->m_uiRxMsgLen;

    if (AmiGetWordFromBe(&pFrame->m_be_wEtherType) != EPL_C_DLL_ETHERTYPE_EPL)
    {   // non-EPL frame
        //TRACE2("EplDllkCbFrameReceived: pfnCbAsync=0x%p SrcMAC=0x%llx\n", EplDllkInstance_g.m_pfnCbAsync, AmiGetQword48FromBe(pFrame->m_be_abSrcMac));
        if (EplDllkInstance_g.m_pfnCbAsync != NULL)
        {   // handler for async frames is registered
            EplDllkInstance_g.m_pfnCbAsync(&FrameInfo);
        }

        goto Exit;
    }

    MsgType = (tEplMsgType)AmiGetByteFromLe(&pFrame->m_le_bMessageType);
    switch (MsgType)
    {
        case kEplMsgTypePreq:
        {
            // PReq frame
            if (AmiGetByteFromLe(&pFrame->m_le_bDstNodeId) != EplDllkInstance_g.m_DllConfigParam.m_uiNodeId)
            {   // this PReq is not intended for us
                goto Exit;
            }
            NmtEvent = kEplNmtEventDllCePreq;

            Ret = EplDllkProcessReceivedPreq(&FrameInfo, NmtState, &ReleaseRxBuffer);
            if (Ret != kEplSuccessful)
            {
                goto Exit;
            }

            break;
        }

        case kEplMsgTypePres:
        {

            Ret = EplDllkProcessReceivedPres(&FrameInfo, NmtState, &NmtEvent, &ReleaseRxBuffer);
            if (Ret != kEplSuccessful)
            {
                goto Exit;
            }

            break;
        }

        case kEplMsgTypeSoc:
        {
            // SoC frame
            NmtEvent = kEplNmtEventDllCeSoc;

            Ret = EplDllkProcessReceivedSoc(pRxBuffer_p, NmtState);
            if (Ret != kEplSuccessful)
            {
                goto Exit;
            }

            break;
        }

        case kEplMsgTypeSoa:
        {
            // SoA frame
            NmtEvent = kEplNmtEventDllCeSoa;

            Ret = EplDllkProcessReceivedSoa(pRxBuffer_p, NmtState);
            if (Ret != kEplSuccessful)
            {
                goto Exit;
            }

            break;
        }

        case kEplMsgTypeAsnd:
        {
            // ASnd frame
            NmtEvent = kEplNmtEventDllCeAsnd;

            Ret = EplDllkProcessReceivedAsnd(&FrameInfo, pRxBuffer_p, NmtState);
            if (Ret != kEplSuccessful)
            {
                goto Exit;
            }

            break;
        }

        default:
        {
            break;
        }
    }

    if (NmtEvent != kEplNmtEventNoEvent)
    {   // event for DLL and NMT state machine generated
        Ret = EplDllkChangeState(NmtEvent, NmtState);
        if (Ret != kEplSuccessful)
        {
            goto Exit;
        }

        if ((NmtEvent != kEplNmtEventDllCeAsnd)
            && ((NmtState <= kEplNmtCsPreOperational1) || (NmtEvent != kEplNmtEventDllCePres)))
        {   // NMT state machine is not interested in ASnd frames and PRes frames when not CsNotActive or CsPreOp1
            // inform NMT module
            Event.m_EventSink = kEplEventSinkNmtk;
            Event.m_EventType = kEplEventTypeNmtEvent;
            Event.m_uiSize = sizeof (NmtEvent);
            Event.m_pArg = &NmtEvent;
            Ret = EplEventkPost(&Event);
        }
    }

Exit:
    if (Ret != kEplSuccessful)
    {
    DWORD   dwArg;

        BENCHMARK_MOD_02_TOGGLE(7);

        dwArg = EplDllkInstance_g.m_DllState | (NmtEvent << 8);

        // Error event for API layer
        Ret = EplEventkPostError(kEplEventSourceDllk,
                        Ret,
                        sizeof(dwArg),
                        &dwArg);
    }
    BENCHMARK_MOD_02_RESET(3);

    TGT_DLLK_LEAVE_CRITICAL_SECTION();

    return ReleaseRxBuffer;
}


//---------------------------------------------------------------------------
//
// Function:    EplDllkProcessReceivedPreq()
//
// Description: called from EplDllkCbFrameReceived()
//
// Parameters:  pFrameInfo_p            = [IN] Pointer to frame info structure
//                                        of received frame
//              NmtState_p              = [IN] Current local NMT state
//              pReleaseRxBuffer_p      = [OUT] Return whether RxBuffer is
//                                        released immediately or later
//
// Returns:     tEplKernel
//
// State:
//
//---------------------------------------------------------------------------

static tEplKernel EplDllkProcessReceivedPreq(   tEplFrameInfo*          pFrameInfo_p,
                                                tEplNmtState            NmtState_p,
                                                tEdrvReleaseRxBuffer*   pReleaseRxBuffer_p)
{
tEplKernel      Ret = kEplSuccessful;
tEplFrame*      pFrame;
BYTE            bFlag1;

    pFrame = pFrameInfo_p->m_pFrame;

    if (NmtState_p >= kEplNmtMsNotActive)
    {   // MN is active -> wrong msg type
        goto Exit;
    }

#if EDRV_EARLY_RX_INT == FALSE
    if (NmtState_p >= kEplNmtCsPreOperational2)
    {   // respond to and process PReq frames only in PreOp2, ReadyToOp and Op

#if (EDRV_AUTO_RESPONSE == FALSE)
    tEdrvTxBuffer*  pTxBuffer = NULL;

        // Auto-response is disabled
        // Does PRes exist?
        pTxBuffer = &EplDllkInstance_g.m_pTxBuffer[EPL_DLLK_TXFRAME_PRES + EplDllkInstance_g.m_bCurTxBufferOffsetCycle];
        if (pTxBuffer->m_pbBuffer != NULL)
        {   // PRes does exist
            // send PRes frame
#if (EPL_DLL_PRES_READY_AFTER_SOA != FALSE) || (EPL_DLL_PRES_READY_AFTER_SOC != FALSE)
            EdrvTxMsgStart(pTxBuffer);
#else
            Ret = EdrvSendTxMsg(pTxBuffer);
            if (Ret != kEplSuccessful)
            {
                goto Exit;
            }
#endif
        }
#endif
#endif

        // save EA flag
        bFlag1 = AmiGetByteFromLe(&pFrame->m_Data.m_Preq.m_le_bFlag1);
        EplDllkInstance_g.m_bMnFlag1 =
            (EplDllkInstance_g.m_bMnFlag1 & ~EPL_FRAME_FLAG1_EA)
            | (bFlag1 & EPL_FRAME_FLAG1_EA);

        // inform PDO module
#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_PDOK)) != 0)
        if (NmtState_p >= kEplNmtCsReadyToOperate)
        {   // inform PDO module only in ReadyToOp and Op
            if (NmtState_p != kEplNmtCsOperational)
            {
                // reset RD flag and all other flags, but that does not matter, because they were processed above
                AmiSetByteToLe(&pFrame->m_Data.m_Preq.m_le_bFlag1, 0);
            }

            // compares real frame size and PDO size
            if (((unsigned int) (AmiGetWordFromLe(&pFrame->m_Data.m_Preq.m_le_wSize) + EPL_FRAME_OFFSET_PDO_PAYLOAD)
                 > pFrameInfo_p->m_uiFrameSize)
                 || (pFrameInfo_p->m_uiFrameSize > (EplDllkInstance_g.m_DllConfigParam.m_uiPreqActPayloadLimit + EPL_FRAME_OFFSET_PDO_PAYLOAD)))
            {   // format error
            tEplErrorHandlerkEvent  DllEvent;

                DllEvent.m_ulDllErrorEvents = EPL_DLL_ERR_INVALID_FORMAT;
                DllEvent.m_uiNodeId = AmiGetByteFromLe(&pFrame->m_le_bSrcNodeId);
                DllEvent.m_NmtState = NmtState_p;
                Ret = EplErrorHandlerkPostError(&DllEvent);
                if (Ret != kEplSuccessful)
                {
                    goto Exit;
                }
                goto Exit;
            }

            // forward PReq frame as RPDO to PDO module
            Ret = EplDllkForwardRpdo(pFrameInfo_p);
            if (Ret == kEplReject)
            {
                *pReleaseRxBuffer_p = kEdrvReleaseRxBufferLater;
                Ret = kEplSuccessful;
            }
            else if (Ret != kEplSuccessful)
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
    EplDllkInstance_g.m_uiCycleCount = 0;

Exit:
    return Ret;
}


//---------------------------------------------------------------------------
//
// Function:    EplDllkProcessReceivedPres()
//
// Description: called from EplDllkCbFrameReceived()
//
// Parameters:  pFrameInfo_p            = [IN] Pointer to frame info structure
//                                        of received frame
//              NmtState_p              = [IN] Current local NMT state
//              pNmtEvent_p             = [OUT] Pointer to NMT event
//              pReleaseRxBuffer_p      = [OUT] Return whether RxBuffer is
//                                        released immediately or later
//
// Returns:     tEplKernel
//
// State:
//
//---------------------------------------------------------------------------

static tEplKernel EplDllkProcessReceivedPres(   tEplFrameInfo*          pFrameInfo_p,
                                                tEplNmtState            NmtState_p,
                                                tEplNmtEvent*           pNmtEvent_p,
                                                tEdrvReleaseRxBuffer*   pReleaseRxBuffer_p)
{
tEplKernel      Ret = kEplSuccessful;
tEplFrame*      pFrame;
unsigned int    uiNodeId;

#if EPL_NMT_MAX_NODE_ID > 0
tEplDllkNodeInfo*   pIntNodeInfo = NULL;
#endif

    pFrame = pFrameInfo_p->m_pFrame;

    // PRes frame
    uiNodeId = AmiGetByteFromLe(&pFrame->m_le_bSrcNodeId);

#if EPL_DLL_PRES_CHAINING_CN != FALSE
    if ((EplDllkInstance_g.m_fPrcEnabled != FALSE) &&
        (uiNodeId == EPL_C_ADR_MN_DEF_NODE_ID))
    {   // handle PResMN as PReq for PRes Chaining
        *pNmtEvent_p = kEplNmtEventDllCePreq;
    }
    else
#endif
    {
        *pNmtEvent_p = kEplNmtEventDllCePres;
    }

    if ((NmtState_p >= kEplNmtCsPreOperational2)
        && (NmtState_p <= kEplNmtCsOperational))
    {   // process PRes frames only in PreOp2, ReadyToOp and Op of CN

#if EPL_NMT_MAX_NODE_ID > 0
        pIntNodeInfo = EplDllkGetNodeInfo(uiNodeId);
        if (pIntNodeInfo == NULL)
        {   // no node info structure available
            Ret = kEplDllNoNodeInfo;
            goto Exit;
        }
#endif
    }
#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)
    else
#if EPL_DLL_DISABLE_EDRV_CYCLIC != FALSE
        if (EplDllkInstance_g.m_DllState == kEplDllMsWaitPres)
#else
        if (EplDllkInstance_g.m_DllState > kEplDllMsNonCyclic)
#endif
    {   // or process PRes frames in MsWaitPres
    tEplHeartbeatEvent  HeartbeatEvent;
    BYTE                bFlag1;

#if EPL_DLL_DISABLE_EDRV_CYCLIC != FALSE
        pIntNodeInfo = EplDllkInstance_g.m_pCurNodeInfo;

        if ((pIntNodeInfo == NULL) || (pIntNodeInfo->m_uiNodeId != uiNodeId))
        {   // ignore PRes, because it is from wrong CN
            // $$$ maybe post event to NmtMn module
            *pNmtEvent_p = kEplNmtEventNoEvent;
            goto Exit;
        }
#else
    BYTE    bNextNodeIndex = EplDllkInstance_g.m_bCurNodeIndex;
    BYTE*   pbCnNodeId = &EplDllkInstance_g.m_aabCnNodeIdList[EplDllkInstance_g.m_bCurTxBufferOffsetCycle][bNextNodeIndex];
#if EPL_DLL_PRES_CHAINING_MN != FALSE
    BOOL    fPrcSlotFinished = FALSE;
#endif

        while (*pbCnNodeId != EPL_C_ADR_INVALID)
        {
            if (*pbCnNodeId == uiNodeId)
            {   // CN found in list
                bNextNodeIndex = bNextNodeIndex - EplDllkInstance_g.m_bCurNodeIndex;
                EplDllkInstance_g.m_bCurNodeIndex += bNextNodeIndex + 1;

                for (pbCnNodeId-- ; bNextNodeIndex > 0; bNextNodeIndex--, pbCnNodeId--)
                {   // issue error for each CN in list between last and current
                    Ret = EplDllkIssueLossOfPres(*pbCnNodeId);
                    if (Ret != kEplSuccessful)
                    {
                        goto Exit;
                    }
                }

                pIntNodeInfo = EplDllkGetNodeInfo(uiNodeId);

                break;
            }
#if EPL_DLL_PRES_CHAINING_MN != FALSE
            else if (*pbCnNodeId == EPL_C_ADR_BROADCAST)
            {   // PRC slot finished
                fPrcSlotFinished = TRUE;
            }
#endif
            pbCnNodeId++;
            bNextNodeIndex++;
        }
        if (pIntNodeInfo == NULL)
        {   // ignore PRes, because it is from wrong CN
            *pNmtEvent_p = kEplNmtEventNoEvent;
            goto Exit;
        }

#if EPL_DLL_PRES_CHAINING_MN != FALSE
        if (fPrcSlotFinished != FALSE)
        {
            EplDllkInstance_g.m_fPrcSlotFinished = TRUE;

            if ((EplDllkInstance_g.m_DllConfigParam.m_uiSyncNodeId > EPL_C_ADR_SYNC_ON_SOC)
                && (EplDllkInstance_g.m_fSyncProcessed == FALSE)
                && (EplDllkInstance_g.m_DllConfigParam.m_fSyncOnPrcNode != FALSE))
            {
                EplDllkInstance_g.m_fSyncProcessed = TRUE;
                Ret = EplDllkPostEvent(kEplEventTypeSync);
                if (Ret != kEplSuccessful)
                {
                    goto Exit;
                }
            }
        }
        else
#endif
#endif

        if ((EplDllkInstance_g.m_DllConfigParam.m_uiSyncNodeId > EPL_C_ADR_SYNC_ON_SOC)
            && (EplDllkInstance_g.m_fSyncProcessed == FALSE)
#if EPL_DLL_PRES_CHAINING_MN != FALSE
            && (EplDllkInstance_g.m_DllConfigParam.m_fSyncOnPrcNode != EplDllkInstance_g.m_fPrcSlotFinished)
#endif
            && (uiNodeId > EplDllkInstance_g.m_DllConfigParam.m_uiSyncNodeId))
        {
            EplDllkInstance_g.m_fSyncProcessed = TRUE;
            Ret = EplDllkPostEvent(kEplEventTypeSync);
            if (Ret != kEplSuccessful)
            {
                goto Exit;
            }
        }

        // forward Flag2 to asynchronous scheduler
        bFlag1 = AmiGetByteFromLe(&pFrame->m_Data.m_Asnd.m_Payload.m_StatusResponse.m_le_bFlag2);
        Ret = EplDllkCalAsyncSetPendingRequests(uiNodeId,
            ((tEplDllAsyncReqPriority) ((bFlag1 & EPL_FRAME_FLAG2_PR) >> EPL_FRAME_FLAG2_PR_SHIFT)),
            (bFlag1 & EPL_FRAME_FLAG2_RS));
        if (Ret != kEplSuccessful)
        {
            goto Exit;
        }

        // check NMT state of CN
        HeartbeatEvent.m_wErrorCode = EPL_E_NO_ERROR;
        HeartbeatEvent.m_NmtState =
            (tEplNmtState) (AmiGetByteFromLe(&pFrame->m_Data.m_Pres.m_le_bNmtStatus) | EPL_NMT_TYPE_CS);

        if (pIntNodeInfo->m_NmtState != HeartbeatEvent.m_NmtState)
        {   // NMT state of CN has changed -> post event to NmtMnu module
        tEplEvent   Event;

            if (pIntNodeInfo->m_fSoftDelete == FALSE)
            {   // normal isochronous CN
                HeartbeatEvent.m_uiNodeId = uiNodeId;
                Event.m_EventSink = kEplEventSinkNmtMnu;
                Event.m_EventType = kEplEventTypeHeartbeat;
                Event.m_uiSize = sizeof (HeartbeatEvent);
                Event.m_pArg = &HeartbeatEvent;
            }
            else
            {   // CN shall be deleted softly, so remove it now, without issuing any error
            tEplDllNodeOpParam  NodeOpParam;

                NodeOpParam.m_OpNodeType = kEplDllNodeOpTypeIsochronous;
                NodeOpParam.m_uiNodeId = pIntNodeInfo->m_uiNodeId;

                Event.m_EventSink = kEplEventSinkDllkCal;
                Event.m_EventType = kEplEventTypeDllkDelNode;
                // $$$ d.k. set Event.m_NetTime to current time
                Event.m_uiSize = sizeof (NodeOpParam);
                Event.m_pArg = &NodeOpParam;
            }
            Ret = EplEventkPost(&Event);
            if (Ret != kEplSuccessful)
            {
                goto Exit;
            }

            // save current NMT state of CN in internal node structure
            pIntNodeInfo->m_NmtState = HeartbeatEvent.m_NmtState;
        }
    }
#endif
    else
    {   // ignore PRes, because it was received in wrong NMT state
        // but execute EplDllkChangeState() and post event to NMT module
        goto Exit;
    }

    // inform PDO module
#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_PDOK)) != 0)
    if ((NmtState_p != kEplNmtCsPreOperational2)
        && (NmtState_p != kEplNmtMsPreOperational2))
    {   // inform PDO module only in ReadyToOp and Op
        // compare real frame size and PDO size?
        if (((unsigned int) (AmiGetWordFromLe(&pFrame->m_Data.m_Pres.m_le_wSize) + EPL_FRAME_OFFSET_PDO_PAYLOAD)
            > pFrameInfo_p->m_uiFrameSize)
#if EPL_NMT_MAX_NODE_ID > 0
            || (pFrameInfo_p->m_uiFrameSize > (unsigned int) (pIntNodeInfo->m_wPresPayloadLimit + EPL_FRAME_OFFSET_PDO_PAYLOAD))
#endif
            )
        {   // format error
        tEplErrorHandlerkEvent  DllEvent;

#if EPL_NMT_MAX_NODE_ID > 0
            if (pIntNodeInfo->m_wPresPayloadLimit > 0)
#endif
            {   // This PRes frame was expected, but it is too large
                // otherwise it will be silently ignored
                DllEvent.m_ulDllErrorEvents = EPL_DLL_ERR_INVALID_FORMAT;
                DllEvent.m_uiNodeId = uiNodeId;
                DllEvent.m_NmtState = NmtState_p;
                Ret = EplErrorHandlerkPostError(&DllEvent);
                if (Ret != kEplSuccessful)
                {
                    goto Exit;
                }
            }
            goto Exit;
        }
        if ((NmtState_p != kEplNmtCsOperational)
            && (NmtState_p != kEplNmtMsOperational))
        {
            // reset RD flag and all other flags, but that does not matter, because they were processed above
            AmiSetByteToLe(&pFrame->m_Data.m_Pres.m_le_bFlag1, 0);
        }
        Ret = EplDllkForwardRpdo(pFrameInfo_p);
        if (Ret == kEplReject)
        {
            *pReleaseRxBuffer_p = kEdrvReleaseRxBufferLater;
            Ret = kEplSuccessful;
        }
        else if (Ret != kEplSuccessful)
        {
            goto Exit;
        }
    }
#endif

#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)
    if ((EplDllkInstance_g.m_DllState > kEplDllMsNonCyclic)
        && (EplDllkInstance_g.m_DllConfigParam.m_uiSyncNodeId > EPL_C_ADR_SYNC_ON_SOC)
        && (EplDllkInstance_g.m_fSyncProcessed == FALSE))
    {   // check if Sync event needs to be triggered
        if (
#if EPL_DLL_PRES_CHAINING_MN != FALSE
            (EplDllkInstance_g.m_DllConfigParam.m_fSyncOnPrcNode != EplDllkInstance_g.m_fPrcSlotFinished)
            &&
#endif
            (uiNodeId == EplDllkInstance_g.m_DllConfigParam.m_uiSyncNodeId))
        {
            EplDllkInstance_g.m_fSyncProcessed = TRUE;
            Ret = EplDllkPostEvent(kEplEventTypeSync);
        }
    }
#endif

Exit:
    return Ret;
}


//---------------------------------------------------------------------------
//
// Function:    EplDllkIssueLossOfPres()
//
// Description: forwards this event to ErrorHandlerk module.
//
// Parameters:  uiNodeId_p      = node-ID of CN where no PRes frame was received
//
// Returns:     tEplKernel
//
// State:
//
//---------------------------------------------------------------------------

#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)
static tEplKernel EplDllkIssueLossOfPres(unsigned int uiNodeId_p)
{
tEplKernel          Ret = kEplSuccessful;
tEplDllkNodeInfo*   pIntNodeInfo;
tEplEvent           Event;

    pIntNodeInfo = EplDllkGetNodeInfo(uiNodeId_p);
    if (pIntNodeInfo != NULL)
    {
        if (pIntNodeInfo->m_fSoftDelete == FALSE)
        {   // normal isochronous CN
        tEplErrorHandlerkEvent  DllEvent;

            DllEvent.m_ulDllErrorEvents = EPL_DLL_ERR_MN_CN_LOSS_PRES;
            DllEvent.m_uiNodeId = pIntNodeInfo->m_uiNodeId;
            Ret = EplErrorHandlerkPostError(&DllEvent);
            if (Ret != kEplSuccessful)
            {
                goto Exit;
            }
        }
        else
        {   // CN shall be deleted softly, so remove it now, without issuing any error
        tEplDllNodeOpParam  NodeOpParam;

            NodeOpParam.m_OpNodeType = kEplDllNodeOpTypeIsochronous;
            NodeOpParam.m_uiNodeId = pIntNodeInfo->m_uiNodeId;

            Event.m_EventSink = kEplEventSinkDllkCal;
            Event.m_EventType = kEplEventTypeDllkDelNode;
            // $$$ d.k. set Event.m_NetTime to current time
            Event.m_uiSize = sizeof (NodeOpParam);
            Event.m_pArg = &NodeOpParam;
            Ret = EplEventkPost(&Event);
            if (Ret != kEplSuccessful)
            {
                goto Exit;
            }
        }
    }

Exit:
    return Ret;
}
#endif


//---------------------------------------------------------------------------
//
// Function:    EplDllkProcessReceivedSoc()
//
// Description: called from EplDllkCbFrameReceived()
//
// Parameters:  pRxBuffer_p             = Edrv receive buffer structure
//              NmtState_p              = current local NMT state
//
// Returns:     tEplKernel
//
// State:
//
//---------------------------------------------------------------------------

static tEplKernel EplDllkProcessReceivedSoc(tEdrvRxBuffer* pRxBuffer_p, tEplNmtState NmtState_p)
{
tEplKernel      Ret = kEplSuccessful;
#if EPL_DLL_PRES_READY_AFTER_SOC != FALSE
tEdrvTxBuffer*  pTxBuffer = NULL;
#endif

#if (EPL_DLL_PROCESS_SYNC != EPL_DLL_PROCESS_SYNC_ON_TIMER)
    UNUSED_PARAMETER(pRxBuffer_p);
#endif

    if (NmtState_p >= kEplNmtMsNotActive)
    {   // MN is active -> wrong msg type
        goto Exit;
    }

#if EPL_DLL_PRES_READY_AFTER_SOC != FALSE
    // post PRes to transmit FIFO of the ethernet controller, but don't start
    // transmission over bus
    pTxBuffer = &EplDllkInstance_g.m_pTxBuffer[EPL_DLLK_TXFRAME_PRES + EplDllkInstance_g.m_bCurTxBufferOffsetCycle];
    // Does PRes exist?
    if (pTxBuffer->m_pbBuffer != NULL)
    {   // PRes does exist
        // mark PRes frame as ready for transmission
        Ret = EdrvTxMsgReady(pTxBuffer);
        if (Ret != kEplSuccessful)
        {
            goto Exit;
        }
    }
#endif

    if (NmtState_p >= kEplNmtCsPreOperational2)
    {   // SoC frames only in PreOp2, ReadyToOp and Op

#if (EPL_DLL_PROCESS_SYNC == EPL_DLL_PROCESS_SYNC_ON_SOC)
        // trigger synchronous task
        Ret = EplDllkPostEvent(kEplEventTypeSync);
        if (Ret != kEplSuccessful)
        {
            goto Exit;
        }
#elif (EPL_DLL_PROCESS_SYNC == EPL_DLL_PROCESS_SYNC_ON_TIMER)
        Ret = EplTimerSynckTriggerAtTimeStamp(pRxBuffer_p->m_pTgtTimeStamp);
        if (Ret != kEplSuccessful)
        {
            goto Exit;
        }
#endif

        // update cycle counter
        if (EplDllkInstance_g.m_DllConfigParam.m_uiMultiplCycleCnt > 0)
        {   // multiplexed cycle active
            EplDllkInstance_g.m_uiCycleCount = (EplDllkInstance_g.m_uiCycleCount + 1) % EplDllkInstance_g.m_DllConfigParam.m_uiMultiplCycleCnt;
        }
    }

    // reprogram timer
#if EPL_TIMER_USE_HIGHRES != FALSE
    if (EplDllkInstance_g.m_ullFrameTimeout != 0)
    {
        Ret = EplTimerHighReskModifyTimerNs(&EplDllkInstance_g.m_TimerHdlCycle, EplDllkInstance_g.m_ullFrameTimeout, EplDllkCbCnTimer, 0L, FALSE);
        if (Ret != kEplSuccessful)
        {
            goto Exit;
        }
    }
#endif

Exit:
    return Ret;
}


//---------------------------------------------------------------------------
//
// Function:    EplDllkProcessReceivedSoa()
//
// Description: called from EplDllkCbFrameReceived()
//
// Parameters:  pRxBuffer_p             = Edrv receive buffer structure
//              NmtState_p              = current local NMT state
//
// Returns:     tEplKernel
//
// State:
//
//---------------------------------------------------------------------------

static tEplKernel EplDllkProcessReceivedSoa(tEdrvRxBuffer* pRxBuffer_p, tEplNmtState NmtState_p)
{
tEplKernel      Ret = kEplSuccessful;
tEplFrame*      pFrame;
tEdrvTxBuffer*  pTxBuffer = NULL;
tEplDllReqServiceId     ReqServiceId;
unsigned int    uiNodeId;
BYTE            bFlag1;

    pFrame = (tEplFrame *) pRxBuffer_p->m_pbBuffer;

    if (NmtState_p >= kEplNmtMsNotActive)
    {   // MN is active -> wrong msg type
        goto Exit;
    }

    pTxBuffer = NULL;

    if ((NmtState_p & EPL_NMT_SUPERSTATE_MASK) != EPL_NMT_CS_EPLMODE)
    {   // do not respond, if NMT state is < PreOp1 (i.e. not EPL_MODE)
        goto Exit;
    }

    // check TargetNodeId
    uiNodeId = AmiGetByteFromLe(&pFrame->m_Data.m_Soa.m_le_bReqServiceTarget);
    if (uiNodeId == EplDllkInstance_g.m_DllConfigParam.m_uiNodeId)
    {   // local node is the target of the current request

        // check ServiceId
        ReqServiceId = (tEplDllReqServiceId) AmiGetByteFromLe(&pFrame->m_Data.m_Soa.m_le_bReqServiceId);
        switch (ReqServiceId)
        {
            case kEplDllReqServiceStatus:
            {   // StatusRequest
#if (EDRV_AUTO_RESPONSE == FALSE)
                // Auto-response is not available
                pTxBuffer = &EplDllkInstance_g.m_pTxBuffer[EPL_DLLK_TXFRAME_STATUSRES + EplDllkInstance_g.m_bCurTxBufferOffsetStatusRes];
                if (pTxBuffer->m_pbBuffer != NULL)
                {   // StatusRes does exist

                    // send StatusRes
                    Ret = EdrvSendTxMsg(pTxBuffer);
                    if (Ret != kEplSuccessful)
                    {
                        goto Exit;
                    }
                    TGT_DBG_SIGNAL_TRACE_POINT(8);
                }
                else
                {   // no frame transmitted
                    pTxBuffer = NULL;
                }
#endif

                // update error signaling
                bFlag1 = AmiGetByteFromLe(&pFrame->m_Data.m_Soa.m_le_bFlag1);
                if (((bFlag1 ^ EplDllkInstance_g.m_bMnFlag1) & EPL_FRAME_FLAG1_ER) != 0)
                {   // exception reset flag was changed by MN
                    // assume same state for EC in next cycle (clear all other bits)
                    if ((bFlag1 & EPL_FRAME_FLAG1_ER) != 0)
                    {
                        // set EC and reset rest
                        EplDllkInstance_g.m_bFlag1 = EPL_FRAME_FLAG1_EC;
                    }
                    else
                    {
                        // reset entire flag 1 (including EC and EN)
                        EplDllkInstance_g.m_bFlag1 = 0;
                    }

                    // signal update of StatusRes
                    Ret = EplDllkPostEvent(kEplEventTypeDllkFlag1);
                    if (Ret != kEplSuccessful)
                    {
                        goto Exit;
                    }
                }
                // save flag 1 from MN for Status request response cycle
                // $$$ d.k. only in PreOp1 and when async-only or not accessed isochronously
                EplDllkInstance_g.m_bMnFlag1 = bFlag1;
                goto Exit;
            }

            case kEplDllReqServiceIdent:
            {   // IdentRequest
#if (EDRV_AUTO_RESPONSE == FALSE)
                // Auto-response is not available
                pTxBuffer = &EplDllkInstance_g.m_pTxBuffer[EPL_DLLK_TXFRAME_IDENTRES + EplDllkInstance_g.m_bCurTxBufferOffsetIdentRes];
                if (pTxBuffer->m_pbBuffer != NULL)
                {   // IdentRes does exist
                    // send IdentRes
                    Ret = EdrvSendTxMsg(pTxBuffer);
                    if (Ret != kEplSuccessful)
                    {
                        goto Exit;
                    }
                    TGT_DBG_SIGNAL_TRACE_POINT(7);
                }
                else
                {   // no frame transmitted
                    pTxBuffer = NULL;
                }
#endif
                goto Exit;
            }

            case kEplDllReqServiceNmtRequest:
            {   // NmtRequest
#if (EDRV_AUTO_RESPONSE == FALSE)
                // Auto-response is not available
                pTxBuffer = &EplDllkInstance_g.m_pTxBuffer[EPL_DLLK_TXFRAME_NMTREQ + EplDllkInstance_g.m_bCurTxBufferOffsetNmtReq];
                if (pTxBuffer->m_pbBuffer != NULL)
                {   // NmtRequest does exist
                    // check if frame is not empty and not being filled
                    if (pTxBuffer->m_uiTxMsgLen > EPL_DLLK_BUFLEN_FILLING)
                    {
                        // send NmtRequest
                        Ret = EdrvSendTxMsg(pTxBuffer);
                        if (Ret != kEplSuccessful)
                        {
                            goto Exit;
                        }

                        // decrement RS in Flag 2
                        // The real update will be done later on event FillTx,
                        // but for now it assures that a quite good value gets via the SoA event into the next PRes.
                        if ((EplDllkInstance_g.m_bFlag2 & EPL_FRAME_FLAG2_RS) != 0)
                        {
                            EplDllkInstance_g.m_bFlag2--;
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
            }

#if (EPL_DLL_PRES_CHAINING_CN != FALSE) || (EPL_DLL_PRES_CHAINING_MN != FALSE)
            case kEplDllReqServiceSync:
            {   // SyncRequest
#if EPL_DLL_PRES_CHAINING_CN != FALSE
            DWORD       dwSyncControl;
            tEplFrame*  pTxFrameSyncRes;
            tEplDllkPrcCycleTiming  PrcCycleTiming;

                pTxFrameSyncRes = (tEplFrame *) EplDllkInstance_g.m_pTxBuffer[EPL_DLLK_TXFRAME_SYNCRES].m_pbBuffer;

                dwSyncControl = AmiGetDwordFromLe(&pFrame->m_Data.m_Soa.m_Payload.m_SyncRequest.m_le_dwSyncControl);

                if (dwSyncControl & EPL_SYNC_DEST_MAC_ADDRESS_VALID)
                {
                    if (EPL_MEMCMP(&pFrame->m_Data.m_Soa.m_Payload.m_SyncRequest.m_be_abDestMacAddress,
                                   &EplDllkInstance_g.m_be_abLocalMac, 6) != 0)
                    {   // DestMacAddress valid but unequal to own MAC address
                        // SyncReq is ignored
                        goto Exit;
                    }
                }

                PrcCycleTiming.m_dwPResTimeFirstNs = AmiGetDwordFromLe(&pFrame->m_Data.m_Soa.m_Payload.m_SyncRequest.m_le_dwPResTimeFirst);

                if ((dwSyncControl & EPL_SYNC_PRES_TIME_FIRST_VALID) &&
                    (EplDllkInstance_g.m_dwPrcPResTimeFirst != PrcCycleTiming.m_dwPResTimeFirstNs))
                {
                    EplDllkInstance_g.m_dwPrcPResTimeFirst = PrcCycleTiming.m_dwPResTimeFirstNs;

                    EplDllkInstance_g.m_pTxBuffer[EPL_DLLK_TXFRAME_PRES].m_dwTimeOffsetNs = PrcCycleTiming.m_dwPResTimeFirstNs;
                    Ret = EdrvChangeFilter(EplDllkInstance_g.m_aFilter, EPL_DLLK_FILTER_COUNT,
                                           EPL_DLLK_FILTER_PREQ,
                                           EDRV_FILTER_CHANGE_AUTO_RESPONSE_DELAY);
                    if (Ret != kEplSuccessful)
                    {
                        goto Exit;
                    }

                    AmiSetDwordToLe(&pTxFrameSyncRes->m_Data.m_Asnd.m_Payload.m_SyncResponse.m_le_dwPResTimeFirst,
                                    PrcCycleTiming.m_dwPResTimeFirstNs);
                    AmiSetDwordToLe(&pTxFrameSyncRes->m_Data.m_Asnd.m_Payload.m_SyncResponse.m_le_dwSyncStatus,
                                    AmiGetDwordFromLe(&pTxFrameSyncRes->m_Data.m_Asnd.m_Payload.m_SyncResponse.m_le_dwSyncStatus)
                                    | EPL_SYNC_PRES_TIME_FIRST_VALID);
                    // update SyncRes Tx buffer in Edrv
                    Ret = EdrvUpdateTxMsgBuffer(&EplDllkInstance_g.m_pTxBuffer[EPL_DLLK_TXFRAME_SYNCRES]);
                    if (Ret != kEplSuccessful)
                    {
                        goto Exit;
                    }
                }

                if (dwSyncControl & EPL_SYNC_PRES_FALL_BACK_TIMEOUT_VALID)
                {
                    EplDllkInstance_g.m_dwPrcPResFallBackTimeout = AmiGetDwordFromLe(&pFrame->m_Data.m_Soa.m_Payload.m_SyncRequest.m_le_dwPResFallBackTimeout);

#if (EPL_DLL_PROCESS_SYNC == EPL_DLL_PROCESS_SYNC_ON_TIMER)
                    if (EplDllkInstance_g.m_fPrcEnabled != FALSE)
                    {
                        Ret = EplTimerSynckSetLossOfSyncTolerance2Ns(EplDllkInstance_g.m_dwPrcPResFallBackTimeout);
                    }
#endif
                }

                if (dwSyncControl & EPL_SYNC_PRES_MODE_RESET)
                {
                    // PResModeReset overrules PResModeSet
                    dwSyncControl &= ~EPL_SYNC_PRES_MODE_SET;

                    Ret = EplDllkPResChainingDisable();
                    if (Ret != kEplSuccessful)
                    {
                        goto Exit;
                    }
                }
                else if (dwSyncControl & EPL_SYNC_PRES_MODE_SET)
                {   // PRes Chaining is Enabled
                    Ret = EplDllkPResChainingEnable();
                    if (Ret != kEplSuccessful)
                    {
                        goto Exit;
                    }
                }

                PrcCycleTiming.m_dwSyncControl = dwSyncControl & (EPL_SYNC_PRES_TIME_FIRST_VALID
                                                                  | EPL_SYNC_PRES_TIME_SECOND_VALID
                                                                  | EPL_SYNC_SYNC_MN_DELAY_FIRST_VALID
                                                                  | EPL_SYNC_SYNC_MN_DELAY_SECOND_VALID
                                                                  | EPL_SYNC_PRES_MODE_RESET
                                                                  | EPL_SYNC_PRES_MODE_SET);

                if (PrcCycleTiming.m_dwSyncControl != 0)
                {
                    PrcCycleTiming.m_dwPResTimeSecondNs = AmiGetDwordFromLe(&pFrame->m_Data.m_Soa.m_Payload.m_SyncRequest.m_le_dwPResTimeSecond);
                    PrcCycleTiming.m_dwSyncMNDelayFirstNs = AmiGetDwordFromLe(&pFrame->m_Data.m_Soa.m_Payload.m_SyncRequest.m_le_dwSyncMnDelayFirst);
                    PrcCycleTiming.m_dwSyncMNDelaySecondNs = AmiGetDwordFromLe(&pFrame->m_Data.m_Soa.m_Payload.m_SyncRequest.m_le_dwSyncMnDelaySecond);

                    // $$$ m.u.: CbUpdatePrcCycleTiming
                }
#endif
                goto Exit;
            }
#endif

            case kEplDllReqServiceUnspecified:
            {   // unspecified invite
#if (EDRV_AUTO_RESPONSE == FALSE)
                // Auto-response is not available
                pTxBuffer = &EplDllkInstance_g.m_pTxBuffer[EPL_DLLK_TXFRAME_NONEPL + EplDllkInstance_g.m_bCurTxBufferOffsetNonEpl];
                if (pTxBuffer->m_pbBuffer != NULL)
                {   // non-EPL frame does exist
                    // check if frame is not empty and not being filled
                    if (pTxBuffer->m_uiTxMsgLen > EPL_DLLK_BUFLEN_FILLING)
                    {
                        // send non-EPL frame
                        Ret = EdrvSendTxMsg(pTxBuffer);
                        if (Ret != kEplSuccessful)
                        {
                            goto Exit;
                        }

                        // decrement RS in Flag 2
                        // The real update will be done later on event FillTx,
                        // but for now it assures that a quite good value gets via the SoA event into the next PRes.
                        if ((EplDllkInstance_g.m_bFlag2 & EPL_FRAME_FLAG2_RS) != 0)
                        {
                            EplDllkInstance_g.m_bFlag2--;
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
            }

            case kEplDllReqServiceNo:
            {   // no async service requested -> do nothing
                goto Exit;
            }
        }
    }
#if EPL_DLL_PRES_CHAINING_CN != FALSE
    else
    {   // other node is the target of the current request
        // check ServiceId
        ReqServiceId = (tEplDllReqServiceId) AmiGetByteFromLe(&pFrame->m_Data.m_Soa.m_le_bReqServiceId);
        if (ReqServiceId == kEplDllReqServiceSync)
        {   // SyncRequest
            // store node ID and TimeStamp
            EplDllkInstance_g.m_uiSyncReqPrevNodeId = uiNodeId;
            EplTgtTimeStampCopy(EplDllkInstance_g.m_pSyncReqPrevTimeStamp,
                                pRxBuffer_p->m_pTgtTimeStamp);
        }
    }
#endif

#if EPL_DLL_PRES_READY_AFTER_SOA != FALSE
    if (pTxBuffer == NULL)
    {   // signal process function readiness of PRes frame
        Ret = EplDllkPostEvent(kEplEventTypeDllkPresReady);
        if (Ret != kEplSuccessful)
        {
            goto Exit;
        }
    }
#endif

    // $$$ put SrcNodeId, NMT state and NetTime as HeartbeatEvent into eventqueue

    // $$$ inform emergency protocol handling about flags

Exit:
    return Ret;
}


//---------------------------------------------------------------------------
//
// Function:    EplDllkProcessReceivedAsnd()
//
// Description: called from EplDllkCbFrameReceived()
//
// Parameters:  pFrameInfo_p            = pointer to frame info structure of received frame
//              NmtState_p              = current local NMT state
//
// Returns:     tEplKernel
//
// State:
//
//---------------------------------------------------------------------------

static tEplKernel EplDllkProcessReceivedAsnd(tEplFrameInfo* pFrameInfo_p,
                                             tEdrvRxBuffer* pRxBuffer_p,
                                             tEplNmtState   NmtState_p)
{
tEplKernel      Ret = kEplSuccessful;
tEplFrame*      pFrame;
unsigned int    uiAsndServiceId;
unsigned int    uiNodeId;

    UNUSED_PARAMETER(NmtState_p);
#if EPL_DLL_PRES_CHAINING_CN == FALSE
    UNUSED_PARAMETER(pRxBuffer_p);
#endif

    pFrame = pFrameInfo_p->m_pFrame;

    // ASnd service registered?
    uiAsndServiceId = (unsigned int) AmiGetByteFromLe(&pFrame->m_Data.m_Asnd.m_le_bServiceId);

#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)
    if (EplDllkInstance_g.m_DllState >= kEplDllMsNonCyclic)
    {
        switch ((tEplDllAsndServiceId) uiAsndServiceId)
        {
            case kEplDllAsndStatusResponse:
            case kEplDllAsndIdentResponse:
#if (EPL_DLL_PRES_CHAINING_MN != FALSE)
            case kEplDllAsndSyncResponse:
#endif
            {
            BYTE    bFlag1;

                uiNodeId = AmiGetByteFromLe(&pFrame->m_le_bSrcNodeId);
                if ((EplDllkInstance_g.m_aLastReqServiceId[EplDllkInstance_g.m_bCurLastSoaReq] == ((tEplDllReqServiceId) uiAsndServiceId))
                    && (uiNodeId == EplDllkInstance_g.m_auiLastTargetNodeId[EplDllkInstance_g.m_bCurLastSoaReq]))
                {   // mark request as responded
                    EplDllkInstance_g.m_aLastReqServiceId[EplDllkInstance_g.m_bCurLastSoaReq] = kEplDllReqServiceNo;
                }
                if (((tEplDllAsndServiceId) uiAsndServiceId) == kEplDllAsndIdentResponse)
                {   // memorize MAC address of CN for PReq
                tEplDllkNodeInfo*   pIntNodeInfo;

                    pIntNodeInfo = EplDllkGetNodeInfo(uiNodeId);
                    if (pIntNodeInfo == NULL)
                    {   // no node info structure available
                        Ret = kEplDllNoNodeInfo;
                        goto Exit;
                    }
                    else
                    {
                        EPL_MEMCPY(pIntNodeInfo->m_be_abMacAddr, pFrame->m_be_abSrcMac, 6);
                    }
                }
#if (EPL_DLL_PRES_CHAINING_MN != FALSE)
                else if (((tEplDllAsndServiceId) uiAsndServiceId) == kEplDllAsndSyncResponse)
                {
                    break;
                }
#endif

                // forward Flag2 to asynchronous scheduler
                bFlag1 = AmiGetByteFromLe(&pFrame->m_Data.m_Asnd.m_Payload.m_StatusResponse.m_le_bFlag2);
                Ret = EplDllkCalAsyncSetPendingRequests(uiNodeId,
                    ((tEplDllAsyncReqPriority) ((bFlag1 & EPL_FRAME_FLAG2_PR) >> EPL_FRAME_FLAG2_PR_SHIFT)),
                    (bFlag1 & EPL_FRAME_FLAG2_RS));
                if (Ret != kEplSuccessful)
                {
                    goto Exit;
                }

                break;
            }

            default:
            {
                break;
            }
        }
    }
#endif

    if (uiAsndServiceId < EPL_DLL_MAX_ASND_SERVICE_ID)
    {   // ASnd service ID is valid

#if EPL_DLL_PRES_CHAINING_CN != FALSE
        if (uiAsndServiceId == kEplDllAsndSyncResponse)
        {
        tEplFrame*  pTxFrameSyncRes;

            pTxFrameSyncRes = (tEplFrame *) EplDllkInstance_g.m_pTxBuffer[EPL_DLLK_TXFRAME_SYNCRES].m_pbBuffer;
            uiNodeId = (unsigned int) AmiGetByteFromLe(&pFrame->m_le_bSrcNodeId);

            if (uiNodeId == EplDllkInstance_g.m_uiSyncReqPrevNodeId)
            {
            DWORD   dwSyncDelayNs;

                dwSyncDelayNs = EplTgtTimeStampTimeDiffNs(EplDllkInstance_g.m_pSyncReqPrevTimeStamp,
                                                          pRxBuffer_p->m_pTgtTimeStamp)
                                // Transmission time for SyncReq frame
                                - (EPL_C_DLL_T_MIN_FRAME + EPL_C_DLL_T_PREAMBLE);

                // update SyncRes frame (SyncDelay and SyncNodeNumber)
                AmiSetDwordToLe(&pTxFrameSyncRes->m_Data.m_Asnd.m_Payload.m_SyncResponse.m_le_dwSyncDelay, dwSyncDelayNs);
                AmiSetDwordToLe(&pTxFrameSyncRes->m_Data.m_Asnd.m_Payload.m_SyncResponse.m_le_dwSyncNodeNumber, (DWORD) uiNodeId);

                // $$$ m.u.: CbUpdateRelativeLatencyDiff
            }
            else
            {
                AmiSetDwordToLe(&pTxFrameSyncRes->m_Data.m_Asnd.m_Payload.m_SyncResponse.m_le_dwSyncDelay, 0);
                AmiSetDwordToLe(&pTxFrameSyncRes->m_Data.m_Asnd.m_Payload.m_SyncResponse.m_le_dwSyncNodeNumber, (DWORD) 0);
            }

            // update Tx buffer in Edrv
            Ret = EdrvUpdateTxMsgBuffer(&EplDllkInstance_g.m_pTxBuffer[EPL_DLLK_TXFRAME_SYNCRES]);
            if (Ret != kEplSuccessful)
            {
                goto Exit;
            }

            // reset stored node ID
            EplDllkInstance_g.m_uiSyncReqPrevNodeId = 0;
        }
#endif

        if (EplDllkInstance_g.m_aAsndFilter[uiAsndServiceId] == kEplDllAsndFilterAny)
        {   // ASnd service ID is registered
            // forward frame via async receive FIFO to userspace
            Ret = EplDllkCalAsyncFrameReceived(pFrameInfo_p);
            if (Ret != kEplSuccessful)
            {
                goto Exit;
            }
        }
        else if (EplDllkInstance_g.m_aAsndFilter[uiAsndServiceId] == kEplDllAsndFilterLocal)
        {   // ASnd service ID is registered, but only local node ID or broadcasts
            // shall be forwarded
            uiNodeId = AmiGetByteFromLe(&pFrame->m_le_bDstNodeId);
            if ((uiNodeId == EplDllkInstance_g.m_DllConfigParam.m_uiNodeId)
                || (uiNodeId == EPL_C_ADR_BROADCAST))
            {   // ASnd frame is intended for us
                // forward frame via async receive FIFO to userspace
                Ret = EplDllkCalAsyncFrameReceived(pFrameInfo_p);
                if (Ret != kEplSuccessful)
                {
                    goto Exit;
                }
            }
        }
    }

Exit:
    return Ret;
}


//---------------------------------------------------------------------------
//
// Function:    EplDllkCbFrameTransmitted()
//
// Description: called from EdrvInterruptHandler().
//              It signals the transmission of the specified transmit buffer.
//
// Parameters:  pTxBuffer_p             = transmit buffer structure
//
// Returns:     (none)
//
// State:
//
//---------------------------------------------------------------------------

static void EplDllkCbTransmittedNmtReq(tEdrvTxBuffer * pTxBuffer_p)
{
tEplKernel      Ret = kEplSuccessful;
tEplEvent       Event;
tEplDllAsyncReqPriority Priority;
tEplNmtState    NmtState;
unsigned int    uiHandle = EPL_DLLK_TXFRAME_NMTREQ;
TGT_DLLK_DECLARE_FLAGS

    TGT_DLLK_ENTER_CRITICAL_SECTION();

    NmtState = EplDllkInstance_g.m_NmtState;

    if (NmtState <= kEplNmtGsResetConfiguration)
    {
        goto Exit;
    }

#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)
    if (NmtState >= kEplNmtMsNotActive)
    {
    tEplFrame*      pTxFrame;

        // check if this frame is a NMT command,
        // then forward this frame back to NmtMnu module,
        // because it needs the time, when this frame is
        // actually sent, to start the timer for monitoring
        // the NMT state change.

        pTxFrame = (tEplFrame *) pTxBuffer_p->m_pbBuffer;
        if ((AmiGetByteFromLe(&pTxFrame->m_le_bMessageType)
                == (BYTE) kEplMsgTypeAsnd)
            && (AmiGetByteFromLe(&pTxFrame->m_Data.m_Asnd.m_le_bServiceId)
                == (BYTE) kEplDllAsndNmtCommand))
        {   // post event directly to NmtMnu module
            Event.m_EventSink = kEplEventSinkNmtMnu;
            Event.m_EventType = kEplEventTypeNmtMnuNmtCmdSent;
            Event.m_uiSize = pTxBuffer_p->m_uiTxMsgLen;
            Event.m_pArg = pTxFrame;
//            PRINTF4("%s TxB=%p, TxF=%p, s=%u\n", __func__, pTxBuffer_p, Event.m_pArg, Event.m_uiSize);
            Ret = EplEventkPost(&Event);
            if (Ret != kEplSuccessful)
            {
                goto Exit;
            }
        }
    }
#endif

    // frame from NMT request FIFO sent
    // mark Tx-buffer as empty
    pTxBuffer_p->m_uiTxMsgLen = EPL_DLLK_BUFLEN_EMPTY;

    // post event to DLL
    Priority = kEplDllAsyncReqPrioNmt;
    Event.m_EventSink = kEplEventSinkDllk;
    Event.m_EventType = kEplEventTypeDllkFillTx;
    EPL_MEMSET(&Event.m_NetTime, 0x00, sizeof(Event.m_NetTime));
    Event.m_pArg = &Priority;
    Event.m_uiSize = sizeof(Priority);
    Ret = EplEventkPost(&Event);
    if (Ret != kEplSuccessful)
    {
        goto Exit;
    }

Exit:
    if (Ret != kEplSuccessful)
    {
    DWORD   dwArg;

        BENCHMARK_MOD_02_TOGGLE(7);

        dwArg = EplDllkInstance_g.m_DllState | (uiHandle << 16);

        // Error event for API layer
        Ret = EplEventkPostError(kEplEventSourceDllk,
                        Ret,
                        sizeof(dwArg),
                        &dwArg);
    }

    TGT_DLLK_LEAVE_CRITICAL_SECTION();

    return;
}


//---------------------------------------------------------------------------
//
// Function:    EplDllkCbFrameTransmitted()
//
// Description: called from EdrvInterruptHandler().
//              It signals the transmission of the specified transmit buffer.
//
// Parameters:  pTxBuffer_p             = transmit buffer structure
//
// Returns:     (none)
//
// State:
//
//---------------------------------------------------------------------------

static void EplDllkCbTransmittedNonEpl(tEdrvTxBuffer * pTxBuffer_p)
{
tEplKernel      Ret = kEplSuccessful;
tEplEvent       Event;
tEplDllAsyncReqPriority Priority;
tEplNmtState    NmtState;
unsigned int    uiHandle = EPL_DLLK_TXFRAME_NONEPL;
TGT_DLLK_DECLARE_FLAGS

    TGT_DLLK_ENTER_CRITICAL_SECTION();

    NmtState = EplDllkInstance_g.m_NmtState;

    if (NmtState <= kEplNmtGsResetConfiguration)
    {
        goto Exit;
    }

    // frame from generic priority FIFO sent
    // mark Tx-buffer as empty
    pTxBuffer_p->m_uiTxMsgLen = EPL_DLLK_BUFLEN_EMPTY;

    // post event to DLL
    Priority = kEplDllAsyncReqPrioGeneric;
    Event.m_EventSink = kEplEventSinkDllk;
    Event.m_EventType = kEplEventTypeDllkFillTx;
    EPL_MEMSET(&Event.m_NetTime, 0x00, sizeof(Event.m_NetTime));
    Event.m_pArg = &Priority;
    Event.m_uiSize = sizeof(Priority);
    Ret = EplEventkPost(&Event);

Exit:
    if (Ret != kEplSuccessful)
    {
    DWORD   dwArg;

        BENCHMARK_MOD_02_TOGGLE(7);

        dwArg = EplDllkInstance_g.m_DllState | (uiHandle << 16);

        // Error event for API layer
        Ret = EplEventkPostError(kEplEventSourceDllk,
                        Ret,
                        sizeof(dwArg),
                        &dwArg);
    }

    TGT_DLLK_LEAVE_CRITICAL_SECTION();

    return;
}


//---------------------------------------------------------------------------
//
// Function:    EplDllkCbFrameTransmitted()
//
// Description: called from EdrvInterruptHandler().
//              It signals the transmission of the specified transmit buffer.
//
// Parameters:  pTxBuffer_p             = transmit buffer structure
//
// Returns:     (none)
//
// State:
//
//---------------------------------------------------------------------------

#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)
#if (EPL_DLL_DISABLE_EDRV_CYCLIC != FALSE)
static void EplDllkCbTransmittedPreq(tEdrvTxBuffer * pTxBuffer_p)
{
tEplKernel      Ret = kEplSuccessful;
tEplNmtState    NmtState;
unsigned int    uiHandle = EPL_DLLK_TXFRAME_PREQ;
TGT_DLLK_DECLARE_FLAGS

    TGT_DLLK_ENTER_CRITICAL_SECTION();

    NmtState = EplDllkInstance_g.m_NmtState;

    if (NmtState <= kEplNmtGsResetConfiguration)
    {
        goto Exit;
    }

#if ((((EPL_MODULE_INTEGRATION) & (EPL_MODULE_PDOK)) != 0) \
    && (EPL_DLL_PRES_READY_AFTER_SOC == FALSE)) \
    || (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)
    // PRes resp. PReq frame sent

    #if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)
    {
        if (NmtState >= kEplNmtMsPreOperational2)
        {   // local node runs as MN in PREOP2
            if (pTxBuffer_p == &EplDllkInstance_g.m_pTxBuffer[EPL_DLLK_TXFRAME_PRES + EplDllkInstance_g.m_bCurTxBufferOffsetCycle])
            {   // own Pres on MN, trigger SoA
                Ret = EplDllkChangeState(kEplNmtEventDllMeSoaTrig, NmtState);
            }
            else // if (pTxBuffer_p->m_EplMsgType == kEplMsgTypePreq)
            {
                // start PRes Timer
#if EPL_TIMER_USE_HIGHRES != FALSE
                Ret = EplTimerHighReskModifyTimerNs(&EplDllkInstance_g.m_TimerHdlResponse,
                    EplDllkInstance_g.m_pCurNodeInfo->m_dwPresTimeoutNs,
                    EplDllkCbMnTimerResponse,
                    0L,
                    FALSE);
#endif
            }
        }
    }
    #endif
#endif

Exit:
    if (Ret != kEplSuccessful)
    {
    DWORD   dwArg;

        BENCHMARK_MOD_02_TOGGLE(7);

        dwArg = EplDllkInstance_g.m_DllState | (uiHandle << 16);

        // Error event for API layer
        Ret = EplEventkPostError(kEplEventSourceDllk,
                        Ret,
                        sizeof(dwArg),
                        &dwArg);
    }

    TGT_DLLK_LEAVE_CRITICAL_SECTION();

    return;
}
#endif
#endif


//---------------------------------------------------------------------------
//
// Function:    EplDllkCbFrameTransmitted()
//
// Description: called from EdrvInterruptHandler().
//              It signals the transmission of the specified transmit buffer.
//
// Parameters:  pTxBuffer_p             = transmit buffer structure
//
// Returns:     (none)
//
// State:
//
//---------------------------------------------------------------------------

#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)
static void EplDllkCbTransmittedSoa(tEdrvTxBuffer * pTxBuffer_p)
{
tEplKernel      Ret = kEplSuccessful;
tEplNmtState    NmtState;
unsigned int    uiHandle = EPL_DLLK_TXFRAME_SOA;
TGT_DLLK_DECLARE_FLAGS

    UNUSED_PARAMETER(pTxBuffer_p);

    TGT_DLLK_ENTER_CRITICAL_SECTION();

    NmtState = EplDllkInstance_g.m_NmtState;

    if (NmtState <= kEplNmtGsResetConfiguration)
    {
        goto Exit;
    }

    // SoA frame sent

    // check if we are invited
#if (EPL_DLL_DISABLE_EDRV_CYCLIC == FALSE)
    // old handling only in PreOp1
    if (EplDllkInstance_g.m_DllState == kEplDllMsNonCyclic)
#endif
    if (EplDllkInstance_g.m_auiLastTargetNodeId[EplDllkInstance_g.m_bCurLastSoaReq] == EplDllkInstance_g.m_DllConfigParam.m_uiNodeId)
    {
        switch (EplDllkInstance_g.m_aLastReqServiceId[EplDllkInstance_g.m_bCurLastSoaReq])
        {
            case kEplDllReqServiceStatus:
            {   // StatusRequest
                if (EplDllkInstance_g.m_pTxBuffer[EPL_DLLK_TXFRAME_STATUSRES + EplDllkInstance_g.m_bCurTxBufferOffsetStatusRes].m_pbBuffer != NULL)
                {   // StatusRes does exist

                    // send StatusRes
                    Ret = EdrvSendTxMsg(&EplDllkInstance_g.m_pTxBuffer[EPL_DLLK_TXFRAME_STATUSRES + EplDllkInstance_g.m_bCurTxBufferOffsetStatusRes]);
                    if (Ret != kEplSuccessful)
                    {
                        goto Exit;
                    }
                    TGT_DBG_SIGNAL_TRACE_POINT(8);

                }

                break;
            }

            case kEplDllReqServiceIdent:
            {   // IdentRequest
                if (EplDllkInstance_g.m_pTxBuffer[EPL_DLLK_TXFRAME_IDENTRES + EplDllkInstance_g.m_bCurTxBufferOffsetIdentRes].m_pbBuffer != NULL)
                {   // IdentRes does exist

                    // send IdentRes
                    Ret = EdrvSendTxMsg(&EplDllkInstance_g.m_pTxBuffer[EPL_DLLK_TXFRAME_IDENTRES + EplDllkInstance_g.m_bCurTxBufferOffsetIdentRes]);
                    if (Ret != kEplSuccessful)
                    {
                        goto Exit;
                    }
                    TGT_DBG_SIGNAL_TRACE_POINT(7);
                }

                break;
            }

            case kEplDllReqServiceNmtRequest:
            {   // NmtRequest
                if (EplDllkInstance_g.m_pTxBuffer[EPL_DLLK_TXFRAME_NMTREQ + EplDllkInstance_g.m_bCurTxBufferOffsetNmtReq].m_pbBuffer != NULL)
                {   // NmtRequest does exist
                    // check if frame is not empty and not being filled
                    if (EplDllkInstance_g.m_pTxBuffer[EPL_DLLK_TXFRAME_NMTREQ + EplDllkInstance_g.m_bCurTxBufferOffsetNmtReq].m_uiTxMsgLen > EPL_DLLK_BUFLEN_FILLING)
                    {
                        // send NmtRequest
                        Ret = EdrvSendTxMsg(&EplDllkInstance_g.m_pTxBuffer[EPL_DLLK_TXFRAME_NMTREQ + EplDllkInstance_g.m_bCurTxBufferOffsetNmtReq]);
                        if (Ret != kEplSuccessful)
                        {
                            goto Exit;
                        }

                        EplDllkInstance_g.m_bCurTxBufferOffsetNmtReq ^= 1;
                    }
                }

                break;
            }

            case kEplDllReqServiceUnspecified:
            {   // unspecified invite
                if (EplDllkInstance_g.m_pTxBuffer[EPL_DLLK_TXFRAME_NONEPL + EplDllkInstance_g.m_bCurTxBufferOffsetNonEpl].m_pbBuffer != NULL)
                {   // non-EPL frame does exist
                    // check if frame is not empty and not being filled
                    if (EplDllkInstance_g.m_pTxBuffer[EPL_DLLK_TXFRAME_NONEPL + EplDllkInstance_g.m_bCurTxBufferOffsetNonEpl].m_uiTxMsgLen > EPL_DLLK_BUFLEN_FILLING)
                    {
                        // send non-EPL frame
                        Ret = EdrvSendTxMsg(&EplDllkInstance_g.m_pTxBuffer[EPL_DLLK_TXFRAME_NONEPL + EplDllkInstance_g.m_bCurTxBufferOffsetNonEpl]);
                        if (Ret != kEplSuccessful)
                        {
                            goto Exit;
                        }

                        EplDllkInstance_g.m_bCurTxBufferOffsetNonEpl ^= 1;
                    }
                }

                break;
            }

            default:
            {
                break;
            }
        }

        // ASnd frame was sent, remove the request
        EplDllkInstance_g.m_aLastReqServiceId[EplDllkInstance_g.m_bCurLastSoaReq] = kEplDllReqServiceNo;
    }

    // reprogram timer in PREOP1
#if EPL_TIMER_USE_HIGHRES != FALSE
    if ((EplDllkInstance_g.m_DllState == kEplDllMsNonCyclic)
        && (EplDllkInstance_g.m_DllConfigParam.m_dwAsyncSlotTimeout != 0))
    {
        Ret = EplTimerHighReskModifyTimerNs(&EplDllkInstance_g.m_TimerHdlCycle,
            EplDllkInstance_g.m_DllConfigParam.m_dwAsyncSlotTimeout,
            EplDllkCbMnTimerCycle,
            0L,
            FALSE);
        if (Ret != kEplSuccessful)
        {
            goto Exit;
        }

        // forward event to ErrorHandler and DLLk module
        Ret = EplDllkPostEvent(kEplEventTypeDllkCycleFinish);
        if (Ret != kEplSuccessful)
        {
            goto Exit;
        }
    }
#endif

    if ((EplDllkInstance_g.m_DllState > kEplDllMsNonCyclic)
        && (EplDllkInstance_g.m_DllConfigParam.m_uiSyncNodeId > EPL_C_ADR_SYNC_ON_SOC)
        && (EplDllkInstance_g.m_fSyncProcessed == FALSE))
    {   // cyclic state is active, so preprocessing is necessary
        EplDllkInstance_g.m_fSyncProcessed = TRUE;
        Ret = EplDllkPostEvent(kEplEventTypeSync);
        if (Ret != kEplSuccessful)
        {
            goto Exit;
        }
    }

Exit:
    if (Ret != kEplSuccessful)
    {
    DWORD   dwArg;

        BENCHMARK_MOD_02_TOGGLE(7);

        dwArg = EplDllkInstance_g.m_DllState | (uiHandle << 16);

        // Error event for API layer
        Ret = EplEventkPostError(kEplEventSourceDllk,
                        Ret,
                        sizeof(dwArg),
                        &dwArg);
    }

    TGT_DLLK_LEAVE_CRITICAL_SECTION();

    return;
}
#endif


//---------------------------------------------------------------------------
//
// Function:    EplDllkCbFrameTransmitted()
//
// Description: called from EdrvInterruptHandler().
//              It signals the transmission of the specified transmit buffer.
//
// Parameters:  pTxBuffer_p             = transmit buffer structure
//
// Returns:     (none)
//
// State:
//
//---------------------------------------------------------------------------

#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)
static void EplDllkCbTransmittedSoc(tEdrvTxBuffer * pTxBuffer_p)
{
tEplKernel      Ret = kEplSuccessful;
tEplNmtState    NmtState;
unsigned int    uiHandle = EPL_DLLK_TXFRAME_SOC;
TGT_DLLK_DECLARE_FLAGS

    UNUSED_PARAMETER(pTxBuffer_p);

    TGT_DLLK_ENTER_CRITICAL_SECTION();

    NmtState = EplDllkInstance_g.m_NmtState;

    if (NmtState <= kEplNmtGsResetConfiguration)
    {
        goto Exit;
    }

    // SoC frame sent
#if EPL_DLL_DISABLE_EDRV_CYCLIC != FALSE
    if (EplDllkInstance_g.m_DllState == kEplDllMsWaitPreqTrig)
    {

#if EPL_TIMER_USE_HIGHRES != FALSE
        if (EplDllkInstance_g.m_DllConfigParam.m_dwWaitSocPreq != 0)
        {   // start WaitSoCPReq Timer
            Ret = EplTimerHighReskModifyTimerNs(&EplDllkInstance_g.m_TimerHdlResponse,
                EplDllkInstance_g.m_DllConfigParam.m_dwWaitSocPreq,
                EplDllkCbMnTimerResponse,
                0L,
                FALSE);
            if (Ret != kEplSuccessful)
            {
                goto Exit;
            }
        }
        else
#endif
        {   // immediately send first PReq
            Ret = EplDllkChangeState(kEplNmtEventDllMePresTimeout, NmtState);
            if (Ret != kEplSuccessful)
            {
                goto Exit;
            }
        }
    }
#else
    Ret = EplDllkChangeState(kEplNmtEventDllMeAsndTimeout, NmtState);
    if (Ret != kEplSuccessful)
    {
        goto Exit;
    }
#endif

Exit:
    if (Ret != kEplSuccessful)
    {
    DWORD   dwArg;

        BENCHMARK_MOD_02_TOGGLE(7);

        dwArg = EplDllkInstance_g.m_DllState | (uiHandle << 16);

        // Error event for API layer
        Ret = EplEventkPostError(kEplEventSourceDllk,
                        Ret,
                        sizeof(dwArg),
                        &dwArg);
    }

    TGT_DLLK_LEAVE_CRITICAL_SECTION();

    return;
}
#endif


//---------------------------------------------------------------------------
//
// Function:    EplDllkCbCyclicError()
//
// Description: called from EdrvInterruptHandler().
//              It signals the transmission of the specified transmit buffer.
//
// Parameters:  pTxBuffer_p             = transmit buffer structure
//
// Returns:     tEplKernel
//
// State:
//
//---------------------------------------------------------------------------

#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)
#if EPL_DLL_DISABLE_EDRV_CYCLIC == FALSE
static tEplKernel EplDllkCbCyclicError(tEplKernel ErrorCode_p, tEdrvTxBuffer * pTxBuffer_p)
{
tEplKernel      Ret = kEplSuccessful;
tEplNmtState    NmtState;
unsigned int    uiHandle = 0;
DWORD           dwArg;
TGT_DLLK_DECLARE_FLAGS

    UNUSED_PARAMETER(pTxBuffer_p);

    TGT_DLLK_ENTER_CRITICAL_SECTION();

    NmtState = EplDllkInstance_g.m_NmtState;

    if (NmtState <= kEplNmtGsResetConfiguration)
    {
        goto Exit;
    }

    if (pTxBuffer_p != NULL)
    {
        uiHandle = (unsigned int) (pTxBuffer_p - EplDllkInstance_g.m_pTxBuffer);
    }

    BENCHMARK_MOD_02_TOGGLE(7);

    switch (ErrorCode_p)
    {
        case kEplEdrvCurTxListEmpty:
        case kEplEdrvTxListNotFinishedYet:
        case kEplEdrvNoFreeTxDesc:
        {
        tEplErrorHandlerkEvent  DllEvent;

            DllEvent.m_ulDllErrorEvents = EPL_DLL_ERR_MN_CYCTIMEEXCEED;
            DllEvent.m_uiNodeId = uiHandle;
            DllEvent.m_NmtState = NmtState;
            DllEvent.m_EplError = ErrorCode_p;

            Ret = EplErrorHandlerkPostError(&DllEvent);

            break;
        }

        default:
        {
            dwArg = EplDllkInstance_g.m_DllState | (uiHandle << 16);

            // Error event for API layer
            Ret = EplEventkPostError(kEplEventSourceDllk,
                            ErrorCode_p,
                            sizeof(dwArg),
                            &dwArg);
            break;
        }
    }

Exit:
    TGT_DLLK_LEAVE_CRITICAL_SECTION();

    return Ret;
}
#endif
#endif


//---------------------------------------------------------------------------
//
// Function:    EplDllkPostEvent
//
// Description: posts events to  DLL itself
//
// Parameters:  pDllEvent_p = pointer to event-structure
//
// Returns:     tEpKernel  = errorcode
//
// State:
//
//---------------------------------------------------------------------------
static tEplKernel EplDllkPostEvent(tEplEventType EventType_p)
{
tEplKernel              Ret;
tEplEvent               Event;

    Event.m_EventSink = kEplEventSinkDllk;
    Event.m_EventType = EventType_p;
    Event.m_uiSize = 0;
    Event.m_pArg = NULL;
    Ret = EplEventkPost(&Event);

    return Ret;

}


//---------------------------------------------------------------------------
//
// Function:    EplDllkForwardRpdo
//
// Description: This function is called by DLL if PRes or PReq frame was
//              received. It posts the frame to the event queue.
//              It is called in states NMT_CS_READY_TO_OPERATE and NMT_CS_OPERATIONAL.
//              The passed PDO needs not to be valid.
//
// Parameters:  pFrameInfo_p            = pointer to frame info structure
//
// Returns:     tEplKernel              = error code
//
//
// State:
//
//---------------------------------------------------------------------------

static tEplKernel EplDllkForwardRpdo(tEplFrameInfo * pFrameInfo_p)
{
tEplKernel      Ret = kEplSuccessful;

    if (EplDllkInstance_g.m_pfnCbProcessRpdo != NULL)
    {
        Ret = EplDllkInstance_g.m_pfnCbProcessRpdo(pFrameInfo_p);
    }

    return Ret;
}


//---------------------------------------------------------------------------
//
// Function:    EplDllkProcessTpdo
//
// Description: This function forward the specified TPDO for processing to
//              the registered callback function (i.e. to the PDO module).
//
// Parameters:  pFrameInfo_p            = pointer to frame info structure
//
// Returns:     tEplKernel              = error code
//
//
// State:
//
//---------------------------------------------------------------------------

static tEplKernel EplDllkProcessTpdo(tEplFrameInfo * pFrameInfo_p, BOOL fReadyFlag_p)
{
tEplKernel      Ret = kEplSuccessful;

    if (EplDllkInstance_g.m_pfnCbProcessTpdo != NULL)
    {
        Ret = EplDllkInstance_g.m_pfnCbProcessTpdo(pFrameInfo_p, fReadyFlag_p);
    }

    return Ret;
}


//---------------------------------------------------------------------------
//
// Function:    EplDllkUpdateFrameIdentRes()
//
// Description: update IdentRes frame
//
// Parameters:  pTxBuffer_p             = Tx buffer of IdentRes
//              NmtState_p              = current NMT state
//
// Returns:     tEplKernel              = error code
//
//
// State:
//
//---------------------------------------------------------------------------

static tEplKernel EplDllkUpdateFrameIdentRes(tEdrvTxBuffer* pTxBuffer_p, tEplNmtState NmtState_p)
{
tEplKernel      Ret = kEplSuccessful;
tEplFrame*      pTxFrame;

    pTxFrame = (tEplFrame *) pTxBuffer_p->m_pbBuffer;

    // update frame (NMT state, RD, RS, PR flags)
    AmiSetByteToLe(&pTxFrame->m_Data.m_Asnd.m_Payload.m_IdentResponse.m_le_bNmtStatus, (BYTE) NmtState_p);
    AmiSetByteToLe(&pTxFrame->m_Data.m_Asnd.m_Payload.m_IdentResponse.m_le_bFlag2, EplDllkInstance_g.m_bFlag2);

#if (EDRV_AUTO_RESPONSE != FALSE)
    if (NmtState_p < kEplNmtMsNotActive)
    {
        // update Tx buffer in Edrv
        Ret = EdrvUpdateTxMsgBuffer(pTxBuffer_p);
    }
#endif

    return Ret;
}


//---------------------------------------------------------------------------
//
// Function:    EplDllkUpdateFrameStatusRes()
//
// Description: update StatusRes frame
//
// Parameters:  pTxBuffer_p             = Tx buffer of StatusRes
//              NmtState_p              = current NMT state
//
// Returns:     tEplKernel              = error code
//
//
// State:
//
//---------------------------------------------------------------------------

static tEplKernel EplDllkUpdateFrameStatusRes(tEdrvTxBuffer* pTxBuffer_p, tEplNmtState NmtState_p)
{
tEplKernel      Ret = kEplSuccessful;
tEplFrame*      pTxFrame;

    pTxFrame = (tEplFrame *) pTxBuffer_p->m_pbBuffer;

    // update frame (NMT state, RD, RS, PR, EC, EN flags)
    AmiSetByteToLe(&pTxFrame->m_Data.m_Asnd.m_Payload.m_StatusResponse.m_le_bNmtStatus, (BYTE) NmtState_p);
    AmiSetByteToLe(&pTxFrame->m_Data.m_Asnd.m_Payload.m_StatusResponse.m_le_bFlag2, EplDllkInstance_g.m_bFlag2);
    AmiSetByteToLe(&pTxFrame->m_Data.m_Asnd.m_Payload.m_StatusResponse.m_le_bFlag1, EplDllkInstance_g.m_bFlag1);

#if (EDRV_AUTO_RESPONSE != FALSE)
    if (NmtState_p < kEplNmtMsNotActive)
    {
        // update Tx buffer in Edrv
        Ret = EdrvUpdateTxMsgBuffer(pTxBuffer_p);
    }
#endif

    return Ret;
}


//---------------------------------------------------------------------------
//
// Function:    EplDllkUpdateFramePres()
//
// Description: update PRes frame
//
// Parameters:  pTxBuffer_p             = Tx buffer
//              NmtState_p              = current NMT state
//
// Returns:     tEplKernel              = error code
//
//
// State:
//
//---------------------------------------------------------------------------

static tEplKernel EplDllkUpdateFramePres(tEdrvTxBuffer* pTxBuffer_p, tEplNmtState NmtState_p)
{
tEplKernel      Ret = kEplSuccessful;
tEplFrame*      pTxFrame;
BYTE            bFlag1;

    pTxFrame = (tEplFrame *) pTxBuffer_p->m_pbBuffer;

    // update frame (NMT state, RD, RS, PR, MS, EN flags)
    AmiSetByteToLe(&pTxFrame->m_Data.m_Pres.m_le_bNmtStatus, (BYTE) NmtState_p);
    AmiSetByteToLe(&pTxFrame->m_Data.m_Pres.m_le_bFlag2, EplDllkInstance_g.m_bFlag2);

    // get RD flag
    bFlag1 = AmiGetByteFromLe(&pTxFrame->m_Data.m_Pres.m_le_bFlag1) & EPL_FRAME_FLAG1_RD;

    if (EplDllkInstance_g.m_DllConfigParam.m_uiMultiplCycleCnt > 0)
    {   // set MS flag, because PRes will be sent multiplexed with other CNs
        bFlag1 |= EPL_FRAME_FLAG1_MS;
    }

    // add EN flag from Error signaling module
    bFlag1 |= EplDllkInstance_g.m_bFlag1 & EPL_FRAME_FLAG1_EN;

    if (NmtState_p != kEplNmtCsOperational)
    {   // mark PDO as invalid in all NMT states but OPERATIONAL
        // reset only RD flag
        bFlag1 &= ~EPL_FRAME_FLAG1_RD;
    }

    // update frame (flag1)
    AmiSetByteToLe(&pTxFrame->m_Data.m_Pres.m_le_bFlag1, bFlag1);

#if (EDRV_AUTO_RESPONSE != FALSE)
//    if (NmtState_p < kEplNmtMsNotActive)
    {   // currently, this function is only called on CN
        // update Tx buffer in Edrv
        Ret = EdrvUpdateTxMsgBuffer(pTxBuffer_p);
    }
#endif

    return Ret;
}


//---------------------------------------------------------------------------
//
// Function:    EplDllkCreateTxFrame
//
// Description: creates the buffer for a Tx frame and registers it to the
//              ethernet driver
//
// Parameters:  puiHandle_p             = IN: handle to last allocated frame buffer
//                                            used for faster search for PReq buffers
//                                        OUT: handle to new frame buffer
//              puiFrameSize_p          = IN/OUT: pointer to size of frame
//                                        returned size is always equal or larger than
//                                        requested size, if that is not possible
//                                        an error will be returned
//              MsgType_p               = EPL message type
//              ServiceId_p             = Service ID in case of ASnd frame, otherwise
//                                        kEplDllAsndNotDefined
//
// Returns:     tEplKernel              = error code
//
//
// State:
//
//---------------------------------------------------------------------------

static tEplKernel EplDllkCreateTxFrame (unsigned int * puiHandle_p,
                                 unsigned int * puiFrameSize_p,
                                 tEplMsgType MsgType_p,
                                 tEplDllAsndServiceId ServiceId_p)
{
tEplKernel      Ret = kEplSuccessful;
tEplFrame*      pTxFrame;
unsigned int    uiHandle = *puiHandle_p;
tEdrvTxBuffer*  pTxBuffer = NULL;
unsigned int    nIndex = 0;

    switch (MsgType_p)
    {
        case kEplMsgTypeAsnd:
        {
            // search for fixed Tx buffers
            switch (ServiceId_p)
            {
                case kEplDllAsndIdentResponse:
                {
                    uiHandle = EPL_DLLK_TXFRAME_IDENTRES;
                    break;
                }

                case kEplDllAsndStatusResponse:
                {
                    uiHandle = EPL_DLLK_TXFRAME_STATUSRES;
                    break;
                }

                case kEplDllAsndNmtRequest:
                case kEplDllAsndNmtCommand:
                {
                    uiHandle = EPL_DLLK_TXFRAME_NMTREQ;
                    break;
                }

#if EPL_DLL_PRES_CHAINING_CN != FALSE
                case kEplDllAsndSyncResponse:
                {
                    uiHandle = EPL_DLLK_TXFRAME_SYNCRES;
                    break;
                }
#endif

                case kEplDllAsndNotDefined:
                {
                    Ret = kEplDllInvalidParam;
                    goto Exit;
                }

                case kEplDllAsndSdo:
#if (EPL_DLL_PRES_CHAINING_CN == FALSE) && (EPL_DLL_PRES_CHAINING_MN != FALSE)
                case kEplDllAsndSyncResponse:
#endif
                {
                    Ret = kEplEdrvBufNotExisting;
                    goto Exit;
                }
            }
            break;
        }

        case kEplMsgTypeNonEpl:
        {
            uiHandle = EPL_DLLK_TXFRAME_NONEPL;
            break;
        }

        case kEplMsgTypePres:
        {
            uiHandle = EPL_DLLK_TXFRAME_PRES;
            break;
        }

#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)
        case kEplMsgTypeSoc:
        {
            uiHandle = EPL_DLLK_TXFRAME_SOC;
            break;
        }

        case kEplMsgTypeSoa:
        {
            uiHandle = EPL_DLLK_TXFRAME_SOA;
            break;
        }

        case kEplMsgTypePreq:
        {   // look for free entry
            if ((uiHandle < EPL_DLLK_TXFRAME_PREQ)
                || (uiHandle >= EplDllkInstance_g.m_uiMaxTxFrames))
            {   // start with first PReq buffer
                uiHandle = EPL_DLLK_TXFRAME_PREQ;
            }
            // otherwise start with last allocated handle
            pTxBuffer = &EplDllkInstance_g.m_pTxBuffer[uiHandle];
            for (; uiHandle < EplDllkInstance_g.m_uiMaxTxFrames; uiHandle++, pTxBuffer++)
            {
                if (pTxBuffer->m_pbBuffer == NULL)
                {   // free entry found
                    break;
                }
            }
            if (pTxBuffer->m_pbBuffer != NULL)
            {
                Ret = kEplEdrvNoFreeBufEntry;
                goto Exit;
            }
            break;
        }
#else
        default:
        {
            Ret = kEplEdrvBufNotExisting;
            goto Exit;
        }
#endif
    }

    *puiHandle_p = uiHandle;

    for ( ; nIndex < 2; nIndex++, uiHandle++)
    {
        // test if requested entry is free
        pTxBuffer = &EplDllkInstance_g.m_pTxBuffer[uiHandle];
        if (pTxBuffer->m_pbBuffer != NULL)
        {   // entry is not free
            Ret = kEplEdrvNoFreeBufEntry;
            goto Exit;
        }

        // setup Tx buffer
        pTxBuffer->m_uiMaxBufferLen = *puiFrameSize_p;

        Ret = EdrvAllocTxMsgBuffer(pTxBuffer);
        if (Ret != kEplSuccessful)
        {   // error occurred while registering Tx frame
            goto Exit;
        }

        // because buffer size may be larger than requested
        // memorize real length of frame
        pTxBuffer->m_uiTxMsgLen = *puiFrameSize_p;

        // initialize time offset
        pTxBuffer->m_dwTimeOffsetNs = 0;

        // fill whole frame with 0
        EPL_MEMSET(pTxBuffer->m_pbBuffer, 0, pTxBuffer->m_uiMaxBufferLen);

        pTxFrame = (tEplFrame *) pTxBuffer->m_pbBuffer;

        if (MsgType_p != kEplMsgTypeNonEpl)
        {   // fill out Frame only if it is an EPL frame
            // ethertype
            AmiSetWordToBe(&pTxFrame->m_be_wEtherType, EPL_C_DLL_ETHERTYPE_EPL);
            // source node ID
            AmiSetByteToLe(&pTxFrame->m_le_bSrcNodeId, (BYTE) EplDllkInstance_g.m_DllConfigParam.m_uiNodeId);
            // source MAC address
            EPL_MEMCPY(&pTxFrame->m_be_abSrcMac[0], &EplDllkInstance_g.m_be_abLocalMac[0], 6);
            switch (MsgType_p)
            {
                case kEplMsgTypeAsnd:
                {
                    // destination MAC address
                    AmiSetQword48ToBe(&pTxFrame->m_be_abDstMac[0], EPL_C_DLL_MULTICAST_ASND);
                    // destination node ID
                    switch (ServiceId_p)
                    {
                        case kEplDllAsndIdentResponse:
                        {
                            // EPL profile version
                            AmiSetByteToLe(&pTxFrame->m_Data.m_Asnd.m_Payload.m_IdentResponse.m_le_bEplProfileVersion,
                                (BYTE) EPL_SPEC_VERSION);
                            // FeatureFlags
                            AmiSetDwordToLe(&pTxFrame->m_Data.m_Asnd.m_Payload.m_IdentResponse.m_le_dwFeatureFlags,
                                EplDllkInstance_g.m_DllConfigParam.m_dwFeatureFlags);
                            // MTU
                            AmiSetWordToLe(&pTxFrame->m_Data.m_Asnd.m_Payload.m_IdentResponse.m_le_wMtu,
                                (WORD) EplDllkInstance_g.m_DllConfigParam.m_uiAsyncMtu);
                            // PollInSize
                            AmiSetWordToLe(&pTxFrame->m_Data.m_Asnd.m_Payload.m_IdentResponse.m_le_wPollInSize,
                                (WORD)EplDllkInstance_g.m_DllConfigParam.m_uiPreqActPayloadLimit);
                            // PollOutSize
                            AmiSetWordToLe(&pTxFrame->m_Data.m_Asnd.m_Payload.m_IdentResponse.m_le_wPollOutSize,
                                (WORD)EplDllkInstance_g.m_DllConfigParam.m_uiPresActPayloadLimit);
                            // ResponseTime / PresMaxLatency
                            AmiSetDwordToLe(&pTxFrame->m_Data.m_Asnd.m_Payload.m_IdentResponse.m_le_dwResponseTime,
                                EplDllkInstance_g.m_DllConfigParam.m_dwPresMaxLatency);
                            // DeviceType
                            AmiSetDwordToLe(&pTxFrame->m_Data.m_Asnd.m_Payload.m_IdentResponse.m_le_dwDeviceType,
                                EplDllkInstance_g.m_DllIdentParam.m_dwDeviceType);
                            // VendorId
                            AmiSetDwordToLe(&pTxFrame->m_Data.m_Asnd.m_Payload.m_IdentResponse.m_le_dwVendorId,
                                EplDllkInstance_g.m_DllIdentParam.m_dwVendorId);
                            // ProductCode
                            AmiSetDwordToLe(&pTxFrame->m_Data.m_Asnd.m_Payload.m_IdentResponse.m_le_dwProductCode,
                                EplDllkInstance_g.m_DllIdentParam.m_dwProductCode);
                            // RevisionNumber
                            AmiSetDwordToLe(&pTxFrame->m_Data.m_Asnd.m_Payload.m_IdentResponse.m_le_dwRevisionNumber,
                                EplDllkInstance_g.m_DllIdentParam.m_dwRevisionNumber);
                            // SerialNumber
                            AmiSetDwordToLe(&pTxFrame->m_Data.m_Asnd.m_Payload.m_IdentResponse.m_le_dwSerialNumber,
                                EplDllkInstance_g.m_DllIdentParam.m_dwSerialNumber);
                            // VendorSpecificExt1
                            AmiSetQword64ToLe(&pTxFrame->m_Data.m_Asnd.m_Payload.m_IdentResponse.m_le_qwVendorSpecificExt1,
                                EplDllkInstance_g.m_DllIdentParam.m_qwVendorSpecificExt1);
                            // VerifyConfigurationDate
                            AmiSetDwordToLe(&pTxFrame->m_Data.m_Asnd.m_Payload.m_IdentResponse.m_le_dwVerifyConfigurationDate,
                                EplDllkInstance_g.m_DllIdentParam.m_dwVerifyConfigurationDate);
                            // VerifyConfigurationTime
                            AmiSetDwordToLe(&pTxFrame->m_Data.m_Asnd.m_Payload.m_IdentResponse.m_le_dwVerifyConfigurationTime,
                                EplDllkInstance_g.m_DllIdentParam.m_dwVerifyConfigurationTime);
                            // ApplicationSwDate
                            AmiSetDwordToLe(&pTxFrame->m_Data.m_Asnd.m_Payload.m_IdentResponse.m_le_dwApplicationSwDate,
                                EplDllkInstance_g.m_DllIdentParam.m_dwApplicationSwDate);
                            // ApplicationSwTime
                            AmiSetDwordToLe(&pTxFrame->m_Data.m_Asnd.m_Payload.m_IdentResponse.m_le_dwApplicationSwTime,
                                EplDllkInstance_g.m_DllIdentParam.m_dwApplicationSwTime);
                            // IPAddress
                            AmiSetDwordToLe(&pTxFrame->m_Data.m_Asnd.m_Payload.m_IdentResponse.m_le_dwIpAddress,
                                EplDllkInstance_g.m_DllIdentParam.m_dwIpAddress);
                            // SubnetMask
                            AmiSetDwordToLe(&pTxFrame->m_Data.m_Asnd.m_Payload.m_IdentResponse.m_le_dwSubnetMask,
                                EplDllkInstance_g.m_DllIdentParam.m_dwSubnetMask);
                            // DefaultGateway
                            AmiSetDwordToLe(&pTxFrame->m_Data.m_Asnd.m_Payload.m_IdentResponse.m_le_dwDefaultGateway,
                                EplDllkInstance_g.m_DllIdentParam.m_dwDefaultGateway);
                            // HostName
                            EPL_MEMCPY(&pTxFrame->m_Data.m_Asnd.m_Payload.m_IdentResponse.m_le_sHostname[0],
                                &EplDllkInstance_g.m_DllIdentParam.m_sHostname[0],
                                sizeof (EplDllkInstance_g.m_DllIdentParam.m_sHostname));
                            // VendorSpecificExt2
                            EPL_MEMCPY(&pTxFrame->m_Data.m_Asnd.m_Payload.m_IdentResponse.m_le_abVendorSpecificExt2[0],
                                &EplDllkInstance_g.m_DllIdentParam.m_abVendorSpecificExt2[0],
                                sizeof (EplDllkInstance_g.m_DllIdentParam.m_abVendorSpecificExt2));

                            // fall-through
                        }

                        case kEplDllAsndStatusResponse:
                        {   // IdentResponses and StatusResponses are Broadcast

                            AmiSetByteToLe(&pTxFrame->m_le_bDstNodeId, (BYTE) EPL_C_ADR_BROADCAST);

                            break;
                        }

#if EPL_DLL_PRES_CHAINING_CN != FALSE
                        case kEplDllAsndSyncResponse:
                        {   // SyncRes destination node ID is MN node ID
                            AmiSetByteToLe(&pTxFrame->m_le_bDstNodeId, (BYTE) EPL_C_ADR_MN_DEF_NODE_ID);
                            // Latency
                            AmiSetDwordToLe(&pTxFrame->m_Data.m_Asnd.m_Payload.m_SyncResponse.m_le_dwLatency,
                                EplDllkInstance_g.m_DllConfigParam.m_dwSyncResLatency);
                            // SyncStatus: PResMode disabled / PResTimeFirst and PResTimeSecond invalid
                            // AmiSetDwordToLe(&pTxFrame->m_Data.m_Asnd.m_Payload.m_SyncResponse.m_le_dwSyncStatus, 0);
                            // init SyncNodeNumber
                            // AmiSetDwordToLe(&pTxFrame->m_Data.m_Asnd.m_Payload.m_SyncResponse.m_le_dwSyncNodeNumber, 0);
                            // init SyncDelay
                            // AmiSetDwordToLe(&pTxFrame->m_Data.m_Asnd.m_Payload.m_SyncResponse.m_le_dwSyncDelay, 0);
                            break;
                        }
#endif

                        default:
                        {
                            break;
                        }
                    }
                    // ASnd Service ID
                    AmiSetByteToLe(&pTxFrame->m_Data.m_Asnd.m_le_bServiceId, ServiceId_p);
                    break;
                }

                case kEplMsgTypePres:
                {
                    // destination MAC address
                    AmiSetQword48ToBe(&pTxFrame->m_be_abDstMac[0], EPL_C_DLL_MULTICAST_PRES);
                    // destination node ID
                    AmiSetByteToLe(&pTxFrame->m_le_bDstNodeId, (BYTE) EPL_C_ADR_BROADCAST);
                    break;
                }

#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)
                case kEplMsgTypeSoc:
                {
                    // destination MAC address
                    AmiSetQword48ToBe(&pTxFrame->m_be_abDstMac[0], EPL_C_DLL_MULTICAST_SOC);
                    // destination node ID
                    AmiSetByteToLe(&pTxFrame->m_le_bDstNodeId, (BYTE) EPL_C_ADR_BROADCAST);
                    break;
                }

                case kEplMsgTypeSoa:
                {
                    // destination MAC address
                    AmiSetQword48ToBe(&pTxFrame->m_be_abDstMac[0], EPL_C_DLL_MULTICAST_SOA);
                    // destination node ID
                    AmiSetByteToLe(&pTxFrame->m_le_bDstNodeId, (BYTE) EPL_C_ADR_BROADCAST);
                    // EPL profile version
                    AmiSetByteToLe(&pTxFrame->m_Data.m_Soa.m_le_bEplVersion, (BYTE) EPL_SPEC_VERSION);
                    break;
                }

                case kEplMsgTypePreq:
                {
                    break;
                }
#endif

                default:
                {
                    break;
                }
            }
            // EPL message type
            AmiSetByteToLe(&pTxFrame->m_le_bMessageType, (BYTE) MsgType_p);
        }
    }

    *puiFrameSize_p = pTxBuffer->m_uiMaxBufferLen;

Exit:
    return Ret;
}

//---------------------------------------------------------------------------
//
// Function:    EplDllkDeleteTxFrame
//
// Description: deletes the buffer for a Tx frame and frees it in the
//              ethernet driver
//
// Parameters:  uiHandle_p              = IN: handle to frame buffer
//
// Returns:     tEplKernel              = error code
//
//
// State:
//
//---------------------------------------------------------------------------

static tEplKernel EplDllkDeleteTxFrame (unsigned int uiHandle_p)
{
tEplKernel      Ret = kEplSuccessful;
tEdrvTxBuffer*  pTxBuffer = NULL;
unsigned int    nIndex = 0;

    if (uiHandle_p >= EplDllkInstance_g.m_uiMaxTxFrames)
    {   // handle is not valid
        Ret = kEplDllIllegalHdl;
        goto Exit;
    }

    for ( ; nIndex < 2; nIndex++, uiHandle_p++)
    {
        pTxBuffer = &EplDllkInstance_g.m_pTxBuffer[uiHandle_p];

        // mark buffer as free so that frame will not be send in future anymore
        // $$$ d.k. What's up with running transmissions?
        pTxBuffer->m_uiTxMsgLen = EPL_DLLK_BUFLEN_EMPTY;

        // delete Tx buffer
        Ret = EdrvReleaseTxMsgBuffer(pTxBuffer);
        if (Ret != kEplSuccessful)
        {   // error occurred while releasing Tx frame
            goto Exit;
        }

        pTxBuffer->m_pbBuffer = NULL;
    }
Exit:
    return Ret;
}


//---------------------------------------------------------------------------
//
// Function:    EplDllkCheckFrame()
//
// Description: check frame and set missing information
//
// Parameters:  pFrame_p                = ethernet frame
//              uiFrameSize_p           = size of frame
//
// Returns:     tEplKernel              = error code
//
//
// State:
//
//---------------------------------------------------------------------------

static tEplKernel EplDllkCheckFrame(tEplFrame * pFrame_p, unsigned int uiFrameSize_p)
{
tEplMsgType     MsgType;
WORD            wEtherType;

    UNUSED_PARAMETER(uiFrameSize_p);

    // check frame
    if (pFrame_p != NULL)
    {
        // check SrcMAC
        if (AmiGetQword48FromBe(pFrame_p->m_be_abSrcMac) == 0)
        {
            // source MAC address
            EPL_MEMCPY(&pFrame_p->m_be_abSrcMac[0], &EplDllkInstance_g.m_be_abLocalMac[0], 6);
        }

        // check ethertype
        wEtherType = AmiGetWordFromBe(&pFrame_p->m_be_wEtherType);
        if (wEtherType == 0)
        {
            // assume EPL frame
            wEtherType = EPL_C_DLL_ETHERTYPE_EPL;
            AmiSetWordToBe(&pFrame_p->m_be_wEtherType, wEtherType);
        }

        if (wEtherType == EPL_C_DLL_ETHERTYPE_EPL)
        {
            // source node ID
            AmiSetByteToLe(&pFrame_p->m_le_bSrcNodeId, (BYTE) EplDllkInstance_g.m_DllConfigParam.m_uiNodeId);

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

//---------------------------------------------------------------------------
//
// Function:    EplDllkCbCnTimer()
//
// Description: called by timer module. It monitors the EPL cycle when it is a CN.
//
// Parameters:  pEventArg_p             = timer event argument
//
// Returns:     tEplKernel              = error code
//
//
// State:
//
//---------------------------------------------------------------------------

#if EPL_TIMER_USE_HIGHRES != FALSE
static tEplKernel PUBLIC EplDllkCbCnTimer(tEplTimerEventArg* pEventArg_p)
{
tEplKernel      Ret = kEplSuccessful;
tEplNmtState    NmtState;

TGT_DLLK_DECLARE_FLAGS

    TGT_DLLK_ENTER_CRITICAL_SECTION();

    if (pEventArg_p->m_TimerHdl != EplDllkInstance_g.m_TimerHdlCycle)
    {   // zombie callback
        // just exit
        goto Exit;
    }

    NmtState = EplDllkInstance_g.m_NmtState;

    if (NmtState <= kEplNmtGsResetConfiguration)
    {
        goto Exit;
    }

    Ret = EplDllkChangeState(kEplNmtEventDllCeFrameTimeout, NmtState);
    if (Ret != kEplSuccessful)
    {
        goto Exit;
    }

    // restart the timer to detect further loss of SoC
    Ret = EplTimerHighReskModifyTimerNs(&EplDllkInstance_g.m_TimerHdlCycle, EplDllkInstance_g.m_DllConfigParam.m_dwCycleLen, EplDllkCbCnTimer, 0L, FALSE);
    if (Ret != kEplSuccessful)
    {
        goto Exit;
    }

Exit:
    if (Ret != kEplSuccessful)
    {
    DWORD   dwArg;

        BENCHMARK_MOD_02_TOGGLE(7);

        dwArg = EplDllkInstance_g.m_DllState | (kEplNmtEventDllCeFrameTimeout << 8);

        // Error event for API layer
        Ret = EplEventkPostError(kEplEventSourceDllk,
                        Ret,
                        sizeof(dwArg),
                        &dwArg);
    }

    TGT_DLLK_LEAVE_CRITICAL_SECTION();

    return Ret;
}
#endif


#if (EPL_DLL_PROCESS_SYNC == EPL_DLL_PROCESS_SYNC_ON_TIMER)
//---------------------------------------------------------------------------
//
// Function:    EplDllkCbCnTimerSync()
//
// Description: called by timer sync module. It signals the Sync event.
//
// Parameters:  void
//
// Returns:     tEplKernel              = error code
//
//
// State:
//
//---------------------------------------------------------------------------

static tEplKernel PUBLIC EplDllkCbCnTimerSync(void)
{
tEplKernel      Ret = kEplSuccessful;

    // trigger synchronous task
    Ret = EplDllkPostEvent(kEplEventTypeSync);

    return Ret;
}


//---------------------------------------------------------------------------
//
// Function:    EplDllkCbCnLossOfSync()
//
// Description: called by timer sync module. It signals that one Sync/SoC was lost.
//
// Parameters:  void
//
// Returns:     tEplKernel              = error code
//
//
// State:
//
//---------------------------------------------------------------------------

static tEplKernel PUBLIC EplDllkCbCnLossOfSync(void)
{
tEplKernel      Ret = kEplSuccessful;
tEplNmtState    NmtState;

TGT_DLLK_DECLARE_FLAGS

    TGT_DLLK_ENTER_CRITICAL_SECTION();

    NmtState = EplDllkInstance_g.m_NmtState;

    if (NmtState <= kEplNmtGsResetConfiguration)
    {
        goto Exit;
    }

    Ret = EplDllkChangeState(kEplNmtEventDllCeFrameTimeout, NmtState);
    if (Ret != kEplSuccessful)
    {
        goto Exit;
    }

Exit:
    if (Ret != kEplSuccessful)
    {
    DWORD   dwArg;

        BENCHMARK_MOD_02_TOGGLE(7);

        dwArg = EplDllkInstance_g.m_DllState | (kEplNmtEventDllCeFrameTimeout << 8);

        // Error event for API layer
        Ret = EplEventkPostError(kEplEventSourceDllk,
                        Ret,
                        sizeof(dwArg),
                        &dwArg);
    }

    TGT_DLLK_LEAVE_CRITICAL_SECTION();

    return Ret;
}
#endif


#if EPL_NMT_MAX_NODE_ID > 0

//---------------------------------------------------------------------------
//
// Function:    EplDllkGetNodeInfo()
//
// Description: returns node info structure of the specified node.
//
// Parameters:  uiNodeId_p              = node ID
//
// Returns:     tEplDllkNodeInfo*       = pointer to internal node info structure
//
//
// State:
//
//---------------------------------------------------------------------------

static tEplDllkNodeInfo* EplDllkGetNodeInfo(unsigned int uiNodeId_p)
{
    // $$$ d.k.: use hash algorithm to retrieve the appropriate node info structure
    //           if size of array is less than 254.
    uiNodeId_p--;   // node ID starts at 1 but array at 0
    if (uiNodeId_p >= tabentries (EplDllkInstance_g.m_aNodeInfo))
    {
        return NULL;
    }
    else
    {
        return &EplDllkInstance_g.m_aNodeInfo[uiNodeId_p];
    }
}


//---------------------------------------------------------------------------
//
// Function:    EplDllkAddNodeFilter()
//
// Description: adds PRes filter for the specified node.
//
// Parameters:  pIntNodeInfo_p          = pointer of internal node info structure
//              NodeOpType_p            = type of PRes filter
//
// Returns:     tEplKernel              = error code
//
//
// State:
//
//---------------------------------------------------------------------------

static tEplKernel EplDllkAddNodeFilter(tEplDllkNodeInfo* pIntNodeInfo_p, tEplDllNodeOpType NodeOpType_p, BOOL fUpdateEdrv_p)
{
tEplKernel      Ret = kEplSuccessful;
BYTE            bPresFilterFlags = 0;

    switch (NodeOpType_p)
    {
        case kEplDllNodeOpTypeFilterPdo:
        {
            bPresFilterFlags = EPL_DLLK_FILTER_FLAG_PDO;
            break;
        }

        case kEplDllNodeOpTypeFilterHeartbeat:
        {
            bPresFilterFlags = EPL_DLLK_FILTER_FLAG_HB;
            break;
        }

        default:
        {
            Ret = kEplDllInvalidParam;
            goto Exit;
        }
    }

    if (fUpdateEdrv_p != FALSE)
    {
        if ((pIntNodeInfo_p->m_bPresFilterFlags & (EPL_DLLK_FILTER_FLAG_PDO | EPL_DLLK_FILTER_FLAG_HB)) == 0)
        {
#if EPL_DLL_PRES_FILTER_COUNT < 0
            EplDllkInstance_g.m_uiUsedPresFilterCount++;
            if (EplDllkInstance_g.m_uiUsedPresFilterCount == 1)
            {
                // enable PRes Rx filter
                EplDllkInstance_g.m_aFilter[EPL_DLLK_FILTER_PRES].m_fEnable = TRUE;
                Ret = EdrvChangeFilter(EplDllkInstance_g.m_aFilter,
                                       EPL_DLLK_FILTER_COUNT,
                                       EPL_DLLK_FILTER_PRES,
                                       EDRV_FILTER_CHANGE_STATE);
                if (Ret != kEplSuccessful)
                {
                    goto Exit;
                }
            }

#else
        unsigned int    uiHandle;

            for (uiHandle = EPL_DLLK_FILTER_PRES; uiHandle < EPL_DLLK_FILTER_COUNT; uiHandle++)
            {
                if (AmiGetByteFromLe(&EplDllkInstance_g.m_aFilter[uiHandle].m_abFilterValue[16]) == EPL_C_ADR_INVALID)
                {
                    AmiSetByteToBe(&EplDllkInstance_g.m_aFilter[uiHandle].m_abFilterValue[16],
                                   pIntNodeInfo_p->m_uiNodeId);
                    EplDllkInstance_g.m_aFilter[uiHandle].m_fEnable = TRUE;

                    Ret = EdrvChangeFilter(EplDllkInstance_g.m_aFilter,
                                           EPL_DLLK_FILTER_COUNT,
                                           uiHandle,
                                           (EDRV_FILTER_CHANGE_STATE | EDRV_FILTER_CHANGE_VALUE));
                    if (Ret != kEplSuccessful)
                    {
                        goto Exit;
                    }
                    break;
                }
            }
#endif
        }

    }
    pIntNodeInfo_p->m_bPresFilterFlags |= bPresFilterFlags;

Exit:
    return Ret;
}


//---------------------------------------------------------------------------
//
// Function:    EplDllkDeleteNodeFilter()
//
// Description: deletes PRes filter for the specified node.
//
// Parameters:  pIntNodeInfo_p          = pointer of internal node info structure
//              NodeOpType_p            = type of PRes filter
//
// Returns:     tEplKernel              = error code
//
//
// State:
//
//---------------------------------------------------------------------------

static tEplKernel EplDllkDeleteNodeFilter(tEplDllkNodeInfo* pIntNodeInfo_p, tEplDllNodeOpType NodeOpType_p, BOOL fUpdateEdrv_p)
{
tEplKernel      Ret = kEplSuccessful;
BYTE            bPresFilterFlags = 0;

    switch (NodeOpType_p)
    {
        case kEplDllNodeOpTypeFilterPdo:
        {
            bPresFilterFlags = EPL_DLLK_FILTER_FLAG_PDO;
            break;
        }

        case kEplDllNodeOpTypeFilterHeartbeat:
        {
            bPresFilterFlags = EPL_DLLK_FILTER_FLAG_HB;
            break;
        }

        default:
        {
            Ret = kEplDllInvalidParam;
            goto Exit;
        }
    }

    pIntNodeInfo_p->m_bPresFilterFlags &= ~bPresFilterFlags;

    if (fUpdateEdrv_p != FALSE)
    {
        if ((pIntNodeInfo_p->m_bPresFilterFlags & (EPL_DLLK_FILTER_FLAG_PDO | EPL_DLLK_FILTER_FLAG_HB)) == 0)
        {
#if EPL_DLL_PRES_FILTER_COUNT < 0
            if (EplDllkInstance_g.m_uiUsedPresFilterCount > 0)
            {
                EplDllkInstance_g.m_uiUsedPresFilterCount--;
            }
            if (EplDllkInstance_g.m_uiUsedPresFilterCount == 0)
            {
                // disable PRes Rx filter
                EplDllkInstance_g.m_aFilter[EPL_DLLK_FILTER_PRES].m_fEnable = FALSE;
                Ret = EdrvChangeFilter(EplDllkInstance_g.m_aFilter,
                                       EPL_DLLK_FILTER_COUNT,
                                       EPL_DLLK_FILTER_PRES,
                                       EDRV_FILTER_CHANGE_STATE);
                if (Ret != kEplSuccessful)
                {
                    goto Exit;
                }
            }

#else
        unsigned int    uiHandle;

            for (uiHandle = EPL_DLLK_FILTER_PRES; uiHandle < EPL_DLLK_FILTER_COUNT; uiHandle++)
            {
                if (AmiGetByteFromLe(&EplDllkInstance_g.m_aFilter[uiHandle].m_abFilterValue[16]) == pIntNodeInfo_p->m_uiNodeId)
                {
                    AmiSetByteToBe(&EplDllkInstance_g.m_aFilter[uiHandle].m_abFilterValue[16],
                                   EPL_C_ADR_INVALID);
                    EplDllkInstance_g.m_aFilter[uiHandle].m_fEnable = FALSE;

                    Ret = EdrvChangeFilter(EplDllkInstance_g.m_aFilter,
                                           EPL_DLLK_FILTER_COUNT,
                                           uiHandle,
                                           EDRV_FILTER_CHANGE_STATE);
                    if (Ret != kEplSuccessful)
                    {
                        goto Exit;
                    }
                    break;
                }
            }
#endif
        }
    }

Exit:
    return Ret;
}


#endif // EPL_NMT_MAX_NODE_ID > 0


#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)

//---------------------------------------------------------------------------
//
// Function:    EplDllkAddNodeIsochronous()
//
// Description: adds the specified node to the isochronous phase.
//
// Parameters:  pNodeInfo_p             = pointer of node info structure
//
// Returns:     tEplKernel              = error code
//
//
// State:
//
//---------------------------------------------------------------------------

static tEplKernel EplDllkAddNodeIsochronous(tEplDllkNodeInfo* pIntNodeInfo_p)
{
tEplKernel          Ret = kEplSuccessful;
tEplDllkNodeInfo**  ppIntNodeInfo;
tEplFrame*          pTxFrame;

    if (pIntNodeInfo_p->m_uiNodeId == EplDllkInstance_g.m_DllConfigParam.m_uiNodeId)
    {   // we shall send PRes ourself
        // insert our node as first entry in the list
        ppIntNodeInfo = &EplDllkInstance_g.m_pFirstNodeInfo;
#if EPL_DLL_DISABLE_EDRV_CYCLIC != FALSE
        // exception when EdrvCyclic is disabled: add PResMN at end of isochronous phase
        while ((*ppIntNodeInfo != NULL) && ((*ppIntNodeInfo)->m_pNextNodeInfo != NULL))
        {
            ppIntNodeInfo = &(*ppIntNodeInfo)->m_pNextNodeInfo;
        }
#endif
        if (*ppIntNodeInfo != NULL)
        {
            if ((*ppIntNodeInfo)->m_uiNodeId == pIntNodeInfo_p->m_uiNodeId)
            {   // node was already added to list
                // $$$ d.k. maybe this should be an error
                goto Exit;
            }
#if EPL_DLL_DISABLE_EDRV_CYCLIC != FALSE
            else
            {   // add our node at the end of the list
                ppIntNodeInfo = &(*ppIntNodeInfo)->m_pNextNodeInfo;
            }
#endif
        }
        // set "PReq"-TxBuffer to PRes-TxBuffer
        pIntNodeInfo_p->m_pPreqTxBuffer = &EplDllkInstance_g.m_pTxBuffer[EPL_DLLK_TXFRAME_PRES];

        // Reset PRC Slot Timeout
        // which is required if falling back to PreOp1
        pIntNodeInfo_p->m_dwPresTimeoutNs = 0;
    }
    else
    {   // normal CN shall be added to isochronous phase
        // insert node into list in ascending order
#if EPL_DLL_PRES_CHAINING_MN != FALSE
        if (pIntNodeInfo_p->m_pPreqTxBuffer == NULL)
        {
            ppIntNodeInfo = &EplDllkInstance_g.m_pFirstPrcNodeInfo;
        }
        else
#endif
        {
            ppIntNodeInfo = &EplDllkInstance_g.m_pFirstNodeInfo;
        }
        while ((*ppIntNodeInfo != NULL)
               && (((*ppIntNodeInfo)->m_uiNodeId < pIntNodeInfo_p->m_uiNodeId)
#if EPL_DLL_DISABLE_EDRV_CYCLIC != FALSE
               && ((*ppIntNodeInfo)->m_uiNodeId != EplDllkInstance_g.m_DllConfigParam.m_uiNodeId)))
#else
                   || ((*ppIntNodeInfo)->m_uiNodeId == EplDllkInstance_g.m_DllConfigParam.m_uiNodeId)))
#endif
        {
            ppIntNodeInfo = &(*ppIntNodeInfo)->m_pNextNodeInfo;
        }
        if ((*ppIntNodeInfo != NULL) && ((*ppIntNodeInfo)->m_uiNodeId == pIntNodeInfo_p->m_uiNodeId))
        {   // node was already added to list
            // $$$ d.k. maybe this should be an error
            goto Exit;
        }

        if (pIntNodeInfo_p->m_pPreqTxBuffer != NULL)
        {   // TxBuffer entry exists
        tEplEvent       Event;

            pTxFrame = (tEplFrame *) pIntNodeInfo_p->m_pPreqTxBuffer[0].m_pbBuffer;

            // set up destination MAC address
            EPL_MEMCPY(pTxFrame->m_be_abDstMac, pIntNodeInfo_p->m_be_abMacAddr, 6);

            // set destination node-ID in PReq
            AmiSetByteToLe(&pTxFrame->m_le_bDstNodeId, (BYTE) pIntNodeInfo_p->m_uiNodeId);

            // do the same for second frame buffer
            pTxFrame = (tEplFrame *) pIntNodeInfo_p->m_pPreqTxBuffer[1].m_pbBuffer;

            // set up destination MAC address
            EPL_MEMCPY(pTxFrame->m_be_abDstMac, pIntNodeInfo_p->m_be_abMacAddr, 6);

            // set destination node-ID in PReq
            AmiSetByteToLe(&pTxFrame->m_le_bDstNodeId, (BYTE) pIntNodeInfo_p->m_uiNodeId);

            Event.m_EventSink = kEplEventSinkNmtMnu;
            Event.m_EventType = kEplEventTypeNmtMnuNodeAdded;
            Event.m_uiSize = sizeof (pIntNodeInfo_p->m_uiNodeId);
            Event.m_pArg = &pIntNodeInfo_p->m_uiNodeId;
            Ret = EplEventkPost(&Event);
            if (Ret != kEplSuccessful)
            {
                goto Exit;
            }

        }
#if EPL_DLL_PRES_CHAINING_MN == FALSE
        else
        {   // TxBuffer for PReq does not exist
            Ret = kEplDllTxFrameInvalid;
            goto Exit;
        }
#endif

        Ret = EplErrorHandlerkResetCnError(pIntNodeInfo_p->m_uiNodeId);
    }

    // initialize elements of internal node info structure
    pIntNodeInfo_p->m_bSoaFlag1 = 0;
    pIntNodeInfo_p->m_fSoftDelete = FALSE;
    pIntNodeInfo_p->m_NmtState = kEplNmtCsNotActive;
    pIntNodeInfo_p->m_ulDllErrorEvents = 0L;
    // add node to list
    pIntNodeInfo_p->m_pNextNodeInfo = *ppIntNodeInfo;
    *ppIntNodeInfo = pIntNodeInfo_p;

Exit:
    return Ret;
}


//---------------------------------------------------------------------------
//
// Function:    EplDllkDeleteNodeIsochronous()
//
// Description: removes the specified node from the isochronous phase.
//
// Parameters:  uiNodeId_p              = node ID
//
// Returns:     tEplKernel              = error code
//
//
// State:
//
//---------------------------------------------------------------------------

static tEplKernel EplDllkDeleteNodeIsochronous(tEplDllkNodeInfo* pIntNodeInfo_p)
{
tEplKernel          Ret = kEplSuccessful;
tEplDllkNodeInfo**  ppIntNodeInfo;
tEplFrame*          pTxFrame;

#if EPL_DLL_PRES_CHAINING_MN != FALSE
    if (pIntNodeInfo_p->m_pPreqTxBuffer == NULL)
    {
        ppIntNodeInfo = &EplDllkInstance_g.m_pFirstPrcNodeInfo;
    }
    else
#endif
    {
        ppIntNodeInfo = &EplDllkInstance_g.m_pFirstNodeInfo;
    }
    // search node in whole list
    while ((*ppIntNodeInfo != NULL) && (*ppIntNodeInfo != pIntNodeInfo_p))
    {
        ppIntNodeInfo = &(*ppIntNodeInfo)->m_pNextNodeInfo;
    }
    if ((*ppIntNodeInfo == NULL) || (*ppIntNodeInfo != pIntNodeInfo_p))
    {   // node was not found in list
        // $$$ d.k. maybe this should be an error
        goto Exit;
    }

    // remove node from list
    *ppIntNodeInfo = pIntNodeInfo_p->m_pNextNodeInfo;

    if (pIntNodeInfo_p->m_pPreqTxBuffer != NULL)
    {   // disable TPDO
        pTxFrame = (tEplFrame *) pIntNodeInfo_p->m_pPreqTxBuffer[0].m_pbBuffer;
        if (pTxFrame != NULL)
        {   // frame does exist
            // update frame (disable RD in Flag1)
            AmiSetByteToLe(&pTxFrame->m_Data.m_Preq.m_le_bFlag1, 0);
        }

        pTxFrame = (tEplFrame *) pIntNodeInfo_p->m_pPreqTxBuffer[1].m_pbBuffer;
        if (pTxFrame != NULL)
        {   // frame does exist
            // update frame (disable RD in Flag1)
            AmiSetByteToLe(&pTxFrame->m_Data.m_Preq.m_le_bFlag1, 0);
        }
    }

Exit:
    return Ret;
}


//---------------------------------------------------------------------------
//
// Function:    EplDllkCbMnTimerCycle()
//
// Description: called by timer module. It triggers the SoC when it is a MN.
//
// Parameters:  pEventArg_p             = timer event argument
//
// Returns:     tEplKernel              = error code
//
//
// State:
//
//---------------------------------------------------------------------------

static tEplKernel PUBLIC EplDllkCbMnTimerCycle(tEplTimerEventArg* pEventArg_p)
{
tEplKernel      Ret = kEplSuccessful;
tEplNmtState    NmtState;

TGT_DLLK_DECLARE_FLAGS

    TGT_DLLK_ENTER_CRITICAL_SECTION();

#if EPL_TIMER_USE_HIGHRES != FALSE
    if (pEventArg_p->m_TimerHdl != EplDllkInstance_g.m_TimerHdlCycle)
    {   // zombie callback
        // just exit
        goto Exit;
    }
#endif

    NmtState = EplDllkInstance_g.m_NmtState;

    if (NmtState <= kEplNmtGsResetConfiguration)
    {
        goto Exit;
    }

    Ret = EplDllkChangeState(kEplNmtEventDllMeSocTrig, NmtState);

Exit:
    if (Ret != kEplSuccessful)
    {
    DWORD   dwArg;

        BENCHMARK_MOD_02_TOGGLE(7);

        dwArg = EplDllkInstance_g.m_DllState | (kEplNmtEventDllMeSocTrig << 8);

        // Error event for API layer
        Ret = EplEventkPostError(kEplEventSourceDllk,
                        Ret,
                        sizeof(dwArg),
                        &dwArg);
    }

    TGT_DLLK_LEAVE_CRITICAL_SECTION();

    return Ret;
}


//---------------------------------------------------------------------------
//
// Function:    EplDllkCbMnSyncHandler()
//
// Description: called by EdrvCyclic module. It triggers the Sync event.
//
// Parameters:  pEventArg_p             = timer event argument
//
// Returns:     tEplKernel              = error code
//
//
// State:
//
//---------------------------------------------------------------------------

#if EPL_DLL_DISABLE_EDRV_CYCLIC == FALSE
static tEplKernel EplDllkCbMnSyncHandler(void)
{
tEplKernel      Ret = kEplSuccessful;
tEplNmtState    NmtState;
BYTE*           pbCnNodeId;

TGT_DLLK_DECLARE_FLAGS

    TGT_DLLK_ENTER_CRITICAL_SECTION();

    NmtState = EplDllkInstance_g.m_NmtState;

    if (NmtState <= kEplNmtGsResetConfiguration)
    {
        goto Exit;
    }

    // do cycle finish which has to be done inside the callback function triggered by interrupt
    pbCnNodeId = &EplDllkInstance_g.m_aabCnNodeIdList[EplDllkInstance_g.m_bCurTxBufferOffsetCycle][EplDllkInstance_g.m_bCurNodeIndex];

    while (*pbCnNodeId != EPL_C_ADR_INVALID)
    {   // issue error for each CN in list which was not processed yet, i.e. PRes received

        Ret = EplDllkIssueLossOfPres(*pbCnNodeId);
        if (Ret != kEplSuccessful)
        {
            goto Exit;
        }

        pbCnNodeId++;
    }

    EplDllkInstance_g.m_fSyncProcessed = FALSE;
#if EPL_DLL_PRES_CHAINING_MN != FALSE
    EplDllkInstance_g.m_fPrcSlotFinished = FALSE;
#endif

    // switch to next cycle
    EplDllkInstance_g.m_bCurTxBufferOffsetCycle ^= 1;
    EplDllkInstance_g.m_bCurNodeIndex = 0;

    Ret = EplDllkPostEvent(kEplEventTypeDllkCycleFinish);

Exit:
    if (Ret != kEplSuccessful)
    {
    DWORD   dwArg;

        BENCHMARK_MOD_02_TOGGLE(7);

        dwArg = EplDllkInstance_g.m_DllState | (kEplNmtEventDllMeSocTrig << 8);

        // Error event for API layer
        Ret = EplEventkPostError(kEplEventSourceDllk,
                        Ret,
                        sizeof(dwArg),
                        &dwArg);
    }

    TGT_DLLK_LEAVE_CRITICAL_SECTION();

    return Ret;
}
#endif


//---------------------------------------------------------------------------
//
// Function:    EplDllkCbMnTimerResponse()
//
// Description: called by timer module. It monitors the PRes timeout.
//
// Parameters:  pEventArg_p             = timer event argument
//
// Returns:     tEplKernel              = error code
//
//
// State:
//
//---------------------------------------------------------------------------
#if EPL_DLL_DISABLE_EDRV_CYCLIC != FALSE
static tEplKernel PUBLIC EplDllkCbMnTimerResponse(tEplTimerEventArg* pEventArg_p)
{
tEplKernel      Ret = kEplSuccessful;
tEplNmtState    NmtState;

TGT_DLLK_DECLARE_FLAGS

    TGT_DLLK_ENTER_CRITICAL_SECTION();

#if EPL_TIMER_USE_HIGHRES != FALSE
    if (pEventArg_p->m_TimerHdl != EplDllkInstance_g.m_TimerHdlResponse)
    {   // zombie callback
        // just exit
        goto Exit;
    }
#endif

    NmtState = EplDllkInstance_g.m_NmtState;

    if (NmtState <= kEplNmtGsResetConfiguration)
    {
        goto Exit;
    }

    Ret = EplDllkChangeState(kEplNmtEventDllMePresTimeout, NmtState);

Exit:
    if (Ret != kEplSuccessful)
    {
    DWORD   dwArg;

        BENCHMARK_MOD_02_TOGGLE(7);

        dwArg = EplDllkInstance_g.m_DllState | (kEplNmtEventDllMePresTimeout << 8);

        // Error event for API layer
        Ret = EplEventkPostError(kEplEventSourceDllk,
                        Ret,
                        sizeof(dwArg),
                        &dwArg);
    }

    TGT_DLLK_LEAVE_CRITICAL_SECTION();

    return Ret;
}
#endif

//---------------------------------------------------------------------------
//
// Function:    EplDllkUpdateFrameSoa()
//
// Description: update SoA frame
//
// Parameters:  pTxBuffer_p             = Tx buffer of StatusRes
//              NmtState_p              = current NMT state
//              fEnableInvitation_p     = enable invitation for asynchronous phase
//                                        it will be disabled for the first EPL_C_DLL_PREOP1_START_CYCLES SoAs
//
// Returns:     tEplKernel              = error code
//
//
// State:
//
//---------------------------------------------------------------------------

static tEplKernel EplDllkUpdateFrameSoa(tEdrvTxBuffer* pTxBuffer_p, tEplNmtState NmtState_p, BOOL fEnableInvitation_p, BYTE bCurReq_p)
{
tEplKernel          Ret = kEplSuccessful;
tEplFrame*          pTxFrame;
tEplDllkNodeInfo*   pNodeInfo;

    pTxFrame = (tEplFrame *) pTxBuffer_p->m_pbBuffer;

    if (fEnableInvitation_p != FALSE)
    {   // fetch target of asynchronous phase
        if (EplDllkInstance_g.m_bFlag2 == 0)
        {   // own queues are empty
            EplDllkInstance_g.m_aLastReqServiceId[bCurReq_p] = kEplDllReqServiceNo;
        }
        else if (((tEplDllAsyncReqPriority) (EplDllkInstance_g.m_bFlag2 >> EPL_FRAME_FLAG2_PR_SHIFT)) == kEplDllAsyncReqPrioNmt)
        {   // frames in own NMT request queue available
            EplDllkInstance_g.m_aLastReqServiceId[bCurReq_p] = kEplDllReqServiceNmtRequest;
        }
        else
        {
            EplDllkInstance_g.m_aLastReqServiceId[bCurReq_p] = kEplDllReqServiceUnspecified;
        }
        Ret = EplDllkCalAsyncGetSoaRequest(&EplDllkInstance_g.m_aLastReqServiceId[bCurReq_p],
                                           &EplDllkInstance_g.m_auiLastTargetNodeId[bCurReq_p],
                                           &pTxFrame->m_Data.m_Soa.m_Payload);
        if (Ret != kEplSuccessful)
        {
            goto Exit;
        }
        if (EplDllkInstance_g.m_aLastReqServiceId[bCurReq_p] != kEplDllReqServiceNo)
        {   // asynchronous phase will be assigned to one node
            if (EplDllkInstance_g.m_auiLastTargetNodeId[bCurReq_p] == EPL_C_ADR_INVALID)
            {   // exchange invalid node ID with local node ID
                EplDllkInstance_g.m_auiLastTargetNodeId[bCurReq_p] = EplDllkInstance_g.m_DllConfigParam.m_uiNodeId;
            }

            pNodeInfo = EplDllkGetNodeInfo(EplDllkInstance_g.m_auiLastTargetNodeId[bCurReq_p]);
            if (pNodeInfo == NULL)
            {   // no node info structure available
                Ret = kEplDllNoNodeInfo;
                goto Exit;
            }

            // update frame (EA, ER flags)
            AmiSetByteToLe(&pTxFrame->m_Data.m_Soa.m_le_bFlag1,
                pNodeInfo->m_bSoaFlag1 & (EPL_FRAME_FLAG1_EA | EPL_FRAME_FLAG1_ER));
        }
        else
        {   // no assignment of asynchronous phase
            EplDllkInstance_g.m_auiLastTargetNodeId[bCurReq_p] = EPL_C_ADR_INVALID;
        }
    }
    else
    {   // invite nobody
        EplDllkInstance_g.m_aLastReqServiceId[bCurReq_p] = kEplDllReqServiceNo;
        EplDllkInstance_g.m_auiLastTargetNodeId[bCurReq_p] = EPL_C_ADR_INVALID;
    }

    // update frame (target)
    AmiSetByteToLe(&pTxFrame->m_Data.m_Soa.m_le_bReqServiceId, (BYTE) EplDllkInstance_g.m_aLastReqServiceId[bCurReq_p]);
    AmiSetByteToLe(&pTxFrame->m_Data.m_Soa.m_le_bReqServiceTarget, (BYTE) EplDllkInstance_g.m_auiLastTargetNodeId[bCurReq_p]);

    // update frame (NMT state)
    AmiSetByteToLe(&pTxFrame->m_Data.m_Soa.m_le_bNmtStatus, (BYTE) NmtState_p);

Exit:
    return Ret;
}


//---------------------------------------------------------------------------
//
// Function:    EplDllkMnSendSoa()
//
// Description: it updates and transmits the SoA.
//
// Parameters:  NmtState_p              = current NMT state
//              pDllStateProposed_p     = proposed DLL state
//              fEnableInvitation_p     = enable invitation for asynchronous phase
//                                        it will be disabled for EPL_C_DLL_PREOP1_START_CYCLES SoAs
//
// Returns:     tEplKernel              = error code
//
//
// State:
//
//---------------------------------------------------------------------------

static tEplKernel EplDllkMnSendSoa(tEplNmtState NmtState_p,
                                   tEplDllState* pDllStateProposed_p,
                                   BOOL fEnableInvitation_p)
{
tEplKernel      Ret = kEplSuccessful;
tEdrvTxBuffer  *pTxBuffer = NULL;
tEplFrame      *pTxFrame;

    *pDllStateProposed_p = kEplDllMsNonCyclic;

    pTxBuffer = &EplDllkInstance_g.m_pTxBuffer[EPL_DLLK_TXFRAME_SOA];
    if (pTxBuffer->m_pbBuffer != NULL)
    {   // SoA does exist
        pTxFrame = (tEplFrame *) pTxBuffer->m_pbBuffer;

        Ret = EplDllkUpdateFrameSoa(pTxBuffer, NmtState_p, fEnableInvitation_p, EplDllkInstance_g.m_bCurLastSoaReq);
        if (Ret != kEplSuccessful)
        {
            goto Exit;
        }

        if (EplDllkInstance_g.m_aLastReqServiceId[EplDllkInstance_g.m_bCurLastSoaReq] != kEplDllReqServiceNo)
        {   // asynchronous phase will be assigned to one node
            if (EplDllkInstance_g.m_auiLastTargetNodeId[EplDllkInstance_g.m_bCurLastSoaReq] == EplDllkInstance_g.m_DllConfigParam.m_uiNodeId)
            {   // d.k. DLL state WaitAsndTrig is not helpful;
                //      so just step over to WaitSocTrig,
                //      because own ASnd is sent automatically in CbFrameTransmitted() after SoA.
                //*pDllStateProposed_p = kEplDllMsWaitAsndTrig;
                *pDllStateProposed_p = kEplDllMsWaitSocTrig;
            }
            else
            {   // assignment to CN
                *pDllStateProposed_p = kEplDllMsWaitAsnd;
            }

        }
        else
        {   // no assignment of asynchronous phase
            *pDllStateProposed_p = kEplDllMsWaitSocTrig;
        }

        // send SoA frame
        Ret = EdrvSendTxMsg(pTxBuffer);
    }

Exit:
     return Ret;
}


//---------------------------------------------------------------------------
//
// Function:    EplDllkMnSendSoc()
//
// Description: it updates and transmits the SoA.
//
// Parameters:  (none)
//
// Returns:     tEplKernel              = error code
//
//
// State:
//
//---------------------------------------------------------------------------

#if EPL_DLL_DISABLE_EDRV_CYCLIC != FALSE
static tEplKernel EplDllkMnSendSoc(void)
{
tEplKernel      Ret = kEplSuccessful;
tEdrvTxBuffer*  pTxBuffer = NULL;
tEplFrame*      pTxFrame;

    pTxBuffer = &EplDllkInstance_g.m_pTxBuffer[EPL_DLLK_TXFRAME_SOC];
    if (pTxBuffer->m_pbBuffer != NULL)
    {   // SoC does exist
        pTxFrame = (tEplFrame *) pTxBuffer->m_pbBuffer;

        // $$$ update NetTime

        // send SoC frame
        Ret = EdrvSendTxMsg(pTxBuffer);
        if (Ret != kEplSuccessful)
        {
            goto Exit;
        }

        if (EplDllkInstance_g.m_DllConfigParam.m_uiSyncNodeId == EPL_C_ADR_SYNC_ON_SOC)
        {
            // trigger synchronous task on SoC
            Ret = EplDllkPostEvent(kEplEventTypeSync);
        }
    }

Exit:
     return Ret;
}
#endif


//---------------------------------------------------------------------------
//
// Function:    EplDllkMnSendPreq()
//
// Description: it updates and transmits the PReq for the next isochronous CN
//              or own PRes if enabled.
//
// Parameters:  NmtState_p              = current NMT state
//              pDllStateProposed_p     = proposed DLL state
//
// Returns:     tEplKernel              = error code
//
//
// State:
//
//---------------------------------------------------------------------------

#if EPL_DLL_DISABLE_EDRV_CYCLIC != FALSE
static tEplKernel EplDllkMnSendPreq(tEplNmtState NmtState_p,
                                    tEplDllState* pDllStateProposed_p)
{
tEplKernel      Ret = kEplSuccessful;
tEdrvTxBuffer  *pTxBuffer = NULL;

#if EPL_TIMER_USE_HIGHRES != FALSE
    Ret = EplTimerHighReskDeleteTimer(&EplDllkInstance_g.m_TimerHdlResponse);
#endif

    if (EplDllkInstance_g.m_pCurNodeInfo == NULL)
    {   // start with first isochronous CN
        EplDllkInstance_g.m_pCurNodeInfo = EplDllkInstance_g.m_pFirstNodeInfo;
    }
    else
    {   // iterate to next isochronous CN
        EplDllkInstance_g.m_pCurNodeInfo = EplDllkInstance_g.m_pCurNodeInfo->m_pNextNodeInfo;
    }

    if (EplDllkInstance_g.m_pCurNodeInfo == NULL)
    {   // last isochronous CN reached
        Ret = EplDllkMnSendSoa(NmtState_p, pDllStateProposed_p, TRUE);
        goto Exit;
    }
    else
    {
        pTxBuffer = &EplDllkInstance_g.m_pCurNodeInfo->m_pPreqTxBuffer[EplDllkInstance_g.m_bCurTxBufferOffsetCycle];
        *pDllStateProposed_p = kEplDllMsWaitPres;

        // start PRes Timer in CbFrameTransmitted()
    }

    if (pTxBuffer == NULL)
    {   // PReq does not exist
        Ret = kEplDllTxBufNotReady;
        goto Exit;
    }

    if (pTxBuffer == &EplDllkInstance_g.m_pTxBuffer[EPL_DLLK_TXFRAME_PRES + EplDllkInstance_g.m_bCurTxBufferOffsetCycle])
    {   // PRes of MN will be sent
        *pDllStateProposed_p = kEplDllMsWaitSoaTrig;
    }

    // send PReq frame
    Ret = EdrvSendTxMsg(pTxBuffer);

Exit:
     return Ret;
}
#endif


//---------------------------------------------------------------------------
//
// Function:    EplDllkAsyncFrameNotReceived()
//
// Description: passes empty ASnd frame to receive FIFO.
//              It will be called only for frames with registered AsndServiceIds
//              (only kEplDllAsndFilterAny).
//
// Parameters:  none
//
// Returns:     tEplKernel              = error code
//
//
// State:
//
//---------------------------------------------------------------------------

static tEplKernel EplDllkAsyncFrameNotReceived(tEplDllReqServiceId ReqServiceId_p, unsigned int uiNodeId_p)
{
tEplKernel      Ret = kEplSuccessful;
BYTE            abBuffer[18];
tEplFrame*      pFrame = (tEplFrame*) abBuffer;
tEplFrameInfo   FrameInfo;

    // check if previous SoA invitation was not answered
    switch (ReqServiceId_p)
    {
        case kEplDllReqServiceIdent:
        case kEplDllReqServiceStatus:
#if (EPL_DLL_PRES_CHAINING_MN != FALSE)
        case kEplDllReqServiceSync:
#endif
            // ASnd service registered?
            if (EplDllkInstance_g.m_aAsndFilter[ReqServiceId_p] == kEplDllAsndFilterAny)
            {   // ASnd service ID is registered
                AmiSetByteToLe(&pFrame->m_le_bSrcNodeId, (BYTE) uiNodeId_p);
                // EPL MsgType ASnd
                AmiSetByteToLe(&pFrame->m_le_bMessageType, (BYTE) kEplMsgTypeAsnd);
                // ASnd Service ID
                AmiSetByteToLe(&pFrame->m_Data.m_Asnd.m_le_bServiceId, (BYTE) ReqServiceId_p);
                // create frame info structure
                FrameInfo.m_pFrame = pFrame;
                FrameInfo.m_uiFrameSize = 18;   // empty non existing ASnd frame
                // forward frame via async receive FIFO to userspace
                Ret = EplDllkCalAsyncFrameReceived(&FrameInfo);
            }
            break;
        default:
            // no invitation issued or it was successfully answered or it is uninteresting
            break;
    }

    return Ret;
}


#endif // (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)


#if EPL_DLL_PRES_CHAINING_CN != FALSE

//---------------------------------------------------------------------------
//
// Function:    EplDllkPResChainingEnable()
//
// Description: Enables PRes Chaining mode
//
// Parameters:  none
//
// Returns:     tEplKernel              = error code
//
//
// State:
//
//---------------------------------------------------------------------------

static tEplKernel EplDllkPResChainingEnable (void)
{
tEplKernel      Ret = kEplSuccessful;
tEplFrame*      pTxFrameSyncRes;

    if (EplDllkInstance_g.m_fPrcEnabled == FALSE)
    {
        // relocate PReq filter to PResMN
        AmiSetQword48ToBe(&EplDllkInstance_g.m_aFilter[EPL_DLLK_FILTER_PREQ].m_abFilterValue[0],
                          EPL_C_DLL_MULTICAST_PRES);
        AmiSetByteToBe(&EplDllkInstance_g.m_aFilter[EPL_DLLK_FILTER_PREQ].m_abFilterValue[14],
                       kEplMsgTypePres);
        AmiSetByteToBe(&EplDllkInstance_g.m_aFilter[EPL_DLLK_FILTER_PREQ].m_abFilterValue[15],
                       EPL_C_ADR_BROADCAST); // Set Destination Node ID to C_ADR_BROADCAST

        EplDllkInstance_g.m_pTxBuffer[EPL_DLLK_TXFRAME_PRES].m_dwTimeOffsetNs = EplDllkInstance_g.m_dwPrcPResTimeFirst;

        Ret = EdrvChangeFilter(EplDllkInstance_g.m_aFilter, EPL_DLLK_FILTER_COUNT,
                               EPL_DLLK_FILTER_PREQ,
                               EDRV_FILTER_CHANGE_VALUE | EDRV_FILTER_CHANGE_AUTO_RESPONSE_DELAY);
        if (Ret != kEplSuccessful)
        {
            goto Exit;
        }

        EplDllkInstance_g.m_fPrcEnabled = TRUE;

        pTxFrameSyncRes = (tEplFrame *) EplDllkInstance_g.m_pTxBuffer[EPL_DLLK_TXFRAME_SYNCRES].m_pbBuffer;

        AmiSetDwordToLe(&pTxFrameSyncRes->m_Data.m_Asnd.m_Payload.m_SyncResponse.m_le_dwSyncStatus,
                        AmiGetDwordFromLe(&pTxFrameSyncRes->m_Data.m_Asnd.m_Payload.m_SyncResponse.m_le_dwSyncStatus)
                        | EPL_SYNC_PRES_MODE_SET);
        // update SyncRes Tx buffer in Edrv
        Ret = EdrvUpdateTxMsgBuffer(&EplDllkInstance_g.m_pTxBuffer[EPL_DLLK_TXFRAME_SYNCRES]);
        if (Ret != kEplSuccessful)
        {
            goto Exit;
        }

#if (EPL_DLL_PROCESS_SYNC == EPL_DLL_PROCESS_SYNC_ON_TIMER)
        Ret = EplTimerSynckSetLossOfSyncTolerance2Ns(EplDllkInstance_g.m_dwPrcPResFallBackTimeout);
#endif
    }

Exit:
//    PRINTF("%s: returns %d\n", __func__, Ret);
    return Ret;
}


//---------------------------------------------------------------------------
//
// Function:    EplDllkPResChainingDisable()
//
// Description: Disables PRes Chaining mode, thus restoring PReq/PRes mode
//
// Parameters:  none
//
// Returns:     tEplKernel              = error code
//
//
// State:
//
//---------------------------------------------------------------------------

static tEplKernel EplDllkPResChainingDisable (void)
{
tEplKernel      Ret = kEplSuccessful;
tEplFrame*      pTxFrameSyncRes;

    if (EplDllkInstance_g.m_fPrcEnabled != FALSE)
    {   // relocate PReq filter from PResMN to PReq
        EPL_MEMCPY(&EplDllkInstance_g.m_aFilter[EPL_DLLK_FILTER_PREQ].m_abFilterValue[0],
                   &EplDllkInstance_g.m_be_abLocalMac[0], 6);
        AmiSetByteToBe(&EplDllkInstance_g.m_aFilter[EPL_DLLK_FILTER_PREQ].m_abFilterValue[14],
                       kEplMsgTypePreq);
        AmiSetByteToBe(&EplDllkInstance_g.m_aFilter[EPL_DLLK_FILTER_PREQ].m_abFilterMask[15],
                       (BYTE) EplDllkInstance_g.m_DllConfigParam.m_uiNodeId); // Set Dst Node ID

        // disable auto-response delay
        EplDllkInstance_g.m_pTxBuffer[EPL_DLLK_TXFRAME_PRES].m_dwTimeOffsetNs = 0;

        EplDllkInstance_g.m_fPrcEnabled = FALSE;

        Ret = EdrvChangeFilter(EplDllkInstance_g.m_aFilter, EPL_DLLK_FILTER_COUNT,
                               EPL_DLLK_FILTER_PREQ,
                               EDRV_FILTER_CHANGE_VALUE | EDRV_FILTER_CHANGE_AUTO_RESPONSE_DELAY);
        if (Ret != kEplSuccessful)
        {
            goto Exit;
        }

        pTxFrameSyncRes = (tEplFrame *) EplDllkInstance_g.m_pTxBuffer[EPL_DLLK_TXFRAME_SYNCRES].m_pbBuffer;

        AmiSetDwordToLe(&pTxFrameSyncRes->m_Data.m_Asnd.m_Payload.m_SyncResponse.m_le_dwSyncStatus,
                        AmiGetDwordFromLe(&pTxFrameSyncRes->m_Data.m_Asnd.m_Payload.m_SyncResponse.m_le_dwSyncStatus)
                        & ~EPL_SYNC_PRES_MODE_SET);
        // update SyncRes Tx buffer in Edrv
        Ret = EdrvUpdateTxMsgBuffer(&EplDllkInstance_g.m_pTxBuffer[EPL_DLLK_TXFRAME_SYNCRES]);
        if (Ret != kEplSuccessful)
        {
            goto Exit;
        }

#if (EPL_DLL_PROCESS_SYNC == EPL_DLL_PROCESS_SYNC_ON_TIMER)
        Ret = EplTimerSynckSetLossOfSyncTolerance2Ns(0);
#endif
    }

Exit:
    return Ret;
}


#if (EPL_DLL_PROCESS_SYNC == EPL_DLL_PROCESS_SYNC_ON_TIMER)

//---------------------------------------------------------------------------
//
// Function:    EplDllkCbCnPResFallBackTimeout()
//
// Description: called by timer sync module. It signals that PResFallBackTimeout has triggered.
//
// Parameters:  void
//
// Returns:     tEplKernel              = error code
//
//
// State:
//
//---------------------------------------------------------------------------

static tEplKernel PUBLIC EplDllkCbCnPResFallBackTimeout(void)
{
tEplKernel      Ret = kEplSuccessful;
tEplNmtState    NmtState;

TGT_DLLK_DECLARE_FLAGS

    TGT_DLLK_ENTER_CRITICAL_SECTION();

    NmtState = EplDllkInstance_g.m_NmtState;

    if (NmtState <= kEplNmtGsResetConfiguration)
    {
        goto Exit;
    }

    Ret = EplDllkPResChainingDisable();
    if (Ret != kEplSuccessful)
    {
        goto Exit;
    }

Exit:
    if (Ret != kEplSuccessful)
    {
    DWORD   dwArg;

        BENCHMARK_MOD_02_TOGGLE(7);

        dwArg = EplDllkInstance_g.m_DllState | (kEplNmtEventDllCeFrameTimeout << 8);

        // Error event for API layer
        Ret = EplEventkPostError(kEplEventSourceDllk,
                        Ret,
                        sizeof(dwArg),
                        &dwArg);
    }

    TGT_DLLK_LEAVE_CRITICAL_SECTION();

    return Ret;
}
#endif

#endif // #if EPL_DLL_PRES_CHAINING_CN != FALSE

#endif // #if(((EPL_MODULE_INTEGRATION) & (EPL_MODULE_DLLK)) != 0)
// EOF

