/**
********************************************************************************
\file   nmtmnu.c

\brief  Implementation of NMT MNU module

This file contains the implementation of the NMT MNU module.

\ingroup module_nmtmnu
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2013, SYSTEC electronic GmbH
Copyright (c) 2013, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
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
#include "user/nmtmnu.h"
#include "user/timeru.h"
#include "user/identu.h"
#include "user/statusu.h"
#include "user/dllucal.h"
#include "Benchmark.h"
#include "obd.h"

#if EPL_NMTMNU_PRES_CHAINING_MN != FALSE
#include "user/syncu.h"
#endif

#if defined(CONFIG_INCLUDE_NMT_MN)

#if !defined(CONFIG_INCLUDE_OBD)
#error "NmtMnu module needs module OBD!"
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

// TracePoint support for realtime-debugging
#ifdef _DBG_TRACE_POINTS_
    void  PUBLIC  TgtDbgSignalTracePoint (UINT8 bTracePointNumber_p);
    void  PUBLIC  TgtDbgPostTraceValue (UINT32 dwTraceValue_p);
    #define TGT_DBG_SIGNAL_TRACE_POINT(p)   TgtDbgSignalTracePoint(p)
    #define TGT_DBG_POST_TRACE_VALUE(v)     TgtDbgPostTraceValue(v)
#else
    #define TGT_DBG_SIGNAL_TRACE_POINT(p)
    #define TGT_DBG_POST_TRACE_VALUE(v)
#endif
#define NMTMNU_DBG_POST_TRACE_VALUE(Event_p, uiNodeId_p, wErrorCode_p) \
    TGT_DBG_POST_TRACE_VALUE((kEplEventSinkNmtMnu << 28) | (Event_p << 24) \
                             | (uiNodeId_p << 16) | wErrorCode_p)

// defines for flags in node info structure
#define NMTMNU_NODE_FLAG_ISOCHRON               0x0001  // CN is being accessed isochronously
#define NMTMNU_NODE_FLAG_NOT_SCANNED            0x0002  // CN was not scanned once -> decrement SignalCounter and reset flag
#define NMTMNU_NODE_FLAG_HALTED                 0x0004  // boot process for this CN is halted
#define NMTMNU_NODE_FLAG_NMT_CMD_ISSUED         0x0008  // NMT command was just issued, wrong NMT states will be tolerated
#define NMTMNU_NODE_FLAG_PREOP2_REACHED         0x0010  // NodeAddIsochronous has been called, waiting for ISOCHRON
#define NMTMNU_NODE_FLAG_COUNT_STATREQ          0x0300  // counter for StatusRequest timer handle
#define NMTMNU_NODE_FLAG_COUNT_LONGER           0x0C00  // counter for longer timeouts timer handle
#define NMTMNU_NODE_FLAG_INC_STATREQ            0x0100  // increment for StatusRequest timer handle
#define NMTMNU_NODE_FLAG_INC_LONGER             0x0400  // increment for longer timeouts timer handle

#if EPL_NMTMNU_PRES_CHAINING_MN != FALSE
#define NMTMNU_NODE_FLAG_PRC_ADD_SCHEDULED      0x0001
#define NMTMNU_NODE_FLAG_PRC_ADD_IN_PROGRESS    0x0002
#define NMTMNU_NODE_FLAG_PRC_ADD_SYNCREQ_SENT   0x0004  // Covers time between SyncReq and SyncRes for addition
#define NMTMNU_NODE_FLAG_PRC_SHIFT_REQUIRED     0x0010
#define NMTMNU_NODE_FLAG_PRC_SYNC_ERR           0x0020  // SyncRes 1-bit error counter

#define NMTMNU_NODE_FLAG_PRC_CALL_MEASURE       0x0100
#define NMTMNU_NODE_FLAG_PRC_CALL_SHIFT         0x0200
#define NMTMNU_NODE_FLAG_PRC_CALL_ADD           0x0300
#define NMTMNU_NODE_FLAG_PRC_CALL_MASK          0x0300

#define NMTMNU_NODE_FLAG_PRC_VERIFY             0x0400

#define NMTMNU_NODE_FLAG_PRC_STOP_NODE          0x1000
#define NMTMNU_NODE_FLAG_PRC_RESET_NODE         0x2000
#define NMTMNU_NODE_FLAG_PRC_RESET_COM          0x3000
#define NMTMNU_NODE_FLAG_PRC_RESET_CONF         0x4000
#define NMTMNU_NODE_FLAG_PRC_RESET_SW           0x5000
#define NMTMNU_NODE_FLAG_PRC_RESET_MASK         0x7000
#endif

// defines for timer arguments to draw a distinction between several events
#define NMTMNU_TIMERARG_NODE_MASK               0x000000FFL // mask that contains the node-ID
#define NMTMNU_TIMERARG_IDENTREQ                0x00010000L // timer event is for IdentRequest
#define NMTMNU_TIMERARG_STATREQ                 0x00020000L // timer event is for StatusRequest
#define NMTMNU_TIMERARG_LONGER                  0x00040000L // timer event is for longer timeouts
#define NMTMNU_TIMERARG_STATE_MON               0x00080000L // timer event for StatusRequest to monitor execution of NMT state changes
#define NMTMNU_TIMERARG_COUNT_SR                0x00000300L // counter for StatusRequest
#define NMTMNU_TIMERARG_COUNT_LO                0x00000C00L // counter for longer timeouts
// The counters must have the same position as in the node flags above.

#define NMTMNU_SET_FLAGS_TIMERARG_STATREQ(pNodeInfo_p, nodeId_p, timerArg_p)            \
    pNodeInfo_p->flags = ((pNodeInfo_p->flags + NMTMNU_NODE_FLAG_INC_STATREQ) &         \
                          NMTMNU_NODE_FLAG_COUNT_STATREQ) |                             \
                          (pNodeInfo_p->flags & ~NMTMNU_NODE_FLAG_COUNT_STATREQ);       \
    timerArg_p.m_Arg.m_dwVal = NMTMNU_TIMERARG_STATREQ | nodeId_p  |                    \
                               (pNodeInfo_p->flags & NMTMNU_NODE_FLAG_COUNT_STATREQ);   \
    timerArg_p.m_EventSink  = kEplEventSinkNmtMnu;

#define NMTMNU_SET_FLAGS_TIMERARG_IDENTREQ(pNodeInfo_p, nodeId_p, timerArg_p)           \
    pNodeInfo_p->flags = ((pNodeInfo_p->flags + NMTMNU_NODE_FLAG_INC_STATREQ) &         \
                          NMTMNU_NODE_FLAG_COUNT_STATREQ) |                             \
                          (pNodeInfo_p->flags & ~NMTMNU_NODE_FLAG_COUNT_STATREQ);       \
    timerArg_p.m_Arg.m_dwVal = NMTMNU_TIMERARG_IDENTREQ | nodeId_p |                    \
                               (pNodeInfo_p->flags & NMTMNU_NODE_FLAG_COUNT_STATREQ);   \
    timerArg_p.m_EventSink = kEplEventSinkNmtMnu;

#define NMTMNU_SET_FLAGS_TIMERARG_LONGER(pNodeInfo_p, nodeId_p, timerArg_p)             \
    pNodeInfo_p->flags = ((pNodeInfo_p->flags + NMTMNU_NODE_FLAG_INC_LONGER) &          \
                          NMTMNU_NODE_FLAG_COUNT_LONGER) |                              \
                          (pNodeInfo_p->flags & ~NMTMNU_NODE_FLAG_COUNT_LONGER);        \
    timerArg_p.m_Arg.m_dwVal = NMTMNU_TIMERARG_LONGER | nodeId_p |                      \
                               (pNodeInfo_p->flags & NMTMNU_NODE_FLAG_COUNT_LONGER);    \
    timerArg_p.m_EventSink = kEplEventSinkNmtMnu;

#define NMTMNU_SET_FLAGS_TIMERARG_STATE_MON(pNodeInfo_p, nodeId_p, timerArg_p)          \
    pNodeInfo_p->flags = ((pNodeInfo_p->flags + NMTMNU_NODE_FLAG_INC_STATREQ) &         \
                          NMTMNU_NODE_FLAG_COUNT_STATREQ) |                             \
                          (pNodeInfo_p->flags & ~NMTMNU_NODE_FLAG_COUNT_STATREQ);       \
    timerArg_p.m_Arg.m_dwVal = NMTMNU_TIMERARG_STATE_MON | nodeId_p |                   \
                               (pNodeInfo_p->flags & NMTMNU_NODE_FLAG_COUNT_STATREQ);   \
    timerArg_p.m_EventSink = kEplEventSinkNmtMnu;

// defines for global flags
#define NMTMNU_FLAG_HALTED                      0x0001  // boot process is halted
#define NMTMNU_FLAG_APP_INFORMED                0x0002  // application was informed about possible NMT state change
#define NMTMNU_FLAG_USER_RESET                  0x0004  // NMT reset issued by user / diagnostic node
#if EPL_NMTMNU_PRES_CHAINING_MN != FALSE
#define NMTMNU_FLAG_PRC_ADD_SCHEDULED           0x0008  // at least one node is scheduled
                                                        // for addition to isochronous phase
#define NMTMNU_FLAG_PRC_ADD_IN_PROGRESS         0x0010  // add-PRC-node process is in progress
#endif

// return pointer to node info structure for specified node ID
// d.k. may be replaced by special (hash) function if node ID array is smaller than 254
#define NMTMNU_GET_NODEINFO(nodeId_p) (&nmtMnuInstance_g.aNodeInfo[nodeId_p - 1])

//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------

/**
* \brief Node command structure
*
* The structure defines a node command.
*/
typedef struct
{
    UINT                nodeId;
    tNmtNodeCommand     nodeCommand;
} tNmtMnuNodeCmd;

/*
Do not change the constants as the array with function pointers to the
handlers depends on these constants!
*/
typedef enum
{
    kNmtMnuIntNodeEventNoIdentResponse      = 0x00,
    kNmtMnuIntNodeEventIdentResponse        = 0x01,
    kNmtMnuIntNodeEventBoot                 = 0x02,
    kNmtMnuIntNodeEventExecResetConf        = 0x03,
    kNmtMnuIntNodeEventExecResetNode        = 0x04,
    kNmtMnuIntNodeEventConfigured           = 0x05,
    kNmtMnuIntNodeEventNoStatusResponse     = 0x06,
    kNmtMnuIntNodeEventStatusResponse       = 0x07,
    kNmtMnuIntNodeEventHeartbeat            = 0x08,
    kNmtMnuIntNodeEventNmtCmdSent           = 0x09,
    kNmtMnuIntNodeEventTimerIdentReq        = 0x0A,
    kNmtMnuIntNodeEventTimerStatReq         = 0x0B,
    kNmtMnuIntNodeEventTimerStateMon        = 0x0C,
    kNmtMnuIntNodeEventTimerLonger          = 0x0D,
    kNmtMnuIntNodeEventError                = 0x0E,
} tNmtMnuIntNodeEvent;

typedef enum
{
    kNmtMnuNodeStateUnknown                 = 0x00,
    kNmtMnuNodeStateIdentified              = 0x01,
    kNmtMnuNodeStateConfRestored            = 0x02, // CN ResetNode after restore configuration
    kNmtMnuNodeStateResetConf               = 0x03, // CN ResetConf after configuration update
    kNmtMnuNodeStateConfigured              = 0x04, // BootStep1 completed
    kNmtMnuNodeStateReadyToOp               = 0x05, // BootStep2 completed
    kNmtMnuNodeStateComChecked              = 0x06, // Communication checked successfully
    kNmtMnuNodeStateOperational             = 0x07, // CN is in NMT state OPERATIONAL
} tNmtMnuNodeState;

typedef INT (*tProcessNodeEventFunc)(UINT nodeId_p, tNmtState nodeNmtState_p,
                                     tNmtState nmtState_p, UINT16 errorCode_p,
                                     tEplKernel* pRet_p);

/**
* \brief Node information structure
*
* The following struct specifies a node.
*/
typedef struct
{
    tEplTimerHdl        timerHdlStatReq;        ///< Timer to delay StatusRequests and IdentRequests
    tEplTimerHdl        timerHdlLonger;         ///< 2nd timer for NMT command EnableReadyToOp and CheckCommunication
    tNmtMnuNodeState    nodeState;              ///< Internal node state (kind of sub state of NMT state)
    UINT32              nodeCfg;                ///< Subindex from 0x1F81
    UINT16              flags;                  ///< Node flags (see node flag defines)
#if EPL_NMTMNU_PRES_CHAINING_MN != FALSE
    UINT16              prcFlags;               ///< PRC specific node flags
    UINT32              relPropagationDelayNs;  ///< Propagation delay in nanoseconds
    UINT32              pResTimeFirstNs;        ///< PRes time
#endif
} tNmtMnuNodeInfo;

/**
* \brief nmtmnu instance structure
*
* The following struct implements the instance information of the NMT MNU module.
*/
typedef struct
{
    tNmtMnuNodeInfo     aNodeInfo[EPL_NMT_MAX_NODE_ID];  ///< Information about CNs
    tEplTimerHdl        timerHdlNmtState;       ///< Timeout for stay in NMT state
    UINT                mandatorySlaveCount;    ///< Count of found mandatory CNs
    UINT                signalSlaveCount;       ///< Count of CNs which are not identified
    ULONG               statusRequestDelay;     ///< In [ms] (object 0x1006 * EPL_C_NMT_STATREQ_CYCLE)
    ULONG               timeoutReadyToOp;       ///< In [ms] (object 0x1F89/4)
    ULONG               timeoutCheckCom;        ///< In [ms] (object 0x1006 * MultiplexedCycleCount)
    UINT16              flags;                  ///< Global flags
    UINT32              nmtStartup;             ///< Object 0x1F80 NMT_StartUp_U32
    tNmtMnuCbNodeEvent  pfnCbNodeEvent;         ///< Callback function for node events
    tNmtMnuCbBootEvent  pfnCbBootEvent;         ///< Callback function for boot events
#if EPL_NMTMNU_PRES_CHAINING_MN != FALSE
    UINT32              prcPResMnTimeoutNs;             ///< to be commented!
    UINT32              prcPResTimeFirstCorrectionNs;   ///< to be commented!
    UINT32              prcPResTimeFirstNegOffsetNs;    ///< to be commented!
#endif
} tNmtMnuInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tNmtMnuInstance   nmtMnuInstance_g;



//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static tEplKernel cbNmtRequest(tFrameInfo * pFrameInfo_p);
static tEplKernel cbIdentResponse(UINT nodeId_p, tEplIdentResponse* pIdentResponse_p);
static tEplKernel cbStatusResponse(UINT nodeId_p, tEplStatusResponse* pStatusResponse_p);
static tEplKernel cbNodeAdded(UINT nodeId_p);
static tEplKernel checkNmtState(UINT nodeId_p, tNmtMnuNodeInfo* pNodeInfo_p,
                                tNmtState nodeNmtState_p, UINT16 errorCode_p,
                                tNmtState localNmtState_p);
static tEplKernel addNodeIsochronous(UINT nodeId_p);
static tEplKernel startBootStep1(BOOL fNmtResetAllIssued_p);
static tEplKernel startBootStep2(void);
static tEplKernel startCheckCom(void);
static tEplKernel nodeBootStep2(UINT nodeId_p, tNmtMnuNodeInfo* pNodeInfo_p);
static tEplKernel nodeCheckCom(UINT nodeId_p, tNmtMnuNodeInfo* pNodeInfo_p);
static tEplKernel startNodes(void);
static tEplKernel doPreop1(tEventNmtStateChange nmtStateChange_p);
static tEplKernel processInternalEvent(UINT nodeId_p, tNmtState nodeNmtState_p,
                                       UINT16 errorCode_p, tNmtMnuIntNodeEvent nodeEvent_p);
static tEplKernel reset(void);

#if EPL_NMTMNU_PRES_CHAINING_MN != FALSE
static tEplKernel prcMeasure(void);
static tEplKernel prcCalculate(UINT nodeIdFirstNode_p);
static tEplKernel prcShift(UINT nodeIdPrevShift_p);
static tEplKernel prcAdd(UINT nodeIdPrevAdd_p);
static tEplKernel prcVerify(UINT nodeId_p);

static tEplKernel prcCbSyncResMeasure(UINT, tEplSyncResponse*);
static tEplKernel prcCbSyncResShift(UINT, tEplSyncResponse*);
static tEplKernel prcCbSyncResAdd(UINT, tEplSyncResponse*);
static tEplKernel prcCbSyncResVerify(UINT, tEplSyncResponse*);
static tEplKernel prcCbSyncResNextAction(UINT, tEplSyncResponse*);

static tEplKernel prcCalcPResResponseTimeNs(UINT nodeId_p, UINT nodeIdPredNode_p,
                                            UINT32* pPResResponseTimeNs_p);
static tEplKernel prcCalcPResChainingSlotTimeNs(UINT nodeIdLastNode_p,
                                                UINT32* pPResChainingSlotTimeNs_p);

static tEplKernel prcFindPredecessorNode(UINT nodeId_p);
static void       prcSyncError(tNmtMnuNodeInfo* pNodeInfo_p);
static void       prcSetFlagsNmtCommandReset(tNmtMnuNodeInfo* pNodeInfo_p,
                                             tNmtCommand nmtCommand_p);
#endif

/* internal node event handler functions */
static INT processNodeEventNoIdentResponse (UINT nodeId_p, tNmtState nodeNmtState_p,
                                            tNmtState nmtState_p, UINT16 errorCode_p, tEplKernel* pRet_p);
static INT processNodeEventIdentResponse   (UINT nodeId_p, tNmtState nodeNmtState_p,
                                            tNmtState nmtState_p, UINT16 errorCode_p, tEplKernel* pRet_p);
static INT processNodeEventBoot            (UINT nodeId_p, tNmtState nodeNmtState_p,
                                            tNmtState nmtState_p, UINT16 errorCode_p, tEplKernel* pRet_p);
static INT processNodeEventExecResetConf   (UINT nodeId_p, tNmtState nodeNmtState_p,
                                            tNmtState nmtState_p, UINT16 errorCode_p, tEplKernel* pRet_p);
static INT processNodeEventExecResetNode   (UINT nodeId_p, tNmtState nodeNmtState_p,
                                            tNmtState nmtState_p, UINT16 errorCode_p, tEplKernel* pRet_p);
static INT processNodeEventConfigured      (UINT nodeId_p, tNmtState nodeNmtState_p,
                                            tNmtState nmtState_p, UINT16 errorCode_p, tEplKernel* pRet_p);
static INT processNodeEventNoStatusResponse(UINT nodeId_p, tNmtState nodeNmtState_p,
                                            tNmtState nmtState_p, UINT16 errorCode_p, tEplKernel* pRet_p);
static INT processNodeEventStatusResponse  (UINT nodeId_p, tNmtState nodeNmtState_p,
                                            tNmtState nmtState_p, UINT16 errorCode_p, tEplKernel* pRet_p);
static INT processNodeEventHeartbeat       (UINT nodeId_p, tNmtState nodeNmtState_p,
                                            tNmtState nmtState_p, UINT16 errorCode_p, tEplKernel* pRet_p);
static INT processNodeEventNmtCmdSent      (UINT nodeId_p, tNmtState nodeNmtState_p,
                                            tNmtState nmtState_p, UINT16 errorCode_p, tEplKernel* pRet_p);
static INT processNodeEventTimerIdentReq   (UINT nodeId_p, tNmtState nodeNmtState_p,
                                            tNmtState nmtState_p, UINT16 errorCode_p, tEplKernel* pRet_p);
static INT processNodeEventTimerStatReq    (UINT nodeId_p, tNmtState nodeNmtState_p,
                                            tNmtState nmtState_p, UINT16 errorCode_p, tEplKernel* pRet_p);
static INT processNodeEventTimerStateMon   (UINT nodeId_p, tNmtState nodeNmtState_p,
                                            tNmtState nmtState_p, UINT16 errorCode_p, tEplKernel* pRet_p);
static INT processNodeEventTimerLonger     (UINT nodeId_p, tNmtState nodeNmtState_p,
                                            tNmtState nmtState_p, UINT16 errorCode_p, tEplKernel* pRet_p);
static INT processNodeEventError           (UINT nodeId_p, tNmtState nodeNmtState_p,
                                            tNmtState nmtState_p, UINT16 errorCode_p, tEplKernel* pRet_p);

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
/*
The following function table depends on the types defined in tNmtMnuIntNodeEvent.
Do not reorder them without adapting the constants!
*/
static tProcessNodeEventFunc apfnNodeEventFuncs_l[] =
{
    processNodeEventNoIdentResponse,    // kNmtMnuIntNodeEventNoIdentResponse
    processNodeEventIdentResponse,      // kNmtMnuIntNodeEventIdentResponse
    processNodeEventBoot,               // kNmtMnuIntNodeEventBoot
    processNodeEventExecResetConf,      // kNmtMnuIntNodeEventExecResetConf
    processNodeEventExecResetNode,      // kNmtMnuIntNodeEventExecResetNode
    processNodeEventConfigured,         // kNmtMnuIntNodeEventConfigured
    processNodeEventNoStatusResponse,   // kNmtMnuIntNodeEventNoStatusResponse
    processNodeEventStatusResponse,     // kNmtMnuIntNodeEventStatusResponse
    processNodeEventHeartbeat,          // kNmtMnuIntNodeEventHeartbeat
    processNodeEventNmtCmdSent,         // kNmtMnuIntNodeEventNmtCmdSent
    processNodeEventTimerIdentReq,      // kNmtMnuIntNodeEventTimerIdentReq
    processNodeEventTimerStatReq,       // kNmtMnuIntNodeEventTimerStatReq
    processNodeEventTimerStateMon,      // kNmtMnuIntNodeEventTimerStateMon
    processNodeEventTimerLonger,        // kNmtMnuIntNodeEventTimerLonger
    processNodeEventError               // kNmtMnuIntNodeEventError
};

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//


//------------------------------------------------------------------------------
/**
\brief  Init nmtmnu module

The function initializes an instance of the nmtmnu module

\param  pfnCbNodeEvent_p        Pointer to node event callback function.
\param  pfnCbBootEvent_p        Pointer to boot event callback function.

\return The function returns a tEplKernel error code.

\ingroup module_nmtmnu
*/
//------------------------------------------------------------------------------
tEplKernel nmtmnu_init(tNmtMnuCbNodeEvent pfnCbNodeEvent_p, tNmtMnuCbBootEvent pfnCbBootEvent_p)
{
    tEplKernel ret;
    ret = nmtmnu_addInstance(pfnCbNodeEvent_p, pfnCbBootEvent_p);
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Add nmtmnu module instance

The function adds a nmtmnu module instance.

\param  pfnCbNodeEvent_p        Pointer to node event callback function.
\param  pfnCbBootEvent_p        Pointer to boot event callback function.

\return The function returns a tEplKernel error code.

\ingroup module_nmtmnu
*/
//------------------------------------------------------------------------------
tEplKernel nmtmnu_addInstance(tNmtMnuCbNodeEvent pfnCbNodeEvent_p,
                              tNmtMnuCbBootEvent pfnCbBootEvent_p)
{
    tEplKernel ret = kEplSuccessful;

    EPL_MEMSET(&nmtMnuInstance_g, 0, sizeof(nmtMnuInstance_g));

    if ((pfnCbNodeEvent_p == NULL) || (pfnCbBootEvent_p == NULL))
    {
        ret = kEplNmtInvalidParam;
        goto Exit;
    }
    nmtMnuInstance_g.pfnCbNodeEvent = pfnCbNodeEvent_p;
    nmtMnuInstance_g.pfnCbBootEvent = pfnCbBootEvent_p;
    nmtMnuInstance_g.statusRequestDelay = 5000L;

    // register NmtMnResponse callback function
    ret = dllucal_regAsndService(kDllAsndNmtRequest, cbNmtRequest, kDllAsndFilterLocal);

#if EPL_NMTMNU_PRES_CHAINING_MN != FALSE
    nmtMnuInstance_g.prcPResTimeFirstCorrectionNs =  50;
    nmtMnuInstance_g.prcPResTimeFirstNegOffsetNs  = 500;
#endif

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Delete nmtmnu module instance

The function deletes an nmtmnu module instance.

\return The function returns a tEplKernel error code.

\ingroup module_nmtmnu
*/
//------------------------------------------------------------------------------
tEplKernel nmtmnu_delInstance(void)
{
    tEplKernel  ret = kEplSuccessful;

    dllucal_regAsndService(kDllAsndNmtRequest, NULL, kDllAsndFilterNone);
    ret = reset();
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Send extended NMT command

The function sends a extended NMT command.

\param  nodeId_p            Node ID to which the NMT command will be sent.
\param  nmtCommand_p        NMT command to send.
\param  pNmtCommandData_p   Pointer to additional NMT command data.
\param  uiDataSize_p        Length of additional NMT command data.

\return The function returns a tEplKernel error code.

\ingroup module_nmtmnu
*/
//------------------------------------------------------------------------------
tEplKernel nmtmnu_sendNmtCommandEx(UINT nodeId_p, tNmtCommand nmtCommand_p,
                                   void* pNmtCommandData_p, UINT uiDataSize_p)
{
    tEplKernel          ret;
    tFrameInfo       frameInfo;
    UINT8               aBuffer[EPL_C_DLL_MINSIZE_NMTCMDEXT];
    tEplFrame*          pFrame;
    tDllNodeOpParam     nodeOpParam;
#if EPL_NMTMNU_PRES_CHAINING_MN != FALSE
    tNmtMnuNodeInfo*    pNodeInfo;
#endif

    ret = kEplSuccessful;

    if ((nodeId_p == 0) || (nodeId_p > EPL_C_ADR_BROADCAST))
    {   // invalid node ID specified
        ret = kEplInvalidNodeId;
        goto Exit;
    }

    if ((pNmtCommandData_p != NULL) &&
        (uiDataSize_p > (EPL_C_DLL_MINSIZE_NMTCMDEXT - EPL_C_DLL_MINSIZE_NMTCMD)))
    {
        ret = kEplNmtInvalidParam;
        goto Exit;
    }

    // $$$ d.k. may be check in future versions if the caller wants to perform
    //          prohibited state transitions. The CN should not perform these
    //          transitions, but the expected NMT state will be changed and never fullfilled.

#if EPL_NMTMNU_PRES_CHAINING_MN != FALSE
    pNodeInfo = NMTMNU_GET_NODEINFO(nodeId_p);

    if (pNodeInfo->nodeCfg & EPL_NODEASSIGN_PRES_CHAINING)
    {   // Node is a PRes Chaining node
        switch (nmtCommand_p)
        {
            case kNmtCmdStopNode:
            case kNmtCmdResetNode:
            case kNmtCmdResetCommunication:
            case kNmtCmdResetConfiguration:
            case kNmtCmdSwReset:
                if (pNodeInfo->prcFlags & (NMTMNU_NODE_FLAG_PRC_ADD_SCHEDULED |
                                              NMTMNU_NODE_FLAG_PRC_ADD_IN_PROGRESS))
                {   // For this node, addition to the isochronous phase is scheduled
                    // or in progress
                    // Skip addition for this node
                    pNodeInfo->prcFlags &= ~(NMTMNU_NODE_FLAG_PRC_ADD_SCHEDULED |
                                                NMTMNU_NODE_FLAG_PRC_ADD_IN_PROGRESS);
                }

                if (pNodeInfo->flags & NMTMNU_NODE_FLAG_ISOCHRON)
                {   // PRes Chaining is enabled
                    tDllSyncRequest    SyncReqData;
                    UINT               size;

                    // Store NMT command for later execution
                    prcSetFlagsNmtCommandReset(pNodeInfo, nmtCommand_p);

                    // Disable PRes Chaining
                    SyncReqData.nodeId      = nodeId_p;
                    SyncReqData.syncControl = EPL_SYNC_PRES_MODE_RESET |
                                                  EPL_SYNC_DEST_MAC_ADDRESS_VALID;
                    size = sizeof(UINT) + sizeof(UINT32);

                    ret = syncu_requestSyncResponse(prcCbSyncResNextAction, &SyncReqData, size);
                    switch (ret)
                    {
                        case kEplSuccessful:
                            // Mark node as removed from the isochronous phase
                            pNodeInfo->flags &= ~NMTMNU_NODE_FLAG_ISOCHRON;
                            // Send NMT command when SyncRes is received
                            goto Exit;

                        case kEplNmtSyncReqRejected:
                            // There has already been posted a SyncReq for this node.
                            // Retry when SyncRes is received
                            ret = kEplSuccessful;
                            goto Exit;

                        default:
                            goto Exit;
                    }
                }

                if (pNodeInfo->prcFlags & (NMTMNU_NODE_FLAG_PRC_RESET_MASK |
                                              NMTMNU_NODE_FLAG_PRC_ADD_SYNCREQ_SENT))
                {   // A Node-reset NMT command was already scheduled or
                    // PRes Chaining is going to be enabled but the appropriate SyncRes
                    // has not been received, yet.

                    // Set the current NMT command if it has higher priority than a present one.
                    prcSetFlagsNmtCommandReset(pNodeInfo, nmtCommand_p);

                    // Wait for the SyncRes
                    goto Exit;
                }

                break;

            default:
                // Other NMT commands
                break;
        }
    }
#endif

    // build frame
    pFrame = (tEplFrame*) aBuffer;
    EPL_MEMSET(pFrame, 0x00, sizeof(aBuffer));
    AmiSetByteToLe(&pFrame->m_le_bDstNodeId, (UINT8) nodeId_p);
    AmiSetByteToLe(&pFrame->m_Data.m_Asnd.m_le_bServiceId, (UINT8) kDllAsndNmtCommand);
    AmiSetByteToLe(&pFrame->m_Data.m_Asnd.m_Payload.m_NmtCommandService.m_le_bNmtCommandId,
        (UINT8)nmtCommand_p);
    if ((pNmtCommandData_p != NULL) && (uiDataSize_p > 0))
    {   // copy command data to frame
        EPL_MEMCPY(&pFrame->m_Data.m_Asnd.m_Payload.m_NmtCommandService.m_le_abNmtCommandData[0], pNmtCommandData_p, uiDataSize_p);
    }

    // build info structure
    frameInfo.pFrame = pFrame;
    frameInfo.frameSize = sizeof(aBuffer);

    // send NMT-Request
#if defined(CONFIG_INCLUDE_DLLU)
    ret = dllucal_sendAsyncFrame(&frameInfo, kDllAsyncReqPrioNmt);
    if (ret != kEplSuccessful)
    {
        goto Exit;
    }
#endif

    EPL_DBGLVL_NMTMN_TRACE("NMTCmd(%02X->%02X)\n", NmtCommand_p, nodeId_p);

#if EPL_NMTMNU_PRES_CHAINING_MN != FALSE
    if (pNodeInfo->nodeCfg & EPL_NODEASSIGN_PRES_CHAINING)
    {   // Node is a PRes Chaining node
        // The following action (delete node) is only necessary for non-PRC nodes
        goto Exit;
    }
#endif

    switch (nmtCommand_p)
    {
        case kNmtCmdStartNode:
        case kNmtCmdEnterPreOperational2:
        case kNmtCmdEnableReadyToOperate:
            // nothing left to do,
            // because any further processing is done
            // when the NMT command is actually sent
            goto Exit;

        case kNmtCmdStopNode:
            // remove CN from isochronous phase softly
            nodeOpParam.opNodeType = kDllNodeOpTypeSoftDelete;
            break;

        case kNmtCmdResetNode:
        case kNmtCmdResetCommunication:
        case kNmtCmdResetConfiguration:
        case kNmtCmdSwReset:
            // remove CN immediately from isochronous phase
            nodeOpParam.opNodeType = kDllNodeOpTypeIsochronous;
            break;

        default:
            goto Exit;
    }

    // The expected node state will be updated when the NMT command
    // was actually sent.
    // See functions processInternalEvent(kNmtMnuIntNodeEventNmtCmdSent),
    // EplNmtMnuProcessEvent(kEplEventTypeNmtMnuNmtCmdSent).

    // remove CN from isochronous phase;
    // This must be done here and not when NMT command is actually sent
    // because it will be too late and may cause unwanted errors
    if (nodeId_p != EPL_C_ADR_BROADCAST)
    {
        nodeOpParam.nodeId = nodeId_p;
        ret = dllucal_deleteNode(&nodeOpParam);
    }
    else
    {   // do it for all active CNs
        for (nodeId_p = 1; nodeId_p <= tabentries(nmtMnuInstance_g.aNodeInfo); nodeId_p++)
        {
            if ((NMTMNU_GET_NODEINFO(nodeId_p)->nodeCfg & (EPL_NODEASSIGN_NODE_IS_CN | EPL_NODEASSIGN_NODE_EXISTS)) != 0)
            {
                nodeOpParam.nodeId = nodeId_p;
                ret = dllucal_deleteNode(&nodeOpParam);
            }
        }
    }

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Send NMT command

The function sends a NMT command.

\param  nodeId_p            Node ID to which the NMT command will be sent.
\param  nmtCommand_p        NMT command to send.

\return The function returns a tEplKernel error code.

\ingroup module_nmtmnu
*/
//------------------------------------------------------------------------------
tEplKernel nmtmnu_sendNmtCommand(UINT nodeId_p, tNmtCommand  nmtCommand_p)
{
    return nmtmnu_sendNmtCommandEx(nodeId_p, nmtCommand_p, NULL, 0);
}

//------------------------------------------------------------------------------
/**
\brief  Request NMT command

The function requests the specified NMT command for the specified node. It may
also be applied to the local node.

\param  nodeId_p            Node ID for which the NMT command will be requested.
\param  nmtCommand_p        NMT command to request.

\return The function returns a tEplKernel error code.

\ingroup module_nmtmnu
*/
//------------------------------------------------------------------------------
tEplKernel nmtmnu_requestNmtCommand(UINT nodeId_p, tNmtCommand  nmtCommand_p)
{
    tEplKernel      ret = kEplSuccessful;
    tNmtState       nmtState;

    nmtState = nmtu_getNmtState();
    if (nmtState <= kNmtMsNotActive)
    {
        ret = kEplInvalidOperation;
        goto Exit;
    }

    if (nodeId_p > EPL_C_ADR_BROADCAST)
    {
        ret = kEplInvalidNodeId;
        goto Exit;
    }

    if (nodeId_p == 0x00)
        nodeId_p = EPL_C_ADR_MN_DEF_NODE_ID;

    if (nodeId_p == EPL_C_ADR_MN_DEF_NODE_ID)
    {   // apply command to local node-ID
        switch (nmtCommand_p)
        {
            case kNmtCmdIdentResponse:
                // issue request for local node
                ret = identu_requestIdentResponse(0x00, NULL);
                goto Exit;

            case kNmtCmdStatusResponse:
                // issue request for local node
                ret = statusu_requestStatusResponse(0x00, NULL);
                goto Exit;

            case kNmtCmdResetNode:
            case kNmtCmdResetCommunication:
            case kNmtCmdResetConfiguration:
            case kNmtCmdSwReset:
                nodeId_p = EPL_C_ADR_BROADCAST;
                break;

            case kNmtCmdInvalidService:
            default:
                ret = kEplObdAccessViolation;
                goto Exit;
        }
    }

    if (nodeId_p != EPL_C_ADR_BROADCAST)
    {   // apply command to remote node-ID, but not broadcast
        tNmtMnuNodeInfo* pNodeInfo;

        pNodeInfo = NMTMNU_GET_NODEINFO(nodeId_p);
        switch (nmtCommand_p)
        {
            case kNmtCmdIdentResponse:
                // issue request for remote node
                // if it is a non-existing node or no identrequest is running
                if (((pNodeInfo->nodeCfg &
                      (EPL_NODEASSIGN_NODE_IS_CN | EPL_NODEASSIGN_NODE_EXISTS))
                       != (EPL_NODEASSIGN_NODE_IS_CN | EPL_NODEASSIGN_NODE_EXISTS)) ||
                    ((pNodeInfo->nodeState != kNmtMnuNodeStateResetConf) &&
                     (pNodeInfo->nodeState != kNmtMnuNodeStateConfRestored) &&
                     (pNodeInfo->nodeState != kNmtMnuNodeStateUnknown)))
                {
                    ret = identu_requestIdentResponse(nodeId_p, NULL);
                }
                goto Exit;

            case kNmtCmdStatusResponse:
                // issue request for remote node
                // if it is a non-existing node or operational and not async-only
                if (((pNodeInfo->nodeCfg &
                     (EPL_NODEASSIGN_NODE_IS_CN | EPL_NODEASSIGN_NODE_EXISTS))
                      != (EPL_NODEASSIGN_NODE_IS_CN | EPL_NODEASSIGN_NODE_EXISTS)) ||
                    (((pNodeInfo->nodeCfg & EPL_NODEASSIGN_ASYNCONLY_NODE) == 0) &&
                      (pNodeInfo->nodeState == kNmtMnuNodeStateOperational)))
                {
                    ret = statusu_requestStatusResponse(nodeId_p, NULL);
                }
                goto Exit;

            default:
                break;
        }
    }

    switch (nmtCommand_p)
    {
        case kNmtCmdResetNode:
        case kNmtCmdResetCommunication:
        case kNmtCmdResetConfiguration:
        case kNmtCmdSwReset:
            if (nodeId_p == EPL_C_ADR_BROADCAST)
            {   // memorize that this is a user requested reset
                nmtMnuInstance_g.flags |= NMTMNU_FLAG_USER_RESET;
            }
            break;

        case kNmtCmdStartNode:
        case kNmtCmdStopNode:
        case kNmtCmdEnterPreOperational2:
        case kNmtCmdEnableReadyToOperate:
        default:
            break;
    }

    // send command to remote node
    ret = nmtmnu_sendNmtCommand(nodeId_p, nmtCommand_p);
Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Trigger NMT state change

The function triggers a NMT state change by sending the specified node command
to the specified node.

\param  nodeId_p            Node ID to send the node command to.
\param  nodeCommand_p       Node command to send.

\return The function returns a tEplKernel error code.

\ingroup module_nmtmnu
*/
//------------------------------------------------------------------------------
tEplKernel nmtmnu_triggerStateChange(UINT nodeId_p, tNmtNodeCommand nodeCommand_p)
{
    tEplKernel          ret = kEplSuccessful;
    tNmtMnuNodeCmd      nodeCmd;
    tEplEvent           event;

    if ((nodeId_p == 0) || (nodeId_p >= EPL_C_ADR_BROADCAST))
        return kEplInvalidNodeId;

    nodeCmd.nodeCommand = nodeCommand_p;
    nodeCmd.nodeId = nodeId_p;
    event.m_EventSink = kEplEventSinkNmtMnu;
    event.m_EventType = kEplEventTypeNmtMnuNodeCmd;
    EPL_MEMSET(&event.m_NetTime, 0x00, sizeof(event.m_NetTime));
    event.m_pArg = &nodeCmd;
    event.m_uiSize = sizeof (nodeCmd);
    ret = eventu_postEvent(&event);

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Callback function for NMT state changes

The function implements the callback function for NMT state changes

\param  nmtStateChange_p    The received NMT state change event.

\return The function returns a tEplKernel error code.

\ingroup module_nmtmnu
*/
//------------------------------------------------------------------------------
tEplKernel nmtmnu_cbNmtStateChange(tEventNmtStateChange nmtStateChange_p)
{
    tEplKernel      ret = kEplSuccessful;
    UINT8           newMnNmtState;

    // Save new MN state in object 0x1F8E
    newMnNmtState   = (UINT8) nmtStateChange_p.newNmtState;
    ret = obd_writeEntry(0x1F8E, 240, &newMnNmtState, 1);
    if(ret != kEplSuccessful)
        return  ret;

    // do work which must be done in that state
    switch (nmtStateChange_p.newNmtState)
    {
        // build the configuration with infos from OD
        case kNmtGsResetConfiguration:
            {
                UINT32          dwTimeout;
                tObdSize        obdSize;

                // read object 0x1F80 NMT_StartUp_U32
                obdSize = 4;
                ret = obd_readEntry(0x1F80, 0, &nmtMnuInstance_g.nmtStartup, &obdSize);
                if (ret != kEplSuccessful)
                    break;

                // compute StatusReqDelay = object 0x1006 * EPL_C_NMT_STATREQ_CYCLE
                obdSize = sizeof (dwTimeout);
                ret = obd_readEntry(0x1006, 0, &dwTimeout, &obdSize);
                if (ret != kEplSuccessful)
                    break;

                if (dwTimeout != 0L)
                {
                    nmtMnuInstance_g.statusRequestDelay = dwTimeout * EPL_C_NMT_STATREQ_CYCLE / 1000L;
                    if (nmtMnuInstance_g.statusRequestDelay == 0L)
                    {
                        nmtMnuInstance_g.statusRequestDelay = 1L;    // at least 1 ms
                    }

                    // $$$ fetch and use MultiplexedCycleCount from OD
                    nmtMnuInstance_g.timeoutCheckCom = dwTimeout * EPL_C_NMT_STATREQ_CYCLE / 1000L;
                    if (nmtMnuInstance_g.timeoutCheckCom == 0L)
                    {
                        nmtMnuInstance_g.timeoutCheckCom = 1L;    // at least 1 ms
                    }
                }

                // fetch MNTimeoutPreOp2_U32 from OD
                obdSize = sizeof (dwTimeout);
                ret = obd_readEntry(0x1F89, 4, &dwTimeout, &obdSize);
                if (ret != kEplSuccessful)
                    break;

                if (dwTimeout != 0L)
                {
                    // convert [us] to [ms]
                    dwTimeout /= 1000L;
                    if (dwTimeout == 0L)
                    {
                        dwTimeout = 1L;    // at least 1 ms
                    }
                    nmtMnuInstance_g.timeoutReadyToOp = dwTimeout;
                }
                else
                {
                    nmtMnuInstance_g.timeoutReadyToOp = 0L;
                }
            }
            break;

        //-----------------------------------------------------------
        // MN part of the state machine

        // node listens for EPL-Frames and check timeout
        case kNmtMsNotActive:
            break;

        // node processes only async frames
        case kNmtMsPreOperational1:
            ret = doPreop1(nmtStateChange_p);
            break;

        // node processes isochronous and asynchronous frames
        case kNmtMsPreOperational2:
            ret = startBootStep2();
            // wait for NMT state change of CNs
            break;

        // node should be configured and application is ready
        case kNmtMsReadyToOperate:
            // check if PRes of CNs are OK
            // d.k. that means wait CycleLength * MultiplexCycleCount (i.e. start timer)
            //      because Dllk checks PRes of CNs automatically in ReadyToOp
            ret = startCheckCom();
            break;

        // normal work state
        case kNmtMsOperational:
            // send StartNode to CNs
            // wait for NMT state change of CNs
            ret = startNodes();
            break;

        // no EPL cycle
        // -> normal ethernet communication
        case kNmtMsBasicEthernet:
            break;

        default:
            break;
    }
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Callback function for NMT event checks

The function implements the callback function for NMT event checks. It
checks events before they are actually executed. The openPOWERLINK API layer
must forward NMT events from NmtCnu module.

This module will reject some NMT commands while MN.

\param  nmtEvent_p      The received NMT event.

\return The function returns a tEplKernel error code.

\ingroup module_nmtmnu
*/
//------------------------------------------------------------------------------
tEplKernel nmtmnu_cbCheckEvent(tNmtEvent nmtEvent_p)
{
    tEplKernel      ret = kEplSuccessful;
    UNUSED_PARAMETER(nmtEvent_p);
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Callback function for NMT events

The function implements the callback function for NMT events.

\param  pEvent_p            Pointer to the received NMT event.

\return The function returns a tEplKernel error code.

\ingroup module_nmtmnu
*/
//------------------------------------------------------------------------------
tEplKernel nmtmnu_processEvent(tEplEvent* pEvent_p)
{
    tEplKernel      ret = kEplSuccessful;

    // process event
    switch(pEvent_p->m_EventType)
    {
        // timer event
        case kEplEventTypeTimer:
            {
                tEplTimerEventArg*  pTimerEventArg = (tEplTimerEventArg*)pEvent_p->m_pArg;
                UINT                nodeId;

                nodeId = (UINT) (pTimerEventArg->m_Arg.m_dwVal & NMTMNU_TIMERARG_NODE_MASK);
                if (nodeId != 0)
                {
                    tObdSize             ObdSize;
                    UINT8                bNmtState;
                    tNmtMnuNodeInfo* pNodeInfo;

                    pNodeInfo = NMTMNU_GET_NODEINFO(nodeId);
                    ObdSize = 1;
                    ret = obd_readEntry(0x1F8E, nodeId, &bNmtState, &ObdSize);
                    if (ret != kEplSuccessful)
                        break;

                    if ((pTimerEventArg->m_Arg.m_dwVal & NMTMNU_TIMERARG_IDENTREQ) != 0L)
                    {
                        if ((UINT32)(pNodeInfo->flags & NMTMNU_NODE_FLAG_COUNT_STATREQ) !=
                            (pTimerEventArg->m_Arg.m_dwVal & NMTMNU_TIMERARG_COUNT_SR))
                        {   // this is an old (already deleted or modified) timer
                            // but not the current timer
                            // so discard it
                            NMTMNU_DBG_POST_TRACE_VALUE(kNmtMnuIntNodeEventTimerIdentReq,
                                                        nodeId, ((pNodeInfo->nodeState << 8) | 0xFF));

                            break;
                        }
                        /*NMTMNU_DBG_POST_TRACE_VALUE(kNmtMnuIntNodeEventTimerIdentReq, uiNodeId,
                                                        ((pNodeInfo->nodeState << 8) | 0x80
                                                         | ((pNodeInfo->flags & NMTMNU_NODE_FLAG_COUNT_STATREQ) >> 6)
                                                         | ((pTimerEventArg->m_Arg.m_dwVal & NMTMNU_TIMERARG_COUNT_SR) >> 8)));*/
                        ret = processInternalEvent(nodeId, (tNmtState) (bNmtState | NMT_TYPE_CS),
                                                   EPL_E_NO_ERROR, kNmtMnuIntNodeEventTimerIdentReq);
                    }

                    else if ((pTimerEventArg->m_Arg.m_dwVal & NMTMNU_TIMERARG_STATREQ) != 0L)
                    {
                        if ((UINT32)(pNodeInfo->flags & NMTMNU_NODE_FLAG_COUNT_STATREQ)
                            != (pTimerEventArg->m_Arg.m_dwVal & NMTMNU_TIMERARG_COUNT_SR))
                        {   // this is an old (already deleted or modified) timer
                            // but not the current timer
                            // so discard it
                            NMTMNU_DBG_POST_TRACE_VALUE(kNmtMnuIntNodeEventTimerStatReq,
                                                            nodeId, ((pNodeInfo->nodeState << 8) | 0xFF));

                            break;
                        }
                        /* NMTMNU_DBG_POST_TRACE_VALUE(kNmtMnuIntNodeEventTimerStatReq, uiNodeId,
                                                        ((pNodeInfo->nodeState << 8) | 0x80
                                                         | ((pNodeInfo->flags & NMTMNU_NODE_FLAG_COUNT_STATREQ) >> 6)
                                                         | ((pTimerEventArg->m_Arg.m_dwVal & NMTMNU_TIMERARG_COUNT_SR) >> 8))); */
                        ret = processInternalEvent(nodeId, (tNmtState) (bNmtState | NMT_TYPE_CS),
                                                   EPL_E_NO_ERROR, kNmtMnuIntNodeEventTimerStatReq);
                    }

                    else if ((pTimerEventArg->m_Arg.m_dwVal & NMTMNU_TIMERARG_STATE_MON) != 0L)
                    {
                        if ((UINT32)(pNodeInfo->flags & NMTMNU_NODE_FLAG_COUNT_STATREQ)
                            != (pTimerEventArg->m_Arg.m_dwVal & NMTMNU_TIMERARG_COUNT_SR))
                        {   // this is an old (already deleted or modified) timer
                            // but not the current timer
                            // so discard it
                            NMTMNU_DBG_POST_TRACE_VALUE(kNmtMnuIntNodeEventTimerStateMon,
                                                            nodeId, ((pNodeInfo->nodeState << 8) | 0xFF));

                            break;
                        }
                        /* NMTMNU_DBG_POST_TRACE_VALUE(kNmtMnuIntNodeEventTimerStatReq, uiNodeId,
                                                        ((pNodeInfo->nodeState << 8) | 0x80
                                                         | ((pNodeInfo->flags & NMTMNU_NODE_FLAG_COUNT_STATREQ) >> 6)
                                                         | ((pTimerEventArg->m_Arg.m_dwVal & NMTMNU_TIMERARG_COUNT_SR) >> 8))); */
                        ret = processInternalEvent(nodeId, (tNmtState) (bNmtState | NMT_TYPE_CS),
                                                   EPL_E_NO_ERROR, kNmtMnuIntNodeEventTimerStateMon);
                    }

                    else if ((pTimerEventArg->m_Arg.m_dwVal & NMTMNU_TIMERARG_LONGER) != 0L)
                    {
                        if ((UINT32)(pNodeInfo->flags & NMTMNU_NODE_FLAG_COUNT_LONGER)
                            != (pTimerEventArg->m_Arg.m_dwVal & NMTMNU_TIMERARG_COUNT_LO))
                        {   // this is an old (already deleted or modified) timer
                            // but not the current timer
                            // so discard it
                            NMTMNU_DBG_POST_TRACE_VALUE(kNmtMnuIntNodeEventTimerLonger, nodeId,
                                                            ((pNodeInfo->nodeState << 8) | 0xFF));

                            break;
                        }
                        /* NMTMNU_DBG_POST_TRACE_VALUE(kNmtMnuIntNodeEventTimerLonger, uiNodeId,
                                                        ((pNodeInfo->nodeState << 8) | 0x80
                                                         | ((pNodeInfo->flags & NMTMNU_NODE_FLAG_COUNT_LONGER) >> 6)
                                                         | ((pTimerEventArg->m_Arg.m_dwVal & NMTMNU_TIMERARG_COUNT_LO) >> 8))); */
                        ret = processInternalEvent(nodeId, (tNmtState) (bNmtState | NMT_TYPE_CS),
                                                   EPL_E_NO_ERROR, kNmtMnuIntNodeEventTimerLonger);
                    }

                }
                else
                {   // global timer event
                }
            }
            break;

        case kEplEventTypeHeartbeat:
            {
                tHeartbeatEvent* pHeartbeatEvent = (tHeartbeatEvent*)pEvent_p->m_pArg;
                ret = processInternalEvent(pHeartbeatEvent->nodeId, pHeartbeatEvent->nmtState,
                                           pHeartbeatEvent->errorCode, kNmtMnuIntNodeEventHeartbeat);
            }
            break;

        case kEplEventTypeNmtMnuNmtCmdSent:
            {
                tEplFrame* pFrame = (tEplFrame*)pEvent_p->m_pArg;
                UINT                uiNodeId;
                tNmtCommand         NmtCommand;
                UINT8                bNmtState;

                if (pEvent_p->m_uiSize < EPL_C_DLL_MINSIZE_NMTCMD)
                {
                    ret = eventu_postError(kEplEventSourceNmtMnu, kEplNmtInvalidFramePointer, sizeof (pEvent_p->m_uiSize), &pEvent_p->m_uiSize);
                    break;
                }

                uiNodeId = AmiGetByteFromLe(&pFrame->m_le_bDstNodeId);
                NmtCommand = (tNmtCommand) AmiGetByteFromLe(&pFrame->m_Data.m_Asnd.m_Payload.m_NmtCommandService.m_le_bNmtCommandId);

                switch (NmtCommand)
                {
                    case kNmtCmdStartNode:
                        bNmtState = (UINT8) (kNmtCsOperational & 0xFF);
                        break;

                    case kNmtCmdStopNode:
                        bNmtState = (UINT8) (kNmtCsStopped & 0xFF);
                        break;

                    case kNmtCmdEnterPreOperational2:
                        bNmtState = (UINT8) (kNmtCsPreOperational2 & 0xFF);
                        break;

                    case kNmtCmdEnableReadyToOperate:
                        // d.k. do not change expected node state, because of DS 1.0.0 7.3.1.2.1 Plain NMT State Command
                        //      and because node may not change NMT state within EPL_C_NMT_STATE_TOLERANCE
                        bNmtState = (UINT8) (kNmtCsPreOperational2 & 0xFF);
                        break;

                    case kNmtCmdResetNode:
                    case kNmtCmdResetCommunication:
                    case kNmtCmdResetConfiguration:
                    case kNmtCmdSwReset:
                        bNmtState = (UINT8) (kNmtCsNotActive & 0xFF);
                        // processInternalEvent() sets internal node state to kNmtMnuNodeStateUnknown
                        // after next unresponded IdentRequest/StatusRequest
                        break;

                    default:
                        goto Exit;
                }

                // process as internal event which update expected NMT state in OD
                if (uiNodeId != EPL_C_ADR_BROADCAST)
                {
                    ret = processInternalEvent(uiNodeId,
                                                        (tNmtState) (bNmtState | NMT_TYPE_CS),
                                                        0,
                                                        kNmtMnuIntNodeEventNmtCmdSent);

                }
                else
                {   // process internal event for all active nodes (except myself)
                    for (uiNodeId = 1; uiNodeId <= tabentries(nmtMnuInstance_g.aNodeInfo); uiNodeId++)
                    {
                        if ((NMTMNU_GET_NODEINFO(uiNodeId)->nodeCfg & (EPL_NODEASSIGN_NODE_IS_CN | EPL_NODEASSIGN_NODE_EXISTS)) != 0)
                        {
                            ret = processInternalEvent(uiNodeId, (tNmtState) (bNmtState | NMT_TYPE_CS),
                                                       0, kNmtMnuIntNodeEventNmtCmdSent);
                            if (ret != kEplSuccessful)
                                goto Exit;
                        }
                    }

                    if ((nmtMnuInstance_g.flags & NMTMNU_FLAG_USER_RESET) != 0)
                    {   // user or diagnostic nodes requests a reset of the MN
                        tNmtEvent    NmtEvent;

                        switch (NmtCommand)
                        {
                            case kNmtCmdResetNode:
                                NmtEvent = kNmtEventResetNode;
                                break;

                            case kNmtCmdResetCommunication:
                                NmtEvent = kNmtEventResetCom;
                                break;

                            case kNmtCmdResetConfiguration:
                                NmtEvent = kNmtEventResetConfig;
                                break;

                            case kNmtCmdSwReset:
                                NmtEvent = kNmtEventSwReset;
                                break;

                            case kNmtCmdInvalidService:
                            default:    // actually no reset was requested
                                goto Exit;
                        }
                        ret = nmtu_postNmtEvent(NmtEvent);
                        if (ret != kEplSuccessful)
                            goto Exit;
                    }
                }
            }
            break;

        case kEplEventTypeNmtMnuNodeCmd:
            {
                tNmtMnuNodeCmd*      pNodeCmd = (tNmtMnuNodeCmd*)pEvent_p->m_pArg;
                tNmtMnuIntNodeEvent  NodeEvent;
                tObdSize             ObdSize;
                UINT8                    bNmtState;
                UINT16                    wErrorCode = EPL_E_NO_ERROR;

                if ((pNodeCmd->nodeId == 0) || (pNodeCmd->nodeId >= EPL_C_ADR_BROADCAST))
                {
                    ret = kEplInvalidNodeId;
                    goto Exit;
                }

                switch (pNodeCmd->nodeCommand)
                {
                    case kNmtNodeCommandBoot:
                        NodeEvent = kNmtMnuIntNodeEventBoot;
                        break;

                    case kNmtNodeCommandConfOk:
                        NodeEvent = kNmtMnuIntNodeEventConfigured;
                        break;

                    case kNmtNodeCommandConfErr:
                        NodeEvent = kNmtMnuIntNodeEventError;
                        wErrorCode = EPL_E_NMT_BPO1_CF_VERIFY;
                        break;

                    case kNmtNodeCommandConfRestored:
                        NodeEvent = kNmtMnuIntNodeEventExecResetNode;
                        break;

                    case kNmtNodeCommandConfReset:
                        NodeEvent = kNmtMnuIntNodeEventExecResetConf;
                        break;

                    default:
                       // invalid node command
                        goto Exit;
                }

                // fetch current NMT state
                ObdSize = sizeof (bNmtState);
                ret = obd_readEntry(0x1F8E, pNodeCmd->nodeId, &bNmtState, &ObdSize);
                if (ret != kEplSuccessful)
                    goto Exit;

                ret = processInternalEvent(pNodeCmd->nodeId,
                                                    (tNmtState) (bNmtState | NMT_TYPE_CS),
                                                    wErrorCode,
                                                    NodeEvent);
            }
            break;

        case kEplEventTypeNmtMnuNodeAdded:
            {
                UINT        nodeId;
                nodeId = *((UINT*) pEvent_p->m_pArg);
                ret = cbNodeAdded(nodeId);
            }
            break;

        default:
            ret = kEplNmtInvalidEvent;
            break;
    }

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Get diagnostic info

The function returns diagnostic information.

\param  pMandatorySlaveCount_p  Pointer to store mandatory slave count.
\param  pSignalSlaveCount_p     Pointer to store signal slave count.
\param  pFlags_p                Pointer to store global flags.

\return The function returns a tEplKernel error code.

\ingroup module_nmtmnu
*/
//------------------------------------------------------------------------------
tEplKernel nmtmnu_getDiagnosticInfo(UINT* pMandatorySlaveCount_p,
                                    UINT* pSignalSlaveCount_p, UINT16* pFlags_p)
{
    if ((pMandatorySlaveCount_p == NULL) || (pSignalSlaveCount_p == NULL) || (pFlags_p == NULL))
        return kEplNmtInvalidParam;

    *pMandatorySlaveCount_p = nmtMnuInstance_g.mandatorySlaveCount;
    *pSignalSlaveCount_p = nmtMnuInstance_g.signalSlaveCount;
    *pFlags_p = nmtMnuInstance_g.flags;

    return kEplSuccessful;
}

#if EPL_NMTMNU_PRES_CHAINING_MN != FALSE
//------------------------------------------------------------------------------
/**
\brief  Configure PRes chaining parameters

The function configures the PRes chaining parameters

\param  pConfigParam_p          PRes chaining parameters.

\return The function returns a tEplKernel error code.

\ingroup module_nmtmnu
*/
//------------------------------------------------------------------------------
tEplKernel nmtmnu_configPrc(tEplNmtMnuConfigParam* pConfigParam_p)
{
    nmtMnuInstance_g.prcPResTimeFirstCorrectionNs = pConfigParam_p->prcPResTimeFirstCorrectionNs;
    nmtMnuInstance_g.prcPResTimeFirstNegOffsetNs = pConfigParam_p->prcPResTimeFirstNegOffsetNs;
    return kEplSuccessful;
}
#endif

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief  Callback function for NMT requests

The function implements the callback function for NMT requests.

\param  pFrameInfo_p        Pointer to NMT request frame information.

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
static tEplKernel cbNmtRequest(tFrameInfo * pFrameInfo_p)
{
    tEplKernel              ret = kEplSuccessful;
    UINT                    targetNodeId;
    tNmtCommand             nmtCommand;
    tEplNmtRequestService*  pNmtRequestService;
    UINT                    sourceNodeId;

    if ((pFrameInfo_p == NULL) || (pFrameInfo_p->pFrame == NULL))
        return kEplNmtInvalidFramePointer;

    pNmtRequestService = &pFrameInfo_p->pFrame->m_Data.m_Asnd.m_Payload.m_NmtRequestService;
    nmtCommand = (tNmtCommand)AmiGetByteFromLe(&pNmtRequestService->m_le_bNmtCommandId);
    targetNodeId = AmiGetByteFromLe(&pNmtRequestService->m_le_bTargetNodeId);
    ret = nmtmnu_requestNmtCommand(targetNodeId, nmtCommand);
    if (ret != kEplSuccessful)
    {   // error -> reply with kNmtCmdInvalidService
        sourceNodeId = AmiGetByteFromLe(&pFrameInfo_p->pFrame->m_le_bSrcNodeId);
        ret = nmtmnu_sendNmtCommand(sourceNodeId, kNmtCmdInvalidService);
    }
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Callback function for Ident responses

The function implements the callback function for Ident responses

\param  nodeId_p            Node ID for which IdentResponse was received.
\param  pIdentResponse_p    Pointer to IdentResponse. It is NULL if node did
                            not answer.

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
static tEplKernel PUBLIC cbIdentResponse(UINT nodeId_p, tEplIdentResponse* pIdentResponse_p)
{
    tEplKernel      ret = kEplSuccessful;
    tObdSize        obdSize;
    UINT32          dwDevType;
    UINT16          errorCode;
    tNmtState       nmtState;

    if (pIdentResponse_p == NULL)
    {   // node did not answer
        ret = processInternalEvent(nodeId_p, kNmtCsNotActive, EPL_E_NMT_NO_IDENT_RES, // was EPL_E_NO_ERROR
                                   kNmtMnuIntNodeEventNoIdentResponse);
    }
    else
    {   // node answered IdentRequest
        errorCode = EPL_E_NO_ERROR;
        nmtState = (tNmtState)(AmiGetByteFromLe(&pIdentResponse_p->m_le_bNmtStatus) | NMT_TYPE_CS);

        // check IdentResponse $$$ move to ProcessIntern, because this function may be called also if CN

        // check DeviceType (0x1F84)
        obdSize = 4;
        ret = obd_readEntry(0x1F84, nodeId_p, &dwDevType, &obdSize);
        if (ret != kEplSuccessful)
            goto Exit;

        if (dwDevType != 0L)
        {   // actually compare it with DeviceType from IdentResponse
            if (AmiGetDwordFromLe(&pIdentResponse_p->m_le_dwDeviceType) != dwDevType)
            {   // wrong DeviceType
                nmtState = kNmtCsNotActive;
                errorCode = EPL_E_NMT_BPO1_DEVICE_TYPE;
            }
        }
        ret = processInternalEvent(nodeId_p, nmtState, errorCode,
                                   kNmtMnuIntNodeEventIdentResponse);
    }
Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Callback function for Status responses

The function implements the callback function for Status responses

\param  nodeId_p            Node ID for which StatusResponse was received.
\param  pStatusResponse_p   Pointer to StatusResponse. It is NULL if node did
                            not answer.

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
static tEplKernel PUBLIC cbStatusResponse(UINT nodeId_p, tEplStatusResponse* pStatusResponse_p)
{
    tEplKernel      ret = kEplSuccessful;

    if (pStatusResponse_p == NULL)
    {   // node did not answer
        ret = processInternalEvent(nodeId_p, kNmtCsNotActive, EPL_E_NMT_NO_STATUS_RES, // was EPL_E_NO_ERROR
                                   kNmtMnuIntNodeEventNoStatusResponse);
    }
    else
    {   // node answered StatusRequest
        ret = processInternalEvent(nodeId_p,
                                   (tNmtState)(AmiGetByteFromLe(&pStatusResponse_p->m_le_bNmtStatus) | NMT_TYPE_CS),
                                   EPL_E_NO_ERROR, kNmtMnuIntNodeEventStatusResponse);
    }
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Callback function for added node events

The function implements the callback function for added node events. It is
called after the addressed node has been added in module dllk.

\param  nodeId_p            Node ID for which the event was received.

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
static tEplKernel cbNodeAdded(UINT nodeId_p)
{
    tEplKernel          ret = kEplSuccessful;
    tNmtMnuNodeInfo*    pNodeInfo;
    tNmtState           nmtState;

    pNodeInfo = NMTMNU_GET_NODEINFO(nodeId_p);
    pNodeInfo->flags |= NMTMNU_NODE_FLAG_ISOCHRON;

    if (pNodeInfo->nodeState == kNmtMnuNodeStateConfigured)
    {
        nmtState = nmtu_getNmtState();
        if (nmtState >= kNmtMsPreOperational2)
        {
            ret = nodeBootStep2(nodeId_p, pNodeInfo);
        }
    }
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Add node into isochronous phase

The function adds the specified node into the isochronous phase

\param  nodeId_p            Node ID which will be added.

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
static tEplKernel addNodeIsochronous(UINT nodeId_p)
{
    tEplKernel          ret;

#if EPL_NMTMNU_PRES_CHAINING_MN != FALSE
    tNmtMnuNodeInfo* pNodeInfo;

    ret = kEplSuccessful;

    if (nodeId_p != EPL_C_ADR_INVALID)
    {
        pNodeInfo = NMTMNU_GET_NODEINFO(nodeId_p);
        if (pNodeInfo == NULL)
        {
            ret = kEplInvalidNodeId;
            goto Exit;
        }

        // clear PRC specific values
        pNodeInfo->prcFlags = 0;
        pNodeInfo->pResTimeFirstNs = 0;
        pNodeInfo->relPropagationDelayNs = 0;

        if ((pNodeInfo->nodeCfg & EPL_NODEASSIGN_PRES_CHAINING) == 0)
#endif

        {   // node is added as PReq/PRes node
        tDllNodeOpParam     NodeOpParam;

            NodeOpParam.opNodeType = kDllNodeOpTypeIsochronous;
            NodeOpParam.nodeId = nodeId_p;
            ret = dllucal_addNode(&NodeOpParam);
            goto Exit;
        }
#if EPL_NMTMNU_PRES_CHAINING_MN != FALSE
        else
        {   // node is a PRC node

            nmtMnuInstance_g.flags |= NMTMNU_FLAG_PRC_ADD_SCHEDULED;
            pNodeInfo->prcFlags |= NMTMNU_NODE_FLAG_PRC_ADD_SCHEDULED;
        }
    }

    if (nmtMnuInstance_g.flags & NMTMNU_FLAG_PRC_ADD_IN_PROGRESS)
    {   // add PRC nodes is already in progress
        goto Exit;
    }

    if (nmtMnuInstance_g.flags & NMTMNU_FLAG_PRC_ADD_SCHEDULED)
    {
        UINT            nodeId;
        BOOL            fInvalidateNext;

        fInvalidateNext = FALSE;
        for (nodeId = 1; nodeId < 254; nodeId++)
        {
            pNodeInfo = NMTMNU_GET_NODEINFO(nodeId);
            if (pNodeInfo == NULL)
                continue;

            // $$$ only PRC

            if (pNodeInfo->flags & NMTMNU_NODE_FLAG_ISOCHRON)
            {
                if (fInvalidateNext != FALSE)
                {
                    // set relative propagation delay to invalid
                    pNodeInfo->relPropagationDelayNs = 0;
                    fInvalidateNext = FALSE;
                }
            }
            else if (pNodeInfo->prcFlags & NMTMNU_NODE_FLAG_PRC_ADD_SCHEDULED)
            {
                pNodeInfo->prcFlags &= ~NMTMNU_NODE_FLAG_PRC_ADD_SCHEDULED;
                pNodeInfo->prcFlags |=  NMTMNU_NODE_FLAG_PRC_ADD_IN_PROGRESS;
                nmtMnuInstance_g.flags |= NMTMNU_FLAG_PRC_ADD_IN_PROGRESS;

                // set relative propagation delay to invalid
                pNodeInfo->relPropagationDelayNs = 0;
                fInvalidateNext = TRUE;
            }
            // $$$ else if: falls es noch einen ADD_IN_PROGRESS gibt, Fehler
        }

        nmtMnuInstance_g.flags &= ~NMTMNU_FLAG_PRC_ADD_SCHEDULED;

        if (nmtMnuInstance_g.flags & NMTMNU_FLAG_PRC_ADD_IN_PROGRESS)
        {
            ret = prcMeasure();
        }
    }
#endif

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Start BootStep1

The function starts the BootStep1.

\param  fNmtResetAllIssued_p    Determines if all nodes should be reset.

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
static tEplKernel startBootStep1(BOOL fNmtResetAllIssued_p)
{
    tEplKernel            ret = kEplSuccessful;
    UINT                subIndex;
    UINT                localNodeId;
    UINT32                nodeCfg;
    tObdSize            obdSize;
    tNmtMnuNodeInfo*    pNodeInfo;

    // $$$ d.k.: save current time for 0x1F89/2 MNTimeoutPreOp1_U32

    // start network scan
    nmtMnuInstance_g.mandatorySlaveCount = 0;
    nmtMnuInstance_g.signalSlaveCount = 0;
    // check 0x1F81
    localNodeId = obd_getNodeId();
    for (subIndex = 1; subIndex <= 254; subIndex++)
    {
        obdSize = 4;
        ret = obd_readEntry(0x1F81, subIndex, &nodeCfg, &obdSize);
        if (ret != kEplSuccessful)
            goto Exit;

        if (subIndex != localNodeId)
        {
            pNodeInfo = NMTMNU_GET_NODEINFO(subIndex);

            // reset flags "not scanned" and "isochronous"
            pNodeInfo->flags &= ~(NMTMNU_NODE_FLAG_ISOCHRON | NMTMNU_NODE_FLAG_NOT_SCANNED);

#if EPL_NMTMNU_PRES_CHAINING_MN != FALSE
            // Reset all PRC flags and PRC related values
            pNodeInfo->prcFlags = 0;
            pNodeInfo->pResTimeFirstNs = 0;
            pNodeInfo->relPropagationDelayNs = 0;
#endif

            if (subIndex == EPL_C_ADR_DIAG_DEF_NODE_ID)
            {   // diagnostic node must be scanned by MN in any case
                nodeCfg |= (EPL_NODEASSIGN_NODE_IS_CN | EPL_NODEASSIGN_NODE_EXISTS);
                // and it must be isochronously accessed
                nodeCfg &= ~EPL_NODEASSIGN_ASYNCONLY_NODE;
            }

            // save node config in local node info structure
            pNodeInfo->nodeCfg = nodeCfg;
            pNodeInfo->nodeState = kNmtMnuNodeStateUnknown;

            if ((nodeCfg & (EPL_NODEASSIGN_NODE_IS_CN | EPL_NODEASSIGN_NODE_EXISTS)) != 0)
            {   // node is configured as CN
                if (fNmtResetAllIssued_p == FALSE)
                {
                    // identify the node
                    ret = identu_requestIdentResponse(subIndex, cbIdentResponse);
                    if (ret != kEplSuccessful)
                        goto Exit;
                }

                // set flag "not scanned"
                pNodeInfo->flags |= NMTMNU_NODE_FLAG_NOT_SCANNED;
                nmtMnuInstance_g.signalSlaveCount++;
                // signal slave counter shall be decremented if IdentRequest was sent once to a CN

                if ((nodeCfg & EPL_NODEASSIGN_MANDATORY_CN) != 0)
                {   // node is a mandatory CN
                    nmtMnuInstance_g.mandatorySlaveCount++;
                    // mandatory slave counter shall be decremented if mandatory CN was configured successfully
                }
            }
        }
        else
        {   // subindex of MN
            if ((nodeCfg & (EPL_NODEASSIGN_MN_PRES | EPL_NODEASSIGN_NODE_EXISTS)) != 0)
            {   // MN shall send PRes
                ret = addNodeIsochronous(localNodeId);
            }
        }
    }

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Handle MN PreOperational1 State

The function handles the PreOperational1 state of the MN.

\param  nmtStateChange_p            The received NMT state change event.

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
static tEplKernel doPreop1(tEventNmtStateChange nmtStateChange_p)
{
    UINT32          dwTimeout;
    tEplTimerArg    timerArg;
    tObdSize        obdSize;
    tEplEvent       event;
    BOOL            fNmtResetAllIssued = FALSE;
    tEplKernel      ret = kEplSuccessful;

    // reset IdentResponses and running IdentRequests and StatusRequests
    ret = identu_reset();
    ret = statusu_reset();
#if EPL_NMTMNU_PRES_CHAINING_MN != FALSE
    ret = syncu_reset();
#endif

    // reset timers
    ret = reset();

    // 2008/11/18 d.k. reset internal node info is not necessary,
    //                 because timer flags are important and other
    //                 things are reset by startBootStep1().

    // inform DLL about NMT state change,
    // so that it can clear the asynchronous queues and start the reduced cycle
    event.m_EventSink = kEplEventSinkDllk;
    event.m_EventType = kEplEventTypeDllkStartReducedCycle;
    EPL_MEMSET(&event.m_NetTime, 0x00, sizeof(event.m_NetTime));
    event.m_pArg = NULL;
    event.m_uiSize = 0;
    ret = eventu_postEvent(&event);
    if (ret != kEplSuccessful)
        return ret;

    // reset all nodes
    // skip this step if we come directly from OPERATIONAL
    // or it was just done before, e.g. because of a ResetNode command
    // from a diagnostic node
    if ((nmtStateChange_p.nmtEvent == kNmtEventTimerMsPreOp1) ||
        ((nmtMnuInstance_g.flags & NMTMNU_FLAG_USER_RESET) == 0))
    {
        BENCHMARK_MOD_07_TOGGLE(7);
        NMTMNU_DBG_POST_TRACE_VALUE(0, EPL_C_ADR_BROADCAST, kNmtCmdResetNode);

        ret = nmtmnu_sendNmtCommand(EPL_C_ADR_BROADCAST, kNmtCmdResetNode);
        if (ret != kEplSuccessful)
            return ret;

        fNmtResetAllIssued = TRUE;
    }

    // clear global flags, e.g. reenable boot process
    nmtMnuInstance_g.flags = 0;

    // start network scan
    ret = startBootStep1(fNmtResetAllIssued);

    // start timer for 0x1F89/2 MNTimeoutPreOp1_U32
    obdSize = sizeof (dwTimeout);
    ret = obd_readEntry(0x1F89, 2, &dwTimeout, &obdSize);
    if (ret != kEplSuccessful)
        return ret;

    if (dwTimeout != 0L)
    {
        dwTimeout /= 1000L;
        if (dwTimeout == 0L)
        {
            dwTimeout = 1L; // at least 1 ms
        }
        timerArg.m_EventSink = kEplEventSinkNmtMnu;
        timerArg.m_Arg.m_dwVal = 0;
        ret = timeru_modifyTimer(&nmtMnuInstance_g.timerHdlNmtState, dwTimeout, timerArg);
    }
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Start BootStep2

The function starts the BootStep2. This means checking if a node has reached
PreOp2 and has been added to the isochronous phase. If this is met, the
NMT command EnableReadyToOp is sent.

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
static tEplKernel startBootStep2(void)
{
    tEplKernel          ret = kEplSuccessful;
    UINT                index;
    tNmtMnuNodeInfo*    pNodeInfo;
    tObdSize            obdSize;
    UINT8               nmtState;
    tNmtState           expNmtState;

    if ((nmtMnuInstance_g.flags & NMTMNU_FLAG_HALTED) == 0)
    {   // boot process is not halted
        nmtMnuInstance_g.mandatorySlaveCount = 0;
        nmtMnuInstance_g.signalSlaveCount = 0;
        // reset flag that application was informed about possible state change
        nmtMnuInstance_g.flags &= ~NMTMNU_FLAG_APP_INFORMED;
    }

    pNodeInfo = nmtMnuInstance_g.aNodeInfo;
    for (index = 1; index <= tabentries(nmtMnuInstance_g.aNodeInfo); index++, pNodeInfo++)
    {
        obdSize = 1;
        // read object 0x1F8F NMT_MNNodeExpState_AU8
        ret = obd_readEntry(0x1F8F, index, &nmtState, &obdSize);
        if (ret != kEplSuccessful)
            goto Exit;

        // compute expected NMT state
        expNmtState = (tNmtState) (nmtState | NMT_TYPE_CS);

        if (expNmtState == kNmtCsPreOperational1)
        {
            tEplTimerArg    timerArg;

            // The change to PreOp2 is an implicit NMT command.
            // Unexpected NMT states of the nodes are ignored until
            // the state monitor timer is elapsed.
            NMTMNU_SET_FLAGS_TIMERARG_STATE_MON(pNodeInfo, index, timerArg);

            // set NMT state change flag
            pNodeInfo->flags |= NMTMNU_NODE_FLAG_NMT_CMD_ISSUED;

            ret = timeru_modifyTimer(&pNodeInfo->timerHdlStatReq,
                                         nmtMnuInstance_g.statusRequestDelay, timerArg);
            if (ret != kEplSuccessful)
                goto Exit;

            // update object 0x1F8F NMT_MNNodeExpState_AU8 to PreOp2
            nmtState = (UINT8)(kNmtCsPreOperational2 & 0xFF);
            ret = obd_writeEntry(0x1F8F, index, &nmtState, 1);
            if (ret != kEplSuccessful)
                goto Exit;

            if ((pNodeInfo->nodeState == kNmtMnuNodeStateConfigured) &&
                ((nmtMnuInstance_g.flags & NMTMNU_FLAG_HALTED) == 0))
            {   // boot process is not halted
                // set flag "not scanned"
                pNodeInfo->flags |= NMTMNU_NODE_FLAG_NOT_SCANNED;

                nmtMnuInstance_g.signalSlaveCount++;
                // signal slave counter shall be decremented if StatusRequest was sent once to a CN

                if ((pNodeInfo->nodeCfg & EPL_NODEASSIGN_MANDATORY_CN) != 0)
                {   // node is a mandatory CN
                    nmtMnuInstance_g.mandatorySlaveCount++;
                }
                // mandatory slave counter shall be decremented if mandatory CN is ReadyToOp
            }
        }
    }
Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Start BootStep2 for specified node

The function starts BootStep2 for the specified node. This means checking
whether the CN is in NMT state PreOp2 and whether it has been added to the
isochronous phase. If both checks pass, it gets the NMT command EnableReadyToOp.

The CN must be in node state Configured, when it enters BootStep2. When
BootStep2 finishes, the CN is in node state ReadyToOp. If TimeoutReadyToOp
in object 0x1F89/5 is configured, timerHdlLonger will be started with this
timeout.

\param  nodeId_p        Node ID for which to start BootStep2.
\param  pNodeInfo_p     Pointer to node info structure of node.

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
static tEplKernel nodeBootStep2(UINT nodeId_p, tNmtMnuNodeInfo* pNodeInfo_p)
{
    tEplKernel          ret = kEplSuccessful;
    tEplTimerArg        timerArg;
    UINT8               bNmtState;
    tNmtState           nmtState;
    tObdSize            obdSize;

    if (pNodeInfo_p->nodeCfg & EPL_NODEASSIGN_ASYNCONLY_NODE)
    {   // node is async-only
        // read object 0x1F8E NMT_MNNodeCurrState_AU8
        obdSize = 1;
        ret = obd_readEntry(0x1F8E, nodeId_p, &bNmtState, &obdSize);
        if (ret != kEplSuccessful)
            goto Exit;

        nmtState = (tNmtState) (bNmtState | NMT_TYPE_CS);

        if (nmtState != kNmtCsPreOperational2)
            goto Exit;
    }
    else
    {   // node is not async-only

        // The check whether the node has been added to the isochronous phase
        // implicates the check for NMT state PreOp2
        if ((pNodeInfo_p->flags & NMTMNU_NODE_FLAG_ISOCHRON) == 0)
            goto Exit;
    }

    NMTMNU_DBG_POST_TRACE_VALUE(0, nodeId_p, kNmtCmdEnableReadyToOperate);

    ret = nmtmnu_sendNmtCommand(nodeId_p, kNmtCmdEnableReadyToOperate);
    if (ret != kEplSuccessful)
        goto Exit;

    if (nmtMnuInstance_g.timeoutReadyToOp != 0L)
    {   // start timer
        // when the timer expires the CN must be ReadyToOp
        NMTMNU_SET_FLAGS_TIMERARG_LONGER(pNodeInfo_p, nodeId_p, timerArg);
        ret = timeru_modifyTimer(&pNodeInfo_p->timerHdlLonger,
                                     nmtMnuInstance_g.timeoutReadyToOp, timerArg);
    }
Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Start CheckCommunication

The function starts CheckCommunication.

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
static tEplKernel startCheckCom(void)
{
    tEplKernel      ret = kEplSuccessful;
    UINT            index;
    tNmtMnuNodeInfo* pNodeInfo;

    if ((nmtMnuInstance_g.flags & NMTMNU_FLAG_HALTED) == 0)
    {   // boot process is not halted
        // wait some time and check that no communication error occurs
        nmtMnuInstance_g.mandatorySlaveCount = 0;
        nmtMnuInstance_g.signalSlaveCount = 0;
        // reset flag that application was informed about possible state change
        nmtMnuInstance_g.flags &= ~NMTMNU_FLAG_APP_INFORMED;

        pNodeInfo = nmtMnuInstance_g.aNodeInfo;
        for (index = 1; index <= tabentries(nmtMnuInstance_g.aNodeInfo); index++, pNodeInfo++)
        {
            if (pNodeInfo->nodeState == kNmtMnuNodeStateReadyToOp)
            {
                ret = nodeCheckCom(index, pNodeInfo);
                if (ret == kEplReject)
                {   // timer was started
                    // wait until it expires
                    if ((pNodeInfo->nodeCfg & EPL_NODEASSIGN_MANDATORY_CN) != 0)
                    {   // node is a mandatory CN
                        nmtMnuInstance_g.mandatorySlaveCount++;
                    }
                }
                else
                {
                    if (ret != kEplSuccessful)
                        goto Exit;
                }

                // set flag "not scanned"
                pNodeInfo->flags |= NMTMNU_NODE_FLAG_NOT_SCANNED;

                nmtMnuInstance_g.signalSlaveCount++;
                // signal slave counter shall be decremented if timeout elapsed and regardless of an error
                // mandatory slave counter shall be decremented if timeout elapsed and no error occurred
            }
        }
    }
    ret = kEplSuccessful;
Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Start CheckCommunication for the specified node

The function starts CheckCommunication for the specified node. That means it
waits some time and if no error occured everything is OK.

\param  nodeId_p        Node ID for which to start CheckCommunication.
\param  pNodeInfo_p     Pointer to node info structure of node.

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
static tEplKernel nodeCheckCom(UINT nodeId_p, tNmtMnuNodeInfo* pNodeInfo_p)
{
    tEplKernel      ret = kEplSuccessful;
    UINT32          nodeCfg;
    tEplTimerArg    timerArg;

    nodeCfg = pNodeInfo_p->nodeCfg;
    if (((nodeCfg & EPL_NODEASSIGN_ASYNCONLY_NODE) == 0) &&
        (nmtMnuInstance_g.timeoutCheckCom != 0L))
    {   // CN is not async-only and timeout for CheckCom was set

        // check communication,
        // that means wait some time and if no error occurred everything is OK;

        // start timer (when the timer expires the CN must be still ReadyToOp)
        NMTMNU_SET_FLAGS_TIMERARG_LONGER(pNodeInfo_p, nodeId_p, timerArg);
        ret = timeru_modifyTimer(&pNodeInfo_p->timerHdlLonger,
                                     nmtMnuInstance_g.timeoutCheckCom, timerArg);

        // update mandatory slave counter, because timer was started
        if (ret == kEplSuccessful)
            ret = kEplReject;
    }
    else
    {   // timer was not started
        // assume everything is OK
        pNodeInfo_p->nodeState = kNmtMnuNodeStateComChecked;
    }
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Start Nodes

The function starts all nodes which are ReadyToOp and CheckCom did not fail.

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
static tEplKernel startNodes(void)
{
    tEplKernel      ret = kEplSuccessful;
    UINT            index;
    tNmtMnuNodeInfo* pNodeInfo;

    if ((nmtMnuInstance_g.flags & NMTMNU_FLAG_HALTED) == 0)
    {   // boot process is not halted
        // send NMT command Start Node
        nmtMnuInstance_g.mandatorySlaveCount = 0;
        nmtMnuInstance_g.signalSlaveCount = 0;
        // reset flag that application was informed about possible state change
        nmtMnuInstance_g.flags &= ~NMTMNU_FLAG_APP_INFORMED;

        pNodeInfo = nmtMnuInstance_g.aNodeInfo;
        for (index = 1; index <= tabentries(nmtMnuInstance_g.aNodeInfo); index++, pNodeInfo++)
        {
            if (pNodeInfo->nodeState == kNmtMnuNodeStateComChecked)
            {
                if ((nmtMnuInstance_g.nmtStartup & EPL_NMTST_STARTALLNODES) == 0)
                {
                    NMTMNU_DBG_POST_TRACE_VALUE(0, index, kNmtCmdStartNode);
                    ret = nmtmnu_sendNmtCommand(index, kNmtCmdStartNode);
                    if (ret != kEplSuccessful)
                        goto Exit;
                }

                if ((pNodeInfo->nodeCfg & EPL_NODEASSIGN_MANDATORY_CN) != 0)
                {   // node is a mandatory CN
                    nmtMnuInstance_g.mandatorySlaveCount++;
                }

                // set flag "not scanned"
                pNodeInfo->flags |= NMTMNU_NODE_FLAG_NOT_SCANNED;

                nmtMnuInstance_g.signalSlaveCount++;
                // signal slave counter shall be decremented if StatusRequest was sent once to a CN
                // mandatory slave counter shall be decremented if mandatory CN is OPERATIONAL
            }
        }

        // $$$ inform application if EPL_NMTST_NO_STARTNODE is set

        if ((nmtMnuInstance_g.nmtStartup & EPL_NMTST_STARTALLNODES) != 0)
        {
            NMTMNU_DBG_POST_TRACE_VALUE(0, EPL_C_ADR_BROADCAST, kNmtCmdStartNode);
            ret = nmtmnu_sendNmtCommand(EPL_C_ADR_BROADCAST, kNmtCmdStartNode);
            if (ret != kEplSuccessful)
                goto Exit;
        }
    }
Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Process IdentResponse node event

The function processes the internal node event kNmtMnuIntNodeEventIdentResponse.

\param  nodeId_p            Node ID to process.
\param  nodeNmtState_p      NMT state of the node.
\param  nmtState_p          NMT state of the MN
\param  errorCode_p         Error codes.
\param  pRet_p              Pointer to store return value

\return The function returns 0 if the higher level event handler should continue
        processing or -1 if it should exit.
*/
//------------------------------------------------------------------------------
static INT processNodeEventIdentResponse(UINT nodeId_p, tNmtState nodeNmtState_p, tNmtState nmtState_p,
                                         UINT16 errorCode_p, tEplKernel* pRet_p)
{
    UINT8               bNmtState;
    tNmtMnuNodeInfo*    pNodeInfo;

    pNodeInfo = NMTMNU_GET_NODEINFO(nodeId_p);

    NMTMNU_DBG_POST_TRACE_VALUE(kNmtMnuIntNodeEventIdentResponse, nodeId_p, pNodeInfo->nodeState);

    if ((pNodeInfo->nodeState != kNmtMnuNodeStateResetConf) &&
        (pNodeInfo->nodeState != kNmtMnuNodeStateConfRestored))
    {
        pNodeInfo->nodeState = kNmtMnuNodeStateIdentified;
    }

    pNodeInfo->flags &= ~(NMTMNU_NODE_FLAG_ISOCHRON |
                          NMTMNU_NODE_FLAG_NMT_CMD_ISSUED |
                          NMTMNU_NODE_FLAG_PREOP2_REACHED);

    if (nmtState_p == kNmtMsPreOperational1)
    {
        if ((pNodeInfo->flags & NMTMNU_NODE_FLAG_NOT_SCANNED) != 0)
        {   // decrement only signal slave count
            nmtMnuInstance_g.signalSlaveCount--;
            pNodeInfo->flags &= ~NMTMNU_NODE_FLAG_NOT_SCANNED;
        }
        // update object 0x1F8F NMT_MNNodeExpState_AU8 to PreOp1
        bNmtState = (UINT8) (kNmtCsPreOperational1 & 0xFF);
    }
    else
    {   // MN is running full cycle
        // update object 0x1F8F NMT_MNNodeExpState_AU8 to PreOp2
        bNmtState = (UINT8) (kNmtCsPreOperational2 & 0xFF);
        if (nodeNmtState_p == kNmtCsPreOperational1)
        {   // The CN did not yet switch to PreOp2
            tEplTimerArg timerArg;

            // Set NMT state change flag and ignore unexpected NMT states
            // until the state monitor timer is elapsed.
            pNodeInfo->flags |= NMTMNU_NODE_FLAG_NMT_CMD_ISSUED;

            NMTMNU_SET_FLAGS_TIMERARG_STATE_MON(pNodeInfo, nodeId_p, timerArg);

            *pRet_p = timeru_modifyTimer(&pNodeInfo->timerHdlStatReq,
                                             nmtMnuInstance_g.statusRequestDelay, timerArg);
            if (*pRet_p != kEplSuccessful)
                return -1;
        }
    }
    *pRet_p = obd_writeEntry(0x1F8F, nodeId_p, &bNmtState, 1);
    if (*pRet_p != kEplSuccessful)
        return -1;

    // check NMT state of CN
    *pRet_p = checkNmtState(nodeId_p, pNodeInfo, nodeNmtState_p, errorCode_p, nmtState_p);
    if (*pRet_p != kEplSuccessful)
    {
        if (*pRet_p == kEplReject)
            *pRet_p = kEplSuccessful;
        return 0;
    }

    if ((pNodeInfo->flags & NMTMNU_NODE_FLAG_NMT_CMD_ISSUED) == 0)
    {   // No state monitor timer is required
        // Request StatusResponse immediately,
        // because we want a fast boot-up of CNs
        *pRet_p = statusu_requestStatusResponse(nodeId_p, cbStatusResponse);
        if (*pRet_p != kEplSuccessful)
        {
            NMTMNU_DBG_POST_TRACE_VALUE(kNmtMnuIntNodeEventIdentResponse, nodeId_p, *pRet_p);
            if (*pRet_p == kEplInvalidOperation)
            {   // the only situation when this should happen is, when
                // StatusResponse was already requested from within
                // the StatReq timer event.
                // so ignore this error.
                *pRet_p = kEplSuccessful;
            }
            else
            {
                return 0;
            }
        }
    }

    if ((pNodeInfo->nodeState != kNmtMnuNodeStateResetConf) &&
        (pNodeInfo->nodeState != kNmtMnuNodeStateConfRestored))
    {
        // inform application
        *pRet_p = nmtMnuInstance_g.pfnCbNodeEvent(nodeId_p, kNmtNodeEventFound,
                                              nodeNmtState_p, EPL_E_NO_ERROR,
                                              (pNodeInfo->nodeCfg & EPL_NODEASSIGN_MANDATORY_CN) != 0);
        if (*pRet_p == kEplReject)
        {   // interrupt boot process on user request
            NMTMNU_DBG_POST_TRACE_VALUE(kNmtMnuIntNodeEventIdentResponse, nodeId_p,
                                            ((pNodeInfo->nodeState << 8) | *pRet_p));

            *pRet_p = kEplSuccessful;
            return 0;
        }
        else if (*pRet_p != kEplSuccessful)
        {
            NMTMNU_DBG_POST_TRACE_VALUE(kNmtMnuIntNodeEventIdentResponse, nodeId_p,
                                            ((pNodeInfo->nodeState << 8) | *pRet_p));
            return 0;
        }
    }

    // continue BootStep1
    return processNodeEventBoot(nodeId_p, nodeNmtState_p, nmtState_p, errorCode_p,  pRet_p);
}

//------------------------------------------------------------------------------
/**
\brief  Process boot node event

The function processes the internal node event kNmtMnuIntNodeEventBoot.

\param  nodeId_p            Node ID to process.
\param  nodeNmtState_p      NMT state of the node.
\param  nmtState_p          NMT state of the MN
\param  errorCode_p         Error codes.
\param  pRet_p              Pointer to store return value

\return The function returns 0 if the higher level event handler should continue
        processing or -1 if it should exit.
*/
//------------------------------------------------------------------------------
static INT processNodeEventBoot(UINT nodeId_p, tNmtState nodeNmtState_p, tNmtState nmtState_p,
                                UINT16 errorCode_p, tEplKernel* pRet_p)
{
    tNmtMnuNodeInfo*    pNodeInfo;

    UNUSED_PARAMETER(errorCode_p);
    UNUSED_PARAMETER(nmtState_p);

    pNodeInfo = NMTMNU_GET_NODEINFO(nodeId_p);

    // $$$ check identification (vendor ID, product code, revision no, serial no)
    if (pNodeInfo->nodeState == kNmtMnuNodeStateIdentified)
    {
        // $$$ check software
        // check/start configuration
        // inform application
        *pRet_p = nmtMnuInstance_g.pfnCbNodeEvent(nodeId_p, kNmtNodeEventCheckConf,
                                              nodeNmtState_p, EPL_E_NO_ERROR,
                                              (pNodeInfo->nodeCfg & EPL_NODEASSIGN_MANDATORY_CN) != 0);
        if (*pRet_p == kEplReject)
        {   // interrupt boot process on user request
            NMTMNU_DBG_POST_TRACE_VALUE(kNmtMnuIntNodeEventBoot, nodeId_p,
                                            ((pNodeInfo->nodeState << 8) | *pRet_p));
            *pRet_p = kEplSuccessful;
            return 0;
        }
        else if (*pRet_p != kEplSuccessful)
        {
            NMTMNU_DBG_POST_TRACE_VALUE(kNmtMnuIntNodeEventBoot, nodeId_p,
                                            ((pNodeInfo->nodeState << 8) | *pRet_p));
            return 0;
        }
    }
    else if (pNodeInfo->nodeState == kNmtMnuNodeStateConfRestored)
    {
        // check/start configuration
        // inform application
        *pRet_p = nmtMnuInstance_g.pfnCbNodeEvent(nodeId_p, kNmtNodeEventUpdateConf,
                                              nodeNmtState_p, EPL_E_NO_ERROR,
                                              (pNodeInfo->nodeCfg & EPL_NODEASSIGN_MANDATORY_CN) != 0);
        if (*pRet_p == kEplReject)
        {   // interrupt boot process on user request
            NMTMNU_DBG_POST_TRACE_VALUE(kNmtMnuIntNodeEventBoot, nodeId_p,
                                            ((pNodeInfo->nodeState << 8) | *pRet_p));
            *pRet_p = kEplSuccessful;
            return 0;
        }
        else if (*pRet_p != kEplSuccessful)
        {
            NMTMNU_DBG_POST_TRACE_VALUE(kNmtMnuIntNodeEventBoot, nodeId_p,
                                            ((pNodeInfo->nodeState << 8) | *pRet_p));
            return 0;
        }
    }
    else if (pNodeInfo->nodeState != kNmtMnuNodeStateResetConf)
    {   // wrong CN state
        // ignore event
        return 0;
    }

    // we assume configuration is OK
    // continue BootStep1
    processNodeEventConfigured(nodeId_p, nodeNmtState_p, nmtState_p, errorCode_p, pRet_p);
    return 0;
}

//------------------------------------------------------------------------------
/**
\brief  Process Configured node event

The function processes the internal node event kNmtMnuIntNodeEventConfigured.

\param  nodeId_p            Node ID to process.
\param  nodeNmtState_p      NMT state of the node.
\param  nmtState_p          NMT state of the MN
\param  errorCode_p         Error codes.
\param  pRet_p              Pointer to store return value

\return The function returns 0 if the higher level event handler should continue
        processing or -1 if it should exit.
*/
//------------------------------------------------------------------------------
static INT processNodeEventConfigured(UINT nodeId_p, tNmtState nodeNmtState_p, tNmtState nmtState_p,
                                      UINT16 errorCode_p, tEplKernel* pRet_p)
{
    tNmtMnuNodeInfo*    pNodeInfo;

    UNUSED_PARAMETER(errorCode_p);
    UNUSED_PARAMETER(nodeNmtState_p);

    pNodeInfo = NMTMNU_GET_NODEINFO(nodeId_p);

    if ((pNodeInfo->nodeState != kNmtMnuNodeStateIdentified) &&
        (pNodeInfo->nodeState != kNmtMnuNodeStateConfRestored) &&
        (pNodeInfo->nodeState != kNmtMnuNodeStateResetConf))
    {   // wrong CN state, ignore event
        return 0;
    }

    pNodeInfo->nodeState = kNmtMnuNodeStateConfigured;
    if (nmtState_p == kNmtMsPreOperational1)
    {
        if ((pNodeInfo->nodeCfg & EPL_NODEASSIGN_MANDATORY_CN) != 0)
        {   // decrement mandatory CN counter
            nmtMnuInstance_g.mandatorySlaveCount--;
        }
    }
    else
    {
        // put optional node to next step (BootStep2)
        *pRet_p = nodeBootStep2(nodeId_p, pNodeInfo);
    }
    return 0;
}

//------------------------------------------------------------------------------
/**
\brief  Process NoIdentResponse node event

The function processes the internal node event kNmtMnuIntNodeEventNoIdentResponse.

\param  nodeId_p            Node ID to process.
\param  nodeNmtState_p      NMT state of the node.
\param  nmtState_p          NMT state of the MN
\param  errorCode_p         Error codes.
\param  pRet_p              Pointer to store return value

\return The function returns 0 if the higher level event handler should continue
        processing or -1 if it should exit.
*/
//------------------------------------------------------------------------------
INT processNodeEventNoIdentResponse(UINT nodeId_p, tNmtState nodeNmtState_p, tNmtState nmtState_p,
                                    UINT16 errorCode_p, tEplKernel* pRet_p)
{
    tEplTimerArg        timerArg;
    tNmtMnuNodeInfo*    pNodeInfo;

    pNodeInfo = NMTMNU_GET_NODEINFO(nodeId_p);

    if ((nmtState_p == kNmtMsPreOperational1) &&
        ((pNodeInfo->flags & NMTMNU_NODE_FLAG_NOT_SCANNED) != 0))
    {
        // decrement only signal slave count
        nmtMnuInstance_g.signalSlaveCount--;
        pNodeInfo->flags &= ~NMTMNU_NODE_FLAG_NOT_SCANNED;
    }

    if ((pNodeInfo->nodeState != kNmtMnuNodeStateResetConf) &&
        (pNodeInfo->nodeState != kNmtMnuNodeStateConfRestored))
    {
        pNodeInfo->nodeState = kNmtMnuNodeStateUnknown;
    }

    // check NMT state of CN
    *pRet_p = checkNmtState(nodeId_p, pNodeInfo, nodeNmtState_p, errorCode_p, nmtState_p);
    if (*pRet_p == kEplReject)
    {
        *pRet_p = kEplSuccessful;
    }
    else if (*pRet_p != kEplSuccessful)
    {
        return 0;
    }

    // $$$ d.k. check start time for 0x1F89/2 MNTimeoutPreOp1_U32
    // $$$ d.k. check individual timeout 0x1F89/6 MNIdentificationTimeout_U32
    // if mandatory node and timeout elapsed -> halt boot procedure
    // trigger IdentRequest again (if >= PreOp2, after delay)
    if (nmtState_p >= kNmtMsPreOperational2)
    {   // start timer
        NMTMNU_SET_FLAGS_TIMERARG_IDENTREQ(pNodeInfo, nodeId_p, timerArg);
        /* NMTMNU_DBG_POST_TRACE_VALUE(kNmtMnuIntNodeEventNoIdentResponse, nodeId_p,
                                          ((pNodeInfo->nodeState << 8) | 0x80
                                           | ((pNodeInfo->flags & NMTMNU_NODE_FLAG_COUNT_STATREQ) >> 6)
                                           | ((TimerArg.m_Arg.m_dwVal & NMTMNU_TIMERARG_COUNT_SR) >> 8)));*/
        *pRet_p = timeru_modifyTimer(&pNodeInfo->timerHdlStatReq,
                                     nmtMnuInstance_g.statusRequestDelay, timerArg);
    }
    else
    {   // trigger IdentRequest immediately
        *pRet_p = identu_requestIdentResponse(nodeId_p, cbIdentResponse);
    }

    return 0;
}

//------------------------------------------------------------------------------
/**
\brief  Process StatusResponse node event

The function processes the internal node event kNmtMnuIntNodeEventStatusResponse.

\param  nodeId_p            Node ID to process.
\param  nodeNmtState_p      NMT state of the node.
\param  nmtState_p          NMT state of the MN
\param  errorCode_p         Error codes.
\param  pRet_p              Pointer to store return value

\return The function returns 0 if the higher level event handler should continue
        processing or -1 if it should exit.
*/
//------------------------------------------------------------------------------
static INT processNodeEventStatusResponse(UINT nodeId_p, tNmtState nodeNmtState_p, tNmtState nmtState_p,
                                   UINT16 errorCode_p, tEplKernel* pRet_p)
{
    tEplTimerArg        timerArg;
    tNmtMnuNodeInfo*    pNodeInfo;

    pNodeInfo = NMTMNU_GET_NODEINFO(nodeId_p);

    if ((nmtState_p >= kNmtMsPreOperational2) &&
        ((pNodeInfo->flags & NMTMNU_NODE_FLAG_NOT_SCANNED) != 0))
    {
        // decrement only signal slave count if checked once for ReadyToOp, CheckCom, Operational
        nmtMnuInstance_g.signalSlaveCount--;
        pNodeInfo->flags &= ~NMTMNU_NODE_FLAG_NOT_SCANNED;
    }

    // check NMT state of CN
    *pRet_p = checkNmtState(nodeId_p, pNodeInfo, nodeNmtState_p, errorCode_p, nmtState_p);
    if (*pRet_p != kEplSuccessful)
    {
        if (*pRet_p == kEplReject)
            *pRet_p = kEplSuccessful;
        return 0;
    }

    if (nmtState_p == kNmtMsPreOperational1)
    {
        // request next StatusResponse immediately
        *pRet_p = statusu_requestStatusResponse(nodeId_p, cbStatusResponse);
        if (*pRet_p != kEplSuccessful)
        {
            NMTMNU_DBG_POST_TRACE_VALUE(kNmtMnuIntNodeEventStatusResponse, nodeId_p, *pRet_p);
        }

    }
    else if ((pNodeInfo->flags & NMTMNU_NODE_FLAG_ISOCHRON) == 0)
    {   // start timer
        // not isochronously accessed CN (e.g. async-only or stopped CN)
        NMTMNU_SET_FLAGS_TIMERARG_STATREQ(pNodeInfo, nodeId_p, timerArg);

        /*NMTMNU_DBG_POST_TRACE_VALUE(kNmtMnuIntNodeEventStatusResponse, nodeId_p,
                                        ((pNodeInfo->nodeState << 8) | 0x80
                                         | ((pNodeInfo->flags & NMTMNU_NODE_FLAG_COUNT_STATREQ) >> 6)
                                         | ((TimerArg.m_Arg.m_dwVal & NMTMNU_TIMERARG_COUNT_SR) >> 8)));*/
        *pRet_p = timeru_modifyTimer(&pNodeInfo->timerHdlStatReq,
                                     nmtMnuInstance_g.statusRequestDelay, timerArg);
    }
    return 0;
}

//------------------------------------------------------------------------------
/**
\brief  Process NoStatustResponse node event

The function processes the internal node event kNmtMnuIntNodeEventNoStatusResponse.

\param  nodeId_p            Node ID to process.
\param  nodeNmtState_p      NMT state of the node.
\param  nmtState_p          NMT state of the MN
\param  errorCode_p         Error codes.
\param  pRet_p              Pointer to store return value

\return The function returns 0 if the higher level event handler should continue
        processing or -1 if it should exit.
*/
//------------------------------------------------------------------------------
static INT processNodeEventNoStatusResponse(UINT nodeId_p, tNmtState nodeNmtState_p, tNmtState nmtState_p,
                                     UINT16 errorCode_p, tEplKernel* pRet_p)
{
    tNmtMnuNodeInfo*    pNodeInfo;

    pNodeInfo = NMTMNU_GET_NODEINFO(nodeId_p);

    // check NMT state of CN
    *pRet_p = checkNmtState(nodeId_p, pNodeInfo, nodeNmtState_p, errorCode_p, nmtState_p);
    if (*pRet_p == kEplReject)
        *pRet_p = kEplSuccessful;

    return 0;
}

//------------------------------------------------------------------------------
/**
\brief  Process Error node event

The function processes the internal node event kNmtMnuIntNodeEventError.

\param  nodeId_p            Node ID to process.
\param  nodeNmtState_p      NMT state of the node.
\param  nmtState_p          NMT state of the MN
\param  errorCode_p         Error codes.
\param  pRet_p              Pointer to store return value

\return The function returns 0 if the higher level event handler should continue
        processing or -1 if it should exit.
*/
//------------------------------------------------------------------------------
static INT processNodeEventError(UINT nodeId_p, tNmtState nodeNmtState_p, tNmtState nmtState_p,
                                 UINT16 errorCode_p, tEplKernel* pRet_p)
{
    tNmtMnuNodeInfo*    pNodeInfo;

    UNUSED_PARAMETER(nodeNmtState_p);

    pNodeInfo = NMTMNU_GET_NODEINFO(nodeId_p);

    // currently only issued on kNmtNodeCommandConfErr
    if ((pNodeInfo->nodeState != kNmtMnuNodeStateIdentified) &&
        (pNodeInfo->nodeState != kNmtMnuNodeStateConfRestored))
    {   // wrong CN state, ignore event
        return 0;
    }

    // check NMT state of CN
    *pRet_p = checkNmtState(nodeId_p, pNodeInfo, kNmtCsNotActive,
                            errorCode_p, nmtState_p);
    if (*pRet_p == kEplReject)
        *pRet_p = kEplSuccessful;

    return 0;
}

//------------------------------------------------------------------------------
/**
\brief  Process ExecResetNode node event

The function processes the internal node event kNmtMnuIntNodeEventExecResetNode.

\param  nodeId_p            Node ID to process.
\param  nodeNmtState_p      NMT state of the node.
\param  nmtState_p          NMT state of the MN
\param  errorCode_p         Error codes.
\param  pRet_p              Pointer to store return value

\return The function returns 0 if the higher level event handler should continue
        processing or -1 if it should exit.
*/
//------------------------------------------------------------------------------
static INT processNodeEventExecResetNode(UINT nodeId_p, tNmtState nodeNmtState_p, tNmtState nmtState_p,
                                         UINT16 errorCode_p, tEplKernel* pRet_p)
{
    tNmtMnuNodeInfo*    pNodeInfo;

    UNUSED_PARAMETER(errorCode_p);
    UNUSED_PARAMETER(nmtState_p);
    UNUSED_PARAMETER(nodeNmtState_p);

    pNodeInfo = NMTMNU_GET_NODEINFO(nodeId_p);

    if (pNodeInfo->nodeState != kNmtMnuNodeStateIdentified)
    {   // wrong CN state, ignore event
        return 0;
    }

    pNodeInfo->nodeState = kNmtMnuNodeStateConfRestored;
    NMTMNU_DBG_POST_TRACE_VALUE(kNmtMnuIntNodeEventExecResetNode, nodeId_p,
                                    (((nodeNmtState_p & 0xFF) << 8) | kNmtCmdResetNode));

    // send NMT reset node to CN for activation of restored configuration
    *pRet_p = nmtmnu_sendNmtCommand(nodeId_p, kNmtCmdResetNode);

    return 0;
}

//------------------------------------------------------------------------------
/**
\brief  Process ExecResetConf node event

The function processes the internal node event kNmtMnuIntNodeEventExecResetConf.

\param  nodeId_p            Node ID to process.
\param  nodeNmtState_p      NMT state of the node.
\param  nmtState_p          NMT state of the MN
\param  errorCode_p         Error codes.
\param  pRet_p              Pointer to store return value

\return The function returns 0 if the higher level event handler should continue
        processing or -1 if it should exit.
*/
//------------------------------------------------------------------------------
static INT processNodeEventExecResetConf(UINT nodeId_p, tNmtState nodeNmtState_p, tNmtState nmtState_p,
                                         UINT16 errorCode_p, tEplKernel* pRet_p)
{
    tNmtMnuNodeInfo*    pNodeInfo;

    UNUSED_PARAMETER(nodeNmtState_p);
    UNUSED_PARAMETER(errorCode_p);
    UNUSED_PARAMETER(nmtState_p);

    pNodeInfo = NMTMNU_GET_NODEINFO(nodeId_p);

    if ((pNodeInfo->nodeState != kNmtMnuNodeStateIdentified) &&
        (pNodeInfo->nodeState != kNmtMnuNodeStateConfRestored))
    {   // wrong CN state
       // ignore event
       return 0;
    }

    pNodeInfo->nodeState = kNmtMnuNodeStateResetConf;
    NMTMNU_DBG_POST_TRACE_VALUE(nodeEvent_p, nodeId_p,
                                   (((nodeNmtState_p & 0xFF) << 8) |
                                   kNmtCmdResetConfiguration));

    // send NMT reset configuration to CN for activation of configuration
    *pRet_p = nmtmnu_sendNmtCommand(nodeId_p, kNmtCmdResetConfiguration);

    return 0;
}

//------------------------------------------------------------------------------
/**
\brief  Process heartbeat node event

The function processes the internal node event kNmtMnuIntNodeEventHeartbeat.

\param  nodeId_p            Node ID to process.
\param  nodeNmtState_p      NMT state of the node.
\param  nmtState_p          NMT state of the MN
\param  errorCode_p         Error codes.
\param  pRet_p              Pointer to store return value

\return The function returns 0 if the higher level event handler should continue
        processing or -1 if it should exit.
*/
//------------------------------------------------------------------------------
static INT processNodeEventHeartbeat(UINT nodeId_p, tNmtState nodeNmtState_p, tNmtState nmtState_p,
                                     UINT16 errorCode_p, tEplKernel* pRet_p)
{
    tNmtMnuNodeInfo*    pNodeInfo;

    pNodeInfo = NMTMNU_GET_NODEINFO(nodeId_p);

    // check NMT state of CN
    *pRet_p = checkNmtState(nodeId_p, pNodeInfo, nodeNmtState_p, errorCode_p, nmtState_p);
    if (*pRet_p == kEplReject)
        *pRet_p = kEplSuccessful;

    return 0;
}

//------------------------------------------------------------------------------
/**
\brief  Process TimerIdentReq node event

The function processes the internal node event kNmtMnuIntNodeEventTimerIdentReq.

\param  nodeId_p            Node ID to process.
\param  nodeNmtState_p      NMT state of the node.
\param  nmtState_p          NMT state of the MN
\param  errorCode_p         Error codes.
\param  pRet_p              Pointer to store return value

\return The function returns 0 if the higher level event handler should continue
        processing or -1 if it should exit.
*/
//------------------------------------------------------------------------------
static INT processNodeEventTimerIdentReq(UINT nodeId_p, tNmtState nodeNmtState_p, tNmtState nmtState_p,
                                         UINT16 errorCode_p, tEplKernel* pRet_p)
{
    UNUSED_PARAMETER(errorCode_p);
    UNUSED_PARAMETER(nmtState_p);
    UNUSED_PARAMETER(nodeNmtState_p);

    EPL_DBGLVL_NMTMN_TRACE("TimerStatReq->IdentReq(%02X)\n", nodeId_p);
    // trigger IdentRequest again
    *pRet_p = identu_requestIdentResponse(nodeId_p, cbIdentResponse);
    if (*pRet_p != kEplSuccessful)
    {
        NMTMNU_DBG_POST_TRACE_VALUE(kNmtMnuIntNodeEventTimerIdentReq, nodeId_p,
                                        (((nodeNmtState_p & 0xFF) << 8) | *pRet_p));
        if (*pRet_p == kEplInvalidOperation)
        {   // this can happen because of a bug in EplTimeruLinuxKernel.c
            // so ignore this error.
            *pRet_p = kEplSuccessful;
        }
    }
    return 0;
}

//------------------------------------------------------------------------------
/**
\brief  Process TimerStatReq node event

The function processes the internal node event kNmtMnuIntNodeEventTimerStatReq.

\param  nodeId_p            Node ID to process.
\param  nodeNmtState_p      NMT state of the node.
\param  nmtState_p          NMT state of the MN
\param  errorCode_p         Error codes.
\param  pRet_p              Pointer to store return value

\return The function returns 0 if the higher level event handler should continue
        processing or -1 if it should exit.
*/
//------------------------------------------------------------------------------
static INT processNodeEventTimerStatReq(UINT nodeId_p, tNmtState nodeNmtState_p, tNmtState nmtState_p,
                                        UINT16 errorCode_p, tEplKernel* pRet_p)
{
    UNUSED_PARAMETER(errorCode_p);
    UNUSED_PARAMETER(nodeNmtState_p);
    UNUSED_PARAMETER(nmtState_p);

    EPL_DBGLVL_NMTMN_TRACE("TimerStatReq->StatReq(%02X)\n", nodeId_p);
    // request next StatusResponse
    *pRet_p = statusu_requestStatusResponse(nodeId_p, cbStatusResponse);
    if (*pRet_p != kEplSuccessful)
    {
       NMTMNU_DBG_POST_TRACE_VALUE(kNmtMnuIntNodeEventTimerStatReq, nodeId_p,
                                       (((nodeNmtState_p & 0xFF) << 8) | *pRet_p));
       if (*pRet_p == kEplInvalidOperation)
       {   // the only situation when this should happen is, when
           // StatusResponse was already requested while processing
           // event IdentResponse.
           // so ignore this error.
           *pRet_p = kEplSuccessful;
       }
    }
    return 0;
}

//------------------------------------------------------------------------------
/**
\brief  Process TimerStateMon node event

The function processes the internal node event kNmtMnuIntNodeEventTimerStateMon.

\param  nodeId_p            Node ID to process.
\param  nodeNmtState_p      NMT state of the node.
\param  nmtState_p          NMT state of the MN
\param  errorCode_p         Error codes.
\param  pRet_p              Pointer to store return value

\return The function returns 0 if the higher level event handler should continue
        processing or -1 if it should exit.
*/
//------------------------------------------------------------------------------
static INT processNodeEventTimerStateMon(UINT nodeId_p, tNmtState nodeNmtState_p, tNmtState nmtState_p,
                                         UINT16 errorCode_p, tEplKernel* pRet_p)
{
    tNmtMnuNodeInfo*    pNodeInfo;

    UNUSED_PARAMETER(nodeId_p);
    UNUSED_PARAMETER(errorCode_p);
    UNUSED_PARAMETER(nmtState_p);
    UNUSED_PARAMETER(nodeNmtState_p);
    UNUSED_PARAMETER(pRet_p);

    pNodeInfo = NMTMNU_GET_NODEINFO(nodeId_p);

    // reset NMT state change flag
    // because from now on the CN must have the correct NMT state
    pNodeInfo->flags &= ~NMTMNU_NODE_FLAG_NMT_CMD_ISSUED;

    // continue with normal StatReq processing
    return processNodeEventTimerStatReq(nodeId_p, nodeNmtState_p, nmtState_p, errorCode_p, pRet_p);
}

//------------------------------------------------------------------------------
/**
\brief  Process TimerLonger node event

The function processes the internal node event kNmtMnuIntNodeEventTimerLonger.

\param  nodeId_p            Node ID to process.
\param  nodeNmtState_p      NMT state of the node.
\param  nmtState_p          NMT state of the MN
\param  errorCode_p         Error codes.
\param  pRet_p              Pointer to store return value

\return The function returns 0 if the higher level event handler should continue
        processing or -1 if it should exit.
*/
//------------------------------------------------------------------------------
static INT processNodeEventTimerLonger(UINT nodeId_p, tNmtState nodeNmtState_p, tNmtState nmtState_p,
                                       UINT16 errorCode_p, tEplKernel* pRet_p)
{
    tNmtMnuNodeInfo*    pNodeInfo;

    UNUSED_PARAMETER(errorCode_p);
    UNUSED_PARAMETER(nodeNmtState_p);

    pNodeInfo = NMTMNU_GET_NODEINFO(nodeId_p);

    switch (pNodeInfo->nodeState)
    {
        case kNmtMnuNodeStateConfigured:
            // node should be ReadyToOp but it is not
            // check NMT state which shall be intentionally wrong, so that ERROR_TREATMENT will be started
            *pRet_p = checkNmtState(nodeId_p, pNodeInfo, kNmtCsNotActive, EPL_E_NMT_BPO2, nmtState_p);
            if (*pRet_p == kEplReject)
                *pRet_p = kEplSuccessful;
            break;

        case kNmtMnuNodeStateReadyToOp:
            // CheckCom finished successfully
            pNodeInfo->nodeState = kNmtMnuNodeStateComChecked;

            if ((pNodeInfo->flags & NMTMNU_NODE_FLAG_NOT_SCANNED) != 0)
            {
                // decrement only signal slave count if checked once for ReadyToOp, CheckCom, Operational
                nmtMnuInstance_g.signalSlaveCount--;
                pNodeInfo->flags &= ~NMTMNU_NODE_FLAG_NOT_SCANNED;
            }

            if ((pNodeInfo->nodeCfg & EPL_NODEASSIGN_MANDATORY_CN) != 0)
            {
                // decrement mandatory slave counter
                nmtMnuInstance_g.mandatorySlaveCount--;
            }

            if (nmtState_p != kNmtMsReadyToOperate)
            {
                NMTMNU_DBG_POST_TRACE_VALUE(kNmtMnuIntNodeEventTimerLonger, nodeId_p,
                                                (((nodeNmtState_p & 0xFF) << 8) | kNmtCmdStartNode));

                // start optional CN
                *pRet_p = nmtmnu_sendNmtCommand(nodeId_p, kNmtCmdStartNode);
            }
            break;

        default:
            break;
    }
    return 0;
}

//------------------------------------------------------------------------------
/**
\brief  Process NMTCmdSent node event

The function processes the internal node event kNmtMnuIntNodeEventNmtCmdSent.

\param  nodeId_p            Node ID to process.
\param  nodeNmtState_p      NMT state of the node.
\param  nmtState_p          NMT state of the MN
\param  errorCode_p         Error codes.
\param  pRet_p              Pointer to store return value

\return The function returns 0 if the higher level event handler should continue
        processing or -1 if it should exit.
*/
//------------------------------------------------------------------------------
static INT processNodeEventNmtCmdSent(UINT nodeId_p, tNmtState nodeNmtState_p, tNmtState nmtState_p,
                                      UINT16 errorCode_p, tEplKernel* pRet_p)
{
    UINT8               bNmtState;
    tEplTimerArg        timerArg;
    tNmtMnuNodeInfo*    pNodeInfo;

    UNUSED_PARAMETER(errorCode_p);
    UNUSED_PARAMETER(nmtState_p);

    pNodeInfo = NMTMNU_GET_NODEINFO(nodeId_p);

    // update expected NMT state with the one that results
    // from the sent NMT command
    bNmtState = (UINT8) (nodeNmtState_p & 0xFF);

    // write object 0x1F8F NMT_MNNodeExpState_AU8
    *pRet_p = obd_writeEntry(0x1F8F, nodeId_p, &bNmtState, 1);
    if (*pRet_p != kEplSuccessful)
        return -1;

    if (nodeNmtState_p == kNmtCsNotActive)
    {   // restart processing with IdentRequest
        NMTMNU_SET_FLAGS_TIMERARG_IDENTREQ(pNodeInfo, nodeId_p, timerArg);
    }
    else
    {   // monitor NMT state change with StatusRequest after
        // the corresponding delay;
        // until then wrong NMT states will be ignored
        NMTMNU_SET_FLAGS_TIMERARG_STATE_MON(pNodeInfo, nodeId_p, timerArg);

        // set NMT state change flag
        pNodeInfo->flags |= NMTMNU_NODE_FLAG_NMT_CMD_ISSUED;
    }
    *pRet_p = timeru_modifyTimer(&pNodeInfo->timerHdlStatReq,
                                 nmtMnuInstance_g.statusRequestDelay, timerArg);
    // finish processing, because NmtState_p is the expected and not the current state
    return -1;
}

//------------------------------------------------------------------------------
/**
\brief  Process internal node events

The function processes internal node events.

\param  nodeId_p            Node ID to process.
\param  nodeNmtState_p      NMT state of the node.
\param  errorCode_p         Error codes.
\param  nodeEvent_p         Occurred events.

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
static tEplKernel processInternalEvent(UINT nodeId_p, tNmtState nodeNmtState_p,
                                       UINT16 errorCode_p, tNmtMnuIntNodeEvent nodeEvent_p)
{
    tEplKernel          ret = kEplSuccessful;
    tNmtState           nmtState;

    nmtState = nmtu_getNmtState();
    if (nmtState <= kNmtMsNotActive)        // MN is not active
        goto Exit;

    // call internal node event handler
    if (apfnNodeEventFuncs_l[nodeEvent_p](nodeId_p, nodeNmtState_p, nmtState, errorCode_p, &ret) < 0)
        goto Exit;

    // check if network is ready to change local NMT state and this was not done before
    if ((nmtMnuInstance_g.flags & (NMTMNU_FLAG_HALTED | NMTMNU_FLAG_APP_INFORMED)) == 0)
    {   // boot process is not halted
        switch (nmtState)
        {
            case kNmtMsPreOperational1:
                if ((nmtMnuInstance_g.signalSlaveCount == 0) &&
                    (nmtMnuInstance_g.mandatorySlaveCount == 0))
                {   // all optional CNs scanned once and all mandatory CNs configured successfully
                    nmtMnuInstance_g.flags |= NMTMNU_FLAG_APP_INFORMED;
                    // inform application
                    ret = nmtMnuInstance_g.pfnCbBootEvent(kNmtBootEventBootStep1Finish,
                                                          nmtState, EPL_E_NO_ERROR);
                    if (ret != kEplSuccessful)
                    {
                        if (ret == kEplReject)
                            // wait for application
                            ret = kEplSuccessful;
                        break;
                    }
                    // enter PreOp2
                    ret = nmtu_postNmtEvent(kNmtEventAllMandatoryCNIdent);
                }
                break;

            case kNmtMsPreOperational2:
                if ((nmtMnuInstance_g.signalSlaveCount == 0) &&
                    (nmtMnuInstance_g.mandatorySlaveCount == 0))
                {   // all optional CNs checked once for ReadyToOp and all mandatory CNs are ReadyToOp
                    nmtMnuInstance_g.flags |= NMTMNU_FLAG_APP_INFORMED;
                    // inform application
                    ret = nmtMnuInstance_g.pfnCbBootEvent(kNmtBootEventBootStep2Finish,
                                                          nmtState, EPL_E_NO_ERROR);
                    if (ret != kEplSuccessful)
                    {
                        if (ret == kEplReject)
                            // wait for application
                            ret = kEplSuccessful;
                        break;
                    }
                    // enter ReadyToOp
                    ret = nmtu_postNmtEvent(kNmtEventEnterReadyToOperate);
                }
                break;

            case kNmtMsReadyToOperate:
                if ((nmtMnuInstance_g.signalSlaveCount == 0) &&
                    (nmtMnuInstance_g.mandatorySlaveCount == 0))
                {   // all CNs checked for errorless communication
                    nmtMnuInstance_g.flags |= NMTMNU_FLAG_APP_INFORMED;
                    // inform application
                    ret = nmtMnuInstance_g.pfnCbBootEvent(kNmtBootEventCheckComFinish,
                                                          nmtState, EPL_E_NO_ERROR);
                    if (ret != kEplSuccessful)
                    {
                        if (ret == kEplReject)
                            // wait for application
                            ret = kEplSuccessful;
                        break;
                    }
                    // enter Operational
                    ret = nmtu_postNmtEvent(kNmtEventEnterMsOperational);
                }
                break;

            case kNmtMsOperational:
                if ((nmtMnuInstance_g.signalSlaveCount == 0) &&
                    (nmtMnuInstance_g.mandatorySlaveCount == 0))
                {   // all optional CNs scanned once and all mandatory CNs are OPERATIONAL
                    nmtMnuInstance_g.flags |= NMTMNU_FLAG_APP_INFORMED;
                    // inform application
                    ret = nmtMnuInstance_g.pfnCbBootEvent(kNmtBootEventOperational,
                                                          nmtState, EPL_E_NO_ERROR);
                    if (ret != kEplSuccessful)
                    {
                        if (ret == kEplReject)
                            // ignore error code
                            ret = kEplSuccessful;
                        break;
                    }
                }
                break;

            default:
                break;
        }
    }

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Check NMT state

The function checks the NMT state, i.e. evaluates it with object 0x1F8F
NMT_MNNodeExpState_AU8 and updates object 0x1F8E NMT_MNNodeCurrState_AU8.
It manipulates the nodeState in the internal node info structure.

\param  nodeId_p            Node ID to check.
\param  pNodeInfo_p         Pointer to node information structure.
\param  nodeNmtState_p      NMT state of the node.
\param  errorCode_p         Error codes.
\param  localNmtState_p     The local NMT state.

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
static tEplKernel checkNmtState(UINT nodeId_p, tNmtMnuNodeInfo* pNodeInfo_p,
                                tNmtState nodeNmtState_p, UINT16 errorCode_p,
                                tNmtState localNmtState_p)
{
    tEplKernel      ret = kEplSuccessful;
    tEplKernel      retUpdate = kEplSuccessful;
    tObdSize        obdSize;
    UINT8           nodeNmtState;
    UINT8           bExpNmtState;
    UINT8           nmtStatePrev;
    tNmtState       expNmtState;

    // compute UINT8 of current NMT state
    nodeNmtState = ((UINT8) nodeNmtState_p & 0xFF);

    if (pNodeInfo_p->nodeState == kNmtMnuNodeStateUnknown)
    {   // CN is already in state unknown, which means that it got
        // NMT reset command earlier
        ret = kEplReject;
        goto ExitButUpdate;
    }

    obdSize = 1;
    // read object 0x1F8F NMT_MNNodeExpState_AU8
    ret = obd_readEntry(0x1F8F, nodeId_p, &bExpNmtState, &obdSize);
    if (ret != kEplSuccessful)
        goto Exit;

    // compute expected NMT state
    expNmtState = (tNmtState) (bExpNmtState | NMT_TYPE_CS);

    if (expNmtState == kNmtCsNotActive)
    {   // ignore the current state, because the CN shall be not active
        ret = kEplReject;
        goto ExitButUpdate;
    }
    else if ((expNmtState == kNmtCsStopped) && (nodeNmtState_p == kNmtCsStopped))
    {
        // reset flags ISOCHRON and PREOP2_REACHED
        pNodeInfo_p->flags &= ~(NMTMNU_NODE_FLAG_ISOCHRON
                                 | NMTMNU_NODE_FLAG_PREOP2_REACHED);
    }
    else if ((expNmtState == kNmtCsPreOperational2) && (nodeNmtState_p == kNmtCsPreOperational2))
    {   // CN is PreOp2
        if ((pNodeInfo_p->flags & NMTMNU_NODE_FLAG_PREOP2_REACHED) == 0)
        {   // CN switched to PreOp2
            pNodeInfo_p->flags |= NMTMNU_NODE_FLAG_PREOP2_REACHED;

            if (pNodeInfo_p->nodeCfg & EPL_NODEASSIGN_ASYNCONLY_NODE)
            {
                if ((pNodeInfo_p->nodeState == kNmtMnuNodeStateConfigured) &&
                    (localNmtState_p >= kNmtMsPreOperational2))
                {
                    ret = nodeBootStep2(nodeId_p, pNodeInfo_p);
                }
            }
            else
            {   // add node to isochronous phase
                ret = addNodeIsochronous(nodeId_p);
                if (ret != kEplSuccessful)
                    goto Exit;
            }
        }
    }
    else if ((expNmtState == kNmtCsPreOperational2) && (nodeNmtState_p == kNmtCsReadyToOperate))
    {   // CN switched to ReadyToOp
        // delete timer for timeout handling
        ret = timeru_deleteTimer(&pNodeInfo_p->timerHdlLonger);
        if (ret != kEplSuccessful)
            goto Exit;

        pNodeInfo_p->nodeState = kNmtMnuNodeStateReadyToOp;

        // update object 0x1F8F NMT_MNNodeExpState_AU8 to ReadyToOp
        ret = obd_writeEntry(0x1F8F, nodeId_p, &nodeNmtState, 1);
        if (ret != kEplSuccessful)
            goto Exit;

        if ((pNodeInfo_p->nodeCfg & EPL_NODEASSIGN_MANDATORY_CN) != 0)
        {   // node is a mandatory CN -> decrement counter
            nmtMnuInstance_g.mandatorySlaveCount--;
        }
        if (localNmtState_p >= kNmtMsReadyToOperate)
        {   // start procedure CheckCommunication for this node
            ret = nodeCheckCom(nodeId_p, pNodeInfo_p);
            if (ret != kEplSuccessful)
                goto ExitButUpdate;

            if ((localNmtState_p == kNmtMsOperational) && (pNodeInfo_p->nodeState == kNmtMnuNodeStateComChecked))
            {
                NMTMNU_DBG_POST_TRACE_VALUE(0, nodeId_p,
                                                (((nodeNmtState_p & 0xFF) << 8) | kNmtCmdStartNode));

                // immediately start optional CN, because communication is always OK (e.g. async-only CN)
                ret = nmtmnu_sendNmtCommand(nodeId_p, kNmtCmdStartNode);
                if (ret != kEplSuccessful)
                    goto Exit;
            }
        }
    }
    else if ((pNodeInfo_p->nodeState == kNmtMnuNodeStateComChecked) && (nodeNmtState_p == kNmtCsOperational))
    {   // CN switched to OPERATIONAL
        pNodeInfo_p->nodeState = kNmtMnuNodeStateOperational;

        if ((pNodeInfo_p->nodeCfg & EPL_NODEASSIGN_MANDATORY_CN) != 0)
        {   // node is a mandatory CN -> decrement counter
            nmtMnuInstance_g.mandatorySlaveCount--;
        }
    }
    else if (expNmtState != nodeNmtState_p)
    {   // CN is not in expected NMT state
        UINT16 beErrorCode;

        if (errorCode_p == 0)
        {   // assume wrong NMT state error
            if ((pNodeInfo_p->flags & NMTMNU_NODE_FLAG_NMT_CMD_ISSUED) != 0)
            {   // NMT command has been just issued;
                // ignore wrong NMT state until timer expires;
                // other errors like LOSS_PRES_TH are still processed
                goto Exit;
            }
            errorCode_p = EPL_E_NMT_WRONG_STATE;
        }

        if ((pNodeInfo_p->flags & NMTMNU_NODE_FLAG_NOT_SCANNED) != 0)
        {
            // decrement only signal slave count if checked once
            nmtMnuInstance_g.signalSlaveCount--;
            pNodeInfo_p->flags &= ~NMTMNU_NODE_FLAG_NOT_SCANNED;
        }

        // -> CN is in wrong NMT state
        pNodeInfo_p->nodeState = kNmtMnuNodeStateUnknown;

        BENCHMARK_MOD_07_TOGGLE(7);

        // $$$ start ERROR_TREATMENT and inform application
        ret = nmtMnuInstance_g.pfnCbNodeEvent(nodeId_p, kNmtNodeEventError,
                                              nodeNmtState_p, errorCode_p,
                                              (pNodeInfo_p->nodeCfg & EPL_NODEASSIGN_MANDATORY_CN) != 0);
        if (ret != kEplSuccessful)
            goto ExitButUpdate;

        NMTMNU_DBG_POST_TRACE_VALUE(0, nodeId_p, (((nodeNmtState_p & 0xFF) << 8) | kNmtCmdResetNode));

        // reset CN
        // store error code in NMT command data for diagnostic purpose
        AmiSetWordToLe(&beErrorCode, errorCode_p);
        ret = nmtmnu_sendNmtCommandEx(nodeId_p, kNmtCmdResetNode, &beErrorCode, sizeof (beErrorCode));
        if (ret == kEplSuccessful)
            ret = kEplReject;

        // d.k. continue with updating the current NMT state of the CN
//        goto Exit;
    }

ExitButUpdate:
    // check if NMT_MNNodeCurrState_AU8 has to be changed
    obdSize = 1;
    retUpdate = obd_readEntry(0x1F8E, nodeId_p, &nmtStatePrev, &obdSize);
    if (retUpdate != kEplSuccessful)
    {
        ret = retUpdate;
        goto Exit;
    }
    if (nodeNmtState != nmtStatePrev)
    {
        // update object 0x1F8E NMT_MNNodeCurrState_AU8
        retUpdate = obd_writeEntry(0x1F8E, nodeId_p, &nodeNmtState, 1);
        if (retUpdate != kEplSuccessful)
        {
            ret =retUpdate;
            goto Exit;
        }
        retUpdate = nmtMnuInstance_g.pfnCbNodeEvent(nodeId_p, kNmtNodeEventNmtState,
                                                    nodeNmtState_p, errorCode_p,
                                                    (pNodeInfo_p->nodeCfg & EPL_NODEASSIGN_MANDATORY_CN) != 0);
        if (retUpdate != kEplSuccessful)
        {
            ret = retUpdate;
            goto Exit;
        }
    }

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Reset internal structures

The function resets the internal structures, e.g. timers.

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
static tEplKernel reset(void)
{
    tEplKernel  ret;
    UINT        index;

    ret = timeru_deleteTimer(&nmtMnuInstance_g.timerHdlNmtState);
    for (index = 1; index <= tabentries (nmtMnuInstance_g.aNodeInfo); index++)
    {
        ret = timeru_deleteTimer(&NMTMNU_GET_NODEINFO(index)->timerHdlStatReq);
        ret = timeru_deleteTimer(&NMTMNU_GET_NODEINFO(index)->timerHdlLonger);
    }

#if EPL_NMTMNU_PRES_CHAINING_MN != FALSE
    nmtMnuInstance_g.prcPResMnTimeoutNs = 0;
#endif
    return ret;
}


#if EPL_NMTMNU_PRES_CHAINING_MN != FALSE
//------------------------------------------------------------------------------
/**
\brief  Perform measure phase of PRC node insertion

The function performs the measure phase of a PRC node insertion

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
static tEplKernel prcMeasure(void)
{
    tEplKernel          ret;
    UINT                nodeId;
    tNmtMnuNodeInfo*    pNodeInfo;
    BOOL                fSyncReqSentToPredNode;
    UINT                nodeIdFirstNode;
    UINT                nodeIdPredNode;
    UINT                nodeIdPrevSyncReq;

    ret = kEplSuccessful;

    fSyncReqSentToPredNode = FALSE;
    nodeIdPredNode       = EPL_C_ADR_INVALID;
    nodeIdPrevSyncReq    = EPL_C_ADR_INVALID;
    nodeIdFirstNode      = EPL_C_ADR_INVALID;

    for (nodeId = 1; nodeId < 254; nodeId++)
    {
        pNodeInfo = NMTMNU_GET_NODEINFO(nodeId);
        if (pNodeInfo == NULL)
            continue;

        if (   (pNodeInfo->nodeCfg & EPL_NODEASSIGN_PRES_CHAINING)
            && (   (pNodeInfo->flags & NMTMNU_NODE_FLAG_ISOCHRON)
                || (pNodeInfo->prcFlags & NMTMNU_NODE_FLAG_PRC_ADD_IN_PROGRESS)))
        {
            if (nodeIdFirstNode == EPL_C_ADR_INVALID)
            {
                if (pNodeInfo->prcFlags & NMTMNU_NODE_FLAG_PRC_ADD_IN_PROGRESS)
                {   // First add-in-progress node found
                    nodeIdFirstNode = nodeId;
                }
                else
                {
                    nodeIdPredNode = nodeId;
                    continue;
                }
            }

            // Start processing with the first add-in-progress node
            if (pNodeInfo->relPropagationDelayNs == 0)
            {
                if (nodeIdPredNode == EPL_C_ADR_INVALID)
                {   // No predecessor node in isochronous phase
                    pNodeInfo->relPropagationDelayNs = EPL_C_DLL_T_IFG;
                    // No SyncReq needs to be send
                }
                else
                {   // Predecessor node exists
                tDllSyncRequest    SyncRequestData;
                UINT               uiSize;

                    SyncRequestData.syncControl = EPL_SYNC_DEST_MAC_ADDRESS_VALID;
                    uiSize = sizeof(UINT) + sizeof(UINT32);

                    if (fSyncReqSentToPredNode == FALSE)
                    {
                        SyncRequestData.nodeId = nodeIdPredNode;

                        ret = syncu_requestSyncResponse(prcCbSyncResMeasure, &SyncRequestData, uiSize);
                        if (ret != kEplSuccessful)
                        {
                            goto Exit;
                        }
                    }

                    SyncRequestData.nodeId = nodeId;

                    ret = syncu_requestSyncResponse(prcCbSyncResMeasure, &SyncRequestData, uiSize);
                    if (ret != kEplSuccessful)
                    {
                        goto Exit;
                    }

                    fSyncReqSentToPredNode = TRUE;
                    nodeIdPrevSyncReq    = nodeId;
                }
            }
            else
            {
                fSyncReqSentToPredNode = FALSE;
            }

            nodeIdPredNode = nodeId;
        }
    }

    if (nodeIdPrevSyncReq != EPL_C_ADR_INVALID)
    {   // At least one SyncReq has been sent
        pNodeInfo = NMTMNU_GET_NODEINFO(nodeIdPrevSyncReq);
        pNodeInfo->prcFlags |= NMTMNU_NODE_FLAG_PRC_CALL_MEASURE;
    }
    else
    {   // No SyncReq has been sent
        if (nodeIdFirstNode == EPL_C_ADR_INVALID)
        {   // No add-in-progress node has been found. This might happen
            // due to reset-node NMT commands which were issued
            // between the first and the second measure scan.
            nmtMnuInstance_g.flags &= ~NMTMNU_FLAG_PRC_ADD_IN_PROGRESS;

            // A new insertion process can be started
            ret = addNodeIsochronous(EPL_C_ADR_INVALID);
        }
        else
        {
            // Prepare shift phase and add phase
            ret = prcCalculate(nodeIdFirstNode);
        }
    }

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Calculation of PRes Chaining relevant times

The function calculation of PRes Response Times (CNs) and PRes Chaining Slot
Time (MN).

\param  nodeIdFirstNode_p       Node ID of the first (lowest node ID) of nodes
                                whose addition is in progress.

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
static tEplKernel prcCalculate(UINT nodeIdFirstNode_p)
{
    tEplKernel          ret;
    UINT                nodeId;
    tNmtMnuNodeInfo*    pNodeInfo;
    UINT                nodeIdPredNode;
    UINT32              pResResponseTimeNs;
    UINT32              pResMnTimeoutNs;

    if ((nodeIdFirstNode_p == EPL_C_ADR_INVALID) || (nodeIdFirstNode_p >= EPL_C_ADR_BROADCAST))
    {   // invalid node ID specified
        return kEplInvalidNodeId;
    }

    nodeIdPredNode = EPL_C_ADR_INVALID;
    for (nodeId = nodeIdFirstNode_p; nodeId < 254; nodeId++)
    {
        pNodeInfo = NMTMNU_GET_NODEINFO(nodeId);
        if (pNodeInfo == NULL)
            continue;

        if ((pNodeInfo->nodeCfg & EPL_NODEASSIGN_PRES_CHAINING) &&
             ((pNodeInfo->flags & NMTMNU_NODE_FLAG_ISOCHRON) ||
              (pNodeInfo->prcFlags & NMTMNU_NODE_FLAG_PRC_ADD_IN_PROGRESS)))
        {
            ret = prcCalcPResResponseTimeNs(nodeId, nodeIdPredNode, &pResResponseTimeNs);
            if (ret != kEplSuccessful)
                goto Exit;

            if (pNodeInfo->pResTimeFirstNs < pResResponseTimeNs)
            {
                pNodeInfo->pResTimeFirstNs = pResResponseTimeNs;
                if (pNodeInfo->flags & NMTMNU_NODE_FLAG_ISOCHRON)
                {
                    pNodeInfo->prcFlags |= NMTMNU_NODE_FLAG_PRC_SHIFT_REQUIRED;
                }
            }
            nodeIdPredNode = nodeId;
        }
    }

    ret = prcCalcPResChainingSlotTimeNs(nodeIdPredNode, &pResMnTimeoutNs);
    if (ret != kEplSuccessful)
        goto Exit;

    if (nmtMnuInstance_g.prcPResMnTimeoutNs < pResMnTimeoutNs)
    {
        tDllNodeInfo    dllNodeInfo;

        EPL_MEMSET(&dllNodeInfo, 0, sizeof(tDllNodeInfo));
        nmtMnuInstance_g.prcPResMnTimeoutNs = pResMnTimeoutNs;
        dllNodeInfo.presTimeoutNs = pResMnTimeoutNs;
        dllNodeInfo.nodeId = EPL_C_ADR_MN_DEF_NODE_ID;
        ret = dllucal_configNode(&dllNodeInfo);
        if (ret != kEplSuccessful)
            goto Exit;
    }
    // enter next phase
    ret = prcShift(EPL_C_ADR_INVALID);
Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Calculation of PRes Response Time of a node

The function calculates the PRes Response Time of the specified node.

\param  nodeId_p                Node ID for which to calculate time.
\param  nodeIdPredNode_p        Node ID of the predecessor node.
\param  pPResResponseTimeNs_p   Pointer to store calculated time.

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
static tEplKernel prcCalcPResResponseTimeNs(UINT nodeId_p, UINT nodeIdPredNode_p,
                                            UINT32* pPResResponseTimeNs_p)
{
    tEplKernel              ret;
    UINT16                  pResPayloadLimitPredNode;
    tNmtMnuNodeInfo*        pNodeInfoPredNode;
    tObdSize                obdSize;

    ret = kEplSuccessful;

    if (nodeIdPredNode_p == EPL_C_ADR_INVALID)
    {   // no predecessor node passed
        nodeIdPredNode_p = prcFindPredecessorNode(nodeId_p);

        if (nodeIdPredNode_p == EPL_C_ADR_INVALID)
        {   // no predecessor node found
            // PRes Response Time of first PRC node is defined to 0
            *pPResResponseTimeNs_p = 0;
            goto Exit;
        }
    }

    pNodeInfoPredNode = NMTMNU_GET_NODEINFO(nodeIdPredNode_p);

    // read object 0x1F8D NMT_PResPayloadLimitList_AU16
    obdSize = 2;
    ret = obd_readEntry(0x1F8D, nodeIdPredNode_p, &pResPayloadLimitPredNode, &obdSize);
    if (ret != kEplSuccessful)
        goto Exit;

    *pPResResponseTimeNs_p =
          // PRes Response Time of predecessor node
          pNodeInfoPredNode->pResTimeFirstNs +
          // Transmission time for PRes frame of predecessor node
          (8 * EPL_C_DLL_T_BITTIME * (pResPayloadLimitPredNode +
                                      EPL_C_DLL_T_EPL_PDO_HEADER +
                                      EPL_C_DLL_T_ETH2_WRAPPER) +
           EPL_C_DLL_T_PREAMBLE) +
          // Relative propragation delay from predecessor node to addressed node
          NMTMNU_GET_NODEINFO(nodeId_p)->relPropagationDelayNs +
          // Time correction (hub jitter and part of measurement inaccuracy)
          nmtMnuInstance_g.prcPResTimeFirstCorrectionNs;

    // apply negative offset for the second node, only
    if (pNodeInfoPredNode->pResTimeFirstNs == 0)
    {
        if (*pPResResponseTimeNs_p > nmtMnuInstance_g.prcPResTimeFirstNegOffsetNs)
        {
            *pPResResponseTimeNs_p -= nmtMnuInstance_g.prcPResTimeFirstNegOffsetNs;
        }
        else
        {
            *pPResResponseTimeNs_p = 0;
        }
    }
Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Calculation of PRes chaining slot time

The function calculates the PRes chaining slot time.

\param  nodeIdLastNode_p            Node ID of the last node.
\param  pPResChainingSlotTimeNs_p   Pointer to store calculated time.

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
static tEplKernel prcCalcPResChainingSlotTimeNs(UINT nodeIdLastNode_p,
                                                UINT32* pPResChainingSlotTimeNs_p)
{
    tEplKernel      ret;
    UINT16          pResActPayloadLimit;
    UINT16          cnPReqPayloadLastNode;
    UINT32          cnResTimeoutLastNodeNs;
    tObdSize        obdSize;

    // read object 0x1F98 NMT_CycleTiming_REC
    // Sub-Index 05h PResActPayloadLimit_U16
    obdSize = 2;
    ret = obd_readEntry(0x1F98, 5, &pResActPayloadLimit, &obdSize);
    if (ret != kEplSuccessful)
        goto Exit;

    // read object 0x1F8B NMT_MNPReqPayloadLimitList_AU16
    obdSize = 2;
    ret = obd_readEntry(0x1F8B, nodeIdLastNode_p, &cnPReqPayloadLastNode, &obdSize);
    if (ret != kEplSuccessful)
        goto Exit;

    // read object 0x1F92 NMT_MNCNPResTimeout_AU32
    obdSize = 4;
    ret = obd_readEntry(0x1F92, nodeIdLastNode_p, &cnResTimeoutLastNodeNs, &obdSize);
    if (ret != kEplSuccessful)
        goto Exit;

    *pPResChainingSlotTimeNs_p =
          // Transmission time for PResMN frame
          (8 * EPL_C_DLL_T_BITTIME * (pResActPayloadLimit +
                                      EPL_C_DLL_T_EPL_PDO_HEADER +
                                      EPL_C_DLL_T_ETH2_WRAPPER) +
           EPL_C_DLL_T_PREAMBLE) +
          // PRes Response Time of last node
          NMTMNU_GET_NODEINFO(nodeIdLastNode_p)->pResTimeFirstNs +
          // Relative propagation delay from last node to MN
          // Due to Soft-MN limitations, NMT_MNCNPResTimeout_AU32.CNResTimeout
          // of the last node is used.
          cnResTimeoutLastNodeNs -
          // Transmission time for PReq frame of last node
          (8 * EPL_C_DLL_T_BITTIME * (cnPReqPayloadLastNode
                                      + EPL_C_DLL_T_EPL_PDO_HEADER
                                      + EPL_C_DLL_T_ETH2_WRAPPER) +
             EPL_C_DLL_T_PREAMBLE) +
          // Time correction (hub jitter and part of measurement inaccuracy)
         nmtMnuInstance_g.prcPResTimeFirstCorrectionNs;

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Find predecessor node for PRC

The function searches the predecessor of the addressed node. The function
processes only PRC nodes which are added to the isochronous phase or whose
addition is in progress.

\param  nodeId_p            Node ID of the processed node.

\return The function returns the node ID of the predecessor node or
        EPL_C_ADR_INVALID if no node was found.
*/
//------------------------------------------------------------------------------
static UINT prcFindPredecessorNode(UINT nodeId_p)
{
    UINT                    nodeId;
    tNmtMnuNodeInfo*        pNodeInfo;

    for (nodeId = nodeId_p - 1; nodeId >= 1; nodeId--)
    {
        pNodeInfo = NMTMNU_GET_NODEINFO(nodeId);
        if (pNodeInfo == NULL)
            continue;

        if ((pNodeInfo->nodeCfg & EPL_NODEASSIGN_PRES_CHAINING) &&
            ((pNodeInfo->flags & NMTMNU_NODE_FLAG_ISOCHRON) ||
             (pNodeInfo->prcFlags & NMTMNU_NODE_FLAG_PRC_ADD_IN_PROGRESS)))
        {
            break;
        }
    }
    return nodeId;
}

//------------------------------------------------------------------------------
/**
\brief  Callback function for sync response frames

The function implements the callback function for SyncRes frames after sending
of SyncReq which is used for measurement.

\param  nodeId_p          Node ID of the node.
\param  pSyncResponse_p   Pointer to SyncResponse frame.

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
static tEplKernel PUBLIC prcCbSyncResMeasure(
                                  UINT                  nodeId_p,
                                  tEplSyncResponse*     pSyncResponse_p)
{
    tEplKernel          ret;
    UINT                nodeIdPredNode;
    tNmtMnuNodeInfo*    pNodeInfo;
    UINT32              syncNodeNumber;

    pNodeInfo = NMTMNU_GET_NODEINFO(nodeId_p);

    if (pSyncResponse_p == NULL)
    {   // SyncRes not received
        prcSyncError(pNodeInfo);
        goto Exit;
    }

    if (pNodeInfo->relPropagationDelayNs != 0)
    {   // Relative propagation delay is already present
        goto Exit;
    }

    nodeIdPredNode = prcFindPredecessorNode(nodeId_p);
    syncNodeNumber = AmiGetDwordFromLe(&pSyncResponse_p->m_le_dwSyncNodeNumber);

    if (syncNodeNumber != nodeIdPredNode)
    {   // SyncNodeNumber does not match predecessor node
        prcSyncError(pNodeInfo);
        goto Exit;
    }

    pNodeInfo->relPropagationDelayNs = AmiGetDwordFromLe(&pSyncResponse_p->m_le_dwSyncDelay);

    // If a previous SyncRes frame was not usable,
    // the Sync Error flag is cleared as this one is OK
    pNodeInfo->prcFlags &= ~NMTMNU_NODE_FLAG_PRC_SYNC_ERR;

Exit:
    ret = prcCbSyncResNextAction(nodeId_p, pSyncResponse_p);

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Set sync error flag

The function sets the Sync Error flag and schedules reset node if required.

\param  pNodeInfo_p         Pointer to node information structure.
*/
//------------------------------------------------------------------------------
static void prcSyncError(tNmtMnuNodeInfo* pNodeInfo_p)
{
    if (pNodeInfo_p->prcFlags & NMTMNU_NODE_FLAG_PRC_SYNC_ERR)
    {   // Sync Error flag already set
        // Schedule reset node
        pNodeInfo_p->prcFlags &= ~NMTMNU_NODE_FLAG_PRC_RESET_MASK;
        pNodeInfo_p->prcFlags |= NMTMNU_NODE_FLAG_PRC_RESET_NODE;
    }
    else
    {   // Set Sync Error flag
        pNodeInfo_p->prcFlags |= NMTMNU_NODE_FLAG_PRC_SYNC_ERR;
    }
}

//------------------------------------------------------------------------------
/**
\brief  Perform shift phase for PRC node insertion

The function performs the shift phase for PRC node insertion.

\param  nodeIdPrevShift_p   Node ID of previously shifted node.

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
static tEplKernel prcShift(UINT nodeIdPrevShift_p)
{
    tEplKernel          ret;
    UINT                nodeId;
    tNmtMnuNodeInfo*    pNodeInfo;
    tDllSyncRequest     syncRequestData;
    UINT                size;

    ret = kEplSuccessful;
    if (nodeIdPrevShift_p == EPL_C_ADR_INVALID)
        nodeIdPrevShift_p = 254;

    // The search starts with the previous shift node
    // as this node might require a second SyncReq
    nodeId = nodeIdPrevShift_p;
    do
    {
        pNodeInfo = NMTMNU_GET_NODEINFO(nodeId);
        if (pNodeInfo == NULL)
            continue;

        if ((pNodeInfo->nodeCfg & EPL_NODEASSIGN_PRES_CHAINING) &&
            ((pNodeInfo->flags & NMTMNU_NODE_FLAG_ISOCHRON) ||
             (pNodeInfo->prcFlags & NMTMNU_NODE_FLAG_PRC_ADD_IN_PROGRESS)))
        {
            if (pNodeInfo->prcFlags & NMTMNU_NODE_FLAG_PRC_SHIFT_REQUIRED)
                break;
        }
        nodeId--;
    } while (nodeId >= 1);

    if (nodeId == 0)
    {   // No node requires shifting
        // Enter next phase
        ret = prcAdd(EPL_C_ADR_INVALID);
        goto Exit;
    }

    // Call shift on reception of SyncRes
    pNodeInfo->prcFlags |= NMTMNU_NODE_FLAG_PRC_CALL_SHIFT;

    // Send SyncReq
    syncRequestData.nodeId        = nodeId;
    syncRequestData.syncControl   = EPL_SYNC_PRES_TIME_FIRST_VALID |
                                        EPL_SYNC_DEST_MAC_ADDRESS_VALID;
    syncRequestData.pResTimeFirst = pNodeInfo->pResTimeFirstNs;
    size = sizeof(UINT) + 2*sizeof(UINT32);
    ret = syncu_requestSyncResponse(prcCbSyncResShift, &syncRequestData, size);

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Callback function for Sync Response after shifting

The function performs the the callback function for SyncRes frames after sending
of SyncReq which is used for shifting.

\param  nodeId_p            Node ID of node.
\param  pSyncResponse_p     Pointer to received SyncRes frame.

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
static tEplKernel PUBLIC prcCbSyncResShift(UINT nodeId_p, tEplSyncResponse* pSyncResponse_p)
{
    tEplKernel              ret;
    tNmtMnuNodeInfo*        pNodeInfo;

    pNodeInfo = NMTMNU_GET_NODEINFO(nodeId_p);
    if (pSyncResponse_p == NULL)
    {   // SyncRes not received
        prcSyncError(pNodeInfo);
        goto Exit;
    }

    pNodeInfo->prcFlags &= ~NMTMNU_NODE_FLAG_PRC_SHIFT_REQUIRED;

    // If a previous SyncRes frame was not usable,
    // the Sync Error flag is cleared as this one is OK
    pNodeInfo->prcFlags &= ~NMTMNU_NODE_FLAG_PRC_SYNC_ERR;

    // Schedule verify
    pNodeInfo->prcFlags |= NMTMNU_NODE_FLAG_PRC_VERIFY;

Exit:
    ret = prcCbSyncResNextAction(nodeId_p, pSyncResponse_p);

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Perform add phase of PRC node insertion

The function performs the add phase of a PRC node insertion.

\param  nodeIdPrevAdd_p     Node ID of previously added node.

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
static tEplKernel prcAdd(UINT nodeIdPrevAdd_p)
{
    tEplKernel          ret;
    tObdSize            obdSize;
    UINT32              cycleLenUs;
    UINT32              cNLossOfSocToleranceNs;
    UINT                nodeId;
    tNmtMnuNodeInfo*    pNodeInfo;
    tDllSyncRequest     syncReqData;
    UINT                syncReqNum;
    tNmtMnuNodeInfo*    pNodeInfoLastSyncReq;

    ret = kEplSuccessful;
    // prepare SyncReq
    syncReqData.syncControl = EPL_SYNC_PRES_MODE_SET |
                                  EPL_SYNC_PRES_TIME_FIRST_VALID |
                                  EPL_SYNC_PRES_FALL_BACK_TIMEOUT_VALID |
                                  EPL_SYNC_DEST_MAC_ADDRESS_VALID;

    // read object 0x1006 NMT_CycleLen_U32
    obdSize = sizeof(UINT32);
    ret = obd_readEntry(0x1006, 0, &cycleLenUs, &obdSize);
    if (ret != kEplSuccessful)
        goto Exit;

    // read object 0x1C14 DLL_CNLossOfSocTolerance_U32
    ret = obd_readEntry(0x1C14, 0, &cNLossOfSocToleranceNs, &obdSize);
    if (ret != kEplSuccessful)
        goto Exit;

    syncReqData.pResFallBackTimeout = cycleLenUs * 1000 + cNLossOfSocToleranceNs;
    syncReqNum = 0;
    pNodeInfoLastSyncReq = NULL;

    // The search starts with the next node after the previous one
    for (nodeId = nodeIdPrevAdd_p + 1; nodeId <= 254; nodeId++)
    {
        pNodeInfo = NMTMNU_GET_NODEINFO(nodeId);
        if (pNodeInfo == NULL)
            continue;

        if (pNodeInfo->prcFlags & NMTMNU_NODE_FLAG_PRC_ADD_IN_PROGRESS)
        {
            // Send SyncReq which starts PRes Chaining
            syncReqData.nodeId        = nodeId;
            syncReqData.pResTimeFirst = pNodeInfo->pResTimeFirstNs;
            ret = syncu_requestSyncResponse(prcCbSyncResAdd, &syncReqData, sizeof(syncReqData));
            if (ret != kEplSuccessful)
                goto Exit;

            pNodeInfo->prcFlags |= NMTMNU_NODE_FLAG_PRC_ADD_SYNCREQ_SENT;
            syncReqNum++;
            pNodeInfoLastSyncReq = pNodeInfo;

            if (syncReqNum == EPL_NMTMNU_PRC_NODE_ADD_MAX_NUM)
                break;
        }
    }

    if (pNodeInfoLastSyncReq != NULL)
    {
        pNodeInfoLastSyncReq->prcFlags |= NMTMNU_NODE_FLAG_PRC_CALL_ADD;
    }
    else
    {   // No nodes need to be added to the isochronous phase
        if (nodeIdPrevAdd_p != EPL_C_ADR_INVALID)
        {
            pNodeInfo = NMTMNU_GET_NODEINFO(nodeIdPrevAdd_p);
            if (pNodeInfo->prcFlags & NMTMNU_NODE_FLAG_PRC_VERIFY)
            {   // Verification of the last node is still in progress
                // Wait for verify and try again
                pNodeInfo->prcFlags |= NMTMNU_NODE_FLAG_PRC_CALL_ADD;
                goto Exit;
            }
        }

        // Eigher no nodes had to be added, at all, or add is finished
        nmtMnuInstance_g.flags &= ~NMTMNU_FLAG_PRC_ADD_IN_PROGRESS;

        // A new insertion process can be started
        ret = addNodeIsochronous(EPL_C_ADR_INVALID);
    }
Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Callback function for Sync Response for insertion

The function performs the the callback function for SyncRes frames after sending
of SyncReq which is used for insertion.

\param  nodeId_p            Node ID of node.
\param  pSyncResponse_p     Pointer to received SyncRes frame.

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
static tEplKernel PUBLIC prcCbSyncResAdd(UINT nodeId_p, tEplSyncResponse* pSyncResponse_p)
{
    tEplKernel              ret;
    tNmtMnuNodeInfo*        pNodeInfo;

    pNodeInfo = NMTMNU_GET_NODEINFO(nodeId_p);
    if (pSyncResponse_p == NULL)
    {   // SyncRes not received
        // Immediately, schedule reset node
        // because node has already been added to isochronous phase in module Dllk
        pNodeInfo->prcFlags &= ~NMTMNU_NODE_FLAG_PRC_RESET_MASK;
        pNodeInfo->prcFlags |= NMTMNU_NODE_FLAG_PRC_RESET_NODE;
        pNodeInfo->prcFlags &= ~NMTMNU_NODE_FLAG_PRC_ADD_SYNCREQ_SENT;
        goto NextAction;
    }

    // No additional node-added event is received for PRC nodes
    // thus the handler has to be called manually.
    ret = cbNodeAdded(nodeId_p);
    if (ret != kEplSuccessful)
        goto Exit;

    // Flag ISOCHRON has been set in cbNodeAdded,
    // thus this flags are reset.
    pNodeInfo->prcFlags &= ~(NMTMNU_NODE_FLAG_PRC_ADD_IN_PROGRESS |
                                NMTMNU_NODE_FLAG_PRC_ADD_SYNCREQ_SENT);
    // Schedule verify
    pNodeInfo->prcFlags |= NMTMNU_NODE_FLAG_PRC_VERIFY;

NextAction:
    ret = prcCbSyncResNextAction(nodeId_p, pSyncResponse_p);

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Perform verify

The function performs a verify for the phase shift and phase add.

\param  nodeId_p     Node ID of node..

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
static tEplKernel prcVerify(UINT nodeId_p)
{
    tEplKernel              ret;
    tNmtMnuNodeInfo*        pNodeInfo;
    tDllSyncRequest         syncReqData;
    UINT                    size;

    ret = kEplSuccessful;

    pNodeInfo = NMTMNU_GET_NODEINFO(nodeId_p);
    if (pNodeInfo->flags & NMTMNU_NODE_FLAG_ISOCHRON)
    {
        syncReqData.nodeId      = nodeId_p;
        syncReqData.syncControl = EPL_SYNC_DEST_MAC_ADDRESS_VALID;
        size = sizeof(UINT) + sizeof(UINT32);
        ret = syncu_requestSyncResponse(prcCbSyncResVerify, &syncReqData, size);
    }
    else
    {   // Node has been removed by a reset-node NMT command
        // Verification is no longer necessary
        pNodeInfo->prcFlags &= ~NMTMNU_NODE_FLAG_PRC_VERIFY;
    }
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Callback function for Sync Response for verification

The function performs the the callback function for SyncRes frames after sending
of SyncReq which is used for verification.

\param  nodeId_p            Node ID of node.
\param  pSyncResponse_p     Pointer to received SyncRes frame.

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
static tEplKernel prcCbSyncResVerify(UINT nodeId_p, tEplSyncResponse* pSyncResponse_p)
{
    tEplKernel              ret;
    tNmtMnuNodeInfo*        pNodeInfo;
    UINT32                  pResTimeFirstNs;

    pNodeInfo = NMTMNU_GET_NODEINFO(nodeId_p);

    if (pSyncResponse_p == NULL)
    {   // SyncRes not received
        prcSyncError(pNodeInfo);
        goto Exit;
    }

    pResTimeFirstNs = AmiGetDwordFromLe(&pSyncResponse_p->m_le_dwPResTimeFirst);

    if (pResTimeFirstNs != pNodeInfo->pResTimeFirstNs)
    {   // Configuration of PRes Response Time was not successful
        // Schedule reset node
        pNodeInfo->prcFlags &= ~NMTMNU_NODE_FLAG_PRC_RESET_MASK;
        pNodeInfo->prcFlags |= NMTMNU_NODE_FLAG_PRC_RESET_NODE;
        goto Exit;
    }

    // Verification was successful
    pNodeInfo->prcFlags &= ~NMTMNU_NODE_FLAG_PRC_VERIFY;

    // If a previous SyncRes frame was not usable,
    // the Sync Error flag is cleared as this one is OK
    pNodeInfo->prcFlags &= ~NMTMNU_NODE_FLAG_PRC_SYNC_ERR;

Exit:
    ret = prcCbSyncResNextAction(nodeId_p, pSyncResponse_p);

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Callback function for Sync Response without action

The function performs the the callback function for SyncRes frames after sending
of SyncReq which is used if no specific handling is required. The next-action
node flags are evaluated.

\param  nodeId_p            Node ID of node.
\param  pSyncResponse_p     Pointer to received SyncRes frame.

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
static tEplKernel prcCbSyncResNextAction(UINT nodeId_p, tEplSyncResponse* pSyncResponse_p)
{
    tEplKernel              ret;
    tNmtMnuNodeInfo*        pNodeInfo;
    tNmtCommand             nmtCommand;

    UNUSED_PARAMETER(pSyncResponse_p);
    ret = kEplSuccessful;

    pNodeInfo = NMTMNU_GET_NODEINFO(nodeId_p);
    switch (pNodeInfo->prcFlags & NMTMNU_NODE_FLAG_PRC_RESET_MASK)
    {
        case NMTMNU_NODE_FLAG_PRC_STOP_NODE:
            nmtCommand = kNmtCmdStopNode;
            break;

        case NMTMNU_NODE_FLAG_PRC_RESET_NODE:
            nmtCommand = kNmtCmdResetNode;
            break;

        case NMTMNU_NODE_FLAG_PRC_RESET_COM:
            nmtCommand = kNmtCmdResetCommunication;
            break;

        case NMTMNU_NODE_FLAG_PRC_RESET_CONF:
            nmtCommand = kNmtCmdResetConfiguration;
            break;

        case NMTMNU_NODE_FLAG_PRC_RESET_SW:
            nmtCommand = kNmtCmdSwReset;
            break;

        default:
            nmtCommand = kNmtCmdInvalidService;
            break;
    }

    pNodeInfo->prcFlags &= ~NMTMNU_NODE_FLAG_PRC_RESET_MASK;

    if (nmtCommand != kNmtCmdInvalidService)
    {
        ret = nmtmnu_sendNmtCommand(nodeId_p, nmtCommand);
        if (ret != kEplSuccessful)
            goto Exit;
    }

    switch (pNodeInfo->prcFlags & NMTMNU_NODE_FLAG_PRC_CALL_MASK)
    {
        case NMTMNU_NODE_FLAG_PRC_CALL_MEASURE:
            pNodeInfo->prcFlags &= ~NMTMNU_NODE_FLAG_PRC_CALL_MASK;
            ret = prcMeasure();
            goto Exit;

        case NMTMNU_NODE_FLAG_PRC_CALL_SHIFT:
            pNodeInfo->prcFlags &= ~NMTMNU_NODE_FLAG_PRC_CALL_MASK;
            ret = prcShift(nodeId_p);
            if (ret != kEplSuccessful)
                goto Exit;
            break;

        case NMTMNU_NODE_FLAG_PRC_CALL_ADD:
            pNodeInfo->prcFlags &= ~NMTMNU_NODE_FLAG_PRC_CALL_MASK;
            ret = prcAdd(nodeId_p);
            if (ret != kEplSuccessful)
                goto Exit;
            break;

        default:
            break;
    }

    if (pNodeInfo->prcFlags & NMTMNU_NODE_FLAG_PRC_VERIFY)
    {
        ret = prcVerify(nodeId_p);
    }
Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Set PRC reset flags

Before sending a reset-node NMT command, PRes Chaining has to be disabled by
sending an appropriate SyncReq. The requested NMT command is stored until the
SyncRes returns. Commands of higher priority overwrite those of lower priority.

\param  pNodeInfo_p     Pointer to node information structure.
\param  nmtCommand_p    NMT command.
*/
//------------------------------------------------------------------------------
static void prcSetFlagsNmtCommandReset(tNmtMnuNodeInfo* pNodeInfo_p,
                                       tNmtCommand nmtCommand_p)
{
    UINT16 prcFlagsReset;

    prcFlagsReset = pNodeInfo_p->prcFlags & NMTMNU_NODE_FLAG_PRC_RESET_MASK;

    switch (nmtCommand_p)
    {
        case kNmtCmdResetNode:
            prcFlagsReset = NMTMNU_NODE_FLAG_PRC_RESET_NODE;
            break;

        case kNmtCmdResetCommunication:
            switch (prcFlagsReset)
            {
                case NMTMNU_NODE_FLAG_PRC_RESET_CONF:
                case NMTMNU_NODE_FLAG_PRC_RESET_SW:
                case NMTMNU_NODE_FLAG_PRC_STOP_NODE:
                case 0:
                    prcFlagsReset = NMTMNU_NODE_FLAG_PRC_RESET_COM;
                    break;

                default:
                    break;
            }
            break;

        case kNmtCmdResetConfiguration:
            switch (prcFlagsReset)
            {
                case NMTMNU_NODE_FLAG_PRC_RESET_SW:
                case NMTMNU_NODE_FLAG_PRC_STOP_NODE:
                case 0:
                    prcFlagsReset = NMTMNU_NODE_FLAG_PRC_RESET_CONF;
                    break;

                default:
                    break;
            }
            break;

        case kNmtCmdSwReset:
            switch (prcFlagsReset)
            {
                case NMTMNU_NODE_FLAG_PRC_STOP_NODE:
                case 0:
                    prcFlagsReset = NMTMNU_NODE_FLAG_PRC_RESET_SW;
                    break;

                default:
                    break;
            }
            break;

        case kNmtCmdStopNode:
            if (prcFlagsReset == 0)
            {
                prcFlagsReset = NMTMNU_NODE_FLAG_PRC_STOP_NODE;
            }
            break;

        default:
            break;
    }

    pNodeInfo_p->prcFlags &= ~NMTMNU_NODE_FLAG_PRC_RESET_MASK;
    pNodeInfo_p->prcFlags |= prcFlagsReset;

    return;
}

#endif // #if EPL_NMTMNU_PRES_CHAINING_MN != FALSE

#endif // #if defined(CONFIG_INCLUDE_NMT_MN)

///\}

