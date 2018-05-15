/**
********************************************************************************
\file   nmtmnu.c

\brief  Implementation of NMT MNU module

This file contains the implementation of the NMT MNU module.

\ingroup module_nmtmnu
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2015, SYSTEC electronic GmbH
Copyright (c) 2017, B&R Industrial Automation GmbH
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
#include <stddef.h>

#include <common/oplkinc.h>
#include <user/nmtmnu.h>
#include <user/timeru.h>
#include <user/dllucal.h>
#include <user/identu.h>
#include <user/statusu.h>
#include <user/syncu.h>
#include <user/eventu.h>
#include <common/ami.h>
#include <user/obdu.h>
#include <oplk/frame.h>
#include <oplk/benchmark.h>

//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

#if defined(CONFIG_INCLUDE_NMT_MN)
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

#define NMTMNU_DBG_POST_TRACE_VALUE(event_p, nodeId_p, errorCode_p)               \
    DEBUG_LVL_NMTMN_TRACE("nmtmnu: event: %02X, nodeId: %02X, errorCode: %04X\n", \
                          event_p,                                                \
                          nodeId_p,                                               \
                          errorCode_p);

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
    timerArg_p.argument.value = NMTMNU_TIMERARG_STATREQ | nodeId_p  |                   \
                                (pNodeInfo_p->flags & NMTMNU_NODE_FLAG_COUNT_STATREQ);  \
    timerArg_p.eventSink  = kEventSinkNmtMnu;

#define NMTMNU_SET_FLAGS_TIMERARG_IDENTREQ(pNodeInfo_p, nodeId_p, timerArg_p)           \
    pNodeInfo_p->flags = ((pNodeInfo_p->flags + NMTMNU_NODE_FLAG_INC_STATREQ) &         \
                          NMTMNU_NODE_FLAG_COUNT_STATREQ) |                             \
                          (pNodeInfo_p->flags & ~NMTMNU_NODE_FLAG_COUNT_STATREQ);       \
    timerArg_p.argument.value = NMTMNU_TIMERARG_IDENTREQ | nodeId_p |                   \
                                (pNodeInfo_p->flags & NMTMNU_NODE_FLAG_COUNT_STATREQ);  \
    timerArg_p.eventSink = kEventSinkNmtMnu;

#define NMTMNU_SET_FLAGS_TIMERARG_LONGER(pNodeInfo_p, nodeId_p, timerArg_p)             \
    pNodeInfo_p->flags = ((pNodeInfo_p->flags + NMTMNU_NODE_FLAG_INC_LONGER) &          \
                          NMTMNU_NODE_FLAG_COUNT_LONGER) |                              \
                          (pNodeInfo_p->flags & ~NMTMNU_NODE_FLAG_COUNT_LONGER);        \
    timerArg_p.argument.value = NMTMNU_TIMERARG_LONGER | nodeId_p |                     \
                                (pNodeInfo_p->flags & NMTMNU_NODE_FLAG_COUNT_LONGER);   \
    timerArg_p.eventSink = kEventSinkNmtMnu;

#define NMTMNU_SET_FLAGS_TIMERARG_STATE_MON(pNodeInfo_p, nodeId_p, timerArg_p)          \
    pNodeInfo_p->flags = ((pNodeInfo_p->flags + NMTMNU_NODE_FLAG_INC_STATREQ) &         \
                          NMTMNU_NODE_FLAG_COUNT_STATREQ) |                             \
                          (pNodeInfo_p->flags & ~NMTMNU_NODE_FLAG_COUNT_STATREQ);       \
    timerArg_p.argument.value = NMTMNU_TIMERARG_STATE_MON | nodeId_p |                  \
                                (pNodeInfo_p->flags & NMTMNU_NODE_FLAG_COUNT_STATREQ);  \
    timerArg_p.eventSink = kEventSinkNmtMnu;

// defines for global flags
#define NMTMNU_FLAG_HALTED                      0x0001  // boot process is halted
#define NMTMNU_FLAG_APP_INFORMED                0x0002  // application was informed about possible NMT state change
#define NMTMNU_FLAG_USER_RESET                  0x0004  // NMT reset issued by user / diagnostic node
#define NMTMNU_FLAG_PRC_ADD_SCHEDULED           0x0008  // at least one node is scheduled
                                                        // for addition to isochronous phase
#define NMTMNU_FLAG_PRC_ADD_IN_PROGRESS         0x0010  // add-PRC-node process is in progress
#define NMTMNU_FLAG_REDUNDANCY                  0x0020  // redundancy flag

// return pointer to node info structure for specified node ID
// d.k. may be replaced by special (hash) function if node ID array is smaller than 254
#define NMTMNU_GET_NODEINFO(nodeId_p) (&nmtMnuInstance_g.aNodeInfo[nodeId_p - 1])

//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------

typedef struct
{
    UINT8               cmdData;
    UINT                cmdDataId;
    UINT                nodeIdCnt;
    UINT8               nodeIdMask;
    UINT                nodeId;
} tNmtMnuGetNodeId;

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

/**
* \brief Enumeration for internal node events
*
* This enumeration specifies all internal node events.
*
* Do not change the constants as the array with function pointers to the
* handlers depends on these constants!
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
    kNmtMnuIntNodeEventSwUpdated            = 0x0F,
    kNmtMnuIntNodeEventSwOk                 = 0x10,
} eNmtMnuIntNodeEvent;

/**
\brief NMT MN internal node event data type

Data type for the enumerator \ref eNmtMnuIntNodeEvent.
*/
typedef UINT32 tNmtMnuIntNodeEvent;


/**
* \brief Enumeration for node states
*
* This enumeration lists valid node states.
*/
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
    kNmtMnuNodeStateSwOk                    = 0x08, // CN software updated or already up-to-date
} eNmtMnuNodeState;

/**
\brief NMT MN node state data type

Data type for the enumerator \ref eNmtMnuNodeState.
*/
typedef UINT32 tNmtMnuNodeState;

typedef INT (*tProcessNodeEventFunc)(UINT nodeId_p,
                                     tNmtState nodeNmtState_p,
                                     tNmtState nmtState_p,
                                     UINT16 errorCode_p,
                                     tOplkError* pRet_p);

/**
* \brief Node information structure
*
* The following struct specifies a node.
*/
typedef struct
{
    tTimerHdl           timerHdlStatReq;        ///< Timer to delay StatusRequests and IdentRequests
    tTimerHdl           timerHdlLonger;         ///< 2nd timer for NMT command EnableReadyToOp and CheckCommunication
    tNmtMnuNodeState    nodeState;              ///< Internal node state (kind of sub state of NMT state)
    UINT32              nodeCfg;                ///< Subindex from 0x1F81
    UINT16              flags;                  ///< Node flags (see node flag defines)
    UINT16              prcFlags;               ///< PRC specific node flags
    UINT32              relPropagationDelayNs;  ///< Propagation delay in nanoseconds
    UINT32              pResTimeFirstNs;        ///< PRes time
    BOOL                fPrcSupportIsMissing;   ///< A node configured to be used for PRC is not supporting it
    UINT32              nodeCfgBackup;          ///< Backup of nodeCfg member is used if fPrcSupportIsMissing is TRUE
} tNmtMnuNodeInfo;

/**
* \brief nmtmnu instance structure
*
* The following struct implements the instance information of the NMT MNU module.
*/
typedef struct
{
    tNmtMnuNodeInfo     aNodeInfo[NMT_MAX_NODE_ID];     ///< Information about CNs
    tTimerHdl           timerHdlNmtState;               ///< Timeout for stay in NMT state
    UINT                mandatorySlaveCount;            ///< Count of found mandatory CNs
    UINT                signalSlaveCount;               ///< Count of CNs which are not identified
    ULONG               statusRequestDelay;             ///< In [ms] (object 0x1006 * C_NMT_STATREQ_CYCLE)
    ULONG               timeoutReadyToOp;               ///< In [ms] (object 0x1F89/4)
    ULONG               timeoutCheckCom;                ///< In [ms] (object 0x1006 * MultiplexedCycleCount)
    UINT16              flags;                          ///< Global flags
    UINT32              nmtStartup;                     ///< Object 0x1F80 NMT_StartUp_U32
    tNmtMnuCbNodeEvent  pfnCbNodeEvent;                 ///< Callback function for node events
    tNmtMnuCbBootEvent  pfnCbBootEvent;                 ///< Callback function for boot events
    UINT32              prcPResMnTimeoutNs;             ///< to be commented!
    UINT32              prcPResTimeFirstCorrectionNs;   ///< to be commented!
    UINT32              prcPResTimeFirstNegOffsetNs;    ///< to be commented!
} tNmtMnuInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tNmtMnuInstance   nmtMnuInstance_g;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static tOplkError cbNmtRequest(const tFrameInfo* pFrameInfo_p);
static tOplkError cbIdentResponse(UINT nodeId_p,
                                  const tIdentResponse* pIdentResponse_p);
static tOplkError cbStatusResponse(UINT nodeId_p,
                                   const tStatusResponse* pStatusResponse_p);
static tOplkError cbNodeAdded(UINT nodeId_p);
static tOplkError checkNmtState(UINT nodeId_p,
                                tNmtMnuNodeInfo* pNodeInfo_p,
                                tNmtState nodeNmtState_p,
                                UINT16 errorCode_p,
                                tNmtState localNmtState_p);
static tOplkError addNodeIsochronous(UINT nodeId_p);
static tOplkError startBootStep1(BOOL fNmtResetAllIssued_p);

#if defined(CONFIG_INCLUDE_NMT_RMN)
static tOplkError resetRedundancy(void);
static tOplkError switchoverRedundancy(void);
static tOplkError processRedundancyHeartbeat(UINT nodeId_p,
                                             tNmtState nodeNmtState_p);
#endif

static tOplkError startBootStep2(void);
static tOplkError startCheckCom(void);
static tOplkError nodeBootStep2(UINT nodeId_p,
                                tNmtMnuNodeInfo* pNodeInfo_p);
static tOplkError nodeCheckCom(UINT nodeId_p,
                               tNmtMnuNodeInfo* pNodeInfo_p);
static tOplkError startNodes(void);
static tOplkError doPreop1(tEventNmtStateChange nmtStateChange_p);
static tOplkError processInternalEvent(UINT nodeId_p,
                                       tNmtState nodeNmtState_p,
                                       UINT16 errorCode_p,
                                       tNmtMnuIntNodeEvent nodeEvent_p);
static tOplkError reset(void);

static tOplkError prcMeasure(void);
static tOplkError prcCalculate(UINT nodeIdFirstNode_p);
static tOplkError prcShift(UINT nodeIdPrevShift_p);
static tOplkError prcAdd(UINT nodeIdPrevAdd_p);
static tOplkError prcVerify(UINT nodeId_p);

static tOplkError prcCbSyncResMeasure(UINT nodeId_p,
                                      const tSyncResponse* pSyncResponse_p);
static tOplkError prcCbSyncResShift(UINT nodeId_p,
                                    const tSyncResponse* pSyncResponse_p);
static tOplkError prcCbSyncResAdd(UINT nodeId_p,
                                  const tSyncResponse* pSyncResponse_p);
static tOplkError prcCbSyncResVerify(UINT nodeId_p,
                                     const tSyncResponse* pSyncResponse_p);
static tOplkError prcCbSyncResNextAction(UINT nodeId_p,
                                         const tSyncResponse* pSyncResponse_p);

static tOplkError prcCalcPResResponseTimeNs(UINT nodeId_p,
                                            UINT nodeIdPredNode_p,
                                            UINT32* pPResResponseTimeNs_p);
static tOplkError prcCalcPResChainingSlotTimeNs(UINT nodeIdLastNode_p,
                                                UINT32* pPResChainingSlotTimeNs_p);

static UINT       prcFindPredecessorNode(UINT nodeId_p);
static void       prcSyncError(tNmtMnuNodeInfo* pNodeInfo_p);
static void       prcSetFlagsNmtCommandReset(tNmtMnuNodeInfo* pNodeInfo_p,
                                             tNmtCommand nmtCommand_p);
static tOplkError prcHandleNmtReset(UINT nodeId_p,
                                    tNmtCommand nmtCommand_p,
                                    BOOL* pfWaitForSyncResp_p);

static tOplkError sendNmtCommand(UINT nodeId_p,
                                 tNmtCommand nmtCommand_p,
                                 const UINT8* pNmtCommandData_p,
                                 size_t dataSize_p);

static tOplkError getNodeIdFromCmd(UINT nodeId_p,
                                   tNmtCommand nmtCommand_p,
                                   const UINT8* pCmdData_p,
                                   tNmtMnuGetNodeId* pOp_p,
                                   UINT* pNodeId_p);
static tOplkError nodeListToNodeId(const UINT8* pCmdData_p,
                                   tNmtMnuGetNodeId* pOp_p,
                                   UINT* pNodeId_p);
static tOplkError removeNodeIdFromExtCmd(UINT nodeId_p,
                                         const UINT8* pCmdData_p,
                                         size_t size_p);

static ULONG      computeCeilDiv(ULONG numerator_p,
                                 ULONG denominator_p);

static tNmtState  correctNmtState(UINT8 nmtState_p);

static void handleMissingPrcSupport(UINT nodeId_p, UINT32 featureFlags_p);

/* internal node event handler functions */
static INT processNodeEventNoIdentResponse(UINT nodeId_p,
                                           tNmtState nodeNmtState_p,
                                           tNmtState nmtState_p,
                                           UINT16 errorCode_p,
                                           tOplkError* pRet_p);
static INT processNodeEventIdentResponse(UINT nodeId_p,
                                         tNmtState nodeNmtState_p,
                                         tNmtState nmtState_p,
                                         UINT16 errorCode_p,
                                         tOplkError* pRet_p);
static INT processNodeEventBoot(UINT nodeId_p,
                                tNmtState nodeNmtState_p,
                                tNmtState nmtState_p,
                                UINT16 errorCode_p,
                                tOplkError* pRet_p);
static INT processNodeEventExecResetConf(UINT nodeId_p,
                                         tNmtState nodeNmtState_p,
                                         tNmtState nmtState_p,
                                         UINT16 errorCode_p,
                                         tOplkError* pRet_p);
static INT processNodeEventExecResetNode(UINT nodeId_p,
                                         tNmtState nodeNmtState_p,
                                         tNmtState nmtState_p,
                                         UINT16 errorCode_p,
                                         tOplkError* pRet_p);
static INT processNodeEventConfigured(UINT nodeId_p,
                                      tNmtState nodeNmtState_p,
                                      tNmtState nmtState_p,
                                      UINT16 errorCode_p,
                                      tOplkError* pRet_p);
static INT processNodeEventNoStatusResponse(UINT nodeId_p,
                                            tNmtState nodeNmtState_p,
                                            tNmtState nmtState_p,
                                            UINT16 errorCode_p,
                                            tOplkError* pRet_p);
static INT processNodeEventStatusResponse(UINT nodeId_p,
                                          tNmtState nodeNmtState_p,
                                          tNmtState nmtState_p,
                                          UINT16 errorCode_p,
                                          tOplkError* pRet_p);
static INT processNodeEventHeartbeat(UINT nodeId_p,
                                     tNmtState nodeNmtState_p,
                                     tNmtState nmtState_p,
                                     UINT16 errorCode_p,
                                     tOplkError* pRet_p);
static INT processNodeEventNmtCmdSent(UINT nodeId_p,
                                      tNmtState nodeNmtState_p,
                                      tNmtState nmtState_p,
                                      UINT16 errorCode_p,
                                      tOplkError* pRet_p);
static INT processNodeEventTimerIdentReq(UINT nodeId_p,
                                         tNmtState nodeNmtState_p,
                                         tNmtState nmtState_p,
                                         UINT16 errorCode_p,
                                         tOplkError* pRet_p);
static INT processNodeEventTimerStatReq(UINT nodeId_p,
                                        tNmtState nodeNmtState_p,
                                        tNmtState nmtState_p,
                                        UINT16 errorCode_p,
                                        tOplkError* pRet_p);
static INT processNodeEventTimerStateMon(UINT nodeId_p,
                                         tNmtState nodeNmtState_p,
                                         tNmtState nmtState_p,
                                         UINT16 errorCode_p,
                                         tOplkError* pRet_p);
static INT processNodeEventTimerLonger(UINT nodeId_p,
                                       tNmtState nodeNmtState_p,
                                       tNmtState nmtState_p,
                                       UINT16 errorCode_p,
                                       tOplkError* pRet_p);
static INT processNodeEventError(UINT nodeId_p,
                                 tNmtState nodeNmtState_p,
                                 tNmtState nmtState_p,
                                 UINT16 errorCode_p,
                                 tOplkError* pRet_p);
static INT processNodeEventSwUpdated(UINT nodeId_p,
                                     tNmtState nodeNmtState_p,
                                     tNmtState nmtState_p,
                                     UINT16 errorCode_p,
                                     tOplkError* pRet_p);
static INT processNodeEventSwOk(UINT nodeId_p,
                                tNmtState nodeNmtState_p,
                                tNmtState nmtState_p,
                                UINT16 errorCode_p,
                                tOplkError* pRet_p);

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
/*
The following function table depends on the types defined in tNmtMnuIntNodeEvent.
Do not re-order them without adapting the constants!
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
    processNodeEventError,              // kNmtMnuIntNodeEventError
    processNodeEventSwUpdated,          // kNmtMnuIntNodeEventSwUpdated
    processNodeEventSwOk,               // kNmtMnuIntNodeEventSwOk
};

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//


//------------------------------------------------------------------------------
/**
\brief  Init nmtmnu module

The function initializes an instance of the nmtmnu module

\param[in]      pfnCbNodeEvent_p    Pointer to node event callback function.
\param[in]      pfnCbBootEvent_p    Pointer to boot event callback function.

\return The function returns a tOplkError error code.

\ingroup module_nmtmnu
*/
//------------------------------------------------------------------------------
tOplkError nmtmnu_init(tNmtMnuCbNodeEvent pfnCbNodeEvent_p,
                       tNmtMnuCbBootEvent pfnCbBootEvent_p)
{
    tOplkError  ret;

    OPLK_MEMSET(&nmtMnuInstance_g, 0, sizeof(nmtMnuInstance_g));

    if ((pfnCbNodeEvent_p == NULL) || (pfnCbBootEvent_p == NULL))
    {
        ret = kErrorNmtInvalidParam;
        goto Exit;
    }

    nmtMnuInstance_g.pfnCbNodeEvent = pfnCbNodeEvent_p;
    nmtMnuInstance_g.pfnCbBootEvent = pfnCbBootEvent_p;
    nmtMnuInstance_g.statusRequestDelay = 5000L;

    // register NmtMnResponse callback function

    // NMT requests are targeted to node-ID 240 according to specification.
    // RMNs have node-IDs unequal 240. So the filter is opened for NMT
    // requests targeted to any node-ID.
    ret = dllucal_regAsndService(kDllAsndNmtRequest, cbNmtRequest, kDllAsndFilterAny);

    nmtMnuInstance_g.prcPResTimeFirstCorrectionNs = 50;
    nmtMnuInstance_g.prcPResTimeFirstNegOffsetNs = 500;

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Shutdown nmtmnu module instance

The function shuts down the nmtmnu module instance.

\return The function returns a tOplkError error code.

\ingroup module_nmtmnu
*/
//------------------------------------------------------------------------------
tOplkError nmtmnu_exit(void)
{
    tOplkError  ret;

    dllucal_regAsndService(kDllAsndNmtRequest, NULL, kDllAsndFilterNone);
    ret = reset();

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Send extended NMT command

The function sends an extended NMT command.

\param[in]      nodeId_p            Node ID to which the NMT command will be sent.
\param[in]      nmtCommand_p        NMT command to send.
\param[in]      pNmtCommandData_p   Pointer to additional NMT command data.
\param[in]      dataSize_p          Length of additional NMT command data.

\return The function returns a tOplkError error code.

\ingroup module_nmtmnu
*/
//------------------------------------------------------------------------------
tOplkError nmtmnu_sendNmtCommandEx(UINT nodeId_p,
                                   tNmtCommand nmtCommand_p,
                                   const void* pNmtCommandData_p,
                                   size_t dataSize_p)
{
    tOplkError          ret = kErrorOk;
    tDllNodeOpParam     nodeOpParam;
    tNmtMnuNodeInfo*    pNodeInfo;
    UINT                tempNodeId;
    const UINT8*        pCmdData = (const UINT8*)pNmtCommandData_p;
    tNmtMnuGetNodeId    nodeListOp;
    tOplkError          retGetNodeId;
    BOOL                fIsExtNmtCmd;
    UINT                dstNodeCnt;

    if (!NMT_IF_ACTIVE_MN(nmtu_getNmtState()))
    {
        ret = kErrorInvalidOperation;
        goto Exit;
    }

    if ((nodeId_p == 0) || (nodeId_p > C_ADR_BROADCAST))
    {   // invalid node ID specified
        ret = kErrorInvalidNodeId;
        goto Exit;
    }

    // Clip given size to extended NMT command size
    dataSize_p = min(dataSize_p, (size_t)(C_DLL_MINSIZE_NMTCMDEXT - C_DLL_MINSIZE_NMTCMD));

    // Set flag if it is extended NMT command
    fIsExtNmtCmd = (nmtCommand_p >= NMT_EXT_COMMAND_START && nmtCommand_p <= NMT_EXT_COMMAND_END);

    // $$$ d.k. may be check in future versions if the caller wants to perform
    //          prohibited state transitions. The CN should not perform these
    //          transitions, but the expected NMT state will be changed and never fulfilled.

    if (fIsExtNmtCmd || (nodeId_p < C_ADR_BROADCAST))
    {
        // Handle plain extended NMT and individual plain NMT commands
        tempNodeId = C_ADR_INVALID;
        OPLK_MEMSET(&nodeListOp, 0x00, sizeof(nodeListOp));
        retGetNodeId = getNodeIdFromCmd(nodeId_p, nmtCommand_p, pCmdData, &nodeListOp, &tempNodeId);

        dstNodeCnt = 0;
        while (retGetNodeId == kErrorRetry)
        {
            BOOL fRemoveNmtCmd = FALSE;

            ret = prcHandleNmtReset(tempNodeId, nmtCommand_p, &fRemoveNmtCmd);
            if (ret != kErrorOk)
                goto Exit;

            if (fRemoveNmtCmd)
            {
                // Remove from extended NMT command
                if (fIsExtNmtCmd)
                {
                    ret = removeNodeIdFromExtCmd(tempNodeId, pCmdData, dataSize_p);
                    if (ret != kErrorOk)
                        goto Exit;
                }
            }
            else
            {
                // Destination may receive NMT command
                dstNodeCnt++;
            }

            retGetNodeId = getNodeIdFromCmd(nodeId_p,
                                            nmtCommand_p,
                                            pCmdData,
                                            &nodeListOp,
                                            &tempNodeId);
        }
    }
    else
    {
        // Handle plain broadcast NMT command
        dstNodeCnt = C_ADR_BROADCAST;
    }

    if (dstNodeCnt > 0)
    {
        ret = sendNmtCommand(nodeId_p, nmtCommand_p, pCmdData, dataSize_p);
        if (ret != kErrorOk)
            goto Exit;
    }
    else
    {
        // No destination node is interested into this NMT command.
        // E.g. wait for syncResponse(s)
        goto Exit;
    }

    DEBUG_LVL_NMTMN_TRACE("NMTCmd(%02X->%02X)\n", nmtCommand_p, nodeId_p);

    switch (nmtCommand_p)
    {
        case kNmtCmdStartNode:
        case kNmtCmdStartNodeEx:
        case kNmtCmdEnterPreOperational2:
        case kNmtCmdEnterPreOperational2Ex:
        case kNmtCmdEnableReadyToOperate:
        case kNmtCmdEnableReadyToOperateEx:
            // nothing left to do,
            // because any further processing is done
            // when the NMT command is actually sent
            goto Exit;

        case kNmtCmdStopNode:
        case kNmtCmdStopNodeEx:
            // remove CN from isochronous phase softly
            nodeOpParam.opNodeType = kDllNodeOpTypeSoftDelete;
            break;

        case kNmtCmdResetNode:
        case kNmtCmdResetNodeEx:
        case kNmtCmdResetCommunication:
        case kNmtCmdResetCommunicationEx:
        case kNmtCmdResetConfiguration:
        case kNmtCmdResetConfigurationEx:
        case kNmtCmdSwReset:
        case kNmtCmdSwResetEx:
            // remove CN immediately from isochronous phase
            nodeOpParam.opNodeType = kDllNodeOpTypeIsochronous;
            break;

        default:
            goto Exit;
    }

    // The expected node state will be updated when the NMT command
    // was actually sent.
    // See functions processInternalEvent(kNmtMnuIntNodeEventNmtCmdSent),
    // nmtmnu_processEvent(kEventTypeNmtMnuNmtCmdSent).

    // remove CN from isochronous phase;
    // This must be done here and not when NMT command is actually sent
    // because it will be too late and may cause unwanted errors

    tempNodeId = C_ADR_INVALID;
    OPLK_MEMSET(&nodeListOp, 0x00, sizeof(nodeListOp));
    retGetNodeId = getNodeIdFromCmd(nodeId_p, nmtCommand_p, pCmdData, &nodeListOp, &tempNodeId);

    while (retGetNodeId == kErrorRetry)
    {
        pNodeInfo = NMTMNU_GET_NODEINFO(tempNodeId);

        if (!(pNodeInfo->nodeCfg & NMT_NODEASSIGN_PRES_CHAINING))
        {   // Remove non-PRC nodes from isochronous phase
            if ((pNodeInfo->nodeCfg & (NMT_NODEASSIGN_NODE_IS_CN | NMT_NODEASSIGN_NODE_EXISTS)))
            {
                nodeOpParam.nodeId = tempNodeId;
                ret = dllucal_deleteNode(&nodeOpParam);
            }
        }

        retGetNodeId = getNodeIdFromCmd(nodeId_p,
                                        nmtCommand_p,
                                        pCmdData,
                                        &nodeListOp,
                                        &tempNodeId);
    }

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Send NMT command

The function sends an NMT command.

\param[in]      nodeId_p            Node ID to which the NMT command will be sent.
\param[in]      nmtCommand_p        NMT command to send.

\return The function returns a tOplkError error code.

\ingroup module_nmtmnu
*/
//------------------------------------------------------------------------------
tOplkError nmtmnu_sendNmtCommand(UINT nodeId_p, tNmtCommand nmtCommand_p)
{
    return nmtmnu_sendNmtCommandEx(nodeId_p, nmtCommand_p, NULL, 0);
}

//------------------------------------------------------------------------------
/**
\brief  Request NMT command

The function requests the specified NMT command for the specified node. It may
also be applied to the local node.

\param[in]      nodeId_p            Node ID for which the NMT command will be requested.
\param[in]      nmtCommand_p        NMT command to request.
\param[in]      pNmtCommandData_p   Pointer to NMT command data (32 Byte).
\param[in]      dataSize_p          Size of NMT command data.

\return The function returns a tOplkError error code.

\ingroup module_nmtmnu
*/
//------------------------------------------------------------------------------
tOplkError nmtmnu_requestNmtCommand(UINT nodeId_p,
                                    tNmtCommand nmtCommand_p,
                                    const void* pNmtCommandData_p,
                                    size_t dataSize_p)
{
    tOplkError  ret = kErrorOk;
    tNmtState   nmtState;

    nmtState = nmtu_getNmtState();
    if (!NMT_IF_ACTIVE_MN(nmtState))
    {
        ret = kErrorInvalidOperation;
        goto Exit;
    }

    if (nodeId_p > C_ADR_BROADCAST)
    {
        ret = kErrorInvalidNodeId;
        goto Exit;
    }

    if (nodeId_p == 0x00)
        nodeId_p = C_ADR_MN_DEF_NODE_ID;

    if (nmtCommand_p == kNmtCmdGoToStandby)
    {
        UINT8       flags = 0;
        tNmtEvent   nmtEvent;

        if (pNmtCommandData_p && (dataSize_p >= 1))
            flags = ami_getUint8Le(pNmtCommandData_p);

        if (flags && NMT_CMD_DATA_FLAG_DELAY)
            nmtEvent = kNmtEventGoToStandbyDelayed;
        else
            nmtEvent = kNmtEventGoToStandby;

        ret = nmtu_postNmtEvent(nmtEvent);
        goto Exit;
    }

    if (nodeId_p == C_ADR_MN_DEF_NODE_ID)
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
                nodeId_p = C_ADR_BROADCAST;
                break;

            case kNmtCmdInvalidService:
            default:
                ret = kErrorObdAccessViolation;
                goto Exit;
        }
    }

    if (nodeId_p != C_ADR_BROADCAST)
    {   // apply command to remote node-ID, but not broadcast
        const tNmtMnuNodeInfo* pNodeInfo = NMTMNU_GET_NODEINFO(nodeId_p);

        switch (nmtCommand_p)
        {
            case kNmtCmdIdentResponse:
                // issue request for remote node
                // if it is a non-existing node or no IdentRequest is running
                if (((pNodeInfo->nodeCfg & (NMT_NODEASSIGN_NODE_IS_CN | NMT_NODEASSIGN_NODE_EXISTS)) !=
                     (NMT_NODEASSIGN_NODE_IS_CN | NMT_NODEASSIGN_NODE_EXISTS)) ||
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
                if (((pNodeInfo->nodeCfg & (NMT_NODEASSIGN_NODE_IS_CN | NMT_NODEASSIGN_NODE_EXISTS)) !=
                     (NMT_NODEASSIGN_NODE_IS_CN | NMT_NODEASSIGN_NODE_EXISTS)) ||
                    (((pNodeInfo->nodeCfg & NMT_NODEASSIGN_ASYNCONLY_NODE) == 0) &&
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
        case kNmtCmdResetNodeEx:
        case kNmtCmdResetCommunication:
        case kNmtCmdResetCommunicationEx:
        case kNmtCmdResetConfiguration:
        case kNmtCmdResetConfigurationEx:
        case kNmtCmdSwReset:
        case kNmtCmdSwResetEx:
            if (nodeId_p == C_ADR_BROADCAST)
            {   // memorize that this is a user requested reset
                nmtMnuInstance_g.flags |= NMTMNU_FLAG_USER_RESET;
            }
            break;

        case kNmtCmdStartNode:
        case kNmtCmdStartNodeEx:
        case kNmtCmdStopNode:
        case kNmtCmdStopNodeEx:
        case kNmtCmdEnterPreOperational2:
        case kNmtCmdEnterPreOperational2Ex:
        case kNmtCmdEnableReadyToOperate:
        case kNmtCmdEnableReadyToOperateEx:
        default:
            break;
    }

    // send command to remote node
    if ((pNmtCommandData_p != NULL) && (dataSize_p != 0))
        ret = nmtmnu_sendNmtCommandEx(nodeId_p, nmtCommand_p, pNmtCommandData_p, dataSize_p);
    else
        ret = nmtmnu_sendNmtCommand(nodeId_p, nmtCommand_p);

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Trigger NMT state change

The function triggers an NMT state change by sending the specified node command
to the specified node.

\param[in]      nodeId_p            Node ID to send the node command to.
\param[in]      nodeCommand_p       Node command to send.

\return The function returns a tOplkError error code.

\ingroup module_nmtmnu
*/
//------------------------------------------------------------------------------
tOplkError nmtmnu_triggerStateChange(UINT nodeId_p, tNmtNodeCommand nodeCommand_p)
{
    tOplkError      ret;
    tNmtMnuNodeCmd  nodeCmd;
    tEvent          event;

    if ((nodeId_p == 0) || (nodeId_p >= C_ADR_BROADCAST))
        return kErrorInvalidNodeId;

    nodeCmd.nodeCommand = nodeCommand_p;
    nodeCmd.nodeId = nodeId_p;

    event.eventSink = kEventSinkNmtMnu;
    event.eventType = kEventTypeNmtMnuNodeCmd;
    OPLK_MEMSET(&event.netTime, 0x00, sizeof(event.netTime));
    event.eventArg.pEventArg = &nodeCmd;
    event.eventArgSize = sizeof(nodeCmd);

    ret = eventu_postEvent(&event);

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Callback function for NMT state changes

The function implements the callback function for NMT state changes

\param[in]      nmtStateChange_p    The received NMT state change event.

\return The function returns a tOplkError error code.

\ingroup module_nmtmnu
*/
//------------------------------------------------------------------------------
tOplkError nmtmnu_cbNmtStateChange(tEventNmtStateChange nmtStateChange_p)
{
    tOplkError  ret;
    UINT8       newMnNmtState;

    // Save new MN state in object 0x1F8E
    newMnNmtState = (UINT8)nmtStateChange_p.newNmtState;
    ret = obdu_writeEntry(0x1F8E, 240, &newMnNmtState, 1);
    if (ret != kErrorOk)
        return ret;

    // do work which must be done in that state
    switch (nmtStateChange_p.newNmtState)
    {
        // build the configuration with infos from OD
        case kNmtGsResetConfiguration:
            {
                UINT32      timeout;
                tObdSize    obdSize;

#if defined(CONFIG_INCLUDE_NMT_RMN)
                nmtMnuInstance_g.flags &= ~NMTMNU_FLAG_REDUNDANCY;
#endif

                // read object 0x1F80 NMT_StartUp_U32
                obdSize = 4;
                ret = obdu_readEntry(0x1F80, 0, &nmtMnuInstance_g.nmtStartup, &obdSize);
                if (ret != kErrorOk)
                    break;

                // compute StatusReqDelay = object 0x1006 * C_NMT_STATREQ_CYCLE
                obdSize = sizeof(timeout);
                ret = obdu_readEntry(0x1006, 0, &timeout, &obdSize);
                if (ret != kErrorOk)
                    break;

                if (timeout != 0L)
                {
                    nmtMnuInstance_g.statusRequestDelay =
                            computeCeilDiv(timeout * C_NMT_STATREQ_CYCLE, 1000L);

                    // $$$ fetch and use MultiplexedCycleCount from OD
                    nmtMnuInstance_g.timeoutCheckCom =
                            computeCeilDiv(timeout * C_NMT_STATREQ_CYCLE, 1000L);
                }

                // fetch MNTimeoutPreOp2_U32 from OD
                obdSize = sizeof(timeout);
                ret = obdu_readEntry(0x1F89, 4, &timeout, &obdSize);
                if (ret != kErrorOk)
                    break;

                if (timeout != 0L)
                {
                    // convert [us] to [ms]
                    nmtMnuInstance_g.timeoutReadyToOp = computeCeilDiv(timeout, 1000L);
                }
                else
                    nmtMnuInstance_g.timeoutReadyToOp = 0L;
            }
            break;

        //-----------------------------------------------------------
        // MN part of the state machine

        // node listens for POWERLINK frames and checks timeout
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
#if defined(CONFIG_INCLUDE_NMT_RMN)
            if (nmtStateChange_p.oldNmtState == kNmtCsOperational)
            {
                switchoverRedundancy();
                break;
            }
#endif
            // send StartNode to CNs
            // wait for NMT state change of CNs
            ret = startNodes();
            break;

        // no POWERLINK cycle
        // -> normal Ethernet communication
        case kNmtMsBasicEthernet:
            break;

#if defined(CONFIG_INCLUDE_NMT_RMN)
        case kNmtRmsNotActive:
            nmtMnuInstance_g.flags |= NMTMNU_FLAG_REDUNDANCY;
            break;

        case kNmtCsPreOperational1:
            if (!(nmtMnuInstance_g.flags & NMTMNU_FLAG_REDUNDANCY))
                break;
            resetRedundancy();
            break;
#endif

        default:
            break;
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Callback function for NMT events

The function implements the callback function for NMT events.

\param[in]      pEvent_p            Pointer to the received NMT event.

\return The function returns a tOplkError error code.

\ingroup module_nmtmnu
*/
//------------------------------------------------------------------------------
tOplkError nmtmnu_processEvent(const tEvent* pEvent_p)
{
    tOplkError  ret = kErrorOk;

    // process event
    switch (pEvent_p->eventType)
    {
        // timer event
        case kEventTypeTimer:
            {
                const tTimerEventArg*   pTimerEventArg =
                                            (const tTimerEventArg*)pEvent_p->eventArg.pEventArg;
                UINT                    nodeId;

                nodeId = (UINT)(pTimerEventArg->argument.value & NMTMNU_TIMERARG_NODE_MASK);
                if (nodeId != 0)
                {
                    tObdSize                obdSize;
                    UINT8                   obdNmtState;
                    const tNmtMnuNodeInfo*  pNodeInfo = NMTMNU_GET_NODEINFO(nodeId);

                    obdSize = 1;
                    ret = obdu_readEntry(0x1F8E, nodeId, &obdNmtState, &obdSize);
                    if (ret != kErrorOk)
                        break;

                    if ((pTimerEventArg->argument.value & NMTMNU_TIMERARG_IDENTREQ) != 0L)
                    {
                        if ((UINT32)(pNodeInfo->flags & NMTMNU_NODE_FLAG_COUNT_STATREQ) !=
                            (pTimerEventArg->argument.value & NMTMNU_TIMERARG_COUNT_SR))
                        {   // this is an old (already deleted or modified) timer
                            // but not the current timer
                            // so discard it
                            NMTMNU_DBG_POST_TRACE_VALUE(kNmtMnuIntNodeEventTimerIdentReq,
                                                        nodeId,
                                                        ((pNodeInfo->nodeState << 8) | 0xFF));

                            break;
                        }

                        NMTMNU_DBG_POST_TRACE_VALUE(kNmtMnuIntNodeEventTimerIdentReq,
                                                    nodeId,
                                                    ((pNodeInfo->nodeState << 8) | 0x80 |
                                                     ((pNodeInfo->flags & NMTMNU_NODE_FLAG_COUNT_STATREQ) >> 6) |
                                                     ((pTimerEventArg->argument.value & NMTMNU_TIMERARG_COUNT_SR) >> 8)));

                        ret = processInternalEvent(nodeId,
                                                   (tNmtState)(obdNmtState | NMT_TYPE_CS),
                                                   E_NO_ERROR,
                                                   kNmtMnuIntNodeEventTimerIdentReq);
                    }
                    else if ((pTimerEventArg->argument.value & NMTMNU_TIMERARG_STATREQ) != 0L)
                    {
                        if ((UINT32)(pNodeInfo->flags & NMTMNU_NODE_FLAG_COUNT_STATREQ) !=
                            (pTimerEventArg->argument.value & NMTMNU_TIMERARG_COUNT_SR))
                        {   // this is an old (already deleted or modified) timer
                            // but not the current timer
                            // so discard it
                            NMTMNU_DBG_POST_TRACE_VALUE(kNmtMnuIntNodeEventTimerStatReq,
                                                        nodeId,
                                                        ((pNodeInfo->nodeState << 8) | 0xFF));
                            break;
                        }

                        NMTMNU_DBG_POST_TRACE_VALUE(kNmtMnuIntNodeEventTimerStatReq,
                                                    nodeId,
                                                    ((pNodeInfo->nodeState << 8) | 0x80 |
                                                     ((pNodeInfo->flags & NMTMNU_NODE_FLAG_COUNT_STATREQ) >> 6) |
                                                     ((pTimerEventArg->argument.value & NMTMNU_TIMERARG_COUNT_SR) >> 8)));
                        ret = processInternalEvent(nodeId,
                                                   (tNmtState)(obdNmtState | NMT_TYPE_CS),
                                                   E_NO_ERROR,
                                                   kNmtMnuIntNodeEventTimerStatReq);
                    }
                    else if ((pTimerEventArg->argument.value & NMTMNU_TIMERARG_STATE_MON) != 0L)
                    {
                        if ((UINT32)(pNodeInfo->flags & NMTMNU_NODE_FLAG_COUNT_STATREQ) !=
                            (pTimerEventArg->argument.value & NMTMNU_TIMERARG_COUNT_SR))
                        {   // this is an old (already deleted or modified) timer
                            // but not the current timer
                            // so discard it
                            NMTMNU_DBG_POST_TRACE_VALUE(kNmtMnuIntNodeEventTimerStateMon,
                                                        nodeId,
                                                        ((pNodeInfo->nodeState << 8) | 0xFF));

                            break;
                        }

                        NMTMNU_DBG_POST_TRACE_VALUE(kNmtMnuIntNodeEventTimerStatReq,
                                                    nodeId,
                                                    ((pNodeInfo->nodeState << 8) | 0x80 |
                                                     ((pNodeInfo->flags & NMTMNU_NODE_FLAG_COUNT_STATREQ) >> 6) |
                                                     ((pTimerEventArg->argument.value & NMTMNU_TIMERARG_COUNT_SR) >> 8)));
                        ret = processInternalEvent(nodeId,
                                                   (tNmtState)(obdNmtState | NMT_TYPE_CS),
                                                   E_NO_ERROR,
                                                   kNmtMnuIntNodeEventTimerStateMon);
                    }
                    else if ((pTimerEventArg->argument.value & NMTMNU_TIMERARG_LONGER) != 0L)
                    {
                        if ((UINT32)(pNodeInfo->flags & NMTMNU_NODE_FLAG_COUNT_LONGER) !=
                            (pTimerEventArg->argument.value & NMTMNU_TIMERARG_COUNT_LO))
                        {   // this is an old (already deleted or modified) timer
                            // but not the current timer
                            // so discard it
                            NMTMNU_DBG_POST_TRACE_VALUE(kNmtMnuIntNodeEventTimerLonger,
                                                        nodeId,
                                                        ((pNodeInfo->nodeState << 8) | 0xFF));

                            break;
                        }

                        NMTMNU_DBG_POST_TRACE_VALUE(kNmtMnuIntNodeEventTimerLonger,
                                                    nodeId,
                                                    ((pNodeInfo->nodeState << 8) | 0x80 |
                                                     ((pNodeInfo->flags & NMTMNU_NODE_FLAG_COUNT_LONGER) >> 6) |
                                                     ((pTimerEventArg->argument.value & NMTMNU_TIMERARG_COUNT_LO) >> 8)));
                        ret = processInternalEvent(nodeId,
                                                   (tNmtState)(obdNmtState | NMT_TYPE_CS),
                                                   E_NO_ERROR,
                                                   kNmtMnuIntNodeEventTimerLonger);
                    }
                }
                else
                {   // global timer event
                }
            }
            break;

        case kEventTypeHeartbeat:
            {
                const tHeartbeatEvent*  pHeartbeatEvent =
                                            (const tHeartbeatEvent*)pEvent_p->eventArg.pEventArg;

                ret = processInternalEvent(pHeartbeatEvent->nodeId,
                                           pHeartbeatEvent->nmtState,
                                           pHeartbeatEvent->errorCode,
                                           kNmtMnuIntNodeEventHeartbeat);
            }
            break;

        case kEventTypeNmtMnuNmtCmdSent:
            {
                const tPlkFrame*    pFrame = (const tPlkFrame*)pEvent_p->eventArg.pEventArg;
                UINT                nodeId;
                tNmtCommand         nmtCommand;
                UINT8               nmtState;
                UINT                tempNodeId = C_ADR_INVALID; // Init to non-existent nodeId
                const UINT8*        pCmdData;
                tNmtMnuGetNodeId    nodeListOp;
                tOplkError          retGetNodeId;

                if (pEvent_p->eventArgSize < C_DLL_MINSIZE_NMTCMD)
                {
                    ret = eventu_postError(kEventSourceNmtMnu,
                                           kErrorNmtInvalidFramePointer,
                                           sizeof(pEvent_p->eventArgSize),
                                           &pEvent_p->eventArgSize);
                    break;
                }

                nodeId = ami_getUint8Le(&pFrame->dstNodeId);

                nmtCommand = (tNmtCommand)ami_getUint8Le(&pFrame->data.asnd.payload.nmtCommandService.nmtCommandId);
                switch (nmtCommand)
                {
                    case kNmtCmdStartNode:
                    case kNmtCmdStartNodeEx:
                        nmtState = (UINT8)(kNmtCsOperational & 0xFF);
                        break;

                    case kNmtCmdStopNode:
                    case kNmtCmdStopNodeEx:
                        nmtState = (UINT8)(kNmtCsStopped & 0xFF);
                        break;

                    case kNmtCmdEnterPreOperational2:
                    case kNmtCmdEnterPreOperational2Ex:
                        nmtState = (UINT8)(kNmtCsPreOperational2 & 0xFF);
                        break;

                    case kNmtCmdEnableReadyToOperate:
                    case kNmtCmdEnableReadyToOperateEx:
                        // d.k. do not change expected node state, because of DS 1.0.0 7.3.1.2.1 Plain NMT State Command
                        //      and because node may not change NMT state within C_NMT_STATE_TOLERANCE
                        nmtState = (UINT8)(kNmtCsPreOperational2 & 0xFF);
                        break;

                    case kNmtCmdResetNode:
                    case kNmtCmdResetCommunication:
                    case kNmtCmdResetConfiguration:
                    case kNmtCmdSwReset:
                    case kNmtCmdResetNodeEx:
                    case kNmtCmdResetCommunicationEx:
                    case kNmtCmdResetConfigurationEx:
                    case kNmtCmdSwResetEx:
                        nmtState = (UINT8)(kNmtCsNotActive & 0xFF);
                        // processInternalEvent() sets internal node state to kNmtMnuNodeStateUnknown
                        // after next unresponded IdentRequest/StatusRequest
                        break;

                    default:
                        goto Exit;
                }

                // process as internal event which update expected NMT state in OD
                pCmdData = pFrame->data.asnd.payload.nmtCommandService.aNmtCommandData;
                OPLK_MEMSET(&nodeListOp, 0x00, sizeof(nodeListOp));

                retGetNodeId = getNodeIdFromCmd(nodeId,
                                                nmtCommand,
                                                pCmdData,
                                                &nodeListOp,
                                                &tempNodeId);
                while (retGetNodeId == kErrorRetry)
                {
                    if ((NMTMNU_GET_NODEINFO(tempNodeId)->nodeCfg & (NMT_NODEASSIGN_NODE_IS_CN | NMT_NODEASSIGN_NODE_EXISTS)) ==
                        (NMT_NODEASSIGN_NODE_IS_CN | NMT_NODEASSIGN_NODE_EXISTS))
                    {
                        ret = processInternalEvent(tempNodeId,
                                                   (tNmtState)(nmtState | NMT_TYPE_CS),
                                                   0,
                                                   kNmtMnuIntNodeEventNmtCmdSent);
                        if (ret != kErrorOk)
                            goto Exit;
                    }

                    retGetNodeId = getNodeIdFromCmd(nodeId,
                                                    nmtCommand,
                                                    pCmdData,
                                                    &nodeListOp,
                                                    &tempNodeId);
                }

                // User requested reset commands that were broadcast to all nodes
                // This has to be forwarded to nmtk module
                if ((nodeId == C_ADR_BROADCAST) &&
                    ((nmtMnuInstance_g.flags & NMTMNU_FLAG_USER_RESET) != 0))
                {   // user or diagnostic nodes requests a reset of the MN
                    tNmtEvent   nmtEvent;

                    switch (nmtCommand)
                    {
                        case kNmtCmdResetNode:
                            nmtEvent = kNmtEventResetNode;
                            break;

                        case kNmtCmdResetCommunication:
                            nmtEvent = kNmtEventResetCom;
                            break;

                        case kNmtCmdResetConfiguration:
                            nmtEvent = kNmtEventResetConfig;
                            break;

                        case kNmtCmdSwReset:
                            nmtEvent = kNmtEventSwReset;
                            break;

                        case kNmtCmdInvalidService:
                        default:    // actually no reset was requested
                            goto Exit;
                    }

                    ret = nmtu_postNmtEvent(nmtEvent);
                    if (ret != kErrorOk)
                        goto Exit;
                }
            }
            break;

        case kEventTypeNmtMnuNodeCmd:
            {
                const tNmtMnuNodeCmd*   pNodeCmd =
                                            (const tNmtMnuNodeCmd*)pEvent_p->eventArg.pEventArg;
                tNmtMnuIntNodeEvent     nodeEvent;
                tObdSize                obdSize;
                UINT8                   obdNmtState;
                UINT16                  errorCode = E_NO_ERROR;

                if ((pNodeCmd->nodeId == 0) || (pNodeCmd->nodeId >= C_ADR_BROADCAST))
                {
                    ret = kErrorInvalidNodeId;
                    goto Exit;
                }

                switch (pNodeCmd->nodeCommand)
                {
                    case kNmtNodeCommandBoot:
                        nodeEvent = kNmtMnuIntNodeEventBoot;
                        break;

                    case kNmtNodeCommandSwOk:
                        nodeEvent = kNmtMnuIntNodeEventSwOk;
                        break;

                    case kNmtNodeCommandSwUpdated:
                        nodeEvent = kNmtMnuIntNodeEventSwUpdated;
                        break;

                    case kNmtNodeCommandSwErr:
                        nodeEvent = kNmtMnuIntNodeEventError;
                        errorCode = E_NMT_BPO1_SW_UPDATE;
                        break;

                    case kNmtNodeCommandConfOk:
                        nodeEvent = kNmtMnuIntNodeEventConfigured;
                        break;

                    case kNmtNodeCommandConfErr:
                        nodeEvent = kNmtMnuIntNodeEventError;
                        errorCode = E_NMT_BPO1_CF_VERIFY;
                        break;

                    case kNmtNodeCommandConfRestored:
                        nodeEvent = kNmtMnuIntNodeEventExecResetNode;
                        break;

                    case kNmtNodeCommandConfReset:
                        nodeEvent = kNmtMnuIntNodeEventExecResetConf;
                        break;

                    default:
                       // invalid node command
                        goto Exit;
                }

                // fetch current NMT state
                obdSize = sizeof(obdNmtState);
                ret = obdu_readEntry(0x1F8E, pNodeCmd->nodeId, &obdNmtState, &obdSize);
                if (ret != kErrorOk)
                    goto Exit;

                ret = processInternalEvent(pNodeCmd->nodeId,
                                           (tNmtState)(obdNmtState | NMT_TYPE_CS),
                                           errorCode,
                                           nodeEvent);
            }
            break;

        case kEventTypeNmtMnuNodeAdded:
            {
                const UINT  nodeId = *((const UINT*)pEvent_p->eventArg.pEventArg);

                ret = cbNodeAdded(nodeId);
            }
            break;

        case kEventTypeReceivedAmni:
            {
                const UINT  nodeId = *((const UINT*)pEvent_p->eventArg.pEventArg);

                ret = nmtMnuInstance_g.pfnCbNodeEvent(nodeId,
                                                      kNmtNodeEventAmniReceived,
                                                      kNmtGsOff,
                                                      0,
                                                      FALSE);
            }
            break;

        default:
            ret = kErrorNmtInvalidEvent;
            break;
    }

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Get diagnostic info

The function returns diagnostic information.

\param[out]     pMandatorySlaveCount_p  Pointer to store mandatory slave count.
\param[out]     pSignalSlaveCount_p     Pointer to store signal slave count.
\param[out]     pFlags_p                Pointer to store global flags.

\return The function returns a tOplkError error code.

\ingroup module_nmtmnu
*/
//------------------------------------------------------------------------------
tOplkError nmtmnu_getDiagnosticInfo(UINT* pMandatorySlaveCount_p,
                                    UINT* pSignalSlaveCount_p,
                                    UINT16* pFlags_p)
{
    if ((pMandatorySlaveCount_p == NULL) ||
        (pSignalSlaveCount_p == NULL) ||
        (pFlags_p == NULL))
        return kErrorNmtInvalidParam;

    *pMandatorySlaveCount_p = nmtMnuInstance_g.mandatorySlaveCount;
    *pSignalSlaveCount_p = nmtMnuInstance_g.signalSlaveCount;
    *pFlags_p = nmtMnuInstance_g.flags;

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Configure PRes chaining parameters

The function configures the PRes chaining parameters

\param[in]      pConfigParam_p      PRes chaining parameters.

\return The function returns a tOplkError error code.

\ingroup module_nmtmnu
*/
//------------------------------------------------------------------------------
tOplkError nmtmnu_configPrc(const tNmtMnuConfigParam* pConfigParam_p)
{
    nmtMnuInstance_g.prcPResTimeFirstCorrectionNs = pConfigParam_p->prcPResTimeFirstCorrectionNs;
    nmtMnuInstance_g.prcPResTimeFirstNegOffsetNs = pConfigParam_p->prcPResTimeFirstNegOffsetNs;

    return kErrorOk;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief  Callback function for NMT requests

The function implements the callback function for NMT requests.

\param[in]      pFrameInfo_p        Pointer to NMT request frame information.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError cbNmtRequest(const tFrameInfo* pFrameInfo_p)
{
    tOplkError                  ret;
    UINT                        targetNodeId;
    tNmtCommand                 nmtCommand;
    const tNmtRequestService*   pNmtRequestService;
    UINT                        sourceNodeId;
    UINT                        commandSize;

    if ((pFrameInfo_p == NULL) ||
        (pFrameInfo_p->frame.pBuffer == NULL))
        return kErrorNmtInvalidFramePointer;

    pNmtRequestService = &pFrameInfo_p->frame.pBuffer->data.asnd.payload.nmtRequestService;
    nmtCommand = (tNmtCommand)ami_getUint8Le(&pNmtRequestService->nmtCommandId);
    targetNodeId = ami_getUint8Le(&pNmtRequestService->targetNodeId);
    commandSize = min(sizeof(pNmtRequestService->aNmtCommandData),
                      pFrameInfo_p->frameSize - offsetof(tPlkFrame, data.asnd.payload.nmtRequestService.aNmtCommandData));

    ret = nmtmnu_requestNmtCommand(targetNodeId,
                                   nmtCommand,
                                   pNmtRequestService->aNmtCommandData,
                                   commandSize);
    if (ret != kErrorOk)
    {   // error -> reply with kNmtCmdInvalidService
        sourceNodeId = ami_getUint8Le(&pFrameInfo_p->frame.pBuffer->srcNodeId);

        ret = nmtmnu_sendNmtCommand(sourceNodeId, kNmtCmdInvalidService);
        if (ret == kErrorInvalidOperation)
            ret = kErrorOk;
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Callback function for Ident responses

The function implements the callback function for Ident responses

\param[in]      nodeId_p            Node ID for which IdentResponse was received.
\param[in]      pIdentResponse_p    Pointer to IdentResponse. It is NULL if node did
                                    not answer.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError cbIdentResponse(UINT nodeId_p,
                                  const tIdentResponse* pIdentResponse_p)
{
    tOplkError  ret = kErrorOk;
    tObdSize    obdSize;
    UINT32      devType;
    UINT16      errorCode;
    tNmtState   nmtState;

    if (pIdentResponse_p == NULL)
    {   // node did not answer
        ret = processInternalEvent(nodeId_p,
                                   kNmtCsNotActive,
                                   E_NMT_NO_IDENT_RES,
                                   kNmtMnuIntNodeEventNoIdentResponse);
    }
    else
    {   // node answered IdentRequest
        errorCode = E_NO_ERROR;
        nmtState = correctNmtState(ami_getUint8Le(&pIdentResponse_p->nmtStatus));

        handleMissingPrcSupport(nodeId_p, pIdentResponse_p->featureFlagsLe);

        // check IdentResponse $$$ move to ProcessIntern, because this function may be called also if CN

        // check DeviceType (0x1F84)
        obdSize = 4;
        ret = obdu_readEntry(0x1F84, nodeId_p, &devType, &obdSize);
        if (ret != kErrorOk)
            goto Exit;

        if (devType != 0L)
        {   // actually compare it with DeviceType from IdentResponse
            if (ami_getUint32Le(&pIdentResponse_p->deviceTypeLe) != devType)
            {   // wrong DeviceType
                nmtState = kNmtCsNotActive;
                errorCode = E_NMT_BPO1_DEVICE_TYPE;
            }
        }

        ret = processInternalEvent(nodeId_p,
                                   nmtState,
                                   errorCode,
                                   kNmtMnuIntNodeEventIdentResponse);
    }

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Callback function for Status responses

The function implements the callback function for Status responses

\param[in]      nodeId_p            Node ID for which StatusResponse was received.
\param[in]      pStatusResponse_p   Pointer to StatusResponse. It is NULL if node did
                                    not answer.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError cbStatusResponse(UINT nodeId_p,
                                   const tStatusResponse* pStatusResponse_p)
{
    tOplkError  ret = kErrorOk;
    tNmtState   nmtState;

    if (pStatusResponse_p == NULL)
    {   // node did not answer
        ret = processInternalEvent(nodeId_p,
                                   kNmtCsNotActive,
                                   E_NMT_NO_STATUS_RES,
                                   kNmtMnuIntNodeEventNoStatusResponse);
    }
    else
    {
        nmtState = correctNmtState(ami_getUint8Le(&pStatusResponse_p->nmtStatus));

        ret = processInternalEvent(nodeId_p,
                                   nmtState,
                                   E_NO_ERROR,
                                   kNmtMnuIntNodeEventStatusResponse);
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Callback function for added node events

The function implements the callback function for added node events. It is
called after the addressed node has been added in module dllk.

\param[in]      nodeId_p            Node ID for which the event was received.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError cbNodeAdded(UINT nodeId_p)
{
    tOplkError          ret = kErrorOk;
    tNmtMnuNodeInfo*    pNodeInfo;
    tNmtState           nmtState;

    pNodeInfo = NMTMNU_GET_NODEINFO(nodeId_p);
    pNodeInfo->flags |= NMTMNU_NODE_FLAG_ISOCHRON;

    if (pNodeInfo->nodeState == kNmtMnuNodeStateConfigured)
    {
        nmtState = nmtu_getNmtState();
        if (nmtState >= kNmtMsPreOperational2)
            ret = nodeBootStep2(nodeId_p, pNodeInfo);
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Add node into isochronous phase

The function adds the specified node into the isochronous phase

\param[in]      nodeId_p            Node ID which will be added.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError addNodeIsochronous(UINT nodeId_p)
{
    tOplkError          ret = kErrorOk;
    tNmtMnuNodeInfo*    pNodeInfo;

    if (nodeId_p != C_ADR_INVALID)
    {
        pNodeInfo = NMTMNU_GET_NODEINFO(nodeId_p);
        if (pNodeInfo == NULL)
        {
            ret = kErrorInvalidNodeId;
            goto Exit;
        }

        // clear PRC specific values
        pNodeInfo->prcFlags = 0;
        pNodeInfo->pResTimeFirstNs = 0;
        pNodeInfo->relPropagationDelayNs = 0;

        if ((pNodeInfo->nodeCfg & NMT_NODEASSIGN_PRES_CHAINING) == 0)
        {   // node is added as PReq/PRes node
            tDllNodeOpParam nodeOpParam;

            nodeOpParam.opNodeType = kDllNodeOpTypeIsochronous;
            nodeOpParam.nodeId = nodeId_p;

            ret = dllucal_addNode(&nodeOpParam);
            goto Exit;
        }
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
        UINT    nodeId;
        BOOL    fInvalidateNext;

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
            // $$$ else if: error, if there is still some ADD_IN_PROGRESS
        }

        nmtMnuInstance_g.flags &= ~NMTMNU_FLAG_PRC_ADD_SCHEDULED;

        if (nmtMnuInstance_g.flags & NMTMNU_FLAG_PRC_ADD_IN_PROGRESS)
            ret = prcMeasure();
    }

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Start BootStep1

The function starts the BootStep1.

\param[in]      fNmtResetAllIssued_p    Determines if all nodes should be reset.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError startBootStep1(BOOL fNmtResetAllIssued_p)
{
    tOplkError          ret;
    UINT                subIndex;
    UINT                localNodeId;
    UINT32              nodeCfg;
    tObdSize            obdSize;
    tNmtMnuNodeInfo*    pNodeInfo;
    UINT8               count;

    // $$$ d.k.: save current time for 0x1F89/2 MNTimeoutPreOp1_U32

    // read number of nodes from object 0x1F81/0
    obdSize = sizeof(count);
    ret = obdu_readEntry(0x1F81, 0, &count, &obdSize);
    if (ret != kErrorOk)
        return ret;

    if (count > tabentries(nmtMnuInstance_g.aNodeInfo))
        count = tabentries(nmtMnuInstance_g.aNodeInfo);

    // start network scan
    nmtMnuInstance_g.mandatorySlaveCount = 0;
    nmtMnuInstance_g.signalSlaveCount = 0;
    // check 0x1F81
    localNodeId = obdu_getNodeId();

    pNodeInfo = nmtMnuInstance_g.aNodeInfo;
    for (subIndex = 1; subIndex <= count; subIndex++, pNodeInfo++)
    {
        obdSize = 4;
        ret = obdu_readEntry(0x1F81, subIndex, &nodeCfg, &obdSize);
        if (ret != kErrorOk)
            goto Exit;

        if (subIndex != localNodeId)
        {
            // reset flags "not scanned" and "isochronous"
            pNodeInfo->flags &= ~(NMTMNU_NODE_FLAG_ISOCHRON | NMTMNU_NODE_FLAG_NOT_SCANNED);

            // Reset all PRC flags and PRC related values
            pNodeInfo->prcFlags = 0;
            pNodeInfo->pResTimeFirstNs = 0;
            pNodeInfo->relPropagationDelayNs = 0;

            if (subIndex == C_ADR_DIAG_DEF_NODE_ID)
            {   // diagnostic node must be scanned by MN in any case
                nodeCfg |= (NMT_NODEASSIGN_NODE_IS_CN | NMT_NODEASSIGN_NODE_EXISTS);
                // and it must be isochronously accessed
                nodeCfg &= ~NMT_NODEASSIGN_ASYNCONLY_NODE;
            }

            // save node config in local node info structure
            pNodeInfo->nodeCfg = nodeCfg;
            pNodeInfo->nodeState = kNmtMnuNodeStateUnknown;

            if ((nodeCfg & (NMT_NODEASSIGN_NODE_IS_CN | NMT_NODEASSIGN_NODE_EXISTS)) ==
                (NMT_NODEASSIGN_NODE_IS_CN | NMT_NODEASSIGN_NODE_EXISTS))
            {   // node is configured as CN
                if (fNmtResetAllIssued_p == FALSE)
                {
                    // identify the node
                    ret = identu_requestIdentResponse(subIndex, cbIdentResponse);
                    if (ret != kErrorOk)
                        goto Exit;
                }

                // set flag "not scanned"
                pNodeInfo->flags |= NMTMNU_NODE_FLAG_NOT_SCANNED;
                nmtMnuInstance_g.signalSlaveCount++;
                // signal slave counter shall be decremented if IdentRequest was sent once to a CN

                if ((nodeCfg & NMT_NODEASSIGN_MANDATORY_CN) != 0)
                {   // node is a mandatory CN
                    nmtMnuInstance_g.mandatorySlaveCount++;
                    // mandatory slave counter shall be decremented if mandatory CN was configured successfully
                }
            }
        }
        else
        {   // subindex of MN
            if ((nodeCfg & (NMT_NODEASSIGN_MN_PRES | NMT_NODEASSIGN_NODE_EXISTS)) ==
                (NMT_NODEASSIGN_MN_PRES | NMT_NODEASSIGN_NODE_EXISTS))
            {   // MN shall send PRes
                ret = addNodeIsochronous(localNodeId);
            }
        }
    }

    for (; subIndex <= tabentries(nmtMnuInstance_g.aNodeInfo); subIndex++, pNodeInfo++)
    {   // clear node structure of unused entries
        OPLK_MEMSET(pNodeInfo, 0, sizeof(*pNodeInfo));
    }

Exit:
    return ret;
}

#if defined(CONFIG_INCLUDE_NMT_RMN)
//------------------------------------------------------------------------------
/**
\brief  Prepares BootStep1 if running as Standby Managing Node

The function prepares the BootStep1, so the switch-over to Active Managing Node
can take place seamlessly.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError resetRedundancy(void)
{
    tOplkError          ret;
    UINT                subIndex;
    UINT                localNodeId;
    UINT32              nodeCfg;
    tObdSize            obdSize;
    tNmtMnuNodeInfo*    pNodeInfo;
    UINT8               count;

    identu_reset();
    statusu_reset();
    syncu_reset();
    reset(); // reset timers

    // read number of nodes from object 0x1F81/0
    obdSize = sizeof(count);
    ret = obdu_readEntry(0x1F81, 0, &count, &obdSize);
    if (ret != kErrorOk)
        return ret;

    if (count > tabentries(nmtMnuInstance_g.aNodeInfo))
        count = tabentries(nmtMnuInstance_g.aNodeInfo);

    nmtMnuInstance_g.mandatorySlaveCount = 0;
    nmtMnuInstance_g.signalSlaveCount = 0;
    // check 0x1F81
    localNodeId = obdu_getNodeId();

    pNodeInfo = nmtMnuInstance_g.aNodeInfo;
    for (subIndex = 1; subIndex <= count; subIndex++, pNodeInfo++)
    {
        obdSize = 4;
        ret = obdu_readEntry(0x1F81, subIndex, &nodeCfg, &obdSize);
        if (ret != kErrorOk)
            goto Exit;

        if (subIndex != localNodeId)
        {
            // reset flags "not scanned" and "isochronous"
            pNodeInfo->flags &= ~(NMTMNU_NODE_FLAG_ISOCHRON | NMTMNU_NODE_FLAG_NOT_SCANNED);

            // Reset all PRC flags and PRC related values
            pNodeInfo->prcFlags = 0;
            pNodeInfo->pResTimeFirstNs = 0;
            pNodeInfo->relPropagationDelayNs = 0;

            if (subIndex == C_ADR_DIAG_DEF_NODE_ID)
            {   // diagnostic node must be scanned by MN in any case
                nodeCfg |= (NMT_NODEASSIGN_NODE_IS_CN | NMT_NODEASSIGN_NODE_EXISTS);
                // and it must be isochronously accessed
                nodeCfg &= ~NMT_NODEASSIGN_ASYNCONLY_NODE;
            }

            // save node config in local node info structure
            pNodeInfo->nodeCfg = nodeCfg;
            pNodeInfo->nodeState = kNmtMnuNodeStateUnknown;
        }
        else
        {   // subindex of MN
            if ((~nodeCfg & (NMT_NODEASSIGN_MN_PRES | NMT_NODEASSIGN_NODE_EXISTS)) == 0)
            {   // MN shall send PRes
                ret = addNodeIsochronous(localNodeId);
            }
        }
    }

    for (; subIndex <= tabentries(nmtMnuInstance_g.aNodeInfo); subIndex++, pNodeInfo++)
    {   // clear node structure of unused entries
        OPLK_MEMSET(pNodeInfo, 0, sizeof(*pNodeInfo));
    }

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Performs the switch-over from SMN to AMN

The function performs the switch-over from SMN to AMN, when in OPERATIONAL.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError switchoverRedundancy(void)
{
    tOplkError          ret = kErrorOk;
    UINT                subIndex;
    tNmtMnuNodeInfo*    pNodeInfo;
    UINT                localNodeId;
    UINT8               destinationNmtState;

    pNodeInfo = nmtMnuInstance_g.aNodeInfo;
    localNodeId = obdu_getNodeId();
    for (subIndex = 1;
         subIndex <= tabentries(nmtMnuInstance_g.aNodeInfo);
         subIndex++, pNodeInfo++)
    {
        // identify the node
        if (((~pNodeInfo->nodeCfg & (NMT_NODEASSIGN_NODE_IS_CN | NMT_NODEASSIGN_NODE_EXISTS)) == 0) &&
            (localNodeId != subIndex))
        {   // node is configured as CN
            if (pNodeInfo->nodeState != kNmtMnuNodeStateOperational)
            {
                destinationNmtState = (UINT8)kNmtCsNotActive;
                ret = nmtmnu_sendNmtCommand(subIndex, kNmtCmdResetNode);
                if (ret != kErrorOk)
                    goto Exit;
            }
            else
                destinationNmtState = (UINT8)kNmtCsOperational;

            // write object 0x1F8F NMT_MNNodeExpState_AU8
            ret = obdu_writeEntry(0x1F8F, subIndex, &destinationNmtState, 1);
            if (ret != kErrorOk)
                goto Exit;
        }
    }

Exit:
    return ret;
}
#endif

//------------------------------------------------------------------------------
/**
\brief  Handle MN PreOperational1 State

The function handles the PreOperational1 state of the MN.

\param[in]      nmtStateChange_p    The received NMT state change event.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError doPreop1(tEventNmtStateChange nmtStateChange_p)
{
    UINT32      timeout;
    tTimerArg   timerArg;
    tObdSize    obdSize;
    tEvent      event;
    BOOL        fNmtResetAllIssued = FALSE;
    tOplkError  ret;

    // reset IdentResponses and running IdentRequests and StatusRequests
    identu_reset();
    statusu_reset();
    syncu_reset();
    reset(); // reset timers

    // 2008/11/18 d.k. reset internal node info is not necessary,
    //                 because timer flags are important and other
    //                 things are reset by startBootStep1().

    // inform DLL about NMT state change,
    // so that it can clear the asynchronous queues and start the reduced cycle
    event.eventSink = kEventSinkDllk;
    event.eventType = kEventTypeDllkStartReducedCycle;
    OPLK_MEMSET(&event.netTime, 0x00, sizeof(event.netTime));
    event.eventArg.pEventArg = NULL;
    event.eventArgSize = 0;

    ret = eventu_postEvent(&event);
    if (ret != kErrorOk)
        return ret;

    // reset all nodes
    // skip this step if we come directly from OPERATIONAL
    // or it was just done before, e.g. because of a ResetNode command
    // from a diagnostic node
    if ((nmtStateChange_p.nmtEvent == kNmtEventTimerMsPreOp1) ||
        ((nmtMnuInstance_g.flags & NMTMNU_FLAG_USER_RESET) == 0))
    {
        BENCHMARK_MOD_07_TOGGLE(7);
        NMTMNU_DBG_POST_TRACE_VALUE(0, C_ADR_BROADCAST, kNmtCmdResetNode);

        ret = nmtmnu_sendNmtCommand(C_ADR_BROADCAST, kNmtCmdResetNode);
        if (ret != kErrorOk)
            return ret;

        fNmtResetAllIssued = TRUE;
    }

    // clear global flags, e.g. re-enable boot process
    nmtMnuInstance_g.flags = 0;

    // start network scan
    ret = startBootStep1(fNmtResetAllIssued);
    if (ret != kErrorOk)
    {
        DEBUG_LVL_ERROR_TRACE("%s startBootStep1 failed with 0x%X\n", __func__, ret);
    }

    // start timer for 0x1F89/2 MNTimeoutPreOp1_U32
    obdSize = sizeof(timeout);
    ret = obdu_readEntry(0x1F89, 2, &timeout, &obdSize);
    if (ret != kErrorOk)
        return ret;

    if (timeout != 0L)
    {
        timeout /= 1000L;
        if (timeout == 0L)
            timeout = 1L; // at least 1 ms

        timerArg.eventSink = kEventSinkNmtMnu;
        timerArg.argument.value = 0;
        ret = timeru_modifyTimer(&nmtMnuInstance_g.timerHdlNmtState, timeout, &timerArg);
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Start BootStep2

The function starts BootStep2 which means checking whether a node has reached
PreOp2 and has been added to the isochronous phase. If this is so, the
NMT command EnableReadyToOp is sent.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError startBootStep2(void)
{
    tOplkError          ret = kErrorOk;
    UINT                index;
    tNmtMnuNodeInfo*    pNodeInfo;
    tObdSize            obdSize;
    UINT8               obdNmtState;
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
        ret = obdu_readEntry(0x1F8F, index, &obdNmtState, &obdSize);
        if (ret != kErrorOk)
            goto Exit;

        // compute expected NMT state
        expNmtState = (tNmtState)(obdNmtState | NMT_TYPE_CS);
        if (expNmtState == kNmtCsPreOperational1)
        {
            tTimerArg   timerArg;

            // The change to PreOp2 is an implicit NMT command.
            // Unexpected NMT states of the nodes are ignored until
            // the state monitor timer is elapsed.
            NMTMNU_SET_FLAGS_TIMERARG_STATE_MON(pNodeInfo, index, timerArg);

            // set NMT state change flag
            pNodeInfo->flags |= NMTMNU_NODE_FLAG_NMT_CMD_ISSUED;

            ret = timeru_modifyTimer(&pNodeInfo->timerHdlStatReq,
                                     nmtMnuInstance_g.statusRequestDelay,
                                     &timerArg);
            if (ret != kErrorOk)
                goto Exit;

            // update object 0x1F8F NMT_MNNodeExpState_AU8 to PreOp2
            obdNmtState = (UINT8)(kNmtCsPreOperational2 & 0xFF);
            ret = obdu_writeEntry(0x1F8F, index, &obdNmtState, 1);
            if (ret != kErrorOk)
                goto Exit;

            if ((pNodeInfo->nodeState == kNmtMnuNodeStateConfigured) &&
                ((nmtMnuInstance_g.flags & NMTMNU_FLAG_HALTED) == 0))
            {   // boot process is not halted
                // set flag "not scanned"
                pNodeInfo->flags |= NMTMNU_NODE_FLAG_NOT_SCANNED;

                nmtMnuInstance_g.signalSlaveCount++;
                // signal slave counter shall be decremented if StatusRequest was sent once to a CN

                if ((pNodeInfo->nodeCfg & NMT_NODEASSIGN_MANDATORY_CN) != 0)
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

\param[in]      nodeId_p            Node ID for which to start BootStep2.
\param[in,out]  pNodeInfo_p         Pointer to node info structure of node.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError nodeBootStep2(UINT nodeId_p, tNmtMnuNodeInfo* pNodeInfo_p)
{
    tOplkError  ret = kErrorOk;
    tTimerArg   timerArg;
    UINT8       obdNmtState;
    tNmtState   nmtState;
    tObdSize    obdSize;

    if (pNodeInfo_p->nodeCfg & NMT_NODEASSIGN_ASYNCONLY_NODE)
    {   // node is async-only
        // read object 0x1F8E NMT_MNNodeCurrState_AU8
        obdSize = 1;
        ret = obdu_readEntry(0x1F8E, nodeId_p, &obdNmtState, &obdSize);
        if (ret != kErrorOk)
            goto Exit;

        nmtState = (tNmtState)(obdNmtState | NMT_TYPE_CS);

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
    if (ret != kErrorOk)
        goto Exit;

    if (nmtMnuInstance_g.timeoutReadyToOp != 0L)
    {   // start timer
        // when the timer expires the CN must be ReadyToOp
        NMTMNU_SET_FLAGS_TIMERARG_LONGER(pNodeInfo_p, nodeId_p, timerArg);
        ret = timeru_modifyTimer(&pNodeInfo_p->timerHdlLonger,
                                 nmtMnuInstance_g.timeoutReadyToOp,
                                 &timerArg);
    }

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Start CheckCommunication

The function starts CheckCommunication.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError startCheckCom(void)
{
    tOplkError          ret = kErrorOk;
    UINT                index;
    tNmtMnuNodeInfo*    pNodeInfo;

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
                if (ret == kErrorReject)
                {   // timer was started
                    // wait until it expires
                    if ((pNodeInfo->nodeCfg & NMT_NODEASSIGN_MANDATORY_CN) != 0)
                    {   // node is a mandatory CN
                        nmtMnuInstance_g.mandatorySlaveCount++;
                    }
                }
                else
                {
                    if (ret != kErrorOk)
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
    ret = kErrorOk;

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Start CheckCommunication for the specified node

The function starts CheckCommunication for the specified node. That means it
waits some time and if no error occurred everything is OK.

\param[in]      nodeId_p            Node ID for which to start CheckCommunication.
\param[in,out]  pNodeInfo_p         Pointer to node info structure of node.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError nodeCheckCom(UINT nodeId_p, tNmtMnuNodeInfo* pNodeInfo_p)
{
    tOplkError  ret = kErrorOk;
    UINT32      nodeCfg;
    tTimerArg   timerArg;

    nodeCfg = pNodeInfo_p->nodeCfg;
    if (((nodeCfg & NMT_NODEASSIGN_ASYNCONLY_NODE) == 0) &&
        (nmtMnuInstance_g.timeoutCheckCom != 0L))
    {   // CN is not async-only and timeout for CheckCom was set

        // check communication,
        // that means wait some time and if no error occurred everything is OK;

        // start timer (when the timer expires the CN must be still ReadyToOp)
        NMTMNU_SET_FLAGS_TIMERARG_LONGER(pNodeInfo_p, nodeId_p, timerArg);
        ret = timeru_modifyTimer(&pNodeInfo_p->timerHdlLonger,
                                 nmtMnuInstance_g.timeoutCheckCom,
                                 &timerArg);

        // update mandatory slave counter, because timer was started
        if (ret == kErrorOk)
            ret = kErrorReject;
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

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError startNodes(void)
{
    tOplkError          ret = kErrorOk;
    UINT                index;
    tNmtMnuNodeInfo*    pNodeInfo;

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
                if ((nmtMnuInstance_g.nmtStartup & NMT_STARTUP_STARTALLNODES) == 0)
                {
                    NMTMNU_DBG_POST_TRACE_VALUE(0, index, kNmtCmdStartNode);
                    ret = nmtmnu_sendNmtCommand(index, kNmtCmdStartNode);
                    if (ret != kErrorOk)
                        goto Exit;
                }

                if ((pNodeInfo->nodeCfg & NMT_NODEASSIGN_MANDATORY_CN) != 0)
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

        // $$$ inform application if NMT_STARTUP_NO_STARTNODE is set

        if ((nmtMnuInstance_g.nmtStartup & NMT_STARTUP_STARTALLNODES) != 0)
        {
            NMTMNU_DBG_POST_TRACE_VALUE(0, C_ADR_BROADCAST, kNmtCmdStartNode);
            ret = nmtmnu_sendNmtCommand(C_ADR_BROADCAST, kNmtCmdStartNode);
            if (ret != kErrorOk)
                goto Exit;
        }
    }

Exit:
    return ret;
}

#if defined(CONFIG_INCLUDE_NMT_RMN)
//------------------------------------------------------------------------------
/**
\brief  Process heartbeat event on a redundant MN

The function processes a heartbeat event on a redundant MN.

\param[in]      nodeId_p            Node ID to process.
\param[in]      nodeNmtState_p      NMT state of the node.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError processRedundancyHeartbeat(UINT nodeId_p, tNmtState nodeNmtState_p)
{
    tNmtMnuNodeInfo*    pNodeInfo;
    UINT8               obdNodeNmtState;
    tOplkError          ret;
    tDllNodeOpParam     nodeOpParam;

    pNodeInfo = NMTMNU_GET_NODEINFO(nodeId_p);

    obdNodeNmtState = (UINT8)nodeNmtState_p;
    // update object 0x1F8E NMT_MNNodeCurrState_AU8
    ret = obdu_writeEntry(0x1F8E, nodeId_p, &obdNodeNmtState, 1);
    if (ret != kErrorOk)
        return ret;

    ret = nmtMnuInstance_g.pfnCbNodeEvent(nodeId_p,
                                          kNmtNodeEventNmtState,
                                          nodeNmtState_p,
                                          0,
                                          (pNodeInfo->nodeCfg & NMT_NODEASSIGN_MANDATORY_CN) != 0);
    if (ret != kErrorOk)
        return ret;

    if (nodeNmtState_p == kNmtCsOperational)
    {
        addNodeIsochronous(nodeId_p);

        pNodeInfo->nodeState = kNmtMnuNodeStateOperational;
    }
    else
    {
        nodeOpParam.nodeId = nodeId_p;
        nodeOpParam.opNodeType = kDllNodeOpTypeIsochronous;
        ret = dllucal_deleteNode(&nodeOpParam);

        pNodeInfo->nodeState = kNmtMnuNodeStateUnknown;
    }

    return ret;
}
#endif

//------------------------------------------------------------------------------
/**
\brief  Process IdentResponse node event

The function processes the internal node event kNmtMnuIntNodeEventIdentResponse.

\param[in]      nodeId_p            Node ID to process.
\param[in]      nodeNmtState_p      NMT state of the node.
\param[in]      nmtState_p          NMT state of the MN
\param[in]      errorCode_p         Error codes.
\param[out]     pRet_p              Pointer to store return value

\return The function returns 0 if the higher level event handler should continue
        processing or -1 if it should exit.
*/
//------------------------------------------------------------------------------
static INT processNodeEventIdentResponse(UINT nodeId_p,
                                         tNmtState nodeNmtState_p,
                                         tNmtState nmtState_p,
                                         UINT16 errorCode_p,
                                         tOplkError* pRet_p)
{
    UINT8               obdNmtState;
    tNmtMnuNodeInfo*    pNodeInfo;

    pNodeInfo = NMTMNU_GET_NODEINFO(nodeId_p);

    NMTMNU_DBG_POST_TRACE_VALUE(kNmtMnuIntNodeEventIdentResponse,
                                nodeId_p,
                                pNodeInfo->nodeState);

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
        obdNmtState = (UINT8)(kNmtCsPreOperational1 & 0xFF);
    }
    else
    {   // MN is running full cycle
        // update object 0x1F8F NMT_MNNodeExpState_AU8 to PreOp2
        obdNmtState = (UINT8)(kNmtCsPreOperational2 & 0xFF);
        if (nodeNmtState_p == kNmtCsPreOperational1)
        {   // The CN did not yet switch to PreOp2
            tTimerArg   timerArg;

            // Set NMT state change flag and ignore unexpected NMT states
            // until the state monitor timer is elapsed.
            pNodeInfo->flags |= NMTMNU_NODE_FLAG_NMT_CMD_ISSUED;

            NMTMNU_SET_FLAGS_TIMERARG_STATE_MON(pNodeInfo, nodeId_p, timerArg);

            *pRet_p = timeru_modifyTimer(&pNodeInfo->timerHdlStatReq,
                                         nmtMnuInstance_g.statusRequestDelay,
                                         &timerArg);
            if (*pRet_p != kErrorOk)
                return -1;
        }
    }
    *pRet_p = obdu_writeEntry(0x1F8F, nodeId_p, &obdNmtState, 1);
    if (*pRet_p != kErrorOk)
        return -1;

    // check NMT state of CN
    *pRet_p = checkNmtState(nodeId_p,
                            pNodeInfo,
                            nodeNmtState_p,
                            errorCode_p,
                            nmtState_p);
    if (*pRet_p != kErrorOk)
    {
        if (*pRet_p == kErrorReject)
            *pRet_p = kErrorOk;

        return 0;
    }

    if ((pNodeInfo->flags & NMTMNU_NODE_FLAG_NMT_CMD_ISSUED) == 0)
    {   // No state monitor timer is required
        // Request StatusResponse immediately,
        // because we want a fast boot-up of CNs
        *pRet_p = statusu_requestStatusResponse(nodeId_p, cbStatusResponse);
        if (*pRet_p != kErrorOk)
        {
            NMTMNU_DBG_POST_TRACE_VALUE(kNmtMnuIntNodeEventIdentResponse,
                                        nodeId_p,
                                        *pRet_p);
            if (*pRet_p == kErrorInvalidOperation)
            {   // the only situation when this should happen is, when
                // StatusResponse was already requested from within
                // the StatReq timer event.
                // so ignore this error.
                *pRet_p = kErrorOk;
            }
            else
                return 0;
        }
    }

    if ((pNodeInfo->nodeState != kNmtMnuNodeStateResetConf) &&
        (pNodeInfo->nodeState != kNmtMnuNodeStateConfRestored))
    {
        // inform application
        *pRet_p = nmtMnuInstance_g.pfnCbNodeEvent(nodeId_p,
                                                  kNmtNodeEventFound,
                                                  nodeNmtState_p,
                                                  E_NO_ERROR,
                                                  (pNodeInfo->nodeCfg & NMT_NODEASSIGN_MANDATORY_CN) != 0);
        if (*pRet_p == kErrorReject)
        {   // interrupt boot process on user request
            NMTMNU_DBG_POST_TRACE_VALUE(kNmtMnuIntNodeEventIdentResponse,
                                        nodeId_p,
                                        ((pNodeInfo->nodeState << 8) | *pRet_p));

            *pRet_p = kErrorOk;
            return 0;
        }
        else if (*pRet_p != kErrorOk)
        {
            NMTMNU_DBG_POST_TRACE_VALUE(kNmtMnuIntNodeEventIdentResponse,
                                        nodeId_p,
                                        ((pNodeInfo->nodeState << 8) | *pRet_p));
            return 0;
        }
    }

    // continue BootStep1
    return processNodeEventBoot(nodeId_p,
                                nodeNmtState_p,
                                nmtState_p,
                                errorCode_p,
                                pRet_p);
}

//------------------------------------------------------------------------------
/**
\brief  Process boot node event

The function processes the internal node event kNmtMnuIntNodeEventBoot.

\param[in]      nodeId_p            Node ID to process.
\param[in]      nodeNmtState_p      NMT state of the node.
\param[in]      nmtState_p          NMT state of the MN
\param[in]      errorCode_p         Error codes.
\param[out]     pRet_p              Pointer to store return value

\return The function returns 0 if the higher level event handler should continue
        processing or -1 if it should exit.
*/
//------------------------------------------------------------------------------
static INT processNodeEventBoot(UINT nodeId_p,
                                tNmtState nodeNmtState_p,
                                tNmtState nmtState_p,
                                UINT16 errorCode_p,
                                tOplkError* pRet_p)
{
    tNmtMnuNodeInfo*  pNodeInfo;

    UNUSED_PARAMETER(errorCode_p);
    UNUSED_PARAMETER(nmtState_p);

    pNodeInfo = NMTMNU_GET_NODEINFO(nodeId_p);

    if (pNodeInfo->nodeState == kNmtMnuNodeStateIdentified)
    {
        if ((nmtMnuInstance_g.nmtStartup & NMT_STARTUP_SWVERSIONCHECK) != 0)
        {
            *pRet_p = nmtMnuInstance_g.pfnCbNodeEvent(nodeId_p,
                                                      kNmtNodeEventUpdateSw,
                                                      nodeNmtState_p,
                                                      E_NO_ERROR,
                                                      (pNodeInfo->nodeCfg & NMT_NODEASSIGN_MANDATORY_CN) != 0);
            if (*pRet_p == kErrorOk)
            {
                pNodeInfo->nodeState = kNmtMnuNodeStateSwOk;
            }
        }
        else
        {
            pNodeInfo->nodeState = kNmtMnuNodeStateSwOk;
        }

        if (pNodeInfo->nodeState == kNmtMnuNodeStateSwOk)
        {
            // check/start configuration
            // inform application
            *pRet_p = nmtMnuInstance_g.pfnCbNodeEvent(nodeId_p,
                                                      kNmtNodeEventCheckConf,
                                                      nodeNmtState_p,
                                                      E_NO_ERROR,
                                                      (pNodeInfo->nodeCfg & NMT_NODEASSIGN_MANDATORY_CN) != 0);
        }
    }
    else if (pNodeInfo->nodeState == kNmtMnuNodeStateSwOk)
    {
        // check/start configuration
        // inform application
        *pRet_p = nmtMnuInstance_g.pfnCbNodeEvent(nodeId_p,
                                                  kNmtNodeEventCheckConf,
                                                  nodeNmtState_p,
                                                  E_NO_ERROR,
                                                  (pNodeInfo->nodeCfg & NMT_NODEASSIGN_MANDATORY_CN) != 0);
    }
    else if (pNodeInfo->nodeState == kNmtMnuNodeStateConfRestored)
    {
        // check/start configuration
        // inform application
        *pRet_p = nmtMnuInstance_g.pfnCbNodeEvent(nodeId_p,
                                                  kNmtNodeEventUpdateConf,
                                                  nodeNmtState_p,
                                                  E_NO_ERROR,
                                                  (pNodeInfo->nodeCfg & NMT_NODEASSIGN_MANDATORY_CN) != 0);
    }
    else if (pNodeInfo->nodeState == kNmtMnuNodeStateResetConf)
    {
        *pRet_p = nmtMnuInstance_g.pfnCbNodeEvent(nodeId_p,
                                                  kNmtNodeEventConfDone,
                                                  nodeNmtState_p,
                                                  E_NO_ERROR,
                                                  (pNodeInfo->nodeCfg & NMT_NODEASSIGN_MANDATORY_CN) != 0);
    }
    else
    {   // wrong CN state
        // ignore event
        return 0;
    }

    if (*pRet_p == kErrorReject)
    {   // interrupt boot process on user request
        NMTMNU_DBG_POST_TRACE_VALUE(kNmtMnuIntNodeEventBoot,
                                    nodeId_p,
                                    ((pNodeInfo->nodeState << 8) | *pRet_p));
        *pRet_p = kErrorOk;
        return 0;
    }
    else if (*pRet_p != kErrorOk)
    {
        NMTMNU_DBG_POST_TRACE_VALUE(kNmtMnuIntNodeEventBoot,
                                    nodeId_p,
                                    ((pNodeInfo->nodeState << 8) | *pRet_p));
        return 0;
    }

    // we assume configuration is OK
    // continue BootStep1
    processNodeEventConfigured(nodeId_p,
                               nodeNmtState_p,
                               nmtState_p,
                               errorCode_p,
                               pRet_p);

    return 0;
}

//------------------------------------------------------------------------------
/**
\brief  Process Configured node event

The function processes the internal node event kNmtMnuIntNodeEventConfigured.

\param[in]      nodeId_p            Node ID to process.
\param[in]      nodeNmtState_p      NMT state of the node.
\param[in]      nmtState_p          NMT state of the MN
\param[in]      errorCode_p         Error codes.
\param[out]     pRet_p              Pointer to store return value

\return The function returns 0 if the higher level event handler should continue
        processing or -1 if it should exit.
*/
//------------------------------------------------------------------------------
static INT processNodeEventConfigured(UINT nodeId_p,
                                      tNmtState nodeNmtState_p,
                                      tNmtState nmtState_p,
                                      UINT16 errorCode_p,
                                      tOplkError* pRet_p)
{
    tNmtMnuNodeInfo*    pNodeInfo;

    UNUSED_PARAMETER(errorCode_p);
    UNUSED_PARAMETER(nodeNmtState_p);

    pNodeInfo = NMTMNU_GET_NODEINFO(nodeId_p);

    if ((pNodeInfo->nodeState != kNmtMnuNodeStateSwOk) &&
        (pNodeInfo->nodeState != kNmtMnuNodeStateConfRestored) &&
        (pNodeInfo->nodeState != kNmtMnuNodeStateResetConf))
    {   // wrong CN state, ignore event
        return 0;
    }

    pNodeInfo->nodeState = kNmtMnuNodeStateConfigured;
    if (nmtState_p == kNmtMsPreOperational1)
    {
        if ((pNodeInfo->nodeCfg & NMT_NODEASSIGN_MANDATORY_CN) != 0)
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

\param[in]      nodeId_p            Node ID to process.
\param[in]      nodeNmtState_p      NMT state of the node.
\param[in]      nmtState_p          NMT state of the MN
\param[in]      errorCode_p         Error codes.
\param[out]     pRet_p              Pointer to store return value

\return The function returns 0 if the higher level event handler should continue
        processing or -1 if it should exit.
*/
//------------------------------------------------------------------------------
INT processNodeEventNoIdentResponse(UINT nodeId_p,
                                    tNmtState nodeNmtState_p,
                                    tNmtState nmtState_p,
                                    UINT16 errorCode_p,
                                    tOplkError* pRet_p)
{
    tTimerArg           timerArg;
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
    *pRet_p = checkNmtState(nodeId_p,
                            pNodeInfo,
                            nodeNmtState_p,
                            errorCode_p,
                            nmtState_p);
    if (*pRet_p == kErrorReject)
        *pRet_p = kErrorOk;
    else if (*pRet_p != kErrorOk)
        return 0;

    // $$$ d.k. check start time for 0x1F89/2 MNTimeoutPreOp1_U32
    // $$$ d.k. check individual timeout 0x1F89/6 MNIdentificationTimeout_U32
    // if mandatory node and timeout elapsed -> halt boot procedure
    // trigger IdentRequest again (if >= PreOp2, after delay)
    if (nmtState_p >= kNmtMsPreOperational2)
    {   // start timer
        NMTMNU_SET_FLAGS_TIMERARG_IDENTREQ(pNodeInfo, nodeId_p, timerArg);
        NMTMNU_DBG_POST_TRACE_VALUE(kNmtMnuIntNodeEventNoIdentResponse,
                                    nodeId_p,
                                    ((pNodeInfo->nodeState << 8) | 0x80 |
                                    ((pNodeInfo->flags & NMTMNU_NODE_FLAG_COUNT_STATREQ) >> 6) |
                                    ((timerArg.argument.value & NMTMNU_TIMERARG_COUNT_SR) >> 8)));
        *pRet_p = timeru_modifyTimer(&pNodeInfo->timerHdlStatReq,
                                     nmtMnuInstance_g.statusRequestDelay,
                                     &timerArg);
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

\param[in]      nodeId_p            Node ID to process.
\param[in]      nodeNmtState_p      NMT state of the node.
\param[in]      nmtState_p          NMT state of the MN
\param[in]      errorCode_p         Error codes.
\param[out]     pRet_p              Pointer to store return value

\return The function returns 0 if the higher level event handler should continue
        processing or -1 if it should exit.
*/
//------------------------------------------------------------------------------
static INT processNodeEventStatusResponse(UINT nodeId_p,
                                          tNmtState nodeNmtState_p,
                                          tNmtState nmtState_p,
                                          UINT16 errorCode_p,
                                          tOplkError* pRet_p)
{
    tTimerArg           timerArg;
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
    *pRet_p = checkNmtState(nodeId_p,
                            pNodeInfo,
                            nodeNmtState_p,
                            errorCode_p,
                            nmtState_p);
    if (*pRet_p != kErrorOk)
    {
        if (*pRet_p == kErrorReject)
            *pRet_p = kErrorOk;

        return 0;
    }

    if (nmtState_p == kNmtMsPreOperational1)
    {
        // request next StatusResponse immediately
        *pRet_p = statusu_requestStatusResponse(nodeId_p, cbStatusResponse);
        if (*pRet_p != kErrorOk)
        {
            NMTMNU_DBG_POST_TRACE_VALUE(kNmtMnuIntNodeEventStatusResponse,
                                        nodeId_p,
                                        *pRet_p);
        }
    }
    else if ((pNodeInfo->flags & NMTMNU_NODE_FLAG_ISOCHRON) == 0)
    {   // start timer
        // not isochronously accessed CN (e.g. async-only or stopped CN)
        NMTMNU_SET_FLAGS_TIMERARG_STATREQ(pNodeInfo, nodeId_p, timerArg);

        NMTMNU_DBG_POST_TRACE_VALUE(kNmtMnuIntNodeEventStatusResponse,
                                    nodeId_p,
                                    ((pNodeInfo->nodeState << 8) | 0x80 |
                                     ((pNodeInfo->flags & NMTMNU_NODE_FLAG_COUNT_STATREQ) >> 6) |
                                     ((timerArg.argument.value & NMTMNU_TIMERARG_COUNT_SR) >> 8)));
        *pRet_p = timeru_modifyTimer(&pNodeInfo->timerHdlStatReq,
                                     nmtMnuInstance_g.statusRequestDelay,
                                     &timerArg);
    }

    return 0;
}

//------------------------------------------------------------------------------
/**
\brief  Process NoStatustResponse node event

The function processes the internal node event kNmtMnuIntNodeEventNoStatusResponse.

\param[in]      nodeId_p            Node ID to process.
\param[in]      nodeNmtState_p      NMT state of the node.
\param[in]      nmtState_p          NMT state of the MN
\param[in]      errorCode_p         Error codes.
\param[out]     pRet_p              Pointer to store return value

\return The function returns 0 if the higher level event handler should continue
        processing or -1 if it should exit.
*/
//------------------------------------------------------------------------------
static INT processNodeEventNoStatusResponse(UINT nodeId_p,
                                            tNmtState nodeNmtState_p,
                                            tNmtState nmtState_p,
                                            UINT16 errorCode_p,
                                            tOplkError* pRet_p)
{
    tNmtMnuNodeInfo*    pNodeInfo = NMTMNU_GET_NODEINFO(nodeId_p);

    // check NMT state of CN
    *pRet_p = checkNmtState(nodeId_p,
                            pNodeInfo,
                            nodeNmtState_p,
                            errorCode_p,
                            nmtState_p);
    if (*pRet_p == kErrorReject)
        *pRet_p = kErrorOk;

    return 0;
}

//------------------------------------------------------------------------------
/**
\brief  Process Error node event

The function processes the internal node event kNmtMnuIntNodeEventError.

\param[in]      nodeId_p            Node ID to process.
\param[in]      nodeNmtState_p      NMT state of the node.
\param[in]      nmtState_p          NMT state of the MN
\param[in]      errorCode_p         Error codes.
\param[out]     pRet_p              Pointer to store return value

\return The function returns 0 if the higher level event handler should continue
        processing or -1 if it should exit.
*/
//------------------------------------------------------------------------------
static INT processNodeEventError(UINT nodeId_p,
                                 tNmtState nodeNmtState_p,
                                 tNmtState nmtState_p,
                                 UINT16 errorCode_p,
                                 tOplkError* pRet_p)
{
    tNmtMnuNodeInfo*    pNodeInfo = NMTMNU_GET_NODEINFO(nodeId_p);

    UNUSED_PARAMETER(nodeNmtState_p);

    // currently only issued on kNmtNodeCommandConfErr
    if ((pNodeInfo->nodeState != kNmtMnuNodeStateIdentified) &&
        (pNodeInfo->nodeState != kNmtMnuNodeStateSwOk) &&
        (pNodeInfo->nodeState != kNmtMnuNodeStateConfRestored))
    {   // wrong CN state, ignore event
        return 0;
    }

    // check NMT state of CN
    *pRet_p = checkNmtState(nodeId_p,
                            pNodeInfo,
                            kNmtCsNotActive,
                            errorCode_p,
                            nmtState_p);
    if (*pRet_p == kErrorReject)
        *pRet_p = kErrorOk;

    return 0;
}

//------------------------------------------------------------------------------
/**
\brief  Process software update done node event

The function processes the internal node event kNmtMnuIntNodeEventSwUpdated.

\param[in]      nodeId_p            Node ID to process.
\param[in]      nodeNmtState_p      NMT state of the node.
\param[in]      nmtState_p          NMT state of the MN
\param[in]      errorCode_p         Error codes.
\param[out]     pRet_p              Pointer to store return value

\return The function returns 0 if the higher level event handler should continue
        processing or -1 if it should exit.
*/
//------------------------------------------------------------------------------
static INT processNodeEventSwUpdated(UINT nodeId_p,
                                     tNmtState nodeNmtState_p,
                                     tNmtState nmtState_p,
                                     UINT16 errorCode_p,
                                     tOplkError* pRet_p)
{
    tNmtMnuNodeInfo*    pNodeInfo;

    UNUSED_PARAMETER(nodeNmtState_p);
    UNUSED_PARAMETER(errorCode_p);
    UNUSED_PARAMETER(nmtState_p);

    pNodeInfo = NMTMNU_GET_NODEINFO(nodeId_p);

    if (pNodeInfo->nodeState != kNmtMnuNodeStateIdentified)
    {
       return 0;
    }

    *pRet_p = nmtmnu_sendNmtCommand(nodeId_p, kNmtCmdSwReset);

    return 0;
}

//------------------------------------------------------------------------------
/**
\brief  Process software update okay node event

The function processes the internal node event kNmtMnuIntNodeEventSwOk.

\param[in]      nodeId_p            Node ID to process.
\param[in]      nodeNmtState_p      NMT state of the node.
\param[in]      nmtState_p          NMT state of the MN
\param[in]      errorCode_p         Error codes.
\param[out]     pRet_p              Pointer to store return value

\return The function returns 0 if the higher level event handler should continue
        processing or -1 if it should exit.
*/
//------------------------------------------------------------------------------
static INT processNodeEventSwOk(UINT nodeId_p,
                                tNmtState nodeNmtState_p,
                                tNmtState nmtState_p,
                                UINT16 errorCode_p,
                                tOplkError* pRet_p)
{
    tNmtMnuNodeInfo*    pNodeInfo;

    UNUSED_PARAMETER(nodeNmtState_p);
    UNUSED_PARAMETER(errorCode_p);
    UNUSED_PARAMETER(nmtState_p);

    pNodeInfo = NMTMNU_GET_NODEINFO(nodeId_p);

    if (pNodeInfo->nodeState != kNmtMnuNodeStateIdentified)
    {
       return 0;
    }

    pNodeInfo->nodeState = kNmtMnuNodeStateSwOk;

    processNodeEventBoot(nodeId_p,
                         nodeNmtState_p,
                         nmtState_p,
                         errorCode_p,
                         pRet_p);

    return 0;
}

//------------------------------------------------------------------------------
/**
\brief  Process ExecResetNode node event

The function processes the internal node event kNmtMnuIntNodeEventExecResetNode.

\param[in]      nodeId_p            Node ID to process.
\param[in]      nodeNmtState_p      NMT state of the node.
\param[in]      nmtState_p          NMT state of the MN
\param[in]      errorCode_p         Error codes.
\param[out]     pRet_p              Pointer to store return value

\return The function returns 0 if the higher level event handler should continue
        processing or -1 if it should exit.
*/
//------------------------------------------------------------------------------
static INT processNodeEventExecResetNode(UINT nodeId_p,
                                         tNmtState nodeNmtState_p,
                                         tNmtState nmtState_p,
                                         UINT16 errorCode_p,
                                         tOplkError* pRet_p)
{
    tNmtMnuNodeInfo*    pNodeInfo;

    UNUSED_PARAMETER(errorCode_p);
    UNUSED_PARAMETER(nmtState_p);
    UNUSED_PARAMETER(nodeNmtState_p);

    pNodeInfo = NMTMNU_GET_NODEINFO(nodeId_p);

    if ((pNodeInfo->nodeState != kNmtMnuNodeStateIdentified) &&
        (pNodeInfo->nodeState != kNmtMnuNodeStateSwOk))
    {   // wrong CN state, ignore event
        return 0;
    }

    pNodeInfo->nodeState = kNmtMnuNodeStateConfRestored;
    NMTMNU_DBG_POST_TRACE_VALUE(kNmtMnuIntNodeEventExecResetNode,
                                nodeId_p,
                                (((nodeNmtState_p & 0xFF) << 8) | kNmtCmdResetNode));

    // send NMT reset node to CN for activation of restored configuration
    *pRet_p = nmtmnu_sendNmtCommand(nodeId_p, kNmtCmdResetNode);

    return 0;
}

//------------------------------------------------------------------------------
/**
\brief  Process ExecResetConf node event

The function processes the internal node event kNmtMnuIntNodeEventExecResetConf.

\param[in]      nodeId_p            Node ID to process.
\param[in]      nodeNmtState_p      NMT state of the node.
\param[in]      nmtState_p          NMT state of the MN
\param[in]      errorCode_p         Error codes.
\param[out]     pRet_p              Pointer to store return value

\return The function returns 0 if the higher level event handler should continue
        processing or -1 if it should exit.
*/
//------------------------------------------------------------------------------
static INT processNodeEventExecResetConf(UINT nodeId_p,
                                         tNmtState nodeNmtState_p,
                                         tNmtState nmtState_p,
                                         UINT16 errorCode_p,
                                         tOplkError* pRet_p)
{
    tNmtMnuNodeInfo*    pNodeInfo;

    UNUSED_PARAMETER(nodeNmtState_p);
    UNUSED_PARAMETER(errorCode_p);
    UNUSED_PARAMETER(nmtState_p);

    pNodeInfo = NMTMNU_GET_NODEINFO(nodeId_p);

    if ((pNodeInfo->nodeState != kNmtMnuNodeStateSwOk) &&
        (pNodeInfo->nodeState != kNmtMnuNodeStateConfRestored))
    {   // wrong CN state
       // ignore event
       return 0;
    }

    pNodeInfo->nodeState = kNmtMnuNodeStateResetConf;
    NMTMNU_DBG_POST_TRACE_VALUE(kNmtMnuIntNodeEventExecResetConf,
                                nodeId_p,
                                (((nodeNmtState_p & 0xFF) << 8) | kNmtCmdResetConfiguration));

    // send NMT reset configuration to CN for activation of configuration
    *pRet_p = nmtmnu_sendNmtCommand(nodeId_p, kNmtCmdResetConfiguration);

    return 0;
}

//------------------------------------------------------------------------------
/**
\brief  Process heartbeat node event

The function processes the internal node event kNmtMnuIntNodeEventHeartbeat.

\param[in]      nodeId_p            Node ID to process.
\param[in]      nodeNmtState_p      NMT state of the node.
\param[in]      nmtState_p          NMT state of the MN
\param[in]      errorCode_p         Error codes.
\param[out]     pRet_p              Pointer to store return value

\return The function returns 0 if the higher level event handler should continue
        processing or -1 if it should exit.
*/
//------------------------------------------------------------------------------
static INT processNodeEventHeartbeat(UINT nodeId_p,
                                     tNmtState nodeNmtState_p,
                                     tNmtState nmtState_p,
                                     UINT16 errorCode_p,
                                     tOplkError* pRet_p)
{
    tNmtMnuNodeInfo*    pNodeInfo = NMTMNU_GET_NODEINFO(nodeId_p);

    // check NMT state of CN
    *pRet_p = checkNmtState(nodeId_p,
                            pNodeInfo,
                            nodeNmtState_p,
                            errorCode_p,
                            nmtState_p);
    if (*pRet_p == kErrorReject)
        *pRet_p = kErrorOk;

    return 0;
}

//------------------------------------------------------------------------------
/**
\brief  Process TimerIdentReq node event

The function processes the internal node event kNmtMnuIntNodeEventTimerIdentReq.

\param[in]      nodeId_p            Node ID to process.
\param[in]      nodeNmtState_p      NMT state of the node.
\param[in]      nmtState_p          NMT state of the MN
\param[in]      errorCode_p         Error codes.
\param[out]     pRet_p              Pointer to store return value

\return The function returns 0 if the higher level event handler should continue
        processing or -1 if it should exit.
*/
//------------------------------------------------------------------------------
static INT processNodeEventTimerIdentReq(UINT nodeId_p,
                                         tNmtState nodeNmtState_p,
                                         tNmtState nmtState_p,
                                         UINT16 errorCode_p,
                                         tOplkError* pRet_p)
{
    UNUSED_PARAMETER(errorCode_p);
    UNUSED_PARAMETER(nmtState_p);
    UNUSED_PARAMETER(nodeNmtState_p);

    DEBUG_LVL_NMTMN_TRACE("TimerStatReq->IdentReq(%02X)\n", nodeId_p);

    // trigger IdentRequest again
    *pRet_p = identu_requestIdentResponse(nodeId_p, cbIdentResponse);
    if (*pRet_p != kErrorOk)
    {
        NMTMNU_DBG_POST_TRACE_VALUE(kNmtMnuIntNodeEventTimerIdentReq,
                                    nodeId_p,
                                    (((nodeNmtState_p & 0xFF) << 8) | *pRet_p));
        if (*pRet_p == kErrorInvalidOperation)
        {   // this can happen because of a bug in timer-linuxkernel.c
            // so ignore this error.
            *pRet_p = kErrorOk;
        }
    }

    return 0;
}

//------------------------------------------------------------------------------
/**
\brief  Process TimerStatReq node event

The function processes the internal node event kNmtMnuIntNodeEventTimerStatReq.

\param[in]      nodeId_p            Node ID to process.
\param[in]      nodeNmtState_p      NMT state of the node.
\param[in]      nmtState_p          NMT state of the MN
\param[in]      errorCode_p         Error codes.
\param[out]     pRet_p              Pointer to store return value

\return The function returns 0 if the higher level event handler should continue
        processing or -1 if it should exit.
*/
//------------------------------------------------------------------------------
static INT processNodeEventTimerStatReq(UINT nodeId_p,
                                        tNmtState nodeNmtState_p,
                                        tNmtState nmtState_p,
                                        UINT16 errorCode_p,
                                        tOplkError* pRet_p)
{
    UNUSED_PARAMETER(errorCode_p);
    UNUSED_PARAMETER(nodeNmtState_p);
    UNUSED_PARAMETER(nmtState_p);

    DEBUG_LVL_NMTMN_TRACE("TimerStatReq->StatReq(%02X)\n", nodeId_p);

    // request next StatusResponse
    *pRet_p = statusu_requestStatusResponse(nodeId_p, cbStatusResponse);
    if (*pRet_p != kErrorOk)
    {
       NMTMNU_DBG_POST_TRACE_VALUE(kNmtMnuIntNodeEventTimerStatReq,
                                   nodeId_p,
                                   (((nodeNmtState_p & 0xFF) << 8) | *pRet_p));
       if (*pRet_p == kErrorInvalidOperation)
       {   // the only situation when this should happen is, when
           // StatusResponse was already requested while processing
           // event IdentResponse.
           // so ignore this error.
           *pRet_p = kErrorOk;
       }
    }

    return 0;
}

//------------------------------------------------------------------------------
/**
\brief  Process TimerStateMon node event

The function processes the internal node event kNmtMnuIntNodeEventTimerStateMon.

\param[in]      nodeId_p            Node ID to process.
\param[in]      nodeNmtState_p      NMT state of the node.
\param[in]      nmtState_p          NMT state of the MN
\param[in]      errorCode_p         Error codes.
\param[out]     pRet_p              Pointer to store return value

\return The function returns 0 if the higher level event handler should continue
        processing or -1 if it should exit.
*/
//------------------------------------------------------------------------------
static INT processNodeEventTimerStateMon(UINT nodeId_p,
                                         tNmtState nodeNmtState_p,
                                         tNmtState nmtState_p,
                                         UINT16 errorCode_p,
                                         tOplkError* pRet_p)
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
    return processNodeEventTimerStatReq(nodeId_p,
                                        nodeNmtState_p,
                                        nmtState_p,
                                        errorCode_p,
                                        pRet_p);
}

//------------------------------------------------------------------------------
/**
\brief  Process TimerLonger node event

The function processes the internal node event kNmtMnuIntNodeEventTimerLonger.

\param[in]      nodeId_p            Node ID to process.
\param[in]      nodeNmtState_p      NMT state of the node.
\param[in]      nmtState_p          NMT state of the MN
\param[in]      errorCode_p         Error codes.
\param[out]     pRet_p              Pointer to store return value

\return The function returns 0 if the higher level event handler should continue
        processing or -1 if it should exit.
*/
//------------------------------------------------------------------------------
static INT processNodeEventTimerLonger(UINT nodeId_p,
                                       tNmtState nodeNmtState_p,
                                       tNmtState nmtState_p,
                                       UINT16 errorCode_p,
                                       tOplkError* pRet_p)
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
            *pRet_p = checkNmtState(nodeId_p,
                                    pNodeInfo,
                                    kNmtCsNotActive,
                                    E_NMT_BPO2,
                                    nmtState_p);
            if (*pRet_p == kErrorReject)
                *pRet_p = kErrorOk;
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

            if ((pNodeInfo->nodeCfg & NMT_NODEASSIGN_MANDATORY_CN) != 0)
            {
                // decrement mandatory slave counter
                nmtMnuInstance_g.mandatorySlaveCount--;
            }

            if (nmtState_p != kNmtMsReadyToOperate)
            {
                NMTMNU_DBG_POST_TRACE_VALUE(kNmtMnuIntNodeEventTimerLonger,
                                            nodeId_p,
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

\param[in]      nodeId_p            Node ID to process.
\param[in]      nodeNmtState_p      NMT state of the node.
\param[in]      nmtState_p          NMT state of the MN
\param[in]      errorCode_p         Error codes.
\param[out]     pRet_p              Pointer to store return value

\return The function returns 0 if the higher level event handler should continue
        processing or -1 if it should exit.
*/
//------------------------------------------------------------------------------
static INT processNodeEventNmtCmdSent(UINT nodeId_p,
                                      tNmtState nodeNmtState_p,
                                      tNmtState nmtState_p,
                                      UINT16 errorCode_p,
                                      tOplkError* pRet_p)
{
    UINT8               destinationNmtState;
    UINT8               initialNmtState;
    tObdSize            objSize = sizeof(initialNmtState);
    tTimerArg           timerArg;
    tNmtMnuNodeInfo*    pNodeInfo;

    UNUSED_PARAMETER(errorCode_p);
    UNUSED_PARAMETER(nmtState_p);

    pNodeInfo = NMTMNU_GET_NODEINFO(nodeId_p);

    // update expected NMT state with the one that results
    // from the sent NMT command
    destinationNmtState = (UINT8)(nodeNmtState_p & 0xFF);

    if (nodeNmtState_p == kNmtCsPreOperational2)
    {
        // If the MN sends NMTEnableReadyToOperate to the node, the initial and
        // destination NMT state is NMT_CS_PRE_OPERATIONAL_2. Therefore it is
        // not necessary to issue a status request frame to monitor the
        // C_NMT_STATE_TOLERANCE!

        // Get initial NMT state
        *pRet_p = obdu_readEntry(0x1F8F, nodeId_p, &initialNmtState, &objSize);
        if (*pRet_p != kErrorOk)
            return -1;

        if (initialNmtState == destinationNmtState)
        {
            // set NMT state change flag
            pNodeInfo->flags |= NMTMNU_NODE_FLAG_NMT_CMD_ISSUED;

            // Return without setting up timer for state tolerance monitoring,
            // the "longer" timer is already set up!
            // And no need to update 0x1F8F, it already has the right value!
            return -1;
        }
    }

    // write object 0x1F8F NMT_MNNodeExpState_AU8
    *pRet_p = obdu_writeEntry(0x1F8F, nodeId_p, &destinationNmtState, 1);
    if (*pRet_p != kErrorOk)
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
                                 nmtMnuInstance_g.statusRequestDelay,
                                 &timerArg);
    // finish processing, because NmtState_p is the expected and not the current state
    return -1;
}

//------------------------------------------------------------------------------
/**
\brief  Process internal node events

The function processes internal node events.

\param[in]      nodeId_p            Node ID to process.
\param[in]      nodeNmtState_p      NMT state of the node.
\param[in]      errorCode_p         Error codes.
\param[in]      nodeEvent_p         Occurred events.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError processInternalEvent(UINT nodeId_p,
                                       tNmtState nodeNmtState_p,
                                       UINT16 errorCode_p,
                                       tNmtMnuIntNodeEvent nodeEvent_p)
{
    tOplkError  ret = kErrorOk;
    tNmtState   nmtState;

    nmtState = nmtu_getNmtState();

#if defined(CONFIG_INCLUDE_NMT_RMN)
    if (NMT_IF_ACTIVE_CN(nmtState) &&
        (nmtMnuInstance_g.flags & NMTMNU_FLAG_REDUNDANCY) &&
        (nodeEvent_p == kNmtMnuIntNodeEventHeartbeat))
    {
        ret = processRedundancyHeartbeat(nodeId_p, nodeNmtState_p);
        goto Exit;
    }
#endif

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
                                                          nmtState,
                                                          E_NO_ERROR);
                    if (ret != kErrorOk)
                    {
                        if (ret == kErrorReject)
                            // wait for application
                            ret = kErrorOk;
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
                                                          nmtState,
                                                          E_NO_ERROR);
                    if (ret != kErrorOk)
                    {
                        if (ret == kErrorReject)
                            // wait for application
                            ret = kErrorOk;
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
                                                          nmtState,
                                                          E_NO_ERROR);
                    if (ret != kErrorOk)
                    {
                        if (ret == kErrorReject)
                            // wait for application
                            ret = kErrorOk;
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
                                                          nmtState,
                                                          E_NO_ERROR);
                    if (ret != kErrorOk)
                    {
                        if (ret == kErrorReject)
                            // ignore error code
                            ret = kErrorOk;
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

\param[in]      nodeId_p            Node ID to check.
\param[in,out]  pNodeInfo_p         Pointer to node information structure.
\param[in]      nodeNmtState_p      NMT state of the node.
\param[in]      errorCode_p         Error codes.
\param[in]      localNmtState_p     The local NMT state.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError checkNmtState(UINT nodeId_p,
                                tNmtMnuNodeInfo* pNodeInfo_p,
                                tNmtState nodeNmtState_p,
                                UINT16 errorCode_p,
                                tNmtState localNmtState_p)
{
    tOplkError  ret;
    tOplkError  retUpdate;
    tObdSize    obdSize;
    UINT8       nodeNmtState;
    UINT8       obdExpNmtState;
    UINT8       nmtStatePrev;
    tNmtState   expNmtState;

    // compute UINT8 of current NMT state
    nodeNmtState = ((UINT8)nodeNmtState_p & 0xFF);

    if (pNodeInfo_p->nodeState == kNmtMnuNodeStateUnknown)
    {   // CN is already in state unknown, which means that it got
        // NMT reset command earlier
        ret = kErrorReject;
        goto ExitButUpdate;
    }

    obdSize = 1;
    // read object 0x1F8F NMT_MNNodeExpState_AU8
    ret = obdu_readEntry(0x1F8F, nodeId_p, &obdExpNmtState, &obdSize);
    if (ret != kErrorOk)
        goto Exit;

    // compute expected NMT state
    expNmtState = (tNmtState)(obdExpNmtState | NMT_TYPE_CS);

    if (expNmtState == kNmtCsNotActive)
    {   // ignore the current state, because the CN shall be not active
        ret = kErrorReject;
        goto ExitButUpdate;
    }
    else if ((expNmtState == kNmtCsStopped) && (nodeNmtState_p == kNmtCsStopped))
    {
        // reset flags ISOCHRON and PREOP2_REACHED
        pNodeInfo_p->flags &= ~(NMTMNU_NODE_FLAG_ISOCHRON | NMTMNU_NODE_FLAG_PREOP2_REACHED);
    }
    else if ((expNmtState == kNmtCsPreOperational2) && (nodeNmtState_p == kNmtCsPreOperational2))
    {   // CN is PreOp2
        if ((pNodeInfo_p->flags & NMTMNU_NODE_FLAG_PREOP2_REACHED) == 0)
        {   // CN switched to PreOp2
            pNodeInfo_p->flags |= NMTMNU_NODE_FLAG_PREOP2_REACHED;

            if (pNodeInfo_p->nodeCfg & NMT_NODEASSIGN_ASYNCONLY_NODE)
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
                if (ret != kErrorOk)
                    goto Exit;
            }
        }
    }
    else if ((expNmtState == kNmtCsPreOperational2) && (nodeNmtState_p == kNmtCsReadyToOperate))
    {   // CN switched to ReadyToOp
        // delete timer for timeout handling
        ret = timeru_deleteTimer(&pNodeInfo_p->timerHdlLonger);
        if (ret != kErrorOk)
            goto Exit;

        pNodeInfo_p->nodeState = kNmtMnuNodeStateReadyToOp;

        // update object 0x1F8F NMT_MNNodeExpState_AU8 to ReadyToOp
        ret = obdu_writeEntry(0x1F8F, nodeId_p, &nodeNmtState, 1);
        if (ret != kErrorOk)
            goto Exit;

        if ((pNodeInfo_p->nodeCfg & NMT_NODEASSIGN_MANDATORY_CN) != 0)
        {   // node is a mandatory CN -> decrement counter
            nmtMnuInstance_g.mandatorySlaveCount--;
        }
        if (localNmtState_p >= kNmtMsReadyToOperate)
        {   // start procedure CheckCommunication for this node
            ret = nodeCheckCom(nodeId_p, pNodeInfo_p);
            if (ret != kErrorOk)
                goto ExitButUpdate;

            if ((localNmtState_p == kNmtMsOperational) && (pNodeInfo_p->nodeState == kNmtMnuNodeStateComChecked))
            {
                NMTMNU_DBG_POST_TRACE_VALUE(0,
                                            nodeId_p,
                                            (((nodeNmtState_p & 0xFF) << 8) | kNmtCmdStartNode));

                // immediately start optional CN, because communication is always OK (e.g. async-only CN)
                ret = nmtmnu_sendNmtCommand(nodeId_p, kNmtCmdStartNode);
                if (ret != kErrorOk)
                    goto Exit;
            }
        }
    }
    else if ((pNodeInfo_p->nodeState == kNmtMnuNodeStateComChecked) && (nodeNmtState_p == kNmtCsOperational))
    {   // CN switched to OPERATIONAL
        pNodeInfo_p->nodeState = kNmtMnuNodeStateOperational;

        if ((pNodeInfo_p->nodeCfg & NMT_NODEASSIGN_MANDATORY_CN) != 0)
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
            errorCode_p = E_NMT_WRONG_STATE;
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
        ret = nmtMnuInstance_g.pfnCbNodeEvent(nodeId_p,
                                              kNmtNodeEventError,
                                              nodeNmtState_p,
                                              errorCode_p,
                                              (pNodeInfo_p->nodeCfg & NMT_NODEASSIGN_MANDATORY_CN) != 0);
        if (ret != kErrorOk)
            goto ExitButUpdate;

        NMTMNU_DBG_POST_TRACE_VALUE(0,
                                    nodeId_p,
                                    (((nodeNmtState_p & 0xFF) << 8) | kNmtCmdResetNode));

        // reset CN
        // store error code in NMT command data for diagnostic purpose
        ami_setUint16Le(&beErrorCode, errorCode_p);
        ret = nmtmnu_sendNmtCommandEx(nodeId_p,
                                      kNmtCmdResetNode,
                                      &beErrorCode,
                                      sizeof(beErrorCode));
        if (ret == kErrorOk)
            ret = kErrorReject;

        // Sometimes we get an invalid state in a StatusResponse/IdentResponse
        // In this case we don't want to store it and inform the application!
        // If it's a correct NMT state we do update it.
        switch (nodeNmtState_p)
        {
            case kNmtCsNotActive:
            case kNmtCsBasicEthernet:
            case kNmtCsPreOperational1:
            case kNmtCsPreOperational2:
            case kNmtCsReadyToOperate:
            case kNmtCsOperational:
                goto ExitButUpdate;

            default:
                goto Exit;
        }
    }

ExitButUpdate:
    // check if NMT_MNNodeCurrState_AU8 has to be changed
    obdSize = 1;
    retUpdate = obdu_readEntry(0x1F8E, nodeId_p, &nmtStatePrev, &obdSize);
    if (retUpdate != kErrorOk)
    {
        ret = retUpdate;
        goto Exit;
    }

    if (nodeNmtState != nmtStatePrev)
    {
        // update object 0x1F8E NMT_MNNodeCurrState_AU8
        retUpdate = obdu_writeEntry(0x1F8E, nodeId_p, &nodeNmtState, 1);
        if (retUpdate != kErrorOk)
        {
            ret = retUpdate;
            goto Exit;
        }

        retUpdate = nmtMnuInstance_g.pfnCbNodeEvent(nodeId_p,
                                                    kNmtNodeEventNmtState,
                                                    nodeNmtState_p,
                                                    errorCode_p,
                                                    (pNodeInfo_p->nodeCfg & NMT_NODEASSIGN_MANDATORY_CN) != 0);
        if (retUpdate != kErrorOk)
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

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError reset(void)
{
    tOplkError  ret;
    UINT        index;

    ret = timeru_deleteTimer(&nmtMnuInstance_g.timerHdlNmtState);
    for (index = 1; index <= tabentries(nmtMnuInstance_g.aNodeInfo); index++)
    {
        ret = timeru_deleteTimer(&NMTMNU_GET_NODEINFO(index)->timerHdlStatReq);
        if (ret != kErrorOk)
        {
            DEBUG_LVL_ERROR_TRACE("%s delete StatReq timer failed with 0x%X\n",
                                  __func__,
                                  ret);
        }

        ret = timeru_deleteTimer(&NMTMNU_GET_NODEINFO(index)->timerHdlLonger);
        if (ret != kErrorOk)
        {
            DEBUG_LVL_ERROR_TRACE("%s delete longer timer failed with 0x%X\n",
                                  __func__,
                                  ret);
        }
    }

    nmtMnuInstance_g.prcPResMnTimeoutNs = 0;

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Perform measure phase of PRC node insertion

The function performs the measure phase of a PRC node insertion

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError prcMeasure(void)
{
    tOplkError          ret = kErrorOk;
    UINT                nodeId;
    tNmtMnuNodeInfo*    pNodeInfo;
    BOOL                fSyncReqSentToPredNode;
    UINT                nodeIdFirstNode;
    UINT                nodeIdPredNode;
    UINT                nodeIdPrevSyncReq;

    fSyncReqSentToPredNode = FALSE;
    nodeIdPredNode = C_ADR_INVALID;
    nodeIdPrevSyncReq = C_ADR_INVALID;
    nodeIdFirstNode = C_ADR_INVALID;

    for (nodeId = 1; nodeId < 254; nodeId++)
    {
        pNodeInfo = NMTMNU_GET_NODEINFO(nodeId);
        if (pNodeInfo == NULL)
            continue;

        if ((pNodeInfo->nodeCfg & NMT_NODEASSIGN_PRES_CHAINING) &&
            ((pNodeInfo->flags & NMTMNU_NODE_FLAG_ISOCHRON) ||
             (pNodeInfo->prcFlags & NMTMNU_NODE_FLAG_PRC_ADD_IN_PROGRESS)))
        {
            if (nodeIdFirstNode == C_ADR_INVALID)
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
                if (nodeIdPredNode == C_ADR_INVALID)
                {   // No predecessor node in isochronous phase
                    pNodeInfo->relPropagationDelayNs = C_DLL_T_IFG;
                    // No SyncReq needs to be send
                }
                else
                {   // Predecessor node exists
                    tDllSyncRequest    syncRequestData;
                    size_t             size;

                    syncRequestData.syncControl = PLK_SYNC_DEST_MAC_ADDRESS_VALID;
                    size = sizeof(UINT) + sizeof(UINT32);

                    if (fSyncReqSentToPredNode == FALSE)
                    {
                        syncRequestData.nodeId = nodeIdPredNode;

                        ret = syncu_requestSyncResponse(prcCbSyncResMeasure, &syncRequestData, size);
                        if (ret != kErrorOk)
                            goto Exit;
                    }

                    syncRequestData.nodeId = nodeId;
                    ret = syncu_requestSyncResponse(prcCbSyncResMeasure, &syncRequestData, size);
                    if (ret != kErrorOk)
                        goto Exit;

                    fSyncReqSentToPredNode = TRUE;
                    nodeIdPrevSyncReq = nodeId;
                }
            }
            else
                fSyncReqSentToPredNode = FALSE;

            nodeIdPredNode = nodeId;
        }
    }

    if (nodeIdPrevSyncReq != C_ADR_INVALID)
    {   // At least one SyncReq has been sent
        pNodeInfo = NMTMNU_GET_NODEINFO(nodeIdPrevSyncReq);
        pNodeInfo->prcFlags |= NMTMNU_NODE_FLAG_PRC_CALL_MEASURE;
    }
    else
    {   // No SyncReq has been sent
        if (nodeIdFirstNode == C_ADR_INVALID)
        {   // No add-in-progress node has been found. This might happen
            // due to reset-node NMT commands which were issued
            // between the first and the second measure scan.
            nmtMnuInstance_g.flags &= ~NMTMNU_FLAG_PRC_ADD_IN_PROGRESS;

            // A new insertion process can be started
            ret = addNodeIsochronous(C_ADR_INVALID);
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

\param[in]      nodeIdFirstNode_p   Node ID of the first (lowest node ID) of nodes
                                    whose addition is in progress.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError prcCalculate(UINT nodeIdFirstNode_p)
{
    tOplkError          ret;
    UINT                nodeId;
    tNmtMnuNodeInfo*    pNodeInfo;
    UINT                nodeIdPredNode;
    UINT32              pResResponseTimeNs;
    UINT32              pResMnTimeoutNs;

    if ((nodeIdFirstNode_p == C_ADR_INVALID) ||
        (nodeIdFirstNode_p >= C_ADR_BROADCAST))
    {   // invalid node ID specified
        return kErrorInvalidNodeId;
    }

    nodeIdPredNode = C_ADR_INVALID;
    for (nodeId = nodeIdFirstNode_p; nodeId < 254; nodeId++)
    {
        pNodeInfo = NMTMNU_GET_NODEINFO(nodeId);
        if (pNodeInfo == NULL)
            continue;

        if ((pNodeInfo->nodeCfg & NMT_NODEASSIGN_PRES_CHAINING) &&
             ((pNodeInfo->flags & NMTMNU_NODE_FLAG_ISOCHRON) ||
              (pNodeInfo->prcFlags & NMTMNU_NODE_FLAG_PRC_ADD_IN_PROGRESS)))
        {
            ret = prcCalcPResResponseTimeNs(nodeId, nodeIdPredNode, &pResResponseTimeNs);
            if (ret != kErrorOk)
                goto Exit;

            if (pNodeInfo->pResTimeFirstNs < pResResponseTimeNs)
            {
                pNodeInfo->pResTimeFirstNs = pResResponseTimeNs;
                if (pNodeInfo->flags & NMTMNU_NODE_FLAG_ISOCHRON)
                    pNodeInfo->prcFlags |= NMTMNU_NODE_FLAG_PRC_SHIFT_REQUIRED;
            }

            nodeIdPredNode = nodeId;
        }
    }

    ret = prcCalcPResChainingSlotTimeNs(nodeIdPredNode, &pResMnTimeoutNs);
    if (ret != kErrorOk)
        goto Exit;

    if (nmtMnuInstance_g.prcPResMnTimeoutNs < pResMnTimeoutNs)
    {
        tDllNodeInfo    dllNodeInfo;

        nmtMnuInstance_g.prcPResMnTimeoutNs = pResMnTimeoutNs;

        OPLK_MEMSET(&dllNodeInfo, 0, sizeof(tDllNodeInfo));
        dllNodeInfo.presTimeoutNs = pResMnTimeoutNs;
        dllNodeInfo.nodeId = C_ADR_MN_DEF_NODE_ID;

        ret = dllucal_configNode(&dllNodeInfo);
        if (ret != kErrorOk)
            goto Exit;
    }

    // enter next phase
    ret = prcShift(C_ADR_INVALID);

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Calculation of PRes Response Time of a node

The function calculates the PRes Response Time of the specified node.

\param[in]      nodeId_p                Node ID for which to calculate time.
\param[in]      nodeIdPredNode_p        Node ID of the predecessor node.
\param[out]     pPResResponseTimeNs_p   Pointer to store calculated time.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError prcCalcPResResponseTimeNs(UINT nodeId_p,
                                            UINT nodeIdPredNode_p,
                                            UINT32* pPResResponseTimeNs_p)
{
    tOplkError              ret = kErrorOk;
    UINT16                  pResPayloadLimitPredNode;
    const tNmtMnuNodeInfo*  pNodeInfoPredNode;
    tObdSize                obdSize;

    if (nodeIdPredNode_p == C_ADR_INVALID)
    {   // no predecessor node passed
        nodeIdPredNode_p = prcFindPredecessorNode(nodeId_p);

        if (nodeIdPredNode_p == C_ADR_INVALID)
        {   // no predecessor node found
            // PRes Response Time of first PRC node is defined to 0
            *pPResResponseTimeNs_p = 0;
            goto Exit;
        }
    }

    pNodeInfoPredNode = NMTMNU_GET_NODEINFO(nodeIdPredNode_p);

    // read object 0x1F8D NMT_PResPayloadLimitList_AU16
    obdSize = 2;
    ret = obdu_readEntry(0x1F8D, nodeIdPredNode_p, &pResPayloadLimitPredNode, &obdSize);
    if (ret != kErrorOk)
        goto Exit;

    *pPResResponseTimeNs_p =
        // PRes Response Time of predecessor node
        pNodeInfoPredNode->pResTimeFirstNs +
        // Transmission time for PRes frame of predecessor node
        (8 * C_DLL_T_BITTIME * (pResPayloadLimitPredNode +
                                C_DLL_T_EPL_PDO_HEADER +
                                C_DLL_T_ETH2_WRAPPER) +
         C_DLL_T_PREAMBLE) +
        // Relative propagation delay from predecessor node to addressed node
        NMTMNU_GET_NODEINFO(nodeId_p)->relPropagationDelayNs +
        // Time correction (hub jitter and part of measurement inaccuracy)
        nmtMnuInstance_g.prcPResTimeFirstCorrectionNs;

    // apply negative offset for the second node, only
    if (pNodeInfoPredNode->pResTimeFirstNs == 0)
    {
        if (*pPResResponseTimeNs_p > nmtMnuInstance_g.prcPResTimeFirstNegOffsetNs)
            *pPResResponseTimeNs_p -= nmtMnuInstance_g.prcPResTimeFirstNegOffsetNs;
        else
            *pPResResponseTimeNs_p = 0;
    }

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Calculation of PRes chaining slot time

The function calculates the PRes chaining slot time.

\param[in]      nodeIdLastNode_p            Node ID of the last node.
\param[out]     pPResChainingSlotTimeNs_p   Pointer to store calculated time.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError prcCalcPResChainingSlotTimeNs(UINT nodeIdLastNode_p,
                                                UINT32* pPResChainingSlotTimeNs_p)
{
    tOplkError  ret;
    UINT16      pResActPayloadLimit;
    UINT16      cnPReqPayloadLastNode;
    UINT32      cnResTimeoutLastNodeNs;
    tObdSize    obdSize;

    // read object 0x1F98 NMT_CycleTiming_REC
    // Sub-Index 05h PResActPayloadLimit_U16
    obdSize = 2;
    ret = obdu_readEntry(0x1F98, 5, &pResActPayloadLimit, &obdSize);
    if (ret != kErrorOk)
        goto Exit;

    // read object 0x1F8B NMT_MNPReqPayloadLimitList_AU16
    obdSize = 2;
    ret = obdu_readEntry(0x1F8B, nodeIdLastNode_p, &cnPReqPayloadLastNode, &obdSize);
    if (ret != kErrorOk)
        goto Exit;

    // read object 0x1F92 NMT_MNCNPResTimeout_AU32
    obdSize = 4;
    ret = obdu_readEntry(0x1F92, nodeIdLastNode_p, &cnResTimeoutLastNodeNs, &obdSize);
    if (ret != kErrorOk)
        goto Exit;

    *pPResChainingSlotTimeNs_p =
        // Transmission time for PResMN frame
        (8 * C_DLL_T_BITTIME * (pResActPayloadLimit +
                                C_DLL_T_EPL_PDO_HEADER +
                                C_DLL_T_ETH2_WRAPPER) +
         C_DLL_T_PREAMBLE) +
        // PRes Response Time of last node
        NMTMNU_GET_NODEINFO(nodeIdLastNode_p)->pResTimeFirstNs +
        // Relative propagation delay from last node to MN
        // Due to Soft-MN limitations, NMT_MNCNPResTimeout_AU32.CNResTimeout
        // of the last node is used.
        cnResTimeoutLastNodeNs -
        // Transmission time for PReq frame of last node
        (8 * C_DLL_T_BITTIME * (cnPReqPayloadLastNode +
                                C_DLL_T_EPL_PDO_HEADER +
                                C_DLL_T_ETH2_WRAPPER) +
         C_DLL_T_PREAMBLE) +
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

\param[in]      nodeId_p            Node ID of the processed node.

\return The function returns the node ID of the predecessor node or
        C_ADR_INVALID if no node was found.
*/
//------------------------------------------------------------------------------
static UINT prcFindPredecessorNode(UINT nodeId_p)
{
    UINT                    nodeId;
    const tNmtMnuNodeInfo*  pNodeInfo;

    for (nodeId = nodeId_p - 1; nodeId >= 1; nodeId--)
    {
        pNodeInfo = NMTMNU_GET_NODEINFO(nodeId);
        if (pNodeInfo == NULL)
            continue;

        if ((pNodeInfo->nodeCfg & NMT_NODEASSIGN_PRES_CHAINING) &&
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

\param[in]      nodeId_p            Node ID of the node.
\param[in]      pSyncResponse_p     Pointer to SyncResponse frame.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError prcCbSyncResMeasure(UINT nodeId_p,
                                      const tSyncResponse* pSyncResponse_p)
{
    tOplkError          ret;
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
    syncNodeNumber = ami_getUint32Le(&pSyncResponse_p->syncNodeNumberLe);

    if (syncNodeNumber != nodeIdPredNode)
    {   // SyncNodeNumber does not match predecessor node
        prcSyncError(pNodeInfo);
        goto Exit;
    }

    pNodeInfo->relPropagationDelayNs = ami_getUint32Le(&pSyncResponse_p->syncDelayLe);

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

\param[in,out]  pNodeInfo_p         Pointer to node information structure.
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

\param[in]      nodeIdPrevShift_p   Node ID of previously shifted node.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError prcShift(UINT nodeIdPrevShift_p)
{
    tOplkError          ret = kErrorOk;
    UINT                nodeId;
    tNmtMnuNodeInfo*    pNodeInfo;
    tDllSyncRequest     syncRequestData;
    size_t              size;

    if (nodeIdPrevShift_p == C_ADR_INVALID)
        nodeIdPrevShift_p = 254;

    // The search starts with the previous shift node
    // as this node might require a second SyncReq
    nodeId = nodeIdPrevShift_p;
    do
    {
        pNodeInfo = NMTMNU_GET_NODEINFO(nodeId);
        if (pNodeInfo == NULL)
            continue;

        if ((pNodeInfo->nodeCfg & NMT_NODEASSIGN_PRES_CHAINING) &&
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
        ret = prcAdd(C_ADR_INVALID);
        goto Exit;
    }

    // Call shift on reception of SyncRes
    pNodeInfo->prcFlags |= NMTMNU_NODE_FLAG_PRC_CALL_SHIFT;

    // Send SyncReq
    syncRequestData.nodeId = nodeId;
    syncRequestData.syncControl = PLK_SYNC_PRES_TIME_FIRST_VALID |
                                  PLK_SYNC_DEST_MAC_ADDRESS_VALID;
    syncRequestData.pResTimeFirst = pNodeInfo->pResTimeFirstNs;
    size = sizeof(UINT) + 2 * sizeof(UINT32);
    ret = syncu_requestSyncResponse(prcCbSyncResShift, &syncRequestData, size);

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Callback function for Sync Response after shifting

The function performs the the callback function for SyncRes frames after sending
of SyncReq which is used for shifting.

\param[in]      nodeId_p            Node ID of node.
\param[in]      pSyncResponse_p     Pointer to received SyncRes frame.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError prcCbSyncResShift(UINT nodeId_p,
                                    const tSyncResponse* pSyncResponse_p)
{
    tOplkError          ret;
    tNmtMnuNodeInfo*    pNodeInfo;

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

\param[in]      nodeIdPrevAdd_p     Node ID of previously added node.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError prcAdd(UINT nodeIdPrevAdd_p)
{
    tOplkError          ret = kErrorOk;
    tObdSize            obdSize;
    UINT32              cycleLenUs;
    UINT32              cNLossOfSocToleranceNs;
    UINT                nodeId;
    tNmtMnuNodeInfo*    pNodeInfo;
    tDllSyncRequest     syncReqData;
    UINT                syncReqNum;
    tNmtMnuNodeInfo*    pNodeInfoLastSyncReq;

    // prepare SyncReq
    syncReqData.syncControl = PLK_SYNC_PRES_MODE_SET |
                              PLK_SYNC_PRES_TIME_FIRST_VALID |
                              PLK_SYNC_PRES_FALL_BACK_TIMEOUT_VALID |
                              PLK_SYNC_DEST_MAC_ADDRESS_VALID;

    // read object 0x1006 NMT_CycleLen_U32
    obdSize = sizeof(UINT32);
    ret = obdu_readEntry(0x1006, 0, &cycleLenUs, &obdSize);
    if (ret != kErrorOk)
        goto Exit;

    // read object 0x1C14 DLL_CNLossOfSocTolerance_U32
    ret = obdu_readEntry(0x1C14, 0, &cNLossOfSocToleranceNs, &obdSize);
    if (ret != kErrorOk)
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
            syncReqData.nodeId = nodeId;
            syncReqData.pResTimeFirst = pNodeInfo->pResTimeFirstNs;

            ret = syncu_requestSyncResponse(prcCbSyncResAdd, &syncReqData, sizeof(syncReqData));
            if (ret != kErrorOk)
                goto Exit;

            pNodeInfo->prcFlags |= NMTMNU_NODE_FLAG_PRC_ADD_SYNCREQ_SENT;
            syncReqNum++;
            pNodeInfoLastSyncReq = pNodeInfo;

            if (syncReqNum == NMTMNU_PRC_NODE_ADD_MAX_NUM)
                break;
        }
    }

    if (pNodeInfoLastSyncReq != NULL)
        pNodeInfoLastSyncReq->prcFlags |= NMTMNU_NODE_FLAG_PRC_CALL_ADD;
    else
    {   // No nodes need to be added to the isochronous phase
        if (nodeIdPrevAdd_p != C_ADR_INVALID)
        {
            pNodeInfo = NMTMNU_GET_NODEINFO(nodeIdPrevAdd_p);
            if (pNodeInfo->prcFlags & NMTMNU_NODE_FLAG_PRC_VERIFY)
            {   // Verification of the last node is still in progress
                // Wait for verify and try again
                pNodeInfo->prcFlags |= NMTMNU_NODE_FLAG_PRC_CALL_ADD;
                goto Exit;
            }
        }

        // Either no nodes had to be added, at all, or add is finished
        nmtMnuInstance_g.flags &= ~NMTMNU_FLAG_PRC_ADD_IN_PROGRESS;

        // A new insertion process can be started
        ret = addNodeIsochronous(C_ADR_INVALID);
    }

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Callback function for Sync Response for insertion

The function performs the the callback function for SyncRes frames after sending
of SyncReq which is used for insertion.

\param[in]      nodeId_p            Node ID of node.
\param[in]      pSyncResponse_p     Pointer to received SyncRes frame.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError prcCbSyncResAdd(UINT nodeId_p,
                                  const tSyncResponse* pSyncResponse_p)
{
    tOplkError          ret;
    tNmtMnuNodeInfo*    pNodeInfo;

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
    if (ret != kErrorOk)
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

\param[in]      nodeId_p            Node ID of node.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError prcVerify(UINT nodeId_p)
{
    tOplkError          ret;
    tNmtMnuNodeInfo*    pNodeInfo;
    tDllSyncRequest     syncReqData;
    size_t              size;

    ret = kErrorOk;

    pNodeInfo = NMTMNU_GET_NODEINFO(nodeId_p);
    if (pNodeInfo->flags & NMTMNU_NODE_FLAG_ISOCHRON)
    {
        syncReqData.nodeId = nodeId_p;
        syncReqData.syncControl = PLK_SYNC_DEST_MAC_ADDRESS_VALID;
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

\param[in]      nodeId_p            Node ID of node.
\param[in]      pSyncResponse_p     Pointer to received SyncRes frame.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError prcCbSyncResVerify(UINT nodeId_p,
                                     const tSyncResponse* pSyncResponse_p)
{
    tOplkError          ret;
    tNmtMnuNodeInfo*    pNodeInfo;
    UINT32              pResTimeFirstNs;

    pNodeInfo = NMTMNU_GET_NODEINFO(nodeId_p);

    if (pSyncResponse_p == NULL)
    {   // SyncRes not received
        prcSyncError(pNodeInfo);
        goto Exit;
    }

    pResTimeFirstNs = ami_getUint32Le(&pSyncResponse_p->presTimeFirstLe);

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

\param[in]      nodeId_p            Node ID of node.
\param[in]      pSyncResponse_p     Pointer to received SyncRes frame.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError prcCbSyncResNextAction(UINT nodeId_p,
                                         const tSyncResponse* pSyncResponse_p)
{
    tOplkError          ret = kErrorOk;
    tNmtMnuNodeInfo*    pNodeInfo;
    tNmtCommand         nmtCommand;

    UNUSED_PARAMETER(pSyncResponse_p);

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
        if (ret != kErrorOk)
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
            if (ret != kErrorOk)
                goto Exit;
            break;

        case NMTMNU_NODE_FLAG_PRC_CALL_ADD:
            pNodeInfo->prcFlags &= ~NMTMNU_NODE_FLAG_PRC_CALL_MASK;
            ret = prcAdd(nodeId_p);
            if (ret != kErrorOk)
                goto Exit;
            break;

        default:
            break;
    }

    if (pNodeInfo->prcFlags & NMTMNU_NODE_FLAG_PRC_VERIFY)
        ret = prcVerify(nodeId_p);

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Set PRC reset flags

Before sending a reset-node NMT command, PRes Chaining has to be disabled by
sending an appropriate SyncReq. The requested NMT command is stored until the
SyncRes returns. Commands of higher priority overwrite those of lower priority.
Furthermore, extended is converted to plain NMT.

\param[in,out]  pNodeInfo_p         Pointer to node information structure.
\param[in]      nmtCommand_p        NMT command.
*/
//------------------------------------------------------------------------------
static void prcSetFlagsNmtCommandReset(tNmtMnuNodeInfo* pNodeInfo_p,
                                       tNmtCommand nmtCommand_p)
{
    UINT16  prcFlagsReset;

    prcFlagsReset = pNodeInfo_p->prcFlags & NMTMNU_NODE_FLAG_PRC_RESET_MASK;

    switch (nmtCommand_p)
    {
        case kNmtCmdResetNode:
        case kNmtCmdResetNodeEx:
            prcFlagsReset = NMTMNU_NODE_FLAG_PRC_RESET_NODE;
            break;

        case kNmtCmdResetCommunication:
        case kNmtCmdResetCommunicationEx:
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
        case kNmtCmdResetConfigurationEx:
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
        case kNmtCmdSwResetEx:
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
        case kNmtCmdStopNodeEx:
            if (prcFlagsReset == 0)
                prcFlagsReset = NMTMNU_NODE_FLAG_PRC_STOP_NODE;
            break;

        default:
            break;
    }

    pNodeInfo_p->prcFlags &= ~NMTMNU_NODE_FLAG_PRC_RESET_MASK;
    pNodeInfo_p->prcFlags |= prcFlagsReset;
}

//------------------------------------------------------------------------------
/**
\brief  Handle NMT reset commands to PRes Chaining nodes

This function handles NMT reset commands intended to be dispatched to PRes Chaining
nodes. If the node's PRes Chaining mode is activated, a syncRequest frame is
issued to the node. The NMT command is stored for later operation.
The function returns with \p pfWaitForSyncResp_p if a syncRequest was issued
to the node.

\param[in]      nodeId_p            Node ID
\param[in]      nmtCommand_p        NMT command to be issued to the node
\param[out]     pfWaitForSyncResp_p Pointer to flag that instructs to wait for syncResponse

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError prcHandleNmtReset(UINT nodeId_p,
                                    tNmtCommand nmtCommand_p,
                                    BOOL* pfWaitForSyncResp_p)
{
    tOplkError          ret = kErrorOk;
    tNmtMnuNodeInfo*    pNodeInfo = NMTMNU_GET_NODEINFO(nodeId_p);

    if (pfWaitForSyncResp_p == NULL)
    {
        ret = kErrorNmtInvalidParam;
        goto Exit;
    }

    // default is not waiting for syncResp
    *pfWaitForSyncResp_p = FALSE;

    if (pNodeInfo->nodeCfg & NMT_NODEASSIGN_PRES_CHAINING)
    {
        // Node is configured for PRC
        switch (nmtCommand_p)
        {
            case kNmtCmdStopNodeEx:
            case kNmtCmdResetNodeEx:
            case kNmtCmdResetCommunicationEx:
            case kNmtCmdResetConfigurationEx:
            case kNmtCmdSwResetEx:
            case kNmtCmdStopNode:
            case kNmtCmdResetNode:
            case kNmtCmdResetCommunication:
            case kNmtCmdResetConfiguration:
            case kNmtCmdSwReset:
                if (pNodeInfo->prcFlags & (NMTMNU_NODE_FLAG_PRC_ADD_SCHEDULED |
                                           NMTMNU_NODE_FLAG_PRC_ADD_IN_PROGRESS))
                {   // For this node, addition to the isochronous phase is scheduled
                    // or in progress. Skip adding this node
                    pNodeInfo->prcFlags &= ~(NMTMNU_NODE_FLAG_PRC_ADD_SCHEDULED |
                                             NMTMNU_NODE_FLAG_PRC_ADD_IN_PROGRESS);
                }

                if (pNodeInfo->flags & NMTMNU_NODE_FLAG_ISOCHRON)
                {   // PRes Chaining is enabled
                    tDllSyncRequest syncReqData;
                    size_t          size;

                    // Store NMT command for later execution
                    prcSetFlagsNmtCommandReset(pNodeInfo, nmtCommand_p);

                    // Disable PRes Chaining
                    syncReqData.nodeId = nodeId_p;
                    syncReqData.syncControl = PLK_SYNC_PRES_MODE_RESET |
                                              PLK_SYNC_DEST_MAC_ADDRESS_VALID;
                    size = sizeof(UINT) + sizeof(UINT32);

                    ret = syncu_requestSyncResponse(prcCbSyncResNextAction, &syncReqData, size);
                    switch (ret)
                    {
                        case kErrorOk:
                            // Mark node as removed from the isochronous phase
                            pNodeInfo->flags &= ~NMTMNU_NODE_FLAG_ISOCHRON;

                            // Send NMT command when SyncRes is received
                            *pfWaitForSyncResp_p = TRUE;
                            goto Exit;

                        case kErrorNmtSyncReqRejected:
                            // A SyncReq has already been posted for this node .
                            // Retry when SyncRes is received
                            ret = kErrorOk;

                            *pfWaitForSyncResp_p = TRUE;
                            goto Exit;

                        default:
                            goto Exit;
                    }
                }

                if (pNodeInfo->prcFlags &
                    (NMTMNU_NODE_FLAG_PRC_RESET_MASK | NMTMNU_NODE_FLAG_PRC_ADD_SYNCREQ_SENT))
                {   // A Node-reset NMT command was already scheduled or
                    // PRes Chaining is going to be enabled but the appropriate SyncRes
                    // has not been received, yet.

                    // Set the current NMT command if it has higher priority than a present one.
                    prcSetFlagsNmtCommandReset(pNodeInfo, nmtCommand_p);

                    // Wait for the SyncRes
                    *pfWaitForSyncResp_p = TRUE;
                    goto Exit;
                }
                break;

            default:
                // Other NMT commands
                break;
        }
    }

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Send NMT command to network

This function creates the NMT command frame and forwards it to DLL for
transmission.

\param[in]      nodeId_p            Node ID of target node
\param[in]      nmtCommand_p        NMT command
\param[in]      pNmtCommandData_p   Pointer to NMT command data
\param[in]      dataSize_p          Size of NMT command data

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError sendNmtCommand(UINT nodeId_p,
                                 tNmtCommand nmtCommand_p,
                                 const UINT8* pNmtCommandData_p,
                                 size_t dataSize_p)
{
    tOplkError  ret = kErrorOk;
    tFrameInfo  frameInfo;
    UINT8       aBuffer[C_DLL_MINSIZE_NMTCMDEXT];
    tPlkFrame*  pFrame;

    // build frame
    pFrame = (tPlkFrame*)aBuffer;
    OPLK_MEMSET(pFrame, 0x00, sizeof(aBuffer));
    ami_setUint8Le(&pFrame->dstNodeId, (UINT8)nodeId_p);
    ami_setUint8Le(&pFrame->data.asnd.serviceId, (UINT8)kDllAsndNmtCommand);
    ami_setUint8Le(&pFrame->data.asnd.payload.nmtCommandService.nmtCommandId, (UINT8)nmtCommand_p);

    if ((pNmtCommandData_p != NULL) && (dataSize_p > 0))
    {   // copy command data to frame
        OPLK_MEMCPY(&pFrame->data.asnd.payload.nmtCommandService.aNmtCommandData[0],
                    pNmtCommandData_p,
                    dataSize_p);
    }

    // build info structure
    frameInfo.frame.pBuffer = pFrame;
    frameInfo.frameSize = sizeof(aBuffer);

    // send NMT-Request
    ret = dllucal_sendAsyncFrame(&frameInfo, kDllAsyncReqPrioNmt);

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Calculates node numbers from NMT command frame

Three types of NMT commands are distinguished:
- Plain NMT commands sent via unicast node ID to a single node
- Plain NMT commands sent via broadcast node ID to all nodes
- Extended NMT commands, who are sent via broadcast node ID to all nodes,
  but are only accepted by those specified in the node list.

This function detect the command that was used, and returns all used node IDs:
Unicast Plain Cmds:     Only the single node ID
Broadcast Plain Cmds:   All node IDs
Extended Cmds:          All nodes that are in the node list
                        The subfunction nodeListToNodeId()
                        does the main work in this case

\param[in]      nodeId_p            Node ID that was addressed.
\param[in]      nmtCommand_p        Command that was sent.
\param[in]      pCmdData_p          The node list (extended commands only).
\param[in,out]  pOp_p               Structure to remember current state of operation.
                                    Calling function is responsible to zero before first
                                    call!(memset to zero before first call!)
\param[out]     pNodeId_p           Pointer to store node ID.

\return The function returns a tOplkError error code.
\retval kErrorOk                    Parsing nodes has finished.
\retval kErrorRetry                 Node ID found, call again to go on
*/
//------------------------------------------------------------------------------
static tOplkError getNodeIdFromCmd(UINT nodeId_p,
                                   tNmtCommand nmtCommand_p,
                                   const UINT8* pCmdData_p,
                                   tNmtMnuGetNodeId* pOp_p,
                                   UINT* pNodeId_p)
{
    tOplkError  ret = kErrorNmtUnknownCommand;

    if (nodeId_p != C_ADR_BROADCAST)
    {
        if (*pNodeId_p != nodeId_p)
        {
            *pNodeId_p = nodeId_p;
            ret = kErrorRetry;
        }
        else
            ret = kErrorOk;
    }
    else if ((nmtCommand_p >= NMT_PLAIN_COMMAND_START) &&
             (nmtCommand_p <= NMT_PLAIN_COMMAND_END))
    {
        // First valid CN node ID 1
        if (pOp_p->nodeId == C_ADR_INVALID)
            pOp_p->nodeId = 1;

        // Last valid CN node id is EPL_NMT_MAX_NODE_ID
        if (pOp_p->nodeId <= NMT_MAX_NODE_ID)
        {
            *pNodeId_p = pOp_p->nodeId;
            pOp_p->nodeId++;
            ret = kErrorRetry;
        }
        else
            ret = kErrorOk;
    }
    else if ((nmtCommand_p >= NMT_EXT_COMMAND_START) &&
             (nmtCommand_p <= NMT_EXT_COMMAND_END))
        ret = nodeListToNodeId(pCmdData_p, pOp_p, pNodeId_p);

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Calculates node numbers from a given bit field

Extended NMT commands use 'Node Lists'. These are 32 byte wide bit fields, where
each bit corresponds to a node number. This function walks through the bit
field, and returns for every found node ID.

For a detailed description of the bit field format, see section 7.3.1.2.3
'POWERLINK Node List Format' of the Ethernet POWERLINK specification
DS 301 V1.2.0

\param[in]      pCmdData_p          The node list.
\param[in,out]  pOp_p               Structure to remember current state of operation.
                                    Calling function is responsible to zero before first
                                    call!
\param[out]     pNodeId_p           Pointer to store found node ID.

\return The function returns a tOplkError error code.
\retval kErrorOk                    Parsing node list has finished.
\retval kErrorRetry                 Node ID found, call again to go on
*/
//------------------------------------------------------------------------------
static tOplkError nodeListToNodeId(const UINT8* pCmdData_p,
                                   tNmtMnuGetNodeId* pOp_p,
                                   UINT* pNodeId_p)
{
    tOplkError  ret = kErrorOk;
    BOOL        matchFound = FALSE;

    *pNodeId_p = C_ADR_INVALID;

    // Loop over bit array, handle only nodes whose bits are set
    while ((pOp_p->cmdDataId < 32) && (matchFound == FALSE))
    {
        pOp_p->cmdData = ami_getUint8Le(&pCmdData_p[pOp_p->cmdDataId]);

        if (pOp_p->nodeIdCnt == 0)
            pOp_p->nodeIdMask = 0x01;

        while ((pOp_p->nodeIdCnt < 8) && (matchFound == FALSE))
        {
            if ((pOp_p->cmdData & pOp_p->nodeIdMask) != 0)
            {   // Match
                *pNodeId_p = pOp_p->nodeId;
                matchFound = TRUE;
            }

            pOp_p->nodeId++;
            pOp_p->nodeIdCnt++;
            pOp_p->nodeIdMask <<= 1;
        }

        if (pOp_p->nodeIdCnt == 8)
        {
            pOp_p->nodeIdCnt = 0;
            pOp_p->cmdDataId++;
        }
    }

    if (matchFound != FALSE)
        ret = kErrorRetry;

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Removes the node from the extended NMT command

This function removes the specified node from the extended NMT command.

\param[in]      nodeId_p            Node ID to be removed from extended NMT command
\param[in]      pCmdData_p          Extended NMT command node list
\param[in]      size_p              Size of extended NMT command node list

\return The function returns a tOplkError error code.
\retval kErrorOk                    Node removed from node list
\retval kErrorNmtInvalidParam       Node list size too short
*/
//------------------------------------------------------------------------------
static tOplkError removeNodeIdFromExtCmd(UINT nodeId_p,
                                         const UINT8* pCmdData_p,
                                         size_t size_p)
{
    size_t  byteOffset;
    UINT8   bitMask;
    UINT8   cmdByte;

    // Byte offset --> nodeid divide by 8
    // Bit offset  --> 2 ^ (nodeid AND 0b111)
    byteOffset = (size_t)(nodeId_p >> 3);
    bitMask = 1 << ((UINT8)nodeId_p & 7);

    if (byteOffset < size_p)
    {
        cmdByte = ami_getUint8Le(&pCmdData_p[byteOffset]);
        cmdByte &= ~bitMask;    // Inactivate bit

        ami_setUint8Le(&pCmdData_p[byteOffset], cmdByte);
        return kErrorOk;
    }

    return kErrorNmtInvalidParam;
}

//------------------------------------------------------------------------------
/**
\brief  Compute ceiling of integer division

This function computes the division of the given \p numerator_p and
\p denominator_p.

\param[in]      numerator_p         Division numerator
\param[in]      denominator_p       Division denominator

\return The function returns the ceiling of the integer division.
*/
//------------------------------------------------------------------------------
static ULONG computeCeilDiv(ULONG numerator_p, ULONG denominator_p)
{
    ULONG result;

    result = (numerator_p % denominator_p) ?
                (numerator_p / denominator_p + 1) : (numerator_p / denominator_p);

    return result;
}

//------------------------------------------------------------------------------
/**
\brief  Correct a NMT state variable

The function checks if \p nmtState_p is a valid tNmtState. If it is a global
state it is unmodified. If it is a node specific state, we add the CN flag and
if it contains garbage, it is set to kNmtStateInvalid.

\param[in]      nmtState_p          The NMT state to be corrected

\return Returns the corrected NMT state value.
*/
//------------------------------------------------------------------------------
static tNmtState correctNmtState(UINT8 nmtState_p)
{
    tNmtState   correctedNmtState;

    switch (nmtState_p)
    {
        // Check if it's a valid node specific NMT state and if it is, set the CN flag
        case NMT_STATE_XX_PRE_OPERATIONAL_1:
        case NMT_STATE_XX_PRE_OPERATIONAL_2:
        case NMT_STATE_XX_READY_TO_OPERATE:
        case NMT_STATE_XX_OPERATIONAL:
        case NMT_STATE_XX_NOT_ACTIVE:
        case NMT_STATE_XX_BASIC_ETHERNET:
            correctedNmtState = (tNmtState)(nmtState_p | NMT_TYPE_CS);
            break;

        // if it is a global state don't modify it
        case NMT_STATE_GS_INITIALISING:
        case NMT_STATE_GS_RESET_APPLICATION:
        case NMT_STATE_GS_RESET_COMMUNICATION:
        case NMT_STATE_GS_RESET_CONFIGURATION:
            correctedNmtState = (tNmtState)nmtState_p;
            break;

        default:
            // NMT state contains garbage, set it to a defined value
            correctedNmtState = kNmtStateInvalid;
            break;
    }

    return correctedNmtState;
}

//------------------------------------------------------------------------------
/**
\brief  Handle missing PRC support

The function handles a node which is configured to be used with PRC but misses
to support PRC. The node configuration is changed to asynchronous only what
avoids that the NMTMNU continuously resets the node. This enables the application
to e.g. update the node with a firmware revision that supports PRC.

\param[in]      nodeId_p            Node ID of an identified node
\param[in]      featureFlags_p      The node's feature flags
*/
//------------------------------------------------------------------------------
static void handleMissingPrcSupport(UINT nodeId_p, UINT32 featureFlags_p)
{
    tNmtMnuNodeInfo* pNodeInfo = NMTMNU_GET_NODEINFO(nodeId_p);

    if (pNodeInfo->fPrcSupportIsMissing)
    {
        if ((featureFlags_p & NMT_FEATUREFLAGS_PRC) != 0)
        {
            pNodeInfo->nodeCfg = pNodeInfo->nodeCfgBackup;
            pNodeInfo->nodeCfgBackup = 0;
            pNodeInfo->fPrcSupportIsMissing = FALSE;
        }
    }
    else
    {
        if (((featureFlags_p & NMT_FEATUREFLAGS_PRC) == 0) &&
            ((pNodeInfo->nodeCfg & NMT_NODEASSIGN_PRES_CHAINING) != 0))
        {
            pNodeInfo->fPrcSupportIsMissing = TRUE;
            pNodeInfo->nodeCfgBackup = pNodeInfo->nodeCfg;

            pNodeInfo->nodeCfg &= ~NMT_NODEASSIGN_PRES_CHAINING;
            pNodeInfo->nodeCfg |= NMT_NODEASSIGN_ASYNCONLY_NODE;
        }
    }
}

/// \}

#endif
