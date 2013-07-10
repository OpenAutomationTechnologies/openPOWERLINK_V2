/****************************************************************************

  (c) SYSTEC electronic GmbH, D-07973 Greiz, August-Bebel-Str. 29
      www.systec-electronic.com

  Project:      openPOWERLINK

  Description:  source file for NMT-MN-Module

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

  2006/06/09 k.t.:   start of the implementation

****************************************************************************/

#include "user/nmtmnu.h"
#include "user/EplTimeru.h"
#include "user/identu.h"
#include "user/statusu.h"
#include "user/EplObdu.h"
#include "user/dllucal.h"
#include "Benchmark.h"

#if EPL_NMTMNU_PRES_CHAINING_MN != FALSE
#include "user/EplSyncu.h"
#endif

#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)

#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_OBDU)) == 0) && (EPL_OBD_USE_KERNEL == FALSE)
#error "EPL NmtMnu module needs EPL module OBDU or OBDK!"
#endif

//=========================================================================//
//                                                                         //
//          P R I V A T E   D E F I N I T I O N S                          //
//                                                                         //
//=========================================================================//

//---------------------------------------------------------------------------
// const defines
//---------------------------------------------------------------------------

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
#define EPL_NMTMNU_DBG_POST_TRACE_VALUE(Event_p, uiNodeId_p, wErrorCode_p) \
    TGT_DBG_POST_TRACE_VALUE((kEplEventSinkNmtMnu << 28) | (Event_p << 24) \
                             | (uiNodeId_p << 16) | wErrorCode_p)

// defines for flags in node info structure
#define EPL_NMTMNU_NODE_FLAG_ISOCHRON       0x0001  // CN is being accessed isochronously
#define EPL_NMTMNU_NODE_FLAG_NOT_SCANNED    0x0002  // CN was not scanned once -> decrement SignalCounter and reset flag
#define EPL_NMTMNU_NODE_FLAG_HALTED         0x0004  // boot process for this CN is halted
#define EPL_NMTMNU_NODE_FLAG_NMT_CMD_ISSUED 0x0008  // NMT command was just issued, wrong NMT states will be tolerated
#define EPL_NMTMNU_NODE_FLAG_PREOP2_REACHED 0x0010  // NodeAddIsochronous has been called, waiting for ISOCHRON
#define EPL_NMTMNU_NODE_FLAG_COUNT_STATREQ  0x0300  // counter for StatusRequest timer handle
#define EPL_NMTMNU_NODE_FLAG_COUNT_LONGER   0x0C00  // counter for longer timeouts timer handle
#define EPL_NMTMNU_NODE_FLAG_INC_STATREQ    0x0100  // increment for StatusRequest timer handle
#define EPL_NMTMNU_NODE_FLAG_INC_LONGER     0x0400  // increment for longer timeouts timer handle
                    // These counters will be incremented at every timer start
                    // and copied to timerarg. When the timer event occures
                    // both will be compared and if unequal the timer event
                    // will be discarded, because it is an old one.
#if EPL_NMTMNU_PRES_CHAINING_MN != FALSE
#define EPL_NMTMNU_NODE_FLAG_PRC_ADD_SCHEDULED      0x0001
#define EPL_NMTMNU_NODE_FLAG_PRC_ADD_IN_PROGRESS    0x0002
#define EPL_NMTMNU_NODE_FLAG_PRC_ADD_SYNCREQ_SENT   0x0004  // Covers time between SyncReq and SyncRes for addition
#define EPL_NMTMNU_NODE_FLAG_PRC_SHIFT_REQUIRED     0x0010
#define EPL_NMTMNU_NODE_FLAG_PRC_SYNC_ERR           0x0020  // SyncRes 1-bit error counter

#define EPL_NMTMNU_NODE_FLAG_PRC_CALL_MEASURE       0x0100
#define EPL_NMTMNU_NODE_FLAG_PRC_CALL_SHIFT         0x0200
#define EPL_NMTMNU_NODE_FLAG_PRC_CALL_ADD           0x0300
#define EPL_NMTMNU_NODE_FLAG_PRC_CALL_MASK          0x0300

#define EPL_NMTMNU_NODE_FLAG_PRC_VERIFY             0x0400

#define EPL_NMTMNU_NODE_FLAG_PRC_STOP_NODE          0x1000
#define EPL_NMTMNU_NODE_FLAG_PRC_RESET_NODE         0x2000
#define EPL_NMTMNU_NODE_FLAG_PRC_RESET_COM          0x3000
#define EPL_NMTMNU_NODE_FLAG_PRC_RESET_CONF         0x4000
#define EPL_NMTMNU_NODE_FLAG_PRC_RESET_SW           0x5000
#define EPL_NMTMNU_NODE_FLAG_PRC_RESET_MASK         0x7000
#endif

// defines for timer arguments to draw a distinction between several events
#define EPL_NMTMNU_TIMERARG_NODE_MASK   0x000000FFL // mask that contains the node-ID
#define EPL_NMTMNU_TIMERARG_IDENTREQ    0x00010000L // timer event is for IdentRequest
#define EPL_NMTMNU_TIMERARG_STATREQ     0x00020000L // timer event is for StatusRequest
#define EPL_NMTMNU_TIMERARG_LONGER      0x00040000L // timer event is for longer timeouts
#define EPL_NMTMNU_TIMERARG_STATE_MON   0x00080000L // timer event for StatusRequest to monitor execution of NMT state changes
#define EPL_NMTMNU_TIMERARG_COUNT_SR    0x00000300L // counter for StatusRequest
#define EPL_NMTMNU_TIMERARG_COUNT_LO    0x00000C00L // counter for longer timeouts
                    // The counters must have the same position as in the node flags above.

#define EPL_NMTMNU_SET_FLAGS_TIMERARG_STATREQ(pNodeInfo_p, nodeId_p, TimerArg_p)                  \
    pNodeInfo_p->flags       =   ((pNodeInfo_p->flags + EPL_NMTMNU_NODE_FLAG_INC_STATREQ) &   \
                                    EPL_NMTMNU_NODE_FLAG_COUNT_STATREQ)                         |   \
                                    (pNodeInfo_p->flags & ~EPL_NMTMNU_NODE_FLAG_COUNT_STATREQ);  \
    TimerArg_p.m_Arg.m_dwVal    =   EPL_NMTMNU_TIMERARG_STATREQ                                 |   \
                                    nodeId_p                                                  |   \
                                    (pNodeInfo_p->flags & EPL_NMTMNU_NODE_FLAG_COUNT_STATREQ);   \
    TimerArg_p.m_EventSink      =   kEplEventSinkNmtMnu;

#define EPL_NMTMNU_SET_FLAGS_TIMERARG_IDENTREQ(pNodeInfo_p, nodeId_p, TimerArg_p)                 \
    pNodeInfo_p->flags       =   ((pNodeInfo_p->flags + EPL_NMTMNU_NODE_FLAG_INC_STATREQ) &   \
                                    EPL_NMTMNU_NODE_FLAG_COUNT_STATREQ)                         |   \
                                    (pNodeInfo_p->flags & ~EPL_NMTMNU_NODE_FLAG_COUNT_STATREQ);  \
    TimerArg_p.m_Arg.m_dwVal    =   EPL_NMTMNU_TIMERARG_IDENTREQ                                |   \
                                    nodeId_p                                                  |   \
                                    (pNodeInfo_p->flags & EPL_NMTMNU_NODE_FLAG_COUNT_STATREQ);   \
    TimerArg_p.m_EventSink      =   kEplEventSinkNmtMnu;

#define EPL_NMTMNU_SET_FLAGS_TIMERARG_LONGER(pNodeInfo_p, nodeId_p, TimerArg_p)                   \
    pNodeInfo_p->flags       =   ((pNodeInfo_p->flags + EPL_NMTMNU_NODE_FLAG_INC_LONGER)  &   \
                                    EPL_NMTMNU_NODE_FLAG_COUNT_LONGER)                          |   \
                                    (pNodeInfo_p->flags & ~EPL_NMTMNU_NODE_FLAG_COUNT_LONGER);   \
    TimerArg_p.m_Arg.m_dwVal    =   EPL_NMTMNU_TIMERARG_LONGER                                  |   \
                                    nodeId_p                                                  |   \
                                    (pNodeInfo_p->flags & EPL_NMTMNU_NODE_FLAG_COUNT_LONGER);    \
    TimerArg_p.m_EventSink      =   kEplEventSinkNmtMnu;

#define EPL_NMTMNU_SET_FLAGS_TIMERARG_STATE_MON(pNodeInfo_p, nodeId_p, TimerArg_p)                \
    pNodeInfo_p->flags       =   ((pNodeInfo_p->flags + EPL_NMTMNU_NODE_FLAG_INC_STATREQ) &   \
                                    EPL_NMTMNU_NODE_FLAG_COUNT_STATREQ)                         |   \
                                    (pNodeInfo_p->flags & ~EPL_NMTMNU_NODE_FLAG_COUNT_STATREQ);  \
    TimerArg_p.m_Arg.m_dwVal    =   EPL_NMTMNU_TIMERARG_STATE_MON                               |   \
                                    nodeId_p                                                  |   \
                                    (pNodeInfo_p->flags & EPL_NMTMNU_NODE_FLAG_COUNT_STATREQ);   \
    TimerArg_p.m_EventSink      =   kEplEventSinkNmtMnu;

// defines for global flags
#define EPL_NMTMNU_FLAG_HALTED              0x0001  // boot process is halted
#define EPL_NMTMNU_FLAG_APP_INFORMED        0x0002  // application was informed about possible NMT state change
#define EPL_NMTMNU_FLAG_USER_RESET          0x0004  // NMT reset issued by user / diagnostic node
#if EPL_NMTMNU_PRES_CHAINING_MN != FALSE
#define EPL_NMTMNU_FLAG_PRC_ADD_SCHEDULED   0x0008  // at least one node is scheduled
                                                    // for addition to isochronous phase
#define EPL_NMTMNU_FLAG_PRC_ADD_IN_PROGRESS 0x0010  // add-PRC-node process is in progress
#endif

// return pointer to node info structure for specified node ID
// d.k. may be replaced by special (hash) function if node ID array is smaller than 254
#define EPL_NMTMNU_GET_NODEINFO(nodeId_p) (&nmtMnuInstance_g.aNodeInfo[nodeId_p - 1])

//---------------------------------------------------------------------------
// local types
//---------------------------------------------------------------------------

typedef struct
{
    UINT                nodeId;
    tEplNmtNodeCommand  nodeCommand;

} tNmtMnuNodeCmd;

typedef enum
{
    kNmtMnuIntNodeEventNoIdentResponse   = 0x00,
    kNmtMnuIntNodeEventIdentResponse     = 0x01,
    kNmtMnuIntNodeEventBoot              = 0x02,
    kNmtMnuIntNodeEventExecResetConf     = 0x03,
    kNmtMnuIntNodeEventExecResetNode     = 0x04,
    kNmtMnuIntNodeEventConfigured        = 0x05,
    kNmtMnuIntNodeEventNoStatusResponse  = 0x06,
    kNmtMnuIntNodeEventStatusResponse    = 0x07,
    kNmtMnuIntNodeEventHeartbeat         = 0x08,
    kNmtMnuIntNodeEventNmtCmdSent        = 0x09,
    kNmtMnuIntNodeEventTimerIdentReq     = 0x0A,
    kNmtMnuIntNodeEventTimerStatReq      = 0x0B,
    kNmtMnuIntNodeEventTimerStateMon     = 0x0C,
    kNmtMnuIntNodeEventTimerLonger       = 0x0D,
    kNmtMnuIntNodeEventError             = 0x0E,

} tNmtMnuIntNodeEvent;


typedef enum
{
    kNmtMnuNodeStateUnknown      = 0x00,
    kNmtMnuNodeStateIdentified   = 0x01,
    kNmtMnuNodeStateConfRestored = 0x02, // CN ResetNode after restore configuration
    kNmtMnuNodeStateResetConf    = 0x03, // CN ResetConf after configuration update
    kNmtMnuNodeStateConfigured   = 0x04, // BootStep1 completed
    kNmtMnuNodeStateReadyToOp    = 0x05, // BootStep2 completed
    kNmtMnuNodeStateComChecked   = 0x06, // Communication checked successfully
    kNmtMnuNodeStateOperational  = 0x07, // CN is in NMT state OPERATIONAL

} tNmtMnuNodeState;


typedef struct
{
    tEplTimerHdl        timerHdlStatReq;    // timer to delay StatusRequests and IdentRequests
    tEplTimerHdl        timerHdlLonger;     // 2nd timer for NMT command EnableReadyToOp and CheckCommunication
    tNmtMnuNodeState    nodeState;          // internal node state (kind of sub state of NMT state)
    UINT32              nodeCfg;            // subindex from 0x1F81
    UINT16              flags;              // node flags (see defines above)
#if EPL_NMTMNU_PRES_CHAINING_MN != FALSE
    UINT16              prcFlags;           // PRC specific node flags
    UINT32              relPropagationDelayNs;
    UINT32              pResTimeFirstNs;
#endif
} tNmtMnuNodeInfo;


typedef struct
{
    tNmtMnuNodeInfo     aNodeInfo[EPL_NMT_MAX_NODE_ID];
    tEplTimerHdl        timerHdlNmtState;       // timeout for stay in NMT state
    UINT                mandatorySlaveCount;
    UINT                signalSlaveCount;
    ULONG               statusRequestDelay;     // in [ms] (object 0x1006 * EPL_C_NMT_STATREQ_CYCLE)
    ULONG               timeoutReadyToOp;       // in [ms] (object 0x1F89/4)
    ULONG               timeoutCheckCom;        // in [ms] (object 0x1006 * MultiplexedCycleCount)
    UINT16              flags;                  // global flags
    UINT32              nmtStartup;             // object 0x1F80 NMT_StartUp_U32
    tNmtMnuCbNodeEvent  pfnCbNodeEvent;
    tNmtMnuCbBootEvent  pfnCbBootEvent;
#if EPL_NMTMNU_PRES_CHAINING_MN != FALSE
    UINT32              prcPResMnTimeoutNs;
    UINT32              prcPResTimeFirstCorrectionNs;
    UINT32              prcPResTimeFirstNegOffsetNs;
#endif
} tNmtMnuInstance;


//---------------------------------------------------------------------------
// local vars
//---------------------------------------------------------------------------

static tNmtMnuInstance   nmtMnuInstance_g;


//---------------------------------------------------------------------------
// local function prototypes
//---------------------------------------------------------------------------

static tEplKernel EplNmtMnuCbNmtRequest(tEplFrameInfo * pFrameInfo_p);

static tEplKernel EplNmtMnuCbIdentResponse(
                                    UINT                nodeId_p,
                                    tEplIdentResponse*  pIdentResponse_p);

static tEplKernel EplNmtMnuCbStatusResponse(
                                    UINT                nodeId_p,
                                    tEplStatusResponse* pStatusResponse_p);

static tEplKernel EplNmtMnuCbNodeAdded(UINT nodeId_p);

static tEplKernel EplNmtMnuCheckNmtState(
                                    UINT                nodeId_p,
                                    tNmtMnuNodeInfo*    pNodeInfo_p,
                                    tEplNmtState        nodeNmtState_p,
                                    UINT16              errorCode_p,
                                    tEplNmtState        localNmtState_p);

static tEplKernel EplNmtMnuAddNodeIsochronous(UINT nodeId_p);

static tEplKernel EplNmtMnuStartBootStep1(BOOL fNmtResetAllIssued_p);

static tEplKernel EplNmtMnuStartBootStep2(void);

static tEplKernel EplNmtMnuStartCheckCom(void);

static tEplKernel EplNmtMnuNodeBootStep2(UINT nodeId_p, tNmtMnuNodeInfo* pNodeInfo_p);

static tEplKernel EplNmtMnuNodeCheckCom(UINT nodeId_p, tNmtMnuNodeInfo* pNodeInfo_p);

static tEplKernel EplNmtMnuStartNodes(void);

static tEplKernel EplNmtMnuProcessInternalEvent(
                                    UINT                nodeId_p,
                                    tEplNmtState        nodeNmtState_p,
                                    UINT16              errorCode_p,
                                    tNmtMnuIntNodeEvent nodeEvent_p);

static tEplKernel EplNmtMnuReset(void);

#if EPL_NMTMNU_PRES_CHAINING_MN != FALSE
static tEplKernel EplNmtMnuPrcMeasure(void);
static tEplKernel EplNmtMnuPrcCalculate(UINT nodeIdFirstNode_p);
static tEplKernel EplNmtMnuPrcShift(UINT nodeIdPrevShift_p);
static tEplKernel EplNmtMnuPrcAdd(UINT nodeIdPrevAdd_p);
static tEplKernel EplNmtMnuPrcVerify(UINT nodeId_p);

static tEplKernel EplNmtMnuPrcCbSyncResMeasure(UINT, tEplSyncResponse*);
static tEplKernel EplNmtMnuPrcCbSyncResShift(UINT, tEplSyncResponse*);
static tEplKernel EplNmtMnuPrcCbSyncResAdd(UINT, tEplSyncResponse*);
static tEplKernel EplNmtMnuPrcCbSyncResVerify(UINT, tEplSyncResponse*);
static tEplKernel EplNmtMnuPrcCbSyncResNextAction(UINT, tEplSyncResponse*);

static tEplKernel EplNmtMnuPrcCalcPResResponseTimeNs(
                                    UINT            nodeId_p,
                                    UINT            nodeIdPredNode_p,
                                    UINT32*         pPResResponseTimeNs_p);
static tEplKernel EplNmtMnuPrcCalcPResChainingSlotTimeNs(
                                    UINT            nodeIdLastNode_p,
                                    UINT32*         pPResChainingSlotTimeNs_p);

static tEplKernel EplNmtMnuPrcFindPredecessorNode(UINT nodeId_p);
static void       EplNmtMnuPrcSyncError(tNmtMnuNodeInfo* pNodeInfo_p);
static void       EplNmtMnuPrcSetFlagsNmtCommandReset(
                                        tNmtMnuNodeInfo* pNodeInfo_p,
                                        tEplNmtCommand      nmtCommand_p);
#endif


//=========================================================================//
//                                                                         //
//          P U B L I C   F U N C T I O N S                                //
//                                                                         //
//=========================================================================//

//---------------------------------------------------------------------------
//
// Function:    EplNmtMnuInit
//
// Description: init first instance of the module
//
//
//
// Parameters:
//
//
// Returns:     tEplKernel  = errorcode
//
//
// State:
//
//---------------------------------------------------------------------------

tEplKernel nmtmnu_init(tNmtMnuCbNodeEvent pfnCbNodeEvent_p, tNmtMnuCbBootEvent pfnCbBootEvent_p)
{
tEplKernel ret;

    ret = nmtmnu_addInstance(pfnCbNodeEvent_p, pfnCbBootEvent_p);

    return ret;
}


//---------------------------------------------------------------------------
//
// Function:    EplNmtMnuAddInstance
//
// Description: init other instances of the module
//
//
//
// Parameters:
//
//
// Returns:     tEplKernel  = errorcode
//
//
// State:
//
//---------------------------------------------------------------------------

tEplKernel nmtmnu_addInstance(tNmtMnuCbNodeEvent pfnCbNodeEvent_p,
                                tNmtMnuCbBootEvent pfnCbBootEvent_p)
{
tEplKernel ret;

    ret = kEplSuccessful;

    // reset instance structure
    EPL_MEMSET(&nmtMnuInstance_g, 0, sizeof (nmtMnuInstance_g));

    if ((pfnCbNodeEvent_p == NULL) || (pfnCbBootEvent_p == NULL))
    {
        ret = kEplNmtInvalidParam;
        goto Exit;
    }
    nmtMnuInstance_g.pfnCbNodeEvent = pfnCbNodeEvent_p;
    nmtMnuInstance_g.pfnCbBootEvent = pfnCbBootEvent_p;

    // initialize StatusRequest delay
    nmtMnuInstance_g.statusRequestDelay = 5000L;

    // register NmtMnResponse callback function
    ret = dllucal_regAsndService(kEplDllAsndNmtRequest, EplNmtMnuCbNmtRequest, kEplDllAsndFilterLocal);

#if EPL_NMTMNU_PRES_CHAINING_MN != FALSE
    nmtMnuInstance_g.prcPResTimeFirstCorrectionNs =  50;
    nmtMnuInstance_g.prcPResTimeFirstNegOffsetNs  = 500;
#endif

Exit:
    return ret;

}


//---------------------------------------------------------------------------
//
// Function:    EplNmtMnuDelInstance
//
// Description: delete instance
//
//
//
// Parameters:
//
//
// Returns:     tEplKernel  = errorcode
//
//
// State:
//
//---------------------------------------------------------------------------

tEplKernel nmtmnu_delInstance(void)
{
tEplKernel  ret;

    ret = kEplSuccessful;

    // deregister NmtMnResponse callback function
    ret = dllucal_regAsndService(kEplDllAsndNmtRequest, NULL, kEplDllAsndFilterNone);

    ret = EplNmtMnuReset();

    return ret;

}


//---------------------------------------------------------------------------
//
// Function:    EplNmtMnuSendNmtCommandEx
//
// Description: sends the specified NMT command to the specified node.
//
// Parameters:  nodeId_p              = node ID to which the NMT command will be sent
//              NmtCommand_p            = NMT command
//
// Returns:     tEplKernel              = error code
//
// State:
//
//---------------------------------------------------------------------------

tEplKernel nmtmnu_sendNmtCommandEx(UINT nodeId_p, tEplNmtCommand nmtCommand_p,
                                   void* pNmtCommandData_p, UINT uiDataSize_p)
{
tEplKernel          ret;
tEplFrameInfo       frameInfo;
UINT8               aBuffer[EPL_C_DLL_MINSIZE_NMTCMDEXT];
tEplFrame*          pFrame;
tEplDllNodeOpParam  nodeOpParam;
#if EPL_NMTMNU_PRES_CHAINING_MN != FALSE
tNmtMnuNodeInfo*    pNodeInfo;
#endif

    ret = kEplSuccessful;

    if ((nodeId_p == 0) || (nodeId_p > EPL_C_ADR_BROADCAST))
    {   // invalid node ID specified
        ret = kEplInvalidNodeId;
        goto Exit;
    }

    if ((pNmtCommandData_p != NULL) && (uiDataSize_p > (EPL_C_DLL_MINSIZE_NMTCMDEXT - EPL_C_DLL_MINSIZE_NMTCMD)))
    {
        ret = kEplNmtInvalidParam;
        goto Exit;
    }

    // $$$ d.k. may be check in future versions if the caller wants to perform prohibited state transitions
    //     the CN should not perform these transitions, but the expected NMT state will be changed and never fullfilled.

#if EPL_NMTMNU_PRES_CHAINING_MN != FALSE
    pNodeInfo = EPL_NMTMNU_GET_NODEINFO(nodeId_p);

    if (pNodeInfo->nodeCfg & EPL_NODEASSIGN_PRES_CHAINING)
    {   // Node is a PRes Chaining node
        switch (nmtCommand_p)
        {
            case kEplNmtCmdStopNode:
            case kEplNmtCmdResetNode:
            case kEplNmtCmdResetCommunication:
            case kEplNmtCmdResetConfiguration:
            case kEplNmtCmdSwReset:
            {
                if (pNodeInfo->prcFlags & (EPL_NMTMNU_NODE_FLAG_PRC_ADD_SCHEDULED |
                                              EPL_NMTMNU_NODE_FLAG_PRC_ADD_IN_PROGRESS))
                {   // For this node, addition to the isochronous phase is scheduled
                    // or in progress
                    // Skip addition for this node
                    pNodeInfo->prcFlags &= ~(EPL_NMTMNU_NODE_FLAG_PRC_ADD_SCHEDULED |
                                                EPL_NMTMNU_NODE_FLAG_PRC_ADD_IN_PROGRESS);
                }

                if (pNodeInfo->flags & EPL_NMTMNU_NODE_FLAG_ISOCHRON)
                {   // PRes Chaining is enabled
                tEplDllSyncRequest SyncReqData;
                UINT               uiSize;

                    // Store NMT command for later execution
                    EplNmtMnuPrcSetFlagsNmtCommandReset(pNodeInfo, nmtCommand_p);

                    // Disable PRes Chaining
                    SyncReqData.m_uiNodeId      = nodeId_p;
                    SyncReqData.m_dwSyncControl = EPL_SYNC_PRES_MODE_RESET |
                                                  EPL_SYNC_DEST_MAC_ADDRESS_VALID;
                    uiSize = sizeof(UINT) + sizeof(UINT32);

                    ret = EplSyncuRequestSyncResponse(EplNmtMnuPrcCbSyncResNextAction, &SyncReqData, uiSize);
                    switch (ret)
                    {
                        case kEplSuccessful:
                        {
                            // Mark node as removed from the isochronous phase
                            pNodeInfo->flags &= ~EPL_NMTMNU_NODE_FLAG_ISOCHRON;
                            // Send NMT command when SyncRes is received
                            goto Exit;
                        }

                        case kEplNmtSyncReqRejected:
                        {   // There has already been posted a SyncReq for this node.
                            // Retry when SyncRes is received
                            ret = kEplSuccessful;
                            goto Exit;
                        }

                        default:
                        {
                            goto Exit;
                        }
                    }
                }

                if (pNodeInfo->prcFlags & (EPL_NMTMNU_NODE_FLAG_PRC_RESET_MASK |
                                              EPL_NMTMNU_NODE_FLAG_PRC_ADD_SYNCREQ_SENT))
                {   // A Node-reset NMT command was already scheduled or
                    // PRes Chaining is going to be enabled but the appropriate SyncRes
                    // has not been received, yet.

                    // Set the current NMT command if it has higher priority than a present one.
                    EplNmtMnuPrcSetFlagsNmtCommandReset(pNodeInfo, nmtCommand_p);

                    // Wait for the SyncRes
                    goto Exit;
                }

                break;
            }
            default:
            {   // Other NMT commands
                break;
            }
        }
    }
#endif

    // build frame
    pFrame = (tEplFrame*) aBuffer;
    EPL_MEMSET(pFrame, 0x00, sizeof(aBuffer));
    AmiSetByteToLe(&pFrame->m_le_bDstNodeId, (UINT8) nodeId_p);
    AmiSetByteToLe(&pFrame->m_Data.m_Asnd.m_le_bServiceId, (UINT8) kEplDllAsndNmtCommand);
    AmiSetByteToLe(&pFrame->m_Data.m_Asnd.m_Payload.m_NmtCommandService.m_le_bNmtCommandId,
        (UINT8)nmtCommand_p);
    if ((pNmtCommandData_p != NULL) && (uiDataSize_p > 0))
    {   // copy command data to frame
        EPL_MEMCPY(&pFrame->m_Data.m_Asnd.m_Payload.m_NmtCommandService.m_le_abNmtCommandData[0], pNmtCommandData_p, uiDataSize_p);
    }

    // build info structure
    frameInfo.m_pFrame = pFrame;
    frameInfo.m_uiFrameSize = sizeof(aBuffer);

    // send NMT-Request
#if(((EPL_MODULE_INTEGRATION) & (EPL_MODULE_DLLU)) != 0)
    ret = dllucal_sendAsyncFrame(&frameInfo, kEplDllAsyncReqPrioNmt);
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
        case kEplNmtCmdStartNode:
        case kEplNmtCmdEnterPreOperational2:
        case kEplNmtCmdEnableReadyToOperate:
        {
            // nothing left to do,
            // because any further processing is done
            // when the NMT command is actually sent
            goto Exit;
        }

        case kEplNmtCmdStopNode:
        {
            // remove CN from isochronous phase softly
            nodeOpParam.m_OpNodeType = kEplDllNodeOpTypeSoftDelete;
            break;
        }

        case kEplNmtCmdResetNode:
        case kEplNmtCmdResetCommunication:
        case kEplNmtCmdResetConfiguration:
        case kEplNmtCmdSwReset:
        {
            // remove CN immediately from isochronous phase
            nodeOpParam.m_OpNodeType = kEplDllNodeOpTypeIsochronous;
            break;
        }

        default:
        {
            goto Exit;
        }
    }

    // The expected node state will be updated when the NMT command
    // was actually sent.
    // See functions EplNmtMnuProcessInternalEvent(kNmtMnuIntNodeEventNmtCmdSent),
    // EplNmtMnuProcessEvent(kEplEventTypeNmtMnuNmtCmdSent).

    // remove CN from isochronous phase;
    // This must be done here and not when NMT command is actually sent
    // because it will be too late and may cause unwanted errors
    if (nodeId_p != EPL_C_ADR_BROADCAST)
    {
        nodeOpParam.m_uiNodeId = nodeId_p;
        ret = dllucal_deleteNode(&nodeOpParam);
    }
    else
    {   // do it for all active CNs
        for (nodeId_p = 1; nodeId_p <= tabentries(nmtMnuInstance_g.aNodeInfo); nodeId_p++)
        {
            if ((EPL_NMTMNU_GET_NODEINFO(nodeId_p)->nodeCfg & (EPL_NODEASSIGN_NODE_IS_CN | EPL_NODEASSIGN_NODE_EXISTS)) != 0)
            {
                nodeOpParam.m_uiNodeId = nodeId_p;
                ret = dllucal_deleteNode(&nodeOpParam);
            }
        }
    }

Exit:
    return ret;
}

//---------------------------------------------------------------------------
//
// Function:    EplNmtMnuSendNmtCommand
//
// Description: sends the specified NMT command to the specified node.
//
// Parameters:  nodeId_p              = node ID to which the NMT command will be sent
//              NmtCommand_p            = NMT command
//
// Returns:     tEplKernel              = error code
//
// State:
//
//---------------------------------------------------------------------------

tEplKernel nmtmnu_sendNmtCommand(UINT nodeId_p, tEplNmtCommand  nmtCommand_p)
{
tEplKernel      ret = kEplSuccessful;

    ret = nmtmnu_sendNmtCommandEx(nodeId_p, nmtCommand_p, NULL, 0);

//Exit:
    return ret;
}

//---------------------------------------------------------------------------
//
// Function:    EplNmtMnuRequestNmtCommand
//
// Description: requests the specified NMT command for the specified node.
//              It may also be applied to the local node.
//
// Parameters:  nodeId_p              = node ID to which the NMT command will be sent
//              NmtCommand_p            = NMT command
//
// Returns:     tEplKernel              = error code
//
// State:
//
//---------------------------------------------------------------------------

tEplKernel nmtmnu_requestNmtCommand(UINT nodeId_p,
                                    tEplNmtCommand  nmtCommand_p)
{
tEplKernel      ret = kEplSuccessful;
tEplNmtState    nmtState;

    nmtState = EplNmtuGetNmtState();

    if (nmtState <= kEplNmtMsNotActive)
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
    {
        nodeId_p = EPL_C_ADR_MN_DEF_NODE_ID;
    }

    if (nodeId_p == EPL_C_ADR_MN_DEF_NODE_ID)
    {   // apply command to local node-ID

        switch (nmtCommand_p)
        {
            case kEplNmtCmdIdentResponse:
            {   // issue request for local node
                ret = identu_requestIdentResponse(0x00, NULL);
                goto Exit;
            }

            case kEplNmtCmdStatusResponse:
            {   // issue request for local node
                ret = statusu_requestStatusResponse(0x00, NULL);
                goto Exit;
            }

            case kEplNmtCmdResetNode:
            case kEplNmtCmdResetCommunication:
            case kEplNmtCmdResetConfiguration:
            case kEplNmtCmdSwReset:
            {
                nodeId_p = EPL_C_ADR_BROADCAST;
                break;
            }

            case kEplNmtCmdInvalidService:
            default:
            {
                ret = kEplObdAccessViolation;
                goto Exit;
            }
        }
    }

    if (nodeId_p != EPL_C_ADR_BROADCAST)
    {   // apply command to remote node-ID, but not broadcast
    tNmtMnuNodeInfo* pNodeInfo;

        pNodeInfo = EPL_NMTMNU_GET_NODEINFO(nodeId_p);

        switch (nmtCommand_p)
        {
            case kEplNmtCmdIdentResponse:
            {   // issue request for remote node
                // if it is a non-existing node or no identrequest is running
                if (((pNodeInfo->nodeCfg
                     & (EPL_NODEASSIGN_NODE_IS_CN | EPL_NODEASSIGN_NODE_EXISTS))
                        != (EPL_NODEASSIGN_NODE_IS_CN | EPL_NODEASSIGN_NODE_EXISTS))
                    || ((pNodeInfo->nodeState != kNmtMnuNodeStateResetConf)
                        && (pNodeInfo->nodeState != kNmtMnuNodeStateConfRestored)
                        && (pNodeInfo->nodeState != kNmtMnuNodeStateUnknown)))
                {
                    ret = identu_requestIdentResponse(nodeId_p, NULL);
                }
                goto Exit;
            }

            case kEplNmtCmdStatusResponse:
            {   // issue request for remote node
                // if it is a non-existing node or operational and not async-only
                if (((pNodeInfo->nodeCfg
                     & (EPL_NODEASSIGN_NODE_IS_CN | EPL_NODEASSIGN_NODE_EXISTS))
                        != (EPL_NODEASSIGN_NODE_IS_CN | EPL_NODEASSIGN_NODE_EXISTS))
                    || (((pNodeInfo->nodeCfg & EPL_NODEASSIGN_ASYNCONLY_NODE) == 0)
                        && (pNodeInfo->nodeState == kNmtMnuNodeStateOperational)))
                {
                    ret = statusu_requestStatusResponse(nodeId_p, NULL);
                }
                goto Exit;
            }

            default:
            {
                break;
            }
        }
    }

    switch (nmtCommand_p)
    {
        case kEplNmtCmdResetNode:
        case kEplNmtCmdResetCommunication:
        case kEplNmtCmdResetConfiguration:
        case kEplNmtCmdSwReset:
        {
            if (nodeId_p == EPL_C_ADR_BROADCAST)
            {   // memorize that this is a user requested reset
                nmtMnuInstance_g.flags |= EPL_NMTMNU_FLAG_USER_RESET;
            }
            break;
        }

        case kEplNmtCmdStartNode:
        case kEplNmtCmdStopNode:
        case kEplNmtCmdEnterPreOperational2:
        case kEplNmtCmdEnableReadyToOperate:
        default:
        {
            break;
        }
/*
        case kEplNmtCmdInvalidService:
        default:
        {
            ret = kEplObdAccessViolation;
            goto Exit;
        }
*/
    }

    // send command to remote node
    ret = nmtmnu_sendNmtCommand(nodeId_p, nmtCommand_p);

Exit:
    return ret;
}

//---------------------------------------------------------------------------
//
// Function:    EplNmtMnuTriggerStateChange
//
// Description: triggers the specified node command for the specified node.
//
// Parameters:  nodeId_p              = node ID for which the node command will be executed
//              NodeCommand_p           = node command
//
// Returns:     tEplKernel              = error code
//
// State:
//
//---------------------------------------------------------------------------

tEplKernel nmtmnu_triggerStateChange(UINT nodeId_p,
                                       tEplNmtNodeCommand  nodeCommand_p)
{
tEplKernel          ret = kEplSuccessful;
tNmtMnuNodeCmd      nodeCmd;
tEplEvent           event;

    if ((nodeId_p == 0) || (nodeId_p >= EPL_C_ADR_BROADCAST))
    {
        ret = kEplInvalidNodeId;
        goto Exit;
    }

    nodeCmd.nodeCommand = nodeCommand_p;
    nodeCmd.nodeId = nodeId_p;
    event.m_EventSink = kEplEventSinkNmtMnu;
    event.m_EventType = kEplEventTypeNmtMnuNodeCmd;
    EPL_MEMSET(&event.m_NetTime, 0x00, sizeof(event.m_NetTime));
    event.m_pArg = &nodeCmd;
    event.m_uiSize = sizeof (nodeCmd);
    ret = eventu_postEvent(&event);
    if (ret != kEplSuccessful)
    {
        goto Exit;
    }

Exit:
    return ret;
}


//---------------------------------------------------------------------------
//
// Function:    EplNmtMnuCbNmtStateChange
//
// Description: callback function for NMT state changes
//
// Parameters:  NmtStateChange_p        = NMT state change event
//
// Returns:     tEplKernel              = error code
//
//
// State:
//
//---------------------------------------------------------------------------

tEplKernel nmtmnu_cbNmtStateChange(tEplEventNmtStateChange nmtStateChange_p)
{
    tEplKernel      ret = kEplSuccessful;
    UINT8           newMnNmtState;

    // Save new MN state in object 0x1F8E
    newMnNmtState   = (UINT8) nmtStateChange_p.m_NewNmtState;

    ret = EplObdWriteEntry(0x1F8E, 240, &newMnNmtState, 1);
    if(ret != kEplSuccessful)
    {
        return  ret;
    }

    // do work which must be done in that state
    switch (nmtStateChange_p.m_NewNmtState)
    {
        // EPL stack is not running
/*        case kEplNmtGsOff:
            break;

        // first init of the hardware
        case kEplNmtGsInitialising:
            break;

        // init of the manufacturer-specific profile area and the
        // standardised device profile area
        case kEplNmtGsResetApplication:
        {
            break;
        }

        // init of the communication profile area
        case kEplNmtGsResetCommunication:
        {
            break;
        }
*/
        // build the configuration with infos from OD
        case kEplNmtGsResetConfiguration:
        {
        UINT32          dwTimeout;
        tEplObdSize     ObdSize;

            // read object 0x1F80 NMT_StartUp_U32
            ObdSize = 4;
            ret = EplObduReadEntry(0x1F80, 0, &nmtMnuInstance_g.nmtStartup, &ObdSize);
            if (ret != kEplSuccessful)
            {
                break;
            }

            // compute StatusReqDelay = object 0x1006 * EPL_C_NMT_STATREQ_CYCLE
            ObdSize = sizeof (dwTimeout);
            ret = EplObduReadEntry(0x1006, 0, &dwTimeout, &ObdSize);
            if (ret != kEplSuccessful)
            {
                break;
            }
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
            ObdSize = sizeof (dwTimeout);
            ret = EplObduReadEntry(0x1F89, 4, &dwTimeout, &ObdSize);
            if (ret != kEplSuccessful)
            {
                break;
            }
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
            break;
        }
/*
        //-----------------------------------------------------------
        // CN part of the state machine

        // node liste for EPL-Frames and check timeout
        case kEplNmtCsNotActive:
        {
            break;
        }

        // node process only async frames
        case kEplNmtCsPreOperational1:
        {
            break;
        }

        // node process isochronous and asynchronous frames
        case kEplNmtCsPreOperational2:
        {
            break;
        }

        // node should be configured and application is ready
        case kEplNmtCsReadyToOperate:
        {
            break;
        }

        // normal work state
        case kEplNmtCsOperational:
        {
            break;
        }

        // node stopped by MN
        // -> only process asynchronous frames
        case kEplNmtCsStopped:
        {
            break;
        }

        // no EPL cycle
        // -> normal ethernet communication
        case kEplNmtCsBasicEthernet:
        {
            break;
        }
*/
        //-----------------------------------------------------------
        // MN part of the state machine

        // node listens for EPL-Frames and check timeout
        case kEplNmtMsNotActive:
        {
            break;
        }

        // node processes only async frames
        case kEplNmtMsPreOperational1:
        {
        UINT32          dwTimeout;
        tEplTimerArg    TimerArg;
        tEplObdSize     ObdSize;
        tEplEvent       Event;
		BOOL			fNmtResetAllIssued = FALSE;

            // reset IdentResponses and running IdentRequests and StatusRequests
            ret = identu_reset();
            ret = statusu_reset();
#if EPL_NMTMNU_PRES_CHAINING_MN != FALSE
            ret = EplSyncuReset();
#endif

            // reset timers
            ret = EplNmtMnuReset();

            // 2008/11/18 d.k. reset internal node info is not necessary,
            //                 because timer flags are important and other
            //                 things are reset by EplNmtMnuStartBootStep1().
/*
            EPL_MEMSET(nmtMnuInstance_g.aNodeInfo,
                       0,
                       sizeof (nmtMnuInstance_g.aNodeInfo));
*/

            // inform DLL about NMT state change,
            // so that it can clear the asynchronous queues and start the reduced cycle
            Event.m_EventSink = kEplEventSinkDllk;
            Event.m_EventType = kEplEventTypeDllkStartReducedCycle;
            EPL_MEMSET(&Event.m_NetTime, 0x00, sizeof(Event.m_NetTime));
            Event.m_pArg = NULL;
            Event.m_uiSize = 0;
            ret = eventu_postEvent(&Event);
            if (ret != kEplSuccessful)
            {
                break;
            }

            // reset all nodes
            // skip this step if we come directly from OPERATIONAL
            // or it was just done before, e.g. because of a ResetNode command
            // from a diagnostic node
            if ((nmtStateChange_p.m_NmtEvent == kEplNmtEventTimerMsPreOp1)
                || ((nmtMnuInstance_g.flags & EPL_NMTMNU_FLAG_USER_RESET) == 0))
            {
                BENCHMARK_MOD_07_TOGGLE(7);

                EPL_NMTMNU_DBG_POST_TRACE_VALUE(0,
                                                EPL_C_ADR_BROADCAST,
                                                kEplNmtCmdResetNode);

                ret = nmtmnu_sendNmtCommand(EPL_C_ADR_BROADCAST, kEplNmtCmdResetNode);
                if (ret != kEplSuccessful)
                {
                    break;
                }
				fNmtResetAllIssued = TRUE;
            }

            // clear global flags, e.g. reenable boot process
            nmtMnuInstance_g.flags = 0;

            // start network scan
            ret = EplNmtMnuStartBootStep1(fNmtResetAllIssued);

            // start timer for 0x1F89/2 MNTimeoutPreOp1_U32
            ObdSize = sizeof (dwTimeout);
            ret = EplObduReadEntry(0x1F89, 2, &dwTimeout, &ObdSize);
            if (ret != kEplSuccessful)
            {
                break;
            }
            if (dwTimeout != 0L)
            {
                dwTimeout /= 1000L;
                if (dwTimeout == 0L)
                {
                    dwTimeout = 1L; // at least 1 ms
                }
                TimerArg.m_EventSink = kEplEventSinkNmtMnu;
                TimerArg.m_Arg.m_dwVal = 0;
                ret = EplTimeruModifyTimerMs(&nmtMnuInstance_g.timerHdlNmtState, dwTimeout, TimerArg);
            }
            break;
        }

        // node processes isochronous and asynchronous frames
        case kEplNmtMsPreOperational2:
        {
            ret = EplNmtMnuStartBootStep2();

            // wait for NMT state change of CNs
            break;
        }

        // node should be configured and application is ready
        case kEplNmtMsReadyToOperate:
        {
            // check if PRes of CNs are OK
            // d.k. that means wait CycleLength * MultiplexCycleCount (i.e. start timer)
            //      because Dllk checks PRes of CNs automatically in ReadyToOp
            ret = EplNmtMnuStartCheckCom();
            break;
        }

        // normal work state
        case kEplNmtMsOperational:
        {
            // send StartNode to CNs
            // wait for NMT state change of CNs
            ret = EplNmtMnuStartNodes();
            break;
        }

        // no EPL cycle
        // -> normal ethernet communication
        case kEplNmtMsBasicEthernet:
        {
            break;
        }

        default:
        {
//            TRACE("EplNmtMnuCbNmtStateChange(): unhandled NMT state\n");
        }
    }

    return ret;
}


//---------------------------------------------------------------------------
//
// Function:    EplNmtMnuCbCheckEvent
//
// Description: callback function for NMT events before they are actually executed.
//              The EPL API layer must forward NMT events from NmtCnu module.
//              This module will reject some NMT commands while MN.
//
// Parameters:  NmtEvent_p              = outstanding NMT event for approval
//
// Returns:     tEplKernel              = error code
//                      kEplReject      = reject the NMT event
//
// State:
//
//---------------------------------------------------------------------------

tEplKernel nmtmnu_cbCheckEvent(tEplNmtEvent nmtEvent_p)
{
tEplKernel      ret = kEplSuccessful;

    UNUSED_PARAMETER(nmtEvent_p);

    return ret;
}


//---------------------------------------------------------------------------
//
// Function:    EplNmtuProcessEvent
//
// Description: processes events from event queue
//
// Parameters:  pEvent_p        = pointer to event
//
// Returns:     tEplKernel      = errorcode
//
// State:
//
//---------------------------------------------------------------------------
//jba why EPLDLLEXPORT
EPLDLLEXPORT tEplKernel PUBLIC nmtmnu_processEvent(tEplEvent* pEvent_p)
{
tEplKernel      ret;

    ret = kEplSuccessful;

    // process event
    switch(pEvent_p->m_EventType)
    {
        // timer event
        case kEplEventTypeTimer:
        {
        tEplTimerEventArg*  pTimerEventArg = (tEplTimerEventArg*)pEvent_p->m_pArg;
        UINT                nodeId;

            nodeId = (UINT) (pTimerEventArg->m_Arg.m_dwVal & EPL_NMTMNU_TIMERARG_NODE_MASK);
            if (nodeId != 0)
            {
            tEplObdSize         ObdSize;
            UINT8                bNmtState;
            tNmtMnuNodeInfo* pNodeInfo;

                pNodeInfo = EPL_NMTMNU_GET_NODEINFO(nodeId);

                ObdSize = 1;
                ret = EplObduReadEntry(0x1F8E, nodeId, &bNmtState, &ObdSize);
                if (ret != kEplSuccessful)
                {
                    break;
                }

                if ((pTimerEventArg->m_Arg.m_dwVal & EPL_NMTMNU_TIMERARG_IDENTREQ) != 0L)
                {
                    if ((UINT32)(pNodeInfo->flags & EPL_NMTMNU_NODE_FLAG_COUNT_STATREQ)
                        != (pTimerEventArg->m_Arg.m_dwVal & EPL_NMTMNU_TIMERARG_COUNT_SR))
                    {   // this is an old (already deleted or modified) timer
                        // but not the current timer
                        // so discard it
                        EPL_NMTMNU_DBG_POST_TRACE_VALUE(kNmtMnuIntNodeEventTimerIdentReq,
                                                        nodeId,
                                                        ((pNodeInfo->nodeState << 8)
                                                         | 0xFF));

                        break;
                    }
/*
                    EPL_NMTMNU_DBG_POST_TRACE_VALUE(kNmtMnuIntNodeEventTimerIdentReq,
                                                    uiNodeId,
                                                    ((pNodeInfo->nodeState << 8)
                                                     | 0x80
                                                     | ((pNodeInfo->flags & EPL_NMTMNU_NODE_FLAG_COUNT_STATREQ) >> 6)
                                                     | ((pTimerEventArg->m_Arg.m_dwVal & EPL_NMTMNU_TIMERARG_COUNT_SR) >> 8)));
*/
                    ret = EplNmtMnuProcessInternalEvent(nodeId,
                                                        (tEplNmtState) (bNmtState | EPL_NMT_TYPE_CS),
                                                        EPL_E_NO_ERROR,
                                                        kNmtMnuIntNodeEventTimerIdentReq);
                }

                else if ((pTimerEventArg->m_Arg.m_dwVal & EPL_NMTMNU_TIMERARG_STATREQ) != 0L)
                {
                    if ((UINT32)(pNodeInfo->flags & EPL_NMTMNU_NODE_FLAG_COUNT_STATREQ)
                        != (pTimerEventArg->m_Arg.m_dwVal & EPL_NMTMNU_TIMERARG_COUNT_SR))
                    {   // this is an old (already deleted or modified) timer
                        // but not the current timer
                        // so discard it
                        EPL_NMTMNU_DBG_POST_TRACE_VALUE(kNmtMnuIntNodeEventTimerStatReq,
                                                        nodeId,
                                                        ((pNodeInfo->nodeState << 8)
                                                         | 0xFF));

                        break;
                    }
/*
                    EPL_NMTMNU_DBG_POST_TRACE_VALUE(kNmtMnuIntNodeEventTimerStatReq,
                                                    uiNodeId,
                                                    ((pNodeInfo->nodeState << 8)
                                                     | 0x80
                                                     | ((pNodeInfo->flags & EPL_NMTMNU_NODE_FLAG_COUNT_STATREQ) >> 6)
                                                     | ((pTimerEventArg->m_Arg.m_dwVal & EPL_NMTMNU_TIMERARG_COUNT_SR) >> 8)));
*/
                    ret = EplNmtMnuProcessInternalEvent(nodeId,
                                                        (tEplNmtState) (bNmtState | EPL_NMT_TYPE_CS),
                                                        EPL_E_NO_ERROR,
                                                        kNmtMnuIntNodeEventTimerStatReq);
                }

                else if ((pTimerEventArg->m_Arg.m_dwVal & EPL_NMTMNU_TIMERARG_STATE_MON) != 0L)
                {
                    if ((UINT32)(pNodeInfo->flags & EPL_NMTMNU_NODE_FLAG_COUNT_STATREQ)
                        != (pTimerEventArg->m_Arg.m_dwVal & EPL_NMTMNU_TIMERARG_COUNT_SR))
                    {   // this is an old (already deleted or modified) timer
                        // but not the current timer
                        // so discard it
                        EPL_NMTMNU_DBG_POST_TRACE_VALUE(kNmtMnuIntNodeEventTimerStateMon,
                                                        nodeId,
                                                        ((pNodeInfo->nodeState << 8)
                                                         | 0xFF));

                        break;
                    }
/*
                    EPL_NMTMNU_DBG_POST_TRACE_VALUE(kNmtMnuIntNodeEventTimerStatReq,
                                                    uiNodeId,
                                                    ((pNodeInfo->nodeState << 8)
                                                     | 0x80
                                                     | ((pNodeInfo->flags & EPL_NMTMNU_NODE_FLAG_COUNT_STATREQ) >> 6)
                                                     | ((pTimerEventArg->m_Arg.m_dwVal & EPL_NMTMNU_TIMERARG_COUNT_SR) >> 8)));
*/
                    ret = EplNmtMnuProcessInternalEvent(nodeId,
                                                        (tEplNmtState) (bNmtState | EPL_NMT_TYPE_CS),
                                                        EPL_E_NO_ERROR,
                                                        kNmtMnuIntNodeEventTimerStateMon);
                }

                else if ((pTimerEventArg->m_Arg.m_dwVal & EPL_NMTMNU_TIMERARG_LONGER) != 0L)
                {
                    if ((UINT32)(pNodeInfo->flags & EPL_NMTMNU_NODE_FLAG_COUNT_LONGER)
                        != (pTimerEventArg->m_Arg.m_dwVal & EPL_NMTMNU_TIMERARG_COUNT_LO))
                    {   // this is an old (already deleted or modified) timer
                        // but not the current timer
                        // so discard it
                        EPL_NMTMNU_DBG_POST_TRACE_VALUE(kNmtMnuIntNodeEventTimerLonger,
                                                        nodeId,
                                                        ((pNodeInfo->nodeState << 8)
                                                         | 0xFF));

                        break;
                    }
/*
                    EPL_NMTMNU_DBG_POST_TRACE_VALUE(kNmtMnuIntNodeEventTimerLonger,
                                                    uiNodeId,
                                                    ((pNodeInfo->nodeState << 8)
                                                     | 0x80
                                                     | ((pNodeInfo->flags & EPL_NMTMNU_NODE_FLAG_COUNT_LONGER) >> 6)
                                                     | ((pTimerEventArg->m_Arg.m_dwVal & EPL_NMTMNU_TIMERARG_COUNT_LO) >> 8)));
*/
                    ret = EplNmtMnuProcessInternalEvent(nodeId,
                                                        (tEplNmtState) (bNmtState | EPL_NMT_TYPE_CS),
                                                        EPL_E_NO_ERROR,
                                                        kNmtMnuIntNodeEventTimerLonger);
                }

            }
            else
            {   // global timer event
            }
            break;
        }

        case kEplEventTypeHeartbeat:
        {
        tEplHeartbeatEvent* pHeartbeatEvent = (tEplHeartbeatEvent*)pEvent_p->m_pArg;

            ret = EplNmtMnuProcessInternalEvent(pHeartbeatEvent->m_uiNodeId,
                                                pHeartbeatEvent->m_NmtState,
                                                pHeartbeatEvent->m_wErrorCode,
                                                kNmtMnuIntNodeEventHeartbeat);
            break;
        }

        case kEplEventTypeNmtMnuNmtCmdSent:
        {
        tEplFrame* pFrame = (tEplFrame*)pEvent_p->m_pArg;
        UINT                uiNodeId;
        tEplNmtCommand      NmtCommand;
        UINT8                bNmtState;

            if (pEvent_p->m_uiSize < EPL_C_DLL_MINSIZE_NMTCMD)
            {
                ret = eventu_postError(kEplEventSourceNmtMnu, kEplNmtInvalidFramePointer, sizeof (pEvent_p->m_uiSize), &pEvent_p->m_uiSize);
                break;
            }

            uiNodeId = AmiGetByteFromLe(&pFrame->m_le_bDstNodeId);
            NmtCommand = (tEplNmtCommand) AmiGetByteFromLe(&pFrame->m_Data.m_Asnd.m_Payload.m_NmtCommandService.m_le_bNmtCommandId);

            switch (NmtCommand)
            {
                case kEplNmtCmdStartNode:
                    bNmtState = (UINT8) (kEplNmtCsOperational & 0xFF);
                    break;

                case kEplNmtCmdStopNode:
                    bNmtState = (UINT8) (kEplNmtCsStopped & 0xFF);
                    break;

                case kEplNmtCmdEnterPreOperational2:
                    bNmtState = (UINT8) (kEplNmtCsPreOperational2 & 0xFF);
                    break;

                case kEplNmtCmdEnableReadyToOperate:
                    // d.k. do not change expected node state, because of DS 1.0.0 7.3.1.2.1 Plain NMT State Command
                    //      and because node may not change NMT state within EPL_C_NMT_STATE_TOLERANCE
                    bNmtState = (UINT8) (kEplNmtCsPreOperational2 & 0xFF);
                    break;

                case kEplNmtCmdResetNode:
                case kEplNmtCmdResetCommunication:
                case kEplNmtCmdResetConfiguration:
                case kEplNmtCmdSwReset:
                    bNmtState = (UINT8) (kEplNmtCsNotActive & 0xFF);
                    // EplNmtMnuProcessInternalEvent() sets internal node state to kNmtMnuNodeStateUnknown
                    // after next unresponded IdentRequest/StatusRequest
                    break;

                default:
                    goto Exit;
            }

            // process as internal event which update expected NMT state in OD
            if (uiNodeId != EPL_C_ADR_BROADCAST)
            {
                ret = EplNmtMnuProcessInternalEvent(uiNodeId,
                                                    (tEplNmtState) (bNmtState | EPL_NMT_TYPE_CS),
                                                    0,
                                                    kNmtMnuIntNodeEventNmtCmdSent);

            }
            else
            {   // process internal event for all active nodes (except myself)

                for (uiNodeId = 1; uiNodeId <= tabentries(nmtMnuInstance_g.aNodeInfo); uiNodeId++)
                {
                    if ((EPL_NMTMNU_GET_NODEINFO(uiNodeId)->nodeCfg & (EPL_NODEASSIGN_NODE_IS_CN | EPL_NODEASSIGN_NODE_EXISTS)) != 0)
                    {
                        ret = EplNmtMnuProcessInternalEvent(uiNodeId,
                                                            (tEplNmtState) (bNmtState | EPL_NMT_TYPE_CS),
                                                            0,
                                                            kNmtMnuIntNodeEventNmtCmdSent);

                        if (ret != kEplSuccessful)
                        {
                            goto Exit;
                        }
                    }
                }

                if ((nmtMnuInstance_g.flags & EPL_NMTMNU_FLAG_USER_RESET) != 0)
                {   // user or diagnostic nodes requests a reset of the MN
                tEplNmtEvent    NmtEvent;

                    switch (NmtCommand)
                    {
                        case kEplNmtCmdResetNode:
                        {
                            NmtEvent = kEplNmtEventResetNode;
                            break;
                        }

                        case kEplNmtCmdResetCommunication:
                        {
                            NmtEvent = kEplNmtEventResetCom;
                            break;
                        }

                        case kEplNmtCmdResetConfiguration:
                        {
                            NmtEvent = kEplNmtEventResetConfig;
                            break;
                        }

                        case kEplNmtCmdSwReset:
                        {
                            NmtEvent = kEplNmtEventSwReset;
                            break;
                        }

                        case kEplNmtCmdInvalidService:
                        default:
                        {   // actually no reset was requested
                            goto Exit;
                        }
                    }

                    ret = EplNmtuNmtEvent(NmtEvent);
                    if (ret != kEplSuccessful)
                    {
                        goto Exit;
                    }
                }
            }

            break;
        }

        case kEplEventTypeNmtMnuNodeCmd:
        {
        tNmtMnuNodeCmd*      pNodeCmd = (tNmtMnuNodeCmd*)pEvent_p->m_pArg;
        tNmtMnuIntNodeEvent  NodeEvent;
        tEplObdSize             ObdSize;
        UINT8                    bNmtState;
        UINT16                    wErrorCode = EPL_E_NO_ERROR;

            if ((pNodeCmd->nodeId == 0) || (pNodeCmd->nodeId >= EPL_C_ADR_BROADCAST))
            {
                ret = kEplInvalidNodeId;
                goto Exit;
            }

            switch (pNodeCmd->nodeCommand)
            {
                case kEplNmtNodeCommandBoot:
                {
                    NodeEvent = kNmtMnuIntNodeEventBoot;
                    break;
                }

                case kEplNmtNodeCommandConfOk:
                {
                    NodeEvent = kNmtMnuIntNodeEventConfigured;
                    break;
                }

                case kEplNmtNodeCommandConfErr:
                {
                    NodeEvent = kNmtMnuIntNodeEventError;
                    wErrorCode = EPL_E_NMT_BPO1_CF_VERIFY;
                    break;
                }

                case kEplNmtNodeCommandConfRestored:
                {
                    NodeEvent = kNmtMnuIntNodeEventExecResetNode;
                    break;
                }

                case kEplNmtNodeCommandConfReset:
                {
                    NodeEvent = kNmtMnuIntNodeEventExecResetConf;
                    break;
                }

                default:
                {   // invalid node command
                    goto Exit;
                }
            }

            // fetch current NMT state
            ObdSize = sizeof (bNmtState);
            ret = EplObduReadEntry(0x1F8E, pNodeCmd->nodeId, &bNmtState, &ObdSize);
            if (ret != kEplSuccessful)
            {
                goto Exit;
            }

            ret = EplNmtMnuProcessInternalEvent(pNodeCmd->nodeId,
                                                (tEplNmtState) (bNmtState | EPL_NMT_TYPE_CS),
                                                wErrorCode,
                                                NodeEvent);
            break;
        }

        case kEplEventTypeNmtMnuNodeAdded:
        {
        UINT        uiNodeId;

            uiNodeId = *((UINT*) pEvent_p->m_pArg);

            ret = EplNmtMnuCbNodeAdded(uiNodeId);
            break;
        }

        default:
        {
            ret = kEplNmtInvalidEvent;
        }

    }

Exit:
    return ret;
}


//---------------------------------------------------------------------------
//
// Function:    EplNmtMnuGetDiagnosticInfo
//
// Description: returns diagnostic information
//
// Parameters:  puiMandatorySlaveCount_p    = OUT: Mandatory Slave Count
//              puiSignalSlaveCount_p       = OUT: Signal Slave Count
//              pwFlags_p                   = OUT: Global flags
//
// Returns:     tEplKernel                  = error code
//
// State:
//
//---------------------------------------------------------------------------

tEplKernel nmtmnu_getDiagnosticInfo(UINT* pMandatorySlaveCount_p,
                                    UINT* pSignalSlaveCount_p,
                                    UINT16* pFlags_p)
{
tEplKernel      ret = kEplSuccessful;

    if ((pMandatorySlaveCount_p == NULL)
        || (pSignalSlaveCount_p == NULL)
        || (pFlags_p == NULL))
    {
        ret = kEplNmtInvalidParam;
        goto Exit;
    }

    *pMandatorySlaveCount_p = nmtMnuInstance_g.mandatorySlaveCount;
    *pSignalSlaveCount_p = nmtMnuInstance_g.signalSlaveCount;
    *pFlags_p = nmtMnuInstance_g.flags;

Exit:
    return ret;
}


//---------------------------------------------------------------------------
//
// Function:    EplNmtMnuGetRunningTimerStatReq
//
// Description: returns a bit field with running StatReq timers
//              just for debugging purposes
//
// Parameters:  (none)
//
// Returns:     tEplKernel              = error code
//
// State:
//
//---------------------------------------------------------------------------
/*
UINT32 nmtmnu_getRunningTimerStatReq(void)
{
tEplKernel      ret = kEplSuccessful;
UINT    uiIndex;
tNmtMnuNodeInfo* pNodeInfo;

    pNodeInfo = nmtMnuInstance_g.aNodeInfo;
    for (uiIndex = 1; uiIndex <= tabentries(nmtMnuInstance_g.aNodeInfo); uiIndex++, pNodeInfo++)
    {
        if (pNodeInfo->nodeState == kNmtMnuNodeStateConfigured)
        {
            // reset flag "scanned once"
            pNodeInfo->flags &= ~EPL_NMTMNU_NODE_FLAG_SCANNED;

            ret = EplNmtMnuNodeBootStep2(uiIndex, pNodeInfo);
            if (ret != kEplSuccessful)
            {
                goto Exit;
            }
            nmtMnuInstance_g.signalSlaveCount++;
            // signal slave counter shall be decremented if StatusRequest was sent once to a CN
            // mandatory slave counter shall be decremented if mandatory CN is ReadyToOp
        }
    }

Exit:
    return ret;
}
*/


#if EPL_NMTMNU_PRES_CHAINING_MN != FALSE
//---------------------------------------------------------------------------
//
// Function:    EplNmtMnuPrcConfig
//
// Description: Configure PRes Chaining parameters
//
// Parameters:  void
//
// Returns:     tEplKernel              = error code
//
// State:
//
//---------------------------------------------------------------------------

tEplKernel nmtmnu_configPrc(tEplNmtMnuConfigParam* pConfigParam_p)
{
tEplKernel  ret;

    ret = kEplSuccessful;

    nmtMnuInstance_g.prcPResTimeFirstCorrectionNs =
        pConfigParam_p->prcPResTimeFirstCorrectionNs;
    nmtMnuInstance_g.prcPResTimeFirstNegOffsetNs =
        pConfigParam_p->prcPResTimeFirstNegOffsetNs;

    return ret;
}
#endif


//=========================================================================//
//                                                                         //
//          P R I V A T E   F U N C T I O N S                              //
//                                                                         //
//=========================================================================//

//---------------------------------------------------------------------------
//
// Function:    EplNmtMnuCbNmtRequest
//
// Description: callback funktion for NmtRequest
//
// Parameters:  pFrameInfo_p            = Frame with the NmtRequest
//
// Returns:     tEplKernel              = error code
//
// State:
//
//---------------------------------------------------------------------------

static tEplKernel PUBLIC EplNmtMnuCbNmtRequest(tEplFrameInfo * pFrameInfo_p)
{
tEplKernel      ret = kEplSuccessful;
UINT            targetNodeId;
tEplNmtCommand  nmtCommand;
tEplNmtRequestService*  pNmtRequestService;

    if ((pFrameInfo_p == NULL)
        || (pFrameInfo_p->m_pFrame == NULL))
    {
        ret = kEplNmtInvalidFramePointer;
        goto Exit;
    }

    pNmtRequestService = &pFrameInfo_p->m_pFrame->m_Data.m_Asnd.m_Payload.m_NmtRequestService;

    nmtCommand = (tEplNmtCommand)AmiGetByteFromLe(
            &pNmtRequestService->m_le_bNmtCommandId);

    targetNodeId = AmiGetByteFromLe(
            &pNmtRequestService->m_le_bTargetNodeId);

    ret = nmtmnu_requestNmtCommand(targetNodeId,
                                     nmtCommand);
    if (ret != kEplSuccessful)
    {   // error -> reply with kEplNmtCmdInvalidService
    UINT uiSourceNodeId;

        uiSourceNodeId = AmiGetByteFromLe(
                &pFrameInfo_p->m_pFrame->m_le_bSrcNodeId);
        ret = nmtmnu_sendNmtCommand(uiSourceNodeId, kEplNmtCmdInvalidService);
    }

Exit:
    return ret;
}


//---------------------------------------------------------------------------
//
// Function:    EplNmtMnuCbIdentResponse
//
// Description: callback funktion for IdentResponse
//
// Parameters:  nodeId_p              = node ID for which IdentReponse was received
//              pIdentResponse_p        = pointer to IdentResponse
//                                        is NULL if node did not answer
//
// Returns:     tEplKernel              = error code
//
// State:
//
//---------------------------------------------------------------------------

static tEplKernel PUBLIC EplNmtMnuCbIdentResponse(
                                  UINT        nodeId_p,
                                  tEplIdentResponse* pIdentResponse_p)
{
tEplKernel      ret = kEplSuccessful;

    if (pIdentResponse_p == NULL)
    {   // node did not answer
        ret = EplNmtMnuProcessInternalEvent(nodeId_p,
                                            kEplNmtCsNotActive,
                                            EPL_E_NMT_NO_IDENT_RES, // was EPL_E_NO_ERROR
                                            kNmtMnuIntNodeEventNoIdentResponse);
    }
    else
    {   // node answered IdentRequest
    tEplObdSize ObdSize;
    UINT32      dwDevType;
    UINT16        wErrorCode = EPL_E_NO_ERROR;
    tEplNmtState NmtState = (tEplNmtState) (AmiGetByteFromLe(&pIdentResponse_p->m_le_bNmtStatus) | EPL_NMT_TYPE_CS);

        // check IdentResponse $$$ move to ProcessIntern, because this function may be called also if CN

        // check DeviceType (0x1F84)
        ObdSize = 4;
        ret = EplObduReadEntry(0x1F84, nodeId_p, &dwDevType, &ObdSize);
        if (ret != kEplSuccessful)
        {
            goto Exit;
        }
        if (dwDevType != 0L)
        {   // actually compare it with DeviceType from IdentResponse
            if (AmiGetDwordFromLe(&pIdentResponse_p->m_le_dwDeviceType) != dwDevType)
            {   // wrong DeviceType
                NmtState = kEplNmtCsNotActive;
                wErrorCode = EPL_E_NMT_BPO1_DEVICE_TYPE;
            }
        }

        ret = EplNmtMnuProcessInternalEvent(nodeId_p,
                                            NmtState,
                                            wErrorCode,
                                            kNmtMnuIntNodeEventIdentResponse);
    }

Exit:
    return ret;
}


//---------------------------------------------------------------------------
//
// Function:    EplNmtMnuCbStatusResponse
//
// Description: callback funktion for StatusResponse
//
// Parameters:  nodeId_p              = node ID for which IdentReponse was received
//              pIdentResponse_p        = pointer to IdentResponse
//                                        is NULL if node did not answer
//
// Returns:     tEplKernel              = error code
//
// State:
//
//---------------------------------------------------------------------------

static tEplKernel PUBLIC EplNmtMnuCbStatusResponse(
                                  UINT        nodeId_p,
                                  tEplStatusResponse* pStatusResponse_p)
{
tEplKernel      ret = kEplSuccessful;

    if (pStatusResponse_p == NULL)
    {   // node did not answer
        ret = EplNmtMnuProcessInternalEvent(nodeId_p,
                                            kEplNmtCsNotActive,
                                            EPL_E_NMT_NO_STATUS_RES, // was EPL_E_NO_ERROR
                                            kNmtMnuIntNodeEventNoStatusResponse);
    }
    else
    {   // node answered StatusRequest
        ret = EplNmtMnuProcessInternalEvent(nodeId_p,
                                            (tEplNmtState) (AmiGetByteFromLe(&pStatusResponse_p->m_le_bNmtStatus) | EPL_NMT_TYPE_CS),
                                            EPL_E_NO_ERROR,
                                            kNmtMnuIntNodeEventStatusResponse);
    }

    return ret;
}


//---------------------------------------------------------------------------
//
// Function:    EplNmtMnuCbNodeAdded
//
// Description: This function is called after the addressed node has been
//              added in module Dllk.
//
// Parameters:  nodeId_p                  = Node ID
//
// Returns:     tEplKernel                  = error code
//
// State:
//
//---------------------------------------------------------------------------
static tEplKernel EplNmtMnuCbNodeAdded(UINT nodeId_p)
{
tEplKernel          ret;
tNmtMnuNodeInfo* pNodeInfo;

    ret = kEplSuccessful;
    pNodeInfo = EPL_NMTMNU_GET_NODEINFO(nodeId_p);

    pNodeInfo->flags |= EPL_NMTMNU_NODE_FLAG_ISOCHRON;

    if (pNodeInfo->nodeState == kNmtMnuNodeStateConfigured)
    {
    tEplNmtState    NmtState;

        NmtState = EplNmtuGetNmtState();
        if (NmtState >= kEplNmtMsPreOperational2)
        {
            ret = EplNmtMnuNodeBootStep2(nodeId_p, pNodeInfo);
        }
    }

    return ret;
}


//---------------------------------------------------------------------------
//
// Function:    EplNmtMnuAddNodeIsochronous
//
// Description: Adds the given node to the isochronous phase
//
// Parameters:  nodeId_p              = node ID of node which is to be added
//
// Returns:     tEplKernel              = error code
//
// State:
//
//---------------------------------------------------------------------------

static tEplKernel EplNmtMnuAddNodeIsochronous(UINT nodeId_p)
{
tEplKernel          ret;

#if EPL_NMTMNU_PRES_CHAINING_MN != FALSE
tNmtMnuNodeInfo* pNodeInfo;

    ret = kEplSuccessful;

    if (nodeId_p != EPL_C_ADR_INVALID)
    {
        pNodeInfo = EPL_NMTMNU_GET_NODEINFO(nodeId_p);
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
        tEplDllNodeOpParam  NodeOpParam;

            NodeOpParam.m_OpNodeType = kEplDllNodeOpTypeIsochronous;
            NodeOpParam.m_uiNodeId = nodeId_p;

            ret = dllucal_addNode(&NodeOpParam);
            goto Exit;
        }
#if EPL_NMTMNU_PRES_CHAINING_MN != FALSE
        else
        {   // node is a PRC node

            nmtMnuInstance_g.flags |= EPL_NMTMNU_FLAG_PRC_ADD_SCHEDULED;
            pNodeInfo->prcFlags |= EPL_NMTMNU_NODE_FLAG_PRC_ADD_SCHEDULED;
        }
    }

    if (nmtMnuInstance_g.flags & EPL_NMTMNU_FLAG_PRC_ADD_IN_PROGRESS)
    {   // add PRC nodes is already in progress
        goto Exit;
    }

    if (nmtMnuInstance_g.flags & EPL_NMTMNU_FLAG_PRC_ADD_SCHEDULED)
    {
    UINT            nodeId;
    BOOL            fInvalidateNext;

        fInvalidateNext = FALSE;

        for (nodeId = 1; nodeId < 254; nodeId++)
        {
            pNodeInfo = EPL_NMTMNU_GET_NODEINFO(nodeId);
            if (pNodeInfo == NULL)
            {
                continue;
            }

            // $$$ only PRC

            if (pNodeInfo->flags & EPL_NMTMNU_NODE_FLAG_ISOCHRON)
            {
                if (fInvalidateNext != FALSE)
                {
                    // set relative propagation delay to invalid
                    pNodeInfo->relPropagationDelayNs = 0;
                    fInvalidateNext = FALSE;
                }
            }
            else if (pNodeInfo->prcFlags & EPL_NMTMNU_NODE_FLAG_PRC_ADD_SCHEDULED)
            {
                pNodeInfo->prcFlags &= ~EPL_NMTMNU_NODE_FLAG_PRC_ADD_SCHEDULED;
                pNodeInfo->prcFlags |=  EPL_NMTMNU_NODE_FLAG_PRC_ADD_IN_PROGRESS;
                nmtMnuInstance_g.flags |= EPL_NMTMNU_FLAG_PRC_ADD_IN_PROGRESS;

                // set relative propagation delay to invalid
                pNodeInfo->relPropagationDelayNs = 0;
                fInvalidateNext = TRUE;
            }
            // $$$ else if: falls es noch einen ADD_IN_PROGRESS gibt, Fehler
        }

        nmtMnuInstance_g.flags &= ~EPL_NMTMNU_FLAG_PRC_ADD_SCHEDULED;

        if (nmtMnuInstance_g.flags & EPL_NMTMNU_FLAG_PRC_ADD_IN_PROGRESS)
        {
            ret = EplNmtMnuPrcMeasure();
        }
    }
#endif

Exit:
    return ret;
}


//---------------------------------------------------------------------------
//
// Function:    EplNmtMnuStartBootStep1
//
// Description: starts BootStep1
//
// Parameters:  (none)
//
// Returns:     tEplKernel              = error code
//
// State:
//
//---------------------------------------------------------------------------

static tEplKernel EplNmtMnuStartBootStep1(BOOL fNmtResetAllIssued_p)
{
tEplKernel			ret = kEplSuccessful;
UINT		        subIndex;
UINT		        localNodeId;
UINT32				nodeCfg;
tEplObdSize			obdSize;
tNmtMnuNodeInfo*	pNodeInfo;

    // $$$ d.k.: save current time for 0x1F89/2 MNTimeoutPreOp1_U32

    // start network scan
    nmtMnuInstance_g.mandatorySlaveCount = 0;
    nmtMnuInstance_g.signalSlaveCount = 0;
    // check 0x1F81
    localNodeId = EplObduGetNodeId();
    for (subIndex = 1; subIndex <= 254; subIndex++)
    {
        obdSize = 4;
        ret = EplObduReadEntry(0x1F81, subIndex, &nodeCfg, &obdSize);
        if (ret != kEplSuccessful)
        {
            goto Exit;
        }
        if (subIndex != localNodeId)
        {
			pNodeInfo = EPL_NMTMNU_GET_NODEINFO(subIndex);

            // reset flags "not scanned" and "isochronous"
            pNodeInfo->flags &= ~(EPL_NMTMNU_NODE_FLAG_ISOCHRON | EPL_NMTMNU_NODE_FLAG_NOT_SCANNED);

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
	                ret = identu_requestIdentResponse(subIndex, EplNmtMnuCbIdentResponse);
					if (ret != kEplSuccessful)
					{
						goto Exit;
					}
				}

                // set flag "not scanned"
                pNodeInfo->flags |= EPL_NMTMNU_NODE_FLAG_NOT_SCANNED;
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
                ret = EplNmtMnuAddNodeIsochronous(localNodeId);
            }
        }
    }

Exit:
    return ret;
}


//---------------------------------------------------------------------------
//
// Function:    EplNmtMnuStartBootStep2
//
// Description: starts BootStep2.
//              That means checking if a node has reached PreOp2 and
//              has been added to the isochronous phase.
//              If this is met, the NMT EnableReadyToOp command is sent.
//
// Parameters:  (none)
//
// Returns:     tEplKernel              = error code
//
// State:
//
//---------------------------------------------------------------------------

static tEplKernel EplNmtMnuStartBootStep2(void)
{
tEplKernel      ret = kEplSuccessful;
UINT            index;
tNmtMnuNodeInfo* pNodeInfo;
tEplObdSize     obdSize;
UINT8            nmtState;
tEplNmtState    expNmtState;

    if ((nmtMnuInstance_g.flags & EPL_NMTMNU_FLAG_HALTED) == 0)
    {   // boot process is not halted
        nmtMnuInstance_g.mandatorySlaveCount = 0;
        nmtMnuInstance_g.signalSlaveCount = 0;
        // reset flag that application was informed about possible state change
        nmtMnuInstance_g.flags &= ~EPL_NMTMNU_FLAG_APP_INFORMED;
    }

    pNodeInfo = nmtMnuInstance_g.aNodeInfo;
    for (index = 1; index <= tabentries(nmtMnuInstance_g.aNodeInfo); index++, pNodeInfo++)
    {
        obdSize = 1;
        // read object 0x1F8F NMT_MNNodeExpState_AU8
        ret = EplObduReadEntry(0x1F8F, index, &nmtState, &obdSize);
        if (ret != kEplSuccessful)
        {
            goto Exit;
        }

        // compute expected NMT state
        expNmtState = (tEplNmtState) (nmtState | EPL_NMT_TYPE_CS);

        if (expNmtState == kEplNmtCsPreOperational1)
        {
        tEplTimerArg    TimerArg;

            // The change to PreOp2 is an implicit NMT command.
            // Unexpected NMT states of the nodes are ignored until
            // the state monitor timer is elapsed.
            EPL_NMTMNU_SET_FLAGS_TIMERARG_STATE_MON(
                    pNodeInfo, index, TimerArg);

            // set NMT state change flag
            pNodeInfo->flags |= EPL_NMTMNU_NODE_FLAG_NMT_CMD_ISSUED;

            ret = EplTimeruModifyTimerMs(&pNodeInfo->timerHdlStatReq, nmtMnuInstance_g.statusRequestDelay, TimerArg);
            if (ret != kEplSuccessful)
            {
                goto Exit;
            }

            // update object 0x1F8F NMT_MNNodeExpState_AU8 to PreOp2
            nmtState = (UINT8)(kEplNmtCsPreOperational2 & 0xFF);
            ret = EplObduWriteEntry(0x1F8F, index, &nmtState, 1);
            if (ret != kEplSuccessful)
            {
                goto Exit;
            }

            if ((pNodeInfo->nodeState == kNmtMnuNodeStateConfigured) &&
                ((nmtMnuInstance_g.flags & EPL_NMTMNU_FLAG_HALTED) == 0))
            {   // boot process is not halted
                // set flag "not scanned"
                pNodeInfo->flags |= EPL_NMTMNU_NODE_FLAG_NOT_SCANNED;

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


//---------------------------------------------------------------------------
//
// Function:    EplNmtMnuNodeBootStep2
//
// Description: starts BootStep2 for the specified node.
//              This means checking whether the CN is in NMT state PreOp2
//              and whether it has been added to the isochronous phase.
//              If both checks pass, it gets the NMT command EnableReadyToOp.
//              The CN must be in node state Configured, when it enters
//              BootStep2. When BootStep2 finishes, the CN is in node state
//              ReadyToOp.
//              If TimeoutReadyToOp in object 0x1F89/5 is configured,
//              TimerHdlLonger will be started with this timeout.
//
// Parameters:  nodeId_p              = node ID
//              pNodeInfo_p             = pointer to internal node info structure
//
// Returns:     tEplKernel              = error code
//
// State:
//
//---------------------------------------------------------------------------

static tEplKernel EplNmtMnuNodeBootStep2(UINT nodeId_p, tNmtMnuNodeInfo* pNodeInfo_p)
{
tEplKernel          ret;
tEplTimerArg        timerArg;

    ret = kEplSuccessful;

    if (pNodeInfo_p->nodeCfg & EPL_NODEASSIGN_ASYNCONLY_NODE)
    {   // node is async-only
    UINT8            bNmtState;
    tEplNmtState    nmtState;
    tEplObdSize     obdSize;


        // read object 0x1F8E NMT_MNNodeCurrState_AU8
        obdSize = 1;
        ret = EplObduReadEntry(0x1F8E, nodeId_p, &bNmtState, &obdSize);
        if (ret != kEplSuccessful)
        {
            goto Exit;
        }
        nmtState = (tEplNmtState) (bNmtState | EPL_NMT_TYPE_CS);

        if (nmtState != kEplNmtCsPreOperational2)
        {
            goto Exit;
        }
    }
    else
    {   // node is not async-only

        // The check whether the node has been added to the isochronous phase
        // implicates the check for NMT state PreOp2
        if ((pNodeInfo_p->flags & EPL_NMTMNU_NODE_FLAG_ISOCHRON) == 0)
        {
            goto Exit;
        }
    }

    EPL_NMTMNU_DBG_POST_TRACE_VALUE(0,
                                    nodeId_p,
                                    kEplNmtCmdEnableReadyToOperate);

    ret = nmtmnu_sendNmtCommand(nodeId_p, kEplNmtCmdEnableReadyToOperate);
    if (ret != kEplSuccessful)
    {
        goto Exit;
    }

    if (nmtMnuInstance_g.timeoutReadyToOp != 0L)
    {   // start timer
        // when the timer expires the CN must be ReadyToOp
        EPL_NMTMNU_SET_FLAGS_TIMERARG_LONGER(
                pNodeInfo_p, nodeId_p, timerArg);

        ret = EplTimeruModifyTimerMs(&pNodeInfo_p->timerHdlLonger, nmtMnuInstance_g.timeoutReadyToOp, timerArg);
    }

Exit:
    return ret;
}


//---------------------------------------------------------------------------
//
// Function:    EplNmtMnuStartCheckCom
//
// Description: starts CheckCommunication
//
// Parameters:  (none)
//
// Returns:     tEplKernel              = error code
//
// State:
//
//---------------------------------------------------------------------------

static tEplKernel EplNmtMnuStartCheckCom(void)
{
tEplKernel      ret = kEplSuccessful;
UINT            index;
tNmtMnuNodeInfo* pNodeInfo;


    if ((nmtMnuInstance_g.flags & EPL_NMTMNU_FLAG_HALTED) == 0)
    {   // boot process is not halted
        // wait some time and check that no communication error occurs
        nmtMnuInstance_g.mandatorySlaveCount = 0;
        nmtMnuInstance_g.signalSlaveCount = 0;
        // reset flag that application was informed about possible state change
        nmtMnuInstance_g.flags &= ~EPL_NMTMNU_FLAG_APP_INFORMED;

        pNodeInfo = nmtMnuInstance_g.aNodeInfo;
        for (index = 1; index <= tabentries(nmtMnuInstance_g.aNodeInfo); index++, pNodeInfo++)
        {
            if (pNodeInfo->nodeState == kNmtMnuNodeStateReadyToOp)
            {
                ret = EplNmtMnuNodeCheckCom(index, pNodeInfo);
                if (ret == kEplReject)
                {   // timer was started
                    // wait until it expires
                    if ((pNodeInfo->nodeCfg & EPL_NODEASSIGN_MANDATORY_CN) != 0)
                    {   // node is a mandatory CN
                        nmtMnuInstance_g.mandatorySlaveCount++;
                    }
                }
                else if (ret != kEplSuccessful)
                {
                    goto Exit;
                }

                // set flag "not scanned"
                pNodeInfo->flags |= EPL_NMTMNU_NODE_FLAG_NOT_SCANNED;

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


//---------------------------------------------------------------------------
//
// Function:    EplNmtMnuNodeCheckCom
//
// Description: checks communication of the specified node.
//              That means wait some time and if no error occurred everything
//              is OK.
//
// Parameters:  nodeId_p              = node ID
//              pNodeInfo_p             = pointer to internal node info structure
//
// Returns:     tEplKernel              = error code
//
// State:
//
//---------------------------------------------------------------------------

static tEplKernel EplNmtMnuNodeCheckCom(UINT nodeId_p, tNmtMnuNodeInfo* pNodeInfo_p)
{
tEplKernel      ret = kEplSuccessful;
UINT32          nodeCfg;
tEplTimerArg    timerArg;

    nodeCfg = pNodeInfo_p->nodeCfg;
    if (((nodeCfg & EPL_NODEASSIGN_ASYNCONLY_NODE) == 0)
        && (nmtMnuInstance_g.timeoutCheckCom != 0L))
    {   // CN is not async-only and timeout for CheckCom was set

        // check communication,
        // that means wait some time and if no error occurred everything is OK;

        // start timer (when the timer expires the CN must be still ReadyToOp)
        EPL_NMTMNU_SET_FLAGS_TIMERARG_LONGER(
                pNodeInfo_p, nodeId_p, timerArg);
//        timerArg.m_EventSink = kEplEventSinkNmtMnu;
//        timerArg.m_Arg.m_dwVal = EPL_NMTMNU_TIMERARG_LONGER | nodeId_p;
        ret = EplTimeruModifyTimerMs(&pNodeInfo_p->timerHdlLonger, nmtMnuInstance_g.timeoutCheckCom, timerArg);

        // update mandatory slave counter, because timer was started
        if (ret == kEplSuccessful)
        {
            ret = kEplReject;
        }
    }
    else
    {   // timer was not started
        // assume everything is OK
        pNodeInfo_p->nodeState = kNmtMnuNodeStateComChecked;
    }

//Exit:
    return ret;
}


//---------------------------------------------------------------------------
//
// Function:    EplNmtMnuStartNodes
//
// Description: really starts all nodes which are ReadyToOp and CheckCom did not fail
//
// Parameters:  (none)
//
// Returns:     tEplKernel              = error code
//
// State:
//
//---------------------------------------------------------------------------

static tEplKernel EplNmtMnuStartNodes(void)
{
tEplKernel      ret = kEplSuccessful;
UINT            index;
tNmtMnuNodeInfo* pNodeInfo;


    if ((nmtMnuInstance_g.flags & EPL_NMTMNU_FLAG_HALTED) == 0)
    {   // boot process is not halted
        // send NMT command Start Node
        nmtMnuInstance_g.mandatorySlaveCount = 0;
        nmtMnuInstance_g.signalSlaveCount = 0;
        // reset flag that application was informed about possible state change
        nmtMnuInstance_g.flags &= ~EPL_NMTMNU_FLAG_APP_INFORMED;

        pNodeInfo = nmtMnuInstance_g.aNodeInfo;
        for (index = 1; index <= tabentries(nmtMnuInstance_g.aNodeInfo); index++, pNodeInfo++)
        {
            if (pNodeInfo->nodeState == kNmtMnuNodeStateComChecked)
            {
                if ((nmtMnuInstance_g.nmtStartup & EPL_NMTST_STARTALLNODES) == 0)
                {
                    EPL_NMTMNU_DBG_POST_TRACE_VALUE(0,
                                                    index,
                                                    kEplNmtCmdStartNode);

                    ret = nmtmnu_sendNmtCommand(index, kEplNmtCmdStartNode);
                    if (ret != kEplSuccessful)
                    {
                        goto Exit;
                    }
                }

                if ((pNodeInfo->nodeCfg & EPL_NODEASSIGN_MANDATORY_CN) != 0)
                {   // node is a mandatory CN
                    nmtMnuInstance_g.mandatorySlaveCount++;
                }

                // set flag "not scanned"
                pNodeInfo->flags |= EPL_NMTMNU_NODE_FLAG_NOT_SCANNED;

                nmtMnuInstance_g.signalSlaveCount++;
                // signal slave counter shall be decremented if StatusRequest was sent once to a CN
                // mandatory slave counter shall be decremented if mandatory CN is OPERATIONAL
            }
        }

        // $$$ inform application if EPL_NMTST_NO_STARTNODE is set

        if ((nmtMnuInstance_g.nmtStartup & EPL_NMTST_STARTALLNODES) != 0)
        {
            EPL_NMTMNU_DBG_POST_TRACE_VALUE(0,
                                            EPL_C_ADR_BROADCAST,
                                            kEplNmtCmdStartNode);

            ret = nmtmnu_sendNmtCommand(EPL_C_ADR_BROADCAST, kEplNmtCmdStartNode);
            if (ret != kEplSuccessful)
            {
                goto Exit;
            }
        }
    }

Exit:
    return ret;
}


//---------------------------------------------------------------------------
//
// Function:    EplNmtMnuProcessInternalEvent
//
// Description: processes internal node events
//
// Parameters:  nodeId_p              = node ID
//              nodeNmtState_p          = NMT state of CN
//              nodeEvent_p             = occurred events
//
// Returns:     tEplKernel              = error code
//
//
// State:
//
//---------------------------------------------------------------------------

static tEplKernel EplNmtMnuProcessInternalEvent(
                                    UINT                nodeId_p,
                                    tEplNmtState        nodeNmtState_p,
                                    UINT16                errorCode_p,
                                    tNmtMnuIntNodeEvent nodeEvent_p)
{
tEplKernel          ret = kEplSuccessful;
tEplNmtState        nmtState;
tNmtMnuNodeInfo* pNodeInfo;
tEplTimerArg        timerArg;

    pNodeInfo = EPL_NMTMNU_GET_NODEINFO(nodeId_p);
    nmtState = EplNmtuGetNmtState();
    if (nmtState <= kEplNmtMsNotActive)
    {   // MN is not active
        goto Exit;
    }

    switch (nodeEvent_p)
    {
        case kNmtMnuIntNodeEventIdentResponse:
        {
        UINT8    bNmtState;

            EPL_NMTMNU_DBG_POST_TRACE_VALUE(nodeEvent_p,
                                            nodeId_p,
                                            pNodeInfo->nodeState);

            if ((pNodeInfo->nodeState != kNmtMnuNodeStateResetConf)
                && (pNodeInfo->nodeState != kNmtMnuNodeStateConfRestored))
            {
                pNodeInfo->nodeState = kNmtMnuNodeStateIdentified;
            }

            // reset flags ISOCHRON, NMT_CMD_ISSUED, and PREOP2_REACHED
            pNodeInfo->flags &= ~(EPL_NMTMNU_NODE_FLAG_ISOCHRON
                                     | EPL_NMTMNU_NODE_FLAG_NMT_CMD_ISSUED
                                     | EPL_NMTMNU_NODE_FLAG_PREOP2_REACHED);

            if (nmtState == kEplNmtMsPreOperational1)
            {
                if ((pNodeInfo->flags & EPL_NMTMNU_NODE_FLAG_NOT_SCANNED) != 0)
                {
                    // decrement only signal slave count
                    nmtMnuInstance_g.signalSlaveCount--;
                    pNodeInfo->flags &= ~EPL_NMTMNU_NODE_FLAG_NOT_SCANNED;
                }

                // update object 0x1F8F NMT_MNNodeExpState_AU8 to PreOp1
                bNmtState = (UINT8) (kEplNmtCsPreOperational1 & 0xFF);
            }
            else
            {   // MN is running full cycle
                // update object 0x1F8F NMT_MNNodeExpState_AU8 to PreOp2
                bNmtState = (UINT8) (kEplNmtCsPreOperational2 & 0xFF);

                if (nodeNmtState_p == kEplNmtCsPreOperational1)
                {   // The CN did not yet switch to PreOp2
                tEplTimerArg TimerArg;

                    // Set NMT state change flag and ignore unexpected NMT states
                    // until the state monitor timer is elapsed.
                    pNodeInfo->flags |= EPL_NMTMNU_NODE_FLAG_NMT_CMD_ISSUED;

                    EPL_NMTMNU_SET_FLAGS_TIMERARG_STATE_MON(
                            pNodeInfo, nodeId_p, TimerArg);

                    ret = EplTimeruModifyTimerMs(&pNodeInfo->timerHdlStatReq, nmtMnuInstance_g.statusRequestDelay, TimerArg);
                    if (ret != kEplSuccessful)
                    {
                        goto Exit;
                    }
                }
            }

            ret = EplObduWriteEntry(0x1F8F, nodeId_p, &bNmtState, 1);
            if (ret != kEplSuccessful)
            {
                goto Exit;
            }

            // check NMT state of CN
            ret = EplNmtMnuCheckNmtState(nodeId_p, pNodeInfo, nodeNmtState_p, errorCode_p, nmtState);
            if (ret != kEplSuccessful)
            {
                if (ret == kEplReject)
                {
                    ret = kEplSuccessful;
                }
                break;
            }

            if ((pNodeInfo->flags & EPL_NMTMNU_NODE_FLAG_NMT_CMD_ISSUED) == 0)
            {   // No state monitor timer is required
                // Request StatusResponse immediately,
                // because we want a fast boot-up of CNs
                ret = statusu_requestStatusResponse(nodeId_p, EplNmtMnuCbStatusResponse);
                if (ret != kEplSuccessful)
                {
                    EPL_NMTMNU_DBG_POST_TRACE_VALUE(nodeEvent_p,
                                                    nodeId_p,
                                                    ret);

                    if (ret == kEplInvalidOperation)
                    {   // the only situation when this should happen is, when
                        // StatusResponse was already requested from within
                        // the StatReq timer event.
                        // so ignore this error.
                        ret = kEplSuccessful;
                    }
                    else
                    {
                        break;
                    }
                }
            }

            if ((pNodeInfo->nodeState != kNmtMnuNodeStateResetConf)
                && (pNodeInfo->nodeState != kNmtMnuNodeStateConfRestored))
            {
                // inform application
                ret = nmtMnuInstance_g.pfnCbNodeEvent(nodeId_p,
                                                           kEplNmtNodeEventFound,
                                                           nodeNmtState_p,
                                                           EPL_E_NO_ERROR,
                                                           (pNodeInfo->nodeCfg & EPL_NODEASSIGN_MANDATORY_CN) != 0);
                if (ret == kEplReject)
                {   // interrupt boot process on user request
                    EPL_NMTMNU_DBG_POST_TRACE_VALUE(nodeEvent_p,
                                                    nodeId_p,
                                                    ((pNodeInfo->nodeState << 8)
                                                     | ret));

                    ret = kEplSuccessful;
                    break;
                }
                else if (ret != kEplSuccessful)
                {
                    EPL_NMTMNU_DBG_POST_TRACE_VALUE(nodeEvent_p,
                                                    nodeId_p,
                                                    ((pNodeInfo->nodeState << 8)
                                                     | ret));

                    break;
                }
            }

            // continue BootStep1
        }

        case kNmtMnuIntNodeEventBoot:
        {

            // $$$ check identification (vendor ID, product code, revision no, serial no)

            if (pNodeInfo->nodeState == kNmtMnuNodeStateIdentified)
            {
                // $$$ check software

                // check/start configuration
                // inform application
                ret = nmtMnuInstance_g.pfnCbNodeEvent(nodeId_p,
                                                           kEplNmtNodeEventCheckConf,
                                                           nodeNmtState_p,
                                                           EPL_E_NO_ERROR,
                                                           (pNodeInfo->nodeCfg & EPL_NODEASSIGN_MANDATORY_CN) != 0);
                if (ret == kEplReject)
                {   // interrupt boot process on user request
                    EPL_NMTMNU_DBG_POST_TRACE_VALUE(kNmtMnuIntNodeEventBoot,
                                                    nodeId_p,
                                                    ((pNodeInfo->nodeState << 8)
                                                     | ret));

                    ret = kEplSuccessful;
                    break;
                }
                else if (ret != kEplSuccessful)
                {
                    EPL_NMTMNU_DBG_POST_TRACE_VALUE(kNmtMnuIntNodeEventBoot,
                                                    nodeId_p,
                                                    ((pNodeInfo->nodeState << 8)
                                                     | ret));

                    break;
                }
            }
            else if (pNodeInfo->nodeState == kNmtMnuNodeStateConfRestored)
            {
                // check/start configuration
                // inform application
                ret = nmtMnuInstance_g.pfnCbNodeEvent(nodeId_p,
                                                           kEplNmtNodeEventUpdateConf,
                                                           nodeNmtState_p,
                                                           EPL_E_NO_ERROR,
                                                           (pNodeInfo->nodeCfg & EPL_NODEASSIGN_MANDATORY_CN) != 0);
                if (ret == kEplReject)
                {   // interrupt boot process on user request
                    EPL_NMTMNU_DBG_POST_TRACE_VALUE(kNmtMnuIntNodeEventBoot,
                                                    nodeId_p,
                                                    ((pNodeInfo->nodeState << 8)
                                                     | ret));

                    ret = kEplSuccessful;
                    break;
                }
                else if (ret != kEplSuccessful)
                {
                    EPL_NMTMNU_DBG_POST_TRACE_VALUE(kNmtMnuIntNodeEventBoot,
                                                    nodeId_p,
                                                    ((pNodeInfo->nodeState << 8)
                                                     | ret));

                    break;
                }
            }
            else if (pNodeInfo->nodeState != kNmtMnuNodeStateResetConf)
            {   // wrong CN state
                // ignore event
                break;
            }

            // we assume configuration is OK

            // continue BootStep1
        }

        case kNmtMnuIntNodeEventConfigured:
        {
            if ((pNodeInfo->nodeState != kNmtMnuNodeStateIdentified)
                && (pNodeInfo->nodeState != kNmtMnuNodeStateConfRestored)
                && (pNodeInfo->nodeState != kNmtMnuNodeStateResetConf))
            {   // wrong CN state
                // ignore event
                break;
            }

            pNodeInfo->nodeState = kNmtMnuNodeStateConfigured;

            if (nmtState == kEplNmtMsPreOperational1)
            {
                if ((pNodeInfo->nodeCfg & EPL_NODEASSIGN_MANDATORY_CN) != 0)
                {   // decrement mandatory CN counter
                    nmtMnuInstance_g.mandatorySlaveCount--;
                }
            }
            else
            {
                // put optional node to next step (BootStep2)
                ret = EplNmtMnuNodeBootStep2(nodeId_p, pNodeInfo);
            }
            break;
        }

        case kNmtMnuIntNodeEventNoIdentResponse:
        {
            if ((nmtState == kEplNmtMsPreOperational1)
                && ((pNodeInfo->flags & EPL_NMTMNU_NODE_FLAG_NOT_SCANNED) != 0))
            {
                // decrement only signal slave count
                nmtMnuInstance_g.signalSlaveCount--;
                pNodeInfo->flags &= ~EPL_NMTMNU_NODE_FLAG_NOT_SCANNED;
            }

            if ((pNodeInfo->nodeState != kNmtMnuNodeStateResetConf)
                && (pNodeInfo->nodeState != kNmtMnuNodeStateConfRestored))
            {
                pNodeInfo->nodeState = kNmtMnuNodeStateUnknown;
            }

            // check NMT state of CN
            ret = EplNmtMnuCheckNmtState(nodeId_p, pNodeInfo, nodeNmtState_p, errorCode_p, nmtState);
            if (ret == kEplReject)
            {
                ret = kEplSuccessful;
            }
            else if (ret != kEplSuccessful)
            {
                break;
            }

            // $$$ d.k. check start time for 0x1F89/2 MNTimeoutPreOp1_U32
            // $$$ d.k. check individual timeout 0x1F89/6 MNIdentificationTimeout_U32
            // if mandatory node and timeout elapsed -> halt boot procedure
            // trigger IdentRequest again (if >= PreOp2, after delay)
            if (nmtState >= kEplNmtMsPreOperational2)
            {   // start timer
                EPL_NMTMNU_SET_FLAGS_TIMERARG_IDENTREQ(
                        pNodeInfo, nodeId_p, timerArg);
//                TimerArg.m_EventSink = kEplEventSinkNmtMnu;
//                TimerArg.m_Arg.m_dwVal = EPL_NMTMNU_TIMERARG_IDENTREQ | nodeId_p;
/*
                EPL_NMTMNU_DBG_POST_TRACE_VALUE(kNmtMnuIntNodeEventNoIdentResponse,
                                                nodeId_p,
                                                ((pNodeInfo->nodeState << 8)
                                                 | 0x80
                                                 | ((pNodeInfo->flags & EPL_NMTMNU_NODE_FLAG_COUNT_STATREQ) >> 6)
                                                 | ((TimerArg.m_Arg.m_dwVal & EPL_NMTMNU_TIMERARG_COUNT_SR) >> 8)));
*/
                ret = EplTimeruModifyTimerMs(&pNodeInfo->timerHdlStatReq, nmtMnuInstance_g.statusRequestDelay, timerArg);
            }
            else
            {   // trigger IdentRequest immediately
                ret = identu_requestIdentResponse(nodeId_p, EplNmtMnuCbIdentResponse);
            }
            break;
        }

        case kNmtMnuIntNodeEventStatusResponse:
        {
            if ((nmtState >= kEplNmtMsPreOperational2)
                && ((pNodeInfo->flags & EPL_NMTMNU_NODE_FLAG_NOT_SCANNED) != 0))
            {
                // decrement only signal slave count if checked once for ReadyToOp, CheckCom, Operational
                nmtMnuInstance_g.signalSlaveCount--;
                pNodeInfo->flags &= ~EPL_NMTMNU_NODE_FLAG_NOT_SCANNED;
            }

            // check NMT state of CN
            ret = EplNmtMnuCheckNmtState(nodeId_p, pNodeInfo, nodeNmtState_p, errorCode_p, nmtState);
            if (ret != kEplSuccessful)
            {
                if (ret == kEplReject)
                {
                    ret = kEplSuccessful;
                }
                break;
            }

            if (nmtState == kEplNmtMsPreOperational1)
            {
                // request next StatusResponse immediately
                ret = statusu_requestStatusResponse(nodeId_p, EplNmtMnuCbStatusResponse);
                if (ret != kEplSuccessful)
                {
                    EPL_NMTMNU_DBG_POST_TRACE_VALUE(nodeEvent_p,
                                                    nodeId_p,
                                                    ret);
                }

            }
            else if ((pNodeInfo->flags & EPL_NMTMNU_NODE_FLAG_ISOCHRON) == 0)
            {   // start timer
                // not isochronously accessed CN (e.g. async-only or stopped CN)
                EPL_NMTMNU_SET_FLAGS_TIMERARG_STATREQ(
                        pNodeInfo, nodeId_p, timerArg);
//                TimerArg.m_EventSink = kEplEventSinkNmtMnu;
//                TimerArg.m_Arg.m_dwVal = EPL_NMTMNU_TIMERARG_STATREQ | nodeId_p;
/*
                EPL_NMTMNU_DBG_POST_TRACE_VALUE(kNmtMnuIntNodeEventStatusResponse,
                                                nodeId_p,
                                                ((pNodeInfo->nodeState << 8)
                                                 | 0x80
                                                 | ((pNodeInfo->flags & EPL_NMTMNU_NODE_FLAG_COUNT_STATREQ) >> 6)
                                                 | ((TimerArg.m_Arg.m_dwVal & EPL_NMTMNU_TIMERARG_COUNT_SR) >> 8)));
*/
                ret = EplTimeruModifyTimerMs(&pNodeInfo->timerHdlStatReq, nmtMnuInstance_g.statusRequestDelay, timerArg);
            }

            break;
        }

        case kNmtMnuIntNodeEventNoStatusResponse:
        {
            // function CheckNmtState sets node state to unknown if necessary
/*
            if ((NmtState >= kEplNmtMsPreOperational2)
                && ((pNodeInfo->flags & EPL_NMTMNU_NODE_FLAG_NOT_SCANNED) != 0))
            {
                // decrement only signal slave count if checked once for ReadyToOp, CheckCom, Operational
                nmtMnuInstance_g.signalSlaveCount--;
                pNodeInfo->flags &= ~EPL_NMTMNU_NODE_FLAG_NOT_SCANNED;
            }
*/
            // check NMT state of CN
            ret = EplNmtMnuCheckNmtState(nodeId_p, pNodeInfo, nodeNmtState_p, errorCode_p, nmtState);
            if (ret != kEplSuccessful)
            {
                if (ret == kEplReject)
                {
                    ret = kEplSuccessful;
                }
                break;
            }

            break;
        }

        case kNmtMnuIntNodeEventError:
        {   // currently only issued on kEplNmtNodeCommandConfErr

            if ((pNodeInfo->nodeState != kNmtMnuNodeStateIdentified)
                && (pNodeInfo->nodeState != kNmtMnuNodeStateConfRestored))
            {   // wrong CN state
                // ignore event
                break;
            }

            // check NMT state of CN
            ret = EplNmtMnuCheckNmtState(nodeId_p, pNodeInfo, kEplNmtCsNotActive, errorCode_p, nmtState);
            if (ret != kEplSuccessful)
            {
                if (ret == kEplReject)
                {
                    ret = kEplSuccessful;
                }
                break;
            }

            break;
        }

        case kNmtMnuIntNodeEventExecResetNode:
        {
            if (pNodeInfo->nodeState != kNmtMnuNodeStateIdentified)
            {   // wrong CN state
                // ignore event
                break;
            }

            pNodeInfo->nodeState = kNmtMnuNodeStateConfRestored;

            EPL_NMTMNU_DBG_POST_TRACE_VALUE(nodeEvent_p,
                                            nodeId_p,
                                            (((nodeNmtState_p & 0xFF) << 8)
                                            | kEplNmtCmdResetNode));

            // send NMT reset node to CN for activation of restored configuration
            ret = nmtmnu_sendNmtCommand(nodeId_p, kEplNmtCmdResetNode);

            break;
        }

        case kNmtMnuIntNodeEventExecResetConf:
        {
            if ((pNodeInfo->nodeState != kNmtMnuNodeStateIdentified)
                && (pNodeInfo->nodeState != kNmtMnuNodeStateConfRestored))
            {   // wrong CN state
                // ignore event
                break;
            }

            pNodeInfo->nodeState = kNmtMnuNodeStateResetConf;

            EPL_NMTMNU_DBG_POST_TRACE_VALUE(nodeEvent_p,
                                            nodeId_p,
                                            (((nodeNmtState_p & 0xFF) << 8)
                                            | kEplNmtCmdResetConfiguration));

            // send NMT reset configuration to CN for activation of configuration
            ret = nmtmnu_sendNmtCommand(nodeId_p, kEplNmtCmdResetConfiguration);

            break;
        }

        case kNmtMnuIntNodeEventHeartbeat:
        {
/*
            if ((NmtState >= kEplNmtMsPreOperational2)
                && ((pNodeInfo->flags & EPL_NMTMNU_NODE_FLAG_NOT_SCANNED) != 0))
            {
                // decrement only signal slave count if checked once for ReadyToOp, CheckCom, Operational
                nmtMnuInstance_g.signalSlaveCount--;
                pNodeInfo->flags &= ~EPL_NMTMNU_NODE_FLAG_NOT_SCANNED;
            }
*/
            // check NMT state of CN
            ret = EplNmtMnuCheckNmtState(nodeId_p, pNodeInfo, nodeNmtState_p, errorCode_p, nmtState);
            if (ret != kEplSuccessful)
            {
                if (ret == kEplReject)
                {
                    ret = kEplSuccessful;
                }
                break;
            }

            break;
        }

        case kNmtMnuIntNodeEventTimerIdentReq:
        {
            EPL_DBGLVL_NMTMN_TRACE("TimerStatReq->IdentReq(%02X)\n", nodeId_p);
            // trigger IdentRequest again
            ret = identu_requestIdentResponse(nodeId_p, EplNmtMnuCbIdentResponse);
            if (ret != kEplSuccessful)
            {
                EPL_NMTMNU_DBG_POST_TRACE_VALUE(nodeEvent_p,
                                                nodeId_p,
                                                (((nodeNmtState_p & 0xFF) << 8)
                                                 | ret));
                if (ret == kEplInvalidOperation)
                {   // this can happen because of a bug in EplTimeruLinuxKernel.c
                    // so ignore this error.
                    ret = kEplSuccessful;
                }
            }

            break;
        }

        case kNmtMnuIntNodeEventTimerStateMon:
        {
            // reset NMT state change flag
            // because from now on the CN must have the correct NMT state
            pNodeInfo->flags &= ~EPL_NMTMNU_NODE_FLAG_NMT_CMD_ISSUED;

            // continue with normal StatReq processing
        }

        case kNmtMnuIntNodeEventTimerStatReq:
        {
            EPL_DBGLVL_NMTMN_TRACE("TimerStatReq->StatReq(%02X)\n", nodeId_p);
            // request next StatusResponse
            ret = statusu_requestStatusResponse(nodeId_p, EplNmtMnuCbStatusResponse);
            if (ret != kEplSuccessful)
            {
                EPL_NMTMNU_DBG_POST_TRACE_VALUE(nodeEvent_p,
                                                nodeId_p,
                                                (((nodeNmtState_p & 0xFF) << 8)
                                                 | ret));
                if (ret == kEplInvalidOperation)
                {   // the only situation when this should happen is, when
                    // StatusResponse was already requested while processing
                    // event IdentResponse.
                    // so ignore this error.
                    ret = kEplSuccessful;
                }
            }

            break;
        }

        case kNmtMnuIntNodeEventTimerLonger:
        {
            switch (pNodeInfo->nodeState)
            {
                case kNmtMnuNodeStateConfigured:
                {   // node should be ReadyToOp but it is not

                    // check NMT state which shall be intentionally wrong, so that ERROR_TREATMENT will be started
                    ret = EplNmtMnuCheckNmtState(nodeId_p, pNodeInfo, kEplNmtCsNotActive, EPL_E_NMT_BPO2, nmtState);
                    if (ret != kEplSuccessful)
                    {
                        if (ret == kEplReject)
                        {
                            ret = kEplSuccessful;
                        }
                        break;
                    }

                    break;
                }

                case kNmtMnuNodeStateReadyToOp:
                {   // CheckCom finished successfully

                    pNodeInfo->nodeState = kNmtMnuNodeStateComChecked;

                    if ((pNodeInfo->flags & EPL_NMTMNU_NODE_FLAG_NOT_SCANNED) != 0)
                    {
                        // decrement only signal slave count if checked once for ReadyToOp, CheckCom, Operational
                        nmtMnuInstance_g.signalSlaveCount--;
                        pNodeInfo->flags &= ~EPL_NMTMNU_NODE_FLAG_NOT_SCANNED;
                    }

                    if ((pNodeInfo->nodeCfg & EPL_NODEASSIGN_MANDATORY_CN) != 0)
                    {
                        // decrement mandatory slave counter
                        nmtMnuInstance_g.mandatorySlaveCount--;
                    }
                    if (nmtState != kEplNmtMsReadyToOperate)
                    {
                        EPL_NMTMNU_DBG_POST_TRACE_VALUE(nodeEvent_p,
                                                        nodeId_p,
                                                        (((nodeNmtState_p & 0xFF) << 8)
                                                        | kEplNmtCmdStartNode));

                        // start optional CN
                        ret = nmtmnu_sendNmtCommand(nodeId_p, kEplNmtCmdStartNode);
                    }
                    break;
                }

                default:
                {
                    break;
                }
            }
            break;
        }

        case kNmtMnuIntNodeEventNmtCmdSent:
        {
        UINT8    bNmtState;

            // update expected NMT state with the one that results
            // from the sent NMT command
            bNmtState = (UINT8) (nodeNmtState_p & 0xFF);

            // write object 0x1F8F NMT_MNNodeExpState_AU8
            ret = EplObduWriteEntry(0x1F8F, nodeId_p, &bNmtState, 1);
            if (ret != kEplSuccessful)
            {
                goto Exit;
            }

            if (nodeNmtState_p == kEplNmtCsNotActive)
            {   // restart processing with IdentRequest
                EPL_NMTMNU_SET_FLAGS_TIMERARG_IDENTREQ(
                        pNodeInfo, nodeId_p, timerArg);
            }
            else
            {   // monitor NMT state change with StatusRequest after
                // the corresponding delay;
                // until then wrong NMT states will be ignored
                EPL_NMTMNU_SET_FLAGS_TIMERARG_STATE_MON(
                        pNodeInfo, nodeId_p, timerArg);

                // set NMT state change flag
                pNodeInfo->flags |= EPL_NMTMNU_NODE_FLAG_NMT_CMD_ISSUED;
            }

            ret = EplTimeruModifyTimerMs(&pNodeInfo->timerHdlStatReq, nmtMnuInstance_g.statusRequestDelay, timerArg);

            // finish processing, because NmtState_p is the expected and not the current state
            goto Exit;
        }

        default:
        {
            break;
        }
    }

    // check if network is ready to change local NMT state and this was not done before
    if ((nmtMnuInstance_g.flags & (EPL_NMTMNU_FLAG_HALTED | EPL_NMTMNU_FLAG_APP_INFORMED)) == 0)
    {   // boot process is not halted
        switch (nmtState)
        {
            case kEplNmtMsPreOperational1:
            {
                if ((nmtMnuInstance_g.signalSlaveCount == 0)
                    && (nmtMnuInstance_g.mandatorySlaveCount == 0))
                {   // all optional CNs scanned once and all mandatory CNs configured successfully
                    nmtMnuInstance_g.flags |= EPL_NMTMNU_FLAG_APP_INFORMED;
                    // inform application
                    ret = nmtMnuInstance_g.pfnCbBootEvent(kEplNmtBootEventBootStep1Finish,
                                                               nmtState,
                                                               EPL_E_NO_ERROR);
                    if (ret != kEplSuccessful)
                    {
                        if (ret == kEplReject)
                        {
                            // wait for application
                            ret = kEplSuccessful;
                        }
                        break;
                    }
                    // enter PreOp2
                    ret = EplNmtuNmtEvent(kEplNmtEventAllMandatoryCNIdent);
                }
                break;
            }

            case kEplNmtMsPreOperational2:
            {
                if ((nmtMnuInstance_g.signalSlaveCount == 0)
                    && (nmtMnuInstance_g.mandatorySlaveCount == 0))
                {   // all optional CNs checked once for ReadyToOp and all mandatory CNs are ReadyToOp
                    nmtMnuInstance_g.flags |= EPL_NMTMNU_FLAG_APP_INFORMED;
                    // inform application
                    ret = nmtMnuInstance_g.pfnCbBootEvent(kEplNmtBootEventBootStep2Finish,
                                                               nmtState,
                                                               EPL_E_NO_ERROR);
                    if (ret != kEplSuccessful)
                    {
                        if (ret == kEplReject)
                        {
                            // wait for application
                            ret = kEplSuccessful;
                        }
                        break;
                    }
                    // enter ReadyToOp
                    ret = EplNmtuNmtEvent(kEplNmtEventEnterReadyToOperate);
                }
                break;
            }

            case kEplNmtMsReadyToOperate:
            {
                if ((nmtMnuInstance_g.signalSlaveCount == 0)
                    && (nmtMnuInstance_g.mandatorySlaveCount == 0))
                {   // all CNs checked for errorless communication
                    nmtMnuInstance_g.flags |= EPL_NMTMNU_FLAG_APP_INFORMED;
                    // inform application
                    ret = nmtMnuInstance_g.pfnCbBootEvent(kEplNmtBootEventCheckComFinish,
                                                               nmtState,
                                                               EPL_E_NO_ERROR);
                    if (ret != kEplSuccessful)
                    {
                        if (ret == kEplReject)
                        {
                            // wait for application
                            ret = kEplSuccessful;
                        }
                        break;
                    }
                    // enter Operational
                    ret = EplNmtuNmtEvent(kEplNmtEventEnterMsOperational);
                }
                break;
            }

            case kEplNmtMsOperational:
            {
                if ((nmtMnuInstance_g.signalSlaveCount == 0)
                    && (nmtMnuInstance_g.mandatorySlaveCount == 0))
                {   // all optional CNs scanned once and all mandatory CNs are OPERATIONAL
                    nmtMnuInstance_g.flags |= EPL_NMTMNU_FLAG_APP_INFORMED;
                    // inform application
                    ret = nmtMnuInstance_g.pfnCbBootEvent(kEplNmtBootEventOperational,
                                                               nmtState,
                                                               EPL_E_NO_ERROR);
                    if (ret != kEplSuccessful)
                    {
                        if (ret == kEplReject)
                        {
                            // ignore error code
                            ret = kEplSuccessful;
                        }
                        break;
                    }
                }
                break;
            }

            default:
            {
                break;
            }
        }
    }

Exit:
    return ret;
}


//---------------------------------------------------------------------------
//
// Function:    EplNmtMnuCheckNmtState
//
// Description: checks the NMT state, i.e. evaluates it with object 0x1F8F
//              NMT_MNNodeExpState_AU8 and updates object 0x1F8E
//              NMT_MNNodeCurrState_AU8.
//              It manipulates nodeState in internal node info structure.
//
// Parameters:  nodeId_p              = node ID
//              nodeNmtState_p          = NMT state of CN
//
// Returns:     tEplKernel              = error code
//                  kEplReject          = CN was in wrong state and has been reset
//
// State:
//
//---------------------------------------------------------------------------

static tEplKernel EplNmtMnuCheckNmtState(
                                    UINT                nodeId_p,
                                    tNmtMnuNodeInfo* pNodeInfo_p,
                                    tEplNmtState        nodeNmtState_p,
                                    UINT16                errorCode_p,
                                    tEplNmtState        localNmtState_p)
{
tEplKernel      ret = kEplSuccessful;
tEplKernel      retUpdate = kEplSuccessful;
tEplObdSize     obdSize;
UINT8            nodeNmtState;
UINT8            bExpNmtState;
UINT8            nmtStatePrev;
tEplNmtState    expNmtState;

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
    ret = EplObduReadEntry(0x1F8F, nodeId_p, &bExpNmtState, &obdSize);
    if (ret != kEplSuccessful)
    {
        goto Exit;
    }

    // compute expected NMT state
    expNmtState = (tEplNmtState) (bExpNmtState | EPL_NMT_TYPE_CS);

    if (expNmtState == kEplNmtCsNotActive)
    {   // ignore the current state, because the CN shall be not active
        ret = kEplReject;
        goto ExitButUpdate;
    }
    else if ((expNmtState == kEplNmtCsStopped) &&
            (nodeNmtState_p == kEplNmtCsStopped))
    {
        // reset flags ISOCHRON and PREOP2_REACHED
        pNodeInfo_p->flags &= ~(EPL_NMTMNU_NODE_FLAG_ISOCHRON
                                 | EPL_NMTMNU_NODE_FLAG_PREOP2_REACHED);
    }
    else if ((expNmtState == kEplNmtCsPreOperational2) &&
             (nodeNmtState_p == kEplNmtCsPreOperational2))
    {   // CN is PreOp2
        if ((pNodeInfo_p->flags & EPL_NMTMNU_NODE_FLAG_PREOP2_REACHED) == 0)
        {   // CN switched to PreOp2
            pNodeInfo_p->flags |= EPL_NMTMNU_NODE_FLAG_PREOP2_REACHED;

            if (pNodeInfo_p->nodeCfg & EPL_NODEASSIGN_ASYNCONLY_NODE)
            {
                if ((pNodeInfo_p->nodeState == kNmtMnuNodeStateConfigured) &&
                    (localNmtState_p >= kEplNmtMsPreOperational2))
                {
                    ret = EplNmtMnuNodeBootStep2(nodeId_p, pNodeInfo_p);
                }
            }
            else
            {   // add node to isochronous phase
                ret = EplNmtMnuAddNodeIsochronous(nodeId_p);
                if (ret != kEplSuccessful)
                {
                    goto Exit;
                }
            }
        }
    }
    else if ((expNmtState == kEplNmtCsPreOperational2)
         && (nodeNmtState_p == kEplNmtCsReadyToOperate))
    {   // CN switched to ReadyToOp
        // delete timer for timeout handling
        ret = EplTimeruDeleteTimer(&pNodeInfo_p->timerHdlLonger);
        if (ret != kEplSuccessful)
        {
            goto Exit;
        }
        pNodeInfo_p->nodeState = kNmtMnuNodeStateReadyToOp;

        // update object 0x1F8F NMT_MNNodeExpState_AU8 to ReadyToOp
        ret = EplObduWriteEntry(0x1F8F, nodeId_p, &nodeNmtState, 1);
        if (ret != kEplSuccessful)
        {
            goto Exit;
        }

        if ((pNodeInfo_p->nodeCfg & EPL_NODEASSIGN_MANDATORY_CN) != 0)
        {   // node is a mandatory CN -> decrement counter
            nmtMnuInstance_g.mandatorySlaveCount--;
        }
        if (localNmtState_p >= kEplNmtMsReadyToOperate)
        {   // start procedure CheckCommunication for this node
            ret = EplNmtMnuNodeCheckCom(nodeId_p, pNodeInfo_p);
            if (ret != kEplSuccessful)
            {
                goto ExitButUpdate;
            }

            if ((localNmtState_p == kEplNmtMsOperational)
                && (pNodeInfo_p->nodeState == kNmtMnuNodeStateComChecked))
            {
                EPL_NMTMNU_DBG_POST_TRACE_VALUE(0,
                                                nodeId_p,
                                                (((nodeNmtState_p & 0xFF) << 8)
                                                | kEplNmtCmdStartNode));

                // immediately start optional CN, because communication is always OK (e.g. async-only CN)
                ret = nmtmnu_sendNmtCommand(nodeId_p, kEplNmtCmdStartNode);
                if (ret != kEplSuccessful)
                {
                    goto Exit;
                }
            }
        }

    }
    else if ((pNodeInfo_p->nodeState == kNmtMnuNodeStateComChecked)
             && (nodeNmtState_p == kEplNmtCsOperational))
    {   // CN switched to OPERATIONAL
        pNodeInfo_p->nodeState = kNmtMnuNodeStateOperational;

        if ((pNodeInfo_p->nodeCfg & EPL_NODEASSIGN_MANDATORY_CN) != 0)
        {   // node is a mandatory CN -> decrement counter
            nmtMnuInstance_g.mandatorySlaveCount--;
        }

    }
    else if (expNmtState != nodeNmtState_p)
    {   // CN is not in expected NMT state
    UINT16 wbeErrorCode;

        if (errorCode_p == 0)
        {   // assume wrong NMT state error
            if ((pNodeInfo_p->flags & EPL_NMTMNU_NODE_FLAG_NMT_CMD_ISSUED) != 0)
            {   // NMT command has been just issued;
                // ignore wrong NMT state until timer expires;
                // other errors like LOSS_PRES_TH are still processed
                goto Exit;
            }

            errorCode_p = EPL_E_NMT_WRONG_STATE;
        }

        if ((pNodeInfo_p->flags & EPL_NMTMNU_NODE_FLAG_NOT_SCANNED) != 0)
        {
            // decrement only signal slave count if checked once
            nmtMnuInstance_g.signalSlaveCount--;
            pNodeInfo_p->flags &= ~EPL_NMTMNU_NODE_FLAG_NOT_SCANNED;
        }

        // -> CN is in wrong NMT state
        pNodeInfo_p->nodeState = kNmtMnuNodeStateUnknown;

        BENCHMARK_MOD_07_TOGGLE(7);

        // $$$ start ERROR_TREATMENT and inform application
        ret = nmtMnuInstance_g.pfnCbNodeEvent(nodeId_p,
                                                   kEplNmtNodeEventError,
                                                   nodeNmtState_p,
                                                   errorCode_p,
                                                   (pNodeInfo_p->nodeCfg & EPL_NODEASSIGN_MANDATORY_CN) != 0);
        if (ret != kEplSuccessful)
        {
            goto ExitButUpdate;
        }

        EPL_NMTMNU_DBG_POST_TRACE_VALUE(0,
                                        nodeId_p,
                                        (((nodeNmtState_p & 0xFF) << 8)
                                        | kEplNmtCmdResetNode));

        // reset CN
        // store error code in NMT command data for diagnostic purpose
        AmiSetWordToLe(&wbeErrorCode, errorCode_p);
        ret = nmtmnu_sendNmtCommandEx(nodeId_p, kEplNmtCmdResetNode, &wbeErrorCode, sizeof (wbeErrorCode));
        if (ret == kEplSuccessful)
        {
            ret = kEplReject;
        }

        // d.k. continue with updating the current NMT state of the CN
//        goto Exit;
    }

ExitButUpdate:
    // check if NMT_MNNodeCurrState_AU8 has to be changed
    obdSize = 1;
    retUpdate = EplObduReadEntry(0x1F8E, nodeId_p, &nmtStatePrev, &obdSize);
    if (retUpdate != kEplSuccessful)
    {
        ret = retUpdate;
        goto Exit;
    }
    if (nodeNmtState != nmtStatePrev)
    {
        // update object 0x1F8E NMT_MNNodeCurrState_AU8
        retUpdate = EplObduWriteEntry(0x1F8E, nodeId_p, &nodeNmtState, 1);
        if (retUpdate != kEplSuccessful)
        {
            ret =retUpdate;
            goto Exit;
        }
        retUpdate = nmtMnuInstance_g.pfnCbNodeEvent(nodeId_p,
                                               kEplNmtNodeEventNmtState,
                                               nodeNmtState_p,
                                               errorCode_p,
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

//---------------------------------------------------------------------------
//
// Function:    EplNmtMnuReset
//
// Description: reset internal structures, e.g. timers
//
// Parameters:  void
//
// Returns:     tEplKernel              = error code
//
// State:
//
//---------------------------------------------------------------------------

static tEplKernel EplNmtMnuReset(void)
{
tEplKernel  ret;
UINT        index;

    ret = EplTimeruDeleteTimer(&nmtMnuInstance_g.timerHdlNmtState);

    for (index = 1; index <= tabentries (nmtMnuInstance_g.aNodeInfo); index++)
    {
        // delete timer handles
        ret = EplTimeruDeleteTimer(&EPL_NMTMNU_GET_NODEINFO(index)->timerHdlStatReq);
        ret = EplTimeruDeleteTimer(&EPL_NMTMNU_GET_NODEINFO(index)->timerHdlLonger);
    }

#if EPL_NMTMNU_PRES_CHAINING_MN != FALSE
    nmtMnuInstance_g.prcPResMnTimeoutNs = 0;
#endif

    return ret;
}


#if EPL_NMTMNU_PRES_CHAINING_MN != FALSE
//---------------------------------------------------------------------------
//
// Function:    EplNmtMnuPrcMeasure
//
// Description: Perform measure phase of PRC node insertion
//
// Parameters:  void
//
// Returns:     tEplKernel              = error code
//
// State:
//
//---------------------------------------------------------------------------

static tEplKernel EplNmtMnuPrcMeasure(void)
{
tEplKernel          ret;
UINT                nodeId;
tNmtMnuNodeInfo* pNodeInfo;
BOOL                fSyncReqSentToPredNode;
UINT        nodeIdFirstNode;
UINT        nodeIdPredNode;
UINT        nodeIdPrevSyncReq;

    ret = kEplSuccessful;

    fSyncReqSentToPredNode = FALSE;
    nodeIdPredNode       = EPL_C_ADR_INVALID;
    nodeIdPrevSyncReq    = EPL_C_ADR_INVALID;
    nodeIdFirstNode      = EPL_C_ADR_INVALID;

    for (nodeId = 1; nodeId < 254; nodeId++)
    {
        pNodeInfo = EPL_NMTMNU_GET_NODEINFO(nodeId);
        if (pNodeInfo == NULL)
        {
            continue;
        }

        if (   (pNodeInfo->nodeCfg & EPL_NODEASSIGN_PRES_CHAINING)
            && (   (pNodeInfo->flags & EPL_NMTMNU_NODE_FLAG_ISOCHRON)
                || (pNodeInfo->prcFlags & EPL_NMTMNU_NODE_FLAG_PRC_ADD_IN_PROGRESS)))
        {
            if (nodeIdFirstNode == EPL_C_ADR_INVALID)
            {
                if (pNodeInfo->prcFlags & EPL_NMTMNU_NODE_FLAG_PRC_ADD_IN_PROGRESS)
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
                tEplDllSyncRequest SyncRequestData;
                UINT               uiSize;

                    SyncRequestData.m_dwSyncControl = EPL_SYNC_DEST_MAC_ADDRESS_VALID;
                    uiSize = sizeof(UINT) + sizeof(UINT32);

                    if (fSyncReqSentToPredNode == FALSE)
                    {
                        SyncRequestData.m_uiNodeId = nodeIdPredNode;

                        ret = EplSyncuRequestSyncResponse(EplNmtMnuPrcCbSyncResMeasure, &SyncRequestData, uiSize);
                        if (ret != kEplSuccessful)
                        {
                            goto Exit;
                        }
                    }

                    SyncRequestData.m_uiNodeId = nodeId;

                    ret = EplSyncuRequestSyncResponse(EplNmtMnuPrcCbSyncResMeasure, &SyncRequestData, uiSize);
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
        pNodeInfo = EPL_NMTMNU_GET_NODEINFO(nodeIdPrevSyncReq);
        pNodeInfo->prcFlags |= EPL_NMTMNU_NODE_FLAG_PRC_CALL_MEASURE;
    }
    else
    {   // No SyncReq has been sent
        if (nodeIdFirstNode == EPL_C_ADR_INVALID)
        {   // No add-in-progress node has been found. This might happen
            // due to reset-node NMT commands which were issued
            // between the first and the second measure scan.
            nmtMnuInstance_g.flags &= ~EPL_NMTMNU_FLAG_PRC_ADD_IN_PROGRESS;

            // A new insertion process can be started
            ret = EplNmtMnuAddNodeIsochronous(EPL_C_ADR_INVALID);
        }
        else
        {
            // Prepare shift phase and add phase
            ret = EplNmtMnuPrcCalculate(nodeIdFirstNode);
        }
    }

Exit:
    return ret;
}


//---------------------------------------------------------------------------
//
// Function:    EplNmtMnuPrcCalculate
//
// Description: Update calculation of PRes Response Times (CNs) and
//              PRes Chaining Slot Time (MN).
//
// Parameters:  nodeIdFirstNode_p = Node ID of the first (lowest node ID)
//                                    of nodes whose addition is in progress
//
// Returns:     tEplKernel          = error code
//
// State:
//
//---------------------------------------------------------------------------

static tEplKernel EplNmtMnuPrcCalculate(UINT nodeIdFirstNode_p)
{
tEplKernel          ret;
UINT                nodeId;
tNmtMnuNodeInfo* pNodeInfo;
UINT                nodeIdPredNode;
UINT32              pResResponseTimeNs;
UINT32              pResMnTimeoutNs;

    if (   (nodeIdFirstNode_p == EPL_C_ADR_INVALID)
        || (nodeIdFirstNode_p >= EPL_C_ADR_BROADCAST))
    {   // invalid node ID specified
        ret = kEplInvalidNodeId;
        goto Exit;
    }

    nodeIdPredNode = EPL_C_ADR_INVALID;

    for (nodeId = nodeIdFirstNode_p; nodeId < 254; nodeId++)
    {
        pNodeInfo = EPL_NMTMNU_GET_NODEINFO(nodeId);
        if (pNodeInfo == NULL)
        {
            continue;
        }

        if (   (pNodeInfo->nodeCfg & EPL_NODEASSIGN_PRES_CHAINING)
            && (   (pNodeInfo->flags & EPL_NMTMNU_NODE_FLAG_ISOCHRON)
                || (pNodeInfo->prcFlags & EPL_NMTMNU_NODE_FLAG_PRC_ADD_IN_PROGRESS)))
        {
            ret = EplNmtMnuPrcCalcPResResponseTimeNs(nodeId, nodeIdPredNode, &pResResponseTimeNs);
            if (ret != kEplSuccessful)
            {
                goto Exit;
            }

            if (pNodeInfo->pResTimeFirstNs < pResResponseTimeNs)
            {
                pNodeInfo->pResTimeFirstNs = pResResponseTimeNs;

                if (pNodeInfo->flags & EPL_NMTMNU_NODE_FLAG_ISOCHRON)
                {
                    pNodeInfo->prcFlags |= EPL_NMTMNU_NODE_FLAG_PRC_SHIFT_REQUIRED;
                }
            }

            nodeIdPredNode = nodeId;
        }
    }

    ret = EplNmtMnuPrcCalcPResChainingSlotTimeNs(nodeIdPredNode, &pResMnTimeoutNs);
    if (ret != kEplSuccessful)
    {
        goto Exit;
    }

    if (nmtMnuInstance_g.prcPResMnTimeoutNs < pResMnTimeoutNs)
    {
    tEplDllNodeInfo DllNodeInfo;

        EPL_MEMSET(&DllNodeInfo, 0, sizeof(tEplDllNodeInfo));
        nmtMnuInstance_g.prcPResMnTimeoutNs = pResMnTimeoutNs;
        DllNodeInfo.m_dwPresTimeoutNs              = pResMnTimeoutNs;
        DllNodeInfo.m_uiNodeId                     = EPL_C_ADR_MN_DEF_NODE_ID;

        ret = dllucal_configNode(&DllNodeInfo);
        if (ret != kEplSuccessful)
        {
            goto Exit;
        }
    }

    // enter next phase
    ret = EplNmtMnuPrcShift(EPL_C_ADR_INVALID);

Exit:
    return ret;
}


//---------------------------------------------------------------------------
//
// Function:    EplNmtMnuPrcCalcPResResponseTimeNs
//
// Description: Calculate PRes Response Time of a node
//
// Parameters:  nodeId_p              = IN:  Node ID
//              nodeIdPredNode_p      = IN:  Node ID of the predecessor node
//              pPResResponseTimeNs_p = OUT: PRes Response Time in ns
//
// Returns:     tEplKernel              = error code
//
// State:
//
//---------------------------------------------------------------------------

static tEplKernel EplNmtMnuPrcCalcPResResponseTimeNs(
                                    UINT            nodeId_p,
                                    UINT            nodeIdPredNode_p,
                                    UINT32*          pPResResponseTimeNs_p)
{
tEplKernel          ret;
UINT16                pResPayloadLimitPredNode;
tNmtMnuNodeInfo* pNodeInfoPredNode;
tEplObdSize         obdSize;

    ret = kEplSuccessful;

    if (nodeIdPredNode_p == EPL_C_ADR_INVALID)
    {   // no predecessor node passed
        nodeIdPredNode_p = EplNmtMnuPrcFindPredecessorNode(nodeId_p);

        if (nodeIdPredNode_p == EPL_C_ADR_INVALID)
        {   // no predecessor node found
            // PRes Response Time of first PRC node is defined to 0
            *pPResResponseTimeNs_p = 0;
            goto Exit;
        }
    }

    pNodeInfoPredNode = EPL_NMTMNU_GET_NODEINFO(nodeIdPredNode_p);

    // read object 0x1F8D NMT_PResPayloadLimitList_AU16
    obdSize = 2;
    ret = EplObduReadEntry(0x1F8D, nodeIdPredNode_p, &pResPayloadLimitPredNode, &obdSize);
    if (ret != kEplSuccessful)
    {
        goto Exit;
    }

    *pPResResponseTimeNs_p =
          // PRes Response Time of predecessor node
          pNodeInfoPredNode->pResTimeFirstNs
          // Transmission time for PRes frame of predecessor node
        + (8 * EPL_C_DLL_T_BITTIME * (pResPayloadLimitPredNode
                                      + EPL_C_DLL_T_EPL_PDO_HEADER
                                      + EPL_C_DLL_T_ETH2_WRAPPER)
           + EPL_C_DLL_T_PREAMBLE)
          // Relative propragation delay from predecessor node to addressed node
        + EPL_NMTMNU_GET_NODEINFO(nodeId_p)->relPropagationDelayNs
          // Time correction (hub jitter and part of measurement inaccuracy)
        + nmtMnuInstance_g.prcPResTimeFirstCorrectionNs;

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


//---------------------------------------------------------------------------
//
// Function:    EplNmtMnuPrcCalcPResChainingSlotTimeNs
//
// Description: Calculate PRes Chaining Slot Time
//
// Parameters:  pPResChainingSlotTimeNs_p = OUT: PRes Chaining Slot Time in ns
//
// Returns:     tEplKernel                  = error code
//
// State:
//
//---------------------------------------------------------------------------

static tEplKernel EplNmtMnuPrcCalcPResChainingSlotTimeNs(
                                    UINT            nodeIdLastNode_p,
                                    UINT32*          pPResChainingSlotTimeNs_p)
{
tEplKernel  ret;
UINT16        pResActPayloadLimit;
UINT16        cnPReqPayloadLastNode;
UINT32      cnResTimeoutLastNodeNs;
tEplObdSize obdSize;

    // read object 0x1F98 NMT_CycleTiming_REC
    // Sub-Index 05h PResActPayloadLimit_U16
    obdSize = 2;
    ret = EplObduReadEntry(0x1F98, 5, &pResActPayloadLimit, &obdSize);
    if (ret != kEplSuccessful)
    {
        goto Exit;
    }

    // read object 0x1F8B NMT_MNPReqPayloadLimitList_AU16
    obdSize = 2;
    ret = EplObduReadEntry(0x1F8B, nodeIdLastNode_p, &cnPReqPayloadLastNode, &obdSize);
    if (ret != kEplSuccessful)
    {
        goto Exit;
    }

    // read object 0x1F92 NMT_MNCNPResTimeout_AU32
    obdSize = 4;
    ret = EplObduReadEntry(0x1F92, nodeIdLastNode_p, &cnResTimeoutLastNodeNs, &obdSize);
    if (ret != kEplSuccessful)
    {
        goto Exit;
    }

    *pPResChainingSlotTimeNs_p =
          // Transmission time for PResMN frame
          (8 * EPL_C_DLL_T_BITTIME * (pResActPayloadLimit
                                      + EPL_C_DLL_T_EPL_PDO_HEADER
                                      + EPL_C_DLL_T_ETH2_WRAPPER)
           + EPL_C_DLL_T_PREAMBLE)
          // PRes Response Time of last node
        + EPL_NMTMNU_GET_NODEINFO(nodeIdLastNode_p)->pResTimeFirstNs
          // Relative propagation delay from last node to MN
          // Due to Soft-MN limitations, NMT_MNCNPResTimeout_AU32.CNResTimeout
          // of the last node is used.
        + cnResTimeoutLastNodeNs
          // Transmission time for PReq frame of last node
        - (8 * EPL_C_DLL_T_BITTIME * (cnPReqPayloadLastNode
                                      + EPL_C_DLL_T_EPL_PDO_HEADER
                                      + EPL_C_DLL_T_ETH2_WRAPPER)
           + EPL_C_DLL_T_PREAMBLE)
          // Time correction (hub jitter and part of measurement inaccuracy)
        + nmtMnuInstance_g.prcPResTimeFirstCorrectionNs;

Exit:
    return ret;
}


//---------------------------------------------------------------------------
//
// Function:    EplNmtMnuPrcFindPredecessorNode
//
// Description: Find the predecessor of the addressed node.
//              The function processes only PRC nodes which are added
//              to the isochronous phase or whose addition is in progress.
//
// Parameters:  nodeId_p          = Node ID of addressed node
//
// Returns:     UINT        = Node ID of the predecessor node
//                                    EPL_C_ADR_INVALID if no node was found
//
// State:
//
//---------------------------------------------------------------------------

static tEplKernel EplNmtMnuPrcFindPredecessorNode(UINT nodeId_p)
{
UINT                nodeId;
tNmtMnuNodeInfo* pNodeInfo;

    for (nodeId = nodeId_p - 1; nodeId >= 1; nodeId--)
    {
        pNodeInfo = EPL_NMTMNU_GET_NODEINFO(nodeId);
        if (pNodeInfo == NULL)
        {
            continue;
        }

        if (   (pNodeInfo->nodeCfg & EPL_NODEASSIGN_PRES_CHAINING)
            && (   (pNodeInfo->flags & EPL_NMTMNU_NODE_FLAG_ISOCHRON)
                || (pNodeInfo->prcFlags & EPL_NMTMNU_NODE_FLAG_PRC_ADD_IN_PROGRESS)))
        {
            break;
        }
    }

    return nodeId;
}


//---------------------------------------------------------------------------
//
// Function:    EplNmtMnuPrcCbSyncResMeasure
//
// Description: SyncRes call-back function after SyncReq for measurement
//
// Parameters:  nodeId_p              = Source node ID
//              pSyncResponse_p         = Pointer to payload of SyncRes frame
//
// Returns:     tEplKernel              = error code
//
// State:
//
//---------------------------------------------------------------------------

static tEplKernel PUBLIC EplNmtMnuPrcCbSyncResMeasure(
                                  UINT                  nodeId_p,
                                  tEplSyncResponse*     pSyncResponse_p)
{
tEplKernel          ret;
UINT                nodeIdPredNode;
tNmtMnuNodeInfo* pNodeInfo;
UINT32              syncNodeNumber;

    pNodeInfo = EPL_NMTMNU_GET_NODEINFO(nodeId_p);

    if (pSyncResponse_p == NULL)
    {   // SyncRes not received
        EplNmtMnuPrcSyncError(pNodeInfo);
        goto Exit;
    }

    if (pNodeInfo->relPropagationDelayNs != 0)
    {   // Relative propagation delay is already present
        goto Exit;
    }

    nodeIdPredNode = EplNmtMnuPrcFindPredecessorNode(nodeId_p);
    syncNodeNumber = AmiGetDwordFromLe(&pSyncResponse_p->m_le_dwSyncNodeNumber);

    if (syncNodeNumber != nodeIdPredNode)
    {   // SyncNodeNumber does not match predecessor node
        EplNmtMnuPrcSyncError(pNodeInfo);
        goto Exit;
    }

    pNodeInfo->relPropagationDelayNs = AmiGetDwordFromLe(&pSyncResponse_p->m_le_dwSyncDelay);

    // If a previous SyncRes frame was not usable,
    // the Sync Error flag is cleared as this one is OK
    pNodeInfo->prcFlags &= ~EPL_NMTMNU_NODE_FLAG_PRC_SYNC_ERR;

Exit:
    ret = EplNmtMnuPrcCbSyncResNextAction(nodeId_p, pSyncResponse_p);

    return ret;
}


//---------------------------------------------------------------------------
//
// Function:    EplNmtMnuPrcSyncError
//
// Description: Sets the Sync Error flag and schedules reset node
//              if required.
//
// Parameters:  pNodeInfo_p             = Pointer to NodeInfo
//
// Returns:     (none)
//
// State:
//
//---------------------------------------------------------------------------

static void EplNmtMnuPrcSyncError(tNmtMnuNodeInfo* pNodeInfo_p)
{
    if (pNodeInfo_p->prcFlags & EPL_NMTMNU_NODE_FLAG_PRC_SYNC_ERR)
    {   // Sync Error flag already set
        // Schedule reset node
        pNodeInfo_p->prcFlags &= ~EPL_NMTMNU_NODE_FLAG_PRC_RESET_MASK;
        pNodeInfo_p->prcFlags |= EPL_NMTMNU_NODE_FLAG_PRC_RESET_NODE;
    }
    else
    {   // Set Sync Error flag
        pNodeInfo_p->prcFlags |= EPL_NMTMNU_NODE_FLAG_PRC_SYNC_ERR;
    }
}


//---------------------------------------------------------------------------
//
// Function:    EplNmtMnuPrcShift
//
// Description: Perform shift phase of PRC node insertion
//
// Parameters:  nodeIdPrevShift_p     = Node ID of previously shifted node
//
// Returns:     tEplKernel              = error code
//
// State:
//
//---------------------------------------------------------------------------

static tEplKernel EplNmtMnuPrcShift(UINT nodeIdPrevShift_p)
{
tEplKernel          ret;
UINT                nodeId;
tNmtMnuNodeInfo* pNodeInfo;
tEplDllSyncRequest  syncRequestData;
UINT                size;

    ret = kEplSuccessful;

    if (nodeIdPrevShift_p == EPL_C_ADR_INVALID)
    {
        nodeIdPrevShift_p = 254;
    }

    // The search starts with the previous shift node
    // as this node might require a second SyncReq
    nodeId = nodeIdPrevShift_p;
    do
    {
        pNodeInfo = EPL_NMTMNU_GET_NODEINFO(nodeId);
        if (pNodeInfo == NULL)
        {
            continue;
        }

        if (   (pNodeInfo->nodeCfg & EPL_NODEASSIGN_PRES_CHAINING)
            && (   (pNodeInfo->flags & EPL_NMTMNU_NODE_FLAG_ISOCHRON)
                || (pNodeInfo->prcFlags & EPL_NMTMNU_NODE_FLAG_PRC_ADD_IN_PROGRESS)))
        {
            if (pNodeInfo->prcFlags & EPL_NMTMNU_NODE_FLAG_PRC_SHIFT_REQUIRED)
            {
                break;
            }
        }

        nodeId--;
    }
    while (nodeId >= 1);

    if (nodeId == 0)
    {   // No node requires shifting
        // Enter next phase
        ret = EplNmtMnuPrcAdd(EPL_C_ADR_INVALID);
        goto Exit;
    }

    // Call shift on reception of SyncRes
    pNodeInfo->prcFlags |= EPL_NMTMNU_NODE_FLAG_PRC_CALL_SHIFT;

    // Send SyncReq
    syncRequestData.m_uiNodeId        = nodeId;
    syncRequestData.m_dwSyncControl   = EPL_SYNC_PRES_TIME_FIRST_VALID |
                                        EPL_SYNC_DEST_MAC_ADDRESS_VALID;
    syncRequestData.m_dwPResTimeFirst = pNodeInfo->pResTimeFirstNs;
    size = sizeof(UINT) + 2*sizeof(UINT32);

    ret = EplSyncuRequestSyncResponse(EplNmtMnuPrcCbSyncResShift, &syncRequestData, size);

Exit:
    return ret;
}


//---------------------------------------------------------------------------
//
// Function:    EplNmtMnuPrcCbSyncResShift
//
// Description: SyncRes call-back function after SyncReq for shifting
//
// Parameters:  nodeId_p              = Source node ID
//              pSyncResponse_p         = Pointer to payload of SyncRes frame
//
// Returns:     tEplKernel              = error code
//
// State:
//
//---------------------------------------------------------------------------

static tEplKernel PUBLIC EplNmtMnuPrcCbSyncResShift(
                                  UINT                  nodeId_p,
                                  tEplSyncResponse*     pSyncResponse_p)
{
tEplKernel          ret;
tNmtMnuNodeInfo* pNodeInfo;

    pNodeInfo = EPL_NMTMNU_GET_NODEINFO(nodeId_p);

    if (pSyncResponse_p == NULL)
    {   // SyncRes not received
        EplNmtMnuPrcSyncError(pNodeInfo);
        goto Exit;
    }

    pNodeInfo->prcFlags &= ~EPL_NMTMNU_NODE_FLAG_PRC_SHIFT_REQUIRED;

    // If a previous SyncRes frame was not usable,
    // the Sync Error flag is cleared as this one is OK
    pNodeInfo->prcFlags &= ~EPL_NMTMNU_NODE_FLAG_PRC_SYNC_ERR;

    // Schedule verify
    pNodeInfo->prcFlags |= EPL_NMTMNU_NODE_FLAG_PRC_VERIFY;

Exit:
    ret = EplNmtMnuPrcCbSyncResNextAction(nodeId_p, pSyncResponse_p);

    return ret;
}


//---------------------------------------------------------------------------
//
// Function:    EplNmtMnuPrcAdd
//
// Description: Perform add phase of PRC node insertion
//
// Parameters:  nodeIdPrevAdd_p       = Node ID of previously added node
//
// Returns:     tEplKernel              = error code
//
// State:
//
//---------------------------------------------------------------------------

static tEplKernel EplNmtMnuPrcAdd(UINT nodeIdPrevAdd_p)
{
tEplKernel          ret;
tEplObdSize         obdSize;
UINT32              cycleLenUs;
UINT32              cNLossOfSocToleranceNs;
UINT                nodeId;
tNmtMnuNodeInfo* pNodeInfo;
tEplDllSyncRequest  syncReqData;
UINT                syncReqNum;
tNmtMnuNodeInfo* pNodeInfoLastSyncReq;

    ret = kEplSuccessful;

    // prepare SyncReq
    syncReqData.m_dwSyncControl = EPL_SYNC_PRES_MODE_SET |
                                  EPL_SYNC_PRES_TIME_FIRST_VALID |
                                  EPL_SYNC_PRES_FALL_BACK_TIMEOUT_VALID |
                                  EPL_SYNC_DEST_MAC_ADDRESS_VALID;

    // read object 0x1006 NMT_CycleLen_U32
    obdSize = sizeof(UINT32);
    ret = EplObduReadEntry(0x1006, 0, &cycleLenUs, &obdSize);
    if (ret != kEplSuccessful)
    {
        goto Exit;
    }

    // read object 0x1C14 DLL_CNLossOfSocTolerance_U32
    ret = EplObduReadEntry(0x1C14, 0, &cNLossOfSocToleranceNs, &obdSize);
    if (ret != kEplSuccessful)
    {
        goto Exit;
    }

    syncReqData.m_dwPResFallBackTimeout = cycleLenUs * 1000
                                          + cNLossOfSocToleranceNs;

    syncReqNum = 0;
    pNodeInfoLastSyncReq = NULL;

    // The search starts with the next node after the previous one
    for (nodeId = nodeIdPrevAdd_p + 1; nodeId <= 254; nodeId++)
    {
        pNodeInfo = EPL_NMTMNU_GET_NODEINFO(nodeId);
        if (pNodeInfo == NULL)
        {
            continue;
        }

        if (pNodeInfo->prcFlags & EPL_NMTMNU_NODE_FLAG_PRC_ADD_IN_PROGRESS)
        {
            // Send SyncReq which starts PRes Chaining
            syncReqData.m_uiNodeId        = nodeId;
            syncReqData.m_dwPResTimeFirst = pNodeInfo->pResTimeFirstNs;

            ret = EplSyncuRequestSyncResponse(EplNmtMnuPrcCbSyncResAdd,
                                              &syncReqData,
                                              sizeof(syncReqData));
            if (ret != kEplSuccessful)
            {
                goto Exit;
            }

            pNodeInfo->prcFlags |= EPL_NMTMNU_NODE_FLAG_PRC_ADD_SYNCREQ_SENT;

            syncReqNum++;
            pNodeInfoLastSyncReq = pNodeInfo;

            if (syncReqNum == EPL_NMTMNU_PRC_NODE_ADD_MAX_NUM)
            {
                break;
            }
        }
    }

    if (pNodeInfoLastSyncReq != NULL)
    {
        pNodeInfoLastSyncReq->prcFlags |= EPL_NMTMNU_NODE_FLAG_PRC_CALL_ADD;
    }
    else
    {   // No nodes need to be added to the isochronous phase
        if (nodeIdPrevAdd_p != EPL_C_ADR_INVALID)
        {
            pNodeInfo = EPL_NMTMNU_GET_NODEINFO(nodeIdPrevAdd_p);

            if (pNodeInfo->prcFlags & EPL_NMTMNU_NODE_FLAG_PRC_VERIFY)
            {   // Verification of the last node is still in progress
                // Wait for verify and try again
                pNodeInfo->prcFlags |= EPL_NMTMNU_NODE_FLAG_PRC_CALL_ADD;
                goto Exit;
            }
        }

        // Eigher no nodes had to be added, at all, or add is finished
        nmtMnuInstance_g.flags &= ~EPL_NMTMNU_FLAG_PRC_ADD_IN_PROGRESS;

        // A new insertion process can be started
        ret = EplNmtMnuAddNodeIsochronous(EPL_C_ADR_INVALID);
    }

Exit:
    return ret;
}


//---------------------------------------------------------------------------
//
// Function:    EplNmtMnuPrcCbSyncResAdd
//
// Description: SyncRes call-back function after SyncReq for insertion
//
// Parameters:  nodeId_p              = Source node ID
//              pSyncResponse_p         = Pointer to payload of SyncRes frame
//
// Returns:     tEplKernel              = error code
//
// State:
//
//---------------------------------------------------------------------------

static tEplKernel PUBLIC EplNmtMnuPrcCbSyncResAdd(
                                  UINT                  nodeId_p,
                                  tEplSyncResponse*     pSyncResponse_p)
{
tEplKernel          ret;
tNmtMnuNodeInfo* pNodeInfo;

    pNodeInfo = EPL_NMTMNU_GET_NODEINFO(nodeId_p);

    if (pSyncResponse_p == NULL)
    {   // SyncRes not received
        // Immediately, schedule reset node
        // because node has already been added to isochronous phase in module Dllk
        pNodeInfo->prcFlags &= ~EPL_NMTMNU_NODE_FLAG_PRC_RESET_MASK;
        pNodeInfo->prcFlags |= EPL_NMTMNU_NODE_FLAG_PRC_RESET_NODE;

        pNodeInfo->prcFlags &= ~EPL_NMTMNU_NODE_FLAG_PRC_ADD_SYNCREQ_SENT;
        goto NextAction;
    }

    // No additional node-added event is received for PRC nodes
    // thus the handler has to be called manually.
    ret = EplNmtMnuCbNodeAdded(nodeId_p);
    if (ret != kEplSuccessful)
    {
        goto Exit;
    }

    // Flag ISOCHRON has been set in EplNmtMnuCbNodeAdded,
    // thus this flags are reset.
    pNodeInfo->prcFlags &= ~(EPL_NMTMNU_NODE_FLAG_PRC_ADD_IN_PROGRESS |
                                EPL_NMTMNU_NODE_FLAG_PRC_ADD_SYNCREQ_SENT);

    // Schedule verify
    pNodeInfo->prcFlags |= EPL_NMTMNU_NODE_FLAG_PRC_VERIFY;

NextAction:
    ret = EplNmtMnuPrcCbSyncResNextAction(nodeId_p, pSyncResponse_p);

Exit:
    return ret;
}


//---------------------------------------------------------------------------
//
// Function:    EplNmtMnuPrcVerify
//
// Description: Perform verify for phase shift and phase add
//
// Parameters:  void
//
// Returns:     tEplKernel              = error code
//
// State:
//
//---------------------------------------------------------------------------

static tEplKernel EplNmtMnuPrcVerify(UINT nodeId_p)
{
tEplKernel          ret;
tNmtMnuNodeInfo* pNodeInfo;
tEplDllSyncRequest  syncReqData;
UINT                size;

    ret = kEplSuccessful;

    pNodeInfo = EPL_NMTMNU_GET_NODEINFO(nodeId_p);

    if (pNodeInfo->flags & EPL_NMTMNU_NODE_FLAG_ISOCHRON)
    {
        syncReqData.m_uiNodeId      = nodeId_p;
        syncReqData.m_dwSyncControl = EPL_SYNC_DEST_MAC_ADDRESS_VALID;
        size = sizeof(UINT) + sizeof(UINT32);

        ret = EplSyncuRequestSyncResponse(EplNmtMnuPrcCbSyncResVerify, &syncReqData, size);
    }
    else
    {   // Node has been removed by a reset-node NMT command
        // Verification is no longer necessary
        pNodeInfo->prcFlags &= ~EPL_NMTMNU_NODE_FLAG_PRC_VERIFY;
    }

    return ret;
}


//---------------------------------------------------------------------------
//
// Function:    EplNmtMnuPrcCbSyncResVerify
//
// Description: SyncRes call-back function after SyncReq for verification
//
// Parameters:  nodeId_p              = Source node ID
//              pSyncResponse_p         = Pointer to payload of SyncRes frame
//
// Returns:     tEplKernel              = error code
//
// State:
//
//---------------------------------------------------------------------------

static tEplKernel PUBLIC EplNmtMnuPrcCbSyncResVerify(
                                  UINT                  nodeId_p,
                                  tEplSyncResponse*     pSyncResponse_p)
{
tEplKernel          ret;
tNmtMnuNodeInfo* pNodeInfo;
UINT32              pResTimeFirstNs;

    pNodeInfo = EPL_NMTMNU_GET_NODEINFO(nodeId_p);

    if (pSyncResponse_p == NULL)
    {   // SyncRes not received
        EplNmtMnuPrcSyncError(pNodeInfo);
        goto Exit;
    }

    pResTimeFirstNs = AmiGetDwordFromLe(&pSyncResponse_p->m_le_dwPResTimeFirst);

    if (pResTimeFirstNs != pNodeInfo->pResTimeFirstNs)
    {   // Configuration of PRes Response Time was not successful
        // Schedule reset node
        pNodeInfo->prcFlags &= ~EPL_NMTMNU_NODE_FLAG_PRC_RESET_MASK;
        pNodeInfo->prcFlags |= EPL_NMTMNU_NODE_FLAG_PRC_RESET_NODE;
        goto Exit;
    }

    // Verification was successful
    pNodeInfo->prcFlags &= ~EPL_NMTMNU_NODE_FLAG_PRC_VERIFY;

    // If a previous SyncRes frame was not usable,
    // the Sync Error flag is cleared as this one is OK
    pNodeInfo->prcFlags &= ~EPL_NMTMNU_NODE_FLAG_PRC_SYNC_ERR;

Exit:
    ret = EplNmtMnuPrcCbSyncResNextAction(nodeId_p, pSyncResponse_p);

    return ret;
}


//---------------------------------------------------------------------------
//
// Function:    EplNmtMnuPrcCbSyncResNextAction
//
// Description: SyncRes call-back function if no specific handling is required.
//              The next-action node flags are evaluated.
//
// Parameters:  nodeId_p              = Source node ID
//              pSyncResponse_p         = Pointer to payload of SyncRes frame
//
// Returns:     tEplKernel              = error code
//
// State:
//
//---------------------------------------------------------------------------

static tEplKernel PUBLIC EplNmtMnuPrcCbSyncResNextAction(
                                  UINT                  nodeId_p,
                                  tEplSyncResponse*     pSyncResponse_p)
{
tEplKernel          ret;
tNmtMnuNodeInfo* pNodeInfo;
tEplNmtCommand      nmtCommand;

    UNUSED_PARAMETER(pSyncResponse_p);
    ret = kEplSuccessful;

    pNodeInfo = EPL_NMTMNU_GET_NODEINFO(nodeId_p);

    switch (pNodeInfo->prcFlags & EPL_NMTMNU_NODE_FLAG_PRC_RESET_MASK)
    {
        case EPL_NMTMNU_NODE_FLAG_PRC_STOP_NODE:
        {
            nmtCommand = kEplNmtCmdStopNode;
            break;
        }

        case EPL_NMTMNU_NODE_FLAG_PRC_RESET_NODE:
        {
            nmtCommand = kEplNmtCmdResetNode;
            break;
        }

        case EPL_NMTMNU_NODE_FLAG_PRC_RESET_COM:
        {
            nmtCommand = kEplNmtCmdResetCommunication;
            break;
        }

        case EPL_NMTMNU_NODE_FLAG_PRC_RESET_CONF:
        {
            nmtCommand = kEplNmtCmdResetConfiguration;
            break;
        }

        case EPL_NMTMNU_NODE_FLAG_PRC_RESET_SW:
        {
            nmtCommand = kEplNmtCmdSwReset;
            break;
        }

        default:
        {
            nmtCommand = kEplNmtCmdInvalidService;
            break;
        }
    }

    pNodeInfo->prcFlags &= ~EPL_NMTMNU_NODE_FLAG_PRC_RESET_MASK;

    if (nmtCommand != kEplNmtCmdInvalidService)
    {
        ret = nmtmnu_sendNmtCommand(nodeId_p, nmtCommand);
        if (ret != kEplSuccessful)
        {
            goto Exit;
        }
    }

    switch (pNodeInfo->prcFlags & EPL_NMTMNU_NODE_FLAG_PRC_CALL_MASK)
    {
        case EPL_NMTMNU_NODE_FLAG_PRC_CALL_MEASURE:
        {
            pNodeInfo->prcFlags &= ~EPL_NMTMNU_NODE_FLAG_PRC_CALL_MASK;
            ret = EplNmtMnuPrcMeasure();
            goto Exit;
        }

        case EPL_NMTMNU_NODE_FLAG_PRC_CALL_SHIFT:
        {
            pNodeInfo->prcFlags &= ~EPL_NMTMNU_NODE_FLAG_PRC_CALL_MASK;
            ret = EplNmtMnuPrcShift(nodeId_p);
            if (ret != kEplSuccessful)
            {
                goto Exit;
            }
            break;
        }

        case EPL_NMTMNU_NODE_FLAG_PRC_CALL_ADD:
        {
            pNodeInfo->prcFlags &= ~EPL_NMTMNU_NODE_FLAG_PRC_CALL_MASK;
            ret = EplNmtMnuPrcAdd(nodeId_p);
            if (ret != kEplSuccessful)
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

    if (pNodeInfo->prcFlags & EPL_NMTMNU_NODE_FLAG_PRC_VERIFY)
    {
        ret = EplNmtMnuPrcVerify(nodeId_p);
    }

Exit:
    return ret;
}


//---------------------------------------------------------------------------
//
// Function:    EplNmtMnuPrcSetResetFlags
//
// Description: Before sending a reset-node NMT command PRes Chaining has to
//              be disabled by sending an appropriate SyncReq. The requested
//              NMT command is stored until the SyncRes returns.
//              Commands of higher priority overwrite those of lower priority.
//
// Parameters:  pNodeInfo_p             = Pointer to NodeInfo structure
//                                        of the addressed node
//              NmtCommand_p            = NMT command
//
// Returns:     (none)
//
// State:
//
//---------------------------------------------------------------------------

static void EplNmtMnuPrcSetFlagsNmtCommandReset(
                                        tNmtMnuNodeInfo* pNodeInfo_p,
                                        tEplNmtCommand      NmtCommand_p)
{
UINT16 prcFlagsReset;

    prcFlagsReset = pNodeInfo_p->prcFlags & EPL_NMTMNU_NODE_FLAG_PRC_RESET_MASK;

    switch (NmtCommand_p)
    {
        case kEplNmtCmdResetNode:
        {
            prcFlagsReset = EPL_NMTMNU_NODE_FLAG_PRC_RESET_NODE;
            break;
        }
        case kEplNmtCmdResetCommunication:
        {
            switch (prcFlagsReset)
            {
                case EPL_NMTMNU_NODE_FLAG_PRC_RESET_CONF:
                case EPL_NMTMNU_NODE_FLAG_PRC_RESET_SW:
                case EPL_NMTMNU_NODE_FLAG_PRC_STOP_NODE:
                case 0:
                {
                    prcFlagsReset = EPL_NMTMNU_NODE_FLAG_PRC_RESET_COM;
                }
                default:
                {
                    break;
                }
            }
            break;
        }
        case kEplNmtCmdResetConfiguration:
        {
            switch (prcFlagsReset)
            {
                case EPL_NMTMNU_NODE_FLAG_PRC_RESET_SW:
                case EPL_NMTMNU_NODE_FLAG_PRC_STOP_NODE:
                case 0:
                {
                    prcFlagsReset = EPL_NMTMNU_NODE_FLAG_PRC_RESET_CONF;
                }
                default:
                {
                    break;
                }
            }
            break;
        }
        case kEplNmtCmdSwReset:
        {
            switch (prcFlagsReset)
            {
                case EPL_NMTMNU_NODE_FLAG_PRC_STOP_NODE:
                case 0:
                {
                    prcFlagsReset = EPL_NMTMNU_NODE_FLAG_PRC_RESET_SW;
                }
                default:
                {
                    break;
                }
            }
            break;
        }
        case kEplNmtCmdStopNode:
        {
            if (prcFlagsReset == 0)
            {
                prcFlagsReset = EPL_NMTMNU_NODE_FLAG_PRC_STOP_NODE;
            }
            break;
        }
        default:
        {
            break;
        }
    }

    pNodeInfo_p->prcFlags &= ~EPL_NMTMNU_NODE_FLAG_PRC_RESET_MASK;
    pNodeInfo_p->prcFlags |= prcFlagsReset;

    return;
}
#endif // #if EPL_NMTMNU_PRES_CHAINING_MN != FALSE

#endif // #if(((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)

// EOF

