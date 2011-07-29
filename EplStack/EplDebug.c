/*******************************************************************************

  File:         EplDebug.c

  (c) Bernecker + Rainer Ges.m.b.H.,  B&R Strasse 1, 5142 Eggelsberg, Austria
      www.br-automation.com

  Project:      openPOWERLINK

  Author:       Josef Baumgartner

  Description:  additional openPOWERLINK debugging functions

  License:

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    1. Redistributions of source code must retain the above copyright
       notice, this list of conditions and the following disclaimer.

    2. Redistributions in binary form must reproduce the above copyright
       notice, this list of conditions and the following disclaimer in the
       documentation and/or other materials provided with the distribution.

    3. Neither the name of Bernecker + Rainer Ges.m.b.H nor the names of its
       contributors may be used to endorse or promote products derived
       from this software without prior written permission. For written
       permission, please contact office@br-automation.com.

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

*******************************************************************************/


//=========================================================================//
// Includes                                                                //
//=========================================================================//
#include "Epl.h"
#include "EplNmt.h"
#include <stdlib.h>

//=========================================================================//
// Type definitions                                                        //
//=========================================================================//
typedef struct {
    tEplNmtState        m_nmtState;
    char                *m_sNmtState;
} tNmtStateInfo;

typedef struct
{
    tEplApiEventType    m_ApiEvent;
    char                *m_sApiEvent;
} tApiEventInfo;

static char *eplInvalidStr_g = "INVALID";

//---------------------------------------------------------------------------
// module global vars
//---------------------------------------------------------------------------
/* text strings for POWERLINK events */
static char *eplEvtStr_g[] = {
    "NmtEventNoEvent",
    "NmtEventDllMePres",
    "NmtEventDllMePresTimeout",
    "NmtEventDllMeAsndTimeout",
    "NmtEventDllMeSoaSent",
    "NmtEventDllMeSocTrig",
    "NmtEventDllMeSoaTrig",
    "NmtEventDllCeSoc",
    "NmtEventDllCePreq",
    "NmtEventDllCePres",
    "NmtEventDllCeSoa",
    "NmtEventDllCeAsnd",
    "NmtEventDllCeFrameTimeout",
    "0xd",
    "0xe",
    "0xf",
    "NmtEventSwReset", // NMT_GT1, NMT_GT2, NMT_GT8
    "NmtEventResetNode",
    "NmtEventResetCom",
    "NmtEventResetConfig",
    "NmtEventEnterPreOperational2",
    "NmtEventEnableReadyToOperate",
    "NmtEventStartNode", // NMT_CT7
    "NmtEventStopNode",
    "0x18",
    "0x19",
    "0x1A",
    "0x1B",
    "0x1C",
    "0x1D",
    "0x1E",
    "0x1F",
    "NmtEventEnterResetApp",
    "NmtEventEnterResetCom",
    "NmtEventInternComError", // NMT_GT6, internal communication error -> enter ResetCommunication
    "NmtEventEnterResetConfig",
    "NmtEventEnterCsNotActive",
    "NmtEventEnterMsNotActive",
    "NmtEventTimerBasicEthernet", // NMT_CT3; timer triggered state change (NotActive -> BasicEth)
    "NmtEventTimerMsPreOp1", // enter PreOp1 on MN (NotActive -> MsPreOp1)
    "NmtEventNmtCycleError", // NMT_CT11, NMT_MT6; error during cycle -> enter PreOp1
    "NmtEventTimerMsPreOp2", // enter PreOp2 on MN (MsPreOp1 -> MsPreOp2 if NmtEventAllMandatoryCNIdent)
    "NmtEventAllMandatoryCNIdent", // enter PreOp2 on MN if NmtEventTimerMsPreOp2
    "NmtEventEnterReadyToOperate", // application ready for the state ReadyToOp
    "NmtEventEnterMsOperational", // enter Operational on MN
    "NmtEventSwitchOff", // enter state Off
    "NmtEventCriticalError", // enter state Off because of critical error
};
unsigned int uiNumEplEvtStr_g = (sizeof(eplEvtStr_g) / sizeof(*(eplEvtStr_g)));

/* text strings for POWERLINK event sources */
static char    *eplEvtSrcStr_g[] = {
    "0",
    // kernelspace modules
    "EventSourceDllk", // Dllk module
    "EventSourceNmtk", // Nmtk module
    "EventSourceObdk", // Obdk module
    "EventSourcePdok", // Pdok module
    "EventSourceTimerk", // Timerk module
    "EventSourceEventk", // Eventk module
    "EventSourceSyncCb", // sync-Cb
    "EventSourceErrk", // Error handler module
    "0x09", "0x0a", "0x0b", "0x0c", "0x0d", "0x0e", "0x0f",
    // userspace modules
    "EventSourceDllu", // Dllu module
    "EventSourceNmtu", // Nmtu module
    "EventSourceNmtCnu", // NmtCnu module
    "EventSourceNmtMnu", // NmtMnu module
    "EventSourceObdu", // Obdu module
    "EventSourceSdoUdp", // Sdo/Udp module
    "EventSourceSdoAsnd", // Sdo/Asnd module
    "EventSourceSdoAsySeq", // Sdo asynchronus Sequence Layer module
    "EventSourceSdoCom", // Sdo command layer module
    "EventSourceTimeru", // Timeru module
    "EventSourceCfgMau", // CfgMau module
    "EventSourceEventu", // Eventu module
    "EventSourceEplApi", // Api module
    "EventSourceLedu", // Ledu module
    "EventSourceGw309Ascii", // GW309ASCII module
};
unsigned int uiNumEplEvtSrcStr_g = (sizeof(eplEvtSrcStr_g) / sizeof(*(eplEvtSrcStr_g)));

/* text strings for POWERLINK event sinks */
static char * eplEvtSinkStr_g[] = {
    "EventSinkSync",
    "EventSinkNmtk",
    "EventSinkDllk",
    "EventSinkDlluCal",
    "EventSinkDllkCal",
    "EventSinkPdok",
    "EventSinkNmtu",
    "EventSinkErrk",
    "EventSinkErru",
    "EventSinkSdoAsySeq",
    "EventSinkNmtMnu",
    "EventSinkLedu",
    "EventSinkPdokCal",
    "EventSinkGw309Ascii",
    "EventSinkApi"
};
unsigned int uiNumEplEvtSinkStr_g = (sizeof(eplEvtSinkStr_g) / sizeof(*(eplEvtSinkStr_g)));

/* text strings for POWERLINK event types */
static char *eplEvtTypeStr_g[] = {
    "0",
    "EventTypeNmtEvent", // NMT event
    "EventTypePdoRx", // PDO frame received event (PRes/PReq)
    "EventTypePdoTx", // PDO frame transmitted event (PRes/PReq)
    "EventTypePdoSoa", // SoA frame received event (isochronous phase completed)
    "EventTypeSync", // Sync event (e.g. SoC or anticipated SoC)
    "EventTypeTimer", // Timer event
    "EventTypeHeartbeat", // Heartbeat event
    "EventTypeHistoryEntry", // Error history entry event
    "EventTypeDllkFlag1", // DLL kernel Flag 1 changed event
    "EventTypeDllkFillTx", // DLL kernel fill TxBuffer event
    "EventTypeDllkPresReady", // DLL kernel PRes ready event
    "EventTypeError", // Error event for API layer
    "EventTypeNmtStateChange", // indicate change of NMT-State
    "EventTypeDllError", // DLL error event for Error handler
    "EventTypeAsndRx", // received ASnd frame for DLL user module
    "EventTypeDllkServFilter", // configure ServiceIdFilter
    "EventTypeDllkIdentity", // configure Identity
    "EventTypeDllkConfig", // configure ConfigParam
    "EventTypeDllkIssueReq", // issue Ident/Status request
    "EventTypeDllkAddNode", // add node to isochronous phase
    "EventTypeDllkDelNode", // remove node from isochronous phase
    "EventTypeDllkConfigNode", // configures parameters of node
    "EventTypeDllkStartReducedCycle", // start reduced EPL cycle on MN
    "EventTypeNmtMnuNmtCmdSent", // NMT command was actually sent
    "EventTypeApiUserDef", // user-defined event
    "EventTypeDllkCycleFinish", // SoA sent, cycle finished
    "0x1B", "0x1C", "0x1D", "0x1E", "0x1F",
    "EventTypePdokAlloc", // alloc PDOs
    "EventTypePdokConfig", // configure PDO channel
    "EventTypeNmtMnuNodeCmd", // trigger NMT node command
    "EventTypeGw309AsciiReq", // GW309ASCII request
    "EventTypeNmtMnuNodeAdded", // node was added to isochronous phase by DLL
};
unsigned int uiNumEplEvtTypeStr_g = (sizeof(eplEvtTypeStr_g) / sizeof(*(eplEvtTypeStr_g)));

/* text strings for POWERLINK states */
static tNmtStateInfo nmtStateInfo_g[] =
{
    {kEplNmtGsOff, "NmtGsOff"},
    {kEplNmtGsInitialising, "NmtGsInitializing"},
    {kEplNmtGsResetApplication, "NmtGsResetApplication"},
    {kEplNmtGsResetCommunication, "NmtGsResetCommunication"},
    {kEplNmtGsResetConfiguration, "NmtGsResetConfiguration"},
    {kEplNmtCsNotActive, "NmtCsNotActive"},
    {kEplNmtCsPreOperational1, "NmtCsPreOperational1"},
    {kEplNmtCsStopped, "NmtCsStopped"},
    {kEplNmtCsPreOperational2, "NmtCsPreOperational2"},
    {kEplNmtCsReadyToOperate, "NmtCsReadyToOperate"},
    {kEplNmtCsOperational, "NmtCsOperational"},
    {kEplNmtCsBasicEthernet, "NmtCsBasicEthernet"},
    {kEplNmtMsNotActive, "NmtMsNotActive"},
    {kEplNmtMsPreOperational1, "NmtMsPreOperational1"},
    {kEplNmtMsPreOperational2, "NmtMsPreOperational2"},
    {kEplNmtMsReadyToOperate, "NmtMsReadyToOperate"},
    {kEplNmtMsOperational, "NmtMsOperational"},
    {kEplNmtMsBasicEthernet, "NmtMsBasicEthernet"},
};
unsigned int uiNumNmtStateInfo_g = (sizeof(nmtStateInfo_g) / sizeof(*(nmtStateInfo_g)));

/* text strings for API events */
static tApiEventInfo ApiEventInfo_g[] =
{
    { kEplApiEventUserDef,          "User defined"                      },
    { kEplApiEventNmtStateChange,   "NMT state change"                  },
    { kEplApiEventCriticalError,    "CRITICAL error -> stack halted"    },
    { kEplApiEventWarning,          "Warning"                           },
    { kEplApiEventHistoryEntry,     "History entry"                     },
    { kEplApiEventNode,             "Node event"                        },
    { kEplApiEventBoot,             "Boot event"                        },
    { kEplApiEventSdo,              "SDO event"                         },
    { kEplApiEventObdAccess,        "OBD access"                        },
    { kEplApiEventLed,              "LED event"                         },
    { kEplApiEventCfmProgress,      "CFM progress"                      },
    { kEplApiEventCfmResult,        "CFM result"                        },
    { kEplApiEventReceivedPres,     "Received PRes frame"               },
};

static unsigned int uiNumApiEventInfo_g = (sizeof(ApiEventInfo_g) / sizeof(*(ApiEventInfo_g)));

// text strings for NMT node events
static char *EplNmtNodeEvtTypeStr_g[] =
{
    "Found",                    // 0x00
    "Update software",          // 0x01
    "Check configuration",      // 0x02
    "Update configuration",     // 0x03
    "Verify configuration",     // 0x04
    "Ready to start",           // 0x05
    "NMT state",                // 0x06
    "NMT error",                // 0x07
};
unsigned int uiEplNmtNodeEvtTypeStr_g = (sizeof(EplNmtNodeEvtTypeStr_g) / sizeof(*(EplNmtNodeEvtTypeStr_g)));

//=========================================================================//
//                                                                         //
//          P U B L I C   F U N C T I O N S                                //
//                                                                         //
//=========================================================================//

//---------------------------------------------------------------------------
//
// Function:    EplGetNmtEventStr()
//
// Description: returns the string of the specified event
//
// Parameters:  nmtEvent_p            event
//
// Returns:     event string
//---------------------------------------------------------------------------
char *EplGetNmtEventStr(tEplNmtEvent nmtEvent_p)
{
    if (nmtEvent_p >= uiNumEplEvtStr_g)
    {
        return eplInvalidStr_g;
    }
    else
    {
        return eplEvtStr_g[nmtEvent_p];
    }
}

//---------------------------------------------------------------------------
//
// Function:    EplGetEventSourceStr()
//
// Description: returns the string of the specified event source
//
// Parameters:  eventSrc_p            event source
//
// Returns:     event source string
//---------------------------------------------------------------------------
char *EplGetEventSourceStr(tEplEventSource eventSrc_p)
{
    if (eventSrc_p >= uiNumEplEvtSrcStr_g)
    {
        return eplInvalidStr_g;
    }
    else
    {
        return eplEvtSrcStr_g[eventSrc_p];
    }
}

//---------------------------------------------------------------------------
//
// Function:    EplGetEventSinkStr()
//
// Description: returns the string of the specified event sink
//
// Parameters:  eventSink_p            event sink
//
// Returns:     event sink string
//---------------------------------------------------------------------------
char *EplGetEventSinkStr(tEplEventSink eventSink_p)
{
    if (eventSink_p >= uiNumEplEvtSinkStr_g)
    {
        return eplInvalidStr_g;
    }
    else
    {
        return eplEvtSinkStr_g[eventSink_p];
    }
}

//---------------------------------------------------------------------------
//
// Function:    EplGetEventTypeStr()
//
// Description: returns the string of the specified event type
//
// Parameters:  eventType_p            event type
//
// Returns:     event type string
//---------------------------------------------------------------------------
char *EplGetEventTypeStr(tEplEventType eventType_p)
{
    if (eventType_p >= uiNumEplEvtTypeStr_g)
    {
        return eplInvalidStr_g;
    }
    else
    {
        return eplEvtTypeStr_g[eventType_p];
    }
}

//---------------------------------------------------------------------------
//
// Function:    EplGetNmtStateStr()
//
// Description: returns the string of the specified NMT state
//
// Parameters:  nmtState_p            NMT state
//
// Returns:     NMT state string
//---------------------------------------------------------------------------
char *EplGetNmtStateStr(tEplNmtState nmtState_p)
{
    unsigned int         i;

    for (i = 0; i < uiNumNmtStateInfo_g; i++)
    {
        if (nmtStateInfo_g[i].m_nmtState == nmtState_p)
            return (nmtStateInfo_g[i].m_sNmtState);
    }
    return eplInvalidStr_g;
}

//---------------------------------------------------------------------------
//
// Function:    EplDebugCompareApiEvent()
//
// Description: Compare two API events. Used by EplGetApiEventStr's bsearch().
//
// Parameters:  Key (element to search), Array
//
// Returns:     -1, 0, or 1 if event id is smaller, equal or greater
//
//---------------------------------------------------------------------------
static int
EplDebugCompareApiEvent( const void *pvEventKey, const void *pvEventArray )
{
    tApiEventInfo    *pEventKey  = (tEplApiEventType *) pvEventKey;
    tApiEventInfo    *pEventArry = (tEplApiEventType *) pvEventArray;

    if( pEventKey->m_ApiEvent < pEventArry->m_ApiEvent )
    {
        return  -1;
    }
    else if( pEventKey->m_ApiEvent > pEventArry->m_ApiEvent )
    {
        return  1;
    }

    return 0;
}

//---------------------------------------------------------------------------
//
// Function:    EplGetApiEventStr()
//
// Description: returns the string of the specified API event
//
//              The function uses stdlib's bsearch() to shorten
//              the average search time.
//
// Parameters:  ApiEvent_p            API event to translate
//
// Returns:     String describing the API event, if found
//              eplInvalidStr_g if not found
//
//---------------------------------------------------------------------------
char *EplGetApiEventStr( tEplApiEventType ApiEvent_p)
{
    char            *pstrReturn;
    tApiEventInfo   *pApiEventInfo;
    tApiEventInfo   Key;

    // Init
    Key.m_ApiEvent  = ApiEvent_p;

    // Search element
    pApiEventInfo   = bsearch(  &Key, ApiEventInfo_g,
                                uiNumApiEventInfo_g, sizeof(Key),
                                EplDebugCompareApiEvent);

    // Check result
    if( NULL != pApiEventInfo )
    {
        return  pApiEventInfo->m_sApiEvent;
    }

    return eplInvalidStr_g;
}

//---------------------------------------------------------------------------
//
// Function:    EplGetNmtNodeEventTypeStr()
//
// Description: returns the string of the specified NMT node event
//
// Parameters:  NodeEventType_p        Type of NMT node event
//
// Returns:     event type string if found
//              eplInvalidStr_g if not found
//
//---------------------------------------------------------------------------
char *EplGetNmtNodeEventTypeStr( tEplNmtNodeEvent NodeEventType_p )
{
    if( NodeEventType_p >= uiEplNmtNodeEvtTypeStr_g )
    {
        return  eplInvalidStr_g;
    }
    else
    {
        return  EplNmtNodeEvtTypeStr_g[ NodeEventType_p ];
    }
}
