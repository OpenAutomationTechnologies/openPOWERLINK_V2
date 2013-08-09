/**
********************************************************************************
\file   event.h

\brief  Header file for event module

This file contains definitions for the event module.

*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2012, SYSTEC electronic GmbH
Copyright (c) 2012, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
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

#ifndef _INC_event_H_
#define _INC_event_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <EplInc.h>
#include <nmt.h>

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

// max size of event argument
#ifndef EPL_MAX_EVENT_ARG_SIZE
#define EPL_MAX_EVENT_ARG_SIZE          2048
#endif

//------------------------------------------------------------------------------
// typedef
//------------------------------------------------------------------------------

/**
\brief  Type of an event

This enumeration defines all valid event types.
*/
typedef enum
{
    kEplEventTypeNmtEvent       = 0x01, ///< NMT event (arg is pointer to tNmtEvent)
    kEplEventTypePdoRx          = 0x02, ///< PDO frame received event (PRes/PReq) (arg is pointer to tFrameInfo)
    kEplEventTypePdoTx          = 0x03, ///< PDO frame transmitted event (PRes/PReq) (arg is pointer to tFrameInfo)
    kEplEventTypePdoSoa         = 0x04, ///< SoA frame received event (isochronous phase completed) (arg is pointer to nothing)
    kEplEventTypeSync           = 0x05, ///< Sync event (e.g. SoC or anticipated SoC) (arg is pointer to nothing)
    kEplEventTypeTimer          = 0x06, ///< Timer event (arg is pointer to tEplTimerEventArg)
    kEplEventTypeHeartbeat      = 0x07, ///< Heartbeat event (arg is pointer to tEplHeartbeatEvent)
    kEplEventTypeHistoryEntry   = 0x08, ///< Error history entry event (arg is pointer to the tEplErrHistoryEntry)
    kEplEventTypeDllkFlag1      = 0x09, ///< DLL kernel Flag 1 changed event (arg is pointer to nothing)
    kEplEventTypeDllkFillTx     = 0x0A, ///< DLL kernel fill TxBuffer event (arg is pointer to tDllAsyncReqPriority)
    kEplEventTypeDllkPresReady  = 0x0B, ///< DLL kernel PRes ready event (arg is pointer to nothing)
    kEplEventTypeError          = 0x0C, ///< Error event for API layer (arg is pointer to tEplEventError)
    kEplEventTypeNmtStateChange = 0x0D, ///< Indicate change of NMT-State (arg is pointer to tEventNmtStateChange)
    kEplEventTypeDllError       = 0x0E, ///< DLL error event for error handler (arg is pointer to tEplErrorHandlerkEvent)
    kEplEventTypeAsndRx         = 0x0F, ///< received ASnd frame for DLL user module (arg is pointer to tEplFrame)
    kEplEventTypeDllkServFilter = 0x10, ///< configure ServiceIdFilter (arg is pointer to tEplDllCalServiceIdFilter)
    kEplEventTypeDllkIdentity   = 0x11, ///< configure Identity (arg is pointer to tDllIdentParam)
    kEplEventTypeDllkConfig     = 0x12, ///< configure ConfigParam (arg is pointer to tDllConfigParam)
    kEplEventTypeDllkIssueReq   = 0x13, ///< issue Ident/Status request (arg is pointer to tEplDllCalIssueRequest)
    kEplEventTypeDllkAddNode    = 0x14, ///< add node to isochronous phase (arg is pointer to tDllNodeOpParam)
    kEplEventTypeDllkDelNode    = 0x15, ///< remove node from isochronous phase (arg is pointer to tDllNodeOpParam)
    kEplEventTypeDllkConfigNode = 0x16, ///< configures parameters of node (arg is pointer to tDllNodeInfo)
    kEplEventTypeDllkStartReducedCycle = 0x17, ///< start reduced EPL cycle on MN (arg is pointer to nothing)
    kEplEventTypeNmtMnuNmtCmdSent = 0x18, ///< NMT command was actually sent (arg is pointer to tEplFrame)
    kEplEventTypeApiUserDef     = 0x19, ///< user-defined event (arg is user-defined pointer)
    kEplEventTypeDllkCycleFinish= 0x1A, ///< SoA sent, cycle finished (arg is pointer to nothing)
    kEplEventTypePdokAlloc      = 0x20, ///< alloc PDOs (arg is pointer to tEplPdoAllocationParam)
    kEplEventTypePdokConfig     = 0x21, ///< configure PDO channel (arg is pointer to tEplPdoChannelConf)
    kEplEventTypeNmtMnuNodeCmd  = 0x22, ///< trigger NMT node command (arg is pointer to tEplNmtMnuNodeCmd)
    kEplEventTypeGw309AsciiReq  = 0x23, ///< GW309ASCII request (arg is pointer to pointer of tEplGw309AsciiRequest)
    kEplEventTypeNmtMnuNodeAdded = 0x24, ///< node was added to isochronous phase by DLL (arg is pointer to unsigned int containing the node-ID)
    kEplEventTypePdokSetupPdoBuf = 0x25,  ///< dealloc PDOs
    kEplEventTypePdokControlSync = 0x26 ///< enable/disable the pdokcal sync trigger (arg is pointer to BOOL)

} tEplEventType;


/**
\brief  Valid sinks for an event

This enumeration defines all valid event sinks.
*/
typedef enum
{
    kEplEventSinkSync           = 0x00, ///< Sync event for application or kernel EPL module
    kEplEventSinkNmtk           = 0x01, ///< events for Nmtk module
    kEplEventSinkDllk           = 0x02, ///< events for Dllk module
    kEplEventSinkDlluCal        = 0x03, ///< events for DlluCal module
    kEplEventSinkDllkCal        = 0x04, ///< events for DllkCal module
    kEplEventSinkPdok           = 0x05, ///< events for Pdok module
    kEplEventSinkNmtu           = 0x06, ///< events for Nmtu module
    kEplEventSinkErrk           = 0x07, ///< events for Error handler module
    kEplEventSinkErru           = 0x08, ///< events for Error signaling module
    kEplEventSinkSdoAsySeq      = 0x09, ///< events for asyncronous SDO Sequence Layer module
    kEplEventSinkNmtMnu         = 0x0A, ///< events for NmtMnu module
    kEplEventSinkLedu           = 0x0B, ///< events for Ledu module
    kEplEventSinkPdokCal        = 0x0C, ///< events for PdokCal module
    kEplEventSinkGw309Ascii     = 0x0E, ///< events for GW309ASCII module
    kEplEventSinkApi            = 0x0F, ///< events for API module

    kEplEventSinkInvalid        = 0xFF  ///< Identifies an invalid sink
} tEplEventSink;

/**
\brief  Valid sources for an event

This enumeration defines all valid event sources.
*/

typedef enum
{
    kEplEventSourceDllk         = 0x01, ///< Events from Dllk module
    kEplEventSourceNmtk         = 0x02, ///< Events from Nmtk module
    kEplEventSourceObdk         = 0x03, ///< Events from Obdk module
    kEplEventSourcePdok         = 0x04, ///< Events from Pdok module
    kEplEventSourceTimerk       = 0x05, ///< Events from Timerk module
    kEplEventSourceEventk       = 0x06, ///< Events from Eventk module
    kEplEventSourceSyncCb       = 0x07, ///< Events from sync-Cb
    kEplEventSourceErrk         = 0x08, ///< Events from kernel error handler module

    kEplEventSourceDllu         = 0x10, ///< Events from Dllu module
    kEplEventSourceNmtu         = 0x11, ///< Events from Nmtu module
    kEplEventSourceNmtCnu       = 0x12, ///< Events from NmtCnu module
    kEplEventSourceNmtMnu       = 0x13, ///< Events from NmtMnu module
    kEplEventSourceObdu         = 0x14, ///< Events from Obdu module
    kEplEventSourceSdoUdp       = 0x15, ///< Events from Sdo/Udp module
    kEplEventSourceSdoAsnd      = 0x16, ///< Events from Sdo/Asnd module
    kEplEventSourceSdoAsySeq    = 0x17, ///< Events from Sdo asynchronous Sequence Layer module
    kEplEventSourceSdoCom       = 0x18, ///< Events from Sdo command layer module
    kEplEventSourceTimeru       = 0x19, ///< Events from Timeru module
    kEplEventSourceCfgMau       = 0x1A, ///< Events from CfgMau module
    kEplEventSourceEventu       = 0x1B, ///< Events from Eventu module
    kEplEventSourceEplApi       = 0x1C, ///< Events from Api module
    kEplEventSourceLedu         = 0x1D, ///< Events from Ledu module
    kEplEventSourceGw309Ascii   = 0x1E, ///< Events from GW309ASCII module
    kEplEventSourceErru         = 0x1F, ///< Events from User Error handler module

    kEplEventSourceInvalid      = 0xFF  ///< Identifies an invalid event source
} tEplEventSource;

/**
\brief Enumerator for Queue

This enumerator identifies an event queue instance in order to differ between
layer queues (kernel-to-user or user-to-kernel) and layer-internal queues
(kernel- or user-internal).
*/
typedef enum
{
    kEventQueueK2U              = 0x00, ///< kernel-to-user queue
    kEventQueueKInt             = 0x01, ///< kernel-internal queue
    kEventQueueU2K              = 0x02, ///< user-to-kernel queue
    kEventQueueUInt             = 0x03, ///< user-internal queue
    kEventQueueNum              = 0x04  ///< maximum number of queues
} tEventQueue;


/**
\brief  structure for events

The structure defines an openPOWERLINK event.
(element order must not be changed!)
*/
typedef struct
{
    tEplEventType       m_EventType /*:28*/;    ///< Type of this event
    tEplEventSink       m_EventSink /*:4*/;     ///< Sink of this event
    tEplNetTime         m_NetTime;              ///< Timestamp of the event
    UINT                m_uiSize;               ///< Size of the event argument
    void*               m_pArg;                 ///< Argument of the event
} tEplEvent;


/**
\brief  short structure for events

The structure defines an openPOWERLINK event without its argument.
(element order must not be changed!)
*/
typedef struct
{
    tEplEventType       m_EventType /*:28*/;    ///< Type of this event
    tEplEventSink       m_EventSink /*:4*/;     ///< Sink of this event
    tEplNetTime         m_NetTime;              ///< Timestamp of the event
} tEplEventShort;


/**
\brief  structure for OBD error information

The structure defines the error event information provided
by the obd module.
*/
typedef struct
{
    UINT                m_uiIndex;
    UINT                m_uiSubIndex;
} tEplEventObdError;


/**
\brief  structure for error events

The structure defines an error event.
*/
typedef struct
{
    tEplEventSource     m_EventSource;          ///< Module which posted this error event
    tEplKernel          m_EplError;             ///< Error which occurred
    union
    {
        BYTE                    m_bArg;         ///< BYTE argument
        UINT32                  m_dwArg;        ///< UINT32 argument
        tEplEventSource         m_EventSource;  ///< Argument from Eventk/u module (originating error source)
        tEplEventObdError       m_ObdError;     ///< Argument from Obd module
        tEplEventSink           m_EventSink;    ///< Argument from Eventk/u module on m_EplError == kEplEventUnknownSink
        //tEplErrHistoryEntry   m_HistoryEntry; ///< from Nmtk/u module
    } m_Arg;
} tEplEventError;


/**
\brief  structure for DLL error events

The structure defines an DLL error event.
*/
typedef struct
{
    ULONG               m_ulDllErrorEvents;     ///< EPL_DLL_ERR_*
    UINT                m_uiNodeId;             ///< Node ID
    tNmtState           m_NmtState;             ///< NMT state
    tEplKernel          m_EplError;             ///< Error code
} tErrHndkEvent;

/**
\brief  callback function to get informed about sync event
*/
typedef tEplKernel (PUBLIC* tEplSyncCb) (void);

/**
\brief callback for event post

This callback is used to call event processing over the module boundaries.
e.g. EplEventkCal -> EplEventkProcess

\param pEplEvent_p          Pointer to event which should be processed.
*/
typedef tEplKernel (PUBLIC* tEplProcessEventCb) (tEplEvent* pEplEvent_p);

/**
\brief callback for event error post

This callback is used to call error event posting over the module boundaries.
e.g. EplEventkCal -> eventk_postError
*/
typedef tEplKernel (PUBLIC* tEplPostErrorEventCb) (tEplEventSource EventSource_p, tEplKernel eplError_p, UINT argSize_p, void *pArg_p);

/**
\brief  event dispatch entry

The following struct specifies the entry for an event dispatch table.
The table is used to store the appropriate event handlers for a specific
event sink.
 */
typedef struct
{
    tEplEventSink       sink;               ///< Event sink
    tEplEventSource     source;             ///< Corresponding event source
    tEplProcessEventCb  pfnEventHandler;    ///< Event handler responsible for this sink
} tEventDispatchEntry;

/**
\brief Pointer to event queue instances

This typedef is used to identify the abstracted queue instance in
eEventcal-*. The queue itself is defined by its implementation (e.g. DIRECT or
SHB)
*/
typedef void* tEventQueueInstPtr;

//------------------------------------------------------------------------------
// function prototypes
//------------------------------------------------------------------------------

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
}
#endif

#endif /* _INC_event_H_ */


