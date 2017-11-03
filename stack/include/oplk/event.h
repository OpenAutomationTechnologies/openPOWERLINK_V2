/**
********************************************************************************
\file   oplk/event.h

\brief  Header file for event module

This file contains definitions for the event module.
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2012, SYSTEC electronic GmbH
Copyright (c) 2016, B&R Industrial Automation GmbH
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
#ifndef _INC_oplk_event_H_
#define _INC_oplk_event_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <oplk/oplkinc.h>
#include <oplk/nmt.h>

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

// max size of event argument
#ifndef MAX_EVENT_ARG_SIZE
#define MAX_EVENT_ARG_SIZE          2048
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
    kEventTypeNmtEvent              = 0x01,     ///< NMT event (arg is pointer to tNmtEvent)
    kEventTypePdoRx                 = 0x02,     ///< PDO frame received event (PRes/PReq) (arg is pointer to tFrameInfo)
    kEventTypePdoTx                 = 0x03,     ///< PDO frame transmitted event (PRes/PReq) (arg is pointer to tFrameInfo)
    kEventTypePdoSoa                = 0x04,     ///< SoA frame received event (isochronous phase completed) (arg is pointer to nothing)
    kEventTypeSync                  = 0x05,     ///< Sync event (e.g. SoC or anticipated SoC) (arg is pointer to nothing)
    kEventTypeTimer                 = 0x06,     ///< Timer event (arg is pointer to tTimerEventArg)
    kEventTypeHeartbeat             = 0x07,     ///< Heartbeat event (arg is pointer to tHeartbeatEvent)
    kEventTypeHistoryEntry          = 0x08,     ///< Error history entry event (arg is pointer to the tErrHistoryEntry)
    kEventTypeDllkFlag1             = 0x09,     ///< DLL kernel Flag 1 changed event (arg is pointer to nothing)
    kEventTypeDllkFillTx            = 0x0A,     ///< DLL kernel fill TxBuffer event (arg is pointer to tDllAsyncReqPriority)
    kEventTypeDllkPresReady         = 0x0B,     ///< DLL kernel PRes ready event (arg is pointer to nothing)
    kEventTypeError                 = 0x0C,     ///< Error event for API layer (arg is pointer to tEventError)
    kEventTypeNmtStateChange        = 0x0D,     ///< Indicate change of NMT-State (arg is pointer to tEventNmtStateChange)
    kEventTypeDllError              = 0x0E,     ///< DLL error event for error handler (arg is pointer to tEventDllError)
    kEventTypeAsndRx                = 0x0F,     ///< received ASnd frame for DLL user module (arg is pointer to tPlkFrame)
    kEventTypeDllkServFilter        = 0x10,     ///< configure ServiceIdFilter (arg is pointer to tDllAsndServiceId)
    kEventTypeDllkIdentity          = 0x11,     ///< configure Identity (arg is pointer to tDllIdentParam)
    kEventTypeDllkConfig            = 0x12,     ///< configure ConfigParam (arg is pointer to tDllConfigParam)
    kEventTypeDllkIssueReq          = 0x13,     ///< issue Ident/Status request (arg is pointer to tDllCalIssueRequest)
    kEventTypeDllkAddNode           = 0x14,     ///< add node to isochronous phase (arg is pointer to tDllNodeOpParam)
    kEventTypeDllkDelNode           = 0x15,     ///< remove node from isochronous phase (arg is pointer to tDllNodeOpParam)
    kEventTypeDllkConfigNode        = 0x16,     ///< configures parameters of node (arg is pointer to tDllNodeInfo)
    kEventTypeDllkStartReducedCycle = 0x17,     ///< start reduced POWERLINK cycle on MN (arg is pointer to nothing)
    kEventTypeNmtMnuNmtCmdSent      = 0x18,     ///< NMT command was actually sent (arg is pointer to tPlkFrame)
    kEventTypeApiUserDef            = 0x19,     ///< user-defined event (arg is user-defined pointer)
    kEventTypeDllkCycleFinish       = 0x1A,     ///< SoA sent, cycle finished (arg is pointer to nothing)
    kEventTypePdokAlloc             = 0x20,     ///< allocate PDOs (arg is pointer to tPdoAllocationParam)
    kEventTypePdokConfig            = 0x21,     ///< configure PDO channel (arg is pointer to tPdoChannelConf)
    kEventTypeNmtMnuNodeCmd         = 0x22,     ///< trigger NMT node command (arg is pointer to tNmtNodeCommand)
    kEventTypeGw309AsciiReq         = 0x23,     ///< GW309ASCII request (arg is pointer to pointer of tGw309AsciiRequest)
    kEventTypeNmtMnuNodeAdded       = 0x24,     ///< node was added to isochronous phase by DLL (arg is pointer to unsigned int containing the node-ID)
    kEventTypePdokSetupPdoBuf       = 0x25,     ///< deallocate PDOs
    kEventTypeTimesynckControl      = 0x26,     ///< Enable/disable the timesync trigger (arg is pointer to BOOL)
    kEventTypeReleaseRxFrame        = 0x27,     ///< Free receive buffer (arg is pointer to the buffer to release)
    kEventTypeAsndNotRx             = 0x28,     ///< Didn't receive ASnd frame for DLL user module (arg is pointer to tDllAsndNotRx)
    kEventTypeAsndRxInfo            = 0x29,     ///< Received ASnd frame for DLL user module (arg is pointer to tFrameInfo)
    kEventTypeReceivedAmni          = 0x2A,     ///< Received AMNI frame (arg is pointer to unsigned int containing the source node-ID)
    kEventTypeReceivedPres          = 0x30,     ///< Received a PRes frame, which shall be forwarded to application (arg is pointer to tEventReceivedPres)
    kEventTypeRequPresForward       = 0x31,     ///< Request forwarding of a PRes frame to API layer (e.g. for conformance test)
    kEventTypeSdoAsySend            = 0x32,     ///< SDO sequence layer event (for SDO command layer testing module)
} eEventType;

/**
\brief Event type data type

Data type for the enumerator \ref eEventType.
*/
typedef UINT8 tEventType;

/**
\brief  Valid sinks for an event

This enumeration defines all valid event sinks.
*/
typedef enum
{
    kEventSinkSync                  = 0x00,     ///< Sync event for application or kernel POWERLINK module
    kEventSinkNmtk                  = 0x01,     ///< events for Nmtk module
    kEventSinkDllk                  = 0x02,     ///< events for Dllk module
    kEventSinkDlluCal               = 0x03,     ///< events for DlluCal module
    kEventSinkDllkCal               = 0x04,     ///< events for DllkCal module
    kEventSinkPdok                  = 0x05,     ///< events for Pdok module
    kEventSinkNmtu                  = 0x06,     ///< events for Nmtu module
    kEventSinkErrk                  = 0x07,     ///< events for Error handler module
    kEventSinkErru                  = 0x08,     ///< events for Error signaling module
    kEventSinkSdoAsySeq             = 0x09,     ///< events for asynchronous SDO Sequence Layer module
    kEventSinkNmtMnu                = 0x0A,     ///< events for NmtMnu module
    // Reserved (0x0B)
    kEventSinkPdokCal               = 0x0C,     ///< events for PdokCal module
    kEventSinkGw309Ascii            = 0x0E,     ///< events for GW309ASCII module
    kEventSinkApi                   = 0x0F,     ///< events for API module
    kEventSinkSdoTest               = 0x10,     ///< events for SDO testing module
    kEventSinkTimesynck             = 0x11,     ///< events for Timesynck module

    kEventSinkInvalid               = 0xFF      ///< Identifies an invalid sink
} eEventSink;

/**
\brief Event sink data type

Data type for the enumerator \ref eEventSink.
*/
typedef UINT8 tEventSink;

/**
\brief  Valid sources for an event

This enumeration defines all valid event sources.
*/

typedef enum
{
    kEventSourceDllk                = 0x01,     ///< Events from Dllk module
    kEventSourceNmtk                = 0x02,     ///< Events from Nmtk module
    kEventSourceObdk                = 0x03,     ///< Events from Obdk module
    kEventSourcePdok                = 0x04,     ///< Events from Pdok module
    kEventSourceTimerk              = 0x05,     ///< Events from Timerk module
    kEventSourceEventk              = 0x06,     ///< Events from Eventk module
    kEventSourceSyncCb              = 0x07,     ///< Events from sync-Cb
    kEventSourceErrk                = 0x08,     ///< Events from kernel error handler module

    kEventSourceDllu                = 0x10,     ///< Events from Dllu module
    kEventSourceNmtu                = 0x11,     ///< Events from Nmtu module
    kEventSourceNmtCnu              = 0x12,     ///< Events from NmtCnu module
    kEventSourceNmtMnu              = 0x13,     ///< Events from NmtMnu module
    kEventSourceObdu                = 0x14,     ///< Events from Obdu module
    kEventSourceSdoUdp              = 0x15,     ///< Events from Sdo/Udp module
    kEventSourceSdoAsnd             = 0x16,     ///< Events from Sdo/Asnd module
    kEventSourceSdoAsySeq           = 0x17,     ///< Events from Sdo asynchronous Sequence Layer module
    kEventSourceSdoCom              = 0x18,     ///< Events from Sdo command layer module
    kEventSourceTimeru              = 0x19,     ///< Events from Timeru module
    kEventSourceCfgMau              = 0x1A,     ///< Events from CfgMau module
    kEventSourceEventu              = 0x1B,     ///< Events from Eventu module
    kEventSourceOplkApi             = 0x1C,     ///< Events from Api module
    // Reserved (0x1D)
    kEventSourceGw309Ascii          = 0x1E,     ///< Events from GW309ASCII module
    kEventSourceErru                = 0x1F,     ///< Events from User Error handler module
    kEventSourceSdoTest             = 0x20,     ///< Events from SDO testing module
    kEventSourceTimesynck           = 0x21,     ///< events from Timesynck module

    kEventSourceInvalid             = 0xFF      ///< Identifies an invalid event source
} eEventSource;

/**
\brief Event source data type

Data type for the enumerator \ref eEventSource.
*/
typedef UINT8 tEventSource;

/**
\brief Enumerator for queue

This enumerator identifies an event queue instance in order to differ between
layer queues (kernel-to-user or user-to-kernel) and layer-internal queues
(kernel- or user-internal).
*/
typedef enum
{
    kEventQueueK2U                  = 0x00,     ///< Kernel-to-user queue
    kEventQueueKInt                 = 0x01,     ///< Kernel-internal queue
    kEventQueueU2K                  = 0x02,     ///< User-to-kernel queue
    kEventQueueUInt                 = 0x03,     ///< User-internal queue
    kEventQueueNum                  = 0x04      ///< Maximum number of queues
} eEventQueue;

/**
\brief Event queue data type

Data type for the enumerator \ref eEventQueue.
*/
typedef UINT8 tEventQueue;

/**
\brief  Structure for events

The structure defines an openPOWERLINK event.
(element order must not be changed!)
*/
typedef struct
{
    tEventType          eventType;              ///< Type of this event
    tEventSink          eventSink;              ///< Sink of this event
    tNetTime            netTime;                ///< Timestamp of the event
    UINT                eventArgSize;           ///< Size of the event argument
    // Replace the event argument pointer variable with
    // union to maintain same size and alignment
    // for the structure on different processor architectures
    // like x86_64 (64 bit) and NIOS2 (32 bit).
    union
    {
        void*           pEventArg;              ///< Pointer to event argument
        UINT64          padding;                ///< 64 Bit placeholder
    } eventArg;
} tEvent;

/**
\brief  Structure for OBD error information

The structure defines the error event information provided
by the obd module.
*/
typedef struct
{
    UINT                index;                  ///< Index of object
    UINT                subIndex;               ///< Sub index of object
} tEventObdError;

/**
\brief  Structure for error events

The structure defines an error event.
*/
typedef struct
{
    tEventSource        eventSource;            ///< Module which posted this error event
    tOplkError          oplkError;              ///< Error which occurred
    union
    {
        BYTE                    byteArg;        ///< BYTE argument
        UINT32                  uintArg;        ///< UINT32 argument
        tEventSource            eventSource;    ///< Argument from Eventk/u module (originating error source)
        tEventObdError          obdError;       ///< Argument from Obd module
        tEventSink              eventSink;      ///< Argument from Eventk/u module on oplkError == kErrorEventUnknownSink
    } errorArg;
} tEventError;

/**
\brief  Structure for DLL error events

The structure defines an DLL error event.
*/
typedef struct
{
    ULONG               dllErrorEvents;         ///< Contains the DLL error (DLL_ERR_*)
    UINT                nodeId;                 ///< Node ID
    tNmtState           nmtState;               ///< NMT state
    tOplkError          oplkError;              ///< openPOWERLINK error code
} tEventDllError;

/**
\brief  Callback function to get informed about sync event

\return The function returns a tOplkError error code.
*/
typedef tOplkError (*tSyncCb)(void);

#endif /* _INC_oplk_event_H_ */
