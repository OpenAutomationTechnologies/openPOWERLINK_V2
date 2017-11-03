/**
********************************************************************************
\file   debugstr.c

\brief  Debug String module

This file implements the debug string module. It is used to convert a lot
of openPOWERLINK enumerations and error codes into descriptive strings.

\ingroup module_debugstr
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2015, SYSTEC electronic GmbH
Copyright (c) 2016, B&R Industrial Automation GmbH
Copyright (c) 2010, E. Dumas
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
#include <oplk/debugstr.h>

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

//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------
typedef struct
{
    tNmtState           nmtState;
    const char*         sNmtState;
} tNmtStateInfo;

typedef struct
{
    tOplkApiEventType   apiEvent;
    const char*         sApiEvent;
} tApiEventInfo;

typedef struct
{
    tOplkError          key;
    const char*         sName;
} tRetValInfo;

typedef struct
{
    UINT16              key;
    const char*         sName;
} tEmergErrCodeInfo;

typedef struct
{
    UINT32              abortCode;
    const char*         sAbortCode;
} tAbortCodeInfo;

typedef struct
{
    tNmtNodeCommand     nodeCommand;
    const char*         sNodeCommand;
} tNmtNodeCommandInfo;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static const char* const invalidStr_l = "INVALID";

// text strings for POWERLINK events
static const char* const nmtEventStr_l[] =
{
    "NmtEventNoEvent",                      //
    "NmtEventDllMePres",                    //
    "NmtEventDllMePresTimeout",             //
    "NmtEventDllMeAsndTimeout",             //
    "NmtEventDllMeSoaSent",                 //
    "NmtEventDllMeSocTrig",                 //
    "NmtEventDllMeSoaTrig",                 //
    "NmtEventDllCeSoc",                     //
    "NmtEventDllCePreq",                    //
    "NmtEventDllCePres",                    //
    "NmtEventDllCeSoa",                     //
    "NmtEventDllCeAInv",                    //
    "NmtEventDllCeAsnd",                    //
    "NmtEventDllCeFrameTimeout",            //
    "NmtEventDllReAmni",                    //
    "NmtEventDllReSwitchOverTimeout",       // reserved
    "NmtEventSwReset",                      // NMT_GT1, NMT_GT2, NMT_GT8
    "NmtEventResetNode",                    //
    "NmtEventResetCom",                     //
    "NmtEventResetConfig",                  //
    "NmtEventEnterPreOperational2",         //
    "NmtEventEnableReadyToOperate",         //
    "NmtEventStartNode",                    // NMT_CT7
    "NmtEventStopNode",                     //
    "NmtEventGoToStandby",                  //
    "NmtEventGoToStandbyDelayed",           //
    "0x1A",                                 // reserved
    "0x1B",                                 // reserved
    "0x1C",                                 // reserved
    "0x1D",                                 // reserved
    "0x1E",                                 // reserved
    "0x1F",                                 // reserved
    "NmtEventEnterResetApp",                //
    "NmtEventEnterResetCom",                //
    "NmtEventInternComError",               // NMT_GT6, internal communication error -> enter ResetCommunication
    "NmtEventEnterResetConfig",             //
    "NmtEventEnterCsNotActive",             //
    "NmtEventEnterMsNotActive",             //
    "NmtEventTimerBasicEthernet",           // NMT_CT3; timer triggered state change (NotActive -> BasicEth)
    "NmtEventTimerMsPreOp1",                // enter PreOp1 on MN (NotActive -> MsPreOp1)
    "NmtEventNmtCycleError",                // NMT_CT11, NMT_MT6; error during cycle -> enter PreOp1
    "NmtEventTimerMsPreOp2",                // enter PreOp2 on MN (MsPreOp1 -> MsPreOp2 if NmtEventAllMandatoryCNIdent)
    "NmtEventAllMandatoryCNIdent",          // enter PreOp2 on MN if NmtEventTimerMsPreOp2
    "NmtEventEnterReadyToOperate",          // application ready for the state ReadyToOp
    "NmtEventEnterMsOperational",           // enter Operational on MN
    "NmtEventSwitchOff",                    // enter state Off
    "NmtEventCriticalError",                // enter state Off because of critical error
    "NmtEventEnterRmsNotActive",            //
};

// text strings for POWERLINK event sources
static const char* const eventSourceStr_l[] =
{
    "0",                                    // reserved

    // openPOWERLINK kernel modules
    "EventSourceDllk",                      // Dllk module
    "EventSourceNmtk",                      // Nmtk module
    "EventSourceObdk",                      // Obdk module
    "EventSourcePdok",                      // Pdok module
    "EventSourceTimerk",                    // Timerk module
    "EventSourceEventk",                    // Eventk module
    "EventSourceSyncCb",                    // sync-Cb
    "EventSourceErrk",                      // Error handler module
    "0x09",                                 // reserved
    "0x0a",                                 // reserved
    "0x0b",                                 // reserved
    "0x0c",                                 // reserved
    "0x0d",                                 // reserved
    "0x0e",                                 // reserved
    "0x0f",                                 // reserved

    // openPOWERLINK user modules
    "EventSourceDllu",                      // Dllu module
    "EventSourceNmtu",                      // Nmtu module
    "EventSourceNmtCnu",                    // NmtCnu module
    "EventSourceNmtMnu",                    // NmtMnu module
    "EventSourceObdu",                      // Obdu module
    "EventSourceSdoUdp",                    // Sdo/Udp module
    "EventSourceSdoAsnd",                   // Sdo/Asnd module
    "EventSourceSdoAsySeq",                 // Sdo asynchronous Sequence Layer module
    "EventSourceSdoCom",                    // Sdo command layer module
    "EventSourceTimeru",                    // Timeru module
    "EventSourceCfgMau",                    // CfgMau module
    "EventSourceEventu",                    // Eventu module
    "EventSourceOplkApi",                   // Api module
    "0x1d",                                 // reserved
    "EventSourceGw309Ascii",                // GW309ASCII module
    "EventSourceErru"                       // User error module
};

// text strings for POWERLINK event sinks
static const char* const eventSinkStr_l[] =
{
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
    "0x0b",                                 // reserved
    "EventSinkPdokCal",
    "EventSinkGw309Ascii",
    "EventSinkApi"
};

// text strings for POWERLINK event types
static const char* const eventTypeStr_l[] =
{
    "0",                                    // reserved
    "EventTypeNmtEvent",                    // NMT event
    "EventTypePdoRx",                       // PDO frame received event (PRes/PReq)
    "EventTypePdoTx",                       // PDO frame transmitted event (PRes/PReq)
    "EventTypePdoSoa",                      // SoA frame received event (isochronous phase completed)
    "EventTypeSync",                        // Sync event (e.g. SoC or anticipated SoC)
    "EventTypeTimer",                       // Timer event
    "EventTypeHeartbeat",                   // Heartbeat event
    "EventTypeHistoryEntry",                // Error history entry event
    "EventTypeDllkFlag1",                   // DLL kernel Flag 1 changed event
    "EventTypeDllkFillTx",                  // DLL kernel fill TxBuffer event
    "EventTypeDllkPresReady",               // DLL kernel PRes ready event
    "EventTypeError",                       // Error event for API layer
    "EventTypeNmtStateChange",              // indicate change of NMT-State
    "EventTypeDllError",                    // DLL error event for Error handler
    "EventTypeAsndRx",                      // received ASnd frame for DLL user module
    "EventTypeDllkServFilter",              // configure ServiceIdFilter
    "EventTypeDllkIdentity",                // configure Identity
    "EventTypeDllkConfig",                  // configure ConfigParam
    "EventTypeDllkIssueReq",                // issue Ident/Status request
    "EventTypeDllkAddNode",                 // add node to isochronous phase
    "EventTypeDllkDelNode",                 // remove node from isochronous phase
    "EventTypeDllkConfigNode",              // configures parameters of node
    "EventTypeDllkStartReducedCycle",       // start reduced POWERLINK cycle on MN
    "EventTypeNmtMnuNmtCmdSent",            // NMT command was actually sent
    "EventTypeApiUserDef",                  // user-defined event
    "EventTypeDllkCycleFinish",             // SoA sent, cycle finished
    "0x1B",                                 // reserved
    "0x1C",                                 // reserved
    "0x1D",                                 // reserved
    "0x1E",                                 // reserved
    "0x1F",                                 // reserved
    "EventTypePdokAlloc",                   // alloc PDOs
    "EventTypePdokConfig",                  // configure PDO channel
    "EventTypeNmtMnuNodeCmd",               // trigger NMT node command
    "EventTypeGw309AsciiReq",               // GW309ASCII request
    "EventTypeNmtMnuNodeAdded",             // node was added to isochronous phase by DLL
    "EventTypePdokSetupPdoBuf",             // dealloc PDOs
    "EventTypePdokControlSync",             // enable/disable the pdokcal sync trigger (arg is pointer to BOOL)
    "EventTypeReleaseRxFrame",              // Free receive buffer
    "EventTypeASndNotRx"                    // Didn't receive ASnd frame for DLL user module
};

// text strings for POWERLINK states
static const tNmtStateInfo nmtStateInfo_l[] =
{
    { kNmtGsOff,                 "NmtGsOff"                  },
    { kNmtGsInitialising,        "NmtGsInitializing"         },
    { kNmtGsResetApplication,    "NmtGsResetApplication"     },
    { kNmtGsResetCommunication,  "NmtGsResetCommunication"   },
    { kNmtGsResetConfiguration,  "NmtGsResetConfiguration"   },
    { kNmtCsNotActive,           "NmtCsNotActive"            },
    { kNmtCsPreOperational1,     "NmtCsPreOperational1"      },
    { kNmtCsStopped,             "NmtCsStopped"              },
    { kNmtCsPreOperational2,     "NmtCsPreOperational2"      },
    { kNmtCsReadyToOperate,      "NmtCsReadyToOperate"       },
    { kNmtCsOperational,         "NmtCsOperational"          },
    { kNmtCsBasicEthernet,       "NmtCsBasicEthernet"        },
    { kNmtMsNotActive,           "NmtMsNotActive"            },
    { kNmtMsPreOperational1,     "NmtMsPreOperational1"      },
    { kNmtMsPreOperational2,     "NmtMsPreOperational2"      },
    { kNmtMsReadyToOperate,      "NmtMsReadyToOperate"       },
    { kNmtMsOperational,         "NmtMsOperational"          },
    { kNmtMsBasicEthernet,       "NmtMsBasicEthernet"        },
    { kNmtRmsNotActive,          "NmtRmsNotActive"           },
    { kNmtStateInvalid,          "Invalid NMT State"         },
};

// text strings for API events
static const tApiEventInfo apiEventInfo_l[] =
{
    { kOplkApiEventUserDef,          "User defined"                      },
    { kOplkApiEventNmtStateChange,   "NMT state change"                  },
    { kOplkApiEventCriticalError,    "CRITICAL error -> stack halted"    },
    { kOplkApiEventWarning,          "Warning"                           },
    { kOplkApiEventHistoryEntry,     "History entry"                     },
    { kOplkApiEventNode,             "Node event"                        },
    { kOplkApiEventBoot,             "Boot event"                        },
    { kOplkApiEventSdo,              "SDO event"                         },
    { kOplkApiEventObdAccess,        "OBD access"                        },
    { kOplkApiEventCfmProgress,      "CFM progress"                      },
    { kOplkApiEventCfmResult,        "CFM result"                        },
    { kOplkApiEventReceivedAsnd,     "Received ASnd"                     },
    { kOplkApiEventPdoChange,        "PDO change"                        }
};

// text strings for values of type tOplkError
static const tRetValInfo retValInfo_l[] =
{
    /* area for generic errors 0x0000 - 0x000F */
    { kErrorOk,                       "No error / function call successful"},
    { kErrorIllegalInstance,          "The called instance does not exist"},
    { kErrorInvalidInstanceParam,     "Invalid instance parameter"},
    { kErrorNoFreeInstance,           "Xxx_addInstance was called but no free instance is available"},
    { kErrorWrongSignature,           "Wrong signature while writing to object 0x1010 or 0x1011"},
    { kErrorInvalidOperation,         "Operation not allowed in this situation"},
    { kErrorInvalidNodeId,            "Invalid NodeId was specified"},
    { kErrorNoResource,               "Resource could not be created (Windows, PxROS, ...)"},
    { kErrorShutdown,                 "Stack is shutting down"},
    { kErrorReject,                   "Reject the subsequent command"},
    { kErrorRetry,                    "Retry this command"},
    { kErrorInvalidEvent,             "Invalid event was posted to process function"},
    { kErrorGeneralError,             "General error"},
    { kErrorFeatureMismatch,          "Features of user and kernel stack are mismatched"},

    /* area for EDRV module 0x0010 - 0x001F */
    { kErrorEdrvNoFreeTxDesc,         "No free Tx descriptor available"},
    { kErrorEdrvInvalidCycleLen,      "Invalid cycle length (e.g. 0)"},
    { kErrorEdrvInit,                 "Initialization error"},
    { kErrorEdrvNoFreeBufEntry,       "No free entry in internal buffer table for Tx frames"},
    { kErrorEdrvBufNotExisting,       "Specified Tx buffer does not exist"},
    { kErrorEdrvInvalidRxBuf,         "Specified Rx buffer is invalid"},
    { kErrorEdrvInvalidParam,         "Invalid parameter in function call"},
    { kErrorEdrvNextTxListNotEmpty,   "Next Tx buffer list is not empty, i.e. still in use"},
    { kErrorEdrvCurTxListEmpty,       "Current Tx buffer list is empty, i.e. DLL didn't provide one"},
    { kErrorEdrvTxListNotFinishedYet, "Current Tx buffer list has not been finished yet, but new cycle has started"},


    /* area for DLL module 0x0020 - 0x002F */
    { kErrorDllOutOfMemory,           "Out of memory"},
    { kErrorDllIllegalHdl,            "Illegal handle for a TxFrame was passed"},
    { kErrorDllCbAsyncRegistered,     "Handler for non-POWERLINK frames had already been registered before"},
    { kErrorDllAsyncSyncReqFull,      "Buffer for SyncRequests is full"},
    { kErrorDllAsyncTxBufferEmpty,    "Transmit buffer for asynchronous frames is empty"},
    { kErrorDllAsyncTxBufferFull,     "Transmit buffer for asynchronous frames is full"},
    { kErrorDllNoNodeInfo,            "MN: Too less space in the internal node info structure"},
    { kErrorDllInvalidParam,          "Invalid parameters passed to function"},
    { kErrorDllInvalidAsndServiceId,  "Invalid AsndServiceId specified"},
    { kErrorDllTxBufNotReady,         "TxBuffer (e.g. for PReq) is not ready yet"},
    { kErrorDllTxFrameInvalid,        "TxFrame (e.g. for PReq) is invalid or does not exist"},

    /* area for OBD module 0x0030 - 0x003F */
    { kErrorObdIllegalPart,           "Unknown OD part"},
    { kErrorObdIndexNotExist,         "Object index does not exist in the OD"},
    { kErrorObdSubindexNotExist,      "Subindex does not exist in the object index"},
    { kErrorObdReadViolation,         "Read access to a write-only object"},
    { kErrorObdWriteViolation,        "Write access to a read-only object"},
    { kErrorObdAccessViolation,       "Access not allowed"},
    { kErrorObdUnknownObjectType,     "Object type not defined or unknown"},
    { kErrorObdVarEntryNotExist,      "Object does not contain VarEntry structure"},
    { kErrorObdValueTooLow,           "Value to write to the object is too low"},
    { kErrorObdValueTooHigh,          "Value to write to the object is too high"},
    { kErrorObdValueLengthError,      "Value to write is too long or too short"},
    { kErrorObdErrnoSet,              "File I/O error occurred and errno is set"},
    { kErrorObdInvalidDcf,            "Device configuration file (CDC) is not valid"},
    { kErrorObdOutOfMemory,           "Out of memory"},
    { kErrorObdNoConfigData,          "No configuration data present (CDC is empty)"},


    /* area for NMT module 0x0040 - 0x004F */
    { kErrorNmtUnknownCommand,        "Unknown NMT command"},
    { kErrorNmtInvalidFramePointer,   "Pointer to the frame is not valid"},
    { kErrorNmtInvalidEvent,          "Invalid event sent to NMT module"},
    { kErrorNmtInvalidState,          "Unknown state in NMT state machine"},
    { kErrorNmtInvalidParam,          "Invalid parameters specified"},
    { kErrorNmtSyncReqRejected,       "SyncReq could not be issued"},

    /* area for SDO/UDP module 0x0050 - 0x005F */
    { kErrorSdoUdpMissCb,             "Missing callback-function pointer during init of module"},
    { kErrorSdoUdpNoSocket,           "Error during init of socket"},
    { kErrorSdoUdpSocketError,        "Error during usage of socket"},
    { kErrorSdoUdpThreadError,        "Error during start of listen thread"},
    { kErrorSdoUdpNoFreeHandle,       "No free connection handle for UDP"},
    { kErrorSdoUdpSendError,          "Error during sending the frame"},
    { kErrorSdoUdpInvalidHdl,         "The connection handle is invalid"},

    /* area for SDO Sequence layer module 0x0060 - 0x006F */
    { kErrorSdoSeqMissCb,             "No callback-function assigned"},
    { kErrorSdoSeqNoFreeHandle,       "No free handle for connection"},
    { kErrorSdoSeqInvalidHdl,         "Invalid handle in SDO sequence layer"},
    { kErrorSdoSeqUnsupportedProt,    "Unsupported protocol selected"},
    { kErrorSdoSeqNoFreeHistory,      "No free history entry"},
    { kErrorSdoSeqFrameSizeError,     "The size of the frames is not correct"},
    { kErrorSdoSeqRequestAckNeeded,   "Indicates that the history buffer is full and an ACK request is needed"},
    { kErrorSdoSeqInvalidFrame,       "Frame not valid"},
    { kErrorSdoSeqConnectionBusy,     "Connection is busy -> retry later"},
    { kErrorSdoSeqInvalidEvent,       "Invalid event received"},

    /* area for SDO Command Layer module 0x0070 - 0x007F */
    { kErrorSdoComUnsupportedProt,    "Unsupported Protocol selected"},
    { kErrorSdoComNoFreeHandle,       "No free handle for connection"},
    { kErrorSdoComInvalidServiceType, "Invalid SDO service type specified"},
    { kErrorSdoComInvalidHandle,      "Handle invalid"},
    { kErrorSdoComInvalidSendType,    "The passed frame type is invalid for sending"},
    { kErrorSdoComNotResponsible,     "Internal error: command layer handle is not responsible for this event from sequence layer"},
    { kErrorSdoComHandleExists,       "Handle to same node already exists"},
    { kErrorSdoComHandleBusy,         "Transfer via this handle is already running"},
    { kErrorSdoComInvalidParam,       "Invalid parameters passed to function"},

    /* area for openPOWERLINK event module 0x0080 - 0x008F */
    { kErrorEventUnknownSink,         "Unknown sink for event"},
    { kErrorEventPostError,           "Error during posting the event"},
    { kErrorEventReadError,           "Error during reading the event from queue"},
    { kErrorEventWrongSize,           "Event arg has wrong size"},

    /* area for openPOWERLINK timer module 0x0090 - 0x009F */
    { kErrorTimerInvalidHandle,       "Invalid handle for timer"},
    { kErrorTimerNoTimerCreated,      "No timer was created because of an error"},
    { kErrorTimerThreadError,         "Process thread could not be created"},

    /* area for openPOWERLINK SDO/Asnd module 0x00A0 - 0x0AF */
    { kErrorSdoAsndInvalidNodeId,     "Node id is invalid"},
    { kErrorSdoAsndNoFreeHandle,      "No free handle for connection"},
    { kErrorSdoAsndInvalidHandle,     "Handle for connection is invalid"},

    /* area for PDO module 0x00B0 - 0x00BF  */
    { kErrorPdoNotExist,              "Selected PDO does not exist"},
    { kErrorPdoLengthExceeded,        "Length of PDO mapping exceeds the current payload limit"},
    { kErrorPdoGranularityMismatch,   "Configured PDO granularity is not equal to supported granularity"},
    { kErrorPdoInitError,             "Error during initialization of PDO module"},
    { kErrorPdoConfWhileEnabled,      "PDO configuration cannot be changed while it is enabled"},
    { kErrorPdoErrorMapp,             "Invalid PDO mapping"},
    { kErrorPdoVarNotFound,           "The referenced object in a PDO mapping does not exist"},
    { kErrorPdoVarNotMappable,        "The referenced object in a PDO mapping is not mappable"},
    { kErrorPdoSizeMismatch,          "Bit size of object mapping is larger than the object size"},
    { kErrorPdoTooManyTxPdos,         "Too many TPDOs are existing (only one TPDO is allowed on a CN)"},
    { kErrorPdoInvalidObjIndex,       "Invalid object index used for PDO mapping or communication parameter"},
    { kErrorPdoTooManyPdos,           "Too many PDOs are existing"},

    /* Configuration manager module 0x00C0 - 0x00CF */
    { kErrorCfmConfigError,           "Error in configuration manager"},
    { kErrorCfmSdocTimeOutError,      "Error in configuration manager due to an SDO timeout"},
    { kErrorCfmInvalidDcf,            "Device configuration file (CDC) is not valid"},
    { kErrorCfmUnsupportedDcf,        "Unsupported DCF format"},
    { kErrorCfmConfigWithErrors,      "Configuration finished with errors"},
    { kErrorCfmNoFreeConfig,          "No free configuration entry"},
    { kErrorCfmNoConfigData,          "No configuration data present"},
    { kErrorCfmUnsuppDatatypeDcf,     "Unsupported datatype found in DCF -> this entry was not configured"},

    /* area for OD configuration store restore module 0x0D0 - 0x0DF */
    { kErrorObdStoreHwError,           "HW error while accessing non-volatile memory"},
    { kErrorObdStoreInvalidState,      "Non-volatile memory is in invalid state (nothing saved)"},
    { kErrorObdStoreDataLimitExceeded, "Data count is less than the expected size"},
    { kErrorObdStoreDataObsolete,      "Data stored in the archive is obsolete"},

    { kErrorApiTaskDeferred,          "openPOWERLINK performs a task in the background and informs the application (or vice-versa) when it is finished"},
    { kErrorApiInvalidParam,          "Invalid parameters were passed to a function (e.g. invalid node id)"},
    { kErrorApiNoObdInitRam,          "No function pointer for ObdInitRam supplied"},
    { kErrorApiSdoBusyIntern,         "The SDO channel to this node is internally used by the stack (e.g. the CFM) and currently not available for the application."},
    { kErrorApiPIAlreadyAllocated,    "Process image is already allocated"},
    { kErrorApiPIOutOfMemory,         "Process image: out of memory"},
    { kErrorApiPISizeExceeded,        "Process image: variable linking or copy job exceeds the size of the PI"},
    { kErrorApiPINotAllocated,        "Process image is not allocated"},
    { kErrorApiPIJobQueueFull,        "Process image: job queue is full"},
    { kErrorApiPIJobQueueEmpty,       "Process image: job queue is empty"},
    { kErrorApiPIInvalidJobSize,      "Process image: invalid job size"},
    { kErrorApiPIInvalidPIPointer,    "Process image: pointer to application's process image is invalid"},
    { kErrorApiPINonBlockingNotSupp,  "Process image: non-blocking copy jobs are not supported on this target"},
    { kErrorApiNotInitialized,        "API called but stack is not initialized/running"},
};

static const tEmergErrCodeInfo emergErrCodeInfo_l[] =
{
    { E_NO_ERROR,                    "E_NO_ERROR"               },

    // 0xFxxx manufacturer specific error codes
    { E_NMT_NO_IDENT_RES,            "E_NMT_NO_IDENT_RES"       },
    { E_NMT_NO_STATUS_RES,           "E_NMT_NO_STATUS_RES"      },

    // 0x816x HW errors
    { E_DLL_BAD_PHYS_MODE,           "E_DLL_BAD_PHYS_MODE"      },
    { E_DLL_COLLISION,               "E_DLL_COLLISION"          },
    { E_DLL_COLLISION_TH,            "E_DLL_COLLISION_TH"       },
    { E_DLL_CRC_TH,                  "E_DLL_CRC_TH "            },
    { E_DLL_LOSS_OF_LINK,            "E_DLL_LOSS_OF_LINK",      },
    { E_DLL_MAC_BUFFER,              "E_DLL_MAC_BUFFER",        },

    // 0x82xx Protocol errors
    { E_DLL_ADDRESS_CONFLICT,        "E_DLL_ADDRESS_CONFLICT"   },
    { E_DLL_MULTIPLE_MN,             "E_DLL_MULTIPLE_MN"        },

    // 0x821x Frame size errors
    { E_PDO_SHORT_RX,                "E_PDO_SHORT_RX"           },
    { E_PDO_MAP_VERS,                "E_PDO_MAP_VERS"           },
    { E_NMT_ASND_MTU_DIF,            "E_NMT_ASND_MTU_DIF"       },
    { E_NMT_ASND_MTU_LIM,            "E_NMT_ASND_MTU_LIM"       },
    { E_NMT_ASND_TX_LIM,             "E_NMT_ASND_TX_LIM"        },

    // 0x823x Timing errors
    { E_NMT_CYCLE_LEN,               "E_NMT_CYCLE_LEN"          },
    { E_DLL_CYCLE_EXCEED,            "E_DLL_CYCLE_EXCEED"       },
    { E_DLL_CYCLE_EXCEED_TH,         "E_DLL_CYCLE_EXCEED_TH"    },
    { E_NMT_IDLE_LIM,                "E_NMT_IDLE_LIM"           },
    { E_DLL_JITTER_TH,               "E_DLL_JITTER_TH"          },
    { E_DLL_LATE_PRES_TH,            "E_DLL_LATE_PRES_TH"       },
    { E_NMT_PREQ_CN,                 "E_NMT_PREQ_CN"            },
    { E_NMT_PREQ_LIM,                "E_NMT_PREQ_LIM"           },
    { E_NMT_PRES_CN,                 "E_NMT_PRES_CN"            },
    { E_NMT_PRES_RX_LIM,             "E_NMT_PRES_RX_LIM"        },
    { E_NMT_PRES_TX_LIM,             "E_NMT_PRES_TX_LIM"        },

    // 0x824x Frame errors
    { E_DLL_INVALID_FORMAT,          "E_DLL_INVALID_FORMAT"     },
    { E_DLL_LOSS_PREQ_TH,            "E_DLL_LOSS_PREQ_TH"       },
    { E_DLL_LOSS_PRES_TH,            "E_DLL_LOSS_PRES_TH"       },
    { E_DLL_LOSS_SOA_TH,             "E_DLL_LOSS_SOA_TH"        },
    { E_DLL_LOSS_SOC_TH,             "E_DLL_LOSS_SOC_TH"        },

    // 0x84xx BootUp Errors
    { E_NMT_BA1,                     "E_NMT_BA1"                },
    { E_NMT_BA1_NO_MN_SUPPORT,       "E_NMT_BA1_NO_MN_SUPPORT"  },
    { E_NMT_BPO1,                    "E_NMT_BPO1"               },
    { E_NMT_BPO1_GET_IDENT,          "E_NMT_BPO1_GET_IDENT"     },
    { E_NMT_BPO1_DEVICE_TYPE,        "E_NMT_BPO1_DEVICE_TYPE"   },
    { E_NMT_BPO1_VENDOR_ID,          "E_NMT_BPO1_VENDOR_ID"     },
    { E_NMT_BPO1_PRODUCT_CODE,       "E_NMT_BPO1_PRODUCT_CODE"  },
    { E_NMT_BPO1_REVISION_NO,        "E_NMT_BPO1_REVISION_NO"   },
    { E_NMT_BPO1_SERIAL_NO,          "E_NMT_BPO1_SERIAL_NO"     },
    { E_NMT_BPO1_CF_VERIFY,          "E_NMT_BPO1_CF_VERIFY"     },
    { E_NMT_BPO1_SW_UPDATE,          "E_NMT_BPO1_SW_UPDATE"     },
    { E_NMT_BPO2,                    "E_NMT_BPO2"               },
    { E_NMT_BRO,                     "E_NMT_BRO"                },
    { E_NMT_WRONG_STATE,             "E_NMT_WRONG_STATE"        },
};

// text strings for NMT node events
static const char* nmtNodeEventTypeStr_l[] =
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

// text strings for NMT boot events
static const char* nmtBootEventTypeStr_l[] =
{
    "Boot step 1 finished",     // 0x00     PreOp2 is possible
    "Boot step 2 finished",     // 0x01     ReadyToOp is possible for MN
    "EnableReadyToOp",          // 0x02     ReadyToOP is possible for CN
    "CheckComFinish",           // 0x03     Operational is possible
    "Operational",              // 0x04     all mandatory CNs are Operational
    "Error",                    // 0x05
};

// text strings for SDO command layer connection states
static const char* sdoComConStateStr_l[] =
{
    "NotActive",                // 0x00
    "Running",                  // 0x01
    "TxAborted",                // 0x02
    "RxAborted",                // 0x03
    "Finished",                 // 0x04
    "LowerLayerAbort",          // 0x05
};

// text strings for abort codes
static const tAbortCodeInfo sdoAbortCodeInfo_l[] =
{
    { SDO_AC_TIME_OUT,                          "SDO_AC_TIME_OUT" },
    { SDO_AC_UNKNOWN_COMMAND_SPECIFIER,         "SDO_AC_UNKNOWN_COMMAND_SPECIFIER" },
    { SDO_AC_INVALID_BLOCK_SIZE,                "SDO_AC_INVALID_BLOCK_SIZE" },
    { SDO_AC_INVALID_SEQUENCE_NUMBER,           "SDO_AC_INVALID_SEQUENCE_NUMBER" },
    { SDO_AC_OUT_OF_MEMORY,                     "SDO_AC_OUT_OF_MEMORY" },
    { SDO_AC_UNSUPPORTED_ACCESS,                "SDO_AC_UNSUPPORTED_ACCESS" },
    { SDO_AC_READ_TO_WRITE_ONLY_OBJ,            "SDO_AC_READ_TO_WRITE_ONLY_OBJ" },
    { SDO_AC_WRITE_TO_READ_ONLY_OBJ,            "SDO_AC_WRITE_TO_READ_ONLY_OBJ" },
    { SDO_AC_OBJECT_NOT_EXIST,                  "SDO_AC_OBJECT_NOT_EXIST" },
    { SDO_AC_OBJECT_NOT_MAPPABLE,               "SDO_AC_OBJECT_NOT_MAPPABLE" },
    { SDO_AC_PDO_LENGTH_EXCEEDED,               "SDO_AC_PDO_LENGTH_EXCEEDED" },
    { SDO_AC_GEN_PARAM_INCOMPATIBILITY,         "SDO_AC_GEN_PARAM_INCOMPATIBILITY" },
    { SDO_AC_INVALID_HEARTBEAT_DEC,             "SDO_AC_INVALID_HEARTBEAT_DEC" },
    { SDO_AC_GEN_INTERNAL_INCOMPATIBILITY,      "SDO_AC_GEN_INTERNAL_INCOMPATIBILITY" },
    { SDO_AC_ACCESS_FAILED_DUE_HW_ERROR,        "SDO_AC_ACCESS_FAILED_DUE_HW_ERROR" },
    { SDO_AC_DATA_TYPE_LENGTH_NOT_MATCH,        "SDO_AC_DATA_TYPE_LENGTH_NOT_MATCH" },
    { SDO_AC_DATA_TYPE_LENGTH_TOO_HIGH,         "SDO_AC_DATA_TYPE_LENGTH_TOO_HIGH" },
    { SDO_AC_DATA_TYPE_LENGTH_TOO_LOW,          "SDO_AC_DATA_TYPE_LENGTH_TOO_LOW" },
    { SDO_AC_SUB_INDEX_NOT_EXIST,               "SDO_AC_SUB_INDEX_NOT_EXIST" },
    { SDO_AC_VALUE_RANGE_EXCEEDED,              "SDO_AC_VALUE_RANGE_EXCEEDED" },
    { SDO_AC_VALUE_RANGE_TOO_HIGH,              "SDO_AC_VALUE_RANGE_TOO_HIGH" },
    { SDO_AC_VALUE_RANGE_TOO_LOW,               "SDO_AC_VALUE_RANGE_TOO_LOW" },
    { SDO_AC_MAX_VALUE_LESS_MIN_VALUE,          "SDO_AC_MAX_VALUE_LESS_MIN_VALUE" },
    { SDO_AC_GENERAL_ERROR,                     "SDO_AC_GENERAL_ERROR" },
    { SDO_AC_DATA_NOT_TRANSF_OR_STORED,         "SDO_AC_DATA_NOT_TRANSF_OR_STORED" },
    { SDO_AC_DATA_NOT_TRANSF_DUE_LOCAL_CONTROL, "SDO_AC_DATA_NOT_TRANSF_DUE_LOCAL_CONTROL" },
    { SDO_AC_DATA_NOT_TRANSF_DUE_DEVICE_STATE,  "SDO_AC_DATA_NOT_TRANSF_DUE_DEVICE_STATE" },
    { SDO_AC_OBJECT_DICTIONARY_NOT_EXIST,       "SDO_AC_OBJECT_DICTIONARY_NOT_EXIST" },
    { SDO_AC_CONFIG_DATA_EMPTY,                 "SDO_AC_CONFIG_DATA_EMPTY" },
    { 0,                                        "SDO_AC_OK" }
};

static const tNmtNodeCommandInfo nmtNodeCommandInfo_l[] =
{
    { kNmtNodeCommandBoot,         "NmtNodeCommandBoot" },
    { kNmtNodeCommandSwOk,         "NmtNodeCommandSwOk" },
    { kNmtNodeCommandSwUpdated,    "NmtNodeCommandSwUpdated" },
    { kNmtNodeCommandConfOk,       "NmtNodeCommandConfOk" },
    { kNmtNodeCommandConfRestored, "NmtNodeCommandConfRestored" },
    { kNmtNodeCommandConfReset,    "NmtNodeCommandConfReset" },
    { kNmtNodeCommandConfErr,      "NmtNodeCommandConfErr" },
    { kNmtNodeCommandStart,        "NmtNodeCommandStart" },
    { kNmtNodeCommandSwErr,        "kNmtNodeCommandSwErr" },
};

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Return the string of the specified event

The function returns the string describing the specified event.

\param[in]      nmtEvent_p          Event to print

\return The function returns a string describing the specified event.

\ingroup module_debugstr
*/
//------------------------------------------------------------------------------
const char* debugstr_getNmtEventStr(tNmtEvent nmtEvent_p)
{
    if (nmtEvent_p >= tabentries(nmtEventStr_l))
        return invalidStr_l;
    else
        return nmtEventStr_l[nmtEvent_p];
}

//------------------------------------------------------------------------------
/**
\brief  Return the string of the specified event source

The function returns the string describing the specified event source.

\param[in]      eventSrc_p          Event source to print

\return The function returns a string describing the specified event source.

\ingroup module_debugstr
*/
//------------------------------------------------------------------------------
const char* debugstr_getEventSourceStr(tEventSource eventSrc_p)
{
    if (eventSrc_p >= tabentries(eventSourceStr_l))
        return invalidStr_l;
    else
        return eventSourceStr_l[eventSrc_p];
}

//------------------------------------------------------------------------------
/**
\brief  Return the string of the specified event sink

The function returns the string describing the specified event sink.

\param[in]      eventSink_p         Event sink to print

\return The function returns a string describing the specified event sink.

\ingroup module_debugstr
*/
//------------------------------------------------------------------------------
const char* debugstr_getEventSinkStr(tEventSink eventSink_p)
{
    if (eventSink_p >= tabentries(eventSinkStr_l))
        return invalidStr_l;
    else
        return eventSinkStr_l[eventSink_p];
}

//------------------------------------------------------------------------------
/**
\brief  Return the string of the specified event type

The function returns the string describing the specified event type.

\param[in]      eventType_p         Event type to print

\return The function returns a string describing the specified event type.

\ingroup module_debug
*/
//------------------------------------------------------------------------------
const char* debugstr_getEventTypeStr(tEventType eventType_p)
{
    if (eventType_p >= tabentries(eventTypeStr_l))
        return invalidStr_l;
    else
        return eventTypeStr_l[eventType_p];
}

//------------------------------------------------------------------------------
/**
\brief  Return the string of the specified NMT state

The function returns the string describing the specified NMT state.

\param[in]      nmtState_p          NMT state to print

\return The function returns a string describing the specified NMT state.

\ingroup module_debugstr
*/
//------------------------------------------------------------------------------
const char* debugstr_getNmtStateStr(tNmtState nmtState_p)
{
    unsigned int    i;

    for (i = 0; i < tabentries(nmtStateInfo_l); i++)
    {
        if (nmtStateInfo_l[i].nmtState == nmtState_p)
            return nmtStateInfo_l[i].sNmtState;
    }

    return invalidStr_l;
}

//------------------------------------------------------------------------------
/**
\brief  Return the string of the specified API event

The function returns the string describing the specified API event.

\param[in]      apiEvent_p          API event to print

\return The function returns a string describing the specified API event.

\ingroup module_debugstr
*/
//------------------------------------------------------------------------------
const char* debugstr_getApiEventStr(tOplkApiEventType apiEvent_p)
{
    unsigned int    i;

    for (i = 0; i < tabentries(apiEventInfo_l); i++)
    {
        if (apiEventInfo_l[i].apiEvent == apiEvent_p)
            return apiEventInfo_l[i].sApiEvent;
    }

    return invalidStr_l;
}

//------------------------------------------------------------------------------
/**
\brief  Return the string of the specified NMT node event

The function returns the string describing the specified NMT node event.

\param[in]      nodeEventType_p     NMT node event to print

\return The function returns a string describing the specified NMT node event.

\ingroup module_debugstr
*/
//------------------------------------------------------------------------------
const char* debugstr_getNmtNodeEventTypeStr(tNmtNodeEvent nodeEventType_p)
{
    if (nodeEventType_p >= tabentries(nmtNodeEventTypeStr_l))
        return invalidStr_l;
    else
        return nmtNodeEventTypeStr_l[nodeEventType_p];
}

//------------------------------------------------------------------------------
/**
\brief  Return the string of the specified NMT boot event

The function returns the string describing the specified NMT boot event.

\param[in]      bootEventType_p     NMT boot event to print

\return The function returns a string describing the specified NMT boot event.

\ingroup module_debugstr
*/
//------------------------------------------------------------------------------
const char* debugstr_getNmtBootEventTypeStr(tNmtBootEvent bootEventType_p)
{
    if (bootEventType_p >= tabentries(nmtBootEventTypeStr_l))
        return invalidStr_l;
    else
        return nmtBootEventTypeStr_l[bootEventType_p];
}

//------------------------------------------------------------------------------
/**
\brief  Return the string of the specified NMT node command

The function returns the string describing the specified NMT node command.

\param[in]      nodeCommand_p       NMT node command to print

\return The function returns a string describing the specified NMT node command.

\ingroup module_debugstr
*/
//------------------------------------------------------------------------------
const char* debugstr_getNmtNodeCommandTypeStr(tNmtNodeCommand nodeCommand_p)
{
    unsigned int    i;

    for (i = 0; i < tabentries(nmtNodeCommandInfo_l); i++)
    {
        if (nmtNodeCommandInfo_l[i].nodeCommand == nodeCommand_p)
            return nmtNodeCommandInfo_l[i].sNodeCommand;
    }

    return invalidStr_l;
}

//------------------------------------------------------------------------------
/**
\brief  Return the string of the specified SDO command connection state

The function returns the string describing the specified SDO command connection
state.

\param[in]      sdoComConState_p    SDO command connection state to print

\return The function returns a string describing the specified SDO command
connection state.

\ingroup module_debugstr
*/
//------------------------------------------------------------------------------
const char* debugstr_getSdoComConStateStr(tSdoComConState sdoComConState_p)
{
    if (sdoComConState_p >= tabentries(sdoComConStateStr_l))
        return invalidStr_l;
    else
        return sdoComConStateStr_l[sdoComConState_p];
}

//------------------------------------------------------------------------------
/**
\brief  Return the string of the specified SDO command connection state

The function returns the string describing the given entry of type tOplkError.

\param[in]      oplkError_p         tOplkError value to print

\return The function returns a string describing the specified tOplkError type.

\ingroup module_debugstr
*/
//------------------------------------------------------------------------------
const char* debugstr_getRetValStr(tOplkError oplkError_p)
{
    unsigned int    i;

    for (i = 0; i < tabentries(retValInfo_l); i++)
    {
        if (retValInfo_l[i].key == oplkError_p)
            return retValInfo_l[i].sName;
    }

    return invalidStr_l;
}

//------------------------------------------------------------------------------
/**
\brief  Return the string describing the specified emergency error code

The function returns the string describing the specified emergency error code.

\param[in]      emergErrCode_p      Emergency error code value to print

\return The function returns a string describing the specified emergency error
code.

\ingroup module_debugstr
*/
//------------------------------------------------------------------------------
const char* debugstr_getEmergErrCodeStr(UINT16 emergErrCode_p)
{

    unsigned int    i;

    for (i = 0; i < tabentries(emergErrCodeInfo_l); i++)
    {
        if (emergErrCodeInfo_l[i].key == emergErrCode_p)
            return emergErrCodeInfo_l[i].sName;
    }

    return invalidStr_l;
}

//------------------------------------------------------------------------------
/**
\brief  Return the string describing the specified abort code

The function returns the string describing the specified abort code.

\param[in]      abortCode_p         Abort code value to print

\return The function returns a string describing the specified abort code.

\ingroup module_debugstr
*/
//------------------------------------------------------------------------------
const char* debugstr_getAbortCodeStr(UINT32 abortCode_p)
{
    const tAbortCodeInfo*   pEntry;
    unsigned int            i;

    pEntry = sdoAbortCodeInfo_l;
    for (i = 0; i < tabentries(sdoAbortCodeInfo_l); i++)
    {
        if (pEntry->abortCode == abortCode_p)
            return pEntry->sAbortCode;

        pEntry++;
    }

    return invalidStr_l;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

/// \}
