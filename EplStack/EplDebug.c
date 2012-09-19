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
typedef struct
{
    tEplNmtState        m_nmtState;
    char                *m_sNmtState;
} tNmtStateInfo;

typedef struct
{
    tEplApiEventType    m_ApiEvent;
    char                *m_sApiEvent;
} tApiEventInfo;

typedef struct
{
    tEplKernel          m_Key;
    char                *m_sName;
} tEplDebugEplKernelInfo;

typedef struct
{
    WORD                m_Key;
    char                *m_sName;
} tEplEmergErrCodeInfo;

//=========================================================================//
// Module global vars                                                      //
//=========================================================================//

static char *eplInvalidStr_g = "INVALID";

// text strings for POWERLINK events
static char *eplEvtStr_g[] =
{
    "NmtEventNoEvent",              //
    "NmtEventDllMePres",            //
    "NmtEventDllMePresTimeout",     //
    "NmtEventDllMeAsndTimeout",     //
    "NmtEventDllMeSoaSent",         //
    "NmtEventDllMeSocTrig",         //
    "NmtEventDllMeSoaTrig",         //
    "NmtEventDllCeSoc",             //
    "NmtEventDllCePreq",            //
    "NmtEventDllCePres",            //
    "NmtEventDllCeSoa",             //
    "NmtEventDllCeAsnd",            //
    "NmtEventDllCeFrameTimeout",    //
    "0xd",                          // reserved
    "0xe",                          // reserved
    "0xf",                          // reserved
    "NmtEventSwReset",              // NMT_GT1, NMT_GT2, NMT_GT8
    "NmtEventResetNode",            //
    "NmtEventResetCom",             //
    "NmtEventResetConfig",          //
    "NmtEventEnterPreOperational2", //
    "NmtEventEnableReadyToOperate", //
    "NmtEventStartNode",            // NMT_CT7
    "NmtEventStopNode",             //
    "0x18",                         // reserved
    "0x19",                         // reserved
    "0x1A",                         // reserved
    "0x1B",                         // reserved
    "0x1C",                         // reserved
    "0x1D",                         // reserved
    "0x1E",                         // reserved
    "0x1F",                         // reserved
    "NmtEventEnterResetApp",        //
    "NmtEventEnterResetCom",        //
    "NmtEventInternComError",       // NMT_GT6, internal communication error -> enter ResetCommunication
    "NmtEventEnterResetConfig",     //
    "NmtEventEnterCsNotActive",     //
    "NmtEventEnterMsNotActive",     //
    "NmtEventTimerBasicEthernet",   // NMT_CT3; timer triggered state change (NotActive -> BasicEth)
    "NmtEventTimerMsPreOp1",        // enter PreOp1 on MN (NotActive -> MsPreOp1)
    "NmtEventNmtCycleError",        // NMT_CT11, NMT_MT6; error during cycle -> enter PreOp1
    "NmtEventTimerMsPreOp2",        // enter PreOp2 on MN (MsPreOp1 -> MsPreOp2 if NmtEventAllMandatoryCNIdent)
    "NmtEventAllMandatoryCNIdent",  // enter PreOp2 on MN if NmtEventTimerMsPreOp2
    "NmtEventEnterReadyToOperate",  // application ready for the state ReadyToOp
    "NmtEventEnterMsOperational",   // enter Operational on MN
    "NmtEventSwitchOff",            // enter state Off
    "NmtEventCriticalError",        // enter state Off because of critical error
};

// text strings for POWERLINK event sources
static char    *eplEvtSrcStr_g[] =
{
    "0",                        // reserved

    // kernelspace modules
    "EventSourceDllk",          // Dllk module
    "EventSourceNmtk",          // Nmtk module
    "EventSourceObdk",          // Obdk module
    "EventSourcePdok",          // Pdok module
    "EventSourceTimerk",        // Timerk module
    "EventSourceEventk",        // Eventk module
    "EventSourceSyncCb",        // sync-Cb
    "EventSourceErrk",          // Error handler module
    "0x09", "0x0a", "0x0b",     // reserved
    "0x0c", "0x0d", "0x0e",     // reserved
    "0x0f",                     // reserved

    // userspace modules
    "EventSourceDllu",          // Dllu module
    "EventSourceNmtu",          // Nmtu module
    "EventSourceNmtCnu",        // NmtCnu module
    "EventSourceNmtMnu",        // NmtMnu module
    "EventSourceObdu",          // Obdu module
    "EventSourceSdoUdp",        // Sdo/Udp module
    "EventSourceSdoAsnd",       // Sdo/Asnd module
    "EventSourceSdoAsySeq",     // Sdo asynchronous Sequence Layer module
    "EventSourceSdoCom",        // Sdo command layer module
    "EventSourceTimeru",        // Timeru module
    "EventSourceCfgMau",        // CfgMau module
    "EventSourceEventu",        // Eventu module
    "EventSourceEplApi",        // Api module
    "EventSourceLedu",          // Ledu module
    "EventSourceGw309Ascii",    // GW309ASCII module
};

// text strings for POWERLINK event sinks
static char * eplEvtSinkStr_g[] =
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
    "EventSinkLedu",
    "EventSinkPdokCal",
    "EventSinkGw309Ascii",
    "EventSinkApi"
};

// text strings for POWERLINK event types
static char *eplEvtTypeStr_g[] =
{
    "0",                                // reserved
    "EventTypeNmtEvent",                // NMT event
    "EventTypePdoRx",                   // PDO frame received event (PRes/PReq)
    "EventTypePdoTx",                   // PDO frame transmitted event (PRes/PReq)
    "EventTypePdoSoa",                  // SoA frame received event (isochronous phase completed)
    "EventTypeSync",                    // Sync event (e.g. SoC or anticipated SoC)
    "EventTypeTimer",                   // Timer event
    "EventTypeHeartbeat",               // Heartbeat event
    "EventTypeHistoryEntry",            // Error history entry event
    "EventTypeDllkFlag1",               // DLL kernel Flag 1 changed event
    "EventTypeDllkFillTx",              // DLL kernel fill TxBuffer event
    "EventTypeDllkPresReady",           // DLL kernel PRes ready event
    "EventTypeError",                   // Error event for API layer
    "EventTypeNmtStateChange",          // indicate change of NMT-State
    "EventTypeDllError",                // DLL error event for Error handler
    "EventTypeAsndRx",                  // received ASnd frame for DLL user module
    "EventTypeDllkServFilter",          // configure ServiceIdFilter
    "EventTypeDllkIdentity",            // configure Identity
    "EventTypeDllkConfig",              // configure ConfigParam
    "EventTypeDllkIssueReq",            // issue Ident/Status request
    "EventTypeDllkAddNode",             // add node to isochronous phase
    "EventTypeDllkDelNode",             // remove node from isochronous phase
    "EventTypeDllkConfigNode",          // configures parameters of node
    "EventTypeDllkStartReducedCycle",   // start reduced EPL cycle on MN
    "EventTypeNmtMnuNmtCmdSent",        // NMT command was actually sent
    "EventTypeApiUserDef",              // user-defined event
    "EventTypeDllkCycleFinish",         // SoA sent, cycle finished
    "0x1B", "0x1C", "0x1D",             // reserved
    "0x1E", "0x1F",                     // reserved
    "EventTypePdokAlloc",               // alloc PDOs
    "EventTypePdokConfig",              // configure PDO channel
    "EventTypeNmtMnuNodeCmd",           // trigger NMT node command
    "EventTypeGw309AsciiReq",           // GW309ASCII request
    "EventTypeNmtMnuNodeAdded",         // node was added to isochronous phase by DLL
};

// text strings for POWERLINK states
static tNmtStateInfo nmtStateInfo_g[] =
{
    { kEplNmtGsOff,                 "NmtGsOff"                  },
    { kEplNmtGsInitialising,        "NmtGsInitializing"         },
    { kEplNmtGsResetApplication,    "NmtGsResetApplication"     },
    { kEplNmtGsResetCommunication,  "NmtGsResetCommunication"   },
    { kEplNmtGsResetConfiguration,  "NmtGsResetConfiguration"   },
    { kEplNmtCsNotActive,           "NmtCsNotActive"            },
    { kEplNmtCsPreOperational1,     "NmtCsPreOperational1"      },
    { kEplNmtCsStopped,             "NmtCsStopped"              },
    { kEplNmtCsPreOperational2,     "NmtCsPreOperational2"      },
    { kEplNmtCsReadyToOperate,      "NmtCsReadyToOperate"       },
    { kEplNmtCsOperational,         "NmtCsOperational"          },
    { kEplNmtCsBasicEthernet,       "NmtCsBasicEthernet"        },
    { kEplNmtMsNotActive,           "NmtMsNotActive"            },
    { kEplNmtMsPreOperational1,     "NmtMsPreOperational1"      },
    { kEplNmtMsPreOperational2,     "NmtMsPreOperational2"      },
    { kEplNmtMsReadyToOperate,      "NmtMsReadyToOperate"       },
    { kEplNmtMsOperational,         "NmtMsOperational"          },
    { kEplNmtMsBasicEthernet,       "NmtMsBasicEthernet"        },
};

// text strings for API events
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
};

// text strings for values of type tEplKernel
static tEplDebugEplKernelInfo EplKernelInfo_g[] =
{
    { kEplSuccessful,                   "kEplSuccessful"                },
    { kEplIllegalInstance,              "kEplIllegalInstance"           },
    { kEplInvalidInstanceParam,         "kEplInvalidInstanceParam"      },
    { kEplNoFreeInstance,               "kEplNoFreeInstance"            },
    { kEplWrongSignature,               "kEplWrongSignature"            },
    { kEplInvalidOperation,             "kEplInvalidOperation"          },
    { kEplInvalidNodeId,                "kEplInvalidNodeId"             },
    { kEplNoResource,                   "kEplNoResource"                },
    { kEplShutdown,                     "kEplShutdown"                  },
    { kEplReject,                       "kEplReject"                    },
    { kEplRetry,                        "kEplRetry"                     },
    { kEplInvalidEvent,                 "kEplInvalidEvent"              },

    { kEplEdrvNoFreeTxDesc,             "kEplEdrvNoFreeTxDesc"          },
    { kEplEdrvInvalidCycleLen,          "kEplEdrvInvalidCycleLen"       },
    { kEplEdrvInitError,                "kEplEdrvInitError"             },
    { kEplEdrvNoFreeBufEntry,           "kEplEdrvNoFreeBufEntry"        },
    { kEplEdrvBufNotExisting,           "kEplEdrvBufNotExisting"        },
    { kEplEdrvInvalidRxBuf,             "kEplEdrvInvalidRxBuf"          },
    { kEplEdrvInvalidParam,             "kEplEdrvInvalidParam"          },
    { kEplEdrvNextTxListNotEmpty,       "kEplEdrvNextTxListNotEmpty"    },
    { kEplEdrvCurTxListEmpty,           "kEplEdrvCurTxListEmpty"        },
    { kEplEdrvTxListNotFinishedYet,      "kEplEdrvTxListNotFinishedYet" },

    { kEplDllOutOfMemory,               "kEplDllOutOfMemory"            },
    { kEplDllIllegalHdl,                "kEplDllIllegalHdl"             },
    { kEplDllCbAsyncRegistered,         "kEplDllCbAsyncRegistered"      },
    { kEplDllAsyncSyncReqFull,          "kEplDllAsyncSyncReqFull"       },
    { kEplDllAsyncTxBufferEmpty,        "kEplDllAsyncTxBufferEmpty"     },
    { kEplDllAsyncTxBufferFull,         "kEplDllAsyncTxBufferFull"      },
    { kEplDllNoNodeInfo,                "kEplDllNoNodeInfo"             },
    { kEplDllInvalidParam,              "kEplDllInvalidParam"           },
    { kEplDllInvalidAsndServiceId,      "kEplDllInvalidAsndServiceId"   },
    { kEplDllTxBufNotReady,             "kEplDllTxBufNotReady"          },
    { kEplDllTxFrameInvalid,            "kEplDllTxFrameInvalid"         },

    { kEplObdIllegalPart,               "kEplObdIllegalPart"            },
    { kEplObdIndexNotExist,             "kEplObdIndexNotExist"          },
    { kEplObdSubindexNotExist,          "kEplObdSubindexNotExist"       },
    { kEplObdReadViolation,             "kEplObdReadViolation"          },
    { kEplObdWriteViolation,            "kEplObdWriteViolation"         },
    { kEplObdAccessViolation,           "kEplObdAccessViolation"        },
    { kEplObdUnknownObjectType,         "kEplObdUnknownObjectType"      },
    { kEplObdVarEntryNotExist,          "kEplObdVarEntryNotExist"       },
    { kEplObdValueTooLow,               "kEplObdValueTooLow"            },
    { kEplObdValueTooHigh,              "kEplObdValueTooHigh"           },
    { kEplObdValueLengthError,          "kEplObdValueLengthError"       },
    { kEplObdErrnoSet,                  "kEplObdErrnoSet"               },
    { kEplObdInvalidDcf,                "kEplObdInvalidDcf"             },
    { kEplObdOutOfMemory,               "kEplObdOutOfMemory"            },
    { kEplObdNoConfigData,              "kEplObdNoConfigData"           },

    { kEplNmtUnknownCommand,            "kEplNmtUnknownCommand"         },
    { kEplNmtInvalidFramePointer,       "kEplNmtInvalidFramePointer"    },
    { kEplNmtInvalidEvent,              "kEplNmtInvalidEvent"           },
    { kEplNmtInvalidState,              "kEplNmtInvalidState"           },
    { kEplNmtInvalidParam,              "kEplNmtInvalidParam"           },
    { kEplNmtSyncReqRejected,           "kEplNmtSyncReqRejected"        },

    { kEplSdoUdpMissCb,                 "kEplSdoUdpMissCb"              },
    { kEplSdoUdpNoSocket,               "kEplSdoUdpNoSocket"            },
    { kEplSdoUdpSocketError,            "kEplSdoUdpSocketError"         },
    { kEplSdoUdpThreadError,            "kEplSdoUdpThreadError"         },
    { kEplSdoUdpNoFreeHandle,           "kEplSdoUdpNoFreeHandle"        },
    { kEplSdoUdpSendError,              "kEplSdoUdpSendError"           },
    { kEplSdoUdpInvalidHdl,             "kEplSdoUdpInvalidHdl"          },

    { kEplSdoSeqMissCb,                 "kEplSdoSeqMissCb"              },
    { kEplSdoSeqNoFreeHandle,           "kEplSdoSeqNoFreeHandle"        },
    { kEplSdoSeqInvalidHdl,             "kEplSdoSeqInvalidHdl"          },
    { kEplSdoSeqUnsupportedProt,        "kEplSdoSeqUnsupportedProt"     },
    { kEplSdoSeqNoFreeHistory,          "kEplSdoSeqNoFreeHistory"       },
    { kEplSdoSeqFrameSizeError,         "kEplSdoSeqFrameSizeError"      },
    { kEplSdoSeqRequestAckNeeded,       "kEplSdoSeqRequestAckNeeded"    },

    { kEplSdoSeqInvalidFrame,           "kEplSdoSeqInvalidFrame"        },
    { kEplSdoSeqConnectionBusy,         "kEplSdoSeqConnectionBusy"      },
    { kEplSdoSeqInvalidEvent,           "kEplSdoSeqInvalidEvent"        },

    { kEplSdoComUnsupportedProt,        "kEplSdoComUnsupportedProt"     },
    { kEplSdoComNoFreeHandle,           "kEplSdoComNoFreeHandle"        },
    { kEplSdoComInvalidServiceType,     "kEplSdoComInvalidServiceType"  },
    { kEplSdoComInvalidHandle,          "kEplSdoComInvalidHandle"       },
    { kEplSdoComInvalidSendType,        "kEplSdoComInvalidSendType"     },
    { kEplSdoComNotResponsible,         "kEplSdoComNotResponsible"      },
    { kEplSdoComHandleExists,           "kEplSdoComHandleExists"        },
    { kEplSdoComHandleBusy,             "kEplSdoComHandleBusy"          },
    { kEplSdoComInvalidParam,           "kEplSdoComInvalidParam"        },

    { kEplEventUnknownSink,             "kEplEventUnknownSink"          },
    { kEplEventPostError,               "kEplEventPostError"            },
    { kEplEventReadError,               "kEplEventReadError"            },
    { kEplEventWrongSize,               "kEplEventWrongSize"            },

    { kEplTimerInvalidHandle,           "kEplTimerInvalidHandle"        },
    { kEplTimerNoTimerCreated,          "kEplTimerNoTimerCreated"       },
    { kEplTimerThreadError,             "kEplTimerThreadError"          },

    { kEplSdoAsndInvalidNodeId,         "kEplSdoAsndInvalidNodeId"      },
    { kEplSdoAsndNoFreeHandle,          "kEplSdoAsndNoFreeHandle"       },
    { kEplSdoAsndInvalidHandle,         "kEplSdoAsndInvalidHandle"      },

    { kEplPdoNotExist,                  "kEplPdoNotExist"               },
    { kEplPdoLengthExceeded,            "kEplPdoLengthExceeded"         },
    { kEplPdoGranularityMismatch,       "kEplPdoGranularityMismatch"    },
    { kEplPdoInitError,                 "kEplPdoInitError"              },
    { kEplPdoConfWhileEnabled,          "kEplPdoConfWhileEnabled"       },
    { kEplPdoErrorMapp,                 "kEplPdoErrorMapp"              },
    { kEplPdoVarNotFound,               "kEplPdoVarNotFound"            },
    { kEplPdoVarNotMappable,            "kEplPdoVarNotMappable"         },

    { kEplPdoSizeMismatch,              "kEplPdoSizeMismatch"           },
    { kEplPdoTooManyTxPdos,             "kEplPdoTooManyTxPdos"          },
    { kEplPdoInvalidObjIndex,           "kEplPdoInvalidObjIndex"        },
    { kEplPdoTooManyPdos,               "kEplPdoTooManyPdos"            },

    { kEplCfmConfigError,               "kEplCfmConfigError"            },
    { kEplCfmSdocTimeOutError,          "kEplCfmSdocTimeOutError"       },
    { kEplCfmInvalidDcf,                "kEplCfmInvalidDcf"             },
    { kEplCfmUnsupportedDcf,            "kEplCfmUnsupportedDcf"         },
    { kEplCfmConfigWithErrors,          "kEplCfmConfigWithErrors"       },
    { kEplCfmNoFreeConfig,              "kEplCfmNoFreeConfig"           },
    { kEplCfmNoConfigData,              "kEplCfmNoConfigData"           },
    { kEplCfmUnsuppDatatypeDcf,         "kEplCfmUnsuppDatatypeDcf"      },

    { kEplApiTaskDeferred,              "kEplApiTaskDeferred"           },
    { kEplApiInvalidParam,              "kEplApiInvalidParam"           },
    { kEplApiNoObdInitRam,              "kEplApiNoObdInitRam"           },
    { kEplApiSdoBusyIntern,             "kEplApiSdoBusyIntern"          },
    { kEplApiPIAlreadyAllocated,        "kEplApiPIAlreadyAllocated"     },
    { kEplApiPIOutOfMemory,             "kEplApiPIOutOfMemory"          },
    { kEplApiPISizeExceeded,            "kEplApiPISizeExceeded"         },
    { kEplApiPINotAllocated,            "kEplApiPINotAllocated"         },
    { kEplApiPIJobQueueFull,            "kEplApiPIJobQueueFull"         },
    { kEplApiPIJobQueueEmpty,           "kEplApiPIJobQueueEmpty"        },
    { kEplApiPIInvalidJobSize,          "kEplApiPIInvalidJobSize"       },
    { kEplApiPIInvalidPIPointer,        "kEplApiPIInvalidPIPointer"     },
    { kEplApiPINonBlockingNotSupp,      "kEplApiPINonBlockingNotSupp"   },
};

static const tEplEmergErrCodeInfo   EplEmergErrCodeInfo_g[] =
{
    { EPL_E_NO_ERROR,                    "EPL_E_NO_ERROR"               },

    // 0xFxxx manufacturer specific error codes
    { EPL_E_NMT_NO_IDENT_RES,            "EPL_E_NMT_NO_IDENT_RES"       },
    { EPL_E_NMT_NO_STATUS_RES,           "EPL_E_NMT_NO_STATUS_RES"      },

    // 0x816x HW errors
    { EPL_E_DLL_BAD_PHYS_MODE,           "EPL_E_DLL_BAD_PHYS_MODE"      },
    { EPL_E_DLL_COLLISION,               "EPL_E_DLL_COLLISION"          },
    { EPL_E_DLL_COLLISION_TH,            "EPL_E_DLL_COLLISION_TH"       },
    { EPL_E_DLL_CRC_TH,                  "EPL_E_DLL_CRC_TH "            },
    { EPL_E_DLL_LOSS_OF_LINK,            "EPL_E_DLL_LOSS_OF_LINK",      },
    { EPL_E_DLL_MAC_BUFFER,              "EPL_E_DLL_MAC_BUFFER",        },

    // 0x82xx Protocol errors
    { EPL_E_DLL_ADDRESS_CONFLICT,        "EPL_E_DLL_ADDRESS_CONFLICT"   },
    { EPL_E_DLL_MULTIPLE_MN,             "EPL_E_DLL_MULTIPLE_MN"        },

    // 0x821x Frame size errors
    { EPL_E_PDO_SHORT_RX,                "EPL_E_PDO_SHORT_RX"           },
    { EPL_E_PDO_MAP_VERS,                "EPL_E_PDO_MAP_VERS"           },
    { EPL_E_NMT_ASND_MTU_DIF,            "EPL_E_NMT_ASND_MTU_DIF"       },
    { EPL_E_NMT_ASND_MTU_LIM,            "EPL_E_NMT_ASND_MTU_LIM"       },
    { EPL_E_NMT_ASND_TX_LIM,             "EPL_E_NMT_ASND_TX_LIM"        },

    // 0x823x Timing errors
    { EPL_E_NMT_CYCLE_LEN,               "EPL_E_NMT_CYCLE_LEN"          },
    { EPL_E_DLL_CYCLE_EXCEED,            "EPL_E_DLL_CYCLE_EXCEED"       },
    { EPL_E_DLL_CYCLE_EXCEED_TH,         "EPL_E_DLL_CYCLE_EXCEED_TH"    },
    { EPL_E_NMT_IDLE_LIM,                "EPL_E_NMT_IDLE_LIM"           },
    { EPL_E_DLL_JITTER_TH,               "EPL_E_DLL_JITTER_TH"          },
    { EPL_E_DLL_LATE_PRES_TH,            "EPL_E_DLL_LATE_PRES_TH"       },
    { EPL_E_NMT_PREQ_CN,                 "EPL_E_NMT_PREQ_CN"            },
    { EPL_E_NMT_PREQ_LIM,                "EPL_E_NMT_PREQ_LIM"           },
    { EPL_E_NMT_PRES_CN,                 "EPL_E_NMT_PRES_CN"            },
    { EPL_E_NMT_PRES_RX_LIM,             "EPL_E_NMT_PRES_RX_LIM"        },
    { EPL_E_NMT_PRES_TX_LIM,             "EPL_E_NMT_PRES_TX_LIM"        },

    // 0x824x Frame errors
    { EPL_E_DLL_INVALID_FORMAT,          "EPL_E_DLL_INVALID_FORMAT"     },
    { EPL_E_DLL_LOSS_PREQ_TH,            "EPL_E_DLL_LOSS_PREQ_TH"       },
    { EPL_E_DLL_LOSS_PRES_TH,            "EPL_E_DLL_LOSS_PRES_TH"       },
    { EPL_E_DLL_LOSS_SOA_TH,             "EPL_E_DLL_LOSS_SOA_TH"        },
    { EPL_E_DLL_LOSS_SOC_TH,             "EPL_E_DLL_LOSS_SOC_TH"        },

    // 0x84xx BootUp Errors
    { EPL_E_NMT_BA1,                     "EPL_E_NMT_BA1"                },
    { EPL_E_NMT_BA1_NO_MN_SUPPORT,       "EPL_E_NMT_BA1_NO_MN_SUPPORT"  },
    { EPL_E_NMT_BPO1,                    "EPL_E_NMT_BPO1"               },
    { EPL_E_NMT_BPO1_GET_IDENT,          "EPL_E_NMT_BPO1_GET_IDENT"     },
    { EPL_E_NMT_BPO1_DEVICE_TYPE,        "EPL_E_NMT_BPO1_DEVICE_TYPE"   },
    { EPL_E_NMT_BPO1_VENDOR_ID,          "EPL_E_NMT_BPO1_VENDOR_ID"     },
    { EPL_E_NMT_BPO1_PRODUCT_CODE,       "EPL_E_NMT_BPO1_PRODUCT_CODE"  },
    { EPL_E_NMT_BPO1_REVISION_NO,        "EPL_E_NMT_BPO1_REVISION_NO"   },
    { EPL_E_NMT_BPO1_SERIAL_NO,          "EPL_E_NMT_BPO1_SERIAL_NO"     },
    { EPL_E_NMT_BPO1_CF_VERIFY,          "EPL_E_NMT_BPO1_CF_VERIFY"     },
    { EPL_E_NMT_BPO2,                    "EPL_E_NMT_BPO2"               },
    { EPL_E_NMT_BRO,                     "EPL_E_NMT_BRO"                },
    { EPL_E_NMT_WRONG_STATE,             "EPL_E_NMT_WRONG_STATE"        },
};

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

// text strings for NMT boot events
static char *EplNmtBootEvtTypeStr_g[] =
{
    "Boot step 1 finished",     // 0x00     PreOp2 is possible
    "Boot step 2 finished",     // 0x01     ReadyToOp is possible for MN
    "EnableReadyToOp",          // 0x02     ReadyToOP is possible for CN
    "CheckComFinish",           // 0x03     Operational is possible
    "Operational",              // 0x04     all mandatory CNs are Operational
    "Error",                    // 0x05
};

// text strings for SDO command layer connection states
static char *EplSdoComConStateStr_g[] =
{
    "NotActive",                // 0x00
    "Running",                  // 0x01
    "TxAborted",                // 0x02
    "RxAborted",                // 0x03
    "Finished",                 // 0x04
    "LowerLayerAbort",          // 0x05
};


//=========================================================================//
// Module internal prototypes                                              //
//=========================================================================//

static int  EplDebugCompareApiEvent( const void *pvKey, const void *pvArray );
static int  EplDebugCompareEplKernel( const void *pvKey, const void *pvArray );
static int  EplDebugCompareEmergErrCode( const void *pvKey, const void *pvArray );

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
    if (nmtEvent_p >= tabentries(eplEvtStr_g))
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
    if (eventSrc_p >= tabentries(eplEvtSrcStr_g))
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
    if (eventSink_p >= tabentries(eplEvtSinkStr_g))
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
    if (eventType_p >= tabentries(eplEvtTypeStr_g))
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

    for (i = 0; i < tabentries(nmtStateInfo_g); i++)
    {
        if (nmtStateInfo_g[i].m_nmtState == nmtState_p)
            return (nmtStateInfo_g[i].m_sNmtState);
    }
    return eplInvalidStr_g;
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
    tApiEventInfo   *pApiEventInfo;
    tApiEventInfo   Key;

    // Init
    Key.m_ApiEvent  = ApiEvent_p;

    // Search element
    pApiEventInfo   = bsearch(  &Key, ApiEventInfo_g,
                                tabentries(ApiEventInfo_g),
                                sizeof(Key),
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
    if( NodeEventType_p >= tabentries(EplNmtNodeEvtTypeStr_g) )
    {
        return  eplInvalidStr_g;
    }
    else
    {
        return  EplNmtNodeEvtTypeStr_g[ NodeEventType_p ];
    }
}

//---------------------------------------------------------------------------
//
// Function:    EplGetNmtBootEventTypeStr()
//
// Description: returns the string of the specified NMT boot event
//
// Parameters:  NodeEventType_p        Type of NMT boot event
//
// Returns:     event type string if found
//              eplInvalidStr_g if not found
//
//---------------------------------------------------------------------------
char *EplGetNmtBootEventTypeStr( tEplNmtBootEvent BootEventType_p )
{
    if( BootEventType_p >= tabentries(EplNmtBootEvtTypeStr_g) )
    {
        return  eplInvalidStr_g;
    }
    else
    {
        return  EplNmtBootEvtTypeStr_g[ BootEventType_p ];
    }
}

//---------------------------------------------------------------------------
//
// Function:    EplGetSdoComConStateStr
//
// Description: returns the string of the specified SDO command connection state
//
// Parameters:  SdoComConState_p        SDO comand layer connection state
//
// Returns:     SDO command connection state string if found
//              eplInvalidStr_g if not found
//
//---------------------------------------------------------------------------
char *EplGetSdoComConStateStr( tEplSdoComConState SdoComConState_p )
{
    if( SdoComConState_p >= tabentries(EplSdoComConStateStr_g) )
    {
        return  eplInvalidStr_g;
    }
    else
    {
        return  EplSdoComConStateStr_g[ SdoComConState_p ];
    }
}

//---------------------------------------------------------------------------
//
// Function:    EplGetEplKernelStr()
//
// Description: Return a string describing a given entry of type tEplKernel.
//
// Parameters:  Parameter of type tEplKernel
//
// Returns:     Pointer to string with human readable description of given parameter
//              Pointer to a default string if the parameter could not be found
//
//---------------------------------------------------------------------------
char *EplGetEplKernelStr( tEplKernel EplKernel_p )
{
    tEplDebugEplKernelInfo   *pEntry;
    tEplDebugEplKernelInfo   Key;

    // Init
    Key.m_Key  = EplKernel_p;

    // Search element
    pEntry   = bsearch( &Key,
                        EplKernelInfo_g,
                        tabentries(EplKernelInfo_g),
                        sizeof(Key),
                        EplDebugCompareEplKernel);

    // Check result
    if( NULL != pEntry )
    {
        return  pEntry->m_sName;
    }

    return eplInvalidStr_g;
}

//---------------------------------------------------------------------------
//
// Function:    EplGetEmergErrCodeStr()
//
// Description: Return a string describing a given entry of type
//              emergency error code.
//
// Parameters:  Parameter of type emergency error code
//
// Returns:     Pointer to string with human readable description of given parameter
//              Pointer to a default string if the parameter could not be found
//
//---------------------------------------------------------------------------
const char *EplGetEmergErrCodeStr( WORD EmergErrCode_p )
{
    tEplEmergErrCodeInfo   *pEntry;
    tEplEmergErrCodeInfo   Key;

    // Init
    Key.m_Key  = EmergErrCode_p;

    // Search element
    pEntry   = bsearch( &Key,
                        EplEmergErrCodeInfo_g,
                        tabentries(EplEmergErrCodeInfo_g),
                        sizeof(Key),
                        EplDebugCompareEmergErrCode);

    // Check result
    if( NULL != pEntry )
    {
        return  pEntry->m_sName;
    }

    return eplInvalidStr_g;
}

//=========================================================================//
//                                                                         //
//          P R I V A T E   D E F I N I T I O N S                          //
//                                                                         //
//=========================================================================//

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
EplDebugCompareApiEvent( const void *pvKey, const void *pvArray )
{
    tApiEventInfo    *pKey      = (tApiEventInfo *) pvKey;
    tApiEventInfo    *pArray    = (tApiEventInfo *) pvArray;

    if( pKey->m_ApiEvent < pArray->m_ApiEvent )
    {
        return  -1;
    }
    else if( pKey->m_ApiEvent > pArray->m_ApiEvent )
    {
        return  1;
    }

    return 0;
}

//---------------------------------------------------------------------------
//
// Function:    EplDebugCompareEplKernel()
//
// Description: Compare two values of type tEplKernel. Used by EplGetEplKernelStr's bsearch().
//
// Parameters:  Key (element to search), Array
//
// Returns:     -1, 0, or 1 if event id is smaller, equal or greater
//
//---------------------------------------------------------------------------
static int
EplDebugCompareEplKernel( const void *pvKey, const void *pvArray )
{
    tEplDebugEplKernelInfo    *pKey     = (tEplDebugEplKernelInfo *) pvKey;
    tEplDebugEplKernelInfo    *pArray   = (tEplDebugEplKernelInfo *) pvArray;

    if( pKey->m_Key < pArray->m_Key )
    {
        return  -1;
    }
    else if( pKey->m_Key > pArray->m_Key )
    {
        return  1;
    }

    return 0;
}

//---------------------------------------------------------------------------
//
// Function:    EplDebugCompareEmergErrCode()
//
// Description: Compare two values of type emergency error code.
//              Used by EplGetEmergErrCodeStr's bsearch().
//
// Parameters:  Key (element to search), Array
//
// Returns:     -1, 0, or 1 if event id is smaller, equal or greater
//
//---------------------------------------------------------------------------
static int
EplDebugCompareEmergErrCode( const void *pvKey, const void *pvArray )
{
    tEplEmergErrCodeInfo    *pKey     = (tEplEmergErrCodeInfo *) pvKey;
    tEplEmergErrCodeInfo    *pArray   = (tEplEmergErrCodeInfo *) pvArray;

    if( pKey->m_Key < pArray->m_Key )
    {
        return  -1;
    }
    else if( pKey->m_Key > pArray->m_Key )
    {
        return  1;
    }

    return 0;
}

