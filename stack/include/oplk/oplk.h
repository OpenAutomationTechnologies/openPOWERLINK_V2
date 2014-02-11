/**
********************************************************************************
\file   oplk/oplk.h

\brief  Definitions for openPOWERLINK API

This file contains all definitions and declarations to use the openPOWERLINK
API.
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2014, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
Copyright (c) 2013, SYSTEC electronic GmbH
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

#ifndef _INC_oplk_oplk_H_
#define _INC_oplk_oplk_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <oplk/oplkinc.h>
#include <oplk/sdo.h>
#include <oplk/obd.h>
#include <oplk/led.h>
#include <oplk/cfm.h>
#include <oplk/event.h>

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// typedef
//------------------------------------------------------------------------------

typedef enum
{
    tOplkApiAsndFilterNone      = 0x00,
    tOplkApiAsndFilterLocal     = 0x01,  // receive only ASnd frames with local or broadcast node ID
    tOplkApiAsndFilterAny       = 0x02,  // receive any ASnd frame
} tOplkApiAsndFilter;

/**
\brief Node Event

The following structure specifies a node event on a MN. The application will be
informed with this event if the state of the specified node has changed.
*/
typedef struct
{
    UINT                        nodeId;         ///< Node ID of the node that changed the state
    tNmtState                   nmtState;       ///< The NMT state of the CN
    tNmtNodeEvent               nodeEvent;      ///< The event that happens at the node
    UINT16                      errorCode;      ///< This variable contains an error code if nodeEvent == \ref kNmtNodeEventError
    BOOL                        fMandatory;     ///< Determines if it is a mandatory node (TRUE) or an optional node (FALSE)
} tOplkApiEventNode;

/**
\brief Boot Event

The following structure specifies a boot event. It is used to inform the application
about events concerning the entire boot-up process of the MN.
*/
typedef struct
{
    tNmtState                   nmtState;       ///< NMT state of the local node
    tNmtBootEvent               bootEvent;      ///< Boot event that occured
    UINT16                      errorCode;      ///< Contains an error code if bootEvent == \ref kNmtBootEventError
} tOplkApiEventBoot;


/**
\brief LED Event

This structure specifies a LED event. It contains change events for the POWERLINK
status and error LED. It allows the application to change the status and error
LEDs on the device according to the specification.
*/
typedef struct
{
    tLedType                    ledType;        ///< Determines the type of the LED
    BOOL                        fOn;            ///< The state of the LED
} tOplkApiEventLed;

/**
\brief CFM Result Event

The structure describes the CFM result event. This includes the normal progress
but also errors which occurred during the configuration process.

\note It is only valid for a MN.
*/
typedef struct
{
    UINT                        nodeId;         ///< Node ID of the CN which created the event
    tNmtNodeCommand             nodeCommand;    ///< Node command which will be issued to the CN as result of the configuration process. See \ref tNmtNodeCommand

} tOplkApiEventCfmResult;

/**
\brief Received ASnd Event

This structure specifies the event for received ASnd frames. It is used to inform
the application about received ASnd frames.
*/
typedef struct
{
    tPlkFrame                   *pFrame;        ///< Pointer to the received ASnd frame
    size_t                      frameSize;      ///< Size of the received ASnd frame
}
tOplkApiEventRcvAsnd;

/**
\brief Application event types

This enumeration specifies the valid application events which could be
sent by the openPOWERLINK stack.
*/
typedef enum
{
    /** User defined event. It is issued for sending user-defined events. It
    can be used for e.g. synchronization purposes. The event argument contains
    a pointer to the user specific argument. */
    kOplkApiEventUserDef            = 0x00,

    /** NMT state change event. If \ref kErrorReject is returned the subsequent
    NMT state will not be entered. In this case the application is in charge of
    executing the appropriate NMT commands. The event argument contains a NMT
    state change event \ref tEventNmtStateChange .*/
    kOplkApiEventNmtStateChange     = 0x10,

    /** Critical error event. When this event occurs the NMT state machine will
    be switched off with NMT event \ref kNmtEventCriticalError. The application
    may restart the NMT state machine afterwards, but it is unlikely that the
    openPOWERLINK stack will run stable, because this critical error or the
    source of it often is a configuration error and not a run-time error. The
    event argument contains an error event (\ref tEventError). */
    kOplkApiEventCriticalError      = 0x12,

    /** Warning event. The warning may be a run-time error, which should be
    logged into an error log for further diagnostics. In any case the openPOWERLINK
    stack proceeds. The event argument contains an error event. (\ref tEventError) */
    kOplkApiEventWarning            = 0x13,

    /** New error history event. The event argument contains an error history
    entry (\ref tErrHistoryEntry). */
    kOplkApiEventHistoryEntry       = 0x14,

    /** Node event on MN. The state of the specified CN has changed. The event
    argument contains the node event information(\ref tOplkApiEventNode). */
    kOplkApiEventNode               = 0x20,

    /** Boot event on MN. The MN reached the specified state in the boot-up
    process. The event argument contains the boot event information
    (\ref tOplkApiEventBoot).*/
    kOplkApiEventBoot               = 0x21,

    /** SDO transfer finished. This event informs about a finished SDO transfer.
    The event argument contains the SDO command layer information
    (\ref tSdoComFinished). */
    kOplkApiEventSdo                = 0x62,

    /** Object dictionary access. This event informs about an access of the
    object dictionary. The event argument contains a OBD callback parameter
    (\ref tObdCbParam). */
    kOplkApiEventObdAccess          = 0x69,

    /** Status and error LED event. The event allows the application to perform
    the signaling of the POWERLINK LEDs according to the application. The event
    argument contains a LED event (\ref kOplkApiEventLed). */
    kOplkApiEventLed                = 0x70,

    /** CFM progress event. This event informs the application about the progress
    of the configuration of a specific CN. The event argument contains the CN
    progress information (\ref tCfmEventCnProgress). */
    kOplkApiEventCfmProgress        = 0x71,

    /** CFM result event. This event informs the application about the result
    of the configuration of a specific CN. The event argument contains the
    CFM result information (\ref tOplkApiEventCfmResult). */
    kOplkApiEventCfmResult          = 0x72,

    /** Received ASnd event. This event informs the application about a received
    ASnd frame. This event is forwarded only if the application has enabled
    the forwarding of ASnd frames by oplk_setAsndForward(). The event argument
    contains information on the received ASnd frame (\ref tOplkApiEventRcvAsnd). */
    kOplkApiEventReceivedAsnd       = 0x73,
} tOplkApiEventType;


/**
\brief Event argument

This union specifies all data that can be specified as an event argument.
Depending on the event type (\ref tOplkApiEventType) the according member of
this union is used.
*/
typedef union
{
    void*                       pUserArg;           ///< User argument (\ref kOplkApiEventUserDef)
    tEventNmtStateChange        nmtStateChange;     ///< NMT state change information (\ref kOplkApiEventNmtStateChange)
    tEventError                 internalError;      ///< Internal stack error (\ref kOplkApiEventCriticalError, \ref kOplkApiEventWarning)
    tSdoComFinished             sdoInfo;            ///< SDO information (\ref kOplkApiEventSdo)
    tObdCbParam                 obdCbParam;         ///< OBD callback parameter (\ref kOplkApiEventObdAccess)
    tOplkApiEventNode           nodeEvent;          ///< Node event information (\ref kOplkApiEventNode)
    tOplkApiEventBoot           bootEvent;          ///< Boot event information (\ref kOplkApiEventBoot)
    tOplkApiEventLed            ledEvent;           ///< LED event information (\ref kOplkApiEventLed)
    tCfmEventCnProgress         cfmProgress;        ///< CFM progress information (\ref kOplkApiEventCfmProgress)
    tOplkApiEventCfmResult      cfmResult;          ///< CFM result information (\ref kOplkApiEventCfmResult)
    tErrHistoryEntry            errorHistoryEntry;  ///< Error history entry (\ref kOplkApiEventHistoryEntry)
    tOplkApiEventRcvAsnd        receivedAsnd;       ///< Received ASnd frame information (\ref kOplkApiEventReceivedAsnd)
} tOplkApiEventArg;

/**
\brief Type for API event callback function pointer

This type defines a function pointer to an API event callback function.

\param eventType_p  The type of the event
\param pEventArg_p  Pointer to the event argument
\param pUserArg_p   Pointer to the user defined argument

\return The function returns a tOplkError error code
*/
typedef tOplkError (*tOplkApiCbEvent)(tOplkApiEventType eventType_p, tOplkApiEventArg* pEventArg_p, void* pUserArg_p);

/**
\brief openPOWERLINK initialization parameters

The structure defines the openPOWERLINK initialization parameters. The openPOWERLINK
stack will be initialized with this parameters when oplk_init() is called. Most
of the parameters will be stored in the object dictionary. Some of these objects
are constant (read-only) objects and the initialization parameters are the only way of
setting their values. Writable objects could be overwritten later at the boot-up
process. This could be done by reading an CDC file for a MN or by configuration
of a CN from a MN with SDO transfers.

\note The elements of the parameter structure must be specified in platform
byte order!
*/
typedef struct
{
    UINT                sizeOfInitParam;            ///< This field contains the size of the initialization parameter structure.
    BOOL                fAsyncOnly;                 ///< Determines if this node is an async-only node. If TRUE the node communicates only asynchronously.
    UINT                nodeId;                     ///< The node ID of this node.
    BYTE                aMacAddress[6];             ///< The MAC address of this node.
    UINT32              featureFlags;               ///< The POWERLINK feature flags of this node (0x1F82: NMT_FeatureFlags_U32)
    UINT32              cycleLen;                   ///< The cycle Length (0x1006: NMT_CycleLen_U32) in [us]
    UINT                isochrTxMaxPayload;         ///< Maximum isochronous transmit payload (0x1F98.1: IsochrTxMaxPayload_U16) Const!
    UINT                isochrRxMaxPayload;         ///< Maximum isochronous receive payload (0x1F98.2: IsochrRxMaxPayload_U16) Const!
    UINT32              presMaxLatency;             ///< Maximum PRes latency in ns (0x1F98.3: PResMaxLatency_U32) Read-only!
    UINT                preqActPayloadLimit;        ///< Actual PReq payload limit (0x1F98.4: PReqActPayloadLimit_U16)
    UINT                presActPayloadLimit;        ///< Actual PRes payload limit (0x1F98.5: PResActPayloadLimit_U16)
    UINT32              asndMaxLatency;             ///< Maximum ASnd latency in ns (0x1F98.6: ASndMaxLatency_U32) Const!
    UINT                multiplCylceCnt;            ///< Multiplexed cycle count (0x1F98.7: MultiplCycleCnt_U8)
    UINT                asyncMtu;                   ///< Asynchronous MTU (0x1F98.8: AsyncMTU_U16)
    UINT                prescaler;                  ///< SoC prescaler (0x1F98.9: Prescaler_U16)
    UINT32              lossOfFrameTolerance;       ///< Loss of frame tolerance in ns (0x1C14: DLL_LossOfFrameTolerance_U32)
    UINT32              waitSocPreq;                ///< Wait time for first PReq in ns (0x1F8A.1: WaitSoCPReq_U32) Only for MN!
    UINT32              asyncSlotTimeout;           ///< Asynchronous slot timeout in ns (0x1F8A.2: AsyncSlotTimeout_U32) Only for MN!
    UINT32              deviceType;                 ///< The device type of this node (0x1000.0: NMT_DeviceType_U32) Const!
    UINT32              vendorId;                   ///< The vendor ID of this node (0x1018.1: NMT_IdentityObject_REC.VendorId_U32) Const!
    UINT32              productCode;                ///< The product code of this node (0x1018.2: NMT_IdentityObject_REC.ProductCode_U32) Const!
    UINT32              revisionNumber;             ///< The revision number of this node (0x1018.3: NMT_IdentityObject_REC.RevisionNo_U32) Const!
    UINT32              serialNumber;               ///< The serial number of this node (0x1018.4: NMT_IdentityObject_REC.SerialNo_U32) Const!
    UINT64              vendorSpecificExt1;         ///< Vendor specific extensions 1 listed in the IdentResponse frame
    UINT32              verifyConfigurationDate;    ///< Local configuration date (0x1020.1 CFM_VerifyConfiguration_REC.ConfDate_U32)
    UINT32              verifyConfigurationTime;    ///< Local configuration time (0x1020.2 CFM_VerifyConfiguration_REC.ConfTime_U32)
    UINT32              applicationSwDate;          ///< Local program Date (0x1F52.1 PDL_LocVerApplSw_REC.ApplSwDate_U32)
    UINT32              applicationSwTime;          ///< Local program Time (0x1F52.2 PDL_LocVerApplSw_REC.ApplSwTime_U32)
    UINT32              ipAddress;                  ///< IP address of the node
    UINT32              subnetMask;                 ///< Subnet mask of the node
    UINT32              defaultGateway;             ///< Default gateway used by this node
    UINT8               sHostname[32];              ///< DNS host name of the node (maximum length: 32 characters!)
    UINT8               aVendorSpecificExt2[48];    ///< Vendor specific extensions 2 listed in the IdentResponse frame
    char*               pDevName;                   ///< Pointer to manufacturer device name (0x1008.0: NMT_ManufactDevName_VS) Const!
    char*               pHwVersion;                 ///< Pointer to manufacturer hardware version (0x1009.0: NMT_ManufactHwVers_VS) Const!
    char*               pSwVersion;                 ///< Pointer to manufacturer software version (0x100A.0: NMT_ManufactSwVers_VS) Const!
    tOplkApiCbEvent     pfnCbEvent;                 ///< Pointer to the applications event handling function.
    void*               pEventUserArg;              ///< Pointer to a user argument that is supplied to the event callback function (\ref tOplkApiCbEvent)
    tSyncCb             pfnCbSync;                  ///< Pointer to the application sync callback function.
                                                    /**< It is normally used only for non-threaded systems.
                                                         If the application processes synchronous data by a separate thread it must be initialized with NULL! */
    tHwParam            hwParam;                    ///< The hardware parameters of the node
    UINT32              syncResLatency;             ///< Constant response latency for SyncRes in ns
    UINT                syncNodeId;                 ///< Specifies the synchronization point for the MN. The synchronization take place after a PRes from a CN with this node-ID (0 = SoC, 255 = SoA)
    BOOL                fSyncOnPrcNode;             ///< If it is TRUE, Sync on PRes chained CN; FALSE: conventional CN (PReq/PRes)
} tOplkApiInitParam;

/**
\brief  Process image information structure

This structure provides information about a process image.
*/
typedef struct
{
    void*          pImage;                          ///< Pointer to the process image
    UINT           imageSize;                       ///< Size of the process image
} tOplkApiProcessImage;

//------------------------------------------------------------------------------
// function prototypes
//------------------------------------------------------------------------------
#ifdef __cplusplus
extern "C" {
#endif

// Generic API functions
OPLKDLLEXPORT tOplkError oplk_init(tOplkApiInitParam* pInitParam_p);
OPLKDLLEXPORT tOplkError oplk_shutdown(void);
OPLKDLLEXPORT tOplkError oplk_execNmtCommand(tNmtEvent NmtEvent_p);
OPLKDLLEXPORT tOplkError oplk_linkObject(UINT objIndex_p, void* pVar_p, UINT* pVarEntries_p,
                                        tObdSize* pEntrySize_p, UINT firstSubindex_p);
OPLKDLLEXPORT tOplkError oplk_readObject(tSdoComConHdl* pSdoComConHdl_p, UINT  nodeId_p, UINT index_p,
                                        UINT subindex_p, void* pDstData_le_p, UINT* pSize_p,
                                        tSdoType sdoType_p, void* pUserArg_p);
OPLKDLLEXPORT tOplkError oplk_writeObject(tSdoComConHdl* pSdoComConHdl_p, UINT nodeId_p, UINT index_p,
                                         UINT subindex_p, void* pSrcData_le_p, UINT size_p,
                                         tSdoType sdoType_p, void* pUserArg_p);
OPLKDLLEXPORT tOplkError oplk_freeSdoChannel(tSdoComConHdl sdoComConHdl_p);
OPLKDLLEXPORT tOplkError oplk_abortSdo(tSdoComConHdl sdoComConHdl_p, UINT32 abortCode_p);
OPLKDLLEXPORT tOplkError oplk_readLocalObject(UINT index_p, UINT subindex_p, void* pDstData_p, UINT* pSize_p);
OPLKDLLEXPORT tOplkError oplk_writeLocalObject(UINT index_p, UINT subindex_p, void* pSrcData_p, UINT size_p);
OPLKDLLEXPORT tOplkError oplk_sendAsndFrame(UINT8 dstNodeId_p, tAsndFrame *pAsndFrame_p, size_t asndSize_p);
OPLKDLLEXPORT tOplkError oplk_setAsndForward(UINT8 serviceId_p, tOplkApiAsndFilter FilterType_p);
OPLKDLLEXPORT tOplkError oplk_postUserEvent(void* pUserArg_p);
OPLKDLLEXPORT tOplkError oplk_triggerMnStateChange(UINT nodeId_p, tNmtNodeCommand nodeCommand_p);
OPLKDLLEXPORT tOplkError oplk_setCdcBuffer(BYTE* pbCdc_p, UINT cdcSize_p);
OPLKDLLEXPORT tOplkError oplk_setCdcFilename(char* pszCdcFilename_p);
OPLKDLLEXPORT tOplkError oplk_process(void);
OPLKDLLEXPORT tOplkError oplk_getIdentResponse(UINT nodeId_p, tIdentResponse** ppIdentResponse_p);
OPLKDLLEXPORT BOOL       oplk_checkKernelStack(void);
OPLKDLLEXPORT tOplkError oplk_waitSyncEvent(ULONG timeout_p);

// Process image API functions
OPLKDLLEXPORT tOplkError oplk_allocProcessImage(UINT sizeProcessImageIn_p, UINT sizeProcessImageOut_p);
OPLKDLLEXPORT tOplkError oplk_freeProcessImage(void);
OPLKDLLEXPORT tOplkError oplk_linkProcessImageObject(UINT objIndex_p, UINT firstSubindex_p, UINT offsetPI_p,
                                                    BOOL fOutputPI_p, tObdSize entrySize_p, UINT* pVarEntries_p);
OPLKDLLEXPORT tOplkError oplk_exchangeProcessImageIn(void);
OPLKDLLEXPORT tOplkError oplk_exchangeProcessImageOut(void);
OPLKDLLEXPORT void*      oplk_getProcessImageIn(void);
OPLKDLLEXPORT void*      oplk_getProcessImageOut(void);

// objdict specific process image functions
OPLKDLLEXPORT tOplkError oplk_setupProcessImage(void);

#ifdef __cplusplus
}
#endif

#endif  // #ifndef _INC_oplk_oplk_H_
