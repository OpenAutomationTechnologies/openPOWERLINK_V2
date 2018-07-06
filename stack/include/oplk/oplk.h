/**
********************************************************************************
\file   oplk/oplk.h

\brief  Definitions for openPOWERLINK API

This file contains all definitions and declarations to use the openPOWERLINK
API.
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2017, B&R Industrial Automation GmbH
Copyright (c) 2013, SYSTEC electronic GmbH
Copyright (c) 2018, Kalycito Infotech Private Limited
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
#include <oplk/frame.h>
#include <oplk/sdo.h>
#include <oplk/obd.h>
#include <oplk/obdal.h>
#include <oplk/cfm.h>
#include <oplk/event.h>


//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#define OPLK_MAC_ADDRESS_LENGTH     6
#define OPLK_MAX_ETH_DEVICE_NAME    64
#define OPLK_MAX_ETH_DEVICE_DESC    256

//------------------------------------------------------------------------------
// typedef
//------------------------------------------------------------------------------

/**
\brief Structure identifying a network interface

This structure identifies a network interface.
*/
typedef struct
{
    UINT8   aMacAddress[OPLK_MAC_ADDRESS_LENGTH];           ///< MAC address of the interface
    char    aDeviceName[OPLK_MAX_ETH_DEVICE_NAME];          ///< Name (system-internal identifier) of the interface
    char    aDeviceDescription[OPLK_MAX_ETH_DEVICE_DESC];   ///< Description of the interface
} tNetIfId;

typedef enum
{
    tOplkApiAsndFilterNone      = 0x00,
    tOplkApiAsndFilterLocal     = 0x01,  // receive only ASnd frames with local or broadcast node ID
    tOplkApiAsndFilterAny       = 0x02,  // receive any ASnd frame
} eOplkApiAsndFilter;

typedef UINT32 tOplkApiAsndFilter;

/**
\brief SDO stack

The following enum defines the different SDO stacks. The application can
switch between the SDO stacks.
*/
typedef enum
{
    tOplkApiStdSdoStack         = 0x00,  // Use the standard SDO stack (default)
    tOplkApiTestSdoCom          = 0x10,  // Use testing functions for SDO command layer
    tOplkApiTestSdoSeq          = 0x20   // Use testing functions for SDO sequence layer
} eOplkApiSdoStack;

/**
\brief SDO stack data type

Data type for the enumerator \ref eOplkApiSdoStack.
*/
typedef UINT32 tOplkApiSdoStack;

/**
\brief Node event

The following structure specifies a node event on an MN. The application will be
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
\brief Boot event

The following structure specifies a boot event. It is used to inform the application
about events concerning the entire boot-up process of the node.
*/
typedef struct
{
    tNmtState                   nmtState;       ///< NMT state of the local node
    tNmtBootEvent               bootEvent;      ///< Boot event that occurred
    UINT16                      errorCode;      ///< Contains an error code if bootEvent == \ref kNmtBootEventError
} tOplkApiEventBoot;

/**
\brief CFM result event

The structure describes the CFM result event. This includes the normal progress
but also errors which occurred during the configuration process.

\note It is only valid for an MN.
*/
typedef struct
{
    UINT                        nodeId;         ///< Node ID of the CN which generated the event
    tNmtNodeCommand             nodeCommand;    ///< Node command which will be issued to the CN as a result of the configuration process. See \ref tNmtNodeCommand
} tOplkApiEventCfmResult;

/**
\brief Received ASnd event

This structure specifies the event for received ASnd frames. It is used to inform
the application about received ASnd frames.
*/
typedef struct
{
    tPlkFrame*                  pFrame;         ///< Pointer to the received ASnd frame
    size_t                      frameSize;      ///< Size of the received ASnd frame
} tOplkApiEventRcvAsnd;

/**
\brief PDO changed event

This structure specifies the event for PDO changes. It will be sent to the
application if the PDO mapping has changed
*/
typedef struct
{
    BOOL                        fActivated;     ///< Determines if mapping is activated.
    BOOL                        fTx;            ///< Determines if it is a TXPDO or RXPDO.
    UINT                        nodeId;         ///< The node ID the mapping is related to.
    UINT                        mappParamIndex; ///< The object index of the mapping parameter object.
    UINT                        mappObjectCount;///< The number of mapped objects (channels).
} tOplkApiEventPdoChange;

/**
\brief Received PRes event

This structure specifies the event for received PRes frames. It is used to
forward requested PRes frames to the application (e.g. for diagnosis).
*/
typedef struct
{
    UINT                        nodeId;         ///< Node ID of the received PRes frame
    size_t                      frameSize;      ///< Size of the received PRes frame
    tPlkFrame*                  pFrame;         ///< Pointer to the received PRes frame
} tOplkApiEventReceivedPres;

/**
\brief Received non-POWERLINK Ethernet frame event

This structure specifies the event for received Ethernet frames. It is used to
inform the application about received Ethernet frames.
*/
typedef struct
{
    tPlkFrame*                  pFrame;         ///< Pointer to the received Ethernet frame
    size_t                      frameSize;      ///< Size of the received Ethernet frame
} tOplkApiEventReceivedNonPlk;

/**
\brief Default gateway changed event

This structure specifies the event for default gateway changed. It is used to
inform the application about the changed default gateway address.
*/
typedef struct
{
    UINT32                      defaultGateway; ///< Default gateway
} tOplkApiEventDefaultGwChange;

/**
\brief SDO command layer receive event

This structure specifies the event for an received SDO command layer.
It is used to inform the application about the received SDO command layer.
*/
typedef struct
{
    tAsySdoCom*                 pAsySdoCom;     ///< Pointer to the SDO command layer
    size_t                      dataSize;       ///< Size of the received SDO command layer
} tOplkApiEventReceivedSdoCom;

/**
\brief SDO sequence layer receive event

This structure specifies the event for an received SDO sequence layer.
It is used to inform the application about the received SDO sequence layer.
*/
typedef struct
{
    tAsySdoSeq*                 pAsySdoSeq;     ///< Pointer to the SDO sequence layer
    size_t                      dataSize;       ///< Size of the received SDO sequence layer
} tOplkApiEventReceivedSdoSeq;

/**
\brief User specific OD access event

This structure specifies the event for a user specific object access.
It is used to forward an access to an object which doesn't exist in the default
object dictionary to the API.
*/
typedef struct
{
    tObdAlConHdl*               pUserObdAccHdl; ///< Pointer to handle for user specific OD access
} tOplkApiEventUserObdAccess;

/**
\brief Application event types

This enumeration specifies the valid application events which can be
sent by the openPOWERLINK stack.
*/
typedef enum
{
    /** User defined event. It is issued for sending user-defined events. It
    can be used for e.g. synchronization purposes. The event argument contains
    a pointer to the user specific argument. */
    kOplkApiEventUserDef            = 0x00,

    /** NMT state change event. If \ref kErrorReject is returned, the subsequent
    NMT state will not be entered. In this case the application is in charge of
    executing the appropriate NMT commands. The event argument contains an NMT
    state change event \ref tEventNmtStateChange .*/
    kOplkApiEventNmtStateChange     = 0x10,

    /** Critical error event. If this event occurs, the NMT state machine will
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
    object dictionary. It happens only, if the call flag is set to TRUE in the
    concerning object in the object dictionary definition.
    The event argument contains an OD callback parameter (\ref tObdCbParam). */
    kOplkApiEventObdAccess          = 0x69,

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

    /** PDO changed event. This event informs the application about a changed
    PDO mapping. */
    kOplkApiEventPdoChange          = 0x74,

    /** Received PRes event. This event informs the application that a requested
    PRes frame was received. It can be used for diagnosis purpose. */
    kOplkApiEventReceivedPres       = 0x80,

    /** Received Ethernet frame event. This event informs the application about
    a received Ethernet frame. The event argument contains information on the
    received Ethernet frame (\ref tOplkApiEventReceivedNonPlk). */
    kOplkApiEventReceivedNonPlk     = 0x81,

    /** Default gateway changed event. This event informs the application about
    a changed default gateway. The event argument gives the default gateway
    (\ref tOplkApiEventDefaultGwChange).*/
    kOplkApiEventDefaultGwChange    = 0x82,

    /** Received SDO command layer. This event informs the application about
    a received SDO command layer. This event argument contains information on the
    received SDO command layer (\ref tOplkApiEventReceivedSdoCom). */
    kOplkApiEventReceivedSdoCom     = 0x83,

    /** Received SDO sequence layer. This event informs the application about
    a received SDO sequence layer. This event argument contains information on the
    received SDO sequence layer (\ref tOplkApiEventReceivedSdoSeq). */
    kOplkApiEventReceivedSdoSeq     = 0x84,

    /** User specific OD access. This event informs the application about
    an object access to a non-existing object in the default OD. The event
    argument contains information about the accessed object and used data
    (\ref tOplkApiEventUserObdAccess).
    Per default, this event is disabled. It can be enabled with
    \ref oplk_enableUserObdAccess. Either, the processing finishes within the
    event function call, or \ref kErrorReject has to be returned, whereas the
    processing must finish with a call to \ref oplk_finishUserObdAccess. */
    kOplkApiEventUserObdAccess       = 0x85,
} eOplkApiEventType;

/**
\brief Application event data type

Data type for the enumerator \ref eOplkApiEventType.
*/
typedef UINT32 tOplkApiEventType;

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
    tObdCbParam                 obdCbParam;         ///< OD callback parameter (\ref kOplkApiEventObdAccess)
    tOplkApiEventNode           nodeEvent;          ///< Node event information (\ref kOplkApiEventNode)
    tOplkApiEventBoot           bootEvent;          ///< Boot event information (\ref kOplkApiEventBoot)
    tCfmEventCnProgress         cfmProgress;        ///< CFM progress information (\ref kOplkApiEventCfmProgress)
    tOplkApiEventCfmResult      cfmResult;          ///< CFM result information (\ref kOplkApiEventCfmResult)
    tErrHistoryEntry            errorHistoryEntry;  ///< Error history entry (\ref kOplkApiEventHistoryEntry)
    tOplkApiEventRcvAsnd        receivedAsnd;       ///< Received ASnd frame information (\ref kOplkApiEventReceivedAsnd)
    tOplkApiEventPdoChange      pdoChange;          ///< PDO change event (\ref kOplkApiEventPdoChange)
    tOplkApiEventReceivedPres   receivedPres;       ///< Received PRes frame (\ref kOplkApiEventReceivedPres)
    tOplkApiEventReceivedNonPlk receivedEth;        ///< Received Ethernet frame (\ref kOplkApiEventReceivedNonPlk)
    tOplkApiEventDefaultGwChange defaultGwChange;   ///< Default gateway change event (\ref kOplkApiEventDefaultGwChange)
    tOplkApiEventReceivedSdoCom receivedSdoCom;     ///< Received SDO command layer (\ref kOplkApiEventReceivedSdoCom)
    tOplkApiEventReceivedSdoSeq receivedSdoSeq;     ///< Received SDO sequence layer (\ref kOplkApiEventReceivedSdoSeq)
    tOplkApiEventUserObdAccess  userObdAccess;      ///< Access to user specific object (\ref kOplkApiEventUserObdAccess)
} tOplkApiEventArg;

/**
\brief Type for API event callback function pointer

This type defines a function pointer to an API event callback function.

\param[in]      eventType_p         The type of the event
\param[in]      pEventArg_p         Pointer to the event argument
\param[in]      pUserArg_p          Pointer to the user defined argument

\return The function returns a tOplkError error code
*/
typedef tOplkError (*tOplkApiCbEvent)(tOplkApiEventType eventType_p,
                                      const tOplkApiEventArg* pEventArg_p,
                                      void* pUserArg_p);

/**
\brief openPOWERLINK initialization parameters

The structure defines the openPOWERLINK initialization parameters. The openPOWERLINK
stack will be initialized with these parameters when oplk_create() is called. Most
of the parameters will be stored in the object dictionary. Some of these objects
are constant (read-only) objects and the initialization parameters are the only way of
setting their values. Writable objects could be overwritten later at the boot-up
process. This could be done by reading a CDC file for an MN or by configuration
of a CN from an MN via SDO transfers.

\note The elements of the parameter structure must be specified in platform
byte order!
*/
typedef struct
{
    UINT                sizeOfInitParam;            ///< This field contains the size of the initialization parameter structure.
    UINT8               fAsyncOnly;                 ///< Determines if this node is an async-only node. If TRUE the node communicates only asynchronously.
    UINT8               nodeId;                     ///< The node ID of this node.
    UINT8               padding1[3];                ///< Padding to 32 bit boundary
    UINT8               aMacAddress[6];             ///< The MAC address of this node.
    UINT32              featureFlags;               ///< The POWERLINK feature flags of this node (0x1F82: NMT_FeatureFlags_U32)
    UINT32              cycleLen;                   ///< The cycle Length (0x1006: NMT_CycleLen_U32) in [us]
    UINT16              isochrTxMaxPayload;         ///< Maximum isochronous transmit payload (0x1F98.1: IsochrTxMaxPayload_U16) Const!
    UINT16              isochrRxMaxPayload;         ///< Maximum isochronous receive payload (0x1F98.2: IsochrRxMaxPayload_U16) Const!
    UINT32              presMaxLatency;             ///< Maximum PRes latency in ns (0x1F98.3: PResMaxLatency_U32) Read-only!
    UINT16              preqActPayloadLimit;        ///< Actual PReq payload limit (0x1F98.4: PReqActPayloadLimit_U16)
    UINT16              presActPayloadLimit;        ///< Actual PRes payload limit (0x1F98.5: PResActPayloadLimit_U16)
    UINT32              asndMaxLatency;             ///< Maximum ASnd latency in ns (0x1F98.6: ASndMaxLatency_U32) Const!
    UINT8               multiplCylceCnt;            ///< Multiplexed cycle count (0x1F98.7: MultiplCycleCnt_U8)
    UINT8               padding2[3];                ///< Padding to 32 bit boundary
    UINT16              asyncMtu;                   ///< Asynchronous MTU (0x1F98.8: AsyncMTU_U16)
    UINT16              prescaler;                  ///< SoC prescaler (0x1F98.9: Prescaler_U16)
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
                                                    /**< It is used for single process systems where the whole openPOWERLINK stack is linked to the application.
                                                         In this case the stack calls the provided application callback function when synchronous data can be
                                                         exchanged. If a split stack is used (e.g. Linux user/kernel) it must be initialized with NULL. In
                                                         this case the application must use oplk_waitSyncEvent() for waiting on synchronous data. */
    tNetIfParameter     hwParam;                    ///< The network interface card parameters of the node
    UINT32              syncResLatency;             ///< Constant response latency for SyncRes in ns
    UINT                syncNodeId;                 ///< Specifies the synchronization point for the MN. The synchronization take place after a PRes from a CN with this node-ID (0 = SoC, 255 = SoA)
    UINT8               fSyncOnPrcNode;             ///< If it is TRUE, Sync on PRes chained CN; FALSE: conventional CN (PReq/PRes)
    tOplkApiSdoStack    sdoStackType;               ///< Specifies the SDO stack that should be used.
                                                    /**< It is used for switching between the standard SDO stack and alternative SDO stacks. The available SDO stacks are defined by the \ref tOplkApiSdoStack enumeration.
                                                         If the standard SDO stack is used it must be initialized with 0x00.*/
    UINT32              minSyncTime;                ///< Minimum synchronization period supported by the application [us]
                                                    /**< This parameter configures the period of synchronization events triggered by the openPOWERLINK stack.
                                                         Note that the resulting synchronization period can only be a multiple of the configured cycle length.
                                                         If this value is set to 0, no minimum synchronization period is specified. */
    tObdInitParam       obdInitParam;               ///< Initialization parameters for the object dictionary
} tOplkApiInitParam;

/**
\brief  Process image information structure

This structure provides information about a process image.
*/
typedef struct
{
    void*          pImage;                          ///< Pointer to the process image
    size_t         imageSize;                       ///< Size of the process image
} tOplkApiProcessImage;

/**
\brief  File chunk descriptor

This structure defines the file chunk descriptor used for transferring a file
to the kernel stack.
*/
typedef struct
{
    UINT8           fFirst;     ///< Start flag to identify the first chunk of the file
    UINT8           fLast;      ///< Last flag to identify the last chunk of the file
    UINT16          reserved;
    UINT32          length;     ///< Length of the chunk
    UINT32          offset;     ///< Offset of the chunk in the file
} tOplkApiFileChunkDesc;

/**
\brief  Stack information structure

This structure provides information about the kernel and user stack.
*/
typedef struct
{
    UINT32          userVersion;                    ///< User stack version
    UINT32          userFeature;                    ///< User stack feature bit mask
    UINT32          kernelVersion;                  ///< Kernel stack version
    UINT32          kernelFeature;                  ///< Kernel stack feature bit mask
} tOplkApiStackInfo;

/**
\brief  SoC time information structure

This structure provides the SoC time information to the API.
*/
typedef struct
{
    tNetTime        netTime;                        ///< Net time given in IEEE 1588 format
    UINT64          relTime;                        ///< Relative time given in us
    BOOL            fValidRelTime;                  ///< TRUE if relative time is validated
} tOplkApiSocTimeInfo;

//------------------------------------------------------------------------------
// function prototypes
//------------------------------------------------------------------------------
#ifdef __cplusplus
extern "C"
{
#endif

// Generic API functions
OPLKDLLEXPORT tOplkError oplk_initialize(void);
OPLKDLLEXPORT tOplkError oplk_create(const tOplkApiInitParam* pInitParam_p);
OPLKDLLEXPORT tOplkError oplk_destroy(void);
OPLKDLLEXPORT void oplk_exit(void);
OPLKDLLEXPORT OPLK_DEPRECATED tOplkError oplk_init(const tOplkApiInitParam* pInitParam_p);
OPLKDLLEXPORT OPLK_DEPRECATED tOplkError oplk_shutdown(void);
OPLKDLLEXPORT tOplkError oplk_enumerateNetworkInterfaces(tNetIfId* pInterfaces_p,
                                                         size_t* pNoInterfaces_p);
OPLKDLLEXPORT tOplkError oplk_execNmtCommand(tNmtEvent NmtEvent_p);
OPLKDLLEXPORT tOplkError oplk_linkObject(UINT objIndex_p,
                                         void* pVar_p,
                                         UINT* pVarEntries_p,
                                         tObdSize* pEntrySize_p,
                                         UINT firstSubindex_p);
OPLKDLLEXPORT tOplkError oplk_readObject(tSdoComConHdl* pSdoComConHdl_p,
                                         UINT nodeId_p,
                                         UINT index_p,
                                         UINT subindex_p,
                                         void* pDstData_le_p,
                                         size_t* pSize_p,
                                         tSdoType sdoType_p,
                                         void* pUserArg_p);
OPLKDLLEXPORT tOplkError oplk_readMultipleObjects(tSdoComConHdl* pSdoComConHdl_p,
                                                  UINT nodeId_p,
                                                  tSdoMultiAccEntry* paSubAcc_p,
                                                  UINT subAccCnt_p,
                                                  tSdoType sdoType_p,
                                                  void* pBuffer_p,
                                                  size_t bufSize_p,
                                                  void* pUserArg_p);
OPLKDLLEXPORT tOplkError oplk_writeObject(tSdoComConHdl* pSdoComConHdl_p,
                                          UINT nodeId_p,
                                          UINT index_p,
                                          UINT subindex_p,
                                          const void* pSrcData_le_p,
                                          size_t size_p,
                                          tSdoType sdoType_p,
                                          void* pUserArg_p);
OPLKDLLEXPORT tOplkError oplk_writeMultipleObjects(tSdoComConHdl* pSdoComConHdl_p,
                                                   UINT nodeId_p,
                                                   tSdoMultiAccEntry* paSubAcc_p,
                                                   UINT subAccCnt_p,
                                                   tSdoType sdoType_p,
                                                   void* pBuffer_p,
                                                   size_t bufSize_p,
                                                   void* pUserArg_p);
OPLKDLLEXPORT tOplkError oplk_finishUserObdAccess(tObdAlConHdl* pUserObdConHdl_p);
OPLKDLLEXPORT tOplkError oplk_enableUserObdAccess(BOOL fEnable_p);
OPLKDLLEXPORT tOplkError oplk_freeSdoChannel(tSdoComConHdl sdoComConHdl_p);
OPLKDLLEXPORT tOplkError oplk_abortSdo(tSdoComConHdl sdoComConHdl_p,
                                       UINT32 abortCode_p);
OPLKDLLEXPORT tOplkError oplk_readLocalObject(UINT index_p,
                                              UINT subindex_p,
                                              void* pDstData_p,
                                              size_t* pSize_p);
OPLKDLLEXPORT tOplkError oplk_writeLocalObject(UINT index_p,
                                               UINT subindex_p,
                                               const void* pSrcData_p,
                                               size_t size_p);
OPLKDLLEXPORT tOplkError oplk_sendAsndFrame(UINT8 dstNodeId_p,
                                            const tAsndFrame* pAsndFrame_p,
                                            size_t asndSize_p);
OPLKDLLEXPORT tOplkError oplk_sendEthFrame(const tPlkFrame* pFrame_p,
                                           size_t frameSize_p);
OPLKDLLEXPORT tOplkError oplk_setAsndForward(UINT8 serviceId_p,
                                             tOplkApiAsndFilter FilterType_p);
OPLKDLLEXPORT tOplkError oplk_setNonPlkForward(BOOL fEnable_p);
OPLKDLLEXPORT tOplkError oplk_postUserEvent(void* pUserArg_p);
OPLKDLLEXPORT tOplkError oplk_triggerMnStateChange(UINT nodeId_p,
                                                   tNmtNodeCommand nodeCommand_p);
OPLKDLLEXPORT tOplkError oplk_setCdcBuffer(const void* pbCdc_p,
                                           size_t cdcSize_p);
OPLKDLLEXPORT tOplkError oplk_setCdcFilename(const char* pszCdcFilename_p);
OPLKDLLEXPORT tOplkError oplk_setOdArchivePath(const char* pBackupPath_p);
OPLKDLLEXPORT tOplkError oplk_process(void);
OPLKDLLEXPORT tOplkError oplk_getIdentResponse(UINT nodeId_p,
                                               const tIdentResponse** ppIdentResponse_p);
OPLKDLLEXPORT tOplkError oplk_getEthMacAddr(UINT8* pMacAddr_p);
OPLKDLLEXPORT BOOL oplk_checkKernelStack(void);
OPLKDLLEXPORT tOplkError oplk_waitSyncEvent(ULONG timeout_p);
OPLKDLLEXPORT UINT32 oplk_getVersion(void);
OPLKDLLEXPORT const char* oplk_getVersionString(void);
OPLKDLLEXPORT UINT32 oplk_getStackConfiguration(void);
OPLKDLLEXPORT tOplkError oplk_getStackInfo(tOplkApiStackInfo* pStackInfo_p);
OPLKDLLEXPORT tOplkError oplk_getSocTime(tOplkApiSocTimeInfo* pTimeInfo_p);
OPLKDLLEXPORT tOplkError oplk_exchangeAppPdoIn(void);
OPLKDLLEXPORT tOplkError oplk_exchangeAppPdoOut(void);

// Process image API functions
OPLKDLLEXPORT tOplkError oplk_allocProcessImage(size_t sizeProcessImageIn_p,
                                                size_t sizeProcessImageOut_p);
OPLKDLLEXPORT tOplkError oplk_freeProcessImage(void);
OPLKDLLEXPORT tOplkError oplk_linkProcessImageObject(UINT objIndex_p,
                                                     UINT firstSubindex_p,
                                                     size_t offsetPI_p,
                                                     BOOL fOutputPI_p,
                                                     tObdSize entrySize_p,
                                                     UINT* pVarEntries_p);
OPLKDLLEXPORT tOplkError oplk_exchangeProcessImageIn(void);
OPLKDLLEXPORT tOplkError oplk_exchangeProcessImageOut(void);
OPLKDLLEXPORT void* oplk_getProcessImageIn(void);
OPLKDLLEXPORT void* oplk_getProcessImageOut(void);

// objdict specific process image functions
OPLKDLLEXPORT OPLK_DEPRECATED tOplkError oplk_setupProcessImage(void);

// Request forwarding of Pres frame from DLL -> API
OPLKDLLEXPORT tOplkError oplk_triggerPresForward(UINT nodeId_p);

// SDO Test API functions
OPLKDLLEXPORT void oplk_testSdoSetVal(const tOplkApiInitParam* pInitParam_p);
OPLKDLLEXPORT tOplkError oplk_testSdoComInit(void);
OPLKDLLEXPORT tOplkError oplk_testSdoSeqInit(void);
// Testing functions for SDO command layer
OPLKDLLEXPORT tOplkError oplk_testSdoComSend(UINT uiNodeId_p,
                                             tSdoType SdoType_p,
                                             const tAsySdoCom* pSdoCom_p,
                                             size_t SdoSize_p);
OPLKDLLEXPORT tOplkError oplk_testSdoComDelCon(void);
// Testing functions for SDO sequence layer
OPLKDLLEXPORT tOplkError oplk_testSdoSeqSend(UINT uiNodeId_p,
                                             tSdoType SdoType_p,
                                             const tAsySdoSeq* pSdoCom_p,
                                             size_t SdoSize_p);
OPLKDLLEXPORT tOplkError oplk_testSdoSeqDelCon(void);

// Service API functions
OPLKDLLEXPORT tOplkError oplk_serviceWriteFileChunk(const tOplkApiFileChunkDesc* pDesc_p,
                                                    const void* pChunkData_p);
OPLKDLLEXPORT size_t oplk_serviceGetFileChunkSize(void);
OPLKDLLEXPORT tOplkError oplk_serviceExecFirmwareReconfig(BOOL fFactory_p);

#ifdef __cplusplus
}
#endif

#endif  /* _INC_oplk_oplk_H_ */
