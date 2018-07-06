/**
********************************************************************************
\file   oplk/dll.h

\brief  Definitions for DLL module

This file contains the definitions for the DLL module.

*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2013, SYSTEC electronic GmbH
Copyright (c) 2016, B&R Industrial Automation GmbH
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
#ifndef _INC_oplk_dll_H_
#define _INC_oplk_dll_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <oplk/oplkinc.h>
#include <oplk/frame.h>

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

#ifndef DLL_MAX_ASND_SERVICE_ID
#define DLL_MAX_ASND_SERVICE_ID             (C_DLL_MAX_ASND_SERVICE_IDS + 1)
#endif

#define MAX_PRES_FORWARD_BUFLEN             300

//------------------------------------------------------------------------------
// typedef
//------------------------------------------------------------------------------

/**
\brief Enumeration for ASnd service IDs

The enumeration contains all valid ASnd service IDs.
*/
typedef enum
{
    kDllAsndNotDefined       = 0x00,                        ///< No service
    kDllAsndIdentResponse    = 0x01,                        ///< Send an IdentResponse frame
    kDllAsndStatusResponse   = 0x02,                        ///< Send a StatusResponse frame
    kDllAsndNmtRequest       = 0x03,                        ///< Send an NMT request
    kDllAsndNmtCommand       = 0x04,                        ///< Send an NMT command
    kDllAsndSdo              = 0x05,                        ///< Send an SDO/ASnd frame
#if ((CONFIG_DLL_PRES_CHAINING_CN != FALSE) || defined(CONFIG_INCLUDE_NMT_MN))
    kDllAsndSyncResponse     = 0x06                         ///< Send a SyncResponse frame
#endif
} eDllAsndServiceId;

/**
\brief ASnd service ID data type

Data type for the enumerator \ref eDllAsndServiceId.
*/
typedef UINT8 tDllAsndServiceId;

/**
\brief Enumeration for ASnd filters

The enumeration contains all valid ASnd filters.
*/
typedef enum
{
    kDllAsndFilterNone      = 0x00,                         ///< No filter
    kDllAsndFilterLocal     = 0x01,                         ///< Receive only ASnd frames with local or broadcast node ID
    kDllAsndFilterAny       = 0x02,                         ///< Receive any ASnd frame
} eDllAsndFilter;

/**
\brief ASnd filter data type

Data type for the enumerator \ref eDllAsndFilter.
*/
typedef UINT8 tDllAsndFilter;

/**
\brief Enumeration for request service IDs

The enumeration contains all valid request service IDs.
*/
typedef enum
{
    kDllReqServiceNo         = 0x00,                        ///< No service
    kDllReqServiceIdent      = 0x01,                        ///< Send an IdentRequest frame
    kDllReqServiceStatus     = 0x02,                        ///< Send a StatusRequest frame
    kDllReqServiceNmtRequest = 0x03,                        ///< Invite a node for sending an NMT priority frame
#if (CONFIG_DLL_PRES_CHAINING_CN != FALSE) || defined(CONFIG_INCLUDE_NMT_MN)
    kDllReqServiceSync       = 0x06,                        ///< Send a SyncRequest frame
#endif
    kDllReqServiceUnspecified= 0xFF,                        ///< Invite a node for sending a generic asynchronous frame
} eDllReqServiceId;

/**
\brief Request service ID data type

Data type for the enumerator \ref eDllReqServiceId.
*/
typedef UINT8 tDllReqServiceId;

/**
\brief Enumeration for asynchronous request priorities

The enumeration contains all valid asynchronous request priorities.
*/
typedef enum
{
    kDllAsyncReqPrioNmt      = 0x07,            ///< PRIO_NMT_REQUEST
    kDllAsyncReqPrio6        = 0x06,            ///< Priority 6 (unused)
    kDllAsyncReqPrio5        = 0x05,            ///< Priority 5 (unused)
    kDllAsyncReqPrio4        = 0x04,            ///< Priority 4 (unused)
    kDllAsyncReqPrioGeneric  = 0x03,            ///< PRIO_GENERIC_REQUEST
    kDllAsyncReqPrio2        = 0x02,            ///< Priority 2 (unused - until WSP 0.1.3: PRIO_ABOVE_GENERIC)
    kDllAsyncReqPrio1        = 0x01,            ///< Priority 1 (unused - until WSP 0.1.3: PRIO_BELOW_GENERIC)
    kDllAsyncReqPrio0        = 0x00,            ///< Priority 0 (unused - until WSP 0.1.3: PRIO_GENERIC_REQUEST)
} eDllAsyncReqPriority;

/**
\brief Asynchronous request priorities data type

Data type for the enumerator \ref eDllAsyncReqPriority.
*/
typedef UINT8 tDllAsyncReqPriority;

/**
\brief Structure for frame information

The structure contains all information about a POWERLINK frame.
*/
typedef struct
{
    UINT            frameSize;                      ///< Size of the frame
    UINT32          padding1;                       ///< Padding variable 1
    // Use a union of tPlkFrame pointer variable and 64 bit
    // variable to avoid corruption of pointer variable in
    // heterogeneous processor system. The padding2 variable is
    // used to reserve 8 bytes memory for pBuffer and
    // to ensure that garbage value is cleared before sharing
    // the pointer variable.
    union
    {
        tPlkFrame*      pBuffer;                   ///< Pointer to the frame buffer
        UINT64          padding2;                  ///< 64 bit place holder
    } frame;
} tFrameInfo;

/**
\brief Struct for not received Asnd events

This struct provides information about the unreceived Asnd frame.
*/
typedef struct
{
    UINT8   nodeId;         ///< Source node ID of missed Asnd frame
    UINT8   serviceId;      ///< Service ID of missed Asnd frame
} tDllAsndNotRx;

/**
\brief Structure for DLL configuration

The structure contains all information which is needed for initialization of
the data link layer (DLL).
*/
typedef struct
{
    UINT32              sizeOfStruct;               ///< Size of the structure
    UINT8               fAsyncOnly;                 ///< Async only node, does not need to register PRes-Frame
    UINT8               nodeId;                     ///< Local node ID
    UINT8               padding0[3];                ///< Padding to 32 bit boundary
    UINT32              featureFlags;               ///< 0x1F82: NMT_FeatureFlags_U32
    UINT32              cycleLen;                   ///< Cycle Length (0x1006: NMT_CycleLen_U32) in [us], required for error detection
    UINT16              isochrTxMaxPayload;         ///< 0x1F98.1: IsochrTxMaxPayload_U16
    UINT16              isochrRxMaxPayload;         ///< 0x1F98.2: IsochrRxMaxPayload_U16
    UINT32              presMaxLatency;             ///< 0x1F98.3: PResMaxLatency_U32 in [ns], only required for IdentRes
    UINT16              preqActPayloadLimit;        ///< 0x1F98.4: PReqActPayloadLimit_U16, required for initialization (+24 bytes)
    UINT16              presActPayloadLimit;        ///< 0x1F98.5: PResActPayloadLimit_U16, required for initialization of Pres frame (+24 bytes)
    UINT32              asndMaxLatency;             ///< 0x1F98.6: ASndMaxLatency_U32 in [ns], only required for IdentRes
    UINT8               multipleCycleCnt;           ///< 0x1F98.7: MultiplCycleCnt_U8, required for error detection
    UINT8               padding1[3];                ///< Padding to 32 bit boundary
    UINT16              asyncMtu;                   ///< 0x1F98.8: AsyncMTU_U16, required to set up max frame size
    UINT16              prescaler;                  ///< 0x1F98.9: Prescaler_U16, configures the toggle rate of the SoC PS flag
    // $$$ Multiplexed Slot
    UINT32              lossOfFrameTolerance;       ///< 0x1C14: DLL_LossOfFrameTolerance_U32 in [ns]
    UINT32              waitSocPreq;                ///< 0x1F8A.1: WaitSoCPReq_U32 in [ns]
    UINT32              asyncSlotTimeout;           ///< 0x1F8A.2: AsyncSlotTimeout_U32 in [ns]
    UINT32              syncResLatency;             ///< Constant response latency for SyncRes in [ns]
    UINT32              syncNodeId;                 ///< Synchronization trigger (AppCbSync, cycle preparation) after PRes from CN with this node-ID (0 = SoC, 255 = SoA)
    UINT8               fSyncOnPrcNode;             ///< TRUE: CN is PRes chained; FALSE: conventional CN (PReq/PRes)
#if defined(CONFIG_INCLUDE_NMT_RMN)
    UINT32              switchOverTimeMn;           ///< Switch over time when CS_OPERATIONAL in [us]
    UINT32              reducedSwitchOverTimeMn;    ///< Switch over time when CS_PREOPERATIONAL1 in [us]
    UINT32              delayedSwitchOverTimeMn;    ///< Switch over time otherwise in [us]
#endif
    UINT32              minSyncTime;                ///< Minimum synchronization period supported by the application [us]
} tDllConfigParam;

/**
\brief Structure for DLL Ident Parameters

The structure contains all information needed for the identification of a
node on the network.
*/
typedef struct
{
    UINT32              sizeOfStruct;                   ///< Size of the structure
    UINT32              deviceType;                     ///< NMT_DeviceType_U32
    UINT32              vendorId;                       ///< NMT_IdentityObject_REC.VendorId_U32
    UINT32              productCode;                    ///< NMT_IdentityObject_REC.ProductCode_U32
    UINT32              revisionNumber;                 ///< NMT_IdentityObject_REC.RevisionNo_U32
    UINT32              serialNumber;                   ///< NMT_IdentityObject_REC.SerialNo_U32
    UINT64              vendorSpecificExt1;             ///< Vendor-specific extension field 1
    UINT32              verifyConfigurationDate;        ///< CFM_VerifyConfiguration_REC.ConfDate_U32
    UINT32              verifyConfigurationTime;        ///< CFM_VerifyConfiguration_REC.ConfTime_U32
    UINT32              applicationSwDate;              ///< PDL_LocVerApplSw_REC.ApplSwDate_U32 on programmable device or date portion of NMT_ManufactSwVers_VS on non-programmable device
    UINT32              applicationSwTime;              ///< PDL_LocVerApplSw_REC.ApplSwTime_U32 on programmable device or time portion of NMT_ManufactSwVers_VS on non-programmable device
    UINT32              ipAddress;                      ///< IP address
    UINT32              subnetMask;                     ///< IP subnet mask
    UINT32              defaultGateway;                 ///< IP default gateway address
    UINT8               sHostname[32];                  ///< Hostname
    UINT8               aVendorSpecificExt2[48];        ///< Vendor-specific extension field 2
    UINT32              padding1;                       ///< Padding variable 1
} tDllIdentParam;

/**
\brief Structure for DLL Node Information

The structure contains all information of a node needed by the data link layer
(DLL) module.
*/
typedef struct
{
    UINT                nodeId;                         ///< Node ID
    UINT16              presPayloadLimit;               ///< NMT_PResPayloadLimitList_AU16 (0x1F8D)
#if defined(CONFIG_INCLUDE_NMT_MN)
    UINT16              preqPayloadLimit;               ///< NMT_MNPReqPayloadLimitList_AU16 (0x1F8B)
    UINT32              presTimeoutNs;                  ///< NMT_MNCNPResTimeout_AU32 (0x1F92)
#endif
} tDllNodeInfo;

/**
\brief Enumeration for node operations

The enumeration contains all valid node operations.
*/
typedef enum
{
    kDllNodeOpTypeIsochronous        = 0x00,            ///< Isochronous operation
    kDllNodeOpTypeFilterPdo          = 0x01,            ///< Filter PDO
    kDllNodeOpTypeFilterHeartbeat    = 0x02,            ///< Filter heartbeat
    kDllNodeOpTypeSoftDelete         = 0x03,            ///< Remove the node via soft delete
} eDllNodeOpType;

/**
\brief Node operations data type

Data type for the enumerator \ref eDllNodeOpType.
*/
typedef UINT8 tDllNodeOpType;

/**
\brief Structure for DLL Node Operation Parameters

The structure contains all parameters for a node operation.
*/
typedef struct
{
    UINT                nodeId;             ///< Node ID for this operation
    tDllNodeOpType      opNodeType;         ///< Node operation type
} tDllNodeOpParam;

/**
\brief Structure for DLL Node Operation Parameters

The structure contains all parameters for a node operation.
*/
typedef struct
{
    UINT                nodeId;                         ///< Node ID
    UINT32              syncControl;                    ///< Sync control register
    UINT32              pResTimeFirst;                  ///< PRes time of the first communication path
    UINT32              pResFallBackTimeout;            ///< Timeout to fall back to PReq/PRes mode
} tDllSyncRequest;

/**
\brief Structure for forwarded PRes frames

The structure describes a PRes forwarding event which is used by the
DLL to forward PRes frames to the application.
*/
typedef struct
{
    UINT16          nodeId;                             ///< Node ID from which the PRes was received.
    UINT16          frameSize;                          ///< The size of the received PRes frame.
    UINT8           frameBuf[MAX_PRES_FORWARD_BUFLEN];  ///< The received PRes frame.
} tDllEventReceivedPres;

#endif  /* _INC_oplk_dll_H_ */
