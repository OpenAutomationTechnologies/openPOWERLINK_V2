/**
********************************************************************************
\file   dll.h

\brief  Definitions for DLL module

This file contains the definitions for the DLL module.

*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2013, SYSTEC electronic GmbH
Copyright (c) 2013, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
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

#ifndef _INC_dll_H_
#define _INC_dll_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <oplk/oplkinc.h>
#include <oplk/frame.h>

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

#ifndef DLL_MAX_ASND_SERVICE_ID
#define DLL_MAX_ASND_SERVICE_ID (C_DLL_MAX_ASND_SERVICE_IDS + 1)
#endif

//------------------------------------------------------------------------------
// typedef
//------------------------------------------------------------------------------

/**
\brief Enumeration for ASnd service IDs

The enumeration contains all valid ASnd service IDs.
*/
typedef enum
{
    kDllAsndNotDefined       = 0x00,
    kDllAsndIdentResponse    = 0x01,
    kDllAsndStatusResponse   = 0x02,
    kDllAsndNmtRequest       = 0x03,
    kDllAsndNmtCommand       = 0x04,
    kDllAsndSdo              = 0x05,
#if (CONFIG_DLL_PRES_CHAINING_CN != FALSE) || (CONFIG_DLL_PRES_CHAINING_MN != FALSE)
    kDllAsndSyncResponse     = 0x06
#endif
} tDllAsndServiceId;

/**
\brief Enumeration for ASnd filters

The enumeration contains all valid ASnd filters.
*/
typedef enum
{
    kDllAsndFilterNone      = 0x00,
    kDllAsndFilterLocal     = 0x01,  // receive only ASnd frames with local or broadcast node ID
    kDllAsndFilterAny       = 0x02,  // receive any ASnd frame
} tDllAsndFilter;

/**
\brief Enumeration for request service IDs

The enumeration contains all valid request service IDs.
*/
typedef enum
{
    kDllReqServiceNo         = 0x00,
    kDllReqServiceIdent      = 0x01,
    kDllReqServiceStatus     = 0x02,
    kDllReqServiceNmtRequest = 0x03,
#if (CONFIG_DLL_PRES_CHAINING_CN != FALSE) || (CONFIG_DLL_PRES_CHAINING_MN != FALSE)
    kDllReqServiceSync       = 0x06,
#endif
    kDllReqServiceUnspecified= 0xFF,
} tDllReqServiceId;

/**
\brief Enumeration for asynchronous request priorities

The enumeration contains all valid asynchronous request priorities.
*/
typedef enum
{
    kDllAsyncReqPrioNmt      = 0x07,            ///< PRIO_NMT_REQUEST
    kDllAsyncReqPrio6        = 0x06,
    kDllAsyncReqPrio5        = 0x05,
    kDllAsyncReqPrio4        = 0x04,
    kDllAsyncReqPrioGeneric  = 0x03,            ///< PRIO_GENERIC_REQUEST
    kDllAsyncReqPrio2        = 0x02,            ///< till WSP 0.1.3: PRIO_ABOVE_GENERIC
    kDllAsyncReqPrio1        = 0x01,            ///< till WSP 0.1.3: PRIO_BELOW_GENERIC
    kDllAsyncReqPrio0        = 0x00,            ///< till WSP 0.1.3: PRIO_GENERIC_REQUEST
} tDllAsyncReqPriority;

/**
\brief Structure for frame information

The structure contains all information about a POWERLINK frame.
*/
typedef struct
{
    UINT            frameSize;                      ///< Size of the frame
    tPlkFrame *     pFrame;                         ///< Pointer to the frame
} tFrameInfo;

/**
\brief Struct for not received Asnd events

This struct provides information about the unreceived Asnd frame.
*/
typedef struct
{
    BYTE    nodeId;         ///< Source node ID of missed Asnd frame
    BYTE    serviceId;      ///< Service ID of missed Asnd frame
} tDllAsndNotRx;

/**
\brief Structure for DLL configuration

The structure contains all information which is needed for initialization of
the data link layer (DLL).
*/
typedef struct
{
    UINT                sizeOfStruct;
    BOOL                fAsyncOnly;                 ///< Async only node, does not need to register PRes-Frame
    UINT                nodeId;                     ///< Local node ID
    UINT32              featureFlags;               ///< 0x1F82: NMT_FeatureFlags_U32
    UINT32              cycleLen;                   ///< Cycle Length (0x1006: NMT_CycleLen_U32) in [us], required for error detection
    UINT                isochrTxMaxPayload;         ///< 0x1F98.1: IsochrTxMaxPayload_U16
    UINT                isochrRxMaxPayload;         ///< 0x1F98.2: IsochrRxMaxPayload_U16
    UINT32              presMaxLatency;             ///< 0x1F98.3: PResMaxLatency_U32 in [ns], only required for IdentRes
    UINT                preqActPayloadLimit;        ///< 0x1F98.4: PReqActPayloadLimit_U16, required for initialisation (+24 bytes)
    UINT                presActPayloadLimit;        ///< 0x1F98.5: PResActPayloadLimit_U16, required for initialisation of Pres frame (+24 bytes)
    UINT32              asndMaxLatency;             ///< 0x1F98.6: ASndMaxLatency_U32 in [ns], only required for IdentRes
    UINT                multipleCycleCnt;           ///< 0x1F98.7: MultiplCycleCnt_U8, required for error detection
    UINT                asyncMtu;                   ///< 0x1F98.8: AsyncMTU_U16, required to set up max frame size
    // $$$ 0x1F98.9: Prescaler_U16
    UINT                prescaler;                  ///< 0x1F98.9: Prescaler_U16, configures the toggle rate of the SoC PS flag
    // $$$ Multiplexed Slot
    UINT32              lossOfFrameTolerance;       ///< 0x1C14: DLL_LossOfFrameTolerance_U32 in [ns]
    UINT32              waitSocPreq;                ///< 0x1F8A.1: WaitSoCPReq_U32 in [ns]
    UINT32              asyncSlotTimeout;           ///< 0x1F8A.2: AsyncSlotTimeout_U32 in [ns]
    UINT32              syncResLatency;             ///< Constant response latency for SyncRes in [ns]
    UINT                syncNodeId;                 ///< Synchronization trigger (AppCbSync, cycle preparation) after PRes from CN with this node-ID (0 = SoC, 255 = SoA)
    BOOL                fSyncOnPrcNode;             ///< TRUE: CN is PRes chained; FALSE: conventional CN (PReq/PRes)
} tDllConfigParam;

/**
\brief Structure for DLL Ident Parameters

The structure contains all information needed for the identification of a
node on the network.
*/
typedef struct
{
    UINT                sizeOfStruct;
    UINT32              deviceType;                     ///< NMT_DeviceType_U32
    UINT32              vendorId;                       ///< NMT_IdentityObject_REC.VendorId_U32
    UINT32              productCode;                    ///< NMT_IdentityObject_REC.ProductCode_U32
    UINT32              revisionNumber;                 ///< NMT_IdentityObject_REC.RevisionNo_U32
    UINT32              serialNumber;                   ///< NMT_IdentityObject_REC.SerialNo_U32
    UINT64              vendorSpecificExt1;
    UINT32              verifyConfigurationDate;        ///< CFM_VerifyConfiguration_REC.ConfDate_U32
    UINT32              verifyConfigurationTime;        ///< CFM_VerifyConfiguration_REC.ConfTime_U32
    UINT32              applicationSwDate;              ///< PDL_LocVerApplSw_REC.ApplSwDate_U32 on programmable device or date portion of NMT_ManufactSwVers_VS on non-programmable device
    UINT32              applicationSwTime;              ///< PDL_LocVerApplSw_REC.ApplSwTime_U32 on programmable device or time portion of NMT_ManufactSwVers_VS on non-programmable device
    UINT32              ipAddress;
    UINT32              subnetMask;
    UINT32              defaultGateway;
    UINT8               sHostname[32];
    UINT8               aVendorSpecificExt2[48];
} tDllIdentParam;

/**
\brief Structure for DLL Node Information

The structure contains all information of a node needed by the data link layer
(DLL) module.
*/
typedef struct
{
    UINT                nodeId;
    UINT16              presPayloadLimit;               ///< object 0x1F8D: NMT_PResPayloadLimitList_AU16
#if defined(CONFIG_INCLUDE_NMT_MN)
    UINT16              preqPayloadLimit;               ///< object 0x1F8B: NMT_MNPReqPayloadLimitList_AU16
    UINT32              presTimeoutNs;                  ///< object 0x1F92: NMT_MNCNPResTimeout_AU32
#endif
} tDllNodeInfo;

/**
\brief Enumeration for node operations

The enumeration contains all valid node operations.
*/
typedef enum
{
    kDllNodeOpTypeIsochronous        = 0x00,
    kDllNodeOpTypeFilterPdo          = 0x01,
    kDllNodeOpTypeFilterHeartbeat    = 0x02,
    kDllNodeOpTypeSoftDelete         = 0x03,
} tDllNodeOpType;

/**
\brief Structure for DLL Node Operation Parameters

The structure contains all parameters for a node operation.
*/
typedef struct
{
    UINT                nodeId;             ///< Node ID for this operation
    tDllNodeOpType      opNodeType;         ///< Node operation type
} tDllNodeOpParam;

#if CONFIG_DLL_PRES_CHAINING_MN != FALSE
/**
\brief Structure for DLL Node Operation Parameters

The structure contains all parameters for a node operation.
*/
typedef struct
{
    UINT                nodeId;
    UINT32              syncControl;
    UINT32              pResTimeFirst;
    UINT32              pResFallBackTimeout;
} tDllSyncRequest;
#endif

//------------------------------------------------------------------------------
// function prototypes
//------------------------------------------------------------------------------
#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
}
#endif

#endif  // #ifndef _INC_dll_H_

