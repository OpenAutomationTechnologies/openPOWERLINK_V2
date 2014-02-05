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

typedef struct
{
    UINT                m_uiNodeId;
    tNmtState           m_NmtState;
    tNmtNodeEvent       m_NodeEvent;
    UINT16              m_wErrorCode;   // EPL error code if m_NodeEvent == kNmtNodeEventError
    BOOL                m_fMandatory;
} tOplkApiEventNode;


typedef struct
{
    tNmtState           m_NmtState;     // local NMT state
    tNmtBootEvent       m_BootEvent;
    UINT16              m_wErrorCode;   // EPL error code if m_BootEvent == kNmtBootEventError
} tOplkApiEventBoot;

typedef struct
{
    tLedType            m_LedType;      // type of the LED (e.g. Status or Error)
    BOOL                m_fOn;          // state of the LED (e.g. on or off)
} tOplkApiEventLed;

typedef struct
{
    UINT                m_uiNodeId;
    tNmtNodeCommand     m_NodeCommand;
} tOplkApiEventCfmResult;

typedef struct
{
    tPlkFrame           *m_pFrame;
    size_t              m_FrameSize;
}
tOplkApiEventRcvAsnd;

typedef enum
{
    kOplkApiEventUserDef            = 0x00,    // m_pUserArg
    kOplkApiEventNmtStateChange     = 0x10,    // m_NmtStateChange
    kOplkApiEventCriticalError      = 0x12,    // m_InternalError, Stack halted
    kOplkApiEventWarning            = 0x13,    // m_InternalError, Stack running
    kOplkApiEventHistoryEntry       = 0x14,    // m_ErrHistoryEntry
    kOplkApiEventNode               = 0x20,    // m_Node
    kOplkApiEventBoot               = 0x21,    // m_Boot
    kOplkApiEventSdo                = 0x62,    // m_Sdo
    kOplkApiEventObdAccess          = 0x69,    // m_ObdCbParam
    kOplkApiEventLed                = 0x70,    // m_Led
    kOplkApiEventCfmProgress        = 0x71,    // m_CfmProgress
    kOplkApiEventCfmResult          = 0x72,    // m_CfmResult
    kOplkApiEventReceivedAsnd       = 0x73,    // m_RcvAsnd
} tOplkApiEventType;


typedef union
{
    void*                       m_pUserArg;
    tEventNmtStateChange        m_NmtStateChange;
    tEplEventError              m_InternalError;
    tSdoComFinished             m_Sdo;
    tObdCbParam                 m_ObdCbParam;
    tOplkApiEventNode           m_Node;
    tOplkApiEventBoot           m_Boot;
    tOplkApiEventLed            m_Led;
    tCfmEventCnProgress         m_CfmProgress;
    tOplkApiEventCfmResult      m_CfmResult;
    tErrHistoryEntry            m_ErrHistoryEntry;
    tOplkApiEventRcvAsnd        m_RcvAsnd;
} tOplkApiEventArg;

typedef tOplkError (*tOplkApiCbEvent) (
    tOplkApiEventType   EventType_p,   // IN: event type (enum)
    tOplkApiEventArg*   pEventArg_p,   // IN: event argument (union)
    void*               pUserArg_p);

typedef struct
{
    UINT                m_uiSizeOfStruct;
    BOOL                m_fAsyncOnly;               // do not need to register PRes
    UINT                m_uiNodeId;                 // local node ID
    BYTE                m_abMacAddress[6];          // local MAC address
    UINT32              m_dwFeatureFlags;           // 0x1F82: NMT_FeatureFlags_U32
    UINT32              m_dwCycleLen;               // Cycle Length (0x1006: NMT_CycleLen_U32) in [us] - required for error detection
    // 0x1F98: NMT_CycleTiming_REC
    UINT                m_uiIsochrTxMaxPayload;     // 0x1F98.1: IsochrTxMaxPayload_U16 - const
    UINT                m_uiIsochrRxMaxPayload;     // 0x1F98.2: IsochrRxMaxPayload_U16 - const
    UINT32              m_dwPresMaxLatency;         // 0x1F98.3: PResMaxLatency_U32 - const in [ns], only required for IdentRes
    UINT                m_uiPreqActPayloadLimit;    // 0x1F98.4: PReqActPayloadLimit_U16 - required for initialisation (+28 bytes)
    UINT                m_uiPresActPayloadLimit;    // 0x1F98.5: PResActPayloadLimit_U16 - required for initialisation of Pres frame (+28 bytes)
    UINT32              m_dwAsndMaxLatency;         // 0x1F98.6: ASndMaxLatency_U32 - const in [ns], only required for IdentRes
    UINT                m_uiMultiplCycleCnt;        // 0x1F98.7: MultiplCycleCnt_U8 - required for error detection
    UINT                m_uiAsyncMtu;               // 0x1F98.8: AsyncMTU_U16 - required to set up max frame size
    UINT                m_uiPrescaler;              // 0x1F98.9: Prescaler_U16 - required for sync
    UINT32              m_dwLossOfFrameTolerance;   // 0x1C14: DLL_LossOfFrameTolerance_U32 in [ns]
    // 0x1F8A: NMT_MNCycleTiming_REC
    UINT32              m_dwWaitSocPreq;            // 0x1F8A.1: WaitSoCPReq_U32 in [ns] - required on MN
    UINT32              m_dwAsyncSlotTimeout;       // 0x1F8A.2: AsyncSlotTimeout_U32 in [ns] - required on MN

    UINT32              m_dwDeviceType;             // NMT_DeviceType_U32
    UINT32              m_dwVendorId;               // NMT_IdentityObject_REC.VendorId_U32
    UINT32              m_dwProductCode;            // NMT_IdentityObject_REC.ProductCode_U32
    UINT32              m_dwRevisionNumber;         // NMT_IdentityObject_REC.RevisionNo_U32
    UINT32              m_dwSerialNumber;           // NMT_IdentityObject_REC.SerialNo_U32
    UINT64              m_qwVendorSpecificExt1;
    UINT32              m_dwVerifyConfigurationDate;// CFM_VerifyConfiguration_REC.ConfDate_U32
    UINT32              m_dwVerifyConfigurationTime;// CFM_VerifyConfiguration_REC.ConfTime_U32
    UINT32              m_dwApplicationSwDate;      // PDL_LocVerApplSw_REC.ApplSwDate_U32 on programmable device or date portion of NMT_ManufactSwVers_VS on non-programmable device
    UINT32              m_dwApplicationSwTime;      // PDL_LocVerApplSw_REC.ApplSwTime_U32 on programmable device or time portion of NMT_ManufactSwVers_VS on non-programmable device
    UINT32              m_dwIpAddress;
    UINT32              m_dwSubnetMask;
    UINT32              m_dwDefaultGateway;
    UINT8               m_sHostname[32];
    UINT8               m_abVendorSpecificExt2[48];
    char*               m_pszDevName;               // NMT_ManufactDevName_VS (0x1008/0 local OD)
    char*               m_pszHwVersion;             // NMT_ManufactHwVers_VS  (0x1009/0 local OD)
    char*               m_pszSwVersion;             // NMT_ManufactSwVers_VS  (0x100A/0 local OD)
    tOplkApiCbEvent      m_pfnCbEvent;
    void*               m_pEventUserArg;
    tEplSyncCb          m_pfnCbSync;
    tEplHwParam         m_HwParam;
    UINT32              m_dwSyncResLatency;        // constant response latency for SyncRes in [ns]
    // synchronization trigger (AppCbSync, cycle preparation)
    UINT                m_uiSyncNodeId;     // after PRes from CN with this node-ID (0 = SoC, 255 = SoA)
    BOOL                m_fSyncOnPrcNode;   // TRUE: CN is PRes chained; FALSE: conventional CN (PReq/PRes)
} tOplkApiInitParam;

typedef struct
{
    void*          m_pImage;
    UINT           m_uiSize;
} tOplkApiProcessImage;

typedef struct
{
    void*          m_pPart;
    UINT           m_uiOffset;
    UINT           m_uiSize;
} tOplkApiProcessImagePart;

//------------------------------------------------------------------------------
// function prototypes
//------------------------------------------------------------------------------
#ifdef __cplusplus
extern "C" {
#endif

// Generic API functions
EPLDLLEXPORT tOplkError oplk_init(tOplkApiInitParam* pInitParam_p);
EPLDLLEXPORT tOplkError oplk_shutdown(void);
EPLDLLEXPORT tOplkError oplk_execNmtCommand(tNmtEvent NmtEvent_p);
EPLDLLEXPORT tOplkError oplk_linkObject(UINT objIndex_p, void* pVar_p, UINT* pVarEntries_p,
                                        tObdSize* pEntrySize_p, UINT firstSubindex_p);
EPLDLLEXPORT tOplkError oplk_readObject(tSdoComConHdl* pSdoComConHdl_p, UINT  nodeId_p, UINT index_p,
                                        UINT subindex_p, void* pDstData_le_p, UINT* pSize_p,
                                        tSdoType sdoType_p, void* pUserArg_p);
EPLDLLEXPORT tOplkError oplk_writeObject(tSdoComConHdl* pSdoComConHdl_p, UINT nodeId_p, UINT index_p,
                                         UINT subindex_p, void* pSrcData_le_p, UINT size_p,
                                         tSdoType sdoType_p, void* pUserArg_p);
EPLDLLEXPORT tOplkError oplk_freeSdoChannel(tSdoComConHdl sdoComConHdl_p);
EPLDLLEXPORT tOplkError oplk_abortSdo(tSdoComConHdl sdoComConHdl_p, UINT32 abortCode_p);
EPLDLLEXPORT tOplkError oplk_readLocalObject(UINT index_p, UINT subindex_p, void* pDstData_p, UINT* pSize_p);
EPLDLLEXPORT tOplkError oplk_writeLocalObject(UINT index_p, UINT subindex_p, void* pSrcData_p, UINT size_p);
EPLDLLEXPORT tOplkError oplk_sendAsndFrame(UINT8 dstNodeId_p, tAsndFrame *pAsndFrame_p, size_t asndSize_p);
EPLDLLEXPORT tOplkError oplk_setAsndForward(UINT8 serviceId_p, tOplkApiAsndFilter FilterType_p);
EPLDLLEXPORT tOplkError oplk_postUserEvent(void* pUserArg_p);
EPLDLLEXPORT tOplkError oplk_triggerMnStateChange(UINT nodeId_p, tNmtNodeCommand nodeCommand_p);
EPLDLLEXPORT tOplkError oplk_setCdcBuffer(BYTE* pbCdc_p, UINT cdcSize_p);
EPLDLLEXPORT tOplkError oplk_setCdcFilename(char* pszCdcFilename_p);
EPLDLLEXPORT tOplkError oplk_process(void);
EPLDLLEXPORT tOplkError oplk_getIdentResponse(UINT nodeId_p, tIdentResponse** ppIdentResponse_p);
EPLDLLEXPORT BOOL       oplk_checkKernelStack(void);
EPLDLLEXPORT tOplkError oplk_waitSyncEvent(ULONG timeout_p);

// Process image API functions
EPLDLLEXPORT tOplkError oplk_allocProcessImage(UINT sizeProcessImageIn_p, UINT sizeProcessImageOut_p);
EPLDLLEXPORT tOplkError oplk_freeProcessImage(void);
EPLDLLEXPORT tOplkError oplk_linkProcessImageObject(UINT objIndex_p, UINT firstSubindex_p, UINT offsetPI_p,
                                                    BOOL fOutputPI_p, tObdSize entrySize_p, UINT* pVarEntries_p);
EPLDLLEXPORT tOplkError oplk_exchangeProcessImageIn(void);
EPLDLLEXPORT tOplkError oplk_exchangeProcessImageOut(void);
EPLDLLEXPORT void*      oplk_getProcessImageIn(void);
EPLDLLEXPORT void*      oplk_getProcessImageOut(void);

// objdict specific process image functions
EPLDLLEXPORT tOplkError oplk_setupProcessImage(void);

#ifdef __cplusplus
}
#endif

#endif  // #ifndef _INC_oplk_oplk_H_
