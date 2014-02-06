/****************************************************************************

  (c) SYSTEC electronic GmbH, D-07973 Greiz, August-Bebel-Str. 29
      www.systec-electronic.com

  Project:      openPOWERLINK

  Description:  include file for EPL API layer

  License:

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    1. Redistributions of source code must retain the above copyright
       notice, this list of conditions and the following disclaimer.

    2. Redistributions in binary form must reproduce the above copyright
       notice, this list of conditions and the following disclaimer in the
       documentation and/or other materials provided with the distribution.

    3. Neither the name of SYSTEC electronic GmbH nor the names of its
       contributors may be used to endorse or promote products derived
       from this software without prior written permission. For written
       permission, please contact info@systec-electronic.com.

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

  -------------------------------------------------------------------------

                $RCSfile$

                $Author$

                $Revision$  $Date$

                $State$

                Build Environment:
                    GCC V3.4

  -------------------------------------------------------------------------

  Revision History:

  2006/05/22 d.k.:   start of the implementation, version 1.00


****************************************************************************/

#ifndef _EPL_API_H_
#define _EPL_API_H_

#include <oplk/EplInc.h>
#include <oplk/sdo.h>
#include <oplk/obd.h>
#include <oplk/led.h>
#include <oplk/cfm.h>
#include <oplk/event.h>
#include <stddef.h>

//---------------------------------------------------------------------------
// const defines
//---------------------------------------------------------------------------


//---------------------------------------------------------------------------
// typedef
//---------------------------------------------------------------------------
#ifdef __cplusplus
    extern "C" {
#endif

typedef enum
{
    kEplApiAsndFilterNone     = 0x00,
    kEplApiAsndFilterLocal    = 0x01,  // receive only ASnd frames with local or broadcast node ID
    kEplApiAsndFilterAny      = 0x02,  // receive any ASnd frame
} tEplApiAsndFilter;

typedef struct
{
    unsigned int        m_uiNodeId;
    tNmtState           m_NmtState;
    tNmtNodeEvent       m_NodeEvent;
    WORD                m_wErrorCode;   // EPL error code if m_NodeEvent == kNmtNodeEventError
    BOOL                m_fMandatory;

} tEplApiEventNode;


typedef struct
{
    tNmtState           m_NmtState;     // local NMT state
    tNmtBootEvent       m_BootEvent;
    WORD                m_wErrorCode;   // EPL error code if m_BootEvent == kNmtBootEventError

} tEplApiEventBoot;


typedef struct
{
    tLedType            m_LedType;      // type of the LED (e.g. Status or Error)
    BOOL                m_fOn;          // state of the LED (e.g. on or off)

} tEplApiEventLed;


typedef struct
{
    unsigned int        m_uiNodeId;
    tNmtNodeCommand     m_NodeCommand;

} tEplApiEventCfmResult;

typedef struct
{
    tEplFrame           *m_pFrame;
    size_t              m_FrameSize;
}
tEplApiEventRcvAsnd;

typedef enum
{
    kEplApiEventUserDef        = 0x00,    // m_pUserArg
    kEplApiEventNmtStateChange = 0x10,    // m_NmtStateChange
//    kEplApiEventRequestNmt     = 0x11,    // m_bNmtCmd
    kEplApiEventCriticalError  = 0x12,    // m_InternalError, Stack halted
    kEplApiEventWarning        = 0x13,    // m_InternalError, Stack running
    kEplApiEventHistoryEntry   = 0x14,    // m_ErrHistoryEntry
    kEplApiEventNode           = 0x20,    // m_Node
    kEplApiEventBoot           = 0x21,    // m_Boot
    kEplApiEventSdo            = 0x62,    // m_Sdo
    kEplApiEventObdAccess      = 0x69,    // m_ObdCbParam
    kEplApiEventLed            = 0x70,    // m_Led
    kEplApiEventCfmProgress    = 0x71,    // m_CfmProgress
    kEplApiEventCfmResult      = 0x72,    // m_CfmResult
    kEplApiEventReceivedAsnd   = 0x73,    // m_RcvAsnd
} tEplApiEventType;


typedef union
{
    void*                   m_pUserArg;
    tEventNmtStateChange    m_NmtStateChange;
    tEplEventError          m_InternalError;
    tSdoComFinished         m_Sdo;
    tObdCbParam             m_ObdCbParam;
    tEplApiEventNode        m_Node;
    tEplApiEventBoot        m_Boot;
    tEplApiEventLed         m_Led;
    tCfmEventCnProgress     m_CfmProgress;
    tEplApiEventCfmResult   m_CfmResult;
    tEplErrHistoryEntry     m_ErrHistoryEntry;
    tEplApiEventRcvAsnd     m_RcvAsnd;
} tEplApiEventArg;


typedef tEplKernel (PUBLIC ROM* tEplApiCbEvent) (
    tEplApiEventType        EventType_p,   // IN: event type (enum)
    tEplApiEventArg*        pEventArg_p,   // IN: event argument (union)
    void GENERIC*           pUserArg_p);


typedef struct
{
    unsigned int        m_uiSizeOfStruct;
    BOOL                m_fAsyncOnly;   // do not need to register PRes
    unsigned int        m_uiNodeId;     // local node ID
    BYTE                m_abMacAddress[6];  // local MAC address

    // 0x1F82: NMT_FeatureFlags_U32
    DWORD               m_dwFeatureFlags;
    // Cycle Length (0x1006: NMT_CycleLen_U32) in [us]
    DWORD               m_dwCycleLen;           // required for error detection
    // 0x1F98: NMT_CycleTiming_REC
    // 0x1F98.1: IsochrTxMaxPayload_U16
    unsigned int        m_uiIsochrTxMaxPayload; // const
    // 0x1F98.2: IsochrRxMaxPayload_U16
    unsigned int        m_uiIsochrRxMaxPayload; // const
    // 0x1F98.3: PResMaxLatency_U32
    DWORD               m_dwPresMaxLatency;     // const in [ns], only required for IdentRes
    // 0x1F98.4: PReqActPayloadLimit_U16
    unsigned int        m_uiPreqActPayloadLimit;// required for initialisation (+28 bytes)
    // 0x1F98.5: PResActPayloadLimit_U16
    unsigned int        m_uiPresActPayloadLimit;// required for initialisation of Pres frame (+28 bytes)
    // 0x1F98.6: ASndMaxLatency_U32
    DWORD               m_dwAsndMaxLatency;     // const in [ns], only required for IdentRes
    // 0x1F98.7: MultiplCycleCnt_U8
    unsigned int        m_uiMultiplCycleCnt;    // required for error detection
    // 0x1F98.8: AsyncMTU_U16
    unsigned int        m_uiAsyncMtu;           // required to set up max frame size
    // 0x1F98.9: Prescaler_U16
    unsigned int        m_uiPrescaler;          // required for sync
    // $$$ Multiplexed Slot

    // 0x1C14: DLL_LossOfFrameTolerance_U32 in [ns]
    DWORD               m_dwLossOfFrameTolerance;

    // 0x1F8A: NMT_MNCycleTiming_REC
    // 0x1F8A.1: WaitSoCPReq_U32 in [ns]
    DWORD               m_dwWaitSocPreq;        // required on MN

    // 0x1F8A.2: AsyncSlotTimeout_U32 in [ns]
    DWORD               m_dwAsyncSlotTimeout;   // required on MN

    DWORD               m_dwDeviceType;              // NMT_DeviceType_U32
    DWORD               m_dwVendorId;                // NMT_IdentityObject_REC.VendorId_U32
    DWORD               m_dwProductCode;             // NMT_IdentityObject_REC.ProductCode_U32
    DWORD               m_dwRevisionNumber;          // NMT_IdentityObject_REC.RevisionNo_U32
    DWORD               m_dwSerialNumber;            // NMT_IdentityObject_REC.SerialNo_U32
    QWORD               m_qwVendorSpecificExt1;
    DWORD               m_dwVerifyConfigurationDate; // CFM_VerifyConfiguration_REC.ConfDate_U32
    DWORD               m_dwVerifyConfigurationTime; // CFM_VerifyConfiguration_REC.ConfTime_U32
    DWORD               m_dwApplicationSwDate;       // PDL_LocVerApplSw_REC.ApplSwDate_U32 on programmable device or date portion of NMT_ManufactSwVers_VS on non-programmable device
    DWORD               m_dwApplicationSwTime;       // PDL_LocVerApplSw_REC.ApplSwTime_U32 on programmable device or time portion of NMT_ManufactSwVers_VS on non-programmable device
    DWORD               m_dwIpAddress;
    DWORD               m_dwSubnetMask;
    DWORD               m_dwDefaultGateway;
    BYTE                m_sHostname[32];
    BYTE                m_abVendorSpecificExt2[48];

    char*               m_pszDevName;       // NMT_ManufactDevName_VS (0x1008/0 local OD)
    char*               m_pszHwVersion;     // NMT_ManufactHwVers_VS  (0x1009/0 local OD)
    char*               m_pszSwVersion;     // NMT_ManufactSwVers_VS  (0x100A/0 local OD)

    tEplApiCbEvent      m_pfnCbEvent;
    void*               m_pEventUserArg;
    tEplSyncCb          m_pfnCbSync;

    tEplHwParam         m_HwParam;

    DWORD               m_dwSyncResLatency; // constant response latency for SyncRes in [ns]

    // synchronization trigger (AppCbSync, cycle preparation)
    unsigned int        m_uiSyncNodeId;     // after PRes from CN with this node-ID (0 = SoC, 255 = SoA)
    BOOL                m_fSyncOnPrcNode;   // TRUE: CN is PRes chained; FALSE: conventional CN (PReq/PRes)

} tEplApiInitParam;


typedef struct
{
    void*          m_pImage;
    unsigned int   m_uiSize;

} tEplApiProcessImage;


typedef struct
{
    void*          m_pPart;
    unsigned int   m_uiOffset;
    unsigned int   m_uiSize;

} tEplApiProcessImagePart;


typedef struct
{
    tEplApiProcessImagePart    m_In;
    tEplApiProcessImagePart    m_Out;
    unsigned int               m_uiPriority; // 0 = highest
    BOOL                       m_fNonBlocking;

} tEplApiProcessImageCopyJob;

//---------------------------------------------------------------------------
// function prototypes
//---------------------------------------------------------------------------

//------------------------------------------------------------------------------
// Compatibility macros for old API function names - to be removed in future versions!!
#define EplApiInitialize                oplk_init
#define EplApiShutdown                  oplk_shutdown
#define EplApiExecNmtCommand            oplk_execNmtCommand
#define EplApiLinkObject                oplk_linkObject
#define EplApiReadObject                oplk_readObject
#define EplApiWriteObject               oplk_writeObject
#define EplApiFreeSdoChannel            oplk_freeSdoChannel
#define EplApiAbortSdo                  oplk_abortSdo
#define EplApiReadLocalObject           oplk_readLocalObject
#define EplApiWriteLocalObject          oplk_writeLocalObject
#define EplApiSendAsndFrame             oplk_sendAsndFrame
#define EplApiSetAsndForward            oplk_setAsndForward
#define EplApiPostUserEvent             oplk_postUserEvent
#define EplApiMnTriggerStateChange      oplk_triggerMnStateChange
#define EplApiSetCdcBuffer              oplk_setCdcBuffer
#define EplApiSetCdcFilename            oplk_setCdcFilename
#define EplApiProcess                   oplk_process
#define EplApiGetIdentResponse          oplk_getIdentResponse

#define EplApiProcessImageFree          oplk_freeProcessImage
#define EplApiProcessImageAlloc         oplk_allocProcessImage
#define EplApiProcessImageLinkObject    oplk_linkProcessImageObject
//------------------------------------------------------------------------------

// Generic API functions
EPLDLLEXPORT tEplKernel oplk_init(tEplApiInitParam* pInitParam_p);
EPLDLLEXPORT tEplKernel oplk_shutdown(void);
EPLDLLEXPORT tEplKernel oplk_execNmtCommand(tNmtEvent NmtEvent_p);
EPLDLLEXPORT tEplKernel oplk_linkObject(UINT objIndex_p, void* pVar_p, UINT* pVarEntries_p,
                                        tObdSize* pEntrySize_p, UINT firstSubindex_p);
EPLDLLEXPORT tEplKernel oplk_readObject(tSdoComConHdl* pSdoComConHdl_p, UINT  nodeId_p, UINT index_p,
                                        UINT subindex_p, void* pDstData_le_p, UINT* pSize_p,
                                        tSdoType sdoType_p, void* pUserArg_p);
EPLDLLEXPORT tEplKernel oplk_writeObject(tSdoComConHdl* pSdoComConHdl_p, UINT nodeId_p, UINT index_p,
                                         UINT subindex_p, void* pSrcData_le_p, UINT size_p,
                                         tSdoType sdoType_p, void* pUserArg_p);
EPLDLLEXPORT tEplKernel oplk_freeSdoChannel(tSdoComConHdl sdoComConHdl_p);
EPLDLLEXPORT tEplKernel oplk_abortSdo(tSdoComConHdl sdoComConHdl_p, UINT32 abortCode_p);
EPLDLLEXPORT tEplKernel oplk_readLocalObject(UINT index_p, UINT subindex_p, void* pDstData_p, UINT* pSize_p);
EPLDLLEXPORT tEplKernel oplk_writeLocalObject(UINT index_p, UINT subindex_p, void* pSrcData_p, UINT size_p);
EPLDLLEXPORT tEplKernel oplk_sendAsndFrame(UINT8 dstNodeId_p, tEplAsndFrame *pAsndFrame_p, size_t asndSize_p);
EPLDLLEXPORT tEplKernel oplk_setAsndForward(UINT8 serviceId_p, tEplApiAsndFilter FilterType_p);
EPLDLLEXPORT tEplKernel oplk_postUserEvent(void* pUserArg_p);
EPLDLLEXPORT tEplKernel oplk_triggerMnStateChange(UINT nodeId_p, tNmtNodeCommand nodeCommand_p);
EPLDLLEXPORT tEplKernel oplk_setCdcBuffer(BYTE* pbCdc_p, UINT cdcSize_p);
EPLDLLEXPORT tEplKernel oplk_setCdcFilename(char* pszCdcFilename_p);
EPLDLLEXPORT tEplKernel oplk_process(void);
EPLDLLEXPORT tEplKernel oplk_getIdentResponse(UINT nodeId_p, tEplIdentResponse** ppIdentResponse_p);
EPLDLLEXPORT BOOL       oplk_checkKernelStack(void);
EPLDLLEXPORT tEplKernel oplk_waitSyncEvent(ULONG timeout_p);

// Process image API functions
EPLDLLEXPORT tEplKernel oplk_allocProcessImage(UINT sizeProcessImageIn_p, UINT sizeProcessImageOut_p);
EPLDLLEXPORT tEplKernel oplk_freeProcessImage(void);
EPLDLLEXPORT tEplKernel oplk_linkProcessImageObject(UINT objIndex_p, UINT firstSubindex_p, UINT offsetPI_p,
                                                    BOOL fOutputPI_p, tObdSize entrySize_p, UINT* pVarEntries_p);
EPLDLLEXPORT tEplKernel oplk_exchangeProcessImageIn(void);
EPLDLLEXPORT tEplKernel oplk_exchangeProcessImageOut(void);
EPLDLLEXPORT void*      oplk_getProcessImageIn(void);
EPLDLLEXPORT void*      oplk_getProcessImageOut(void);

// objdict specific process image functions
EPLDLLEXPORT tEplKernel oplk_setupProcessImage(void);

#ifdef __cplusplus
    }
#endif


#endif  // #ifndef _EPL_API_H_
