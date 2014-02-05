/**
********************************************************************************
\file   frame.h

\brief  Definitions for POWERLINK frames

This header file contains definitions describing POWERLINK frames.
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

#ifndef _INC_oplk_frame_H_
#define _INC_oplk_frame_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <oplk/oplkinc.h>

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
// defines for EplFrame.m_wFlag
#define EPL_FRAME_FLAG1_RD          0x01    // ready                                    (PReq, PRes)
#define EPL_FRAME_FLAG1_ER          0x02    // exception reset (error signalling)       (SoA)
#define EPL_FRAME_FLAG1_EA          0x04    // exception acknowledge (error signalling) (PReq, SoA)
#define EPL_FRAME_FLAG1_EC          0x08    // exception clear (error signalling)       (StatusRes)
#define EPL_FRAME_FLAG1_EN          0x10    // exception new (error signalling)         (PRes, StatusRes)
#define EPL_FRAME_FLAG1_MS          0x20    // multiplexed slot                         (PReq)
#define EPL_FRAME_FLAG1_PS          0x40    // prescaled slot                           (SoC)
#define EPL_FRAME_FLAG1_MC          0x80    // multiplexed cycle completed              (SoC)
#define EPL_FRAME_FLAG2_RS          0x07    // number of pending requests to send       (PRes, StatusRes, IdentRes)
#define EPL_FRAME_FLAG2_PR          0x38    // priority of requested asynch. frame      (PRes, StatusRes, IdentRes)
#define EPL_FRAME_FLAG2_PR_SHIFT    3       // shift of priority of requested asynch. frame

// error history/status entry types
#define EPL_ERR_ENTRYTYPE_STATUS        0x8000
#define EPL_ERR_ENTRYTYPE_HISTORY       0x0000
#define EPL_ERR_ENTRYTYPE_EMCY          0x4000
#define EPL_ERR_ENTRYTYPE_MODE_ACTIVE   0x1000
#define EPL_ERR_ENTRYTYPE_MODE_CLEARED  0x2000
#define EPL_ERR_ENTRYTYPE_MODE_OCCURRED 0x3000
#define EPL_ERR_ENTRYTYPE_MODE_MASK     0x3000
#define EPL_ERR_ENTRYTYPE_PROF_VENDOR   0x0001
#define EPL_ERR_ENTRYTYPE_PROF_EPL      0x0002
#define EPL_ERR_ENTRYTYPE_PROF_MASK     0x0FFF

// defines for EPL version / PDO version
#define EPL_VERSION_SUB             0x0F  // sub version
#define EPL_VERSION_MAIN            0xF0  // main version


#define EPL_FRAME_OFFSET_DST_MAC        0
#define EPL_FRAME_OFFSET_SRC_MAC        6
#define EPL_FRAME_OFFSET_ETHER_TYPE     12
#define EPL_FRAME_OFFSET_MSG_TYPE       14
#define EPL_FRAME_OFFSET_DST_NODEID     15
#define EPL_FRAME_OFFSET_SRC_NODEID     16
#define EPL_FRAME_OFFSET_PDO_PAYLOAD    24

// defines for bit fields SyncControl and SyncStatus
#define EPL_SYNC_PRES_TIME_FIRST_VALID          0x00000001
#define EPL_SYNC_PRES_TIME_SECOND_VALID         0x00000002
#define EPL_SYNC_SYNC_MN_DELAY_FIRST_VALID      0x00000004
#define EPL_SYNC_SYNC_MN_DELAY_SECOND_VALID     0x00000008
#define EPL_SYNC_PRES_FALL_BACK_TIMEOUT_VALID   0x00000010
#define EPL_SYNC_DEST_MAC_ADDRESS_VALID         0x00000020
#define EPL_SYNC_PRES_MODE_RESET                0x40000000
#define EPL_SYNC_PRES_MODE_SET                  0x80000000

// defines for SDO command layer
#define SDO_CMDL_HDR_FIXED_SIZE             8       // size of fixed header part
#define SDO_CMDL_HDR_VAR_SIZE               4       // size of variable header part
#define SDO_CMDL_HDR_WRITEBYINDEX_SIZE      4       // size of write by index header (index + subindex + reserved)
#define SDO_CMDL_HDR_READBYINDEX_SIZE       4       // size of read by index header (index + subindex + reserved)

// defines for SDO command layer flags
#define SDO_CMDL_FLAG_RESPONSE       0x80
#define SDO_CMDL_FLAG_ABORT          0x40
#define SDO_CMDL_FLAG_EXPEDITED      0x00
#define SDO_CMDL_FLAG_SEGMINIT       0x10
#define SDO_CMDL_FLAG_SEGMENTED      0x20
#define SDO_CMDL_FLAG_SEGMCOMPL      0x30
#define SDO_CMDL_FLAG_SEGM_MASK      0x30

//------------------------------------------------------------------------------
// typedef
//------------------------------------------------------------------------------

// byte-align structures
#ifdef _MSC_VER
#pragma pack(push, packing)
#pragma pack(1)
#define PACK_STRUCT
#elif defined(__GNUC__)
#define PACK_STRUCT    __attribute__((packed))
#else
#error you must byte-align these structures with the appropriate compiler directives
#endif


typedef struct
{
    UINT8                   m_le_bRes1;                     ///< Offset 17: reserved
    UINT8                   m_le_bFlag1;                    ///< Offset 18: Flags: MC, PS
    UINT8                   m_le_bFlag2;                    ///< Offset 19: Flags: res
    tEplNetTime             m_le_NetTime;                   ///< Offset 20: supported if D_NMT_NetTimeIsRealTime_BOOL is set
    UINT64                  m_le_RelativeTime;              ///< Offset 28: in us (supported if D_NMT_RelativeTime_BOOL is set)
} PACK_STRUCT tSocFrame;

typedef struct
{
    UINT8                   m_le_bRes1;                     ///< Offset 17: reserved
    UINT8                   m_le_bFlag1;                    ///< Offset 18: Flags: MS, EA, RD
    UINT8                   m_le_bFlag2;                    ///< Offset 19: Flags: res
    UINT8                   m_le_bPdoVersion;               ///< Offset 20: PDO Version
    UINT8                   m_le_bRes2;                     ///< Offset 21: reserved
    UINT16                  m_le_wSize;                     ///< Offset 22:
    UINT8                   m_le_abPayload[256];            ///< Offset 24: Payload
} PACK_STRUCT tPreqFrame;

typedef struct
{
    UINT8                   m_le_bNmtStatus;                ///< Offset 17: NMT state
    UINT8                   m_le_bFlag1;                    ///< Offset 18: Flags: MS, EN, RD
    UINT8                   m_le_bFlag2;                    ///< Offset 19: Flags: PR, RS
    UINT8                   m_le_bPdoVersion;               ///< Offset 20:
    UINT8                   m_le_bRes2;                     ///< Offset 21: reserved
    UINT16                  m_le_wSize;                     ///< Offset 22:
    UINT8                   m_le_abPayload[256];            ///< Offset 24: Payload
} PACK_STRUCT tPresFrame;

typedef struct
{
    UINT8                   m_le_bReserved;                 ///< Offset 23:
    UINT32                  m_le_dwSyncControl;
    UINT32                  m_le_dwPResTimeFirst;
    UINT32                  m_le_dwPResTimeSecond;
    UINT32                  m_le_dwSyncMnDelayFirst;
    UINT32                  m_le_dwSyncMnDelaySecond;
    UINT32                  m_le_dwPResFallBackTimeout;
    UINT8                   m_be_abDestMacAddress[6];
} PACK_STRUCT tSyncRequest;

typedef union
{
    tSyncRequest            m_SyncRequest;                  ///< Offset 23
} tSoaPayload;

typedef struct
{
    UINT8                   m_le_bNmtStatus;                ///< Offset 17:NMT state
    UINT8                   m_le_bFlag1;                    ///< Offset 18: Flags: EA, ER
    UINT8                   m_le_bFlag2;                    ///< Offset 19: Flags: res
    UINT8                   m_le_bReqServiceId;             ///< Offset 20:
    UINT8                   m_le_bReqServiceTarget;         ///< Offset 21:
    UINT8                   m_le_bEplVersion;               ///< Offset 22:
    tSoaPayload             m_Payload;                      ///< Offset 23:
} PACK_STRUCT tSoaFrame;

typedef struct
{
    UINT16                  m_wEntryType;
    UINT16                  m_wErrorCode;
    tEplNetTime             m_TimeStamp;
    UINT8                   m_abAddInfo[8];
} PACK_STRUCT tErrHistoryEntry;

typedef struct
{
    UINT8                   m_le_bFlag1;                    ///< Offset 18: Flags: EN, EC
    UINT8                   m_le_bFlag2;                    ///< Offset 19: Flags: PR, RS
    UINT8                   m_le_bNmtStatus;                ///< Offset 20: NMT state
    UINT8                   m_le_bRes1[3];
    UINT64                  m_le_qwStaticError;             ///< static error bit field
    tErrHistoryEntry        m_le_aErrHistoryEntry[14];
} PACK_STRUCT tStatusResponse;

typedef struct
{
    UINT8                   m_le_bFlag1;                    ///< Offset 18: Flags: res
    UINT8                   m_le_bFlag2;                    ///< Flags: PR, RS
    UINT8                   m_le_bNmtStatus;                ///< NMT state
    UINT8                   m_le_bIdentRespFlags;           ///< Flags: FW
    UINT8                   m_le_bEplProfileVersion;
    UINT8                   m_le_bRes1;
    UINT32                  m_le_dwFeatureFlags;            ///< NMT_FeatureFlags_U32
    UINT16                  m_le_wMtu;                      ///< NMT_CycleTiming_REC.AsyncMTU_U16: C_IP_MIN_MTU - C_IP_MAX_MTU
    UINT16                  m_le_wPollInSize;               ///< NMT_CycleTiming_REC.PReqActPayload_U16
    UINT16                  m_le_wPollOutSize;              ///< NMT_CycleTiming_REC.PResActPayload_U16
    UINT32                  m_le_dwResponseTime;            ///< NMT_CycleTiming_REC.PResMaxLatency_U32
    UINT16                  m_le_wRes2;
    UINT32                  m_le_dwDeviceType;              ///< NMT_DeviceType_U32
    UINT32                  m_le_dwVendorId;                ///< NMT_IdentityObject_REC.VendorId_U32
    UINT32                  m_le_dwProductCode;             ///< NMT_IdentityObject_REC.ProductCode_U32
    UINT32                  m_le_dwRevisionNumber;          ///< NMT_IdentityObject_REC.RevisionNo_U32
    UINT32                  m_le_dwSerialNumber;            ///< NMT_IdentityObject_REC.SerialNo_U32
    UINT64                  m_le_qwVendorSpecificExt1;
    UINT32                  m_le_dwVerifyConfigurationDate; ///< CFM_VerifyConfiguration_REC.ConfDate_U32
    UINT32                  m_le_dwVerifyConfigurationTime; ///< CFM_VerifyConfiguration_REC.ConfTime_U32
    UINT32                  m_le_dwApplicationSwDate;       ///< PDL_LocVerApplSw_REC.ApplSwDate_U32 on programmable device or date portion of NMT_ManufactSwVers_VS on non-programmable device
    UINT32                  m_le_dwApplicationSwTime;       ///< PDL_LocVerApplSw_REC.ApplSwTime_U32 on programmable device or time portion of NMT_ManufactSwVers_VS on non-programmable device
    UINT32                  m_le_dwIpAddress;
    UINT32                  m_le_dwSubnetMask;
    UINT32                  m_le_dwDefaultGateway;
    UINT8                   m_le_sHostname[32];
    UINT8                   m_le_abVendorSpecificExt2[48];
} PACK_STRUCT tIdentResponse;

typedef struct
{
    UINT8                   m_le_bNmtCommandId;             ///< Offset 18:
    UINT8                   m_le_bRes1;
    UINT8                   m_le_abNmtCommandData[32];
} PACK_STRUCT tNmtCommandService;

typedef struct
{
    UINT16                  m_le_wReserved;                 ///< Offset 18:
    UINT32                  m_le_dwSyncStatus;
    UINT32                  m_le_dwLatency;
    UINT32                  m_le_dwSyncNodeNumber;
    UINT32                  m_le_dwSyncDelay;
    UINT32                  m_le_dwPResTimeFirst;
    UINT32                  m_le_dwPResTimeSecond;
} PACK_STRUCT tSyncResponse;

typedef struct
{
    UINT8                   m_le_bReserved;
    UINT8                   m_le_bTransactionId;
    UINT8                   m_le_bFlags;
    UINT8                   m_le_bCommandId;
    UINT16                  m_le_wSegmentSize;
    UINT16                  m_le_wReserved;
    UINT8                   m_le_abCommandData[8];          // just reserve a minimum number of bytes as a placeholder
}PACK_STRUCT tAsySdoCom;


// asynchronous SDO Sequence Header
typedef struct
{
    UINT8                   m_le_bRecSeqNumCon;
    UINT8                   m_le_bSendSeqNumCon;
    UINT8                   m_le_abReserved[2];
    tAsySdoCom              m_le_abSdoSeqPayload;
} PACK_STRUCT tAsySdoSeq;

typedef struct
{
    // Offset 18
    UINT8                   m_le_bNmtCommandId;
    UINT8                   m_le_bTargetNodeId;
    UINT8                   m_le_abNmtCommandData[32];
} PACK_STRUCT tNmtRequestService;


typedef union
{
    tStatusResponse         m_StatusResponse;               ///< Offset 18:
    tIdentResponse          m_IdentResponse;
    tNmtCommandService      m_NmtCommandService;
    tNmtRequestService      m_NmtRequestService;
    tAsySdoSeq              m_SdoSequenceFrame;
    tSyncResponse           m_SyncResponse;
    UINT8                   m_le_abPayload[256];
} tAsndPayload;

typedef struct
{
    UINT8                   m_le_bServiceId;                ///< Offset 17
    tAsndPayload            m_Payload;                      ///< Offset 18
} PACK_STRUCT tAsndFrame;

typedef union
{
    tSocFrame               m_Soc;                          ///< Offset 17
    tPreqFrame              m_Preq;
    tPresFrame              m_Pres;
    tSoaFrame               m_Soa;
    tAsndFrame              m_Asnd;
} tFrameData;

typedef struct
{
    UINT8                   m_be_abDstMac[6];               ///< Offset 0: MAC address of the addressed nodes
    UINT8                   m_be_abSrcMac[6];               ///< Offset 6: MAC address of the transmitting node
    UINT16                  m_be_wEtherType;                ///< Offset 12: Ethernet message type (big endian)
    UINT8                   m_le_bMessageType;              ///< Offset 14: EPL message type
    UINT8                   m_le_bDstNodeId;                ///< Offset 15: EPL node ID of the addressed nodes
    UINT8                   m_le_bSrcNodeId;                ///< Offset 16: EPL node ID of the transmitting node
    tFrameData              m_Data;                         ///< Offset 17:
} PACK_STRUCT tPlkFrame;

// reset byte-align of structures
#ifdef _MSC_VER
#pragma pack(pop, packing)
#endif


typedef enum
{
    kEplMsgTypeNonEpl = 0x00,
    kEplMsgTypeSoc    = 0x01,
    kEplMsgTypePreq   = 0x03,
    kEplMsgTypePres   = 0x04,
    kEplMsgTypeSoa    = 0x05,
    kEplMsgTypeAsnd   = 0x06,
    kEplMsgTypeAInv   = 0x0D,
} tMsgType;

#endif /* _INC_oplk_frame_H_ */
