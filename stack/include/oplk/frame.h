/**
********************************************************************************
\file   oplk/frame.h

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
// Defines for tFrameData.flag1 and tFrameData.flag2
#define PLK_FRAME_FLAG1_RD          0x01    // ready                                    (PReq, PRes)
#define PLK_FRAME_FLAG1_ER          0x02    // exception reset (error signalling)       (SoA)
#define PLK_FRAME_FLAG1_EA          0x04    // exception acknowledge (error signalling) (PReq, SoA)
#define PLK_FRAME_FLAG1_EC          0x08    // exception clear (error signalling)       (StatusRes)
#define PLK_FRAME_FLAG1_EN          0x10    // exception new (error signalling)         (PRes, StatusRes)
#define PLK_FRAME_FLAG1_MS          0x20    // multiplexed slot                         (PReq)
#define PLK_FRAME_FLAG1_PS          0x40    // prescaled slot                           (SoC)
#define PLK_FRAME_FLAG1_MC          0x80    // multiplexed cycle completed              (SoC)
#define PLK_FRAME_FLAG2_RS          0x07    // number of pending requests to send       (PRes, StatusRes, IdentRes)
#define PLK_FRAME_FLAG2_PR          0x38    // priority of requested asynch. frame      (PRes, StatusRes, IdentRes)
#define PLK_FRAME_FLAG2_PR_SHIFT    3       // shift of priority of requested asynch. frame

// error history/status entry types
#define ERR_ENTRYTYPE_STATUS        0x8000
#define ERR_ENTRYTYPE_HISTORY       0x0000
#define ERR_ENTRYTYPE_EMCY          0x4000
#define ERR_ENTRYTYPE_MODE_ACTIVE   0x1000
#define ERR_ENTRYTYPE_MODE_CLEARED  0x2000
#define ERR_ENTRYTYPE_MODE_OCCURRED 0x3000
#define ERR_ENTRYTYPE_MODE_MASK     0x3000
#define ERR_ENTRYTYPE_PROF_VENDOR   0x0001
#define ERR_ENTRYTYPE_PROF_EPL      0x0002
#define ERR_ENTRYTYPE_PROF_MASK     0x0FFF

// defines for POWERLINK version / PDO version
#define PLK_VERSION_SUB             0x0F  // sub version
#define PLK_VERSION_MAIN            0xF0  // main version


#define PLK_FRAME_OFFSET_DST_MAC        0
#define PLK_FRAME_OFFSET_SRC_MAC        6
#define PLK_FRAME_OFFSET_ETHER_TYPE     12
#define PLK_FRAME_OFFSET_MSG_TYPE       14
#define PLK_FRAME_OFFSET_DST_NODEID     15
#define PLK_FRAME_OFFSET_SRC_NODEID     16
#define PLK_FRAME_OFFSET_PDO_PAYLOAD    24

// defines for bit fields SyncControl and SyncStatus
#define PLK_SYNC_PRES_TIME_FIRST_VALID          0x00000001
#define PLK_SYNC_PRES_TIME_SECOND_VALID         0x00000002
#define PLK_SYNC_SYNC_MN_DELAY_FIRST_VALID      0x00000004
#define PLK_SYNC_SYNC_MN_DELAY_SECOND_VALID     0x00000008
#define PLK_SYNC_PRES_FALL_BACK_TIMEOUT_VALID   0x00000010
#define PLK_SYNC_DEST_MAC_ADDRESS_VALID         0x00000020
#define PLK_SYNC_PRES_MODE_RESET                0x40000000
#define PLK_SYNC_PRES_MODE_SET                  0x80000000

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
#error You must Byte-align these structures with the appropriate compiler directives
#endif

typedef struct
{
    UINT8                   reserved1;                      ///< Offset 17: reserved
    UINT8                   flag1;                          ///< Offset 18: Flags: MC, PS
    UINT8                   flag2;                          ///< Offset 19: Flags: res
    tNetTime                netTimeLe;                      ///< Offset 20: supported if D_NMT_NetTimeIsRealTime_BOOL is set
    UINT64                  relativeTimeLe;                 ///< Offset 28: in us (supported if D_NMT_RelativeTime_BOOL is set)
} PACK_STRUCT tSocFrame;

typedef struct
{
    UINT8                   reserved1;                      ///< Offset 17: reserved
    UINT8                   flag1;                          ///< Offset 18: Flags: MS, EA, RD
    UINT8                   flag2;                          ///< Offset 19: Flags: res
    UINT8                   pdoVersion;                     ///< Offset 20: PDO Version
    UINT8                   reserved2;                      ///< Offset 21: reserved
    UINT16                  sizeLe;                         ///< Offset 22:
    UINT8                   aPayload[256];                  ///< Offset 24: Payload
} PACK_STRUCT tPreqFrame;

typedef struct
{
    UINT8                   nmtStatus;                      ///< Offset 17: NMT state
    UINT8                   flag1;                          ///< Offset 18: Flags: MS, EN, RD
    UINT8                   flag2;                          ///< Offset 19: Flags: PR, RS
    UINT8                   pdoVersion;                     ///< Offset 20:
    UINT8                   reserved2;                      ///< Offset 21: reserved
    UINT16                  sizeLe;                         ///< Offset 22:
    UINT8                   aPayload[256];                  ///< Offset 24: Payload
} PACK_STRUCT tPresFrame;

typedef struct
{
    UINT8                   reserved;                       ///< Offset 23:
    UINT32                  syncControlLe;
    UINT32                  presTimeFirstLe;
    UINT32                  presTimeSecondLe;
    UINT32                  syncMnDelayFirstLe;
    UINT32                  syncMnDelaySecondLe;
    UINT32                  presFallBackTimeoutLe;
    UINT8                   aDestMacAddress[6];
} PACK_STRUCT tSyncRequest;

typedef union
{
    tSyncRequest            syncRequest;                    ///< Offset 23
} tSoaPayload;

typedef struct
{
    UINT8                   nmtStatus;                      ///< Offset 17:NMT state
    UINT8                   flag1;                          ///< Offset 18: Flags: EA, ER
    UINT8                   flag2;                          ///< Offset 19: Flags: res
    UINT8                   reqServiceId;                   ///< Offset 20:
    UINT8                   reqServiceTarget;               ///< Offset 21:
    UINT8                   powerlinkVersion;               ///< Offset 22:
    tSoaPayload             payload;                        ///< Offset 23:
} PACK_STRUCT tSoaFrame;

/**
\brief Structure for error history entries

The following structure defines an error history entry.
*/
typedef struct
{
    UINT16                  entryType;                      ///< The type of the entry
    UINT16                  errorCode;                      ///< The error code of the entry
    tNetTime                timeStamp;                      ///< The timestamp when the error was added
    UINT8                   aAddInfo[8];                    ///< Additional error information
} PACK_STRUCT tErrHistoryEntry;

typedef struct
{
    UINT8                   flag1;                          ///< Offset 18: Flags: EN, EC
    UINT8                   flag2;                          ///< Offset 19: Flags: PR, RS
    UINT8                   nmtStatus;                      ///< Offset 20: NMT state
    UINT8                   reserved1[3];
    UINT64                  staticErrorLe;                  ///< static error bit field
    tErrHistoryEntry        aErrorHistoryEntry[14];
} PACK_STRUCT tStatusResponse;

typedef struct
{
    UINT8                   flag1;                          ///< Offset 18: Flags: res
    UINT8                   flag2;                          ///< Flags: PR, RS
    UINT8                   nmtStatus;                      ///< NMT state
    UINT8                   identResponseFlags;             ///< Flags: FW
    UINT8                   powerlinkProfileVersion;
    UINT8                   reserved1;
    UINT32                  featureFlagsLe;                 ///< NMT_FeatureFlags_U32
    UINT16                  mtuLe;                          ///< NMT_CycleTiming_REC.AsyncMTU_U16: C_IP_MIN_MTU - C_IP_MAX_MTU
    UINT16                  pollInSizeLe;                   ///< NMT_CycleTiming_REC.PReqActPayload_U16
    UINT16                  pollOutSizeLe;                  ///< NMT_CycleTiming_REC.PResActPayload_U16
    UINT32                  responseTimeLe;                 ///< NMT_CycleTiming_REC.PResMaxLatency_U32
    UINT16                  reserved2;
    UINT32                  deviceTypeLe;                   ///< NMT_DeviceType_U32
    UINT32                  vendorIdLe;                     ///< NMT_IdentityObject_REC.VendorId_U32
    UINT32                  productCodeLe;                  ///< NMT_IdentityObject_REC.ProductCode_U32
    UINT32                  revisionNumberLe;               ///< NMT_IdentityObject_REC.RevisionNo_U32
    UINT32                  serialNumberLe;                 ///< NMT_IdentityObject_REC.SerialNo_U32
    UINT64                  vendorSpecificExt1Le;
    UINT32                  verifyConfigurationDateLe;      ///< CFM_VerifyConfiguration_REC.ConfDate_U32
    UINT32                  verifyConfigurationTimeLe;      ///< CFM_VerifyConfiguration_REC.ConfTime_U32
    UINT32                  applicationSwDateLe;            ///< PDL_LocVerApplSw_REC.ApplSwDate_U32 on programmable device or date portion of NMT_ManufactSwVers_VS on non-programmable device
    UINT32                  applicationSwTimeLe;            ///< PDL_LocVerApplSw_REC.ApplSwTime_U32 on programmable device or time portion of NMT_ManufactSwVers_VS on non-programmable device
    UINT32                  ipAddressLe;
    UINT32                  subnetMaskLe;
    UINT32                  defaultGatewayLe;
    UINT8                   sHostName[32];
    UINT8                   aVendorSpecificExt2[48];
} PACK_STRUCT tIdentResponse;

typedef struct
{
    UINT8                   nmtCommandId;                   ///< Offset 18:
    UINT8                   reserved1;
    UINT8                   aNmtCommandData[32];
} PACK_STRUCT tNmtCommandService;

typedef struct
{
    UINT16                  reserved;                       ///< Offset 18:
    UINT32                  syncStatusLe;
    UINT32                  latencyLe;
    UINT32                  syncNodeNumberLe;
    UINT32                  syncDelayLe;
    UINT32                  presTimeFirstLe;
    UINT32                  presTimeSecondLe;
} PACK_STRUCT tSyncResponse;

typedef struct
{
    UINT8                   reserved1;
    UINT8                   transactionId;
    UINT8                   flags;
    UINT8                   commandId;
    UINT16                  segmentSizeLe;
    UINT16                  reserved2;
    UINT8                   aCommandData[8];                // just reserve a minimum number of bytes as a placeholder
}PACK_STRUCT tAsySdoCom;


// asynchronous SDO Sequence Header
typedef struct
{
    UINT8                   recvSeqNumCon;
    UINT8                   sendSeqNumCon;
    UINT8                   aReserved[2];
    tAsySdoCom              sdoSeqPayload;
} PACK_STRUCT tAsySdoSeq;

typedef struct
{
    // Offset 18
    UINT8                   nmtCommandId;
    UINT8                   targetNodeId;
    UINT8                   aNmtCommandData[32];
} PACK_STRUCT tNmtRequestService;


typedef union
{
    tStatusResponse         statusResponse;                 ///< Offset 18:
    tIdentResponse          identResponse;
    tNmtCommandService      nmtCommandService;
    tNmtRequestService      nmtRequestService;
    tAsySdoSeq              sdoSequenceFrame;
    tSyncResponse           syncResponse;
    UINT8                   aPayload[256];
} tAsndPayload;

typedef struct
{
    UINT8                   serviceId;                      ///< Offset 17
    tAsndPayload            payload;                        ///< Offset 18
} PACK_STRUCT tAsndFrame;

typedef union
{
    tSocFrame               soc;                            ///< Offset 17
    tPreqFrame              preq;
    tPresFrame              pres;
    tSoaFrame               soa;
    tAsndFrame              asnd;
} tFrameData;

typedef struct
{
    UINT8                   aDstMac[6];                     ///< Offset 0: MAC address of the addressed nodes
    UINT8                   aSrcMac[6];                     ///< Offset 6: MAC address of the transmitting node
    UINT16                  etherType;                      ///< Offset 12: Ethernet message type (big endian)
    UINT8                   messageType;                    ///< Offset 14: POWERLINK message type
    UINT8                   dstNodeId;                      ///< Offset 15: POWERLINK node ID of the addressed nodes
    UINT8                   srcNodeId;                      ///< Offset 16: POWERLINK node ID of the transmitting node
    tFrameData              data;                           ///< Offset 17:
} PACK_STRUCT tPlkFrame;

// reset byte-align of structures
#ifdef _MSC_VER
#pragma pack(pop, packing)
#endif


typedef enum
{
    kMsgTypeNonPowerlink        = 0x00,
    kMsgTypeSoc                 = 0x01,
    kMsgTypePreq                = 0x03,
    kMsgTypePres                = 0x04,
    kMsgTypeSoa                 = 0x05,
    kMsgTypeAsnd                = 0x06,
    kMsgTypeAInv                = 0x0D,
} tMsgType;

#endif /* _INC_oplk_frame_H_ */

