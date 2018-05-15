/**
********************************************************************************
\file   oplk/frame.h

\brief  Definitions for POWERLINK frames

This header file contains definitions describing POWERLINK frames.
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2016, B&R Industrial Automation GmbH
Copyright (c) 2015, SYSTEC electronic GmbH
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
#define PLK_FRAME_FLAG1_ER          0x02    // exception reset (error signaling)        (SoA)
#define PLK_FRAME_FLAG1_EA          0x04    // exception acknowledge (error signaling)  (PReq, SoA)
#define PLK_FRAME_FLAG1_EC          0x08    // exception clear (error signaling)        (StatusRes)
#define PLK_FRAME_FLAG1_EN          0x10    // exception new (error signaling)          (PRes, StatusRes)
#define PLK_FRAME_FLAG1_MS          0x20    // multiplexed slot                         (PReq)
#define PLK_FRAME_FLAG1_PS          0x40    // prescaled slot                           (SoC)
#define PLK_FRAME_FLAG1_MC          0x80    // multiplexed cycle completed              (SoC)
#define PLK_FRAME_FLAG2_RS          0x07    // number of pending requests to send       (PRes, StatusRes, IdentRes)
#define PLK_FRAME_FLAG2_PR          0x38    // priority of requested asynch. frame      (PRes, StatusRes, IdentRes)
#define PLK_FRAME_FLAG2_PR_SHIFT    3       // shift of priority of requested asynch. frame
#define PLK_FRAME_FLAG3_MR          0x01    // MN redundancy active                     (SoA)

// error history/status entry types
#define ERR_ENTRYTYPE_STATUS        0x8000
#define ERR_ENTRYTYPE_HISTORY       0x0000
#define ERR_ENTRYTYPE_EMCY          0x4000
#define ERR_ENTRYTYPE_MODE_ACTIVE   0x1000
#define ERR_ENTRYTYPE_MODE_CLEARED  0x2000
#define ERR_ENTRYTYPE_MODE_OCCURRED 0x3000
#define ERR_ENTRYTYPE_MODE_MASK     0x3000
#define ERR_ENTRYTYPE_PROF_VENDOR   0x0001
#define ERR_ENTRYTYPE_PROF_PLK      0x0002
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
#define PLK_FRAME_OFFSET_SDO_SEQU       18
#define PLK_FRAME_OFFSET_SDO_COMU       22
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
#define SDO_CMDL_HDR_WRITEMULTBYINDEX_SIZE  8       // size of write multiple parameters by index sub-header (index + subindex + reserved)

// defines for SDO command layer flags
#define SDO_CMDL_FLAG_RESPONSE       0x80
#define SDO_CMDL_FLAG_ABORT          0x40
#define SDO_CMDL_FLAG_EXPEDITED      0x00
#define SDO_CMDL_FLAG_SEGMINIT       0x10
#define SDO_CMDL_FLAG_SEGMENTED      0x20
#define SDO_CMDL_FLAG_SEGMCOMPL      0x30
#define SDO_CMDL_FLAG_SEGM_MASK      0x30
#define SDO_CMDL_FLAG_PADSIZE_MASK   0x03

// defines for NMT command data of NMTGoToStandby
#define NMT_CMD_DATA_FLAG_DELAY             0x01    // include MNSwitchOverDelay_U32

//------------------------------------------------------------------------------
// typedef
//------------------------------------------------------------------------------

/**
* \brief Message type
*
* This enumeration defines the POWERLINK message type IDs.
*
* For more information consult the POWERLINK specification document "EPSG DS 301 V1.2.0" on page 349.
*/
typedef enum
{
    kMsgTypeNonPowerlink        = 0x00,                     ///< Defines non POWERLINK Frame
    kMsgTypeSoc                 = 0x01,                     ///< Defines Start of Cycle Frame
    kMsgTypePreq                = 0x03,                     ///< Defines Poll Request Frame
    kMsgTypePres                = 0x04,                     ///< Defines Poll Response Frame
    kMsgTypeSoa                 = 0x05,                     ///< Defines Start of Asynchronous Cycle Frame
    kMsgTypeAsnd                = 0x06,                     ///< Defines Asynchronous Send Frame
    kMsgTypeAmni                = 0x07,                     ///< Defines Active Managing Node Indication Frame
    kMsgTypeAInv                = 0x0D,                     ///< Defines Asynchronous Invite Frame
} eMsgType;

/**
\brief Message type data type

Data type for the enumerator \ref eMsgType.
*/
typedef UINT8 tMsgType;

// byte-align structures
#ifdef _MSC_VER
#pragma pack(push, packing)
#pragma pack(1)
#define PACK_STRUCT
#elif defined(__GNUC__)
#define PACK_STRUCT    __attribute__((packed))
#else /* defined(__GNUC__) */
#error You must Byte-align these structures with the appropriate compiler directives
#endif /* defined(__GNUC__) */

/**
* \brief Start of cycle Frame (SoC)
*
* This structure contains the layout of an SoC frame. At the beginning of a POWERLINK cycle, the MN sends an SoC frame to all nodes via Ethernet
* multicast. The send and receive time of this frame shall be the basis for the common timing of all the nodes.
*
* For more information consult the POWERLINK specification document "EPSG DS 301 V1.2.0" chapter 4.2.4.1.1 and 4.6.1.1.2.
* */
typedef struct
{
    UINT8                   reserved1;                      ///< Reserved (Offset 17)
    UINT8                   flag1;                          ///< Contains the flag Multiplexed Cycle Completed (MC) and the flag Prescaled Slot (PS). (Offset 18)
    UINT8                   flag2;                          ///< Reserved (Offset 19)
    tNetTime                netTimeLe;                      ///< Optional, if D_NMT_NetTimeIsRealTime_BOOL is set. MN may distribute the starting time of the POWERLINK cycle. (Offset 20)
    UINT64                  relativeTimeLe;                 ///< Optional, if D_NMT_RelativeTime_BOOL is set. The relative time (in us) is incremented in every cycle by the cycle time. It shall be set to 0 when NMT state equals NMT_GS_INITIALISING. (Offset 28)
} PACK_STRUCT tSocFrame;

/**
* \brief Poll Request Frame (PReq)
*
* This structure defines the PReq frame. It is transmitted cyclically by the MN to a CN via Ethernet unicast.
*
* For more information consult the POWERLINK specification document "EPSG DS 301 V1.2.0" chapter 4.6.1.1.3.
*/
typedef struct
{
    UINT8                   reserved1;                      ///< Reserved (Offset 17)
    UINT8                   flag1;                          ///< Contains the flags Multiplexed Slot (MS), Exception Acknowledge (EA) and Ready (RD). (Offset 18)
    UINT8                   flag2;                          ///< Reserved (Offset 19)
    UINT8                   pdoVersion;                     ///< Indicates the PDO Version (Offset 20)
    UINT8                   reserved2;                      ///< Reserved (Offset 21)
    UINT16                  sizeLe;                         ///< Contains the number of payload data octets (Offset 22)
    UINT8                   aPayload[256];                  ///< Payload (Offset 24)
} PACK_STRUCT tPreqFrame;

/**
* \brief Poll Response Frame (PRes)
*
* This structure defines the PRes frame. It is transmitted cyclically via Ethernet multicast.
*
* For more information consult the POWERLINK specification document "EPSG DS 301 V1.2.0" chapter 4.6.1.1.4.
*/
typedef struct
{
    UINT8                   nmtStatus;                      ///< Defines the NMT state. (Offset 17)
    UINT8                   flag1;                          ///< Contains the flags multiplexed Slot (MS), Exception New (EN) and Ready (RD). (Offset 18)
    UINT8                   flag2;                          ///< Contains the flags Priority (PR) and Request to send (RS). (Offset 19)
    UINT8                   pdoVersion;                     ///< Indicates the PDO Version. (Offset 20)
    UINT8                   reserved2;                      ///< Reserved (Offset 21)
    UINT16                  sizeLe;                         ///< Contains the number of payload data octets. (Offset 22)
    UINT8                   aPayload[256];                  ///< Payload (Offset 24)
} PACK_STRUCT tPresFrame;

/**
* \brief Synchronization Request (SyncReq)
*
* The SyncReq is a special form of an SoA frame used in PollResponse Chaining mode.
*
* For detailed information, refer to the POWERLINK specification addendum EPSG DS 302-C V-1-0-0 chapter 3.3.
*/
typedef struct
{
    UINT32                  syncControlLe;                  ///< Sync Control (Offset 24)
                                                            //*< Contains the flags PResTimeFirstValid, PResTimeSecondValid, SyncMNDelayFirstValid, SyncMNDelaySecondValid, PResFallBackTimeoutValid, DestMacAddressValid, PResModeReset and PResModeSet.*/
    UINT32                  presTimeFirstLe;                ///< Contains the PRes Response Time [ns] starting from the end of the PResMN.
    UINT32                  presTimeSecondLe;               ///< In case of ring redundancy this parameter contains the PRes Response Time [ns] for the secondary direction of communication starting from the end of the PResMN. Otherwise this parameter shall be ignored.
    UINT32                  syncMnDelayFirstLe;             ///< Contains the propagation delay [ns] between the Managing Node and the Controlled Node.
    UINT32                  syncMnDelaySecondLe;            ///< In case of ring redundancy this parameter contains the propagation delay [ns] between the Managing Node and the Controlled Node for the secondary direction of communication.
    UINT32                  presFallBackTimeoutLe;          ///< In NMT_CN_PRE_OPERATIONAL_2 the node is not able to monitor the cycle time. The actual cycle time might not be configured yet.
    UINT8                   aDestMacAddress[6];             ///< Holds the MAC address of the node the SyncReq is sent to.
} PACK_STRUCT tSyncRequest;

/**
* \brief Start of asynchronous Payload
*
* This union contains the SoA payload.
*
* It currently contains only the structure syncRequest.
*/
typedef union
{
    tSyncRequest            syncRequest;                    ///< Used in PollResponse Chaining mode. (Offset 24)
} tSoaPayload;

/**
* \brief Start of asynchronous frame (SoA)
*
* This structure defines the SoA frame structure. In the asynchronous phase of the cycle, access to the POWERLINK network may be granted to any node for the
* transfer of an asynchronous message. There shall be two types of asynchronous frames available:
* - The POWERLINK ASnd frame shall use the POWERLINK addressing scheme and shall be sent via unicast, multicast or broadcast to any other node.
* - A Legacy Ethernet message may be sent.
*
* For more information consult the POWERLINK specification document "EPSG DS 301 V1.2.0" chapter 4.6.1.1.5 and 4.2.4.1.2.
*/
typedef struct
{
    UINT8                   nmtStatus;                      ///< Reports the current MN NMT status (Offset 17)
    UINT8                   flag1;                          ///< Contains the flags Exception Acknowledgment (EA) and Exception Reset (ER). (Offset 18)
    UINT8                   flag2;                          ///< Reserved (Offset 19)
    UINT8                   reqServiceId;                   ///< Indicates the asynchronous service ID dedicated to the SoA and to the following asynchronous slot (refer below). (Offset 20)
    UINT8                   reqServiceTarget;               ///< Indicates the POWERLINK address of the node, which is allowed to send. (Offset 21)
    UINT8                   powerlinkVersion;               ///< Indicates the current POWERLINK Version of the MN (Offset 22)
    UINT8                   flag3;                          ///< Indicates if MN redundancy is active (Offset 23)
    tSoaPayload             payload;                        ///< SoA Payload (Offset 24)
} PACK_STRUCT tSoaFrame;

/**
* \brief Error History
*
* The following structure defines an error history entry.
*/
typedef struct
{
    UINT16                  entryType;                      ///< Contains the type of the entry
    UINT16                  errorCode;                      ///< Contains the error code of the entry
    tNetTime                timeStamp;                      ///< Contains the timestamp when the error was added
    UINT8                   aAddInfo[8];                    ///< Includes the additional error information
} PACK_STRUCT tErrHistoryEntry;

/**
* \brief Status Response
*
* The following structure defines the StatusResponse.
* This service is used by the MN to query the current status of a CN.
*
* For more information consult the POWERLINK specification document "EPSG DS 301 V1.2.0" chapter 7.3.3.3.1.
*/
typedef struct
{
    UINT8                   flag1;                          ///< Contains the flags Exception New (EN) and Exception Clear (EC). (Offset 18)
    UINT8                   flag2;                          ///< Contains the flags PR (priority of the requested asynchronous frame) and RS (number of pending requests to send at the CN). (Offset 19)
    UINT8                   nmtStatus;                      ///< Reports the current status of the CN’s NMT state machine. (Offset 20)
    UINT8                   reserved1[3];                   ///< Reserved
    UINT64                  staticErrorLe;                  ///< Includes specific bits, which are set to indicate pending errors at the CN.
    tErrHistoryEntry        aErrorHistoryEntry[13];         ///< Contains a list of errors, that have occurred at the CN.
} PACK_STRUCT tStatusResponse;

/**
* \brief Ident Response
*
* The IdentResponse is sent by a CN in reply to an IdentRequest. It is used to inform the network about
* the identity and features of a CN.
*
* For more information consult the POWERLINK specification document "EPSG DS 301 V1.2.0" chapter 7.3.3.2.1.
*/
typedef struct
{
    UINT8                   flag1;                          ///< Reserved (Offset 18)
    UINT8                   flag2;                          ///< Contains the flags PR (priority of the requested asynchronous frame) and RS (number of pending requests to send at the CN).
    UINT8                   nmtStatus;                      ///< Reports the current status of the CN’s NMT state machine
    UINT8                   identResponseFlags;             ///< Ident Response Flags
    UINT8                   powerlinkProfileVersion;        ///< Indicates the POWERLINK Version to which the CN conforms
    UINT8                   reserved1;                      ///< Reserved
    UINT32                  featureFlagsLe;                 ///< Reports the device’s feature flags (NMT_FeatureFlags_U32)
    UINT16                  mtuLe;                          ///< Reports the maximum size of an asynchronous frame that can be handled by the CN (without Ethernet header and trailer).
    UINT16                  pollInSizeLe;                   ///< Reports the current CN setting.
    UINT16                  pollOutSizeLe;                  ///< Reports the current CN setting.
    UINT32                  responseTimeLe;                 ///< Reports the maximum time required by the CN to respond to a PReq with a PRes.
    UINT16                  reserved2;                      ///< Reserved
    UINT32                  deviceTypeLe;                   ///< Reports the CN’s Device type.
    UINT32                  vendorIdLe;                     ///< Reports the CN’s Vendor ID.
    UINT32                  productCodeLe;                  ///< Reports the CN’s Product Code
    UINT32                  revisionNumberLe;               ///< Reports the CN’s Revision Number.
    UINT32                  serialNumberLe;                 ///< Reports the CN’s Serial Number.
    UINT64                  vendorSpecificExt1Le;           ///< May be used for vendor specific purpose, to be filled with zeros if not in use.
    UINT32                  verifyConfigurationDateLe;      ///< Reports the CN’s Configuration date.
    UINT32                  verifyConfigurationTimeLe;      ///< Reports the CN’s Configuration time.
    UINT32                  applicationSwDateLe;            ///< Reports the CN’s Application SW date.
    UINT32                  applicationSwTimeLe;            ///< Reports the CN’s Application SW time.
    UINT32                  ipAddressLe;                    ///< Reports the current IP address value of the CN.
    UINT32                  subnetMaskLe;                   ///< Reports the current IP subnet mask value of the CN.
    UINT32                  defaultGatewayLe;               ///< Reports the current IP default gateway value of the CN.
    UINT8                   sHostName[32];                  ///< Reports the current DNS hostname of the CN.
    UINT8                   aVendorSpecificExt2[48];        ///< May be used for vendor specific purpose.
} PACK_STRUCT tIdentResponse;

/**
* \brief Command Service
*
* The MN uses NMT State Command Services to control the CN state machine(s).
*
* For more information consult the POWERLINK specification document "EPSG DS 301 V1.2.0" chapter 7.3.4.
*/
typedef struct
{
    UINT8                   nmtCommandId;                   ///< Qualifies the NMT state command. (Offset 18)
    UINT8                   reserved1;                      ///< Reserved
    UINT8                   aNmtCommandData[32];            ///< NMT command-specific data to be issued by the MN.
} PACK_STRUCT tNmtCommandService;

/**
* \brief Synchronization Response (SyncRes)
*
* The SyncRes is a special form of an SoA frame used in PollResponse Chaining mode.
*
* For detailed information, refer to the POWERLINK specification addendum EPSG DS 302-C V-1-0-0 chapter 3.4.
*/
typedef struct
{
    UINT16                  reserved;                       ///< Reserved (Offset 18)
    UINT32                  syncStatusLe;                   ///< Contains the flags PResTimeFirstValid, PResTimeSecondValid and PResModeStatus.
    UINT32                  latencyLe;                      ///< Contains the PollResponse latency in [ns]. The value is constant.
    UINT32                  syncNodeNumberLe;               ///< Contains the node number received last inside the SyncReq/SyncRes frames.
    UINT32                  syncDelayLe;                    ///< Contains the time difference between the end of receiving the SyncReq and the beginning of receiving the SyncRes in [ns].
    UINT32                  presTimeFirstLe;                ///< Holds the current value of the PRes Response Time [ns] i.e. PResTimeFirst_U32 for the first direction of communication.
    UINT32                  presTimeSecondLe;               ///< In case of ring redundancy this parameter holds the current value of the PRes Response Time [ns] i.e. PResTimeSecond_U32 for the secondary direction of communication.
} PACK_STRUCT tSyncResponse;

/**
 * \brief SDO Command Layer Protocol
 *
 * This structure defines the fixed part of the POWERLINK SDO Command Layer protocol.
 *
 * For more information consult the POWERLINK specification document "EPSG DS 301 V1.2.0" chapter 6.3.2.4.1.
 */
typedef struct
{
    UINT8                   reserved1;                      ///< Reserved
    UINT8                   transactionId;                  ///< Contains unambiguous transaction ID for a command. Changed by the client with every new command.
    UINT8                   flags;                          ///< Contains the flags Request, Response (rsp), requested transfer (a) and the differentiates between expedited and segmented transfer (seg)
    UINT8                   commandId;                      ///< Specifies the command (cid)
    UINT16                  segmentSizeLe;                  ///< Segment size (ss)
    UINT16                  reserved2;                      ///< Reserved
    UINT8                   aCommandData[8];                ///< Reserves a minimum number of bytes as a placeholder
} PACK_STRUCT tAsySdoCom;

/**
 * \brief SDO Command Layer Write Multiple Parameter by Index Request or Read Multiple by Index response sub-header
 *
 * This structure defines the sub-header of a WriteMultParam request or ReadMultParam response command
 *
 * For more information consult the POWERLINK specification document "EPSG DS 301 V1.3.0" chapter 6.3.2.4.2.3
 */
typedef struct
{
    UINT32                  byteOffsetNext;                 ///< Byte Offset of the next data set, counting from the beginning of the fixed command header. If the value is 0, the last data set has been reached.
    UINT16                  index;                          ///< Specifies an entry of the device object dictionary
    UINT8                   subIndex;                       ///< Specifies a component of a device object dictionary entry
    UINT8                   info;                           ///< Reserved (alignment-padding) and 2 LSB bits as info for padding bytes after payload data. For mulit-read the MSB is additionally a sub-abort flag.
    UINT8                   aCommandData[4];                ///< Payload data or sub-abort code
} PACK_STRUCT tAsySdoComMultWriteReqReadResp;

/**
 * \brief SDO Command Layer Write Multiple Parameter by Index Response sub-header
 *
 * This structure defines the sub-header of a WriteMultParam command response
 *
 * For more information consult the POWERLINK specification document "EPSG DS 301 V1.3.0" chapter 6.3.2.4.2.3
 */
typedef struct
{
    UINT16                  index;                          ///< Specifies an entry of the device object dictionary
    UINT8                   subIndex;                       ///< Specifies a component of a device object dictionary entry
    UINT8                   abortFlag;                      ///< Flag is 1-bit MSB; 0: transfer ok 1: abort transfer
    UINT32                  subAbortCode;                   ///< Reason of the sub-abort
} PACK_STRUCT tAsySdoComWriteMultResp;

/**
 * \brief SDO Command Layer Read Multiple Parameter by Index request sub-header
 *
 * This structure defines the sub-header of a ReadMultParam command request
 *
 * For more information consult the POWERLINK specification document "EPSG DS 301 V1.3.0" chapter 6.3.2.4.2.3
 */
typedef struct
{
    UINT16                  index;                          ///< Specifies an entry of the device object dictionary
    UINT8                   subIndex;                       ///< Specifies a component of a device object dictionary entry
    UINT8                   reserved;                       ///< alignment for next sub-header
} PACK_STRUCT tAsySdoComReadMultReq;

/** \brief Asynchronous SDO Sequence Header
*
* The POWERLINK SDO Sequence Layer provides the service of a reliable bidirectional connection that guarantees that no messages are lost or duplicated and
* that all messages arrive in the correct order.
*
* For more information consult the POWERLINK specification document "EPSG DS 301 V1.2.0" chapter 6.3.2.3.
*/
typedef struct
{
    UINT8                   recvSeqNumCon;                  ///< Contains the sequence number of the last correctly received frame
    UINT8                   sendSeqNumCon;                  ///< Contains own sequence number of the frame, shall be increased by 1 with every new frame
    UINT8                   aReserved[2];                   ///< Reserved
    tAsySdoCom              sdoSeqPayload;                  ///< SDO Payload Data
} PACK_STRUCT tAsySdoSeq;

/**
* \brief Network management Request Service
*
* The Network management Request Service is issued by a CN that received a NMTRequestInvite via SoA.
*/
typedef struct
{
    UINT8                   nmtCommandId;                   ///< Qualifies the NMT Managing Command (Offset 18)
    UINT8                   targetNodeId;                   ///< Includes the target node ID
    UINT8                   aNmtCommandData[32];            ///< Contains managing command-specific data to be issued by the MN.
} PACK_STRUCT tNmtRequestService;

/**
* \brief ServiceID Values
*
* This union contains the ServiceID Values in the Asnd frame.
*
* For more information consult the POWERLINK specification document "EPSG DS 301 V1.2.0" chapter 4.6.1.1.6.1.
*/
typedef union
{
    tStatusResponse         statusResponse;                 ///< Issued by a node that received a StatusRequest via SoA (Offset 18)
    tIdentResponse          identResponse;                  ///< Issued by a node that received an IdentRequest via SoA
    tNmtCommandService      nmtCommandService;              ///< Issued by the MN upon an internal request or upon an external request via NMTRequest.
    tNmtRequestService      nmtRequestService;              ///< Issued by a CN that received a NMTRequestInvite via SoA
    tAsySdoSeq              sdoSequenceFrame;               ///< Issued by a CN that received an UnspecifiedInvite via SoA to indicate SDO transmission via ASnd.
    tSyncResponse           syncResponse;                   ///< Received by all nodes supporting PResChaining. The SyncRes is an ASnd frame.
    UINT8                   aPayload[256];
} tAsndPayload;

/**
* \brief Asynchronous Send Frame
*
* This structure defines the Asynchronous Send Frame (ASnd) frame structure.
*
* For more information consult the POWERLINK specification document "EPSG DS 301 V1.2.0" chapter 4.6.1.1.6.
*/
typedef struct
{
    UINT8                   serviceId;                      ///< Indicates the service ID dedicated to the asynchronous slot (Offset 17)
    tAsndPayload            payload;                        ///< Contains data, that are specific for the current ServiceID (Offset 18)
} PACK_STRUCT tAsndFrame;

/**
* \brief Frame Data
*
* This union contains the various POWERLINK messages types.
*
* For more information consult the POWERLINK specification document "EPSG DS 301 V1.2.0" chapter 4.6.1.1.1.
*/
typedef union
{
    tSocFrame               soc;                            ///< Contains the Soc frame structure (Multicast)
    tPreqFrame              preq;                           ///< Contains the Poll request frame structure (Unicast)
    tPresFrame              pres;                           ///< Contains the Poll response frame structure (Multicast)
    tSoaFrame               soa;                            ///< Contains the Start of asynchronous frame structure (Multicast)
    tAsndFrame              asnd;                           ///< Contains the Asynchronous send frame structure (Multicast)
} tFrameData;

/**
* \brief POWERLINK Frame
*
* This structure contains the POWERLINK Basic Frame Format. The POWERLINK Basic Frame format shall be encapsulated by the Ethernet wrapper consisting of
* 14 octets of leading Ethernet header (Destination and Source MAC addresses, EtherType) and 4 octets of terminating CRC32 checksum.
*
* For more information consult the POWERLINK specification document "EPSG DS 301 V1.2.0" chapter 4.6.1.1.
*/
typedef struct
{
    UINT8                   aDstMac[6];                     ///< Contains the MAC address of the addressed nodes (Offset 0)
    UINT8                   aSrcMac[6];                     ///< Contains the MAC address of the transmitting node (Offset 6)
    UINT16                  etherType;                      ///< Contains the Ethernet message type (big endian) (Offset 12)
    tMsgType                messageType;                    ///< Contains the POWERLINK message type (Offset 14)
    UINT8                   dstNodeId;                      ///< Contains the POWERLINK node ID of the addressed nodes (Offset 15)
    UINT8                   srcNodeId;                      ///< Contains the POWERLINK node ID of the transmitting node (Offset 16)
    tFrameData              data;                           ///< Contains the Frame Data (Offset 17)
} PACK_STRUCT tPlkFrame;

// reset byte-align of structures
#ifdef _MSC_VER
#pragma pack(pop, packing)
#endif /* _MSC_VER */

#endif /* _INC_oplk_frame_H_ */
