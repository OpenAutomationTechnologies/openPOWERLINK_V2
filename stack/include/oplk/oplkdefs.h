/**
********************************************************************************
\file   oplk/oplkdefs.h

\brief  Default definitions and configuration for openPOWERLINK

This file contains a default definitions and configuration macros for the
openPOWERLINK stack.
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
#ifndef _INC_oplk_oplkdefs_H_
#define _INC_oplk_oplkdefs_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
/// \name Version macros
/// \{
#define PLK_SPEC_VERSION                                0x20                                                    ///< Ethernet POWERLINK V 2.0
#define PLK_STACK_VERSION(ver, rev, rel, rc)            (((((UINT32)(ver)) & 0xFF) << 24) | ((((UINT32)(rev)) & 0xFF) << 16) | ((((UINT32)(rel)) & 0xFF) << 8) | ((((UINT32)(rc)) & 0xFF) << 0))      ///< Version in UINT32 format
#define PLK_OBJ1018_VERSION(ver, rev, rel, rc)          ((((UINT32)(ver))<<16) | (((UINT32)(rev)) & 0xFFFF))    ///< Version in UINT32 format for Object 0x1018 which contains general information about a device.
#define PLK_STRING_VERSION(ver, rev, rel, rc)           "V" #ver "." #rev "." #rel                              ///< Version in string format
#define PLK_STRING_VERSION_RC(ver, rev, rel, rc)        "V" #ver "." #rev "." #rel "-rc" #rc                    ///< Release candidate version in string format
#define PLK_STACK_VER(ver)                              ((UINT32)ver & 0xFF000000) >> 24                        ///< Parses the 32 bit version number and returns the major number.
#define PLK_STACK_REF(ver)                              ((UINT32)ver & 0x00FF0000) >> 16                        ///< Parses the 32 bit version number and returns the minor number.
#define PLK_STACK_REL(ver)                              ((UINT32)ver & 0x0000FF00) >> 8                         ///< Parses the 32 bit version number and returns the build number.
#define PLK_STACK_RC(ver)                               ((UINT32)ver & 0x000000FF) >> 0                         ///< Parses the 32 bit version number and returns the rc number.
/// \}

//------------------------------------------------------------------------------
/// \name Kernel feature flags
/// \{
// These flags are used to determine how the kernel part of the stack is
// configured. It is needed to ensure that user and kernel part are
// working together in a multi processor system.
#define OPLK_KERNEL_MN                                  0x00000001      ///< NMT functionality compiled in
#define OPLK_KERNEL_ISOCHR                              0x00000002      ///< PDO module for isochronous transfers
#define OPLK_KERNEL_PRES_CHAINING_CN                    0x00000004      ///< Feature Poll Response Chaining
#define OPLK_KERNEL_VETH                                0x00000008      ///< Virtual Ethernet module
#define OPLK_KERNEL_PRES_FORWARD                        0x00000010      ///< PRES forwarding module (used for diagnosis)
#define OPLK_KERNEL_RMN                                 0x00000020      ///< Redundancy MN (RMN)
#define OPLK_KERNEL_SOC_TIME_FORWARD                    0x00000040      ///< SoC time forwarding capability

/// \}

//------------------------------------------------------------------------------
/// \name Default POWERLINK constants
/// \{
#define C_ADR_BROADCAST                                 0xFF                ///< POWERLINK broadcast address
#define C_ADR_DIAG_DEF_NODE_ID                          0xFD                ///< POWERLINK default address of diagnostic device
#define C_ADR_DUMMY_NODE_ID                             0xFC                ///< POWERLINK dummy node address
#define C_ADR_INVALID                                   0x00                ///< Invalid POWERLINK address
#define C_ADR_MN_DEF_NODE_ID                            0xF0                ///< POWERLINK default address of MN
#define C_ADR_RT1_DEF_NODE_ID                           0xFE                ///< POWERLINK default address of router type 1

#define C_DLL_ASND_PRIO_NMTRQST                         7                   ///< Increased ASnd request priority to be used by NMT Requests
#define C_DLL_ASND_PRIO_STD                             0                   ///< Standard ASnd request priority
#define C_DLL_ETHERTYPE_EPL                             0x88AB              ///< POWERLINK ethertype
#define C_DLL_ISOCHR_MAX_PAYL                           1490                ///< Maximum size of PReq and PRes payload data
#define C_DLL_MAX_ASYNC_MTU                             1500                ///< Maximum asynchronous payload in bytes
#define C_DLL_MAX_ETH_FRAME                             1514                ///< Maximum Ethernet frame in bytes without checksum
#define C_DLL_MIN_ETH_FRAME                             60                  ///< Minimum Ethernet frame in bytes without checksum
#define C_DLL_MAX_PAYL_OFFSET                           1499                ///< Maximum offset of Ethernet frame payload
#define C_DLL_MAX_RS                                    7                   ///< Maximum number of pending requests
#define C_DLL_MIN_ASYNC_MTU                             300                 ///< Minimum asynchronous payload in bytes.
#define C_DLL_MIN_PAYL_OFFSET                           45                  ///< Minimum offset of Ethernet frame payload
#define C_DLL_MULTICAST_AMNI                            0x01111E000005LL    ///< POWERLINK Active managing node indication multicast MAC address, canonical form
#define C_DLL_MULTICAST_ASND                            0x01111E000004LL    ///< POWERLINK ASnd multicast MAC address, canonical form
#define C_DLL_MULTICAST_PRES                            0x01111E000002LL    ///< POWERLINK PRes multicast MAC address, canonical form
#define C_DLL_MULTICAST_SOA                             0x01111E000003LL    ///< POWERLINK SoA multicast MAC address, canonical form
#define C_DLL_MULTICAST_SOC                             0x01111E000001LL    ///< POWERLINK Soc multicast MAC address, canonical form
#define C_DLL_PREOP1_START_CYCLES                       10                  ///< Number of unassigning SoA frames at start of NMT_MS_PRE_OPERATIONAL_1
#define C_DLL_T_BITTIME                                 10                  ///< Transmission time per bit on 100 Mbit/s network [ns]
#define C_DLL_T_EPL_PDO_HEADER                          10                  ///< Size of PReq and PRes POWERLINK PDO message header [Byte]
#define C_DLL_T_ETH2_WRAPPER                            18                  ///< Size of Ethernet type II wrapper consisting of header and checksum [Byte]
#define C_DLL_T_IFG                                     960                 ///< Ethernet inter frame gap [ns]
#define C_DLL_T_MIN_FRAME                               5120                ///< Size of minimum Ethernet frame (without preamble) [ns]
#define C_DLL_T_PREAMBLE                                640                 ///< Size of Ethernet frame preamble [ns]

#define C_ERR_MONITOR_DELAY                             10                  ///< Error monitoring start delay (not used in DS 1.0.0)

#define C_IP_ADR_INVALID                                0x00000000L         ///< Invalid IP address (0.0.0.0) used to indicate no change
#define C_IP_INVALID_MTU                                0                   ///< Invalid MTU size used to indicate no change [Byte]

#define C_NMT_STATE_TOLERANCE                           5                   ///< Maximum reaction time to NMT state commands [cycles]
#define C_NMT_STATREQ_CYCLE                             5                   ///< StatusRequest cycle time to be applied to AsyncOnly CNs [sec]

#define C_SDO_EPL_PORT                                  3819                ///< Port to be used for POWERLINK specific UDP/IP frames

#define C_ADR_SYNC_ON_SOC                               0x00                ///< Synchronization on SoC
#define C_ADR_SYNC_ON_SOA                               0xFF                ///< Synchronization on SoA

#define C_DLL_MINSIZE_SOC                               36                  ///< Minimum size of SoC without padding and CRC
#define C_DLL_MINSIZE_PREQ                              60                  ///< Minimum size of PRec without CRC
#define C_DLL_MINSIZE_PRES                              60                  ///< Minimum size of PRes without CRC
#define C_DLL_MINSIZE_SOA                               54                  ///< Minimum size of SoA without padding and CRC
#define C_DLL_MINSIZE_IDENTRES                          176                 ///< Minimum size of IdentResponse without CRC
#define C_DLL_MINSIZE_STATUSRES                         72                  ///< Minimum size of StatusResponse without CRC
#define C_DLL_MINSIZE_SYNCRES                           44                  ///< Minimum size of SyncResponse without padding and CRC
#define C_DLL_MINSIZE_NMTCMD                            20                  ///< Minimum size of NmtCommand without CommandData, padding and CRC
#define C_DLL_MINSIZE_NMTCMDEXT                         52                  ///< Minimum size of NmtCommand without padding and CRC
#define C_DLL_MINSIZE_NMTREQ                            20                  ///< Minimum size of NmtRequest without CommandData, padding and CRC
#define C_DLL_MINSIZE_NMTREQEXT                         52                  ///< Minimum size of NmtRequest without padding and CRC
#define C_DLL_MINSIZE_AMNI                              20                  ///< Minimum size of AMNI

#define C_DLL_MACADDR_MASK                              0xFFFFFFFFFFFFLL    ///< MAC address mask, canonical form
/// \}

//------------------------------------------------------------------------------
/// \name Constants for CDC file
/// \{
#define CDC_OFFSET_INDEX                                0                   ///< Offset of index in CDC file entry
#define CDC_OFFSET_SUBINDEX                             2                   ///< Offset of sub-index in CDC file entry
#define CDC_OFFSET_SIZE                                 3                   ///< Offset of size field in CDC file entry
#define CDC_OFFSET_DATA                                 7                   ///< Offset of object data in CDC file entry
/// \}

//------------------------------------------------------------------------------
/// \name Constants for queue implementations
/** \{
These constants determine the different queue implementations.
*/
#define DIRECT_QUEUE                                    1                   ///< Using "direct" implementation (actually no queue)
#define IOCTL_QUEUE                                     4                   ///< Use Linux IOCTL calls for queue
#define CIRCBUF_QUEUE                                   5                   ///< Use circular buffer library for queue
/// \}

//------------------------------------------------------------------------------
// definitions for usage of circular buffer library

/// \name  Circular buffer instances used in openPOWERLINK
/// \{
#define CIRCBUF_USER_TO_KERNEL_QUEUE                    0                   ///< User-to-kernel event queue
#define CIRCBUF_KERNEL_TO_USER_QUEUE                    1                   ///< Kernel-to-user event queue
#define CIRCBUF_KERNEL_INTERNAL_QUEUE                   2                   ///< Kernel internal event queue
#define CIRCBUF_USER_INTERNAL_QUEUE                     3                   ///< User internal event queue
#define CIRCBUF_DLLCAL_TXGEN                            4                   ///< Queue for sending generic requests in the DLLCAL
#define CIRCBUF_DLLCAL_TXNMT                            5                   ///< Queue for sending NMT requests in the DLLCAL
#define CIRCBUF_DLLCAL_TXSYNC                           6                   ///< Queue for sending sync requests in the DLLCAL
#define CIRCBUF_DLLCAL_CN_REQ_NMT                       7                   ///< NMT request queue for MN asynchronous scheduler
#define CIRCBUF_DLLCAL_CN_REQ_GEN                       8                   ///< Generic request queue for MN asynchronous scheduler
#define CIRCBUF_DLLCAL_CN_REQ_IDENT                     9                   ///< Ident request queue for MN asynchronous scheduler
#define CIRCBUF_DLLCAL_CN_REQ_STATUS                    10                  ///< Status request queue for MN asynchronous scheduler
#define CIRCBUF_DLLCAL_TXVETH                           11                  ///< Queue for sending virtual Ethernet frames in the DLLCAL
/// \}

//------------------------------------------------------------------------------
// Defines for object 0x1F80 NMT_StartUp_U32
#define NMT_STARTUP_STARTALLNODES                       0x00000002L
#define NMT_STARTUP_NO_AUTOSTART                        0x00000004L
#define NMT_STARTUP_NO_STARTNODE                        0x00000008L
#define NMT_STARTUP_RESETALL_MAND_CN                    0x00000010L
#define NMT_STARTUP_STOPALL_MAND_CN                     0x00000040L
#define NMT_STARTUP_NO_AUTOPREOP2                       0x00000080L
#define NMT_STARTUP_NO_AUTOREADYTOOP                    0x00000100L
#define NMT_STARTUP_EXT_CNIDENTCHECK                    0x00000200L
#define NMT_STARTUP_SWVERSIONCHECK                      0x00000400L
#define NMT_STARTUP_CONFCHECK                           0x00000800L
#define NMT_STARTUP_NO_RETURN_PREOP1                    0x00001000L
#define NMT_STARTUP_BASICETHERNET                       0x00002000L
#define NMT_STARTUP_REDUNDANCY                          0x00004000L

//------------------------------------------------------------------------------
// Defines for object 0x1F81 NMT_NodeAssignment_AU32
#define NMT_NODEASSIGN_NODE_EXISTS                      0x00000001L
#define NMT_NODEASSIGN_NODE_IS_CN                       0x00000002L
#define NMT_NODEASSIGN_START_CN                         0x00000004L
#define NMT_NODEASSIGN_MANDATORY_CN                     0x00000008L
#define NMT_NODEASSIGN_KEEPALIVE                        0x00000010L
#define NMT_NODEASSIGN_SWVERSIONCHECK                   0x00000020L
#define NMT_NODEASSIGN_SWUPDATE                         0x00000040L
#define NMT_NODEASSIGN_ASYNCONLY_NODE                   0x00000100L
#define NMT_NODEASSIGN_MULTIPLEXED_CN                   0x00000200L
#define NMT_NODEASSIGN_RT1                              0x00000400L
#define NMT_NODEASSIGN_RT2                              0x00000800L
#define NMT_NODEASSIGN_MN_PRES                          0x00001000L
#define NMT_NODEASSIGN_PRES_CHAINING                    0x00004000L
#define NMT_NODEASSIGN_VALID                            0x80000000L

//------------------------------------------------------------------------------
// Defines for object 0x1F82 NMT_FeatureFlags_AU32
#define NMT_FEATUREFLAGS_ISOCHR                         0x00000001
#define NMT_FEATUREFLAGS_SDO_UDP                        0x00000002
#define NMT_FEATUREFLAGS_SDO_ASND                       0x00000004
#define NMT_FEATUREFLAGS_SDO_PDO                        0x00000008
#define NMT_FEATUREFLAGS_NMT_INFO                       0x00000010
#define NMT_FEATUREFLAGS_NMT_EXT                        0x00000020
#define NMT_FEATUREFLAGS_PDO_DYN                        0x00000040
#define NMT_FEATUREFLAGS_NMT_UDP                        0x00000080
#define NMT_FEATUREFLAGS_CFM                            0x00000100
#define NMT_FEATUREFLAGS_MUX_CN                         0x00000200
#define NMT_FEATUREFLAGS_NODEID_SW                      0x00000400
#define NMT_FEATUREFLAGS_BASIC_ETH_MN                   0x00000800
#define NMT_FEATUREFLAGS_RT1                            0x00001000
#define NMT_FEATUREFLAGS_RT2                            0x00002000
#define NMT_FEATUREFLAGS_SDO_RW_ALL                     0x00004000
#define NMT_FEATUREFLAGS_SDO_RW_MULTIPLE                0x00008000
#define NMT_FEATUREFLAGS_MASND                          0x00010000
#define NMT_FEATUREFLAGS_RR_MN                          0x00020000
#define NMT_FEATUREFLAGS_PRC                            0x00040000
#define NMT_FEATUREFLAGS_MULTI_PREQ_PRES                0x00080000
#define NMT_FEATUREFLAGS_DNA                            0x00100000
#define NMT_FEATUREFLAGS_MODULAR_DEVICE                 0x00200000

#endif /* _INC_oplk_oplkdefs_H_ */
