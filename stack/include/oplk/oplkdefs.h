/**
********************************************************************************
\file   oplkdefs.h

\brief  Default definitions and configuration for openPOWERLINK

This file contains a default definitions and configuration macros for the
openPOWERLINK stack.
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

#ifndef _INC_oplk_oplkdefs_H_
#define _INC_oplk_oplkdefs_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <oplk/oplkinc.h>

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// Version macros
#define EPL_SPEC_VERSION                    0x20    // Ethernet POWERLINK V 2.0
#define EPL_STACK_VERSION(ver,rev,rel)      (((((DWORD)(ver)) & 0xFF)<<24)|((((DWORD)(rev))&0xFF)<<16)|(((DWORD)(rel))&0xFFFF))
#define EPL_OBJ1018_VERSION(ver,rev,rel)    ((((DWORD)(ver))<<16) |(((DWORD)(rev))&0xFFFF))
#define EPL_STRING_VERSION(ver,rev,rel,bld) "V" #ver "." #rev "." #rel "-" #bld

//------------------------------------------------------------------------------
// Default POWERLINK constants
#define EPL_C_ADR_BROADCAST                 0xFF    // EPL broadcast address
#define EPL_C_ADR_DIAG_DEF_NODE_ID          0xFD    // EPL default address of diagnostic device
#define EPL_C_ADR_DUMMY_NODE_ID             0xFC    // EPL dummy node address
#define EPL_C_ADR_INVALID                   0x00    // invalid EPL address
#define EPL_C_ADR_MN_DEF_NODE_ID            0xF0    // EPL default address of MN
#define EPL_C_ADR_RT1_DEF_NODE_ID           0xFE    // EPL default address of router type 1
#define EPL_C_ADR_SYNC_ON_SOC               0x00    // sync on SoC
#define EPL_C_ADR_SYNC_ON_SOA               0xFF    // sync on SoA
#define EPL_C_DLL_ASND_PRIO_NMTRQST         7       // increased ASnd request priority to be used by NMT Requests
#define EPL_C_DLL_ASND_PRIO_STD             0       // standard ASnd request priority
#define EPL_C_DLL_ETHERTYPE_EPL             0x88AB
#define EPL_C_DLL_ISOCHR_MAX_PAYL           1490    // Byte: maximum size of PReq and PRes payload data, requires C_IP_MAX_MTU
#define EPL_C_DLL_MAX_ASYNC_MTU             1500    // Byte: maximum asynchronous payload in bytes
#define EPL_C_DLL_MAX_PAYL_OFFSET           1499    // Byte: maximum offset of Ethernet frame payload, requires C_IP_MAX_MTU
#define EPL_C_DLL_MAX_RS                    7
#define EPL_C_DLL_MIN_ASYNC_MTU             300     // Byte: minimum asynchronous payload in bytes.
#define EPL_C_DLL_MIN_PAYL_OFFSET           45      // Byte: minimum offset of Ethernet frame payload
#define EPL_C_DLL_MULTICAST_ASND            0x01111E000004LL // EPL ASnd multicast MAC address, canonical form
#define EPL_C_DLL_MULTICAST_PRES            0x01111E000002LL // EPL PRes multicast MAC address, canonical form
#define EPL_C_DLL_MULTICAST_SOA             0x01111E000003LL // EPL SoA multicast MAC address, canonical form
#define EPL_C_DLL_MULTICAST_SOC             0x01111E000001LL // EPL Soc multicast MAC address, canonical form
#define EPL_C_DLL_PREOP1_START_CYCLES       10      // number of unassigning SoA frames at start of NMT_MS_PRE_OPERATIONAL_1
#define EPL_C_DLL_T_BITTIME                 10      // ns: Transmission time per bit on 100 Mbit/s network
#define EPL_C_DLL_T_EPL_PDO_HEADER          10      // Byte: size of PReq and PRes EPL PDO message header
#define EPL_C_DLL_T_ETH2_WRAPPER            18      // Byte: size of Ethernet type II wrapper consisting of header and checksum
#define EPL_C_DLL_T_IFG                     960     // ns: Ethernet Interframe Gap
#define EPL_C_DLL_T_MIN_FRAME               5120    // ns: Size of minimum Ethernet frame (without preamble)
#define EPL_C_DLL_T_PREAMBLE                640     // ns: Size of Ethernet frame preamble

#define EPL_C_DLL_MINSIZE_SOC               36      // minimum size of SoC without padding and CRC
#define EPL_C_DLL_MINSIZE_PREQ              60      // minimum size of PRec without CRC
#define EPL_C_DLL_MINSIZE_PRES              60      // minimum size of PRes without CRC
#define EPL_C_DLL_MINSIZE_SOA               54      // minimum size of SoA without padding and CRC
#define EPL_C_DLL_MINSIZE_IDENTRES          176     // minimum size of IdentResponse without CRC
#define EPL_C_DLL_MINSIZE_STATUSRES         72      // minimum size of StatusResponse without CRC
#define EPL_C_DLL_MINSIZE_SYNCRES           44      // minimum size of SyncResponse without padding and CRC
#define EPL_C_DLL_MINSIZE_NMTCMD            20      // minimum size of NmtCommand without CommandData, padding and CRC
#define EPL_C_DLL_MINSIZE_NMTCMDEXT         52      // minimum size of NmtCommand without padding and CRC
#define EPL_C_DLL_MINSIZE_NMTREQ            20      // minimum size of NmtRequest without CommandData, padding and CRC
#define EPL_C_DLL_MINSIZE_NMTREQEXT         52      // minimum size of NmtRequest without padding and CRC

#define EPL_C_ERR_MONITOR_DELAY             10      // Error monitoring start delay (not used in DS 1.0.0)
#define EPL_C_IP_ADR_INVALID                0x00000000L // invalid IP address (0.0.0.0) used to indicate no change
#define EPL_C_IP_INVALID_MTU                0       // Byte: invalid MTU size used to indicate no change
#define EPL_C_IP_MAX_MTU                    1518    // Byte: maximum size in bytes of the IP stack which must be processed.
#define EPL_C_IP_MIN_MTU                    300     // Byte: minimum size in bytes of the IP stack which must be processed.
#define EPL_C_NMT_STATE_TOLERANCE           5       // Cycles: maximum reaction time to NMT state commands
#define EPL_C_NMT_STATREQ_CYCLE             5       // sec: StatusRequest cycle time to be applied to AsyncOnly CNs
#define EPL_C_SDO_EPL_PORT                  3819

#define EPL_DLL_MACADDR_MASK                0xFFFFFFFFFFFFLL // MAC address mask, canonical form

//------------------------------------------------------------------------------
// constants for CDC file

#define EPL_CDC_OFFSET_INDEX                0
#define EPL_CDC_OFFSET_SUBINDEX             2
#define EPL_CDC_OFFSET_SIZE                 3
#define EPL_CDC_OFFSET_DATA                 7

//------------------------------------------------------------------------------
// constants for queue implementations

// define for event queue implementation
// These constants determine the implementation of the event queues
// Use this constants for EPL_***_QUEUE constants
#define EPL_QUEUE_DIRECT                    1
#define EPL_QUEUE_SHB                       2
#define EPL_QUEUE_HOSTINTERFACE             3       //use special host interface
#define EPL_QUEUE_IOCTL                     4
#define EPL_QUEUE_CIRCBUF                   5

#ifndef CONFIG_DLLCAL_QUEUE
#define CONFIG_DLLCAL_QUEUE                 EPL_QUEUE_CIRCBUF
#endif

//------------------------------------------------------------------------------
// definitions for usage of circular buffer library

// Circular buffer instances used in openPOWERLINK
#define CIRCBUF_USER_TO_KERNEL_QUEUE    0
#define CIRCBUF_KERNEL_TO_USER_QUEUE    1
#define CIRCBUF_KERNEL_INTERNAL_QUEUE   2
#define CIRCBUF_USER_INTERNAL_QUEUE     3
#define CIRCBUF_DLLCAL_TXGEN            4
#define CIRCBUF_DLLCAL_TXNMT            5
#define CIRCBUF_DLLCAL_TXSYNC           6
#define CIRCBUF_DLLCAL_CN_REQ_NMT       7
#define CIRCBUF_DLLCAL_CN_REQ_GEN       8
#define CIRCBUF_DLLCAL_CN_REQ_IDENT     9
#define CIRCBUF_DLLCAL_CN_REQ_STATUS    10

#ifndef EVENT_SIZE_CIRCBUF_KERNEL_TO_USER
#define EVENT_SIZE_CIRCBUF_KERNEL_TO_USER   32768   // default: 32 kByte
#endif

#ifndef EVENT_SIZE_CIRCBUF_USER_TO_KERNEL
#define EVENT_SIZE_CIRCBUF_USER_TO_KERNEL   32768   // default: 32 kByte
#endif

#ifndef EVENT_SIZE_CIRCBUF_KERNEL_INTERNAL
#define EVENT_SIZE_CIRCBUF_KERNEL_INTERNAL  32768   // default: 32 kByte
#endif

#ifndef EVENT_SIZE_CIRCBUF_USER_INTERNAL
#define EVENT_SIZE_CIRCBUF_USER_INTERNAL    32768   // default: 32 kByte
#endif

#ifndef DLLCAL_SIZE_CIRCBUF_CN_REQ_NMT
#define DLLCAL_SIZE_CIRCBUF_CN_REQ_NMT      2048
#endif

#ifndef DLLCAL_SIZE_CIRCBUF_CN_REQ_GEN
#define DLLCAL_SIZE_CIRCBUF_CN_REQ_GEN      2048
#endif

#ifndef DLLCAL_SIZE_CIRCBUF_REQ_IDENT
#define DLLCAL_SIZE_CIRCBUF_REQ_IDENT       256
#endif

#ifndef DLLCAL_SIZE_CIRCBUF_REQ_STATUS
#define DLLCAL_SIZE_CIRCBUF_REQ_STATUS      256
#endif

//------------------------------------------------------------------------------
// Default configuration macros

#ifndef EPL_DLL_PRES_CHAINING_CN
#define EPL_DLL_PRES_CHAINING_CN            FALSE
#endif

#ifndef EPL_DLL_PRES_CHAINING_MN
#define EPL_DLL_PRES_CHAINING_MN            FALSE
#else

// disable PRC MN support if NMT MN module is not activated
#if !defined(CONFIG_INCLUDE_NMT_MN)
#undef EPL_DLL_PRES_CHAINING_MN
#define EPL_DLL_PRES_CHAINING_MN            FALSE
#endif
#endif

#ifndef DLL_DEFERRED_RXFRAME_RELEASE_ISOCHRONOUS
#define DLL_DEFERRED_RXFRAME_RELEASE_ISOCHRONOUS    TRUE
#endif

#ifndef DLL_DEFERRED_RXFRAME_RELEASE_ASYNCHRONOUS
#define DLL_DEFERRED_RXFRAME_RELEASE_ASYNCHRONOUS   FALSE
#endif

#if defined(CONFIG_INCLUDE_NMT_MN)
    // MN should support generic Asnd frames, thus the maximum ID
    // is set to a large value
    #define EPL_C_DLL_MAX_ASND_SERVICE_IDS      253
#else
    // CN is usually low on resources, thus the maximum ID is
    // set as low as possible
    #if (EPL_DLL_PRES_CHAINING_CN == FALSE)
      #define EPL_C_DLL_MAX_ASND_SERVICE_IDS    5   // see tDllAsndServiceId in dll.h
    #else
      #define EPL_C_DLL_MAX_ASND_SERVICE_IDS    6
    #endif
#endif

#if (EPL_DLL_PRES_CHAINING_CN != FALSE)
#define EPL_TIMER_SYNC_SECOND_LOSS_OF_SYNC      TRUE
#endif

#ifndef EPL_D_PDO_Granularity_U8
#define EPL_D_PDO_Granularity_U8                8    // minimum size of objects to be mapped in [bit]
#endif

#ifndef EPL_D_PDO_RPDOChannelObjects_U8
#define EPL_D_PDO_RPDOChannelObjects_U8         254 // number of supported mapped objects per RPDO channel
#endif

#ifndef EPL_D_PDO_TPDOChannelObjects_U8
#define EPL_D_PDO_TPDOChannelObjects_U8         254 // number of supported mapped objects per TPDO channel
#endif

#ifndef EPL_D_PDO_RPDOChannels_U16
#define EPL_D_PDO_RPDOChannels_U16              256     // number of supported RPDO channels
#endif

#ifndef EPL_D_PDO_TPDOChannels_U16
#if defined(CONFIG_INCLUDE_NMT_MN)
#define EPL_D_PDO_TPDOChannels_U16              256     // number of supported TPDO channels
#else
#define EPL_D_PDO_TPDOChannels_U16              1       // number of supported TPDO channels
#endif
#endif

#define EPL_DLL_PROCESS_SYNC_ON_SOC             0
#define EPL_DLL_PROCESS_SYNC_ON_SOA             1
#define EPL_DLL_PROCESS_SYNC_ON_TIMER           2   // use special timer sync module

#ifndef EPL_DLL_PROCESS_SYNC
#define EPL_DLL_PROCESS_SYNC                    EPL_DLL_PROCESS_SYNC_ON_SOA // time of processing the isochronous task (sync callback of application and cycle preparation)
#endif

#ifndef EPL_DLL_SOC_SYNC_SHIFT_US
#define EPL_DLL_SOC_SYNC_SHIFT_US               150 // negative time shift of isochronous task in relation to SoC
#endif

#ifndef EPL_DLL_PRES_FILTER_COUNT
#if defined(CONFIG_INCLUDE_NMT_MN)
#define EPL_DLL_PRES_FILTER_COUNT               -1   // maximum count of Rx filter entries for PRes frames
#else
#define EPL_DLL_PRES_FILTER_COUNT               0    // maximum count of Rx filter entries for PRes frames
#endif
#endif

#ifndef EPL_NMT_MAX_NODE_ID
#if defined(CONFIG_INCLUDE_NMT_MN) || (EPL_DLL_PRES_FILTER_COUNT != 0)
#define EPL_NMT_MAX_NODE_ID                     254  // maximum node-ID with MN or cross-traffic support
#else
#define EPL_NMT_MAX_NODE_ID                     0    // maximum node-ID with MN or cross-traffic support
#endif
#endif

#ifndef EPL_D_NMT_MaxCNNumber_U8
#define EPL_D_NMT_MaxCNNumber_U8                239  // maximum number of supported regular CNs in the Node ID range 1 .. 239
#endif

#ifndef EPL_NMTMNU_PRES_CHAINING_MN
#define EPL_NMTMNU_PRES_CHAINING_MN             EPL_DLL_PRES_CHAINING_MN
#endif

#ifndef EPL_NMTMNU_PRC_NODE_ADD_MAX_NUM
#define EPL_NMTMNU_PRC_NODE_ADD_MAX_NUM         EPL_D_NMT_MaxCNNumber_U8
#endif

// defines for EPL API layer static process image
#ifndef EPL_API_PROCESS_IMAGE_SIZE_IN
#define EPL_API_PROCESS_IMAGE_SIZE_IN           0
#endif

#ifndef EPL_API_PROCESS_IMAGE_SIZE_OUT
#define EPL_API_PROCESS_IMAGE_SIZE_OUT          0
#endif

// configure whether OD access events shall be forwarded
// to user callback function.
// Because of reentrancy for local OD accesses, this has to be disabled
// when application resides in other address space as the stack (e.g. if
// EplApiLinuxUser.c and EplApiLinuxKernel.c are used)
#ifndef EPL_API_OBD_FORWARD_EVENT
#define EPL_API_OBD_FORWARD_EVENT               TRUE
#endif

#ifndef OBD_MAX_STRING_SIZE
#define OBD_MAX_STRING_SIZE                     32      // is used for objects 0x1008/0x1009/0x100A
#endif

#ifndef CONFIG_OBD_USE_STORE_RESTORE
#define CONFIG_OBD_USE_STORE_RESTORE            FALSE
#endif

#ifndef CONFIG_OBD_USE_LOAD_CONCISEDCF
#define CONFIG_OBD_USE_LOAD_CONCISEDCF          FALSE
#endif

#ifndef CONFIG_OBD_DEF_CONCISEDCF_FILENAME
#define CONFIG_OBD_DEF_CONCISEDCF_FILENAME      "pl_obd.cdc"
#endif

#ifndef CONFIG_OBD_CHECK_OBJECT_RANGE
#define CONFIG_OBD_CHECK_OBJECT_RANGE           TRUE
#endif

#ifndef CONFIG_OBD_USE_STRING_DOMAIN_IN_RAM
#define CONFIG_OBD_USE_STRING_DOMAIN_IN_RAM     TRUE
#endif

#ifndef CONFIG_OBD_INCLUDE_A000_TO_DEVICE_PART
#define CONFIG_OBD_INCLUDE_A000_TO_DEVICE_PART  FALSE
#endif

#ifndef EPL_VETH_NAME
#define EPL_VETH_NAME                           "plk"   // name of net device in Linux
#endif

// rough approximation of max. number of timer entries for module EplTimeruGeneric
#ifndef EPL_TIMERU_MAX_ENTRIES
#if defined(CONFIG_INCLUDE_NMT_MN)
#define EPL_TIMERU_MAX_ENTRIES                  (EPL_NMT_MAX_NODE_ID * 3)   // 3 timers for each node
#else
#define EPL_TIMERU_MAX_ENTRIES                  7   // LED module 1 + NMT module 1 + SDO sequence layer 5
#endif
#endif

#ifndef EDRV_FILTER_WITH_RX_HANDLER
#define EDRV_FILTER_WITH_RX_HANDLER             FALSE
#endif

#ifndef EDRV_AUTO_RESPONSE
#define EDRV_AUTO_RESPONSE                      FALSE
#endif

#ifndef EDRV_AUTO_RESPONSE_DELAY
#define EDRV_AUTO_RESPONSE_DELAY                FALSE
#endif

//------------------------------------------------------------------------------
// Defines for object 0x1F80 NMT_StartUp_U32
#define EPL_NMTST_STARTALLNODES                 0x00000002L // Bit 1
#define EPL_NMTST_NO_AUTOSTART                  0x00000004L // Bit 2
#define EPL_NMTST_NO_STARTNODE                  0x00000008L // Bit 3
#define EPL_NMTST_RESETALL_MAND_CN              0x00000010L // Bit 4
#define EPL_NMTST_STOPALL_MAND_CN               0x00000040L // Bit 6
#define EPL_NMTST_NO_AUTOPREOP2                 0x00000080L // Bit 7
#define EPL_NMTST_NO_AUTOREADYTOOP              0x00000100L // Bit 8
#define EPL_NMTST_EXT_CNIDENTCHECK              0x00000200L // Bit 9
#define EPL_NMTST_SWVERSIONCHECK                0x00000400L // Bit 10
#define EPL_NMTST_CONFCHECK                     0x00000800L // Bit 11
#define EPL_NMTST_NO_RETURN_PREOP1              0x00001000L // Bit 12
#define EPL_NMTST_BASICETHERNET                 0x00002000L // Bit 13

//------------------------------------------------------------------------------
// Defines for object 0x1F81 NMT_NodeAssignment_AU32
#define EPL_NODEASSIGN_NODE_EXISTS              0x00000001L // Bit 0
#define EPL_NODEASSIGN_NODE_IS_CN               0x00000002L // Bit 1
#define EPL_NODEASSIGN_START_CN                 0x00000004L // Bit 2
#define EPL_NODEASSIGN_MANDATORY_CN             0x00000008L // Bit 3
#define EPL_NODEASSIGN_KEEPALIVE                0x00000010L //currently not used in EPL V2 standard
#define EPL_NODEASSIGN_SWVERSIONCHECK           0x00000020L // Bit 5
#define EPL_NODEASSIGN_SWUPDATE                 0x00000040L // Bit 6
#define EPL_NODEASSIGN_ASYNCONLY_NODE           0x00000100L // Bit 8
#define EPL_NODEASSIGN_MULTIPLEXED_CN           0x00000200L // Bit 9
#define EPL_NODEASSIGN_RT1                      0x00000400L // Bit 10
#define EPL_NODEASSIGN_RT2                      0x00000800L // Bit 11
#define EPL_NODEASSIGN_MN_PRES                  0x00001000L // Bit 12
#define EPL_NODEASSIGN_PRES_CHAINING            0x00004000L // Bit 14
#define EPL_NODEASSIGN_VALID                    0x80000000L // Bit 31

#endif /* _INC_oplk_oplkdefs_H_ */
