/**
********************************************************************************
\file   common/defaultcfg.h

\brief  Default configuration values

This file defines openPOWERLINK default configuration values.
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2016, B&R Industrial Automation GmbH
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
#ifndef _INC_common_defaultcfg_H_
#define _INC_common_defaultcfg_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// Default configuration macros
//------------------------------------------------------------------------------
#ifndef CONFIG_DLLCAL_QUEUE
#define CONFIG_DLLCAL_QUEUE                             CIRCBUF_QUEUE       // Configuration of DLLCAL queue: uses circular buffer per default
#endif

#ifndef CONFIG_EVENT_SIZE_CIRCBUF_KERNEL_TO_USER
#define CONFIG_EVENT_SIZE_CIRCBUF_KERNEL_TO_USER        32768               // Default size for kernel-to-user event queue
#endif

#ifndef CONFIG_EVENT_SIZE_CIRCBUF_USER_TO_KERNEL
#define CONFIG_EVENT_SIZE_CIRCBUF_USER_TO_KERNEL        32768               // Default size for user-to-kernel event queue
#endif

#ifndef CONFIG_EVENT_SIZE_CIRCBUF_KERNEL_INTERNAL
#define CONFIG_EVENT_SIZE_CIRCBUF_KERNEL_INTERNAL       32768               // Default size for kernel-internal event queue
#endif

#ifndef CONFIG_EVENT_SIZE_CIRCBUF_USER_INTERNAL
#define CONFIG_EVENT_SIZE_CIRCBUF_USER_INTERNAL         32768               // Default size for user-internal event queue
#endif

#ifndef CONFIG_DLLCAL_SIZE_CIRCBUF_CN_REQ_NMT
#define CONFIG_DLLCAL_SIZE_CIRCBUF_CN_REQ_NMT           2048                // Default size for NMT request queue
#endif

#ifndef CONFIG_DLLCAL_SIZE_CIRCBUF_CN_REQ_GEN
#define CONFIG_DLLCAL_SIZE_CIRCBUF_CN_REQ_GEN           2048                // Default size for generic request queue
#endif

#ifndef CONFIG_DLLCAL_SIZE_CIRCBUF_REQ_IDENT
#define CONFIG_DLLCAL_SIZE_CIRCBUF_REQ_IDENT            2048                // Default size for ident request queue
#endif

#ifndef CONFIG_DLLCAL_SIZE_CIRCBUF_REQ_STATUS
#define CONFIG_DLLCAL_SIZE_CIRCBUF_REQ_STATUS           2048                // Default size for status request queue
#endif

#ifndef CONFIG_DLLCAL_BUFFER_SIZE_TX_VETH
#define CONFIG_DLLCAL_BUFFER_SIZE_TX_VETH               32768               // Default size for virtual Ethernet Tx queue
#endif

#ifndef CONFIG_CTRL_FILE_CHUNK_SIZE
#define CONFIG_CTRL_FILE_CHUNK_SIZE                     1024
#endif

#ifndef CONFIG_DLL_PRES_CHAINING_CN
#define CONFIG_DLL_PRES_CHAINING_CN                     FALSE
#endif

#ifndef CONFIG_DLL_DEFERRED_RXFRAME_RELEASE_SYNC
#define CONFIG_DLL_DEFERRED_RXFRAME_RELEASE_SYNC        TRUE
#endif

#ifndef CONFIG_DLL_DEFERRED_RXFRAME_RELEASE_ASYNC
#define CONFIG_DLL_DEFERRED_RXFRAME_RELEASE_ASYNC       FALSE
#endif

#if defined(CONFIG_INCLUDE_NMT_MN)

// MN should support generic Asnd frames, thus the maximum ID
// is set to a large value
#define C_DLL_MAX_ASND_SERVICE_IDS                  253

#else /* defined(CONFIG_INCLUDE_NMT_MN) */

// CN is usually low on resources, thus the maximum ID is
// set as low as possible
#if (CONFIG_DLL_PRES_CHAINING_CN == FALSE)
#define C_DLL_MAX_ASND_SERVICE_IDS                  5                       // see tDllAsndServiceId in dll.h
#else /* (CONFIG_DLL_PRES_CHAINING_CN == FALSE) */
#define C_DLL_MAX_ASND_SERVICE_IDS                  6
#endif /* (CONFIG_DLL_PRES_CHAINING_CN == FALSE) */

#endif /* defined(CONFIG_INCLUDE_NMT_MN) */

#if (CONFIG_DLL_PRES_CHAINING_CN != FALSE)
#define TIMER_SYNC_SECOND_LOSS_OF_SYNC                  TRUE
#endif

#ifndef D_PDO_Granularity_U8
#define D_PDO_Granularity_U8                            8                   // minimum size of objects to be mapped in [bit]
#endif

#ifndef D_PDO_RPDOChannelObjects_U8
#define D_PDO_RPDOChannelObjects_U8                     254                 // number of supported mapped objects per RPDO channel
#endif

#ifndef D_PDO_TPDOChannelObjects_U8
#define D_PDO_TPDOChannelObjects_U8                     254                 // number of supported mapped objects per TPDO channel
#endif

#ifndef D_PDO_RPDOChannels_U16
#define D_PDO_RPDOChannels_U16                          256                 // number of supported RPDO channels
#endif

#ifndef D_PDO_TPDOChannels_U16
#if defined(CONFIG_INCLUDE_NMT_MN)
#define D_PDO_TPDOChannels_U16                          256                 // number of supported TPDO channels
#else /* defined(CONFIG_INCLUDE_NMT_MN) */
#define D_PDO_TPDOChannels_U16                          1                   // number of supported TPDO channels
#endif /* defined(CONFIG_INCLUDE_NMT_MN) */
#endif /* D_PDO_TPDOChannels_U16 */

/// \{ \name CN Synchronization options
#define DLL_PROCESS_SYNC_ON_SOC                         0                   ///< Sync on SoC frame
#define DLL_PROCESS_SYNC_ON_SOA                         1                   ///< Sync on SoA frame
#define DLL_PROCESS_SYNC_ON_TIMER                       2                   ///< Sync using special timer sync module
/// \}

#ifndef CONFIG_DLL_PROCESS_SYNC
#define CONFIG_DLL_PROCESS_SYNC                         DLL_PROCESS_SYNC_ON_SOA     // time of processing the isochronous task (sync callback of application and cycle preparation)
#endif

#ifndef CONFIG_DLL_SOC_SYNC_SHIFT_US
#define CONFIG_DLL_SOC_SYNC_SHIFT_US                    150                 // negative time shift of isochronous task in relation to SoC
#endif

#ifndef CONFIG_DLL_PRES_FILTER_COUNT
#if defined(CONFIG_INCLUDE_NMT_MN)
#define CONFIG_DLL_PRES_FILTER_COUNT                    -1                  // maximum count of Rx filter entries for PRes frames
#else /* defined(CONFIG_INCLUDE_NMT_MN) */
#define CONFIG_DLL_PRES_FILTER_COUNT                    0                   // maximum count of Rx filter entries for PRes frames
#endif /* defined(CONFIG_INCLUDE_NMT_MN) */
#endif /* CONFIG_DLL_PRES_FILTER_COUNT */

#ifndef NMT_MAX_NODE_ID
#if (defined(CONFIG_INCLUDE_NMT_MN) || (CONFIG_DLL_PRES_FILTER_COUNT != 0))
#define NMT_MAX_NODE_ID                                 254                 // maximum node-ID with MN or cross-traffic support
#else /* (defined(CONFIG_INCLUDE_NMT_MN) || (CONFIG_DLL_PRES_FILTER_COUNT != 0)) */
#define NMT_MAX_NODE_ID                                 0                   // maximum node-ID with MN or cross-traffic support
#endif /* (defined(CONFIG_INCLUDE_NMT_MN) || (CONFIG_DLL_PRES_FILTER_COUNT != 0)) */
#endif /* NMT_MAX_NODE_ID */

#ifndef D_NMT_MaxCNNumber_U8
#define D_NMT_MaxCNNumber_U8                            239                 // maximum number of supported regular CNs in the Node ID range 1 .. 239
#endif

#ifndef NMTMNU_PRC_NODE_ADD_MAX_NUM
#define NMTMNU_PRC_NODE_ADD_MAX_NUM                     D_NMT_MaxCNNumber_U8
#endif

// defines for POWERLINK API layer static process image
#ifndef API_PROCESS_IMAGE_SIZE_IN
#define API_PROCESS_IMAGE_SIZE_IN                       0
#endif

#ifndef API_PROCESS_IMAGE_SIZE_OUT
#define API_PROCESS_IMAGE_SIZE_OUT                      0
#endif

// configure whether OD access events shall be forwarded
// to user callback function.
// Because of reentrant behavior for local OD accesses, this has to be disabled
// when application resides in other address space as the stack
#ifndef API_OBD_FORWARD_EVENT
#define API_OBD_FORWARD_EVENT                           TRUE
#endif

#ifndef CONFIG_OBD_USE_STORE_RESTORE
#define CONFIG_OBD_USE_STORE_RESTORE                    FALSE
#endif

#ifndef CONFIG_OBD_CALC_OD_SIGNATURE
#define CONFIG_OBD_CALC_OD_SIGNATURE                    FALSE
#endif

#ifndef CONFIG_OBD_DEF_CONCISEDCF_FILENAME
#define CONFIG_OBD_DEF_CONCISEDCF_FILENAME              "pl_obd.cdc"
#endif

#ifndef CONFIG_OBD_CHECK_OBJECT_RANGE
#define CONFIG_OBD_CHECK_OBJECT_RANGE                   TRUE
#endif

#ifndef CONFIG_OBD_USE_STRING_DOMAIN_IN_RAM
#define CONFIG_OBD_USE_STRING_DOMAIN_IN_RAM             TRUE
#endif

#ifndef CONFIG_OBD_INCLUDE_A000_TO_DEVICE_PART
#define CONFIG_OBD_INCLUDE_A000_TO_DEVICE_PART          FALSE
#endif

#ifndef PLK_VETH_NAME
#define PLK_VETH_NAME                                   "plk_veth"          // name of net device in Linux
#endif

#ifndef CONFIG_VETH_SET_DEFAULT_GATEWAY
#define CONFIG_VETH_SET_DEFAULT_GATEWAY                 FALSE
#endif

#if (defined(CONFIG_INCLUDE_IP) && !defined(CONFIG_INCLUDE_VETH))
#error "CONFIG_INCLUDE_VETH needs to be enabled for using IP objects!"
#endif

// rough approximation of max. number of timer entries for module user/timer-generic
#ifndef TIMERU_MAX_ENTRIES
#if defined(CONFIG_INCLUDE_NMT_MN)
#define TIMERU_MAX_ENTRIES                              (NMT_MAX_NODE_ID * 3)  // 3 timers for each node
#else /* defined(CONFIG_INCLUDE_NMT_MN) */
#define TIMERU_MAX_ENTRIES                              7                      // LED module 1 + NMT module 1 + SDO sequence layer 5
#endif /* defined(CONFIG_INCLUDE_NMT_MN) */
#endif /* TIMERU_MAX_ENTRIES */

#ifndef EDRV_FILTER_WITH_RX_HANDLER
#define EDRV_FILTER_WITH_RX_HANDLER                     FALSE
#endif

#ifndef CONFIG_EDRV_AUTO_RESPONSE
#define CONFIG_EDRV_AUTO_RESPONSE                       FALSE
#endif

#ifndef CONFIG_EDRV_AUTO_RESPONSE_DELAY
#define CONFIG_EDRV_AUTO_RESPONSE_DELAY                 FALSE
#endif

#ifndef CONFIG_PDO_SETUP_WAIT_TIME
#define CONFIG_PDO_SETUP_WAIT_TIME                      500
#endif

#endif /* _INC_common_defaultcfg_H_ */
