/**
********************************************************************************
\file   oplkcfg.h

\brief  configuration file

This header file configures the POWERLINK node.
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2018, B&R Industrial Automation GmbH
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


#ifndef _INC_oplkcfg_H_
#define _INC_oplkcfg_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <hostiflib-mem.h> // For host interface memory sizes
#include <oplkcfg-board.h> // Board specific configuration

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

/**
\name Generic defines
The generic defines are valid for the whole openPOWERLINK stack.
*/
/**@{*/

// These macros define all modules which are included
#define CONFIG_INCLUDE_PDO
#define CONFIG_INCLUDE_NMT_MN
#define CONFIG_INCLUDE_SDOS
#define CONFIG_INCLUDE_SDOC
#define CONFIG_INCLUDE_LEDK
#define CONFIG_INCLUDE_SDO_ASND
#define CONFIG_INCLUDE_VETH
#define CONFIG_INCLUDE_SOC_TIME_FORWARD

#ifndef BENCHMARK_MODULES
#define BENCHMARK_MODULES                   (0 | \
                                             BENCHMARK_MOD_01 | \
                                             BENCHMARK_MOD_02 | \
                                             BENCHMARK_MOD_03 | \
                                             BENCHMARK_MOD_24)
    ///< enable benchmark for specific stack modules
#endif
#ifndef DEF_DEBUG_LVL
#define DEF_DEBUG_LVL                       0xC0000000L
    ///< determine debug level for specific stack modules
#endif
/**@}*/

/**
\name Ethernet driver defines
The Ethernet driver (Edrv) defines determine the stack's Ethernet module.
Note: The settings are specific for MN with openMAC!
*/
/**@{*/
#define CONFIG_EDRV_AUTO_RESPONSE           TRUE
    ///< support auto-response (e.g. openMAC)
#define CONFIG_EDRV_AUTO_RESPONSE_DELAY     TRUE
#define CONFIG_EDRV_TIME_TRIG_TX            TRUE
    ///< support time triggered transmission (e.g. openMAC)
#define CONFIG_EDRV_MAX_TX2_BUFFERS         64
    ///< set number for second Tx buffer queue to support larger networks
/**@}*/

/**
\name Data Link Layer defines
The Data Link Layer (DLL) defines determine the POWERLINK DLL module.
*/
/**@{*/
#define CONFIG_DLL_PRES_FILTER_COUNT           3
    ///< max. supported PRes packet filters (for specific nodes)
#define CONFIG_DLL_DEFERRED_RXFRAME_RELEASE_SYNC    FALSE
    ///< disable deferred RX frames if Edrv does not support it
#define CONFIG_DLL_DEFERRED_RXFRAME_RELEASE_ASYNC   TRUE
#define CONFIG_EDRV_ASND_DEFERRED_RX_BUFFERS        8
#define CONFIG_EDRV_VETH_DEFERRED_RX_BUFFERS        5
/**@}*/

/**
\name Timer defines
The timer defines determine the high resolution timer module.
*/
/**@{*/
#define CONFIG_TIMER_USE_HIGHRES               TRUE
    ///< use high resolution timer
/**@}*/

/**
\name Memory size for queues
These defines are set by the host interface ipcore settings
*/
/**@{*/
#define CONFIG_EVENT_SIZE_CIRCBUF_KERNEL_TO_USER    HOSTIF_SIZE_K2UQ
#define CONFIG_EVENT_SIZE_CIRCBUF_USER_TO_KERNEL    HOSTIF_SIZE_U2KQ
#define CONFIG_DLLCAL_BUFFER_SIZE_TX_NMT            HOSTIF_SIZE_TXNMTQ
#define CONFIG_DLLCAL_BUFFER_SIZE_TX_GEN            HOSTIF_SIZE_TXGENQ
#define CONFIG_DLLCAL_BUFFER_SIZE_TX_SYNC           HOSTIF_SIZE_TXSYNCQ
#define CONFIG_DLLCAL_BUFFER_SIZE_TX_VETH           HOSTIF_SIZE_TXVETHQ
/**@}*/

//------------------------------------------------------------------------------
// typedef
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// function prototypes
//------------------------------------------------------------------------------

#ifdef __cplusplus
extern "C"
{
#endif

#ifdef __cplusplus
}
#endif

#endif /* _INC_oplkcfg_H_ */
