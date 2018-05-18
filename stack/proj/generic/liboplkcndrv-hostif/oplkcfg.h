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
//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

//==============================================================================
// generic defines which for whole openPOWERLINK stack
//==============================================================================

#ifndef BENCHMARK_MODULES
#define BENCHMARK_MODULES                           0xEE800043L
#endif

// Default debug level:
// Only debug traces of these modules will be compiled which flags are set in define DEF_DEBUG_LVL.
#ifndef DEF_DEBUG_LVL
#define DEF_DEBUG_LVL                               0xC0000000L
#endif


// These macros define all modules which are included
#define CONFIG_INCLUDE_PDO
#define CONFIG_INCLUDE_MASND
#define CONFIG_INCLUDE_VETH
#define CONFIG_INCLUDE_LEDK
#define CONFIG_INCLUDE_SOC_TIME_FORWARD

#define CONFIG_DLLCAL_QUEUE                         CIRCBUF_QUEUE

#define CONFIG_VETH_SET_DEFAULT_GATEWAY             FALSE

#define CONFIG_CHECK_HEARTBEAT_PERIOD               1000            // 1000 ms

//==============================================================================
// Ethernet driver (Edrv) specific defines
//==============================================================================

// openMAC supports auto-response
#define CONFIG_EDRV_AUTO_RESPONSE                   TRUE

// Number of deferred Rx buffers
#define CONFIG_EDRV_ASND_DEFERRED_RX_BUFFERS       6

// Number of deferred Rx buffers
#define CONFIG_EDRV_VETH_DEFERRED_RX_BUFFERS       5

// openMAC supports auto-response delay
#define CONFIG_EDRV_AUTO_RESPONSE_DELAY             TRUE


//==============================================================================
// Data Link Layer (DLL) specific defines
//==============================================================================

// maximum count of Rx filter entries for PRes frames
#define CONFIG_DLL_PRES_FILTER_COUNT                3

#define CONFIG_DLL_PROCESS_SYNC                     DLL_PROCESS_SYNC_ON_TIMER

// negative time shift of isochronous task in relation to SoC
#define CONFIG_DLL_SOC_SYNC_SHIFT_US                150

// CN supports PRes Chaining
#define CONFIG_DLL_PRES_CHAINING_CN                 TRUE

// Disable/Enable late release
#define CONFIG_DLL_DEFERRED_RXFRAME_RELEASE_SYNC    FALSE
#define CONFIG_DLL_DEFERRED_RXFRAME_RELEASE_ASYNC   TRUE
#define CONFIG_EVENT_SIZE_CIRCBUF_KERNEL_TO_USER    HOSTIF_SIZE_K2UQ
#define CONFIG_EVENT_SIZE_CIRCBUF_USER_TO_KERNEL    HOSTIF_SIZE_U2KQ
#define CONFIG_DLLCAL_BUFFER_SIZE_TX_NMT            HOSTIF_SIZE_TXNMTQ
#define CONFIG_DLLCAL_BUFFER_SIZE_TX_GEN            HOSTIF_SIZE_TXGENQ
#define CONFIG_DLLCAL_BUFFER_SIZE_TX_SYNC           HOSTIF_SIZE_TXSYNCQ
#define CONFIG_DLLCAL_BUFFER_SIZE_TX_VETH           HOSTIF_SIZE_TXVETHQ
// Size of kernel internal queue
#define CONFIG_EVENT_SIZE_CIRCBUF_KERNEL_INTERNAL   1024

//==============================================================================
// Timer module specific defines
//==============================================================================

// if TRUE the high resolution timer module will be used
#define CONFIG_TIMER_USE_HIGHRES                    FALSE

#endif /* _INC_oplkcfg_H_ */
