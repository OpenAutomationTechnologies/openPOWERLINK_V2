/**
********************************************************************************
\file   oplkcfg.h

\brief  Configuration options for openPOWERLINK CN driver library

This file contains the configuration options for the openPOWERLINK CN driver
library without an OS which is using the dualprocshm interface.
*******************************************************************************/

/*------------------------------------------------------------------------------
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

#ifndef _INC_oplkcfg_H_
#define _INC_oplkcfg_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
// Include memory file to override default queue sizes
#include <dualprocshm-mem.h>
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
#define CONFIG_INCLUDE_SDOS
#define CONFIG_INCLUDE_SDOC
#define CONFIG_INCLUDE_LEDK
#define CONFIG_INCLUDE_SDO_ASND
#define CONFIG_INCLUDE_VETH

// Enable feature manually if the platform is supporting it (only C5 SoC and Zynq).
#define CONFIG_INCLUDE_SOC_TIME_FORWARD

#ifndef BENCHMARK_MODULES
#define BENCHMARK_MODULES                           (0 |\
                                                     BENCHMARK_MOD_01 | \
                                                     BENCHMARK_MOD_02 | \
                                                     BENCHMARK_MOD_03 | \
                                                     BENCHMARK_MOD_24)
    ///< enable benchmark for specific stack modules
#endif
#ifndef DEF_DEBUG_LVL
#define DEF_DEBUG_LVL                               0xC0000000L
    ///< determine debug level for specific stack modules
#endif
/**@}*/

/**
\name Ethernet driver defines
The Ethernet driver (Edrv) defines determine the stack's Ethernet module.
Note: The settings are specific for CN with openMAC!
*/
/**@{*/
#define CONFIG_EDRV_FAST_TXFRAMES                   FALSE
    ///< fast TX support by Edrv
#define CONFIG_EDRV_EARLY_RX_INT                    FALSE
    ///< support TX handler call when DMA transfer finished
#define CONFIG_EDRV_AUTO_RESPONSE                   TRUE
    ///< support auto-response (e.g. openMAC)
#define CONFIG_EDRV_AUTO_RESPONSE_DELAY             TRUE
/**@}*/

/**
\name Data Link Layer defines
The Data Link Layer (DLL) defines determine the POWERLINK DLL module.
*/
/**@{*/
#define CONFIG_DLL_PRES_READY_AFTER_SOC             FALSE
    ///< support PRes packet ready after SoC (CONFIG_EDRV_FAST_TXFRAMES necessary)
#define CONFIG_DLL_PRES_READY_AFTER_SOA             FALSE
    ///< support PRes packet ready after SoA (CONFIG_EDRV_FAST_TXFRAMES necessary)
#define CONFIG_DLL_PRES_FILTER_COUNT                3
    ///< max. supported PRes packet filters (for specific nodes)
#define CONFIG_DLL_PROCESS_SYNC                     DLL_PROCESS_SYNC_ON_TIMER
    ///< support usage of sync timer for process synchronization
#define CONFIG_DLL_DEFERRED_RXFRAME_RELEASE_SYNC    FALSE

#if !defined(CONFIG_DLL_DEFERRED_RXFRAME_RELEASE_ASYNC)
#define CONFIG_DLL_DEFERRED_RXFRAME_RELEASE_ASYNC   TRUE
#endif

#define CONFIG_EDRV_ASND_DEFERRED_RX_BUFFERS        8
    ///< disable deferred RX frames if Edrv does not support it
#define CONFIG_EDRV_VETH_DEFERRED_RX_BUFFERS        5
/**@}*/

/**
\name Timer defines
The timer defines determine the high resolution timer module.
*/
/**@{*/
#define CONFIG_TIMER_USE_HIGHRES                    FALSE
    ///< use high resolution timer
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
