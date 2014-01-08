/**
********************************************************************************
\file   EplCfg.h

\brief  configuration file

This header file configures the POWERLINK node.

*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2012, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
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


#ifndef _EPLCFG_H_
#define _EPLCFG_H_

#include "EplInc.h"


// =========================================================================
// generic defines which for whole EPL Stack
// =========================================================================
// needed to support datatypes over 32 bit by global.h
#define USE_VAR64

// EPL_MAX_INSTANCES specifies count of instances of all EPL modules.
// If it is greater than 1 the first parameter of all
// functions is the instance number.
#define EPL_MAX_INSTANCES               1

// This defines the target hardware. Here is encoded wich CPU and wich external
// peripherals are connected. For possible values refere to target.h. If
// necessary value is not available EPL stack has to
// be adapted and tested.
//#define TARGET_HARDWARE                 TGTHW_PC_WRAPP

// determine event queue implementation
// -> internal and u2k queues: direct call
// -> k2u queue: circular buffer
#define EPL_EVENT_K2U_QUEUE             EPL_QUEUE_CIRCBUF
#define EPL_EVENT_U2K_QUEUE             EPL_QUEUE_DIRECT
#define EPL_EVENT_KINT_QUEUE            EPL_QUEUE_DIRECT
#define EPL_EVENT_UINT_QUEUE            EPL_QUEUE_DIRECT

#define CONFIG_DLLCAL_QUEUE             EPL_QUEUE_DIRECT

#ifndef BENCHMARK_MODULES
//#define BENCHMARK_MODULES       0 //0xEE800042L
#define BENCHMARK_MODULES       0xEE800043L
#endif

// Default debug level:
// Only debug traces of these modules will be compiled which flags are set in define DEF_DEBUG_LVL.
#ifndef DEF_DEBUG_LVL
#define DEF_DEBUG_LVL           0x40000000L //0xEC000000L
#endif
//   EPL_DBGLVL_OBD         =   0x00000004L
// * EPL_DBGLVL_ASSERT      =   0x20000000L
// * EPL_DBGLVL_ERROR       =   0x40000000L
// * EPL_DBGLVL_ALWAYS      =   0x80000000L

// These macros define all modules which are included
#define CONFIG_INCLUDE_PDOU
#define CONFIG_INCLUDE_PDOK
#define CONFIG_INCLUDE_SDOS
#define CONFIG_INCLUDE_SDOC
#define CONFIG_INCLUDE_SDO_ASND
#define CONFIG_INCLUDE_NMTK
#define CONFIG_INCLUDE_NMTU
#define CONFIG_INCLUDE_LEDU

// =========================================================================
// EPL ethernet driver (Edrv) specific defines
// =========================================================================

// switch this define to TRUE if Edrv supports fast tx frames
#define EDRV_FAST_TXFRAMES              FALSE
//#define EDRV_FAST_TXFRAMES              TRUE

// switch this define to TRUE if Edrv supports early receive interrupts
#define EDRV_EARLY_RX_INT               FALSE
//#define EDRV_EARLY_RX_INT               TRUE

// enables setting of several port pins for benchmarking purposes
#define EDRV_BENCHMARK                  FALSE
//#define EDRV_BENCHMARK                  TRUE // MCF_GPIO_PODR_PCIBR

// Call Tx handler (i.e. EplDllCbFrameTransmitted()) already if DMA has finished,
// otherwise call the Tx handler if frame was actually transmitted over ethernet.
#define EDRV_DMA_TX_HANDLER             FALSE
//#define EDRV_DMA_TX_HANDLER             TRUE

// number of used ethernet controller
#define EDRV_USED_ETH_CTRL              1

// openMAC supports auto-response
#define EDRV_AUTO_RESPONSE              TRUE

#define EDRV_ASND_DEFFERRED_RX_BUFFERS  6   ///< Number of deferred Rx buffers

// increase the number of Tx buffers, because we are master
// and need one Tx buffer for each PReq and CN
// + SoC + SoA + MN PRes + NmtCmd + ASnd + IdentRes + StatusRes.
//#define EDRV_MAX_TX_BUFFERS             5

// openMAC supports auto-response delay
#define EDRV_AUTO_RESPONSE_DELAY        TRUE


// =========================================================================
// Data Link Layer (DLL) specific defines
// =========================================================================

// switch this define to TRUE if Edrv supports fast tx frames
// and DLL shall pass PRes as ready to Edrv after SoC
#define EPL_DLL_PRES_READY_AFTER_SOC    FALSE
//#define EPL_DLL_PRES_READY_AFTER_SOC    TRUE

// switch this define to TRUE if Edrv supports fast tx frames
// and DLL shall pass PRes as ready to Edrv after SoA
#define EPL_DLL_PRES_READY_AFTER_SOA    FALSE
//#define EPL_DLL_PRES_READY_AFTER_SOA    TRUE

// maximum count of Rx filter entries for PRes frames
#define EPL_DLL_PRES_FILTER_COUNT       3


#define EPL_DLL_PROCESS_SYNC        EPL_DLL_PROCESS_SYNC_ON_TIMER

// negative time shift of isochronous task in relation to SoC
#define EPL_DLL_SOC_SYNC_SHIFT_US       150

// CN supports PRes Chaining
#define EPL_DLL_PRES_CHAINING_CN        TRUE

// Disable isochronous late release
#define DLL_DEFERRED_RXFRAME_RELEASE_ISOCHRONOUS    FALSE

// Enable late release for asynchronous frames being processed in background
#define DLL_DEFERRED_RXFRAME_RELEASE_ASYNCHRONOUS   TRUE

// Asynchronous transmit buffer for NMT frames in bytes
#define DLLCAL_BUFFER_SIZE_TX_NMT           4096

// Asynchronous transmit buffer for generic Asnd frames in bytes
#define DLLCAL_BUFFER_SIZE_TX_GEN_ASND      8192

// Asynchronous transmit buffer for generic Ethernet frames in bytes
#define DLLCAL_BUFFER_SIZE_TX_GEN_ETH       8192

// Asynchronous transmit buffer for sync response frames in bytes
#define DLLCAL_BUFFER_SIZE_TX_SYNC          4096

// Size of kernel to user queue
#define EVENT_SIZE_CIRCBUF_KERNEL_TO_USER   8192

// =========================================================================
// OBD specific defines
// =========================================================================

// switch this define to TRUE if Epl should compare object range
// automaticly
#define CONFIG_OBD_CHECK_OBJECT_RANGE          FALSE
//#define CONFIG_OBD_CHECK_OBJECT_RANGE          TRUE

// set this define to TRUE if there are strings or domains in OD, which
// may be changed in object size and/or object data pointer by its object
// callback function (called event kObdEvWrStringDomain)
//#define CONFIG_OBD_USE_STRING_DOMAIN_IN_RAM    FALSE
#define CONFIG_OBD_USE_STRING_DOMAIN_IN_RAM    TRUE

// =========================================================================
// Timer module specific defines
// =========================================================================

// if TRUE the high resolution timer module will be used
#define EPL_TIMER_USE_HIGHRES           FALSE



// =========================================================================
// SDO module specific defines
// =========================================================================

// do not increase the number of SDO channels, because we are not master
//#define SDO_MAX_CONNECTION_ASND     100
//#define MAX_SDO_SEQ_CON             100
//#define MAX_SDO_COM_CON             100
//#define SDO_MAX_CONNECTION_UDP      50


// =========================================================================
// API Layer specific defines
// =========================================================================

//#define EPL_API_PROCESS_IMAGE_SIZE_IN 0 //disable
//#define EPL_API_PROCESS_IMAGE_SIZE_OUT 0 //disable

#endif //_EPLCFG_H_



