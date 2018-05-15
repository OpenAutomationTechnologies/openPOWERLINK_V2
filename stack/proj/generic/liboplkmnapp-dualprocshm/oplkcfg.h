/**
********************************************************************************
\file   oplkcfg.h

\brief  Configuration options for openPOWERLINK MN application library

This file contains the configuration options for the openPOWERLINK MN
application library without an OS which is using the dualprocshm interface.

*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2017, B&R Industrial Automation GmbH
Copyright (c) 2017, Kalycito Infotech Private Limited.
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
#define CONFIG_INCLUDE_SDO_ASND
#define CONFIG_INCLUDE_SDO_RW_MULTIPLE
#define CONFIG_INCLUDE_CFM
#define CONFIG_INCLUDE_VETH
#define CONFIG_INCLUDE_IP
#define CONFIG_INCLUDE_SOC_TIME_FORWARD

#ifndef BENCHMARK_MODULES
#define BENCHMARK_MODULES                   (0 | \
                                             BENCHMARK_MOD_32 \
                                            )
    ///< enable benchmark for specific stack modules
#endif
#ifndef DEF_DEBUG_LVL
#define DEF_DEBUG_LVL                       0xC0000000L
    ///< determine debug level for specific stack modules
#endif
/**@}*/

#define CONFIG_VETH_SET_DEFAULT_GATEWAY        FALSE

#define CONFIG_CHECK_HEARTBEAT_PERIOD           1000        // 1000 ms

/**
\name Object Dictionary defines
The OBD defines determine the Object Dictionary.
*/
/**@{*/
    ///< enable OBD in kernel layer
#define CONFIG_OBD_CHECK_OBJECT_RANGE              TRUE
    ///< support automatic object range check
#define CONFIG_OBD_USE_STRING_DOMAIN_IN_RAM        TRUE
    ///< support variable subindex
#define CONFIG_OBD_INCLUDE_A000_TO_DEVICE_PART     TRUE
/**@}*/

#if defined(CONFIG_INCLUDE_CFM)
#define CONFIG_CFM_CONFIGURE_CYCLE_LENGTH          TRUE
#endif

/**
\name Service Date Object defines
The SDO defines determine the SDO stack configuration.
*/
/**@{*/
#define CONFIG_SDO_MAX_CONNECTION_ASND             100
    ///< max. supported ASND SDO connections
#define CONFIG_SDO_MAX_CONNECTION_SEQ              100
    ///< max. supported SDO sequence connections
#define CONFIG_SDO_MAX_CONNECTION_COM              100
    ///< max. supported SDO command connections
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
