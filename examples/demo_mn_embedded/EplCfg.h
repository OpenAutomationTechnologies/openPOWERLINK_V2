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


#ifndef _INC_EPLCFG_H_
#define _INC_EPLCFG_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include "EplInc.h"

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

/**
\name Generic defines
The generic defines are valid for the whole openPOWERLINK stack.
*/
/**@{*/
    ///< use delete instance functions
#define USE_VAR64
    ///< data type support higher 32 bit
#define EPL_MAX_INSTANCES                   1
    ///< number of EPL instances
#define EPL_MODULE_INTEGRATION              (0 \
                                            | EPL_MODULE_OBD \
                                            | EPL_MODULE_PDOU \
                                            | EPL_MODULE_SDOS \
                                            | EPL_MODULE_SDOC \
                                            | EPL_MODULE_SDO_ASND \
                                            | EPL_MODULE_DLLU \
                                            | EPL_MODULE_NMT_MN \
                                            | EPL_MODULE_NMT_CN \
                                            | EPL_MODULE_NMTU \
                                            | EPL_MODULE_LEDU \
                                            | EPL_MODULE_CFM \
                                            )
    ///< integrate modules to stack
#ifndef BENCHMARK_MODULES
#define BENCHMARK_MODULES                   (0 \
                                            | BENCHMARK_MOD_32 \
                                            )
    ///< enable benchmark for specific stack modules
#endif
#ifndef DEF_DEBUG_LVL
#define DEF_DEBUG_LVL                       0x4C000001L
    ///< determine debug level for specific stack modules
#endif
/**@}*/

/**
\name Queue defines
The queue defines determine the stack's queues.
*/
/**@{*/
#define EPL_EVENT_K2U_QUEUE                 EPL_QUEUE_HOSTINTERFACE
    ///< kernel-to-user queue implementation
#define EPL_EVENT_U2K_QUEUE                 EPL_QUEUE_HOSTINTERFACE
    ///< user-to-kernel queue implementation
#define EPL_EVENT_UINT_QUEUE                EPL_QUEUE_DIRECT
    ///< user-internal queue implementation
#define CONFIG_DLLCAL_QUEUE                 EPL_QUEUE_HOSTINTERFACE
    ///< DLLCAL queue implementation
#define EPL_USE_SHAREDBUFF                  FALSE
    ///< disable shared buffer
/**@}*/

// Workaround: Specify any queue other than shb to disable shared buffer
#define EPL_EVENT_KINT_QUEUE                EPL_QUEUE_DIRECT

/**
\name Data Link Layer defines
The Data Link Layer (DLL) defines determine the POWERLINK DLL module.
*/
/**@{*/
#define EPL_DLL_PRES_CHAINING_MN            TRUE
    ///< support PRes-Chaining (PRC) for MN
/**@}*/

/**
\name Object Dictionary defines
The OBD defines determine the Object Dictionary.
*/
/**@{*/
    ///< enable OBD in kernel layer
#define CONFIG_OBD_CHECK_OBJECT_RANGE              FALSE
    ///< support automatic object range check
#define CONFIG_OBD_USE_STRING_DOMAIN_IN_RAM        TRUE
    ///< support variable subindex
#define CONFIG_OBD_INCLUDE_A000_TO_DEVICE_PART     TRUE
#define CONFIG_OBD_USE_LOAD_CONCISEDCF             TRUE
/**@}*/

/**
\name Service Date Object defines
The SDO defines determine the SDO stack configuration.
*/
/**@{*/
#define SDO_MAX_CONNECTION_ASND             100
    ///< max. supported ASND SDO connections
#define EPL_MAX_SDO_SEQ_CON                 100
    ///< max. supported SDO sequence connections
#define EPL_MAX_SDO_COM_CON                 100
    ///< max. supported SDO command connections
/**@}*/

//------------------------------------------------------------------------------
// typedef
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// function prototypes
//------------------------------------------------------------------------------

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
}
#endif

#endif /* _INC_EPLCFG_H_ */
