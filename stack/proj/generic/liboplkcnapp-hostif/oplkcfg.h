/**
********************************************************************************
\file   oplkcfg.h

\brief  configuration file

This header file configures the POWERLINK node.

*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2017, B&R Industrial Automation GmbH
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
#define CONFIG_INCLUDE_SDOS
#define CONFIG_INCLUDE_SDOC
#define CONFIG_INCLUDE_SDO_ASND
#define CONFIG_INCLUDE_SDO_RW_MULTIPLE
#define CONFIG_INCLUDE_MASND
#define CONFIG_INCLUDE_VETH
#define CONFIG_INCLUDE_SOC_TIME_FORWARD

#define CONFIG_DLLCAL_QUEUE                         CIRCBUF_QUEUE

#define CONFIG_VETH_SET_DEFAULT_GATEWAY             FALSE

#define CONFIG_CHECK_HEARTBEAT_PERIOD               1000            // 1000 ms

// CN supports PRes Chaining
#define CONFIG_DLL_PRES_CHAINING_CN                 TRUE

// maximum count of Rx filter entries for PRes frames
#define CONFIG_DLL_PRES_FILTER_COUNT                3               //FIXME
// This dependancy should be removed, since the user should not need to
// set the number of supported crosstraffic PDOs.
// Check also the correlation with 0x1F8D.

//==============================================================================
// OBD specific defines
//==============================================================================

// Switch this define to TRUE if the stack should check the object ranges
#define CONFIG_OBD_CHECK_OBJECT_RANGE               TRUE

// set this define to TRUE if there are strings or domains in OD, which
// may be changed in object size and/or object data pointer by its object
// callback function (called event kObdEvWrStringDomain)
#define CONFIG_OBD_USE_STRING_DOMAIN_IN_RAM         TRUE

// Set this string to true if OD configuration save and load feature is
// supported by the device
#ifdef CONFIG_INCLUDE_STORE_RESTORE
#define CONFIG_OBD_USE_STORE_RESTORE                TRUE
#define CONFIG_OBD_CALC_OD_SIGNATURE                TRUE
#endif

//==============================================================================
// SDO module specific defines
//==============================================================================

#endif /* _INC_oplkcfg_H_ */
