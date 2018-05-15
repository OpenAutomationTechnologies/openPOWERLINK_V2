/**
********************************************************************************
\file   oplkcfg.h

\brief  Configuration options for openPOWERLINK MN application library

This file contains the configuration options for the openPOWERLINK MN
application libary on Linux which is using the kernelspace interface.

*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2012, SYSTEC electronik GmbH
Copyright (c) 2017, B&R Industrial Automation GmbH
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

//==============================================================================
// generic defines which for whole openPOWERLINK stack
//==============================================================================

#ifndef BENCHMARK_MODULES
#define BENCHMARK_MODULES                               0 //0xEE800042L
#endif

// Default debug level:
// Only debug traces of these modules will be compiled which flags are set in define DEF_DEBUG_LVL.
#ifndef DEF_DEBUG_LVL
#define DEF_DEBUG_LVL                                   0xC0000000L
#endif

#undef FTRACE_DEBUG

/* assure that system priorities of hrtimer and net-rx kernel threads are set appropriate */
#define CONFIG_THREAD_PRIORITY_HIGH                     75
#define CONFIG_THREAD_PRIORITY_MEDIUM                   50
#define CONFIG_THREAD_PRIORITY_LOW                      49

// These macros defines all modules which are included
#define CONFIG_INCLUDE_NMT_MN
#define CONFIG_INCLUDE_PDO
#define CONFIG_INCLUDE_SDOS
#define CONFIG_INCLUDE_SDOC
#define CONFIG_INCLUDE_SDO_ASND
#define CONFIG_INCLUDE_SDO_UDP
#define CONFIG_INCLUDE_SDO_RW_MULTIPLE
#define CONFIG_INCLUDE_CFM
#define CONFIG_INCLUDE_VETH
#define CONFIG_INCLUDE_SOC_TIME_FORWARD

#define CONFIG_DLLCAL_QUEUE                             IOCTL_QUEUE

#define CONFIG_VETH_SET_DEFAULT_GATEWAY                 FALSE

#define CONFIG_CHECK_HEARTBEAT_PERIOD                   100        // 100 ms

//==============================================================================
// Data Link Layer (DLL) specific defines
//==============================================================================

// CN supports PRes Chaining
// NOTE: Ensure that this setting is equally configured in user and kernel layer!!
#define CONFIG_DLL_PRES_CHAINING_CN                     FALSE

// Disable deferred release of rx-buffers until EdrvPcap supports it
// NOTE: Ensure that these setting is equally configured in user and kernel layer!!
#define CONFIG_DLL_DEFERRED_RXFRAME_RELEASE_SYNC        FALSE
#define CONFIG_DLL_DEFERRED_RXFRAME_RELEASE_ASYNC       FALSE

//==============================================================================
// OBD specific defines
//==============================================================================

// Switch this define to TRUE if the stack should check the object ranges
#define CONFIG_OBD_CHECK_OBJECT_RANGE                   TRUE

// set this define to TRUE if there are strings or domains in OD, which
// may be changed in object size and/or object data pointer by its object
// callback function (called event kObdEvWrStringDomain)
#define CONFIG_OBD_USE_STRING_DOMAIN_IN_RAM             TRUE

#if defined(CONFIG_INCLUDE_CFM)
#define CONFIG_OBD_DEF_CONCISEDCF_FILENAME              "mnobd.cdc"
#define CONFIG_CFM_CONFIGURE_CYCLE_LENGTH               TRUE
#endif

// Configure if the range from 0xA000 is used for mapping client objects.
// openCONFIGURATOR uses this range for mapping objects.
#define CONFIG_OBD_INCLUDE_A000_TO_DEVICE_PART          TRUE

//==============================================================================
// SDO module specific defines
//==============================================================================

// increase the number of SDO channels, because we are master
#define CONFIG_SDO_MAX_CONNECTION_ASND                  100
#define CONFIG_SDO_MAX_CONNECTION_SEQ                   100
#define CONFIG_SDO_MAX_CONNECTION_COM                   100
#define CONFIG_SDO_MAX_CONNECTION_UDP                   50

#endif // _INC_oplkcfg_H_
