/**
********************************************************************************
\file   featureflags.h

\brief  Feature flag definitions

The header file contains definitions for the POWERLINK feature flags.
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2014, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
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

#ifndef _INC_featureflags_H_
#define _INC_featureflags_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <oplk/oplkinc.h>

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

// defines for EPL FeatureFlags
#define EPL_FEATURE_ISOCHR          0x00000001
#define EPL_FEATURE_SDO_UDP         0x00000002
#define EPL_FEATURE_SDO_ASND        0x00000004
#define EPL_FEATURE_SDO_PDO         0x00000008
#define EPL_FEATURE_NMT_INFO        0x00000010
#define EPL_FEATURE_NMT_EXT         0x00000020
#define EPL_FEATURE_PDO_DYN         0x00000040
#define EPL_FEATURE_NMT_UDP         0x00000080
#define EPL_FEATURE_CFM             0x00000100
#define EPL_FEATURE_DLL_MULTIPLEX   0x00000200  // CN specific
#define EPL_FEATURE_NODEID_SW       0x00000400
#define EPL_FEATURE_NMT_BASICETH    0x00000800  // MN specific
#define EPL_FEATURE_RT1             0x00001000
#define EPL_FEATURE_RT2             0x00002000
#define EPL_FEATURE_PRES_CHAINING   0x00040000


// generate EPL NMT_FeatureFlags_U32
#ifndef EPL_DEF_FEATURE_ISOCHR
    #if defined(CONFIG_INCLUDE_PDO)
        #define EPL_DEF_FEATURE_ISOCHR          (EPL_FEATURE_ISOCHR)
    #else
        #define EPL_DEF_FEATURE_ISOCHR          0
    #endif
#endif

#ifndef EPL_DEF_FEATURE_SDO_ASND
    #if defined(CONFIG_INCLUDE_SDO_ASND)
        #define EPL_DEF_FEATURE_SDO_ASND        (EPL_FEATURE_SDO_ASND)
    #else
        #define EPL_DEF_FEATURE_SDO_ASND        0
    #endif
#endif

#ifndef EPL_DEF_FEATURE_SDO_UDP
    #if defined(CONFIG_INCLUDE_SDO_UDP)
        #define EPL_DEF_FEATURE_SDO_UDP         (EPL_FEATURE_SDO_UDP)
    #else
        #define EPL_DEF_FEATURE_SDO_UDP         0
    #endif
#endif

#ifndef EPL_DEF_FEATURE_SDO_PDO
    #if defined(CONFIG_INCLUDE_SDO_PDO)
        #define EPL_DEF_FEATURE_SDO_PDO         (EPL_FEATURE_SDO_PDO)
    #else
        #define EPL_DEF_FEATURE_SDO_PDO         0
    #endif
#endif

#ifndef EPL_DEF_FEATURE_PDO_DYN
    #if defined(CONFIG_INCLUDE_PDO)
        #define EPL_DEF_FEATURE_PDO_DYN         (EPL_FEATURE_PDO_DYN)
    #else
        #define EPL_DEF_FEATURE_PDO_DYN         0
    #endif
#endif

#ifndef EPL_DEF_FEATURE_CFM
    #if defined(CONFIG_INCLUDE_CFM)
        #define EPL_DEF_FEATURE_CFM           (EPL_FEATURE_CFM)
    #else
        #define EPL_DEF_FEATURE_CFM           0
    #endif
#endif

#ifndef EPL_DEF_FEATURE_DLL_MULTIPLEX
    #define EPL_DEF_FEATURE_DLL_MULTIPLEX       (EPL_FEATURE_DLL_MULTIPLEX)
#endif

#ifndef EPL_DEF_FEATURE_PRES_CHAINING
    #if CONFIG_DLL_PRES_CHAINING_CN != FALSE
        #define EPL_DEF_FEATURE_PRES_CHAINING   (EPL_FEATURE_PRES_CHAINING)
    #else
        #define EPL_DEF_FEATURE_PRES_CHAINING   0
    #endif
#endif

#define EPL_DEF_FEATURE_FLAGS                   (EPL_DEF_FEATURE_ISOCHR \
                                                | EPL_DEF_FEATURE_SDO_ASND \
                                                | EPL_DEF_FEATURE_SDO_UDP \
                                                | EPL_DEF_FEATURE_SDO_PDO \
                                                | EPL_DEF_FEATURE_PDO_DYN \
                                                | EPL_DEF_FEATURE_CFM \
                                                | EPL_DEF_FEATURE_DLL_MULTIPLEX \
                                                | EPL_DEF_FEATURE_PRES_CHAINING)

#endif /* _INC_featureflags_H_ */
