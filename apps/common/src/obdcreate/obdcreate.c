/**
********************************************************************************
\file   obdcreate/obdcreate.c

\brief  Object dictionary creation

This file contains the OD data tables and the OD data structure initialization
function.

The OD data structure initialization is a very tricky part of the openPOWERLINK
stack. To create the different tables and code parts a set of macros defined in
obdmacro.h is used. These macros are redefined depending on some other
"type definition" macros. To create the different tables the specific
"type definition" macro will be set, the file objdict.h is included and therefore
the specified data structures are created. Afterwards the "type definition"
macro is unset, the next one is set and objdict.h is included again to generate
the next table.

\ingroup module_obd
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2016, B&R Industrial Automation GmbH
Copyright (c) 2013, SYSTEC electronic GmbH
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

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include "obdcreate.h"

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

// Definitions according to the stack configuration (needed for the OD)
#define OBD_MAX_STRING_SIZE                 32                  // used for objects 0x1008/0x1009/0x100A

#define PLK_PRODUCT_NAME                    "OPLK"
#define PLK_PRODUCT_VERSION                 PLK_DEFINED_STRING_VERSION


// generate NMT_FeatureFlags_U32 depending on configuration values

// Set to true if this node is able to communicate synchronously
#if defined(CONFIG_INCLUDE_PDO)
#define CONFIG_FF_ISOCHR                    (NMT_FEATUREFLAGS_ISOCHR)
#else
#define CONFIG_FF_ISOCHR                    0
#endif

// Set to true SDO via UDP is implemented
#if defined(CONFIG_INCLUDE_SDO_UDP)
#define CONFIG_FF_SDO_UDP                   (NMT_FEATUREFLAGS_SDO_UDP)
#else
#define CONFIG_FF_SDO_UDP                   0
#endif

// Set to true SDO via ASnd is implemented
#if defined(CONFIG_INCLUDE_SDO_ASND)
#define CONFIG_FF_SDO_ASND                  (NMT_FEATUREFLAGS_SDO_ASND)
#else
#define CONFIG_FF_SDO_ASND                  0
#endif

// Set to true if SDO/PDO is used (not implemented in the stack)
#if 0
#define CONFIG_FF_SDO_PDO                   (NMT_FEATUREFLAGS_SDO_PDO)
#else
#define CONFIG_FF_SDO_PDO                   0
#endif

// Set to true if NMT Info Services are used (not implemented in the stack)
#if 0
#define CONFIG_FF_NMT_INFO                  (NMT_FEATUREFLAGS_NMT_INFO)
#else
#define CONFIG_FF_NMT_INFO                  0
#endif

// Set to true if NMT Extended Commands are supported (always implemented in the stack)
#define CONFIG_FF_NMT_EXT                   (NMT_FEATUREFLAGS_NMT_EXT)

// Set to true if dynamic mapping is used (by default always true)
#if defined(CONFIG_INCLUDE_PDO)
#define CONFIG_FF_PDO_DYN                   (NMT_FEATUREFLAGS_PDO_DYN)
#else
#define CONFIG_FF_PDO_DYN                   0
#endif

// Set to true if NMT Services via UDP are used (not implemented in the stack)
#if 0
#define CONFIG_FF_NMT_UDP                   (NMT_FEATUREFLAGS_NMT_UDP)
#else
#define CONFIG_FF_NMT_UDP                   0
#endif

// Set to true if Configuration Manager is implemented on the MN
#if defined(CONFIG_INCLUDE_CFM)
#define CONFIG_FF_CFM                       (NMT_FEATUREFLAGS_CFM)
#else
#define CONFIG_FF_CFM                       0
#endif

// Multiplexing CN is implemented (always implemented in the stack)
#define CONFIG_FF_MUX_CN                    (NMT_FEATUREFLAGS_MUX_CN)

// Set to true if Node ID setup by SW is used
#if 0
#define CONFIG_FF_NODEID_SW                 (NMT_FEATUREFLAGS_NODEID_SW)
#else
#define CONFIG_FF_NODEID_SW                 0
#endif

// Set to true if Basic Ethernet mode is supported on an MN
#if 0
#define CONFIG_FF_BASIC_ETH_MN              (NMT_FEATUREFLAGS_BASIC_ETH_MN)
#else
#define CONFIG_FF_BASIC_ETH_MN              0
#endif

// Set to true if Routing Type 1 is supported
#if 0
#define CONFIG_FF_RT1                       (NMT_FEATUREFLAGS_RT1)
#else
#define CONFIG_FF_RT1                       0
#endif

// Set to true if Routing Type 2 is supported
#if 0
#define CONFIG_FF_RT2                       (NMT_FEATUREFLAGS_RT2)
#else
#define CONFIG_FF_RT2                       0
#endif

// Set to true if SDO Read/Write All By Index is used (not implemented in the stack)
#if 0
#define CONFIG_FF_SDO_RW_ALL                (NMT_FEATUREFLAGS_SDO_RW_ALL)
#else
#define CONFIG_FF_SDO_RW_ALL                0
#endif

// Set to true if SDO Read/Write Multiple Parameter By Index is used
#if defined(CONFIG_INCLUDE_SDO_RW_MULTIPLE)
#define CONFIG_FF_SDO_RW_MULTIPLE           (NMT_FEATUREFLAGS_SDO_RW_MULTIPLE)
#else
#define CONFIG_FF_SDO_RW_MULTIPLE           0
#endif

// Set to true if Multiple ASnd (DS302-B) is implemented
#if defined(CONFIG_INCLUDE_MASND)
#define CONFIG_FF_MASND                     (NMT_FEATUREFLAGS_MASND)
#else
#define CONFIG_FF_MASND                     0
#endif

// Set to true if Ring Redundancy Manager (DS302-A) is used (not implemented in the stack)
#if 0
#define CONFIG_FF_RR_MN                     (NMT_FEATUREFLAGS_RR_MN)
#else
#define CONFIG_FF_RR_MN                     0
#endif

// Set to true if PResChaining on the CN (DS302-C) is used
#if defined(CONFIG_DLL_PRES_CHAINING_CN)
#define CONFIG_FF_PRC                       (NMT_FEATUREFLAGS_PRC)
#else
#define CONFIG_FF_PRC                       0
#endif

// Set to true if Multiple PReq/PRes (DS302-D) is used (not implemented in the stack)
#if 0
#define CONFIG_FF_MULTI_PREQ_PRES           (NMT_FEATUREFLAGS_MULTI_PREQ_PRES)
#else
#define CONFIG_FF_MULTI_PREQ_PRES           0
#endif

// Set to true if Dynamic Node Allocation (DS302-E) is used (not implemented in the stack)
#if 0
#define CONFIG_FF_DNA                       (NMT_FEATUREFLAGS_DNA)
#else
#define CONFIG_FF_DNA                       0
#endif

// Set to true if a modular device (DS302-F) is created
#if defined(CONFIG_INCLUDE_MODULAR_DEVICE)
#define CONFIG_FF_MODULAR_DEVICE            (NMT_FEATUREFLAGS_MODULAR_DEVICE)
#else
#define CONFIG_FF_MODULAR_DEVICE            0
#endif

#define PLK_DEF_FEATURE_FLAGS (CONFIG_FF_ISOCHR             | \
                               CONFIG_FF_SDO_UDP            | \
                               CONFIG_FF_SDO_ASND           | \
                               CONFIG_FF_SDO_PDO            | \
                               CONFIG_FF_NMT_INFO           | \
                               CONFIG_FF_NMT_EXT            | \
                               CONFIG_FF_PDO_DYN            | \
                               CONFIG_FF_NMT_UDP            | \
                               CONFIG_FF_CFM                | \
                               CONFIG_FF_MUX_CN             | \
                               CONFIG_FF_NODEID_SW          | \
                               CONFIG_FF_BASIC_ETH_MN       | \
                               CONFIG_FF_RT1                | \
                               CONFIG_FF_RT2                | \
                               CONFIG_FF_SDO_RW_ALL         | \
                               CONFIG_FF_SDO_RW_MULTIPLE    | \
                               CONFIG_FF_MASND              | \
                               CONFIG_FF_RR_MN              | \
                               CONFIG_FF_PRC                | \
                               CONFIG_FF_MULTI_PREQ_PRES    | \
                               CONFIG_FF_DNA                | \
                               CONFIG_FF_MODULAR_DEVICE)


// macros to help building OD

// To prevent unused memory in subindex tables we need this macro.
// But not all compilers support to preset the last struct value followed by a comma.
// For compilers not supporting a comma after last struct value,  a dummy subindex
// has to be added.
#if ((DEV_SYSTEM & _DEV_COMMA_EXT_) != 0)
#define OBD_END_SUBINDEX()
#define OBD_MAX_ARRAY_SUBENTRIES    2
#else
#define OBD_END_SUBINDEX()          {0, 0, 0, NULL, NULL}
#define OBD_MAX_ARRAY_SUBENTRIES    3
#endif


//------------------------------------------------------------------------------
// global variables
//------------------------------------------------------------------------------

#if !defined(DOXYGEN_PARSER) // skip section for doxygen

// creation of data in ROM memory
#define OBD_CREATE_ROM_DATA
#include "objdict.h"
#undef OBD_CREATE_ROM_DATA

// creation of data in RAM memory
#define OBD_CREATE_RAM_DATA
#include "objdict.h"
#undef OBD_CREATE_RAM_DATA

// creation of subindex tables in ROM and RAM
#define OBD_CREATE_SUBINDEX_TAB
#include "objdict.h"
#undef OBD_CREATE_SUBINDEX_TAB

// creation of index tables for generic, manufacturer and device part
#define OBD_CREATE_INDEX_TAB
#include "objdict.h"
#undef OBD_CREATE_INDEX_TAB

#endif

//------------------------------------------------------------------------------
/**
\brief  Initialize OD data structures

The function initializes the object dictionary data structures.

\param[out]     pInitParam_p        Pointer to OD initialization parameters.

\return The function returns a tOplkError error code.

\ingroup module_obd
*/
//------------------------------------------------------------------------------
tOplkError obdcreate_initObd(tObdInitParam* pInitParam_p)
{
// Doxygen is confused by the inclusion of objdict.h in this function, therefore
// we exclude the function body from parsing by doxygen!
#if !defined(DOXYGEN_PARSER)

    tObdInitParam* pInitParam = pInitParam_p;           // pInitParam is required by obdmacro.h

    // check if pointer to parameter structure is valid
    // if not then only copy subindex tables below
    if (pInitParam != NULL)
    {
        // at first delete all parameters (all pointers will be set to NULL)
        memset(pInitParam, 0, sizeof(tObdInitParam));

        #define OBD_CREATE_INIT_FUNCTION
        {
            // inserts code to init pointer to index tables
            #include "objdict.h"
        }
        #undef OBD_CREATE_INIT_FUNCTION

#if (defined(OBD_USER_OD) && (OBD_USER_OD != FALSE))
        {
            // at the beginning no user OD is defined
            pInitParam->pUserPart = NULL;
        }
#endif
    }
    #define OBD_CREATE_INIT_SUBINDEX
    {
        // inserts code to copy subindex tables
        #include "objdict.h"
    }
    #undef OBD_CREATE_INIT_SUBINDEX

#endif // !defined(DOXYGEN_PARSER)
    return kErrorOk;
}
