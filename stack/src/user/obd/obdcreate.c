/**
********************************************************************************
\file   obdcreate.c

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
Copyright (c) 2014, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
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
#include <common/oplkinc.h>
#include <oplk/obd.h>             // function prototypes of the obd module
#include <user/pdou.h>            // function prototype of OD callback function
#include <user/errhndu.h>         // function prototype of error handler od callback functions
#include <user/ctrlu.h>

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

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

\param  pInitParam_p            Pointer to OD initialization parameters.

\return The function returns a tOplkError error code.

\ingroup module_obd
*/
//------------------------------------------------------------------------------
tOplkError obd_initObd(tObdInitParam MEM* pInitParam_p)
{
// Doxygen is confused by the inclusion of objdict.h in this function, therefore
// we exclude the function body from parsing by doxygen!
#if !defined(DOXYGEN_PARSER)

    tObdInitParam MEM* pInitParam = pInitParam_p;

    // check if pointer to parameter structure is valid
    // if not then only copy subindex tables below
    if (pInitParam != NULL)
    {
        // at first delete all parameters (all pointers will be set to NULL)
        OPLK_MEMSET(pInitParam, 0, sizeof(tObdInitParam));

        #define OBD_CREATE_INIT_FUNCTION
        {
            // inserts code to init pointer to index tables
            #include "objdict.h"
        }
        #undef OBD_CREATE_INIT_FUNCTION

#if (defined(OBD_USER_OD) && (OBD_USER_OD != FALSE))
        {
            // at the beginning no user OD is defined
            pInitParam_p->pUserPart = NULL;
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
