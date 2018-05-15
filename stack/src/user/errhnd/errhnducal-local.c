/**
********************************************************************************
\file   errhnducal-local.c

\brief  Implementation of user CAL module for error handler

This module implements the user layer CAL functions of the error handler.
This implementation uses a global variable which will be referenced from user
and from kernel space. It can be used if the user and kernel part are running
in the same domain and global variables can be shared. It has to be used
together with errhndkcal-local.c which provides the global variable to store
the error objects!

\ingroup module_errhnducal
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2016, B&R Industrial Automation GmbH
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
#include "errhnducal.h"
#include <stddef.h>

//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// module global vars
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// global variable declaration
//------------------------------------------------------------------------------
extern tErrHndObjects   errhndk_errorObjects_g;

//------------------------------------------------------------------------------
// global function prototypes
//------------------------------------------------------------------------------

//============================================================================//
//            P R I V A T E   D E F I N I T I O N S                           //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tErrHndObjects* pLocalObjects_l;       ///< Pointer to user error objects

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief    Initialize error handler user CAL module

The function initializes the user layer CAL module of the error handler.

\param[in]      pLocalObjects_p     Pointer to local error objects

\return Always returns kErrorOk

\ingroup module_errhnducal
*/
//------------------------------------------------------------------------------
tOplkError errhnducal_init(tErrHndObjects* pLocalObjects_p)
{
    pLocalObjects_l = pLocalObjects_p;

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief    Shutdown error handler user CAL module

The function is used to de-initialize and shutdown the user layer CAL module of
the error handler.

\ingroup module_errhnducal
*/
//------------------------------------------------------------------------------
void errhnducal_exit(void)
{
    // currently nothing to do!
}

//------------------------------------------------------------------------------
/**
\brief    Write an error handler object

The function writes an error handler object to the shared memory region used
by user and kernel modules.

\param[in]      index_p             Index of object in object dictionary
\param[in]      subIndex_p          Subindex of object
\param[in]      pParam_p            Pointer to object in error handlers memory space

\return Returns a tOplkError error code.

\ingroup module_errhnducal
*/
//------------------------------------------------------------------------------
tOplkError errhnducal_writeErrorObject(UINT index_p,
                                       UINT subIndex_p,
                                       const UINT32* pParam_p)
{
    ptrdiff_t   offset;

    UNUSED_PARAMETER(index_p);
    UNUSED_PARAMETER(subIndex_p);

    // Check parameter validity
    ASSERT(pParam_p != NULL);

    offset = (UINT8*)pParam_p - (UINT8*)pLocalObjects_l;

    *(UINT32*)((UINT8*)&errhndk_errorObjects_g + offset) = *pParam_p;

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief    Read an error handler object

The function reads an error handler object from the shared memory region used
by user and kernel modules.

\param[in]      index_p             Index of object in object dictionary
\param[in]      subIndex_p          Subindex of object
\param[out]     pParam_p            Pointer to object in error handlers memory space

\return Returns a tOplkError error code.

\ingroup module_errhnducal
*/
//------------------------------------------------------------------------------
tOplkError errhnducal_readErrorObject(UINT index_p,
                                      UINT subIndex_p,
                                      UINT32* pParam_p)
{
    ptrdiff_t   offset;

    UNUSED_PARAMETER(index_p);
    UNUSED_PARAMETER(subIndex_p);

    // Check parameter validity
    ASSERT(pParam_p != NULL);

    offset = (UINT8*)pParam_p - (UINT8*)pLocalObjects_l;

    *pParam_p = *(UINT32*)((UINT8*)&errhndk_errorObjects_g + offset);

    return kErrorOk;
}


//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

/// \}
