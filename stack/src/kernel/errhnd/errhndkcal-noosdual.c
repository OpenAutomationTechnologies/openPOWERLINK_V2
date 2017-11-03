/**
********************************************************************************
\file   errhndkcal-noosdual.c

\brief  Implementation of kernel CAL module for error handler

This module implements the CAL functions in kernel layer for the error handler.
This implementation uses shared memory to share the error objects
between user and kernel part running on two different processors.

\ingroup module_errhndkcal
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2014, Kalycito Infotech Private Limited
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
#include <oplk/oplkinc.h>
#include "errhndkcal.h"

#include <dualprocshm.h>

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
// global function prototypes
//------------------------------------------------------------------------------

//============================================================================//
//            P R I V A T E   D E F I N I T I O N S                           //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#define DUALPROCSHM_BUFF_ID_ERRHDLR    12

//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tErrHndObjects*  pErrHndObjects_l;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Initialize kernel layer error handler memory

The function initializes the kernel layer error handler memory.

\return The function returns a tOplkError error code.

\ingroup module_errhndkcal
*/
//------------------------------------------------------------------------------
tOplkError errhndkcal_initMemory(void)
{
    tDualprocReturn         dualRet;
    tDualprocDrvInstance    pInstance = dualprocshm_getLocalProcDrvInst();
    void*                   pBase;
    size_t                  span;

    if (pInstance == NULL)
        return kErrorNoResource;

    if (pErrHndObjects_l != NULL)
        return kErrorNoFreeInstance;

    span = sizeof(tErrHndObjects);

    dualRet = dualprocshm_getMemory(pInstance,
                                    DUALPROCSHM_BUFF_ID_ERRHDLR,
                                    &pBase,
                                    &span,
                                    TRUE);
    if (dualRet != kDualprocSuccessful)
    {
        DEBUG_LVL_ERROR_TRACE("%s() couldn't allocate Error counter buffer (%d)\n",
                              __func__,
                              dualRet);
        return kErrorNoResource;
    }

    if (span < sizeof(tErrHndObjects))
    {
        DEBUG_LVL_ERROR_TRACE("%s: Error Handler Object Buffer too small\n",
                              __func__);
        return kErrorNoResource;
    }

    pErrHndObjects_l = (tErrHndObjects*)pBase;

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  De-initialize kernel layer error handler memory

The function is used to de-initialize the kernel layer error handler memory.

\ingroup module_errhndkcal
*/
//------------------------------------------------------------------------------
void errhndkcal_deinitMemory(void)
{
    tDualprocDrvInstance    pInstance = dualprocshm_getLocalProcDrvInst();

    if (pErrHndObjects_l != NULL)
    {
        dualprocshm_freeMemory(pInstance, DUALPROCSHM_BUFF_ID_ERRHDLR, TRUE);
        pErrHndObjects_l = NULL;
    }
}

//------------------------------------------------------------------------------
/**
\brief  Get pointer to error handler objects

The function returns a pointer to the memory block where the error handler
objects are stored.

\return The function returns a pointer to the error handler objects.

\ingroup module_errhndkcal
*/
//------------------------------------------------------------------------------
tErrHndObjects* errhndkcal_getMemPtr(void)
{
    return pErrHndObjects_l;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

/// \}
