/**
********************************************************************************
\file   errhndkcal-hostif.c

\brief  Implementation of kernel CAL module for error handler

This module implements the CAL functions in kernel layer for the error handler.
This implementation uses the host interface IP-Core from the kernel side.

\ingroup module_errhndkcal
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

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <common/oplkinc.h>
#include "errhndkcal.h"

#include <hostiflib.h>

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

\return tOplkError
\retval kErrorOk                    Successfully returned
\retval kErrorNoResource            IP-Core instance not found

\ingroup module_errhndkcal
*/
//------------------------------------------------------------------------------
tOplkError errhndkcal_initMemory(void)
{
    tHostifInstance pHostifInstance = hostif_getInstance(0);
    tOplkError      ret = kErrorOk;
    tHostifReturn   hifRet;
    void*           pBase;
    size_t          span;

    if (pHostifInstance == NULL)
    {
        ret = kErrorNoResource;
        goto Exit;
    }

    hifRet = hostif_getBuf(pHostifInstance, kHostifInstIdErrCount, &pBase, &span);
    if (hifRet != kHostifSuccessful)
    {
        DEBUG_LVL_ERROR_TRACE("%s() Could not get buffer from host interface (%d)\n",
                              __func__,
                              hifRet);
        ret = kErrorNoResource;
        goto Exit;
    }

    if (span < sizeof(tErrHndObjects))
    {
        DEBUG_LVL_ERROR_TRACE("%s: Error Handler Object Buffer too small\n",
                              __func__);
        ret = kErrorNoResource;
        goto Exit;
    }

    OPLK_MEMSET(pBase, 0, span);
    pErrHndObjects_l = (tErrHndObjects*)pBase;

Exit:
    return ret;
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
    pErrHndObjects_l = NULL;
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
