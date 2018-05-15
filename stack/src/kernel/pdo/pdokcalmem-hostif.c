/**
********************************************************************************
\file   pdokcalmem-hostif.c

\brief  PDO kernel CAL shared-memory module using host interface IP-Core

This file contains an implementation for the user PDO CAL shared-memroy module
utilizing the host interface IP-Core. Shared memory is used to transfer PDO data
between user and kernel layer. This implementation is used if user and kernel
layer are separated by host interface IP-Core.

\ingroup module_pdokcal
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
#include <kernel/pdokcal.h>

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
typedef struct
{
    void*   pBase;
    size_t  span;
} tLimInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tLimInstance         limPdo_l;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Open PDO shared memory

The function performs all actions needed to setup the shared memory at the
start of the stack.

\return The function returns a tOplkError error code.

\ingroup module_pdokcal
*/
//------------------------------------------------------------------------------
tOplkError pdokcal_openMem(void)
{
    tHostifReturn   hifret;
    tHostifInstance pInstance = hostif_getInstance(0);

    if (pInstance == NULL)
    {
        DEBUG_LVL_ERROR_TRACE("%s() couldn't get PCP hostif instance\n",
                              __func__);
        return kErrorNoResource;
    }

    hifret = hostif_getBuf(pInstance, kHostifInstIdPdo, &limPdo_l.pBase, &limPdo_l.span);

    if (hifret != kHostifSuccessful)
    {
        DEBUG_LVL_ERROR_TRACE("%s() Could not get buffer from host interface (%d)\n",
                              __func__,
                              hifret);
        return kErrorNoResource;
    }

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Close PDO shared memory

The function performs all actions needed to clean up the shared memory at
shutdown.

\return The function returns a tOplkError error code.

\ingroup module_pdokcal
*/
//------------------------------------------------------------------------------
tOplkError pdokcal_closeMem(void)
{
    OPLK_MEMSET(&limPdo_l, 0, sizeof(limPdo_l));

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Allocate PDO shared memory

The function allocates shared memory for the kernel needed to transfer the PDOs.

\param[in]      memSize_p           Size of PDO memory
\param[out]     ppPdoMem_p          Pointer to store the PDO memory pointer

\return The function returns a tOplkError error code.

\ingroup module_pdokcal
*/
//------------------------------------------------------------------------------
tOplkError pdokcal_allocateMem(size_t memSize_p, void** ppPdoMem_p)
{
    // Check parameter validity
    ASSERT(ppPdoMem_p != NULL);

    if (memSize_p > limPdo_l.span)
    {
        DEBUG_LVL_ERROR_TRACE("%s() out of memory (%d > %d)\n",
                              __func__,
                              memSize_p,
                              limPdo_l.span);
        return kErrorNoResource;
    }

    *ppPdoMem_p = (UINT8*)limPdo_l.pBase;

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Free PDO shared memory

The function frees shared memory which was allocated in the kernel layer for
transferring the PDOs.

\param[in,out]  pMem_p              Pointer to the shared memory segment
\param[in]      memSize_p           Size of PDO memory

\return The function returns a tOplkError error code.

\ingroup module_pdokcal
*/
//------------------------------------------------------------------------------
tOplkError pdokcal_freeMem(void* pMem_p, size_t memSize_p)
{
    UNUSED_PARAMETER(pMem_p);
    UNUSED_PARAMETER(memSize_p);

    DEBUG_LVL_PDO_TRACE("%s() try to free address %p (%p)\n",
                        __func__,
                        pMem_p,
                        limPdo_l.pBase);

    return kErrorOk;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

/// \}
