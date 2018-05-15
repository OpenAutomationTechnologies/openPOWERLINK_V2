/**
********************************************************************************
\file   pdoucalmem-noosdual.c

\brief  PDO user CAL module using dual processor library

This file contains an implementation for the user PDO CAL shared-memory
module which uses the dual processor library to access it. The shared memory
is used to transfer PDO data between user and kernel layer. This implementation
is used if user and kernel layer are on two different processors using
a common memory.

\ingroup module_pdoucal
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2014, Kalycito Infotech Private Limited
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
#include <oplk/oplkinc.h>
#include <common/pdo.h>
#include <common/target.h>

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
#define DUALPROCSHM_BUFF_ID_PDO             13
#define DUALPROCSHM_ADDR_READ_TIMEOUT_MS    1000

//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------
/**
\brief Memory instance user PDO module

The structure contains all necessary information needed by the PDO CAL memory
module for a no-OS dual processor design.
*/
typedef struct
{
    tDualprocDrvInstance    pDrvInstance;   ///< Pointer to dual processor driver instance
} tMemInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tMemInstance         memPdo_l;

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

\ingroup module_pdoucal
*/
//------------------------------------------------------------------------------
tOplkError pdoucal_openMem(void)
{
    tDualprocDrvInstance    pInstance = dualprocshm_getLocalProcDrvInst();

    if (pInstance == NULL)
    {
        DEBUG_LVL_ERROR_TRACE("%s() couldn't get Host dual proc driver instance\n",
                              __func__);
        return kErrorNoResource;
    }

    memPdo_l.pDrvInstance = pInstance;

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Close PDO shared memory

The function performs all actions needed to clean up the shared memory at
shutdown.

\return The function returns a tOplkError error code.

\ingroup module_pdoucal
*/
//------------------------------------------------------------------------------
tOplkError pdoucal_closeMem(void)
{
    memPdo_l.pDrvInstance = NULL;

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Allocate PDO shared memory

The function allocates shared memory for the kernel needed to transfer the PDOs.

\param[in]      memSize_p           Size of PDO memory
\param[out]     ppPdoMem_p          Pointer to store the PDO memory pointer.

\return The function returns a tOplkError error code.

\ingroup module_pdoucal
*/
//------------------------------------------------------------------------------
tOplkError pdoucal_allocateMem(size_t memSize_p, void** ppPdoMem_p)
{
    tDualprocReturn dualRet;
    INT             loopCount = 0;

    // Check parameter validity
    ASSERT(ppPdoMem_p != NULL);

    for (loopCount = 0; loopCount < DUALPROCSHM_ADDR_READ_TIMEOUT_MS; loopCount++)
    {
        dualRet = dualprocshm_getMemory(memPdo_l.pDrvInstance,
                                        DUALPROCSHM_BUFF_ID_PDO,
                                        ppPdoMem_p,
                                        &memSize_p,
                                        FALSE);
        if (dualRet == kDualprocSuccessful)
            break;

        target_msleep(1);
    }

    if (loopCount == DUALPROCSHM_ADDR_READ_TIMEOUT_MS)
    {
        DEBUG_LVL_ERROR_TRACE("%s() couldn't allocate Pdo buffer (%d)\n",
                              __func__,
                              dualRet);
        return kErrorNoResource;
    }

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Free PDO shared memory

The function frees shared memory which was allocated in the kernel layer for
transferring the PDOs.

\param[in,out]  pMem_p              Pointer to the shared memory segment.
\param[in]      memSize_p           Size of PDO memory

\return The function returns a tOplkError error code.

\ingroup module_pdoucal
*/
//------------------------------------------------------------------------------
tOplkError pdoucal_freeMem(void* pMem_p, size_t memSize_p)
{
    tDualprocReturn dualRet;

    UNUSED_PARAMETER(memSize_p);
    UNUSED_PARAMETER(pMem_p); // Avoid warning if debug is disabled

    DEBUG_LVL_PDO_TRACE("%s() try to free address %p\n",
                        __func__,
                        pMem_p);

    dualRet = dualprocshm_freeMemory(memPdo_l.pDrvInstance,
                                     DUALPROCSHM_BUFF_ID_PDO,
                                     FALSE);
    if (dualRet != kDualprocSuccessful)
    {
        DEBUG_LVL_ERROR_TRACE("%s() couldn't free Pdo buffer (%d)\n",
                              __func__,
                              dualRet);
        return kErrorNoResource;
    }

    return kErrorOk;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

/// \}
