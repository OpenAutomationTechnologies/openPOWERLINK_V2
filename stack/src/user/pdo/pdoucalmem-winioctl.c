/**
********************************************************************************
\file   pdoucalmem-winioctl.c

\brief  PDO user CAL shared-memory module using the Windows kernel driver

This file implements the PDO CAL memory module to acquire the memory resources
for PDO exchange between user-kernel layers for a Windows user-kernel interface
using IOCTL.

The kernel driver is responsible for allocating/acquiring the shared memory
for the PDOs and map virtual address in user space. The control CAL module
then retrieves the virtual address for the memory and manages future access
in user layer. The PDO memory address is retrieved from the control CAL
module using the offset acquired from the kernel driver.

\ingroup module_pdoucal
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2017, Kalycito Infotech Private Limited
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
#include <common/driver.h>
#include <user/ctrlucal.h>


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
static OPLK_FILE_HANDLE hFileHandle_l;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Open PDO shared memory

The function performs all actions needed to set up the shared memory at the
start/initialization of the stack.

\return The function returns a tOplkError error code.

\ingroup module_pdokcal
*/
//------------------------------------------------------------------------------
tOplkError pdoucal_openMem(void)
{
    hFileHandle_l = ctrlucal_getFd();

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
tOplkError pdoucal_closeMem(void)
{
    hFileHandle_l = NULL;

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Allocate PDO shared memory

The function allocates shared memory for the user needed to transfer the PDOs.

\param[in]      memSize_p           Size of PDO memory.
\param[out]     ppPdoMem_p          Pointer to store the PDO memory pointer.

\return The function returns a tOplkError error code.

\ingroup module_pdokcal
*/
//------------------------------------------------------------------------------
tOplkError pdoucal_allocateMem(size_t memSize_p, void** ppPdoMem_p)
{
    ULONG       bytesReturned;
    tPdoMem     inPdoMem;
    tPdoMem     outPdoMem;
    tOplkError  ret;
    void*       pPdoMem = NULL;
    BOOL        fIoctlRet;

    // Check parameter validity
    ASSERT(ppPdoMem_p != NULL);

    if (hFileHandle_l == NULL)
        return kErrorNoResource;

    inPdoMem.memSize = memSize_p;

    fIoctlRet = DeviceIoControl(hFileHandle_l,
                                PLK_CMD_PDO_GET_MEM,
                                &inPdoMem,
                                sizeof(tPdoMem),
                                &outPdoMem,
                                sizeof(tPdoMem),
                                &bytesReturned,
                                NULL);
    if (!fIoctlRet || (bytesReturned == 0))
    {
        *ppPdoMem_p = NULL;
        return kErrorNoResource;
    }

    ret = ctrlucal_getMappedMem(outPdoMem.pdoMemOffset,
                                outPdoMem.memSize,
                                &pPdoMem);

    if ((ret != kErrorOk) || (pPdoMem == NULL))
        return kErrorNoResource;

    *ppPdoMem_p = pPdoMem;

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Free PDO shared memory

The function frees shared memory which was allocated in the user layer for
transferring the PDOs.

\param[in,out]  pMem_p              Pointer to the shared memory segment.
\param[in]      memSize_p           Size of PDO memory.

\return The function returns a tOplkError error code.

\ingroup module_pdokcal
*/
//------------------------------------------------------------------------------
tOplkError pdoucal_freeMem(void* pMem_p, size_t memSize_p)
{
    UNUSED_PARAMETER(pMem_p);
    UNUSED_PARAMETER(memSize_p);

    if (hFileHandle_l == NULL)
        return kErrorNoResource;

    return kErrorOk;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

/// \}
