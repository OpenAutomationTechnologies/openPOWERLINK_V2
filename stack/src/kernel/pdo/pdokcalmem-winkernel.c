/**
********************************************************************************
\file   pdokcalmem-winkernel.c

\brief  PDO kernel CAL shared-memory module using the Windows kernel module

This file contains the implementation for the kernel PDO CAL module for windows
kernel space.

The PDO memory is allocated in the kernel layer and mapped into user space for
sharing. The access to the user stack is provided by passing the address of the
mapped memory in a IOCTL call.

User and kernel stack access the memory directly once it is initialized and
mapped.

\ingroup module_pdokcal
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2015, Kalycito Infotech Private Limited
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
#include <kernel/pdokcal.h>

#include <ndis.h>
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
/**
/brief PDO CAL local instance

The structure contains the variables used by PDO memory module for providing
memory access.

*/
typedef struct
{
    PMDL      pMdl;                 ///< Memory descriptor list describing the PDO memory.
    size_t    memSize;              ///< Size of PDO memory.
    void*     pKernelVa;            ///< Pointer to PDO memory in kernel space.
    void*     pUserVa;              ///< Pointer to PDO memory mapped in user space.
} tPdoCalInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tPdoCalInstance    instance_l;

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

For the Windows kernel nothing needs to be done.

\return The function returns a tOplkError error code.

\ingroup module_pdokcal
*/
//------------------------------------------------------------------------------
tOplkError pdokcal_openMem(void)
{
    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Close PDO shared memory

The function performs all actions needed to clean up the shared memory at
shutdown.

For the Windows kernel nothing needs to be done.

\return The function returns a tOplkError error code.

\ingroup module_pdokcal
*/
//------------------------------------------------------------------------------
tOplkError pdokcal_closeMem(void)
{
    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Allocate PDO shared memory

The function allocates shared memory for the kernel needed to transfer the PDOs.

\param  memSize_p               Size of PDO memory.
\param  ppPdoMem_p              Pointer to store the PDO memory pointer.

\return The function returns a tOplkError error code.

\ingroup module_pdokcal
*/
//------------------------------------------------------------------------------
tOplkError pdokcal_allocateMem(size_t memSize_p, UINT8** ppPdoMem_p)
{
    instance_l.pKernelVa = OPLK_MALLOC(memSize_p);

    if (instance_l.pKernelVa == NULL)
    {
        DEBUG_LVL_ERROR_TRACE("%s() Unable to allocate PDO memory !\n", __func__);
        return kErrorNoResource;
    }

    *ppPdoMem_p = instance_l.pKernelVa;
    instance_l.memSize = memSize_p;

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Free PDO shared memory

The function frees shared memory which was allocated in the kernel layer for
transferring the PDOs.

\param  pMem_p                  Pointer to the shared memory segment
\param  memSize_p               Size of PDO memory

\return The function returns a tOplkError error code.

\ingroup module_pdokcal
*/
//------------------------------------------------------------------------------
tOplkError pdokcal_freeMem(UINT8* pMem_p, size_t memSize_p)
{
    if (instance_l.pKernelVa != NULL)
    {
        OPLK_FREE(instance_l.pKernelVa);
    }

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Map PDO shared memory

This routine maps the PDO memory allocated in the kernel layer of the openPOWERLINK
stack. This allows user stack to access the PDO memory directly.

\param  ppKernelMem_p           Double pointer to the shared memory segment in kernel space.
\param  ppUserMem_p             Double pointer to the shared memory segment in user space.
\param  memSize_p               Pointer to size of PDO memory.

\return The function returns a tOplkError error code.

\ingroup module_pdokcal
*/
//------------------------------------------------------------------------------
tOplkError pdokcal_mapMem(UINT8** ppKernelMem_p, UINT8** ppUserMem_p, size_t* pMemSize_p)
{
    if (*pMemSize_p > instance_l.memSize)
    {
        DEBUG_LVL_ERROR_TRACE("%s() Higher Memory requested (Kernel-%d User-%d) !\n",
                              __func__, instance_l.memSize, *pMemSize_p);
        *pMemSize_p = 0;
        return kErrorNoResource;
    }

    // Allocate new MDL pointing to PDO memory
    instance_l.pMdl = IoAllocateMdl(instance_l.pKernelVa, instance_l.memSize, FALSE, FALSE,
                                    NULL);

    if (instance_l.pMdl == NULL)
    {
        DEBUG_LVL_ERROR_TRACE("%s() Error allocating MDL !\n", __func__);
        return kErrorNoResource;
    }

    // Update the MDL with physical addresses
    MmBuildMdlForNonPagedPool(instance_l.pMdl);

    // Map the memory in user space and get the address
    instance_l.pUserVa = MmMapLockedPagesSpecifyCache(instance_l.pMdl,      // MDL
                                                      UserMode,             // Mode
                                                      MmCached,             // Caching
                                                      NULL,                 // Address
                                                      FALSE,                // Bug-check?
                                                      NormalPagePriority);  // Priority

    if (instance_l.pUserVa == NULL)
    {
        MmUnmapLockedPages(instance_l.pUserVa, instance_l.pMdl);
        IoFreeMdl(instance_l.pMdl);
        DEBUG_LVL_ERROR_TRACE("%s() Error mapping MDL !\n", __func__);
        return kErrorNoResource;
    }

    *ppKernelMem_p = instance_l.pKernelVa;
    *ppUserMem_p = instance_l.pUserVa;
    *pMemSize_p = instance_l.memSize;

    TRACE("Mapped memory info U:%p K:%p size %x", instance_l.pUserVa,
                                                     (UINT8*)instance_l.pKernelVa,
                                                     instance_l.memSize);
    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Unmap PDO shared memory

Unmap the PDO memory shared with the user layer. The memory will be freed in
pdokcal_freeMem().

\param  pMem_p                  Pointer to the shared memory segment.
\param  memSize_p               Size of PDO memory.

\ingroup module_pdokcal
*/
//------------------------------------------------------------------------------
void pdokcal_unMapMem(UINT8* pMem_p, size_t memSize_p)
{
    if (instance_l.pMdl == NULL)
    {
        DEBUG_LVL_ERROR_TRACE("%s() MDL already deleted !\n", __func__);
        return;
    }

    if (instance_l.pUserVa != NULL)
    {
        MmUnmapLockedPages(instance_l.pUserVa, instance_l.pMdl);
        IoFreeMdl(instance_l.pMdl);
    }

    pMem_p = NULL;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

/// \}
