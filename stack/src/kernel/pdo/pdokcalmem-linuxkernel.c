/**
********************************************************************************
\file   pdokcalmem-linuxkernel.c

\brief  PDO kernel CAL shared-memory module using the openPOWERLINK Linux kernel driver

This file contains an implementation for the kernel PDO CAL module which uses
the Linux kernel driver to provide its kernel memory to the user layer by
the mmap device operation.

\ingroup module_pdokcal
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
//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <common/oplkinc.h>
#include <kernel/pdokcal.h>

#include <linux/slab.h>

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

For the linux kernel mmap implementation nothing needs to be done.

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

For the linux kernel mmap implementation nothing needs to be done.

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

\param  memSize_p               Size of PDO memory
\param  ppPdoMem_p              Pointer to store the PDO memory pointer

\return The function returns a tOplkError error code.

\ingroup module_pdokcal
*/
//------------------------------------------------------------------------------
tOplkError pdokcal_allocateMem(size_t memSize_p, BYTE** ppPdoMem_p)
{
    ULONG         order;

    order = get_order(memSize_p);
    if ((*ppPdoMem_p = (BYTE*)__get_free_pages(GFP_KERNEL, order)) == NULL)
    {
        return kErrorNoResource;
    }
    DEBUG_LVL_PDO_TRACE("%s() Allocated memory for PDO at %p size:%d/%d\n",
                        __func__, *ppPdoMem_p, memSize_p, order);
    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Free PDO shared memory

The function frees shared memory which was allocated in the kernel layer for
transfering the PDOs.

\param  pMem_p                  Pointer to the shared memory segment
\param  memSize_p               Size of PDO memory

\return The function returns a tOplkError error code.

\ingroup module_pdokcal
*/
//------------------------------------------------------------------------------
tOplkError pdokcal_freeMem(BYTE* pMem_p, size_t memSize_p)
{
    ULONG         order;

    order = get_order(memSize_p);
    free_pages((ULONG)pMem_p, order);
    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Map PDO shared memory

This routine maps the PDO memory allocated in the kernel layer of the openPOWERLINK
stack. This allows user stack to access the PDO memory directly.

\param  ppKernelMem_p           Double pointer to the shared memory segment in kernel space.
\param  ppUserMem_p             Double pointer to the shared memory segment in user space.
\param  pMemSize_p              Pointer to size of PDO memory.

\return The function returns a tOplkError error code.

\ingroup module_pdokcal
*/
//------------------------------------------------------------------------------
tOplkError pdokcal_mapMem(UINT8** ppKernelMem_p, UINT8** ppUserMem_p, size_t* pMemSize_p)
{
    UNUSED_PARAMETER(ppKernelMem_p);
    UNUSED_PARAMETER(ppUserMem_p);
    UNUSED_PARAMETER(pMemSize_p);

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
    UNUSED_PARAMETER(pMem_p);
    UNUSED_PARAMETER(memSize_p);
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

/// \}
