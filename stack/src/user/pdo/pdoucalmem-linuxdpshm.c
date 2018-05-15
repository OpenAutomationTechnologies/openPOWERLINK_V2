/**
********************************************************************************
\file   pdoucalmem-linuxdpshm.c

\brief  PDO user CAL shared-memory module for openPOWERLINK Linux PCIe driver

This file contains an implementation for the user PDO CAL module which uses
the openPOWERLINK Linux PCIe driver. The PCIe memory is accessed using ioctl
operations.

\ingroup module_pdoucal
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2017, B&R Industrial Automation GmbH
Copyright (c) 2017, Kalycito Infotech Private Limited
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
#include <user/pdoucal.h>
#include <user/ctrlucal.h>
#include <oplk/targetsystem.h>
#include <common/driver.h>

#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <stddef.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>

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
static OPLK_FILE_HANDLE     fd_l;
static ptrdiff_t            pdoMemOffset_l = 0;

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

For the PCIe driver shared memory access we need to get the device
descriptor of the kernel PCIe interface driver.

\return The function returns a tOplkError error code.

\ingroup module_pdokcal
*/
//------------------------------------------------------------------------------
tOplkError pdoucal_openMem(void)
{
    fd_l = ctrlucal_getFd();

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Close PDO shared memory

The function performs all actions needed to clean up the shared memory at
shutdown.

For the mmap implementation nothing needs to be done, as the memory is only
remapped to the user space.

\return The function returns a tOplkError error code.

\ingroup module_pdokcal
*/
//------------------------------------------------------------------------------
tOplkError pdoucal_closeMem(void)
{
    fd_l = (OPLK_FILE_HANDLE)0;

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
tOplkError pdoucal_allocateMem(size_t memSize_p,
                               void** ppPdoMem_p)
{
    int ret;

    // Check parameter validity
    ASSERT(ppPdoMem_p != NULL);

    *ppPdoMem_p = mmap(NULL,
                       memSize_p + ATOMIC_MEM_OFFSET + getpagesize(),
                       PROT_READ | PROT_WRITE, MAP_SHARED,
                       fd_l,
                       0);
    if (*ppPdoMem_p == MAP_FAILED)
    {
        DEBUG_LVL_ERROR_TRACE("%s() mmap failed!\n", __func__);

        *ppPdoMem_p = NULL;
        return kErrorNoResource;
    }

    ret = ioctl(fd_l, PLK_CMD_PDO_MAP_OFFSET, &pdoMemOffset_l);
    if (ret != 0)
    {
        DEBUG_LVL_ERROR_TRACE("%s() error %d\n", __func__, ret);
        return kErrorNoResource;
    }
    else
        *ppPdoMem_p = (UINT8*)*ppPdoMem_p + pdoMemOffset_l;

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
    // Check parameter validity
    ASSERT(pMem_p != NULL);

    pMem_p = (UINT8*)pMem_p - pdoMemOffset_l;
    if (munmap(pMem_p, memSize_p + getpagesize()) != 0)
    {
        DEBUG_LVL_ERROR_TRACE("%s() munmap failed (%s)\n",
                              __func__,
                              strerror(errno));

        return kErrorGeneralError;
    }

    return kErrorOk;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

/// \}
