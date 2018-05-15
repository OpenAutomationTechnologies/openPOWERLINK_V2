/**
********************************************************************************
\file   memmap/memmap-linuxdpshm.c

\brief  Memory mapping implementation for openPOWERLINK PCIe driver on Linux

This file contains the architecture specific memory mapping implementation
for Linux systems using openPOWERLINK PCIe driver.

\ingroup module_lib_memmap
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2017, Kalycito Infotech Private Limited.
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
#include <common/memmap.h>
#include <user/ctrlucal.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/mman.h>
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
/**
\brief Structure holding the memory parameters

The structure holds the kernel pointer, the mapped user memory pointer, mapping
size and byte offset of mapped buffer in the mapped region.
*/
typedef struct
{
    size_t      memSize;    ///< Size of the last mapped buffer.
    size_t      offset;     ///< Offset of the mapped buffer inside the vma.
    void*       pKernelBuf; ///< PCP address corresponding to the last mapped vma start address.
    void*       pUserBuf;   ///< Start address of the last mapped vma.
} tMemmapInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static OPLK_FILE_HANDLE fd_l;
static tMemmapInstance  memmapInstance_l;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Initialize memory mapping

The function initializes the memory mapping service.

\return The function returns a tMemMapReturn error code.

\ingroup module_lib_memmap
*/
//------------------------------------------------------------------------------
tMemMapReturn memmap_init(void)
{
    fd_l = ctrlucal_getFd();
    OPLK_MEMSET(&memmapInstance_l, 0, sizeof(memmapInstance_l));

    return kMemMapOk;
}

//------------------------------------------------------------------------------
/**
\brief  Shut down memory mapping

The function shuts down the memory mapping service.

\return The function returns a tMemMapReturn error code.

\ingroup module_lib_memmap
*/
//------------------------------------------------------------------------------
tMemMapReturn memmap_shutdown(void)
{
    fd_l = (OPLK_FILE_HANDLE)0;

    return kMemMapOk;
}

//------------------------------------------------------------------------------
/**
\brief  Map kernel buffer

The function maps a kernel layer buffer address into user application
virtual address space which is of the size specified by \p bufferSize_p.

\param[in]      pKernelBuffer_p     The pointer to the kernel buffer.
\param[in]      bufferSize_p        The size of the kernel buffer.

\return The functions returns the pointer to the mapped kernel buffer.

\ingroup module_lib_memmap
*/
//------------------------------------------------------------------------------
void* memmap_mapKernelBuffer(const void* pKernelBuffer_p, size_t bufferSize_p)
{
    void* pMappedBuffer = NULL;

    // Check parameter validity
    ASSERT(pKernelBuffer_p != NULL);

    memmapInstance_l.memSize = bufferSize_p;

    // Find the page aligned address before the buffer to be remapped and store
    // the byte offset of the buffer from the page boundary.
    memmapInstance_l.pKernelBuf = (void*)((ULONG)pKernelBuffer_p &
                                           ~(sysconf(_SC_PAGE_SIZE) - 1));
    memmapInstance_l.offset = (size_t)((ULONG)pKernelBuffer_p & (sysconf(_SC_PAGE_SIZE) - 1));

    memmapInstance_l.pUserBuf = mmap(NULL,                       // Map at any address in vma
                                     memmapInstance_l.memSize + 2 * sysconf(_SC_PAGE_SIZE),
                                     PROT_READ | PROT_WRITE,     // Map as read and write memory
                                     MAP_SHARED,                 // Map as shared memory, $$ private is enough for us
                                     fd_l,                       // file descriptor
                                     (ULONG)memmapInstance_l.pKernelBuf);
    if (memmapInstance_l.pUserBuf == MAP_FAILED)
    {
        DEBUG_LVL_ERROR_TRACE("%s() mmap failed!\n", __func__);
        pMappedBuffer = NULL;
    }
    else
    {
        // Add the byte offset of the buffer back to the mapped page
        pMappedBuffer = (UINT8*)memmapInstance_l.pUserBuf + memmapInstance_l.offset;
    }

    return pMappedBuffer;
}

//------------------------------------------------------------------------------
/**
\brief  Disconnect from a memory mapping

The function disconnects from a memory mapping.

\param[in]      pBuffer_p           The pointer to the previously mapped buffer.

\ingroup module_lib_memmap
*/
//------------------------------------------------------------------------------
void memmap_unmapKernelBuffer(const void* pBuffer_p)
{
    // Check parameter validity
    ASSERT(pBuffer_p != NULL);

    if ((UINT8*)memmapInstance_l.pUserBuf + memmapInstance_l.offset != (UINT8*)pBuffer_p)
    {
        DEBUG_LVL_ERROR_TRACE("%s() called with unknown memory address\n",
                              __func__);
    }

    // Unmap the actual mapped vma i.e. the page aligned address range
    if (munmap(memmapInstance_l.pUserBuf,
               memmapInstance_l.memSize + 2 * sysconf(_SC_PAGE_SIZE)) != 0)
    {
        DEBUG_LVL_ERROR_TRACE("%s() munmap failed (%s)\n",
                              __func__, strerror(errno));
    }

    OPLK_MEMSET(&memmapInstance_l, 0, sizeof(memmapInstance_l));
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

/// \}
