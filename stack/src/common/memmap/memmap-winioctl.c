/**
********************************************************************************
\file   memmap-winioctl.c

\brief  Memory mapping implementation for IOCTL interface on Windows

This file contains the implementation of the memory mapping module which uses
IOCTLs to communicate with openPOWERLINK kernel layer.

This module enables the user application in Windows to map a specific memory of
the openPOWERLINK kernel stack into Windows user space.

The module uses Windows IOCTLs to get the kernel memory which is mapped in Windows
user space by the kernel driver.

\ingroup module_lib_memmap
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2015, Kalycito Infotech Private Limited
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
#include <common/target.h>
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
/**
\brief Memory mapping instance

Structure to hold all local parameters used by memory mapping module.

*/
typedef struct
{
    OPLK_FILE_HANDLE    hFileHandle;    ///< File handle for the openPOWERLINK driver.
    tMemStruc           memStruc;       ///< Shared memory structure to exchange addresses.
} tMemMapInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tMemMapInstance  memMapInstance_l;

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
    ULONG       bytesReturned;
    tMemStruc   inMemStruc;
    tMemStruc*  pOutMemStruc = &memMapInstance_l.memStruc;

    OPLK_MEMSET(&inMemStruc, 0, sizeof(inMemStruc));

    memMapInstance_l.hFileHandle = ctrlucal_getFd();

    if (memMapInstance_l.hFileHandle == NULL)
        return kMemMapNoResource;

    if (!DeviceIoControl(memMapInstance_l.hFileHandle,
                         PLK_CMD_MAP_MEM,
                         &inMemStruc,
                         sizeof(tMemStruc),
                         pOutMemStruc,
                         sizeof(tMemStruc),
                         &bytesReturned,
                         NULL))
    {
        return kMemMapNoResource;
    }

    if ((bytesReturned == 0) || (pOutMemStruc->pUserAddr == NULL))
        return kMemMapNoResource;

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
    tMemStruc*  pMemStruc = &memMapInstance_l.memStruc;
    ULONG       bytesReturned;

    if (!DeviceIoControl(memMapInstance_l.hFileHandle,
                         PLK_CMD_UNMAP_MEM,
                         pMemStruc,
                         sizeof(tMemStruc),
                         NULL,
                         0,
                         &bytesReturned,
                         NULL))
    {
        DEBUG_LVL_ERROR_TRACE("%s() Unable to free mem %d\n", __func__, GetLastError());
        return kErrorGeneralError;
    }

    pMemStruc->pKernelAddr = NULL;
    pMemStruc->pUserAddr = NULL;
    pMemStruc->size = 0;
    memMapInstance_l.hFileHandle = NULL;

    return kMemMapOk;
}

//------------------------------------------------------------------------------
/**
\brief  Map kernel buffer

The function maps a kernel buffer address.

\param[in]      pKernelBuffer_p     The pointer to the kernel buffer.
\param[in]      bufferSize_p        The size of the kernel buffer.

\return The functions returns the pointer to the mapped kernel buffer.

\ingroup module_lib_memmap
*/
//------------------------------------------------------------------------------
void* memmap_mapKernelBuffer(const void* pKernelBuffer_p, size_t bufferSize_p)
{
    ptrdiff_t   offset;
    void*       pUserAddr;
    tMemStruc*  pMemStruc = &memMapInstance_l.memStruc;

    // Check parameter validity
    ASSERT(pKernelBuffer_p != NULL);

    // Negative offset not possible
    if ((pKernelBuffer_p < pMemStruc->pKernelAddr) ||
        (((const UINT8*)pKernelBuffer_p + bufferSize_p) >
        ((const UINT8*)pMemStruc->pKernelAddr + pMemStruc->size)))
        return NULL;

    offset = (const UINT8*)pKernelBuffer_p - (const UINT8*)pMemStruc->pKernelAddr;
    pUserAddr = (UINT8*)pMemStruc->pUserAddr + offset;

    return pUserAddr;
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
    UNUSED_PARAMETER(pBuffer_p);
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

/// \}
