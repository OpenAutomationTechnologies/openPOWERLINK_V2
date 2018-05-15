/**
********************************************************************************
\file   memmap/memmap-nooshostif.c

\brief  Memory mapping implementation for non-os systems with host interface

This file contains the architecture specific memory mapping implementation
for systems without operating system (e.g. microcontrollers, Nios II, Microblaze)
and host interface IP-Core.

\ingroup module_lib_memmap
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
#include <common/memmap.h>
#include <common/target.h>

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
    tHostifInstance*    pHifInstance;
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
    OPLK_MEMSET(&memMapInstance_l, 0, sizeof(tMemMapInstance));

    memMapInstance_l.pHifInstance = hostif_getInstance(0);

    if (memMapInstance_l.pHifInstance == NULL)
    {
        DEBUG_LVL_ERROR_TRACE("%s Host interface instance not found!\n", __func__);
        return kMemMapNoResource;
    }

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
    tHostifReturn       ret;
    tHostifInstance*    pHif = memMapInstance_l.pHifInstance;
    void*               pBuffer;

    // Check parameter validity
    ASSERT(pKernelBuffer_p != NULL);
    UNUSED_PARAMETER(bufferSize_p);

    ret = hostif_dynBufAcquire(pHif, (UINT32)pKernelBuffer_p, &pBuffer);

    if (ret != kHostifSuccessful)
    {
        DEBUG_LVL_ERROR_TRACE("%s Dynamic buffer acquire failed (%d)\n", __func__, ret);
        pBuffer = NULL;
    }

    return pBuffer;
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
    tHostifReturn       ret;
    tHostifInstance*    pHif = memMapInstance_l.pHifInstance;

    // Check parameter validity
    ASSERT(pBuffer_p != NULL);

    ret = hostif_dynBufFree(pHif, pBuffer_p);

    if (ret != kHostifSuccessful)
    {
        DEBUG_LVL_ERROR_TRACE("%s Dynamic buffer free failed (%d)\n", __func__, ret);
    }
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

/// \}
