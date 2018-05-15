/**
********************************************************************************
\file   common/memmap.h

\brief  Definitions for memory mapping library

This file contains the definitions for the memory mapping library.
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
#ifndef _INC_common_memmap_H_
#define _INC_common_memmap_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <oplk/oplkinc.h>

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// typedef
//------------------------------------------------------------------------------
/**
\brief Return codes of the memory mapping library

The enumeration defines the codes returned by functions of the memory mapping
library.
*/
typedef enum
{
    kMemMapOk               =  0,   ///< No error/successful run
    kMemMapInvalidArg       =  9,   ///< There was an invalid argument given
    kMemMapNoResource       = 20    ///< The resource could not be created
} eMemMapReturn;

/**
\brief Memory map return code data type

Data type for the enumerator \ref eMemMapReturn.
*/
typedef UINT32 tMemMapReturn;

//------------------------------------------------------------------------------
// function prototypes
//------------------------------------------------------------------------------
#ifdef __cplusplus
extern "C"
{
#endif

tMemMapReturn memmap_init(void);
tMemMapReturn memmap_shutdown(void);

void*         memmap_mapKernelBuffer(const void* pKernelBuffer_p, size_t bufferSize_p);
void          memmap_unmapKernelBuffer(const void* pBuffer_p);

#ifdef __cplusplus
}
#endif

#endif /* _INC_common_memmap_H_ */
