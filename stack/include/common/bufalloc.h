/**
********************************************************************************
\file   common/bufalloc.h

\brief  Definitions for buffer allocation library

This file contains the definitions for the buffer allocation library.

*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2016, B&R Industrial Automation GmbH
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
#ifndef _INC_common_bufalloc_H_
#define _INC_common_bufalloc_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <common/oplkinc.h>

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// typedef
//------------------------------------------------------------------------------
/**
\brief Structure describing the buffer data

This structure describes the buffer data used in the buffer allocation stack.
*/
typedef struct
{
    UINT        bufferNumber;               ///< Index of buffer
    void*       pBuffer;                    ///< Buffer pointer
} tBufData;

/**
\brief Structure describing the buffer allocation stack

This structure describes a stack-based buffer allocation.
*/
typedef struct
{
    char        checkId[8];                 ///< Verification string
    UINT        maxSize;                    ///< Maximum amount of buffers
    UINT        releasedBufCnt;             ///< Counter of released buffers
    UINT        allocatedBufCnt;            ///< Counter of allocated buffers
    tBufData*   pBufDataBegin;              ///< Pointer to the start of the buffer array
    tBufData*   pBufData;                   ///< Pointer to the next available buffer
} tBufAlloc;

//------------------------------------------------------------------------------
// function prototypes
//------------------------------------------------------------------------------
#ifdef __cplusplus
extern "C"
{
#endif

tBufAlloc* bufalloc_init(UINT maxBuffer_p);
void       bufalloc_exit(tBufAlloc* pBufAlloc_p);
tOplkError bufalloc_addBuffer(tBufAlloc* pBufAlloc_p, const tBufData* pBufData_p);
tOplkError bufalloc_getBuffer(tBufAlloc* pBufAlloc_p, tBufData* pBufData_p);
tOplkError bufalloc_releaseBuffer(tBufAlloc* pBufAlloc_p, const tBufData* pBufData_p);

#ifdef __cplusplus
}
#endif

#endif /* _INC_common_bufalloc_H_ */
