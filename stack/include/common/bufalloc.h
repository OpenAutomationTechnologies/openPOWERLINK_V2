/**
********************************************************************************
\file   bufalloc.h

\brief  Definitions for buffer allocation library

This file contains the definitions for the buffer allocation library.

*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2015, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
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

#ifndef _INC_bufalloc_H_
#define _INC_bufalloc_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <common/oplkinc.h>
//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#define BUFALLOC_CHECKID                       "bufalloc\0"
//------------------------------------------------------------------------------
// typedef
//------------------------------------------------------------------------------
/**
\brief Structure describing the BufData

This structure describes BufData used in the buffer allocation stack.
*/
typedef struct
{
    UINT    bufferNumber;                   ///< Index of buffer
    UINT8*  pBuffer;                        ///< Buffer pointer
} tBufData;

/**
\brief Structure describing the buffer allocation stack

This structure describes a stack-based buffer allocation.
*/
typedef struct
{
    UINT        maxSize;                    ///< Maximum amount of buffers
    UINT        releasedBufCnt;             ///< Counter of released buffer
    UINT        allocatedBufCnt;            ///< Counter of allocated buffer
    char        checkId[9];                 ///< Verification string
    tBufData*   pBufData;                    ///< Pointer to the array of stored buffers
} tBufAlloc;

//------------------------------------------------------------------------------
// function prototypes
//------------------------------------------------------------------------------

#ifdef __cplusplus
extern "C"
{
#endif

tBufAlloc*  bufalloc_init(UINT maxBuffer_p);
tOplkError  bufalloc_addBuffer(tBufAlloc* pBufAlloc_p, void* pfreeBuf_p, UINT bufferNumber_p);
tBufData*   bufalloc_getBuffer(tBufAlloc* pBufAlloc_p);
tOplkError  bufalloc_releaseBuffer(tBufAlloc* pBufAlloc_p, void* pfreeBuf_p, UINT bufferNumber_p);
tOplkError  bufalloc_exit(tBufAlloc* pBufAlloc_p);

#ifdef __cplusplus
}
#endif

#endif /* _INC_bufalloc_H_ */
