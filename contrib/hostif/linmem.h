/**
********************************************************************************
\file   linmem.h

\brief  This is the interface description of a linear memory buffer.

The linear memory buffer module enables a shared memory region for multiple
processes. Note that there is no data corruption protection provided.

*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2012, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
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

#ifndef _INC_LINMEM_H_
#define _INC_LINMEM_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include "hostiflib_target.h"

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// typedef
//------------------------------------------------------------------------------
/**
\brief Linear memory instance return type

The interface provides a specific error code.
*/
typedef enum eLimReturn
{
    kLimSuccessful          = 0x0,
    kLimInvalidParameter    = 0x1,
    kLimInvalidInstance     = 0x2,
    kLimNoResource          = 0x3,
    kLimAlignment           = 0x4,
    kLimOverflow            = 0x5,

} tLimReturn;

/**
\brief Linear memory instance configuration

The linear memory creation is directed by the parameters in this structure.
*/
typedef struct sLimConfig
{
    BOOL            fAllocHeap;
    UINT8           *pBase;
    UINT16          span;
} tLimConfig;

typedef void* tLimInstance;

//------------------------------------------------------------------------------
// function prototypes
//------------------------------------------------------------------------------

#ifdef __cplusplus
extern "C" {
#endif

tLimReturn lim_create (tLimConfig *pConfig, tLimInstance *ppInstance_p);
tLimReturn lim_delete (tLimInstance pInstance_p);

tLimReturn lim_getBase (tLimInstance pInstance_p, UINT8 **ppBase_p);
tLimReturn lim_getSpan (tLimInstance pInstance_p, UINT16 *pSpan_p);

tLimReturn lim_write (tLimInstance pInstance_p, UINT16 offset_p,
        UINT8 *pSrc_p, UINT16 size_p);
tLimReturn lim_read (tLimInstance pInstance_p, UINT16 offset_p,
        UINT8 *pDst_p, UINT16 size_p);

#ifdef __cplusplus
}
#endif

#endif /* _INC_LINMEM_H_ */
