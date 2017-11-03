/**
********************************************************************************
\file   circbuf/circbuf-arch.h

\brief  Architecture specific definitions for circular buffer library

This file contains the architecture specific definitions for the circular
buffer library.

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
#ifndef _INC_circbuf_circbuf_arch_H_
#define _INC_circbuf_circbuf_arch_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <common/circbuffer.h>

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// typedef
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// function prototypes
//------------------------------------------------------------------------------

#ifdef __cplusplus
extern "C"
{
#endif

tCircBufInstance* circbuf_createInstance(UINT8 id_p, BOOL fNew_p);
void              circbuf_freeInstance(tCircBufInstance* pInstance_p);
tCircBufError     circbuf_allocBuffer(tCircBufInstance* pInstance_p, size_t* pSize_p);
void              circbuf_freeBuffer(tCircBufInstance* pInstance_p);
tCircBufError     circbuf_connectBuffer(tCircBufInstance* pInstance_p);
void              circbuf_disconnectBuffer(tCircBufInstance* pInstance_p);
void              circbuf_lock(tCircBufInstance* pInstance_p) SECTION_CIRCBUF_LOCK;
void              circbuf_unlock(tCircBufInstance* pInstance_p) SECTION_CIRCBUF_UNLOCK;

#ifdef __cplusplus
}
#endif

#endif /* _INC_circbuf_circbuf_arch_H_ */
