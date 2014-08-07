/**
********************************************************************************
\file   cachemem.h

\brief  Definitions for data cache aware memory access

This file contains definitions for data cache aware memory accesses.
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2014, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
Copyright (c) 2014 Kalycito Infotech Private Limited
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

#ifndef _INC_cachemem_H_
#define _INC_cachemem_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------

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
extern "C" {
#endif

unsigned long   cachemem_invAndRead32(unsigned int base_p, unsigned int offset_p);
unsigned short  cachemem_invAndRead16(unsigned int base_p, unsigned int offset_p);
unsigned char   cachemem_invAndRead8(unsigned int base_p, unsigned int offset_p);

void cachemem_writeAndFlush32(unsigned int base_p, unsigned int offset_p, unsigned long val_p);
void cachemem_writeAndFlush16(unsigned int base_p, unsigned int offset_p, unsigned short val_p);
void cachemem_writeAndFlush8(unsigned int base_p, unsigned int offset_p, unsigned char val_p);

#ifdef __cplusplus
}
#endif

#endif /* _INC_cachemem_H_ */
