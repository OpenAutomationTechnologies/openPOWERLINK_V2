/**
********************************************************************************
\file   common/ami.h

\brief  Definitions for the abstract memory interface (ami)

*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2014, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
Copyright (c) 2013, SYSTEC electronic GmbH
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

#ifndef _INC_common_ami_H_
#define _INC_common_ami_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <common/oplkinc.h>

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

// Conversion macros for datatype UINT8 (saves code size)
#define ami_setUint8Be(pAddr_p, uint8Val_p) {*(UINT8*)(pAddr_p) = (uint8Val_p);}
#define ami_setUint8Le(pAddr_p, uint8Val_p) {*(UINT8*)(pAddr_p) = (uint8Val_p);}

#define ami_getUint8Be(pAddr_p) (*(UINT8*)(pAddr_p))
#define ami_getUint8Le(pAddr_p) (*(UINT8*)(pAddr_p))

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

// Conversion functions for datatype WORD
void ami_setUint16Be(void* pAddr_p, UINT16 uint16Val_p);
void ami_setUint16Le(void* pAddr_p, UINT16 uint16Val_p);

UINT16 ami_getUint16Be(void* pAddr_p);
UINT16 ami_getUint16Le(void* pAddr_p);

// Conversion functions for datatype DWORD24
void ami_setUint24Be(void* pAddr_p, UINT32 uint32Val_p);
void ami_setUint24Le(void* pAddr_p, UINT32 uint32Val_p);

UINT32 ami_getUint24Be(void* pAddr_p);
UINT32 ami_getUint24Le(void* pAddr_p);

// Conversion functions for datatype DWORD
void ami_setUint32Be(void* pAddr_p, UINT32 uint32Val_p);
void ami_setUint32Le(void* pAddr_p, UINT32 uint32Val_p);

UINT32 ami_getUint32Be(void* pAddr_p);
UINT32 ami_getUint32Le(void* pAddr_p);

// Conversion functions for datatype QWORD40
void ami_setUint40Be(void* pAddr_p, UINT64 uint64Val_p);
void ami_setUint40Le(void* pAddr_p, UINT64 uint64Val_p);

UINT64 ami_getUint40Be(void* pAddr_p);
UINT64 ami_getUint40Le(void* pAddr_p);

// Conversion functions for datatype QWORD48
void ami_setUint48Be(void* pAddr_p, UINT64 uint64Val_p);
void ami_setUint48Le(void* pAddr_p, UINT64 uint64Val_p);

UINT64 ami_getUint48Be(void* pAddr_p);
UINT64 ami_getUint48Le(void* pAddr_p);

// Conversion functions for datatype QWORD56
void ami_setUint56Be(void* pAddr_p, UINT64 uint64Val_p);
void ami_setUint56Le(void* pAddr_p, UINT64 uint64Val_p);

UINT64 ami_getUint56Be(void* pAddr_p);
UINT64 ami_getUint56Le(void* pAddr_p);

// Conversion functions for datatype QWORD
void ami_setUint64Be(void* pAddr_p, UINT64 uint64Val_p);
void ami_setUint64Le(void* pAddr_p, UINT64 uint64Val_p);

UINT64 ami_getUint64Be(void* pAddr_p);
UINT64 ami_getUint64Le(void* pAddr_p);

// Conversion functions for type tTimeOfDay
void ami_setTimeOfDay(void* pAddr_p, tTimeOfDay* pTimeOfDay_p);
void ami_getTimeOfDay(void* pAddr_p, tTimeOfDay* pTimeOfDay_p);

#ifdef __cplusplus
}
#endif


#endif /* _INC_common_ami_H_ */
