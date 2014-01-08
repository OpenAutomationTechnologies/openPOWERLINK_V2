/**
********************************************************************************
\file   ami.h

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

#ifndef _INC_ami_H_
#define _INC_ami_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------

#include <EplInc.h>

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

// Conversion macros for datatype UINT8 (saves code size)
#define AmiSetByteToBe(pAddr_p, uint8Val_p) {*(UINT8 *)(pAddr_p) = (uint8Val_p);}
#define AmiSetByteToLe(pAddr_p, uint8Val_p) {*(UINT8 *)(pAddr_p) = (uint8Val_p);}

#define AmiGetByteFromBe(pAddr_p) (*(UINT8 *)(pAddr_p))
#define AmiGetByteFromLe(pAddr_p) (*(UINT8 *)(pAddr_p))

//------------------------------------------------------------------------------
// typedef
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// function prototypes
//------------------------------------------------------------------------------

#ifdef __cplusplus
    extern "C" {
#endif

// Conversion functions for datatype WORD
void AmiSetWordToBe(void* pAddr_p, UINT16 uint16Val_p);
void AmiSetWordToLe(void* pAddr_p, UINT16 uint16Val_p);

UINT16 AmiGetWordFromBe(void* pAddr_p);
UINT16 AmiGetWordFromLe(void* pAddr_p);

// Conversion functions for datatype DWORD24
void AmiSetDword24ToBe(void* pAddr_p, UINT32 uint32Val_p);
void AmiSetDword24ToLe(void* pAddr_p, UINT32 uint32Val_p);

UINT32 AmiGetDword24FromBe(void* pAddr_p);
UINT32 AmiGetDword24FromLe(void* pAddr_p);

// Conversion functions for datatype DWORD
void AmiSetDwordToBe(void* pAddr_p, UINT32 uint32Val_p);
void AmiSetDwordToLe(void* pAddr_p, UINT32 uint32Val_p);

UINT32 AmiGetDwordFromBe(void* pAddr_p);
UINT32 AmiGetDwordFromLe(void* pAddr_p);

// Conversion functions for datatype QWORD40
void AmiSetQword40ToBe(void* pAddr_p, UINT64 uint64Val_p);
void AmiSetQword40ToLe(void* pAddr_p, UINT64 uint64Val_p);

UINT64 AmiGetQword40FromBe(void* pAddr_p);
UINT64 AmiGetQword40FromLe(void* pAddr_p);

// Conversion functions for datatype QWORD48
void AmiSetQword48ToBe(void* pAddr_p, UINT64 uint64Val_p);
void AmiSetQword48ToLe(void* pAddr_p, UINT64 uint64Val_p);

UINT64 AmiGetQword48FromBe(void* pAddr_p);
UINT64 AmiGetQword48FromLe(void* pAddr_p);

// Conversion functions for datatype QWORD56
void AmiSetQword56ToBe(void* pAddr_p, UINT64 uint64Val_p);
void AmiSetQword56ToLe(void* pAddr_p, UINT64 uint64Val_p);

UINT64 AmiGetQword56FromBe(void* pAddr_p);
UINT64 AmiGetQword56FromLe(void* pAddr_p);

// Conversion functions for datatype QWORD
void AmiSetQword64ToBe(void* pAddr_p, UINT64 uint64Val_p);
void AmiSetQword64ToLe(void* pAddr_p, UINT64 uint64Val_p);

UINT64 AmiGetQword64FromBe(void* pAddr_p);
UINT64 AmiGetQword64FromLe(void* pAddr_p);

// Conversion functions for type tTimeOfDay
void AmiSetTimeOfDay(void* pAddr_p, tTimeOfDay* pTimeOfDay_p);
void AmiGetTimeOfDay(void* pAddr_p, tTimeOfDay* pTimeOfDay_p);

#ifdef __cplusplus
    }
#endif


#endif /* _INC_ami_H_ */
