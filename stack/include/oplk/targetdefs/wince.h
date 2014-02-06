/**
********************************************************************************
\file   targetdefs/wince.h

\brief  Target specific definitions for Windows CE

This file contains target specific definitions for Windows CE systems.
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

#ifndef _INC_targetdefs_wince_H_
#define _INC_targetdefs_wince_H_


#include <oplk/basictypes.h>

#define ROM_INIT                // variables will be initialized directly in ROM (means no copy from RAM in startup)
#define ROM                     // code or variables mapped to ROM (i.e. flash)
                                // usage: CONST BYTE ROM foo = 0x00;
#define HWACC                   // hardware access through external memory (i.e. CAN)

// These types can be adjusted by users to match application requirements. The goal is to
// minimize code memory and maximize speed.
#define GENERIC                 // generic pointer to point to application data
                                // Variables with this attribute can be located in external
                                // or internal data memory.
#define MEM                     // Memory attribute to optimize speed and code of pointer access.

#ifndef NEAR
#define NEAR                // variables mapped to internal data storage location
#endif

#ifndef FAR
#define FAR                 // variables mapped to external data storage location
#endif

#ifndef CONST
#define CONST const         // variables mapped to ROM (i.e. flash)
#endif

#define LARGE

#ifndef NO_QWORD
#ifndef QWORD
//#define QWORD long long int // MSVC .NET can use "long long int" too (like GNU)
#define QWORD __int64
#endif
#endif

#define REENTRANT
#define PUBLIC __cdecl

#ifdef ASSERTMSG
#undef ASSERTMSG
#endif

#define UNUSED_PARAMETER(par)   (void)par

#define __func__ __FUNCTION__

void trace (const char* fmt, ...);
#define PRINTF(...)                 TRACE(__VA_ARGS__)

#ifdef ASSERTMSG
#undef ASSERTMSG

#define ASSERTMSG(expr,string) \
    if (!(expr))\
    { \
    MessageBox (NULL, string, L"Assertion failed", MB_OK | MB_ICONERROR); \
    exit (-1);\
    }

#endif

#if defined(_WINDLL)
#define EPLDLLEXPORT extern __declspec(dllexport)
#else
#define EPLDLLEXPORT
#endif

#endif /* _INC_targetdefs_wince_H_ */
