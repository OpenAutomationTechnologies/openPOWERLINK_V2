/**
********************************************************************************
\file   targetdefs/windows.h

\brief  Target definitions for Windows

This file contains target specific defintions for Windows.
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

#ifndef _INC_targetdefs_windows_H_
#define _INC_targetdefs_windows_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#define _WIN32_WINNT 0x0501     // Windows version must be at least Windows XP
#define WIN32_LEAN_AND_MEAN     // Do not use extended Win32 API functions
#include <Windows.h>

#include <oplk/basictypes.h>

#define ROM_INIT                // variables will be initialized directly in ROM (means no copy from RAM in startup)
#define ROM                     // code or variables mapped to ROM (i.e. flash)
                                // usage: CONST BYTE ROM foo = 0x00;

#define MEM                     // Memory attribute to optimize speed and code of pointer access.

#ifndef CONST
#define CONST const             // variables mapped to ROM (i.e. flash)
#endif

#define UNUSED_PARAMETER(par) (void)par

// MS Visual C++ compiler supports function inlining
#define INLINE_FUNCTION_DEF __forceinline

// QWORD will not be set for windows
#ifndef QWORD
#define QWORD unsigned long long int
#endif

// Redefine TRUE/FALSE if necessary
#ifdef FALSE
#undef FALSE
#endif
#define FALSE 0
#ifdef TRUE
#undef TRUE
#endif
#define TRUE 1

#ifdef _CONSOLE // use standard printf in console applications
#define PRINTF(...)                      printf(__VA_ARGS__)
#else           // use trace for output in debug window in Windows applications
#define PRINTF(...)                      TRACE(__VA_ARGS__)
#endif

#ifdef ASSERTMSG
#undef ASSERTMSG
#define ASSERTMSG(expr, string) \
    if (!(expr))\
    { \
        MessageBox(NULL, string, "Assertion failed", MB_OK | MB_ICONERROR); \
        exit(-1); \
    }
#endif

#if defined(_DLL)
#define OPLKDLLEXPORT extern __declspec(dllexport)
#else
#define OPLKDLLEXPORT
#endif

#define OPLK_ATOMIC_T    ULONG
#define OPLK_ATOMIC_EXCHANGE(address, newval, oldval) \
            oldval = InterlockedExchange(address, newval);

#endif /* _INC_targetdefs_windows_H_ */

