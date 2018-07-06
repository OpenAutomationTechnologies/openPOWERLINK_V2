/**
********************************************************************************
\file   oplk/targetdefs/windows.h

\brief  Target definitions for Windows

This file contains target specific definitions for Windows.
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2018, Kalycito Infotech Private Limited
Copyright (c) 2016, B&R Industrial Automation GmbH
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
#ifndef _INC_oplk_targetdefs_windows_H_
#define _INC_oplk_targetdefs_windows_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

// Windows version must be at least Windows Vista
#if (defined(_WIN32_WINNT) && (_WIN32_WINNT < 0x0600))
#undef _WIN32_WINNT
#endif
#if !defined(_WIN32_WINNT)
#define _WIN32_WINNT 0x0600
#endif

// Do not use extended Win32 API functions
#if !defined(WIN32_LEAN_AND_MEAN)
#define WIN32_LEAN_AND_MEAN
#endif
#include <Windows.h>

#include <oplk/basictypes.h>

#define INLINE

#define OPLK_FILE_HANDLE        HANDLE

#define UNUSED_PARAMETER(par)   (void)par

// QWORD will not be set for windows
#ifndef QWORD
#define QWORD unsigned long long int
#endif

// Redefine TRUE/FALSE if necessary
#ifdef FALSE
#undef FALSE
#endif
#define FALSE 0x00

#ifdef TRUE
#undef TRUE
#endif
#define TRUE 0xFF

#ifdef _CONSOLE // use standard printf in console applications
#define PRINTF(...)             printf(__VA_ARGS__)
#else           // use trace for output in debug window in Windows applications
#define PRINTF(...)             TRACE(__VA_ARGS__)
#endif

#if defined(_DLL)
#define OPLKDLLEXPORT extern __declspec(dllexport)
#else /* defined(_DLL) */
#define OPLKDLLEXPORT
#endif /* defined(_DLL) */

// Target IO functions
// - Write
#define OPLK_IO_WR8(addr, val)      (*(volatile UINT8*)(addr)) = (val)
#define OPLK_IO_WR16(addr, val)     (*(volatile UINT16*)(addr)) = (val)
#define OPLK_IO_WR32(addr, val)     (*(volatile UINT32*)(addr)) = (val)
// - Read
#define OPLK_IO_RD8(addr)           (*(volatile UINT8*)(addr))
#define OPLK_IO_RD16(addr)          (*(volatile UINT16*)(addr))
#define OPLK_IO_RD32(addr)          (*(volatile UINT32*)(addr))

// Target data cache functions
#define OPLK_DCACHE_FLUSH(addr, len)        ((void)0)
#define OPLK_DCACHE_INVALIDATE(addr, len)   ((void)0)

// Target memory barrier function
#define OPLK_MEMBAR()               ((void)0)

// Target lock
#define OPLK_LOCK_T                 UINT8

#ifdef CONFIG_PCIE
#define ATOMIC_MEM_OFFSET       0x80000 //TODO@gks: Retrieve the Atomic memory base address from PCIe headers

#define OPLK_ATOMIC_T           UINT8
#define OPLK_ATOMIC_INIT(base)
#define OPLK_ATOMIC_EXCHANGE(address, newval, oldval) \
                        OPLK_IO_WR8((address + ATOMIC_MEM_OFFSET), newval); \
                        oldval = OPLK_IO_RD8((address + ATOMIC_MEM_OFFSET))
#else /* CONFIG_PCIE */
#define OPLK_ATOMIC_T           ULONG
#define OPLK_ATOMIC_EXCHANGE(address, newval, oldval) \
            oldval = InterlockedExchange(address, newval);
#endif /* CONFIG_PCIE */

#define OPLK_MUTEX_T    HANDLE

#endif /* _INC_oplk_targetdefs_windows_H_ */
