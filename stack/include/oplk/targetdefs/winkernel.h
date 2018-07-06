/**
********************************************************************************
\file   oplk/targetdefs/winkernel.h

\brief  Target definitions for Windows kernel

This file contains target specific definitions for Windows kernel.
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2018, Kalycito Infotech Private Limited
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
#ifndef _INC_oplk_targetdefs_winkernel_H_
#define _INC_oplk_targetdefs_winkernel_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include <ntdef.h>
#include <ndis.h>
#include <oplk/basictypes.h>

#define UNUSED_PARAMETER(par) (void)par

#define INLINE

#define OPLK_FILE_HANDLE        HANDLE

// QWORD will not be set for windows
#ifndef QWORD
#define QWORD unsigned long long int
#endif

#ifndef BYTE
#define BYTE unsigned char
#endif

#ifndef BOOL
#define BOOL unsigned char
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

#define PRINTF(...)    DbgPrint(__VA_ARGS__)

#define OPLKDLLEXPORT

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
#define OPLK_MEMBAR()               KeMemoryBarrier()

// Target lock
#define OPLK_LOCK_T                 UINT8

#define OPLK_ATOMIC_T               ULONG
#define OPLK_ATOMIC_EXCHANGE(address, newval, oldval) \
            oldval = InterlockedExchange(address, newval);

#define OPLK_MEM_TAG                'negO'
#define OPLK_MALLOC(siz)            ExAllocatePool(NonPagedPool, (siz))
#define OPLK_FREE(ptr)              ExFreePool(ptr)
#define OPLK_MEMSET(dst, val, siz)  NdisFillMemory(dst, siz, val)

#define OPLK_MUTEX_T                HANDLE

#endif /* _INC_oplk_targetdefs_winkernel_H_ */
