/**
********************************************************************************
\file   hostiflib_nios.h

\brief  Host Interface Library - For Nios II target

This header file provides specific macros for Altera Nios II soft-core CPU.

*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2016, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
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
#ifndef _INC_hostiflib_nios_H_
#define _INC_hostiflib_nios_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <stdint.h>
#include <sys/alt_cache.h>
#include <unistd.h>
#include <io.h>
#include <system.h>

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#ifndef HOSTIF_BASE
// Get hostinterface base address from system.h
#if defined(__HOSTINTERFACE)
#define HOSTIF_BASE                 HOSTINTERFACE_0_BASE
#elif defined(__PRLMASTER)
#define HOSTIF_BASE                 PRLMASTER_0_BASE
#else
#error "Please set HOSTIF_BASE define to base address of host interface!"
#endif
#endif //HOSTIF_BASE

#define HOSTIF_IRQ_IC_ID            0   ///< Irq Controller Id
#define HOSTIF_IRQ                  0   ///< Irq Id

/// borrowed from alt_remap_uncached.c
#ifdef NIOS2_MMU_PRESENT
#define NIOS2_BYPASS_DCACHE_MASK    (0x1 << 29)
#else
#define NIOS2_BYPASS_DCACHE_MASK    (0x1 << 31)
#endif

/// cache
#define HOSTIF_MAKE_NONCACHEABLE(ptr)  \
    (void*)(((unsigned long)ptr)|NIOS2_BYPASS_DCACHE_MASK)

#define HOSTIF_UNCACHED_MALLOC(size)  alt_uncached_malloc(size)
#define HOSTIF_UNCACHED_FREE(ptr)     alt_uncached_free(ptr)

/// hw access
#define HOSTIF_RD32(base)             IORD_32DIRECT(base, 0)
#define HOSTIF_RD16(base)             IORD_16DIRECT(base, 0)
#define HOSTIF_RD8(base)              IORD_8DIRECT(base, 0)

#define HOSTIF_WR32(base, dword)      IOWR_32DIRECT(base, 0, dword)
#define HOSTIF_WR16(base, word)       IOWR_16DIRECT(base, 0, word)
#define HOSTIF_WR8(base, byte)        IOWR_8DIRECT(base, 0, byte)

#define HOSTIF_DCACHE_FLUSH(base, len)
#define HOSTIF_DCACHE_INVALIDATE(base, len)

#define HOSTIF_INLINE inline
//------------------------------------------------------------------------------
// typedef
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// function prototypes
//------------------------------------------------------------------------------

#endif /* _INC_hostiflib_nios_H_ */
