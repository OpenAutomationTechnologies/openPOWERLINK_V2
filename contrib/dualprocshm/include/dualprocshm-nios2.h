/**
********************************************************************************
\file   dualprocshm-nios2.h

\brief  Dual processor Library Target support header - For NIOS2 target

This header file provides specific macros for Altera NIOS2 CPU.

*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2015, Kalycito Infotech Private Limited
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
#ifndef _INC_dualprocshm_nios2_H_
#define _INC_dualprocshm_nios2_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <stdint.h>         // For uint*_t
#include <stdlib.h>         // For malloc/free
#include <string.h>         // For memset/memcpy
#include <stddef.h>
#include <stdio.h>
#include <unistd.h>
#include <alt_types.h>
#include <sys/alt_cache.h>
#include <sys/alt_irq.h>
#include <io.h>
#include <targetsection.h>
#include <dualprocshm-mem.h>

/// borrowed from alt_remap_uncached.c
#ifdef NIOS2_MMU_PRESENT
#define NIOS2_BYPASS_DCACHE_MASK    (0x1 << 29)
#else
#define NIOS2_BYPASS_DCACHE_MASK    (0x1 << 31)
#endif

// Memory
#define DPSHM_MAKE_NONCACHEABLE(ptr) \
    (void*)(((unsigned long)ptr) | NIOS2_BYPASS_DCACHE_MASK)

#define DUALPROCSHM_MALLOC(size)                (void*)alt_uncached_malloc(size)
#define DUALPROCSHM_FREE(ptr)                   alt_uncached_free(ptr)
#define DUALPROCSHM_MEMCPY(dest, src, siz)      memcpy((dest), (src), (siz))
#define DPSHM_UNREG_SYNC_INTR(callback, arg)
#define DPSHM_CLEAR_SYNC_IRQ()

// IO operations
#define DPSHM_READ8(base)               IORD_8DIRECT((UINT32)base, 0)
#define DPSHM_WRITE8(base, val)         IOWR_8DIRECT((UINT32)base, 0, val)
#define DPSHM_READ16(base)              IORD_16DIRECT((UINT32)base, 0)
#define DPSHM_WRITE16(base, val)        IOWR_16DIRECT((UINT32)base, 0, val)
#define DPSHM_READ32(base)              IORD_32DIRECT((UINT32)base, 0)
#define DPSHM_WRITE32(base, val)        IOWR_32DIRECT((UINT32)base, 0, val)

#ifdef __INT_BUS__

#define DPSHM_CONNECT_SYNC_IRQ()
#define DPSHM_DISCONNECT_SYNC_IRQ()

#elif defined(__PCIE__)

#define DPSHM_CONNECT_SYNC_IRQ()    \
                    DPSHM_WRITE8(TARGET_SYNC_INT_BASE, 0x1)
#define DPSHM_DISCONNECT_SYNC_IRQ()   \
                    DPSHM_WRITE8(TARGET_SYNC_INT_BASE, 0x0)

#else
#error "Currently only Internal Bus between shared memory and driver is supported!!"
#endif

// Memory barrier
#define DPSHM_DMB()                     __asm("sync")

// Cache handling. NIOS2 supports only uncached memory regions.
#define DUALPROCSHM_FLUSH_DCACHE_RANGE(base, range) \
    ((void)0)

#define DUALPROCSHM_INVALIDATE_DCACHE_RANGE(base, range) \
    ((void)0)

#define DPSHM_REG_SYNC_INTR(pfnIrqCb_p, pArg_p) \
    UNUSED_PARAMETER(pfnIrqCb_p);               \
    UNUSED_PARAMETER(pArg_p)

#define DPSHM_ENABLE_SYNC_INTR() \
        alt_ic_irq_enable(TARGET_SYNC_IRQ_ID, TARGET_SYNC_IRQ)

#define DPSHM_DISABLE_SYNC_INTR() \
        alt_ic_irq_disable(TARGET_SYNC_IRQ_ID, TARGET_SYNC_IRQ)

#endif /* _INC_dualprocshm_nios2_H_ */
