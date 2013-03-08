/**
********************************************************************************
\file   hostiflib_nios.h

\brief  Host Interface Library - For Nios II target

This header file provides specific macros for Altera Nios II soft-core CPU.

*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2012, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
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

#ifndef _INC_HOSTIF_NIOS_H_
#define _INC_HOSTIF_NIOS_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <stdint.h>
#include <sys/alt_cache.h>
#include <sys/alt_irq.h>
#include <unistd.h>
#include <io.h>
#include <system.h>

// include section header file for special functions in
// tightly-coupled memory
#include <section-nios2.h>

// include generated header file for memory structure and version filed
#include "hostiflib-mem.h"

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

#if defined(HOSTINTERFACE_0_BASE)

#define HOSTIF_PCP_BASE             HOSTINTERFACE_0_BASE
#define HOSTIF_HOST_BASE            HOSTINTERFACE_0_BASE

#define HOSTIF_IRQ_IC_ID            0 //FIXME: obtain from system.h
#define HOSTIF_IRQ                  0 //FIXME: obtain from system.h

#elif (defined(PCP_0_HOSTINTERFACE_0_PCP_BASE) && \
       defined(PCP_0_HOSTINTERFACE_0_HOST_BASE))
/* If one Nios II does Pcp and Host (makes no sense, but why not?) */
#define HOSTIF_PCP_BASE             PCP_0_HOSTINTERFACE_0_PCP_BASE
#define HOSTIF_HOST_BASE            PCP_0_HOSTINTERFACE_0_HOST_BASE

#else

#warning "Host Interface base is assumed! Set the correct address!"

#define HOSTIF_PCP_BASE             0x10000000
#define HOSTIF_HOST_BASE            0x10000000
#define HOSTIF_IRQ_IC_ID            0
#define HOSTIF_IRQ                  0

#endif

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

/// sleep
#define HOSTIF_USLEEP(x)              usleep((useconds_t)x)

/// hw access
#define HOSTIF_RD32(base, offset)         IORD_32DIRECT(base, offset)
#define HOSTIF_RD16(base, offset)         IORD_16DIRECT(base, offset)
#define HOSTIF_RD8(base, offset)          IORD_8DIRECT(base, offset)

#define HOSTIF_WR32(base, offset, dword)  IOWR_32DIRECT(base, offset, dword)
#define HOSTIF_WR16(base, offset, word)   IOWR_16DIRECT(base, offset, word)
#define HOSTIF_WR8(base, offset, byte)    IOWR_8DIRECT(base, offset, byte)

/// irq handling
#define HOSTIF_IRQ_REG(cb, arg)     \
    alt_ic_isr_register(HOSTIF_IRQ_IC_ID, HOSTIF_IRQ, cb, arg, NULL)

#define HOSTIF_IRQ_ENABLE()         \
    alt_ic_irq_enable(HOSTIF_IRQ_IC_ID, HOSTIF_IRQ)

#define HOSTIF_IRQ_DISABLE()        \
    alt_ic_irq_disable(HOSTIF_IRQ_IC_ID, HOSTIF_IRQ)

//------------------------------------------------------------------------------
// typedef
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// function prototypes
//------------------------------------------------------------------------------

#endif /* _INC_HOSTIF_NIOS_H_ */
