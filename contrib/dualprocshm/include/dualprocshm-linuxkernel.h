/**
********************************************************************************
\file   dualprocshm-linuxkernel.h

\brief  Dual processor library - target support header for Linux kernel target

This header file provides specific macros for dual processor shared memory
interface in Linux kernel.

*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2015 Kalycito Infotech Private Limited
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

#ifndef _INC_dualprocshm_linuxkernel_H_
#define _INC_dualprocshm_linuxkernel_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------

#include <linux/types.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <asm/io.h>
#include <linux/gfp.h>
#include <linux/slab.h>

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

#ifndef TRACE
#ifndef NDEBUG
#define TRACE(...)                              printk(__VA_ARGS__)
#else
#define TRACE(...)
#endif
#endif

/// memory
#define DPSHM_MAKE_NONCACHEABLE(ptr)            (void*)(((unsigned long)ptr))
#define DUALPROCSHM_MALLOC(size)                kmalloc(size, GFP_KERNEL)
#define DUALPROCSHM_FREE(ptr)                   kfree(ptr)
#define DUALPROCSHM_MEMCPY(dest, src, siz)      memcpy(dest, src, siz)

/// IO operations
#define DPSHM_READ8(base)                       readb((UINT8*)base)
#define DPSHM_WRITE8(base, val)                 writeb(val, (UINT8*)base)
#define DPSHM_READ16(base)                      readw((UINT16*)base)
#define DPSHM_WRITE16(base, val)                writew(val, (UINT16*)base)
#define DPSHM_READ32(base)                      readl((UINT32*)base)
#define DPSHM_WRITE32(base, val)                writel(val, (UINT32*)base)
#define DPSHM_ENABLE_INTR(fEnable)              // No operations on Linux host
#define DPSHM_DMB()                             mb()

/// cache handling
#define DUALPROCSHM_FLUSH_DCACHE_RANGE(base, range)

#define DUALPROCSHM_INVALIDATE_DCACHE_RANGE(base, range)

#define DPSHM_REG_SYNC_INTR(callback, arg)      // No operation on Linux host as irq is handled through the PCIe driver
#define DPSHM_UNREG_SYNC_INTR(callback, arg)    // No operation on Linux host

#define DPSHM_ENABLE_SYNC_INTR()                // Not used on Linux host
#define DPSHM_DISABLE_SYNC_INTR()               // Not used on Linux host
#define DPSHM_CLEAR_SYNC_IRQ()                  // No operation on Linux host

#define DPSHM_CONNECT_SYNC_IRQ()                // No operations on Linux host
#define DPSHM_DISCONNECT_SYNC_IRQ()             // No operations on Linux host

//------------------------------------------------------------------------------
// typedef
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// function prototypes
//------------------------------------------------------------------------------

#endif /* _INC_dualprocshm_linuxkernel_H_ */
