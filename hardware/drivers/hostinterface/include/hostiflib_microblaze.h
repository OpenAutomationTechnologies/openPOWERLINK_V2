/**
********************************************************************************
\file   hostiflib_microblaze.h

\brief  Host Interface Library - For Microblaze target

This header file provides specific macros for Xilinx Microblaze CPU.

*******************************************************************************/
/*------------------------------------------------------------------------------
Copyright (c) 2014 Kalycito Infotech Private Limited
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
#ifndef _INC_hostiflib_microblaze_H_
#define _INC_hostiflib_microblaze_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <stddef.h>
#include <xil_types.h>
#include <xil_cache.h>
#include <xintc_l.h>
#include <xil_io.h>
#include <xparameters.h>

//=============================================================================
//  F U N C T I O N    P R O T O T Y P E S
//=============================================================================
// TODO: Move this to a target specific file
int     hostiflib_registerHandler(u32 baseAddress_p,
                                  int interruptId_p,
                                  XInterruptHandler handler_p,
                                  void* callBackRef_p);
u32     mb_read32(u32 base_p, u32 offset_p);
u16     mb_read16(u32 base_p, u32 offset_p);
u8      mb_read8(u32 base_p, u32 offset_p);
void    mb_write32(u32 base_p, u32 offset_p, u32 val_p);
void    mb_write16(u32 base_p, u32 offset_p, u16 val_p);
void    mb_write8(u32 base_p, u32 offset_p, u8 val_p);

//=============================================================================
//  H O S T - I N T E R F A C E       D E F I N I T I O N S
//=============================================================================
#if CONFIG_HOSTIF_PCP != FALSE

#define HOSTIF_BASE             XPAR_AXI_HOSTINTERFACE_0_BASEADDR

#else

#ifdef XPAR_AXI_HOSTINTERFACE_0_HOST_BASEADDR

#define HOSTIF_BASE             XPAR_AXI_HOSTINTERFACE_0_HOST_BASEADDR

#ifdef XPAR_HOST_INTC_DEVICE_ID

#define HOSTIF_IRQ_IC_ID        XPAR_HOST_INTC_BASEADDR
#define HOSTIF_IRQ              XPAR_HOST_INTC_AXI_HOSTINTERFACE_0_IRQOUT_IRQ_INTR

#else

#error No Valid Interrupt Controller found for Host!

#endif // XPAR_HOST_INTC_DEVICE_ID

#else
// Parallel Interface Host  External Processor
#warning using external host. Ensure the interface base address is correct
#define HOSTIF_BASE             XPAR_AXI_EPC_0_PRH0_BASEADDR

#ifdef XPAR_HOST_INTC_DEVICE_ID

#define HOSTIF_IRQ_IC_ID        XPAR_HOST_INTC_BASEADDR
#define HOSTIF_IRQ              XPAR_HOST_INTC_SYSTEM_AXI_HOSTINTERFACE_0_IRQOUT_IRQ_PIN_0_INTR

#else

#error No Valid Interrupt Controller found for Host!

#endif  // XPAR_HOST_INTC_DEVICE_ID

#endif  // defined(XPAR_AXI_HOSTINTERFACE_0_HOST_BASEADDR)
#endif  // CONFIG_HOSTIF_PCP != FALSE

#if (XPAR_MICROBLAZE_USE_DCACHE == 1)
#define HOSTIF_SYNC_DCACHE      TRUE
#else
#define HOSTIF_SYNC_DCACHE      FALSE
#endif

#define HOSTIF_MAKE_NONCACHEABLE(ptr)       (void*) ptr

#define HOSTIF_UNCACHED_MALLOC(size)        malloc(size)
#define HOSTIF_UNCACHED_FREE(ptr)           free(ptr)

// sleep
#define HOSTIF_USLEEP(x)                    usleep(x)

// hw access
#define HOSTIF_RD32(base, offset)           mb_read32(base, offset)
#define HOSTIF_RD16(base, offset)           mb_read16(base, offset)
#define HOSTIF_RD8(base, offset)            mb_read8(base, offset)

#define HOSTIF_WR32(base, offset, dword)    mb_write32(base, offset, dword)
#define HOSTIF_WR16(base, offset, word)     mb_write16(base, offset, word)
#define HOSTIF_WR8(base, offset, byte)      mb_write8(base, offset, byte)

#define HOSTIF_IRQ_REG(cb, arg) \
    hostiflib_registerHandler(HOSTIF_IRQ_IC_ID, HOSTIF_IRQ, cb, arg)
#define HOSTIF_IRQ_ENABLE()                                           \
    {                                                                 \
        UINT32    mask = Xil_In32(HOSTIF_IRQ_IC_ID + XIN_IER_OFFSET); \
        mask = (mask | (1 << HOSTIF_IRQ));                            \
        XIntc_EnableIntr(HOSTIF_IRQ_IC_ID, mask);                     \
    }

#define HOSTIF_IRQ_DISABLE()                                          \
    {                                                                 \
        UINT32    mask = Xil_In32(HOSTIF_IRQ_IC_ID + XIN_IER_OFFSET); \
        mask = (mask & ~(1 << HOSTIF_IRQ));                           \
        XIntc_DisableIntr(HOSTIF_IRQ_IC_ID, mask);                    \
    }

#define HOSTIF_FLUSH_DCACHE_RANGE(base, range) \
    microblaze_flush_dcache_range(base, range)

#define HOSTIF_INVALIDATE_DCACHE_RANGE(base, range) \
    microblaze_invalidate_dcache_range(base, range)

#endif /* _INC_HOST_IF_MICROBLAZE_H_ */
