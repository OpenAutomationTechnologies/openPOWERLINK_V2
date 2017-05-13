/**
********************************************************************************
\file   hostiflib_microblaze.h

\brief  Host Interface Library - For Microblaze target

This header file provides specific macros for Xilinx Microblaze CPU.

*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2014 Kalycito Infotech Private Limited
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
#ifndef _INC_hostiflib_microblaze_H_
#define _INC_hostiflib_microblaze_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <stddef.h>
#include <stdlib.h>
#include <stdint.h>
#include <xil_types.h>
#include <xil_io.h>
#include <xil_cache.h>
#include <mb_interface.h>

#include <xparameters.h>

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#if (CONFIG_HOSTIF_PCP != FALSE)

#define HOSTIF_BASE             XPAR_AXI_HOSTINTERFACE_0_BASEADDR
#define HOSTIF_IRQ_IC_ID        -1
#define HOSTIF_IRQ              (UINT32_MAX + 1) // the irq mask is 32 bit wide

#else

#ifdef XPAR_AXI_HOSTINTERFACE_0_HOST_BASEADDR

#define HOSTIF_BASE             XPAR_AXI_HOSTINTERFACE_0_HOST_BASEADDR

#ifdef XPAR_HOST_INTC_DEVICE_ID

#define HOSTIF_IRQ_IC_ID        XPAR_HOST_INTC_BASEADDR
#define HOSTIF_IRQ              XPAR_HOST_INTC_AXI_HOSTINTERFACE_0_IRQOUT_IRQ_INTR

#else

#error "No Valid Interrupt Controller found for Host!"

#endif // XPAR_HOST_INTC_DEVICE_ID

#else
// Parallel Interface Host  External Processor
// Note: Using external host, ensure the interface base address is correct!
#define HOSTIF_BASE             XPAR_AXI_EPC_0_PRH0_BASEADDR

#ifdef XPAR_HOST_INTC_DEVICE_ID

#define HOSTIF_IRQ_IC_ID        XPAR_HOST_INTC_BASEADDR
#define HOSTIF_IRQ              XPAR_HOST_INTC_SYSTEM_AXI_HOSTINTERFACE_0_IRQOUT_IRQ_PIN_0_INTR

#else

#error "No Valid Interrupt Controller found for Host!"

#endif  // XPAR_HOST_INTC_DEVICE_ID

#endif  // defined(XPAR_AXI_HOSTINTERFACE_0_HOST_BASEADDR)
#endif  // (CONFIG_HOSTIF_PCP != FALSE)

#if (XPAR_MICROBLAZE_USE_DCACHE == 1)
#define HOSTIF_SYNC_DCACHE      TRUE
#else
#define HOSTIF_SYNC_DCACHE      FALSE
#endif

#define HOSTIF_MAKE_NONCACHEABLE(ptr)       (void*) ptr

#define HOSTIF_UNCACHED_MALLOC(size)        malloc(size)
#define HOSTIF_UNCACHED_FREE(ptr)           free(ptr)

// hw access
#define HOSTIF_RD32(base)                   Xil_In32(base)
#define HOSTIF_RD16(base)                   Xil_In16(base)
#define HOSTIF_RD8(base)                    Xil_In8(base)

#define HOSTIF_WR32(base, dword)            Xil_Out32(base, dword)
#define HOSTIF_WR16(base, word)             Xil_Out16(base, word)
#define HOSTIF_WR8(base, byte)              Xil_Out8(base, byte)

#define HOSTIF_DCACHE_FLUSH(base, len)      Xil_L1DCacheFlushRange((unsigned int)(base), len)
#define HOSTIF_DCACHE_INVALIDATE(base, len) Xil_L1DCacheInvalidateRange((unsigned int)(base), len)

//------------------------------------------------------------------------------
// typedef
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// function prototypes
//------------------------------------------------------------------------------

#endif /* _INC_HOST_IF_MICROBLAZE_H_ */
