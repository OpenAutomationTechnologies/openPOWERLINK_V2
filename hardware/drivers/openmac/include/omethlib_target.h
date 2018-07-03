/*****************************************************************************
Copyright (c) 2009, B&R
All rights reserved.

Redistribution and use in source and binary forms,
with or without modification,
are permitted provided that the following conditions are met:

- Redistributions of source code must retain the above copyright notice,
this list of conditions and the following disclaimer.

- Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer
in the documentation and/or other materials provided with the distribution.

- Neither the name of the B&R nor the names of
its contributors may be used to endorse or promote products derived
from this software without specific prior written permission.


THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 *****************************************************************************
 * Module:      omethLib
 * File:        omethLib_target.h
 * Author:      Thomas Enzinger (enzingert)
 * Created:     11.10.2004
 * Revised:     25.01.2010
 * State:       tested on Altera Nios II and Xilinx Microblaze
 ******************************************************************************
 * Description:
 *
 *    Target specific defines for the module omethLib
 *
 *  - OMETH_HW_MODE must be defined to select the hardware mode
 *    1: big endian        (tested on Xilinx Microblaze CPU)
 *    0: little endian    (tested on Altera Nios II CPU)
 *
 *    - OMETH_MAKE_NONCACHABLE(ptr)
 *      Defines a function to make a ptr noncachable
 *      (Important for processors with enabled data cache)
 *
 *
 ******************************************************************************/

#ifndef __OMETHLIB_TARGET_H__
#define __OMETHLIB_TARGET_H__

#if defined(__NIOS2__)
    #include <io.h>

    #include <sys/alt_cache.h>

    // Nios II is little endian
    #define OMETH_HW_MODE                    0

    static inline uint8_t ometh_rd_8(volatile uint8_t *p)
    {
        return IORD_8DIRECT((uint32_t)p, 0);
    }

    static inline void ometh_wr_8(volatile uint8_t *p, uint8_t v)
    {
        IOWR_8DIRECT((uint32_t)p, 0, v);
    }

    static inline uint16_t ometh_rd_16(volatile uint16_t *p)
    {
        return IORD_16DIRECT((uint32_t)p, 0);
    }

    static inline void ometh_wr_16(volatile uint16_t *p, uint16_t v)
    {
        IOWR_16DIRECT((uint32_t)p, 0, v);
    }

    static inline uint32_t ometh_rd_32(volatile uint32_t *p)
    {
        return IORD_32DIRECT((uint32_t)p, 0);
    }

    static inline void ometh_wr_32(volatile uint32_t *p, uint32_t v)
    {
        IOWR_32DIRECT((uint32_t)p, 0, v);
    }

    static inline void *ometh_rd_cpu_ptr(const void *p)
    {
        return (void *)IORD_32DIRECT((uint32_t)p, 0);
    }

    static inline void ometh_wr_cpu_ptr(void *p, void *v)
    {
        IOWR_32DIRECT((uint32_t)p, 0, (uint32_t)v);
    }

    //---------------------------------------------------------
    // borrowed from Altera Nios II Toolchain
    //  alt_remap_uncached.c
        #ifdef NIOS2_MMU_PRESENT
        /* Convert KERNEL region address to IO region address */
        #define NIOS2_BYPASS_DCACHE_MASK   (0x1 << 29)
        #else
        /* Set bit 31 of address to bypass D-cache */
        #define NIOS2_BYPASS_DCACHE_MASK   (0x1 << 31)
        #endif
    //
    //---------------------------------------------------------
    #define OMETH_MAKE_NONCACHABLE(ptr)        (void*)(((uint32_t)ptr)|NIOS2_BYPASS_DCACHE_MASK);
    #define OMETH_UNCACHED_MALLOC(size)        alt_uncached_malloc(size)
    #define OMETH_UNCACHED_FREE(ptr)           alt_uncached_free(ptr)

#elif defined(__MICROBLAZE__)

#include <xil_io.h>
#include <xparameters.h>

#ifndef XPAR_MICROBLAZE_ENDIANNESS
    #error "XPAR_MICROBLAZE_ENDIANNESS not defined in xparameters.h!"
#endif

#if XPAR_MICROBLAZE_ENDIANNESS == 0
    // Microblaze is big endian (with PLB)
    #define OMETH_HW_MODE                    1
#else
    // Microblaze is little endian (with AXI)
    #define OMETH_HW_MODE                     0
#endif

    static inline uint8_t ometh_rd_8(volatile uint8_t *p)
    {
        return Xil_In8((uint32_t)p);
    }

    static inline void ometh_wr_8(volatile uint8_t *p, uint8_t v)
    {
        Xil_Out8((uint32_t)p, v);
    }

    static inline uint16_t ometh_rd_16(volatile uint16_t *p)
    {
        return Xil_In16((uint32_t)p);
    }

    static inline void ometh_wr_16(volatile uint16_t *p, uint16_t v)
    {
        Xil_Out16((uint32_t)p, v);
    }

    static inline uint32_t ometh_rd_32(volatile uint32_t *p)
    {
        return Xil_In32((uint32_t)p);
    }

    static inline void ometh_wr_32(volatile uint32_t *p, uint32_t v)
    {
        Xil_Out32((uint32_t)p, v);
    }

    static inline void *ometh_rd_cpu_ptr(const void *p)
    {
        return (void *)Xil_In32((uint32_t)p);
    }

    static inline void ometh_wr_cpu_ptr(void *p, void *v)
    {
        Xil_Out32((uint32_t)p, (uint32_t)v);
    }

    #define OMETH_MAKE_NONCACHABLE(ptr)     (ptr)
    #define OMETH_UNCACHED_MALLOC(size)     malloc(size)
    #define OMETH_UNCACHED_FREE(ptr)        free(ptr)

#else
    #error "Host CPU is unknown, set OMETH_HW_MODE and OMETH_MAKE_NONCACHABLE!"
    #define OMETH_HW_MODE                    0
    #define OMETH_MAKE_NONCACHABLE(ptr)     (ptr)
    #define OMETH_UNCACHED_MALLOC(size)     malloc(size)
    #define OMETH_UNCACHED_FREE(ptr)        free(ptr)
#endif

//---------------------------------------------------------
// include section header file for special functions in
// local memory
#include <targetsection.h>

// Set empty defines for undefined section assignments
#ifndef SECTION_OMETHLIB_RX_IRQ_HDL
#define SECTION_OMETHLIB_RX_IRQ_HDL
#endif
#ifndef SECTION_OMETHLIB_TX_IRQ_HDL
#define SECTION_OMETHLIB_TX_IRQ_HDL
#endif
#ifndef SECTION_OMETHLIB_RXTX_IRQ_MUX
#define SECTION_OMETHLIB_RXTX_IRQ_MUX
#endif
#ifndef SECTION_OMETHLIB_TX_TIME
#define SECTION_OMETHLIB_TX_TIME
#endif

#endif
