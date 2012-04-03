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
 *    0: big endian        (tested on Xilinx Microblaze CPU)
 *    1: little endian    (tested on Altera Nios II CPU)
 *
 *    - OMETH_MAKE_NONCACHABLE(ptr)
 *      Defines a function to make a ptr noncachable
 *      (Important for processors with enabled data cache)
 *
 * History:
 *
 *    11.10.2004    enzinger    created
 *  26.03.2009  zelenkaj    revised
 *    25.01.2010    zelenkaj    added Nios II uncache data
 *    07.09.2011    zelenkaj    added Microblaze (PLB) handling
 *
 ******************************************************************************/

#ifndef __OMETHLIB_TARGET_H__
#define __OMETHLIB_TARGET_H__

#if defined(__NIOS2__)
    // Nios II is little endian
    #define OMETH_HW_MODE                    1
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
    #define OMETH_MAKE_NONCACHABLE(ptr)        (volatile void*)(((unsigned long)ptr)|NIOS2_BYPASS_DCACHE_MASK);
#elif defined(__MICROBLAZE__)
    // Microblaze is big endian (with PLB)
    //FIXME: Is Microblaze big endian with AXI!?!?!?
    #define OMETH_HW_MODE                    0
    #define OMETH_MAKE_NONCACHABLE(ptr)     (ptr)
#else
    #error "Host CPU is unknown, set OMETH_HW_MODE and OMETH_MAKE_NONCACHABLE!"
    #define OMETH_HW_MODE                    0
    #define OMETH_MAKE_NONCACHABLE(ptr)     (ptr)
#endif

#endif

