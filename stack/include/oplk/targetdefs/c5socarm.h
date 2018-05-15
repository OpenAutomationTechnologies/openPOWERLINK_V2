/**
********************************************************************************
\file   oplk/targetdefs/c5socarm.h

\brief  Target specific definitions for Altera ARM core systems

This file contains target specific definitions for Altera ARM cortex A9 systems.
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2015, Kalycito Infotech Private Limited
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
#ifndef _INC_oplk_targetdefs_c5socarm_H_
#define _INC_oplk_targetdefs_c5socarm_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include <oplk/basictypes.h>
#include <socal/socal.h>
#include <alt_cache.h>

#define OPLKDLLEXPORT

#define INLINE

#define OPLK_FILE_HANDLE                        int

#define UNUSED_PARAMETER(par)                   (void)par

#ifndef NDEBUG
#define PRINTF(...)                             printf(__VA_ARGS__)
#else /* NDEBUG */
#define PRINTF(...)
#endif

#define FPGA_BUS_WIDTH                          32
#define __IO_CALC_ADDRESS_NATIVE(base, offset) \
    (base + offset * (FPGA_BUS_WIDTH / 8))
#define IORD16(base, offset)                    alt_read_word(base + offset * (FPGA_BUS_WIDTH / 8))
#define IORD32(base, offset)                    alt_read_word(base + offset * (FPGA_BUS_WIDTH / 8))
#define IOWR16(base, offset, val)               alt_write_word(base + offset * (FPGA_BUS_WIDTH / 8), val)
#define IOWR32(base, offset, val)               alt_write_word(base + offset * (FPGA_BUS_WIDTH / 8), val)

/* GPIO register read write macros */
#define IOADDR_ALTERA_AVALON_PIO_DATA(base)                 __IO_CALC_ADDRESS_NATIVE(base, 0)
#define IORD_ALTERA_AVALON_PIO_DATA(base)                   IORD32(base, 0)
#define IOWR_ALTERA_AVALON_PIO_DATA(base, data)             IOWR32(base, 0, data)

#define IOADDR_ALTERA_AVALON_PIO_DIRECTION(base)            __IO_CALC_ADDRESS_NATIVE(base, 1)
#define IORD_ALTERA_AVALON_PIO_DIRECTION(base)              IORD32(base, 1)
#define IOWR_ALTERA_AVALON_PIO_DIRECTION(base, data)        IOWR32(base, 1, data)

#define IOADDR_ALTERA_AVALON_PIO_IRQ_MASK(base)             __IO_CALC_ADDRESS_NATIVE(base, 2)
#define IORD_ALTERA_AVALON_PIO_IRQ_MASK(base)               IORD32(base, 2)
#define IOWR_ALTERA_AVALON_PIO_IRQ_MASK(base, data)         IOWR32(base, 2, data)

#define IOADDR_ALTERA_AVALON_PIO_EDGE_CAP(base)             __IO_CALC_ADDRESS_NATIVE(base, 3)
#define IORD_ALTERA_AVALON_PIO_EDGE_CAP(base)               IORD32(base, 3)
#define IOWR_ALTERA_AVALON_PIO_EDGE_CAP(base, data)         IOWR32(base, 3, data)

#define IOADDR_ALTERA_AVALON_PIO_SET_BIT(base)              __IO_CALC_ADDRESS_NATIVE(base, 4)
#define IORD_ALTERA_AVALON_PIO_SET_BITS(base)               IORD32(base, 4)
#define IOWR_ALTERA_AVALON_PIO_SET_BITS(base, data)         IOWR32(base, 4, data)

#define IOADDR_ALTERA_AVALON_PIO_CLEAR_BITS(base)           __IO_CALC_ADDRESS_NATIVE(base, 5)
#define IORD_ALTERA_AVALON_PIO_CLEAR_BITS(base)             IORD32(base, 5)
#define IOWR_ALTERA_AVALON_PIO_CLEAR_BITS(base, data)       IOWR32(base, 5, data)

// Target IO functions
// - Write
#define OPLK_IO_WR32(base, offset, dword)   alt_write_word((UINT)base + (UINT)offset, dword)
#define OPLK_IO_WR16(base, offset, word)    alt_write_hword((UINT)base + (UINT)offset, word)
#define OPLK_IO_WR8(base, offset, byte)     alt_write_byte((UINT)base + (UINT)offset, byte)

// - Read
#define OPLK_IO_RD32(base, offset)          alt_read_word((UINT)base + (UINT)offset)
#define OPLK_IO_RD16(base, offset)          alt_read_hword((UINT)base + (UINT)offset)
#define OPLK_IO_RD8(base, offset)           alt_read_byte((UINT)base + (UINT)offset)

// Target memory barrier function
#define OPLK_MEMBAR()                       __asm("dmb")

/* NOTE:
 * ARM does not support atomic instructions, hence, pseudo atomic
 * macro is applied with spin lock.
 */

#define OPLK_MUTEX_T                UINT8
#define OPLK_ATOMIC_T               UINT8
#define OPLK_LOCK_T                 UINT8
#define OPLK_ATOMIC_INIT(base)             \
    if (target_initLock(&base->lock) != 0) \
        return kErrorNoResource
#define OPLK_ATOMIC_EXCHANGE(address, newval, oldval) \
    target_lock();                                    \
    oldval = alt_read_byte(address);                  \
    alt_write_byte(address, newval);                  \
    target_unlock()

#define CACHE_ALIGNED_BYTE_CHECK    (ALT_CACHE_LINE_SIZE - 1)

#ifdef ALTARM_CACHE_ENABLE
#define OPLK_DCACHE_FLUSH(base, range)                                                                                                   \
    do                                                                                                                                   \
    {                                                                                                                                    \
         UINT32 tempBase = (UINT32)(((UINT32) base) & ~((UINT32)CACHE_ALIGNED_BYTE_CHECK));                                              \
         UINT32 tempCeil = (UINT32)((((UINT32) base + (UINT32)range) + CACHE_ALIGNED_BYTE_CHECK) & ~((UINT32)CACHE_ALIGNED_BYTE_CHECK)); \
         alt_cache_system_clean((void*)tempBase, (size_t)(tempCeil - tempBase));                                                         \
     } while (0)

#define OPLK_DCACHE_INVALIDATE(base, range)                                                                                              \
    do                                                                                                                                   \
    {                                                                                                                                    \
         UINT32 tempBase = (UINT32)(((UINT32) base) & ~((UINT32)CACHE_ALIGNED_BYTE_CHECK));                                              \
         UINT32 tempCeil = (UINT32)((((UINT32) base + (UINT32)range) + CACHE_ALIGNED_BYTE_CHECK) & ~((UINT32)CACHE_ALIGNED_BYTE_CHECK)); \
         alt_cache_system_invalidate((void*)tempBase, (size_t)(tempCeil - tempBase));                                                    \
     } while (0)
#else /* ALTARM_CACHE_ENABLE */
#define OPLK_DCACHE_FLUSH(base, range)
#define OPLK_DCACHE_INVALIDATE(base, range)
#endif /* ALTARM_CACHE_ENABLE */

#endif /* _INC_oplk_targetdefs_c5socarm_H_ */
