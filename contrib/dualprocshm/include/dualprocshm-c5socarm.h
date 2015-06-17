/**
********************************************************************************
\file   dualprocshm-c5socarm.h

\brief  Dual Processor Library Target support Header - For Altera SoC ARM target

This header file provides specific macros for Altera Cyclone V SoC ARM CPU.

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

#ifndef _INC_dualprocshm_c5socarm_H_
#define _INC_dualprocshm_c5socarm_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <stdint.h>
#include <unistd.h>

#include <system.h>

#include <socal/socal.h>
#include <alt_timers.h>
#include <alt_globaltmr.h>
#include <alt_interrupt.h>
#include <alt_cache.h>
#include <alt_fpga_manager.h>
#include <alt_bridge_manager.h>
#include <alt_address_space.h>
#include <alt_mpu_registers.h>
#include <alt_clock_manager.h>
#include <dualprocshm-mem.h>

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

#ifndef TRACE
#ifndef NDEBUG
#define TRACE(...)                      printf(__VA_ARGS__)
#else
#define TRACE(...)
#endif
#endif

// memory
#define DPSHM_MAKE_NONCACHEABLE(ptr)    (void*)(((unsigned long)ptr))
#define DUALPROCSHM_MALLOC(size)        malloc(size)
#define DUALPROCSHM_FREE(ptr)           free(ptr)

#define CALC_OFFSET(addr_p, baseAddr_p)  \
    ({                                   \
         ULONG offset = 0;               \
         offset = (addr_p - baseAddr_p); \
         offset;                         \
     })

// sleep
#define DUALPROCSHM_USLEEP(x)           usleep((UINT32)x)
#define DPSHM_DMB()                     __asm("dmb")

// IO operations
#define DPSHM_READ8(base)               alt_read_byte((UINT32)base)
#define DPSHM_WRITE8(base, val)         alt_write_byte((UINT32)base, val)
#define DPSHM_READ16(base)              alt_read_hword((UINT32)base)
#define DPSHM_WRITE16(base, val)        alt_write_hword((UINT32)base, val)
#define DPSHM_READ32(base)              alt_read_word((UINT32)base)
#define DPSHM_WRITE32(base, val)        alt_write_word((UINT32)base, val)

// Memory barrier
#define CACHE_ALIGNED_BYTE_CHECK    (ALT_CACHE_LINE_SIZE - 1)

// cache handling
#ifdef ALTARM_CACHE_ENABLE
#define DUALPROCSHM_FLUSH_DCACHE_RANGE(base, range)                                                                                         \
    ({                                                                                                                                      \
         UINT32 tempBase = (UINT32)(((UINT32)base) & ~((UINT32)CACHE_ALIGNED_BYTE_CHECK));                                                  \
         UINT32 tempCeil = (UINT32)((((UINT32)base + (UINT32)range) + CACHE_ALIGNED_BYTE_CHECK) & ~((UINT32)CACHE_ALIGNED_BYTE_CHECK));     \
         alt_cache_system_clean((void*)tempBase, (size_t)(tempCeil - tempBase));                                                            \
     })

#define DUALPROCSHM_INVALIDATE_DCACHE_RANGE(base, range)                                                                                    \
    ({                                                                                                                                      \
         UINT32 tempBase = (UINT32)(((UINT32)base) & ~((UINT32)CACHE_ALIGNED_BYTE_CHECK));                                                  \
         UINT32 tempCeil = (UINT32)((((UINT32)base + (UINT32)range) + CACHE_ALIGNED_BYTE_CHECK) & ~((UINT32)CACHE_ALIGNED_BYTE_CHECK));     \
         alt_cache_system_invalidate((void*)tempBase, (size_t)(tempCeil - tempBase));                                                       \
     })
#else

#define DUALPROCSHM_FLUSH_DCACHE_RANGE(base, range)
#define DUALPROCSHM_INVALIDATE_DCACHE_RANGE(base, range)
#endif

// Sync Manager
#define SYNC_IRQ                    (ALT_INT_INTERRUPT_F2S_FPGA_IRQ0 + TARGET_SYNC_IRQ)
#define TARGET_CPU                  0x1

#define DPSHM_REG_SYNC_INTR(callback, arg) \
    alt_int_isr_register(SYNC_IRQ, callback, arg)

#define DPSHM_UNREG_SYNC_INTR(callback, arg)    alt_int_isr_unregister(SYNC_IRQ)

#define DPSHM_CLEAR_SYNC_IRQ()                  alt_int_dist_pending_clear(SYNC_IRQ)

#define DPSHM_ENABLE_SYNC_INTR()                                                              \
    ({                                                                                        \
         int ret = 0;                                                                         \
         alt_int_dist_pending_clear(SYNC_IRQ);                                                \
                                                                                              \
         if (alt_int_dist_target_set(SYNC_IRQ, TARGET_CPU) != ALT_E_SUCCESS)                  \
         {                                                                                    \
             ret = -1;                                                                        \
             TRACE("Sync IRQ target cpu set failed\n");                                       \
         }                                                                                    \
         else                                                                                 \
         {                                                                                    \
             if (alt_int_dist_trigger_set(SYNC_IRQ, ALT_INT_TRIGGER_LEVEL) != ALT_E_SUCCESS)  \
             {                                                                                \
                 ret = -1;                                                                    \
                 TRACE("Sync IRQ trigger set failed\n");                                      \
             }                                                                                \
             else                                                                             \
             {                                                                                \
                 if (alt_int_dist_enable(SYNC_IRQ) != ALT_E_SUCCESS)                          \
                 {                                                                            \
                     /* Set interrupt distributor target */                                   \
                     ret = -1;                                                                \
                     TRACE("Sync IRQ could not be enabled in the distributor\n");             \
                 }                                                                            \
                                                                                              \
             }                                                                                \
         }                                                                                    \
                                                                                              \
         ret;                                                                                 \
     })

#define DPSHM_DISABLE_SYNC_INTR()                                          \
    ({                                                                     \
         int ret = 0;                                                      \
         if (alt_int_dist_disable(SYNC_IRQ) != ALT_E_SUCCESS)              \
         {                                                                 \
             /* access to any FPGA registers if required */                \
             ret = -1;                                                     \
             TRACE("Sync IRQ could not be disabled in the distributor\n"); \
         }                                                                 \
                                                                           \
         ret;                                                              \
     })

//------------------------------------------------------------------------------
// typedef
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// function prototypes
//------------------------------------------------------------------------------

#ifdef __cplusplus
extern "C"
{
#endif
#ifdef __cplusplus
}
#endif

#endif /* _INC_dualprocshm_c5socarm_H_ */
