/**
********************************************************************************
\file   dualprocshm-microblaze.h

\brief  Dual processor Library Target support header - For Microblaze target

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

#ifndef _INC_dualprocshm_microblaze_H_
#define _INC_dualprocshm_microblaze_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <stddef.h>
#include <stdio.h>
#include <xil_types.h>
#include <xil_cache.h>
#include <xintc_l.h>
#include <xil_io.h>
#include <xparameters.h>
#include <targetsection.h>
#include <dualprocshm-mem.h>
//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

// Memory
#define DUALPROCSHM_MALLOC(size)    malloc(size)
#define DUALPROCSHM_FREE(ptr)       free(ptr)

// Sleep
#define DUALPROCSHM_USLEEP(x)       usleep((unsigned int)x)

// IO operations
#define DPSHM_READ8(base)           Xil_In8((UINT32)base);
#define DPSHM_WRITE8(base, val)     Xil_Out8((UINT32)base, val);
#define DPSHM_READ16(base)          Xil_In16((UINT32)base);
#define DPSHM_WRITE16(base, val)    Xil_Out16((UINT32)base, val);

// Memory barrier
// FIXME: Find other suitable way to handle memory barrier for Microblaze
#define DPSHM_DMB()                 usleep(2)

// Cache hadling
#define DUALPROCSHM_FLUSH_DCACHE_RANGE(base, range) \
    microblaze_flush_dcache_range((UINT32)base, range);

#define DUALPROCSHM_INVALIDATE_DCACHE_RANGE(base, range) \
    microblaze_invalidate_dcache_range((UINT32)base, range);

#define DPSHM_REG_SYNC_INTR(callback, arg)                     \
    UINT32      intcMask;                                      \
    XIntc_RegisterHandler(TARGET_IRQ_IC_BASE, TARGET_SYNC_IRQ, \
                         (XInterruptHandler) callback, arg);   \
    intcMask = Xil_In32(TARGET_IRQ_IC_BASE + XIN_IER_OFFSET);  \
    XIntc_EnableIntr(TARGET_IRQ_IC_DIST_BASE,                  \
                     TARGET_SYNC_IRQ | intcMask)

#define DPSHM_ENABLE_SYNC_INTR()                                   \
    XIntc_EnableIntr(TARGET_SYNC_IRQ_ID, TARGET_SYNC_IRQ |         \
                     Xil_In32(TARGET_IRQ_IC_BASE + XIN_IER_OFFSET))

#define DPSHM_DISABLE_SYNC_INTR()                                  \
    XIntc_DisableIntr(TARGET_SYNC_IRQ_ID, TARGET_SYNC_IRQ |        \
                      Xil_In32(TARGET_IRQ_IC_BASE + XIN_IER_OFFSET))

#ifndef TRACE
#ifndef NDEBUG
#define TRACE(...) printf(__VA_ARGS__)
#else
#define TRACE(...)
#endif
#endif

//------------------------------------------------------------------------------
// typedef
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// function prototypes
//------------------------------------------------------------------------------

#ifdef __cplusplus
extern "C" {
#endif
#ifdef __cplusplus
}
#endif

#endif /* _INC_dualprocshm_microblaze_H_ */
