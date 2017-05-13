/**
********************************************************************************
\file   dualprocshm-microblaze.h

\brief  Dual processor Library Target support header - For Microblaze target

This header file provides specific macros for Xilinx Microblaze CPU.

*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2016, Kalycito Infotech Private Limited
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
#ifndef _INC_dualprocshm_microblaze_H_
#define _INC_dualprocshm_microblaze_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <unistd.h>

#include <xil_types.h>
#include <xil_cache.h>
#include <xintc_l.h>
#include <xil_io.h>
#include <mb_interface.h>
#include <xparameters.h>
#include <targetsection.h>
#include <dualprocshm-mem.h>

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

// Memory
#define DUALPROCSHM_MALLOC(size)            malloc(size)
#define DUALPROCSHM_FREE(ptr)               free(ptr)
#define DUALPROCSHM_MEMCPY(dest, src, siz)  memcpy((dest), (src), (siz))

// IO operations
#define DPSHM_READ8(base)                   Xil_In8((UINT32)base)
#define DPSHM_WRITE8(base, val)             Xil_Out8((UINT32)base, val)
#define DPSHM_READ16(base)                  Xil_In16((UINT32)base)
#define DPSHM_WRITE16(base, val)            Xil_Out16((UINT32)base, val)
#define DPSHM_READ32(base)                  Xil_In32((UINT32)base)
#define DPSHM_WRITE32(base, val)            Xil_Out32((UINT32)base, val)

// Memory barrier
#ifdef __GNUC__
// Note: Suppress gcc braced-group warning what is not available in ISO C.
#define DPSHM_DMB()                         __extension__ mbar(1)
#else
#define DPSHM_DMB()                         mbar(1)
#endif

// Cache handling
#define DUALPROCSHM_FLUSH_DCACHE_RANGE(base, range) \
    Xil_L1DCacheFlushRange((UINT32)(base), range)

#define DUALPROCSHM_INVALIDATE_DCACHE_RANGE(base, range) \
    Xil_L1DCacheInvalidateRange((UINT32)(base), range)

#define DPSHM_REG_SYNC_INTR(callback, arg)                        \
    do                                                            \
    {                                                             \
        UINT32  intcMask;                                         \
                                                                  \
        XIntc_RegisterHandler(TARGET_IRQ_IC_BASE,                 \
                              TARGET_SYNC_IRQ,                    \
                              (XInterruptHandler)callback,        \
                              arg);                               \
        intcMask = Xil_In32(TARGET_IRQ_IC_BASE + XIN_IER_OFFSET); \
        XIntc_EnableIntr(TARGET_IRQ_IC_DIST_BASE,                 \
                         TARGET_SYNC_IRQ | intcMask);             \
    } while (0)

#define DPSHM_ENABLE_SYNC_INTR()                                   \
    XIntc_EnableIntr(TARGET_SYNC_IRQ_ID,                           \
                     TARGET_SYNC_IRQ |                             \
                       Xil_In32(TARGET_IRQ_IC_BASE + XIN_IER_OFFSET))

#define DPSHM_DISABLE_SYNC_INTR()                                  \
    XIntc_DisableIntr(TARGET_SYNC_IRQ_ID,                          \
                      TARGET_SYNC_IRQ |                            \
                        Xil_In32(TARGET_IRQ_IC_BASE + XIN_IER_OFFSET))

#define DPSHM_CONNECT_SYNC_IRQ()
#define DPSHM_DISCONNECT_SYNC_IRQ()

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

#endif /* _INC_dualprocshm_microblaze_H_ */
