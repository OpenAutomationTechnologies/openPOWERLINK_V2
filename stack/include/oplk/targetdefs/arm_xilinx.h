/**
********************************************************************************
\file   targetdefs/arm_xilinx.h

\brief  Target specific definitions for Xilinx ARM core systems

This file contains target specific definitions for Xilinx ARM cortex A9 systems.
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2014, Kalycito Infotech Private Limited
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

#ifndef _INC_targetdefs_arm_xilinx_H_
#define _INC_targetdefs_arm_xilinx_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <xil_types.h>
#include <xil_io.h>
#include <xil_cache.h>
#include <lock.h>

#include <oplk/basictypes.h>

#define ROM_INIT                    // variables will be initialized directly in ROM (means no copy from RAM in startup)
#define ROM                         // code or variables mapped to ROM (i.e. flash)
                                    // usage: CONST BYTE ROM foo = 0x00;

#define MEM                         // Memory attribute to optimize speed and code of pointer access.

#ifndef CONST
#define CONST               const   // variables mapped to ROM (i.e. flash)
#endif

#define OPLKDLLEXPORT

#define UNUSED_PARAMETER(par)       (void)par

#ifndef NDEBUG
#define PRINTF(...)                 printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

#define OPLK_IO_WR8(addr, val)      Xil_Out8(addr, val)
#define OPLK_IO_WR16(addr, val)     Xil_Out16(addr, val)
#define OPLK_IO_WR32(addr, val)     Xil_Out32(addr, val)
// - Read
#define OPLK_IO_RD8(addr)           Xil_In8(addr)
#define OPLK_IO_RD16(addr)          Xil_In16(addr)
#define OPLK_IO_RD32(addr)          Xil_In32(addr)

// Target data cache functions
#define OPLK_DCACHE_FLUSH(addr, len)        Xil_DCacheFlushRange((unsigned int) addr, len)
#define OPLK_DCACHE_INVALIDATE(addr, len)   Xil_DCacheInvalidateRange((unsigned int) addr, len)

// Target memory barrier function
#define OPLK_MEMBAR()               dmb()
/* NOTE:
 * Pseudo atomic macro is applied with locking.
 */

#define OPLK_ATOMIC_T       u8
#define OPLK_LOCK_T         LOCK_T
#define OPLK_ATOMIC_INIT(base)             \
    if (target_initLock(&base->lock) != 0) \
        return kErrorNoResource
#define OPLK_ATOMIC_EXCHANGE(address, newval, oldval) \
    target_lock();                                    \
    oldval = Xil_In8((u32)address);                   \
    Xil_Out8((u32)address, newval);                   \
    target_unlock()

#endif /* _INC_targetdefs_arm_xilinx_H_ */
