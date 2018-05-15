/**
********************************************************************************
\file   oplk/targetdefs/microblaze.h

\brief  Target specific definitions for microblaze systems

This file contains target specific definitions for microblaze systems.
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2016, B&R Industrial Automation GmbH
Copyright (c) 2013, SYSTEC electronic GmbH
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
#ifndef _INC_oplk_targetdefs_microblaze_H_
#define _INC_oplk_targetdefs_microblaze_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <xil_types.h>
#include <xil_io.h>
#include <xil_cache.h>
#include <mb_interface.h>

#ifdef __ZYNQ__
// Required to include UART redirection for Zynq Microblaze
#include <mb_uart.h>
#endif /* __ZYNQ__ */
#include <oplk/basictypes.h>

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#define OPLKDLLEXPORT

#define INLINE                  inline

#define OPLK_FILE_HANDLE        int

#define UNUSED_PARAMETER(par)   (void)par

#ifndef NDEBUG
#define PRINTF(...)             printf(__VA_ARGS__)
#else /* NDEBUG */
#define PRINTF(...)
#endif /* NDEBUG */

// Target IO functions
// - Write
#define OPLK_IO_WR8(addr, val)      Xil_Out8(addr, val)
#define OPLK_IO_WR16(addr, val)     Xil_Out16(addr, val)
#define OPLK_IO_WR32(addr, val)     Xil_Out32(addr, val)
// - Read
#define OPLK_IO_RD8(addr)           Xil_In8(addr)
#define OPLK_IO_RD16(addr)          Xil_In16(addr)
#define OPLK_IO_RD32(addr)          Xil_In32(addr)

// Target data cache functions
#define OPLK_DCACHE_FLUSH(addr, len)        Xil_L1DCacheFlushRange((unsigned int)(addr), len)
#define OPLK_DCACHE_INVALIDATE(addr, len)   Xil_L1DCacheInvalidateRange((unsigned int)(addr), len)

// Target memory barrier function
#ifdef __GNUC__
// Note: Suppress gcc braced-group warning what is not available in ISO C.
#define OPLK_MEMBAR()               __extension__ mbar(1)
#else /* __GNUC__ */
#define OPLK_MEMBAR()               mbar(1)
#endif /* __GNUC__ */

// Target lock
#define OPLK_LOCK_T                 UINT8

/* NOTE:
 * Pseudo atomic macro is applied with locking.
 */
#define OPLK_ATOMIC_T    u8
#define OPLK_ATOMIC_INIT(base) \
                        if (target_initLock(&base->lock) != 0) \
                            return kErrorNoResource
#define OPLK_ATOMIC_EXCHANGE(address, newval, oldval) \
                        target_lock(); \
                        oldval = Xil_In8((u32)(address)); \
                        Xil_Out8((u32)(address), newval); \
                        target_unlock()

#define OPLK_MUTEX_T    u8

#endif /* _INC_oplk_targetdefs_microblaze_H_ */
