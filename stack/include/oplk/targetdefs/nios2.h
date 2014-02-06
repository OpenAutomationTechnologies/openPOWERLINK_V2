/**
********************************************************************************
\file   targetdefs/nios2.h

\brief  Target specific definitions for NIOS2 systems

This file contains target specific definitions for NIOS2 systems.
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2014, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
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

#ifndef _INC_targetdefs_nios2_H_
#define _INC_targetdefs_nios2_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <alt_types.h>
#include <io.h>
#include <lock.h>

#include <oplk/basictypes.h>

#define ROM_INIT                // variables will be initialized directly in ROM (means no copy from RAM in startup)
#define ROM                     // code or variables mapped to ROM (i.e. flash)
                                // usage: CONST BYTE ROM foo = 0x00;
#define HWACC                   // hardware access through external memory (i.e. CAN)

// These types can be adjusted by users to match application requirements. The goal is to
// minimize code memory and maximize speed.
#define GENERIC                 // generic pointer to point to application data
                                // Variables with this attribute can be located in external
                                // or internal data memory.
#define MEM                     // Memory attribute to optimize speed and code of pointer access.

#ifndef NEAR
#define NEAR                    // variables mapped to internal data storage location
#endif

#ifndef FAR
#define FAR                     // variables mapped to external data storage location
#endif

#ifndef CONST
#define CONST const             // variables mapped to ROM (i.e. flash)
#endif

#define LARGE

#define REENTRANT
#define PUBLIC
#define EPLDLLEXPORT

#define UNUSED_PARAMETER(par)   (void)par

#if !defined(__OPTIMIZE__)
//restore default: disable inlining if optimization is disabled
#define INLINE_FUNCTION
#undef  INLINE_ENABLED
#undef  INLINE_FUNCTION_DEF
#endif

#include <oplk/section-nios2.h>

#ifndef NDEBUG
#define PRINTF(...)                 printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

/* NOTE:
 * Nios II does not support atomic instructions, hence, pseudo atomic
 * macro is applied with locking.
 */
#define OPLK_ATOMIC_T    alt_u8
#define OPLK_LOCK_T      LOCK_T
#define OPLK_ATOMIC_INIT(base) \
                        if(target_initLock(&base->lock) != 0) \
                            return kEplNoResource
#define OPLK_ATOMIC_EXCHANGE(address, newval, oldval) \
                        target_lock(); \
                        oldval = IORD(address, 0); \
                        IOWR(address, 0, newval); \
                        target_unlock()

#endif /* _INC_targetdefs_nios2_H_ */
