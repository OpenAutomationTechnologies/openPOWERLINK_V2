/**
********************************************************************************
\file   targetdefs/linux.h

\brief  Target defintions for Linux

This file contains target definitions for Linux systems
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

#ifndef _INC_targetdefs_linux_H_
#define _INC_targetdefs_linux_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#ifndef __KERNEL__
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#else
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/major.h>
#include <linux/version.h>
#include <linux/slab.h>
#endif

#include <oplk/basictypes.h>

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

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

#ifdef __KERNEL__
#define EPL_MALLOC(siz)             kmalloc(siz, GFP_KERNEL)
#define EPL_FREE(ptr)               kfree(ptr)
#endif

#ifdef __KERNEL__
#define PRINTF(...)                 printk(__VA_ARGS__)
#else
#define PRINTF(...)                 printf(__VA_ARGS__)
#endif

#define OPLK_ATOMIC_T    UINT8
#define OPLK_ATOMIC_EXCHANGE(address, newval, oldval) \
    oldval = __sync_lock_test_and_set(address, newval);



#endif /* _INC_targetdefs_linux_H_ */
