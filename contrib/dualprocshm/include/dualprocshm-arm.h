/**
********************************************************************************
\file   dualprocshm-arm.h

\brief  Dual Processor Library Target support Header - For ARM target

This header file provides specific macros for Zynq ARM(ps7_cortexa9_0) CPU.

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


#ifndef _INC_dualprocshm_arm_H_
#define _INC_dualprocshm_arm_H_


//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <stdint.h>
#include <stdlib.h>
#include <xil_cache.h>
#include <xscugic.h>
#include <xil_exception.h>
#include <unistd.h>
#include <xil_io.h>
#include <xparameters.h>
#include <xil_types.h>

// include section header file for special functions in
// tightly-coupled memory
//#include <section-arm.h>//TODO@gks: Check if this is needed with no TCM

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------


/// memory
#define DUALPROCSHM_MALLOC(size)            malloc(size)
#define DUALPROCSHM_FREE(ptr)               free(ptr)

/// sleep
#define DUALPROCSHM_USLEEP(x)               usleep((UINT32)x)

/// IO operations
#define DPSHM_READ8(base)                   Xil_In8((UINT32)base);
#define DPSHM_WRITE8(base,val)              Xil_Out8((UINT32)base,val);
#define DPSHM_READ16(base)                  Xil_In16((UINT32)base);
#define DPSHM_WRITE16(base,val)             Xil_Out16((UINT32)base,val);

/// cache handling
#define DUALPROCSHM_FLUSH_DCACHE_RANGE(base,range) \
                    Xil_DCacheFlushRange((UINT32)base, range);

#define DUALPROCSHM_INVALIDATE_DCACHE_RANGE(base,range) \
                    Xil_DCacheInvalidateRange((UINT32)base, range);

//------------------------------------------------------------------------------
// typedef
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// function prototypes
//------------------------------------------------------------------------------


#endif /* _INC_DUALPROCSHM_ARM_H_ */
