/**
********************************************************************************
\file   dualprocshm-zynq.h

\brief  Dual processor Library platform support header - For Zynq Platform

This header file provides specific macros for Xilinx Zynq platform .

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
#ifndef _INC_dualprocshm_ZYNQ_H_
#define _INC_dualprocshm_ZYNQ_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

/* BASE ADDRESSES */
#if defined(__MICROBLAZE__)
#include "dualprocshm-microblaze.h"

#define TARGET_SYNC_IRQ_ID          -1
#define TARGET_SYNC_IRQ             -1

///< Interrupt controller specific defines
#define TARGET_IRQ_IC_BASE          -1
#define TARGET_IRQ_IC_DIST_BASE     -1

#else

#error "Unknown target for Zynq"

#endif

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#define DUALPROC_INSTANCE_COUNT     2       ///< Number of supported instances

//------------------------------------------------------------------------------
// typedef
//------------------------------------------------------------------------------
/**
\brief Dual processor lock

The structure holds the locking parameters used for the
locking mechanism in dual processor shared memory library.

*/
typedef struct sDualprocLock
{
    unsigned char   lockToken;              ///< Locking token
    unsigned char   aPadding1[3];           ///< Padding array variable 1
} tDualprocLock;

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

#endif //_INC_dualprocshm_ZYNQ_H_
