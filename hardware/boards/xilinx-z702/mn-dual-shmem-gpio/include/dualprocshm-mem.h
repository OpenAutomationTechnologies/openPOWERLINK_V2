/**
********************************************************************************
\file   dualprocshm-mem.h

\brief  Board specific memory definitions for dualprocshm library

This file contains the definitions for memory offsets for dualprocshm library
for a Xilinx ZC702 board.
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2015, Kalycito Infotech Private Limited
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

#ifndef _INC_dualprocshm_mem_H_
#define _INC_dualprocshm_mem_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <xparameters.h>

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
/* Memory size */
#define MAX_COMMON_MEM_SIZE         2048                         ///< Max common memory size
#define MAX_DYNAMIC_BUFF_COUNT      20                           ///< Number of maximum dynamic buffers
#define MAX_DYNAMIC_BUFF_SIZE       MAX_DYNAMIC_BUFF_COUNT * 4   ///< Max dynamic buffer size

/* BASE ADDRESSES */

#if defined(__arm__)
    // TODO : gks check if this can be retrieved from hardware configuration
    #define COMMON_MEM_BASE             0x2C000000

    #if defined(XPAR_PS7_DDR_0_S_AXI_HP0_BASEADDR) && defined (XPAR_PS7_DDR_0_S_AXI_HP0_HIGHADDR)
        #define SHARED_MEM_BASE         (XPAR_PS7_DDR_0_S_AXI_HP0_BASEADDR + 0x20000000)
        #define SHARED_MEM_SPAN         (XPAR_PS7_DDR_0_S_AXI_HP0_HIGHADDR - \
                                         SHARED_MEM_BASE)
    #else
        #error "Shared memory base address(SHARED_MEM_BASE) could not be set!"
    #endif
#elif defined(__MICROBLAZE__)
    // TODO : gks check if this can be retrieved from hardware configuration
    #define COMMON_MEM_BASE             0x2C000000

    #if defined(XPAR_PS7_DDR_0_S_AXI_HP0_BASEADDR) && defined (XPAR_PS7_DDR_0_S_AXI_HP0_HIGHADDR)
        #define SHARED_MEM_BASE         (XPAR_PS7_DDR_0_S_AXI_HP0_BASEADDR + 0x20000000)
        #define SHARED_MEM_SPAN         (XPAR_PS7_DDR_0_S_AXI_HP0_HIGHADDR - \
                                         SHARED_MEM_BASE)
    #else
        #error "Shared memory base address(SHARED_MEM_BASE) could not be set!"
    #endif
#else
    #error "Processor not supported!!"
#endif

#define MEM_ADDR_TABLE_BASE         (COMMON_MEM_BASE + MAX_COMMON_MEM_SIZE)
#define MEM_INTR_BASE               (MEM_ADDR_TABLE_BASE + MAX_DYNAMIC_BUFF_SIZE)

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

#endif /* _INC_dualprocshm_mem_H_ */
