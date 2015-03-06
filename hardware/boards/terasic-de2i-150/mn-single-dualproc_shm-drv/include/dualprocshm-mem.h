/**
********************************************************************************
\file   dualprocshm-mem.h

\brief  Memory configuration file for designs using dualprocshm library

This file contains the definitions for memory offsets for dualprocshm library
for a specific platform. It also defines memory sizes for event and dll queues
to be used for the platform.
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

#ifndef _INC_dualprocshm_mem_H_
#define _INC_dualprocshm_mem_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
/* Memory size */
#define MAX_COMMON_MEM_SIZE        2048                         ///< Max common memory size
#define MAX_DYNAMIC_BUFF_COUNT     20                           ///< Number of maximum dynamic buffers
#define MAX_DYNAMIC_BUFF_SIZE      MAX_DYNAMIC_BUFF_COUNT * 4   ///< Max dynamic buffer size

/* BASE ADDRESSES */
#define SHARED_MEM_BASE           SSRAM_0_BASE
#define SHARED_MEM_SIZE           SSRAM_0_SPAN
#define COMMON_MEM_BASE            PCIE_SUBSYTEM_ONCHIP_MEMORY_BASE
#define MEM_ADDR_TABLE_OFFSET      MAX_COMMON_MEM_SIZE
#define MEM_INTR_OFFSET            (MEM_ADDR_TABLE_OFFSET + MAX_DYNAMIC_BUFF_SIZE)

/* Queue Size */
#define CONFIG_EVENT_SIZE_CIRCBUF_KERNEL_TO_USER    16384
#define CONFIG_EVENT_SIZE_CIRCBUF_USER_TO_KERNEL    16384
#define CONFIG_DLLCAL_BUFFER_SIZE_TX_NMT            2048
#define CONFIG_DLLCAL_BUFFER_SIZE_TX_GEN            2048
#define CONFIG_DLLCAL_BUFFER_SIZE_TX_VETH           1024

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

#endif /* _INC_dualprocshm_mem_H_ */

