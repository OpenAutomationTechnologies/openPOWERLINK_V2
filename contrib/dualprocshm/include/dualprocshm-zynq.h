/**
********************************************************************************
\file   dualprocshm-zynq.h

\brief  Dual processor Library platform support header - For Zynq Platform

This header file provides specific macros for Xilinx Zynq platform .

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

#ifndef _INC_dualprocshm_ZYNQ_H_
#define _INC_dualprocshm_ZYNQ_H_

#define OPLK_OPTIMIZE               TRUE                        ///< Optimize the dualprocessor library for openPOWERLINK stack on non-OS

/* SIZE */
#define MAX_COMMON_MEM_SIZE         2048                        ///< Max common memory size
#define MAX_DYNAMIC_BUFF_COUNT      15                          ///< Number of maximum dynamic buffers
#define MAX_DYNAMIC_BUFF_SIZE       MAX_DYNAMIC_BUFF_COUNT * 4  ///< Max dynamic buffer size

/* BASE ADDRESSES */
#if defined(__MICROBLAZE__)
#include "dualprocshm-microblaze.h"

// TODO : gks check if this can be retrieved from hardware configuration
#define COMMON_MEM_BASE             0x2C000000
#define MEM_ADDR_TABLE_BASE         COMMON_MEM_BASE + MAX_COMMON_MEM_SIZE
#define MEM_INTR_BASE               MEM_ADDR_TABLE_BASE + MAX_DYNAMIC_BUFF_SIZE

#define TARGET_SYNC_IRQ_ID         -1
#define TARGET_SYNC_IRQ            -1

///< Interrupt controller specific defines
#define TARGET_IRQ_IC_BASE         -1
#define TARGET_IRQ_IC_DIST_BASE    -1


#elif defined(__xilinx_arm__)
#include "dualprocshm-arm.h"

// TODO : gks check if this can be retrieved from hardware configuration
#define COMMON_MEM_BASE             0x2C000000
#define MEM_ADDR_TABLE_BASE         COMMON_MEM_BASE + MAX_COMMON_MEM_SIZE
#define MEM_INTR_BASE               MEM_ADDR_TABLE_BASE + MAX_DYNAMIC_BUFF_SIZE

#define TARGET_SYNC_IRQ_ID         XPAR_PS7_SCUGIC_0_DEVICE_ID
#define TARGET_SYNC_IRQ            XPAR_FABRIC_AXI_OPENMAC_0_TIMER_IRQ_INTR

///< Interrupt controller specific defines
#ifdef XPAR_PS7_SCUGIC_0_BASEADDR
#define TARGET_IRQ_IC_BASE         XPAR_PS7_SCUGIC_0_BASEADDR
#endif

#ifdef XPAR_PS7_SCUGIC_0_DIST_BASEADDR
#define TARGET_IRQ_IC_DIST_BASE    XPAR_PS7_SCUGIC_0_DIST_BASEADDR
#endif

#else

#error "unknown target for Zynq"

#endif

#endif //_INC_dualprocshm_ZYNQ_H_
