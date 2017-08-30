/**
********************************************************************************
\file   dualprocshm-pcie.h

\brief  Dual processor library platform support header - For PCIe solutions

This header file provides specific macros for external PCIe based solutions.

*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2015 Kalycito Infotech Private Limited
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
#ifndef _INC_dualprocshm_pcie_H_
#define _INC_dualprocshm_pcie_H_

#if defined(_KERNEL_MODE)

#include <dualprocshm-winkernel.h>

// The base addresses are retrieved dynamically after
// BAR mapping.
#define SHARED_MEM_BASE
#define COMMON_MEM_BASE
#define MEM_ADDR_TABLE_BASE
#define MEM_INTR_BASE

/* Memory size */
#define MAX_COMMON_MEM_SIZE        3072                         ///< Max common memory size
#define MAX_DYNAMIC_BUFF_COUNT     20                           ///< Number of maximum dynamic buffers
#define MAX_DYNAMIC_BUFF_SIZE      MAX_DYNAMIC_BUFF_COUNT * 4   ///< Max dynamic buffer size
#define MEM_ADDR_TABLE_OFFSET      MAX_COMMON_MEM_SIZE
#define MEM_INTR_OFFSET            MAX_COMMON_MEM_SIZE + MAX_DYNAMIC_BUFF_SIZE
#define OPLK_PCIEBAR_SHM           0
#define OPLK_PCIEBAR_COMM_MEM      1

#elif defined(__NIOS2__)
#include <system.h>

#define TARGET_SYNC_IRQ_ID         -1
#define TARGET_SYNC_IRQ            -1

///< Interrupt controller specific defines
#define TARGET_IRQ_IC_BASE         -1
#define TARGET_IRQ_IC_DIST_BASE    -1

#include "dualprocshm-nios2.h"

#elif defined(__linux__)

#include <dualprocshm-linuxkernel.h>

// The base addresses are retrieved dynamically after
// BAR mapping.
#define SHARED_MEM_BASE
#define COMMON_MEM_BASE
#define MEM_ADDR_TABLE_BASE
#define MEM_INTR_BASE

/* Memory size */
#define MAX_COMMON_MEM_SIZE        3072                         ///< Max common memory size
#define MAX_DYNAMIC_BUFF_COUNT     20                           ///< Number of maximum dynamic buffers
#define MAX_DYNAMIC_BUFF_SIZE      (MAX_DYNAMIC_BUFF_COUNT * 4) ///< Max dynamic buffer size

#define MEM_ADDR_TABLE_OFFSET      MAX_COMMON_MEM_SIZE          ///< Offset of the address table from the start of common memory
#define MEM_INTR_OFFSET            (MAX_COMMON_MEM_SIZE + MAX_DYNAMIC_BUFF_SIZE) ///< Offset of the interrupt register from the start of common memory
#define OPLK_PCIEBAR_SHM           0                            ///< BAR id to access the shared memory segment
#define OPLK_PCIEBAR_COMM_MEM      1                            ///< BAR id to access the common memory segment

#else

#error "unknown target for external PCIe solutions"

#endif

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#define DUALPROC_INSTANCE_COUNT    2    ///< Number of supported instances

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
    unsigned char   turn;                               ///< Flag to determine the processor holding or requesting the lock
    unsigned char   afFlag[DUALPROC_INSTANCE_COUNT];    ///< Flag to request lock for the processor
    unsigned char   reserved1;                          ///< Reserved
} tDualprocLock;

#endif //_INC_dualprocshm_pcie_H_
