/**
********************************************************************************
\file   dualprocshm-c5soc.h

\brief  Dual processor Library platform support header for ALtera Cyclone V SoC

This header file provides specific macros for Altera Cyclone V SoC platform .

*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2015, Kalycito Infotech Private Limited
Copyright (c) 2016, B&R Industrial Automation GmbH
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
#ifndef _INC_dualprocshm_c5soc_H_
#define _INC_dualprocshm_c5soc_H_

/* BASE ADDRESSES */
#if defined(__NIOS2__)

#include <system.h>
#define TARGET_SYNC_IRQ_ID          -1
#define TARGET_SYNC_IRQ             -1

#include "dualprocshm-nios2.h"

#elif defined(__ALTERA_ARM__)

#include <system.h>
#define TARGET_SYNC_IRQ_ID          0
#define TARGET_SYNC_IRQ             0

#include "dualprocshm-c5socarm.h"

#else

#error "DPSHM ERROR: Selected processor is not supported"
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
    unsigned char   turn;                               ///< Flag to determine the processor holding or requesting the lock
    unsigned char   afFlag[DUALPROC_INSTANCE_COUNT];    ///< Flag to request lock for the processor
    unsigned char   reserved1;                          ///< Reserved
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

#endif //_INC_dualprocshm_c5soc_H_
