/**
********************************************************************************
\file   target/openmac.h

\brief  Definition for openMAC drivers

This file contains definitions used by openMAC Ethernet and timer drivers.

*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2013, SYSTEC electronic GmbH
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
#ifndef _INC_target_openmac_H_
#define _INC_target_openmac_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <common/oplkinc.h>
#include <openmac_cfg.h>
#include <omethlib.h>

#if (DEV_SYSTEM == _DEV_NIOS2_)
#include "openmac-nios2.h"
#elif ((DEV_SYSTEM == _DEV_MICROBLAZE_LITTLE_) || (DEV_SYSTEM == _DEV_MICROBLAZE_BIG_))
#include "openmac-microblaze.h"
#else
#error "Target-specific implementation for openMAC not available!"
#endif

//------------------------------------------------------------------------------
// check for correct compilation options
//------------------------------------------------------------------------------
//TODO: Check here OPENMAC_PKTLOCTX and OPENMAC_PKTLOCRX configuration
//      (OPENMAC_PKTBUF_LOCAL and OPENMAC_PKTBUF_EXTERN)

#if ((OPENMAC_PKTLOCRX == OPENMAC_PKTBUF_LOCAL) && (OPENMAC_PKTLOCTX == OPENMAC_PKTBUF_EXTERN))
#error "This Packet Buffer configuration is not supported!"
#endif

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#define EDRV_PHY_RST_PULSE_MS       10      ///< Phy reset pulse [ms]
#define EDRV_PHY_RST_READY_MS       5       ///< Phy ready after reset [ms]

#define EDRV_MAX_BUFFER_SIZE        1518    ///< MTU
#define EDRV_MAX_RX_BUFFERS         32      ///< Number of supported Rx buffers
#define EDRV_MAX_FILTERS            16      ///< Number of supported Rx Filters
#define EDRV_MAX_AUTO_RESPONSES     14      ///< Number of supported auto-response

#define HWTIMER_SYNC            0   ///< Sync hardware timer
#define HWTIMER_EXT_SYNC        1   ///< External sync hardware timer

#if (OPENMAC_TIMERPULSE != 0)
#define TIMER_USE_EXT_SYNC_INT
#endif

// borrowed from omethlibint.h
#define GET_TYPE_BASE(typ, element, ptr)    \
    ((typ*)(((size_t)ptr) - (size_t)&((typ*)0)->element ))

//------------------------------------------------------------------------------
// typedef
//------------------------------------------------------------------------------
typedef void (*tOpenmacIrqCb)(void* pArg_p);

/**
\brief openMAC IRQ sources

This enumeration defines the available IRQ sources of openMAC.
*/
typedef enum
{
    kOpenmacIrqSync     = 0,    ///< Sync timer IRQ
    kOpenmacIrqTxRx     = 1,    ///< MAC IRQ (Tx and Rx)
    kOpenmacIrqLast             ///< Dummy, count of valid interrupt sources
} eOpenmacIrqSource;

/**
\brief openMAC IRQ source data type

Data type for the enumerator \ref eOpenmacIrqSource.
*/
typedef UINT32 tOpenmacIrqSource;

//------------------------------------------------------------------------------
// global variable declarations
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// function prototypes
//------------------------------------------------------------------------------
#ifdef __cplusplus
extern "C"
{
#endif

tOplkError openmac_isrReg(tOpenmacIrqSource irqSource_p, tOpenmacIrqCb pfnIsrCb_p, void* pArg_p);

void*      openmac_uncachedMalloc(size_t size_p);
void       openmac_uncachedFree(void* pMem_p);

#ifdef __cplusplus
}
#endif

#endif /* _INC_target_openmac_H_ */
