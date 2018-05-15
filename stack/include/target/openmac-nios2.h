/**
********************************************************************************
\file   target/openmac-nios2.h

\brief  Definition for openMAC drivers on Nios II

This file contains definitions used by openMAC Ethernet and timer drivers
specific to Nios II targets.

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
#ifndef _INC_target_openmac_nios2_H_
#define _INC_target_openmac_nios2_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <common/oplkinc.h>
#include <openmac_cfg.h>
#include <omethlib.h>

#include <sys/alt_cache.h>
#include <sys/alt_irq.h>
#include <io.h>
#include <unistd.h>

//------------------------------------------------------------------------------
// check for correct compilation options
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

#define OPENMAC_MEMUNCACHED(pMem_p, size_p)                 (UINT8*)alt_remap_uncached(pMem_p, size_p)
#define OPENMAC_FLUSHDATACACHE(pMem_p, size_p)
#define OPENMAC_INVALIDATEDATACACHE(pMem_p, size_p)
#define OPENMAC_GETDMAOBSERVER()                            IORD_16DIRECT(OPENMAC_DOB_BASE, 0)

#if defined(NIOS2_EIC_PRESENT)
/* Only an External Interruption Controller with a Vectored Interrupt Controller (named vic_0) on PCP processor is supported. */
#if (defined(__ALTERA_VIC) && !defined(PCP_0_VIC_0_BASE))
#error "Currently only vic_0 on pcp_0 is supported."
#endif
#define OPENMAC_GETPENDINGIRQ()                             IORD_ALTERA_VIC_INT_PENDING(PCP_0_VIC_0_BASE)
#else /* defined(NIOS2_EIC_PRESENT) */
#define OPENMAC_GETPENDINGIRQ()                             alt_irq_pending()
#endif /* defined(NIOS2_EIC_PRESENT) */

#define OPENMAC_TIMER_OFFSET(timer_p)                       (timer_p << 4)

#define OPENMAC_TIMERIRQDISABLE(timer_p)                    IOWR_8DIRECT(OPENMAC_TIMER_BASE, OPENMAC_TIMER_OFFSET(timer_p) + 0x0, 0)
#define OPENMAC_TIMERIRQENABLE(timer_p)                     IOWR_8DIRECT(OPENMAC_TIMER_BASE, OPENMAC_TIMER_OFFSET(timer_p) + 0x0, 1)
#define OPENMAC_TIMERIRQACK(timer_p)                        IOWR_8DIRECT(OPENMAC_TIMER_BASE, OPENMAC_TIMER_OFFSET(timer_p) + 0x1, 1)
#define OPENMAC_TIMERSETCOMPAREVALUE(timer_p, val_p)        IOWR_32DIRECT(OPENMAC_TIMER_BASE, OPENMAC_TIMER_OFFSET(timer_p) + 0x4, val_p)

#if (OPENMAC_TIMERPULSECONTROL != 0)
#define OPENMAC_TIMERIRQSETPULSE(timer_p, pulseWidth_p)     IOWR_32DIRECT(OPENMAC_TIMER_BASE, OPENMAC_TIMER_OFFSET(timer_p) + 0x8, pulseWidth_p)
#endif

#define OPENMAC_TIMERGETTIMEVALUE()                         IORD_32DIRECT(OPENMAC_TIMER_BASE, 0xC)

//------------------------------------------------------------------------------
// typedef
//------------------------------------------------------------------------------

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

#ifdef __cplusplus
}
#endif

#endif /* _INC_target_openmac_nios2_H_ */
