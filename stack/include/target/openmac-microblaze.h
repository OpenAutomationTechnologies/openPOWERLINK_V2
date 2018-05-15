/**
********************************************************************************
\file   target/openmac-microblaze.h

\brief  Definition for openMAC drivers on Microblaze

This file contains definitions used by openMAC Ethernet and timer drivers
specific to Microblaze targets.

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
#ifndef _INC_target_openmac_microblaze_H_
#define _INC_target_openmac_microblaze_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <common/oplkinc.h>
#include <openmac_cfg.h>
#include <omethlib.h>

#include <mb_interface.h>
#include <xio.h>

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

#define OPENMAC_MEMUNCACHED(pMem_p, size_p)                 pMem_p
#define OPENMAC_FLUSHDATACACHE(pMem_p, size_p)              Xil_L1DCacheFlushRange((UINT32)(pMem_p), size_p)
#define OPENMAC_INVALIDATEDATACACHE(pMem_p, size_p)         Xil_L1DCacheInvalidateRange((UINT32)(pMem_p), size_p)
#define OPENMAC_GETDMAOBSERVER()                            Xil_In16(OPENMAC_DOB_BASE)
#define OPENMAC_GETPENDINGIRQ()                             0
//FIXME: Test IRQ timing on Xilinx, and implement OPENMAC_GETPENDINGIRQ macro if required

#define OPENMAC_TIMER_OFFSET(timer_p)                       (timer_p << 4)

#define OPENMAC_TIMERIRQDISABLE(timer_p)                    Xil_Out8(OPENMAC_TIMER_BASE + OPENMAC_TIMER_OFFSET(timer_p) + 0x0, 0)
#define OPENMAC_TIMERIRQENABLE(timer_p)                     Xil_Out8(OPENMAC_TIMER_BASE + OPENMAC_TIMER_OFFSET(timer_p) + 0x0, 1)
#define OPENMAC_TIMERIRQACK(timer_p)                        Xil_Out8(OPENMAC_TIMER_BASE + OPENMAC_TIMER_OFFSET(timer_p) + 0x1, 1)
#define OPENMAC_TIMERSETCOMPAREVALUE(timer_p, val_p)        Xil_Out32(OPENMAC_TIMER_BASE + OPENMAC_TIMER_OFFSET(timer_p) + 0x4, val_p)

#if (OPENMAC_TIMERPULSECONTROL != 0)
#define OPENMAC_TIMERIRQSETPULSE(timer_p, pulseWidth_p)     Xil_Out32(OPENMAC_TIMER_BASE + OPENMAC_TIMER_OFFSET(timer_p) + 0x8, pulseWidth_p)
#endif

#define OPENMAC_TIMERGETTIMEVALUE()                         Xil_In32(OPENMAC_TIMER_BASE + 0xC)

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

#endif /* _INC_target_openmac_microblaze_H_ */
