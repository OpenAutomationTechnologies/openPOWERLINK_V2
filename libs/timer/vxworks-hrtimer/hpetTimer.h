/*******************************************************************************
  File:         hpetTimer.h

  (c) Bernecker + Rainer Ges.m.b.H.,  B&R Strasse 1, 5142 Eggelsberg, Austria
      www.br-automation.com

  Project:      openPOWERLINK

  Description:

    Header file contains definitions for the HPET timer device implementation

  License:

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    1. Redistributions of source code must retain the above copyright
       notice, this list of conditions and the following disclaimer.

    2. Redistributions in binary form must reproduce the above copyright
       notice, this list of conditions and the following disclaimer in the
       documentation and/or other materials provided with the distribution.

    3. Neither the name of Bernecker + Rainer Ges.m.b.H nor the names of its
       contributors may be used to endorse or promote products derived
       from this software without prior written permission. For written
       permission, please contact office@br-automation.com.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    POSSIBILITY OF SUCH DAMAGE.

    Severability Clause:

        If a provision of this License is or becomes illegal, invalid or
        unenforceable in any jurisdiction, that shall not affect:
        1. the validity or enforceability in that jurisdiction of any other
           provision of this License; or
        2. the validity or enforceability in other jurisdictions of that or
           any other provision of this License.

*******************************************************************************/

#ifndef __INC_HPETTIMER_H
#define __INC_HPETTIMER_H


//============================================================================//
// Includes                                                                   //
//============================================================================//
#include "hwif/timer/vxbHpetLib.h"
#include <timers.h>

//============================================================================//
// Defines
//============================================================================//

// minimum/maximum timeout value in nanoseconds
#define TIMERDEV_MIN_TIMEOUT            10000
#define TIMERDEV_MAX_TIMEOUT            299000000000LL

/* HPET register address and offset definitions */
#define HPET_ADRS                       (HPET_ADDR_SEL_00)
#define HPET_OFF_GCAP_ID                0x0000
#define HPET_OFF_GCAP_PERIOD            0x0004
#define HPET_OFF_GEN_CONF               0x0010
#define HPET_OFF_STATUS                 0x0020
#define HPET_OFF_MAIN_CNT               0x00F0

#define HPET_OFF_TIM_CONF(X)            (0x100 + 0x20 * X)
#define HPET_OFF_TIM_CMP(X)             (0x108 + 0x20 * X)
#define HPET_OFF_TIM_ROUTE(X)           (0x110 + 0x20 * X)

/* Bit definitions for HPET GEN_CONF register */
#define HPET_GEN_CONF_ENA               0x00000001
#define HPET_GEN_CONF_LEGACY            0x00000002

/* Bit definitions for HPET GCAP register */
#define HPET_GCAP_TIMNUM                0x00001F00
#define HPET_GCAP_TIMNUM_SHIFT          8

/* Bit definitions for HPET TIM_CONF register */
#define HPET_TIM_CONF_LEVEL             0x00000002
#define HPET_TIM_CONF_ENABLE            0x00000004
#define HPET_TIM_CONF_PERIOD            0x00000008
#define HPET_TIM_CONF_PERIOD_CAP        0x00000010
#define HPET_TIM_CONF_64BIT_CAP         0x00000020
#define HPET_TIM_CONF_SETVAL            0x00000040
#define HPET_TIM_CONF_32BIT             0x00000100
#define HPET_TIM_CONF_ROUTE             0x00003E00
#define HPET_TIM_CONF_FSB               0x00004000
#define HPET_TIM_CONF_FSB_CAP           0x00008000
#define HPET_TIM_CONF_ROUTE_SHIFT       9

/* Bit definitions for HPET GINTR_STA register */
#define HPET_TIM_STAT_T00               0x00000001
#define HPET_TIM_STAT_T01               0x00000002
#define HPET_TIM_STAT_T02               0x00000004

/* interrupt definitions */
#define    HPET_INT_LVL                 0x2b
#define    HPET_INT_NUM                 11

//============================================================================//
// Function declarations                                                      //
//============================================================================//
int timerdev_init(void);
void timerdev_shutdown(void);
void timerdev_arm(unsigned long long ullDiffTimeout_p);
void timerdev_disarm(void);
int timerdev_readClock(unsigned long long *pUllClock_p);
int timerdev_registerInterruptHandler(VOIDFUNCPTR pfnHandler_p, int iArg_p);
void timerdev_start(void);
void timerdev_stop(void);
void timerdev_reset(void);
void timerdev_show(void);

#endif /* ifndef __INC_HPETTIMER_H */
