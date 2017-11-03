/**
********************************************************************************
\file   xilinx-microblaze/usleep.c

\brief  Inexact usleep implementation for Microblaze

Waits an amount of microseconds. This implementation is without a hardware timer
and therefore only an approximation. It is recommended to put the sleep function
into BRAM blocks to increase the accuracy.

\ingroup module_target
*******************************************************************************/

/*------------------------------------------------------------------------------
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

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include "usleep.h"

#include <xil_types.h>
#include <xparameters.h>

//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// module global vars
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// global function prototypes
//------------------------------------------------------------------------------

//============================================================================//
//            P R I V A T E   D E F I N I T I O N S                           //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#if defined(XPAR_MICROBLAZE_CORE_CLOCK_FREQ_HZ)
#define CPU_SPEED_TICKS     XPAR_MICROBLAZE_CORE_CLOCK_FREQ_HZ
#else
#error "There is no CPU speed available! Please check xparameters.h"
#endif

#define CPU_SPEED_MHZ       (CPU_SPEED_TICKS / 1000000)     ///< CPU speed in Mhz

#define SMALL_LOOP_SPEED    (10 * (CPU_SPEED_MHZ / 50))     ///< The small loop always takes 1us
                                                            ///< -> it is adjusted to need 10 iterations with 50Mhz

//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
void usleep(u32 usecs_p) __attribute__((section(".local_memory")));

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief    Sleep until limit of microseconds is reached

\param[in]      usecs_p             Count of microseconds to sleep

\ingroup module_target
*/
//------------------------------------------------------------------------------
void usleep(u32 usecs_p)
{
    u16 smallLoop = SMALL_LOOP_SPEED;

    __asm
    (
      "       addik r11, r0, 1         \n\t"    // fill r11 with decrement value
      "outerLoop: rsub %0, r11, %0     \n\t"
      "innerLoop: rsub %1, r11, %1     \n\t"    //1 cycle
      "       nop                      \n\t"    //1 cycle
      "       bnei %1, innerLoop       \n\t"    //3 cycles
      "       add %1, r0, %2           \n\t"
      "       bnei %0, outerLoop       \n\t"
          : /* no output registers */
          : "r"(usecs_p), "r"(smallLoop), "r"(smallLoop)
          : "r11"
    );
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

/// \}
