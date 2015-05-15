/**
********************************************************************************
\file   altera_arm/sleep.c

\brief  Inexact usleep implementation for Altera Cyclone V ARM

Waits an amount of microseconds or milliseconds. Uses global 64 bit timer ticks
to check the time.

\ingroup module_target
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

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------

#include "sleep.h"
#include <sys/unistd.h>
#include <alt_timers.h>
#include <alt_globaltmr.h>
#include <alt_interrupt.h>
#include <alt_cache.h>
#include <alt_fpga_manager.h>
#include <alt_bridge_manager.h>
#include <alt_address_space.h>
#include <alt_mpu_registers.h>
#include <alt_clock_manager.h>

#include <system.h>

#include <oplk/oplk.h>
#include <oplk/debug.h>

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

#define SECS_TO_MILLISECS    1000

//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------

static inline uint64_t getTimerTicksFromScaled(ALT_GPT_TIMER_t timerId_p,
                                               uint32_t scalingFactor_p,
                                               uint32_t scaledTimeDuration_p);

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief    Sleep until limit of microseconds is reached

\param  usecs_p                Count of microseconds to sleep

\ingroup module_target
*/
//------------------------------------------------------------------------------
int usleep(uint32_t usecs_p)
{
    uint64_t        startTime = alt_globaltmr_get64();
    uint32_t        timerPrescaler = alt_globaltmr_prescaler_get() + 1;
    uint64_t        endTime;
    alt_freq_t      timerClkSrc;

    alt_clk_freq_get(ALT_CLK_MPU_PERIPH, &timerClkSrc);
    endTime = startTime + usecs_p * ((timerClkSrc / timerPrescaler) / 1000000);

    while (alt_globaltmr_get64() < endTime)
    {
    }
}

//------------------------------------------------------------------------------
/**
\brief Sleep for the specified number of milliseconds

The function makes the calling thread sleep until the number of specified
milliseconds have elapsed.

\param  milliSeconds_p      Number of milliseconds to sleep

\ingroup module_target
*/
//------------------------------------------------------------------------------
int msleep(unsigned long int milliSeconds_p)
{
    uint64_t                startTickStamp = alt_globaltmr_get64();
    uint64_t                waitTickCount = getTimerTicksFromScaled(ALT_GPT_CPU_GLOBAL_TMR, SECS_TO_MILLISECS, milliSeconds_p);
    volatile uint32_t*      glbTimerRegCntBase_l = (volatile uint32_t*) (GLOBALTMR_BASE + GLOBALTMR_CNTR_LO_REG_OFFSET);
    volatile uint32_t*      glbTimerRegCntBase_h = (volatile uint32_t*) (GLOBALTMR_BASE + GLOBALTMR_CNTR_HI_REG_OFFSET);
    uint64_t                curTickStamp = 0;
    uint32_t                temp = 0;
    uint32_t                hi = 0;
    uint32_t                lo = 0;
    volatile uint32_t       waitCount = 0;
    uint32_t                waitCountLimit = (uint32_t) (5 + (waitTickCount >> 12));
    uint8_t                 fExit = FALSE;
    uint8_t                 readCntLimit = 3;

    curTickStamp = alt_globaltmr_get64();

    while (fExit != TRUE)
    {
        fExit = FALSE;

        if (waitCount == waitCountLimit)
        {
            readCntLimit = 3;

            do
            {
                temp = (*glbTimerRegCntBase_h);
                lo =  (*glbTimerRegCntBase_l);
                hi = (*glbTimerRegCntBase_h);
            } while ((temp != hi) && (--readCntLimit));

            if (readCntLimit != 0)
            {
                curTickStamp =  (uint64_t) hi;
                curTickStamp =  (((curTickStamp << 32) & ~((uint64_t) UINT32_MAX)) | lo);
                if ((((curTickStamp >= startTickStamp) && ((curTickStamp - startTickStamp) < waitTickCount)) ||
                     ((curTickStamp < startTickStamp) && ((UINT64_MAX - startTickStamp) +
                                                          curTickStamp) < waitTickCount)))
                {
                    fExit = FALSE;
                }
                else
                {
                    fExit = TRUE;
                }

                waitCount = 1;
            }
            else
            {
                waitCount--;
            }
        }

        waitCount++;
    }
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//

/// \name Private Functions
/// \{
//------------------------------------------------------------------------------
/**
\brief Convert time units into timer ticks

The function converts the time in standard unit into ticks for the
given timer.

\param  timerId_p               The ALT_GPT_TIMER_t enum Id of the timer used
\param  scalingFactor_p         Ratio of provided time duration scale to seconds
\param  scaledTimeDuration_p    Time duration in standard unit to be converted

\return The function returns a unsigned 64 bit value.
\retval The converted tick count for the given timer.
*/
//------------------------------------------------------------------------------
static inline uint64_t getTimerTicksFromScaled(ALT_GPT_TIMER_t timerId_p,
                                               uint32_t scalingFactor_p,
                                               uint32_t scaledTimeDuration_p)
{
    uint64_t        ticks = 0;                      // value to return
    ALT_CLK_t       clkSrc = ALT_CLK_UNKNOWN;
    uint32_t        preScaler = 0;
    uint32_t        freq = 1;

    preScaler = alt_gpt_prescaler_get(timerId_p);
    if (preScaler <= UINT8_MAX)
    {
        if (timerId_p == ALT_GPT_CPU_GLOBAL_TMR)    // Global Timer
        {
            ticks = 1;
            clkSrc = ALT_CLK_MPU_PERIPH;
        }
        else
        {
            ticks = 0;
            goto Exit;
        }

        if (alt_clk_freq_get(clkSrc, &freq) == ALT_E_SUCCESS)
        {
            // clock ticks per second
            ticks *= freq;

            // total clock ticks
            ticks *= (uint64_t) (scaledTimeDuration_p / scalingFactor_p);

            // convert into timer ticks
            ticks /= (preScaler + 1);
        }
    }

Exit:
    return ticks;
}

///\}
