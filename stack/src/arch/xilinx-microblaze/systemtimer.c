/**
********************************************************************************
\file   xilinx-microblaze/systemtimer.c

\brief  Implement system timer by using a periodic millisecond counter

Initialize the system timer and count the milliseconds

\ingroup module_target
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2016, B&R Industrial Automation GmbH
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

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include "systemtimer.h"

#include <xintc.h>              //interrupt controller higher level
#include <mb_interface.h>
#include <xparameters.h>

//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#if !defined(CONFIG_PCP)
#error "CONFIG_PCP is needed for this implementation!"
#endif

#if (CONFIG_PCP == FALSE)
#define TGT_INTC_BASE           XPAR_INTC_0_BASEADDR
#define TGT_TIMER_INTR          XPAR_HOST_INTC_FIT_TIMER_0_INTERRUPT_INTR
#elif (CONFIG_PCP != FALSE)
#define TGT_INTC_BASE           XPAR_INTC_0_BASEADDR
#define TGT_TIMER_INTR          XPAR_PCP_INTC_FIT_TIMER_0_INTERRUPT_INTR
#else
#error  "Unable to determine the processor instance"
#endif

#define TGT_TIMER_INTR_MASK     XPAR_FIT_TIMER_0_INTERRUPT_MASK

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

//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static unsigned int msCount_l = 0;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static void irqHandler(void* pArg_p);

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief    Initialize system timer

\ingroup module_target
*/
//------------------------------------------------------------------------------
void timer_init(void)
{
    //register fit interrupt handler
    XIntc_RegisterHandler(TGT_INTC_BASE,
                          TGT_TIMER_INTR,
                          (XInterruptHandler)irqHandler,
                          0);

    //enable the fit interrupt
    XIntc_EnableIntr(TGT_INTC_BASE, TGT_TIMER_INTR_MASK);
}

//------------------------------------------------------------------------------
/**
\brief    Get current timer in ms

\return The timer in ms

\ingroup module_target
*/
//------------------------------------------------------------------------------
unsigned int timer_getMsCount(void)
{
    return msCount_l;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//

/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief    User timer interrupt handler

\param[in]      pArg_p              Interrupt handler argument

\ingroup module_target
*/
//------------------------------------------------------------------------------
static void irqHandler(void* pArg_p)
{
    (void)pArg_p;       // Unused parameter

    msCount_l++;
}

/// \}
