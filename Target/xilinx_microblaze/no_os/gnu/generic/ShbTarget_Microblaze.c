/****************************************************************************

  Copyright (c) 2009, B&R
  (c) SYSTEC electronic GmbH, D-07973 Greiz, August-Bebel-Str. 29
      www.systec-electronic.com

  Project:      Project independend shared buffer (linear + circular)

  Description:  target specific functions
                to Xilinx Microblaze CPU without any OS

  License:

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    1. Redistributions of source code must retain the above copyright
       notice, this list of conditions and the following disclaimer.

    2. Redistributions in binary form must reproduce the above copyright
       notice, this list of conditions and the following disclaimer in the
       documentation and/or other materials provided with the distribution.

    3. Neither the name of SYSTEC electronic GmbH nor the names of its
       contributors may be used to endorse or promote products derived
       from this software without prior written permission. For written
       permission, please contact info@systec-electronic.com.

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

  -------------------------------------------------------------------------

                Author: ZelenkaJ

                2009-11-23

  -------------------------------------------------------------------------

  Revision History:
  2009-11-23	generated from Altera Nios II Target source

****************************************************************************/

#include "global.h"
#include "Benchmark.h"

#include "xparameters.h"
#include "IRQ_Microblaze.h"


/***************************************************************************/
/*                                                                         */
/*                                                                         */
/*          G L O B A L   D E F I N I T I O N S                            */
/*                                                                         */
/*                                                                         */
/***************************************************************************/

//---------------------------------------------------------------------------
// const defines
//---------------------------------------------------------------------------


//---------------------------------------------------------------------------
// modul global types
//---------------------------------------------------------------------------


//---------------------------------------------------------------------------
// local vars
//---------------------------------------------------------------------------


//---------------------------------------------------------------------------
// local function prototypes
//---------------------------------------------------------------------------


//=========================================================================//
//                                                                         //
//          P U B L I C   F U N C T I O N S                                //
//                                                                         //
//=========================================================================//


//---------------------------------------------------------------------------
//
// Function:     ShbTgtGetTickCount
//
// Description:  This function returns the current value of timer.
//
//
// Parameters:   none
//
// 
// Returns:      DWORD with tick count in µs
//
//
// State:        
//
//---------------------------------------------------------------------------

DWORD PUBLIC ShbTgtGetTickCountMs(void)
{
DWORD dwTicks;

    dwTicks = *((DWORD*)XPAR_XIL_SYSTICK_0_MEM0_BASEADDR);

    return dwTicks;
}



//---------------------------------------------------------------------------
//
// Function:    ShbTgtEnableGlobalInterrupt()
//
// Description: enable resp. disable interrupts globally
//
// Parameters:  fEnable_p               = TRUE: enable interrupts
//                                        FALSE: disable interrupts
//
// Return:
//
// State:       not tested
//
//---------------------------------------------------------------------------

void  PUBLIC  ShbTgtEnableGlobalInterrupt(BYTE fEnable_p)
{
static int              iLockCount = 0;
static int				iFirstTime = 0;
XIntc *Intc = getIntc();

    if (fEnable_p != FALSE)
    {   // restore interrupts
		if (--iLockCount == 0)
        {
    		if(iFirstTime == 0)
    		{
	    		//start IRC
	    		XIntc_Start(Intc, XIN_REAL_MODE);
    		}
    		
    		//enable all IRQs
			XIntc_Enable(Intc, XPAR_XPS_INTC_0_XIL_OPENMAC_0_RX_IR_N_INTR);
			XIntc_Enable(Intc, XPAR_XPS_INTC_0_XIL_OPENMAC_0_TX_IR_N_INTR);
			XIntc_Enable(Intc, XPAR_XPS_INTC_0_XIL_OPENMAC_0_CMP_IR_N_INTR);
			
    		if(iFirstTime++ == 0)
    		{
	    		//enable all IRQs on Microblaze
	    		microblaze_enable_interrupts();
    		}
        }
    }
    else
    {   // disable interrupts
        if (iLockCount == 0)
        {
			XIntc_Disable(Intc, XPAR_XPS_INTC_0_XIL_OPENMAC_0_RX_IR_N_INTR);
			XIntc_Disable(Intc, XPAR_XPS_INTC_0_XIL_OPENMAC_0_TX_IR_N_INTR);
			XIntc_Disable(Intc, XPAR_XPS_INTC_0_XIL_OPENMAC_0_CMP_IR_N_INTR);
        }
        iLockCount++;
    }

}



//---------------------------------------------------------------------------
//
// Function:    ShbTgtIsInterruptContext()
//
// Description: check if processor is in interrupt context
//
// Parameters:  none
//
// Return:      FALSE - not in interrupt context
//              TRUE  - in interrupt context
//
// State:       not tested
//
//---------------------------------------------------------------------------

BYTE  PUBLIC  ShbTgtIsInterruptContext(void)
{
    // No real interrupt context check is performed.
    // This would be possible with a flag in the ISR, only.
    // For now, the global interrupt enable flag is checked.
	return FALSE;
}


//=========================================================================//
//                                                                         //
//          S T A T I C   F U N C T I O N S                                //
//                                                                         //
//=========================================================================//


