/****************************************************************************

  (c) SYSTEC electronic GmbH, D-07973 Greiz, August-Bebel-Str. 29
      www.systec-electronic.com

  Project:      Project independend shared buffer (linear + circular)

  Description:  target specific functions
                to Altera NiosII CPU without any OS

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

                $RCSfile$

                $Author$

                $Revision$  $Date$

                $State$

                Build Environment:
                    GNU

  -------------------------------------------------------------------------

  Revision History:

****************************************************************************/

#include "global.h"
#include "sys/alt_irq.h"
#include "sys/alt_alarm.h"



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
// Function:     ShbTgtTimerInit
//
// Description:  This function initializes one system timer for generating
//               1µs-timer ticks.
//
//
// Parameters:   none
//
// 
// Returns:      
//
//
// State:        
//
//---------------------------------------------------------------------------
/*
void PUBLIC ShbTgtTimerInit(void)
{
    TimerValue_l = 0; // init tick count
}
*/


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

    dwTicks = alt_nticks();

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
static alt_irq_context  irq_context = 0; 
static int              iLockCount = 0;

    if (fEnable_p != FALSE)
    {   // restore interrupts
        if (--iLockCount == 0)
        {
            alt_irq_enable_all(irq_context);
        }
    }
    else
    {   // disable interrupts
        if (iLockCount == 0)
        {
            irq_context = alt_irq_disable_all();
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

    if (alt_irq_enabled() == 0)
    {
        return TRUE;
    }
    else
    {
        return FALSE;
    }
}



//=========================================================================//
//                                                                         //
//          S T A T I C   F U N C T I O N S                                //
//                                                                         //
//=========================================================================//


