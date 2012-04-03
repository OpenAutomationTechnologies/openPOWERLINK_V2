/****************************************************************************
  (c) SYSTEC electronic GmbH, D-07973 Greiz, August-Bebel-Str. 29
      www.systec-electronic.com
  (c) Bernecker + Rainer Industrie-Elektronik Ges.m.b.H.
      A-5142 Eggelsberg, B&R Strasse 1
      www.br-automation.com

  Project:      openPOWERLINK

  Description:  target specific functions
                to Xilinx Microblaze CPU without any OS
                file for interrupt settings

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
                    GCC V3.4

****************************************************************************/

#include "xilinx_irq.h"

DWORD dwMsCount_g = 0;

static void FitTimerIrqHandler (void* pArg_p)
{
    dwMsCount_g++;
}

//---------------------------------------------------------------------------
//
// Function:    initInterrupts
//
// Description: inits the global interrupt and registers the fit timer handler
//
// Parameters:  void
//
// Returns:     void
//
// State:
//
//---------------------------------------------------------------------------
void initInterrupts(void)
{
    //note: master enable is asserted, otherwise sharedBuff would enter deadlock
    enableInterruptMaster();

    //register fit irq handler
    XIntc_RegisterHandler(XPAR_PCP_INTC_BASEADDR, XPAR_PCP_INTC_FIT_TIMER_0_INTERRUPT_INTR,
            (XInterruptHandler)FitTimerIrqHandler, 0);

    //enable the fit interrupt
    XIntc_EnableIntr(XPAR_PCP_INTC_BASEADDR, XPAR_FIT_TIMER_0_INTERRUPT_MASK);
}

//---------------------------------------------------------------------------
//
// Function:    enableInterrupts
//
// Description: enable the microblaze interrupt
//
// Parameters:  void
//
// Returns:     void
//
// State:
//
//---------------------------------------------------------------------------
void enableInterrupts(void)
{
    //enable microblaze interrupts
    microblaze_enable_interrupts();
}

//---------------------------------------------------------------------------
//
// Function:    disableInterrupts
//
// Description: disable the microblaze interrupt
//
// Parameters:  void
//
// Returns:     void
//
// State:
//
//---------------------------------------------------------------------------
void disableInterrupts(void)
{
    //disable microblaze interrupts
    microblaze_disable_interrupts();
}

//---------------------------------------------------------------------------
//
// Function:    enableInterruptMaster
//
// Description: enables the interrupt master interrupt
//
// Parameters:  void
//
// Returns:     void
//
// State:
//
//---------------------------------------------------------------------------
void enableInterruptMaster(void)
{
    //enable global interrupt master
    XIntc_MasterEnable(XPAR_PCP_INTC_BASEADDR);
}

//---------------------------------------------------------------------------
//
// Function:    disableInterruptMaster
//
// Description: disable the interrupt master interrupt
//
// Parameters:  void
//
// Returns:     void
//
// State:
//
//---------------------------------------------------------------------------
void disableInterruptMaster(void)
{
    //disable global interrupt master
    XIntc_MasterDisable(XPAR_PCP_INTC_BASEADDR);
}

//---------------------------------------------------------------------------
//
// Function:    getMSCount
//
// Description:
//
// Parameters:  void
//
// Returns:     mscount           = the count in ms
//
// State:
//
//---------------------------------------------------------------------------
DWORD getMSCount(void)
{
    return dwMsCount_g;
}

