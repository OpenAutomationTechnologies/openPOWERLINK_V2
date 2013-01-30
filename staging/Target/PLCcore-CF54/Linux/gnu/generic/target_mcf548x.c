/****************************************************************************

  (c) SYSTEC electronic GmbH, D-07973 Greiz, August-Bebel-Str. 29
      www.systec-electronic.com

  Project:      openPOWERLINK

  Description:  target specific adaption for ethernet driver
                to MCF548x (e.g. SYSTEC ECUcore-5484) under Linux

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

#include "EplInc.h"
#include "edrv.h"

#include <linux/config.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/major.h>
#include <linux/version.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/uaccess.h>
#include <asm/atomic.h>
#include <linux/interrupt.h>

#ifdef CONFIG_COLDFIRE
    #include <asm/coldfire.h>
    #include <asm/m5485sim.h>
    #include <asm/m5485gpio.h>
#endif

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

// The DMA and the FEC interrupt must have the same level but different priorities.
// Both interrupts handlers are mutual exclusive.
#define MCF_ILP_DMA                 (MCF_ICR_IL(5) | MCF_ICR_IP(6))
#define MCF_ILP_FEC                 (MCF_ICR_IL(5) | MCF_ICR_IP(5))


//---------------------------------------------------------------------------
// modul global types
//---------------------------------------------------------------------------


//---------------------------------------------------------------------------
// local vars
//---------------------------------------------------------------------------

// d.k.: for multiple instance support make array from the following variables
static  int             iCntrEnableIntEth_l;
static  int             iCntrEnableIntDma_l;
static  BYTE            bTgtOldIlpDma_l;


//---------------------------------------------------------------------------
// local function prototypes
//---------------------------------------------------------------------------

static  int         TgtEthIsr (int nIrqNum_p, void* ppDevInstData_p, struct pt_regs* ptRegs_p);

//=========================================================================//
//                                                                         //
//          P U B L I C   F U N C T I O N S                                //
//                                                                         //
//=========================================================================//

//---------------------------------------------------------------------------
//
// Function:    TgtInitEthIsr()
//
// Description: sets up the ethernet interrupt handler
//
// Parameters:
//
// Return:
//
// State:       not tested
//
//---------------------------------------------------------------------------

tEplKernel PUBLIC TgtInitEthIsr()
{

#if (EDRV_USED_ETH_CTRL == 0)
    // disable FEC0 interrupt
    //MCF_INTC_IMRH |= MCF_INTC_IMRH_INT_MASK39;
    // setup level and priority FEC0 interrupt
    MCF_ICR(ISC_FEC0) = MCF_ILP_FEC;
#else
    // disable FEC1 interrupt
    //MCF_INTC_IMRH |= MCF_INTC_IMRH_INT_MASK38;
    // setup level and priority FEC1 interrupt
    MCF_ICR(ISC_FEC1) = MCF_ILP_FEC;
#endif

    if (request_irq(
#if (EDRV_USED_ETH_CTRL == 0)
         64 + ISC_FEC0,
#else
         64 + ISC_FEC1,
#endif
         TgtEthIsr,
         SA_INTERRUPT,
         "EPL-FEC",
         NULL)) {
            printk("Cannot allocate the FEC IRQ(39/38)\n");
            return kEplNoResource;
    }

    // initialize interrupt enable counter
    iCntrEnableIntEth_l = 0;
    iCntrEnableIntDma_l = 0;

    // save level and priority of DMA interrupt
    bTgtOldIlpDma_l = MCF_ICR(ISC_DMA);
    // setup level and priority DMA interrupt
    MCF_ICR(ISC_DMA) = MCF_ILP_DMA;


#if (BENCHMARK_MODULES != 0)
    // configure GPIOs for benchmarking

    MCF_GPIO_PODR_PCIBR |= 0x10;        // set LED data bits to off
    MCF_GPIO_PDDR_PCIBR |= 0x10;        // set LED direction bits to out
    MCF_GPIO_PAR_PCIBR  &= 0xFCFF;      // set PCIBR4 to GPIO pin
#endif

    return kEplSuccessful;
}

//---------------------------------------------------------------------------
//
// Function:    TgtFreeEthIsr
//
// Description: frees the ethernet interrupt handler
//
// Parameters:
//
// Return:
//
// State:       not tested
//
//---------------------------------------------------------------------------

void PUBLIC TgtFreeEthIsr()
{
    // free ethernet irq
#if (EDRV_USED_ETH_CTRL == 0)
    free_irq(64 + ISC_FEC0, NULL);
#else
    free_irq(64 + ISC_FEC1, NULL);
#endif

    // restore level and priority of DMA interrupt
    MCF_ICR(ISC_DMA) = bTgtOldIlpDma_l;
}

//---------------------------------------------------------------------------
//
// Function:    TgtEnableEthInterrupt0()
//
// Description: enables resp. disables the selected interrupts
//
// Parameters:  fEnable_p               = TRUE: enable interrupts
//                                        FALSE: disable interrupts
//              uiInterruptMask_p       = bit field which selects the interrupts
//
// Return:
//
// State:       not tested
//
//---------------------------------------------------------------------------

void PUBLIC TgtEnableEthInterrupt0(BYTE fEnable_p, unsigned int uiInterruptMask_p)
{
#if (EDRV_USED_ETH_CTRL == 0)
    if (fEnable_p != FALSE)
    {   // enable interrupts if necessary
        if ((uiInterruptMask_p & EPL_TGT_INTMASK_ETH) != 0)
        {
            iCntrEnableIntEth_l++;
            if (iCntrEnableIntEth_l == 0)
            {
                iCntrEnableIntEth_l = 1;
            }

            if (iCntrEnableIntEth_l == 1)
            {
                // enable FEC0 interrupt
                enable_irq(64 + ISC_FEC0);
            }
        }
        if ((uiInterruptMask_p & EPL_TGT_INTMASK_DMA) != 0)
        {
            iCntrEnableIntDma_l++;
            if (iCntrEnableIntDma_l == 0)
            {
                iCntrEnableIntDma_l = 1;
            }

            if (iCntrEnableIntDma_l == 1)
            {
                // enable DMA interrupt
                enable_irq(64 + ISC_DMA);
            }
        }
    }
    else
    {   // disable interrupts if necessary
        if ((uiInterruptMask_p & EPL_TGT_INTMASK_ETH) != 0)
        {
            iCntrEnableIntEth_l--;
            if (iCntrEnableIntEth_l == 0)
            {
                iCntrEnableIntEth_l = -1;
            }

            if (iCntrEnableIntEth_l == -1)
            {
                // disable FEC0 interrupt
                disable_irq(64 + ISC_FEC0);
            }
        }
        if ((uiInterruptMask_p & EPL_TGT_INTMASK_DMA) != 0)
        {
            iCntrEnableIntDma_l--;
            if (iCntrEnableIntDma_l == 0)
            {
                iCntrEnableIntDma_l = -1;
            }

            if (iCntrEnableIntDma_l == -1)
            {
                // disable DMA interrupt
                disable_irq(64 + ISC_DMA);
            }
        }
    }
#endif
}

//---------------------------------------------------------------------------
//
// Function:    TgtEnableEthInterrupt1()
//
// Description: enables resp. disables the selected interrupts
//
// Parameters:  fEnable_p               = TRUE: enable interrupts
//                                        FALSE: disable interrupts
//              uiInterruptMask_p       = bit field which selects the interrupts
//
// Return:
//
// State:       not tested
//
//---------------------------------------------------------------------------

void PUBLIC TgtEnableEthInterrupt1(BYTE fEnable_p, unsigned int uiInterruptMask_p)
{
#if (EDRV_USED_ETH_CTRL == 1)
    if (fEnable_p != FALSE)
    {   // enable interrupts if necessary
        if ((uiInterruptMask_p & EPL_TGT_INTMASK_ETH) != 0)
        {
            iCntrEnableIntEth_l++;
            if (iCntrEnableIntEth_l == 0)
            {
                iCntrEnableIntEth_l = 1;
            }

            if (iCntrEnableIntEth_l == 1)
            {
                // enable FEC1 interrupt
                enable_irq(64 + ISC_FEC1);
            }
        }
        if ((uiInterruptMask_p & EPL_TGT_INTMASK_DMA) != 0)
        {
            iCntrEnableIntDma_l++;
            if (iCntrEnableIntDma_l == 0)
            {
                iCntrEnableIntDma_l = 1;
            }

            if (iCntrEnableIntDma_l == 1)
            {
                // enable DMA interrupt
                enable_irq(64 + ISC_DMA);
            }
        }
    }
    else
    {   // disable interrupts if necessary
        if ((uiInterruptMask_p & EPL_TGT_INTMASK_ETH) != 0)
        {
            iCntrEnableIntEth_l--;
            if (iCntrEnableIntEth_l == 0)
            {
                iCntrEnableIntEth_l = -1;
            }

            if (iCntrEnableIntEth_l == -1)
            {
                // disable FEC1 interrupt
                disable_irq(64 + ISC_FEC1);
            }
        }
        if ((uiInterruptMask_p & EPL_TGT_INTMASK_DMA) != 0)
        {
            iCntrEnableIntDma_l--;
            if (iCntrEnableIntDma_l == 0)
            {
                iCntrEnableIntDma_l = -1;
            }

            if (iCntrEnableIntDma_l == -1)
            {
                // disable DMA interrupt
                disable_irq(64 + ISC_DMA);
            }
        }
    }
#endif
}

static  int         TgtEthIsr (int nIrqNum_p, void* ppDevInstData_p, struct pt_regs* ptRegs_p)
{
    EdrvInterruptHandler();
    return 0;
}

