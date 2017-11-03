/**
********************************************************************************
\file   xilinx-microblaze/openmac-microblaze.c

\brief  Implementation of openMAC drivers

This file contains the implementation of the openMAC driver.

\ingroup module_openmac
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

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <common/oplkinc.h>
#include <target/openmac.h>

#include <xparameters.h>
#include <xintc_l.h>
#include <xio.h>

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
#define INTC_BASE               XPAR_PCP_INTC_BASEADDR

#define OPENMAC_SYNC_IRQ        XPAR_PCP_INTC_AXI_OPENMAC_0_TIMER_IRQ_INTR
#define OPENMAC_SYNC_IRQ_MASK   XPAR_AXI_OPENMAC_0_TIMER_IRQ_MASK
#define OPENMAC_TXRX_IRQ        XPAR_PCP_INTC_AXI_OPENMAC_0_MAC_IRQ_INTR
#define OPENMAC_TXRX_IRQ_MASK   XPAR_AXI_OPENMAC_0_MAC_IRQ_MASK

//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Register interrupt callback for openMAC

This function registers a callback for a specific interrupt source.

\param[in]      irqSource_p         Specified interrupt source
\param[in]      pfnIsrCb_p          Interrupt service routine callback
\param[in]      pArg_p              Argument given to the callback

\return The function returns a tOplkError error code.

\ingroup module_openmac
*/
//------------------------------------------------------------------------------
tOplkError openmac_isrReg(tOpenmacIrqSource irqSource_p,
                          tOpenmacIrqCb pfnIsrCb_p,
                          void* pArg_p)
{
    tOplkError  ret = kErrorOk;
    UINT32      irqId;
    UINT32      irqMask;
    UINT32      intcMask;

    switch (irqSource_p)
    {
        case kOpenmacIrqSync:
            irqId = OPENMAC_SYNC_IRQ;
            irqMask = OPENMAC_SYNC_IRQ_MASK;
            break;

        case kOpenmacIrqTxRx:
            irqId = OPENMAC_TXRX_IRQ;
            irqMask = OPENMAC_TXRX_IRQ_MASK;
            break;

        default:
            ret = kErrorNoResource;
            goto Exit;
    }

    // Get Interrupt controller's current mask
    intcMask = Xil_In32(INTC_BASE + XIN_IER_OFFSET);

    // Register interrupt callback
    XIntc_RegisterHandler(INTC_BASE, irqId, (XInterruptHandler)pfnIsrCb_p, (void*)pArg_p);

    // Enable interrupt by or'ing with controllers mask
    XIntc_EnableIntr(INTC_BASE, irqMask | intcMask);

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Allocated uncached memory

This function allocates memory.
Since the packet buffers are allocated in cached memory, no uncaching is implemented.

\param[in]      size_p              Size of uncached memory to be allocated

\return The function returns the base address of the allocated, uncached memory.

\ingroup module_openmac
*/
//------------------------------------------------------------------------------
void* openmac_uncachedMalloc(size_t size_p)
{
    return malloc(size_p);
}

//------------------------------------------------------------------------------
/**
\brief  Free uncached memory

This function frees the memory pMem_p.

\param[in]      pMem_p              Uncached memory to be freed

\ingroup module_openmac
*/
//------------------------------------------------------------------------------
void openmac_uncachedFree(void* pMem_p)
{
    free(pMem_p);
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

/// \}
