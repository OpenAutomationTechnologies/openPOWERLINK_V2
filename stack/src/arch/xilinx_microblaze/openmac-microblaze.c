/**
********************************************************************************
\file   xilinx_microblaze/openmac-microblaze.c

\brief  Implementation of openMAC drivers

This file contains the implementation of the openMAC driver.

\ingroup module_openmac
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2013, SYSTEC electronic GmbH
Copyright (c) 2014, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
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
#include <mb_interface.h>
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
#define INTC_BASE   XPAR_PCP_INTC_BASEADDR

#define OPENMAC_SYNC_IRQ        XPAR_PCP_INTC_AXI_OPENMAC_0_TIMER_IRQ_INTR
#define OPENMAC_SYNC_IRQ_MASK   XPAR_AXI_OPENMAC_0_TIMER_IRQ_MASK
#define OPENMAC_TXRX_IRQ        XPAR_PCP_INTC_AXI_OPENMAC_0_MAC_IRQ_INTR
#define OPENMAC_TXRX_IRQ_MASK   XPAR_AXI_OPENMAC_0_MAC_IRQ_MASK

#define OPENMAC_TIMER_OFFSET_CMP_VAL        0
#define OPENMAC_TIMER_OFFSET_TIME_VAL       0
#define OPENMAC_TIMER_OFFSET_CTRL           4
#define OPENMAC_TIMER_OFFSET_STATUS         4
#define OPENMAC_TIMER_OFFSET_2ND_CMP_VAL    8
#define OPENMAC_TIMER_OFFSET_2ND_CTRL       12

//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------
typedef struct
{
    tOpenmacIrqCb   pfnIrqCb[kOpenmacIrqLast];
    void*           pIrqCbArg[kOpenmacIrqLast];
} tOpenmacInst;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tOpenmacInst instance_l;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static void irqHandler(void* pArg_p) SECTION_EDRVOPENMAC_IRQ_HDL;

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Register interrupt callback for openMAC

This function registers a callback for a specific interrupt source.

\param  irqSource_p     Specified interrupt source
\param  pfnIsrCb_p      Interrupt service routine callback
\param  pArg_p          Argument given to the callback

\return The function returns a tOplkError error code.

\ingroup module_openmac
*/
//------------------------------------------------------------------------------
tOplkError openmac_isrReg(tOpenmacIrqSource irqSource_p, tOpenmacIrqCb pfnIsrCb_p, void* pArg_p)
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
    XIntc_RegisterHandler(INTC_BASE, irqId, (XInterruptHandler)irqHandler, (void*)irqSource_p);

    // Enable interrupt by or'ing with controllers mask
    XIntc_EnableIntr(INTC_BASE, irqMask | intcMask);

    instance_l.pfnIrqCb[irqSource_p] = pfnIsrCb_p;
    instance_l.pIrqCbArg[irqSource_p] = pArg_p;

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Set a memory uncached

This function simply returns the memory pointer.
Since the packet buffers are allocated in cached memory no uncaching is implemented.

\param  pMem_p      Base address of memory region
\param  size_p      Size of memory region

\return The function returns the base address of the uncached memory.

\ingroup module_openmac
*/
//------------------------------------------------------------------------------
UINT8* openmac_memUncached(UINT8* pMem_p, UINT size_p)
{
    UNUSED_PARAMETER(size_p);

    return pMem_p;
}

//------------------------------------------------------------------------------
/**
\brief  Allocated uncached memory

This function allocates memory.
Since the packet buffers are allocated in cached memory, no uncaching is implemented.

\param  size_p      Size of uncached memory to be allocated

\return The function returns the base address of the allocated, uncached memory.

\ingroup module_openmac
*/
//------------------------------------------------------------------------------
UINT8* openmac_uncachedMalloc(UINT size_p)
{
    return (UINT8*)malloc(size_p);
}

//------------------------------------------------------------------------------
/**
\brief  Free uncached memory

This function frees the memory pMem_p.

\param  pMem_p      Uncached memory to be freed

\ingroup module_openmac
*/
//------------------------------------------------------------------------------
void openmac_uncachedFree(UINT8* pMem_p)
{
    free(pMem_p);
}

//------------------------------------------------------------------------------
/**
\brief  Flush data cache region

This function flushes the data cache associated to the memory region.

\param  pMem_p      Associated memory region address
\param  size_p      Size of memory region

\ingroup module_openmac
*/
//------------------------------------------------------------------------------
void openmac_flushDataCache(UINT8* pMem_p, UINT size_p)
{
    microblaze_flush_dcache_range((UINT32)pMem_p, size_p);
}

//------------------------------------------------------------------------------
/**
\brief  Invalidates data cache region

This function invalidates the data cache associated to the memory region.

\param  pMem_p      Associated memory region address
\param  size_p      Size of memory region

\ingroup module_openmac
*/
//------------------------------------------------------------------------------
void openmac_invalidateDataCache(UINT8* pMem_p, UINT size_p)
{
    microblaze_invalidate_dcache_range((UINT32)pMem_p, size_p);
}

//------------------------------------------------------------------------------
/**
\brief  Get DMA observer value

This function returns the DMA observer value.

\param  adapter_p       MAC adapter number

\return The function returns the DMA observer value.

\ingroup module_openmac
*/
//------------------------------------------------------------------------------
UINT16 openmac_getDmaObserver(UINT adapter_p)
{
    UNUSED_PARAMETER(adapter_p);

    return Xil_In16(OPENMAC_DOB_BASE);
}

//------------------------------------------------------------------------------
/**
\brief  Timer interrupt disable

This function disables the associated timer.

\param  timer_p     Timer instance to be disabled

\ingroup module_openmac
*/
//------------------------------------------------------------------------------
void openmac_timerIrqDisable(UINT timer_p)
{
    UINT offset;

    switch (timer_p)
    {
        case HWTIMER_SYNC:
            offset = OPENMAC_TIMER_OFFSET_CTRL;
            break;

        case HWTIMER_EXT_SYNC:
            offset = OPENMAC_TIMER_OFFSET_2ND_CTRL;
            break;

        default:
            return;
    }

    Xil_Out32(OPENMAC_TIMER_BASE + offset, 0);
}

//------------------------------------------------------------------------------
/**
\brief  Timer interrupt enable

This function enables the timer instance and sets the generated pulse width, if
the associated HW timer supports it.

\param  timer_p         Timer instance to be disabled
\param  pulseWidthNs_p  Pulse width [ns]

\ingroup module_openmac
*/
//------------------------------------------------------------------------------
void openmac_timerIrqEnable(UINT timer_p, UINT32 pulseWidthNs_p)
{
    UINT    offset;
    UINT32  value;

    switch (timer_p)
    {
        case HWTIMER_SYNC:
            offset = OPENMAC_TIMER_OFFSET_CTRL;
            value = 1;
            break;

        case HWTIMER_EXT_SYNC:
            offset = OPENMAC_TIMER_OFFSET_2ND_CTRL;
            value = 1 | (OMETH_NS_2_TICKS(pulseWidthNs_p) << 1);
            break;

        default:
            return;
    }

    Xil_Out32(OPENMAC_TIMER_BASE + offset, value);
}

//------------------------------------------------------------------------------
/**
\brief  Set timer interrupt compare value

This function sets the timer interrupt compare value to the given instance.

\param  timer_p         Timer instance to be disabled
\param  val_p           Compare value

\ingroup module_openmac
*/
//------------------------------------------------------------------------------
void openmac_timerSetCompareValue(UINT timer_p, UINT32 val_p)
{
    UINT offset;

    switch (timer_p)
    {
        case HWTIMER_SYNC:
            offset = OPENMAC_TIMER_OFFSET_CMP_VAL;
            break;

        case HWTIMER_EXT_SYNC:
            offset = OPENMAC_TIMER_OFFSET_2ND_CMP_VAL;
            break;

        default:
            return;
    }

    Xil_Out32(OPENMAC_TIMER_BASE + offset, val_p);
}

//------------------------------------------------------------------------------
/**
\brief  Get timer value

This function gets the current timer instance value.

\param  timer_p         Timer instance to be disabled

\return The function returns the current timer instance value.
        It returns 0, if the instance does not provide the current value.

\ingroup module_openmac
*/
//------------------------------------------------------------------------------
UINT32 openmac_timerGetTimeValue(UINT timer_p)
{
    UINT offset;

    switch (timer_p)
    {
        case HWTIMER_SYNC:
            offset = OPENMAC_TIMER_OFFSET_TIME_VAL;
            break;

        case HWTIMER_EXT_SYNC:
        default:
            return 0;
    }

    return Xil_In32(OPENMAC_TIMER_BASE + offset);
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief  Interrupt handler

This function is invoked by openMAC's interrupt requests. The function calls
the registered interrupt callbacks.

\param  pArg_p      Argument holds the interrupt source

\return The function returns the current timer instance value.
        It returns 0, if the instance does not provide the current value.
*/
//------------------------------------------------------------------------------
static void irqHandler(void* pArg_p)
{
    tOpenmacIrqSource   irqSource = (tOpenmacIrqSource)pArg_p;

    // Check if given argument is within array range.
    if (irqSource >= kOpenmacIrqLast)
        return;

    // Invoke callback with argument for corresponding source.
    if (instance_l.pfnIrqCb[irqSource] != NULL)
        instance_l.pfnIrqCb[irqSource](instance_l.pIrqCbArg[irqSource]);
}

///\}
