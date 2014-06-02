/**
********************************************************************************
\file   altera_nios2/openmac-nios2.c

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

#include <system.h>

#include <sys/alt_cache.h>
#include <sys/alt_irq.h>
#include <alt_types.h>

#include <io.h>
#include <unistd.h>

#include <target/openmac.h>


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
#define OPENMAC_SYNC_IRQ    0
#define OPENMAC_TXRX_IRQ    1
#define OPENMAC_IRQ_IC_ID   0

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
static void irqHandler(void* pArg_p
#ifndef ALT_ENHANCED_INTERRUPT_API_PRESENT
        , UINT32 int_p
#endif
        ) SECTION_EDRVOPENMAC_IRQ_HDL;

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
    UINT32      icId;

    icId = OPENMAC_IRQ_IC_ID;

    switch (irqSource_p)
    {
        case kOpenmacIrqSync:
            irqId = OPENMAC_SYNC_IRQ;
            break;

        case kOpenmacIrqTxRx:
            irqId = OPENMAC_TXRX_IRQ;
            break;

        default:
            ret = kErrorNoResource;
            goto Exit;
    }

    if (alt_ic_isr_register(icId, irqId, irqHandler, (void*)irqSource_p, NULL))
    {
        return kErrorNoResource;
    }

    instance_l.pfnIrqCb[irqSource_p] = pfnIsrCb_p;
    instance_l.pIrqCbArg[irqSource_p] = pArg_p;

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Set a memory uncached

This function sets the given memory region as uncached.

\param  pMem_p      Base address of memory region
\param  size_p      Size of memory region

\return The function returns the base address of the uncached memory.

\ingroup module_openmac
*/
//------------------------------------------------------------------------------
UINT8* openmac_memUncached(UINT8* pMem_p, UINT size_p)
{
    return (UINT8*)alt_remap_uncached(pMem_p, size_p);
}

//------------------------------------------------------------------------------
/**
\brief  Allocated uncached memory

This function allocates memory and marks it as uncached.

\param  size_p      Size of uncached memory to be allocated

\return The function returns the base address of the allocated, uncached memory.

\ingroup module_openmac
*/
//------------------------------------------------------------------------------
UINT8* openmac_uncachedMalloc(UINT size_p)
{
    return (UINT8*)alt_uncached_malloc(size_p);
}

//------------------------------------------------------------------------------
/**
\brief  Free uncached memory

This function frees the uncached memory pMem_p.

\param  pMem_p      Uncached memory to be freed

\ingroup module_openmac
*/
//------------------------------------------------------------------------------
void openmac_uncachedFree(UINT8* pMem_p)
{
    alt_uncached_free(pMem_p);
}

//------------------------------------------------------------------------------
/**
\brief  Flush data cache region

This function flushes the data cache associated to the memory region.
Since the packet buffers are allocated by uncached malloc no flush is needed.

\param  pMem_p      Associated memory region address
\param  size_p      Size of memory region

\ingroup module_openmac
*/
//------------------------------------------------------------------------------
void openmac_flushDataCache(UINT8* pMem_p, UINT size_p)
{
    UNUSED_PARAMETER(pMem_p);
    UNUSED_PARAMETER(size_p);

    return;
}

//------------------------------------------------------------------------------
/**
\brief  Invalidates data cache region

This function invalidates the data cache associated to the memory region.
Since the packet buffers are allocated by uncached malloc no invalidate is needed.

\param  pMem_p      Associated memory region address
\param  size_p      Size of memory region

\ingroup module_openmac
*/
//------------------------------------------------------------------------------
void openmac_invalidateDataCache(UINT8* pMem_p, UINT size_p)
{
    UNUSED_PARAMETER(pMem_p);
    UNUSED_PARAMETER(size_p);

    return;
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

    return IORD_16DIRECT(OPENMAC_DOB_BASE, 0);
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

    IOWR_32DIRECT(OPENMAC_TIMER_BASE, offset, 0);
}

//------------------------------------------------------------------------------
/**
\brief  Timer interrupt enable

This function enables the timer instance and sets the generated pulse width if
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

    IOWR_32DIRECT(OPENMAC_TIMER_BASE, offset, value);
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

    IOWR_32DIRECT(OPENMAC_TIMER_BASE, offset, val_p);
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

    return IORD_32DIRECT(OPENMAC_TIMER_BASE, offset);
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
\param  int_p       Optional interrupt id

\return The function returns the current timer instance value.
        It returns 0, if the instance does not provide the current value.
*/
//------------------------------------------------------------------------------
static void irqHandler(void* pArg_p
#ifndef ALT_ENHANCED_INTERRUPT_API_PRESENT
        , UINT32 int_p
#endif
        )
{
    tOpenmacIrqSource   irqSource = (tOpenmacIrqSource)pArg_p;

#ifndef ALT_ENHANCED_INTERRUPT_API_PRESENT
    UNUSED_PARAMETER(int_p);
#endif

    // Check if given argument is within array range.
    if (irqSource >= kOpenmacIrqLast)
        return;

    // Invoke callback with argument for corresponding source.
    if (instance_l.pfnIrqCb[irqSource] != NULL)
        instance_l.pfnIrqCb[irqSource](instance_l.pIrqCbArg[irqSource]);
}

///\}
