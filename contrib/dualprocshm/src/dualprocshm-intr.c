/**
********************************************************************************
\file   dualprocshm-intr.c

\brief  Dual Processor Library - Interrupt Handling module

This file contains implementation of interrupt handling for dual processor library.

Interrupt handling support for interrupts from single processor is only supported
till now.

\ingroup module_dualprocshm
*******************************************************************************/
/*------------------------------------------------------------------------------
Copyright (c) 2014 Kalycito Infotech Private Limited
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
#include <dualprocshm.h>

#include <stdlib.h>
#include <string.h>
//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#define TARGET_MAX_INTERRUPTS    4                              ///< Max interrupts supported
//------------------------------------------------------------------------------
// module global vars
//------------------------------------------------------------------------------

/**
\brief Interrupt Registers

The interrupt handling registers for dual processor interface
*/
typedef struct sDualProcShmIntrReg
{
    UINT16    irqEnable;                                        ///< Enable irqs
    union
    {
        volatile UINT16     irqSet;                             ///< Set irq (Pcp)
        volatile UINT16     irqAck;                             ///< Acknowledge irq (Host)
        volatile UINT16     irqPending;                         ///< Pending irq
    }irq;
}tDualProcShmIntrReg;

/**
\brief Interrupt instance

The interrupt instance structure used to store interrupt
information for the local instance.
*/
typedef struct
{
    tTargetIrqCb            apfnIrqCb[TARGET_MAX_INTERRUPTS];   ///< User applications interrupt callbacks
    tDualProcShmIntrReg*    intrReg;                            ///< Pointer to interrupt register
}tDualProcShmIntrInst;

//------------------------------------------------------------------------------
// global function prototypes
//------------------------------------------------------------------------------

//============================================================================//
//            P R I V A T E   D E F I N I T I O N S                           //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//-----------------------------------------------------------------------------
//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
tDualProcShmIntrInst    intrInst_l;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static void targetInterruptHandler(void* pArg_p) SECTION_DUALPROCSHM_IRQ_HDL;

//------------------------------------------------------------------------------
/**
\brief  Initialize Interrupts for the platform

The function registers the common platform interrupt handler and enable the
interrupts

\param  pInstance_p      Driver instance

\return The function returns a tDualprocReturn Error code.

\ingroup module_dualprocshm
*/
//------------------------------------------------------------------------------
tDualprocReturn dualprocshm_initInterrupts(tDualprocDrvInstance pInstance_p)
{
    tDualProcDrv*   pDrvInst = (tDualProcDrv*) pInstance_p;

    if (pInstance_p == NULL )
        return kDualprocInvalidParameter;

    intrInst_l.intrReg = (tDualProcShmIntrReg*)dualprocshm_getIntrMemAddr();

    if (intrInst_l.intrReg == NULL)
    {
        printf("Error Initializing interrupt routine\n");
        return kDualprocNoResource;
    }

    if (pDrvInst->config.procInstance == kDualProcFirst)
        memset(intrInst_l.intrReg, 0, sizeof(tDualProcShmIntrReg));

    dualprocshm_regSyncIrqHdl(targetInterruptHandler, (void*)pInstance_p);

    // enable system irq
    dualprocshm_enableSyncIrq(TRUE);

    return kDualprocSuccessful;
}

//------------------------------------------------------------------------------
/**
\brief  Free Interrupts for the platform

The function frees the interrupts registered before

\param  pInstance_p      Driver instance

\return The function returns a tDualprocReturn Error code.

\ingroup module_dualprocshm
*/
//------------------------------------------------------------------------------
tDualprocReturn dualprocshm_freeInterrupts(tDualprocDrvInstance pInstance_p)
{
    if (pInstance_p == NULL)
        return kDualprocInvalidParameter;

    dualprocshm_enableSyncIrq(FALSE);
    dualprocshm_regSyncIrqHdl(NULL, NULL);

    intrInst_l.intrReg = NULL;

    return kDualprocSuccessful;
}

//------------------------------------------------------------------------------
/**
\brief  Register interrupt handler

The function registers a interrupt handler for the specified interrupt.

\param  pInstance_p      Driver instance
\param  irqId_p          Interrupt id.
\param  pfnIrqHandler_p  Interrupt handler

\return The function returns a tDualprocReturn Error code.

\ingroup module_dualprocshm
*/
//------------------------------------------------------------------------------
tDualprocReturn dualprocshm_registerHandler(tDualprocDrvInstance pInstance_p,
                                            UINT8 irqId_p, tTargetIrqCb pfnIrqHandler_p)
{
    UINT16    irqEnableVal;

    if (irqId_p > TARGET_MAX_INTERRUPTS || pInstance_p == NULL )
        return kDualprocInvalidParameter;

    if (intrInst_l.intrReg == NULL)
        return kDualprocNoResource;

    DUALPROCSHM_INVALIDATE_DCACHE_RANGE(&intrInst_l.intrReg->irqEnable, sizeof(intrInst_l.intrReg->irqEnable));
    irqEnableVal = DPSHM_READ16(&intrInst_l.intrReg->irqEnable);

    if (pfnIrqHandler_p != NULL)
        irqEnableVal |= (1 << irqId_p);
    else
        irqEnableVal &= ~(1 << irqId_p);

    intrInst_l.apfnIrqCb[irqId_p] = pfnIrqHandler_p;

    DPSHM_WRITE16(&intrInst_l.intrReg->irqEnable, irqEnableVal);
    DUALPROCSHM_FLUSH_DCACHE_RANGE(&intrInst_l.intrReg->irqEnable, sizeof(intrInst_l.intrReg->irqEnable));

    return kDualprocSuccessful;
}

//------------------------------------------------------------------------------
/**
\brief  Enable Interrupt

The function enables the specified interrupt.

\param  pInstance_p      Driver instance
\param  irqId_p          Interrupt id.
\param  fEnable_p        Enable if TRUE, Disable if FALSE

\return The function returns a tDualprocReturn Error code.

\ingroup module_dualprocshm
*/
//------------------------------------------------------------------------------
tDualprocReturn dualprocshm_enableIrq(tDualprocDrvInstance pInstance_p,
                                      UINT8 irqId_p, BOOL fEnable_p)
{
    UINT16    irqEnableVal;

    if (irqId_p > TARGET_MAX_INTERRUPTS || pInstance_p == NULL )
        return kDualprocInvalidParameter;

    if (intrInst_l.intrReg == NULL)
        return kDualprocNoResource;

    DUALPROCSHM_INVALIDATE_DCACHE_RANGE(&intrInst_l.intrReg->irqEnable, sizeof(intrInst_l.intrReg->irqEnable));
    irqEnableVal = DPSHM_READ16(&intrInst_l.intrReg->irqEnable);

    if (fEnable_p)
        irqEnableVal |= (1 << irqId_p);
    else
        irqEnableVal &= ~(1 << irqId_p);

    DPSHM_WRITE16(&intrInst_l.intrReg->irqEnable, irqEnableVal);
    DUALPROCSHM_FLUSH_DCACHE_RANGE(&intrInst_l.intrReg->irqEnable, sizeof(intrInst_l.intrReg->irqEnable));

    return kDualprocSuccessful;
}

//------------------------------------------------------------------------------
/**
\brief  Set a Interrupt

The function triggers the specified interrupt.

\param  pInstance_p      Driver instance
\param  irqId_p          Interrupt id.
\param  fSet_p           Trigger if TRUE, clear if FALSE

\return The function returns a tDualprocReturn Error code.

\ingroup module_dualprocshm
*/
//------------------------------------------------------------------------------
tDualprocReturn dualprocshm_setIrq(tDualprocDrvInstance pInstance_p, UINT8 irqId_p, BOOL fSet_p)
{
    UINT16      irqActive;
    UINT16      irqEnable;

    if (irqId_p > TARGET_MAX_INTERRUPTS || pInstance_p == NULL)
        return kDualprocInvalidParameter;

    if (intrInst_l.intrReg == NULL)
        return kDualprocNoResource;

    DUALPROCSHM_INVALIDATE_DCACHE_RANGE(&intrInst_l.intrReg->irqEnable, sizeof(intrInst_l.intrReg->irqEnable));
    irqEnable = DPSHM_READ16(&intrInst_l.intrReg->irqEnable);

    if (irqEnable & (1 << irqId_p))
    {
        irqActive = DPSHM_READ16(&intrInst_l.intrReg->irq.irqSet);

        if (fSet_p)
            irqActive |= (1 << irqId_p);
        else
            irqActive &= ~(1 << irqId_p);

        DPSHM_WRITE16(&intrInst_l.intrReg->irq.irqSet, irqActive);
        DUALPROCSHM_FLUSH_DCACHE_RANGE(&intrInst_l.intrReg->irq.irqSet, sizeof(intrInst_l.intrReg->irq.irqSet));
    }

    return kDualprocSuccessful;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
//------------------------------------------------------------------------------
/// \name Private Functions
/// \{

/**
\brief  Master Interrupt Handler

This is the master interrupt handler which should by called by the system if the
irq signal is asserted by PCP. This will be used to handle multiple interrupt sources
with a single interrupt line available. This handler acknowledges the processed
interrupt sources and calls the corresponding callbacks registered with
dualprocshm_registerHandler().

\param  pArg_p                  Driver instance passed during initialisation.

\ingroup module_dualprocshm
*/
//------------------------------------------------------------------------------
static void targetInterruptHandler(void* pArg_p)
{
    UINT16      pendings;
    UINT16      mask;
    int         i;

    UNUSED_PARAMETER(pArg_p);

    if (intrInst_l.intrReg == NULL)
        return;

    DUALPROCSHM_INVALIDATE_DCACHE_RANGE(&intrInst_l.intrReg->irq.irqPending, sizeof(intrInst_l.intrReg->irq.irqPending));
    pendings = DPSHM_READ16(&intrInst_l.intrReg->irq.irqPending);

    for (i = 0; i < TARGET_MAX_INTERRUPTS; i++)
    {
        mask = 1 << i;

        // ack irq source first
        if (pendings & mask)
        {
            pendings &= ~mask;
            DPSHM_WRITE16(&intrInst_l.intrReg->irq.irqAck, pendings);
            DUALPROCSHM_FLUSH_DCACHE_RANGE(&intrInst_l.intrReg->irq.irqAck, sizeof(intrInst_l.intrReg->irq.irqAck));
        }

        // then try to execute the callback
        if (intrInst_l.apfnIrqCb[i] != NULL)
            intrInst_l.apfnIrqCb[i]();
    }
}

///\}
