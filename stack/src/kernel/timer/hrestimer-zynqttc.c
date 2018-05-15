/**
********************************************************************************
\file   hrestimer-zynqttc.c

\brief  High-resolution timer module for Zynq linux kernel module

This module is the target specific implementation of the high-resolution
timer module for Linux kernelspace running on Zynq platform.

The module uses Triple Timer Counter (TTC) device 1 for providing the
one-shot and continuous timer support of POWERLINK module.

\ingroup module_hrestimer
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2014, Kalycito Infotech Pvt. Ltd.
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
#include <kernel/hrestimer.h>

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/irq.h>
#include <linux/interrupt.h>

//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#define TIMER_COUNT                         2           // max 2 timers selectable
#define TIMER_MIN_VAL_SINGLE                5000        // min 5us
#define TIMER_MIN_VAL_CYCLE                 100000      // min 100us

#define XTTC_BASE                           0xf8002000  // base addr of TTC1
#define SIZE                                0x00001000
#define XTTC1_TIMERBASE                     0x00000000  // offset of counter 0
#define XTTC2_TIMERBASE                     0x00000004  // offset of counter 1
#define XTTC_IRQ                            69          // Interrupt no
#define XTTC1                               0
#define XTTC2                               1

/*
 * Timer Register Offset Definitions Timer
 */
#define XTTCPSS_CLK_CNTRL_OFFSET            0x00        // Clock Control Reg
#define XTTCPSS_CNT_CNTRL_OFFSET            0x0C        // Counter Control Reg
#define XTTCPSS_COUNT_VAL_OFFSET            0x18        // Counter Value Reg
#define XTTCPSS_INTR_VAL_OFFSET             0x24        // Interval Count Reg
#define XTTCPSS_MATCH_1_OFFSET              0x30        // Match 1 Value Reg
#define XTTCPSS_MATCH_2_OFFSET              0x3C        // Match 2 Value Reg
#define XTTCPSS_MATCH_3_OFFSET              0x48        // Match 3 Value Reg
#define XTTCPSS_ISR_OFFSET                  0x54        // Interrupt Status Reg
#define XTTCPSS_IER_OFFSET                  0x60        // Interrupt Enable Reg

#define XTTCPSS_INTR_MATCH_1                0x02
#define XTTCPSS_INTR_MATCH_2                0x04
#define XTTCPSS_INTR_INTERVAL               0x01
#define XTTCPSS_CLEAR                       0x0000
#define XTTCPSS_CNT_CNTRL_EN_WAVE           0x20
#define XTTCPSS_CNT_CNTRL_RST               0x10
#define XTTCPSS_CNT_CNTRL_MATCH             0x08
#define XTTCPSS_CNT_CNTRL_DECR              0x04
#define XTTCPSS_CNT_CNTRL_INTERVAL          0x02
#define XTTCPSS_CNT_CNTRL_DISABLE           0x01
#define XTTCPSS_CNT_CNTRL_DISABLE_MASK      0x01

/* Setup the timers to use pre-scaling, using a fixed value for now that will work
 * across most input frequency, but it may need to be more dynamic
 */
#define PRESCALE_EXPONENT                   8       // 2 ^ PRESCALE_EXPONENT = PRESCALE
#define PRESCALE                            256     // The exponent must match this
#define CLK_CNTRL_PRESCALE                  (((PRESCALE_EXPONENT - 1) << 1) | 0x1)
#define TTC_RESOLUTION_FACTOR               2304    // Resolution factor for counter. This depends on the source clock frequency for the TTC and the prescaler value.
                                                    // Current design uses a source clock frequency of 111 MHz.

#define NSEC_TO_COUNT(timeout)                      ((UINT)timeout / TTC_RESOLUTION_FACTOR)

#define XTTCPSS_READ_REG(index, offset)             __raw_readl(pTtcBaseAddr_l[index] + offset)

#define XTTCPSS_WRITE_REG(index, offset, val)       __raw_writel(val, (pTtcBaseAddr_l[index] + offset))

#define TIMERHDL_MASK                       0x0FFFFFFF
#define TIMERHDL_SHIFT                      28
#define HDL_TO_IDX(hdl)                     ((hdl >> TIMERHDL_SHIFT) - 1)
#define HDL_INIT(idx)                       ((idx + 1) << TIMERHDL_SHIFT)
#define HDL_INC(hdl)                        (((hdl + 1) & TIMERHDL_MASK) | \
                                             (hdl & ~TIMERHDL_MASK))

//------------------------------------------------------------------------------
// module global vars
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// global function prototypes
//------------------------------------------------------------------------------

//============================================================================//
//          P R I V A T E   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------

/**
\brief  High-resolution timer info

The structure provides information for a high-resolution timer.
*/
typedef struct
{
    tTimerEventArg  eventArg;                   ///< Argument for timer event
    tTimerkCallback pfnCallback;                ///< Timer callback function
    UINT            index;                      ///< Timer index
    BOOL            fContinuously;              ///< Determines if it is a continuous or one-shot timer
} tHresTimerInfo;

/**
\brief  High-resolution timer instance

The structure defines a high-resolution timer module instance.
*/
typedef struct
{
    tHresTimerInfo  aTimerInfo[TIMER_COUNT];    ///< Array with timer information for a set of timers
    void*           pIoAddr;                    ///< Pointer to register space of Timer/Counter unit
} tHresTimerInstance;

//------------------------------------------------------------------------------
// module local vars
//------------------------------------------------------------------------------
static tHresTimerInstance       hresTimerInstance_l;
static void* __iomem            pTtcBaseAddr_l[TIMER_COUNT];

//---------------------------------------------------------------------------
// local function prototypes
//---------------------------------------------------------------------------
static irqreturn_t timerCounterIsr(int irqNum_p, void* pDevInstData_p)

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief    Initialize high-resolution timer module

The function initializes the high-resolution timer module.

\return Returns a tOplkError error code.

\ingroup module_hrestimer
*/
//------------------------------------------------------------------------------
tOplkError hrestimer_init(void)
{
    int                 result = 0;
    UINT                index;
    tHresTimerInfo*     pTimerInfo;
    UINT8               reg;
    unsigned int        irqNo;

    OPLK_MEMSET(&hresTimerInstance_l, 0, sizeof(tHresTimerInstance));

    hresTimerInstance_l.pIoAddr = ioremap(XTTC_BASE, SIZE);
    if (hresTimerInstance_l.pIoAddr == NULL)
    {
        return kErrorNoResource;
    }
    PRINTF("%s: IoAddr=%p\n", __func__, hresTimerInstance_l.pIoAddr);

    pTtcBaseAddr_l[XTTC1] = (void*)(hresTimerInstance_l.pIoAddr + XTTC1_TIMERBASE);
    pTtcBaseAddr_l[XTTC2] = (void*)(hresTimerInstance_l.pIoAddr + XTTC2_TIMERBASE);

    pTimerInfo = &hresTimerInstance_l.aTimerInfo[0];

    for (index = 0; index < TIMER_COUNT; index++, pTimerInfo++)
    {
        pTimerInfo->index = index;
        irqNo = XTTC_IRQ + index;
        result = request_irq(irqNo, timerCounterIsr, IRQF_DISABLED, "Zynq", pTimerInfo);
        PRINTF("%s: interrupt%d registered (return=%d)\n", __func__, pTimerInfo->index, result);

        if (result != 0)
        {
            iounmap(hresTimerInstance_l.pIoAddr);
            return kErrorNoResource;
        }

        reg = XTTCPSS_CNT_CNTRL_DISABLE;
        XTTCPSS_WRITE_REG(index, XTTCPSS_CNT_CNTRL_OFFSET, reg);

        reg = CLK_CNTRL_PRESCALE;
        XTTCPSS_WRITE_REG(index, XTTCPSS_CLK_CNTRL_OFFSET, reg);

        // set the necessary counter control param , for now we keep the timer disabled
        reg = 0;
        reg = XTTCPSS_READ_REG(index, XTTCPSS_CNT_CNTRL_OFFSET);
        reg |= (XTTCPSS_CNT_CNTRL_EN_WAVE);
        XTTCPSS_WRITE_REG(index, XTTCPSS_CNT_CNTRL_OFFSET, reg);

        // clear all interrupts
        reg = 0;
        XTTCPSS_WRITE_REG(index, XTTCPSS_IER_OFFSET, reg);
    }

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief    Shut down high-resolution timer module

The function shuts down the high-resolution timer module.

\return Returns a tOplkError error code.

\ingroup module_hrestimer
*/
//------------------------------------------------------------------------------
tOplkError hrestimer_exit(void)
{
    UINT8               reg;
    UINT                index;
    tHresTimerInfo*     pTimerInfo;
    unsigned int        irqNo;

    pTimerInfo = &hresTimerInstance_l.aTimerInfo[0];
    for (index = 0; index < TIMER_COUNT; index++, pTimerInfo++)
    {
        reg = XTTCPSS_READ_REG(pTimerInfo->index, XTTCPSS_CNT_CNTRL_OFFSET);
        // set the necessary counter control parameter, for now we keep the timer disabled
        reg = (XTTCPSS_CNT_CNTRL_DISABLE);
        XTTCPSS_WRITE_REG(pTimerInfo->index, XTTCPSS_CNT_CNTRL_OFFSET, reg);
        irqNo = XTTC_IRQ + pTimerInfo->index;
        free_irq(irqNo, pTimerInfo);
    }

    iounmap(hresTimerInstance_l.pIoAddr);

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief    Modify a high-resolution timer

The function modifies the timeout of the timer with the specified handle.
If the handle to which the pointer points to is zero, the timer must be created
first. If it is not possible to stop the old timer, this function always assures
that the old timer does not trigger the callback function with the same handle
as the new timer. That means the callback function must check the passed handle
with the one returned by this function. If these are unequal, the call can be
discarded.

\param[in,out]  pTimerHdl_p         Pointer to timer handle.
\param[in]      time_p              Relative timeout in [ns].
\param[in]      pfnCallback_p       Callback function, which is called when timer expires.
                                    (The function is called mutually exclusive with
                                    the Edrv callback functions (Rx and Tx)).
\param[in]      argument_p          User-specific argument.
\param[in]      fContinue_p         If TRUE, the callback function will be called continuously.
                                    Otherwise, it is a one-shot timer.

\return Returns a tOplkError error code.

\ingroup module_hrestimer
*/
//------------------------------------------------------------------------------
tOplkError hrestimer_modifyTimer(tTimerHdl* pTimerHdl_p,
                                 ULONGLONG time_p,
                                 tTimerkCallback pfnCallback_p,
                                 ULONG argument_p,
                                 BOOL fContinue_p)
{
    tHresTimerInfo*     pTimerInfo;
    ULONGLONG           counter = 0;
    UINT                index;
    UINT8               reg;

    // check pointer to handle
    if (pTimerHdl_p == NULL)
    {
        return kErrorTimerInvalidHandle;
    }

    if (*pTimerHdl_p == 0)
    {
        // no timer created yet
        // search free timer info structure
        pTimerInfo = &hresTimerInstance_l.aTimerInfo[0];
        for (index = 0; index < TIMER_COUNT; index++, pTimerInfo++)
        {
            if (pTimerInfo->eventArg.timerHdl.handle == 0)
            {
                // free structure found
                break;
            }
        }
        if (index >= TIMER_COUNT)
        {
            // no free structure found
            return kErrorTimerNoTimerCreated;
        }

        pTimerInfo->eventArg.timerHdl.handle = HDL_INIT(index);
    }
    else
    {
        index = HDL_TO_IDX(*pTimerHdl_p);
        if (index >= TIMER_COUNT)
        {
            // invalid handle
            return kErrorTimerInvalidHandle;
        }

        pTimerInfo = &hresTimerInstance_l.aTimerInfo[index];
    }

    // increment timer handle (if timer expires right after this statement,
    // the user would detect an unknown timer handle and discard it)
    pTimerInfo->eventArg.timerHdl.handle = HDL_INC(pTimerInfo->eventArg.timerHdl.handle);
    *pTimerHdl_p = pTimerInfo->eventArg.timerHdl.handle;

    // Adjust the Timeout if its to small
    if (fContinue_p != FALSE)
    {
        if (time_p < TIMER_MIN_VAL_CYCLE)
        {
            time_p = TIMER_MIN_VAL_CYCLE;
        }
    }
    else
    {
        if (time_p < TIMER_MIN_VAL_SINGLE)
        {
            time_p = TIMER_MIN_VAL_SINGLE;
        }
    }

    // Get the counter value from the timeout
    counter = (ULONGLONG)NSEC_TO_COUNT(time_p);
    if (counter > 0xFFFF)
    {
        return kErrorTimerNoTimerCreated;
    }

    pTimerInfo->eventArg.argument.value = argument_p;
    pTimerInfo->pfnCallback = pfnCallback_p;

    // disable the Timer
    reg = 0;
    reg = XTTCPSS_READ_REG(pTimerInfo->index, XTTCPSS_CNT_CNTRL_OFFSET);
    reg |= XTTCPSS_CNT_CNTRL_DISABLE;
    XTTCPSS_WRITE_REG(pTimerInfo->index, XTTCPSS_CNT_CNTRL_OFFSET, reg);

    if (fContinue_p != FALSE)
    {
        pTimerInfo->fContinuously = fContinue_p;
        // Set the interval for continuous timer
        XTTCPSS_WRITE_REG(pTimerInfo->index, XTTCPSS_INTR_VAL_OFFSET, (UINT16)counter);

        // Enable the interval interrupt
        reg = XTTCPSS_READ_REG(pTimerInfo->index, XTTCPSS_IER_OFFSET);
        reg = XTTCPSS_INTR_INTERVAL;
        XTTCPSS_WRITE_REG(pTimerInfo->index, XTTCPSS_IER_OFFSET, reg);

        // Set specific values in Counter control reg
        reg = XTTCPSS_READ_REG(pTimerInfo->index, XTTCPSS_CNT_CNTRL_OFFSET);
        reg |= (XTTCPSS_CNT_CNTRL_RST | XTTCPSS_CNT_CNTRL_INTERVAL);
        XTTCPSS_WRITE_REG(pTimerInfo->index, XTTCPSS_CNT_CNTRL_OFFSET, reg);
    }
    else
    {
        pTimerInfo->fContinuously = fContinue_p;
        // Set match counter for one-shot timer
        XTTCPSS_WRITE_REG(pTimerInfo->index, XTTCPSS_MATCH_1_OFFSET, (UINT16)counter);
        // Enable the Match1 interrupt
        reg = XTTCPSS_READ_REG(pTimerInfo->index, XTTCPSS_IER_OFFSET);
        reg = XTTCPSS_INTR_MATCH_1;
        XTTCPSS_WRITE_REG(pTimerInfo->index, XTTCPSS_IER_OFFSET, reg);

        // Set specific values in Counter control reg
        reg = XTTCPSS_READ_REG(pTimerInfo->index, XTTCPSS_CNT_CNTRL_OFFSET);
        reg |= (XTTCPSS_CNT_CNTRL_MATCH | XTTCPSS_CNT_CNTRL_RST);
        XTTCPSS_WRITE_REG(pTimerInfo->index, XTTCPSS_CNT_CNTRL_OFFSET, reg);
    }

    // Re-enable the timer
    reg = XTTCPSS_READ_REG(pTimerInfo->index, XTTCPSS_CNT_CNTRL_OFFSET);
    reg &= ~XTTCPSS_CNT_CNTRL_DISABLE;
    XTTCPSS_WRITE_REG(pTimerInfo->index, XTTCPSS_CNT_CNTRL_OFFSET, reg);

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief    Delete a high-resolution timer

The function deletes a created high-resolution timer. The timer is specified
by its timer handle. After deleting, the handle is reset to zero.

\param[in,out]  pTimerHdl_p         Pointer to timer handle.

\return Returns a tOplkError error code.

\ingroup module_hrestimer
*/
//------------------------------------------------------------------------------
tOplkError hrestimer_deleteTimer(tTimerHdl* pTimerHdl_p)
{
    tOplkError          ret = kErrorOk;
    UINT                index;
    tHresTimerInfo*     pTimerInfo;
    UINT8               reg;

    // check pointer to handle
    if (pTimerHdl_p == NULL)
    {
        return kErrorTimerInvalidHandle;
    }

    if (*pTimerHdl_p == 0)
    {
        // no timer created yet
        return ret;
    }
    else
    {
        index = HDL_TO_IDX(*pTimerHdl_p);
        if (index >= TIMER_COUNT)
        {
            // invalid handle
            return kErrorTimerInvalidHandle;
        }
        pTimerInfo = &hresTimerInstance_l.aTimerInfo[index];
        if (pTimerInfo->eventArg.timerHdl.handle != *pTimerHdl_p)
        {
            // invalid handle
            return ret;
        }
    }

    *pTimerHdl_p = 0;
    pTimerInfo->eventArg.timerHdl.handle = 0;
    pTimerInfo->pfnCallback = NULL;

    // Disable the interrupt for this timer
    reg = 0;
    XTTCPSS_WRITE_REG(pTimerInfo->index, XTTCPSS_IER_OFFSET, reg);

    // Stop the timer
    reg = XTTCPSS_READ_REG(pTimerInfo->index, XTTCPSS_CNT_CNTRL_OFFSET);
    reg = 0;
    reg |= (XTTCPSS_CNT_CNTRL_DISABLE | XTTCPSS_CNT_CNTRL_EN_WAVE);
    XTTCPSS_WRITE_REG(pTimerInfo->index, XTTCPSS_CNT_CNTRL_OFFSET, reg);

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Control external synchronization interrupt

This function enables/disables the external synchronization interrupt. If the
external synchronization interrupt is not supported, the call is ignored.

\param[in]      fEnable_p           Flag determines if sync should be enabled or disabled.

\ingroup module_hrestimer
*/
//------------------------------------------------------------------------------
void hrestimer_controlExtSyncIrq(BOOL fEnable_p)
{
    UNUSED_PARAMETER(fEnable_p);
}

//------------------------------------------------------------------------------
/**
\brief  Set external synchronization interrupt time

This function sets the time when the external synchronization interrupt shall
be triggered to synchronize the host processor. If the external synchronization
interrupt is not supported, the call is ignored.

\param[in]      time_p              Time when the sync shall be triggered

\ingroup module_hrestimer
*/
//------------------------------------------------------------------------------
void hrestimer_setExtSyncIrqTime(tTimestamp time_p)
{
    UNUSED_PARAMETER(time_p);
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief    TTC timer interrupt controller

Interrupt service routine for the triple timer counter timer module used by
POWERLINK kernel module on Xilinx Zynq platform.

\param[in]      irqNum_p            IRQ number
\param[in,out]  pDevInstData_p      Pointer to private data provided by request_irq

\return Returns a interrupt handled status.
*/
//---------------------------------------------------------------------------------
static irqreturn_t timerCounterIsr(int irqNum_p, void* pDevInstData_p)
{
    irqreturn_t         ret = IRQ_HANDLED;
    UINT                index;
    tHresTimerInfo*     pTimerInfo = (tHresTimerInfo*)pDevInstData_p;
    UINT8               reg = 0;

    UNUSED_PARAMETER(irqNum_p);

    reg = XTTCPSS_READ_REG(pTimerInfo->index, XTTCPSS_ISR_OFFSET);
    if (reg == 0)
    {
        return IRQ_NONE;
    }

    // Acknowledge the Interrupt
    XTTCPSS_WRITE_REG(pTimerInfo->index, XTTCPSS_ISR_OFFSET, reg);
    index = HDL_TO_IDX(pTimerInfo->eventArg.timerHdl.handle);

    if (index >= TIMER_COUNT)
    {
        // invalid handle
        return ret;
    }

    if (!(reg & (XTTCPSS_INTR_MATCH_1)) && !(reg & XTTCPSS_INTR_INTERVAL))
    {
        // unknown interrupt
        return ret;
    }

    if (!pTimerInfo->fContinuously)
    {
        // Disable the interrupt for this timer
        reg = XTTCPSS_READ_REG(pTimerInfo->index, XTTCPSS_IER_OFFSET);
        reg = 0;
        XTTCPSS_WRITE_REG(pTimerInfo->index, XTTCPSS_IER_OFFSET, reg);

        // stop the timer
        reg = 0;
        reg = XTTCPSS_READ_REG(pTimerInfo->index, XTTCPSS_CNT_CNTRL_OFFSET);
        reg |= XTTCPSS_CNT_CNTRL_DISABLE;
        XTTCPSS_WRITE_REG(pTimerInfo->index, XTTCPSS_CNT_CNTRL_OFFSET, reg);

        // Clear the Match counter
        XTTCPSS_WRITE_REG(pTimerInfo->index, XTTCPSS_MATCH_1_OFFSET,
                          XTTCPSS_CLEAR);
    }

    if (pTimerInfo->pfnCallback != NULL)
    {
        // call the timer callback
        pTimerInfo->pfnCallback(&pTimerInfo->eventArg);
    }

    return ret;
}

/// \}
