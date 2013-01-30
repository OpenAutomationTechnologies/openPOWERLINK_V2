/****************************************************************************

  (c) SYSTEC electronic GmbH, D-07973 Greiz, August-Bebel-Str. 29
      www.systec-electronic.com

  Project:      openPOWERLINK

  Description:  target specific implementation of
                high resolution timer module for
                MCF548x (e.g. SYSTEC ECUcore-5484) under Linux

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
#include "kernel/EplTimerHighResk.h"
#include "Benchmark.h"

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
    #include <asm/m5485gpt.h>
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

// The slice timer, the DMA and the FEC interrupt must have the same level but different priorities.
// These interrupts handlers are mutual exclusive.
#define MCF_ILP_SLT                 (MCF_ICR_IL(5) | MCF_ICR_IP(4))
#define MCF_ILP_GPT                 (MCF_ICR_IL(5) | MCF_ICR_IP(3))
#define SLT                         1   // use SLT 1 (0 is used by Linux scheduler)

#define TIMER_COUNT                 2
#define TIMERHDL_MASK               0x0FFFFFFF
#define TIMERHDL_SHIFT              28

#define PRESCALER                   5000    // = 50 us


//---------------------------------------------------------------------------
// module global types
//---------------------------------------------------------------------------

typedef struct
{
    tEplTimerEventArg    m_EventArg;
    tEplTimerkCallback   m_pfnCallback;

} tEplTimerHighReskTimerInfo;

typedef struct
{
    tEplTimerHighReskTimerInfo  m_aTimerInfo[TIMER_COUNT];

} tEplTimerHighReskInstance;

//---------------------------------------------------------------------------
// local vars
//---------------------------------------------------------------------------

static tEplTimerHighReskInstance    EplTimerHighReskInstance_l;

//---------------------------------------------------------------------------
// local function prototypes
//---------------------------------------------------------------------------

static  int         TgtSltIsr (int nIrqNum_p, void* ppDevInstData_p, struct pt_regs* ptRegs_p);

//=========================================================================//
//                                                                         //
//          P U B L I C   F U N C T I O N S                                //
//                                                                         //
//=========================================================================//


//---------------------------------------------------------------------------
//
// Function:    EplTimerHighReskInit()
//
// Description: initializes the high resolution timer module.
//
// Parameters:  void
//
// Return:      tEplKernel      = error code
//
// State:       not tested
//
//---------------------------------------------------------------------------

tEplKernel PUBLIC EplTimerHighReskInit(void)
{
tEplKernel  Ret;

    Ret = EplTimerHighReskAddInstance();

    return Ret;

}


//---------------------------------------------------------------------------
//
// Function:    EplTimerHighReskAddInstance()
//
// Description: initializes the high resolution timer module.
//
// Parameters:  void
//
// Return:      tEplKernel      = error code
//
// State:       not tested
//
//---------------------------------------------------------------------------

tEplKernel PUBLIC EplTimerHighReskAddInstance(void)
{
tEplKernel  Ret = kEplSuccessful;
int         iIrq;
unsigned int                uiIndex;

    EPL_MEMSET(&EplTimerHighReskInstance_l, 0, sizeof (EplTimerHighReskInstance_l));

    // set up SLT interrupt
    iIrq = 64 + ISC_SLTn(SLT);
    MCF_SCR(SLT) = 0;
    MCF_SSR(SLT) = MCF_SSR_ST;
    MCF_ICR(iIrq - 64) = MCF_ILP_SLT;
    request_irq(iIrq, TgtSltIsr, SA_INTERRUPT, "ColdFire Timer 1", &EplTimerHighReskInstance_l.m_aTimerInfo[0]);
    enable_irq (iIrq);

    for (uiIndex = 0; uiIndex < (TIMER_COUNT - 1); uiIndex++)
    {
        MCF_GPT_GMS(uiIndex) = MCF_GPT_GMS_TMS_DISABLE;
        iIrq = 64 + ISC_GPTn(uiIndex);
        MCF_ICR(iIrq - 64) = MCF_ILP_GPT;
        request_irq(iIrq, TgtSltIsr, SA_INTERRUPT, "ColdFire General Purpose Timer", &EplTimerHighReskInstance_l.m_aTimerInfo[uiIndex + 1]);
        enable_irq (iIrq);
    }

    return Ret;

}


//---------------------------------------------------------------------------
//
// Function:    EplTimerHighReskDelInstance()
//
// Description: shuts down the high resolution timer module.
//
// Parameters:  void
//
// Return:      tEplKernel      = error code
//
// State:       not tested
//
//---------------------------------------------------------------------------

tEplKernel PUBLIC EplTimerHighReskDelInstance(void)
{
tEplKernel  Ret = kEplSuccessful;
int         iIrq;
unsigned int                uiIndex;

    iIrq = 64 + ISC_SLTn(SLT);
    MCF_SCR(SLT) = 0;
    MCF_SSR(SLT) = MCF_SSR_ST;
    disable_irq (iIrq);
    free_irq(iIrq, &EplTimerHighReskInstance_l.m_aTimerInfo[0]);

    for (uiIndex = 0; uiIndex < (TIMER_COUNT - 1); uiIndex++)
    {
        MCF_GPT_GMS(uiIndex) = MCF_GPT_GMS_TMS_DISABLE;
        iIrq = 64 + ISC_GPTn(uiIndex);
        disable_irq (iIrq);
        free_irq(iIrq, &EplTimerHighReskInstance_l.m_aTimerInfo[uiIndex + 1]);
    }

    return Ret;

}



//---------------------------------------------------------------------------
//
// Function:    EplTimerHighReskModifyTimerNs()
//
// Description: modifies the timeout of the timer with the specified handle.
//              If the handle the pointer points to is zero, the timer must
//              be created first.
//              If it is not possible to stop the old timer,
//              this function always assures that the old timer does not
//              trigger the callback function with the same handle as the new
//              timer. That means the callback function must check the passed
//              handle with the one returned by this function. If these are
//              unequal, the call can be discarded.
//
// Parameters:  pTimerHdl_p     = pointer to timer handle
//              ullTimeNs_p     = relative timeout in [ns]
//              pfnCallback_p   = callback function, which is called mutual
//                                exclusive with the Edrv callback functions
//                                (Rx and Tx).
//              ulArgument_p    = user-specific argument
//              fContinuously_p = if TRUE, callback function will be called
//                                continuously;
//                                otherwise, it is a oneshot timer.
//
// Return:      tEplKernel      = error code
//
// State:       not tested
//
//---------------------------------------------------------------------------

tEplKernel PUBLIC EplTimerHighReskModifyTimerNs(tEplTimerHdl*     pTimerHdl_p,
                                    unsigned long long  ullTimeNs_p,
                                    tEplTimerkCallback  pfnCallback_p,
                                    unsigned long       ulArgument_p,
                                    BOOL                fContinuously_p)
{
tEplKernel                  Ret = kEplSuccessful;
unsigned int                uiIndex;
tEplTimerHighReskTimerInfo* pTimerInfo;
DWORD                       dwTime;

    // check pointer to handle
    if (pTimerHdl_p == NULL)
    {
        Ret = kEplTimerInvalidHandle;
        goto Exit;
    }

    if (*pTimerHdl_p == 0)
    {   // no timer created yet
        // search free timer info structure
        pTimerInfo = &EplTimerHighReskInstance_l.m_aTimerInfo[0];
        for (uiIndex = 0; uiIndex < TIMER_COUNT; uiIndex++, pTimerInfo++)
        {
            if (pTimerInfo->m_pfnCallback == NULL)
            {   // free structure found
                break;
            }
        }
        if (uiIndex >= TIMER_COUNT)
        {   // no free structure found
            Ret = kEplTimerNoTimerCreated;
            goto Exit;
        }
//        pTimerInfo->m_EventArg.m_TimerHdl = (uiIndex + 1) << TIMERHDL_SHIFT;
    }
    else
    {
        uiIndex = (*pTimerHdl_p >> TIMERHDL_SHIFT) - 1;
        if (uiIndex >= TIMER_COUNT)
        {   // invalid handle
            Ret = kEplTimerInvalidHandle;
            goto Exit;
        }
        pTimerInfo = &EplTimerHighReskInstance_l.m_aTimerInfo[uiIndex];
        // d.k.: assume that this info structure is the correct one
    }

    if (uiIndex == 0)
    {   // Slice timer
        MCF_SCR(SLT) = 0;

        // increment timer handle (if timer expires right after this statement,
        // the user would detect an unknown timer handle and discard it)
        pTimerInfo->m_EventArg.m_TimerHdl = ((pTimerInfo->m_EventArg.m_TimerHdl + 1) & TIMERHDL_MASK)
                                            | ((uiIndex + 1) << TIMERHDL_SHIFT);

        // calculate counter
    //<    ullTimeNs_p = (ullTimeNs_p / (unsigned long)(1000000000L / MCF_BUSCLK));
        dwTime = (DWORD) ((unsigned long) ullTimeNs_p / (1000000000L / MCF_BUSCLK));
        if (dwTime < 255)
        {   // time is too less, so increase it to the minimum time
            dwTime = 255;
        }
        else if (ullTimeNs_p > 0xFFFFFFFF)
        {   // time is too large, so decrease it to the maximum time
            dwTime = 0xFFFFFFFF;
        }
        MCF_SLTCNT(SLT) = dwTime;

        pTimerInfo->m_EventArg.m_Arg.m_dwVal = ulArgument_p;
        pTimerInfo->m_pfnCallback = pfnCallback_p;

        *pTimerHdl_p = pTimerInfo->m_EventArg.m_TimerHdl;

        // enable timer
        MCF_SSR(SLT) = MCF_SSR_ST;
        MCF_SCR(SLT) |=  MCF_SCR_TEN | MCF_SCR_IEN | ((fContinuously_p != FALSE) ? MCF_SCR_RUN : 0);
    }
    else
    {   // General purpose timer
        // decrement uiIndex so it is based on 0
        uiIndex--;
        MCF_GPT_GMS(uiIndex) = MCF_GPT_GMS_TMS_DISABLE;

        // increment timer handle (if timer expires right after this statement,
        // the user would detect an unknown timer handle and discard it)
        pTimerInfo->m_EventArg.m_TimerHdl = ((pTimerInfo->m_EventArg.m_TimerHdl + 1) & TIMERHDL_MASK)
                                            | ((uiIndex + 2) << TIMERHDL_SHIFT);

        // calculate counter
    //<    ullTimeNs_p = (ullTimeNs_p / (unsigned long)(1000000000L / MCF_BUSCLK));
        dwTime = ((unsigned long) ullTimeNs_p / (1000000000L / MCF_BUSCLK));
        // unit of time is [10ns]
        if (dwTime < 255)
        {   // time is too less, so increase it to the minimum time
            dwTime = 255;
        }
        else if (dwTime <= 0xFFFF)
        {   // time fits into the counter value
            dwTime |= 0x00010000;  // set prescaler
        }
        else if (dwTime > (0xFFFF * PRESCALER))
        {   // time is too large, so decrease it to the maximum time
            dwTime = 0xFFFF | (PRESCALER << 16);
        }
        else
        {
            dwTime /= PRESCALER;
            // unit of time is [50us]
            dwTime |= (PRESCALER << 16); // set prescaler
        }

        MCF_GPT_GCIR(uiIndex) = dwTime;

        pTimerInfo->m_EventArg.m_Arg.m_dwVal = ulArgument_p;
        pTimerInfo->m_pfnCallback = pfnCallback_p;

        *pTimerHdl_p = pTimerInfo->m_EventArg.m_TimerHdl;

        // enable timer
        MCF_GPT_GMS(uiIndex) = MCF_GPT_GMS_TMS_GPIO | MCF_GPT_GMS_IEN | MCF_GPT_GMS_CE
                               | ((fContinuously_p != FALSE) ? MCF_GPT_GMS_SC : 0);
    }

Exit:
    return Ret;

}


//---------------------------------------------------------------------------
//
// Function:    EplTimerHighReskDeleteTimer()
//
// Description: deletes the timer with the specified handle. Afterward the
//              handle is set to zero.
//
// Parameters:  pTimerHdl_p     = pointer to timer handle
//
// Return:      tEplKernel      = error code
//
// State:       not tested
//
//---------------------------------------------------------------------------

tEplKernel PUBLIC EplTimerHighReskDeleteTimer(tEplTimerHdl*     pTimerHdl_p)
{
tEplKernel                  Ret = kEplSuccessful;
unsigned int                uiIndex;
tEplTimerHighReskTimerInfo* pTimerInfo;

    // check pointer to handle
    if (pTimerHdl_p == NULL)
    {
        Ret = kEplTimerInvalidHandle;
        goto Exit;
    }

    if (*pTimerHdl_p == 0)
    {   // no timer created yet
        goto Exit;
    }
    else
    {
        uiIndex = (*pTimerHdl_p >> TIMERHDL_SHIFT) - 1;
        if (uiIndex >= TIMER_COUNT)
        {   // invalid handle
            Ret = kEplTimerInvalidHandle;
            goto Exit;
        }
        pTimerInfo = &EplTimerHighReskInstance_l.m_aTimerInfo[uiIndex];
        if (pTimerInfo->m_EventArg.m_TimerHdl != *pTimerHdl_p)
        {   // invalid handle
            goto Exit;
        }
    }

    pTimerInfo->m_pfnCallback = NULL;

    *pTimerHdl_p = 0;

    if (uiIndex == 0)
    {   // Slice timer
        MCF_SCR(SLT) = 0;
        MCF_SSR(SLT) = MCF_SSR_ST;
    }
    else
    {   // General purpose timer
        MCF_GPT_GMS((uiIndex - 2)) = MCF_GPT_GMS_TMS_DISABLE;
    }

    // increment timer handle (if timer expires right after this statement,
    // the user would detect an unknown timer handle and discard it)
//    EplTimerHighReskInstance_l.m_EventArg.m_TimerHdl++;

Exit:
    return Ret;

}



//---------------------------------------------------------------------------
//
// Function:    TgtSltIsr()
//
// Description: target specific interrupt handler for slice timer and
//              general purpose timer.
//
// Parameters:  nIrqNum_p       = IRQ number
//              pDevInstData_p  = pointer to instance structure
//              ptRegs_p        = pointer to register structure
//
// Return:      int
//
// State:       not tested
//
//---------------------------------------------------------------------------

static  int         TgtSltIsr (int nIrqNum_p, void* pDevInstData_p, struct pt_regs* ptRegs_p)
{
unsigned int                uiIndex;
tEplTimerHighReskTimerInfo* pTimerInfo = (tEplTimerHighReskTimerInfo*)pDevInstData_p;

    BENCHMARK_MOD_24_SET(4);

    uiIndex = (pTimerInfo->m_EventArg.m_TimerHdl >> TIMERHDL_SHIFT) - 1;
    if (uiIndex >= TIMER_COUNT)
    {   // invalid handle
        goto Exit;
    }

    if (uiIndex == 0)
    {   // Slice timer
        if ((MCF_SCR(SLT) & MCF_SCR_RUN) == 0)
        {   // completely disable the timer, because it shall be a oneshot timer,
            // otherwise it would start again automatically after clearing the status flag
            MCF_SCR(SLT) = 0;
        }
        MCF_SSR(SLT) = MCF_SSR_ST;
    }
    else
    {   // General purpose timer
        MCF_GPT_GSR((uiIndex - 1)) = MCF_GPT_GSR_TEXP;
    }

    if (pTimerInfo->m_pfnCallback != NULL)
    {
        pTimerInfo->m_pfnCallback(&pTimerInfo->m_EventArg);
    }

Exit:
    BENCHMARK_MOD_24_RESET(4);
    return 0;
}

