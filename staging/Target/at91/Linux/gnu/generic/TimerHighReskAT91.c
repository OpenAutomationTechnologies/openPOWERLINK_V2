/****************************************************************************

  (c) SYSTEC electronic GmbH, D-07973 Greiz, August-Bebel-Str. 29
      www.systec-electronic.com

  Project:      openPOWERLINK

  Description:  target specific implementation of
                high resolution timer module for
                AT91 compatible Timer Counter channels
                (e.g. AT91RM9200) under Linux (2.6.22)

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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/major.h>
#include <linux/version.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/irq.h>
//#include <asm/uaccess.h>
//#include <asm/atomic.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/err.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27)
#include <asm/arch/at91rm9200.h>
#include <asm/arch/at91_tc.h>
#else
#include <mach/at91rm9200.h>
#include <mach/at91_tc.h>
#endif

#include <asm/div64.h>


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


#define FIRST_USED_TC_UNIT          0   // TC0 is the first unit we use
#define TIMER_COUNT                 2   // the used units must not cross block boundaries
                                        // (e.g. TC2 and TC3 cannot be used by this implementation)
#define PRESCALER_COUNT             4

#define TIMERHDL_MASK               0x0FFFFFFF
#define TIMERHDL_SHIFT              28


#define AT91_TC_REG(nTc_p, sReg_p)  (EplTimerHighReskInstance_l.m_pIoAddr + \
                                     ((nTc_p + ((FIRST_USED_TC_UNIT < 3) ? FIRST_USED_TC_UNIT : FIRST_USED_TC_UNIT - 3)) \
                                      << 6) \
                                     + AT91_TC_ ## sReg_p)


//---------------------------------------------------------------------------
// module global types
//---------------------------------------------------------------------------

typedef struct
{
    tEplTimerEventArg    m_EventArg;
    tEplTimerkCallback   m_pfnCallback;
    struct clk*          m_pClk;

} tEplTimerHighReskTimerInfo;

typedef struct
{
    tEplTimerHighReskTimerInfo  m_aTimerInfo[TIMER_COUNT];
    void*                   m_pIoAddr;      // pointer to register space of Timer / Counter unit
    unsigned long long      m_aullMaxTimeoutNs[PRESCALER_COUNT];
    unsigned int            m_auiFreq[PRESCALER_COUNT];   // timer frequencies for each prescaler

} tEplTimerHighReskInstance;


//---------------------------------------------------------------------------
// local vars
//---------------------------------------------------------------------------

static tEplTimerHighReskInstance    EplTimerHighReskInstance_l;

static const unsigned int auiEpltimerHighReskPrescaler_l[PRESCALER_COUNT] =
    { 2, 8, 32, 128 };


//---------------------------------------------------------------------------
// local function prototypes
//---------------------------------------------------------------------------

static irqreturn_t TgtTimerCounterIsr (int nIrqNum_p, void* pDevInstData_p);


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
tEplKernel      Ret = kEplSuccessful;
int             iResult = 0;
int             iIrq;
unsigned int    uiIndex;
char            szClkName[8];
tEplTimerHighReskTimerInfo* pTimerInfo;
unsigned long   ulMck;  // master clock rate

    EPL_MEMSET(&EplTimerHighReskInstance_l, 0, sizeof (EplTimerHighReskInstance_l));

    // fetch the IO address of the timer/counter block
    EplTimerHighReskInstance_l.m_pIoAddr = ioremap(((FIRST_USED_TC_UNIT < 3) ? AT91RM9200_BASE_TCB0 : AT91RM9200_BASE_TCB1),
                                                   256);

    PRINTF("%s: IoAddr=%p\n", __func__, EplTimerHighReskInstance_l.m_pIoAddr);
    if (EplTimerHighReskInstance_l.m_pIoAddr == NULL)
    {
        Ret = kEplNoResource;
        goto Exit;
    }

    pTimerInfo = &EplTimerHighReskInstance_l.m_aTimerInfo[0];

    for (uiIndex = 0; uiIndex < TIMER_COUNT; uiIndex++, pTimerInfo++)
    {
        iResult = snprintf(szClkName, sizeof (szClkName), "tc%u_clk", (FIRST_USED_TC_UNIT + uiIndex));

        pTimerInfo->m_pClk = clk_get(NULL, szClkName);
        PRINTF("%s: Clk '%s'=%p\n", __func__, szClkName, pTimerInfo->m_pClk);
        if (IS_ERR(pTimerInfo->m_pClk))
        {
            Ret = kEplNoResource;
            goto Exit;
        }

        clk_enable(pTimerInfo->m_pClk);
        PRINTF("%s: Clk '%s' enabled\n", __func__, szClkName);

        // disable the clock counter
        __raw_writel(AT91_TC_CLKDIS, AT91_TC_REG(uiIndex, CCR));
        PRINTF("%s: clock counter disabled (reg=%p)\n", __func__, AT91_TC_REG(uiIndex, CCR));

        // disable all interrupts from the timer/counter unit
        __raw_writel(0xFFFFFFFF, AT91_TC_REG(uiIndex, IDR));

        // acknowledge any pending interrupt
        iResult = __raw_readl(AT91_TC_REG(uiIndex, SR));

        // enable the RC compare interrupt
        __raw_writel(AT91_TC_CPCS, AT91_TC_REG(uiIndex, IER));
        PRINTF("%s: RC compare interrupt enabled (reg=%p)\n", __func__, AT91_TC_REG(uiIndex, IER));

        iIrq = AT91RM9200_ID_TC0 + FIRST_USED_TC_UNIT + uiIndex;
        iResult = request_irq(iIrq, TgtTimerCounterIsr, IRQF_SHARED,
                              "Timer/Counter",
                              pTimerInfo);
        PRINTF("%s: interrupt registered (return=%d)\n", __func__, iResult);
        if (iResult != 0)
        {
            Ret = kEplNoResource;
            goto Exit;
        }

    }

    // fetch frequency of MCK (master clock)
    ulMck = clk_get_rate(EplTimerHighReskInstance_l.m_aTimerInfo[0].m_pClk);
    PRINTF("%s: master clock rate = %lu Hz\n", __func__, ulMck);

    // calculate t_max for each prescaler
    for (uiIndex = 0; uiIndex < PRESCALER_COUNT; uiIndex++)
    {
        EplTimerHighReskInstance_l.m_aullMaxTimeoutNs[uiIndex] = 65535000000000LL;
        EplTimerHighReskInstance_l.m_auiFreq[uiIndex] = ulMck / auiEpltimerHighReskPrescaler_l[uiIndex];
        PRINTF("%s: prescaler[%u]=%u Hz)\n", __func__, uiIndex, EplTimerHighReskInstance_l.m_auiFreq[uiIndex]);

        do_div(EplTimerHighReskInstance_l.m_aullMaxTimeoutNs[uiIndex], EplTimerHighReskInstance_l.m_auiFreq[uiIndex]);

        PRINTF("%s: t_max[%u]=%Lu ns)\n", __func__, uiIndex, EplTimerHighReskInstance_l.m_aullMaxTimeoutNs[uiIndex]);
    }


Exit:
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
tEplTimerHighReskTimerInfo* pTimerInfo;

    pTimerInfo = &EplTimerHighReskInstance_l.m_aTimerInfo[0];

    for (uiIndex = 0; uiIndex < TIMER_COUNT; uiIndex++, pTimerInfo++)
    {
        // disable the clock counter
        __raw_writel(AT91_TC_CLKDIS, AT91_TC_REG(uiIndex, CCR));

        // disable all interrupts from the timer/counter unit
        __raw_writel(0xFFFFFFFF, AT91_TC_REG(uiIndex, IDR));

        // acknowledge any pending interrupt
        __raw_readl(AT91_TC_REG(uiIndex, SR));

        clk_disable(pTimerInfo->m_pClk);
        clk_put(pTimerInfo->m_pClk);

        iIrq = AT91RM9200_ID_TC0 + FIRST_USED_TC_UNIT + uiIndex;
        free_irq(iIrq, pTimerInfo);
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
unsigned int                uiPrescaler;
tEplTimerHighReskTimerInfo* pTimerInfo;
WORD                        wCounter;

    // check pointer to handle
    if(pTimerHdl_p == NULL)
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
/*        if ((pTimerInfo->m_EventArg.m_TimerHdl != *pTimerHdl_p)
            && (pTimerInfo->m_pfnCallback == NULL))
        {   // invalid handle
            Ret = kEplTimerInvalidHandle;
            goto Exit;
        }*/
    }

    // disable the timer
    __raw_writel(AT91_TC_CLKDIS, AT91_TC_REG(uiIndex, CCR));

    // increment timer handle (if timer expires right after this statement,
    // the user would detect an unknown timer handle and discard it)
    pTimerInfo->m_EventArg.m_TimerHdl = ((pTimerInfo->m_EventArg.m_TimerHdl + 1) & TIMERHDL_MASK)
                                        | ((uiIndex + 1) << TIMERHDL_SHIFT);

    // select appropriate prescaler
    for (uiPrescaler = 0; uiPrescaler < PRESCALER_COUNT; uiPrescaler++)
    {
        if (ullTimeNs_p <= EplTimerHighReskInstance_l.m_aullMaxTimeoutNs[uiPrescaler])
        {   // appropriate prescaler found
            break;
        }
        // prescaler not sufficient, try next one
    }
    if (uiPrescaler >= PRESCALER_COUNT)
    {   // no appropriate prescaler found, time is too large
        Ret = kEplTimerNoTimerCreated;
        goto Exit;
    }

    // calculate counter
    ullTimeNs_p = ullTimeNs_p
                  * EplTimerHighReskInstance_l.m_auiFreq[uiPrescaler]
                  + 500000000ULL;
    do_div(ullTimeNs_p, 1000000000UL);
    wCounter = (WORD) ullTimeNs_p;

//    PRINTF("%s: [%u] wCounter=%u presc=%u)\n", __func__, uiIndex, wCounter, uiPrescaler);

    // configure the timer unit
    __raw_writel(uiPrescaler | AT91_TC_WAVE | AT91_TC_WAVESEL_UP_AUTO
                | ((fContinuously_p != FALSE) ? 0 : AT91_TC_CPCSTOP),
                AT91_TC_REG(uiIndex, CMR));

    // configure the counter
    __raw_writel(wCounter, AT91_TC_REG(uiIndex, RC));

    pTimerInfo->m_EventArg.m_Arg.m_dwVal = ulArgument_p;
    pTimerInfo->m_pfnCallback = pfnCallback_p;

    *pTimerHdl_p = pTimerInfo->m_EventArg.m_TimerHdl;

    // start timer
    __raw_writel((AT91_TC_SWTRG | AT91_TC_CLKEN), AT91_TC_REG(uiIndex, CCR));

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
    if(pTimerHdl_p == NULL)
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

    // disable the timer
    __raw_writel(AT91_TC_CLKDIS, AT91_TC_REG(uiIndex, CCR));

    // increment timer handle (if timer expires right after this statement,
    // the user would detect an unknown timer handle and discard it)
//    EplTimerHighReskInstance_l.m_EventArg.m_TimerHdl++;

Exit:
    return Ret;

}



//---------------------------------------------------------------------------
//
// Function:    TgtTimerCounterIsr()
//
// Description: target specific interrupt handler for slice timer and
//              general purpose timer.
//
// Parameters:  nIrqNum_p       = IRQ number
//              pDevInstData_p  = pointer to instance structure
//              ptRegs_p        = pointer to register structure
//
// Return:      irqreturn_t
//
// State:       not tested
//
//---------------------------------------------------------------------------

static irqreturn_t TgtTimerCounterIsr (int nIrqNum_p, void* pDevInstData_p)
{
unsigned int                uiIndex;
tEplTimerHighReskTimerInfo* pTimerInfo = (tEplTimerHighReskTimerInfo*)pDevInstData_p;

    BENCHMARK_MOD_24_SET(4);

    uiIndex = (pTimerInfo->m_EventArg.m_TimerHdl >> TIMERHDL_SHIFT) - 1;
    if (uiIndex >= TIMER_COUNT)
    {   // invalid handle
        goto Exit;
    }

    // acknowledge the pending interrupt
    __raw_readl(AT91_TC_REG(uiIndex, SR));

    if (pTimerInfo->m_pfnCallback != NULL)
    {
        pTimerInfo->m_pfnCallback(&pTimerInfo->m_EventArg);
    }

Exit:
    BENCHMARK_MOD_24_RESET(4);
    return IRQ_HANDLED;
}

