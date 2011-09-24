/*******************************************************************************
  File:         hpetTimer.c

  (c) Bernecker + Rainer Ges.m.b.H.,  B&R Strasse 1, 5142 Eggelsberg, Austria
      www.br-automation.com

  Project:      openPOWERLINK

  Description:

    The hpetTimer module implements the timer device functions of the high-
    resolution timer library for the Intel HPET timer device.

    The hpet timer driver uses hpet timer 3. It will be used to generate a
    clock timebase. Every time an overrun is detected a variable will be
    incremented and therefore a 64bit counter is built. To be able to detect an
    overrun it has to be ensured that the clock will be read within the 32-bit
    timebase.


  License:

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    1. Redistributions of source code must retain the above copyright
       notice, this list of conditions and the following disclaimer.

    2. Redistributions in binary form must reproduce the above copyright
       notice, this list of conditions and the following disclaimer in the
       documentation and/or other materials provided with the distribution.

    3. Neither the name of Bernecker + Rainer Ges.m.b.H nor the names of its
       contributors may be used to endorse or promote products derived
       from this software without prior written permission. For written
       permission, please contact office@br-automation.com.

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

*******************************************************************************/

//============================================================================//
// Includes                                                                   //
//============================================================================//
#include "hpetTimer.h"
#include "hrtimer.h"
#include "hrtimerLib.h"

#include <intLib.h>
#include <iv.h>
#include <drv/pci/pciIntLib.h>
#include <logLib.h>
#include <sysLib.h>

//============================================================================//
// Global variables                                                           //
//============================================================================//
// Period of HPET in femptoseconds
unsigned int uiHpetPeriod_l;

// Period of higher part of timer counter implemented by variable
// highCounter_g
unsigned long long ullHpetPeriodHigh_l;

// Higher part of clock counter. It will be incremented on every clock
// overrun.
unsigned int uiHighCounter_l;

// Calculation factor to calculate counter from timeout value
unsigned long long ullCalcFactor_l;

// Pointer to installed interrupt handler
VOIDFUNCPTR pfnIntHandler_l;

// Argument to interrupt handler function
int iIntHandlerArg_l;

// Last counter value
unsigned int iLastCounter_l;

//============================================================================//
// Function declarations                                                      //
//============================================================================//
static void hpet_irq(int arg_p);

//============================================================================//
// Local functions                                                            //
//============================================================================//

//------------------------------------------------------------------------------
//
// Function: hpet_writel
//
// Description:
//    Write a long word into a HPET register
//
// Parameters:
//    uiAddress_p   pointer to register
//    uiData_p      data to write
//
// Return:    void
//------------------------------------------------------------------------------
static inline void hpet_writel(unsigned int uiAddress_p, unsigned int uiData_p)
{
    *(unsigned int *)(HPET_ADRS + uiAddress_p) = uiData_p;

}

//------------------------------------------------------------------------------
//
// Function: hpet_readl
//
// Description:
//    Read a long word from a HPET register. The function returns the register
//    contents.
//
// Parameters:
//    uiAddress_p       pointer to register
//
// Return:    unsigned int
//------------------------------------------------------------------------------
static inline unsigned int hpet_readl(unsigned int uiAddress_p)
{
    return *(unsigned int *)(HPET_ADRS + uiAddress_p);
}

//------------------------------------------------------------------------------
//
// Function: hpet_irq
//
// Description:
//    This Interrupt routine will be connected to HPET timer 2 to get
//    timer interrupts
//
// Parameters:
//    iArg_p     not used
//
// Return:    void
//------------------------------------------------------------------------------
static void hpet_irq(int iArg_p)
{
    unsigned int    iReg;

    while (1)
    {
        iReg = hpet_readl(HPET_OFF_STATUS);

        if (iReg & HPET_TIM_STAT_T02)
        {
            /* we are level triggered, therefore acknowledge interrupt */
            hpet_writel(HPET_OFF_STATUS, HPET_TIM_STAT_T02);

            if (pfnIntHandler_l != NULL)
            {
                pfnIntHandler_l(iIntHandlerArg_l);
            }
        }
        else
        {
            break;
        }
    }
}

//============================================================================//
// Functions                                                                  //
//============================================================================//

//------------------------------------------------------------------------------
//
// Function: timerdev_init
//
// Description:
//    Initialize the timer device. Returns kHrtimerReturnOk on success or
//    kHrtimerReturnError on error.
//
// Parameters:
//    none
//
// Return:    int
//------------------------------------------------------------------------------
int timerdev_init(void)
{
    unsigned int        iReg;

    /* read the HPET period */
    uiHpetPeriod_l = hpet_readl(HPET_OFF_GCAP_PERIOD);
    /* calculate high period in 10 psec units because we get too much error
     * if we use nsec and an overflow if we use psec! */
    ullHpetPeriodHigh_l = ((unsigned long long)uiHpetPeriod_l << 32) / FSEC_PER_10PSEC;
    /* The calculation factor is used to calculate counter value from
     * nanoseconds */
    ullCalcFactor_l = (FSEC_PER_NSEC << 32) / uiHpetPeriod_l;

    EPL_DBGLVL_TIMERH_TRACE3 ("%s() calcFactor_g = %lu hpetPeriodHigh_g = %lu\n",
               __func__, ullCalcFactor_l, ullHpetPeriodHigh_l);

    /* reset timer */
    timerdev_stop();
    timerdev_reset();

    /* reset interrupt handler data */
    pfnIntHandler_l = NULL;
    iIntHandlerArg_l = 0;

    /* setup interrupt handler */
    if (intConnect (INUM_TO_IVEC(HPET_INT_LVL), hpet_irq, 0) == ERROR)
    {
        EPL_DBGLVL_ERROR_TRACE1 ("%s() Couldn't connect PCI interrupt\n", __func__);
        return kHrtimerReturnError;
    }

    /* setup timer 2 */
    iReg = HPET_TIM_CONF_LEVEL | HPET_TIM_CONF_32BIT;
    iReg |= (HPET_INT_NUM << HPET_TIM_CONF_ROUTE_SHIFT);    // set to interrupt 11
    hpet_writel(HPET_OFF_TIM_CONF(2), iReg);

    /* Reset interrupt of timer 2 */
    hpet_writel(HPET_OFF_STATUS, HPET_TIM_STAT_T02);

    /* enable interrupts */
    sysIntEnablePIC(HPET_INT_LVL);

    /* start timers */
    timerdev_start();

    return kHrtimerReturnOk;
}

//------------------------------------------------------------------------------
//
// Function: timerdev_shutdown
//
// Description:
//    Shutdown timer device.
//
// Parameters:
//    none
//
// Return:    void
//------------------------------------------------------------------------------
void timerdev_shutdown(void)
{
    /* stop timers */
    timerdev_stop();

    /* disable interrupt */
    sysIntDisablePIC(HPET_INT_LVL);

    timerdev_reset();

    uiHighCounter_l = 0;

    /* reset interrupt handler data */
    pfnIntHandler_l = NULL;
    iIntHandlerArg_l = 0;
}

//------------------------------------------------------------------------------
//
// Function: timerdev_arm
//
// Description:
//    Arms the hpet timer with the specified timeout
//
// Parameters:
//    ullDiffTimeout_p  relative timeout in nanoseconds
//
// Return:    void
//------------------------------------------------------------------------------
void timerdev_arm(unsigned long long ullDiffTimeout_p)
{
    unsigned int        uiCfg, uiCounter, uiNewCounter;
    unsigned long long  ullDelta;

    /* setup comparator value */
    ullDelta = (ullDiffTimeout_p * ullCalcFactor_l) >> 32;
    uiCounter = hpet_readl(HPET_OFF_MAIN_CNT);
    uiNewCounter = uiCounter + ullDelta;

    hpet_writel(HPET_OFF_TIM_CMP(2), uiNewCounter);

    EPL_DBGLVL_TIMERH_TRACE4 ("%s() -> setup hpet %lu counter:%08x -> %08x\n",
               __func__, ullDiffTimeout_p, uiCounter, uiNewCounter);

    /* enable timer interrupt */
    uiCfg = hpet_readl(HPET_OFF_TIM_CONF(2));
    uiCfg |= (HPET_TIM_CONF_32BIT | HPET_TIM_CONF_ENABLE);
    hpet_writel(HPET_OFF_TIM_CONF(2), uiCfg);
}

//------------------------------------------------------------------------------
//
// Function: timerdev_disarm
//
// Description:
//    Disarms the hpet timer
//
// Parameters:
//    none
//
// Return:    void
//------------------------------------------------------------------------------
void timerdev_disarm(void)
{
    unsigned int    uiCfg;

    /* Disable interrupt */
    uiCfg = hpet_readl(HPET_OFF_TIM_CONF(2));
    uiCfg &= ~HPET_TIM_CONF_ENABLE;
    hpet_writel(HPET_OFF_TIM_CONF(2), uiCfg);
}

//------------------------------------------------------------------------------
//
// Function: timerdev_readClock
//
// Description:
//    Reads the current clock value. It returns kHrtimerReturnOk.
//
// Parameters:
//    pUllClock_p       pointer to clock value in nanoseconds
//
// Return:    int
//------------------------------------------------------------------------------
int timerdev_readClock(unsigned long long* pUllClock_p)
{
    unsigned int        uiCounter;
    unsigned long long  ullNsec;
    int                 iLockKey;

    iLockKey = intLock();
    uiCounter = hpet_readl(HPET_OFF_MAIN_CNT);

    /* check if overrun occured since last read. If it is we had an overrun and
     * have to increment the higher counter part. */
    if (uiCounter <= iLastCounter_l)
    {
        uiHighCounter_l++;
    }
    iLastCounter_l = uiCounter;
    intUnlock(iLockKey);

    if (pUllClock_p != NULL)
    {
        ullNsec = (uiHighCounter_l * ullHpetPeriodHigh_l) / NSEC_PER_10PSEC;
        ullNsec += ((unsigned long long)uiCounter * uiHpetPeriod_l) / FSEC_PER_NSEC;
        *pUllClock_p = ullNsec;
    }

    return kHrtimerReturnOk;
}

//------------------------------------------------------------------------------
//
// Function: timerdev_registerInterruptHandler
//
// Description:
//    Register an interrupt handler for the timer interrupt.
//
// Parameters:
//    pfnHandler_p =  Pointer to interrupt handler function
//
//    iArg_p =  Function pointer to interrupt handler function
//
// Return:    int
//------------------------------------------------------------------------------
int timerdev_registerInterruptHandler(VOIDFUNCPTR pfnHandler_p, int iArg_p)
{
    pfnIntHandler_l = pfnHandler_p;
    iIntHandlerArg_l = iArg_p;

    return 0;
}

//------------------------------------------------------------------------------
//
// Function: timerdev_start
//
// Description:
//    Start the hpet counter
//
// Parameters:
//    none
//
// Return:    void
//------------------------------------------------------------------------------
void timerdev_start(void)
{
    unsigned int    uiCfg;

    /* start HPET counter */
    uiCfg = hpet_readl(HPET_OFF_GEN_CONF);
    uiCfg |= HPET_GEN_CONF_ENA;
    hpet_writel(HPET_OFF_GEN_CONF, uiCfg);
}

//------------------------------------------------------------------------------
//
// Function: timerdev_stop
//
// Description:
//    Stop the hpet counter
//
// Parameters:
//    none
//
// Return:    void
//------------------------------------------------------------------------------
void timerdev_stop(void)
{
    unsigned int    uiCfg;

    /* stop HPET counter */
    uiCfg = hpet_readl(HPET_OFF_GEN_CONF);
    uiCfg &= ~HPET_GEN_CONF_ENA;
    hpet_writel(HPET_OFF_GEN_CONF, uiCfg);
}

//------------------------------------------------------------------------------
//
// Function: timerdev_reset
//
// Description:
//    Reset the hpet counter
//
// Parameters:
//    none
//
// Return:    void
//------------------------------------------------------------------------------
void timerdev_reset(void)
{
    /* reset the HPET counter */
    hpet_writel(HPET_OFF_MAIN_CNT, 0);

    /* reset upper 32bit counter */
    iLastCounter_l = 0;
}

//------------------------------------------------------------------------------
//
// Function: timerdev_show
//
// Description:
//    Show timer device information
//
// Parameters:
//    none
//
// Return:    void
//------------------------------------------------------------------------------
void timerdev_show(void)
{
    unsigned int    iReg;

    printf ("----- HPET -----\n");
    iReg = hpet_readl(HPET_OFF_GCAP_ID);
    printf ("GCAP_ID: %08x\n", iReg);
    iReg = hpet_readl(HPET_OFF_GCAP_PERIOD);
    printf ("GCAP_ID(PERIOD): %08x\n", iReg);
    iReg = hpet_readl(HPET_OFF_GEN_CONF);
    printf ("GEN_CONF: %08x\n", iReg);
    iReg = hpet_readl(HPET_OFF_STATUS);
    printf ("GINTR_STA: %08x\n", iReg);
    iReg = hpet_readl(HPET_OFF_MAIN_CNT);
    printf ("MAIN_CNT: %08x\n", iReg);
    printf ("-- TIMER 0 --\n");
    iReg = hpet_readl(HPET_OFF_TIM_CONF(0));
    printf ("TIM_CONF(0): %08x\n", iReg);
    iReg = hpet_readl(HPET_OFF_TIM_CMP(0));
    printf ("TIM_CMP(0): %08x\n", iReg);
    printf ("-- TIMER 1 --\n");
    iReg = hpet_readl(HPET_OFF_TIM_CONF(1));
    printf ("TIM_CONF(1): %08x\n", iReg);
    iReg = hpet_readl(HPET_OFF_TIM_CMP(1));
    printf ("TIM_CMP(1): %08x\n", iReg);
    printf ("-- TIMER 2 --\n");
    iReg = hpet_readl(HPET_OFF_TIM_CONF(2));
    printf ("TIM_CONF(2): %08x\n", iReg);
    iReg = hpet_readl(HPET_OFF_TIM_CMP(2));
    printf ("TIM_CMP(2): %08x\n\n", iReg);
}
