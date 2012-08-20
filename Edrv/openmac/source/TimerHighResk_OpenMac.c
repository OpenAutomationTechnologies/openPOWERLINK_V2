/****************************************************************************

  (c) SYSTEC electronic GmbH, D-07973 Greiz, August-Bebel-Str. 29
      www.systec-electronic.com
  (c) Bernecker + Rainer Industrie-Elektronik Ges.m.b.H.
      A-5142 Eggelsberg, B&R Strasse 1
      www.br-automation.com


  Project:      openPOWERLINK

  Description:  implementation of high resolution timer module
                using the MAC-Time Compare unit of the openMAC

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

#include "EplInc.h"
#include "kernel/EplTimerHighResk.h"
#include "Benchmark.h"

#include "omethlib.h"
#ifdef __NIOS2__
#include "system.h"
#include <sys/alt_irq.h>
#include <alt_types.h>
#include <io.h>
#elif defined(__MICROBLAZE__)
#include "xparameters.h"
#include "xil_io.h"
#include "xintc_l.h"
#else
#error
#endif

#ifdef __NIOS2__
#define THRK_RD32(base, offset)             IORD_32DIRECT(base, offset)
#define THRK_WR32(base, offset, write)      IOWR_32DIRECT(base, offset, write)
#elif defined(__MICROBLAZE__)
#define THRK_RD32(base, offset)             Xil_In32((base+offset))
#define THRK_WR32(base, offset, write)      Xil_Out32((base+offset), write)
#else
#error "Configuration unknown!"
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

#define TIMER_COUNT                 1   // only slice timer supported
#define TIMER_STEP_NS               20

#define TIMERCMP_REG_OFF_CTRL       4
#define TIMERCMP_REG_OFF_CMP_VAL    0
#define TIMERCMP_REG_OFF_STATUS     4
#define TIMERCMP_REG_OFF_TIME_VAL   0

//--- set the system's base addresses ---
#if defined(__NIOS2__)

//POWERLINK IP-Core in "pcp_0" subsystem
#if defined(PCP_0_POWERLINK_0_MAC_REG_BASE)
#include "EdrvOpenMac_qsys.h"

//POWERLINK IP-Core in SOPC
#elif defined(POWERLINK_0_MAC_REG_BASE)
#include "EdrvOpenMac_sopc.h"

#else
#error "POWERLINK IP-Core is not found in Nios II (sub-)system!"
#endif

#elif defined(__MICROBLAZE__)

//POWERLINK IP-Core with PLB
#if defined(POWERLINK_USES_PLB_BUS)
#include "EdrvOpenMac_plb.h"

#elif defined(POWERLINK_USES_AXI_BUS)
#include "EdrvOpenMac_axi.h"

#else
#error "POWERLINK IP-Core is not found in Microblaze system!"
#endif

#else
#error "Configuration unknown!"
#endif

#define TIMERHDL_MASK               0x0FFFFFFF
#define TIMERHDL_SHIFT              28

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
    tEplTimerHighReskTimerInfo  m_TimerInfo;

} tEplTimerHighReskInstance;

//---------------------------------------------------------------------------
// local vars
//---------------------------------------------------------------------------

static tEplTimerHighReskInstance    EplTimerHighReskInstance_l;

//---------------------------------------------------------------------------
// local function prototypes
//---------------------------------------------------------------------------

static inline void  EplTimerHighReskCompareInterruptDisable (void);
static inline void  EplTimerHighReskCompareInterruptEnable  (void);
static inline DWORD EplTimerHighReskGetTimeValue            (void);
static inline void  EplTimerHighReskSetCompareValue         (DWORD dwVal);

static void EplTimerHighReskInterruptHandler (void* pArg_p
#ifndef ALT_ENHANCED_INTERRUPT_API_PRESENT
        , DWORD dwInt_p
#endif
        );


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
tEplKernel      Ret;

    Ret = kEplSuccessful;

    EPL_MEMSET(&EplTimerHighReskInstance_l, 0, sizeof (EplTimerHighReskInstance_l));

    EplTimerHighReskCompareInterruptDisable();
    EplTimerHighReskSetCompareValue( 0 );

#ifdef __NIOS2__
    if (alt_ic_isr_register(HIGHRES_TIMER_IRQ_IC_ID, HIGHRES_TIMER_IRQ,
            EplTimerHighReskInterruptHandler, NULL, NULL))
    {
        Ret = kEplNoResource;
    }
#elif defined(__MICROBLAZE__)
    {
        DWORD curIntEn = THRK_RD32(EPL_TIMER_INTC_BASE, XIN_IER_OFFSET);

        XIntc_RegisterHandler(EPL_TIMER_INTC_BASE, HIGHRES_TIMER_IRQ,
                (XInterruptHandler)EplTimerHighReskInterruptHandler, (void*)NULL);
        XIntc_EnableIntr(EPL_TIMER_INTC_BASE, HIGHRES_TIMER_IRQ_MASK | curIntEn);
    }
#else
#error"Configuration unknown!"
#endif
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


    EplTimerHighReskCompareInterruptDisable();
    EplTimerHighReskSetCompareValue( 0 );

#ifdef __NIOS2__
    alt_ic_isr_register(HIGHRES_TIMER_IRQ_IC_ID, HIGHRES_TIMER_IRQ,
            NULL, NULL, NULL);
#elif defined(__MICROBLAZE__)
    XIntc_RegisterHandler(EPL_TIMER_INTC_BASE, HIGHRES_TIMER_IRQ,
            (XInterruptHandler)NULL, (void*)NULL);
#else
#error "Configuration unknown!"
#endif

    EPL_MEMSET(&EplTimerHighReskInstance_l, 0, sizeof (EplTimerHighReskInstance_l));

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
DWORD                       dwTimeNs;
DWORD                       dwTimeSteps;

    // check pointer to handle
    if (pTimerHdl_p == NULL)
    {
        Ret = kEplTimerInvalidHandle;
        goto Exit;
    }

    if (fContinuously_p != FALSE)
    {
        Ret = kEplTimerNoTimerCreated;
        goto Exit;
    }

    if (*pTimerHdl_p == 0)
    {   // no timer created yet
        uiIndex = 0;
        if (EplTimerHighReskInstance_l.m_TimerInfo.m_pfnCallback != NULL)
        {   // no free structure found
            Ret = kEplTimerNoTimerCreated;
            goto Exit;
        }
    }
    else
    {
        uiIndex = (*pTimerHdl_p >> TIMERHDL_SHIFT) - 1;
        if (uiIndex >= TIMER_COUNT)
        {   // invalid handle
            Ret = kEplTimerInvalidHandle;
            goto Exit;
        }
    }

    // modify slice timer
    pTimerInfo = &EplTimerHighReskInstance_l.m_TimerInfo;
    EplTimerHighReskCompareInterruptDisable();

    // increment timer handle (if timer expires right after this statement,
    // the user would detect an unknown timer handle and discard it)
    // => unused in this implementation, as the timer can always be stopped
    pTimerInfo->m_EventArg.m_TimerHdl = ((pTimerInfo->m_EventArg.m_TimerHdl + 1) & TIMERHDL_MASK)
                                        | ((uiIndex + 1) << TIMERHDL_SHIFT);

    *pTimerHdl_p = pTimerInfo->m_EventArg.m_TimerHdl;

    pTimerInfo->m_EventArg.m_Arg.m_dwVal = ulArgument_p;
    pTimerInfo->m_pfnCallback = pfnCallback_p;

    // calculate counter
    if (ullTimeNs_p > 0xFFFFFFFF)
    {   // time is too large, so decrease it to the maximum time
        dwTimeNs = 0xFFFFFFFF;
    }
    else
    {
        dwTimeNs = (DWORD) ullTimeNs_p;
    }

    if (dwTimeNs < 10000)
    {   // time is too less, so increase it to the minimum time
        dwTimeNs = 10000;
    }

    dwTimeSteps = OMETH_NS_2_TICKS(dwTimeNs);

    EplTimerHighReskSetCompareValue( EplTimerHighReskGetTimeValue() + dwTimeSteps);

    // enable timer
    EplTimerHighReskCompareInterruptEnable();

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

    pTimerInfo = &EplTimerHighReskInstance_l.m_TimerInfo;

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
        if (pTimerInfo->m_EventArg.m_TimerHdl != *pTimerHdl_p)
        {   // invalid handle
            goto Exit;
        }
    }

    pTimerInfo->m_pfnCallback = NULL;

    *pTimerHdl_p = 0;

    EplTimerHighReskCompareInterruptDisable();
    EplTimerHighReskSetCompareValue( 0 );

Exit:
    return Ret;

}



//=========================================================================//
//                                                                         //
//          P R I V A T E   F U N C T I O N S                              //
//                                                                         //
//=========================================================================//

static inline void EplTimerHighReskCompareInterruptDisable (void)
{
    THRK_WR32(HIGHRES_TIMER_BASE, TIMERCMP_REG_OFF_CTRL, 0 );
}

static inline void EplTimerHighReskCompareInterruptEnable (void)
{
    THRK_WR32( HIGHRES_TIMER_BASE, TIMERCMP_REG_OFF_CTRL, 1 );
}

static inline void EplTimerHighReskSetCompareValue (DWORD dwVal)
{
    THRK_WR32( HIGHRES_TIMER_BASE, TIMERCMP_REG_OFF_CMP_VAL, dwVal );
}

static inline DWORD EplTimerHighReskGetTimeValue (void)
{
    return THRK_RD32( HIGHRES_TIMER_BASE, TIMERCMP_REG_OFF_TIME_VAL );
}

static void EplTimerHighReskInterruptHandler (void* pArg_p
#ifndef ALT_ENHANCED_INTERRUPT_API_PRESENT
        , DWORD dwInt_p
#endif
        )
{

    BENCHMARK_MOD_24_SET(4);

    EplTimerHighReskSetCompareValue(0);
    EplTimerHighReskCompareInterruptDisable();

    if (EplTimerHighReskInstance_l.m_TimerInfo.m_pfnCallback != NULL)
    {
        EplTimerHighReskInstance_l.m_TimerInfo.m_pfnCallback(&EplTimerHighReskInstance_l.m_TimerInfo.m_EventArg);
    }

    BENCHMARK_MOD_24_RESET(4);

    return;

}
