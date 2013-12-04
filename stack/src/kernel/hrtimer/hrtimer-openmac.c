/**
********************************************************************************
\file   hrtimer-openmac.c

\brief  Implementation of openMAC high-resolution timer module

This file contains the implementation of the openMAC high-resolution timer module.

\ingroup module_hrtimer
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2013, SYSTEC electronic GmbH
Copyright (c) 2013, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
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
#include <global.h>
#include <Epl.h>

#include <kernel/EplTimerHighResk.h>
#include <openmac.h>
#include <omethlib.h>

#include <Benchmark.h>

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
#define TIMER_COUNT                 1   // only slice timer supported
#define TIMER_STEP_NS               20

#define TIMERHDL_MASK               0x0FFFFFFF
#define TIMERHDL_SHIFT              28

//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------
typedef struct
{
    tEplTimerEventArg    eventArg;
    tEplTimerkCallback   pfnCb;

} tTimerInfo;

typedef struct
{
    tTimerInfo  timerInfo;

} tTimerInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tTimerInstance instance_l;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static void drvInterruptHandler(void* pArg_p) SECTION_HRTIMER_IRQ_HDL;

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Initialize high-resolution timer

This function initializes the high-resolution timer module.

\return The function returns a tEplKernel error code.

\ingroup module_hrtimer
*/
//------------------------------------------------------------------------------
tEplKernel EplTimerHighReskInit(void)
{
    return EplTimerHighReskAddInstance();
}

//------------------------------------------------------------------------------
/**
\brief  Add high-resolution timer instance

This function adds a high-resolution timer module instance.

\return The function returns a tEplKernel error code.

\ingroup module_hrtimer
*/
//------------------------------------------------------------------------------
tEplKernel EplTimerHighReskAddInstance(void)
{
    tEplKernel ret = kEplSuccessful;

    EPL_MEMSET(&instance_l, 0, sizeof (instance_l));

    openmac_timerIrqDisable(HWTIMER_SYNC);
    openmac_timerSetCompareValue(HWTIMER_SYNC, 0);

    ret = openmac_isrReg(kOpenmacIrqSync, drvInterruptHandler, NULL);

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Delete high-resolution timer instance

This function deletes the high-resolution timer module instance.

\return The function returns a tEplKernel error code.

\ingroup module_hrtimer
*/
//------------------------------------------------------------------------------
tEplKernel EplTimerHighReskDelInstance(void)
{
    tEplKernel ret = kEplSuccessful;

    openmac_timerIrqDisable(HWTIMER_SYNC);
    openmac_timerSetCompareValue(HWTIMER_SYNC, 0);

    openmac_isrReg(kOpenmacIrqSync, NULL, NULL);

    EPL_MEMSET(&instance_l, 0, sizeof (instance_l));

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Modify high-resolution timer instance timeout

This function modifies the timer handles timeout.
If the handle the pointer points to is zero, the timer must be created first.
If it is not possible to stop the old timer, this function always assures
that the old timer does not trigger the callback function with the same handle
as the new timer.
That means the callback function must check the passed handle with the one
returned by this function. If these are unequal, the call can be discarded.

\param  pTimerHdl_p     Pointer to timer handle
\param  timeNs_p        Relative timeout in [ns]
\param  pfnCb_p         Callback function, which is called mutual exclusive
                        with the Edrv callback functions (Rx and Tx).
\param  arg_p           User-specific argument
\param  fContinuously_p If TRUE, callback function will be called continuously,
                        otherwise, it is a oneshot timer.

\return The function returns a tEplKernel error code.

\ingroup module_hrtimer
*/
//------------------------------------------------------------------------------
tEplKernel EplTimerHighReskModifyTimerNs(tEplTimerHdl* pTimerHdl_p,
        ULONGLONG timeNs_p, tEplTimerkCallback pfnCb_p,
        ULONG arg_p, BOOL fContinuously_p)
{
    tEplKernel  ret = kEplSuccessful;
    UINT        index;
    tTimerInfo* pTimerInfo;
    UINT32      timeNs;
    UINT32      timeSteps;

    // check pointer to handle
    if (pTimerHdl_p == NULL)
    {
        ret = kEplTimerInvalidHandle;
        goto Exit;
    }

    if (fContinuously_p != FALSE)
    {
        ret = kEplTimerNoTimerCreated;
        goto Exit;
    }

    if (*pTimerHdl_p == 0)
    {   // no timer created yet
        index = 0;
        if (instance_l.timerInfo.pfnCb != NULL)
        {   // no free structure found
            ret = kEplTimerNoTimerCreated;
            goto Exit;
        }
    }
    else
    {
        index = (*pTimerHdl_p >> TIMERHDL_SHIFT) - 1;
        if (index >= TIMER_COUNT)
        {   // invalid handle
            ret = kEplTimerInvalidHandle;
            goto Exit;
        }
    }

    // modify slice timer
    pTimerInfo = &instance_l.timerInfo;
    openmac_timerIrqDisable(HWTIMER_SYNC);

    // increment timer handle (if timer expires right after this statement,
    // the user would detect an unknown timer handle and discard it)
    // => unused in this implementation, as the timer can always be stopped
    pTimerInfo->eventArg.m_TimerHdl = ((pTimerInfo->eventArg.m_TimerHdl + 1) & TIMERHDL_MASK)
                                        | ((index + 1) << TIMERHDL_SHIFT);

    *pTimerHdl_p = pTimerInfo->eventArg.m_TimerHdl;

    pTimerInfo->eventArg.m_Arg.m_dwVal = arg_p;
    pTimerInfo->pfnCb = pfnCb_p;

    // calculate counter
    if (timeNs_p > 0xFFFFFFFF)
    {   // time is too large, so decrease it to the maximum time
        timeNs = 0xFFFFFFFF;
    }
    else
    {
        timeNs = (UINT32) timeNs_p;
    }

    if (timeNs < 10000)
    {   // time is too less, so increase it to the minimum time
        timeNs = 10000;
    }

    timeSteps = OMETH_NS_2_TICKS(timeNs);

    timeSteps += openmac_timerGetTimeValue(HWTIMER_SYNC);
    openmac_timerSetCompareValue(HWTIMER_SYNC, timeSteps);

    // enable timer
    openmac_timerIrqEnable(HWTIMER_SYNC, 0);

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Delete high-resolution timer instance timeout

This function deletes the timer with the specified handle. Afterwards the handle
is set to zero.

\param  pTimerHdl_p     Pointer to timer handle

\return The function returns a tEplKernel error code.

\ingroup module_hrtimer
*/
//------------------------------------------------------------------------------
tEplKernel EplTimerHighReskDeleteTimer(tEplTimerHdl* pTimerHdl_p)
{
    tEplKernel  ret = kEplSuccessful;
    UINT        index;
    tTimerInfo* pTimerInfo;

    // check pointer to handle
    if (pTimerHdl_p == NULL)
    {
        ret = kEplTimerInvalidHandle;
        goto Exit;
    }

    pTimerInfo = &instance_l.timerInfo;

    if (*pTimerHdl_p == 0)
    {   // no timer created yet
        goto Exit;
    }
    else
    {
        index = (*pTimerHdl_p >> TIMERHDL_SHIFT) - 1;
        if (index >= TIMER_COUNT)
        {   // invalid handle
            ret = kEplTimerInvalidHandle;
            goto Exit;
        }
        if (pTimerInfo->eventArg.m_TimerHdl != *pTimerHdl_p)
        {   // invalid handle
            goto Exit;
        }
    }

    pTimerInfo->pfnCb = NULL;

    *pTimerHdl_p = 0;

    openmac_timerIrqDisable(HWTIMER_SYNC);
    openmac_timerSetCompareValue(HWTIMER_SYNC, 0);

Exit:
    return ret;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief  Interrupt handler

This function is invoked by the openMAC HW sync timer interrupt.
*/
//------------------------------------------------------------------------------
static void drvInterruptHandler (void* pArg_p)
{
    BENCHMARK_MOD_24_SET(4);

    openmac_timerSetCompareValue(HWTIMER_SYNC, 0);
    openmac_timerIrqDisable(HWTIMER_SYNC);

    if (instance_l.timerInfo.pfnCb != NULL)
    {
        instance_l.timerInfo.pfnCb(&instance_l.timerInfo.eventArg);
    }

    BENCHMARK_MOD_24_RESET(4);
}

///\}
