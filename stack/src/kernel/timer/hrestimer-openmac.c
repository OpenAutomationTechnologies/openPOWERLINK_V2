/**
********************************************************************************
\file   hrestimer-openmac.c

\brief  Implementation of openMAC high-resolution timer module

This file contains the implementation of the openMAC high-resolution timer module.

\ingroup module_hrestimer
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
#include <common/target.h>
#include <kernel/hrestimer.h>
#include <target/openmac.h>
#include <omethlib.h>
#include <oplk/benchmark.h>

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

#define TIMER_MIN_NS                10000

#define TIMERHDL_MASK               0x0FFFFFFF
#define TIMERHDL_SHIFT              28

//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------

/**
\brief  High-resolution timer information structure

The structure contains all necessary information for a high-resolution timer.
*/
typedef struct
{
    tTimerEventArg       eventArg;          ///< Argument for timer event
    tTimerkCallback      pfnCb;             ///< Timer callback function
} tTimerInfo;

/**
\brief  High-resolution timer instance

The structure defines a high-resolution timer module instance.
*/
typedef struct
{
    tTimerInfo          timerInfo;          ///< Timer information for a timer
} tTimerInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tTimerInstance instance_l;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static INLINE tOplkError setupTimerInfo(tTimerHdl* pTimerHdl_p,
                                        tTimerkCallback pfnCallback_p,
                                        ULONG argument_p);
static void drvInterruptHandler(void* pArg_p) SECTION_HRTIMER_IRQ_HDL;

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Initialize high-resolution timer

This function initializes the high-resolution timer module.

\return The function returns a tOplkError error code.

\ingroup module_hrestimer
*/
//------------------------------------------------------------------------------
tOplkError hrestimer_init(void)
{
    tOplkError ret = kErrorOk;

    OPLK_MEMSET(&instance_l, 0, sizeof(instance_l));

    OPENMAC_TIMERIRQDISABLE(HWTIMER_SYNC);
    OPENMAC_TIMERIRQACK(HWTIMER_SYNC);

    ret = openmac_isrReg(kOpenmacIrqSync, drvInterruptHandler, NULL);

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief    Shut down high-resolution timer module

The function shuts down the high-resolution timer module.

\return The function returns a tOplkError error code.

\ingroup module_hrestimer
*/
//------------------------------------------------------------------------------
tOplkError hrestimer_exit(void)
{
    tOplkError ret = kErrorOk;

    OPENMAC_TIMERIRQDISABLE(HWTIMER_SYNC);
    OPENMAC_TIMERIRQACK(HWTIMER_SYNC);

    openmac_isrReg(kOpenmacIrqSync, NULL, NULL);

    OPLK_MEMSET(&instance_l, 0, sizeof(instance_l));

    return ret;
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
    tOplkError  ret = kErrorOk;
    UINT32      timeNs;
    UINT32      timeSteps;

    // check pointer to handle
    if (pTimerHdl_p == NULL)
    {
        ret = kErrorTimerInvalidHandle;
        goto Exit;
    }

    if (fContinue_p != FALSE)
    {
        ret = kErrorTimerNoTimerCreated;
        goto Exit;
    }

    ret = setupTimerInfo(pTimerHdl_p, pfnCallback_p, argument_p);
    if (ret != kErrorOk)
        goto Exit;

    // calculate counter
    if (time_p > 0xFFFFFFFF)
    {   // time is too large, so decrease it to the maximum time
        timeNs = 0xFFFFFFFF;
    }
    else
    {
        timeNs = (UINT32)time_p;
    }

    if (timeNs < TIMER_MIN_NS)
    {   // time is too less, so increase it to the minimum time
        timeNs = TIMER_MIN_NS;
    }

    timeSteps = OMETH_NS_2_TICKS(timeNs);

    timeSteps += OPENMAC_TIMERGETTIMEVALUE();
    OPENMAC_TIMERSETCOMPAREVALUE(HWTIMER_SYNC, timeSteps);

    // enable timer
    OPENMAC_TIMERIRQENABLE(HWTIMER_SYNC);

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Set high-resolution timer to absolute time

The function sets the timer to an absolute timestamp value with the specified
handle. If the handle to which the pointer points to is zero, the timer must be
created first. If it is not possible to stop the old timer, this function always
assures that the old timer does not trigger the callback function with the same
handle as the new timer. That means the callback function must check the passed
handle with the one returned by this function. If these are unequal, the call
can be discarded.

\param[in,out]  pTimerHdl_p         Pointer to timer handle.
\param[in]      time_p              Absolute timestamp when the timer shall expire.
\param[in]      pfnCallback_p       Callback function which is called when the timer expires.
                                    (The function is called mutually exclusive with the Edrv
                                    callback functions (Rx and Tx)).
\param[in]      argument_p          User-specific argument.

\return Returns a tOplkError error code.

\ingroup module_hrestimer
*/
//------------------------------------------------------------------------------
tOplkError hrestimer_setAbsoluteTimer(tTimerHdl* pTimerHdl_p,
                                      tTimestamp time_p,
                                      tTimerkCallback pfnCallback_p,
                                      ULONG argument_p)
{
    tOplkError  ret = kErrorOk;
    INT32       timeDiff;

    // check pointer to handle
    if (pTimerHdl_p == NULL)
    {
        ret = kErrorTimerInvalidHandle;
        goto Exit;
    }

    ret = setupTimerInfo(pTimerHdl_p, pfnCallback_p, argument_p);
    if (ret != kErrorOk)
        goto Exit;

    // Get current time and check if due time expires too soon.
    timeDiff = time_p.timeStamp - OPENMAC_TIMERGETTIMEVALUE();

    if (timeDiff < (INT32)OMETH_NS_2_TICKS(TIMER_MIN_NS))
    {
        // Timer would expire too soon, therefore shift away due time!
        time_p.timeStamp += OMETH_NS_2_TICKS(TIMER_MIN_NS);
    }

    OPENMAC_TIMERSETCOMPAREVALUE(HWTIMER_SYNC, time_p.timeStamp);

    // enable timer
    OPENMAC_TIMERIRQENABLE(HWTIMER_SYNC);

Exit:
    return ret;
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
    tOplkError  ret = kErrorOk;
    UINT        index;
    tTimerInfo* pTimerInfo;

    // check pointer to handle
    if (pTimerHdl_p == NULL)
    {
        ret = kErrorTimerInvalidHandle;
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
            ret = kErrorTimerInvalidHandle;
            goto Exit;
        }
        if (pTimerInfo->eventArg.timerHdl.handle != *pTimerHdl_p)
        {   // invalid handle
            goto Exit;
        }
    }

    pTimerInfo->pfnCb = NULL;

    *pTimerHdl_p = 0;

    OPENMAC_TIMERIRQDISABLE(HWTIMER_SYNC);
    OPENMAC_TIMERIRQACK(HWTIMER_SYNC);

Exit:
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
#ifdef TIMER_USE_EXT_SYNC_INT
    if (fEnable_p)
    {
        OPENMAC_TIMERIRQENABLE(HWTIMER_EXT_SYNC);
    }
    else
    {
        OPENMAC_TIMERIRQDISABLE(HWTIMER_EXT_SYNC);
    }
#else
    UNUSED_PARAMETER(fEnable_p);
#endif //TIMER_USE_EXT_SYNC_INT
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
#ifdef TIMER_USE_EXT_SYNC_INT
    OPENMAC_TIMERSETCOMPAREVALUE(HWTIMER_EXT_SYNC, time_p.timeStamp);
#else
    UNUSED_PARAMETER(time_p);
#endif //TIMER_USE_EXT_SYNC_INT
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief  Set up timer info

The function sets up the timer info of the given handle. If the handle is zero,
the timer must be created first.

\param[in,out]  pTimerHdl_p         Pointer to timer handle.
\param[in]      pfnCallback_p       Callback function which is called when the timer expires.
\param[in]      argument_p          User-specific argument.

\return Returns a tOplkError error code.

*/
//------------------------------------------------------------------------------
static tOplkError setupTimerInfo(tTimerHdl* pTimerHdl_p,
                                 tTimerkCallback pfnCallback_p,
                                 ULONG argument_p)
{
    tOplkError  ret = kErrorOk;
    UINT        index;
    tTimerInfo* pTimerInfo;

    if (*pTimerHdl_p == 0)
    {   // no timer created yet
        index = 0;
        if (instance_l.timerInfo.pfnCb != NULL)
        {   // no free structure found
            ret = kErrorTimerNoTimerCreated;
            goto Exit;
        }
    }
    else
    {
        index = (*pTimerHdl_p >> TIMERHDL_SHIFT) - 1;
        if (index >= TIMER_COUNT)
        {   // invalid handle
            ret = kErrorTimerInvalidHandle;
            goto Exit;
        }
    }

    // modify slice timer
    pTimerInfo = &instance_l.timerInfo;
    OPENMAC_TIMERIRQDISABLE(HWTIMER_SYNC);

    // increment timer handle (if timer expires right after this statement,
    // the user would detect an unknown timer handle and discard it)
    // => unused in this implementation, as the timer can always be stopped
    pTimerInfo->eventArg.timerHdl.handle = ((pTimerInfo->eventArg.timerHdl.handle + 1) & TIMERHDL_MASK) |
                                    ((index + 1) << TIMERHDL_SHIFT);

    *pTimerHdl_p = pTimerInfo->eventArg.timerHdl.handle;

    pTimerInfo->eventArg.argument.value = argument_p;
    pTimerInfo->pfnCb = pfnCallback_p;

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Interrupt handler

This function is invoked by the openMAC HW sync timer interrupt.

\param[in,out]  pArg_p              Interrupt service routine argument
*/
//------------------------------------------------------------------------------
static void drvInterruptHandler(void* pArg_p)
{
    UNUSED_PARAMETER(pArg_p);

    BENCHMARK_MOD_24_SET(4);

    target_setInterruptContextFlag(TRUE);

    OPENMAC_TIMERIRQACK(HWTIMER_SYNC);
    OPENMAC_TIMERIRQDISABLE(HWTIMER_SYNC);

    if (instance_l.timerInfo.pfnCb != NULL)
    {
        instance_l.timerInfo.pfnCb(&instance_l.timerInfo.eventArg);
    }

    target_setInterruptContextFlag(FALSE);

    BENCHMARK_MOD_24_RESET(4);
}

/// \}
