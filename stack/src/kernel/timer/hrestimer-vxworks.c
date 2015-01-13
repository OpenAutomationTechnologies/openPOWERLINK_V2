/**
********************************************************************************
\file   hrestimer-vxworks.c

\brief  High-resolution timer module for VxWorks

This module is the target specific implementation of the high-resolution
timer module for VxWorks. It uses a special timer library contained in the
contrib directory for its implementation.

\ingroup module_hrestimer
*******************************************************************************/

/*------------------------------------------------------------------------------
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
#include <kernel/hrestimer.h>

#include <time.h>
#include <unistd.h>
#include <taskLib.h>
#include <timers.h>

#include <hrtimerLib.h>

//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

#define TIMER_COUNT           2            ///< number of high-resolution timers
#define TIMER_MIN_VAL_SINGLE  20000        ///< minimum timer intervall for single timeouts
#define TIMER_MIN_VAL_CYCLE   100000       ///< minimum timer intervall for continuous timeouts

/* macros for timer handles */
#define TIMERHDL_MASK         0x0FFFFFFF
#define TIMERHDL_SHIFT        28
#define HDL_TO_IDX(Hdl)       ((Hdl >> TIMERHDL_SHIFT) - 1)
#define HDL_INIT(Idx)         ((Idx + 1) << TIMERHDL_SHIFT)
#define HDL_INC(Hdl)          (((Hdl + 1) & TIMERHDL_MASK) | (Hdl & ~TIMERHDL_MASK))

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

/**
\brief  High-resolution timer information structure

The structure contains all necessary information for a high-resolution timer.
*/
typedef struct
{
    tTimerEventArg          eventArg;       ///< Event argument
    tTimerkCallback         pfnCallback;    ///< Pointer to timer callback function
    struct timespec         timeout;        ///< Timestamp of timeout value
    timer_t                 timer;          ///< timer_t struct of this timer
} tHresTimerInfo;

/**
\brief  High-resolution timer instance

The structure defines a high-resolution timer module instance.
*/
typedef struct
{
    tHresTimerInfo          aTimerInfo[TIMER_COUNT];    ///< Array with timer information for a set of timers
    int                     taskId;                     ///< Timer task Id
} tHresTimerInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tHresTimerInstance    hresTimerInstance_l;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief    Initialize high-resolution timer module

The function initializes the high-resolution timer module

\return Returns a tOplkError error code.

\ingroup module_hrestimer
*/
//------------------------------------------------------------------------------
tOplkError hrestimer_init(void)
{
    return hrestimer_addInstance();
}

//------------------------------------------------------------------------------
/**
\brief    Add instance of high-resolution timer module

The function adds an instance of the high-resolution timer module.

\return Returns a tOplkError error code.

\ingroup module_hrestimer
*/
//------------------------------------------------------------------------------
tOplkError hrestimer_addInstance(void)
{
    tOplkError                  ret = kErrorOk;
    UINT                        index;
    tHresTimerInfo*             pTimerInfo;
    tHrtimerSig                 sig;

    OPLK_MEMSET(&hresTimerInstance_l, 0, sizeof(hresTimerInstance_l));

    /* Initialize timer tasks for all usable timers. */
    for (index = 0; index < TIMER_COUNT; index++)
    {
        pTimerInfo = &hresTimerInstance_l.aTimerInfo[index];

        sig.sigType = kHrtimerSigCallback;
        sig.sigParam.m_signalCallback.pfnCallback = NULL;
        sig.sigParam.m_signalCallback.m_arg = 0;

        if (hrtimer_create(CLOCK_MONOTONIC, &sig, &pTimerInfo->timer) != 0)
            return kErrorNoResource;
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief    Delete instance of high-resolution timer module

The function deletes an instance of the high-resolution timer module.

\return Returns a tOplkError error code.

\ingroup module_hrestimer
*/
//------------------------------------------------------------------------------
tOplkError hrestimer_delInstance(void)
{
    tHresTimerInfo*             pTimerInfo;
    UINT                        index;

    for (index = 0; index < TIMER_COUNT; index++)
    {
        pTimerInfo = &hresTimerInstance_l.aTimerInfo[index];
        hrtimer_delete(pTimerInfo->timer);
        pTimerInfo->eventArg.timerHdl = 0;
        pTimerInfo->pfnCallback = NULL;
    }
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

\param  pTimerHdl_p     Pointer to timer handle.
\param  time_p          Relative timeout in [ns].
\param  pfnCallback_p   Callback function, which is called when timer expires.
                        (The function is called mutually exclusive with the Edrv
                        callback functions (Rx and Tx)).
\param  argument_p      User-specific argument
\param  fContinue_p     If TRUE, the callback function will be called continuously.
                        Otherwise, it is a one-shot timer.

\return Returns a tOplkError error code.

\ingroup module_hrestimer
*/
//------------------------------------------------------------------------------
tOplkError hrestimer_modifyTimer(tTimerHdl* pTimerHdl_p, ULONGLONG time_p,
                                 tTimerkCallback pfnCallback_p, ULONG argument_p,
                                 BOOL fContinue_p)
{
    tOplkError                  ret = kErrorOk;
    UINT                        index;
    tHresTimerInfo*             pTimerInfo;
    struct itimerspec           relTime;

    if (pTimerHdl_p == NULL)
    {
        DEBUG_LVL_ERROR_TRACE("%s() Invalid timer handle\n", __func__);
        return kErrorTimerInvalidHandle;
    }

    if (*pTimerHdl_p == 0)
    {   // no timer created yet, search free timer info structure
        pTimerInfo = &hresTimerInstance_l.aTimerInfo[0];
        for (index = 0; index < TIMER_COUNT; index++, pTimerInfo++)
        {
            if (pTimerInfo->eventArg.timerHdl == 0)
                break;      // free structure found
        }
        if (index >= TIMER_COUNT)
        {   // no free structure found
            DEBUG_LVL_ERROR_TRACE("%s() Invalid timer index:%d\n", __func__, index);
            return kErrorTimerNoTimerCreated;
        }
        pTimerInfo->eventArg.timerHdl = HDL_INIT(uiIndex);
    }
    else
    {
        index = HDL_TO_IDX(*pTimerHdl_p);
        if (index >= TIMER_COUNT)
        {   // invalid handle
            DEBUG_LVL_ERROR_TRACE("%s() Invalid timer index:%d\n", __func__, index);
            return kErrorTimerInvalidHandle;
        }
        pTimerInfo = &hresTimerInstance_l.aTimerInfo[index];
    }

    // increase too small time values
    if (fContinue_p != FALSE)
    {
        if (time_p < TIMER_MIN_VAL_CYCLE)
            time_p = TIMER_MIN_VAL_CYCLE;
    }
    else
    {
        if (time_p < TIMER_MIN_VAL_SINGLE)
            time_p = TIMER_MIN_VAL_SINGLE;
    }

    /* Increment timer handle (if timer expires right after this statement, the user
     * would detect an unknown timer handle and discard it) */
    pTimerInfo->eventArg.timerHdl = HDL_INC(pTimerInfo->eventArg.timerHdl);
    *pTimerHdl_p = pTimerInfo->eventArg.timerHdl;

    /* initialize timer info */
    pTimerInfo->eventArg.argument.value = argument_p;
    pTimerInfo->pfnCallback = pfnCallback_p;
    hrtimer_setCallback(pTimerInfo->timer, (void*)pTimerInfo->pfnCallback,
                        (void*)&pTimerInfo->eventArg);

    /*logMsg("set TCB: %p(%p)\n", (int)pTimerInfo->pfnCallback, (int)pTimerInfo->eventArg.argument.value, 0, 0, 0, 0);*/

    if (time_p >= 1000000000L)
    {
        relTime.it_value.tv_sec = (time_p / 1000000000L);
        relTime.it_value.tv_nsec = (time_p % 1000000000);
    }
    else
    {
        relTime.it_value.tv_sec = 0;
        relTime.it_value.tv_nsec = time_p;
    }

    if (fContinue_p)
    {
        relTime.it_interval.tv_nsec = relTime.it_value.tv_nsec;
        relTime.it_interval.tv_sec = relTime.it_value.tv_sec;
    }
    else
    {
        relTime.it_interval.tv_nsec = 0;
        relTime.it_interval.tv_sec = 0;
    }
#if 0
    DEBUG_LVL_TIMERH_TRACE("hrestimer_modifyTimer() timer=%lx ",
            pTimerInfo->eventArg.timerHdl);
    DEBUG_LVL_TIMERH_TRACE("        timeout=%ld:%ld/%ld:%ld\n",
           relTime.it_value.tv_sec, relTime.it_value.tv_nsec,
           relTime.it_interval.tv_sec, relTime.it_interval.tv_nsec);
#endif

    hrtimer_settime(pTimerInfo->timer, 0, &relTime, NULL);
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief    Delete a high-resolution timer

The function deletes an created high-resolution timer. The timer is specified
by its timer handle. After deleting, the handle is reset to zero.

\param  pTimerHdl_p     Pointer to timer handle.

\return Returns a tOplkError error code.

\ingroup module_hrestimer
*/
//------------------------------------------------------------------------------
tOplkError hrestimer_deleteTimer(tTimerHdl* pTimerHdl_p)
{
    tOplkError                  ret = kErrorOk;
    UINT                        index;
    tHresTimerInfo*             pTimerInfo;
    struct itimerspec           relTime;

    DEBUG_LVL_TIMERH_TRACE("%s() Deleting timer:%lx\n", __func__, *pTimerHdl_p);

    if (pTimerHdl_p == NULL)
        return kErrorTimerInvalidHandle;

    if (*pTimerHdl_p == 0)
    {   // no timer created yet
        return ret;
    }
    else
    {
        index = HDL_TO_IDX(*pTimerHdl_p);
        if (index >= TIMER_COUNT)
        {   // invalid handle
            return kErrorTimerInvalidHandle;
        }
        pTimerInfo = &hresTimerInstance_l.aTimerInfo[index];
        if (pTimerInfo->eventArg.timerHdl != *pTimerHdl_p)
            return ret;
    }

    // values of 0 disarms the timer
    relTime.it_value.tv_sec = 0;
    relTime.it_value.tv_nsec = 0;
    hrtimer_settime(pTimerInfo->timer, 0, &relTime, NULL);

    *pTimerHdl_p = 0;
    pTimerInfo->eventArg.timerHdl = 0;
    pTimerInfo->pfnCallback = NULL;
    hrtimer_setCallback(pTimerInfo->timer, (void*)pTimerInfo->pfnCallback, 0);
    return ret;
}
