/*******************************************************************************

  File:         TimerHighReskVxWorks.c

  (c) Bernecker + Rainer Ges.m.b.H.,  B&R Strasse 1, 5142 Eggelsberg, Austria
      www.br-automation.com

  (c) SYSTEC electronic GmbH, D-07973 Greiz, August-Bebel-Str. 29
      www.systec-electronic.com

  Project:      openPOWERLINK

  Description:  VxWorks implementation of high resolution timers
                using the high-resolution timer library

  TimerHighReskVxWorks.c contains the high-resolution timer implementation
  for VxWorks.


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

//=========================================================================//
// Includes                                                                //
//=========================================================================//
#include "EplInc.h"
#include "kernel/EplTimerHighResk.h"
#include "Benchmark.h"

#include <time.h>
#include <unistd.h>

#include <timers.h>
#include <taskLib.h>
#include "hrtimerLib.h"


//=========================================================================//
// Definitions                                                             //
//=========================================================================//
#define TIMER_COUNT           2            ///< number of high-resolution timers
#define TIMER_MIN_VAL_SINGLE  20000        ///< minimum timer intervall for single timeouts
#define TIMER_MIN_VAL_CYCLE   100000       ///< minimum timer intervall for continuous timeouts

/* macros for timer handles */
#define TIMERHDL_MASK         0x0FFFFFFF
#define TIMERHDL_SHIFT        28
#define HDL_TO_IDX(Hdl)       ((Hdl >> TIMERHDL_SHIFT) - 1)
#define HDL_INIT(Idx)         ((Idx + 1) << TIMERHDL_SHIFT)
#define HDL_INC(Hdl)          (((Hdl + 1) & TIMERHDL_MASK) | (Hdl & ~TIMERHDL_MASK))

//=========================================================================//
// Type definitions                                                        //
//=========================================================================//

/*******************************************************************************
* \brief       high-resolution timer information structure
*
* tEplTimerHighReskTimerInfo defines the structure which contains all
* necessary information for a high-resolution timer.
*******************************************************************************/
typedef struct
{
    tEplTimerEventArg   m_EventArg;
    tEplTimerkCallback  m_pfnCallback;  ///< pointer to timer callback function
    struct timespec     m_timeout;
    timer_t             m_timer;
} tEplTimerHighReskTimerInfo;


/*******************************************************************************
* \brief       high-resolution timer instance
*
* tEplTimerHighReskInstance contains all data of a high-resolution timer
* instance.
*******************************************************************************/
typedef struct
{
    tEplTimerHighReskTimerInfo  m_aTimerInfo[TIMER_COUNT];
    int                         m_task;
} tEplTimerHighReskInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tEplTimerHighReskInstance    EplTimerHighReskInstance_l;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------


//=========================================================================//
//                                                                         //
//          P U B L I C   F U N C T I O N S                                //
//                                                                         //
//=========================================================================//

//---------------------------------------------------------------------------
// Function:    EplTimerHighReskInit()
//
// Description: initializes the high resolution timer module.
//
// Parameters:  void
//
// Return:      tEplKernel      = error code
//---------------------------------------------------------------------------
tEplKernel PUBLIC EplTimerHighReskInit(void)
{
    tEplKernel  Ret;

    Ret = EplTimerHighReskAddInstance();

    return Ret;
}

//---------------------------------------------------------------------------
// Function:    EplTimerHighReskAddInstance()
//
// Description: initializes the high resolution timer module.
//
// Parameters:  void
//
// Return:      tEplKernel      = error code
//---------------------------------------------------------------------------
tEplKernel PUBLIC EplTimerHighReskAddInstance(void)
{
    tEplKernel                   Ret;
    UINT                         uiIndex;
    tEplTimerHighReskTimerInfo*  pTimerInfo;
    tHrtimerSig                  sig;

    Ret = kEplSuccessful;

    EPL_MEMSET(&EplTimerHighReskInstance_l, 0, sizeof (EplTimerHighReskInstance_l));

    /* Initialize timer threads for all usable timers. */
    for (uiIndex = 0; uiIndex < TIMER_COUNT; uiIndex++)
    {
        pTimerInfo = &EplTimerHighReskInstance_l.m_aTimerInfo[uiIndex];


        sig.sigType = kHrtimerSigCallback;
        sig.sigParam.m_signalCallback.m_pfnCallback = NULL;
        sig.sigParam.m_signalCallback.m_arg = 0;

        if (hrtimer_create(CLOCK_MONOTONIC, &sig, &pTimerInfo->m_timer) != 0)
        {
            Ret = kEplNoResource;
            goto Exit;
        }
    }

Exit:
    return Ret;
}

//---------------------------------------------------------------------------
// Function:    EplTimerHighReskDelInstance()
//
// Description: shuts down the high resolution timer module.
//
// Parameters:  void
//
// Return:      tEplKernel      = error code
//---------------------------------------------------------------------------
tEplKernel PUBLIC EplTimerHighReskDelInstance(void)
{
    tEplTimerHighReskTimerInfo* pTimerInfo;
    tEplKernel                  Ret;
    UINT                        uiIndex;

    Ret = kEplSuccessful;

    for (uiIndex = 0; uiIndex < TIMER_COUNT; uiIndex++)
    {
        pTimerInfo = &EplTimerHighReskInstance_l.m_aTimerInfo[uiIndex];
        hrtimer_delete(pTimerInfo->m_timer);
        pTimerInfo->m_EventArg.m_TimerHdl = 0;
        pTimerInfo->m_pfnCallback = NULL;
    }

    return Ret;
}

//---------------------------------------------------------------------------
// Function:    EplTimerHighReskModifyTimerNs()
//
// Description: modifies the timeout of the timer with the specified handle.
//              If the handle the pointer points to is zero, the timer must
//              be created first.
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
//---------------------------------------------------------------------------
tEplKernel PUBLIC EplTimerHighReskModifyTimerNs(tEplTimerHdl*     pTimerHdl_p,
                                    ULONGLONG           ullTimeNs_p,
                                    tEplTimerkCallback  pfnCallback_p,
                                    ULONG               ulArgument_p,
                                    BOOL                fContinuously_p)
{
    tEplKernel                   Ret;
    UINT                         uiIndex;
    tEplTimerHighReskTimerInfo*  pTimerInfo;
    struct itimerspec            RelTime;


    Ret = kEplSuccessful;

    // check pointer to handle
    if(pTimerHdl_p == NULL)
    {
        EPL_DBGLVL_ERROR_TRACE1("%s() Invalid timer handle\n", __func__);
        Ret = kEplTimerInvalidHandle;
        goto Exit;
    }

    if (*pTimerHdl_p == 0)
    {   // no timer created yet
        // search free timer info structure
        pTimerInfo = &EplTimerHighReskInstance_l.m_aTimerInfo[0];
        for (uiIndex = 0; uiIndex < TIMER_COUNT; uiIndex++, pTimerInfo++)
        {
            if (pTimerInfo->m_EventArg.m_TimerHdl == 0)
            {   // free structure found
                break;
            }
        }
        if (uiIndex >= TIMER_COUNT)
        {   // no free structure found
            EPL_DBGLVL_ERROR_TRACE2("%s() Invalid timer index:%d\n", __func__,
                                    uiIndex);
            Ret = kEplTimerNoTimerCreated;
            goto Exit;
        }

        pTimerInfo->m_EventArg.m_TimerHdl = HDL_INIT(uiIndex);
    }
    else
    {
        uiIndex = HDL_TO_IDX(*pTimerHdl_p);
        if (uiIndex >= TIMER_COUNT)
        {   // invalid handle
            EPL_DBGLVL_ERROR_TRACE2("%s() Invalid timer index:%d\n", __func__,
                                    uiIndex);
            Ret = kEplTimerInvalidHandle;
            goto Exit;
        }

        pTimerInfo = &EplTimerHighReskInstance_l.m_aTimerInfo[uiIndex];
    }

    // increase too small time values
    if (fContinuously_p != FALSE)
    {
        if (ullTimeNs_p < TIMER_MIN_VAL_CYCLE)
        {
            ullTimeNs_p = TIMER_MIN_VAL_CYCLE;
        }
    }
    else
    {
        if (ullTimeNs_p < TIMER_MIN_VAL_SINGLE)
        {
            ullTimeNs_p = TIMER_MIN_VAL_SINGLE;
        }
    }

    /*
     * increment timer handle
     * (if timer expires right after this statement, the user
     * would detect an unknown timer handle and discard it)
     */
    pTimerInfo->m_EventArg.m_TimerHdl = HDL_INC(pTimerInfo->m_EventArg.m_TimerHdl);
    *pTimerHdl_p = pTimerInfo->m_EventArg.m_TimerHdl;

    /* initialize timer info */
    pTimerInfo->m_EventArg.m_Arg.m_dwVal = ulArgument_p;
    pTimerInfo->m_pfnCallback      = pfnCallback_p;
    hrtimer_setCallback(pTimerInfo->m_timer, (void *)pTimerInfo->m_pfnCallback,
                        (void*)&pTimerInfo->m_EventArg);

    /*logMsg("set TCB: %p(%p)\n", (int)pTimerInfo->m_pfnCallback,
                                (int)pTimerInfo->m_EventArg.m_Arg.m_dwVal, 0, 0, 0, 0);*/


    if (ullTimeNs_p >= 1000000000L)
    {
        RelTime.it_value.tv_sec = (ullTimeNs_p / 1000000000L);
        RelTime.it_value.tv_nsec = (ullTimeNs_p % 1000000000) ;
    }
    else
    {
        RelTime.it_value.tv_sec = 0;
        RelTime.it_value.tv_nsec = ullTimeNs_p;
    }

    if (fContinuously_p)
    {
        RelTime.it_interval.tv_nsec = RelTime.it_value.tv_nsec;
        RelTime.it_interval.tv_sec = RelTime.it_value.tv_sec;
    }
    else
    {
        RelTime.it_interval.tv_nsec = 0;
        RelTime.it_interval.tv_sec = 0;
    }
#if 0
    EPL_DBGLVL_TIMERH_TRACE1("EplTimerHighReskModifyTimerNs() timer=%lx ",
            pTimerInfo->m_EventArg.m_TimerHdl);
    EPL_DBGLVL_TIMERH_TRACE4("        timeout=%ld:%ld/%ld:%ld\n",
           RelTime.it_value.tv_sec, RelTime.it_value.tv_nsec,
           RelTime.it_interval.tv_sec, RelTime.it_interval.tv_nsec);
#endif

    hrtimer_settime(pTimerInfo->m_timer, 0, &RelTime, NULL);

Exit:
    return Ret;
}

//---------------------------------------------------------------------------
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
//---------------------------------------------------------------------------
tEplKernel PUBLIC EplTimerHighReskDeleteTimer(tEplTimerHdl* pTimerHdl_p)
{
    tEplKernel                  Ret = kEplSuccessful;
    UINT                        uiIndex;
    tEplTimerHighReskTimerInfo* pTimerInfo;
    struct itimerspec           RelTime;

    EPL_DBGLVL_TIMERH_TRACE2("%s() Deleting timer:%lx\n", __func__,
                             *pTimerHdl_p);

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
        uiIndex = HDL_TO_IDX(*pTimerHdl_p);
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

    // values of 0 disarms the timer
    RelTime.it_value.tv_sec = 0;
    RelTime.it_value.tv_nsec = 0;
    hrtimer_settime(pTimerInfo->m_timer, 0, &RelTime, NULL);

    *pTimerHdl_p = 0;
    pTimerInfo->m_EventArg.m_TimerHdl = 0;
    pTimerInfo->m_pfnCallback = NULL;
    hrtimer_setCallback(pTimerInfo->m_timer, (void*)pTimerInfo->m_pfnCallback, 0);

Exit:
    return Ret;
}


