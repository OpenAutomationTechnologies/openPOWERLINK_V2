/**
********************************************************************************
\file   synctimer-openmac.c

\brief  Implementation of openMAC synchronization timer module

This file contains the implementation of the openMAC synchronization timer module.

\ingroup module_synctimer
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
#include <oplk/oplkinc.h>

#include <kernel/synctimer.h>
#include <target/openmac.h>
#include <omethlib.h>

#include <oplk/benchmark.h>

//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

#ifndef TIMER_SYNC_SECOND_LOSS_OF_SYNC
#define TIMER_SYNC_SECOND_LOSS_OF_SYNC      FALSE
#endif

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
#define TIMER_HDL_SYNC          0
#define TIMER_HDL_LOSSOFSYNC    1
#define TIMER_HDL_INVALID       0xFF
#if (TIMER_SYNC_SECOND_LOSS_OF_SYNC != FALSE)
#define TIMER_HDL_LOSSOFSYNC2   2
#define TIMER_COUNT             3
#else
#define TIMER_COUNT             2
#endif

#define TIMEDIFF_COUNT_SHIFT    3
#define TIMEDIFF_COUNT          (1 << TIMEDIFF_COUNT_SHIFT)

#define PROPORTIONAL_FRACTION_SHIFT 3
#define PROPORTIONAL_FRACTION       (1 << PROPORTIONAL_FRACTION_SHIFT)

#define TIMER_DRV_MIN_TIME_DIFF     500

//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------
typedef struct
{
    UINT32  absoluteTime;
    BOOL    fEnable;
} tTimerInfo;

typedef struct
{
    tSyncTimerCbSync            pfnSyncCb;
    UINT32                      lossOfSyncTolerance;
    tSyncTimerCbLossOfSync      pfnLossOfSyncCb;
    UINT32                      lossOfSyncTimeout;
#if (TIMER_SYNC_SECOND_LOSS_OF_SYNC != FALSE)
    UINT32                      lossOfSyncTolerance2;
    tSyncTimerCbLossOfSync      pfnLossOfSync2Cb;
    UINT32                      lossOfSyncTimeout2;
#endif
    // EplTimerSynckCtrl specific
    BOOL                        fRun;
    UINT32                      aActualTimeDiff[TIMEDIFF_COUNT];
    UINT                        actualTimeDiffNextIndex;
    UINT32                      meanTimeDiff;
    UINT32                      configuredTimeDiff;
    UINT32                      advanceShift;
    UINT32                      rejectThreshold;
    UINT32                      targetSyncTime;
    UINT32                      previousSyncTime;
    // EplTimerSynckDrv specific
    tTimerInfo                  aTimerInfo[TIMER_COUNT];
    UINT                        activeTimerHdl;
#ifdef TIMER_USE_EXT_SYNC_INT
    BOOL                        fExtSyncEnable;
    UINT32                      syncIntCycle;
#endif //TIMER_USE_EXT_SYNC_INT
} tTimerInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tTimerInstance   instance_l;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static tOplkError ctrlDoSyncAdjustment(UINT32 timeStamp_p);
static void  ctrlSetConfiguredTimeDiff(UINT32 configuredTimeDiff_p);
static void  ctrlUpdateLossOfSyncTolerance(void);
static UINT32 ctrlGetNextAbsoluteTime(UINT timerHdl_p, UINT32 currentTime_p);

#ifdef TIMER_USE_EXT_SYNC_INT
static void drvCalcExtSyncIrqValue(void);
#endif //TIMER_USE_EXT_SYNC_INT

static void drvInterruptHandler(void* pArg_p);

static UINT drvFindShortestTimer(void);
static void drvConfigureShortestTimer(void);
static void ctrlAddActualTimeDiff(UINT32 actualTimeDiff_p);
static void ctrlCalcMeanTimeDiff(void);
static void ctrlUpdateRejectThreshold(void);

static tOplkError drvModifyTimerAbs(UINT timerHdl_p, UINT32 absoluteTime_p);

static tOplkError drvModifyTimerRel(UINT timerHdl_p, INT timeAdjustment_p,
                                    UINT32* pAbsoluteTime_p, BOOL* pfAbsoluteTimeAlreadySet_p);

static tOplkError drvDeleteTimer(UINT timerHdl_p);

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Synchronization timer module initialization

This function initializes the Synchronization timer module.

\return The function returns a tOplkError error code.

\ingroup module_synctimer
*/
//------------------------------------------------------------------------------
tOplkError synctimer_addInstance(void)
{
    tOplkError ret = kErrorOk;

    OPLK_MEMSET(&instance_l, 0, sizeof (instance_l));

    openmac_timerIrqDisable(HWTIMER_SYNC);
    openmac_timerSetCompareValue(HWTIMER_SYNC, 0);
#ifdef TIMER_USE_EXT_SYNC_INT
    openmac_timerIrqDisable(HWTIMER_EXT_SYNC);
    openmac_timerSetCompareValue(HWTIMER_EXT_SYNC, 0);
#endif //TIMER_USE_EXT_SYNC_INT

    ret = openmac_isrReg(kOpenmacIrqSync, drvInterruptHandler, NULL);

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Synchronization timer delete module

This function deletes the Synchronization timer module.

\return The function returns a tOplkError error code.

\ingroup module_synctimer
*/
//------------------------------------------------------------------------------
tOplkError synctimer_delInstance(void)
{
    tOplkError ret = kErrorOk;

    openmac_timerIrqDisable(HWTIMER_SYNC);
    openmac_timerSetCompareValue(HWTIMER_SYNC, 0);
#ifdef TIMER_USE_EXT_SYNC_INT
    openmac_timerIrqDisable(HWTIMER_EXT_SYNC);
    openmac_timerSetCompareValue(HWTIMER_EXT_SYNC, 0);
#endif //TIMER_USE_EXT_SYNC_INT

    openmac_isrReg(kOpenmacIrqSync, NULL, NULL);

    OPLK_MEMSET(&instance_l, 0, sizeof (instance_l));

    return ret;

}

//------------------------------------------------------------------------------
/**
\brief  Synchronization timer register synchronization handler

This function registers the synchronization handler callback.

\param  pfnSyncCb_p     Synchronization callback

\return The function returns a tOplkError error code.

\ingroup module_synctimer
*/
//------------------------------------------------------------------------------
tOplkError synctimer_registerHandler(tSyncTimerCbSync pfnSyncCb_p)
{
    tOplkError ret = kErrorOk;

    instance_l.pfnSyncCb = pfnSyncCb_p;

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Synchronization timer register loss of synchronization handler

This function registers the loss of synchronization handler callback.

\param  pfnLossOfSyncCb_p   Loss of synchronization callback

\return The function returns a tOplkError error code.

\ingroup module_synctimer
*/
//------------------------------------------------------------------------------
tOplkError synctimer_registerLossOfSyncHandler(tSyncTimerCbLossOfSync pfnLossOfSyncCb_p)
{
    tOplkError ret = kErrorOk;

    instance_l.pfnLossOfSyncCb = pfnLossOfSyncCb_p;

    return ret;
}

#if (TIMER_SYNC_SECOND_LOSS_OF_SYNC != FALSE)
//------------------------------------------------------------------------------
/**
\brief  Synchronization timer register second synchronization handler

This function registers the second synchronization handler callback.

\param  pfnLossOfSync2Cb_p  Second synchronization callback

\return The function returns a tOplkError error code.

\ingroup module_synctimer
*/
//------------------------------------------------------------------------------
tOplkError synctimer_registerLossOfSyncHandler2(tSyncTimerCbLossOfSync pfnLossOfSync2Cb_p)
{
    tOplkError ret = kErrorOk;

    instance_l.pfnLossOfSync2Cb = pfnLossOfSync2Cb_p;

    return ret;
}
#endif

//------------------------------------------------------------------------------
/**
\brief  Synchronization timer shift setter

This function sets the negative time shift.

\param  advanceShift_p      Time shift in microseconds

\return The function returns a tOplkError error code.

\ingroup module_synctimer
*/
//------------------------------------------------------------------------------
tOplkError synctimer_setSyncShift(UINT32 advanceShift_p)
{
    tOplkError ret = kErrorOk;

    instance_l.advanceShift = OMETH_US_2_TICKS(advanceShift_p);

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Synchronization timer cycle time

This function sets the cycle time.

\param  cycleLen_p      Cycle time in mircroseconds

\return The function returns a tOplkError error code.

\ingroup module_synctimer
*/
//------------------------------------------------------------------------------
tOplkError synctimer_setCycleLen(UINT32 cycleLen_p)
{
    tOplkError ret = kErrorOk;

    ctrlSetConfiguredTimeDiff(OMETH_US_2_TICKS(cycleLen_p));

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Synchronization timer loss of synchronization setter

This function sets the loss of synchronization tolerance.

\param  lossOfSyncTolerance_p   Loss of sync tolerance in nanoseconds

\return The function returns a tOplkError error code.

\ingroup module_synctimer
*/
//------------------------------------------------------------------------------
tOplkError synctimer_setLossOfSyncTolerance(UINT32 lossOfSyncTolerance_p)
{
    tOplkError ret = kErrorOk;

    instance_l.lossOfSyncTolerance = lossOfSyncTolerance_p;

    ctrlUpdateLossOfSyncTolerance();

    return ret;
}

#if (TIMER_SYNC_SECOND_LOSS_OF_SYNC != FALSE)
//------------------------------------------------------------------------------
/**
\brief  Synchronization timer second loss of synchronization setter

This function sets the loss of synchronization tolerance.

\param  lossOfSyncTolerance2_p      Second loss of sync tolerance in nanoseconds

\return The function returns a tOplkError error code.

\ingroup module_synctimer
*/
//------------------------------------------------------------------------------
tOplkError synctimer_setLossOfSyncTolerance2(UINT32 lossOfSyncTolerance2_p)
{
    tOplkError ret = kErrorOk;

    instance_l.lossOfSyncTolerance2 = lossOfSyncTolerance2_p;

    if (lossOfSyncTolerance2_p > 0)
    {
        instance_l.lossOfSyncTimeout2 = instance_l.configuredTimeDiff
                + OMETH_NS_2_TICKS(instance_l.lossOfSyncTolerance2);
    }
    else
    {
        instance_l.lossOfSyncTimeout2 = 0;
    }

    return ret;
}

#endif
//------------------------------------------------------------------------------
/**
\brief  Synchronization timer trigger setter

This function sets the synchronization time trigger at a specific time stamp.

\param  pTimeStamp_p    Time stamp when the sync module should trigger

\return The function returns a tOplkError error code.

\ingroup module_synctimer
*/
//------------------------------------------------------------------------------
tOplkError synctimer_syncTriggerAtTimeStamp(tTimestamp* pTimeStamp_p)
{
    tOplkError ret = kErrorOk;

    ret = drvModifyTimerAbs(TIMER_HDL_LOSSOFSYNC,
                                      (pTimeStamp_p->timeStamp + instance_l.lossOfSyncTimeout));
    if (ret != kErrorOk)
    {
        goto Exit;
    }

#if (TIMER_SYNC_SECOND_LOSS_OF_SYNC != FALSE)
    if (instance_l.lossOfSyncTimeout2 > 0)
    {
        ret = drvModifyTimerAbs(TIMER_HDL_LOSSOFSYNC2,
                                          (pTimeStamp_p->timeStamp + instance_l.lossOfSyncTimeout2));
        if (ret != kErrorOk)
        {
            goto Exit;
        }
    }
#endif

    ret = ctrlDoSyncAdjustment(pTimeStamp_p->timeStamp);

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Stop Synchronization timer module

This function stops the module.

\return The function returns a tOplkError error code.

\ingroup module_synctimer
*/
//------------------------------------------------------------------------------
tOplkError synctimer_stopSync(void)
{
    tOplkError ret = kErrorOk;

    instance_l.fRun = FALSE;

    ret = drvDeleteTimer(TIMER_HDL_SYNC);
    ret = drvDeleteTimer(TIMER_HDL_LOSSOFSYNC);
#if (TIMER_SYNC_SECOND_LOSS_OF_SYNC != FALSE)
    ret = drvDeleteTimer(TIMER_HDL_LOSSOFSYNC2);
#endif

    return ret;
}


#ifdef TIMER_USE_EXT_SYNC_INT
//------------------------------------------------------------------------------
/**
\brief  Enable second sync interrupt

This function enables the external sync interrupt of 2nd CMP timer

\param  syncIntCycle_p      Trigger external sync int every nth cycle
\param  pulseWidth_p        Pulse width of external sync interrupt in nanoseconds.
                            If 0 external sync int is just toggled.

\return The function returns a tOplkError error code.

\ingroup module_synctimer
*/
//------------------------------------------------------------------------------
void synctimer_enableExtSyncIrq(UINT32 syncIntCycle_p, UINT32 pulseWidth_p)
{
    instance_l.fExtSyncEnable = TRUE;
    instance_l.syncIntCycle = syncIntCycle_p;

    openmac_timerIrqEnable(HWTIMER_EXT_SYNC, pulseWidth_p);
}

//------------------------------------------------------------------------------
/**
\brief  Disable second sync interrupt

This function disables the external sync interrupt of 2nd CMP timer

\return The function returns a tOplkError error code.

\ingroup module_synctimer
*/
//------------------------------------------------------------------------------
void synctimer_disableExtSyncIrq(void)
{
    instance_l.fExtSyncEnable = FALSE;
    instance_l.syncIntCycle = 0;

    openmac_timerIrqDisable(HWTIMER_EXT_SYNC);
}
#endif //TIMER_USE_EXT_SYNC_INT

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief  Adjust the synchronization

This function adjusts the synchronization mechanism with a filter.

\param  timeStamp_p     New sync time stamp

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError ctrlDoSyncAdjustment(UINT32 timeStamp_p)
{
    tOplkError  ret = kErrorOk;
    UINT32      actualTimeDiff;
    INT         deviation;
    BOOL        fCurrentSyncModified = FALSE;

    timeStamp_p -= instance_l.advanceShift;

    if (instance_l.fRun != FALSE)
    {
        actualTimeDiff = timeStamp_p - instance_l.previousSyncTime;

        ctrlAddActualTimeDiff(actualTimeDiff);

        deviation = timeStamp_p - instance_l.targetSyncTime;

        deviation = deviation >> PROPORTIONAL_FRACTION_SHIFT;

        ret = drvModifyTimerRel(TIMER_HDL_SYNC, deviation, &instance_l.targetSyncTime, &fCurrentSyncModified);

        if (fCurrentSyncModified != FALSE)
        {   // set target to next sync
            instance_l.targetSyncTime += instance_l.meanTimeDiff;
        }
    }
    else
    {   // first trigger
        instance_l.targetSyncTime = timeStamp_p + instance_l.meanTimeDiff;
        instance_l.fRun = TRUE;

        ret = drvModifyTimerAbs(TIMER_HDL_SYNC, instance_l.targetSyncTime);
    }

    instance_l.previousSyncTime = timeStamp_p;

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Add actual time difference

This function adds the actual time difference for the next synchronization.

\param  actualTimeDiff_p    Actual time difference
*/
//------------------------------------------------------------------------------
static void ctrlAddActualTimeDiff(UINT32 actualTimeDiff_p)
{
    // always add small TimeDiff values
    // reject TimeDiff values which are too large
    if (actualTimeDiff_p < instance_l.rejectThreshold)
    {
        instance_l.aActualTimeDiff[instance_l.actualTimeDiffNextIndex]
            = actualTimeDiff_p;
        instance_l.actualTimeDiffNextIndex++;
        instance_l.actualTimeDiffNextIndex &= (TIMEDIFF_COUNT - 1);

        ctrlCalcMeanTimeDiff();
    }
    else
    {   // adjust target sync time, because of Loss of Sync
        for (; actualTimeDiff_p >= instance_l.rejectThreshold;
             actualTimeDiff_p -= instance_l.meanTimeDiff,
             instance_l.targetSyncTime += instance_l.meanTimeDiff)
        {
        }
    }
}

//------------------------------------------------------------------------------
/**
\brief  Calculate mean time difference

This function calculates the average of the time differences (filter).
*/
//------------------------------------------------------------------------------
static void ctrlCalcMeanTimeDiff(void)
{
    INT     i;
    UINT32  timeDiffSum;

    timeDiffSum = 0;

    for (i=0; i < TIMEDIFF_COUNT; i++)
    {
        timeDiffSum += instance_l.aActualTimeDiff[i];
    }

    instance_l.meanTimeDiff = timeDiffSum >> TIMEDIFF_COUNT_SHIFT;
}

//------------------------------------------------------------------------------
/**
\brief  Set configured time difference

This function sets the configured time difference.

\param  configuredTimeDiff_p    Configured time difference
*/
//------------------------------------------------------------------------------
static void ctrlSetConfiguredTimeDiff(UINT32 configuredTimeDiff_p)
{
    INT i;

    instance_l.configuredTimeDiff = configuredTimeDiff_p;

    for (i=0; i < TIMEDIFF_COUNT; i++)
    {
        instance_l.aActualTimeDiff[i] = configuredTimeDiff_p;
    }

    instance_l.meanTimeDiff = configuredTimeDiff_p;

    ctrlUpdateRejectThreshold();
}

//------------------------------------------------------------------------------
/**
\brief  Update loss of sync tolerance

This function updates the loss of sync tolerance.
*/
//------------------------------------------------------------------------------
static void ctrlUpdateLossOfSyncTolerance(void)
{
    ctrlUpdateRejectThreshold();
}

//------------------------------------------------------------------------------
/**
\brief  Update reject threshold

This function updates the reject threshold
*/
//------------------------------------------------------------------------------
static void ctrlUpdateRejectThreshold(void)
{
    UINT32  lossOfSyncTolerance;
    UINT32  maxRejectThreshold;

    lossOfSyncTolerance = OMETH_NS_2_TICKS(instance_l.lossOfSyncTolerance);
    maxRejectThreshold  = instance_l.configuredTimeDiff >> 1;  // half of cycle length

    instance_l.rejectThreshold = instance_l.configuredTimeDiff;

    if (lossOfSyncTolerance > maxRejectThreshold)
    {
        instance_l.rejectThreshold += maxRejectThreshold;
    }
    else
    {
        instance_l.rejectThreshold += lossOfSyncTolerance;
    }

    instance_l.lossOfSyncTimeout = instance_l.configuredTimeDiff + lossOfSyncTolerance;
}

//------------------------------------------------------------------------------
/**
\brief  Get next absolute time for synchronization

This function returns the absolute time stamp for the next time synchronization.

\param  timerHdl_p          Timer handle
\param  currentTime_p       Current time

\return Next absolute time value.
*/
//------------------------------------------------------------------------------
static UINT32 ctrlGetNextAbsoluteTime(UINT timerHdl_p, UINT32 currentTime_p)
{
    UINT32 nextAbsoluteTime;

    switch (timerHdl_p)
    {
        case TIMER_HDL_SYNC:
            nextAbsoluteTime = currentTime_p + instance_l.meanTimeDiff;
            break;

        case TIMER_HDL_LOSSOFSYNC:
            nextAbsoluteTime = currentTime_p + instance_l.configuredTimeDiff;
            break;

        default:
            nextAbsoluteTime = 0;
            break;
    }

    return nextAbsoluteTime;
}

//------------------------------------------------------------------------------
/**
\brief  Modify absolute timer

This function modifies the timer's absolute timer value.

\param  timerHdl_p      Timer handle
\param  absoluteTime_p  Absolute time value

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError drvModifyTimerAbs(UINT timerHdl_p, UINT32 absoluteTime_p)
{
    tOplkError  ret = kErrorOk;
    tTimerInfo* pTimerInfo;

    if (timerHdl_p >= TIMER_COUNT)
    {
        ret = kErrorTimerInvalidHandle;
        goto Exit;
    }

    pTimerInfo = &instance_l.aTimerInfo[timerHdl_p];
    pTimerInfo->absoluteTime = absoluteTime_p;
    pTimerInfo->fEnable = TRUE;

    drvConfigureShortestTimer();

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Modify relative timer

This function modifies the timer's realtive timer value.

\param  timerHdl_p                  Timer handle
\param  timeAdjustment_p            Relative time adjustment
\param  pAbsoluteTime_p             Pointer to the timer's absolute time
\param  pfAbsoluteTimeAlreadySet_p  Some weird flag

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError drvModifyTimerRel(UINT timerHdl_p, INT timeAdjustment_p,
        UINT32* pAbsoluteTime_p, BOOL* pfAbsoluteTimeAlreadySet_p)
{
    tOplkError  ret = kErrorOk;
    tTimerInfo* pTimerInfo;

    if (timerHdl_p >= TIMER_COUNT)
    {
        ret = kErrorTimerInvalidHandle;
        goto Exit;
    }

    pTimerInfo = &instance_l.aTimerInfo[timerHdl_p];
    if (pTimerInfo->absoluteTime == *pAbsoluteTime_p)
    {
        *pfAbsoluteTimeAlreadySet_p = TRUE;
    }
    else
    {
        *pfAbsoluteTimeAlreadySet_p = FALSE;
    }

    pTimerInfo->absoluteTime += timeAdjustment_p;

    *pAbsoluteTime_p = pTimerInfo->absoluteTime;
    pTimerInfo->fEnable = TRUE;

    drvConfigureShortestTimer();

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Delete sync timer

This function deletes the timer handle.

\param  timerHdl_p  Timer handle

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError drvDeleteTimer(UINT timerHdl_p)
{
    tOplkError  ret = kErrorOk;
    tTimerInfo* pTimerInfo;

    if (timerHdl_p >= TIMER_COUNT)
    {
        ret = kErrorTimerInvalidHandle;
        goto Exit;
    }

    pTimerInfo = &instance_l.aTimerInfo[timerHdl_p];
    pTimerInfo->fEnable = FALSE;

    drvConfigureShortestTimer();

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Find shortest due timer

This function searches for the next timer that shall trigger the interrupt.

\return The function returns the next due timer handle.
*/
//------------------------------------------------------------------------------
static UINT drvFindShortestTimer(void)
{
    UINT        targetTimerHdl;
    UINT        currentTimerHdl;
    tTimerInfo* pTimerInfo;
    UINT32      absoluteTime = 0;

    targetTimerHdl = TIMER_HDL_INVALID;

    for (pTimerInfo = &instance_l.aTimerInfo[0],
         currentTimerHdl = 0;
         currentTimerHdl < TIMER_COUNT;
         pTimerInfo++, currentTimerHdl++)
    {
        if (pTimerInfo->fEnable != FALSE)
        {
            if ((targetTimerHdl == TIMER_HDL_INVALID) ||
                ((LONG)(pTimerInfo->absoluteTime - absoluteTime) < 0))
            {
                absoluteTime = pTimerInfo->absoluteTime;
                targetTimerHdl = (UINT)(pTimerInfo - &instance_l.aTimerInfo[0]);
            }
        }
    }

    return targetTimerHdl;
}

//------------------------------------------------------------------------------
/**
\brief  Configure shortest due timer

This function configures the next due timer.
*/
//------------------------------------------------------------------------------
static void drvConfigureShortestTimer(void)
{
    UINT        nextTimerHdl;
    tTimerInfo* pTimerInfo;
    UINT32      targetAbsoluteTime;
    UINT32      currentTime;

    openmac_timerIrqDisable(HWTIMER_SYNC);

    nextTimerHdl = drvFindShortestTimer();
    if (nextTimerHdl != TIMER_HDL_INVALID)
    {
        pTimerInfo = &instance_l.aTimerInfo[nextTimerHdl];

        instance_l.activeTimerHdl = nextTimerHdl;
        targetAbsoluteTime = pTimerInfo->absoluteTime;

        currentTime = openmac_timerGetTimeValue(HWTIMER_SYNC);
        if ((LONG)(targetAbsoluteTime - currentTime) < TIMER_DRV_MIN_TIME_DIFF)
        {
            targetAbsoluteTime = currentTime + TIMER_DRV_MIN_TIME_DIFF;
        }

        openmac_timerSetCompareValue(HWTIMER_SYNC, targetAbsoluteTime);

        // enable timer
        openmac_timerIrqEnable(HWTIMER_SYNC, 0);
    }
    else
    {
        openmac_timerSetCompareValue(HWTIMER_SYNC, 0);

        instance_l.activeTimerHdl = TIMER_HDL_INVALID;
    }
}

#ifdef TIMER_USE_EXT_SYNC_INT
//------------------------------------------------------------------------------
/**
\brief  Calculate external sync irq value

This function calculates the external sync timer value triggering the interrupt.
*/
//------------------------------------------------------------------------------
static void drvCalcExtSyncIrqValue(void)
{
    tTimerInfo*     pTimerInfo;
    UINT32          targetAbsoluteTime;
    static UINT32   cycleCnt = 0;

    if ((++cycleCnt == instance_l.syncIntCycle))
    {
        // get absolute time from sync timer
        pTimerInfo = &instance_l.aTimerInfo[TIMER_HDL_SYNC];
        targetAbsoluteTime = pTimerInfo->absoluteTime;

        openmac_timerSetCompareValue(HWTIMER_EXT_SYNC,
                targetAbsoluteTime -
                instance_l.meanTimeDiff +    // minus one cycle
                instance_l.advanceShift);      // plus sync shift
        cycleCnt = 0;
    }
    else if (cycleCnt > instance_l.syncIntCycle)
    {
         cycleCnt = 0;
    }
}
#endif //TIMER_USE_EXT_SYNC_INT

//------------------------------------------------------------------------------
/**
\brief  Interrupt handler

This function is invoked by the openMAC HW sync timer interrupt.

\param  pArg_p  Interrupt service routine argument
*/
//------------------------------------------------------------------------------
static void drvInterruptHandler(void* pArg_p)
{
    UINT        timerHdl;
    UINT        nextTimerHdl;
    tTimerInfo* pTimerInfo;

    BENCHMARK_MOD_24_SET(4);

    UNUSED_PARAMETER(pArg_p);

    timerHdl = instance_l.activeTimerHdl;
    if (timerHdl < TIMER_COUNT)
    {
        pTimerInfo = &instance_l.aTimerInfo[timerHdl];
        pTimerInfo->absoluteTime = ctrlGetNextAbsoluteTime(timerHdl, pTimerInfo->absoluteTime);

        // execute the sync if it will elapse in a very short moment
        // to give the sync event the highest priority.
        nextTimerHdl = drvFindShortestTimer();
        if ((nextTimerHdl != timerHdl) &&
            (nextTimerHdl == TIMER_HDL_SYNC))
        {
            pTimerInfo = &instance_l.aTimerInfo[timerHdl];

            if ((pTimerInfo->fEnable != FALSE) &&
                ((LONG)(pTimerInfo->absoluteTime - openmac_timerGetTimeValue(HWTIMER_SYNC)) < TIMER_DRV_MIN_TIME_DIFF))
            {
                pTimerInfo->absoluteTime = ctrlGetNextAbsoluteTime(nextTimerHdl, pTimerInfo->absoluteTime);

                if (instance_l.pfnSyncCb != NULL)
                {
                    instance_l.pfnSyncCb();
                }
            }
        }

        switch (timerHdl)
        {
            case TIMER_HDL_SYNC:
            {
                #ifdef TIMER_USE_EXT_SYNC_INT
                    BENCHMARK_MOD_24_SET(0);
                    if(instance_l.fExtSyncEnable != FALSE)
                    {
                        drvCalcExtSyncIrqValue();
                    }
                    BENCHMARK_MOD_24_RESET(0);
                #endif //TIMER_USE_EXT_SYNC_INT

                if (instance_l.pfnSyncCb != NULL)
                {
                    instance_l.pfnSyncCb();
                }
                break;
            }
            case TIMER_HDL_LOSSOFSYNC:
            {
                if (instance_l.pfnLossOfSyncCb != NULL)
                {
                    instance_l.pfnLossOfSyncCb();
                }
                break;
            }
#if (TIMER_SYNC_SECOND_LOSS_OF_SYNC != FALSE)
            case TIMER_HDL_LOSSOFSYNC2:
            {
                if (instance_l.pfnLossOfSync2Cb != NULL)
                {
                    instance_l.pfnLossOfSync2Cb();
                }
                break;
            }
#endif
            default:
            {
                break;
            }
        }
    }

    drvConfigureShortestTimer();

    BENCHMARK_MOD_24_RESET(4);
}

///\}
