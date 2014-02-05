/**
********************************************************************************
\file   edrvcyclic.c

\brief  Implementation of cyclic Ethernet driver

This file contains the general implementation of the cyclic Ethernet driver.
It implements time-triggered transmission of frames necessary for MN.

\ingroup module_edrv
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
#include <common/edrv.h>
#include <kernel/hrestimer.h>
#include <common/target.h>

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
#if EPL_TIMER_USE_HIGHRES == FALSE
#error "EdrvCyclic needs EPL_TIMER_USE_HIGHRES = TRUE"
#endif

#if EDRV_CYCLIC_USE_DIAGNOSTICS != FALSE
#ifndef EDRV_CYCLIC_SAMPLE_TH_CYCLE_TIME_DIFF_US
#define EDRV_CYCLIC_SAMPLE_TH_CYCLE_TIME_DIFF_US         50
#endif

#ifndef EDRV_CYCLIC_SAMPLE_TH_SPARE_TIME_US
#define EDRV_CYCLIC_SAMPLE_TH_SPARE_TIME_US             150
#endif
#endif

//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------
typedef struct
{
    tEdrvTxBuffer**         ppTxBufferList;
    UINT                    maxTxBufferCount;
    UINT                    curTxBufferList;
    UINT                    curTxBufferEntry;
    UINT32                  cycleTimeUs;
    tEplTimerHdl            timerHdlCycle;
    tEplTimerHdl            timerHdlSlot;
    tEdrvCyclicCbSync       pfnSyncCb;
    tEdrvCyclicCbError      pfnErrorCb;
#if EDRV_CYCLIC_USE_DIAGNOSTICS != FALSE
    UINT                    sampleCount;
    ULONGLONG               startCycleTimeStamp;
    ULONGLONG               lastSlotTimeStamp;
    tEdrvCyclicDiagnostics  diagnostics;
#endif
} tEdrvcyclicInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tEdrvcyclicInstance edrvcyclicInstance_l;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static tOplkError timerHdlCycleCb(tEplTimerEventArg* pEventArg_p);
static tOplkError timerHdlSlotCb(tEplTimerEventArg* pEventArg_p);
static tOplkError processTxBufferList(void);

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Cyclic Ethernet driver initialization

This function initializes the cyclic Ethernet driver.

\return The function returns a tOplkError error code.

\ingroup module_edrv
*/
//------------------------------------------------------------------------------
tOplkError edrvcyclic_init(void)
{
    // clear instance structure
    EPL_MEMSET(&edrvcyclicInstance_l, 0, sizeof (edrvcyclicInstance_l));

#if EDRV_CYCLIC_USE_DIAGNOSTICS != FALSE
    edrvcyclicInstance_l.diagnostics.cycleTimeMin        = 0xFFFFFFFF;
    edrvcyclicInstance_l.diagnostics.usedCycleTimeMin    = 0xFFFFFFFF;
    edrvcyclicInstance_l.diagnostics.spareCycleTimeMin   = 0xFFFFFFFF;
#endif

    return kEplSuccessful;
}

//------------------------------------------------------------------------------
/**
\brief  Cyclic Ethernet driver shutdown

This function shuts down the cyclic Ethernet driver.

\return The function returns a tOplkError error code.

\ingroup module_edrv
*/
//------------------------------------------------------------------------------
tOplkError edrvcyclic_shutdown(void)
{
    if (edrvcyclicInstance_l.ppTxBufferList != NULL)
    {
        EPL_FREE(edrvcyclicInstance_l.ppTxBufferList);
        edrvcyclicInstance_l.ppTxBufferList = NULL;
        edrvcyclicInstance_l.maxTxBufferCount = 0;
    }

    return kEplSuccessful;
}

//------------------------------------------------------------------------------
/**
\brief  Set maximum size of Tx buffer list

This function determines the maxmimum size of the cyclic Tx buffer list.

\param  maxListSize_p   Maximum Tx buffer list size

\return The function returns a tOplkError error code.

\ingroup module_edrv
*/
//------------------------------------------------------------------------------
tOplkError edrvcyclic_setMaxTxBufferListSize(UINT maxListSize_p)
{
    tOplkError ret = kEplSuccessful;

    if (edrvcyclicInstance_l.maxTxBufferCount != maxListSize_p)
    {
        edrvcyclicInstance_l.maxTxBufferCount = maxListSize_p;
        if (edrvcyclicInstance_l.ppTxBufferList != NULL)
        {
            EPL_FREE(edrvcyclicInstance_l.ppTxBufferList);
            edrvcyclicInstance_l.ppTxBufferList = NULL;
        }

        edrvcyclicInstance_l.ppTxBufferList = EPL_MALLOC(sizeof (*edrvcyclicInstance_l.ppTxBufferList) * maxListSize_p * 2);
        if (edrvcyclicInstance_l.ppTxBufferList == NULL)
        {
            ret = kEplEdrvNoFreeBufEntry;
        }

        edrvcyclicInstance_l.curTxBufferList = 0;

        EPL_MEMSET(edrvcyclicInstance_l.ppTxBufferList, 0, sizeof (*edrvcyclicInstance_l.ppTxBufferList) * maxListSize_p * 2);
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Set next Tx buffer list

This function forwards the next cycle Tx buffer list to the cyclic Edrv.

\param  ppTxBuffer_p        Pointer to next cycle Tx buffer list
\param  txBufferCount_p     Tx buffer list count

\return The function returns a tOplkError error code.

\ingroup module_edrv
*/
//------------------------------------------------------------------------------
tOplkError edrvcyclic_setNextTxBufferList(tEdrvTxBuffer** ppTxBuffer_p, UINT txBufferCount_p)
{
    tOplkError  ret = kEplSuccessful;
    UINT        nextTxBufferList;

    nextTxBufferList = edrvcyclicInstance_l.curTxBufferList ^ edrvcyclicInstance_l.maxTxBufferCount;

    // check if next list is free
    if (edrvcyclicInstance_l.ppTxBufferList[nextTxBufferList] != NULL)
    {
        ret = kEplEdrvNextTxListNotEmpty;
        goto Exit;
    }

    if ((txBufferCount_p == 0) || (txBufferCount_p > edrvcyclicInstance_l.maxTxBufferCount))
    {
        ret = kEplEdrvInvalidParam;
        goto Exit;
    }

    // check if last entry in list equals a NULL pointer
    if (ppTxBuffer_p[txBufferCount_p - 1] != NULL)
    {
        ret = kEplEdrvInvalidParam;
        goto Exit;
    }

    EPL_MEMCPY(&edrvcyclicInstance_l.ppTxBufferList[nextTxBufferList], ppTxBuffer_p,
                sizeof (*ppTxBuffer_p) * txBufferCount_p);

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Set cycle time

This function sets the cycle time controlled by the cyclic Edrv.

\param  cycleTimeUs_p   Cycle time [us]

\return The function returns a tOplkError error code.

\ingroup module_edrv
*/
//------------------------------------------------------------------------------
tOplkError edrvcyclic_setCycleTime(UINT32 cycleTimeUs_p)
{
    edrvcyclicInstance_l.cycleTimeUs = cycleTimeUs_p;

    return kEplSuccessful;
}

//------------------------------------------------------------------------------
/**
\brief  Start cycle

This function starts the cycles.

\return The function returns a tOplkError error code.

\ingroup module_edrv
*/
//------------------------------------------------------------------------------
tOplkError edrvcyclic_startCycle(void)
{
    tOplkError ret = kEplSuccessful;

    if (edrvcyclicInstance_l.cycleTimeUs == 0)
    {
        ret = kEplEdrvInvalidCycleLen;
        goto Exit;
    }

    // clear Tx buffer list
    edrvcyclicInstance_l.curTxBufferList = 0;
    edrvcyclicInstance_l.curTxBufferEntry = 0;
    EPL_MEMSET(edrvcyclicInstance_l.ppTxBufferList, 0,
        sizeof (*edrvcyclicInstance_l.ppTxBufferList) * edrvcyclicInstance_l.maxTxBufferCount * 2);

    ret = hrestimer_modifyTimer(&edrvcyclicInstance_l.timerHdlCycle,
        edrvcyclicInstance_l.cycleTimeUs * 1000ULL,
        timerHdlCycleCb,
        0L,
        TRUE);

#if EDRV_CYCLIC_USE_DIAGNOSTICS != FALSE
    edrvcyclicInstance_l.lastSlotTimeStamp = 0;
#endif

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Stop cycle

This function stops the cycles.

\return The function returns a tOplkError error code.

\ingroup module_edrv
*/
//------------------------------------------------------------------------------
tOplkError edrvcyclic_stopCycle(void)
{
    tOplkError ret = kEplSuccessful;

    ret = hrestimer_deleteTimer(&edrvcyclicInstance_l.timerHdlCycle);
    ret = hrestimer_deleteTimer(&edrvcyclicInstance_l.timerHdlSlot);

#if EDRV_CYCLIC_USE_DIAGNOSTICS != FALSE
    edrvcyclicInstance_l.startCycleTimeStamp = 0;
#endif

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Register synchronization callback

This function registers the synchronization callback.

\param  pfnCbSync_p     Function pointer called at the configured synchronisation point

\return The function returns a tOplkError error code.

\ingroup module_edrv
*/
//------------------------------------------------------------------------------
tOplkError edrvcyclic_regSyncHandler(tEdrvCyclicCbSync pfnCbSync_p)
{
    edrvcyclicInstance_l.pfnSyncCb = pfnCbSync_p;

    return kEplSuccessful;
}

//------------------------------------------------------------------------------
/**
\brief  Register error callback

This function registers the error callback.

\param  pfnCbError_p    Function pointer called in case of a cycle processing error

\return The function returns a tOplkError error code.

\ingroup module_edrv
*/
//------------------------------------------------------------------------------
tOplkError edrvcyclic_regErrorHandler(tEdrvCyclicCbError pfnCbError_p)
{
    edrvcyclicInstance_l.pfnErrorCb = pfnCbError_p;

    return kEplSuccessful;
}


#if EDRV_CYCLIC_USE_DIAGNOSTICS != FALSE
//------------------------------------------------------------------------------
/**
\brief  Obtain diagnostic information

This function returns diagnostic information provided by the cyclic Edrv.

\param  ppDiagnostics_p     Pointer to store the pointer to the diagnostic information

\return The function returns a tOplkError error code.

\ingroup module_edrv
*/
//------------------------------------------------------------------------------
tOplkError edrvcyclic_getDiagnostics(tEdrvCyclicDiagnostics** ppDiagnostics_p)
{
    *ppDiagnostics_p = &edrvcyclicInstance_l.diagnostics;

    return kEplSuccessful;
}
#endif

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief  Cycle timer callback

This function is called by the timer module. It starts the next cycle.

\param  pEventArg_p     Timer event argument

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError timerHdlCycleCb(tEplTimerEventArg* pEventArg_p)
{
    tOplkError      ret = kEplSuccessful;
#if EDRV_CYCLIC_USE_DIAGNOSTICS != FALSE
    UINT32          cycleTime;
    UINT32          usedCycleTime;
    UINT32          spareCycleTime;
    ULONGLONG       startNewCycleTimeStamp;
#endif

    if (pEventArg_p->m_TimerHdl != edrvcyclicInstance_l.timerHdlCycle)
    {   // zombie callback
        // just exit
        goto Exit;
    }

#if EDRV_CYCLIC_USE_DIAGNOSTICS != FALSE
    startNewCycleTimeStamp = target_getCurrentTimestamp();
#endif

    if (edrvcyclicInstance_l.ppTxBufferList[edrvcyclicInstance_l.curTxBufferEntry] != NULL)
    {
        ret = kEplEdrvTxListNotFinishedYet;
        goto Exit;
    }

    edrvcyclicInstance_l.ppTxBufferList[edrvcyclicInstance_l.curTxBufferList] = NULL;

    // enter new cycle -> switch Tx buffer list
    edrvcyclicInstance_l.curTxBufferList ^= edrvcyclicInstance_l.maxTxBufferCount;
    edrvcyclicInstance_l.curTxBufferEntry = edrvcyclicInstance_l.curTxBufferList;

    if (edrvcyclicInstance_l.ppTxBufferList[edrvcyclicInstance_l.curTxBufferEntry] == NULL)
    {
        ret = kEplEdrvCurTxListEmpty;
        goto Exit;
    }

    ret = processTxBufferList();
    if (ret != kEplSuccessful)
    {
        goto Exit;
    }

    if (edrvcyclicInstance_l.pfnSyncCb != NULL)
    {
        ret = edrvcyclicInstance_l.pfnSyncCb();
    }

#if EDRV_CYCLIC_USE_DIAGNOSTICS != FALSE
    if (edrvcyclicInstance_l.startCycleTimeStamp != 0)
    {
        // calculate time diffs of previous cycle
        cycleTime      = (UINT32) (startNewCycleTimeStamp - edrvcyclicInstance_l.startCycleTimeStamp);
        if (edrvcyclicInstance_l.diagnostics.cycleTimeMin > cycleTime)
        {
            edrvcyclicInstance_l.diagnostics.cycleTimeMin = cycleTime;
        }
        if (edrvcyclicInstance_l.diagnostics.cycleTimeMax < cycleTime)
        {
            edrvcyclicInstance_l.diagnostics.cycleTimeMax = cycleTime;
        }

        if (edrvcyclicInstance_l.lastSlotTimeStamp != 0)
        {
            usedCycleTime  = (UINT32) (edrvcyclicInstance_l.lastSlotTimeStamp - edrvcyclicInstance_l.startCycleTimeStamp);
            spareCycleTime = (UINT32) (startNewCycleTimeStamp - edrvcyclicInstance_l.lastSlotTimeStamp);

            if (edrvcyclicInstance_l.diagnostics.usedCycleTimeMin > usedCycleTime)
            {
                edrvcyclicInstance_l.diagnostics.usedCycleTimeMin = usedCycleTime;
            }
            if (edrvcyclicInstance_l.diagnostics.usedCycleTimeMax < usedCycleTime)
            {
                edrvcyclicInstance_l.diagnostics.usedCycleTimeMax = usedCycleTime;
            }
            if (edrvcyclicInstance_l.diagnostics.spareCycleTimeMin > spareCycleTime)
            {
                edrvcyclicInstance_l.diagnostics.spareCycleTimeMin = spareCycleTime;
            }
            if (edrvcyclicInstance_l.diagnostics.spareCycleTimeMax < spareCycleTime)
            {
                edrvcyclicInstance_l.diagnostics.spareCycleTimeMax = spareCycleTime;
            }
        }
        else
        {
            usedCycleTime = 0;
            spareCycleTime = cycleTime;
        }

        edrvcyclicInstance_l.diagnostics.cycleTimeMeanSum      += cycleTime;
        edrvcyclicInstance_l.diagnostics.usedCycleTimeMeanSum  += usedCycleTime;
        edrvcyclicInstance_l.diagnostics.spareCycleTimeMeanSum += spareCycleTime;
        edrvcyclicInstance_l.diagnostics.cycleCount++;

        // sample previous cycle if deviations exceed threshold
        if (    (edrvcyclicInstance_l.diagnostics.sampleNum == 0) || /* sample first cycle for start time */
                (abs(cycleTime - edrvcyclicInstance_l.cycleTimeUs * 1000) > EDRV_CYCLIC_SAMPLE_TH_CYCLE_TIME_DIFF_US * 1000) ||
                (spareCycleTime < EDRV_CYCLIC_SAMPLE_TH_SPARE_TIME_US * 1000))
        {
        UINT uiSampleNo = edrvcyclicInstance_l.sampleCount;

            edrvcyclicInstance_l.diagnostics.aSampleTimeStamp[uiSampleNo] = edrvcyclicInstance_l.startCycleTimeStamp;
            edrvcyclicInstance_l.diagnostics.aCycleTime[uiSampleNo]       = cycleTime;
            edrvcyclicInstance_l.diagnostics.aUsedCycleTime[uiSampleNo]   = usedCycleTime;
            edrvcyclicInstance_l.diagnostics.aSpareCycleTime[uiSampleNo]  = spareCycleTime;

            edrvcyclicInstance_l.diagnostics.sampleNum++;
            if (edrvcyclicInstance_l.diagnostics.sampleBufferedNum != EDRV_CYCLIC_SAMPLE_NUM)
            {
                edrvcyclicInstance_l.diagnostics.sampleBufferedNum++;
            }

            edrvcyclicInstance_l.sampleCount++;
            if (edrvcyclicInstance_l.sampleCount == EDRV_CYCLIC_SAMPLE_NUM)
            {
                edrvcyclicInstance_l.sampleCount = 1;
            }
        }
    }

    edrvcyclicInstance_l.startCycleTimeStamp = startNewCycleTimeStamp;
    edrvcyclicInstance_l.lastSlotTimeStamp = 0;
#endif

Exit:
    if (ret != kEplSuccessful)
    {
        if (edrvcyclicInstance_l.pfnErrorCb != NULL)
        {
            ret = edrvcyclicInstance_l.pfnErrorCb(ret, NULL);
        }
    }
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Slot timer callback

This function is called by the timer module. It triggers the transmission of the
next frame.

\param  pEventArg_p     Timer event argument

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError timerHdlSlotCb(tEplTimerEventArg* pEventArg_p)
{
    tOplkError      ret = kEplSuccessful;
    tEdrvTxBuffer*  pTxBuffer = NULL;

    if (pEventArg_p->m_TimerHdl != edrvcyclicInstance_l.timerHdlSlot)
    {   // zombie callback
        // just exit
        goto Exit;
    }

#if EDRV_CYCLIC_USE_DIAGNOSTICS != FALSE
    edrvcyclicInstance_l.lastSlotTimeStamp = target_getCurrentTimestamp();
#endif

    pTxBuffer = edrvcyclicInstance_l.ppTxBufferList[edrvcyclicInstance_l.curTxBufferEntry];
    ret = edrv_sendTxBuffer(pTxBuffer);
    if (ret != kEplSuccessful)
    {
        goto Exit;
    }

    edrvcyclicInstance_l.curTxBufferEntry++;

    ret = processTxBufferList();

Exit:
    if (ret != kEplSuccessful)
    {
        if (edrvcyclicInstance_l.pfnErrorCb != NULL)
        {
            ret = edrvcyclicInstance_l.pfnErrorCb(ret, pTxBuffer);
        }
    }
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Process cycle Tx buffer list

This function processes the cycle Tx buffer list provided by dllk.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError processTxBufferList(void)
{
    tOplkError      ret = kEplSuccessful;
    tEdrvTxBuffer*  pTxBuffer;

    while ((pTxBuffer = edrvcyclicInstance_l.ppTxBufferList[edrvcyclicInstance_l.curTxBufferEntry]) != NULL)
    {
        if (pTxBuffer->timeOffsetNs == 0)
        {
            ret = edrv_sendTxBuffer(pTxBuffer);
            if (ret != kEplSuccessful)
            {
                goto Exit;
            }
        }
        else
        {
            ret = hrestimer_modifyTimer(&edrvcyclicInstance_l.timerHdlSlot,
                pTxBuffer->timeOffsetNs,
                timerHdlSlotCb,
                0L,
                FALSE);

            break;
        }

        edrvcyclicInstance_l.curTxBufferEntry++;
    }

Exit:
    if (ret != kEplSuccessful)
    {
        if (edrvcyclicInstance_l.pfnErrorCb != NULL)
        {
            ret = edrvcyclicInstance_l.pfnErrorCb(ret, pTxBuffer);
        }
    }
    return ret;
}

///\}
