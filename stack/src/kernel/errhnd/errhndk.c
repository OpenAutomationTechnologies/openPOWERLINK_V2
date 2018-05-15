/**
********************************************************************************
\file   errhndk.c

\brief  Implementation of kernel error handler module

This module implements the kernel part of the error handler module.
It is responsible for handling errors and incrementing the appropriate
error counters.

\ingroup module_errhndk
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2012, SYSTEC electronic GmbH
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
#include <kernel/errhndk.h>
#include <kernel/eventk.h>

#if defined(CONFIG_INCLUDE_NMT_MN)
#include <kernel/dllk.h>
#endif

#include <common/ami.h>
#include <oplk/nmt.h>
#include <oplk/frame.h>
#include <oplk/benchmark.h>
#include "errhndkcal.h"

//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#define ERRORHANDLERK_CN_LOSS_PRES_EVENT_NONE   0   // error not occurred
#define ERRORHANDLERK_CN_LOSS_PRES_EVENT_OCC    1   // occurred
#define ERRORHANDLERK_CN_LOSS_PRES_EVENT_THR    2   // threshold exceeded

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
//------------------------------------------------------------------------------

/**
\brief  Instance of kernel error handler

The structure defines the instance variables of the kernel error handler.
*/
typedef struct
{
    UINT32              dllErrorEvents;                                 ///< Variable stores detected error events
    UINT8               aMnCnLossPresEvent[NUM_DLL_MNCN_LOSSPRES_OBJS]; ///< Variable stores detected error events from CNs
    tErrHndObjects      errorObjects;                                   ///< Error objects (counters and thresholds)
} tErrHndkInstance;

//------------------------------------------------------------------------------
// module local vars
//------------------------------------------------------------------------------
static tErrHndkInstance instance_l;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static tOplkError postNmtEvent(tNmtEvent nmtEvent_p);
static tOplkError generateHistoryEntry(UINT16 errorCode_p, tNetTime netTime_p);
static tOplkError generateHistoryEntryNodeId(UINT16 errorCode_p, tNetTime netTime_p, UINT nodeId_p);
static void       decrementCnCounters(void);
static tOplkError postHistoryEntryEvent(const tErrHistoryEntry* pHistoryEntry_p);
static tOplkError handleDllErrors(const tEvent* pEvent_p);

#if defined(CONFIG_INCLUDE_NMT_MN)
static tOplkError decrementMnCounters(void);
static tOplkError postHeartbeatEvent(UINT nodeId_p, tNmtState state_p, UINT16 errorCode_p);
static tOplkError generateHistoryEntryWithError(UINT16 errorCode_p, tNetTime netTime_p, UINT16 oplkError_p);
#endif

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief    Initialize kernel error handler module

The function initializes the kernel error handler module.

\return Returns a tOplkError error code.

\ingroup module_errhndk
*/
//------------------------------------------------------------------------------
tOplkError errhndk_init(void)
{
    tOplkError  ret;

    instance_l.dllErrorEvents = 0L;
    ret = errhndkcal_init();

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief    Shutdown kernel error handler module

The function shuts down the kernel error handler module.

\return Returns always kErrorOk

\ingroup module_errhndk
*/
//------------------------------------------------------------------------------
tOplkError errhndk_exit()
{
    errhndkcal_exit();

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Get pointer to error handler objects

The function returns a pointer to the memory block where the error handler
objects are stored.

\return The function returns a pointer to the error handler objects.

\ingroup module_errhndkcal
*/
//------------------------------------------------------------------------------
tErrHndObjects* errhndk_getMemPtr(void)
{
    return errhndkcal_getMemPtr();
}

//------------------------------------------------------------------------------
/**
\brief    Process error events

The function processes error events and modifies the appropriate error counters.
It will be called by the DLL.

\param[in]      pEvent_p            Pointer to error event which should be processed.

\return Returns a tOplkError error code
\retval kErrorOk                    Event was successfully handled.
\retval kErrorInvalidEvent          An invalid event was supplied.

\ingroup module_errhndk
*/
//------------------------------------------------------------------------------
tOplkError errhndk_process(const tEvent* pEvent_p)
{
    tOplkError  ret;

    // Check parameter validity
    ASSERT(pEvent_p != NULL);

    switch (pEvent_p->eventType)
    {
        case kEventTypeDllError:
            ret = handleDllErrors(pEvent_p);
            break;

        // unknown type
        default:
            ret = kErrorInvalidEvent;
            break;
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief    Decrement error counters

The function decrements the error counters. It should be called at the end
of each cycle.

\param[in]      fMN_p               Flag determines if node is running as MN.

\return Returns always kErrorOk

\ingroup module_errhndk
*/
//------------------------------------------------------------------------------
tOplkError errhndk_decrementCounters(BOOL fMN_p)
{
#if defined(CONFIG_INCLUDE_NMT_MN)
    if (fMN_p != FALSE)
    {   // local node is MN -> decrement MN threshold counters
        decrementMnCounters();
    }
    else
    {
        decrementCnCounters();
    }
#else
    UNUSED_PARAMETER(fMN_p);

    decrementCnCounters();
#endif

    // reset error events
    instance_l.dllErrorEvents = 0L;

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief    Post error event

The function posts an error event to the error handler module. It is provided
to other modules which need to post error events to the error handler.

\param[in]      pErrEvent_p         Pointer to error event which should be posted.

\return Returns error code provided by eventk_postEvent()

\ingroup module_errhndk
*/
//------------------------------------------------------------------------------
tOplkError errhndk_postError(const tEventDllError* pErrEvent_p)
{
    tOplkError  ret;
    tEvent      event;

    event.eventSink = kEventSinkErrk;
    event.eventType = kEventTypeDllError;
    event.eventArgSize = sizeof(tEventDllError);
    event.eventArg.pEventArg = (void*)pErrEvent_p;

    ret = eventk_postEvent(&event);

    return ret;
}


#if defined(CONFIG_INCLUDE_NMT_MN)
//------------------------------------------------------------------------------
/**
\brief    Reset error flag for specified CN

The function resets the error flag for the specified CN.

\param[in]      nodeId_p            Node ID of CN for which error flag will be reset.

\return Returns always kErrorOk

\ingroup module_errhndk
*/
//------------------------------------------------------------------------------
tOplkError errhndk_resetCnError(UINT nodeId_p)
{
    UINT    nodeIdx = nodeId_p - 1;

    if (nodeIdx >= NUM_DLL_MNCN_LOSSPRES_OBJS)
        return kErrorInvalidNodeId;

    instance_l.aMnCnLossPresEvent[nodeIdx] = ERRORHANDLERK_CN_LOSS_PRES_EVENT_NONE;

    return kErrorOk;
}
#endif

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

#if defined(CONFIG_INCLUDE_NMT_MN)
//------------------------------------------------------------------------------
/**
\brief    Decrement MN error counters

The function decrements the error counters used by a MN node.

\return Returns kErrorOk or error code
*/
//------------------------------------------------------------------------------
static tOplkError decrementMnCounters(void)
{
    UINT8*  pCnNodeId;
    UINT    nodeIdx;
    UINT32  thresholdCnt;

    dllk_getCurrentCnNodeIdList(&pCnNodeId);

    // iterate through node info structure list
    while (*pCnNodeId != C_ADR_INVALID)
    {
        nodeIdx = *pCnNodeId - 1;
        if (nodeIdx < NUM_DLL_MNCN_LOSSPRES_OBJS)
        {
            if (instance_l.aMnCnLossPresEvent[nodeIdx] ==
                ERRORHANDLERK_CN_LOSS_PRES_EVENT_NONE)
            {
                errhndkcal_getMnCnLossPresThresholdCnt(nodeIdx, &thresholdCnt);
                if (thresholdCnt > 0)
                {
                    thresholdCnt--;
                    errhndkcal_setMnCnLossPresThresholdCnt(nodeIdx, thresholdCnt);
                }
            }
            else
            {
                if (instance_l.aMnCnLossPresEvent[nodeIdx] ==
                    ERRORHANDLERK_CN_LOSS_PRES_EVENT_OCC)
                {
                    instance_l.aMnCnLossPresEvent[nodeIdx] =
                                          ERRORHANDLERK_CN_LOSS_PRES_EVENT_NONE;
                }
            }
        }
        pCnNodeId++;
    }

    if ((instance_l.dllErrorEvents & DLL_ERR_MN_CRC) == 0)
    {   // decrement CRC threshold counter, because it didn't occur last cycle
        errhndkcal_getMnCrcThresholdCnt(&thresholdCnt);
        if (thresholdCnt > 0)
        {
            thresholdCnt--;
            errhndkcal_setMnCrcThresholdCnt(thresholdCnt);
        }
    }

    if ((instance_l.dllErrorEvents & DLL_ERR_MN_CYCTIMEEXCEED) == 0)
    {   // decrement cycle exceed threshold counter, because it didn't occur last cycle
        errhndkcal_getMnCycTimeExceedThresholdCnt(&thresholdCnt);
        if (thresholdCnt > 0)
        {
            thresholdCnt--;
            errhndkcal_setMnCycTimeExceedThresholdCnt(thresholdCnt);
        }
    }

    return kErrorOk;
}
#endif

//------------------------------------------------------------------------------
/**
\brief    Decrement CN error counters

The function decrements the error counters used by a CN node.
*/
//------------------------------------------------------------------------------
static void decrementCnCounters(void)
{
    UINT32  thresholdCnt;

    if ((instance_l.dllErrorEvents & DLL_ERR_CN_LOSS_SOC) == 0)
    {   // decrement loss of SoC threshold counter, because it didn't occur last cycle
        errhndkcal_getLossSocThresholdCnt(&thresholdCnt);
        if (thresholdCnt > 0)
        {
            thresholdCnt--;
            errhndkcal_setLossSocThresholdCnt(thresholdCnt);
        }
    }

    if ((instance_l.dllErrorEvents & DLL_ERR_CN_CRC) == 0)
    {   // decrement CRC threshold counter, because it didn't occur last cycle
        errhndkcal_getCnCrcThresholdCnt(&thresholdCnt);
        if (thresholdCnt > 0)
        {
            thresholdCnt--;
            errhndkcal_setCnCrcThresholdCnt(thresholdCnt);
        }
    }
}

//------------------------------------------------------------------------------
/**
\brief    Handle a CN LossOfSoc error

The function checks if a CN Loss of SoC error occurred. It updates the
appropriate error counters, generates a history entry and posts the error event
to the NMT.

\param[in]      pEvent_p            Pointer to error event provided by DLL.

\return Returns kErrorOk or error code
*/
//------------------------------------------------------------------------------
static tOplkError handleCnLossSoc(const tEvent* pEvent_p)
{
    tOplkError              ret = kErrorOk;
    const tEventDllError*   pErrorHandlerEvent = (const tEventDllError*)pEvent_p->eventArg.pEventArg;
    UINT32                  threshold, thresholdCnt, cumulativeCnt;

    // Check if loss of SoC event occurred
    if ((pErrorHandlerEvent->dllErrorEvents & DLL_ERR_CN_LOSS_SOC) == 0)
        return kErrorOk;

    errhndkcal_getCnLossSocError(&cumulativeCnt, &thresholdCnt, &threshold);

    cumulativeCnt++;
    // According to spec threshold counting is disabled by setting threshold to 0
    if (threshold > 0)
    {
        thresholdCnt += 8;

        if (thresholdCnt >= threshold)
        {
            generateHistoryEntry(E_DLL_LOSS_SOC_TH, pEvent_p->netTime);
            if (ret != kErrorOk)
            {
                errhndkcal_setCnLossSocCounters(cumulativeCnt, thresholdCnt);
                return ret;
            }

            BENCHMARK_MOD_02_TOGGLE(7);

            postNmtEvent(kNmtEventNmtCycleError);
            thresholdCnt = 0;
        }
        instance_l.dllErrorEvents |= DLL_ERR_CN_LOSS_SOC;
    }

    errhndkcal_setCnLossSocCounters(cumulativeCnt, thresholdCnt);

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief    Handle a CN LossOfPReq error

The function checks if a CN Loss of PReq error occurred. It updates the
appropriate error counters, generates a history entry and posts the error event
to the NMT.

\param[in]      pEvent_p            Pointer to error event provided by DLL.

\return Returns kErrorOk or error code
*/
//------------------------------------------------------------------------------
static tOplkError handleCnLossPreq(const tEvent* pEvent_p)
{
    tOplkError              ret;
    const tEventDllError*   pErrorHandlerEvent = (const tEventDllError*)pEvent_p->eventArg.pEventArg;
    UINT32                  threshold, thresholdCnt, cumulativeCnt;

    // check if loss of PReq event occurred
    if ((pErrorHandlerEvent->dllErrorEvents & DLL_ERR_CN_LOSS_PREQ) == 0)
        return kErrorOk;

    errhndkcal_getCnLossPreqError(&cumulativeCnt, &thresholdCnt, &threshold);

    cumulativeCnt++;
    // According to spec threshold counting is disabled by setting threshold to 0
    if (threshold > 0)
    {
        thresholdCnt += 8;

        if (thresholdCnt >= threshold)
        {
            ret = generateHistoryEntry(E_DLL_LOSS_PREQ_TH, pEvent_p->netTime);
            if (ret != kErrorOk)
            {
                errhndkcal_setCnLossPreqCounters(cumulativeCnt, thresholdCnt);
                return ret;
            }

            BENCHMARK_MOD_02_TOGGLE(7);

            postNmtEvent(kNmtEventNmtCycleError);
            thresholdCnt = 0;
        }
    }
    errhndkcal_setCnLossPreqCounters(cumulativeCnt, thresholdCnt);

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief    Handle a correct PReq

The function checks if a PReq was successfully received. If it is, the
appropriate error counter will be decremented.

\param[in]      pEvent_p            Pointer to error event provided by DLL.
*/
//------------------------------------------------------------------------------
static void handleCorrectPreq(const tEvent* pEvent_p)
{
    const tEventDllError*   pErrorHandlerEvent = (const tEventDllError*)pEvent_p->eventArg.pEventArg;
    UINT32                  thresholdCnt;

    errhndkcal_getLossPreqThresholdCnt(&thresholdCnt);

    if ((thresholdCnt == 0) ||
        ((pErrorHandlerEvent->dllErrorEvents & DLL_ERR_CN_RECVD_PREQ) == 0))
        return;

    // PReq correctly received
    thresholdCnt--;
    errhndkcal_setLossPreqThresholdCnt(thresholdCnt);
}

//------------------------------------------------------------------------------
/**
\brief    Handle a CN CRC error

The function checks if a CN CRC error occurred. It updates the
appropriate error counters, generates a history entry and posts the error event
to the NMT.

\param[in]      pEvent_p            Pointer to error event provided by DLL.

\return Returns kErrorOk or error code
*/
//------------------------------------------------------------------------------
static tOplkError handleCnCrc(const tEvent* pEvent_p)
{
    tOplkError              ret;
    const tEventDllError*   pErrorHandlerEvent = (const tEventDllError*)pEvent_p->eventArg.pEventArg;
    UINT32                  threshold, thresholdCnt, cumulativeCnt;

    // Check if CRC error event occurred
    if ((pErrorHandlerEvent->dllErrorEvents & DLL_ERR_CN_CRC) == 0)
        return kErrorOk;

    errhndkcal_getCnCrcError(&cumulativeCnt, &thresholdCnt, &threshold);

    cumulativeCnt++;
    // According to spec threshold counting is disabled by setting threshold to 0
    if (threshold > 0)
    {
        thresholdCnt += 8;

        if (thresholdCnt >= threshold)
        {
            ret = generateHistoryEntry(E_DLL_CRC_TH, pEvent_p->netTime);
            if (ret != kErrorOk)
            {
                errhndkcal_setCnCrcCounters(cumulativeCnt, thresholdCnt);
                return ret;
            }

            BENCHMARK_MOD_02_TOGGLE(7);

            postNmtEvent(kNmtEventNmtCycleError);
            thresholdCnt = 0;
        }
        instance_l.dllErrorEvents |= DLL_ERR_CN_CRC;
    }

    errhndkcal_setCnCrcCounters(cumulativeCnt, thresholdCnt);
    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief    Handle a invalid format error

The function checks if a invalid format error occurred. An appropriate history
entry will be generated. If the node is acting as MN, the CN causing the error
is removed from the isochronous phase.

\param[in]      pEvent_p            Pointer to error event provided by DLL.

\return Returns kErrorOk or error code
*/
//------------------------------------------------------------------------------
static tOplkError handleInvalidFormat(const tEvent* pEvent_p)
{
    tOplkError              ret;
    const tEventDllError*   pErrorHandlerEvent = (const tEventDllError*)pEvent_p->eventArg.pEventArg;

    // check if invalid format error occurred (only direct reaction)
    if ((pErrorHandlerEvent->dllErrorEvents & DLL_ERR_INVALID_FORMAT) == 0)
        return kErrorOk;

    ret = generateHistoryEntryNodeId(E_DLL_INVALID_FORMAT,
                                     pEvent_p->netTime,
                                     pErrorHandlerEvent->nodeId);
    if (ret != kErrorOk)
        return ret;

    BENCHMARK_MOD_02_TOGGLE(7);

#if defined(CONFIG_INCLUDE_NMT_MN)
    if (NMT_IF_ACTIVE_MN(pErrorHandlerEvent->nmtState))
    {   // MN is active
        if (pErrorHandlerEvent->nodeId != 0)
        {
            tDllNodeOpParam     nodeOpParam;

            nodeOpParam.opNodeType = kDllNodeOpTypeIsochronous;
            nodeOpParam.nodeId = pErrorHandlerEvent->nodeId;
            // remove node from isochronous phase
            dllk_deleteNode(&nodeOpParam);

            // inform NmtMnu module about state change, which shall send
            // NMT command ResetNode to this CN
            postHeartbeatEvent(pErrorHandlerEvent->nodeId,
                               kNmtCsNotActive,
                               E_DLL_INVALID_FORMAT);
        }
        // $$$ and else should lead to InternComError
    }
    else
    {
        postNmtEvent(kNmtEventInternComError);
    }
#else
    // CN is active
    postNmtEvent(kNmtEventInternComError);
#endif

    return kErrorOk;
}

#if defined(CONFIG_INCLUDE_NMT_MN)
//------------------------------------------------------------------------------
/**
\brief    Handle an MN CRC error

The function checks if an MN CRC error occurred. It updates the
appropriate error counters, generates a history entry and posts the error event
to the NMT.

\param[in]      pEvent_p            Pointer to error event provided by DLL.

\return Returns kErrorOk or error code
*/
//------------------------------------------------------------------------------
static tOplkError handleMnCrc(const tEvent* pEvent_p)
{
    tOplkError              ret;
    const tEventDllError*   pErrorHandlerEvent = (const tEventDllError*)pEvent_p->eventArg.pEventArg;
    UINT32                  threshold, thresholdCnt, cumulativeCnt;

    // check if CRC error event occurred
    if ((pErrorHandlerEvent->dllErrorEvents & DLL_ERR_MN_CRC) == 0)
        return kErrorOk;

    errhndkcal_getMnCrcError(&cumulativeCnt, &thresholdCnt, &threshold);

    cumulativeCnt++;
    // According to spec threshold counting is disabled by setting threshold to 0
    if (threshold > 0)
    {
        thresholdCnt += 8;
        if (thresholdCnt >= threshold)
        {
            ret = generateHistoryEntry(E_DLL_CRC_TH, pEvent_p->netTime);
            if (ret != kErrorOk)
            {
                errhndkcal_setMnCrcCounters(cumulativeCnt, thresholdCnt);
                return ret;
            }
            postNmtEvent(kNmtEventNmtCycleError);
            thresholdCnt = 0;
        }
        instance_l.dllErrorEvents |= DLL_ERR_MN_CRC;
    }
    errhndkcal_setMnCrcCounters(cumulativeCnt, thresholdCnt);

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief    Handle an MN cycle time exceeded error

The function checks if an MN cycle time exceeded error occurred. It updates the
appropriate error counters and generates a history entry.

\param[in]      pEvent_p            Pointer to error event provided by DLL.

\return Returns kErrorOk or error code
*/
//------------------------------------------------------------------------------
static tOplkError handleMnCycTimeExceed(const tEvent* pEvent_p)
{
    tOplkError              ret = kErrorOk;
    const tEventDllError*   pErrorHandlerEvent = (const tEventDllError*)pEvent_p->eventArg.pEventArg;
    UINT32                  threshold, thresholdCnt, cumulativeCnt;

    // check if cycle time exceeded event occurred
    if ((pErrorHandlerEvent->dllErrorEvents & DLL_ERR_MN_CYCTIMEEXCEED) == 0)
        return kErrorOk;

    errhndkcal_getMnCycTimeExceedError(&cumulativeCnt,
                                       &thresholdCnt,
                                       &threshold);

    cumulativeCnt++;

    // According to spec threshold counting is disabled by setting threshold to 0
    if (threshold > 0)
    {
        thresholdCnt += 8;
        if (thresholdCnt >= threshold)
        {
            ret = generateHistoryEntryWithError(E_DLL_CYCLE_EXCEED_TH,
                                                pEvent_p->netTime,
                                                pErrorHandlerEvent->oplkError);
            if (ret != kErrorOk)
            {
                errhndkcal_setMnCycTimeExceedCounters(thresholdCnt, cumulativeCnt);
                return ret;
            }
            postNmtEvent(kNmtEventNmtCycleError);
            thresholdCnt = 0;
        }
        else
        {
            ret = generateHistoryEntryWithError(E_DLL_CYCLE_EXCEED,
                                                pEvent_p->netTime,
                                                pErrorHandlerEvent->oplkError);
            if (ret != kErrorOk)
            {
                errhndkcal_setMnCycTimeExceedCounters(cumulativeCnt, thresholdCnt);
                return ret;
            }
        }
        instance_l.dllErrorEvents |= DLL_ERR_MN_CYCTIMEEXCEED;
    }
    errhndkcal_setMnCycTimeExceedCounters(cumulativeCnt, thresholdCnt);

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief    Handle a Loss of PRes error

The function checks if a Loss of PRes error occurred for a CN. It updates the
appropriate error counters and generates a history entry. If the threshold
count is reached, the CN will be removed from the isochronous phase.

\param[in]      pEvent_p            Pointer to error event provided by DLL.

\return Returns kErrorOk or error code
*/
//------------------------------------------------------------------------------
static tOplkError handleMnCnLossPres(const tEvent* pEvent_p)
{
    tOplkError              ret;
    UINT                    nodeIdx;
    tDllNodeOpParam         nodeOpParam;
    const tEventDllError*   pErrorHandlerEvent = (const tEventDllError*)pEvent_p->eventArg.pEventArg;
    UINT32                  threshold, thresholdCnt, cumulativeCnt;

    if ((pErrorHandlerEvent->dllErrorEvents & DLL_ERR_MN_CN_LOSS_PRES) == 0)
        return kErrorOk;

    nodeIdx = pErrorHandlerEvent->nodeId - 1;

    //if (nodeIdx >= tabentries(pErrorObjects_p->mnCnLossPresCumCnt))
    //    return kErrorOk;

    errhndkcal_getMnCnLossPresError(nodeIdx,
                                    &cumulativeCnt,
                                    &thresholdCnt,
                                    &threshold);

    if (instance_l.aMnCnLossPresEvent[nodeIdx] !=
                                  ERRORHANDLERK_CN_LOSS_PRES_EVENT_NONE)
        return kErrorOk;

    cumulativeCnt++;

    // According to spec threshold counting is disabled by setting threshold to 0
    if (threshold > 0)
    {
        thresholdCnt += 8;

        if (thresholdCnt >= threshold)
        {
            instance_l.aMnCnLossPresEvent[nodeIdx] =
                            ERRORHANDLERK_CN_LOSS_PRES_EVENT_THR;

            ret = generateHistoryEntryNodeId(E_DLL_LOSS_PRES_TH,
                                             pEvent_p->netTime,
                                             pErrorHandlerEvent->nodeId);
            if (ret != kErrorOk)
            {
                errhndkcal_setMnCnLossPresCounters(nodeIdx,
                                                   cumulativeCnt,
                                                   thresholdCnt);
                return ret;
            }

            // remove node from isochronous phase
            nodeOpParam.opNodeType = kDllNodeOpTypeIsochronous;
            nodeOpParam.nodeId = pErrorHandlerEvent->nodeId;
            ret = dllk_deleteNode(&nodeOpParam);
            if (ret != kErrorOk)
            {
                DEBUG_LVL_ERROR_TRACE("%s remove node %d from isochronous phase failed with 0x%X\n",
                                      __func__,
                                      nodeOpParam.nodeId,
                                      ret);
            }

            // inform NmtMnu module about state change, which shall send
            // NMT command ResetNode to this CN
            postHeartbeatEvent(pErrorHandlerEvent->nodeId,
                               kNmtCsNotActive,
                               E_DLL_LOSS_PRES_TH);
            thresholdCnt = 0;
        }
        else
        {
            instance_l.aMnCnLossPresEvent[nodeIdx] =
                            ERRORHANDLERK_CN_LOSS_PRES_EVENT_OCC;
        }
    }
    errhndkcal_setMnCnLossPresCounters(nodeIdx, cumulativeCnt, thresholdCnt);

    return kErrorOk;
}

#endif

//------------------------------------------------------------------------------
/**
\brief    Handle a DLL errors

The function is called by the error handler's process function and calls the
different error handling functions to update the error counters.

\param[in]      pEvent_p            Pointer to error event provided by DLL.

\return Returns kErrorOk or error code
*/
//------------------------------------------------------------------------------
static tOplkError handleDllErrors(const tEvent* pEvent_p)
{
    tOplkError  ret;

    // check the different error events
    ret = handleCnLossSoc(pEvent_p);
    if (ret != kErrorOk)
        return ret;

    ret = handleCnLossPreq(pEvent_p);
    if (ret != kErrorOk)
        return ret;

    handleCorrectPreq(pEvent_p);

    ret = handleCnCrc(pEvent_p);
    if (ret != kErrorOk)
        return ret;

    ret = handleInvalidFormat(pEvent_p);
    if (ret != kErrorOk)
        return ret;

#if defined(CONFIG_INCLUDE_NMT_MN)
    ret = handleMnCrc(pEvent_p);
    if (ret != kErrorOk)
        return ret;

    ret = handleMnCycTimeExceed(pEvent_p);
    if (ret != kErrorOk)
        return ret;

    ret = handleMnCnLossPres(pEvent_p);
    if (ret != kErrorOk)
        return ret;
#endif

    return ret;
}

#if defined(CONFIG_INCLUDE_NMT_MN)
//------------------------------------------------------------------------------
/**
\brief    Post a heartbeat event

The function is used to post a heartbeat event to the NMT.

\param[in]      nodeId_p            Node Id of CN to be posted.
\param[in]      state_p             State to be posted.
\param[in]      errorCode_p         Error Code which occurred.

\return Returns kErrorOk or error code
*/
//------------------------------------------------------------------------------
static tOplkError postHeartbeatEvent(UINT nodeId_p,
                                     tNmtState state_p,
                                     UINT16 errorCode_p)
{
    tOplkError      ret;
    tHeartbeatEvent heartbeatEvent;
    tEvent          event;

    heartbeatEvent.nodeId = nodeId_p;
    heartbeatEvent.nmtState = state_p;
    heartbeatEvent.errorCode = errorCode_p;
    event.eventSink = kEventSinkNmtMnu;
    event.eventType = kEventTypeHeartbeat;
    event.eventArgSize = sizeof(heartbeatEvent);
    event.eventArg.pEventArg = &heartbeatEvent;

    ret = eventk_postEvent(&event);

    return ret;
}
#endif

//------------------------------------------------------------------------------
/**
\brief    Post a history entry event

The function is used to post a history entry event to the API.

\param[in]      pHistoryEntry_p     Pointer to event which should be posted.

\return Returns kErrorOk or error code
*/
//------------------------------------------------------------------------------
static tOplkError postHistoryEntryEvent(const tErrHistoryEntry* pHistoryEntry_p)
{
    tOplkError  ret;
    tEvent      event;

    event.eventSink = kEventSinkApi;
    event.eventType = kEventTypeHistoryEntry;
    event.eventArgSize = sizeof(*pHistoryEntry_p);
    event.eventArg.pEventArg = (void*)pHistoryEntry_p;

    ret = eventk_postEvent(&event);

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief    Generate a history entry

The function generates a history entry by setting up a history entry event and
posting it to the API.

\param[in]      errorCode_p         Error which occurred.
\param[in]      netTime_p           Timestamp at which error occurred.

\return Returns kErrorOk or error code
*/
//------------------------------------------------------------------------------
static tOplkError generateHistoryEntry(UINT16 errorCode_p, tNetTime netTime_p)
{
    tOplkError          ret;
    tErrHistoryEntry    historyEntry;

    historyEntry.entryType = ERR_ENTRYTYPE_MODE_OCCURRED |
                             ERR_ENTRYTYPE_PROF_PLK |
                             ERR_ENTRYTYPE_HISTORY;

    historyEntry.errorCode = errorCode_p;
    historyEntry.timeStamp = netTime_p;
    OPLK_MEMSET(historyEntry.aAddInfo, 0, sizeof(historyEntry.aAddInfo));

    ret = postHistoryEntryEvent(&historyEntry);

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief    Generate a history entry for a specific node ID

The function generates a history entry for a specific node ID. This is done
by setting up a history entry event and posting it to the API.

\param[in]      errorCode_p         Error which occurred
\param[in]      netTime_p           Timestamp at which error occurred
\param[in]      nodeId_p            Node ID for which to generate history entry

\return Returns kErrorOk or error code
*/
//------------------------------------------------------------------------------
static tOplkError generateHistoryEntryNodeId(UINT16 errorCode_p,
                                             tNetTime netTime_p,
                                             UINT nodeId_p)
{
    tOplkError          ret;
    tErrHistoryEntry    historyEntry;

    historyEntry.entryType = ERR_ENTRYTYPE_MODE_OCCURRED |
                             ERR_ENTRYTYPE_PROF_PLK |
                             ERR_ENTRYTYPE_HISTORY;

    historyEntry.errorCode = errorCode_p;
    historyEntry.timeStamp = netTime_p;
    ami_setUint8Le(&historyEntry.aAddInfo[0], (UINT8)nodeId_p);

    ret = postHistoryEntryEvent(&historyEntry);

    return ret;
}

#if defined(CONFIG_INCLUDE_NMT_MN)
//------------------------------------------------------------------------------
/**
\brief    Generate a history entry containing an error flag

The function generates a history entry which contains an additional error
flag. This is done by setting up a history entry event and posting it to the
API.

\param[in]      errorCode_p         Error which occurred.
\param[in]      netTime_p           Timestamp at which error occurred.
\param[in]      oplkError_p         Error flag to be included in history entry.

\return Returns kErrorOk or error code
*/
//------------------------------------------------------------------------------
static tOplkError generateHistoryEntryWithError(UINT16 errorCode_p,
                                                tNetTime netTime_p,
                                                UINT16 oplkError_p)
{
    tOplkError          ret;
    tErrHistoryEntry    historyEntry;

    historyEntry.entryType = ERR_ENTRYTYPE_MODE_OCCURRED |
                             ERR_ENTRYTYPE_PROF_PLK |
                             ERR_ENTRYTYPE_HISTORY;

    historyEntry.errorCode = errorCode_p;
    historyEntry.timeStamp = netTime_p;
    ami_setUint16Le(&historyEntry.aAddInfo[0], (UINT16)oplkError_p);

    ret = postHistoryEntryEvent(&historyEntry);

    return ret;
}
#endif

//------------------------------------------------------------------------------
/**
\brief    Post an NMT event

The function posts an NMT event to the NMT.

\param[in]      nmtEvent_p          NMT event to post.

\return Returns kErrorOk or error code
*/
//------------------------------------------------------------------------------
static tOplkError postNmtEvent(tNmtEvent nmtEvent_p)
{
    tOplkError  ret;
    tNmtEvent   nmtEvent;
    tEvent      event;

    nmtEvent = nmtEvent_p;
    event.eventSink = kEventSinkNmtk;
    event.eventType = kEventTypeNmtEvent;
    event.eventArg.pEventArg = &nmtEvent;
    event.eventArgSize = sizeof(nmtEvent);

    ret = eventk_postEvent(&event);

    return ret;
}

/// \}
