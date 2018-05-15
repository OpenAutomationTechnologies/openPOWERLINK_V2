/**
********************************************************************************
\file   dllkcal.c

\brief  Kernel DLL CAL module

This file contains the kernel DLL CAL module.

\ingroup module_dllkcal
*******************************************************************************/
/*------------------------------------------------------------------------------
Copyright (c) 2013, SYSTEC electronic GmbH
Copyright (c) 2017, B&R Industrial Automation GmbH
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
#include <stddef.h>

#include <common/ami.h>
#include <common/dllcal.h>
#include <common/nmt.h>
#include <kernel/dllkcal.h>
#include <kernel/dllk.h>

#include <kernel/eventk.h>

#ifdef CONFIG_INCLUDE_NMT_MN
#include <common/circbuffer.h>
#endif

#if (defined(CONFIG_INCLUDE_NMT_MN) && (CONFIG_DLLCAL_QUEUE == DIRECT_QUEUE))
#error "DLLCal module does not support direct calls with PRC MN"
#endif

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
#if defined(CONFIG_INCLUDE_NMT_MN)
#define DLLKCAL_MAX_QUEUES  6   // CnGenReq, CnNmtReq, {MnGenReq, MnNmtReq}, MnIdentReq, MnStatusReq, SyncReq
#else
#define DLLKCAL_MAX_QUEUES  5   // CnGenReq, CnNmtReq, {MnGenReq, MnNmtReq}, MnIdentReq, MnStatusReq
#endif

//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------
/**
\brief Asynchronous Tx queue select enum for generic priority

This enum is used to select the asynchronous Tx queue with generic priority.
*/
typedef enum
{
    kDllkCalTxQueueSelectGen    = 0,    ///< TxGen is selected
    kDllkCalTxQueueSelectVeth   = 1,    ///< TxVeth is selected
    kDllkCalTxQueueSelectLast,          ///< Dummy enum to get select count
} eDllkCalTxQueueSelect;

/**
\brief Asynchronous Tx queue select data type

Data type for the enumerator \ref eDllkCalTxQueueSelect.
*/
typedef UINT32 tDllkCalTxQueueSelect;

/**
\brief Node instance

This structure contains local parameters for node, used to
identify extended NMT commands.
*/
typedef struct
{
    UINT                    nodeId;                 ///< Node ID
    UINT                    extNmtCmdByteOffset;    ///< Extended NMT command byte offset
    UINT8                   extNmtCmdBitMask;       ///< Extended NMT command Bit mask
} tDllkNodeInstance;

/**
\brief DLLk CAL instance type

This structure defines an instance of the POWERLINK Data Link Layer
Communication Abstraction Layer kernel module.
*/
typedef struct
{
#if defined(CONFIG_INCLUDE_VETH)
    tDllCalQueueInstance    dllCalQueueTxVeth;      ///< DLL CAL queue instance for virtual Ethernet
    tDllCalFuncIntf*        pTxVethFuncs;           ///< Function pointer to the TX functions for virtual Ethernet
#endif

    tDllkCalTxQueueSelect   currentTxQueueSelect;   ///< Current Tx queue (TxGen vs. TxVeth)

    tDllCalQueueInstance    dllCalQueueTxNmt;       ///< DLL CAL queue instance for NMT priority
    tDllCalFuncIntf*        pTxNmtFuncs;            ///< Function pointer to the TX functions for NMT priority

    tDllCalQueueInstance    dllCalQueueTxGen;       ///< DLL CAL queue instance for generic priority
    tDllCalFuncIntf*        pTxGenFuncs;            ///< Function pointer to the TX functions for generic priority

#if defined(CONFIG_INCLUDE_NMT_MN)
    tDllCalQueueInstance    dllCalQueueTxSync;      ///< DLL CAL queue instance for SyncRequest frames
    tDllCalFuncIntf*        pTxSyncFuncs;           ///< Function pointer to the TX functions for SyncRequest frames
#endif

    tDllkCalStatistics      statistics;             ///< DLL CAL statistics

#if (CONFIG_DLL_DEFERRED_RXFRAME_RELEASE_ASYNC != FALSE)
    UINT                    asyncFrameReceived;     ///< Asynchronous frames received counter
    UINT                    asyncFrameFreed;        ///< Asynchronous frames freed counter
#endif

#if defined(CONFIG_INCLUDE_NMT_MN)
    tCircBufInstance*       pQueueIdentReq;         ///< IdentRequest queue with the CN node IDs
    tCircBufInstance*       pQueueStatusReq;        ///< StatusRequest queue with the CN node IDs

    tCircBufInstance*       pQueueCnRequestNmt;     ///< Queue for NMT priority CN requests
    UINT                    aCnRequestCntNmt[254];  ///< Array of requested frames in the NMT priority queues of each CN
    tCircBufInstance*       pQueueCnRequestGen;     ///< Queue for generic priority CN requests
    UINT                    aCnRequestCntGen[254];  ///< Array of requested frames in the generic priority queues of each CN

    UINT                    nextRequestQueue;       ///< Number of next request queue to be scheduled
#endif

    tDllkNodeInstance       nodeInstance;           ///< Initialize the node instance
} tDllkCalInstance;
//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tDllkCalInstance     instance_l;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
#if defined(CONFIG_INCLUDE_NMT_MN)
static BOOL getCnGenRequest(tDllReqServiceId* pReqServiceId_p, UINT* pNodeId_p);
static BOOL getCnNmtRequest(tDllReqServiceId* pReqServiceId_p, UINT* pNodeId_p);
static BOOL getMnGenNmtRequest(tDllReqServiceId* pReqServiceId_p, UINT* pNodeId_p);
static BOOL getMnIdentRequest(tDllReqServiceId* pReqServiceId_p, UINT* pNodeId_p);
static BOOL getMnStatusRequest(tDllReqServiceId* pReqServiceId_p, UINT* pNodeId_p);
static BOOL getMnSyncRequest(tDllReqServiceId* pReqServiceId_p,
                             UINT* pNodeId_p,
                             tSoaPayload* pSoaPayload_p);
#endif

static tOplkError sendGenericAsyncFrame(tFrameInfo* pFrameInfo_p);
static tOplkError getGenericAsyncFrame(void* pFrame_p, size_t* pFrameSize_p);
static tNmtEvent  commandTranslator(const tNmtCommandService* pNmtCommand_p);
static BOOL       checkNodeIdList(const tNmtCommandService* pNmtCommand_p);
static void       initNodeInstance(UINT nodeId_p);

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief Initialize kernel DLL CAL module

This function initializes the kernel DLL CAL module.

\return The function returns a tOplkError error code.

\ingroup module_dllkcal
*/
//------------------------------------------------------------------------------
tOplkError dllkcal_init(void)
{
    tOplkError      ret = kErrorOk;
#if defined(CONFIG_INCLUDE_NMT_MN)
    tCircBufError   circErr;
#endif

    // reset instance structure
    OPLK_MEMSET(&instance_l, 0, sizeof(instance_l));

    instance_l.pTxNmtFuncs = GET_DLLKCAL_INTERFACE();
    instance_l.pTxGenFuncs = GET_DLLKCAL_INTERFACE();
#if defined(CONFIG_INCLUDE_NMT_MN)
    instance_l.pTxSyncFuncs = GET_DLLKCAL_INTERFACE();
#endif
#if defined(CONFIG_INCLUDE_VETH)
    instance_l.pTxVethFuncs = GET_DLLKCAL_INTERFACE();
#endif

    ret = instance_l.pTxNmtFuncs->pfnAddInstance(&instance_l.dllCalQueueTxNmt,
                                                 kDllCalQueueTxNmt);
    if (ret != kErrorOk)
    {
        DEBUG_LVL_ERROR_TRACE("%s() TxNmt failed\n", __func__);
        goto Exit;
    }

    ret = instance_l.pTxGenFuncs->pfnAddInstance(&instance_l.dllCalQueueTxGen,
                                                 kDllCalQueueTxGen);
    if (ret != kErrorOk)
    {
        DEBUG_LVL_ERROR_TRACE("%s() TxGen failed\n", __func__);
        goto Exit;
    }

#if defined(CONFIG_INCLUDE_NMT_MN)
    ret = instance_l.pTxSyncFuncs->pfnAddInstance(&instance_l.dllCalQueueTxSync,
                                                  kDllCalQueueTxSync);
    if (ret != kErrorOk)
    {
        DEBUG_LVL_ERROR_TRACE("%s() TxSync failed\n", __func__);
        goto Exit;
    }
    circErr = circbuf_alloc(CIRCBUF_DLLCAL_CN_REQ_NMT,
                            CONFIG_DLLCAL_SIZE_CIRCBUF_CN_REQ_NMT,
                            &instance_l.pQueueCnRequestNmt);
    if (circErr != kCircBufOk)
    {
        DEBUG_LVL_ERROR_TRACE("%s() Allocate CIRCBUF_ASYNC_SCHED_NMT failed\n", __func__);
        goto Exit;
    }

    circErr = circbuf_alloc(CIRCBUF_DLLCAL_CN_REQ_GEN,
                            CONFIG_DLLCAL_SIZE_CIRCBUF_CN_REQ_GEN,
                            &instance_l.pQueueCnRequestGen);
    if (circErr != kCircBufOk)
    {
        DEBUG_LVL_ERROR_TRACE("%s() Allocate CIRCBUF_ASYNC_SCHED_GEN failed\n", __func__);
        goto Exit;
    }

    circErr = circbuf_alloc(CIRCBUF_DLLCAL_CN_REQ_IDENT,
                            CONFIG_DLLCAL_SIZE_CIRCBUF_REQ_IDENT,
                            &instance_l.pQueueIdentReq);
    if (circErr != kCircBufOk)
    {
        DEBUG_LVL_ERROR_TRACE("%s() Allocate CIRCBUF_DLLCAL_CN_REQ_IDENT failed\n", __func__);
        goto Exit;
    }

    circErr = circbuf_alloc(CIRCBUF_DLLCAL_CN_REQ_STATUS,
                            CONFIG_DLLCAL_SIZE_CIRCBUF_REQ_STATUS,
                            &instance_l.pQueueStatusReq);
    if (circErr != kCircBufOk)
    {
        DEBUG_LVL_ERROR_TRACE("%s() Allocate CIRCBUF_DLLCAL_CN_REQ_STATUS failed\n", __func__);
        goto Exit;
    }
#endif

#if defined(CONFIG_INCLUDE_VETH)
    ret = instance_l.pTxVethFuncs->pfnAddInstance(&instance_l.dllCalQueueTxVeth,
                                                  kDllCalQueueTxVeth);
    if (ret != kErrorOk)
    {
        goto Exit;
    }
#endif

    instance_l.currentTxQueueSelect = kDllkCalTxQueueSelectGen;

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief Clean up the kernel DLL CAL module

This function cleans up the kernel DLL CAL module.

\return The function returns a tOplkError error code.

\ingroup module_dllkcal
*/
//------------------------------------------------------------------------------
tOplkError dllkcal_exit(void)
{
    tOplkError      ret = kErrorOk;

#ifdef CONFIG_INCLUDE_NMT_MN
    if (instance_l.pQueueCnRequestGen != NULL)
        circbuf_free(instance_l.pQueueCnRequestGen);

    if (instance_l.pQueueCnRequestNmt != NULL)
        circbuf_free(instance_l.pQueueCnRequestNmt);

    if (instance_l.pQueueIdentReq != NULL)
        circbuf_free(instance_l.pQueueIdentReq);

    if (instance_l.pQueueStatusReq != NULL)
        circbuf_free(instance_l.pQueueStatusReq);
#endif

    if (instance_l.pTxNmtFuncs != NULL)
        instance_l.pTxNmtFuncs->pfnDelInstance(instance_l.dllCalQueueTxNmt);

    if (instance_l.pTxGenFuncs != NULL)
        instance_l.pTxGenFuncs->pfnDelInstance(instance_l.dllCalQueueTxGen);

#if defined(CONFIG_INCLUDE_NMT_MN)
    if (instance_l.pTxSyncFuncs != NULL)
        instance_l.pTxSyncFuncs->pfnDelInstance(instance_l.dllCalQueueTxSync);
#endif
#if defined(CONFIG_INCLUDE_VETH)
    if (instance_l.pTxVethFuncs != NULL)
        instance_l.pTxVethFuncs->pfnDelInstance(instance_l.dllCalQueueTxVeth);
#endif

    // reset instance structure
    OPLK_MEMSET(&instance_l, 0, sizeof(instance_l));

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief Process events

This function is the event handler of the kernel DLL CAL module.

\param[in]      pEvent_p            Pointer to event to be processed.

\return The function returns a tOplkError error code.

\ingroup module_dllkcal
*/
//------------------------------------------------------------------------------
tOplkError dllkcal_process(const tEvent* pEvent_p)
{
    tOplkError                          ret = kErrorOk;
    const tDllCalAsndServiceIdFilter*   pServFilter;
    tDllIdentParam*                     pIdentParam;
    tDllConfigParam*                    pConfigParam;

#if defined(CONFIG_INCLUDE_NMT_MN)
    const tDllCalIssueRequest*          pIssueReq;
#endif

#if (NMT_MAX_NODE_ID > 0)
    const tDllNodeInfo*                 pNodeInfo;
    const tDllNodeOpParam*              pNodeOpParam;
#endif

#if (CONFIG_DLL_DEFERRED_RXFRAME_RELEASE_ASYNC != FALSE)
    const tFrameInfo*                   pFrameInfo;
#endif

    switch (pEvent_p->eventType)
    {
        case kEventTypeDllkServFilter:
            pServFilter = (const tDllCalAsndServiceIdFilter*)pEvent_p->eventArg.pEventArg;
            ret = dllk_setAsndServiceIdFilter(pServFilter->serviceId,
                                              pServFilter->filter);
            break;

#if defined(CONFIG_INCLUDE_NMT_MN)
        case kEventTypeDllkIssueReq:
            pIssueReq = (const tDllCalIssueRequest*)pEvent_p->eventArg.pEventArg;
            ret = dllkcal_issueRequest(pIssueReq->service,
                                       pIssueReq->nodeId,
                                       pIssueReq->soaFlag1);
            break;
#endif

#if (NMT_MAX_NODE_ID > 0)
        case kEventTypeDllkConfigNode:
            pNodeInfo = (const tDllNodeInfo*)pEvent_p->eventArg.pEventArg;
            ret = dllk_configNode(pNodeInfo);
            break;

        case kEventTypeDllkAddNode:
            pNodeOpParam = (const tDllNodeOpParam*)pEvent_p->eventArg.pEventArg;
            ret = dllk_addNode(pNodeOpParam);
            break;

        case kEventTypeDllkDelNode:
            pNodeOpParam = (const tDllNodeOpParam*)pEvent_p->eventArg.pEventArg;
            ret = dllk_deleteNode(pNodeOpParam);
            break;
#endif // NMT_MAX_NODE_ID > 0

        case kEventTypeDllkIdentity:
            pIdentParam = (tDllIdentParam*)pEvent_p->eventArg.pEventArg;
            if (pIdentParam->sizeOfStruct > pEvent_p->eventArgSize)
                pIdentParam->sizeOfStruct = pEvent_p->eventArgSize;

            ret = dllk_setIdentity(pIdentParam);
            break;

        case kEventTypeDllkConfig:
            pConfigParam = (tDllConfigParam*)pEvent_p->eventArg.pEventArg;
            if (pConfigParam->sizeOfStruct > pEvent_p->eventArgSize)
                pConfigParam->sizeOfStruct = pEvent_p->eventArgSize;

            ret = dllk_config(pConfigParam);
            initNodeInstance(pConfigParam->nodeId);
            break;

#if (CONFIG_DLL_DEFERRED_RXFRAME_RELEASE_ASYNC != FALSE)
        case kEventTypeReleaseRxFrame:
            pFrameInfo = (const tFrameInfo*)pEvent_p->eventArg.pEventArg;
            ret = dllk_releaseRxFrame(pFrameInfo->frame.pBuffer, pFrameInfo->frameSize);
            if (ret == kErrorOk)
            {
                instance_l.asyncFrameFreed++;
                instance_l.statistics.curRxFrameCount = instance_l.asyncFrameReceived - instance_l.asyncFrameFreed;
            }
            break;
#endif

        default:
            ret = kErrorInvalidEvent;
            break;
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief Get count of TX frames

This function returns the count of TX frames of the FIFO with highest priority.

\param[out]     pPriority_p         Pointer to store the FIFO type.
\param[out]     pCount_p            Pointer to store the number of TX frames.

\return The function returns a tOplkError error code.

\ingroup module_dllkcal
*/
//------------------------------------------------------------------------------
tOplkError dllkcal_getAsyncTxCount(tDllAsyncReqPriority* pPriority_p,
                                   UINT* pCount_p)
{
    tOplkError  ret = kErrorOk;
    UINT        frameCount;
    UINT        frameCountVeth;

    ret = instance_l.pTxNmtFuncs->pfnGetDataBlockCount(instance_l.dllCalQueueTxNmt,
                                                       &frameCount);
    if (ret != kErrorOk)
        goto Exit;

    if (frameCount > instance_l.statistics.maxTxFrameCountNmt)
        instance_l.statistics.maxTxFrameCountNmt = frameCount;

    if (frameCount != 0)
    {   // NMT requests are in queue
        *pPriority_p = kDllAsyncReqPrioNmt;
        *pCount_p = frameCount;
        goto Exit;
    }

    ret = instance_l.pTxGenFuncs->pfnGetDataBlockCount(instance_l.dllCalQueueTxGen,
                                                       &frameCount);
    if (ret != kErrorOk)
        goto Exit;

#if defined(CONFIG_INCLUDE_VETH)
    // Add VEth count to the generic queue count
    ret = instance_l.pTxVethFuncs->pfnGetDataBlockCount(instance_l.dllCalQueueTxVeth,
                                                        &frameCountVeth);
    if (ret != kErrorOk)
        goto Exit;

    frameCount += frameCountVeth;
#else
    UNUSED_PARAMETER(frameCountVeth);
#endif

    if (frameCount > instance_l.statistics.maxTxFrameCountGen)
        instance_l.statistics.maxTxFrameCountGen = frameCount;

    *pPriority_p = kDllAsyncReqPrioGeneric;
    *pCount_p = frameCount;

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief Get TX frame of specified FIFO

The function return TX frames form the specified FIFO.

\param[out]     pFrame_p            Pointer to store TX frame.
\param[out]     pFrameSize_p        Pointer to maximum size of buffer. Will be
                                    rewritten with actual size of frame.
\param[in]      priority_p          Priority of the FIFO.

\return The function returns a tOplkError error code.

\ingroup module_dllkcal
*/
//------------------------------------------------------------------------------
tOplkError dllkcal_getAsyncTxFrame(void* pFrame_p,
                                   size_t* pFrameSize_p,
                                   tDllAsyncReqPriority priority_p)
{
    tOplkError  ret = kErrorOk;

    switch (priority_p)
    {
        case kDllAsyncReqPrioNmt:    // NMT request priority
            ret = instance_l.pTxNmtFuncs->pfnGetDataBlock(
                                              instance_l.dllCalQueueTxNmt,
                                              pFrame_p,
                                              pFrameSize_p);
            break;

        default:    // generic priority
            ret = getGenericAsyncFrame(pFrame_p, pFrameSize_p);
            break;
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief Pass received ASnd frame to receive FIFO

The function passes a received ASnd frame to the receive FIFO. It will be called
only for frames with registered AsndServiceIds.

\param[in,out]  pFrameInfo_p        Pointer to frame info of received frame

\return The function returns a tOplkError error code.

\ingroup module_dllkcal
*/
//------------------------------------------------------------------------------
tOplkError dllkcal_asyncFrameReceived(tFrameInfo* pFrameInfo_p)
{
    tOplkError  ret = kErrorOk;
    tEvent      event;

#if (CONFIG_DLL_DEFERRED_RXFRAME_RELEASE_ASYNC == FALSE)
    // Copy the frame into event queue
    event.eventType = kEventTypeAsndRx;
    event.eventArg.pEventArg = pFrameInfo_p->frame.pBuffer;
    event.eventArgSize = pFrameInfo_p->frameSize;
#else
    tPlkFrame*  pTempFrame;

    // Clear padding before forwarding the event to user layer.
    pTempFrame = pFrameInfo_p->frame.pBuffer;
    pFrameInfo_p->frame.padding2 = 0;
    pFrameInfo_p->frame.pBuffer = pTempFrame;
    event.eventType = kEventTypeAsndRxInfo;
    event.eventArg.pEventArg = pFrameInfo_p;
    event.eventArgSize = sizeof(tFrameInfo);
#endif
    event.eventSink = kEventSinkDlluCal;

    ret = eventk_postEvent(&event);
#if (CONFIG_DLL_DEFERRED_RXFRAME_RELEASE_ASYNC != FALSE)
    if (ret == kErrorOk)
    {
        instance_l.asyncFrameReceived++;

        instance_l.statistics.curRxFrameCount = instance_l.asyncFrameReceived - instance_l.asyncFrameFreed;
        if (instance_l.statistics.curRxFrameCount > instance_l.statistics.maxRxFrameCount)
            instance_l.statistics.maxRxFrameCount = instance_l.statistics.curRxFrameCount;

        ret = kErrorReject; // Signalizes dllk to release buffer later
    }
#endif

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief Handle received NMT command

The function parses the received NMT commands and pass the corresponding NMT
event to the NMTK module for NMT command handling.

\param[in]      pNmtCommand_p       Pointer to the NMT command service info.

\return The function returns a tOplkError error code.

\ingroup module_dllkcal
*/
//------------------------------------------------------------------------------
tOplkError dllkcal_nmtCmdReceived(const tNmtCommandService* pNmtCommand_p)
{
    tOplkError  ret = kErrorOk;
    tEvent      event;
    tNmtEvent   nmtEvent;

    // Parse the NMT command and get the corresponding NMT event.
    nmtEvent = commandTranslator(pNmtCommand_p);
    event.eventSink = kEventSinkNmtk;
    event.netTime.nsec = 0;
    event.netTime.sec = 0;
    event.eventType = kEventTypeNmtEvent;
    event.eventArg.pEventArg = &nmtEvent;
    event.eventArgSize = sizeof(nmtEvent);

    ret = eventk_postEvent(&event);

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Send an asynchronous frame

The function puts the given frame into the transmit queue with the specified
priority.

\param[in]      pFrameInfo_p        Pointer to frame info structure
\param[in]      priority_p          Priority to send frame with

\return The function returns a tOplkError error code.

\ingroup module_dllkcal
*/
//------------------------------------------------------------------------------
tOplkError dllkcal_sendAsyncFrame(tFrameInfo* pFrameInfo_p,
                                  tDllAsyncReqPriority priority_p)
{
    tOplkError  ret = kErrorOk;
    tEvent      event;

    switch (priority_p)
    {
        case kDllAsyncReqPrioNmt:    // NMT request priority
            ret = instance_l.pTxNmtFuncs->pfnInsertDataBlock(
                                              instance_l.dllCalQueueTxNmt,
                                              pFrameInfo_p->frame.pBuffer,
                                              (size_t)pFrameInfo_p->frameSize);
            break;

        default:    // generic priority
            ret = sendGenericAsyncFrame(pFrameInfo_p);
            break;
    }

    if (ret != kErrorOk)
        goto Exit;

    // post event to DLL
    event.eventSink = kEventSinkDllk;
    event.eventType = kEventTypeDllkFillTx;
    OPLK_MEMSET(&event.netTime, 0x00, sizeof(event.netTime));
    event.eventArg.pEventArg = &priority_p;
    event.eventArgSize = sizeof(priority_p);

    ret = eventk_postEvent(&event);

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Write an asynchronous frame into the buffer

The function writes the given frame into the specified dll CAL queue.

\param[in]      pFrameInfo_p        Pointer to frame info structure
\param[in]      dllQueue_p          DllCal Queue to use

\return The function returns a tOplkError error code.

\ingroup module_dllkcal
*/
//------------------------------------------------------------------------------
tOplkError dllkcal_writeAsyncFrame(tFrameInfo* pFrameInfo_p, tDllCalQueue dllQueue_p)
{
    tOplkError  ret = kErrorOk;

    switch (dllQueue_p)
    {
        case kDllCalQueueTxNmt:    // NMT request priority
            ret = instance_l.pTxNmtFuncs->pfnInsertDataBlock(
                                              instance_l.dllCalQueueTxNmt,
                                              pFrameInfo_p->frame.pBuffer,
                                              (size_t)pFrameInfo_p->frameSize);
            break;

        case kDllCalQueueTxGen:    // generic priority
            ret = instance_l.pTxGenFuncs->pfnInsertDataBlock(
                                              instance_l.dllCalQueueTxGen,
                                              pFrameInfo_p->frame.pBuffer,
                                              (size_t)pFrameInfo_p->frameSize);
            break;
#if defined(CONFIG_INCLUDE_NMT_MN)
        case kDllCalQueueTxSync:   // sync request priority
            ret = instance_l.pTxSyncFuncs->pfnInsertDataBlock(
                                               instance_l.dllCalQueueTxSync,
                                               pFrameInfo_p->frame.pBuffer,
                                               (size_t)pFrameInfo_p->frameSize);
            break;
#endif
#if defined(CONFIG_INCLUDE_VETH)
        case kDllCalQueueTxVeth:   // virtual Ethernet
            ret = instance_l.pTxVethFuncs->pfnInsertDataBlock(
                                               instance_l.dllCalQueueTxVeth,
                                               pFrameInfo_p->frame.pBuffer,
                                               (size_t)pFrameInfo_p->frameSize);
            break;
#endif
        default:
            break;
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief Clear the asynchronous transmit buffer

The function clears the asynchronous transmit buffers.

\return The function returns a tOplkError error code.

\ingroup module_dllkcal
*/
//------------------------------------------------------------------------------
tOplkError dllkcal_clearAsyncBuffer(void)
{
    tOplkError  ret = kErrorOk;

    ret = instance_l.pTxNmtFuncs->pfnResetDataBlockQueue(instance_l.dllCalQueueTxNmt);
    if (ret != kErrorOk)
    {
        DEBUG_LVL_ERROR_TRACE("%s() Reset NMT Tx queue returned 0x%X\n", __func__, ret);
    }

    ret = instance_l.pTxGenFuncs->pfnResetDataBlockQueue(instance_l.dllCalQueueTxGen);
    if (ret != kErrorOk)
    {
        DEBUG_LVL_ERROR_TRACE("%s() Reset Generic Tx queue returned 0x%X\n", __func__, ret);
    }

#if defined(CONFIG_INCLUDE_VETH)
    ret = instance_l.pTxVethFuncs->pfnResetDataBlockQueue(instance_l.dllCalQueueTxVeth);
    if (ret != kErrorOk)
    {
        DEBUG_LVL_ERROR_TRACE("%s() Reset Virtual Ethernet Tx queue returned 0x%X\n", __func__, ret);
    }
#endif

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief Clear the asynchronous transmit queues

The function clears the asynchronous transmit queues.

\return The function returns a tOplkError error code.

\ingroup module_dllkcal
*/
//------------------------------------------------------------------------------
#if defined(CONFIG_INCLUDE_NMT_MN)
tOplkError dllkcal_clearAsyncQueues(void)
{
    tOplkError  ret = kErrorOk;

    ret = instance_l.pTxSyncFuncs->pfnResetDataBlockQueue(instance_l.dllCalQueueTxSync);
    if (ret != kErrorOk)
    {
        DEBUG_LVL_ERROR_TRACE("%s() Reset Sync Tx queue returned 0x%X\n", __func__, ret);
    }

    // clear MN asynchronous queues
    instance_l.nextRequestQueue = 0;

    circbuf_reset(instance_l.pQueueCnRequestGen);
    circbuf_reset(instance_l.pQueueCnRequestNmt);
    circbuf_reset(instance_l.pQueueIdentReq);
    circbuf_reset(instance_l.pQueueStatusReq);

    return ret;
}
#endif

//------------------------------------------------------------------------------
/**
\brief Get statistics of asynchronous queues

The function returns statistics of the asynchronous queues

\param[out]     ppStatistics        Pointer to store statistics pointer.

\return The function returns a tOplkError error code.

\ingroup module_dllkcal
*/
//------------------------------------------------------------------------------
tOplkError dllkcal_getStatistics(tDllkCalStatistics** ppStatistics)
{
    tOplkError  ret = kErrorOk;
    UINT        frameCount;
    UINT        frameCountVeth;

    ret = instance_l.pTxNmtFuncs->pfnGetDataBlockCount(instance_l.dllCalQueueTxNmt,
                                                       &instance_l.statistics.curTxFrameCountNmt);
    if (ret != kErrorOk)
    {
        DEBUG_LVL_ERROR_TRACE("%s() Get data count of NMT Tx queue returned 0x%X\n",
                              __func__,
                              ret);
    }

    ret = instance_l.pTxGenFuncs->pfnGetDataBlockCount(instance_l.dllCalQueueTxGen,
                                                       &frameCount);
    if (ret != kErrorOk)
    {
        DEBUG_LVL_ERROR_TRACE("%s() Get data count of Generic Tx queue returned 0x%X\n",
                              __func__,
                              ret);
    }

#if defined(CONFIG_INCLUDE_VETH)
    ret = instance_l.pTxVethFuncs->pfnGetDataBlockCount(instance_l.dllCalQueueTxVeth,
                                                        &frameCountVeth);
    if (ret != kErrorOk)
    {
        DEBUG_LVL_ERROR_TRACE("%s() Get data count of Virtual Ethernet Tx queue returned 0x%X\n",
                              __func__,
                              ret);
    }

    frameCount += frameCountVeth;
#else
    UNUSED_PARAMETER(frameCountVeth);
#endif
    instance_l.statistics.curTxFrameCountGen = frameCount;

    *ppStatistics = &instance_l.statistics;

    return ret;
}


#if defined(CONFIG_INCLUDE_NMT_MN)
//------------------------------------------------------------------------------
/**
\brief Issue a StatusRequest or IdentRequest

The function issues a StatusRequest or an IdentRequest to the specified node.

\param[in]      service_p           Service ID of request.
\param[in]      nodeId_p            Node ID to which the request should be sent.
\param[in]      soaFlag1_p          Flag1 for this node (transmit in SoA and PReq).
                                    If 0xFF, this flag is ignored.

\return The function returns a tOplkError error code.

\ingroup module_dllkcal
*/
//------------------------------------------------------------------------------
tOplkError dllkcal_issueRequest(tDllReqServiceId service_p,
                                UINT nodeId_p,
                                UINT8 soaFlag1_p)
{
    tOplkError      ret = kErrorOk;
    tCircBufError   err;

    if (soaFlag1_p != 0xFF)
    {
        ret = dllk_setFlag1OfNode(nodeId_p, soaFlag1_p);
        if (ret != kErrorOk)
            goto Exit;
    }

    // add node to appropriate request queue
    switch (service_p)
    {
        case kDllReqServiceIdent:
            err = circbuf_writeData(instance_l.pQueueIdentReq, &nodeId_p, sizeof(nodeId_p));
            if (err != kCircBufOk)
            {   // queue is full
                ret = kErrorDllAsyncTxBufferFull;
                goto Exit;
            }
            break;

        case kDllReqServiceStatus:
            err = circbuf_writeData(instance_l.pQueueStatusReq, &nodeId_p, sizeof(nodeId_p));
            if (err != kCircBufOk)
            {   // queue is full
                ret = kErrorDllAsyncTxBufferFull;
                goto Exit;
            }
            break;

        default:
            ret = kErrorDllInvalidParam;
            goto Exit;
    }

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief Return next request for SoA

The function returns the next request for SoA. It is called by the kernel
DLL module.

\param[out]     pReqServiceId_p     Pointer to the request service ID of available
                                    request for MN NMT or generic request queue
                                    (Flag2.PR) or kDllReqServiceNo if queues are
                                    empty. The function store the next request at
                                    this location.
\param[out]     pNodeId_p           Pointer to store the node ID of the next request.
                                    C_ADR_INVALID is stored if request is self
                                    addressed.
\param[out]     pSoaPayload_p       Pointer to SoA payload.

\return The function returns a tOplkError error code.

\ingroup module_dllkcal
*/
//------------------------------------------------------------------------------
tOplkError dllkcal_getSoaRequest(tDllReqServiceId* pReqServiceId_p,
                                 UINT* pNodeId_p,
                                 tSoaPayload* pSoaPayload_p)
{
    tOplkError  ret = kErrorOk;
    UINT        count;

#if ((CONFIG_DLL_DEFERRED_RXFRAME_RELEASE_ASYNC != FALSE) && defined(CONFIG_EDRV_ASND_DEFERRED_RX_BUFFERS))
    UINT        rxCount = instance_l.asyncFrameReceived - instance_l.asyncFrameFreed;

    // At the same time another frame could be waiting in the Rx queue of the MAC,
    // which is not yet handled by the software layers. This case is considered
    // by correcting the rxCount.
    if (rxCount + 1 >= CONFIG_EDRV_ASND_DEFERRED_RX_BUFFERS)
    {
        // Edrv has no more asynchronous Rx buffers, thus, do not assign the
        // next asynchronous phase to any node. Otherwise the MAC would drop
        // those asynchronous frames anyway.
        goto Exit;
    }
#endif

    for (count = DLLKCAL_MAX_QUEUES; count > 0; count--)
    {
        switch (instance_l.nextRequestQueue)
        {
            case 0:
                if (getCnGenRequest(pReqServiceId_p, pNodeId_p) != FALSE)
                    goto Exit;
                break;

            case 1:
                if (getCnNmtRequest(pReqServiceId_p, pNodeId_p) != FALSE)
                    goto Exit;
                break;

            case 2:
                if (getMnGenNmtRequest(pReqServiceId_p, pNodeId_p) != FALSE)
                    goto Exit;
                break;

            case 3:
                if (getMnIdentRequest(pReqServiceId_p, pNodeId_p) != FALSE)
                    goto Exit;
                break;

            case 4:
                if (getMnStatusRequest(pReqServiceId_p, pNodeId_p) != FALSE)
                    goto Exit;
                break;

            case 5:
                if (getMnSyncRequest(pReqServiceId_p, pNodeId_p, pSoaPayload_p) != FALSE)
                    goto Exit;
                break;
        }
    }

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief Set pending asynchronous request

The function sets the pending asynchronous frame request of the specified node.
This will add the node to the asynchronous request scheduler.

\param[in]      nodeId_p            Specifies the node to set the pending request.
\param[in]      asyncReqPrio_p      The asynchronous request priority.
\param[in]      count_p             The count of asynchronous frames.

\return The function returns a tOplkError error code.

\ingroup module_dllkcal
*/
//------------------------------------------------------------------------------
tOplkError dllkcal_setAsyncPendingRequests(UINT nodeId_p,
                                           tDllAsyncReqPriority asyncReqPrio_p,
                                           UINT count_p)
{
    tOplkError          ret = kErrorOk;
    tCircBufError       err;
    UINT*               pLocalRequestCnt;
    tCircBufInstance*   pTargetQueue;

    // get local request count for the node and the target queue
    switch (asyncReqPrio_p)
    {
        case kDllAsyncReqPrioNmt:
            pLocalRequestCnt = &instance_l.aCnRequestCntNmt[nodeId_p-1];
            pTargetQueue = instance_l.pQueueCnRequestNmt;
            break;

        default:
            pLocalRequestCnt = &instance_l.aCnRequestCntGen[nodeId_p-1];
            pTargetQueue = instance_l.pQueueCnRequestGen;
            break;
    }

    // compare the node request count with the locally stored one
    if (*pLocalRequestCnt < count_p)
    {
        // The node has added some requests, but post only one for fair
        // scheduling among the other nodes.
        err = circbuf_writeData(pTargetQueue, &nodeId_p, sizeof(nodeId_p));
        if (err == kCircBufOk)
            (*pLocalRequestCnt)++; // increment locally only by successful post
    }
    else
    {
        // the node's request count is equal or less the local one
        *pLocalRequestCnt = count_p;
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief Acknowledge a pending asynchronous request

The function acknowledges a pending asynchronous frame request of the specified
node.

\param[in]      nodeId_p            Specifies the node to acknowledge the request.
\param[in]      reqServiceId_p      The request service ID.

\return The function returns a tOplkError error code.

\ingroup module_dllkcal
*/
//------------------------------------------------------------------------------
tOplkError dllkcal_ackAsyncRequest(UINT nodeId_p, tDllReqServiceId reqServiceId_p)
{
    tOplkError  ret = kErrorOk;
    UINT*       pLocalRequestCnt;

    switch (reqServiceId_p)
    {
        case kDllReqServiceNmtRequest:
            pLocalRequestCnt = &instance_l.aCnRequestCntNmt[nodeId_p-1];
            break;

        case kDllReqServiceUnspecified:
            pLocalRequestCnt = &instance_l.aCnRequestCntGen[nodeId_p-1];
            break;

        default:
            // No need to handle other requests
            return ret;
    }

    if (*pLocalRequestCnt > 0)
        (*pLocalRequestCnt)--;

    return ret;
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
\brief Get CN Generic request

The function returns the next CN generic request.

\param[out]     pReqServiceId_p     Pointer to store the next request.
\param[out]     pNodeId_p           Pointer to store the node ID for the next
                                    request.

\return Returns whether a request was found
\retval TRUE                        A request was found
\retval FALSE                       No request was found
*/
//------------------------------------------------------------------------------
static BOOL getCnGenRequest(tDllReqServiceId* pReqServiceId_p, UINT* pNodeId_p)
{
    tCircBufError   err;
    UINT            rxNodeId;
    size_t          size = sizeof(rxNodeId);

    // next queue will be CnNmtReq queue
    instance_l.nextRequestQueue = 1;

    err = circbuf_readData(instance_l.pQueueCnRequestGen, &rxNodeId, size, &size);

    switch (err)
    {
        case kCircBufOk:
            if (instance_l.aCnRequestCntGen[rxNodeId-1] > 0)
            {
                *pNodeId_p = rxNodeId;
                *pReqServiceId_p = kDllReqServiceUnspecified;
                // dllkcal_ackAsyncRequest() will decrement the request count!

                return TRUE;
            }
            // fall-trough

        case kCircBufNoReadableData:
        default:
            // an empty or faulty queue has no requests
            return FALSE;
    }
}

//------------------------------------------------------------------------------
/**
\brief  Get CN NMT request

The function returns the next CN NMT request.

\param[out]     pReqServiceId_p     Pointer to store the next request.
\param[out]     pNodeId_p           Pointer to store the node ID for the next
                                    request.

\return Returns whether a request was found
\retval TRUE                        A request was found
\retval FALSE                       No request was found
*/
//------------------------------------------------------------------------------
static BOOL getCnNmtRequest(tDllReqServiceId* pReqServiceId_p, UINT* pNodeId_p)
{
    tCircBufError   err;
    UINT            rxNodeId;
    size_t          size = sizeof(rxNodeId);

    // next queue will be MnGenReq queue
    instance_l.nextRequestQueue = 2;

    err = circbuf_readData(instance_l.pQueueCnRequestNmt, &rxNodeId, size, &size);

    switch (err)
    {
        case kCircBufOk:
            if (instance_l.aCnRequestCntNmt[rxNodeId-1] > 0)
            {
                *pNodeId_p = rxNodeId;
                *pReqServiceId_p = kDllReqServiceNmtRequest;
                // dllkcal_ackAsyncRequest() will decrement the request count!

                return TRUE;
            }
            // fall-trough

        case kCircBufNoReadableData:
        default:
            // an empty or faulty queue has no requests
            return FALSE;
    }
}

//------------------------------------------------------------------------------
/**
\brief  Get MN Generic or NMT request

The function returns the next MN Generic/NMT request.

\param[out]     pReqServiceId_p     Pointer to store the next request.
\param[out]     pNodeId_p           Pointer to store the node ID for the next
                                    request.

\return Returns whether a request was found
\retval TRUE                        A request was found
\retval FALSE                       No request was found
*/
//------------------------------------------------------------------------------
static BOOL getMnGenNmtRequest(tDllReqServiceId* pReqServiceId_p, UINT* pNodeId_p)
{
    // MnNmtReq and MnGenReq
    // next queue will be MnIdentReq queue
    instance_l.nextRequestQueue = 3;
    if (*pReqServiceId_p != kDllReqServiceNo)
    {
        *pNodeId_p = C_ADR_INVALID;   // DLLk must exchange this with the actual node ID
        return TRUE;
    }

    return FALSE;
}

//------------------------------------------------------------------------------
/**
\brief  Get MN Ident request

The function returns the next MN ident request.

\param[out]     pReqServiceId_p     Pointer to store the next request.
\param[out]     pNodeId_p           Pointer to store the node ID for the next
                                    request.

\return Returns whether a request was found
\retval TRUE                        A request was found
\retval FALSE                       No request was found
*/
//------------------------------------------------------------------------------
static BOOL getMnIdentRequest(tDllReqServiceId* pReqServiceId_p, UINT* pNodeId_p)
{
    tCircBufError   err;
    UINT            rxNodeId;
    size_t          size = sizeof(rxNodeId);

    // next queue will be MnStatusReq queue
    instance_l.nextRequestQueue = 4;

    err = circbuf_readData(instance_l.pQueueIdentReq, &rxNodeId, size, &size);

    if (err == kCircBufOk)
    {   // queue is not empty
        *pNodeId_p = rxNodeId;
        *pReqServiceId_p = kDllReqServiceIdent;
        return TRUE;
    }

    return FALSE;
}

//------------------------------------------------------------------------------
/**
\brief  Get MN status request

The function returns the next MN status request.

\param[out]     pReqServiceId_p     Pointer to store the next request.
\param[out]     pNodeId_p           Pointer to store the node ID for the next
                                    request.

\return Returns whether a request was found
\retval TRUE                        A request was found
\retval FALSE                       No request was found
*/
//------------------------------------------------------------------------------
static BOOL getMnStatusRequest(tDllReqServiceId* pReqServiceId_p, UINT* pNodeId_p)
{
    tCircBufError   err;
    UINT            rxNodeId;
    size_t          size = sizeof(rxNodeId);

    // next queue will be MnSyncReq queue
    instance_l.nextRequestQueue = 5;
    err = circbuf_readData(instance_l.pQueueStatusReq, &rxNodeId, size, &size);

    if (err == kCircBufOk)
    {   // queue is not empty
        *pNodeId_p = rxNodeId;
        *pReqServiceId_p = kDllReqServiceStatus;
        return TRUE;
    }

    return FALSE;
}

//------------------------------------------------------------------------------
/**
\brief  Get MN Sync request

The function returns the next MN Sync request.

todo how to handle errors (ret != kErrorOk)? Is it sufficient that we
     return TRUE as when we successfully finish?

\param[out]     pReqServiceId_p     Pointer to store the next request.
\param[out]     pNodeId_p           Pointer to store the node ID for the next
                                    request.
\param[out]     pSoaPayload_p       Pointer to SoA payload.

\return Returns whether a request was found
\retval TRUE                        A request was found
\retval FALSE                       No request was found
*/
//------------------------------------------------------------------------------
static BOOL getMnSyncRequest(tDllReqServiceId* pReqServiceId_p,
                             UINT* pNodeId_p,
                             tSoaPayload* pSoaPayload_p)
{
    tOplkError          ret;
    UINT                syncReqCount = 0;
    size_t              syncReqSize = 0;
    tDllSyncRequest     syncRequest;
    tDllNodeOpParam     nodeOpParam;

    // next queue will be CnGenReq queue
    instance_l.nextRequestQueue = 0;
    ret = instance_l.pTxSyncFuncs->pfnGetDataBlockCount(instance_l.dllCalQueueTxSync,
                                                        &syncReqCount);
    if (ret != kErrorOk)
        return TRUE;

    if (syncReqCount > 0)
    {
        syncReqSize = sizeof(syncRequest);
        ret = instance_l.pTxSyncFuncs->pfnGetDataBlock(instance_l.dllCalQueueTxSync,
                                                       &syncRequest,
                                                       &syncReqSize);
        if (ret != kErrorOk)
            return TRUE;

        if (syncReqSize > offsetof(tDllSyncRequest, syncControl))
        {
            ami_setUint32Le(&pSoaPayload_p->syncRequest.syncControlLe,
                            syncRequest.syncControl);
            if ((syncRequest.syncControl & PLK_SYNC_PRES_MODE_SET) != 0)
            {
                nodeOpParam.opNodeType = kDllNodeOpTypeIsochronous;
                nodeOpParam.nodeId = syncRequest.nodeId;
                ret = dllk_addNode(&nodeOpParam);
                if (ret != kErrorOk)
                    return TRUE;
            }

            if ((syncRequest.syncControl & PLK_SYNC_PRES_MODE_RESET) != 0)
            {
                nodeOpParam.opNodeType = kDllNodeOpTypeIsochronous;
                nodeOpParam.nodeId = syncRequest.nodeId;
                ret = dllk_deleteNode(&nodeOpParam);
                if (ret != kErrorOk)
                    return TRUE;
            }
        }

        if (syncReqSize > offsetof(tDllSyncRequest, pResTimeFirst))
        {
            ami_setUint32Le(&pSoaPayload_p->syncRequest.presTimeFirstLe,
                            syncRequest.pResTimeFirst);
        }

        if (syncReqSize > offsetof(tDllSyncRequest, pResFallBackTimeout))
        {
            ami_setUint32Le(&pSoaPayload_p->syncRequest.presFallBackTimeoutLe,
                            syncRequest.pResFallBackTimeout);
        }

        if ((syncRequest.syncControl & PLK_SYNC_DEST_MAC_ADDRESS_VALID) != 0)
        {
            ret = dllk_getCnMacAddress(syncRequest.nodeId,
                                       &pSoaPayload_p->syncRequest.aDestMacAddress[0]);
            if (ret != kErrorOk)
                return TRUE;
        }

        *pNodeId_p = syncRequest.nodeId;
        *pReqServiceId_p = kDllReqServiceSync;
        return TRUE;
    }

    return FALSE;
}

#endif

//------------------------------------------------------------------------------
/**
\brief  Send asynchronous frame

This function sends an asynchronous frame with generic priority.
The EtherType of the given frame determines the queue to be used for queuing.
If the frame is a POWERLINK frame or EtherType is 0x0, the generic priority Tx
queue is used. Other frame types (e.g. IP) are forwarded with the virtual
Ethernet Tx queue.

\param[in]      pFrameInfo_p        Pointer to asynchronous frame. The frame size
                                    includes the Ethernet header (14 bytes).

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError sendGenericAsyncFrame(tFrameInfo* pFrameInfo_p)
{
    tOplkError  ret = kErrorOk;
    UINT16      etherType = ami_getUint16Be(&pFrameInfo_p->frame.pBuffer->etherType);

    if ((etherType == 0) || (etherType == C_DLL_ETHERTYPE_EPL))
    {
        ret = instance_l.pTxGenFuncs->pfnInsertDataBlock(instance_l.dllCalQueueTxGen,
                                                         pFrameInfo_p->frame.pBuffer,
                                                         (size_t)pFrameInfo_p->frameSize);
    }
    else
    {
#if defined(CONFIG_INCLUDE_VETH)
        ret = instance_l.pTxVethFuncs->pfnInsertDataBlock(instance_l.dllCalQueueTxVeth,
                                                          pFrameInfo_p->frame.pBuffer,
                                                          (size_t)pFrameInfo_p->frameSize);
#else
        // Return error since virtual Ethernet is not existing!
        ret = kErrorIllegalInstance;
#endif
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Get current asynchronous frame with generic priority

\param[out]     pFrame_p            Pointer to the asynchronous frame.
\param[out]     pFrameSize_p        Size of the asynchronous frame.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError getGenericAsyncFrame(void* pFrame_p, size_t* pFrameSize_p)
{
    tOplkError  ret = kErrorOk;
#if defined(CONFIG_INCLUDE_VETH)
    UINT        i;

    for (i = 0; i < kDllkCalTxQueueSelectLast; i++)
    {
        switch (instance_l.currentTxQueueSelect)
        {
            case kDllkCalTxQueueSelectGen:
                ret = instance_l.pTxGenFuncs->pfnGetDataBlock(instance_l.dllCalQueueTxGen,
                                                              pFrame_p,
                                                              pFrameSize_p);

                // Set current queue select to next queue
                instance_l.currentTxQueueSelect = kDllkCalTxQueueSelectVeth;
                break;

            case kDllkCalTxQueueSelectVeth:
                ret = instance_l.pTxVethFuncs->pfnGetDataBlock(instance_l.dllCalQueueTxVeth,
                                                               pFrame_p,
                                                               pFrameSize_p);

                // Set current queue select to next queue
                instance_l.currentTxQueueSelect = kDllkCalTxQueueSelectGen;
                break;

            default:
                DEBUG_LVL_ERROR_TRACE("%s current selected Tx queue %d invalid!\n",
                                      __func__,
                                      instance_l.currentTxQueueSelect);
                ret = kErrorDllInvalidParam;
                break;
        }

        // Break loop earlier if data is found or an error happens
        if (ret != kErrorDllAsyncTxBufferEmpty)
            break;
    }
#else
    ret = instance_l.pTxGenFuncs->pfnGetDataBlock(instance_l.dllCalQueueTxGen,
                                                  pFrame_p,
                                                  pFrameSize_p);
#endif

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Command translator function for NMT commands

The function translates NMT commands to the corresponding NMT events.

\param[in]      pNmtCommand_p       Pointer to the NMT command service info.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tNmtEvent commandTranslator(const tNmtCommandService* pNmtCommand_p)
{
    tNmtCommand nmtCommand;
    BOOL        fNodeIdInList;
    tNmtEvent   nmtEvent = kNmtEventNoEvent;

    if (pNmtCommand_p == NULL)
        return kErrorNmtInvalidFramePointer;

    nmtCommand = (tNmtCommand)ami_getUint8Le(&pNmtCommand_p->nmtCommandId);
    switch (nmtCommand)
    {
        //------------------------------------------------------------------------
        // plain NMT state commands
        case kNmtCmdStartNode:
            nmtEvent = kNmtEventStartNode;
            break;

        case kNmtCmdStopNode:
            nmtEvent = kNmtEventStopNode;
            break;

        case kNmtCmdEnterPreOperational2:
            nmtEvent = kNmtEventEnterPreOperational2;
            break;

        case kNmtCmdEnableReadyToOperate:
            nmtEvent = kNmtEventEnableReadyToOperate;
            break;

        case kNmtCmdResetNode:
            nmtEvent = kNmtEventResetNode;
            break;

        case kNmtCmdResetCommunication:
            nmtEvent = kNmtEventResetCom;
            break;

        case kNmtCmdResetConfiguration:
            nmtEvent = kNmtEventResetConfig;
            break;

        case kNmtCmdSwReset:
            nmtEvent = kNmtEventSwReset;
            break;

        //------------------------------------------------------------------------
        // extended NMT state commands
        case kNmtCmdStartNodeEx:
            // check if own nodeid is in the POWERLINK node list
            fNodeIdInList = checkNodeIdList(pNmtCommand_p);
            if (fNodeIdInList)
            {   // own nodeid in list
                // send event to process command
                nmtEvent = kNmtEventStartNode;
            }
            break;

        case kNmtCmdStopNodeEx:
            // check if own nodeid is in the POWERLINK node list
            fNodeIdInList = checkNodeIdList(pNmtCommand_p);
            if (fNodeIdInList)
            {   // own nodeid in list
                // send event to process command
                nmtEvent = kNmtEventStopNode;
            }
            break;

        case kNmtCmdEnterPreOperational2Ex:
            // check if own nodeid is in the POWERLINK node list
            fNodeIdInList = checkNodeIdList(pNmtCommand_p);
            if (fNodeIdInList)
            {   // own nodeid in list
                // send event to process command
                nmtEvent = kNmtEventEnterPreOperational2;
            }
            break;

        case kNmtCmdEnableReadyToOperateEx:
            // check if own nodeid is in the POWERLINK node list
            fNodeIdInList = checkNodeIdList(pNmtCommand_p);
            if (fNodeIdInList)
            {   // own nodeid in list
                // send event to process command
                nmtEvent = kNmtEventEnableReadyToOperate;
            }
            break;

        case kNmtCmdResetNodeEx:
            // check if own nodeid is in the POWERLINK node list
            fNodeIdInList = checkNodeIdList(pNmtCommand_p);
            if (fNodeIdInList)
            {   // own nodeid in list
                // send event to process command
                nmtEvent = kNmtEventResetNode;
            }
            break;

        case kNmtCmdResetCommunicationEx:
            // check if own nodeid is in the POWERLINK node list
            fNodeIdInList = checkNodeIdList(pNmtCommand_p);
            if (fNodeIdInList)
            {   // own nodeid in list
                // send event to process command
                nmtEvent = kNmtEventResetCom;
            }
            break;

        case kNmtCmdResetConfigurationEx:
            // check if own nodeid is in the POWERLINK node list
            fNodeIdInList = checkNodeIdList(pNmtCommand_p);
            if (fNodeIdInList)
            {   // own nodeid in list
                // send event to process command
                nmtEvent = kNmtEventResetConfig;
            }
            break;

        case kNmtCmdSwResetEx:
            // check if own nodeid is in the POWERLINK node list
            fNodeIdInList = checkNodeIdList(pNmtCommand_p);
            if (fNodeIdInList)
            {   // own nodeid in list
                // send event to process command
                nmtEvent = kNmtEventSwReset;
            }
            break;

        //------------------------------------------------------------------------
        // NMT managing commands
        // TODO: add functions to process managing command (optional)
        case kNmtCmdNetHostNameSet:
            break;

        case kNmtCmdFlushArpEntry:
            break;

        //------------------------------------------------------------------------
        // NMT info services
        // TODO: forward event with infos to the application (optional)
        case kNmtCmdPublishConfiguredCN:
            break;

        case kNmtCmdPublishActiveCN:
            break;

        case kNmtCmdPublishPreOperational1:
            break;

        case kNmtCmdPublishPreOperational2:
            break;

        case kNmtCmdPublishReadyToOperate:
            break;

        case kNmtCmdPublishOperational:
            break;

        case kNmtCmdPublishStopped:
            break;

        case kNmtCmdPublishEmergencyNew:
            break;

        case kNmtCmdPublishTime:
            break;

        //-----------------------------------------------------------------------
        // error from MN
        // -> requested command not supported by MN
        case kNmtCmdInvalidService:
            // TODO: error event to application
            break;

        //------------------------------------------------------------------------
        // default
        default:
            return kErrorNmtUnknownCommand;
    } // end of switch (nmtCommand)

    return nmtEvent;
}

//------------------------------------------------------------------------------
/**
\brief  Init local node Id

The function initializes an node Id.

\param[in]      nodeId_p            Local node ID.
*/
//------------------------------------------------------------------------------
static void initNodeInstance(UINT nodeId_p)
{
    instance_l.nodeInstance.nodeId = nodeId_p;

    // Byte offset --> nodeid divide by 8
    // Bit offset  --> 2 ^ (nodeid AND 0b111)
    instance_l.nodeInstance.extNmtCmdByteOffset = (UINT)(nodeId_p >> 3);
    instance_l.nodeInstance.extNmtCmdBitMask = 1 << ((UINT8)nodeId_p & 7);
}

//------------------------------------------------------------------------------
/**
\brief  Check node ID list

The function checks if the own node ID is set in the node list.

\param[in]      pNmtCommand_p       Pointer to the NMT command service info.

\return The function returns \b TRUE if the node is found in the node list or
        \b FALSE if it is not found in the node list.
*/
//------------------------------------------------------------------------------
static BOOL checkNodeIdList(const tNmtCommandService* pNmtCommand_p)
{
    BOOL    fNodeIdInList;
    UINT    byteOffset = instance_l.nodeInstance.extNmtCmdByteOffset;
    UINT8   bitMask = instance_l.nodeInstance.extNmtCmdBitMask;
    UINT8   nodeListByte = ami_getUint8Le(&pNmtCommand_p->aNmtCommandData[byteOffset]);

    if ((nodeListByte & bitMask) == 0)
        fNodeIdInList = FALSE;
    else
        fNodeIdInList = TRUE;

    return fNodeIdInList;
}

/// \}
