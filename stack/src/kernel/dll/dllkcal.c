/**
********************************************************************************
\file   dllkcal.c

\brief  Kernel DLL CAL module

This file contains the kernel DLL CAL module.

\ingroup module_dllkcal
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
#include <dllcal.h>
#include <kernel/dllkcal.h>
#include <kernel/dllk.h>

#include <kernel/eventk.h>

#ifdef CONFIG_INCLUDE_NMT_MN
#include <circbuffer.h>
#endif

#if (EPL_DLL_PRES_CHAINING_MN != FALSE) && (CONFIG_DLLCAL_QUEUE == EPL_QUEUE_DIRECT)
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
#if EPL_DLL_PRES_CHAINING_MN != FALSE
#define EPL_DLLKCAL_MAX_QUEUES  6   // CnGenReq, CnNmtReq, {MnGenReq, MnNmtReq}, MnIdentReq, MnStatusReq, SyncReq
#else
#define EPL_DLLKCAL_MAX_QUEUES  5   // CnGenReq, CnNmtReq, {MnGenReq, MnNmtReq}, MnIdentReq, MnStatusReq
#endif

//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------
typedef struct
{
    tDllCalQueueInstance    dllCalQueueTxNmt;       ///< Dll Cal Queue instance for NMT priority
    tDllCalQueueInstance    dllCalQueueTxGen;       ///< Dll Cal Queue instance for Generic priority
    tDllCalFuncIntf*        pTxNmtFuncs;
    tDllCalFuncIntf*        pTxGenFuncs;

#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0) \
    && (EPL_DLL_PRES_CHAINING_MN != FALSE)
    tDllCalQueueInstance    dllCalQueueTxSync;      ///< Dll Cal Queue instance for Sync Request
    tDllCalFuncIntf*        pTxSyncFuncs;
#endif
    tDllkCalStatistics      statistics;

#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)
    // IdentRequest queue with CN node IDs

    tCircBufInstance*       pQueueIdentReq;

    // StatusRequest queue with CN node IDs
    tCircBufInstance*       pQueueStatusReq;

    tCircBufInstance*       pQueueCnRequestNmt;
    UINT                    aCnRequestCntNmt[254];
    tCircBufInstance*       pQueueCnRequestGen;
    UINT                    aCnRequestCntGen[254];

    UINT                    nextRequestQueue;
#endif
} tDllkCalInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tDllkCalInstance     instance_l;


//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)
static BOOL getCnGenRequest(tDllReqServiceId* pReqServiceId_p, UINT* pNodeId_p);
static BOOL getCnNmtRequest(tDllReqServiceId* pReqServiceId_p, UINT* pNodeId_p);
static BOOL getMnGenNmtRequest(tDllReqServiceId* pReqServiceId_p, UINT* pNodeId_p);
static BOOL getMnIdentRequest(tDllReqServiceId* pReqServiceId_p, UINT* pNodeId_p);
static BOOL getMnStatusRequest(tDllReqServiceId* pReqServiceId_p, UINT* pNodeId_p);

#if (EPL_DLL_PRES_CHAINING_MN != FALSE)
static BOOL getMnSyncRequest(tDllReqServiceId* pReqServiceId_p, UINT* pNodeId_p,
                             tEplSoaPayload* pSoaPayload_p);
#endif
#endif

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief	Initialize kernel DLL CAL module

This function initializes the kernel DLL CAL module.

\return The function returns a tEplKernel error code.

\ingroup module_dllkcal
*/
//------------------------------------------------------------------------------
tEplKernel dllkcal_init(void)
{
    tEplKernel      ret = kEplSuccessful;
#ifdef CONFIG_INCLUDE_NMT_MN
    tCircBufError   circErr;
#endif

    // reset instance structure
    EPL_MEMSET(&instance_l, 0, sizeof (instance_l));

    instance_l.pTxNmtFuncs = GET_DLLKCAL_INTERFACE();
    instance_l.pTxGenFuncs = GET_DLLKCAL_INTERFACE();
#if EPL_DLL_PRES_CHAINING_MN != FALSE
    instance_l.pTxSyncFuncs = GET_DLLKCAL_INTERFACE();
#endif

    ret = instance_l.pTxNmtFuncs->pfnAddInstance(&instance_l.dllCalQueueTxNmt,
                                                 kDllCalQueueTxNmt);
    if(ret != kEplSuccessful)
    {
        EPL_DBGLVL_ERROR_TRACE ("%s() TxNmt failed\n", __func__);
        goto Exit;
    }

    ret = instance_l.pTxGenFuncs->pfnAddInstance(&instance_l.dllCalQueueTxGen,
                                                 kDllCalQueueTxGen);
    if(ret != kEplSuccessful)
    {
        EPL_DBGLVL_ERROR_TRACE("%s() TxGen failed\n", __func__);
        goto Exit;
    }

#if EPL_DLL_PRES_CHAINING_MN != FALSE
    ret = instance_l.pTxSyncFuncs->pfnAddInstance(&instance_l.dllCalQueueTxSync,
                                                  kDllCalQueueTxSync);
    if(ret != kEplSuccessful)
    {
        EPL_DBGLVL_ERROR_TRACE("%s() TxSync failed\n", __func__);
        goto Exit;
    }
#endif

#ifdef CONFIG_INCLUDE_NMT_MN
    circErr = circbuf_alloc(CIRCBUF_DLLCAL_CN_REQ_NMT, DLLCAL_SIZE_CIRCBUF_CN_REQ_NMT,
            &instance_l.pQueueCnRequestNmt);
    if(circErr != kCircBufOk)
    {
        EPL_DBGLVL_ERROR_TRACE("%s() Allocate CIRCBUF_ASYNC_SCHED_NMT failed\n", __func__);
        goto Exit;
    }

    circErr = circbuf_alloc(CIRCBUF_DLLCAL_CN_REQ_GEN, DLLCAL_SIZE_CIRCBUF_CN_REQ_GEN,
            &instance_l.pQueueCnRequestGen);
    if(circErr != kCircBufOk)
    {
        EPL_DBGLVL_ERROR_TRACE("%s() Allocate CIRCBUF_ASYNC_SCHED_GEN failed\n", __func__);
        goto Exit;
    }

    circErr = circbuf_alloc(CIRCBUF_DLLCAL_CN_REQ_IDENT, DLLCAL_SIZE_CIRCBUF_REQ_IDENT,
            &instance_l.pQueueIdentReq);
    if(circErr != kCircBufOk)
    {
        EPL_DBGLVL_ERROR_TRACE("%s() Allocate CIRCBUF_DLLCAL_CN_REQ_IDENT failed\n", __func__);
        goto Exit;
    }

    circErr = circbuf_alloc(CIRCBUF_DLLCAL_CN_REQ_STATUS, DLLCAL_SIZE_CIRCBUF_REQ_STATUS,
            &instance_l.pQueueStatusReq);
    if(circErr != kCircBufOk)
    {
        EPL_DBGLVL_ERROR_TRACE("%s() Allocate CIRCBUF_DLLCAL_CN_REQ_STATUS failed\n", __func__);
        goto Exit;
    }
#endif

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief	Cleanup the kernel DLL CAL module

This function cleans up the kernel DLL CAL module.

\return The function returns a tEplKernel error code.

\ingroup module_dllkcal
*/
//------------------------------------------------------------------------------
tEplKernel dllkcal_exit(void)
{
    tEplKernel      ret = kEplSuccessful;

#ifdef CONFIG_INCLUDE_NMT_MN
    circbuf_free(instance_l.pQueueCnRequestGen);
    circbuf_free(instance_l.pQueueCnRequestNmt);
    circbuf_free(instance_l.pQueueIdentReq);
    circbuf_free(instance_l.pQueueStatusReq);
#endif

    instance_l.pTxNmtFuncs->pfnDelInstance(instance_l.dllCalQueueTxNmt);
    instance_l.pTxGenFuncs->pfnDelInstance(instance_l.dllCalQueueTxGen);
#if EPL_DLL_PRES_CHAINING_MN != FALSE
    instance_l.pTxSyncFuncs->pfnDelInstance(instance_l.dllCalQueueTxSync);
#endif

    // reset instance structure
    EPL_MEMSET(&instance_l, 0, sizeof (instance_l));

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief	Process events

This function is the event handler of the kernel DLL CAL module.

\param  pEvent_p                Pointer to event to be processed.

\return The function returns a tEplKernel error code.

\ingroup module_dllkcal
*/
//------------------------------------------------------------------------------
tEplKernel dllkcal_process(tEplEvent* pEvent_p)
{
    tEplKernel                  ret = kEplSuccessful;
    tDllCalAsndServiceIdFilter* pServFilter;
    tDllIdentParam*             pIdentParam;
    tDllConfigParam*            pConfigParam;

#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)
    tDllCalIssueRequest*        pIssueReq;
#endif

#if EPL_NMT_MAX_NODE_ID > 0
    tDllNodeInfo*               pNodeInfo;
    tDllNodeOpParam*            pNodeOpParam;
#endif

    switch (pEvent_p->m_EventType)
    {
        case kEplEventTypeDllkServFilter:
            pServFilter = (tDllCalAsndServiceIdFilter*) pEvent_p->m_pArg;
            ret = dllk_setAsndServiceIdFilter(pServFilter->serviceId,
                                                pServFilter->filter);
            break;

#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)
        case kEplEventTypeDllkIssueReq:
            pIssueReq = (tDllCalIssueRequest*) pEvent_p->m_pArg;
            ret = dllkcal_issueRequest(pIssueReq->service, pIssueReq->nodeId,
                                       pIssueReq->soaFlag1);
            break;
#endif

#if EPL_NMT_MAX_NODE_ID > 0
        case kEplEventTypeDllkConfigNode:
            pNodeInfo = (tDllNodeInfo*) pEvent_p->m_pArg;
            ret = dllk_configNode(pNodeInfo);
            break;

        case kEplEventTypeDllkAddNode:
            pNodeOpParam = (tDllNodeOpParam*) pEvent_p->m_pArg;
            ret = dllk_addNode(pNodeOpParam);
            break;

        case kEplEventTypeDllkDelNode:
            pNodeOpParam = (tDllNodeOpParam*) pEvent_p->m_pArg;
            ret = dllk_deleteNode(pNodeOpParam);
            break;
#endif // EPL_NMT_MAX_NODE_ID > 0

        case kEplEventTypeDllkIdentity:
            pIdentParam = (tDllIdentParam*) pEvent_p->m_pArg;
            if (pIdentParam->sizeOfStruct > pEvent_p->m_uiSize)
            {
                pIdentParam->sizeOfStruct = pEvent_p->m_uiSize;
            }
            ret = dllk_setIdentity(pIdentParam);
            break;

        case kEplEventTypeDllkConfig:
            pConfigParam = (tDllConfigParam*) pEvent_p->m_pArg;
            if (pConfigParam->sizeOfStruct > pEvent_p->m_uiSize)
            {
                pConfigParam->sizeOfStruct = pEvent_p->m_uiSize;
            }
            ret = dllk_config(pConfigParam);
            break;

        default:
            ret = kEplInvalidEvent;
            break;
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief	Get count of TX frames

This function returns the count of TX frames of the FIFO with highest priority.

\param  pPriority_p				Pointer to store the FIFO type.
\param  pCount_p                Pointer to store the number of TX frames.

\return The function returns a tEplKernel error code.

\ingroup module_dllkcal
*/
//------------------------------------------------------------------------------
tEplKernel dllkcal_getAsyncTxCount(tDllAsyncReqPriority* pPriority_p,
                                   UINT* pCount_p)
{
    tEplKernel  ret = kEplSuccessful;
    ULONG       frameCount;

    ret = instance_l.pTxNmtFuncs->pfnGetDataBlockCount(instance_l.dllCalQueueTxNmt,
                                                       &frameCount);
    if(ret != kEplSuccessful)
    {
        goto Exit;
    }

    if (frameCount > instance_l.statistics.maxTxFrameCountNmt)
    {
        instance_l.statistics.maxTxFrameCountNmt = frameCount;
    }

    if (frameCount != 0)
    {   // NMT requests are in queue
        *pPriority_p = kDllAsyncReqPrioNmt;
        *pCount_p = (UINT) frameCount;
        goto Exit;
    }

    ret = instance_l.pTxGenFuncs->pfnGetDataBlockCount(instance_l.dllCalQueueTxGen,
                                                       &frameCount);
    if(ret != kEplSuccessful)
    {
        goto Exit;
    }

    if (frameCount > instance_l.statistics.maxTxFrameCountGen)
    {
        instance_l.statistics.maxTxFrameCountGen = frameCount;
    }

    *pPriority_p = kDllAsyncReqPrioGeneric;
    *pCount_p = (UINT) frameCount;

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief	Get TX frame of specified FIFO

The function return TX frames form the specified FIFO.

\param  pFrame_p                Pointer to store TX frame.
\param  pFrameSize_p            Pointer to maximum size of buffer. Will be
                                rewritten with actual size of frame.
\param  priority_p              Priority of the FIFO.

\return The function returns a tEplKernel error code.

\ingroup module_dllkcal
*/
//------------------------------------------------------------------------------
tEplKernel dllkcal_getAsyncTxFrame(void* pFrame_p, UINT* pFrameSize_p,
                                   tDllAsyncReqPriority priority_p)
{
    tEplKernel      ret = kEplSuccessful;

    switch (priority_p)
    {
        case kDllAsyncReqPrioNmt:    // NMT request priority
            ret = instance_l.pTxNmtFuncs->pfnGetDataBlock(
                                            instance_l.dllCalQueueTxNmt,
                                            (BYTE*) pFrame_p, pFrameSize_p);
            break;

        default:    // generic priority
            ret = instance_l.pTxGenFuncs->pfnGetDataBlock(
                                            instance_l.dllCalQueueTxGen,
                                            (BYTE*) pFrame_p, pFrameSize_p);
            break;
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief	Pass received ASnd frame to receive FIFO

The function passes a received ASnd frame to the receive FIFO. It will be called
only for frames with registered AsndServiceIds.

\param  pFrameInfo_p            Pointer to frame info of received frame

\return The function returns a tEplKernel error code.

\ingroup module_dllkcal
*/
//------------------------------------------------------------------------------
tEplKernel dllkcal_asyncFrameReceived(tFrameInfo* pFrameInfo_p)
{
    tEplKernel  ret = kEplSuccessful;
    tEplEvent   event;

    event.m_EventSink = kEplEventSinkDlluCal;
    event.m_EventType = kEplEventTypeAsndRx;
    event.m_pArg = pFrameInfo_p->pFrame;
    event.m_uiSize = pFrameInfo_p->frameSize;

    ret = eventk_postEvent(&event);
    if (ret != kEplSuccessful)
    {
        instance_l.statistics.curRxFrameCount++;
    }
    else
    {
        instance_l.statistics.maxRxFrameCount++;
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Send an asynchronous frame

The function puts the given frame into the transmit queue with the specified
priority.

\param  pFrameInfo_p            Pointer to frame info structure
\param  priority_p              Priority to send frame with

\return The function returns a tEplKernel error code.

\ingroup module_dllkcal
*/
//------------------------------------------------------------------------------
tEplKernel dllkcal_sendAsyncFrame(tFrameInfo* pFrameInfo_p,
                                  tDllAsyncReqPriority priority_p)
{
    tEplKernel  ret = kEplSuccessful;
    tEplEvent   event;

    switch (priority_p)
    {
        case kDllAsyncReqPrioNmt:    // NMT request priority
            ret = instance_l.pTxNmtFuncs->pfnInsertDataBlock(
                                        instance_l.dllCalQueueTxNmt,
                                        (BYTE*)pFrameInfo_p->pFrame,
                                        &(pFrameInfo_p->frameSize));
            break;

        default:    // generic priority
            ret = instance_l.pTxGenFuncs->pfnInsertDataBlock(
                                        instance_l.dllCalQueueTxGen,
                                        (BYTE*)pFrameInfo_p->pFrame,
                                        &(pFrameInfo_p->frameSize));
            break;
    }

    if(ret != kEplSuccessful)
    {
        goto Exit;
    }

    // post event to DLL
    event.m_EventSink = kEplEventSinkDllk;
    event.m_EventType = kEplEventTypeDllkFillTx;
    EPL_MEMSET(&event.m_NetTime, 0x00, sizeof(event.m_NetTime));
    event.m_pArg = &priority_p;
    event.m_uiSize = sizeof(priority_p);
    ret = eventk_postEvent(&event);

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Write an asynchronous frame into buffer

The function writes the given frame into the specified dll CAL queue.

\param  pFrameInfo_p            Pointer to frame info structure
\param  dllQueue                DllCal Queue to use

\return The function returns a tEplKernel error code.

\ingroup module_dllkcal
*/
//------------------------------------------------------------------------------
tEplKernel dllkcal_writeAsyncFrame(tFrameInfo* pFrameInfo_p, tDllCalQueue dllQueue)
{
    tEplKernel  ret = kEplSuccessful;

    switch (dllQueue)
    {
        case kDllCalQueueTxNmt:    // NMT request priority
            ret = instance_l.pTxNmtFuncs->pfnInsertDataBlock(
                                        instance_l.dllCalQueueTxNmt,
                                        (BYTE*)pFrameInfo_p->pFrame,
                                        &(pFrameInfo_p->frameSize));
            break;

        case kDllCalQueueTxGen:    // generic priority
            ret = instance_l.pTxGenFuncs->pfnInsertDataBlock(
                                        instance_l.dllCalQueueTxGen,
                                        (BYTE*)pFrameInfo_p->pFrame,
                                        &(pFrameInfo_p->frameSize));
            break;
#if EPL_DLL_PRES_CHAINING_MN != FALSE
        case kDllCalQueueTxSync:   // sync request priority
            ret = instance_l.pTxGenFuncs->pfnInsertDataBlock(
                                        instance_l.dllCalQueueTxSync,
                                        (BYTE*)pFrameInfo_p->pFrame,
                                        &(pFrameInfo_p->frameSize));
            break;
#endif
        default:
            break;
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief	Clear the asynchronous transmit buffer

The function clear the asynchronous transmit buffers.

\return The function returns a tEplKernel error code.

\ingroup module_dllkcal
*/
//------------------------------------------------------------------------------
tEplKernel dllkcal_clearAsyncBuffer(void)
{
    tEplKernel  ret = kEplSuccessful;

    //ret is ignored
    ret = instance_l.pTxNmtFuncs->pfnResetDataBlockQueue(
                                    instance_l.dllCalQueueTxNmt, 1000);

    //ret is ignored
    ret = instance_l.pTxGenFuncs->pfnResetDataBlockQueue(
                                    instance_l.dllCalQueueTxGen, 1000);
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief	Clear the asynchronous transmit queues

The function clears the asynchronous transmit queues.

\return The function returns a tEplKernel error code.

\ingroup module_dllkcal
*/
//------------------------------------------------------------------------------
#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)
tEplKernel dllkcal_clearAsyncQueues(void)
{
    tEplKernel  ret = kEplSuccessful;

#if EPL_DLL_PRES_CHAINING_MN != FALSE
    //ret is ignored
    ret = instance_l.pTxSyncFuncs->pfnResetDataBlockQueue(
                                    instance_l.dllCalQueueTxSync, 1000);
#endif

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
\brief	Get statistics of asynchronous queues

The function returns statistics of the asynchronous queues

\param  ppStatistics		    Pointer to store statistics pointer.

\return The function returns a tEplKernel error code.

\ingroup module_dllkcal
*/
//------------------------------------------------------------------------------
tEplKernel dllkcal_getStatistics(tDllkCalStatistics** ppStatistics)
{
    tEplKernel  ret = kEplSuccessful;

    //ret is ignored
    ret = instance_l.pTxNmtFuncs->pfnGetDataBlockCount(instance_l.dllCalQueueTxNmt,
                                     &instance_l.statistics.curTxFrameCountNmt);

    //ret is ignored
    ret = instance_l.pTxGenFuncs->pfnGetDataBlockCount(instance_l.dllCalQueueTxGen,
                                     &instance_l.statistics.curTxFrameCountGen);

    *ppStatistics = &instance_l.statistics;
    return ret;
}


#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)
//------------------------------------------------------------------------------
/**
\brief	Issue a StatusRequest or IdentRequest

The function issues a StatusRequest or an IdentRequest to the specified node.

\param  service_p               Service ID of request.
\param  nodeId_p                Node ID to which the request should be sent.
\param  soaFlag1_p              Flag1 for this node (transmit in SoA and PReq).
                                If 0xFF this flag is ignored.

\return The function returns a tEplKernel error code.

\ingroup module_dllkcal
*/
//------------------------------------------------------------------------------
tEplKernel dllkcal_issueRequest(tDllReqServiceId service_p, UINT nodeId_p,
                                  BYTE soaFlag1_p)
{
    tEplKernel      ret = kEplSuccessful;
    tCircBufError   err;

    if (soaFlag1_p != 0xFF)
    {
        ret = dllk_setFlag1OfNode(nodeId_p, soaFlag1_p);
        if (ret != kEplSuccessful)
        {
            goto Exit;
        }
    }

    // add node to appropriate request queue
    switch (service_p)
    {
        case kDllReqServiceIdent:
            err = circbuf_writeData(instance_l.pQueueIdentReq, &nodeId_p, sizeof(nodeId_p));
            if(err != kCircBufOk)
            {   // queue is full
                ret = kEplDllAsyncTxBufferFull;
                goto Exit;
            }
            break;

        case kDllReqServiceStatus:
            err = circbuf_writeData(instance_l.pQueueStatusReq, &nodeId_p, sizeof(nodeId_p));
            if(err != kCircBufOk)
            {   // queue is full
                ret = kEplDllAsyncTxBufferFull;
                goto Exit;
            }
            break;

        default:
            ret = kEplDllInvalidParam;
            goto Exit;
    }

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief	Return next request for SoA

The function returns the next request for SoA. It is called by the kernel
DLL module.

\param  pReqServiceId_p         Pointer to the request service ID of available
                                request for MN NMT or generic request queue
                                (Flag2.PR) or kDllReqServiceNo if queues are
                                emptry. The function store the next request at
                                this location.
\param  pNodeId_p               Pointer to store the node ID of the next request.
                                EPL_C_ADR_INVALID is stored if request is self
                                addressed.
\param  pSoaPayload_p           Pointer to SoA payload.
\param  PARAMETER

\return The function returns a tEplKernel error code.

\ingroup module_dllkcal
*/
//------------------------------------------------------------------------------
tEplKernel dllkcal_getSoaRequest(tDllReqServiceId* pReqServiceId_p,
                                 UINT* pNodeId_p, tEplSoaPayload* pSoaPayload_p)
{
    tEplKernel      ret = kEplSuccessful;
    UINT            count;

#if EPL_DLL_PRES_CHAINING_MN == FALSE
    UNUSED_PARAMETER(pSoaPayload_p);
#endif

    for (count = EPL_DLLKCAL_MAX_QUEUES; count > 0; count--)
    {
        switch (instance_l.nextRequestQueue)
        {
            case 0:
                if (getCnGenRequest(pReqServiceId_p, pNodeId_p) == TRUE)
                    goto Exit;
                break;

            case 1:
                if (getCnNmtRequest(pReqServiceId_p, pNodeId_p) == TRUE)
                    goto Exit;
                break;

            case 2:
                if (getMnGenNmtRequest(pReqServiceId_p, pNodeId_p) == TRUE)
                    goto Exit;
                break;

            case 3:
                if (getMnIdentRequest(pReqServiceId_p, pNodeId_p) == TRUE)
                    goto Exit;
                break;

            case 4:
                if (getMnStatusRequest(pReqServiceId_p, pNodeId_p) == TRUE)
                    goto Exit;
                break;

#if EPL_DLL_PRES_CHAINING_MN != FALSE
            case 5:
                if (getMnSyncRequest(pReqServiceId_p, pNodeId_p, pSoaPayload_p)
                                == TRUE)
                    goto Exit;
                break;
#endif
        }
    }

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief	Set pending asynchronous request

The function sets the pending asynchronous frame request of the specified node.
This will add the node to the asynchronous request scheduler.

\param  nodeId_p                Specifies the node to set the pending request.
\param  asyncReqPrio_p          The asynchronous request priority.
\param  count_p                 The count of asynchronous frames.

\return The function returns a tEplKernel error code.

\ingroup module_dllkcal
*/
//------------------------------------------------------------------------------
tEplKernel dllkcal_setAsyncPendingRequests(UINT nodeId_p,
                                           tDllAsyncReqPriority asyncReqPrio_p,
                                           UINT count_p)
{
    tEplKernel          ret = kEplSuccessful;
    tCircBufError       err;
    UINT*               pLocalRequestCnt;
    tCircBufInstance*   pTargetQueue;

    // get local request count for the node and the target queue
    switch(asyncReqPrio_p)
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
    if(*pLocalRequestCnt < count_p)
    {
        // The node has added some requests, but post only one for fair
        // scheduling among the other nodes.
        err = circbuf_writeData(pTargetQueue, &nodeId_p, sizeof(nodeId_p));
        if(err == kCircBufOk)
            (*pLocalRequestCnt)++; // increment locally only by successful post
    }
    else
    {
        // the node's request count is equal or less the local one
        *pLocalRequestCnt = count_p;
    }

    return ret;
}
#endif //(((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//

#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)
//------------------------------------------------------------------------------
/**
\brief	Get CN Generic request

The function returns the next CN generic request.

\param  pReqServiceId_p			Pointer to store the next request.
\param  pNodeId_p               Pointer to store the node ID for the next
                                request.

\return Returns if a request was found
\retval TRUE        A request was found
\retval FALSE       No request was found
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

    switch(err)
    {
        case kCircBufOk:
            if(instance_l.aCnRequestCntGen[rxNodeId-1] > 0)
            {
                *pNodeId_p = rxNodeId;
                *pReqServiceId_p = kDllReqServiceUnspecified;

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

\param  pReqServiceId_p         Pointer to store the next request.
\param  pNodeId_p               Pointer to store the node ID for the next
                                request.

\return Returns if a request was found
\retval TRUE        A request was found
\retval FALSE       No request was found
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

    switch(err)
    {
        case kCircBufOk:
            if(instance_l.aCnRequestCntNmt[rxNodeId-1] > 0)
            {
                *pNodeId_p = rxNodeId;
                *pReqServiceId_p = kDllReqServiceNmtRequest;

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

\param  pReqServiceId_p         Pointer to store the next request.
\param  pNodeId_p               Pointer to store the node ID for the next
                                request.

\return Returns if a request was found
\retval TRUE        A request was found
\retval FALSE       No request was found
*/
//------------------------------------------------------------------------------
static BOOL getMnGenNmtRequest(tDllReqServiceId* pReqServiceId_p, UINT* pNodeId_p)
{
    // MnNmtReq and MnGenReq
    // next queue will be MnIdentReq queue
    instance_l.nextRequestQueue = 3;
    if (*pReqServiceId_p != kDllReqServiceNo)
    {
        *pNodeId_p = EPL_C_ADR_INVALID;   // DLLk must exchange this with the actual node ID
        return TRUE;
    }
    return FALSE;
}

//------------------------------------------------------------------------------
/**
\brief  Get MN Ident request

The function returns the next MN ident request.

\param  pReqServiceId_p         Pointer to store the next request.
\param  pNodeId_p               Pointer to store the node ID for the next
                                request.

\return Returns if a request was found
\retval TRUE        A request was found
\retval FALSE       No request was found
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

    if(err == kCircBufOk)
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

\param  pReqServiceId_p         Pointer to store the next request.
\param  pNodeId_p               Pointer to store the node ID for the next
                                request.

\return Returns if a request was found
\retval TRUE        A request was found
\retval FALSE       No request was found
*/
//------------------------------------------------------------------------------
static BOOL getMnStatusRequest(tDllReqServiceId* pReqServiceId_p, UINT* pNodeId_p)
{
    tCircBufError   err;
    UINT            rxNodeId;
    size_t          size = sizeof(rxNodeId);
#if EPL_DLL_PRES_CHAINING_MN != FALSE
    // next queue will be MnSyncReq queue
    instance_l.nextRequestQueue = 5;
#else
    // next queue will be CnGenReq queue
    instance_l.nextRequestQueue = 0;
#endif
    err = circbuf_readData(instance_l.pQueueStatusReq, &rxNodeId, size, &size);

    if(err == kCircBufOk)
    {   // queue is not empty
        *pNodeId_p = rxNodeId;
        *pReqServiceId_p = kDllReqServiceStatus;
        return TRUE;
    }
    return FALSE;
}



#if EPL_DLL_PRES_CHAINING_MN != FALSE
//------------------------------------------------------------------------------
/**
\brief  Get MN Sync request

The function returns the next MN Sync request.

todo how to handle errors (ret != kEplSuccessful)? Is it sufficient that we
     return TRUE as when we successfully finish?

\param  pReqServiceId_p         Pointer to store the next request.
\param  pNodeId_p               Pointer to store the node ID for the next
                                request.
\param  pSoaPayload_p           Pointer to SoA payload.

\return Returns if a request was found
\retval TRUE        A request was found
\retval FALSE       No request was found
*/
//------------------------------------------------------------------------------
static BOOL getMnSyncRequest(tDllReqServiceId* pReqServiceId_p, UINT* pNodeId_p,
                             tEplSoaPayload* pSoaPayload_p)
{
    tEplKernel          ret;
    ULONG               syncReqCount = 0;
    UINT                syncReqSize = 0;
    tDllSyncRequest     syncRequest;
    tDllNodeOpParam     nodeOpParam;

    // next queue will be CnGenReq queue
    instance_l.nextRequestQueue = 0;
    ret = instance_l.pTxSyncFuncs->pfnGetDataBlockCount(
                                instance_l.dllCalQueueTxSync, &syncReqCount);
    if(ret != kEplSuccessful)
    {
        return TRUE;
    }
    if (syncReqCount > 0)
    {
        syncReqSize = sizeof(syncRequest);
        ret = instance_l.pTxSyncFuncs->pfnGetDataBlock(
                               instance_l.dllCalQueueTxSync,
                               (BYTE *)&syncRequest, &syncReqSize);
        if(ret != kEplSuccessful)
        {
            return TRUE;
        }

        if (syncReqSize > memberoffs(tDllSyncRequest, syncControl))
        {
            AmiSetDwordToLe(&pSoaPayload_p->m_SyncRequest.m_le_dwSyncControl,
                            syncRequest.syncControl);
            if ((syncRequest.syncControl & EPL_SYNC_PRES_MODE_SET) != 0)
            {
                nodeOpParam.opNodeType = kDllNodeOpTypeIsochronous;
                nodeOpParam.nodeId = syncRequest.nodeId;
                ret = dllk_addNode(&nodeOpParam);
                if (ret != kEplSuccessful)
                {
                    return TRUE;
                }
            }
            if ((syncRequest.syncControl & EPL_SYNC_PRES_MODE_RESET) != 0)
            {
                nodeOpParam.opNodeType = kDllNodeOpTypeIsochronous;
                nodeOpParam.nodeId = syncRequest.nodeId;
                ret = dllk_deleteNode(&nodeOpParam);
                if (ret != kEplSuccessful)
                {
                    return TRUE;
                }
            }
        }
        if (syncReqSize > memberoffs(tDllSyncRequest, pResTimeFirst))
        {
            AmiSetDwordToLe(&pSoaPayload_p->m_SyncRequest.m_le_dwPResTimeFirst,
                            syncRequest.pResTimeFirst);
        }
        if (syncReqSize > memberoffs(tDllSyncRequest, pResFallBackTimeout))
        {
            AmiSetDwordToLe(&pSoaPayload_p->m_SyncRequest.m_le_dwPResFallBackTimeout,
                            syncRequest.pResFallBackTimeout);
        }

        if ((syncRequest.syncControl & EPL_SYNC_DEST_MAC_ADDRESS_VALID) != 0)
        {
            ret = dllk_getCnMacAddress(syncRequest.nodeId,
                            &pSoaPayload_p->m_SyncRequest.m_be_abDestMacAddress[0]);
            if (ret != kEplSuccessful)
            {
                return TRUE;
            }
        }

        *pNodeId_p = syncRequest.nodeId;
        *pReqServiceId_p = kDllReqServiceSync;
        return TRUE;
    }
    return FALSE;
}
#endif

#endif //(((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)
