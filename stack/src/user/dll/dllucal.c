/**
********************************************************************************
\file   dllucal.c

\brief  User DLL CAL module

This file contains the user DLL CAL module.

\ingroup module_dllucal
*******************************************************************************/
/*------------------------------------------------------------------------------
Copyright (c) 2013, SYSTEC electronic GmbH
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
#include <oplk/ami.h>
#include <common/dllcal.h>
#include <user/dllucal.h>
#include <user/eventu.h>

//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#if defined(CONFIG_INCLUDE_NMT_MN) && (CONFIG_DLLCAL_QUEUE == DIRECT_QUEUE)
#error "DLLCal module does not support direct calls with PRC MN"
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
#define DLLUCAL_NOTRX_FRAME_SIZE    18  ///< Size of not received frame dummy

//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------
typedef struct
{
    tEplDlluCbAsnd           apfnDlluCbAsnd[DLL_MAX_ASND_SERVICE_ID];
    tDllCalQueueInstance     dllCalQueueTxNmt;          ///< Dll Cal Queue instance for NMT priority
    tDllCalQueueInstance     dllCalQueueTxGen;          ///< Dll Cal Queue instance for Generic priority
#if defined(CONFIG_INCLUDE_NMT_MN)
    tDllCalQueueInstance     dllCalQueueTxSync;         ///< Dll Cal Queue instance for Sync Request
    tDllCalFuncIntf*         pTxSyncFuncs;
#endif
    tDllCalFuncIntf*         pTxNmtFuncs;
    tDllCalFuncIntf*         pTxGenFuncs;
} tDlluCalInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
// if no dynamic memory allocation shall be used
// define structures statically
static tDlluCalInstance     instance_l;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------

static tOplkError SetAsndServiceIdFilter(tDllAsndServiceId ServiceId_p,
                                         tDllAsndFilter Filter_p);
static tOplkError HandleRxAsndFrame(tFrameInfo* pFrameInfo_p);
static tOplkError HandleNotRxAsndFrame(tDllAsndNotRx* pAsndNotRx_p);

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Initialize User DLL CAL module

This function initializes the user DLL CAL module.

\return The function returns a tOplkError error code.

\ingroup module_dllucal
*/
//------------------------------------------------------------------------------
tOplkError dllucal_init(void)
{
    tOplkError      ret = kErrorOk;

    // reset instance structure
    OPLK_MEMSET(&instance_l, 0, sizeof(instance_l));

    instance_l.pTxNmtFuncs = GET_DLLUCAL_INTERFACE();
    instance_l.pTxGenFuncs = GET_DLLUCAL_INTERFACE();
#if defined(CONFIG_INCLUDE_NMT_MN)
    instance_l.pTxSyncFuncs = GET_DLLUCAL_INTERFACE();
#endif

    ret = instance_l.pTxNmtFuncs->pfnAddInstance(&instance_l.dllCalQueueTxNmt,
                                                 kDllCalQueueTxNmt);
    if (ret != kErrorOk)
    {
        goto Exit;
    }

    ret = instance_l.pTxGenFuncs->pfnAddInstance(&instance_l.dllCalQueueTxGen,
                                                 kDllCalQueueTxGen);
    if (ret != kErrorOk)
    {
        goto Exit;
    }

#if defined(CONFIG_INCLUDE_NMT_MN)
    ret = instance_l.pTxSyncFuncs->pfnAddInstance(&instance_l.dllCalQueueTxSync,
                                                  kDllCalQueueTxSync);
    if (ret != kErrorOk)
    {
        goto Exit;
    }
#endif

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Clean up user DLL CAL module

This function cleans up the user DLL CAL module

\return The function returns a tOplkError error code.

\ingroup module_dllucal
*/
//------------------------------------------------------------------------------
tOplkError dllucal_exit(void)
{
    tOplkError      ret = kErrorOk;

    instance_l.pTxNmtFuncs->pfnDelInstance(instance_l.dllCalQueueTxNmt);
    instance_l.pTxGenFuncs->pfnDelInstance(instance_l.dllCalQueueTxGen);
#if defined(CONFIG_INCLUDE_NMT_MN)
    instance_l.pTxSyncFuncs->pfnDelInstance(instance_l.dllCalQueueTxSync);
#endif
    // reset instance structure
    OPLK_MEMSET(&instance_l, 0, sizeof(instance_l));

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Process asynchronous frame event

The function processes an asynchronous frame event

\param  pEvent_p               Event to process

\return The function returns a tOplkError error code.

\ingroup module_dllucal
*/
//------------------------------------------------------------------------------
tOplkError dllucal_process(tEvent* pEvent_p)
{
    tOplkError      ret = kErrorOk;
    tFrameInfo*     pFrameInfo = NULL;
    tDllAsndNotRx*  pAsndNotRx = NULL;
#if CONFIG_DLL_DEFERRED_RXFRAME_RELEASE_ASYNC == FALSE
    tFrameInfo      FrameInfo;
#endif

    switch (pEvent_p->eventType)
    {
        case kEventTypeAsndRx:
#if CONFIG_DLL_DEFERRED_RXFRAME_RELEASE_ASYNC == FALSE
            FrameInfo.pFrame = (tPlkFrame*)pEvent_p->pEventArg;
            FrameInfo.frameSize = pEvent_p->eventArgSize;
            pFrameInfo = &FrameInfo;
#else
            pFrameInfo = (tFrameInfo*)pEvent_p->pEventArg;
#endif
            ret = HandleRxAsndFrame(pFrameInfo);
            break;

        case kEventTypeAsndNotRx:
            pAsndNotRx = (tDllAsndNotRx*)pEvent_p->pEventArg;
            ret = HandleNotRxAsndFrame(pAsndNotRx);
            break;

        default:
            ret = kErrorInvalidEvent;
            break;
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Configure DLL parameters

This function posts a DLL configuration event to the kernel DLL CAL module

\param  pDllConfigParam_p       Pointer to the DLL configuration parameters

\return The function returns a tOplkError error code.

\ingroup module_dllucal
*/
//------------------------------------------------------------------------------
tOplkError dllucal_config(tDllConfigParam* pDllConfigParam_p)
{
    tOplkError  ret = kErrorOk;
    tEvent      event;

    event.eventSink = kEventSinkDllkCal;
    event.eventType = kEventTypeDllkConfig;
    event.pEventArg = pDllConfigParam_p;
    event.eventArgSize = sizeof(*pDllConfigParam_p);
    ret = eventu_postEvent(&event);

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Configure identity of local node

This function posts a dll identity event to the kernel DLL CAL module to
configure the identity of a local node for IdentResponse.

\param  pDllIdentParam_p        Pointer to ident parameters

\return The function returns a tOplkError error code.

\ingroup module_dllucal
*/
//------------------------------------------------------------------------------
tOplkError dllucal_setIdentity(tDllIdentParam* pDllIdentParam_p)
{
    tOplkError  ret = kErrorOk;
    tEvent      event;

    event.eventSink = kEventSinkDllkCal;
    event.eventType = kEventTypeDllkIdentity;
    event.pEventArg = pDllIdentParam_p;
    event.eventArgSize = sizeof(*pDllIdentParam_p);
    ret = eventu_postEvent(&event);
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Register ASnd handler

This function register the specified handler for the specified ASnd service
ID with the specified node ID filter.

\param  serviceId_p             ASnd service ID to register handler for.
\param  pfnDlluCbAsnd_p         Pointer to callback function.
\param  filter_p                Node filter ID.

\return The function returns a tOplkError error code.

\ingroup module_dllucal
*/
//------------------------------------------------------------------------------
tOplkError dllucal_regAsndService(tDllAsndServiceId serviceId_p,
                                  tEplDlluCbAsnd pfnDlluCbAsnd_p,
                                  tDllAsndFilter filter_p)
{
    tOplkError  ret = kErrorOk;

    if (serviceId_p < tabentries(instance_l.apfnDlluCbAsnd))
    {
        // memorize function pointer
        instance_l.apfnDlluCbAsnd[serviceId_p] = pfnDlluCbAsnd_p;

        if (pfnDlluCbAsnd_p == NULL)
        {   // close filter
            filter_p = kDllAsndFilterNone;
        }

        // set filter in DLL module in kernel part
        ret = SetAsndServiceIdFilter(serviceId_p, filter_p);
    }
    else
    {
        ret = kErrorDllInvalidAsndServiceId;
    }
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Send asynchronous frame

This function sends an asynchronous fram with the specified priority.

\param  pFrameInfo_p            Pointer to asynchronous frame. The frame size
                                includes the ethernet header (14 bytes).
\param  priority_p              Priority for sending this frame.

\return The function returns a tOplkError error code.

\ingroup module_dllucal
*/
//------------------------------------------------------------------------------
tOplkError dllucal_sendAsyncFrame(tFrameInfo* pFrameInfo_p,
                                  tDllAsyncReqPriority priority_p)
{
    tOplkError  ret = kErrorOk;
    tEvent      event;

    switch (priority_p)
    {
        case kDllAsyncReqPrioNmt:
            ret = instance_l.pTxNmtFuncs->pfnInsertDataBlock(
                                        instance_l.dllCalQueueTxNmt,
                                        (BYTE*)pFrameInfo_p->pFrame,
                                        &(pFrameInfo_p->frameSize));
            break;

        default:
            ret = instance_l.pTxGenFuncs->pfnInsertDataBlock(
                                        instance_l.dllCalQueueTxGen,
                                        (BYTE*)pFrameInfo_p->pFrame,
                                        &(pFrameInfo_p->frameSize));
            break;
    }

    if (ret != kErrorOk)
    {
        goto Exit;
    }

    // post event to DLL
    event.eventSink = kEventSinkDllk;
    event.eventType = kEventTypeDllkFillTx;
    OPLK_MEMSET(&event.netTime, 0x00, sizeof(event.netTime));
    event.pEventArg = &priority_p;
    event.eventArgSize = sizeof(priority_p);
    ret = eventu_postEvent(&event);

Exit:
    return ret;
}

#if defined(CONFIG_INCLUDE_NMT_MN)
//------------------------------------------------------------------------------
/**
\brief  Issue a StatusRequest or IdentRequest

This function issues a StatusRequest or an IdentRequest to the specified node.

\param  service_p               Request service ID
\param  nodeId_p                The node to send the request.
\param  soaFlag1_p              Flag1 for this node (transmit in SoA and PReq).
                                If 0xff this flag is ignored.

\return The function returns a tOplkError error code.

\ingroup module_dllucal
*/
//------------------------------------------------------------------------------
tOplkError dllucal_issueRequest(tDllReqServiceId service_p, UINT nodeId_p,
                                BYTE soaFlag1_p)
{
    tOplkError          ret = kErrorOk;
    tEvent              event;
    tDllCalIssueRequest issueReq;

    // add node to appropriate request queue
    switch (service_p)
    {
        case kDllReqServiceIdent:
        case kDllReqServiceStatus:
            event.eventSink = kEventSinkDllkCal;
            event.eventType = kEventTypeDllkIssueReq;
            issueReq.service = service_p;
            issueReq.nodeId = nodeId_p;
            issueReq.soaFlag1 = soaFlag1_p;
            event.pEventArg = &issueReq;
            event.eventArgSize = sizeof(issueReq);
            ret = eventu_postEvent(&event);
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
\brief  Issue a SyncRequest

This function issues a SyncRequest or an IdentRequest to the specified node.

\param  pSyncRequest_p          Pointer to sync request structure.
\param  size_p                  Size of sync request structure.

\return The function returns a tOplkError error code.

\ingroup module_dllucal
*/
//------------------------------------------------------------------------------
tOplkError dllucal_issueSyncRequest(tDllSyncRequest* pSyncRequest_p, UINT size_p)
{
    tOplkError  ret = kErrorOk;

    ret = instance_l.pTxSyncFuncs->pfnInsertDataBlock(instance_l.dllCalQueueTxSync,
                                                      (BYTE*)pSyncRequest_p, &size_p);
    return ret;
}
#endif


#if NMT_MAX_NODE_ID > 0
//------------------------------------------------------------------------------
/**
\brief  Configure the specified node

The function configures the specified node by sending a
kEventTypeDllkConfigNode event to the kernel DLL CAL module.

\param  pNodeInfo_p             Pointer to node info structure.

\return The function returns a tOplkError error code.

\ingroup module_dllucal
*/
//------------------------------------------------------------------------------
tOplkError dllucal_configNode(tDllNodeInfo* pNodeInfo_p)
{
    tOplkError  ret = kErrorOk;
    tEvent      event;

    event.eventSink = kEventSinkDllkCal;
    event.eventType = kEventTypeDllkConfigNode;
    event.pEventArg = pNodeInfo_p;
    event.eventArgSize = sizeof(*pNodeInfo_p);

    ret = eventu_postEvent(&event);

    return ret;
}


//------------------------------------------------------------------------------
/**
\brief  Add a node to the isochronous phase

The function adds a node to the isonchronous phase by sending a
kEventTypeDllkAddNode event to the kernel DLL CAL module.

\param  pNodeOpParam_p          Pointer to node info structure

\return The function returns a tOplkError error code.

\ingroup module_dllucal
*/
//------------------------------------------------------------------------------
tOplkError dllucal_addNode(tDllNodeOpParam* pNodeOpParam_p)
{
    tOplkError  ret = kErrorOk;
    tEvent      event;

    event.eventSink = kEventSinkDllkCal;
    event.eventType = kEventTypeDllkAddNode;
    event.pEventArg = pNodeOpParam_p;
    event.eventArgSize = sizeof(*pNodeOpParam_p);

    ret = eventu_postEvent(&event);

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Remove a node from the isochronous phase

The function removes the specified node from the isochronous phase by sending
a kEventTypeDllkDelNode event to the kernel DLL CAL module.

\param  pNodeOpParam_p          Pointer to node info structure

\return The function returns a tOplkError error code.

\ingroup module_dllucal
*/
//------------------------------------------------------------------------------
tOplkError dllucal_deleteNode(tDllNodeOpParam* pNodeOpParam_p)
{
    tOplkError  ret = kErrorOk;
    tEvent      event;

    event.eventSink = kEventSinkDllkCal;
    event.eventType = kEventTypeDllkDelNode;
    event.pEventArg = pNodeOpParam_p;
    event.eventArgSize = sizeof(*pNodeOpParam_p);

    ret = eventu_postEvent(&event);

    return ret;
}

#endif

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Forward filter event to kernel part

The function forwards a filter event to the kernel DLL CAL module.

\param  serviceId_p             ASnd Service ID to forward.
\param  filter_p                Node ID filter to forward.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError SetAsndServiceIdFilter(tDllAsndServiceId serviceId_p,
                                         tDllAsndFilter filter_p)
{
    tOplkError                  ret = kErrorOk;
    tEvent                      event;
    tDllCalAsndServiceIdFilter  servFilter;

    event.eventSink = kEventSinkDllkCal;
    event.eventType = kEventTypeDllkServFilter;
    servFilter.serviceId = serviceId_p;
    servFilter.filter = filter_p;
    event.pEventArg = &servFilter;
    event.eventArgSize = sizeof(servFilter);
    ret = eventu_postEvent(&event);

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Forward Asnd frame to desired user space module

\param  pFrameInfo_p             Pointer to the frame information structure

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError HandleRxAsndFrame(tFrameInfo *pFrameInfo_p)
{
    tMsgType        msgType;
    unsigned int    asndServiceId;
    tOplkError      ret = kErrorOk;
#if CONFIG_DLL_DEFERRED_RXFRAME_RELEASE_ASYNC != FALSE
    tOplkError      eventRet;
    tEvent          event;
#endif

    msgType = (tMsgType)ami_getUint8Le(&pFrameInfo_p->pFrame->messageType);
    if (msgType != kMsgTypeAsnd)
    {
        ret = kErrorInvalidOperation;
        goto Exit;
    }

    asndServiceId = (unsigned int)ami_getUint8Le(&pFrameInfo_p->pFrame->data.asnd.serviceId);
    if (asndServiceId < DLL_MAX_ASND_SERVICE_ID)
    {   // ASnd service ID is valid
        if (instance_l.apfnDlluCbAsnd[asndServiceId] != NULL)
        {   // handler was registered
            ret = instance_l.apfnDlluCbAsnd[asndServiceId](pFrameInfo_p);
        }
    }

Exit:
#if CONFIG_DLL_DEFERRED_RXFRAME_RELEASE_ASYNC != FALSE
    // call free function for Asnd frame
    event.eventSink = kEventSinkDllkCal;
    event.eventType = kEventTypeReleaseRxFrame;
    event.eventArgSize = sizeof(tFrameInfo);
    event.pEventArg = pFrameInfo_p;

    eventRet = eventu_postEvent(&event);

    if(eventRet != kErrorOk)
    {
        // Event post error is returned
        ret = eventRet;
    }
#endif

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Forward unreceived Asnd frame to desired user space module

\param  pAsndNotRx_p             Pointer to the frame information structure

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError HandleNotRxAsndFrame(tDllAsndNotRx* pAsndNotRx_p)
{
    tOplkError  ret = kErrorOk;
    BYTE        aBuffer[DLLUCAL_NOTRX_FRAME_SIZE];
    tPlkFrame*  pFrame = (tPlkFrame*)aBuffer;
    tFrameInfo  frameInfo;
    UINT        asndServiceId;

    ami_setUint8Le(&pFrame->srcNodeId, pAsndNotRx_p->nodeId);
    ami_setUint8Le(&pFrame->messageType, (BYTE)kMsgTypeAsnd);
    ami_setUint8Le(&pFrame->data.asnd.serviceId, pAsndNotRx_p->serviceId);

    frameInfo.frameSize = DLLUCAL_NOTRX_FRAME_SIZE;
    frameInfo.pFrame = pFrame;

    asndServiceId = (UINT)ami_getUint8Le(&pFrame->data.asnd.serviceId);
    if (asndServiceId < DLL_MAX_ASND_SERVICE_ID)
    {   // ASnd service ID is valid
        if (instance_l.apfnDlluCbAsnd[asndServiceId] != NULL)
        {   // handler was registered
            ret = instance_l.apfnDlluCbAsnd[asndServiceId](&frameInfo);
        }
    }

    return ret;
}

