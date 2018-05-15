/**
********************************************************************************
\file   dllucal.c

\brief  User DLL CAL module

This file contains the user DLL CAL module.

\ingroup module_dllucal
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
#include <common/oplkinc.h>
#include <common/memmap.h>
#include <user/dllucal.h>
#include <user/eventu.h>
#include <common/dllcal.h>
#include <common/ami.h>

//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#if (defined(CONFIG_INCLUDE_NMT_MN) && (CONFIG_DLLCAL_QUEUE == DIRECT_QUEUE))
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
/**
\brief DLLu CAL instance type

This structure defines an instance of the POWERLINK Data Link Layer
Communication Abstraction Layer user module.
*/
typedef struct
{
    tDlluCbAsnd              apfnDlluCbAsnd[DLL_MAX_ASND_SERVICE_ID];
                                                        ///< Array of callback functions registered for receiving incoming ASnd frames with a specific ServiceId

#if defined(CONFIG_INCLUDE_VETH)
    tDlluCbNonPlk            pfnDlluCbNonPlk;           ///< Callback function for received non-POWERLINK frames
    tDllCalQueueInstance     dllCalQueueTxVeth;         ///< DLL CAL queue instance for virtual Ethernet
    tDllCalFuncIntf*         pTxVethFuncs;              ///< Function pointer to the TX functions for virtual Ethernet
#endif

    tDllCalQueueInstance     dllCalQueueTxNmt;          ///< DLL CAL queue instance for NMT priority
    tDllCalFuncIntf*         pTxNmtFuncs;               ///< Function pointer to the TX functions for NMT priority

    tDllCalQueueInstance     dllCalQueueTxGen;          ///< DLL CAL queue instance for generic priority
    tDllCalFuncIntf*         pTxGenFuncs;               ///< Function pointer to the TX functions for generic priority

#if defined(CONFIG_INCLUDE_NMT_MN)
    tDllCalQueueInstance     dllCalQueueTxSync;         ///< DLL CAL queue instance for SyncRequest frames
    tDllCalFuncIntf*         pTxSyncFuncs;              ///< Function pointer to the TX functions for SyncRequest frames
#endif
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

static tOplkError setAsndServiceIdFilter(tDllAsndServiceId ServiceId_p,
                                         tDllAsndFilter Filter_p);
static tOplkError handleRxAsyncFrame(const tFrameInfo* pFrameInfo_p);
static tOplkError handleRxAsndFrame(const tFrameInfo* pFrameInfo_p);
static tOplkError handleRxAsyncFrameInfo(tFrameInfo* pFrameInfo_p);
static tOplkError handleNotRxAsndFrame(const tDllAsndNotRx* pAsndNotRx_p);
static tOplkError sendGenericAsyncFrame(const tFrameInfo* pFrameInfo_p);

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
    tOplkError  ret;

    // reset instance structure
    OPLK_MEMSET(&instance_l, 0, sizeof(instance_l));

    instance_l.pTxNmtFuncs = GET_DLLUCAL_INTERFACE();
    instance_l.pTxGenFuncs = GET_DLLUCAL_INTERFACE();
#if defined(CONFIG_INCLUDE_NMT_MN)
    instance_l.pTxSyncFuncs = GET_DLLUCAL_INTERFACE();
#endif
#if defined(CONFIG_INCLUDE_VETH)
    instance_l.pTxVethFuncs = GET_DLLUCAL_INTERFACE();
#endif

    ret = instance_l.pTxNmtFuncs->pfnAddInstance(&instance_l.dllCalQueueTxNmt,
                                                 kDllCalQueueTxNmt);
    if (ret != kErrorOk)
        goto Exit;

    ret = instance_l.pTxGenFuncs->pfnAddInstance(&instance_l.dllCalQueueTxGen,
                                                 kDllCalQueueTxGen);
    if (ret != kErrorOk)
        goto Exit;

#if defined(CONFIG_INCLUDE_NMT_MN)
    ret = instance_l.pTxSyncFuncs->pfnAddInstance(&instance_l.dllCalQueueTxSync,
                                                  kDllCalQueueTxSync);
    if (ret != kErrorOk)
        goto Exit;
#endif

#if defined(CONFIG_INCLUDE_VETH)
    ret = instance_l.pTxVethFuncs->pfnAddInstance(&instance_l.dllCalQueueTxVeth,
                                                  kDllCalQueueTxVeth);
    if (ret != kErrorOk)
        goto Exit;
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
    tOplkError  ret = kErrorOk;

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
    dllucal_regNonPlkHandler(NULL);
#endif
    // reset instance structure
    OPLK_MEMSET(&instance_l, 0, sizeof(instance_l));

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Process asynchronous frame event

The function processes an asynchronous frame event

\param[in]      pEvent_p            Event to process

\return The function returns a tOplkError error code.

\ingroup module_dllucal
*/
//------------------------------------------------------------------------------
tOplkError dllucal_process(const tEvent* pEvent_p)
{
    tOplkError              ret;
    tFrameInfo*             pFrameInfo;
    const tDllAsndNotRx*    pAsndNotRx;
    tFrameInfo              frameInfo;

    // Check parameter validity
    ASSERT(pEvent_p != NULL);

    switch (pEvent_p->eventType)
    {
        case kEventTypeAsndRx:
            // Argument pointer is frame
            frameInfo.frame.pBuffer = (tPlkFrame*)pEvent_p->eventArg.pEventArg;
            frameInfo.frameSize = pEvent_p->eventArgSize;
            pFrameInfo = &frameInfo;
            ret = handleRxAsyncFrame(pFrameInfo);
            break;

        case kEventTypeAsndRxInfo:
            // Argument pointer is frame info
            pFrameInfo = (tFrameInfo*)pEvent_p->eventArg.pEventArg;
            ret = handleRxAsyncFrameInfo(pFrameInfo);
            break;

        case kEventTypeAsndNotRx:
            pAsndNotRx = (const tDllAsndNotRx*)pEvent_p->eventArg.pEventArg;
            ret = handleNotRxAsndFrame(pAsndNotRx);
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

\param[in]      pDllConfigParam_p   Pointer to the DLL configuration parameters

\return The function returns a tOplkError error code.

\ingroup module_dllucal
*/
//------------------------------------------------------------------------------
tOplkError dllucal_config(const tDllConfigParam* pDllConfigParam_p)
{
    tOplkError  ret;
    tEvent      event;

    // Check parameter validity
    ASSERT(pDllConfigParam_p != NULL);

    event.eventSink = kEventSinkDllkCal;
    event.eventType = kEventTypeDllkConfig;
    event.eventArg.pEventArg = (void*)pDllConfigParam_p;
    event.eventArgSize = sizeof(*pDllConfigParam_p);

    ret = eventu_postEvent(&event);

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Configure identity of local node

This function posts a DLL identity event to the kernel DLL CAL module to
configure the identity of a local node for IdentResponse.

\param[in]      pDllIdentParam_p    Pointer to ident parameters

\return The function returns a tOplkError error code.

\ingroup module_dllucal
*/
//------------------------------------------------------------------------------
tOplkError dllucal_setIdentity(const tDllIdentParam* pDllIdentParam_p)
{
    tOplkError  ret;
    tEvent      event;

    // Check parameter validity
    ASSERT(pDllIdentParam_p != NULL);

    event.eventSink = kEventSinkDllkCal;
    event.eventType = kEventTypeDllkIdentity;
    event.eventArg.pEventArg = (void*)pDllIdentParam_p;
    event.eventArgSize = sizeof(*pDllIdentParam_p);

    ret = eventu_postEvent(&event);

    return ret;
}

#if defined(CONFIG_INCLUDE_VETH)
//------------------------------------------------------------------------------
/**
\brief  Register non-POWERLINK receive handler

This function register the handler for non-POWERLINK frames.

\param[in]  pfnNonPlkCb_p           Pointer to callback function

\return The function returns a tOplkError error code.

\ingroup module_dllucal
*/
//------------------------------------------------------------------------------
tOplkError dllucal_regNonPlkHandler(tDlluCbNonPlk pfnNonPlkCb_p)
{
    tOplkError ret = kErrorOk;

    instance_l.pfnDlluCbNonPlk = pfnNonPlkCb_p;

    //jz Disable vethcal forwarding if no callback is registered?

    return ret;
}
#endif

//------------------------------------------------------------------------------
/**
\brief  Register ASnd handler

This function register the specified handler for the specified ASnd service
ID with the specified node ID filter.

\param[in]      serviceId_p         ASnd service ID to register handler for.
\param[in]      pfnDlluCbAsnd_p     Pointer to callback function.
\param[in]      filter_p            Node filter ID.

\return The function returns a tOplkError error code.

\ingroup module_dllucal
*/
//------------------------------------------------------------------------------
tOplkError dllucal_regAsndService(tDllAsndServiceId serviceId_p,
                                  tDlluCbAsnd pfnDlluCbAsnd_p,
                                  tDllAsndFilter filter_p)
{
    tOplkError  ret;

    if (serviceId_p < tabentries(instance_l.apfnDlluCbAsnd))
    {
        // memorize function pointer
        instance_l.apfnDlluCbAsnd[serviceId_p] = pfnDlluCbAsnd_p;

        if (pfnDlluCbAsnd_p == NULL)
        {   // close filter
            filter_p = kDllAsndFilterNone;
        }

        // set filter in DLL module in kernel part
        ret = setAsndServiceIdFilter(serviceId_p, filter_p);
    }
    else
        ret = kErrorDllInvalidAsndServiceId;

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Send asynchronous frame

This function sends an asynchronous frame with the specified priority.

\param[in]      pFrameInfo_p        Pointer to asynchronous frame. The frame size
                                    includes the Ethernet header (14 bytes).
\param[in]      priority_p          Priority for sending this frame.

\return The function returns a tOplkError error code.

\ingroup module_dllucal
*/
//------------------------------------------------------------------------------
tOplkError dllucal_sendAsyncFrame(const tFrameInfo* pFrameInfo_p,
                                  tDllAsyncReqPriority priority_p)
{
    tOplkError  ret;
    tEvent      event;

    // Check parameter validity
    ASSERT(pFrameInfo_p != NULL);

    switch (priority_p)
    {
        case kDllAsyncReqPrioNmt:
            ret = instance_l.pTxNmtFuncs->pfnInsertDataBlock(
                                              instance_l.dllCalQueueTxNmt,
                                              pFrameInfo_p->frame.pBuffer,
                                              (size_t)pFrameInfo_p->frameSize);
            break;

        default:
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

    ret = eventu_postEvent(&event);

Exit:
    return ret;
}

#if defined(CONFIG_INCLUDE_NMT_MN)
//------------------------------------------------------------------------------
/**
\brief  Issue a StatusRequest or IdentRequest

This function issues a StatusRequest or an IdentRequest to the specified node.

\param[in]      service_p           Request service ID
\param[in]      nodeId_p            The node to send the request.
\param[in]      soaFlag1_p          Flag1 for this node (transmit in SoA and PReq).
                                    If 0xff this flag is ignored.

\return The function returns a tOplkError error code.

\ingroup module_dllucal
*/
//------------------------------------------------------------------------------
tOplkError dllucal_issueRequest(tDllReqServiceId service_p,
                                UINT nodeId_p,
                                UINT8 soaFlag1_p)
{
    tOplkError          ret;
    tEvent              event;
    tDllCalIssueRequest issueReq;

    // add node to appropriate request queue
    switch (service_p)
    {
        case kDllReqServiceIdent:
        case kDllReqServiceStatus:
            issueReq.service = service_p;
            issueReq.nodeId = nodeId_p;
            issueReq.soaFlag1 = soaFlag1_p;

            event.eventSink = kEventSinkDllkCal;
            event.eventType = kEventTypeDllkIssueReq;
            event.eventArg.pEventArg = &issueReq;
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

\param[in]      pSyncRequest_p      Pointer to sync request structure.
\param[in]      size_p              Size of sync request structure.

\return The function returns a tOplkError error code.

\ingroup module_dllucal
*/
//------------------------------------------------------------------------------
tOplkError dllucal_issueSyncRequest(const tDllSyncRequest* pSyncRequest_p,
                                    size_t size_p)
{
    tOplkError  ret;

    // Check parameter validity
    ASSERT(pSyncRequest_p != NULL);

    ret = instance_l.pTxSyncFuncs->pfnInsertDataBlock(instance_l.dllCalQueueTxSync,
                                                      pSyncRequest_p,
                                                      size_p);

    return ret;
}
#endif


#if (NMT_MAX_NODE_ID > 0)
//------------------------------------------------------------------------------
/**
\brief  Configure the specified node

The function configures the specified node by sending a
kEventTypeDllkConfigNode event to the kernel DLL CAL module.

\param[in]      pNodeInfo_p         Pointer to node info structure.

\return The function returns a tOplkError error code.

\ingroup module_dllucal
*/
//------------------------------------------------------------------------------
tOplkError dllucal_configNode(const tDllNodeInfo* pNodeInfo_p)
{
    tOplkError  ret;
    tEvent      event;

    // Check parameter validity
    ASSERT(pNodeInfo_p != NULL);

    event.eventSink = kEventSinkDllkCal;
    event.eventType = kEventTypeDllkConfigNode;
    event.eventArg.pEventArg = (void*)pNodeInfo_p;
    event.eventArgSize = sizeof(*pNodeInfo_p);

    ret = eventu_postEvent(&event);

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Add a node to the isochronous phase

The function adds a node to the isochronous phase by sending a
kEventTypeDllkAddNode event to the kernel DLL CAL module.

\param[in]      pNodeOpParam_p      Pointer to node info structure

\return The function returns a tOplkError error code.

\ingroup module_dllucal
*/
//------------------------------------------------------------------------------
tOplkError dllucal_addNode(const tDllNodeOpParam* pNodeOpParam_p)
{
    tOplkError  ret;
    tEvent      event;

    // Check parameter validity
    ASSERT(pNodeOpParam_p != NULL);

    event.eventSink = kEventSinkDllkCal;
    event.eventType = kEventTypeDllkAddNode;
    event.eventArg.pEventArg = (void*)pNodeOpParam_p;
    event.eventArgSize = sizeof(*pNodeOpParam_p);

    ret = eventu_postEvent(&event);

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Remove a node from the isochronous phase

The function removes the specified node from the isochronous phase by sending
a kEventTypeDllkDelNode event to the kernel DLL CAL module.

\param[in]      pNodeOpParam_p      Pointer to node info structure

\return The function returns a tOplkError error code.

\ingroup module_dllucal
*/
//------------------------------------------------------------------------------
tOplkError dllucal_deleteNode(const tDllNodeOpParam* pNodeOpParam_p)
{
    tOplkError  ret;
    tEvent      event;

    // Check parameter validity
    ASSERT(pNodeOpParam_p != NULL);

    event.eventSink = kEventSinkDllkCal;
    event.eventType = kEventTypeDllkDelNode;
    event.eventArg.pEventArg = (void*)pNodeOpParam_p;
    event.eventArgSize = sizeof(*pNodeOpParam_p);

    ret = eventu_postEvent(&event);

    return ret;
}

#endif

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief  Forward filter event to kernel part

The function forwards a filter event to the kernel DLL CAL module.

\param[in]      serviceId_p         ASnd Service ID to forward.
\param[in]      filter_p            Node ID filter to forward.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError setAsndServiceIdFilter(tDllAsndServiceId serviceId_p,
                                         tDllAsndFilter filter_p)
{
    tOplkError                  ret;
    tEvent                      event;
    tDllCalAsndServiceIdFilter  servFilter;

    servFilter.serviceId = serviceId_p;
    servFilter.filter = filter_p;

    event.eventSink = kEventSinkDllkCal;
    event.eventType = kEventTypeDllkServFilter;
    event.eventArg.pEventArg = &servFilter;
    event.eventArgSize = sizeof(servFilter);

    ret = eventu_postEvent(&event);

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Forward asynchronous frame to desired user space module

This function forwards the asynchronous frame to the desired module depending
on the frame type (e.g. POWERLINK or non-POWERLINK frames).

\param[in]      pFrameInfo_p        Pointer to the frame information structure

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError handleRxAsyncFrame(const tFrameInfo* pFrameInfo_p)
{
    tOplkError  ret = kErrorOk;
    UINT16      etherType = ami_getUint16Be(&pFrameInfo_p->frame.pBuffer->etherType);

    switch (etherType)
    {
        case C_DLL_ETHERTYPE_EPL:
            ret = handleRxAsndFrame(pFrameInfo_p);
            break;

        default:
            DEBUG_LVL_DLL_TRACE("Received frame with etherType=0x%04X\n", etherType);
#if defined(CONFIG_INCLUDE_VETH)
            if (instance_l.pfnDlluCbNonPlk != NULL)
                ret = instance_l.pfnDlluCbNonPlk(pFrameInfo_p);
#endif
            break;
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Forward Asnd frame to desired user space module

This function forwards Asnd frames depending on the Asnd service ID to the
corresponding module. If the module has not registered any callback function,
the Asnd frame is ignored silently.

\param[in]      pFrameInfo_p        Pointer to the frame information structure

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError handleRxAsndFrame(const tFrameInfo* pFrameInfo_p)
{
    tMsgType    msgType;
    UINT        asndServiceId;
    tOplkError  ret = kErrorOk;

    msgType = (tMsgType)ami_getUint8Le(&pFrameInfo_p->frame.pBuffer->messageType);
    if (msgType != kMsgTypeAsnd)
    {
        ret = kErrorInvalidOperation;
        goto Exit;
    }

    asndServiceId = (UINT)ami_getUint8Le(&pFrameInfo_p->frame.pBuffer->data.asnd.serviceId);
    if (asndServiceId < DLL_MAX_ASND_SERVICE_ID)
    {   // ASnd service ID is valid
        if (instance_l.apfnDlluCbAsnd[asndServiceId] != NULL)
        {   // handler was registered
            ret = instance_l.apfnDlluCbAsnd[asndServiceId](pFrameInfo_p);
        }
    }

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Forward Asnd frame to desired user space module

This function gets the Asnd frame from the kernel layer and forwards it to
user layer modules.

\param[in,out]  pFrameInfo_p        Pointer to the frame information structure

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError handleRxAsyncFrameInfo(tFrameInfo* pFrameInfo_p)
{
    tOplkError  ret;
    tEvent      event;
    tPlkFrame*  pKernelBuffer = pFrameInfo_p->frame.pBuffer;
    tPlkFrame*  pAcqBuffer;

    // Get Rx buffer from kernel layer
    pAcqBuffer = (tPlkFrame*)memmap_mapKernelBuffer(pKernelBuffer, pFrameInfo_p->frameSize);
    if (pAcqBuffer == NULL)
    {
        DEBUG_LVL_ERROR_TRACE("%s Getting the Rx buffer from kernel failed!\n", __func__);
        return kErrorDllOutOfMemory; //jz Use other error code?
    }

    // Set reference to kernel buffer for processing
    pFrameInfo_p->frame.pBuffer = pAcqBuffer;

    // Now handle the async frame
    ret = handleRxAsyncFrame(pFrameInfo_p);

    // Free the acquired kernel buffer
    memmap_unmapKernelBuffer(pAcqBuffer);

    // Restore frame info for releasing Rx frame
    pFrameInfo_p->frame.pBuffer = pKernelBuffer;

    // call free function for Asnd frame
    event.eventSink = kEventSinkDllkCal;
    event.eventType = kEventTypeReleaseRxFrame;
    event.eventArgSize = sizeof(tFrameInfo);
    event.eventArg.pEventArg = pFrameInfo_p;

    eventu_postEvent(&event);

    // Return handleRxAsyncFrameInfo() return value (ignore others)
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Forward unreceived Asnd frame to desired user space module

\param[in]      pAsndNotRx_p        Pointer to the frame information structure

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError handleNotRxAsndFrame(const tDllAsndNotRx* pAsndNotRx_p)
{
    tOplkError  ret = kErrorOk;
    UINT8       aBuffer[DLLUCAL_NOTRX_FRAME_SIZE];
    tPlkFrame*  pFrame = (tPlkFrame*)aBuffer;
    tFrameInfo  frameInfo;
    UINT        asndServiceId;

    ami_setUint8Le(&pFrame->srcNodeId, pAsndNotRx_p->nodeId);
    ami_setUint8Le(&pFrame->messageType, (UINT8)kMsgTypeAsnd);
    ami_setUint8Le(&pFrame->data.asnd.serviceId, pAsndNotRx_p->serviceId);

    frameInfo.frameSize = DLLUCAL_NOTRX_FRAME_SIZE;
    frameInfo.frame.pBuffer = pFrame;

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

//------------------------------------------------------------------------------
/**
\brief  Send asynchronous frame

This function sends an asynchronous frame with generic priority.
The EtherType of the given frame determines the queue to be used for queuing.
If the frame is a POWERLINK frame or EtherType is 0x0000, the generic priority Tx
queue is used. Other frame types (e.g. IP) are forwarded with the virtual
Ethernet Tx queue.

\param[in]      pFrameInfo_p        Pointer to asynchronous frame. The frame size
                                    includes the Ethernet header (14 bytes).

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError sendGenericAsyncFrame(const tFrameInfo* pFrameInfo_p)
{
    tOplkError  ret = kErrorOk;
    UINT16      etherType = ami_getUint16Be(&pFrameInfo_p->frame.pBuffer->etherType);

    if ((etherType == 0x0000) || (etherType == C_DLL_ETHERTYPE_EPL))
    {
        ret = instance_l.pTxGenFuncs->pfnInsertDataBlock(
                                          instance_l.dllCalQueueTxGen,
                                          pFrameInfo_p->frame.pBuffer,
                                          (size_t)pFrameInfo_p->frameSize);
    }
    else
    {
#if defined(CONFIG_INCLUDE_VETH)
        ret = instance_l.pTxVethFuncs->pfnInsertDataBlock(
                                           instance_l.dllCalQueueTxVeth,
                                           pFrameInfo_p->frame.pBuffer,
                                           (size_t)pFrameInfo_p->frameSize);
#else
    // Return error since virtual Ethernet is not existing!
    ret = kErrorIllegalInstance;
    DEBUG_LVL_ERROR_TRACE("%s() frame cannot be sent, "
                          "because virtual Ethernet is inactive!\n",
                          __func__);
#endif
    }

    return ret;
}

/// \}
