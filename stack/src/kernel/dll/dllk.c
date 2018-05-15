/**
********************************************************************************
\file   dllk.c

\brief  Implementation of DLL kernel module

This file contains the implementation of the DLL kernel module.

\ingroup module_dllk
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2015, SYSTEC electronic GmbH
Copyright (c) 2017, B&R Industrial Automation GmbH
Copyright (c) 2017, Kalycito Infotech Private Limited
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
#include <kernel/dllk.h>

#include "dllkframe.h"
#include "dllknode.h"
#include "dllk-internal.h"
#include "dllkstatemachine.h"

#include <kernel/dllktgt.h>
#include <kernel/edrv.h>
#include <kernel/errhndk.h>
#include <kernel/eventk.h>
#include <oplk/benchmark.h>

#if (CONFIG_DLL_PROCESS_SYNC == DLL_PROCESS_SYNC_ON_TIMER)
#include <kernel/synctimer.h>
#endif

#if defined(CONFIG_INCLUDE_NMT_RMN)
#include <kernel/hrestimer.h>
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
tDllkInstance               dllkInstance_g;
TGT_DLLK_DEFINE_CRITICAL_SECTION

//------------------------------------------------------------------------------
// global function prototypes
//------------------------------------------------------------------------------

//============================================================================//
//            P R I V A T E   D E F I N I T I O N S                           //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#if (defined(CONFIG_INCLUDE_NMT_MN) && defined(CONFIG_INCLUDE_SOC_TIME_FORWARD))
#define DLLK_SEC_TO_USEC_FACTOR 1000000U
#endif

//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tEdrvTxBuffer        aDllkTxBuffer_l[DLLK_TXFRAME_COUNT];

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
/* Cycle/Sync Callback functions */
#if (CONFIG_DLL_PROCESS_SYNC == DLL_PROCESS_SYNC_ON_TIMER)
static tOplkError cbCnTimerSync(void);
static tOplkError cbCnLossOfSync(void);
#if (CONFIG_DLL_PRES_CHAINING_CN != FALSE)
static tOplkError cbCnPresFallbackTimeout(void);
#endif
#endif

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Initialize DLL kernel module

The function initializes the DLL kernel module.

\return The function returns a tOplkError error code.

\ingroup module_dllk
*/
//------------------------------------------------------------------------------
tOplkError dllk_init(void)
{
    tOplkError      ret = kErrorOk;
    UINT            index;

    // Reset the instance
    OPLK_MEMSET(&dllkInstance_g, 0, sizeof(dllkInstance_g));

#if (CONFIG_DLL_PROCESS_SYNC == DLL_PROCESS_SYNC_ON_TIMER)
    if ((ret = synctimer_registerHandler(cbCnTimerSync)) != kErrorOk)
        return ret;

    if ((ret = synctimer_registerLossOfSyncHandler(cbCnLossOfSync)) != kErrorOk)
        return ret;

#if (CONFIG_DLL_PRES_CHAINING_CN != FALSE)
    ret = synctimer_registerLossOfSyncHandler2(cbCnPresFallbackTimeout);
    if (ret != kErrorOk)
        return ret;
#endif

   ret = synctimer_setSyncShift(CONFIG_DLL_SOC_SYNC_SHIFT_US);
   if (ret != kErrorOk)
       return ret;
#endif

    // initialize and link pointers in instance structure to frame tables
    dllkInstance_g.pTxBuffer = aDllkTxBuffer_l;
    dllkInstance_g.maxTxFrames = sizeof(aDllkTxBuffer_l) / sizeof(tEdrvTxBuffer);
    dllkInstance_g.dllState = kDllGsInit;               // initialize state

#if (NMT_MAX_NODE_ID > 0)
    // set up node info structure
    for (index = 0; index < tabentries(dllkInstance_g.aNodeInfo); index++)
    {
        dllkInstance_g.aNodeInfo[index].nodeId = index + 1;
#if defined(CONFIG_INCLUDE_NMT_MN)
        dllkInstance_g.aNodeInfo[index].soaFlag1 |= PLK_FRAME_FLAG1_ER;
        dllkInstance_g.aNodeInfo[index].errSigState = STATE_MN_ERRSIG_INIT_WAIT_EC1;
        dllkInstance_g.aNodeInfo[index].errSigReqCnt = 0;
#endif
    }
#endif

    // initialize TxBuffer array
    for (index = 0; index < dllkInstance_g.maxTxFrames; index++)
    {
        dllkInstance_g.pTxBuffer[index].pBuffer = NULL;
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Shut down DLL kernel module

The function shuts down the DLL kernel module.

\return The function returns a tOplkError error code.

\ingroup module_dllk
*/
//------------------------------------------------------------------------------
tOplkError dllk_exit(void)
{
    tOplkError      ret = kErrorOk;

    // reset state
    dllkInstance_g.dllState = kDllGsInit;

    //De-register synctimer handler
#if (CONFIG_DLL_PROCESS_SYNC == DLL_PROCESS_SYNC_ON_TIMER)
    ret = synctimer_registerHandler(NULL);
    if (ret != kErrorOk)
        return ret;

    ret = synctimer_registerLossOfSyncHandler(NULL);
    if (ret != kErrorOk)
        return ret;

#if (CONFIG_DLL_PRES_CHAINING_CN != FALSE)
    ret = synctimer_registerLossOfSyncHandler2(NULL);
    if (ret != kErrorOk)
        return ret;
#endif
#endif

    // Reset the instance
    OPLK_MEMSET(&dllkInstance_g, 0, sizeof(dllkInstance_g));

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Configure parameters of DLL

The function configures the parameters of the DLL. It is called before
NMT_GS_COMMUNICATING will be entered.

\param[in]      pDllConfigParam_p   Pointer to configuration parameters.

\return The function returns a tOplkError error code.

\ingroup module_dllk
*/
//------------------------------------------------------------------------------
tOplkError dllk_config(const tDllConfigParam* pDllConfigParam_p)
{
    tNmtState       nmtState;
    tOplkError      ret = kErrorOk;

    // Check parameter validity
    ASSERT(pDllConfigParam_p != NULL);

    nmtState = dllkInstance_g.nmtState;

    if (nmtState > kNmtGsResetConfiguration)
    {   // configuration updates are only allowed in state DLL_GS_INIT, except LossOfFrameTolerance,
        // because all other parameters are "valid on reset".
        dllkInstance_g.dllConfigParam.lossOfFrameTolerance = pDllConfigParam_p->lossOfFrameTolerance;

#if (CONFIG_DLL_PROCESS_SYNC == DLL_PROCESS_SYNC_ON_TIMER)
        ret = synctimer_setLossOfSyncTolerance(pDllConfigParam_p->lossOfFrameTolerance);
#endif
    }
    else
    {   // copy entire configuration to local storage,
        // because we are in state DLL_GS_INIT
        OPLK_MEMCPY(&dllkInstance_g.dllConfigParam,
                    pDllConfigParam_p,
                    ((size_t)pDllConfigParam_p->sizeOfStruct < sizeof(tDllConfigParam) ?
                        (size_t)pDllConfigParam_p->sizeOfStruct : sizeof(tDllConfigParam)));
#if (EDRV_USE_TTTX != FALSE)
        // Time triggered sending requires sync on SOC
        dllkInstance_g.dllConfigParam.syncNodeId = C_ADR_SYNC_ON_SOC;
#endif
    }

    if ((dllkInstance_g.dllConfigParam.cycleLen != 0) &&
        (dllkInstance_g.dllConfigParam.lossOfFrameTolerance != 0))
    {   // monitor POWERLINK cycle, calculate frame timeout
        dllkInstance_g.frameTimeout = (1000LL * ((UINT64)dllkInstance_g.dllConfigParam.cycleLen)) +
            ((UINT64)dllkInstance_g.dllConfigParam.lossOfFrameTolerance);
#if (defined(CONFIG_INCLUDE_SOC_TIME_FORWARD) && defined(CONFIG_INCLUDE_NMT_MN))
        // Store the cycle time value in nsec and sec format for net time computation
        dllkInstance_g.cycleLength.sec = dllkInstance_g.dllConfigParam.cycleLen / DLLK_SEC_TO_USEC_FACTOR;
        dllkInstance_g.cycleLength.nsec = (dllkInstance_g.dllConfigParam.cycleLen % DLLK_SEC_TO_USEC_FACTOR) * 1000;
#endif
    }
    else
    {
        dllkInstance_g.frameTimeout = 0LL;
    }

    if (dllkInstance_g.dllConfigParam.fAsyncOnly != FALSE)
    {   // it is configured as async-only CN
        // disable multiplexed cycle, so that cycleCount will not be incremented spuriously on SoC
        dllkInstance_g.dllConfigParam.multipleCycleCnt = 0;
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Configure identity of local node for IdentResponse

The function sets the identity of the local node (may be at any time, e.g. in
case of host name change).

\param[in]      pDllIdentParam_p    Pointer to identity parameters.

\return The function returns a tOplkError error code.

\ingroup module_dllk
*/
//------------------------------------------------------------------------------
tOplkError dllk_setIdentity(const tDllIdentParam* pDllIdentParam_p)
{
    // Check parameter validity
    ASSERT(pDllIdentParam_p != NULL);

    OPLK_MEMCPY(&dllkInstance_g.dllIdentParam,
                pDllIdentParam_p,
                ((size_t)pDllIdentParam_p->sizeOfStruct < sizeof(tDllIdentParam) ?
                    (size_t)pDllIdentParam_p->sizeOfStruct : sizeof(tDllIdentParam)));

    // $$$ if IdentResponse frame exists, update it
    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Register handler for non-POWERLINK frames

The function registers the handler for non-POWERLINK frames (used by Virtual
Ethernet driver).

\param[in]      pfnDllkCbAsync_p    Pointer to callback function which will be called in
                                    interrupt context normally.

\return The function returns a tOplkError error code.
\retval kErrorOk                    Handler is successfully registered.
\retval kErrorDllCbAsyncRegistered  There is already a handler registered.

\ingroup module_dllk
*/
//------------------------------------------------------------------------------
tOplkError dllk_regAsyncHandler(tDllkCbAsync pfnDllkCbAsync_p)
{
    tOplkError  ret = kErrorOk;

    if (dllkInstance_g.pfnCbAsync == NULL)
    {   // no handler registered yet
        dllkInstance_g.pfnCbAsync = pfnDllkCbAsync_p;
    }
    else
    {   // handler already registered
        ret = kErrorDllCbAsyncRegistered;
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  De-register handler for non-POWERLINK frames

The function de-registers the handler for non-POWERLINK frames (used by Virtual
Ethernet driver).

\param[in]      pfnDllkCbAsync_p    Pointer to callback function. It will be checked if
                                    this function was registered before de-registering
                                    it.

\return The function returns a tOplkError error code.
\retval kErrorOk                    Handler is successfully de-registered.
\retval kErrorDllCbAsyncRegistered  Another handler is registered.

\ingroup module_dllk
*/
//------------------------------------------------------------------------------
tOplkError dllk_deregAsyncHandler(tDllkCbAsync pfnDllkCbAsync_p)
{
    tOplkError  ret = kErrorOk;

    if (dllkInstance_g.pfnCbAsync == pfnDllkCbAsync_p)
    {   // same handler is registered, de-register it
        dllkInstance_g.pfnCbAsync = NULL;
    }
    else
    {   // wrong handler or no handler registered
        ret = kErrorDllCbAsyncRegistered;
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Register handler for sync events

The function registers the handler for SYNC events.

\param[in]      pfnCbSync_p         Pointer to callback function for sync event. It
                                    will be called in event context.

\return The function returns the previously registered sync callback function.

\ingroup module_dllk
*/
//------------------------------------------------------------------------------
tSyncCb dllk_regSyncHandler(tSyncCb pfnCbSync_p)
{
    tSyncCb  pfnCbOld;

    pfnCbOld = dllkInstance_g.pfnCbSync;
    dllkInstance_g.pfnCbSync = pfnCbSync_p;

    return pfnCbOld;
}

//------------------------------------------------------------------------------
/**
\brief  Register handler for RPDO frames

The function registers the handler for RPDO frames.

\param[in]      pfnDllkCbProcessRpdo_p  Pointer to callback function. It
                                        will be called in interrupt context.

\ingroup module_dllk
*/
//------------------------------------------------------------------------------
void dllk_regRpdoHandler(tDllkCbProcessRpdo pfnDllkCbProcessRpdo_p)
{
    dllkInstance_g.pfnCbProcessRpdo = pfnDllkCbProcessRpdo_p;
}

//------------------------------------------------------------------------------
/**
\brief  Register handler for TPDO frames

The function registers the handler for TPDO frames.

\param[in]      pfnDllkCbProcessTpdo_p  Pointer to callback function. It
                                        will be called in context of kernel part event
                                        queue.

\ingroup module_dllk
*/
//------------------------------------------------------------------------------
void dllk_regTpdoHandler(tDllkCbProcessTpdo pfnDllkCbProcessTpdo_p)
{
    dllkInstance_g.pfnCbProcessTpdo = pfnDllkCbProcessTpdo_p;
}

//------------------------------------------------------------------------------
/**
\brief  Set the specified node ID filter

The function sets the specified node ID filter for the specified AsndServiceId.
It registers C_DLL_MULTICAST_ASND in Ethernet driver if any AsndServiceId is open.

\param[in]      serviceId_p         ASnd service ID.
\param[in]      filter_p            Node ID filter.

\return The function returns a tOplkError error code.
\retval kErrorOk                          Filter was successfully set.
\retval kErrorDllInvalidAsndServiceId     An invalid service ID was specified.

\ingroup module_dllk
*/
//------------------------------------------------------------------------------
tOplkError dllk_setAsndServiceIdFilter(tDllAsndServiceId serviceId_p,
                                       tDllAsndFilter filter_p)
{
    tOplkError  ret = kErrorOk;

    if (serviceId_p < tabentries(dllkInstance_g.aAsndFilter))
        dllkInstance_g.aAsndFilter[serviceId_p] = filter_p;
    else
        ret = kErrorDllInvalidAsndServiceId;

    return ret;
}


#if ((CONFIG_DLL_DEFERRED_RXFRAME_RELEASE_SYNC != FALSE) || \
     (CONFIG_DLL_DEFERRED_RXFRAME_RELEASE_ASYNC != FALSE))
//------------------------------------------------------------------------------
/**
\brief  Release RX buffer frame in Edrv

The function releases the RX buffer for the specified RX frame in Edrv.

\param[in]      pFrame_p            Pointer to frame.
\param[in]      frameSize_p         Size of frame.

\return The function returns a tOplkError error code.

\ingroup module_dllk
*/
//------------------------------------------------------------------------------
tOplkError dllk_releaseRxFrame(tPlkFrame* pFrame_p, UINT frameSize_p)
{
    tOplkError      ret;
    tEdrvRxBuffer   rxBuffer;

    rxBuffer.pBuffer = pFrame_p;
    rxBuffer.rxFrameSize = frameSize_p;

    ret = edrv_releaseRxBuffer(&rxBuffer);

    return ret;
}
#endif

#if (NMT_MAX_NODE_ID > 0)
//------------------------------------------------------------------------------
/**
\brief  Configure the specified node

The function configures the specified node (e.g. payload limits and timeouts).

\param[in]      pNodeInfo_p         Pointer to node information.

\return The function returns a tOplkError error code.

\ingroup module_dllk
*/
//------------------------------------------------------------------------------
tOplkError dllk_configNode(const tDllNodeInfo* pNodeInfo_p)
{
    tOplkError          ret = kErrorOk;
    tDllkNodeInfo*      pIntNodeInfo;
    tNmtState           nmtState;

    // Check parameter validity
    ASSERT(pNodeInfo_p != NULL);

    nmtState = dllkInstance_g.nmtState;

    if ((nmtState > kNmtGsResetConfiguration) &&
        (pNodeInfo_p->nodeId != dllkInstance_g.dllConfigParam.nodeId))
    {   // configuration updates are only allowed in reset states
        return kErrorInvalidOperation;
    }

    pIntNodeInfo = dllknode_getNodeInfo(pNodeInfo_p->nodeId);
    if (pIntNodeInfo == NULL)
    {   // no node info structure available
        return kErrorDllNoNodeInfo;
    }

    // copy node configuration
    if (pNodeInfo_p->presPayloadLimit > dllkInstance_g.dllConfigParam.isochrRxMaxPayload)
        pIntNodeInfo->presPayloadLimit = dllkInstance_g.dllConfigParam.isochrRxMaxPayload;
    else
        pIntNodeInfo->presPayloadLimit = pNodeInfo_p->presPayloadLimit;

#if defined(CONFIG_INCLUDE_NMT_MN)
    pIntNodeInfo->presTimeoutNs = pNodeInfo_p->presTimeoutNs;
    if (pNodeInfo_p->preqPayloadLimit > dllkInstance_g.dllConfigParam.isochrTxMaxPayload)
        pIntNodeInfo->preqPayloadLimit = dllkInstance_g.dllConfigParam.isochrTxMaxPayload;
    else
        pIntNodeInfo->preqPayloadLimit = pNodeInfo_p->preqPayloadLimit;

    // initialize elements of internal node info structure
    pIntNodeInfo->soaFlag1 = PLK_FRAME_FLAG1_ER;
    pIntNodeInfo->errSigState = STATE_MN_ERRSIG_INIT_WAIT_EC1;
    pIntNodeInfo->errSigReqCnt = 0;
    pIntNodeInfo->fSoftDelete = FALSE;
    pIntNodeInfo->dllErrorEvents = 0L;
    pIntNodeInfo->nmtState = kNmtCsNotActive;
#endif

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Add the specified node

The function adds the specified node into the isochronous phase.

\param[in]      pNodeOpParam_p      Pointer to node operation parameters.

\return The function returns a tOplkError error code.

\ingroup module_dllk
*/
//------------------------------------------------------------------------------
tOplkError dllk_addNode(const tDllNodeOpParam* pNodeOpParam_p)
{
    tOplkError          ret = kErrorOk;
    tDllkNodeInfo*      pIntNodeInfo;
    tNmtState           nmtState;
    BOOL                fUpdateEdrv = FALSE;

    // Check parameter validity
    ASSERT(pNodeOpParam_p != NULL);

    nmtState = dllkInstance_g.nmtState;

    pIntNodeInfo = dllknode_getNodeInfo(pNodeOpParam_p->nodeId);
    if (pIntNodeInfo == NULL)
    {   // no node info structure available
        return kErrorDllNoNodeInfo;
    }

    DEBUG_LVL_DLL_TRACE("%s() nodeId: %d\n", __func__, pNodeOpParam_p->nodeId);

    switch (pNodeOpParam_p->opNodeType)
    {
#if defined(CONFIG_INCLUDE_NMT_MN)
        case kDllNodeOpTypeIsochronous:
            ret = dllknode_addNodeIsochronous(pIntNodeInfo);
            break;
#endif

        case kDllNodeOpTypeFilterPdo:
        case kDllNodeOpTypeFilterHeartbeat:
            if (NMT_IF_CN_OR_RMN(nmtState))
                fUpdateEdrv = TRUE;
            ret = dllknode_addNodeFilter(pIntNodeInfo, pNodeOpParam_p->opNodeType, fUpdateEdrv);
            break;

        default:
            ret = kErrorDllInvalidParam;
            break;
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Delete the specified node

The function deletes the specified node from the isochronous phase.

\param[in]      pNodeOpParam_p      Pointer to node operation parameters.

\return The function returns a tOplkError error code.

\ingroup module_dllk
*/
//------------------------------------------------------------------------------
tOplkError dllk_deleteNode(const tDllNodeOpParam* pNodeOpParam_p)
{
    tOplkError          ret = kErrorOk;
    tDllkNodeInfo*      pIntNodeInfo;
    tNmtState           nmtState;
    BOOL                fUpdateEdrv = FALSE;
    UINT                index;

    // Check parameter validity
    ASSERT(pNodeOpParam_p != NULL);

    nmtState = dllkInstance_g.nmtState;

    if (pNodeOpParam_p->nodeId == C_ADR_BROADCAST)
    {
        switch (pNodeOpParam_p->opNodeType)
        {
            case kDllNodeOpTypeFilterPdo:
            case kDllNodeOpTypeFilterHeartbeat:
                if (NMT_IF_CN_OR_RMN(nmtState))
                    fUpdateEdrv = TRUE;

                for (index = 0, pIntNodeInfo = &dllkInstance_g.aNodeInfo[0];
                     index < tabentries(dllkInstance_g.aNodeInfo);
                     index++, pIntNodeInfo++)
                {
                    ret = dllknode_deleteNodeFilter(pIntNodeInfo, pNodeOpParam_p->opNodeType, fUpdateEdrv);
                }
                break;

            default:
                ret = kErrorDllInvalidParam;
                break;
        }
        return ret;
    }

    pIntNodeInfo = dllknode_getNodeInfo(pNodeOpParam_p->nodeId);
    if (pIntNodeInfo == NULL)
    {   // no node info structure available
        return kErrorDllNoNodeInfo;
    }

    DEBUG_LVL_DLL_TRACE("%s() nodeId: %d\n", __func__, pNodeOpParam_p->nodeId);

    switch (pNodeOpParam_p->opNodeType)
    {
#if defined(CONFIG_INCLUDE_NMT_MN)
        case kDllNodeOpTypeIsochronous:
            ret = dllknode_deleteNodeIsochronous(pIntNodeInfo);
            break;

        case kDllNodeOpTypeSoftDelete:
            pIntNodeInfo->fSoftDelete = TRUE;
            break;
#endif

        case kDllNodeOpTypeFilterPdo:
        case kDllNodeOpTypeFilterHeartbeat:
            if (NMT_IF_CN_OR_RMN(nmtState))
                fUpdateEdrv = TRUE;
            ret = dllknode_deleteNodeFilter(pIntNodeInfo, pNodeOpParam_p->opNodeType, fUpdateEdrv);
            break;

        default:
            ret = kErrorDllInvalidParam;
            break;
    }

    return ret;
}

#endif // NMT_MAX_NODE_ID > 0

#if defined(CONFIG_INCLUDE_NMT_MN)
//------------------------------------------------------------------------------
/**
\brief  Set Flag1 of the specified node

The function sets Flag1 (for PReq and SoA) of the specified node.

\param[in]      nodeId_p            Node ID to set the flag.
\param[in]      soaFlag1_p          Flag1.

\return The function returns a tOplkError error code.
\retval kErrorOk                    Flag is successfully set.
\retval kErrorDllNoNodeInfo         Node is not found.

\ingroup module_dllk
*/
//------------------------------------------------------------------------------
tOplkError dllk_setFlag1OfNode(UINT nodeId_p, UINT8 soaFlag1_p)
{
    tDllkNodeInfo*   pNodeInfo;

    pNodeInfo = dllknode_getNodeInfo(nodeId_p);
    if (pNodeInfo == NULL)
    {   // no node info structure available
        return kErrorDllNoNodeInfo;
    }
    // store flag1 in internal node info structure
    pNodeInfo->soaFlag1 = soaFlag1_p;

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Get current CN node list

The function returns the first info structure of the first node in the isochronous
phase. It is only useful for the kernel error handler module.

\param[out]     ppCnNodeIdList_p    Pointer to array of bytes with node-ID list.
                                    Array is terminated by value 0.

\ingroup module_dllk
*/
//------------------------------------------------------------------------------
void dllk_getCurrentCnNodeIdList(UINT8** ppCnNodeIdList_p)
{
    // Check parameter validity
    ASSERT(ppCnNodeIdList_p != NULL);

    *ppCnNodeIdList_p = &dllkInstance_g.aCnNodeIdList[dllkInstance_g.curTxBufferOffsetCycle ^ 1][0];
}

//------------------------------------------------------------------------------
/**
\brief  Get MAC address of the specified node

The function returns the MAC address of the specified node.

\param[in]      nodeId_p            Node ID from which to get MAC address.
\param[out]     pCnMacAddress_p     Pointer to store MAC address.

\return The function returns a tOplkError error code.
\retval kErrorOk                    MAC address is successfully read.
\retval kErrorDllNoNodeInfo         Node is not found.

\ingroup module_dllk
*/
//------------------------------------------------------------------------------
tOplkError dllk_getCnMacAddress(UINT nodeId_p, UINT8* pCnMacAddress_p)
{
    tDllkNodeInfo*   pNodeInfo;

    // Check parameter validity
    ASSERT(pCnMacAddress_p != NULL);

    pNodeInfo = dllknode_getNodeInfo(nodeId_p);
    if (pNodeInfo == NULL)
    {   // no node info structure available
        return kErrorDllNoNodeInfo;
    }

    OPLK_MEMCPY(pCnMacAddress_p, pNodeInfo->aMacAddr, 6);

    return kErrorOk;
}

#endif

//------------------------------------------------------------------------------
/**
\brief  Post an event

The function posts the specified event type to itself.

\param[in]      eventType_p         Event type to post.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
tOplkError dllk_postEvent(tEventType eventType_p)
{
    tOplkError  ret;
    tEvent      event;

    event.eventSink = kEventSinkDllk;
    event.eventType = eventType_p;
    event.eventArgSize = 0;
    event.eventArg.pEventArg = NULL;

    ret = eventk_postEvent(&event);

    return ret;
}

#if defined(CONFIG_INCLUDE_NMT_RMN)
//------------------------------------------------------------------------------
/**
\brief  RMN switch-over timer callback function

This function is called by the timer module.

\param[in]      pEventArg_p         Pointer to timer event argument.

\return The function returns a pointer to the node Information of the node.
*/
//------------------------------------------------------------------------------
tOplkError dllk_cbTimerSwitchOver(const tTimerEventArg* pEventArg_p)
{
    tOplkError      ret = kErrorOk;
    tNmtState       nmtState;
    UINT32          arg;
    tNmtEvent       nmtEvent;
    tEvent          event;

    TGT_DLLK_DECLARE_FLAGS;

#if (CONFIG_TIMER_USE_HIGHRES != FALSE)
    // Check parameter validity
    ASSERT(pEventArg_p != NULL);
#else
    UNUSED_PARAMETER(pEventArg_p);
#endif

    TGT_DLLK_ENTER_CRITICAL_SECTION();

#if (CONFIG_TIMER_USE_HIGHRES != FALSE)
    if (pEventArg_p->timerHdl.handle != dllkInstance_g.timerHdlSwitchOver)
    {   // zombie callback - just exit
        goto Exit;
    }
#endif

    nmtState = dllkInstance_g.nmtState;
    if (!NMT_IF_ACTIVE_CN(nmtState))
        goto Exit;

    ret = edrv_sendTxBuffer(&dllkInstance_g.pTxBuffer[DLLK_TXFRAME_AMNI]);
    if (ret != kErrorOk)
        goto Exit;

    // increment relativeTime for missing SoC
    dllkInstance_g.socTime.relTime += dllkInstance_g.dllConfigParam.cycleLen;

    nmtEvent = kNmtEventDllReSwitchOverTimeout;
    event.eventSink = kEventSinkNmtk;
    event.eventType = kEventTypeNmtEvent;
    event.eventArgSize = sizeof(nmtEvent);
    event.eventArg.pEventArg = &nmtEvent;
    ret = eventk_postEvent(&event);

Exit:
    if (ret != kErrorOk)
    {
        BENCHMARK_MOD_02_TOGGLE(7);
        arg = dllkInstance_g.dllState | (kNmtEventDllReSwitchOverTimeout << 8);
        // Error event for API layer
        ret = eventk_postError(kEventSourceDllk, ret, sizeof(arg), &arg);
    }

    TGT_DLLK_LEAVE_CRITICAL_SECTION();

    return ret;
}
#endif

#if defined(CONFIG_INCLUDE_NMT_MN)
//------------------------------------------------------------------------------
/**
\brief  Callback function for cyclic error

The function implements the callback function which is called when a cycle
error occurred.

\param[in]      errorCode_p         Error to signal.
\param[in]      pTxBuffer_p         Pointer to TxBuffer structure of transmitted frame.

\return The function returns a tOplkError error code.

\ingroup module_dllk
*/
//------------------------------------------------------------------------------
tOplkError dllk_cbCyclicError(tOplkError errorCode_p, const tEdrvTxBuffer* pTxBuffer_p)
{
    tOplkError      ret = kErrorOk;
    tNmtState       nmtState;
    UINT            handle = 0;
    UINT32          arg;
    tEventDllError  dllEvent;

    TGT_DLLK_DECLARE_FLAGS;

    TGT_DLLK_ENTER_CRITICAL_SECTION();

    nmtState = dllkInstance_g.nmtState;
    if (!NMT_IF_MN(nmtState))
    {
#if defined(CONFIG_INCLUDE_NMT_RMN)
        if (errorCode_p == kErrorEdrvCurTxListEmpty)
        {
            switch (dllkInstance_g.nmtEventGoToStandby)
            {
                case kNmtEventGoToStandby:
                    hrestimer_modifyTimer(&dllkInstance_g.timerHdlSwitchOver,
                                          (dllkInstance_g.dllConfigParam.switchOverTimeMn -
                                              dllkInstance_g.dllConfigParam.cycleLen) * 1000ULL,
                                          dllk_cbTimerSwitchOver, 0L, FALSE);
                    dllkInstance_g.nmtEventGoToStandby = kNmtEventNoEvent;
                    break;

                case kNmtEventGoToStandbyDelayed:
                    hrestimer_modifyTimer(&dllkInstance_g.timerHdlSwitchOver,
                                          (dllkInstance_g.dllConfigParam.delayedSwitchOverTimeMn -
                                              dllkInstance_g.dllConfigParam.cycleLen)  * 1000ULL,
                                          dllk_cbTimerSwitchOver, 0L, FALSE);
                    dllkInstance_g.nmtEventGoToStandby = kNmtEventNoEvent;
                    break;

                default:
                    break;
            }
        }
#endif
        // ignore errors if not MN
        goto Exit;
    }

    if (pTxBuffer_p != NULL)
        handle = (UINT)(pTxBuffer_p - dllkInstance_g.pTxBuffer);

    BENCHMARK_MOD_02_TOGGLE(7);

    switch (errorCode_p)
    {
        case kErrorEdrvCurTxListEmpty:
        case kErrorEdrvTxListNotFinishedYet:
        case kErrorEdrvNoFreeTxDesc:
            dllEvent.dllErrorEvents = DLL_ERR_MN_CYCTIMEEXCEED;
            dllEvent.nodeId = handle;
            dllEvent.nmtState = nmtState;
            dllEvent.oplkError = errorCode_p;
            ret = errhndk_postError(&dllEvent);
            break;

        default:
            arg = dllkInstance_g.dllState | (handle << 16);
            // Error event for API layer
            ret = eventk_postError(kEventSourceDllk, errorCode_p, sizeof(arg), &arg);
            break;
    }

Exit:
    TGT_DLLK_LEAVE_CRITICAL_SECTION();

    return ret;
}
#endif

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

#if (CONFIG_DLL_PROCESS_SYNC == DLL_PROCESS_SYNC_ON_TIMER)
//------------------------------------------------------------------------------
/**
\brief  CN sync timer callback function

This function is called by the timer sync module. It signals the sync event.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError cbCnTimerSync(void)
{
    tOplkError  ret;

    // trigger synchronous task
    ret = dllk_postEvent(kEventTypeSync);

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  CN Loss of Sync Timer callback function

This function is called by the timer sync module. It signals that one Sync/SoC
was lost.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError cbCnLossOfSync(void)
{
    tOplkError  ret = kErrorOk;
    tNmtState   nmtState;
    UINT32      arg;

    TGT_DLLK_DECLARE_FLAGS;

    TGT_DLLK_ENTER_CRITICAL_SECTION();

    nmtState = dllkInstance_g.nmtState;

    if (nmtState <= kNmtGsResetConfiguration)
        goto Exit;

    ret = dllkstatemachine_changeState(kNmtEventDllCeFrameTimeout, nmtState);
    if (ret != kErrorOk)
        goto Exit;

Exit:
    if (ret != kErrorOk)
    {
        BENCHMARK_MOD_02_TOGGLE(7);
        arg = dllkInstance_g.dllState | (kNmtEventDllCeFrameTimeout << 8);
        // Error event for API layer
        ret = eventk_postError(kEventSourceDllk, ret, sizeof(arg), &arg);
    }

    TGT_DLLK_LEAVE_CRITICAL_SECTION();

    return ret;
}

#if (CONFIG_DLL_PRES_CHAINING_CN != FALSE)
//------------------------------------------------------------------------------
/**
\brief  Callback function for PResFallBackTimeout

This function is called by the timer sync module. It signals that the
PResFallBackTimeout hat triggered.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError cbCnPresFallbackTimeout(void)
{
    tOplkError  ret = kErrorOk;
    tNmtState   nmtState;
    UINT32      arg;

    TGT_DLLK_DECLARE_FLAGS;
    TGT_DLLK_ENTER_CRITICAL_SECTION();

    nmtState = dllkInstance_g.nmtState;
    if (nmtState <= kNmtGsResetConfiguration)
        goto Exit;

    ret = dllkframe_presChainingDisable();

Exit:
    if (ret != kErrorOk)
    {
        BENCHMARK_MOD_02_TOGGLE(7);
        arg = dllkInstance_g.dllState | (kNmtEventDllCeFrameTimeout << 8);
        // Error event for API layer
        ret = eventk_postError(kEventSourceDllk, ret, sizeof(arg), &arg);
    }

    TGT_DLLK_LEAVE_CRITICAL_SECTION();

    return ret;
}
#endif

#endif

/// \}
