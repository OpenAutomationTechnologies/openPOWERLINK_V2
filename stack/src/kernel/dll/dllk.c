/**
********************************************************************************
\file   dllk.c

\brief  Implementation of DLL kernel module

This file contains the implementation of the DLL kernel module.

\ingroup module_dllk
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
#include <oplk/ami.h>
#include "dllk-internal.h"

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

//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
tDllkInstance               dllkInstance_g;
static tEdrvTxBuffer        aDllkTxBuffer_l[DLLK_TXFRAME_COUNT];
TGT_DLLK_DEFINE_CRITICAL_SECTION

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Add DLL kernel module instance

The function adds a DLL kernel module instance.

\param  pInitParam_p            Initialization parameters for DLL.

\return The function returns a tEplKernel error code.

\ingroup module_dllk
*/
//------------------------------------------------------------------------------
tEplKernel dllk_addInstance(tDllkInitParam* pInitParam_p)
{
    tEplKernel      ret = kEplSuccessful;
    UINT            index;
    tEdrvInitParam  EdrvInitParam;

    // reset instance structure
    EPL_MEMSET(&dllkInstance_g, 0, sizeof (dllkInstance_g));

    //jba able to work without hresk?
#if EPL_TIMER_USE_HIGHRES != FALSE
    if ((ret = hrestimer_init()) != kEplSuccessful)
        return ret;
#endif

#if (EPL_DLL_PROCESS_SYNC == EPL_DLL_PROCESS_SYNC_ON_TIMER)
    if ((ret = synctimer_addInstance()) != kEplSuccessful)
        return ret;

    if ((ret = synctimer_registerHandler(dllk_cbCnTimerSync)) != kEplSuccessful)
        return ret;

    if ((ret = synctimer_registerLossOfSyncHandler(dllk_cbCnLossOfSync)) != kEplSuccessful)
        return ret;

#if EPL_DLL_PRES_CHAINING_CN != FALSE
    if ((ret = synctimer_registerLossOfSyncHandler2(dllk_cbCnPresFallbackTimeout)) != kEplSuccessful)
        return ret;
#endif

   if ((ret = synctimer_setSyncShift(EPL_DLL_SOC_SYNC_SHIFT_US)) != kEplSuccessful)
       return ret;
#endif

    // if dynamic memory allocation available allocate instance structure

    // initialize and link pointers in instance structure to frame tables
    dllkInstance_g.pTxBuffer = aDllkTxBuffer_l;
    dllkInstance_g.maxTxFrames = sizeof (aDllkTxBuffer_l) / sizeof (tEdrvTxBuffer);
    dllkInstance_g.dllState = kDllGsInit;               // initialize state

#if EPL_NMT_MAX_NODE_ID > 0
    // set up node info structure
    for (index = 0; index < tabentries (dllkInstance_g.aNodeInfo); index++)
    {
        dllkInstance_g.aNodeInfo[index].nodeId = index + 1;
    }
#endif

    // initialize Edrv
    EPL_MEMCPY(EdrvInitParam.aMacAddr, pInitParam_p->aLocalMac, 6);
    EdrvInitParam.hwParam = pInitParam_p->hwParam;
    EdrvInitParam.pfnRxHandler = dllk_processFrameReceived;
    //    EdrvInitParam.pfnTxHandler = EplDllkCbFrameTransmitted; //jba why commented out?
    if ((ret = edrv_init(&EdrvInitParam)) != kEplSuccessful)
        return ret;

    // copy local MAC address from Ethernet driver back to local instance structure
    // because Ethernet driver may have read it from controller EEPROM
    EPL_MEMCPY(dllkInstance_g.aLocalMac, EdrvInitParam.aMacAddr, 6);
    EPL_MEMCPY(pInitParam_p->aLocalMac, EdrvInitParam.aMacAddr, 6);

    // initialize TxBuffer array
    for (index = 0; index < dllkInstance_g.maxTxFrames; index++)
    {
        dllkInstance_g.pTxBuffer[index].pBuffer = NULL;
    }

#if defined(CONFIG_INCLUDE_NMT_MN)
    if ((ret = edrvcyclic_init()) != kEplSuccessful)
        return ret;

    if ((ret = edrvcyclic_regErrorHandler(dllk_cbCyclicError)) != kEplSuccessful)
        return ret;
#endif

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Delete DLL kernel module instance

The function deletes an DLL kernel module instance.

\return The function returns a tEplKernel error code.

\ingroup module_dllk
*/
//------------------------------------------------------------------------------
tEplKernel dllk_delInstance(void)
{
    tEplKernel      ret = kEplSuccessful;

    // reset state
    dllkInstance_g.dllState = kDllGsInit;

#if defined (CONFIG_INCLUDE_NMT_MN)
    ret = edrvcyclic_shutdown();
#endif

#if (EPL_DLL_PROCESS_SYNC == EPL_DLL_PROCESS_SYNC_ON_TIMER)
    ret = synctimer_delInstance();
#endif

#if EPL_TIMER_USE_HIGHRES != FALSE
    ret = hrestimer_delInstance();
#endif

    ret = edrv_shutdown();
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Configure parameters of DLL

The function configures the parameters of the DLL. It is called before
NMT_GS_COMMUNICATING will be entered.

\param  pDllConfigParam_p       Pointer to configuration parameters.

\return The function returns a tEplKernel error code.

\ingroup module_dllk
*/
//------------------------------------------------------------------------------
tEplKernel dllk_config(tDllConfigParam * pDllConfigParam_p)
{
    tNmtState       nmtState;
    tEplKernel      ret = kEplSuccessful;

    nmtState = dllkInstance_g.nmtState;

    if (nmtState > kNmtGsResetConfiguration)
    {   // configuration updates are only allowed in state DLL_GS_INIT, except LossOfFrameTolerance,
        // because all other parameters are "valid on reset".
        dllkInstance_g.dllConfigParam.lossOfFrameTolerance = pDllConfigParam_p->lossOfFrameTolerance;

#if (EPL_DLL_PROCESS_SYNC == EPL_DLL_PROCESS_SYNC_ON_TIMER)
        ret = synctimer_setLossOfSyncTolerance(pDllConfigParam_p->lossOfFrameTolerance);
#endif
    }
    else
    {   // copy entire configuration to local storage,
        // because we are in state DLL_GS_INIT
        EPL_MEMCPY (&dllkInstance_g.dllConfigParam, pDllConfigParam_p,
            (pDllConfigParam_p->sizeOfStruct < sizeof (tDllConfigParam) ?
            pDllConfigParam_p->sizeOfStruct : sizeof (tDllConfigParam)));
    }

    if (nmtState < kNmtMsNotActive)
    {   // CN or NMT reset states are active, so we can calculate the frame timeout.
        // MN calculates on kEplEventTypeDllkCreate, its own frame timeout.
        if ((dllkInstance_g.dllConfigParam.cycleLen != 0) &&
            (dllkInstance_g.dllConfigParam.lossOfFrameTolerance != 0))
        {   // monitor EPL cycle, calculate frame timeout
            dllkInstance_g.frameTimeout = (1000LL * ((UINT64)dllkInstance_g.dllConfigParam.cycleLen)) +
                ((UINT64)dllkInstance_g.dllConfigParam.lossOfFrameTolerance);
        }
        else
        {
            dllkInstance_g.frameTimeout = 0LL;
        }
    }

    if (dllkInstance_g.dllConfigParam.fAsyncOnly != FALSE)
    {   // it is configured as async-only CN
        // disable multiplexed cycle, that cycleCount will not be incremented spuriously on SoC
        dllkInstance_g.dllConfigParam.multipleCycleCnt = 0;
    }
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Configure identity of local node for IdentResponse

The function sets the identity of the local node (may be at any time, e.g. in
case of hostname change).

\param  pDllIdentParam_p    Pointer to identity parameters.

\return The function returns a tEplKernel error code.

\ingroup module_dllk
*/
//------------------------------------------------------------------------------
tEplKernel dllk_setIdentity(tDllIdentParam * pDllIdentParam_p)
{
    EPL_MEMCPY (&dllkInstance_g.dllIdentParam, pDllIdentParam_p,
        (pDllIdentParam_p->sizeOfStruct < sizeof (tDllIdentParam) ?
        pDllIdentParam_p->sizeOfStruct : sizeof (tDllIdentParam)));

    // $$$ if IdentResponse frame exists update it
    return kEplSuccessful;
}

//------------------------------------------------------------------------------
/**
\brief  Register handler for non-POWERLINK frames

The function registers the handler for non-POWERLINK frames (used by Virtual
Ethernet driver).

\param  pfnDllkCbAsync_p    Pointer to callback function which will be called in
                            interrupt context normally.

\return The function returns a tEplKernel error code.
\retval kEplSuccessful              If handler is successfully registered.
\retval kEplDllCbAsyncRegistered    If there is already a handler registered.

\ingroup module_dllk
*/
//------------------------------------------------------------------------------
tEplKernel dllk_regAsyncHandler(tEplDllkCbAsync pfnDllkCbAsync_p)
{
    tEplKernel  ret = kEplSuccessful;

    if (dllkInstance_g.pfnCbAsync == NULL)
    {   // no handler registered yet
        dllkInstance_g.pfnCbAsync = pfnDllkCbAsync_p;
    }
    else
    {   // handler already registered
        ret = kEplDllCbAsyncRegistered;
    }
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Deregister handler for non-POWERLINK frames

The function deregisters the handler for non-POWERLINK frames (used by Virtual
Ethernet driver).

\param  pfnDllkCbAsync_p    Pointer to callback function. It will be checked if
                            this function was registered before deregistering
                            it.

\return The function returns a tEplKernel error code.
\retval kEplSuccessful              If handler is successfully deregistered.
\retval kEplDllCbAsyncRegistered    If another handler is registered.

\ingroup module_dllk
*/
//------------------------------------------------------------------------------
tEplKernel dllk_deregAsyncHandler(tEplDllkCbAsync pfnDllkCbAsync_p)
{
    tEplKernel  ret = kEplSuccessful;

    if (dllkInstance_g.pfnCbAsync == pfnDllkCbAsync_p)
    {   // same handler is registered, deregister it
        dllkInstance_g.pfnCbAsync = NULL;
    }
    else
    {   // wrong handler or no handler registered
        ret = kEplDllCbAsyncRegistered;
    }
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Register handler for sync events

The function registers the handler for SYNC events.

\param  pfnCbSync_p         Pointer to callback function for sync event. It
                            will be called in event context.

\return The function returns the previously registered sync callback function.

\ingroup module_dllk
*/
//------------------------------------------------------------------------------
tEplSyncCb dllk_regSyncHandler(tEplSyncCb pfnCbSync_p)
{
    tEplSyncCb  pfnCbOld;

    pfnCbOld = dllkInstance_g.pfnCbSync;
    dllkInstance_g.pfnCbSync = pfnCbSync_p;
    return pfnCbOld;
}

//------------------------------------------------------------------------------
/**
\brief  Register handler for RPDO frames

The function registers the handler for RPDO frames.

\param  pfnDllkCbProcessRpdo_p    Pointer to callback function. It
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

\param  pfnDllkCbProcessTpdo_p    Pointer to callback function. It
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

\param  serviceId_p         ASnd service ID.
\param  filter_p            Node ID filter.

\return The function returns a tEplKernel error code.
\retval kEplSuccessful                  If filter was successfully set.
\retval kEplDllInvalidAsndServiceId     If an invalid service ID was specified.

\ingroup module_dllk
*/
//------------------------------------------------------------------------------
tEplKernel dllk_setAsndServiceIdFilter(tDllAsndServiceId serviceId_p,
                                       tDllAsndFilter filter_p)
{
    tEplKernel  ret = kEplSuccessful;

    if (serviceId_p < tabentries (dllkInstance_g.aAsndFilter))
        dllkInstance_g.aAsndFilter[serviceId_p] = filter_p;
    else
        ret = kEplDllInvalidAsndServiceId;

    return ret;
}


#if DLL_DEFERRED_RXFRAME_RELEASE_ISOCHRONOUS != FALSE || DLL_DEFERRED_RXFRAME_RELEASE_ASYNCHRONOUS != FALSE
//------------------------------------------------------------------------------
/**
\brief  Release RX buffer frame in Edrv

The function releases the RX buffer for the specified RX frame in Edrv.

\param  pFrame_p                Pointer to frame.
\param  frameSize_p             Size of frame.

\return The function returns a tEplKernel error code.

\ingroup module_dllk
*/
//------------------------------------------------------------------------------
tEplKernel dllk_releaseRxFrame(tEplFrame* pFrame_p, UINT frameSize_p)
{
    tEplKernel      ret;
    tEdrvRxBuffer   rxBuffer;

    rxBuffer.pBuffer   = (UINT8*)pFrame_p;
    rxBuffer.rxFrameSize = frameSize_p;

    ret = edrv_releaseRxBuffer(&rxBuffer);

    return ret;
}
#endif


#if EPL_NMT_MAX_NODE_ID > 0
//------------------------------------------------------------------------------
/**
\brief  Configure the specified node

The function configures the specified node (e.g. payload limits and timeouts).

\param  pNodeInfo_p             Pointer to node information.

\return The function returns a tEplKernel error code.

\ingroup module_dllk
*/
//------------------------------------------------------------------------------
tEplKernel dllk_configNode(tDllNodeInfo * pNodeInfo_p)
{
    tEplKernel          ret = kEplSuccessful;
    tDllkNodeInfo*      pIntNodeInfo;
    tNmtState           nmtState;

    nmtState = dllkInstance_g.nmtState;

    if ((nmtState > kNmtGsResetConfiguration) &&
        (pNodeInfo_p->nodeId != dllkInstance_g.dllConfigParam.nodeId))
    {   // configuration updates are only allowed in reset states
        return kEplInvalidOperation;
    }

    pIntNodeInfo = dllk_getNodeInfo(pNodeInfo_p->nodeId);
    if (pIntNodeInfo == NULL)
    {   // no node info structure available
        return kEplDllNoNodeInfo;
    }

    // copy node configuration
    if (pNodeInfo_p->presPayloadLimit > dllkInstance_g.dllConfigParam.isochrRxMaxPayload)
        pIntNodeInfo->presPayloadLimit = (UINT16)dllkInstance_g.dllConfigParam.isochrRxMaxPayload;
    else
        pIntNodeInfo->presPayloadLimit = pNodeInfo_p->presPayloadLimit;

#if defined(CONFIG_INCLUDE_NMT_MN)
    pIntNodeInfo->presTimeoutNs = pNodeInfo_p->presTimeoutNs;
    if (pNodeInfo_p->preqPayloadLimit > dllkInstance_g.dllConfigParam.isochrTxMaxPayload)
        pIntNodeInfo->preqPayloadLimit = (UINT16)dllkInstance_g.dllConfigParam.isochrTxMaxPayload;
    else
        pIntNodeInfo->preqPayloadLimit = pNodeInfo_p->preqPayloadLimit;

    // initialize elements of internal node info structure
    pIntNodeInfo->soaFlag1 = 0;
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

\param  pNodeOpParam_p      Pointer to node operation parameters.

\return The function returns a tEplKernel error code.

\ingroup module_dllk
*/
//------------------------------------------------------------------------------
tEplKernel dllk_addNode(tDllNodeOpParam* pNodeOpParam_p)
{
    tEplKernel          ret = kEplSuccessful;
    tDllkNodeInfo*      pIntNodeInfo;
    tNmtState           nmtState;
    BOOL                fUpdateEdrv = FALSE;

    nmtState = dllkInstance_g.nmtState;

    pIntNodeInfo = dllk_getNodeInfo(pNodeOpParam_p->nodeId);
    if (pIntNodeInfo == NULL)
    {   // no node info structure available
        return kEplDllNoNodeInfo;
    }

    DLLK_DBG_POST_TRACE_VALUE(kEplEventTypeDllkAddNode, pNodeOpParam_p->nodeId, 0);

    switch (pNodeOpParam_p->opNodeType)
    {
#if defined(CONFIG_INCLUDE_NMT_MN)
        case kDllNodeOpTypeIsochronous:
            if (nmtState >= kNmtMsNotActive)
                ret = dllk_addNodeIsochronous(pIntNodeInfo);
            else
                ret = kEplDllInvalidParam;
            break;
#endif

        case kDllNodeOpTypeFilterPdo:
        case kDllNodeOpTypeFilterHeartbeat:
            if ((nmtState >= kNmtCsNotActive) && (nmtState < kNmtMsNotActive))
                fUpdateEdrv = TRUE;
            ret = dllk_addNodeFilter(pIntNodeInfo, pNodeOpParam_p->opNodeType, fUpdateEdrv);
            break;

        default:
            ret = kEplDllInvalidParam;
            break;
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Delete the specified node

The function deletes the specified node from the isochronous phase.

\param  pNodeOpParam_p      Pointer to node operation parameters.

\return The function returns a tEplKernel error code.

\ingroup module_dllk
*/
//------------------------------------------------------------------------------
tEplKernel dllk_deleteNode(tDllNodeOpParam* pNodeOpParam_p)
{
    tEplKernel          ret = kEplSuccessful;
    tDllkNodeInfo*      pIntNodeInfo;
    tNmtState           nmtState;
    BOOL                fUpdateEdrv = FALSE;
    UINT                index;

    nmtState = dllkInstance_g.nmtState;

    if (pNodeOpParam_p->nodeId == EPL_C_ADR_BROADCAST)
    {
        switch (pNodeOpParam_p->opNodeType)
        {
            case kDllNodeOpTypeFilterPdo:
            case kDllNodeOpTypeFilterHeartbeat:
                if ((nmtState >= kNmtCsNotActive) && (nmtState < kNmtMsNotActive))
                    fUpdateEdrv = TRUE;

                for (index = 0, pIntNodeInfo = &dllkInstance_g.aNodeInfo[0];
                     index < tabentries (dllkInstance_g.aNodeInfo);
                     index++, pIntNodeInfo++)
                {
                    ret = dllk_deleteNodeFilter(pIntNodeInfo, pNodeOpParam_p->opNodeType, fUpdateEdrv);
                }
                break;

            default:
                ret = kEplDllInvalidParam;
                break;
        }
        return ret;
    }

    pIntNodeInfo = dllk_getNodeInfo(pNodeOpParam_p->nodeId);
    if (pIntNodeInfo == NULL)
    {   // no node info structure available
        return kEplDllNoNodeInfo;
    }

    DLLK_DBG_POST_TRACE_VALUE(kEplEventTypeDllkDelNode, pNodeOpParam_p->nodeId, 0);

    switch (pNodeOpParam_p->opNodeType)
    {
#if defined(CONFIG_INCLUDE_NMT_MN)
        case kDllNodeOpTypeIsochronous:
            if (nmtState >= kNmtMsNotActive)
                ret = dllk_deleteNodeIsochronous(pIntNodeInfo);
            else
                ret = kEplDllInvalidParam;
            break;

        case kDllNodeOpTypeSoftDelete:
            pIntNodeInfo->fSoftDelete = TRUE;
            break;
#endif

        case kDllNodeOpTypeFilterPdo:
        case kDllNodeOpTypeFilterHeartbeat:
            if ((nmtState >= kNmtCsNotActive) && (nmtState < kNmtMsNotActive))
                fUpdateEdrv = TRUE;
            ret = dllk_deleteNodeFilter(pIntNodeInfo, pNodeOpParam_p->opNodeType, fUpdateEdrv);
            break;

        default:
            ret = kEplDllInvalidParam;
            break;
    }

    return ret;
}

#endif // EPL_NMT_MAX_NODE_ID > 0

#if defined(CONFIG_INCLUDE_NMT_MN)
//------------------------------------------------------------------------------
/**
\brief  Set Flag1 of the specified node

The function sets Flag1 (for PReq and SoA) of the specified node.

\param  nodeId_p            Node ID to set the flag.
\param  soaFlag1_p          Flag1.

\return The function returns a tEplKernel error code.
\retval kEplSuccessful          If flag is successfully set.
\retval kEplDllNoNodeInfo       If node is not found.

\ingroup module_dllk
*/
//------------------------------------------------------------------------------
tEplKernel dllk_setFlag1OfNode(UINT nodeId_p, UINT8 soaFlag1_p)
{
    tDllkNodeInfo*   pNodeInfo;

    pNodeInfo = dllk_getNodeInfo(nodeId_p);
    if (pNodeInfo == NULL)
    {   // no node info structure available
        return kEplDllNoNodeInfo;
    }
    // store flag1 in internal node info structure
    pNodeInfo->soaFlag1 = soaFlag1_p;

    return kEplSuccessful;
}

//------------------------------------------------------------------------------
/**
\brief  Get current CN node list

The function returns the first info structure of first node in isochronous phase.
It is only useful for kernel error handler module.

\param  ppbCnNodeIdList_p       Pointer to array of bytes with node-ID list.
                                Array is terminated by value 0.

\ingroup module_dllk
*/
//------------------------------------------------------------------------------
void dllk_getCurrentCnNodeIdList(BYTE** ppbCnNodeIdList_p)
{
    *ppbCnNodeIdList_p = &dllkInstance_g.aCnNodeIdList[dllkInstance_g.curTxBufferOffsetCycle ^ 1][0];
}

#if (EPL_DLL_PRES_CHAINING_MN == TRUE)
//------------------------------------------------------------------------------
/**
\brief  Get MAC address of the specified node

The function returns the MAC address of the specified node.

\param  nodeId_p                Node ID from which to get MAC address.
\param  pCnMacAddress_p         Pointer to store MAC address.

\return The function returns a tEplKernel error code.
\retval kEplSuccessful          If MAC address is successfully read.
\retval kEplDllNoNodeInfo       If node is not found.

\ingroup module_dllk
*/
//------------------------------------------------------------------------------
tEplKernel dllk_getCnMacAddress(UINT nodeId_p, BYTE* pCnMacAddress_p)
{
    tDllkNodeInfo*   pNodeInfo;

    pNodeInfo = dllk_getNodeInfo(nodeId_p);
    if (pNodeInfo == NULL)
    {   // no node info structure available
        return kEplDllNoNodeInfo;
    }

    EPL_MEMCPY(pCnMacAddress_p, pNodeInfo->aMacAddr, 6);
    return kEplSuccessful;
}
#endif

#endif

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

#if defined CONFIG_INCLUDE_NMT_MN
//------------------------------------------------------------------------------
/**
\brief  MN timer callback function

This function is called by the timer module. It triggers the SoC for a MN.

\param  pEventArg_p         Pointer to timer event argument.

\return The function returns a pointer to the node Information of the node
*/
//------------------------------------------------------------------------------
tEplKernel dllk_cbMnTimerCycle(tEplTimerEventArg* pEventArg_p)
{
    tEplKernel      ret = kEplSuccessful;
    tNmtState       nmtState;
    UINT32          arg;

    TGT_DLLK_DECLARE_FLAGS;

    TGT_DLLK_ENTER_CRITICAL_SECTION();

#if EPL_TIMER_USE_HIGHRES != FALSE
    if (pEventArg_p->m_TimerHdl != dllkInstance_g.timerHdlCycle)
    {   // zombie callback - just exit
        goto Exit;
    }
#endif

    nmtState = dllkInstance_g.nmtState;
    if (nmtState <= kNmtGsResetConfiguration)
        goto Exit;

    ret = dllk_changeState(kNmtEventDllMeSocTrig, nmtState);

Exit:
    if (ret != kEplSuccessful)
    {
        BENCHMARK_MOD_02_TOGGLE(7);
        arg = dllkInstance_g.dllState | (kNmtEventDllMeSocTrig << 8);
        // Error event for API layer
        ret = eventk_postError(kEplEventSourceDllk, ret, sizeof(arg), &arg);
    }
    TGT_DLLK_LEAVE_CRITICAL_SECTION();
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Callback function for cyclic error

The function implements the callback function which is called when a cycle
error occurred.

\param  errorCode_p         Error to signal.
\param  pTxBuffer_p         Pointer to TxBuffer structure of transmitted frame.

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
tEplKernel dllk_cbCyclicError(tEplKernel errorCode_p, tEdrvTxBuffer * pTxBuffer_p)
{
    tEplKernel      ret = kEplSuccessful;
    tNmtState       nmtState;
    UINT            handle = 0;
    UINT32          arg;
    tErrHndkEvent   dllEvent;

    TGT_DLLK_DECLARE_FLAGS;

    UNUSED_PARAMETER(pTxBuffer_p);

    TGT_DLLK_ENTER_CRITICAL_SECTION();

    nmtState = dllkInstance_g.nmtState;
    if (nmtState <= kNmtGsResetConfiguration)
        goto Exit;

    if (pTxBuffer_p != NULL)
    {
        handle = (UINT) (pTxBuffer_p - dllkInstance_g.pTxBuffer);
    }

    BENCHMARK_MOD_02_TOGGLE(7);

    switch (errorCode_p)
    {
        case kEplEdrvCurTxListEmpty:
        case kEplEdrvTxListNotFinishedYet:
        case kEplEdrvNoFreeTxDesc:
            dllEvent.m_ulDllErrorEvents = EPL_DLL_ERR_MN_CYCTIMEEXCEED;
            dllEvent.m_uiNodeId = handle;
            dllEvent.m_NmtState = nmtState;
            dllEvent.m_EplError = errorCode_p;
            ret = errhndk_postError(&dllEvent);
            break;

        default:
            arg = dllkInstance_g.dllState | (handle << 16);
            // Error event for API layer
            ret = eventk_postError(kEplEventSourceDllk, errorCode_p, sizeof(arg), &arg);
            break;
    }

Exit:
    TGT_DLLK_LEAVE_CRITICAL_SECTION();

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  MN sync callback function

This function is called by the ethernet driver module. It triggers the sync
event.

\return The function returns a pointer to the node Information of the node
*/
//------------------------------------------------------------------------------
tEplKernel dllk_cbMnSyncHandler(void)
{
    tEplKernel      ret = kEplSuccessful;
    tNmtState       nmtState;
    BYTE*           pbCnNodeId;
    UINT32          arg;

    TGT_DLLK_DECLARE_FLAGS;

    TGT_DLLK_ENTER_CRITICAL_SECTION();

    nmtState = dllkInstance_g.nmtState;
    if (nmtState <= kNmtGsResetConfiguration)
        goto Exit;

    // do cycle finish which has to be done inside the callback function triggered by interrupt
    pbCnNodeId = &dllkInstance_g.aCnNodeIdList[dllkInstance_g.curTxBufferOffsetCycle][dllkInstance_g.curNodeIndex];

    while (*pbCnNodeId != EPL_C_ADR_INVALID)
    {   // issue error for each CN in list which was not processed yet, i.e. PRes received
        ret = dllk_issueLossOfPres(*pbCnNodeId);
        if (ret != kEplSuccessful)
            goto Exit;
        pbCnNodeId++;
    }

    dllkInstance_g.fSyncProcessed = FALSE;
#if EPL_DLL_PRES_CHAINING_MN != FALSE
    dllkInstance_g.fPrcSlotFinished = FALSE;
#endif

    // switch to next cycle
    dllkInstance_g.curTxBufferOffsetCycle ^= 1;
    dllkInstance_g.curNodeIndex = 0;

    ret = dllk_postEvent(kEplEventTypeDllkCycleFinish);

Exit:
    if (ret != kEplSuccessful)
    {
        BENCHMARK_MOD_02_TOGGLE(7);
        arg = dllkInstance_g.dllState | (kNmtEventDllMeSocTrig << 8);
        // Error event for API layer
        ret = eventk_postError(kEplEventSourceDllk, ret, sizeof(arg), &arg);
    }
    TGT_DLLK_LEAVE_CRITICAL_SECTION();
    return ret;
}
#endif

#if EPL_TIMER_USE_HIGHRES != FALSE
//------------------------------------------------------------------------------
/**
\brief  CN Timer callback function

This function is called by the timer module. It monitors the POWERLINK cycle
when it is running as CN node.

\param  pEventArg_p         Pointer to timer event argument.

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
tEplKernel dllk_cbCnTimer(tEplTimerEventArg* pEventArg_p)
{
    tEplKernel      ret = kEplSuccessful;
    tNmtState       nmtState;
    UINT32          arg;

    TGT_DLLK_DECLARE_FLAGS;

    TGT_DLLK_ENTER_CRITICAL_SECTION();

    if (pEventArg_p->m_TimerHdl != dllkInstance_g.timerHdlCycle)
    {   // zombie callback - just exit
        goto Exit;
    }

    nmtState = dllkInstance_g.nmtState;
    if (nmtState <= kNmtGsResetConfiguration)
        goto Exit;

    ret = dllk_changeState(kNmtEventDllCeFrameTimeout, nmtState);
    if (ret != kEplSuccessful)
        goto Exit;

    // restart the timer to detect further loss of SoC
    ret = hrestimer_modifyTimer(&dllkInstance_g.timerHdlCycle,
               dllkInstance_g.dllConfigParam.cycleLen, dllk_cbCnTimer, 0L, FALSE);
    if (ret != kEplSuccessful)
        goto Exit;

Exit:
    if (ret != kEplSuccessful)
    {
        BENCHMARK_MOD_02_TOGGLE(7);
        arg = dllkInstance_g.dllState | (kNmtEventDllCeFrameTimeout << 8);
        // Error event for API layer
        ret = eventk_postError(kEplEventSourceDllk, ret, sizeof(arg), &arg);
    }
    TGT_DLLK_LEAVE_CRITICAL_SECTION();
    return ret;
}
#endif

#if (EPL_DLL_PROCESS_SYNC == EPL_DLL_PROCESS_SYNC_ON_TIMER)
//------------------------------------------------------------------------------
/**
\brief  CN Sync Timer callback function

This function is called by the timer sync module. It signals the sync event.

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
tEplKernel dllk_cbCnTimerSync(void)
{
    tEplKernel      ret = kEplSuccessful;

    // trigger synchronous task
    ret = dllk_postEvent(kEplEventTypeSync);
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  CN Loss of Sync Timer callback function

This function is called by the timer sync module. It signals that one Sync/SoC
was lost.

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
tEplKernel dllk_cbCnLossOfSync(void)
{
    tEplKernel      ret = kEplSuccessful;
    tNmtState       nmtState;
    UINT32          arg;

    TGT_DLLK_DECLARE_FLAGS;

    TGT_DLLK_ENTER_CRITICAL_SECTION();

    nmtState = dllkInstance_g.nmtState;

    if (nmtState <= kNmtGsResetConfiguration)
        goto Exit;

    ret = dllk_changeState(kNmtEventDllCeFrameTimeout, nmtState);
    if (ret != kEplSuccessful)
        goto Exit;

Exit:
    if (ret != kEplSuccessful)
    {
        BENCHMARK_MOD_02_TOGGLE(7);
        arg = dllkInstance_g.dllState | (kNmtEventDllCeFrameTimeout << 8);
        // Error event for API layer
        ret = eventk_postError(kEplEventSourceDllk, ret, sizeof(arg), &arg);
    }
    TGT_DLLK_LEAVE_CRITICAL_SECTION();
    return ret;
}

#endif

//------------------------------------------------------------------------------
/**
\brief  Setup the local node

The function is called if the node enters the "not active" state from a reset
state and initializes all stuff that is needed for operation.

\param  nmtState_p              NMT state of the node. Could be kNmtMsNotActive
                                or kNmtCsNotActive

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
tEplKernel dllk_setupLocalNode(tNmtState nmtState_p)
{
    tEplKernel      ret = kEplSuccessful;
    UINT            handle;
    UINT            frameSize;
    UINT8           aMulticastMac[6];

#if !defined(CONFIG_INCLUDE_NMT_MN)
    UNUSED_PARAMETER(nmtState_p);
#endif

    // initialize flags for PRes and StatusRes (leave Flag 1 unchanged)
    dllkInstance_g.mnFlag1 = 0;
    dllkInstance_g.flag2 = 0;

#if defined(CONFIG_INCLUDE_NMT_MN)
    // initialize linked node list
    dllkInstance_g.pFirstNodeInfo = NULL;
#if EPL_DLL_PRES_CHAINING_MN != FALSE
    dllkInstance_g.pFirstPrcNodeInfo = NULL;
#endif
#endif

    /*-----------------------------------------------------------------------*/
    /* register TxFrames in Edrv */
    // IdentResponse
    frameSize = EPL_C_DLL_MINSIZE_IDENTRES;
    ret = dllk_createTxFrame(&handle, &frameSize, kEplMsgTypeAsnd, kDllAsndIdentResponse);
    if (ret != kEplSuccessful)
        return ret;

    // StatusResponse
    frameSize = EPL_C_DLL_MINSIZE_STATUSRES;
    ret = dllk_createTxFrame(&handle, &frameSize, kEplMsgTypeAsnd, kDllAsndStatusResponse);
    if (ret != kEplSuccessful)
        return ret;

#if EPL_DLL_PRES_CHAINING_CN != FALSE
    // SyncResponse
    frameSize = EPL_C_DLL_MINSIZE_SYNCRES;
    ret = dllk_createTxFrame(&handle, &frameSize, kEplMsgTypeAsnd, kDllAsndSyncResponse);
    if (ret != kEplSuccessful)
        return ret;
#endif

    // PRes
    if ((dllkInstance_g.dllConfigParam.fAsyncOnly == FALSE) &&
        (dllkInstance_g.dllConfigParam.presActPayloadLimit >= 36))
    {   // it is not configured as async-only CN,
        // so take part in isochronous phase and register PRes frame
        frameSize = dllkInstance_g.dllConfigParam.presActPayloadLimit + EPL_FRAME_OFFSET_PDO_PAYLOAD;
        ret = dllk_createTxFrame(&handle, &frameSize, kEplMsgTypePres, kDllAsndNotDefined);
        if (ret != kEplSuccessful)
            return ret;

        // reset cycle counter
        dllkInstance_g.cycleCount = 0;
    }
    else
    {   // it is an async-only CN -> fool changeState() to think that PRes was not expected
        dllkInstance_g.cycleCount = 1;
    }

    // NMT request
    frameSize = EPL_C_IP_MAX_MTU;
    ret = dllk_createTxFrame(&handle, &frameSize, kEplMsgTypeAsnd, kDllAsndNmtRequest);
    if (ret != kEplSuccessful)
        return ret;
    // mark Tx buffer as empty
    dllkInstance_g.pTxBuffer[handle].txFrameSize = DLLK_BUFLEN_EMPTY;
    dllkInstance_g.pTxBuffer[handle].pfnTxHandler = dllk_processTransmittedNmtReq;
    handle++;
    dllkInstance_g.pTxBuffer[handle].txFrameSize = DLLK_BUFLEN_EMPTY;
    dllkInstance_g.pTxBuffer[handle].pfnTxHandler = dllk_processTransmittedNmtReq;

    // non-EPL frame
    frameSize = EPL_C_IP_MAX_MTU;
    ret = dllk_createTxFrame(&handle, &frameSize, kEplMsgTypeNonEpl, kDllAsndNotDefined);
    if (ret != kEplSuccessful)
        return ret;
    // mark Tx buffer as empty
    dllkInstance_g.pTxBuffer[handle].txFrameSize = DLLK_BUFLEN_EMPTY;
    dllkInstance_g.pTxBuffer[handle].pfnTxHandler = dllk_processTransmittedNonEpl;
    handle++;
    dllkInstance_g.pTxBuffer[handle].txFrameSize = DLLK_BUFLEN_EMPTY;
    dllkInstance_g.pTxBuffer[handle].pfnTxHandler = dllk_processTransmittedNonEpl;

    /*------------------------------------------------------------------------*/
    /* setup filter structure for Edrv */
    EPL_MEMSET(dllkInstance_g.aFilter, 0, sizeof (dllkInstance_g.aFilter));
    dllk_setupAsndFilter(&dllkInstance_g.aFilter[DLLK_FILTER_ASND]);
    dllk_setupSocFilter(&dllkInstance_g.aFilter[DLLK_FILTER_SOC]);
    dllk_setupSoaFilter(&dllkInstance_g.aFilter[DLLK_FILTER_SOA]);
    dllk_setupSoaIdentReqFilter(&dllkInstance_g.aFilter[DLLK_FILTER_SOA_IDREQ],
                                dllkInstance_g.dllConfigParam.nodeId,
                                &dllkInstance_g.pTxBuffer[DLLK_TXFRAME_IDENTRES]);
    dllk_setupSoaStatusReqFilter(&dllkInstance_g.aFilter[DLLK_FILTER_SOA_STATREQ],
                                 dllkInstance_g.dllConfigParam.nodeId,
                                 &dllkInstance_g.pTxBuffer[DLLK_TXFRAME_STATUSRES]);
    dllk_setupSoaNmtReqFilter(&dllkInstance_g.aFilter[DLLK_FILTER_SOA_NMTREQ],
                                 dllkInstance_g.dllConfigParam.nodeId,
                                 &dllkInstance_g.pTxBuffer[DLLK_TXFRAME_NMTREQ]);
#if EPL_DLL_PRES_CHAINING_CN != FALSE
    dllk_setupSoaSyncReqFilter(&dllkInstance_g.aFilter[DLLK_FILTER_SOA_SYNCREQ],
                                 dllkInstance_g.dllConfigParam.nodeId,
                                 &dllkInstance_g.pTxBuffer[DLLK_TXFRAME_SYNCRES]);
#endif
    dllk_setupSoaUnspecReqFilter(&dllkInstance_g.aFilter[DLLK_FILTER_SOA_NONEPL],
                                 dllkInstance_g.dllConfigParam.nodeId,
                                 &dllkInstance_g.pTxBuffer[DLLK_TXFRAME_NONEPL]);

    // register multicast MACs in ethernet driver
    ami_setUint48Be(&aMulticastMac[0], EPL_C_DLL_MULTICAST_SOC);
    ret = edrv_setRxMulticastMacAddr(aMulticastMac);
    ami_setUint48Be(&aMulticastMac[0], EPL_C_DLL_MULTICAST_SOA);
    ret = edrv_setRxMulticastMacAddr(aMulticastMac);
    ami_setUint48Be(&aMulticastMac[0], EPL_C_DLL_MULTICAST_PRES);
    ret = edrv_setRxMulticastMacAddr(aMulticastMac);
    ami_setUint48Be(&aMulticastMac[0], EPL_C_DLL_MULTICAST_ASND);
    ret = edrv_setRxMulticastMacAddr(aMulticastMac);

#if defined(CONFIG_INCLUDE_NMT_MN)
    if (nmtState_p >= kNmtMsNotActive)
    {
        if ((ret = dllk_setupLocalNodeMn()) != kEplSuccessful)
            return ret;
    }
    else
    {
        if ((ret = dllk_setupLocalNodeCn()) != kEplSuccessful)
            return ret;
    }
#else
    if ((ret = dllk_setupLocalNodeCn()) != kEplSuccessful)
        return ret;
#endif

    // clear all asynchronous buffers
    ret = dllkcal_clearAsyncBuffer();
    if (ret != kEplSuccessful)
        return ret;

    // set filters in Edrv
    ret = edrv_changeRxFilter(dllkInstance_g.aFilter, DLLK_FILTER_COUNT, DLLK_FILTER_COUNT, 0);

    return ret;
}

#if defined(CONFIG_INCLUDE_NMT_MN)
//------------------------------------------------------------------------------
/**
\brief   Setup MN specific stuff of local node

The function initializes the MN specific stuff of the local node.

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
tEplKernel dllk_setupLocalNodeMn(void)
{
    tEplKernel      ret = kEplSuccessful;
    UINT            handle;
    UINT            index;
    UINT            frameSize;
    UINT            count = 0;
    tDllkNodeInfo*  pIntNodeInfo;

    /*-------------------------------------------------------------------*/
    /* register TxFrames in Edrv */
    // SoC
    frameSize = EPL_C_DLL_MINSIZE_SOC;
    ret = dllk_createTxFrame(&handle, &frameSize, kEplMsgTypeSoc, kDllAsndNotDefined);
    if (ret != kEplSuccessful)
    {   // error occurred while registering Tx frame
        return ret;
    }
    dllkInstance_g.pTxBuffer[handle].pfnTxHandler = dllk_processTransmittedSoc;
    handle++;
    dllkInstance_g.pTxBuffer[handle].pfnTxHandler = dllk_processTransmittedSoc;

    // SoA
    frameSize = EPL_C_DLL_MINSIZE_SOA;
    ret = dllk_createTxFrame(&handle, &frameSize, kEplMsgTypeSoa, kDllAsndNotDefined);
    if (ret != kEplSuccessful)
    {   // error occurred while registering Tx frame
        return ret;
    }
    dllkInstance_g.pTxBuffer[handle].pfnTxHandler = dllk_processTransmittedSoa;
    handle++;
    dllkInstance_g.pTxBuffer[handle].pfnTxHandler = dllk_processTransmittedSoa;

    for (index = 0, pIntNodeInfo = &dllkInstance_g.aNodeInfo[0];
         index < tabentries (dllkInstance_g.aNodeInfo);
         index++, pIntNodeInfo++)
    {
        if (pIntNodeInfo->preqPayloadLimit > 0)
        {   // create PReq frame for this node
            count++;

            frameSize = pIntNodeInfo->preqPayloadLimit + EPL_FRAME_OFFSET_PDO_PAYLOAD;
            ret = dllk_createTxFrame(&handle, &frameSize, kEplMsgTypePreq, kDllAsndNotDefined);
            if (ret != kEplSuccessful)
                return ret;
            pIntNodeInfo->pPreqTxBuffer = &dllkInstance_g.pTxBuffer[handle];
        }
    }

    // alloc TxBuffer pointer list
    count += 5;   // SoC, PResMN, SoA, ASnd, NULL
    dllkInstance_g.ppTxBufferList = EPL_MALLOC(sizeof (tEdrvTxBuffer*) * count);
    if (dllkInstance_g.ppTxBufferList == NULL)
        return kEplDllOutOfMemory;

    ret = edrvcyclic_setMaxTxBufferListSize(count);
    if (ret != kEplSuccessful)
        return ret;

    ret = edrvcyclic_setCycleTime(dllkInstance_g.dllConfigParam.cycleLen);
    if (ret != kEplSuccessful)
        return ret;

    ret = edrvcyclic_regSyncHandler(dllk_cbMnSyncHandler);
    if (ret != kEplSuccessful)
        return ret;

    dllkInstance_g.frameTimeout = 1000LL * ((UINT64)dllkInstance_g.dllConfigParam.cycleLen);

    dllk_setupPresFilter(&dllkInstance_g.aFilter[DLLK_FILTER_PRES], TRUE);

    return ret;
}
#endif

//------------------------------------------------------------------------------
/**
\brief   Setup MN specific stuff of local node

The function initializes the MN specific stuff of the local node.

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
tEplKernel dllk_setupLocalNodeCn(void)
{
    tEplKernel      ret = kEplSuccessful;

#if (EPL_DLL_PRES_FILTER_COUNT >= 0)
    UINT            handle;

#if (EPL_NMT_MAX_NODE_ID > 0)
    UINT            index;
    tDllkNodeInfo*  pIntNodeInfo;
#endif
#endif

    dllk_setupPreqFilter(&dllkInstance_g.aFilter[DLLK_FILTER_PREQ],
                         dllkInstance_g.dllConfigParam.nodeId,
                         &dllkInstance_g.pTxBuffer[DLLK_TXFRAME_PRES],
                         &dllkInstance_g.aLocalMac[0]);

    // setup PRes filter
#if EPL_DLL_PRES_FILTER_COUNT < 0
    if (dllkInstance_g.usedPresFilterCount > 0)
        dllk_setupPresFilter(&dllkInstance_g.aFilter[DLLK_FILTER_PRES], TRUE);
    else
        dllk_setupPresFilter(&dllkInstance_g.aFilter[DLLK_FILTER_PRES], FALSE);

#else
    for (handle = DLLK_FILTER_PRES; handle < DLLK_FILTER_COUNT; handle++)
    {
        dllk_setupPresFilter(&dllkInstance_g.aFilter[handle], FALSE);
        ami_setUint8Be(&dllkInstance_g.aFilter[handle].aFilterMask[16], 0xFF);
    }

    handle = DLLK_FILTER_PRES;
#if EPL_NMT_MAX_NODE_ID > 0
    for (index = 0, pIntNodeInfo = &dllkInstance_g.aNodeInfo[0];
         index < tabentries (dllkInstance_g.aNodeInfo);
         index++, pIntNodeInfo++)
    {
        if ((pIntNodeInfo->presFilterFlags & (DLLK_FILTER_FLAG_PDO | DLLK_FILTER_FLAG_HB)) != 0)
        {
            ami_setUint8Be(&dllkInstance_g.aFilter[handle].aFilterValue[16],
                           pIntNodeInfo->nodeId);
            dllkInstance_g.aFilter[handle].fEnable = TRUE;
            handle++;
            if (handle >= DLLK_FILTER_COUNT)
                break;
        }
    }
#endif
#endif

#if (EPL_DLL_PROCESS_SYNC == EPL_DLL_PROCESS_SYNC_ON_TIMER)
    ret = synctimer_setCycleLen(dllkInstance_g.dllConfigParam.cycleLen);
    if (ret != kEplSuccessful)
        return ret;

    ret = synctimer_setLossOfSyncTolerance(dllkInstance_g.dllConfigParam.lossOfFrameTolerance);
    if (ret != kEplSuccessful)
        return ret;
#endif

#if EPL_DLL_PRES_CHAINING_CN != FALSE
    dllkInstance_g.fPrcEnabled = FALSE;
    dllkInstance_g.syncReqPrevNodeId = 0;
#endif

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Cleanup the local node

The function is called if the node falls back into a reset state. It cleans
up all stuff for the local node.

\param  oldNmtState_p           Previous NMT state of the local node.

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
tEplKernel dllk_cleanupLocalNode(tNmtState oldNmtState_p)
{
    tEplKernel      ret = kEplSuccessful;
    BYTE            aMulticastMac[6];
#if EPL_NMT_MAX_NODE_ID > 0
    UINT            index;
#endif

#if defined(CONFIG_INCLUDE_NMT_MN)
    UINT            handle;
#else
    UNUSED_PARAMETER(oldNmtState_p);
#endif

    // remove all filters from Edrv
    ret = edrv_changeRxFilter(NULL, 0, 0, 0);

#if defined(CONFIG_INCLUDE_NMT_MN)
    // destroy all data structures
    EPL_FREE(dllkInstance_g.ppTxBufferList);
    dllkInstance_g.ppTxBufferList = NULL;
#endif

    // delete timer
#if EPL_TIMER_USE_HIGHRES != FALSE
    if ((ret = hrestimer_deleteTimer(&dllkInstance_g.timerHdlCycle)) != kEplSuccessful)
        return ret;
#endif

#if defined(CONFIG_INCLUDE_NMT_MN)
    if ((ret = edrvcyclic_stopCycle()) != kEplSuccessful)
        return ret;

    if ((ret = edrvcyclic_regSyncHandler(NULL)) != kEplSuccessful)
        return ret;
#endif

#if (EPL_DLL_PROCESS_SYNC == EPL_DLL_PROCESS_SYNC_ON_TIMER)
    if ((ret = synctimer_stopSync()) != kEplSuccessful)
        return ret;
#endif

    // delete Tx frames
    if ((ret = dllk_deleteTxFrame(DLLK_TXFRAME_IDENTRES)) != kEplSuccessful)
        return ret;

    if ((ret = dllk_deleteTxFrame(DLLK_TXFRAME_STATUSRES)) != kEplSuccessful)
        return ret;

    if ((ret = dllk_deleteTxFrame(DLLK_TXFRAME_PRES)) != kEplSuccessful)
        return ret;

    if ((ret = dllk_deleteTxFrame(DLLK_TXFRAME_NMTREQ)) != kEplSuccessful)
        return ret;

#if EPL_DLL_PRES_CHAINING_CN != FALSE
    if ((ret = dllk_deleteTxFrame(DLLK_TXFRAME_SYNCRES)) != kEplSuccessful)
        return ret;
#endif

    if ((ret = dllk_deleteTxFrame(DLLK_TXFRAME_NONEPL)) != kEplSuccessful)
        return ret;

#if defined(CONFIG_INCLUDE_NMT_MN)
    if (oldNmtState_p >= kNmtMsNotActive)
    {   // local node was MN
        if ((ret = dllk_deleteTxFrame(DLLK_TXFRAME_SOC)) != kEplSuccessful)
            return ret;

        if ((ret = dllk_deleteTxFrame(DLLK_TXFRAME_SOA)) != kEplSuccessful)
            return ret;

        for (index = 0; index < tabentries (dllkInstance_g.aNodeInfo); index++)
        {
            if (dllkInstance_g.aNodeInfo[index].pPreqTxBuffer != NULL)
            {
                handle = (UINT) (dllkInstance_g.aNodeInfo[index].pPreqTxBuffer - dllkInstance_g.pTxBuffer);
                dllkInstance_g.aNodeInfo[index].pPreqTxBuffer = NULL;
                if (handle != DLLK_TXFRAME_PRES)
                {
                    if ((ret = dllk_deleteTxFrame(handle))  != kEplSuccessful)
                        return ret;
                }
            }
            // disable PReq and PRes for this node
            dllkInstance_g.aNodeInfo[index].preqPayloadLimit = 0;
            dllkInstance_g.aNodeInfo[index].presPayloadLimit = 0;
        }
    }
    else
    {   // local node was CN
        for (index = 0; index < tabentries (dllkInstance_g.aNodeInfo); index++)
        {
            // disable PReq and PRes for this node
            dllkInstance_g.aNodeInfo[index].presPayloadLimit = 0;
            dllkInstance_g.aNodeInfo[index].preqPayloadLimit = 0;
        }
    }
#else
    // must be CN because MN part is not compiled!
#if EPL_NMT_MAX_NODE_ID > 0
    for (index = 0; index < tabentries (dllkInstance_g.aNodeInfo); index++)
    {
        // disable PRes for this node
        dllkInstance_g.aNodeInfo[index].presPayloadLimit = 0;
    }
#endif

#endif

    // deregister multicast MACs in ethernet driver
    ami_setUint48Be(&aMulticastMac[0], EPL_C_DLL_MULTICAST_SOC);
    ret = edrv_clearRxMulticastMacAddr(aMulticastMac);
    ami_setUint48Be(&aMulticastMac[0], EPL_C_DLL_MULTICAST_SOA);
    ret = edrv_clearRxMulticastMacAddr(aMulticastMac);
    ami_setUint48Be(&aMulticastMac[0], EPL_C_DLL_MULTICAST_PRES);
    ret = edrv_clearRxMulticastMacAddr(aMulticastMac);
    ami_setUint48Be(&aMulticastMac[0], EPL_C_DLL_MULTICAST_ASND);
    ret = edrv_clearRxMulticastMacAddr(aMulticastMac);

    return ret;
}

#if EPL_NMT_MAX_NODE_ID > 0
//------------------------------------------------------------------------------
/**
\brief  Get node info of specified node

This function returns the node info structure of the specified node.

\param  nodeId_p            Node ID of node for which to get info.

\return The function returns a pointer to the node Information of the node
*/
//------------------------------------------------------------------------------
tDllkNodeInfo* dllk_getNodeInfo(UINT nodeId_p)
{
    // $$$ d.k.: use hash algorithm to retrieve the appropriate node info structure
    //           if size of array is less than 254.
    nodeId_p--;   // node ID starts at 1 but array at 0
    if (nodeId_p >= tabentries (dllkInstance_g.aNodeInfo))
        return NULL;
    else
        return &dllkInstance_g.aNodeInfo[nodeId_p];
}
#endif // EPL_NMT_MAX_NODE_ID > 0


#if defined(CONFIG_INCLUDE_NMT_MN)
//------------------------------------------------------------------------------
/**
\brief  Add node to the isochronous phase

This function adds a node to the isochronous phase.

\param  pIntNodeInfo_p      Pointer to internal node info structure.

\return The function returns a pointer to the node Information of the node
*/
//------------------------------------------------------------------------------
tEplKernel dllk_addNodeIsochronous(tDllkNodeInfo* pIntNodeInfo_p)
{
    tEplKernel          ret = kEplSuccessful;
    tDllkNodeInfo**     ppIntNodeInfo;
    tEplFrame*          pTxFrame;

    if (pIntNodeInfo_p->nodeId == dllkInstance_g.dllConfigParam.nodeId)
    {   // we shall send PRes ourself
        // insert our node as first entry in the list
        ppIntNodeInfo = &dllkInstance_g.pFirstNodeInfo;
        if (*ppIntNodeInfo != NULL)
        {
            if ((*ppIntNodeInfo)->nodeId == pIntNodeInfo_p->nodeId)
            {   // node was already added to list
                // $$$ d.k. maybe this should be an error
                goto Exit;
            }
        }
        // set "PReq"-TxBuffer to PRes-TxBuffer
        pIntNodeInfo_p->pPreqTxBuffer = &dllkInstance_g.pTxBuffer[DLLK_TXFRAME_PRES];

        // Reset PRC Slot Timeout
        // which is required if falling back to PreOp1
        pIntNodeInfo_p->presTimeoutNs = 0;
    }
    else
    {   // normal CN shall be added to isochronous phase
        // insert node into list in ascending order
#if EPL_DLL_PRES_CHAINING_MN != FALSE
        if (pIntNodeInfo_p->pPreqTxBuffer == NULL)
        {
            ppIntNodeInfo = &dllkInstance_g.pFirstPrcNodeInfo;
        }
        else
#endif
        {
            ppIntNodeInfo = &dllkInstance_g.pFirstNodeInfo;
        }

        while ((*ppIntNodeInfo != NULL) &&
               (((*ppIntNodeInfo)->nodeId < pIntNodeInfo_p->nodeId) ||
               ((*ppIntNodeInfo)->nodeId == dllkInstance_g.dllConfigParam.nodeId)))
        {
            ppIntNodeInfo = &(*ppIntNodeInfo)->pNextNodeInfo;
        }

        if ((*ppIntNodeInfo != NULL) && ((*ppIntNodeInfo)->nodeId == pIntNodeInfo_p->nodeId))
        {   // node was already added to list
            // $$$ d.k. maybe this should be an error
            goto Exit;
        }

        if (pIntNodeInfo_p->pPreqTxBuffer != NULL)
        {   // TxBuffer entry exists
            tEplEvent       event;

            pTxFrame = (tEplFrame *) pIntNodeInfo_p->pPreqTxBuffer[0].pBuffer;
            // set up destination MAC address
            EPL_MEMCPY(pTxFrame->m_be_abDstMac, pIntNodeInfo_p->aMacAddr, 6);
            // set destination node-ID in PReq
            ami_setUint8Le(&pTxFrame->m_le_bDstNodeId, (UINT8)pIntNodeInfo_p->nodeId);
            // do the same for second frame buffer
            pTxFrame = (tEplFrame *) pIntNodeInfo_p->pPreqTxBuffer[1].pBuffer;
            // set up destination MAC address
            EPL_MEMCPY(pTxFrame->m_be_abDstMac, pIntNodeInfo_p->aMacAddr, 6);
            // set destination node-ID in PReq
            ami_setUint8Le(&pTxFrame->m_le_bDstNodeId, (UINT8) pIntNodeInfo_p->nodeId);

            event.m_EventSink = kEplEventSinkNmtMnu;
            event.m_EventType = kEplEventTypeNmtMnuNodeAdded;
            event.m_uiSize = sizeof (pIntNodeInfo_p->nodeId);
            event.m_pArg = &pIntNodeInfo_p->nodeId;
            ret = eventk_postEvent(&event);
            if (ret != kEplSuccessful)
                goto Exit;

        }
#if EPL_DLL_PRES_CHAINING_MN == FALSE
        else
        {   // TxBuffer for PReq does not exist
            ret = kEplDllTxFrameInvalid;
            goto Exit;
        }
#endif

        ret = errhndk_resetCnError(pIntNodeInfo_p->nodeId);
    }

    // initialize elements of internal node info structure
    pIntNodeInfo_p->soaFlag1 = 0;
    pIntNodeInfo_p->fSoftDelete = FALSE;
    pIntNodeInfo_p->nmtState = kNmtCsNotActive;
    pIntNodeInfo_p->dllErrorEvents = 0L;
    // add node to list
    pIntNodeInfo_p->pNextNodeInfo = *ppIntNodeInfo;
    *ppIntNodeInfo = pIntNodeInfo_p;

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Delete node from the isochronous phase

This function removes a node from the isochronous phase.

\param  pIntNodeInfo_p      Pointer to internal node info structure.

\return The function returns a pointer to the node Information of the node
*/
//------------------------------------------------------------------------------
tEplKernel dllk_deleteNodeIsochronous(tDllkNodeInfo* pIntNodeInfo_p)
{
    tEplKernel          ret = kEplSuccessful;
    tDllkNodeInfo**     ppIntNodeInfo;
    tEplFrame*          pTxFrame;

#if EPL_DLL_PRES_CHAINING_MN != FALSE
    if (pIntNodeInfo_p->pPreqTxBuffer == NULL)
    {
        ppIntNodeInfo = &dllkInstance_g.pFirstPrcNodeInfo;
    }
    else
#endif
    {
        ppIntNodeInfo = &dllkInstance_g.pFirstNodeInfo;
    }
    // search node in whole list
    while ((*ppIntNodeInfo != NULL) && (*ppIntNodeInfo != pIntNodeInfo_p))
    {
        ppIntNodeInfo = &(*ppIntNodeInfo)->pNextNodeInfo;
    }
    if ((*ppIntNodeInfo == NULL) || (*ppIntNodeInfo != pIntNodeInfo_p))
    {   // node was not found in list
        // $$$ d.k. maybe this should be an error
        return ret;
    }

    // remove node from list
    *ppIntNodeInfo = pIntNodeInfo_p->pNextNodeInfo;
    if (pIntNodeInfo_p->pPreqTxBuffer != NULL)
    {   // disable TPDO
        pTxFrame = (tEplFrame *) pIntNodeInfo_p->pPreqTxBuffer[0].pBuffer;
        if (pTxFrame != NULL)
        {   // frame does exist
            // update frame (disable RD in Flag1)
            ami_setUint8Le(&pTxFrame->m_Data.m_Preq.m_le_bFlag1, 0);
        }

        pTxFrame = (tEplFrame *) pIntNodeInfo_p->pPreqTxBuffer[1].pBuffer;
        if (pTxFrame != NULL)
        {   // frame does exist
            // update frame (disable RD in Flag1)
            ami_setUint8Le(&pTxFrame->m_Data.m_Preq.m_le_bFlag1, 0);
        }
    }
    return ret;
}
#endif

#if EPL_DLL_PRES_CHAINING_CN != FALSE
//------------------------------------------------------------------------------
/**
\brief  Enable PRes chaining mode

This function enables the PRes chaining mode.

\return The function returns a pointer to the node Information of the node
*/
//------------------------------------------------------------------------------
tEplKernel dllk_presChainingEnable (void)
{
    tEplKernel      Ret = kEplSuccessful;
    tEplFrame*      pTxFrameSyncRes;

    if (dllkInstance_g.fPrcEnabled == FALSE)
    {
        // relocate PReq filter to PResMN
        ami_setUint48Be(&dllkInstance_g.aFilter[DLLK_FILTER_PREQ].aFilterValue[0],
                          EPL_C_DLL_MULTICAST_PRES);
        ami_setUint8Be(&dllkInstance_g.aFilter[DLLK_FILTER_PREQ].aFilterValue[14],
                       kEplMsgTypePres);
        ami_setUint8Be(&dllkInstance_g.aFilter[DLLK_FILTER_PREQ].aFilterValue[15],
                       EPL_C_ADR_BROADCAST); // Set Destination Node ID to C_ADR_BROADCAST

        dllkInstance_g.pTxBuffer[DLLK_TXFRAME_PRES].timeOffsetNs = dllkInstance_g.prcPResTimeFirst;

        Ret = edrv_changeRxFilter(dllkInstance_g.aFilter, DLLK_FILTER_COUNT, DLLK_FILTER_PREQ,
                               EDRV_FILTER_CHANGE_VALUE | EDRV_FILTER_CHANGE_AUTO_RESPONSE_DELAY);
        if (Ret != kEplSuccessful)
            return Ret;

        dllkInstance_g.fPrcEnabled = TRUE;
        pTxFrameSyncRes = (tEplFrame *) dllkInstance_g.pTxBuffer[DLLK_TXFRAME_SYNCRES].pBuffer;

        ami_setUint32Le(&pTxFrameSyncRes->m_Data.m_Asnd.m_Payload.m_SyncResponse.m_le_dwSyncStatus,
                        ami_getUint32Le(&pTxFrameSyncRes->m_Data.m_Asnd.m_Payload.m_SyncResponse.m_le_dwSyncStatus)
                        | EPL_SYNC_PRES_MODE_SET);
        // update SyncRes Tx buffer in Edrv
        Ret = edrv_updateTxBuffer(&dllkInstance_g.pTxBuffer[DLLK_TXFRAME_SYNCRES]);
        if (Ret != kEplSuccessful)
            return Ret;

#if (EPL_DLL_PROCESS_SYNC == EPL_DLL_PROCESS_SYNC_ON_TIMER)
        Ret = synctimer_setLossOfSyncTolerance2(dllkInstance_g.prcPResFallBackTimeout);
#endif
    }
    return Ret;
}

//------------------------------------------------------------------------------
/**
\brief  Disable PRes chaining mode

This function disables the PRes chaining mode, thus restoring PReq/PRes mode.

\return The function returns a pointer to the node Information of the node
*/
//------------------------------------------------------------------------------
tEplKernel dllk_presChainingDisable (void)
{
    tEplKernel      ret = kEplSuccessful;
    tEplFrame*      pTxFrameSyncRes;

    if (dllkInstance_g.fPrcEnabled != FALSE)
    {   // relocate PReq filter from PResMN to PReq
        EPL_MEMCPY(&dllkInstance_g.aFilter[DLLK_FILTER_PREQ].aFilterValue[0],
                   &dllkInstance_g.aLocalMac[0], 6);
        ami_setUint8Be(&dllkInstance_g.aFilter[DLLK_FILTER_PREQ].aFilterValue[14],
                       kEplMsgTypePreq);
        ami_setUint8Be(&dllkInstance_g.aFilter[DLLK_FILTER_PREQ].aFilterMask[15],
                       (BYTE) dllkInstance_g.dllConfigParam.nodeId); // Set Dst Node ID

        // disable auto-response delay
        dllkInstance_g.pTxBuffer[DLLK_TXFRAME_PRES].timeOffsetNs = 0;

        dllkInstance_g.fPrcEnabled = FALSE;

        ret = edrv_changeRxFilter(dllkInstance_g.aFilter, DLLK_FILTER_COUNT, DLLK_FILTER_PREQ,
                               EDRV_FILTER_CHANGE_VALUE | EDRV_FILTER_CHANGE_AUTO_RESPONSE_DELAY);
        if (ret != kEplSuccessful)
            return ret;

        pTxFrameSyncRes = (tEplFrame *) dllkInstance_g.pTxBuffer[DLLK_TXFRAME_SYNCRES].pBuffer;

        ami_setUint32Le(&pTxFrameSyncRes->m_Data.m_Asnd.m_Payload.m_SyncResponse.m_le_dwSyncStatus,
                        ami_getUint32Le(&pTxFrameSyncRes->m_Data.m_Asnd.m_Payload.m_SyncResponse.m_le_dwSyncStatus)
                        & ~EPL_SYNC_PRES_MODE_SET);
        // update SyncRes Tx buffer in Edrv
        ret = edrv_updateTxBuffer(&dllkInstance_g.pTxBuffer[DLLK_TXFRAME_SYNCRES]);
        if (ret != kEplSuccessful)
            return ret;

#if (EPL_DLL_PROCESS_SYNC == EPL_DLL_PROCESS_SYNC_ON_TIMER)
        ret = synctimer_setLossOfSyncTolerance2(0);
#endif
    }
    return ret;
}

#if (EPL_DLL_PROCESS_SYNC == EPL_DLL_PROCESS_SYNC_ON_TIMER)
//------------------------------------------------------------------------------
/**
\brief  Callback function for PResFallBackTimeout

This function is called by the timer sync module. It signals that the
PResFallBackTimeout hat triggered.

\return The function returns a pointer to the node Information of the node
*/
//------------------------------------------------------------------------------
tEplKernel dllk_cbCnPresFallbackTimeout(void)
{
    tEplKernel      ret = kEplSuccessful;
    tNmtState       nmtState;
    UINT32          arg;

    TGT_DLLK_DECLARE_FLAGS;
    TGT_DLLK_ENTER_CRITICAL_SECTION();

    nmtState = dllkInstance_g.nmtState;
    if (nmtState <= kNmtGsResetConfiguration)
        goto Exit;

    ret = dllk_presChainingDisable();

Exit:
    if (ret != kEplSuccessful)
    {

        BENCHMARK_MOD_02_TOGGLE(7);
        arg = dllkInstance_g.dllState | (kNmtEventDllCeFrameTimeout << 8);
        // Error event for API layer
        ret = eventk_postError(kEplEventSourceDllk, ret, sizeof(arg), &arg);
    }

    TGT_DLLK_LEAVE_CRITICAL_SECTION();
    return ret;
}
#endif

#endif // #if EPL_DLL_PRES_CHAINING_CN != FALSE

#if defined(CONFIG_INCLUDE_NMT_MN)
//------------------------------------------------------------------------------
/**
\brief  Setup asynchronous phase of cycle

The function sets up the buffer structures for the asynchronous phase.

\param  nmtState_p              NMT state of the node.
\param  nextTxBufferOffset_p    Next txBuffer offset.
\param  nextTimeOffsetNs_p      Next time offset in cycle (in ns).
\param  pIndex_p                Pointer to next index in TX buffer list.
                                Will be updated in function

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
tEplKernel dllk_setupAsyncPhase(tNmtState nmtState_p, UINT nextTxBufferOffset_p,
                                UINT32 nextTimeOffsetNs_p, UINT* pIndex_p)
{
    tEplKernel          ret = kEplSuccessful;
    BOOL                fEnableInvitation;
    tEdrvTxBuffer*      pTxBuffer;
    UINT                soaIndex;

    pTxBuffer = &dllkInstance_g.pTxBuffer[DLLK_TXFRAME_SOA + nextTxBufferOffset_p];
    pTxBuffer->timeOffsetNs = nextTimeOffsetNs_p;
    // switch SoAReq buffer
    dllkInstance_g.syncLastSoaReq++;
    if (dllkInstance_g.syncLastSoaReq >= DLLK_SOAREQ_COUNT)
    {
        dllkInstance_g.syncLastSoaReq = 0;
    }

    // $$$ d.k. fEnableInvitation_p = ((NmtState_p != kNmtMsPreOperational1) ||
    //             (dllkInstance_g.cycleCount >= EPL_C_DLL_PREOP1_START_CYCLES))
    //          currently, processSync is not called in PreOp1
    ret = dllk_updateFrameSoa(pTxBuffer, nmtState_p, TRUE, dllkInstance_g.syncLastSoaReq);
    dllkInstance_g.ppTxBufferList[*pIndex_p] = pTxBuffer;
    //store SoA *pIndex_p
    soaIndex = *pIndex_p;
    (*pIndex_p)++;

    // check if we are invited in SoA
    if (dllkInstance_g.aLastTargetNodeId[dllkInstance_g.syncLastSoaReq] ==
                                       dllkInstance_g.dllConfigParam.nodeId)
    {
        //disable invitation per default
        fEnableInvitation = FALSE;

        switch (dllkInstance_g.aLastReqServiceId[dllkInstance_g.syncLastSoaReq])
        {
            case kDllReqServiceStatus:
                // StatusRequest
                pTxBuffer = &dllkInstance_g.pTxBuffer[DLLK_TXFRAME_STATUSRES +
                                                      dllkInstance_g.curTxBufferOffsetStatusRes];
                if (pTxBuffer->pBuffer != NULL)
                {   // StatusRes does exist
                    dllkInstance_g.ppTxBufferList[*pIndex_p] = pTxBuffer;
                    (*pIndex_p)++;

                    //TX buffer is ready, invitation enabled
                    fEnableInvitation = TRUE;

                    TGT_DBG_SIGNAL_TRACE_POINT(8);
                }

                break;

            case kDllReqServiceIdent:
                // IdentRequest
                pTxBuffer = &dllkInstance_g.pTxBuffer[DLLK_TXFRAME_IDENTRES +
                                                      dllkInstance_g.curTxBufferOffsetIdentRes];
                if (pTxBuffer->pBuffer != NULL)
                {   // IdentRes does exist
                    dllkInstance_g.ppTxBufferList[*pIndex_p] = pTxBuffer;
                    (*pIndex_p)++;

                    //TX buffer is ready, invitation enabled
                    fEnableInvitation = TRUE;

                    TGT_DBG_SIGNAL_TRACE_POINT(7);
                }
                break;

            case kDllReqServiceNmtRequest:
                // NmtRequest
                pTxBuffer = &dllkInstance_g.pTxBuffer[DLLK_TXFRAME_NMTREQ +
                                                      dllkInstance_g.curTxBufferOffsetNmtReq];
                if (pTxBuffer->pBuffer != NULL)
                {   // NmtRequest does exist
                    // check if frame is not empty and not being filled
                    if (pTxBuffer->txFrameSize > DLLK_BUFLEN_FILLING)
                    {
                        dllkInstance_g.ppTxBufferList[*pIndex_p] = pTxBuffer;
                        (*pIndex_p)++;
                        dllkInstance_g.curTxBufferOffsetNmtReq ^= 1;

                        //TX buffer is ready, invitation enabled
                        fEnableInvitation = TRUE;
                    }
                }
                break;

            case kDllReqServiceUnspecified:
                // unspecified invite
                pTxBuffer = &dllkInstance_g.pTxBuffer[DLLK_TXFRAME_NONEPL +
                                                      dllkInstance_g.curTxBufferOffsetNonEpl];
                if (pTxBuffer->pBuffer != NULL)
                {   // non-EPL frame does exist
                    // check if frame is not empty and not being filled
                    if (pTxBuffer->txFrameSize > DLLK_BUFLEN_FILLING)
                    {
                        dllkInstance_g.ppTxBufferList[*pIndex_p] = pTxBuffer;
                        (*pIndex_p)++;
                        dllkInstance_g.curTxBufferOffsetNonEpl ^= 1;

                        //TX buffer is ready, invitation enabled
                        fEnableInvitation = TRUE;
                    }
                }
                break;

            default:
                break;
        }

        //is invitation allowed?
        if(fEnableInvitation == FALSE)
        {
            tEplFrame *pTxFrame = (tEplFrame *)
                dllkInstance_g.ppTxBufferList[soaIndex]->pBuffer;

            //reset invitation
            ami_setUint8Le(&pTxFrame->m_Data.m_Soa.m_le_bReqServiceId,
                    kDllReqServiceNo);
            ami_setUint8Le(&pTxFrame->m_Data.m_Soa.m_le_bReqServiceTarget,
                    EPL_C_ADR_INVALID);
        }
        else
        {
            // Asnd frame will be sent, remove the request
            dllkInstance_g.aLastReqServiceId[dllkInstance_g.syncLastSoaReq] = kDllReqServiceNo;
        }
    }
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Setup synchronous phase of cycle

The function sets up the buffer structures for the synchronous phase.

\param  nmtState_p              NMT state of the node.
\param  fReadyFlag_p            Status of ready flag.
\param  nextTxBufferOffset_p    Next txBuffer offset.
\param  pNextTimeOffsetNs_p     Pointer to next time offset in cycle (in ns).
                                Will be updated in function.
\param  pIndex_p                Pointer to next index in TX buffer list.
                                Will be updated in function

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
tEplKernel dllk_setupSyncPhase(tNmtState nmtState_p, BOOL fReadyFlag_p,
                               UINT nextTxBufferOffset_p, UINT32* pNextTimeOffsetNs_p, UINT* pIndex_p)
{
    tEplKernel          ret = kEplSuccessful;
    BYTE*               pCnNodeId;
    UINT32              accFrameLenNs = 0;
    UINT                nextTimeOffsetNs = 0;
    tEplFrame*          pTxFrame;
    tEdrvTxBuffer*      pTxBuffer;
    tFrameInfo          FrameInfo;
    tDllkNodeInfo*      pIntNodeInfo;
    BYTE                flag1;

    // calculate WaitSoCPReq delay
    if (dllkInstance_g.dllConfigParam.waitSocPreq != 0)
    {
        *pNextTimeOffsetNs_p = dllkInstance_g.dllConfigParam.waitSocPreq
                            + EPL_C_DLL_T_PREAMBLE + EPL_C_DLL_T_MIN_FRAME + EPL_C_DLL_T_IFG;
    }
    else
    {
        accFrameLenNs = EPL_C_DLL_T_PREAMBLE + EPL_C_DLL_T_MIN_FRAME + EPL_C_DLL_T_IFG;
    }

    pCnNodeId = &dllkInstance_g.aCnNodeIdList[nextTxBufferOffset_p][0];

    if (nmtState_p != kNmtMsOperational)
        fReadyFlag_p = FALSE;

    pIntNodeInfo = dllkInstance_g.pFirstNodeInfo;
    while (pIntNodeInfo != NULL)
    {
        pTxBuffer = &pIntNodeInfo->pPreqTxBuffer[nextTxBufferOffset_p];
        if ((pTxBuffer != NULL) && (pTxBuffer->pBuffer != NULL))
        {   // PReq does exist
            pTxFrame = (tEplFrame *) pTxBuffer->pBuffer;

            flag1 = pIntNodeInfo->soaFlag1 & EPL_FRAME_FLAG1_EA;

            // $$$ d.k. set EPL_FRAME_FLAG1_MS if necessary
            // update frame (Flag1)
            ami_setUint8Le(&pTxFrame->m_Data.m_Preq.m_le_bFlag1, flag1);

            // process TPDO
            FrameInfo.pFrame = pTxFrame;
            FrameInfo.frameSize = pTxBuffer->txFrameSize;
            ret = dllk_processTpdo(&FrameInfo, fReadyFlag_p);
            if (ret != kEplSuccessful)
                return ret;

            pTxBuffer->timeOffsetNs = *pNextTimeOffsetNs_p;
            dllkInstance_g.ppTxBufferList[*pIndex_p] = pTxBuffer;
            (*pIndex_p)++;

            if (pTxBuffer == &dllkInstance_g.pTxBuffer[DLLK_TXFRAME_PRES + nextTxBufferOffset_p])
            {   // PRes of MN will be sent
                // update NMT state
                ami_setUint8Le(&pTxFrame->m_Data.m_Pres.m_le_bNmtStatus, (BYTE) nmtState_p);

#if EPL_DLL_PRES_CHAINING_MN != FALSE
                *pNextTimeOffsetNs_p = pIntNodeInfo->presTimeoutNs;
                {
                    tDllkNodeInfo*   pIntPrcNodeInfo;

                    pIntPrcNodeInfo = dllkInstance_g.pFirstPrcNodeInfo;
                    while (pIntPrcNodeInfo != NULL)
                    {
                        *pCnNodeId = (BYTE) pIntPrcNodeInfo->nodeId;
                        pCnNodeId++;
                        *pNextTimeOffsetNs_p = pIntNodeInfo->presTimeoutNs;
                        pIntPrcNodeInfo = pIntPrcNodeInfo->pNextNodeInfo;
                    }

                    *pCnNodeId = EPL_C_ADR_BROADCAST;    // mark this entry as PRC slot finished
                    pCnNodeId++;
                }
#else
                *pNextTimeOffsetNs_p = 0;
#endif

            }
            else
            {   // PReq to CN
                *pCnNodeId = (BYTE) pIntNodeInfo->nodeId;
                pCnNodeId++;
                *pNextTimeOffsetNs_p = pIntNodeInfo->presTimeoutNs;
            }

            if (nextTimeOffsetNs == 0)
            {   // add SoC frame length
                accFrameLenNs += EPL_C_DLL_T_PREAMBLE +
                                 (pTxBuffer->txFrameSize * EPL_C_DLL_T_BITTIME) + EPL_C_DLL_T_IFG;
            }
            else
            {
                *pNextTimeOffsetNs_p += accFrameLenNs;
                accFrameLenNs = 0;
            }
        }

        pIntNodeInfo = pIntNodeInfo->pNextNodeInfo;
    }
    *pCnNodeId = EPL_C_ADR_INVALID;    // mark last entry in node-ID list

    return ret;
}
#endif

///\}
