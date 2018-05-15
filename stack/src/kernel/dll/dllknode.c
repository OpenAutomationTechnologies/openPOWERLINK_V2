/**
********************************************************************************
\file   dllknode.c

\brief  Implementation of DLL kernel node functions

This file contains the implementation of the DLL kernel node functions.
It is part of the DLL kernel module.

\ingroup module_dllk
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2016, B&R Industrial Automation GmbH
Copyright (c) 2015, SYSTEC electronic GmbH
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
#include "dllknode.h"
#include "dllkframe.h"
#include "dllk-internal.h"

#include <kernel/dllkfilter.h>
#include <kernel/dllkcal.h>
#include <kernel/edrv.h>
#include <kernel/eventk.h>
#include <kernel/errhndk.h>
#include <kernel/timesynck.h>
#include <common/ami.h>
#include <oplk/benchmark.h>

#if (CONFIG_TIMER_USE_HIGHRES != FALSE)
#include <kernel/hrestimer.h>
#endif

#if (CONFIG_DLL_PROCESS_SYNC == DLL_PROCESS_SYNC_ON_TIMER)
#include <kernel/synctimer.h>
#endif

#if defined(CONFIG_INCLUDE_NMT_MN)
#include <kernel/edrvcyclic.h>
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

//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static tOplkError setupLocalNodeCn(void);

#if defined(CONFIG_INCLUDE_NMT_MN)
static tOplkError setupLocalNodeMn(void);
static tOplkError cbMnSyncHandler(void) SECTION_DLLK_MN_SYNC_CB;
#endif

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief  Clean up the local node

The function is called if the node falls back into a reset state. It cleans
up all stuff for the local node.

\param[in]      oldNmtState_p       Previous NMT state of the local node.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
tOplkError dllknode_cleanupLocalNode(tNmtState oldNmtState_p)
{
    tOplkError  ret;
    UINT8       aMulticastMac[6];
#if (NMT_MAX_NODE_ID > 0)
    UINT        index;
#endif

#if defined(CONFIG_INCLUDE_NMT_MN)
    UINT        handle;
#else
    UNUSED_PARAMETER(oldNmtState_p);
#endif

    // remove all filters from Edrv
    ret = edrv_changeRxFilter(NULL, 0, 0, 0);
    if (ret != kErrorOk)
    {
        DEBUG_LVL_ERROR_TRACE("%s remove all filters failed with 0x%X\n",
                              __func__,
                              ret);
    }

    // delete timer
#if (CONFIG_TIMER_USE_HIGHRES != FALSE)
    ret = hrestimer_deleteTimer(&dllkInstance_g.timerHdlCycle);
    if (ret != kErrorOk)
        return ret;

#if defined(CONFIG_INCLUDE_NMT_RMN)
    ret = hrestimer_deleteTimer(&dllkInstance_g.timerHdlSwitchOver);
    if (ret != kErrorOk)
        return ret;
#endif
#endif

#if defined(CONFIG_INCLUDE_NMT_MN)
    ret = edrvcyclic_stopCycle(FALSE);
    if (ret != kErrorOk)
        return ret;

    ret = edrvcyclic_regSyncHandler(NULL);
    if (ret != kErrorOk)
        return ret;
#endif

#if (CONFIG_DLL_PROCESS_SYNC == DLL_PROCESS_SYNC_ON_TIMER)
    ret = synctimer_stopSync();
    if (ret != kErrorOk)
        return ret;
#endif

#if defined(CONFIG_INCLUDE_NMT_MN)
    // destroy all data structures
    OPLK_FREE(dllkInstance_g.ppTxBufferList);
    dllkInstance_g.ppTxBufferList = NULL;
#endif

    // delete Tx frames
    ret = dllkframe_deleteTxFrame(DLLK_TXFRAME_IDENTRES);
    if (ret != kErrorOk)
        return ret;

    ret = dllkframe_deleteTxFrame(DLLK_TXFRAME_STATUSRES);
    if (ret != kErrorOk)
        return ret;

    ret = dllkframe_deleteTxFrame(DLLK_TXFRAME_PRES);
    if (ret != kErrorOk)
        return ret;

#if defined(CONFIG_INCLUDE_NMT_RMN)
    if (dllkInstance_g.fRedundancy)
    {
      ret = dllkframe_deleteTxFrame(DLLK_TXFRAME_AMNI);
      if (ret != kErrorOk)
        return ret;
    }
#endif

    dllkInstance_g.aTxBufferStateNmtReq[0] = kDllkTxBufEmpty;
    dllkInstance_g.aTxBufferStateNmtReq[1] = kDllkTxBufEmpty;
    ret = dllkframe_deleteTxFrame(DLLK_TXFRAME_NMTREQ);
    if (ret != kErrorOk)
        return ret;

#if (CONFIG_DLL_PRES_CHAINING_CN != FALSE)
    ret = dllkframe_deleteTxFrame(DLLK_TXFRAME_SYNCRES);
    if (ret != kErrorOk)
        return ret;
#endif

    dllkInstance_g.aTxBufferStateNonPlk[0] = kDllkTxBufEmpty;
    dllkInstance_g.aTxBufferStateNonPlk[1] = kDllkTxBufEmpty;
    ret = dllkframe_deleteTxFrame(DLLK_TXFRAME_NONPLK);
    if (ret != kErrorOk)
        return ret;

#if defined(CONFIG_INCLUDE_NMT_MN)
#if !defined(CONFIG_INCLUDE_NMT_RMN)
    if (NMT_IF_MN_OR_RMN(oldNmtState_p))
#else
    if (NMT_IF_MN_OR_RMN(oldNmtState_p) || (dllkInstance_g.fRedundancy))
#endif
    {   // local node was MN
        ret = dllkframe_deleteTxFrame(DLLK_TXFRAME_SOC);
        if (ret != kErrorOk)
            return ret;

        ret = dllkframe_deleteTxFrame(DLLK_TXFRAME_SOA);
        if (ret != kErrorOk)
            return ret;

        for (index = 0; index < tabentries(dllkInstance_g.aNodeInfo); index++)
        {
            if (dllkInstance_g.aNodeInfo[index].pPreqTxBuffer != NULL)
            {
                handle = (UINT)(dllkInstance_g.aNodeInfo[index].pPreqTxBuffer - dllkInstance_g.pTxBuffer);
                dllkInstance_g.aNodeInfo[index].pPreqTxBuffer = NULL;
                if (handle != DLLK_TXFRAME_PRES)
                {
                    ret = dllkframe_deleteTxFrame(handle);
                    if (ret != kErrorOk)
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
        for (index = 0; index < tabentries(dllkInstance_g.aNodeInfo); index++)
        {
            // disable PReq and PRes for this node
            dllkInstance_g.aNodeInfo[index].presPayloadLimit = 0;
            dllkInstance_g.aNodeInfo[index].preqPayloadLimit = 0;
        }
    }
#else
    // must be CN, because MN part is not compiled!
#if (NMT_MAX_NODE_ID > 0)
    for (index = 0; index < tabentries(dllkInstance_g.aNodeInfo); index++)
    {
        // disable PRes for this node
        dllkInstance_g.aNodeInfo[index].presPayloadLimit = 0;
    }
#endif

#endif

    // de-register multicast MACs in Ethernet driver
    ami_setUint48Be(&aMulticastMac[0], C_DLL_MULTICAST_SOC);
    edrv_clearRxMulticastMacAddr(aMulticastMac);
    ami_setUint48Be(&aMulticastMac[0], C_DLL_MULTICAST_SOA);
    edrv_clearRxMulticastMacAddr(aMulticastMac);
    ami_setUint48Be(&aMulticastMac[0], C_DLL_MULTICAST_PRES);
    edrv_clearRxMulticastMacAddr(aMulticastMac);
    ami_setUint48Be(&aMulticastMac[0], C_DLL_MULTICAST_ASND);
    ret = edrv_clearRxMulticastMacAddr(aMulticastMac);

    // Reset relative time validation flag
    dllkInstance_g.socTime.fRelTimeValid = FALSE;

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Setup the local node

The function is called if the node enters the "not active" state from a reset
state and initializes all stuff that is needed for operation.

\param[in]      nmtState_p          NMT state of the node. Could be kNmtMsNotActive
                                    or kNmtCsNotActive

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
tOplkError dllknode_setupLocalNode(tNmtState nmtState_p)
{
    tOplkError  ret = kErrorOk;
    UINT        handle;
    size_t      frameSize;
    UINT8       aMulticastMac[6];

#if !defined(CONFIG_INCLUDE_NMT_MN)
    UNUSED_PARAMETER(nmtState_p);
#endif

    // initialize flags for PRes and StatusRes (leave Flag 1 unchanged)
    dllkInstance_g.mnFlag1 = 0;
    dllkInstance_g.flag2 = 0;

#if defined(CONFIG_INCLUDE_NMT_MN)
    // initialize linked node list
    dllkInstance_g.pFirstNodeInfo = NULL;
    dllkInstance_g.pFirstPrcNodeInfo = NULL;
#endif
#if defined(CONFIG_INCLUDE_NMT_RMN)
    if (nmtState_p == kNmtRmsNotActive)
        dllkInstance_g.fRedundancy = TRUE;
    else
        dllkInstance_g.fRedundancy = FALSE;

    // AMNI
    if (dllkInstance_g.fRedundancy)
    {
        frameSize = C_DLL_MINSIZE_AMNI;
        ret = dllkframe_createTxFrame(&handle, &frameSize, kMsgTypeAmni, kDllAsndNotDefined);
        if (ret != kErrorOk)
        {   // error occurred while registering Tx frame
            return ret;
        }
    }
#endif

    /*-----------------------------------------------------------------------*/
    /* register TxFrames in Edrv */
    // IdentResponse
    frameSize = C_DLL_MINSIZE_IDENTRES;
    ret = dllkframe_createTxFrame(&handle, &frameSize, kMsgTypeAsnd, kDllAsndIdentResponse);
    if (ret != kErrorOk)
        return ret;

    // StatusResponse
    frameSize = C_DLL_MINSIZE_STATUSRES;
    ret = dllkframe_createTxFrame(&handle, &frameSize, kMsgTypeAsnd, kDllAsndStatusResponse);
    if (ret != kErrorOk)
        return ret;

#if (CONFIG_DLL_PRES_CHAINING_CN != FALSE)
    // SyncResponse
    frameSize = C_DLL_MINSIZE_SYNCRES;
    ret = dllkframe_createTxFrame(&handle, &frameSize, kMsgTypeAsnd, kDllAsndSyncResponse);
    if (ret != kErrorOk)
        return ret;
#endif

    // PRes
    if ((dllkInstance_g.dllConfigParam.fAsyncOnly == FALSE) &&
        (dllkInstance_g.dllConfigParam.presActPayloadLimit >= 36))
    {   // it is not configured as async-only CN,
        // so take part in isochronous phase and register PRes frame
        frameSize = dllkInstance_g.dllConfigParam.presActPayloadLimit + PLK_FRAME_OFFSET_PDO_PAYLOAD;
        ret = dllkframe_createTxFrame(&handle, &frameSize, kMsgTypePres, kDllAsndNotDefined);
        if (ret != kErrorOk)
            return ret;

        // reset cycle counter
        dllkInstance_g.cycleCount = 0;
        dllkInstance_g.prescaleCycleCount = 0;
    }
    else
    {   // it is an async-only CN -> fool changeState() to think that PRes was not expected
        dllkInstance_g.cycleCount = 1;
    }

    // NMT request
    frameSize = C_DLL_MAX_ETH_FRAME;
    ret = dllkframe_createTxFrame(&handle, &frameSize, kMsgTypeAsnd, kDllAsndNmtRequest);
    if (ret != kErrorOk)
        return ret;

    // mark Tx buffer as empty
    dllkInstance_g.aTxBufferStateNmtReq[0] = kDllkTxBufEmpty;
    dllkInstance_g.pTxBuffer[handle].txFrameSize = 0;
    dllkInstance_g.pTxBuffer[handle].pfnTxHandler = dllkframe_processTransmittedNmtReq;
    handle++;
    dllkInstance_g.aTxBufferStateNmtReq[1] = kDllkTxBufEmpty;
    dllkInstance_g.pTxBuffer[handle].txFrameSize = 0;
    dllkInstance_g.pTxBuffer[handle].pfnTxHandler = dllkframe_processTransmittedNmtReq;

    // non-POWERLINK frame
    frameSize = C_DLL_MAX_ETH_FRAME;
    ret = dllkframe_createTxFrame(&handle, &frameSize, kMsgTypeNonPowerlink, kDllAsndNotDefined);
    if (ret != kErrorOk)
        return ret;

    // mark Tx buffer as empty
    dllkInstance_g.aTxBufferStateNonPlk[0] = kDllkTxBufEmpty;
    dllkInstance_g.pTxBuffer[handle].txFrameSize = 0;
    dllkInstance_g.pTxBuffer[handle].pfnTxHandler = dllkframe_processTransmittedNonPlk;
    handle++;
    dllkInstance_g.aTxBufferStateNonPlk[1] = kDllkTxBufEmpty;
    dllkInstance_g.pTxBuffer[handle].txFrameSize = 0;
    dllkInstance_g.pTxBuffer[handle].pfnTxHandler = dllkframe_processTransmittedNonPlk;

    /*------------------------------------------------------------------------*/
    /* setup filter structure for Edrv */
    dllkfilter_setupFilters();

    // register multicast MACs in Ethernet driver
    ami_setUint48Be(&aMulticastMac[0], C_DLL_MULTICAST_SOC);
    edrv_setRxMulticastMacAddr(aMulticastMac);
    ami_setUint48Be(&aMulticastMac[0], C_DLL_MULTICAST_SOA);
    edrv_setRxMulticastMacAddr(aMulticastMac);
    ami_setUint48Be(&aMulticastMac[0], C_DLL_MULTICAST_PRES);
    edrv_setRxMulticastMacAddr(aMulticastMac);
    ami_setUint48Be(&aMulticastMac[0], C_DLL_MULTICAST_ASND);
    edrv_setRxMulticastMacAddr(aMulticastMac);

#if defined(CONFIG_INCLUDE_NMT_RMN)
    if (dllkInstance_g.fRedundancy)
    {
        ami_setUint48Be(&aMulticastMac[0], C_DLL_MULTICAST_AMNI);
        edrv_setRxMulticastMacAddr(aMulticastMac);
    }
#endif

    ret = timesynck_setCycleTime(dllkInstance_g.dllConfigParam.cycleLen,
                                 dllkInstance_g.dllConfigParam.minSyncTime);
    if (ret != kErrorOk)
        return ret;

#if defined(CONFIG_INCLUDE_NMT_MN)
    if (NMT_IF_MN_OR_RMN(nmtState_p))
    {
        ret = setupLocalNodeMn();
        if (ret != kErrorOk)
            return ret;

#if defined(CONFIG_INCLUDE_NMT_RMN)
        if (nmtState_p == kNmtRmsNotActive)
        {
            ret = setupLocalNodeCn();
            if (ret != kErrorOk)
                return ret;
        }
#endif
    }
    else
    {
        ret = setupLocalNodeCn();
        if (ret != kErrorOk)
            return ret;
    }
#else
    ret = setupLocalNodeCn();
    if (ret != kErrorOk)
        return ret;
#endif

    // clear all asynchronous buffers
    ret = dllkcal_clearAsyncBuffer();
    if (ret != kErrorOk)
        return ret;

    // set filters in Edrv
    ret = edrv_changeRxFilter(dllkInstance_g.aFilter, DLLK_FILTER_COUNT, DLLK_FILTER_COUNT, 0);

    return ret;
}

#if defined(CONFIG_INCLUDE_NMT_MN)
//------------------------------------------------------------------------------
/**
\brief  Add node to the isochronous phase

This function adds a node to the isochronous phase.

\param[in,out]  pIntNodeInfo_p      Pointer to internal node info structure.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
tOplkError dllknode_addNodeIsochronous(tDllkNodeInfo* pIntNodeInfo_p)
{
    tOplkError      ret = kErrorOk;
    tDllkNodeInfo** ppIntNodeInfo;
    tPlkFrame*      pTxFrame;

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
        if (pIntNodeInfo_p->pPreqTxBuffer == NULL)
            ppIntNodeInfo = &dllkInstance_g.pFirstPrcNodeInfo;
        else
            ppIntNodeInfo = &dllkInstance_g.pFirstNodeInfo;

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
            tEvent  event;

            pTxFrame = (tPlkFrame*)pIntNodeInfo_p->pPreqTxBuffer[0].pBuffer;
            // set up destination MAC address
            OPLK_MEMCPY(pTxFrame->aDstMac, pIntNodeInfo_p->aMacAddr, 6);
            // set destination node-ID in PReq
            ami_setUint8Le(&pTxFrame->dstNodeId, (UINT8)pIntNodeInfo_p->nodeId);
            // do the same for second frame buffer
            pTxFrame = (tPlkFrame*)pIntNodeInfo_p->pPreqTxBuffer[1].pBuffer;
            // set up destination MAC address
            OPLK_MEMCPY(pTxFrame->aDstMac, pIntNodeInfo_p->aMacAddr, 6);
            // set destination node-ID in PReq
            ami_setUint8Le(&pTxFrame->dstNodeId, (UINT8)pIntNodeInfo_p->nodeId);

            event.eventSink = kEventSinkNmtMnu;
            event.eventType = kEventTypeNmtMnuNodeAdded;
            event.eventArgSize = sizeof(pIntNodeInfo_p->nodeId);
            event.eventArg.pEventArg = &pIntNodeInfo_p->nodeId;

            ret = eventk_postEvent(&event);
            if (ret != kErrorOk)
                goto Exit;
        }

        ret = errhndk_resetCnError(pIntNodeInfo_p->nodeId);
    }

    // initialize elements of internal node info structure
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

\param[in]      pIntNodeInfo_p      Pointer to internal node info structure.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
tOplkError dllknode_deleteNodeIsochronous(const tDllkNodeInfo* pIntNodeInfo_p)
{
    tOplkError      ret = kErrorOk;
    tDllkNodeInfo** ppIntNodeInfo;
    tPlkFrame*      pTxFrame;

    if ((pIntNodeInfo_p->pPreqTxBuffer == NULL) &&
        (pIntNodeInfo_p->nodeId != dllkInstance_g.dllConfigParam.nodeId))
    {
        ppIntNodeInfo = &dllkInstance_g.pFirstPrcNodeInfo;
    }
    else
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
        pTxFrame = (tPlkFrame*)pIntNodeInfo_p->pPreqTxBuffer[0].pBuffer;
        if (pTxFrame != NULL)
        {   // frame does exist
            // update frame (disable RD in Flag1)
            ami_setUint8Le(&pTxFrame->data.preq.flag1, 0);
        }

        pTxFrame = (tPlkFrame*)pIntNodeInfo_p->pPreqTxBuffer[1].pBuffer;
        if (pTxFrame != NULL)
        {   // frame does exist
            // update frame (disable RD in Flag1)
            ami_setUint8Le(&pTxFrame->data.preq.flag1, 0);
        }
    }

    return ret;
}
#endif

#if defined(CONFIG_INCLUDE_NMT_MN)
//------------------------------------------------------------------------------
/**
\brief  Setup asynchronous phase of cycle

The function sets up the buffer structures for the asynchronous phase.

\param[in]      nmtState_p              NMT state of the node.
\param[in]      nextTxBufferOffset_p    Next txBuffer offset.
\param[in]      nextTimeOffsetNs_p      Next time offset in cycle (in ns).
\param[in,out]  pIndex_p                Pointer to next index in TX buffer list.
                                        Will be updated in the function.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
tOplkError dllknode_setupAsyncPhase(tNmtState nmtState_p,
                                    UINT nextTxBufferOffset_p,
                                    UINT32 nextTimeOffsetNs_p,
                                    UINT* pIndex_p)
{
    tOplkError      ret = kErrorOk;
    tEdrvTxBuffer*  pTxBuffer;

    pTxBuffer = &dllkInstance_g.pTxBuffer[DLLK_TXFRAME_SOA + nextTxBufferOffset_p];
    pTxBuffer->timeOffsetNs = nextTimeOffsetNs_p;
    // switch SoAReq buffer
    dllkInstance_g.syncLastSoaReq++;
    if (dllkInstance_g.syncLastSoaReq >= DLLK_SOAREQ_COUNT)
        dllkInstance_g.syncLastSoaReq = 0;

    // $$$ d.k. fEnableInvitation_p = ((nmtState_p != kNmtMsPreOperational1) ||
    //             (dllkInstance_g.cycleCount >= C_DLL_PREOP1_START_CYCLES))
    //          currently, processSync is not called in PreOp1
    ret = dllkframe_updateFrameSoa(pTxBuffer, nmtState_p, TRUE, dllkInstance_g.syncLastSoaReq);
    dllkInstance_g.ppTxBufferList[*pIndex_p] = pTxBuffer;
    (*pIndex_p)++;

    // check if we are invited in SoA
    if (dllkInstance_g.aLastTargetNodeId[dllkInstance_g.syncLastSoaReq] ==
            dllkInstance_g.dllConfigParam.nodeId)
    {   // Note: The Tx buffers exist / are ready!
        //       This is checked in dllk_updateFrameSoa()
        switch (dllkInstance_g.aLastReqServiceId[dllkInstance_g.syncLastSoaReq])
        {
            case kDllReqServiceStatus:
                // StatusRequest
                pTxBuffer = &dllkInstance_g.pTxBuffer[DLLK_TXFRAME_STATUSRES +
                                                      dllkInstance_g.curTxBufferOffsetStatusRes];
                dllkInstance_g.ppTxBufferList[*pIndex_p] = pTxBuffer;
                (*pIndex_p)++;

                TGT_DBG_SIGNAL_TRACE_POINT(8);
                break;

            case kDllReqServiceIdent:
                // IdentRequest
                pTxBuffer = &dllkInstance_g.pTxBuffer[DLLK_TXFRAME_IDENTRES +
                                                      dllkInstance_g.curTxBufferOffsetIdentRes];
                dllkInstance_g.ppTxBufferList[*pIndex_p] = pTxBuffer;
                (*pIndex_p)++;


                TGT_DBG_SIGNAL_TRACE_POINT(7);
                break;

            case kDllReqServiceNmtRequest:
                // NmtRequest
                pTxBuffer = &dllkInstance_g.pTxBuffer[DLLK_TXFRAME_NMTREQ +
                                                      dllkInstance_g.curTxBufferOffsetNmtReq];
                dllkInstance_g.ppTxBufferList[*pIndex_p] = pTxBuffer;
                (*pIndex_p)++;
                dllkInstance_g.curTxBufferOffsetNmtReq ^= 1;
                break;

            case kDllReqServiceUnspecified:
                // unspecified invite
                pTxBuffer = &dllkInstance_g.pTxBuffer[DLLK_TXFRAME_NONPLK +
                                                      dllkInstance_g.curTxBufferOffsetNonPlk];
                dllkInstance_g.ppTxBufferList[*pIndex_p] = pTxBuffer;
                (*pIndex_p)++;
                dllkInstance_g.curTxBufferOffsetNonPlk ^= 1;
                break;

            default:
                break;
        }

        // Asnd frame will be sent, remove the request
        dllkInstance_g.aLastReqServiceId[dllkInstance_g.syncLastSoaReq] = kDllReqServiceNo;
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Setup synchronous phase of cycle

The function sets up the buffer structures for the synchronous phase.

\param[in]      nmtState_p              NMT state of the node.
\param[in]      fReadyFlag_p            Status of ready flag.
\param[in]      nextTxBufferOffset_p    Next txBuffer offset.
\param[in,out]  pNextTimeOffsetNs_p     Pointer to next time offset in cycle (in ns).
                                        Will be updated in function.
\param[in,out]  pIndex_p                Pointer to next index in TX buffer list.
                                        Will be updated in the function.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
tOplkError dllknode_setupSyncPhase(tNmtState nmtState_p,
                                   BOOL fReadyFlag_p,
                                   UINT nextTxBufferOffset_p,
                                   UINT32* pNextTimeOffsetNs_p,
                                   UINT* pIndex_p)
{
    tOplkError      ret = kErrorOk;
    UINT8*          pCnNodeId;
    UINT32          accFrameLenNs = 0;
    tPlkFrame*      pTxFrame;
    tEdrvTxBuffer*  pTxBuffer;
    tFrameInfo      frameInfo;
    tDllkNodeInfo*  pIntNodeInfo;
    UINT8           flag1;

    // calculate WaitSoCPReq delay
    if (dllkInstance_g.dllConfigParam.waitSocPreq != 0)
    {
        *pNextTimeOffsetNs_p = dllkInstance_g.dllConfigParam.waitSocPreq +
                                   C_DLL_T_PREAMBLE + C_DLL_T_MIN_FRAME + C_DLL_T_IFG;
    }
    else
    {
        accFrameLenNs = C_DLL_T_PREAMBLE + C_DLL_T_MIN_FRAME + C_DLL_T_IFG;
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
            pTxFrame = (tPlkFrame*)pTxBuffer->pBuffer;

            flag1 = pIntNodeInfo->soaFlag1 & PLK_FRAME_FLAG1_EA;

            // $$$ d.k. set PLK_FRAME_FLAG1_MS if necessary
            // update frame (Flag1)
            ami_setUint8Le(&pTxFrame->data.preq.flag1, flag1);

            // process TPDO
            frameInfo.frame.pBuffer = pTxFrame;
            frameInfo.frameSize = (UINT)pTxBuffer->txFrameSize;
            ret = dllkframe_processTpdo(&frameInfo, fReadyFlag_p);
            if (ret != kErrorOk)
                return ret;

            pTxBuffer->timeOffsetNs = *pNextTimeOffsetNs_p;
            dllkInstance_g.ppTxBufferList[*pIndex_p] = pTxBuffer;
            (*pIndex_p)++;

            if (pTxBuffer == &dllkInstance_g.pTxBuffer[DLLK_TXFRAME_PRES + nextTxBufferOffset_p])
            {   // PRes of MN will be sent
                // update NMT state
                ami_setUint8Le(&pTxFrame->data.pres.nmtStatus, (UINT8)nmtState_p);

                *pNextTimeOffsetNs_p = pIntNodeInfo->presTimeoutNs;
                {
                    tDllkNodeInfo*   pIntPrcNodeInfo;

                    pIntPrcNodeInfo = dllkInstance_g.pFirstPrcNodeInfo;
                    while (pIntPrcNodeInfo != NULL)
                    {
                        *pCnNodeId = (UINT8)pIntPrcNodeInfo->nodeId;
                        pCnNodeId++;
                        *pNextTimeOffsetNs_p = pIntNodeInfo->presTimeoutNs;
                        pIntPrcNodeInfo = pIntPrcNodeInfo->pNextNodeInfo;
                    }

                    *pCnNodeId = C_ADR_BROADCAST;    // mark this entry as PRC slot finished
                    pCnNodeId++;
                }
            }
            else
            {   // PReq to CN
                *pCnNodeId = (UINT8)pIntNodeInfo->nodeId;
                pCnNodeId++;
                *pNextTimeOffsetNs_p = pIntNodeInfo->presTimeoutNs;
            }

            if (*pNextTimeOffsetNs_p == 0)
            {   // add SoC frame length
                accFrameLenNs += (UINT32)(C_DLL_T_PREAMBLE +
                                     (pTxBuffer->txFrameSize * C_DLL_T_BITTIME) + C_DLL_T_IFG);
            }
            else
            {
                *pNextTimeOffsetNs_p += accFrameLenNs;
                accFrameLenNs = 0;
            }
        }

        pIntNodeInfo = pIntNodeInfo->pNextNodeInfo;
    }
    *pCnNodeId = C_ADR_INVALID;    // mark last entry in node-ID list

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Issue loss of PRes

The function forwards a loss of PRes event to the error handler module.

\param[in]      nodeId_p            Node ID of CN from which no PRes frame has been
                                    received.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
tOplkError dllknode_issueLossOfPres(UINT nodeId_p)
{
    tOplkError      ret = kErrorOk;
    tDllkNodeInfo*  pIntNodeInfo;
    tEvent          event;
    tDllNodeOpParam nodeOpParam;

    pIntNodeInfo = dllknode_getNodeInfo(nodeId_p);
    if (pIntNodeInfo != NULL)
    {
        if (pIntNodeInfo->fSoftDelete == FALSE)
        {   // normal isochronous CN
            tEventDllError  dllEvent;

            dllEvent.dllErrorEvents = DLL_ERR_MN_CN_LOSS_PRES;
            dllEvent.nodeId = pIntNodeInfo->nodeId;
            ret = errhndk_postError(&dllEvent);
            if (ret != kErrorOk)
                return ret;
        }
        else
        {   // CN shall be deleted softly, so remove it now without issuing any error
            nodeOpParam.opNodeType = kDllNodeOpTypeIsochronous;
            nodeOpParam.nodeId = pIntNodeInfo->nodeId;

            event.eventSink = kEventSinkDllkCal;
            event.eventType = kEventTypeDllkDelNode;
            // $$$ d.k. set Event.netTime to current time
            event.eventArgSize = sizeof(nodeOpParam);
            event.eventArg.pEventArg = &nodeOpParam;

            eventk_postEvent(&event);
        }
    }

    return ret;
}

#endif


#if (NMT_MAX_NODE_ID > 0)
//------------------------------------------------------------------------------
/**
\brief  Get node info of specified node

This function returns the node info structure of the specified node.

\param[in]      nodeId_p            Node ID of node for which to get info.

\return The function returns a pointer to the node information of the node.
*/
//------------------------------------------------------------------------------
tDllkNodeInfo* dllknode_getNodeInfo(UINT nodeId_p)
{
    // $$$ d.k.: use hash algorithm to retrieve the appropriate node info structure
    //           if size of array is less than 254.
    nodeId_p--;   // node ID starts at 1 but array at 0
    if (nodeId_p >= tabentries(dllkInstance_g.aNodeInfo))
        return NULL;
    else
        return &dllkInstance_g.aNodeInfo[nodeId_p];
}

//------------------------------------------------------------------------------
/**
\brief  Add PRes filter for the specified node

This function adds a PRes filter for the specified node.

\param[in,out]  pIntNodeInfo_p      Pointer to internal node info structure.
\param[in]      nodeOpType_p        Type of PRes filter.
\param[in]      fUpdateEdrv_p       Flag determines if Edrv Filter should be updated.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
tOplkError dllknode_addNodeFilter(tDllkNodeInfo* pIntNodeInfo_p,
                                  tDllNodeOpType nodeOpType_p,
                                  BOOL fUpdateEdrv_p)
{
    tOplkError  ret = kErrorOk;
    UINT8       presFilterFlags = 0;

    switch (nodeOpType_p)
    {
        case kDllNodeOpTypeFilterPdo:
            presFilterFlags = DLLK_FILTER_FLAG_PDO;
            break;

        case kDllNodeOpTypeFilterHeartbeat:
            presFilterFlags = DLLK_FILTER_FLAG_HB;
            break;

        default:
            ret = kErrorDllInvalidParam;
            goto Exit;
    }

    if (fUpdateEdrv_p != FALSE)
    {
        if ((pIntNodeInfo_p->presFilterFlags & (DLLK_FILTER_FLAG_PDO | DLLK_FILTER_FLAG_HB)) == 0)
        {
#if (CONFIG_DLL_PRES_FILTER_COUNT < 0)
            dllkInstance_g.usedPresFilterCount++;
            if (dllkInstance_g.usedPresFilterCount == 1)
            {
                // enable PRes Rx filter
                dllkInstance_g.aFilter[DLLK_FILTER_PRES].fEnable = TRUE;
                ret = edrv_changeRxFilter(dllkInstance_g.aFilter,
                                          DLLK_FILTER_COUNT,
                                          DLLK_FILTER_PRES,
                                          EDRV_FILTER_CHANGE_STATE);
                if (ret != kErrorOk)
                    goto Exit;
            }

#else
            UINT handle;

            for (handle = DLLK_FILTER_PRES; handle < DLLK_FILTER_COUNT; handle++)
            {
                if (ami_getUint8Le(&dllkInstance_g.aFilter[handle].aFilterValue[16]) == C_ADR_INVALID)
                {
                    ami_setUint8Be(&dllkInstance_g.aFilter[handle].aFilterValue[16],
                                   pIntNodeInfo_p->nodeId);
                    dllkInstance_g.aFilter[handle].fEnable = TRUE;

                    ret = edrv_changeRxFilter(dllkInstance_g.aFilter,
                                              DLLK_FILTER_COUNT,
                                              handle,
                                              (EDRV_FILTER_CHANGE_STATE | EDRV_FILTER_CHANGE_VALUE));
                    if (ret != kErrorOk)
                        goto Exit;
                    break;
                }
            }
#endif
        }
    }
    pIntNodeInfo_p->presFilterFlags |= presFilterFlags;

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Delete PRes filter for the specified node

This function deletes a PRes filter for the specified node.

\param[in,out]  pIntNodeInfo_p      Pointer to internal node info structure.
\param[in]      nodeOpType_p        Type of PRes filter.
\param[in]      fUpdateEdrv_p       Flag determines if Edrv Filter should be updated.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
tOplkError dllknode_deleteNodeFilter(tDllkNodeInfo* pIntNodeInfo_p,
                                     tDllNodeOpType nodeOpType_p,
                                     BOOL fUpdateEdrv_p)
{
    tOplkError  ret = kErrorOk;
    UINT8       presFilterFlags = 0;

    switch (nodeOpType_p)
    {
        case kDllNodeOpTypeFilterPdo:
            presFilterFlags = DLLK_FILTER_FLAG_PDO;
            break;

        case kDllNodeOpTypeFilterHeartbeat:
            presFilterFlags = DLLK_FILTER_FLAG_HB;
            break;

        default:
            ret = kErrorDllInvalidParam;
            goto Exit;
    }

    pIntNodeInfo_p->presFilterFlags &= ~presFilterFlags;

    if (fUpdateEdrv_p != FALSE)
    {
        if ((pIntNodeInfo_p->presFilterFlags & (DLLK_FILTER_FLAG_PDO | DLLK_FILTER_FLAG_HB)) == 0)
        {
#if (CONFIG_DLL_PRES_FILTER_COUNT < 0)
            if (dllkInstance_g.usedPresFilterCount > 0)
                dllkInstance_g.usedPresFilterCount--;

            if (dllkInstance_g.usedPresFilterCount == 0)
            {
                // disable PRes Rx filter
                dllkInstance_g.aFilter[DLLK_FILTER_PRES].fEnable = FALSE;
                ret = edrv_changeRxFilter(dllkInstance_g.aFilter,
                                          DLLK_FILTER_COUNT,
                                          DLLK_FILTER_PRES,
                                          EDRV_FILTER_CHANGE_STATE);
                if (ret != kErrorOk)
                    goto Exit;
            }

#else
            UINT handle;

            for (handle = DLLK_FILTER_PRES; handle < DLLK_FILTER_COUNT; handle++)
            {
                if (ami_getUint8Le(&dllkInstance_g.aFilter[handle].aFilterValue[16]) ==
                        pIntNodeInfo_p->nodeId)
                {
                    ami_setUint8Be(&dllkInstance_g.aFilter[handle].aFilterValue[16], C_ADR_INVALID);
                    dllkInstance_g.aFilter[handle].fEnable = FALSE;

                    ret = edrv_changeRxFilter(dllkInstance_g.aFilter,
                                              DLLK_FILTER_COUNT,
                                              handle,
                                              EDRV_FILTER_CHANGE_STATE);
                    if (ret != kErrorOk)
                        goto Exit;
                    break;
                }
            }
#endif
        }
    }
Exit:
    return ret;
}

#endif // NMT_MAX_NODE_ID > 0


//----------------------------------------------------------------------------//
//                L O C A L   F U N C T I O N S                               //
//----------------------------------------------------------------------------//

//------------------------------------------------------------------------------
/**
\brief   Setup CN specific stuff of local node

The function initializes the CN specific stuff of the local node.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError setupLocalNodeCn(void)
{
    tOplkError      ret = kErrorOk;

#if (CONFIG_DLL_PRES_FILTER_COUNT >= 0)
    UINT            handle;

#if (NMT_MAX_NODE_ID > 0)
    size_t          index;
    tDllkNodeInfo*  pIntNodeInfo;
#endif
#endif

    dllkfilter_setupPreqFilter(&dllkInstance_g.aFilter[DLLK_FILTER_PREQ],
                               dllkInstance_g.dllConfigParam.nodeId,
                               &dllkInstance_g.pTxBuffer[DLLK_TXFRAME_PRES],
                               edrv_getMacAddr());

    // setup PRes filter
#if (CONFIG_DLL_PRES_FILTER_COUNT < 0)
    if (dllkInstance_g.usedPresFilterCount > 0)
        dllkfilter_setupPresFilter(&dllkInstance_g.aFilter[DLLK_FILTER_PRES], TRUE);
    else
        dllkfilter_setupPresFilter(&dllkInstance_g.aFilter[DLLK_FILTER_PRES], FALSE);

#else
    for (handle = DLLK_FILTER_PRES; handle < DLLK_FILTER_COUNT; handle++)
    {
        dllkfilter_setupPresFilter(&dllkInstance_g.aFilter[handle], FALSE);
        ami_setUint8Be(&dllkInstance_g.aFilter[handle].aFilterMask[16], 0xFF);
    }

#if (NMT_MAX_NODE_ID > 0)
    handle = DLLK_FILTER_PRES;
    for (index = 0, pIntNodeInfo = &dllkInstance_g.aNodeInfo[0];
         index < tabentries(dllkInstance_g.aNodeInfo);
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

#if (CONFIG_DLL_PROCESS_SYNC == DLL_PROCESS_SYNC_ON_TIMER)
    ret = synctimer_setCycleLen(dllkInstance_g.dllConfigParam.cycleLen,
                                dllkInstance_g.dllConfigParam.minSyncTime);
    if (ret != kErrorOk)
        return ret;

    ret = synctimer_setLossOfSyncTolerance(dllkInstance_g.dllConfigParam.lossOfFrameTolerance);
    if (ret != kErrorOk)
        return ret;
#endif

#if (CONFIG_DLL_PRES_CHAINING_CN != FALSE)
    dllkInstance_g.fPrcEnabled = FALSE;
    dllkInstance_g.syncReqPrevNodeId = 0;
#endif

    return ret;
}


#if defined(CONFIG_INCLUDE_NMT_MN)
//------------------------------------------------------------------------------
/**
\brief   Setup MN specific stuff of local node

The function initializes the MN specific stuff of the local node.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError setupLocalNodeMn(void)
{
    tOplkError      ret = kErrorOk;
    UINT            handle;
    size_t          index;
    size_t          frameSize;
    UINT            count = 0;
    tDllkNodeInfo*  pIntNodeInfo;

    /*-------------------------------------------------------------------*/
    /* register TxFrames in Edrv */
    // SoC
    frameSize = C_DLL_MINSIZE_SOC;
    ret = dllkframe_createTxFrame(&handle, &frameSize, kMsgTypeSoc, kDllAsndNotDefined);
    if (ret != kErrorOk)
    {   // error occurred while registering Tx frame
        return ret;
    }

    dllkInstance_g.pTxBuffer[handle].pfnTxHandler = dllkframe_processTransmittedSoc;
    handle++;
    dllkInstance_g.pTxBuffer[handle].pfnTxHandler = dllkframe_processTransmittedSoc;

    // SoA
    frameSize = C_DLL_MINSIZE_SOA;
    ret = dllkframe_createTxFrame(&handle, &frameSize, kMsgTypeSoa, kDllAsndNotDefined);
    if (ret != kErrorOk)
    {   // error occurred while registering Tx frame
        return ret;
    }

    dllkInstance_g.pTxBuffer[handle].pfnTxHandler = dllkframe_processTransmittedSoa;
    handle++;
    dllkInstance_g.pTxBuffer[handle].pfnTxHandler = dllkframe_processTransmittedSoa;

    for (index = 0, pIntNodeInfo = &dllkInstance_g.aNodeInfo[0];
         index < tabentries(dllkInstance_g.aNodeInfo);
         index++, pIntNodeInfo++)
    {
        if (pIntNodeInfo->preqPayloadLimit > 0)
        {   // create PReq frame for this node
            count++;

            frameSize = pIntNodeInfo->preqPayloadLimit + PLK_FRAME_OFFSET_PDO_PAYLOAD;
            ret = dllkframe_createTxFrame(&handle, &frameSize, kMsgTypePreq, kDllAsndNotDefined);
            if (ret != kErrorOk)
                return ret;
            pIntNodeInfo->pPreqTxBuffer = &dllkInstance_g.pTxBuffer[handle];
        }
    }

    // alloc TxBuffer pointer list
    count += 5;   // SoC, PResMN, SoA, ASnd, NULL
    dllkInstance_g.ppTxBufferList = (tEdrvTxBuffer**)OPLK_MALLOC(sizeof(tEdrvTxBuffer*) * count);
    if (dllkInstance_g.ppTxBufferList == NULL)
        return kErrorDllOutOfMemory;

    ret = edrvcyclic_setMaxTxBufferListSize(count);
    if (ret != kErrorOk)
        return ret;

    ret = edrvcyclic_setCycleTime(dllkInstance_g.dllConfigParam.cycleLen,
                                  dllkInstance_g.dllConfigParam.minSyncTime);
    if (ret != kErrorOk)
        return ret;

    ret = edrvcyclic_regSyncHandler(cbMnSyncHandler);
    if (ret != kErrorOk)
        return ret;

    dllkfilter_setupPresFilter(&dllkInstance_g.aFilter[DLLK_FILTER_PRES], TRUE);

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  MN sync callback function

This function is called by the Ethernet driver module. It triggers the sync
event.

\return The function returns a pointer to the node information of the node.
*/
//------------------------------------------------------------------------------
static tOplkError cbMnSyncHandler(void)
{
    tOplkError  ret = kErrorOk;
    tNmtState   nmtState;
    UINT8*      pCnNodeId;
    UINT32      arg;

    TGT_DLLK_DECLARE_FLAGS;

    TGT_DLLK_ENTER_CRITICAL_SECTION();

    nmtState = dllkInstance_g.nmtState;
    if (nmtState <= kNmtGsResetConfiguration)
        goto Exit;

    // do cycle finish which has to be done inside the callback function triggered by interrupt
    pCnNodeId = &dllkInstance_g.aCnNodeIdList[dllkInstance_g.curTxBufferOffsetCycle][dllkInstance_g.curNodeIndex];

    while (*pCnNodeId != C_ADR_INVALID)
    {   // issue error for each CN in list which was not processed yet, i.e. PRes received
        ret = dllknode_issueLossOfPres(*pCnNodeId);
        if (ret != kErrorOk)
            goto Exit;

        pCnNodeId++;
    }

    dllkInstance_g.fSyncProcessed = FALSE;
    dllkInstance_g.fPrcSlotFinished = FALSE;

    // switch to next cycle
    dllkInstance_g.curTxBufferOffsetCycle ^= 1;
    dllkInstance_g.curNodeIndex = 0;

    ret = dllk_postEvent(kEventTypeDllkCycleFinish);

Exit:
    if (ret != kErrorOk)
    {
        BENCHMARK_MOD_02_TOGGLE(7);
        arg = dllkInstance_g.dllState | (kNmtEventDllMeSocTrig << 8);
        // Error event for API layer
        ret = eventk_postError(kEventSourceDllk, ret, sizeof(arg), &arg);
    }

    TGT_DLLK_LEAVE_CRITICAL_SECTION();

    return ret;
}

#endif

/// \}
