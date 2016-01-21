/**
********************************************************************************
\file   ndis-imProtocol.c

\brief  Protocol driver implementation for NDIS intermediate driver

This file contains implementation of the protocol section of the intermediate
driver. Protocol section of the intermediate driver is responsible for managing
the interface with the native miniport drivers of the NIC.

\ingroup module_ndis
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2015, Kalycito Infotech Private Limited
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
#include <ndis.h>

#include "ndis-imInternal.h"

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
NDIS_MEDIUM          aMediumArray_l[1] =
{
    NdisMedium802_3,    // Ethernet
};

static tProtocolInstance    protocolInstance_l;
static BOOLEAN              fBinding_l = FALSE;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static NDIS_STATUS startVEth(PNDIS_STRING pInstanceName_p);
static void        freeVEthInstance(tVEthInstance* pVEthInstance_p);
static void        stopVEth(tVEthInstance* pVEthInstance_p);
static NDIS_STATUS allocateTxRxBuf(ULONG txBufCount, ULONG rxBufCount);
static void        closeBinding(void);

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief Free VEth instance

Free all resources allocated for the VEth interface and close the binding.

\param  pVEthInstance_p     Pointer to the VEth instance.

\ingroup module_ndis
*/
//------------------------------------------------------------------------------
void protocol_freeVEthInstance(tVEthInstance* pVEthInstance_p)
{
    freeVEthInstance(pVEthInstance_p);
}

//------------------------------------------------------------------------------
/**
\brief  Check protocol binding state

Check the current state for protocol binding.

\return Returns the status for binding.
\retval TRUE if ready.
\retval FALSE If not ready.

\ingroup module_ndis
*/
//------------------------------------------------------------------------------
BOOLEAN protocol_checkBindingState(void)
{
    if (protocolInstance_l.bindingState >= kNdisBindingReady)
        return TRUE;
    else
        return FALSE;
}

//------------------------------------------------------------------------------
/**
\brief  Set binding state

Update the binding state of protocol driver.

\param state_p      Value of new state to set.

\ingroup module_ndis
*/
//------------------------------------------------------------------------------
void protocol_setBindingState(ULONG state_p)
{
    protocolInstance_l.bindingState = (tNdisBindingState)state_p;
}

//------------------------------------------------------------------------------
/**
\brief  Register Tx and Rx handler

NDIS intermediate driver calls the Tx callback from SentNetBufferListsComplete
handler and Rx callback from the NetBufferListsReceive handler.

\param pfnTxCallback_p      Pointer to Tx complete callback.
\param pfnRxCallback_p      Pointer to Rx callback.

\ingroup module_ndis
*/
//------------------------------------------------------------------------------
void protocol_registerTxRxHandler(tNdisTransmitCompleteCb pfnTxCallback_p,
                                  tNdisReceiveCb pfnRxCallback_p)
{
    protocolInstance_l.pfnReceiveCb = pfnRxCallback_p;
    protocolInstance_l.pfnTransmitCompleteCb = pfnTxCallback_p;
}

//------------------------------------------------------------------------------
/**
\brief Allocate Tx and Rx buffers

This driver uses doubly linked list mechanism provided by LIST_ENTRY structure to
manage transmit queues whereas Rx is handled using a circular queue.

The buffers, NetBufferList and MDLs for each of the Tx and Rx buffers are
preallocated to avoid delays in handling individual packets.

\param  txBufCount_p      Number of Tx buffers to be allocated.
\param  rxBufCount_p      Number of Rx buffers to be allocated.

\return The function returns an NDIS_STATUS error code.

\ingroup module_ndis
*/
//------------------------------------------------------------------------------
NDIS_STATUS protocol_allocateTxRxBuf(ULONG txBufCount_p, ULONG rxBufCount_p)
{
    UINT                               index;
    tRxBufInfo*                        rxBufInfo = NULL;
    NDIS_STATUS                        status = NDIS_STATUS_SUCCESS;
    NET_BUFFER_LIST_POOL_PARAMETERS    poolParameters;

    NdisInitializeListHead(&protocolInstance_l.txList);
    NdisAllocateSpinLock(&protocolInstance_l.txListLock);
    NdisZeroMemory(&poolParameters, sizeof(NET_BUFFER_LIST_POOL_PARAMETERS));

    poolParameters.Header.Type = NDIS_OBJECT_TYPE_DEFAULT;
    poolParameters.Header.Revision = NET_BUFFER_LIST_POOL_PARAMETERS_REVISION_1;
    poolParameters.Header.Size = sizeof(poolParameters);
    poolParameters.ProtocolId = NDIS_PROTOCOL_ID_IPX;
    poolParameters.ContextSize = 0;
    poolParameters.fAllocateNetBuffer = TRUE;
    poolParameters.PoolTag = OPLK_MEM_TAG;

    protocolInstance_l.hSendNblPool = NdisAllocateNetBufferListPool(driverInstance_g.hProtocolHandle,
                                                                   &poolParameters);
    if (protocolInstance_l.hSendNblPool == NULL)
    {
        TRACE("%s(): Failed to alloc send net buffer list pool\n", __FUNCTION__);
        status = NDIS_STATUS_RESOURCES;
        goto Exit;
    }

    // Allocate Tx memory
    protocolInstance_l.pTransmitBuf = NdisAllocateMemoryWithTagPriority(driverInstance_g.hProtocolHandle,
                                                                        (OPLK_MAX_FRAME_SIZE * txBufCount_p),
                                                                        OPLK_MEM_TAG, NormalPoolPriority);

    if (protocolInstance_l.pTransmitBuf == NULL)
    {
        TRACE("%s() Failed to allocate Tx buffers\n", __FUNCTION__);
        return NDIS_STATUS_RESOURCES;
    }

    protocolInstance_l.pTxBuffInfo = NdisAllocateMemoryWithTagPriority(driverInstance_g.hProtocolHandle,
                                                                       (sizeof(tTxBufInfo) * txBufCount_p),
                                                                       OPLK_MEM_TAG, NormalPoolPriority);

    if (protocolInstance_l.pTxBuffInfo == NULL)
    {
        TRACE("%s() Failed to allocate Tx buffers info\n", __FUNCTION__);
        status = NDIS_STATUS_RESOURCES;
        goto Exit;
    }

    // Initialize the queue
    for (index = 0; index < txBufCount_p; index++)
    {
        tTxBufInfo*   pTxInfo = &protocolInstance_l.pTxBuffInfo[index];

        if (pTxInfo != NULL)
        {
            pTxInfo->fFree = TRUE;
            pTxInfo->maxLength = OPLK_MAX_FRAME_SIZE;
            pTxInfo->pData = (void*)(((UCHAR*)protocolInstance_l.pTransmitBuf) + (OPLK_MAX_FRAME_SIZE * index));

            // Allocate MDL to define the buffers
            pTxInfo->pMdl = NdisAllocateMdl(protocolInstance_l.hBindingHandle, pTxInfo->pData, OPLK_MAX_FRAME_SIZE);

            if (pTxInfo->pMdl == NULL)
            {
                TRACE("%s() Error Allocating MDL\n", __FUNCTION__);
                status = NDIS_STATUS_RESOURCES;
                goto Exit;
            }

            // Allocate empty NetBufferLists
            pTxInfo->pNbl = NdisAllocateNetBufferAndNetBufferList(protocolInstance_l.hSendNblPool,
                                                                  0, 0, pTxInfo->pMdl, 0, 0);

            if (pTxInfo->pNbl == NULL)
            {
                TRACE("%s() Failed to allocate Tx NBL\n", __FUNCTION__);
                status = NDIS_STATUS_RESOURCES;
                goto Exit;
            }

            // Mark the NetBufferList to indicate that it has been allocated by this protocol driver
            NBL_SET_PROT_RSVD_FLAG(pTxInfo->pNbl, OPLK_ALLOCATED_NBL);
            pTxInfo->pNbl->SourceHandle = protocolInstance_l.hBindingHandle;
            TXINFO_FROM_NBL(pTxInfo->pNbl) = pTxInfo;
            NdisInterlockedInsertTailList(&protocolInstance_l.txList, &pTxInfo->txLink, &protocolInstance_l.txListLock);
        }
    }

    // Allocate Rx buffers
    protocolInstance_l.pReceiveBuf = NdisAllocateMemoryWithTagPriority(driverInstance_g.hProtocolHandle,
                                                                       (OPLK_MAX_FRAME_SIZE * rxBufCount_p),
                                                                       OPLK_MEM_TAG, NormalPoolPriority);
    if (protocolInstance_l.pReceiveBuf == NULL)
    {
        TRACE("%s() Failed to allocate Rx buffers\n", __FUNCTION__);
        status = NDIS_STATUS_RESOURCES;
        goto Exit;
    }

    protocolInstance_l.pReceiveBufInfo = NdisAllocateMemoryWithTagPriority(driverInstance_g.hProtocolHandle,
                                                                           (sizeof(tRxBufInfo) * rxBufCount_p),
                                                                           OPLK_MEM_TAG, NormalPoolPriority);
    if (protocolInstance_l.pReceiveBufInfo == NULL)
    {
        TRACE("%s() Failed to allocate Rx Info buffers\n", __FUNCTION__);
        status = NDIS_STATUS_RESOURCES;
        goto Exit;
    }

    // Setup Rx buffers
    for (index = 0; index < rxBufCount_p; index++)
    {
        rxBufInfo = &protocolInstance_l.pReceiveBufInfo[index];
        rxBufInfo->fFree = TRUE;
        rxBufInfo->length = 0;
        rxBufInfo->maxLength = OPLK_MAX_FRAME_SIZE;
        rxBufInfo->pData = (void*)(((UCHAR*)protocolInstance_l.pReceiveBuf) + (index * OPLK_MAX_FRAME_SIZE));
    }

    protocolInstance_l.receiveBufCount = rxBufCount_p;
    protocolInstance_l.transmitBufCount = txBufCount_p;

Exit:
    if (status != NDIS_STATUS_SUCCESS)
    {
        protocol_freeTxRxBuffers();
    }

    TRACE("OK %x\n", status);
    return status;
}

//------------------------------------------------------------------------------
/**
\brief  Free Tx and Rx buffers

Free Tx and Rx resources allocated during initialization.

\ingroup module_ndis
*/
//------------------------------------------------------------------------------
void protocol_freeTxRxBuffers(void)
{
    PLIST_ENTRY    pTxLink;
    tTxBufInfo*    pTxBufInfo;

    if (protocolInstance_l.txList.Flink != NULL)
    {
        while (!IsListEmpty(&protocolInstance_l.txList))
        {
            pTxLink = NdisInterlockedRemoveHeadList(&protocolInstance_l.txList,
                                                    &protocolInstance_l.txListLock);
            pTxBufInfo = CONTAINING_RECORD(pTxLink, tTxBufInfo, txLink);
            if (pTxBufInfo->pNbl != NULL)
            {
                NdisFreeNetBufferList(pTxBufInfo->pNbl);
            }

            if (pTxBufInfo->pMdl != NULL)
            {
                NdisFreeMdl(pTxBufInfo->pMdl);
            }
        }
    }

    if (protocolInstance_l.hSendNblPool != NULL)
    {
        NdisFreeNetBufferListPool(protocolInstance_l.hSendNblPool);
        protocolInstance_l.hSendNblPool = NULL;
    }

    if (protocolInstance_l.pTxBuffInfo != NULL)
    {
        NdisFreeMemory(protocolInstance_l.pTxBuffInfo,
                       (protocolInstance_l.transmitBufCount * sizeof(tTxBufInfo)), 0);
    }

    if (protocolInstance_l.pTransmitBuf != NULL)
    {
        NdisFreeMemory(protocolInstance_l.pTransmitBuf,
                       (protocolInstance_l.transmitBufCount * OPLK_MAX_FRAME_SIZE),
                       0);
    }

    NdisFreeSpinLock(&protocolInstance_l.txListLock);

    if (protocolInstance_l.pReceiveBuf != NULL)
    {
        NdisFreeMemory(protocolInstance_l.pReceiveBuf,
                       (protocolInstance_l.receiveBufCount * OPLK_MAX_FRAME_SIZE),
                       0);
    }

    if (protocolInstance_l.pReceiveBufInfo != NULL)
    {
        NdisFreeMemory(protocolInstance_l.pReceiveBufInfo,
                       (protocolInstance_l.receiveBufCount * sizeof(tRxBufInfo)),
                       0);
    }
}

//------------------------------------------------------------------------------
/**
\brief  Get free Tx buffer

This routine removes a free Tx buffer entry from the head of the doubly linked
list and returns the pointer to Tx buffer structure to the caller.

\param  size_p      Size of the buffer expected by the caller.

\return Returns pointer to the free Tx buffer info structure.

\ingroup module_ndis
*/
//------------------------------------------------------------------------------
tTxBufInfo* protocol_getTxBuff(size_t size_p)
{
    PLIST_ENTRY    pTxLink;
    tTxBufInfo*    pTxBufInfo = NULL;

    if (IsListEmpty(&protocolInstance_l.txList) || (size_p > OPLK_MAX_FRAME_SIZE))
    {
        return NULL;
    }

    pTxLink = NdisInterlockedRemoveHeadList(&protocolInstance_l.txList,
                                            &protocolInstance_l.txListLock);
    if (pTxLink != NULL)
    {
        pTxBufInfo = CONTAINING_RECORD(pTxLink, tTxBufInfo, txLink);

        if ((pTxBufInfo == NULL))
        {
            return NULL;
        }
    }

    return pTxBufInfo;
}

//------------------------------------------------------------------------------
/**
\brief  Free Tx buffer

Inserts the Tx buffer entry back into the tail of the doubly linked list.

\param  pTxLink_p       Pointer to the LIST_ENTRY of the Tx buff info structure.

\ingroup module_ndis
*/
//------------------------------------------------------------------------------
void protocol_freeTxBuff(PVOID pTxLink_p)
{
    PLIST_ENTRY    pTxLink = NULL;

    if (pTxLink_p != NULL)
    {
        pTxLink = (PLIST_ENTRY)pTxLink_p;
    }

    // Return the buffer to linked list. The resources will be freed later.
    NdisInterlockedInsertTailList(&protocolInstance_l.txList, pTxLink,
                                  &protocolInstance_l.txListLock);
}

//------------------------------------------------------------------------------
/**
\brief  Send OID request to lower miniport driver

Utility routine to query the lower end miniport for a single OID value.
The functions blocks till the query is completed by the lower miniport driver.

\param  requestType_p           The type of the NDIS request.
\param  oid_p                   OID to query for.
\param  oidReqBuffer_p          Buffer to hold the OID request's result.
\param  oidReqBufferLength_p    Size of the buffer.

\return Returns an NDIS_STATUS error code.

\ingroup module_ndis
*/
//------------------------------------------------------------------------------
NDIS_STATUS protocol_sendOidRequest(NDIS_REQUEST_TYPE requestType_p, NDIS_OID oid_p,
                                    PVOID oidReqBuffer_p, ULONG oidReqBufferLength_p)
{
    tNdisOidRequest*   pNdisOidReq = NULL;
    NDIS_STATUS        status;

    // Allocate memory for the new request structure
    pNdisOidReq = NdisAllocateMemoryWithTagPriority(protocolInstance_l.hBindingHandle,
                                                    sizeof(tNdisOidRequest),
                                                    OPLK_MEM_TAG, LowPoolPriority);

    if (pNdisOidReq == NULL)
    {
        return NDIS_STATUS_FAILURE;
    }

    NdisInitializeEvent(&pNdisOidReq->waitEvent);

    // Create the OID request
    pNdisOidReq->oidRequest.Header.Type = NDIS_OBJECT_TYPE_OID_REQUEST;
    pNdisOidReq->oidRequest.Header.Revision = NDIS_OID_REQUEST_REVISION_1;
    pNdisOidReq->oidRequest.Header.Size = sizeof(NDIS_OID_REQUEST);

    pNdisOidReq->oidRequest.RequestType = requestType_p;
    pNdisOidReq->oidRequest.DATA.QUERY_INFORMATION.Oid = oid_p;
    pNdisOidReq->oidRequest.DATA.QUERY_INFORMATION.InformationBuffer = oidReqBuffer_p;
    pNdisOidReq->oidRequest.DATA.QUERY_INFORMATION.InformationBufferLength = oidReqBufferLength_p;

    NdisAcquireSpinLock(&protocolInstance_l.driverLock);
    protocolInstance_l.oidReq++;
    NdisReleaseSpinLock(&protocolInstance_l.driverLock);

    if (protocolInstance_l.bindingState < kNdisBindingReady)
    {
        status = NDIS_STATUS_CLOSING;
    }
    else
    {
        status = NdisOidRequest(protocolInstance_l.hBindingHandle, &pNdisOidReq->oidRequest);
    }

    if (status != NDIS_STATUS_PENDING)
    {
        // Request completed or failed
        NdisAcquireSpinLock(&protocolInstance_l.driverLock);
        protocolInstance_l.oidReq--;

        if (protocolInstance_l.oidReq == 0 && protocolInstance_l.pOidCompleteEvent != NULL)
        {
            NdisSetEvent(protocolInstance_l.pOidCompleteEvent);
            protocolInstance_l.pOidCompleteEvent = NULL;
        }

        NdisReleaseSpinLock(&protocolInstance_l.driverLock);
    }
    else
    {
        // Wait for request to complete
        NdisWaitEvent(&pNdisOidReq->waitEvent, 0);
        status = pNdisOidReq->status;
    }

    if (pNdisOidReq != NULL)
    {
        NdisFreeMemory(pNdisOidReq, sizeof(tNdisOidRequest), 0);
    }

    return status;
}

//------------------------------------------------------------------------------
/**
\brief  Send packet

This routine handles the frames to be sent to the lower miniport driver using
NdisSendNetBufferLists.

\param  pToken_p        Pointer to a token value used to identify the buffer.
\param  size_p          Size of the buffer to send.
\param  pTxLink_p       Pointer to the LIST_ENTRY of the Tx buffer to be sent.

\return Returns an NDIS_STATUS error code.

\ingroup module_ndis
*/
//------------------------------------------------------------------------------
NDIS_STATUS protocol_sendPacket(void* pToken_p, size_t size_p, void* pTxLink_p)
{
    PLIST_ENTRY    pTxLink = (PLIST_ENTRY)pTxLink_p;
    tTxBufInfo*    pTxBufInfo;
    PNET_BUFFER    netBuffer;
    ULONG          sendFlags = 0;

    pTxBufInfo = CONTAINING_RECORD(pTxLink, tTxBufInfo, txLink);

    if (pTxBufInfo == NULL)
    {
        return NDIS_STATUS_INVALID_DATA;
    }

    pTxBufInfo->pToken = pToken_p;

    // Update the size
    netBuffer = NET_BUFFER_LIST_FIRST_NB(pTxBufInfo->pNbl);
    NET_BUFFER_DATA_LENGTH(netBuffer) = size_p;

    // Remove Loop back flag to avoid repeated receives
    sendFlags &= ~NDIS_SEND_FLAGS_CHECK_FOR_LOOPBACK;

    // Forward the packet to lower binding
    NdisSendNetBufferLists(protocolInstance_l.hBindingHandle,
                           pTxBufInfo->pNbl,
                           NDIS_DEFAULT_PORT_NUMBER,
                           sendFlags);

    return NDIS_STATUS_SUCCESS;
}

//------------------------------------------------------------------------------
/**
\brief  Get current MAC address

This routine returns the current MAC address value to the caller.

\return Returns pointer to the buffer holding current MAC address string.

\ingroup module_ndis
*/
//------------------------------------------------------------------------------
UCHAR* protocol_getCurrentMac(void)
{
    tVEthInstance*   pVethInst = (tVEthInstance*)protocolInstance_l.pVEthInstance;

    return &pVethInst->aCurrentAddress[0];
}

//------------------------------------------------------------------------------
/**
\brief  Register VEth Tx handler

Ndis intermediate driver calls the VEth Tx handler from the
miniportSendNetBufferLists routine. The registered handler receives the frames
which are forwarded by the protocol drivers to sent.

\param pfnTxCallback_p      Pointer to VEth transmit callback.

\ingroup module_ndis
*/
//------------------------------------------------------------------------------
void protocol_registerVEthHandler(tVEthSendCb pfnTxCallback_p)
{
    tVEthInstance*      pVethInstance = (tVEthInstance*)protocolInstance_l.pVEthInstance;

    if (pVethInstance == NULL)
        return;

    pVethInstance->pfnVEthSendCb = pfnTxCallback_p;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief  Protocol entry point for bind adapter

Called by NDIS to bind to a miniport below. This routine creates a binding by
calling NdisOpenAdapterEx and then initiates creation of all configured
virtual Ethernet interface on this binding.

\param  protocolDriverContext_p     A pointer to the driver context.
\param  bindContext_p               A pointer to the bind context.
\param  pBindParameters_p           Pointer to related information about
                                    this new binding.

\return The function returns an NDIS_STATUS error code.
*/
//------------------------------------------------------------------------------
NDIS_STATUS protocolBindAdapter(NDIS_HANDLE protocolDriverContext_p,
                                NDIS_HANDLE bindContext_p,
                                PNDIS_BIND_PARAMETERS pBindParameters_p)
{
    NDIS_STATUS             status = NDIS_STATUS_SUCCESS;
    NDIS_OPEN_PARAMETERS    openParameters;
    UINT                    mediumIndex = 0;
    NDIS_STRING             deviceName;
    PNDIS_STRING            pConfigString;

    UNREFERENCED_PARAMETER(protocolDriverContext_p);

    TRACE("%s()...\n", __FUNCTION__);

    if (fBinding_l == TRUE)
    {
        return NDIS_STATUS_SUCCESS;
    }

    pConfigString = (PNDIS_STRING)pBindParameters_p->ProtocolSection;

    TRACE("Binding to Adapter: %ws\n", pConfigString->Buffer);

    if (driverInstance_g.hProtocolHandle == NULL)
    {
        // Driver not registered yet. Is this even possible?
        status = NDIS_STATUS_RESOURCES;
        goto Exit;
    }

    NdisInitializeEvent(&protocolInstance_l.adapterEvent);
    NdisAllocateSpinLock(&protocolInstance_l.pauseEventLock);
    NdisAllocateSpinLock(&protocolInstance_l.driverLock);

    // Set the state to Paused till we complete the initialization
    protocolInstance_l.bindingState = kNdisBindingPaused;

    protocolInstance_l.lastLinkState.Header.Revision = NDIS_LINK_STATE_REVISION_1;
    protocolInstance_l.lastLinkState.Header.Type = NDIS_OBJECT_TYPE_DEFAULT;
    protocolInstance_l.lastLinkState.Header.Size = sizeof(NDIS_LINK_STATE);
    protocolInstance_l.lastLinkState.MediaConnectState = pBindParameters_p->MediaConnectState;
    protocolInstance_l.lastLinkState.MediaDuplexState = pBindParameters_p->MediaDuplexState;
    protocolInstance_l.lastLinkState.XmitLinkSpeed = pBindParameters_p->XmitLinkSpeed;
    protocolInstance_l.lastLinkState.RcvLinkSpeed = pBindParameters_p->RcvLinkSpeed;

    // Now open the adapter below and complete the initialization
    NdisZeroMemory(&openParameters, sizeof(NDIS_OPEN_PARAMETERS));

    openParameters.Header.Type = NDIS_OBJECT_TYPE_OPEN_PARAMETERS;
    openParameters.Header.Revision = NDIS_OPEN_PARAMETERS_REVISION_1;
    openParameters.Header.Size = sizeof(NDIS_OPEN_PARAMETERS);
    openParameters.AdapterName = pBindParameters_p->AdapterName;
    openParameters.MediumArray = aMediumArray_l;
    openParameters.MediumArraySize = sizeof(aMediumArray_l) / sizeof(NDIS_MEDIUM);
    openParameters.SelectedMediumIndex = &mediumIndex;
    openParameters.FrameTypeArray = NULL;
    openParameters.FrameTypeArraySize = 0;

    NDIS_DECLARE_PROTOCOL_OPEN_CONTEXT(tProtocolInstance);
    status = NdisOpenAdapterEx(driverInstance_g.hProtocolHandle,
                               &protocolInstance_l,
                               &openParameters,
                               bindContext_p,
                               &protocolInstance_l.hBindingHandle);

    if (status == NDIS_STATUS_PENDING)
    {
        // Wait for initialization of the adapter to complete
        NdisWaitEvent(&protocolInstance_l.adapterEvent, 0);
        status = protocolInstance_l.adapterInitStatus;
    }

    if (status != NDIS_STATUS_SUCCESS)
    {
        goto ExitFail;
    }

    protocolInstance_l.bindParameters = *pBindParameters_p;

    if (pBindParameters_p->RcvScaleCapabilities)
    {
        protocolInstance_l.bindParameters.RcvScaleCapabilities = pBindParameters_p->RcvScaleCapabilities;
    }

    // Zeroing out fields that are not needed by the driver
    protocolInstance_l.bindParameters.ProtocolSection = NULL;
    protocolInstance_l.bindParameters.AdapterName = NULL;
    protocolInstance_l.bindParameters.PhysicalDeviceObject = NULL;

    // Initialize VETH instance
    // TODO: Identify the lower bindings and select one before starting as we support only one lower binding
    status = startVEth(&deviceName);

    if (status != NDIS_STATUS_SUCCESS)
    {
        goto ExitFail;
    }

    fBinding_l = TRUE;
    goto Exit;

ExitFail:
    if (status != NDIS_STATUS_SUCCESS)
    {
        if (protocolInstance_l.hBindingHandle != NULL)
        {
            closeBinding();
            protocolInstance_l.hBindingHandle = NULL;
        }
    }

Exit:
    TRACE("%s() - OK\n", __FUNCTION__);
    return status;
}

//------------------------------------------------------------------------------
/**
\brief  Open adapter complete callback

Completion routine for NdisOpenAdapter issued from within the bind-adapter.
Unblock the caller to mark completion.

\param  protocolBindingContext_p    Pointer to the protocol instance.
\param  status_p                    Status of the NdisOpenAdapter call.

*/
//------------------------------------------------------------------------------
VOID protocolOpenAdapterComplete(NDIS_HANDLE protocolBindingContext_p, NDIS_STATUS status_p)
{
    tProtocolInstance*   protInstance = (tProtocolInstance*)protocolBindingContext_p;
    TRACE("%s()...\n", __FUNCTION__);
    protInstance->adapterInitStatus = status_p;
    NdisSetEvent(&protInstance->adapterEvent);
    TRACE("%s() - OK\n", __FUNCTION__);
}

//------------------------------------------------------------------------------
/**
\brief  Unbind adapter callback

Called by NDIS when it is required to unbind from the adapter below. Stop the VEth
instance and close the binding to lower miniport.

\param  unbindContext_p             Context for NdisUnbindComplete() if the
                                    call is deferred.
\param  protocolBindingContext_p    Pointer to the protocol instance structure.

\return The function returns an NDIS_STATUS error code.
*/
//------------------------------------------------------------------------------
NDIS_STATUS protocolUnbindAdapter(NDIS_HANDLE unbindContext_p,
                                  NDIS_HANDLE protocolBindingContext_p)
{
    NDIS_STATUS      status = NDIS_STATUS_SUCCESS;
    tVEthInstance*   pVEthInstance = NULL;
    INT              waitCount;

    UNREFERENCED_PARAMETER(protocolBindingContext_p);
    UNREFERENCED_PARAMETER(unbindContext_p);

    TRACE("%s()...\n", __FUNCTION__);

    if (protocolInstance_l.pVEthInstance != NULL)
    {
        pVEthInstance = (tVEthInstance*)protocolInstance_l.pVEthInstance;
        stopVEth(pVEthInstance);
        protocolInstance_l.pVEthInstance = NULL;
    }

    // Close the binding to the lower adapter.
    if (protocolInstance_l.hBindingHandle != NULL)
    {
        closeBinding();
    }
    else
    {
        // Binding handle can not be NULL at this point.
        status = NDIS_STATUS_FAILURE;
        ASSERT(0);
    }

    fBinding_l = FALSE;

    TRACE("%s() - OK\n", __FUNCTION__);
    return status;
}

//------------------------------------------------------------------------------
/**
\brief  Close adapter complete callback

Completion routine for the NdisCloseAdapterEx call.

\param  protocolBindingContext_p    Pointer to the protocol instance structure.

*/
//------------------------------------------------------------------------------
VOID protocolCloseAdapterComplete(NDIS_HANDLE protocolBindingContext_p)
{
    UNREFERENCED_PARAMETER(protocolBindingContext_p);

    TRACE("%s()... \n", __FUNCTION__);
    NdisSetEvent(&protocolInstance_l.adapterEvent);
    TRACE("%s() - OK \n", __FUNCTION__);
}

//------------------------------------------------------------------------------
/**
\brief Request complete callback

Completion handler for an NDIS request sent to a lower miniport.

\param  protocolBindingContext_p    Pointer to the protocol instance structure.
\param  pNdisRequest_p              The completed request.
\param  status_p                    Completion status.

*/
//------------------------------------------------------------------------------
VOID protocolRequestComplete(NDIS_HANDLE protocolBindingContext_p,
                             PNDIS_OID_REQUEST pNdisRequest_p,
                             NDIS_STATUS status_p)
{
    tNdisOidRequest*   pNdisOidRequest = NULL;

    UNREFERENCED_PARAMETER(protocolBindingContext_p);

    pNdisOidRequest = CONTAINING_RECORD(pNdisRequest_p, tNdisOidRequest, oidRequest);

    // Complete the request
    pNdisOidRequest->status = status_p;
    NdisSetEvent(&pNdisOidRequest->waitEvent);

    NdisAcquireSpinLock(&protocolInstance_l.driverLock);
    // Decrement the request count
    protocolInstance_l.oidReq--;

    // If no requests are pending and driver is waiting for the pending requests to complete,
    // set the event to mark completion.
    if (protocolInstance_l.oidReq == 0 && protocolInstance_l.pOidCompleteEvent != NULL)
    {
        NdisSetEvent(protocolInstance_l.pOidCompleteEvent);
        protocolInstance_l.pOidCompleteEvent = NULL;
    }

    NdisReleaseSpinLock(&protocolInstance_l.driverLock);
}

//------------------------------------------------------------------------------
/**
\brief  Status indication callback

Handle a status indication of the lower binding. If this is a media status
indication, we also pass this on to VEth.

\param  protocolBindingContext_p    Pointer to the protocol instance structure.
\param  pStatusIndication_p         Status buffer.

*/
//------------------------------------------------------------------------------
VOID protocolStatus(NDIS_HANDLE protocolBindingContext_p,
                    PNDIS_STATUS_INDICATION pStatusIndication_p)
{
    NDIS_STATUS               generalStatus = pStatusIndication_p->StatusCode;
    NDIS_STATUS_INDICATION    newStatusIndication;
    tVEthInstance*            pVEthinstance = (tVEthInstance*)protocolInstance_l.pVEthInstance;

    UNREFERENCED_PARAMETER(protocolBindingContext_p);

    TRACE("%s()... \n", __FUNCTION__);

    if (pVEthinstance == NULL)
    {
        return;
    }

    if (generalStatus != NDIS_STATUS_LINK_STATE)
    {
        // We only handle Link states here
        return;
    }

    protocolInstance_l.lastLinkState = *((PNDIS_LINK_STATE)(pStatusIndication_p->StatusBuffer));

    if (pVEthinstance->fMiniportHalting || (pVEthinstance->hMiniportAdapterHandle == NULL))
    {
        pVEthinstance->pendingStatusIndication = generalStatus;
        pVEthinstance->lastPendingLinkState = *((PNDIS_LINK_STATE)(pStatusIndication_p->StatusBuffer));
        return;
    }

    pVEthinstance->lastLinkStatus = generalStatus;
    if (generalStatus != NDIS_STATUS_LINK_STATE)
    {
        pVEthinstance->lastLinkState = *((PNDIS_LINK_STATE)(pStatusIndication_p->StatusBuffer));
    }

    // Allocate a new status indication and pass it to protocols bound to VEth
    NdisZeroMemory(&newStatusIndication, sizeof(NDIS_STATUS_INDICATION));

    newStatusIndication.Header.Type = NDIS_OBJECT_TYPE_STATUS_INDICATION;
    newStatusIndication.Header.Revision = NDIS_STATUS_INDICATION_REVISION_1;
    newStatusIndication.Header.Size = sizeof(NDIS_STATUS_INDICATION);

    newStatusIndication.StatusCode = pStatusIndication_p->StatusCode;
    newStatusIndication.SourceHandle = pVEthinstance->hMiniportAdapterHandle;
    newStatusIndication.DestinationHandle = NULL;

    newStatusIndication.StatusBuffer = pStatusIndication_p->StatusBuffer;
    newStatusIndication.StatusBufferSize = pStatusIndication_p->StatusBufferSize;

    NdisMIndicateStatusEx(pVEthinstance->hMiniportAdapterHandle, &newStatusIndication);

    TRACE("%s() - OK \n", __FUNCTION__);
}

//------------------------------------------------------------------------------
/**
\brief  PNP callback

This routine is called by NDIS to notify us of a PNP event related to a lower
binding.

\param  protocolBindingContext_p        Pointer to the protocol instance structure.
\param  pNetPnpEventNotification_p      Pointer to the PNP event to be processed.

\return The function returns an NDIS_STATUS error code.
*/
//------------------------------------------------------------------------------
NDIS_STATUS protocolPnpHandler(NDIS_HANDLE protocolBindingContext_p,
                               PNET_PNP_EVENT_NOTIFICATION pNetPnpEventNotification_p)
{
    NDIS_STATUS    status = NDIS_STATUS_SUCCESS;
    NDIS_EVENT     pPauseEvent;
    ULONG          packetFilter = NDIS_PACKET_TYPE_PROMISCUOUS;

    UNREFERENCED_PARAMETER(protocolBindingContext_p);

    TRACE("%s()... \n", __FUNCTION__);

    switch (pNetPnpEventNotification_p->NetPnPEvent.NetEvent)
    {
        case NetEventSetPower:
            // Driver does not handles power state configuration with the current
            // implementation
            status = NDIS_STATUS_SUCCESS;
            break;

        case NetEventReconfigure:
            status = NDIS_STATUS_SUCCESS;
            break;

        case NetEventIMReEnableDevice:
            status = NDIS_STATUS_SUCCESS;
            break;

        case NetEventPause:
            NdisAcquireSpinLock(&protocolInstance_l.driverLock);

            ASSERT(protocolInstance_l.pPauseEvent == NULL);
            // Wait for all the send requests to complete
            if (protocolInstance_l.sendRequest != 0)
            {
                NdisInitializeEvent(&pPauseEvent);

                protocolInstance_l.pPauseEvent = &pPauseEvent;

                NdisReleaseSpinLock(&protocolInstance_l.driverLock);

                NdisWaitEvent(&pPauseEvent, 0);
                NdisAcquireSpinLock(&protocolInstance_l.driverLock);
            }

            protocolInstance_l.bindingState = kNdisBindingPaused;

            NdisReleaseSpinLock(&protocolInstance_l.driverLock);
            status = NDIS_STATUS_SUCCESS;
            break;

        case NetEventRestart:
            protocolInstance_l.bindingState = kNdisBindingReady;
            protocol_sendOidRequest(NdisRequestSetInformation,
                                    OID_GEN_CURRENT_PACKET_FILTER, &packetFilter,
                                    sizeof(packetFilter));
            status = NDIS_STATUS_SUCCESS;
            break;

        default:
            status = NDIS_STATUS_SUCCESS;
            break;
    }

    TRACE("%s() - OK\n", __FUNCTION__);
    return status;
}

//------------------------------------------------------------------------------
/**
\brief  Receive NetBufferLists handler

Handles the receive indication from the lower miniport and parses the NetBufferLists
to retrieve the buffer and pass it to the registered callbacks for Rx.

\param  protocolBindingContext_p        Pointer to the protocol instance structure.
\param  pNetBufferLists_p               Net Buffer Lists received.
\param  portNumber_p                    Port on which Net Buffer Lists were received.
\param  numberOfNbl_p                   Number of Net Buffer Lists.
\param  receiveFlags_p                  Flags associated with the receive.

*/
//------------------------------------------------------------------------------
VOID protocolReceiveNbl(NDIS_HANDLE protocolBindingContext_p,
                        PNET_BUFFER_LIST pNetBufferLists_p,
                        NDIS_PORT_NUMBER portNumber_p, ULONG numberOfNbl_p,
                        ULONG receiveFlags_p)
{
    PNET_BUFFER_LIST    pCurrentNbl = NULL;
    PNET_BUFFER_LIST    pReturnNbl = NULL;
    PNET_BUFFER_LIST    pLastReturnNbl = NULL;
    ULONG               returnFlags = 0;
    PMDL                pMdl;
    ULONG               offset = 0;
    ULONG               totalLength;

    UNREFERENCED_PARAMETER(numberOfNbl_p);
    UNREFERENCED_PARAMETER(portNumber_p);
    UNREFERENCED_PARAMETER(protocolBindingContext_p);

    if (NDIS_TEST_RECEIVE_AT_DISPATCH_LEVEL(receiveFlags_p))
    {
        NDIS_SET_RETURN_FLAG(returnFlags, NDIS_RETURN_FLAGS_DISPATCH_LEVEL);
    }

    ASSERT(pNetBufferLists_p != NULL);

    // Return NetBufferList immediately if the binding is not in running state
    if (protocolInstance_l.bindingState != kNdisBindingRunning)
    {
        if (NDIS_TEST_RECEIVE_CAN_PEND(receiveFlags_p) == TRUE)
        {
            NdisReturnNetBufferLists(protocolInstance_l.hBindingHandle,
                                     pNetBufferLists_p,
                                     returnFlags);
        }

        return;
    }

    while (pNetBufferLists_p != NULL)
    {
        ULONG         bytesAvailable = 0;
        PUCHAR        pRxDataSrc;
        PUCHAR        pRxDataDest;
        ULONG         bytesToCopy = 0;
        tRxBufInfo*   rxBufInfo = NULL;

        pCurrentNbl = pNetBufferLists_p;
        pNetBufferLists_p = NET_BUFFER_LIST_NEXT_NBL(pNetBufferLists_p);
        NET_BUFFER_LIST_NEXT_NBL(pCurrentNbl) = NULL;

        rxBufInfo = &protocolInstance_l.pReceiveBufInfo[protocolInstance_l.receiveHead];
        pMdl = NET_BUFFER_CURRENT_MDL(NET_BUFFER_LIST_FIRST_NB(pCurrentNbl));
        totalLength = NET_BUFFER_DATA_LENGTH(NET_BUFFER_LIST_FIRST_NB(pCurrentNbl));
        if (rxBufInfo->maxLength >= totalLength)
        {
            rxBufInfo->length = totalLength;
        }

        offset = NET_BUFFER_CURRENT_MDL_OFFSET(NET_BUFFER_LIST_FIRST_NB(pCurrentNbl));
        pRxDataDest = rxBufInfo->pData;

        while ((pMdl != NULL) && (totalLength > 0))
        {
            pRxDataSrc = NULL;
            NdisQueryMdl(pMdl, &pRxDataSrc, &bytesAvailable, NormalPagePriority);
            if ((pRxDataSrc == NULL) || (pRxDataDest == NULL))
            {
                break;
            }

            bytesToCopy = bytesAvailable - offset;
            bytesToCopy = min(bytesToCopy, totalLength);

            NdisMoveMemory(pRxDataDest, pRxDataSrc, bytesToCopy);
            pRxDataDest = (PVOID)((UCHAR*)pRxDataDest + bytesToCopy);
            totalLength -= bytesToCopy;

            offset = 0;
            NdisGetNextMdl(pMdl, &pMdl);
        }

        if (protocolInstance_l.pfnReceiveCb != NULL)
        {
            protocolInstance_l.pfnReceiveCb(rxBufInfo->pData, rxBufInfo->length);
        }

        protocolInstance_l.receiveHead = (protocolInstance_l.receiveHead + 1) &
                                         (protocolInstance_l.receiveBufCount - 1);

        if (NDIS_TEST_RECEIVE_CAN_PEND(receiveFlags_p) == TRUE)
        {
            if (pReturnNbl == NULL)
            {
                pReturnNbl = pCurrentNbl;
            }
            else
            {
                NET_BUFFER_LIST_NEXT_NBL(pLastReturnNbl) = pCurrentNbl;
            }

            pLastReturnNbl = pCurrentNbl;
            NET_BUFFER_LIST_NEXT_NBL(pLastReturnNbl) = NULL;
        }
        else
        {
            // Restore the NetBufferList chain
            NET_BUFFER_LIST_NEXT_NBL(pCurrentNbl) = pNetBufferLists_p;
        }
    }

    if (pReturnNbl != NULL)
    {
        NdisReturnNetBufferLists(protocolInstance_l.hBindingHandle,
                                 pReturnNbl,
                                 returnFlags);
    }
}

//------------------------------------------------------------------------------
/**
\brief  Send Complete callback

Called by NDIS when the miniport below has completed a send. We call the Tx complete
callback for the specific buffer.

\param  protocolBindingContext_p        Pointer to the protocol instance structure.
\param  pNetBufferLists_p               Packet being completed by the lower miniport
\param  sendCompleteFlags_p             Is the call at DispatchLevel.

*/
//------------------------------------------------------------------------------
VOID protocolSendNblComplete(NDIS_HANDLE protocolBindingContext_p,
                             PNET_BUFFER_LIST pNetBufferLists_p,
                             ULONG sendCompleteFlags_p)
{
    PNET_BUFFER_LIST    pCurrentNbl;
    PLIST_ENTRY         pTxLink;
    tTxBufInfo*         pTxBufInfo;

    UNREFERENCED_PARAMETER(sendCompleteFlags_p);
    UNREFERENCED_PARAMETER(protocolBindingContext_p);

    while (pNetBufferLists_p)
    {
        pCurrentNbl = pNetBufferLists_p;
        pNetBufferLists_p = NET_BUFFER_LIST_NEXT_NBL(pNetBufferLists_p);
        NET_BUFFER_LIST_NEXT_NBL(pCurrentNbl) = NULL;

        if (NBL_TEST_PROT_RSVD_FLAG(pCurrentNbl, OPLK_ALLOCATED_NBL))
        {
            pTxLink = (PLIST_ENTRY)TXINFO_FROM_NBL(pCurrentNbl);

            if (pTxLink == NULL)
            {
                break;
            }

            pTxBufInfo = CONTAINING_RECORD(pTxLink, tTxBufInfo, txLink);
            if (protocolInstance_l.pfnTransmitCompleteCb != NULL)
            {
                protocolInstance_l.pfnTransmitCompleteCb(pTxBufInfo->pToken);
            }
        }
    }
}

//------------------------------------------------------------------------------
/**
\brief  Close lower binding

Severs the binding with the lower miniport by calling NdisCloseAdapter and frees
all resources allocated for the protocol driver.

*/
//------------------------------------------------------------------------------
static void closeBinding(void)
{
    NDIS_STATUS    status;
    NDIS_EVENT     oidCompleteEvent;

    TRACE("%s()... \n", __FUNCTION__);

    protocolInstance_l.bindingState = kNdisBindingPausing;

    NdisAcquireSpinLock(&protocolInstance_l.driverLock);

    // Wait for all OID requests to be completed
    if (protocolInstance_l.oidReq != 0)
    {
        NdisInitializeEvent(&oidCompleteEvent);
        protocolInstance_l.pOidCompleteEvent = &oidCompleteEvent;
        NdisReleaseSpinLock(&protocolInstance_l.driverLock);
        NdisWaitEvent(protocolInstance_l.pOidCompleteEvent, 0);
        NdisAcquireSpinLock(&protocolInstance_l.driverLock);
    }

    // Free NBL pool
    NdisFreeNetBufferListPool(protocolInstance_l.hSendNblPool);
    NdisReleaseSpinLock(&protocolInstance_l.driverLock);

    NdisResetEvent(&protocolInstance_l.adapterEvent);

    // Close binding with lower layer miniport
    status = NdisCloseAdapterEx(protocolInstance_l.hBindingHandle);

    if (status == NDIS_STATUS_PENDING)
    {
        NdisWaitEvent(&protocolInstance_l.adapterEvent, 0);
    }

    // Binding closed
    protocolInstance_l.hBindingHandle = NULL;

    TRACE("%s() - OK\n", __FUNCTION__);
}

//------------------------------------------------------------------------------
/**
\brief  Start VEth interface

Creates the necessary resources for the VEth adapter and initializes the VEth
interface using NdisIMInitializeDeviceInstanceEx() routine. This results in
a call to Miniport initialize routine of the Miniport section.

\param  pInstanceName_p      Instance name to be used to open the VEth interface.

\return Returns NDIS_STATUS error code.

*/
//------------------------------------------------------------------------------
static NDIS_STATUS startVEth(PNDIS_STRING pInstanceName_p)
{
    NDIS_STATUS                      status = NDIS_STATUS_SUCCESS;
    tVEthInstance*                   pVEthInstance = NULL;
    NDIS_HANDLE                      adapterConfigHandle;
    PNDIS_CONFIGURATION_PARAMETER    pConfigParam;
    NDIS_STRING                      upperBindingStr = NDIS_STRING_CONST("UpperBindings");
    PWSTR                            devName;
    NDIS_CONFIGURATION_OBJECT        configObject;

    TRACE("%s()...\n", __FUNCTION__);

    if (protocolInstance_l.hBindingHandle == NULL)
    {
        // Adapter instance is not ready
        return status;
    }

    NdisZeroMemory(&configObject, sizeof(NDIS_CONFIGURATION_OBJECT));
    // Get the configuration of lower binding
    configObject.Header.Type = NDIS_OBJECT_TYPE_CONFIGURATION_OBJECT;
    configObject.Header.Revision = NDIS_CONFIGURATION_OBJECT_REVISION_1;
    configObject.Header.Size = sizeof(NDIS_CONFIGURATION_OBJECT);
    configObject.NdisHandle = protocolInstance_l.hBindingHandle;
    configObject.Flags = 0;

    status = NdisOpenConfigurationEx(&configObject, &adapterConfigHandle);

    if (status != NDIS_STATUS_SUCCESS)
    {
        adapterConfigHandle = NULL;
        TRACE("%s() Failed to open adapter configuration\n", __FUNCTION__);
        return NDIS_STATUS_OPEN_FAILED;
    }

    // Read the "UpperBindings" reserved key that contains a list
    // of device names representing our miniport instances corresponding
    // to this lower binding. The UpperBindings is a
    // MULTI_SZ containing a list of device names. We will loop through
    // this list and initialize the virtual miniports.

    NdisReadConfiguration(&status,
                          &pConfigParam,
                          adapterConfigHandle,
                          &upperBindingStr,
                          NdisParameterMultiString);

    if (status != NDIS_STATUS_SUCCESS)
    {
        TRACE("Unable to read configuration\n");
        return status;
    }

    devName = pConfigParam->ParameterData.StringData.Buffer;

    while (*devName != L'\0')
    {
        NDIS_STRING    devString;
        ULONG          length;

        NdisInitUnicodeString(&devString, devName);

        if (pInstanceName_p != NULL)
        {
                length = sizeof(tVEthInstance) + devString.Length + sizeof(WCHAR);
                // Allocate a new VEth instance
                pVEthInstance = NdisAllocateMemoryWithTagPriority(protocolInstance_l.hBindingHandle,
                                                                  length, OPLK_MEM_TAG, NormalPoolPriority);
                if (pVEthInstance == NULL)
                {
                    TRACE("%s() Failed to allocate memory for VEth instance length %x\n", __FUNCTION__, length);
                    goto ExitFail;
                }

                NdisZeroMemory(pVEthInstance, length);
                pVEthInstance->cfgDeviceName.Length = 0;
                pVEthInstance->cfgDeviceName.Buffer = (PWCHAR)((PUCHAR)pVEthInstance + sizeof(tVEthInstance));
                pVEthInstance->cfgDeviceName.MaximumLength = devString.Length + sizeof(WCHAR);
                (void)NdisUpcaseUnicodeString(&pVEthInstance->cfgDeviceName, &devString);
                pVEthInstance->cfgDeviceName.Buffer[devString.Length / sizeof(WCHAR)] = ((WCHAR)0);

                // Setup initial miniport parameters
                pVEthInstance->lastLinkStatus = NDIS_STATUS_LINK_STATE;

                pVEthInstance->lookAhead = protocolInstance_l.bindParameters.LookaheadSize;
                pVEthInstance->linkSpeed = protocolInstance_l.bindParameters.RcvLinkSpeed;

                if (protocolInstance_l.bindParameters.MacAddressLength == 6)
                {
                    NdisMoveMemory(pVEthInstance->aPermanentAddress,
                                   &protocolInstance_l.bindParameters.CurrentMacAddress,
                                   protocolInstance_l.bindParameters.MacAddressLength);

                    NdisMoveMemory(pVEthInstance->aCurrentAddress,
                                   &protocolInstance_l.bindParameters.CurrentMacAddress,
                                   protocolInstance_l.bindParameters.MacAddressLength);
                }
                else
                {
                    status = NDIS_STATUS_NOT_SUPPORTED;
                    goto ExitFail;
                }

                NdisAllocateSpinLock(&pVEthInstance->miniportLock);
                NdisAllocateSpinLock(&pVEthInstance->pauseLock);

                pVEthInstance->fMiniportInitPending = TRUE;
                NdisInitializeEvent(&pVEthInstance->miniportInitEvent);

                pVEthInstance->hBindingHandle = protocolInstance_l.hBindingHandle;
                pVEthInstance->pProtocolInstance = &protocolInstance_l;

                status = NdisIMInitializeDeviceInstanceEx(driverInstance_g.hMiniportHandle,
                                                          &pVEthInstance->cfgDeviceName,
                                                          pVEthInstance);

                if (status != NDIS_STATUS_SUCCESS)
                {
                    TRACE("Failed to initialize miniport %x\n", status);
                    if (!pVEthInstance->fMiniportHalting)
                    {
                        goto ExitFail;
                    }
                }
        }

        devName = (PWSTR)((PUCHAR)devName + devString.Length + sizeof(WCHAR));
        break;
    }

    if (adapterConfigHandle != NULL)
    {
        NdisCloseConfiguration(adapterConfigHandle);
    }

    protocolInstance_l.pVEthInstance = (void*)pVEthInstance;
    goto Exit;

ExitFail:
    if (status != NDIS_STATUS_SUCCESS)
    {
        if (pVEthInstance)
        {
            freeVEthInstance(pVEthInstance);
            pVEthInstance = NULL;
        }
    }

Exit:
    TRACE("%s() - OK\n", __FUNCTION__);
    return status;
}

//------------------------------------------------------------------------------
/**
\brief  Free VEth Instance

Free VEth instance structure

\param  pVEthInstance_p     Pointer to VEth instance

*/
//------------------------------------------------------------------------------
static void freeVEthInstance(tVEthInstance* pVEthInstance_p)
{
    if (pVEthInstance_p != NULL)
    {
        NdisFreeSpinLock(&pVEthInstance_p->pauseLock);
        NdisFreeSpinLock(&pVEthInstance_p->miniportLock);
        NdisFreeMemory(pVEthInstance_p, 0, 0);
    }
}

//------------------------------------------------------------------------------
/**
\brief  Stop VEth

Close Virtual miniport adapter and free resources allocated for VEth instance.

\param  pVEthInstance_p     Pointer to VEth instance.

*/
//------------------------------------------------------------------------------
static void stopVEth(tVEthInstance* pVEthInstance_p)
{
    NDIS_STATUS    status;
    BOOLEAN        fMiniportInitCancelled = FALSE;

    TRACE("%s()... \n", __FUNCTION__);

    NdisAcquireSpinLock(&pVEthInstance_p->miniportLock);

    if (pVEthInstance_p->fOidRequestPending)
    {
        pVEthInstance_p->fOidRequestPending = FALSE;
        NdisReleaseSpinLock(&pVEthInstance_p->miniportLock);
        protocolRequestComplete(&protocolInstance_l, &pVEthInstance_p->ndisOidReq.oidRequest,
                                NDIS_STATUS_FAILURE);
    }
    else
    {
        NdisReleaseSpinLock(&pVEthInstance_p->miniportLock);
    }

    if (pVEthInstance_p->fMiniportInitPending)
    {
        // Initialization has not completed but driver is exiting, so cancel the initialization.
        status = NdisIMCancelInitializeDeviceInstance(driverInstance_g.hMiniportHandle,
                                                      &pVEthInstance_p->cfgDeviceName);
        if (status != NDIS_STATUS_SUCCESS)
        {
            pVEthInstance_p->fMiniportInitPending = FALSE;
            fMiniportInitCancelled = TRUE;
        }
        else
        {
            NdisWaitEvent(&pVEthInstance_p->miniportInitEvent, 200);
        }
    }

    if (pVEthInstance_p->hMiniportAdapterHandle != NULL &&
        (!pVEthInstance_p->fMiniportHalting))
    {
        // Stop the miniport
        (void)NdisIMDeInitializeDeviceInstance(pVEthInstance_p->hMiniportAdapterHandle);
    }
    else
    {
        if (fMiniportInitCancelled)
        {
            freeVEthInstance(pVEthInstance_p);
        }
    }

    TRACE("%s() - OK\n", __FUNCTION__);
}

/// \}
