/**
********************************************************************************
\file   ndis-imMiniport.c

\brief  Miniport implementation of NDIS intermediate driver

This implements the miniport section of the NDIS intermediate driver. The
miniport provides a communication interface for other NDIS protocol drivers.
This interface will act as a virtual Ethernet interface with which socket based
communication can be achieved for non-PLK packets.

The current implementation has only basic miniport functionalities such as
send, receive, lower layer binding and communication.

The following features will be added in future:
1. Statistics for send and receive.
2. Ethernet status update and state change handling.
3. OID request handling.
4. Additional PNP capabilities.

\ingroup ndis_intermediate
*******************************************************************************/

/*------------------------------------------------------------------------------k
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
// List of OIDs supported by this miniport.
NDIS_OID                aVEthSupportedOids[] =
{
    OID_GEN_SUPPORTED_LIST,
    OID_GEN_HARDWARE_STATUS,
    OID_GEN_MEDIA_SUPPORTED,
    OID_GEN_MEDIA_IN_USE,
    OID_GEN_MAXIMUM_LOOKAHEAD,
    OID_GEN_MAXIMUM_FRAME_SIZE,
    OID_GEN_LINK_SPEED,
    OID_GEN_TRANSMIT_BUFFER_SPACE,
    OID_GEN_RECEIVE_BUFFER_SPACE,
    OID_GEN_TRANSMIT_BLOCK_SIZE,
    OID_GEN_RECEIVE_BLOCK_SIZE,
    OID_GEN_VENDOR_ID,
    OID_GEN_VENDOR_DESCRIPTION,
    OID_GEN_VENDOR_DRIVER_VERSION,
    OID_GEN_CURRENT_PACKET_FILTER,
    OID_GEN_CURRENT_LOOKAHEAD,
    OID_GEN_DRIVER_VERSION,
    OID_GEN_MAXIMUM_TOTAL_SIZE,
    OID_GEN_PROTOCOL_OPTIONS,
    OID_GEN_MAC_OPTIONS,
    OID_GEN_MEDIA_CONNECT_STATUS,
    OID_GEN_MAXIMUM_SEND_PACKETS,
    OID_GEN_XMIT_OK,
    OID_GEN_RCV_OK,
    OID_GEN_XMIT_ERROR,
    OID_GEN_RCV_ERROR,
    OID_GEN_RCV_NO_BUFFER,
    OID_GEN_RCV_CRC_ERROR,
    OID_GEN_TRANSMIT_QUEUE_LENGTH,
    OID_GEN_STATISTICS,
    OID_802_3_PERMANENT_ADDRESS,
    OID_802_3_CURRENT_ADDRESS,
    OID_802_3_MULTICAST_LIST,
    OID_802_3_MAXIMUM_LIST_SIZE,
    OID_802_3_RCV_ERROR_ALIGNMENT,
    OID_802_3_XMIT_ONE_COLLISION,
    OID_802_3_XMIT_MORE_COLLISIONS,
    OID_802_3_XMIT_DEFERRED,
    OID_802_3_XMIT_MAX_COLLISIONS,
    OID_802_3_RCV_OVERRUN,
    OID_802_3_XMIT_UNDERRUN,
    OID_802_3_XMIT_HEARTBEAT_FAILURE,
    OID_802_3_XMIT_TIMES_CRS_LOST,
    OID_802_3_XMIT_LATE_COLLISIONS,
    OID_PNP_CAPABILITIES,
    OID_PNP_SET_POWER,
    OID_PNP_QUERY_POWER,
    OID_PNP_ADD_WAKE_UP_PATTERN,
    OID_PNP_REMOVE_WAKE_UP_PATTERN,
    OID_PNP_ENABLE_WAKE_UP
};

static BOOLEAN          fInitialize_l = FALSE;
static tVEthInstance*   pVEthInstance_l;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static void        miniportFreeVethRxBuff(tVEthInstance* pVEthInstance_p);
static NDIS_STATUS miniportAllocateVEthRxBuff(tVEthInstance* pVEthInstance_p);

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Miniport receive handler

This is the receive handler for the NDIS miniport. The receive handler forwards
the received frames to the protocol drivers enabled on this miniport.

\param  pDataBuff_p     Pointer to the data buffer.
\param  size_p          Size of the buffer.

\return The function returns an NDIS_STATUS error code.

\ingroup module_ndis
*/
//------------------------------------------------------------------------------
NDIS_STATUS miniport_handleReceive(void* pDataBuff_p, size_t size_p)
{
    PLIST_ENTRY        pRxLink;
    tVEthRcvBufInfo*   pVethRxInfo = NULL;
    PNET_BUFFER        pNetBuffer = NULL;

    if (pDataBuff_p == NULL || size_p == 0)
        return NDIS_STATUS_INVALID_PACKET;

    if (pVEthInstance_l == NULL)
        return NDIS_STATUS_RESOURCES;

    if (pVEthInstance_l->fMiniportPaused)
    {
        // Receive indication should not be forwarded in paused state.
        // We just drop the packet for now.
        // TODO: Implement a waitqueue to handle the received packet
        // indication after entering running state.
        return NDIS_STATUS_SUCCESS;
    }

    if (IsListEmpty(&pVEthInstance_l->rxList))
    {
        return NDIS_STATUS_RESOURCES;
    }

    pRxLink = NdisInterlockedRemoveHeadList(&pVEthInstance_l->rxList,
                                            &pVEthInstance_l->rxListLock);

    if (pRxLink == NULL)
        return NDIS_STATUS_RESOURCES;

    pVethRxInfo = CONTAINING_RECORD(pRxLink, tVEthRcvBufInfo, rxLink);

    if (pVethRxInfo->pData == NULL || pVethRxInfo == NULL)
        return NDIS_STATUS_RESOURCES;

    NdisMoveMemory(pVethRxInfo->pData, pDataBuff_p, size_p);

    // Update the size
    pNetBuffer = NET_BUFFER_LIST_FIRST_NB(pVethRxInfo->pNbl);
    NET_BUFFER_DATA_LENGTH(pNetBuffer) = size_p;

    // Indicate the received packet to upper layer protocols
    NdisMIndicateReceiveNetBufferLists(pVEthInstance_l->hMiniportAdapterHandle,
                                       pVethRxInfo->pNbl, 0, 1, 0);

    pVEthInstance_l->receiveIndication++;

    return NDIS_STATUS_SUCCESS;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief  Miniport initialization routine

This is the miniport initialization routine which gets called as a result of the
call to NdisIMInitializeDeviceInstanceEx. The context parameter
'VEthInstance' which is passed during the miniport initialization is retrieved
here for processing.

\param  adapterHandle_p     NDIS handle for this miniport adapter.
\param  driverContext_p     Handle passed to NDIS when the driver is registered.
\param  initParams_p        Miniport initialization parameters such
                            as the device context and resources.

\return The function returns an NDIS_STATUS error code.
*/
//------------------------------------------------------------------------------
NDIS_STATUS miniportInitialize(NDIS_HANDLE adapterHandle_p,
                               NDIS_HANDLE driverContext_p,
                               PNDIS_MINIPORT_INIT_PARAMETERS initParams_p)
{
    tVEthInstance*                      pVEthInstance;
    NDIS_STATUS                         status = NDIS_STATUS_FAILURE;
    NDIS_MINIPORT_ADAPTER_ATTRIBUTES    miniportAttributes;
    NDIS_CONFIGURATION_OBJECT           configObject;
    NDIS_HANDLE                         configHandle;
    PVOID                               pMacAddress;
    UINT                                macLength;
    NET_IFINDEX                         highLayerIfIndex;
    NET_IFINDEX                         lowerLayerIfIndex;
    ULONG                               packetFilter = NDIS_PACKET_TYPE_PROMISCUOUS;

    UNREFERENCED_PARAMETER(driverContext_p);

    TRACE("%s()... \n", __func__);

    if (fInitialize_l)
        return NDIS_STATUS_SUCCESS;

    pVEthInstance = (tVEthInstance*)initParams_p->IMDeviceInstanceContext;
    pVEthInstance_l = pVEthInstance;
    NdisZeroMemory(&miniportAttributes, sizeof(NDIS_MINIPORT_ADAPTER_ATTRIBUTES));

    pVEthInstance->hMiniportAdapterHandle = adapterHandle_p;

    // Register IOCTL interface here
    ndis_createDrvIntf();

    miniportAttributes.RegistrationAttributes.Header.Type = NDIS_OBJECT_TYPE_MINIPORT_ADAPTER_REGISTRATION_ATTRIBUTES;
    miniportAttributes.RegistrationAttributes.Header.Revision = NDIS_MINIPORT_ADAPTER_REGISTRATION_ATTRIBUTES_REVISION_1;
    miniportAttributes.RegistrationAttributes.Header.Size = sizeof(NDIS_MINIPORT_ADAPTER_REGISTRATION_ATTRIBUTES);

    miniportAttributes.RegistrationAttributes.MiniportAdapterContext = (NDIS_HANDLE)pVEthInstance;
    miniportAttributes.RegistrationAttributes.AttributeFlags = NDIS_MINIPORT_ATTRIBUTES_NO_HALT_ON_SUSPEND;
    miniportAttributes.RegistrationAttributes.CheckForHangTimeInSeconds = 0;
    miniportAttributes.RegistrationAttributes.InterfaceType = 0;

    NDIS_DECLARE_MINIPORT_ADAPTER_CONTEXT(tVEthInstance);
    status = NdisMSetMiniportAttributes(adapterHandle_p, &miniportAttributes);

    if (status != NDIS_STATUS_SUCCESS)
    {
        goto Exit;
    }

    configObject.Header.Type = NDIS_OBJECT_TYPE_CONFIGURATION_OBJECT;
    configObject.Header.Revision = NDIS_CONFIGURATION_OBJECT_REVISION_1;
    configObject.Header.Size = sizeof(NDIS_CONFIGURATION_OBJECT);
    configObject.NdisHandle = pVEthInstance->hMiniportAdapterHandle;
    configObject.Flags = 0;

    status = NdisOpenConfigurationEx(&configObject, &configHandle);

    if (status != NDIS_STATUS_SUCCESS)
    {
        goto Exit;
    }

    NdisReadNetworkAddress(&status, &pMacAddress, &macLength, configHandle);

    if ((status == NDIS_STATUS_SUCCESS) && (macLength == ETH_LENGTH_OF_ADDRESS) &&
        (!ETH_IS_MULTICAST(pMacAddress)))
    {
        ETH_COPY_NETWORK_ADDRESS(pVEthInstance->aCurrentAddress, pMacAddress);
    }
    else
    {
        ETH_COPY_NETWORK_ADDRESS(pVEthInstance->aCurrentAddress, pVEthInstance->aPermanentAddress);
    }

    status = NDIS_STATUS_SUCCESS;

    NdisCloseConfiguration(configHandle);

    miniportAttributes.GeneralAttributes.Header.Type = NDIS_OBJECT_TYPE_MINIPORT_ADAPTER_GENERAL_ATTRIBUTES;
    miniportAttributes.GeneralAttributes.Header.Revision = NDIS_MINIPORT_ADAPTER_GENERAL_ATTRIBUTES_REVISION_1;
    miniportAttributes.GeneralAttributes.Header.Size = sizeof(NDIS_MINIPORT_ADAPTER_GENERAL_ATTRIBUTES);
    miniportAttributes.GeneralAttributes.MediaType = NdisMedium802_3;

    miniportAttributes.GeneralAttributes.MtuSize = pVEthInstance->pProtocolInstance->bindParameters.MtuSize;
    miniportAttributes.GeneralAttributes.MaxXmitLinkSpeed = pVEthInstance->pProtocolInstance->bindParameters.MaxXmitLinkSpeed;
    miniportAttributes.GeneralAttributes.MaxRcvLinkSpeed = pVEthInstance->pProtocolInstance->bindParameters.MaxRcvLinkSpeed;
    miniportAttributes.GeneralAttributes.XmitLinkSpeed = pVEthInstance->pProtocolInstance->bindParameters.XmitLinkSpeed;
    miniportAttributes.GeneralAttributes.RcvLinkSpeed = pVEthInstance->pProtocolInstance->bindParameters.RcvLinkSpeed;

    miniportAttributes.GeneralAttributes.MediaConnectState = pVEthInstance->pProtocolInstance->lastLinkState.MediaConnectState;
    miniportAttributes.GeneralAttributes.MediaDuplexState = pVEthInstance->pProtocolInstance->lastLinkState.MediaDuplexState;
    miniportAttributes.GeneralAttributes.XmitLinkSpeed = pVEthInstance->pProtocolInstance->lastLinkState.XmitLinkSpeed;
    miniportAttributes.GeneralAttributes.RcvLinkSpeed = pVEthInstance->pProtocolInstance->lastLinkState.RcvLinkSpeed;

    pVEthInstance->lastLinkStatus = NDIS_STATUS_LINK_STATE;

    pVEthInstance->lastLinkState = pVEthInstance->pProtocolInstance->lastLinkState;

    miniportAttributes.GeneralAttributes.LookaheadSize = pVEthInstance->pProtocolInstance->bindParameters.LookaheadSize;
    miniportAttributes.GeneralAttributes.MaxMulticastListSize = pVEthInstance->pProtocolInstance->bindParameters.MaxMulticastListSize;
    miniportAttributes.GeneralAttributes.MacAddressLength = pVEthInstance->pProtocolInstance->bindParameters.MacAddressLength;

    miniportAttributes.GeneralAttributes.PhysicalMediumType = pVEthInstance->pProtocolInstance->bindParameters.PhysicalMediumType;
    miniportAttributes.GeneralAttributes.AccessType = pVEthInstance->pProtocolInstance->bindParameters.AccessType;
    miniportAttributes.GeneralAttributes.DirectionType = pVEthInstance->pProtocolInstance->bindParameters.DirectionType;
    miniportAttributes.GeneralAttributes.ConnectionType = pVEthInstance->pProtocolInstance->bindParameters.ConnectionType;
    miniportAttributes.GeneralAttributes.IfType = pVEthInstance->pProtocolInstance->bindParameters.IfType;
    miniportAttributes.GeneralAttributes.IfConnectorPresent = FALSE; // RFC 2665 TRUE if physical adapter
    miniportAttributes.GeneralAttributes.RecvScaleCapabilities = NULL;
    miniportAttributes.GeneralAttributes.MacOptions = NDIS_MAC_OPTION_NO_LOOPBACK;

    miniportAttributes.GeneralAttributes.SupportedPacketFilters = pVEthInstance->pProtocolInstance->bindParameters.SupportedPacketFilters;

    miniportAttributes.GeneralAttributes.SupportedStatistics = NDIS_STATISTICS_XMIT_OK_SUPPORTED |
                                                               NDIS_STATISTICS_RCV_OK_SUPPORTED |
                                                               NDIS_STATISTICS_XMIT_ERROR_SUPPORTED |
                                                               NDIS_STATISTICS_RCV_ERROR_SUPPORTED |
                                                               NDIS_STATISTICS_RCV_CRC_ERROR_SUPPORTED |
                                                               NDIS_STATISTICS_RCV_NO_BUFFER_SUPPORTED |
                                                               NDIS_STATISTICS_TRANSMIT_QUEUE_LENGTH_SUPPORTED |
                                                               NDIS_STATISTICS_GEN_STATISTICS_SUPPORTED;

    NdisMoveMemory(&miniportAttributes.GeneralAttributes.CurrentMacAddress,
                   &pVEthInstance->aCurrentAddress,
                   ETH_LENGTH_OF_ADDRESS);

    NdisMoveMemory(&miniportAttributes.GeneralAttributes.PermanentMacAddress,
                   &pVEthInstance->aPermanentAddress,
                   ETH_LENGTH_OF_ADDRESS);
    miniportAttributes.GeneralAttributes.PowerManagementCapabilities = NULL;
    miniportAttributes.GeneralAttributes.SupportedOidList = aVEthSupportedOids;
    miniportAttributes.GeneralAttributes.SupportedOidListLength = sizeof(aVEthSupportedOids);

    status = NdisMSetMiniportAttributes(adapterHandle_p, &miniportAttributes);

    // Allocate buffers
    status = miniportAllocateVEthRxBuff(pVEthInstance);

    if (status != NDIS_STATUS_SUCCESS)
    {
        TRACE("%s() Unable to allocate VEth resources\n", __func__);
        goto Exit;
    }

Exit:

    if (status == NDIS_STATUS_SUCCESS)
    {
        pVEthInstance->fMiniportInitPending = FALSE;

        // Save the IfIndex
        highLayerIfIndex = initParams_p->IfIndex;
        lowerLayerIfIndex = pVEthInstance->pProtocolInstance->bindParameters.BoundIfIndex;

        status = NdisIfAddIfStackEntry(highLayerIfIndex, lowerLayerIfIndex);

        if (status == NDIS_STATUS_SUCCESS)
        {
            pVEthInstance->ifIndex = highLayerIfIndex;
        }
        else
        {
            TRACE("Unable to set the NDIS stack order (0x%X)\n", status);
        }

        // We do not fail initialization if the add fails.
        status = NDIS_STATUS_SUCCESS;
    }
    else
    {
        pVEthInstance->hMiniportAdapterHandle = NULL;
    }

    if (status == NDIS_STATUS_SUCCESS)
    {
        protocol_sendOidRequest(NdisRequestSetInformation, OID_GEN_CURRENT_PACKET_FILTER, &packetFilter,
                                sizeof(packetFilter));
    }

    // We set the event even if initialization fails, the error will be identified using NULL check.
    NdisSetEvent(&pVEthInstance->miniportInitEvent);

    TRACE("%s() - OK \n", __func__);

    return status;
}

//------------------------------------------------------------------------------
/**
\brief Miniport OID request handler

This routine handles the OID requests directed towards this miniport.

\note OIDs are not processed in current implementation.

\param  adapterContext_p        Pointer to the adapter structure.
\param  ndisRequest_p           Pointer to NDIS_OID_REQUEST sent down by NDIS.

\return The function returns an NDIS_STATUS error code.
*/
//------------------------------------------------------------------------------
NDIS_STATUS miniportOidRequest(NDIS_HANDLE adapterContext_p,
                               PNDIS_OID_REQUEST ndisRequest_p)
{
    NDIS_REQUEST_TYPE    requestType;
    NDIS_STATUS          status;

    UNREFERENCED_PARAMETER(adapterContext_p);

    requestType = ndisRequest_p->RequestType;

    switch (requestType)
    {
        case NdisRequestMethod:
        case NdisRequestSetInformation:
        case NdisRequestQueryInformation:
        case NdisRequestQueryStatistics:
            // Do nothing for now
            status = NDIS_STATUS_SUCCESS;
            break;

        default:
            status = NDIS_STATUS_NOT_SUPPORTED;
            break;
    }

    return status;
}

//------------------------------------------------------------------------------
/**
\brief Halt handler

Stop all pending I/O on the VEth and then unbind it from lower miniport.

This functions updates the driver state to indicate that the miniport is
halted and cannot handle requests.

\param  adapterContext_p    Pointer to the adapter structure.
\param  haltAction_p        The reason why the adapter is being halted.

*/
//------------------------------------------------------------------------------
VOID miniportHalt(NDIS_HANDLE adapterContext_p, NDIS_HALT_ACTION haltAction_p)
{
    tVEthInstance*   pVEthInstance = (tVEthInstance*)adapterContext_p;
    NET_IFINDEX      lowerLayerIfIndex;

    UNREFERENCED_PARAMETER(haltAction_p);

    TRACE("%s()...\n", __func__);

    pVEthInstance->fMiniportHalting = TRUE;

    // The driver should wait for the completion of all requests
    // because freeing the resources before completion, may result in
    // system crash.
    while (pVEthInstance->sendRequests)
    {
        // Wait for completion of ASynchronous sends on VEth
        NdisMSleep(500);
    }

    while (pVEthInstance->receiveIndication)
    {
        // Wait for all the receive packets to be returned back to us
        NdisMSleep(500);
    }

    miniportFreeVethRxBuff(pVEthInstance);

    ndis_closeDrvIntf();

    if (pVEthInstance->ifIndex != 0)
    {
        // Remove driver from the stack
        lowerLayerIfIndex = pVEthInstance->pProtocolInstance->bindParameters.BoundIfIndex;
        NdisIfDeleteIfStackEntry(pVEthInstance->ifIndex, lowerLayerIfIndex);

        pVEthInstance->ifIndex = 0;
    }

    // Free the VETh instance
    pVEthInstance->hMiniportAdapterHandle = NULL;

    protocol_freeVEthInstance(pVEthInstance);

    TRACE("%s() - OK\n", __func__);
}

//------------------------------------------------------------------------------
/**
\brief Miniport PNP event handler

This handler is called to notify us of PnP events directed to our miniport
device object.

\param  adapterContext_p    Pointer to the adapter structure.
\param  pPnpEvent_p         Pointer to the PNP event.

*/
//------------------------------------------------------------------------------
VOID miniportPnpEventNotify(NDIS_HANDLE adapterContext_p, PNET_DEVICE_PNP_EVENT pPnpEvent_p)
{
    // Not a real device
    UNREFERENCED_PARAMETER(adapterContext_p);
    UNREFERENCED_PARAMETER(pPnpEvent_p);
}

//------------------------------------------------------------------------------
/**
\brief Miniport Shut-down Handler

This handler is called to notify us of an impending system shutdown.
Since this is not a hardware driver, there isn't anything specific
we need to do about this.

\param  adapterContext_p    Pointer to the adapter structure.
\param  shutdownAction_p    Specific reason to shut down the adapter.

*/
//------------------------------------------------------------------------------
VOID miniportShutdown(NDIS_HANDLE adapterContext_p, NDIS_SHUTDOWN_ACTION shutdownAction_p)
{
    UNREFERENCED_PARAMETER(adapterContext_p);
    UNREFERENCED_PARAMETER(shutdownAction_p);
}

//------------------------------------------------------------------------------
/**
\brief Miniport Unload Handler

This handler is used to unload the miniport during the uninstallation.

\param  driverObject_p  Pointer to the system's driver object structure
                        for this driver.

*/
//------------------------------------------------------------------------------
VOID miniportUnload(PDRIVER_OBJECT driverObject_p)
{
    TRACE("%s()...\n", __func__);
    UNREFERENCED_PARAMETER(driverObject_p);
    if (driverInstance_g.hProtocolHandle != NULL)
    {
        NdisDeregisterProtocolDriver(driverInstance_g.hProtocolHandle);
    }

    NdisMDeregisterMiniportDriver(driverInstance_g.hMiniportHandle);
    TRACE("%s() - OK\n", __func__);
}

//------------------------------------------------------------------------------
/**
\brief Miniport pause handler

This handler is used to pause the miniport. When paused, all network data flow
through this miniport will be stopped.

In current implementation, all frames received during the paused state are
dropped. In future the state change handling shall be implemented to queue
the receive frames and indicate them when the interface is restarted.

\param  adapterContext_p    Pointer to the adapter structure.
\param  pauseParams_p       Pause parameters.

\return The function returns an NDIS_STATUS error code.
*/
//------------------------------------------------------------------------------
NDIS_STATUS miniportPause(NDIS_HANDLE adapterContext_p,
                          PNDIS_MINIPORT_PAUSE_PARAMETERS pauseParams_p)
{
    tVEthInstance*   pVEthInstance = (tVEthInstance*)adapterContext_p;
    NDIS_STATUS      status = NDIS_STATUS_SUCCESS;

    UNREFERENCED_PARAMETER(pauseParams_p);

    TRACE("%s()...\n", __func__);
    NdisAcquireSpinLock(&pVEthInstance->pauseLock);
    pVEthInstance->fMiniportPaused = TRUE;
    NdisReleaseSpinLock(&pVEthInstance->pauseLock);

    TRACE("%s() - OK\n", __func__);
    return status;
}

//------------------------------------------------------------------------------
/**
\brief Miniport restart handler

This handler is used to restart the miniport.  When the miniport is
back in the restart state, it can indicate NET_BUFFER_LISTs to the
upper binding protocols.

\param  adapterContext_p    Pointer to the adapter structure.
\param  restartParams_p     Restart parameters for miniport.

\return The function returns an NDIS_STATUS error code.
*/
//------------------------------------------------------------------------------
NDIS_STATUS miniportRestart(NDIS_HANDLE adapterContext_p,
                            PNDIS_MINIPORT_RESTART_PARAMETERS restartParams_p)
{
    tVEthInstance*   pVEthInstance = (tVEthInstance*)adapterContext_p;
    NDIS_STATUS      status = NDIS_STATUS_SUCCESS;

    UNREFERENCED_PARAMETER(restartParams_p);

    TRACE("%s()... \n", __func__);
    NdisAcquireSpinLock(&pVEthInstance->pauseLock);
    pVEthInstance->fMiniportPaused = FALSE;
    NdisReleaseSpinLock(&pVEthInstance->pauseLock);
    TRACE("%s() - OK \n", __func__);
    return status;
}

//------------------------------------------------------------------------------
/**
\brief Miniport send request handler

Send NET_BUFFER_LISTs to the Kernel layer of stack for ASync scheduling.

\param  adapterContext_p    Pointer to the adapter structure.
\param  netBufferLists_p    Set of NET_BUFFER_LISTs to send.
\param  portNumber_p        A port number that identifies a miniport adapter port.
\param  sendFlags_p         Flags that define attributes for the send operation.

*/
//------------------------------------------------------------------------------
VOID miniportSendNetBufferLists(NDIS_HANDLE adapterContext_p,
                                PNET_BUFFER_LIST netBufferLists_p,
                                NDIS_PORT_NUMBER portNumber_p, ULONG sendFlags_p)
{
    tVEthInstance*      pVEthInstance = (tVEthInstance*)adapterContext_p;
    NDIS_STATUS         status = NDIS_STATUS_SUCCESS;
    PNET_BUFFER_LIST    pCurrentNbl = netBufferLists_p;
    ULONG               completeFlags = 0;
    PUCHAR              pVethTxBuff;
    PUCHAR              pVethData;
    PMDL                pMdl;
    ULONG               totalLength;
    ULONG               txLength;
    ULONG               offset = 0;                 // CurrentMdlOffset

    UNREFERENCED_PARAMETER(portNumber_p);

    // If the VEth transmit callback is not registered, mark all the packets to
    // be sent and complete the send requests with NdisMSendNetBufferListsComplete().
    if (pVEthInstance->pfnVEthSendCb == NULL)
    {
        PNET_BUFFER_LIST    tempNetBufferList;

        for (tempNetBufferList = pCurrentNbl;
             tempNetBufferList != NULL;
             tempNetBufferList = NET_BUFFER_LIST_NEXT_NBL(tempNetBufferList))
        {
            NET_BUFFER_LIST_STATUS(tempNetBufferList) = status;
        }
        if (NDIS_TEST_SEND_AT_DISPATCH_LEVEL(sendFlags_p))
        {
            NDIS_SET_SEND_COMPLETE_FLAG(completeFlags, NDIS_SEND_COMPLETE_FLAGS_DISPATCH_LEVEL);
        }

        NdisMSendNetBufferListsComplete(pVEthInstance->hMiniportAdapterHandle,
                                        pCurrentNbl,
                                        completeFlags);
        goto Exit;
    }

    // Handle the send request.

    // Allocate a new buffer to copy the transmit frame.
    pVethTxBuff = NdisAllocateMemoryWithTagPriority(driverInstance_g.hMiniportHandle,
                                                    OPLK_MAX_FRAME_SIZE,
                                                    OPLK_MEM_TAG, NormalPoolPriority);

    if (pVethTxBuff == NULL)
    {
        TRACE("%s() Failed to allocate memory for VEth Tx frame ", __func__);
        status = NDIS_STATUS_RESOURCES;
        goto Exit;
    }

    // Loop through the NetBufferList and copy the frame data into the send buffer.
    while (netBufferLists_p != NULL)
    {
        ULONG                   bytesAvailable = 0;
        PUCHAR                  pRxDataSrc;
        ULONG                   bytesToCopy = 0;

        pVethData = pVethTxBuff;

        // Current NetBufferList to process
        pCurrentNbl = netBufferLists_p;

        // Get the next NetBufferList to process
        netBufferLists_p = NET_BUFFER_LIST_NEXT_NBL(netBufferLists_p);
        NET_BUFFER_LIST_NEXT_NBL(pCurrentNbl) = NULL;

        // Increment send count to identify the pending send requests.
        pVEthInstance->sendRequests++;

        // Extract the MDL for the frame in the NetBufferList
        pMdl = NET_BUFFER_CURRENT_MDL(NET_BUFFER_LIST_FIRST_NB(pCurrentNbl));
        txLength = totalLength = NET_BUFFER_DATA_LENGTH(NET_BUFFER_LIST_FIRST_NB(pCurrentNbl));

        if (totalLength > OPLK_MAX_FRAME_SIZE)
        {
            break;
        }

        // Get the offset for the start of the frame in the NetBufferList buffer.
        offset = NET_BUFFER_CURRENT_MDL_OFFSET(NET_BUFFER_LIST_FIRST_NB(pCurrentNbl));

        // Process the MDL link list for the NetBufferList to extract complete frame to send.
        while ((pMdl != NULL) && (totalLength > 0))
        {
            pRxDataSrc = NULL;
            NdisQueryMdl(pMdl, &pRxDataSrc, &bytesAvailable, NormalPagePriority);
            if (pRxDataSrc == NULL)
            {
                break;
            }

            bytesToCopy = bytesAvailable - offset;
            bytesToCopy = min(bytesToCopy, totalLength);
            pRxDataSrc = pRxDataSrc + offset;

            // Frame is copied into the local buffer.
            NdisMoveMemory(pVethData, pRxDataSrc, bytesToCopy);
            pVethData = (PUCHAR)((ULONG_PTR)pVethData + bytesToCopy);
            totalLength -= bytesToCopy;

            offset = 0;
            NdisGetNextMdl(pMdl, &pMdl);
        }

        // Forward the frame to be sent.
        pVEthInstance->pfnVEthSendCb(pVethTxBuff, txLength);

        // Requests are not deferred, so decrease the send counter.
        pVEthInstance->sendRequests--;

        // Mark the status of the NetBufferList
        NET_BUFFER_LIST_STATUS(pCurrentNbl) = status;

        // If the caller forwarded the packets at DISPATCH_LEVEL IRQL,
        // specify the complete flags to handle send complete at same IRQL.
        if (NDIS_TEST_SEND_AT_DISPATCH_LEVEL(sendFlags_p))
        {
            NDIS_SET_SEND_COMPLETE_FLAG(completeFlags, NDIS_SEND_COMPLETE_FLAGS_DISPATCH_LEVEL);
        }

        // Complete the send request
        NdisMSendNetBufferListsComplete(pVEthInstance->hMiniportAdapterHandle,
                                        pCurrentNbl,
                                        completeFlags);
    }

    if (pVethTxBuff != NULL)
    {
        NdisFreeMemory(pVethTxBuff, 0, 0);
    }

Exit:
    return;
}

//------------------------------------------------------------------------------
/**
\brief Miniport receive complete handler

NDIS Miniport entry point called whenever protocol drivers complete processing
of a received packet that was indicated up and was queued to be freed later.

\param  adapterContext_p        Pointer to the adapter structure.
\param  netBufferLists_p        A pointer to a linked list of NET_BUFFER_LIST
                                structures that NDIS is returning to the
                                miniport driver.
\param  returnFlags_p           Return flags.

\return The function returns an NDIS_STATUS error code.
*/
//------------------------------------------------------------------------------
VOID miniportReturnNetBufferLists(NDIS_HANDLE adapterContext_p,
                                  PNET_BUFFER_LIST netBufferLists_p, ULONG returnFlags_p)
{
    tVEthInstance*      pVEthInstance = (tVEthInstance*)adapterContext_p;
    tVEthRcvBufInfo*    pVethRxInfo = NULL;

    UNREFERENCED_PARAMETER(returnFlags_p);

    while (netBufferLists_p != NULL)
    {
        pVethRxInfo = VETHINFO_FROM_NBL(netBufferLists_p);

        if (pVethRxInfo != NULL)
        {
            NdisInterlockedInsertTailList(&pVEthInstance->rxList,
                                          &pVethRxInfo->rxLink,
                                          &pVEthInstance->rxListLock);

            pVEthInstance_l->receiveIndication--;
        }

        netBufferLists_p = NET_BUFFER_LIST_NEXT_NBL(netBufferLists_p);
    }
}

//------------------------------------------------------------------------------
/**
\brief Miniport cancel send handler

The miniport entry point to handle cancellation of all packets that
match the given CancelID. If we have queued any packets that match this CancelID,
then we should dequeue them and call NdisMSendCompleteNetBufferLists for
all such packets. All the packets canceled, should be updated with a status
of NDIS_STATUS_REQUEST_ABORTED.

We should also call NdisCancelSendPackets in turn, on each lower binding
that this adapter corresponds to. This is to enable the miniports below to
cancel any matching packets.

\param  adapterContext_p    Pointer to the adapter structure.
\param  cancelId_p          ID of NetBufferLists to be canceled.

*/
//------------------------------------------------------------------------------
VOID miniportCancelSendNetBufferLists(NDIS_HANDLE adapterContext_p, PVOID cancelId_p)
{
    // We don't pend any sends so this is not required.
    UNREFERENCED_PARAMETER(adapterContext_p);
    UNREFERENCED_PARAMETER(cancelId_p);
}

//------------------------------------------------------------------------------
/**
\brief Miniport cancel OID request handler

The miniport entry point to handle cancellation of an OID request.

\param  adapterContext_p    Pointer to the adapter structure.
\param  requestId_p         RequestId to be canceled.

*/
//------------------------------------------------------------------------------
VOID miniportCancelOidRequest(NDIS_HANDLE adapterContext_p, PVOID requestId_p)
{
    UNREFERENCED_PARAMETER(adapterContext_p);
    UNREFERENCED_PARAMETER(requestId_p);
}

//------------------------------------------------------------------------------
/**
\brief Allocate VEth receive buffers

This routine allocates all the resources required for VETH interface to handle
packet receives. The receive buffers are prepared and stored in a link list
which is accessed from the receive handler.

\param  pVEthInstance_p     Pointer to the VEth instance structure.

*/
//------------------------------------------------------------------------------
NDIS_STATUS miniportAllocateVEthRxBuff(tVEthInstance* pVEthInstance_p)
{
    UINT                               index;
    NDIS_STATUS                        status = NDIS_STATUS_SUCCESS;
    NET_BUFFER_LIST_POOL_PARAMETERS    poolParameters;

    NdisInitializeListHead(&pVEthInstance_p->rxList);
    NdisAllocateSpinLock(&pVEthInstance_p->rxListLock);

    NdisZeroMemory(&poolParameters, sizeof(NET_BUFFER_LIST_POOL_PARAMETERS));

    poolParameters.Header.Type = NDIS_OBJECT_TYPE_DEFAULT;
    poolParameters.Header.Revision = NET_BUFFER_LIST_POOL_PARAMETERS_REVISION_1;
    poolParameters.Header.Size = sizeof(poolParameters);
    poolParameters.ProtocolId = NDIS_PROTOCOL_ID_DEFAULT;       // Sequenced packet exchanger
    poolParameters.ContextSize = 0;
    poolParameters.fAllocateNetBuffer = TRUE;
    poolParameters.PoolTag = OPLK_MEM_TAG;

    pVEthInstance_p->hReceiveNblPool = NdisAllocateNetBufferListPool(driverInstance_g.hMiniportHandle,
                                                                    &poolParameters);
    if (pVEthInstance_p->hReceiveNblPool == NULL)
    {
        TRACE("%s(): failed to alloc send net buffer list pool\n", __func__);
        status = NDIS_STATUS_RESOURCES;
        goto Exit;
    }

    // Allocate Tx memory
    pVEthInstance_p->pReceiveBuf = NdisAllocateMemoryWithTagPriority(driverInstance_g.hProtocolHandle,
                                                                     (OPLK_MAX_FRAME_SIZE * OPLK_MAX_VETH_BUFF),
                                                                     OPLK_MEM_TAG, NormalPoolPriority);
    if (pVEthInstance_p->pReceiveBuf == NULL)
    {
        TRACE("%s() Failed to allocate VETH Rx buffers\n", __func__);
        status = NDIS_STATUS_RESOURCES;
        goto Exit;
    }

    pVEthInstance_p->pReceiveBufInfo = NdisAllocateMemoryWithTagPriority(driverInstance_g.hProtocolHandle,
                                                                         (sizeof(tVEthRcvBufInfo) * OPLK_MAX_VETH_BUFF),
                                                                         OPLK_MEM_TAG, NormalPoolPriority);
    if (pVEthInstance_p->pReceiveBufInfo == NULL)
    {
        TRACE("%s() Failed to allocate VETH Rx buffers info\n", __func__);
        status = NDIS_STATUS_RESOURCES;
        goto Exit;
    }

    // Initialize the queue
    for (index = 0; index < OPLK_MAX_VETH_BUFF; index++)
    {
        tVEthRcvBufInfo*   pVethRxInfo = &pVEthInstance_p->pReceiveBufInfo[index];

        if (pVethRxInfo != NULL)
        {
            pVethRxInfo->fFree = TRUE;
            pVethRxInfo->maxLength = OPLK_MAX_FRAME_SIZE;
            pVethRxInfo->pData = (void*)(((UCHAR*)pVEthInstance_p->pReceiveBuf) +
                                         (OPLK_MAX_FRAME_SIZE * index));

            // Allocate MDL to define the buffers
            pVethRxInfo->pMdl = NdisAllocateMdl(pVEthInstance_p->hMiniportAdapterHandle,
                                                pVethRxInfo->pData, OPLK_MAX_FRAME_SIZE);

            if (pVethRxInfo->pMdl == NULL)
            {
                TRACE("%s() Error Allocating MDL\n", __func__);
                status = NDIS_STATUS_RESOURCES;
                goto Exit;
            }

            // Allocate empty NetBufferLists
            pVethRxInfo->pNbl = NdisAllocateNetBufferAndNetBufferList(pVEthInstance_p->hReceiveNblPool,
                                                                      0, 0, pVethRxInfo->pMdl, 0, 0);

            if (pVethRxInfo->pNbl == NULL)
            {
                TRACE("%s() Failed to allocate Tx NBL\n", __func__);
                status = NDIS_STATUS_RESOURCES;
                goto Exit;
            }

            // Mark the NetBufferList as allocated by this protocol driver
            NBL_SET_PROT_RSVD_FLAG(pVethRxInfo->pNbl, OPLK_ALLOCATED_NBL);
            pVethRxInfo->pNbl->SourceHandle = pVEthInstance_p->hMiniportAdapterHandle;
            VETHINFO_FROM_NBL(pVethRxInfo->pNbl) = pVethRxInfo;
            NdisInterlockedInsertTailList(&pVEthInstance_p->rxList, &pVethRxInfo->rxLink,
                                          &pVEthInstance_p->rxListLock);
        }
    }

Exit:
    if (status != NDIS_STATUS_SUCCESS)
    {
        miniportFreeVethRxBuff(pVEthInstance_p);
    }

    return status;
}

//------------------------------------------------------------------------------
/**
\brief Free VEth receive buffers

This routine frees all the receive resources allocated during initialization.

\param  pVEthInstance_p     Pointer to the VEth instance structure.

*/
//------------------------------------------------------------------------------
void miniportFreeVethRxBuff(tVEthInstance* pVEthInstance_p)
{
    PLIST_ENTRY         pRxLink;
    tVEthRcvBufInfo*    pRxBufInfo;

    if (pVEthInstance_p == NULL)
        return;

    if (pVEthInstance_p->rxList.Flink != NULL)
    {
        while (!IsListEmpty(&pVEthInstance_p->rxList))
        {
            pRxLink = NdisInterlockedRemoveHeadList(&pVEthInstance_p->rxList,
                                                    &pVEthInstance_p->rxListLock);
            pRxBufInfo = CONTAINING_RECORD(pRxLink, tVEthRcvBufInfo, rxLink);
            if (pRxBufInfo->pNbl != NULL)
            {
                NdisFreeNetBufferList(pRxBufInfo->pNbl);
                pRxBufInfo->pNbl = NULL;
            }

            if (pRxBufInfo->pMdl != NULL)
            {
                NdisFreeMdl(pRxBufInfo->pMdl);
                pRxBufInfo->pMdl = NULL;
            }
        }
    }

    if (pVEthInstance_p->hReceiveNblPool != NULL)
    {
        NdisFreeNetBufferListPool(pVEthInstance_p->hReceiveNblPool);
        pVEthInstance_p->hReceiveNblPool = NULL;
    }

    if (pVEthInstance_p->pReceiveBufInfo != NULL)
    {
        NdisFreeMemory(pVEthInstance_p->pReceiveBufInfo, 0, 0);
        pVEthInstance_p->pReceiveBufInfo = NULL;
    }

    if (pVEthInstance_p->pReceiveBuf != NULL)
    {
        NdisFreeMemory(pVEthInstance_p->pReceiveBuf, 0, 0);
        pVEthInstance_p->pReceiveBuf = NULL;
    }

    NdisFreeSpinLock(&pVEthInstance_p->rxListLock);
}

/// \}
