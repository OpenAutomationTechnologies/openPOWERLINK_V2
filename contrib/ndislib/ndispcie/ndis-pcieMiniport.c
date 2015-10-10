/**
********************************************************************************
\file   ndis-pcieMiniport.c

\brief  NDIS miniport section of the PCIe driver

This file implements the necessary callback routine for an NDIS miniport. The
miniport section is responsible for interacting directly with hardware and
acquire hardware resources such as memory and interrupts.

It also implements other important callbacks to register a virtual Ethernet
interface shared with the operating system.

\ingroup ndis_pcie
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
#include "ndisDriver.h"

//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// module global vars
//------------------------------------------------------------------------------
tVEthInstance    vethInstance_g;

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
NDIS_OID                VEthSupportedOids[] =
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

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static NDIS_STATUS prepareHardware(PNDIS_RESOURCE_LIST pResourceList_p);
static void        releaseHardware(void);
static BOOLEAN     interruptHandler(NDIS_HANDLE interruptContext_p, ULONG messageId_p,
                                    PBOOLEAN queueDefaultInterruptDpc_p,
                                    PULONG targetProcessors_p);
static void        interruptDpc(NDIS_HANDLE interruptContext_p, ULONG messageId_p,
                                PVOID dpcContext_p, PULONG reserved1_p,
                                PULONG reserved2_p);
static void        miniportFreeVethRxBuff(void);
static NDIS_STATUS miniportAllocateVEthRxBuff(void);

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Frame receive handler

This is the receive handler for the NDIS miniport. The receive handler forwards
the received frames to the protocol drivers enabled on this miniport.

\param  pDataBuff_p     Pointer to the received frame buffer.
\param  size_p          Size of the received frame.

\return The function returns an NDIS_STATUS error code.

\ingroup module_ndis
*/
//------------------------------------------------------------------------------
NDIS_STATUS miniport_handleReceive(UINT8* pDataBuff_p, size_t size_p)
{
    PLIST_ENTRY        pRxLink;
    tVEthRxBufInfo*    pVethRxInfo = NULL;
    PNET_BUFFER        pNetBuffer = NULL;

    if (pDataBuff_p == NULL || size_p == 0)
        return NDIS_STATUS_INVALID_PACKET;

    if (IsListEmpty(&vethInstance_g.rxList))
    {
        return NDIS_STATUS_RESOURCES;
    }

    pRxLink = NdisInterlockedRemoveHeadList(&vethInstance_g.rxList,
                                            &vethInstance_g.rxListLock);

    if (pRxLink == NULL)
        return NDIS_STATUS_RESOURCES;

    pVethRxInfo = CONTAINING_RECORD(pRxLink, tVEthRxBufInfo, rxLink);

    if (pVethRxInfo == NULL || pVethRxInfo->pData == NULL)
        return NDIS_STATUS_RESOURCES;

    NdisMoveMemory(pVethRxInfo->pData, pDataBuff_p, size_p);

    // Update the size
    pNetBuffer = NET_BUFFER_LIST_FIRST_NB(pVethRxInfo->pNbl);
    NET_BUFFER_DATA_LENGTH(pNetBuffer) = size_p;

    // Indicate the received packet to upper layer protocols
    NdisMIndicateReceiveNetBufferLists(vethInstance_g.miniportAdapterHandle,
                                       pVethRxInfo->pNbl, 0, 1, 0);

    vethInstance_g.receiveIndication++;

    return NDIS_STATUS_SUCCESS;
}

//------------------------------------------------------------------------------
/**
\brief  Set binding state for miniport

This routine handles the media state change requests from the callers.
Based on the state change, the routine prepares an NDIS status indication request
and forwards it to the operating system to indicate the change in adapter status.

\param  state_p     New state to which adapter should move.

\ingroup module_ndis
*/
//------------------------------------------------------------------------------
void miniport_setAdapterState(ULONG state_p)
{
    NDIS_LINK_STATE         linkState;
    NDIS_STATUS_INDICATION  statusIndication;

    NdisZeroMemory(&statusIndication, sizeof(NDIS_STATUS_INDICATION));
    switch (state_p)
    {
        case kNdisBindingPaused:
        case kNdisBindingPausing:
        case kNdisBindingReady:
            linkState.MediaConnectState = MediaConnectStateDisconnected;
            break;

        case kNdisBindingRunning:
            linkState.MediaConnectState = MediaConnectStateConnected;
            break;

        default:
            linkState.MediaConnectState = MediaConnectStateUnknown;
    }

    if (vethInstance_g.lastLinkState.MediaConnectState != linkState.MediaConnectState)
    {
        linkState.Header.Revision = NDIS_LINK_STATE_REVISION_1;
        linkState.Header.Type = NDIS_OBJECT_TYPE_DEFAULT;
        linkState.Header.Size = sizeof(NDIS_LINK_STATE);
        linkState.MediaDuplexState = MediaDuplexStateHalf;
        linkState.XmitLinkSpeed = linkState.RcvLinkSpeed = OPLK_LINK_SPEED;

        statusIndication.Header.Type = NDIS_OBJECT_TYPE_STATUS_INDICATION;
        statusIndication.Header.Revision = NDIS_STATUS_INDICATION_REVISION_1;
        statusIndication.Header.Size = sizeof(NDIS_STATUS_INDICATION);
        statusIndication.SourceHandle = vethInstance_g.miniportAdapterHandle;
        statusIndication.StatusCode = NDIS_STATUS_LINK_STATE;
        statusIndication.StatusBuffer = (PVOID)&linkState;
        statusIndication.StatusBufferSize = sizeof(linkState);
        statusIndication.DestinationHandle = NULL;

        // Post the status indication:
        NdisMIndicateStatusEx(vethInstance_g.miniportAdapterHandle, &statusIndication);
        NdisMoveMemory(&vethInstance_g.lastLinkState, &linkState,
                       sizeof(NDIS_LINK_STATE));
    }
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief  Miniport initialization routine

This function initializes and registers the mandatory attributes which describes
the device characteristics of the PCIe device.

It also claims necessary hardware resources from the OS and allocates resources
which the driver needs to carry out further operations.

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
    NDIS_STATUS                                status = NDIS_STATUS_SUCCESS;
    NDIS_MINIPORT_INTERRUPT_CHARACTERISTICS    intrChars;
    NDIS_MINIPORT_ADAPTER_ATTRIBUTES           miniportAttributes;
    NDIS_PNP_CAPABILITIES                      pnpCaps;

    UNREFERENCED_PARAMETER(driverContext_p);

    TRACE("%s()... \n", __FUNCTION__);

    NdisZeroMemory(&miniportAttributes, sizeof(NDIS_MINIPORT_ADAPTER_ATTRIBUTES));

    NdisZeroMemory(&vethInstance_g, sizeof(tVEthInstance));

    // Register IOCTL interface
    ndis_createDrvIntf();

    NdisAllocateSpinLock(&vethInstance_g.pauseLock);

    vethInstance_g.miniportAdapterHandle = adapterHandle_p;

    miniportAttributes.RegistrationAttributes.Header.Type = NDIS_OBJECT_TYPE_MINIPORT_ADAPTER_REGISTRATION_ATTRIBUTES;
    miniportAttributes.RegistrationAttributes.Header.Revision = NDIS_MINIPORT_ADAPTER_REGISTRATION_ATTRIBUTES_REVISION_1;
    miniportAttributes.RegistrationAttributes.Header.Size = sizeof(NDIS_MINIPORT_ADAPTER_REGISTRATION_ATTRIBUTES);

    miniportAttributes.RegistrationAttributes.MiniportAdapterContext = (NDIS_HANDLE)&vethInstance_g;
    miniportAttributes.RegistrationAttributes.AttributeFlags = NDIS_MINIPORT_ATTRIBUTES_HARDWARE_DEVICE;
    miniportAttributes.RegistrationAttributes.CheckForHangTimeInSeconds = 4;
    miniportAttributes.RegistrationAttributes.InterfaceType = NdisInterfacePci;

    status = NdisMSetMiniportAttributes(adapterHandle_p, &miniportAttributes);

    if (status != NDIS_STATUS_SUCCESS)
    {
        TRACE("%s() Miniport registration attribute configuration failed \n", __FUNCTION__);
        goto Exit;
    }

    miniportAttributes.GeneralAttributes.Header.Type = NDIS_OBJECT_TYPE_MINIPORT_ADAPTER_GENERAL_ATTRIBUTES;
    miniportAttributes.GeneralAttributes.Header.Revision = NDIS_MINIPORT_ADAPTER_GENERAL_ATTRIBUTES_REVISION_1;
    miniportAttributes.GeneralAttributes.Header.Size = sizeof(NDIS_MINIPORT_ADAPTER_GENERAL_ATTRIBUTES);

    miniportAttributes.GeneralAttributes.MediaType = NdisMedium802_3;
    miniportAttributes.GeneralAttributes.MtuSize = OPLK_MTU_SIZE;
    miniportAttributes.GeneralAttributes.MaxXmitLinkSpeed = OPLK_LINK_SPEED;
    miniportAttributes.GeneralAttributes.MaxRcvLinkSpeed = OPLK_LINK_SPEED;
    miniportAttributes.GeneralAttributes.XmitLinkSpeed = OPLK_LINK_SPEED;
    miniportAttributes.GeneralAttributes.RcvLinkSpeed = OPLK_LINK_SPEED;
    miniportAttributes.GeneralAttributes.MediaConnectState = MediaConnectStateUnknown;
    miniportAttributes.GeneralAttributes.MediaDuplexState = MediaDuplexStateHalf;
    miniportAttributes.GeneralAttributes.LookaheadSize = OPLK_LINK_SPEED;

    miniportAttributes.GeneralAttributes.PowerManagementCapabilities = NULL;
    miniportAttributes.GeneralAttributes.MacOptions = NDIS_MAC_OPTION_TRANSFERS_NOT_PEND |
                                                      NDIS_MAC_OPTION_NO_LOOPBACK;

    miniportAttributes.GeneralAttributes.SupportedPacketFilters = NDIS_PACKET_TYPE_DIRECTED |
                                                                  NDIS_PACKET_TYPE_MULTICAST |
                                                                  NDIS_PACKET_TYPE_ALL_MULTICAST |
                                                                  NDIS_PACKET_TYPE_BROADCAST;
    miniportAttributes.GeneralAttributes.MaxMulticastListSize = 32;
    miniportAttributes.GeneralAttributes.MacAddressLength = ETH_LENGTH_OF_ADDRESS;
    miniportAttributes.GeneralAttributes.PhysicalMediumType = NdisPhysicalMedium802_3;
    miniportAttributes.GeneralAttributes.RecvScaleCapabilities = NULL;
    miniportAttributes.GeneralAttributes.AccessType = NET_IF_ACCESS_BROADCAST;
    miniportAttributes.GeneralAttributes.DirectionType = NET_IF_DIRECTION_SENDRECEIVE;
    miniportAttributes.GeneralAttributes.ConnectionType = NET_IF_CONNECTION_DEDICATED;
    miniportAttributes.GeneralAttributes.IfType = IF_TYPE_ETHERNET_CSMACD;
    miniportAttributes.GeneralAttributes.IfConnectorPresent = TRUE;
    miniportAttributes.GeneralAttributes.SupportedStatistics = NDIS_STATISTICS_XMIT_OK_SUPPORTED |
                                                               NDIS_STATISTICS_RCV_OK_SUPPORTED;
    miniportAttributes.GeneralAttributes.SupportedPauseFunctions = NdisPauseFunctionsUnsupported;
    miniportAttributes.GeneralAttributes.AutoNegotiationFlags = NDIS_LINK_STATE_DUPLEX_AUTO_NEGOTIATED;
    miniportAttributes.GeneralAttributes.SupportedOidList = VEthSupportedOids;
    miniportAttributes.GeneralAttributes.SupportedOidListLength = sizeof(VEthSupportedOids);

    status = NdisMSetMiniportAttributes(adapterHandle_p, &miniportAttributes);

    if (status != NDIS_STATUS_SUCCESS)
    {
        TRACE("%s() General attribute registration failed \n", __FUNCTION__);
        goto Exit;
    }

    // Register Interrupt
    NdisZeroMemory(&intrChars, sizeof(NDIS_MINIPORT_INTERRUPT_CHARACTERISTICS));

    intrChars.Header.Type = NDIS_OBJECT_TYPE_MINIPORT_INTERRUPT;
    intrChars.Header.Revision = NDIS_MINIPORT_INTERRUPT_REVISION_1;
    intrChars.Header.Size = sizeof(NDIS_MINIPORT_INTERRUPT_CHARACTERISTICS);
    intrChars.MsiSyncWithAllMessages = TRUE;
    intrChars.MessageInterruptDpcHandler = interruptDpc;
    intrChars.MessageInterruptHandler = interruptHandler;
    intrChars.EnableMessageInterruptHandler = NULL;
    intrChars.DisableMessageInterruptHandler = NULL;

    status = NdisMRegisterInterruptEx(adapterHandle_p, &vethInstance_g, &intrChars,
                                      &vethInstance_g.interruptHandle);

    if (status != NDIS_STATUS_SUCCESS)
    {
        TRACE("%s() Failed to register interrupt \n", __FUNCTION__);
        goto Exit;
    }

    vethInstance_g.interruptType = intrChars.InterruptType;
    if (intrChars.InterruptType == NDIS_CONNECT_MESSAGE_BASED)
        vethInstance_g.intrMsgInfo = intrChars.MessageInfoTable;

    status = prepareHardware(initParams_p->AllocatedResources);

    if (status != NDIS_STATUS_SUCCESS)
    {
        TRACE("%s() Failed to parse resources \n", __FUNCTION__);
        status = NDIS_STATUS_FAILURE;
        goto Exit;
    }

    // Allocate buffers
    status = miniportAllocateVEthRxBuff();

    if (status != NDIS_STATUS_SUCCESS)
    {
        TRACE("%s() Unable to allocate VEth resources\n", __FUNCTION__);
        goto Exit;
    }

    vethInstance_g.state = kNdisBindingReady;

Exit:
    if (status != NDIS_STATUS_SUCCESS)
    {
        releaseHardware();
    }

    TRACE("Return with status 0x%X\n", status);
    return status;
}

//------------------------------------------------------------------------------
/**
\brief Miniport OID request handler

This function handles an OID request to query or set information in the driver.

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

Stop all pending I/O on the VEth, release the hardware resources acquired
before and close the driver interface for the application.

Also update the driver state to indicate that the miniport is halted and can't
handle requests.

\param  adapterContext_p    Pointer to the adapter structure.
\param  haltAction_p        The reason why the adapter is being halted.

*/
//------------------------------------------------------------------------------
VOID miniportHalt(NDIS_HANDLE adapterContext_p, NDIS_HALT_ACTION haltAction_p)
{
    TRACE("%s()...\n", __FUNCTION__);

    UNREFERENCED_PARAMETER(adapterContext_p);
    UNREFERENCED_PARAMETER(haltAction_p);

    vethInstance_g.miniportHalting = TRUE;

    miniportFreeVethRxBuff();

    releaseHardware();

    ndis_closeDrvIntf();

    vethInstance_g.miniportAdapterHandle = NULL;

    TRACE("%s() - OK\n", __FUNCTION__);
}

//------------------------------------------------------------------------------
/**
\brief Miniport PNP event handler

This handler is called to notify the PnP events directed to this miniport
device.

\param  adapterContext_p    Pointer to the adapter structure.
\param  pnpEvent_p          Pointer to the PNP event.

*/
//------------------------------------------------------------------------------
VOID miniportPnpEventNotify(NDIS_HANDLE adapterContext_p, PNET_DEVICE_PNP_EVENT pnpEvent_p)
{
    UNREFERENCED_PARAMETER(adapterContext_p);
    UNREFERENCED_PARAMETER(pnpEvent_p);
}

//------------------------------------------------------------------------------
/**
\brief Miniport Shut-down Handler

This handler is called to notify us of an impending system shutdown.

\note All necessary shutdown steps handled in halt.

\param  adapterContext_p    Pointer to the adapter structure.
\param  shutdownAction_P    Specific reason to shut down the adapter.

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
    UNREFERENCED_PARAMETER(driverObject_p);
    TRACE("%s()...\n", __FUNCTION__);

    NdisMDeregisterMiniportDriver(driverInstance_g.pMiniportHandle);

    TRACE("%s() - OK\n", __FUNCTION__);
}

//------------------------------------------------------------------------------
/**
\brief Miniport pause handler

This handler is used to pause the miniport. During which, no NET_BUFFER_LIST
will be indicated to the upper bindings as well as no status indications.

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

    TRACE("%s()...\n", __FUNCTION__);

    NdisAcquireSpinLock(&pVEthInstance->pauseLock);
    pVEthInstance->miniportPaused = TRUE;
    NdisReleaseSpinLock(&pVEthInstance->pauseLock);

    TRACE("%s() - OK\n", __FUNCTION__);
    return status;
}

//------------------------------------------------------------------------------
/**
\brief Miniport restart handler

This handler is used to restart the miniport.  When the miniport is
back in the running state, it can indicate NET_BUFFER_LISTs to the
upper binding.

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

    TRACE("%s()... \n", __FUNCTION__);

    NdisAcquireSpinLock(&pVEthInstance->pauseLock);
    pVEthInstance->miniportPaused = FALSE;
    NdisReleaseSpinLock(&pVEthInstance->pauseLock);

    TRACE("%s() - OK \n", __FUNCTION__);
    return status;
}

//------------------------------------------------------------------------------
/**
\brief Miniport send request handler

This routine handles the send requests from the upper protocol drivers which bind
to this miniport.

\note Currently, the passed NET_BUFFER_LISTs are silently dropped without
      processing and successful completion status is indicated for each
      NET_BUFFER_LISTs.

\param  adapterContext_p    Pointer to the adapter structure.
\param  netBufferLists_p    Set of NET_BUFFER_LISTs to send.
\param  portNumber_p        A port number that identifies a miniport adapter port.
\param  sendFlags_p         Flags that define attributes for the send operation.

*/
//------------------------------------------------------------------------------
VOID miniportSendNetBufferLists(NDIS_HANDLE adapterContext_p, PNET_BUFFER_LIST netBufferLists_p,
                                NDIS_PORT_NUMBER portNumber_p, ULONG sendFlags_p)
{
    tVEthInstance*      pVEthInstance = (tVEthInstance*)adapterContext_p;
    NDIS_STATUS         status = NDIS_STATUS_SUCCESS;
    PNET_BUFFER_LIST    currentNbl = netBufferLists_p;
    PNET_BUFFER_LIST    returnNbl = NULL;
    PNET_BUFFER_LIST    lastReturnNbl = NULL;
    ULONG               completeFlags = 0;
    PUCHAR              pVethTxBuff;
    PUCHAR              pVethData;
    PMDL                pMdl;
    ULONG               totalLength, txLength;
    ULONG               offset = 0;                 // CurrentMdlOffset

    UNREFERENCED_PARAMETER(portNumber_p);

    if (pVEthInstance->pfnVEthSendCb == NULL)
    {
        PNET_BUFFER_LIST    tempNetBufferList;

        for (tempNetBufferList = currentNbl;
             tempNetBufferList != NULL;
             tempNetBufferList = NET_BUFFER_LIST_NEXT_NBL(tempNetBufferList))
        {
            NET_BUFFER_LIST_STATUS(tempNetBufferList) = status;
        }

        if (NDIS_TEST_SEND_AT_DISPATCH_LEVEL(sendFlags_p))
        {
            NDIS_SET_SEND_COMPLETE_FLAG(completeFlags, NDIS_SEND_COMPLETE_FLAGS_DISPATCH_LEVEL);
        }

        NdisMSendNetBufferListsComplete(pVEthInstance->miniportAdapterHandle,
                                        currentNbl,
                                        completeFlags);
        goto Exit;
    }

    pVethTxBuff = NdisAllocateMemoryWithTagPriority(driverInstance_g.pMiniportHandle,
                                                    OPLK_MAX_FRAME_SIZE,
                                                    OPLK_MEM_TAG, NormalPoolPriority);

    if (pVethTxBuff == NULL)
    {
        DbgPrint("%s() Failed to allocate memory for VEth Tx frame ", __FUNCTION__);
        status = NDIS_STATUS_RESOURCES;
        goto Exit;
    }

    while (netBufferLists_p != NULL)
    {
        ULONG                   bytesAvailable = 0;
        PUCHAR                  pRxDataSrc;
        ULONG                   bytesToCopy = 0;

        pVethData = pVethTxBuff;
        currentNbl = netBufferLists_p;
        netBufferLists_p = NET_BUFFER_LIST_NEXT_NBL(netBufferLists_p);
        NET_BUFFER_LIST_NEXT_NBL(currentNbl) = NULL;

        pVEthInstance->sendRequests++;

        pMdl = NET_BUFFER_CURRENT_MDL(NET_BUFFER_LIST_FIRST_NB(currentNbl));
        txLength = totalLength = NET_BUFFER_DATA_LENGTH(NET_BUFFER_LIST_FIRST_NB(currentNbl));

        if (totalLength > OPLK_MAX_FRAME_SIZE)
        {
            break;
        }

        offset = NET_BUFFER_CURRENT_MDL_OFFSET(NET_BUFFER_LIST_FIRST_NB(currentNbl));

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

            NdisMoveMemory(pVethData, pRxDataSrc, bytesToCopy);
            pVethData = (PUCHAR)((ULONG_PTR)pVethData + bytesToCopy);
            totalLength -= bytesToCopy;

            offset = 0;
            NdisGetNextMdl(pMdl, &pMdl);
        }

        pVEthInstance->pfnVEthSendCb(pVethTxBuff, txLength);

        // We don't pend the requests so decrease the send counter.
        pVEthInstance->sendRequests--;

        NET_BUFFER_LIST_STATUS(currentNbl) = status;

        if (NDIS_TEST_SEND_AT_DISPATCH_LEVEL(sendFlags_p))
        {
            NDIS_SET_SEND_COMPLETE_FLAG(completeFlags, NDIS_SEND_COMPLETE_FLAGS_DISPATCH_LEVEL);
        }

        NdisMSendNetBufferListsComplete(pVEthInstance->miniportAdapterHandle,
                                        currentNbl,
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

NDIS Miniport entry point called whenever protocols are done with a received
packet that was indicated up and was queued to be freed later.

\param  adapterContext_p        Pointer to the adapter structure.
\param  netBufferLists_p        A pointer to a linked list of NET_BUFFER_LIST
                                structures that NDIS is returning to the
                                miniport driver.
\param  returnFlags_p           Return flags.

*/
//------------------------------------------------------------------------------
VOID miniportReturnNetBufferLists(NDIS_HANDLE adapterContext_p,
                                  PNET_BUFFER_LIST netBufferLists_p, ULONG returnFlags_p)
{
    tVEthRxBufInfo*    pVethRxInfo = NULL;

    UNREFERENCED_PARAMETER(adapterContext_p);
    UNREFERENCED_PARAMETER(returnFlags_p);

    while (netBufferLists_p != NULL)
    {
        pVethRxInfo = VETHINFO_FROM_NBL(netBufferLists_p);

        if (pVethRxInfo != NULL)
        {
            NdisInterlockedInsertTailList(&vethInstance_g.rxList,
                                          &pVethRxInfo->rxLink,
                                          &vethInstance_g.rxListLock);

            vethInstance_g.receiveIndication--;
        }

        netBufferLists_p = NET_BUFFER_LIST_NEXT_NBL(netBufferLists_p);
    }
}

//------------------------------------------------------------------------------
/**
\brief Miniport cancel NET_BUFFER_LISTs handler

The miniport entry point to handle cancellation of all send packets that
match the given CancelId. If any packets in the queue match the specified ID,
then it should be dequeued and NdisMSendCompleteNetBufferLists should be called
for all such packets with a status of NDIS_STATUS_REQUEST_ABORTED.

\param  adapterContext_p    Pointer to the adapter structure.
\param  cancelId_p          ID of NetBufferLists to be canceled.

*/
//------------------------------------------------------------------------------
VOID miniportCancelSendNetBufferLists(NDIS_HANDLE adapterContext_p, PVOID cancelId_p)
{
    UNREFERENCED_PARAMETER(adapterContext_p);
    UNREFERENCED_PARAMETER(cancelId_p);
    // TODO: Handle cancel requests
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
    // TODO: Cancel OID requests
}

//------------------------------------------------------------------------------
/**
\brief Miniport check for hang handler

This routine is called to check if the hardware device responds correctly.

\param  adapterContext_p    Pointer to the adapter structure.

\return The function returns hardware status.
\retval TRUE if the device is not responding.
\retval FALSE if the device is working properly.
*/
//------------------------------------------------------------------------------
BOOLEAN miniportCheckForHang(NDIS_HANDLE adapterContext_p)
{
    UNREFERENCED_PARAMETER(adapterContext_p);
    return FALSE;
}

//------------------------------------------------------------------------------
/**
\brief  Miniport reset

This routine is called by the operating system to reset all pending send and
receive requests on this adapter.

\param  adapterContext_p    Pointer to the adapter structure.
\param  addressingReset_p   Pointer to flag to acknowledge reset.

\return The function returns an NDIS_STATUS error code.
*/
//------------------------------------------------------------------------------
NDIS_STATUS miniportReset(NDIS_HANDLE adapterContext_p, PBOOLEAN addressingReset_p)
{
    NDIS_STATUS    ndisStatus = NDIS_STATUS_SUCCESS;
    TRACE("%s() ....");
    UNREFERENCED_PARAMETER(adapterContext_p);
    UNREFERENCED_PARAMETER(addressingReset_p);
    TRACE("OK\n");
    return ndisStatus;
}

//------------------------------------------------------------------------------
/**
\brief Prepare hardware resources

This routine parses the hardware resources such as memory and interrupt
assigned by the OS during device enumeration and maps it into system space.

\param  resourceList_p    Pointer to resources allocated for device.

\return The function returns an NDIS_STATUS error code.
*/
//------------------------------------------------------------------------------
static NDIS_STATUS prepareHardware(PNDIS_RESOURCE_LIST pResourceList_p)
{
    NDIS_STATUS                         status = NDIS_STATUS_SUCCESS;
    PCM_PARTIAL_RESOURCE_DESCRIPTOR     pResDescriptor;
    ULONG                               resCount;
    ULONG                               barId = 0;

    PAGED_CODE();

    // Scan through the allocated resources and store them in device context
    for (resCount = 0; resCount < pResourceList_p->Count; resCount++)
    {
        pResDescriptor = &pResourceList_p->PartialDescriptors[resCount];

        if (pResDescriptor == NULL)
        {
            return STATUS_DEVICE_CONFIGURATION_ERROR;
        }

        switch (pResDescriptor->Type)
        {
            case CmResourceTypePort:
                // Device doesn't have any I/O Port Resources so ignore this
                break;

            case CmResourceTypeMemory:
            {
                tBarInfo*   pBarInfo;

                if (pResDescriptor->u.Memory.Length <= 0 ||
                    barId >= OPLK_MAX_BAR_COUNT)
                    break;

                pBarInfo = &vethInstance_g.barInfo[barId];

                pBarInfo->phyAddr = pResDescriptor->u.Memory.Start;
                status = NdisMMapIoSpace(&pBarInfo->pVirtualAddr,
                                         vethInstance_g.miniportAdapterHandle,
                                         pResDescriptor->u.Memory.Start,
                                         pResDescriptor->u.Memory.Length);

                if (status != NDIS_STATUS_SUCCESS)
                {
                    TRACE("%s() BAR %d not mapped\n", __FUNCTION__, barId);
                    goto Exit;
                }

                pBarInfo->length = pResDescriptor->u.Memory.Length;

                TRACE("BAR %d PhyAddr:%p VirtAddr: %p Size %d\n", barId,
                      pBarInfo->phyAddr, pBarInfo->pVirtualAddr, pBarInfo->length);

                barId++;
            }
            break;

            case CmResourceTypeInterrupt:
                if (pResDescriptor->Flags & CM_RESOURCE_INTERRUPT_MESSAGE)
                {
                    vethInstance_g.msiVector = pResDescriptor->u.MessageInterrupt.Translated.Vector;
                }
                break;
        }
    }

Exit:
    return status;
}

//------------------------------------------------------------------------------
/**
\brief Release hardware resources

This routine frees resources allocated during initialization.

*/
//------------------------------------------------------------------------------
static void releaseHardware(void)
{
    INT         barId;
    tBarInfo*   pBarInfo;

    vethInstance_g.state = kNdisBindingPausing;

    for (barId = 0; barId < OPLK_MAX_BAR_COUNT; barId++)
    {
        pBarInfo = &vethInstance_g.barInfo[barId];

        if (pBarInfo->pVirtualAddr != NULL)
        {
            NdisMUnmapIoSpace(vethInstance_g.miniportAdapterHandle, pBarInfo->pVirtualAddr,
                              pBarInfo->length);
            pBarInfo->pVirtualAddr = NULL;
        }
    }

    if (vethInstance_g.interruptHandle)
    {
        NdisMDeregisterInterruptEx(vethInstance_g.interruptHandle);
        vethInstance_g.interruptHandle = NULL;
    }

    NdisFreeSpinLock(&vethInstance_g.pauseLock);
}

//------------------------------------------------------------------------------
/**
\brief Interrupt service routine

This is the master interrupt service routine for the PCIe. Nothing is handled
in the interrupt context instead we handle it in a deferred procedure call (DPC)
for the interrupt.

\param  interruptContext_p          Pointer to context memory of the driver.
\param  messageId_p                 Message ID of the interrupt.
\param  queueDefaultInterruptDpc_p  Flag to invoke DPC.
\param  targetProcessors_p          Target processor interrupted.

\return Returns status of interrupt being handled in this ISR.
\retval TRUE if the interrupt is handled.
\retval FALSE if interrupt was not invoked by handling device.

*/
//------------------------------------------------------------------------------
static BOOLEAN interruptHandler(NDIS_HANDLE interruptContext_p, ULONG messageId_p,
                         PBOOLEAN queueDefaultInterruptDpc_p, PULONG targetProcessors_p)
{
    UNREFERENCED_PARAMETER(interruptContext_p);
    UNREFERENCED_PARAMETER(messageId_p);
    UNREFERENCED_PARAMETER(targetProcessors_p);

    *queueDefaultInterruptDpc_p = TRUE;

    return TRUE;
}

//------------------------------------------------------------------------------
/**
\brief Interrupt DPC routine

Deferred procedure call for the ISR. This routine runs at a lower IRQ level than
ISR and can perform blocking operations which are not recommended in ISRs.

\param  interruptContext_p    Pointer to context memory of the driver.
\param  messageId_p           Message ID of the interrupt.
\param  dpcContext_p          Pointer to DPC context memory.
\param  reserved1_p           Pointer to reserve context 1.
\param  reserved2_p           Pointer to reserve context 2.

*/
//------------------------------------------------------------------------------
static void interruptDpc(NDIS_HANDLE interruptContext_p, ULONG messageId_p,
                  PVOID dpcContext_p, PULONG reserved1_p, PULONG reserved2_p)
{
    UNREFERENCED_PARAMETER(interruptContext_p);
    UNREFERENCED_PARAMETER(messageId_p);
    UNREFERENCED_PARAMETER(dpcContext_p);
    UNREFERENCED_PARAMETER(reserved1_p);
    UNREFERENCED_PARAMETER(reserved2_p);

    if (vethInstance_g.pfnIntrCb != NULL)
    {
        vethInstance_g.pfnIntrCb();
    }
}

//------------------------------------------------------------------------------
/**
\brief Allocate VEth receive buffers

This routine allocates all the receive resources required for VETH interface.
The receive buffers are prepared and stored in a linked list.

\return The function returns an NDIS_STATUS error code.

*/
//------------------------------------------------------------------------------
static NDIS_STATUS miniportAllocateVEthRxBuff(void)
{
    UINT                               index;
    NDIS_STATUS                        status = NDIS_STATUS_SUCCESS;
    NET_BUFFER_LIST_POOL_PARAMETERS    poolParameters;

    NdisInitializeListHead(&vethInstance_g.rxList);
    NdisAllocateSpinLock(&vethInstance_g.rxListLock);

    NdisZeroMemory(&poolParameters, sizeof(NET_BUFFER_LIST_POOL_PARAMETERS));

    poolParameters.Header.Type = NDIS_OBJECT_TYPE_DEFAULT;
    poolParameters.Header.Revision = NET_BUFFER_LIST_POOL_PARAMETERS_REVISION_1;
    poolParameters.Header.Size = sizeof(poolParameters);
    poolParameters.ProtocolId = NDIS_PROTOCOL_ID_DEFAULT;       // Sequenced Packet exchanger
    poolParameters.ContextSize = 0;
    poolParameters.fAllocateNetBuffer = TRUE;
    poolParameters.PoolTag = OPLK_MEM_TAG;

    vethInstance_g.receiveNblPool = NdisAllocateNetBufferListPool(driverInstance_g.pMiniportHandle,
                                                                  &poolParameters);
    if (vethInstance_g.receiveNblPool == NULL)
    {
        DbgPrint("%s(): failed to alloc send net buffer list pool\n", __FUNCTION__);
        status = NDIS_STATUS_RESOURCES;
        goto Exit;
    }

    // Allocate Tx memory
    vethInstance_g.pReceiveBuf = NdisAllocateMemoryWithTagPriority(driverInstance_g.pProtocolHandle,
                                                                   (OPLK_MAX_FRAME_SIZE * OPLK_MAX_VETH_BUFF),
                                                                   OPLK_MEM_TAG, NormalPoolPriority);
    if (vethInstance_g.pReceiveBuf == NULL)
    {
        DbgPrint("%s() Failed to allocate VETH Rx buffers\n", __FUNCTION__);
        status = NDIS_STATUS_RESOURCES;
        goto Exit;
    }

    vethInstance_g.pReceiveBufInfo = NdisAllocateMemoryWithTagPriority(driverInstance_g.pProtocolHandle,
                                                                       (sizeof(tVEthRxBufInfo) * OPLK_MAX_VETH_BUFF),
                                                                       OPLK_MEM_TAG, NormalPoolPriority);
    if (vethInstance_g.pReceiveBufInfo == NULL)
    {
        DbgPrint("%s() Failed to allocate VETH Rx buffers info\n", __FUNCTION__);
        status = NDIS_STATUS_RESOURCES;
        goto Exit;
    }

    // Initialize the queue
    for (index = 0; index < OPLK_MAX_VETH_BUFF; index++)
    {
        tVEthRxBufInfo*   pVethRxInfo = &vethInstance_g.pReceiveBufInfo[index];

        if (pVethRxInfo != NULL)
        {
            pVethRxInfo->free = TRUE;
            pVethRxInfo->maxLength = OPLK_MAX_FRAME_SIZE;
            pVethRxInfo->pData = (void*)(((UCHAR*)vethInstance_g.pReceiveBuf) +
                                         (OPLK_MAX_FRAME_SIZE * index));

            // Allocate MDL to define the buffers
            pVethRxInfo->pMdl = NdisAllocateMdl(vethInstance_g.miniportAdapterHandle,
                                                pVethRxInfo->pData, OPLK_MAX_FRAME_SIZE);

            if (pVethRxInfo->pMdl == NULL)
            {
                DbgPrint("%s() Error Allocating MDL\n", __FUNCTION__);
                status = NDIS_STATUS_RESOURCES;
                goto Exit;
            }

            // Allocate empty NetBufferLists
            pVethRxInfo->pNbl = NdisAllocateNetBufferAndNetBufferList(vethInstance_g.receiveNblPool,
                                                                      0, 0, pVethRxInfo->pMdl, 0, 0);

            if (pVethRxInfo->pNbl == NULL)
            {
                DbgPrint("%s() Failed to allocate Tx NBL\n", __FUNCTION__);
                status = NDIS_STATUS_RESOURCES;
                goto Exit;
            }

            // Mark the NetBufferList as allocated by this protocol driver
            NBL_SET_PROT_RSVD_FLAG(pVethRxInfo->pNbl, OPLK_ALLOCATED_NBL);
            pVethRxInfo->pNbl->SourceHandle = vethInstance_g.miniportAdapterHandle;
            VETHINFO_FROM_NBL(pVethRxInfo->pNbl) = pVethRxInfo;
            NdisInterlockedInsertTailList(&vethInstance_g.rxList, &pVethRxInfo->rxLink,
                                          &vethInstance_g.rxListLock);
        }
    }

Exit:
    if (status != NDIS_STATUS_SUCCESS)
    {
        miniportFreeVethRxBuff();
    }

    return status;
}

//------------------------------------------------------------------------------
/**
\brief Free VEth receive buffers

This routine frees all the receive resources allocated during initialization.

*/
//------------------------------------------------------------------------------
static void miniportFreeVethRxBuff(void)
{
    PLIST_ENTRY         pRxLink;
    tVEthRxBufInfo*     pRxBufInfo;

    if (vethInstance_g.rxList.Flink != NULL)
    {
        while (!IsListEmpty(&vethInstance_g.rxList))
        {
            pRxLink = NdisInterlockedRemoveHeadList(&vethInstance_g.rxList,
                                                    &vethInstance_g.rxListLock);
            pRxBufInfo = CONTAINING_RECORD(pRxLink, tVEthRxBufInfo, rxLink);
            if (pRxBufInfo->pNbl != NULL)
            {
                NdisFreeNetBufferList(pRxBufInfo->pNbl);
            }

            if (pRxBufInfo->pMdl != NULL)
            {
                NdisFreeMdl(pRxBufInfo->pMdl);
            }
        }
    }

    if (vethInstance_g.receiveNblPool != NULL)
    {
        NdisFreeNetBufferListPool(vethInstance_g.receiveNblPool);
        vethInstance_g.receiveNblPool = NULL;
    }

    if (vethInstance_g.pReceiveBufInfo != NULL)
    {
        NdisFreeMemory(vethInstance_g.pReceiveBufInfo, 0, 0);
    }

    if (vethInstance_g.pReceiveBuf != NULL)
    {
        NdisFreeMemory(vethInstance_g.pReceiveBuf, 0, 0);
    }

    NdisFreeSpinLock(&vethInstance_g.rxListLock);
}

/// \}
