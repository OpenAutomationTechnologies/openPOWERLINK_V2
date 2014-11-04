/**
********************************************************************************
\file   ndis-im.c

\brief  NDIS library interface routines for NDIS Intermediate driver

This file implements the initialization routines for NDIS intermediate drivers
and provides helper routines which can be used to access, modify, send and
receive frames.

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

#include "ndis-imInternal.h"

//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#define OPLK_MAJOR_NDIS_VERSION         6
#define OPLK_MINOR_NDIS_VERSION         0

#define OPLK_MAJOR_DRIVER_VERSION       3
#define OPLK_MINOR_DRIVER_VERSION       0

#define OPLK_PROT_MAJOR_NDIS_VERSION    6
#define OPLK_PROT_MINOR_NDIS_VERSION    0

//------------------------------------------------------------------------------
// module global vars
//------------------------------------------------------------------------------
tNdisDriverInstance    driverInstance_g;

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

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//
//------------------------------------------------------------------------------
/**
\brief  Initialization routine for NDIS driver

This routine registers the NDIS protocol/miniport characteristics and entry
routines to the OS using NdisXRegisterXXXDriver.

\param  pDriverObject_p     Pointer to the driver object structure
                            for this driver.
\param  pRegistryPath_p     System's registry path for this driver.

\return The function returns an NDIS_STATUS error code.

\ingroup module_ndis
*/
//------------------------------------------------------------------------------
NDIS_STATUS ndis_initDriver(PDRIVER_OBJECT pDriverObject_p,
                            PUNICODE_STRING pRegistryPath_p)
{
    NDIS_STATUS                            ndisStatus = NDIS_STATUS_SUCCESS;
    NDIS_PROTOCOL_DRIVER_CHARACTERISTICS   protocolChars;
    NDIS_MINIPORT_DRIVER_CHARACTERISTICS   miniportChars;
    NDIS_HANDLE                            miniportDriverContext = NULL;
    NDIS_HANDLE                            protocolDriverContext = NULL;
    NDIS_STRING                            ndisDriverName;

    NdisZeroMemory(&driverInstance_g, sizeof(tNdisDriverInstance));

    NdisZeroMemory(&miniportChars, sizeof(NDIS_MINIPORT_DRIVER_CHARACTERISTICS));
    miniportChars.Header.Type = NDIS_OBJECT_TYPE_DEFAULT;
    miniportChars.Header.Size = sizeof(NDIS_MINIPORT_DRIVER_CHARACTERISTICS);
    miniportChars.Header.Revision = NDIS_MINIPORT_DRIVER_CHARACTERISTICS_REVISION_1;
    miniportChars.MajorNdisVersion = OPLK_MAJOR_NDIS_VERSION;
    miniportChars.MinorNdisVersion = OPLK_MINOR_NDIS_VERSION;
    miniportChars.MajorDriverVersion = OPLK_MAJOR_DRIVER_VERSION;
    miniportChars.MinorDriverVersion = OPLK_MINOR_DRIVER_VERSION;

    miniportChars.SetOptionsHandler = miniportSetOptions;
    miniportChars.InitializeHandlerEx = miniportInitialize;
    miniportChars.UnloadHandler = miniportUnload;
    miniportChars.HaltHandlerEx = miniportHalt;

    miniportChars.OidRequestHandler = miniportOidRequest;

    miniportChars.CancelSendHandler = miniportCancelSendNetBufferLists;
    miniportChars.DevicePnPEventNotifyHandler = miniportPnpEventNotify;
    miniportChars.ShutdownHandlerEx = miniportShutdown;
    miniportChars.CancelOidRequestHandler = miniportCancelOidRequest;

    // Intermediate drivers do not require handler to check device status.
    miniportChars.CheckForHangHandlerEx = NULL;

    miniportChars.ReturnNetBufferListsHandler = miniportReturnNetBufferLists;
    miniportChars.SendNetBufferListsHandler = miniportSendNetBufferLists;

    miniportChars.PauseHandler = miniportPause;
    miniportChars.RestartHandler = miniportRestart;

    miniportChars.Flags = NDIS_INTERMEDIATE_DRIVER;

    ndisStatus = NdisMRegisterMiniportDriver(pDriverObject_p, pRegistryPath_p,
                                             miniportDriverContext,
                                             &miniportChars,
                                             &driverInstance_g.hMiniportHandle);
    if (ndisStatus != NDIS_STATUS_SUCCESS)
    {
        TRACE("%s() Miniport driver registration failed 0x%X\n", __FUNCTION__, ndisStatus);
        return ndisStatus;
    }

    NdisZeroMemory(&protocolChars, sizeof(NDIS_PROTOCOL_DRIVER_CHARACTERISTICS));

    protocolChars.Header.Type = NDIS_OBJECT_TYPE_DEFAULT;
    protocolChars.Header.Size = sizeof(NDIS_PROTOCOL_DRIVER_CHARACTERISTICS);
    protocolChars.Header.Revision = NDIS_PROTOCOL_DRIVER_CHARACTERISTICS_REVISION_1;
    protocolChars.MajorNdisVersion = OPLK_PROT_MAJOR_NDIS_VERSION;
    protocolChars.MinorNdisVersion = OPLK_PROT_MINOR_NDIS_VERSION;

    protocolChars.MajorDriverVersion = OPLK_MAJOR_DRIVER_VERSION;
    protocolChars.MinorDriverVersion = OPLK_MINOR_DRIVER_VERSION;

    protocolChars.SetOptionsHandler = protocolSetOptions;

    NdisInitUnicodeString(&ndisDriverName, L"PLKP");    // Protocol name
    protocolChars.Name = ndisDriverName;
    protocolChars.OpenAdapterCompleteHandlerEx = protocolOpenAdapterComplete;
    protocolChars.CloseAdapterCompleteHandlerEx = protocolCloseAdapterComplete;

    protocolChars.ReceiveNetBufferListsHandler = protocolReceiveNbl;
    protocolChars.SendNetBufferListsCompleteHandler = protocolSendNblComplete;
    protocolChars.OidRequestCompleteHandler = protocolRequestComplete;
    protocolChars.StatusHandlerEx = protocolStatus;
    protocolChars.BindAdapterHandlerEx = protocolBindAdapter;
    protocolChars.UnbindAdapterHandlerEx = protocolUnbindAdapter;
    protocolChars.NetPnPEventHandler = protocolPnpHandler;

    ndisStatus = NdisRegisterProtocolDriver(protocolDriverContext, &protocolChars,
                                            &driverInstance_g.hProtocolHandle);

    if (ndisStatus != NDIS_STATUS_SUCCESS)
    {
        TRACE("%s() Protocol driver registration failed 0x%X\n", __FUNCTION__, ndisStatus);
        NdisMDeregisterMiniportDriver(driverInstance_g.hMiniportHandle);
        return ndisStatus;
    }

    // Create association between protocol and miniport driver
    NdisIMAssociateMiniport(driverInstance_g.hMiniportHandle, driverInstance_g.hProtocolHandle);

    return ndisStatus;
}

//------------------------------------------------------------------------------
/**
\brief  Get miniport driver handle

Get miniport driver handle returned by OS during miniport registration for this
driver.

\return The function returns miniport adapter handle.

\ingroup module_ndis
*/
//------------------------------------------------------------------------------
NDIS_HANDLE ndis_getAdapterHandle(void)
{
    return driverInstance_g.hMiniportHandle;
}

//------------------------------------------------------------------------------
/**
\brief  Get Mac address of the device

Get MAC address of the Ethernet controller.

\param pMac_p       Pointer to MAC address.

\ingroup module_ndis
*/
//------------------------------------------------------------------------------
void ndis_getMacAddress(UCHAR*  pMac_p)
{
    UCHAR*   pCurrentMac;

    if (pMac_p == NULL)
        return;

    pCurrentMac = protocol_getCurrentMac();

    if (pCurrentMac != NULL)
    {
        NdisMoveMemory(pMac_p, pCurrentMac, ETH_LENGTH_OF_ADDRESS);
    }
}

//------------------------------------------------------------------------------
/**
\brief  Register driver interface routines

Register routines for IOCTL interface registration and deregistration. The
driver calls the registration routine from miniport initialization
callback and deregistration from miniport halt callback.

\param pDrvIntfRegCb_p       Driver interface registration callback.
\param pDrvIntfDeregCb_p     Driver interface deregistration callback.

\ingroup module_ndis
*/
//------------------------------------------------------------------------------
void ndis_registerDrvIntf(tDrvIntfRegister pfnDrvIntfRegCb_p,
                          tDrvIntfDeregister pfnDrvIntfDeregCb_p)
{
    driverInstance_g.pfnDrvIntfRegCb = pfnDrvIntfRegCb_p;
    driverInstance_g.pfnDrvIntfDeregisterCb = pfnDrvIntfDeregCb_p;
}

//------------------------------------------------------------------------------
/**
\brief  Check driver binding state

Check the current state of lower edge binding of protocol.

\return Returns status of the protocol binding.
\retval TRUE if ready.
\retval FALSE if not ready.

\ingroup module_ndis
*/
//------------------------------------------------------------------------------
BOOLEAN ndis_checkBindingState(void)
{
    return protocol_checkBindingState();
}

//------------------------------------------------------------------------------
/**
\brief  Set binding state for the protocol driver

Set the status of the protocol binding.

\param state_p  State to set.

\ingroup module_ndis
*/
//------------------------------------------------------------------------------
void ndis_setBindingState(ULONG state_p)
{
    protocol_setBindingState(state_p);
}

//------------------------------------------------------------------------------
/**
\brief  Allocate Tx and Rx buffers

This routine invokes the allocation routine for the Tx and Rx buffers in the
NDIS driver which initializes the receive and transmit queues.

\param  txBuffCount_p       Tx buffer count.
\param  rxBuffCount_p       Rx buffer count.

\return Returns tNdisErrorStatus error code.
\retval kNdisStatusSuccess If buffers allocated successfully.
\retval kNdisStatusNoResource If some error occurred during allocation.

\ingroup module_ndis
*/
//------------------------------------------------------------------------------
tNdisErrorStatus ndis_allocateTxRxBuff(UINT txBuffCount_p, UINT rxBuffCount_p)
{
    NDIS_STATUS status;
    status = protocol_allocateTxRxBuf(txBuffCount_p, rxBuffCount_p);

    if (status != NDIS_STATUS_SUCCESS)
        return kNdisStatusNoResource;

    return kNdisStatusSuccess;
}

//------------------------------------------------------------------------------
/**
\brief  Free Tx and Rx buffer

\ingroup module_ndis
*/
//------------------------------------------------------------------------------
void ndis_freeTxRxBuff(void)
{
    protocol_freeTxRxBuffers();
}

//------------------------------------------------------------------------------
/**
\brief  Get Tx buffer

This routines gets a Tx buffer to be shared with the caller from the NDIS driver.

\param  ppData_p     Double pointer to buffer pointer.
\param  size_p       Size of the buffer.
\param  ppTxLink_p   Double pointer to LIST_ENTRY to track the Tx buffer during
                     subsequent calls.

\return Returns tNdisErrorStatus error code.

\ingroup module_ndis
*/
//------------------------------------------------------------------------------
tNdisErrorStatus ndis_getTxBuff(void** ppData_p, size_t size_p, void** ppTxLink_p)
{
    tTxBufInfo*   pTxBuffInfo = protocol_getTxBuff(size_p);

    if (pTxBuffInfo == NULL)
        return kNdisStatusNoResource;

    if (ppData_p == NULL || ppTxLink_p == NULL)
        return kNdisStatusInvalidParams;

    *ppData_p = pTxBuffInfo->pData;
    *ppTxLink_p = (void*)&pTxBuffInfo->txLink;

    return kNdisStatusSuccess;
}

//------------------------------------------------------------------------------
/**
\brief  Free Tx buffer

This routines frees the previously allocated Tx buffer that was shared with
the caller.

\param  pTxLink_p      Pointer to LIST_ENTRY of the Txbuffer.

\ingroup module_ndis
*/
//------------------------------------------------------------------------------
void ndis_freeTxBuff(void* pTxLink_p)
{
    protocol_freeTxBuff(pTxLink_p);
}

//------------------------------------------------------------------------------
/**
\brief  Send packet to the lower miniport

Hands the packet to the lower miniport driver to forward it to the NIC.

\param  pData_p      Pointer to data buffer.
\param  size_p       Size of the packet to send.
\param  pTxLink_p    Pointer to link list entry for the Tx buffer.

\return Returns tNdisErrorStatus error code.

\ingroup module_ndis
*/
//------------------------------------------------------------------------------
tNdisErrorStatus ndis_sendPacket(void* pData_p, size_t size_p, void* pTxLink_p)
{
    NDIS_STATUS    ndisStatus;

    if (pData_p == NULL || pTxLink_p == NULL)
    {
        return kNdisStatusInvalidParams;
    }

    ndisStatus = protocol_sendPacket(pData_p, size_p, pTxLink_p);

    if (ndisStatus != NDIS_STATUS_SUCCESS)
    {
        return kNdisStatusTxError;
    }

    return kNdisStatusSuccess;
}

//------------------------------------------------------------------------------
/**
\brief  Register Tx and Rx callbacks

Register Tx and Rx callbacks with NDIS driver.

\param  pfnTxCallback_p      Pointer to Tx callback routine.
\param  pfnRxCallback_p      Pointer to Rx callback routine.

\ingroup module_ndis
*/
//------------------------------------------------------------------------------
void ndis_registerTxRxHandler(tNdisTransmitCompleteCb pfnTxCallback_p,
                              tNdisReceiveCb pfnRxCallback_p)
{
    protocol_registerTxRxHandler(pfnTxCallback_p, pfnRxCallback_p);
}

//------------------------------------------------------------------------------
/**
\brief  Register VEth Tx callback

Register VEth Tx callback with NDIS driver.

\param  pfnVEthTxCallback_p      Pointer to Tx callback routine.

\ingroup module_ndis
*/
//------------------------------------------------------------------------------
void ndis_registerVethHandler(tVEthSendCb pfnVEthTxCallback_p)
{
    protocol_registerVEthHandler(pfnVEthTxCallback_p);
}

//------------------------------------------------------------------------------
/**
\brief  Receive packet handler for non-PLK packets

This routine can be invoked to indicate the reception of a non-PLK packets and
forward it to NDIS driver. The miniport section of the driver then assembles
the packets into the NET_BUFFER_LISTS structure and forwards it to protocol
drivers bound to the miniport.

\param  pData_p      Pointer to the received frame.
\param  size_p       Size of the received frame.

\ingroup module_ndis
*/
//------------------------------------------------------------------------------
tNdisErrorStatus ndis_vethReceive(void* pData_p, size_t size_p)
{
    NDIS_STATUS     status = NDIS_STATUS_SUCCESS;

    if (pData_p == NULL)
        return kNdisStatusInvalidParams;

    status = miniport_handleReceive(pData_p, size_p);

    if (status != NDIS_STATUS_SUCCESS)
    {
        return kNdisStatusTxError;
    }

    return kNdisStatusSuccess;
}

//------------------------------------------------------------------------------
/**
\brief  Create driver interface device

This routines calls the IOCTL interface registration routine which initializes
an IOCTL interface for user application to interact with the driver.

\ingroup module_ndis
*/
//------------------------------------------------------------------------------
void ndis_createDrvIntf(void)
{
    driverInstance_g.pfnDrvIntfRegCb(driverInstance_g.hMiniportHandle);
}

//------------------------------------------------------------------------------
/**
\brief  Close driver interface device

Close the IOCTL interface created previously.

\ingroup module_ndis
*/
//------------------------------------------------------------------------------
void ndis_closeDrvIntf(void)
{
    driverInstance_g.pfnDrvIntfDeregisterCb();
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief  Miniport set options routine for NDIS driver

This routine registers optional services for the miniport section of the NDIS driver
and can allocate other driver resources which the NDIS driver may use.

\param  driverHandle_p      Miniport driver handle.
\param  driverContext_p     Specifies a handle to a driver-allocated context area
                            where the driver maintains state and configuration
                            information.

\note Nothing to be done since this feature is not used by the device in current
      implementation.

\return The function returns an NDIS_STATUS error code.

*/
//------------------------------------------------------------------------------
NDIS_STATUS miniportSetOptions(NDIS_HANDLE driverHandle_p, NDIS_HANDLE driverContext_p)
{
    UNREFERENCED_PARAMETER(driverHandle_p);
    UNREFERENCED_PARAMETER(driverContext_p);
    return NDIS_STATUS_SUCCESS;
}

//------------------------------------------------------------------------------
/**
\brief  Protocol set options routine for NDIS driver

This routine registers the optional handlers for the protocol section of driver
with NDIS.

\param  driverHandle_p      Protocol driver handle.
\param  driverContext_p     Specifies a handle to a driver-allocated context area
                            where the driver maintains state and configuration
                            information.

\note: Nothing to be done in this routine in current implementation..

\return The function returns an NDIS_STATUS error code.

*/
//------------------------------------------------------------------------------
NDIS_STATUS protocolSetOptions(NDIS_HANDLE driverHandle_p, NDIS_HANDLE driverContext_p)
{
    UNREFERENCED_PARAMETER(driverHandle_p);
    UNREFERENCED_PARAMETER(driverContext_p);
    return NDIS_STATUS_SUCCESS;
}

/// \}
