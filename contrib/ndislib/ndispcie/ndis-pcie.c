/**
********************************************************************************
\file   ndis-pcie.c

\brief  NDIS library interface routines for NDIS miniport driver for PCIe

This file implements the initialization routines of an NDIS miniport driver for
PCIe card. It also provides helper routines for memory initialization,
interrupt registration and other communication.

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

#include "ndisDriver.h"

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

This routine registers the NDIS miniport characteristics and entry
routines to the OS using NdisXRegisterXXXDriver.

\param  pDriverObject_p     Pointer to the system's driver object structure
                            for this driver.
\param  pRegistryPath_p     System's registry path for this driver.

\return The function returns an NDIS_STATUS error code.

\ingroup module_ndis
*/
//------------------------------------------------------------------------------
NDIS_STATUS ndis_initDriver(PDRIVER_OBJECT pDriverObject_p, PUNICODE_STRING pRegistryPath_p)
{
    NDIS_STATUS                             ndisStatus = NDIS_STATUS_SUCCESS;
    NDIS_MINIPORT_DRIVER_CHARACTERISTICS    miniportChars;
    NDIS_HANDLE                             miniportDriverContext = NULL;

    NdisZeroMemory(&driverInstance_g, sizeof(tNdisDriverInstance));

    NdisZeroMemory(&miniportChars, sizeof(NDIS_MINIPORT_DRIVER_CHARACTERISTICS));
    miniportChars.Header.Type                   = NDIS_OBJECT_TYPE_MINIPORT_DRIVER_CHARACTERISTICS;
    miniportChars.Header.Size                   = sizeof(NDIS_MINIPORT_DRIVER_CHARACTERISTICS);
    miniportChars.Header.Revision               = NDIS_MINIPORT_DRIVER_CHARACTERISTICS_REVISION_1;
    miniportChars.MajorNdisVersion              = OPLK_MAJOR_NDIS_VERSION;
    miniportChars.MinorNdisVersion              = OPLK_MINOR_NDIS_VERSION;
    miniportChars.MajorDriverVersion            = OPLK_MAJOR_DRIVER_VERSION;
    miniportChars.MinorDriverVersion            = OPLK_MINOR_DRIVER_VERSION;

    miniportChars.SetOptionsHandler             = miniportSetOptions;
    miniportChars.InitializeHandlerEx           = miniportInitialize;
    miniportChars.UnloadHandler                 = miniportUnload;
    miniportChars.HaltHandlerEx                 = miniportHalt;
    miniportChars.OidRequestHandler             = miniportOidRequest;
    miniportChars.CancelSendHandler             = miniportCancelSendNetBufferLists;
    miniportChars.DevicePnPEventNotifyHandler   = miniportPnpEventNotify;
    miniportChars.ShutdownHandlerEx             = miniportShutdown;
    miniportChars.CancelOidRequestHandler       = miniportCancelOidRequest;
    miniportChars.CheckForHangHandlerEx         = miniportCheckForHang;
    miniportChars.ReturnNetBufferListsHandler   = miniportReturnNetBufferLists;
    miniportChars.SendNetBufferListsHandler     = miniportSendNetBufferLists;
    miniportChars.PauseHandler                  = miniportPause;
    miniportChars.RestartHandler                = miniportRestart;
    miniportChars.ResetHandlerEx                = miniportReset;

    ndisStatus = NdisMRegisterMiniportDriver(pDriverObject_p, pRegistryPath_p, miniportDriverContext,
                                             &miniportChars, &driverInstance_g.pMiniportHandle);
    if (ndisStatus != NDIS_STATUS_SUCCESS)
    {
        TRACE("%s() Miniport driver registration failed 0x%X\n", __FUNCTION__, ndisStatus);
    }

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
    return driverInstance_g.pMiniportHandle;
}

//------------------------------------------------------------------------------
/**
\brief  Register driver interface routines

Register routines for IOCTL interface registration and de-registration. NDIS
driver calls the registration routine from miniport initialization callback,
and de-registration from miniport halt callback.

\param  pAppIntfRegCb_p       Pointer to register callback.
\param  pAppIntfDeregCb_p     Pointer to de-register callback.

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
\brief  Create driver interface device

This routine calls the IOCTL interface registration routine which initializes
an IOCTL interface for the user application to interact with the driver.

\ingroup module_ndis
*/
//------------------------------------------------------------------------------
void ndis_createDrvIntf(void)
{
    if (driverInstance_g.pfnDrvIntfRegCb != NULL)
        driverInstance_g.pfnDrvIntfRegCb(driverInstance_g.pMiniportHandle);
}

//------------------------------------------------------------------------------
/**
\brief  Close driver interface device

Close the IOCTL interface created with \ref ndis_createDrvIntf.

\ingroup module_ndis
*/
//------------------------------------------------------------------------------
void ndis_closeDrvIntf(void)
{
    if (driverInstance_g.pfnDrvIntfDeregisterCb != NULL)
        driverInstance_g.pfnDrvIntfDeregisterCb();
}

//------------------------------------------------------------------------------
/**
\brief  Get PCIe BAR virtual address

This routine fetches the BAR address of the requested BAR.

\param  barId_p     ID of the requested BAR.

\return Returns the address of the requested PCIe BAR.

\ingroup module_ndis
*/
//------------------------------------------------------------------------------
PULONG ndis_getBarAddr(ULONG barId_p)
{
    if (vethInstance_g.state != kNdisBindingReady || barId_p >= OPLK_MAX_BAR_COUNT)
        return NULL;

    return vethInstance_g.barInfo[barId_p].pVirtualAddr;
}

//------------------------------------------------------------------------------
/**
\brief  Get BAR Length

\param  barId_p     ID of the requested BAR.

\return Returns the length of the requested PCIe BAR.

\ingroup module_ndis
*/
//------------------------------------------------------------------------------
ULONG ndis_getBarLength(ULONG barId_p)
{
    if (vethInstance_g.state != kNdisBindingReady || barId_p >= OPLK_MAX_BAR_COUNT)
        return 0;

    return vethInstance_g.barInfo[barId_p].length;
}

//------------------------------------------------------------------------------
/**
\brief  Register master interrupt handler

Register interrupt callback called from the ISR.

\param  pfnIntrCb_p     Pointer to master interrupt routine.

\ingroup module_ndis
*/
//------------------------------------------------------------------------------
void ndis_registerIntrHandler(tIntrHandler pfnIntrCb_p)
{
    vethInstance_g.pfnIntrCb = pfnIntrCb_p;
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

\note Nothing to be done since the feature is not used by the device now.

\return The function returns an NDIS_STATUS error code.

*/
//------------------------------------------------------------------------------
NDIS_STATUS miniportSetOptions(NDIS_HANDLE driverHandle_p, NDIS_HANDLE driverContext_p)
{
    UNREFERENCED_PARAMETER(driverHandle_p);
    UNREFERENCED_PARAMETER(driverContext_p);
    return NDIS_STATUS_SUCCESS;
}

///\}
