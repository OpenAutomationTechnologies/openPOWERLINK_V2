/**
********************************************************************************
\file   vethMiniport.h

\brief  Declaration of the CVEthMiniport

This file contains the class declaration for the CVEthMiniport class which
represents the virtual Ethernet adapter created by the NDIS intermediate driver.

The class helps to manage the initialization, PNP events, installation,
user actions on the adapter etc. for the virtual Ethernet adapter.

\ingroup module_notify_ndisim
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

#ifndef _INC_vethminiport_H_
#define _INC_vethminiport_H_


#include <windows.h>

#include "netcfgn.h"
#include "common.h"

//------------------------------------------------------------------------------
/**
\class CVEthMiniport

\brief Virtual Ethernet miniport instance

This class represents the virtual miniport adapter which is registered to the OS.
It is used to manage the list of protocols which can attach to the miniport and
communicate to the driver.

*/
//------------------------------------------------------------------------------
class CVEthMiniport
{
public:
    //-------------------------------------------------------------------------
    /**
    \brief Constructor for class CVEthMiniport

    \param  pInetCfg_p      Pointer to InetCfg assigned to the notify object.
    \param  pGuidMiniport_p Pointer to the Miniport GUID to initialize.
    \param  pGuidAdapter_p  Pointer to adapter GUID for which the miniport is
                            initialized.

    */
    //-------------------------------------------------------------------------
    CVEthMiniport(INetCfg* pInetCfg_p, GUID* pGuidMiniport_p, GUID* pGuidAdapter_p);

    //-------------------------------------------------------------------------
    /**
    \brief Destructor for class CVEthMiniport

    */
    //-------------------------------------------------------------------------
    ~CVEthMiniport();

    //-------------------------------------------------------------------------
    /**
    \brief  Initialize the miniport

    \note: Nothing to be done in current implementation.

    \return HRESULT error code.
    \retval Always returns S_OK.

    */
    //-------------------------------------------------------------------------
    HRESULT Initialize(VOID);

    //-------------------------------------------------------------------------
    /**
    \brief  Return the GUID the adapter to which miniport is linked with

    \param  pGuidAdapter_p  Pointer to receive the adapter GUID.

    \return Returns the adapter GUID.

    */
    //-------------------------------------------------------------------------
    VOID GetAdapterGUID(GUID* pGuidAdapter_p);

    //-------------------------------------------------------------------------
    /**
    \brief  Return the GUID of this miniport

    \param  pGuidMiniport_p Pointer to receive the miniport GUID.

    \return Returns the miniport GUID.

    */
    //-------------------------------------------------------------------------
    VOID GetMiniportGUID(GUID* pGuidMiniport_p);

    //-------------------------------------------------------------------------
    /**
    \brief  Install the virtual Ethernet miniport

    Performs the necessary operation to install this miniport and register it
    with the OS.

    \return HRESULT error code.

    */
    //-------------------------------------------------------------------------
    HRESULT Install(VOID);

    //-------------------------------------------------------------------------
    /**
    \brief  Uninstall the virtual Ethernet miniport

    \return HRESULT error code.

    */
    //-------------------------------------------------------------------------
    HRESULT DeInstall(VOID);

    //-------------------------------------------------------------------------
    /**
    \brief  Set configuration action to be performed

    \param  applyAction_p   Action to be performed.

    \return HRESULT error code.

    */
    //-------------------------------------------------------------------------
    void SetConfigAction(tConfigAction applyAction_p);

    //-------------------------------------------------------------------------
    /**
    \brief  Store the changes in the registry

    \return HRESULT error code.

    */
    //-------------------------------------------------------------------------
    HRESULT ApplyRegistryChanges(VOID);

    //-------------------------------------------------------------------------
    /**
    \brief  Make necessary PNP changes for the miniport

    \note The implementation does not perform any PNP changes.

    \param  pfnCallback     Pointer to PNP reconfigure callback interface.

    \return HRESULT error code.
    \retval Always returns S_OK.

    */
    //-------------------------------------------------------------------------
    HRESULT ApplyPnpChanges(INetCfgPnpReconfigCallback* pfnCallback);

private:
    INetCfg*        pInetCfg;           ///< Pointer to InetCfg component for the notify object
    GUID            guidAdapter;        ///< Adapter GUID for this miniport
    GUID            guidMiniport;       ///< GUID for this miniport
    tConfigAction   applyAction;        ///< Configuration to be applied in registry
};

#endif // _INC_vethminiport_H_
