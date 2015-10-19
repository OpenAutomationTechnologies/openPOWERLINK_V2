/**
********************************************************************************
\file   phyAdapter.h

\brief  Declaration of the CPhyAdapter

This file contains the class declaration for the CPhyAdapter class which
represents the physical adapter present in the system. The CPhyAdapter class
is used to manage the initialization, protocol binding etc. for a physical
adapter.

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

#ifndef _INC_phyadapter_H_
#define _INC_phyadapter_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <windows.h>
#include <netcfgn.h>

#include "common.h"
#include "vethMiniport.h"

//------------------------------------------------------------------------------
/**
\class CPhyAdapter

\brief Physical adapter class implementation

The physical adapter represents Miniport driver to which the protocol section of
the NDIS intermediate driver will bind with.

*/
//------------------------------------------------------------------------------
class CPhyAdapter
{
public:
    //-------------------------------------------------------------------------
    /**
    \brief Constructor for class CPhyAdapter

    \param  pInetCfg_p      Pointer to InetCfg assigned to the notify object.
    \param  pGuidAdapter_p  Reference to adapter GUID to add.

    */
    //-------------------------------------------------------------------------
    CPhyAdapter(INetCfg* pInetCfg_p, const GUID& pGuidAdapter_p);

    //-------------------------------------------------------------------------
    /**
    \brief Destructor for class CPhyAdapter

    */
    //-------------------------------------------------------------------------
    ~CPhyAdapter();

    //-------------------------------------------------------------------------
    /**
    \brief  Initialize the adapter

    Read the registry to get the device IDs of the virtual miniports installed
    on the adapter and crate an instance to represent each virtual miniport.

    \return HRESULT error code.

    */
    //-------------------------------------------------------------------------
    HRESULT Initialize(VOID);

    //-------------------------------------------------------------------------
    /**
    \brief  Returns adapter GUID

    \param  pGuidAdapter_p  Pointer to adapter GUID returned.

    \return Adapter GUID of the current instance.

    */
    //-------------------------------------------------------------------------
    VOID    GetAdapterGUID(GUID* pGuidAdapter_p);

    //-------------------------------------------------------------------------
    /**
    \brief  Add the miniport

    Initializes the  miniport for this adapter and marks it to be installed.

    \param  pNewMiniport    Pointer of the miniport to add.

    \return HRESULT error code.

    */
    //-------------------------------------------------------------------------
    HRESULT AddMiniport(CVEthMiniport* pNewMiniport);

    //-------------------------------------------------------------------------
    /**
    \brief  Remove the miniport

    Uninstall the miniport instance for this adapter and delete the instance.

    \return HRESULT error code.

    */
    //-------------------------------------------------------------------------
    HRESULT RemoveMiniport(VOID);

    //-------------------------------------------------------------------------
    /**
    \brief  Update the registry depending on the action to performed

    Perform the registry changes which were updated in SetConfigAction().

    \return HRESULT error code.

    */
    //-------------------------------------------------------------------------
    HRESULT ApplyRegistryChanges(void);

    //-------------------------------------------------------------------------
    /**
    \brief  Update config action for this adapter

    \param  applyAction_p       Action to be performed.

    \return HRESULT error code.

    */
    //-------------------------------------------------------------------------
    HRESULT SetConfigAction(tConfigAction applyAction_p);

    //-------------------------------------------------------------------------
    /**
    \brief  Apply the PnP changes depending on the action to be performed

    \param  pfnCallback Pointer to sendPnpConfig callback interface.

    \return HRESULT error code.

    */
    //-------------------------------------------------------------------------
    HRESULT ApplyPnpChanges(INetCfgPnpReconfigCallback* pfnCallback);

    //-------------------------------------------------------------------------
    /**
    \brief  Cancel any changes made.

    \return HRESULT error code.

    */
    //-------------------------------------------------------------------------
    HRESULT CancelChanges(VOID);

    //-------------------------------------------------------------------------
    /**
    \brief  Check if the miniport instance is initialized

    \return The routine returns BOOL value.
    \retval TRUE, if the miniport instance exists.
    \retval FALSE, otherwise.

    */
    //-------------------------------------------------------------------------
    BOOL MiniportPresent(VOID);

private:
    GUID            guidAdapter;                ///< GUID of the adapter
    INetCfg*        pInetCfg;                   ///< Pointer to the InetCfg instance of the notify object
    CVEthMiniport*  pVethMiniport;              ///< Pointer to the miniport instance for this adapter
    tConfigAction   applyAction;                ///< Configuration action to be applied to registry
};

#endif // _INC_phyadapter_H_
