/**
********************************************************************************
\file   phyAdapter.cpp

\brief  Implementation of CPhyAdapter class

This file implements the methods and private routines for CPhyAdapter class.

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

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include "stdafx.h"
#include "phyAdapter.h"

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

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

CPhyAdapter::CPhyAdapter(INetCfg* pInetCfg_p, const GUID& guidAdapter_p)
{
    this->pInetCfg = pInetCfg_p;
    this->pInetCfg->AddRef();

    CopyMemory(&guidAdapter, &guidAdapter_p, sizeof(GUID));

    this->pVethMiniport = NULL;
}

CPhyAdapter::~CPhyAdapter()
{
    if (this->pVethMiniport != NULL)
        delete this->pVethMiniport;

    RELEASE_OBJ(this->pInetCfg);
}

HRESULT CPhyAdapter::Initialize(VOID)
{
    HKEY                adapterGuidKey;
    WCHAR               aAdapterGuidKey[MAX_PATH + 1];
    WCHAR               aAdapterGuid[MAX_PATH + 1];
    LPWSTR              pMiniportList = NULL;
    LPWSTR              pMiniport = NULL;
    LPWSTR              pMiniportGuid = NULL;
    DWORD               dispostion;
    CVEthMiniport*      pMiniportInst = NULL;
    GUID                miniportGUID;
    DWORD               bytes = 0;;
    LONG                result;

    StringFromGUID2(guidAdapter, aAdapterGuid, MAX_PATH + 1);

    StringCchPrintf(aAdapterGuidKey,
                    celems(aAdapterGuidKey),
                    L"%s\\%s",
                    aAdapterList_g,
                    aAdapterGuid);

    aAdapterGuidKey[MAX_PATH] = '\0';

    result = RegCreateKeyEx(HKEY_LOCAL_MACHINE,
                            aAdapterGuidKey,
                            0,
                            NULL,
                            REG_OPTION_NON_VOLATILE,
                            KEY_ALL_ACCESS,
                            NULL,
                            &adapterGuidKey,
                            &dispostion);

    if (result != ERROR_SUCCESS)
        return HRESULT_FROM_WIN32(result);

    if (dispostion == REG_CREATED_NEW_KEY)
    {
        // New key created, no need to setup
        TRACE(L"New Key Created\n");
        goto Exit;
    }

    result = RegQueryValueEx(adapterGuidKey,   // Registry key to query
                             aUpperBindings_g,  // Name of the registry value
                             NULL,              // Reserved
                             NULL,              // Variable type
                             NULL,              // Data in the variable
                             &bytes);
    if (result != ERROR_SUCCESS)
    {
        TRACE(L"Unable to query registry. Failed with (0xX)\n", result);
        goto Exit;
    }

    pMiniportList = (LPWSTR)calloc(bytes, 1);

    if (pMiniportList == NULL)
    {
        result = ERROR_NOT_ENOUGH_MEMORY;
        goto Exit;
    }

    result = RegQueryValueEx(adapterGuidKey,        // Registry key to query
                             aUpperBindings_g,       // Name of the registry value
                             NULL,                   // Reserved
                             NULL,                   // Variable type
                             (LPBYTE)pMiniportList,  // Data in the variable
                             &bytes);

    if (result != ERROR_SUCCESS)
    {
        goto Exit;
    }

    pMiniport = pMiniportList;

    pMiniportGuid = notify_removeDevicePrefix(pMiniport);

    TRACE(L"Loading configuration for miniport %s...\n",
          pMiniportGuid);

    if (pMiniportGuid)
    {
        CLSIDFromString(pMiniportGuid, &miniportGUID);

        pMiniportInst = new CVEthMiniport(pInetCfg, &miniportGUID,
                                          &this->guidAdapter);

        if (pMiniportInst == NULL)
        {
            result = ERROR_NOT_ENOUGH_MEMORY;
            goto Exit;
        }

        // Initialize the miniport
        pMiniportInst->Initialize();

        // Store the pointer
        this->pVethMiniport = pMiniportInst;
    }

Exit:
    if (pMiniportGuid != NULL)
        free(pMiniportGuid);

    if (pMiniportList != NULL)
        free(pMiniportList);

    RegCloseKey(adapterGuidKey);

    TRACE(L"<---CPhyAdapter::Initialize HRESULT %x \n", result);
    return HRESULT_FROM_WIN32(result);
}

VOID CPhyAdapter::GetAdapterGUID(GUID* pGuidAdapter_p)
{
    CopyMemory(pGuidAdapter_p, &this->guidAdapter, sizeof(GUID));
}

HRESULT CPhyAdapter::AddMiniport(CVEthMiniport* pNewMiniport)
{
    if (this->pVethMiniport != NULL)
        return HRESULT_FROM_WIN32(ERROR_ALREADY_ASSIGNED);

    this->pVethMiniport = pNewMiniport;
    this->pVethMiniport->SetConfigAction(kActAdd);

    return S_OK;
}

HRESULT CPhyAdapter::RemoveMiniport(VOID)
{
    HRESULT hret = S_OK;

    if (this->pVethMiniport == NULL)
        return HRESULT_FROM_WIN32(ERROR_INVALID_PARAMETER);

    hret = this->pVethMiniport->DeInstall();
    if (hret != S_OK)
    {
        TRACE(L"Failed to Uninstall VEth\n");
        return hret;
    }

    this->pVethMiniport->SetConfigAction(kActRemove);
    return S_OK;
}

HRESULT CPhyAdapter::ApplyRegistryChanges()
{
    HKEY                    adapterListKey;
    HKEY                    adapterGuidKey;
    WCHAR                   aAdapterGuid[MAX_PATH + 1];
    CVEthMiniport*          pMiniportInst = NULL;
    DWORD                   miniportCount;
    DWORD                   disposition;
    DWORD                   i;
    LONG                    result;
    HRESULT                 hret;

    StringFromGUID2(this->guidAdapter, aAdapterGuid, (MAX_PATH + 1));

    result = RegCreateKeyEx(HKEY_LOCAL_MACHINE,
                            aAdapterList_g,
                            0,
                            NULL,
                            REG_OPTION_NON_VOLATILE,
                            KEY_ALL_ACCESS,
                            NULL,
                            &adapterListKey,
                            &disposition);

    if (result == ERROR_SUCCESS)
    {
        result = RegCreateKeyEx(adapterListKey,
                                aAdapterGuid,
                                0,
                                NULL,
                                REG_OPTION_NON_VOLATILE,
                                KEY_ALL_ACCESS,
                                NULL,
                                &adapterGuidKey,
                                &disposition);

        if (result == ERROR_SUCCESS)
        {
            RegCloseKey(adapterGuidKey);
        }
        else
        {
            TRACE(L"Failed to Create/Open the registry key: %s\\%s.\n",
                  aAdapterList_g, aAdapterGuid);
        }
    }
    else
    {
        TRACE(L"Failed to open the registry key: %s.\n",
              aAdapterList_g);
        return HRESULT_FROM_WIN32(result);
    }

    this->pVethMiniport->ApplyRegistryChanges();

    // Delete the registry key if adapter is being removed
    if (this->applyAction == kActRemove)
    {
        // Delete the key
        RegDeleteKey(adapterListKey, aAdapterGuid);
        RegCloseKey(adapterListKey);
    }

    return S_OK;
}

HRESULT CPhyAdapter::SetConfigAction(tConfigAction applyAction_p)
{
    this->applyAction = applyAction_p;

    if (this->pVethMiniport != NULL)
        this->pVethMiniport->SetConfigAction(applyAction_p);

    return S_OK;
}

HRESULT CPhyAdapter::ApplyPnpChanges(INetCfgPnpReconfigCallback* pfnCallback)
{
    CVEthMiniport*      pMiniportInst = this->pVethMiniport;

    switch (this->applyAction)
    {
        case kActRemove:
            pMiniportInst->SetConfigAction(kActRemove);
            pMiniportInst->ApplyPnpChanges(pfnCallback);
            delete pMiniportInst;
            this->pVethMiniport = NULL;
            break;

        case kActAdd:
            pMiniportInst->SetConfigAction(kActAdd);
            pMiniportInst->ApplyPnpChanges(pfnCallback);
            break;

        case kActUpdate:
            pMiniportInst->ApplyPnpChanges(pfnCallback);
            break;

        default:
            TRACE(L"Unknown configuration action\n");
            break;
    }

    return S_OK;
}

HRESULT CPhyAdapter::CancelChanges(VOID)
{
    return S_OK;
}

BOOL CPhyAdapter::MiniportPresent(VOID)
{
    if (this->pVethMiniport != NULL)
        return TRUE;

    return FALSE;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

/// \}
