/**
********************************************************************************
\file   vethMiniport.cpp

\brief  Implementation of CVethMiniport class

This file implements the class methods and private routines for CVethMiniport
class.

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
#include "vethMiniport.h"

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

CVEthMiniport::CVEthMiniport(INetCfg* pInetCfg_p, GUID* pGuidMiniport_p, GUID* pGuidAdapter_p)
{
    TRACE(L"---->Constructor CVEthMiniport.\n");

    this->pInetCfg = pInetCfg_p;
    this->pInetCfg->AddRef();

    CopyMemory(&this->guidAdapter, pGuidAdapter_p, sizeof(GUID));

    if (pGuidMiniport_p)
        CopyMemory(&this->guidMiniport, pGuidMiniport_p, sizeof(GUID));
    else
        ZeroMemory(&this->guidMiniport, sizeof(GUID));

    this->applyAction = kActUnknown;

    TRACE(L"<----Constructor CVEthMiniport.\n");
}

CVEthMiniport::~CVEthMiniport()
{
    TRACE(L"--->~CVEthMiniport.\n");
    RELEASE_OBJ(this->pInetCfg);
    TRACE(L"<---~CVEthMiniport.\n");
}

HRESULT CVEthMiniport::Initialize(VOID)
{
    TRACE(L"Initialize.\n");
    return S_OK;
}

VOID CVEthMiniport::GetAdapterGUID(GUID* pGuidAdapter_p)
{
    TRACE(L"GetAdapterGUID.\n");
    if (pGuidAdapter_p)
        CopyMemory(pGuidAdapter_p, &this->guidAdapter, sizeof(GUID));

}

VOID CVEthMiniport::GetMiniportGUID(GUID* pGuidMiniport_p)
{
    TRACE(L"GetMiniportGUID.\n");
    if (pGuidMiniport_p)
        CopyMemory(pGuidMiniport_p, &this->guidMiniport, sizeof(GUID));
}

HRESULT CVEthMiniport::Install(VOID)
{
    INetCfgClass*           pNCClass = NULL;
    INetCfgClassSetup*      pNCClassSetup = NULL;
    INetCfgComponent*       pNCComponentMiniport = NULL;
    HRESULT                 hret = S_OK;
    LPWSTR*                 pRefs = NULL;
    OBO_TOKEN*              pOboToken = NULL;
    DWORD                   setupFlags = 0;
    LPCWSTR                 pAnswerFile = NULL;
    LPCWSTR                 pAnswerSections = NULL;

    TRACE(L"------>CVEthMiniport::Install.\n");

    hret = this->pInetCfg->QueryNetCfgClass(&GUID_DEVCLASS_NET, IID_INetCfgClass,
                                            (void**)&pNCClass);
    if (hret != S_OK)
    {
        return hret;
    }

    hret = pNCClass->QueryInterface(IID_INetCfgClassSetup, (void**)&pNCClassSetup);
    if (hret != S_OK)
    {
        goto Exit;
    }

    hret = pNCClassSetup->Install(aOplkMiniport_g,  // Component identifier
                                  pOboToken,        // Obo token for reference count,check
                                  setupFlags,       // Installation setup flags, check
                                  0,                // Build identifier if any, check
                                  pAnswerFile,
                                  pAnswerSections,
                                  &pNCComponentMiniport);
    if (hret != S_OK)
    {
        goto Exit;
    }

    hret = pNCComponentMiniport->GetInstanceGuid(&this->guidMiniport);

    if (hret != S_OK)
    {
        pNCClassSetup->DeInstall(pNCComponentMiniport,
                                 pOboToken,
                                 pRefs);
    }

Exit:
    if (pNCComponentMiniport)
        RELEASE_OBJ(pNCComponentMiniport);

    if (pNCClassSetup)
        RELEASE_OBJ(pNCClassSetup);

    if (pNCClass)
        RELEASE_OBJ(pNCClass);

    TRACE(L"<------Install HRESULT (%x)\n", hret);
    return hret;
}

HRESULT CVEthMiniport::DeInstall(VOID)
{
    INetCfgClass*           pNCClass;
    INetCfgClassSetup*      pNCClassSetup;
    INetCfgComponent*       pNCComponentMiniport;
    HRESULT                 hret;
    LPWSTR*                 pRefs = NULL;
    OBO_TOKEN*              pOboToken = NULL;

    hret = this->pInetCfg->QueryNetCfgClass(&GUID_DEVCLASS_NET, IID_INetCfgClass,
                                            (PVOID*)&pNCClass);

    if (hret != S_OK)
        return hret;

    hret = pNCClass->QueryInterface(IID_INetCfgClassSetup,
                                    (PVOID*)&pNCClassSetup);

    if (hret != S_OK)
        return hret;

    hret = notify_findInstance(this->pInetCfg,
                               this->guidMiniport,
                               &pNCComponentMiniport);

    if (hret != S_OK)
        goto Exit;

    hret = pNCClassSetup->DeInstall(pNCComponentMiniport,
                                    pOboToken,
                                    pRefs);

Exit:
    if (pNCComponentMiniport)
        RELEASE_OBJ(pNCComponentMiniport);

    if (pNCClassSetup)
        RELEASE_OBJ(pNCClassSetup);

    if (pNCClass)
        RELEASE_OBJ(pNCClass);
    TRACE(L"<------DeInstall (HRESULT(%x).\n", hret);
    return hret;
}

HRESULT CVEthMiniport::ApplyRegistryChanges(VOID)
{
    HKEY                    adapterGuidKey;
    WCHAR                   aAdapterGuid[MAX_PATH + 1];
    WCHAR                   aAdapterGuidKey[MAX_PATH + 1];
    WCHAR                   aMiniportGuid[MAX_PATH + 1];
    LPWSTR                  pDevice = NULL;
    LONG                    result = 0;

    TRACE(L"----->CVEthMiniport::ApplyRegistryChanges.\n");

    switch (this->applyAction)
    {
        case kActAdd:
            // Check if the adapter exists
            TRACE(L"kActAdd\n");
            StringFromGUID2(this->guidAdapter,
                            aAdapterGuid,
                            MAX_PATH + 1);

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
                                    NULL);

            if (result == ERROR_SUCCESS)
            {
                StringFromGUID2(this->guidMiniport,
                                aMiniportGuid,
                                MAX_PATH + 1);

                pDevice = notify_addDevicePrefix(aMiniportGuid);

                if (pDevice == NULL)
                {
                    result = ERROR_NOT_ENOUGH_MEMORY;
                    break;
                }

                result = RegSetValueEx(adapterGuidKey,
                                       aUpperBindings_g,
                                       0,
                                       REG_SZ,
                                       (LPBYTE)pDevice,
                                       (wcslen(pDevice) + 1) *
                                       (DWORD)sizeof(WCHAR)
                                       );
                if (result != ERROR_SUCCESS)
                {

                    TRACE(L"   Failed to save %s at %s\\%s.\n",
                          result,
                          aAdapterGuidKey,
                          aUpperBindings_g);

                }

                RegCloseKey(adapterGuidKey);
            }
            else
            {
                TRACE(L"   Failed to open the registry key: %s.\n",
                      aAdapterGuidKey);
            }

            break;

        case kActRemove:
            // Check if the adapter exists
            TRACE(L"kActRemove\n");
            StringFromGUID2(this->guidAdapter,
                            aAdapterGuid,
                            MAX_PATH + 1);


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
                                    NULL);

            if (result == ERROR_SUCCESS)
            {
                StringFromGUID2(this->guidMiniport,
                                aMiniportGuid,
                                MAX_PATH + 1);

                pDevice = notify_addDevicePrefix(aMiniportGuid);

                if (pDevice == NULL)
                {
                    result = ERROR_NOT_ENOUGH_MEMORY;
                    break;
                }

                result = RegDeleteValue(adapterGuidKey,
                                        aUpperBindings_g);
                if (result != ERROR_SUCCESS)
                {

                    TRACE(L"   Failed to delete %s at %s\\%s.\n",
                          pDevice,
                          aAdapterGuidKey,
                          aUpperBindings_g);
                }
            }

            break;

        default:
            TRACE(L"Unknown configuration action\n");
            break;
    }

    if (pDevice)
        free(pDevice);

    TRACE(L"<----- CVEthMiniport::ApplyRegistryChanges (HRESULT (%x)).\n", HRESULT_FROM_WIN32(result));
    return HRESULT_FROM_WIN32(result);
}

HRESULT CVEthMiniport::ApplyPnpChanges(INetCfgPnpReconfigCallback* pfnCallback)
{
    TRACE(L"CVEthMiniport::ApplyPnpChanges.\n");

    UNREFERENCED_PARAMETER(pfnCallback);
    return S_OK;
}

void CVEthMiniport::SetConfigAction(tConfigAction applyAction_p)
{
    applyAction = applyAction_p;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

/// \}
