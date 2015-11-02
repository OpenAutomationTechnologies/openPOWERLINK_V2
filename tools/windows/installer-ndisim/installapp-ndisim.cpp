/**
********************************************************************************
\file   installapp-ndisim.cpp

\brief  Installation application for Windows NDIS intermediate driver

This file contains the implementation for installing openPOWERLINK Windows
NDIS intermediate driver.

It uses NETCFG COM interface routine to install the components.

The application is based on the ProtInstall application published at
http://ndis.com/ndis-general/ndisinstall/programinstall.htm by
Printing Communications Associates, Inc. (PCAUSA)

Please refer the readme.md file for additional license details

\ingroup install_app
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2004-2006, Printing Communications Associates, Inc. (PCAUSA)
                         http://www.pcausa.com
GPL software is an abomination. Far from being free, it is available ONLY
to members of the "GPL Club". If you don't want to join the club, then GPL
software is poison.

This software IS free software under the terms of a BSD-style license:

The right to use this code in your own derivative works is granted so long
as 1.) your own derivative works include significant modifications of your
own, 2.) you retain the above copyright notices and this paragraph in its
entirety within sources derived from this code.

This product includes software developed by PCAUSA. The name of PCAUSA
may not be used to endorse or promote products derived from this software
without specific prior written permission.

THIS SOFTWARE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR IMPLIED
WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED WARRANTIES OF
MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
-------------------------------------------------------------------------------

Microsoft Windows
Copyright (C) Microsoft Corporation, 2001.
Please refer readme.md in the source directory for detailed license information
-------------------------------------------------------------------------------

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
#include "installapp-ndisim.h"

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
static UINT32    installDriver(void);
static UINT32    uninstallDriver(void);
static HRESULT  getINetCfg(IN BOOL fGetWriteLock_p, IN LPCTSTR pAppName_p,
                           OUT INetCfg** ppNetCfg_p, OUT LPTSTR* ppLockedBy_p);
static HRESULT  releaseINetCfg(INetCfg* pNetCfg_p, BOOL fHasWriteLock_p);
static HRESULT  installNetComponent(IN INetCfg* pNetCfg_p,
                                    IN LPCTSTR pComponentId_p,
                                    IN const GUID* pGuidClass_p,
                                    IN LPCTSTR pInfFullPath_p);
static HRESULT  installComponent(IN INetCfg* pNetCfg_p,
                                 IN LPCTSTR pComponentId_p,
                                 IN const GUID* pGuidClass_p);
static HRESULT  setupComponent(LPTSTR pInfFile_p, LPTSTR pPnpID_p,
                               BOOL fInstall_p);
static void     releaseRef(IUnknown* pComponent_p);

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Main function of installation application

This is the main function for the driver installation application for
openPOWERLINK PCIe devices.

\param  argc_p         Argument count.
\param  aArgv_p        Argument list.

\return The function returns Windows system error code.
*/
//------------------------------------------------------------------------------
//driver entry point
int _tmain(int argc_p, _TCHAR* aArgv_p[])
{
    // Handle Driver Install
    if (_tcsicmp(aArgv_p[1], _T("/Install")) == 0)
    {
        SetConsoleTitle(_T("Installing openPOWERLINK NDIS intermediate driver"));
        return installDriver();
    }

    // Handle Driver Uninstall
    if (_tcsicmp(aArgv_p[1], _T("/Uninstall")) == 0)
    {
        SetConsoleTitle(_T("Uninstalling openPOWERLINK NDIS intermediate driver"));
        return uninstallDriver();
    }
}

//------------------------------------------------------------------------------
/**
\brief  Get the INF file path for the openPOWERLINK application

This functions creates the INF path name for the driver using the application
path.

\param  pInfFilename_p    Pointer to the buffer to receive the INF name.
\param  inSize_p          Size of the INF buffer.

\return The size of the INF buffer.
*/
//------------------------------------------------------------------------------
static UINT32 getInfFile(LPTSTR pInfFilename_p, UINT32 inSize_p)
{
    UINT32   result;
    TCHAR   aDrive[_MAX_DRIVE];
    TCHAR   aDirectory[_MAX_DIR];

    // Get the location of this installer
    result = GetModuleFileName(NULL, pInfFilename_p, inSize_p);

    if (result == 0)
        return 0;

    // Split the driver and directory name from the retrieved path
    _wsplitpath(pInfFilename_p, aDrive, aDirectory, NULL, NULL);

    // Create the path to the INF file.
    // The application expects the inf to be in the same directory as the application.
    _tmakepath(pInfFilename_p, aDrive, aDirectory, PLK_SERVICE_INF_FILE, _T(".inf"));

    return (UINT32)wcslen(pInfFilename_p);
}

//------------------------------------------------------------------------------
/**
\brief  Initialize the INETCFG COM interface to be used

This routine initializes the INETCFG COM interface and acquires the lock to
access the interface.

The initialized interface is used by the application to install the driver
using protocol class methods.

\param  fGetWriteLock_p     Flag to identify if lock in required.
\param  pAppName_p          Pointer to application name string.
\param  ppNetCfg_p          Double pointer to receive INetCfg pointer.
\param  ppLockedBy_p        Double pointer to receive lock configuration.

\return HRESULT error code.
*/
//------------------------------------------------------------------------------
static HRESULT getINetCfg(IN BOOL fGetWriteLock_p, IN LPCTSTR pAppName_p,
                   OUT INetCfg** ppNetCfg_p, OUT LPTSTR* ppLockedBy_p)
{
    INetCfg*        pNetCfg_p = NULL;
    INetCfgLock*    pNetCfgLock = NULL;
    HRESULT         hret = S_OK;

    *ppNetCfg_p = NULL;

    if (ppLockedBy_p)
    {
        *ppLockedBy_p = NULL;
    }

    hret = CoInitialize(NULL);

    if (hret != S_OK)
    {
        printf("Unable to initialize COM library (0x%X)\n", hret);
        return hret;
    }

    // Create NETCFG object
    hret = CoCreateInstance(CLSID_CNetCfg,
                              NULL, CLSCTX_INPROC_SERVER,
                              IID_INetCfg,
                              (void**)&pNetCfg_p);
    if (hret != S_OK)
    {
        printf("Unable to create class object for NETCFG (0x%X)\n", hret);
        CoUninitialize();
        return hret;
    }

    if (fGetWriteLock_p)
    {
        // Get the locking interface handle and acquire the lock
        hret = pNetCfg_p->QueryInterface(IID_INetCfgLock,
                                    (LPVOID *)&pNetCfgLock);
        if (hret != S_OK)
        {
            printf("Lock interface not present (0x%X)\n", hret);
            goto Exit;
        }

        hret = pNetCfgLock->AcquireWriteLock(LOCK_TIME_OUT,
                                             pAppName_p,
                                             ppLockedBy_p);
        if (hret == S_FALSE)
        {
            printf("Unable to acquire lock \n");
            hret = NETCFG_E_NO_WRITE_LOCK;
        }
    }

    if (hret == S_OK)
    {
        // Initialize the INETCFG interface
        hret = pNetCfg_p->Initialize(NULL);
        if (hret == S_OK)
        {
            *ppNetCfg_p = pNetCfg_p;
            pNetCfg_p->AddRef();
        }
        else
        {
            if (pNetCfgLock)
            {
                pNetCfgLock->ReleaseWriteLock();
            }
        }
    }

Exit:
    releaseRef(pNetCfgLock);
    releaseRef(pNetCfg_p);

    if (hret != S_OK)
    {
        CoUninitialize();
    }

    return hret;
}

//------------------------------------------------------------------------------
/**
\brief  Free the INETCFG COM interface

This routine frees the INETCFG COM interface and releases the lock acquired
during initialization.

\param  pNetCfg_p          Pointer to INetCfg interface object.
\param  fHasWriteLock_p    Flag to identify if lock was acquired.

\return HRESULT error code.
*/
//------------------------------------------------------------------------------
static HRESULT releaseINetCfg(IN INetCfg* pNetCfg_p, IN BOOL fHasWriteLock_p)
{
    INetCfgLock*    pNetCfgLock = NULL;
    HRESULT         hret = S_OK;

    hret = pNetCfg_p->Uninitialize();

    if (hret == S_OK && fHasWriteLock_p)
    {
        hret = pNetCfg_p->QueryInterface(IID_INetCfgLock,
                                 (LPVOID *)&pNetCfgLock);
        if (hret == S_OK)
        {
            hret = pNetCfgLock->ReleaseWriteLock();
            releaseRef(pNetCfgLock);
        }
    }

    releaseRef(pNetCfg_p);

    CoUninitialize();

    return hret;
}

//------------------------------------------------------------------------------
/**
\brief  Install a NET component

This function installs the specified net component using the INetCfgClassSetup
instance

\param  pNetCfg_p         Pointer to INetCfg interface object.
\param  pComponentId_p    Pointer to the component ID string.
\param  pGuidClass_p      Constant pointer to the class GUID of the component.
\param  pInfFullPath_p    Pointer to the INF path for the component.

\return HRESULT error code.
*/
//------------------------------------------------------------------------------
static HRESULT installNetComponent(IN INetCfg* pNetCfg_p, IN LPCTSTR pComponentId_p,
                                   IN const GUID* pGuidClass_p,
                                   IN LPCTSTR pInfFullPath_p)
{
    UINT32               error;
    HRESULT             hret = S_OK;
    TCHAR               aDrive[_MAX_DRIVE];
    TCHAR               aDirectory[_MAX_DIR];
    TCHAR               aDirWithDrive[_MAX_DRIVE + _MAX_DIR];
    INetCfgClassSetup*  pNetCfgClassSetup = NULL;
    INetCfgComponent*   pNCComponent = NULL;
    OBO_TOKEN           oboToken;

    // Copy the INF to system path
    if (pInfFullPath_p)
    {
        // Get the path where the INF file is.
        _tsplitpath(pInfFullPath_p, aDrive, aDirectory, NULL, NULL);
        _tcscpy(aDirWithDrive, aDrive);
        _tcscat_s(aDirWithDrive, aDirectory);

        // Copy INF file to system install directory
        if (!SetupCopyOEMInfW(pInfFullPath_p, aDirWithDrive,
                              SPOST_PATH,
                              0,
                              NULL,
                              0,
                              NULL,
                              NULL))
        {
            error = GetLastError();
            printf("SetupCopyOEMInfW failed with error (0x%x)\n", error);
            hret = HRESULT_FROM_WIN32(error);
            return hret;
        }
    }

    // Install the component
    ZeroMemory(&oboToken, sizeof(oboToken));
    oboToken.Type = OBO_USER;

    // Get the INetCfgClassSetup object instance
    hret = pNetCfg_p->QueryNetCfgClass(pGuidClass_p,
                                       IID_INetCfgClassSetup,
                                       (void**)&pNetCfgClassSetup);

    if (hret != S_OK)
    {
        printf("Failed to get INetCfgClassSetup instance (0x%X)\n", hret);
        return hret;
    }

    // Install the driver
    hret = pNetCfgClassSetup->Install(pComponentId_p,
                                      &oboToken,
                                      0,
                                      0,
                                      NULL,
                                      NULL,
                                      &pNCComponent);
    if (S_OK == hret)
    {

        releaseRef(pNCComponent);
    }

    releaseRef(pNetCfgClassSetup);

    hret = pNetCfg_p->Apply();

    return hret;
}

//------------------------------------------------------------------------------
/**
\brief Uninstall a NET component

This function uninstalls the specified NET component.

\param  pNetCfg_p          Pointer to INetCfg interface object.
\param  pComponentId_p     Pointer to component ID string.

\return HRESULT error code.
*/
//------------------------------------------------------------------------------
static HRESULT uninstallNetComponent(IN INetCfg* pNetCfg_p, IN LPCTSTR pComponentId_p)
{
    HRESULT             hret = S_OK;
    INetCfgClass*       pNetCfgClass = NULL;
    INetCfgClassSetup*  pNetCfgClassSetup = NULL;
    INetCfgComponent*   pNCComponent = NULL;
    OBO_TOKEN           oboToken;
    GUID                guidClass;

    if (pNetCfg_p == NULL)
        return S_FALSE;

    hret = pNetCfg_p->FindComponent(pComponentId_p, &pNCComponent);
    if (hret != S_OK)
    {
        wprintf(_T("Couldn't get an interface pointer to %s"), pComponentId_p);
        return hret;
    }

    hret = pNCComponent->GetClassGuid(&guidClass);
    if (hret != S_OK)
    {
        wprintf(_T("Couldn't get an class interface %s"), pComponentId_p);
        goto Exit;
    }

    // Uninstall the component
    ZeroMemory(&oboToken, sizeof(oboToken));
    oboToken.Type = OBO_USER;

    // Get the INetCfgClassSetup object instance
    hret = pNetCfg_p->QueryNetCfgClass(&guidClass,
                                       IID_INetCfgClass,
                                       (void**)&pNetCfgClass);
    if (hret != S_OK)
    {
        printf("Failed to get INetCfgClass instance (0x%X)\n", hret);
        goto Exit;
    }

    hret = pNetCfgClass->QueryInterface(IID_INetCfgClassSetup,
                                        (void**)&pNetCfgClassSetup);
    if (hret != S_OK)
    {
        printf("Failed to get INetCfgClassSetup instance (0x%X)\n", hret);
        goto Exit;
    }

    // Uninstall the driver
    hret = pNetCfgClassSetup->DeInstall(pNCComponent,
                                      &oboToken,
                                      NULL);
    if ((hret == S_OK) || (hret == NETCFG_S_REBOOT))
    {
        hret = pNetCfg_p->Apply();

        if ((hret != S_OK) && (hret != NETCFG_S_REBOOT))
        {
            wprintf(_T("Couldn't apply the changes after uninstalling %s\n"),
                    pComponentId_p);
        }
    }

Exit:
    releaseRef(pNetCfgClassSetup);
    releaseRef(pNetCfgClass);
    releaseRef(pNCComponent);

    return hret;
}
//------------------------------------------------------------------------------
/**
\brief  Release reference to the IUnknown instance

This is a utility routine to release references to the IUnknown components.

\param  pComponent_p       Pointer to IUnknown component.
*/
//------------------------------------------------------------------------------
static void releaseRef(IN IUnknown* pComponent_p)
{
    if (pComponent_p)
    {
        pComponent_p->Release();
    }
}

//------------------------------------------------------------------------------
/**
\brief Setup the specified NET component

This routine implements handling of installation and uninstallation of NET
components using the INetCfg COM interface.

\param  pInfFile_p      Pointer to INF filename for the component.
\param  pPnpID_p        Pointer to the PNP ID of the component.
\param  fInstall_p      Flag to identify installation or uninstallation.

\return HRESULT error code.
*/
//------------------------------------------------------------------------------
static HRESULT setupComponent(LPTSTR pInfFile_p, LPTSTR pPnpID_p, BOOL fInstall_p)
{
    INetCfg*   pNetCfg;
    LPTSTR     pAppName;
    HRESULT    hret;

    // Initialize the INetCfg object and retrieve the object pointer
    hret = getINetCfg(TRUE, APPNAME, &pNetCfg, &pAppName);

    if (hret != S_OK)
    {
        if ((hret == NETCFG_E_NO_WRITE_LOCK) && pAppName)
        {
            printf("Failed to initialize INetCfg interface\n");
            wprintf(_T("%s currently holds the lock, try later.\n"), pAppName);
            CoTaskMemFree(pAppName);
        }
        else
        {
            printf("Couldn't the get notify object interface %x\n", hret);
        }

        return hret;
    }

    if (fInstall_p)
    {
        // Install the network component.
        hret = installNetComponent(pNetCfg,
                                   pPnpID_p,
                                   &GUID_DEVCLASS_NETTRANS,
                                   pInfFile_p);

        if ((hret == S_OK) || (hret == NETCFG_S_REBOOT))
        {
            hret = pNetCfg->Apply();
        }
        else
        {
            if (hret != HRESULT_FROM_WIN32(ERROR_CANCELLED))
            {
                printf("Couldn't install the network component\n");
            }
        }
    }
    else
    {
        hret = uninstallNetComponent(pNetCfg,
                                     pPnpID_p);
    }

    releaseINetCfg(pNetCfg, TRUE);

    return hret;
}

//------------------------------------------------------------------------------
/**
\brief Install driver

This routine implements the installation steps for the driver.

First the INF file path is retrieved and then the NET component for the driver
is installed.

\return Returns Windows system error code.
\retval 0, if no error.
*/
//------------------------------------------------------------------------------
static UINT32 installDriver(void)
{
    UINT32   result;
    TCHAR   aInfFilePath[_MAX_PATH];
    HRESULT hret = S_OK;

    _tprintf(_T("Installing NDIS driver %s...\n"), DRIVER_NAME);

    // Get Path to Service INF File
    result = getInfFile(aInfFilePath, MAX_PATH);
    if (result == 0)
    {
        printf("Unable to get INF file path.\n");
        return 0;
    }

    wprintf(_T("INF Path :%s\n"), aInfFilePath);
    wprintf(_T("PnpID: %s\n"), PLK_SERVICE_PNP_DEVICE_ID);

    hret = setupComponent(aInfFilePath,
                                  PLK_SERVICE_PNP_DEVICE_ID,
                                  TRUE);
    if (hret != S_OK)
    {
        wprintf(_T("Failed to install driver for %s \n"), PLK_SERVICE_PNP_DEVICE_ID);
    }

    return 0;
}

//------------------------------------------------------------------------------
/**
\brief Uninstall driver

This routine implements the uninstallation steps for the driver.

\return Returns Windows system error code.
\retval 0, if no error.
*/
//------------------------------------------------------------------------------
static UINT32 uninstallDriver(void)
{
    HRESULT hret = S_OK;
    TCHAR   aInfFilePath[_MAX_PATH];

    _tprintf(_T("Uninstalling NDIS driver %s...\n"), DRIVER_NAME);
    wprintf(_T("PnpID: %s\n"), PLK_SERVICE_PNP_DEVICE_ID);

    hret = setupComponent(aInfFilePath,
                                  PLK_SERVICE_PNP_DEVICE_ID,
                                  FALSE);
    if (hret != S_OK)
    {
        wprintf(_T("Failed to uninstall driver for %s\n"), PLK_SERVICE_PNP_DEVICE_ID);
    }

    return 0;
}
