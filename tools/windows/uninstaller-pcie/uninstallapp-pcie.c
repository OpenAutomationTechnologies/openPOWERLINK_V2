/**
********************************************************************************
\file   uninstallapp-pcie.c

\brief  Uninstaller application for Windows PCIe driver

This file contains the implementation of the uninstaller application for openPOWERLINK
Windows drivers. It uses the setupOEM APIs provided by Windows to uninstall the
driver.

\ingroup uninstall_app
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
#include <DriverSpecs.h>
__user_code

#include <windows.h>
#pragma warning(disable:4201) // nameless struct/union
#pragma warning(default:4201)

#include "uninstallapp-pcie.h"

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
static INT findDriverPackage(void);
static INT deleteDriverPackage(LPCTSTR lpInf);

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Main function of uninstallation application

This is the main function for the driver uninstallation application for
openPOWERLINK PCIe devices.

\param  argc         Argument count.
\param  argv         Argument list.

*/
//------------------------------------------------------------------------------
VOID __cdecl main(ULONG argc, __in_ecount(argc) PCHAR argv[])
{
    SP_REMOVEDEVICE_PARAMS         rmdParams;
    SP_DEVINSTALL_PARAMS           devParams;
    DWORD                          devIndex;
    HDEVINFO                       devSet = INVALID_HANDLE_VALUE;
    TCHAR                          devID[MAX_DEVICE_ID_LEN];
    SP_DEVINFO_LIST_DETAIL_DATA*   devInfoListDetail = NULL;
    SP_DEVINFO_DATA                devInfo;
    static BOOL                    reboot = FALSE;
    BOOL                           fDevFound = FALSE;

    UNREFERENCED_PARAMETER(argc);
    UNREFERENCED_PARAMETER(argv);

    // Get the list of all installed devices present in system
    devSet = SetupDiGetClassDevsEx(NULL, NULL, NULL,
                                   DIGCF_ALLCLASSES | DIGCF_PRESENT,
                                   NULL, NULL, NULL);

    if (devSet == INVALID_HANDLE_VALUE)
    {
        TRACE("SetupDiGetClassDevsEx failed with error :%d", GetLastError());
    }

    devInfo.cbSize = sizeof(SP_DEVINFO_DATA);

    // Parse the device list and remove the driver for the device
    for (devIndex = 0; SetupDiEnumDeviceInfo(devSet, devIndex, &devInfo); devIndex++)
    {
        INT     deviceInstanceLen;

        // Get ID for the device
        CM_Get_Device_ID_Ex(devInfo.DevInst, devID, MAX_DEVICE_ID_LEN, 0, NULL);

        deviceInstanceLen = strlen(OPLK_DEVICE_ID);
        if (strlen(devID) >= deviceInstanceLen)
        {
            if (strncmp(devID, OPLK_DEVICE_ID, deviceInstanceLen) == 0)
            {
                // POWERLINK device found
                TRACE("POWERLINK Device found %s\n", devID);
                fDevFound = TRUE;
                break;
            }
        }
    }

    if (!fDevFound)
    {
        TRACE("Device Not found\n");
        return;
    }
    // Setup remove parameters
    rmdParams.ClassInstallHeader.cbSize = sizeof(SP_CLASSINSTALL_HEADER);
    rmdParams.ClassInstallHeader.InstallFunction = DIF_REMOVE;
    rmdParams.Scope = DI_REMOVEDEVICE_GLOBAL;
    rmdParams.HwProfile = 0;

    if (!SetupDiSetClassInstallParams(devSet, &devInfo, &rmdParams.ClassInstallHeader, sizeof(rmdParams)) ||
        !SetupDiCallClassInstaller(DIF_REMOVE, devSet, &devInfo))
    {
        // failed to invoke DIF_REMOVE
        TRACE("\nSetupDiCallClassInstaller returns : %d", GetLastError());
    }
    else
    {
        // see if device needs reboot
        devParams.cbSize = sizeof(devParams);
        if (SetupDiGetDeviceInstallParams(devSet, &devInfo, &devParams) && (devParams.Flags & (DI_NEEDRESTART | DI_NEEDREBOOT)))
        {
            // Reboot required
            reboot = TRUE;
        }
        else
        {
            // Device removed successfully
            reboot = FALSE;
        }
    }

    // Find the driver package and delete it from the system
    findDriverPackage();

    if (reboot)
        TRACE("Reboot system to complete uninstallation\n");
    else
        TRACE("Device Removed Successfully\n");

    return;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief  Delete driver package

This function deletes the driver package for the uninstalled driver.

\param  pInf_p         INF for the driver to uninstall.

\returns Zero if removed successfully else -1.
*/
//------------------------------------------------------------------------------
static INT deleteDriverPackage(LPCTSTR pInf_p)
{
    DWORD                        error;
    TCHAR                        infFileName[MAX_PATH];
    PTSTR                        filePart = NULL;
    HMODULE                      setupapiMod = NULL;
    tSetupUninstallOEMInfo       pfnSetupUninstallOEMInfo;

    if (pInf_p == NULL)
    {
        TRACE("No INF file specified\n");
        return -1;
    }

    error = GetFullPathName(pInf_p, ARRAYSIZE(infFileName),
                            infFileName, &filePart);
    if ((!error) || (!filePart))
    {
        error = -1;
        goto Exit;
    }

    setupapiMod = LoadLibrary(TEXT("setupapi.dll"));
    if (!setupapiMod)
    {
        TRACE("Unable to load setupapi.dll\n");
        error = -1;
        goto Exit;
    }

    // Get the SetupUninstallOEMInfo address from the setupapi.dll.
    pfnSetupUninstallOEMInfo = (tSetupUninstallOEMInfo)GetProcAddress(setupapiMod, SETUPUNINSTALL_STRING);
    if (!pfnSetupUninstallOEMInfo)
    {
        TRACE("Unable to retrieve unistallProto routine\n");
        error = -1;
        goto Exit;
    }

    if (!pfnSetupUninstallOEMInfo(filePart, OPLK_FLAG_FORCE, NULL))
    {
        error = GetLastError();
        if (error == ERROR_INF_IN_USE_BY_DEVICES)
        {
            TRACE("INF still in use\n");
        }
        else
        {
            if (error == ERROR_NOT_AN_INSTALLED_OEM_INF)
                TRACE("INF not found\n");
        }

        goto Exit;
    }

    error = 0;

Exit:
    if (setupapiMod)
    {
        FreeLibrary(setupapiMod);
    }

    return error;
}

//------------------------------------------------------------------------------
/**
\brief  Find driver package to delete

This function searches the system inf repository for the Windows driver to be
uninstalled and deletes the package to avoid future references.

\returns Zero if found and removed successfully else -1.
*/
//------------------------------------------------------------------------------
static INT findDriverPackage(void)
{
    INT                error = 0;
    TCHAR              winInfDir[MAX_PATH];
    HANDLE             winDirHandle = INVALID_HANDLE_VALUE;
    WIN32_FIND_DATA    infPackage;
    HINF               infHandle = INVALID_HANDLE_VALUE;
    TCHAR              infData[MAX_INF_STRING_LENGTH];
    TCHAR              pInf[MAX_INF_STRING_LENGTH];
    UINT               errorLine;
    INFCONTEXT         context;
    BOOL               found;

    // Get The Windows Directory
    if (!GetWindowsDirectory(winInfDir, ARRAYSIZE(winInfDir)) ||
        FAILED(StringCchCat(winInfDir, ARRAYSIZE(winInfDir), TEXT("\\INF\\OEM*.INF"))))
    {
        error = -1;
        goto Exit;
    }

    // Get the first package
    winDirHandle = FindFirstFile(winInfDir, &infPackage);
    if (winDirHandle == INVALID_HANDLE_VALUE)
    {
        // No OEM driver packages on this machine.
        error = -1;
        goto Exit;
    }

    // Go through all the files and find OPLK Inf
    do
    {
        infHandle = SetupOpenInfFile(infPackage.cFileName, NULL,
                                     INF_STYLE_WIN4, &errorLine);
        if (infHandle == INVALID_HANDLE_VALUE)
        {
            TRACE("Error in SetupOpenInfFile Err %d\n", GetLastError());
            error = -1;
            goto Exit;
        }

        if (SetupFindFirstLine(infHandle, INFSTR_SECT_VERSION, INFSTR_KEY_PROVIDER, &context) &&
            (SetupGetStringField(&context, 1, infData, sizeof(infData), NULL)))
        {
            if (strcmp(infData, OPLK_PROVIDER_STRING) == 0)
            {
                found = TRUE;
                strcpy(pInf, infPackage.cFileName);
                break;
            }
        }
        else
        {
            TRACE("Error in SetupGetStringField\n");
            error = -1;
            goto Exit;
        }
    } while (FindNextFile(winDirHandle, &infPackage));

    if (!found)
    {
        TRACE("Unable to find specified driver\n");
        error = -1;
        goto Exit;
    }

    FindClose(infHandle);
    // delete the Package
    deleteDriverPackage(pInf);

    error = 0;

Exit:
    return error;
}

/// \}
