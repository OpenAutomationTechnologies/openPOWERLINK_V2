/**
********************************************************************************
\file   installapp-pcie.c

\brief  Installation application for Windows PCIe driver

This file contains the implementation for installing openPOWERLINK Windows
PCIe drivers. It uses the update driver APIs provided by Windows framework
to identify the openPOWERLINK PCIe device and install the Windows driver for it.

\ingroup install_app
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
#include <windows.h>

#include "installapp-pcie.h"

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
static INT rescanDevices(void);
static INT installApp(LPCTSTR pInf_p, LPCTSTR pHwid_p);

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Main function of installation application

This is the main function for the driver installation application for
openPOWERLINK PCIe devices.

\param  argc         Argument count.
\param  argv         Argument list.

*/
//------------------------------------------------------------------------------
void __cdecl main(ULONG argc, __in_ecount(argc) PCHAR argv[])
{
    INT        error;

    UNREFERENCED_PARAMETER(argc);
    UNREFERENCED_PARAMETER(argv);

    error = rescanDevices();
    if (error != 0)
    {
        TRACE("Error enumerating devices\n");
        return;
    }

    Sleep(2000);

    error = installApp((LPCTSTR)OPLK_INF_NAME, (LPCTSTR)OPLK_DEVICE_ID);

    if (error != 0)
    {
        TRACE("Error installing driver\n");
    }

}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief  Scan system for openPOWERLINK device

This function re-enumerates the device tree to check for openPOWERLINK device.

\returns Zero if scan successful else -1.
*/
//------------------------------------------------------------------------------
static INT rescanDevices(void)
{
    DEVINST devRoot;

    if (CM_Locate_DevNode_Ex(&devRoot, NULL, CM_LOCATE_DEVNODE_NORMAL, NULL) != CR_SUCCESS)
    {
        return -1;
    }

    if (CM_Reenumerate_DevNode_Ex(devRoot, 0, NULL) != CR_SUCCESS)
    {
        return -1;
    }

    return 0;
}

//------------------------------------------------------------------------------
/**
\brief  Install Windows driver

This function installs the specified Windows driver for the openPOWERLINK device.

\param  pInf_p         INF for the driver to install.
\param  pHwid_p        Hardware ID of the device.

\returns Zero if installed successfully else -1.
*/
//------------------------------------------------------------------------------
static INT installApp(LPCTSTR pInf_p, LPCTSTR pHwid_p)
{
    HMODULE             newdevMod = NULL;
    tUpdateDriver       pfnUpdateFunc;
    BOOL                reboot = TRUE;
    DWORD               flags = 0;
    DWORD               error;
    TCHAR               infPath[MAX_PATH];

    // Get full path for INF file specified
    error = GetFullPathNameA((LPCSTR)pInf_p, MAX_PATH, (LPSTR)infPath, NULL);
    if ((error >= MAX_PATH) || (error == 0))
    {
        // INF pathname too long
        TRACE("Cannot parse Inf name:%s\n", pInf_p);
        return -1;
    }

    if (GetFileAttributesA((LPCSTR)infPath) == (DWORD)(-1))
    {
        // INF file doesn't exist
        TRACE("Get File Attribute failed:%s with error 0x%x\n", infPath, GetLastError());
        return -1;
    }

    flags |= INSTALLFLAG_FORCE;

    // Use UpdateDriverForPlugAndPlayDevices
    newdevMod = LoadLibrary(TEXT("newdev.dll"));
    if (!newdevMod)
        goto Exit;

    // Get the address of the driver update routine from the newdev.dll library
    pfnUpdateFunc = (tUpdateDriver)GetProcAddress(newdevMod, UPDATEDRIVER_STRING);
    if (!pfnUpdateFunc)
    {
        TRACE("Unable to retrieve updateFunc \n");
        goto Exit;
    }

    if (!pfnUpdateFunc(NULL, pHwid_p, infPath, flags, &reboot))
    {
        TRACE("updateFunc failed with error 0x%x\n", GetLastError());
        goto Exit;
    }

Exit:

    if (newdevMod)
    {
        FreeLibrary(newdevMod);
    }

    return 0;
}

/// \}
