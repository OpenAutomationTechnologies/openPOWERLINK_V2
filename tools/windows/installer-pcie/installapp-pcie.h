/**
********************************************************************************
\file   installapp-pcie.h

\brief  Installation application for Windows driver - Header

This file contains the definitions and prototypes required for the installation
application to install a PCIe driver on Windows system.
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

#ifndef _INC_installapp_pcie_H_
#define _INC_installapp_pcie_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <cfgmgr32.h>
#include <stdio.h>
#include <tchar.h>
#include <string.h>
#include <newdev.h>

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#define OPLK_INF_NAME              "drv_ndis_pcie.inf"
// TODO: The device ID should be fetched from another configurable file or
//       as user input
#define OPLK_DEVICE_ID             "PCI\\VEN_1677&DEV_E53F"

#define UPDATEDRIVER_STRING         "UpdateDriverForPlugAndPlayDevicesA"

#ifndef NDEBUG
#define TRACE(...)    printf(__VA_ARGS__)
#else
#define TRACE(...)
#endif

//------------------------------------------------------------------------------
// typedef
//------------------------------------------------------------------------------
/**
\brief Function type for driver update routine

This function installs the updated drivers for devices that match the hardware ID
specified by the INF.

\param  hwndParent_p        A handle to the top-level window to use for any
                            UI related to installing devices.
\param  hardwareId_p        A pointer to a NULL-terminated string that
                            supplies the hardware identifier to match existing
                            devices on the computer.
\param  fullInfPath_p       A pointer to a NULL-terminated string that supplies
                            the full path file name of an INF file.
\param  installFlags_p      Installation flags.
\param  pRebootRequired_p   A pointer to a BOOL-typed variable that indicates
                            whether a restart is required and who should prompt
                            for it. This pointer is optional and can be NULL.

\return The function returns TRUE if a device was upgraded to the specified driver.
        Otherwise, it returns FALSE and the logged error can be retrieved with
        a call to GetLastError.
*/
typedef BOOL (WINAPI *tUpdateDriver)(__in HWND hwndParent_p,
                                     __in LPCTSTR hardwareId_p,
                                     __in LPCTSTR fullInfPath_p,
                                     __in DWORD installFlags_p,
                                     __out_opt PBOOL pRebootRequired_p
                                    );

//------------------------------------------------------------------------------
// function prototypes
//------------------------------------------------------------------------------

#ifdef __cplusplus
extern "C"
{
#endif

#ifdef __cplusplus
}
#endif

#endif /* _INC_installapp_pcie_H_ */
