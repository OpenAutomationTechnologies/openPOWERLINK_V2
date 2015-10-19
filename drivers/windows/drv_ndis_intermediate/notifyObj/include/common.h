/**
********************************************************************************
\file   common.h

\brief  Common header for notify DLL

This file contains the common variables and routines used across the notify DLL
classes.

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

#ifndef _INC_common_H_
#define _INC_common_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <devguid.h>
#include <strsafe.h>

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#define celems(_x)          (sizeof(_x) / sizeof(_x[0]))
#define RELEASE_OBJ( x )  if ( x ) \
                            ((IUnknown*)(x))->Release();
//------------------------------------------------------------------------------
// typedef
//------------------------------------------------------------------------------

/**
\brief Configuration actions for miniport

This enumeration defines the configuration actions which the notify object will
handle for the miniports.

*/
typedef enum
{
    kActUnknown,            ///< Unknown action.
    kActInstall,            ///< Install the virtual Ethernet miniport.
    kActAdd,                ///< Add the virtual Ethernet to the registry.
    kActRemove,             ///< Remove and uninstall the virtual Ethernet.
    kActUpdate,             ///< Update virtual Ethernet configuration status in registry.
} eConfigAction;

/**
\brief Data type for configuration actions for miniport

Data type for the enumerator \ref eConfigAction.

*/
typedef UINT8 tConfigAction;

//------------------------------------------------------------------------------
// global defines
//------------------------------------------------------------------------------

// PnP ID, also referred to as Hardware ID, of the protocol interface.
const WCHAR aOplkProtocol_g[] = {L"ms_oplkp"};

// PnP ID, also referred to as Hardware ID, of the Miniport interface.
const WCHAR aOplkMiniport_g[] = {L"ms_oplkmp"};

// Name of the service as specified in the inf file in AddService directive.
const WCHAR aOplkService_g[] = {L"oplkp"};

// Path to the configure string where the virtual miniport instance names
// are stored.
const WCHAR aAdapterList_g[] =
{L"System\\CurrentControlSet\\Services\\oplkp\\Parameters\\Adapters"};

// Value name in the registry where miniport device id is stored.
const WCHAR aUpperBindings_g[] = {L"UpperBindings"};

const WCHAR aDevicePrefix_g[] = {L"\\Device\\"};

#ifndef NDEBUG

void notify_debugTrace(_In_ LPWSTR format, ...);
#define TRACE(...)     notify_debugTrace(__VA_ARGS__)

#else

#define TRACE(...)

#endif

HRESULT notify_findInstance(INetCfg* pInetCfg_p,
                            GUID &guidInstance_p,
                            INetCfgComponent** ppInetCfgComp_p);

LPWSTR notify_addDevicePrefix(_In_ LPWSTR pStr_p);
LPWSTR notify_removeDevicePrefix(_In_ LPWSTR pStr_p);

#endif // _INC_common_H_
