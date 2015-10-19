/**
********************************************************************************
\file   notifyObj.cpp

\brief  Implementation of DLL export routines for COM interface

This file contains the implementation for the mandatory methods for COM
interface for the notify object.

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
#include <netcfgn.h>
#include "stdafx.h"

#include "resource.h"
#include "notifyObj_i.h"
#include "dllmain.h"
#include "common.h"


//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// module global vars
//------------------------------------------------------------------------------
using namespace ATL;

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
\brief  DLL unload check

Used to determine whether the DLL can be unloaded by OLE.

\ingroup module_notify_ndisim
*/
//------------------------------------------------------------------------------
STDAPI DllCanUnloadNow(void)
{
    HRESULT hresult = E_FAIL;;

    TRACE(L"-->DllCanUnloadNow.\n");

    hresult = (_AtlModule.GetLockCount() == 0) ? S_OK : S_FALSE;

    TRACE(L"-->DllCanUnloadNow(HRESULT = %x).\n",
          hresult);

    return hresult;
}

//------------------------------------------------------------------------------
/**
\brief  Get method for class object

Returns a class factory to create an object of the requested type.

\ingroup module_notify_ndisim
*/
//------------------------------------------------------------------------------
STDAPI DllGetClassObject(_In_ REFCLSID rclsid, _In_ REFIID riid, _Outptr_ LPVOID* ppv)
{
    TRACE(L"DllGetClassObject.\n");
    return _AtlModule.DllGetClassObject(rclsid, riid, ppv);
}

//------------------------------------------------------------------------------
/**
\brief  Get method for class object

Adds entries to the system registry.

\ingroup module_notify_ndisim
*/
//------------------------------------------------------------------------------
STDAPI DllRegisterServer(void)
{
    HRESULT hresult = E_FAIL;;

    // registers object, typelib and all interfaces in typelib
    TRACE(L"DllRegisterServer.\n");

    hresult = _AtlModule.DllRegisterServer(TRUE);

    TRACE(L"DllRegisterServer (HRESULT = %x).\n", hresult);

    return hresult;
}

//------------------------------------------------------------------------------
/**
\brief  Get method for class object

Removes entries from the system registry.

\ingroup module_notify_ndisim
*/
//------------------------------------------------------------------------------
STDAPI DllUnregisterServer(void)
{
    HRESULT hresult = E_FAIL;;

    TRACE(L"DllUnregisterServer.\n");

    hresult = _AtlModule.DllUnregisterServer();

    TRACE(L"DllUnregisterServer (HRESULT = %x).\n", hresult);

    return hresult;
}

//------------------------------------------------------------------------------
/**
\brief  Get method for class object

Adds/Removes entries to the system registry per user per machine.

\ingroup module_notify_ndisim
*/
//------------------------------------------------------------------------------
STDAPI DllInstall(BOOL fInstall_p, _In_opt_  LPCWSTR pCmdLine_p)
{
    HRESULT                 hresult = E_FAIL;
    static const wchar_t    aUserSwitch[] = L"user";

    TRACE(L"DllInstall.\n");

    if (pCmdLine_p != NULL)
    {
        if (_wcsnicmp(pCmdLine_p, aUserSwitch, _countof(aUserSwitch)) == 0)
        {
            ATL::AtlSetPerUserRegistration(true);
        }
    }

    if (fInstall_p)
    {
        hresult = DllRegisterServer();
        if (FAILED(hresult))
        {
            DllUnregisterServer();
        }
    }
    else
    {
        hresult = DllUnregisterServer();
    }

    TRACE(L"DllInstall (HRESULT = %x).\n", hresult);

    return hresult;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

/// \}
