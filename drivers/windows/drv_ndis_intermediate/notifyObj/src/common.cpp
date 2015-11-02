/**
********************************************************************************
\file   common.cpp

\brief  Common function definitions for notify DLL

This file contains the definition of common routines used across the notify DLL
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

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <windows.h>
#include <stdio.h>
#include <netcfgn.h>
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

#ifndef NDEBUG
//-------------------------------------------------------------------------
/**
\brief Print debug trace to standard console using wide character format

\param  format         Format string.
\param  ...            Arguments to print.

*/
//-------------------------------------------------------------------------
void notify_debugTrace(_In_ LPWSTR format, ...)
{
    static WCHAR aTempBuf[4096];

    va_list arglist;

    va_start(arglist, format);

    StringCchVPrintfW(aTempBuf,
                      celems(aTempBuf),
                      format,
                      arglist);

    OutputDebugStringW(aTempBuf);

    va_end(arglist);
}
#endif

//-------------------------------------------------------------------------
/**
\brief Find InetCfgComponent instance

This routine searches for the InetCfgComponent using the GUID of the
component in the specified InetCfg link list and returns the instance back.

\param  pInetCfg_p           Pointer to InetCfg base to be searched.
\param  guidInstance_p       GUID of the component to be searched.
\param  ppInetCfgComp_p      Double pointer to the InetCfg component.

\return HRESULT error code.
\retval S_OK, if the instance was found.
\retval ppInetCfgComp_p holds the pointer to the component, if found.

*/
//-------------------------------------------------------------------------
HRESULT notify_findInstance(INetCfg* pInetCfg_p,
                            GUID& guidInstance_p,
                            INetCfgComponent** ppInetCfgComp_p)
{
    IEnumNetCfgComponent* pIEnumNetCfgComp;
    INetCfgComponent*     pInetCfgComp;
    GUID                  guid;
    WCHAR                 aGuid[MAX_PATH + 1];
    ULONG                 count;
    BOOL                  fFound;
    HRESULT               hret;

    TRACE(L"-->notify_findInstance.\n");

    hret = pInetCfg_p->EnumComponents(&GUID_DEVCLASS_NET,
                                      &pIEnumNetCfgComp);

    if (hret == S_OK)
    {
        StringFromGUID2(guidInstance_p, aGuid, MAX_PATH + 1);

        TRACE(L"  Looking for component with InstanceGuid %s\n",
              aGuid);

        hret = pIEnumNetCfgComp->Next(1, &pInetCfgComp, &count);

        for (fFound = FALSE; (hret == S_OK) && (fFound == FALSE);)
        {

            hret = pInetCfgComp->GetInstanceGuid(&guid);

            if (hret == S_OK)
            {

                StringFromGUID2(guid,
                                aGuid,
                                MAX_PATH + 1);

                TRACE(L"Found component with InstanceGuid %s\n",
                      aGuid);

                fFound = IsEqualGUID(guid,  guidInstance_p);

                if (fFound == FALSE)
                {

                    RELEASE_OBJ(pInetCfgComp);

                    hret = pIEnumNetCfgComp->Next(1, &pInetCfgComp, &count);
                }
                else
                {
                    *ppInetCfgComp_p = pInetCfgComp;
                }
            }
        }

        RELEASE_OBJ(pIEnumNetCfgComp);
    }
    else
    {

        TRACE(L"EnumComponents failed(HRESULT = %x).\n",
              hret);
    }

    TRACE(L"<--notify_findInstance(HRESULT = %x).\n",
          hret);

    return hret;
}
//-------------------------------------------------------------------------
/**
\brief Remove device prefix from the system path

This is a utility routine to remove the device prefix from the system path
of a miniport which is read from the registry.

\param  pStr_p           Pointer to string to be updated.

\return The routine returns the updated string without the device prefix.

*/
//-------------------------------------------------------------------------
LPWSTR notify_removeDevicePrefix(_In_ LPWSTR pStr_p)
{
    LPWSTR      pNewStr = NULL;
    LPWSTR      pTemp = wcsrchr(pStr_p, '\\');

    if (pTemp != NULL)
    {
        pNewStr = _wcsdup(pTemp + 1);
    }

    return pNewStr;
}

//-------------------------------------------------------------------------
/**
\brief Add device prefix to the system path

This is a utility routine to add the device prefix to the system path
of a physical adapter which is read from the registry.

The "Device" prefix is used by the Windows to identify the
protocols/components which are bound to the adapter.

\param  pStr_p           Pointer to string to be updated.

\return The routine returns the updated string with the device prefix.

*/
//-------------------------------------------------------------------------
LPWSTR notify_addDevicePrefix(_In_ LPWSTR pStr_p)
{
    LPWSTR      pNewStr;
    size_t      cchNewStr = wcslen(pStr_p) + wcslen(aDevicePrefix_g) + 1;

    pNewStr = (LPWSTR)malloc(cchNewStr * sizeof(WCHAR));
    if (pNewStr != NULL)
    {
        StringCchCopyW(pNewStr,
                       cchNewStr,
                       aDevicePrefix_g);
        pNewStr[cchNewStr - 1] = '\0';
        StringCchCatW(pNewStr,
                      cchNewStr,
                      pStr_p);
    }

    return pNewStr;
}
