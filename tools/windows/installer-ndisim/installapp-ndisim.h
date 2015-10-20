/**
********************************************************************************
\file   installapp-ndisim.h

\brief  Installation application for Windows NDIS intermediate driver - Header

This file contains the definitions required for installation application.

The application is based on the ProtInstall application published at
http://ndis.com/ndis-general/ndisinstall/programinstall.htm by
Printing Communications Associates, Inc. (PCAUSA)

Please refer the readme.md file for additional license details

\ingroup install_app
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright(c) 2015, Kalycito Infotech Private Limited
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met :
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and / or other materials provided with the distribution.
    * Neither the name of the copyright holders nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED.IN NO EVENT SHALL COPYRIGHT HOLDERS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
------------------------------------------------------------------------------*/

#ifndef _INC_installapp_ndisim_H_
#define _INC_installapp_ndisim_H_

#include <stdio.h>
#include <stdlib.h>
#include <windows.h>
#include <wchar.h>
#include <netcfgx.h>
#include <netcfgn.h>
#include <setupapi.h>
#include <devguid.h>
#include <objbase.h>
#include <iostream>
#include <tchar.h>
#include <Windows.h>


// "Friendly" Name
#define PLK_FRIENDLY_NAME_A          "openPOWERLINK Intermediate Driver"
#define PLK_FRIENDLY_NAME_W          L"openPOWERLINK Intermediate Driver"

#ifdef UNICODE
#define DRIVER_NAME                       PLK_FRIENDLY_NAME_W
#else
#define DRIVER_NAME                       PLK_FRIENDLY_NAME_A
#endif

// Driver INF File and PnP ID Names
#define PLK_SERVICE_PNP_DEVICE_ID_A      "MS_OPLKP"
#define PLK_SERVICE_PNP_DEVICE_ID_W      L"MS_OPLKP"

#define PLK_SERVICE_INF_FILE_A           "oplk_ndisimp"
#define PLK_SERVICE_INF_FILE_W           L"oplk_ndisimp"

#ifdef UNICODE
#define PLK_SERVICE_PNP_DEVICE_ID        PLK_SERVICE_PNP_DEVICE_ID_W
#define PLK_SERVICE_INF_FILE             PLK_SERVICE_INF_FILE_W
#else
#define PLK_SERVICE_PNP_DEVICE_ID        PLK_SERVICE_PNP_DEVICE_ID_A
#define PLK_SERVICE_INF_FILE             PLK_SERVICE_INF_FILE_A
#endif

#pragma warning(disable: 4996)

#define APPNAME _T("NdisDriver")
#define LOCK_TIME_OUT     5000

#endif // _INC_installapp_ndisim_H_
