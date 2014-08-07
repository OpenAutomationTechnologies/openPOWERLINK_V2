/**
********************************************************************************
\file   common/driver.h

\brief  Header file for openPOWERLINK drivers

This file contains the necessary definitions for using the openPOWERLINK
kernel driver modules.
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2014, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
Copyright (c) 2013, SYSTEC electronic GmbH
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

#ifndef _INC_common_driver_H_
#define _INC_common_driver_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <common/oplkinc.h>
#include <common/dllcal.h>

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#define PLK_CLASS_NAME    "plk"
#define PLK_DEV_NAME      "plk" // used for "/dev" and "/proc" entry
#define PLK_DRV_NAME      "plk"
#define PLK_DEV_FILE      "/dev/plk"
#define PLK_IOC_MAGIC     '='

//------------------------------------------------------------------------------
// typedef
//------------------------------------------------------------------------------
typedef struct
{
    tDllCalQueue            queue;
    void*                   pData;
    size_t                  size;
} tIoctlDllCalAsync;

typedef struct
{
    void*                   pData;
    size_t                  size;
} tIoctlBufInfo;

typedef struct
{
    UINT32                  offset;
    UINT32                  errVal;
} tErrHndIoctl;

//------------------------------------------------------------------------------
// function prototypes
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// include architecture specific definitions
//------------------------------------------------------------------------------
#if (TARGET_SYSTEM == _LINUX_)
#include <common/driver-linux.h>
#endif

#endif /* _INC_common_driver_H_ */
