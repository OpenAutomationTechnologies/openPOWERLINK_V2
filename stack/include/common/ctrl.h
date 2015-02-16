/**
********************************************************************************
\file   common/ctrl.h

\brief  Definitions for ctrl module

This file contains the definitions for the ctrl modules.

*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2014, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
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

#ifndef _INC_common_ctrl_H_
#define _INC_common_ctrl_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <common/oplkinc.h>

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// typedef
//------------------------------------------------------------------------------

/**
\brief Control commands

The following enumeration lists all valid ctrl commands.
*/
typedef enum
{
    kCtrlNone = 0,                      ///< No command
    kCtrlInitStack,                     ///< Initialize kernel modules
    kCtrlCleanupStack,                  ///< Cleanup kernel modules
    kCtrlShutdown,                      ///< Shutdown stack
    kCtrlGetVersionHigh,                ///< Get higher part of kernel stack version
    kCtrlGetVersionLow,                 ///< Get lower part of kernel stack version
    kCtrlGetFeaturesHigh,               ///< Get higher part of features of kernel stack
    kCtrlGetFeaturesLow,                ///< Get lower part of features of kernel stack
} eCtrlCmdType;

/**
\brief Control command data type

Data type for the enumerator \ref eCtrlCmdType.
*/
typedef UINT16 tCtrlCmdType;

/**
\brief Status of kernel stack

The following enumeration defines valid states of the kernel stack.
*/
typedef enum
{
    kCtrlStatusUnavailable = 0,         ///< Kernel stack is unavailable
    kCtrlStatusReady,                   ///< Kernel stack is ready
    kCtrlStatusRunning,                 ///< Kernel stack is running
    kCtrlStatusUnchanged,               ///< State has not changed
} eCtrlKernelStatus;

/**
\brief Status of kernel stack data type

Data type for the enumerator \ref eCtrlKernelStatus.
*/
typedef UINT16 tCtrlKernelStatus;

typedef struct
{
    UINT32      version;
    UINT32      featureFlags;
} tCtrlKernelInfo;

/**
\brief Init Parameters

The following structure defines the initialization parameters to be transfered
between user and kernel stack.
*/
typedef struct
{
    BYTE            aMacAddress[6];     ///< MAC address of the ethernet interface
    UINT            ethDevNumber;       ///< Device number of the ethernet interface
    char            szEthDevName[128];  ///< Device name of the ethernet interface
} tCtrlInitParam;

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

#endif /* _INC_common_ctrl_H_ */
