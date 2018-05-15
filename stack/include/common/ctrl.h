/**
********************************************************************************
\file   common/ctrl.h

\brief  Definitions for ctrl module

This file contains the definitions for the ctrl modules.

*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2016, B&R Industrial Automation GmbH
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
    kCtrlNone                   = 0x0000,   ///< No command
    kCtrlInitStack              = 0x0001,   ///< Initialize kernel modules
    kCtrlCleanupStack           = 0x0002,   ///< Cleanup kernel modules
    kCtrlShutdown               = 0x0003,   ///< Shutdown stack
    kCtrlGetVersionHigh         = 0x0004,   ///< Get higher part of kernel stack version
    kCtrlGetVersionLow          = 0x0005,   ///< Get lower part of kernel stack version
    kCtrlGetFeaturesHigh        = 0x0006,   ///< Get higher part of features of kernel stack
    kCtrlGetFeaturesLow         = 0x0007,   ///< Get lower part of features of kernel stack
    kCtrlWriteFileChunk         = 0x0008,   ///< Write file chunk to kernel stack
    kCtrlReconfigFactoryImage   = 0x0009,   ///< Reconfigure kernel stack with factory image
    kCtrlReconfigUpdateImage    = 0x000A,   ///< Reconfigure kernel stack with update image
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
    kCtrlStatusUnavailable      = 0x0000,   ///< Kernel stack is unavailable
    kCtrlStatusReady            = 0x0001,   ///< Kernel stack is ready
    kCtrlStatusRunning          = 0x0002,   ///< Kernel stack is running
    kCtrlStatusUnchanged        = 0x0003,   ///< State has not changed
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
    UINT8           aMacAddress[6];     ///< MAC address of the network interface
    char            aNetIfName[128];    ///< Device name of the network interface
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
