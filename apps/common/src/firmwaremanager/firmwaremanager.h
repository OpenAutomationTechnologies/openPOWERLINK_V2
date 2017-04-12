/**
********************************************************************************
\file   firmwaremanager.h

\brief  Header file for the firmware manager modules

This header file contains the general definitions for all firmware manager
modules.
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2017, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
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
#ifndef _INC_firmwaremanager_H_
#define _INC_firmwaremanager_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <oplk/oplk.h>

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// typedef
//------------------------------------------------------------------------------

/**
\brief Enum with return values used by the firmware manager modules
*/
typedef enum
{
    kFwReturnOk = 0,                ///< Function call was successfull
    kFwReturnInvalidParameter,      ///< An invalid parameter was passed
    kFwReturnInvalidInstance,       ///< An invalid instance was passed
    kFwReturnNoRessource,           ///< The allocation of required ressources failed
    kFwReturnFileOperationFailed,   ///< A File operation failed
    kFwReturnInfoFormatError,       ///< The supplied fw.info file in formated invalid
    kFwReturnModuleNotFound,        ///< The requested module was not found
    kFwReturnAlreadyInitialized,    ///< Firmware manager is already initialized
    kFwReturnInvalidNodeId,         ///< An invalid node ID was passed
    kFwReturnSdoWriteFailed,        ///< A SDO write command failed
    kFwReturnInvalidSdoSize,        ///< A SDO event returned an invalid number of transferred bytes
    kFwReturnInvalidSdoEvent,       ///< An invalid SDO finished event was passed
    kFwReturnNoIdent,               ///< Failed to get ident for node
    kFwReturnOdError,               ///< Failed to access local OD
    kFwReturnInterruptBoot,         ///< Signalize interruption of boot process
    kFwReturnSdoWriteError,         ///< Failed to issue an SDO write
    kFwReturnSdoReadError,          ///< Failed to issue an SDO read
    kFwReturnInvalidNodeInfo,       ///< An invalid node info was passed
    kFwReturnResetNodeFailed,       ///< Failed to send reset node
    kFwReturnInvalidCheckState,     ///< Invalid state in firmware check module
} tFirmwareRet;

//------------------------------------------------------------------------------
// function prototypes
//------------------------------------------------------------------------------

#ifdef __cplusplus
extern "C"
{
#endif

tFirmwareRet firmwaremanager_init(const char* fwInfoFileName_p);
void         firmwaremanager_exit(void);


tOplkError   firmwaremanager_processEvent(tOplkApiEventType eventType_p,
                                          const tOplkApiEventArg* pEventArg_p,
                                          void* pUserArg_p);

#ifdef __cplusplus
}
#endif

#endif /* _INC_firmwaremanager_H_ */
