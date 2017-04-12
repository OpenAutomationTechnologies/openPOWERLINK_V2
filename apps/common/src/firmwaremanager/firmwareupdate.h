/**
********************************************************************************
\file   firmwareupdate.h

\brief  Header file of the firmware update module

This header file contains the definitions of the firware update module.
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
#ifndef _INC_firmwareupdate_H_
#define _INC_firmwareupdate_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <oplk/oplk.h>
#include <firmwaremanager/firmwaremanager.h>
#include <firmwaremanager/firmwareinfodecode.h>

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// typedef
//------------------------------------------------------------------------------

typedef tFirmwareRet (*tFirmwareUpdateNodeCb)(UINT nodeId_p,
                                              tSdoComConHdl* pSdoConnection_p);

typedef struct
{
    tFirmwareUpdateNodeCb pfnUpdateComplete;
    tFirmwareUpdateNodeCb pfnError;
} tFirmwareUpdateConfig;

typedef struct tFirmwareUpdateEntry
{
    UINT                            nodeId;         ///< Node ID
    UINT                            index;          ///< Index of remote domain object
    UINT                            subindex;       ///< Subindex of remote domain object
    tFirmwareStoreHandle            pStoreHandle;   ///< Handle to firmware update file
    struct tFirmwareUpdateEntry*    pNext;          ///< Pointer to next update entry
} tFirmwareUpdateEntry;

typedef tFirmwareUpdateEntry* tFirmwareUpdateList;

//------------------------------------------------------------------------------
// function prototypes
//------------------------------------------------------------------------------

#ifdef __cplusplus
extern "C"
{
#endif

tFirmwareRet    firmwareupdate_init(const tFirmwareUpdateConfig* pConfig_p);
void            firmwareupdate_exit(void);
tFirmwareRet    firmwareupdate_processUpdateList(tFirmwareUpdateList pList_p);
tFirmwareRet    firmwareupdate_processSdoEvent(const tSdoComFinished* pSdoComFinished_p);

#ifdef __cplusplus
}
#endif

#endif /* _INC_firmwareinfo_H_ */
