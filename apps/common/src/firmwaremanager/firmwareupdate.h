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
#include <firmwaremanager/firmwaremanager.h>
#include <firmwaremanager/firmwareinfodecode.h>

#include <oplk/oplk.h>

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// typedef
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
/**
\brief  Firmware update node callback

This function is called by the firmware update module to inform about a node's
update completion.

\param nodeId_p [in]            Node ID
\param pSdoConnection_p [in]    Pointer to SDO connection used for this node

\return This functions returns a value of \ref tFirmwareRet.
*/
//------------------------------------------------------------------------------
typedef tFirmwareRet (*tFirmwareUpdateNodeCb)(UINT nodeId_p,
                                              tSdoComConHdl* pSdoConnection_p);

/**
\brief Firmware update transmission status
*/
typedef struct
{
    UINT numberOfPendingTransmissions;  ///< Number of pending transmissions
    BOOL fTransmissionActive;           ///< Active transmission flag
} tFirmwareUpdateTransmissionStatus;

/**
\brief Firmware update configuration
*/
typedef struct
{
    tFirmwareUpdateNodeCb pfnNodeUpdateComplete;    ///< Node update complete callback
    tFirmwareUpdateNodeCb pfnModuleUpdateComplete;  ///< Modules of a node update complete callback
    tFirmwareUpdateNodeCb pfnError;                 ///< Node update error callback
} tFirmwareUpdateConfig;

/**
\brief Firmware update entry
*/
typedef struct tFirmwareUpdateEntry
{
    UINT                            nodeId;         ///< Node ID
    UINT                            index;          ///< Index of remote domain object
    UINT                            subindex;       ///< Subindex of remote domain object
    tFirmwareStoreHandle            pStoreHandle;   ///< Handle to firmware update file
    BOOL                            fIsNode;        ///< Flag for identification of node (Head) update
    struct tFirmwareUpdateEntry*    pNext;          ///< Pointer to next update entry
} tFirmwareUpdateEntry;

/**
\brief Firmware update list
*/
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
tFirmwareRet    firmwareupdate_processUpdateList(tFirmwareUpdateList* ppList_p);
tFirmwareRet    firmwareupdate_processSdoEvent(const tSdoComFinished* pSdoComFinished_p);
tFirmwareRet    firmwareupdate_getTransmissionStatus(UINT nodeId_p,
                                                     tFirmwareUpdateTransmissionStatus* pStatus_p);

#ifdef __cplusplus
}
#endif

#endif /* _INC_firmwareinfo_H_ */
