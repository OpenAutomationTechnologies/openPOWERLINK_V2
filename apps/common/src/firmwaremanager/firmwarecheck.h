/**
********************************************************************************
\file   firmwarecheck.h

\brief  Header file of the firmware check module

This header file contains the definitions of the firware check module.
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
#ifndef _INC_firmwarecheck_H_
#define _INC_firmwarecheck_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <oplk/oplk.h>
#include <firmwaremanager/firmwaremanager.h>
#include <firmwaremanager/firmwareinfo.h>

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

#define FIRMWARECHECK_INVALID_NODEID    C_ADR_INVALID
#define FIRMWARECHECK_MAX_NODEID        C_ADR_BROADCAST
#define FIRMWARECHECK_INVALID_SDO       ((UINT)-1)

#ifndef tabentries
#define tabentries(aVar_p)      (sizeof(aVar_p) / sizeof(*(aVar_p)))
#endif

//------------------------------------------------------------------------------
// typedef
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
/**
\brief  No firmware update required

This function is called if a firmware update is not required for a certain node.

\param nodeId_p [in]            ID of the already up-to-date node
\param pSdoConnection_p [in]    Pointer to SDO connection handle

\return This functions returns a value of \ref tFirmwareRet.
*/
//------------------------------------------------------------------------------
typedef tFirmwareRet (*tNoFirmwareCheckNodeCb)(UINT nodeId_p,
                                               tSdoComConHdl* pSdoConnection_p);

/**
\brief Firmware check configuration
*/
typedef struct
{
    tFirmwareInfoHandle     pFwInfo;                ///< Handle of the firmware store instance
                                                    ///< for accessing the firmware configuration
    tNoFirmwareCheckNodeCb  pfnNoUpdateRequired;    ///< No update required callback
} tFirmwareCheckConfig;

//------------------------------------------------------------------------------
// function prototypes
//------------------------------------------------------------------------------

#ifdef __cplusplus
extern "C"
{
#endif

tFirmwareRet    firmwarecheck_init(const tFirmwareCheckConfig* pConfig_p);
void            firmwarecheck_exit(void);
tFirmwareRet    firmwarecheck_processNodeEvent(UINT nodeId_p);
tFirmwareRet    firmwarecheck_processSdoEvent(const tSdoComFinished* pSdoComFinished_p);
tFirmwareRet    firmwarecheck_checkModulesOfNextNode(void);

#ifdef __cplusplus
}
#endif

#endif /* _INC_firmwareinfo_H_ */
