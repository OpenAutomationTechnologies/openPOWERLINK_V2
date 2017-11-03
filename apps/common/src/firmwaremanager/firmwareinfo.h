/**
********************************************************************************
\file   firmwareinfo.h

\brief  Header file of the firmware info module

This header file contains the definitions of the firware info module.
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2017, B&R Industrial Automation GmbH
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
#ifndef _INC_firmwareinfo_H_
#define _INC_firmwareinfo_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <firmwaremanager/firmwaremanager.h>
#include <firmwaremanager/firmwareinfodecode.h>
#include <firmwaremanager/firmwarestore.h>

#include <oplk/oplk.h>

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// typedef
//------------------------------------------------------------------------------

/**
 * \brief Handle to a firmware info instance
 */
typedef struct tFirmwareInfoInstance* tFirmwareInfoHandle;

/**
 * \brief Configuration structure for a firmware info instance
 */
typedef struct
{
    tFirmwareStoreHandle pFwStore; ///< Handle of the firmware store instance
                                   ///< for accessing the firmware configuration
} tFirmwareInfoConfig;

//------------------------------------------------------------------------------
// function prototypes
//------------------------------------------------------------------------------

#ifdef __cplusplus
extern "C"
{
#endif

tFirmwareRet firmwareinfo_create(const tFirmwareInfoConfig* pConfig_p,
                                 tFirmwareInfoHandle* ppHandle_p);
tFirmwareRet firmwareinfo_destroy(tFirmwareInfoHandle pHandle_p);
tFirmwareRet firmwareinfo_getInfoForNode(tFirmwareInfoHandle pHandle_p,
                                         const tFirmwareModuleInfo * pModuleInfo_p,
                                         tFirmwareInfo** ppFwInfo_p);

#ifdef __cplusplus
}
#endif

#endif /* _INC_firmwareinfo_H_ */
