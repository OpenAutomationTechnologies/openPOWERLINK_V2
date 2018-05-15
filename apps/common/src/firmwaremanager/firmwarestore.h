/**
********************************************************************************
\file   firmwarestore.h

\brief  Header file of the firmware store module

This header file contains the definitions of the firware store module.
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
#ifndef _INC_firmwarestore_H_
#define _INC_firmwarestore_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <firmwaremanager/firmwaremanager.h>

#include <oplk/oplk.h>

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

#if defined(_WIN32)
#define FIRMWARESTORE_PATH_DIR_SEP '\\'
#else
#define FIRMWARESTORE_PATH_DIR_SEP '/'
#endif

//------------------------------------------------------------------------------
// typedef
//------------------------------------------------------------------------------

/**
 * \brief Handle to a firmware store instance
 */
typedef struct tFirmwareStoreInstance* tFirmwareStoreHandle;

/**
 * \brief Configuration structure for a firmware store instance
 */
typedef struct
{
    const char* pFilename;          ///< Filename of the accessed file.
                                    ///< This member is only used by the file
                                    ///< system based implementation.
    const void* pStorageBuffer;     ///< Pointer to the memory holding the
                                    ///< accessed information. This member is
                                    ///< only used by the non file system
                                    ///< implementation.
} tFirmwareStoreConfig;

//------------------------------------------------------------------------------
// function prototypes
//------------------------------------------------------------------------------

#ifdef __cplusplus
extern "C"
{
#endif

tFirmwareRet firmwarestore_create (const tFirmwareStoreConfig* pConfig_p,
                                   tFirmwareStoreHandle* pHandle_p);
tFirmwareRet firmwarestore_destroy (tFirmwareStoreHandle pHandle_p);
tFirmwareRet firmwarestore_loadData(tFirmwareStoreHandle pHandle_p);
tFirmwareRet firmwarestore_flushData(tFirmwareStoreHandle pHandle_p);
tFirmwareRet firmwarestore_getData(tFirmwareStoreHandle pHandle_p,
                                   void** ppData_p, size_t* pDataSize_p);
tFirmwareRet firmwarestore_getBase(tFirmwareStoreHandle pHandle_p,
                                   void** ppBase_p);

#ifdef __cplusplus
}
#endif

#endif /* _INC_firmwarestore_H_ */
