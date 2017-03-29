/**
********************************************************************************
 \file   firmwarestore-file.c

\brief  Source file of the firmware store module using a file system

This module implements an access to stored informations and firmware images
provided by the openCONFIGURATOR.

\ingroup module_app_firmwaremanager
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

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include "firmwarestore.h"
#include <stdio.h>

//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// module global vars
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// global function prototypes
//------------------------------------------------------------------------------

//============================================================================//
//            P R I V A T E   D E F I N I T I O N S                           //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

#define FWSTORE_READ_MODE "r"
#define FWSTORE_FILEPATH_LENGTH 128u

//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------

typedef struct tFirmwareStoreInstance
{
    char aFilename[FWSTORE_FILEPATH_LENGTH];
    char aPathToFile[FWSTORE_FILEPATH_LENGTH];
    FILE* pStorageFd;
    void* pData;
    size_t dataSize;
} tFirmwareStoreInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------

static tFirmwareRet allocAndReadData(FILE* pFile_p,
                                     void** ppBuffer_p, size_t* pDataSize_p);
static tFirmwareRet flushData(tFirmwareStoreHandle pHandle_p);
static void getPathToFile(const char* pFilename_p, char* aPath_p);

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Create a firmware store instance

This function creates an instance of the firmware store module.

\param pConfig_p [in] Pointer to the configuration structure for the created
                      instance.
\param ppHandle_p [out] Pointer to an instance handle which is filled if
                        creation was successful.

\return This functions returns a value of \ref tFirmwareRet.

\ingroup module_app_firmwaremanager
*/
//------------------------------------------------------------------------------
tFirmwareRet firmwarestore_create(const tFirmwareStoreConfig* pConfig_p,
                                  tFirmwareStoreHandle* ppHandle_p)
{
    tFirmwareRet ret = kFwReturnOk;
    tFirmwareStoreInstance* instance;

    if ((pConfig_p == NULL) || (ppHandle_p == NULL))
    {
        ret = kFwReturnInvalidParameter;
    }

    instance = malloc(sizeof(tFirmwareStoreInstance));
    if (instance == NULL)
    {
        ret = kFwReturnNoRessource;
        goto EXIT;
    }

    memset(instance, 0, sizeof(tFirmwareStoreInstance));
    strncpy(instance->aFilename, pConfig_p->pFilename, FWSTORE_FILEPATH_LENGTH);

    getPathToFile(instance->aFilename, instance->aPathToFile);

    *ppHandle_p = instance;

EXIT:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Destroy a firmware store instance

This function destroys an instance of the firmware store module.

\param pHandle_p [in] Handle of the firmware store module which shall be
                      destroyed.

\return This functions returns a value of \ref tFirmwareRet.

\ingroup module_app_firmwaremanager
*/
//------------------------------------------------------------------------------
tFirmwareRet firmwarestore_destroy (tFirmwareStoreHandle pHandle_p)
{
    tFirmwareRet ret = kFwReturnOk;

    if (pHandle_p == NULL)
    {
        ret = kFwReturnInvalidInstance;
    }

    ret = flushData(pHandle_p);

    if (ret == kFwReturnOk)
    {
        free (pHandle_p);
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Load data represented by the firmware store instance

This function acquires necessary resources and frees the data. All
acquired resources can be flushed manually by calling
\ref firmwarestore_flushData, unflushed resources will be freed within
\ref firmwarestore_destroy.

\param pHandle_p [in] Handle of the firmware store module

\return This functions returns a value of \ref tFirmwareRet.

\ingroup module_app_firmwaremanager
*/
//------------------------------------------------------------------------------
tFirmwareRet firmwarestore_loadData(tFirmwareStoreHandle pHandle_p)
{
    tFirmwareRet ret = kFwReturnOk;

    if (pHandle_p == NULL)
    {
        ret = kFwReturnInvalidInstance;
        goto EXIT;
    }

    pHandle_p->pStorageFd = fopen(pHandle_p->aFilename, FWSTORE_READ_MODE);
    if (pHandle_p->pStorageFd == NULL)
    {
        ret = kFwReturnFileOperationFailed;
        goto EXIT;
    }

    ret = allocAndReadData(pHandle_p->pStorageFd, &pHandle_p->pData, &pHandle_p->dataSize);

EXIT:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Flush data represented by the firmware store instance

This function frees necessary resources and frees the data.

\param pHandle_p [in] Handle of the firmware store module

\return This functions returns a value of \ref tFirmwareRet.

\ingroup module_app_firmwaremanager
*/
//------------------------------------------------------------------------------
tFirmwareRet firmwarestore_flushData(tFirmwareStoreHandle pHandle_p)
{
    tFirmwareRet ret = kFwReturnOk;

    if (pHandle_p != NULL)
    {
        ret = flushData(pHandle_p);
    }
    else
    {
        ret = kFwReturnInvalidInstance;
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Access the data provided by the firmware store instance

This function provides access to the data represented by the firmware store
module.

\param pHandle_p [in] Handle of the firmware store module
\param ppData_p [out] Pointer which will be filled with the data buffer
\param pDataSize_p [out] Pointer which will be filled with the available datasize

\return This functions returns a value of \ref tFirmwareRet.

\ingroup module_app_firmwaremanager
*/
//------------------------------------------------------------------------------
tFirmwareRet firmwarestore_getData(tFirmwareStoreHandle pHandle_p,
                                   void** ppData_p, size_t* pDataSize_p)
{
    tFirmwareRet ret = kFwReturnOk;

    if (pHandle_p == NULL)
    {
        ret = kFwReturnInvalidInstance;
        goto EXIT;
    }

    if ((ppData_p == NULL) || (pDataSize_p == NULL))
    {
        ret = kFwReturnInvalidParameter;
        goto EXIT;
    }

    if (pHandle_p->pData == NULL)
    {
        ret = allocAndReadData(pHandle_p->pStorageFd, &pHandle_p->pData, &pHandle_p->dataSize);
    }

    if (ret == kFwReturnOk)
    {
        *ppData_p = pHandle_p->pData;
        *pDataSize_p = pHandle_p->dataSize;
    }

EXIT:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Get the base of the represented storage

This function provides the base information of the storage, i.e. the base
address of a memory storage or the containing directory of a file.

\param pHandle_p [in] Handle of the firware store module
\param ppData_p [out] Pointer which will be filled with the base information

\return This functions returns a value of \ref tFirmwareRet.

\ingroup module_app_firmwaremanager
*/
//------------------------------------------------------------------------------
tFirmwareRet firmwarestore_getBase(tFirmwareStoreHandle pHandle_p,
                                   void** ppBase_p)
{
    tFirmwareRet ret = kFwReturnOk;

    if (pHandle_p == NULL)
    {
        ret = kFwReturnInvalidInstance;
        goto EXIT;
    }

    if (ppBase_p == NULL)
    {
        ret = kFwReturnInvalidParameter;
        goto EXIT;
    }

    *ppBase_p = pHandle_p->aPathToFile;

EXIT:
    return ret;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

static tFirmwareRet allocAndReadData(FILE* pFile_p,
                                     void** ppBuffer_p, size_t* pDataSize_p)
{
    tFirmwareRet ret = kFwReturnOk;
    UINT8* pBuffer = NULL;
    size_t fileSize;
    size_t readBytes;
    int result;
    long tellResult;

    result = fseek(pFile_p, 0, SEEK_END);
    if (result < 0)
    {
        ret = kFwReturnFileOperationFailed;
        goto EXIT;
    }

    tellResult = ftell(pFile_p);
    if (tellResult < 0)
    {
        ret = kFwReturnFileOperationFailed;
        goto EXIT;
    }

    fileSize = (size_t)tellResult;

    pBuffer = malloc(fileSize);
    if (pBuffer == NULL)
    {
        ret = kFwReturnNoRessource;
        goto EXIT;
    }

    rewind(pFile_p);

    readBytes = fread(pBuffer, 1u, fileSize, pFile_p);
    if (readBytes != fileSize)
    {
        ret = kFwReturnFileOperationFailed;
    }

    *ppBuffer_p = pBuffer;
    pBuffer = NULL;
    *pDataSize_p = fileSize;

EXIT:
    free(pBuffer);
    rewind(pFile_p);
    return ret;
}

static tFirmwareRet flushData(tFirmwareStoreHandle pHandle_p)
{
    tFirmwareRet ret = kFwReturnOk;
    int result;

    if (pHandle_p->pStorageFd != NULL)
    {
        result = fclose(pHandle_p->pStorageFd);

        if (result == 0)
        {
            pHandle_p->pStorageFd = NULL;
        }
        else
        {
            ret = kFwReturnFileOperationFailed;
        }
    }

    free(pHandle_p->pData);
    pHandle_p->pData = NULL;

    return ret;
}

static void getPathToFile(const char* pFilename_p, char* aPath_p)
{
    char* pPos;

    pPos = strrchr(pFilename_p, FIRMWARESTORE_PATH_DIR_SEP);

    memset(aPath_p, 0, FWSTORE_FILEPATH_LENGTH);

    if (pPos != NULL)
    {
        memcpy(aPath_p, pFilename_p, (pPos - pFilename_p));
    }
    else
    {
        aPath_p[0] = '.';
    }
}

/// \}
