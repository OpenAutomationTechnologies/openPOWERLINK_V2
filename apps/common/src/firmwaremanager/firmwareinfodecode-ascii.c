/**
********************************************************************************
 \file   firmwareinfodecode-ascii.c

\brief  Source file of the firmware info decode module for ASCII format

This module implements the decoding of the firmware informations stored as
ASCII information.

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
#include <firmwaremanager/firmwareinfodecode.h>
#include <firmwaremanager/firmwarestore.h>

#include <string.h>

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

#define FIRMWAREINFO_ASCII_LINE_SEPERATOR       "\n"
#define FIRMWAREINFO_LINE_FORMAT                "%x\t%x\t%x\t%x\t%x\t%x\t%x\t%255s"
#define FIRMWAREINFO_FW_IMAGE_PATH_LENGTH       256u
#define FIRMWAREINFO_NUMBER_OF_VALUES_PER_LINE  8

//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------

static tFirmwareRet parseLine(tFirmwareStoreHandle pFwStore_p, char* pLine_p,
                              tFirmwareInfo* pInfo_p);

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Decode the firmware information

This function decodes the information contained in the passed storage. The
resulting information will be stored in a list returned via \ref ppInfoList_p.
The memory required for the firmware informations is allocated and must be freed
by calling \ref firmwareinfodecode_freeInfo.

\param pStore_p [in]        Handle of the firmware store module accessing the
                            information storage.
\param ppInfoList_p [out]   Pointer to a list of firmware infos which will be
                            filled with the decoded information.

\return This functions returns a value of \ref tFirmwareRet.

\ingroup module_app_firmwaremanager
*/
//------------------------------------------------------------------------------
tFirmwareRet firmwareinfodecode_decodeInfo(tFirmwareStoreHandle pStore_p,
                                           tFirmwareInfoList* ppInfoList_p)
{
    tFirmwareRet            ret = kFwReturnOk;
    size_t                  dataSize;
    void*                   pData;
    char*                   pFileString;
    char*                   pLine;
    tFirmwareInfoList       pList = NULL;
    tFirmwareInfoEntry**    ppInsertIter = &pList;
    tFirmwareInfoEntry*     pEntry = NULL;

    if (ppInfoList_p == NULL)
    {
        ret = kFwReturnInvalidParameter;
        goto EXIT;
    }

    ret = firmwarestore_loadData(pStore_p);
    if (ret != kFwReturnOk)
    {
        goto EXIT;
    }

    ret = firmwarestore_getData(pStore_p, &pData, &dataSize);
    if (ret != kFwReturnOk)
    {
        goto EXIT;
    }

    pFileString = pData;

    pLine = strtok(pFileString, FIRMWAREINFO_ASCII_LINE_SEPERATOR);

    while (pLine != NULL)
    {
        pEntry = malloc(sizeof(tFirmwareInfoEntry));
        if (pEntry == NULL)
        {
            ret = kFwReturnNoResource;
            goto EXIT;
        }

        ret = parseLine(pStore_p, pLine, &pEntry->fwInfo);
        if (ret == kFwReturnOk)
        {
            if (*ppInsertIter != NULL)
            {
                (*ppInsertIter)->pNext = pEntry;
            }
            *ppInsertIter = pEntry;
            ppInsertIter = &pEntry->pNext;
        }
        else
        {
            free(pEntry);
        }

        pLine = strtok(NULL, FIRMWAREINFO_ASCII_LINE_SEPERATOR);
    }

    *ppInfoList_p = pList;

EXIT:
    (void)firmwarestore_flushData(pStore_p);

    if (ret != kFwReturnOk)
    {
        while (pList != NULL)
        {
            pEntry = pList;
            pList = pEntry->pNext;
            free(pEntry);
        }
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Free the decoded firmware information

This function frees the by \ref firmwareinfodecode_decodeInfo previously
allocated resources.

\param pInfoList_p [int]    List of firmware informations which shall be freed

\return This functions returns a value of \ref tFirmwareRet.

\ingroup module_app_firmwaremanager
*/
//------------------------------------------------------------------------------
tFirmwareRet firmwareinfodecode_freeInfo(tFirmwareInfoList pInfoList_p)
{
    tFirmwareRet        ret = kFwReturnOk;
    tFirmwareInfoEntry* rem;
    tFirmwareInfoList   pInfoList = pInfoList_p;

    if (pInfoList_p != NULL)
    {
        while (pInfoList != NULL)
        {
            rem = pInfoList;
            pInfoList = rem->pNext;
            free(rem);
        }
    }
    else
    {
        ret = kFwReturnInvalidParameter;
    }

    return ret;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief  Parse line of firmware info file

This function parses the given line of a firmware info file and returns the
firmware information.

\param pStore_p [in]        Handle of the firmware store module accessing the
                            information storage.
\param pLine_p [in]         Line to be parsed
\param pInfo_p [out]        Pointer used to return the firmware information

\return This functions returns a value of \ref tFirmwareRet.
*/
//------------------------------------------------------------------------------
static tFirmwareRet parseLine(tFirmwareStoreHandle pFwStore_p, char* pLine_p,
                              tFirmwareInfo* pInfo_p)
{
    tFirmwareRet            ret = kFwReturnOk;
    tFirmwareInfo           info;
    int                     result;
    tFirmwareStoreConfig    storeConfig;
    char                    imagePath[FIRMWAREINFO_FW_IMAGE_PATH_LENGTH];
    char                    completePath[FIRMWAREINFO_FW_IMAGE_PATH_LENGTH];
    void*                   baseInfo;

    memset(imagePath, 0, FIRMWAREINFO_FW_IMAGE_PATH_LENGTH);

    result = sscanf(pLine_p, FIRMWAREINFO_LINE_FORMAT, (unsigned int*)&info.moduleInfo.nodeId,
                    &info.moduleInfo.vendorId, &info.moduleInfo.productId,
                    &info.moduleInfo.hwVariant, &info.appSwDate,
                    &info.appSwTime, (unsigned int*)&info.fFirmwareLocked, imagePath);

    if (result != FIRMWAREINFO_NUMBER_OF_VALUES_PER_LINE)
    {
        goto EXIT;
    }

    memset(&storeConfig, 0, sizeof(tFirmwareStoreConfig));

    ret = firmwarestore_getBase(pFwStore_p, &baseInfo);
    if (ret != kFwReturnOk)
    {
        goto EXIT;
    }

    sprintf(completePath, "%s%c%s", (char*)baseInfo, FIRMWARESTORE_PATH_DIR_SEP, imagePath);

    storeConfig.pFilename = completePath;

    ret = firmwarestore_create(&storeConfig, &info.pFwImage);
    if (ret != kFwReturnOk)
    {
        goto EXIT;
    }

    memcpy(pInfo_p, &info, sizeof(tFirmwareInfo));

EXIT:
    return ret;
}

/// \}
