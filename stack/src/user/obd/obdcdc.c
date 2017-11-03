/**
********************************************************************************
\file   obdcdc.c

\brief  Implementation of OBD CDC functions

This file contains the functions for parsing a Concise Device Configuration (CDC)
and write the configured data into the object dictionary.

\ingroup module_obd
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2017, B&R Industrial Automation GmbH
Copyright (c) 2013, SYSTEC electronic GmbH
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
#include <common/oplkinc.h>
#include <oplk/obdcdc.h>
#include <user/obdu.h>
#include <common/ami.h>
#include <user/eventu.h>

#if defined(CONFIG_INCLUDE_CFM)

#include <errno.h>

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
#if ((TARGET_SYSTEM == _NO_OS_) && (DEV_SYSTEM == _DEV_NIOS2_))
#define OBDCDC_DISABLE_FILE_SUPPORT     TRUE
#endif

#if ((TARGET_SYSTEM == _NO_OS_) && (DEV_SYSTEM == _DEV_ARM_ALTERA_EABI_))
#define OBDCDC_DISABLE_FILE_SUPPORT     TRUE
#endif

#if ((TARGET_SYSTEM == _NO_OS_) && ((DEV_SYSTEM == _DEV_MICROBLAZE_LITTLE_) || (DEV_SYSTEM == _DEV_MICROBLAZE_BIG_)))
#define OBDCDC_DISABLE_FILE_SUPPORT     TRUE
#endif

#ifndef OBDCDC_DISABLE_FILE_SUPPORT
#define OBDCDC_DISABLE_FILE_SUPPORT     FALSE
#endif

//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------
/**
\brief  Valid CDC types

This enumeration defines the supported CDC types
*/
typedef enum
{
    kObdCdcTypeFile      = 0,   ///< CDC provided as file
    kObdCdcTypeBuffer    = 1,   ///< CDC provided as memory buffer
} eObdCdcType;

/**
\brief OBD CDC type data type

Data type for the enumerator \ref eObdCdcType.
*/
typedef UINT32 tObdCdcType;

typedef struct
{
    tObdCdcType         type;
    union
    {
        FILE*           pFdCdcFile;
        const UINT8*    pNextBuffer;
    } handle;
    size_t              cdcSize;
    size_t              bufferSize;
    UINT8*              pCurBuffer;
} tObdCdcInfo;

typedef struct
{
    const void*         pCdcBuffer;
    size_t              cdcBufSize;
    const char*         pCdcFilename;
} tObdCdcInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tObdCdcInstance         cdcInstance_l;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static tOplkError processCdc(tObdCdcInfo* pCdcInfo_p);
static tOplkError loadNextBuffer(tObdCdcInfo* pCdcInfo_p, size_t bufferSize_p);
static tOplkError loadCdcBuffer(const void* pCdc_p, size_t cdcSize_p);
static tOplkError loadCdcFile(const char* pCdcFilename_p);

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Initialize OBD CDC module

The function initializes the OBD CDC module.

\return The function returns a tOplkError error code.

\ingroup module_obd
*/
//------------------------------------------------------------------------------
tOplkError obdcdc_init(void)
{
    OPLK_MEMSET(&cdcInstance_l, 0, sizeof(tObdCdcInstance));

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Exit OBD CDC module

The function exits and cleans up the OBD CDC module.

\ingroup module_obd
*/
//------------------------------------------------------------------------------
void obdcdc_exit(void)
{
    OPLK_MEMSET(&cdcInstance_l, 0, sizeof(tObdCdcInstance));
}

//------------------------------------------------------------------------------
/**
\brief  Set the CDC filename

The function sets the filename of the concise device configuration (CDC) file.

\param[in]      pCdcFilename_p      The filename of the CDC file to load.

\ingroup module_obd
*/
//------------------------------------------------------------------------------
void obdcdc_setFilename(const char* pCdcFilename_p)
{
    cdcInstance_l.pCdcFilename = pCdcFilename_p;
}

//------------------------------------------------------------------------------
/**
\brief  Set the CDC buffer

The function sets the buffer which contains the concise device description (CDC).

\param[in]      pCdc_p              Pointer to the buffer which contains the
                                    concise object definition.
\param[in]      cdcSize_p           Size of the buffer.

\ingroup module_obd
*/
//------------------------------------------------------------------------------
void obdcdc_setBuffer(const void* pCdc_p, size_t cdcSize_p)
{
    cdcInstance_l.pCdcBuffer = pCdc_p;
    cdcInstance_l.cdcBufSize = cdcSize_p;
}

//------------------------------------------------------------------------------
/**
\brief  Load Concise Device Configuration

The function loads the concise device configuration (CDC) and writes the
contents into the object dictionary. Depending on the CDC type either a CDC
file is parsed and loaded or a CDC buffer is parsed and loaded.

\return The function returns a tOplkError error code.

\ingroup module_obd
*/
//------------------------------------------------------------------------------
tOplkError obdcdc_loadCdc(void)
{
    tOplkError  ret;

    if (cdcInstance_l.pCdcBuffer != NULL)
        ret = loadCdcBuffer(cdcInstance_l.pCdcBuffer, cdcInstance_l.cdcBufSize);
    else if (cdcInstance_l.pCdcFilename != NULL)
        ret = loadCdcFile(cdcInstance_l.pCdcFilename);
    else
        ret = kErrorObdInvalidDcf;

    if (ret == kErrorReject)
        ret = kErrorOk;

    return ret;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief  Load Concise Device Configuration file

The function loads the concise device configuration (CDC) from the specified
file and writes its contents into the OD.

\param[in]      pCdcFilename_p      The filename of the CDC file to load.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError loadCdcFile(const char* pCdcFilename_p)
{
    tOplkError  ret = kErrorOk;
#if (OBDCDC_DISABLE_FILE_SUPPORT == FALSE)
    tObdCdcInfo cdcInfo;
    UINT32      error;

    OPLK_MEMSET(&cdcInfo, 0, sizeof(tObdCdcInfo));
    cdcInfo.type = kObdCdcTypeFile;
    cdcInfo.handle.pFdCdcFile = fopen(pCdcFilename_p, "rb");

    if (cdcInfo.handle.pFdCdcFile == NULL)
    {   // error occurred
        error = (UINT32)errno;
        DEBUG_LVL_OBD_TRACE("%s: failed to open '%s'\n", __func__, pCdcFilename_p);
        ret = eventu_postError(kEventSourceObdu, kErrorObdErrnoSet, sizeof(UINT32), &error);
        return ret;
    }

    // Get the file length
    fseek(cdcInfo.handle.pFdCdcFile, 0, SEEK_END);
    cdcInfo.cdcSize = (size_t)ftell(cdcInfo.handle.pFdCdcFile);
    fseek(cdcInfo.handle.pFdCdcFile, 0, SEEK_SET);

    ret = processCdc(&cdcInfo);

    if (cdcInfo.pCurBuffer != NULL)
    {
        OPLK_FREE(cdcInfo.pCurBuffer);
        cdcInfo.pCurBuffer = NULL;
        cdcInfo.bufferSize = 0;
    }

    fclose(cdcInfo.handle.pFdCdcFile);
#else
    UNUSED_PARAMETER(pCdcFilename_p);

    ret = kErrorNoResource;
#endif

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Load Concise Device Configuration Buffer

The function loads the concise device configuration (CDC) from the specified
buffer and writes the contents into the OD.

\param[in]      pCdc_p              Pointer to the buffer which contains the
                                    concise object definition.
\param[in]      cdcSize_p           Size of the buffer.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError loadCdcBuffer(const void* pCdc_p, size_t cdcSize_p)
{
    tOplkError  ret = kErrorOk;
    tObdCdcInfo cdcInfo;

    OPLK_MEMSET(&cdcInfo, 0, sizeof(tObdCdcInfo));
    cdcInfo.type = kObdCdcTypeBuffer;
    cdcInfo.handle.pNextBuffer = (const UINT8*)pCdc_p;
    if (cdcInfo.handle.pNextBuffer == NULL)
    {   // error occurred
        ret = eventu_postError(kEventSourceObdu, kErrorObdInvalidDcf, 0, NULL);
        goto Exit;
    }

    cdcInfo.cdcSize = cdcSize_p;
    cdcInfo.bufferSize = cdcInfo.cdcSize;

    ret = processCdc(&cdcInfo);

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Process Concise Device Configuration

The function processes the concise device configuration and writes it into the
OD.

\param[in,out]  pCdcInfo_p          Pointer to the CDC information.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError processCdc(tObdCdcInfo* pCdcInfo_p)
{
    tOplkError  ret;
    UINT32      entriesRemaining;
    UINT        objectIndex;
    UINT        objectSubIndex;
    size_t      curDataSize;

    ret = loadNextBuffer(pCdcInfo_p, sizeof(UINT32));
    if (ret != kErrorOk)
        return ret;

    entriesRemaining = ami_getUint32Le(pCdcInfo_p->pCurBuffer);
    if (entriesRemaining == 0)
    {
        ret = eventu_postError(kEventSourceObdu, kErrorObdNoConfigData, 0, NULL);
        return ret;
    }

    for (; entriesRemaining != 0; entriesRemaining--)
    {
        ret = loadNextBuffer(pCdcInfo_p, CDC_OFFSET_DATA);
        if (ret  != kErrorOk)
            return ret;

        objectIndex = ami_getUint16Le(&pCdcInfo_p->pCurBuffer[CDC_OFFSET_INDEX]);
        objectSubIndex = ami_getUint8Le(&pCdcInfo_p->pCurBuffer[CDC_OFFSET_SUBINDEX]);
        curDataSize = (size_t)ami_getUint32Le(&pCdcInfo_p->pCurBuffer[CDC_OFFSET_SIZE]);

        DEBUG_LVL_OBD_TRACE("%s: Reading object 0x%04X/%u with size %u from CDC\n",
                            __func__,
                            objectIndex,
                            objectSubIndex,
                            curDataSize);

        ret = loadNextBuffer(pCdcInfo_p, curDataSize);
        if (ret != kErrorOk)
        {
            DEBUG_LVL_OBD_TRACE("%s: Reading the corresponding data from CDC failed with 0x%02X\n", __func__, ret);
            return ret;
        }

        ret = obdu_writeEntryFromLe(objectIndex,
                                    objectSubIndex,
                                    pCdcInfo_p->pCurBuffer,
                                    (tObdSize)curDataSize);
        if (ret != kErrorOk)
        {
            tEventObdError  obdError;

            obdError.index = objectIndex;
            obdError.subIndex = objectSubIndex;

            DEBUG_LVL_OBD_TRACE("%s: Writing object 0x%04X/%u to local OBD failed with 0x%02X\n",
                                __func__,
                                objectIndex,
                                objectSubIndex,
                                ret);
            ret = eventu_postError(kEventSourceObdu, ret, sizeof(tEventObdError), &obdError);
            if (ret != kErrorOk)
                return ret;
        }
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Load next CDC buffer

The function loads the next buffer from the CDC

\param[in,out]  pCdcInfo_p          Pointer to the CDC information.
\param[in]      bufferSize_p        Size of the buffer.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError loadNextBuffer(tObdCdcInfo* pCdcInfo_p, size_t bufferSize_p)
{
    tOplkError  ret = kErrorOk;
#if (OBDCDC_DISABLE_FILE_SUPPORT == FALSE)
    size_t      count;
#endif

    switch (pCdcInfo_p->type)
    {
#if (OBDCDC_DISABLE_FILE_SUPPORT == FALSE)
        case kObdCdcTypeFile:
            if (pCdcInfo_p->bufferSize < bufferSize_p)
            {
                if (pCdcInfo_p->pCurBuffer != NULL)
                {
                    OPLK_FREE(pCdcInfo_p->pCurBuffer);
                    pCdcInfo_p->pCurBuffer = NULL;
                    pCdcInfo_p->bufferSize = 0;
                }

                pCdcInfo_p->pCurBuffer = (UINT8*)OPLK_MALLOC(bufferSize_p);
                if (pCdcInfo_p->pCurBuffer == NULL)
                {
                    ret = eventu_postError(kEventSourceObdu, kErrorObdOutOfMemory, 0, NULL);
                    if (ret != kErrorOk)
                        return ret;

                    return kErrorReject;
                }
                pCdcInfo_p->bufferSize = bufferSize_p;
            }

            count = fread(pCdcInfo_p->pCurBuffer, bufferSize_p, 1, pCdcInfo_p->handle.pFdCdcFile);
            if (ferror(pCdcInfo_p->handle.pFdCdcFile) || feof(pCdcInfo_p->handle.pFdCdcFile) || (count == 0))
            {
                ret = eventu_postError(kEventSourceObdu, kErrorObdInvalidDcf, 0, NULL);
                if (ret != kErrorOk)
                    return ret;

                return kErrorReject;
            }
            pCdcInfo_p->cdcSize -= bufferSize_p;
            break;
#endif
        case kObdCdcTypeBuffer:
            if (pCdcInfo_p->bufferSize < bufferSize_p)
            {
                ret = eventu_postError(kEventSourceObdu, kErrorObdInvalidDcf, 0, NULL);
                if (ret != kErrorOk)
                    return ret;

                return kErrorReject;
            }
            pCdcInfo_p->pCurBuffer = (UINT8*)pCdcInfo_p->handle.pNextBuffer;
            pCdcInfo_p->handle.pNextBuffer += bufferSize_p;
            pCdcInfo_p->bufferSize -= bufferSize_p;
            break;
    }

    return ret;
}

/// \}

#endif
