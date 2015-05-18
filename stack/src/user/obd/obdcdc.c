/**
********************************************************************************
\file   obdcdc.c

\brief  Implementation of OBD CDC functions

This file contains the functions for parsing a Concise Device Configuration (CDC)
and write the configured data into the object dictionary.

\ingroup module_obd
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2014, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
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
#define _CRT_NONSTDC_NO_WARNINGS    // for MSVC 2005 or higher

#include <common/oplkinc.h>
#include <oplk/obd.h>
#include <oplk/obdcdc.h>
#include <common/ami.h>
#include <user/eventu.h>

#if (CONFIG_OBD_USE_LOAD_CONCISEDCF != FALSE)

#include <sys/stat.h>
#include <assert.h>
#include <stdlib.h>
#include <fcntl.h>
#include <errno.h>

#if (TARGET_SYSTEM == _WIN32_)

    #include <io.h>
    #include <sys/types.h>
    #include <sys/utime.h>
    #include <sys/timeb.h>
    #include <time.h>
    #include <direct.h>
    #include <string.h>

#elif (TARGET_SYSTEM == _LINUX_)

    #include <unistd.h>
    #include <sys/types.h>
    #include <sys/timeb.h>
    #include <utime.h>
    #include <limits.h>

#elif (TARGET_SYSTEM == _VXWORKS_)
        #include "ioLib.h"
#elif (TARGET_SYSTEM == _NO_OS_)
    #include <unistd.h>
#endif

#if ((TARGET_SYSTEM == _NO_OS_) && (DEV_SYSTEM == _DEV_ARM_ALTERA_EABI_))
    #include <sys/unistd.h>
#endif

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
#if (TARGET_SYSTEM == _WIN32_)

    #define flush  _commit
    #define mode_t int

#elif (TARGET_SYSTEM == _LINUX_)

    #define O_BINARY 0
    #define _MAX_PATH PATH_MAX
    #define flush  fsync

#elif (TARGET_SYSTEM == _VXWORKS_)

    #define O_BINARY 0

#endif

#if (TARGET_SYSTEM == _NO_OS_ && DEV_SYSTEM == _DEV_NIOS2_)
#define OBDCDC_DISABLE_FILE_SUPPORT     TRUE
#endif

#if ((TARGET_SYSTEM == _NO_OS_) && (DEV_SYSTEM == _DEV_ARM_ALTERA_EABI_))
#define OBDCDC_DISABLE_FILE_SUPPORT     TRUE
#endif

#if (TARGET_SYSTEM == _NO_OS_ && (DEV_SYSTEM == _DEV_MICROBLAZE_LITTLE_ || DEV_SYSTEM == _DEV_MICROBLAZE_BIG_))
#define OBDCDC_DISABLE_FILE_SUPPORT     TRUE
#endif

#ifndef FD_TYPE
#define FD_TYPE     int
#endif

#ifndef IS_FD_VALID
#define IS_FD_VALID(iFd_p)  ((iFd_p) >= 0)
#endif

#ifndef OBDCDC_DISABLE_FILE_SUPPORT
#define OBDCDC_DISABLE_FILE_SUPPORT     FALSE
#endif

//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------
typedef enum
{
    kObdCdcTypeFile      = 0,
    kObdCdcTypeBuffer    = 1,
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
        FD_TYPE         fdCdcFile;
        UINT8*          pNextBuffer;
    } handle;
    size_t              cdcSize;
    size_t              bufferSize;
    BYTE*               pCurBuffer;
} tObdCdcInfo;

typedef struct
{
    UINT8*              pCdcBuffer;
    unsigned int        cdcBufSize;
    char*               pCdcFilename;
} tObdCdcInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tObdCdcInstance         cdcInstance_l;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static tOplkError processCdc(tObdCdcInfo* pCdcInfo_p);
static tOplkError loadNextBuffer(tObdCdcInfo* pCdcInfo_p, size_t bufferSize);
static tOplkError loadCdcBuffer(UINT8* pCdc_p, size_t cdcSize_p);
static tOplkError loadCdcFile(char* pCdcFilename_p);

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
    cdcInstance_l.pCdcFilename = NULL;
    cdcInstance_l.pCdcBuffer = NULL;
    cdcInstance_l.cdcBufSize = 0;
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
    cdcInstance_l.pCdcFilename = NULL;
    cdcInstance_l.pCdcBuffer = NULL;
    cdcInstance_l.cdcBufSize = 0;
}

//------------------------------------------------------------------------------
/**
\brief  Set the CDC filename

The function sets the filename of the concise device configuration (CDC) file.

\param  pCdcFilename_p      The filename of the CDC file to load.

\ingroup module_obd
*/
//------------------------------------------------------------------------------
void obdcdc_setFilename(char* pCdcFilename_p)
{
    cdcInstance_l.pCdcFilename = pCdcFilename_p;
}

//------------------------------------------------------------------------------
/**
\brief  Set the CDC buffer

The function sets the buffer which contains the concise device description (CDC).

\param  pCdc_p          Pointer to the buffer which contains the concise object
                        definition.
\param  cdcSize_p       Size of the buffer.

\ingroup module_obd
*/
//------------------------------------------------------------------------------
void obdcdc_setBuffer(UINT8* pCdc_p, size_t cdcSize_p)
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
    tOplkError          ret;

    if (cdcInstance_l.pCdcBuffer != NULL)
    {
        ret = loadCdcBuffer(cdcInstance_l.pCdcBuffer, cdcInstance_l.cdcBufSize);
    }
    else if (cdcInstance_l.pCdcFilename != NULL)
    {
        ret = loadCdcFile(cdcInstance_l.pCdcFilename);
    }
    else
    {
        ret = kErrorObdInvalidDcf;
    }

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

\param  pCdcFilename_p  The filename of the CDC file to load.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError loadCdcFile(char* pCdcFilename_p)
{
    tOplkError      ret = kErrorOk;
#if OBDCDC_DISABLE_FILE_SUPPORT == FALSE
    tObdCdcInfo     cdcInfo;
    UINT32          error;

    OPLK_MEMSET(&cdcInfo, 0, sizeof(tObdCdcInfo));
    cdcInfo.type = kObdCdcTypeFile;
    cdcInfo.handle.fdCdcFile = open(pCdcFilename_p, O_RDONLY | O_BINARY, 0666);
    if (!IS_FD_VALID(cdcInfo.handle.fdCdcFile))
    {   // error occurred
        error = (UINT32)errno;
        ret = eventu_postError(kEventSourceObdu, kErrorObdErrnoSet, sizeof(UINT32), &error);
        return ret;
    }

    cdcInfo.cdcSize = lseek(cdcInfo.handle.fdCdcFile, 0, SEEK_END);
    lseek(cdcInfo.handle.fdCdcFile, 0, SEEK_SET);

    ret = processCdc(&cdcInfo);

    if (cdcInfo.pCurBuffer != NULL)
    {
        OPLK_FREE(cdcInfo.pCurBuffer);
        cdcInfo.pCurBuffer = NULL;
        cdcInfo.bufferSize = 0;
    }

    close(cdcInfo.handle.fdCdcFile);
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

\param  pCdc_p          Pointer to the buffer which contains the concise object
                        definition.
\param  cdcSize_p       Size of the buffer.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError loadCdcBuffer(UINT8* pCdc_p, size_t cdcSize_p)
{
    tOplkError      ret = kErrorOk;
    tObdCdcInfo     cdcInfo;

    OPLK_MEMSET(&cdcInfo, 0, sizeof(tObdCdcInfo));
    cdcInfo.type = kObdCdcTypeBuffer;
    cdcInfo.handle.pNextBuffer = pCdc_p;
    if (cdcInfo.handle.pNextBuffer == NULL)
    {   // error occurred
        ret = eventu_postError(kEventSourceObdu, kErrorObdInvalidDcf, 0, NULL);
        goto Exit;
    }

    cdcInfo.cdcSize = (size_t)cdcSize_p;
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

\param  pCdcInfo_p      Pointer to the CDC information.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError processCdc(tObdCdcInfo* pCdcInfo_p)
{
    tOplkError      ret = kErrorOk;
    UINT32          entriesRemaining;
    UINT            objectIndex;
    UINT            objectSubIndex;
    size_t          curDataSize;

    if ((ret = loadNextBuffer(pCdcInfo_p, sizeof(UINT32))) != kErrorOk)
        return ret;

    entriesRemaining = ami_getUint32Le(pCdcInfo_p->pCurBuffer);

    if (entriesRemaining == 0)
    {
        ret = eventu_postError(kEventSourceObdu, kErrorObdNoConfigData, 0, NULL);
        return ret;
    }

    for (; entriesRemaining != 0; entriesRemaining--)
    {
        if ((ret = loadNextBuffer(pCdcInfo_p, CDC_OFFSET_DATA))  != kErrorOk)
            return ret;

        objectIndex = ami_getUint16Le(&pCdcInfo_p->pCurBuffer[CDC_OFFSET_INDEX]);
        objectSubIndex = ami_getUint8Le(&pCdcInfo_p->pCurBuffer[CDC_OFFSET_SUBINDEX]);
        curDataSize = (size_t)ami_getUint32Le(&pCdcInfo_p->pCurBuffer[CDC_OFFSET_SIZE]);

        DEBUG_LVL_OBD_TRACE("%s: Reading object 0x%04X/%u with size %u from CDC\n",
                             __func__, objectIndex, objectSubIndex, curDataSize);
        if ((ret = loadNextBuffer(pCdcInfo_p, curDataSize)) != kErrorOk)
        {
            DEBUG_LVL_OBD_TRACE("%s: Reading the corresponding data from CDC failed with 0x%02X\n", __func__, ret);
            return ret;
        }

        ret = obd_writeEntryFromLe(objectIndex, objectSubIndex, pCdcInfo_p->pCurBuffer,
                                   (tObdSize)curDataSize);
        if (ret != kErrorOk)
        {
            tEventObdError          obdError;

            obdError.index = objectIndex;
            obdError.subIndex = objectSubIndex;

            DEBUG_LVL_OBD_TRACE("%s: Writing object 0x%04X/%u to local OBD failed with 0x%02X\n",
                                 __func__, objectIndex, objectSubIndex, ret);
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

\param  pCdcInfo_p      Pointer to the CDC information.
\param  bufferSize      Size of the buffer.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError loadNextBuffer(tObdCdcInfo* pCdcInfo_p, size_t bufferSize)
{
    tOplkError  ret = kErrorOk;
#if OBDCDC_DISABLE_FILE_SUPPORT == FALSE
    int         readSize;
    UINT8*      pBuffer;
#endif

    switch (pCdcInfo_p->type)
    {
#if OBDCDC_DISABLE_FILE_SUPPORT == FALSE
        case kObdCdcTypeFile:
            if (pCdcInfo_p->bufferSize < bufferSize)
            {
                if (pCdcInfo_p->pCurBuffer != NULL)
                {
                    OPLK_FREE(pCdcInfo_p->pCurBuffer);
                    pCdcInfo_p->pCurBuffer = NULL;
                    pCdcInfo_p->bufferSize = 0;
                }
                pCdcInfo_p->pCurBuffer = OPLK_MALLOC(bufferSize);
                if (pCdcInfo_p->pCurBuffer == NULL)
                {
                    ret = eventu_postError(kEventSourceObdu, kErrorObdOutOfMemory, 0, NULL);
                    if (ret != kErrorOk)
                        return ret;
                    return kErrorReject;
                }
                pCdcInfo_p->bufferSize = bufferSize;
            }
            pBuffer = pCdcInfo_p->pCurBuffer;

            do
            {
                readSize = read(pCdcInfo_p->handle.fdCdcFile, pBuffer, bufferSize);
                if (readSize <= 0)
                {
                    ret = eventu_postError(kEventSourceObdu, kErrorObdInvalidDcf, 0, NULL);
                    if (ret != kErrorOk)
                        return ret;
                    return kErrorReject;
                }
                pBuffer += readSize;
                bufferSize -= readSize;
                pCdcInfo_p->cdcSize -= readSize;
            }
            while (bufferSize > 0);
            break;
#endif
        case kObdCdcTypeBuffer:
            if (pCdcInfo_p->bufferSize < bufferSize)
            {
                ret = eventu_postError(kEventSourceObdu, kErrorObdInvalidDcf, 0, NULL);
                if (ret != kErrorOk)
                    return ret;
                return kErrorReject;
            }
            pCdcInfo_p->pCurBuffer = pCdcInfo_p->handle.pNextBuffer;
            pCdcInfo_p->handle.pNextBuffer += bufferSize;
            pCdcInfo_p->bufferSize -= bufferSize;
            break;
    }

    return ret;
}

///\}

#endif
