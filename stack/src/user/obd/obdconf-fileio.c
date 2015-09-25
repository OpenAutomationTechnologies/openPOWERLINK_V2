/**
********************************************************************************
\file   obdconf.c

\brief Implementation of the object dictionary (OD) archive module.

The file contains implementation for the object dictionary (OD) configuration
store, load, restore functionality.

\ingroup module_obdconf
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2015, Kalycito Infotech Private Limited
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
#ifdef _WIN32
    #include <windows.h>
    #pragma warning(disable:4996)
#endif

#include <common/oplkinc.h>
#include <oplk/obd.h>
#include <oplk/obdconf.h>
#include <common/target.h>

#include <sys/stat.h>
#include <assert.h>
#include <stdlib.h>
#include <fcntl.h>
#include <errno.h>

#if (CONFIG_OBD_USE_STORE_RESTORE != FALSE)

#if (TARGET_SYSTEM == _WIN32_)

    #include <io.h>
    #include <sys/types.h>
    #include <sys/utime.h>
    #include <sys/timeb.h>
    #include <time.h>
    #include <direct.h>
    #include <string.h>

#elif (TARGET_SYSTEM == _LINUX_)

    #include <sys/io.h>
    #include <unistd.h>
    #include <sys/vfs.h>
    #include <sys/types.h>
    #include <sys/timeb.h>
    #include <utime.h>
    #include <limits.h>

#elif (DEV_SYSTEM == _DEV_PAR_BECK1X3_) // $$: not yet supported

    #include <io.h>
    #include <string.h>

#endif

//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#if (TARGET_SYSTEM == _WIN32_)

    #define MAX_PATH_LEN _MAX_PATH
    #define flush  _commit

#elif (TARGET_SYSTEM == _LINUX_)

    #define O_BINARY 0
    #define MAX_PATH_LEN PATH_MAX
    #define flush  fsync

#elif (DEV_SYSTEM == _DEV_PAR_BECK1X3_) // $$: not yet supported

    #define MAX_PATH_LEN _MAX_PATH
    #define flush(h)                    // #define flush() to nothing

#endif
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
#ifndef OBD_ARCHIVE_FILENAME_PREFIX
    #define OBD_ARCHIVE_FILENAME_PREFIX        "oplkOd"
#endif

#ifndef OBD_ARCHIVE_FILENAME_EXTENSION
    #define OBD_ARCHIVE_FILENAME_EXTENSION     ".bin"
#endif

//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------
typedef struct
{
    INT         hBkupArchiveFile;       ///< Handle for the opened OD part archive
    char*       pBackupPath;            ///< The parent directory for the archives
    BOOL        fOpenForWrite;          ///< Flag to indicate whether the archive is opened for a write operation
    UINT16      odDataCrc;              ///< 2 Byte CRC for the last opened archive
    tObdType    curOdPart;              ///< Currently opened OD part archive
}tObdConfInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static UINT32 obdConfSignature_l;
static tObdConfInstance aObdConfInstance_l[1];

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static tOplkError getOdPartArchivePath(tObdPart odPart_p, char* pBkupPath_p,
                                       char* pFilePathName_p);

/***************************************************************************/
/*          C L A S S  <Store/Load>                                        */
/***************************************************************************/
/**
  Description:

  File oplkOd_Com.bin:
          +----------------------+
  0x0000  | target signature     | (4 Bytes)
          +----------------------+
  0x0004  | OD signature of part | (4 Bytes)
          |  0x1000 - 0x1FFF     |
          +----------------------+
  0x0008  | all OD data of part  | (n Bytes)
          |  0x1000 - 0x1FFF     |
          +----------------------+
  0xNNNN  | OD data CRC          | (2 Bytes)
          +----------------------+

  File oplkOd_Man.bin:
          +----------------------+
  0x0000  | target signature     | (4 Bytes)
          +----------------------+
  0x0004  | OD signature of part | (4 Bytes)
          |  0x2000 - 0x5FFF     |
          +----------------------+
  0x0008  | all OD data of part  | (n Bytes)
          |  0x2000 - 0x5FFF     |
          +----------------------+
  0xNNNN  | OD data CRC          | (2 Bytes)
          +----------------------+

  File oplkOd_Dev.bin:
          +----------------------+
  0x0000  | target signature     | (4 Bytes)
          +----------------------+
  0x0004  | OD signature of part | (4 Bytes)
          |  0x6000 - 0x9FFF     |
          +----------------------+
  0x0008  | all OD data of part  | (n Bytes)
          |  0x6000 - 0x9FFF     |
          +----------------------+
  0xNNNN  | OD data CRC          | (2 Bytes)
          +----------------------+
*/

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//
//------------------------------------------------------------------------------
/**
\brief  Initialize OD archive module

The function initializes OD archive module.

\return The function returns a tOplkError error code.

\ingroup module_obdconf
*/
//------------------------------------------------------------------------------
tOplkError obdconf_init(void)
{
    tOplkError          ret = kErrorOk;
    tObdConfInstance*   pInstEntry;

    obdConfSignature_l = 0x444F4C50;  // signature PLOD

    // Get current instance entry and initialize all members
    pInstEntry = &aObdConfInstance_l[0];
    OPLK_MEMSET(pInstEntry, 0, sizeof(tObdConfInstance));
    pInstEntry->hBkupArchiveFile = -1;
    pInstEntry->curOdPart = kObdPartNo;

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Cleanup OD archive module

The function cleans up OD archive module.

\return The function returns a tOplkError error code.

\ingroup module_obdconf
*/
//------------------------------------------------------------------------------
tOplkError obdconf_exit(void)
{
    tOplkError        ret = kErrorOk;
    tObdConfInstance* pInstEntry;

    pInstEntry = &aObdConfInstance_l[0];
    pInstEntry->curOdPart = kObdPartNo;
    pInstEntry->hBkupArchiveFile = -1;
    OPLK_MEMSET(pInstEntry, 0, sizeof(tObdConfInstance));
    obdConfSignature_l = (UINT32)~0U;

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Create an archive for the OD part

The function creates an archive for the selected OD part.
Existing archive is set invalid.

\param  odPart_p                OD part specifier

\return The function returns a tOplkError error code.

\ingroup module_obdconf
*/
//------------------------------------------------------------------------------
tOplkError obdconf_createPart(tObdPart odPart_p, UINT32 odPartSignature_p)
{
    tOplkError          ret = kErrorObdStoreHwError;
    INT                 writeCount;
    char                aFilePath[MAX_PATH_LEN];
    tObdConfInstance*   pInstEntry;

    // Get current instance entry
    pInstEntry = &aObdConfInstance_l[0];

    // Get the file path for current OD part and instance
    ret = getOdPartArchivePath(odPart_p, pInstEntry->pBackupPath, &aFilePath[0]);
    if (ret != kErrorOk)
        goto Exit;

    // Is the file already opened?
    if (pInstEntry->hBkupArchiveFile >= 0)
    {
        ret = kErrorObdStoreInvalidState;
        goto Exit;
    }

    // Open file for writing
    pInstEntry->fOpenForWrite = TRUE;
    pInstEntry->curOdPart = odPart_p;
    pInstEntry->hBkupArchiveFile = open(aFilePath, O_CREAT | O_TRUNC | O_WRONLY | O_BINARY, 0666);
    if (pInstEntry->hBkupArchiveFile < 0)
    {
        pInstEntry->curOdPart = kObdPartNo;
        pInstEntry->hBkupArchiveFile = -1;
        ret = kErrorObdStoreHwError;
        goto Exit;
    }

    // Write target signature and calculate CRC for it
    pInstEntry->odDataCrc = OPLK_CALCULATE_CRC16(0,
                                                 (UINT8*)&obdConfSignature_l,
                                                 sizeof(obdConfSignature_l));
    writeCount = write(pInstEntry->hBkupArchiveFile,
                       (UINT8*)&obdConfSignature_l,
                       sizeof(obdConfSignature_l));
    if (writeCount != (INT)sizeof(obdConfSignature_l))
    {
        ret = kErrorObdStoreHwError;
        goto Exit;
    }

    // Write OD signature and calculate CRC for it
    pInstEntry->odDataCrc = OPLK_CALCULATE_CRC16(pInstEntry->odDataCrc,
                                                 (UINT8*)&odPartSignature_p,
                                                 sizeof(odPartSignature_p));
    writeCount = write(pInstEntry->hBkupArchiveFile,
                       (UINT8*)&odPartSignature_p,
                       sizeof(odPartSignature_p));
    if (writeCount != (INT)sizeof(odPartSignature_p))
    {
        ret = kErrorObdStoreHwError;
        goto Exit;
    }

    ret = kErrorOk;

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Delete the archive for the OD part

The function deletes the archive for the selected OD part or sets it invalid.

\param  odPart_p                OD part specifier

\return The function returns a tOplkError error code.

\ingroup module_obdconf
*/
//------------------------------------------------------------------------------
tOplkError obdconf_deletePart(tObdPart odPart_p)
{
    tOplkError          ret = kErrorObdStoreHwError;
    char                aFilePath[MAX_PATH_LEN];
    tObdConfInstance*   pInstEntry;
    INT                 writeCount;
    UINT32              data = (UINT32)~0U;

    // Get current instance entry
    pInstEntry = &aObdConfInstance_l[0];

    // Get the file path for current OD part and instance
    ret = getOdPartArchivePath(odPart_p, pInstEntry->pBackupPath, &aFilePath[0]);
    if (ret != kErrorOk)
        goto Exit;

    // Use the local file handle to avoid race condition
    if (pInstEntry->hBkupArchiveFile >= 0)
    {
        ret = kErrorObdStoreInvalidState;
        goto Exit;
    }

    pInstEntry->hBkupArchiveFile = open(aFilePath, O_WRONLY | O_BINARY, 0666);
    if (pInstEntry->hBkupArchiveFile < 0)
    {
        ret = kErrorObdStoreHwError;
        goto Exit;
    }

    // Set file position to OD part signature
    lseek(pInstEntry->hBkupArchiveFile, sizeof(obdConfSignature_l), SEEK_SET);

    // Mark the signature as invalid
    writeCount = write(pInstEntry->hBkupArchiveFile, &data, sizeof(data));
    if (writeCount != (INT)sizeof(data))
    {
        ret = kErrorObdStoreHwError;
        goto Exit;
    }

    // $$ Ideally recalculate the CRC and write to make the file as uncorrupted

    // Sync file to disc
    flush(pInstEntry->hBkupArchiveFile);

    ret = kErrorOk;

Exit:
    // Close archive file and set file handle invalid
    if (pInstEntry->hBkupArchiveFile >= 0)
    {
        close(pInstEntry->hBkupArchiveFile);
        pInstEntry->hBkupArchiveFile = -1;
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Open the archive for the OD part

This function is called to open the selected OD part archive, for the subsequent
read operation to load the OD configuration. The OD part archive must exist prior
to calling this function.

\param  odPart_p                OD part specifier

\return The function returns a tOplkError error code.

\ingroup module_obdconf
*/
//------------------------------------------------------------------------------
tOplkError obdconf_openReadPart(tObdPart odPart_p)
{
    UINT8               ret = kErrorObdStoreHwError;
    char                aFilePath[MAX_PATH_LEN];
    tObdConfInstance*   pInstEntry;

    // Get current instance entry
    pInstEntry = &aObdConfInstance_l[0];

    // Get the file path for current OD part and instance
    ret = getOdPartArchivePath(odPart_p, pInstEntry->pBackupPath, &aFilePath[0]);
    if (ret != kErrorOk)
        goto Exit;

    // Is the file already opened?
    if (pInstEntry->hBkupArchiveFile >= 0)
    {
        ret = kErrorObdStoreInvalidState;
        goto Exit;
    }

    // Open backup archive file for read
    pInstEntry->fOpenForWrite = FALSE;
    pInstEntry->curOdPart = odPart_p;
    pInstEntry->hBkupArchiveFile = open(aFilePath, O_RDONLY | O_BINARY, 0666);

    if (pInstEntry->hBkupArchiveFile < 0)
    {
        // Backup archive file could not be opened
        pInstEntry->curOdPart = kObdPartNo;
        pInstEntry->hBkupArchiveFile = -1;
        ret = kErrorObdStoreHwError;
        goto Exit;
    }

    // Set file position to the object data part
    lseek(pInstEntry->hBkupArchiveFile, 2 * sizeof(UINT32), SEEK_SET);

    ret = kErrorOk;

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Close the archive for the OD part

This function is called to close the selected OD part archive, after read/write
operation to load/store the OD configuration. The OD part archive must exist
prior to calling this function.

For a write operation, the function appends the calculated CRC to the end of
archive.

\param  odPart_p                OD part specifier

\return The function returns a tOplkError error code.

\ingroup module_obdconf
*/
//------------------------------------------------------------------------------
tOplkError obdconf_closePart(tObdPart odPart_p)
{
    tOplkError          ret = kErrorOk;
    INT                 writeCount;
    UINT8               data;
    tObdConfInstance*   pInstEntry;

    // Get current instance entry
    pInstEntry = &aObdConfInstance_l[0];
    if (odPart_p != pInstEntry->curOdPart)
    {
        ret = kErrorInvalidInstanceParam;
        goto Exit;
    }

    // Is the file not opened?
    if (pInstEntry->hBkupArchiveFile < 0)
    {
        ret = kErrorObdStoreInvalidState;
        goto Exit;
    }

    // If file was opened for write we have to add the OD data CRC at the end of the file
    if (pInstEntry->fOpenForWrite != FALSE)
    {
        // Write CRC16 to end of the file (in big endian format)
        data = (UINT8)((pInstEntry->odDataCrc >> 8) & 0xFF);
        writeCount = write(pInstEntry->hBkupArchiveFile, (UINT8*)&data, sizeof(data));
        data = (UINT8)((pInstEntry->odDataCrc >> 0) & 0xFF);
        writeCount += write(pInstEntry->hBkupArchiveFile, (UINT8*)&data, sizeof(data));
        if (writeCount != (INT)(sizeof(data) * 2))
        {   // Save error code and close the file
            ret = kErrorObdStoreHwError;
        }

        // Sync file to disc
        flush(pInstEntry->hBkupArchiveFile);
    }

    // Close archive file and set file handle invalid
    close(pInstEntry->hBkupArchiveFile);
    pInstEntry->curOdPart = kObdPartNo;
    pInstEntry->hBkupArchiveFile = -1;

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Store the OD part configuration

This function writes the specified OD part configuration parameters into the
corresponding part archive.

\param  odPart_p                OD part specifier.
\param  pData_p                 Pointer to the buffer containing configuration
                                data to be stored.
\param  size_p                  Total size of the data to be stored, in bytes.

\return The function returns a tOplkError error code.

\ingroup module_obdconf
*/
//------------------------------------------------------------------------------
tOplkError obdconf_storePart(tObdPart odPart_p, UINT8 *pData, UINT32 size_p)
{
    tOplkError          ret = kErrorObdStoreHwError;
    INT                 writeCount;
    tObdConfInstance*   pInstEntry;

    // Get current instance entry
    pInstEntry = &aObdConfInstance_l[0];

    if (odPart_p != pInstEntry->curOdPart)
    {
        ret = kErrorInvalidInstanceParam;
        goto Exit;
    }

    // Is the file not opened?
    if (pInstEntry->hBkupArchiveFile < 0)
    {
        ret = kErrorObdStoreInvalidState;
        goto Exit;
    }

    // Write current OD data to the file and calculate the CRC for it
    pInstEntry->odDataCrc = OPLK_CALCULATE_CRC16(pInstEntry->odDataCrc, pData, size_p);
    writeCount = write(pInstEntry->hBkupArchiveFile, (UINT8*)pData, size_p);
    if (writeCount != (INT)size_p)
    {
        ret = kErrorObdStoreHwError;
        goto Exit;
    }

    ret = kErrorOk;
Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Load the OD part configuration

This function reads the specified OD part configuration parameters from the
corresponding part archive.

\param  odPart_p                OD part specifier.
\param  pData_p                 Pointer to the buffer to hold the configuration
                                data read from the archive.
\param  size_p                  Total size of the data to be read, in bytes.

\return The function returns a tOplkError error code.

\ingroup module_obdconf
*/
//------------------------------------------------------------------------------
tOplkError obdconf_loadPart(tObdPart odPart_p, UINT8 *pData, UINT32 size_p)
{
    tOplkError          ret = kErrorObdStoreHwError;
    INT                 readCount;
    tObdConfInstance*   pInstEntry;

    // Get current instance entry
    pInstEntry = &aObdConfInstance_l[0];

    if (odPart_p != pInstEntry->curOdPart)
    {
        ret = kErrorInvalidInstanceParam;
        goto Exit;
    }

    // Is the file not opened?
    if (pInstEntry->hBkupArchiveFile < 0)
    {
        ret = kErrorObdStoreInvalidState;
        goto Exit;
    }

    // Read OD data from current file position
    readCount = read(pInstEntry->hBkupArchiveFile, (UINT8*)pData, size_p);
    if (readCount < 0)
    {
        ret = kErrorObdStoreHwError;
        goto Exit;
    }

    if (readCount != (INT)size_p)
    {
        ret = kErrorObdStoreDataLimitExceeded;
        goto Exit;
    }

    ret = kErrorOk;

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Get target's store/restore capabilities

This function returns the store/restore capabilities of the target platform
for the OD part corresponding to the specified store/restore parameter object
sub-index.

\param  index_p                 OD store/restore parameter object index.
\param  subIndex_p              OD store/restore parameter object sub-index.
\param  pOdPart_p               Pointer to hold the OD part specifier
                                corresponding to the sub-index.
\param  pDevCap_p               Pointer to hold the target device store/restore
                                capabilities.

\note For the current filesystem based archive implementation, all OD parts
      are configured to use 0x00000001 mode i.e. 'store on command'.

\return The function returns a tOplkError error code.

\ingroup module_obdconf
*/
//------------------------------------------------------------------------------
tOplkError obdconf_getTargetCapabilities(UINT index_p, UINT subIndex_p,
                                         tObdPart* pOdPart_p, UINT32* pDevCap_p)
{
    tOplkError ret = kErrorOk;

    if ((pOdPart_p == NULL) || (pDevCap_p == NULL) ||
        ((index_p != 0x1010) && (index_p != 0x1011)))
    {
        ret = kErrorApiInvalidParam;
        goto Exit;
    }

    switch (subIndex_p)
    {
        // Device supports store/restore of index ranges
        // 0x1000-0x1FFF, 0x2000-0x5FFF, 0x6000-0x9FFF
        case 1:
            *pDevCap_p = OBD_STORE_ON_COMMAND;
            *pOdPart_p = kObdPartAll;
            break;

        // Device supports store/restore of index range
        // 0x1000-0x1FFF
        case 2:
            *pDevCap_p = OBD_STORE_ON_COMMAND;
            *pOdPart_p = kObdPartGen;
            break;

        // Device supports store/restore of index
        // 0x6000-0x9FFF
        case 3:
            *pDevCap_p = OBD_STORE_ON_COMMAND;
            *pOdPart_p = kObdPartDev;
            break;

        // Device supports store/restore of index
        // 0x2000-0x5FFF
        case 4:
            *pDevCap_p = OBD_STORE_ON_COMMAND;
            *pOdPart_p = kObdPartMan;
            break;

        default:
            *pDevCap_p = 0;
            break;
    }

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Check if the OD part archive is valid

This function checks the data integrity and signature of the specified
OD part archive and returns if the archive is valid or invalid.

The OD part archive has to opened prior to calling this function.
The function resets the read pointer for the archive to the beginning
of the configuration data.

\param  odPart_p                OD part specifier.

\return The function returns a boolean value.
\retVal TRUE                    The OD part archive is valid.
\retVal FALSE                   The OD part archive is not valid.

\return The function returns a tOplkError error code.

\ingroup module_obdconf
*/
//------------------------------------------------------------------------------
tOplkError obdconf_getPartArchiveState(tObdPart odPart_p, UINT32 odPartSignature_p)
{
    tOplkError          ret = kErrorOk;
    INT                 readCount;
    UINT32              readTargetSign;
    UINT32              readOdSign;
    UINT8               aTempBuffer[8];
    INT                 count;
    UINT16              dataCrc;
    tObdConfInstance*   pInstEntry;
    char                aFilePath[MAX_PATH_LEN];

    // Get current instance entry
    pInstEntry = &aObdConfInstance_l[0];

    // Get the file path for current OD part and instance
    ret = getOdPartArchivePath(odPart_p, pInstEntry->pBackupPath, &aFilePath[0]);
    if (ret != kErrorOk)
        goto Exit;

    // Use the local file handle to avoid race conditions
    if (pInstEntry->hBkupArchiveFile >= 0)
    {
        ret = kErrorObdStoreInvalidState;
        goto Exit;
    }

    // Open backup archive file for read
    pInstEntry->hBkupArchiveFile = open(aFilePath, O_RDONLY | O_BINARY, 0666);

    if (pInstEntry->hBkupArchiveFile < 0)
    {
        // Backup archive file could not be opened
        ret = kErrorObdStoreHwError;
        goto Exit;
    }

    // Set file position to the begin of the file
    lseek(pInstEntry->hBkupArchiveFile, 0, SEEK_SET);

    // Read target signature and calculate the CRC for it
    readCount = read(pInstEntry->hBkupArchiveFile, &readTargetSign, sizeof(readTargetSign));
    dataCrc = OPLK_CALCULATE_CRC16(0, (UINT8*)&readTargetSign, sizeof(readTargetSign));
    if (readCount != (INT)sizeof(readTargetSign))
    {
        ret = kErrorObdStoreHwError;
        goto Exit;
    }

    // Read OD signature and calculate the CRC for it
    readCount = read(pInstEntry->hBkupArchiveFile, &readOdSign, sizeof(readOdSign));
    dataCrc = OPLK_CALCULATE_CRC16(dataCrc, (UINT8*)&readOdSign, sizeof(readOdSign));
    if (readCount != (INT)sizeof(readOdSign))
    {
        ret = kErrorObdStoreHwError;
        goto Exit;
    }

    // Check if both target signature and OD signature are correct
    if ((readTargetSign != obdConfSignature_l) || (readOdSign  != odPartSignature_p))
    {
        ret = kErrorObdStoreDataObsolete;
        goto Exit;
    }

    // Calculate OD data CRC over all data bytes
    for (;;)
    {
        count = read(pInstEntry->hBkupArchiveFile, &aTempBuffer[0], sizeof(aTempBuffer));
        if (count > 0)
        {
            dataCrc = OPLK_CALCULATE_CRC16(dataCrc, (UINT8*)&aTempBuffer[0], count);
        }
        else
        {
            break;
        }
    }

    // Check OD data CRC (always zero because CRC has to be set at the end of the file in big endian format)
    if (dataCrc != 0)
    {
        ret = kErrorObdStoreHwError;
        goto Exit;
    }

Exit:
    if (pInstEntry->hBkupArchiveFile >= 0)
    {
        close(pInstEntry->hBkupArchiveFile);
        pInstEntry->hBkupArchiveFile = -1;
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Set OD archive path

This function sets the root path for all OD part archives. This module uses
this path for searching the OD part archives ensuring store/restore operation.

\param  pBackupPath_p           OD part archives' path.

\return The function returns a tOplkError error code.

\ingroup module_obdconf
*/
//------------------------------------------------------------------------------
tOplkError obdconf_setBackupArchivePath(const char* pBackupPath_p)
{
    tOplkError          ret = kErrorObdStoreHwError;
    tObdConfInstance*   pInstEntry;

    // Check pointer to backup path string
    if (pBackupPath_p == NULL)
    {
        ret = kErrorApiInvalidParam;
        goto Exit;
    }

    // Get current instance entry
    pInstEntry = &aObdConfInstance_l[0];

    // Save pointer to backup path
    pInstEntry->pBackupPath = (char*)pBackupPath_p;

    ret = kErrorOk;

Exit:
    return ret;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief  Get complete path to the OD part archive

The functions returns the complete path to the specifed OD part archive, from
the archive parent directory path provided.

\param  odPart_p                OD part specifier.
\param  pBkupPath_p             Parent directory path string.
\param  pFilePathName_p         String pointer to hold the archive file path.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError getOdPartArchivePath(tObdPart odPart_p, char* pBkupPath_p,
                                       char* pFilePathName_p)
{
    tOplkError ret = kErrorOk;
    size_t     len;

    // Build complete file path string
    if (pBkupPath_p != NULL)
    {
        strcpy(pFilePathName_p, pBkupPath_p);
    }
    else
    {
        pFilePathName_p[0] = '\0';
    }

    len = strlen(pFilePathName_p);
    if ((len > 0) &&
        (pFilePathName_p[len-1] != '\\') &&
        (pFilePathName_p[len-1] != '/'))
    {
        strcat(pFilePathName_p, "/");
    }

    strcat(pFilePathName_p, OBD_ARCHIVE_FILENAME_PREFIX);

    // Check OD part archive name suffix
    switch (odPart_p)
    {
        case kObdPartGen:
            strcat(pFilePathName_p, "_partCom");
            break;

        case kObdPartMan:
            strcat(pFilePathName_p, "_partMan");
            break;

        case kObdPartDev:
            strcat(pFilePathName_p, "_partDev");
            break;

        default:
            ret = kErrorApiInvalidParam;
            goto Exit;
    }

    strcat(pFilePathName_p, OBD_ARCHIVE_FILENAME_EXTENSION);

Exit:
    return ret;
}

/// \}
#endif // if (CONFIG_OBD_USE_STORE_RESTORE != FALSE)
