/**
********************************************************************************
\file   obdconf-nios2.c

\brief Implementation of the object dictionary (OD) archive module on Nios2.

The file contains implementation for the object dictionary (OD) configuration
store, load, restore functionality.

The object dictionary is stored in a CFI flash memory.

\ingroup module_obdconf
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2015, Kalycito Infotech Private Limited
Copyright (c) 2013, SYSTEC electronic GmbH
Copyright (c) 2017, Romain Naour (Smile)
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
#include <oplk/obd.h>
#include <user/obdconf.h>

#include <sys/stat.h>
#include <assert.h>
#include <stdlib.h>
#include <fcntl.h>
#include <errno.h>

#if (CONFIG_OBD_USE_STORE_RESTORE != FALSE)


#include <limits.h>
#include <unistd.h>

#include <sys/syslimits.h>

#include "system.h"
#include "sys/alt_flash.h"
#include "sys/alt_flash_dev.h"

//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#ifndef CFI_FLASH_NAME
    #define CFI_FLASH_NAME CFI_FLASH_0_NAME
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

// Hardware specific to S29GL032N (Model 04) 32Mbit flash device.
// Nb of Regions                = 2
// Nb of Sectors                = 71 (SA0 - SA70)

// Region 1 (Sectors SA0 to SA7)
// Region size                  = 65536
// Number of blocks per region  = 8
// Block (Sector) size          = 8192 bits

// Region 2 (Sectors SA8 to SA70)
// Region size                  = 4128768
// Number of blocks per region  = 63
// Block (Sector) size          = 65536 bits

/* /!\
 * By default, the BSP place the reset region at the beginning of CFI flash (0x8000000).
 * So the reset vector must be at 0x8000000 too.
 *
 * But we want to keep the first region to save some small data and the CDC.
 * The CDC is saved in three parts and for each par we use one blocks of 8Ko.
 * The kObdPartGen will be saved in the first block of 8ko (0x000000)
 * The kObdPartMan will be saved in the second block of 8ko (0x002000)
 * The kObdPartDev will be saved in the third block of 8ko (0x004000)
 *
 * The reset region has been moved to the beginning of the CFI flash region 2 (0x8010000)
 * along with the reset vector.
 *
 * The Linker script look like:
 * | Linker Region Name     | Adress Range             | Memory Device Name | Size (bytes) | Offset(bytes)
 * | cfi_flash_BEFORE_RESET | 0x08000000 - 0x0800FFFF  | cfi_flash          | 65536        | 0
 * | reset                  | 0x08010000 - 0x0801001F  | cfi_flash          | 32           | 65536
 * | cfi_flash              | 0x08010020 - 0x083FFFFF  | cfi_flash          | 4128736      | 65568
 */

// Use the CFI flash region 1
#define REGION 0

// kObdPartGen
#define OBD_ARCHIVE_FLASH_PART_COM(region) ( \
    region[REGION].offset + \
    region[REGION].region_size - \
    (region[REGION].number_of_blocks * region[REGION].block_size) \
)

// kObdPartMan
#define OBD_ARCHIVE_FLASH_PART_MAN(region) ( \
    region[REGION].offset + \
    region[REGION].region_size - \
    ((region[REGION].number_of_blocks - 1) * region[REGION].block_size) \
)

// kObdPartDev
#define OBD_ARCHIVE_FLASH_PART_DEV(region) ( \
    region[REGION].offset + \
    region[REGION].region_size - \
    ((region[REGION].number_of_blocks - 2) * region[REGION].block_size) \
)

// We use a CRC16.
#define CRC_SIZE (sizeof(UINT16))

// See OD flash layout below.
#define OBD_DATA_MIN (3 * sizeof(UINT32) + CRC_SIZE)
#define OBD_DATA_MAX(region) (region[REGION].block_size - OBD_DATA_MIN)

#define OBD_DATA_START (3 * sizeof(UINT32))
#define OBD_DATA_END(region) (region[REGION].block_size - CRC_SIZE)

//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------
typedef struct
{
    alt_flash_fd* hBkupArchiveFlashFd;    ///< Handle for the opened flash device
    flash_region* flashRegion;            ///< Region info of the CFI flash
    UINT32        curOdSeek;              ///< Current position in the OD part archive
    UINT32        odSize;                 ///< Current size of the OD part archive
    tObdType      curOdPart;              ///< Id of the currently opened OD part archive
    UINT16        odDataCrc;              ///< 2 Byte CRC16 for the last opened archive
    UINT8*        pDataBuff;              ///< Buffer where to store OD date to be read from or write to the flash device
    BOOL          fOpenForWrite;          ///< Flag to indicate whether the archive is opened for a write operation
}tObdConfInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static UINT32 obdConfSignature_l;
static tObdConfInstance aObdConfInstance_l[1];

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------

/***************************************************************************/
/*          C L A S S  <Store/Load>                                        */
/***************************************************************************/
/**
  Description:

  Since there is no file system, the write() and read() function can't be
  used here. Especially read() function which is used in obdconf-fileio.c
  to read the OD file up to the EOF. When read() reach EOF it means that
  "OD size" bytes has been read. So, on system without a file system, we
  have to store somewhere the OD size. Save it at the beginning of the OD
  part.
  The OD size is not included in the CRC.

  Communication Profile Area part:
          +----------------------+
  0x0000  | OD size of part      | (4 Bytes)
          +----------------------+
  0x0004  | target signature     | (4 Bytes)
          +----------------------+
  0x0008  | OD signature of part | (4 Bytes)
          |  0x1000 - 0x1FFF     |
          +----------------------+
  0x000C  | all OD data of part  | (n Bytes)
          |  0x1000 - 0x1FFF     |
          +----------------------+
  0xNNNN  | OD data CRC          | (2 Bytes)
          +----------------------+

  Manufacturer Specific Profile Area:
          +----------------------+
  0x0000  | OD size of part      | (4 Bytes)
          +----------------------+
  0x0004  | target signature     | (4 Bytes)
          +----------------------+
  0x0008  | OD signature of part | (4 Bytes)
          |  0x2000 - 0x5FFF     |
          +----------------------+
  0x000C  | all OD data of part  | (n Bytes)
          |  0x2000 - 0x5FFF     |
          +----------------------+
  0xNNNN  | OD data CRC          | (2 Bytes)
          +----------------------+

  Standardized Device Profile Area:
          +----------------------+
  0x0000  | OD size of part      | (4 Bytes)
          +----------------------+
  0x0004  | target signature     | (4 Bytes)
          +----------------------+
  0x0008  | OD signature of part | (4 Bytes)
          |  0x6000 - 0x9FFF     |
          +----------------------+
  0x000C  | all OD data of part  | (n Bytes)
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
    int                 ret_code = 0;
    int                 numRegions = 0;

    obdConfSignature_l = 0x444F4C50;  // Signature PLOD

    // Get current instance entry and initialize all members
    pInstEntry = &aObdConfInstance_l[0];
    OPLK_MEMSET(pInstEntry, 0, sizeof(tObdConfInstance));
    pInstEntry->curOdPart = kObdPartNo;
    pInstEntry->pDataBuff = NULL;
    pInstEntry->curOdSeek = 0;
    pInstEntry->odSize = 0;

    // Open flash device to get some useful info about the flash characteristics
    pInstEntry->hBkupArchiveFlashFd = alt_flash_open_dev(CFI_FLASH_NAME);

    if (pInstEntry->hBkupArchiveFlashFd == NULL)
    {
        ret = kErrorObdStoreHwError;
        goto Exit;
    }

    /* Get the flash info. */
    ret_code = alt_get_flash_info(pInstEntry->hBkupArchiveFlashFd,
                             &(pInstEntry->flashRegion),
                             &numRegions);
    if (ret_code != 0)
    {
        ret = kErrorObdStoreHwError;
        goto Exit;
    }

Exit:
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

    // Get current instance entry and initialize all members
    pInstEntry = &aObdConfInstance_l[0];

    OPLK_MEMSET(pInstEntry, 0, sizeof(tObdConfInstance));
    obdConfSignature_l = (UINT32)~0U;

    if (pInstEntry->hBkupArchiveFlashFd != NULL)
    {
        alt_flash_close_dev(pInstEntry->hBkupArchiveFlashFd);
        pInstEntry->hBkupArchiveFlashFd = NULL;
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Create an archive for the OD part

The function creates an archive for the selected OD part.
Existing archive is set invalid.

\param  odPart_p                OD part specifier
\param  odPartSignature_p       Signature for the specified OD part.

\return The function returns a tOplkError error code.

\ingroup module_obdconf
*/
//------------------------------------------------------------------------------
tOplkError obdconf_createPart(tObdPart odPart_p, UINT32 odPartSignature_p)
{
    tOplkError          ret = kErrorObdStoreHwError;
    UINT32*             pOdPartIndex = NULL;
    tObdConfInstance*   pInstEntry;

    // Get current instance entry
    pInstEntry = &aObdConfInstance_l[0];

    // The flash device must be already opened
    if (pInstEntry->hBkupArchiveFlashFd == NULL)
    {
        ret = kErrorObdStoreHwError;
        goto Exit;
    }

    // Is the buffer already created?
    if (pInstEntry->pDataBuff != NULL)
    {
        ret = kErrorObdStoreInvalidState;
        goto Exit;
    }

    // Open file for writing
    pInstEntry->fOpenForWrite = TRUE;
    pInstEntry->curOdPart = odPart_p;
    pInstEntry->curOdSeek = 0;
    pInstEntry->odSize = 0;
    pInstEntry->pDataBuff = calloc(1, (pInstEntry->flashRegion)->block_size);

    if (pInstEntry->pDataBuff == NULL)
    {
        pInstEntry->curOdPart = kObdPartNo;
        ret = kErrorObdStoreHwError;
        goto Exit;
    }

    // Calculate target signature CRC
    pInstEntry->odDataCrc = obdconf_calculateCrc16(0,
                                                   (UINT8*)&obdConfSignature_l,
                                                   sizeof(obdConfSignature_l));

    // Calculate OD signature CRC
    pInstEntry->odDataCrc = obdconf_calculateCrc16(pInstEntry->odDataCrc,
                                                   (UINT8*)&odPartSignature_p,
                                                   sizeof(odPartSignature_p));

    pOdPartIndex = (UINT32*) pInstEntry->pDataBuff;

    // Reserved of OD part size
    pOdPartIndex[0] = 0;
    pInstEntry->curOdSeek += sizeof(UINT32);

    // Write target signature CRC to buffer
    pOdPartIndex[1] = obdConfSignature_l;
    pInstEntry->curOdSeek += sizeof(obdConfSignature_l);

    // Write OD signature CRC to buffer
    pOdPartIndex[2] = odPartSignature_p;
    pInstEntry->curOdSeek += sizeof(odPartSignature_p);

    // odSize take into account only the OD data.
    pInstEntry->odSize = 0;

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
    int                 ret_code = 0x0;
    tObdConfInstance*   pInstEntry;
    UINT32              data = 0;
    UINT32              flashObdPartOffset = 0;

    // Get current instance entry
    pInstEntry = &aObdConfInstance_l[0];

    // The flash device must be opened
    if (pInstEntry->hBkupArchiveFlashFd == NULL)
    {
        ret = kErrorObdStoreHwError;
        goto Exit;
    }

    // Check OD part archive part and set the corresponding flash offset
    switch (odPart_p)
    {
        case kObdPartGen:
            flashObdPartOffset = OBD_ARCHIVE_FLASH_PART_COM(pInstEntry->flashRegion);
            break;

        case kObdPartMan:
            flashObdPartOffset = OBD_ARCHIVE_FLASH_PART_MAN(pInstEntry->flashRegion);
            break;

        case kObdPartDev:
            flashObdPartOffset = OBD_ARCHIVE_FLASH_PART_DEV(pInstEntry->flashRegion);
            break;

        default:
            ret = kErrorObdStoreHwError;
            goto Exit;
    }

    // Do not delete OD part if the buffer doesn't exist
    if (pInstEntry->pDataBuff != NULL)
    {
        ret = kErrorObdStoreInvalidState;
        goto Exit;
    }

    // Mark the signature as invalid (Write to OD signature)
    ret_code = alt_write_flash(
        pInstEntry->hBkupArchiveFlashFd,
        flashObdPartOffset + 2 * sizeof(UINT32),
        &data,
        sizeof(data));
    if (ret_code != 0)
    {
        ret = kErrorObdStoreHwError;
        goto Exit;
    }

    // Also clear the buffer content
    memset(pInstEntry->pDataBuff, 0, OBD_DATA_START);

    // $$ Ideally recalculate the CRC and write to make the flash as uncorrupted

    ret = kErrorOk;

Exit:
    // Free OBD part buffer
    if (pInstEntry->pDataBuff != NULL)
    {
        free(pInstEntry->pDataBuff);
        pInstEntry->pDataBuff = NULL;
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
    tObdConfInstance*   pInstEntry;
    UINT32              flashObdPartOffset = 0;
    int                 ret_code = 0x0;

    // Get current instance entry
    pInstEntry = &aObdConfInstance_l[0];

    // The flash device must be opened
    if (pInstEntry->hBkupArchiveFlashFd == NULL)
    {
        ret = kErrorObdStoreHwError;
        goto Exit;
    }

    // Check OD part archive part and set the corresponding flash offset
    switch (odPart_p)
    {
        case kObdPartGen:
            flashObdPartOffset = OBD_ARCHIVE_FLASH_PART_COM(pInstEntry->flashRegion);
            break;

        case kObdPartMan:
            flashObdPartOffset = OBD_ARCHIVE_FLASH_PART_MAN(pInstEntry->flashRegion);
            break;

        case kObdPartDev:
            flashObdPartOffset = OBD_ARCHIVE_FLASH_PART_DEV(pInstEntry->flashRegion);
            break;

        default:
            ret = kErrorObdStoreHwError;
            goto Exit;
    }

    // Is the buffer already created?
    if (pInstEntry->pDataBuff != NULL)
    {
        ret = kErrorObdStoreInvalidState;
        goto Exit;
    }

    // Open backup archive file for read
    pInstEntry->fOpenForWrite = FALSE;
    pInstEntry->curOdPart = odPart_p;
    pInstEntry->curOdSeek = 0;
    pInstEntry->pDataBuff = calloc(1, (pInstEntry->flashRegion)->block_size);

    if (pInstEntry->pDataBuff == NULL)
    {
        pInstEntry->curOdPart = kObdPartNo;
        ret = kErrorObdStoreHwError;
        goto Exit;
    }

    // Don't check Target signature and OD signature (see obdconf-fileio.c).
    // The signature is already checked by obdconf_getPartArchiveState().

    // Get the OD size
    ret_code = alt_read_flash(pInstEntry->hBkupArchiveFlashFd,
                              flashObdPartOffset,
                              &(pInstEntry->odSize),
                              sizeof(UINT32));
    if (ret_code != 0)
    {
        ret = kErrorObdStoreHwError;
        goto Exit;
    }

    // When the flash device is erased with the flash programmer, the odSize
    // value is 0xFFFFFFFF.
    if ((pInstEntry->odSize > OBD_DATA_MAX(pInstEntry->flashRegion)))
    {
        pInstEntry->odSize = 0;
    }

    // Read the OD part stored in the flash device.
    ret_code = alt_read_flash(pInstEntry->hBkupArchiveFlashFd,
                              flashObdPartOffset,
                              pInstEntry->pDataBuff,
                              pInstEntry->odSize + OBD_DATA_MIN);
    if (ret_code != 0)
    {
        ret = kErrorObdStoreHwError;
        goto Exit;
    }

    // Set file position to the object data part (skip OD size, target and OD signature)
    pInstEntry->curOdSeek = OBD_DATA_START;

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
    UINT8               data;
    int                 ret_code = 0x0;
    tObdConfInstance*   pInstEntry;
    UINT32              flashObdPartOffset = 0;

    // Get current instance entry
    pInstEntry = &aObdConfInstance_l[0];

    // The flash device must be opened
    if (pInstEntry->hBkupArchiveFlashFd == NULL)
    {
        ret = kErrorObdStoreHwError;
        goto Exit;
    }

    // Check OD part archive part and set the corresponding flash offset
    switch (odPart_p)
    {
        case kObdPartGen:
            flashObdPartOffset = OBD_ARCHIVE_FLASH_PART_COM(pInstEntry->flashRegion);
            break;

        case kObdPartMan:
            flashObdPartOffset = OBD_ARCHIVE_FLASH_PART_MAN(pInstEntry->flashRegion);
            break;

        case kObdPartDev:
            flashObdPartOffset = OBD_ARCHIVE_FLASH_PART_DEV(pInstEntry->flashRegion);
            break;

        default:
            ret = kErrorObdStoreHwError;
            goto Exit;
    }

    if (odPart_p != pInstEntry->curOdPart)
    {
        ret = kErrorInvalidInstanceParam;
        goto Exit;
    }

    // Is the buffer exist?
    if (pInstEntry->pDataBuff == NULL)
    {
        ret = kErrorObdStoreInvalidState;
        goto Exit;
    }

    // If flash was opened for write we have to add the CRC16 at the end of the OD data
    if (pInstEntry->fOpenForWrite != FALSE)
    {
        UINT32* pOdPartIndex = (UINT32*) pInstEntry->pDataBuff;
        // Write CRC16 to end of the OD data (in big endian format)
        UINT8* pOdPartCrcMsb = (UINT8*) (pInstEntry->pDataBuff + pInstEntry->odSize + OBD_DATA_START);
        UINT8* pOdPartCrcLsb = (UINT8*) (pInstEntry->pDataBuff + pInstEntry->odSize + OBD_DATA_START + 1);

        if (pInstEntry->odSize > OBD_DATA_MAX(pInstEntry->flashRegion))
        {
            ret = kErrorObdStoreHwError;
            goto Exit;
        }

        // CRC MSB
        data = (UINT8)((pInstEntry->odDataCrc >> 8) & 0xFF);
        *pOdPartCrcMsb = data;

        // CRC LSB
        data = (UINT8)((pInstEntry->odDataCrc >> 0) & 0xFF);
        *pOdPartCrcLsb = data;

        /* Save the OD part size */
        pOdPartIndex[0] = pInstEntry->odSize;

        /* write the buffer to flash */
        ret_code = alt_write_flash(
            pInstEntry->hBkupArchiveFlashFd,
            flashObdPartOffset,
            pInstEntry->pDataBuff,
            pInstEntry->odSize + OBD_DATA_MIN);
        if (ret_code != 0)
        {
            ret = kErrorObdStoreHwError;
            goto Exit;
        }
    }

    free(pInstEntry->pDataBuff);
    pInstEntry->pDataBuff = NULL;
    pInstEntry->curOdSeek = 0;
    pInstEntry->odSize = 0;
    pInstEntry->curOdPart = kObdPartNo;

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
tOplkError obdconf_storePart(tObdPart odPart_p,
                             const void* pData,
                             size_t size_p)
{
    tOplkError          ret = kErrorObdStoreHwError;
    tObdConfInstance*   pInstEntry;
    UINT8*              pWriteOdData = NULL;

    // Get current instance entry
    pInstEntry = &aObdConfInstance_l[0];

    // The flash device must be opened
    if (pInstEntry->hBkupArchiveFlashFd == NULL)
    {
        ret = kErrorObdStoreHwError;
        goto Exit;
    }

    if (odPart_p != pInstEntry->curOdPart)
    {
        ret = kErrorInvalidInstanceParam;
        goto Exit;
    }

    // Is the buffer exist?
    if (pInstEntry->pDataBuff == NULL)
    {
        ret = kErrorObdStoreInvalidState;
        goto Exit;
    }

    pWriteOdData = pInstEntry->pDataBuff + pInstEntry->curOdSeek;

    if (pInstEntry->curOdSeek < OBD_DATA_START)
    {
        ret = kErrorObdStoreHwError;
        goto Exit;
    }

    // OD must have 2 bytes available for the CRC16
    if ((pInstEntry->curOdSeek + size_p) > OBD_DATA_END(pInstEntry->flashRegion))
    {
        ret = kErrorObdStoreHwError;
        goto Exit;
    }

    // Write current OD data to the file and calculate the CRC for it
    pInstEntry->odDataCrc = obdconf_calculateCrc16(pInstEntry->odDataCrc, pData, size_p);

    memcpy((void *)pWriteOdData,
           (void *)pData,
           size_p);

    pInstEntry->curOdSeek += size_p;

    // Update the OD size since we can only write at the end of the data
    pInstEntry->odSize = pInstEntry->curOdSeek - OBD_DATA_START;

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
tOplkError obdconf_loadPart(tObdPart odPart_p,
                            void* pData,
                            size_t size_p)
{
    tOplkError          ret = kErrorObdStoreHwError;
    tObdConfInstance*   pInstEntry;
    UINT8*              pReadOdData = NULL;

    // Get current instance entry
    pInstEntry = &aObdConfInstance_l[0];

    // The flash device must be opened
    if (pInstEntry->hBkupArchiveFlashFd == NULL)
    {
        ret = kErrorObdStoreHwError;
        goto Exit;
    }

    if (odPart_p != pInstEntry->curOdPart)
    {
        ret = kErrorInvalidInstanceParam;
        goto Exit;
    }

    // Is the buffer exist?
    if (pInstEntry->pDataBuff == NULL)
    {
        ret = kErrorObdStoreInvalidState;
        goto Exit;
    }

    pReadOdData = (UINT8*)pInstEntry->pDataBuff + pInstEntry->curOdSeek;

    if (pInstEntry->curOdSeek < OBD_DATA_START)
    {
        ret = kErrorObdStoreHwError;
        goto Exit;
    }

    if ((pInstEntry->curOdSeek + size_p) > OBD_DATA_END(pInstEntry->flashRegion))
    {
        ret = kErrorObdStoreDataLimitExceeded;
        goto Exit;
    }

    // Read OD data from current file position
    memcpy((void *)pData,
           (void *)pReadOdData,
           size_p);

    pInstEntry->curOdSeek += size_p;

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
\param  odPartSignature_p       Signature for the specified OD part.

\return The function returns a tOplkError error code.
\retVal kErrorOk                The OD part archive is valid.
\retVal Otherwise               The OD part archive is not valid.

\ingroup module_obdconf
*/
//------------------------------------------------------------------------------
tOplkError obdconf_getPartArchiveState(tObdPart odPart_p, UINT32 odPartSignature_p)
{
    tOplkError          ret = kErrorOk;
    UINT32*             pOdPartIndex = NULL;
    UINT8*              pReadOdData = NULL;
    UINT16              dataCrc = 0;
    tObdConfInstance*   pInstEntry;
    int                 ret_code = 0x0;
    UINT32              flashObdPartOffset = 0;

    // Get current instance entry
    pInstEntry = &aObdConfInstance_l[0];

    // The flash device must be opened
    if (pInstEntry->hBkupArchiveFlashFd == NULL)
    {
        ret = kErrorObdStoreHwError;
        goto Exit;
    }

    // Is the buffer already created?
    if (pInstEntry->pDataBuff != NULL)
    {
        ret = kErrorObdStoreInvalidState;
        goto Exit;
    }

    // Check OD part archive part and set the corresponding flash offset
    switch (odPart_p)
    {
        case kObdPartGen:
            flashObdPartOffset = OBD_ARCHIVE_FLASH_PART_COM(pInstEntry->flashRegion);
            break;

        case kObdPartMan:
            flashObdPartOffset = OBD_ARCHIVE_FLASH_PART_MAN(pInstEntry->flashRegion);
            break;

        case kObdPartDev:
            flashObdPartOffset = OBD_ARCHIVE_FLASH_PART_DEV(pInstEntry->flashRegion);
            break;

        default:
            ret = kErrorObdStoreHwError;
            goto Exit;
    }

    pInstEntry->pDataBuff = calloc(1, (pInstEntry->flashRegion)->block_size);
    if (pInstEntry->pDataBuff == NULL)
    {
        ret = kErrorObdStoreHwError;
        goto Exit;
    }

    // Get the OD size
    ret_code = alt_read_flash(pInstEntry->hBkupArchiveFlashFd,
                              flashObdPartOffset,
                              &(pInstEntry->odSize),
                              sizeof(UINT32));
    if (ret_code != 0)
    {
        ret = kErrorObdStoreHwError;
        goto Exit;
    }

    // When the flash device is erased with the flash programmer, the odSize
    // value is 0xFFFFFFFF. In this case return kErrorObdStoreDataObsolete to
    // download the new CDC by SDO transfer.
    if (pInstEntry->odSize > OBD_DATA_MAX(pInstEntry->flashRegion))
    {
        ret = kErrorObdStoreDataObsolete;
        goto Exit;
    }

    // Read the OD part stored in the flash device.
    ret_code = alt_read_flash(pInstEntry->hBkupArchiveFlashFd,
                              flashObdPartOffset,
                              pInstEntry->pDataBuff,
                              pInstEntry->odSize + OBD_DATA_MIN);
    if (ret_code != 0)
    {
        ret = kErrorObdStoreHwError;
        goto Exit;
    }

    pOdPartIndex = (UINT32*) pInstEntry->pDataBuff;

    // Read target signature and calculate the CRC for it
    dataCrc = obdconf_calculateCrc16(0, (UINT8*)&pOdPartIndex[1], sizeof(UINT32));

    // Read OD signature and calculate the CRC for it
    dataCrc = obdconf_calculateCrc16(dataCrc, (UINT8*)&pOdPartIndex[2], sizeof(UINT32));

    // Check if both target signature and OD signature are correct
    if ((pOdPartIndex[1] != obdConfSignature_l) || (pOdPartIndex[2]  != odPartSignature_p))
    {
        ret = kErrorObdStoreDataObsolete;
        goto Exit;
    }

    // Set the pReadOdData to the beginning of OD data.
    pReadOdData = pInstEntry->pDataBuff + OBD_DATA_START;

    // Calculate OD data CRC over all data bytes.
    // Include the CDC value to get dataCrc = 0 if the OD part is not corrupted.
    dataCrc = obdconf_calculateCrc16(dataCrc, pReadOdData, pInstEntry->odSize + CRC_SIZE);

    // Check OD data CRC (always zero because CRC has to be set at the end of the file in big endian format)
    if (dataCrc != 0)
    {
        ret = kErrorObdStoreHwError;
        goto Exit;
    }

Exit:
    // Free OBD part buffer
    if (pInstEntry->pDataBuff != NULL)
    {
        free(pInstEntry->pDataBuff);
    }

    pInstEntry->pDataBuff = NULL;
    pInstEntry->curOdPart = kObdPartNo;
    pInstEntry->curOdSeek = 0;
    pInstEntry->odSize = 0;

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Set OD archive path

This function sets the root path for all OD part archives. This module uses
this path for searching the OD part archives ensuring store/restore operation.

Note: There is no filesystem on NOOS Nios2 target, so this function is a nop.

\param  pBackupPath_p           OD part archives' path.

\return The function returns a tOplkError error code.

\ingroup module_obdconf
*/
//------------------------------------------------------------------------------
tOplkError obdconf_setBackupArchivePath(const char* pBackupPath_p)
{
    UNUSED_PARAMETER(pBackupPath_p);
    return kErrorOk;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//

#endif // if (CONFIG_OBD_USE_STORE_RESTORE != FALSE)
