/**
********************************************************************************
\file  fs_sdcard.c

\brief  Contains code for the SD card FLASH functionality.

This is support file to provide support for reading and writing to a SD FLASH
especially to read the CDC file during runtime.

\ingroup module_demo
*******************************************************************************/
/*------------------------------------------------------------------------------
Copyright (c) 2014, Kalycito Infotech Private Limited
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

/***************************** Include Files *********************************/

#include "fs_sdcard.h"
#ifdef XPAR_PS7_SD_0_S_AXI_BASEADDR

#include <stdio.h>
#include "xstatus.h"

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

static FATFS    fatfs;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
 \brief  Init routine for SD flash

 This function initializes the controller for the SD FLASH interface

 \return     - XST_SUCCESS if the controller initializes correctly
 *           - XST_FAILURE if the controller fails to initializes correctly

 \ingroup module_demo
 */
//------------------------------------------------------------------------------
INT sd_fs_init(void)
{
    FRESULT    ret;

    /* Register volume work area, initialize device */
    ret = f_mount(0, &fatfs);

    if (ret != FR_OK)
    {
        printf("{%s}: failed to mount file system %d\n", __func__, ret);
        return XST_FAILURE;
    }

    return XST_SUCCESS;
}

//------------------------------------------------------------------------------
/**
 \brief  Open File

 This function opens a file to read/write on SD FLASH

 \param      pFile_p        pointer to receive the FILE structure
                            to the file to be opened
 \param      strFilename_p  Name of the file to be opened
 \param      mode_p         FA_READ - open for reading
                            FA_WRITE - open for read/write

 \return     - XST_SUCCESS if the controller opens file correctly
 *           - XST_FAILURE if the controller fails to open correctly

 \ingroup module_demo
 */
//------------------------------------------------------------------------------
INT sd_open(FIL* pFile_p, char* strFilename_p, BYTE mode_p)
{
    FRESULT    ret;

    ret = f_open(pFile_p, strFilename_p, mode_p);
    if (ret)
    {
        printf("{%s}: Unable to open file %s: %d\n", __func__, strFilename_p, ret);
        return XST_FAILURE;
    }

    return XST_SUCCESS;
}

//------------------------------------------------------------------------------
/**
 \brief  Read File

 This function Reads the specified number of bytes from a file on SD FLASH

 \param      pFile_p        pointer to the FILE structure to the file to be read
 \param      pBuffer_p      Buffer to receive the read data
 \param      count_p      Number of bytes to read
 \param      pReadNum_p     Number of bytes read

 \return     - XST_SUCCESS if the controller reads correctly
 *           - XST_FAILURE if the controller fails to read

 \ingroup module_demo
 */
//------------------------------------------------------------------------------
INT sd_read(FIL* pFile_p, void* pBuffer_p, UINT count_p, UINT* pReadNum_p)
{
    FRESULT    ret;

    ret = f_read(pFile_p, pBuffer_p, count_p, pReadNum_p);
    if (ret)
    {
        printf("ERROR: {%s} failed %d\r\n", __func__, ret);
        return XST_FAILURE;
    }

    return XST_SUCCESS;
}

//------------------------------------------------------------------------------
/**
 \brief  Write File

 This function writes the specified number of bytes to a file on SD FLASH

 \param      pFile_p        pointer to the FILE structure to the file to write
 \param      pBuffer_p      Buffer to containing data to write
 \param      count_p      Number of bytes to write
 \param      pWriteNum_p     Number of bytes written

 \return     - XST_SUCCESS if the controller writes correctly
 *           - XST_FAILURE if the controller fails to write

 \ingroup module_demo
 */
//------------------------------------------------------------------------------
#if !_FS_READONLY
INT sd_write(FIL* pFile_p, void* pBuffer_p, UINT count_p, UINT* pWriteNum_p)
{
    FRESULT    ret;

    ret = f_write(pFile_p, pBuffer_p, count_p, pWriteNum_p);
    if (ret)
    {
        printf("ERROR: {%s} failed %d\r\n", __func__, ret);
        return XST_FAILURE;
    }

    return XST_SUCCESS;
}

#endif /* _FS_READONLY */

//------------------------------------------------------------------------------
/**
 \brief  Get size of file

 This function returns the size of a file on SD FLASH

 \param      pFile_p        pointer to the FILE structure to the file to write

 \return     Size of File

 \ingroup module_demo
 */
//------------------------------------------------------------------------------
UINT sd_get_fsize(FIL* pFile_p)
{
    return f_size(pFile_p);
}

//------------------------------------------------------------------------------
/**
 \brief  Close file

 This function closes the file

 \param      pFile_p        pointer to the FILE structure to the file to write

 \return     Size of File

 \ingroup module_demo
 */
//------------------------------------------------------------------------------
void sd_close(FIL* pFile_p)
{
    f_close(pFile_p);
    return;
}

#endif // XPAR_PS7_SD_0_S_AXI_BASEADDR
