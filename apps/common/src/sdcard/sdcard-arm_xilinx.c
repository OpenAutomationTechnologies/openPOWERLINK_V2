/**
********************************************************************************
\file   sdcard-arm_xilinx.c

\brief  SDCARD support routine for reading CDC file.

Target specific SDACAR file system interface routine to read CDC file from a
SDCARD.

\ingroup module_app_common
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

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <oplk/oplk.h>
#include <fs_sdcard.h>
#include <xstatus.h>

#include "sdcard.h"

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

//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief    Read CDC from SD

This function is used to read CDC file from a SD CARD

\return The function returns a tEplKernel error code.

\ingroup module_demo
*/
//------------------------------------------------------------------------------
tOplkError sdcard_getCdcOnSd(char* pszCdcFilename_p, tCdcBuffInfo* pCdcBuffInfo_p)
{
    tOplkError      ret = kErrorOk;
    INT             result = 0;
    UINT            cdcSize;
    UINT            readSize;
    FIL             file;

    /* Flush the Caches */
    Xil_DCacheFlush();

    /* Disable Data Cache */
    Xil_DCacheDisable();

    result = sd_fs_init();
    if (result != XST_SUCCESS)
    {
        // error occurred
        PRINTF("%s Error Initializing SD Card \n", __func__);
        ret = kErrorNoResource;
        goto Exit;
    }

    result = sd_open(&file, pszCdcFilename_p, FA_READ);
    if (result != XST_SUCCESS)
    {
        // error occurred
        PRINTF("%s Error opening file \n", __func__);
        ret = kErrorNoResource;
        goto Exit;
    }
    cdcSize = sd_get_fsize(&file);
    pCdcBuffInfo_p->cdcSize = cdcSize;
    PRINTF("CDC file size %d\n", pCdcBuffInfo_p->cdcSize);

    pCdcBuffInfo_p->pCdcBuffer = malloc(pCdcBuffInfo_p->cdcSize);
    if (pCdcBuffInfo_p->pCdcBuffer == NULL)
    {
        PRINTF("Memory Allocation failed for CDC\n");
        ret = kErrorNoResource;
        goto Exit;
    }

    result = sd_read(&file, pCdcBuffInfo_p->pCdcBuffer, cdcSize, &readSize);
    if (readSize != cdcSize || result != XST_SUCCESS)
    {
        PRINTF("%s CDC Read failed \n", __func__);
        ret = kErrorNoResource;
        goto Exit;
    }

    sd_close(&file);

Exit:
    Xil_DCacheEnable();
    return ret;
}


//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{


///\}

