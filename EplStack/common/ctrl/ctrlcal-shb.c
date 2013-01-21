/**
********************************************************************************
\file   ctrlcal-shb.c

\brief  Shared buffer implementation for control CAL module

The file contains a shared buffer implementation which can be used by the
memory block control CAL modules.

\ingroup module_ctrl
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2012, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
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
#include <unistd.h>
#include <stddef.h>
#include <Epl.h>
#include <SharedBuff.h>

//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#define CTRL_SHB_ID     "EplCtrl"

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
static tShbInstance    ctrlBufShbInst_l;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief    Initialize control CAL module

The function initializes the control CAL module.

\param  size_p      The size of the memory control block.

\return The function returns a tEplKernel error code.

\ingroup module_ctrlcal
*/
//------------------------------------------------------------------------------
tEplKernel ctrlcal_init(UINT size_p)
{
    tShbError       shbError;
    UINT            fBufCreated;
    char*           pTemp;

    /* Allocate shared buffer for control module */
    shbError = ShbLinAllocBuffer (size_p, CTRL_SHB_ID, &ctrlBufShbInst_l, &fBufCreated);
    if (shbError != kShbOk)
    {
        EPL_DBGLVL_ERROR_TRACE ("Couldn't allocate control buffer!\n");
        return kEplNoResource;
    }

    if (fBufCreated)
    {
        /* Reset buffer */
        if ((pTemp = EPL_MALLOC(size_p)) != NULL)
        {
            EPL_MEMSET(pTemp, 0, size_p);
            ShbLinWriteDataBlock(ctrlBufShbInst_l, 0, pTemp, size_p);
            EPL_FREE(pTemp);
        }
    }

    return kEplSuccessful;
}

//------------------------------------------------------------------------------
/**
\brief    Cleanup control module

The function cleans up the control CAL module.

\return The function returns a tEplKernel error code.

\ingroup module_ctrlcal
*/
//------------------------------------------------------------------------------
tEplKernel ctrlcal_exit (void)
{
    tShbError       shbError;
    tEplKernel      ret = kEplSuccessful;

    if (ctrlBufShbInst_l != NULL)
    {
        shbError = ShbLinReleaseBuffer(ctrlBufShbInst_l);
        if (shbError != kShbOk)
        {
            ret = kEplGeneralError;
        }
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief Write data to control block

The function writes data to the control block.

\param  offset_p            Offset in memory block to store the data.
\param  pSrc_p              Pointer to the data which should be stored.
\param  length_p            The length of the data to be stored.

\ingroup module_ctrlcal
*/
//------------------------------------------------------------------------------
void ctrlcal_writeData(UINT offset_p, void* pSrc_p, size_t length_p)
{
    tShbError       shbError;

    if (ctrlBufShbInst_l == NULL)
    {
        EPL_DBGLVL_ERROR_TRACE ("%s() instance == NULL!\n", __func__);
        return;
    }

    shbError = ShbLinWriteDataBlock(ctrlBufShbInst_l, offset_p, pSrc_p, length_p);
    if (shbError != kShbOk)
    {
        EPL_DBGLVL_ERROR_TRACE ("%s() ShbLinWriteDataBlock failed! (%d)\n",
                                __func__, shbError);
    }
}

//------------------------------------------------------------------------------
/**
\brief Read data from control block

The function reads data from the control block.

\param  pDest_p             Pointer to store the read data.
\param  offset_p            Offset in memory block from which to read.
\param  length_p            The length of the data to be read.

\return The function returns a tEplKernel error code.

\ingroup module_ctrlcal
*/
//------------------------------------------------------------------------------
tEplKernel ctrlcal_readData(void* pDest_p, UINT offset_p, size_t length_p)
{
    tShbError       shbError;
    tEplKernel      ret = kEplSuccessful;

    if (ctrlBufShbInst_l == NULL)
    {
        EPL_DBGLVL_ERROR_TRACE ("%s() instance == NULL!\n", __func__);
        return kEplGeneralError;
    }

    shbError = ShbLinReadDataBlock(ctrlBufShbInst_l, pDest_p, offset_p, length_p);
    if (shbError != kShbOk)
    {
        EPL_DBGLVL_ERROR_TRACE ("%s() ShbLinReadDataBlock failed! (%d)\n",
                                __func__, shbError);
        ret = kEplGeneralError;
    }
    return ret;
}

