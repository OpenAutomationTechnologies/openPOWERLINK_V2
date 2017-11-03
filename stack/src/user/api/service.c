/**
********************************************************************************
\file   service.c

\brief  Service module API

This file contains the implementation of the service module.

\ingroup module_service
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2016, B&R Industrial Automation GmbH
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

#include <user/ctrlu.h>
#include <user/ctrlucal.h>

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
\brief  Write file chunk

The function forwards the given file chunk to the kernel stack. The caller has to
segment the file into chunks of a maximum size which can be obtained from
\ref oplk_serviceGetFileChunkSize. The chunk descriptor is used by the kernel stack
to reassemble the file.

\param[in]      pDesc_p             File chunk descriptor
\param[in]      pChunkData_p        File chunk data buffer

\return The function returns a \ref tOplkError error code.

\ingroup module_service
*/
//------------------------------------------------------------------------------
tOplkError oplk_serviceWriteFileChunk(const tOplkApiFileChunkDesc* pDesc_p,
                                      const void* pChunkData_p)
{
    if ((pDesc_p == NULL) || (pChunkData_p == NULL))
        return kErrorApiInvalidParam;

    return ctrlu_writeFileChunk(pDesc_p, pChunkData_p);
}

//------------------------------------------------------------------------------
/**
\brief  Get supported file chunk size

The function obtains the supported file chunk size which must not be exceeded
when calling \ref oplk_serviceWriteFileChunk.

\return The function returns the supported file chunk size in bytes.
\retval 0                           This size indicates that no file transfer support
                                    is available in the used stack library.

\ingroup module_service
*/
//------------------------------------------------------------------------------
size_t oplk_serviceGetFileChunkSize(void)
{
    return ctrlu_getMaxFileChunkSize();
}

//------------------------------------------------------------------------------
/**
\brief  Execute firmware reconfiguration

The function executes a firmware reconfiguration of the kernel stack.

\note   Make sure to shut down the stack with \ref oplk_exit after triggering
        the firmware reconfiguration.

\param[in]      fFactory_p          Determines if the firmware shall reconfigure
                                    to factory image/configuration.

\return The function returns a \ref tOplkError error code.

\ingroup module_service
*/
//------------------------------------------------------------------------------
tOplkError oplk_serviceExecFirmwareReconfig(BOOL fFactory_p)
{
    tOplkError      ret;
    UINT16          retval;
    tCtrlCmdType    cmd;

    cmd = (fFactory_p) ? kCtrlReconfigFactoryImage : kCtrlReconfigUpdateImage;

    ret = ctrlucal_executeCmd(cmd, &retval);
    if (ret != kErrorOk)
    {
        DEBUG_LVL_ERROR_TRACE("%s(): Reconfigure to %s image failed with 0x%X\n",
                              __func__,
                              (fFactory_p) ? "factory" : "update",
                              ret);
        return ret;
    }

    if ((tOplkError)retval != kErrorOk)
    {
        DEBUG_LVL_ERROR_TRACE("%s(): Reconfigure to %s image rejected by kernel stack (0x%X)!\n",
                              __func__,
                              (fFactory_p) ? "factory" : "update",
                              (tOplkError)retval);
        return (tOplkError)retval;
    }

    return kErrorOk;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

/// \}
