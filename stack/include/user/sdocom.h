/**
********************************************************************************
\file   user/sdocom.h

\brief  Definitions for SDO Command Layer module

The file contains definitions for the SDO Command Layer module.
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
#ifndef _INC_user_sdocom_H_
#define _INC_user_sdocom_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <common/oplkinc.h>
#include <oplk/sdo.h>

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// typedef
//------------------------------------------------------------------------------
/// Callback function pointer to inform application about connection
typedef tOplkError (*tSdoFinishedCb)(const tSdoComFinished* pSdoComFinished_p);

/**
\brief Structure for initializing Read/Write by Index SDO transfer

This structure is used to initialize a SDO transfer of a Read or Write
by Index command.
*/
typedef struct
{
    tSdoComConHdl       sdoComConHdl;           ///< Handle to SDO command layer connection
    UINT16              index;                  ///< Index to read/write
    UINT8               subindex;               ///< Sub-index to read/write
    void*               pData;                  ///< Pointer to data which should be transfered
    size_t              dataSize;               ///< Size of data to be transfered
    UINT                timeout;                ///< Timeout: not supported in this version of openPOWERLINK
    tSdoAccessType      sdoAccessType;          ///< The SDO access type (Read or Write) for this transfer
    tSdoFinishedCb      pfnSdoFinishedCb;       ///< Pointer to callback function which will be called when transfer is finished.
    tSdoMultiAccEntry*  paMultiAcc;             ///< Multiple object access array.
                                                /**< Provided array has to remain unchanged until the transfer has finished. */
    UINT                multiAccCnt;            ///< Count of multiple object access array elements aggregated in one transfered by local client
    void*               pMultiBuffer;           ///< Pointer to buffer for multiple object transfer frame storage.
                                                /**< Provided buffer has to remain unchanged until the transfer has finished. */
    size_t              multiBufSize;           ///< Buffer size for multiple object transfer frame storage.
    void*               pUserArg;               ///< User definable argument pointer
} tSdoComTransParamByIndex;

/**
\brief Structure for describing the SDO command layer function interface

This structure specifies the SDO command layer modules. It contains several command
layer implementations in parallel without naming conflicts. Inside the sdocom_init() function
the command layer implementations for the SDO stack defined in the init parameters are loaded.
*/
typedef struct
{
    tOplkError          (*pfnInit)(tComdLayerObdCb, tComdLayerObdCb);       ///< Init function pointer
    tOplkError          (*pfnExit)(void);                                   ///< Exit function pointer
#if defined(CONFIG_INCLUDE_SDOC)
    tOplkError          (*pfnDefineCon)(tSdoComConHdl*, UINT, tSdoType);    ///< Define Connection function pointer
    tOplkError          (*pfnTransByIdx)(const tSdoComTransParamByIndex*);  ///< Transfer by Index function pointer
    tOplkError          (*pfnDeleteCon)(tSdoComConHdl);                     ///< Delete Connection function pointer
    tOplkError          (*pfnGetState)(tSdoComConHdl, tSdoComFinished*);    ///< Get State function pointer
    UINT                (*pfnGetNodeId)(tSdoComConHdl);                     ///< Get Node Id function pointer
    tOplkError          (*pfnSdoAbort)(tSdoComConHdl, UINT32);              ///< SDO abort function pointer
#endif /* defined(CONFIG_INCLUDE_SDOC) */
} tSdoComFunctions;

//------------------------------------------------------------------------------
// function prototypes
//------------------------------------------------------------------------------
#ifdef __cplusplus
extern "C"
{
#endif

tOplkError sdocom_init(UINT stackType_p,
                       tComdLayerObdCb pfnObdWrite_p,
                       tComdLayerObdCb pfnObdRead_p);
tOplkError sdocom_exit(void);

#if defined(CONFIG_INCLUDE_SDOC)
tOplkError sdocom_defineConnection(tSdoComConHdl* pSdoComConHdl_p,
                                   UINT targetNodeId_p,
                                   tSdoType sdoType_p);
tOplkError sdocom_initTransferByIndex(const tSdoComTransParamByIndex* pSdoComTransParam_p);
UINT       sdocom_getNodeId(tSdoComConHdl sdoComConHdl_p);
tOplkError sdocom_undefineConnection(tSdoComConHdl sdoComConHdl_p);
tOplkError sdocom_getState(tSdoComConHdl sdoComConHdl_p,
                           tSdoComFinished* pSdoComFinished_p);
tOplkError sdocom_abortTransfer(tSdoComConHdl sdoComConHdl_p,
                                UINT32 abortCode_p);
#endif /* defined(CONFIG_INCLUDE_SDOC) */

#ifdef __cplusplus
}
#endif

#endif /* _INC_user_sdocom_H_ */
