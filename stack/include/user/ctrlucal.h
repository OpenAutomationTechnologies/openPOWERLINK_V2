/**
********************************************************************************
\file   user/ctrlucal.h

\brief  Definitions for user ctrl CAL module

This file contains the definitions for the user ctrl CAL module.

*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2016, B&R Industrial Automation GmbH
Copyright (c) 2017, Kalycito Infotech Private Limited
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
#ifndef _INC_user_ctrlucal_H_
#define _INC_user_ctrlucal_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <common/oplkinc.h>
#include <common/ctrl.h>

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// typedef
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// function prototypes
//------------------------------------------------------------------------------
#ifdef __cplusplus
extern "C"
{
#endif

tOplkError       ctrlucal_init(void);
void             ctrlucal_exit(void);
tOplkError       ctrlucal_process(void);
tOplkError       ctrlucal_executeCmd(tCtrlCmdType cmd_p,
                                     UINT16* pRetVal_p);
tOplkError       ctrlucal_checkKernelStack(void);
UINT16           ctrlucal_getStatus(void);
UINT16           ctrlucal_getHeartbeat(void);
void             ctrlucal_storeInitParam(const tCtrlInitParam* pInitParam_p);
tOplkError       ctrlucal_readInitParam(tCtrlInitParam* pInitParam_p);
tOplkError       ctrlucal_writeFileBuffer(const tOplkApiFileChunkDesc* pDesc_p,
                                          const void* pBuffer_p);
size_t           ctrlucal_getFileBufferSize(void);
OPLK_FILE_HANDLE ctrlucal_getFd(void);
tOplkError       ctrlucal_getMappedMem(size_t kernelOffs_p,
                                       size_t size_p,
                                       void** ppUserMem_p);
#ifdef __cplusplus
}
#endif

#endif /* _INC_user_ctrlucal_H_ */
