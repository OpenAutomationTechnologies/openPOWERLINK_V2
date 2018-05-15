/**
********************************************************************************
\file   kernel/ctrlkcal.h

\brief  Definitions for kernel ctrl CAL module

This file contains the definitions for the kernel ctrl CAL module.

*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2017, B&R Industrial Automation GmbH
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
#ifndef _INC_kernel_ctrlkcal_H_
#define _INC_kernel_ctrlkcal_H_

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

tOplkError        ctrlkcal_init(void);
void              ctrlkcal_exit(void);
tOplkError        ctrlkcal_process(void);
tOplkError        ctrlkcal_getCmd(tCtrlCmdType* pCmd_p);
void              ctrlkcal_sendReturn(UINT16 retval_p);
void              ctrlkcal_setStatus(tCtrlKernelStatus status_p);
tCtrlKernelStatus ctrlkcal_getStatus(void);
void              ctrlkcal_updateHeartbeat(UINT16 heartbeat_p);
tOplkError        ctrlkcal_readInitParam(tCtrlInitParam* pInitParam_p);
void              ctrlkcal_storeInitParam(const tCtrlInitParam* pInitParam_p);
tOplkError        ctrlkcal_readFileChunk(tOplkApiFileChunkDesc* pDesc_p,
                                         size_t bufferSize_p,
                                         void* pBuffer_p);
size_t            ctrlkcal_getMaxFileChunkSize(void);

#ifdef __cplusplus
}
#endif

#endif /* _INC_kernel_ctrlkcal_H_ */
