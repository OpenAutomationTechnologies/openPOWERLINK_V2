/**
********************************************************************************
\file   user/ctrlu.h

\brief  Definitions for user control module

This file contains the definitions for the user control module.

*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2016, B&R Industrial Automation GmbH
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
#ifndef _INC_user_ctrlu_H_
#define _INC_user_ctrlu_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <common/oplkinc.h>
#include <oplk/obd.h>
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

tOplkError   ctrlu_init(void);
void         ctrlu_exit(void);
tOplkError   ctrlu_checkKernelStackInfo(void);
tOplkError   ctrlu_initStack(const tOplkApiInitParam* pInitParam_p);
tOplkError   ctrlu_shutdownStack(void);
tOplkError   ctrlu_processStack(void);
BOOL         ctrlu_checkKernelStack(void);
tOplkError   ctrlu_getKernelInfo(tCtrlKernelInfo* pKernelInfo_p);
tOplkError   ctrlu_callUserEventCallback(tOplkApiEventType eventType_p,
                                         const tOplkApiEventArg* pEventArg_p);
const UINT8* ctrlu_getEthMacAddr(void);
BOOL         ctrlu_stackIsInitialized(void);
UINT32       ctrlu_getFeatureFlags(void);
tOplkError   ctrlu_writeFileChunk(const tOplkApiFileChunkDesc* pDesc_p,
                                  const void* pBuffer_p);
size_t       ctrlu_getMaxFileChunkSize(void);

#ifdef __cplusplus
}
#endif

#endif /* _INC_user_ctrlu_H_ */
