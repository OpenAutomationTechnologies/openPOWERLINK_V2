/**
********************************************************************************
\file   dllucal.h

\brief  Definitions for user DLL CAL module

This header file contains definitions for the user DLL CAL module.

Copyright (c) 2012, SYSTEC electronik GmbH
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
*******************************************************************************/

#ifndef _INC_dllucal_H_
#define _INC_dllucal_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <oplk/oplkinc.h>
#include <oplk/dll.h>
#include <oplk/event.h>

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// typedef
//------------------------------------------------------------------------------
typedef tOplkError (PUBLIC * tEplDlluCbAsnd) (tFrameInfo * pFrameInfo_p);

//------------------------------------------------------------------------------
// function prototypes
//------------------------------------------------------------------------------

#ifdef __cplusplus
extern "C" {
#endif

tOplkError dllucal_init(void);

tOplkError dllucal_exit(void);

tOplkError dllucal_config(tDllConfigParam * pDllConfigParam_p);

tOplkError dllucal_setIdentity(tDllIdentParam * pDllIdentParam_p);

tOplkError dllucal_regAsndService(tDllAsndServiceId ServiceId_p,
                                  tEplDlluCbAsnd pfnDlluCbAsnd_p,
                                  tDllAsndFilter Filter_p);

tOplkError dllucal_sendAsyncFrame(tFrameInfo * pFrameInfo, tDllAsyncReqPriority Priority_p);

tOplkError dllucal_process(tEvent * pEvent_p);


#if EPL_NMT_MAX_NODE_ID > 0

tOplkError dllucal_configNode(tDllNodeInfo* pNodeInfo_p);

tOplkError dllucal_addNode(tDllNodeOpParam* pNodeOpParam_p);

tOplkError dllucal_deleteNode(tDllNodeOpParam* pNodeOpParam_p);

#endif

#if defined(CONFIG_INCLUDE_NMT_MN)

tOplkError dllucal_issueRequest(tDllReqServiceId Service_p, unsigned int uiNodeId_p, BYTE bSoaFlag1_p);

#if EPL_DLL_PRES_CHAINING_MN != FALSE
tOplkError dllucal_issueSyncRequest(tDllSyncRequest* pSyncRequest_p, unsigned int uiSize_p);
#endif

#endif

#ifdef __cplusplus
}
#endif

#endif /* _INC_dllucal_H_ */

