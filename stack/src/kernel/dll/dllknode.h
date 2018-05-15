/**
********************************************************************************
\file   dllknode.h

\brief  Definitions for the DLL kernel node functions

This file contains the definitions for the DLL kernel node functions.
It is part of the DLL kernel module.

*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2013, SYSTEC electronic GmbH
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
#ifndef _INC_dllknode_H_
#define _INC_dllknode_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <common/oplkinc.h>
#include <kernel/dllk.h>
#include <oplk/nmt.h>

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

tOplkError dllknode_cleanupLocalNode(tNmtState oldNmtState_p);
tOplkError dllknode_setupLocalNode(tNmtState nmtState_p);

#if defined(CONFIG_INCLUDE_NMT_MN)
tOplkError dllknode_addNodeIsochronous(tDllkNodeInfo* pIntNodeInfo_p);
tOplkError dllknode_deleteNodeIsochronous(const tDllkNodeInfo* pIntNodeInfo_p);
tOplkError dllknode_setupAsyncPhase(tNmtState nmtState_p,
                                    UINT nextTxBufferOffset_p,
                                    UINT32 nextTimeOffsetNs_p,
                                    UINT* pIndex_p)
                                    SECTION_DLLK_PROCESS_SYNC;
tOplkError dllknode_setupSyncPhase(tNmtState nmtState_p,
                                   BOOL fReadyFlag_p,
                                   UINT nextTxBufferOffset_p,
                                   UINT32* pNextTimeOffsetNs_p,
                                   UINT* pIndex_p)
                                   SECTION_DLLK_PROCESS_SYNC;
tOplkError dllknode_issueLossOfPres(UINT nodeId_p);
#endif

#if (NMT_MAX_NODE_ID > 0)
tDllkNodeInfo* dllknode_getNodeInfo(UINT nodeId_p) SECTION_DLLK_GETNODEINFO;
tOplkError     dllknode_addNodeFilter(tDllkNodeInfo* pIntNodeInfo_p,
                                      tDllNodeOpType nodeOpType_p,
                                      BOOL fUpdateEdrv_p);
tOplkError     dllknode_deleteNodeFilter(tDllkNodeInfo* pIntNodeInfo_p,
                                         tDllNodeOpType nodeOpType_p,
                                         BOOL fUpdateEdrv_p);
#endif

#ifdef __cplusplus
}
#endif

#endif  /* _INC_dllknode_H_ */
