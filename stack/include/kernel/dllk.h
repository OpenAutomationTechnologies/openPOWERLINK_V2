/**
********************************************************************************
\file   dllk.h

\brief  Definitions for DLL kernel module

This file contains the definitions for the DLL kernel module.

*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2013, SYSTEC electronic GmbH
Copyright (c) 2013, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
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

#ifndef _INC_dllk_H_
#define _INC_dllk_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <oplk/oplkinc.h>
#include <oplk/dll.h>
#include <oplk/event.h>
#include <common/edrv.h>


//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// typedef
//------------------------------------------------------------------------------
typedef tOplkError (* tEplDllkCbAsync) (tFrameInfo * pFrameInfo_p);

typedef struct
{
    UINT8               aLocalMac[6];
    tEplHwParam         hwParam;
} tDllkInitParam;

// forward declaration
struct _tEdrvTxBuffer;

struct _tDllkNodeInfo
{
    UINT                        nodeId;
    UINT16                      presPayloadLimit;       // object 0x1F8D: NMT_PResPayloadLimitList_AU16
    UINT8                       presFilterFlags;
#if defined(CONFIG_INCLUDE_NMT_MN)
    UINT8                       aMacAddr[6];
    UINT8                       soaFlag1;
    BOOL                        fSoftDelete;            // delete node after error and ignore error
    UINT16                      preqPayloadLimit;       // object 0x1F8B: NMT_MNPReqPayloadLimitList_AU16
    tNmtState                   nmtState;
    ULONG                       dllErrorEvents;
    UINT32                      presTimeoutNs;          // object 0x1F92: NMT_MNCNPResTimeout_AU32
    struct sEdrvTxBuffer*       pPreqTxBuffer;
    struct _tDllkNodeInfo*      pNextNodeInfo;
#endif

};
typedef struct _tDllkNodeInfo tDllkNodeInfo;

typedef struct
{
    UINT32   syncControl;
    UINT32   pResTimeFirstNs;
    UINT32   pResTimeSecondNs;
    UINT32   syncMNDelayFirstNs;
    UINT32   syncMNDelaySecondNs;
} tDllkPrcCycleTiming;

// callback function for frame processing
typedef tOplkError (*tDllkCbProcessRpdo) (tFrameInfo * pFrameInfo_p);
typedef tOplkError (*tDllkCbProcessTpdo) (tFrameInfo * pFrameInfo_p, BOOL fReadyFlag_p);

typedef enum
{
    kDllGsInit           = 0x00,    // MN/CN: initialisation (< PreOp2)
    kDllCsWaitPreq       = 0x01,    // CN: wait for PReq frame
    kDllCsWaitSoc        = 0x02,    // CN: wait for SoC frame
    kDllCsWaitSoa        = 0x03,    // CN: wait for SoA frame
    kDllMsNonCyclic      = 0x04,    // MN: reduced EPL cycle (PreOp1)
    kDllMsWaitSocTrig    = 0x05,    // MN: wait for SoC trigger (cycle timer)
    kDllMsWaitPreqTrig   = 0x06,    // MN: wait for (first) PReq trigger (WaitSoCPReq_U32)
    kDllMsWaitPres       = 0x07,    // MN: wait for PRes frame from CN
    kDllMsWaitSoaTrig    = 0x08,    // MN: wait for SoA trigger (PRes transmitted)
    kDllMsWaitAsndTrig   = 0x09,    // MN: wait for ASnd trigger (SoA transmitted)
    kDllMsWaitAsnd       = 0x0A,    // MN: wait for ASnd frame if SoA contained invitation
} tDllState;

//------------------------------------------------------------------------------
// function prototypes
//------------------------------------------------------------------------------
#ifdef __cplusplus
extern "C" {
#endif

tOplkError dllk_addInstance(tDllkInitParam* pInitParam_p);
tOplkError dllk_delInstance(void);
tOplkError dllk_config(tDllConfigParam * pDllConfigParam_p);
tOplkError dllk_setIdentity(tDllIdentParam * pDllIdentParam_p);
tOplkError dllk_process(tEvent * pEvent_p) SECTION_DLLK_PROCESS;
tOplkError dllk_regAsyncHandler(tEplDllkCbAsync pfnDllkCbAsync_p);
tOplkError dllk_deregAsyncHandler(tEplDllkCbAsync pfnDllkCbAsync_p);
tOplkError dllk_setAsndServiceIdFilter(tDllAsndServiceId ServiceId_p, tDllAsndFilter Filter_p);
void       dllk_regRpdoHandler(tDllkCbProcessRpdo pfnDllkCbProcessRpdo_p);
void       dllk_regTpdoHandler(tDllkCbProcessTpdo pfnDllkCbProcessTpdo_p);
tSyncCb dllk_regSyncHandler(tSyncCb pfnCbSync_p);
#if DLL_DEFERRED_RXFRAME_RELEASE_ISOCHRONOUS != FALSE || DLL_DEFERRED_RXFRAME_RELEASE_ASYNCHRONOUS != FALSE
tOplkError dllk_releaseRxFrame(tPlkFrame* pFrame_p, UINT uiFrameSize_p);
#endif

#if EPL_NMT_MAX_NODE_ID > 0
tOplkError dllk_configNode(tDllNodeInfo* pNodeInfo_p);
tOplkError dllk_addNode(tDllNodeOpParam* pNodeOpParam_p);
tOplkError dllk_deleteNode(tDllNodeOpParam* pNodeOpParam_p);
#endif // EPL_NMT_MAX_NODE_ID > 0

#if defined(CONFIG_INCLUDE_NMT_MN)
tOplkError dllk_setFlag1OfNode(UINT nodeId_p, UINT8 soaFlag1_p);
void       dllk_getCurrentCnNodeIdList(BYTE** ppbCnNodeIdList_p);

#if EPL_DLL_PRES_CHAINING_MN != FALSE
tOplkError dllk_getCnMacAddress(UINT nodeId_p, UINT8* pCnMacAddress_p);
#endif

#endif

#ifdef __cplusplus
}
#endif

#endif  // #ifndef _INC_dllk_H_

