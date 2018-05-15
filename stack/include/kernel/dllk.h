/**
********************************************************************************
\file   kernel/dllk.h

\brief  Definitions for DLL kernel module

This file contains the definitions for the DLL kernel module.

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
#ifndef _INC_kernel_dllk_H_
#define _INC_kernel_dllk_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <common/oplkinc.h>
#include <kernel/edrv.h>
#include <oplk/dll.h>
#include <oplk/nmt.h>
#include <oplk/event.h>

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

// States for error signaling initialization on a MN
#define STATE_MN_ERRSIG_INIT_WAIT_EC1       0
#define STATE_MN_ERRSIG_INIT_WAIT_EC0       1
#define STATE_MN_ERRSIG_INIT_READY          2

#define ERRSIG_WAIT_EC0_TIMEOUT_CNT         10

//------------------------------------------------------------------------------
// typedef
//------------------------------------------------------------------------------
/**
\brief Type for DLLk asynchronous frame receive callback function pointer

This type defines a function pointer to the DLLk asynchronous frame received
callback function.

\param  pFrameInfo_p        Frame info of the received frame
\param  pReleaseRxBuffer_p  Pointer to buffer release flag. The function must
                            set this flag to determine if the RxBuffer could be
                            released immediately.

\return The function returns a tOplkError error code
*/
typedef tOplkError (*tDllkCbAsync)(tFrameInfo* pFrameInfo_p,
                                   tEdrvReleaseRxBuffer* pReleaseRxBuffer_p);

/**
\brief Structure defining node informations

This structure defines informations about a POWERLINK node in the network.
*/
struct _tDllkNodeInfo
{
    UINT                        nodeId;                 ///< Node ID
    UINT16                      presPayloadLimit;       ///< NMT_PResPayloadLimitList_AU16 (0x1F8D)
    UINT8                       presFilterFlags;        ///< PRes filter flags
#if defined(CONFIG_INCLUDE_NMT_MN)
    UINT8                       aMacAddr[6];            ///< Ethernet MAC address
    UINT8                       soaFlag1;               ///< SoA flag1 register
    BOOL                        fSoftDelete;            ///< Delete node after error and ignore error
    UINT16                      preqPayloadLimit;       ///< NMT_MNPReqPayloadLimitList_AU16 (0x1F8B)
    tNmtState                   nmtState;               ///< NMT state
    ULONG                       dllErrorEvents;         ///< DLL error events
    UINT32                      presTimeoutNs;          ///< NMT_MNCNPResTimeout_AU32 (0x1F92)
    struct sEdrvTxBuffer*       pPreqTxBuffer;          ///< PReq TX buffer list
    struct _tDllkNodeInfo*      pNextNodeInfo;          ///< Next node information structure
    UINT8                       errSigState;            ///< State of error signaling initialization state machine
    UINT8                       errSigReqCnt;           ///< Request counter for error signaling initialization
#endif /* defined(CONFIG_INCLUDE_NMT_MN) */
};
typedef struct _tDllkNodeInfo tDllkNodeInfo;

/**
\brief Structure defining the cycle timing in PRC mode

This structure defines the cycle timing in PollResponse Chaining mode.
*/
typedef struct
{
    UINT32   syncControl;                               ///< Sync control register
    UINT32   pResTimeFirstNs;                           ///< PRes time on the first communication path (ns)
    UINT32   pResTimeSecondNs;                          ///< PRes time on the second communication path (ns)
    UINT32   syncMNDelayFirstNs;                        ///< Sync delay of the MN on the first communication path (ns)
    UINT32   syncMNDelaySecondNs;                       ///< Sync delay of the MN on the second communication path (ns)
} tDllkPrcCycleTiming;

// callback function for frame processing
typedef tOplkError (*tDllkCbProcessRpdo)(const tFrameInfo* pFrameInfo_p);
typedef tOplkError (*tDllkCbProcessTpdo)(tFrameInfo* pFrameInfo_p, BOOL fReadyFlag_p);

/**
\brief Enum defining the DLL node states

This enumeration defines the DLL node states.
*/
typedef enum
{
    kDllGsInit           = 0x00,                        ///< MN/CN: initialization (< PreOp2)
    kDllCsWaitPreq       = 0x01,                        ///< CN: wait for PReq frame
    kDllCsWaitSoc        = 0x02,                        ///< CN: wait for SoC frame
    kDllCsWaitSoa        = 0x03,                        ///< CN: wait for SoA frame
    kDllMsNonCyclic      = 0x04,                        ///< MN: reduced POWERLINK cycle (PreOp1)
    kDllMsWaitSocTrig    = 0x05,                        ///< MN: wait for SoC trigger (cycle timer)
    kDllMsWaitPreqTrig   = 0x06,                        ///< MN: wait for (first) PReq trigger (WaitSoCPReq_U32)
    kDllMsWaitPres       = 0x07,                        ///< MN: wait for PRes frame from CN
    kDllMsWaitSoaTrig    = 0x08,                        ///< MN: wait for SoA trigger (PRes transmitted)
    kDllMsWaitAsndTrig   = 0x09,                        ///< MN: wait for ASnd trigger (SoA transmitted)
    kDllMsWaitAsnd       = 0x0A,                        ///< MN: wait for ASnd frame if SoA contained invitation
} eDllState;

/**
\brief DLL node state data type

Data type for the enumerator \ref eDllState.
*/
typedef UINT32 tDllState;

//------------------------------------------------------------------------------
// function prototypes
//------------------------------------------------------------------------------
#ifdef __cplusplus
extern "C"
{
#endif

tOplkError dllk_init(void);
tOplkError dllk_exit(void);
tOplkError dllk_config(const tDllConfigParam* pDllConfigParam_p);
tOplkError dllk_setIdentity(const tDllIdentParam* pDllIdentParam_p);
tOplkError dllk_regAsyncHandler(tDllkCbAsync pfnDllkCbAsync_p);
tOplkError dllk_deregAsyncHandler(tDllkCbAsync pfnDllkCbAsync_p);
tOplkError dllk_setAsndServiceIdFilter(tDllAsndServiceId ServiceId_p, tDllAsndFilter filter_p);
void       dllk_regRpdoHandler(tDllkCbProcessRpdo pfnDllkCbProcessRpdo_p);
void       dllk_regTpdoHandler(tDllkCbProcessTpdo pfnDllkCbProcessTpdo_p);
tSyncCb    dllk_regSyncHandler(tSyncCb pfnCbSync_p);

#if defined(CONFIG_INCLUDE_NMT_MN)
tOplkError dllk_cbCyclicError(tOplkError errorCode_p, const tEdrvTxBuffer* pTxBuffer_p);
#endif

#if ((CONFIG_DLL_DEFERRED_RXFRAME_RELEASE_SYNC != FALSE) || \
     (CONFIG_DLL_DEFERRED_RXFRAME_RELEASE_ASYNC != FALSE))
tOplkError dllk_releaseRxFrame(tPlkFrame* pFrame_p, UINT frameSize_p);
#endif

#if (NMT_MAX_NODE_ID > 0)
tOplkError dllk_configNode(const tDllNodeInfo* pNodeInfo_p);
tOplkError dllk_addNode(const tDllNodeOpParam* pNodeOpParam_p);
tOplkError dllk_deleteNode(const tDllNodeOpParam* pNodeOpParam_p);
#endif // NMT_MAX_NODE_ID > 0

#if defined(CONFIG_INCLUDE_NMT_MN)
tOplkError dllk_setFlag1OfNode(UINT nodeId_p, UINT8 soaFlag1_p);
void       dllk_getCurrentCnNodeIdList(UINT8** ppCnNodeIdList_p);
tOplkError dllk_getCnMacAddress(UINT nodeId_p, UINT8* pCnMacAddress_p);
#endif /* defined(CONFIG_INCLUDE_NMT_MN) */

// dllkevent.c
tOplkError dllk_process(const tEvent* pEvent_p) SECTION_DLLK_PROCESS;

#ifdef __cplusplus
}
#endif

#endif  /* _INC_kernel_dllk_H_ */
