/**
********************************************************************************
\file   dllkframe.h

\brief  Definitions for the frame processing functions of the DLL kernel module

This file contains the definitions for the frame processing functions of the
DLL kernel module.

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
#ifndef _INC_dllkframe_H_
#define _INC_dllkframe_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <common/oplkinc.h>
#include <kernel/edrv.h>
#include <kernel/dllk.h>
#include <common/timer.h>
#include <oplk/dll.h>
#include <oplk/nmt.h>
#include <oplk/frame.h>

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

tEdrvReleaseRxBuffer dllkframe_processFrameReceived(tEdrvRxBuffer* pRxBuffer_p)
                     SECTION_DLLK_FRAME_RCVD_CB;

void       dllkframe_processTransmittedNmtReq(tEdrvTxBuffer* pTxBuffer_p)
           SECTION_DLLK_PROCESS_TX_NMT;
void       dllkframe_processTransmittedNonPlk(tEdrvTxBuffer* pTxBuffer_p)
           SECTION_DLLK_PROCESS_TX_NPLK;

tOplkError dllkframe_updateFrameIdentRes(tEdrvTxBuffer* pTxBuffer_p, tNmtState nmtState_p);
tOplkError dllkframe_updateFrameStatusRes(tEdrvTxBuffer* pTxBuffer_p, tNmtState NmtState_p);
tOplkError dllkframe_updateFrameAsyncRes(tNmtState nmtState_p);
tOplkError dllkframe_updateFramePres(tEdrvTxBuffer* pTxBuffer_p, tNmtState nmtState_p);
tOplkError dllkframe_checkFrame(tPlkFrame* pFrame_p, size_t frameSize_p);
tOplkError dllkframe_createTxFrame(UINT* pHandle_p,
                                   size_t* pFrameSize_p,
                                   tMsgType msgType_p,
                                   tDllAsndServiceId serviceId_p);
tOplkError dllkframe_deleteTxFrame(UINT handle_p);
tOplkError dllkframe_processTpdo(tFrameInfo* pFrameInfo_p, BOOL fReadyFlag_p);

#if defined(CONFIG_INCLUDE_NMT_MN)
void       dllkframe_processTransmittedSoc(tEdrvTxBuffer* pTxBuffer_p)
           SECTION_DLLK_PROCESS_TX_SOC;
void       dllkframe_processTransmittedSoa(tEdrvTxBuffer* pTxBuffer_p)
           SECTION_DLLK_PROCESS_TX_SOA;
tOplkError dllkframe_mnSendSoa(tNmtState nmtState_p,
                               tDllState* pDllStateProposed_p,
                               BOOL fEnableInvitation_p);
tOplkError dllkframe_updateFrameSoa(tEdrvTxBuffer* pTxBuffer_p,
                                    tNmtState NmtState_p,
                                    BOOL fEnableInvitation_p,
                                    UINT8 curReq_p)
           SECTION_DLLK_FRAME_UPDATE_SOA;
tOplkError dllkframe_asyncFrameNotReceived(tDllReqServiceId reqServiceId_p,
                                           UINT nodeId_p)
           SECTION_DLLK_FRAME_ASYNC_NRX;

/* Cycle/Sync Callback functions */
tOplkError dllkframe_cbMnTimerCycle(const tTimerEventArg* pEventArg_p);
#endif

/* PRes Chaining functions */
#if (CONFIG_DLL_PRES_CHAINING_CN != FALSE)
tOplkError dllkframe_presChainingEnable(void);
tOplkError dllkframe_presChainingDisable(void);
#endif

#ifdef __cplusplus
}
#endif

#endif  /* _INC_dllkframe_H_ */
