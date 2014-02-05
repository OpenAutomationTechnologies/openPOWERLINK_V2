/**
********************************************************************************
\file   pdoucal.h

\brief  include file for user PDO Communication Abstraction Layer module

*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2012, SYSTEC electronic GmbH
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
------------------------------------------------------------------------------*/

#ifndef _INC_pdoucal_H_
#define _INC_pdoucal_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <oplk/oplkinc.h>
#include <common/pdo.h>
#include <oplk/event.h>

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
extern "C" {
#endif

tOplkError pdoucal_init(tEplSyncCb pfnSyncCb_p);
tOplkError pdoucal_exit(void);

tOplkError pdoucal_postPdokChannelAlloc(tPdoAllocationParam* pAllocationParam_p);
tOplkError pdoucal_postConfigureChannel(tPdoChannelConf* pChannelConf_p);
tOplkError pdoucal_postSetupPdoBuffers(size_t rxPdoMemSize_p, size_t txPdoMemSize_p);

// PDO memory functions
tOplkError pdoucal_openMem(void);
tOplkError pdoucal_closeMem(void);
tOplkError pdoucal_allocateMem(size_t memSize_p, BYTE** pPdoMem_p);
tOplkError pdoucal_freeMem(BYTE* pMem_p, size_t memSize_p);

//PDO buffer functions
tOplkError pdoucal_initPdoMem(tPdoChannelSetup* pPdoChannels_p, size_t rxPdoMemSize_p,
                              size_t txPdoMemSize_p);
void       pdoucal_cleanupPdoMem(void);
BYTE*      pdoucal_getTxPdoAdrs(UINT channelId_p);
tOplkError pdoucal_setTxPdo(UINT channelId_p, BYTE* pPdo_p,  WORD pdoSize_p);
tOplkError pdoucal_getRxPdo(BYTE** ppPdo_p, UINT channelId_p, WORD pdoSize_p);

// PDO sync functions
tOplkError pdoucal_initSync(tEplSyncCb pfnSyncCb_p);
void       pdoucal_exitSync(void);
tOplkError pdoucal_waitSyncEvent(ULONG timeout_p);
tOplkError pdoucal_callSyncCb(void);

#ifdef __cplusplus
}
#endif

#endif /* _INC_PdouCal_H_ */
