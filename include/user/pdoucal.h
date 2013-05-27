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
#include <EplInc.h>
#include <pdo.h>
#include <event.h>

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

tEplKernel pdoucal_init(tEplSyncCb pfnSyncCb_p);
tEplKernel pdoucal_exit(void);

tEplKernel pdoucal_postPdokChannelAlloc(tPdoAllocationParam* pAllocationParam_p);
tEplKernel pdoucal_postConfigureChannel(tPdoChannelConf* pChannelConf_p);
tEplKernel pdoucal_postSetupPdoBuffers(size_t rxPdoMemSize_p, size_t txPdoMemSize_p);

// PDO memory functions
tEplKernel pdoucal_openMem(void);
tEplKernel pdoucal_closeMem(void);
tEplKernel pdoucal_allocateMem(size_t memSize_p, BYTE** pPdoMem_p);
tEplKernel pdoucal_freeMem(BYTE* pMem_p, size_t memSize_p);

//PDO buffer functions
tEplKernel pdoucal_initPdoMem(tPdoChannelSetup* pPdoChannels_p, size_t rxPdoMemSize_p,
                              size_t txPdoMemSize_p);
void       pdoucal_cleanupPdoMem(void);
BYTE*      pdoucal_getTxPdoAdrs(UINT channelId_p);
tEplKernel pdoucal_setTxPdo(UINT channelId_p, BYTE* pPdo_p,  WORD pdoSize_p);
tEplKernel pdoucal_getRxPdo(BYTE** ppPdo_p, UINT channelId_p, WORD pdoSize_p);

// PDO sync functions
tEplKernel pdoucal_initSync(tEplSyncCb pfnSyncCb_p);
void       pdoucal_exitSync(void);
tEplKernel pdoucal_waitSyncEvent(ULONG timeout_p);
tEplKernel pdoucal_callSyncCb(void);

#ifdef __cplusplus
}
#endif

#endif /* _INC_PdouCal_H_ */
