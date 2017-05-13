/**
********************************************************************************
\file   user/pdoucal.h

\brief  include file for user PDO Communication Abstraction Layer module

*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2012, SYSTEC electronic GmbH
Copyright (c) 2016, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
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
#ifndef _INC_user_pdoucal_H_
#define _INC_user_pdoucal_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <common/oplkinc.h>
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
extern "C"
{
#endif

tOplkError pdoucal_init(void);
tOplkError pdoucal_exit(void);

tOplkError pdoucal_postPdokChannelAlloc(const tPdoAllocationParam* pAllocationParam_p);
tOplkError pdoucal_postConfigureChannel(const tPdoChannelConf* pChannelConf_p);
tOplkError pdoucal_postSetupPdoBuffers(size_t rxPdoMemSize_p,
                                       size_t txPdoMemSize_p);

// PDO memory functions
tOplkError pdoucal_openMem(void);
tOplkError pdoucal_closeMem(void);
tOplkError pdoucal_allocateMem(size_t memSize_p,
                               UINT8** pPdoMem_p);
tOplkError pdoucal_freeMem(UINT8* pMem_p,
                           size_t memSize_p);

//PDO buffer functions
tOplkError pdoucal_initPdoMem(const tPdoChannelSetup* pPdoChannels_p,
                              size_t rxPdoMemSize_p,
                              size_t txPdoMemSize_p);
void       pdoucal_cleanupPdoMem(void);
UINT8*     pdoucal_getTxPdoAdrs(UINT channelId_p);
tOplkError pdoucal_setTxPdo(UINT channelId_p,
                            UINT8* pPdo_p,
                            WORD pdoSize_p);
tOplkError pdoucal_getRxPdo(UINT8** ppPdo_p,
                            UINT channelId_p,
                            WORD pdoSize_p);

#ifdef __cplusplus
}
#endif

#endif /* _INC_user_pdoucal_H_ */
