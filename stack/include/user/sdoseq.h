/**
********************************************************************************
\file   sdoseq.h

\brief  Definitions for SDO sequence layer module

The file contains definitions for the SDO sequence layer module.
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2013, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
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

#ifndef _INC_sdoseq_H_
#define _INC_sdoseq_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------

#include <sdo.h>
#include <user/sdoudp.h>
#include <user/sdoasnd.h>
#include <event.h>
#include <user/timeru.h>

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

tEplKernel sdoseq_init(tSdoComReceiveCb pfnSdoComRecvCb_p, tSdoComConCb pfnSdoComConCb_p);
tEplKernel sdoseq_addInstance (tSdoComReceiveCb pfnSdoComRecvCb_p, tSdoComConCb pfnSdoComConCb_p);
tEplKernel sdoseq_delInstance(void);
tEplKernel sdoseq_initCon(tSdoSeqConHdl* pSdoSeqConHdl_p, UINT nodeId_p, tSdoType sdoType_p);
tEplKernel sdoseq_sendData(tSdoSeqConHdl sdoSeqConHdl_p, UINT dataSize_p, tEplFrame* pData_p );
tEplKernel sdoseq_processEvent(tEplEvent* pEvent_p);
tEplKernel sdoseq_deleteCon(tSdoSeqConHdl sdoSeqConHdl_p);
tEplKernel sdoseq_setTimeout(UINT32 timeout_p);

#ifdef __cplusplus
}
#endif

#endif /* _INC_sdoseq_H_ */

