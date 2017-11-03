/**
********************************************************************************
\file   user/sdocomclt.h

\brief  Definitions for SDO Command Layer client functions

The file contains definitions for the SDO Command Layer client module.
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2017, B&R Industrial Automation GmbH
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

#ifndef _INC_user_sdocomclt_H_
#define _INC_user_sdocomclt_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <user/sdocomint.h>

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// typedef
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// function prototypes
//------------------------------------------------------------------------------

tOplkError sdocomclt_defineConnection(tSdoComConHdl* pSdoComConHdl_p,
                                      UINT targetNodeId_p,
                                      tSdoType protType_p);
tOplkError sdocomclt_initTransferByIndex(const tSdoComTransParamByIndex* pSdoComTransParam_p);
tOplkError sdocomclt_undefineConnection(tSdoComConHdl sdoComConHdl_p);
tOplkError sdocomclt_getState(tSdoComConHdl sdoComConHdl_p,
                              tSdoComFinished* pSdoComFinished_p);
UINT       sdocomclt_getNodeId(tSdoComConHdl sdoComConHdl_p);
tOplkError sdocomclt_abortTransfer(tSdoComConHdl sdoComConHdl_p,
                                   UINT32 abortCode_p);
tOplkError sdocomclt_processStateWaitInit(tSdoComConHdl sdoComConHdl_p,
                                          tSdoComConEvent sdoComConEvent_p,
                                          const tAsySdoCom* pRecvdCmdLayer_p);
tOplkError sdocomclt_processStateConnected(tSdoComConHdl sdoComConHdl_p,
                                           tSdoComConEvent sdoComConEvent_p,
                                           const tAsySdoCom* pRecvdCmdLayer_p);
tOplkError sdocomclt_processStateSegmTransfer(tSdoComConHdl sdoComConHdl_p,
                                              tSdoComConEvent sdoComConEvent_p,
                                              const tAsySdoCom* pRecvdCmdLayer_p);

#endif /* _INC_user_sdocomclt_H_ */
