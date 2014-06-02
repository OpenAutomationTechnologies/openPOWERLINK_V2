/**
********************************************************************************
\file   pdou.h

\brief  Include file for user PDO module

*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2012, SYSTEC electronic GmbH
Copyright (c) 2014, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
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

#ifndef _INC_pdou_H_
#define _INC_pdou_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <common/oplkinc.h>
#include <common/pdo.h>
#include <oplk/nmt.h>

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// typedef
//------------------------------------------------------------------------------

typedef struct
{
    BOOL                fActivated;
    BOOL                fTx;
    UINT                nodeId;
    UINT                mappParamIndex;
    UINT                mappObjectCount;
} tPdoEventPdoChange;

typedef tOplkError (*tPdoCbEventPdoChange)(tPdoEventPdoChange* pEventPdoChange_p);

//------------------------------------------------------------------------------
// function prototypes
//------------------------------------------------------------------------------

#ifdef __cplusplus
extern "C" {
#endif

tOplkError pdou_init(tSyncCb pfnSyncCb_p);

tOplkError pdou_exit(void);

#if defined(CONFIG_INCLUDE_PDO)
OPLKDLLEXPORT tOplkError pdou_cbObdAccess(tObdCbParam MEM* pParam_p);
#else
#define pdou_cbObdAccess        NULL
#endif

tOplkError pdou_cbNmtStateChange(tEventNmtStateChange NmtStateChange_p);

tOplkError pdou_copyRxPdoToPi (void);
tOplkError pdou_copyTxPdoFromPi (void);
tOplkError pdou_registerEventPdoChangeCb(tPdoCbEventPdoChange pfnCbEventPdoChange_p);

#ifdef __cplusplus
}
#endif

#endif /* _INC_pdou_H_ */
