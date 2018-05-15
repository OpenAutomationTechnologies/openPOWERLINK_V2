/**
********************************************************************************
\file   user/pdou.h

\brief  Include file for user PDO module

*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2012, SYSTEC electronic GmbH
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
#ifndef _INC_user_pdou_H_
#define _INC_user_pdou_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <common/oplkinc.h>
#include <oplk/event.h>
#include <oplk/nmt.h>
#include <oplk/obd.h>

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// typedef
//------------------------------------------------------------------------------

/**
\brief Structure for a PDO change event

The structure contains the arguments for a PDO change event.
*/
typedef struct
{
    BOOL                fActivated;             ///< Flag indicating whether the PDO is activated
    BOOL                fTx;                    ///< Flag indicating whether the PDO is a TPDO
    UINT                nodeId;                 ///< Node ID for which the PDO has changed
    UINT                mappParamIndex;         ///< Object index of mapping parameter object
    UINT                mappObjectCount;        ///< Number of mapped objects
} tPdoEventPdoChange;

typedef tOplkError (*tPdoCbEventPdoChange)(const tPdoEventPdoChange* pEventPdoChange_p);

//------------------------------------------------------------------------------
// function prototypes
//------------------------------------------------------------------------------
#ifdef __cplusplus
extern "C"
{
#endif

tOplkError pdou_init(void);
tOplkError pdou_exit(void);

#if defined(CONFIG_INCLUDE_PDO)
OPLKDLLEXPORT tOplkError pdou_cbObdAccess(tObdCbParam* pParam_p);
#else
#define pdou_cbObdAccess NULL
#endif

tOplkError pdou_cbNmtStateChange(tEventNmtStateChange nmtStateChange_p);

tOplkError pdou_copyRxPdoToPi(void);
tOplkError pdou_copyTxPdoFromPi(void);
tOplkError pdou_registerEventPdoChangeCb(tPdoCbEventPdoChange pfnCbEventPdoChange_p);

#ifdef __cplusplus
}
#endif

#endif /* _INC_user_pdou_H_ */
