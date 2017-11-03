/**
********************************************************************************
\file   user/nmtmnu.h

\brief  Definitions for nmtmnu module

This file contains the definitions for the nmtmnu module.

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
#ifndef _INC_user_nmtmnu_H_
#define _INC_user_nmtmnu_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <common/oplkinc.h>
#include <common/nmt.h>
#include <user/nmtu.h>
#include <oplk/event.h>

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// typedef
//------------------------------------------------------------------------------
#if defined(CONFIG_INCLUDE_NMT_MN)
typedef tOplkError (*tNmtMnuCbNodeEvent)(UINT nodeId_p,
                                         tNmtNodeEvent nodeEvent_p,
                                         tNmtState nmtState_p,
                                         UINT16 errorCode_p,
                                         BOOL fMandatory_p);
typedef tOplkError (*tNmtMnuCbBootEvent)(tNmtBootEvent bootEvent_p,
                                         tNmtState nmtState_p,
                                         UINT16 errorCode_p);

/**
* \brief NMT MN configuration parameter structure
*
* This structure defines MN configuration parameters.
*/
typedef struct
{
    UINT32   prcPResTimeFirstCorrectionNs;      ///< First correction time of a PRes in PRC mode (in ns)
    UINT32   prcPResTimeFirstNegOffsetNs;       ///< First negative offset of a PRes in PRC mode (in ns)
} tNmtMnuConfigParam;
#endif

//------------------------------------------------------------------------------
// function prototypes
//------------------------------------------------------------------------------
#ifdef __cplusplus
extern "C"
{
#endif

#if defined(CONFIG_INCLUDE_NMT_MN)
tOplkError nmtmnu_init(tNmtMnuCbNodeEvent pfnCbNodeEvent_p,
                       tNmtMnuCbBootEvent pfnCbBootEvent_p);
tOplkError nmtmnu_exit(void);
tOplkError nmtmnu_processEvent(const tEvent* pEvent_p);
tOplkError nmtmnu_sendNmtCommand(UINT nodeId_p,
                                 tNmtCommand nmtCommand_p);
tOplkError nmtmnu_sendNmtCommandEx(UINT nodeId_p,
                                   tNmtCommand nmtCommand_p,
                                   const void* pNmtCommandData_p,
                                   size_t dataSize_p);
tOplkError nmtmnu_requestNmtCommand(UINT nodeId_p,
                                    tNmtCommand nmtCommand_p,
                                    const void* pNmtCommandData_p,
                                    size_t dataSize_p);
tOplkError nmtmnu_triggerStateChange(UINT nodeId_p,
                                     tNmtNodeCommand nodeCommand_p);
tOplkError nmtmnu_cbNmtStateChange(tEventNmtStateChange nmtStateChange_p);
tOplkError nmtmnu_getDiagnosticInfo(UINT* pMandatorySlaveCount_p,
                                    UINT* pSignalSlaveCount_p,
                                    UINT16* pflags_p);
tOplkError nmtmnu_configPrc(const tNmtMnuConfigParam* pConfigParam_p);
#endif

#ifdef __cplusplus
}
#endif

#endif /* _INC_user_nmtmnu_H_ */
