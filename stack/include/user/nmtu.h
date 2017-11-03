/**
********************************************************************************
\file   user/nmtu.h

\brief  Definitions for nmtu module

This file contains the definitions for the nmtu module.

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
#ifndef _INC_user_nmtu_H_
#define _INC_user_nmtu_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <common/oplkinc.h>
#include <oplk/nmt.h>
#include <oplk/event.h>

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#define NMT_PLAIN_COMMAND_START     0x20        // Start of plain NMT command range
#define NMT_PLAIN_COMMAND_END       0x3F        // End of plain NMT command range

#define NMT_EXT_COMMAND_START       0x40        // Start of extended NMT command range
#define NMT_EXT_COMMAND_END         0x5F        // End of extended NMT command range

#define C_MAX_NMT_CMD_DATA_SIZE     (C_DLL_MAX_PAYL_OFFSET - 6)

//------------------------------------------------------------------------------
// typedef
//------------------------------------------------------------------------------
typedef tOplkError (*tNmtuStateChangeCallback)(tEventNmtStateChange nmtStateChange_p);
typedef tOplkError (*tNmtuCheckEventCallback)(tNmtEvent nmtEvent_p);

//------------------------------------------------------------------------------
// function prototypes
//------------------------------------------------------------------------------
#ifdef __cplusplus
extern "C"
{
#endif

tOplkError nmtu_init(void);
tOplkError nmtu_exit(void);
tOplkError nmtu_postNmtEvent(tNmtEvent nmtEvent_p);
tNmtState  nmtu_getNmtState(void);
tOplkError nmtu_processEvent(const tEvent* pEvent_p);
tOplkError nmtu_registerStateChangeCb(tNmtuStateChangeCallback pfnNmtStateChangeCb_p);

#ifdef __cplusplus
}
#endif

#endif  /* _INC_user_nmtu_H_ */
