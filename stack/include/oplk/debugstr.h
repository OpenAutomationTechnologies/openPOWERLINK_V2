/**
********************************************************************************
\file   oplk/debugstr.h

\brief  Definitions for debug-string module

This file contains the definitions for the debug-string module.

*******************************************************************************/

/*------------------------------------------------------------------------------
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
#ifndef _INC_oplk_debugstr_H_
#define _INC_oplk_debugstr_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <oplk/oplk.h>
#include <oplk/event.h>
#include <oplk/nmt.h>

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

OPLKDLLEXPORT const char* debugstr_getNmtEventStr(tNmtEvent nmtEvent_p);
OPLKDLLEXPORT const char* debugstr_getEventTypeStr(tEventType eventType_p);
OPLKDLLEXPORT const char* debugstr_getEventSourceStr(tEventSource eventSrc_p);
OPLKDLLEXPORT const char* debugstr_getEventSinkStr(tEventSink eventSink_p);
OPLKDLLEXPORT const char* debugstr_getNmtStateStr(tNmtState nmtState_p);
OPLKDLLEXPORT const char* debugstr_getApiEventStr(tOplkApiEventType apiEvent_p);
OPLKDLLEXPORT const char* debugstr_getNmtNodeEventTypeStr(tNmtNodeEvent nodeEventType_p);
OPLKDLLEXPORT const char* debugstr_getNmtBootEventTypeStr(tNmtBootEvent bootEventType_p);
OPLKDLLEXPORT const char* debugstr_getNmtNodeCommandTypeStr(tNmtNodeCommand nodeCommand_p);
OPLKDLLEXPORT const char* debugstr_getSdoComConStateStr(tSdoComConState sdoComConState_p);
OPLKDLLEXPORT const char* debugstr_getRetValStr(tOplkError oplkError_p);
OPLKDLLEXPORT const char* debugstr_getEmergErrCodeStr(UINT16 emergErrCode_p);
OPLKDLLEXPORT const char* debugstr_getAbortCodeStr(UINT32 abortCode_p);

#ifdef __cplusplus
}
#endif

#endif /* _INC_oplk_debugstr_H_ */
