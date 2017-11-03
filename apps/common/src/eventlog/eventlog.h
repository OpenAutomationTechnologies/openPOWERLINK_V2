/**
********************************************************************************
\file   eventlog.h

\brief  Definitions for event log module

This file contains definitions for the eventlog module.
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
#ifndef _INC_eventlog_H_
#define _INC_eventlog_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <oplk/oplk.h>
#include <oplk/nmt.h>
#include "eventlogstring.h"

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// typedef
//------------------------------------------------------------------------------
typedef int (*tEventlogOutputCb)(const char* fmt_p, ...);

//------------------------------------------------------------------------------
// function prototypes
//------------------------------------------------------------------------------

#ifdef __cplusplus
extern "C"
{
#endif

void eventlog_init(tEventlogFormat format_p,
                   UINT32 filterLevel,
                   UINT32 filterCategory,
                   tEventlogOutputCb pfnOutput_p);
void eventlog_printMessage(tEventlogLevel level_p,
                           tEventlogCategory category_p,
                           const char* fmt_p,
                           ...);
void eventlog_printNodeEvent(const tOplkApiEventNode* pNodeEvent_p);
void eventlog_printStateEvent(const tEventNmtStateChange* pStateChangeEvent_p);
void eventlog_printCfmResultEvent(UINT8 nodeId_p,
                                  tNmtNodeCommand nodeCommand_p);
void eventlog_printCfmProgressEvent(const tCfmEventCnProgress* pProgress_p);
void eventlog_printPdoEvent(const tOplkApiEventPdoChange* pPdoChange_p);
void eventlog_printHistoryEvent(const tErrHistoryEntry* pHistory_p);
void eventlog_printErrorEvent(const tEventError* pError_p);
void eventlog_printPdoMap(UINT16 mapObject_p,
                          UINT8 subIndex_p,
                          UINT64 mapping_p);

#ifdef __cplusplus
}
#endif

#endif /* _INC_eventlog_H_ */
