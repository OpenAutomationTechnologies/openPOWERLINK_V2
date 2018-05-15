/**
********************************************************************************
\file   errhndkcal.h

\brief  Definitions for kernel CAL module of Error Handler

This file provides the interface of the error handlers kernel CAL
module.

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
#ifndef _INC_errhndkcal_H_
#define _INC_errhndkcal_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <common/oplkinc.h>
#include <common/errhnd.h>

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

tOplkError      errhndkcal_init(void);
void            errhndkcal_exit(void);
tOplkError      errhndkcal_initMemory(void);
void            errhndkcal_deinitMemory(void);
tErrHndObjects* errhndkcal_getMemPtr(void);

/* Reading of error objects */
void errhndkcal_getCnLossSocError(UINT32* pCumulativeCnt_p, UINT32* pThresholdCnt_p, UINT32* pThreshold_p);
void errhndkcal_getCnLossPreqError(UINT32* pCumulativeCnt_p, UINT32* pThresholdCnt_p, UINT32* pThreshold_p);
void errhndkcal_getCnCrcError(UINT32* pCumulativeCnt_p, UINT32* pThresholdCnt_p, UINT32* pThreshold_p);

#if defined(CONFIG_INCLUDE_NMT_MN)
void errhndkcal_getMnCrcError(UINT32* pCumulativeCnt_p, UINT32* pThresholdCnt_p, UINT32* pThreshold_p) SECTION_ERRHNDKCAL_GETMNCNT;
void errhndkcal_getMnCycTimeExceedError(UINT32* pCumulativeCnt_p, UINT32* pThresholdCnt_p, UINT32* pThreshold_p) SECTION_ERRHNDKCAL_GETMNCNT;
void errhndkcal_getMnCnLossPresError(UINT nodeIdx_p, UINT32* pCumulativeCnt_p, UINT32* pThresholdCnt_p, UINT32* pThreshold_p) SECTION_ERRHNDKCAL_GETMNCNT;
#endif

void errhndkcal_getLossSocThresholdCnt(UINT32* pThresholdCnt_p);
void errhndkcal_getLossPreqThresholdCnt(UINT32* pThresholdCnt_p);
void errhndkcal_getCnCrcThresholdCnt(UINT32* pThresholdCnt_p);

#if defined(CONFIG_INCLUDE_NMT_MN)
void errhndkcal_getMnCrcThresholdCnt(UINT32* pThresholdCnt_p) SECTION_ERRHNDKCAL_GETMNCNT;
void errhndkcal_getMnCycTimeExceedThresholdCnt(UINT32* pThresholdCnt_p) SECTION_ERRHNDKCAL_GETMNCNT;
void errhndkcal_getMnCnLossPresThresholdCnt(UINT nodeIdx_p, UINT32* pThresholdCnt_p) SECTION_ERRHNDKCAL_GETMNCNT;
#endif

/* Writing of error counters */
void errhndkcal_setCnLossSocCounters(UINT32 cumulativeCnt_p, UINT32 thresholdCnt_p);
void errhndkcal_setCnLossPreqCounters(UINT32 cumulativeCnt_p, UINT32 thresholdCnt_p);
void errhndkcal_setCnCrcCounters(UINT32 cumulativeCnt_p, UINT32 thresholdCnt_p);

#if defined(CONFIG_INCLUDE_NMT_MN)
void errhndkcal_setMnCrcCounters(UINT32 cumulativeCnt_p, UINT32 thresholdCnt_p) SECTION_ERRHNDKCAL_SETMNCNT;
void errhndkcal_setMnCycTimeExceedCounters(UINT32 cumulativeCnt_p, UINT32 thresholdCnt_p) SECTION_ERRHNDKCAL_SETMNCNT;
void errhndkcal_setMnCnLossPresCounters(UINT nodeIdx_p, UINT32 cumulativeCnt_p, UINT32 thresholdCnt_p) SECTION_ERRHNDKCAL_SETMNCNT;
#endif

void errhndkcal_setLossSocThresholdCnt(UINT32 thresholdCnt_p);
void errhndkcal_setLossPreqThresholdCnt(UINT32 thresholdCnt_p);
void errhndkcal_setCnCrcThresholdCnt(UINT32 thresholdCnt_p);

#if defined(CONFIG_INCLUDE_NMT_MN)
void errhndkcal_setMnCrcThresholdCnt(UINT32 thresholdCnt_p) SECTION_ERRHNDKCAL_SETMNCNT;
void errhndkcal_setMnCycTimeExceedThresholdCnt(UINT32 thresholdCnt_p) SECTION_ERRHNDKCAL_SETMNCNT;
void errhndkcal_setMnCnLossPresThresholdCnt(UINT nodeIdx_p, UINT32 thresholdCnt_p) SECTION_ERRHNDKCAL_SETMNCNT;
#endif

#ifdef __cplusplus
}
#endif

#endif /* _INC_errhndkcal_H_ */
