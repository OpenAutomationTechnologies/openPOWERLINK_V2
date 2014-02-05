/**
********************************************************************************
\file   errhndkcal-hostif.c

\brief  Implementation of kernel CAL module for error handler

This module implements the CAL functions in kernel layer for the error handler.
This implementation uses the host interface ipcore from the kernel side.

\ingroup module_errhndkcal
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2012, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
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

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <oplk/oplkinc.h>
#include <oplk/event.h>
#include <common/errhnd.h>

#include "errhndkcal.h"

#include <hostiflib.h>

//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// module global vars
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// global function prototypes
//------------------------------------------------------------------------------


//============================================================================//
//            P R I V A T E   D E F I N I T I O N S                           //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tErrHndObjects *pErrHnd_l;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief    Initialize error handler user CAL module

The function initializes the user layer CAL module of the error handler.

\return     tOplkError
\retval     kEplSuccessful      successful return
\retval     kEplNoResource      ipcore instance not found
*/
//------------------------------------------------------------------------------
tOplkError errhndkcal_init (void)
{
    tHostifInstance pHostifInstance = hostif_getInstance(0);
    tOplkError      ret = kEplSuccessful;
    tHostifReturn   hifRet;
    UINT8*          pBase;
    UINT            span;

    if(pHostifInstance == NULL)
    {
        ret = kEplNoResource;
        goto Exit;
    }

    hifRet = hostif_getBuf(pHostifInstance, kHostifInstIdErrCount, &pBase, &span);

    if(hifRet != kHostifSuccessful)
    {
        DEBUG_LVL_ERROR_TRACE("%s() Could not get buffer from host interface (%d)\n",
                __func__, hifRet);
        ret = kEplNoResource;
        goto Exit;
    }

    if(span < sizeof(tErrHndObjects))
    {
        DEBUG_LVL_ERROR_TRACE("%s: Error Handler Object Buffer too small\n",
                __func__);
        ret = kEplNoResource;
        goto Exit;
    }

    pErrHnd_l = (tErrHndObjects*)pBase;

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief    shutdown error handler user CAL module

The function is used to deinitialize and shutdown the user layer
CAL module of the error handler.
*/
//------------------------------------------------------------------------------
void errhndkcal_exit (void)
{
    pErrHnd_l = NULL;
}

//------------------------------------------------------------------------------
/**
\brief    get pointer to error handler objects

The function returns a pointer to the memory block where the error handler
objects are stored.

\return The function returns a pointer to the error handler objects.

\ingroup module_errhndkcal
*/
//------------------------------------------------------------------------------
tErrHndObjects* errhndkcal_getMemPtr(void)
{
    return pErrHnd_l;
}

//------------------------------------------------------------------------------
// getters
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
/**
\brief  Get error objects for LossOfSoc error

\param  pCumulativeCnt_p      Pointer to store cumulative counter
\param  pThresholdCnt_p       Pointer to store threshold counter
\param  pThreshold_p          Pointer to store threshold
*/
//------------------------------------------------------------------------------
void errhndkcal_getCnLossSocError(UINT32 *pCumulativeCnt_p, UINT32 *pThresholdCnt_p,
                                UINT32 *pThreshold_p)
{
    *pCumulativeCnt_p = pErrHnd_l->cnLossSoc.cumulativeCnt;
    *pThresholdCnt_p = pErrHnd_l->cnLossSoc.thresholdCnt;
    *pThreshold_p = pErrHnd_l->cnLossSoc.threshold;
}

//------------------------------------------------------------------------------
/**
\brief  Get error objects for LossOfPreq error

\param  pCumulativeCnt_p      Pointer to store cumulative counter
\param  pThresholdCnt_p       Pointer to store threshold counter
\param  pThreshold_p          Pointer to store threshold
*/
//------------------------------------------------------------------------------
void errhndkcal_getCnLossPreqError(UINT32 *pCumulativeCnt_p, UINT32 *pThresholdCnt_p,
                                 UINT32 *pThreshold_p)
{
    *pCumulativeCnt_p = pErrHnd_l->cnLossPreq.cumulativeCnt;
    *pThresholdCnt_p = pErrHnd_l->cnLossPreq.thresholdCnt;
    *pThreshold_p = pErrHnd_l->cnLossPreq.threshold;
}

//------------------------------------------------------------------------------
/**
\brief  Get error objects for CN CRC error

\param  pCumulativeCnt_p      Pointer to store cumulative counter
\param  pThresholdCnt_p       Pointer to store threshold counter
\param  pThreshold_p          Pointer to store threshold
*/
//------------------------------------------------------------------------------
void errhndkcal_getCnCrcError(UINT32 *pCumulativeCnt_p, UINT32 *pThresholdCnt_p,
                              UINT32 *pThreshold_p)

{
    *pCumulativeCnt_p = pErrHnd_l->cnCrcErr.cumulativeCnt;
    *pThresholdCnt_p = pErrHnd_l->cnCrcErr.thresholdCnt;
    *pThreshold_p = pErrHnd_l->cnCrcErr.threshold;
}

#ifdef CONFIG_INCLUDE_NMT_MN
//------------------------------------------------------------------------------
/**
\brief  Get error objects for MN CRC error

\param  pCumulativeCnt_p      Pointer to store cumulative counter
\param  pThresholdCnt_p       Pointer to store threshold counter
\param  pThreshold_p          Pointer to store threshold
*/
//------------------------------------------------------------------------------
void errhndkcal_getMnCrcError(UINT32 *pCumulativeCnt_p, UINT32 *pThresholdCnt_p,
                              UINT32 *pThreshold_p)
{
    *pCumulativeCnt_p = pErrHnd_l->mnCrcErr.cumulativeCnt;
    *pThresholdCnt_p = pErrHnd_l->mnCrcErr.thresholdCnt;
    *pThreshold_p = pErrHnd_l->mnCrcErr.threshold;
}

//------------------------------------------------------------------------------
/**
\brief  Get error objects for MNCycleTimeExceed error

\param  pCumulativeCnt_p      Pointer to store cumulative counter
\param  pThresholdCnt_p       Pointer to store threshold counter
\param  pThreshold_p          Pointer to store threshold
*/
//------------------------------------------------------------------------------
void errhndkcal_getMnCycTimeExceedError(UINT32 *pCumulativeCnt_p,
                           UINT32 *pThresholdCnt_p, UINT32 *pThreshold_p)
{
    *pCumulativeCnt_p = pErrHnd_l->mnCycTimeExceed.cumulativeCnt;
    *pThresholdCnt_p = pErrHnd_l->mnCycTimeExceed.thresholdCnt;
    *pThreshold_p = pErrHnd_l->mnCycTimeExceed.threshold;
}

//------------------------------------------------------------------------------
/**
\brief  Get error objects for MNCNLossPres error

\param  nodeIdx_p             Index of node (node ID - 1)
\param  pCumulativeCnt_p      Pointer to store cumulative counter
\param  pThresholdCnt_p       Pointer to store threshold counter
\param  pThreshold_p          Pointer to store threshold
*/
//------------------------------------------------------------------------------
void errhndkcal_getMnCnLossPresError(UINT nodeIdx_p, UINT32 *pCumulativeCnt_p,
                                UINT32 *pThresholdCnt_p, UINT32 *pThreshold_p)
{
    *pCumulativeCnt_p = pErrHnd_l->aMnCnLossPres[nodeIdx_p].cumulativeCnt;
    *pThresholdCnt_p = pErrHnd_l->aMnCnLossPres[nodeIdx_p].thresholdCnt;
    *pThreshold_p = pErrHnd_l->aMnCnLossPres[nodeIdx_p].threshold;
}
#endif

//------------------------------------------------------------------------------
/**
\brief  Get threshold counter for LosSoc error

\param  pThresholdCnt_p       Pointer to store threshold counter
*/
//------------------------------------------------------------------------------
void errhndkcal_getLossSocThresholdCnt(UINT32 *pThresholdCnt_p)
{
    *pThresholdCnt_p = pErrHnd_l->cnLossSoc.thresholdCnt;
}
//------------------------------------------------------------------------------
/**
\brief  Get threshold counter for LosPreq error

\param  pThresholdCnt_p       Pointer to store threshold counter
*/
//------------------------------------------------------------------------------
void errhndkcal_getLossPreqThresholdCnt(UINT32 *pThresholdCnt_p)
{
    *pThresholdCnt_p = pErrHnd_l->cnLossPreq.thresholdCnt;
}
//------------------------------------------------------------------------------
/**
\brief  Get threshold counter for CnCrc error

\param  pThresholdCnt_p       Pointer to store threshold counter
*/
//------------------------------------------------------------------------------
void errhndkcal_getCnCrcThresholdCnt(UINT32 *pThresholdCnt_p)
{
    *pThresholdCnt_p = pErrHnd_l->cnCrcErr.thresholdCnt;
}

#ifdef CONFIG_INCLUDE_NMT_MN
//------------------------------------------------------------------------------
/**
\brief  Get threshold counter for MnCrc error

\param  pThresholdCnt_p       Pointer to store threshold counter
*/
//------------------------------------------------------------------------------
void errhndkcal_getMnCrcThresholdCnt(UINT32 *pThresholdCnt_p)
{
    *pThresholdCnt_p = pErrHnd_l->mnCrcErr.thresholdCnt;
}
//------------------------------------------------------------------------------
/**
\brief  Get threshold counter for MnCycTimeExceed error

\param  pThresholdCnt_p       Pointer to store threshold counter
*/
//------------------------------------------------------------------------------
void errhndkcal_getMnCycTimeExceedThresholdCnt(UINT32 *pThresholdCnt_p)
{
    *pThresholdCnt_p = pErrHnd_l->mnCycTimeExceed.thresholdCnt;
}

//------------------------------------------------------------------------------
/**
\brief  Get threshold counter for MnCnLossPres error

\param  nodeIdx_p             Index of node (node ID - 1)
\param  pThresholdCnt_p       Pointer to store threshold counter
*/
//------------------------------------------------------------------------------
void errhndkcal_getMnCnLossPresThresholdCnt(UINT nodeIdx_p, UINT32 *pThresholdCnt_p)
{
    *pThresholdCnt_p = pErrHnd_l->aMnCnLossPres[nodeIdx_p].thresholdCnt;
}
#endif

//------------------------------------------------------------------------------
/**
\brief  Set error counters of LossSoc error

\param  dwCumulativeCnt_p       Cumulative counter to set
\param  dwThresholdCnt_p        Threshold counter to set
*/
//------------------------------------------------------------------------------
void errhndkcal_setCnLossSocCounters(UINT32 dwCumulativeCnt_p, UINT32 dwThresholdCnt_p)
{
    pErrHnd_l->cnLossSoc.cumulativeCnt = dwCumulativeCnt_p;
    pErrHnd_l->cnLossSoc.thresholdCnt = dwThresholdCnt_p;
}

//------------------------------------------------------------------------------
/**
\brief  Set error counters of LossPreq error

\param  dwCumulativeCnt_p       Cumulative counter to set
\param  dwThresholdCnt_p        Threshold counter to set
*/
//------------------------------------------------------------------------------
void errhndkcal_setCnLossPreqCounters(UINT32 dwCumulativeCnt_p, UINT32 dwThresholdCnt_p)
{
    pErrHnd_l->cnLossPreq.cumulativeCnt = dwCumulativeCnt_p;
    pErrHnd_l->cnLossPreq.thresholdCnt = dwThresholdCnt_p;
}

//------------------------------------------------------------------------------
/**
\brief  Set error counters of CnCrc error

\param  dwCumulativeCnt_p       Cumulative counter to set
\param  dwThresholdCnt_p        Threshold counter to set
*/
//------------------------------------------------------------------------------
void errhndkcal_setCnCrcCounters(UINT32 dwCumulativeCnt_p, UINT32 dwThresholdCnt_p)
{
    pErrHnd_l->cnCrcErr.cumulativeCnt = dwCumulativeCnt_p;
    pErrHnd_l->cnCrcErr.thresholdCnt = dwThresholdCnt_p;
}

#ifdef CONFIG_INCLUDE_NMT_MN
//------------------------------------------------------------------------------
/**
\brief  Set error counters of MnCrc error

\param  dwCumulativeCnt_p       Cumulative counter to set
\param  dwThresholdCnt_p        Threshold counter to set
*/
//------------------------------------------------------------------------------
void errhndkcal_setMnCrcCounters(UINT32 dwCumulativeCnt_p, UINT32 dwThresholdCnt_p)
{
    pErrHnd_l->mnCrcErr.cumulativeCnt = dwCumulativeCnt_p;
    pErrHnd_l->mnCrcErr.thresholdCnt = dwThresholdCnt_p;
}

//------------------------------------------------------------------------------
/**
\brief  Set error counters of CycTimeExceed error

\param  dwCumulativeCnt_p       Cumulative counter to set
\param  dwThresholdCnt_p        Threshold counter to set
*/
//------------------------------------------------------------------------------
void errhndkcal_setMnCycTimeExceedCounters(UINT32 dwCumulativeCnt_p, UINT32 dwThresholdCnt_p)
{
    pErrHnd_l->mnCycTimeExceed.cumulativeCnt = dwCumulativeCnt_p;
    pErrHnd_l->mnCycTimeExceed.thresholdCnt = dwThresholdCnt_p;
}

//------------------------------------------------------------------------------
/**
\brief  Set error counters of MnCnLossPres error

\param  nodeIdx_p             Index of node (node ID - 1)
\param  dwCumulativeCnt_p       Cumulative counter to set
\param  dwThresholdCnt_p        Threshold counter to set
*/
//------------------------------------------------------------------------------
void errhndkcal_setMnCnLossPresCounters(UINT nodeIdx_p, UINT32 dwCumulativeCnt_p,
                                        UINT32 dwThresholdCnt_p)
{
    pErrHnd_l->aMnCnLossPres[nodeIdx_p].cumulativeCnt = dwCumulativeCnt_p;
    pErrHnd_l->aMnCnLossPres[nodeIdx_p].thresholdCnt = dwThresholdCnt_p;
}
#endif

//------------------------------------------------------------------------------
/**
\brief  Set threshold counter of LossSoc error

\param  dwThresholdCnt_p        Threshold counter to set
*/
//------------------------------------------------------------------------------
void errhndkcal_setLossSocThresholdCnt(UINT32 dwThresholdCnt_p)
{
    pErrHnd_l->cnLossSoc.thresholdCnt = dwThresholdCnt_p;
}

//------------------------------------------------------------------------------
/**
\brief  Set threshold counter of LossPreq error

\param  dwThresholdCnt_p        Threshold counter to set
*/
//------------------------------------------------------------------------------
void errhndkcal_setLossPreqThresholdCnt(UINT32 dwThresholdCnt_p)
{
    pErrHnd_l->cnLossPreq.thresholdCnt = dwThresholdCnt_p;
}

//------------------------------------------------------------------------------
/**
\brief  Set threshold counter of CnCrc error

\param  dwThresholdCnt_p        Threshold counter to set
*/
//------------------------------------------------------------------------------
void errhndkcal_setCnCrcThresholdCnt(UINT32 dwThresholdCnt_p)
{
    pErrHnd_l->cnCrcErr.thresholdCnt = dwThresholdCnt_p;
}

#ifdef CONFIG_INCLUDE_NMT_MN
//------------------------------------------------------------------------------
/**
\brief  Set threshold counter of MnCrc error

\param  dwThresholdCnt_p        Threshold counter to set
*/
//------------------------------------------------------------------------------
void errhndkcal_setMnCrcThresholdCnt(UINT32 dwThresholdCnt_p)
{
    pErrHnd_l->mnCrcErr.thresholdCnt = dwThresholdCnt_p;
}

//------------------------------------------------------------------------------
/**
\brief  Set threshold counter of MnCycTimeExceed error

\param  dwThresholdCnt_p        Threshold counter to set
*/
//------------------------------------------------------------------------------
void errhndkcal_setMnCycTimeExceedThresholdCnt(UINT32 dwThresholdCnt_p)
{
    pErrHnd_l->mnCycTimeExceed.thresholdCnt = dwThresholdCnt_p;
}

//------------------------------------------------------------------------------
/**
\brief  Set threshold counter of MnCnLossPres error

\param  nodeIdx_p             Index of node (node ID - 1)
\param  dwThresholdCnt_p        Threshold counter to set
*/
//------------------------------------------------------------------------------
void errhndkcal_setMnCnLossPresThresholdCnt(UINT nodeIdx_p, UINT32 dwThresholdCnt_p)
{
    pErrHnd_l->aMnCnLossPres[nodeIdx_p].thresholdCnt = dwThresholdCnt_p;
}
#endif

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//


