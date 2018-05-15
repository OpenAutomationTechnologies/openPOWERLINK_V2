/**
********************************************************************************
\file   errhndkcal.c

\brief  Implementation of generic parts of the kernel CAL module for error handler

This module implements the generic parts of the CAL functions in kernel layer for
the error handler.

\ingroup module_errhndkcal
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

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <oplk/oplkinc.h>
#include "errhndkcal.h"

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
static tErrHndObjects*  pErrHndObjects_l;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Initialize error handler kernel CAL module

The function initializes the kernel layer CAL module of the error handler.

\return The function returns a tOplkError error code.

\ingroup module_errhndkcal
*/
//------------------------------------------------------------------------------
tOplkError errhndkcal_init(void)
{
    tOplkError  ret;

    ret = errhndkcal_initMemory();
    if (ret != kErrorOk)
        return ret;

    pErrHndObjects_l = errhndkcal_getMemPtr();

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Shutdown error handler kernel CAL module

The function is used to de-initialize and shutdown the kernel layer
CAL module of the error handler.

\ingroup module_errhndkcal
*/
//------------------------------------------------------------------------------
void errhndkcal_exit(void)
{
    errhndkcal_deinitMemory();

    pErrHndObjects_l = NULL;
}

//------------------------------------------------------------------------------
// getters
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
/**
\brief  Get error objects for LossOfSoc error

\param[out]     pCumulativeCnt_p    Pointer to store cumulative counter.
\param[out]     pThresholdCnt_p     Pointer to store threshold counter.
\param[out]     pThreshold_p        Pointer to store threshold.

\ingroup module_errhndkcal
*/
//------------------------------------------------------------------------------
void errhndkcal_getCnLossSocError(UINT32* pCumulativeCnt_p,
                                  UINT32* pThresholdCnt_p,
                                  UINT32* pThreshold_p)
{
    // Check parameter validity
    ASSERT(pCumulativeCnt_p != NULL);
    ASSERT(pThresholdCnt_p != NULL);
    ASSERT(pThreshold_p != NULL);

    OPLK_DCACHE_INVALIDATE(&pErrHndObjects_l->cnLossSoc, sizeof(tErrorObject));

    *pCumulativeCnt_p = pErrHndObjects_l->cnLossSoc.cumulativeCnt;
    *pThresholdCnt_p = pErrHndObjects_l->cnLossSoc.thresholdCnt;
    *pThreshold_p = pErrHndObjects_l->cnLossSoc.threshold;
}

//------------------------------------------------------------------------------
/**
\brief  Get error objects for LossOfPreq error

\param[out]     pCumulativeCnt_p    Pointer to store cumulative counter.
\param[out]     pThresholdCnt_p     Pointer to store threshold counter.
\param[out]     pThreshold_p        Pointer to store threshold.

\ingroup module_errhndkcal
*/
//------------------------------------------------------------------------------
void errhndkcal_getCnLossPreqError(UINT32* pCumulativeCnt_p,
                                   UINT32* pThresholdCnt_p,
                                   UINT32* pThreshold_p)
{
    // Check parameter validity
    ASSERT(pCumulativeCnt_p != NULL);
    ASSERT(pThresholdCnt_p != NULL);
    ASSERT(pThreshold_p != NULL);

    OPLK_DCACHE_INVALIDATE(&pErrHndObjects_l->cnLossPreq, sizeof(tErrorObject));

    *pCumulativeCnt_p = pErrHndObjects_l->cnLossPreq.cumulativeCnt;
    *pThresholdCnt_p = pErrHndObjects_l->cnLossPreq.thresholdCnt;
    *pThreshold_p = pErrHndObjects_l->cnLossPreq.threshold;
}

//------------------------------------------------------------------------------
/**
\brief  Get error objects for CN CRC error

\param[out]     pCumulativeCnt_p    Pointer to store cumulative counter.
\param[out]     pThresholdCnt_p     Pointer to store threshold counter.
\param[out]     pThreshold_p        Pointer to store threshold.

\ingroup module_errhndkcal
*/
//------------------------------------------------------------------------------
void errhndkcal_getCnCrcError(UINT32* pCumulativeCnt_p,
                              UINT32* pThresholdCnt_p,
                              UINT32* pThreshold_p)
{
    // Check parameter validity
    ASSERT(pCumulativeCnt_p != NULL);
    ASSERT(pThresholdCnt_p != NULL);
    ASSERT(pThreshold_p != NULL);

    OPLK_DCACHE_INVALIDATE(&pErrHndObjects_l->cnCrcErr, sizeof(tErrorObject));

    *pCumulativeCnt_p = pErrHndObjects_l->cnCrcErr.cumulativeCnt;
    *pThresholdCnt_p = pErrHndObjects_l->cnCrcErr.thresholdCnt;
    *pThreshold_p = pErrHndObjects_l->cnCrcErr.threshold;
}

#if defined(CONFIG_INCLUDE_NMT_MN)
//------------------------------------------------------------------------------
/**
\brief  Get error objects for MN CRC error

\param[out]     pCumulativeCnt_p    Pointer to store cumulative counter.
\param[out]     pThresholdCnt_p     Pointer to store threshold counter.
\param[out]     pThreshold_p        Pointer to store threshold.

\ingroup module_errhndkcal
*/
//------------------------------------------------------------------------------
void errhndkcal_getMnCrcError(UINT32* pCumulativeCnt_p,
                              UINT32* pThresholdCnt_p,
                              UINT32* pThreshold_p)
{
    // Check parameter validity
    ASSERT(pCumulativeCnt_p != NULL);
    ASSERT(pThresholdCnt_p != NULL);
    ASSERT(pThreshold_p != NULL);

    OPLK_DCACHE_INVALIDATE(&pErrHndObjects_l->mnCrcErr, sizeof(tErrorObject));

    *pCumulativeCnt_p = pErrHndObjects_l->mnCrcErr.cumulativeCnt;
    *pThresholdCnt_p = pErrHndObjects_l->mnCrcErr.thresholdCnt;
    *pThreshold_p = pErrHndObjects_l->mnCrcErr.threshold;
}

//------------------------------------------------------------------------------
/**
\brief  Get error objects for MNCycleTimeExceed error

\param[out]     pCumulativeCnt_p    Pointer to store cumulative counter.
\param[out]     pThresholdCnt_p     Pointer to store threshold counter.
\param[out]     pThreshold_p        Pointer to store threshold.

\ingroup module_errhndkcal
*/
//------------------------------------------------------------------------------
void errhndkcal_getMnCycTimeExceedError(UINT32* pCumulativeCnt_p,
                                        UINT32* pThresholdCnt_p,
                                        UINT32* pThreshold_p)
{
    // Check parameter validity
    ASSERT(pCumulativeCnt_p != NULL);
    ASSERT(pThresholdCnt_p != NULL);
    ASSERT(pThreshold_p != NULL);

    OPLK_DCACHE_INVALIDATE(&pErrHndObjects_l->mnCycTimeExceed, sizeof(tErrorObject));

    *pCumulativeCnt_p = pErrHndObjects_l->mnCycTimeExceed.cumulativeCnt;
    *pThresholdCnt_p = pErrHndObjects_l->mnCycTimeExceed.thresholdCnt;
    *pThreshold_p = pErrHndObjects_l->mnCycTimeExceed.threshold;
}

//------------------------------------------------------------------------------
/**
\brief  Get error objects for MNCNLossPres error

\param[in]      nodeIdx_p           Index of node (node ID - 1).
\param[out]     pCumulativeCnt_p    Pointer to store cumulative counter.
\param[out]     pThresholdCnt_p     Pointer to store threshold counter.
\param[out]     pThreshold_p        Pointer to store threshold.

\ingroup module_errhndkcal
*/
//------------------------------------------------------------------------------
void errhndkcal_getMnCnLossPresError(UINT nodeIdx_p,
                                     UINT32* pCumulativeCnt_p,
                                     UINT32* pThresholdCnt_p,
                                     UINT32* pThreshold_p)
{
    // Check parameter validity
    ASSERT(pCumulativeCnt_p != NULL);
    ASSERT(pThresholdCnt_p != NULL);
    ASSERT(pThreshold_p != NULL);

    OPLK_DCACHE_INVALIDATE(&pErrHndObjects_l->aMnCnLossPres, sizeof(tErrorObject));

    *pCumulativeCnt_p = pErrHndObjects_l->aMnCnLossPres[nodeIdx_p].cumulativeCnt;
    *pThresholdCnt_p = pErrHndObjects_l->aMnCnLossPres[nodeIdx_p].thresholdCnt;
    *pThreshold_p = pErrHndObjects_l->aMnCnLossPres[nodeIdx_p].threshold;
}
#endif

//------------------------------------------------------------------------------
/**
\brief  Get threshold counter for LosSoc error

\param[out]     pThresholdCnt_p     Pointer to store threshold counter.

\ingroup module_errhndkcal
*/
//------------------------------------------------------------------------------
void errhndkcal_getLossSocThresholdCnt(UINT32* pThresholdCnt_p)
{
    // Check parameter validity
    ASSERT(pThresholdCnt_p != NULL);

    OPLK_DCACHE_INVALIDATE(&pErrHndObjects_l->cnLossSoc, sizeof(tErrorObject));

    *pThresholdCnt_p = pErrHndObjects_l->cnLossSoc.thresholdCnt;
}

//------------------------------------------------------------------------------
/**
\brief  Get threshold counter for LosPreq error

\param[out]     pThresholdCnt_p     Pointer to store threshold counter.

\ingroup module_errhndkcal
*/
//------------------------------------------------------------------------------
void errhndkcal_getLossPreqThresholdCnt(UINT32* pThresholdCnt_p)
{
    // Check parameter validity
    ASSERT(pThresholdCnt_p != NULL);

    OPLK_DCACHE_INVALIDATE(&pErrHndObjects_l->cnLossPreq, sizeof(tErrorObject));

    *pThresholdCnt_p = pErrHndObjects_l->cnLossPreq.thresholdCnt;
}

//------------------------------------------------------------------------------
/**
\brief  Get threshold counter for CnCrc error

\param[out]     pThresholdCnt_p     Pointer to store threshold counter.

\ingroup module_errhndkcal
*/
//------------------------------------------------------------------------------
void errhndkcal_getCnCrcThresholdCnt(UINT32* pThresholdCnt_p)
{
    // Check parameter validity
    ASSERT(pThresholdCnt_p != NULL);

    OPLK_DCACHE_INVALIDATE(&pErrHndObjects_l->cnCrcErr, sizeof(tErrorObject));

    *pThresholdCnt_p = pErrHndObjects_l->cnCrcErr.thresholdCnt;
}

#if defined(CONFIG_INCLUDE_NMT_MN)
//------------------------------------------------------------------------------
/**
\brief  Get threshold counter for MnCrc error

\param[out]     pThresholdCnt_p     Pointer to store threshold counter.

\ingroup module_errhndkcal
*/
//------------------------------------------------------------------------------
void errhndkcal_getMnCrcThresholdCnt(UINT32* pThresholdCnt_p)
{
    // Check parameter validity
    ASSERT(pThresholdCnt_p != NULL);

    OPLK_DCACHE_INVALIDATE(&pErrHndObjects_l->mnCrcErr, sizeof(tErrorObject));

    *pThresholdCnt_p = pErrHndObjects_l->mnCrcErr.thresholdCnt;
}

//------------------------------------------------------------------------------
/**
\brief  Get threshold counter for MnCycTimeExceed error

\param[out]     pThresholdCnt_p     Pointer to store threshold counter.

\ingroup module_errhndkcal
*/
//------------------------------------------------------------------------------
void errhndkcal_getMnCycTimeExceedThresholdCnt(UINT32* pThresholdCnt_p)
{
    // Check parameter validity
    ASSERT(pThresholdCnt_p != NULL);

    OPLK_DCACHE_INVALIDATE(&pErrHndObjects_l->mnCycTimeExceed, sizeof(tErrorObject));

    *pThresholdCnt_p = pErrHndObjects_l->mnCycTimeExceed.thresholdCnt;
}

//------------------------------------------------------------------------------
/**
\brief  Get threshold counter for MnCnLossPres error

\param[in]      nodeIdx_p           Index of node (node ID - 1).
\param[out]     pThresholdCnt_p     Pointer to store threshold counter.

\ingroup module_errhndkcal
*/
//------------------------------------------------------------------------------
void errhndkcal_getMnCnLossPresThresholdCnt(UINT nodeIdx_p,
                                            UINT32* pThresholdCnt_p)
{
    // Check parameter validity
    ASSERT(pThresholdCnt_p != NULL);

    OPLK_DCACHE_INVALIDATE(&pErrHndObjects_l->aMnCnLossPres[nodeIdx_p], sizeof(tErrorObject));

    *pThresholdCnt_p = pErrHndObjects_l->aMnCnLossPres[nodeIdx_p].thresholdCnt;
}
#endif

//------------------------------------------------------------------------------
// setters
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
/**
\brief  Set error counters of LossSoc error

\param[in]      cumulativeCnt_p     Cumulative counter to set.
\param[in]      thresholdCnt_p      Threshold counter to set.

\ingroup module_errhndkcal
*/
//------------------------------------------------------------------------------
void errhndkcal_setCnLossSocCounters(UINT32 cumulativeCnt_p,
                                     UINT32 thresholdCnt_p)
{
    pErrHndObjects_l->cnLossSoc.cumulativeCnt = cumulativeCnt_p;
    pErrHndObjects_l->cnLossSoc.thresholdCnt = thresholdCnt_p;

    OPLK_DCACHE_FLUSH(&pErrHndObjects_l->cnLossSoc, sizeof(tErrorObject));
}

//------------------------------------------------------------------------------
/**
\brief  Set error counters of LossPreq error

\param[in]      cumulativeCnt_p     Cumulative counter to set.
\param[in]      thresholdCnt_p      Threshold counter to set.

\ingroup module_errhndkcal
*/
//------------------------------------------------------------------------------
void errhndkcal_setCnLossPreqCounters(UINT32 cumulativeCnt_p,
                                      UINT32 thresholdCnt_p)
{
    pErrHndObjects_l->cnLossPreq.cumulativeCnt = cumulativeCnt_p;
    pErrHndObjects_l->cnLossPreq.thresholdCnt = thresholdCnt_p;

    OPLK_DCACHE_FLUSH(&pErrHndObjects_l->cnLossPreq, sizeof(tErrorObject));
}

//------------------------------------------------------------------------------
/**
\brief  Set error counters of CnCrc error

\param[in]      cumulativeCnt_p     Cumulative counter to set.
\param[in]      thresholdCnt_p      Threshold counter to set.

\ingroup module_errhndkcal
*/
//------------------------------------------------------------------------------
void errhndkcal_setCnCrcCounters(UINT32 cumulativeCnt_p,
                                 UINT32 thresholdCnt_p)
{
    pErrHndObjects_l->cnCrcErr.cumulativeCnt = cumulativeCnt_p;
    pErrHndObjects_l->cnCrcErr.thresholdCnt = thresholdCnt_p;

    OPLK_DCACHE_FLUSH(&pErrHndObjects_l->cnCrcErr, sizeof(tErrorObject));
}

#if defined(CONFIG_INCLUDE_NMT_MN)
//------------------------------------------------------------------------------
/**
\brief  Set error counters of MnCrc error

\param[in]      cumulativeCnt_p     Cumulative counter to set.
\param[in]      thresholdCnt_p      Threshold counter to set.

\ingroup module_errhndkcal
*/
//------------------------------------------------------------------------------
void errhndkcal_setMnCrcCounters(UINT32 cumulativeCnt_p,
                                 UINT32 thresholdCnt_p)
{
    pErrHndObjects_l->mnCrcErr.cumulativeCnt = cumulativeCnt_p;
    pErrHndObjects_l->mnCrcErr.thresholdCnt = thresholdCnt_p;

    OPLK_DCACHE_FLUSH(&pErrHndObjects_l->mnCrcErr, sizeof(tErrorObject));
}

//------------------------------------------------------------------------------
/**
\brief  Set error counters of CycTimeExceed error

\param[in]      cumulativeCnt_p     Cumulative counter to set.
\param[in]      thresholdCnt_p      Threshold counter to set.

\ingroup module_errhndkcal
*/
//------------------------------------------------------------------------------
void errhndkcal_setMnCycTimeExceedCounters(UINT32 cumulativeCnt_p,
                                           UINT32 thresholdCnt_p)
{
    pErrHndObjects_l->mnCycTimeExceed.cumulativeCnt = cumulativeCnt_p;
    pErrHndObjects_l->mnCycTimeExceed.thresholdCnt = thresholdCnt_p;

    OPLK_DCACHE_FLUSH(&pErrHndObjects_l->mnCycTimeExceed, sizeof(tErrorObject));
}

//------------------------------------------------------------------------------
/**
\brief  Set error counters of MnCnLossPres error

\param[in]      nodeIdx_p           Index of node (node ID - 1).
\param[in]      cumulativeCnt_p     Cumulative counter to set.
\param[in]      thresholdCnt_p      Threshold counter to set.

\ingroup module_errhndkcal
*/
//------------------------------------------------------------------------------
void errhndkcal_setMnCnLossPresCounters(UINT nodeIdx_p,
                                        UINT32 cumulativeCnt_p,
                                        UINT32 thresholdCnt_p)
{
    pErrHndObjects_l->aMnCnLossPres[nodeIdx_p].cumulativeCnt = cumulativeCnt_p;
    pErrHndObjects_l->aMnCnLossPres[nodeIdx_p].thresholdCnt = thresholdCnt_p;

    OPLK_DCACHE_FLUSH(&pErrHndObjects_l->aMnCnLossPres[nodeIdx_p], sizeof(tErrorObject));
}
#endif

//------------------------------------------------------------------------------
/**
\brief  Set threshold counter of LossSoc error

\param[in]      thresholdCnt_p      Threshold counter to set.

\ingroup module_errhndkcal
*/
//------------------------------------------------------------------------------
void errhndkcal_setLossSocThresholdCnt(UINT32 thresholdCnt_p)
{
    pErrHndObjects_l->cnLossSoc.thresholdCnt = thresholdCnt_p;

    OPLK_DCACHE_FLUSH(&pErrHndObjects_l->cnLossSoc, sizeof(tErrorObject));
}

//------------------------------------------------------------------------------
/**
\brief  Set threshold counter of LossPreq error

\param[in]      thresholdCnt_p      Threshold counter to set.

\ingroup module_errhndkcal
*/
//------------------------------------------------------------------------------
void errhndkcal_setLossPreqThresholdCnt(UINT32 thresholdCnt_p)
{
    pErrHndObjects_l->cnLossPreq.thresholdCnt = thresholdCnt_p;

    OPLK_DCACHE_FLUSH(&pErrHndObjects_l->cnLossPreq, sizeof(tErrorObject));
}

//------------------------------------------------------------------------------
/**
\brief  Set threshold counter of CnCrc error

\param[in]      thresholdCnt_p      Threshold counter to set.

\ingroup module_errhndkcal
*/
//------------------------------------------------------------------------------
void errhndkcal_setCnCrcThresholdCnt(UINT32 thresholdCnt_p)
{
    pErrHndObjects_l->cnCrcErr.thresholdCnt = thresholdCnt_p;

    OPLK_DCACHE_FLUSH(&pErrHndObjects_l->cnCrcErr, sizeof(tErrorObject));
}

#if defined(CONFIG_INCLUDE_NMT_MN)
//------------------------------------------------------------------------------
/**
\brief  Set threshold counter of MnCrc error

\param[in]      thresholdCnt_p      Threshold counter to set.

\ingroup module_errhndkcal
*/
//------------------------------------------------------------------------------
void errhndkcal_setMnCrcThresholdCnt(UINT32 thresholdCnt_p)
{
    pErrHndObjects_l->mnCrcErr.thresholdCnt = thresholdCnt_p;

    OPLK_DCACHE_FLUSH(&pErrHndObjects_l->mnCrcErr, sizeof(tErrorObject));
}

//------------------------------------------------------------------------------
/**
\brief  Set threshold counter of MnCycTimeExceed error

\param[in]      thresholdCnt_p      Threshold counter to set.

\ingroup module_errhndkcal
*/
//------------------------------------------------------------------------------
void errhndkcal_setMnCycTimeExceedThresholdCnt(UINT32 thresholdCnt_p)
{
    pErrHndObjects_l->mnCycTimeExceed.thresholdCnt = thresholdCnt_p;

    OPLK_DCACHE_FLUSH(&pErrHndObjects_l->mnCycTimeExceed, sizeof(tErrorObject));
}

//------------------------------------------------------------------------------
/**
\brief  Set threshold counter of MnCnLossPres error

\param[in]      nodeIdx_p           Index of node (node ID - 1).
\param[in]      thresholdCnt_p      Threshold counter to set.

\ingroup module_errhndkcal
*/
//------------------------------------------------------------------------------
void errhndkcal_setMnCnLossPresThresholdCnt(UINT nodeIdx_p,
                                            UINT32 thresholdCnt_p)
{
    pErrHndObjects_l->aMnCnLossPres[nodeIdx_p].thresholdCnt = thresholdCnt_p;

    OPLK_DCACHE_FLUSH(&pErrHndObjects_l->aMnCnLossPres[nodeIdx_p], sizeof(tErrorObject));
}
#endif

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

/// \}
