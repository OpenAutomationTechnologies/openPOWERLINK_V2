/**
********************************************************************************
\file   errhndkcal-shb.c

\brief  Kernel CAL module for error handler

This module implements the CAL functions in kernel layer for the error handler.
This implementation uses linear shared buffers provided by the shared buffer
module to share the error objects between user and kernel part.

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
#include <stddef.h>         // needed for offsetof macro

#include <EplInc.h>
#include <event.h>
#include <errhnd.h>

#include <SharedBuff.h>
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
typedef struct
{
    tShbInstance        shbInstance;        ///< Shared buffer instance for shared mem
    tErrHndObjects      errorObjects;       ///< Error objects
} tErrShbInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tErrShbInstance          instance_l;             ///< Shared buffer instance for shared mem
static BOOL                     fInitialized_l = FALSE; ///< Flag determines if module is initialized

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

\return     Returns always kEplSuccessful
*/
//------------------------------------------------------------------------------
tEplKernel errhndkcal_init (void)
{
    tShbError       shbError;
    UINT            fCreated;

    if (fInitialized_l)
    {
        return kEplNoFreeInstance;
    }

    /* allocate linear shared buffer for error counters */
    shbError = ShbLinAllocBuffer(sizeof(tErrHndObjects),
                                 ERRHND_SHB_ID,
                                 &instance_l.shbInstance,
                                 &fCreated);
    if (shbError != kShbOk)
    {
        return kEplNoResource;
    }
    else
    {
        /* If the buffer is just created we have to initialize it */
        if (fCreated)
        {
            EPL_MEMSET(&instance_l.errorObjects,
                       0,
                       sizeof(tErrHndObjects));

            shbError = ShbLinWriteDataBlock(instance_l.shbInstance,
                                            0,
                                            &instance_l.errorObjects,
                                            sizeof(tErrHndObjects));
            if (shbError != kShbOk)
            {
                ShbLinReleaseBuffer(instance_l.shbInstance);
                return kEplNoResource;
            }
        }
        fInitialized_l = TRUE;
    }
    return kEplSuccessful;
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
    if (fInitialized_l)
    {
        ShbLinReleaseBuffer(instance_l.shbInstance);
        fInitialized_l = FALSE;
    }
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
    ShbLinReadDataBlock(instance_l.shbInstance, pCumulativeCnt_p,
                        offsetof(tErrHndObjects, cnLossSoc.cumulativeCnt),
                        sizeof(UINT32));
    ShbLinReadDataBlock(instance_l.shbInstance, pThresholdCnt_p,
                        offsetof(tErrHndObjects, cnLossSoc.thresholdCnt),
                        sizeof(UINT32));
    ShbLinReadDataBlock(instance_l.shbInstance, pThreshold_p,
                        offsetof(tErrHndObjects, cnLossSoc.threshold),
                        sizeof(UINT32));
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
    ShbLinReadDataBlock(instance_l.shbInstance, pCumulativeCnt_p,
                        offsetof(tErrHndObjects, cnLossPreq.cumulativeCnt),
                        sizeof(UINT32));
    ShbLinReadDataBlock(instance_l.shbInstance, pThresholdCnt_p,
                        offsetof(tErrHndObjects, cnLossPreq.thresholdCnt),
                        sizeof(UINT32));
    ShbLinReadDataBlock(instance_l.shbInstance, pThreshold_p,
                        offsetof(tErrHndObjects, cnLossPreq.threshold),
                        sizeof(UINT32));
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
    ShbLinReadDataBlock(instance_l.shbInstance, pCumulativeCnt_p,
                        offsetof(tErrHndObjects, cnCrcErr.cumulativeCnt),
                        sizeof(UINT32));
    ShbLinReadDataBlock(instance_l.shbInstance, pThresholdCnt_p,
                        offsetof(tErrHndObjects, cnCrcErr.thresholdCnt),
                        sizeof(UINT32));
    ShbLinReadDataBlock(instance_l.shbInstance, pThreshold_p,
                        offsetof(tErrHndObjects, cnCrcErr.threshold),
                        sizeof(UINT32));
}

#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)
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
    ShbLinReadDataBlock(instance_l.shbInstance, pCumulativeCnt_p,
                        offsetof(tErrHndObjects, mnCrcErr.cumulativeCnt),
                        sizeof(UINT32));
    ShbLinReadDataBlock(instance_l.shbInstance, pThresholdCnt_p,
                        offsetof(tErrHndObjects, mnCrcErr.thresholdCnt),
                        sizeof(UINT32));
    ShbLinReadDataBlock(instance_l.shbInstance, pThreshold_p,
                        offsetof(tErrHndObjects, mnCrcErr.threshold),
                        sizeof(UINT32));
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
    ShbLinReadDataBlock(instance_l.shbInstance, pCumulativeCnt_p,
                        offsetof(tErrHndObjects, mnCycTimeExceed.cumulativeCnt),
                        sizeof(UINT32));
    ShbLinReadDataBlock(instance_l.shbInstance, pThresholdCnt_p,
                        offsetof(tErrHndObjects, mnCycTimeExceed.thresholdCnt),
                        sizeof(UINT32));
    ShbLinReadDataBlock(instance_l.shbInstance, pThreshold_p,
                        offsetof(tErrHndObjects, mnCycTimeExceed.threshold),
                        sizeof(UINT32));
}

//------------------------------------------------------------------------------
/**
\brief  Get error objects for MNCNLossPres error

\param  uiNodeIdx_p             Index of node (node ID - 1)
\param  pCumulativeCnt_p      Pointer to store cumulative counter
\param  pThresholdCnt_p       Pointer to store threshold counter
\param  pThreshold_p          Pointer to store threshold
*/
//------------------------------------------------------------------------------
void errhndkcal_getMnCnLossPresError(UINT uiNodeIdx_p, UINT32 *pCumulativeCnt_p,
                                UINT32 *pThresholdCnt_p, UINT32 *pThreshold_p)
{
    ShbLinReadDataBlock(instance_l.shbInstance, pCumulativeCnt_p,
                        offsetof(tErrHndObjects, aMnCnLossPres) +
                        (uiNodeIdx_p * sizeof(tErrorObject)) +
                        offsetof(tErrorObject, cumulativeCnt),
                        sizeof(UINT32));
    ShbLinReadDataBlock(instance_l.shbInstance, pThresholdCnt_p,
                        offsetof(tErrHndObjects, aMnCnLossPres) +
                        (uiNodeIdx_p * sizeof(tErrorObject)) +
                        offsetof(tErrorObject, thresholdCnt),
                        sizeof(UINT32));
    ShbLinReadDataBlock(instance_l.shbInstance, pThreshold_p,
                        offsetof(tErrHndObjects, aMnCnLossPres) +
                        (uiNodeIdx_p * sizeof(tErrorObject)) +
                        offsetof(tErrorObject, threshold),
                        sizeof(UINT32));
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
    ShbLinReadDataBlock(instance_l.shbInstance, pThresholdCnt_p,
                        offsetof(tErrHndObjects, cnLossSoc.thresholdCnt),
                        sizeof(UINT32));
}

//------------------------------------------------------------------------------
/**
\brief  Get threshold counter for LosPreq error

\param  pThresholdCnt_p       Pointer to store threshold counter
*/
//------------------------------------------------------------------------------
void errhndkcal_getLossPreqThresholdCnt(UINT32 *pThresholdCnt_p)
{
    ShbLinReadDataBlock(instance_l.shbInstance, pThresholdCnt_p,
                        offsetof(tErrHndObjects, cnLossPreq.thresholdCnt),
                        sizeof(UINT32));
}
//------------------------------------------------------------------------------
/**
\brief  Get threshold counter for CnCrc error

\param  pThresholdCnt_p       Pointer to store threshold counter
*/
//------------------------------------------------------------------------------
void errhndkcal_getCnCrcThresholdCnt(UINT32 *pThresholdCnt_p)
{
    ShbLinReadDataBlock(instance_l.shbInstance, pThresholdCnt_p,
                        offsetof(tErrHndObjects, cnCrcErr.thresholdCnt),
                        sizeof(UINT32));
}

#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)
//------------------------------------------------------------------------------
/**
\brief  Get threshold counter for MnCrc error

\param  pThresholdCnt_p       Pointer to store threshold counter
*/
//------------------------------------------------------------------------------
void errhndkcal_getMnCrcThresholdCnt(UINT32 *pThresholdCnt_p)
{
    ShbLinReadDataBlock(instance_l.shbInstance, pThresholdCnt_p,
                        offsetof(tErrHndObjects, mnCrcErr.thresholdCnt),
                        sizeof(UINT32));
}
//------------------------------------------------------------------------------
/**
\brief  Get threshold counter for MnCycTimeExceed error

\param  pThresholdCnt_p       Pointer to store threshold counter
*/
//------------------------------------------------------------------------------
void errhndkcal_getMnCycTimeExceedThresholdCnt(UINT32 *pThresholdCnt_p)
{
    ShbLinReadDataBlock(instance_l.shbInstance, pThresholdCnt_p,
                        offsetof(tErrHndObjects, mnCycTimeExceed.thresholdCnt),
                        sizeof(UINT32));
}

//------------------------------------------------------------------------------
/**
\brief  Get threshold counter for MnCnLossPres error

\param  uiNodeIdx_p             Index of node (node ID - 1)
\param  pThresholdCnt_p       Pointer to store threshold counter
*/
//------------------------------------------------------------------------------
void errhndkcal_getMnCnLossPresThresholdCnt(UINT uiNodeIdx_p, UINT32 *pThresholdCnt_p)
{
    ShbLinReadDataBlock(instance_l.shbInstance, pThresholdCnt_p,
                        offsetof(tErrHndObjects, aMnCnLossPres) +
                        (uiNodeIdx_p * sizeof(tErrorObject)) +
                        offsetof(tErrorObject, thresholdCnt),
                        sizeof(UINT32));
}
#endif

//------------------------------------------------------------------------------
// setters
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
/**
\brief  Set error counters of LossSoc error

\param  dwCumulativeCnt_p       Cumulative counter to set
\param  dwThresholdCnt_p        Threshold counter to set
*/
//------------------------------------------------------------------------------
void errhndkcal_setCnLossSocCounters(UINT32 dwCumulativeCnt_p, UINT32 dwThresholdCnt_p)
{
    ShbLinWriteDataBlock(instance_l.shbInstance,
                         offsetof(tErrHndObjects, cnLossSoc.cumulativeCnt),
                         &dwCumulativeCnt_p,
                         sizeof(UINT32));
    ShbLinWriteDataBlock(instance_l.shbInstance,
                         offsetof(tErrHndObjects, cnLossSoc.thresholdCnt),
                         &dwThresholdCnt_p,
                         sizeof(UINT32));
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
    ShbLinWriteDataBlock(instance_l.shbInstance,
                         offsetof(tErrHndObjects, cnLossPreq.cumulativeCnt),
                         &dwCumulativeCnt_p,
                         sizeof(UINT32));
    ShbLinWriteDataBlock(instance_l.shbInstance,
                         offsetof(tErrHndObjects, cnLossPreq.thresholdCnt),
                         &dwThresholdCnt_p,
                         sizeof(UINT32));
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
    ShbLinWriteDataBlock(instance_l.shbInstance,
                         offsetof(tErrHndObjects, cnCrcErr.cumulativeCnt),
                         &dwCumulativeCnt_p,
                         sizeof(UINT32));
    ShbLinWriteDataBlock(instance_l.shbInstance,
                         offsetof(tErrHndObjects, cnCrcErr.thresholdCnt),
                         &dwThresholdCnt_p,
                         sizeof(UINT32));
}

#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)
//------------------------------------------------------------------------------
/**
\brief  Set error counters of MnCrc error

\param  dwCumulativeCnt_p       Cumulative counter to set
\param  dwThresholdCnt_p        Threshold counter to set
*/
//------------------------------------------------------------------------------
void errhndkcal_setMnCrcCounters(UINT32 dwCumulativeCnt_p, UINT32 dwThresholdCnt_p)
{
    ShbLinWriteDataBlock(instance_l.shbInstance,
                         offsetof(tErrHndObjects, mnCrcErr.cumulativeCnt),
                         &dwCumulativeCnt_p,
                         sizeof(UINT32));
    ShbLinWriteDataBlock(instance_l.shbInstance,
                         offsetof(tErrHndObjects, mnCrcErr.thresholdCnt),
                         &dwThresholdCnt_p,
                         sizeof(UINT32));
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
    ShbLinWriteDataBlock(instance_l.shbInstance,
                         offsetof(tErrHndObjects, mnCycTimeExceed.cumulativeCnt),
                         &dwCumulativeCnt_p,
                         sizeof(UINT32));
    ShbLinWriteDataBlock(instance_l.shbInstance,
                         offsetof(tErrHndObjects, mnCycTimeExceed.thresholdCnt),
                         &dwThresholdCnt_p,
                         sizeof(UINT32));
}

//------------------------------------------------------------------------------
/**
\brief  Set error counters of MnCnLossPres error

\param  uiNodeIdx_p             Index of node (node ID - 1)
\param  dwCumulativeCnt_p       Cumulative counter to set
\param  dwThresholdCnt_p        Threshold counter to set
*/
//------------------------------------------------------------------------------
void errhndkcal_setMnCnLossPresCounters(UINT uiNodeIdx_p, UINT32 dwCumulativeCnt_p,
                                        UINT32 dwThresholdCnt_p)
{
    ShbLinWriteDataBlock(instance_l.shbInstance,
                         offsetof(tErrHndObjects, aMnCnLossPres) +
                         (uiNodeIdx_p * sizeof(tErrorObject)) +
                         offsetof(tErrorObject, cumulativeCnt),
                         &dwCumulativeCnt_p,
                         sizeof(UINT32));
    ShbLinWriteDataBlock(instance_l.shbInstance,
                         offsetof(tErrHndObjects, aMnCnLossPres) +
                         (uiNodeIdx_p * sizeof(tErrorObject)) +
                         offsetof(tErrorObject, thresholdCnt),
                         &dwThresholdCnt_p,
                         sizeof(UINT32));
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
    ShbLinWriteDataBlock(instance_l.shbInstance,
                         offsetof(tErrHndObjects, cnLossSoc.thresholdCnt),
                         &dwThresholdCnt_p,
                         sizeof(UINT32));
}

//------------------------------------------------------------------------------
/**
\brief  Set threshold counter of LossPreq error

\param  dwThresholdCnt_p        Threshold counter to set
*/
//------------------------------------------------------------------------------
void errhndkcal_setLossPreqThresholdCnt(UINT32 dwThresholdCnt_p)
{
    ShbLinWriteDataBlock(instance_l.shbInstance,
                         offsetof(tErrHndObjects, cnLossPreq.thresholdCnt),
                         &dwThresholdCnt_p,
                         sizeof(UINT32));
}

//------------------------------------------------------------------------------
/**
\brief  Set threshold counter of CnCrc error

\param  dwThresholdCnt_p        Threshold counter to set
*/
//------------------------------------------------------------------------------
void errhndkcal_setCnCrcThresholdCnt(UINT32 dwThresholdCnt_p)
{
    ShbLinWriteDataBlock(instance_l.shbInstance,
                         offsetof(tErrHndObjects, cnCrcErr.thresholdCnt),
                         &dwThresholdCnt_p,
                         sizeof(UINT32));
}

#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)
//------------------------------------------------------------------------------
/**
\brief  Set threshold counter of MnCrc error

\param  dwThresholdCnt_p        Threshold counter to set
*/
//------------------------------------------------------------------------------
void errhndkcal_setMnCrcThresholdCnt(UINT32 dwThresholdCnt_p)
{
    ShbLinWriteDataBlock(instance_l.shbInstance,
                         offsetof(tErrHndObjects, mnCrcErr.thresholdCnt),
                         &dwThresholdCnt_p,
                         sizeof(UINT32));
}

//------------------------------------------------------------------------------
/**
\brief  Set threshold counter of MnCycTimeExceed error

\param  dwThresholdCnt_p        Threshold counter to set
*/
//------------------------------------------------------------------------------
void errhndkcal_setMnCycTimeExceedThresholdCnt(UINT32 dwThresholdCnt_p)
{
    ShbLinWriteDataBlock(instance_l.shbInstance,
                         offsetof(tErrHndObjects, mnCycTimeExceed.thresholdCnt),
                         &dwThresholdCnt_p,
                         sizeof(UINT32));
}

//------------------------------------------------------------------------------
/**
\brief  Set threshold counter of MnCnLossPres error

\param  uiNodeIdx_p             Index of node (node ID - 1)
\param  dwThresholdCnt_p        Threshold counter to set
*/
//------------------------------------------------------------------------------
void errhndkcal_setMnCnLossPresThresholdCnt(UINT uiNodeIdx_p, UINT32 dwThresholdCnt_p)
{
    ShbLinWriteDataBlock(instance_l.shbInstance,
                         offsetof(tErrHndObjects, aMnCnLossPres) +
                         (uiNodeIdx_p * sizeof(tErrorObject)) +
                         offsetof(tErrorObject, thresholdCnt),
                         &dwThresholdCnt_p,
                         sizeof(UINT32));
}
#endif

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//


