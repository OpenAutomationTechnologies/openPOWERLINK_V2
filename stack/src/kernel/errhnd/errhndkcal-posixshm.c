/**
********************************************************************************
\file   errhndkcal-posixshm.c

\brief  Implementation of kernel CAL module for error handler

This module implements the CAL functions in kernel layer for the error handler.
This implementation uses posix shared memory to share the error objects
between user and kernel part.

\ingroup module_errhndkcal
*******************************************************************************/

/*------------------------------------------------------------------------------
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

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <common/oplkinc.h>
#include "errhndkcal.h"

#include <unistd.h>
#include <fcntl.h>           /* For O_* constants */
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>


//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#define ERRHND_SHM_NAME "/shmErrHnd"

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
static int                      fd_l;
static tErrHndObjects*          pErrHndMem_l;
static BOOL                     fCreator_l;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief    Initialize error handler kernel CAL module

The function initializes the kernel layer CAL module of the error handler.

\return     Returns always kErrorOk

\ingroup module_errhndkcal
*/
//------------------------------------------------------------------------------
tOplkError errhndkcal_init(void)
{
    struct stat             stat;

    if (pErrHndMem_l != NULL)
        return kErrorNoFreeInstance;

    if ((fd_l = shm_open(ERRHND_SHM_NAME, O_RDWR | O_CREAT, 0)) < 0)
    {
        DEBUG_LVL_ERROR_TRACE("%s() shm_open failed!\n", __func__);
        return kErrorNoResource;
    }

    if (fstat(fd_l, &stat) != 0)
    {
        close(fd_l);
        return kErrorNoResource;
    }

    if (stat.st_size == 0)
    {
        if (ftruncate(fd_l, sizeof(tErrHndObjects)) == -1)
        {
            DEBUG_LVL_ERROR_TRACE("%s() ftruncate failed!\n", __func__);
            close(fd_l);
            shm_unlink(ERRHND_SHM_NAME);
            return kErrorNoResource;
        }
        fCreator_l = TRUE;
    }

    pErrHndMem_l = mmap(NULL, sizeof(tErrHndObjects), PROT_READ | PROT_WRITE, MAP_SHARED, fd_l, 0);
    if (pErrHndMem_l == MAP_FAILED)
    {
        DEBUG_LVL_ERROR_TRACE("%s() mmap header failed!\n", __func__);
        close(fd_l);
        if (fCreator_l)
            shm_unlink(ERRHND_SHM_NAME);
        return kErrorNoResource;
    }

    if (fCreator_l)
    {
        OPLK_MEMSET(pErrHndMem_l, 0, sizeof(tErrHndObjects));
    }
    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief    Shutdown error handler kernel CAL module

The function is used to deinitialize and shutdown the kernel layer
CAL module of the error handler.

\ingroup module_errhndkcal
*/
//------------------------------------------------------------------------------
void errhndkcal_exit(void)
{
    if (pErrHndMem_l != NULL)
    {
        munmap(pErrHndMem_l, sizeof(tErrHndObjects));
        close(fd_l);
        if (fCreator_l)
            shm_unlink(ERRHND_SHM_NAME);
        fd_l = 0;
        pErrHndMem_l = 0;
    }
}

//------------------------------------------------------------------------------
// getters
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
/**
\brief  Get error objects for LossOfSoc error

\param  pCumulativeCnt_p      Pointer to store cumulative counter.
\param  pThresholdCnt_p       Pointer to store threshold counter.
\param  pThreshold_p          Pointer to store threshold.

\ingroup module_errhndkcal
*/
//------------------------------------------------------------------------------
void errhndkcal_getCnLossSocError(UINT32* pCumulativeCnt_p,
                                  UINT32* pThresholdCnt_p,
                                  UINT32* pThreshold_p)
{
    *pCumulativeCnt_p = pErrHndMem_l->cnLossSoc.cumulativeCnt;
    *pThresholdCnt_p = pErrHndMem_l->cnLossSoc.thresholdCnt;
    *pThreshold_p = pErrHndMem_l->cnLossSoc.threshold;
}

//------------------------------------------------------------------------------
/**
\brief  Get error objects for LossOfPreq error

\param  pCumulativeCnt_p      Pointer to store cumulative counter.
\param  pThresholdCnt_p       Pointer to store threshold counter.
\param  pThreshold_p          Pointer to store threshold.

\ingroup module_errhndkcal
*/
//------------------------------------------------------------------------------
void errhndkcal_getCnLossPreqError(UINT32* pCumulativeCnt_p,
                                   UINT32* pThresholdCnt_p,
                                   UINT32* pThreshold_p)
{
    *pCumulativeCnt_p = pErrHndMem_l->cnLossPreq.cumulativeCnt;
    *pThresholdCnt_p = pErrHndMem_l->cnLossPreq.thresholdCnt;
    *pThreshold_p = pErrHndMem_l->cnLossPreq.threshold;
}

//------------------------------------------------------------------------------
/**
\brief  Get error objects for CN CRC error

\param  pCumulativeCnt_p      Pointer to store cumulative counter.
\param  pThresholdCnt_p       Pointer to store threshold counter.
\param  pThreshold_p          Pointer to store threshold.

\ingroup module_errhndkcal
*/
//------------------------------------------------------------------------------
void errhndkcal_getCnCrcError(UINT32* pCumulativeCnt_p,
                              UINT32* pThresholdCnt_p,
                              UINT32* pThreshold_p)

{
    *pCumulativeCnt_p = pErrHndMem_l->cnCrcErr.cumulativeCnt;
    *pThresholdCnt_p = pErrHndMem_l->cnCrcErr.thresholdCnt;
    *pThreshold_p = pErrHndMem_l->cnCrcErr.threshold;
}

#ifdef CONFIG_INCLUDE_NMT_MN
//------------------------------------------------------------------------------
/**
\brief  Get error objects for MN CRC error

\param  pCumulativeCnt_p      Pointer to store cumulative counter.
\param  pThresholdCnt_p       Pointer to store threshold counter.
\param  pThreshold_p          Pointer to store threshold.

\ingroup module_errhndkcal
*/
//------------------------------------------------------------------------------
void errhndkcal_getMnCrcError(UINT32* pCumulativeCnt_p,
                              UINT32* pThresholdCnt_p,
                              UINT32* pThreshold_p)
{
    *pCumulativeCnt_p = pErrHndMem_l->mnCrcErr.cumulativeCnt;
    *pThresholdCnt_p = pErrHndMem_l->mnCrcErr.thresholdCnt;
    *pThreshold_p = pErrHndMem_l->mnCrcErr.threshold;
}

//------------------------------------------------------------------------------
/**
\brief  Get error objects for MNCycleTimeExceed error

\param  pCumulativeCnt_p      Pointer to store cumulative counter.
\param  pThresholdCnt_p       Pointer to store threshold counter.
\param  pThreshold_p          Pointer to store threshold.

\ingroup module_errhndkcal
*/
//------------------------------------------------------------------------------
void errhndkcal_getMnCycTimeExceedError(UINT32* pCumulativeCnt_p,
                                        UINT32* pThresholdCnt_p,
                                        UINT32* pThreshold_p)
{
    *pCumulativeCnt_p = pErrHndMem_l->mnCycTimeExceed.cumulativeCnt;
    *pThresholdCnt_p = pErrHndMem_l->mnCycTimeExceed.thresholdCnt;
    *pThreshold_p = pErrHndMem_l->mnCycTimeExceed.threshold;
}

//------------------------------------------------------------------------------
/**
\brief  Get error objects for MNCNLossPres error

\param  nodeIdx_p             Index of node (node ID - 1).
\param  pCumulativeCnt_p      Pointer to store cumulative counter.
\param  pThresholdCnt_p       Pointer to store threshold counter.
\param  pThreshold_p          Pointer to store threshold.

\ingroup module_errhndkcal
*/
//------------------------------------------------------------------------------
void errhndkcal_getMnCnLossPresError(UINT nodeIdx_p,
                                     UINT32* pCumulativeCnt_p,
                                     UINT32* pThresholdCnt_p,
                                     UINT32* pThreshold_p)
{
    *pCumulativeCnt_p = pErrHndMem_l->aMnCnLossPres[nodeIdx_p].cumulativeCnt;
    *pThresholdCnt_p = pErrHndMem_l->aMnCnLossPres[nodeIdx_p].thresholdCnt;
    *pThreshold_p = pErrHndMem_l->aMnCnLossPres[nodeIdx_p].threshold;
}
#endif

//------------------------------------------------------------------------------
/**
\brief  Get threshold counter for LosSoc error

\param  pThresholdCnt_p       Pointer to store threshold counter.

\ingroup module_errhndkcal
*/
//------------------------------------------------------------------------------
void errhndkcal_getLossSocThresholdCnt(UINT32* pThresholdCnt_p)
{
    *pThresholdCnt_p = pErrHndMem_l->cnLossSoc.thresholdCnt;
}

//------------------------------------------------------------------------------
/**
\brief  Get threshold counter for LosPreq error

\param  pThresholdCnt_p       Pointer to store threshold counter

\ingroup module_errhndkcal
*/
//------------------------------------------------------------------------------
void errhndkcal_getLossPreqThresholdCnt(UINT32* pThresholdCnt_p)
{
    *pThresholdCnt_p = pErrHndMem_l->cnLossPreq.thresholdCnt;
}
//------------------------------------------------------------------------------
/**
\brief  Get threshold counter for CnCrc error

\param  pThresholdCnt_p       Pointer to store threshold counter.

\ingroup module_errhndkcal
*/
//------------------------------------------------------------------------------
void errhndkcal_getCnCrcThresholdCnt(UINT32* pThresholdCnt_p)
{
    *pThresholdCnt_p = pErrHndMem_l->cnCrcErr.thresholdCnt;
}

#ifdef CONFIG_INCLUDE_NMT_MN
//------------------------------------------------------------------------------
/**
\brief  Get threshold counter for MnCrc error

\param  pThresholdCnt_p       Pointer to store threshold counter.

\ingroup module_errhndkcal
*/
//------------------------------------------------------------------------------
void errhndkcal_getMnCrcThresholdCnt(UINT32* pThresholdCnt_p)
{
    *pThresholdCnt_p = pErrHndMem_l->mnCrcErr.thresholdCnt;
}
//------------------------------------------------------------------------------
/**
\brief  Get threshold counter for MnCycTimeExceed error

\param  pThresholdCnt_p       Pointer to store threshold counter.

\ingroup module_errhndkcal
*/
//------------------------------------------------------------------------------
void errhndkcal_getMnCycTimeExceedThresholdCnt(UINT32* pThresholdCnt_p)
{
    *pThresholdCnt_p = pErrHndMem_l->mnCycTimeExceed.thresholdCnt;
}

//------------------------------------------------------------------------------
/**
\brief  Get threshold counter for MnCnLossPres error

\param  nodeIdx_p             Index of node (node ID - 1).
\param  pThresholdCnt_p       Pointer to store threshold counter.

\ingroup module_errhndkcal
*/
//------------------------------------------------------------------------------
void errhndkcal_getMnCnLossPresThresholdCnt(UINT nodeIdx_p,
                                            UINT32* pThresholdCnt_p)
{
    *pThresholdCnt_p = pErrHndMem_l->aMnCnLossPres[nodeIdx_p].thresholdCnt;
}
#endif

//------------------------------------------------------------------------------
// setters
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
/**
\brief  Set error counters of LossSoc error

\param  cumulativeCnt_p       Cumulative counter to set.
\param  thresholdCnt_p        Threshold counter to set.

\ingroup module_errhndkcal
*/
//------------------------------------------------------------------------------
void errhndkcal_setCnLossSocCounters(UINT32 cumulativeCnt_p,
                                     UINT32 thresholdCnt_p)
{
    pErrHndMem_l->cnLossSoc.cumulativeCnt = cumulativeCnt_p;
    pErrHndMem_l->cnLossSoc.thresholdCnt = thresholdCnt_p;
}

//------------------------------------------------------------------------------
/**
\brief  Set error counters of LossPreq error

\param  cumulativeCnt_p       Cumulative counter to set.
\param  thresholdCnt_p        Threshold counter to set.

\ingroup module_errhndkcal
*/
//------------------------------------------------------------------------------
void errhndkcal_setCnLossPreqCounters(UINT32 cumulativeCnt_p,
                                      UINT32 thresholdCnt_p)
{
    pErrHndMem_l->cnLossPreq.cumulativeCnt = cumulativeCnt_p;
    pErrHndMem_l->cnLossPreq.thresholdCnt = thresholdCnt_p;
}

//------------------------------------------------------------------------------
/**
\brief  Set error counters of CnCrc error

\param  cumulativeCnt_p       Cumulative counter to set.
\param  thresholdCnt_p        Threshold counter to set.

\ingroup module_errhndkcal
*/
//------------------------------------------------------------------------------
void errhndkcal_setCnCrcCounters(UINT32 cumulativeCnt_p,
                                 UINT32 thresholdCnt_p)
{
    pErrHndMem_l->cnCrcErr.cumulativeCnt = cumulativeCnt_p;
    pErrHndMem_l->cnCrcErr.thresholdCnt = thresholdCnt_p;
}

#ifdef CONFIG_INCLUDE_NMT_MN
//------------------------------------------------------------------------------
/**
\brief  Set error counters of MnCrc error

\param  cumulativeCnt_p       Cumulative counter to set.
\param  thresholdCnt_p        Threshold counter to set.

\ingroup module_errhndkcal
*/
//------------------------------------------------------------------------------
void errhndkcal_setMnCrcCounters(UINT32 cumulativeCnt_p,
                                 UINT32 thresholdCnt_p)
{
    pErrHndMem_l->mnCrcErr.cumulativeCnt = cumulativeCnt_p;
    pErrHndMem_l->mnCrcErr.thresholdCnt = thresholdCnt_p;
}

//------------------------------------------------------------------------------
/**
\brief  Set error counters of CycTimeExceed error

\param  cumulativeCnt_p       Cumulative counter to set.
\param  thresholdCnt_p        Threshold counter to set.

\ingroup module_errhndkcal
*/
//------------------------------------------------------------------------------
void errhndkcal_setMnCycTimeExceedCounters(UINT32 cumulativeCnt_p,
                                           UINT32 thresholdCnt_p)
{
    pErrHndMem_l->mnCycTimeExceed.cumulativeCnt = cumulativeCnt_p;
    pErrHndMem_l->mnCycTimeExceed.thresholdCnt = thresholdCnt_p;
}

//------------------------------------------------------------------------------
/**
\brief  Set error counters of MnCnLossPres error

\param  nodeIdx_p               Index of node (node ID - 1).
\param  cumulativeCnt_p       Cumulative counter to set.
\param  thresholdCnt_p        Threshold counter to set.

\ingroup module_errhndkcal
*/
//------------------------------------------------------------------------------
void errhndkcal_setMnCnLossPresCounters(UINT nodeIdx_p,
                                        UINT32 cumulativeCnt_p,
                                        UINT32 thresholdCnt_p)
{
    pErrHndMem_l->aMnCnLossPres[nodeIdx_p].cumulativeCnt = cumulativeCnt_p;
    pErrHndMem_l->aMnCnLossPres[nodeIdx_p].thresholdCnt = thresholdCnt_p;
}
#endif

//------------------------------------------------------------------------------
/**
\brief  Set threshold counter of LossSoc error

\param  thresholdCnt_p        Threshold counter to set.

\ingroup module_errhndkcal
*/
//------------------------------------------------------------------------------
void errhndkcal_setLossSocThresholdCnt(UINT32 thresholdCnt_p)
{
    pErrHndMem_l->cnLossSoc.thresholdCnt = thresholdCnt_p;
}

//------------------------------------------------------------------------------
/**
\brief  Set threshold counter of LossPreq error

\param  thresholdCnt_p        Threshold counter to set.

\ingroup module_errhndkcal
*/
//------------------------------------------------------------------------------
void errhndkcal_setLossPreqThresholdCnt(UINT32 thresholdCnt_p)
{
    pErrHndMem_l->cnLossPreq.thresholdCnt = thresholdCnt_p;
}

//------------------------------------------------------------------------------
/**
\brief  Set threshold counter of CnCrc error

\param  thresholdCnt_p        Threshold counter to set.

\ingroup module_errhndkcal
*/
//------------------------------------------------------------------------------
void errhndkcal_setCnCrcThresholdCnt(UINT32 thresholdCnt_p)
{
    pErrHndMem_l->cnCrcErr.thresholdCnt = thresholdCnt_p;
}

#ifdef CONFIG_INCLUDE_NMT_MN
//------------------------------------------------------------------------------
/**
\brief  Set threshold counter of MnCrc error

\param  thresholdCnt_p        Threshold counter to set.

\ingroup module_errhndkcal
*/
//------------------------------------------------------------------------------
void errhndkcal_setMnCrcThresholdCnt(UINT32 thresholdCnt_p)
{
    pErrHndMem_l->mnCrcErr.thresholdCnt = thresholdCnt_p;
}

//------------------------------------------------------------------------------
/**
\brief  Set threshold counter of MnCycTimeExceed error

\param  thresholdCnt_p        Threshold counter to set.

\ingroup module_errhndkcal
*/
//------------------------------------------------------------------------------
void errhndkcal_setMnCycTimeExceedThresholdCnt(UINT32 thresholdCnt_p)
{
    pErrHndMem_l->mnCycTimeExceed.thresholdCnt = thresholdCnt_p;
}

//------------------------------------------------------------------------------
/**
\brief  Set threshold counter of MnCnLossPres error

\param  nodeIdx_p               Index of node (node ID - 1).
\param  thresholdCnt_p        Threshold counter to set.

\ingroup module_errhndkcal
*/
//------------------------------------------------------------------------------
void errhndkcal_setMnCnLossPresThresholdCnt(UINT nodeIdx_p,
                                            UINT32 thresholdCnt_p)
{
    pErrHndMem_l->aMnCnLossPres[nodeIdx_p].thresholdCnt = thresholdCnt_p;
}
#endif

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

///\}
