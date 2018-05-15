/**
********************************************************************************
\file   errhndu.c

\brief  Implementation of user part of error handler module

This file implements the user part of the error handler module. It is
responsible for linking the POWERLINK error counters into the object
dictionary. It contains object callback function which are responsible
to read values from kernel layer when reading the object or writing them
into kernel layer after writing the object. The communication with the
kernel part of the error handler is implemented via an error handler CAL
module.

\ingroup module_errhndu
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
#include <common/oplkinc.h>
#include <user/errhndu.h>
#include <user/obdu.h>
#include "errhnducal.h"

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
/**
\brief  instance of user error handler

The structure defines the instance variables of the user error handler.
*/
typedef struct
{
    tErrHndObjects      errorObjects;       ///< Contains the supported error objects
} tErrHnduInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tErrHnduInstance        instance_l;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static tOplkError linkErrorCounter(tErrorObject* pErrorCounter_p, UINT index_p);

#ifdef CONFIG_INCLUDE_NMT_MN
static tOplkError checkErrorObject(UINT index_p, UINT8* pEntries_p);
static tOplkError linkMnCnLossPresErrors(tErrHndObjects* pError_p);
#endif

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief    Initialize user error handler module

The function initializes the user error handler module.

\return Returns a tOplkError error code.

\ingroup module_errhndu
*/
//------------------------------------------------------------------------------
tOplkError errhndu_init(void)
{
    tOplkError  ret;

    ret = errhnducal_init(&instance_l.errorObjects);
    if (ret != kErrorOk)
    {
        DEBUG_LVL_ERROR_TRACE("Couldn't init error handler CAL (%d)\n", ret);
        goto Exit;
    }

    // link counters to OD
    ret = linkErrorCounter(&instance_l.errorObjects.cnLossSoc, OID_DLL_CN_LOSSSOC_REC);
    if (ret != kErrorOk)
        goto Exit;

    linkErrorCounter(&instance_l.errorObjects.cnLossPreq, OID_DLL_CN_LOSSPREQ_REC);
    // ignore return code, because object 0x1C0D is conditional

    ret = linkErrorCounter(&instance_l.errorObjects.cnCrcErr, OID_DLL_CN_CRCERROR_REC);
    if (ret != kErrorOk)
        goto Exit;

#ifdef CONFIG_INCLUDE_NMT_MN
    ret = linkErrorCounter(&instance_l.errorObjects.mnCrcErr, OID_DLL_MN_CRCERROR_REC);
    if (ret != kErrorOk)
        goto Exit;

    ret = linkErrorCounter(&instance_l.errorObjects.mnCycTimeExceed,
                           OID_DLL_MN_CYCTIME_EXCEED_REC);
    if (ret != kErrorOk)
        goto Exit;

    ret = linkMnCnLossPresErrors(&instance_l.errorObjects);
    if (ret != kErrorOk)
        goto Exit;
#endif

Exit:
    if (ret != kErrorOk)
        errhnducal_exit();

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief    Exit user error handler module

The function shuts down the user error handler module.

\return Returns a tOplkError error code.

\ingroup module_errhndu
*/
//------------------------------------------------------------------------------
tOplkError errhndu_exit()
{
    errhnducal_exit();

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief    Error handler OD callback function

The function implements an OD callback function. It will be added to the
error objects so that error counters can be updated in shared memory by
PostWrite events and local objects will be updated from shared memory on
PreRead events.

\param[in]      pParam_p            OD callback parameter

\return Returns always kErrorOk
*/
//------------------------------------------------------------------------------
tOplkError errhndu_cbObdAccess(const tObdCbParam* pParam_p)
{
    // Check parameter validity
    ASSERT(pParam_p != NULL);

    switch (pParam_p->obdEvent)
    {
        case kObdEvPostDefault:
            if (pParam_p->subIndex == SUBIDX_DLL_ERROR_CUM_CNT)
                break;

            // fall through!
        case kObdEvPostWrite:
            switch (pParam_p->subIndex)
            {
                // only cumulative counter and threshold will be written by
                // the application
                case SUBIDX_DLL_ERROR_CUM_CNT:
                case SUBIDX_DLL_ERROR_THRESHOLD:
                    errhnducal_writeErrorObject(pParam_p->index,
                                                pParam_p->subIndex,
                                                (const UINT32*)pParam_p->pArg);
                    break;
            }
            break;

        case kObdEvPreRead:
            switch (pParam_p->subIndex)
            {
                // the error handler only modifies the cumulative counter
                // and threshold counter
                case SUBIDX_DLL_ERROR_CUM_CNT:
                case SUBIDX_DLL_ERROR_THR_CNT:
                    errhnducal_readErrorObject(pParam_p->index,
                                               pParam_p->subIndex,
                                               (UINT32*)pParam_p->pArg);
                    break;
            }
            break;
        // other events must not be handled
        default:
            break;
    }

    return kErrorOk;
}

#ifdef CONFIG_INCLUDE_NMT_MN
//------------------------------------------------------------------------------
/**
\brief    Loss of PRes error handler OD callback function

The function implements an OD callback function for Loss of PRes errors.
There is a separate callback function because these error counters are stored
as subindexes for each CN. The function will be added to the appropriate
error objects so that error counters can be updated in shared memory by
PostWrite events and local objects will be updated from shared memory on
PreRead objects.

\param[in]      pParam_p            OD callback parameter

\return Returns always kErrorOk

\ingroup module_errhndu
*/
//------------------------------------------------------------------------------
tOplkError errhndu_mnCnLossPresCbObdAccess(const tObdCbParam* pParam_p)
{
    tOplkError  ret = kErrorOk;

    // Check parameter validity
    ASSERT(pParam_p != NULL);

    if (pParam_p->subIndex == 0)
        return kErrorOk;

    switch (pParam_p->obdEvent)
    {
        case kObdEvPostDefault:
            if (pParam_p->index == OID_DLL_MNCN_LOSSPRES_CUMCNT_AU32)
                break;

            // fall through!
        case kObdEvPostWrite:
            switch (pParam_p->index)
            {
                // only cumulative counter and threshold will be written by
                // the application
                case OID_DLL_MNCN_LOSSPRES_CUMCNT_AU32:
                case OID_DLL_MNCN_LOSSPRES_THRESHOLD_AU32:
                    errhnducal_writeErrorObject(pParam_p->index,
                                                pParam_p->subIndex,
                                                (const UINT32*)pParam_p->pArg);
                    break;
            }
            break;

        case kObdEvPreRead:
            switch (pParam_p->index)
            {
                // the error handler only modifies the cumulative counter
                // and threshold counter
                case OID_DLL_MNCN_LOSSPRES_CUMCNT_AU32:
                case OID_DLL_MNCN_LOSSPRES_THRCNT_AU32:
                    errhnducal_readErrorObject(pParam_p->index,
                                               pParam_p->subIndex,
                                               (UINT32*)pParam_p->pArg);
                    break;
            }
            break;
        // other events must not be handled
        default:
            break;

    }
    return ret;
}
#endif

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief    Link error counter structure to OD entry

The function links an error counter structure to the according object
directory entry.

\param[in]      pErrorCounter_p     Pointer to error counter structure
\param[in]      index_p             OD index

\return Returns a tOplkError error code.

\ingroup module_errhndu
*/
//------------------------------------------------------------------------------
static tOplkError linkErrorCounter(tErrorObject* pErrorCounter_p, UINT index_p)
{
    tOplkError  ret = kErrorOk;
    tVarParam   varParam;

    varParam.validFlag = kVarValidAll;
    varParam.index = index_p;
    varParam.size = sizeof(UINT32);

    varParam.pData = &(pErrorCounter_p->cumulativeCnt);
    varParam.subindex = 0x01;
    ret = obdu_defineVar(&varParam);
    if (ret != kErrorOk)
        return ret;

    varParam.pData = &(pErrorCounter_p->thresholdCnt);
    varParam.subindex = 0x02;
    ret = obdu_defineVar(&varParam);
    if (ret != kErrorOk)
        return ret;

    varParam.pData = &(pErrorCounter_p->threshold);
    varParam.subindex = 0x03;
    ret = obdu_defineVar(&varParam);

    return ret;
}

#ifdef CONFIG_INCLUDE_NMT_MN
//------------------------------------------------------------------------------
/**
\brief    Check if error object exists in OD

The function checks if the specified error object exists in the object
dictionary.

\param[in]      index_p             Object index of object to check.
\param[out]     pEntries_p          Pointer to store the number of entries of this
                                    object.

\return Returns a tOplkError error code.
\retval kErrorOk                    Object exists
\retval kErrorObdIndexNotExist      Index does not exist

\ingroup module_errhndu
*/
//------------------------------------------------------------------------------
static tOplkError checkErrorObject(UINT index_p, UINT8* pEntries_p)
{
    tOplkError  ret;
    UINT8       indexEntries;
    tObdSize    entrySize = (tObdSize)sizeof(indexEntries);

    ret = obdu_readEntry(index_p, 0x00, (void*)&indexEntries, &entrySize);
    if ((ret != kErrorOk) || (indexEntries == 0x00))
    {
        // Object doesn't exist or invalid entry number
        return kErrorObdIndexNotExist;
    }

    *pEntries_p = indexEntries;

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief    Link Loss of PRes errors to object dictionary

The function links the Loss of PRes error to the object dictionary. This
error must be handled differently because its entries are stored as
subindexes for each node ID.

\param[in]      pError_p            Pointer to error handler object data.

\return Returns a tOplkError error code.

\ingroup module_errhndu
*/
//------------------------------------------------------------------------------
static tOplkError linkMnCnLossPresErrors(tErrHndObjects* pError_p)
{
    tOplkError      ret;
    tVarParam       varParam;
    UINT8           indexEntries;
    UINT8           numObjs;
    tErrorObject*   pErrCnt;

    /* Check if error objects exist and use the minimum number of subindexes
     * for initialization as it makes no sense to have different number
     * of subindexes for threshold and counters.
     */
    ret = checkErrorObject(OID_DLL_MNCN_LOSSPRES_CUMCNT_AU32, &indexEntries);
    if (ret != kErrorOk)
        return kErrorObdIndexNotExist;
    numObjs = indexEntries;

    ret = checkErrorObject(OID_DLL_MNCN_LOSSPRES_THRCNT_AU32, &indexEntries);
    if (ret != kErrorOk)
        return kErrorObdIndexNotExist;
    numObjs = min(numObjs, indexEntries);

    ret = checkErrorObject(OID_DLL_MNCN_LOSSPRES_THRESHOLD_AU32, &indexEntries);
    if (ret != kErrorOk)
        return kErrorObdIndexNotExist;
    numObjs = min(numObjs, indexEntries);

    varParam.size = sizeof(UINT32);
    varParam.validFlag = kVarValidAll;
    pErrCnt = &(pError_p->aMnCnLossPres[0]);

    for (varParam.subindex = 1; varParam.subindex <= numObjs;
         varParam.subindex++)
    {
        // CumulativeCnt
        varParam.index = OID_DLL_MNCN_LOSSPRES_CUMCNT_AU32;
        varParam.pData = &(pErrCnt->cumulativeCnt);
        ret = obdu_defineVar(&varParam);
        if (ret != kErrorOk)
            break;

        // ThresholdCnt 1C08
        varParam.index = OID_DLL_MNCN_LOSSPRES_THRCNT_AU32;
        varParam.pData = &(pErrCnt->thresholdCnt);
        ret = obdu_defineVar(&varParam);
        if (ret != kErrorOk)
            break;

        // Threshold 1C09
        varParam.index = OID_DLL_MNCN_LOSSPRES_THRESHOLD_AU32;
        varParam.pData = &(pErrCnt->threshold);
        ret = obdu_defineVar(&varParam);
        if (ret != kErrorOk)
            break;

        pErrCnt++;
    }

    return ret;
}

#endif

/// \}
