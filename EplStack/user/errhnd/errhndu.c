/**
********************************************************************************
\file   errhndu.c

\brief  Implementation of user part of error handler module

This file implements the user part of the error handler module.It is
responsible for linking the POWERLINK error counters into the object
dictionary. It contains object callback function which are responsible
to read values from kernel layer when reading the object or writing them
into kernel layer after writing the object. The communication with the
kernel part of the error handler is implemented by a error handler CAL
module.

\ingroup module_errhndu
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
#include <EplNmt.h>
#include <Benchmark.h>
#include <EplObd.h>
#include <kernel/EplObdk.h>

#include <errhnd.h>
#include <user/errhndu.h>

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
static tEplKernel linkErrorCounter(tErrorObject* pErrorCounter_p, UINT index_p);

#ifdef CONFIG_INCLUDE_NMT_MN
static tEplKernel checkErrorObject(UINT index_p, BYTE *pEntries_p);
static tEplKernel linkMnCnLossPresErrors(tErrHndObjects* pError_p);
#endif

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief    Initialize user error handler module

The function initializes the user error handler module.

\return Returns a tEplKernel error code.

\ingroup module_errhndu
*/
//------------------------------------------------------------------------------
tEplKernel errhndu_init(void)
{
    tEplKernel      ret;

    ret = kEplSuccessful;

    ret = errhnducal_init(&instance_l.errorObjects);
    if (ret != kEplSuccessful)
    {
        TRACE ("Couldn't init error handler CAL (%d)\n", ret);
        goto Exit;
    }

    // link counters to OD
    ret = linkErrorCounter(&instance_l.errorObjects.cnLossSoc, OID_DLL_CN_LOSSSOC_REC);
    if (ret != kEplSuccessful)
    {
        goto Exit;
    }

    ret = linkErrorCounter(&instance_l.errorObjects.cnLossPreq, OID_DLL_CN_LOSSPREQ_REC);
    // ignore return code, because object 0x1C0D is conditional

    ret = linkErrorCounter(&instance_l.errorObjects.cnCrcErr, OID_DLL_CN_CRCERROR_REC);
    if (ret != kEplSuccessful)
    {
        goto Exit;
    }

#ifdef CONFIG_INCLUDE_NMT_MN
    ret = linkErrorCounter(&instance_l.errorObjects.mnCrcErr, OID_DLL_MN_CRCERROR_REC);
    if (ret != kEplSuccessful)
    {
        goto Exit;
    }

    ret = linkErrorCounter(&instance_l.errorObjects.mnCycTimeExceed,
                           OID_DLL_MN_CYCTIME_EXCEED_REC);
    if (ret != kEplSuccessful)
    {
        goto Exit;
    }

    ret = linkMnCnLossPresErrors(&instance_l.errorObjects);
    if (ret != kEplSuccessful)
    {
        goto Exit;
    }
#endif

Exit:
    if (ret != kEplSuccessful)
    {
        errhnducal_exit();
    }
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief    Exit user error handler module

The function shuts down the user error handler module.

\return Returns a tEplKernel error code.

\ingroup module_errhndu
*/
//------------------------------------------------------------------------------
tEplKernel errhndu_exit()
{
    errhnducal_exit();
    return kEplSuccessful;
}

//------------------------------------------------------------------------------
/**
\brief    Error handler OD callback function

The function implements an OD callback function. It will be added to the
error objects so that error counters could be updated in shared memory by
PostWrite events and local objects will be updated from shared memory on
PreRead events.

\param  pParam_p            OD callback parameter

\return Returns always kEplSuccessful
*/
//------------------------------------------------------------------------------
tEplKernel errhndu_cbObdAccess(tEplObdCbParam MEM* pParam_p)
{
    switch (pParam_p->m_ObdEvent)
    {
        case kEplObdEvPostWrite:
        case kEplObdEvPostDefault:
            switch (pParam_p->m_uiSubIndex)
            {
                // only cumulative counter and threshold will be written by
                // application
                case SUBIDX_DLL_ERROR_CUM_CNT:
                case SUBIDX_DLL_ERROR_THRESHOLD:
                    errhnducal_writeErrorObject(pParam_p->m_uiIndex,
                                                pParam_p->m_uiSubIndex,
                                                (UINT32 *)pParam_p->m_pArg);
                    break;
            }
            break;

        case kEplObdEvPreRead:
            switch (pParam_p->m_uiSubIndex)
            {
                // the error handler only modifies the cumulative counter
                // and threshold counter
                case SUBIDX_DLL_ERROR_CUM_CNT:
                case SUBIDX_DLL_ERROR_THR_CNT:
                    errhnducal_readErrorObject(pParam_p->m_uiIndex,
                                               pParam_p->m_uiSubIndex,
                                               (UINT32 *)pParam_p->m_pArg);
                    break;
            }
            break;
        // other events must not be handled
        default:
            break;
    }
    return kEplSuccessful;
}

#ifdef CONFIG_INCLUDE_NMT_MN
//------------------------------------------------------------------------------
/**
\brief    Loss of PRes error handler OD callback function

The function implements an OD callback function for Loss of PRes errors.
There's a separate callback function because these errors counters are stored
as subindexes for each CN. The function will be added to the appropriate
error objects so that error counters could be updated in shared memory by
PostWrite events and local objects will be updated from shared memory on
PreRead objects.

\param  pParam_p            OD callback parameter

\return Returns always kEplSuccessful

\ingroup module_errhndu
*/
//------------------------------------------------------------------------------
tEplKernel errhndu_mnCnLossPresCbObdAccess(tEplObdCbParam MEM* pParam_p)
{
    tEplKernel          ret = kEplSuccessful;

    if (pParam_p->m_uiSubIndex == 0)
        return kEplSuccessful;

    switch (pParam_p->m_ObdEvent)
    {
        case kEplObdEvPostWrite:
        case kEplObdEvPostDefault:
            switch (pParam_p->m_uiIndex)
            {
                // only cumulative counter and threshold will be written by
                // application
                case OID_DLL_MNCN_LOSSPRES_CUMCNT_AU32:
                case OID_DLL_MNCN_LOSSPRES_THRESHOLD_AU32:
                    errhnducal_writeErrorObject(pParam_p->m_uiIndex,
                                                pParam_p->m_uiSubIndex,
                                                (UINT32 *)pParam_p->m_pArg);
                    break;
            }
            break;

        case kEplObdEvPreRead:
            switch (pParam_p->m_uiIndex)
            {
                // the error handler only modifies the cumulative counter
                // and threshold counter
                case OID_DLL_MNCN_LOSSPRES_CUMCNT_AU32:
                case OID_DLL_MNCN_LOSSPRES_THRCNT_AU32:
                    errhnducal_readErrorObject(pParam_p->m_uiIndex,
                                               pParam_p->m_uiSubIndex,
                                               (UINT32 *)pParam_p->m_pArg);
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

//------------------------------------------------------------------------------
/**
\brief    link error counter structure to OD entry

The function links an error counter structure to the according object
directory entry.

\param  pErrorCounter_p     Pointer to error counter structure
\param  index_p             OD index

\return Returns a tEplKernel error code.

\ingroup module_errhndu
*/
//------------------------------------------------------------------------------
static tEplKernel linkErrorCounter(tErrorObject* pErrorCounter_p, UINT index_p)
{
    tEplKernel      ret = kEplSuccessful;
    tEplVarParam    varParam;

    varParam.m_ValidFlag = kVarValidAll;
    varParam.m_uiIndex = index_p;
    varParam.m_Size = sizeof(UINT32);

    varParam.m_pData = &(pErrorCounter_p->cumulativeCnt);
    varParam.m_uiSubindex = 0x01;
    ret = EplObdDefineVar(&varParam);
    if (ret != kEplSuccessful)
        return ret;

    varParam.m_pData = &(pErrorCounter_p->thresholdCnt);
    varParam.m_uiSubindex = 0x02;
    ret = EplObdDefineVar(&varParam);
    if (ret != kEplSuccessful)
        return ret;

    varParam.m_pData = &(pErrorCounter_p->threshold);
    varParam.m_uiSubindex = 0x03;
    ret = EplObdDefineVar(&varParam);
    return ret;
}

#ifdef CONFIG_INCLUDE_NMT_MN
//------------------------------------------------------------------------------
/**
\brief    check if error object exists in OD

The function checks if the specified error object exists in the object
dictionary.

\param  index_p             Object index of object to check.
\param  pEntries_p          Pointer to store the number of entries of this
                            object.

\return Returns a tEplKernel error code.
\retval kEplSuccessful          If object exists
\retval kEplObdIndexNotExist    IF index does not exist

\ingroup module_errhndu
*/
//------------------------------------------------------------------------------
static tEplKernel checkErrorObject(UINT index_p, BYTE *pEntries_p)
{
    tEplKernel      ret = kEplSuccessful;
    tEplObdSize     entrySize;
    BYTE            indexEntries;

    entrySize = (tEplObdSize)  sizeof(indexEntries);
    ret = EplObdReadEntry ( index_p, 0x00, (void GENERIC*) &indexEntries, &entrySize );

    if ((ret != kEplSuccessful) || (indexEntries == 0x00))
    {
        // Object doesn't exist or invalid entry number
        return kEplObdIndexNotExist;
    }

    *pEntries_p = indexEntries;
    return kEplSuccessful;
}

//------------------------------------------------------------------------------
/**
\brief    Link Loss of PRes errors to object dictionary

The function links the Loss of PRes error to the object dictionary. These
error must be handled differently because its entries are stored as
subindexes for each node ID.

\param  pError_p            Pointer to error handler object data.

\return Returns a tEplKernel error code.

\ingroup module_errhndu
*/
//------------------------------------------------------------------------------
static tEplKernel linkMnCnLossPresErrors(tErrHndObjects* pError_p)
{
    tEplKernel      ret = kEplSuccessful;
    tEplVarParam    varParam;
    BYTE            indexEntries;
    BYTE            numObjs;
    tErrorObject   *pErrCnt;

    /* Check if error objects exist and use the minimum number of subindexes
     * for initialization as it makes no sense to have different number
     * of subindexes for threshold and counters.
     */
    ret = checkErrorObject(OID_DLL_MNCN_LOSSPRES_CUMCNT_AU32, &indexEntries);
    if (ret != kEplSuccessful)
        return kEplObdIndexNotExist;
    numObjs = indexEntries;

    ret = checkErrorObject(OID_DLL_MNCN_LOSSPRES_THRCNT_AU32, &indexEntries);
    if (ret != kEplSuccessful)
        return kEplObdIndexNotExist;
    numObjs = min(numObjs, indexEntries);

    ret = checkErrorObject(OID_DLL_MNCN_LOSSPRES_THRESHOLD_AU32, &indexEntries);
    if (ret != kEplSuccessful)
        return kEplObdIndexNotExist;
    numObjs = min(numObjs, indexEntries);

    varParam.m_Size = sizeof(UINT32);
    varParam.m_ValidFlag = kVarValidAll;
    pErrCnt = &(pError_p->aMnCnLossPres[0]);

    for (varParam.m_uiSubindex = 1; varParam.m_uiSubindex <= numObjs;
         varParam.m_uiSubindex++)
    {
        // CumulativeCnt
        varParam.m_uiIndex = OID_DLL_MNCN_LOSSPRES_CUMCNT_AU32;
        varParam.m_pData = &(pErrCnt->cumulativeCnt);
        ret = EplObdDefineVar(&varParam);
        if (ret != kEplSuccessful)
            break;

        // ThresholdCnt 1C08
        varParam.m_uiIndex = OID_DLL_MNCN_LOSSPRES_THRCNT_AU32;
        varParam.m_pData = &(pErrCnt->thresholdCnt);
        ret = EplObdDefineVar(&varParam);
        if (ret != kEplSuccessful)
            break;

        // Threshold 1C09
        varParam.m_uiIndex = OID_DLL_MNCN_LOSSPRES_THRESHOLD_AU32;
        varParam.m_pData = &(pErrCnt->threshold);
        ret = EplObdDefineVar(&varParam);
        if (ret != kEplSuccessful)
            break;

        pErrCnt++;
    }
    return ret;
}

#endif

