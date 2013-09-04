/**
********************************************************************************
\file   cfmu.c

\brief  Implementation of configuration file manager (CFM) module

This file contains the implementation of the configuration file manager (CFM).

\ingroup module_cfmu
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2013, SYSTEC electronic GmbH
Copyright (c) 2013, Kalycito Infotech Private Ltd.All rights reserved.
Copyright (c) 2013, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
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
#include <limits.h>

#include <EplInc.h>
#include <user/cfmu.h>
#include <EplSdoAc.h>
#include <user/identu.h>
#include <user/EplObdu.h>
#include <user/EplSdoComu.h>
#include <user/nmtu.h>

#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_OBDU)) == 0) && (EPL_OBD_USE_KERNEL == FALSE)
#error "CFM module needs openPOWERLINK module OBDU or OBDK!"
#endif

#if !defined(CONFIG_INCLUDE_SDOC)
#error "CFM module needs openPOWERLINK module SDO client!"
#endif

#if (EPL_OBD_USE_STRING_DOMAIN_IN_RAM == FALSE)
#error "CFM module needs define EPL_OBD_USE_STRING_DOMAIN_IN_RAM == TRUE"
#endif

//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#ifndef EPL_CFM_CONFIGURE_CYCLE_LENGTH
#define EPL_CFM_CONFIGURE_CYCLE_LENGTH  FALSE
#endif

// return pointer to node info structure for specified node ID
// d.k. may be replaced by special (hash) function if node ID array is smaller than 254
#define CFM_GET_NODEINFO(uiNodeId_p) (cfmInstance_g.apNodeInfo[uiNodeId_p - 1])

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
\brief Enumeration for CFM states

The following enumeration lists all valid CFM states.
*/
typedef enum
{
    kCfmStateIdle           = 0x00,
    kCfmStateWaitRestore,
    kCfmStateDownload,
    kCfmStateWaitStore,
    kCfmStateUpToDate,
    kCfmStateInternalAbort,
} tCfmState;

/**
\brief CFM node information structure

The following structure defines the node information that is needed by the
configuration manager.
*/
typedef struct
{
    tCfmEventCnProgress     eventCnProgress;
    UINT8*                  pObdBufferConciseDcf;
    UINT8*                  pDataConciseDcf;
    UINT32                  bytesRemaining;
    UINT32                  entriesRemaining;
    tEplSdoComConHdl        sdoComConHdl;
    tCfmState               cfmState;
    UINT                    curDataSize;
    BOOL                    fDoStore;
} tCfmNodeInfo;

/**
\brief CFM instance

The following structure defines the instance of the configuration manager.
*/
typedef struct
{
    tCfmNodeInfo*           apNodeInfo[EPL_NMT_MAX_NODE_ID];
    UINT32                  leDomainSizeNull;
#if (EPL_CFM_CONFIGURE_CYCLE_LENGTH != FALSE)
    UINT32                  leCycleLength;
#endif
    tCfmCbEventCnProgress   pfnCbEventCnProgress;
    tCfmCbEventCnResult     pfnCbEventCnResult;
} tCfmInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tCfmInstance             cfmInstance_g;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------

static tCfmNodeInfo* allocNodeInfo(UINT nodeId_p);
static tEplKernel callCbProgress(tCfmNodeInfo* pNodeInfo_p);
static tEplKernel downloadCycleLength(tCfmNodeInfo* pNodeInfo_p);
static tEplKernel downloadObject(tCfmNodeInfo* pNodeInfo_p);
static tEplKernel sdoWriteObject(tCfmNodeInfo* pNodeInfo_p, void* pLeSrcData_p, UINT size_p);
static tEplKernel cbSdoCon(tEplSdoComFinished* pSdoComFinished_p);

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Init CFM module

The function initializes the CFM module.

\param  pfnCbEventCnProgress_p      Pointer to callback function for CN progress
                                    events.
\param  pfnCbEventCnResult_p          Pointer to callback function for CN result
                                    events.

\return The function returns a tEplKernel error code.

\ingroup module_cfmu
*/
//------------------------------------------------------------------------------
tEplKernel cfmu_init(tCfmCbEventCnProgress pfnCbEventCnProgress_p,
                     tCfmCbEventCnResult pfnCbEventCnResult_p)
{
    tEplKernel      ret = kEplSuccessful;
    UINT            subindex;
    tEplVarParam    varParam;

    EPL_MEMSET(&cfmInstance_g, 0, sizeof(tCfmInstance));

    cfmInstance_g.pfnCbEventCnProgress = pfnCbEventCnProgress_p;
    cfmInstance_g.pfnCbEventCnResult = pfnCbEventCnResult_p;

    // link domain with 4 zero-bytes to object 0x1F22 CFM_ConciseDcfList_ADOM
    varParam.m_pData = &cfmInstance_g.leDomainSizeNull;
    varParam.m_Size = sizeof(cfmInstance_g.leDomainSizeNull);
    varParam.m_uiIndex = 0x1F22;    // CFM_ConciseDcfList_ADOM
    for (subindex = 1; subindex <= EPL_NMT_MAX_NODE_ID; subindex++)
    {
        varParam.m_uiSubindex = subindex;
        varParam.m_ValidFlag = kVarValidAll;
        ret = EplObdDefineVar(&varParam);
        if ((ret != kEplSuccessful) &&
            (ret != kEplObdIndexNotExist) &&
            (ret != kEplObdSubindexNotExist))
        {
            return ret;
        }
    }
    return kEplSuccessful;
}

//------------------------------------------------------------------------------
/**
\brief  Exit CFM module

The function deinitializes the CFM module.

\return The function returns a tEplKernel error code.

\ingroup module_cfmu
*/
//------------------------------------------------------------------------------
tEplKernel cfmu_exit(void)
{
    UINT                nodeId;
    tEplVarParam        varParam;
    UINT8*              pBuffer;
    tCfmNodeInfo*       pNodeInfo;

    // free domain for object 0x1F22 CFM_ConciseDcfList_ADOM
    varParam.m_pData = NULL;
    varParam.m_Size = 0;
    varParam.m_uiIndex = 0x1F22;    //CFM_ConciseDcfList_ADOM
    for (nodeId = 1; nodeId <= EPL_NMT_MAX_NODE_ID; nodeId++)
    {
        pNodeInfo = CFM_GET_NODEINFO(nodeId);
        if (pNodeInfo != NULL)
        {
            if (pNodeInfo->sdoComConHdl != UINT_MAX)
            {
                EplSdoComSdoAbort(pNodeInfo->sdoComConHdl, EPL_SDOAC_DATA_NOT_TRANSF_DUE_DEVICE_STATE);
            }

            pBuffer = pNodeInfo->pObdBufferConciseDcf;
            if (pBuffer != NULL)
            {
                varParam.m_uiSubindex = nodeId;
                varParam.m_ValidFlag = kVarValidAll;
                EplObdDefineVar(&varParam);
                // ignore return code, because buffer has to be freed anyway

                EPL_FREE(pBuffer);
                pNodeInfo->pObdBufferConciseDcf = NULL;
            }
            EPL_FREE(pNodeInfo);
            CFM_GET_NODEINFO(nodeId) = NULL;
        }
    }

    return kEplSuccessful;
}

//------------------------------------------------------------------------------
/**
\brief  Process node event

The function processes a node event. It starts configuration of the specified
CN if configuration data and time differs with local values.

\param  nodeId_p        Node ID of node to configure.
\param  nodeEvent_p     Node event to process.

\return The function returns a tEplKernel error code.
\retval kEplSuccessful  Configuration is OK -> continue boot process for this CN.
\retval kEplReject      Defer further processing until configuration process has finished
\retval other error     Major error occurred.

\ingroup module_cfmu
*/
//------------------------------------------------------------------------------
tEplKernel cfmu_processNodeEvent(UINT nodeId_p, tNmtNodeEvent nodeEvent_p)
{
    tEplKernel          ret = kEplSuccessful;
    static UINT32       leSignature;
    tCfmNodeInfo*       pNodeInfo = NULL;
    tEplObdSize         obdSize;
    UINT32              expConfTime = 0;
    UINT32              expConfDate = 0;
    tEplIdentResponse*  pIdentResponse = NULL;
    BOOL                fDoUpdate = FALSE;

    if ((nodeEvent_p != kNmtNodeEventCheckConf) && (nodeEvent_p != kNmtNodeEventUpdateConf))
        return ret;

    if ((pNodeInfo = allocNodeInfo(nodeId_p)) == NULL)
        return kEplInvalidNodeId;

    if (pNodeInfo->cfmState != kCfmStateIdle)
    {
        // send abort
        pNodeInfo->cfmState = kCfmStateInternalAbort;
        ret = EplSdoComSdoAbort(pNodeInfo->sdoComConHdl, EPL_SDOAC_DATA_NOT_TRANSF_DUE_LOCAL_CONTROL);
        if (ret != kEplSuccessful)
            return ret;

        // close connection
        ret = EplSdoComUndefineCon(pNodeInfo->sdoComConHdl);
        pNodeInfo->sdoComConHdl = UINT_MAX;
        if (ret != kEplSuccessful)
        {
            EPL_DBGLVL_CFM_TRACE("SDO Free Error!\n");
            return ret;
        }
    }

    pNodeInfo->curDataSize = 0;

    // fetch pointer to ConciseDCF from object 0x1F22
    // (this allows the application to link its own memory to this object)
    pNodeInfo->pDataConciseDcf = EplObduGetObjectDataPtr(0x1F22, nodeId_p);
    if (pNodeInfo->pDataConciseDcf == NULL)
        return kEplCfmNoConfigData;

    obdSize = EplObduGetDataSize(0x1F22, nodeId_p);
    pNodeInfo->bytesRemaining = (UINT32) obdSize;
    pNodeInfo->eventCnProgress.totalNumberOfBytes = pNodeInfo->bytesRemaining;
#if (EPL_CFM_CONFIGURE_CYCLE_LENGTH != FALSE)
    pNodeInfo->eventCnProgress.totalNumberOfBytes += sizeof (UINT32);
#endif
    pNodeInfo->eventCnProgress.bytesDownloaded = 0;
    if (obdSize < sizeof(UINT32))
    {
        pNodeInfo->eventCnProgress.error = kEplCfmInvalidDcf;
        ret = callCbProgress(pNodeInfo);
        if (ret != kEplSuccessful)
            return ret;
        return pNodeInfo->eventCnProgress.error;
    }

    pNodeInfo->entriesRemaining = AmiGetDwordFromLe(pNodeInfo->pDataConciseDcf);
    pNodeInfo->pDataConciseDcf += sizeof(UINT32);
    pNodeInfo->bytesRemaining -= sizeof(UINT32);
    pNodeInfo->eventCnProgress.bytesDownloaded += sizeof(UINT32);

    if (pNodeInfo->entriesRemaining == 0)
    {
        pNodeInfo->eventCnProgress.error = kEplCfmNoConfigData;
        ret = callCbProgress(pNodeInfo);
        if (ret != kEplSuccessful)
            return ret;
    }
    else
    {
        obdSize = sizeof (expConfDate);
        ret = EplObduReadEntry(0x1F26, nodeId_p, &expConfDate, &obdSize);
        if (ret != kEplSuccessful)
        {
            EPL_DBGLVL_CFM_TRACE("CN%x Error Reading 0x1F26 returns 0x%X\n", uiNodeId_p, ret);
        }
        obdSize = sizeof (expConfTime);
        ret = EplObduReadEntry(0x1F27, nodeId_p, &expConfTime, &obdSize);
        if (ret != kEplSuccessful)
        {
            EPL_DBGLVL_CFM_TRACE("CN%x Error Reading 0x1F27 returns 0x%X\n", uiNodeId_p, ret);
        }
        if ((expConfDate != 0) || (expConfTime != 0))
        {   // store configuration in CN at the end of the download,
            // because expected configuration date or time is set
            pNodeInfo->fDoStore = TRUE;
            pNodeInfo->eventCnProgress.totalNumberOfBytes += sizeof (UINT32);
        }
        else
        {   // expected configuration date and time is not set
            fDoUpdate = TRUE;
        }
        identu_getIdentResponse(nodeId_p, &pIdentResponse);
        if (pIdentResponse == NULL)
        {
            EPL_DBGLVL_CFM_TRACE("CN%x Ident Response is NULL\n", uiNodeId_p);
            return kEplInvalidNodeId;
        }
    }

#if (EPL_CFM_CONFIGURE_CYCLE_LENGTH != FALSE)
    obdSize = sizeof(cfmInstance_g.leCycleLength);
    ret = EplObduReadEntryToLe(0x1006, 0x00, &cfmInstance_g.leCycleLength, &obdSize);
    if (ret != kEplSuccessful)
    {   // local OD access failed
        EPL_DBGLVL_CFM_TRACE("Local OBD read failed %d\n", ret);
        return ret;
    }
#endif

    if ((pNodeInfo->entriesRemaining == 0) ||
        ((nodeEvent_p != kNmtNodeEventUpdateConf) && (fDoUpdate == FALSE) &&
         ((AmiGetDwordFromLe(&pIdentResponse->m_le_dwVerifyConfigurationDate) == expConfDate) &&
          (AmiGetDwordFromLe(&pIdentResponse->m_le_dwVerifyConfigurationTime) == expConfTime))))
    {
        pNodeInfo->cfmState = kCfmStateIdle;

        // current version is already available on the CN, no need to write new values, we can continue
        EPL_DBGLVL_CFM_TRACE("CN%x - Cfg Upto Date\n", nodeId_p);

        ret = downloadCycleLength(pNodeInfo);
        if (ret == kEplReject)
        {
            pNodeInfo->cfmState = kCfmStateUpToDate;
        }
    }
    else if (nodeEvent_p == kNmtNodeEventUpdateConf)
    {
        pNodeInfo->cfmState = kCfmStateDownload;
        ret = downloadObject(pNodeInfo);
        if (ret == kEplSuccessful)
        {   // SDO transfer started
            ret = kEplReject;
        }
    }
    else
    {
        pNodeInfo->cfmState = kCfmStateWaitRestore;

        pNodeInfo->eventCnProgress.totalNumberOfBytes += sizeof(leSignature);
        AmiSetDwordToLe(&leSignature, 0x64616F6C);
        //Restore Default Parameters
        EPL_DBGLVL_CFM_TRACE("CN%x - Cfg Mismatch | MN Expects: %lx-%lx ", nodeId_p, expConfDate, expConfTime);
        EPL_DBGLVL_CFM_TRACE("CN Has: %lx-%lx. Restoring Default...\n",
                             AmiGetDwordFromLe(&pIdentResponse->m_le_dwVerifyConfigurationDate),
                             AmiGetDwordFromLe(&pIdentResponse->m_le_dwVerifyConfigurationTime));

        pNodeInfo->eventCnProgress.objectIndex = 0x1011;
        pNodeInfo->eventCnProgress.objectSubIndex = 0x01;
        ret = sdoWriteObject(pNodeInfo, &leSignature, sizeof (leSignature));
        if (ret == kEplSuccessful)
        {   // SDO transfer started
            ret = kEplReject;
        }
        else
        {
            // error occurred
            EPL_DBGLVL_CFM_TRACE("CfmCbEvent(Node): sdoWriteObject() returned 0x%02X\n", Ret);
        }
    }
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Determine if SDO is running

The function determines if the a SDO is running for the specified node.

\param  nodeId_p        Node ID of node to determine SDO state.

\return The function returns TRUE if SDO is running and FALSE otherwise.

\ingroup module_cfmu
*/
//------------------------------------------------------------------------------
BOOL cfmu_isSdoRunning(UINT nodeId_p)
{
    tCfmNodeInfo*       pNodeInfo = NULL;

    if ((nodeId_p == 0) || (nodeId_p > EPL_NMT_MAX_NODE_ID))
        return FALSE;

    pNodeInfo = CFM_GET_NODEINFO(nodeId_p);
    if (pNodeInfo == NULL)
        return FALSE;

    if (pNodeInfo->cfmState != kCfmStateIdle)
        return TRUE;

    return FALSE;
}

//------------------------------------------------------------------------------
/**
\brief  Callback function for OD accesses

The function implements the callback function which is called on OD accesses.

\param  pParam_p        OD callback parameter.

\return The function returns a tEplKernel error code.

\ingroup module_cfmu
*/
//------------------------------------------------------------------------------
tEplKernel cfmu_cbObdAccess(tEplObdCbParam MEM* pParam_p)
{
    tEplKernel              ret = kEplSuccessful;
    tEplObdVStringDomain*   pMemVStringDomain;
    tCfmNodeInfo*           pNodeInfo = NULL;
    UINT8*                  pBuffer;

    pParam_p->m_dwAbortCode = 0;

    if ((pParam_p->m_uiIndex != 0x1F22) || (pParam_p->m_ObdEvent != kEplObdEvWrStringDomain))
        return ret;

    // abort any running SDO transfer
    pNodeInfo = CFM_GET_NODEINFO(pParam_p->m_uiSubIndex);
    if ((pNodeInfo != NULL) && (pNodeInfo->sdoComConHdl != UINT_MAX))
    {
        ret = EplSdoComSdoAbort(pNodeInfo->sdoComConHdl, EPL_SDOAC_DATA_NOT_TRANSF_DUE_DEVICE_STATE);
    }

    pMemVStringDomain = pParam_p->m_pArg;
    if ((pMemVStringDomain->m_ObjSize != pMemVStringDomain->m_DownloadSize) ||
        (pMemVStringDomain->m_pData == NULL))
    {
        pNodeInfo = allocNodeInfo(pParam_p->m_uiSubIndex);
        if (pNodeInfo == NULL)
        {
            pParam_p->m_dwAbortCode = EPL_SDOAC_OUT_OF_MEMORY;
            return kEplNoResource;
        }

        pBuffer = pNodeInfo->pObdBufferConciseDcf;
        if (pBuffer != NULL)
        {
            EPL_FREE(pBuffer);
            pNodeInfo->pObdBufferConciseDcf = NULL;
        }
        pBuffer = EPL_MALLOC(pMemVStringDomain->m_DownloadSize);
        if (pBuffer == NULL)
        {
            pParam_p->m_dwAbortCode = EPL_SDOAC_OUT_OF_MEMORY;
            return kEplNoResource;
        }
        pNodeInfo->pObdBufferConciseDcf = pBuffer;
        pMemVStringDomain->m_pData = pBuffer;
        pMemVStringDomain->m_ObjSize = pMemVStringDomain->m_DownloadSize;
    }

    return ret;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief  Allocate node information

The function allocates a node info structure for the specified node.

\param  nodeId_p        Node ID for which to allocate the node info structure.

\return The function returns a pointer to the allocated node info structure.
*/
//------------------------------------------------------------------------------
static tCfmNodeInfo* allocNodeInfo(UINT nodeId_p)
{
    tCfmNodeInfo*   pNodeInfo = NULL;

    if ((nodeId_p == 0) || (nodeId_p > EPL_NMT_MAX_NODE_ID))
        return NULL;

    pNodeInfo = CFM_GET_NODEINFO(nodeId_p);
    if (pNodeInfo != NULL)
        return pNodeInfo;

    pNodeInfo = EPL_MALLOC(sizeof(tCfmNodeInfo));
    EPL_MEMSET(pNodeInfo, 0, sizeof(tCfmNodeInfo));
    pNodeInfo->eventCnProgress.nodeId = nodeId_p;
    pNodeInfo->sdoComConHdl = UINT_MAX;

    CFM_GET_NODEINFO(nodeId_p) = pNodeInfo;
    return pNodeInfo;
}

//------------------------------------------------------------------------------
/**
\brief  Call progress callback function

The function calls the progress callback function of the specified node.

\param  pNodeInfo_p     Node info of the node to call the progress callback
                        function.

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
static tEplKernel callCbProgress(tCfmNodeInfo* pNodeInfo_p)
{
    tEplKernel      ret = kEplSuccessful;

    if (cfmInstance_g.pfnCbEventCnProgress != NULL)
    {
        ret = cfmInstance_g.pfnCbEventCnProgress(&pNodeInfo_p->eventCnProgress);
    }
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Finish configuration by calling result callback function

The function calls the result callback function of the specified node.

\param  pNodeInfo_p     Node info of the node to call the result callback function.
\param  nmtCommand_p    NMT command to execute in the result callback function.

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
static tEplKernel finishConfig(tCfmNodeInfo* pNodeInfo_p, tNmtCommand nmtCommand_p)
{
    tEplKernel      ret = kEplSuccessful;

    if (pNodeInfo_p->sdoComConHdl != UINT_MAX)
    {
        ret = EplSdoComUndefineCon(pNodeInfo_p->sdoComConHdl);
        pNodeInfo_p->sdoComConHdl = UINT_MAX;
        if (ret != kEplSuccessful)
        {
            EPL_DBGLVL_CFM_TRACE("SDO Free Error!\n");
            return ret;
        }
    }

    pNodeInfo_p->cfmState = kCfmStateIdle;
    if (cfmInstance_g.pfnCbEventCnResult != NULL)
    {
        ret = cfmInstance_g.pfnCbEventCnResult(pNodeInfo_p->eventCnProgress.nodeId, nmtCommand_p);
    }
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  SDO finished callback function

The function implements the callback function which is called when the SDO
transfer is finished.

\param  pSdoComFinished_p   Pointer to SDO COM finished structure.

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
static tEplKernel cbSdoCon(tEplSdoComFinished* pSdoComFinished_p)
{
    tEplKernel          ret = kEplSuccessful;
    tCfmNodeInfo*       pNodeInfo = pSdoComFinished_p->m_pUserArg;
    tNmtCommand         nmtCommand;

    if (pNodeInfo == NULL)
        return kEplInvalidNodeId;

    pNodeInfo->eventCnProgress.sdoAbortCode = pSdoComFinished_p->m_dwAbortCode;
    pNodeInfo->eventCnProgress.bytesDownloaded += pSdoComFinished_p->m_uiTransferredByte;

    if ((ret = callCbProgress(pNodeInfo)) != kEplSuccessful)
        return ret;

    switch (pNodeInfo->cfmState)
    {
        case kCfmStateIdle:
            ret = finishConfig(pNodeInfo, kNmtNodeCommandConfErr);
            break;

        case kCfmStateUpToDate:
            if (pSdoComFinished_p->m_SdoComConState == kEplSdoComTransferFinished)
                nmtCommand = kNmtNodeCommandConfReset;  // continue boot-up of CN with NMT command Reset Configuration
            else
                nmtCommand = kNmtNodeCommandConfErr;   // indicate configuration error CN

            ret = finishConfig(pNodeInfo, nmtCommand);
            break;

        case kCfmStateDownload:
            if (pSdoComFinished_p->m_SdoComConState == kEplSdoComTransferFinished)
                ret = downloadObject(pNodeInfo);
            else
                ret = finishConfig(pNodeInfo, kNmtNodeCommandConfErr);      // configuration was not successful
            break;

        case kCfmStateWaitRestore:
            if (pSdoComFinished_p->m_SdoComConState == kEplSdoComTransferFinished)
            {   // configuration successfully restored
                EPL_DBGLVL_CFM_TRACE("\nCN%x - Restore Complete. Resetting Node...\n", pNodeInfo->eventCnProgress.m_uiNodeId);
                // send NMT command reset node to activate the original configuration
                ret = finishConfig(pNodeInfo, kNmtNodeCommandConfRestored);
            }
            else
            {   // restore configuration not available
                // start downloading the ConciseDCF
                pNodeInfo->cfmState = kCfmStateDownload;
                ret = downloadObject(pNodeInfo);
            }
            break;

        case kCfmStateWaitStore:
            if ((ret = downloadCycleLength(pNodeInfo)) == kEplReject)
            {
                pNodeInfo->cfmState = kCfmStateUpToDate;
                ret = kEplSuccessful;
            }
            else
            {
                ret = finishConfig(pNodeInfo, kNmtNodeCommandConfReset);
            }
            break;

        case kCfmStateInternalAbort:
            // configuration was aborted
            break;
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Download cycle length

The function reads the specified entry from the OD of the specified node.
If this node is a remote node, it performs a SDO transfer, which means this
function returns kEplApiTaskDeferred and the application is informed via
the event callback function when the task is completed.

\param  pNodeInfo_p     Node info of the node for which to download the cycle
                        length.

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
static tEplKernel downloadCycleLength(tCfmNodeInfo* pNodeInfo_p)
{
    tEplKernel      ret = kEplSuccessful;

#if (EPL_CFM_CONFIGURE_CYCLE_LENGTH != FALSE)
    pNodeInfo_p->eventCnProgress.objectIndex = 0x1006;
    pNodeInfo_p->eventCnProgress.objectSubIndex = 0x00;

    ret = sdoWriteObject(pNodeInfo_p, &cfmInstance_g.leCycleLength, sizeof(UINT32));
    if (ret == kEplSuccessful)
    {   // SDO transfer started
        ret = kEplReject;
    }
    else
    {
        EPL_DBGLVL_CFM_TRACE("CN%x Writing 0x1006 returns 0x%X\n", pNodeInfo_p->eventCnProgress.nodeId, ret);
    }
#endif

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Download object

The function downloads the next object from the ConciseDCF to the specified
node.

\param  pNodeInfo_p     Node info of the node for which to download the next
                        object.

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
static tEplKernel downloadObject(tCfmNodeInfo* pNodeInfo_p)
{
    tEplKernel          ret = kEplSuccessful;
    static UINT32       leSignature;

    // forward data pointer for last transfer
    pNodeInfo_p->pDataConciseDcf += pNodeInfo_p->curDataSize;
    pNodeInfo_p->bytesRemaining -= pNodeInfo_p->curDataSize;

    if (pNodeInfo_p->entriesRemaining > 0)
    {
        if (pNodeInfo_p->bytesRemaining < EPL_CDC_OFFSET_DATA)
        {
            // not enough bytes left in ConciseDCF
            pNodeInfo_p->eventCnProgress.error = kEplCfmInvalidDcf;
            if ((ret = callCbProgress(pNodeInfo_p)) != kEplSuccessful)
                return ret;
            return finishConfig(pNodeInfo_p, kNmtNodeCommandConfErr);
        }

        // fetch next item from ConciseDCF
        pNodeInfo_p->eventCnProgress.objectIndex = AmiGetWordFromLe(&pNodeInfo_p->pDataConciseDcf[EPL_CDC_OFFSET_INDEX]);
        pNodeInfo_p->eventCnProgress.objectSubIndex = AmiGetByteFromLe(&pNodeInfo_p->pDataConciseDcf[EPL_CDC_OFFSET_SUBINDEX]);
        pNodeInfo_p->curDataSize = (UINT) AmiGetDwordFromLe(&pNodeInfo_p->pDataConciseDcf[EPL_CDC_OFFSET_SIZE]);
        pNodeInfo_p->pDataConciseDcf += EPL_CDC_OFFSET_DATA;
        pNodeInfo_p->bytesRemaining -= EPL_CDC_OFFSET_DATA;
        pNodeInfo_p->eventCnProgress.bytesDownloaded += EPL_CDC_OFFSET_DATA;

        if ((pNodeInfo_p->bytesRemaining < pNodeInfo_p->curDataSize) ||
            (pNodeInfo_p->curDataSize == 0))
        {
            // not enough bytes left in ConciseDCF
            pNodeInfo_p->eventCnProgress.error = kEplCfmInvalidDcf;
            if ((ret = callCbProgress(pNodeInfo_p)) != kEplSuccessful)
                return ret;
            return finishConfig(pNodeInfo_p, kNmtNodeCommandConfErr);
        }

        pNodeInfo_p->entriesRemaining--;
        ret = sdoWriteObject(pNodeInfo_p, pNodeInfo_p->pDataConciseDcf, pNodeInfo_p->curDataSize);
        if (ret != kEplSuccessful)
            return ret;
    }
    else
    {   // download finished
        if (pNodeInfo_p->fDoStore != FALSE)
        {
            // store configuration into non-volatile memory
            pNodeInfo_p->cfmState = kCfmStateWaitStore;
            AmiSetDwordToLe(&leSignature, 0x65766173);
            pNodeInfo_p->eventCnProgress.objectIndex = 0x1010;
            pNodeInfo_p->eventCnProgress.objectSubIndex = 0x01;
            ret = sdoWriteObject(pNodeInfo_p, &leSignature, sizeof (leSignature));
            if (ret != kEplSuccessful)
                return ret;
        }
        else
        {
            ret = downloadCycleLength(pNodeInfo_p);
            if (ret == kEplReject)
            {
                pNodeInfo_p->cfmState = kCfmStateUpToDate;
                return kEplSuccessful;
            }
            else
            {
                return finishConfig(pNodeInfo_p, kNmtNodeCommandConfReset);
            }
        }
    }

    return ret;
}


//------------------------------------------------------------------------------
/**
\brief  Write object by SDO transfer

The function writes the specified entry to the OD of the specified node.

\param  pNodeInfo_p     Node info of the node to write.
\param  pLeSrcData_p    Pointer to data in little endian byte order.
\param  size_p          Size of data.

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
static tEplKernel sdoWriteObject(tCfmNodeInfo* pNodeInfo_p, void* pLeSrcData_p, UINT size_p)
{
    tEplKernel                  ret = kEplSuccessful;
    tEplSdoComTransParamByIndex transParamByIndex;

    if ((pLeSrcData_p == NULL) || (size_p == 0))
        return kEplApiInvalidParam;

    if (pNodeInfo_p->sdoComConHdl == UINT_MAX)
    {
        // init command layer connection
        ret = EplSdoComDefineCon(&pNodeInfo_p->sdoComConHdl,
                                 pNodeInfo_p->eventCnProgress.nodeId,
                                 kEplSdoTypeAsnd);
        if ((ret != kEplSuccessful) && (ret != kEplSdoComHandleExists))
            return ret;
    }

    transParamByIndex.m_pData = pLeSrcData_p;
    transParamByIndex.m_SdoAccessType = kEplSdoAccessTypeWrite;
    transParamByIndex.m_SdoComConHdl = pNodeInfo_p->sdoComConHdl;
    transParamByIndex.m_uiDataSize = size_p;
    transParamByIndex.m_uiIndex = pNodeInfo_p->eventCnProgress.objectIndex;
    transParamByIndex.m_uiSubindex = pNodeInfo_p->eventCnProgress.objectSubIndex;
    transParamByIndex.m_pfnSdoFinishedCb = cbSdoCon;
    transParamByIndex.m_pUserArg = pNodeInfo_p;

    ret = EplSdoComInitTransferByIndex(&transParamByIndex);
    if (ret == kEplSdoComHandleBusy)
    {
        ret = EplSdoComSdoAbort(pNodeInfo_p->sdoComConHdl, EPL_SDOAC_DATA_NOT_TRANSF_DUE_LOCAL_CONTROL);
        if (ret == kEplSuccessful)
        {
            ret = EplSdoComInitTransferByIndex(&transParamByIndex);
        }
    }
    else if (ret == kEplSdoSeqConnectionBusy)
    {
        // close connection
        ret = EplSdoComUndefineCon(pNodeInfo_p->sdoComConHdl);
        pNodeInfo_p->sdoComConHdl = UINT_MAX;
        if (ret != kEplSuccessful)
        {
            EPL_DBGLVL_CFM_TRACE("SDO Free Error!\n");
            return ret;
        }

        // reinit command layer connection
        ret = EplSdoComDefineCon(&pNodeInfo_p->sdoComConHdl,
                                 pNodeInfo_p->eventCnProgress.nodeId,
                                 kEplSdoTypeAsnd);
        if ((ret != kEplSuccessful) && (ret != kEplSdoComHandleExists))
            return ret;

        // retry transfer
        transParamByIndex.m_SdoComConHdl = pNodeInfo_p->sdoComConHdl;
        ret = EplSdoComInitTransferByIndex(&transParamByIndex);
    }

    return ret;
}

///\}

