/****************************************************************************

  (c) Kalycito Infotech Private Limited
  (c) SYSTEC electronic GmbH, D-07973 Greiz, August-Bebel-Str. 29
      www.systec-electronic.com

  Project:      openPOWERLINK

  Description:  source file for Configuration Manager (CFM) module

  License:

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    1. Redistributions of source code must retain the above copyright
       notice, this list of conditions and the following disclaimer.

    2. Redistributions in binary form must reproduce the above copyright
       notice, this list of conditions and the following disclaimer in the
       documentation and/or other materials provided with the distribution.

    3. Neither the name of Kalycito Infotech Private Limited nor the names of
       its contributors may be used to endorse or promote products derived
       from this software without prior written permission. For written
       permission, please contact info@kalycito.com.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    POSSIBILITY OF SUCH DAMAGE.

    Severability Clause:

        If a provision of this License is or becomes illegal, invalid or
        unenforceable in any jurisdiction, that shall not affect:
        1. the validity or enforceability in that jurisdiction of any other
           provision of this License; or
        2. the validity or enforceability in other jurisdictions of that or
           any other provision of this License.

****************************************************************************/

#include "EplInc.h"

#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_CFM)) != 0)

#include "user/EplCfmu.h"
#include "EplSdoAc.h"
#include "user/EplIdentu.h"
#include "user/EplObdu.h"
#include "user/EplSdoComu.h"
#include "user/EplNmtu.h"

#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_OBDU)) == 0) && (EPL_OBD_USE_KERNEL == FALSE)
#error "EPL CFM module needs EPL module OBDU or OBDK!"
#endif

#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_SDOC)) == 0)
#error "EPL CFMu module needs EPL module SDO client!"
#endif

#if (EPL_OBD_USE_STRING_DOMAIN_IN_RAM == FALSE)
#error "EPL CFMu module needs define EPL_OBD_USE_STRING_DOMAIN_IN_RAM == TRUE"
#endif

/***************************************************************************/
/*                                                                         */
/*                                                                         */
/*          G L O B A L   D E F I N I T I O N S                            */
/*                                                                         */
/*                                                                         */
/***************************************************************************/

//---------------------------------------------------------------------------
// const defines
//---------------------------------------------------------------------------

#ifndef EPL_CFM_CONFIGURE_CYCLE_LENGTH
#define EPL_CFM_CONFIGURE_CYCLE_LENGTH  FALSE
#endif

// return pointer to node info structure for specified node ID
// d.k. may be replaced by special (hash) function if node ID array is smaller than 254
#define EPL_CFMU_GET_NODEINFO(uiNodeId_p) (EplCfmuInstance_g.m_apNodeInfo[uiNodeId_p - 1])


//---------------------------------------------------------------------------
// local types
//---------------------------------------------------------------------------

//States in Configuration Manager (CFM)
typedef enum
{
    kEplCfmuStateIdle           = 0x00,
    kEplCfmuStateWaitRestore,
    kEplCfmuStateDownload,
    kEplCfmuStateWaitStore,
    kEplCfmuStateUpToDate,
    kEplCfmuStateInternalAbort,

} tEplCfmuState;

typedef struct
{
    tEplCfmEventCnProgress  m_EventCnProgress;
    BYTE*               m_pbObdBufferConciseDcf;
    BYTE*               m_pbDataConciseDcf;
    DWORD               m_dwBytesRemaining;
    DWORD               m_dwEntriesRemaining;
    tEplSdoComConHdl    m_SdoComConHdl;
    tEplCfmuState       m_CfmState;
    unsigned int        m_uiCurDataSize;
    BOOL                m_fDoStore;

} tEplCfmuNodeInfo;


typedef struct
{
    tEplCfmuNodeInfo*   m_apNodeInfo[EPL_NMT_MAX_NODE_ID];
    DWORD               m_le_dwDomainSizeNull;
#if (EPL_CFM_CONFIGURE_CYCLE_LENGTH != FALSE)
    DWORD               m_le_dwCycleLength;
#endif

    tEplCfmCbEventCnProgress    m_pfnCbEventCnProgress;
    tEplCfmCbEventCnResult      m_pfnCbEventCnResult;

} tEplCfmuInstance;


//---------------------------------------------------------------------------
// local vars
//---------------------------------------------------------------------------

static tEplCfmuInstance  EplCfmuInstance_g;


//---------------------------------------------------------------------------
// local function prototypes
//---------------------------------------------------------------------------

static tEplCfmuNodeInfo* EplCfmuAllocNodeInfo(unsigned int uiNodeId_p);

static tEplKernel EplCfmuCallCbProgress(tEplCfmuNodeInfo* pNodeInfo_p);

static tEplKernel EplCfmuDownloadCycleLength(
            tEplCfmuNodeInfo* pNodeInfo_p);

static tEplKernel EplCfmuDownloadObject(
            tEplCfmuNodeInfo* pNodeInfo_p);

static tEplKernel EplCfmuSdoWriteObject(
            tEplCfmuNodeInfo* pNodeInfo_p,
            void*             pSrcData_le_p,
            unsigned int      uiSize_p);

static tEplKernel PUBLIC  EplCfmuCbSdoCon(tEplSdoComFinished* pSdoComFinished_p);




//=========================================================================//
//                                                                         //
//          P U B L I C   F U N C T I O N S                                //
//                                                                         //
//=========================================================================//

//---------------------------------------------------------------------------
//
// Function:    EplCfmuAddInstance()
//
// Description: add and initialize new instance of the CFM module
//
// Parameters:  none
//
// Returns:     tEplKernel              = error code
//
//
// State:
//
//---------------------------------------------------------------------------

tEplKernel EplCfmuAddInstance(tEplCfmCbEventCnProgress pfnCbEventCnProgress_p, tEplCfmCbEventCnResult pfnCbEventCnResult_p)
{
tEplKernel      Ret = kEplSuccessful;
unsigned int    uiSubindex;
tEplVarParam    VarParam;

    EPL_MEMSET(&EplCfmuInstance_g, 0, sizeof(EplCfmuInstance_g));

    EplCfmuInstance_g.m_pfnCbEventCnProgress = pfnCbEventCnProgress_p;
    EplCfmuInstance_g.m_pfnCbEventCnResult = pfnCbEventCnResult_p;

    // link domain with 4 zero-bytes to object 0x1F22 CFM_ConciseDcfList_ADOM
    VarParam.m_pData = &EplCfmuInstance_g.m_le_dwDomainSizeNull;
    VarParam.m_Size = sizeof (EplCfmuInstance_g.m_le_dwDomainSizeNull);
    VarParam.m_uiIndex = 0x1F22;
    for (uiSubindex = 1; uiSubindex <= EPL_NMT_MAX_NODE_ID; uiSubindex++)
    {
        VarParam.m_uiSubindex = uiSubindex;
        VarParam.m_ValidFlag = kVarValidAll;
        Ret = EplObdDefineVar(&VarParam);
        if ((Ret != kEplSuccessful)
            && (Ret != kEplObdIndexNotExist)
            && (Ret != kEplObdSubindexNotExist))
        {
            goto Exit;
        }
    }
    Ret = kEplSuccessful;

Exit:
    return Ret;
}


//---------------------------------------------------------------------------
//
// Function:    EplCfmuDelInstance()
//
// Description: deletes an instance of the CFM module
//
// Parameters:  none
//
// Returns:     tEplKernel              = error code
//
//
// State:
//
//---------------------------------------------------------------------------

tEplKernel EplCfmuDelInstance(void)
{
tEplKernel          Ret = kEplSuccessful;
unsigned int        uiNodeId;
tEplVarParam        VarParam;
BYTE*               pbBuffer;
tEplCfmuNodeInfo*   pNodeInfo;

    // free domain for object 0x1F22 CFM_ConciseDcfList_ADOM
    VarParam.m_pData = NULL;
    VarParam.m_Size = 0;
    VarParam.m_uiIndex = 0x1F22;
    for (uiNodeId = 1; uiNodeId <= EPL_NMT_MAX_NODE_ID; uiNodeId++)
    {
        pNodeInfo = EPL_CFMU_GET_NODEINFO(uiNodeId);
        if (pNodeInfo != NULL)
        {
            if (pNodeInfo->m_SdoComConHdl != ~0)
            {
                Ret = EplSdoComSdoAbort(pNodeInfo->m_SdoComConHdl, EPL_SDOAC_DATA_NOT_TRANSF_DUE_DEVICE_STATE);
            }

            pbBuffer = pNodeInfo->m_pbObdBufferConciseDcf;
            if (pbBuffer != NULL)
            {
                VarParam.m_uiSubindex = uiNodeId;
                VarParam.m_ValidFlag = kVarValidAll;
                Ret = EplObdDefineVar(&VarParam);
                // ignore return code, because buffer has to be freed anyway

                EPL_FREE(pbBuffer);
                pNodeInfo->m_pbObdBufferConciseDcf = NULL;
            }
            EPL_FREE(pNodeInfo);
            EPL_CFMU_GET_NODEINFO(uiNodeId) = NULL;
        }
    }
    Ret = kEplSuccessful;

//Exit:
    return Ret;
}


//-------------------------------------------------------------------------------------
//
// Function:    EplCfmuProcessNodeEvent
//
// Description: starts the configuration of the specified CN if data/time differs with local values.
//
// Parameters:  uiNodeId_p      = node-ID of CN
//              NodeEvent_p     = node event of CN
//
// Returns:     kEplSuccessful  = configuration is OK -> continue boot process for this CN
//              kEplReject      = defer further processing until configuration process has finished
//              other error     = major error occurred
//
// State:
//
//-------------------------------------------------------------------------------------

tEplKernel EplCfmuProcessNodeEvent(unsigned int uiNodeId_p, tEplNmtNodeEvent NodeEvent_p)
{
tEplKernel          Ret = kEplSuccessful;
static DWORD        dw_le_Signature;
tEplCfmuNodeInfo*   pNodeInfo = NULL;
tEplObdSize         ObdSize;
DWORD               dwExpConfTime = 0;
DWORD               dwExpConfDate = 0;
tEplIdentResponse*  pIdentResponse = NULL;
BOOL                fDoUpdate = FALSE;

    if ((NodeEvent_p != kEplNmtNodeEventCheckConf)
        && (NodeEvent_p != kEplNmtNodeEventUpdateConf))
    {
        goto Exit;
    }

    pNodeInfo = EplCfmuAllocNodeInfo(uiNodeId_p);
    if (pNodeInfo == NULL)
    {
        Ret = kEplInvalidNodeId;
        goto Exit;
    }

    if (pNodeInfo->m_CfmState != kEplCfmuStateIdle)
    {
        // send abort
        pNodeInfo->m_CfmState = kEplCfmuStateInternalAbort;
        Ret = EplSdoComSdoAbort(pNodeInfo->m_SdoComConHdl, EPL_SDOAC_DATA_NOT_TRANSF_DUE_LOCAL_CONTROL);
        if (Ret != kEplSuccessful)
        {
            goto Exit;
        }

        // close connection
        Ret = EplSdoComUndefineCon(pNodeInfo->m_SdoComConHdl);
        pNodeInfo->m_SdoComConHdl = ~0U;
        if (Ret != kEplSuccessful)
        {
            EPL_DBGLVL_CFM_TRACE("SDO Free Error!\n");
            goto Exit;
        }

    }

    pNodeInfo->m_uiCurDataSize = 0;

    // fetch pointer to ConciseDCF from object 0x1F22
    // (this allows the application to link its own memory to this object)
    pNodeInfo->m_pbDataConciseDcf = EplObduGetObjectDataPtr(0x1F22, uiNodeId_p);
    if (pNodeInfo->m_pbDataConciseDcf == NULL)
    {
        Ret = kEplCfmNoConfigData;
        goto Exit;
    }

    ObdSize = EplObduGetDataSize(0x1F22, uiNodeId_p);
    pNodeInfo->m_dwBytesRemaining = (DWORD) ObdSize;
    pNodeInfo->m_EventCnProgress.m_dwTotalNumberOfBytes = pNodeInfo->m_dwBytesRemaining;
#if (EPL_CFM_CONFIGURE_CYCLE_LENGTH != FALSE)
    pNodeInfo->m_EventCnProgress.m_dwTotalNumberOfBytes += sizeof (DWORD);
#endif
    pNodeInfo->m_EventCnProgress.m_dwBytesDownloaded = 0;
    if (ObdSize < sizeof (DWORD))
    {
        pNodeInfo->m_EventCnProgress.m_EplError = kEplCfmInvalidDcf;
        Ret = EplCfmuCallCbProgress(pNodeInfo);
        if (Ret != kEplSuccessful)
        {
            goto Exit;
        }
        Ret = pNodeInfo->m_EventCnProgress.m_EplError;
        goto Exit;
    }

    pNodeInfo->m_dwEntriesRemaining = AmiGetDwordFromLe(pNodeInfo->m_pbDataConciseDcf);

    pNodeInfo->m_pbDataConciseDcf += sizeof (DWORD);
    pNodeInfo->m_dwBytesRemaining -= sizeof (DWORD);
    pNodeInfo->m_EventCnProgress.m_dwBytesDownloaded += sizeof (DWORD);

    if (pNodeInfo->m_dwEntriesRemaining == 0)
    {
        pNodeInfo->m_EventCnProgress.m_EplError = kEplCfmNoConfigData;
        Ret = EplCfmuCallCbProgress(pNodeInfo);
        if (Ret != kEplSuccessful)
        {
            goto Exit;
        }
    }
    else
    {
        ObdSize = sizeof (dwExpConfDate);
        Ret = EplObduReadEntry(0x1F26, uiNodeId_p, &dwExpConfDate, &ObdSize);
        if (Ret != kEplSuccessful)
        {
            EPL_DBGLVL_CFM_TRACE("CN%x Error Reading 0x1F26 returns 0x%X\n", uiNodeId_p, Ret);
        }
        ObdSize = sizeof (dwExpConfTime);
        Ret = EplObduReadEntry(0x1F27, uiNodeId_p, &dwExpConfTime, &ObdSize);
        if (Ret != kEplSuccessful)
        {
            EPL_DBGLVL_CFM_TRACE("CN%x Error Reading 0x1F27 returns 0x%X\n", uiNodeId_p, Ret);
        }
        if ((dwExpConfDate != 0)
            || (dwExpConfTime != 0))
        {   // store configuration in CN at the end of the download,
            // because expected configuration date or time is set
            pNodeInfo->m_fDoStore = TRUE;
            pNodeInfo->m_EventCnProgress.m_dwTotalNumberOfBytes += sizeof (DWORD);
        }
        else
        {   // expected configuration date and time is not set
            fDoUpdate = TRUE;
        }
        EplIdentuGetIdentResponse(uiNodeId_p, &pIdentResponse);
        if (pIdentResponse == NULL)
        {
            EPL_DBGLVL_CFM_TRACE("CN%x Ident Response is NULL\n", uiNodeId_p);
            Ret = kEplInvalidNodeId;
            goto Exit;
        }
    }

#if (EPL_CFM_CONFIGURE_CYCLE_LENGTH != FALSE)
    ObdSize = sizeof (EplCfmuInstance_g.m_le_dwCycleLength);
    Ret = EplObduReadEntryToLe(0x1006, 0x00, &EplCfmuInstance_g.m_le_dwCycleLength, &ObdSize);
    if (Ret != kEplSuccessful)
    {   // local OD access failed
        EPL_DBGLVL_CFM_TRACE("Local OBD read failed %d\n", Ret);
        goto Exit;
    }
#endif

    if ((pNodeInfo->m_dwEntriesRemaining == 0)
        || ((NodeEvent_p != kEplNmtNodeEventUpdateConf)
            && (fDoUpdate == FALSE)
            && ((AmiGetDwordFromLe(&pIdentResponse->m_le_dwVerifyConfigurationDate) == dwExpConfDate)
                && (AmiGetDwordFromLe(&pIdentResponse->m_le_dwVerifyConfigurationTime) == dwExpConfTime))))
    {
        pNodeInfo->m_CfmState = kEplCfmuStateIdle;

        // current version is already available on the CN, no need to write new values, we can continue
        EPL_DBGLVL_CFM_TRACE("CN%x - Cfg Upto Date\n", uiNodeId_p);

        Ret = EplCfmuDownloadCycleLength(pNodeInfo);
        if (Ret == kEplReject)
        {
            pNodeInfo->m_CfmState = kEplCfmuStateUpToDate;
        }
    }
    else if (NodeEvent_p == kEplNmtNodeEventUpdateConf)
    {
        pNodeInfo->m_CfmState = kEplCfmuStateDownload;

        Ret = EplCfmuDownloadObject(pNodeInfo);
        if (Ret == kEplSuccessful)
        {   // SDO transfer started
            Ret = kEplReject;
        }
    }
    else
    {
        pNodeInfo->m_CfmState = kEplCfmuStateWaitRestore;

        pNodeInfo->m_EventCnProgress.m_dwTotalNumberOfBytes += sizeof (dw_le_Signature);
        AmiSetDwordToLe(&dw_le_Signature, 0x64616F6C);
        //Restore Default Parameters
        EPL_DBGLVL_CFM_TRACE("CN%x - Cfg Mismatch | MN Expects: %lx-%lx ", uiNodeId_p, dwExpConfDate, dwExpConfTime);
        EPL_DBGLVL_CFM_TRACE("CN Has: %lx-%lx. Restoring Default...\n", AmiGetDwordFromLe(&pIdentResponse->m_le_dwVerifyConfigurationDate), AmiGetDwordFromLe(&pIdentResponse->m_le_dwVerifyConfigurationTime));

        pNodeInfo->m_EventCnProgress.m_uiObjectIndex = 0x1011;
        pNodeInfo->m_EventCnProgress.m_uiObjectSubIndex = 0x01;
        Ret = EplCfmuSdoWriteObject(pNodeInfo, &dw_le_Signature, sizeof (dw_le_Signature));
        if (Ret == kEplSuccessful)
        {   // SDO transfer started
            Ret = kEplReject;
        }
        else
        {
            // error occurred
            EPL_DBGLVL_CFM_TRACE("CfmCbEvent(Node): EplCfmuSdoWriteObject() returned 0x%02X\n", Ret);
        }
    }

Exit:
    return Ret;
}


//---------------------------------------------------------------------------
//
// Function:    EplCfmuIsSdoRunning
//
// Description: returns TRUE if a SDO is running for the specified node.
//
// Parameters:  uiNodeId_p              = node-ID
//
// Returns:     BOOL                    = TRUE, if SDO is running
//                                        FALSE, otherwise
//
// State:
//
//---------------------------------------------------------------------------

BOOL EplCfmuIsSdoRunning(unsigned int uiNodeId_p)
{
tEplCfmuNodeInfo*   pNodeInfo = NULL;
BOOL                fSdoRunning = FALSE;

    if ((uiNodeId_p == 0)
        || (uiNodeId_p > EPL_NMT_MAX_NODE_ID))
    {
        goto Exit;
    }
    pNodeInfo = EPL_CFMU_GET_NODEINFO(uiNodeId_p);
    if (pNodeInfo == NULL)
    {
        goto Exit;
    }
    if (pNodeInfo->m_CfmState != kEplCfmuStateIdle)
    {
        fSdoRunning = TRUE;
    }

Exit:
    return fSdoRunning;
}


//---------------------------------------------------------------------------
//
// Function:    EplCfmuCbObdAccess
//
// Description: callback function for OD accesses
//
// Parameters:  pParam_p                = OBD parameter
//
// Returns:     tEplKernel              = error code
//
//
// State:
//
//---------------------------------------------------------------------------

tEplKernel PUBLIC EplCfmuCbObdAccess(tEplObdCbParam MEM* pParam_p)
{
tEplKernel          Ret = kEplSuccessful;
tEplObdVStringDomain*   pMemVStringDomain;
tEplCfmuNodeInfo*       pNodeInfo = NULL;
BYTE*                   pbBuffer;

    pParam_p->m_dwAbortCode = 0;

    if ((pParam_p->m_uiIndex != 0x1F22)
        || (pParam_p->m_ObdEvent != kEplObdEvWrStringDomain))
    {
        goto Exit;
    }

    // abort any running SDO transfer
    pNodeInfo = EPL_CFMU_GET_NODEINFO(pParam_p->m_uiSubIndex);
    if ((pNodeInfo != NULL)
        && (pNodeInfo->m_SdoComConHdl != ~0))
    {
        Ret = EplSdoComSdoAbort(pNodeInfo->m_SdoComConHdl, EPL_SDOAC_DATA_NOT_TRANSF_DUE_DEVICE_STATE);
    }

    pMemVStringDomain = pParam_p->m_pArg;
    if ((pMemVStringDomain->m_ObjSize != pMemVStringDomain->m_DownloadSize)
        || (pMemVStringDomain->m_pData == NULL))
    {
        pNodeInfo = EplCfmuAllocNodeInfo(pParam_p->m_uiSubIndex);
        if (pNodeInfo == NULL)
        {
            pParam_p->m_dwAbortCode = EPL_SDOAC_OUT_OF_MEMORY;
            Ret = kEplNoResource;
            goto Exit;
        }

        pbBuffer = pNodeInfo->m_pbObdBufferConciseDcf;
        if (pbBuffer != NULL)
        {
            EPL_FREE(pbBuffer);
            pNodeInfo->m_pbObdBufferConciseDcf = NULL;
        }
        pbBuffer = EPL_MALLOC(pMemVStringDomain->m_DownloadSize);
        if (pbBuffer == NULL)
        {
            pParam_p->m_dwAbortCode = EPL_SDOAC_OUT_OF_MEMORY;
            Ret = kEplNoResource;
            goto Exit;
        }
        pNodeInfo->m_pbObdBufferConciseDcf = pbBuffer;
        pMemVStringDomain->m_pData = pbBuffer;
        pMemVStringDomain->m_ObjSize = pMemVStringDomain->m_DownloadSize;
    }

Exit:
    return Ret;
}



//=========================================================================//
//                                                                         //
//          P R I V A T E   F U N C T I O N S                              //
//                                                                         //
//=========================================================================//

//-------------------------------------------------------------------------------------
//
// Function:    EplCfmuAllocNodeInfo
//
// Description: allocates a node-info structure for the specified node.
//
// Parameters:  uiNodeId_p          = node-ID for which the structure shall be allocated
//
// Returns:     tEplCfmuNodeInfo*   = pointer to node-info structure
//
// State:
//
//-------------------------------------------------------------------------------------

static tEplCfmuNodeInfo* EplCfmuAllocNodeInfo(unsigned int uiNodeId_p)
{
tEplCfmuNodeInfo*   pNodeInfo = NULL;

    if ((uiNodeId_p == 0)
        || (uiNodeId_p > EPL_NMT_MAX_NODE_ID))
    {
        goto Exit;
    }
    pNodeInfo = EPL_CFMU_GET_NODEINFO(uiNodeId_p);
    if (pNodeInfo != NULL)
    {
        goto Exit;
    }
    pNodeInfo = EPL_MALLOC(sizeof (*pNodeInfo));
    EPL_MEMSET(pNodeInfo, 0, sizeof (*pNodeInfo));
    pNodeInfo->m_EventCnProgress.m_uiNodeId = uiNodeId_p;
    pNodeInfo->m_SdoComConHdl = ~0U;

    EPL_CFMU_GET_NODEINFO(uiNodeId_p) = pNodeInfo;

Exit:
    return pNodeInfo;
}


//-------------------------------------------------------------------------------------
//
// Function:    EplCfmuCallCbProgress
//
// Description: calls the event progress callback for the specified node.
//
// Parameters:  pNodeInfo_p         = pointer to internal node-info structure
//
// Returns:     void
//
// State:
//
//-------------------------------------------------------------------------------------

static tEplKernel EplCfmuCallCbProgress(tEplCfmuNodeInfo* pNodeInfo_p)
{
tEplKernel      Ret = kEplSuccessful;

    if (EplCfmuInstance_g.m_pfnCbEventCnProgress != NULL)
    {
        Ret = EplCfmuInstance_g.m_pfnCbEventCnProgress(&pNodeInfo_p->m_EventCnProgress);
    }

    return Ret;
}


//-------------------------------------------------------------------------------------
//
// Function:    EplCfmuFinishConfig
//
// Description: calls the event progress callback for the specified node.
//
// Parameters:  pNodeInfo_p         = pointer to internal node-info structure
//
// Returns:     tEplKernel
//
// State:
//
//-------------------------------------------------------------------------------------

static tEplKernel EplCfmuFinishConfig(tEplCfmuNodeInfo* pNodeInfo_p, tEplNmtCommand NmtCommand_p)
{
tEplKernel      Ret = kEplSuccessful;

    if (pNodeInfo_p->m_SdoComConHdl != ~0U)
    {
        Ret = EplSdoComUndefineCon(pNodeInfo_p->m_SdoComConHdl);
        pNodeInfo_p->m_SdoComConHdl = ~0U;
        if (Ret != kEplSuccessful)
        {
            EPL_DBGLVL_CFM_TRACE("SDO Free Error!\n");
            goto Exit;
        }
    }

    pNodeInfo_p->m_CfmState = kEplCfmuStateIdle;

    if (EplCfmuInstance_g.m_pfnCbEventCnResult != NULL)
    {
        Ret = EplCfmuInstance_g.m_pfnCbEventCnResult(pNodeInfo_p->m_EventCnProgress.m_uiNodeId, NmtCommand_p);
    }

Exit:
    return Ret;
}


//-------------------------------------------------------------------------------------
//
// Function:    EplCfmuCbSdoCon
//
// Description: SDO finished callback.
//
// Parameters:  pSdoComFinished_p   = pointer to structure, which describes the
//                                    the SDO event in detail
//
// Returns:     None
//
// State:
//
//-------------------------------------------------------------------------------------

static tEplKernel PUBLIC  EplCfmuCbSdoCon(tEplSdoComFinished* pSdoComFinished_p)
{
tEplKernel          Ret = kEplSuccessful;
tEplCfmuNodeInfo*   pNodeInfo = pSdoComFinished_p->m_pUserArg;

    if (pNodeInfo == NULL)
    {
        return kEplInvalidNodeId;
    }

    pNodeInfo->m_EventCnProgress.m_dwSdoAbortCode = pSdoComFinished_p->m_dwAbortCode;
    pNodeInfo->m_EventCnProgress.m_dwBytesDownloaded += pSdoComFinished_p->m_uiTransferredByte;

    Ret = EplCfmuCallCbProgress(pNodeInfo);
    if (Ret != kEplSuccessful)
    {
        goto Exit;
    }

    switch (pNodeInfo->m_CfmState)
    {
        case kEplCfmuStateIdle:
        {
            Ret = EplCfmuFinishConfig(pNodeInfo, kEplNmtNodeCommandConfErr);
            break;
        }

        case kEplCfmuStateUpToDate:
        {
        tEplNmtCommand NmtCommand;

            if (pSdoComFinished_p->m_SdoComConState == kEplSdoComTransferFinished)
            {
                // continue boot-up of CN with NMT command Reset Configuration
                NmtCommand = kEplNmtNodeCommandConfReset;
            }
            else
            {
                // indicate configuration error CN
                NmtCommand = kEplNmtNodeCommandConfErr;
            }
            Ret = EplCfmuFinishConfig(pNodeInfo, NmtCommand);
            break;
        }

        case kEplCfmuStateDownload:
        {
            if (pSdoComFinished_p->m_SdoComConState == kEplSdoComTransferFinished)
            {
                Ret = EplCfmuDownloadObject(pNodeInfo);
            }
            else
            {   // configuration was not successful
                Ret = EplCfmuFinishConfig(pNodeInfo, kEplNmtNodeCommandConfErr);
            }

            break;
        }

        case kEplCfmuStateWaitRestore:
        {
            if (pSdoComFinished_p->m_SdoComConState == kEplSdoComTransferFinished)
            {   // configuration successfully restored
                EPL_DBGLVL_CFM_TRACE("\nCN%x - Restore Complete. Resetting Node...\n", pNodeInfo->m_EventCnProgress.m_uiNodeId);

                // send NMT command reset node to activate the original configuration
                Ret = EplCfmuFinishConfig(pNodeInfo, kEplNmtNodeCommandConfRestored);
            }
            else
            {   // restore configuration not available
                // start downloading the ConciseDCF
                pNodeInfo->m_CfmState = kEplCfmuStateDownload;
                Ret = EplCfmuDownloadObject(pNodeInfo);
            }

            break;
        }

        case kEplCfmuStateWaitStore:
        {
            Ret = EplCfmuDownloadCycleLength(pNodeInfo);
            if (Ret == kEplReject)
            {
                pNodeInfo->m_CfmState = kEplCfmuStateUpToDate;
                Ret = kEplSuccessful;
            }
            else
            {
                Ret = EplCfmuFinishConfig(pNodeInfo, kEplNmtNodeCommandConfReset);
            }
            break;
        }

        case kEplCfmuStateInternalAbort:
        {
            // configuration was aborted
            break;
        }
    }

Exit:
    return Ret;
}


// ----------------------------------------------------------------------------
//
// Function:    EplCfmuDownloadCycleLength()
//
// Description: reads the specified entry from the OD of the specified node.
//              If this node is a remote node, it performs a SDO transfer, which
//              means this function returns kEplApiTaskDeferred and the application
//              is informed via the event callback function when the task is completed.
//
// Parameters:  pNodeInfo_p         = pointer to internal node-info structure
//
// Return:      tEplKernel          = error code
//
// ----------------------------------------------------------------------------

static tEplKernel EplCfmuDownloadCycleLength(
            tEplCfmuNodeInfo* pNodeInfo_p)
{
tEplKernel      Ret = kEplSuccessful;

#if (EPL_CFM_CONFIGURE_CYCLE_LENGTH != FALSE)
    pNodeInfo_p->m_EventCnProgress.m_uiObjectIndex = 0x1006;
    pNodeInfo_p->m_EventCnProgress.m_uiObjectSubIndex = 0x00;
    Ret = EplCfmuSdoWriteObject(pNodeInfo_p, &EplCfmuInstance_g.m_le_dwCycleLength, sizeof (EplCfmuInstance_g.m_le_dwCycleLength));
    if (Ret == kEplSuccessful)
    {   // SDO transfer started
        Ret = kEplReject;
    }
    else
    {
        EPL_DBGLVL_CFM_TRACE("CN%x Writing 0x1006 returns 0x%X\n", pNodeInfo_p->m_EventCnProgress.m_uiNodeId, Ret);
    }
#endif

    return Ret;
}


// ----------------------------------------------------------------------------
//
// Function:    EplCfmuDownloadObject()
//
// Description: downloads the next object from the ConciseDCF to the node.
//
// Parameters:  pNodeInfo_p         = pointer to internal node-info structure
//
// Return:      tEplKernel          = error code
//
// ----------------------------------------------------------------------------

static tEplKernel EplCfmuDownloadObject(
            tEplCfmuNodeInfo* pNodeInfo_p)
{
tEplKernel      Ret = kEplSuccessful;
static DWORD    dw_le_Signature;

    // forward data pointer for last transfer
    pNodeInfo_p->m_pbDataConciseDcf += pNodeInfo_p->m_uiCurDataSize;
    pNodeInfo_p->m_dwBytesRemaining -= pNodeInfo_p->m_uiCurDataSize;

    if (pNodeInfo_p->m_dwEntriesRemaining > 0)
    {
        if (pNodeInfo_p->m_dwBytesRemaining < EPL_CDC_OFFSET_DATA)
        {
            // not enough bytes left in ConciseDCF
            pNodeInfo_p->m_EventCnProgress.m_EplError = kEplCfmInvalidDcf;
            Ret = EplCfmuCallCbProgress(pNodeInfo_p);
            if (Ret != kEplSuccessful)
            {
                goto Exit;
            }
            Ret = EplCfmuFinishConfig(pNodeInfo_p, kEplNmtNodeCommandConfErr);
            goto Exit;
        }

        // fetch next item from ConciseDCF
        pNodeInfo_p->m_EventCnProgress.m_uiObjectIndex = AmiGetWordFromLe(&pNodeInfo_p->m_pbDataConciseDcf[EPL_CDC_OFFSET_INDEX]);
        pNodeInfo_p->m_EventCnProgress.m_uiObjectSubIndex = AmiGetByteFromLe(&pNodeInfo_p->m_pbDataConciseDcf[EPL_CDC_OFFSET_SUBINDEX]);
        pNodeInfo_p->m_uiCurDataSize = (unsigned int) AmiGetDwordFromLe(&pNodeInfo_p->m_pbDataConciseDcf[EPL_CDC_OFFSET_SIZE]);
        pNodeInfo_p->m_pbDataConciseDcf += EPL_CDC_OFFSET_DATA;
        pNodeInfo_p->m_dwBytesRemaining -= EPL_CDC_OFFSET_DATA;
        pNodeInfo_p->m_EventCnProgress.m_dwBytesDownloaded += EPL_CDC_OFFSET_DATA;

        if ((pNodeInfo_p->m_dwBytesRemaining < pNodeInfo_p->m_uiCurDataSize)
            || (pNodeInfo_p->m_uiCurDataSize == 0))
        {
            // not enough bytes left in ConciseDCF
            pNodeInfo_p->m_EventCnProgress.m_EplError = kEplCfmInvalidDcf;
            Ret = EplCfmuCallCbProgress(pNodeInfo_p);
            if (Ret != kEplSuccessful)
            {
                goto Exit;
            }
            Ret = EplCfmuFinishConfig(pNodeInfo_p, kEplNmtNodeCommandConfErr);
            goto Exit;
        }

        pNodeInfo_p->m_dwEntriesRemaining--;

        Ret = EplCfmuSdoWriteObject(pNodeInfo_p, pNodeInfo_p->m_pbDataConciseDcf, pNodeInfo_p->m_uiCurDataSize);
        if (Ret != kEplSuccessful)
        {
            goto Exit;
        }
    }
    else
    {   // download finished

        if (pNodeInfo_p->m_fDoStore != FALSE)
        {
            // store configuration into non-volatile memory
            pNodeInfo_p->m_CfmState = kEplCfmuStateWaitStore;
            AmiSetDwordToLe(&dw_le_Signature, 0x65766173);
            pNodeInfo_p->m_EventCnProgress.m_uiObjectIndex = 0x1010;
            pNodeInfo_p->m_EventCnProgress.m_uiObjectSubIndex = 0x01;
            Ret = EplCfmuSdoWriteObject(pNodeInfo_p, &dw_le_Signature, sizeof (dw_le_Signature));
            if (Ret != kEplSuccessful)
            {
                goto Exit;
            }
        }
        else
        {
            Ret = EplCfmuDownloadCycleLength(pNodeInfo_p);
            if (Ret == kEplReject)
            {
                pNodeInfo_p->m_CfmState = kEplCfmuStateUpToDate;
                Ret = kEplSuccessful;
            }
            else
            {
                Ret = EplCfmuFinishConfig(pNodeInfo_p, kEplNmtNodeCommandConfReset);
            }
        }
    }

Exit:
    return Ret;
}


// ----------------------------------------------------------------------------
//
// Function:    EplCfmuSdoWriteObject()
//
// Description: writes the specified entry to the OD of the specified node.
//
// Parameters:  pNodeInfo_p         = pointer to internal node-info structure
//              pSrcData_le_p       = IN: pointer to data in little endian
//              uiSize_p            = IN: size of data in bytes
//
// Return:      tEplKernel          = error code
//
// ----------------------------------------------------------------------------

static tEplKernel EplCfmuSdoWriteObject(
            tEplCfmuNodeInfo* pNodeInfo_p,
            void*             pSrcData_le_p,
            unsigned int      uiSize_p)
{
tEplKernel      Ret = kEplSuccessful;
tEplSdoComTransParamByIndex TransParamByIndex;

    if ((pSrcData_le_p == NULL) || (uiSize_p == 0))
    {
        Ret = kEplApiInvalidParam;
        goto Exit;
    }

    if (pNodeInfo_p->m_SdoComConHdl == ~0)
    {
        // init command layer connection
        Ret = EplSdoComDefineCon(&pNodeInfo_p->m_SdoComConHdl,
                                 pNodeInfo_p->m_EventCnProgress.m_uiNodeId,
                                 kEplSdoTypeAsnd);
        if ((Ret != kEplSuccessful) && (Ret != kEplSdoComHandleExists))
        {
            goto Exit;
        }
    }

    TransParamByIndex.m_pData = pSrcData_le_p;
    TransParamByIndex.m_SdoAccessType = kEplSdoAccessTypeWrite;
    TransParamByIndex.m_SdoComConHdl = pNodeInfo_p->m_SdoComConHdl;
    TransParamByIndex.m_uiDataSize = uiSize_p;
    TransParamByIndex.m_uiIndex = pNodeInfo_p->m_EventCnProgress.m_uiObjectIndex;
    TransParamByIndex.m_uiSubindex = pNodeInfo_p->m_EventCnProgress.m_uiObjectSubIndex;
    TransParamByIndex.m_pfnSdoFinishedCb = EplCfmuCbSdoCon;
    TransParamByIndex.m_pUserArg = pNodeInfo_p;

    Ret = EplSdoComInitTransferByIndex(&TransParamByIndex);
    if (Ret == kEplSdoComHandleBusy)
    {
        Ret = EplSdoComSdoAbort(pNodeInfo_p->m_SdoComConHdl, EPL_SDOAC_DATA_NOT_TRANSF_DUE_LOCAL_CONTROL);
        if (Ret == kEplSuccessful)
        {
            Ret = EplSdoComInitTransferByIndex(&TransParamByIndex);
        }
    }
    else if (Ret == kEplSdoSeqConnectionBusy)
    {
        // close connection
        Ret = EplSdoComUndefineCon(pNodeInfo_p->m_SdoComConHdl);
        pNodeInfo_p->m_SdoComConHdl = ~0U;
        if (Ret != kEplSuccessful)
        {
            EPL_DBGLVL_CFM_TRACE("SDO Free Error!\n");
            goto Exit;
        }

        // reinit command layer connection
        Ret = EplSdoComDefineCon(&pNodeInfo_p->m_SdoComConHdl,
                                 pNodeInfo_p->m_EventCnProgress.m_uiNodeId,
                                 kEplSdoTypeAsnd);
        if ((Ret != kEplSuccessful) && (Ret != kEplSdoComHandleExists))
        {
            goto Exit;
        }

        // retry transfer
        TransParamByIndex.m_SdoComConHdl = pNodeInfo_p->m_SdoComConHdl;
        Ret = EplSdoComInitTransferByIndex(&TransParamByIndex);
    }

Exit:
    return Ret;
}


#endif // (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_CFM)) != 0)

// EOF
