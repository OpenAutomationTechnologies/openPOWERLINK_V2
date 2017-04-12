/**
********************************************************************************
 \file   firmwarecheck.c

\brief  Source file of the firmware check module

This module implements firmware version checks of a found node.

\ingroup module_app_firmwaremanager
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2017, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
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
#include <oplk/oplk.h>
#include <firmwaremanager/firmwaretrace.h>
#include <firmwaremanager/firmwarecheck.h>
#include <firmwaremanager/firmwareupdate.h>
#include <firmwaremanager/firmwareinfo.h>

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

#define FIRMWARE_CHECK_NODE_FIRWMARE_INDEX 0x1F50
#define FIMRWARE_CHECK_NODE_FIRMWARE_SUBINDEX 0x01

#define FIRMWARE_CHECK_INDEX_IDENT_INDICIES 0x1027
#define FIRMWARE_CHECK_INDEX_FW_DOWN_INDICES 0x1F55

#define FIMRWARE_CHECK_SUBINDEX_NUMBER_OF_ENTRIES 0x00

#define FIRMWARE_CHECK_READ_SDO_TYPE kSdoTypeAsnd

#define FIRMWARE_CHECK_START_MODULE_CHECK_RETRIES 5u

//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------

typedef enum
{
    kFwModuleUpdateStateInit,
    kFwModuleUpdateStateCheckFw,
    kFwModuleUpdateStateGetNumberOfFwDownIndices,
    kFwModuleUpdateStateGetIndexOfFwDown,
    kFwModuleUpdateStateGetNumberOfFwDownPerIndex,
    kFwModuleUpdateStateCalculateFwDownsForUpdate,
    kFwModuleUpdateStateUpdateFw,
    kFwModuleUpdateStateComplete,
} tFirmwareModuleUpdateState;

typedef enum
{
    kFwModuleCheckStateInit,
    kFwModuleCheckStateGetNumberOfIdentIndices,
    kFwModuleCheckStateGetIndexOfIdentArray,
    kFwModuleCheckStateGetNumberOfIdentsPerIndex,
    kFwModuleCheckStateGetIdent,
    kFwModuleCheckStateCheckFw,
    kFwModuleCheckStateComplete,
} tFirmwareModuleCheckState;

typedef struct
{
    UINT32 vendorId;        ///< Vendor ID
    UINT32 productId;       ///< Product ID
    UINT32 revision;        ///< Revision
    UINT32 serialNumber;    ///< Serial number
    UINT32 softwareDate;    ///< Software data
    UINT32 softwareTime;    ///< Software time
} tFirmwareCheckModuleIdent;

typedef struct
{
    tFirmwareCheckModuleIdent ident; ///< Ident information of a module/node
    UINT index;             ///< SDO transfer OD index of remote
    UINT subindex;          ///< SDO transfer OD subindex of remotes
    tFirmwareStoreHandle pFwStoreHandle;
} tFirmwareCheckFwInfo;

typedef struct tFirmwareCheckModuleEntry
{
    tFirmwareModuleUpdateState          state;  ///< Module firmware check state
    tFirmwareModuleUpdateState          nextCheckState;  ///< Module firmware check state
    tFirmwareCheckFwInfo                fwInfo; ///< Firmware info from the module
    struct tFirmwareCheckModuleEntry*   pNext;  ///< Pointer to next module entry
    size_t moduleIndex;
} tFirmwareCheckModuleEntry;

typedef tFirmwareCheckModuleEntry* tFirmwareCheckModuleList;

typedef struct
{
    tSdoComConHdl   handle;     ///< SDO command handle
    UINT            index;      ///< SDO transfer OD index of remote
    UINT            subindex;   ///< SDO transfer OD subindex of remotes
    UINT            size;       ///< Size of SDO transfer
    void*           pData;      ///< Pointer for result data storage
} tFirmwareCheckNodeSdo;

typedef struct
{
    UINT8 indexIdx;
    UINT8 numberOfIndices;
    UINT16* pIndices;
} tFirmwareCheckIndexArray;

typedef struct
{
    UINT8 identIdx;
    UINT8 numberOfIdents;
} tFirmwareCheckIdentInfo;

typedef struct
{
    UINT                                nodeId;                     ///< Node ID
    UINT32                              featureFlags;               ///< Node feature flags
    tFirmwareModuleCheckState           checkState;                 ///< Module firmware check state
    tFirmwareModuleCheckState           nextCheckState;             ///< Next module firmware check state
    tFirmwareModuleUpdateState          updateState;                ///< Module firmware update state
    tFirmwareModuleUpdateState          nextUpdateState;            ///< Next module firmware update state
    tFirmwareCheckFwInfo                nodeFwInfo;                 ///< Node firmware info
    UINT                                moduleIdx;                  ///< Index of module within modular node
    tFirmwareCheckModuleList            moduleList;                 ///< Module list
    tFirmwareCheckIndexArray            identIndices;               ///< Index array for modules idents
    tFirmwareCheckIdentInfo*            pIdentArrays;               ///< Field of ident infos for each ident object
    tFirmwareCheckIndexArray            fwDownloadIndices;          ///< Index array for fw download objects
    UINT16*                             pNumbersOfFwDownloads;      ///< Field of Number of entries for each fw download object
    tFirmwareCheckNodeSdo               sdo;                        ///< Information about the current sdo tranmission
    BOOL                                fModuleListContainsHead;    ///< Flag for indicating if the head station was added to the module list
    UINT                                startCounter;               ///< Counter for requested module check starts
} tFirmwareCheckNodeInfo;

typedef struct
{
    BOOL                    fInitialized;
    tFirmwareCheckNodeInfo  aNodeInfo[FIRMWARECHECK_MAX_NODEID]; // TODO: replace by list
    tFirmwareCheckNodeInfo* pNextNodeToCheck;
    tFirmwareInfoHandle     pFwInfo;
    tFirmwareCheckConfig    config;
} tFirmwareCheckInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------

static tFirmwareCheckInstance instance_l;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------

static tFirmwareRet checkPointer(const void* pCheck_p);
static tFirmwareRet checkNodeId(UINT nodeId_p);
static tFirmwareCheckNodeInfo* getNodeCheckInfo(UINT nodeId_p);
static tFirmwareRet checkNodeInfo(tFirmwareCheckNodeInfo* pNodeInfo_p);
static tFirmwareRet finishCheck(tFirmwareCheckNodeInfo* pNodeInfo_p);
static BOOL isModularSupported(tFirmwareCheckNodeInfo* pNodeInfo_p);
static tFirmwareRet startGatheringModuleInfo(tFirmwareCheckNodeInfo* pNodeInfo_p);
static BOOL isExpectedSdoCompleteEvent(tFirmwareCheckNodeInfo* pNodeInfo_p,
                                       const tSdoComFinished* pSdoComFinished_p);

static tFirmwareRet issueSdoRead(tFirmwareCheckNodeInfo* pNodeInfo_p);

static tFirmwareRet getNumberOfModuleIdentIndices(tFirmwareCheckNodeInfo* pNodeInfo_p);
static tFirmwareRet getIndexOfModuleIdent(tFirmwareCheckNodeInfo* pNodeInfo_p,
                                          size_t index_p);
static tFirmwareRet getNumberOfEntriesInIdentObject(tFirmwareCheckNodeInfo* pNodeInfo_p,
                                                    size_t identArray_p);
static tFirmwareRet getModuleIdent(tFirmwareCheckNodeInfo* pNodeInfo_p,
                                   size_t identArray_p, size_t ident_p);
static tFirmwareRet getNumberOfModuleFwDownloadIndices(tFirmwareCheckNodeInfo* pNodeInfo_p);
static tFirmwareRet getIndexOfModuleFwDown(tFirmwareCheckNodeInfo* pNodeInfo_p,
                                          size_t index_p);
static tFirmwareRet getNumberOfModuleFwDownload(tFirmwareCheckNodeInfo* pNodeInfo_p,
                                                size_t fwArray_p);

static tFirmwareRet processSdoEventForModule(tFirmwareCheckNodeInfo* pNodeInfo_p,
                                             const tSdoComFinished* pSdoComFinished_p);


static tFirmwareRet processCheckStateMachine(tFirmwareCheckNodeInfo* pNodeInfo_p);
static tFirmwareRet processUpdateStateMachine(tFirmwareCheckNodeInfo* pNodeInfo_p);

static BOOL isFirmwareUpdateRequired(tFirmwareCheckNodeInfo* pNodeInfo_p,
                                     tFirmwareCheckFwInfo* pFwInfo_p);

static void calcFwObjectForModule(tFirmwareCheckNodeInfo* pNodeInfo_p,
                                  size_t moduleIndex_p,
                                  UINT* pIndex_p,
                                  UINT* pSubIndex_p);

static void moduleCheckFailed(tFirmwareCheckNodeInfo* pNodeInfo_p);
static void cleanupNode(tFirmwareCheckNodeInfo* pNodeInfo_p);
static void finishCheckForNode(tFirmwareCheckNodeInfo* pNodeInfo_p);

static tFirmwareCheckNodeInfo* getNextNodeToCheck(void);

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Initialize the firmware check module

This function initializes the firmware check module.

\param pConfig_p [in]       Pointer to the configuration structure for the
                            firmware check module.

\return This functions returns a value of \ref tFirmwareRet.

\ingroup module_app_firmwaremanager
*/
//------------------------------------------------------------------------------
tFirmwareRet firmwarecheck_init(const tFirmwareCheckConfig* pConfig_p)
{
    tFirmwareRet ret = kFwReturnOk;

    if (instance_l.fInitialized)
    {
        ret = kFwReturnAlreadyInitialized;
        goto EXIT;
    }

    ret = checkPointer(pConfig_p);
    if (ret != kFwReturnOk)
    {
        goto EXIT;
    }

    memset(&instance_l, 0, sizeof(tFirmwareCheckInstance));

    memcpy(&instance_l.config, pConfig_p, sizeof(tFirmwareCheckConfig));

    instance_l.fInitialized = TRUE;

EXIT:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Deinitialize the firmware check module

This function deinitializes the firmware check module.

\ingroup module_app_firmwaremanager
*/
//------------------------------------------------------------------------------
void firmwarecheck_exit(void)
{
    instance_l.fInitialized = FALSE;
}

//------------------------------------------------------------------------------
/**
\brief  Process a Node event

This function processes a Node event for the given node ID. During this function
the firmware and the capabilities (support of modular) of the node are checked.
If a firmware update of the node is required the transmission will be initiated.

\param nodeId_p [in]    The ID of the node which caused the processed event

\return This functions returns a value of \ref tFirmwareRet.

\ingroup module_app_firmwaremanager
*/
//------------------------------------------------------------------------------
tFirmwareRet firmwarecheck_processNodeEvent(UINT nodeId_p)
{
    tFirmwareRet            ret = kFwReturnOk;
    tFirmwareCheckNodeInfo* pNodeInfo;

    ret = checkNodeId(nodeId_p);
    if (ret != kFwReturnOk)
    {
        goto EXIT;
    }

    pNodeInfo = getNodeCheckInfo(nodeId_p);
    if (pNodeInfo == NULL)
    {
        ret = kFwReturnInvalidInstance;
        goto EXIT;
    }

    ret = checkNodeInfo(pNodeInfo);
    if (ret != kFwReturnOk)
    {
        goto EXIT;
    }

EXIT:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Process a SDO event

This function processes a SOD event with the given resulting structure. This
functions manages the progress of gathering informations about the modules and
the transmission of required firmware updates.

\param pSdoComFinished_p [in]    Structure of the SDO event

\return This functions returns a value of \ref tFirmwareRet.

\ingroup module_app_firmwaremanager
*/
//------------------------------------------------------------------------------
tFirmwareRet firmwarecheck_processSdoEvent(const tSdoComFinished* pSdoComFinished_p)
{
    tFirmwareRet ret = kFwReturnOk;
    tFirmwareCheckNodeInfo* pNodeInfo;

    ret = checkPointer(pSdoComFinished_p);
    if (ret != kFwReturnOk)
    {
        goto EXIT;
    }

    pNodeInfo = getNodeCheckInfo(pSdoComFinished_p->nodeId);
    if (pNodeInfo == NULL)
    {
        ret = kFwReturnInvalidInstance;
        goto EXIT;
    }

    if (!isExpectedSdoCompleteEvent(pNodeInfo, pSdoComFinished_p))
    {
        ret = kFwReturnInvalidSdoEvent;
        goto EXIT;
    }

    ret = processSdoEventForModule(pNodeInfo, pSdoComFinished_p);

EXIT:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Check firmware information of modules behind the next node

This function gathers all required information about the available modules if
a module requires a firmware update this is initiated after the check was
completed. This function iterates over all modular nodes and proceeds with
each call.

\return This functions returns a value of \ref tFirmwareRet.

\ingroup module_app_firmwaremanager
*/
//------------------------------------------------------------------------------
tFirmwareRet firmwarecheck_checkModulesOfNextNode(void)
{
    tFirmwareRet ret = kFwReturnOk;
    tFirmwareCheckNodeInfo* pNodeInfo;

    pNodeInfo = getNextNodeToCheck();

    if (pNodeInfo == NULL)
    {
        // No modular node to check
        goto EXIT;
    }

    // check modular flag
    if (isModularSupported(pNodeInfo))
    {
        FWM_TRACE("Checking firmware information of modules of Node: %d\n",
                  pNodeInfo->nodeId);

        ret = startGatheringModuleInfo(pNodeInfo);
    }

EXIT:
    return ret;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

static tFirmwareRet checkPointer(const void* pCheck_p)
{
    tFirmwareRet ret = kFwReturnOk;

    if (pCheck_p == NULL)
    {
        ret = kFwReturnInvalidParameter;
    }

    return ret;
}

static tFirmwareRet checkNodeId(UINT nodeId_p)
{
    tFirmwareRet ret = kFwReturnOk;

    if ((nodeId_p == 0) || (nodeId_p >= FIRMWARECHECK_MAX_NODEID))
    {
        ret = kFwReturnInvalidNodeId;
    }

    return ret;
}

static tFirmwareCheckNodeInfo* getNodeCheckInfo(UINT nodeId_p)
{
    tFirmwareCheckNodeInfo* pNodeInfo;

    pNodeInfo = &instance_l.aNodeInfo[nodeId_p - 1];

    if (pNodeInfo->nodeId == FIRMWARECHECK_INVALID_NODEID)
    {
        memset(pNodeInfo, 0, sizeof(tFirmwareCheckNodeInfo));

        pNodeInfo->nodeId = nodeId_p;
        pNodeInfo->sdo.handle = FIRMWARECHECK_INVALID_SDO;
        pNodeInfo->checkState = kFwModuleCheckStateInit;
    }

    return pNodeInfo;
}

static tFirmwareRet checkNodeInfo(tFirmwareCheckNodeInfo* pNodeInfo_p)
{
    tFirmwareRet    ret = kFwReturnOk;
    tOplkError      oplkRet;
    const tIdentResponse* pIdent;
    tFirmwareCheckModuleEntry* pEntry;
    BOOL fHeadUpdated = FALSE;

    oplkRet = oplk_getIdentResponse(pNodeInfo_p->nodeId, &pIdent);
    if (oplkRet != kErrorOk)
    {
        FWM_TRACE("%s() Failed to get the ident for node %d with error 0x%X\n",
                  __func__,
                  pNodeInfo_p->nodeId,
                  oplkRet);
        ret = kFwReturnNoIdent;
        goto EXIT;
    }

    // TODO: Endian conversion for big endian machines
    pNodeInfo_p->nodeFwInfo.ident.vendorId = pIdent->vendorIdLe;
    pNodeInfo_p->nodeFwInfo.ident.productId = pIdent->productCodeLe;
    pNodeInfo_p->nodeFwInfo.ident.revision = pIdent->revisionNumberLe;
    pNodeInfo_p->nodeFwInfo.ident.softwareDate = pIdent->applicationSwDateLe;
    pNodeInfo_p->nodeFwInfo.ident.softwareTime = pIdent->applicationSwTimeLe;
    pNodeInfo_p->featureFlags = pIdent->featureFlagsLe;

    if (isFirmwareUpdateRequired(pNodeInfo_p, &pNodeInfo_p->nodeFwInfo))
    {
        pEntry = malloc(sizeof(tFirmwareCheckModuleEntry));
        memset(pEntry, 0, sizeof(tFirmwareCheckModuleEntry));
        memcpy(&pEntry->fwInfo, &pNodeInfo_p->nodeFwInfo, sizeof(tFirmwareCheckFwInfo));
        pEntry->fwInfo.index = FIRMWARE_CHECK_NODE_FIRWMARE_INDEX;
        pEntry->fwInfo.subindex = FIMRWARE_CHECK_NODE_FIRMWARE_SUBINDEX;
        pNodeInfo_p->moduleList = pEntry;
        pNodeInfo_p->fModuleListContainsHead = TRUE;
        fHeadUpdated = TRUE;

        FWM_TRACE("Firmware update of head station required for node: %d\n",
                  pNodeInfo_p->nodeId);
    }

    // check modular flag
    if (isModularSupported(pNodeInfo_p))
    {
        FWM_TRACE("Modular device detected, Node: %d\n", pNodeInfo_p->nodeId);
    }

    ret = finishCheck(pNodeInfo_p);
    if (ret != kFwReturnOk)
    {
        FWM_ERROR("Finishing check for node %u failed with return %d\n",
                  pNodeInfo_p->nodeId, ret);
    }

    if (fHeadUpdated)
    {
        ret = kFwReturnInterruptBoot;
    }

EXIT:
    return ret;
}

tFirmwareRet finishCheck(tFirmwareCheckNodeInfo* pNodeInfo_p)
{
    tFirmwareRet ret = kFwReturnOk;
    tFirmwareCheckModuleEntry* pIter;
    tFirmwareUpdateEntry* pList = NULL;
    tFirmwareUpdateEntry* pNew;
    tFirmwareUpdateEntry** ppIter = &pList;
    tFirmwareUpdateEntry* pUpdateRem;

    pIter = pNodeInfo_p->moduleList;

    while (pIter != NULL)
    {
        pNew = malloc(sizeof(tFirmwareUpdateEntry));
        if (pNew == NULL)
        {
            ret = kFwReturnNoRessource;
            goto EXIT;
        }

        memset(pNew, 0, sizeof(tFirmwareUpdateEntry));

        pNew->nodeId = pNodeInfo_p->nodeId;
        pNew->index = pIter->fwInfo.index;
        pNew->subindex = pIter->fwInfo.subindex;
        pNew->pStoreHandle = pIter->fwInfo.pFwStoreHandle;

        pNew->fIsNode = (pNew->index == FIRMWARE_CHECK_NODE_FIRWMARE_INDEX);

        *ppIter = pNew;
        ppIter = &pNew->pNext;

        pIter = pIter->pNext;
    }

    if (pList != NULL)
    {
        ret = firmwareupdate_processUpdateList(&pList);
        if (ret != kFwReturnOk)
        {
            goto EXIT;
        }
    }
    else
    {
        if (instance_l.config.pfnNoUpdateRequired != NULL)
        {
            ret = instance_l.config.pfnNoUpdateRequired(pNodeInfo_p->nodeId,
                                                        &pNodeInfo_p->sdo.handle);
        }
    }

    while (pList != NULL)
    {
        pUpdateRem = pList;
        pList = pList->pNext;
        free(pUpdateRem);
    }

    finishCheckForNode(pNodeInfo_p);

EXIT:
    if (ret != kFwReturnOk)
    {
        moduleCheckFailed(pNodeInfo_p);
    }

    return ret;
}

static BOOL isModularSupported(tFirmwareCheckNodeInfo* pNodeInfo_p)
{
    return ((pNodeInfo_p->featureFlags & NMT_FEATUREFLAGS_MODULAR_DEVICE) != 0);
}

static tFirmwareRet startGatheringModuleInfo(tFirmwareCheckNodeInfo* pNodeInfo_p)
{
    tFirmwareRet ret = kFwReturnOk;

    if ((pNodeInfo_p->nextCheckState == kFwModuleCheckStateInit) || (pNodeInfo_p->startCounter == 0u))
    {
        pNodeInfo_p->startCounter = FIRMWARE_CHECK_START_MODULE_CHECK_RETRIES;
        ret = processCheckStateMachine(pNodeInfo_p);
    }
    else
    {
        FWM_ERROR("Start of gathering module info requested, but next state should be %d, %u retries until reset\n",
                  pNodeInfo_p->nextCheckState, pNodeInfo_p->startCounter);
        pNodeInfo_p->startCounter--;
    }

    return ret;
}

static BOOL isExpectedSdoCompleteEvent(tFirmwareCheckNodeInfo* pNodeInfo_p,
                                       const tSdoComFinished* pSdoComFinished_p)
{
    return ((pSdoComFinished_p->sdoComConHdl == pNodeInfo_p->sdo.handle) &&
            (pSdoComFinished_p->targetIndex == pNodeInfo_p->sdo.index) &&
            (pSdoComFinished_p->targetSubIndex == pNodeInfo_p->sdo.subindex) &&
            (pSdoComFinished_p->nodeId == pNodeInfo_p->nodeId));
}

static tFirmwareRet issueSdoRead(tFirmwareCheckNodeInfo* pNodeInfo_p)
{
    tFirmwareRet ret = kFwReturnOk;
    tOplkError result;

    result = oplk_readObject(&pNodeInfo_p->sdo.handle,
                             pNodeInfo_p->nodeId,
                             pNodeInfo_p->sdo.index,
                             pNodeInfo_p->sdo.subindex,
                             pNodeInfo_p->sdo.pData,
                             &pNodeInfo_p->sdo.size,
                             FIRMWARE_CHECK_READ_SDO_TYPE,
                             pNodeInfo_p);

    if ((result != kErrorOk) && (result != kErrorApiTaskDeferred))
    {
        FWM_ERROR("(%s) - Reading the object 0x%x - 0x%x failed with 0x%x\n",
                  __func__, pNodeInfo_p->sdo.index, pNodeInfo_p->sdo.subindex, result);
        ret = kFwReturnSdoReadError;
    }

    return ret;
}


static tFirmwareRet getNumberOfModuleIdentIndices(tFirmwareCheckNodeInfo* pNodeInfo_p)
{
    tFirmwareRet ret = kFwReturnOk;
    tFirmwareCheckNodeSdo* pSdo = &pNodeInfo_p->sdo;

    pSdo->index = FIRMWARE_CHECK_INDEX_IDENT_INDICIES;
    pSdo->subindex = FIMRWARE_CHECK_SUBINDEX_NUMBER_OF_ENTRIES;
    pSdo->pData = &pNodeInfo_p->identIndices.numberOfIndices;
    pSdo->size = sizeof(UINT8);

    ret = issueSdoRead(pNodeInfo_p);

    return ret;
}

static tFirmwareRet getIndexOfModuleIdent(tFirmwareCheckNodeInfo* pNodeInfo_p,
                                          size_t index_p)
{
    tFirmwareRet ret = kFwReturnOk;
    tFirmwareCheckNodeSdo* pSdo = &pNodeInfo_p->sdo;

    pSdo->index = FIRMWARE_CHECK_INDEX_IDENT_INDICIES;
    pSdo->subindex = (UINT)(index_p + 1u);
    pSdo->pData = &pNodeInfo_p->identIndices.pIndices[index_p];
    pSdo->size = sizeof(UINT16);

    ret = issueSdoRead(pNodeInfo_p);

    return ret;
}

static tFirmwareRet getNumberOfEntriesInIdentObject(tFirmwareCheckNodeInfo* pNodeInfo_p,
                                                    size_t identArray_p)
{
    tFirmwareRet ret = kFwReturnOk;
    tFirmwareCheckNodeSdo* pSdo = &pNodeInfo_p->sdo;

    pSdo->index = pNodeInfo_p->identIndices.pIndices[identArray_p];
    pSdo->subindex = FIMRWARE_CHECK_SUBINDEX_NUMBER_OF_ENTRIES;
    pSdo->pData = &pNodeInfo_p->pIdentArrays[identArray_p].numberOfIdents;
    pSdo->size = sizeof(UINT8);

    ret = issueSdoRead(pNodeInfo_p);

    return ret;
}

static tFirmwareRet getModuleIdent(tFirmwareCheckNodeInfo* pNodeInfo_p,
                                   size_t identArray_p, size_t ident_p)
{
    tFirmwareRet ret = kFwReturnOk;
    tFirmwareCheckNodeSdo* pSdo = &pNodeInfo_p->sdo;
    tFirmwareCheckModuleEntry* pEntry;
    tFirmwareCheckModuleEntry** ppInsertIter = NULL;

    pEntry = malloc(sizeof(tFirmwareCheckModuleEntry));
    if (pEntry == NULL)
    {
        ret = kFwReturnNoRessource;
        goto EXIT;
    }

    memset(pEntry, 0, sizeof(tFirmwareCheckModuleEntry));

    ppInsertIter = &pNodeInfo_p->moduleList;

    while (*ppInsertIter != NULL)
    {
        ppInsertIter = &(*ppInsertIter)->pNext;
    }

    *ppInsertIter = pEntry;

    pSdo->index = pNodeInfo_p->identIndices.pIndices[identArray_p];
    pSdo->subindex = (UINT)(ident_p + 1u);
    pSdo->pData = &pEntry->fwInfo.ident;
    pSdo->size = sizeof(tFirmwareCheckModuleIdent);

    ret = issueSdoRead(pNodeInfo_p);

EXIT:
    if (ret != kFwReturnOk)
    {
        free(pEntry);
        if (ppInsertIter != NULL)
        {
            *ppInsertIter = NULL;
        }
    }
    return ret;
}

static tFirmwareRet getNumberOfModuleFwDownloadIndices(tFirmwareCheckNodeInfo* pNodeInfo_p)
{
    tFirmwareRet ret = kFwReturnOk;
    tFirmwareCheckNodeSdo* pSdo = &pNodeInfo_p->sdo;

    pSdo->index = FIRMWARE_CHECK_INDEX_FW_DOWN_INDICES;
    pSdo->subindex = FIMRWARE_CHECK_SUBINDEX_NUMBER_OF_ENTRIES;
    pSdo->pData = &pNodeInfo_p->fwDownloadIndices.numberOfIndices;
    pSdo->size = sizeof(UINT8);

    ret = issueSdoRead(pNodeInfo_p);

    return ret;
}

static tFirmwareRet getIndexOfModuleFwDown(tFirmwareCheckNodeInfo* pNodeInfo_p,
                                          size_t index_p)
{
    tFirmwareRet ret = kFwReturnOk;
    tFirmwareCheckNodeSdo* pSdo = &pNodeInfo_p->sdo;

    pSdo->index = FIRMWARE_CHECK_INDEX_FW_DOWN_INDICES;
    pSdo->subindex = (UINT)(index_p + 1u);
    pSdo->pData = &pNodeInfo_p->fwDownloadIndices.pIndices[index_p];
    pSdo->size = sizeof(UINT16);

    ret = issueSdoRead(pNodeInfo_p); if (ret != kFwReturnOk)
    {
        FWM_ERROR("(%s) - reading the index of the %zu fw fownload object failed with %d\n",
                  __func__, index_p, ret);
    }

    return ret;
}

static tFirmwareRet getNumberOfModuleFwDownload(tFirmwareCheckNodeInfo* pNodeInfo_p,
                                                size_t fwArray_p)
{
    tFirmwareRet ret = kFwReturnOk;
    tFirmwareCheckNodeSdo* pSdo = &pNodeInfo_p->sdo;

    pSdo->index = pNodeInfo_p->fwDownloadIndices.pIndices[fwArray_p];
    pSdo->subindex = FIMRWARE_CHECK_SUBINDEX_NUMBER_OF_ENTRIES;
    pSdo->pData = &pNodeInfo_p->pNumbersOfFwDownloads[fwArray_p];
    pSdo->size = sizeof(UINT8);

    ret = issueSdoRead(pNodeInfo_p); if (ret != kFwReturnOk)
    {
        FWM_ERROR("(%s) - reading the number of fw downloads failed with %d\n",
                  __func__, ret);
    }

    return ret;
}

static tFirmwareRet processSdoEventForModule(tFirmwareCheckNodeInfo* pNodeInfo_p,
                                             const tSdoComFinished* pSdoComFinished_p)
{
    tFirmwareRet ret = kFwReturnOk;
    BOOL fFailed = FALSE;

    if (pSdoComFinished_p->transferredBytes != pNodeInfo_p->sdo.size)
    {
        FWM_ERROR("Unexpected transferred bytes %u instead of %u for node %u index 0x%X subindex 0x%X\n",
                pSdoComFinished_p->transferredBytes,
                pNodeInfo_p->sdo.size,
                pNodeInfo_p->nodeId,
                pNodeInfo_p->sdo.index,
                pNodeInfo_p->sdo.subindex);

        fFailed = TRUE;
    }
    else
    {
        if (pSdoComFinished_p->sdoComConState == kSdoComTransferFinished)
        {
            ret = processCheckStateMachine(pNodeInfo_p);

            if (ret != kFwReturnOk)
            {
                FWM_ERROR("FW check for node %u failed with %d\n",
                          pNodeInfo_p->nodeId, ret);
                fFailed = TRUE;
            }
        }
        else
        {
            FWM_ERROR("ERROR: Expected SDO event received with error state %d and abort code 0x%x\n",
                      pSdoComFinished_p->sdoComConState, pSdoComFinished_p->abortCode);
            fFailed = TRUE;
        }
    }

    if (fFailed)
    {
        moduleCheckFailed(pNodeInfo_p);
    }

    return ret;
}

static tFirmwareRet processCheckStateMachine(tFirmwareCheckNodeInfo* pNodeInfo_p)
{
    tFirmwareRet ret = kFwReturnOk;
    tFirmwareCheckIdentInfo* pIdentArray;
    BOOL repeat;

    do
    {
        repeat = FALSE;

        pNodeInfo_p->checkState = pNodeInfo_p->nextCheckState;

        switch (pNodeInfo_p->checkState)
        {
            case kFwModuleCheckStateInit:
            case kFwModuleCheckStateGetNumberOfIdentIndices:
                ret = getNumberOfModuleIdentIndices(pNodeInfo_p);
                pNodeInfo_p->nextCheckState = kFwModuleCheckStateGetIndexOfIdentArray;
                pNodeInfo_p->identIndices.indexIdx = 0u;
                break;

            case kFwModuleCheckStateGetIndexOfIdentArray:
                if (pNodeInfo_p->identIndices.numberOfIndices > 0u)
                {
                    if (pNodeInfo_p->identIndices.pIndices == NULL)
                    {
                        pNodeInfo_p->identIndices.pIndices = malloc(pNodeInfo_p->identIndices.numberOfIndices * sizeof(UINT16));
                        memset(pNodeInfo_p->identIndices.pIndices, 0,
                               pNodeInfo_p->identIndices.numberOfIndices * sizeof(UINT16));
                    }

                    if (pNodeInfo_p->identIndices.indexIdx < pNodeInfo_p->identIndices.numberOfIndices)
                    {
                        ret = getIndexOfModuleIdent(pNodeInfo_p,
                                                    pNodeInfo_p->identIndices.indexIdx);
                        pNodeInfo_p->identIndices.indexIdx++;
                    }

                    if (pNodeInfo_p->identIndices.indexIdx >= pNodeInfo_p->identIndices.numberOfIndices)
                    {
                        pNodeInfo_p->identIndices.indexIdx = 0u;
                        pNodeInfo_p->nextCheckState = kFwModuleCheckStateGetNumberOfIdentsPerIndex;
                    }
                }
                else
                {
                    FWM_ERROR("Number of ident indices for node %d equals 0, aborting...\n",
                              pNodeInfo_p->nodeId);
                    pNodeInfo_p->nextUpdateState = kFwModuleUpdateStateComplete;
                }
                break;

            case kFwModuleCheckStateGetNumberOfIdentsPerIndex:
                if (pNodeInfo_p->pIdentArrays == NULL)
                {
                    pNodeInfo_p->pIdentArrays = malloc(pNodeInfo_p->identIndices.numberOfIndices * sizeof(tFirmwareCheckIdentInfo));
                    memset(pNodeInfo_p->pIdentArrays, 0, pNodeInfo_p->identIndices.numberOfIndices * sizeof(tFirmwareCheckIdentInfo));
                }

                if (pNodeInfo_p->identIndices.indexIdx < pNodeInfo_p->identIndices.numberOfIndices)
                {
                    ret = getNumberOfEntriesInIdentObject(pNodeInfo_p,
                                                          pNodeInfo_p->identIndices.indexIdx);
                    pNodeInfo_p->nextCheckState = kFwModuleCheckStateGetIdent;
                }
                else
                {
                    repeat = TRUE;
                    pNodeInfo_p->nextCheckState = kFwModuleCheckStateComplete;

                }
                break;

            case kFwModuleCheckStateGetIdent:
                pIdentArray = &pNodeInfo_p->pIdentArrays[pNodeInfo_p->identIndices.indexIdx];

                if (pIdentArray->identIdx < pIdentArray->numberOfIdents)
                {
                    ret = getModuleIdent(pNodeInfo_p,
                                         pNodeInfo_p->identIndices.indexIdx,
                                         pIdentArray->identIdx);
                    pIdentArray->identIdx++;
                }
                else
                {
                    repeat = TRUE;

                    pNodeInfo_p->identIndices.indexIdx++;
                    pNodeInfo_p->moduleIdx = 0;

                    if (pNodeInfo_p->identIndices.indexIdx < pNodeInfo_p->identIndices.numberOfIndices)
                    {
                        pNodeInfo_p->nextCheckState = kFwModuleCheckStateGetNumberOfIdentsPerIndex;
                    }
                    else
                    {
                        pIdentArray->identIdx = 0u;
                        pNodeInfo_p->nextCheckState = kFwModuleCheckStateCheckFw;
                        pNodeInfo_p->nextUpdateState = kFwModuleUpdateStateCheckFw;
                    }
                }
                break;

            case kFwModuleCheckStateCheckFw:
                break;

            case kFwModuleCheckStateComplete:
                break;
        }
    } while (repeat);

    // TODO: error handling
    ret = processUpdateStateMachine(pNodeInfo_p);

    return ret;
}

static tFirmwareRet processUpdateStateMachine(tFirmwareCheckNodeInfo* pNodeInfo_p)
{
    tFirmwareRet ret = kFwReturnOk;
    tFirmwareCheckModuleEntry** ppIter = &pNodeInfo_p->moduleList;
    tFirmwareCheckModuleEntry* pRem = NULL;
    size_t modIdx = 1u;
    BOOL repeat;

    do
    {
        repeat = FALSE;

        pNodeInfo_p->updateState = pNodeInfo_p->nextUpdateState;

        switch (pNodeInfo_p->updateState)
        {
            case kFwModuleUpdateStateInit:
                break;

            case kFwModuleUpdateStateCheckFw:
                repeat = TRUE;
                pNodeInfo_p->nextUpdateState = kFwModuleUpdateStateComplete;
                if (pNodeInfo_p->fModuleListContainsHead)
                {
                    pNodeInfo_p->nextUpdateState = kFwModuleUpdateStateGetNumberOfFwDownIndices;
                    ppIter = &(*ppIter)->pNext;
                }

                while (*ppIter != NULL)
                {
                    if (isFirmwareUpdateRequired(pNodeInfo_p, &(*ppIter)->fwInfo))
                    {
                        FWM_TRACE("Firmware update required for module %zu of node: %u\n",
                                  modIdx, pNodeInfo_p->nodeId);
                        pNodeInfo_p->nextUpdateState = kFwModuleUpdateStateGetNumberOfFwDownIndices;
                        (*ppIter)->moduleIndex = modIdx;
                        ppIter = &(*ppIter)->pNext;
                    }
                    else
                    {
                        FWM_TRACE("No firmware update required for module %zu of node: %u\n",
                                  modIdx, pNodeInfo_p->nodeId);
                        pRem = *ppIter;
                        *ppIter = pRem->pNext;
                        pRem->pNext = NULL;
                        free(pRem);
                    }
                    modIdx++;
                }
                break;

            case kFwModuleUpdateStateGetNumberOfFwDownIndices:
                ret = getNumberOfModuleFwDownloadIndices(pNodeInfo_p);
                pNodeInfo_p->nextUpdateState = kFwModuleUpdateStateGetIndexOfFwDown;
                pNodeInfo_p->fwDownloadIndices.indexIdx = 0u;
                break;

            case kFwModuleUpdateStateGetIndexOfFwDown:
                if (pNodeInfo_p->fwDownloadIndices.numberOfIndices > 0)
                {
                    if (pNodeInfo_p->fwDownloadIndices.pIndices == NULL)
                    {
                        pNodeInfo_p->fwDownloadIndices.pIndices = malloc(pNodeInfo_p->fwDownloadIndices.numberOfIndices * sizeof(UINT16));
                    }

                    if (pNodeInfo_p->fwDownloadIndices.indexIdx < pNodeInfo_p->fwDownloadIndices.numberOfIndices)
                    {
                        ret = getIndexOfModuleFwDown(pNodeInfo_p, pNodeInfo_p->fwDownloadIndices.indexIdx);
                        pNodeInfo_p->fwDownloadIndices.indexIdx++;
                    }

                    if (pNodeInfo_p->fwDownloadIndices.indexIdx >= pNodeInfo_p->fwDownloadIndices.numberOfIndices)
                    {
                        pNodeInfo_p->nextUpdateState = kFwModuleUpdateStateGetNumberOfFwDownPerIndex;
                        pNodeInfo_p->fwDownloadIndices.indexIdx = 0u;
                    }
                }
                else
                {
                    FWM_ERROR("Number of fw download indices equals 0, aborting...\n");
                    pNodeInfo_p->nextUpdateState = kFwModuleUpdateStateComplete;
                }
                break;

            case kFwModuleUpdateStateGetNumberOfFwDownPerIndex:
                if (pNodeInfo_p->fwDownloadIndices.numberOfIndices > 0)
                {
                    if (pNodeInfo_p->pNumbersOfFwDownloads == NULL)
                    {
                        pNodeInfo_p->pNumbersOfFwDownloads = malloc(sizeof(UINT16) * pNodeInfo_p->fwDownloadIndices.numberOfIndices);
                        memset(pNodeInfo_p->pNumbersOfFwDownloads, 0, sizeof(UINT16) * pNodeInfo_p->fwDownloadIndices.numberOfIndices);
                    }

                    if (pNodeInfo_p->fwDownloadIndices.indexIdx < pNodeInfo_p->fwDownloadIndices.numberOfIndices)
                    {
                        ret = getNumberOfModuleFwDownload(pNodeInfo_p, pNodeInfo_p->fwDownloadIndices.indexIdx);
                        pNodeInfo_p->fwDownloadIndices.indexIdx++;
                    }

                    if (pNodeInfo_p->fwDownloadIndices.indexIdx >= pNodeInfo_p->fwDownloadIndices.numberOfIndices)
                    {
                        pNodeInfo_p->nextUpdateState = kFwModuleUpdateStateCalculateFwDownsForUpdate;
                        pNodeInfo_p->fwDownloadIndices.indexIdx = 0u;
                    }
                }
                else
                {
                    FWM_ERROR("Number of fw download indices equals 0, aborting...\n");
                    pNodeInfo_p->nextUpdateState = kFwModuleUpdateStateComplete;
                }
                break;

            case kFwModuleUpdateStateCalculateFwDownsForUpdate:
                repeat = TRUE;
                while (*ppIter != NULL)
                {
                    if ((*ppIter)->moduleIndex > 0)
                    {
                        calcFwObjectForModule(pNodeInfo_p, (*ppIter)->moduleIndex,
                                              &(*ppIter)->fwInfo.index,
                                              &(*ppIter)->fwInfo.subindex);
                    }

                    ppIter = &(*ppIter)->pNext;
                }
                pNodeInfo_p->nextUpdateState = kFwModuleUpdateStateUpdateFw;
                break;

            case kFwModuleUpdateStateUpdateFw:
                repeat = TRUE;
                pNodeInfo_p->nextCheckState = kFwModuleCheckStateComplete;
                pNodeInfo_p->nextUpdateState = kFwModuleUpdateStateComplete;
                break;

            case kFwModuleUpdateStateComplete:
                ret = finishCheck(pNodeInfo_p);

                pNodeInfo_p->nextCheckState = kFwModuleCheckStateInit;
                pNodeInfo_p->nextUpdateState = kFwModuleUpdateStateInit;
                break;
        }
    } while (repeat);

    return ret;
}

static BOOL isFirmwareUpdateRequired(tFirmwareCheckNodeInfo* pNodeInfo_p,
                                     tFirmwareCheckFwInfo* pFwInfo_p)
{
    BOOL ret = FALSE;
    tFirmwareModuleInfo moduleInfo;
    tFirmwareInfo* pFirmwareInfo;
    tFirmwareRet fwReturn;

    memset(&moduleInfo, 0, sizeof(tFirmwareModuleInfo));

    moduleInfo.nodeId = pNodeInfo_p->nodeId;
    moduleInfo.vendorId = pFwInfo_p->ident.vendorId;
    moduleInfo.productId = pFwInfo_p->ident.productId;
    moduleInfo.hwVariant = pFwInfo_p->ident.revision;

    FWM_TRACE("Check firmware for node %u vendor 0x%x product 0x%x revision 0x%x\n",
              pNodeInfo_p->nodeId, moduleInfo.vendorId, moduleInfo.productId,
              moduleInfo.hwVariant);

    fwReturn = firmwareinfo_getInfoForNode(instance_l.config.pFwInfo,
                                      &moduleInfo,
                                      &pFirmwareInfo);
    if (fwReturn != kFwReturnOk)
    {
        goto EXIT;
    }

    if ((pFirmwareInfo->appSwDate != pFwInfo_p->ident.softwareDate) ||
        (pFirmwareInfo->appSwTime != pFwInfo_p->ident.softwareTime))
    {
        pFwInfo_p->pFwStoreHandle = pFirmwareInfo->pFwImage;
        ret = TRUE;
    }

EXIT:
    return ret;
}

static void calcFwObjectForModule(tFirmwareCheckNodeInfo* pNodeInfo_p,
                                  size_t moduleIndex_p,
                                  UINT* pIndex_p,
                                  UINT* pSubIndex_p)
{
    size_t indexIter;
    UINT modIndex = 0u;

    for (indexIter = 0u; indexIter < pNodeInfo_p->fwDownloadIndices.numberOfIndices; indexIter++)
    {
        if (moduleIndex_p <= modIndex + pNodeInfo_p->pNumbersOfFwDownloads[indexIter])
        {
            *pIndex_p = pNodeInfo_p->fwDownloadIndices.pIndices[indexIter];
            *pSubIndex_p = (UINT)(moduleIndex_p - modIndex);
            break;
        }
        else
        {
            modIndex += pNodeInfo_p->pNumbersOfFwDownloads[indexIter];
        }
    }
}

static void moduleCheckFailed(tFirmwareCheckNodeInfo* pNodeInfo_p)
{
    cleanupNode(pNodeInfo_p);
}

static void cleanupNode(tFirmwareCheckNodeInfo* pNodeInfo_p)
{
    tFirmwareCheckModuleEntry* pIter;
    tFirmwareCheckModuleEntry* pRem;

    free(pNodeInfo_p->identIndices.pIndices);
    pNodeInfo_p->identIndices.pIndices = NULL;
    free(pNodeInfo_p->fwDownloadIndices.pIndices);
    pNodeInfo_p->fwDownloadIndices.pIndices = NULL;
    free(pNodeInfo_p->pIdentArrays);
    pNodeInfo_p->pIdentArrays = NULL;
    free(pNodeInfo_p->pNumbersOfFwDownloads);
    pNodeInfo_p->pNumbersOfFwDownloads = NULL;

    pIter = pNodeInfo_p->moduleList;

    while (pIter != NULL)
    {
        pRem = pIter;

        pIter = pIter->pNext;

        free(pRem);
    }
    pNodeInfo_p->moduleList = NULL;

    pNodeInfo_p->fModuleListContainsHead = FALSE;
    pNodeInfo_p->checkState = kFwModuleCheckStateInit;
    pNodeInfo_p->nextCheckState= kFwModuleCheckStateInit;
    pNodeInfo_p->checkState = kFwModuleUpdateStateInit;
    pNodeInfo_p->nextUpdateState = kFwModuleUpdateStateInit;
    memset(&pNodeInfo_p->sdo, 0, sizeof(tFirmwareCheckNodeSdo));
    pNodeInfo_p->sdo.handle = FIRMWARECHECK_INVALID_SDO;
}

static void finishCheckForNode(tFirmwareCheckNodeInfo* pNodeInfo_p)
{
    FWM_TRACE("Firmware check finished for node %u\n", pNodeInfo_p->nodeId);

    cleanupNode(pNodeInfo_p);
}

static tFirmwareCheckNodeInfo* getNextNodeToCheck(void)
{
    size_t i;
    tFirmwareCheckNodeInfo* pNode = instance_l.pNextNodeToCheck;

    // TODO: replace by iterating through list

    if (pNode == NULL)
    {
        for (i = 0u; i < FIRMWARECHECK_MAX_NODEID; i++)
        {
            pNode = &instance_l.aNodeInfo[i];
            if ((pNode->nodeId != FIRMWARECHECK_INVALID_NODEID) && (isModularSupported(pNode)))
            {
                instance_l.pNextNodeToCheck = pNode;
                break;
            }
        }
    }
    else
    {
        do
        {
            pNode++;

            if ((pNode->nodeId != FIRMWARECHECK_INVALID_NODEID) && (isModularSupported(pNode)))
            {
                instance_l.pNextNodeToCheck = pNode;
                break;
            }

        } while (pNode != &instance_l.aNodeInfo[FIRMWARECHECK_MAX_NODEID -1]);

        if (instance_l.pNextNodeToCheck != pNode)
        {
            for (i = 0u; i < FIRMWARECHECK_MAX_NODEID; i++)
            {
                pNode = &instance_l.aNodeInfo[i];
                if ((pNode->nodeId != FIRMWARECHECK_INVALID_NODEID) && (isModularSupported(pNode)))
                {
                    instance_l.pNextNodeToCheck = pNode;
                    break;
                }
            }
        }

    }

    return instance_l.pNextNodeToCheck;
}

/// \}
