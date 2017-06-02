/**
********************************************************************************
 \file   firmwareupdate.c

\brief  Source file of the firmware update module

This module implements firmware updates of a node with wrong version.

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
#include <firmwaremanager/firmwaretrace.h>
#include <firmwaremanager/firmwareupdate.h>
#include <firmwaremanager/firmwarestore.h>

#include <oplk/oplk.h>

#include <stdio.h>
#include <errno.h>

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

#define FIRMWARE_UPDATE_MAX_NODE_ID     C_ADR_BROADCAST     ///< Maximum value of a node ID

#define FIRMWARE_UPDATE_SDO_TYPE        kSdoTypeAsnd        ///< SDO type used for firmware update
#define FIRMWARE_UPDATE_INVALID_SDO     ((tSdoComConHdl)-1) ///< Invalid SDO handle

#if !defined(FIRMWARE_UPDATE_MAX_PARALLEL_TRANSMISSIONS)
#define FIRMWARE_UPDATE_MAX_PARALLEL_TRANSMISSIONS 5        ///< Number of parallel transmissions of firmware images
#endif

//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------

/**
\brief Firmware update transmission information
*/
typedef struct
{
    BOOL                fTranmissionActive; ///< Active transmission
    tFirmwareUpdateList pUpdateList;        ///< Firmware update list
    void*               pFirmwareImage;     ///< Firmware image in progress
    size_t              firmwareSize;       ///< Size of firmware image in progress
    tSdoComConHdl       sdoComCon;          ///< SDO handle
} tFirmwareUpdateTransmissionInfo;

/**
\brief Firmware update instance
*/
typedef struct
{
    BOOL                            fInitialized;   ///< Instance initialized flag
    tFirmwareUpdateConfig           config;         ///< Instance configuration
    tFirmwareUpdateTransmissionInfo aTransmissions[FIRMWARE_UPDATE_MAX_NODE_ID]; ///< Node transmission array
    size_t                          numberOfStartedTransmissions;   ///< Number of started transmissions
    size_t                          numberOfFinishedTransmissions;  ///< Number of finished/aborted transmissions
} tFirmwareUpdateInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------

static tFirmwareUpdateInstance instance_l;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------

static tFirmwareRet checkPointerAndInstance(const void* pCheck_p);
static tFirmwareUpdateTransmissionInfo* getInfo(UINT nodeId_p);
static BOOL isExpectedSdoCompleteEvent(tFirmwareUpdateTransmissionInfo* pInfo_p,
                                       const tSdoComFinished* pSdoComFinished_p);
static tFirmwareRet transmitFirmware(tFirmwareUpdateTransmissionInfo* pInfo_p);
static tFirmwareRet startTransmission(tFirmwareUpdateTransmissionInfo* pInfo_p);

static void transmissionFailed(tFirmwareUpdateTransmissionInfo* pInfo_p);
static tFirmwareRet transmissionSucceeded(tFirmwareUpdateTransmissionInfo* pInfo_p);
static void cleanupTransmission(tFirmwareUpdateTransmissionInfo* pInfo_p);
static void cleanupInstance(void);
static BOOL isTransmissionAllowed(tFirmwareUpdateTransmissionInfo* pInfo_p);

static tFirmwareUpdateTransmissionInfo* getNextPendingTransmission(void);

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Initialize the firmware update module

This function initializes the firmware update module.

\param pConfig_p [in]       Pointer to the configuration structure for the
                            firmware update module.

\return This functions returns a value of \ref tFirmwareRet.

\ingroup module_app_firmwaremanager
*/
//------------------------------------------------------------------------------
tFirmwareRet firmwareupdate_init(const tFirmwareUpdateConfig* pConfig_p)
{
    tFirmwareRet ret = kFwReturnOk;

    if (instance_l.fInitialized)
    {
        ret = kFwReturnAlreadyInitialized;
        goto EXIT;
    }

    memset(&instance_l, 0, sizeof(tFirmwareUpdateInstance));

    memcpy(&instance_l.config, pConfig_p, sizeof(tFirmwareUpdateConfig));

    instance_l.fInitialized = TRUE;

EXIT:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Deinitialize the firmware update module

This function deinitializes the firmware update module.

\ingroup module_app_firmwaremanager
*/
//------------------------------------------------------------------------------
void firmwareupdate_exit(void)
{
    instance_l.fInitialized = FALSE;
}

//------------------------------------------------------------------------------
/**
\brief  Process the given list of firmware updates

This function processes the required firmware updates defined by the given list.

\param ppList_p [in, out]   Pointer to List of required firmware updates, when
                            an error occurs the returned list contains entries
                            which were not freed.

\return This functions returns a value of \ref tFirmwareRet.

\ingroup module_app_firmwaremanager
*/
//------------------------------------------------------------------------------
tFirmwareRet firmwareupdate_processUpdateList(tFirmwareUpdateList* ppList_p)
{
    tFirmwareRet                        ret = kFwReturnOk;
    tFirmwareUpdateEntry**              ppInsertIter;
    tFirmwareUpdateTransmissionInfo*    pInfo;

    ret = checkPointerAndInstance(ppList_p);
    if (ret != kFwReturnOk)
    {
        goto EXIT;
    }

    pInfo = &instance_l.aTransmissions[(*ppList_p)->nodeId];

    ppInsertIter = &pInfo->pUpdateList;

    while (*ppInsertIter != NULL)
    {
        ppInsertIter = &(*ppInsertIter)->pNext;
    }

    *ppInsertIter = (*ppList_p);

    *ppList_p = NULL;

    if (!pInfo->fTranmissionActive)
    {
        pInfo->sdoComCon = FIRMWARE_UPDATE_INVALID_SDO;
    }

    ret = startTransmission(pInfo);

EXIT:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Process an SDO event

This function processes SDO events given by the passed \ref tSdoComFinished
structure. By processing the according events this module checks the result of
firmware transmissions and proceeds for further required updates.

\param pSdoComFinished_p [in]   Structure of the SDO event

\return This functions returns a value of \ref tFirmwareRet.

\ingroup module_app_firmwaremanager
*/
//------------------------------------------------------------------------------
tFirmwareRet firmwareupdate_processSdoEvent(const tSdoComFinished* pSdoComFinished_p)
{
    tFirmwareRet                        ret = kFwReturnOk;
    tFirmwareUpdateTransmissionInfo*    pInfo = NULL;
    BOOL                                fSucceeded = TRUE;

    ret = checkPointerAndInstance(pSdoComFinished_p);
    if (ret != kFwReturnOk)
    {
        goto EXIT;
    }

    pInfo = &instance_l.aTransmissions[pSdoComFinished_p->nodeId];

    if (!isExpectedSdoCompleteEvent(pInfo, pSdoComFinished_p))
    {
        ret = kFwReturnInvalidSdoEvent;
        goto EXIT;
    }

    if (pSdoComFinished_p->sdoComConState != kSdoComTransferFinished)
    {
        FWM_ERROR("SDO write failed for node 0x%X with state: 0x%x and abort code 0x%x\n",
                  pSdoComFinished_p->nodeId, pSdoComFinished_p->sdoComConState, pSdoComFinished_p->abortCode);

        fSucceeded = FALSE;
    }

    if (pSdoComFinished_p->transferredBytes != pInfo->firmwareSize)
    {
        FWM_ERROR("SDO written number of bytes does not match for node 0x%X: %zu - %zu\n",
                  pSdoComFinished_p->nodeId, pSdoComFinished_p->transferredBytes, pInfo->firmwareSize);

        fSucceeded = FALSE;
    }

    if (pSdoComFinished_p->abortCode != 0)
    {
        FWM_ERROR("SDO event with abort code: 0x%x for node 0x%X\n", pSdoComFinished_p->abortCode, pSdoComFinished_p->nodeId);

        fSucceeded = FALSE;
    }

    if (fSucceeded)
    {
        ret = transmissionSucceeded(pInfo);
    }
    else
    {
        transmissionFailed(pInfo);
    }

    ret = startTransmission(pInfo);

EXIT:
    if (ret != kFwReturnOk)
    {
        cleanupInstance();
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Get update transmission status

This function returns the node's update transmission status.

\param pConfig_p [in]       Pointer to the configuration structure for the
                            firmware update module.
\param nodeId_p [in]        Node ID
\param pStatus_p [in]       Pointer used to return the transmission status

\return This functions returns a value of \ref tFirmwareRet.

\ingroup module_app_firmwaremanager
*/
//------------------------------------------------------------------------------
tFirmwareRet firmwareupdate_getTransmissionStatus(UINT nodeId_p,
                                                  tFirmwareUpdateTransmissionStatus* pStatus_p)
{
    tFirmwareRet                        ret = kFwReturnOk;
    tFirmwareUpdateTransmissionInfo*    pInfo;
    tFirmwareUpdateEntry*               pIter;
    UINT                                count = 0u;

    ret = checkPointerAndInstance(pStatus_p);
    if (ret != kFwReturnOk)
    {
        goto EXIT;
    }

    pInfo = getInfo(nodeId_p);
    if (pInfo == NULL)
    {
        ret = kFwReturnInvalidNodeId;
        goto EXIT;
    }

    pIter = pInfo->pUpdateList;

    while (pIter != NULL)
    {
        count++;
        pIter = pIter->pNext;
    }

    pStatus_p->fTransmissionActive = pInfo->fTranmissionActive;
    pStatus_p->numberOfPendingTransmissions = count;

EXIT:
    return ret;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief  Check pointer and instance

\param pCheck_p [in]        Pointer checked

\return This functions returns a value of \ref tFirmwareRet.
*/
//------------------------------------------------------------------------------
static tFirmwareRet checkPointerAndInstance(const void* pCheck_p)
{
    tFirmwareRet ret = kFwReturnOk;

    if (!instance_l.fInitialized)
    {
        ret = kFwReturnInvalidInstance;
    }
    else if (pCheck_p == NULL)
    {
        ret = kFwReturnInvalidParameter;
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Get transmission information

\param nodeId_p [in]        Node ID

\return This functions returns a pointer to the transmission information.
*/
//------------------------------------------------------------------------------
static tFirmwareUpdateTransmissionInfo* getInfo(UINT nodeId_p)
{
    tFirmwareUpdateTransmissionInfo* pInfo = NULL;

    if (nodeId_p <= FIRMWARE_UPDATE_MAX_NODE_ID)
    {
        pInfo = &instance_l.aTransmissions[nodeId_p];
    }

    return pInfo;
}

//------------------------------------------------------------------------------
/**
\brief  Check SDO complete event's validity

\param pInfo_p [in]             Pointer to transmission information
\param pSdoComFinished_p [in]   Pointer to SDO finish

\return This functions returns a \ref BOOL.
*/
//------------------------------------------------------------------------------
static BOOL isExpectedSdoCompleteEvent(tFirmwareUpdateTransmissionInfo* pInfo_p,
                                       const tSdoComFinished* pSdoComFinished_p)
{
    return ((pSdoComFinished_p->pUserArg == &instance_l) &&
            pInfo_p->fTranmissionActive &&
            (pInfo_p->pUpdateList != NULL) &&
            (pSdoComFinished_p->targetIndex == pInfo_p->pUpdateList->index) &&
            (pSdoComFinished_p->targetSubIndex == pInfo_p->pUpdateList->subindex) &&
            (pSdoComFinished_p->sdoComConHdl == pInfo_p->sdoComCon));
}

//------------------------------------------------------------------------------
/**
\brief  Transmit firmware update image

\param pInfo_p [in]             Pointer to transmission information

\return This functions returns a value of \ref tFirmwareRet.
*/
//------------------------------------------------------------------------------
static tFirmwareRet transmitFirmware(tFirmwareUpdateTransmissionInfo* pInfo_p)
{
    tFirmwareRet    ret = kFwReturnOk;
    tOplkError      oplkRet;

    if (pInfo_p->fTranmissionActive)
    {
        goto EXIT;
    }

    if (!isTransmissionAllowed(pInfo_p))
    {
        FWM_TRACE("Postpone update for node: %u index: 0x%x subindex: 0x%x\n",
                   pInfo_p->pUpdateList->nodeId, pInfo_p->pUpdateList->index,
                   pInfo_p->pUpdateList->subindex);

        goto EXIT;
    }

    FWM_TRACE("Start update for node: %u index: 0x%x subindex: 0x%x\n",
              pInfo_p->pUpdateList->nodeId, pInfo_p->pUpdateList->index,
              pInfo_p->pUpdateList->subindex);

    ret = firmwarestore_loadData(pInfo_p->pUpdateList->pStoreHandle);
    if (ret != kFwReturnOk)
    {
        FWM_ERROR("Loading image for transmission failed with %d and errno %d\n",
                ret, errno);
        goto EXIT;
    }

    ret = firmwarestore_getData(pInfo_p->pUpdateList->pStoreHandle,
                                &pInfo_p->pFirmwareImage,
                                &pInfo_p->firmwareSize);
    if (ret != kFwReturnOk)
    {
        FWM_ERROR("Getting the image for transmission failed with %d and errno %d\n",
                ret, errno);
        goto EXIT;
    }

    oplkRet = oplk_writeObject(&pInfo_p->sdoComCon,
                               pInfo_p->pUpdateList->nodeId,
                               pInfo_p->pUpdateList->index,
                               pInfo_p->pUpdateList->subindex,
                               pInfo_p->pFirmwareImage,
                               (UINT)pInfo_p->firmwareSize,
                               FIRMWARE_UPDATE_SDO_TYPE,
                               &instance_l);

    if ((oplkRet != kErrorApiTaskDeferred) && (pInfo_p->sdoComCon != FIRMWARE_UPDATE_INVALID_SDO))
    {
        FWM_ERROR("Writing the firmware object failed with 0x%X\n", oplkRet);
        ret = kFwReturnSdoWriteFailed;
        goto EXIT;
    }

    instance_l.numberOfStartedTransmissions++;
    pInfo_p->fTranmissionActive = TRUE;

EXIT:
    if (ret != kFwReturnOk)
    {
        (void)firmwarestore_flushData(pInfo_p->pUpdateList->pStoreHandle);
    }
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Start firmware update image transmission

\param pInfo_p [in]             Pointer to transmission information

\return This functions returns a value of \ref tFirmwareRet.
*/
//------------------------------------------------------------------------------
static tFirmwareRet startTransmission(tFirmwareUpdateTransmissionInfo* pInfo_p)
{
    tFirmwareRet ret = kFwReturnOk;

    while (pInfo_p->pUpdateList != NULL)
    {
        ret = transmitFirmware(pInfo_p);
        if (ret == kFwReturnOk)
        {
            break;
        }
        else
        {
            FWM_ERROR("Transmission of the firmware failed with %d\n", ret);
            transmissionFailed(pInfo_p);
        }
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Handle firmware update transmission failed

\param pInfo_p [in]             Pointer to transmission information

\return This functions returns a value of \ref tFirmwareRet.
*/
//------------------------------------------------------------------------------
static void transmissionFailed(tFirmwareUpdateTransmissionInfo* pInfo_p)
{
    pInfo_p->fTranmissionActive = FALSE;
    instance_l.numberOfFinishedTransmissions++;

    if (pInfo_p->pUpdateList->fIsNode)
    {
        if (instance_l.config.pfnError != NULL)
        {
            (void)instance_l.config.pfnError(pInfo_p->pUpdateList->nodeId,
                                             &pInfo_p->sdoComCon);
        }
    }

    cleanupTransmission(pInfo_p);
}

//------------------------------------------------------------------------------
/**
\brief  Handle firmware update transmission succeeded

\param pInfo_p [in]             Pointer to transmission information

\return This functions returns a value of \ref tFirmwareRet.
*/
//------------------------------------------------------------------------------
static tFirmwareRet transmissionSucceeded(tFirmwareUpdateTransmissionInfo* pInfo_p)
{
    tFirmwareRet                        ret = kFwReturnOk;
    tFirmwareUpdateTransmissionInfo*    pNextInfo;

    FWM_TRACE("Update finished for node: %u index: 0x%x subindex: 0x%x\n",
              pInfo_p->pUpdateList->nodeId, pInfo_p->pUpdateList->index,
              pInfo_p->pUpdateList->subindex);


    if (pInfo_p->pUpdateList->fIsNode)
    {
        // Node callback
        if (instance_l.config.pfnNodeUpdateComplete != NULL)
        {
            ret = instance_l.config.pfnNodeUpdateComplete(pInfo_p->pUpdateList->nodeId,
                                                          &pInfo_p->sdoComCon);
        }
    }
    else if (pInfo_p->pUpdateList->pNext == NULL)
    {
        // Module callback
        if (instance_l.config.pfnModuleUpdateComplete != NULL)
        {
            ret = instance_l.config.pfnModuleUpdateComplete(pInfo_p->pUpdateList->nodeId,
                                                            &pInfo_p->sdoComCon);
        }
    }

    instance_l.numberOfFinishedTransmissions++;

    if (pInfo_p->pUpdateList->pNext == NULL)
    {
        pNextInfo = getNextPendingTransmission();

        if (pNextInfo != NULL)
        {
            FWM_TRACE("Start next pending transmission for node 0x%X\n",
                      pNextInfo->pUpdateList->nodeId);
            startTransmission(pNextInfo);
        }
    }

    cleanupTransmission(pInfo_p);

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Clean up firmware update transmission

\param pInfo_p [in]             Pointer to transmission information
*/
//------------------------------------------------------------------------------
static void cleanupTransmission(tFirmwareUpdateTransmissionInfo* pInfo_p)
{
    tFirmwareUpdateEntry*   pRem;
    tFirmwareStoreHandle    pFwStore;

    pInfo_p->fTranmissionActive = FALSE;
    pFwStore = pInfo_p->pUpdateList->pStoreHandle;

    pRem = pInfo_p->pUpdateList;
    pInfo_p->pUpdateList = pInfo_p->pUpdateList->pNext;
    free(pRem);

    (void)firmwarestore_flushData(pFwStore);
}

//------------------------------------------------------------------------------
/**
\brief  Clean up firmware update instance
*/
//------------------------------------------------------------------------------
static void cleanupInstance(void)
{
    tFirmwareUpdateEntry*   pRem;
    size_t                  iter;

    for (iter = 0u; iter < FIRMWARE_UPDATE_MAX_NODE_ID; iter++)
    {
        pRem = instance_l.aTransmissions[iter].pUpdateList;
        while (pRem != NULL)
        {
            instance_l.aTransmissions[iter].pUpdateList = pRem->pNext;
            free(pRem);
            pRem = instance_l.aTransmissions[iter].pUpdateList;
        }
    }
}

static tFirmwareUpdateTransmissionInfo* getNextPendingTransmission(void)
{
    tFirmwareUpdateTransmissionInfo*    pInfo = NULL;
    size_t                              i;

    for (i = 0U; i < FIRMWARE_UPDATE_MAX_NODE_ID; i++)
    {

        if (!instance_l.aTransmissions[i].fTranmissionActive &&
                (instance_l.aTransmissions[i].pUpdateList != NULL))
        {
            pInfo = &instance_l.aTransmissions[i];
            break;
        }
    }

    return pInfo;
}

//------------------------------------------------------------------------------
/**
\brief  Check if the start of the given transmission is allowed

\return This functions returns a boolean value, which is true if the
        transmission is allowed.
*/
//------------------------------------------------------------------------------
static BOOL isTransmissionAllowed(tFirmwareUpdateTransmissionInfo* pInfo_p)
{
    size_t numberOfActiveTransmissions = instance_l.numberOfStartedTransmissions -
                                         instance_l.numberOfFinishedTransmissions;

    UNUSED_PARAMETER(pInfo_p);

    return (numberOfActiveTransmissions < FIRMWARE_UPDATE_MAX_PARALLEL_TRANSMISSIONS);

}

/// \}
