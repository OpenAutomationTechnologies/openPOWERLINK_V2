/**
********************************************************************************
 \file   firmwaremanager.c

\brief  Source file of the firmware manager

 This module implements the firmware maneger.

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
#include <firmwaremanager/firmwaremanager.h>
#include <firmwaremanager/firmwaretrace.h>
#include "firmwarestore.h"
#include "firmwareinfo.h"
#include "firmwarecheck.h"
#include "firmwareupdate.h"

#include <oplk/oplk.h>
#include <oplk/debugstr.h>

#include <stdio.h>

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

#ifndef tabentries
#define tabentries(aVar_p)      (sizeof(aVar_p) / sizeof(*(aVar_p)))
#endif

//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------

typedef tFirmwareRet (*tFirmwareProcessSdoEvent)(const tSdoComFinished* pSdoComFinished_p);

typedef struct
{
    BOOL                    fInitialized;
    tFirmwareStoreHandle    firmwareStore;
    tFirmwareInfoHandle     firmwareInfo;
} tFirmwareManagerInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------

static tFirmwareManagerInstance instance_l;

static tFirmwareProcessSdoEvent apfnProcessSdoEvent_l[] =
{
        firmwarecheck_processSdoEvent,
        firmwareupdate_processSdoEvent
};

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------

static tFirmwareRet firmwareUpdateNotRequired(UINT nodeId_p,
                                              tSdoComConHdl* pSdoConnection_p);
static tFirmwareRet updateCompleteCb(UINT nodeId_p,
                                     tSdoComConHdl* pSdoConnection_p);
static tFirmwareRet errorDuringUpdate(UINT nodeId_p,
                                      tSdoComConHdl* pSdoConnection_p);

static void cleanupSdo(tSdoComConHdl* pSdoConnection_p);

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

tFirmwareRet firmwaremanager_init(const char* fwInfoFileName_p)
{
    tFirmwareRet            ret = kFwReturnOk;
    tFirmwareStoreConfig    storeConfig;
    tFirmwareInfoConfig     infoConfig;
    tFirmwareCheckConfig    checkConfig;
    tFirmwareUpdateConfig   updateConfig;

    if (instance_l.fInitialized)
    {
        ret = kFwReturnAlreadyInitialized;
        goto EXIT;
    }

    memset(&instance_l, 0, sizeof(tFirmwareManagerInstance));

    memset(&storeConfig, 0, sizeof(tFirmwareStoreConfig));

    storeConfig.pFilename = fwInfoFileName_p;

    ret = firmwarestore_create(&storeConfig, &instance_l.firmwareStore);
    if (ret != kFwReturnOk)
    {
        goto EXIT;
    }

    memset(&infoConfig, 0, sizeof(infoConfig));

    infoConfig.pFwStore = instance_l.firmwareStore;

    ret = firmwareinfo_create(&infoConfig, &instance_l.firmwareInfo);

    memset(&checkConfig, 0, sizeof(tFirmwareCheckConfig));

    checkConfig.pFwInfo = instance_l.firmwareInfo;
    checkConfig.pfnNoUpdateRequired = firmwareUpdateNotRequired;
    checkConfig.pfnError = errorDuringUpdate;

    ret = firmwarecheck_init(&checkConfig);
    if (ret != kFwReturnOk)
    {
        goto EXIT;
    }

    memset(&updateConfig, 0, sizeof(tFirmwareUpdateConfig));

    updateConfig.pfnUpdateComplete = updateCompleteCb;
    updateConfig.pfnError = errorDuringUpdate;

    ret = firmwareupdate_init(&updateConfig);
    if (ret != kFwReturnOk)
    {
        goto EXIT;
    }

    instance_l.fInitialized = TRUE;

EXIT:
    if (ret != kFwReturnOk)
    {
        firmwareupdate_exit();
        firmwarecheck_exit();
    }

    return ret;
}

void firmwaremanager_exit(void)
{
    firmwareupdate_exit();
    firmwarecheck_exit();
    (void)firmwareinfo_destroy(instance_l.firmwareInfo);
    (void)firmwarestore_destroy(instance_l.firmwareStore);

    instance_l.fInitialized = FALSE;
}

tOplkError firmwaremanager_processEvent(tOplkApiEventType eventType_p,
                                        const tOplkApiEventArg* pEventArg_p,
                                        void* pUserArg_p)
{
    tOplkError      ret = kErrorOk;
    tFirmwareRet    fwRet;
    size_t          iter;

    UNUSED_PARAMETER(pUserArg_p);

    switch (eventType_p)
    {
        case kOplkApiEventSdo:
            for (iter = 0; iter < tabentries(apfnProcessSdoEvent_l); iter++)
            {
                fwRet = apfnProcessSdoEvent_l[iter](&pEventArg_p->sdoInfo);
                if (fwRet != kFwReturnInvalidSdoEvent)
                {
                    break;
                }
            }

            if (fwRet == kFwReturnInvalidSdoEvent)
            {
                // Event is ignored because no firmware module felt responsible for it.
                ret = kErrorOk;
            }
            else if (fwRet != kFwReturnOk)
            {
                ret = kErrorGeneralError;
            }
            else
            {
                ret = kErrorOk;
            }
            break;

        case kOplkApiEventNode:
            if ((eventType_p == kOplkApiEventNode) &&
                (pEventArg_p->nodeEvent.nodeEvent == kNmtNodeEventUpdateSw))
            {
                fwRet = firmwarecheck_processNodeEvent(pEventArg_p->nodeEvent.nodeId);
                if (fwRet == kFwReturnInterruptBoot)
                {
                    ret = kErrorReject;
                }
                else if (fwRet != kFwReturnOk)
                {
                    ret = kErrorGeneralError;
                }
                else
                {
                    ret = kErrorOk;
                }
            }
            break;

        default:
            break;
    }

    return ret;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

static tFirmwareRet firmwareUpdateNotRequired(UINT nodeId_p,
                                              tSdoComConHdl* pSdoConnection_p)
{
    tFirmwareRet ret = kFwReturnOk;
    tOplkError result;

    cleanupSdo(pSdoConnection_p);

    FWM_TRACE("No firmware update required for node %d, continuing boot/operation\n", nodeId_p);

    result = oplk_triggerMnStateChange(nodeId_p, kNmtNodeCommandSwOk);
    if (result != kErrorOk)
    {
        FWM_ERROR("(%s) - Triggering mn state change failed with %d\n", __func__, result);
    }

    return ret;
}

static tFirmwareRet updateCompleteCb(UINT nodeId_p,
                                     tSdoComConHdl* pSdoConnection_p)
{
    tFirmwareRet ret = kFwReturnOk;
    tOplkError result;

    cleanupSdo(pSdoConnection_p);

    FWM_TRACE("Firmware update complete for node %d\n", nodeId_p);

    result = oplk_triggerMnStateChange(nodeId_p, kNmtNodeCommandSwUpdated);
    if (result != kErrorOk)
    {
        FWM_ERROR("(%s) - Triggering mn state change failed with %d\n",
                  __func__, result);
    }

    return ret;
}

static tFirmwareRet errorDuringUpdate(UINT nodeId_p,
                                      tSdoComConHdl* pSdoConnection_p)
{
    tFirmwareRet ret = kFwReturnOk;
    tOplkError result;

    cleanupSdo(pSdoConnection_p);

    FWM_TRACE("An error occurred during check/update for node %d\n", nodeId_p);

    result = oplk_triggerMnStateChange(nodeId_p, kNmtNodeCommandSwErr);
    if (result != kErrorOk)
    {
        FWM_ERROR("(%s) - Triggering mn state change failed with %d\n",
                  __func__, result);
    }

    return ret;
}


static void cleanupSdo(tSdoComConHdl* pSdoConnection_p)
{
    (void)oplk_freeSdoChannel(*pSdoConnection_p);
    *pSdoConnection_p = FIRMWARECHECK_INVALID_SDO;
}

/// \}
