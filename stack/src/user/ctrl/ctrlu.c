/**
********************************************************************************
\file   ctrlu.c

\brief  User stack control module

This file contains the implementation of the user stack control module.

\ingroup module_ctrlu
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2017, B&R Industrial Automation GmbH
Copyright (c) 2015, SYSTEC electronic GmbH
Copyright (c) 2018, Kalycito Infotech Private Limited
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
#include <common/target.h>
#include <common/ami.h>
#include <user/ctrlu.h>
#include <user/ctrlucal.h>
#include <user/eventu.h>
#include <user/eventucal.h>
#include <user/timeru.h>
#include <user/errhndu.h>
#include <user/dllucal.h>
#include <user/nmtu.h>
#include <user/nmtcnu.h>
#include <user/obdu.h>
#include <user/obdal.h>
#include <user/timesyncu.h>
#include <oplk/dll.h>

#if (CONFIG_OBD_USE_STORE_RESTORE != FALSE)
#include <user/obdconf.h>
#endif

#if defined(CONFIG_INCLUDE_NMT_MN)
#include <user/nmtmnu.h>
#include <user/identu.h>
#include <user/statusu.h>
#include <user/syncu.h>
#endif

#if defined(CONFIG_INCLUDE_PDO)
#include <user/pdou.h>
#endif

#if (defined(CONFIG_INCLUDE_SDOS) || defined(CONFIG_INCLUDE_SDOC))
#include <user/sdoseq.h>
#include <user/sdocom.h>
#endif

#if defined(CONFIG_INCLUDE_CFM)
#include <user/cfmu.h>
#include <oplk/obdcdc.h>
#endif

#include <stddef.h>
#include <limits.h>

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
#if (CONFIG_OBD_USE_STORE_RESTORE != FALSE)
#define OBD_SIGNATURE_STORE     0x65766173L
#define OBD_SIGNATURE_RESTORE   0x64616F6CL
#endif

//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------

/**
\brief Structure describing link requests

This structure describes an object link request.
*/
typedef struct
{
    UINT                index;                  ///< Index of object to link
    void*               pVar;                   ///< Pointer to variable which should be linked
    UINT                varEntries;             ///< Number of entries.
    tObdSize            entrySize;              ///< Size of an entry
    UINT                firstSubindex;          ///< First sub-index this variable should be linked to.
} tLinkObjectRequest;

typedef struct
{
    UINT16              lastHeartbeat;          ///< Last detected heartbeat
    tOplkApiInitParam   initParam;              ///< Stack initialization parameters
    tCtrlKernelInfo     kernelInfo;             ///< Information about kernel stack
    UINT32              requiredKernelFeatures; ///< Kernel stack features we need to run correctly
    UINT32              usableKernelFeatures;   ///< Kernel stack features we use
    BOOL                fInitialized;           ///< Flag determines if stack is initialized/ready
} tCtrluInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tCtrluInstance   ctrlInstance_l;

#if defined(CONFIG_INCLUDE_NMT_MN)
static UINT8    aCmdData_l[C_MAX_NMT_CMD_DATA_SIZE];    // Extended NMT request command data
static size_t   nmtCmdDataSize_l;                       // NMT Command Data Size
// List of objects that need to get linked
static tLinkObjectRequest   aLinkObjectRequestsMn_l[] =
{//     Index       Variable        Count   Object size             SubIndex
    {   0x1F9F,     aCmdData_l,     1,      sizeof(aCmdData_l),     4   },
};
#endif

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static tOplkError initNmtu(const tOplkApiInitParam* pInitParam_p);
static tOplkError initObd(const tOplkApiInitParam* pInitParam_p);
static tOplkError updateDllConfig(const tOplkApiInitParam* pInitParam_p,
                                  BOOL fUpdateIdentity_p);
static tOplkError updateObd(const tOplkApiInitParam* pInitParam_p,
                            BOOL fDisableUpdateStoredConf_p);
static tOplkError cbObdAccess(tObdCbParam* pParam_p, BOOL fUserEvent_p);
static tOplkError handleObdLossOfFrameTolerance(const tObdCbParam* pParam_p);
static tOplkError handleObdVerifyConf(const tObdCbParam* pParam_p);
static tOplkError handleObdResetCmd(tObdCbParam* pParam_p);

#if defined(CONFIG_INCLUDE_IP)
static tOplkError handleObdIpAddrTable(const tObdCbParam* pParam_p);
#endif

static tOplkError processUserEvent(const tEvent* pEvent_p);
static tOplkError cbCnCheckEvent(tNmtEvent NmtEvent_p);
static tOplkError cbNmtStateChange(tEventNmtStateChange nmtStateChange_p);

#if (defined(CONFIG_INCLUDE_SDOS) || defined(CONFIG_INCLUDE_SDOC))
static tOplkError updateSdoConfig(void);
#endif

static tOplkError cbEventUserObdAccess(tObdAlConHdl* pObdAlConHdl_p);

#if defined(CONFIG_INCLUDE_PDO)
static tOplkError cbEventPdoChange(const tPdoEventPdoChange* pEventPdoChange_p);
#endif

#if defined(CONFIG_INCLUDE_NMT_MN)
static tOplkError cbNodeEvent(UINT nodeId_p,
                              tNmtNodeEvent nodeEvent_p,
                              tNmtState nmtState_p,
                              UINT16 errorCode_p,
                              BOOL fMandatory_p);
static tOplkError linkDomainObjects(const tLinkObjectRequest* pLinkRequest_p,
                                    size_t requestCnt_p);
static tOplkError handleObdRequestCmd(tObdCbParam* pParam_p);
#endif

static tOplkError cbBootEvent(tNmtBootEvent BootEvent_p,
                              tNmtState NmtState_p,
                              UINT16 errorCode_p);

#if defined(CONFIG_INCLUDE_CFM)
static tOplkError cbCfmEventCnProgress(const tCfmEventCnProgress* pEventCnProgress_p);
static tOplkError cbCfmEventCnResult(UINT nodeId_p, tNmtNodeCommand NodeCommand_p);
#endif
UINT32            getRequiredKernelFeatures(void);

#if (CONFIG_OBD_USE_STORE_RESTORE != FALSE)
static tOplkError storeOdPart(tObdCbParam* pParam_p);
static tOplkError restoreOdPart(tObdCbParam* pParam_p);
static tOplkError cbStoreLoadObject(const tObdCbStoreParam* pCbStoreParam_p);
static tOplkError initDefaultOdPartArchive(void);
#endif

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Initialize user control module

The function initializes the user control module.

\return The function returns a tOplkError error code.

\ingroup module_ctrlu
*/
//------------------------------------------------------------------------------
tOplkError ctrlu_init(void)
{
    tOplkError  ret;

    DEBUG_LVL_CTRL_TRACE("Initialize ctrl module ...\n");

    ctrlInstance_l.lastHeartbeat = 0;

    ret = ctrlucal_init();
    if (ret != kErrorOk)
    {
        DEBUG_LVL_ERROR_TRACE("Could not initialize ctrlucal\n");
        goto Exit;
    }

    ret = ctrlucal_checkKernelStack();
    if (ret != kErrorOk)
    {
        ctrlucal_exit();
        goto Exit;
    }

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Cleanup user control module

The function cleans up the user control module.

\ingroup module_ctrlu
*/
//------------------------------------------------------------------------------
void ctrlu_exit(void)
{
    ctrlucal_exit();
}

//------------------------------------------------------------------------------
/**
\brief  Check kernel stack information

The function checks the kernel stack version and feature flags.

\return The function returns a tOplkError error code.

\ingroup module_ctrlu
*/
//------------------------------------------------------------------------------
tOplkError ctrlu_checkKernelStackInfo(void)
{
    tOplkError  ret;

    ctrlInstance_l.requiredKernelFeatures = getRequiredKernelFeatures();

    ret = ctrlu_getKernelInfo(&ctrlInstance_l.kernelInfo);
    if (ret != kErrorOk)
        goto Exit;

    // Obtain the usable features by masking the kernel stack feature with the
    // required feature flags.
    ctrlInstance_l.usableKernelFeatures = ctrlInstance_l.requiredKernelFeatures &
                                          ctrlInstance_l.kernelInfo.featureFlags;

    if ((ctrlInstance_l.usableKernelFeatures == ctrlInstance_l.requiredKernelFeatures) &&
        (ctrlInstance_l.kernelInfo.version == PLK_DEFINED_STACK_VERSION))
    {
        DEBUG_LVL_ALWAYS_TRACE("Kernel features: 0x%08x\n", ctrlInstance_l.kernelInfo.featureFlags);
        DEBUG_LVL_ALWAYS_TRACE("Usable features: 0x%08x\n", ctrlInstance_l.usableKernelFeatures);
        DEBUG_LVL_ALWAYS_TRACE("Kernel version: 0x%08x\n", ctrlInstance_l.kernelInfo.version);
        ret = kErrorOk;
    }
    else
    {
        DEBUG_LVL_ERROR_TRACE("Kernel feature/version mismatch:\n");
        DEBUG_LVL_ERROR_TRACE("  Version: Is:%08x - required:%08x\n",
                              ctrlInstance_l.kernelInfo.version,
                              PLK_DEFINED_STACK_VERSION);
        DEBUG_LVL_ERROR_TRACE("  Features: Is:%08x - required:%08x\n",
                              ctrlInstance_l.kernelInfo.featureFlags,
                              ctrlInstance_l.requiredKernelFeatures);
        ret = kErrorFeatureMismatch;
    }

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Initialize openPOWERLINK stack

The function initializes the openPOWERLINK stack. It initializes all
user modules and communication with the kernel control module to initialize
the kernel modules.

After returning from this function, the application must start the NMT state
machine via oplk_execNmtCommand(kNmtEventSwReset) and thereby the whole
openPOWERLINK stack.

\param[in]      pInitParam_p        Pointer to the initialization parameters
                                    provided by the application.

\return The function returns a tOplkError error code.

\ingroup module_ctrlu
*/
//------------------------------------------------------------------------------
tOplkError ctrlu_initStack(const tOplkApiInitParam* pInitParam_p)
{
    tOplkError      ret = kErrorOk;
    tCtrlInitParam  ctrlParam;
    UINT16          retVal;

    // Check parameter validity
    ASSERT(pInitParam_p != NULL);

    // reset instance structure
    OPLK_MEMSET(&ctrlInstance_l.initParam, 0, sizeof(tOplkApiInitParam));
    OPLK_MEMCPY(&ctrlInstance_l.initParam,
                pInitParam_p,
                min(sizeof(tOplkApiInitParam), (size_t)pInitParam_p->sizeOfInitParam));

    // check event callback function pointer
    if (ctrlInstance_l.initParam.pfnCbEvent == NULL)
    {   // application must always have an event callback function
        ret = kErrorApiInvalidParam;
        goto Exit;
    }

    ret = initObd(&ctrlInstance_l.initParam);
    if (ret != kErrorOk)
        goto Exit;

#if (CONFIG_OBD_USE_STORE_RESTORE != FALSE)
    // Initialize target-specific obdconf module
    ret = obdconf_init();
    if (ret != kErrorOk)
        goto Exit;

    // Check the state of each OD part archive; create blank archives for non-existent ones
    ret = initDefaultOdPartArchive();
    if (ret != kErrorOk)
        goto Exit;

    // Register store/restore callback function
    ret = obdu_storeLoadObjCallback(cbStoreLoadObject);
    if (ret != kErrorOk)
        goto Exit;
#endif // (CONFIG_OBD_USE_STORE_RESTORE != FALSE)


    ret = obdal_init(cbEventUserObdAccess);
    if (ret != kErrorOk)
        goto Exit;

    DEBUG_LVL_CTRL_TRACE("Initializing kernel modules ...\n");
    OPLK_MEMCPY(ctrlParam.aMacAddress, ctrlInstance_l.initParam.aMacAddress, sizeof(ctrlParam.aMacAddress));
    strncpy(ctrlParam.aNetIfName, ctrlInstance_l.initParam.hwParam.pDevName, sizeof(ctrlParam.aNetIfName) - 1);
    ctrlucal_storeInitParam(&ctrlParam);

    ret = ctrlucal_executeCmd(kCtrlInitStack, &retVal);
    if (ret != kErrorOk)
        goto Exit;

    if ((tOplkError)retVal != kErrorOk)
    {
        ret = (tOplkError)retVal;
        goto Exit;
    }

    /* Read back init param because current MAC address was copied by DLLK */
    ret = ctrlucal_readInitParam(&ctrlParam);
    if (ret != kErrorOk)
        goto Exit;

    OPLK_MEMCPY(ctrlInstance_l.initParam.aMacAddress, ctrlParam.aMacAddress, 6);

    DEBUG_LVL_CTRL_TRACE("Initialize eventu module...\n");
    ret = eventu_init(processUserEvent);
    if (ret != kErrorOk)
        goto Exit;

    DEBUG_LVL_CTRL_TRACE("Initialize timeru module...\n");
    ret = timeru_init();
    if (ret != kErrorOk)
        goto Exit;

    DEBUG_LVL_CTRL_TRACE("Initialize error handler user module...\n");
    ret = errhndu_init();
    if (ret != kErrorOk)
        goto Exit;

    DEBUG_LVL_CTRL_TRACE("Initialize dllucal module...\n");
    ret = dllucal_init();
    if (ret != kErrorOk)
        goto Exit;

#if defined(CONFIG_INCLUDE_PDO)
    DEBUG_LVL_CTRL_TRACE("Initialize pdou module...\n");
    ret = pdou_init();
    if (ret != kErrorOk)
        goto Exit;

    ret = pdou_registerEventPdoChangeCb(cbEventPdoChange);
    if (ret != kErrorOk)
        goto Exit;
#endif

    DEBUG_LVL_CTRL_TRACE("Initialize timesync module...\n");
    ret = timesyncu_init(ctrlInstance_l.initParam.pfnCbSync);
    if (ret != kErrorOk)
        goto Exit;

    DEBUG_LVL_CTRL_TRACE("Initialize nmtu module...\n");
    ret = initNmtu(&ctrlInstance_l.initParam);
    if (ret != kErrorOk)
        goto Exit;

#if (defined(CONFIG_INCLUDE_SDOS) || defined(CONFIG_INCLUDE_SDOC))
    // init sdo command layer
    DEBUG_LVL_CTRL_TRACE("Initialize sdocom module...\n");
    ret = sdocom_init(pInitParam_p->sdoStackType,
                      obdal_processSdoWrite,
                      obdal_processSdoRead);
    if (ret != kErrorOk)
        goto Exit;
#endif

#if defined(CONFIG_INCLUDE_CFM)
    DEBUG_LVL_CTRL_TRACE("Initialize cfm module...\n");
    ret = cfmu_init(cbCfmEventCnProgress, cbCfmEventCnResult);
    if (ret != kErrorOk)
        goto Exit;
#endif

    // the application must start NMT state machine
    // via oplk_execNmtCommand(kNmtEventSwReset)
    // and thereby the whole POWERLINK stack
    ctrlInstance_l.fInitialized = TRUE;

    // linkDomainObjects requires an initialized stack
#if defined(CONFIG_INCLUDE_NMT_MN)
    ret = linkDomainObjects(aLinkObjectRequestsMn_l, tabentries(aLinkObjectRequestsMn_l));
#endif

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Shutdown openPOWERLINK stack

The function shuts down the openPOWERLINK stack. I cleans up all user modules
and the kernel modules by using the kernel control module.

\return The function returns a tOplkError error code.

\ingroup module_ctrlu
*/
//------------------------------------------------------------------------------
tOplkError ctrlu_shutdownStack(void)
{
    tOplkError  ret = kErrorOk;
    UINT16      retVal;

    ctrlInstance_l.fInitialized = FALSE;

#if defined(CONFIG_INCLUDE_CFM)
    ret = cfmu_exit();
    if (ret != kErrorOk)
    {
        DEBUG_LVL_ERROR_TRACE("cfmu_exit():    0x%X\n", ret);
    }
#endif

#if (defined(CONFIG_INCLUDE_SDOS) || defined(CONFIG_INCLUDE_SDOC))
    ret = sdocom_exit();
    if (ret != kErrorOk)
    {
        DEBUG_LVL_ERROR_TRACE("sdocom_exit():  0x%X\n", ret);
    }
#endif

#if defined(CONFIG_INCLUDE_NMT_MN)
    ret = nmtmnu_exit();
    if (ret != kErrorOk)
    {
        DEBUG_LVL_ERROR_TRACE("nmtmnu_exit():  0x%X\n", ret);
    }

    ret = identu_exit();
    if (ret != kErrorOk)
    {
        DEBUG_LVL_ERROR_TRACE("identu_exit():  0x%X\n", ret);
    }

    ret = statusu_exit();
    if (ret != kErrorOk)
    {
        DEBUG_LVL_ERROR_TRACE("statusu_exit(): 0x%X\n", ret);
    }

    ret = syncu_exit();
    if (ret != kErrorOk)
    {
        DEBUG_LVL_ERROR_TRACE("syncu_exit():   0x%X\n", ret);
    }
#endif

    ret = nmtcnu_exit();
    if (ret != kErrorOk)
    {
        DEBUG_LVL_ERROR_TRACE("nmtcnu_exit():  0x%X\n", ret);
    }

    ret = nmtu_exit();
    if (ret != kErrorOk)
    {
        DEBUG_LVL_ERROR_TRACE("nmtu_exit():    0x%X\n", ret);
    }

    timesyncu_exit();

#if defined(CONFIG_INCLUDE_PDO)
    ret = pdou_exit();
    if (ret != kErrorOk)
    {
        DEBUG_LVL_ERROR_TRACE("pdou_exit():    0x%X\n", ret);
    }
#endif

    ret = eventu_exit();
    if (ret != kErrorOk)
    {
        DEBUG_LVL_ERROR_TRACE("eventu_exit():  0x%X\n", ret);
    }

    ret = dllucal_exit();
    if (ret != kErrorOk)
    {
        DEBUG_LVL_ERROR_TRACE("dllucal_exit(): 0x%X\n", ret);
    }

    ret = errhndu_exit();
    if (ret != kErrorOk)
    {
        DEBUG_LVL_ERROR_TRACE("errhndu_exit(): 0x%X\n", ret);
    }

    ret = timeru_exit();
    if (ret != kErrorOk)
    {
        DEBUG_LVL_ERROR_TRACE("timeru_exit():  0x%X\n", ret);
    }

    /* shutdown kernel stack */
    ret = ctrlucal_executeCmd(kCtrlCleanupStack, &retVal);
    if (ret != kErrorOk)
    {
        DEBUG_LVL_ERROR_TRACE("shutdown kernel modules():  0x%X\n", ret);
    }

    ret = obdal_exit();
    if (ret != kErrorOk)
    {
        DEBUG_LVL_ERROR_TRACE("obdal_exit():   0x%X\n", ret);
    }

#if (CONFIG_OBD_USE_STORE_RESTORE != FALSE)
    ret = obdu_storeLoadObjCallback(NULL);
    if (ret != kErrorOk)
    {
        DEBUG_LVL_ERROR_TRACE("obdu_storeLoadObjCallback(): 0x%X\n", ret);
    }

    obdconf_exit();
#endif

#if defined(CONFIG_INCLUDE_CFM)
    obdcdc_exit();
#endif

    ret = obdu_exit();
    if (ret != kErrorOk)
    {
        DEBUG_LVL_ERROR_TRACE("obdu_exit():    0x%X\n", ret);
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Process openPOWERLINK stack

This function provides processing time to several tasks in the openPOWERLINK
stack.

\return The function returns a tOplkError error code.

\ingroup module_ctrlu
*/
//------------------------------------------------------------------------------
tOplkError ctrlu_processStack(void)
{
    tOplkError  ret;

    eventucal_process();

    ret = ctrlucal_process();
    if (ret != kErrorOk)
        goto Exit;

    ret = timeru_process();

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Check if kernel stack is running

The function checks if the kernel stack is still running.

\return Returns TRUE if the kernel stack is running or FALSE if it is not running.

\ingroup module_ctrlu
*/
//------------------------------------------------------------------------------
BOOL ctrlu_checkKernelStack(void)
{
    static UINT32   lastTimestamp = 0;
    UINT16          heartbeat;
    UINT32          timestamp;
    UINT32          diff;

    // don't exceed kernel heartbeat frequency
    timestamp = target_getTickCount();
    if (timestamp >= lastTimestamp)
        diff = timestamp - lastTimestamp;
    else
        diff = UINT_MAX - lastTimestamp + timestamp;

    if (diff < CONFIG_CHECK_HEARTBEAT_PERIOD)
        return TRUE;

    lastTimestamp = timestamp;

    heartbeat = ctrlucal_getHeartbeat();
    if (heartbeat == ctrlInstance_l.lastHeartbeat)
    {
        DEBUG_LVL_CTRL_TRACE("heartbeat:%d ctrlInstance_l.lastHeartbeat:%d\n",
                             heartbeat,
                             ctrlInstance_l.lastHeartbeat);
        return FALSE;
    }
    else
    {
        ctrlInstance_l.lastHeartbeat = heartbeat;
        return (ctrlucal_getStatus() == kCtrlStatusRunning);
    }
}

//------------------------------------------------------------------------------
/**
\brief  Get information about kernel stack

The function gets information about the version and features of the kernel stack.

\param[out]     pKernelInfo_p       Pointer to store kernel information.

\return The function returns a tOplkError error code.

\ingroup module_ctrlu
*/
//------------------------------------------------------------------------------
tOplkError ctrlu_getKernelInfo(tCtrlKernelInfo* pKernelInfo_p)
{
    UINT16  retVal;

    // Check parameter validity
    ASSERT(pKernelInfo_p != NULL);

    if (ctrlucal_executeCmd(kCtrlGetFeaturesHigh, &retVal) != kErrorOk)
        return kErrorNoResource;
    pKernelInfo_p->featureFlags = (retVal << 16);

    if (ctrlucal_executeCmd(kCtrlGetFeaturesLow, &retVal) != kErrorOk)
        return kErrorNoResource;
    pKernelInfo_p->featureFlags |= retVal;

    if (ctrlucal_executeCmd(kCtrlGetVersionHigh, &retVal) != kErrorOk)
        return kErrorNoResource;
    pKernelInfo_p->version = (retVal << 16);

    if (ctrlucal_executeCmd(kCtrlGetVersionLow, &retVal) != kErrorOk)
        return kErrorNoResource;
    pKernelInfo_p->version |= retVal;

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Call user event callback

The function calls the user event callback function

\param[in]      eventType_p         Event type to send.
\param[in]      pEventArg_p         Event argument to send.

\return The function returns a tOplkError error code.

\ingroup module_ctrlu
*/
//------------------------------------------------------------------------------
tOplkError ctrlu_callUserEventCallback(tOplkApiEventType eventType_p,
                                       const tOplkApiEventArg* pEventArg_p)
{
    tOplkError  ret = kErrorOk;

    // If the stack is not initialized but we get events, we don't forward
    // them to the application!
    if (ctrlInstance_l.fInitialized)
    {
        ret = ctrlInstance_l.initParam.pfnCbEvent(eventType_p,
                                                  (tOplkApiEventArg*)pEventArg_p,
                                                  ctrlInstance_l.initParam.pEventUserArg);
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Get Ethernet Interface MAC address

The function returns the Ethernet Interface MAC address used by the
Ethernet controller.

\return The function returns the Ethernet MAC address.

\ingroup module_ctrlu
*/
//------------------------------------------------------------------------------
const UINT8* ctrlu_getEthMacAddr(void)
{
    return &ctrlInstance_l.initParam.aMacAddress[0];
}

//------------------------------------------------------------------------------
/**
\brief  Returns the stacks initialization state

The function returns the initialization state of the stack.

\return The function returns TRUE if the stack is initialized and running or
        FALSE if it is shutdown.

\ingroup module_ctrlu
*/
//------------------------------------------------------------------------------
BOOL ctrlu_stackIsInitialized(void)
{
    return ctrlInstance_l.fInitialized;
}

//------------------------------------------------------------------------------
/**
\brief Returns kernel feature flags

The function returns the configured kernel features that are required by the user library.

\return The function returns the kernel feature flags.

\ingroup module_ctrlu
*/
//------------------------------------------------------------------------------
UINT32 ctrlu_getFeatureFlags(void)
{
    return getRequiredKernelFeatures();
}

//------------------------------------------------------------------------------
/**
\brief  Write file chunk

This function writes the given file chunk to the kernel stack.

\param[in]      pDesc_p             Descriptor for the file chunk.
\param[in]      pBuffer_p           Buffer holding the file chunk.

\return The function returns a \ref tOplkError error code.

\ingroup module_ctrlu
*/
//------------------------------------------------------------------------------
tOplkError ctrlu_writeFileChunk(const tOplkApiFileChunkDesc* pDesc_p,
                                const void* pBuffer_p)
{
    tOplkError      ret;
    UINT16          retval;

    // Check parameter validity
    ASSERT(pDesc_p != NULL);
    ASSERT(pBuffer_p != NULL);

    ret = ctrlucal_writeFileBuffer(pDesc_p, pBuffer_p);
    if (ret != kErrorOk)
        return ret;

    ret = ctrlucal_executeCmd(kCtrlWriteFileChunk, &retval);
    if (ret != kErrorOk)
        return ret;

    if (retval != kErrorOk)
        return (tOplkError)retval;

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Get maximum supported file chunk size

This function returns the maximum file chunk size which is supported by the
stack.

\return The function returns the supported file chunk size.

\ingroup module_ctrlu
*/
//------------------------------------------------------------------------------
size_t ctrlu_getMaxFileChunkSize(void)
{
    return ctrlucal_getFileBufferSize();
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief  initialize NMTU modules

The function initializes the NMTU modules.

\param[in]      pInitParam_p        Pointer to init parameters.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError initNmtu(const tOplkApiInitParam* pInitParam_p)
{
    tOplkError  ret = kErrorOk;

    // initialize nmtcnu module
    DEBUG_LVL_CTRL_TRACE("Initialize nmtcn module...\n");
    ret = nmtcnu_init(pInitParam_p->nodeId);
    if (ret != kErrorOk)
        goto Exit;

    ret = nmtcnu_registerCheckEventCb(cbCnCheckEvent);
    if (ret != kErrorOk)
        goto Exit;

    // initialize nmtu module
    DEBUG_LVL_CTRL_TRACE("Initialize nmtu module...\n");
    ret = nmtu_init();
    if (ret != kErrorOk)
        goto Exit;

    // register NMT event callback function
    ret = nmtu_registerStateChangeCb(cbNmtStateChange);
    if (ret != kErrorOk)
        goto Exit;

#if defined(CONFIG_INCLUDE_NMT_MN)
    // initialize nmtmnu module
    DEBUG_LVL_CTRL_TRACE("Initialize nmtmnu module...\n");
    ret = nmtmnu_init(cbNodeEvent, cbBootEvent);
    if (ret != kErrorOk)
        goto Exit;

    // initialize identu module
    DEBUG_LVL_CTRL_TRACE("Initialize identu module...\n");
    ret = identu_init();
    if (ret != kErrorOk)
        goto Exit;

    // initialize statusu module
    DEBUG_LVL_CTRL_TRACE("Initialize statusu module...\n");
    ret = statusu_init();
    if (ret != kErrorOk)
        goto Exit;

    // initialize syncu module
    DEBUG_LVL_CTRL_TRACE("Initialize syncu module...\n");
    ret = syncu_init();
    if (ret != kErrorOk)
        goto Exit;

#endif

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief Initialize object dictionary

The function initializes the object dictionary

\param[in]      pInitParam_p        Pointer to init parameters.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError initObd(const tOplkApiInitParam* pInitParam_p)
{
    tOplkError  ret = kErrorOk;

    DEBUG_LVL_CTRL_TRACE("Initialize obdu module...\n");
    ret = obdu_init(&pInitParam_p->obdInitParam, cbObdAccess);
    if (ret != kErrorOk)
        return ret;

#if defined(CONFIG_INCLUDE_CFM)
    ret = obdcdc_init();
#endif

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Callback function for NMT state change events

The function implements the callback function for node events.

\param[in]      nmtStateChange_p    NMT state change event.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError cbNmtStateChange(tEventNmtStateChange nmtStateChange_p)
{
    tOplkError          ret = kErrorOk;
    BYTE                nmtState;
    tOplkApiEventArg    eventArg;
    BOOL                fDisableUpdateStoredConf = FALSE;
#if (CONFIG_OBD_CALC_OD_SIGNATURE != FALSE)
    UINT32              signature = 0;
#endif

    // save NMT state in OD
    nmtState = (UINT8)nmtStateChange_p.newNmtState;
    ret = obdu_writeEntry(0x1F8C, 0, &nmtState, 1);
    if (ret != kErrorOk)
        return ret;

    // do work which must be done in that state
    switch (nmtStateChange_p.newNmtState)
    {
        // POWERLINK stack is not running
        case kNmtGsOff:
            break;

        // first init of the hardware
        case kNmtGsInitialising:
#if 0
#if defined(CONFIG_INCLUDE_SDO_UDP)
            // configure SDO via UDP (i.e. bind it to the POWERLINK Ethernet interface)
            ret = sdoudp_config(ctrlInstance_l.initParam.ipAddress, C_SDO_EPL_PORT);
            if (ret != kErrorOk)
                return ret;
#endif
#endif
            break;

        // init of the manufacturer-specific profile area and the
        // standardized device profile area
        case kNmtGsResetApplication:
            // reset application part of OD
            ret = obdu_accessOdPart(kObdPartApp, kObdDirLoad);
            if (ret != kErrorOk)
                return ret;
            break;

        // init of the communication profile area
        case kNmtGsResetCommunication:
            // reset communication part of OD
            ret = obdu_accessOdPart(kObdPartGen, kObdDirLoad);
            if (ret != kErrorOk)
                return ret;

#if (CONFIG_OBD_USE_STORE_RESTORE != FALSE)
            // Check if non-volatile memory of OD archive is valid, if no then set the force update flag to TRUE
#if (CONFIG_OBD_CALC_OD_SIGNATURE != FALSE)
            signature = (UINT32)obdu_getOdSignature(kObdPartGen);
#endif
            ret = obdconf_getPartArchiveState(kObdPartGen, signature);
            if (ret == kErrorOk)
                fDisableUpdateStoredConf = TRUE;
#endif
            // From 1.8.x: $$$ d.k.: update OD only if OD was not loaded from non-volatile memory
            ret = updateObd(&ctrlInstance_l.initParam, fDisableUpdateStoredConf);
            if (ret != kErrorOk)
                return ret;

#if defined(CONFIG_INCLUDE_CFM)
            ret = obdcdc_loadCdc();
            if (ret != kErrorOk)
                return ret;
#endif
            break;

        // build the configuration with infos from OD
        case kNmtGsResetConfiguration:
            ret = updateDllConfig(&ctrlInstance_l.initParam, TRUE);
            if (ret != kErrorOk)
                return ret;

#if (defined(CONFIG_INCLUDE_SDOS) || defined(CONFIG_INCLUDE_SDOC))
            ret = updateSdoConfig();
            if (ret != kErrorOk)
                return ret;
#endif
            break;

        //-----------------------------------------------------------
        // CN part of the state machine

        // node list for POWERLINK frames and check timeout
        case kNmtCsNotActive:
            // indicate completion of reset in NMT_ResetCmd_U8
            nmtState = (UINT8)kNmtCmdInvalidService;
            ret = obdu_writeEntry(0x1F9E, 0, &nmtState, 1);
            if (ret != kErrorOk)
                return ret;
            break;

        // node process only async frames
        case kNmtCsPreOperational1:
            break;

        // node process isochronous and asynchronous frames
        case kNmtCsPreOperational2:
            break;

        // node should be configured and application is ready
        case kNmtCsReadyToOperate:
            break;

        // normal work state
        case kNmtCsOperational:
            break;

        // node stopped by MN
        // -> only process asynchronous frames
        case kNmtCsStopped:
            break;

        // no POWERLINK cycle
        // -> normal Ethernet communication
        case kNmtCsBasicEthernet:
            break;

        //-----------------------------------------------------------
        // MN part of the state machine

        // node listens for POWERLINK-Frames and check timeout
        case kNmtMsNotActive:
            break;

        // node processes only async frames
        case kNmtMsPreOperational1:
            break;

        // node processes isochronous and asynchronous frames
        case kNmtMsPreOperational2:
            break;

        // node should be configured and application is ready
        case kNmtMsReadyToOperate:
            break;

        // normal work state
        case kNmtMsOperational:
            break;

        // no POWERLINK cycle
        // -> normal Ethernet communication
        case kNmtMsBasicEthernet:
            break;

        case kNmtRmsNotActive:
          break;

        default:
            DEBUG_LVL_CTRL_TRACE("%s(): unhandled NMT state\n", __func__);
            break;
    }

#if defined(CONFIG_INCLUDE_PDO)
    // forward event to pdou module
    ret = pdou_cbNmtStateChange(nmtStateChange_p);
    if (ret != kErrorOk)
        return ret;
#endif

#if defined(CONFIG_INCLUDE_NMT_MN)
    // forward event to NmtMn module
    ret = nmtmnu_cbNmtStateChange(nmtStateChange_p);
    if (ret != kErrorOk)
        return ret;
#endif

    // call user callback
    eventArg.nmtStateChange = nmtStateChange_p;
    ret = ctrlu_callUserEventCallback(kOplkApiEventNmtStateChange, &eventArg);

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Process User event

The function processes events which are sent to the API module. It checks the
events, creates an appropriate API event and forwards it to the application
by calling the event callback function.

\param[in]      pEvent_p            Event to process.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError processUserEvent(const tEvent* pEvent_p)
{
    tOplkError          ret = kErrorOk;
    const tEventError*  pEventError;
    tOplkApiEventType   eventType;
    tOplkApiEventArg    apiEventArg;

    switch (pEvent_p->eventType)
    {
        // error event
        case kEventTypeError:
            pEventError = (const tEventError*)pEvent_p->eventArg.pEventArg;
            switch (pEventError->eventSource)
            {
                // treat the errors from the following sources as critical
                case kEventSourceEventk:
                case kEventSourceEventu:
                case kEventSourceDllk:
                    eventType = kOplkApiEventCriticalError;
                    // halt the stack by entering NMT state Off
                    nmtu_postNmtEvent(kNmtEventCriticalError);
                    break;

                // the other errors are just warnings
                default:
                    eventType = kOplkApiEventWarning;
                    break;
            }

            // call user callback
            ctrlu_callUserEventCallback(eventType, (const tOplkApiEventArg*)pEventError);
            // discard error from callback function, because this could generate an endless loop
            ret = kErrorOk;
            break;

        // Error history entry event
        case kEventTypeHistoryEntry:
            if (pEvent_p->eventArgSize != sizeof(tErrHistoryEntry))
            {
                ret = kErrorEventWrongSize;
                break;
            }

            eventType = kOplkApiEventHistoryEntry;
            ret = ctrlu_callUserEventCallback(eventType, (const tOplkApiEventArg*)pEvent_p->eventArg.pEventArg);
            break;

        // user-defined event
        case kEventTypeApiUserDef:
            eventType = kOplkApiEventUserDef;
            apiEventArg.pUserArg = *(void**)pEvent_p->eventArg.pEventArg;
            ret = ctrlu_callUserEventCallback(eventType, &apiEventArg);
            break;

#if (defined(CONFIG_INCLUDE_NMT_MN) && defined(CONFIG_INCLUDE_PRES_FORWARD))
        case kEventTypeReceivedPres:
            {
                tOplkApiEventReceivedPres*      pApiData;
                const tDllEventReceivedPres*    pDllData;

                pApiData = &apiEventArg.receivedPres;
                pDllData = (const tDllEventReceivedPres*)pEvent_p->eventArg.pEventArg;

                pApiData->nodeId = (UINT)pDllData->nodeId;
                pApiData->frameSize = (size_t)pDllData->frameSize;
                pApiData->pFrame = (tPlkFrame*)pDllData->frameBuf;

                eventType = kOplkApiEventReceivedPres;
                ret = ctrlu_callUserEventCallback(eventType, &apiEventArg);
            }
            break;
#endif

        // at present, there are no other events for this module
        default:
            ret = kErrorInvalidEvent;
            break;
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Update the DLL configuration

The function updates the DLL configuration.

\param[in]      pInitParam_p        Pointer to the stack initialization parameters.
\param[in]      fUpdateIdentity_p   Flag determines if identity will also be updated.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError updateDllConfig(const tOplkApiInitParam* pInitParam_p,
                                  BOOL fUpdateIdentity_p)
{
    tOplkError      ret = kErrorOk;
    tDllConfigParam dllConfigParam;
    tDllIdentParam  dllIdentParam;
    tObdSize        obdSize;
    UINT16          wordValue;
    UINT8           byteValue;

    // configure Dll
    OPLK_MEMSET(&dllConfigParam, 0, sizeof(dllConfigParam));
    dllConfigParam.nodeId = obdu_getNodeId();

    // Cycle Length (0x1006: NMT_CycleLen_U32) in [us]
    obdSize = 4;
    ret = obdu_readEntry(0x1006, 0, &dllConfigParam.cycleLen, &obdSize);
    if (ret != kErrorOk)
        return ret;

    // 0x1F82: NMT_FeatureFlags_U32
    obdSize = 4;
    ret = obdu_readEntry(0x1F82, 0, &dllConfigParam.featureFlags, &obdSize);
    if (ret != kErrorOk)
        return ret;

    // d.k. There is no dependence between FeatureFlags and async-only CN
    dllConfigParam.fAsyncOnly = pInitParam_p->fAsyncOnly;

    // 0x1C14: DLL_LossOfFrameTolerance_U32 in [ns]
    obdSize = 4;
    ret = obdu_readEntry(0x1C14, 0, &dllConfigParam.lossOfFrameTolerance, &obdSize);
    if (ret != kErrorOk)
        return ret;

    // 0x1F98: NMT_CycleTiming_REC, 0x1F98.1: IsochrTxMaxPayload_U16
    obdSize = 2;
    ret = obdu_readEntry(0x1F98, 1, &wordValue, &obdSize);
    if (ret != kErrorOk)
        return ret;
    dllConfigParam.isochrTxMaxPayload = wordValue;

    // 0x1F98.2: IsochrRxMaxPayload_U16
    obdSize = 2;
    ret = obdu_readEntry(0x1F98, 2, &wordValue, &obdSize);
    if (ret != kErrorOk)
        return ret;
    dllConfigParam.isochrRxMaxPayload = wordValue;

    // 0x1F98.3: PResMaxLatency_U32
    obdSize = 4;
    ret = obdu_readEntry(0x1F98, 3, &dllConfigParam.presMaxLatency, &obdSize);
    if (ret != kErrorOk)
        return ret;

    // 0x1F98.4: PReqActPayloadLimit_U16
    obdSize = 2;
    ret = obdu_readEntry(0x1F98, 4, &wordValue, &obdSize);
    if (ret != kErrorOk)
        return ret;
    dllConfigParam.preqActPayloadLimit = wordValue;

    // 0x1F98.5: PResActPayloadLimit_U16
    obdSize = 2;
    ret = obdu_readEntry(0x1F98, 5, &wordValue, &obdSize);
    if (ret != kErrorOk)
        return ret;
    dllConfigParam.presActPayloadLimit = wordValue;

    // 0x1F98.6: ASndMaxLatency_U32
    obdSize = 4;
    ret = obdu_readEntry(0x1F98, 6, &dllConfigParam.asndMaxLatency, &obdSize);
    if (ret != kErrorOk)
        return ret;

    // 0x1F98.7: MultiplCycleCnt_U8
    obdSize = 1;
    ret = obdu_readEntry(0x1F98, 7, &byteValue, &obdSize);
    if (ret != kErrorOk)
        return ret;
    dllConfigParam.multipleCycleCnt = byteValue;

    // 0x1F98.8: AsyncMTU_U16
    obdSize = 2;
    ret = obdu_readEntry(0x1F98, 8, &wordValue, &obdSize);
    if (ret != kErrorOk)
        return ret;
    dllConfigParam.asyncMtu = wordValue;

    // 0x1F98.9: Prescaler_U16
    obdSize = 2;
    ret = obdu_readEntry(0x1F98, 9, &wordValue, &obdSize);
    if (ret != kErrorOk)
        return ret;
    //User value passed from application will already be updated by now in updateObd
    dllConfigParam.prescaler = wordValue;

#if defined(CONFIG_INCLUDE_NMT_MN)
    // 0x1F8A.1: WaitSoCPReq_U32 in [ns]
    obdSize = 4;
    ret = obdu_readEntry(0x1F8A, 1, &dllConfigParam.waitSocPreq, &obdSize);
    if (ret != kErrorOk)
        return ret;

    // 0x1F8A.2: AsyncSlotTimeout_U32 in [ns] (optional)
    obdSize = 4;
    obdu_readEntry(0x1F8A, 2, &dllConfigParam.asyncSlotTimeout, &obdSize);
#endif

#if CONFIG_DLL_PRES_CHAINING_CN != FALSE
    dllConfigParam.syncResLatency = pInitParam_p->syncResLatency;
#endif

#if defined(CONFIG_INCLUDE_NMT_RMN)
    {
        UINT32  mnSwitchOverPriority = 0;
        UINT32  mnSwitchOverDelay = 0;
        UINT32  mnSwitchOverCycleDivider = 0;
        UINT32  mnWaitNotAct = 0;

        obdSize = 4;
        ret = obdu_readEntry(0x1F89, 0x0a, &mnSwitchOverPriority, &obdSize);
        if (ret != kErrorOk)
            return ret;

        obdSize = 4;
        ret = obdu_readEntry(0x1F89, 0x0b, &mnSwitchOverDelay, &obdSize);
        if (ret != kErrorOk)
            return ret;

        obdSize = 4;
        ret = obdu_readEntry(0x1F89, 0x0c, &mnSwitchOverCycleDivider, &obdSize);
        if (ret != kErrorOk)
            return ret;

        dllConfigParam.switchOverTimeMn = (UINT32)(dllConfigParam.cycleLen +
                                                  (dllConfigParam.cycleLen * (UINT64)mnSwitchOverPriority /
                                                   mnSwitchOverCycleDivider));

        dllConfigParam.delayedSwitchOverTimeMn = (UINT32)(dllConfigParam.cycleLen +
                                                         (dllConfigParam.cycleLen * ((UINT64)mnSwitchOverPriority + mnSwitchOverDelay) /
                                                          mnSwitchOverCycleDivider));

        obdSize = 4;
        ret = obdu_readEntry(0x1F89, 0x01, &mnWaitNotAct, &obdSize);
        if (ret != kErrorOk)
            return ret;

        dllConfigParam.reducedSwitchOverTimeMn = mnWaitNotAct;
    }
#endif

    dllConfigParam.fSyncOnPrcNode = pInitParam_p->fSyncOnPrcNode;
    dllConfigParam.syncNodeId = pInitParam_p->syncNodeId;
    dllConfigParam.minSyncTime = pInitParam_p->minSyncTime;

    dllConfigParam.sizeOfStruct = (UINT32)sizeof(dllConfigParam);
    ret = dllucal_config(&dllConfigParam);
    if (ret != kErrorOk)
        return ret;

    if (fUpdateIdentity_p != FALSE)
    {
        // configure Identity
        OPLK_MEMSET(&dllIdentParam, 0, sizeof(dllIdentParam));

        obdSize = 4;
        ret = obdu_readEntry(0x1000, 0, &dllIdentParam.deviceType, &obdSize);
        if (ret != kErrorOk)
            return ret;

        obdSize = 4;
        ret = obdu_readEntry(0x1018, 1, &dllIdentParam.vendorId, &obdSize);
        if (ret != kErrorOk)
            return ret;

        obdSize = 4;
        ret = obdu_readEntry(0x1018, 2, &dllIdentParam.productCode, &obdSize);
        if (ret != kErrorOk)
            return ret;

        obdSize = 4;
        ret = obdu_readEntry(0x1018, 3, &dllIdentParam.revisionNumber, &obdSize);
        if (ret != kErrorOk)
            return ret;

        obdSize = 4;
        ret = obdu_readEntry(0x1018, 4, &dllIdentParam.serialNumber, &obdSize);
        if (ret != kErrorOk)
            return ret;

        dllIdentParam.ipAddress = pInitParam_p->ipAddress;
        dllIdentParam.subnetMask = pInitParam_p->subnetMask;

        obdSize = sizeof(dllIdentParam.defaultGateway);
        ret = obdu_readEntry(0x1E40, 5, &dllIdentParam.defaultGateway, &obdSize);
        if (ret != kErrorOk)
        {   // NWL_IpAddrTable_Xh_REC.DefaultGateway_IPAD seams to not exist,
            // so use the one supplied in the init parameter
            dllIdentParam.defaultGateway = pInitParam_p->defaultGateway;
        }

#if defined(CONFIG_INCLUDE_VETH)
        // configure Virtual Ethernet Driver
        ret = target_setIpAdrs(PLK_VETH_NAME,
                               dllIdentParam.ipAddress,
                               dllIdentParam.subnetMask,
                               dllConfigParam.asyncMtu);
        if (ret != kErrorOk)
            return ret;

#if (CONFIG_VETH_SET_DEFAULT_GATEWAY != FALSE)
        ret = target_setDefaultGateway(dllIdentParam.defaultGateway);
        if (ret != kErrorOk)
            return ret;
#endif

#endif

        obdSize = sizeof(dllIdentParam.sHostname);
        ret = obdu_readEntry(0x1F9A, 0, &dllIdentParam.sHostname[0], &obdSize);
        if (ret != kErrorOk)
        {   // NMT_HostName_VSTR seams to not exist,
            // so use the one supplied in the init parameter
            OPLK_MEMCPY(dllIdentParam.sHostname, pInitParam_p->sHostname, sizeof(dllIdentParam.sHostname));
        }

        obdSize = 4;
        obdu_readEntry(0x1020, 1, &dllIdentParam.verifyConfigurationDate, &obdSize);
        // ignore any error, because this object is optional

        obdSize = 4;
        obdu_readEntry(0x1020, 2, &dllIdentParam.verifyConfigurationTime, &obdSize);
        // ignore any error, because this object is optional

        dllIdentParam.applicationSwDate = pInitParam_p->applicationSwDate;
        dllIdentParam.applicationSwTime = pInitParam_p->applicationSwTime;

        dllIdentParam.vendorSpecificExt1 = pInitParam_p->vendorSpecificExt1;
        OPLK_MEMCPY(&dllIdentParam.aVendorSpecificExt2[0],
                    &pInitParam_p->aVendorSpecificExt2[0],
                    sizeof(dllIdentParam.aVendorSpecificExt2));

        dllIdentParam.sizeOfStruct = (UINT32)sizeof(dllIdentParam);
        ret = dllucal_setIdentity(&dllIdentParam);
        if (ret != kErrorOk)
            return ret;
    }

    return ret;
}

#if (defined(CONFIG_INCLUDE_SDOS) || defined(CONFIG_INCLUDE_SDOC))
//------------------------------------------------------------------------------
/**
\brief  Update the SDO configuration

The function updates the SDO configuration from the object dictionary.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError updateSdoConfig(void)
{
    tOplkError  ret = kErrorOk;
    tObdSize    obdSize;
    DWORD       sdoSequTimeout;

    obdSize = sizeof(sdoSequTimeout);
    ret = obdu_readEntry(0x1300, 0, &sdoSequTimeout, &obdSize);
    if (ret != kErrorOk)
        return ret;

    ret = sdoseq_setTimeout(sdoSequTimeout);
    return ret;
}
#endif

//------------------------------------------------------------------------------
/**
\brief  Update the object dictionary

The function updates the object dictionary from the stack initialization
parameters.

\param[in]      pInitParam_p                Pointer to the stack initialization parameters.
\param[in]      fDisableUpdateStoredConf_p  Flag to disable update to object entries which
                                            are set from the stored configuration in
                                            non-volatile memory.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError updateObd(const tOplkApiInitParam* pInitParam_p,
                            BOOL fDisableUpdateStoredConf_p)
{
    tOplkError  ret = kErrorOk;
    UINT16      wordValue;
    UINT8       byteValue;

    // set node id in OD
    ret = obdu_setNodeId(pInitParam_p->nodeId,      // node id
                         kObdNodeIdHardware);       // set by hardware
    if (ret != kErrorOk)
        return ret;

    if (!fDisableUpdateStoredConf_p && (pInitParam_p->cycleLen != UINT_MAX))
        obdu_writeEntry(0x1006, 0, &pInitParam_p->cycleLen, 4);

    if (!fDisableUpdateStoredConf_p && (pInitParam_p->lossOfFrameTolerance != UINT_MAX))
        obdu_writeEntry(0x1C14, 0, &pInitParam_p->lossOfFrameTolerance, 4);

    // d.k. There is no dependence between FeatureFlags and async-only CN.
    if (pInitParam_p->featureFlags != UINT_MAX)
        obdu_writeEntry(0x1F82, 0, &pInitParam_p->featureFlags, 4);

    wordValue = pInitParam_p->isochrTxMaxPayload;
    obdu_writeEntry(0x1F98, 1, &wordValue, 2);

    wordValue = pInitParam_p->isochrRxMaxPayload;
    obdu_writeEntry(0x1F98, 2, &wordValue, 2);

    obdu_writeEntry(0x1F98, 3, &pInitParam_p->presMaxLatency, 4);

    if (!fDisableUpdateStoredConf_p && (pInitParam_p->preqActPayloadLimit <= C_DLL_ISOCHR_MAX_PAYL))
    {
        wordValue = pInitParam_p->preqActPayloadLimit;
        obdu_writeEntry(0x1F98, 4, &wordValue, 2);
    }

    if (!fDisableUpdateStoredConf_p && (pInitParam_p->presActPayloadLimit <= C_DLL_ISOCHR_MAX_PAYL))
    {
        wordValue = pInitParam_p->presActPayloadLimit;
        obdu_writeEntry(0x1F98, 5, &wordValue, 2);
    }

    obdu_writeEntry(0x1F98, 6, &pInitParam_p->asndMaxLatency, 4);

    if (!fDisableUpdateStoredConf_p)
    {
        byteValue = pInitParam_p->multiplCylceCnt;
        obdu_writeEntry(0x1F98, 7, &byteValue, 1);
    }

    if (!fDisableUpdateStoredConf_p && (pInitParam_p->asyncMtu <= C_DLL_MAX_ASYNC_MTU))
    {
        wordValue = pInitParam_p->asyncMtu;
        obdu_writeEntry(0x1F98, 8, &wordValue, 2);
    }

    if (!fDisableUpdateStoredConf_p && (pInitParam_p->prescaler <= 1000))
    {
        wordValue = pInitParam_p->prescaler;
        obdu_writeEntry(0x1F98, 9, &wordValue, 2);
    }

#if defined(CONFIG_INCLUDE_NMT_MN)
    if (!fDisableUpdateStoredConf_p && (pInitParam_p->waitSocPreq != UINT_MAX))
        obdu_writeEntry(0x1F8A, 1, &pInitParam_p->waitSocPreq, 4);

    if (!fDisableUpdateStoredConf_p && (pInitParam_p->asyncSlotTimeout != 0) &&
        (pInitParam_p->asyncSlotTimeout != UINT_MAX))
        obdu_writeEntry(0x1F8A, 2, &pInitParam_p->asyncSlotTimeout, 4);
#endif

#if defined(CONFIG_INCLUDE_NMT_RMN)
    {
        UINT32  switchOverPriority;

        if (pInitParam_p->nodeId > C_ADR_MN_DEF_NODE_ID)
        {
            switchOverPriority = pInitParam_p->nodeId - C_ADR_MN_DEF_NODE_ID;
            obdu_writeEntry(0x1F89, 0x0a, &switchOverPriority, 4);
        }
    }
#endif

    // configure Identity
    if (pInitParam_p->deviceType != UINT_MAX)
        obdu_writeEntry(0x1000, 0, &pInitParam_p->deviceType, 4);

    if (pInitParam_p->vendorId != UINT_MAX)
        obdu_writeEntry(0x1018, 1, &pInitParam_p->vendorId, 4);

    if (pInitParam_p->productCode != UINT_MAX)
        obdu_writeEntry(0x1018, 2, &pInitParam_p->productCode, 4);

    if (pInitParam_p->revisionNumber != UINT_MAX)
        obdu_writeEntry(0x1018, 3, &pInitParam_p->revisionNumber, 4);

    if (pInitParam_p->serialNumber != UINT_MAX)
        obdu_writeEntry(0x1018, 4, &pInitParam_p->serialNumber, 4);

    if (pInitParam_p->pDevName != NULL)
    {
        // write Device Name (0x1008)
        obdu_writeEntry(0x1008,
                        0,
                        pInitParam_p->pDevName,
                        (tObdSize)strlen(pInitParam_p->pDevName));
    }

    if (pInitParam_p->pHwVersion != NULL)
    {
        // write Hardware version (0x1009)
        obdu_writeEntry(0x1009,
                        0,
                        pInitParam_p->pHwVersion,
                        (tObdSize)strlen(pInitParam_p->pHwVersion));
    }

    if (pInitParam_p->pSwVersion != NULL)
    {
        // write Software version (0x100A)
        obdu_writeEntry(0x100A,
                        0,
                        pInitParam_p->pSwVersion,
                        (tObdSize)strlen(pInitParam_p->pSwVersion));
    }

#if defined(CONFIG_INCLUDE_VETH)
    // write NMT_HostName_VSTR (0x1F9A)
    obdu_writeEntry(0x1F9A,
                    0,
                    &pInitParam_p->sHostname[0],
                    sizeof(pInitParam_p->sHostname));

    // write NWL_IpAddrTable_Xh_REC.Addr_IPAD (0x1E40/2)
    obdu_writeEntry(0x1E40,
                    2,
                    &pInitParam_p->ipAddress,
                    sizeof(pInitParam_p->ipAddress));

    // write NWL_IpAddrTable_Xh_REC.NetMask_IPAD (0x1E40/3)
    obdu_writeEntry(0x1E40,
                    3,
                    &pInitParam_p->subnetMask,
                    sizeof(pInitParam_p->subnetMask));

    // write NWL_IpAddrTable_Xh_REC.DefaultGateway_IPAD (0x1E40/5)
    obdu_writeEntry(0x1E40,
                    5,
                    &pInitParam_p->defaultGateway,
                    sizeof(pInitParam_p->defaultGateway));
#endif

    // ignore return code
    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  OD callback function

The function implements the OD callback function. It contains basic actions for
system objects and forwards the access to the user via a user event.

\param[in,out]  pParam_p            OD callback parameter.
\param[in]      fUserEvent_p        Flag indicating whether a user event shall be generated.

\return The function returns a tOplkError error code.

\ingroup module_ctrlu
*/
//------------------------------------------------------------------------------
static tOplkError cbObdAccess(tObdCbParam* pParam_p, BOOL fUserEvent_p)
{
    tOplkError          ret = kErrorOk;
#if (API_OBD_FORWARD_EVENT != FALSE)
    tOplkApiEventArg    obdCbEventArg;
#endif

    // Check parameter validity
    ASSERT(pParam_p != NULL);

#if (API_OBD_FORWARD_EVENT != FALSE)
    if (fUserEvent_p)
    {
        // call user callback
        obdCbEventArg.obdCbParam = *pParam_p;
        ret = ctrlu_callUserEventCallback(kOplkApiEventObdAccess, &obdCbEventArg);
        if (ret != kErrorOk)
        {   // do not do any further processing on this object
            if (ret == kErrorReject)
                ret = kErrorOk;
            return ret;
        }
    }
#endif

    switch (pParam_p->index)
    {
        case 0x1C14:    // DLL_LossOfFrameTolerance_U32
            ret = handleObdLossOfFrameTolerance(pParam_p);
            break;

        case 0x1020:    // CFM_VerifyConfiguration_REC
            ret = handleObdVerifyConf(pParam_p);
            break;

        case 0x1F9E:    // NMT_ResetCmd_U8
            ret = handleObdResetCmd(pParam_p);
            break;

#if defined(CONFIG_INCLUDE_IP)
        case 0x1E40:    // NWL_IpAddrTable_0h_REC
            ret = handleObdIpAddrTable(pParam_p);
            break;
#endif

#if defined(CONFIG_INCLUDE_NMT_MN)
        case 0x1C00:    // DLL_MNCRCError_REC
        case 0x1C02:    // DLL_MNCycTimeExceed_REC
#endif
        case 0x1C0B:    // DLL_CNLossSoC_REC
        case 0x1C0D:    // DLL_CNLossPReq_REC
        case 0x1C0F:    // DLL_CNCRCError_REC
            ret = errhndu_cbObdAccess(pParam_p);
            break;

#if defined(CONFIG_INCLUDE_NMT_MN)
        case 0x1C07:    // DLL_MNCNLossPResCumCnt_AU32
        case 0x1C08:    // DLL_MNCNLossPResThrCnt_AU32
        case 0x1C09:    // DLL_MNCNLossPResThreshold_AU32
            ret = errhndu_mnCnLossPresCbObdAccess(pParam_p);
            break;

        case 0x1F9F:    // NMT_RequestCmd_REC
            ret = handleObdRequestCmd(pParam_p);
            break;
#endif

#if defined(CONFIG_INCLUDE_CFM)
        case 0x1F22:    // CFM_ConciseDcfList_ADOM
            ret = cfmu_cbObdAccess(pParam_p);
            break;
#endif

#if (CONFIG_OBD_USE_STORE_RESTORE != FALSE)
        case 0x1010:    // NMT_StoreParam_REC
            // Call back for Store action
            ret = storeOdPart(pParam_p);
            break;

        case 0x1011:    // NMT_RestoreDefParam_REC
            // Call back for Restore action
            ret = restoreOdPart(pParam_p);
            break;
#endif
        default:
#if defined(CONFIG_INCLUDE_PDO)
            // PDO mapping objects
            if (((pParam_p->index >= 0x1400) && (pParam_p->index <= 0x14FF)) ||
                ((pParam_p->index >= 0x1600) && (pParam_p->index <= 0x16FF)) ||
                ((pParam_p->index >= 0x1800) && (pParam_p->index <= 0x18FF)) ||
                ((pParam_p->index >= 0x1A00) && (pParam_p->index <= 0x1AFF)))
                ret = pdou_cbObdAccess(pParam_p);
#endif
            break;
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Handle an access to the LossOfFrameTolerance object

This function handles an access to the LossOfFrameTolerance object in the object
dictionary.

\param[in]      pParam_p            OBD callback parameter.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError handleObdLossOfFrameTolerance(const tObdCbParam* pParam_p)
{
    tOplkError  ret = kErrorOk;

    if (pParam_p->obdEvent == kObdEvPostWrite)
    {
        // Update DLL configuration
        // NOTE: This is not fully correct, since it triggers a re-read of all
        //       (i.e. including the "valid on reset") objects!
        ret = updateDllConfig(&ctrlInstance_l.initParam, FALSE);
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Handle an access to the VerifyConfiguration object

This function handles an access to the VerifyConfiguration object in the object
dictionary.

\param[in]      pParam_p            OBD callback parameter.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError handleObdVerifyConf(const tObdCbParam* pParam_p)
{
    tOplkError  ret = kErrorOk;

    // Check if a value unequal to 0 has been written to subindex 3
    // (CFM_VerifyConfiguration_REC.ConfId_U32)
    if ((pParam_p->obdEvent == kObdEvPostWrite) &&
        (pParam_p->subIndex == 3) &&
        (*((const UINT32*)pParam_p->pArg) != 0))
    {
        UINT32  verifyConfInvalid = 0;

        // Set CFM_VerifyConfiguration_REC.VerifyConfInvalid_U32 to 0
        obdu_writeEntry(0x1020, 4, &verifyConfInvalid, 4);
        // ignore any error because this object is optional
        ret = kErrorOk;
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Handle an access to the NMT ResetCommand object

This function handles an access to the NMT ResetCommand object in the object
dictionary.

\param[in,out]  pParam_p            OBD callback parameter.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError handleObdResetCmd(tObdCbParam* pParam_p)
{
    tOplkError  ret = kErrorOk;
    tNmtCommand nmtCommand;

    switch (pParam_p->obdEvent)
    {
        case kObdEvPreWrite:
            // Check if a valid command is being passed
            nmtCommand = (tNmtCommand)(*((const UINT8*)pParam_p->pArg));
            switch (nmtCommand)
            {
                case kNmtCmdResetNode:
                case kNmtCmdResetCommunication:
                case kNmtCmdResetConfiguration:
                case kNmtCmdSwReset:
                case kNmtCmdInvalidService:
                    // Valid command identifier found
                    break;

                default:
                    // Invalid command identifier passed
                    pParam_p->abortCode = SDO_AC_VALUE_RANGE_EXCEEDED;
                    ret = kErrorObdAccessViolation;
                    break;
            }
            break;

        case kObdEvPostWrite:
            // Execute the NMT command
            nmtCommand = (tNmtCommand)(*((const UINT8*)pParam_p->pArg));
            switch (nmtCommand)
            {
                case kNmtCmdResetNode:
                    ret = nmtu_postNmtEvent(kNmtEventResetNode);
                    break;

                case kNmtCmdResetCommunication:
                    ret = nmtu_postNmtEvent(kNmtEventResetCom);
                    break;

                case kNmtCmdResetConfiguration:
                    ret = nmtu_postNmtEvent(kNmtEventResetConfig);
                    break;

                case kNmtCmdSwReset:
                    ret = nmtu_postNmtEvent(kNmtEventSwReset);
                    break;

                case kNmtCmdInvalidService:
                    break;

                default:
                    pParam_p->abortCode = SDO_AC_VALUE_RANGE_EXCEEDED;
                    ret = kErrorObdAccessViolation;
                    break;
            }
            break;
    }

    return ret;
}

#if defined(CONFIG_INCLUDE_IP)

//------------------------------------------------------------------------------
/**
\brief  Handle an access to the IpAddrTable object

This function handles an access to the IpAddrTable in the object dictionary.

\param[in]      pParam_p            OBD callback parameter.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError handleObdIpAddrTable(const tObdCbParam* pParam_p)
{
    tOplkError  ret = kErrorOk;

    // Check if a value has been written to subindex 5
    // (NWL_IpAddrTable_Xh_REC.DefaultGateway_IPAD)
    if ((pParam_p->obdEvent == kObdEvPostWrite) && (pParam_p->subIndex == 5))
    {
        tOplkApiEventArg    vethEventArg;

        // Copy the new IP gateway into the event argument
        vethEventArg.defaultGwChange.defaultGateway = *((const UINT32*)pParam_p->pArg);

        // Inform the user about the IP gateway change
        ret = ctrlu_callUserEventCallback(kOplkApiEventDefaultGwChange, &vethEventArg);
        if (ret == kErrorReject)
            ret = kErrorOk; // Ignore reject
    }

    return ret;
}

#endif


#if defined(CONFIG_INCLUDE_NMT_MN)
//------------------------------------------------------------------------------
/**
\brief  Callback function for node event

The function implements the callback function for node events.

\param[in]      nodeId_p            Node ID of the CN.
\param[in]      nodeEvent_p         Event from the specified CN.
\param[in]      nmtState_p          Current NMT state of the CN.
\param[in]      errorCode_p         Contains the error code for the node event kNmtNodeEventError
\param[in]      fMandatory_p        Flag determines if the CN is mandatory.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError cbNodeEvent(UINT nodeId_p,
                              tNmtNodeEvent nodeEvent_p,
                              tNmtState nmtState_p,
                              UINT16 errorCode_p,
                              BOOL fMandatory_p)
{
    tOplkError          ret = kErrorOk;
    tOplkApiEventArg    eventArg;

    // call user callback
    eventArg.nodeEvent.nodeId = nodeId_p;
    eventArg.nodeEvent.nodeEvent = nodeEvent_p;
    eventArg.nodeEvent.nmtState = nmtState_p;
    eventArg.nodeEvent.errorCode = errorCode_p;
    eventArg.nodeEvent.fMandatory = fMandatory_p;

    ret = ctrlu_callUserEventCallback(kOplkApiEventNode, &eventArg);
    if (((nodeEvent_p == kNmtNodeEventCheckConf) ||
        (nodeEvent_p == kNmtNodeEventUpdateConf) ||
        (nodeEvent_p == kNmtNodeEventUpdateSw)) &&
        (ret != kErrorOk))
        return ret;

#if defined(CONFIG_INCLUDE_CFM)
    ret = cfmu_processNodeEvent(nodeId_p, nodeEvent_p, nmtState_p);
#endif

    return ret;
}
#endif

//------------------------------------------------------------------------------
/**
\brief  Callback function for node event

The function implements the callback function for node events.

\param[in]      bootEvent_p         Event from the boot-up process.
\param[in]      nmtState_p          Current local NMT state.
\param[in]      errorCode_p         Contains the error code for the node event kNmtBootEventError

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError cbBootEvent(tNmtBootEvent bootEvent_p,
                              tNmtState nmtState_p,
                              UINT16 errorCode_p)
{
    tOplkError          ret = kErrorOk;
    tOplkApiEventArg    eventArg;

    // call user callback
    eventArg.bootEvent.bootEvent = bootEvent_p;
    eventArg.bootEvent.nmtState = nmtState_p;
    eventArg.bootEvent.errorCode = errorCode_p;

    ret = ctrlu_callUserEventCallback(kOplkApiEventBoot, &eventArg);

    return ret;
}

#if defined(CONFIG_INCLUDE_CFM)
//------------------------------------------------------------------------------
/**
\brief  Callback function for CFM progress events

The function implements the callback function for CFM progress events.

\param[in]      pEventCnProgress_p  Pointer to structure with additional
                                    information.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError cbCfmEventCnProgress(const tCfmEventCnProgress* pEventCnProgress_p)
{
    tOplkError          ret = kErrorOk;
    tOplkApiEventArg    eventArg;

    eventArg.cfmProgress = *pEventCnProgress_p;
    ret = ctrlu_callUserEventCallback(kOplkApiEventCfmProgress, &eventArg);

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Callback function for CFM result events

The function implements the callback function for CFM result events.

\param[in]      nodeId_p            Node ID of CN.
\param[in]      nodeCommand_p       NMT command which shall be executed.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError cbCfmEventCnResult(UINT nodeId_p,
                                     tNmtNodeCommand nodeCommand_p)
{
    tOplkError          ret;
    tOplkApiEventArg    eventArg;

    eventArg.cfmResult.nodeId = nodeId_p;
    eventArg.cfmResult.nodeCommand = nodeCommand_p;
    ret = ctrlu_callUserEventCallback(kOplkApiEventCfmResult, &eventArg);
    if (ret != kErrorOk)
    {
        if (ret == kErrorReject)
            ret = kErrorOk;

        return ret;
    }

    ret = nmtmnu_triggerStateChange(nodeId_p, nodeCommand_p);

    return ret;
}
#endif

//------------------------------------------------------------------------------
/**
\brief  Callback function for CN to check events

The function posts boot event directly to API layer.

\param[in]      nmtEvent_p          NMT event to check.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError cbCnCheckEvent(tNmtEvent nmtEvent_p)
{
    tOplkError  ret = kErrorOk;
    tNmtState   nmtState;

    switch (nmtEvent_p)
    {
        case kNmtEventEnableReadyToOperate:
            nmtState = nmtu_getNmtState();
            // inform application
            ret = cbBootEvent(kNmtBootEventEnableReadyToOp, nmtState, E_NO_ERROR);
            break;

        default:
            break;
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Callback function for user specific object access

The function posts accesses to non-existing objects in the default OD directly
to the API layer.

\param[in,out]  pObdAlConHdl_p      Pointer to OD abstraction layer handle.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError cbEventUserObdAccess(tObdAlConHdl* pObdAlConHdl_p)
{
    tOplkError          ret = kErrorOk;
    tOplkApiEventArg    eventArg;

    eventArg.userObdAccess.pUserObdAccHdl = pObdAlConHdl_p;

    ret = ctrlu_callUserEventCallback(kOplkApiEventUserObdAccess, &eventArg);

    return ret;
}

#if defined(CONFIG_INCLUDE_PDO)
//------------------------------------------------------------------------------
/**
\brief  Callback function for PDO change events

The function posts PDO change events directly to API layer.

\param[in]      pEventPdoChange_p   Pointer to PDO change event.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError cbEventPdoChange(const tPdoEventPdoChange* pEventPdoChange_p)
{
    tOplkError          ret = kErrorOk;
    tOplkApiEventArg    eventArg;

    eventArg.pdoChange.fActivated = pEventPdoChange_p->fActivated;
    eventArg.pdoChange.fTx = pEventPdoChange_p->fTx;
    eventArg.pdoChange.nodeId = pEventPdoChange_p->nodeId;
    eventArg.pdoChange.mappParamIndex = pEventPdoChange_p->mappParamIndex;
    eventArg.pdoChange.mappObjectCount = pEventPdoChange_p->mappObjectCount;

    ret = ctrlu_callUserEventCallback(kOplkApiEventPdoChange, &eventArg);

    return ret;
}
#endif

#if defined(CONFIG_INCLUDE_NMT_MN)
//------------------------------------------------------------------------------
/**
\brief  Link domain objects to variables

The function links domain objects to variables. The objects to be mapped are
specified by a list of tLinkObjectRequest entries.

\param[in]      pLinkRequest_p      Pointer to link request entry table.
\param[in]      requestCnt_p        Number of link requests.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError linkDomainObjects(const tLinkObjectRequest* pLinkRequest_p,
                                    size_t requestCnt_p)
{
    tOplkError  ret = kErrorOk;
    tObdSize    entrySize;
    UINT        varEntries;
    size_t      cnt;

    for (cnt = 0; cnt < requestCnt_p; cnt++, pLinkRequest_p++)
    {
        entrySize = pLinkRequest_p->entrySize;
        varEntries = pLinkRequest_p->varEntries;

        ret = oplk_linkObject(pLinkRequest_p->index,
                              pLinkRequest_p->pVar,
                              &varEntries,
                              &entrySize,
                              pLinkRequest_p->firstSubindex);
        if (ret != kErrorOk)
            break;
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Handle an access to the NMT RequestCommand object

This function handles an access to the NMT RequestCommand object in the object
dictionary.

\param[in,out]  pParam_p            OBD callback parameter.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError handleObdRequestCmd(tObdCbParam* pParam_p)
{
    tOplkError  ret = kErrorOk;

    // Receive the data size of a domain object
    if (pParam_p->obdEvent == kObdEvWrStringDomain)
    {
        tObdVStringDomain* pMemVStringDomain = (tObdVStringDomain*)pParam_p->pArg;

        ASSERT(pMemVStringDomain != NULL);

        nmtCmdDataSize_l = pMemVStringDomain->downloadSize;
    }

    // Check if a value unequal to 0 has been written to subindex 1
    // (NMT_RequestCmd_REC.Release_BOOL)
    if ((pParam_p->obdEvent == kObdEvPostWrite) &&
        (pParam_p->subIndex == 1) &&
        (*((const UINT8*)pParam_p->pArg) != 0))
    {
        UINT8       cmdId;
        UINT8       cmdTarget;
        tObdSize    obdSize;
        tNmtState   nmtState;

        // Read value of subindex 2 (NMT_RequestCmd_REC.CmdID_U8)
        obdSize = sizeof(cmdId);
        ret = obdu_readEntry(0x1F9F, 2, &cmdId, &obdSize);
        if (ret != kErrorOk)
        {
            pParam_p->abortCode = SDO_AC_GENERAL_ERROR;
            return ret;
        }

        // Read value of subindex 3 (NMT_RequestCmd_REC.CmdTarget_U8)
        obdSize = sizeof(cmdTarget);
        ret = obdu_readEntry(0x1F9F, 3, &cmdTarget, &obdSize);
        if (ret != kErrorOk)
        {
            pParam_p->abortCode = SDO_AC_GENERAL_ERROR;
            return ret;
        }

        // Subindex 4 (NMT_RequestCmd_REC.CmdData_DOM) is directly linked
        // to the static variable aCmdData_l

        // Check whether the command can be issued directly (we are MN)
        // or needs to be forwarded (we are CN)
        nmtState = nmtu_getNmtState();

        if (NMT_IF_CN_OR_RMN(nmtState))
        {   // Local node is CN
            // Forward the command to the MN
            // (this is a manufacturer specific feature)
            ret = nmtcnu_sendNmtRequestEx(cmdTarget, (tNmtCommand)cmdId,
                                          aCmdData_l, nmtCmdDataSize_l);
        }
        else
        {   // Local node is MN
            // Directly execute the requested NMT command
            ret = nmtmnu_requestNmtCommand(cmdTarget,
                                           (tNmtCommand)cmdId,
                                           aCmdData_l,
                                           sizeof(aCmdData_l));
        }

        // Return an error via SDO, if problem has occurred
        if (ret != kErrorOk)
            pParam_p->abortCode = SDO_AC_GENERAL_ERROR;

        // Reset the release flag (subindex 1)
        *((UINT8*)pParam_p->pArg) = 0;
    }

    return ret;
}

#endif

//------------------------------------------------------------------------------
/**
\brief  Get required kernel features

The function returns the features which are required from the kernel stack.
They will be set depending on the compilation guarded by feature macros
of the user stack.

\return Returns the required kernel features
\retval Returns a UINT32 variable with the kernel feature flags.
*/
//------------------------------------------------------------------------------
UINT32 getRequiredKernelFeatures(void)
{
    UINT32  requiredKernelFeatures = 0;

#if defined(CONFIG_INCLUDE_NMT_MN)
    // We do have NMT functionality compiled in and therefore need an MN
    // kernel stack
    requiredKernelFeatures |= OPLK_KERNEL_MN;
#endif

#if defined(CONFIG_INCLUDE_PDO)
    // We contain the PDO module for isochronous transfers and therefore need
    // a kernel module which can handle isochronous transfers.
    requiredKernelFeatures |= OPLK_KERNEL_ISOCHR;
#endif

#if defined(CONFIG_INCLUDE_PRES_FORWARD)
    // We contain the PRES forwarding module (used for diagnosis) and therefore
    // need a kernel with this feature.
    requiredKernelFeatures |= OPLK_KERNEL_PRES_FORWARD;
#endif

#if defined(CONFIG_INCLUDE_VETH)
    // We contain the virtual Ethernet module and therefore need a kernel
    // which supports virtual Ethernet.
    requiredKernelFeatures |= OPLK_KERNEL_VETH;
#endif

#if defined(CONFIG_INCLUDE_NMT_RMN)
    // We contain the code for the redundancy MN (RMN)
    requiredKernelFeatures |= OPLK_KERNEL_RMN;
#endif

#if (CONFIG_DLL_PRES_CHAINING_CN != FALSE)
    requiredKernelFeatures |= OPLK_KERNEL_PRES_CHAINING_CN;
#endif

#if defined(CONFIG_INCLUDE_SOC_TIME_FORWARD)
    requiredKernelFeatures |= OPLK_KERNEL_SOC_TIME_FORWARD;
#endif

    return requiredKernelFeatures;
}

#if (CONFIG_OBD_USE_STORE_RESTORE != FALSE)
//----------------------------------------------------------------------------
/**
\brief  Restore OD part archive

This is the callback function called when there is an access to object 0x1010
Writing will only be done if following conditions are met:
    - The signature 'save' is correct.
    - The selected part of OD supports the writing to non-volatile memory
      by command. If the device does not support saving parameters then
      abort transfer with abort code 0x08000020. If device supports only
      autonomously writing then abort transfer with abort code 0x08000021.
    - The memory device is ready for writing or the device is in the right
      state for writing. If this condition not fulfilled abort transfer
      (0x08000022).

\param[in,out]  pParam_p            OBD callback parameter.

\return The function returns a tOplkError error code.
*/
//----------------------------------------------------------------------------
static tOplkError storeOdPart(tObdCbParam* pParam_p)
{
    tOplkError  ret = kErrorOk;
    tObdPart    odPart = kObdPartNo;
    UINT32      devCap = 0;

    if ((pParam_p->subIndex == 0) ||
        ((pParam_p->obdEvent != kObdEvPreWrite) &&
         (pParam_p->obdEvent != kObdEvPostRead)))
    {
        goto Exit;
    }

    ret = obdconf_getTargetCapabilities(pParam_p->index,
                                        pParam_p->subIndex,
                                        &odPart,
                                        &devCap);
    if (ret != kErrorOk)
    {
        pParam_p->abortCode = SDO_AC_DATA_NOT_TRANSF_DUE_DEVICE_STATE;
        goto Exit;
    }

    // Function is called before writing to OD
    if (pParam_p->obdEvent == kObdEvPreWrite)
    {
        // Check if signature "save" will be written to object 0x1010
        if (*((const UINT32*)pParam_p->pArg) != OBD_SIGNATURE_STORE)
        {
            // Abort SDO because wrong signature
            pParam_p->abortCode = SDO_AC_DATA_NOT_TRANSF_OR_STORED;
            ret = kErrorWrongSignature;
            goto Exit;
        }

        // Does device support saving by command?
        if ((devCap & OBD_STORE_ON_COMMAND) == 0)
        {
            // Device does not support saving parameters or not by command
            if ((devCap & OBD_STORE_AUTONOMOUSLY) != 0)
            {
                // Device saves parameters autonomously.
                pParam_p->abortCode = SDO_AC_DATA_NOT_TRANSF_DUE_LOCAL_CONTROL;
            }
            else
            {
                // Device does not support saving parameters.
                pParam_p->abortCode = SDO_AC_DATA_NOT_TRANSF_OR_STORED;
            }

            ret = kErrorObdStoreInvalidState;
            goto Exit;
        }

        // Read all storable objects of selected OD part and store the
        // current object values to non-volatile memory
        ret = obdu_accessOdPart(odPart, kObdDirStore);
        if (ret != kErrorOk)
        {
            // Abort SDO because access failed
            pParam_p->abortCode = SDO_AC_ACCESS_FAILED_DUE_HW_ERROR;
            goto Exit;
        }
    }
    else
    {
        // Function is called after reading from OD
        if (pParam_p->obdEvent == kObdEvPostRead)
        {
            ami_setUint32Le((void*)pParam_p->pArg, *(const UINT32*)&devCap);
        }
    }

Exit:
    return ret;
}

//----------------------------------------------------------------------------
/**
\brief  Restore OD part archive

This is the callback function called when there is an access to object 0x1011.
Reseting default parameters will only be done if following conditions are met:
    - The signature 'load' is correct.
    - The selected part of OD supports the restoring parameters by command.
      If the device does not support restoring parameters then abort transfer
      with abort code 0x08000020.
    - The memory device is ready for restoring or the device is in the right
      state for restoring. If this condition not fulfilled abort transfer
      (0x08000022).
    - Restoring is done by declaring the stored parameters as invalid.
      The function does not load the default parameters. This is only done
      by command reset node or reset communication or power-on.

\param[in,out]  pParam_p            OBD callback parameter.

\return The function returns a tOplkError error code.
*/
//----------------------------------------------------------------------------
static tOplkError restoreOdPart(tObdCbParam* pParam_p)
{
    tOplkError  ret = kErrorOk;
    tObdPart    odPart = kObdPartNo;
    UINT32      devCap = 0;

    if ((pParam_p->subIndex == 0) ||
        ((pParam_p->obdEvent != kObdEvPreWrite) &&
         (pParam_p->obdEvent != kObdEvPostRead)))
    {
        goto Exit;
    }

    ret = obdconf_getTargetCapabilities(pParam_p->index,
                                        pParam_p->subIndex,
                                        &odPart,
                                        &devCap);
    if (ret != kErrorOk)
    {
        pParam_p->abortCode = SDO_AC_DATA_NOT_TRANSF_DUE_DEVICE_STATE;
        goto Exit;
    }

    // Was this function called before writing to OD
    if (pParam_p->obdEvent == kObdEvPreWrite)
    {
        // Check if signature "load" will be written to object 0x1011 subindex X
        if (*((const UINT32*)pParam_p->pArg) != OBD_SIGNATURE_RESTORE)
        {
            // Abort SDO
            pParam_p->abortCode = SDO_AC_DATA_NOT_TRANSF_OR_STORED;
            ret = kErrorWrongSignature;
            goto Exit;
        }

        // Does device support saving by command?
        if ((devCap & OBD_STORE_ON_COMMAND) == 0)
        {
            // Device does not support saving parameters.
            pParam_p->abortCode = SDO_AC_DATA_NOT_TRANSF_OR_STORED;
            ret = kErrorObdStoreInvalidState;
            goto Exit;
        }

        ret = obdu_accessOdPart(odPart, kObdDirRestore);
        if (ret != kErrorOk)
        {
            // abort SDO
            pParam_p->abortCode = SDO_AC_ACCESS_FAILED_DUE_HW_ERROR;
            goto Exit;
        }
    }
    else
    {
        // Function is called after reading from OD
        if (pParam_p->obdEvent == kObdEvPostRead)
        {
            ami_setUint32Le((void*)pParam_p->pArg, *(const UINT32*)&devCap);
        }
    }

Exit:
    return ret;
}

//----------------------------------------------------------------------------
/**
\brief  Callback for OD store load object

This is the callback function, called by OBD module, that notifies of
commands STORE or LOAD.
The function is called with the selected OD part. For each part the function
creates an archive and saves the parameters. Following command sequence is used:
              1. kEplObdCommOpenWrite(Read)  --> create archive (set last archive invalid)
              2. kEplObdCommWrite(Read)Obj   --> write data to archive
              3. kEplObdCommCloseWrite(Read) --> close archive (set archive valid)

\param[in]      pCbStoreParam_p     Callback instance parameters

\return The function returns a tOplkError error code.
*/
//----------------------------------------------------------------------------
static tOplkError cbStoreLoadObject(const tObdCbStoreParam* pCbStoreParam_p)
{
    tOplkError  ret = kErrorOk;
    tOplkError  archiveState = kErrorOk;
    tObdPart    odPart = pCbStoreParam_p->currentOdPart;    // only one bit is set!
    UINT32      signature = 0;

    // Which event is notified
    switch (pCbStoreParam_p->command)
    {
        case kObdCmdOpenWrite:
            // Create archive to write all objects of this OD part
#if (CONFIG_OBD_CALC_OD_SIGNATURE != FALSE)
            signature = (UINT32)obdu_getOdSignature(odPart);
#endif
            ret = obdconf_createPart(odPart, signature);
            break;

        case kObdCmdWriteObj:
            // Store value from pData_p (with size ObjSize_p) to memory medium
            ret = obdconf_storePart(odPart,
                                    pCbStoreParam_p->pData,
                                    pCbStoreParam_p->objSize);
            break;

        case kObdCmdCloseRead:
        case kObdCmdCloseWrite:
            ret = obdconf_closePart(odPart);
            break;

        case kObdCmdOpenRead:
            // Check signature for data valid on medium for this OD part
#if (CONFIG_OBD_CALC_OD_SIGNATURE != FALSE)
            signature = (UINT32)obdu_getOdSignature(odPart);
#endif
            archiveState = obdconf_getPartArchiveState(odPart, signature);
            if ((archiveState == kErrorOk) || (archiveState == kErrorObdStoreDataObsolete))
            {
                ret = obdconf_openReadPart(odPart);
                if (ret == kErrorOk)
                    ret = archiveState;
            }

            break;

        case kObdCmdReadObj:
            ret = obdconf_loadPart(odPart,
                                   pCbStoreParam_p->pData,
                                   pCbStoreParam_p->objSize);
            break;

        case kObdCmdClear:
            ret = obdconf_deletePart(odPart);
            break;

        default:
            ret = kErrorInvalidOperation;
            break;
    }

    return ret;
}

//----------------------------------------------------------------------------
/**
\brief  Initialize default OD part archives

The function checks if the specified OD archive part in non-volatile memory
is valid.

\return The function returns a tOplkError error code.
*/
//----------------------------------------------------------------------------
static tOplkError initDefaultOdPartArchive(void)
{
    tOplkError  ret = kErrorOk;
    tObdPart    curOdPart = kObdPartNo;
    tObdPart    nextOdPart = kObdPartNo;
    BOOL        fExit = FALSE;
#if (CONFIG_OBD_CALC_OD_SIGNATURE != FALSE)
    UINT32      signature = 0;
#endif

    while (fExit == FALSE)
    {
        switch (curOdPart)
        {
            case kObdPartNo:
                nextOdPart = kObdPartGen;
                break;

            case kObdPartGen:
            case kObdPartMan:
            case kObdPartDev:
#if (CONFIG_OBD_CALC_OD_SIGNATURE != FALSE)
            signature = (UINT32)obdu_getOdSignature(curOdPart);
#endif
                if (obdconf_getPartArchiveState(curOdPart, signature) == kErrorObdStoreHwError)
                {
                    // Create a part archive marked obsolete
                    ret = obdconf_createPart(curOdPart, (UINT32)~0);
                    if (ret != kErrorOk)
                    {
                        fExit = TRUE;
                        break;
                    }

                    ret = obdconf_closePart(curOdPart);
                    if (ret != kErrorOk)
                        fExit = TRUE;
                }

                nextOdPart = curOdPart << 1;
                break;

            case kObdPartUsr:
                nextOdPart = kObdPartApp;
                break;

            case kObdPartApp:
                nextOdPart = kObdPartAll;
                break;

            case kObdPartAll:
            default:
                fExit = TRUE;
                break;
        }

        // switch to the next OD part
        curOdPart = nextOdPart;
    }

    return ret;
}
#endif

/// \}
