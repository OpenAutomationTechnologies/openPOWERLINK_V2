/**
********************************************************************************
\file   ctrlu.c

\brief  User stack control module

This file contains the implementation of the user stack control module.

\ingroup module_ctrlu
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2015, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
Copyright (c) 2015, SYSTEC electronic GmbH
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
#include <user/ctrlu.h>
#include <user/ctrlucal.h>
#include <user/eventu.h>
#include <user/eventucal.h>
#include <user/timeru.h>
#include <user/errhndu.h>
#include <user/dllucal.h>
#include <user/nmtu.h>
#include <user/nmtcnu.h>
#include <common/target.h>
#include <oplk/obd.h>
#include <oplk/dll.h>

#if defined(CONFIG_INCLUDE_NMT_MN)
#include <user/nmtmnu.h>
#include <user/identu.h>
#include <user/statusu.h>
#include <user/syncu.h>
#endif

#if defined(CONFIG_INCLUDE_PDO)
#include <user/pdou.h>
#endif

#if defined(CONFIG_INCLUDE_LEDU)
#include <user/ledu.h>
#endif

#if defined(CONFIG_INCLUDE_SDOS) || defined(CONFIG_INCLUDE_SDOC)
#include <user/sdoseq.h>
#include <user/sdocom.h>
#endif

#if defined(CONFIG_INCLUDE_CFM)
#include <user/cfmu.h>
#endif

#if (CONFIG_OBD_USE_LOAD_CONCISEDCF != FALSE)
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
    BOOL                fInitialized;           ///< Flag determines if stack is initialized/ready
} tCtrluInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tCtrluInstance       ctrlInstance_l;

#if defined(CONFIG_INCLUDE_NMT_MN)
UINT8    aCmdData[C_MAX_NMT_CMD_DATA_SIZE];     // Extended NMT request command data

// List of objects that need to get linked
tLinkObjectRequest    linkObjectRequestsMn[]  =
{//     Index       Variable        Count   Object size             SubIndex
    {   0x1F9F,     aCmdData,       1,      sizeof(aCmdData),       4   },
};
#endif

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static tOplkError initNmtu(tOplkApiInitParam* pInitParam_p);
static tOplkError initObd(tOplkApiInitParam* pInitParam_p);
static tOplkError updateDllConfig(tOplkApiInitParam* pInitParam_p, BOOL fUpdateIdentity_p);
static tOplkError updateObd(tOplkApiInitParam* pInitParam_p);
static tOplkError processUserEvent(tEvent* pEvent_p);
static tOplkError cbCnCheckEvent(tNmtEvent NmtEvent_p);
static tOplkError cbNmtStateChange(tEventNmtStateChange nmtStateChange_p);

#if defined(CONFIG_INCLUDE_SDOS) || defined(CONFIG_INCLUDE_SDOC)
static tOplkError updateSdoConfig(void);
#endif

#if defined(CONFIG_INCLUDE_PDO)
static tOplkError cbEventPdoChange(tPdoEventPdoChange* pEventPdoChange_p);
#endif

#if defined(CONFIG_INCLUDE_NMT_MN)
static tOplkError cbNodeEvent(UINT nodeId_p, tNmtNodeEvent nodeEvent_p,
                              tNmtState nmtState_p, UINT16 errorCode_p,
                              BOOL fMandatory_p);
static tOplkError linkDomainObjects(tLinkObjectRequest* pLinkRequest_p,
                                    size_t requestCnt_p);
#endif

static tOplkError cbBootEvent(tNmtBootEvent BootEvent_p, tNmtState NmtState_p,
                              UINT16 errorCode_p);

#if defined(CONFIG_INCLUDE_LEDU)
static tOplkError cbLedStateChange(tLedType LedType_p, BOOL fOn_p);
#endif

#if defined(CONFIG_INCLUDE_CFM)
static tOplkError cbCfmEventCnProgress(tCfmEventCnProgress* pEventCnProgress_p);
static tOplkError cbCfmEventCnResult(unsigned int uiNodeId_p, tNmtNodeCommand NodeCommand_p);
#endif
UINT32 getRequiredKernelFeatures(void);

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
    tOplkError          ret;

    DEBUG_LVL_CTRL_TRACE("Initialize ctrl module ...\n");

    ctrlInstance_l.lastHeartbeat = 0;

    if ((ret = ctrlucal_init()) != kErrorOk)
    {
        DEBUG_LVL_ERROR_TRACE("Could not initialize ctrlucal\n");
        goto Exit;
    }

    if ((ret = ctrlucal_checkKernelStack()) != kErrorOk)
    {
        ctrlucal_exit();
        goto Exit;
    }

    ctrlInstance_l.requiredKernelFeatures = getRequiredKernelFeatures();
    if ((ret = ctrlu_getKernelInfo(&ctrlInstance_l.kernelInfo)) != kErrorOk)
    {
        ctrlucal_exit();
        goto Exit;
    }

    if ((ctrlInstance_l.kernelInfo.featureFlags == ctrlInstance_l.requiredKernelFeatures) &&
        (ctrlInstance_l.kernelInfo.version == PLK_DEFINED_STACK_VERSION))
    {
        DEBUG_LVL_ALWAYS_TRACE("Kernel features: 0x%08x\n", ctrlInstance_l.kernelInfo.featureFlags);
        DEBUG_LVL_ALWAYS_TRACE("Kernel version: 0x%08x\n", ctrlInstance_l.kernelInfo.version);
        return kErrorOk;
    }
    else
    {
        DEBUG_LVL_ERROR_TRACE("Kernel feature/version mismatch:\n");
        DEBUG_LVL_ERROR_TRACE("  Version: Is:%08x - required:%08x\n",
                              ctrlInstance_l.kernelInfo.version, PLK_DEFINED_STACK_VERSION);
        DEBUG_LVL_ERROR_TRACE("  Features: Is:%08x - required:%08x\n",
                              ctrlInstance_l.kernelInfo.featureFlags, ctrlInstance_l.requiredKernelFeatures);
        ctrlucal_exit();
        return kErrorFeatureMismatch;
    }

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Cleanup user control module

The function cleans up the user control module.

\return The function returns a tOplkError error code.

\ingroup module_ctrlu
*/
//------------------------------------------------------------------------------
void ctrlu_exit(void)
{
    ctrlucal_exit();
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

\param  pInitParam_p            Pointer to the initialization parameters
                                provided by the application.

\return The function returns a tOplkError error code.

\ingroup module_ctrlu
*/
//------------------------------------------------------------------------------
tOplkError ctrlu_initStack(tOplkApiInitParam* pInitParam_p)
{
    tOplkError              ret = kErrorOk;
    tCtrlInitParam          ctrlParam;
    UINT16                  retVal;

    // reset instance structure
    OPLK_MEMSET(&ctrlInstance_l.initParam, 0, sizeof(tOplkApiInitParam));
    OPLK_MEMCPY(&ctrlInstance_l.initParam, pInitParam_p,
                min(sizeof(tOplkApiInitParam), (size_t)pInitParam_p->sizeOfInitParam));

    // check event callback function pointer
    if (ctrlInstance_l.initParam.pfnCbEvent == NULL)
    {   // application must always have an event callback function
        ret = kErrorApiInvalidParam;
        goto Exit;
    }

    if ((ret = initObd(&ctrlInstance_l.initParam)) != kErrorOk)
        goto Exit;

#if defined(CONFIG_INCLUDE_NMT_MN)
    ret = linkDomainObjects(linkObjectRequestsMn, tabentries(linkObjectRequestsMn));
#endif

    DEBUG_LVL_CTRL_TRACE("Initializing kernel modules ...\n");
    OPLK_MEMCPY(ctrlParam.aMacAddress, ctrlInstance_l.initParam.aMacAddress, 6);
    strncpy(ctrlParam.szEthDevName, ctrlInstance_l.initParam.hwParam.pDevName, 127);
    ctrlParam.ethDevNumber = ctrlInstance_l.initParam.hwParam.devNum;
    ctrlucal_storeInitParam(&ctrlParam);

    if ((ret = ctrlucal_executeCmd(kCtrlInitStack, &retVal)) != kErrorOk)
        goto Exit;

    if ((tOplkError)retVal != kErrorOk)
    {
        ret = (tOplkError)retVal;
        goto Exit;
    }

    /* Read back init param because current MAC address was copied by DLLK */
    ret = ctrlucal_readInitParam(&ctrlParam);
    if (ret != kErrorOk)
    {
        goto Exit;
    }

    OPLK_MEMCPY(ctrlInstance_l.initParam.aMacAddress, ctrlParam.aMacAddress, 6);

    DEBUG_LVL_CTRL_TRACE("Initialize Eventu module...\n");
    if ((ret = eventu_init(processUserEvent)) != kErrorOk)
        goto Exit;

    DEBUG_LVL_CTRL_TRACE("Initialize Timeru module...\n");
    if ((ret = timeru_init()) != kErrorOk)
        goto Exit;

    DEBUG_LVL_CTRL_TRACE("initialize error handler user module...\n");
    ret = errhndu_init();
    if (ret != kErrorOk)
    {
        goto Exit;
    }

    DEBUG_LVL_CTRL_TRACE("Initialize DlluCal module...\n");
    ret = dllucal_init();
    if (ret != kErrorOk)
    {
        goto Exit;
    }

#if defined(CONFIG_INCLUDE_PDO)
    DEBUG_LVL_CTRL_TRACE("Initialize Pdou module...\n");
    ret = pdou_init(ctrlInstance_l.initParam.pfnCbSync);
    if (ret != kErrorOk)
    {
        goto Exit;
    }

    ret = pdou_registerEventPdoChangeCb(cbEventPdoChange);
    if (ret != kErrorOk)
    {
        goto Exit;
    }
#endif

    if ((ret = initNmtu(&ctrlInstance_l.initParam)) != kErrorOk)
        goto Exit;

#if defined(CONFIG_INCLUDE_LEDU)
    ret = ledu_init(cbLedStateChange);
    if (ret != kErrorOk)
    {
        goto Exit;
    }
#endif

#if defined(CONFIG_INCLUDE_SDOS) || defined(CONFIG_INCLUDE_SDOC)
    // init sdo command layer
    DEBUG_LVL_CTRL_TRACE("Initialize SdoCom module...\n");
    ret = sdocom_init(pInitParam_p->sdoStackType);
    if (ret != kErrorOk)
    {
        goto Exit;
    }
#endif

#if defined (CONFIG_INCLUDE_CFM)
    DEBUG_LVL_CTRL_TRACE("Initialize Cfm module...\n");
    ret = cfmu_init(cbCfmEventCnProgress, cbCfmEventCnResult);
    if (ret != kErrorOk)
    {
        goto Exit;
    }
#endif

    // the application must start NMT state machine
    // via oplk_execNmtCommand(kNmtEventSwReset)
    // and thereby the whole POWERLINK stack

    ctrlInstance_l.fInitialized = TRUE;

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
    tOplkError      ret = kErrorOk;
    UINT16          retVal;

    ctrlInstance_l.fInitialized = FALSE;

#if defined(CONFIG_INCLUDE_CFM)
    ret = cfmu_exit();
    DEBUG_LVL_CTRL_TRACE("cfmu_exit():    0x%X\n", ret);
#endif

#if defined(CONFIG_INCLUDE_SDOS) || defined(CONFIG_INCLUDE_SDOC)
    ret = sdocom_exit();
    DEBUG_LVL_CTRL_TRACE("sdocom_exit():  0x%X\n", ret);
#endif

#if defined(CONFIG_INCLUDE_LEDU)
    ret = ledu_exit();
    DEBUG_LVL_CTRL_TRACE("ledu_exit():    0x%X\n", ret);
#endif

#if defined(CONFIG_INCLUDE_NMT_MN)
    ret = nmtmnu_exit();
    DEBUG_LVL_CTRL_TRACE("nmtmnu_exit():  0x%X\n", ret);

    ret = identu_exit();
    DEBUG_LVL_CTRL_TRACE("identu_exit():  0x%X\n", ret);

    ret = statusu_exit();
    DEBUG_LVL_CTRL_TRACE("statusu_exit():  0x%X\n", ret);

    ret = syncu_exit();
#endif

    ret = nmtcnu_exit();
    DEBUG_LVL_CTRL_TRACE("nmtcnu_exit():  0x%X\n", ret);

    ret = nmtu_exit();
    DEBUG_LVL_CTRL_TRACE("nmtu_exit():    0x%X\n", ret);

#if defined(CONFIG_INCLUDE_PDO)
    ret = pdou_exit();
    DEBUG_LVL_CTRL_TRACE("pdou_exit():    0x%X\n", ret);
#endif

    ret = dllucal_exit();
    DEBUG_LVL_CTRL_TRACE("dllucal_exit(): 0x%X\n", ret);

    ret = errhndu_exit();
    DEBUG_LVL_CTRL_TRACE("errhndu_exit():  0x%X\n", ret);

    ret = timeru_exit();
    DEBUG_LVL_CTRL_TRACE("timeru_exit():  0x%X\n", ret);

    ret = eventu_exit();
    DEBUG_LVL_CTRL_TRACE("eventu_exit():  0x%X\n", ret);

    /* shutdown kernel stack */
    ret = ctrlucal_executeCmd(kCtrlCleanupStack, &retVal);
    DEBUG_LVL_CTRL_TRACE("shoutdown kernel modules():  0x%X\n", ret);

#if (CONFIG_OBD_USE_LOAD_CONCISEDCF != FALSE)
    obdcdc_exit();
#endif

    ret = obd_exit();

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
    tOplkError ret = kErrorOk;

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
        DEBUG_LVL_CTRL_TRACE("heartbeat:%d ctrlInstance_l.lastHeartbeat:%d\n", heartbeat, ctrlInstance_l.lastHeartbeat);
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

\param  pKernelInfo_p       Pointer to store kernel information.

\return The function returns a tOplkError error code.

\ingroup module_ctrlu
*/
//------------------------------------------------------------------------------
tOplkError ctrlu_getKernelInfo(tCtrlKernelInfo* pKernelInfo_p)
{
    UINT16      retVal;

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

\param  eventType_p         Event type to send.
\param  pEventArg_p         Event argument to send.

\return The function returns a tOplkError error code.

\ingroup module_ctrlu
*/
//------------------------------------------------------------------------------
tOplkError ctrlu_callUserEventCallback(tOplkApiEventType eventType_p, tOplkApiEventArg* pEventArg_p)
{
    tOplkError          ret = kErrorOk;

    // If the stack is not initialized but we get events, we don't forward
    // them to the application!
    if (ctrlInstance_l.fInitialized)
    {
        ret = ctrlInstance_l.initParam.pfnCbEvent(eventType_p, pEventArg_p,
                                              ctrlInstance_l.initParam.pEventUserArg);
    }
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Default OD callback function

The function implements the standard OD callback function. It contains basic
actions for system objects.

\param      pParam_p        OBD callback parameter.

\return The function returns a tOplkError error code.

\ingroup module_ctrlu
*/
//------------------------------------------------------------------------------
tOplkError ctrlu_cbObdAccess(tObdCbParam MEM* pParam_p)
{
    tOplkError          ret = kErrorOk;

#if (API_OBD_FORWARD_EVENT != FALSE)
    tOplkApiEventArg     obdCbEventArg;

    // call user callback
    obdCbEventArg.obdCbParam = *pParam_p;
    ret = ctrlu_callUserEventCallback(kOplkApiEventObdAccess, &obdCbEventArg);
    if (ret != kErrorOk)
    {   // do not do any further processing on this object
        if (ret == kErrorReject)
            ret = kErrorOk;
        return ret;
    }
#endif

    switch (pParam_p->index)
    {
        //case 0x1006:    // NMT_CycleLen_U32 (valid on reset)
        case 0x1C14:    // DLL_LossOfFrameTolerance_U32
        //case 0x1F98:    // NMT_CycleTiming_REC (valid on reset)
            if (pParam_p->obdEvent == kObdEvPostWrite)
            {
                // update DLL configuration
                ret = updateDllConfig(&ctrlInstance_l.initParam, FALSE);
            }
            break;

        case 0x1020:    // CFM_VerifyConfiguration_REC.ConfId_U32 != 0
            if ((pParam_p->obdEvent == kObdEvPostWrite) &&
                (pParam_p->subIndex == 3) &&
                (*((UINT32*)pParam_p->pArg) != 0))
            {
                UINT32      verifyConfInvalid = 0;
                // set CFM_VerifyConfiguration_REC.VerifyConfInvalid_U32 to 0
                ret = obd_writeEntry(0x1020, 4, &verifyConfInvalid, 4);
                // ignore any error because this objekt is optional
                ret = kErrorOk;
            }
            break;

        case 0x1F9E:    // NMT_ResetCmd_U8
            if (pParam_p->obdEvent == kObdEvPreWrite)
            {
                UINT8    nmtCommand;

                nmtCommand = *((UINT8*)pParam_p->pArg);
                // check value range
                switch ((tNmtCommand)nmtCommand)
                {
                    case kNmtCmdResetNode:
                    case kNmtCmdResetCommunication:
                    case kNmtCmdResetConfiguration:
                    case kNmtCmdSwReset:
                    case kNmtCmdInvalidService:
                        // valid command identifier specified
                        break;

                    default:
                        pParam_p->abortCode = SDO_AC_VALUE_RANGE_EXCEEDED;
                        ret = kErrorObdAccessViolation;
                        break;
                }
            }
            else if (pParam_p->obdEvent == kObdEvPostWrite)
            {
                UINT8    nmtCommand;

                nmtCommand = *((UINT8*)pParam_p->pArg);
                // check value range
                switch ((tNmtCommand)nmtCommand)
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
            }
            break;

#if defined(CONFIG_INCLUDE_VETH)
        case 0x1E40:    // NWL_IpAddrTable_0h_REC
            if ((pParam_p->obdEvent == kObdEvPostWrite) && (pParam_p->subIndex == 5))
            {
                tOplkApiEventArg    vethEventArg;

                vethEventArg.defaultGwChange.defaultGateway = *((UINT32*)pParam_p->pArg);

                ret = ctrlu_callUserEventCallback(kOplkApiEventDefaultGwChange, &vethEventArg);
                if (ret == kErrorReject)
                    ret = kErrorOk; // Ignore reject
            }
            break;
#endif

#if defined(CONFIG_INCLUDE_NMT_MN)
        case 0x1F9F:    // NMT_RequestCmd_REC
            if ((pParam_p->obdEvent == kObdEvPostWrite) &&
                (pParam_p->subIndex == 1) &&
                (*((UINT8*)pParam_p->pArg) != 0))
            {
                UINT8       cmdId;
                UINT8       cmdTarget;
                tObdSize    obdSize;
                tNmtState   nmtState;

                obdSize = sizeof(UINT8);
                ret = obd_readEntry(0x1F9F, 2, &cmdId, &obdSize);
                if (ret != kErrorOk)
                {
                    pParam_p->abortCode = SDO_AC_GENERAL_ERROR;
                    break;
                }

                obdSize = sizeof (cmdTarget);
                ret = obd_readEntry(0x1F9F, 3, &cmdTarget, &obdSize);
                if (ret != kErrorOk)
                {
                    pParam_p->abortCode = SDO_AC_GENERAL_ERROR;
                    break;
                }

                nmtState = nmtu_getNmtState();

                if (NMT_IF_CN_OR_RMN(nmtState))
                {   // local node is CN
                    // forward the command to the MN
                    // d.k. this is a manufacturer specific feature
                    ret = nmtcnu_sendNmtRequestEx(cmdTarget, (tNmtCommand) cmdId,
                                                  aCmdData, sizeof(aCmdData));
                }
                else
                {   // local node is MN
                    // directly execute the requested NMT command
                    ret = nmtmnu_requestNmtCommand(cmdTarget, (tNmtCommand) cmdId,
                                                   aCmdData, sizeof(aCmdData));
                }
                if (ret != kErrorOk)
                {
                    pParam_p->abortCode = SDO_AC_GENERAL_ERROR;
                }

                // reset request flag
                *((UINT8*)pParam_p->pArg) = 0;
            }
            break;
#endif

        default:
            break;
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
UINT8* ctrlu_getEthMacAddr(void)
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

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief  initialize NMTU modules

The function initializes the NMTU modules.

\param  pInitParam_p        Pointer to init parameters.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError initNmtu(tOplkApiInitParam* pInitParam_p)
{
    tOplkError      ret = kErrorOk;

    // initialize NmtCnu module
    DEBUG_LVL_CTRL_TRACE("Initialize NMT_CN module...\n");
    ret = nmtcnu_init(pInitParam_p->nodeId);
    if (ret != kErrorOk)
        goto Exit;

    ret = nmtcnu_registerCheckEventCb(cbCnCheckEvent);
    if (ret != kErrorOk)
        goto Exit;

    // initialize Nmtu module
    DEBUG_LVL_CTRL_TRACE("Initialize NMTu module...\n");
    ret = nmtu_init();
    if (ret != kErrorOk)
        goto Exit;

    // register NMT event callback function
    ret = nmtu_registerStateChangeCb(cbNmtStateChange);
    if (ret != kErrorOk)
        goto Exit;

#if defined(CONFIG_INCLUDE_NMT_MN)
    // initialize NmtMnu module
    DEBUG_LVL_CTRL_TRACE("Initialize NMT_MN module...\n");
    ret = nmtmnu_init(cbNodeEvent, cbBootEvent);
    if (ret != kErrorOk)
        goto Exit;

    // initialize identu module
    DEBUG_LVL_CTRL_TRACE("Initialize Identu module...\n");
    ret = identu_init();
    if (ret != kErrorOk)
        goto Exit;

    // initialize Statusu module
    DEBUG_LVL_CTRL_TRACE("Initialize Statusu module...\n");
    ret = statusu_init();
    if (ret != kErrorOk)
        goto Exit;

    // initialize syncu module
    DEBUG_LVL_CTRL_TRACE("Initialize Syncu module...\n");
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

\param  pInitParam_p        Pointer to init parameters.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError initObd(tOplkApiInitParam* pInitParam_p)
{
    tOplkError          ret = kErrorOk;
    tObdInitParam       ObdInitParam;

    UNUSED_PARAMETER(pInitParam_p);

    DEBUG_LVL_CTRL_TRACE("Initialize OBD module...\n");
    ret = obd_initObd(&ObdInitParam);
    if (ret != kErrorOk)
        return ret;

    ret = obd_init(&ObdInitParam);
    if (ret != kErrorOk)
        return ret;

#if (CONFIG_OBD_USE_LOAD_CONCISEDCF != FALSE)
    ret = obdcdc_init();
#endif

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Callback function for NMT state change events

The function implements the callback function for node events.

\param  nmtStateChange_p        NMT state change event.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError cbNmtStateChange(tEventNmtStateChange nmtStateChange_p)
{
    tOplkError          ret = kErrorOk;
    BYTE                nmtState;
    tOplkApiEventArg    eventArg;

    // save NMT state in OD
    nmtState = (UINT8)nmtStateChange_p.newNmtState;
    ret = obd_writeEntry(0x1F8C, 0, &nmtState, 1);
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
        // standardised device profile area
        case kNmtGsResetApplication:
            // reset application part of OD
            ret = obd_accessOdPart(kObdPartApp, kObdDirLoad);
            if (ret != kErrorOk)
                return ret;
            break;

        // init of the communication profile area
        case kNmtGsResetCommunication:
            // reset communication part of OD
            ret = obd_accessOdPart(kObdPartGen, kObdDirLoad);
            if (ret != kErrorOk)
                return ret;

            // $$$ d.k.: update OD only if OD was not loaded from non-volatile memory
            ret = updateObd(&ctrlInstance_l.initParam);
            if (ret != kErrorOk)
                return ret;

#if (CONFIG_OBD_USE_LOAD_CONCISEDCF != FALSE)
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

#if defined(CONFIG_INCLUDE_SDOS) || defined(CONFIG_INCLUDE_SDOC)
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
            ret = obd_writeEntry(0x1F9E, 0, &nmtState, 1);
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
            DEBUG_LVL_CTRL_TRACE("cbNmtStateChange(): unhandled NMT state\n");
            break;
    }

#if defined(CONFIG_INCLUDE_LEDU)
    // forward event to Led module
    ret = ledu_cbNmtStateChange(nmtStateChange_p);
    if (ret != kErrorOk)
        return ret;
#endif

#if defined(CONFIG_INCLUDE_PDO)
    // forward event to Pdou module
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

\param  pEvent_p             Event to process.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError processUserEvent(tEvent* pEvent_p)
{
    tOplkError          ret;
    tEventError*        pEventError;
    tOplkApiEventType   eventType;
    tOplkApiEventArg    apiEventArg;

    ret = kErrorOk;

    switch (pEvent_p->eventType)
    {
        // error event
        case kEventTypeError:
            pEventError = (tEventError*)pEvent_p->pEventArg;
            switch (pEventError->eventSource)
            {
                // treat the errors from the following sources as critical
                case kEventSourceEventk:
                case kEventSourceEventu:
                case kEventSourceDllk:
                    eventType = kOplkApiEventCriticalError;
                    // halt the stack by entering NMT state Off
                    ret = nmtu_postNmtEvent(kNmtEventCriticalError);
                    break;

                // the other errors are just warnings
                default:
                    eventType = kOplkApiEventWarning;
                    break;
            }

            // call user callback
            ret = ctrlu_callUserEventCallback(eventType, (tOplkApiEventArg*)pEventError);
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
            ret = ctrlu_callUserEventCallback(eventType, (tOplkApiEventArg*)pEvent_p->pEventArg);
            break;

        // user-defined event
        case kEventTypeApiUserDef:
            eventType = kOplkApiEventUserDef;
            apiEventArg.pUserArg = *(void**)pEvent_p->pEventArg;
            ret = ctrlu_callUserEventCallback(eventType, &apiEventArg);
            break;

#if defined(CONFIG_INCLUDE_NMT_MN) && defined(CONFIG_INCLUDE_PRES_FORWARD)
        case kEventTypeReceivedPres:
            {
                tOplkApiEventReceivedPres*  pApiData;
                tDllEventReceivedPres*      pDllData;

                pApiData = &apiEventArg.receivedPres;
                pDllData = (tDllEventReceivedPres*)pEvent_p->pEventArg;

                pApiData->nodeId = pDllData->nodeId;
                pApiData->frameSize = pDllData->frameSize;
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

\param  pInitParam_p        Pointer to the stack initialization parameters.
\param  fUpdateIdentity_p   Flag determines if identity will also be updated.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError updateDllConfig(tOplkApiInitParam* pInitParam_p, BOOL fUpdateIdentity_p)
{
    tOplkError          ret = kErrorOk;
    tDllConfigParam     dllConfigParam;
    tDllIdentParam      dllIdentParam;
    tObdSize            obdSize;
    UINT16              wTemp;
    UINT8               bTemp;

    // configure Dll
    OPLK_MEMSET(&dllConfigParam, 0, sizeof(dllConfigParam));
    dllConfigParam.nodeId = obd_getNodeId();

    // Cycle Length (0x1006: NMT_CycleLen_U32) in [us]
    obdSize = 4;
    if ((ret = obd_readEntry(0x1006, 0, &dllConfigParam.cycleLen, &obdSize)) != kErrorOk)
        return ret;

    // 0x1F82: NMT_FeatureFlags_U32
    obdSize = 4;
    if ((ret = obd_readEntry(0x1F82, 0, &dllConfigParam.featureFlags, &obdSize)) != kErrorOk)
        return ret;

    // d.k. There is no dependence between FeatureFlags and async-only CN
    dllConfigParam.fAsyncOnly = pInitParam_p->fAsyncOnly;

    // 0x1C14: DLL_LossOfFrameTolerance_U32 in [ns]
    obdSize = 4;
    if ((ret = obd_readEntry(0x1C14, 0, &dllConfigParam.lossOfFrameTolerance, &obdSize)) != kErrorOk)
        return ret;

    // 0x1F98: NMT_CycleTiming_REC, 0x1F98.1: IsochrTxMaxPayload_U16
    obdSize = 2;
    if ((ret = obd_readEntry(0x1F98, 1, &wTemp, &obdSize)) != kErrorOk)
        return ret;
    dllConfigParam.isochrTxMaxPayload = wTemp;

    // 0x1F98.2: IsochrRxMaxPayload_U16
    obdSize = 2;
    if ((ret = obd_readEntry(0x1F98, 2, &wTemp, &obdSize)) != kErrorOk)
        return ret;
    dllConfigParam.isochrRxMaxPayload = wTemp;

    // 0x1F98.3: PResMaxLatency_U32
    obdSize = 4;
    if ((ret = obd_readEntry(0x1F98, 3, &dllConfigParam.presMaxLatency, &obdSize)) != kErrorOk)
        return ret;

    // 0x1F98.4: PReqActPayloadLimit_U16
    obdSize = 2;
    if ((ret = obd_readEntry(0x1F98, 4, &wTemp, &obdSize)) != kErrorOk)
        return ret;
    dllConfigParam.preqActPayloadLimit = wTemp;

    // 0x1F98.5: PResActPayloadLimit_U16
    obdSize = 2;
    if ((ret = obd_readEntry(0x1F98, 5, &wTemp, &obdSize)) != kErrorOk)
        return ret;
    dllConfigParam.presActPayloadLimit = wTemp;

    // 0x1F98.6: ASndMaxLatency_U32
    obdSize = 4;
    if ((ret = obd_readEntry(0x1F98, 6, &dllConfigParam.asndMaxLatency, &obdSize)) != kErrorOk)
        return ret;

    // 0x1F98.7: MultiplCycleCnt_U8
    obdSize = 1;
    if ((ret = obd_readEntry(0x1F98, 7, &bTemp, &obdSize)) != kErrorOk)
        return ret;
    dllConfigParam.multipleCycleCnt = bTemp;

    // 0x1F98.8: AsyncMTU_U16
    obdSize = 2;
    if ((ret = obd_readEntry(0x1F98, 8, &wTemp, &obdSize)) != kErrorOk)
        return ret;
    dllConfigParam.asyncMtu = wTemp;

    // 0x1F98.9: Prescaler_U16
    obdSize = 2;
    if ((ret = obd_readEntry(0x1F98, 9, &wTemp, &obdSize)) != kErrorOk)
        return ret;
    //User value passed from application will already be updated by now in updateObd
    dllConfigParam.prescaler = wTemp;

#if defined(CONFIG_INCLUDE_NMT_MN)
    // 0x1F8A.1: WaitSoCPReq_U32 in [ns]
    obdSize = 4;
    if ((ret = obd_readEntry(0x1F8A, 1, &dllConfigParam.waitSocPreq, &obdSize)) != kErrorOk)
        return ret;

    // 0x1F8A.2: AsyncSlotTimeout_U32 in [ns] (optional)
    obdSize = 4;
    obd_readEntry(0x1F8A, 2, &dllConfigParam.asyncSlotTimeout, &obdSize);
#endif

#if CONFIG_DLL_PRES_CHAINING_CN != FALSE
    dllConfigParam.syncResLatency = pInitParam_p->syncResLatency;
#endif

#if defined(CONFIG_INCLUDE_NMT_RMN)
    {
        UINT32 mnSwitchOverPriority = 0;
        UINT32 mnSwitchOverDelay = 0;
        UINT32 mnSwitchOverCycleDivider = 0;
        UINT32 mnWaitNotAct = 0;

        obdSize = 4;
        if ((ret = obd_readEntry(0x1F89, 0x0a, &mnSwitchOverPriority, &obdSize)) != kErrorOk)
            return ret;

        obdSize = 4;
        if ((ret = obd_readEntry(0x1F89, 0x0b, &mnSwitchOverDelay, &obdSize)) != kErrorOk)
            return ret;

        obdSize = 4;
        if ((ret = obd_readEntry(0x1F89, 0x0c, &mnSwitchOverCycleDivider, &obdSize)) != kErrorOk)
            return ret;

        dllConfigParam.switchOverTimeMn = (UINT32)(dllConfigParam.cycleLen +
                ((dllConfigParam.cycleLen * ((UINT64)mnSwitchOverPriority)) /
                mnSwitchOverCycleDivider));

        dllConfigParam.delayedSwitchOverTimeMn = (UINT32)(dllConfigParam.cycleLen +
                ((dllConfigParam.cycleLen * ((UINT64)mnSwitchOverPriority + mnSwitchOverDelay)) /
                mnSwitchOverCycleDivider));

        obdSize = 4;
        if ((ret = obd_readEntry(0x1F89, 0x01, &mnWaitNotAct, &obdSize)) != kErrorOk)
            return ret;

        dllConfigParam.reducedSwitchOverTimeMn = mnWaitNotAct;
    }
#endif

    dllConfigParam.fSyncOnPrcNode = pInitParam_p->fSyncOnPrcNode;
    dllConfigParam.syncNodeId = pInitParam_p->syncNodeId;

    dllConfigParam.sizeOfStruct = sizeof(dllConfigParam);
    if ((ret = dllucal_config(&dllConfigParam)) != kErrorOk)
        return ret;

    if (fUpdateIdentity_p != FALSE)
    {
        // configure Identity
        OPLK_MEMSET(&dllIdentParam, 0, sizeof(dllIdentParam));

        obdSize = 4;
        if ((ret = obd_readEntry(0x1000, 0, &dllIdentParam.deviceType, &obdSize)) != kErrorOk)
            return ret;

        obdSize = 4;
        if ((ret = obd_readEntry(0x1018, 1, &dllIdentParam.vendorId, &obdSize)) != kErrorOk)
            return ret;

        obdSize = 4;
        if ((ret = obd_readEntry(0x1018, 2, &dllIdentParam.productCode, &obdSize)) != kErrorOk)
            return ret;

        obdSize = 4;
        if ((ret = obd_readEntry(0x1018, 3, &dllIdentParam.revisionNumber, &obdSize)) != kErrorOk)
            return ret;

        obdSize = 4;
        if ((ret = obd_readEntry(0x1018, 4, &dllIdentParam.serialNumber, &obdSize)) != kErrorOk)
            return ret;

        dllIdentParam.ipAddress = pInitParam_p->ipAddress;
        dllIdentParam.subnetMask = pInitParam_p->subnetMask;

        obdSize = sizeof(dllIdentParam.defaultGateway);
        ret = obd_readEntry(0x1E40, 5, &dllIdentParam.defaultGateway, &obdSize);
        if (ret != kErrorOk)
        {   // NWL_IpAddrTable_Xh_REC.DefaultGateway_IPAD seams to not exist,
            // so use the one supplied in the init parameter
            dllIdentParam.defaultGateway = pInitParam_p->defaultGateway;
        }

#if defined(CONFIG_INCLUDE_VETH)
        // configure Virtual Ethernet Driver
        ret = target_setIpAdrs(PLK_VETH_NAME, dllIdentParam.ipAddress, dllIdentParam.subnetMask, (UINT16)dllConfigParam.asyncMtu);
        if (ret != kErrorOk)
            return ret;

#if (CONFIG_VETH_SET_DEFAULT_GATEWAY == TRUE)
        ret = target_setDefaultGateway(dllIdentParam.defaultGateway);
        if (ret != kErrorOk)
            return ret;
#endif

#endif

        obdSize = sizeof(dllIdentParam.sHostname);
        if ((ret = obd_readEntry(0x1F9A, 0, &dllIdentParam.sHostname[0], &obdSize)) != kErrorOk)
        {   // NMT_HostName_VSTR seams to not exist,
            // so use the one supplied in the init parameter
            OPLK_MEMCPY(dllIdentParam.sHostname, pInitParam_p->sHostname, sizeof(dllIdentParam.sHostname));
        }

        obdSize = 4;
        obd_readEntry(0x1020, 1, &dllIdentParam.verifyConfigurationDate, &obdSize);
        // ignore any error, because this object is optional

        obdSize = 4;
        obd_readEntry(0x1020, 2, &dllIdentParam.verifyConfigurationTime, &obdSize);
        // ignore any error, because this object is optional

        dllIdentParam.applicationSwDate = pInitParam_p->applicationSwDate;
        dllIdentParam.applicationSwTime = pInitParam_p->applicationSwTime;

        dllIdentParam.vendorSpecificExt1 = pInitParam_p->vendorSpecificExt1;

        OPLK_MEMCPY(&dllIdentParam.aVendorSpecificExt2[0], &pInitParam_p->aVendorSpecificExt2[0],
                    sizeof(dllIdentParam.aVendorSpecificExt2));

        dllIdentParam.sizeOfStruct = sizeof (dllIdentParam);
        if ((ret = dllucal_setIdentity(&dllIdentParam)) != kErrorOk)
            return ret;
    }
    return ret;
}

#if defined(CONFIG_INCLUDE_SDOS) || defined(CONFIG_INCLUDE_SDOC)
//------------------------------------------------------------------------------
/**
\brief  Update the SDO configuration

The function updates the SDO configuration from the object dictionary.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError updateSdoConfig(void)
{
    tOplkError          ret = kErrorOk;
    tObdSize            obdSize;
    DWORD               sdoSequTimeout;

    obdSize = sizeof(sdoSequTimeout);
    ret = obd_readEntry(0x1300, 0, &sdoSequTimeout, &obdSize);
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

\param  pInitParam_p        Pointer to the stack initialization parameters.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError updateObd(tOplkApiInitParam* pInitParam_p)
{
    tOplkError          ret = kErrorOk;
    WORD                wTemp;
    BYTE                bTemp;

    // set node id in OD
    ret = obd_setNodeId(pInitParam_p->nodeId,    // node id
                        kObdNodeIdHardware);     // set by hardware
    if (ret != kErrorOk)
        return ret;

    if (pInitParam_p->cycleLen != UINT_MAX)
    {
        obd_writeEntry(0x1006, 0, &pInitParam_p->cycleLen, 4);
    }

    if (pInitParam_p->lossOfFrameTolerance != UINT_MAX)
    {
        obd_writeEntry(0x1C14, 0, &pInitParam_p->lossOfFrameTolerance, 4);
    }

    // d.k. There is no dependance between FeatureFlags and async-only CN.
    if (pInitParam_p->featureFlags != UINT_MAX)
    {
        obd_writeEntry(0x1F82, 0, &pInitParam_p->featureFlags, 4);
    }

    wTemp = (WORD)pInitParam_p->isochrTxMaxPayload;
    obd_writeEntry(0x1F98, 1, &wTemp, 2);

    wTemp = (WORD)pInitParam_p->isochrRxMaxPayload;
    obd_writeEntry(0x1F98, 2, &wTemp, 2);

    obd_writeEntry(0x1F98, 3, &pInitParam_p->presMaxLatency, 4);

    if (pInitParam_p->preqActPayloadLimit <= C_DLL_ISOCHR_MAX_PAYL)
    {
        wTemp = (WORD)pInitParam_p->preqActPayloadLimit;
        obd_writeEntry(0x1F98, 4, &wTemp, 2);
    }

    if (pInitParam_p->presActPayloadLimit <= C_DLL_ISOCHR_MAX_PAYL)
    {
        wTemp = (WORD)pInitParam_p->presActPayloadLimit;
        obd_writeEntry(0x1F98, 5, &wTemp, 2);
    }

    obd_writeEntry(0x1F98, 6, &pInitParam_p->asndMaxLatency, 4);

    if (pInitParam_p->multiplCylceCnt <= 0xFF)
    {
        bTemp = (BYTE)pInitParam_p->multiplCylceCnt;
        obd_writeEntry(0x1F98, 7, &bTemp, 1);
    }

    if (pInitParam_p->asyncMtu <= C_DLL_MAX_ASYNC_MTU)
    {
        wTemp = (WORD)pInitParam_p->asyncMtu;
        obd_writeEntry(0x1F98, 8, &wTemp, 2);
    }

    if (pInitParam_p->prescaler <= 1000)
    {
        wTemp = (WORD)pInitParam_p->prescaler;
        obd_writeEntry(0x1F98, 9, &wTemp, 2);
    }

#if defined(CONFIG_INCLUDE_NMT_MN)
    if (pInitParam_p->waitSocPreq != UINT_MAX)
    {
        obd_writeEntry(0x1F8A, 1, &pInitParam_p->waitSocPreq, 4);
    }

    if ((pInitParam_p->asyncSlotTimeout != 0) && (pInitParam_p->asyncSlotTimeout != UINT_MAX))
    {
        obd_writeEntry(0x1F8A, 2, &pInitParam_p->asyncSlotTimeout, 4);
    }
#endif

#if defined(CONFIG_INCLUDE_NMT_RMN)
    {
        UINT32              switchOverPriority;

        if (pInitParam_p->nodeId > C_ADR_MN_DEF_NODE_ID)
        {
            switchOverPriority = pInitParam_p->nodeId - C_ADR_MN_DEF_NODE_ID;
            obd_writeEntry(0x1F89, 0x0a, &switchOverPriority, 4);
        }
    }
#endif

    // configure Identity
    if (pInitParam_p->deviceType != UINT_MAX)
    {
        obd_writeEntry(0x1000, 0, &pInitParam_p->deviceType, 4);
    }

    if (pInitParam_p->vendorId != UINT_MAX)
    {
        obd_writeEntry(0x1018, 1, &pInitParam_p->vendorId, 4);
    }

    if (pInitParam_p->productCode != UINT_MAX)
    {
        obd_writeEntry(0x1018, 2, &pInitParam_p->productCode, 4);
    }

    if (pInitParam_p->revisionNumber != UINT_MAX)
    {
        obd_writeEntry(0x1018, 3, &pInitParam_p->revisionNumber, 4);
    }

    if (pInitParam_p->serialNumber != UINT_MAX)
    {
        obd_writeEntry(0x1018, 4, &pInitParam_p->serialNumber, 4);
    }

    if (pInitParam_p->pDevName != NULL)
    {
        // write Device Name (0x1008)
        obd_writeEntry(0x1008, 0, (void*)pInitParam_p->pDevName,
                       (tObdSize)strlen(pInitParam_p->pDevName));
    }

    if (pInitParam_p->pHwVersion != NULL)
    {
        // write Hardware version (0x1009)
        obd_writeEntry(0x1009, 0, (void*)pInitParam_p->pHwVersion,
                       (tObdSize)strlen(pInitParam_p->pHwVersion));
    }

    if (pInitParam_p->pSwVersion != NULL)
    {
        // write Software version (0x100A)
        obd_writeEntry(0x100A, 0, (void*)pInitParam_p->pSwVersion,
                       (tObdSize)strlen(pInitParam_p->pSwVersion));
    }

#if defined(CONFIG_INCLUDE_VETH)
    // write NMT_HostName_VSTR (0x1F9A)
    obd_writeEntry(0x1F9A, 0, (void*)&pInitParam_p->sHostname[0],
                   sizeof(pInitParam_p->sHostname));

    //DEBUG_LVL_CTRL_TRACE("%s: write NMT_HostName_VSTR %d\n", __func__, Ret);

    // write NWL_IpAddrTable_Xh_REC.Addr_IPAD (0x1E40/2)
    obd_writeEntry(0x1E40, 2, (void*)&pInitParam_p->ipAddress,
                   sizeof(pInitParam_p->ipAddress));

    // write NWL_IpAddrTable_Xh_REC.NetMask_IPAD (0x1E40/3)
    obd_writeEntry(0x1E40, 3, (void*)&pInitParam_p->subnetMask,
                   sizeof(pInitParam_p->subnetMask));

    // write NWL_IpAddrTable_Xh_REC.DefaultGateway_IPAD (0x1E40/5)
    obd_writeEntry(0x1E40, 5, (void*)&pInitParam_p->defaultGateway,
                   sizeof(pInitParam_p->defaultGateway));
#endif

    // ignore return code
    return kErrorOk;
}

#if defined(CONFIG_INCLUDE_NMT_MN)
//------------------------------------------------------------------------------
/**
\brief  Callback function for node event

The function implements the callback function for node events.

\param  nodeId_p        Node ID of the CN.
\param  nodeEvent_p     Event from the specified CN.
\param  nmtState_p      Current NMT state of the CN.
\param  errorCode_p     Contains the error code for the node event kNmtNodeEventError
\param  fMandatory_p    Flag determines if the CN is mandatory.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError cbNodeEvent(UINT nodeId_p, tNmtNodeEvent nodeEvent_p, tNmtState nmtState_p,
                              UINT16 errorCode_p, BOOL fMandatory_p)
{
    tOplkError              ret;
    tOplkApiEventArg        eventArg;

    ret = kErrorOk;

    // call user callback
    eventArg.nodeEvent.nodeId = nodeId_p;
    eventArg.nodeEvent.nodeEvent = nodeEvent_p;
    eventArg.nodeEvent.nmtState = nmtState_p;
    eventArg.nodeEvent.errorCode = errorCode_p;
    eventArg.nodeEvent.fMandatory = fMandatory_p;

    ret = ctrlu_callUserEventCallback(kOplkApiEventNode, &eventArg);
    if (((nodeEvent_p == kNmtNodeEventCheckConf) || (nodeEvent_p == kNmtNodeEventUpdateConf)) &&
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

\param  bootEvent_p     Event from the boot-up process.
\param  nmtState_p      Current local NMT state.
\param  errorCode_p     Contains the error code for the node event kNmtBootEventError

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError cbBootEvent(tNmtBootEvent bootEvent_p, tNmtState nmtState_p,
                              UINT16 errorCode_p)
{
    tOplkError              ret;
    tOplkApiEventArg        eventArg;

    ret = kErrorOk;

    // call user callback
    eventArg.bootEvent.bootEvent = bootEvent_p;
    eventArg.bootEvent.nmtState = nmtState_p;
    eventArg.bootEvent.errorCode = errorCode_p;

    ret = ctrlu_callUserEventCallback(kOplkApiEventBoot, &eventArg);
    return ret;
}

#if defined(CONFIG_INCLUDE_LEDU)
//------------------------------------------------------------------------------
/**
\brief  Callback function for LED change events

The function implements the callback function for LED change events.

\param  ledType_p       Type of LED.
\param  fOn_p           State of LED. TRUE = on, FALSE = off

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError cbLedStateChange(tLedType ledType_p, BOOL fOn_p)
{
    tOplkError              ret;
    tOplkApiEventArg        eventArg;

    ret = kErrorOk;

    // call user callback
    eventArg.ledEvent.ledType = ledType_p;
    eventArg.ledEvent.fOn = fOn_p;

    ret = ctrlu_callUserEventCallback(kOplkApiEventLed, &eventArg);

    return ret;
}
#endif


#if defined(CONFIG_INCLUDE_CFM)
//------------------------------------------------------------------------------
/**
\brief  Callback function for CFM progress events

The function implements the callback function for CFM progress events.

\param  pEventCnProgress_p         Pointer to structure with additional
                                   information.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError cbCfmEventCnProgress(tCfmEventCnProgress* pEventCnProgress_p)
{
    tOplkError              ret;
    tOplkApiEventArg        eventArg;

    ret = kErrorOk;

    eventArg.cfmProgress = *pEventCnProgress_p;
    ret = ctrlu_callUserEventCallback(kOplkApiEventCfmProgress, &eventArg);
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Callback function for CFM result events

The function implements the callback function for CFM result events.

\param  nodeId_p                Node ID of CN.
\param  nodeCommand_p           NMT command which shall be executed.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError cbCfmEventCnResult(UINT nodeId_p, tNmtNodeCommand nodeCommand_p)
{
    tOplkError              ret;
    tOplkApiEventArg        eventArg;

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

\param  nmtEvent_p              NMT event to check.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError cbCnCheckEvent(tNmtEvent nmtEvent_p)
{
    tOplkError              ret = kErrorOk;
    tNmtState               nmtState;

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

#if defined(CONFIG_INCLUDE_PDO)
//------------------------------------------------------------------------------
/**
\brief  Callback function for PDO change events

The function posts PDO change events directly to API layer.

\param  pEventPdoChange_p       Pointer to PDO change event.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError cbEventPdoChange(tPdoEventPdoChange* pEventPdoChange_p)
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

\param  pLinkRequest_p      Pointer to link request entry table.
\param  requestCnt_p        Number of link requests.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError linkDomainObjects(tLinkObjectRequest* pLinkRequest_p,
                                    size_t requestCnt_p)
{
    tOplkError              ret = kErrorOk;
    tObdSize                entrySize;
    UINT                    varEntries;
    size_t                  cnt;

    for (cnt = 0; cnt < requestCnt_p; cnt++, pLinkRequest_p++)
    {
        entrySize = pLinkRequest_p->entrySize;
        varEntries = pLinkRequest_p->varEntries;

        ret = oplk_linkObject(pLinkRequest_p->index, pLinkRequest_p->pVar,
                              &varEntries, &entrySize, pLinkRequest_p->firstSubindex);
        if (ret != kErrorOk)
            break;
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
    UINT32          requiredKernelFeatures = 0;

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
    // need a kernel whith this feature.
    requiredKernelFeatures |= OPLK_KERNEL_PRES_FORWARD;
#endif

#if defined(CONFIG_INCLUDE_VETH)
    // We contain the virtual ethernet module and therefore need a kernel
    // which supports virtual ethernet.
    requiredKernelFeatures |= OPLK_KERNEL_VETH;
#endif

#if defined(CONFIG_INCLUDE_NMT_RMN)
    // We contain the code for the redundancy MN (RMN)
    requiredKernelFeatures |= OPLK_KERNEL_RMN;
#endif

#if (CONFIG_DLL_PRES_CHAINING_CN == TRUE)
    requiredKernelFeatures |= OPLK_KERNEL_PRES_CHAINING_CN;
#endif

    return requiredKernelFeatures;
}

/// \}
