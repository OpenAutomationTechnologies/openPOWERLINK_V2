/**
********************************************************************************
\file   ctrlu.c

\brief  User stack control module

This file contains the implementation of the user stack control module.

\ingroup module_ctrlu
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2013, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
Copyright (c) 2013, SYSTEC electronic GmbH
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
#include <Epl.h>
#include <kernel/dllk.h>
#include <kernel/eventk.h>
#include <kernel/nmtk.h>
#include <kernel/dllkcal.h>
#include <kernel/pdokcal.h>
#include <user/pdoucal.h>
#include <user/pdou.h>
#include <user/dllucal.h>
#include <user/errhndu.h>
#include <user/ledu.h>
#include <user/nmtcnu.h>
#include <user/nmtmnu.h>
#include <user/EplSdoComu.h>
#include <user/identu.h>
#include <user/statusu.h>
#include <user/EplTimeru.h>
#include <user/cfmu.h>
#include <user/eventucal.h>
#include <EplTarget.h>

#include <ctrl.h>
#include <obd.h>

#include <user/ctrlucal.h>

#if defined(CONFIG_INCLUDE_PDOK)
#include <kernel/pdok.h>
#endif

#if EPL_USE_SHAREDBUFF != FALSE
#include <SharedBuff.h>
#endif

#if (CONFIG_OBD_USE_LOAD_CONCISEDCF != FALSE)
#include <obdcdc.h>
#endif

#if EPL_NMTMNU_PRES_CHAINING_MN != FALSE
#include <user/syncu.h>
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
typedef struct
{
    UINT16              lastHeartbeat;          ///< Last detected heartbeat
    tEplApiInitParam    initParam;              ///< Stack initialization parameters
} tCtrluInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tCtrluInstance       ctrlInstance_l;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static tEplKernel initNmtu(tEplApiInitParam* pInitParam_p);
static tEplKernel initObd(tEplApiInitParam* pInitParam_p);
static tEplKernel updateDllConfig(tEplApiInitParam* pInitParam_p, BOOL fUpdateIdentity_p);
static tEplKernel updateSdoConfig();
static tEplKernel updateObd(tEplApiInitParam* pInitParam_p);

static tEplKernel processUserEvent(tEplEvent* pEplEvent_p);
static tEplKernel cbCnCheckEvent(tNmtEvent NmtEvent_p);
static tEplKernel cbNmtStateChange(tEventNmtStateChange nmtStateChange_p);

#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)
static tEplKernel cbNodeEvent(UINT nodeId_p, tNmtNodeEvent nodeEvent_p,
                                    tNmtState nmtState_p, UINT16 errorCode_p,
                                    BOOL fMandatory_p);
#endif // (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)

static tEplKernel cbBootEvent(tNmtBootEvent BootEvent_p, tNmtState NmtState_p,
                                    UINT16 errorCode_p);

#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_LEDU)) != 0)
static tEplKernel cbLedStateChange(tLedType LedType_p, BOOL fOn_p);
#endif

#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_CFM)) != 0)
static tEplKernel cbCfmEventCnProgress(tCfmEventCnProgress* pEventCnProgress_p);
static tEplKernel cbCfmEventCnResult(unsigned int uiNodeId_p, tNmtNodeCommand NodeCommand_p);
#endif

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Initialize user control module

The function initializes the user control module.

\return The function returns a tEplKernel error code.

\ingroup module_ctrlu
*/
//------------------------------------------------------------------------------
tEplKernel ctrlu_init(void)
{
#if EPL_USE_SHAREDBUFF != FALSE
    tShbError           shbError;
#endif
    tEplKernel          ret;

    TRACE ("Initialize ctrl module ...\n");

    ctrlInstance_l.lastHeartbeat = 0;

#if EPL_USE_SHAREDBUFF != FALSE
    shbError = ShbInit();
    if (shbError != kShbOk)
    {
        EPL_DBGLVL_ERROR_TRACE ("Could not initialize Shared Buffer\n");
        return kEplNoResource;
    }
#endif

    if ((ret = ctrlucal_init()) != kEplSuccessful)
    {
        EPL_DBGLVL_ERROR_TRACE ("Could not initialize ctrlucal\n");
        goto Exit;
    }

    if ((ret = ctrlucal_checkKernelStack()) != kEplSuccessful)
    {
        ctrlucal_exit();
        goto Exit;
    }
    return kEplSuccessful;

Exit:
#if EPL_USE_SHAREDBUFF != FALSE
    ShbExit();
#endif
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Cleanup user control module

The function cleans up the user control module.

\return The function returns a tEplKernel error code.

\ingroup module_ctrlu
*/
//------------------------------------------------------------------------------
void ctrlu_exit(void)
{
    ctrlucal_exit();

#if EPL_USE_SHAREDBUFF != FALSE
    ShbExit();
#endif
}

//------------------------------------------------------------------------------
/**
\brief  Initialize openPOWERLINK stack

The function initializes the openPOWERLINK stack. It initializes all
user modules and communication with the kernel control module to initialize
the kernel modules.

After return from this function the application must start the NMT state
machine via EplApiExecNmtCommand(kNmtEventSwReset) and thereby the whole
openPOWERLINK stack.

\param  pInitParam_p            Pointer to the initialization parameters
                                provided by the application.

\return The function returns a tEplKernel error code.

\ingroup module_ctrlu
*/
//------------------------------------------------------------------------------
tEplKernel ctrlu_initStack(tEplApiInitParam * pInitParam_p)
{
    tEplKernel              ret = kEplSuccessful;
    tCtrlInitParam          ctrlParam;

    // reset instance structure
    EPL_MEMSET(&ctrlInstance_l.initParam, 0, sizeof (tEplApiInitParam));
    EPL_MEMCPY(&ctrlInstance_l.initParam, pInitParam_p,
               min(sizeof(tEplApiInitParam), (size_t)pInitParam_p->m_uiSizeOfStruct));

    // check event callback function pointer
    if (ctrlInstance_l.initParam.m_pfnCbEvent == NULL)
    {   // application must always have an event callback function
        ret = kEplApiInvalidParam;
        goto Exit;
    }

    if ((ret = initObd(&ctrlInstance_l.initParam)) != kEplSuccessful)
        goto Exit;

    TRACE ("Initializing kernel modules ...\n");
    EPL_MEMCPY (ctrlParam.aMacAddress, ctrlInstance_l.initParam.m_abMacAddress, 6);
    strncpy(ctrlParam.szEthDevName, ctrlInstance_l.initParam.m_HwParam.m_pszDevName, 127);
    ctrlParam.ethDevNumber = ctrlInstance_l.initParam.m_HwParam.m_uiDevNumber;
    ctrlucal_storeInitParam(&ctrlParam);

    if ((ret = ctrlucal_executeCmd(kCtrlInitStack)) != kEplSuccessful)
        goto Exit;

    /* Read back init param because current MAC address was copied by DLLK */
    ret = ctrlucal_readInitParam(&ctrlParam);
    if (ret != kEplSuccessful)
    {
        goto Exit;
    }

    EPL_MEMCPY (ctrlInstance_l.initParam.m_abMacAddress, ctrlParam.aMacAddress, 6);

    TRACE ("Initialize Eventu module...\n");
    if ((ret = eventu_init(processUserEvent)) != kEplSuccessful)
        goto Exit;

    TRACE ("Initialize Timeru module...\n");
    if ((ret = EplTimeruInit()) != kEplSuccessful)
        goto Exit;

    TRACE ("initialize error handler user module...\n");
    ret = errhndu_init();
    if (ret != kEplSuccessful)
    {
        goto Exit;
    }

#if defined(CONFIG_INCLUDE_DLLU)
    TRACE ("Initialize DlluCal module...\n");
    ret = dllucal_init();
    if (ret != kEplSuccessful)
    {
        goto Exit;
    }
#endif

#if defined(CONFIG_INCLUDE_PDOU)
    TRACE ("Initialize Pdou module...\n");
    ret = pdou_init(ctrlInstance_l.initParam.m_pfnCbSync);
    if (ret != kEplSuccessful)
    {
        goto Exit;
    }
#endif

#if defined(CONFIG_INCLUDE_NMTU)
    if ((ret = initNmtu(&ctrlInstance_l.initParam)) != kEplSuccessful)
        goto Exit;
#endif

#if defined(CONFIG_INCLUDE_LEDU)
    ret = ledu_init(cbLedStateChange);
    if (ret != kEplSuccessful)
    {
        goto Exit;
    }
#endif

#if defined(CONFIG_INCLUDE_SDOS) || defined(CONFIG_INCLUDE_SDOC)
    // init sdo command layer
    TRACE ("Initialize SdoCom module...\n");
    ret = EplSdoComInit();
    if (ret != kEplSuccessful)
    {
        goto Exit;
    }
#endif

#if defined (CONFIG_INCLUDE_CFM)
    TRACE ("Initialize Cfm module...\n");
    ret = cfmu_init(cbCfmEventCnProgress, cbCfmEventCnResult);
    if (ret != kEplSuccessful)
    {
        goto Exit;
    }
#endif

    // the application must start NMT state machine
    // via EplApiExecNmtCommand(kNmtEventSwReset)
    // and thereby the whole EPL stack

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Shutdown openPOWERLINK stack

The function shuts down the openPOWERLINK stack. I cleans up all user modules
and the kernel modules by using the kernel control module.

\return The function returns a tEplKernel error code.

\ingroup module_ctrlu
*/
//------------------------------------------------------------------------------
tEplKernel ctrlu_shutdownStack(void)
{
    tEplKernel      ret = kEplSuccessful;

    ret = eventu_exit();
    TRACE("eventu_exit():  0x%X\n", ret);

#if defined(CONFIG_INCLUDE_CFM)
    ret = cfmu_exit();
    TRACE("cfmu_exit():    0x%X\n", ret);
#endif

#if defined(CONFIG_INCLUDE_SDOS) || defined(CONFIG_INCLUDE_SDOC)
    ret = EplSdoComDelInstance();
    TRACE("EplSdoComDelInstance():  0x%X\n", ret);
#endif

#if defined(CONFIG_INCLUDE_LEDU)
    ret = ledu_exit();
    TRACE("ledu_exit():    0x%X\n", ret);
#endif

#if defined(CONFIG_INCLUDE_NMT_MN)
    ret = nmtmnu_delInstance();
    TRACE("EplNmtMnuDelInstance():  0x%X\n", ret);

    ret = identu_delInstance();
    TRACE("identu_delInstance():  0x%X\n", ret);

    ret = statusu_delInstance();
    TRACE("statusu_delInstance():  0x%X\n", ret);

#if EPL_NMTMNU_PRES_CHAINING_MN != FALSE
    ret = syncu_delInstance();
#endif

#endif

#if defined(CONFIG_INCLUDE_NMT_CN)
    ret = nmtcnu_delInstance();
    TRACE("EplNmtCnuDelInstance():  0x%X\n", ret);
#endif

#if defined(CONFIG_INCLUDE_NMTU)
    ret = nmtu_delInstance();
    TRACE("nmtu_delInstance():    0x%X\n", ret);
#endif

#if defined(CONFIG_INCLUDE_PDOU)
    ret = pdou_exit();
    TRACE("pdou_exit():    0x%X\n", ret);
#endif

#if defined(CONFIG_INCLUDE_DLLU)
    ret = dllucal_exit();
    TRACE("dllucal_exit(): 0x%X\n", ret);
#endif

    ret = errhndu_exit();
    TRACE("errhndu_exit():  0x%X\n", ret);

    ret = EplTimeruDelInstance();
    TRACE("EplTimeruDelInstance():  0x%X\n", ret);

    ret = ctrlucal_executeCmd(kCtrlCleanupStack);
    TRACE("shoutdown kernel modules():  0x%X\n", ret);

#if (CONFIG_OBD_USE_LOAD_CONCISEDCF != FALSE)
    obdcdc_exit();
#endif

    ret = obd_deleteInstance();

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Process openPOWERLINK stack

This function provides processing timing to several tasks in the openPOWERLINK
stack.

\return The function returns a tEplKernel error code.

\ingroup module_ctrlu
*/
//------------------------------------------------------------------------------
tEplKernel ctrlu_processStack(void)
{
tEplKernel Ret = kEplSuccessful;
#if EPL_USE_SHAREDBUFF != FALSE
tShbError  ShbError;

    ShbError = ShbProcess();
    if (ShbError != kShbOk)
    {
        Ret = kEplInvalidOperation;
        goto Exit;
    }
#endif

    eventucal_process();

    Ret = ctrlucal_process();
    if(Ret != kEplSuccessful)
        goto Exit;

    Ret = EplTimeruProcess();

Exit:
    return Ret;
}

//------------------------------------------------------------------------------
/**
\brief  Check if kernel stack is running

The function checks if the kernel stack is still running.

\return Returns TRUE if the kernel stack is running or FALSE if is not running.

\ingroup module_ctrlu
*/
//------------------------------------------------------------------------------
BOOL ctrlu_checkKernelStack(void)
{
    UINT16 heartbeat;

    heartbeat = ctrlucal_getHeartbeat();
    if (heartbeat == ctrlInstance_l.lastHeartbeat)
    {
        TRACE("heartbeat:%d ctrlInstance_l.lastHeartbeat:%d\n", heartbeat, ctrlInstance_l.lastHeartbeat);
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
\brief  Call user event callback

The function calls the user event callback function

\param  eventType_p         Event type to send.
\param  pEventArg_p         Event argument to send.

\return The function returns a tEplKernel error code.

\ingroup module_ctrlu
*/
//------------------------------------------------------------------------------
tEplKernel ctrlu_callUserEventCallback(tEplApiEventType eventType_p, tEplApiEventArg* pEventArg_p)
{
    tEplKernel          ret = kEplSuccessful;

    ret = ctrlInstance_l.initParam.m_pfnCbEvent(eventType_p, pEventArg_p,
                                                ctrlInstance_l.initParam.m_pEventUserArg);
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Default OD callback function

The function implements the standard OD callback function. It contains basic
actions for system objects.

\param      pParam_p        OBD callback parameter.

\return The function returns a tEplKernel error code.

\ingroup module_ctrlu
*/
//------------------------------------------------------------------------------
tEplKernel ctrlu_cbObdAccess(tObdCbParam MEM* pParam_p)
{
    tEplKernel          ret = kEplSuccessful;

#if (EPL_API_OBD_FORWARD_EVENT != FALSE)
    tEplApiEventArg     eventArg;

    // call user callback
    // must be disabled for EplApiLinuxKernel.c, because of reentrancy problem
    // for local OD access. This is not so bad as user callback function in
    // application does not use OD callbacks at the moment.
    eventArg.m_ObdCbParam = *pParam_p;
    ret = ctrlu_callUserEventCallback(kEplApiEventObdAccess, &eventArg);
    if (ret != kEplSuccessful)
    {   // do not do any further processing on this object
        if (ret == kEplReject)
            ret = kEplSuccessful;
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
                ret = kEplSuccessful;
            }
            break;

        case 0x1F9E:    // NMT_ResetCmd_U8
            if (pParam_p->obdEvent == kObdEvPreWrite)
            {
                UINT8    nmtCommand;

                nmtCommand = *((UINT8 *) pParam_p->pArg);
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
                        pParam_p->abortCode = EPL_SDOAC_VALUE_RANGE_EXCEEDED;
                        ret = kEplObdAccessViolation;
                        break;
                }
            }
            else if (pParam_p->obdEvent == kObdEvPostWrite)
            {
                UINT8    nmtCommand;

                nmtCommand = *((UINT8 *) pParam_p->pArg);
                // check value range
                switch ((tNmtCommand)nmtCommand)
                {
#if(((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMTU)) != 0)
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
#endif

                    case kNmtCmdInvalidService:
                        break;

                    default:
                        pParam_p->abortCode = EPL_SDOAC_VALUE_RANGE_EXCEEDED;
                        ret = kEplObdAccessViolation;
                        break;
                }
            }
            break;

#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)
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
                if (ret != kEplSuccessful)
                {
                    pParam_p->abortCode = EPL_SDOAC_GENERAL_ERROR;
                    goto Exit;
                }

                obdSize = sizeof (cmdTarget);
                ret = obd_readEntry(0x1F9F, 3, &cmdTarget, &obdSize);
                if (ret != kEplSuccessful)
                {
                    pParam_p->abortCode = EPL_SDOAC_GENERAL_ERROR;
                    goto Exit;
                }

                nmtState = nmtu_getNmtState();

                if (nmtState < kNmtMsNotActive)
                {   // local node is CN
                    // forward the command to the MN
                    // d.k. this is a manufacturer specific feature
                    ret = nmtcnu_sendNmtRequest(cmdTarget, (tNmtCommand) cmdId);
                }
                else
                {   // local node is MN
                    // directly execute the requested NMT command
                    ret = nmtmnu_requestNmtCommand(cmdTarget, (tNmtCommand) cmdId);
                }
                if (ret != kEplSuccessful)
                {
                    pParam_p->abortCode = EPL_SDOAC_GENERAL_ERROR;
                }

                // reset request flag
                *((UINT8*)pParam_p->pArg) = 0;
            }
            break;
#endif // (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)

        default:
            break;
    }

Exit:
    return ret;
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

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
static tEplKernel initNmtu(tEplApiInitParam* pInitParam_p)
{
    tEplKernel      Ret = kEplSuccessful;

    // initialize EplNmtCnu module
#if defined(CONFIG_INCLUDE_NMT_CN)
    TRACE ("Initialize NMT_CN module...\n");
    Ret = nmtcnu_addInstance(pInitParam_p->m_uiNodeId);
    if (Ret != kEplSuccessful)
        goto Exit;

    Ret = nmtcnu_registerCheckEventCb(cbCnCheckEvent);
    if (Ret != kEplSuccessful)
        goto Exit;
#else
    UNUSED_PARAMETER(pApiInstance_p);
#endif

    // initialize EplNmtu module
#if defined(CONFIG_INCLUDE_NMTU)
    TRACE ("Initialize NMTu module...\n");
    Ret = nmtu_init();
    if (Ret != kEplSuccessful)
        goto Exit;

    // register NMT event callback function
    Ret = nmtu_registerStateChangeCb(cbNmtStateChange);
    if (Ret != kEplSuccessful)
        goto Exit;
#endif

#if defined(CONFIG_INCLUDE_NMT_MN)
    // initialize EplNmtMnu module
    TRACE ("Initialize NMT_MN module...\n");
    Ret = nmtmnu_init(cbNodeEvent, cbBootEvent);
    if (Ret != kEplSuccessful)
        goto Exit;

    // initialize identu module
    TRACE ("Initialize Identu module...\n");
    Ret = identu_init();
    if (Ret != kEplSuccessful)
        goto Exit;

    // initialize EplStatusu module
    TRACE ("Initialize Statusu module...\n");
    Ret = statusu_init();
    if (Ret != kEplSuccessful)
        goto Exit;

#if EPL_NMTMNU_PRES_CHAINING_MN != FALSE
    // initialize syncu module
    TRACE ("Initialize Syncu module...\n");
    Ret = syncu_init();
    if (Ret != kEplSuccessful)
        goto Exit;
#endif

#endif

Exit:
    return Ret;
}

//------------------------------------------------------------------------------
/**
\brief Initialize object dictionary

The function initializes the object dictionary

\param  pInitParam_p        Pointer to init parameters.

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
static tEplKernel initObd(tEplApiInitParam* pInitParam_p)
{
    tEplKernel          ret = kEplSuccessful;
    tObdInitParam       ObdInitParam;

    UNUSED_PARAMETER(pInitParam_p);

#if defined(CONFIG_INCLUDE_OBD)
    TRACE ("Initialize OBD module...\n");
    ret = obd_initObd(&ObdInitParam);
    if (ret != kEplSuccessful)
        return ret;

    ret = obd_init(&ObdInitParam);
    if (ret != kEplSuccessful)
        return ret;

#if (CONFIG_OBD_USE_LOAD_CONCISEDCF != FALSE)
    ret = obdcdc_init();
#endif

#endif
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Callback function for NMT state change events

The function implements the callback function for node events.

\param  nmtStateChange_p        NMT state change event.

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
static tEplKernel cbNmtStateChange(tEventNmtStateChange nmtStateChange_p)
{
    tEplKernel          ret = kEplSuccessful;
    BYTE                nmtState;
    tEplApiEventArg     eventArg;

    // save NMT state in OD
    nmtState = (UINT8) nmtStateChange_p.newNmtState;
    ret = obd_writeEntry(0x1F8C, 0, &nmtState, 1);
    if(ret != kEplSuccessful)
        return ret;

    // do work which must be done in that state
    switch (nmtStateChange_p.newNmtState)
    {
        // EPL stack is not running
        case kNmtGsOff:
            break;

        // first init of the hardware
        case kNmtGsInitialising:
#if 0
#if defined(CONFIG_INCLUDE_SDO_UDP)
            // configure SDO via UDP (i.e. bind it to the EPL ethernet interface)
            ret = sdoudp_config(stackInstance_l.m_InitParam.m_dwIpAddress, EPL_C_SDO_EPL_PORT);
            if (ret != kEplSuccessful)
                return ret;
#endif
#endif

            break;

        // init of the manufacturer-specific profile area and the
        // standardised device profile area
        case kNmtGsResetApplication:
            // reset application part of OD
            ret = obd_accessOdPart(kObdPartApp, kObdDirLoad);
            if (ret != kEplSuccessful)
                return ret;
            break;

        // init of the communication profile area
        case kNmtGsResetCommunication:
            // reset communication part of OD
            ret = obd_accessOdPart(kObdPartGen, kObdDirLoad);
            if (ret != kEplSuccessful)
                return ret;

            // $$$ d.k.: update OD only if OD was not loaded from non-volatile memory
            ret = updateObd(&ctrlInstance_l.initParam);
            if (ret != kEplSuccessful)
                return ret;

#if (CONFIG_OBD_USE_LOAD_CONCISEDCF != FALSE)
            ret = obdcdc_loadCdc();
            if (ret != kEplSuccessful)
                return ret;
#endif
            break;

        // build the configuration with infos from OD
        case kNmtGsResetConfiguration:
            ret = updateDllConfig(&ctrlInstance_l.initParam, TRUE);
            if (ret != kEplSuccessful)
                return ret;

            ret = updateSdoConfig();
            if (ret != kEplSuccessful)
                return ret;

            break;

        //-----------------------------------------------------------
        // CN part of the state machine

        // node list for EPL-Frames and check timeout
        case kNmtCsNotActive:
            // indicate completion of reset in NMT_ResetCmd_U8
            nmtState = (UINT8) kNmtCmdInvalidService;
            ret = obd_writeEntry(0x1F9E, 0, &nmtState, 1);
            if (ret != kEplSuccessful)
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

        // no EPL cycle
        // -> normal ethernet communication
        case kNmtCsBasicEthernet:
            break;

        //-----------------------------------------------------------
        // MN part of the state machine

        // node listens for EPL-Frames and check timeout
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

        // no EPL cycle
        // -> normal ethernet communication
        case kNmtMsBasicEthernet:
            break;

        default:
            TRACE("cbNmtStateChange(): unhandled NMT state\n");
            break;
    }

#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_LEDU)) != 0)
    // forward event to Led module
    ret = ledu_cbNmtStateChange(nmtStateChange_p);
    if (ret != kEplSuccessful)
        return ret;
#endif

#if defined(CONFIG_INCLUDE_PDOU)
    // forward event to Pdou module
    ret = pdou_cbNmtStateChange(nmtStateChange_p);
    if (ret != kEplSuccessful)
        return ret;
#endif

#if defined(CONFIG_INCLUDE_NMT_MN)
    // forward event to NmtMn module
    ret = nmtmnu_cbNmtStateChange(nmtStateChange_p);
    if (ret != kEplSuccessful)
        return ret;
#endif

    // call user callback
    eventArg.m_NmtStateChange = nmtStateChange_p;
    ret = ctrlu_callUserEventCallback(kEplApiEventNmtStateChange, &eventArg);

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Process User event

The function processes events which are sent to the API module. It checks the
events, creates an appropriate API event and forwards it to the application
by calling the event callback function.

\param  pEvent_p             Event to process.

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
static tEplKernel processUserEvent(tEplEvent* pEvent_p)
{
    tEplKernel          ret;
    tEplEventError*     pEventError;
    tEplApiEventType    eventType;
    tEplApiEventArg     apiEventArg;

    ret = kEplSuccessful;

    switch(pEvent_p->m_EventType)
    {
        // error event
        case kEplEventTypeError:
            pEventError = (tEplEventError*) pEvent_p->m_pArg;
            switch (pEventError->m_EventSource)
            {
                // treat the errors from the following sources as critical
                case kEplEventSourceEventk:
                case kEplEventSourceEventu:
                case kEplEventSourceDllk:
                    eventType = kEplApiEventCriticalError;
                    // halt the stack by entering NMT state Off
                    ret = nmtu_postNmtEvent(kNmtEventCriticalError);
                    break;

                // the other errors are just warnings
                default:
                    eventType = kEplApiEventWarning;
                    break;
            }

            // call user callback
            ret = ctrlu_callUserEventCallback(eventType, (tEplApiEventArg*)pEventError);
            // discard error from callback function, because this could generate an endless loop
            ret = kEplSuccessful;
            break;

        // Error history entry event
        case kEplEventTypeHistoryEntry:
            if (pEvent_p->m_uiSize != sizeof(tEplErrHistoryEntry))
            {
                ret = kEplEventWrongSize;
                break;
            }
            eventType = kEplApiEventHistoryEntry;
            ret = ctrlu_callUserEventCallback(eventType, (tEplApiEventArg*)pEvent_p->m_pArg);
            break;

        // user-defined event
        case kEplEventTypeApiUserDef:
            eventType = kEplApiEventUserDef;
            apiEventArg.m_pUserArg = *(void**)pEvent_p->m_pArg;
            ret = ctrlu_callUserEventCallback(eventType, &apiEventArg);
            break;

        // at present, there are no other events for this module
        default:
            ret = kEplInvalidEvent;
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

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
static tEplKernel updateDllConfig(tEplApiInitParam* pInitParam_p, BOOL fUpdateIdentity_p)
{
    tEplKernel          ret = kEplSuccessful;
    tDllConfigParam     dllConfigParam;
    tDllIdentParam      dllIdentParam;
    tObdSize            obdSize;
    UINT16              wTemp;
    UINT8               bTemp;

    // configure Dll
    EPL_MEMSET(&dllConfigParam, 0, sizeof(dllConfigParam));
    dllConfigParam.nodeId = obd_getNodeId();

    // Cycle Length (0x1006: NMT_CycleLen_U32) in [us]
    obdSize = 4;
    if ((ret = obd_readEntry(0x1006, 0, &dllConfigParam.cycleLen, &obdSize)) != kEplSuccessful)
        return ret;

    // 0x1F82: NMT_FeatureFlags_U32
    obdSize = 4;
    if ((ret = obd_readEntry(0x1F82, 0, &dllConfigParam.featureFlags, &obdSize)) != kEplSuccessful)
        return ret;

    // d.k. There is no dependence between FeatureFlags and async-only CN
    dllConfigParam.fAsyncOnly = pInitParam_p->m_fAsyncOnly;

    // 0x1C14: DLL_LossOfFrameTolerance_U32 in [ns]
    obdSize = 4;
    if ((ret = obd_readEntry(0x1C14, 0, &dllConfigParam.lossOfFrameTolerance, &obdSize)) != kEplSuccessful)
        return ret;

    // 0x1F98: NMT_CycleTiming_REC, 0x1F98.1: IsochrTxMaxPayload_U16
    obdSize = 2;
    if ((ret = obd_readEntry(0x1F98, 1, &wTemp, &obdSize)) != kEplSuccessful)
        return ret;
    dllConfigParam.isochrTxMaxPayload = wTemp;

    // 0x1F98.2: IsochrRxMaxPayload_U16
    obdSize = 2;
    if ((ret = obd_readEntry(0x1F98, 2, &wTemp, &obdSize)) != kEplSuccessful)
        return ret;
    dllConfigParam.isochrRxMaxPayload = wTemp;

    // 0x1F98.3: PResMaxLatency_U32
    obdSize = 4;
    if ((ret = obd_readEntry(0x1F98, 3, &dllConfigParam.presMaxLatency, &obdSize)) != kEplSuccessful)
        return ret;

    // 0x1F98.4: PReqActPayloadLimit_U16
    obdSize = 2;
    if ((ret = obd_readEntry(0x1F98, 4, &wTemp, &obdSize)) != kEplSuccessful)
        return ret;
    dllConfigParam.preqActPayloadLimit = wTemp;

    // 0x1F98.5: PResActPayloadLimit_U16
    obdSize = 2;
    if ((ret = obd_readEntry(0x1F98, 5, &wTemp, &obdSize)) != kEplSuccessful)
        return ret;
    dllConfigParam.presActPayloadLimit = wTemp;

    // 0x1F98.6: ASndMaxLatency_U32
    obdSize = 4;
    if ((ret = obd_readEntry(0x1F98, 6, &dllConfigParam.asndMaxLatency, &obdSize)) != kEplSuccessful)
        return ret;

    // 0x1F98.7: MultiplCycleCnt_U8
    obdSize = 1;
    if ((ret = obd_readEntry(0x1F98, 7, &bTemp, &obdSize)) != kEplSuccessful)
        return ret;
    dllConfigParam.multipleCycleCnt = bTemp;

    // 0x1F98.8: AsyncMTU_U16
    obdSize = 2;
    if ((ret = obd_readEntry(0x1F98, 8, &wTemp, &obdSize)) != kEplSuccessful)
        return ret;
    dllConfigParam.asyncMtu = wTemp;

    // $$$ Prescaler

#if defined(CONFIG_INCLUDE_NMT_MN)
    // 0x1F8A.1: WaitSoCPReq_U32 in [ns]
    obdSize = 4;
    if ((ret = obd_readEntry(0x1F8A, 1, &dllConfigParam.waitSocPreq, &obdSize)) != kEplSuccessful)
        return ret;

    // 0x1F8A.2: AsyncSlotTimeout_U32 in [ns] (optional)
    obdSize = 4;
    obd_readEntry(0x1F8A, 2, &dllConfigParam.asyncSlotTimeout, &obdSize);
#endif

#if EPL_DLL_PRES_CHAINING_CN != FALSE
    dllConfigParam.syncResLatency = pInitParam_p->m_dwSyncResLatency;
#endif

    dllConfigParam.fSyncOnPrcNode = pInitParam_p->m_fSyncOnPrcNode;
    dllConfigParam.syncNodeId = pInitParam_p->m_uiSyncNodeId;

    dllConfigParam.sizeOfStruct = sizeof (dllConfigParam);
    if ((ret = dllucal_config(&dllConfigParam)) != kEplSuccessful)
        return ret;

    if (fUpdateIdentity_p != FALSE)
    {
        // configure Identity
        EPL_MEMSET(&dllIdentParam, 0, sizeof (dllIdentParam));

        obdSize = 4;
        if ((ret = obd_readEntry(0x1000, 0, &dllIdentParam.deviceType, &obdSize)) != kEplSuccessful)
            return ret;

        obdSize = 4;
        if ((ret = obd_readEntry(0x1018, 1, &dllIdentParam.vendorId, &obdSize)) != kEplSuccessful)
            return ret;

        obdSize = 4;
        if ((ret = obd_readEntry(0x1018, 2, &dllIdentParam.productCode, &obdSize)) != kEplSuccessful)
            return ret;

        obdSize = 4;
        if ((ret = obd_readEntry(0x1018, 3, &dllIdentParam.revisionNumber, &obdSize)) != kEplSuccessful)
            return ret;

        obdSize = 4;
        if ((ret = obd_readEntry(0x1018, 4, &dllIdentParam.serialNumber, &obdSize)) != kEplSuccessful)
            return ret;

        dllIdentParam.ipAddress = pInitParam_p->m_dwIpAddress;
        dllIdentParam.subnetMask = pInitParam_p->m_dwSubnetMask;

        obdSize = sizeof (dllIdentParam.defaultGateway);
        ret = obd_readEntry(0x1E40, 5, &dllIdentParam.defaultGateway, &obdSize);
        if (ret != kEplSuccessful)
        {   // NWL_IpAddrTable_Xh_REC.DefaultGateway_IPAD seams to not exist,
            // so use the one supplied in the init parameter
            dllIdentParam.defaultGateway = pInitParam_p->m_dwDefaultGateway;
        }

#if defined(CONFIG_INCLUDE_VETH)
        // configure Virtual Ethernet Driver
        ret = target_setIpAdrs(EPL_VETH_NAME, dllIdentParam.ipAddress, dllIdentParam.subnetMask, (UINT16)dllConfigParam.asyncMtu);
        if(ret != kEplSuccessful)
            return ret;

        ret = target_setDefaultGateway(dllIdentParam.defaultGateway);
        if(ret != kEplSuccessful)
            return ret;
#endif

        obdSize = sizeof (dllIdentParam.sHostname);
        if ((ret = obd_readEntry(0x1F9A, 0, &dllIdentParam.sHostname[0], &obdSize)) != kEplSuccessful)
        {   // NMT_HostName_VSTR seams to not exist,
            // so use the one supplied in the init parameter
            EPL_MEMCPY(dllIdentParam.sHostname, pInitParam_p->m_sHostname, sizeof(dllIdentParam.sHostname));
        }

        obdSize = 4;
        obd_readEntry(0x1020, 1, &dllIdentParam.verifyConfigurationDate, &obdSize);
        // ignore any error, because this object is optional

        obdSize = 4;
        obd_readEntry(0x1020, 2, &dllIdentParam.verifyConfigurationTime, &obdSize);
        // ignore any error, because this object is optional

        dllIdentParam.applicationSwDate = pInitParam_p->m_dwApplicationSwDate;
        dllIdentParam.applicationSwTime = pInitParam_p->m_dwApplicationSwTime;

        dllIdentParam.vendorSpecificExt1 = pInitParam_p->m_qwVendorSpecificExt1;

        EPL_MEMCPY(&dllIdentParam.aVendorSpecificExt2[0], &pInitParam_p->m_abVendorSpecificExt2[0],
                   sizeof(dllIdentParam.aVendorSpecificExt2));

        dllIdentParam.sizeOfStruct = sizeof (dllIdentParam);
        if ((ret = dllucal_setIdentity(&dllIdentParam)) != kEplSuccessful)
            return ret;
    }
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Update the SDO configuration

The function updates the SDO configuration from the object dictionary.

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
static tEplKernel updateSdoConfig(void)
{
    tEplKernel          ret = kEplSuccessful;
    tObdSize            obdSize;
    DWORD               sdoSequTimeout;

    obdSize = sizeof(sdoSequTimeout);
    ret = obd_readEntry(0x1300, 0, &sdoSequTimeout, &obdSize);
    if(ret != kEplSuccessful)
        return ret;

    ret = sdoseq_setTimeout(sdoSequTimeout);
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Update the object dictionary

The function updates the object dictionary from the stack initialization
parameters.

\param  pInitParam_p        Pointer to the stack initialization parameters.

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
static tEplKernel updateObd(tEplApiInitParam* pInitParam_p)
{
    tEplKernel          ret = kEplSuccessful;
    WORD                wTemp;
    BYTE                bTemp;

    // set node id in OD
    ret = obd_setNodeId(pInitParam_p->m_uiNodeId,    // node id
                            kObdNodeIdHardware); // set by hardware
    if (ret != kEplSuccessful)
        return ret;

    if (pInitParam_p->m_dwCycleLen != UINT_MAX)
    {
        obd_writeEntry(0x1006, 0, &pInitParam_p->m_dwCycleLen, 4);
    }

    if (pInitParam_p->m_dwLossOfFrameTolerance != UINT_MAX)
    {
        obd_writeEntry(0x1C14, 0, &pInitParam_p->m_dwLossOfFrameTolerance, 4);
    }

    // d.k. There is no dependance between FeatureFlags and async-only CN.
    if (pInitParam_p->m_dwFeatureFlags != UINT_MAX)
    {
        obd_writeEntry(0x1F82, 0, &pInitParam_p->m_dwFeatureFlags, 4);
    }

    wTemp = (WORD) pInitParam_p->m_uiIsochrTxMaxPayload;
    obd_writeEntry(0x1F98, 1, &wTemp, 2);

    wTemp = (WORD) pInitParam_p->m_uiIsochrRxMaxPayload;
    obd_writeEntry(0x1F98, 2, &wTemp, 2);

    obd_writeEntry(0x1F98, 3, &pInitParam_p->m_dwPresMaxLatency, 4);

    if (pInitParam_p->m_uiPreqActPayloadLimit <= EPL_C_DLL_ISOCHR_MAX_PAYL)
    {
        wTemp = (WORD) pInitParam_p->m_uiPreqActPayloadLimit;
        obd_writeEntry(0x1F98, 4, &wTemp, 2);
    }

    if (pInitParam_p->m_uiPresActPayloadLimit <= EPL_C_DLL_ISOCHR_MAX_PAYL)
    {
        wTemp = (WORD) pInitParam_p->m_uiPresActPayloadLimit;
        obd_writeEntry(0x1F98, 5, &wTemp, 2);
    }

    obd_writeEntry(0x1F98, 6, &pInitParam_p->m_dwAsndMaxLatency, 4);

    if (pInitParam_p->m_uiMultiplCycleCnt <= 0xFF)
    {
        bTemp = (BYTE) pInitParam_p->m_uiMultiplCycleCnt;
        obd_writeEntry(0x1F98, 7, &bTemp, 1);
    }

    if (pInitParam_p->m_uiAsyncMtu <= EPL_C_DLL_MAX_ASYNC_MTU)
    {
        wTemp = (WORD) pInitParam_p->m_uiAsyncMtu;
        obd_writeEntry(0x1F98, 8, &wTemp, 2);
    }

    if (pInitParam_p->m_uiPrescaler <= 1000)
    {
        wTemp = (WORD) pInitParam_p->m_uiPrescaler;
        obd_writeEntry(0x1F98, 9, &wTemp, 2);
    }

#if defined(CONFIG_INCLUDE_NMT_MN)
    if (pInitParam_p->m_dwWaitSocPreq != UINT_MAX)
    {
        obd_writeEntry(0x1F8A, 1, &pInitParam_p->m_dwWaitSocPreq, 4);
    }

    if ((pInitParam_p->m_dwAsyncSlotTimeout != 0) && (pInitParam_p->m_dwAsyncSlotTimeout != UINT_MAX))
    {
        obd_writeEntry(0x1F8A, 2, &pInitParam_p->m_dwAsyncSlotTimeout, 4);
    }
#endif

    // configure Identity
    if (pInitParam_p->m_dwDeviceType != UINT_MAX)
    {
        obd_writeEntry(0x1000, 0, &pInitParam_p->m_dwDeviceType, 4);
    }

    if (pInitParam_p->m_dwVendorId != UINT_MAX)
    {
        obd_writeEntry(0x1018, 1, &pInitParam_p->m_dwVendorId, 4);
    }

    if (pInitParam_p->m_dwProductCode != UINT_MAX)
    {
        obd_writeEntry(0x1018, 2, &pInitParam_p->m_dwProductCode, 4);
    }

    if (pInitParam_p->m_dwRevisionNumber != UINT_MAX)
    {
        obd_writeEntry(0x1018, 3, &pInitParam_p->m_dwRevisionNumber, 4);
    }

    if (pInitParam_p->m_dwSerialNumber != UINT_MAX)
    {
        obd_writeEntry(0x1018, 4, &pInitParam_p->m_dwSerialNumber, 4);
    }

    if (pInitParam_p->m_pszDevName != NULL)
    {
        // write Device Name (0x1008)
        obd_writeEntry (0x1008, 0, (void GENERIC*) pInitParam_p->m_pszDevName,
                          (tObdSize) strlen(pInitParam_p->m_pszDevName));
    }

    if (pInitParam_p->m_pszHwVersion != NULL)
    {
        // write Hardware version (0x1009)
        obd_writeEntry (0x1009, 0, (void GENERIC*) pInitParam_p->m_pszHwVersion,
                          (tObdSize) strlen(pInitParam_p->m_pszHwVersion));
    }

    if (pInitParam_p->m_pszSwVersion != NULL)
    {
        // write Software version (0x100A)
        obd_writeEntry (0x100A, 0, (void GENERIC*) pInitParam_p->m_pszSwVersion,
                          (tObdSize) strlen(pInitParam_p->m_pszSwVersion));
    }

#if defined(CONFIG_INCLUDE_VETH)
    // write NMT_HostName_VSTR (0x1F9A)
    obd_writeEntry (0x1F9A, 0, (void GENERIC*) &pInitParam_p->m_sHostname[0],
                      sizeof (pInitParam_p->m_sHostname));

    //TRACE("%s: write NMT_HostName_VSTR %d\n", __func__, Ret);

    // write NWL_IpAddrTable_Xh_REC.Addr_IPAD (0x1E40/2)
    obd_writeEntry (0x1E40, 2, (void GENERIC*) &pInitParam_p->m_dwIpAddress,
                      sizeof (pInitParam_p->m_dwIpAddress));

    // write NWL_IpAddrTable_Xh_REC.NetMask_IPAD (0x1E40/3)
    obd_writeEntry (0x1E40, 3, (void GENERIC*) &pInitParam_p->m_dwSubnetMask,
                      sizeof (pInitParam_p->m_dwSubnetMask));

    // write NWL_IpAddrTable_Xh_REC.DefaultGateway_IPAD (0x1E40/5)
    obd_writeEntry (0x1E40, 5, (void GENERIC*) &pInitParam_p->m_dwDefaultGateway,
                      sizeof (pInitParam_p->m_dwDefaultGateway));
#endif

    // ignore return code
    return kEplSuccessful;
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

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
static tEplKernel cbNodeEvent(UINT nodeId_p, tNmtNodeEvent nodeEvent_p, tNmtState nmtState_p,
                              UINT16 errorCode_p, BOOL fMandatory_p)
{
    tEplKernel              ret;
    tEplApiEventArg         eventArg;

    ret = kEplSuccessful;

    // call user callback
    eventArg.m_Node.m_uiNodeId = nodeId_p;
    eventArg.m_Node.m_NodeEvent = nodeEvent_p;
    eventArg.m_Node.m_NmtState = nmtState_p;
    eventArg.m_Node.m_wErrorCode = errorCode_p;
    eventArg.m_Node.m_fMandatory = fMandatory_p;

    ret = ctrlu_callUserEventCallback(kEplApiEventNode, &eventArg);
    if (ret != kEplSuccessful)
        return ret;

#if defined(CONFIG_INCLUDE_CFM)
    ret = cfmu_processNodeEvent(nodeId_p, nodeEvent_p);
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

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
static tEplKernel cbBootEvent(tNmtBootEvent bootEvent_p, tNmtState nmtState_p,
                                    UINT16 errorCode_p)
{
    tEplKernel              ret;
    tEplApiEventArg         eventArg;

    ret = kEplSuccessful;

    // call user callback
    eventArg.m_Boot.m_BootEvent = bootEvent_p;
    eventArg.m_Boot.m_NmtState = nmtState_p;
    eventArg.m_Boot.m_wErrorCode = errorCode_p;

    ret = ctrlu_callUserEventCallback(kEplApiEventBoot, &eventArg);
    return ret;
}

#if defined(CONFIG_INCLUDE_LEDU)
//------------------------------------------------------------------------------
/**
\brief  Callback function for LED change events

The function implements the callback function for LED change events.

\param  ledType_p       Type of LED.
\param  fOn_p           State of LED. TRUE = on, FALSE = off

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
static tEplKernel cbLedStateChange(tLedType ledType_p, BOOL fOn_p)
{
    tEplKernel              ret;
    tEplApiEventArg         eventArg;

    ret = kEplSuccessful;

    // call user callback
    eventArg.m_Led.m_LedType = ledType_p;
    eventArg.m_Led.m_fOn = fOn_p;

    ret = ctrlu_callUserEventCallback(kEplApiEventLed, &eventArg);

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

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
static tEplKernel cbCfmEventCnProgress(tCfmEventCnProgress* pEventCnProgress_p)
{
    tEplKernel              ret;
    tEplApiEventArg         eventArg;

    ret = kEplSuccessful;

    eventArg.m_CfmProgress = *pEventCnProgress_p;
    ret = ctrlu_callUserEventCallback(kEplApiEventCfmProgress, &eventArg);
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Callback function for CFM result events

The function implements the callback function for CFM result events.

\param  nodeId_p                Node ID of CN.
\param  nodeCommand_p           NMT command which shall be executed.

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
static tEplKernel cbCfmEventCnResult(UINT nodeId_p, tNmtNodeCommand nodeCommand_p)
{
    tEplKernel              ret;
    tEplApiEventArg         eventArg;

    eventArg.m_CfmResult.m_uiNodeId = nodeId_p;
    eventArg.m_CfmResult.m_NodeCommand = nodeCommand_p;
    ret = ctrlu_callUserEventCallback(kEplApiEventCfmResult, &eventArg);
    if (ret != kEplSuccessful)
    {
        if (ret == kEplReject)
            ret = kEplSuccessful;
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

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
static tEplKernel cbCnCheckEvent(tNmtEvent nmtEvent_p)
{
    tEplKernel              ret = kEplSuccessful;
    tNmtState               nmtState;

    switch (nmtEvent_p)
    {
        case kNmtEventEnableReadyToOperate:
            nmtState = nmtu_getNmtState();
            // inform application
            ret = cbBootEvent(kNmtBootEventEnableReadyToOp, nmtState, EPL_E_NO_ERROR);
            break;

        default:
            break;
    }
    return ret;
}



/// \}

