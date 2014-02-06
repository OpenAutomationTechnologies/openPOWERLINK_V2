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
#include <oplk/oplk.h>
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
#include <user/sdocom.h>
#include <user/identu.h>
#include <user/statusu.h>
#include <user/timeru.h>
#include <user/cfmu.h>
#include <user/eventucal.h>

#include <common/ctrl.h>
#include <oplk/obd.h>
#include <common/target.h>

#include <user/ctrlucal.h>

#if (CONFIG_OBD_USE_LOAD_CONCISEDCF != FALSE)
#include <oplk/obdcdc.h>
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
    tOplkApiInitParam    initParam;              ///< Stack initialization parameters
} tCtrluInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tCtrluInstance       ctrlInstance_l;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static tOplkError initNmtu(tOplkApiInitParam* pInitParam_p);
static tOplkError initObd(tOplkApiInitParam* pInitParam_p);
static tOplkError updateDllConfig(tOplkApiInitParam* pInitParam_p, BOOL fUpdateIdentity_p);
static tOplkError updateSdoConfig();
static tOplkError updateObd(tOplkApiInitParam* pInitParam_p);

static tOplkError processUserEvent(tEplEvent* pEplEvent_p);
static tOplkError cbCnCheckEvent(tNmtEvent NmtEvent_p);
static tOplkError cbNmtStateChange(tEventNmtStateChange nmtStateChange_p);

#if defined(CONFIG_INCLUDE_NMT_MN)
static tOplkError cbNodeEvent(UINT nodeId_p, tNmtNodeEvent nodeEvent_p,
                                    tNmtState nmtState_p, UINT16 errorCode_p,
                                    BOOL fMandatory_p);
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

    TRACE ("Initialize ctrl module ...\n");

    ctrlInstance_l.lastHeartbeat = 0;

    if ((ret = ctrlucal_init()) != kErrorOk)
    {
        DEBUG_LVL_ERROR_TRACE ("Could not initialize ctrlucal\n");
        goto Exit;
    }

    if ((ret = ctrlucal_checkKernelStack()) != kErrorOk)
    {
        ctrlucal_exit();
        goto Exit;
    }
    return kErrorOk;

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

After return from this function the application must start the NMT state
machine via EplApiExecNmtCommand(kNmtEventSwReset) and thereby the whole
openPOWERLINK stack.

\param  pInitParam_p            Pointer to the initialization parameters
                                provided by the application.

\return The function returns a tOplkError error code.

\ingroup module_ctrlu
*/
//------------------------------------------------------------------------------
tOplkError ctrlu_initStack(tOplkApiInitParam * pInitParam_p)
{
    tOplkError              ret = kErrorOk;
    tCtrlInitParam          ctrlParam;

    // reset instance structure
    EPL_MEMSET(&ctrlInstance_l.initParam, 0, sizeof (tOplkApiInitParam));
    EPL_MEMCPY(&ctrlInstance_l.initParam, pInitParam_p,
               min(sizeof(tOplkApiInitParam), (size_t)pInitParam_p->m_uiSizeOfStruct));

    // check event callback function pointer
    if (ctrlInstance_l.initParam.m_pfnCbEvent == NULL)
    {   // application must always have an event callback function
        ret = kErrorApiInvalidParam;
        goto Exit;
    }

    if ((ret = initObd(&ctrlInstance_l.initParam)) != kErrorOk)
        goto Exit;

    TRACE ("Initializing kernel modules ...\n");
    EPL_MEMCPY (ctrlParam.aMacAddress, ctrlInstance_l.initParam.m_abMacAddress, 6);
    strncpy(ctrlParam.szEthDevName, ctrlInstance_l.initParam.m_HwParam.m_pszDevName, 127);
    ctrlParam.ethDevNumber = ctrlInstance_l.initParam.m_HwParam.m_uiDevNumber;
    ctrlucal_storeInitParam(&ctrlParam);

    if ((ret = ctrlucal_executeCmd(kCtrlInitStack)) != kErrorOk)
        goto Exit;

    /* Read back init param because current MAC address was copied by DLLK */
    ret = ctrlucal_readInitParam(&ctrlParam);
    if (ret != kErrorOk)
    {
        goto Exit;
    }

    EPL_MEMCPY (ctrlInstance_l.initParam.m_abMacAddress, ctrlParam.aMacAddress, 6);

    TRACE ("Initialize Eventu module...\n");
    if ((ret = eventu_init(processUserEvent)) != kErrorOk)
        goto Exit;

    TRACE ("Initialize Timeru module...\n");
    if ((ret = timeru_init()) != kErrorOk)
        goto Exit;

    TRACE ("initialize error handler user module...\n");
    ret = errhndu_init();
    if (ret != kErrorOk)
    {
        goto Exit;
    }

    TRACE ("Initialize DlluCal module...\n");
    ret = dllucal_init();
    if (ret != kErrorOk)
    {
        goto Exit;
    }

#if defined(CONFIG_INCLUDE_PDO)
    TRACE ("Initialize Pdou module...\n");
    ret = pdou_init(ctrlInstance_l.initParam.m_pfnCbSync);
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
    TRACE ("Initialize SdoCom module...\n");
    ret = sdocom_init();
    if (ret != kErrorOk)
    {
        goto Exit;
    }
#endif

#if defined (CONFIG_INCLUDE_CFM)
    TRACE ("Initialize Cfm module...\n");
    ret = cfmu_init(cbCfmEventCnProgress, cbCfmEventCnResult);
    if (ret != kErrorOk)
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

\return The function returns a tOplkError error code.

\ingroup module_ctrlu
*/
//------------------------------------------------------------------------------
tOplkError ctrlu_shutdownStack(void)
{
    tOplkError      ret = kErrorOk;

    ret = eventu_exit();
    TRACE("eventu_exit():  0x%X\n", ret);

#if defined(CONFIG_INCLUDE_CFM)
    ret = cfmu_exit();
    TRACE("cfmu_exit():    0x%X\n", ret);
#endif

#if defined(CONFIG_INCLUDE_SDOS) || defined(CONFIG_INCLUDE_SDOC)
    ret = sdocom_delInstance();
    TRACE("sdocom_delInstance():  0x%X\n", ret);
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

    ret = nmtcnu_delInstance();
    TRACE("EplNmtCnuDelInstance():  0x%X\n", ret);

    ret = nmtu_delInstance();
    TRACE("nmtu_delInstance():    0x%X\n", ret);

#if defined(CONFIG_INCLUDE_PDO)
    ret = pdou_exit();
    TRACE("pdou_exit():    0x%X\n", ret);
#endif

    ret = dllucal_exit();
    TRACE("dllucal_exit(): 0x%X\n", ret);

    ret = errhndu_exit();
    TRACE("errhndu_exit():  0x%X\n", ret);

    ret = timeru_delInstance();
    TRACE("timeru_delInstance():  0x%X\n", ret);

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

\return The function returns a tOplkError error code.

\ingroup module_ctrlu
*/
//------------------------------------------------------------------------------
tOplkError ctrlu_processStack(void)
{
    tOplkError Ret = kErrorOk;

    eventucal_process();

    Ret = ctrlucal_process();
    if(Ret != kErrorOk)
        goto Exit;

    Ret = timeru_process();

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

\return The function returns a tOplkError error code.

\ingroup module_ctrlu
*/
//------------------------------------------------------------------------------
tOplkError ctrlu_callUserEventCallback(tOplkApiEventType eventType_p, tOplkApiEventArg* pEventArg_p)
{
    tOplkError          ret = kErrorOk;

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

\return The function returns a tOplkError error code.

\ingroup module_ctrlu
*/
//------------------------------------------------------------------------------
tOplkError ctrlu_cbObdAccess(tObdCbParam MEM* pParam_p)
{
    tOplkError          ret = kErrorOk;

#if (EPL_API_OBD_FORWARD_EVENT != FALSE)
    tOplkApiEventArg     eventArg;

    // call user callback
    // must be disabled for EplApiLinuxKernel.c, because of reentrancy problem
    // for local OD access. This is not so bad as user callback function in
    // application does not use OD callbacks at the moment.
    eventArg.m_ObdCbParam = *pParam_p;
    ret = ctrlu_callUserEventCallback(kOplkApiEventObdAccess, &eventArg);
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
                        pParam_p->abortCode = SDO_AC_VALUE_RANGE_EXCEEDED;
                        ret = kErrorObdAccessViolation;
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
    tOplkError      Ret = kErrorOk;

    // initialize EplNmtCnu module
    TRACE ("Initialize NMT_CN module...\n");
    Ret = nmtcnu_addInstance(pInitParam_p->m_uiNodeId);
    if (Ret != kErrorOk)
        goto Exit;

    Ret = nmtcnu_registerCheckEventCb(cbCnCheckEvent);
    if (Ret != kErrorOk)
        goto Exit;

    // initialize EplNmtu module
    TRACE ("Initialize NMTu module...\n");
    Ret = nmtu_init();
    if (Ret != kErrorOk)
        goto Exit;

    // register NMT event callback function
    Ret = nmtu_registerStateChangeCb(cbNmtStateChange);
    if (Ret != kErrorOk)
        goto Exit;

#if defined(CONFIG_INCLUDE_NMT_MN)
    // initialize EplNmtMnu module
    TRACE ("Initialize NMT_MN module...\n");
    Ret = nmtmnu_init(cbNodeEvent, cbBootEvent);
    if (Ret != kErrorOk)
        goto Exit;

    // initialize identu module
    TRACE ("Initialize Identu module...\n");
    Ret = identu_init();
    if (Ret != kErrorOk)
        goto Exit;

    // initialize EplStatusu module
    TRACE ("Initialize Statusu module...\n");
    Ret = statusu_init();
    if (Ret != kErrorOk)
        goto Exit;

#if EPL_NMTMNU_PRES_CHAINING_MN != FALSE
    // initialize syncu module
    TRACE ("Initialize Syncu module...\n");
    Ret = syncu_init();
    if (Ret != kErrorOk)
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

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError initObd(tOplkApiInitParam* pInitParam_p)
{
    tOplkError          ret = kErrorOk;
    tObdInitParam       ObdInitParam;

    UNUSED_PARAMETER(pInitParam_p);

    TRACE ("Initialize OBD module...\n");
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
    tOplkApiEventArg     eventArg;

    // save NMT state in OD
    nmtState = (UINT8) nmtStateChange_p.newNmtState;
    ret = obd_writeEntry(0x1F8C, 0, &nmtState, 1);
    if(ret != kErrorOk)
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

            ret = updateSdoConfig();
            if (ret != kErrorOk)
                return ret;

            break;

        //-----------------------------------------------------------
        // CN part of the state machine

        // node list for EPL-Frames and check timeout
        case kNmtCsNotActive:
            // indicate completion of reset in NMT_ResetCmd_U8
            nmtState = (UINT8) kNmtCmdInvalidService;
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
    eventArg.m_NmtStateChange = nmtStateChange_p;
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
static tOplkError processUserEvent(tEplEvent* pEvent_p)
{
    tOplkError          ret;
    tEplEventError*     pEventError;
    tOplkApiEventType    eventType;
    tOplkApiEventArg     apiEventArg;

    ret = kErrorOk;

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
        case kEplEventTypeHistoryEntry:
            if (pEvent_p->m_uiSize != sizeof(tErrHistoryEntry))
            {
                ret = kErrorEventWrongSize;
                break;
            }
            eventType = kOplkApiEventHistoryEntry;
            ret = ctrlu_callUserEventCallback(eventType, (tOplkApiEventArg*)pEvent_p->m_pArg);
            break;

        // user-defined event
        case kEplEventTypeApiUserDef:
            eventType = kOplkApiEventUserDef;
            apiEventArg.m_pUserArg = *(void**)pEvent_p->m_pArg;
            ret = ctrlu_callUserEventCallback(eventType, &apiEventArg);
            break;

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
    EPL_MEMSET(&dllConfigParam, 0, sizeof(dllConfigParam));
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
    dllConfigParam.fAsyncOnly = pInitParam_p->m_fAsyncOnly;

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

    // $$$ Prescaler

#if defined(CONFIG_INCLUDE_NMT_MN)
    // 0x1F8A.1: WaitSoCPReq_U32 in [ns]
    obdSize = 4;
    if ((ret = obd_readEntry(0x1F8A, 1, &dllConfigParam.waitSocPreq, &obdSize)) != kErrorOk)
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
    if ((ret = dllucal_config(&dllConfigParam)) != kErrorOk)
        return ret;

    if (fUpdateIdentity_p != FALSE)
    {
        // configure Identity
        EPL_MEMSET(&dllIdentParam, 0, sizeof (dllIdentParam));

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

        dllIdentParam.ipAddress = pInitParam_p->m_dwIpAddress;
        dllIdentParam.subnetMask = pInitParam_p->m_dwSubnetMask;

        obdSize = sizeof (dllIdentParam.defaultGateway);
        ret = obd_readEntry(0x1E40, 5, &dllIdentParam.defaultGateway, &obdSize);
        if (ret != kErrorOk)
        {   // NWL_IpAddrTable_Xh_REC.DefaultGateway_IPAD seams to not exist,
            // so use the one supplied in the init parameter
            dllIdentParam.defaultGateway = pInitParam_p->m_dwDefaultGateway;
        }

#if defined(CONFIG_INCLUDE_VETH)
        // configure Virtual Ethernet Driver
        ret = target_setIpAdrs(EPL_VETH_NAME, dllIdentParam.ipAddress, dllIdentParam.subnetMask, (UINT16)dllConfigParam.asyncMtu);
        if(ret != kErrorOk)
            return ret;

        ret = target_setDefaultGateway(dllIdentParam.defaultGateway);
        if(ret != kErrorOk)
            return ret;
#endif

        obdSize = sizeof (dllIdentParam.sHostname);
        if ((ret = obd_readEntry(0x1F9A, 0, &dllIdentParam.sHostname[0], &obdSize)) != kErrorOk)
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
        if ((ret = dllucal_setIdentity(&dllIdentParam)) != kErrorOk)
            return ret;
    }
    return ret;
}

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
    if(ret != kErrorOk)
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

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError updateObd(tOplkApiInitParam* pInitParam_p)
{
    tOplkError          ret = kErrorOk;
    WORD                wTemp;
    BYTE                bTemp;

    // set node id in OD
    ret = obd_setNodeId(pInitParam_p->m_uiNodeId,    // node id
                            kObdNodeIdHardware); // set by hardware
    if (ret != kErrorOk)
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
    tOplkApiEventArg         eventArg;

    ret = kErrorOk;

    // call user callback
    eventArg.m_Node.nodeId = nodeId_p;
    eventArg.m_Node.nodeEvent = nodeEvent_p;
    eventArg.m_Node.nmtState = nmtState_p;
    eventArg.m_Node.errorCode = errorCode_p;
    eventArg.m_Node.fMandatory = fMandatory_p;

    ret = ctrlu_callUserEventCallback(kOplkApiEventNode, &eventArg);
    if (ret != kErrorOk)
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

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError cbBootEvent(tNmtBootEvent bootEvent_p, tNmtState nmtState_p,
                                    UINT16 errorCode_p)
{
    tOplkError              ret;
    tOplkApiEventArg         eventArg;

    ret = kErrorOk;

    // call user callback
    eventArg.m_Boot.m_BootEvent = bootEvent_p;
    eventArg.m_Boot.m_NmtState = nmtState_p;
    eventArg.m_Boot.m_wErrorCode = errorCode_p;

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
    tOplkApiEventArg         eventArg;

    ret = kErrorOk;

    // call user callback
    eventArg.m_Led.m_LedType = ledType_p;
    eventArg.m_Led.m_fOn = fOn_p;

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
    tOplkApiEventArg         eventArg;

    ret = kErrorOk;

    eventArg.m_CfmProgress = *pEventCnProgress_p;
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
    tOplkApiEventArg         eventArg;

    eventArg.m_CfmResult.m_uiNodeId = nodeId_p;
    eventArg.m_CfmResult.m_NodeCommand = nodeCommand_p;
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
            ret = cbBootEvent(kNmtBootEventEnableReadyToOp, nmtState, EPL_E_NO_ERROR);
            break;

        default:
            break;
    }
    return ret;
}



/// \}

