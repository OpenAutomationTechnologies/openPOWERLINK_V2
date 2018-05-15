/**
********************************************************************************
\file   nmtu.c

\brief  Implementation of NMT user module

This file contains the implementation of the NMT user module.

\ingroup module_nmtu
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2015, SYSTEC electronic GmbH
Copyright (c) 2016, B&R Industrial Automation GmbH
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
#include <user/nmtu.h>
#include <user/timeru.h>
#include <user/eventu.h>
#include <user/obdu.h>

#if (NMT_MAX_NODE_ID > 0)
#include <user/dllucal.h>
#endif

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
    tNmtState                   localNmtState;
    tNmtuStateChangeCallback    pfnNmtChangeCb;
    tTimerHdl                   timerHdl;
} tNmtuInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tNmtuInstance    nmtuInstance_g;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
#if NMT_MAX_NODE_ID > 0
static tOplkError configureDll(void);
#endif

static BOOL processGeneralStateChange(tNmtState newNmtState, tOplkError* pRet_p);
static BOOL processCnStateChange(tNmtState newNmtState, tOplkError* pRet_p);

#if defined(CONFIG_INCLUDE_NMT_MN)
static BOOL processMnStateChange(tNmtState newNmtState, tOplkError* pRet_p);
#endif

static tOplkError setupNmtTimerEvent(UINT32 timeout_p, tNmtEvent event_p);

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Init NMT user module

The function initializes an instance of the NMT user module

\return The function returns a tOplkError error code.

\ingroup module_nmtu
*/
//------------------------------------------------------------------------------
tOplkError nmtu_init(void)
{
    nmtuInstance_g.pfnNmtChangeCb = NULL;

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Shut down NMT user module instance

The function shuts down the NMT user module instance

\return The function returns a tOplkError error code.

\ingroup module_nmtu
*/
//------------------------------------------------------------------------------
tOplkError nmtu_exit(void)
{
    tOplkError  ret;

    nmtuInstance_g.pfnNmtChangeCb = NULL;
    ret = timeru_deleteTimer(&nmtuInstance_g.timerHdl);

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Post a NMT event

The function posts an NMT event for the NMT kernel module.

\param[in]      nmtEvent_p          NMT event to post.

\return The function returns a tOplkError error code.

\ingroup module_nmtu
*/
//------------------------------------------------------------------------------
tOplkError nmtu_postNmtEvent(tNmtEvent nmtEvent_p)
{
    tOplkError  ret;
    tEvent      event;

    event.eventSink = kEventSinkNmtk;
    event.netTime.nsec = 0;
    event.netTime.sec = 0;
    event.eventType = kEventTypeNmtEvent;
    event.eventArg.pEventArg = &nmtEvent_p;
    event.eventArgSize = sizeof(nmtEvent_p);

    ret = eventu_postEvent(&event);

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Get the NMT state

The function returns the current NMT state of the node.

\return The function returns the NMT state

\ingroup module_nmtu
*/
//------------------------------------------------------------------------------
tNmtState nmtu_getNmtState(void)
{
    return nmtuInstance_g.localNmtState;
}

//------------------------------------------------------------------------------
/**
\brief  Process an NMTu event

The function processes events sent to the NMT user module.

\param[in]      pEvent_p            Pointer to event which should be processed.

\return The function returns a tOplkError error code.

\ingroup module_nmtu
*/
//------------------------------------------------------------------------------
tOplkError nmtu_processEvent(const tEvent* pEvent_p)
{
    tOplkError              ret = kErrorOk;
    tEventNmtStateChange*   pNmtStateChange;

    switch (pEvent_p->eventType)
    {
        // state change of NMT-Module
        case kEventTypeNmtStateChange:
            ret = timeru_deleteTimer(&nmtuInstance_g.timerHdl);
            pNmtStateChange = (tEventNmtStateChange*)pEvent_p->eventArg.pEventArg;
            nmtuInstance_g.localNmtState = pNmtStateChange->newNmtState;

            // call cb-functions to inform higher layer
            if (nmtuInstance_g.pfnNmtChangeCb != NULL)
                ret = nmtuInstance_g.pfnNmtChangeCb(*pNmtStateChange);

            if (ret == kErrorOk)
            {
                /* handle state changes, state machine is split in general, CN and
                 * MN states. */
                if (!processGeneralStateChange(pNmtStateChange->newNmtState, &ret))
                {
                    if (!processCnStateChange(pNmtStateChange->newNmtState, &ret))
                    {
#if defined(CONFIG_INCLUDE_NMT_MN)
                        if (!processMnStateChange(pNmtStateChange->newNmtState, &ret))
#endif
                        {
                            ret = kErrorNmtInvalidState;
                            DEBUG_LVL_ERROR_TRACE("%s(): unhandled NMT state 0x%X\n",
                                                  __func__,
                                                  pNmtStateChange->newNmtState);
                        }
                    }
                }

            }
            else if (ret == kErrorReject)
            {   // application wants to change NMT state itself, it's OK
                ret = kErrorOk;
            }
            DEBUG_LVL_NMTU_TRACE("%s(): NMT state machine announce change of NMT state\n",
                                 __func__);
            break;

        default:
            ret = kErrorNmtInvalidEvent;
            break;
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Register an NMT state change callback

The function registers a callback function for NMT state change events.

\param[in]      pfnNmtStateChangeCb_p   Pointer to callback function

\return The function returns a tOplkError error code.

\ingroup module_nmtu
*/
//------------------------------------------------------------------------------
tOplkError nmtu_registerStateChangeCb(tNmtuStateChangeCallback pfnNmtStateChangeCb_p)
{
    nmtuInstance_g.pfnNmtChangeCb = pfnNmtStateChangeCb_p;

    return kErrorOk;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

#if (NMT_MAX_NODE_ID > 0)

//------------------------------------------------------------------------------
/**
\brief  Configure DLL limits and timeouts

The function configures PReq/PRes payload limits and PRes timeouts in the DLL
for each active node. The list of active nodes is read from object 0x1F81.

\return The function returns a tOplkError error code.

\ingroup module_nmtu
*/
//------------------------------------------------------------------------------
static tOplkError configureDll(void)
{
    tOplkError      ret;
    UINT32          nodeCfg;
    tObdSize        obdSize;
    tDllNodeInfo    dllNodeInfo;
    UINT            index;
    UINT8           count;

    // read number of nodes from object 0x1F81/0
    obdSize = sizeof(count);
    ret = obdu_readEntry(0x1F81, 0, &count, &obdSize);
    if ((ret == kErrorObdIndexNotExist) || (ret == kErrorObdSubindexNotExist))
        return kErrorOk;
    else if (ret != kErrorOk)
        return ret;

    for (index = 1; index <= count; index++)
    {
        obdSize = sizeof(nodeCfg);
        ret = obdu_readEntry(0x1F81, index, &nodeCfg, &obdSize);
        if (ret == kErrorObdSubindexNotExist)
        {   // not all subindexes of object 0x1F81 have to exist
            continue;
        }
        else if (ret != kErrorOk)
        {
            return ret;
        }

        if ((nodeCfg & (NMT_NODEASSIGN_NODE_EXISTS | NMT_NODEASSIGN_ASYNCONLY_NODE)) == NMT_NODEASSIGN_NODE_EXISTS)
        {   // node exists and runs in isochronous phase
            dllNodeInfo.nodeId = index;

            obdSize = sizeof(dllNodeInfo.presPayloadLimit);
            ret = obdu_readEntry(0x1F8D, index, &dllNodeInfo.presPayloadLimit, &obdSize);
            if ((ret == kErrorObdIndexNotExist) || (ret == kErrorObdSubindexNotExist))
                dllNodeInfo.presPayloadLimit = 0;
            else if (ret != kErrorOk)
                return ret;

#if defined(CONFIG_INCLUDE_NMT_MN)
            if ((nodeCfg & (NMT_NODEASSIGN_NODE_IS_CN | NMT_NODEASSIGN_PRES_CHAINING)) == NMT_NODEASSIGN_NODE_IS_CN)
            {   // node is CN
                obdSize = sizeof(dllNodeInfo.preqPayloadLimit);
                ret = obdu_readEntry(0x1F8B, index, &dllNodeInfo.preqPayloadLimit, &obdSize);
                if (ret != kErrorOk)
                    return ret;

                obdSize = sizeof(dllNodeInfo.presTimeoutNs);
                ret = obdu_readEntry(0x1F92, index, &dllNodeInfo.presTimeoutNs, &obdSize);
                if (ret != kErrorOk)
                    return ret;
            }
            else
            {
                dllNodeInfo.presTimeoutNs = 0;
                dllNodeInfo.preqPayloadLimit = 0;
            }
#endif // if defined(INCLUDE_CONFIG_NMT_MN)

            ret = dllucal_configNode(&dllNodeInfo);
            if (ret != kErrorOk)
                return ret;
        }
    }

    return ret;
}
#endif // NMT_MAX_NODE_ID > 0

//------------------------------------------------------------------------------
/**
\brief  Process a state change to a general NMT state

The function processes a state change to a general NMT state.

\param[in]      newNmtState_p       New NMT state.
\param[out]     pRet_p              Pointer to store tOplkError return value.

\return The function returns \b TRUE if a state was found or \b FALSE if the
        state was not found.
*/
//------------------------------------------------------------------------------
static BOOL processGeneralStateChange(tNmtState newNmtState_p, tOplkError* pRet_p)
{
    tOplkError  ret = kErrorOk;
    UINT        nodeId;
    BOOL        fHandled = TRUE;
#if defined(CONFIG_INCLUDE_NMT_RMN)
    UINT32      startUp;
    tObdSize    obdSize;
#endif

    // Check parameter
    switch (newNmtState_p)
    {
        // POWERLINK stack is not running
        case kNmtGsOff:
            break;

        // first init of the hardware
        case kNmtGsInitialising:
            ret = nmtu_postNmtEvent(kNmtEventEnterResetApp);
            break;

        // init of the manufacturer-specific profile area and the
        // standardised device profile area
        case kNmtGsResetApplication:
            ret = nmtu_postNmtEvent(kNmtEventEnterResetCom);
            break;

        // init of the communication profile area
        case kNmtGsResetCommunication:
            ret = nmtu_postNmtEvent(kNmtEventEnterResetConfig);
            break;

        // build the configuration with infos from OD
        case kNmtGsResetConfiguration:
#if (NMT_MAX_NODE_ID > 0)
            // configure the DLL (PReq/PRes payload limits and PRes timeout)
            ret = configureDll();
            if (ret != kErrorOk)
                break;
#endif // NMT_MAX_NODE_ID > 0

#if defined(CONFIG_INCLUDE_NMT_RMN)
            obdSize = sizeof(startUp);
            ret = obdu_readEntry(0x1F80, 0x00, &startUp, &obdSize);
            if (ret != kErrorOk)
                break;

            if ((startUp & NMT_STARTUP_REDUNDANCY) != 0)
            {   // NMT_StartUp_U32.Bit14 == 1
                ret = nmtu_postNmtEvent(kNmtEventEnterRmsNotActive);
                break;
            }
#endif
            // get node ID from OD
            nodeId = obdu_getNodeId();
            //check node ID if not should be master or slave
            if (nodeId == C_ADR_MN_DEF_NODE_ID)
            {   // node shall be MN
#if defined(CONFIG_INCLUDE_NMT_MN)
                ret = nmtu_postNmtEvent(kNmtEventEnterMsNotActive);
#else
                DEBUG_LVL_ERROR_TRACE("%s(): no MN functionality implemented\n", __func__);
#endif
            }
            else
            {   // node shall be CN
                ret = nmtu_postNmtEvent(kNmtEventEnterCsNotActive);
            }
            break;

        default:
            fHandled = FALSE;
            break;
    }

    *pRet_p = ret;
    return fHandled;
}

#if defined(CONFIG_INCLUDE_NMT_MN)
//------------------------------------------------------------------------------
/**
\brief  Process a state change to an MN NMT state

The function processes a state change to an MN NMT state.

\param[in]      newNmtState_p       New NMT state.
\param[out]     pRet_p              Pointer to store tOplkError return value.

\return The function returns \b TRUE if a state was found or \b FALSE if the
        state was not found.
*/
//------------------------------------------------------------------------------
static BOOL processMnStateChange(tNmtState newNmtState_p, tOplkError* pRet_p)
{
    tOplkError  ret = kErrorOk;
    BOOL        fHandled = TRUE;
    UINT32      waitTime;
    UINT32      startUp;
    tNmtEvent   timerEvent = kNmtEventTimerMsPreOp1;
    tObdSize    obdSize;

    switch (newNmtState_p)
    {
        // node listens for POWERLINK frames and check timeout
        case kNmtMsNotActive:
            // create timer to switch automatically to BasicEthernet/PreOp1 if no other MN active in network
            // check NMT_StartUp_U32.Bit13
            obdSize = sizeof(startUp);
            ret = obdu_readEntry(0x1F80, 0x00, &startUp, &obdSize);
            if (ret != kErrorOk)
                break;

            if ((startUp & NMT_STARTUP_BASICETHERNET) != 0)
            {   // NMT_StartUp_U32.Bit13 == 1 -> new state BasicEthernet
                timerEvent = kNmtEventTimerBasicEthernet;
            }

        // intentional fall through
        case kNmtRmsNotActive:
            // read NMT_BootTime_REC.MNWaitNotAct_U32 from OD
            obdSize = sizeof(waitTime);
            ret = obdu_readEntry(0x1F89, 0x01, &waitTime, &obdSize);
            if (ret != kErrorOk)
                break;

            ret = setupNmtTimerEvent(waitTime, timerEvent);
            // potential error is forwarded to event queue which generates error event
            break;

        // node processes only async frames
        case kNmtMsPreOperational1:
            // create timer to switch automatically to PreOp2 if MN identified all mandatory CNs

            // read NMT_BootTime_REC.MNWaitPreOp1_U32 from OD
            obdSize = sizeof(waitTime);
            ret = obdu_readEntry(0x1F89, 0x03, &waitTime, &obdSize);
            if (ret != kErrorOk)
            {
                // ignore error, because this timeout is optional
                waitTime = 0;
            }

            if (waitTime == 0)
            {   // delay is deactivated, immediately post timer event
                ret = nmtu_postNmtEvent(kNmtEventTimerMsPreOp2);
            }
            else
                ret = setupNmtTimerEvent(waitTime, kNmtEventTimerMsPreOp2);

            // potential error is forwarded to event queue which generates error event
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

        default:
            fHandled = FALSE;
            break;
    }

    *pRet_p = ret;
    return fHandled;
}
#endif // #if defined(CONFIG_INCLUDE_NMT_MN)

//------------------------------------------------------------------------------
/**
\brief  Process a state change to a CN NMT state

The function processes a state change to a CN NMT state.

\param[in]      newNmtState_p       New NMT state.
\param[out]     pRet_p              Pointer to store tOplkError return value.

\return The function returns \b TRUE if a state was found or \b FALSE if the
        state was not found.
*/
//------------------------------------------------------------------------------
static BOOL processCnStateChange(tNmtState newNmtState_p, tOplkError* pRet_p)
{
    tOplkError  ret = kErrorOk;
    BOOL        fHandled = TRUE;
    UINT32      basicEthernetTimeout;
    tObdSize    obdSize;

    switch (newNmtState_p)
    {
        // node listens for POWERLINK frames and check timeout
        case kNmtCsNotActive:
            // create timer to switch automatically to BasicEthernet if no MN
            // is available in the network
            // read NMT_CNBasicEthernetTimeout_U32 from OD
            obdSize = sizeof(basicEthernetTimeout);
            ret = obdu_readEntry(0x1F99, 0x00, &basicEthernetTimeout, &obdSize);
            if (ret != kErrorOk)
                break;

            if (basicEthernetTimeout != 0)
            {   // BasicEthernet is enabled
                ret = setupNmtTimerEvent(basicEthernetTimeout, kNmtEventTimerBasicEthernet);
                // potential error is forwarded to event queue which generates error event
            }
            break;

        // node processes only async frames
        case kNmtCsPreOperational1:
            break;

        // node processes isochronous and asynchronous frames
        case kNmtCsPreOperational2:
            ret = nmtu_postNmtEvent(kNmtEventEnterReadyToOperate);
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

        default:
            fHandled = FALSE;
            break;
    }

    *pRet_p = ret;
    return fHandled;
}

//------------------------------------------------------------------------------
/**
\brief  Setup an NMT timer event

The function sets up a timer which posts an NMT Event.

\param[in]      timeout_p           Timeout to set in microseconds.
\param[in]      event_p             Event to post after timeout.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError setupNmtTimerEvent(UINT32 timeout_p, tNmtEvent event_p)
{
    tOplkError  ret;
    tTimerArg   timerArg;

    // convert us into ms (ceil div)
    timeout_p = (timeout_p % 1000) ?
                (timeout_p / 1000 + 1) : (timeout_p / 1000);

    // timer was 0 -> set one ms
    if (timeout_p == 0)
        timeout_p = 1;

    timerArg.eventSink = kEventSinkNmtk;
    timerArg.argument.value = (UINT32)event_p;
    ret = timeru_modifyTimer(&nmtuInstance_g.timerHdl, (ULONG)timeout_p, &timerArg);

    return ret;
}

/// \}
