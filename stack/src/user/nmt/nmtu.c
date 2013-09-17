/**
********************************************************************************
\file   nmtu.c

\brief  Implementation of NMT user module

This file contains the implementation of the NMT user module.

\ingroup module_nmtu
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2013, SYSTEC electronic GmbH
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
#include "EplInc.h"
#include "obd.h"
#include "user/nmtu.h"
#include "user/EplTimeru.h"
#include "user/dllucal.h"

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
    tNmtState                       localNmtState;
    tNmtuStateChangeCallback        pfnNmtChangeCb;
    tEplTimerHdl                    timerHdl;
} tNmtuInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tNmtuInstance        nmtuInstance_g;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
#if EPL_NMT_MAX_NODE_ID > 0
static tEplKernel configureDll(void);
#endif

static BOOL processGeneralStateChange(tNmtState newNmtState, tEplKernel* pRet_p);
static BOOL processCnStateChange(tNmtState newNmtState, tEplKernel* pRet_p);

#if defined(CONFIG_INCLUDE_NMT_MN)
static BOOL processMnStateChange(tNmtState newNmtState, tEplKernel* pRet_p);
#endif

static tEplKernel setupNmtTimerEvent(UINT32 timeout_p, tNmtEvent event_p);

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Init NMT user module

The function initializes an instance of the NMT user module

\return The function returns a tEplKernel error code.

\ingroup module_nmtu
*/
//------------------------------------------------------------------------------
tEplKernel nmtu_init(void)
{
    return nmtu_addInstance();
}

//------------------------------------------------------------------------------
/**
\brief  Add NMT user module instance

The function adds a NMT user module instance

\return The function returns a tEplKernel error code.

\ingroup module_nmtu
*/
//------------------------------------------------------------------------------
tEplKernel nmtu_addInstance(void)
{
    nmtuInstance_g.pfnNmtChangeCb = NULL;
    return kEplSuccessful;
}

//------------------------------------------------------------------------------
/**
\brief  Delete NMT user module instance

The function deletes a NMT user module instance

\return The function returns a tEplKernel error code.

\ingroup module_nmtu
*/
//------------------------------------------------------------------------------
tEplKernel nmtu_delInstance(void)
{
    tEplKernel ret = kEplSuccessful;

    nmtuInstance_g.pfnNmtChangeCb = NULL;
    ret = EplTimeruDeleteTimer(&nmtuInstance_g.timerHdl);

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Post a NMT event

The function posts a NMT event for the NMT kernel module.

\param  nmtEvent_p      NMT event to post.

\return The function returns a tEplKernel error code.

\ingroup module_nmtu
*/
//------------------------------------------------------------------------------
tEplKernel nmtu_postNmtEvent(tNmtEvent nmtEvent_p)
{
    tEplKernel  ret;
    tEplEvent   event;

    event.m_EventSink = kEplEventSinkNmtk;
    event.m_NetTime.m_dwNanoSec = 0;
    event.m_NetTime.m_dwSec = 0;
    event.m_EventType = kEplEventTypeNmtEvent;
    event.m_pArg = &nmtEvent_p;
    event.m_uiSize = sizeof(nmtEvent_p);

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
\brief  Process a NMTu event

The function processes events sent to the NMT user module.

\param  pEvent_p            Pointer to event which should be processed.

\return The function returns a tEplKernel error code.

\ingroup module_nmtu
*/
//------------------------------------------------------------------------------
tEplKernel nmtu_processEvent(tEplEvent* pEvent_p)
{
    tEplKernel                  ret = kEplSuccessful;
    tEventNmtStateChange*       pNmtStateChange;

    switch(pEvent_p->m_EventType)
    {
        // state change of NMT-Module
        case kEplEventTypeNmtStateChange:
            ret = EplTimeruDeleteTimer(&nmtuInstance_g.timerHdl);
            pNmtStateChange = (tEventNmtStateChange*)pEvent_p->m_pArg;
            nmtuInstance_g.localNmtState = pNmtStateChange->newNmtState;

            // call cb-functions to inform higher layer
            if(nmtuInstance_g.pfnNmtChangeCb != NULL)
            {
                ret = nmtuInstance_g.pfnNmtChangeCb(*pNmtStateChange);
            }

            if (ret == kEplSuccessful)
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
                            ret = kEplNmtInvalidState;
                            TRACE("EplNmtuProcess(): unhandled NMT state 0x%X\n",
                                  pNmtStateChange->newNmtState);
                        }
                    }
                }

            }
            else if (ret == kEplReject)
            {   // application wants to change NMT state itself, it's OK
                ret = kEplSuccessful;
            }
            EPL_DBGLVL_NMTU_TRACE("EplNmtuProcessEvent(): NMT-State-Maschine announce change of NMT State\n");
            break;

        default:
            ret = kEplNmtInvalidEvent;
            break;
    }
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Register a NMT state change callback

The function registers a callback function for NMT state change events.

\param  pfnNmtStateChangeCb_p      Pointer to callback function

\return The function returns a tEplKernel error code.

\ingroup module_nmtu
*/
//------------------------------------------------------------------------------
tEplKernel nmtu_registerStateChangeCb(tNmtuStateChangeCallback pfnNmtStateChangeCb_p)
{
    nmtuInstance_g.pfnNmtChangeCb = pfnNmtStateChangeCb_p;
    return kEplSuccessful;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

#if EPL_NMT_MAX_NODE_ID > 0

//------------------------------------------------------------------------------
/**
\brief  Configure DLL limits and timeouts

The function configured PReq/PRes payload limits an PRes timeouts in the DLL
for each active node in object 0x1F81.

\return The function returns a tEplKernel error code.

\ingroup module_nmtu
*/
//------------------------------------------------------------------------------
static tEplKernel configureDll(void)
{
    tEplKernel      ret = kEplSuccessful;
    UINT32          nodeCfg;
    tEplObdSize     obdSize;
    tDllNodeInfo    dllNodeInfo;
    UINT            index;
    UINT8           count;

    // read number of nodes from object 0x1F81/0
    obdSize = sizeof (count);
    ret = EplObdReadEntry(0x1F81, 0, &count, &obdSize);
    if ((ret == kEplObdIndexNotExist) || (ret == kEplObdSubindexNotExist))
    {
        return kEplSuccessful;
    }
    else if (ret != kEplSuccessful)
    {
        return ret;
    }

    for (index = 1; index <= count; index++)
    {
        obdSize = sizeof (nodeCfg);
        ret = EplObdReadEntry(0x1F81, index, &nodeCfg, &obdSize);
        if (ret == kEplObdSubindexNotExist)
        {   // not all subindexes of object 0x1F81 have to exist
            continue;
        }
        else if (ret != kEplSuccessful)
        {
            return ret;
        }

        if ((nodeCfg & (EPL_NODEASSIGN_NODE_EXISTS | EPL_NODEASSIGN_ASYNCONLY_NODE)) == EPL_NODEASSIGN_NODE_EXISTS)
        {   // node exists and runs in isochronous phase
            dllNodeInfo.nodeId = index;

            obdSize = sizeof (dllNodeInfo.presPayloadLimit);
            ret = EplObdReadEntry(0x1F8D, index, &dllNodeInfo.presPayloadLimit, &obdSize);
            if ((ret == kEplObdIndexNotExist) || (ret == kEplObdSubindexNotExist))
            {
                dllNodeInfo.presPayloadLimit = 0;
            }
            else if (ret != kEplSuccessful)
            {
                return ret;
            }

#if defined(CONFIG_INCLUDE_NMT_MN)
            if ((nodeCfg & (EPL_NODEASSIGN_NODE_IS_CN
#if EPL_NMTMNU_PRES_CHAINING_MN != FALSE
                            | EPL_NODEASSIGN_PRES_CHAINING
#endif
                    )) == EPL_NODEASSIGN_NODE_IS_CN)
            {   // node is CN
                obdSize = sizeof (dllNodeInfo.preqPayloadLimit);
                ret = EplObdReadEntry(0x1F8B, index, &dllNodeInfo.preqPayloadLimit, &obdSize);
                if (ret != kEplSuccessful)
                    return ret;

                obdSize = sizeof (dllNodeInfo.presTimeoutNs);
                ret = EplObdReadEntry(0x1F92, index, &dllNodeInfo.presTimeoutNs, &obdSize);
                if (ret != kEplSuccessful)
                    return ret;
            }
            else
            {
                dllNodeInfo.presTimeoutNs = 0;
                dllNodeInfo.preqPayloadLimit = 0;
            }
#endif // if defined(INCLUDE_CONFIG_NMT_MN)

            ret = dllucal_configNode(&dllNodeInfo);
            if (ret != kEplSuccessful)
                return ret;
        }
    }

    return ret;
}
#endif // EPL_NMT_MAX_NODE_ID > 0

//------------------------------------------------------------------------------
/**
\brief  Process a state change to a general NMT state

The function processes a state change to a general NMT state.

\param  newNmtState_p           New NMT state.
\param  pRet_p                  Pointer to store tEplKernel return value.

\return The function returns \b TRUE if a state was found or \b FALSE if the
        state was not found.
*/
//------------------------------------------------------------------------------
static BOOL processGeneralStateChange(tNmtState newNmtState_p, tEplKernel* pRet_p)
{
    tEplKernel          ret = kEplSuccessful;
    UINT                nodeId;
    BOOL                fHandled = TRUE;

    switch (newNmtState_p)
    {
        // EPL stack is not running
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
#if EPL_NMT_MAX_NODE_ID > 0
            // configure the DLL (PReq/PRes payload limits and PRes timeout)
            ret = configureDll();
            if (ret != kEplSuccessful)
            {
                break;
            }
#endif // EPL_NMT_MAX_NODE_ID > 0

            // get node ID from OD
#if defined(CONFIG_INCLUDE_OBD)
            nodeId = EplObdGetNodeId();
#else
            nodeId = 0;
#endif
            //check node ID if not should be master or slave
            if (nodeId == EPL_C_ADR_MN_DEF_NODE_ID)
            {   // node shall be MN
#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)
                ret = nmtu_postNmtEvent(kNmtEventEnterMsNotActive);
#else
                TRACE("EplNmtuProcess(): no MN functionality implemented\n");
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
\brief  Process a state change to a MN NMT state

The function processes a state change to a MN NMT state.

\param  newNmtState_p           New NMT state.
\param  pRet_p                  Pointer to store tEplKernel return value.

\return The function returns \b TRUE if a state was found or \b FALSE if the
        state was not found.
*/
//------------------------------------------------------------------------------
static BOOL processMnStateChange(tNmtState newNmtState_p, tEplKernel* pRet_p)
{
    tEplKernel          ret = kEplSuccessful;
    BOOL                fHandled = TRUE;
    UINT32              waitTime;
    UINT32              startUp;
    tNmtEvent           timerEvent;
    tEplObdSize         obdSize;

    switch (newNmtState_p)
    {
        // node listens for EPL-Frames and check timeout
        case kNmtMsNotActive:
            // create timer to switch automatically to BasicEthernet/PreOp1 if no other MN active in network
            // check NMT_StartUp_U32.Bit13
            obdSize = sizeof(startUp);
#if defined(CONFIG_INCLUDE_OBD)
            ret = EplObdReadEntry(0x1F80, 0x00, &startUp,&obdSize);
#else
            ret = kEplObdIndexNotExist;
#endif
            if(ret != kEplSuccessful)
                break;

            if((startUp & EPL_NMTST_BASICETHERNET) == 0)
            {   // NMT_StartUp_U32.Bit13 == 0 -> new state PreOperational1
                timerEvent = kNmtEventTimerMsPreOp1;
            }
            else
            {   // NMT_StartUp_U32.Bit13 == 1 -> new state BasicEthernet
                timerEvent = kNmtEventTimerBasicEthernet;
            }

            // read NMT_BootTime_REC.MNWaitNotAct_U32 from OD
            obdSize = sizeof(waitTime);
#if defined(CONFIG_INCLUDE_OBD)
            ret = EplObdReadEntry(0x1F89, 0x01, &waitTime, &obdSize);
#else
            ret = kEplObdIndexNotExist;
#endif
            if (ret != kEplSuccessful)
                break;

            ret = setupNmtTimerEvent(waitTime, timerEvent);
            // potential error is forwarded to event queue which generates error event
            break;

        // node processes only async frames
        case kNmtMsPreOperational1:
            // create timer to switch automatically to PreOp2 if MN identified all mandatory CNs

            // read NMT_BootTime_REC.MNWaitPreOp1_U32 from OD
            obdSize = sizeof(waitTime);
#if defined(CONFIG_INCLUDE_OBD)
            ret = EplObdReadEntry(0x1F89, 0x03, &waitTime, &obdSize);
            if(ret != kEplSuccessful)
            {
                // ignore error, because this timeout is optional
                waitTime = 0;
            }
#endif
            if (waitTime == 0)
            {   // delay is deactivated, immediately post timer event
                ret = nmtu_postNmtEvent(kNmtEventTimerMsPreOp2);
            }
            else
            {
                ret = setupNmtTimerEvent(waitTime, kNmtEventTimerMsPreOp2);
            }
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

        // no EPL cycle
        // -> normal ethernet communication
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

\param  newNmtState_p           New NMT state.
\param  pRet_p                  Pointer to store tEplKernel return value.

\return The function returns \b TRUE if a state was found or \b FALSE if the
        state was not found.
*/
//------------------------------------------------------------------------------
static BOOL processCnStateChange(tNmtState newNmtState_p, tEplKernel* pRet_p)
{
    tEplKernel          ret = kEplSuccessful;
    BOOL                fHandled = TRUE;
    UINT32              basicEthernetTimeout;
    tEplObdSize         obdSize;

    switch (newNmtState_p)
    {
        // node listens for EPL-Frames and check timeout
        case kNmtCsNotActive:
            // create timer to switch automatically to BasicEthernet if no MN available in network
            // read NMT_CNBasicEthernetTimeout_U32 from OD
            obdSize = sizeof(basicEthernetTimeout);
#if defined(CONFIG_INCLUDE_OBD)
            ret = EplObdReadEntry(0x1F99, 0x00, &basicEthernetTimeout, &obdSize);
#else
            ret = kEplObdIndexNotExist;
#endif
            if (ret != kEplSuccessful)
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

        // no EPL cycle
        // -> normal ethernet communication
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
\brief  Setup a NMT timer event

The function sets up a timer which posts a NMT Event.

\param  timeout_p           Timeout to set in microseconds.
\param  event_p             Event to post after timeout.

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
static tEplKernel setupNmtTimerEvent(UINT32 timeout_p, tNmtEvent event_p)
{
    tEplKernel      ret;
    tEplTimerArg    timerArg;

    timeout_p = timeout_p / 1000; // convert us into ms
    if (timeout_p == 0)  // timer was below one ms -> set one ms
        timeout_p = 1;
    timerArg.m_EventSink = kEplEventSinkNmtk;
    timerArg.m_Arg.m_dwVal = (UINT32) event_p;
    ret = EplTimeruModifyTimerMs(&nmtuInstance_g.timerHdl, (ULONG)timeout_p, timerArg);
    return  ret;
}

///\}

