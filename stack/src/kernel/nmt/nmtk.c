/**
********************************************************************************
\file   nmtk.c

\brief  Implementation of NMT kernel module

This file contains the implementation of the NMT kernel module.

\ingroup module_nmtk
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
#include "kernel/nmtk.h"
#include "EplTimer.h"
#include "kernel/dllk.h"

//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
// TracePoint support for realtime-debugging
#ifdef _DBG_TRACE_POINTS_
    void  TgtDbgSignalTracePoint (BYTE bTracePointNumber_p);
    void  TgtDbgPostTraceValue (DWORD dwTraceValue_p);
    #define TGT_DBG_SIGNAL_TRACE_POINT(p)   TgtDbgSignalTracePoint(p)
    #define TGT_DBG_POST_TRACE_VALUE(v)     TgtDbgPostTraceValue(v)
#else
    #define TGT_DBG_SIGNAL_TRACE_POINT(p)
    #define TGT_DBG_POST_TRACE_VALUE(v)
#endif

#define EPL_NMTK_DBG_POST_TRACE_VALUE(nmtEvent_p, oldNmtState_p, newNmtState_p) \
    TGT_DBG_POST_TRACE_VALUE((kEplEventSinkNmtk << 28) | ((mtEvent_p) << 16) \
                             | (((oldNmtState_p) & 0xFF) << 8) \
                             | ((newNmtState_p) & 0xFF))

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

typedef enum
{
    kNmtkGsOff,
    kNmtkGsInitialising,
    kNmtkGsResetApplication,
    kNmtkGsResetCommunication,
    kNmtkGsResetConfiguration,
    kNmtkCsNotActive,
    kNmtkCsPreOperational1,
    kNmtkCsStopped,
    kNmtkCsPreOperational2,
    kNmtkCsReadyToOperate,
    kNmtkCsOperational,
    kNmtkCsBasicEthernet,
    kNmtkMsNotActive,
    kNmtkMsPreOperational1,
    kNmtkMsPreOperational2,
    kNmtkMsReadyToOperate,
    kNmtkMsOperational,
    kNmtkMsBasicEthernet
} tNmtkStateIndexes;

//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------
typedef tEplKernel (*tNmtkStateFunc)(tNmtEvent nmtEvent_p);

typedef struct
{
    volatile tNmtkStateIndexes  stateIndex;
    volatile BOOL               fEnableReadyToOperate;
    volatile BOOL               fAppReadyToOperate;
    volatile BOOL               fTimerMsPreOp2;
    volatile BOOL               fAllMandatoryCNIdent;
    volatile BOOL               fFrozen;
} tNmtkInstance;

typedef struct
{
    tNmtState                   nmtState;
    tNmtkStateFunc              pfnState;
} tNmtkStateTable;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
tNmtkInstance               nmtkInstance_g;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static tEplKernel doStateGsOff(tNmtEvent nmtEvent_p);
static tEplKernel doStateGsInitialising(tNmtEvent nmtEvent_p);
static tEplKernel doStateGsResetApplication(tNmtEvent nmtEvent_p);
static tEplKernel doStateGsResetCommunication(tNmtEvent nmtEvent_p);
static tEplKernel doStateGsResetConfiguration(tNmtEvent nmtEvent_p);

static tEplKernel doStateMsNotActive(tNmtEvent nmtEvent_p);
#if defined(CONFIG_INCLUDE_NMT_MN)
static tEplKernel doStateMsPreOperational1(tNmtEvent nmtEvent_p);
static tEplKernel doStateMsPreOperational2(tNmtEvent nmtEvent_p);
static tEplKernel doStateMsReadyToOperate(tNmtEvent nmtEvent_p);
static tEplKernel doStateMsOperational(tNmtEvent nmtEvent_p);
static tEplKernel doStateMsBasicEthernet(tNmtEvent nmtEvent_p);
#endif

static tEplKernel doStateCsNotActive(tNmtEvent nmtEvent_p);
static tEplKernel doStateCsBasicEthernet(tNmtEvent nmtEvent_p);
static tEplKernel doStateCsPreOperational1(tNmtEvent nmtEvent_p);
static tEplKernel doStateCsPreOperational2(tNmtEvent nmtEvent_p);
static tEplKernel doStateCsReadyToOperate(tNmtEvent nmtEvent_p);
static tEplKernel doStateCsOperational(tNmtEvent nmtEvent_p);
static tEplKernel doStateCsStopped(tNmtEvent nmtEvent_p);

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
tNmtkStateTable             nmtkStates_g[] =
{
    { kNmtGsOff,                 doStateGsOff },
    { kNmtGsInitialising,        doStateGsInitialising },
    { kNmtGsResetApplication,    doStateGsResetApplication },
    { kNmtGsResetCommunication,  doStateGsResetCommunication },
    { kNmtGsResetConfiguration,  doStateGsResetConfiguration },
    { kNmtCsNotActive,           doStateCsNotActive },
    { kNmtCsPreOperational1,     doStateCsPreOperational1 },
    { kNmtCsStopped,             doStateCsStopped },
    { kNmtCsPreOperational2,     doStateCsPreOperational2 },
    { kNmtCsReadyToOperate,      doStateCsReadyToOperate },
    { kNmtCsOperational,         doStateCsOperational },
    { kNmtCsBasicEthernet,       doStateCsBasicEthernet },
    { kNmtMsNotActive,           doStateMsNotActive },
#if defined(CONFIG_INCLUDE_NMT_MN)
    { kNmtMsPreOperational1,     doStateMsPreOperational1 },
    { kNmtMsPreOperational2,     doStateMsPreOperational2 },
    { kNmtMsReadyToOperate,      doStateMsReadyToOperate },
    { kNmtMsOperational,         doStateMsOperational },
    { kNmtMsBasicEthernet,       doStateMsBasicEthernet }
#endif
};

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Init NMT kernel module

The function initializes an instance of the NMT kernel module

\return The function returns a tEplKernel error code.

\ingroup module_nmtk
*/
//------------------------------------------------------------------------------
tEplKernel nmtk_init(void)
{
    // initialize intern vaiables
    nmtkInstance_g.stateIndex = kNmtkGsOff;
    nmtkInstance_g.fEnableReadyToOperate = FALSE;
    nmtkInstance_g.fAppReadyToOperate = FALSE;
    nmtkInstance_g.fTimerMsPreOp2 = FALSE;
    nmtkInstance_g.fAllMandatoryCNIdent = FALSE;
    nmtkInstance_g.fFrozen = FALSE;

    return kEplSuccessful;
}

//------------------------------------------------------------------------------
/**
\brief  Delete NMT kernel module instance

The function deletes the NMT kernel module instance

\return The function returns a tEplKernel error code.

\ingroup module_nmtk
*/
//------------------------------------------------------------------------------
tEplKernel nmtk_delInstance(void)
{
    nmtkInstance_g.stateIndex = kNmtkGsOff;
    return kEplSuccessful;
}

//------------------------------------------------------------------------------
/**
\brief  Process NMT kernel events

The function processes NMT kernel events. It implements the NMT state machine.

\param  pEvent_p        Event to process.

\return The function returns a tEplKernel error code.

\ingroup module_nmtk
*/
//------------------------------------------------------------------------------
tEplKernel nmtk_process(tEplEvent* pEvent_p)
{
    tEplKernel              ret;
    tNmtkStateIndexes       oldState;
    tNmtEvent               nmtEvent;
    tEplEvent               event;
    tEventNmtStateChange    nmtStateChange;

    ret = kEplSuccessful;

    switch(pEvent_p->m_EventType)
    {
        case kEplEventTypeNmtEvent:
            nmtEvent = *((tNmtEvent*)pEvent_p->m_pArg);
            break;

        case kEplEventTypeTimer:
            nmtEvent = (tNmtEvent)((tEplTimerEventArg*)pEvent_p->m_pArg)->m_Arg.m_dwVal;
            break;

        default:
            return kEplNmtInvalidEvent;
    }

    // save NMT-State
    // needed for later comparison to inform higher layer about state change
    oldState = nmtkInstance_g.stateIndex;

    // process NMT-State-Maschine
    ret = nmtkStates_g[nmtkInstance_g.stateIndex].pfnState(nmtEvent);

    // inform higher layer about State-Change if needed
    if (oldState != nmtkInstance_g.stateIndex)
    {
        EPL_NMTK_DBG_POST_TRACE_VALUE(nmtEvent, nmtkStates_g[oldState].nmtState,
                                      nmtkStates_g[nmtkInstance_g.stateIndex].nmtState);
        EPL_DBGLVL_NMTK_TRACE("EplNmtkProcess(NMT-event = 0x%04X): New NMT-State = 0x%03X\n",
                              nmtEvent, nmtkStates_g[nmtkInstance_g.stateIndex].nmtState);

        nmtStateChange.newNmtState = nmtkStates_g[nmtkInstance_g.stateIndex].nmtState;
        nmtStateChange.oldNmtState = nmtkStates_g[oldState].nmtState;
        nmtStateChange.nmtEvent = nmtEvent;
        event.m_EventType = kEplEventTypeNmtStateChange;
        EPL_MEMSET(&event.m_NetTime, 0x00, sizeof(event.m_NetTime));
        event.m_pArg = &nmtStateChange;
        event.m_uiSize = sizeof(nmtStateChange);

        // inform DLLk module about state change
        event.m_EventSink = kEplEventSinkDllk;
        ret = dllk_process(&event);
        if (ret != kEplSuccessful)
           return ret;

        // inform higher layer about state change
        event.m_EventSink = kEplEventSinkNmtu;
        ret = eventk_postEvent(&event);
    }

    return ret;
}


//=========================================================================//
//                                                                         //
//          P R I V A T E   D E F I N I T I O N S                          //
//                                                                         //
//=========================================================================//

//------------------------------------------------------------------------------
/**
\brief  Process State GS_OFF

The function processes the NMT state GS_OFF.

\param  nmtEvent_p      NMT event to be processed.

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
static tEplKernel doStateGsOff(tNmtEvent nmtEvent_p)
{
    if (nmtEvent_p == kNmtEventSwReset)
    {   // NMT_GT8, NMT_GT1 -> new state kNmtGsInitialising
        nmtkInstance_g.stateIndex = kNmtkGsInitialising;
    }
    return kEplSuccessful;
}

//------------------------------------------------------------------------------
/**
\brief  Process State GS_INITIALISING

The function processes the NMT state GS_INITIALISING.
In this state the first init of the hardware will be done.

\param  nmtEvent_p      NMT event to be processed.

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
static tEplKernel doStateGsInitialising(tNmtEvent nmtEvent_p)
{
    switch(nmtEvent_p)
    {
        // 2006/07/31 d.k.: react also on NMT reset commands in ResetApp state
        // NMT Command SwitchOff
        case kNmtEventCriticalError:
        case kNmtEventSwitchOff:
            // NMT_GT3
            nmtkInstance_g.stateIndex = kNmtkGsOff;
            break;

        // new state kNmtGsResetApplication
        case kNmtEventEnterResetApp:
            // NMT_GT10
            nmtkInstance_g.stateIndex = kNmtkGsResetApplication;
            break;

        default:
            break;
    }
    return kEplSuccessful;
}

//------------------------------------------------------------------------------
/**
\brief  Process State GS_RESET_APPLICATION

The function processes the NMT state GS_RESET_APPLICATION.
In this state the initialization of the manufacturer-specific profile area
and the standardised device profile area is done.

\param  nmtEvent_p      NMT event to be processed.

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
static tEplKernel doStateGsResetApplication(tNmtEvent nmtEvent_p)
{
    switch(nmtEvent_p)
    {
        // 2006/07/31 d.k.: react also on NMT reset commands in ResetApp state
        // NMT Command SwitchOff
        case kNmtEventCriticalError:
        case kNmtEventSwitchOff:
            // NMT_GT3
            nmtkInstance_g.stateIndex = kNmtkGsOff;
            break;

        // NMT Command SwReset
        case kNmtEventSwReset:
            // NMT_GT8
            nmtkInstance_g.stateIndex = kNmtkGsInitialising;
            break;

        // leave this state only if higher layer
        // say so
        case kNmtEventEnterResetCom:
            // NMT_GT11
            // new state kNmtGsResetCommunication
            nmtkInstance_g.stateIndex = kNmtkGsResetCommunication;
            break;

        default:
            break;
    }
    return kEplSuccessful;
}

//------------------------------------------------------------------------------
/**
\brief  Process State GS_RESET_COMMUNICATION

The function processes the NMT state GS_RESET_COMMUNICATION.
In this state the initialization of the communication profile area is done.

\param  nmtEvent_p      NMT event to be processed.

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
static tEplKernel doStateGsResetCommunication(tNmtEvent nmtEvent_p)
{
    switch(nmtEvent_p)
    {
        // 2006/07/31 d.k.: react also on NMT reset commands in ResetComm state
        // NMT Command SwitchOff
        case kNmtEventCriticalError:
        case kNmtEventSwitchOff:
            // NMT_GT3
            nmtkInstance_g.stateIndex = kNmtkGsOff;
            break;

        // NMT Command SwReset
        case kNmtEventSwReset:
            // NMT_GT8
            nmtkInstance_g.stateIndex = kNmtkGsInitialising;
            break;

        // NMT Command ResetNode
        case kNmtEventResetNode:
            // NMT_GT4
            nmtkInstance_g.stateIndex = kNmtkGsResetApplication;
            break;

        // leave this state only if higher layer
        // say so
        case kNmtEventEnterResetConfig:
            // NMT_GT12 -> new state kNmtGsResetConfiguration
            nmtkInstance_g.stateIndex = kNmtkGsResetConfiguration;
            break;

        default:
            break;
    }
    return kEplSuccessful;
}

//------------------------------------------------------------------------------
/**
\brief  Process State GS_RESET_CONFIGURATION

The function processes the NMT state GS_RESET_CONFIGURATION.
In this state we build the configuration with infos from OD.

\param  nmtEvent_p      NMT event to be processed.

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
static tEplKernel doStateGsResetConfiguration(tNmtEvent nmtEvent_p)
{
    // reset flags
    nmtkInstance_g.fEnableReadyToOperate = FALSE;
    nmtkInstance_g.fAppReadyToOperate = FALSE;
    nmtkInstance_g.fFrozen = FALSE;

    // check events
    switch(nmtEvent_p)
    {
        // 2006/07/31 d.k.: react also on NMT reset commands in ResetConf state
        // NMT Command SwitchOff
        case kNmtEventCriticalError:
        case kNmtEventSwitchOff:
            // NMT_GT3
            nmtkInstance_g.stateIndex = kNmtkGsOff;
            break;

        // NMT Command SwReset
        case kNmtEventSwReset:
            // NMT_GT8
            nmtkInstance_g.stateIndex = kNmtkGsInitialising;
            break;

        // NMT Command ResetNode
        case kNmtEventResetNode:
            // NMT_GT4
            nmtkInstance_g.stateIndex = kNmtkGsResetApplication;
            break;

        // NMT Command ResetCommunication
        case kNmtEventResetCom:
            // NMT_GT5
            nmtkInstance_g.stateIndex = kNmtkGsResetCommunication;
            break;

        // leave this state only if higher layer says so
        case kNmtEventEnterCsNotActive:
            // Node should be CN (NMT_CT1)
            nmtkInstance_g.stateIndex = kNmtkCsNotActive;
            break;

        case kNmtEventEnterMsNotActive:
            // Node should be MN (NMT_MT1)
#if !defined(CONFIG_INCLUDE_NMT_MN)
                // no MN functionality
                // TODO: -create error E_NMT_BA1_NO_MN_SUPPORT
                nmtkInstance_g.fFrozen = TRUE;
#else
                nmtkInstance_g.stateIndex = kNmtkMsNotActive;
#endif
            break;

        default:
            break;
    }
    return kEplSuccessful;
}

//------------------------------------------------------------------------------
/**
\brief  Process State CS_NOT_ACTIVE

The function processes the NMT state CS_NOT_ACTIVE.
In this state the node listens for EPL-Frames and checks timeout.

\param  nmtEvent_p      NMT event to be processed.

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
static tEplKernel doStateCsNotActive(tNmtEvent nmtEvent_p)
{
    switch(nmtEvent_p)
    {
        // 2006/07/31 d.k.: react also on NMT reset commands in NotActive state
        // NMT Command SwitchOff
        case kNmtEventCriticalError:
        case kNmtEventSwitchOff:
            // NMT_GT3
            nmtkInstance_g.stateIndex = kNmtkGsOff;
            break;

        // NMT Command SwReset
        case kNmtEventSwReset:
            // NMT_GT8
            nmtkInstance_g.stateIndex = kNmtkGsInitialising;
            break;

        // NMT Command ResetNode
        case kNmtEventResetNode:
            // NMT_GT4
            nmtkInstance_g.stateIndex = kNmtkGsResetApplication;
            break;

        // NMT Command ResetCommunication or internal Communication error
        case kNmtEventResetCom:          // NMT_GT5
        case kNmtEventInternComError:    // NMT_GT6
            nmtkInstance_g.stateIndex = kNmtkGsResetCommunication;
            break;

        // NMT Command Reset Configuration
        case kNmtEventResetConfig:
            // NMT_GT7
            nmtkInstance_g.stateIndex = kNmtkGsResetConfiguration;
            break;

        // see if SoA or SoC received
        case kNmtEventDllCeSoc:
        case kNmtEventDllCeSoa:
            // NMT_CT2 -> new state PRE_OPERATIONAL1
            nmtkInstance_g.stateIndex = kNmtkCsPreOperational1;
            break;

        // timeout for SoA and Soc
        case kNmtEventTimerBasicEthernet:
            // NMT_CT3 -> new state BASIC_ETHERNET
            nmtkInstance_g.stateIndex = kNmtkCsBasicEthernet;
            break;

        default:
            break;
    }
    return kEplSuccessful;
}

//------------------------------------------------------------------------------
/**
\brief  Process State CS_PRE_OPERATIONAL1

The function processes the NMT state CS_PRE_OPERATIONAL1.
In this state the node processes only async frames.

\param  nmtEvent_p      NMT event to be processed.

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
static tEplKernel doStateCsPreOperational1(tNmtEvent nmtEvent_p)
{
    switch(nmtEvent_p)
    {
        // NMT Command SwitchOff
        case kNmtEventCriticalError:
        case kNmtEventSwitchOff:
            // NMT_GT3
            nmtkInstance_g.stateIndex = kNmtkGsOff;
            break;

        // NMT Command SwReset
        case kNmtEventSwReset:
            // NMT_GT8
            nmtkInstance_g.stateIndex = kNmtkGsInitialising;
            break;

        // NMT Command ResetNode
        case kNmtEventResetNode:
            // NMT_GT4
            nmtkInstance_g.stateIndex = kNmtkGsResetApplication;
            break;

        // NMT Command ResetCommunication
        // or internal Communication error
        case kNmtEventResetCom:          // NMT_GT5
        case kNmtEventInternComError:    // NMT_GT6
            nmtkInstance_g.stateIndex = kNmtkGsResetCommunication;
            break;

        // NMT Command Reset Configuration
        case kNmtEventResetConfig:
            // NMT_GT7
            nmtkInstance_g.stateIndex = kNmtkGsResetConfiguration;
            break;

        // check if SoC received
        case kNmtEventDllCeSoc:
            // NMT_CT4
            nmtkInstance_g.stateIndex = kNmtkCsPreOperational2;
            break;

        default:
            break;
    }
    return kEplSuccessful;
}

//------------------------------------------------------------------------------
/**
\brief  Process State CS_PRE_OPERATIONAL2

The function processes the NMT state CS_PRE_OPERATIONAL2.
In this state the node processes isochronous and asynchronous frames.

\param  nmtEvent_p      NMT event to be processed.

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
static tEplKernel doStateCsPreOperational2(tNmtEvent nmtEvent_p)
{
    switch(nmtEvent_p)
    {
        // NMT Command SwitchOff
        case kNmtEventCriticalError:
        case kNmtEventSwitchOff:
            // NMT_GT3
            nmtkInstance_g.stateIndex = kNmtkGsOff;
            break;

        // NMT Command SwReset
        case kNmtEventSwReset:
            // NMT_GT8
            nmtkInstance_g.stateIndex = kNmtkGsInitialising;
            break;

        // NMT Command ResetNode
        case kNmtEventResetNode:
            // NMT_GT4
            nmtkInstance_g.stateIndex = kNmtkGsResetApplication;
            break;

        // NMT Command ResetCommunication
        // or internal Communication error
        case kNmtEventResetCom:          // NMT_GT5
        case kNmtEventInternComError:    // NMT_GT6
            nmtkInstance_g.stateIndex = kNmtkGsResetCommunication;
            break;

        // NMT Command Reset Configuration
        case kNmtEventResetConfig:
            // NMT_GT7
            nmtkInstance_g.stateIndex = kNmtkGsResetConfiguration;
            break;

        // NMT Command StopNode
        case kNmtEventStopNode:
            // NMT_CT8 - reset flags
            nmtkInstance_g.fEnableReadyToOperate = FALSE;
            nmtkInstance_g.fAppReadyToOperate = FALSE;
            nmtkInstance_g.stateIndex = kNmtkCsStopped;
            break;

        // error occurred
        case kNmtEventNmtCycleError:
            // NMT_CT11 - reset flags
            nmtkInstance_g.fEnableReadyToOperate = FALSE;
            nmtkInstance_g.fAppReadyToOperate = FALSE;
            nmtkInstance_g.stateIndex = kNmtkCsPreOperational1;
            break;

        // check if application is ready to operate
        case kNmtEventEnterReadyToOperate:
            // check if command NMTEnableReadyToOperate from MN was received
            if(nmtkInstance_g.fEnableReadyToOperate == TRUE)
            {   // reset flags
                nmtkInstance_g.fEnableReadyToOperate = FALSE;
                nmtkInstance_g.fAppReadyToOperate = FALSE;
                // change state (NMT_CT6)
                nmtkInstance_g.stateIndex = kNmtkCsReadyToOperate;
            }
            else
            {   // set Flag (NMT_CT5)
                nmtkInstance_g.fAppReadyToOperate = TRUE;
            }
            break;

        // NMT Commando EnableReadyToOperate
        case kNmtEventEnableReadyToOperate:
            // check if application is ready
            if(nmtkInstance_g.fAppReadyToOperate == TRUE)
            {   // reset flags
                nmtkInstance_g.fEnableReadyToOperate = FALSE;
                nmtkInstance_g.fAppReadyToOperate = FALSE;
                // change state (NMT_CT6)
                nmtkInstance_g.stateIndex = kNmtkCsReadyToOperate;
            }
            else
            {   // set Flag (NMT_CT5)
                nmtkInstance_g.fEnableReadyToOperate = TRUE;
            }
            break;

        default:
            break;
    }
    return kEplSuccessful;
}

//------------------------------------------------------------------------------
/**
\brief  Process State CS_READY_TO_OPERATE

The function processes the NMT state CS_READY_TO_OPERATE.
In this state the node should be configured and application is ready.

\param  nmtEvent_p      NMT event to be processed.

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
static tEplKernel doStateCsReadyToOperate(tNmtEvent nmtEvent_p)
{
    switch(nmtEvent_p)
    {
        // NMT Command SwitchOff
        case kNmtEventCriticalError:
        case kNmtEventSwitchOff:
            // NMT_GT3
            nmtkInstance_g.stateIndex = kNmtkGsOff;
            break;

        // NMT Command SwReset
        case kNmtEventSwReset:
            // NMT_GT8
            nmtkInstance_g.stateIndex = kNmtkGsInitialising;
            break;

        // NMT Command ResetNode
        case kNmtEventResetNode:
            // NMT_GT4
            nmtkInstance_g.stateIndex = kNmtkGsResetApplication;
            break;

        // NMT Command ResetCommunication or internal Communication error
        case kNmtEventResetCom:          // NMT_GT5
        case kNmtEventInternComError:    // NMT_GT6
            nmtkInstance_g.stateIndex = kNmtkGsResetCommunication;
            break;

        // NMT Command ResetConfiguration
        case kNmtEventResetConfig:
            // NMT_GT7
            nmtkInstance_g.stateIndex = kNmtkGsResetConfiguration;
            break;

        // NMT Command StopNode
        case kNmtEventStopNode:
            // NMT_CT8
            nmtkInstance_g.stateIndex = kNmtkCsStopped;
            break;

        // error occurred
        case kNmtEventNmtCycleError:
            // NMT_CT11
            nmtkInstance_g.stateIndex = kNmtkCsPreOperational1;
            break;

        // NMT Command StartNode
        case kNmtEventStartNode:
            // NMT_CT7
            nmtkInstance_g.stateIndex = kNmtkCsOperational;
            break;

        default:
            break;
    }
    return kEplSuccessful;
}

//------------------------------------------------------------------------------
/**
\brief  Process State CS_OPERATIONAL

The function processes the NMT state CS_OPERATIONAL.
This is the normal working state of a CN.

\param  nmtEvent_p      NMT event to be processed.

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
static tEplKernel doStateCsOperational(tNmtEvent nmtEvent_p)
{
    switch(nmtEvent_p)
    {
        // NMT Command SwitchOff
        case kNmtEventCriticalError:
        case kNmtEventSwitchOff:
            // NMT_GT3
            nmtkInstance_g.stateIndex = kNmtkGsOff;
            break;

        // NMT Command SwReset
        case kNmtEventSwReset:
            // NMT_GT8
            nmtkInstance_g.stateIndex = kNmtkGsInitialising;
            break;

        // NMT Command ResetNode
        case kNmtEventResetNode:
            // NMT_GT4
            nmtkInstance_g.stateIndex = kNmtkGsResetApplication;
            break;

        // NMT Command ResetCommunication or internal Communication error
        case kNmtEventResetCom:          // NMT_GT5
        case kNmtEventInternComError:    // NMT_GT6
            nmtkInstance_g.stateIndex = kNmtkGsResetCommunication;
            break;

        // NMT Command ResetConfiguration
        case kNmtEventResetConfig:
            // NMT_GT7
            nmtkInstance_g.stateIndex = kNmtkGsResetConfiguration;
            break;

        // NMT Command StopNode
        case kNmtEventStopNode:
            // NMT_CT8
            nmtkInstance_g.stateIndex = kNmtkCsStopped;
            break;

        // NMT Command EnterPreOperational2
        case kNmtEventEnterPreOperational2:
            // NMT_CT9
            nmtkInstance_g.stateIndex = kNmtkCsPreOperational2;
            break;

        // error occurred
        case kNmtEventNmtCycleError:
            // NMT_CT11
            nmtkInstance_g.stateIndex = kNmtkCsPreOperational1;
            break;

        default:
            break;
    }
    return kEplSuccessful;
}

//------------------------------------------------------------------------------
/**
\brief  Process State CS_NOT_ACTIVE

The function processes the NMT state CS_NOT_ACTIVE.
In this state the node is stopped by the MN it processes only asynchronous
frames.

\param  nmtEvent_p      NMT event to be processed.

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
static tEplKernel doStateCsStopped(tNmtEvent nmtEvent_p)
{
    switch(nmtEvent_p)
    {
        // NMT Command SwitchOff
        case kNmtEventCriticalError:
        case kNmtEventSwitchOff:
            // NMT_GT3
            nmtkInstance_g.stateIndex = kNmtkGsOff;
            break;

        // NMT Command SwReset
        case kNmtEventSwReset:
            // NMT_GT8
            nmtkInstance_g.stateIndex = kNmtkGsInitialising;
            break;

        // NMT Command ResetNode
        case kNmtEventResetNode:
            // NMT_GT4
            nmtkInstance_g.stateIndex = kNmtkGsResetApplication;
            break;

        // NMT Command ResetCommunication or internal Communication error
        case kNmtEventResetCom:          // NMT_GT5
        case kNmtEventInternComError:    // NMT_GT6
            nmtkInstance_g.stateIndex = kNmtkGsResetCommunication;
            break;

        // NMT Command ResetConfiguration
        case kNmtEventResetConfig:
        {   // NMT_GT7
            nmtkInstance_g.stateIndex = kNmtkGsResetConfiguration;
            break;
        }

        // NMT Command EnterPreOperational2
        case kNmtEventEnterPreOperational2:
            // NMT_CT10
            nmtkInstance_g.stateIndex = kNmtkCsPreOperational2;
            break;

        // error occurred
        case kNmtEventNmtCycleError:
            // NMT_CT11
            nmtkInstance_g.stateIndex = kNmtkCsPreOperational1;
            break;

        default:
            break;
    }
    return kEplSuccessful;
}

//------------------------------------------------------------------------------
/**
\brief  Process State CS_BASIC_ETHERNET

The function processes the NMT state CS_BASIC_ETHERNET.
In this state there is no POWERLINK cycle and the node performs normal ethernet
communication.

\param  nmtEvent_p      NMT event to be processed.

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
static tEplKernel doStateCsBasicEthernet(tNmtEvent nmtEvent_p)
{
    switch(nmtEvent_p)
    {
        // NMT Command SwitchOff
        case kNmtEventCriticalError:
        case kNmtEventSwitchOff:
            // NMT_GT3
            nmtkInstance_g.stateIndex = kNmtkGsOff;
            break;

        // NMT Command SwReset
        case kNmtEventSwReset:
            // NMT_GT8
            nmtkInstance_g.stateIndex = kNmtkGsInitialising;
            break;

        // NMT Command ResetNode
        case kNmtEventResetNode:
            // NMT_GT4
            nmtkInstance_g.stateIndex = kNmtkGsResetApplication;
            break;

        // NMT Command ResetCommunication or internal Communication error
        case kNmtEventResetCom:          // NMT_GT5
        case kNmtEventInternComError:    // NMT_GT6
            nmtkInstance_g.stateIndex = kNmtkGsResetCommunication;
            break;

        // NMT Command ResetConfiguration
        case kNmtEventResetConfig:
            // NMT_GT7
            nmtkInstance_g.stateIndex = kNmtkGsResetConfiguration;
            break;

        // error occurred
        // d.k.: how does this error occur? on CRC errors
/*      case kNmtEventNmtCycleError:
            nmtkInstance_g.stateIndex = kNmtkCsPreOperational1:
            break;
*/
        case kNmtEventDllCeSoc:
        case kNmtEventDllCePreq:
        case kNmtEventDllCePres:
        case kNmtEventDllCeSoa:
            // NMT_CT12 - EPL frame on net -> stop any communication
            nmtkInstance_g.stateIndex = kNmtkCsPreOperational1;
            break;

        default:
            break;
    }
    return kEplSuccessful;
}

//------------------------------------------------------------------------------
/**
\brief  Process State MS_NOT_ACTIVE

The function processes the NMT state MS_NOT_ACTIVE.
In this state the MN listens to the network. If there is no POWERLINK traffic,
the node goes to the next state.

\param  nmtEvent_p      NMT event to be processed.

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
static tEplKernel doStateMsNotActive(tNmtEvent nmtEvent_p)
{

#if !defined(CONFIG_INCLUDE_NMT_MN)
    UNUSED_PARAMETER(nmtEvent_p);

    // no MN functionality
    // TODO: -create error E_NMT_BA1_NO_MN_SUPPORT
    nmtkInstance_g.fFrozen = TRUE;
#else
    switch(nmtEvent_p)
    {
        // NMT Command SwitchOff
        case kNmtEventCriticalError:
        case kNmtEventSwitchOff:
            // NMT_GT3
            nmtkInstance_g.stateIndex = kNmtkGsOff;
            break;

        // NMT Command SwReset
        case kNmtEventSwReset:
            // NMT_GT8
            nmtkInstance_g.stateIndex = kNmtkGsInitialising;
            break;

        // NMT Command ResetNode
        case kNmtEventResetNode:
            // NMT_GT4
            nmtkInstance_g.stateIndex = kNmtkGsResetApplication;
            break;

        // NMT Command ResetCommunication or internal Communication error
        case kNmtEventResetCom:          // NMT_GT5
        case kNmtEventInternComError:    // NMT_GT6
            nmtkInstance_g.stateIndex = kNmtkGsResetCommunication;
            break;

        // NMT Command ResetConfiguration
        case kNmtEventResetConfig:
            // NMT_GT7
            nmtkInstance_g.stateIndex = kNmtkGsResetConfiguration;
            break;

        // EPL frames received
        case kNmtEventDllCeSoc:
        case kNmtEventDllCeSoa:
            // other MN in network
            // $$$ d.k.: generate error history entry
            nmtkInstance_g.fFrozen = TRUE;
            break;

        // timeout event
        case kNmtEventTimerBasicEthernet:
           // NMT_MT7
            if (nmtkInstance_g.fFrozen == FALSE)
            {   // new state BasicEthernet
                nmtkInstance_g.stateIndex = kNmtkMsBasicEthernet;
            }
            break;

        // timeout event
        case kNmtEventTimerMsPreOp1:
        {   // NMT_MT2
            if (nmtkInstance_g.fFrozen == FALSE)
            {   // new state PreOp1
                nmtkInstance_g.stateIndex = kNmtkMsPreOperational1;
                nmtkInstance_g.fTimerMsPreOp2 = FALSE;
                nmtkInstance_g.fAllMandatoryCNIdent = FALSE;
            }
            break;
        }

        default:
            break;

    }
#endif
    return kEplSuccessful;
}

#if defined(CONFIG_INCLUDE_NMT_MN)

//------------------------------------------------------------------------------
/**
\brief  Process State MS_PRE_OPERATIONAL1

The function processes the NMT state MS_PRE_OPERATIONAL1.
In this state the MN processes the reduced POWERLINK cycle.

\param  nmtEvent_p      NMT event to be processed.

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
static tEplKernel doStateMsPreOperational1(tNmtEvent nmtEvent_p)
{
    switch(nmtEvent_p)
    {
        // NMT Command SwitchOff
        case kNmtEventCriticalError:
        case kNmtEventSwitchOff:
            // NMT_GT3
            nmtkInstance_g.stateIndex = kNmtkGsOff;
            break;

        // NMT Command SwReset
        case kNmtEventSwReset:
            // NMT_GT8
            nmtkInstance_g.stateIndex = kNmtkGsInitialising;
            break;

        // NMT Command ResetNode
        case kNmtEventResetNode:
            // NMT_GT4
            nmtkInstance_g.stateIndex = kNmtkGsResetApplication;
            break;

        // NMT Command ResetCommunication or internal Communication error
        case kNmtEventResetCom:          // NMT_GT5
        case kNmtEventInternComError:    // NMT_GT6
            nmtkInstance_g.stateIndex = kNmtkGsResetCommunication;
            break;

        // NMT Command ResetConfiguration
        case kNmtEventResetConfig:
            // NMT_GT7
            nmtkInstance_g.stateIndex = kNmtkGsResetConfiguration;
            break;

        // EPL frames received
        case kNmtEventDllCeSoc:
        case kNmtEventDllCeSoa:
            // other MN in network
            // $$$ d.k.: generate error history entry
            nmtkInstance_g.stateIndex = kNmtkGsResetCommunication;
            break;

        // error occurred
        // d.k. MSPreOp1->CSPreOp1: nonsense -> keep state
        /*
        case kNmtEventNmtCycleError:
            nmtkInstance_g.stateIndex = kNmtkCsPreOperational1;
            break;
        */

        case kNmtEventAllMandatoryCNIdent:
            // all mandatory CN identified
            if (nmtkInstance_g.fTimerMsPreOp2 != FALSE)
            {   // NMT_MT3
                nmtkInstance_g.stateIndex = kNmtkMsPreOperational2;
            }
            else
            {
                nmtkInstance_g.fAllMandatoryCNIdent = TRUE;
            }
            break;

        case kNmtEventTimerMsPreOp2:
            // residence time for PreOp1 is elapsed
            if (nmtkInstance_g.fAllMandatoryCNIdent != FALSE)
            {   // NMT_MT3
                nmtkInstance_g.stateIndex = kNmtkMsPreOperational2;
            }
            else
            {
                nmtkInstance_g.fTimerMsPreOp2 = TRUE;
            }
            break;

        default:
            break;

    }
    return kEplSuccessful;
}

//------------------------------------------------------------------------------
/**
\brief  Process State MS_PRE_OPERATIONAL2

The function processes the NMT state MS_PRE_OPERATIONAL2.
In this state the MN processes the full POWERLINK cycle.

\param  nmtEvent_p      NMT event to be processed.

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
static tEplKernel doStateMsPreOperational2(tNmtEvent nmtEvent_p)
{
    switch(nmtEvent_p)
    {
        // NMT Command SwitchOff
        case kNmtEventCriticalError:
        case kNmtEventSwitchOff:
            // NMT_GT3
            nmtkInstance_g.stateIndex = kNmtkGsOff;
            break;

        // NMT Command SwReset
        case kNmtEventSwReset:
            // NMT_GT8
            nmtkInstance_g.stateIndex = kNmtkGsInitialising;
            break;

        // NMT Command ResetNode
        case kNmtEventResetNode:
            // NMT_GT4
            nmtkInstance_g.stateIndex = kNmtkGsResetApplication;
            break;

        // NMT Command ResetCommunication or internal Communication error
        case kNmtEventResetCom:          // NMT_GT5
        case kNmtEventInternComError:    // NMT_GT6
            nmtkInstance_g.stateIndex = kNmtkGsResetCommunication;
            break;

        // NMT Command ResetConfiguration
        case kNmtEventResetConfig:
            // NMT_GT7
            nmtkInstance_g.stateIndex = kNmtkGsResetConfiguration;
            break;

        // EPL frames received
        case kNmtEventDllCeSoc:
        case kNmtEventDllCeSoa:
            // other MN in network
            // $$$ d.k.: generate error history entry
            nmtkInstance_g.stateIndex = kNmtkGsResetCommunication;
            break;

        // error occurred
        case kNmtEventNmtCycleError:
            // NMT_MT6
            nmtkInstance_g.stateIndex = kNmtkMsPreOperational1;
            break;

        case kNmtEventEnterReadyToOperate:
            // NMT_MT4
            nmtkInstance_g.stateIndex = kNmtkMsReadyToOperate;
            break;

        default:
            break;
    }
    return kEplSuccessful;
}

//------------------------------------------------------------------------------
/**
\brief  Process State MS_READY_TO_OPERATE

The function processes the NMT state MS_READY_TO_OPERATE.
In this state all mandatory CNs are ready to operate. The MN processes the full
POWERLINK cycle.

\param  nmtEvent_p      NMT event to be processed.

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
static tEplKernel doStateMsReadyToOperate(tNmtEvent nmtEvent_p)
{
    switch(nmtEvent_p)
    {
        // NMT Command SwitchOff
        case kNmtEventCriticalError:
        case kNmtEventSwitchOff:
            // NMT_GT3
            nmtkInstance_g.stateIndex = kNmtkGsOff;
            break;

        // NMT Command SwReset
        case kNmtEventSwReset:
            // NMT_GT8
            nmtkInstance_g.stateIndex = kNmtkGsInitialising;
            break;

        // NMT Command ResetNode
        case kNmtEventResetNode:
            // NMT_GT4
            nmtkInstance_g.stateIndex = kNmtkGsResetApplication;
            break;

        // NMT Command ResetCommunication or internal Communication error
        case kNmtEventResetCom:          // NMT_GT5
        case kNmtEventInternComError:    // NMT_GT6
            nmtkInstance_g.stateIndex = kNmtkGsResetCommunication;
            break;

        // NMT Command ResetConfiguration
        case kNmtEventResetConfig:
            // NMT_GT7
            nmtkInstance_g.stateIndex = kNmtkGsResetConfiguration;
            break;

        // EPL frames received
        case kNmtEventDllCeSoc:
        case kNmtEventDllCeSoa:
            // other MN in network
            // $$$ d.k.: generate error history entry
            nmtkInstance_g.stateIndex = kNmtkGsResetCommunication;
            break;

        // error occurred
        case kNmtEventNmtCycleError:
            // NMT_MT6
            nmtkInstance_g.stateIndex = kNmtkMsPreOperational1;
            break;

        case kNmtEventEnterMsOperational:
            // NMT_MT5
            nmtkInstance_g.stateIndex = kNmtkMsOperational;
            break;

        default:
            break;
    }
    return kEplSuccessful;
}
//------------------------------------------------------------------------------
/**
\brief  Process State MS_OPERATIONAL

The function processes the NMT state MS_OPERATIONAL.
This is the normal working state of a MN.

\param  nmtEvent_p      NMT event to be processed.

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
static tEplKernel doStateMsOperational(tNmtEvent nmtEvent_p)
{
    switch(nmtEvent_p)
    {
        // NMT Command SwitchOff
        case kNmtEventCriticalError:
        case kNmtEventSwitchOff:
            // NMT_GT3
            nmtkInstance_g.stateIndex = kNmtkGsOff;
            break;

        // NMT Command SwReset
        case kNmtEventSwReset:
            // NMT_GT8
            nmtkInstance_g.stateIndex = kNmtkGsInitialising;
            break;

        // NMT Command ResetNode
        case kNmtEventResetNode:
            // NMT_GT4
            nmtkInstance_g.stateIndex = kNmtkGsResetApplication;
            break;

        // NMT Command ResetCommunication or internal Communication error
        case kNmtEventResetCom:          // NMT_GT5
        case kNmtEventInternComError:    // NMT_GT6
            nmtkInstance_g.stateIndex = kNmtkGsResetCommunication;
            break;

        // NMT Command ResetConfiguration
        case kNmtEventResetConfig:
            // NMT_GT7
            nmtkInstance_g.stateIndex = kNmtkGsResetConfiguration;
            break;

        // EPL frames received
        case kNmtEventDllCeSoc:
        case kNmtEventDllCeSoa:
            // other MN in network
            // $$$ d.k.: generate error history entry
            nmtkInstance_g.stateIndex = kNmtkGsResetCommunication;
            break;

        // error occurred
        case kNmtEventNmtCycleError:
            // NMT_MT6
            nmtkInstance_g.stateIndex = kNmtkMsPreOperational1;
            break;

        default:
            break;
    }
    return kEplSuccessful;
}

//------------------------------------------------------------------------------
/**
\brief  Process State MS_BASIC_ETHERNET

The function processes the NMT state MS_BASIC_ETHERNET.
In this state the MN processes normal ethernet traffic.

\param  nmtEvent_p      NMT event to be processed.

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
tEplKernel doStateMsBasicEthernet(tNmtEvent nmtEvent_p)
{
    switch(nmtEvent_p)
    {
        // NMT Command SwitchOff
        case kNmtEventCriticalError:
        case kNmtEventSwitchOff:
            // NMT_GT3
            nmtkInstance_g.stateIndex = kNmtkGsOff;
            break;

        // NMT Command SwReset
        case kNmtEventSwReset:
            // NMT_GT8
            nmtkInstance_g.stateIndex = kNmtkGsInitialising;
            break;

        // NMT Command ResetNode
        case kNmtEventResetNode:
            // NMT_GT4
            nmtkInstance_g.stateIndex = kNmtkGsResetApplication;
            break;

        // NMT Command ResetCommunication or internal Communication error
        case kNmtEventResetCom:          // NMT_GT5
        case kNmtEventInternComError:    // NMT_GT6
            nmtkInstance_g.stateIndex = kNmtkGsResetCommunication;
            break;

        // NMT Command ResetConfiguration
        case kNmtEventResetConfig:
            // NMT_GT7
            nmtkInstance_g.stateIndex = kNmtkGsResetConfiguration;
            break;

        // EPL frames received
        case kNmtEventDllCeSoc:
        case kNmtEventDllCeSoa:
            // other MN in network
            // $$$ d.k.: generate error history entry
            nmtkInstance_g.stateIndex = kNmtkGsResetCommunication;
            break;

        // error occurred
        // d.k. BE->PreOp1 on cycle error? No
/*      case kNmtEventNmtCycleError:
            nmtkInstance_g.stateIndex = kNmtkCsPreOperational1;
            break;
*/
        default:
            break;
    }
    return kEplSuccessful;
}

#endif
