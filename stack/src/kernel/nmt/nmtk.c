/**
********************************************************************************
\file   nmtk.c

\brief  Implementation of NMT kernel module

This file contains the implementation of the NMT kernel module.

\ingroup module_nmtk
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2015, SYSTEC electronic GmbH
Copyright (c) 2017, B&R Industrial Automation GmbH
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
#include <kernel/nmtk.h>
#include <oplk/nmt.h>
#include <kernel/dllk.h>
#include <kernel/eventk.h>
#include <common/timer.h>

#if defined(CONFIG_INCLUDE_LEDK)
#include <kernel/ledk.h>
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

/**
\brief NMT state enumeration

Enumeration for NMT states.
*/
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
    kNmtkRmsNotActive,
    kNmtkMsPreOperational1,
    kNmtkMsPreOperational2,
    kNmtkMsReadyToOperate,
    kNmtkMsOperational,
    kNmtkMsBasicEthernet
} eNmtkStateIndexes;

/**
\brief NMT state data type

Data type for the enumerator \ref eNmtkStateIndexes.
*/
typedef UINT32 tNmtkStateIndexes;

//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------
typedef tOplkError (*tNmtkStateFunc)(tNmtEvent nmtEvent_p);

typedef struct
{
    volatile tNmtkStateIndexes  stateIndex;
    volatile BOOL               fEnableReadyToOperate;
    volatile BOOL               fAppReadyToOperate;
    volatile BOOL               fTimerMsPreOp2;
    volatile BOOL               fAllMandatoryCNIdent;
    volatile BOOL               fFrozen;
    volatile BOOL               fRedundancy;
} tNmtkInstance;

typedef struct
{
    tNmtState                   nmtState;
    tNmtkStateFunc              pfnState;
} tNmtkStateTable;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tNmtkInstance        nmtkInstance_l;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static tOplkError doStateGsOff(tNmtEvent nmtEvent_p);
static tOplkError doStateGsInitialising(tNmtEvent nmtEvent_p);
static tOplkError doStateGsResetApplication(tNmtEvent nmtEvent_p);
static tOplkError doStateGsResetCommunication(tNmtEvent nmtEvent_p);
static tOplkError doStateGsResetConfiguration(tNmtEvent nmtEvent_p);

static tOplkError doStateMsNotActive(tNmtEvent nmtEvent_p);
#if defined(CONFIG_INCLUDE_NMT_MN)
static tOplkError doStateMsPreOperational1(tNmtEvent nmtEvent_p);
static tOplkError doStateMsPreOperational2(tNmtEvent nmtEvent_p);
static tOplkError doStateMsReadyToOperate(tNmtEvent nmtEvent_p);
static tOplkError doStateMsOperational(tNmtEvent nmtEvent_p);
static tOplkError doStateMsBasicEthernet(tNmtEvent nmtEvent_p);
#endif

static tOplkError doStateCsNotActive(tNmtEvent nmtEvent_p);
static tOplkError doStateCsBasicEthernet(tNmtEvent nmtEvent_p);
static tOplkError doStateCsPreOperational1(tNmtEvent nmtEvent_p);
static tOplkError doStateCsPreOperational2(tNmtEvent nmtEvent_p);
static tOplkError doStateCsReadyToOperate(tNmtEvent nmtEvent_p);
static tOplkError doStateCsOperational(tNmtEvent nmtEvent_p);
static tOplkError doStateCsStopped(tNmtEvent nmtEvent_p);
static tOplkError doStateRmsNotActive(tNmtEvent nmtEvent_p);

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tNmtkStateTable      nmtkStates_l[] =
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
    { kNmtRmsNotActive,          doStateRmsNotActive },
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

\return The function returns a tOplkError error code.

\ingroup module_nmtk
*/
//------------------------------------------------------------------------------
tOplkError nmtk_init(void)
{
    // initialize internal variables
    nmtkInstance_l.stateIndex = kNmtkGsOff;
    nmtkInstance_l.fEnableReadyToOperate = FALSE;
    nmtkInstance_l.fAppReadyToOperate = FALSE;
    nmtkInstance_l.fTimerMsPreOp2 = FALSE;
    nmtkInstance_l.fAllMandatoryCNIdent = FALSE;
    nmtkInstance_l.fFrozen = FALSE;
    nmtkInstance_l.fRedundancy = FALSE;

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Shut down NMT kernel module

The function shuts down the NMT kernel module.

\return The function returns a tOplkError error code.

\ingroup module_nmtk
*/
//------------------------------------------------------------------------------
tOplkError nmtk_exit(void)
{
    nmtkInstance_l.stateIndex = kNmtkGsOff;

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Process NMT kernel events

The function processes NMT kernel events. It implements the NMT state machine.

\param[in]      pEvent_p            Event to process.

\return The function returns a tOplkError error code.

\ingroup module_nmtk
*/
//------------------------------------------------------------------------------
tOplkError nmtk_process(const tEvent* pEvent_p)
{
    tOplkError              ret = kErrorOk;
    tNmtkStateIndexes       oldState;
    tNmtEvent               nmtEvent;
    tEvent                  event;
    tEventNmtStateChange    nmtStateChange;

    switch (pEvent_p->eventType)
    {
        case kEventTypeNmtEvent:
            nmtEvent = *((const tNmtEvent*)pEvent_p->eventArg.pEventArg);
            break;

        case kEventTypeTimer:
            nmtEvent = (tNmtEvent)((const tTimerEventArg*)pEvent_p->eventArg.pEventArg)->argument.value;
            break;

        default:
            return kErrorNmtInvalidEvent;
    }

    // save NMT-State
    // needed for later comparison to inform higher layer about state change
    oldState = nmtkInstance_l.stateIndex;

    // process NMT state machine
    ret = nmtkStates_l[nmtkInstance_l.stateIndex].pfnState(nmtEvent);

    // inform higher layer about State-Change if needed
    if (oldState != nmtkInstance_l.stateIndex)
    {
        DEBUG_LVL_NMTK_TRACE("%s(): (NMT-event = 0x%04X): New NMT-State = 0x%03X\n",
                             __func__,
                             nmtEvent,
                             nmtkStates_l[nmtkInstance_l.stateIndex].nmtState);

        nmtStateChange.newNmtState = nmtkStates_l[nmtkInstance_l.stateIndex].nmtState;
        nmtStateChange.oldNmtState = nmtkStates_l[oldState].nmtState;
        nmtStateChange.nmtEvent = nmtEvent;

#if defined(CONFIG_INCLUDE_LEDK)
        //ledk state change
        ret = ledk_handleNmtStateChange(&nmtStateChange);
        if (ret != kErrorOk)
           return ret;
#endif

        // inform DLLk module about state change
        event.eventType = kEventTypeNmtStateChange;
        OPLK_MEMSET(&event.netTime, 0x00, sizeof(event.netTime));
        event.eventArg.pEventArg = &nmtStateChange;
        event.eventArgSize = sizeof(nmtStateChange);
        event.eventSink = kEventSinkDllk;

        ret = dllk_process(&event);
        if (ret != kErrorOk)
           return ret;

        // inform higher layer about state change
        event.eventSink = kEventSinkNmtu;
        ret = eventk_postEvent(&event);
    }

    return ret;
}


//=========================================================================//
//                                                                         //
//          P R I V A T E   D E F I N I T I O N S                          //
//                                                                         //
//=========================================================================//
/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief  Process State GS_OFF

The function processes the NMT state GS_OFF.

\param[in]      nmtEvent_p          NMT event to be processed.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError doStateGsOff(tNmtEvent nmtEvent_p)
{
    if (nmtEvent_p == kNmtEventSwReset)
    {   // NMT_GT8, NMT_GT1 -> new state kNmtGsInitialising
        nmtkInstance_l.stateIndex = kNmtkGsInitialising;
    }

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Process State GS_INITIALISING

The function processes the NMT state GS_INITIALISING.
In this state the first init of the hardware will be done.

\param[in]      nmtEvent_p          NMT event to be processed.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError doStateGsInitialising(tNmtEvent nmtEvent_p)
{
    switch (nmtEvent_p)
    {
        // 2006/07/31 d.k.: react also on NMT reset commands in ResetApp state
        // NMT Command SwitchOff
        case kNmtEventCriticalError:
        case kNmtEventSwitchOff:
            // NMT_GT3
            nmtkInstance_l.stateIndex = kNmtkGsOff;
            break;

        // new state kNmtGsResetApplication
        case kNmtEventEnterResetApp:
            // NMT_GT10
            nmtkInstance_l.stateIndex = kNmtkGsResetApplication;
            break;

        default:
            break;
    }

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Process State GS_RESET_APPLICATION

The function processes the NMT state GS_RESET_APPLICATION.
In this state the initialization of the manufacturer-specific profile area
and the standardized device profile area is done.

\param[in]      nmtEvent_p          NMT event to be processed.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError doStateGsResetApplication(tNmtEvent nmtEvent_p)
{
    switch (nmtEvent_p)
    {
        // 2006/07/31 d.k.: react also on NMT reset commands in ResetApp state
        // NMT Command SwitchOff
        case kNmtEventCriticalError:
        case kNmtEventSwitchOff:
            // NMT_GT3
            nmtkInstance_l.stateIndex = kNmtkGsOff;
            break;

        // NMT Command SwReset
        case kNmtEventSwReset:
            // NMT_GT8
            nmtkInstance_l.stateIndex = kNmtkGsInitialising;
            break;

        // leave this state only if higher layer
        // say so
        case kNmtEventEnterResetCom:
            // NMT_GT11
            // new state kNmtGsResetCommunication
            nmtkInstance_l.stateIndex = kNmtkGsResetCommunication;
            break;

        default:
            break;
    }

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Process State GS_RESET_COMMUNICATION

The function processes the NMT state GS_RESET_COMMUNICATION.
In this state the initialization of the communication profile area is done.

\param[in]      nmtEvent_p          NMT event to be processed.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError doStateGsResetCommunication(tNmtEvent nmtEvent_p)
{
    switch (nmtEvent_p)
    {
        // 2006/07/31 d.k.: react also on NMT reset commands in ResetComm state
        // NMT Command SwitchOff
        case kNmtEventCriticalError:
        case kNmtEventSwitchOff:
            // NMT_GT3
            nmtkInstance_l.stateIndex = kNmtkGsOff;
            break;

        // NMT Command SwReset
        case kNmtEventSwReset:
            // NMT_GT8
            nmtkInstance_l.stateIndex = kNmtkGsInitialising;
            break;

        // NMT Command ResetNode
        case kNmtEventResetNode:
            // NMT_GT4
            nmtkInstance_l.stateIndex = kNmtkGsResetApplication;
            break;

        // leave this state only if higher layer
        // say so
        case kNmtEventEnterResetConfig:
            // NMT_GT12 -> new state kNmtGsResetConfiguration
            nmtkInstance_l.stateIndex = kNmtkGsResetConfiguration;
            break;

        default:
            break;
    }

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Process State GS_RESET_CONFIGURATION

The function processes the NMT state GS_RESET_CONFIGURATION.
In this state we build the configuration with information from the OD.

\param[in]      nmtEvent_p          NMT event to be processed.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError doStateGsResetConfiguration(tNmtEvent nmtEvent_p)
{
    // reset flags
    nmtkInstance_l.fEnableReadyToOperate = FALSE;
    nmtkInstance_l.fAppReadyToOperate = FALSE;
    nmtkInstance_l.fFrozen = FALSE;

    // check events
    switch (nmtEvent_p)
    {
        // 2006/07/31 d.k.: react also on NMT reset commands in ResetConf state
        // NMT Command SwitchOff
        case kNmtEventCriticalError:
        case kNmtEventSwitchOff:
            // NMT_GT3
            nmtkInstance_l.stateIndex = kNmtkGsOff;
            break;

        // NMT Command SwReset
        case kNmtEventSwReset:
            // NMT_GT8
            nmtkInstance_l.stateIndex = kNmtkGsInitialising;
            break;

        // NMT Command ResetNode
        case kNmtEventResetNode:
            // NMT_GT4
            nmtkInstance_l.stateIndex = kNmtkGsResetApplication;
            break;

        // NMT Command ResetCommunication
        case kNmtEventResetCom:
            // NMT_GT5
            nmtkInstance_l.stateIndex = kNmtkGsResetCommunication;
            break;

        // leave this state only if higher layer says so
        case kNmtEventEnterCsNotActive:
            // Node should be CN (NMT_CT1)
            nmtkInstance_l.stateIndex = kNmtkCsNotActive;
            break;

        case kNmtEventEnterMsNotActive:
            // Node should be MN (NMT_MT1)
#if !defined(CONFIG_INCLUDE_NMT_MN)
                // no MN functionality
                // TODO: -create error E_NMT_BA1_NO_MN_SUPPORT
                nmtkInstance_l.fFrozen = TRUE;
#else
                nmtkInstance_l.stateIndex = kNmtkMsNotActive;
#endif
            break;

#if defined(CONFIG_INCLUDE_NMT_RMN)
        case kNmtEventEnterRmsNotActive:
            // Node should be RMN (NMT_RMT1)
            nmtkInstance_l.stateIndex = kNmtkRmsNotActive;
            nmtkInstance_l.fRedundancy = TRUE;
            break;

#endif

        default:
            break;
    }

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Process State CS_NOT_ACTIVE

The function processes the NMT state CS_NOT_ACTIVE.
In this state the node listens for POWERLINK frames and checks timeout.

\param[in]      nmtEvent_p          NMT event to be processed.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError doStateCsNotActive(tNmtEvent nmtEvent_p)
{
    switch (nmtEvent_p)
    {
        // 2006/07/31 d.k.: react also on NMT reset commands in NotActive state
        // NMT Command SwitchOff
        case kNmtEventCriticalError:
        case kNmtEventSwitchOff:
            // NMT_GT3
            nmtkInstance_l.stateIndex = kNmtkGsOff;
            break;

        // NMT Command SwReset
        case kNmtEventSwReset:
            // NMT_GT8
            nmtkInstance_l.stateIndex = kNmtkGsInitialising;
            break;

        // NMT Command ResetNode
        case kNmtEventResetNode:
            // NMT_GT4
            nmtkInstance_l.stateIndex = kNmtkGsResetApplication;
            break;

        // NMT Command ResetCommunication or internal Communication error
        case kNmtEventResetCom:          // NMT_GT5
        case kNmtEventInternComError:    // NMT_GT6
            nmtkInstance_l.stateIndex = kNmtkGsResetCommunication;
            break;

        // NMT Command Reset Configuration
        case kNmtEventResetConfig:
            // NMT_GT7
            nmtkInstance_l.stateIndex = kNmtkGsResetConfiguration;
            break;

        // see if SoA or SoC received
        case kNmtEventDllCeSoc:
        case kNmtEventDllCeSoa:
            // NMT_CT2 -> new state PRE_OPERATIONAL1
            nmtkInstance_l.stateIndex = kNmtkCsPreOperational1;
            break;

        // timeout for SoA and Soc
        case kNmtEventTimerBasicEthernet:
            // NMT_CT3 -> new state BASIC_ETHERNET
            nmtkInstance_l.stateIndex = kNmtkCsBasicEthernet;
            break;

        default:
            break;
    }

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Process State CS_PRE_OPERATIONAL1

The function processes the NMT state CS_PRE_OPERATIONAL1.
In this state the node processes only async frames.

\param[in]      nmtEvent_p          NMT event to be processed.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError doStateCsPreOperational1(tNmtEvent nmtEvent_p)
{
    switch (nmtEvent_p)
    {
        // NMT Command SwitchOff
        case kNmtEventCriticalError:
        case kNmtEventSwitchOff:
            // NMT_GT3
            nmtkInstance_l.stateIndex = kNmtkGsOff;
            break;

        // NMT Command SwReset
        case kNmtEventSwReset:
            // NMT_GT8
            nmtkInstance_l.stateIndex = kNmtkGsInitialising;
            break;

        // NMT Command ResetNode
        case kNmtEventResetNode:
            // NMT_GT4
            nmtkInstance_l.stateIndex = kNmtkGsResetApplication;
            break;

        // NMT Command ResetCommunication
        // or internal Communication error
        case kNmtEventResetCom:          // NMT_GT5
        case kNmtEventInternComError:    // NMT_GT6
            nmtkInstance_l.stateIndex = kNmtkGsResetCommunication;
            break;

        // NMT Command Reset Configuration
        case kNmtEventResetConfig:
            // NMT_GT7
            nmtkInstance_l.stateIndex = kNmtkGsResetConfiguration;
            break;

        // check if SoC received
        case kNmtEventDllCeSoc:
            // NMT_CT4
            nmtkInstance_l.stateIndex = kNmtkCsPreOperational2;
            break;

#if defined(CONFIG_INCLUDE_NMT_RMN)
        case kNmtEventDllReSwitchOverTimeout:
            // NMT_RMT4
            nmtkInstance_l.stateIndex = kNmtkMsPreOperational1;
            break;
#endif

        default:
            break;
    }

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Process State CS_PRE_OPERATIONAL2

The function processes the NMT state CS_PRE_OPERATIONAL2.
In this state the node processes isochronous and asynchronous frames.

\param[in]      nmtEvent_p          NMT event to be processed.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError doStateCsPreOperational2(tNmtEvent nmtEvent_p)
{
    switch (nmtEvent_p)
    {
        // NMT Command SwitchOff
        case kNmtEventCriticalError:
        case kNmtEventSwitchOff:
            // NMT_GT3
            nmtkInstance_l.stateIndex = kNmtkGsOff;
            break;

        // NMT Command SwReset
        case kNmtEventSwReset:
            // NMT_GT8
            nmtkInstance_l.stateIndex = kNmtkGsInitialising;
            break;

        // NMT Command ResetNode
        case kNmtEventResetNode:
            // NMT_GT4
            nmtkInstance_l.stateIndex = kNmtkGsResetApplication;
            break;

        // NMT Command ResetCommunication
        // or internal Communication error
        case kNmtEventResetCom:          // NMT_GT5
        case kNmtEventInternComError:    // NMT_GT6
            nmtkInstance_l.stateIndex = kNmtkGsResetCommunication;
            break;

        // NMT Command Reset Configuration
        case kNmtEventResetConfig:
            // NMT_GT7
            nmtkInstance_l.stateIndex = kNmtkGsResetConfiguration;
            break;

        // NMT Command StopNode
        case kNmtEventStopNode:
            // NMT_CT8 - reset flags
            nmtkInstance_l.fEnableReadyToOperate = FALSE;
            nmtkInstance_l.fAppReadyToOperate = FALSE;
            nmtkInstance_l.stateIndex = kNmtkCsStopped;
            break;

        // error occurred
        case kNmtEventNmtCycleError:
            // NMT_CT11 - reset flags
            nmtkInstance_l.fEnableReadyToOperate = FALSE;
            nmtkInstance_l.fAppReadyToOperate = FALSE;
            nmtkInstance_l.stateIndex = kNmtkCsPreOperational1;
            break;

        // check if application is ready to operate
        case kNmtEventEnterReadyToOperate:
            // check if command NMTEnableReadyToOperate from MN was received
            if (nmtkInstance_l.fEnableReadyToOperate != FALSE)
            {   // reset flags
                nmtkInstance_l.fEnableReadyToOperate = FALSE;
                nmtkInstance_l.fAppReadyToOperate = FALSE;
                // change state (NMT_CT6)
                nmtkInstance_l.stateIndex = kNmtkCsReadyToOperate;
            }
            else
            {   // set Flag (NMT_CT5)
                nmtkInstance_l.fAppReadyToOperate = TRUE;
            }
            break;

        // NMT Commando EnableReadyToOperate
        case kNmtEventEnableReadyToOperate:
            // check if application is ready
            if (nmtkInstance_l.fAppReadyToOperate != FALSE)
            {   // reset flags
                nmtkInstance_l.fEnableReadyToOperate = FALSE;
                nmtkInstance_l.fAppReadyToOperate = FALSE;
                // change state (NMT_CT6)
                nmtkInstance_l.stateIndex = kNmtkCsReadyToOperate;
            }
            else
            {   // set Flag (NMT_CT5)
                nmtkInstance_l.fEnableReadyToOperate = TRUE;
            }
            break;

#if defined(CONFIG_INCLUDE_NMT_RMN)
        case kNmtEventDllReSwitchOverTimeout:
            // NMT_RMT4
            nmtkInstance_l.stateIndex = kNmtkMsPreOperational1;
            break;
#endif

        default:
            break;
    }

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Process State CS_READY_TO_OPERATE

The function processes the NMT state CS_READY_TO_OPERATE.
In this state the node should be configured and application is ready.

\param[in]      nmtEvent_p          NMT event to be processed.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError doStateCsReadyToOperate(tNmtEvent nmtEvent_p)
{
    switch (nmtEvent_p)
    {
        // NMT Command SwitchOff
        case kNmtEventCriticalError:
        case kNmtEventSwitchOff:
            // NMT_GT3
            nmtkInstance_l.stateIndex = kNmtkGsOff;
            break;

        // NMT Command SwReset
        case kNmtEventSwReset:
            // NMT_GT8
            nmtkInstance_l.stateIndex = kNmtkGsInitialising;
            break;

        // NMT Command ResetNode
        case kNmtEventResetNode:
            // NMT_GT4
            nmtkInstance_l.stateIndex = kNmtkGsResetApplication;
            break;

        // NMT Command ResetCommunication or internal Communication error
        case kNmtEventResetCom:          // NMT_GT5
        case kNmtEventInternComError:    // NMT_GT6
            nmtkInstance_l.stateIndex = kNmtkGsResetCommunication;
            break;

        // NMT Command ResetConfiguration
        case kNmtEventResetConfig:
            // NMT_GT7
            nmtkInstance_l.stateIndex = kNmtkGsResetConfiguration;
            break;

        // NMT Command StopNode
        case kNmtEventStopNode:
            // NMT_CT8
            nmtkInstance_l.stateIndex = kNmtkCsStopped;
            break;

        // error occurred
        case kNmtEventNmtCycleError:
            // NMT_CT11
            nmtkInstance_l.stateIndex = kNmtkCsPreOperational1;
            break;

        // NMT Command StartNode
        case kNmtEventStartNode:
            // NMT_CT7
            nmtkInstance_l.stateIndex = kNmtkCsOperational;
            break;

#if defined(CONFIG_INCLUDE_NMT_RMN)
        case kNmtEventDllReSwitchOverTimeout:
            // NMT_RMT4
            nmtkInstance_l.stateIndex = kNmtkMsPreOperational1;
            break;
#endif

        default:
            break;
    }

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Process State CS_OPERATIONAL

The function processes the NMT state CS_OPERATIONAL.
This is the normal working state of a CN.

\param[in]      nmtEvent_p          NMT event to be processed.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError doStateCsOperational(tNmtEvent nmtEvent_p)
{
    switch (nmtEvent_p)
    {
        // NMT Command SwitchOff
        case kNmtEventCriticalError:
        case kNmtEventSwitchOff:
            // NMT_GT3
            nmtkInstance_l.stateIndex = kNmtkGsOff;
            break;

        // NMT Command SwReset
        case kNmtEventSwReset:
            // NMT_GT8
            nmtkInstance_l.stateIndex = kNmtkGsInitialising;
            break;

        // NMT Command ResetNode
        case kNmtEventResetNode:
            // NMT_GT4
            nmtkInstance_l.stateIndex = kNmtkGsResetApplication;
            break;

        // NMT Command ResetCommunication or internal Communication error
        case kNmtEventResetCom:          // NMT_GT5
        case kNmtEventInternComError:    // NMT_GT6
            nmtkInstance_l.stateIndex = kNmtkGsResetCommunication;
            break;

        // NMT Command ResetConfiguration
        case kNmtEventResetConfig:
            // NMT_GT7
            nmtkInstance_l.stateIndex = kNmtkGsResetConfiguration;
            break;

        // NMT Command StopNode
        case kNmtEventStopNode:
            // NMT_CT8
            nmtkInstance_l.stateIndex = kNmtkCsStopped;
            break;

        // NMT Command EnterPreOperational2
        case kNmtEventEnterPreOperational2:
            // NMT_CT9
            nmtkInstance_l.stateIndex = kNmtkCsPreOperational2;
            break;

        // error occurred
        case kNmtEventNmtCycleError:
            // NMT_CT11
            nmtkInstance_l.stateIndex = kNmtkCsPreOperational1;
            break;

#if defined(CONFIG_INCLUDE_NMT_RMN)
        case kNmtEventDllReSwitchOverTimeout:
            // NMT_RMT6
            nmtkInstance_l.stateIndex = kNmtkMsOperational;
            break;
#endif

        default:
            break;
    }

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Process State CS_NOT_ACTIVE

The function processes the NMT state CS_NOT_ACTIVE.
In this state the node is stopped by the MN it processes only asynchronous
frames.

\param[in]      nmtEvent_p          NMT event to be processed.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError doStateCsStopped(tNmtEvent nmtEvent_p)
{
    switch (nmtEvent_p)
    {
        // NMT Command SwitchOff
        case kNmtEventCriticalError:
        case kNmtEventSwitchOff:
            // NMT_GT3
            nmtkInstance_l.stateIndex = kNmtkGsOff;
            break;

        // NMT Command SwReset
        case kNmtEventSwReset:
            // NMT_GT8
            nmtkInstance_l.stateIndex = kNmtkGsInitialising;
            break;

        // NMT Command ResetNode
        case kNmtEventResetNode:
            // NMT_GT4
            nmtkInstance_l.stateIndex = kNmtkGsResetApplication;
            break;

        // NMT Command ResetCommunication or internal Communication error
        case kNmtEventResetCom:          // NMT_GT5
        case kNmtEventInternComError:    // NMT_GT6
            nmtkInstance_l.stateIndex = kNmtkGsResetCommunication;
            break;

        // NMT Command ResetConfiguration
        case kNmtEventResetConfig:
        {   // NMT_GT7
            nmtkInstance_l.stateIndex = kNmtkGsResetConfiguration;
            break;
        }

        // NMT Command EnterPreOperational2
        case kNmtEventEnterPreOperational2:
            // NMT_CT10
            nmtkInstance_l.stateIndex = kNmtkCsPreOperational2;
            break;

        // error occurred
        case kNmtEventNmtCycleError:
            // NMT_CT11
            nmtkInstance_l.stateIndex = kNmtkCsPreOperational1;
            break;

#if defined(CONFIG_INCLUDE_NMT_RMN)
        case kNmtEventDllReSwitchOverTimeout:
            // NMT_RMT4
            nmtkInstance_l.stateIndex = kNmtkMsPreOperational1;
            break;
#endif

        default:
            break;
    }

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Process State CS_BASIC_ETHERNET

The function processes the NMT state CS_BASIC_ETHERNET.
In this state there is no POWERLINK cycle and the node performs normal Ethernet
communication.

\param[in]      nmtEvent_p          NMT event to be processed.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError doStateCsBasicEthernet(tNmtEvent nmtEvent_p)
{
    switch (nmtEvent_p)
    {
        // NMT Command SwitchOff
        case kNmtEventCriticalError:
        case kNmtEventSwitchOff:
            // NMT_GT3
            nmtkInstance_l.stateIndex = kNmtkGsOff;
            break;

        // NMT Command SwReset
        case kNmtEventSwReset:
            // NMT_GT8
            nmtkInstance_l.stateIndex = kNmtkGsInitialising;
            break;

        // NMT Command ResetNode
        case kNmtEventResetNode:
            // NMT_GT4
            nmtkInstance_l.stateIndex = kNmtkGsResetApplication;
            break;

        // NMT Command ResetCommunication or internal Communication error
        case kNmtEventResetCom:          // NMT_GT5
        case kNmtEventInternComError:    // NMT_GT6
            nmtkInstance_l.stateIndex = kNmtkGsResetCommunication;
            break;

        // NMT Command ResetConfiguration
        case kNmtEventResetConfig:
            // NMT_GT7
            nmtkInstance_l.stateIndex = kNmtkGsResetConfiguration;
            break;

        // error occurred
        // d.k.: how does this error occur? on CRC errors
/*      case kNmtEventNmtCycleError:
            nmtkInstance_l.stateIndex = kNmtkCsPreOperational1:
            break;
*/
        case kNmtEventDllCeSoc:
        case kNmtEventDllCePreq:
        case kNmtEventDllCePres:
        case kNmtEventDllCeSoa:
            // NMT_CT12 - POWERLINK frame on net -> stop any communication
            nmtkInstance_l.stateIndex = kNmtkCsPreOperational1;
            break;

        default:
            break;
    }

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Process State MS_NOT_ACTIVE

The function processes the NMT state MS_NOT_ACTIVE.
In this state the MN listens to the network. If there is no POWERLINK traffic,
the node goes to the next state.

\param[in]      nmtEvent_p          NMT event to be processed.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError doStateMsNotActive(tNmtEvent nmtEvent_p)
{
#if !defined(CONFIG_INCLUDE_NMT_MN)
    UNUSED_PARAMETER(nmtEvent_p);

    // no MN functionality
    // TODO: -create error E_NMT_BA1_NO_MN_SUPPORT
    nmtkInstance_l.fFrozen = TRUE;
#else
    switch (nmtEvent_p)
    {
        // NMT Command SwitchOff
        case kNmtEventCriticalError:
        case kNmtEventSwitchOff:
            // NMT_GT3
            nmtkInstance_l.stateIndex = kNmtkGsOff;
            break;

        // NMT Command SwReset
        case kNmtEventSwReset:
            // NMT_GT8
            nmtkInstance_l.stateIndex = kNmtkGsInitialising;
            break;

        // NMT Command ResetNode
        case kNmtEventResetNode:
            // NMT_GT4
            nmtkInstance_l.stateIndex = kNmtkGsResetApplication;
            break;

        // NMT Command ResetCommunication or internal Communication error
        case kNmtEventResetCom:          // NMT_GT5
        case kNmtEventInternComError:    // NMT_GT6
            nmtkInstance_l.stateIndex = kNmtkGsResetCommunication;
            break;

        // NMT Command ResetConfiguration
        case kNmtEventResetConfig:
            // NMT_GT7
            nmtkInstance_l.stateIndex = kNmtkGsResetConfiguration;
            break;

        // POWERLINK frames received
        case kNmtEventDllCeSoc:
        case kNmtEventDllCeSoa:
            // other MN in network
            // $$$ d.k.: generate error history entry
            nmtkInstance_l.fFrozen = TRUE;
            break;

        // timeout event
        case kNmtEventTimerBasicEthernet:
           // NMT_MT7
            if (nmtkInstance_l.fFrozen == FALSE)
            {   // new state BasicEthernet
                nmtkInstance_l.stateIndex = kNmtkMsBasicEthernet;
            }
            break;

        // timeout event
        case kNmtEventTimerMsPreOp1:
        {   // NMT_MT2
            if (nmtkInstance_l.fFrozen == FALSE)
            {   // new state PreOp1
                nmtkInstance_l.stateIndex = kNmtkMsPreOperational1;
                nmtkInstance_l.fTimerMsPreOp2 = FALSE;
                nmtkInstance_l.fAllMandatoryCNIdent = FALSE;
            }
            break;
        }

        default:
            break;
    }
#endif

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Process State RMS_NOT_ACTIVE

The function processes the NMT state RMS_NOT_ACTIVE.
In this state the RMN listens to the network. If there is no POWERLINK traffic,
the node goes to the next state.

\param[in]      nmtEvent_p          NMT event to be processed.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError doStateRmsNotActive(tNmtEvent nmtEvent_p)
{
#if !defined(CONFIG_INCLUDE_NMT_RMN)
    UNUSED_PARAMETER(nmtEvent_p);

    // no MN functionality
    // TODO: -create error E_NMT_BA1_NO_MN_SUPPORT
    nmtkInstance_l.fFrozen = TRUE;
#else
    switch (nmtEvent_p)
    {
        // NMT Command SwitchOff
        case kNmtEventCriticalError:
        case kNmtEventSwitchOff:
            // NMT_GT3
            nmtkInstance_l.stateIndex = kNmtkGsOff;
            break;

        // NMT Command SwReset
        case kNmtEventSwReset:
            // NMT_GT8
            nmtkInstance_l.stateIndex = kNmtkGsInitialising;
            break;

        // NMT Command ResetNode
        case kNmtEventResetNode:
            // NMT_GT4
            nmtkInstance_l.stateIndex = kNmtkGsResetApplication;
            break;

        // NMT Command ResetCommunication or internal Communication error
        case kNmtEventResetCom:          // NMT_GT5
        case kNmtEventInternComError:    // NMT_GT6
            nmtkInstance_l.stateIndex = kNmtkGsResetCommunication;
            break;

        // NMT Command ResetConfiguration
        case kNmtEventResetConfig:
            // NMT_GT7
            nmtkInstance_l.stateIndex = kNmtkGsResetConfiguration;
            break;

        // POWERLINK frames received
        case kNmtEventDllCeSoc:
        case kNmtEventDllCeSoa:
        case kNmtEventDllReAmni:
            // NMT_RMT3
            nmtkInstance_l.stateIndex = kNmtkCsPreOperational1;
            break;

        // timeout event
        case kNmtEventTimerMsPreOp1:
            // NMT_RMT2
            if (nmtkInstance_l.fFrozen == FALSE)
            {   // new state PreOp1
                nmtkInstance_l.stateIndex = kNmtkMsPreOperational1;
                nmtkInstance_l.fTimerMsPreOp2 = FALSE;
                nmtkInstance_l.fAllMandatoryCNIdent = FALSE;
            }
            break;

        default:
            break;
    }
#endif

    return kErrorOk;
}

#if defined(CONFIG_INCLUDE_NMT_MN)
//------------------------------------------------------------------------------
/**
\brief  Process State MS_PRE_OPERATIONAL1

The function processes the NMT state MS_PRE_OPERATIONAL1.
In this state the MN processes the reduced POWERLINK cycle.

\param[in]      nmtEvent_p          NMT event to be processed.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError doStateMsPreOperational1(tNmtEvent nmtEvent_p)
{
    switch (nmtEvent_p)
    {
        // NMT Command SwitchOff
        case kNmtEventCriticalError:
        case kNmtEventSwitchOff:
            // NMT_GT3
            nmtkInstance_l.stateIndex = kNmtkGsOff;
            break;

        // NMT Command SwReset
        case kNmtEventSwReset:
            // NMT_GT8
            nmtkInstance_l.stateIndex = kNmtkGsInitialising;
            break;

        // NMT Command ResetNode
        case kNmtEventResetNode:
            // NMT_GT4
            nmtkInstance_l.stateIndex = kNmtkGsResetApplication;
            break;

        // NMT Command ResetCommunication or internal Communication error
        case kNmtEventResetCom:          // NMT_GT5
        case kNmtEventInternComError:    // NMT_GT6
            nmtkInstance_l.stateIndex = kNmtkGsResetCommunication;
            break;

        // NMT Command ResetConfiguration
        case kNmtEventResetConfig:
            // NMT_GT7
            nmtkInstance_l.stateIndex = kNmtkGsResetConfiguration;
            break;

        // POWERLINK frames received
        case kNmtEventDllCeSoc:
        case kNmtEventDllCeSoa:
#if defined(CONFIG_INCLUDE_NMT_RMN)
        case kNmtEventDllReAmni:
        case kNmtEventGoToStandby:
        case kNmtEventGoToStandbyDelayed:
            if (nmtkInstance_l.fRedundancy)
            {
                nmtkInstance_l.stateIndex = kNmtkCsPreOperational1;
                break;
            }
#endif
            // other MN in network
            // $$$ d.k.: generate error history entry
            nmtkInstance_l.stateIndex = kNmtkGsResetCommunication;
            break;

        // error occurred
        // d.k. MSPreOp1->CSPreOp1: nonsense -> keep state
        /*
        case kNmtEventNmtCycleError:
            nmtkInstance_l.stateIndex = kNmtkCsPreOperational1;
            break;
        */

        case kNmtEventAllMandatoryCNIdent:
            // all mandatory CN identified
            if (nmtkInstance_l.fTimerMsPreOp2 != FALSE)
            {   // NMT_MT3
                nmtkInstance_l.stateIndex = kNmtkMsPreOperational2;
            }
            else
            {
                nmtkInstance_l.fAllMandatoryCNIdent = TRUE;
            }
            break;

        case kNmtEventTimerMsPreOp2:
            // residence time for PreOp1 is elapsed
            if (nmtkInstance_l.fAllMandatoryCNIdent != FALSE)
            {   // NMT_MT3
                nmtkInstance_l.stateIndex = kNmtkMsPreOperational2;
            }
            else
            {
                nmtkInstance_l.fTimerMsPreOp2 = TRUE;
            }
            break;

        default:
            break;
    }

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Process State MS_PRE_OPERATIONAL2

The function processes the NMT state MS_PRE_OPERATIONAL2.
In this state the MN processes the full POWERLINK cycle.

\param[in]      nmtEvent_p          NMT event to be processed.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError doStateMsPreOperational2(tNmtEvent nmtEvent_p)
{
    switch (nmtEvent_p)
    {
        // NMT Command SwitchOff
        case kNmtEventCriticalError:
        case kNmtEventSwitchOff:
            // NMT_GT3
            nmtkInstance_l.stateIndex = kNmtkGsOff;
            break;

        // NMT Command SwReset
        case kNmtEventSwReset:
            // NMT_GT8
            nmtkInstance_l.stateIndex = kNmtkGsInitialising;
            break;

        // NMT Command ResetNode
        case kNmtEventResetNode:
            // NMT_GT4
            nmtkInstance_l.stateIndex = kNmtkGsResetApplication;
            break;

        // NMT Command ResetCommunication or internal Communication error
        case kNmtEventResetCom:          // NMT_GT5
        case kNmtEventInternComError:    // NMT_GT6
            nmtkInstance_l.stateIndex = kNmtkGsResetCommunication;
            break;

        // NMT Command ResetConfiguration
        case kNmtEventResetConfig:
            // NMT_GT7
            nmtkInstance_l.stateIndex = kNmtkGsResetConfiguration;
            break;

        // POWERLINK frames received
        case kNmtEventDllCeSoc:
        case kNmtEventDllCeSoa:
#if defined(CONFIG_INCLUDE_NMT_RMN)
        case kNmtEventDllReAmni:
        case kNmtEventGoToStandby:
        case kNmtEventGoToStandbyDelayed:
            if (nmtkInstance_l.fRedundancy)
            {
                nmtkInstance_l.stateIndex = kNmtkCsPreOperational1;
                break;
            }
#endif
            // other MN in network
            // $$$ d.k.: generate error history entry
            nmtkInstance_l.stateIndex = kNmtkGsResetCommunication;
            break;

        // error occurred
        case kNmtEventNmtCycleError:
            // NMT_MT6
            nmtkInstance_l.stateIndex = kNmtkMsPreOperational1;
            break;

        case kNmtEventEnterReadyToOperate:
            // NMT_MT4
            nmtkInstance_l.stateIndex = kNmtkMsReadyToOperate;
            break;

        default:
            break;
    }

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Process State MS_READY_TO_OPERATE

The function processes the NMT state MS_READY_TO_OPERATE.
In this state all mandatory CNs are ready to operate. The MN processes the full
POWERLINK cycle.

\param[in]      nmtEvent_p          NMT event to be processed.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError doStateMsReadyToOperate(tNmtEvent nmtEvent_p)
{
    switch (nmtEvent_p)
    {
        // NMT Command SwitchOff
        case kNmtEventCriticalError:
        case kNmtEventSwitchOff:
            // NMT_GT3
            nmtkInstance_l.stateIndex = kNmtkGsOff;
            break;

        // NMT Command SwReset
        case kNmtEventSwReset:
            // NMT_GT8
            nmtkInstance_l.stateIndex = kNmtkGsInitialising;
            break;

        // NMT Command ResetNode
        case kNmtEventResetNode:
            // NMT_GT4
            nmtkInstance_l.stateIndex = kNmtkGsResetApplication;
            break;

        // NMT Command ResetCommunication or internal Communication error
        case kNmtEventResetCom:          // NMT_GT5
        case kNmtEventInternComError:    // NMT_GT6
            nmtkInstance_l.stateIndex = kNmtkGsResetCommunication;
            break;

        // NMT Command ResetConfiguration
        case kNmtEventResetConfig:
            // NMT_GT7
            nmtkInstance_l.stateIndex = kNmtkGsResetConfiguration;
            break;

        // POWERLINK frames received
        case kNmtEventDllCeSoc:
        case kNmtEventDllCeSoa:
#if defined(CONFIG_INCLUDE_NMT_RMN)
        case kNmtEventDllReAmni:
        case kNmtEventGoToStandby:
        case kNmtEventGoToStandbyDelayed:
            if (nmtkInstance_l.fRedundancy)
            {
                nmtkInstance_l.stateIndex = kNmtkCsPreOperational1;
                break;
            }
#endif
            // other MN in network
            // $$$ d.k.: generate error history entry
            nmtkInstance_l.stateIndex = kNmtkGsResetCommunication;
            break;

        // error occurred
        case kNmtEventNmtCycleError:
            // NMT_MT6
            nmtkInstance_l.stateIndex = kNmtkMsPreOperational1;
            break;

        case kNmtEventEnterMsOperational:
            // NMT_MT5
            nmtkInstance_l.stateIndex = kNmtkMsOperational;
            break;

        default:
            break;
    }

    return kErrorOk;
}
//------------------------------------------------------------------------------
/**
\brief  Process State MS_OPERATIONAL

The function processes the NMT state MS_OPERATIONAL.
This is the normal working state of a MN.

\param[in]      nmtEvent_p          NMT event to be processed.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError doStateMsOperational(tNmtEvent nmtEvent_p)
{
    switch (nmtEvent_p)
    {
        // NMT Command SwitchOff
        case kNmtEventCriticalError:
        case kNmtEventSwitchOff:
            // NMT_GT3
            nmtkInstance_l.stateIndex = kNmtkGsOff;
            break;

        // NMT Command SwReset
        case kNmtEventSwReset:
            // NMT_GT8
            nmtkInstance_l.stateIndex = kNmtkGsInitialising;
            break;

        // NMT Command ResetNode
        case kNmtEventResetNode:
            // NMT_GT4
            nmtkInstance_l.stateIndex = kNmtkGsResetApplication;
            break;

        // NMT Command ResetCommunication or internal Communication error
        case kNmtEventResetCom:          // NMT_GT5
        case kNmtEventInternComError:    // NMT_GT6
            nmtkInstance_l.stateIndex = kNmtkGsResetCommunication;
            break;

        // NMT Command ResetConfiguration
        case kNmtEventResetConfig:
            // NMT_GT7
            nmtkInstance_l.stateIndex = kNmtkGsResetConfiguration;
            break;

        // POWERLINK frames received
        case kNmtEventDllCeSoc:
        case kNmtEventDllCeSoa:
#if defined(CONFIG_INCLUDE_NMT_RMN)
        case kNmtEventDllReAmni:
        case kNmtEventGoToStandby:
        case kNmtEventGoToStandbyDelayed:
            if (nmtkInstance_l.fRedundancy)
            {
                nmtkInstance_l.stateIndex = kNmtkCsOperational;
                break;
            }
#endif
            // other MN in network
            // $$$ d.k.: generate error history entry
            nmtkInstance_l.stateIndex = kNmtkGsResetCommunication;
            break;

        // error occurred
        case kNmtEventNmtCycleError:
            // NMT_MT6
            nmtkInstance_l.stateIndex = kNmtkMsPreOperational1;
            break;

        default:
            break;
    }

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Process State MS_BASIC_ETHERNET

The function processes the NMT state MS_BASIC_ETHERNET.
In this state the MN processes normal Ethernet traffic.

\param[in]      nmtEvent_p          NMT event to be processed.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError doStateMsBasicEthernet(tNmtEvent nmtEvent_p)
{
    switch (nmtEvent_p)
    {
        // NMT Command SwitchOff
        case kNmtEventCriticalError:
        case kNmtEventSwitchOff:
            // NMT_GT3
            nmtkInstance_l.stateIndex = kNmtkGsOff;
            break;

        // NMT Command SwReset
        case kNmtEventSwReset:
            // NMT_GT8
            nmtkInstance_l.stateIndex = kNmtkGsInitialising;
            break;

        // NMT Command ResetNode
        case kNmtEventResetNode:
            // NMT_GT4
            nmtkInstance_l.stateIndex = kNmtkGsResetApplication;
            break;

        // NMT Command ResetCommunication or internal Communication error
        case kNmtEventResetCom:          // NMT_GT5
        case kNmtEventInternComError:    // NMT_GT6
            nmtkInstance_l.stateIndex = kNmtkGsResetCommunication;
            break;

        // NMT Command ResetConfiguration
        case kNmtEventResetConfig:
            // NMT_GT7
            nmtkInstance_l.stateIndex = kNmtkGsResetConfiguration;
            break;

        // POWERLINK frames received
        case kNmtEventDllCeSoc:
        case kNmtEventDllCeSoa:
            // other MN in network
            // $$$ d.k.: generate error history entry
            nmtkInstance_l.stateIndex = kNmtkGsResetCommunication;
            break;

        // error occurred
        // d.k. BE->PreOp1 on cycle error? No
/*      case kNmtEventNmtCycleError:
            nmtkInstance_l.stateIndex = kNmtkCsPreOperational1;
            break;
*/
        default:
            break;
    }

    return kErrorOk;
}

#endif

/// \}
