/**
********************************************************************************
\file   ledk.c

\brief  Implementation of kernel LED module

This file contains the implementation of the kernel LED module.

\ingroup module_ledk
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2015, Kalycito Infotech Private Limited.
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
#include <kernel/ledk.h>
#include <common/target.h>

#if defined(CONFIG_INCLUDE_LEDK)

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

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Initialize kernel LED module

The function initializes the kernel LED module.

\return The function returns a tOplkError error code.

\ingroup module_ledk
*/
//------------------------------------------------------------------------------
tOplkError ledk_init(void)
{
    return ledk_timerInit();
}

//------------------------------------------------------------------------------
/**
\brief  Cleanup kernel LED module

The function cleans up the kernel LED module.

\return The function returns a tOplkError error code.

\ingroup module_ledk
*/
//------------------------------------------------------------------------------
tOplkError ledk_exit(void)
{
    return ledk_timerExit();
}

//------------------------------------------------------------------------------
/**
\brief  Update status LED mode as per the NMT state change

The function handles the NMT state changes and updates the target
LED mode.

\param[in]      pNmtStateChange_p   NMT state change event.

\return The function returns a tOplkError error code.

\ingroup module_ledk
*/
//------------------------------------------------------------------------------
tOplkError ledk_handleNmtStateChange(const tEventNmtStateChange* pNmtStateChange_p)
{
    tOplkError  ret = kErrorOk;

    // activate status LED according to NMT state
    switch (pNmtStateChange_p->newNmtState)
    {
        // status LED off
        case kNmtGsOff:
        case kNmtGsInitialising:
        case kNmtGsResetApplication:
        case kNmtGsResetCommunication:
        case kNmtGsResetConfiguration:
        case kNmtCsNotActive:
        case kNmtMsNotActive:
        case kNmtRmsNotActive:
            ret = ledk_setLedMode(kLedTypeStatus, kLedModeOff);
            break;

        // status LED single flashing
        case kNmtCsPreOperational1:
        case kNmtMsPreOperational1:
            ret = ledk_setLedMode(kLedTypeStatus, kLedModeSingleFlash);
            break;

        // status LED double flashing
        case kNmtCsPreOperational2:
        case kNmtMsPreOperational2:
            ret = ledk_setLedMode(kLedTypeStatus, kLedModeDoubleFlash);
            break;

        // status LED triple flashing
        case kNmtCsReadyToOperate:
        case kNmtMsReadyToOperate:
            ret = ledk_setLedMode(kLedTypeStatus, kLedModeTripleFlash);
            break;

        // status LED on
        case kNmtCsOperational:
        case kNmtMsOperational:
            ret = ledk_setLedMode(kLedTypeStatus, kLedModeOn);
            break;

        // status LED blinking
        case kNmtCsStopped:
            ret = ledk_setLedMode(kLedTypeStatus, kLedModeBlinking);
            break;

        // status LED flickering
        case kNmtCsBasicEthernet:
        case kNmtMsBasicEthernet:
            ret = ledk_setLedMode(kLedTypeStatus, kLedModeFlickering);
            break;

        default:
            break;
    }

    // activate error LED according to NMT event
    switch (pNmtStateChange_p->nmtEvent)
    {
        // error LED off
        case kNmtEventSwReset:               // NMT_GT2
        case kNmtEventStartNode:             // NMT_CT7
        case kNmtEventTimerBasicEthernet:    // NMT_CT3
        case kNmtEventEnterMsOperational:    // NMT_MT5
            ret = target_setLed(kLedTypeError, FALSE);
            break;

        // error LED on
        case kNmtEventNmtCycleError:     // NMT_CT11, NMT_MT6
        case kNmtEventInternComError:    // NMT_GT6
            ret = target_setLed(kLedTypeError, TRUE);
            break;

        default:
            // do nothing
            break;
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Process to update states

The function is called in loop from the ctrlk module to process led state change.

\return The function returns a tOplkError error code.

\ingroup module_ledk
*/
//------------------------------------------------------------------------------
tOplkError ledk_process(void)
{
    tOplkError      ret;

    ret = ledk_updateLedState();

    return ret;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

/// \}

#endif
