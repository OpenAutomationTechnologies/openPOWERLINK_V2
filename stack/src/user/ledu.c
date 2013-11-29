/**
********************************************************************************
\file   ledu.c

\brief  Implementation of user LED module

This file contains the implementation of the user LED module.

\ingroup module_ledu
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
#include <EplInc.h>
#include <user/ledu.h>
#include <user/timeru.h>

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
#define LEDU_DURATION_FLICKERING    50      // [ms]
#define LEDU_DURATION_BLINKING      200     // [ms]
#define LEDU_DURATION_FLASH_ON      200     // [ms]
#define LEDU_DURATION_FLASH_OFF     1000    // [ms]

//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------

/**
 * \brief   Enumeration for valid LED modes
 *
 * The enumeration lists all valid LED modes.
 */
typedef enum
{
    kLeduModeInit           = 0x00,
    kLeduModeOff            = 0x01,
    kLeduModeOn             = 0x02,
    kLeduModeFlickering     = 0x03,
    kLeduModeBlinking       = 0x04,
    kLeduModeSingleFlash    = 0x05,
    kLeduModeDoubleFlash    = 0x06,
    kLeduModeTripleFlash    = 0x07,
} tLeduMode;

/**
 * \brief   User LED module instance
 *
 * The structure defines the instance data of the user LED module.
 */
typedef struct
{
    tEplTimerHdl                timerHdlLedBlink;       ///< Timer for LED blinking
    UINT32                      timerArg;               ///< Argument for timer
    tLeduStateChangeCallback    pfnCbStateChange;       ///< Function pointer to state change function.
    tLeduMode                   statusLedMode;          ///< Mode of the status LED
    /**
     *  0 - long off (e.g. 50 ms while flickering, 200 ms while blinking and 1000 ms while flashing)
     *  1 - on (odd number)
     *  2 - short off (even number)
     */
    UINT                        statusLedState;         ///< State of the status LED
} tLeduInstance;


//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tLeduInstance   leduInstance_g;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static tEplKernel callStateChanged(tLedType LedType_p, BOOL fOn_p);
static tEplKernel changeMode(tLeduMode NewMode_p);

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Initialize user LED module

The function initializes the user LED module.

\param  pfnCbStateChange_p      Pointer to callback function for LED state
                                changes.

\return The function returns a tEplKernel error code.

\ingroup module_ledu
*/
//------------------------------------------------------------------------------
tEplKernel ledu_init(tLeduStateChangeCallback pfnCbStateChange_p)
{
    EPL_MEMSET(&leduInstance_g, 0, sizeof(tLeduInstance));
    leduInstance_g.pfnCbStateChange = pfnCbStateChange_p;

    return kEplSuccessful;
}

//------------------------------------------------------------------------------
/**
\brief  Deinitialize user LED module

The function deinitializes the user LED module.

\return The function returns a tEplKernel error code.

\ingroup module_ledu
*/
//------------------------------------------------------------------------------
tEplKernel ledu_exit(void)
{
    tEplKernel ret = kEplSuccessful;

    ret = timeru_deleteTimer(&leduInstance_g.timerHdlLedBlink);
    EPL_MEMSET(&leduInstance_g, 0, sizeof(tLeduInstance));

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Callback function for NMT state changes

The function implements the LED modules callback function for NMT state
changes.

\param  nmtStateChange_p    NMT state change event.

\return The function returns a tEplKernel error code.

\ingroup module_ledu
*/
//------------------------------------------------------------------------------
tEplKernel ledu_cbNmtStateChange(tEventNmtStateChange nmtStateChange_p)
{
    tEplKernel      ret = kEplSuccessful;

    // activate status LED according to NMT state
    switch (nmtStateChange_p.newNmtState)
    {
        // status LED off
        case kNmtGsOff:
        case kNmtGsInitialising:
        case kNmtGsResetApplication:
        case kNmtGsResetCommunication:
        case kNmtGsResetConfiguration:
        case kNmtCsNotActive:
        case kNmtMsNotActive:
            if (leduInstance_g.statusLedMode != kLeduModeOff)
            {   // state changed
                leduInstance_g.statusLedMode = kLeduModeOff;
                ret = timeru_deleteTimer(&leduInstance_g.timerHdlLedBlink);
                ret = callStateChanged(kLedTypeStatus, FALSE);
            }
            break;

        // status LED single flashing
        case kNmtCsPreOperational1:
        case kNmtMsPreOperational1:
            ret = changeMode(kLeduModeSingleFlash);
            break;

        // status LED double flashing
        case kNmtCsPreOperational2:
        case kNmtMsPreOperational2:
            ret = changeMode(kLeduModeDoubleFlash);
            break;

        // status LED triple flashing
        case kNmtCsReadyToOperate:
        case kNmtMsReadyToOperate:
            ret = changeMode(kLeduModeTripleFlash);
            break;

        // status LED on
        case kNmtCsOperational:
        case kNmtMsOperational:
            if (leduInstance_g.statusLedMode != kLeduModeOn)
            {   // state changed
                leduInstance_g.statusLedMode = kLeduModeOn;
                ret = timeru_deleteTimer(&leduInstance_g.timerHdlLedBlink);
                ret = callStateChanged(kLedTypeStatus, TRUE);
            }
            break;

        // status LED blinking
        case kNmtCsStopped:
            ret = changeMode(kLeduModeBlinking);
            break;

        // status LED flickering
        case kNmtCsBasicEthernet:
        case kNmtMsBasicEthernet:
            ret = changeMode(kLeduModeFlickering);
            break;
    }

    // activate error LED according to NMT event
    switch (nmtStateChange_p.nmtEvent)
    {
        // error LED off
        case kNmtEventSwReset:               // NMT_GT2
        case kNmtEventStartNode:             // NMT_CT7
        case kNmtEventTimerBasicEthernet:    // NMT_CT3
        case kNmtEventEnterMsOperational:    // NMT_MT5
            ret = callStateChanged(kLedTypeError, FALSE);
            break;

        // error LED on
        case kNmtEventNmtCycleError:     // NMT_CT11, NMT_MT6
        case kNmtEventInternComError:    // NMT_GT6
            ret = callStateChanged(kLedTypeError, TRUE);
            break;

        default:
            // do nothing
            break;
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Process events

The function implements the event handler of the LED module.

\param  pEvent_p        Event to process.

\return The function returns a tEplKernel error code.

\ingroup module_ledu
*/
//------------------------------------------------------------------------------
tEplKernel ledu_processEvent(tEplEvent* pEvent_p)
{
    tEplKernel          ret;
    tEplTimerArg        timerArg;
    UINT32              timeout = 0;
    BOOL                fLedOn = FALSE;
    tEplTimerEventArg*  pTimerEventArg;

    ret = kEplSuccessful;

    switch(pEvent_p->m_EventType)
    {
        // timer event
        case kEplEventTypeTimer:
            pTimerEventArg = (tEplTimerEventArg*)pEvent_p->m_pArg;

            if (pTimerEventArg->m_Arg.m_dwVal != leduInstance_g.timerArg)
            {   // zombie timer, ignore it
                break;
            }

            leduInstance_g.statusLedState++;

            // select timeout and new LED state corresponding to mode
            switch (leduInstance_g.statusLedMode)
            {
                case kLeduModeInit:
                case kLeduModeOn:
                case kLeduModeOff:
                    goto Exit;      // should not occur
                    break;

                case kLeduModeFlickering:
                    if (leduInstance_g.statusLedState >= 2)
                    {   // reset state
                        leduInstance_g.statusLedState = 0;
                        fLedOn = FALSE;
                    }
                    else
                    {
                        fLedOn = TRUE;
                    }
                    timeout = LEDU_DURATION_FLICKERING;
                    break;

                case kLeduModeBlinking:
                    if (leduInstance_g.statusLedState >= 2)
                    {   // reset state
                        leduInstance_g.statusLedState = 0;
                        fLedOn = FALSE;
                    }
                    else
                    {
                        fLedOn = TRUE;
                    }
                    timeout = LEDU_DURATION_BLINKING;

                    break;

                case kLeduModeSingleFlash:
                    if (leduInstance_g.statusLedState >= 2)
                    {   // reset state
                        leduInstance_g.statusLedState = 0;
                        timeout = LEDU_DURATION_FLASH_OFF;
                        fLedOn = FALSE;
                    }
                    else
                    {
                        timeout = LEDU_DURATION_FLASH_ON;
                        fLedOn = ((leduInstance_g.statusLedState & 0x01) != 0x00)
                            ? TRUE : FALSE;
                    }
                    break;

                case kLeduModeDoubleFlash:
                    if (leduInstance_g.statusLedState >= 4)
                    {   // reset state
                        leduInstance_g.statusLedState = 0;
                        timeout = LEDU_DURATION_FLASH_OFF;
                        fLedOn = FALSE;
                    }
                    else
                    {
                        timeout = LEDU_DURATION_FLASH_ON;
                        fLedOn = ((leduInstance_g.statusLedState & 0x01) != 0x00)
                            ? TRUE : FALSE;
                    }
                    break;

                case kLeduModeTripleFlash:
                    if (leduInstance_g.statusLedState >= 6)
                    {   // reset state
                        leduInstance_g.statusLedState = 0;
                        timeout = LEDU_DURATION_FLASH_OFF;
                        fLedOn = FALSE;
                    }
                    else
                    {
                        timeout = LEDU_DURATION_FLASH_ON;
                        fLedOn = ((leduInstance_g.statusLedState & 0x01) != 0x00)
                            ? TRUE : FALSE;
                    }
                    break;

            }

            // create new timer
            timerArg.m_EventSink = kEplEventSinkLedu;
            leduInstance_g.timerArg++;
            timerArg.m_Arg.m_dwVal = leduInstance_g.timerArg;
            ret = timeru_modifyTimer(&leduInstance_g.timerHdlLedBlink, timeout, timerArg);

            // call callback function
            ret = callStateChanged(kLedTypeStatus, fLedOn);

            break;

        default:
            ret = kEplNmtInvalidEvent;

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
\brief  Call state changed function

The function calls the registered state change function

\param  ledType_p           The type of LED.
\param  fOn_p               The state of the LED.

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
static tEplKernel callStateChanged(tLedType ledType_p, BOOL fOn_p)
{
    tEplKernel      ret = kEplSuccessful;

    if (leduInstance_g.pfnCbStateChange != NULL)
    {
        ret = leduInstance_g.pfnCbStateChange(ledType_p, fOn_p);
    }
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Change the LED mode

The function changes the LED mode.

\param  newMode_p           The new mode to set.

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
static tEplKernel changeMode(tLeduMode newMode_p)
{
    tEplKernel      ret;
    tLeduMode       oldMode;
    tEplTimerArg    timerArg;
    UINT32          timeout;
    BOOL            fLedOn;

    ret = kEplSuccessful;

    oldMode = leduInstance_g.statusLedMode;

    if (oldMode != newMode_p)
    {   // state changed -> save new mode
        leduInstance_g.statusLedMode = newMode_p;

        // Where are we coming from?
        if (oldMode == kLeduModeOff)
        {   // status LED was off -> switch LED on
            fLedOn = TRUE;
            leduInstance_g.statusLedState = 0xFF;
        }
        else if (oldMode == kLeduModeOn)
        {   // status LED was on -> switch LED off
            fLedOn = FALSE;
            leduInstance_g.statusLedState = 0;
        }
        else
        {   // timer should be up and running
            return ret;
        }

        // select timeout corresponding to mode
        switch (newMode_p)
        {
            case kLeduModeFlickering:
                timeout = LEDU_DURATION_FLICKERING;
                break;

            case kLeduModeBlinking:
                timeout = LEDU_DURATION_BLINKING;
                break;

            case kLeduModeSingleFlash:
            case kLeduModeDoubleFlash:
            case kLeduModeTripleFlash:
                if (fLedOn == FALSE)
                    timeout = LEDU_DURATION_FLASH_OFF;
                else
                    timeout = LEDU_DURATION_FLASH_ON;
                break;

            default:
                return ret;      // should not occur
                break;
        }

        // create new timer
        timerArg.m_EventSink = kEplEventSinkLedu;
        leduInstance_g.timerArg++;
        timerArg.m_Arg.m_dwVal = leduInstance_g.timerArg;
        ret = timeru_modifyTimer(&leduInstance_g.timerHdlLedBlink,
                                     timeout,
                                     timerArg);

        // call callback function
        ret = callStateChanged(kLedTypeStatus, fLedOn);
    }
    return ret;
}

///\}

