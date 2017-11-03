/**
********************************************************************************
\file   ledktimer.c

\brief  Implementation of status LED handling by target timer.

This file contains the implementation of status LED handling by target timer.
Target timer will configure the timeout value for blinking.

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
#include <common/target.h>
#include <kernel/ledk.h>

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
/**
 * \brief   User LED module instance
 *
 * The structure defines the instance data of the user LED module.
 */
typedef struct
{
    tLedMode            statusLedMode;          ///< Mode of the status LED
    UINT32              statusLedState;         ///< State of the status LED
    UINT32              startTimeInMs;          ///< Holds the start time value of the current led state
    UINT32              timeoutInMs;            ///< Holds the time out value of the current led state
} tLedkInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tLedkInstance   ledkInstance_l;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Initialize kernel LED timer module

The function initializes the kernel LED timer module.

\return The function returns a tOplkError error code.

\ingroup module_ledk
*/
//------------------------------------------------------------------------------
tOplkError ledk_timerInit(void)
{
    OPLK_MEMSET(&ledkInstance_l, 0, sizeof(tLedkInstance));

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Cleanup kernel LED timer module

The function cleans up the kernel LED timer module.

\return The function returns a tOplkError error code.

\ingroup module_ledk
*/
//------------------------------------------------------------------------------
tOplkError ledk_timerExit(void)
{
    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Update Led State

This function updates the status LED state. The blinking is achieved by using the system timer ticks

\return The function returns a tOplkError error code.

\ingroup module_ledk
*/
//------------------------------------------------------------------------------
tOplkError ledk_updateLedState(void)
{
    UINT        timeout;
    UINT32      tickCount;
    BOOL        fLedOn = FALSE;
    tOplkError  ret = kErrorOk;

    // Setting the new timeout value
    tickCount = target_getTickCount();
    timeout = tickCount - ledkInstance_l.startTimeInMs;

    if ((timeout >= ledkInstance_l.timeoutInMs) && (timeout > 0))
    {
        ledkInstance_l.statusLedState++;

        // select timeout and new LED state corresponding to mode
        switch (ledkInstance_l.statusLedMode)
        {
            case kLedModeInit:
            case kLedModeOn:
            case kLedModeOff:
                goto Exit;      // should not occur

            case kLedModeFlickering:
                if (ledkInstance_l.statusLedState >= kLedModeOn)
                {   // reset state
                    ledkInstance_l.statusLedState = kLedModeInit;
                    fLedOn = FALSE;
                }
                else
                {
                    fLedOn = TRUE;
                }

                timeout = LEDK_DURATION_FLICKERING;
                break;

            case kLedModeBlinking:
                if (ledkInstance_l.statusLedState >= kLedModeOn)
                {   // reset state
                    ledkInstance_l.statusLedState = kLedModeInit;
                    fLedOn = FALSE;
                }
                else
                {
                    fLedOn = TRUE;
                }

                timeout = LEDK_DURATION_BLINKING;
                break;

            case kLedModeSingleFlash:
                if (ledkInstance_l.statusLedState >= kLedModeOn)
                {   // reset state
                    ledkInstance_l.statusLedState = kLedModeInit;
                    timeout = LEDK_DURATION_FLASH_OFF;
                    fLedOn = FALSE;
                }
                else
                {
                    timeout = LEDK_DURATION_FLASH_ON;
                    fLedOn = ((ledkInstance_l.statusLedState & 0x01) != 0x00) ?
                        TRUE : FALSE;
                }
                break;

            case kLedModeDoubleFlash:
                if (ledkInstance_l.statusLedState >= kLedModeBlinking)
                {   // reset state
                    ledkInstance_l.statusLedState = kLedModeInit;
                    timeout = LEDK_DURATION_FLASH_OFF;
                    fLedOn = FALSE;
                }
                else
                {
                    timeout = LEDK_DURATION_FLASH_ON;
                    fLedOn = ((ledkInstance_l.statusLedState & 0x01) != 0x00) ?
                        TRUE : FALSE;
                }
                break;

            case kLedModeTripleFlash:
                if (ledkInstance_l.statusLedState >= kLedModeDoubleFlash)
                {   // reset state
                    ledkInstance_l.statusLedState = kLedModeInit;
                    timeout = LEDK_DURATION_FLASH_OFF;
                    fLedOn = FALSE;
                }
                else
                {
                    timeout = LEDK_DURATION_FLASH_ON;
                    fLedOn = ((ledkInstance_l.statusLedState & 0x01) != 0x00) ?
                        TRUE : FALSE;
                }
                break;
        }

        ledkInstance_l.timeoutInMs = timeout;
        ledkInstance_l.startTimeInMs = tickCount;
        ret = target_setLed(kLedTypeStatus, fLedOn);
    }

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Set the LED mode

The function sets the LED mode.

\param[in]      ledType_p           The type of LED.
\param[in]      newMode_p           The new mode to set.

\return The function returns a tOplkError error code.

\ingroup module_ledk
*/
//------------------------------------------------------------------------------
tOplkError ledk_setLedMode(tLedType ledType_p, tLedMode newMode_p)
{
    tOplkError  ret = kErrorOk;
    tLedMode    oldMode;
    UINT32      timeout = 0;
    BOOL        fLedOn = FALSE;

    oldMode = ledkInstance_l.statusLedMode;
    if (oldMode != newMode_p)
    {   // state changed -> save new mode
        ledkInstance_l.statusLedMode = newMode_p;

         // select timeout corresponding to mode
        switch (newMode_p)
        {
            case kLedModeOn:
                timeout = 0;
                fLedOn = TRUE;
                break;

            case kLedModeOff:
                timeout = 0;
                fLedOn = FALSE;
                break;

            case kLedModeFlickering:
                timeout = LEDK_DURATION_FLICKERING;
                break;

            case kLedModeBlinking:
                timeout = LEDK_DURATION_BLINKING;
                break;

            case kLedModeSingleFlash:
            case kLedModeDoubleFlash:
            case kLedModeTripleFlash:
                if (fLedOn == FALSE)
                    timeout = LEDK_DURATION_FLASH_OFF;
                else
                    timeout = LEDK_DURATION_FLASH_ON;
                break;

            default:
                return ret;      // should not occur
        }

        switch (oldMode)
        {
            case kLedModeOff:
                // status LED was off -> switch LED on
                fLedOn = TRUE;
                ledkInstance_l.statusLedState = 0xFF;
                break;
            case kLedModeOn:
                // status LED was on -> switch LED off
                fLedOn = FALSE;
                ledkInstance_l.statusLedState = 0;
                break;
            default:
                break;
        }

        //Timeout value
        ledkInstance_l.timeoutInMs = timeout;
    }

    ret = target_setLed(ledType_p, fLedOn);

    return ret;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

/// \}

#endif
