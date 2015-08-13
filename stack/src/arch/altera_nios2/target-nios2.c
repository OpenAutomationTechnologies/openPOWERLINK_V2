/**
********************************************************************************
\file   altera_nios2/target-nios2.c

\brief  target specific functions for Nios II without OS

This target depending module provides several functions that are necessary for
systems without shared buffer and any OS.

\ingroup module_target
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2014, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
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
#include <unistd.h>
#include <sys/alt_irq.h>
#include <sys/alt_alarm.h>
#include <common/oplkinc.h>
#include <common/target.h>
#include <system.h>
#include <altera_avalon_pio_regs.h>
#include <oplk/led.h>

//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#define TGTCONIO_MS_IN_US(x)    (x * 1000U)

#if defined(CONFIG_INCLUDE_LEDK)
#define GPIO_STATUS_LED_BIT     1
#define GPIO_ERROR_LED_BIT      2

#ifdef STATUS_LED_PIO_BASE
#define TARGET_STATUS_LED_IO_BASE STATUS_LED_PIO_BASE
#else
#define TARGET_STATUS_LED_IO_BASE 0
#endif
#endif

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
#if defined(CONFIG_INCLUDE_LEDK)
/**
 * \brief   Kernel LED module instance
 *
 * The structure defines the instance data of the kernel LED module.
 */
typedef struct
{
    tLedMode                    statusLedMode;          ///< Mode of the status LED
    UINT                        statusLedState;         ///< State of the status LED
    UINT32                      startTimeInMs;          ///< Holds the start time value of the current led state
    UINT32                      timeoutInMs;            ///< Holds the time out value of the current led state
    UINT32                      plkStatusErrorLeds;     ///< Local copy of the state of the POWERLINK status LEDs
} tTargetLedInstance;
#endif

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
#if defined(CONFIG_INCLUDE_LEDK)
static tTargetLedInstance   targetledInstance_l;
#endif

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
#if defined(CONFIG_INCLUDE_LEDK)
static void setStatusLed(BOOL fOn_p);
static void setErrorLed(BOOL fOn_p);
#endif

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief    Get current system tick

This function returns the current system tick determined by the system timer.

\return Returns the system tick in milliseconds

\ingroup module_target
*/
//------------------------------------------------------------------------------
UINT32 target_getTickCount(void)
{
    UINT32 ticks;

    ticks = alt_nticks();

    return ticks;
}

//------------------------------------------------------------------------------
/**
\brief    Enable global interrupt

This function enabels/disables global interrupts.

\param  fEnable_p               TRUE = enable interrupts
                                FALSE = disable interrupts

\ingroup module_target
*/
//------------------------------------------------------------------------------
void target_enableGlobalInterrupt(BYTE fEnable_p)
{
static alt_irq_context  irq_context = 0;
static int              iLockCount = 0;

    if (fEnable_p != FALSE)
    {   // restore interrupts
        if (--iLockCount == 0)
        {
            alt_irq_enable_all(irq_context);
        }
    }
    else
    {   // disable interrupts
        if (iLockCount == 0)
        {
            irq_context = alt_irq_disable_all();
        }
        iLockCount++;
    }
}

//------------------------------------------------------------------------------
/**
\brief  Initialize target specific stuff

The function initialize target specific stuff which is needed to run the
openPOWERLINK stack.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
tOplkError target_init(void)
{
#if defined(CONFIG_INCLUDE_LEDK)
    targetledInstance_l.startTimeInMs = 0;
    targetledInstance_l.timeoutInMs = 0;
#endif
    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Clean up target specific stuff

The function cleans up target specific stuff.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
tOplkError target_cleanup(void)
{
    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief Sleep for the specified number of milliseconds

The function makes the calling thread sleep, until the number of specified
milliseconds has elapsed.

\param  milliSecond_p       Number of milliseconds to sleep

\ingroup module_target
*/
//------------------------------------------------------------------------------
void target_msleep(unsigned int milliSecond_p)
{
    usleep(TGTCONIO_MS_IN_US(milliSecond_p));
}

//------------------------------------------------------------------------------
/**
\brief  Set IP address of specified Ethernet interface

The function sets the IP address, subnetMask and MTU of an Ethernet
interface.

\param  ifName_p                Name of Ethernet interface.
\param  ipAddress_p             IP address to set for interface.
\param  subnetMask_p            Subnet mask to set for interface.
\param  mtu_p                   MTU to set for interface.

\return The function returns a tOplkError error code.

\ingroup module_target
*/
//------------------------------------------------------------------------------
tOplkError target_setIpAdrs(char* ifName_p, UINT32 ipAddress_p, UINT32 subnetMask_p,
                            UINT16 mtu_p)
{
    UNUSED_PARAMETER(ifName_p);
    UNUSED_PARAMETER(ipAddress_p);
    UNUSED_PARAMETER(subnetMask_p);
    UNUSED_PARAMETER(mtu_p);

    //Note: The given parameters are ignored because the application must set
    //      these settings to the used IP stack by itself!

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Set default gateway for Ethernet interface

The function sets the default gateway of an Ethernet interface.

\param  defaultGateway_p            Default gateway to set.

\return The function returns a tOplkError error code.

\ingroup module_target
*/
//------------------------------------------------------------------------------
tOplkError target_setDefaultGateway(UINT32 defaultGateway_p)
{
    UNUSED_PARAMETER(defaultGateway_p);

    //Note: The given parameters are ignored because the application must set
    //      these settings to the used IP stack by itself!

    return kErrorOk;
}

#if defined(CONFIG_INCLUDE_LEDK)
//------------------------------------------------------------------------------
/**
\brief  update Led State

The function handles the led status mode.

\return The function returns a tOplkError error code.

\ingroup module_target
*/
//------------------------------------------------------------------------------
tOplkError target_updateLedState(void)
{
    INT                 timeout = 0;
    BOOL                fLedOn = FALSE;
    tOplkError          ret = kErrorOk;

    // Setting the new timeout value
    timeout = alt_nticks() - targetledInstance_l.startTimeInMs;

    if (timeout >= targetledInstance_l.timeoutInMs && timeout > 0)
    {
        targetledInstance_l.statusLedState++;

        // select timeout and new LED state corresponding to mode
        switch (targetledInstance_l.statusLedMode)
        {
            case kLedModeInit:
            case kLedModeOn:
            case kLedModeOff:
                goto Exit;      // should not occur
                break;

            case kLedModeFlickering:
                if (targetledInstance_l.statusLedState >= kLedModeOn)
                {   // reset state
                    targetledInstance_l.statusLedState = kLedModeInit;
                    fLedOn = FALSE;
                }
                else
                {
                    fLedOn = TRUE;
                }

                timeout = LED_DURATION_FLICKERING;
                break;

            case kLedModeBlinking:
                if (targetledInstance_l.statusLedState >= kLedModeOn)
                {   // reset state
                    targetledInstance_l.statusLedState = kLedModeInit;
                    fLedOn = FALSE;
                }
                else
                {
                    fLedOn = TRUE;
                }

                timeout = LED_DURATION_BLINKING;
                break;

            case kLedModeSingleFlash:
                if (targetledInstance_l.statusLedState >= kLedModeOn)
                {   // reset state
                    targetledInstance_l.statusLedState = kLedModeInit;
                    timeout = LED_DURATION_FLASH_OFF;
                    fLedOn = FALSE;
                }
                else
                {
                    timeout = LED_DURATION_FLASH_ON;
                    fLedOn = ((targetledInstance_l.statusLedState & 0x01) != 0x00) ?
                        TRUE : FALSE;
                }

                break;

            case kLedModeDoubleFlash:
                if (targetledInstance_l.statusLedState >= kLedModeBlinking)
                {   // reset state
                    targetledInstance_l.statusLedState = kLedModeInit;
                    timeout = LED_DURATION_FLASH_OFF;
                    fLedOn = FALSE;
                }
                else
                {
                    timeout = LED_DURATION_FLASH_ON;
                    fLedOn = ((targetledInstance_l.statusLedState & 0x01) != 0x00) ?
                        TRUE : FALSE;
                }

                break;

            case kLedModeTripleFlash:
                if (targetledInstance_l.statusLedState >= kLedModeDoubleFlash)
                {   // reset state
                    targetledInstance_l.statusLedState = kLedModeInit;
                    timeout = LED_DURATION_FLASH_OFF;
                    fLedOn = FALSE;
                }
                else
                {
                    timeout = LED_DURATION_FLASH_ON;
                    fLedOn = ((targetledInstance_l.statusLedState & 0x01) != 0x00) ?
                        TRUE : FALSE;
                }

                break;
        }

        targetledInstance_l.timeoutInMs = timeout;
        targetledInstance_l.startTimeInMs += targetledInstance_l.timeoutInMs;
        ret = target_setLed(kLedTypeStatus, fLedOn);
    }

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Change the LED mode

The function changes the LED mode.

\param  ledType_p           The type of LED.
\param  newMode_p           The new mode to set.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
tOplkError target_setLedMode(tLedType ledType_p, tLedMode newMode_p)
{
    tOplkError      ret = kErrorOk;
    tLedMode        oldMode;
    UINT32          timeout;
    BOOL            fLedOn = ((targetledInstance_l.plkStatusErrorLeds &
                              (1 << GPIO_STATUS_LED_BIT)) == 0) ? FALSE : TRUE;

    oldMode = targetledInstance_l.statusLedMode;
    if (oldMode != newMode_p)
    {   // state changed -> save new mode
        targetledInstance_l.statusLedMode = newMode_p;

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
                timeout = LED_DURATION_FLICKERING;
                break;

            case kLedModeBlinking:
                timeout = LED_DURATION_BLINKING;
                break;

            case kLedModeSingleFlash:
            case kLedModeDoubleFlash:
            case kLedModeTripleFlash:
                if (fLedOn == FALSE)
                    timeout = LED_DURATION_FLASH_OFF;
                else
                    timeout = LED_DURATION_FLASH_ON;
                break;

            default:
                return ret;      // should not occur
                break;
        }

        // Where are we coming from?
        if (oldMode == kLedModeOff)
        {   // status LED was off -> switch LED on
            fLedOn = TRUE;
            targetledInstance_l.statusLedState = 0xFF;
        }
        else if (oldMode == kLedModeOn)
        {   // status LED was on -> switch LED off
            fLedOn = FALSE;
            targetledInstance_l.statusLedState = 0;
        }
        else
        {   // timer is handled in target_updateLedState
            goto Exit;
        }

    //Timeout value
    targetledInstance_l.timeoutInMs = timeout;
    }

Exit:
    ret = target_setLed(ledType_p, fLedOn);
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Call state change function

The function calls the type and state of LED.

\param  ledType_p           The type of LED.
\param  fLedOn_p            The state of the LED.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
tOplkError target_setLed(tLedType ledType_p, BOOL fLedOn_p)
{
    tOplkError ret = kErrorOk;

    switch (ledType_p)
     {
         case kLedTypeStatus:
            return setStatusLed(fLedOn_p);
            break;

         case kLedTypeError:
            return setErrorLed(fLedOn_p);
            break;

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
/*
\brief  Sets the status LED

The function sets the POWERLINK status LED.

\param  fOn_p               Determines the LED state.

\ingroup module_drv_common
*/
//------------------------------------------------------------------------------
static void setStatusLed(BOOL fOn_p)
{
#ifdef TARGET_STATUS_LED_IO_BASE
    if (fOn_p != FALSE)
        IOWR_ALTERA_AVALON_PIO_SET_BITS(TARGET_STATUS_LED_IO_BASE, GPIO_STATUS_LED_BIT);
    else
        IOWR_ALTERA_AVALON_PIO_CLEAR_BITS(TARGET_STATUS_LED_IO_BASE, GPIO_STATUS_LED_BIT);
#endif
}

//------------------------------------------------------------------------------
/*
\brief  Sets the error LED

The function sets the POWERLINK error LED.

\param  fOn_p               Determines the LED state.

\ingroup module_drv_common
*/
//------------------------------------------------------------------------------
static void setErrorLed(BOOL fOn_p)
{
#ifdef TARGET_STATUS_LED_IO_BASE
    if (fOn_p != FALSE)
        IOWR_ALTERA_AVALON_PIO_SET_BITS(TARGET_STATUS_LED_IO_BASE, GPIO_ERROR_LED_BIT);
    else
        IOWR_ALTERA_AVALON_PIO_CLEAR_BITS(TARGET_STATUS_LED_IO_BASE, GPIO_ERROR_LED_BIT);
#endif
}
#endif

/// \}