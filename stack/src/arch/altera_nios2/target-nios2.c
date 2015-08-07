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

//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#define TGTCONIO_MS_IN_US(x)    (x * 1000U)

#define GPIO_STATUS_LED_BIT     1
#define GPIO_ERROR_LED_BIT      2

#ifdef PCP_0_POWERLINK_LED_BASE
#define TARGET_POWERLINK_LED_BASE PCP_0_POWERLINK_LED_BASE
#else
#define TARGET_POWERLINK_LED_BASE 0
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

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static void setStatusLed(BOOL fOn_p);
static void setErrorLed(BOOL fOn_p);

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

//------------------------------------------------------------------------------
/**
\brief  Call state change function

The function calls the type and state of LED.

\param  ledType_p           The type of LED.
\param  fLedOn_p            The state of the LED.
\param  modetype            The type of LED mode.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
tOplkError target_setLed(tLedType ledType_p, BOOL fLedOn_p, tLedMode modetype)
{
    tOplkError ret = kErrorOk;

    switch (ledType_p)
     {
         case kLedTypeStatus:
            setStatusLed(fLedOn_p);
            break;

         case kLedTypeError:
            setErrorLed(fLedOn_p);
            break;

         default:
            return kErrorIllegalInstance;
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
#ifdef TARGET_POWERLINK_LED_BASE
    if (fOn_p != FALSE)
        IOWR_ALTERA_AVALON_PIO_SET_BITS(TARGET_POWERLINK_LED_BASE, GPIO_STATUS_LED_BIT);
    else
        IOWR_ALTERA_AVALON_PIO_CLEAR_BITS(TARGET_POWERLINK_LED_BASE, GPIO_STATUS_LED_BIT);
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
#ifdef TARGET_POWERLINK_LED_BASE
    if (fOn_p != FALSE)
        IOWR_ALTERA_AVALON_PIO_SET_BITS(TARGET_POWERLINK_LED_BASE, GPIO_ERROR_LED_BIT);
    else
        IOWR_ALTERA_AVALON_PIO_CLEAR_BITS(TARGET_POWERLINK_LED_BASE, GPIO_ERROR_LED_BIT);
#endif
}

/// \}