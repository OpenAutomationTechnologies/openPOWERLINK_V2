/**
********************************************************************************
\file   xilinx-microblaze/target-microblaze.c

\brief  Target specific functions for Microblaze without OS

This target depending module provides several functions that are necessary for
systems without OS and not using shared buffer library.

\ingroup module_target
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2016, B&R Industrial Automation GmbH
Copyright (c) 2017, Kalycito Infotech Private Limited
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

#include "usleep.h"
#include "systemtimer.h"

#include <xparameters.h>
#include <xgpio_l.h>
#include <xintc.h>                  // interrupt controller

#include <common/target.h>
#if defined(__ZYNQ__)
#include <mb_uart.h>
#endif
//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#define TGTCONIO_MS_IN_US(x)    (x * 1000U)

#if !defined(CONFIG_PCP)
#error "CONFIG_PCP is needed for this implementation!"
#endif

#if (CONFIG_PCP == FALSE)
#define TGT_INTC_BASE           XPAR_HOST_INTC_BASEADDR
#elif (CONFIG_PCP != FALSE)
#define TGT_INTC_BASE           XPAR_PCP_INTC_BASEADDR
#else
#error  "Unable to determine the processor instance"
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
#define GPIO_STATUS_LED_BIT         1
#define GPIO_ERROR_LED_BIT          2

#ifdef XPAR_POWERLINK_LED_BASEADDR
#define TARGET_POWERLINK_LED_BASE   XPAR_POWERLINK_LED_BASEADDR
#endif

//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static UINT32   plkStatusErrorLeds_l;       ///< Local copy of the state of the POWERLINK status LEDs
static BOOL     fInterruptContextFlag_l;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static void enableInterruptMaster(void);
static void disableInterruptMaster(void);

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
    UINT32  ticks;

    ticks = timer_getMsCount();

    return ticks;
}

//------------------------------------------------------------------------------
/**
\brief  Get current timestamp

The function returns the current timestamp in nanoseconds.

\return The function returns the timestamp in nanoseconds
*/
//------------------------------------------------------------------------------
ULONGLONG target_getCurrentTimestamp(void)
{
    // Not implemented for this target
    return 0ULL;
}

//------------------------------------------------------------------------------
/**
\brief    enables global interrupt

This function enables/disables global interrupts.

\param[in]      fEnable_p           TRUE = enable interrupts
                                    FALSE = disable interrupts

\ingroup module_target
*/
//------------------------------------------------------------------------------
void target_enableGlobalInterrupt(BOOL fEnable_p)
{
    static INT  lockCount = 0;

    if (fEnable_p != FALSE) // restore interrupts
    {
        if (--lockCount == 0)
            enableInterruptMaster();
    }
    else
    {                       // disable interrupts
        if (lockCount == 0)
            disableInterruptMaster();

        lockCount++;
    }
}

//------------------------------------------------------------------------------
/**
\brief    Set interrupt context flag

This function enables/disables the interrupt context flag. The flag has to be
set when the CPU enters the interrupt context. The flag has to be cleared when
the interrupt context is left.

\param[in]      fEnable_p           TRUE = enable interrupt context flag
                                    FALSE = disable interrupt context flag

\ingroup module_target
*/
//------------------------------------------------------------------------------
void target_setInterruptContextFlag(BOOL fEnable_p)
{
    fInterruptContextFlag_l = fEnable_p;
}

//------------------------------------------------------------------------------
/**
\brief    Get interrupt context flag

This function returns the interrupt context flag.

\return The function returns the state of the interrupt context flag.

\ingroup module_target
*/
//------------------------------------------------------------------------------
BOOL target_getInterruptContextFlag(void)
{
    return fInterruptContextFlag_l;
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
    // initialize microblaze caches
#if defined(XPAR_MICROBLAZE_USE_ICACHE)
    microblaze_invalidate_icache();
    microblaze_enable_icache();
#endif

#if defined(XPAR_MICROBLAZE_USE_DCACHE)
    microblaze_invalidate_dcache();
    microblaze_enable_dcache();
#endif

    fInterruptContextFlag_l = FALSE;

    //enable microblaze interrupts
    microblaze_enable_interrupts();

    // initialize system timer
    timer_init();

#if defined(__ZYNQ__)
    uart_init();
#endif

    // enable the interrupt master
    enableInterruptMaster();

    plkStatusErrorLeds_l = 0;

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
    // disable microblaze caches
#if defined(XPAR_MICROBLAZE_USE_DCACHE)
    microblaze_invalidate_dcache();
    microblaze_disable_dcache();
#endif

#if defined(XPAR_MICROBLAZE_USE_ICACHE)
    microblaze_invalidate_icache();
    microblaze_disable_icache();
#endif

    //disable microblaze interrupts
    microblaze_disable_interrupts();

    // disable the interrupt master
    disableInterruptMaster();

    fInterruptContextFlag_l = FALSE;

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief Sleep for the specified number of milliseconds

The function makes the calling thread sleep until the number of specified
milliseconds has elapsed.

\param[in]      milliSeconds_p      Number of milliseconds to sleep

\ingroup module_target
*/
//------------------------------------------------------------------------------
void target_msleep(UINT32 milliSeconds_p)
{
    usleep(TGTCONIO_MS_IN_US(milliSeconds_p));
}

//------------------------------------------------------------------------------
/**
\brief  Set IP address of specified Ethernet interface

The function sets the IP address, subnetMask and MTU of an Ethernet
interface.

\param[in]      ifName_p            Name of Ethernet interface.
\param[in]      ipAddress_p         IP address to set for interface.
\param[in]      subnetMask_p        Subnet mask to set for interface.
\param[in]      mtu_p               MTU to set for interface.

\return The function returns a tOplkError error code.

\ingroup module_target
*/
//------------------------------------------------------------------------------
tOplkError target_setIpAdrs(const char* ifName_p,
                            UINT32 ipAddress_p,
                            UINT32 subnetMask_p,
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

\param[in]      defaultGateway_p    Default gateway to set.

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
\brief  Set POWERLINK status/error LED

The function sets the POWERLINK status/error LED.

\param[in]      ledType_p           Determines which LED shall be set/reset.
\param[in]      fLedOn_p            Set the addressed LED on (TRUE) or off (FALSE).

\return The function returns a tOplkError error code.

\ingroup module_target
*/
//------------------------------------------------------------------------------
tOplkError target_setLed(tLedType ledType_p, BOOL fLedOn_p)
{
    tOplkError  ret = kErrorOk;

    switch (ledType_p)
     {
         case kLedTypeStatus:
            setStatusLed(fLedOn_p);
            break;

         case kLedTypeError:
            setErrorLed(fLedOn_p);
            break;

         default:
            ret = kErrorIllegalInstance;
            break;
     }

    return ret;
}

#if (defined(CONFIG_INCLUDE_SOC_TIME_FORWARD) && defined(CONFIG_INCLUDE_NMT_MN))
//------------------------------------------------------------------------------
/**
\brief  Get system time

The function returns the current system timestamp.

\param[out]      pNetTime_p         Pointer to current system timestamp.
\param[out]      pValidSystemTime_p Pointer to flag which is set to indicate
                                    the system time is valid or not.

\return The function returns a tOplkError code.

\ingroup module_target
*/
//------------------------------------------------------------------------------
tOplkError target_getSystemTime(tNetTime* pNetTime_p, BOOL* pValidSystemTime_p)
{
    UNUSED_PARAMETER(pNetTime_p);
    UNUSED_PARAMETER(pValidSystemTime_p);

    //Note: Not implemented for this target

    return kErrorOk;
}
#endif

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief Enable the global interrupt master

\ingroup module_target
*/
//------------------------------------------------------------------------------
static void enableInterruptMaster(void)
{
    //enable global interrupt master
    XIntc_MasterEnable(TGT_INTC_BASE);
}

//------------------------------------------------------------------------------
/**
\brief Disable the global interrupt master

\ingroup module_target
*/
//------------------------------------------------------------------------------
static void disableInterruptMaster(void)
{
    //disable global interrupt master
    XIntc_MasterDisable(TGT_INTC_BASE);
}

//------------------------------------------------------------------------------
/**
\brief  Sets the status LED

The function sets the POWERLINK status LED.

\param[in]      fOn_p               Determines the LED state.

\ingroup module_app_common
*/
//------------------------------------------------------------------------------
static void setStatusLed(BOOL fOn_p)
{
    if (fOn_p)
        plkStatusErrorLeds_l |= (1 << GPIO_STATUS_LED_BIT);
    else
        plkStatusErrorLeds_l &= ~(1 << GPIO_STATUS_LED_BIT);

#if defined(TARGET_POWERLINK_LED_BASE)
    XGpio_WriteReg(TARGET_POWERLINK_LED_BASE, XGPIO_DATA_OFFSET, plkStatusErrorLeds_l);
#else
    UNUSED_PARAMETER(fOn_p);
#endif
}

//------------------------------------------------------------------------------
/**
\brief  Sets the error LED

The function sets the POWERLINK error LED.

\param[in]      fOn_p               Determines the LED state.

\ingroup module_app_common
*/
//------------------------------------------------------------------------------
static void setErrorLed(BOOL fOn_p)
{
    if (fOn_p)
        plkStatusErrorLeds_l |= (1 << GPIO_ERROR_LED_BIT);
    else
        plkStatusErrorLeds_l &= ~(1 << GPIO_ERROR_LED_BIT);

#if defined(TARGET_POWERLINK_LED_BASE)
    XGpio_WriteReg(TARGET_POWERLINK_LED_BASE, XGPIO_DATA_OFFSET, plkStatusErrorLeds_l);
#else
    UNUSED_PARAMETER(fOn_p);
#endif
}

/// \}
