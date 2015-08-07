/**
********************************************************************************
\file   xilinx-zynqarm/target-zynqarm.c

\brief  Target specific functions for ARM on Zynq without OS

This target depending module provides several functions that are necessary for
systems without OS.

\ingroup module_target
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2014, Kalycito Infotech Private Limited
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

#include <xscugic.h>
#include <xtime_l.h>
#include <xil_cache.h>
#include <xil_types.h>
#include <xil_io.h>
#include <xil_exception.h>
#include <unistd.h>

#include <xparameters.h>

//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#define TGTCONIO_MS_IN_US(x)      (x * 1000U)

#if (XPAR_CPU_ID == 0)
#define TARGET_CPU_VALUE           0x01
#else
#define TARGET_CPU_VALUE           0x02
#endif

#define SLCR_LOCK                  0xF8000004       ///< SLCR Write Protection Lock register
#define SLCR_UNLOCK                0xF8000008       ///< SLCR Write Protection Unlock register
#define FPGA_RST_CNTRL             0xF8000240       ///< Zynq PL reset control register
#define SLCR_LOCK_VAL              0x767B           ///< SLCR Lock value
#define SLCR_UNLOCK_VAL            0xDF0D           ///< SLCR unlock value
#define DEFAULT_PRIORITY           0xa0a0a0a0UL     ///< Default priority for GIC

//------------------------------------------------------------------------------
// module global vars
//------------------------------------------------------------------------------
// Get generic interrupt controller config table from xscugic.h
extern XScuGic_Config XScuGic_ConfigTable[];

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
static XScuGic gicInstance_l;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static void enableInterruptMaster(void);
static void disableInterruptMaster(void);
static void initInterrupts(void);

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief    Get current system tick

This function returns the current system tick determined by the system timer.

\return The function returns the system tick in milliseconds

\ingroup module_target
*/
//------------------------------------------------------------------------------
UINT32 target_getTickCount(void)
{
    UINT32    ticks;
    XTime     localticks;
    /* Uses global timer functions */

    XTime_GetTime(&localticks);
    /* Select the lower 32 bit of the timer value */
    ticks = (UINT32)((2000 * localticks) / XPAR_CPU_CORTEXA9_CORE_CLOCK_FREQ_HZ);

    return ticks;
}

//------------------------------------------------------------------------------
/**
\brief    Enables global interrupt

This function enables/disables global interrupts.

\param  fEnable_p               TRUE = enable interrupts
                                FALSE = disable interrupts

\ingroup module_target
*/
//------------------------------------------------------------------------------
void target_enableGlobalInterrupt(UINT8 fEnable_p)
{
    static INT    lockCount = 0;

    if (fEnable_p != FALSE) // restore interrupts
    {
        if (--lockCount == 0)
        {
            enableInterruptMaster();
        }
    }
    else
    {   // disable interrupts
        if (lockCount == 0)
        {
            disableInterruptMaster();
        }
        lockCount++;
    }
}

//------------------------------------------------------------------------------
/**
\brief  Initialize target specific stuff

The function initialize target specific stuff which is needed to run the
openPOWERLINK stack.

\return The function returns a tOplkError error code.

\ingroup module_target
*/
//------------------------------------------------------------------------------
tOplkError target_init(void)
{
    Xil_DCacheFlush();

    // Enable Cache
    Xil_ICacheEnable();
    Xil_DCacheEnable();

    // Initialize Interrupts
    initInterrupts();

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Cleanup target specific stuff

The function cleans up target specific stuff.

\return The function returns a tOplkError error code.

\ingroup module_target
*/
//------------------------------------------------------------------------------
tOplkError target_cleanup(void)
{
    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief Sleep for the specified number of milliseconds

The function makes the calling thread sleep until the number of specified
milliseconds have elapsed.

\param  milliSeconds_p            Number of milliseconds to sleep

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
\brief  Set POWERLINK status/error LED

The function sets the POWERLINK status/error LED.

\param  ledType_p       Determines which LED shall be set/reset.
\param  fLedOn_p        Set the addressed LED on (TRUE) or off (FALSE).

\return The function returns a tOplkError error code.

\ingroup module_target
*/
//------------------------------------------------------------------------------
tOplkError target_setLed(tLedType ledType_p, BOOL fLedOn_p)
{
    UNUSED_PARAMETER(ledType_p);
    UNUSED_PARAMETER(fLedOn_p);

    //Note: This function is not yet implemented since there is no design with
    //      the C5 SoC ARM executing the kernel layer.

    return kErrorOk;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief Enable the global interrupt master

The function enables global interrupts.

*/
//------------------------------------------------------------------------------
static void enableInterruptMaster(void)
{
    // enable global interrupt master
    // Global interrupt enable
    // Distributor global enable
    Xil_Out32((XPAR_PS7_SCUGIC_0_DIST_BASEADDR + XSCUGIC_DIST_EN_OFFSET), XSCUGIC_EN_INT_MASK);
    // CPU interface global enable
    Xil_Out32((XPAR_SCUGIC_0_CPU_BASEADDR + XSCUGIC_CONTROL_OFFSET), XSCUGIC_CNTR_EN_S_MASK);
}

//------------------------------------------------------------------------------
/**
\brief Disable the global interrupt master

The function disables global interrupts.
*/
//------------------------------------------------------------------------------
static void disableInterruptMaster(void)
{
    // Disable all interrupts from the distributor
    Xil_Out32((XPAR_PS7_SCUGIC_0_DIST_BASEADDR + XSCUGIC_DIST_EN_OFFSET), 0UL);
    // Reset the DP (Distributor) and CP (CPU interface)
    Xil_Out32((XPAR_SCUGIC_0_CPU_BASEADDR + XSCUGIC_CONTROL_OFFSET), 0UL);
}

//------------------------------------------------------------------------------
/**
\brief  Set up interrupt controller

This function sets up the interrupt and exception handling for interrupt
controller on ARM.

*/
//------------------------------------------------------------------------------
static void initInterrupts(void)
{
    INT    status;
    static XScuGic_Config*   pConfig = &XScuGic_ConfigTable[XPAR_PS7_SCUGIC_0_DEVICE_ID];

    // TODO@gks: This will only initialize interrupt configuration for the ARM core 0
    //           In order to configure interrupts for ARM core 1 we will need to re-define the
    //           configuration routine separately.
    status = XScuGic_CfgInitialize(&gicInstance_l, pConfig, XPAR_PS7_SCUGIC_0_BASEADDR);

    if (status != XST_SUCCESS)
        return;

    // CPU interrupt interface & distributor has been enabled before this point

    Xil_ExceptionInit();

    // Register the master interrupt handler for Interrupt and data exception
    // This allow to arbitrate different interrupts through common interrupt exception
    // line

    Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_DATA_ABORT_INT,
                                 (Xil_ExceptionHandler)XScuGic_InterruptHandler,
                                 &gicInstance_l);
    Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_IRQ_INT,
                                 (Xil_ExceptionHandler)XScuGic_InterruptHandler,
                                 &gicInstance_l);

    Xil_ExceptionEnable();
}

///\}
