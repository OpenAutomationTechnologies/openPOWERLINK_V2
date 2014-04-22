/**
********************************************************************************
\file   xilinx_microblaze/target-microblaze.c

\brief  Target specific functions for Microblaze without OS

This target depending module provides several functions that are necessary for
systems without OS and not using shared buffer library.

\ingroup module_target
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2014, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
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
#include <oplk/oplk.h>

#include "usleep.h"
#include "systemtimer.h"

#include <xparameters.h>
#include <xintc.h>         // interrupt controller

#ifdef __ZYNQ__
    #include "xil_io.h"
#endif
//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#define TGTCONIO_MS_IN_US(x)    (x * 1000U)

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
#ifdef __ZYNQ__
    // base address of PS Uart1
    #define UART_BASE 0xE0001000
    /* Write to memory location or register */
    #define X_mWriteReg(BASE_ADDRESS, RegOffset, data) \
                *(unsigned int *)(BASE_ADDRESS + RegOffset) = ((unsigned int) data);
    /* Read from memory location or register */
    #define X_mReadReg(BASE_ADDRESS, RegOffset) \
                *(unsigned int *)(BASE_ADDRESS + RegOffset);

    #define XUartChanged_IsTransmitFull(BaseAddress)  \
                ((Xil_In32((BaseAddress) + 0x2C) & \
                            0x10) == 0x10)

    #define XUartChanged_SendByte(BAddr,Data) \
        u32 u32BaseAddress = BAddr; \
        u8 u8Data = Data; \
        while (XUartChanged_IsTransmitFull(u32BaseAddress));\
        X_mWriteReg(u32BaseAddress, 0x30, u8Data);
#endif
//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------

static void enableInterruptMaster(void);
static void disableInterruptMaster(void);
#ifdef __ZYNQ__
void print(char *str);
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

    ticks = timer_getMSCount();

    return ticks;
}

//------------------------------------------------------------------------------
/**
\brief    enables global interrupt

This function enables/disables global interrupts.

\param  fEnable_p               TRUE = enable interrupts
                                FALSE = disable interrupts

\ingroup module_target
*/
//------------------------------------------------------------------------------
void target_enableGlobalInterrupt(UINT8 fEnable_p)
{
    static INT lockCount = 0;

    if (fEnable_p != FALSE)
    {   // restore interrupts
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
*/
//------------------------------------------------------------------------------
tOplkError target_init(void)
{
    // initialize microblaze caches
#if XPAR_MICROBLAZE_USE_ICACHE
    microblaze_invalidate_icache();
    microblaze_enable_icache();
#endif

#if XPAR_MICROBLAZE_USE_DCACHE
    microblaze_invalidate_dcache();
    microblaze_enable_dcache();
#endif

    //enable microblaze interrupts
    microblaze_enable_interrupts();

    // initialize system timer
    timer_init();

    // enable the interrupt master
    enableInterruptMaster();

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
#if XPAR_MICROBLAZE_USE_DCACHE
    microblaze_invalidate_dcache();
    microblaze_disable_dcache();
#endif

#if XPAR_MICROBLAZE_USE_ICACHE
    microblaze_invalidate_icache();
    microblaze_disable_icache();
#endif

    //disable microblaze interrupts
    microblaze_disable_interrupts();

    // disable the interrupt master
    disableInterruptMaster();

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief Sleep for the specified number of milliseconds

The function makes the calling thread sleep until the number of specified
milliseconds has elapsed.

\param  milliSeconds_p      Number of milliseconds to sleep

\ingroup module_target
*/
//------------------------------------------------------------------------------
void target_msleep(UINT32 milliSeconds_p)
{
    usleep(TGTCONIO_MS_IN_US(milliSeconds_p));
}

//------------------------------------------------------------------------------
/**
\brief Register synchronization interrupt handler

The function registers the ISR for target specific synchronization interrupt
used by the application for PDO and event synchronization.

\param  callback_p              Interrupt handler
\param  pArg_p                  Argument to be passed while calling the handler

\return The function returns the error code as a integer value
\retval 0 if able to register
\retval other if not

\ingroup module_target
*/
//------------------------------------------------------------------------------
void target_regSyncIrqHdl( void* callback_p,void* pArg_p)
{
    UNUSED_PARAMETER(callback_p);
    UNUSED_PARAMETER(pArg_p);
    // todo gks: Add Target interrupt registration for sync here
}

//------------------------------------------------------------------------------
/**
\brief Sync interrupt control routine

The function is used to enable or disable the sync interrupt

\param  fEnable_p              enable if TRUE, disable if FALSE

\ingroup module_target
*/
//------------------------------------------------------------------------------
void target_enableSyncIrq(BOOL fEnable_p)
{
    UNUSED_PARAMETER(fEnable_p);
    // todo gks Add interrupt handling
}

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
    XIntc_MasterEnable(XPAR_PCP_INTC_BASEADDR);
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
    XIntc_MasterDisable(XPAR_PCP_INTC_BASEADDR);
}

#ifdef __ZYNQ__
void outbyte(char c)
{
    XUartChanged_SendByte(UART_BASE, c);
}
#endif

///\}

