/**
********************************************************************************
\file   altera-c5socarm/target-c5socarm.c

\brief  Target specific functions for ARM on Altera SoC without OS

This target depending module provides several functions that are necessary for
Altera SoC ARM without OS.

\ingroup module_target
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2017, Kalycito Infotech Private Limited
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
#include <sys/unistd.h>
#include <alt_timers.h>
#include <alt_globaltmr.h>
#include <alt_interrupt.h>
#include <alt_cache.h>
#include <alt_fpga_manager.h>
#include <alt_bridge_manager.h>
#include <alt_address_space.h>
#include <alt_mpu_registers.h>
#include <alt_clock_manager.h>

#include <system.h>

#include <common/target.h>
#include "sleep.h"

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
#define SECS_TO_MILLISECS   1000

//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// external vars
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static inline INT       enableCpuIrqInterface(void);
static inline INT       disableCpuIrqInterface(void);

static inline UINT64    getTimerMaxScaledCount(ALT_GPT_TIMER_t timerId_p,
                                               UINT32 scalingFactor_p);
static inline UINT64    getTimerCurrentScaledCount(ALT_GPT_TIMER_t timerId_p,
                                                   UINT32 scalingFactor_p);

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
    return (UINT32)getTimerCurrentScaledCount(ALT_GPT_CPU_GLOBAL_TMR, SECS_TO_MILLISECS);
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
\brief    Enables global interrupt

This function enables/disables global interrupts.

\param[in]      fEnable_p           TRUE = enable interrupts
                                    FALSE = disable interrupts

\ingroup module_target
*/
//------------------------------------------------------------------------------
void target_enableGlobalInterrupt(BOOL fEnable_p)
{
    static INT    lockCount = 0;

    if (fEnable_p != FALSE) // restore interrupts
    {
        if (--lockCount == 0)
            enableCpuIrqInterface();
    }
    else
    {                       // disable interrupts
        if (lockCount == 0)
            disableCpuIrqInterface();

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
    tOplkError      oplkRet = kErrorOk;
    ALT_STATUS_CODE halRet = ALT_E_SUCCESS;

#if defined(ALTARM_CACHE_ENABLE)
    // Enable Cache
    halRet = alt_cache_system_enable();
#else
    halRet = alt_cache_system_disable();
#endif

    if (halRet != ALT_E_SUCCESS)
    {
        oplkRet = kErrorGeneralError;
        goto Exit;
    }

    // Initialize the global interrupt controller
    halRet = alt_int_global_init();
    if (halRet != ALT_E_SUCCESS)
    {
        DEBUG_LVL_ERROR_TRACE("Global IRQ controller initialization failed!!\n");
        oplkRet = kErrorGeneralError;
        goto Exit;
    }

    // Initialize the CPU interrupt interface
    halRet = alt_int_cpu_init();
    if (halRet != ALT_E_SUCCESS)
    {
        DEBUG_LVL_ERROR_TRACE("CPU IRQ interface initialization failed!!\n");
        oplkRet = kErrorGeneralError;
        goto Exit;
    }

    // Enable global interrupt master
    halRet = alt_int_global_enable();
    if (halRet != ALT_E_SUCCESS)
    {
        DEBUG_LVL_ERROR_TRACE("enabling global interrupt receiver failed!!\n");
        oplkRet = kErrorGeneralError;
        goto Exit;
    }

Exit:
    return oplkRet;
}

//------------------------------------------------------------------------------
/**
\brief  Cleanup target specific stuff

The function cleans-up target specific stuff.

\return The function returns a tOplkError error code.

\ingroup module_target
*/
//------------------------------------------------------------------------------
tOplkError target_cleanup(void)
{
    ALT_STATUS_CODE halRet = ALT_E_SUCCESS;

    disableCpuIrqInterface();
    // Disable all interrupts from the distributor
    alt_int_global_disable();
    alt_int_cpu_uninit();
    alt_int_global_uninit();
    halRet = alt_cache_system_disable();

    return (halRet == ALT_E_SUCCESS) ? kErrorOk : kErrorGeneralError;
}

//------------------------------------------------------------------------------
/**
\brief Sleep for the specified number of milliseconds

The function makes the calling thread sleep until the number of specified
milliseconds have elapsed.

\param[in]      milliSeconds_p      Number of milliseconds to sleep

\ingroup module_target
*/
//------------------------------------------------------------------------------
void target_msleep(UINT32 milliSeconds_p)
{
    msleep(milliSeconds_p);
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
    UNUSED_PARAMETER(fEnable_p);
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
    return FALSE;
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
    UNUSED_PARAMETER(ledType_p);
    UNUSED_PARAMETER(fLedOn_p);

    //Note: This function is not yet implemented since there is no design with
    //      the C5 SoC ARM executing the kernel layer.

    return kErrorOk;
}

#if (defined(CONFIG_INCLUDE_SOC_TIME_FORWARD) && defined(CONFIG_INCLUDE_NMT_MN))
//------------------------------------------------------------------------------
/**
\brief  Get system time

The function returns the current system timestamp.

\param[out]      pNetTime_p         Pointer to current system timestamp.
\param[out]      pValidSystemTime_p Pointer to flag which is set to indicate
                                    the system time is valid.

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
\brief Enable the CPU interrupt interface

The function enables interrupt reception in the target processor
interrupt interface.

\return The function returns an integer
\retval 0                           Success
\retval -1                          Failure
*/
//------------------------------------------------------------------------------
static inline INT enableCpuIrqInterface(void)
{
    ALT_STATUS_CODE retStatus = ALT_E_SUCCESS;
    INT             ret = 0;

    // CPU interface global enable
    retStatus = alt_int_cpu_enable();
    if (retStatus != ALT_E_SUCCESS)
    {
        DEBUG_LVL_ERROR_TRACE("Enabling CPU interrupt receiver failed!!\n");
        ret = -1;
        goto Exit;
    }

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief Disable the CPU interrupt interface

The function disables interrupt reception in the target processor
interrupt interface.

\return The function returns an integer
\retval 0                           Success
\retval -1                          Failure
*/
//------------------------------------------------------------------------------
static inline INT disableCpuIrqInterface(void)
{
    ALT_STATUS_CODE retStatus = ALT_E_SUCCESS;
    INT             ret = 0;

    // Reset the CPU interface
    retStatus = alt_int_cpu_disable();
    if (retStatus != ALT_E_SUCCESS)
    {
        ret = -1;
        goto Exit;
    }

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief Convert timer ticks into time units

The function returns the timestamp of the provided timer in standard time units.

\param[in]      timerId_p           The ALT_GPT_TIMER_t enum Id of the timer used
\param[in]      scalingFactor_p     Ratio of provided time duration scale to seconds

\return The function returns a unsigned 64 bit value.
\retval Timestamp from the given timer in standard time unit provided.
*/
//------------------------------------------------------------------------------
static inline UINT64 getTimerCurrentScaledCount(ALT_GPT_TIMER_t timerId_p,
                                                UINT32 scalingFactor_p)
{
    UINT64      timeStamp_l = 0;
    UINT64      timeStamp_h = 0;
    UINT64      timeStamp = 0;
    UINT64      scaledTime = 0;
    ALT_CLK_t   clkSrc = ALT_CLK_UNKNOWN;
    UINT32      preScaler = 0;
    UINT32      freq = 1;

    preScaler = alt_gpt_prescaler_get(timerId_p);
    if (preScaler <= UINT8_MAX)
    {
        if (timerId_p == ALT_GPT_CPU_GLOBAL_TMR)    // Global Timer
        {
            alt_globaltmr_get((uint32_t*)&timeStamp_h, (uint32_t*)&timeStamp_l);
            clkSrc = ALT_CLK_MPU_PERIPH;
        }
        else
        {
            scaledTime = 0;
            goto Exit;
        }

        if (alt_clk_freq_get(clkSrc, (uint32_t*)&freq) == ALT_E_SUCCESS)
        {
            timeStamp_l *= (preScaler + 1);
            timeStamp_h *= (preScaler + 1);
            timeStamp_l *= scalingFactor_p;
            timeStamp_h *= scalingFactor_p;
            timeStamp = (UINT64)((((timeStamp_h << 32) & ~UINT32_MAX) | timeStamp_l) / freq);
            scaledTime = (timeStamp > UINT64_MAX) ? UINT64_MAX : (UINT64)timeStamp;
        }
    }

Exit:
    return scaledTime;
}

//------------------------------------------------------------------------------
/**
\brief Get maximum timestamp of the timer

The function returns the maximum timestamp of the provided timer
in standard time units.

\param[in]      timerId_p           The ALT_GPT_TIMER_t enum Id of the timer used
\param[in]      scalingFactor_p     Ratio of provided time duration scale to seconds

\return The function returns a unsigned 64 bit value.
\retval Maximum timestamp from the given timer in standard time unit provided.
*/
//------------------------------------------------------------------------------
static inline UINT64 getTimerMaxScaledCount(ALT_GPT_TIMER_t timerId_p,
                                            UINT32 scalingFactor_p)
{
    UINT64      maxScaledTime = 0;
    UINT32      freq = 1;
    UINT64      maxTimeStampLow = 0;
    UINT64      maxTimeStampHigh = 0;
    UINT64      maxTimeStamp = 0;
    UINT32      preScaler = 0;
    ALT_CLK_t   clkSrc;

    preScaler = alt_gpt_prescaler_get(timerId_p);

    if (timerId_p == ALT_GPT_CPU_GLOBAL_TMR)
    {
        clkSrc = ALT_CLK_MPU_PERIPH;
        maxTimeStampLow = (UINT64)UINT32_MAX;
        maxTimeStampHigh = (UINT64)UINT32_MAX;
    }
    else
        goto Exit;

    if (alt_clk_freq_get(clkSrc, (uint32_t*)&freq) == ALT_E_SUCCESS)
    {
        maxTimeStampLow *= (preScaler + 1);
        maxTimeStampHigh *= (preScaler + 1);
        maxTimeStampLow *= scalingFactor_p;
        maxTimeStampHigh *= scalingFactor_p;
        maxTimeStamp = (UINT64)((((maxTimeStampHigh << 32) & ~UINT32_MAX) | maxTimeStampLow) / freq);
        maxScaledTime = (maxTimeStamp > UINT64_MAX) ? UINT64_MAX : (UINT64) maxTimeStamp;
    }

Exit:
    return maxScaledTime;
}

/// \}
