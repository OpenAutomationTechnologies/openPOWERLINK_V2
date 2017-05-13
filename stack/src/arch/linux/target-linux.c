/**
********************************************************************************
\file   linux/target-linux.c

\brief  Target specific functions for Linux

The file implements target specific functions used in the openPOWERLINK stack.

\ingroup module_target
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2016, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
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
#include <common/ftracedebug.h>

#include <unistd.h>
#include <fcntl.h>
#include <signal.h>
#include <time.h>
#include <sys/time.h>

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
\brief  Initialize target specific stuff

The function initialize target specific stuff which is needed to run the
openPOWERLINK stack.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
tOplkError target_init(void)
{
    tOplkError  ret = kErrorOk;
    sigset_t    mask;

    /*
     * We have to block the real time signals used by the timer modules so
     * that they are able to wait on them using sigwaitinfo!
     */
    sigemptyset(&mask);
    sigaddset(&mask, SIGRTMIN);
    sigaddset(&mask, SIGRTMIN + 1);
    pthread_sigmask(SIG_BLOCK, &mask, NULL);

    /* Enabling ftrace for debugging */
    FTRACE_OPEN();
    FTRACE_ENABLE(TRUE);

    return ret;
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
    tOplkError  ret = kErrorOk;

    /* Disable ftrace debugging */
    FTRACE_ENABLE(FALSE);

    return ret;
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
    struct  timeval timeout;
    fd_set          readFds;
    int             maxFd;
    int             selectRetVal;
    unsigned int    seconds;
    unsigned int    microSeconds;

    // initialize file descriptor set
    maxFd = 0 + 1;
    FD_ZERO(&readFds);

    // Calculate timeout values
    seconds = milliSeconds_p / 1000;
    microSeconds = (milliSeconds_p - (seconds * 1000)) * 1000;

    // initialize timeout value
    timeout.tv_sec = seconds;
    timeout.tv_usec = microSeconds;

    selectRetVal = select(maxFd, &readFds, NULL, NULL, &timeout);
    switch (selectRetVal)
    {
        case 0:     // select timeout occurred, no packet received
            break;

        case -1:    // select error occurred
            break;

        default:    // packet available for receive
            break;
    }
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
    tOplkError  ret = kErrorOk;
    INT         iRet;
    char        sBufferIp[16];
    char        sBufferMask[16];
    char        sBufferMtu[6];
    char        command[256];

    // configure IP address of virtual network interface
    // for TCP/IP communication over the POWERLINK network
    snprintf(sBufferIp,
             sizeof(sBufferIp),
             "%u.%u.%u.%u",
             (UINT)(ipAddress_p >> 24),
             (UINT)((ipAddress_p >> 16) & 0xFF),
             (UINT)((ipAddress_p >> 8) & 0xFF),
             (UINT)(ipAddress_p & 0xFF));

    snprintf(sBufferMask,
             sizeof(sBufferMask),
             "%u.%u.%u.%u",
             (UINT)(subnetMask_p >> 24),
             (UINT)((subnetMask_p >> 16) & 0xFF),
             (UINT)((subnetMask_p >> 8) & 0xFF),
             (UINT)(subnetMask_p & 0xFF));

    snprintf(sBufferMtu,
             sizeof(sBufferMtu),
             "%u",
             (UINT)mtu_p);

    /* call ifconfig to configure the virtual network interface */
    sprintf(command,
            "/sbin/ifconfig %s %s netmask %s mtu %s",
            ifName_p,
            sBufferIp,
            sBufferMask,
            sBufferMtu);

    iRet = system(command);
    if (iRet < 0)
    {
        TRACE("ifconfig %s %s returned %d\n", ifName_p, sBufferIp, iRet);
        return kErrorNoResource;
    }

    return ret;
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
    tOplkError  ret = kErrorOk;
    INT         iRet;
    char        sBuffer[16];
    char        command[128];

    if (defaultGateway_p != 0)
    {
        // configure default gateway of virtual network interface
        // for TCP/IP communication over the POWERLINK network
        snprintf(sBuffer,
                 sizeof(sBuffer),
                 "%u.%u.%u.%u",
                 (UINT)(defaultGateway_p >> 24),
                 (UINT)((defaultGateway_p >> 16) & 0xFF),
                 (UINT)((defaultGateway_p >> 8) & 0xFF),
                 (UINT)(defaultGateway_p & 0xFF));

        sprintf(command, "route del default");
        iRet = system(command);
        TRACE("route del default returned %d\n", iRet);

        /* call route to configure the default gateway */
        sprintf(command, "route add default gw %s", sBuffer);
        iRet = system(command);
        if (iRet < 0)
        {
            TRACE("route add default gw %s returned %d\n", sBuffer, iRet);
            return kErrorNoResource;
        }
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief    Enables global interrupt

This function enables/disables global interrupts.

\param[in]      fEnable_p           TRUE = enable interrupts
                                    FALSE = disable interrupts

\note This function is implemented empty for the sim target
\ingroup module_target
*/
//------------------------------------------------------------------------------
void target_enableGlobalInterrupt(BOOL fEnable_p)
{
    UNUSED_PARAMETER(fEnable_p);
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
\brief    Get current system tick

This function returns the current system tick determined by the system timer.

\return Returns the system tick in milliseconds

\ingroup module_target
*/
//------------------------------------------------------------------------------
UINT32 target_getTickCount(void)
{
    UINT32          ticks;
    struct timespec curTime;

    clock_gettime(CLOCK_MONOTONIC, &curTime);
    ticks = (curTime.tv_sec * 1000) + (curTime.tv_nsec / 1000000);

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

    return kErrorOk;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

/// \}
