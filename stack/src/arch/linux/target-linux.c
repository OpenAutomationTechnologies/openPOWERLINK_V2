/**
********************************************************************************
\file   linux/target-linux.c

\brief  Target specific functions for Linux

The file implements target specific functions used in the openPOWERLINK stack.

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
#include <fcntl.h>
#include <signal.h>
#include <time.h>
#include <sys/time.h>

#include <common/oplkinc.h>
#include <common/ftracedebug.h>

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
    tOplkError  Ret  = kErrorOk;
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

    return Ret;
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
    tOplkError  Ret  = kErrorOk;

    /* Disable ftrace debugging */
    FTRACE_ENABLE(FALSE);

    return Ret;
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

\param  ifName_p                Name of ethernet interface.
\param  ipAddress_p             IP address to set for interface.
\param  subnetMask_p            Subnet mask to set for interface.
\param  mtu_p                   MTU to set for interface.

\return The function returns a tOplkError error code.

\ingroup module_target
*/
//------------------------------------------------------------------------------
tOplkError target_setIpAdrs(char* ifName_p, UINT32 ipAddress_p, UINT32 subnetMask_p, UINT16 mtu_p)
{
    tOplkError  ret = kErrorOk;
    INT         iRet;
    char        sBufferIp[16];
    char        sBufferMask[16];
    char        sBufferMtu[6];
    char        command[256];

    // configure IP address of virtual network interface
    // for TCP/IP communication over the POWERLINK network
    snprintf(sBufferIp, sizeof(sBufferIp), "%u.%u.%u.%u",
             (UINT)(ipAddress_p >> 24), (UINT)((ipAddress_p >> 16) & 0xFF),
             (UINT)((ipAddress_p >> 8) & 0xFF), (UINT)(ipAddress_p & 0xFF));

    snprintf(sBufferMask, sizeof(sBufferMask), "%u.%u.%u.%u",
             (UINT)(subnetMask_p >> 24), (UINT)((subnetMask_p >> 16) & 0xFF),
             (UINT)((subnetMask_p >> 8) & 0xFF), (UINT)(subnetMask_p & 0xFF));

    snprintf(sBufferMtu, sizeof(sBufferMtu), "%u", (UINT)mtu_p);

    /* call ifconfig to configure the virtual network interface */
    sprintf (command, "/sbin/ifconfig %s %s netmask %s mtu %s",
             ifName_p, sBufferIp, sBufferMask, sBufferMtu);
    if ((iRet = system(command)) < 0)
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

\param  defaultGateway_p            Default gateway to set.

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
        snprintf(sBuffer, sizeof(sBuffer), "%u.%u.%u.%u",
                 (UINT)(defaultGateway_p >> 24), (UINT)((defaultGateway_p >> 16) & 0xFF),
                 (UINT)((defaultGateway_p >> 8) & 0xFF), (UINT)(defaultGateway_p & 0xFF));

        sprintf(command, "route del default");
        iRet = system(command);
        TRACE("route del default returned %d\n", iRet);

        /* call route to configure the default gateway */
        sprintf(command, "route add default gw %s", sBuffer);
        if ((iRet = system(command)) < 0)
        {
            TRACE("route add default gw %s returned %d\n", sBuffer, iRet);
            return kErrorNoResource;
        }
    }
    return ret;
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
    UINT32                  ticks;
    struct timespec         curTime;

    clock_gettime(CLOCK_MONOTONIC, &curTime);
    ticks = (curTime.tv_sec * 1000) + (curTime.tv_nsec / 1000000);

    return ticks;
}
