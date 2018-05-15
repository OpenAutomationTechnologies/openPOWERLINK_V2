/**
********************************************************************************
\file   netif-linux.c

\brief  Implementation of network interface enumeration functions for Linux

This file contains the implementation of helper functions for enumerating the
network interfaces in Linux.

\ingroup module_target
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2017, B&R Industrial Automation GmbH
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

#include <sys/types.h>
#include <sys/socket.h>
#include <linux/if_packet.h>
#include <net/ethernet.h>
#include <ifaddrs.h>
#include <errno.h>

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
\brief  Enumerate network interfaces

This function enumerates all available network interfaces to be used by
openPOWERLINK.

\param[out]     pInterfaces_p       Pointer to store the list of
                                    found interfaces.
\param[in,out]  pNoInterfaces_p     Pointer to the number of interfaces.
                                    The maximum number of interfaces to be
                                    stored is passed to the function.
                                    The number of interfaces found is returned.

\return The function returns a \ref tOplkError error code.
*/
//------------------------------------------------------------------------------
tOplkError target_enumerateNetworkInterfaces(tNetIfId* pInterfaces_p,
                                             size_t* pNoInterfaces_p)
{
    struct ifaddrs* ifaddr;
    struct ifaddrs* ifa;
    int             family;
    size_t          i = 0;

    // Check parameter validity
    ASSERT(pInterfaces_p != NULL);
    ASSERT(pNoInterfaces_p != NULL);

    // search for the interfaces
    if (getifaddrs(&ifaddr) == -1)
    {
        DEBUG_LVL_ERROR_TRACE("%s() Error retrieving the list of network interfaces. Errno: %i\n",
                              __func__, errno);
        return kErrorNoResource;
    }

    // Walk through linked list maintaining the head pointer,
    // so we can free the list later
    for (ifa = ifaddr; ifa != NULL; ifa = ifa->ifa_next)
    {
        if (ifa->ifa_addr == NULL)
            continue;

        // only list packet interfaces
        family = ifa->ifa_addr->sa_family;
        if (family == AF_PACKET)
        {
            if (i < *pNoInterfaces_p)
            {
                memcpy(pInterfaces_p[i].aMacAddress,
                       ((struct sockaddr_ll*)ifa->ifa_addr)->sll_addr,
                       sizeof(pInterfaces_p[i].aMacAddress));
                strcpy(pInterfaces_p[i].aDeviceName, ifa->ifa_name);
                strcpy(pInterfaces_p[i].aDeviceDescription, ifa->ifa_name);
            }
            i++;
        }
    }

    freeifaddrs(ifaddr);
    *pNoInterfaces_p = i;

    return kErrorOk;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

/// \}
