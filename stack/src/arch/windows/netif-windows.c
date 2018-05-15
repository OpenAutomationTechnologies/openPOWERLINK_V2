/**
********************************************************************************
\file   netif-windows.c

\brief  Implementation of network interface enumeration functions for Windows

This file contains the implementation of helper functions for enumerating the
network interfaces in Windows.

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

#include <winsock2.h>
#include <iphlpapi.h>

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
    ULONG                 outBufLen;
    PIP_ADAPTER_ADDRESSES pAdapterAddresses;
    PIP_ADAPTER_ADDRESSES pAdapter = NULL;
    UINT32                retVal = 0;
    size_t                i = 0;

    // Check parameter validity
    ASSERT(pInterfaces_p != NULL);
    ASSERT(pNoInterfaces_p != NULL);

    // search for the corresponding MAC address via IPHLPAPI
    outBufLen = sizeof(IP_ADAPTER_ADDRESSES);
    pAdapterAddresses = (IP_ADAPTER_ADDRESSES*)malloc(sizeof(IP_ADAPTER_ADDRESSES));
    if (pAdapterAddresses == NULL)
    {
        DEBUG_LVL_ERROR_TRACE("%s() Error allocating memory needed to call GetAdaptersAdresses\n", __func__);
        return kErrorNoResource;
    }

    // Make an initial call to GetAdaptersAdresses to get
    // the necessary size into the outBufLen variable
    retVal = GetAdaptersAddresses(AF_UNSPEC, 0, NULL, pAdapterAddresses, &outBufLen);
    if (retVal == ERROR_BUFFER_OVERFLOW)
    {
        pAdapter = pAdapterAddresses;
        pAdapterAddresses = (IP_ADAPTER_ADDRESSES*)realloc(pAdapter, outBufLen);
        if (pAdapterAddresses == NULL)
        {
            DEBUG_LVL_ERROR_TRACE("%s() Error allocating memory needed to call GetAdaptersAdresses\n", __func__);
            free(pAdapter);
            return kErrorNoResource;
        }
    }

    // Get the real values
    retVal = GetAdaptersAddresses(AF_UNSPEC, 0, NULL, pAdapterAddresses, &outBufLen);
    if (retVal == NO_ERROR)
    {
        pAdapter = pAdapterAddresses;
        while (pAdapter != NULL)
        {
            if (pAdapter->IfType == IF_TYPE_ETHERNET_CSMACD)
            {
                if (i < *pNoInterfaces_p)
                {
                    memcpy(pInterfaces_p[i].aMacAddress,
                           pAdapter->PhysicalAddress,
                           pAdapter->PhysicalAddressLength);
                    strcpy(pInterfaces_p[i].aDeviceName, pAdapter->AdapterName);
                    wcstombs(pInterfaces_p[i].aDeviceDescription,
                             pAdapter->FriendlyName,
                             sizeof(pInterfaces_p[i].aDeviceDescription));
                }
                i++;
            }

            pAdapter = pAdapter->Next;
        }

        *pNoInterfaces_p = i;
    }
    else
    {
        DEBUG_LVL_ERROR_TRACE("%s() GetAdaptersAddresses failed with error: %d\n", __func__, retVal);
        free(pAdapterAddresses);
        return kErrorNoResource;
    }
    free(pAdapterAddresses);

    return kErrorOk;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

/// \}
