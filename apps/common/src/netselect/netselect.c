/**
********************************************************************************
\file   netselect.c

\brief  Implementation of the network interface selection functions.

This header file provides the implementation for the network interface selection
functions used by the openPOWERLINK examples.

\ingroup module_app_common
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2017, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
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
#include "netselect.h"

#include <stdio.h>
#include <string.h>

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
#define MAX_INTERFACES  10  // Define the max. number of interfaces shown to the user

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
\brief  Select network interface

With this function the network interface to be used for openPOWERLINK is selected
from a list of devices.

\param[out]     pDevName_p          Pointer to store the device name which
                                    should be used.
\param[in]      maxLen_p            Max length of the string to store the device
                                    name.

\return The function returns 0 if a device could be selected, otherwise -1.

*/
//------------------------------------------------------------------------------
int netselect_selectNetworkInterface(char* pDevName_p, size_t maxLen_p)
{
    tNetIfId        aInterfaces[MAX_INTERFACES];
    size_t          noInterfaces = sizeof(aInterfaces) / sizeof(aInterfaces[0]);
    size_t          i = 0;
    unsigned int    num;

    if (oplk_enumerateNetworkInterfaces(aInterfaces, &noInterfaces) == kErrorOk)
    {
        printf("--------------------------------------------------\n");
        printf("List of Ethernet cards found in this system:\n");
        printf("--------------------------------------------------\n");

        for (i = 0; i < noInterfaces; i++)
        {
            // Print adapters
            printf("%u. ", (unsigned int)i + 1);
            printf("%s\n      %s\n", aInterfaces[i].aDeviceDescription, aInterfaces[i].aDeviceName);
        }
    }
    else
        return -1;

    printf("--------------------------------------------------\n");
    printf("Select the interface to be used for POWERLINK (1-%u):", (unsigned int)i);
    if (scanf("%u", &num) == EOF)
    {
        return -1;
    }

    printf("--------------------------------------------------\n");
    if ((num < 1) || (num > i))
    {
        printf("\nInterface number out of range.\n");
        return -1;
    }

    // Return the selected interface name
    strncpy(pDevName_p, aInterfaces[num - 1].aDeviceName, maxLen_p);

    return 0;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

/// \}
