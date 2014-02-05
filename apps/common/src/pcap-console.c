/**
********************************************************************************
\file   pcap-console.c

\brief  Implementation of PCAP functions for console applications

This file provides functions which help console applciations using the PCAP
library.

\ingroup module_app_common
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2013, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
Copyright (c) 2013, SYSTEC electronic GmbH
Copyright (c) 2013, Kalycito Infotech Private Ltd.All rights reserved.
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

// todo how could we remove that?
#if (TARGET_SYSTEM == _WIN32_)
#define _WINSOCKAPI_ // prevent windows.h from including winsock.h
#endif  // (TARGET_SYSTEM == _WIN32_)

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <oplk/oplk.h>
#include <pcap.h>

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
\brief  Select PCAP device

The function is used to select the PCAP device to be used for openPOWERLINK
from a list of devices

\param  pDevName_p              Pointer to store device name which should be used.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
tOplkError selectPcapDevice(char *pDevName_p)
{
    char            sErr_Msg[ PCAP_ERRBUF_SIZE ];
    pcap_if_t *     alldevs;
    pcap_if_t *     seldev;
    int             i = 0;
    int             inum;

    /* Retrieve the device list on the local machine */
    if (pcap_findalldevs(&alldevs, sErr_Msg) == -1)
    {
        fprintf(stderr, "Error in pcap_findalldevs: %s\n", sErr_Msg);
        return kErrorNoResource;
    }

    PRINTF("--------------------------------------------------\n");
    PRINTF("List of Ethernet Cards Found in this System: \n");
    PRINTF("--------------------------------------------------\n");
    for (seldev = alldevs; seldev != NULL; seldev = seldev->next)
    {
        PRINTF("%d. ", ++i);
        if (seldev->description)
        {
            PRINTF("%s\n      %s\n", seldev->description, seldev->name);
        }
        else
        {
            PRINTF("%s\n", seldev->name);
        }
    }

    if (i == 0)
    {
        PRINTF("\nNo interfaces found! Make sure pcap library is installed.\n");
        return kErrorNoResource;
    }

    PRINTF("--------------------------------------------------\n");
    PRINTF("Select the interface to be used for POWERLINK (1-%d):",i);
    if (scanf("%d", &inum) == EOF)
    {
        pcap_freealldevs(alldevs);
        return kErrorNoResource;
    }

    PRINTF("--------------------------------------------------\n");
    if ((inum < 1) || (inum > i))
    {
        PRINTF("\nInterface number out of range.\n");
        pcap_freealldevs(alldevs);
        return kErrorNoResource;
    }

    /* Jump to the selected adapter */
    for (seldev = alldevs, i = 0; i < (inum - 1); seldev = seldev->next, i++)
    {   // do nothing
    }
    strncpy(pDevName_p, seldev->name, 127);

    return kErrorOk;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{


///\}







