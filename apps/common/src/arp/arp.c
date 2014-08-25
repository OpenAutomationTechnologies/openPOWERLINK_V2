/**
********************************************************************************
\file   arp.c

\brief  ARP demo implementation

This file implements an ARP demo for demo targets that have no IP stack.

\ingroup module_app_common
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
#include "arp.h"
#include "arp-target.h"

#include <oplk/oplk.h>

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

#define ARP_ETHERTYPE           0x0806  ///< ARP "EtherType"
#define ARP_HWTYPE_ETHERNET     1       ///< ARP hardware type Ethernet
#define ARP_PROTYPE_IPV4        0x0800  ///< ARP protocol type IP V4
#define ARP_HWADDR_LENGTH       6       ///< ARP hardware address length Ethernet
#define ARP_PROADDR_LENGTH      4       ///< ARP protocol address length IP V4
#define ARP_OP_REQUEST          1       ///< ARP operation request
#define ARP_OP_REPLY            2       ///< ARP operation reply

//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------
/**
 * \brief Ethernet with ARP message
 *
 * This struct defines an Ethernet frame with ARP message payload.
 *
 */
typedef struct
{
    UINT8   aDstMac[ARP_HWADDR_LENGTH];                 ///< Ethernet destination MAC address
    UINT8   aSrcMac[ARP_HWADDR_LENGTH];                 ///< Ethernet source MAC address
    UINT16  etherType;                                  ///< Ethernet "etherType"
    UINT16  hardwareType;                               ///< ARP network protocol type
    UINT16  protocolType;                               ///< ARP internetwork protocol type
    UINT8   hardwareAddressLength;                      ///< ARP length of hardware address [octets]
    UINT8   protocolAddressLength;                      ///< ARP length of internetwork address [octets]
    UINT16  operation;                                  ///< ARP operation
    UINT8   aSenderHardwareAddress[ARP_HWADDR_LENGTH];  ///< ARP sender hardware address
    UINT8   aSenderProtocolAddress[ARP_PROADDR_LENGTH]; ///< ARP sender internetwork address
    UINT8   aTargetHardwareAddress[ARP_HWADDR_LENGTH];  ///< ARP target hardware address
    UINT8   aTargetProtocolAddress[ARP_PROADDR_LENGTH]; ///< ARP target internetwork address
} tArpFrame;

/**
 * \brief ARP instance
 *
 * This struct defines the ARP instance.
 *
 */
typedef struct
{
    tArpFrame   frameTemplate;                  ///< ARP frame template
    UINT8       nodeId;                         ///< The local node ID
    UINT8       aMacAddr[ARP_HWADDR_LENGTH];    ///< The local node's MAC address in network order
    UINT8       aIpAddr[ARP_PROADDR_LENGTH];    ///< The local node's IP address in network order
} tArpInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tArpInstance arpInstance_l;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static tOplkError handleReply(tArpFrame* pFrame_p);
static tOplkError handleRequest(tArpFrame* pFrame_p);

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Initialize ARP module

The function initializes the ARP module before being used.

\param  nodeId_p    The local node ID

\return The function returns a tOplkError error code.

\ingroup module_app_common
*/
//------------------------------------------------------------------------------
tOplkError arp_init(UINT8 nodeId_p)
{
    tOplkError  ret = kErrorOk;
    tArpFrame*  pFrame;

    memset(&arpInstance_l, 0, sizeof(tArpInstance));

    arpInstance_l.nodeId = nodeId_p;

    // Set ARP frame template
    pFrame = &arpInstance_l.frameTemplate;

    // EtherType: ARP
    pFrame->etherType = htons(ARP_ETHERTYPE);

    // Hardware Type: Ethernet
    pFrame->hardwareType = htons(ARP_HWTYPE_ETHERNET);

    // Protocol Type: IP V4
    pFrame->protocolType = htons(ARP_PROTYPE_IPV4);

    // Hardware Address Length
    pFrame->hardwareAddressLength = ARP_HWADDR_LENGTH;

    // Protocol Address Length
    pFrame->protocolAddressLength = ARP_PROADDR_LENGTH;

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Shutdown ARP module

The function shuts down the ARP module.

\ingroup module_app_common
*/
//------------------------------------------------------------------------------
void arp_shutdown(void)
{
}

//------------------------------------------------------------------------------
/**
\brief  Set local node MAC address

The function sets the local node's MAC address.

\param  pMacAddr_p      Pointer to buffer that holds the MAC address to be set

\return The function returns a tOplkError error code.

\ingroup module_app_common
*/
//------------------------------------------------------------------------------
tOplkError arp_setMacAddr(UINT8* pMacAddr_p)
{
    memcpy(&arpInstance_l.aMacAddr, pMacAddr_p, ARP_HWADDR_LENGTH);

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Set local node IP address

The function sets the local node's IP address.

\param  ipAddr_p    IP address to be set

\return The function returns a tOplkError error code.

\ingroup module_app_common
*/
//------------------------------------------------------------------------------
tOplkError arp_setIpAddr(UINT32 ipAddr_p)
{
    UINT32 ipAddr = htonl(ipAddr_p); // Swap to get network order

    memcpy(arpInstance_l.aIpAddr, (UINT8*)&ipAddr, ARP_PROADDR_LENGTH);

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Send ARP request to POWERLINK node

The function sends an ARP request to the given POWERLINK node.

\return The function returns a tOplkError error code.

\ingroup module_app_common
*/
//------------------------------------------------------------------------------
tOplkError arp_sendRequest(UINT8 dstNodeId_p)
{
    tOplkError  ret = kErrorOk;
    tArpFrame   frameBuffer;
    tArpFrame*  pFrame = &frameBuffer;

    // Copy ARP frame template to frame buffer
    memcpy(pFrame, &arpInstance_l.frameTemplate, sizeof(tArpFrame));

    // Set Destination MAC address (broadcast)
    memset(pFrame->aDstMac, 0xFF, ARP_HWADDR_LENGTH);

    // Set ARP operation
    pFrame->operation = htons(ARP_OP_REQUEST);

    // Sender IP Address
    memcpy(pFrame->aSenderProtocolAddress, arpInstance_l.aIpAddr, ARP_PROADDR_LENGTH);

    // Overwrite last byte with node ID
    pFrame->aSenderProtocolAddress[3] = arpInstance_l.nodeId;

    // Sender MAC Address
    memcpy(pFrame->aSenderHardwareAddress, &arpInstance_l.aMacAddr[0], ARP_HWADDR_LENGTH);

    // Target IP Address
    memcpy(pFrame->aTargetProtocolAddress, arpInstance_l.aIpAddr, ARP_PROADDR_LENGTH);

    // Overwrite last byte with destination node ID
    pFrame->aTargetProtocolAddress[3] = dstNodeId_p;

    ret = oplk_sendEthFrame((tPlkFrame*)pFrame, sizeof(tArpFrame));

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Process Ethernet ARP frames

The function processes a received Ethernet frame for ARP handling.

\param  pFrame_p    Pointer to ARP frame

\return The function returns a tOplkError error code.

\ingroup module_app_common
*/
//------------------------------------------------------------------------------
tOplkError arp_processReceive(tPlkFrame* pFrame_p, UINT size_p)
{
    tOplkError  ret = kErrorOk;
    tArpFrame*  pFrame = (tArpFrame*)pFrame_p;

    // Handle ARP frames: Check etherType and frame size
    if ((ntohs(pFrame->etherType) == ARP_ETHERTYPE) &&
        (size_p >= sizeof(tArpFrame)))
    {
        //Check for correct hardware, protocol type and length fields
        if ((ntohs(pFrame->hardwareType) == ARP_HWTYPE_ETHERNET) &&
            (ntohs(pFrame->protocolType) == ARP_PROTYPE_IPV4) &&
            (pFrame->hardwareAddressLength == ARP_HWADDR_LENGTH) &&
            (pFrame->protocolAddressLength == ARP_PROADDR_LENGTH))
        {
            switch (ntohs(pFrame->operation))
            {
                case ARP_OP_REPLY:
                    ret = handleReply(pFrame);
                    break;

                case ARP_OP_REQUEST:
                    ret = handleRequest(pFrame);
                    break;

                default:
                    break;
            }
        }
    }
    else
    {
        // It is no ARP frame => other protocol shall try...
        ret = kErrorRetry;
    }

    return ret;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief  Handle ARP Reply frame

The function handles an ARP Reply frame.

\param  pFrame_p    Pointer to ARP frame

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError handleReply(tArpFrame* pFrame_p)
{
    tOplkError ret = kErrorOk;

#ifdef NDEBUG
    UNUSED_PARAMETER(pFrame_p);
#endif

    PRINTF("ARP: Node with IP Address %d.%d.%d.%d ",
           pFrame_p->aSenderProtocolAddress[0],
           pFrame_p->aSenderProtocolAddress[1],
           pFrame_p->aSenderProtocolAddress[2],
           pFrame_p->aSenderProtocolAddress[3]);
    PRINTF("has MAC Address %02X:%02X:%02X:%02X:%02X:%02X\n",
           pFrame_p->aSenderHardwareAddress[0],
           pFrame_p->aSenderHardwareAddress[1],
           pFrame_p->aSenderHardwareAddress[2],
           pFrame_p->aSenderHardwareAddress[3],
           pFrame_p->aSenderHardwareAddress[4],
           pFrame_p->aSenderHardwareAddress[5]);

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Handle ARP Request frame

The function handles an ARP Request frame by sending the corresponding ARP reply.

\param  pFrame_p    Pointer to ARP frame

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError handleRequest(tArpFrame* pFrame_p)
{
    tOplkError  ret = kErrorOk;

    // Reply if the request is addressing us
    if (memcmp(pFrame_p->aTargetProtocolAddress, arpInstance_l.aIpAddr, ARP_PROADDR_LENGTH) == 0)
    {
        tArpFrame   frameBuffer;
        tArpFrame*  pFrame = &frameBuffer;
        tArpFrame*  pRxFrame = pFrame_p;

        // Copy ARP frame template to frame buffer
        memcpy(pFrame, &arpInstance_l.frameTemplate, sizeof(tArpFrame));

        // Copy Destination MAC address to received Source MAC
        memcpy(pFrame->aDstMac, pRxFrame->aSrcMac, ARP_HWADDR_LENGTH);

        // Set ARP operation
        pFrame->operation = htons(ARP_OP_REPLY);

        // Sender IP Address
        memcpy(pFrame->aSenderProtocolAddress, arpInstance_l.aIpAddr, ARP_PROADDR_LENGTH);

        // Overwrite last byte with node ID
        pFrame->aSenderProtocolAddress[3] = arpInstance_l.nodeId;

        // Sender MAC Address
        memcpy(pFrame->aSenderHardwareAddress, &arpInstance_l.aMacAddr[0], ARP_HWADDR_LENGTH);

        // Copy received sender info to target info
        memcpy(pFrame->aTargetHardwareAddress, pRxFrame->aSenderHardwareAddress, ARP_HWADDR_LENGTH);
        memcpy(pFrame->aTargetProtocolAddress, pRxFrame->aSenderProtocolAddress, ARP_PROADDR_LENGTH);

        ret = oplk_sendEthFrame((tPlkFrame*)pFrame, sizeof(tArpFrame));
    }

    return kErrorOk;
}

/// \}
