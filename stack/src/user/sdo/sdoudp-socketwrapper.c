/**
********************************************************************************
\file   sdoudp-socketwrapper.c

\brief  Implementation of SDO over UDP protocol with socket wrapper

This file contains the implementation of the SDO over UDP protocol with using
the socket wrapper.

\ingroup module_sdo_udp
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
#include <user/sdoudp.h>

#include <socketwrapper.h>

#if defined(CONFIG_INCLUDE_SDO_UDP)

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

/**
\brief  SDO/UDP socket wrapper instance

This structure defines the SDO/UDP socket wrapper instance.
*/
typedef struct
{
    tSocketWrapper          pSocketWrapper;     ///< Socket wrapper instance
} tSdoUdpSocketInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tSdoUdpSocketInstance    instance_l;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static void receiveFromSocket(const void* pData_p,
                              size_t dataSize_p,
                              const tSocketWrapperAddress* pRemote_p);

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Initialize SDO over UDP socket module

The function initializes the SDO over UDP socket module.

\return The function returns a tOplkError error code.

\ingroup module_sdo_udp
*/
//------------------------------------------------------------------------------
tOplkError sdoudp_initSocket(void)
{
    OPLK_MEMSET(&instance_l, 0x00, sizeof(instance_l));

    instance_l.pSocketWrapper = SOCKETWRAPPER_INVALID;

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Shut down SDO over UDP socket module

The function shuts down the SDO over UDP socket module.

\ingroup module_sdo_udp
*/
//------------------------------------------------------------------------------
void sdoudp_exitSocket(void)
{

}

//------------------------------------------------------------------------------
/**
\brief  Create socket for SDO over UDP

The function creates a socket for the SDO over UDP connection.

\param[in,out]  pSdoUdpCon_p        UDP connection for which a socket shall be created.

\return The function returns a tOplkError error code.

\ingroup module_sdo_udp
*/
//------------------------------------------------------------------------------
tOplkError sdoudp_createSocket(tSdoUdpCon* pSdoUdpCon_p)
{
    tOplkError              ret;
    tSocketWrapperAddress   socketAddr;

    // Check parameter validity
    ASSERT(pSdoUdpCon_p != NULL);

    instance_l.pSocketWrapper = socketwrapper_create(receiveFromSocket);
    if (instance_l.pSocketWrapper == SOCKETWRAPPER_INVALID)
    {
        DEBUG_LVL_ERROR_TRACE("%s() socketwrapper create failed\n", __func__);
        return kErrorSdoUdpNoSocket;
    }

    if (pSdoUdpCon_p->ipAddr == SDOUDP_INADDR_ANY)
        pSdoUdpCon_p->ipAddr = SOCKETWRAPPER_INADDR_ANY;

    socketAddr.ipAddress = pSdoUdpCon_p->ipAddr;
    socketAddr.port = pSdoUdpCon_p->port;

    ret = socketwrapper_bind(instance_l.pSocketWrapper, &socketAddr);

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Close socket for SDO over UDP

The function closes the created socket for the SDO over UDP connection.

\return The function returns a tOplkError error code.

\ingroup module_sdo_udp
*/
//------------------------------------------------------------------------------
tOplkError sdoudp_closeSocket(void)
{
    if (instance_l.pSocketWrapper != SOCKETWRAPPER_INVALID)
    {
        socketwrapper_close(instance_l.pSocketWrapper);
        instance_l.pSocketWrapper = SOCKETWRAPPER_INVALID;
    }

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Send SDO over UDP frame

The function sends an SDO frame to the given UDP connection.

\param[in]      pSdoUdpCon_p        UDP connection to send the frame to.
\param[in]      pSrcData_p          Pointer to frame data which should be sent.
\param[in]      dataSize_p          Size of data to be send.

\return The function returns a tOplkError error code.

\ingroup module_sdo_udp
*/
//------------------------------------------------------------------------------
tOplkError sdoudp_sendToSocket(const tSdoUdpCon* pSdoUdpCon_p,
                               const tPlkFrame* pSrcData_p,
                               size_t dataSize_p)
{
    tOplkError              ret;
    tSocketWrapperAddress   remote;

    // Check parameter validity
    ASSERT(pSdoUdpCon_p != NULL);
    ASSERT(pSrcData_p != NULL);

    remote.ipAddress = pSdoUdpCon_p->ipAddr;
    remote.port = pSdoUdpCon_p->port;

    ret = socketwrapper_send(instance_l.pSocketWrapper,
                             &remote,
                             &pSrcData_p->messageType,
                             dataSize_p);

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Enter/leave critical section

The function enters or leaves a critical section to ensure correct operation of
the SDO UDP module.

\param[in]      fEnable_p           Specifies if the critical section shall be entered or
                                    left.
                                    If TRUE, the critical section is entered.
                                    If FALSE, the critical section is left.

\return The function returns a tOplkError error code.

\ingroup module_sdo_udp
*/
//------------------------------------------------------------------------------
void sdoudp_criticalSection(BOOL fEnable_p)
{
    socketwrapper_criticalSection(fEnable_p);
}

//------------------------------------------------------------------------------
/**
\brief  Query ARP table

The function enables triggering ARP to obtain the remote node's Ethernet address.

\param[in]      remoteIpAddr_p      The remote node's IP address

\return The function returns a tOplkError error code.
\retval kErrorOk                    The Ethernet address for the given IP is known.
\retval kErrorSdoUdpArpInProgress   The Ethernet address for the given IP is not known.
                                    ARP has been triggered to obtain the Ethernet address.

\ingroup module_sdo_udp
*/
//------------------------------------------------------------------------------
tOplkError sdoudp_arpQuery(UINT32 remoteIpAddr_p)
{
    return socketwrapper_arpQuery(instance_l.pSocketWrapper, remoteIpAddr_p);
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief  Receive data from socket

The function receives data from the UDP socket.

\param[out]     pData_p             Pointer to received data.
\param[in]      dataSize_p          Size of received data.
\param[in]      pRemote_p           Pointer to address structure of remote.

*/
//------------------------------------------------------------------------------
static void receiveFromSocket(const void* pData_p,
                              size_t dataSize_p,
                              const tSocketWrapperAddress* pRemote_p)
{
    const tAsySdoSeq*   pSdoSeqData = (const tAsySdoSeq*)((const UINT8*)pData_p + ASND_HEADER_SIZE);
    size_t              size = dataSize_p - ASND_HEADER_SIZE;
    tSdoUdpCon          sdoUdpCon;

    sdoUdpCon.ipAddr = pRemote_p->ipAddress;
    sdoUdpCon.port = pRemote_p->port;

    sdoudp_receiveData(&sdoUdpCon, pSdoSeqData, size);
}

/// \}

#endif
