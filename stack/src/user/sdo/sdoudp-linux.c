/**
********************************************************************************
\file   sdoudp-linux.c

\brief  Implementation of SDO over UDP protocol for Linux

This file contains the implementation of the SDO over UDP protocol for Linux.

\ingroup module_sdo_udp
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2017, B&R Industrial Automation GmbH
Copyright (c) 2013, SYSTEC electronic GmbH
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

#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <pthread.h>

#include <errno.h>

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
#define INVALID_SOCKET  0

//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------
typedef int SOCKET;

typedef void* tThreadResult;
typedef void* tThreadArg;

typedef struct
{
    SOCKET                      udpSocket;
    pthread_t                   threadHandle;
    BOOL                        fStopThread;
} tSdoUdpSocketInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tSdoUdpSocketInstance    instance_l;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static void          receiveFromSocket(const tSdoUdpSocketInstance* pInstance_p);
static tThreadResult sdoUdpThread(tThreadArg pArg_p);

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

    instance_l.threadHandle = 0;
    instance_l.udpSocket = INVALID_SOCKET;

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
    struct sockaddr_in  addr;
    int                 error;

    // Check parameter validity
    ASSERT(pSdoUdpCon_p != NULL);

    instance_l.udpSocket = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (instance_l.udpSocket == INVALID_SOCKET)
    {
        DEBUG_LVL_SDO_TRACE("%s(): socket() failed\n", __func__);
        return kErrorSdoUdpNoSocket;
    }

    if (pSdoUdpCon_p->ipAddr == SDOUDP_INADDR_ANY)
        pSdoUdpCon_p->ipAddr = INADDR_ANY;

    // bind socket
    addr.sin_family = AF_INET;
    addr.sin_port = htons(pSdoUdpCon_p->port);
    addr.sin_addr.s_addr = htonl(pSdoUdpCon_p->ipAddr);
    error = bind(instance_l.udpSocket, (struct sockaddr*)&addr, sizeof(addr));
    if (error < 0)
    {
        DEBUG_LVL_SDO_TRACE("%s(): bind() finished with %i\n", __func__, error);
        return kErrorSdoUdpNoSocket;
    }

    // create Listen-Thread
    instance_l.fStopThread = FALSE;

    if (pthread_create(&instance_l.threadHandle, NULL, sdoUdpThread, &instance_l) != 0)
        return kErrorSdoUdpThreadError;

#if (defined(__GLIBC__) && (__GLIBC__ >= 2) && (__GLIBC_MINOR__ >= 12))
    pthread_setname_np(instance_l.threadHandle, "oplk-sdoudp");
#endif

    return kErrorOk;
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
    int error;

    if (instance_l.threadHandle != 0)
    {   // listen thread was started -> close old thread

        instance_l.fStopThread = TRUE;
        if (pthread_join(instance_l.threadHandle, NULL) != 0)
            return kErrorSdoUdpThreadError;

        instance_l.threadHandle = 0;
    }

    if (instance_l.udpSocket != INVALID_SOCKET)
    {
        error = close(instance_l.udpSocket);
        instance_l.udpSocket = INVALID_SOCKET;
        if (error != 0)
            return kErrorSdoUdpSocketError;
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
    struct sockaddr_in  addr;
    int                 error;

    // Check parameter validity
    ASSERT(pSdoUdpCon_p != NULL);
    ASSERT(pSrcData_p != NULL);

    addr.sin_family = AF_INET;
    addr.sin_port = pSdoUdpCon_p->port;
    addr.sin_addr.s_addr = pSdoUdpCon_p->ipAddr;

    error = sendto(instance_l.udpSocket,
                   (const char*)&pSrcData_p->messageType,
                   dataSize_p,
                   0,
                   (struct sockaddr*)&addr,
                   sizeof(struct sockaddr_in));
    if (error < 0)
    {
        DEBUG_LVL_SDO_TRACE("%s(): sendto() finished with %i\n", __func__, error);
        return kErrorSdoUdpSendError;
    }

    return kErrorOk;
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

\ingroup module_sdo_udp
*/
//------------------------------------------------------------------------------
void sdoudp_criticalSection(BOOL fEnable_p)
{
    UNUSED_PARAMETER(fEnable_p);

    //TODO: Do we need critical section handling here?
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
    UNUSED_PARAMETER(remoteIpAddr_p);

    // The Linux network stack will take care, so no need to bother!
    return kErrorOk;
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

\param[in]      pInstance_p         Pointer to SDO instance.

*/
//------------------------------------------------------------------------------
static void receiveFromSocket(const tSdoUdpSocketInstance* pInstance_p)
{
    struct sockaddr_in  remoteAddr;
    int                 error;
    UINT8               aBuffer[SDO_MAX_RX_FRAME_SIZE_UDP];
    size_t              size;
    tSdoUdpCon          sdoUdpCon;

    OPLK_MEMSET(&remoteAddr, 0, sizeof(remoteAddr));

    size = sizeof(struct sockaddr);

    error = recvfrom(pInstance_p->udpSocket,
                     (char*)&aBuffer[0],
                     sizeof(aBuffer),
                     0,
                     (struct sockaddr*)&remoteAddr,
                     (socklen_t*)&size);
    if (error > 0)
    {
        const tAsySdoSeq*   pSdoSeqData;
        size_t              dataSize = (size_t)error - ASND_HEADER_SIZE;

        pSdoSeqData = (const tAsySdoSeq*)&aBuffer[ASND_HEADER_SIZE];
        sdoUdpCon.ipAddr = remoteAddr.sin_addr.s_addr;
        sdoUdpCon.port = remoteAddr.sin_port;

        sdoudp_receiveData(&sdoUdpCon, pSdoSeqData, dataSize);
    }
    else
    {
        DEBUG_LVL_SDO_TRACE("%s() error=%d\n", __func__, error);
    }
}

//------------------------------------------------------------------------------
/**
\brief  UDP Receiving thread function

The function implements the UDP receive thread. It waits for packets on the
UDP socket and calls receiveFromSocket() if data is available.

\param[in]      pArg_p              Thread argument. The pointer to the SDO instance is
                                    transferred to the thread as thread argument.

\return The function returns a thread exit code. It returns always NULL (0).
*/
//------------------------------------------------------------------------------
static tThreadResult sdoUdpThread(tThreadArg pArg_p)
{
    const tSdoUdpSocketInstance*    pInstance;
    fd_set                          readFds;
    int                             result;
    struct timeval                  timeout;

    pInstance = (const tSdoUdpSocketInstance*)pArg_p;

    while (!pInstance->fStopThread)
    {
        timeout.tv_sec = 0;
        timeout.tv_usec = 400000;

        FD_ZERO(&readFds);
        FD_SET(pInstance->udpSocket, &readFds);

        result = select(pInstance->udpSocket + 1,
                        &readFds,
                        NULL,
                        NULL,
                        &timeout);
        switch (result)
        {
            case 0:     // timeout
                break;

            case -1:    // error
                DEBUG_LVL_SDO_TRACE("select error: %s\n", strerror(errno));
                break;

            default:    // data available
                receiveFromSocket(pInstance);
                break;
        }
    }

    pthread_exit(NULL);

    return 0;
}

/// \}

#endif
