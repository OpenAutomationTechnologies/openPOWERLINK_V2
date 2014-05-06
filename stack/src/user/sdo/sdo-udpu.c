/**
********************************************************************************
\file   sdo-udpu.c

\brief  Implementation of SDO over UDP protocol

This file contains the implementation of the SDO over UDP protocol.

\todo Platform specific code should be extracted into a cross-platform library
to get rid of "#ifdef" statements for different platforms!

\ingroup module_sdo_udp
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2014, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
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
#include <oplk/ami.h>
#include <user/sdoudp.h>

#if (TARGET_SYSTEM == _LINUX_)
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <pthread.h>
#endif

#include <errno.h>

#if defined(CONFIG_INCLUDE_SDO_UDP)

//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#ifndef CONFIG_SDO_MAX_CONNECTION_UDP
#define CONFIG_SDO_MAX_CONNECTION_UDP  5
#endif

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
#if (TARGET_SYSTEM == _LINUX_)
#define INVALID_SOCKET  0
#define closesocket close
#define SOCKLEN_T   socklen_t*
#endif

#if (TARGET_SYSTEM == _WIN32_)
#define SOCKLEN_T   int*
#endif

//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------
#if (TARGET_SYSTEM == _LINUX_)
typedef int SOCKET;

typedef void* tThreadResult;
typedef void* tThreadArg;

#elif (TARGET_SYSTEM == _WIN32_)

typedef DWORD tThreadResult;
typedef LPVOID tThreadArg;

#endif

typedef struct
{
    ULONG           ipAddr;     /// IP address in network byte order
    ULONG           port;       /// Port in network byte order
} tSdoUdpCon;

// instance table
typedef struct
{
    tSdoUdpCon              aSdoAbsUdpConnection[CONFIG_SDO_MAX_CONNECTION_UDP];
    tSequLayerReceiveCb     pfnSdoAsySeqCb;
    SOCKET                  udpSocket;
#if (TARGET_SYSTEM == _WIN32_)
    HANDLE                  threadHandle;
    LPCRITICAL_SECTION      pCriticalSection;
    CRITICAL_SECTION        criticalSection;
#elif (TARGET_SYSTEM == _LINUX_)
    pthread_t               threadHandle;
#endif
    BOOL                    fStopThread;
} tSdoUdpInstance;


//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tSdoUdpInstance      sdoUdpInstance_l;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static tThreadResult sdoUdpThread(tThreadArg lpParameter);

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Initialize SDO over UDP module

The function initializes the SDO over UDP module.

\param  pfnReceiveCb_p          Pointer to SDO sequence layer receive callback function.

\return The function returns a tOplkError error code.

\ingroup module_sdo_udp
*/
//------------------------------------------------------------------------------
tOplkError sdoudp_init(tSequLayerReceiveCb pfnReceiveCb_p)
{
    return sdoudp_addInstance(pfnReceiveCb_p);
}

//------------------------------------------------------------------------------
/**
\brief  Add an instance of an SDO over UDP module

The function adds an instance of an SDO over UDP module.

\param  pfnReceiveCb_p          Pointer to SDO sequence layer receive callback function.

\return The function returns a tOplkError error code.

\ingroup module_sdo_udp
*/
//------------------------------------------------------------------------------
tOplkError sdoudp_addInstance(tSequLayerReceiveCb pfnReceiveCb_p)
{
    tOplkError          ret = kErrorOk;

#if (TARGET_SYSTEM == _WIN32_)
    int                 error;
    WSADATA             wsa;
#endif

    OPLK_MEMSET(&sdoUdpInstance_l, 0x00, sizeof(sdoUdpInstance_l));

    if (pfnReceiveCb_p != NULL)
    {
        sdoUdpInstance_l.pfnSdoAsySeqCb = pfnReceiveCb_p;
    }
    else
    {
        return kErrorSdoUdpMissCb;
    }

#if (TARGET_SYSTEM == _WIN32_)
    //  windows specific start of socket
    if ((error = WSAStartup(MAKEWORD(2, 0), &wsa)) != 0)
        return kErrorSdoUdpNoSocket;

    // create critical section for access of instance variables
    sdoUdpInstance_l.pCriticalSection = &sdoUdpInstance_l.criticalSection;
    InitializeCriticalSection(sdoUdpInstance_l.pCriticalSection);
#endif

    sdoUdpInstance_l.threadHandle = 0;
    sdoUdpInstance_l.udpSocket = INVALID_SOCKET;

    ret = sdoudp_config(INADDR_ANY, 0);
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Delete an instance of an SDO over UDP module

The function deletes an instance of an SDO over UDP module. It deletes the created
sockets and deletes the listener thread.

\return The function returns a tOplkError error code.

\ingroup module_sdo_udp
*/
//------------------------------------------------------------------------------
tOplkError sdoudp_delInstance(void)
{
    tOplkError      ret = kErrorOk;

#if (TARGET_SYSTEM == _WIN32_)
    BOOL                fTermError;
#endif

    if (sdoUdpInstance_l.threadHandle != 0)
    {   // listen thread was started -> close thread
#if (TARGET_SYSTEM == _WIN32_)
        fTermError = TerminateThread(sdoUdpInstance_l.threadHandle, 0);
        if(fTermError == FALSE)
            return kErrorSdoUdpThreadError;
#elif (TARGET_SYSTEM == _LINUX_)
        sdoUdpInstance_l.fStopThread = TRUE;
        if (pthread_join(sdoUdpInstance_l.threadHandle, NULL) != 0)
            return kErrorSdoUdpThreadError;
#endif
        sdoUdpInstance_l.threadHandle = 0;
    }

    if (sdoUdpInstance_l.udpSocket != INVALID_SOCKET)
    {
        closesocket(sdoUdpInstance_l.udpSocket);
        sdoUdpInstance_l.udpSocket = INVALID_SOCKET;
    }

#if (TARGET_SYSTEM == _WIN32_)
    DeleteCriticalSection(sdoUdpInstance_l.pCriticalSection);
    WSACleanup();
#endif
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Reconfigure socket

The function reconfigures a socket with a new IP address. It is needed for
NMT_ResetConfiguration.

\param  ipAddr_p            IP address to configure.
\param  port_p              Port to configure

\return The function returns a tOplkError error code.

\ingroup module_sdo_udp
*/
//------------------------------------------------------------------------------
tOplkError sdoudp_config(ULONG ipAddr_p, UINT port_p)
{
    tOplkError          ret = kErrorOk;
    struct sockaddr_in  addr;
    INT                 error;

#if (TARGET_SYSTEM == _WIN32_)
    BOOL                fTermError;
    ULONG               threadId;
#endif

    if (port_p == 0)
    {
        port_p = C_SDO_EPL_PORT;  // set UDP port to default port number
    }
    else if (port_p > 65535)
    {
        return kErrorSdoUdpSocketError;
    }

    if (sdoUdpInstance_l.threadHandle != 0)
    {   // listen thread was started -> close old thread
#if (TARGET_SYSTEM == _WIN32_)
        fTermError = TerminateThread(sdoUdpInstance_l.threadHandle, 0);
        if (fTermError == FALSE)
            return kErrorSdoUdpThreadError;
#elif (TARGET_SYSTEM == _LINUX_)
        sdoUdpInstance_l.fStopThread = TRUE;
        if (pthread_join(sdoUdpInstance_l.threadHandle, NULL) != 0)
            return kErrorSdoUdpThreadError;
#endif
        sdoUdpInstance_l.threadHandle = 0;
    }

    if (sdoUdpInstance_l.udpSocket != INVALID_SOCKET)
    {
        error = closesocket(sdoUdpInstance_l.udpSocket);
        sdoUdpInstance_l.udpSocket = INVALID_SOCKET;
        if (error != 0)
            return kErrorSdoUdpSocketError;
    }

    // create Socket
    sdoUdpInstance_l.udpSocket = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sdoUdpInstance_l.udpSocket == INVALID_SOCKET)
    {
        ret = kErrorSdoUdpNoSocket;
        DEBUG_LVL_SDO_TRACE("sdoudp_config: socket() failed\n");
        return ret;
    }

    // bind socket
    addr.sin_family = AF_INET;
    addr.sin_port = htons((USHORT)port_p);
    addr.sin_addr.s_addr = htonl(ipAddr_p);
    error = bind(sdoUdpInstance_l.udpSocket, (struct sockaddr*)&addr, sizeof(addr));
    if (error < 0)
    {
        DEBUG_LVL_SDO_TRACE("sdoudp_config: bind() finished with %i\n", error);
        return kErrorSdoUdpNoSocket;
    }

    // create Listen-Thread
    sdoUdpInstance_l.fStopThread = FALSE;
#if (TARGET_SYSTEM == _WIN32_)
    sdoUdpInstance_l.threadHandle = CreateThread(NULL, 0, sdoUdpThread, &sdoUdpInstance_l,
                                                 0, &threadId);
    if (sdoUdpInstance_l.threadHandle == NULL)
        return kErrorSdoUdpThreadError;
#elif (TARGET_SYSTEM == _LINUX_)
    if (pthread_create(&sdoUdpInstance_l.threadHandle, NULL, sdoUdpThread, (void*)&sdoUdpInstance_l) != 0)
        return kErrorSdoUdpThreadError;

    pthread_setname_np(sdoUdpInstance_l.threadHandle, "oplk-sdoudp");
#endif

    return ret;

}

//------------------------------------------------------------------------------
/**
\brief  Initialize new connection

The function initializes a new connection.

\param  pSdoConHandle_p           Pointer for the new connection handle.
\param  targetNodeId_p            Node ID of the target.

\return The function returns a tOplkError error code.

\ingroup module_sdo_udp
*/
//------------------------------------------------------------------------------
tOplkError sdoudp_initCon(tSdoConHdl* pSdoConHandle_p, UINT targetNodeId_p)
{
    tOplkError          ret = kErrorOk;
    UINT                count;
    UINT                freeCon;
    tSdoUdpCon*         pSdoUdpCon;

    // get free entry in control structure
    count = 0;
    freeCon = CONFIG_SDO_MAX_CONNECTION_UDP;
    pSdoUdpCon = &sdoUdpInstance_l.aSdoAbsUdpConnection[0];

    while (count < CONFIG_SDO_MAX_CONNECTION_UDP)
    {
        if ((pSdoUdpCon->ipAddr & htonl(0xFF)) == htonl(targetNodeId_p))
        {   // existing connection to target node found -> set handle
            *pSdoConHandle_p = (count | SDO_UDP_HANDLE);
            return ret;
        }
        else if ((pSdoUdpCon->ipAddr == 0) && (pSdoUdpCon->port == 0))
        {
            freeCon = count;
        }

        count++;
        pSdoUdpCon++;
    }

    if (freeCon == CONFIG_SDO_MAX_CONNECTION_UDP)
    {
        ret = kErrorSdoUdpNoFreeHandle;
    }
    else
    {
        pSdoUdpCon = &sdoUdpInstance_l.aSdoAbsUdpConnection[freeCon];
        // save infos for connection
        pSdoUdpCon->port = htons(C_SDO_EPL_PORT);
        pSdoUdpCon->ipAddr = htonl(0xC0A86400 | targetNodeId_p);   // 192.168.100.uiTargetNodeId_p

        // set handle
        *pSdoConHandle_p = (freeCon | SDO_UDP_HANDLE);
    }
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Send data using existing connection

The function sends data on an existing connection.

\param  sdoConHandle_p          Connection handle to use for data transfer.
\param  pSrcData_p              Pointer to data which should be sent.
\param  dataSize_p              Size of data to send

\return The function returns a tOplkError error code.

\ingroup module_sdo_udp
*/
//------------------------------------------------------------------------------
tOplkError sdoudp_sendData(tSdoConHdl sdoConHandle_p, tPlkFrame* pSrcData_p, UINT32 dataSize_p)
{
    INT                 error;
    UINT                array;
    struct sockaddr_in  addr;

    array = (sdoConHandle_p & ~SDO_ASY_HANDLE_MASK);
    if (array >= CONFIG_SDO_MAX_CONNECTION_UDP)
        return kErrorSdoUdpInvalidHdl;

    ami_setUint8Le(&pSrcData_p->messageType, 0x06);   // set message type SDO
    ami_setUint8Le(&pSrcData_p->dstNodeId, 0x00);     // target node id (for Udp = 0)
    ami_setUint8Le(&pSrcData_p->srcNodeId, 0x00);     // set source-nodeid (for Udp = 0)
    dataSize_p += ASND_HEADER_SIZE;                   // calc size

    addr.sin_family = AF_INET;
#if (TARGET_SYSTEM == _WIN32_)
    EnterCriticalSection(sdoUdpInstance_l.pCriticalSection);
#endif
    addr.sin_port = (USHORT)sdoUdpInstance_l.aSdoAbsUdpConnection[array].port;
    addr.sin_addr.s_addr = sdoUdpInstance_l.aSdoAbsUdpConnection[array].ipAddr;

#if (TARGET_SYSTEM == _WIN32_)
    LeaveCriticalSection(sdoUdpInstance_l.pCriticalSection);
#endif

    error = sendto(sdoUdpInstance_l.udpSocket, (const char*)&pSrcData_p->messageType,
                   dataSize_p, 0, (struct sockaddr*)&addr, sizeof(struct sockaddr_in));
    if (error < 0)
    {
        DEBUG_LVL_SDO_TRACE("sdoudp_sendData: sendto() finished with %i\n", error);
        return kErrorSdoUdpSendError;
    }
    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Delete connection

The function deletes an existing connection.

\param  sdoConHandle_p          Connection handle to use for data transfer.

\return The function returns a tOplkError error code.

\ingroup module_sdo_udp
*/
//------------------------------------------------------------------------------
tOplkError sdoudp_delConnection(tSdoConHdl sdoConHandle_p)
{
    tOplkError      ret = kErrorOk;
    UINT            array;

    array = (sdoConHandle_p & ~SDO_ASY_HANDLE_MASK);
    if (array >= CONFIG_SDO_MAX_CONNECTION_UDP)
    {
        return kErrorSdoUdpInvalidHdl;
    }
    // delete connection
    sdoUdpInstance_l.aSdoAbsUdpConnection[array].ipAddr = 0;
    sdoUdpInstance_l.aSdoAbsUdpConnection[array].port = 0;

    return ret;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief  receive data from socket

The function receives data from the UDP socket.

\param  pInstance_p           Pointer to SDO instance.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
void receiveFromSocket(tSdoUdpInstance* pInstance_p)
{
    tOplkError          ret;
    struct sockaddr_in  remoteAddr;
    INT                 error;
    INT                 count;
    INT                 freeEntry;
    UINT8               aBuffer[SDO_MAX_REC_FRAME_SIZE];
    UINT                size;
    tSdoConHdl          sdoConHdl;

    size = sizeof(struct sockaddr);

    error = recvfrom(pInstance_p->udpSocket, (char*)&aBuffer[0], sizeof(aBuffer),
                     0, (struct sockaddr*)&remoteAddr, (SOCKLEN_T)&size);
    if (error > 0)
    {
        // get handle for higher layer
        count = 0;
        freeEntry = 0xFFFF;
#if (TARGET_SYSTEM == _WIN32_)
        EnterCriticalSection(sdoUdpInstance_l.pCriticalSection);
#endif
        while (count < CONFIG_SDO_MAX_CONNECTION_UDP)
        {
            // check if this connection is already known
            if ((pInstance_p->aSdoAbsUdpConnection[count].ipAddr == remoteAddr.sin_addr.s_addr) &&
                (pInstance_p->aSdoAbsUdpConnection[count].port == remoteAddr.sin_port))
            {
                break;
            }

            if ((pInstance_p->aSdoAbsUdpConnection[count].ipAddr == 0) &&
                (pInstance_p->aSdoAbsUdpConnection[count].port == 0) &&
                (freeEntry == 0xFFFF))
            {
                freeEntry = count;
            }
            count++;
        }

        if (count == CONFIG_SDO_MAX_CONNECTION_UDP)
        {
            // connection unknown -> see if there is a free handle
            if (freeEntry != 0xFFFF)
            {
                // save address infos
                pInstance_p->aSdoAbsUdpConnection[freeEntry].ipAddr = remoteAddr.sin_addr.s_addr;
                pInstance_p->aSdoAbsUdpConnection[freeEntry].port = remoteAddr.sin_port;
#if (TARGET_SYSTEM == _WIN32_)
                LeaveCriticalSection(sdoUdpInstance_l.pCriticalSection);
#endif
                // call callback
                sdoConHdl = freeEntry;
                sdoConHdl |= SDO_UDP_HANDLE;

                // offset 4 -> start of SDO Sequence header
                ret = pInstance_p->pfnSdoAsySeqCb(sdoConHdl, (tAsySdoSeq*)&aBuffer[4], (error - 4));
                if (ret != kErrorOk)
                {
                    DEBUG_LVL_ERROR_TRACE("%s new con: ip=%lX, port=%u, Ret=0x%X\n", __func__,
                          (ULONG)ntohl(pInstance_p->aSdoAbsUdpConnection[freeEntry].ipAddr),
                          ntohs((USHORT)pInstance_p->aSdoAbsUdpConnection[freeEntry].port), ret);
                }
            }
            else
            {
                DEBUG_LVL_ERROR_TRACE("Error in sdo-udpu: receiveFromSocket(): no free handle\n");
#if (TARGET_SYSTEM == _WIN32_)
                LeaveCriticalSection(sdoUdpInstance_l.pCriticalSection);
#endif
            }
        }
        else
        {
            // known connection -> call callback with correct handle
            sdoConHdl = count;
            sdoConHdl |= SDO_UDP_HANDLE;
#if (TARGET_SYSTEM == _WIN32_)
            LeaveCriticalSection(sdoUdpInstance_l.pCriticalSection);
#endif
            // offset 4 -> start of SDO Sequence header
            ret = pInstance_p->pfnSdoAsySeqCb(sdoConHdl, (tAsySdoSeq*)&aBuffer[4], (error - 4));
            if (ret != kErrorOk)
            {
                DEBUG_LVL_ERROR_TRACE("%s known con: ip=%lX, port=%u, Ret=0x%X\n", __func__,
                      (ULONG)ntohl(pInstance_p->aSdoAbsUdpConnection[count].ipAddr),
                      ntohs((USHORT)pInstance_p->aSdoAbsUdpConnection[count].port), ret);
            }
        }
    }
}

//------------------------------------------------------------------------------
/**
\brief  UDP Receiving thread function

The function implements the UDP receive thread. It waits for packets on the
UDP socket and calls receiveFromSocket() if data is available.


\param  pArg_p          Thread argument. The pointer to the SDO instance is
                        transfered to the thread as thread argument.

\return The function returns a thread exit code. It returns always NULL (0).
*/
//------------------------------------------------------------------------------
static tThreadResult sdoUdpThread(tThreadArg pArg_p)
{
    tSdoUdpInstance*    pInstance;
    fd_set              readFds;
    int                 result;
    struct timeval      timeout;

    pInstance = (tSdoUdpInstance*)pArg_p;

    while (!pInstance->fStopThread)
    {
        timeout.tv_sec = 0;
        timeout.tv_usec = 400000;

        FD_ZERO(&readFds);
        FD_SET(pInstance->udpSocket, &readFds);

        result = select(pInstance->udpSocket + 1, &readFds, NULL, NULL, &timeout);
        switch (result)
        {
            case 0:     // timeout
                //DEBUG_LVL_SDO_TRACE ("select timeout\n");
                break;

            case -1:    // error
                DEBUG_LVL_SDO_TRACE ("select error: %s\n", strerror(errno));
                break;

            default:    // data available
                receiveFromSocket(pInstance);
                break;
        }
    }

#if (TARGET_SYSTEM == _LINUX_)
    pthread_exit(NULL);
#endif
    return 0;
}

///\}

#endif

