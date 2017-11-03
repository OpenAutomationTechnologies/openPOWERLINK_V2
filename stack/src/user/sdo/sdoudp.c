/**
********************************************************************************
\file   sdoudp.c

\brief  Implementation of SDO over UDP protocol

This file contains the implementation of the SDO over UDP protocol.

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
#include <common/ami.h>

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

#if (TARGET_SYSTEM == _LINUX_)
#include <arpa/inet.h>
#else
#if CHECK_IF_BIG_ENDIAN()
// Big endian => no swap needed
#define htons(x)        (x)
#define htonl(x)        (x)
#define ntohs(x)        (x)
#define ntohl(x)        (x)
#else
// Little endian => swap needed
// Swap long: 0x00C0FFEE --> 0xEEFFC000
#define UPDSDO_SWAPL(x) ((((x) >> 24) & 0x000000FF) | (((x) >> 8) & 0x0000FF00) | \
                        (((x) & 0x000000FF) << 24) | (((x) & 0x0000FF00) << 8))

// Swap short: 0xC0FE --> 0xFEC0
#define UDPSDO_SWAPS(x) ((((x) >> 8) & 0x00FF) | (((x) << 8) & 0xFF00))

#define htons(x)        UDPSDO_SWAPS(x)
#define htonl(x)        UPDSDO_SWAPL(x)
#define ntohs(x)        UDPSDO_SWAPS(x)
#define ntohl(x)        UPDSDO_SWAPL(x)
#endif //CHECK_IF_BIG_ENDIAN
#endif //DEV_SYSTEM

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

// instance table
typedef struct
{
    tSdoUdpCon              aSdoUdpConnection[CONFIG_SDO_MAX_CONNECTION_UDP];
    tSequLayerReceiveCb     pfnSdoAsySeqCb;
} tSdoUdpInstance;


//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tSdoUdpInstance      sdoUdpInstance_l;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Initialize SDO over UDP module

The function initializes the SDO over UDP module.

\param[in]      pfnReceiveCb_p      Pointer to SDO sequence layer receive callback function.

\return The function returns a tOplkError error code.

\ingroup module_sdo_udp
*/
//------------------------------------------------------------------------------
tOplkError sdoudp_init(tSequLayerReceiveCb pfnReceiveCb_p)
{
    tOplkError  ret;

    OPLK_MEMSET(&sdoUdpInstance_l, 0x00, sizeof(sdoUdpInstance_l));

    if (pfnReceiveCb_p != NULL)
        sdoUdpInstance_l.pfnSdoAsySeqCb = pfnReceiveCb_p;
    else
        return kErrorSdoUdpMissCb;

    ret = sdoudp_initSocket();
    if (ret != kErrorOk)
        return ret;

    ret = sdoudp_config(SDOUDP_INADDR_ANY, 0);

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Shut down the SDO over UDP module

The function shuts down the SDO over UDP module. It deletes the created
sockets and deletes the listener thread.

\return The function returns a tOplkError error code.

\ingroup module_sdo_udp
*/
//------------------------------------------------------------------------------
tOplkError sdoudp_exit(void)
{
    tOplkError  ret;

    ret = sdoudp_closeSocket();
    sdoudp_exitSocket();

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Reconfigure socket

The function reconfigures a socket with a new IP address. It is needed for
NMT_ResetConfiguration.

\param[in]      ipAddr_p            IP address to configure.
\param[in]      port_p              Port to configure

\return The function returns a tOplkError error code.

\ingroup module_sdo_udp
*/
//------------------------------------------------------------------------------
tOplkError sdoudp_config(UINT32 ipAddr_p, UINT16 port_p)
{
    tOplkError  ret;
    tSdoUdpCon  udpConnection;

    if (port_p == 0)
        port_p = C_SDO_EPL_PORT;            // set UDP port to default port number

    ret = sdoudp_closeSocket();
    if (ret != kErrorOk)
        return ret;

    udpConnection.ipAddr = ipAddr_p;
    udpConnection.port = port_p;
    ret = sdoudp_createSocket(&udpConnection);

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Initialize new connection

The function initializes a new connection.

\param[out]     pSdoConHandle_p     Pointer for the new connection handle.
\param[in]      targetNodeId_p      Node ID of the target.

\return The function returns a tOplkError error code.

\ingroup module_sdo_udp
*/
//------------------------------------------------------------------------------
tOplkError sdoudp_initCon(tSdoConHdl* pSdoConHandle_p, UINT targetNodeId_p)
{
    tOplkError  ret = kErrorOk;
    UINT        count;
    UINT        freeCon;
    tSdoUdpCon* pSdoUdpCon;

    // Check parameter validity
    ASSERT(pSdoConHandle_p != NULL);

    // get free entry in control structure
    count = 0;
    freeCon = CONFIG_SDO_MAX_CONNECTION_UDP;
    pSdoUdpCon = &sdoUdpInstance_l.aSdoUdpConnection[0];

    while (count < CONFIG_SDO_MAX_CONNECTION_UDP)
    {
        if ((pSdoUdpCon->ipAddr & htonl(0x000000FFUL)) == htonl(targetNodeId_p))
        {   // existing connection to target node found -> set handle
            *pSdoConHandle_p = (tSdoConHdl)(count | SDO_UDP_HANDLE);
            return ret;
        }
        else if ((pSdoUdpCon->ipAddr == 0) && (pSdoUdpCon->port == 0))
            freeCon = count;

        count++;
        pSdoUdpCon++;
    }

    if (freeCon == CONFIG_SDO_MAX_CONNECTION_UDP)
        ret = kErrorSdoUdpNoFreeHandle;
    else
    {
        pSdoUdpCon = &sdoUdpInstance_l.aSdoUdpConnection[freeCon];
        // save infos for connection
        pSdoUdpCon->port = htons(C_SDO_EPL_PORT);
        pSdoUdpCon->ipAddr = htonl(0xC0A86400 | targetNodeId_p);    // 192.168.100.targetNodeId_p

        ret = sdoudp_arpQuery(pSdoUdpCon->ipAddr);
        if (ret != kErrorOk)
        {
            // Reset connection handle
            pSdoUdpCon->port = 0;
            pSdoUdpCon->ipAddr = 0;

            return ret;
        }

        // set handle
        *pSdoConHandle_p = (tSdoConHdl)(freeCon | SDO_UDP_HANDLE);
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Send data using existing connection

The function sends data on an existing connection.

\param[in]      sdoConHandle_p      Connection handle to use for data transfer.
\param[in,out]  pSrcData_p          Pointer to data which should be sent.
\param[in]      dataSize_p          Size of data to send

\return The function returns a tOplkError error code.

\ingroup module_sdo_udp
*/
//------------------------------------------------------------------------------
tOplkError sdoudp_sendData(tSdoConHdl sdoConHandle_p,
                           tPlkFrame* pSrcData_p,
                           size_t dataSize_p)
{
    tOplkError  ret;
    UINT        sdoUdpConSel;
    tSdoUdpCon  sdoUdpCon;

    sdoUdpConSel = ((UINT)sdoConHandle_p & ~SDO_ASY_HANDLE_MASK);
    if (sdoUdpConSel >= CONFIG_SDO_MAX_CONNECTION_UDP)
        return kErrorSdoUdpInvalidHdl;

    ami_setUint8Le(&pSrcData_p->messageType, 0x06);     // set message type SDO
    ami_setUint8Le(&pSrcData_p->dstNodeId, 0x00);       // target node id (for Udp = 0)
    ami_setUint8Le(&pSrcData_p->srcNodeId, 0x00);       // set source-nodeid (for Udp = 0)
    dataSize_p += ASND_HEADER_SIZE;                     // calc size

    sdoudp_criticalSection(TRUE);
    sdoUdpCon.port = sdoUdpInstance_l.aSdoUdpConnection[sdoUdpConSel].port;
    sdoUdpCon.ipAddr = sdoUdpInstance_l.aSdoUdpConnection[sdoUdpConSel].ipAddr;
    sdoudp_criticalSection(FALSE);

    ret = sdoudp_sendToSocket(&sdoUdpCon, pSrcData_p, dataSize_p);

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Receive data from socket

The function receives data from the UDP socket.

\param[in]      pSdoUdpCon_p        Pointer to remote SDO over UDP connection.
\param[in,out]  pSdoSeqData_p       Pointer to SDO sequence data.
\param[in]      dataSize_p          Size of SDO sequence data.

\ingroup module_sdo_udp
*/
//------------------------------------------------------------------------------
void sdoudp_receiveData(const tSdoUdpCon* pSdoUdpCon_p,
                        const tAsySdoSeq* pSdoSeqData_p,
                        size_t dataSize_p)
{
    tOplkError  ret;
    UINT        count;
    UINT        freeEntry;
    tSdoConHdl  sdoConHdl;

    // get handle for higher layer
    count = 0;
    freeEntry = 0xFFFF;

    sdoudp_criticalSection(TRUE);

    while (count < CONFIG_SDO_MAX_CONNECTION_UDP)
    {
        // check if this connection is already known
        if ((sdoUdpInstance_l.aSdoUdpConnection[count].ipAddr == pSdoUdpCon_p->ipAddr) &&
            (sdoUdpInstance_l.aSdoUdpConnection[count].port == pSdoUdpCon_p->port))
        {
            break;
        }

        if ((sdoUdpInstance_l.aSdoUdpConnection[count].ipAddr == 0) &&
            (sdoUdpInstance_l.aSdoUdpConnection[count].port == 0) &&
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
            sdoUdpInstance_l.aSdoUdpConnection[freeEntry].ipAddr = pSdoUdpCon_p->ipAddr;
            sdoUdpInstance_l.aSdoUdpConnection[freeEntry].port = pSdoUdpCon_p->port;

            sdoudp_criticalSection(FALSE);

            // call callback
            sdoConHdl = (tSdoConHdl)(freeEntry | SDO_UDP_HANDLE);

            // offset 4 (ASnd header) -> start of SDO Sequence header
            ret = sdoUdpInstance_l.pfnSdoAsySeqCb(sdoConHdl, pSdoSeqData_p, dataSize_p);
            if (ret != kErrorOk)
            {
                DEBUG_LVL_ERROR_TRACE("%s new con: ip=%lX, port=%u, ret=0x%X\n",
                                      __func__,
                                      (UINT32)ntohl(sdoUdpInstance_l.aSdoUdpConnection[freeEntry].ipAddr),
                                      ntohs(sdoUdpInstance_l.aSdoUdpConnection[freeEntry].port),
                                      ret);
            }
        }
        else
        {
            DEBUG_LVL_ERROR_TRACE("Error in sdoudp: %s(): no free handle\n", __func__);
            sdoudp_criticalSection(FALSE);
        }
    }
    else
    {
        // known connection -> call callback with correct handle
        sdoConHdl = (tSdoConHdl)(count | SDO_UDP_HANDLE);

        sdoudp_criticalSection(FALSE);

        // offset 4 (ASnd header) -> start of SDO Sequence header
        ret = sdoUdpInstance_l.pfnSdoAsySeqCb(sdoConHdl, pSdoSeqData_p, dataSize_p);
        if (ret != kErrorOk)
        {
            DEBUG_LVL_ERROR_TRACE("%s known con: ip=%lX, port=%u, ret=0x%X\n",
                                  __func__,
                                  (UINT32)ntohl(sdoUdpInstance_l.aSdoUdpConnection[count].ipAddr),
                                  ntohs(sdoUdpInstance_l.aSdoUdpConnection[count].port),
                                  ret);
        }
    }
}

//------------------------------------------------------------------------------
/**
\brief  Delete connection

The function deletes an existing connection.

\param[in]      sdoConHandle_p      Connection handle to use for data transfer.

\return The function returns a tOplkError error code.

\ingroup module_sdo_udp
*/
//------------------------------------------------------------------------------
tOplkError sdoudp_delConnection(tSdoConHdl sdoConHandle_p)
{
    tOplkError  ret = kErrorOk;
    UINT        array;

    array = ((UINT)sdoConHandle_p & ~SDO_ASY_HANDLE_MASK);
    if (array >= CONFIG_SDO_MAX_CONNECTION_UDP)
        return kErrorSdoUdpInvalidHdl;

    // delete connection
    sdoUdpInstance_l.aSdoUdpConnection[array].ipAddr = 0;
    sdoUdpInstance_l.aSdoUdpConnection[array].port = 0;

    return ret;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

/// \}

#endif
