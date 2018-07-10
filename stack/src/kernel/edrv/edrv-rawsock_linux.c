/**
********************************************************************************
\file   edrv-rawsock_linux.c

\brief  Implementation of Linux raw socket Ethernet driver

This file contains the implementation of the Linux raw socket Ethernet driver.

\ingroup module_edrv
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2017, BE.services GmbH
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
#include <common/ftracedebug.h>
#include <kernel/edrv.h>

#include <unistd.h>
#include <string.h>
#include <semaphore.h>
#include <pthread.h>
#include <sys/select.h>
#include <sys/syscall.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <netinet/in.h>
#include <net/if.h>
#include <errno.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <ifaddrs.h>
#include <linux/if_packet.h>
#include <sys/types.h>

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
#define EDRV_MAX_FRAME_SIZE     0x0600
#define PROTO_PLK               0x88AB
#ifndef PACKET_QDISC_BYPASS
#define PACKET_QDISC_BYPASS     20
#endif
//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------
/**
\brief Structure describing an instance of the Edrv

This structure describes an instance of the Ethernet driver.
*/
typedef struct
{
    tEdrvInitParam      initParam;                       ///< Init parameters
    tEdrvTxBuffer*      pTransmittedTxBufferLastEntry;   ///< Pointer to the last entry of the transmitted TX buffer
    tEdrvTxBuffer*      pTransmittedTxBufferFirstEntry;  ///< Pointer to the first entry of the transmitted Tx buffer
    pthread_mutex_t     mutex;                           ///< Mutex for locking of critical sections
    sem_t               syncSem;                         ///< Semaphore for signaling the start of the worker thread
    int                 sock;                            ///< Raw socket handle
    pthread_t           hThread;                         ///< Handle of the worker thread
    BOOL                fStartCommunication;             ///< Flag to indicate, that communication is started. Set to false on exit
    BOOL                fThreadIsExited;                 ///< Set by thread if already exited
} tEdrvInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tEdrvInstance edrvInstance_l;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static void     packetHandler(void* pParam_p,
                              const int frameSize_p,
                              void* pPktData_p);
static void*    workerThread(void* pArgument_p);
static void     getMacAdrs(const char* pIfName_p, UINT8* pMacAddr_p);
static BOOL     getLinkStatus(const char* pIfName_p);

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Ethernet driver initialization

This function initializes the Ethernet driver.

\param[in]      pEdrvInitParam_p    Edrv initialization parameters

\return The function returns a tOplkError error code.

\ingroup module_edrv
*/
//------------------------------------------------------------------------------
tOplkError edrv_init(const tEdrvInitParam* pEdrvInitParam_p)
{
    struct sched_param  schedParam;
    int                 result = 0;
    int                 sock_qdisc_bypass = 1;
    struct sockaddr_ll  sock_addr;
    struct ifreq        ifr;
    int                 blockingMode = 0;

    // Check parameter validity
    ASSERT(pEdrvInitParam_p != NULL);

    // Clear instance structure
    OPLK_MEMSET(&edrvInstance_l, 0, sizeof(edrvInstance_l));

    if (pEdrvInitParam_p->pDevName == NULL)
        return kErrorEdrvInit;

    // Save the init data
    edrvInstance_l.initParam = *pEdrvInitParam_p;

    edrvInstance_l.fStartCommunication = TRUE;
    edrvInstance_l.fThreadIsExited = FALSE;

    // If no MAC address was specified read MAC address of used
    // Ethernet interface
    if ((edrvInstance_l.initParam.aMacAddr[0] == 0) &&
        (edrvInstance_l.initParam.aMacAddr[1] == 0) &&
        (edrvInstance_l.initParam.aMacAddr[2] == 0) &&
        (edrvInstance_l.initParam.aMacAddr[3] == 0) &&
        (edrvInstance_l.initParam.aMacAddr[4] == 0) &&
        (edrvInstance_l.initParam.aMacAddr[5] == 0))
    {   // read MAC address from controller
        getMacAdrs(edrvInstance_l.initParam.pDevName,
                   edrvInstance_l.initParam.aMacAddr);
    }
    if (pthread_mutex_init(&edrvInstance_l.mutex, NULL) != 0)
    {
        DEBUG_LVL_ERROR_TRACE("%s() couldn't init mutex\n", __func__);
        return kErrorEdrvInit;
    }

    edrvInstance_l.sock = socket(PF_PACKET, SOCK_RAW, htons(PROTO_PLK));
    if (edrvInstance_l.sock < 0)
    {
        DEBUG_LVL_ERROR_TRACE("%s() cannot open socket. Error = %s\n", __func__, strerror(errno));
        return kErrorEdrvInit;
    }

    if (ioctl(edrvInstance_l.sock, FIONBIO, &blockingMode) != 0)
    {
        result = -1;
        DEBUG_LVL_ERROR_TRACE("%s() ioctl(FIONBIO) fails. Error = %s\n", __func__, strerror(errno));
    }

    // Set option PACKET_QDISC_BYPASS. It allows to transmit a frame faster through network stack. Available since linux 3.14
    if (setsockopt(edrvInstance_l.sock, SOL_PACKET, PACKET_QDISC_BYPASS, &sock_qdisc_bypass, sizeof(sock_qdisc_bypass)) != 0)
    {
        DEBUG_LVL_ERROR_TRACE("Couldn't set PACKET_QDISC_BYPASS socket option. Error = %s\n", __func__, strerror(errno));
    }
    else
    {
        DEBUG_LVL_EDRV_TRACE("Kernel qdisc bypass is enabled\n");
    }

    OPLK_MEMSET(&ifr, 0, sizeof(struct ifreq));
    strncpy(ifr.ifr_name, edrvInstance_l.initParam.pDevName, IFNAMSIZ - 1);

    if (ioctl(edrvInstance_l.sock, SIOCGIFFLAGS, &ifr) < 0)
    {
        result = -1;
        DEBUG_LVL_ERROR_TRACE("%s() ioctl(SIOCGIFFLAGS) fails. Error = %s\n", __func__, strerror(errno));
    }

    ifr.ifr_flags = ifr.ifr_flags | IFF_PROMISC;
    if (ioctl(edrvInstance_l.sock, SIOCSIFFLAGS, &ifr))
    {
        result = -1;
        DEBUG_LVL_ERROR_TRACE("%s() ioctl(SIOCSIFFLAGS) with IFF_PROMISC fails. Error = %s\n", __func__, strerror(errno));
    }

    if (ioctl(edrvInstance_l.sock, SIOCGIFINDEX, &ifr) != 0)
    {
        result = -1;
        DEBUG_LVL_ERROR_TRACE("%s() ioctl(SIOCGIFINDEX) fails. Error = %s\n", __func__, strerror(errno));
    }
    sock_addr.sll_ifindex = ifr.ifr_ifindex;


    sock_addr.sll_family = AF_PACKET;
    sock_addr.sll_protocol = htons(PROTO_PLK);
    if (bind(edrvInstance_l.sock, (struct sockaddr*)&sock_addr, sizeof(sock_addr)) != 0)
    {
        result = -1;
        DEBUG_LVL_ERROR_TRACE("%s() bind fails. Error = %s\n", __func__, strerror(errno));
    }
    if (result < 0)
    {
        close(edrvInstance_l.sock);
        DEBUG_LVL_ERROR_TRACE("%s() Couldn't init ethernet adapter:%s", __func__, ifr.ifr_name);
        return kErrorEdrvInit;
    }

    if (sem_init(&edrvInstance_l.syncSem, 0, 0) != 0)
    {
        DEBUG_LVL_ERROR_TRACE("%s() couldn't init semaphore\n", __func__);
        return kErrorEdrvInit;
    }

    if (pthread_create(&edrvInstance_l.hThread, NULL,
                       workerThread, &edrvInstance_l) != 0)
    {
        DEBUG_LVL_ERROR_TRACE("%s() Couldn't create worker thread!\n", __func__);
        return kErrorEdrvInit;
    }

    schedParam.sched_priority = CONFIG_THREAD_PRIORITY_MEDIUM;
    if (pthread_setschedparam(edrvInstance_l.hThread, SCHED_FIFO, &schedParam) != 0)
    {
        DEBUG_LVL_ERROR_TRACE("%s() couldn't set thread scheduling parameters!\n", __func__);
    }

#if (defined(__GLIBC__) && (__GLIBC__ >= 2) && (__GLIBC_MINOR__ >= 12))
    pthread_setname_np(edrvInstance_l.hThread, "oplk-edrvrawsock");
#endif

    // wait until thread is started
    sem_wait(&edrvInstance_l.syncSem);

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Shut down Ethernet driver

This function shuts down the Ethernet driver.

\return The function returns a tOplkError error code.

\ingroup module_edrv
*/
//------------------------------------------------------------------------------
tOplkError edrv_exit(void)
{
    edrvInstance_l.fStartCommunication = FALSE;

    // Wait to terminate thread safely
    usleep(100000);

    if (edrvInstance_l.fThreadIsExited)
        pthread_cancel(edrvInstance_l.hThread);

    pthread_mutex_destroy(&edrvInstance_l.mutex);

    // Close the socket
    close(edrvInstance_l.sock);

    // Clear instance structure
    OPLK_MEMSET(&edrvInstance_l, 0, sizeof(edrvInstance_l));

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Get MAC address

This function returns the MAC address of the Ethernet controller

\return The function returns a pointer to the MAC address.

\ingroup module_edrv
*/
//------------------------------------------------------------------------------
const UINT8* edrv_getMacAddr(void)
{
    return edrvInstance_l.initParam.aMacAddr;
}

//------------------------------------------------------------------------------
/**
\brief  Send Tx buffer

This function sends the Tx buffer.

\param[in,out]  pBuffer_p           Tx buffer descriptor

\return The function returns a tOplkError error code.

\ingroup module_edrv
*/
//------------------------------------------------------------------------------
tOplkError edrv_sendTxBuffer(tEdrvTxBuffer* pBuffer_p)
{
    int    sockRet;

    // Check parameter validity
    ASSERT(pBuffer_p != NULL);

    FTRACE_MARKER("%s", __func__);

    if (pBuffer_p->txBufferNumber.pArg != NULL)
        return kErrorInvalidOperation;

    if (getLinkStatus(edrvInstance_l.initParam.pDevName) == FALSE)
    {
        /* If there is no link, we pretend that the packet is sent and immediately call
         * tx handler. Otherwise the stack would hang! */
        if (pBuffer_p->pfnTxHandler != NULL)
        {
            pBuffer_p->pfnTxHandler(pBuffer_p);
        }
    }
    else
    {
        pthread_mutex_lock(&edrvInstance_l.mutex);
        if (edrvInstance_l.pTransmittedTxBufferLastEntry == NULL)
        {
            edrvInstance_l.pTransmittedTxBufferLastEntry = pBuffer_p;
            edrvInstance_l.pTransmittedTxBufferFirstEntry = pBuffer_p;
        }
        else
        {
            edrvInstance_l.pTransmittedTxBufferLastEntry->txBufferNumber.pArg = pBuffer_p;
            edrvInstance_l.pTransmittedTxBufferLastEntry = pBuffer_p;
        }
        pthread_mutex_unlock(&edrvInstance_l.mutex);

        sockRet = send(edrvInstance_l.sock, (u_char*)pBuffer_p->pBuffer, (int)pBuffer_p->txFrameSize, 0);
        if (sockRet < 0)
        {
            DEBUG_LVL_EDRV_TRACE("%s() send() returned %d\n", __func__, sockRet);
            return kErrorInvalidOperation;
        }
        else
        {
            packetHandler((u_char*)&edrvInstance_l, sockRet, pBuffer_p->pBuffer);
        }
    }

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Allocate Tx buffer

This function allocates a Tx buffer.

\param[in,out]  pBuffer_p           Tx buffer descriptor

\return The function returns a tOplkError error code.

\ingroup module_edrv
*/
//------------------------------------------------------------------------------
tOplkError edrv_allocTxBuffer(tEdrvTxBuffer* pBuffer_p)
{
    // Check parameter validity
    ASSERT(pBuffer_p != NULL);

    if (pBuffer_p->maxBufferSize > EDRV_MAX_FRAME_SIZE)
        return kErrorEdrvNoFreeBufEntry;

    // allocate buffer with malloc
    pBuffer_p->pBuffer = OPLK_MALLOC(pBuffer_p->maxBufferSize);
    if (pBuffer_p->pBuffer == NULL)
        return kErrorEdrvNoFreeBufEntry;

    pBuffer_p->txBufferNumber.pArg = NULL;

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Free Tx buffer

This function releases the Tx buffer.

\param[in,out]  pBuffer_p           Tx buffer descriptor

\return The function returns a tOplkError error code.

\ingroup module_edrv
*/
//------------------------------------------------------------------------------
tOplkError edrv_freeTxBuffer(tEdrvTxBuffer* pBuffer_p)
{
    void*   pBuffer;

    // Check parameter validity
    ASSERT(pBuffer_p != NULL);

    pBuffer = pBuffer_p->pBuffer;

    // mark buffer as free, before actually freeing it
    pBuffer_p->pBuffer = NULL;

    OPLK_FREE(pBuffer);

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Change Rx filter setup

This function changes the Rx filter setup. The parameter entryChanged_p
selects the Rx filter entry that shall be changed and \p changeFlags_p determines
the property.
If \p entryChanged_p is equal or larger count_p all Rx filters shall be changed.

\note Rx filters are not supported by this driver!

\param[in,out]  pFilter_p           Base pointer of Rx filter array
\param[in]      count_p             Number of Rx filter array entries
\param[in]      entryChanged_p      Index of Rx filter entry that shall be changed
\param[in]      changeFlags_p       Bit mask that selects the changing Rx filter property

\return The function returns a tOplkError error code.

\ingroup module_edrv
*/
//------------------------------------------------------------------------------
tOplkError edrv_changeRxFilter(tEdrvFilter* pFilter_p,
                               UINT count_p,
                               UINT entryChanged_p,
                               UINT changeFlags_p)
{
    UNUSED_PARAMETER(pFilter_p);
    UNUSED_PARAMETER(count_p);
    UNUSED_PARAMETER(entryChanged_p);
    UNUSED_PARAMETER(changeFlags_p);

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Clear multicast address entry

This function removes the multicast entry from the Ethernet controller.

\note The multicast filters are not supported by this driver.

\param[in]      pMacAddr_p          Multicast address

\return The function returns a tOplkError error code.

\ingroup module_edrv
*/
//------------------------------------------------------------------------------
tOplkError edrv_clearRxMulticastMacAddr(const UINT8* pMacAddr_p)
{
    UNUSED_PARAMETER(pMacAddr_p);

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Set multicast address entry

This function sets a multicast entry into the Ethernet controller.

\note The multicast filters are not supported by this driver.

\param[in]      pMacAddr_p          Multicast address.

\return The function returns a tOplkError error code.

\ingroup module_edrv
*/
//------------------------------------------------------------------------------
tOplkError edrv_setRxMulticastMacAddr(const UINT8* pMacAddr_p)
{
    UNUSED_PARAMETER(pMacAddr_p);

    return kErrorOk;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief  Edrv packet handler

This function is the packet handler forwarding the frames to the dllk.

\param[in,out]  pParam_p            User specific pointer pointing to the instance structure
\param[in]      frameSize_p         Framesize information
\param[in]      pPktData_p          Packet buffer
*/
//------------------------------------------------------------------------------
static void packetHandler(void* pParam_p,
                          const int frameSize_p,
                          void* pPktData_p)
{
    tEdrvInstance*  pInstance = (tEdrvInstance*)pParam_p;
    tEdrvRxBuffer   rxBuffer;

    if (OPLK_MEMCMP((UINT8*)pPktData_p + 6, pInstance->initParam.aMacAddr, 6) != 0)
    {   // filter out self generated traffic
        rxBuffer.bufferInFrame = kEdrvBufferLastInFrame;
        rxBuffer.rxFrameSize = frameSize_p;
        rxBuffer.pBuffer = pPktData_p;

        FTRACE_MARKER("%s RX", __func__);
        pInstance->initParam.pfnRxHandler(&rxBuffer);
    }
    else
    {   // self generated traffic
        FTRACE_MARKER("%s TX-receive", __func__);

        if (pInstance->pTransmittedTxBufferFirstEntry != NULL)
        {
            tEdrvTxBuffer* pTxBuffer = pInstance->pTransmittedTxBufferFirstEntry;

            if (pTxBuffer->pBuffer != NULL)
            {
                if (OPLK_MEMCMP(pPktData_p, pTxBuffer->pBuffer, 6) == 0)
                {   // compare with packet buffer with destination MAC
                    pthread_mutex_lock(&pInstance->mutex);
                    pInstance->pTransmittedTxBufferFirstEntry = (tEdrvTxBuffer*)pInstance->pTransmittedTxBufferFirstEntry->txBufferNumber.pArg;
                    if (pInstance->pTransmittedTxBufferFirstEntry == NULL)
                    {
                        pInstance->pTransmittedTxBufferLastEntry = NULL;
                    }
                    pthread_mutex_unlock(&pInstance->mutex);

                    pTxBuffer->txBufferNumber.pArg = NULL;

                    if (pTxBuffer->pfnTxHandler != NULL)
                    {
                        pTxBuffer->pfnTxHandler(pTxBuffer);
                    }
                }
                else
                {
                    TRACE("%s: no matching TxB: DstMAC=%02X%02X%02X%02X%02X%02X\n",
                          __func__,
                          (UINT)((UINT8*)pPktData_p)[0],
                          (UINT)((UINT8*)pPktData_p)[1],
                          (UINT)((UINT8*)pPktData_p)[2],
                          (UINT)((UINT8*)pPktData_p)[3],
                          (UINT)((UINT8*)pPktData_p)[4],
                          (UINT)((UINT8*)pPktData_p)[5]);
                    TRACE("   current TxB %p: DstMAC=%02X%02X%02X%02X%02X%02X\n",
                          (void*)pTxBuffer,
                          (UINT)((UINT8*)(pTxBuffer->pBuffer))[0],
                          (UINT)((UINT8*)(pTxBuffer->pBuffer))[1],
                          (UINT)((UINT8*)(pTxBuffer->pBuffer))[2],
                          (UINT)((UINT8*)(pTxBuffer->pBuffer))[3],
                          (UINT)((UINT8*)(pTxBuffer->pBuffer))[4],
                          (UINT)((UINT8*)(pTxBuffer->pBuffer))[5]);
                }
            }
        }
        else
        {
            TRACE("%s: no TxB: DstMAC=%02X%02X%02X%02X%02X%02X\n", __func__,
                  ((UINT8*)pPktData_p)[0],
                  ((UINT8*)pPktData_p)[1],
                  ((UINT8*)pPktData_p)[2],
                  ((UINT8*)pPktData_p)[3],
                  ((UINT8*)pPktData_p)[4],
                  ((UINT8*)pPktData_p)[5]);
        }
    }
}

//------------------------------------------------------------------------------
/**
\brief  Edrv worker thread

This function implements the edrv worker thread. It is responsible to receive frames

\param[in,out]  pArgument_p         User specific pointer pointing to the instance structure

\return The function returns a thread error code.
*/
//------------------------------------------------------------------------------
static void* workerThread(void* pArgument_p)
{
    tEdrvInstance*  pInstance = (tEdrvInstance*)pArgument_p;
    int             rawSockRet;
    u_char          aBuffer[EDRV_MAX_FRAME_SIZE];

    DEBUG_LVL_EDRV_TRACE("%s(): ThreadId:%ld\n", __func__, syscall(SYS_gettid));

    // signal that thread is successfully started
    sem_post(&pInstance->syncSem);

    while (edrvInstance_l.fStartCommunication)
    {
        rawSockRet = recvfrom(edrvInstance_l.sock, aBuffer, EDRV_MAX_FRAME_SIZE, 0, 0, 0);
        if (rawSockRet > 0)
        {
            packetHandler(pInstance, rawSockRet, aBuffer);
        }
    }
    edrvInstance_l.fThreadIsExited = TRUE;

    return NULL;
}

//------------------------------------------------------------------------------
/**
\brief  Get Edrv MAC address

This function gets the interface's MAC address.

\param[in]      pIfName_p           Ethernet interface device name
\param[out]     pMacAddr_p          Pointer to store MAC address
*/
//------------------------------------------------------------------------------
static void getMacAdrs(const char* pIfName_p, UINT8* pMacAddr_p)
{
    int             fd;
    struct ifreq    ifr;

    fd = socket(AF_INET, SOCK_DGRAM, 0);

    ifr.ifr_addr.sa_family = AF_INET;
    strncpy(ifr.ifr_name, pIfName_p, IFNAMSIZ - 1);

    ioctl(fd, SIOCGIFHWADDR, &ifr);

    close(fd);

    OPLK_MEMCPY(pMacAddr_p, ifr.ifr_hwaddr.sa_data, 6);
}

//------------------------------------------------------------------------------
/**
\brief  Get link status

This function returns the interface link status.

\param[in]      pIfName_p           Ethernet interface device name

\return The function returns the link status.
\retval TRUE    The link is up.
\retval FALSE   The link is down.
*/
//------------------------------------------------------------------------------
static BOOL getLinkStatus(const char* pIfName_p)
{
    BOOL            fRunning;
    struct ifreq    ethreq;
    int             fd;

    fd = socket(AF_INET, SOCK_DGRAM, 0);

    OPLK_MEMSET(&ethreq, 0, sizeof(ethreq));

    // Set the name of the interface we wish to check
    strncpy(ethreq.ifr_name, pIfName_p, IFNAMSIZ);

    // Grab flags associated with this interface
    ioctl(fd, SIOCGIFFLAGS, &ethreq);

    if (ethreq.ifr_flags & IFF_RUNNING)
    {
        fRunning = TRUE;
    }
    else
    {
        fRunning = FALSE;
    }

    close(fd);

    return fRunning;
}
