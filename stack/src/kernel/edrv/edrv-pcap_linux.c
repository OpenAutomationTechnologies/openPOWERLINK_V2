/**
********************************************************************************
\file   edrv-pcap_linux.c

\brief  Implementation of Linux pcap Ethernet driver

This file contains the implementation of the Linux pcap Ethernet driver.

\ingroup module_edrv
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2015, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
Copyright (c) 2013, Kalycito Infotech Private Limited
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
#include <pcap.h>
#include <string.h>
#include <semaphore.h>
#include <pthread.h>
#include <sys/select.h>
#include <sys/syscall.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <netinet/in.h>
#include <net/if.h>

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
#define EDRV_MAX_FRAME_SIZE     0x600

//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------
/**
\brief Structure describing an instance of the Edrv

This structure describes an instance of the Ethernet driver.
*/
typedef struct
{
    tEdrvInitParam      initParam;                          ///< Init parameters
    tEdrvTxBuffer*      pTransmittedTxBufferLastEntry;      ///< Pointer to the last entry of the transmitted TX buffer
    tEdrvTxBuffer*      pTransmittedTxBufferFirstEntry;     ///< Pointer to the first entry of the transmitted Tx buffer
    pthread_mutex_t     mutex;                              ///< Mutex for locking of critical sections
    sem_t               syncSem;                            ///< Semaphore for signalling the start of the worker thread
    pcap_t*             pPcap;                              ///< Pointer to the pcap interface instance
    pcap_t*             pPcapThread;                        ///< Handle of the pcap packet handler thread
    pthread_t           hThread;                            ///< Handle of the worker thread
} tEdrvInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tEdrvInstance edrvInstance_l;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static void packetHandler(u_char* pParam_p, const struct pcap_pkthdr* pHeader_p, const u_char* pPktData_p);
static void* workerThread(void* pArgument_p);
static void getMacAdrs(const char* pIfName_p, UINT8* pMacAddr_p);
static INT getLinkStatus(const char* pIfName_p);

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Ethernet driver initialization

This function initializes the Ethernet driver.

\param  pEdrvInitParam_p    Edrv initialization parameters

\return The function returns a tOplkError error code.

\ingroup module_edrv
*/
//------------------------------------------------------------------------------
tOplkError edrv_init(tEdrvInitParam* pEdrvInitParam_p)
{
    char                errorMessage[PCAP_ERRBUF_SIZE];
    struct sched_param  schedParam;

    // clear instance structure
    OPLK_MEMSET(&edrvInstance_l, 0, sizeof(edrvInstance_l));

    if (pEdrvInitParam_p->hwParam.pDevName == NULL)
        return kErrorEdrvInit;

    /* if no MAC address was specified read MAC address of used
     * Ethernet interface
     */
    if ((pEdrvInitParam_p->aMacAddr[0] == 0) &&
        (pEdrvInitParam_p->aMacAddr[1] == 0) &&
        (pEdrvInitParam_p->aMacAddr[2] == 0) &&
        (pEdrvInitParam_p->aMacAddr[3] == 0) &&
        (pEdrvInitParam_p->aMacAddr[4] == 0) &&
        (pEdrvInitParam_p->aMacAddr[5] == 0))
    {   // read MAC address from controller
        getMacAdrs(pEdrvInitParam_p->hwParam.pDevName,
                   pEdrvInitParam_p->aMacAddr);
    }

    // save the init data (with updated MAC address)
    edrvInstance_l.initParam = *pEdrvInitParam_p;

    edrvInstance_l.pPcap = pcap_open_live(
                        edrvInstance_l.initParam.hwParam.pDevName,
                        65535,  // snaplen
                        1,      // promiscuous mode
                        1,      // milli seconds read timeout
                        errorMessage
                    );

    if (edrvInstance_l.pPcap == NULL)
    {
        DEBUG_LVL_ERROR_TRACE("%s() Error!! Can't open pcap: %s\n", __func__, errorMessage);
        return kErrorEdrvInit;
    }

    if (pcap_setdirection(edrvInstance_l.pPcap, PCAP_D_OUT) < 0)
    {
        DEBUG_LVL_ERROR_TRACE("%s() couldn't set PCAP direction\n", __func__);
        return kErrorEdrvInit;
    }

    if (pthread_mutex_init(&edrvInstance_l.mutex, NULL) != 0)
    {
        DEBUG_LVL_ERROR_TRACE("%s() couldn't init mutex\n", __func__);
        return kErrorEdrvInit;
    }

    if (sem_init(&edrvInstance_l.syncSem, 0, 0) != 0)
    {
        DEBUG_LVL_ERROR_TRACE("%s() couldn't init semaphore\n", __func__);
        return kErrorEdrvInit;
    }

    if (pthread_create(&edrvInstance_l.hThread, NULL,
                       workerThread,  &edrvInstance_l) != 0)
    {
        DEBUG_LVL_ERROR_TRACE("%s() Couldn't create worker thread!\n", __func__);
        return kErrorEdrvInit;
    }

    schedParam.sched_priority = CONFIG_THREAD_PRIORITY_MEDIUM;
    if (pthread_setschedparam(edrvInstance_l.hThread, SCHED_FIFO, &schedParam) != 0)
    {
        DEBUG_LVL_ERROR_TRACE("%s() couldn't set thread scheduling parameters!\n",
                                __func__);
    }

#if (defined(__GLIBC__) && __GLIBC__ >= 2 && __GLIBC_MINOR__ >= 12)
    pthread_setname_np(edrvInstance_l.hThread, "oplk-edrvpcap");
#endif

    /* wait until thread is started */
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
    // End the pcap loop and wait for the worker thread to terminate
    pcap_breakloop(edrvInstance_l.pPcapThread);
    pthread_join(edrvInstance_l.hThread, NULL);

    // Close pcap instance
    pcap_close(edrvInstance_l.pPcap);

    // Destroy the mutex
    pthread_mutex_destroy(&edrvInstance_l.mutex);

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
UINT8* edrv_getMacAddr(void)
{
    return edrvInstance_l.initParam.aMacAddr;
}

//------------------------------------------------------------------------------
/**
\brief  Send Tx buffer

This function sends the Tx buffer.

\param  pBuffer_p           Tx buffer descriptor

\return The function returns a tOplkError error code.

\ingroup module_edrv
*/
//------------------------------------------------------------------------------
tOplkError edrv_sendTxBuffer(tEdrvTxBuffer* pBuffer_p)
{
    int         pcapRet;

    FTRACE_MARKER("%s", __func__);

    if (pBuffer_p->txBufferNumber.pArg != NULL)
        return kErrorInvalidOperation;

    if (getLinkStatus(edrvInstance_l.initParam.hwParam.pDevName) == FALSE)
    {
        /* there's no link! We pretend that packet is sent and immediately call
         * tx handler! Otherwise the stack would hang! */
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
            edrvInstance_l.pTransmittedTxBufferLastEntry =
                edrvInstance_l.pTransmittedTxBufferFirstEntry = pBuffer_p;
        }
        else
        {
            edrvInstance_l.pTransmittedTxBufferLastEntry->txBufferNumber.pArg = pBuffer_p;
            edrvInstance_l.pTransmittedTxBufferLastEntry = pBuffer_p;
        }
        pthread_mutex_unlock(&edrvInstance_l.mutex);

        pcapRet = pcap_sendpacket(edrvInstance_l.pPcap, pBuffer_p->pBuffer,
                                  (int)pBuffer_p->txFrameSize);
        if (pcapRet != 0)
        {
            DEBUG_LVL_EDRV_TRACE("%s() pcap_sendpacket returned %d (%s)\n",
                                 __func__, pcapRet, pcap_geterr(edrvInstance_l.pPcap));
            return kErrorInvalidOperation;
        }
    }

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Allocate Tx buffer

This function allocates a Tx buffer.

\param  pBuffer_p           Tx buffer descriptor

\return The function returns a tOplkError error code.

\ingroup module_edrv
*/
//------------------------------------------------------------------------------
tOplkError edrv_allocTxBuffer(tEdrvTxBuffer* pBuffer_p)
{
    if (pBuffer_p->maxBufferSize > EDRV_MAX_FRAME_SIZE)
        return kErrorEdrvNoFreeBufEntry;

    // allocate buffer with malloc
    pBuffer_p->pBuffer = (UINT8*)OPLK_MALLOC(pBuffer_p->maxBufferSize);
    if (pBuffer_p->pBuffer == NULL)
        return kErrorEdrvNoFreeBufEntry;

    pBuffer_p->txBufferNumber.pArg = NULL;

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Free Tx buffer

This function releases the Tx buffer.

\param  pBuffer_p           Tx buffer descriptor

\return The function returns a tOplkError error code.

\ingroup module_edrv
*/
//------------------------------------------------------------------------------
tOplkError edrv_freeTxBuffer(tEdrvTxBuffer* pBuffer_p)
{
    UINT8* pBuffer = pBuffer_p->pBuffer;

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

\param  pFilter_p           Base pointer of Rx filter array
\param  count_p             Number of Rx filter array entries
\param  entryChanged_p      Index of Rx filter entry that shall be changed
\param  changeFlags_p       Bit mask that selects the changing Rx filter property

\return The function returns a tOplkError error code.

\ingroup module_edrv
*/
//------------------------------------------------------------------------------
tOplkError edrv_changeRxFilter(tEdrvFilter* pFilter_p, UINT count_p,
                               UINT entryChanged_p, UINT changeFlags_p)
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

\param  pMacAddr_p  Multicast address

\return The function returns a tOplkError error code.

\ingroup module_edrv
*/
//------------------------------------------------------------------------------
tOplkError edrv_clearRxMulticastMacAddr(UINT8* pMacAddr_p)
{
    UNUSED_PARAMETER(pMacAddr_p);

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Set multicast address entry

This function sets a multicast entry into the Ethernet controller.

\param  pMacAddr_p  Multicast address

\return The function returns a tOplkError error code.

\ingroup module_edrv
*/
//------------------------------------------------------------------------------
tOplkError edrv_setRxMulticastMacAddr(UINT8* pMacAddr_p)
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

\param  pParam_p    User specific pointer pointing to the instance structure
\param  pHeader_p   Packet header information (e.g. size)
\param  pPktData_p  Packet buffer
*/
//------------------------------------------------------------------------------
static void packetHandler(u_char* pParam_p, const struct pcap_pkthdr* pHeader_p, const u_char* pPktData_p)
{
    tEdrvInstance*  pInstance = (tEdrvInstance*)pParam_p;
    tEdrvRxBuffer   rxBuffer;

    if (OPLK_MEMCMP(pPktData_p + 6, pInstance->initParam.aMacAddr, 6) != 0)
    {   // filter out self generated traffic
        rxBuffer.bufferInFrame = kEdrvBufferLastInFrame;
        rxBuffer.rxFrameSize = pHeader_p->caplen;
        rxBuffer.pBuffer = (UINT8*)pPktData_p;

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
                {
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
                          (UINT)pPktData_p[0],
                          (UINT)pPktData_p[1],
                          (UINT)pPktData_p[2],
                          (UINT)pPktData_p[3],
                          (UINT)pPktData_p[4],
                          (UINT)pPktData_p[5]);
                    TRACE("   current TxB %p: DstMAC=%02X%02X%02X%02X%02X%02X\n",
                          (void*)pTxBuffer,
                          (UINT)pTxBuffer->pBuffer[0],
                          (UINT)pTxBuffer->pBuffer[1],
                          (UINT)pTxBuffer->pBuffer[2],
                          (UINT)pTxBuffer->pBuffer[3],
                          (UINT)pTxBuffer->pBuffer[4],
                          (UINT)pTxBuffer->pBuffer[5]);
                }
            }
        }
        else
        {
            TRACE("%s: no TxB: DstMAC=%02X%02X%02X%02X%02X%02X\n", __func__,
                  pPktData_p[0],
                  pPktData_p[1],
                  pPktData_p[2],
                  pPktData_p[3],
                  pPktData_p[4],
                  pPktData_p[5]);
        }
    }
}

//------------------------------------------------------------------------------
/**
\brief  Edrv worker thread

This function implements the edrv worker thread. It is responsible to handle
pcap events.

\param  pArgument_p     User specific pointer pointing to the instance structure

\return The function returns a thread error code.
*/
//------------------------------------------------------------------------------
static void* workerThread(void* pArgument_p)
{
    tEdrvInstance*  pInstance = (tEdrvInstance*)pArgument_p;
    int             pcapRet;
    char            errorMessage[PCAP_ERRBUF_SIZE];

    DEBUG_LVL_EDRV_TRACE("%s(): ThreadId:%ld\n", __func__, syscall(SYS_gettid));

    pInstance->pPcapThread =
        pcap_open_live(pInstance->initParam.hwParam.pDevName,
                       65535,  // snaplen
                       1,      // promiscuous mode
                       1,      // milli seconds read timeout
                       errorMessage);

   if (pInstance->pPcapThread == NULL)
   {
       DEBUG_LVL_ERROR_TRACE("%s() Error!! Can't open pcap: %s\n", __func__, errorMessage);
       return NULL;
   }

   if (pcap_setdirection(pInstance->pPcapThread, PCAP_D_INOUT) < 0)
   {
       DEBUG_LVL_ERROR_TRACE("%s() couldn't set PCAP direction1\n", __func__);
   }

   /* signal that thread is successfully started */
   sem_post(&pInstance->syncSem);

   pcapRet = pcap_loop(pInstance->pPcapThread, -1, packetHandler, (u_char*)pInstance);

   switch (pcapRet)
   {
       case 0:
           DEBUG_LVL_ERROR_TRACE("%s(): pcap_loop ended because 'cnt' is exhausted.\n", __func__);
           break;

       case -1:
           DEBUG_LVL_ERROR_TRACE("%s(): pcap_loop ended because of an error!\n", __func__);
           break;

       case -2:
           DEBUG_LVL_ERROR_TRACE("%s(): pcap_loop ended normally.\n", __func__);
           break;

       default:
           DEBUG_LVL_ERROR_TRACE("%s(): pcap_loop ended (unknown return value).\n", __func__);
           break;
   }

   return NULL;
}

//------------------------------------------------------------------------------
/**
\brief  Get Edrv MAC address

This function gets the interface's MAC address.

\param  pIfName_p   Ethernet interface device name
\param  pMacAddr_p  Pointer to store MAC address
*/
//------------------------------------------------------------------------------
static void getMacAdrs(const char* pIfName_p, UINT8* pMacAddr_p)
{
    INT             fd;
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

\param  pIfName_p  Ethernet interface device name

\return The function returns the link status.
\retval TRUE    The link is up.
\retval FALSE   The link is down.
*/
//------------------------------------------------------------------------------
static INT getLinkStatus(const char* pIfName_p)
{
    BOOL            fRunning;
    struct ifreq    ethreq;
    INT             fd;

    fd = socket(AF_INET, SOCK_DGRAM, 0);

    OPLK_MEMSET(&ethreq, 0, sizeof(ethreq));

    /* set the name of the interface we wish to check */
    strncpy(ethreq.ifr_name, pIfName_p, IFNAMSIZ);

    /* grab flags associated with this interface */
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

/// \}
