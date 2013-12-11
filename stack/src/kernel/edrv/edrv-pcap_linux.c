/****************************************************************************
  File:         EdrvPcapLinux.c

  (c) Bernecker + Rainer Industrie-Elektronik Ges.m.b.H.
      B&R Strasse 1, A-5142 Eggelsberg, Austria
      www.br-automation.com

  (c) Kalycito Infotech Private Limited

  Project:      openPOWERLINK

  Description:  Linux PCAP implementation of openPOWERLINK Edrv module

  License:

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    1. Redistributions of source code must retain the above copyright
       notice, this list of conditions and the following disclaimer.

    2. Redistributions in binary form must reproduce the above copyright
       notice, this list of conditions and the following disclaimer in the
       documentation and/or other materials provided with the distribution.

    3. Neither the name of the copyright holders nor the names of
       its contributors may be used to endorse or promote products derived
       from this software without prior written permission. For written
       permission, please contact office@br-automation.com.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    POSSIBILITY OF SUCH DAMAGE.

    Severability Clause:

        If a provision of this License is or becomes illegal, invalid or
        unenforceable in any jurisdiction, that shall not affect:
        1. the validity or enforceability in that jurisdiction of any other
           provision of this License; or
        2. the validity or enforceability in other jurisdictions of that or
           any other provision of this License.

****************************************************************************/

#include "edrv.h"

#include <unistd.h>
#include <pcap.h>
#include <string.h>
#include <pthread.h>
#include <sys/select.h>
#include <sys/syscall.h>
#include <semaphore.h>

#include <sys/socket.h>
#include <sys/ioctl.h>
#include <netinet/in.h>
#include <net/if.h>

/***************************************************************************/
/*                                                                         */
/*                                                                         */
/*          G L O B A L   D E F I N I T I O N S                            */
/*                                                                         */
/*                                                                         */
/***************************************************************************/
//---------------------------------------------------------------------------
// const defines
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
// module global types
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
// local vars
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
// local function prototypes
//---------------------------------------------------------------------------

/***************************************************************************/
/*                                                                         */
/*                                                                         */
/*          C L A S S  <edrv>                                              */
/*                                                                         */
/*                                                                         */
/***************************************************************************/
//
// Description:
//
//
/***************************************************************************/

//=========================================================================//
//                                                                         //
//          P R I V A T E   D E F I N I T I O N S                          //
//                                                                         //
//=========================================================================//

//---------------------------------------------------------------------------
// const defines
//---------------------------------------------------------------------------
#define EDRV_MAX_FRAME_SIZE     0x600

//---------------------------------------------------------------------------
// local types
//---------------------------------------------------------------------------

// Private structure
typedef struct
{
    tEdrvInitParam      m_initParam;
    tEdrvTxBuffer*      m_pTransmittedTxBufferLastEntry;
    tEdrvTxBuffer*      m_pTransmittedTxBufferFirstEntry;
    pthread_mutex_t     m_mutex;
    sem_t               m_syncSem;
    pcap_t*             m_pPcap;
    pcap_t*             m_pPcapThread;
    pthread_t           m_hThread;
} tEdrvInstance;

//---------------------------------------------------------------------------
// local vars
//---------------------------------------------------------------------------
static tEdrvInstance EdrvInstance_l;

//---------------------------------------------------------------------------
// local function prototypes
//---------------------------------------------------------------------------
static void EdrvPacketHandler(u_char *param, const struct pcap_pkthdr *header, const u_char *pkt_data);
static void *EdrvWorkerThread(void *);

//---------------------------------------------------------------------------
// Function:            getMacAdrs
//
// Description:         get mac address of interface
//
// Parameters:          ifName  device name of ethernet interface
//                      macAdrs Pointer to store MAC address
//
// Returns:             void
//---------------------------------------------------------------------------
static void getMacAdrs(const char *ifName, BYTE *macAdrs)
{
    int    fd;
    struct ifreq ifr;

    fd = socket(AF_INET, SOCK_DGRAM, 0);

    ifr.ifr_addr.sa_family = AF_INET;
    strncpy(ifr.ifr_name, ifName, IFNAMSIZ - 1);

    ioctl(fd, SIOCGIFHWADDR, &ifr);

    close(fd);

    EPL_MEMCPY(macAdrs, ifr.ifr_hwaddr.sa_data, 6);
}

//---------------------------------------------------------------------------
// Function:            getLinkStatus
//
// Description:         get link status of interface
//
// Parameters:          ifName  device name of ethernet interface
//
// Returns:             TRUE if link is up or FALSE otherwise
//---------------------------------------------------------------------------
static int getLinkStatus(const char *ifName)
{
    BOOL            fRunning;
    struct ifreq    ethreq;
    int             fd;

    fd = socket(AF_INET, SOCK_DGRAM, 0);

    memset(&ethreq, 0, sizeof(ethreq));

    /* set the name of the interface we wish to check */
    strncpy(ethreq.ifr_name, ifName, IFNAMSIZ);

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

//---------------------------------------------------------------------------
// Function:    edrv_init
//
// Description: function for init of the Ethernet controller
//
// Parameters:  pEdrvInitParam_p    = pointer to struct including the init-parameters
//
// Returns:     Errorcode           = kEplSuccessful
//                                  = kEplNoResource
//---------------------------------------------------------------------------
tEplKernel edrv_init(tEdrvInitParam *pEdrvInitParam_p)
{
    tEplKernel                  Ret;
    char                        sErr_Msg[PCAP_ERRBUF_SIZE];
    struct sched_param          schedParam;

    Ret = kEplSuccessful;

    // clear instance structure
    EPL_MEMSET(&EdrvInstance_l, 0, sizeof (EdrvInstance_l));

    if (pEdrvInitParam_p->hwParam.m_pszDevName == NULL)
    {
        Ret = kEplEdrvInitError;
        goto Exit;
    }

    /* if no MAC address was specified read MAC address of used
     * ethernet interface
     */
    if ((pEdrvInitParam_p->aMacAddr[0] == 0) &&
        (pEdrvInitParam_p->aMacAddr[1] == 0) &&
        (pEdrvInitParam_p->aMacAddr[2] == 0) &&
        (pEdrvInitParam_p->aMacAddr[3] == 0) &&
        (pEdrvInitParam_p->aMacAddr[4] == 0) &&
        (pEdrvInitParam_p->aMacAddr[5] == 0)  )
    {   // read MAC address from controller
        getMacAdrs(pEdrvInitParam_p->hwParam.m_pszDevName,
                   pEdrvInitParam_p->aMacAddr);
    }

    // save the init data (with updated MAC address)
    EdrvInstance_l.m_initParam = *pEdrvInitParam_p;

    EdrvInstance_l.m_pPcap = pcap_open_live (
                        EdrvInstance_l.m_initParam.hwParam.m_pszDevName,
                        65535,  // snaplen
                        1,      // promiscuous mode
                        1,      // milli seconds read timeout
                        sErr_Msg
                    );

    if ( EdrvInstance_l.m_pPcap == NULL )
    {
        EPL_DBGLVL_ERROR_TRACE("%s() Error!! Can't open pcap: %s\n", __func__,
                                sErr_Msg);
        Ret = kEplEdrvInitError;
        goto Exit;
    }

    if (pcap_setdirection(EdrvInstance_l.m_pPcap, PCAP_D_OUT) < 0)
    {
        EPL_DBGLVL_ERROR_TRACE("%s() couldn't set PCAP direction\n", __func__);
        Ret = kEplEdrvInitError;
        goto Exit;
    }

    if (pthread_mutex_init(&EdrvInstance_l.m_mutex, NULL) != 0)
    {
        EPL_DBGLVL_ERROR_TRACE("%s() couldn't init mutex\n", __func__);
        Ret = kEplEdrvInitError;
        goto Exit;
    }

    if (sem_init(&EdrvInstance_l.m_syncSem, 0, 0) != 0)
    {
        EPL_DBGLVL_ERROR_TRACE("%s() couldn't init semaphore\n", __func__);
        Ret = kEplEdrvInitError;
        goto Exit;
    }

    if (pthread_create(&EdrvInstance_l.m_hThread, NULL,
                       EdrvWorkerThread,  &EdrvInstance_l) != 0)
    {
        EPL_DBGLVL_ERROR_TRACE("%s() Couldn't create worker thread!\n", __func__);
        Ret = kEplEdrvInitError;
        goto Exit;
    }

    schedParam.__sched_priority = EPL_THREAD_PRIORITY_MEDIUM;
    if (pthread_setschedparam(EdrvInstance_l.m_hThread, SCHED_FIFO, &schedParam) != 0)
    {
        EPL_DBGLVL_ERROR_TRACE("%s() couldn't set thread scheduling parameters!\n",
                                __func__);
    }

    /* wait until thread is started */
    sem_wait(&EdrvInstance_l.m_syncSem);

Exit:
    return Ret;
}

//---------------------------------------------------------------------------
// Function:    edrv_shutdown
//
// Description: Shutdown the Ethernet controller
//
// Parameters:  void
//
// Returns:     Errorcode   = kEplSuccessful
//---------------------------------------------------------------------------
tEplKernel edrv_shutdown( void )
{
    // signal shutdown to the thread
    //pthread_cancel(EdrvInstance_l.m_hThread);
    pcap_breakloop(EdrvInstance_l.m_pPcapThread);

    // wait for thread to terminate
    pthread_join (EdrvInstance_l.m_hThread, NULL);

    pcap_close(EdrvInstance_l.m_pPcap);

    pthread_mutex_destroy(&EdrvInstance_l.m_mutex);

    // clear instance structure
    EPL_MEMSET(&EdrvInstance_l, 0, sizeof (EdrvInstance_l));

    return kEplSuccessful; //assuming no problems with closing the handle
}

//---------------------------------------------------------------------------
// Function:    edrv_sendTxBuffer
//
// Description: immediately starts the transmission of the buffer
//
// Parameters:  pBuffer_p   = buffer descriptor to transmit
//
// Returns:     Errorcode   = kEplSuccessful
//---------------------------------------------------------------------------
tEplKernel edrv_sendTxBuffer(tEdrvTxBuffer *pBuffer_p)
{
    tEplKernel  Ret = kEplSuccessful;
    INT         iRet;

    FTRACE_MARKER("%s", __func__);

    if (pBuffer_p->txBufferNumber.pArg != NULL)
    {
        Ret = kEplInvalidOperation;
        goto Exit;
    }

    if (getLinkStatus(EdrvInstance_l.m_initParam.hwParam.m_pszDevName) == FALSE)
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
        pthread_mutex_lock(&EdrvInstance_l.m_mutex);
        if (EdrvInstance_l.m_pTransmittedTxBufferLastEntry == NULL)
        {
            EdrvInstance_l.m_pTransmittedTxBufferLastEntry =
                EdrvInstance_l.m_pTransmittedTxBufferFirstEntry = pBuffer_p;
        }
        else
        {
            EdrvInstance_l.m_pTransmittedTxBufferLastEntry->txBufferNumber.pArg = pBuffer_p;
            EdrvInstance_l.m_pTransmittedTxBufferLastEntry = pBuffer_p;
        }
        pthread_mutex_unlock(&EdrvInstance_l.m_mutex);

        iRet = pcap_sendpacket(EdrvInstance_l.m_pPcap, pBuffer_p->pBuffer,
                               (int) pBuffer_p->txFrameSize);
        if  (iRet != 0)
        {
            EPL_DBGLVL_EDRV_TRACE("%s() pcap_sendpacket returned %d (%s)\n",
                    __func__, iRet, pcap_geterr(EdrvInstance_l.m_pPcap));
            Ret = kEplInvalidOperation;
        }
    }

Exit:
    return Ret;
}

//---------------------------------------------------------------------------
// Function:    edrv_allocTxBuffer
//
// Description: Register a Tx-Buffer
//
// Parameters:  pBuffer_p   = pointer to Buffer structure
//
// Returns:     Errorcode   = kEplSuccessful
//                          = kEplEdrvNoFreeBufEntry
//---------------------------------------------------------------------------
tEplKernel edrv_allocTxBuffer(tEdrvTxBuffer * pBuffer_p)
{
    tEplKernel Ret = kEplSuccessful;

    if (pBuffer_p->maxBufferSize > EDRV_MAX_FRAME_SIZE)
    {
        Ret = kEplEdrvNoFreeBufEntry;
        goto Exit;
    }

    // allocate buffer with malloc
    pBuffer_p->pBuffer = EPL_MALLOC(pBuffer_p->maxBufferSize);
    if (pBuffer_p->pBuffer == NULL)
    {
        Ret = kEplEdrvNoFreeBufEntry;
        goto Exit;
    }

    pBuffer_p->txBufferNumber.pArg = NULL;

Exit:
    return Ret;
}

//---------------------------------------------------------------------------
// Function:    edrv_freeTxBuffer
//
// Description: Register a Tx-Buffer
//
// Parameters:  pBuffer_p   = pointer to Buffer structure
//
// Returns:     Errorcode   = kEplSuccessful
//---------------------------------------------------------------------------
tEplKernel edrv_freeTxBuffer(tEdrvTxBuffer * pBuffer_p)
{
    BYTE*   pbBuffer = pBuffer_p->pBuffer;

    // mark buffer as free, before actually freeing it
    pBuffer_p->pBuffer = NULL;

    EPL_FREE(pbBuffer);

    return kEplSuccessful;
}

//---------------------------------------------------------------------------
// Function:    edrv_changeRxFilter
//
// Description: Change all rx-filters or one specific rx-filter
//              of the openMAC
//
// Parameters:  pFilter_p           = pointer to array of filter entries
//              uiCount_p           = number of filters in array
//              uiEntryChanged_p    = selects one specific filter which is
//                                    to be changed. If value is equal to
//                                    or larger than uiCount_p, all entries
//                                    are selected.
//              uiChangeFlags_p     = If one specific entry is selected,
//                                    these flag bits show which filter
//                                    properties have been changed.
//                                    available flags:
//                                      EDRV_FILTER_CHANGE_MASK
//                                      EDRV_FILTER_CHANGE_VALUE
//                                      EDRV_FILTER_CHANGE_STATE
//                                      EDRV_FILTER_CHANGE_AUTO_RESPONSE
//                                    if auto-response delay is supported:
//                                      EDRV_FILTER_CHANGE_AUTO_RESPONSE_DELAY
//
// Returns:     Errorcode           = kEplSuccessful
//                                  = kEplEdrvInvalidParam
//---------------------------------------------------------------------------
tEplKernel edrv_changeRxFilter(tEdrvFilter*    pFilter_p __attribute__((unused)),
                            unsigned int    uiCount_p __attribute__((unused)),
                            unsigned int    uiEntryChanged_p __attribute__((unused)),
                            unsigned int    uiChangeFlags_p __attribute__((unused)))
{
    return kEplSuccessful;
}

//---------------------------------------------------------------------------
//
// Function:    edrv_clearRxMulticastMacAddr
//
// Description: Reset a multicast entry in the Ethernet controller
//
// Parameters:  pbMacAddr_p     = pointer to multicast entry to reset
//
// Returns:     Errorcode       = kEplSuccessful
//---------------------------------------------------------------------------
tEplKernel edrv_clearRxMulticastMacAddr (BYTE * pbMacAddr_p __attribute__((unused)))
{
    return kEplSuccessful;
}

//---------------------------------------------------------------------------
//
// Function:    edrv_setRxMulticastMacAddr
//
// Description: Set a multicast entry into the Ethernet controller
//
// Parameters:  pbMacAddr_p     = pointer to multicast entry to set
//
// Returns:     Errorcode       = kEplSuccessful
//---------------------------------------------------------------------------
tEplKernel edrv_setRxMulticastMacAddr (BYTE * pbMacAddr_p __attribute__((unused)))
{
    return kEplSuccessful;
}

//---------------------------------------------------------------------------
//
// Function:    EdrvPacketHandler
//
// Description: Pcap packet handler, that forwards the frames to the DLL
//
// Parameters:  pUser_p         = user specific pointer,
//                                which points to the instance structure
//              header          = packet header (size, ...)
//              pkt_data        = packet buffer
//
// Returns:     void
//---------------------------------------------------------------------------
static void EdrvPacketHandler(u_char *pUser_p,
                              const struct pcap_pkthdr *header,
                              const u_char *pkt_data)
{
    tEdrvInstance*  pInstance = (tEdrvInstance*) pUser_p;
    tEdrvRxBuffer   RxBuffer;

    if (memcmp (pkt_data + 6, pInstance->m_initParam.aMacAddr, 6 ) != 0)
    {   // filter out self generated traffic
        RxBuffer.bufferInFrame    = kEdrvBufferLastInFrame;
        RxBuffer.rxFrameSize       = header->caplen;
        RxBuffer.pBuffer         = (BYTE*) pkt_data;

        FTRACE_MARKER("%s RX", __func__);
        pInstance->m_initParam.pfnRxHandler(&RxBuffer);
    }
    else
    {   // self generated traffic
        FTRACE_MARKER("%s TX-receive", __func__);

        if (pInstance->m_pTransmittedTxBufferFirstEntry != NULL)
        {
            tEdrvTxBuffer* pTxBuffer = pInstance->m_pTransmittedTxBufferFirstEntry;

            if (pTxBuffer->pBuffer != NULL)
            {
                if (memcmp(pkt_data, pTxBuffer->pBuffer, 6) == 0)
                {
                    pthread_mutex_lock(&pInstance->m_mutex);
                    pInstance->m_pTransmittedTxBufferFirstEntry =
                        pInstance->m_pTransmittedTxBufferFirstEntry->txBufferNumber.pArg;
                    if (pInstance->m_pTransmittedTxBufferFirstEntry == NULL)
                    {
                        pInstance->m_pTransmittedTxBufferLastEntry = NULL;
                    }
                    pthread_mutex_unlock(&pInstance->m_mutex);

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
                        (UINT)pkt_data[0],
                        (UINT)pkt_data[1],
                        (UINT)pkt_data[2],
                        (UINT)pkt_data[3],
                        (UINT)pkt_data[4],
                        (UINT)pkt_data[5]);
                    TRACE("   current TxB %p: DstMAC=%02X%02X%02X%02X%02X%02X\n",
                        (void *)pTxBuffer,
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
            //TRACE("%s: no TxB: DstMAC=%02X%02X%02X%02X%02X%02X\n", __func__, pkt_data[0], pkt_data[1],
            //      pkt_data[2], pkt_data[3], pkt_data[4], pkt_data[5]);
        }
    }
}

//---------------------------------------------------------------------------
// Function:    EdrvWorkerThread
//
// Description: Worker thread, that processes several events like
//              Pcap events and timer events in one single thread as emulation
//              of non-reentrant interrupt processing. The receive, transmit
//              and timer callback functions of the DLL are mutual exclusive.
//
// Parameters:  pArgument_p     = user specific argument, i.e. pointer to
//                                instance structure
//
// Returns:     DWORD           = thread return code
//---------------------------------------------------------------------------
static void * EdrvWorkerThread(void *pArgument_p)
{
    int PcapRet;

    tEdrvInstance*  pInstance = (tEdrvInstance *)pArgument_p;
    char sErr_Msg[ PCAP_ERRBUF_SIZE ];

    EPL_DBGLVL_EDRV_TRACE("%s(): ThreadId:%ld\n", __func__, syscall(SYS_gettid));

    pInstance->m_pPcapThread =
        pcap_open_live (pInstance->m_initParam.hwParam.m_pszDevName,
                           65535,  // snaplen
                           1,      // promiscuous mode
                           1,      // milli seconds read timeout
                           sErr_Msg);

   if (pInstance->m_pPcapThread == NULL)
   {
       EPL_DBGLVL_ERROR_TRACE("%s() Error!! Can't open pcap: %s\n", __func__,
                               sErr_Msg);
       return NULL;
   }

   if (pcap_setdirection(pInstance->m_pPcapThread, PCAP_D_INOUT) < 0)
   {
       EPL_DBGLVL_ERROR_TRACE("%s() couldn't set PCAP direction1\n", __func__);
   }

   /* signal that thread is successfully started */
   sem_post(&pInstance->m_syncSem);

   PcapRet  = pcap_loop (pInstance->m_pPcapThread, -1, EdrvPacketHandler, (u_char*)pInstance);

   switch( PcapRet )
   {
       case 0:
           EPL_DBGLVL_ERROR_TRACE("%s(): pcap_loop ended because 'cnt' is exhausted.\n", __func__);
           break;

       case -1:
           EPL_DBGLVL_ERROR_TRACE("%s(): pcap_loop ended because of an error!\n", __func__);
           break;

       case -2:
           EPL_DBGLVL_ERROR_TRACE("%s(): pcap_loop ended normally.\n", __func__);
           break;

       default:
           EPL_DBGLVL_ERROR_TRACE("%s(): pcap_loop ended (unknown return value).\n", __func__);
           break;
   }

   return NULL;
}


