/**
********************************************************************************
\file   edrv-pcap_win.c

\brief  Implementation of WinPCAP Ethernet driver

This file contains the implementation of the WinPCAP Ethernet driver.

\ingroup module_edrv
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2013, Kalycito Infotech Private Limited
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
#include <oplk/oplkinc.h>
#include <kernel/edrv.h>

#define WPCAP           // include Windows pcap extensions
#include <pcap.h>
#include <iphlpapi.h>

#include <kernel/hrestimer.h>

//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#define EDRV_MAX_FRAME_SIZE     0x600

#define EDRV_HANDLE_EVENT       0
#define EDRV_HANDLE_PCAP        1
#define EDRV_HANDLE_TIMER0      2
#define EDRV_HANDLE_TIMER1      3
#define EDRV_HANDLE_COUNT       4

//------------------------------------------------------------------------------
// module global vars
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// global function prototypes
//------------------------------------------------------------------------------
void hresTimerCb(UINT index_p);

//============================================================================//
//            P R I V A T E   D E F I N I T I O N S                           //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------

typedef struct
{
    tEdrvInitParam      initParam;
    tEdrvTxBuffer*      pTransmittedTxBufferLastEntry;
    tEdrvTxBuffer*      pTransmittedTxBufferFirstEntry;
    CRITICAL_SECTION    criticalSection;
    pcap_t*             pcap;
    HANDLE              aHandle[EDRV_HANDLE_COUNT];
    HANDLE              threadHandle;
} tEdrvInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tEdrvInstance edrInstance_l;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static void packetHandler(u_char* pParam_p, const struct pcap_pkthdr* pHeader_p, const u_char* pPktData_p);
static UINT32 WINAPI edrvWorkerThread(void*);

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
    tOplkError          ret = kErrorOk;
    UINT32              threadId;
    char                sErr_Msg[PCAP_ERRBUF_SIZE];
    // variables for IPHLPAPI
    ULONG               outBufLen;
    PIP_ADAPTER_INFO    pAdapterInfo;
    PIP_ADAPTER_INFO    pAdapter = NULL;
    UINT32              retVal = 0;

    // clear instance structure
    OPLK_MEMSET(&edrInstance_l, 0, sizeof(edrInstance_l));

    if (pEdrvInitParam_p->hwParam.pDevName == NULL)
        return kErrorEdrvInit;

    // search for the corresponding MAC address via IPHLPAPI
    outBufLen = sizeof(IP_ADAPTER_INFO);
    pAdapterInfo = (IP_ADAPTER_INFO*)OPLK_MALLOC(sizeof(IP_ADAPTER_INFO));
    if (pAdapterInfo == NULL)
    {
        DEBUG_LVL_ERROR_TRACE("Error allocating memory needed to call GetAdaptersinfo\n");
        return kErrorNoResource;
    }

    // Make an initial call to GetAdaptersInfo to get
    // the necessary size into the outBufLen variable
    retVal = GetAdaptersInfo(pAdapterInfo, &outBufLen);
    if (retVal == ERROR_BUFFER_OVERFLOW)
    {
        OPLK_FREE(pAdapterInfo);
        pAdapterInfo = (IP_ADAPTER_INFO*)OPLK_MALLOC(outBufLen);
        if (pAdapterInfo == NULL)
        {
            DEBUG_LVL_ERROR_TRACE("Error allocating memory needed to call GetAdaptersinfo\n");
            return kErrorNoResource;
        }
    }

    retVal = GetAdaptersInfo(pAdapterInfo, &outBufLen);
    if (retVal == NO_ERROR)
    {
        pAdapter = pAdapterInfo;
        while (pAdapter)
        {
            if (pAdapter->Type == MIB_IF_TYPE_ETHERNET)
            {
                if (strstr(pEdrvInitParam_p->hwParam.pDevName, pAdapter->AdapterName) != NULL)
                {   // corresponding adapter found
                    OPLK_MEMCPY(pEdrvInitParam_p->aMacAddr, pAdapter->Address,
                                min(pAdapter->AddressLength, sizeof(pEdrvInitParam_p->aMacAddr)));
                    break;
                }
            }
            pAdapter = pAdapter->Next;
        }
    }
    else
    {
        DEBUG_LVL_ERROR_TRACE("GetAdaptersInfo failed with error: %d\n", retVal);

    }
    if (pAdapterInfo)
        OPLK_FREE(pAdapterInfo);

    // save the init data (with updated MAC address)
    edrInstance_l.initParam = *pEdrvInitParam_p;
    edrInstance_l.pcap = pcap_open_live(pEdrvInitParam_p->hwParam.pDevName,
                                        65535, 1, 1, sErr_Msg);
    if (edrInstance_l.pcap == NULL)
    {
        DEBUG_LVL_ERROR_TRACE("Error!! Can't open pcap: %s\n", sErr_Msg);
        return kErrorEdrvInit;
    }

    // configure pcap for maximum responsiveness
    if (pcap_setmintocopy(edrInstance_l.pcap, 0) != 0)
    {
        DEBUG_LVL_ERROR_TRACE("pcap_setmintocopy failed\n");
    }

    // put pcap into nonblocking mode
    if (pcap_setnonblock(edrInstance_l.pcap, 1, sErr_Msg) != 0)
    {
        DEBUG_LVL_ERROR_TRACE("Can't put pcap into nonblocking mode: %s\n", sErr_Msg);
    }

    // get event handle for pcap instance
    edrInstance_l.aHandle[EDRV_HANDLE_PCAP] = pcap_getevent(edrInstance_l.pcap);

    // Create two unnamed waitable timers for EplTimerHighResk sub-module.
    edrInstance_l.aHandle[EDRV_HANDLE_TIMER0] = CreateWaitableTimer(NULL, FALSE, NULL);
    if (edrInstance_l.aHandle[EDRV_HANDLE_TIMER0] == NULL)
    {
        DEBUG_LVL_ERROR_TRACE("CreateWaitableTimer failed (%d)\n", GetLastError());
        return kErrorEdrvInit;
    }

    edrInstance_l.aHandle[EDRV_HANDLE_TIMER1] = CreateWaitableTimer(NULL, FALSE, NULL);
    if (edrInstance_l.aHandle[EDRV_HANDLE_TIMER1] == NULL)
    {
        DEBUG_LVL_ERROR_TRACE("CreateWaitableTimer failed (%d)\n", GetLastError());
        return kErrorEdrvInit;
    }

    // create event for signalling shutdown
    edrInstance_l.aHandle[EDRV_HANDLE_EVENT] = CreateEvent(NULL, FALSE, FALSE, NULL);

    // Create the thread to begin execution on its own.
    edrInstance_l.threadHandle = CreateThread(NULL, 0, edrvWorkerThread, &edrInstance_l, 0, &threadId);
    if (edrInstance_l.threadHandle == NULL)
         return kErrorEdrvInit;

    InitializeCriticalSection(&edrInstance_l.criticalSection);
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Ethernet driver shutdown

This function shuts down the Ethernet driver.

\return The function returns a tOplkError error code.

\ingroup module_edrv
*/
//------------------------------------------------------------------------------
tOplkError edrv_shutdown(void)
{
    // signal shutdown to the thread
    SetEvent(edrInstance_l.aHandle[EDRV_HANDLE_EVENT]);

    WaitForSingleObject(edrInstance_l.threadHandle, INFINITE);

    CloseHandle(edrInstance_l.threadHandle);

    pcap_close(edrInstance_l.pcap);

    CloseHandle(edrInstance_l.aHandle[EDRV_HANDLE_EVENT]);
    CloseHandle(edrInstance_l.aHandle[EDRV_HANDLE_TIMER0]);
    CloseHandle(edrInstance_l.aHandle[EDRV_HANDLE_TIMER1]);

    DeleteCriticalSection(&edrInstance_l.criticalSection);

    // clear instance structure
    OPLK_MEMSET(&edrInstance_l, 0, sizeof (edrInstance_l));

    return kErrorOk; //assuming no problems with closing the handle
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
    tOplkError  ret = kErrorOk;
    int         iRet;

    //TRACE("%s: TxB=%p (%02X), last TxB=%p\n", __func__, pBuffer_p, (UINT)pBuffer_p->pBuffer[5], edrInstance_l.pTransmittedTxBufferLastEntry);

    if (pBuffer_p->txBufferNumber.pArg != NULL)
    {
        return kErrorInvalidOperation;
    }

    EnterCriticalSection(&edrInstance_l.criticalSection);
    if (edrInstance_l.pTransmittedTxBufferLastEntry == NULL)
    {
        edrInstance_l.pTransmittedTxBufferLastEntry =
            edrInstance_l.pTransmittedTxBufferFirstEntry = pBuffer_p;
    }
    else
    {
        edrInstance_l.pTransmittedTxBufferLastEntry->txBufferNumber.pArg = pBuffer_p;
        edrInstance_l.pTransmittedTxBufferLastEntry = pBuffer_p;
    }
    LeaveCriticalSection(&edrInstance_l.criticalSection);

    iRet = pcap_sendpacket(edrInstance_l.pcap, pBuffer_p->pBuffer, (int)pBuffer_p->txFrameSize);
    if  (iRet != 0)
    {
        DEBUG_LVL_ERROR_TRACE("%s pcap_sendpacket returned %d (%s)\n", __func__, iRet, pcap_geterr(edrInstance_l.pcap));
        ret = kErrorInvalidOperation;
    }
    return ret;
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
    tOplkError ret = kErrorOk;

    if (pBuffer_p->maxBufferSize > EDRV_MAX_FRAME_SIZE)
    {
        return kErrorEdrvNoFreeBufEntry;
    }

    // allocate buffer with malloc
    pBuffer_p->pBuffer = OPLK_MALLOC(pBuffer_p->maxBufferSize);
    if (pBuffer_p->pBuffer == NULL)
    {
        return kErrorEdrvNoFreeBufEntry;
    }

    pBuffer_p->txBufferNumber.pArg = NULL;
    return ret;
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
    BYTE* pbBuffer = pBuffer_p->pBuffer;

    // mark buffer as free, before actually freeing it
    pBuffer_p->pBuffer = NULL;

    OPLK_FREE(pbBuffer);

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
tOplkError edrv_clearRxMulticastMacAddr(BYTE* pMacAddr_p)
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
tOplkError edrv_setRxMulticastMacAddr (BYTE* pMacAddr_p)
{
    UNUSED_PARAMETER(pMacAddr_p);
    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Return timer handle

Currently the edrv module creates the timer for the high-resolution timer
module. This is done to be able to use a single thread for handling timers
and Ethernet packets. This dependancy should be removed.

\param  index_p  Index in array.

\return The function returns a timer handle
*/
//------------------------------------------------------------------------------
HANDLE edrv_getTimerHandle(UINT index_p)
{

    index_p += EDRV_HANDLE_TIMER0;
    if (index_p > EDRV_HANDLE_COUNT)
    {
        return NULL;
    }
    else
    {
        return edrInstance_l.aHandle[index_p];
    }
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
    tEdrvRxBuffer   RxBuffer;

    if (OPLK_MEMCMP(pPktData_p + 6, pInstance->initParam.aMacAddr, 6 ) != 0)
    {   // filter out self generated traffic
        RxBuffer.bufferInFrame = kEdrvBufferLastInFrame;
        RxBuffer.rxFrameSize = pHeader_p->caplen;
        RxBuffer.pBuffer = (BYTE*)pPktData_p;

        pInstance->initParam.pfnRxHandler(&RxBuffer);
    }
    else
    {   // self generated traffic
        if (pInstance->pTransmittedTxBufferFirstEntry != NULL)
        {
            tEdrvTxBuffer* pTxBuffer = pInstance->pTransmittedTxBufferFirstEntry;

//            TRACE("%s: (%02X) first TxB=%p (%02X), last TxB=%p\n", __func__, (UINT)pkt_data[5], pTxBuffer, (UINT)pTxBuffer->pBuffer[5], edrInstance_l.pTransmittedTxBufferLastEntry);

            if (pTxBuffer->pBuffer != NULL)
            {
                if (OPLK_MEMCMP(pPktData_p, pTxBuffer->pBuffer, 6) == 0)
                {
                    EnterCriticalSection(&edrInstance_l.criticalSection);
                    pInstance->pTransmittedTxBufferFirstEntry = pInstance->pTransmittedTxBufferFirstEntry->txBufferNumber.pArg;
                    if (pInstance->pTransmittedTxBufferFirstEntry == NULL)
                    {
                        pInstance->pTransmittedTxBufferLastEntry = NULL;
                    }
                    LeaveCriticalSection(&edrInstance_l.criticalSection);

                    pTxBuffer->txBufferNumber.pArg = NULL;

                    if (pTxBuffer->pfnTxHandler != NULL)
                    {
                        pTxBuffer->pfnTxHandler(pTxBuffer);
                    }
                }
                else
                {
                    TRACE("%s: no matching TxB: DstMAC=%02X%02X%02X%02X%02X%02X\n",
                        __func__, (UINT)pPktData_p[0], (UINT)pPktData_p[1], (UINT)pPktData_p[2],
                                  (UINT)pPktData_p[3], (UINT)pPktData_p[4], (UINT)pPktData_p[5]);
                    TRACE("   current TxB %p: DstMAC=%02X%02X%02X%02X%02X%02X\n",
                        pTxBuffer, (UINT)pTxBuffer->pBuffer[0], (UINT)pTxBuffer->pBuffer[1], (UINT)pTxBuffer->pBuffer[2],
                                   (UINT)pTxBuffer->pBuffer[3], (UINT)pTxBuffer->pBuffer[4], (UINT)pTxBuffer->pBuffer[5]);
                }
            }
        }
        else
        {
            TRACE("%s: no TxB: DstMAC=%02X%02X%02X%02X%02X%02X\n", __func__,
                  pPktData_p[0], pPktData_p[1], pPktData_p[2], pPktData_p[3], pPktData_p[4], pPktData_p[5]);
        }
    }
}

//------------------------------------------------------------------------------
/**
\brief  Edrv worker thread

This function implements the edrv worker thread. It is responsible to handle
pcap and timer events.

\param  pArgument_p    Thread argument

\return The function returns a thread return code
*/
//------------------------------------------------------------------------------
static UINT32 WINAPI edrvWorkerThread(void* pArgument_p)
{
    tEdrvInstance*  pInstance = pArgument_p;
    int             pcapRet;
    UINT32          waitRet;

    // increase priority
    SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_TIME_CRITICAL);

    for (;;)
    {
        // Wait for events
        waitRet = WaitForMultipleObjects(EDRV_HANDLE_COUNT, pInstance->aHandle, FALSE, INFINITE);
        switch (waitRet)
        {
            case WAIT_OBJECT_0 + EDRV_HANDLE_EVENT:
            {   // shutdown was signalled
                return 0;
            }

            case WAIT_OBJECT_0 + EDRV_HANDLE_PCAP:
            {   // frames were received
                // process all frames, that are available
                pcapRet = pcap_dispatch(pInstance->pcap, -1, packetHandler, (u_char*)pInstance);
                break;
            }

            case WAIT_OBJECT_0 + EDRV_HANDLE_TIMER0:
            {   // timer 0 triggered
                hresTimerCb(0);
                break;
            }

            case WAIT_OBJECT_0 + EDRV_HANDLE_TIMER1:
            {   // timer 1 triggered
                hresTimerCb(1);
                break;
            }

            default:
            case WAIT_FAILED:
            {
                DEBUG_LVL_ERROR_TRACE("WaitForMultipleObjects failed (%d)\n", GetLastError());
                break;
            }
        }
    }
    return 0;
}

///\}

