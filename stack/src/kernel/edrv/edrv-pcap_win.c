/**
********************************************************************************
\file   edrv-pcap_win.c

\brief  Implementation of WinPCAP Ethernet driver

This file contains the implementation of the WinPCAP Ethernet driver.

\ingroup module_edrv
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2013, Kalycito Infotech Private Limited
Copyright (c) 2015, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
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
#include <kernel/edrv.h>

#define WPCAP           // include Windows pcap extensions
#include <pcap.h>
#include <iphlpapi.h>

//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#define EDRV_MAX_FRAME_SIZE     0x600

#define EDRV_HANDLE_EVENT       0
#define EDRV_HANDLE_PCAP        1
#define EDRV_HANDLE_COUNT       2

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
\brief Structure describing an instance of the Edrv

This structure describes an instance of the Ethernet driver.
*/
typedef struct
{
    tEdrvInitParam      initParam;                          ///< Init parameters
    tEdrvTxBuffer*      pTransmittedTxBufferLastEntry;      ///< Pointer to the last entry of the transmitted TX buffer
    tEdrvTxBuffer*      pTransmittedTxBufferFirstEntry;     ///< Pointer to the first entry of the transmitted Tx buffer
    CRITICAL_SECTION    criticalSection;                    ///< Critical section locking variable
    pcap_t*             pPcap;                              ///< Pointer to the pcap interface instance
    HANDLE              aHandle[EDRV_HANDLE_COUNT];         ///< Array of event handles of the pcap interface
    HANDLE              hThread;                            ///< Handle of the worker thread
} tEdrvInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tEdrvInstance edrvInstance_l;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static void packetHandler(u_char* pParam_p, const struct pcap_pkthdr* pHeader_p, const u_char* pPktData_p);
static DWORD WINAPI edrvWorkerThread(LPVOID pArgument_p);

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
    DWORD               threadId;
    char                sErr_Msg[PCAP_ERRBUF_SIZE];
    // variables for IPHLPAPI
    ULONG               outBufLen;
    PIP_ADAPTER_INFO    pAdapterInfo;
    PIP_ADAPTER_INFO    pAdapter = NULL;
    UINT32              retVal = 0;

    // clear instance structure
    OPLK_MEMSET(&edrvInstance_l, 0, sizeof(edrvInstance_l));

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
    edrvInstance_l.initParam = *pEdrvInitParam_p;
    edrvInstance_l.pPcap = pcap_open_live(pEdrvInitParam_p->hwParam.pDevName,
                                          65535, 1, 1, sErr_Msg);
    if (edrvInstance_l.pPcap == NULL)
    {
        DEBUG_LVL_ERROR_TRACE("Error!! Can't open pcap: %s\n", sErr_Msg);
        return kErrorEdrvInit;
    }

    // configure pcap for maximum responsiveness
    if (pcap_setmintocopy(edrvInstance_l.pPcap, 0) != 0)
    {
        DEBUG_LVL_ERROR_TRACE("pcap_setmintocopy failed\n");
    }

    // put pcap into nonblocking mode
    if (pcap_setnonblock(edrvInstance_l.pPcap, 1, sErr_Msg) != 0)
    {
        DEBUG_LVL_ERROR_TRACE("Can't put pcap into nonblocking mode: %s\n", sErr_Msg);
    }

    // get event handle for pcap instance
    edrvInstance_l.aHandle[EDRV_HANDLE_PCAP] = pcap_getevent(edrvInstance_l.pPcap);

    // create event for signalling shutdown
    edrvInstance_l.aHandle[EDRV_HANDLE_EVENT] = CreateEvent(NULL, FALSE, FALSE, NULL);

    // Create the thread to begin execution on its own.
    edrvInstance_l.hThread = CreateThread(NULL, 0, edrvWorkerThread, &edrvInstance_l, 0, &threadId);
    if (edrvInstance_l.hThread == NULL)
         return kErrorEdrvInit;

    InitializeCriticalSection(&edrvInstance_l.criticalSection);
    return ret;
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
    // signal shutdown to the thread
    SetEvent(edrvInstance_l.aHandle[EDRV_HANDLE_EVENT]);

    WaitForSingleObject(edrvInstance_l.hThread, INFINITE);

    CloseHandle(edrvInstance_l.hThread);

    pcap_close(edrvInstance_l.pPcap);

    CloseHandle(edrvInstance_l.aHandle[EDRV_HANDLE_EVENT]);

    DeleteCriticalSection(&edrvInstance_l.criticalSection);

    // clear instance structure
    OPLK_MEMSET(&edrvInstance_l, 0, sizeof(edrvInstance_l));

    return kErrorOk; //assuming no problems with closing the handle
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
    tOplkError  ret = kErrorOk;
    int         iRet;

    //TRACE("%s: TxB=%p (%02X), last TxB=%p\n", __func__, pBuffer_p, (UINT)pBuffer_p->pBuffer[5], edrvInstance_l.pTransmittedTxBufferLastEntry);

    if (pBuffer_p->txBufferNumber.pArg != NULL)
    {
        return kErrorInvalidOperation;
    }

    EnterCriticalSection(&edrvInstance_l.criticalSection);
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
    LeaveCriticalSection(&edrvInstance_l.criticalSection);

    iRet = pcap_sendpacket(edrvInstance_l.pPcap, pBuffer_p->pBuffer, (int)pBuffer_p->txFrameSize);
    if (iRet != 0)
    {
        DEBUG_LVL_ERROR_TRACE("%s pcap_sendpacket returned %d (%s)\n", __func__, iRet, pcap_geterr(edrvInstance_l.pPcap));
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
    pBuffer_p->pBuffer = (UINT8*)OPLK_MALLOC(pBuffer_p->maxBufferSize);
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

    if (OPLK_MEMCMP(pPktData_p + 6, pInstance->initParam.aMacAddr, 6) != 0)
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

//            TRACE("%s: (%02X) first TxB=%p (%02X), last TxB=%p\n", __func__, (UINT)pkt_data[5], pTxBuffer, (UINT)pTxBuffer->pBuffer[5], edrvInstance_l.pTransmittedTxBufferLastEntry);

            if (pTxBuffer->pBuffer != NULL)
            {
                if (OPLK_MEMCMP(pPktData_p, pTxBuffer->pBuffer, 6) == 0)
                {
                    EnterCriticalSection(&edrvInstance_l.criticalSection);
                    pInstance->pTransmittedTxBufferFirstEntry = (tEdrvTxBuffer*)pInstance->pTransmittedTxBufferFirstEntry->txBufferNumber.pArg;
                    if (pInstance->pTransmittedTxBufferFirstEntry == NULL)
                    {
                        pInstance->pTransmittedTxBufferLastEntry = NULL;
                    }
                    LeaveCriticalSection(&edrvInstance_l.criticalSection);

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
static DWORD WINAPI edrvWorkerThread(LPVOID pArgument_p)
{
    tEdrvInstance*  pInstance = (tEdrvInstance*)pArgument_p;
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
                pcapRet = pcap_dispatch(pInstance->pPcap, -1, packetHandler, (u_char*)pInstance);
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

/// \}
