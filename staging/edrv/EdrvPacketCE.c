/****************************************************************************

  (c) Kalycito Infotech Private Limited

  Project:      openPOWERLINK - on Windows CE

  Description:  source file for Ethernet driver for PacketCE.

  License:

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    1. Redistributions of source code must retain the above copyright
       notice, this list of conditions and the following disclaimer.

    2. Redistributions in binary form must reproduce the above copyright
       notice, this list of conditions and the following disclaimer in the
       documentation and/or other materials provided with the distribution.

    3. Neither the name of Kalycito Infotech Private Limited nor the names of
       its contributors may be used to endorse or promote products derived
       from this software without prior written permission. For written
       permission, please contact info@kalycito.com.

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

#define _WINSOCKAPI_ // prevent windows.h from including winsock.h
#include <winsock.h>
#include <ndis.h>
#include "Packet32.h"
#include "edrv.h"
#include <string.h>
#include <iphlpapi.h>

#include "kernel/EplTimerHighResk.h"

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

static LPADAPTER m_lpAdapter;
static PACKET m_RxPacket;
static WCHAR m_pRxBuffer[RX_BUFFER_SIZE]={0};
static volatile BOOL g_bRun = TRUE;

//---------------------------------------------------------------------------
// local function prototypes
//---------------------------------------------------------------------------

static void EplTimerHighReskCbTimer (unsigned int uiIndex_p);

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

#define EDRV_HANDLE_EVENT       0
#define EDRV_HANDLE_PCAP        1
#define EDRV_HANDLE_TIMER0      2
#define EDRV_HANDLE_TIMER1      3
#define EDRV_HANDLE_COUNT       4


//---------------------------------------------------------------------------
// local types
//---------------------------------------------------------------------------

// Private structure
typedef struct
{

    tEdrvInitParam      m_InitParam;
    tEdrvTxBuffer*      m_pTransmittedTxBufferLastEntry;
    tEdrvTxBuffer*      m_pTransmittedTxBufferFirstEntry;
    CRITICAL_SECTION    m_CriticalSection;
    HANDLE              m_ahHandle[EDRV_HANDLE_COUNT];
    HANDLE              m_hThread;
    HANDLE              m_hEdrvRead;
    HANDLE              m_hEdrvWrite;

} tEdrvInstance;


//---------------------------------------------------------------------------
// local vars
//---------------------------------------------------------------------------

static tEdrvInstance EdrvInstance_l;

//---------------------------------------------------------------------------
// local function prototypes
//---------------------------------------------------------------------------
static void EdrvPacketHandler(unsigned char *pUser_p, const unsigned char *pkt_data,
                              unsigned int uiRxDataSize);
static DWORD WINAPI  EdrvWorkerThread(void *);


//---------------------------------------------------------------------------
//
// Function:    EdrvInit
//
// Description: function for init of the Ethernet controller
//
// Parameters:  pEdrvInitParam_p    = pointer to struct including the init-parameters
//
// Returns:     Errorcode           = kEplSuccessful
//                                  = kEplNoResource
//
// State:
//
//---------------------------------------------------------------------------

tEplKernel EdrvInit(tEdrvInitParam *pEdrvInitParam_p)
{
tEplKernel Ret;
DWORD dwThreadId;
DWORD dwFilter;
// variables for IPHLPAPI
ULONG ulOutBufLen;
PIP_ADAPTER_INFO pAdapterInfo;
PIP_ADAPTER_INFO pAdapter = NULL;
DWORD dwRetVal = 0;
Ret = kEplSuccessful;
    // clear instance structure
    EPL_MEMSET(&EdrvInstance_l, 0, sizeof (EdrvInstance_l));
    
    if (pEdrvInitParam_p->m_HwParam.m_pszDevName == NULL)
    {
        Ret = kEplEdrvInitError;
        goto Exit;
    }
    
    // search for the corresponding MAC address via IPHLPAPI
    ulOutBufLen = sizeof (IP_ADAPTER_INFO);
    pAdapterInfo = (IP_ADAPTER_INFO *) EPL_MALLOC(sizeof (IP_ADAPTER_INFO));
    if (pAdapterInfo == NULL)
    {
        printf("Error allocating memory needed to call GetAdaptersinfo\n");
        Ret = kEplNoResource;
        goto Exit;
    }
    
    // Make an initial call to GetAdaptersInfo to get
    // the necessary size into the ulOutBufLen variable
    dwRetVal = GetAdaptersInfo(pAdapterInfo, &ulOutBufLen);
    if (dwRetVal == ERROR_BUFFER_OVERFLOW)
    {
        EPL_FREE(pAdapterInfo);
        pAdapterInfo = (IP_ADAPTER_INFO *) EPL_MALLOC(ulOutBufLen);
        if (pAdapterInfo == NULL)
        {
            printf("Error allocating memory needed to call GetAdaptersinfo\n");
            Ret = kEplNoResource;
            goto Exit;
        }
    }

    dwRetVal = GetAdaptersInfo(pAdapterInfo, &ulOutBufLen);
    if (dwRetVal == NO_ERROR)
    {
        pAdapter = pAdapterInfo;
        while (pAdapter)
        {
            if (pAdapter->Type == MIB_IF_TYPE_ETHERNET)
            {
                if (strstr(pEdrvInitParam_p->m_HwParam.m_pszDevName, pAdapter->AdapterName) != NULL)
                {   // corresponding adapter found
                    EPL_MEMCPY(pEdrvInitParam_p->m_abMyMacAddr, pAdapter->Address,
                        min(pAdapter->AddressLength, sizeof (pEdrvInitParam_p->m_abMyMacAddr)));
                    break;
                }
            }
            pAdapter = pAdapter->Next;
        }
    }
    else
    {
        printf("GetAdaptersInfo failed with error: %d\n", dwRetVal);

    }
    if (pAdapterInfo)
    {
        EPL_FREE(pAdapterInfo);
    }

    // save the init data (with updated MAC address)
    EdrvInstance_l.m_InitParam = *pEdrvInitParam_p;

    m_lpAdapter = PacketOpenAdapter((LPTSTR)pEdrvInitParam_p->m_HwParam.m_pszDevName);
    if (m_lpAdapter == NULL)
    {
        OutputDebugString( L"PKT ADATER NAMES: OPEN ADAPTER FAILED\n" );
        PacketSetLastError (GetLastError ());
        Ret = kEplEdrvInitError;
        goto Exit;
    }

    // Create an handle to read packtes from the driver
    EdrvInstance_l.m_hEdrvRead = CreateFile (L"PKT1:", GENERIC_READ, 0, 
        NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
    
    if (EdrvInstance_l.m_hEdrvRead == INVALID_HANDLE_VALUE) 
    {
        OutputDebugString( L"PKT ADATER NAMES: CREATE FILE FAILED\n" );
        PacketSetLastError (GetLastError ());
        Ret = kEplEdrvInitError;
        goto Exit;
    }

    // Create an handle to write packtes to the driver
    EdrvInstance_l.m_hEdrvWrite = CreateFile (L"PKT1:", GENERIC_WRITE, 0, 
    NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
    
    if (EdrvInstance_l.m_hEdrvWrite == INVALID_HANDLE_VALUE) {
        OutputDebugString( L"PKT ADATER NAMES: CREATE FILE FAILED\n" );
        PacketSetLastError (GetLastError ());
        Ret = kEplEdrvInitError;
        goto Exit;
    }
    
    // get event handle for pcap instance
    EdrvInstance_l.m_ahHandle[EDRV_HANDLE_PCAP] = m_lpAdapter->ReadEvent;
    // Create two unnamed waitable timers for EplTimerHighResk sub-module.
    EdrvInstance_l.m_ahHandle[EDRV_HANDLE_TIMER0] =
        CreateEvent(NULL, FALSE, FALSE, NULL);
    if (EdrvInstance_l.m_ahHandle[EDRV_HANDLE_TIMER0] == NULL)
    {
        printf("CreateWaitableTimer failed (%d)\n", GetLastError());
        Ret = kEplEdrvInitError;
        goto Exit;
    }
    EdrvInstance_l.m_ahHandle[EDRV_HANDLE_TIMER1] =
        CreateEvent(NULL, FALSE, FALSE, NULL);
    if (EdrvInstance_l.m_ahHandle[EDRV_HANDLE_TIMER1] == NULL)
    {
        printf("CreateWaitableTimer failed (%d)\n", GetLastError());
        Ret = kEplEdrvInitError;
        goto Exit;
    }
    // create event for signalling shutdown
    EdrvInstance_l.m_ahHandle[EDRV_HANDLE_EVENT] =
        CreateEvent(NULL, FALSE, FALSE, NULL);
    
    // Filter type 
    dwFilter = NDIS_PACKET_TYPE_PROMISCUOUS;
    
    if (! PacketSetHwFilter (m_lpAdapter, dwFilter)) 
    {
            printf("HW filter fails");
            PacketCloseAdapter (m_lpAdapter);
            CloseHandle(EdrvInstance_l.m_hEdrvRead);
            CloseHandle(EdrvInstance_l.m_hEdrvWrite);
            Ret = kEplEdrvInitError;
            goto Exit;
    }

    // init the packet structure
    m_RxPacket.Buffer                = m_pRxBuffer;
    m_RxPacket.Length                = 256 * 1024;
    m_RxPacket.ulBytesReceived        = 0;
    m_RxPacket.bIoComplete            = FALSE;

    // set a 64k buffer in the driver
    //  PacketSetBuff (m_lpAdapter, 64 * 1024);

    // set number or write counter to one
    PacketSetNumWrites (m_lpAdapter, 1);

    // set the read time out to 1ms sec
    PacketSetReadTimeout (m_lpAdapter, 1);

    // configure pcap for maximum responsiveness
    PacketSetMinToCopy(m_lpAdapter, 0);

    // Create the thread to begin execution on its own.
    EdrvInstance_l.m_hThread = CreateThread(
            NULL,                   // default security attributes
            0,                      // use default stack size
            EdrvWorkerThread,         // thread function name
            &EdrvInstance_l,        // argument to thread function
            0,                      // use default creation flags
            &dwThreadId);   // returns the thread identifier


    if (EdrvInstance_l.m_hThread == NULL)
    {
         Ret = kEplEdrvInitError;
         goto Exit;
    }

    InitializeCriticalSection(&EdrvInstance_l.m_CriticalSection);

Exit:
    return Ret;
}


//---------------------------------------------------------------------------
//
// Function:    EdrvShutdown
//
// Description: Shutdown the Ethernet controller
//
// Parameters:  void
//
// Returns:     Errorcode   = kEplSuccessful
//
// State:
//
//---------------------------------------------------------------------------

tEplKernel EdrvShutdown( void )
{
    // signal shutdown to the thread
    SetEvent(EdrvInstance_l.m_ahHandle[EDRV_HANDLE_EVENT]);

    WaitForSingleObject( EdrvInstance_l.m_hThread, INFINITE );

    CloseHandle ( EdrvInstance_l.m_hThread );

    CloseHandle ( EdrvInstance_l.m_ahHandle[EDRV_HANDLE_EVENT] );
    CloseHandle ( EdrvInstance_l.m_ahHandle[EDRV_HANDLE_TIMER0] );
    CloseHandle ( EdrvInstance_l.m_ahHandle[EDRV_HANDLE_TIMER1] );
    
    DeleteCriticalSection(&EdrvInstance_l.m_CriticalSection);

    PacketCloseAdapter (m_lpAdapter);
    CloseHandle(EdrvInstance_l.m_hEdrvRead);
    CloseHandle(EdrvInstance_l.m_hEdrvWrite);

    // clear instance structure
    EPL_MEMSET(&EdrvInstance_l, 0, sizeof (EdrvInstance_l));


    return kEplSuccessful; //assuming no problems with closing the handle
}


//---------------------------------------------------------------------------
//
// Function:    EdrvSendTxMsg
//
// Description: immediately starts the transmission of the buffer
//
// Parameters:  pBuffer_p   = buffer descriptor to transmit
//
// Returns:     Errorcode   = kEplSuccessful
//
// State:
//
//---------------------------------------------------------------------------

tEplKernel EdrvSendTxMsg(tEdrvTxBuffer *pBuffer_p)
{
tEplKernel Ret = kEplSuccessful;
PACKET m_TxPacket;
int         iRet;

//    TRACE("%s: TxB=%p (%02X), last TxB=%p\n", __func__, pBuffer_p, (UINT)pBuffer_p->m_pbBuffer[5], EdrvInstance_l.m_pTransmittedTxBufferLastEntry);

    if (pBuffer_p->m_BufferNumber.m_pVal != NULL)
    {
		printf("Invalid operation\n");
        Ret = kEplInvalidOperation;
        goto Exit;
    }

    EnterCriticalSection(&EdrvInstance_l.m_CriticalSection);
    if (EdrvInstance_l.m_pTransmittedTxBufferLastEntry == NULL)
    {
        EdrvInstance_l.m_pTransmittedTxBufferLastEntry =
            EdrvInstance_l.m_pTransmittedTxBufferFirstEntry = pBuffer_p;
    }
    else
    {
        EdrvInstance_l.m_pTransmittedTxBufferLastEntry->m_BufferNumber.m_pVal = pBuffer_p;
        EdrvInstance_l.m_pTransmittedTxBufferLastEntry = pBuffer_p;
    }
    LeaveCriticalSection(&EdrvInstance_l.m_CriticalSection);

    m_TxPacket.Buffer = pBuffer_p->m_pbBuffer;
    m_TxPacket.Length = pBuffer_p->m_uiTxMsgLen;

	iRet = PacketSendPacket(m_lpAdapter,&m_TxPacket,0); 
    if(!iRet)
    {
        PRINTF("%s PacketSendPacket returned %d (%d)\n", __func__, iRet, PacketGetLastError());
        Ret = kEplInvalidOperation;
    }

Exit:
    return Ret;
}



//---------------------------------------------------------------------------
//
// Function:    EdrvAllocTxMsgBuffer
//
// Description: Register a Tx-Buffer
//
// Parameters:  pBuffer_p   = pointer to Buffer structure
//
// Returns:     Errorcode   = kEplSuccessful
//                          = kEplEdrvNoFreeBufEntry
//
// State:
//
//---------------------------------------------------------------------------

tEplKernel EdrvAllocTxMsgBuffer(tEdrvTxBuffer * pBuffer_p)
{
tEplKernel Ret = kEplSuccessful;

    if (pBuffer_p->m_uiMaxBufferLen > EDRV_MAX_FRAME_SIZE)
    {
        Ret = kEplEdrvNoFreeBufEntry;
        goto Exit;
    }

    // allocate buffer with malloc
    pBuffer_p->m_pbBuffer = EPL_MALLOC(pBuffer_p->m_uiMaxBufferLen);
    if (pBuffer_p->m_pbBuffer == NULL)
    {
        Ret = kEplEdrvNoFreeBufEntry;
        goto Exit;
    }

    pBuffer_p->m_BufferNumber.m_pVal = NULL;

Exit:
    return Ret;

}



//---------------------------------------------------------------------------
//
// Function:    EdrvReleaseTxMsgBuffer
//
// Description: Register a Tx-Buffer
//
// Parameters:  pBuffer_p   = pointer to Buffer structure
//
// Returns:     Errorcode   = kEplSuccessful
//
// State:
//
//---------------------------------------------------------------------------

tEplKernel EdrvReleaseTxMsgBuffer(tEdrvTxBuffer * pBuffer_p)
{
BYTE*   pbBuffer = pBuffer_p->m_pbBuffer;

    // mark buffer as free, before actually freeing it
    pBuffer_p->m_pbBuffer = NULL;

    EPL_FREE(pbBuffer);

    return kEplSuccessful;
}


tEplKernel EdrvChangeFilter(tEdrvFilter*    pFilter_p,
                            unsigned int    uiCount_p,
                            unsigned int    uiEntryChanged_p,
                            unsigned int    uiChangeFlags_p)
{
tEplKernel      Ret = kEplSuccessful;

    UNUSED_PARAMETER(pFilter_p);
    UNUSED_PARAMETER(uiCount_p);
    UNUSED_PARAMETER(uiEntryChanged_p);
    UNUSED_PARAMETER(uiChangeFlags_p);
    return Ret;
}


//---------------------------------------------------------------------------
//
// Function:    EdrvUndefineRxMacAddrEntry
//
// Description: Reset a multicast entry in the Ethernet controller
//
// Parameters:  pbMacAddr_p     = pointer to multicast entry to reset
//
// Returns:     Errorcode       = kEplSuccessful
//
// State:
//
//---------------------------------------------------------------------------

tEplKernel EdrvUndefineRxMacAddrEntry (BYTE * pbMacAddr_p)
{
    UNUSED_PARAMETER(pbMacAddr_p);

    return kEplSuccessful;
}


//---------------------------------------------------------------------------
//
// Function:    EdrvDefineRxMacAddrEntry
//
// Description: Set a multicast entry into the Ethernet controller
//
// Parameters:  pbMacAddr_p     = pointer to multicast entry to set
//
// Returns:     Errorcode       = kEplSuccessful
//
// State:
//
//---------------------------------------------------------------------------

tEplKernel EdrvDefineRxMacAddrEntry   (BYTE * pbMacAddr_p)
{
    UNUSED_PARAMETER(pbMacAddr_p);

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
//
// State:
//
//---------------------------------------------------------------------------

static void EdrvPacketHandler(unsigned char *pUser_p, const unsigned char *pkt_data,
                              unsigned int uiRxDataSize)
{
tEdrvInstance*  pInstance = (tEdrvInstance*) pUser_p;
tEdrvRxBuffer   RxBuffer;

    if (memcmp( pkt_data+6, pInstance->m_InitParam.m_abMyMacAddr, 6 ) != 0)
    {   // filter out self generated traffic
        RxBuffer.m_BufferInFrame    = kEdrvBufferLastInFrame;
        RxBuffer.m_uiRxMsgLen       = uiRxDataSize;
        RxBuffer.m_pbBuffer         = (BYTE*) pkt_data;

        pInstance->m_InitParam.m_pfnRxHandler(&RxBuffer);
    }
    else
    {   // self generated traffic
        if (pInstance->m_pTransmittedTxBufferFirstEntry != NULL)
        {
        tEdrvTxBuffer* pTxBuffer = pInstance->m_pTransmittedTxBufferFirstEntry;

//            TRACE("%s: (%02X) first TxB=%p (%02X), last TxB=%p\n", __func__, (UINT)pkt_data[5], pTxBuffer, (UINT)pTxBuffer->m_pbBuffer[5], EdrvInstance_l.m_pTransmittedTxBufferLastEntry);

            if (memcmp(pkt_data, pTxBuffer->m_pbBuffer, 6) == 0)
            {
                EnterCriticalSection(&EdrvInstance_l.m_CriticalSection);
                pInstance->m_pTransmittedTxBufferFirstEntry = pInstance->m_pTransmittedTxBufferFirstEntry->m_BufferNumber.m_pVal;
                if (pInstance->m_pTransmittedTxBufferFirstEntry == NULL)
                {
                    pInstance->m_pTransmittedTxBufferLastEntry = NULL;
                }
                LeaveCriticalSection(&EdrvInstance_l.m_CriticalSection);

                pTxBuffer->m_BufferNumber.m_pVal = NULL;

                if (pTxBuffer->m_pfnTxHandler != NULL)
                {
                    pTxBuffer->m_pfnTxHandler(pTxBuffer);
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
                    pTxBuffer,
                    (UINT)pTxBuffer->m_pbBuffer[0],
                    (UINT)pTxBuffer->m_pbBuffer[1],
                    (UINT)pTxBuffer->m_pbBuffer[2],
                    (UINT)pTxBuffer->m_pbBuffer[3],
                    (UINT)pTxBuffer->m_pbBuffer[4],
                    (UINT)pTxBuffer->m_pbBuffer[5]);
            }
        }
        else
        {
            TRACE("%s: no TxB: DstMAC=%02X%02X%02X%02X%02X%02X\n", __func__, pkt_data[0], pkt_data[1], pkt_data[2], pkt_data[3], pkt_data[4], pkt_data[5]);
        }
    }
}



//---------------------------------------------------------------------------
//
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
//
// State:
//
//---------------------------------------------------------------------------

static DWORD WINAPI EdrvWorkerThread( void *pArgument_p )
{
tEdrvInstance*  pInstance = pArgument_p;
int             iRet;
DWORD           dwRet;

    // increase priority
    //SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_TIME_CRITICAL);
	if( !CeSetThreadPriority(	GetCurrentThread(),
								CE_THREAD_PRIO_256_HIGHEST	) )
	{
		printf("Error!! EdrvWorkerThread CeSetThreadPriority failed\n");
	}

	printf("INFO: EdrvWorkerThread Priority: %d %d\n", CeGetThreadPriority(GetCurrentThread()), CE_THREAD_PRIO_256_HIGHEST);

    for (;;)
    {
        // Wait for events
        dwRet = WaitForMultipleObjects(
                    EDRV_HANDLE_COUNT,
                    pInstance->m_ahHandle,
                    FALSE,
                    INFINITE);

        switch (dwRet)
        {
            case WAIT_OBJECT_0 + EDRV_HANDLE_EVENT:
            {   // shutdown was signalled
                goto Exit;
            }

            case WAIT_OBJECT_0 + EDRV_HANDLE_PCAP:
            {   
                // frames were received
                if (PacketReceivePacket (m_lpAdapter, &m_RxPacket, TRUE) == FALSE)    // last parameter is ignored
                {
                    printf("APPL:Packet read error\n");
                    iRet = -1;
                    break;
                }

                // print packets
                {
                    UINT    ulBytesReceived;
                    char    *RxBuffer;
                    UINT    offset = 0;
                    UINT    tlen1;
                    struct    bpf_hdr *hdr;
                
                
                    ulBytesReceived = m_RxPacket.ulBytesReceived;
                    RxBuffer = (char*)m_RxPacket.Buffer;    

                    //
                    while (offset < ulBytesReceived) 
                    {    
                        hdr = (struct bpf_hdr *) (RxBuffer + offset);
                        tlen1 = hdr->bh_datalen;
                        offset += hdr->bh_hdrlen;
                        EdrvPacketHandler((unsigned char *)pInstance, RxBuffer+offset,
                                          hdr->bh_datalen);
                        offset = PACKET_WORDALIGN(offset + tlen1);
                    }    
                }

       
                break;
            }

            case WAIT_OBJECT_0 + EDRV_HANDLE_TIMER0:
            {   // timer 0 triggered

                EplTimerHighReskCbTimer(0);

                break;
            }

            case WAIT_OBJECT_0 + EDRV_HANDLE_TIMER1:
            {   // timer 1 triggered

                EplTimerHighReskCbTimer(1);

                break;
            }

            default:
            case WAIT_FAILED:
            {
                printf("WaitForMultipleObjects failed (%d)\n", GetLastError());
                break;
            }
        }
    }

Exit:
    return 0;
}


static HANDLE EdrvGetTimerHandle(unsigned int uiIndex_p)
{

    uiIndex_p += EDRV_HANDLE_TIMER0;
    if (uiIndex_p > EDRV_HANDLE_COUNT)
    {
        return NULL;
    }

/*
   uiIndex_p = (EDRV_HANDLE_COUNT - 1) - uiIndex_p;

   if (uiIndex_p < EDRV_HANDLE_TIMER0)
   {
       return NULL;
   }
*/
    else
    {
        return EdrvInstance_l.m_ahHandle[uiIndex_p];
    }
}



/***************************************************************************/
/*                                                                         */
/*                                                                         */
/*          C L A S S  <EplTimerHighResk>                                  */
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

#define TIMER_COUNT                 2
#define TIMERHDL_MASK               0x0FFFFFFF
#define TIMERHDL_SHIFT              28


//---------------------------------------------------------------------------
// local types
//---------------------------------------------------------------------------

typedef struct
{
    tEplTimerEventArg   m_EventArg;
    tEplTimerkCallback  m_pfnCallback;
    LARGE_INTEGER        m_liDueTime;
    UINT                m_uiTimerId;

} tEplTimerHighReskTimerInfo;

typedef struct
{
    tEplTimerHighReskTimerInfo  m_aTimerInfo[TIMER_COUNT];
    HINSTANCE                   m_hinstLibNtDll;

} tEplTimerHighReskInstance;


//// function types from NTDLL.DLL
//typedef LONG (NTAPI *NTQUERYTIMERRESOLUTION)
//    (OUT PULONG MinimumResolution,
//     OUT PULONG MaximumResolution,
//     OUT PULONG CurrentResolution);
//typedef LONG (NTAPI *NTSETTIMERRESOLUTION)
//    (IN ULONG DesiredResolution,
//     IN BOOLEAN SetResolution,
//     OUT PULONG CurrentResolution);

//---------------------------------------------------------------------------
// local vars
//---------------------------------------------------------------------------

static tEplTimerHighReskInstance    EplTimerHighReskInstance_l;


// function pointers to NTDLL.DLL
//NTQUERYTIMERRESOLUTION NtQueryTimerResolution;
//NTSETTIMERRESOLUTION NtSetTimerResolution;


//---------------------------------------------------------------------------
// local function prototypes
//---------------------------------------------------------------------------


//=========================================================================//
//                                                                         //
//          P U B L I C   F U N C T I O N S                                //
//                                                                         //
//=========================================================================//


//---------------------------------------------------------------------------
//
// Function:    EplTimerHighReskInit()
//
// Description: initializes the high resolution timer module.
//
// Parameters:  void
//
// Return:      tEplKernel      = error code
//
// State:       not tested
//
//---------------------------------------------------------------------------

tEplKernel PUBLIC EplTimerHighReskInit(void)
{
tEplKernel  Ret;

    Ret = EplTimerHighReskAddInstance();

    return Ret;

}


//---------------------------------------------------------------------------
//
// Function:    EplTimerHighReskAddInstance()
//
// Description: initializes the high resolution timer module.
//
// Parameters:  void
//
// Return:      tEplKernel      = error code
//
// State:       not tested
//
//---------------------------------------------------------------------------

tEplKernel PUBLIC EplTimerHighReskAddInstance(void)
{
tEplKernel      Ret = kEplSuccessful;
LONG            lRet = 0;
ULONG           ulMin = ~0;
ULONG           ulMax = ~0;
ULONG           ulCur = ~0;


    //EPL_MEMSET(&EplTimerHighReskInstance_l, 0, sizeof (EplTimerHighReskInstance_l));


    //// load NTDLL.DLL
    //EplTimerHighReskInstance_l.m_hinstLibNtDll = LoadLibrary("ntdll.dll");
    //if (EplTimerHighReskInstance_l.m_hinstLibNtDll == NULL)
    //{
    //    printf("LoadLibrary(ntdll.dll) failed (%d)\n", GetLastError());
    //    Ret = kEplNoResource;
    //    goto Exit;
    //}

    //// load proc address of NtQueryTimerResolution
    //NtQueryTimerResolution = (NTQUERYTIMERRESOLUTION)
    //    GetProcAddress(EplTimerHighReskInstance_l.m_hinstLibNtDll,
    //                    "NtQueryTimerResolution");
    //if (NtQueryTimerResolution == NULL)
    //{
    //    printf("GetProcAddress(NtQueryTimerResolution) failed (%d)\n", GetLastError());
    //    Ret = kEplNoResource;
    //    goto Exit;
    //}

    //// load proc address of NtSetTimerResolution
    //NtSetTimerResolution = (NTSETTIMERRESOLUTION)
    //    GetProcAddress(EplTimerHighReskInstance_l.m_hinstLibNtDll,
    //                    "NtSetTimerResolution");
    //if (NtSetTimerResolution == NULL)
    //{
    //    printf("GetProcAddress(NtSetTimerResolution) failed (%d)\n", GetLastError());
    //    Ret = kEplNoResource;
    //    goto Exit;
    //}


    //// query actual timer resolution
    //NtQueryTimerResolution(&ulMin, &ulMax, &ulCur);
    //printf("TimerResolution Min = %lu, Max = %lu, Cur = %lu\n", ulMin, ulMax, ulCur);

    //// set timer resolution to maximum
    //lRet = NtSetTimerResolution(ulMax, TRUE, &ulCur);
    //printf("NtSetTimerResolution returnd %ld, current resolution = %lu\n", lRet, ulCur);


//Exit:
    return Ret;

}


//---------------------------------------------------------------------------
//
// Function:    EplTimerHighReskDelInstance()
//
// Description: shuts down the high resolution timer module.
//
// Parameters:  void
//
// Return:      tEplKernel      = error code
//
// State:       not tested
//
//---------------------------------------------------------------------------

tEplKernel PUBLIC EplTimerHighReskDelInstance(void)
{
tEplKernel  Ret = kEplSuccessful;
LONG            lRet = 0;
ULONG           ulCur = ~0;

    //// set timer resolution to old value
    //lRet = NtSetTimerResolution(0, FALSE, &ulCur);
    //printf("NtSetTimerResolution returnd %ld, current resolution = %lu\n", lRet, ulCur);

    //// free library NTDLL.DLL
    //FreeLibrary(EplTimerHighReskInstance_l.m_hinstLibNtDll);
    //EplTimerHighReskInstance_l.m_hinstLibNtDll = NULL;

    return Ret;

}



//---------------------------------------------------------------------------
//
// Function:    EplTimerHighReskModifyTimerNs()
//
// Description: modifies the timeout of the timer with the specified handle.
//              If the handle the pointer points to is zero, the timer must
//              be created first.
//              If it is not possible to stop the old timer,
//              this function always assures that the old timer does not
//              trigger the callback function with the same handle as the new
//              timer. That means the callback function must check the passed
//              handle with the one returned by this function. If these are
//              unequal, the call can be discarded.
//
// Parameters:  pTimerHdl_p     = pointer to timer handle
//              ullTimeNs_p     = relative timeout in [ns]
//              pfnCallback_p   = callback function, which is called mutual
//                                exclusive with the Edrv callback functions
//                                (Rx and Tx).
//              ulArgument_p    = user-specific argument
//              fContinuously_p = if TRUE, callback function will be called
//                                continuously;
//                                otherwise, it is a oneshot timer.
//
// Return:      tEplKernel      = error code
//
// State:       not tested
//
//---------------------------------------------------------------------------

tEplKernel PUBLIC EplTimerHighReskModifyTimerNs(tEplTimerHdl*     pTimerHdl_p,
                                    unsigned long long ullTimeNs_p,
                                    tEplTimerkCallback  pfnCallback_p,
                                    unsigned long       ulArgument_p,
                                    BOOL                fContinuously_p)
{
tEplKernel                  Ret = kEplSuccessful;
unsigned int                uiIndex;
tEplTimerHighReskTimerInfo* pTimerInfo;
HANDLE                      hTimer;
unsigned int                uiTimeMs_p = 0;
MMRESULT                    mTempTimerHdl = 0;

        // check pointer to handle
        if(pTimerHdl_p == NULL)
        {
            Ret = kEplTimerInvalidHandle;
            goto Exit;
        }
        
        if( ullTimeNs_p > (long long)1000000 )
        {
        //Convert nano seconds into milli seconds 
        //Windows Timer accepts only milli secs time delays
            uiTimeMs_p = (UINT ) (ullTimeNs_p /(long long)1000000);
        }
        else
        {
        //We create timer with 1 milli second delay
        //If we get ullTimeNs_p with 0ns (OR) the above condition fails
            uiTimeMs_p = 1;
        }
        
         if (*pTimerHdl_p == 0)
        {   // no timer created yet
            // search free timer info structure
            pTimerInfo = &EplTimerHighReskInstance_l.m_aTimerInfo[0];
            for (uiIndex = 0; uiIndex < TIMER_COUNT; uiIndex++, pTimerInfo++)
            {
                if (pTimerInfo->m_pfnCallback == NULL)
                {   // free structure found
                    break;
                }
            }
            if (uiIndex >= TIMER_COUNT)
            {   // no free structure found
				printf("check 1\n");
                Ret = kEplTimerNoTimerCreated;
                goto Exit;
            }
        }
        else
        {
        uiIndex = (unsigned int)(*pTimerHdl_p >> TIMERHDL_SHIFT) - 1;
        if (uiIndex >= TIMER_COUNT)
            {   // invalid handle
                Ret = kEplTimerInvalidHandle;
                goto Exit;
            }
            pTimerInfo = &EplTimerHighReskInstance_l.m_aTimerInfo[uiIndex];
            // d.k.: assume that this info structure is the correct one
    /*        if ((pTimerInfo->m_EventArg.m_TimerHdl != *pTimerHdl_p)
                && (pTimerInfo->m_pfnCallback == NULL))
            {   // invalid handle
                Ret = kEplTimerInvalidHandle;
                goto Exit;
            }*/
        }
        
       // increment timer handle (if timer expires right after this statement,
       // the user would detect an unknown timer handle and discard it)
       pTimerInfo->m_EventArg.m_TimerHdl = ((pTimerInfo->m_EventArg.m_TimerHdl + 1) & TIMERHDL_MASK)
                                        | ((uiIndex + 1) << TIMERHDL_SHIFT);

       if (fContinuously_p != FALSE)
        {   // continuous timer
            pTimerInfo->m_liDueTime.QuadPart = uiTimeMs_p;
        }
        else
        {   // one-shot timer
            pTimerInfo->m_liDueTime.QuadPart = 0LL;
        }

    pTimerInfo->m_EventArg.m_Arg.m_dwVal = ulArgument_p;
    pTimerInfo->m_pfnCallback = pfnCallback_p;

        *pTimerHdl_p = pTimerInfo->m_EventArg.m_TimerHdl;

        // configure timer
        hTimer = EdrvGetTimerHandle(uiIndex);

        pTimerInfo->m_uiTimerId = timeSetEvent(uiTimeMs_p,1,hTimer,0,TIME_CALLBACK_EVENT_SET);

        if (!pTimerInfo->m_uiTimerId)
        {
            printf("timeSetEvent failed (%d)\n", GetLastError());
            Ret = kEplTimerNoTimerCreated;
			printf("check 2\n");
            goto Exit;
        }

Exit:
    return Ret;

}


//---------------------------------------------------------------------------
//
// Function:    EplTimerHighReskDeleteTimer()
//
// Description: deletes the timer with the specified handle. Afterward the
//              handle is set to zero.
//
// Parameters:  pTimerHdl_p     = pointer to timer handle
//
// Return:      tEplKernel      = error code
//
// State:       not tested
//
//---------------------------------------------------------------------------

tEplKernel PUBLIC EplTimerHighReskDeleteTimer(tEplTimerHdl*     pTimerHdl_p)
{
tEplKernel                  Ret = kEplSuccessful;
unsigned int                uiIndex;
tEplTimerHighReskTimerInfo* pTimerInfo;
//HANDLE                      hTimer;
MMRESULT                        mResult;

    // check pointer to handle
    if(pTimerHdl_p == NULL)
    {
        Ret = kEplTimerInvalidHandle;
        goto Exit;
    }

    if (*pTimerHdl_p == 0)
    {   // no timer created yet
         goto Exit;
    }
    else
    {
        uiIndex = (unsigned int)(*pTimerHdl_p >> TIMERHDL_SHIFT) - 1;
        if (uiIndex >= TIMER_COUNT)
        {   // invalid handle
            Ret = kEplTimerInvalidHandle;
            goto Exit;
        }
        pTimerInfo = &EplTimerHighReskInstance_l.m_aTimerInfo[uiIndex];
        if (pTimerInfo->m_EventArg.m_TimerHdl != *pTimerHdl_p)
        {   // invalid handle
            goto Exit;
        }
    }

    pTimerInfo->m_pfnCallback = NULL;

 //   *pTimerHdl_p = 0;

    // cancel timer
    //hTimer = EdrvGetTimerHandle(uiIndex);

    mResult = timeKillEvent(pTimerInfo->m_uiTimerId);

    //printf("Delete Timer: 0x%X\n", mResult);

    *pTimerHdl_p = 0;

Exit:
    return Ret;

}


//---------------------------------------------------------------------------
//
// Function:    EplTimerHighReskCbTimer()
//
// Description: target specific callback function for timer.
//
// Parameters:  uiIndex_p       = timer index (0 or 1)
//
// Return:      void
//
// State:       not tested
//
//---------------------------------------------------------------------------

static void EplTimerHighReskCbTimer (unsigned int uiIndex_p)
{
tEplTimerHighReskTimerInfo* pTimerInfo;

    if (uiIndex_p > TIMER_COUNT)
    {   // invalid handle
        goto Exit;
    }

    pTimerInfo = &EplTimerHighReskInstance_l.m_aTimerInfo[uiIndex_p];

    if (pTimerInfo->m_liDueTime.QuadPart != 0)
    {   // periodic timer
    HANDLE  hTimer;
    unsigned int uiTimeMs_p;

        // configure timer
        hTimer = EdrvGetTimerHandle(uiIndex_p);

        uiTimeMs_p = (UINT)pTimerInfo->m_liDueTime.QuadPart;

        pTimerInfo->m_uiTimerId = timeSetEvent(uiTimeMs_p,1,hTimer,0,TIME_CALLBACK_EVENT_SET);
        if (!pTimerInfo->m_uiTimerId)
        {
            printf("timeSetEvent failed (%d)\n", GetLastError());
            goto Exit;
        }
    }

    if (pTimerInfo->m_pfnCallback != NULL)
    {
        pTimerInfo->m_pfnCallback(&pTimerInfo->m_EventArg);
    }

Exit:
    return;
}



