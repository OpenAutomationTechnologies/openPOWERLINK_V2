/****************************************************************************

  (c) Kalycito Infotech Private Limited

  Project:      openPOWERLINK - on Windows

  Description:  source file for EdrvPcap.

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
#include "pcap.h"
#include "edrv.h"
#include <windows.h>
#include <iphlpapi.h>
#include <string.h>

//#include "kernel/EplDllk.h"

#ifndef EDRV_MAX_TX_BUFFERS
#define EDRV_MAX_TX_BUFFERS     20
#endif


#define EDRV_MAX_FRAME_SIZE     0x600
#define EDRV_TX_BUFFER_SIZE     (EDRV_MAX_TX_BUFFERS * EDRV_MAX_FRAME_SIZE) // n * (MTU + 14 + 4)
#define PCAP_SRC_IF_STRING "rpcap://"


// Private structure
typedef struct
{
//    BYTE*               m_pbRxBuf;      // pointer to Rx buffer
    BYTE*               m_pbTxBuf;      // pointer to Tx buffer
    BOOL                m_afTxBufUsed[EDRV_MAX_TX_BUFFERS];

    tEdrvInitParam      m_InitParam;
//    tEdrvTxBuffer*      m_pLastTransmittedTxBuffer;

} tEdrvInstance;


static tEdrvInstance EdrvInstance_l;

HANDLE  hThread_Handle = NULL; 

pcap_t *fp = NULL;

unsigned int  Stop_Recieve = 0;






CONST BYTE BroadCast[]= { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

void packet_handler(u_char *param, const struct pcap_pkthdr *header, const u_char *pkt_data);

DWORD WINAPI  Recieve_thread(void *);

tEplKernel EdrvInit(tEdrvInitParam *pEdrvInitParam_p)
{
tEplKernel Ret;
DWORD dwThreadId;
int i=0;
char sErr_Msg[ PCAP_ERRBUF_SIZE ];
pcap_if_t *alldevs;
pcap_if_t *seldev;
int inum;
// variables for IPHLPAPI
ULONG ulOutBufLen;
PIP_ADAPTER_INFO pAdapterInfo;
PIP_ADAPTER_INFO pAdapter = NULL;
DWORD dwRetVal = 0;

    Ret = kEplSuccessful; 


    // clear instance structure
    EPL_MEMSET(&EdrvInstance_l, 0, sizeof (EdrvInstance_l));

    // save the init data
    EdrvInstance_l.m_InitParam = *pEdrvInitParam_p;

    EdrvInstance_l.m_pbTxBuf = ( BYTE * )malloc( EDRV_TX_BUFFER_SIZE );

    if (EdrvInstance_l.m_pbTxBuf == NULL)
    {
        Ret = kEplEdrvInitError;
        goto Exit;
    }

    /* Retrieve the device list on the local machine */

    if (pcap_findalldevs(&alldevs, sErr_Msg) == -1)
    {
        fprintf(stderr,"Error in pcap_findalldevs: %s\n", sErr_Msg);
        Ret = kEplNoResource;
        goto Exit;
    }

    printf("--------------------------------------------------\n");
    printf("List of Ethernet Cards Found in this System: \n");
    printf("--------------------------------------------------\n");    
    /* Print the list */
    for (seldev=alldevs; seldev; seldev=seldev->next)
    {
        printf("%d. %s", ++i, seldev->name);
        
        if (seldev->description)
        {
            printf(" (%s)\n", seldev->description);
        }
        else
        {
            printf(" (No description available)\n");
        }
    }

    if (i==0)
    {
        printf("\nNo interfaces found! Make sure WinPcap is installed.\n");
        Ret = kEplNoResource;
        goto Exit;
    }
    printf("--------------------------------------------------\n");    
    printf("Enter the interface number (1-%d):",i);
    scanf("%d", &inum);
    printf("--------------------------------------------------\n");    
    if(inum < 1 || inum > i)
    {
        printf("\nInterface number out of range.\n");
        /* Free the device list */
        pcap_freealldevs(alldevs);
        Ret = kEplNoResource;
        goto Exit;
    }  
    
    /* Jump to the selected adapter */
    for (seldev=alldevs, i=0; i< inum-1 ;seldev=seldev->next, i++)
    {   // do nothing
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

    dwRetVal = GetAdaptersInfo(pAdapterInfo, &ulOutBufLen)
    if (dwRetVal == NO_ERROR)
    {
        pAdapter = pAdapterInfo;
        while (pAdapter)
        {
            if (pAdapter->Type == MIB_IF_TYPE_ETHERNET)
            {
                if (strstr(seldev->name, pAdapter->AdapterName) != NULL)
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


    fp = pcap_open_live (
                        seldev->name,  
                        500,  // snaplen
                        1,     //  8
                        1,      //milli seconds timeout
                        sErr_Msg   
                    ); 

    if ( fp == NULL )
    {
        printf("Error!!Can't open pcap\n");
        printf("%s",sErr_Msg);
        Ret = kEplEdrvInitError;
        goto Exit;
    }
    
    /* At this point, we don't need any more the device list. Free it */
    pcap_freealldevs(alldevs);
    
    // Create the thread to begin execution on its own.

    hThread_Handle = CreateThread(
            NULL,                   // default security attributes
            0,                      // use default stack size  
            Recieve_thread,         // thread function name
            NULL,          // argument to thread function 
            0,                      // use default creation flags 
            &dwThreadId);   // returns the thread identifier 


    if (hThread_Handle == NULL) 
    {
         Ret = kEplEdrvInitError;
         goto Exit;
    }

Exit:
    return Ret;
}


tEplKernel EdrvShutdown( void )
{
    pcap_close(fp);
    
    Stop_Recieve = 1;    

    WaitForSingleObject( hThread_Handle, INFINITE );

    CloseHandle ( hThread_Handle );

    return kEplSuccessful; //assuming no problems with closing the handle
}


tEplKernel EdrvSendTxMsg(tEdrvTxBuffer *pBuffer_p)
{
    tEplKernel Ret = kEplSuccessful;
    tEplFrame      *pTxFrame = NULL;
    unsigned int uiBufferNumber;

    uiBufferNumber = pBuffer_p->m_uiBufferNumber;

    if ((uiBufferNumber >= EDRV_MAX_TX_BUFFERS)
        || (EdrvInstance_l.m_afTxBufUsed[uiBufferNumber] == FALSE))
    {
        Ret = kEplEdrvBufNotExisting;
        goto Exit;
    }

    if  (pcap_sendpacket(fp, pBuffer_p->m_pbBuffer, (int) pBuffer_p->m_uiTxMsgLen) != 0)
    {
        Ret = kEplInvalidOperation;
    }
    else 
    {
        Ret = kEplSuccessful;
    }
    
    if (EdrvInstance_l.m_InitParam.m_pfnTxHandler != NULL)
    {
        EdrvInstance_l.m_InitParam.m_pfnTxHandler(pBuffer_p);
    }

Exit:
    return Ret;
}




tEplKernel EdrvAllocTxMsgBuffer(tEdrvTxBuffer * pBuffer_p)
{

    tEplKernel Ret = kEplSuccessful;
    DWORD i;

    if (pBuffer_p->m_uiMaxBufferLen > EDRV_MAX_FRAME_SIZE)
    {
        Ret = kEplEdrvNoFreeBufEntry;
        goto Exit;
    }

    // search a free Tx buffer with appropriate size
    for (i = 0; i < EDRV_MAX_TX_BUFFERS; i++)
    {
        if (EdrvInstance_l.m_afTxBufUsed[i] == FALSE)
        {
            // free channel found
            EdrvInstance_l.m_afTxBufUsed[i] = TRUE;
            pBuffer_p->m_uiBufferNumber = i;
            pBuffer_p->m_pbBuffer = EdrvInstance_l.m_pbTxBuf + (i * EDRV_MAX_FRAME_SIZE);
            pBuffer_p->m_uiMaxBufferLen = EDRV_MAX_FRAME_SIZE;
            break;
        }
    }
    if (i >= EDRV_MAX_TX_BUFFERS)
    {
        Ret = kEplEdrvNoFreeBufEntry;
        goto Exit;
    }

Exit:
    return Ret;

}



tEplKernel EdrvReleaseTxMsgBuffer(tEdrvTxBuffer * pBuffer_p)
{
    unsigned int uiBufferNumber;

    uiBufferNumber = pBuffer_p->m_uiBufferNumber;

    if (uiBufferNumber < EDRV_MAX_TX_BUFFERS)
    {
        EdrvInstance_l.m_afTxBufUsed[uiBufferNumber] = FALSE;
    }

    return kEplSuccessful;
}



tEplKernel EdrvUndefineRxMacAddrEntry (BYTE * pbMacAddr_p)
{
  
      return kEplSuccessful;
}

tEplKernel EdrvDefineRxMacAddrEntry   (BYTE * pbMacAddr_p)
{
     return kEplSuccessful;
}


void packet_handler(u_char *param, const struct pcap_pkthdr *header, const u_char *pkt_data)
{
    int Data_Cnt = 0;

    static threadcount = 0;

    tEdrvRxBuffer RxBuffer = { 0 };

    if (threadcount == 0)
    {
        threadcount++;
    }

    RxBuffer.m_BufferInFrame    = kEdrvBufferLastInFrame;
    RxBuffer.m_uiRxMsgLen        = header->caplen;
    RxBuffer.m_pbBuffer         = pkt_data;
    
    if ( (  memcmp( pkt_data+6, abMacAddr, 6 ) != 0 ) && ( memcmp( pkt_data, BroadCast, 6 ) != 0 ) )
    {
        EdrvInstance_l.m_InitParam.m_pfnRxHandler(&RxBuffer);
    }
}

DWORD WINAPI  Recieve_thread( void *pArgument )
{


    int Ret;    

    static threadcount = 0;

    if (threadcount == 0)
    {
        threadcount++;
    }

    while( 1 )
    {
    
        Ret = pcap_loop(fp,1,packet_handler,NULL);

    }
    
    return 0;
}
