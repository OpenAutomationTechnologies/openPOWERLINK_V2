#include <windows.h>
#include <windowsx.h>
#include <winsock.h>
#include <ndis.h>
#include "Packet32.h"

//---------------------------------------------------------------------------
// Global device handle
HANDLE	g_hDriver = NULL;
HANDLE	g_hActiveDriver = NULL;
static DWORD g_nW32NLastError = ERROR_SUCCESS;



//---------------------------------------------------------------------------
//
// Function name	: PacketGetLastError
// Description	    : Returns last error code
// Return type		: DWORD 
// Argument         : VOID
//
//---------------------------------------------------------------------------
DWORD PacketGetLastError (VOID)
{
   DWORD nLastError = g_nW32NLastError;

   g_nW32NLastError = ERROR_SUCCESS;

   return (nLastError);
}



//---------------------------------------------------------------------------
//
// Function name	: PacketSetLastError
// Description	    : Sets a new value of error code
// Return type		: DWORD 
// Argument         : DWORD nNewError
//
//---------------------------------------------------------------------------
DWORD PacketSetLastError (DWORD nNewError)
{
   DWORD nLastError = g_nW32NLastError;

   g_nW32NLastError = nNewError;

   return (nLastError);
}


//---------------------------------------------------------------------------
//
// Function name	: DllMain 
// Description	    : Main Dll entry
// Return type		: BOOL APIENTRY - Success or failure
// Argument         : HANDLE hModule - Handle of the module
// Argument         : DWORD dwReason - reason for dll loading
// Argument         : LPVOID lpReserved - reserved
//
//---------------------------------------------------------------------------
//BOOL APIENTRY DllMain (HANDLE hModule, DWORD dwReason, LPVOID lpReserved)
//{
//    BOOL bRet;
//
//    switch (dwReason)
//	{
//		case DLL_PROCESS_ATTACH:
//			bRet = TRUE;
//			break;
//		case DLL_PROCESS_DETACH:
//			bRet = TRUE;
//			break;
//		default:
//			bRet = TRUE;
//			break;
//	}
//
//	return bRet;
//}
//



 
//---------------------------------------------------------------------------
//
// Function name	: PacketReceivePacket 
// Description	    : Reads a packet from device
// Return type		: BOOL - Success or failure of operation
// Argument         : LPADAPTER lpAdapter	- Adapter object
// Argument         : LPPACKET lpPacket			- Packet buffer
// Argument         : BOOL Sync					- Synchronous or Asynchronous operation
//
//---------------------------------------------------------------------------
BOOLEAN PacketReceivePacket(LPADAPTER lpAdapter,LPPACKET lpPacket,BOOLEAN Sync)
{
	BOOL	bResult;

	// parameter checking
	if (lpAdapter == NULL) {
		PacketSetLastError ((DWORD)INVALID_HANDLE_VALUE);
		return FALSE;
	}

	if (lpPacket == NULL) {
		PacketSetLastError ((DWORD)INVALID_HANDLE_VALUE);
		return FALSE;
	}

	bResult = DeviceIoControl (lpAdapter->hFile, PIOC_READ_PACKET, lpPacket->Buffer, 
		lpPacket->Length, lpPacket->Buffer, lpPacket->Length, (LPDWORD) &(lpPacket->ulBytesReceived), NULL);


	PacketSetLastError (GetLastError ());

	lpPacket->bIoComplete = bResult;
	return bResult;

}




//---------------------------------------------------------------------------
//
// Function name	: PacketSetNumWrites
// Description	    : Sets number of times to write the packet
// Return type		: BOOL - Success or failure
// Argument         : LPADAPTER lpAdapter	- Adapter object
// Argument         : int nwrites				- Count of number of writes
//
//---------------------------------------------------------------------------
BOOLEAN PacketSetNumWrites(LPADAPTER lpAdapter,int nWrites)
{
	if (NULL == lpAdapter) {
		PacketSetLastError ((DWORD)INVALID_HANDLE_VALUE);
		return FALSE;
	}

	if (nWrites < 0) {
		return FALSE;
	}

	lpAdapter->nWrites = nWrites;

	PacketSetLastError (ERROR_SUCCESS);

	return TRUE;
}


//---------------------------------------------------------------------------
//
// Function name	: PacketSendPacket 
// Description	    : Sends a packet to driver
// Return type		: BOOL - Success or failure
// Argument         : LPADAPTER lpAdapter	- Adapter object
// Argument         : LPPACKET lpPacket			- Packet to send
// Argument         : BOOL Sync					- Synchronous or asynchronous 
//					:							  operation
//
//---------------------------------------------------------------------------
BOOLEAN PacketSendPacket(LPADAPTER lpAdapter,LPPACKET lpPacket,BOOLEAN Sync)
{
	BOOL	bResult;

	PacketSetLastError (ERROR_SUCCESS);

	// parameter checking
	if (NULL == lpAdapter) {
		PacketSetLastError ((DWORD)INVALID_HANDLE_VALUE);
		return FALSE;
	}

	if (NULL == lpPacket) {
		PacketSetLastError ((DWORD)INVALID_HANDLE_VALUE);
		return FALSE;
	}
	
	// send the packet
	bResult = DeviceIoControl (lpAdapter->hFile, PIOC_WRITE_PACKET, lpPacket->Buffer, 
		lpPacket->Length, lpPacket->Buffer, lpPacket->Length, (LPDWORD) &(lpPacket->ulBytesReceived), NULL);

	lpPacket->bIoComplete = bResult;
	if (bResult == FALSE) {
		PacketSetLastError (GetLastError ());
		return FALSE;
	}

	return TRUE;
}



//---------------------------------------------------------------------------
//
// Function name	: PacketSetBpfProgram
// Description	    : Sets a new BPF structure
// Return type		: BOOL - Success or failure of operation
// Argument         : LPADAPTER lpAdapter	- Adapter object
// Argument         : struct bpf_program *fp	- new BPF program
//
//---------------------------------------------------------------------------
BOOLEAN PacketSetBpf(LPADAPTER lpAdapter,struct bpf_program *fp)
{
	int		BytesReturned;
	BOOL	bResult;
	
	// parameter checking
	if (NULL == lpAdapter) {
		PacketSetLastError ((DWORD)INVALID_HANDLE_VALUE);
		return FALSE;
	}

	if (NULL == fp) {
		PacketSetLastError ((DWORD)INVALID_HANDLE_VALUE);
		return FALSE;
	}

    bResult = DeviceIoControl (lpAdapter->hFile, PIOC_SETF, (char*)fp->bf_insns, 
		fp->bf_len * sizeof (struct bpf_insn), NULL, 0, (LPDWORD) &BytesReturned,NULL);

	PacketSetLastError (GetLastError ());
	return bResult;
}



//---------------------------------------------------------------------------
//
// Function name	: PacketSetBuff
// Description	    : Sets a new value of buffer size
// Return type		: BOOL - Success or failure of operation
// Argument         : LPADAPTER lpAdapter	- Adapter object
// Argument         : int dim	- Count of the buffer
//
//---------------------------------------------------------------------------
BOOLEAN PacketSetBuff(LPADAPTER lpAdapter,int dim)
{
	int		BytesReturned;
	BOOL	bResult;
	
	// parameter checking
	if (lpAdapter == NULL) {
		PacketSetLastError ((DWORD)INVALID_HANDLE_VALUE);
		return FALSE;
	}
	
	if (dim <= 0) {
		return FALSE;
	}

	// send the IOCTL
    bResult = DeviceIoControl (lpAdapter->hFile, PIOC_SET_BUFFER_SIZE, (LPVOID) &dim, sizeof(int), NULL, 
		0, (LPDWORD) &BytesReturned, NULL);

	PacketSetLastError (GetLastError ());

	return bResult;
}




//---------------------------------------------------------------------------
//
// Function name	: PacketSetReadTimeout
// Description	    : Sets a new value of read timeout
// Return type		: BOOL - Success or failure of operation
// Argument         : LPADAPTER lpAdapter	- Adapter object
// Argument         : int timeout				- new value of the read timeout
//
//---------------------------------------------------------------------------
BOOLEAN PacketSetReadTimeout(LPADAPTER lpAdapter,int timeout)
{
	int		BytesReturned;
	BOOL	bResult;

	// parameter checking
	if (lpAdapter == NULL) {
		PacketSetLastError ((DWORD)INVALID_HANDLE_VALUE);
		return FALSE;
	}

    bResult = DeviceIoControl (lpAdapter->hFile, PIOC_SRTIMEOUT, &timeout, 4, 
		NULL, 0, (LPDWORD) &BytesReturned, NULL);

	PacketSetLastError (GetLastError ());

	return bResult;
}




//---------------------------------------------------------------------------
//
// Function name	: PacketCloseAdapter  
// Description	    : Closes the adapter
// Return type		: VOID 
// Argument         : LPADAPTER lpAdapter - Adapter handle to close
//
//---------------------------------------------------------------------------
VOID PacketCloseAdapter  (LPADAPTER lpAdapter)
{
	DWORD dwRet = 0;

	// parameter checking
	if (lpAdapter == NULL) {
		PacketSetLastError ((DWORD)INVALID_HANDLE_VALUE);
		return;
	}

	// send IOCTL to command driver to close
	if (!DeviceIoControl (lpAdapter->hFile, PIOC_CLOSE_ADAPTER, NULL, 0, 
		NULL, 0, &dwRet, NULL)) {
		PacketSetLastError (GetLastError ());
	}

	// close the handles
	if (! CloseHandle (lpAdapter->hFile)) { 
		PacketSetLastError (GetLastError ());
	}

	SetEvent (lpAdapter->ReadEvent);

	if (! CloseHandle (lpAdapter->ReadEvent)) {
		PacketSetLastError (GetLastError ());
	}

	HeapFree (GetProcessHeap (), 0, lpAdapter);
	lpAdapter = NULL;
}


//---------------------------------------------------------------------------
//
// Function name	: PacketRequest
// Description	    : Makes a new request to NIC driver
// Return type		: BOOL - Success or failure of operation
// Argument         : LPADAPTER  lpAdapter - Adapter object
// Argument         : BOOL Set	
// Argument         : PPACKET_OID_DATA  pOidData 
//
//---------------------------------------------------------------------------
BOOLEAN PacketRequest(LPADAPTER  lpAdapter,BOOLEAN Set,PPACKET_OID_DATA  pOidData)
{
    UINT       BytesReturned=0;
    BOOLEAN    Result=FALSE;
	
    Result=DeviceIoControl(lpAdapter->hFile,
		(DWORD) Set ? PIOC_SETOID : PIOC_QUERYOID,
		pOidData, sizeof(PACKET_OID_DATA) - 1 + pOidData->Length,
		pOidData, sizeof(PACKET_OID_DATA) - 1 + pOidData->Length,
		(LPDWORD)&BytesReturned, NULL);

    if(!Result) {

		switch (pOidData->Length) {	

		case NDIS_STATUS_SUCCESS:
			pOidData->Length =101;
			break;
		case NDIS_STATUS_PENDING :
			pOidData->Length =102;
			break;
		case NDIS_STATUS_NOT_RECOGNIZED:
			pOidData->Length =103;
			break;
		case NDIS_STATUS_NOT_COPIED:
			pOidData->Length =104;
			break;
		case NDIS_STATUS_NOT_ACCEPTED:
			pOidData->Length =105;
			break;
		case NDIS_STATUS_CALL_ACTIVE:
			pOidData->Length =106;
			break;

		case NDIS_STATUS_ONLINE:
			pOidData->Length =107;
			break;
		case NDIS_STATUS_RESET_START:
			pOidData->Length =108;
			break;
		case NDIS_STATUS_RESET_END:
			pOidData->Length =109;
			break;
		case NDIS_STATUS_RING_STATUS:
			pOidData->Length =110;
			break;
		case NDIS_STATUS_CLOSED:
			pOidData->Length =111;
			break;
		case NDIS_STATUS_WAN_LINE_UP:
			pOidData->Length =112;
			break;
		case NDIS_STATUS_WAN_LINE_DOWN:
			pOidData->Length =113;
			break;
		case NDIS_STATUS_WAN_FRAGMENT:
			pOidData->Length =114;
			break;
		case NDIS_STATUS_MEDIA_CONNECT:
			pOidData->Length =115;
			break;
		case NDIS_STATUS_MEDIA_DISCONNECT:
			pOidData->Length =116;
			break;
		case NDIS_STATUS_HARDWARE_LINE_UP:
			pOidData->Length =117;
			break;
		case NDIS_STATUS_HARDWARE_LINE_DOWN:
			pOidData->Length =118;
			break;
		case NDIS_STATUS_INTERFACE_UP:
			pOidData->Length =119;
			break;
		case NDIS_STATUS_INTERFACE_DOWN:
			pOidData->Length =120;
			break;
		case NDIS_STATUS_MEDIA_BUSY:
			pOidData->Length =121;
			break;

		case NDIS_STATUS_NOT_RESETTABLE:
			pOidData->Length =122;
			break;
		case NDIS_STATUS_SOFT_ERRORS:
			pOidData->Length =123;
			break;
		case NDIS_STATUS_HARD_ERRORS:
			pOidData->Length =124;
			break;
		case NDIS_STATUS_BUFFER_OVERFLOW:
			pOidData->Length =125;
			break;

		case NDIS_STATUS_FAILURE:
			pOidData->Length =126;
			break;
		case NDIS_STATUS_RESOURCES:
			pOidData->Length =127;
			break;
		case NDIS_STATUS_NOT_SUPPORTED:
			pOidData->Length =128;
			break;
		case NDIS_STATUS_CLOSING:
			pOidData->Length =129;
			break;
		case NDIS_STATUS_BAD_VERSION:
			pOidData->Length =130;
			break;
		case NDIS_STATUS_BAD_CHARACTERISTICS:
			pOidData->Length =131;
			break;
		case NDIS_STATUS_ADAPTER_NOT_FOUND:
			pOidData->Length =132;
			break;
		case NDIS_STATUS_OPEN_FAILED :
			pOidData->Length =133;
			break;
		case NDIS_STATUS_DEVICE_FAILED :
			pOidData->Length =134;
			break;
		case NDIS_STATUS_MULTICAST_FULL:
			pOidData->Length =135;
			break;
		case NDIS_STATUS_MULTICAST_EXISTS:
			pOidData->Length =136;
			break;
		case NDIS_STATUS_MULTICAST_NOT_FOUND:
			pOidData->Length =137;
			break;
		case NDIS_STATUS_REQUEST_ABORTED:
			pOidData->Length =138;
			break;
		case NDIS_STATUS_RESET_IN_PROGRESS:
			pOidData->Length =139;
			break;
		case NDIS_STATUS_CLOSING_INDICATING:
			pOidData->Length =140;
			break;
		case NDIS_STATUS_INVALID_PACKET:
			pOidData->Length =141;
			break;
		case NDIS_STATUS_OPEN_LIST_FULL:
			pOidData->Length =142;
			break;
		case NDIS_STATUS_ADAPTER_NOT_READY:
			pOidData->Length =143;
			break;
		case NDIS_STATUS_ADAPTER_NOT_OPEN:
			pOidData->Length =144;
			break;
		case NDIS_STATUS_NOT_INDICATING:
			pOidData->Length =145;
			break;
		case NDIS_STATUS_INVALID_LENGTH:
			//pOidData->Length =pOI->Request.DATA.QUERY_INFORMATION.BytesNeeded;
			pOidData->Length =146;
			break;
		case NDIS_STATUS_INVALID_DATA:
			pOidData->Length =147;
			break;
		case NDIS_STATUS_BUFFER_TOO_SHORT:
			pOidData->Length =148;
			break;
		case NDIS_STATUS_INVALID_OID:
			pOidData->Length =149;
			break;
		case NDIS_STATUS_ADAPTER_REMOVED:
			pOidData->Length =150;
			break;
		case NDIS_STATUS_UNSUPPORTED_MEDIA:
			pOidData->Length =151;
			break;
		case NDIS_STATUS_GROUP_ADDRESS_IN_USE:
			pOidData->Length =152;
			break;
		case NDIS_STATUS_FILE_NOT_FOUND:
			pOidData->Length =153;
			break;
		case NDIS_STATUS_ERROR_READING_FILE:
			pOidData->Length =154;
			break;
		case NDIS_STATUS_ALREADY_MAPPED:
			pOidData->Length =156;
			break;
		case NDIS_STATUS_RESOURCE_CONFLICT:
			pOidData->Length =157;
			break;
		case NDIS_STATUS_NO_CABLE:
			pOidData->Length =158;
			break;

		case NDIS_STATUS_INVALID_SAP:
			pOidData->Length =159;
			break;
		case NDIS_STATUS_SAP_IN_USE:
			pOidData->Length =160;
			break;
		case NDIS_STATUS_INVALID_ADDRESS:
			pOidData->Length =161;
			break;
		case NDIS_STATUS_VC_NOT_ACTIVATED:
			pOidData->Length =162;
			break;
		case NDIS_STATUS_DEST_OUT_OF_ORDER:
			pOidData->Length =163;
			break;
		case NDIS_STATUS_VC_NOT_AVAILABLE:
			pOidData->Length =164;
			break;
		case NDIS_STATUS_CELLRATE_NOT_AVAILABLE:
			pOidData->Length =165;
			break;
		case NDIS_STATUS_INCOMPATABLE_QOS:
			pOidData->Length =166;
			break;
		case NDIS_STATUS_AAL_PARAMS_UNSUPPORTED :
			pOidData->Length =167;
			break;
		case NDIS_STATUS_NO_ROUTE_TO_DESTINATION :
			pOidData->Length =168;
			break;
		case NDIS_STATUS_TOKEN_RING_OPEN_ERROR :
			pOidData->Length =169;
			break;
		default:
			pOidData->Length = 0;
			//ASSERT(FALSE);
		}
		OutputDebugString (L"Error in NDISRequest..!!!");
	}
	return Result;
}



//---------------------------------------------------------------------------
//
// Function name	: PacketResetAdapter 
// Description	    : Resets the adapter
// Return type		: BOOL - Success or failure of the operation
// Argument         : LPADAPTER lpAdapter - Adapter object
//
//---------------------------------------------------------------------------
BOOLEAN PacketResetAdapter(LPADAPTER lpAdapter)
{
    UINT	BytesReturned;
	BOOL	bResult;

	// check parameter
	if (NULL == lpAdapter) {
		PacketSetLastError ((DWORD)INVALID_HANDLE_VALUE);
		return FALSE;
	}

	// send IOCTL
	bResult = DeviceIoControl (lpAdapter->hFile, (DWORD) PIOC_RESET, NULL,
		0, NULL, 0, (LPDWORD) &BytesReturned, NULL);
	
	PacketSetLastError (GetLastError ());

	return bResult;
}




//---------------------------------------------------------------------------
//
// Function name	: PacketGetNetInfo
// Description	    : Gets the network info
// Return type		: BOOL - Success or failure of operation
// Argument         : LPTSTR AdapterName - Adapter object
// Argument         : PULONG netp
// Argument         : PULONG maskp
//
//---------------------------------------------------------------------------
BOOLEAN PacketGetNetInfo(LPTSTR AdapterName, PULONG netp, PULONG maskp)
{
	struct	hostent* h;
	char	szBuff[80];
	
	// check parameter
	if (NULL == AdapterName) {
		PacketSetLastError ((DWORD)INVALID_HANDLE_VALUE);
		return FALSE;
	}

	if (NULL == netp) {
		PacketSetLastError ((DWORD)INVALID_HANDLE_VALUE);
		return FALSE;
	}

	if (NULL == maskp) {
		PacketSetLastError ((DWORD)INVALID_HANDLE_VALUE);
		return FALSE;
	}

    if (gethostname (szBuff, 79)) {
		if (WSAGetLastError () == WSANOTINITIALISED) {
			WORD		wVersionRequested;
			WSADATA		wsaData;

			wVersionRequested = MAKEWORD (1, 1); 
			if (WSAStartup (wVersionRequested, &wsaData) != 0)
				return FALSE;

			if (gethostname (szBuff, 79)){
				return FALSE;
			}

			h=gethostbyname (szBuff);
			*netp=((h->h_addr_list[0][0]<<24))+
				((h->h_addr_list[0][1]<<16))+
				((h->h_addr_list[0][2]<<8))+
				((h->h_addr_list[0][3]));

			if (((*netp)&0x80000000) == 0) {
				*maskp = 0xFF000000;
			} else if (((*netp)&0xC0000000) == 0x80000000) {
				*maskp = 0xFFFF0000;
			} else if (((*netp)&0xE0000000) == 0xC0000000) {
				*maskp = 0xFFFFFF00;
			} else {
				return FALSE;
			}

			(*netp) &= *maskp;

			PacketSetLastError (ERROR_SUCCESS);

			return TRUE;
		} else {
			PacketSetLastError (GetLastError ());
			return FALSE;
		}
	}
	
	h = gethostbyname (szBuff);

	*netp = ((h->h_addr_list[0][0] << 24)) +
		((h->h_addr_list[0][1] << 16)) +
		((h->h_addr_list[0][2] << 8)) +
		((h->h_addr_list[0][3]));

	if (((*netp)&0x80000000) == 0) {
		*maskp = 0xFF000000;
	} else if (((*netp)&0xC0000000) == 0x80000000) {
		*maskp = 0xFFFF0000;
	} else if (((*netp)&0xE0000000) == 0xC0000000) {
		*maskp = 0xFFFFFF00;
	} else {
		return FALSE;
	}

	(*netp) &= *maskp;

	PacketSetLastError (GetLastError ());
	
	return TRUE;
	
}




//---------------------------------------------------------------------------
//
// Function name	: PacketGetNetType 
// Description	    : Gets the type of the network
// Return type		: BOOL - Success or failure of the operation
// Argument         : LPADAPTER lpAdapter - Adapter object
// Argument         : NetType *type - Structure to be filled with net type
//
//---------------------------------------------------------------------------
BOOLEAN PacketGetNetType (LPADAPTER lpAdapter,NetType *type)
{
	BOOLEAN				bRet;
    PPACKET_OID_DATA	pOidData;
    ULONG				IoCtlBufferLength = (sizeof (PACKET_OID_DATA) + sizeof (ULONG) - 1);

	// check parameter
	if (NULL == lpAdapter) {
		PacketSetLastError ((DWORD)INVALID_HANDLE_VALUE);
		return FALSE;
	}

	if (NULL == type) {
		PacketSetLastError ((DWORD)INVALID_HANDLE_VALUE);
		return FALSE;
	}

    pOidData = (PPACKET_OID_DATA) HeapAlloc (GetProcessHeap (), HEAP_ZERO_MEMORY, IoCtlBufferLength);
    if (pOidData == NULL) {
		PacketSetLastError (GetLastError ());
		return FALSE;
	}

	//get the link-layer type
    pOidData->Oid	= OID_GEN_MEDIA_IN_USE;
    pOidData->Length	= sizeof (ULONG);

    bRet = PacketRequest (lpAdapter, FALSE, pOidData);
	if (bRet == FALSE) {
		PacketSetLastError (GetLastError ());
		return FALSE;
	}

    type->LinkType = *((UINT*)pOidData->Data);

	//get the link-layer speed
    pOidData->Oid		= OID_GEN_LINK_SPEED;
    pOidData->Length		= sizeof (ULONG);

    bRet = PacketRequest (lpAdapter, FALSE, pOidData);
	type->LinkSpeed	= *((UINT*)pOidData->Data) * 100;
    
	HeapFree (GetProcessHeap (), 0, pOidData);
	
	PacketSetLastError (GetLastError ());

    return bRet;
}



//---------------------------------------------------------------------------
//
// Function name	: PacketSetMaxLookahead 
// Description	    : Sets maximum size of look ahead buffer
// Return type		: BOOL - Success or failure of operation
// Argument         : LPADAPTER lpAdapter - Adapter object
//
//---------------------------------------------------------------------------
BOOL PacketSetMaxLookahead (LPADAPTER lpAdapter)
{
    BOOLEAN				bRet;
    ULONG				IoCtlBufferLength=(sizeof (PACKET_OID_DATA)+sizeof (ULONG)-1);
    PPACKET_OID_DATA	pOidData;

	// check paramter
	if (NULL == lpAdapter) {
		PacketSetLastError ((DWORD)INVALID_HANDLE_VALUE);
		return FALSE;
	}

    pOidData = (PPACKET_OID_DATA) GlobalAllocPtr(GMEM_MOVEABLE | GMEM_ZEROINIT, IoCtlBufferLength);
    if (pOidData == NULL) {
		PacketSetLastError (GetLastError ());
		return FALSE;
    }

	//set the size of the lookahead buffer to the maximum available by the the NIC driver
    pOidData->Oid		= OID_GEN_MAXIMUM_LOOKAHEAD;
    pOidData->Length	= sizeof (ULONG);
    bRet				= PacketRequest (lpAdapter, FALSE, pOidData);

    pOidData->Oid		= OID_GEN_CURRENT_LOOKAHEAD;
    bRet				= PacketRequest (lpAdapter, TRUE, pOidData);

	if (! bRet) {
		PacketSetLastError (pOidData->Length);
	}

	GlobalFreePtr (pOidData);

    return bRet;
}




//---------------------------------------------------------------------------
//
// Function name	: PacketSetHwFilter 
// Description	    : Sets a new value of HW filter
// Return type		: BOOL - Success or failure of operation
// Argument         : LPADAPTER lpAdapter - Adapter object
// Argument         : ULONG Filter	- new filter value
//
//---------------------------------------------------------------------------
BOOLEAN PacketSetHwFilter(LPADAPTER lpAdapter,ULONG Filter)
{
	BOOLEAN				bRet;
    PPACKET_OID_DATA	pOidData;
    ULONG				IoCtlBufferLength = (sizeof (PACKET_OID_DATA) + sizeof (ULONG) - 1);

	// check parameter
	if (NULL == lpAdapter) {
		PacketSetLastError ((DWORD)INVALID_HANDLE_VALUE);
		return FALSE;
	}

    pOidData = (PPACKET_OID_DATA) GlobalAllocPtr(GMEM_MOVEABLE | GMEM_ZEROINIT, IoCtlBufferLength);
    if (pOidData == NULL) {
		PacketSetLastError (GetLastError ());
		return FALSE;
	}

    pOidData->Oid				= OID_GEN_CURRENT_PACKET_FILTER;
    pOidData->Length			= sizeof (ULONG);
    *((PULONG) pOidData->Data)	= Filter;
    bRet						= PacketRequest (lpAdapter, TRUE, pOidData);
	
	GlobalFreePtr (pOidData);

    return bRet;
}



//---------------------------------------------------------------------------
//
// Function name	: PacketSetMinToCopy
// Description	    : Sets minimum number of bytes to copy
// Return type		: BOOL - Success or failure of operation
// Argument         : LPADAPTER lpAdapter - Adapter object
// Argument         : int nbytes	- New value of Min Buffer
//
//---------------------------------------------------------------------------
BOOLEAN PacketSetMinToCopy(LPADAPTER lpAdapter,int nbytes)
{
	int		BytesReturned;
	BOOL	bResult;

	// check parameter
	if (NULL == lpAdapter) {
		PacketSetLastError ((DWORD)INVALID_HANDLE_VALUE);
		return FALSE;
	}

    bResult = DeviceIoControl (lpAdapter->hFile, PIOC_SMINTOCOPY, &nbytes, 4, NULL, 
			0, (LPDWORD)&BytesReturned, NULL);

	PacketSetLastError (GetLastError ());

	return bResult;

}




//---------------------------------------------------------------------------
//
// Function name	: PacketLoadDriver
// Description	    : Loads the protocol driver
// Return type		: BOOL - success or failure of open operation
// Argument         : VOID
//
//---------------------------------------------------------------------------
BOOLEAN	PacketLoadDriver(VOID)
{
DWORD dwLastError;

    // Register the packet driver
	
	OutputDebugString( L"About to RegisterDevice...\r\n" );
/*
	g_hDev = RegisterDevice (DRIVER_NAME, 0, DRIVER_DLL, DRIVER_INSTANCE);

	PacketSetLastError (GetLastError ());

	if (g_hDev == NULL) {
		OutputDebugString( L"RegisterDevice failed!\n" );
		return FALSE;
	}
*/
/*    
    g_hActiveDriver = ActivateDeviceEx(L"\\Drivers\\BuiltIn\\PktDrv", NULL, 0, NULL);
    if (g_hActiveDriver == INVALID_HANDLE_VALUE || g_hActiveDriver == NULL)
    {
        dwLastError = GetLastError();
        OutputDebugString( L"(PKT) -> Unable to load driver!\r\n" );
        return FALSE;
    }
    
    g_hDriver = CreateFile (L"PKT1:", GENERIC_READ | GENERIC_WRITE,
                                     FILE_SHARE_READ | FILE_SHARE_WRITE,
                                     NULL,
                                     OPEN_EXISTING,
                                     FILE_ATTRIBUTE_NORMAL,
                                     NULL);
    
    if (g_hDriver == INVALID_HANDLE_VALUE)
    { 
        dwLastError = GetLastError();
        ERRORMSG(1,(TEXT("(PKT) -> Unable to open PacketDrv (PKT) driver!\r\n")));
        return FALSE;
    }
*/	
    return TRUE;
}




//---------------------------------------------------------------------------
//
// Function name	: PacketUnloadDriver
// Description	    : Unloads the driver
// Return type		: BOOL 
// Argument         : VOID
//
//---------------------------------------------------------------------------
BOOL PacketUnloadDriver (VOID)
{
BOOL fRet;

    fRet = TRUE;
    /*
    // Unregister the packet driver
	if (g_hDev != NULL) {
		DeregisterDevice (g_hDev);
		PacketSetLastError (GetLastError ());
		g_hDev = NULL;
	}
	
    if (g_hDriver != INVALID_HANDLE_VALUE)
    {
        fRet = CloseHandle (g_hDriver);
        if (fRet == FALSE)
        {
            ERRORMSG(1,(TEXT("Unable to close PacketDrv (PKT) driver!\r\n")));
        }
    }
    */
/*    
    if (g_hActiveDriver != INVALID_HANDLE_VALUE) 
    {
        fRet = DeactivateDevice (g_hActiveDriver);
        if (fRet == FALSE)
        {
            ERRORMSG(1,(TEXT("Unable to unlod PKT driver!\r\n")));
        }
    }
*/
	return (fRet);
}



//---------------------------------------------------------------------------
//
// Function name	: PacketOpenAdapter
// Description	    : Opens the specified adapter
// Return type		: LPADAPTER 
// Argument         : LPTSTR AdapterName
//
//---------------------------------------------------------------------------
LPADAPTER PacketOpenAdapter (LPTSTR AdapterName)
{

	LPADAPTER	lpAdapter;
	DWORD		dwRet = 0;
	WCHAR		wchEvName[64];

	// Allocate memory for adapter object
	lpAdapter = (LPADAPTER) HeapAlloc (GetProcessHeap (),  HEAP_ZERO_MEMORY, sizeof (ADAPTER));
	if (lpAdapter == NULL) {
		OutputDebugString( L"Check 1\r\n");
		PacketSetLastError (GetLastError ());
		return NULL;
	}
	
	// Create file handle for the driver
//	lpAdapter->hFile = CreateFile (DRIVER_OPEN_STRING, 0, 0, NULL, OPEN_EXISTING, 0, NULL);
	lpAdapter->hFile = CreateFile (DRIVER_OPEN_STRING, GENERIC_READ, 0, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
	if (lpAdapter->hFile == INVALID_HANDLE_VALUE) {
		PacketSetLastError (GetLastError ());
		OutputDebugString( L"Check 2\r\n");
		HeapFree (GetProcessHeap (), 0, lpAdapter);
		return NULL;
	}

	// open the specified adapter
	if (DeviceIoControl (lpAdapter->hFile, PIOC_OPEN_ADAPTER, (LPVOID)AdapterName, 
		wcslen (AdapterName)*sizeof (TCHAR), NULL, 0, &dwRet, NULL) == FALSE) 
    {

		OutputDebugString( L"Check 3\r\n");
		PacketSetLastError (GetLastError ());

		DeviceIoControl (lpAdapter->hFile, PIOC_CLOSE_ADAPTER, NULL, 0, NULL, 0, &dwRet, NULL);
		CloseHandle (lpAdapter->hFile);
		HeapFree (GetProcessHeap (), 0, lpAdapter);
		return NULL;
	}

	// get the event name from the driver
	DeviceIoControl (lpAdapter->hFile, PIOC_EVNAME, NULL, 
		0, wchEvName, sizeof (wchEvName), &dwRet, NULL);
	if (dwRet == 0) {
		OutputDebugString( L"Check 4\n");
		PacketSetLastError (GetLastError ());
		CloseHandle (lpAdapter->hFile);
		HeapFree (GetProcessHeap (), 0, lpAdapter);
		return NULL;
	}

	// create the read event
	lpAdapter->ReadEvent = CreateEvent (NULL, FALSE, FALSE, wchEvName);
	if (lpAdapter->ReadEvent == NULL) {

		OutputDebugString( L"Check 5\n");
		PacketSetLastError (GetLastError ());
		CloseHandle (lpAdapter->hFile);
		HeapFree (GetProcessHeap (), 0, lpAdapter);
		return NULL;
	}

	// Set the number of writes
	PacketSetNumWrites (lpAdapter, 1);

	// set the minimum bytes to copy
	PacketSetMinToCopy (lpAdapter, ETHERNET_HEADER_LENGTH);

	return lpAdapter;
}


//---------------------------------------------------------------------------
//
// Function name	: PacketAllocatePacket
// Description	    : Allocates and initializes a new packet
// Return type		: LPPACKET  - allocated packet
//
//---------------------------------------------------------------------------
LPPACKET PacketAllocatePacket (VOID)
{
	LPPACKET lpPacket = NULL;

	// allocate memory for the packet
	lpPacket = (LPPACKET) HeapAlloc (GetProcessHeap (), HEAP_ZERO_MEMORY, sizeof(PACKET)*1024);
    if (lpPacket == NULL){
		return NULL;
	}

	lpPacket->Buffer			= NULL;
	lpPacket->Length			= 0;
	lpPacket->ulBytesReceived	= 0;
	lpPacket->bIoComplete		= FALSE;

	return lpPacket;
}





//---------------------------------------------------------------------------
//
// Function name	: PacketFreePacket 
// Description	    : Frees a previously allocated packet
// Argument         : LPPACKET lpPacket - Packet to free
//
//---------------------------------------------------------------------------
VOID PacketFreePacket (LPPACKET lpPacket)
{
	if(NULL == lpPacket)
		return ;

    if(HeapFree (GetProcessHeap (), 0, lpPacket) != 0)
		lpPacket = NULL;
}





//---------------------------------------------------------------------------
//
// Function name	: PacketInitPacket 
// Description	    : Initializes a packet
// Return type		: VOID 
// Argument         : LPPACKET lpPacket - Packet to initialize
// Argument         : PVOID Buffer	- Buffer to initialize packet with
// Argument         : UINT Length	- Length of the buffer
//
//---------------------------------------------------------------------------
VOID PacketInitPacket (LPPACKET lpPacket, PVOID Buffer, UINT Length)
{
	if(NULL == lpPacket)
		return ;

	lpPacket->Buffer			= Buffer;
    lpPacket->Length			= Length;
	lpPacket->ulBytesReceived	= 0;
	lpPacket->bIoComplete		= FALSE;
}



//---------------------------------------------------------------------------
//
// Function name	: PacketCancelPacket
// Description	    : Cancel a previously called read operation
// Return type		: DWORD - returns 0 for error, 1 for success
// Argument         : LPADAPTER lpAdapter - Adapter object on which to cancel 
//					: the read operation
// Argument         : LPPACKET lpPacket -  not used (just for compatibility)
//
//---------------------------------------------------------------------------
DWORD PacketCancelPacket (LPADAPTER lpAdapter, LPPACKET lpPacket)
{
	// check parameter
	if (lpAdapter == NULL) {
		PacketSetLastError ((DWORD)INVALID_HANDLE_VALUE);
		return 0;
	}

	if (lpAdapter->hFile == NULL) {
		PacketSetLastError ((DWORD)INVALID_HANDLE_VALUE);
		return 0;
	}

	if (lpAdapter->ReadEvent == NULL) {
		PacketSetLastError ((DWORD)INVALID_HANDLE_VALUE);
		return 0;
	}

	if (SetEvent (lpAdapter->ReadEvent) == FALSE) {
		PacketSetLastError (GetLastError ());
		return 0;
	}

	return 1;
}



//---------------------------------------------------------------------------
//
// Function name	: PacketCheckLoadState
// Description	    : Checks whether the driver is loaded or not
// Return type		: BOOL - TRUE if driver is loaded, FALSE if not
// Argument         : LPCSTR lpDriverBaseName
//
//---------------------------------------------------------------------------
BOOL PacketCheckLoadState (LPCSTR lpDriverBaseName)
{
	BOOL	bIsLoaded = FALSE;
	HANDLE  hDriver = INVALID_HANDLE_VALUE;
	
	//
	// Open the Device Driver
	//
	hDriver = CreateFile (DRIVER_OPEN_STRING, 0, 0,
		NULL, OPEN_EXISTING, 0, NULL);
	
	if (hDriver != INVALID_HANDLE_VALUE) {
		bIsLoaded = TRUE;
		CloseHandle (hDriver);
	}
	
	return bIsLoaded;
}




//---------------------------------------------------------------------------
//
// Function name	: PacketGetAdapterNames
// Description	    : Returns the new line separated list of adapter names
// Return type		: BOOL - TRUE if successfull else FALSE
// Argument         : PWCHAR pBuffer - Buffer to hold the list
// Argument         : DWORD dwBuffLen - length of the buffer
// Argument         : LPADAPTER lpAdapter /*Optional*/ - optional adapter handle if one is already
//					: opened else NULL
//
//---------------------------------------------------------------------------
BOOLEAN PacketGetAdapterNames(PWSTR pBuffer,PULONG  dwBuffLen)
{
	BOOL	bResult = FALSE;
	HANDLE	hDriver = INVALID_HANDLE_VALUE;


	//PacketSetLastError (ERROR_SUCCESS);

	// Create an instance of the driver
	hDriver = CreateFile (DRIVER_OPEN_STRING, GENERIC_READ, 0, 
		NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);

	if (hDriver == INVALID_HANDLE_VALUE) {
		OutputDebugString( L"PKT ADATER NAMES: CREATE FILE FAILED\n" );
		PacketSetLastError (GetLastError ());
		return FALSE;
	}


	// get the mac name list
	if (DeviceIoControl (hDriver, PIOC_GET_MACNAME, NULL, 
		0, (LPVOID)pBuffer, *dwBuffLen, dwBuffLen, NULL) == FALSE){
		OutputDebugString( L"PKT ADATER NAMES: IO CONTROL FAILED\n" );
		PacketSetLastError (GetLastError ());
		CloseHandle (hDriver);
		return FALSE;
	}

	CloseHandle (hDriver);
	return TRUE;

}