#ifndef __PACKET32_H__
#define __PACKET32_H__
#include "support.h"

#define DRIVER_OPEN_STRING	L"PKT1:"
// Globals for retrieving Adapter names
#define RX_BUFFER_SIZE		1024
#define NO_OF_ADAPTERS		5
#define ADAPTER_NAME_SIZE	50

#ifdef __cplusplus
extern "C" {
#endif

typedef struct _PACKET {  
	PVOID        Buffer;
	UINT         Length;
	UINT         ulBytesReceived;
	BOOL		 bIoComplete;
} PACKET,	*LPPACKET;


typedef struct _ADAPTER { 
	UINT	nWrites;
	HANDLE	hFile;
	HANDLE	ReadEvent;
} ADAPTER, *LPADAPTER;

typedef struct NetType
{
	UINT LinkType;
	UINT LinkSpeed;
} NetType;

//---------------------------------------------------------------------------
// Function prototypes

BOOLEAN PacketSetMinToCopy(LPADAPTER AdapterObject,int nbytes);
BOOLEAN PacketSetNumWrites(LPADAPTER AdapterObject,int nwrites);
BOOLEAN PacketSetMode(LPADAPTER AdapterObject,int mode);
BOOLEAN PacketSetMaxLookaheadsize (LPADAPTER AdapterObject);
BOOLEAN PacketSetReadTimeout(LPADAPTER AdapterObject,int timeout);
BOOLEAN PacketSetBpf(LPADAPTER AdapterObject,struct bpf_program *fp);
BOOLEAN PacketGetStats(LPADAPTER AdapterObject,struct bpf_stat *s);
BOOLEAN PacketSetBuff(LPADAPTER AdapterObject,int dim);
BOOLEAN PacketGetNetType (LPADAPTER AdapterObject,NetType *type);
LPADAPTER PacketOpenAdapter(LPTSTR AdapterName);
BOOLEAN PacketSendPacket(LPADAPTER AdapterObject,LPPACKET pPacket,BOOLEAN Sync);
LPPACKET PacketAllocatePacket(void);
LPPACKET PacketAllocateNPacket(UINT n);
VOID PacketInitPacket(LPPACKET lpPacket,PVOID  Buffer,UINT  Length);
VOID PacketFreePacket(LPPACKET lpPacket);
BOOLEAN PacketResetAdapter(LPADAPTER AdapterObject);
BOOLEAN PacketWaitPacket(LPADAPTER AdapterObject,LPPACKET lpPacket);
BOOLEAN PacketReceiveNPacket(LPADAPTER AdapterObject,LPPACKET headLPacket,UINT n,UINT length,BYTE* buffer,BOOLEAN Sync);
BOOLEAN PacketReceivePacket(LPADAPTER AdapterObject,LPPACKET lpPacket,BOOLEAN Sync);
VOID PacketCloseAdapter(LPADAPTER lpAdapter);
BOOLEAN PacketSetHwFilter(LPADAPTER AdapterObject,ULONG Filter);
BOOLEAN PacketGetAdapterNames(PTSTR pStr,PULONG  BufferSize);
BOOLEAN PacketGetNetInfo(LPTSTR AdapterName, PULONG netp, PULONG maskp);
BOOLEAN PacketRequest(LPADAPTER  AdapterObject,BOOLEAN Set,PPACKET_OID_DATA  OidData);
VOID PacketSetNextPacket(LPPACKET lpPacket, LPPACKET next);
VOID PacketSetLengthBuffer(LPPACKET lpPacket, UINT dim);
VOID PacketSetLengthPacket(LPPACKET lpPacket, UINT numBytes);
LPPACKET PacketGetNextPacket(LPPACKET lpPacket);
BOOLEAN	PacketLoadDriver(VOID);
BOOL PacketUnloadDriver (VOID);
BOOL PacketSetMaxLookahead (LPADAPTER lpAdapter);
DWORD PacketSetLastError (DWORD nNewError);
DWORD PacketGetLastError (VOID);
#ifdef __cplusplus
}
#endif


#endif //__PACKET32_H__