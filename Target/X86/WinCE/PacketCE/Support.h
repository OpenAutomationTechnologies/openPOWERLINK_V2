#ifndef __SUPPORT_H__
#define __SUPPORT_H__

#ifdef __cplusplus
extern "C" {
#endif

// Common file required by the driver as well as dll files
#define ETHERNET_HEADER_LENGTH	14

#define PIOC_OPEN_ADAPTER			7419
#define PIOC_CLOSE_ADAPTER			7420
#define PIOC_SET_BUFFER_SIZE		9592
#define PIOC_SETF					9030
#define PIOC_SRTIMEOUT				7416

#define PIOC_SMINTOCOPY				7414
#define PIOC_SETOID					9100
#define PIOC_QUERYOID				9101
#define PIOC_EVNAME					7415

#define PIOC_RELEASEREQUESTS		7412
#define PIOC_READ_PACKET			9032
#define PIOC_WRITE_PACKET			9033
#define PIOC_GET_MACNAME			9034
#define PIOC_RESET					9038

#define DRIVER_NAME					L"PKT"
#define DRIVER_DLL					L"pktdrv.dll"
#define DRIVER_INSTANCE				0x4545

//#define NDIS_PACKET_TYPE_DIRECTED           0x0001
//#define NDIS_PACKET_TYPE_MULTICAST          0x0002
//#define NDIS_PACKET_TYPE_ALL_MULTICAST      0x0004
//#define NDIS_PACKET_TYPE_BROADCAST          0x0008
//#define NDIS_PACKET_TYPE_SOURCE_ROUTING     0x0010
//#define NDIS_PACKET_TYPE_PROMISCUOUS        0x0020
//#define NDIS_PACKET_TYPE_SMT                0x0040
//#define NDIS_PACKET_TYPE_ALL_LOCAL          0x0080
//#define NDIS_PACKET_TYPE_MAC_FRAME          0x8000
//#define NDIS_PACKET_TYPE_FUNCTIONAL         0x4000
//#define NDIS_PACKET_TYPE_ALL_FUNCTIONAL     0x2000
//#define NDIS_PACKET_TYPE_GROUP              0x1000

/////////////////////////////////////////////////////////////////////////////////////
// Declaration related to BPF
struct time_val {
        long    tv_sec;         /* seconds */
        long    tv_usec;        /* and microseconds */
};

struct bpf_program {
	UINT	bf_len;					// length of the program
	struct	bpf_insn *bf_insns;		// pointer to the 
};

struct bpf_insn {					// define the instruction structure
	USHORT	code;					// the instruction code
	UCHAR 	jt;						// first operand
	UCHAR 	jf;						// second operand
	int k;
};


struct bpf_hdr {					// bpf header for every packet
	struct time_val		bh_tstamp;	// time stamp
	UINT				bh_caplen;	// length of captured portion
	UINT				bh_datalen;	// original length of packet
	USHORT				bh_hdrlen;	// length of bpf header (this struct plus alignment padding)
};

typedef struct _PACKET_OID_DATA {
    ULONG Oid;
    ULONG Length;
    UCHAR Data[1];
}PACKET_OID_DATA, *PPACKET_OID_DATA; 

// PACKET_WORDALIGN rounds up to the next even multiple of Packet_ALIGNMENT. 
#define PACKET_ALIGNMENT sizeof(int)
#define PACKET_WORDALIGN(x) (((x) + (PACKET_ALIGNMENT - 1)) & ~(PACKET_ALIGNMENT - 1))

#ifdef __cplusplus
}
#endif

#endif // __SUPPORT_H__