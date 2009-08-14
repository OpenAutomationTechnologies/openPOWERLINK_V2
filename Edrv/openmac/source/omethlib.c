/* omethlib.h - Ethernet Library for FPGA MAC Controller*/ 
/*
------------------------------------------------------------------------------
Copyright (c) 2009, B&R
All rights reserved.

Redistribution and use in source and binary forms,
with or without modification, 
are permitted provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright notice, 
      this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright notice,
      this list of conditions and the following disclaimer 
      in the documentation and/or other materials provided with the distribution.
    * Neither the name of the B&R nor the names of its contributors 
      may be used to endorse or promote products derived from this software 
      without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, 
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR 
A PARTICULAR PURPOSE ARE DISCLAIMED. 
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR 
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES 
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND 
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF 
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
------------------------------------------------------------------------------
 Module:    omethlib
 File:      omethlib.h
 Author:    Thomas Enzinger
 Created:   11.10.2004
 Revised:   26.03.2009
 State:     only tested on Nios II
------------------------------------------------------------------------------

 Functions:
 	omethMiiControl			- control reset pin of MII

 	omethInit				- initialize the ethernet driver (to be called once at startup)
 	omethCreate				- create driver instance
	omethGetHandle			- get handle of an instance based on its adapter number

	omethPeriodic			- must be called cyclic to update phy register table
	omethPhyHardwareAdr		- get hardware address of phy
	omethPhyInfo			- get address to phy register structure
	omethPhyLinkState		- get link state of phy
	omethPhyWrite			- write phy register (blocking)
	omethPhyRead			- read phy register (blocking)
	omethPhyReadNonBlocking	- read phy register (nonblocking, can not be combined with omethPeriodic())

	omethHookCreate			- create hook to get callbacks for received frames
	omethHookSetFunction	- change the callback function of a created hook

	omethFilterCreate		- create a filter to specify which frames should call a hook
	omethFilterSetPattern	- change all filter value/mask bytes of a filter
	omethFilterSetByteValue - change 1 filter value byte of a filter
	omethFilterSetArgument	- change the callback argument which a specific filter passes to the callback
	omethFilterSetNoMatchIRQ- enable the NoMatch-IRQ

	omethSetSCNM			- define a filter for the slot communication management
	omethResponseInit		- prepare a filter for auto response
	omethResponseInitBuf	- prepare a filter for auto response and provide change buffers
	omethResponseSet		- allign a packet which should be responded on a filter event
	omethResponseDisable	- disable response frame
	omethResponseEnable		- enable response frame

	omethTransmit			- transmit frame (immediately or after filter event set with omethSetSCNM)
	omethTransmitArg		- transmit frame with argument for tx-callback
	omethTransmitTime		- transmit frame at defined timestamp
	omethTransmitPending	- get number of pending frames in tx-queue

	omethStart				- start ethernet driver
	omethStop				- stop ethernet driver (disable all rx and tx events)

	omethGetTimestamp		- get timestamp of a received packet
	omethPacketFree			- function to release a received packet

	omethStatistics			- get pointer to ethernet statistics

	omethRxIrqHandler		- irq handler for packet receive
	omethTxIrqHandler		- irq handler for packet transmit

	omethRxIrqHandlerMux	- irq handler for packet receive of all installed ethernet interfaces
	omethTxIrqHandlerMux	- irq handler for packet transmit of all installed ethernet interfaces
	
	omethNoFilterMatchIrqHandler	- irq handler for No-Filter match IRQ

	omethDestroy			- destroy driver instance (stop MAC and free all allocated resources)


	EPLV2 MN Specific:
		omethMnTransmit
		omethMnTransmitTime
		omethMnResponseAndFilterSet
		omethMnResponseEnable
		omethMnResponseDisable
		omethMnSetNextRxBuffer
		omethMnSetCurrentRxBuffer
		omethMnRxIrqHandler
		omethMnTxBegIrqHandler

------------------------------------------------------------------------------
 History:
 12.03.2004 enzingert	created
 23.03.2009 zelenkaj    Changes in omethTxIrqHandler() to free all sent packets
                         in the TX IR, to avoid TX queue overflow.
 26.03.2009 zelenkaj    revised

----------------------------------------------------------------------------*/

#include <omethlib.h>
#include <string.h>				// used functions: memcpy, memset
#include <stdlib.h>				// used functions: calloc
#include <omethlib_target.h>	// target specific defines (BIG/LITTLE endian)

// check if target-specific defines are ok
#ifndef OMETH_HW_MODE
	#error OMETH_HW_MODE not defined (see omethlib_target.h)
#endif

#if OMETH_HW_MODE > 1
	#error OMETH_HW_MODE invalid (see omethlib_target.h)
#endif


#define OMETH_MIN_TX_FRAME		60	// minimal transmit frame length (without checksum)

//-------------------- filter entry type (for 1 byte) --------------------
typedef struct
{
	unsigned char mask;
	unsigned char value;
}ometh_filter_entry_typ;

//-------------------- filter type --------------------
#define OMETH_FILTER_LEN	31
#define OMETH_XFILTER_LEN	17	// x-filters have only 17 entries !
#define OMETH_X_OFFSET		24	// frame offset for x-response

typedef struct
{
	ometh_filter_entry_typ	b[OMETH_FILTER_LEN];	// 31 filter entries
	unsigned char			reserve;
	unsigned char			command;	// command register
}ometh_filter_typ;

typedef struct
{
	ometh_filter_entry_typ	b[OMETH_XFILTER_LEN];	// 17 filter entries
	unsigned char			reserve;
	unsigned char			command;	// command register
}ometh_xfilter_typ;

#define CMD_FILTER_SYNC	0x10	// this is a sync filter
#define CMD_RX_NOMATCH	0x20	// if no filter match -> set nomatch irq
#define CMD_FILTER_ON	0x40	// filter-on flag in command register
#define CMD_TX_ENABLE	0x80	// enables auto transmitting of a tx descriptor

//-------------------- rx/tx descriptor --------------------
typedef struct
{
	unsigned char	high;
	unsigned char	low;
}ometh_desc_flags_bytes;

typedef union	// descriptor status
{
	ometh_desc_flags_bytes	byte;
	unsigned short			word;
}ometh_desc_flags_union;

typedef struct
{
	#if OMETH_HW_MODE==0
		ometh_desc_flags_union	flags;
		unsigned short			len;		// number of bytes
	#endif
	#if OMETH_HW_MODE==1
		unsigned short			len;		// number of bytes
		ometh_desc_flags_union	flags;
	#endif

	unsigned long	pData;		// ptr to data
	unsigned long	txStart;	// tx start time
	unsigned long	time;		// time stamp
}ometh_desc_typ;

//-------------------- mii interface type --------------------
volatile typedef struct
{
	union
	{
		unsigned short req;
		unsigned short ack;
	}cmd;
	unsigned short data;
	unsigned short control;
}ometh_mii_typ;

#define MII_REG_ENABLE		0x0080		// bit mask for reset bit in phy control

#define FLAGS1_OWNER		0x01		// 1 if mac is the owner of the descriptor
#define FLAGS1_LAST			0x02		// last-bit
#define FLAGS1_SENT			0x04		// bit will be set if mac has sent a frame
#define FLAGS1_START_TIME	0x40		// frame will be sent at given start time in descriptor
#define FLAGS1_TX_BEG		0x20		// irq will be sent at given start time in descriptor

#define FLAGS2_ALIGNMENT	0x10		// frame has alignment error

//-------------------- hardware type 01 --------------------
// Typ 01 has 16 filters, 16 rx descriptors and 16 tx descriptors
#define MAC_HW_TYP_01_NB_FILTER		16
#define MAC_HW_TYP_01_NB_RXDESC		16
#define MAC_HW_TYP_01_NB_TXDESC		16

//-------------------- hardware type 02 --------------------
// Typ 02 has 14 standard filters, 4 xnode-filters, everything else like typ 01
#define MAC_HW_TYP_02_NB_XFILTER	 4

//-------------------- hardware type 03 --------------------
// 2 tx descriptor queues, reduced filter possibility
#define MAC_HW_TYP_03_NB_XFILTER		3	// only 1 of them can be used
#define MAC_HW_TYP_03_NB_FILTER			16	// only 8 of them can be used
#define MAC_HW_TYP_03_NB_RXDESC			16
#define MAC_HW_TYP_03_NB_TXDESC_LOW		8
#define MAC_HW_TYP_03_NB_TXDESC_HIGH	8

typedef struct
{
	ometh_filter_typ	filter[MAC_HW_TYP_01_NB_FILTER];
	ometh_desc_typ		rxDesc[MAC_HW_TYP_01_NB_RXDESC];
	ometh_desc_typ		txDesc[MAC_HW_TYP_01_NB_TXDESC];
}ometh_hw_typ1;

typedef struct
{
	ometh_filter_typ	filter[MAC_HW_TYP_03_NB_FILTER];
	ometh_desc_typ		rxDesc[MAC_HW_TYP_03_NB_RXDESC];
	ometh_desc_typ		txDescLow[MAC_HW_TYP_03_NB_TXDESC_LOW];
	ometh_desc_typ		txDescHigh[MAC_HW_TYP_03_NB_TXDESC_HIGH];
}ometh_hw_typ3;

//-------------------- buffer header --------------------
#ifdef OMETH_TRACE
	#include <omethTrace.h>

	static	OMETH_TRACE_RX_FCT	*pTraceFctInt;		// global trace function for all adapters, must be set before omethStart()
#endif

// additional info about pending frames
typedef struct ometh_pending_typ
{
	ometh_buf_typ				*pBuf;	// ptr to buffer
	struct ometh_pending_typ	*pNext;	// ptr to next pending
}ometh_pending_typ;

//-------------------- client hook --------------------
struct OMETH_HOOK
{
	OMETH_HOOK_FCT	*pFct;			// function ptr to hook
	unsigned char	maxPending;		// number of max pending buffers
	unsigned short	cntOverflow;	// statistics: buffer overflow

	OMETH_H				hEth;		// handle to ethernet instance
	ometh_buf_typ		*pRxBufBase;	// (only used for destroy function)
	struct OMETH_HOOK	*pNext;			// ptr to next hook (only used for destroy function)
	ometh_pending_typ	*pFreeRead;	// ptr to get next free ptr in the list
	ometh_pending_typ	*pFreeWrite;// ptr to release next buffer
	ometh_pending_typ	free[1];	// list with free pointers
};

// additional info about tx descriptors
typedef struct ometh_tx_info_typ
{
	unsigned char				flags1;			// flags for the tx descriptor
	unsigned char				index;			// index (required for filter configuration)
	unsigned char				chgIndexWrite;	// last write index of change-buffer-system (auto response buffers)
	unsigned char				chgIndexRead;	// last read index of change-buffer-system (auto response buffers)
	ometh_desc_typ				*pDesc;			// ptr to tx descriptor
	OMETH_BUF_FREE_FCT_ARG		*pFctFree;		// ptr to free function for this buffer (ptr to filter cmd for auto response descriptors)
	void						*fctFreeArg;	// argument passed to free funcion after packet pointer
	ometh_packet_typ			*pProduced[3];	// remember the produced frames for auto-response buffers here
	struct ometh_tx_info_typ	*pNext;			// ptr to next tx info
}ometh_tx_info_typ;

// additional info about rx descriptors
typedef struct ometh_rx_info_typ
{
	unsigned char				flags1;		// flags for the rx descriptor
	ometh_desc_typ				*pDesc;		// ptr to rx descriptor
	struct ometh_rx_info_typ	*pNext;		// ptr to next rx descriptor
}ometh_rx_info_typ;


// reference to filter hardware ports
typedef struct ometh_filter_data_typ
{
	unsigned char			cmd;				// current filter command
	unsigned char			txEnableRequest;	// auto response should be enabled at next omethResponseSet
	unsigned char			len;				// filter length
	unsigned char			*pCommand;			// ptr to filter command (can be different for different filters)
	ometh_filter_entry_typ	*pFilterWriteOnly;	// ptr to filter data (write only area)
}ometh_filter_data_typ;

//-------------------- filter handle (must be 32 byte for efficient access in IRQ) --------------
struct OMETH_FILTER
{
	OMETH_HOOK_H			hHook;			// ptr to used hook structure
	void					*arg;			// argument for hook
	ometh_tx_info_typ		*pTxInfo;		// ptr to tx descriptor which holds the auto-answer for this filter
	ometh_filter_data_typ	*pFilterData;	// ptr to filter hardware
};

//-------------------- register flags ------------------------
#define OMETH_REG_IE			0x8000		// irq enable
#define OMETH_REG_SOFTIRQ		0x4000		// software IRQ
#define OMETH_REG_HALF			0x2000		// hub enable (tx-control)

#define OMETH_REG_XNODE_1		0x2000		// x-node enable (rx-control) filter 14,15
#define OMETH_REG_XNODE_2		0x4000		// x-node enable (rx-control) filter 12,13

#define OMETH_REG_XNODE			(OMETH_REG_XNODE_1 | OMETH_REG_XNODE_2)

#define OMETH_REG_SYNC			0x1000		// set sync mode
#define OMETH_REG_DIAG			0x1000		// enable diag (frames < 31 byte also generate IRQ)
#define OMETH_REG_PENDING		0x0F00		// mask for pending field
#define OMETH_REG_IQUIT			0x0100		// quit interrupt (only valid on clear port)
#define OMETH_REG_RUN			0x0080		// run bit
#define OMETH_REG_TX_BEG		0x0040		// quit tx beg irq
#define OMETH_REG_RX_NOMATCH	0x0040		// quit interrupt -> nomatch
#define OMETH_REG_LOST			0x0010		// lost flag (rx-control)

// only for special functions : mask and bits to adjust rx-mode in status register
#define OMETH_REG_RXMODE_MASK		0x0003

#define OMETH_REG_RXMODE_DEFAULT	0x0000
#define OMETH_REG_RXMODE_HIGH_PRIO	0x0001
#define OMETH_REG_RXMODE_BLOCK		0x0002
#define OMETH_REG_RXMODE_AUTO		0x0003

typedef struct
{
	#if OMETH_HW_MODE==0
		unsigned short	value;			// read/write port
		unsigned short	setBit;			// set single bits in status register
		unsigned short	clrBit;			// clear single bits in status register
		unsigned short	setDescriptor;	// set descriptor index
	#endif
	#if OMETH_HW_MODE==1
		unsigned short	setBit;			// set single bits in status register
		unsigned short	value;			// read/write port
		unsigned short	setDescriptor;	// set descriptor index
		unsigned short	clrBit;			// clear single bits in status register
	#endif
}ometh_status_typ;					// mac rx/tx status registers

typedef struct
{
	ometh_status_typ txStatus;	// tx control register
	ometh_status_typ rxStatus;	// rx control register
}ometh_reg_typ;					// mac control registers

//-------------------- instance of ethernet driver --------------------
struct OMETH_TYP
{
	unsigned short		rxLen;			// data length of rx buffers
	unsigned char		nbFilter;		// number of filters
	unsigned char		nbFilterX;		// number of filters for x-node functionality
	unsigned char		txQueueEnable;	// will be set to 0 if upper layer switches queue sending off

	unsigned char		cntTxQueueIn,cntTxQueueOut;		// counter to evaluate the number of pending tx descriptors

	ometh_reg_typ		*pRegBase;		// control register base adr

	struct OMETH_FILTER	*pFilterList;	// ptr to filter table (list of handles)

	ometh_rx_info_typ	*pRxNext;	// ptr to next rx info

	ometh_stat_typ		stat;

	ometh_filter_data_typ	*pFilterSCNM;	// ptr to filter for SCNM Mode

	ometh_tx_info_typ	*pTxInfo[2];	// ptr to first tx info
	ometh_tx_info_typ	*pTxNext[2];	// ptr to next tx info
	ometh_tx_info_typ	*pTxFree[2];	// ptr to next release buffer

	ometh_tx_info_typ	*pTxAuto;		// first auto tx descriptor

	ometh_config_typ	config;			// copy of config structure from omethCreate

	unsigned char		phyCount;		// number of phy's on this mac (max 8)
	unsigned char		phyLinkCount;	// nubmer of linked phys
	unsigned char		phyLinkMax;		// max number of linked phys
	unsigned char		phyHalfCount;	// nubmer of phys linked with half duplex

	unsigned char		phyAdr[OMETH_MAX_PHY_CNT];		// array with phy addresses on MII interface
	unsigned short		phyCmdRead[OMETH_MAX_PHY_CNT];	// phy write commands
	unsigned short		phyCmdWrite[OMETH_MAX_PHY_CNT];	// phy read commands

	phy_reg_typ			*pPhyReg;	// ptr to all phy register sets
	unsigned short		phyPort;	// current port
	unsigned short		phyReg;		// current register
	
	unsigned short		txVal;		// tx value for phy write
	unsigned short		txPort;		// tx port for phy write
	unsigned char		txReg;		// tx register for phy write

	unsigned char		clearPendingIrqAtStart;	// will be set to 1 only for the first start

	// hardware access for debug
	union
	{
		ometh_hw_typ1	typ1;		// typ 1 hardware
		ometh_hw_typ3	typ3;		// typ 3 hardware
	}*pHardware;

	unsigned char		*pRxBufBase;	// (only used for destroy function)
	struct OMETH_HOOK	*pHookList;		// (only used for destroy function)

	ometh_rx_info_typ	*pRxInfo;		// ptr to first rx info
	
	#if (OMETH_ENABLE_SOFT_IRQ==1)
		OMETH_BUF_FREE_FCT	*pFctSoftIrq;	// function will be called if soft-IRQ was triggered (Tx-IRQ Level)
	#endif

	struct OMETH_TYP	*pNext;			// ptr to next driver instance
};

// constants for phy control
//MII Interface
#define PHY_REG_READ				0x6000	// MII read flags
#define PHY_REG_WRITE				0x5002	// MII write flags 

typedef struct ometh_internal_typ
{
	struct OMETH_TYP *pFirstEth;	// ptr to first instance handle
	struct OMETH_TYP *pPeriodicEth;	// ptr to currently used instance for periodic call
}ometh_internal_typ;

static ometh_internal_typ	omethInternal;	// driver internal data


//*************************************************************************************
//
// Get the base address of a structure where only the address of a sub-element is known
//
//	typ		: type name of the regarding struct (must be done with typedef)
//	element	: name of the structure element from which we know the address
//	ptr		: address of the structure element
//
//	result	: structure pointer to the base of the type
//
//  example:
//		ometh_buf_typ	*pBuf;		// ptr to buffer
//		// 'pPacket' is the address to the element 'packet' in the base type 'ometh_buf_typ'
//		// the macro will return the base ptr
//		pBuf = GET_TYPE_BASE( ometh_buf_typ, packet, pPacket);
//
//*************************************************************************************
#define GET_TYPE_BASE(typ, element, ptr)	\
	((typ*)( ((size_t)ptr) - (size_t)&((typ*)0)->element ))

#ifdef OMETH_MAX_RETRY
	#define SET_MAX_RETRY	pDesc->flags.byte.low = OMETH_MAX_RETRY
#else
	#define SET_MAX_RETRY	
#endif

//*************************************************************************************
//	Begin of OMETH_TRANSMIT / Macro for omethTransmitXX()
//*************************************************************************************
#define	OMETH_TRANSMIT( ARG, TIME, addFlags, TX_QUEUE_INDEX )									\
	ometh_tx_info_typ	*pInfo	= hEth->pTxNext[TX_QUEUE_INDEX];	/* access to next tx info structure */	\
	ometh_desc_typ		*pDesc	= pInfo->pDesc;			/* access to tx descriptor */			\
	unsigned short		len;																	\
																								\
	/* check if descriptor is free */															\
	if(pPacket == 0)				return 0;	/* invalid packet passed */						\
	if(hEth->txQueueEnable == 0)	return 0;													\
	if(pDesc->pData != 0)			return 0;	/* descriptor is not free (queue full !) */		\
																								\
	len = pPacket->length;	/* padding, ethernet frames must be at least 64 byte long */		\
	if(len < OMETH_MIN_TX_FRAME) len=OMETH_MIN_TX_FRAME;										\
																								\
	pDesc->pData	= (unsigned long)&pPacket->data;	/* write buffer ptr to descriptor */	\
	pDesc->len		= len;																		\
	pInfo->fctFreeArg	= ARG;	/* store user argument for tx-callback */						\
	pDesc->txStart		= TIME;	/* scheduled start time of this frame  */						\
	/* (if reaching this descriptor the tx-queue will wait until the time is reached */			\
	pInfo->pFctFree	= (OMETH_BUF_FREE_FCT_ARG*)pFct;	/* store callback for free function */	\
																								\
	hEth->cntTxQueueIn++;																		\
	hEth->pTxNext[TX_QUEUE_INDEX] = pInfo->pNext;	/* switch to next info strucutre	*/		\
																								\
	SET_MAX_RETRY;																				\
	pDesc->flags.byte.high	= pInfo->flags1 | addFlags;	/* set flag to start transmitter */		\
																								\
	return len

//*************************************************************************************
//	End of OMETH_TRANSMIT
//*************************************************************************************

/* -- exchange table to allocate the next change index number for auto transmit descriptors --------------	*/
/* makes sure the result of chgIndexTab[x][y] is different to x and y */
static const unsigned char chgIndexTab[3][4] =
{
	{1,2,1,0},
	{2,2,0,0},
	{1,0,0,0}
};

// index 0-2 in high bits (to avoid shifting)
static const unsigned long chgIndexHighBit[3] =
{
	0x00000000,
	0x40000000,
	0x80000000
};

/*****************************************************************************
* 
* filter_set_flag - set command flag of packet filter
* 
* RETURN: -
* 
*/
#define FILTER_SET_FLAG(pFilter, flag) (*pFilter->pCommand = (pFilter->cmd |= flag))

/*****************************************************************************
* 
* filter_clear_flag - clear command flag of packet filter
* 
* RETURN: -
* 
*/
#define FILTER_CLEAR_FLAG(pFilter, flag) (*pFilter->pCommand = (pFilter->cmd &= ~flag))


#if (OMETH_ENABLE_SOFT_IRQ==1)
	/*****************************************************************************
	* 
	* soft_irq_dummy - dummy function for soft-irq (until user sets another function)
	* 
	* RETURN: -
	* 
	*/
	static void soft_irq_dummy
	(
	ometh_packet_typ	*pPacket	/* packet which should be released	*/
	)
	{
	}
#endif

/*****************************************************************************
* 
* omethMiiControl - enable/disable phys
* 
* RETURN: 
*	-1   ... error
*	 0-x ... depending on command
*		MII_CTRL_GET_STATE .. 0=reset 1=active
*
*	The following reasons can cause an error:
*	- pPhyBase = 0
*	- MII_CTRL_RESET and MII_CTRL_ACTIVE set at the same time
*
*/
int		omethMiiControl
(
 void			*pPhyBase,	/* ptr to phy register */
 unsigned short	command		/* combination of MII_CTRL_... values */
)
{
	// enable phys
	ometh_mii_typ* pPhy = OMETH_MAKE_NONCACHABLE(pPhyBase);

	if(pPhyBase==0) return -1;

	// reset and active at the same time ... not allowed
	if((command & MII_CTRL_RESET) && (command & MII_CTRL_ACTIVE)) return -1;

	if(command & MII_CTRL_RESET)  pPhy->control &= (unsigned short)~MII_REG_ENABLE;
	if(command & MII_CTRL_ACTIVE) pPhy->control |= MII_REG_ENABLE;
	
	// return 1 if state is requested and phy enable is set
	if((command & MII_CTRL_GET_STATE) && (pPhy->control & MII_REG_ENABLE) ) return 1;

	return 0;
}

/*****************************************************************************
* 
* omethInit - initialize ethernet driver
* 
*/
void	omethInit
(
 void
)
{
	// clear internal driver data
	memset(&omethInternal,0,sizeof(omethInternal));
}


// internal create function, only to be used from omethCreate
// (after the instance was allocated the return value is always the instance
//  pointer, the validation is done later in omethCreate)
static OMETH_H		omethCreateInt
(
 ometh_config_typ	*pEthConfig		/* ptr to ethernet config struct			*/
)
{
	OMETH_H			hEth;
	unsigned long	len,i;
	unsigned char	*pByte;
	unsigned short	data,readData;
	unsigned char	nbTxDesc[2]={0,0};
	
	ometh_desc_typ			*pDesc;
	ometh_tx_info_typ		*pTxInfo;
	ometh_rx_info_typ		*pRxInfo;
	ometh_packet_typ		*pPacket;
	ometh_filter_typ		*pFilter;
	ometh_reg_typ			*pRegBase;
	struct OMETH_FILTER		*pFilterList;

	if(hEth->config.macType != OMETH_MAC_TYPE_01)
		return 0; //openMac is type 1!!!

	// check mempart handle and return if invalid
	if(pEthConfig->rxBuffers == 0) return 0;

	// check mode (full or half duplex must be set)
	if((pEthConfig->mode & (OMETH_MODE_FULLDUPLEX|OMETH_MODE_HALFDUPLEX)) == 0) return 0;

	// validate config type (to avoid resource conflicts)
	for( hEth = omethInternal.pFirstEth ; hEth ; hEth = hEth->pNext )
	{
		// adapter number already used
		if(pEthConfig->adapter  == hEth->config.adapter)  return 0;

		if(pEthConfig->pRamBase == hEth->config.pRamBase) return 0;
		if(pEthConfig->pRegBase == hEth->config.pRegBase) return 0;
	}

	pRegBase = OMETH_MAKE_NONCACHABLE(pEthConfig->pRegBase);

	// initialize control registers
	pRegBase->rxStatus.clrBit        = 0xFFFF;			// clear all bits in rx status
	pRegBase->rxStatus.setDescriptor = 0;

	// clear all bits in tx status except the hub-enable
	pRegBase->txStatus.clrBit        = ~(unsigned short)OMETH_REG_HALF;
	pRegBase->txStatus.setDescriptor = 0x0008;	// for MAC_TYP_03 .. reset queue index for 2nd tx queue
	pRegBase->txStatus.setDescriptor = 0;

	// clear all pending rx irqs
	while(pRegBase->rxStatus.value & OMETH_REG_PENDING) pRegBase->rxStatus.clrBit = OMETH_REG_IQUIT;

	// clear all pending rx-fail irq's
	while(pRegBase->rxStatus.value & OMETH_REG_RX_NOMATCH) pRegBase->rxStatus.clrBit = OMETH_REG_RX_NOMATCH;

	// clear all pending rx irqs
	while(pRegBase->txStatus.value & OMETH_REG_PENDING) pRegBase->txStatus.clrBit = OMETH_REG_IQUIT;

	// clear all pending tx-beg irqs
	while(pRegBase->txStatus.value & OMETH_REG_TX_BEG) pRegBase->txStatus.clrBit = OMETH_REG_TX_BEG;

	// turn x-mode on if a specific mac type is defined
	if(pEthConfig->macType == OMETH_MAC_TYPE_02)
	{
		pRegBase->rxStatus.setBit = OMETH_REG_XNODE;

		// check if set-command was sucessful, otherwise the mode is not supported
		if((pRegBase->rxStatus.value & OMETH_REG_XNODE) == 0 ) return 0;
	}

	hEth = calloc(sizeof(struct OMETH_TYP),1);	// allocate memory for instance handle
	if(hEth == 0) return 0;						// return if alloc failed - system is out of memory

	// at the first start all pending IRQs shall be acknowledged
	hEth->clearPendingIrqAtStart = 1;

	// copy config structure to handle
	memcpy(&hEth->config, pEthConfig, sizeof(hEth->config));

	//convert pointers to hardware to non-cachable pointers
	hEth->config.pPhyBase = OMETH_MAKE_NONCACHABLE(hEth->config.pPhyBase);
	hEth->config.pRamBase = OMETH_MAKE_NONCACHABLE(hEth->config.pRamBase);
	hEth->config.pRegBase = OMETH_MAKE_NONCACHABLE(hEth->config.pRegBase);


	// search for connected phys and count link bits
	for(i=0 ; i < 0x20 && hEth->phyCount < OMETH_MAX_PHY_CNT ; i++)
	{
		// do not search for this phy if list is activated and phy is not in list
		if(hEth->config.mode & OMETH_MODE_PHY_LIST)
		{
			for(len=0;len<hEth->config.phyCount;len++)
			{
				// break this loop if phy address was found in phyList
				if(i == hEth->config.phyList[len]) break;
			}

			// continue with next index if phy not in list
			if(len == hEth->config.phyCount) continue;
		}

		hEth->phyAdr[hEth->phyCount] = i;

		// prepare phy address value for MII interface
		data = i << 7;
		hEth->phyCmdRead[hEth->phyCount]  = data | PHY_REG_READ;
		hEth->phyCmdWrite[hEth->phyCount] = data | PHY_REG_WRITE;

		hEth->phyCount++;
		omethPhyRead(hEth, hEth->phyCount-1, 1, &data);	// get register 1 to count linked ports at startup

		if(data==0xFFFF)
		{
			hEth->phyCount--;
			continue;					// phy not connected, look for next
		}

		if(data & PHY_REG1_LINK) hEth->phyLinkMax++;	// count links at startup
	}

	// error ... no phys found
	if(hEth->phyCount==0) return hEth;

	hEth->pRegBase = hEth->config.pRegBase;

	// overtake parameters to instance handle
	hEth->rxLen = sizeof(pPacket->data) - sizeof(pPacket->data.minData) + hEth->config.rxMtu;

	// get size of DPR
	switch(hEth->config.macType)
	{
		case OMETH_MAC_TYPE_02:	// Typ 02 .. like typ 01 but with 2 x-filters
			hEth->nbFilterX = MAC_HW_TYP_02_NB_XFILTER;

		case OMETH_MAC_TYPE_01:	// Typ 01
			hEth->pHardware	= hEth->config.pRamBase;

			hEth->nbFilter	= MAC_HW_TYP_01_NB_FILTER;
			pFilter			= hEth->config.pRamBase;
			len				= sizeof(ometh_filter_typ) * MAC_HW_TYP_01_NB_FILTER;

			// too many rx buffers configured (hardware dependent)
			if(hEth->config.rxBuffers > MAC_HW_TYP_01_NB_RXDESC) return hEth;

			// allocate structure with rx info
			hEth->pRxInfo	= calloc(hEth->config.rxBuffers, sizeof(ometh_rx_info_typ));

			if(hEth->pRxInfo == 0) return hEth;

			// write ptr to first rx descriptor to first info structure
			hEth->pRxInfo->pDesc = (ometh_desc_typ*)((size_t)hEth->config.pRamBase + len);

			len	= len + sizeof(ometh_desc_typ) * (MAC_HW_TYP_01_NB_RXDESC);

			// allocate structure with tx info
			nbTxDesc[0] = MAC_HW_TYP_01_NB_TXDESC;

			hEth->pTxInfo[0] = calloc(nbTxDesc[0] , sizeof(ometh_tx_info_typ));
			if(hEth->pTxInfo[0] == 0) return hEth;

			// write ptr to first tx descriptor to first info structure
			hEth->pTxInfo[0]->pDesc = (ometh_desc_typ*)((size_t)hEth->config.pRamBase + len);
			
			len = len + sizeof(ometh_desc_typ) * nbTxDesc[0];
			break;

		case OMETH_MAC_TYPE_03:	// Typ 03
			hEth->pHardware	= hEth->config.pRamBase;

			hEth->nbFilterX = MAC_HW_TYP_03_NB_XFILTER;
			hEth->nbFilter	= MAC_HW_TYP_03_NB_FILTER;
			pFilter			= hEth->config.pRamBase;

			// due to the strange arrangement in the memory only every second pair is existing ...
			// so the filters are distributed over the double of the memory
			len = sizeof(ometh_filter_typ) * MAC_HW_TYP_03_NB_FILTER;

			// too many rx buffers configured (hardware dependent)
			if(hEth->config.rxBuffers > MAC_HW_TYP_03_NB_RXDESC) return hEth;

			// allocate structure with rx info
			hEth->pRxInfo	= calloc(hEth->config.rxBuffers, sizeof(ometh_rx_info_typ));

			if(hEth->pRxInfo == 0) return hEth;

			// write ptr to first rx descriptor to first info structure
			hEth->pRxInfo->pDesc = (ometh_desc_typ*)((size_t)hEth->config.pRamBase + len);

			len	= len + sizeof(ometh_desc_typ) * (MAC_HW_TYP_03_NB_RXDESC);

			// allocate structure with tx info
			nbTxDesc[1]			= MAC_HW_TYP_03_NB_TXDESC_LOW;
			hEth->pTxInfo[1]	= calloc(nbTxDesc[1],sizeof(ometh_tx_info_typ));

			if(hEth->pTxInfo[1] == 0) return hEth;

			// write ptr to first tx descriptor to first info structure
			hEth->pTxInfo[1]->pDesc = (ometh_desc_typ*)((size_t)hEth->config.pRamBase + len);
			
			len = len + sizeof(ometh_desc_typ) * nbTxDesc[1];

			// allocate structure with tx info
			nbTxDesc[0]			= MAC_HW_TYP_03_NB_TXDESC_HIGH;
			hEth->pTxInfo[0]	= calloc(nbTxDesc[0],sizeof(ometh_tx_info_typ));

			if(hEth->pTxInfo[0] == 0) return hEth;

			// write ptr to first tx descriptor to first info structure
			hEth->pTxInfo[0]->pDesc = (ometh_desc_typ*)((size_t)hEth->config.pRamBase + len);
			
			len = len + sizeof(ometh_desc_typ) * nbTxDesc[0];
			break;

		default:
			return hEth;			// not allowed
	}

	// start with first tx buffer
	memset(hEth->config.pRamBase, 0, len);			// reset DPR

	//----------------- set all tx descriptor pointers in info structure ----------
	for(i=1;i != -1;i--)	// process first [1] and then [0]
	{
		pTxInfo = hEth->pTxFree[i] = hEth->pTxNext[i] = hEth->pTxInfo[i];
		if(pTxInfo == 0) continue;

		pDesc = pTxInfo->pDesc;

		for(len=0 ; len < nbTxDesc[i] ; len++)
		{
			pTxInfo->flags1	= FLAGS1_OWNER;
			pTxInfo->pDesc	= pDesc;			// tx descriptor ptr
			pTxInfo->pNext	= pTxInfo+1;		// ptr to next info
			pTxInfo->index	= len;

			pTxInfo++;	
			pDesc++;
		}

		pTxInfo--;	// switch back to last info strucutre
		pTxInfo->flags1	= FLAGS1_OWNER | FLAGS1_LAST;
		pTxInfo->pNext	= hEth->pTxInfo[i];				// ptr to first info

		// auto transmit descriptors start from the end of the last tx queue
		if(hEth->pTxAuto==0) hEth->pTxAuto = pTxInfo;
	}


	//----------------- allocate buffers for rx descriptors -----------------------

	// calc length of 1 rx buffer (header + ethernet-header(14) + ip(mtu) + checksum(4)  )
	len = sizeof(ometh_buf_typ) - sizeof(pPacket->data.minData) + hEth->config.rxMtu;

	len = (len+3)&(~3);		// round up to next multiple of 4

	// allocate buffers for rx data
	pByte = OMETH_MAKE_NONCACHABLE(calloc(hEth->config.rxBuffers * len ,1));
	if(pByte == 0) return hEth;

	// store buffer address for destroy function
	hEth->pRxBufBase = pByte;

	//----------------- set all rx descriptor pointers in info structure ----------
	pRxInfo	= hEth->pRxNext = hEth->pRxInfo;

	pDesc	= pRxInfo->pDesc;

	for(i=0 ; i<hEth->config.rxBuffers ; i++)
	{
		pDesc->flags.byte.high = pRxInfo->flags1 = FLAGS1_OWNER;

		pRxInfo->pDesc	= pDesc;		// rx descriptor ptr
		pRxInfo->pNext	= pRxInfo+1;	// ptr to next info

		pDesc->len		= hEth->rxLen;
		pDesc->pData 	= (unsigned long)&((ometh_buf_typ*)pByte)->packet.data;

		pByte = pByte + len;			// switch to next allocated buffer

		pRxInfo++;
		pDesc++;
	}

	pRxInfo--;	// switch back to last info strucutre
	pDesc--;	// switch to last descriptor

	pDesc->flags.byte.high = pRxInfo->flags1	= FLAGS1_OWNER | FLAGS1_LAST;

	pRxInfo->pNext = hEth->pRxInfo;				// ptr to first info
	
	//-----------  allocate structure array for filter infos (one filter info for each filter) -----------------
	hEth->pFilterList = calloc(hEth->nbFilter, sizeof(struct OMETH_FILTER));
	if(hEth->pFilterList == 0) return hEth;

	pByte = calloc(hEth->nbFilter, sizeof(ometh_filter_data_typ));
	if(pByte == 0) return hEth;

	// set filter pointers in info structures
	pFilterList = hEth->pFilterList;
	for(i=0; i < hEth->nbFilter ; i++)
	{
		pFilterList->pFilterData = (ometh_filter_data_typ*)pByte;

		pFilterList->pFilterData->pFilterWriteOnly = pFilter->b;

		if(i < (hEth->nbFilter - hEth->nbFilterX))	// normal filter
		{
			pFilterList->pFilterData->len		= OMETH_FILTER_LEN;
			pFilterList->pFilterData->pCommand	= &pFilter->command;
		}
		else	// X-Filter
		{
			pFilterList->pFilterData->len		= OMETH_XFILTER_LEN;
			pFilterList->pFilterData->pCommand	= &((ometh_xfilter_typ*)pFilter)->command;
		}

		// MAC_TYPE_3:
		//	((i&2)==2)   do not use filters with bit1=1 (filter 2,3,6,7,10,11,14,15)
		//	... to save memory these filters are not implemented at MAC_TYPE_03
		if((hEth->config.macType == OMETH_MAC_TYPE_03) &&   ((i&2)==2) )
		{
			pFilterList->hHook = (OMETH_HOOK_H)-1;
		}

		pFilterList++;	// next filter list entry
		pFilter++;		// next filter in RAM

		pByte = pByte + sizeof(ometh_filter_data_typ);	// next filter data
	}

	// set possible auto-negotiate baud rates for phy depending on mode
	// initialize all phy's and switch on/off hub
	data = PHY_REG4_SELECTOR;		// generate value for register 4 (advertisement register)

	if(hEth->config.mode & OMETH_MODE_FULLDUPLEX) data |= PHY_REG4_100TX_FULL;

	// half duplex
	if(hEth->config.mode & OMETH_MODE_HALFDUPLEX)
	{
		// clear full duplex bit if more then 1 phy is linked at startup
		// (but not for mac_type_2 .. can also work with full duplex)
		if((hEth->phyLinkMax > 1) && (hEth->config.macType != OMETH_MAC_TYPE_02))
		{
			data &= ~(unsigned short)PHY_REG4_100TX_FULL;
		}

		data |= PHY_REG4_100TX_HALF;
	}

	// search for connected phys
	for(i=0 ; i < hEth->phyCount ; i++)
	{
		if(hEth->config.mode & OMETH_MODE_DIS_AUTO_NEG)
		{
			data = 0;

			if(hEth->config.mode & OMETH_MODE_FULLDUPLEX) data |= PHY_REG0_FULL;

			// 10/100 must be defined if autoneg is off
			if(hEth->config.mode & OMETH_MODE_100)		data |= PHY_REG0_100;
			else if ((hEth->config.mode & OMETH_MODE_10) == 0) return hEth;

			// write reg0 to turn autoneg off
			omethPhyWrite(hEth, i, 0, data);
		}
		else
		{
			len = 0;	// flag to detect if reset is required

			omethPhyRead(hEth,i,4,&readData);
			if(readData != data) len=1;	// set flag to restart autoneg if register 4 is not the desired value
			
			omethPhyWrite(hEth, i, 4, data);	// set allowed modes (reg 4 , advertisement register)

			// readback register 4 from phy to make sure the phy is existing, otherwise error
			omethPhyRead(hEth,i,4,&readData);
			if(readData != data) return hEth;

			// restart phy if register 4 or register 0 needs to be updated
			omethPhyRead(hEth,i,0,&readData);

			if(len==1 || (readData & (PHY_REG0_ISOLATE|PHY_REG0_POWER_DOWN|PHY_REG0_AUTONEG_ENABLE)) != PHY_REG0_AUTONEG_ENABLE)
			{
				// enable and restart auto negotiation (reg 0)
				omethPhyWrite(hEth, i, 0, PHY_REG0_AUTONEG_ENABLE | PHY_REG0_AUTONEG_RESTART);
			}
		}
	}

	//---------------------------------  allocate phy register memory -------------------------------
	hEth->pPhyReg = calloc(sizeof(phy_reg_typ) * hEth->phyCount, 1);
	if(hEth->pPhyReg == 0) return hEth;

	#if (OMETH_ENABLE_SOFT_IRQ==1)
		hEth->pFctSoftIrq = &soft_irq_dummy;	// set dummy function for Soft-IRQ
	#endif

	hEth->txQueueEnable = 1;				// by default the tx queue is enabled

	return hEth;	// return instance handle
}

/*****************************************************************************
* 
* omethCreate - create and initialize new instance of ethernet driver
* 
*/
OMETH_H			omethCreate
(
 ometh_config_typ	*pEthConfig		/* ptr to ethernet config struct			*/
)
{
	OMETH_H hEth;

	hEth = omethCreateInt(pEthConfig);	// call internal create function
	if(hEth==0)	return 0;

	// instance was allocated but not successful initialized
	if(hEth->txQueueEnable == 0)
	{
		omethDestroy(hEth);
		return 0;
	}

	// add driver instance to list
	hEth->pNext				= omethInternal.pFirstEth;
	omethInternal.pFirstEth	= hEth;

	return hEth;
}

/*****************************************************************************
* 
* omethGetHandle - get adapter handle
* 
*/
OMETH_H			omethGetHandle
(
 int	adapter		/* number of network adapter (0..255) */
)
{
	OMETH_H hEth;

	// validate config type (to avoid resource conflicts)
	for( hEth = omethInternal.pFirstEth ; hEth ; hEth = hEth->pNext )
	{
		if(hEth->config.adapter == adapter) return hEth;
	}

	return 0;
}

/*****************************************************************************
* 
* omethPhyHardwareAdr - Get hardware address of phy on this port
* 
*/
int				omethPhyHardwareAdr
(
 OMETH_H		hEth,		/* handle of ethernet driver, see omethCreate()		*/
 unsigned short	port		/* phy number / port number of integrated hub (0-n)	*/
)
{
	if(hEth==0) return -1;

	if(port >= hEth->phyCount) return -1;

    return hEth->phyAdr[port];
}

/****************************************************************************
* 
* omethPhyInfo - Get ptr to Phy register copy in RAM
* 
*/
phy_reg_typ*	omethPhyInfo
(
 OMETH_H		hEth,		/* handle of ethernet driver, see omethCreate()		*/
 unsigned short	port		/* phy number / port number of integrated hub (0-n)	*/
)
{
	if(hEth==0) return 0;

	if(port >= hEth->phyCount) return 0;

	return hEth->pPhyReg + port;	// return address
}

/*****************************************************************************
* 
* omethPhyLinkState - Get link state of requested port
* 
* !!! The returned result is based on the last update of the
*     phy registers. The registers are updated by the function omethPeriodic().
* !!! After the link state of the phy changes it takes several calls of omethPeriodic()
*     until the function omethPhyLinkState() reports the new state
*
* RETURN: phy status
*
*/
phy_stat_enum		omethPhyLinkState
(
 OMETH_H			hEth,		/* handle of ethernet driver, see omethCreate()		*/
 unsigned short		port		/* phy number / port number of integrated hub (0-n)	*/
)
{
	if(hEth==0)					return OMETH_PHY_STATE_INVALID;
	if(port >= hEth->phyCount)	return OMETH_PHY_STATE_INVALID;

	// no link
	if( (hEth->pPhyReg[port].r[1] & PHY_REG1_LINK) == 0 )		return OMETH_PHY_STATE_NOLINK;

	// Full-Bit in advertisement not set -> link can only be half
	if( (hEth->pPhyReg[port].r[4] & PHY_REG4_100TX_FULL) == 0 )	return OMETH_PHY_STATE_HALF;

	// Full-Bit in link partner ability not set -> link can only be half
	if( (hEth->pPhyReg[port].r[5] & PHY_REG4_100TX_FULL) == 0 )	return OMETH_PHY_STATE_HALF;

	return OMETH_PHY_STATE_FULL;
}

/*****************************************************************************
* 
* omethPhyRead - Read Phy register
* 
* !!! The function can only be called in the same context as omethPeriodic()
*
* RETURN: 
*	 0	... no error
*	-1	... error
*		hEth invalid / port too high / reg too high
*
*/
int					omethPhyRead
(
 OMETH_H			hEth,		/* handle of ethernet driver, see omethCreate()		*/
 unsigned short		port,		/* phy number / port number of integrated hub (0-n)	*/
 unsigned short		reg,		/* read register number */
 unsigned short		*pValue		/* ptr to read value */
)
{
	ometh_mii_typ	*pMII = hEth->config.pPhyBase;
	unsigned short	dataBackup;

	if(hEth==0)					return -1;
	if(port >= hEth->phyCount)	return -1;

	while(pMII->cmd.ack & 1);	// wait until busy = 0
	dataBackup = pMII->data;	// backup data in case omethPeriodic() was waiting for a register

	pMII->cmd.req = hEth->phyCmdRead[port] | (reg<<2);
	while(pMII->cmd.ack & 1);						// wait until busy = 0

	if(pValue) *pValue = pMII->data;	// get response
	pMII->data = dataBackup;			// restore backup for next omethPeriodic() call

	return 0;
}

/*****************************************************************************
* 
* omethPhyWrite - Write Phy register
* 
* !!! The function can only be called in the same context as omethPeriodic()
*
* RETURN: 
*	 0	... no error
*	-1	... error
*		hEth invalid / port too high / reg too high
*
*/
int					omethPhyWrite
(
 OMETH_H			hEth,		/* handle of ethernet driver, see omethCreate()		*/
 unsigned short		port,		/* phy number / port number of integrated hub (0-n)	*/
 unsigned short		reg,		/* read register number */
 unsigned short		value		/* value */
)
{
	ometh_mii_typ *pMII = hEth->config.pPhyBase;
	unsigned short	dataBackup;

	if(hEth==0)					return -1;
	if(port >= hEth->phyCount)	return -1;

	while(pMII->cmd.ack & 1);	// wait until busy = 0
	dataBackup = pMII->data;	// backup data in case omethPeriodic() was waiting for a register
		
	pMII->data = value;
	pMII->cmd.req = hEth->phyCmdWrite[port] | (reg<<2);
	while(pMII->cmd.ack & 1);		// wait until busy = 0

	pMII->data = dataBackup;		// restore backup for next omethPeriodic() call
	return 0;
}

/*****************************************************************************
* 
* omethPhyReadNonBlocking - Read Phy register (non blocking)
*
* !!! use either omethPeriodic() or omethPhyReadNonBlocking(), not both !!!
*
*		- omethPeriodic() is required for single port interfaces (no hub) which support
*		  full+half duplex. In this case  omethPhyReadNonBlocking() can not used.
*
*		- omethPhyReadNonBlocking() can only be used for systems forced to either full
*		  or half-duplex
*
*
* RETURN: 
*	 0	... no error : *pValue contains the value of the register passed at the previous call !
*	-1	... blocking : *pValue was not modified
*
*/
int					omethPhyReadNonBlocking
(
 OMETH_H			hEth,		/* handle of ethernet driver, see omethCreate()		*/
 unsigned short		port,		/* phy number / port number of integrated hub (0-n)	*/
 unsigned short		reg,		/* read register number								*/
 unsigned short		*pValue		/* ptr to read value								*/
)
{
	ometh_mii_typ*	pMII = hEth->config.pPhyBase;

	if (pMII->cmd.ack & 1)		return -1;					// busy -> blocking

	*pValue			= pMII->data;							// get response
	pMII->cmd.req	= hEth->phyCmdRead[port] | (reg<<2);	// set new read commando

	return 0;
}

/*****************************************************************************
* 
* omethPeriodic - periodic call of ethernet driver
*
*/
void			omethPeriodic
(
 void
)
{
	OMETH_H			hEth = omethInternal.pPeriodicEth;

	phy_reg_typ		*pPhyReg;	// ptr to registers of processed phy
	ometh_mii_typ	*pMII = hEth->config.pPhyBase;		// ptr to MII

	if(hEth==0)	// initialize periodic if not yet done or if reset by a destroy-call
	{
		// reset phy-state machines of all instances
		for(hEth = omethInternal.pFirstEth ; hEth->pNext ; hEth = hEth->pNext)
		{
			if(hEth==0) return;	// no instance available

			hEth->txVal			= 0;
			hEth->phyReg		= 0;
			hEth->phyPort		= 0;	// start with port 0 if all are done
			hEth->phyLinkCount	= 0;	// reset link counter for next round
			hEth->phyHalfCount	= 0;	// counts all links with half duplex connection
		}

		omethInternal.pPeriodicEth = hEth;		// start periodic phy control with last found instance
		
		pMII = hEth->config.pPhyBase;
		pMII->cmd.req = hEth->phyCmdRead[0];	// initiate read command of phy 0 / register 0

		return;
	}
	else if (pMII->cmd.ack & 1)
	{
		return;	// mii busy
	}

	// check if tx-job for port0/reg4 is pending
	if(hEth->txVal)
	{
		pMII->data    = hEth->txVal;
		pMII->cmd.req = hEth->phyCmdWrite[hEth->txPort] | (hEth->txReg << 2);
		
		hEth->txVal = 0;

		if(hEth->txReg==4)	// reset phy after writing register 4 (write autoneg-restart to register 0)
		{
			hEth->txVal = PHY_REG0_AUTONEG_ENABLE | PHY_REG0_AUTONEG_RESTART;
			hEth->txReg	= 0;
		}

		return;	// return to wait until ack bit goes to 0
	}

	pPhyReg = hEth->pPhyReg + hEth->phyPort;	//  access to phy register structure

	// get data from MII
	((unsigned short*)(pPhyReg))[hEth->phyReg] = pMII->data;
	
	hEth->phyReg++;
	if(hEth->phyReg >= sizeof(phy_reg_typ)/sizeof(unsigned short))
	{
		// count linked ports and switch to force duplex if required
		if(pPhyReg->r[1] & PHY_REG1_LINK)
		{
			hEth->phyLinkCount++;
			
			// link is made with full duplex, change phy register if mac is forced to half duplex
			if((pPhyReg->r[4] & PHY_REG4_100TX_FULL) && (pPhyReg->r[5] & PHY_REG5_100TX_FULL))
			{
				// (do not switch phys to half duplex if mac_type 2 is selected)
				if((hEth->phyLinkMax > 1) && (hEth->config.macType != OMETH_MAC_TYPE_02))
				{
					hEth->txVal  = (pPhyReg->r[4] & ~(unsigned short)PHY_REG4_100TX_FULL) | PHY_REG4_100TX_HALF;
					hEth->txReg  = 4;
					hEth->txPort = hEth->phyPort;
				}
			}
			else
			{
				// counter for half-duplex link if local or remote does not support full duplex
				hEth->phyHalfCount++;
			}
		}
		
		hEth->phyPort++;	// next port ...
		hEth->phyReg = 0;	// ... start at register 0

		// start with first port if all ports were processed
		if(hEth->phyPort >= hEth->phyCount)
		{
			// store max number of linked phys (if >1 the hub will be always activated and all ports
			// linked with full duplex will be reset to connect with half duplex)
			if(hEth->phyLinkCount > hEth->phyLinkMax) hEth->phyLinkMax = hEth->phyLinkCount;

			// set mac to half mode if ever more than 2 ports were active at the same time or the only active
			// port is linked with half duplex
			// (also set half-bit if MAC Type 2 is selected ... this mode always requires a half duplex mac
			if((hEth->phyLinkMax > 1) || (hEth->phyHalfCount > 0) || (hEth->config.macType == OMETH_MAC_TYPE_02))
			{
				hEth->pRegBase->txStatus.setBit = OMETH_REG_HALF;
			}
			else
			{
				hEth->pRegBase->txStatus.clrBit = OMETH_REG_HALF;
			}

			hEth->phyPort		= 0;	// start with port 0 if all are done
			hEth->phyLinkCount	= 0;	// reset link counter for next round
			hEth->phyHalfCount	= 0;	// counts all links with half duplex connection

			// also switch to next adapter
			hEth = hEth->pNext;
			if(hEth==0) hEth = omethInternal.pFirstEth;	// take first instance if this was the last

			omethInternal.pPeriodicEth = hEth;	// save for next periodic call
		}
	}

	// start read command
	pMII->cmd.req = hEth->phyCmdRead[hEth->phyPort] | (hEth->phyReg<<2);
}


/*****************************************************************************
* 
* omethHookCreate - create a new hook for a client
* 
*/
OMETH_HOOK_H	omethHookCreate
(
 OMETH_H		hEth,		/* handle of ethernet driver, see omethCreate() */
 OMETH_HOOK_FCT	*pFct,		/* callback function							*/
 unsigned short	maxPending	/* maximum number of pending buffers			*/
)
{
	OMETH_HOOK_H		hHook;
	unsigned long		len,i;
	ometh_pending_typ	*pQueue;
	ometh_buf_typ		*pBuf;

	if(hEth==0 || pFct==0) return 0;

	if(pFct == OMETH_HOOK_DISABLED) pFct=0;

	// allocate structure array for hook list (one hook for each filter)
	hHook = calloc(sizeof(struct OMETH_HOOK) + maxPending * sizeof(ometh_pending_typ), 1);
	if(hHook==0) return 0;

	// add to the hook list of the instance to be able to destroy complete instance
	hHook->pNext	= hEth->pHookList;
	hEth->pHookList	= hHook;

	hHook->cntOverflow	= 0;
	hHook->pFct			= pFct;
	hHook->hEth			= hEth;

	pQueue				= hHook->free;	// access to first queue entry
	hHook->maxPending	= maxPending;	// overtake max-pending value to hook (just for debug)

	if(maxPending)
	{
		// allocate buffers for this client and add to the buffer list
		len = sizeof(ometh_buf_typ)-sizeof(pBuf->packet.data) + hEth->rxLen;

		len = (len+3)&(~3);		// round up to next multiple of 4

		hHook->pFreeRead = hHook->free;

		pBuf = calloc(len * maxPending, 1);
		if(pBuf==0) return 0;

		hHook->pRxBufBase = pBuf;	// store for destroy function

		// fill list with buffer pointers
		for(i=0; i<maxPending ; i++)
		{
			pQueue->pBuf	= pBuf;
			pQueue->pNext	= pQueue+1;

			pQueue++;
			pBuf = (ometh_buf_typ*)((size_t)pBuf + len);	// next buffer
		}
	}

	pQueue->pNext		= hHook->free;	// last element points to first
	hHook->pFreeWrite	= pQueue;		// also set freeWrite-pointer in case someone installs a hook with maxpending=0 but still calls omethPacketFree
	
	return hHook;
}

/*****************************************************************************
* 
* omethHookSetFunction - change hook function of existing hook
* 
*/
int				omethHookSetFunction
(
 OMETH_HOOK_H	hHook,		/* handle of existing hook				*/
 OMETH_HOOK_FCT	*pFct		/* new callback function				*/
)
{
	if(hHook==0) return -1;

	if(pFct == OMETH_HOOK_DISABLED) pFct=0;
	
	hHook->pFct = pFct;

	return 0;
}

/*****************************************************************************
* 
* omethFilterCreate - create a filter entry for a hook
* 
*/
OMETH_FILTER_H	omethFilterCreate
(
 OMETH_HOOK_H	hHook,			/* handle of a ethernet client				*/
 void			*arg,			/* argument for hook function				*/
 void			*pMask,			/* ptr to array with 17 mask bytes			*/
 void			*pValue			/* ptr to array with 17 compare values		*/
)
{
	OMETH_FILTER_H	hFilter;
	OMETH_H			hEth;
	int				i;

	if(hHook==0) return 0;

	hEth = hHook->hEth;	// access to driver handle

	// find available filter entry
	// search for free hook structure
	hFilter = hEth->pFilterList;

	for(i=0;i<hEth->nbFilter - hEth->nbFilterX;i++)
	{
		if(hFilter->hHook == 0)	// free filter found
		{
			hFilter->arg	= arg;		// hook argument
			hFilter->hHook	= hHook;	// reference to hook

			omethFilterSetPattern(hFilter, pMask, pValue);	// copy filter mask and values to the filter
			
			FILTER_SET_FLAG(hFilter->pFilterData, CMD_FILTER_ON);	// enable filter

			return hFilter;
		}

		hFilter++;
	}

	return 0;
}

/*****************************************************************************
* 
* omethFilterCreateX - create a x-filter entry for a hook
* 
*/
OMETH_FILTER_H	omethFilterCreateX
(
 OMETH_HOOK_H	hHook,			/* handle of a ethernet client				*/
 void			*arg,			/* argument for hook function				*/
 void			*pMask,			/* ptr to array with 17 mask bytes			*/
 void			*pValue			/* ptr to array with 17 compare values		*/
)
{
	OMETH_FILTER_H	hFilter;
	OMETH_H			hEth;
	int				i;

	if(hHook==0) return 0;

	hEth = hHook->hEth;	// access to driver handle

	// find available filter entry
	// search for free hook structure
	hFilter = hEth->pFilterList + (hEth->nbFilter - hEth->nbFilterX);
	i       = hEth->nbFilterX;

	while(i--)
	{
		if(hFilter->hHook == 0)	// free filter found
		{
			hFilter->arg	= arg;		// hook argument
			hFilter->hHook	= hHook;	// reference to hook

			omethFilterSetPattern(hFilter, pMask, pValue);	// copy filter mask and values to the filter
			
			FILTER_SET_FLAG(hFilter->pFilterData, CMD_FILTER_ON);	// enable filter

			return hFilter;
		}

		hFilter++;
	}

	return 0;
}

/*****************************************************************************
* 
* omethFilterSetPattern - sets a new filter pattern (mask/value) to an existing
*							filter
* 
*/
int				omethFilterSetPattern
(
 OMETH_FILTER_H	hFilter,
 void			*pMask,			/* ptr to array with 17 mask bytes			*/
 void			*pValue			/* ptr to array with 17 compare values		*/
)
{
	ometh_filter_data_typ	*pFilterData;
	ometh_filter_entry_typ	*pFilterEntry;

	OMETH_H		hEth;
	int			i;

	if(hFilter==0 || pMask==0 || pValue==0) return -1;

	hEth = hFilter->hHook->hEth;
	
	if(hEth==0) return -1;

	pFilterData = hFilter->pFilterData;

	// disable filter
	*pFilterData->pCommand = pFilterData->cmd & ~CMD_FILTER_ON;

	pFilterEntry = pFilterData->pFilterWriteOnly;	// access to filter data

	i = pFilterData->len;

	while(i--)
	{
		pFilterEntry->mask  = *(unsigned char*)pMask;
		pFilterEntry->value = *(unsigned char*)pValue;

		pMask  = (unsigned char*)pMask  + 1;
		pValue = (unsigned char*)pValue + 1;

		pFilterEntry++;
	}

	// restore old command flag
	*pFilterData->pCommand = pFilterData->cmd;

	return 0;
}

/*****************************************************************************
* 
* omethFilterSetByteValue - sets one byte (value, not mask) to an filter
*
*	! the function does not disable the filter while changing the value
*		->	if more than 1 byte should be changed consistent omethFilterSetPattern()
*			shall be used instead
*
*/
void			omethFilterSetByteValue
(
 OMETH_FILTER_H	hFilter,		/* filter handle									*/
 unsigned long	nOffset,		/* offset in the filterarray						*/
 unsigned char	nValue			/* value to set										*/
)
{
	hFilter->pFilterData->pFilterWriteOnly[nOffset].value = nValue;
}

/*****************************************************************************
* 
* omethFilterSetArgument - sets a new argument which will be passed to the callback
*
*/
int				omethFilterSetArgument
(
 OMETH_FILTER_H	hFilter,
 void			*arg
)
{
	if(hFilter==0) return -1;	// verify filter
	hFilter->arg = arg;			// overtake new argument
	return 0;
}

/*****************************************************************************
* 
* omethFilterSetHook - sets a new hook for the filter
*
*/
int				omethFilterSetHook
(
 OMETH_FILTER_H	hFilter,
 OMETH_HOOK_H	hHook			/* handle from omethHookCreate()		*/
)
{
	if(hFilter==0) return -1;	// verify filter
	hFilter->hHook = hHook;		// overtake new hook
	return 0;
}

/*****************************************************************************
* 
* omethFilterSetNoMatchIRQ - set/clear irq enable for no_match event on this filter
*
*	If a received frame does not match any of the installed filters, the NoMatch-IRQ
*	will be generated if enabled with this function (can be applied to any of the installed
*	filters)
* 
*/
int				omethFilterSetNoMatchIRQ
(
 OMETH_FILTER_H	hFilter,		/* filter handle							*/
 int			irqEnable		/* TRUE: IRQ will be triggerd if frame does not match this filter */
)
{
	ometh_filter_data_typ	*pFilterData;

	if (hFilter==0) return -1;

	pFilterData = hFilter->pFilterData;

	if (irqEnable)
	{
		// set RxMissFlag
		FILTER_SET_FLAG(hFilter->pFilterData, CMD_RX_NOMATCH);
	}
	else
	{
		// clear RxMissFlag
		FILTER_CLEAR_FLAG(hFilter->pFilterData, CMD_RX_NOMATCH);
	}
	// set/clear RxMiss Flag
	*pFilterData->pCommand = pFilterData->cmd;

	return 0;
}

/*****************************************************************************
* 
* omethFilterEnable - enable filter
*
*/
int				omethFilterEnable
(
 OMETH_FILTER_H	hFilter
)
{
	// turn filter entry on
	FILTER_SET_FLAG(hFilter->pFilterData, CMD_FILTER_ON);
	return 0;
}

/*****************************************************************************
* 
* omethFilterDisable - disable filter
*
*/
int				omethFilterDisable
(
 OMETH_FILTER_H	hFilter
)
{
	// disable filter
	FILTER_CLEAR_FLAG(hFilter->pFilterData, CMD_FILTER_ON);
	return 0;
}

/*****************************************************************************
* 
* omethSetSCNM - set SCNM filter
* 
*/
int				omethSetSCNM
(
 OMETH_H		hEth,		/* handle of ethernet driver, see omethCreate() */
 OMETH_FILTER_H	hFilter		/* filter handle								*/
)
{
	if(hEth==0) return -1;

	if(hFilter)
	{
		// verify if filter belongs to this driver instance (and check for invalid handle)
		// if the handle is not valid the queue transmitter is disabled
		if(hFilter == OMETH_INVALID_FILTER || hEth != hFilter->hHook->hEth )
		{
			hEth->pFilterSCNM = 0;
			hEth->txQueueEnable = 0;	// queue sending is disabled
			return -2;
		}

		// Auto-TX filter can not be declared as SCNM filter
		if(hFilter->pTxInfo) return -1;

		// return directly if the given filter is already used as SCNM filter
		if(hEth->pFilterSCNM == hFilter->pFilterData) return 0;

		// clear sync flag in old filter if still set
		if(hEth->pFilterSCNM) FILTER_CLEAR_FLAG(hEth->pFilterSCNM, CMD_FILTER_SYNC);

		hEth->pFilterSCNM = hFilter->pFilterData;				// access to new filter
		
		hEth->pRegBase->txStatus.setBit = OMETH_REG_SYNC;		// set sync flag in control register

		FILTER_SET_FLAG(hEth->pFilterSCNM, CMD_FILTER_SYNC);	// set sync flag in this filter
	}
	else
	{
		// clear sync flag in old filter if still set
		if(hEth->pFilterSCNM) FILTER_CLEAR_FLAG(hEth->pFilterSCNM, CMD_FILTER_SYNC);

		// clear sync flag in control register
		hEth->pRegBase->txStatus.clrBit = OMETH_REG_SYNC;

		hEth->pFilterSCNM = 0;
	}

	hEth->txQueueEnable = 1;	// queue sending is allowed if valid filter is set or if we run in basic ethernet mode

	return 0;
}

/*****************************************************************************
* 
* allocTxDescriptor - Allocates a tx descriptor for auto-response handling
* 
* RETURN: 0..error
*	
*/
static ometh_tx_info_typ* allocTxDescriptor(OMETH_H hEth)
{
	ometh_tx_info_typ	*pInfo,*pNew;

	// start with last buffer
	if(hEth->pTxInfo[1])	// take 2nd tx queue if available
	{
		pInfo = hEth->pTxInfo[1];
	}
	else					// otherwise take 1st tx queue
	{
		pInfo = hEth->pTxInfo[0];
	}


	// error if no buffers available
	if(pInfo == pInfo->pNext) return 0;

	// find info structure one before the last
	while(1)
	{
		pNew = pInfo->pNext;
		if(pNew->flags1 & FLAGS1_LAST) break;	// last found
		pInfo = pNew;
	}

	pInfo->flags1	= pNew->flags1;	// overtake last-flags
	pInfo->pNext	= pNew->pNext;	// overtake link to next info structure

	pNew->flags1	= 0;			// auto answer buffers have no flags
	pNew->pNext		= pInfo;		// backwards-link of auto answer structures

	return pNew;
}

/*****************************************************************************
* 
* omethResponseInitBuf - initialize a installed filter for auto response frames
*						and provice change buffers
*						(this avoids that the funciton returns 0-pointers at the first
*						1 or 2 calls)
*/
int				omethResponseInitBuf
(
 OMETH_FILTER_H	hFilter,		/* filter handle							*/
 ometh_packet_typ	*pPacket1,	/* spare packet for change buffer			*/
 ometh_packet_typ	*pPacket2	/* spare packet for change buffer			*/
)
{
	OMETH_H				hEth;
	ometh_tx_info_typ	*pTxInfo;

	if(	hFilter==0 ||			// hFilter invalid
		hFilter->hHook==0 ||	// no hook to this filter, filter not valid
		hFilter->pTxInfo		// already initialized
	  )
	{
		return -1;
	}

	hEth = hFilter->hHook->hEth;						// get driver instance from hook
	
	if(hEth->pRegBase->txStatus.value & OMETH_REG_RUN) return -1;	// new setup in run mode not allowed

	if(hFilter->pFilterData == hEth->pFilterSCNM) return -1;		// SCNM filter can not be auto-response

	pTxInfo = allocTxDescriptor(hEth);		// allocate tx descriptor

	if(pTxInfo==0) return -1;				// no desriptor available

	hFilter->pTxInfo = pTxInfo;				// this is the next autoresponse descriptor

	// preset spare buffers for exchange
	pTxInfo->pProduced[0] = pPacket1;
	pTxInfo->pProduced[2] = pPacket2;

	// prepare filter command for auto response
	FILTER_SET_FLAG(hFilter->pFilterData, pTxInfo->index | CMD_FILTER_ON);

	hFilter->pFilterData->txEnableRequest=1;	// auto tx flag should be set at next omethResponseSet
	return 0;
}

/*****************************************************************************
* 
* omethResponseInit - initialize a installed filter for auto response frames
* 
*/
int				omethResponseInit
(
 OMETH_FILTER_H	hFilter		/* filter handle								*/
)
{
	return omethResponseInitBuf(hFilter,0,0);
}

/*****************************************************************************
* 
* omethResponseSet - set new packet for response frame
* 
*/
ometh_packet_typ	*omethResponseSet
(
 OMETH_FILTER_H		hFilter,	/* filter handle							*/
 ometh_packet_typ	*pPacket	/* packet which shall be responded			*/
)
{
	ometh_tx_info_typ	*pInfo;
	ometh_desc_typ		*pDesc;
	unsigned short		len;
	unsigned long		newChgIndex,freeChgIndex;

	if(pPacket==0)			return OMETH_INVALID_PACKET;	// invalid packet
	if(hFilter==0)			return OMETH_INVALID_PACKET;	// hFilter invalid

	pInfo = hFilter->pTxInfo;		// access to assigned tx descriptor

	if(pInfo==0)			return OMETH_INVALID_PACKET;	// response not initialized

	// generate a new change index, different to the old index values of read and write
	freeChgIndex = pInfo->chgIndexWrite;
	newChgIndex = chgIndexTab[freeChgIndex][pInfo->chgIndexRead];

	pDesc = pInfo->pDesc;			// access to hardware

	if(hFilter->pFilterData->len == OMETH_FILTER_LEN)	// normal filter
	{
		// set new packet for auto response
		len = pPacket->length;
		if(len < OMETH_MIN_TX_FRAME) len = OMETH_MIN_TX_FRAME;

		// write length before ptr only if new packet is bigger
		// (if new frame is smaller it could happen that the length is overtaken first and the mac sends
		//  the old ptr with too less bytes)
		if(len > pDesc->len) pDesc->len = len;

		// overtake buffer to descriptor
		pDesc->pData	= (unsigned long)&pPacket->data | chgIndexHighBit[newChgIndex];
	}
	else	// x-filter
	{
		// set new packet for auto response
		len = pPacket->length - OMETH_X_OFFSET;
		if(len < OMETH_MIN_TX_FRAME-OMETH_X_OFFSET) len = OMETH_MIN_TX_FRAME-OMETH_X_OFFSET;

		// write length before ptr only if new packet is bigger
		// (if new frame is smaller it could happen that the length is overtaken first and the mac sends
		//  the old ptr with too less bytes)
		if(len > pDesc->len) pDesc->len = len;

		// overtake buffer to descriptor
		pDesc->pData	= (((unsigned long)&pPacket->data)+OMETH_X_OFFSET) | chgIndexHighBit[newChgIndex];
	}

	pDesc->len		= len;
	
	// decide which buffer can be released
	pInfo->chgIndexRead = pDesc->txStart & 3;	// get current chg index from descritpor

	// if buffer was not overtaken just during the last lines we have to take another buffer to pass back to the user
	if (pInfo->chgIndexRead != newChgIndex) freeChgIndex = chgIndexTab[newChgIndex][pInfo->chgIndexRead];

	pInfo->pProduced[newChgIndex] = pPacket;	// remember the sent packet to free it as soon as possible
	pInfo->chgIndexWrite = newChgIndex;			// the next cycle has to know the change index which was written the last time to generate a different one

	// writing to flags1 is only allowed if owner is not yet set, otherwise collision with tx-irq-access to this field
	if ((pDesc->flags.byte.high & FLAGS1_OWNER) == 0) pDesc->flags.byte.high = FLAGS1_OWNER;

	// check tx-enable of filter and turn on if not already done
	if(hFilter->pFilterData->txEnableRequest)
	{
		hFilter->pFilterData->txEnableRequest = 0;
		FILTER_SET_FLAG(hFilter->pFilterData, CMD_TX_ENABLE);
	}

	return pInfo->pProduced[freeChgIndex];	// release the buffer which was produced at the last cycle
}

/*****************************************************************************
* 
* omethResponseDisable - disable auto response frame
* 
*/
int					omethResponseDisable
(
 OMETH_FILTER_H		hFilter		/* filter handle				*/
)
{
	if(hFilter==0) return -1;	// hFilter invalid

	// clear enable flag for omethResponseSet function
	hFilter->pFilterData->txEnableRequest = 0;

	FILTER_CLEAR_FLAG(hFilter->pFilterData, CMD_TX_ENABLE);

	return 0;
}

/*****************************************************************************
* 
* omethResponseEnable - enable auto response frame
* 
*/
int					omethResponseEnable
(
 OMETH_FILTER_H		hFilter		/* filter handle				*/
)
{
	ometh_desc_typ		*pDesc;

	// hFilter invalid
	if(hFilter==0 || hFilter->pTxInfo==0) return -1;

	// get tx descriptor pointer and validate
	pDesc = hFilter->pTxInfo->pDesc;
	if(pDesc==0) return -1;

	// make sure the tx-enable bit will be set at the next omethResponseSet call
	hFilter->pFilterData->txEnableRequest = 1;

	// if owner bit of descriptor is on ... turn auto tx flag on immediately
	if(pDesc->flags.byte.high & FLAGS1_OWNER)
	{
		FILTER_SET_FLAG(hFilter->pFilterData, CMD_TX_ENABLE);
	}

	return 0;
}

/*****************************************************************************
* 
* omethTransmit - transmit a buffer to the network
* 
*/
unsigned long		omethTransmit
(
 OMETH_H			hEth,		/* handle of ethernet driver, see omethCreate() */
 ometh_packet_typ	*pPacket,	/* packet to be sent							*/
 OMETH_BUF_FREE_FCT	*pFct		/* function ptr to sent-ack-function			*/
)
{
	OMETH_TRANSMIT( 0 , 0 , 0 , 0);		// add frame to send queue
}

/*****************************************************************************
* 
* omethTransmitArg - transmit a buffer to the network with argument for tx-callback
* 
*/
unsigned long		omethTransmitArg
(
 OMETH_H				hEth,		/* handle of ethernet driver, see omethCreate() */
 ometh_packet_typ		*pPacket,	/* packet to be sent							*/
 OMETH_BUF_FREE_FCT_ARG	*pFct,		/* function ptr to sent-ack-function			*/
 void					*arg		/* argument which will be passed to free function */
)
{
	OMETH_TRANSMIT( arg, 0, 0 , 0);		// add frame to send queue
}

/*****************************************************************************
* 
* omethTransmitArg2 - Transmit with 2nd transmit queue (everything else same as omethTransmit)
* 
*/
unsigned long		omethTransmitArg2
(
 OMETH_H				hEth,		/* handle of ethernet driver, see omethCreate() */
 ometh_packet_typ		*pPacket,	/* packet to be sent							*/
 OMETH_BUF_FREE_FCT_ARG	*pFct,		/* function ptr to sent-ack-function			*/
 void					*arg		/* argument which will be passed to free function */
)
{
	OMETH_TRANSMIT( arg, 0, 0 , 1);		// add frame to send queue
}

/*****************************************************************************
* 
* omethTransmitTime - transmit a buffer to the network at defined time
*	(same like omethTransmitArg, just the optional argument is additional)
* 
*/
unsigned long		omethTransmitTime
(
 OMETH_H				hEth,		/* handle of ethernet driver, see omethCreate() */
 ometh_packet_typ		*pPacket,	/* packet to be sent							*/
 OMETH_BUF_FREE_FCT_ARG	*pFct,		/* function ptr to sent-ack-function			*/
 void					*arg,		/* argument which will be passed to free function */
 unsigned long			time		/* timestamp									*/
)
{
	// add frame to send queue (with additional flag to set start time)
	OMETH_TRANSMIT(arg, time, FLAGS1_START_TIME , 0);
}

/*****************************************************************************
* 
* omethTransmitPending - get number of pending transmit frames in queue
* 
*/
unsigned char		omethTransmitPending
(
 OMETH_H				hEth		/* handle of ethernet driver, see omethCreate() */
)
{
	return hEth->cntTxQueueIn - hEth->cntTxQueueOut;
}

/*****************************************************************************
* 
* omethStart - start ethernet driver
* 
*/
void			omethStart
(
 OMETH_H		hEth		/* handle of ethernet driver, see omethCreate() */
)
{
	unsigned short setBit;

	if(hEth==0) return;
	
	if(hEth->clearPendingIrqAtStart)
	{
		hEth->clearPendingIrqAtStart=0;	// important for debugging .. clear pending irqs from last activation
		
		// quit pending irqs and clear lost-bit
		while(hEth->pRegBase->rxStatus.value & OMETH_REG_PENDING) hEth->pRegBase->rxStatus.clrBit = OMETH_REG_IQUIT;
		while(hEth->pRegBase->txStatus.value & OMETH_REG_PENDING) hEth->pRegBase->txStatus.clrBit = OMETH_REG_IQUIT;
	}

	hEth->pRegBase->rxStatus.clrBit = OMETH_REG_LOST;

	setBit = OMETH_REG_RUN | OMETH_REG_IE;

	hEth->pRegBase->txStatus.setBit = setBit;

	#ifdef OMETH_TRACE
		setBit = setBit | OMETH_REG_DIAG;
	#endif

	hEth->pRegBase->rxStatus.setBit = setBit;
}

/*****************************************************************************
* 
* omethStop - stop ethernet driver
*
*/
void			omethStop
(
 OMETH_H		hEth		/* handle of ethernet driver, see omethCreate() */
)
{
	if(hEth==0) return;

	// clear run bits for rx and tx
	hEth->pRegBase->rxStatus.clrBit = OMETH_REG_RUN | OMETH_REG_IE;
	hEth->pRegBase->txStatus.clrBit = OMETH_REG_RUN | OMETH_REG_IE;
}

/*****************************************************************************
* 
* omethGetTimestamp - get timestamp of a received packet
* 
*/
unsigned long	omethGetTimestamp
(
 ometh_packet_typ	*pPacket	/* address of rx packet*/
)
{
	if(pPacket==0) return 0;

	return GET_TYPE_BASE( ometh_buf_typ, packet, pPacket)->timeStamp;
}

/*****************************************************************************
* 
* omethPacketFree - pass rx packet back to ethernet driver
* 
*/
void		omethPacketFree
(
 ometh_packet_typ	*pPacket	/* address of rx packet	*/
)
{
	ometh_pending_typ	*pQueue;	// ptr to buffer queue
	OMETH_HOOK_H		hHook;		// hook handle leading to the data area of the buffer
	ometh_buf_typ		*pBuf;		// ptr to buffer

	// access to header of packet
	pBuf  = GET_TYPE_BASE( ometh_buf_typ, packet, pPacket);
	hHook = pBuf->hHook;

	if(pPacket==0 || hHook==0) return;	// invalid

	pQueue = hHook->pFreeWrite;

	pQueue->pBuf = pBuf;				// pass buffer back to the hooks buffer list (buffer queue)
		
	hHook->pFreeWrite = pQueue->pNext;	// switch to next queue element
}

/*****************************************************************************
* 
* omethStatistics - get ptr to statistics of ethernet adapter
* 
*/
ometh_stat_typ	*omethStatistics
(
 OMETH_H		hEth		/* handle of ethernet driver, see omethCreate() */
)
{
	if(hEth==0) return 0;

	return &hEth->stat;
}

#if (OMETH_ENABLE_SOFT_IRQ==1)
	/*****************************************************************************
	* 
	* omethSoftwareIrqSet/Start - set/start software-irq routine on tx-irq level
	*
	*	!! this function does not work on all MAC designs !!
	* 
	*/
	void		omethSoftwareIrqSet
	(
	OMETH_H			hEth,		/* handle of ethernet driver, see omethCreate() */
	OMETH_BUF_FREE_FCT	*pFct		/* function ptr to soft-irq function			*/
	)
	{
		if(hEth) hEth->pFctSoftIrq = pFct;
	}

	void		omethSoftwareIrqStart
	(
	OMETH_H			hEth		/* handle of ethernet driver, see omethCreate() */
	)
	{
		if(hEth) hEth->pRegBase->txStatus.setBit = OMETH_REG_SOFTIRQ;	
	}
#endif


#if (OMETH_ENABLE_RX_HANDSHAKE==1)
	/*****************************************************************************
	* 
	* omethSetRxHandshake - set rx-handshake
	* 
	* mode : OMETH_RX_HANDSHAKE_....
	*/
	void	omethSetRxHandshake
	(
	OMETH_H		hEth,		/* handle of ethernet driver, see omethCreate() */
	unsigned short	mode
	)
	{
		// use long-access to write element 'setBit' and 'clrBit' of the rx-status with 1 processor cycle
		unsigned short *pSetClrBit = &hEth->pRegBase->rxStatus.setBit;
		unsigned short clrBitValue;

		if(hEth==0) return;

		switch(mode)
		{
			case OMETH_RX_HANDSHAKE_RECEIVE_ALL:
				clrBitValue = (~OMETH_REG_RXMODE_DEFAULT & OMETH_REG_RXMODE_MASK);
				break;
			case OMETH_RX_HANDSHAKE_BLOCK_LOW:
				clrBitValue = (~OMETH_REG_RXMODE_HIGH_PRIO & OMETH_REG_RXMODE_MASK);
				break;
			case OMETH_RX_HANDSHAKE_BLOCK_ALL:
				clrBitValue = (~OMETH_REG_RXMODE_BLOCK & OMETH_REG_RXMODE_MASK);
				break;
			case OMETH_RX_HANDSHAKE_BLOCK_AUTO:
				clrBitValue = (~OMETH_REG_RXMODE_AUTO & OMETH_REG_RXMODE_MASK);
				break;
			default:
				return;
		}

		pSetClrBit[0] = OMETH_REG_RXMODE_MASK;
		pSetClrBit[1] = clrBitValue;
	}
#endif

/*****************************************************************************
* 
* omethRxIrqHandler - to be called from the MAC RX interrupt
* 
*/
void			omethRxIrqHandler
(
 OMETH_H		hEth		/* handle of ethernet driver, see omethCreate() */
)
{
	// Timing measurement on PX32 IP20 Eval Board
	//
	// IRQ for hook with maxPending==0 and empty hook : 3,5 us
	// IRQ for hook with maxPending==x and empty hook : 5 us    (4 us if no buffer available)

	OMETH_HOOK_H			hHook;
	OMETH_FILTER_H			hFilter;
	ometh_buf_typ			*pRxBuf;
	ometh_pending_typ		*pQueue;					// ptr to buffer queue
	ometh_rx_info_typ		*pInfo;
	ometh_desc_typ			*pDesc;
	unsigned long			flags=0;

	pInfo = hEth->pRxNext;	// access to next rx info structure
	pDesc = pInfo->pDesc;	// access to rx descriptor

	if(pDesc->flags.byte.high & FLAGS1_OWNER) return;	// leave IRQ if no rx-buffer available

	// !! here hHook is abused to save stack-variables (until required for its original reason)
	//	(faster processing)
	hHook = (OMETH_HOOK_H)&hEth->pRegBase->rxStatus;
	((ometh_status_typ*)hHook)->clrBit = OMETH_REG_IQUIT;

	flags = pDesc->flags.word;

	// access to buffer structure and set packet length
	pRxBuf = GET_TYPE_BASE( ometh_buf_typ , packet.data, pDesc->pData);

	pRxBuf->packet.length = (unsigned long)pDesc->len - 4;	// length without checksum
	pRxBuf->timeStamp     = pDesc->time;					// overtake timestamp to packet header

	if(((ometh_status_typ*)hHook)->value & OMETH_REG_LOST)
	{
		((ometh_status_typ*)hHook)->clrBit = OMETH_REG_LOST;

		#ifndef OMETH_TRACE	// do not compile the following lines to increase trace performance
			hEth->stat.rxLost++;
		#else
			pTraceFctInt(hEth->config.adapter, pRxBuf, (flags & ETH_FLAGS_ERROR_MASK) | ETH_FLAGS_RX_LOST);
		#endif
	}
	else
	{
		#ifdef OMETH_TRACE
			pTraceFctInt(hEth->config.adapter, pRxBuf, flags & ETH_FLAGS_ERROR_MASK);
		#endif
	}

	if( (flags & (ETH_FLAGS_CRC_ERROR|ETH_FLAGS_OVERSIZE)) == 0 )
	{
		#ifndef OMETH_TRACE	// do not compile the following lines to increase trace performance
			hEth->stat.rxOk++;
		#endif

		hFilter	= hEth->pFilterList + ((flags & 0xF0) >> 4);		// access to filter info structure
		hHook	= hFilter->hHook;		// access to hook function of this filter

		if(hHook->pFct)
		{
			if(hHook->pFreeRead==0)
			{
				// call hook function (no release function is passed because hook is not allowed to queue this frame)
				hHook->pFct(hFilter->arg, &pRxBuf->packet, 0);
			}
			else
			{
				pRxBuf->hHook = hHook;				// overtake hook handle for free function

				pQueue = hHook->pFreeRead;

				if(pQueue->pBuf)		// buffer available for exchange
				{
					// call hook function
					if(hHook->pFct(hFilter->arg, &pRxBuf->packet, omethPacketFree) == 0)
					{
						// use new frame for next rx at this descriptor
						pDesc->pData = (unsigned long)&pQueue->pBuf->packet.data;

						pQueue->pBuf = 0;					// remove buffer from list

						hHook->pFreeRead = pQueue->pNext;	// switch to next buffer in queue
					}
				}
				else
				{		
					hHook->cntOverflow++;			// too many buffers pending, buffer is not passed to the client
					hEth->stat.rxHookOverflow++;	// also count in user accessible statistic structure
				}
			}
		}
		else
		{
			#ifndef OMETH_TRACE	// do not compile the following lines to increase trace performance
				hEth->stat.rxHookDisabled++;
			#endif
		}
	}
	else
	{
		#ifndef OMETH_TRACE	// do not compile the following lines to increase trace performance
			if(flags & ETH_FLAGS_OVERSIZE)	hEth->stat.rxOversize++;
			if(flags & ETH_FLAGS_CRC_ERROR)	hEth->stat.rxCrcError++;
		#endif
	}

	// pass buffer to MAC
	pDesc->len				= hEth->rxLen;		// set maximum length for this buffer
	pDesc->flags.byte.high	= pInfo->flags1;	// set owner and last flag
	hEth->pRxNext			= pInfo->pNext;		// switch to next info for next rx
}

/*****************************************************************************
* some changes (marked) were added by Joerg Zelenka 2009/03/23
* omethTxIrqHandler - to be called from the MAC TX interrupt
* 
*/
void			omethTxIrqHandler
(
 OMETH_H		hEth		/* handle of ethernet driver, see omethCreate() */
)
{    
	ometh_tx_info_typ	*pInfo	= hEth->pTxFree[0];	// access to next tx info structure;
	ometh_desc_typ		*pDesc	= pInfo->pDesc;		// access to tx descriptor
	unsigned long		i;


    #if (OMETH_ENABLE_SOFT_IRQ==1)
		// check if soft IRQ was triggered
		if(hEth->pRegBase->txStatus.value & OMETH_REG_SOFTIRQ)
		{
			hEth->pRegBase->txStatus.clrBit = OMETH_REG_SOFTIRQ;
			
			hEth->pFctSoftIrq(0);		// call user function for software IRQ
	        
			return;
		}
	#endif

	// return if no tx irq pending on this interface
	if((hEth->pRegBase->txStatus.value & OMETH_REG_PENDING) == 0) return;

	while(1)
	{
        // if it was not the last queue-descriptor .. it was an auto answer buffer
		/* BEGIN CHANGES BY JOERG ZELENKA 2009/03/23 */
        while(1) //if(pDesc->flags.byte.high & FLAGS1_SENT)
		{
            if(!(pDesc->flags.byte.high & FLAGS1_SENT))
                break;
            
            i = (unsigned long)pDesc->flags.word & 0x0F;	// collisions of this frame
			hEth->stat.txDone[i]++;							// count transmits depending on occured collisions
			hEth->stat.txCollision += i;					// count collisions

			// call free function with ptr
			if(pInfo->pFctFree)
			{
				pInfo->pFctFree( GET_TYPE_BASE(ometh_packet_typ, data, pDesc->pData), pInfo->fctFreeArg, pDesc->time );
			}

			hEth->pTxFree[0] = pInfo->pNext;	// switch free ptr to next info strucutre

			hEth->cntTxQueueOut++;

			pDesc->flags.byte.high	= 0;				// clear sent-flag
			pDesc->pData			= 0;				// mark buffer as free
            
            // ---
            //next lines were added by Joerg Zelenka
            pInfo = hEth->pTxFree[0];
            pDesc = pInfo->pDesc;
            // ---
            
            //break; //put break to top of while, into if packet was sent...
		}
        /* END CHANGES BY JOERG ZELENKA 2009/03/23 */

		pInfo = hEth->pTxFree[1];	// try second priority queue
		
		if(pInfo) // check if 2nd transmit queue existing
		{
			pDesc	= pInfo->pDesc;		// access to tx descriptor

			// if it was not the last queue-descriptor .. it was an auto answer buffer
			if(pDesc->flags.byte.high & FLAGS1_SENT)
			{
				i = (unsigned long)pDesc->flags.word & 0x0F;	// collisions of this frame
				hEth->stat.txDone[i]++;							// count transmits depending on occured collisions
				hEth->stat.txCollision += i;					// count collisions

				// call free function with ptr
				if(pInfo->pFctFree)
				{
					pInfo->pFctFree( GET_TYPE_BASE(ometh_packet_typ, data, pDesc->pData), pInfo->fctFreeArg, pDesc->time );
				}

				hEth->pTxFree[1] = pInfo->pNext;	// switch free ptr to next info strucutre

				hEth->cntTxQueueOut++;

				pDesc->flags.byte.high	= 0;				// clear sent-flag
				pDesc->pData			= 0;				// mark buffer as free
				break;
			}
		}

		// search through all auto transmit descriptors to find the one causing the irq
		pInfo = hEth->pTxAuto;
		while(1)
		{
			if(pInfo->flags1 & FLAGS1_LAST)	// this is already the last-descriptor of the queue
			{
				// return if no sent buffer found
				hEth->stat.txSpuriousInt++;

				hEth->pRegBase->txStatus.clrBit = OMETH_REG_IQUIT;	// quit tx irq
				return;
			}

			pDesc = pInfo->pDesc;					// access to descriptor

			if(pDesc->flags.byte.high & FLAGS1_SENT) break;	// descriptor found, break loop

			pInfo = pInfo->pNext;					// switch to next tx info
		}

		i = (unsigned long)pDesc->flags.word & 0x0F;	// collisions of this frame
		hEth->stat.txDone[i]++;							// count transmits depending on occured collisions
		hEth->stat.txCollision += i;					// count collisions

		pDesc->flags.byte.high = FLAGS1_OWNER;	// set owner flag for next auto tx
		break;
	}

	hEth->pRegBase->txStatus.clrBit = OMETH_REG_IQUIT;	// quit tx irq
}

/*****************************************************************************
* 
* omethRxIrqHandlerMux - to be called from the MAC RX interrupt (multiplexer for all ethernet interfaces)
* 
*/
void			omethRxIrqHandlerMux
(
 void
)
{
	OMETH_H		hEth = omethInternal.pFirstEth;

	// call irq handler for all ethernet adapters
	while(hEth)
	{
		omethRxIrqHandler(hEth);
		hEth=hEth->pNext;
	}
}

/*****************************************************************************
* 
* omethNoFilterMatchIrqHandler - no filter match irq handler
*
*	Calls the function OMETH_NOFILTERMATCHIRQ_HOOK_FCT() which must be defined
*	in ometh_target.h if a received frame does not match any filter and the 
*	NoMatch-IRQ is enabled.
* 
*/
void			omethNoFilterMatchIrqHandler
(
 OMETH_H		hEth		/* handle of ethernet driver, see omethCreate() */
)
{
	#ifdef OMETH_NOFILTERMATCHIRQ_HOOK_FCT
		OMETH_NOFILTERMATCHIRQ_HOOK_FCT((void*)hEth->pRxNext->pDesc->pData);
	#endif

	hEth->pRegBase->rxStatus.clrBit = OMETH_REG_RX_NOMATCH;	// clear pending irq
}

/*****************************************************************************
* 
* omethTxIrqHandlerMux - to be called from the MAC TX interrupt (multiplexer for all ethernet interfaces)
* 
*/
void			omethTxIrqHandlerMux
(
 void
)
{
	OMETH_H		hEth = omethInternal.pFirstEth;

	// call irq handler for all ethernet adapters
	while(hEth)
	{
		omethTxIrqHandler(hEth);
		hEth=hEth->pNext;
	}
}


// free function which calls the real free function only if p is not 0
static void freePtr(void *p)
{
	if(p) free(p);
}

/*****************************************************************************
* 
* omethDestroy - stop ethernet driver instance and free all allocated resources
* 
*/
int				omethDestroy
(
 OMETH_H		hEth		/* handle of ethernet driver, see omethCreate() */
)
{
	OMETH_H			hFind = omethInternal.pFirstEth;
	OMETH_HOOK_H	hHook,hFree;

	if(hEth==0) return -1;

	// find driver instance in list and remove
	if(hEth==hFind)	// first instance should be removed
	{
		omethInternal.pFirstEth = hEth->pNext;
	}
	else
	{
		while(hFind->pNext)
		{
			if(hFind->pNext == hEth)
			{
				// driver instance found ... take instance out of the internal instance list
				hFind->pNext = hEth->pNext;	
				break;		
			}

			hFind = hFind->pNext;
		}
	}

	omethStop(hEth);	// stop ethernet driver to disable IRQs

	// reset ptr to current periodic instance to reset periodic phy control
	// (periodic phy control could be in a undefined state when removing the currently active instance)
	// (at the next call it will restart automatically)
	omethInternal.pPeriodicEth = 0;

	// release all resources of this instance

	// free all installed hooks
	hHook = hEth->pHookList;
	while(hHook)
	{
		freePtr(hHook->pRxBufBase);	// free frame buffers allocated from the hooks

		hFree = hHook;			// store handle to free it after overtakeing the next-pointer
		hHook = hHook->pNext;	// get next hook handle

		freePtr(hFree);
	}

	// free filter list and attached data structures
	if(hEth->pFilterList)	freePtr(hEth->pFilterList->pFilterData);
	
	freePtr(hEth->pFilterList);
	freePtr(hEth->pPhyReg);		// free phy register image
	freePtr(hEth->pRxBufBase);	// free allocated rx-buffers
	freePtr(hEth->pRxInfo);		// free rx/tx info list
	freePtr(hEth->pTxInfo);
	freePtr(hEth);				// free instance

	return 0;
}

/*****************************************************************************
* 
* omethMnTransmit - transmit a buffer to the network
*	-> without a callback-function
* 
*/
int					omethMnTransmit
(
 OMETH_H			hEth,		/* handle of ethernet driver, see omethCreate() */
 ometh_packet_typ	*pPacket	/* packet to be sent							*/
)
{
	ometh_tx_info_typ	*pInfo	= hEth->pTxNext[0];		// access to next tx info structure
	ometh_desc_typ		*pDesc	= pInfo->pDesc;			// access to tx descriptor

	if (pDesc->flags.byte.high & FLAGS1_OWNER)	return -1;

	pDesc->pData	= (unsigned long)&pPacket->data;	// write buffer ptr to descriptor
	pDesc->len		= (unsigned short)pPacket->length;	// write length

	hEth->pTxNext[0] = pInfo->pNext;					// switch to next info strucutre

	pDesc->flags.byte.high	= pInfo->flags1;			// set flag to start transmitter

	return 0;
}

/*****************************************************************************
* 
* omethMnTransmitTime - transmit a buffer to the network at defined time
*	(same like omethTransmitFast, just the optional argument is additional)
* 
*/
int						omethMnTransmitTime
(
 OMETH_H				hEth,		/* handle of ethernet driver, see omethCreate() */
 ometh_packet_typ*		pPacket,	/* packet to be sent							*/
 unsigned long			time,		/* timestamp									*/
 int					bTxBegIRQ	/* TRUE: Set Tx-BegInt							*/
)
{
	ometh_tx_info_typ	*pInfo	= hEth->pTxNext[0];		// access to next tx info structure
	ometh_desc_typ		*pDesc	= pInfo->pDesc;			// access to tx descriptor

	if (pDesc->flags.byte.high & FLAGS1_OWNER)	return -1;

	pDesc->pData	= (unsigned long)&pPacket->data;	// write buffer ptr to descriptor
	pDesc->len		= (unsigned short)pPacket->length;	// write length
	pDesc->txStart	= time;								// scheduled start time of this frame

	hEth->pTxNext[0] = pInfo->pNext;					// switch to next info strucutre

	if (bTxBegIRQ)
	{
		pDesc->flags.byte.high	= pInfo->flags1 | FLAGS1_START_TIME | FLAGS1_TX_BEG;	// set flag to start transmitter
	}
	else
	{
		pDesc->flags.byte.high	= pInfo->flags1 | FLAGS1_START_TIME;					// set flag to start transmitter
	}

	return 0;
}

/*****************************************************************************
* 
* omethMnResponseAndFilterSet - set new packet and modify filter for
*	response frame
* 
*/
void				omethMnResponseAndFilterSet
(
 OMETH_FILTER_H		hFilter,	/* filter handle							*/
 ometh_packet_typ	*pPacket,	/* packet which shall be responded			*/
 unsigned char		nOffset,	/* offset in the filterarray				*/
 unsigned char		nValue		/* value to set								*/
)
{
	register ometh_desc_typ			*pDesc = hFilter->pTxInfo->pDesc;			// access to hardware

	// set new packet for auto response
	pDesc->len = pPacket->length;

	// overtake buffer to descriptor
	pDesc->pData	= (unsigned long)&pPacket->data;

	// access to filter data and set value
	hFilter->pFilterData->pFilterWriteOnly[nOffset].value = nValue;

	pDesc->flags.byte.high = FLAGS1_OWNER;
}

/*****************************************************************************
* 
* omethMnResponseEnable - enable auto response frame -> fast
* 
*/
void				omethMnResponseEnable
(
 OMETH_FILTER_H		hFilter		/* filter handle				*/
)
{
	FILTER_SET_FLAG(hFilter->pFilterData, CMD_TX_ENABLE);
}

/*****************************************************************************
* 
* omethMnResponseDisable - disable auto response frame -> fast
* 
*/
void				omethMnResponseDisable
(
 OMETH_FILTER_H		hFilter		/* filter handle				*/
)
{
	FILTER_CLEAR_FLAG(hFilter->pFilterData, CMD_TX_ENABLE);
}

/*****************************************************************************
* 
* omethMnSetNextRxBuffer - set the next rx-descriptor
* 
*/
void				omethMnSetNextRxBuffer
(
 OMETH_H			hEth,		/* handle of ethernet driver, see omethCreate()		*/
 ometh_packet_typ	*pPacket	/* new packet for the next rx-descriptor			*/
)
{
	ometh_rx_info_typ		*pInfo;
	ometh_desc_typ			*pDesc;

	pInfo	= hEth->pRxNext->pNext;	// access to next rx info structure
	pDesc	= pInfo->pDesc;			// access to rx descriptor

	pDesc->len				= pPacket->length;
	pDesc->pData			= (unsigned long)&pPacket->data;
	pDesc->flags.byte.high	= pInfo->flags1;
}

/*****************************************************************************
* 
* omethMnSetCurrentRxBuffer - set the current rx-descriptor
* 
*/
void				omethMnSetCurrentRxBuffer
(
 OMETH_H			hEth,		/* handle of ethernet driver, see omethCreate()		*/
 ometh_packet_typ	*pPacket	/* new packet for the current rx-descriptor			*/
)
{
	ometh_rx_info_typ		*pInfo;
	ometh_desc_typ			*pDesc;

	pInfo	= hEth->pRxNext;	// access to current rx info structure
	pDesc	= pInfo->pDesc;		// access to rx descriptor

	pDesc->len				= pPacket->length;
	pDesc->pData			= (unsigned long)&pPacket->data;
	pDesc->flags.byte.high	= pInfo->flags1;

}

/*****************************************************************************
* 
* omethMnRxIrqHandler - Rx-IRQHandler for EPLV2 MN
* 
*/
void			omethMnRxIrqHandler
(
 OMETH_H		hEth			/* handle of ethernet driver, see omethCreate() */
)
{
	ometh_desc_typ			*pDesc;
	ometh_packet_typ		*pRxBuffer;
	OMETH_H					hEthArg;

	hEthArg	= hEth;
	pDesc	= hEth->pRxNext->pDesc;			// access to rx descriptor

	if(pDesc->flags.byte.high & FLAGS1_OWNER) return;	// leave IRQ if no rx-buffer available

	// access to buffer structure and set packet length
	pRxBuffer = GET_TYPE_BASE(ometh_packet_typ , data, pDesc->pData);

	pRxBuffer->length = (unsigned long)pDesc->len;	// length

	#ifdef OMETH_RXIRQFAST_HOOK_FCT
		pRxBuffer = OMETH_RXIRQFAST_HOOK_FCT(pRxBuffer);
	#else
		pRxBuffer = 0;
	#endif

	//
	hEthArg->pRegBase->rxStatus.clrBit = OMETH_REG_IQUIT;

	// pass buffer to MAC
	if (pRxBuffer != 0)
	{
		pDesc->len				= pRxBuffer->length;				// set maximum length for this buffer
		pDesc->pData			= (unsigned long)&pRxBuffer->data;	// set pointer to data
		pDesc->flags.byte.high	= hEthArg->pRxNext->flags1;				// set owner and last flag
	}
	hEth->pRxNext			= hEthArg->pRxNext->pNext;						// switch to next info for next rx
}




/*****************************************************************************
* 
* omethMnTxBegIrqHandler - tx beginn irq handler
* 
*/
void			omethMnTxBegIrqHandler
(
 OMETH_H		hEth		/* handle of ethernet driver, see omethCreate() */
)
{
	#ifdef OMETH_TXBEGIRQ_HOOK_FCT
		OMETH_TXBEGIRQ_HOOK_FCT();
	#endif

	hEth->pRegBase->txStatus.clrBit = OMETH_REG_TX_BEG;		// clear pending irq
}


#ifdef OMETH_TRACE
	/*****************************************************************************
	* 
	* omethSetTraceFct - set trace fct before omethStart !!!
	* 
	* RETURN: -
	*/
	void omethSetTraceFct
	(
	 OMETH_TRACE_RX_FCT	*pTraceFct	/* ptr to trace function */
	)
	{
		pTraceFctInt = pTraceFct;
	}

	/*****************************************************************************
	* 
	* omethRxIrqHandlerTrace2 - special hook for trace to reach better performance (handle 2 is only for trace)
	* 
	*/
	void			omethRxIrqHandlerTrace2
	(
	 OMETH_H		*phEth		/* pointer to 2 handles of ethernet driver, see omethCreate() */
	)
	{
		OMETH_H				hEth;
		OMETH_HOOK_H		hHook;
		OMETH_FILTER_H		hFilter;
		ometh_buf_typ		*pRxBuf;
		ometh_pending_typ	*pQueue;					// ptr to buffer queue
		ometh_rx_info_typ	*pInfo;
		ometh_desc_typ		*pDesc;
		unsigned long		flags=0;
		unsigned long		i=8;

		while(i--)
		{
			hHook = 0;	// to detect if at least 1 interface has received something

			//------------------------------------------------------------- Interface [0] --------------------
			hEth = phEth[0];
			pInfo = hEth->pRxNext;	// access to next rx info structure
			pDesc = pInfo->pDesc;	// access to rx descriptor

			if((pDesc->flags.byte.high & FLAGS1_OWNER) == 0)
			{
				hHook = (OMETH_HOOK_H)&hEth->pRegBase->rxStatus;

				((ometh_status_typ*)hHook)->clrBit = OMETH_REG_IQUIT;

				flags = pDesc->flags.word;

				// access to buffer structure and set packet length
				pRxBuf = GET_TYPE_BASE( ometh_buf_typ , packet.data, pDesc->pData);

				pRxBuf->packet.length = (unsigned long)pDesc->len - 4;	// length without checksum
				pRxBuf->timeStamp     = pDesc->time;					// overtake timestamp to packet header

				if(((ometh_status_typ*)hHook)->value & OMETH_REG_LOST)
				{
					((ometh_status_typ*)hHook)->clrBit = OMETH_REG_LOST;

					pTraceFctInt(hEth->config.adapter, pRxBuf, (flags & ETH_FLAGS_ERROR_MASK) | ETH_FLAGS_RX_LOST);
				}
				else
				{
					pTraceFctInt(hEth->config.adapter, pRxBuf, flags & ETH_FLAGS_ERROR_MASK);
				}

				if( (flags & (ETH_FLAGS_CRC_ERROR|ETH_FLAGS_OVERSIZE)) == 0 )
				{
					hFilter	= hEth->pFilterList + ((flags & 0xF0) >> 4);		// access to filter info structure
					hHook	= hFilter->hHook;		// access to hook function of this filter

					if(hHook->pFct)
					{
						if(hHook->pFreeRead==0)
						{
							// call hook function (no release function is passed because hook is not allowed to queue this frame)
							hHook->pFct(hFilter->arg, &pRxBuf->packet, 0);
						}
						else
						{
							pRxBuf->hHook = hHook;				// overtake hook handle for free function

							pQueue = hHook->pFreeRead;

							if(pQueue->pBuf)		// buffer available for exchange
							{
								// call hook function
								if(hHook->pFct(hFilter->arg, &pRxBuf->packet, omethPacketFree) == 0)
								{
									// use new frame for next rx at this descriptor
									pDesc->pData = (unsigned long)&pQueue->pBuf->packet.data;

									pQueue->pBuf = 0;					// remove buffer from list

									hHook->pFreeRead = pQueue->pNext;	// switch to next buffer in queue
								}
							}
							else
							{		
								hHook->cntOverflow++;			// too many buffers pending, buffer is not passed to the client
								hEth->stat.rxHookOverflow++;	// also count in user accessible statistic structure
							}
						}
					}
				}

				// pass buffer to MAC
				pDesc->len				= hEth->rxLen;		// set maximum length for this buffer
				pDesc->flags.byte.high	= pInfo->flags1;	// set owner and last flag
				hEth->pRxNext			= pInfo->pNext;		// switch to next info for next rx
			}

			//------------------------------------------------------------- Interface [1] (trace only) ------------
			hEth = phEth[1];
			pInfo = hEth->pRxNext;	// access to next rx info structure
			pDesc = pInfo->pDesc;	// access to rx descriptor

			if((pDesc->flags.byte.high & FLAGS1_OWNER)==0)
			{
				hHook = (OMETH_HOOK_H)&hEth->pRegBase->rxStatus;

				((ometh_status_typ*)hHook)->clrBit = OMETH_REG_IQUIT;

				// access to buffer structure and set packet length
				pRxBuf = GET_TYPE_BASE( ometh_buf_typ , packet.data, pDesc->pData);

				pRxBuf->packet.length = (unsigned long)pDesc->len - 4;	// length without checksum
				pRxBuf->timeStamp     = pDesc->time;					// overtake timestamp to packet header

				if(((ometh_status_typ*)hHook)->value & OMETH_REG_LOST)
				{
					((ometh_status_typ*)hHook)->clrBit = OMETH_REG_LOST;

					pTraceFctInt(hEth->config.adapter, pRxBuf, (pDesc->flags.word & ETH_FLAGS_ERROR_MASK) | ETH_FLAGS_RX_LOST);
				}
				else
				{
					pTraceFctInt(hEth->config.adapter, pRxBuf, pDesc->flags.word & ETH_FLAGS_ERROR_MASK);
				}

				// pass buffer to MAC
				pDesc->len				= hEth->rxLen;		// set maximum length for this buffer
				pDesc->flags.byte.high	= pInfo->flags1;	// set owner and last flag
				hEth->pRxNext			= pInfo->pNext;		// switch to next info for next rx
			}

			//------------------------------------------------------------- No Receive - Break ------------
			if(hHook==0) break;
		}
	}
#endif

