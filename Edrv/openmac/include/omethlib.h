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
 12.03.2004     enzingert   reated
 23.03.2009     zelenkaj    Changes in omethTxIrqHandler() to free all sent packets
                             in the TX IR, to avoid TX queue overflow.
 26.03.2009     zelenkaj    revised

----------------------------------------------------------------------------*/

#ifndef __OMETHLIB_H__
#define __OMETHLIB_H__

typedef struct OMETH_TYP*		OMETH_H;		// handle for ethernet driver
typedef struct OMETH_HOOK*		OMETH_HOOK_H;	// handle for client hook
typedef struct OMETH_FILTER*	OMETH_FILTER_H;	// handle for receive filter

#define OMETH_MAC_TYPE_01		0x01	// 16 filters,16 rx,16 tx
#define OMETH_MAC_TYPE_02		0x02	// 14 standard filters,2 x-filters, 16 rx,16 tx
#define OMETH_MAC_TYPE_03		0x03	// 6 standard filters,2 p-filters, 16 rx, 16 tx

#define OMETH_MODE_HALFDUPLEX	0x01	// half duplex supported
#define OMETH_MODE_FULLDUPLEX	0x02	// full duplex supported
#define OMETH_MODE_100			0x04
#define OMETH_MODE_10			0x08
#define OMETH_MODE_DIS_AUTO_NEG	0x10	// disable auto negotiation (10/100 and FULL/HALF must be selected explicitly)
#define OMETH_MODE_PHY_LIST		0x20	// the user defines a list of phy-addresses which shall be used

#define ETH_FLAGS_CRC_ERROR		0x0001		// crc error
#define ETH_FLAGS_OVERSIZE		0x0002		// over size frame received
#define ETH_FLAGS_ERR_PREAMBLE	0x0004		// over size frame received
#define ETH_FLAGS_ERR_NOISE		0x0008		// over size frame received

#define ETH_FLAGS_RX_LOST		0x0010		// used for trace ... signal lost frames
									// 0x0020 .. do not used, reserved for trace internal use
#define ETH_FLAGS_RX_ALIGNMENT	0x1000		// used for trace ... frame alignment error

// combination with all error flags in descriptor
#define ETH_FLAGS_ERROR_MASK	(ETH_FLAGS_CRC_ERROR | ETH_FLAGS_OVERSIZE | ETH_FLAGS_ERR_PREAMBLE | ETH_FLAGS_ERR_NOISE | ETH_FLAGS_RX_ALIGNMENT)

#define OMETH_MAX_PHY_CNT	8		// maximum number of supported phys

//********************* network configuration ******************************
typedef struct
{
	unsigned char	macType;	// MAC type : OMETH_MAC_TYPE_01
	unsigned char	mode;		// OMETH_MODE_FULLDUPLEX, OMETH_MODE_HALFDUPLEX (or both)
	unsigned char	adapter;	// Adapter number (any number can be chosen, must be uniqe)

	void			*pRamBase;	// base address of MAC RAM (filters+descriptors)
	void			*pRegBase;	// base address of MAC control registers

	unsigned short	rxBuffers;	// number of rx buffers (2<=rxBuffers<=max)
	unsigned short	rxMtu;		// MTU for rx buffers

	void			*pPhyBase;	// base address of PHY MII control registers (can also be 0 if no phys should be controlled)
	
	// list of phy addresses which shall be used (only if mode-bit OMETH_MODE_PHY_LIST is set)
	// (only list phys which belong to the respective ethernet mac, not all existing phys !)
	unsigned char	phyCount;	// number of valid bytes in phyList[]
	unsigned char	phyList[OMETH_MAX_PHY_CNT];
}ometh_config_typ;

//********************* network statistics ******************************
typedef struct
{
	#ifndef OMETH_TRACE	// do not compile the following lines to increase trace performance
		unsigned long rxOk;
		unsigned long rxLost;
		unsigned long rxOversize;
		unsigned long rxCrcError;
		unsigned long rxHookDisabled;	// frames received while hook is disabled, frame discarded
		unsigned long rxHookOverflow;	// frames received but no availables buffer, frame discarded
	#endif

	unsigned long txCollision;		// total tx collisions on the bus
	unsigned long txDone[16];		// [0]..number of sent frames with 0 collisions ...
	unsigned long txSpuriousInt;	// tx int occured but no frame sent ??
}ometh_stat_typ;


//******************* type for phy registers **************************
typedef enum
{
	OMETH_PHY_STATE_INVALID,	// phy state can not be evaluated, port not available (or hEth invalid)
	OMETH_PHY_STATE_NOLINK,		// no link
	OMETH_PHY_STATE_HALF,		// phy has half duplex link
	OMETH_PHY_STATE_FULL		// phy has full duplex link
}phy_stat_enum;

typedef struct
{
	unsigned short r[8];
}phy_reg_typ;

//Phy REG 0
#define	PHY_REG0_COLLISIONTEST		0x0080
#define	PHY_REG0_FULL				0x0100
#define	PHY_REG0_AUTONEG_RESTART	0x0200
#define	PHY_REG0_ISOLATE			0x0400
#define	PHY_REG0_POWER_DOWN			0x0800
#define	PHY_REG0_AUTONEG_ENABLE		0x1000
#define	PHY_REG0_100				0x2000
#define	PHY_REG0_LOOPBACK			0x4000
#define	PHY_REG0_RESET				0x8000

//Phy REG 1 (Basic Status)
#define	PHY_REG1_EXTENDED			0x0001	// extended capability
#define	PHY_REG1_JABBER				0x0002	// jabber detected
#define	PHY_REG1_LINK				0x0004	// linked
#define	PHY_REG1_AUTONEGON			0x0008	// auto negotiation availibility
#define	PHY_REG1_REMOTEFAULT		0x0010	//
#define	PHY_REG1_AUTONEGCOMPLETE	0x0020	// auto negotiation complete
#define	PHY_REG1_PREAMBLESUPRESS	0x0040	//
#define	PHY_REG1_10B_BASE_T_HALF	0x0800
#define	PHY_REG1_10B_BASE_T_FULL	0x1000
#define	PHY_REG1_100_BASE_TX_HALF	0x2000
#define	PHY_REG1_100_BASE_TX_FULL	0x4000
#define	PHY_REG1_100_BASE_T4		0x8000

//Phy REG 2 (Phy ID 1)
#define PHY_REG2_OUI_18_3			0xFFFF

//Phy REG 3 (Phy ID 2)
#define PHY_REG3_REVISIONNUMBER		0x000F
#define PHY_REG3_MODELLNUMBER		0x03F0
#define PHY_REG2_OUI_24_19			0xFC00

//Phy REG 4 (Advertisement Register)
#define PHY_REG4_SELECTOR			0x0001	// IEEE 802.3 selector
#define PHY_REG4_100TX_HALF			0x0080	// advertise 100 BASE-TX half duplex
#define PHY_REG4_100TX_FULL			0x0100	// advertise 100 BASE-TX full duplex

//Phy REG 5 (Link Partner Ability)
#define PHY_REG5_SECTORFIELD		0x001F
#define PHY_REG5_10T				0x0020
#define PHY_REG5_10T_FULL			0x0040
#define PHY_REG5_100TX				0x0080
#define PHY_REG5_100TX_FULL			0x0100	// link partner 100 BASE-TX full duplex
#define PHY_REG5_T4					0x0200
#define PHY_REG5_PAUSE				0x0C00
#define PHY_REG5_REMOTEFAULT		0x2000
#define PHY_REG5_ACKNOWLEDGE		0x4000
#define PHY_REG5_NEXTPAGE			0x8000


//********** packet structure for ethernet frames ***********************
typedef struct
{
	unsigned long	length;		// frame length excluding checksum

	struct ometh_packet_data_typ
	{
		unsigned char dstMac[6];
		unsigned char srcMac[6];
		unsigned char ethertype[2];
		unsigned char minData[46];	// minimum nubmer of data bytes for a standard ethernet frame
		unsigned char checkSum[4];
	}data;
}ometh_packet_typ;

//********** buffer structure for ethernet frames ***********************
typedef struct
{
	// buffer header
	unsigned long		timeStamp;	// packet time stamp is stored here
	OMETH_HOOK_H		hHook;		// handle to hook which created the buffer

	// ethernet packet
	ometh_packet_typ	packet;
}ometh_buf_typ;


//******************* function to free buffer ***************************
typedef void	OMETH_BUF_FREE_FCT
(
	ometh_packet_typ	*pPacket	/* packet which should be released	*/
);

//******************* function to free buffer ***************************
typedef void	OMETH_BUF_FREE_FCT_ARG
(
	ometh_packet_typ	*pPacket,	/* packet which should be released	*/
	void				*arg,		/* argument passed to omethTransmit */
	unsigned long		time		/* time when packet was sent */
);

//********* ethernet callback function / buffer received ****************
// 1.) the hook function itself shall not call pFct but return -1 instead
// 2.) ethernet hook has to return 0 (OK) or -1 (buffer not taken)
//
typedef int		OMETH_HOOK_FCT
(
	void 				*arg,		/* function argument						*/
	ometh_packet_typ	*pPacket,	/* pointer to packet (0=auto response sent)	*/
	OMETH_BUF_FREE_FCT	*pFct		/* ptr to release-function				*/
);

#define OMETH_INVALID_FILTER	((OMETH_FILTER_H)-1)	// invalid filter handle
#define OMETH_INVALID_PACKET	((ometh_packet_typ*)-1)	// invalid packet

// can be used with omethHookCreate() or omethHookSetFunction() to disable hook
#define OMETH_HOOK_DISABLED		((OMETH_HOOK_FCT*)-1)

#define MII_CTRL_RESET		0x0001	/* set phys on this MII to reset  */
#define MII_CTRL_ACTIVE		0x0002	/* set phys on this MII to active */
#define MII_CTRL_GET_STATE	0x0004	/* get reset/active state (0/1)   */

// convert eth-ticks to ms and vica versa
#define OMETH_TICKS_2_MS(ticks)		((unsigned long)(ticks)/50000u)
#define OMETH_MS_2_TICKS(ms)		((unsigned long)(ms)*50000u)

// convert eth-ticks to us and vica versa
#define OMETH_TICKS_2_US(ticks)		((unsigned long)(ticks)/50u)
#define OMETH_US_2_TICKS(us)		((unsigned long)(us)*50u)

/*****************************************************************************
* 
* omethMiiControl - enable/disable phys
* 
* RETURN: 
*	-1   ... error
*	 0-x ... depending on command
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
);

/*****************************************************************************
* 
* omethInit - initialize ethernet driver
* 
* RETURN: -
*
*/
void	omethInit
(
 void
);

/*****************************************************************************
* 
* omethCreate - create and initialize new instance of ethernet driver
* 
* Creates a new instance of an ethernet driver and initializes all used
* resources. After creating a new driver instance the clients can
* install hooks with the function omethHookCreate()
* 
* RETURN: Handle to ethernet driver (0..error)
*
*	The following reasons can cause an error:
*	- Parameters in config structure are 0 (hMemPart, rxBuffers)
*	- Mode is incomplete ( DUPLEX info missing)
*	- System is out of memory
*	- Too many rx buffers configured (hardware dependent)
*	- Wrong macType in configuration
*	- At least one of the configured Phys is not responding
*	- Wrong pPhyBase
*
*/
OMETH_H			omethCreate
(
 ometh_config_typ	*pEthConfig		/* ptr to ethernet config struct			*/
);

/*****************************************************************************
* 
* omethGetHandle - get adapter handle
* 
* Get handle of network interface based on a adapter number. The number is
* provided when initializing the instance with omethCreate()
*
* RETURN: Handle to ethernet driver (0..error)
*
*	The following reasons can cause an error:
*	- Adapter with this number not yet initialized
*
*/
OMETH_H			omethGetHandle
(
 int	adapter		/* number of network adapter (0..255) */
);

/*****************************************************************************
* 
* omethPhyHardwareAdr - Get hardware address of phy on this port
* 
* RETURN:
*	-1    .. port not available
*	0..31 .. phy address
*
*/
int				omethPhyHardwareAdr
(
 OMETH_H		hEth,		/* handle of ethernet driver, see omethCreate()		*/
 unsigned short	port		/* phy number / port number of integrated hub (0-n)	*/
);

/*****************************************************************************
* 
* omethPhyInfo - Get ptr to Phy register copy in RAM
* 
* Can be used to read the link status of each port of a integrated HUB.
*
* RETURN: address to structure with phy register values (read only)
*
*/
phy_reg_typ*	omethPhyInfo
(
 OMETH_H		hEth,		/* handle of ethernet driver, see omethCreate()		*/
 unsigned short	port		/* phy number / port number of integrated hub (0-n)	*/
);

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
);

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
);

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
);

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
);

/*****************************************************************************
* 
* omethPeriodic - periodic call of ethernet driver
*
* This function must be called periodic. It will read the duplex mode of the
* PHY (half/full) and tansfer the setting to the MAC
* If full duplex is configured, and more than 1 phys are connected to the
* interface, both ports are full-duplex-able as long only 1 port is connected.
* As soon more than 1 port is connected simultanousely all ports will be 
* switched to half duplex (HUBs can't work with full-duplex)
* 
* RETURN: -
* 
*/
void			omethPeriodic
(
 void
);

/*****************************************************************************
* 
* omethHookCreate - create a new hook for a client
* 
* Creates a new hook for a client. The maximum number of possible hooks
* depends on the macType of the ethernet controller.
*
* After creating a hook the client has to define one ore more filters
* with omethFilterCreate() to define which frame types should be passed
* to this hook.
*
*	maxPending = 0
*		The driver passes the received buffer to the hook function and
*		will re-use the buffer after the hook returned.
*	maxPending = 1-n
*		The driver will pass the buffer to the hook and will not re-use the
*		buffer until the client passes the buffer back to the driver using
*		the free-function which was passed as parameter when to the receive
*		hook.
*		If a hook has reached the maximum of allowed pending buffers the
*		ethernet driver will NOT call the client and the received buffers
*		for this client will be re-used (the received data is lost)
*		To avoid this the client has to announce enough buffers with the
*		parameter maxPending
*
* If the client is called with pData=0 the driver signals that a 
* auto-response (see omethResponseSet()) was sent. In this case the size
* tells the client how many bytes the driver sent to the network. The value
* will be either the value defined by the client or 0 in case of collision.
* 
* RETURN: Handle to ethernet driver (0..error)
*
*	The following reasons can cause an error:
*	- hEth or pFct=0
*	- out of memory
*	
*/
OMETH_HOOK_H	omethHookCreate
(
 OMETH_H		hEth,		/* handle of ethernet driver, see omethCreate() */
 OMETH_HOOK_FCT	*pFct,		/* callback function							*/
 unsigned short	maxPending	/* maximum number of pending buffers			*/
);

/*****************************************************************************
* 
* omethHookSetFunction - change hook function of existing hook
* 
* Changes the function ptr of a existing hook. It is not possible to change the
* maxPending parameter
*
* RETURN: 0..ok / -1..error
*
*	The following reasons can cause an error:
*	- hHook invalid
*	
*/
int				omethHookSetFunction
(
 OMETH_HOOK_H	hHook,		/* handle of existing hook					*/
 OMETH_HOOK_FCT	*pFct		/* new callback function or 0 to disable	*/
);

/*****************************************************************************
* 
* omethFilterCreate - create a filter entry for a hook
* 
* Creates a new filter entry for hook of a client. If a frame is received by
* this filter the clients will be called with the given argument.
*
* RETURN: Handle to ethernet driver (0..error)
*
*	The following reasons can cause an error:
*	- hHook or pMask or pValue = 0
*	- no filter available (number of available filters is hardware dependent)
*
*/
OMETH_FILTER_H	omethFilterCreate
(
 OMETH_HOOK_H	hHook,			/* handle of a ethernet client				*/
 void			*arg,			/* argument for hook function				*/
 void			*pMask,			/* ptr to array with 17 mask bytes			*/
 void			*pValue			/* ptr to array with 17 compare values		*/
);

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
);

/*****************************************************************************
* 
* omethFilterSetPattern - sets a new filter pattern (mask/value) to an existing
*							filter
* 
*
* RETURN: 0..ok / -1..error
*
*	The following reasons can cause an error:
*	- hFilter invalid
*	- pMask or pValue = 0
*	
*/
int				omethFilterSetPattern
(
 OMETH_FILTER_H	hFilter,
 void			*pMask,			/* ptr to array with 17 mask bytes			*/
 void			*pValue			/* ptr to array with 17 compare values		*/
);

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
);

/*****************************************************************************
* 
* omethFilterSetArgument - sets a new argument which will be passed to the callback
*
* RETURN: 0..ok / -1..error
*
*	The following reasons can cause an error:
*	- hFilter invalid
*	
*/
int				omethFilterSetArgument
(
 OMETH_FILTER_H	hFilter,
 void			*arg
);

/*****************************************************************************
* 
* omethFilterSetHook - sets a new hook for the filter
*
*/
int				omethFilterSetHook
(
 OMETH_FILTER_H	hFilter,
 OMETH_HOOK_H	hHook			/* handle from omethHookCreate()		*/
);

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
);

/*****************************************************************************
* 
* omethFilterEnable - enable filter (after omethFilterCreate() the filter is
*                     automatically enabled)
*
* RETURN: 0..ok / -1..error
*
*	The following reasons can cause an error:
*	- hFilter invalid
*	
*/
int				omethFilterEnable
(
 OMETH_FILTER_H	hFilter
);

/*****************************************************************************
* 
* omethFilterDisable - enable filter (after omethFilterCreate() the filter is
*                     automatically enabled)
*
* RETURN: 0..ok / -1..error
*
*	The following reasons can cause an error:
*	- hFilter invalid
*	
*/
int				omethFilterDisable
(
 OMETH_FILTER_H	hFilter
);

/*****************************************************************************
* 
* omethSetSCNM - set SCNM filter
* 
* hFilter = valid filter:
*	The driver will switch to SCNM (Slot Communication Network Management)
*	mode and transmit unknown frames (non-EPL frames) only after a frame was
*	reveived with the given filter.
*
* hFilter = 0:
*	Disables the SCNM mode and transmit buffers will be sent directly after
*	calling omethTransmit().
*
* hFilter = OMETH_INVALID_FILTER
*	The driver will not accept any transmit frames, queue sending is disabled
*
* RETURN: 0..ok / -1..error / -2..error,queue transmitter disabled
*
*	The following reasons can cause an error:
*	- hEth = 0
*	- the give filter handle hFilter does not belong to the driver hEth
*
*/
int				omethSetSCNM
(
 OMETH_H		hEth,		/* handle of ethernet driver, see omethCreate() */
 OMETH_FILTER_H	hFilter		/* filter handle								*/
);

/*****************************************************************************
* 
* omethResponseInit - initialize a installed filter for auto response frames
* 
* RETURN: 0..no error , -1 error
*
*	The following reasons can cause an error:
*	- hFilter is invalid
*	- not enough tx descriptors available
*	- omethReponseInit was already called for this filter
*	
*/
int				omethResponseInit
(
 OMETH_FILTER_H	hFilter		/* filter handle								*/
);

/*****************************************************************************
* 
* omethResponseInitBuf - initialize a installed filter for auto response frames
*						and provice change buffers
*						(this avoids that the funciton returns 0-pointers at the first
*						1 or 2 calls)
*
* RETURN: 0..no error , -1 error
*
*	The following reasons can cause an error:
*	- hFilter is invalid
*	- not enough tx descriptors available
*	- omethReponseInit was already called for this filter
*	
*/
int				omethResponseInitBuf
(
 OMETH_FILTER_H	hFilter,		/* filter handle							*/
 ometh_packet_typ	*pPacket1,	/* spare packet for change buffer			*/
 ometh_packet_typ	*pPacket2	/* spare packet for change buffer			*/
);

/*****************************************************************************
* 
* omethResponseSet - set new buffer for response frame
* 
* The given filter has to be initialized as auto-response filter with the 
* function omethResponseInit() before.
* Depending on the parameters at initialization the switch-over will cause
* a short off time (no response will be sent to an incoming frame) or it will
* perform a seamless switch over
*
* RETURN:
*		OMETH_INVALID_PACKET : error
*		0                    : no error, no packet was installed before
*		packet pointer       : no error, address of last used packet (can be released)
*
*	The following reasons can cause an error:
*	- hFilter is invalid
*	- omethResponseInit was not called for this filter before calling omethStart
*	
*/
ometh_packet_typ	*omethResponseSet
(
 OMETH_FILTER_H		hFilter,	/* filter handle							*/
 ometh_packet_typ	*pPacket	/* packet which shall be responded			*/
);

/*****************************************************************************
* 
* omethResponseDisable - disable auto response frame
* 
* The given filter has to be initialized as auto-response filter with the 
* function omethResponseInit() before.
* After calling omethResponseDisable() the MAC will not send any auto response
* frames for this filter until omethResponseEnable() is called again.
* 
* RETURN: 0..ok / -1..error
*
*	The following reasons can cause an error:
*	- hFilter is invalid
*	- omethResponseInit was not called for this filter before calling omethStart
*	
*/
int					omethResponseDisable
(
 OMETH_FILTER_H		hFilter		/* filter handle				*/
);

/*****************************************************************************
* 
* omethResponseEnable - enable auto response frame
* 
* The given filter has to be initialized as auto-response filter with the 
* function omethResponseInit() before.
* If a auto response frame was disabled with omethResponseDisable() this function
* can be used to enable the auto response again.
* 
* RETURN: 0..ok / -1..error
*
*	The following reasons can cause an error:
*	- hFilter is invalid
*	- omethResponseInit was not called for this filter before calling omethStart
*	
*/
int					omethResponseEnable
(
 OMETH_FILTER_H		hFilter		/* filter handle				*/
);

/*****************************************************************************
* 
* omethTransmit - transmit a buffer to the network
* 
* The buffer will be queued and transmitted as soon as possible. After the
* data was sent to the network the user will get a callback.
*
* RETURN: number of bytes passed to the send queue of the MAC controller
*	The value can only be 0 (MAC buffer overflow) or the given length of the packet type)
*
*/
unsigned long		omethTransmit
(
 OMETH_H			hEth,		/* handle of ethernet driver, see omethCreate() */
 ometh_packet_typ	*pPacket,	/* packet to be sent							*/
 OMETH_BUF_FREE_FCT	*pFct		/* function ptr to sent-ack-function			*/
);

/*****************************************************************************
* 
* omethTransmitArg - transmit a buffer to the network with argument for tx-callback
*	(same like omethTransmit, just the optional argument is additional)
* 
*/
unsigned long		omethTransmitArg
(
 OMETH_H				hEth,		/* handle of ethernet driver, see omethCreate() */
 ometh_packet_typ		*pPacket,	/* packet to be sent							*/
 OMETH_BUF_FREE_FCT_ARG	*pFct,		/* function ptr to sent-ack-function			*/
 void					*arg		/* argument which will be passed to free function */
);

/*****************************************************************************
* 
* omethTransmitArg2 - transmit a buffer to the network with argument for tx-callback
*	(same like omethTransmitArg but using the 2nd transmit queue if existing)
* 
*/
unsigned long		omethTransmitArg2
(
 OMETH_H				hEth,		/* handle of ethernet driver, see omethCreate() */
 ometh_packet_typ		*pPacket,	/* packet to be sent							*/
 OMETH_BUF_FREE_FCT_ARG	*pFct,		/* function ptr to sent-ack-function			*/
 void					*arg		/* argument which will be passed to free function */
);

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
);

/*****************************************************************************
* 
* omethTransmitPending - get number of pending transmit frames in queue
* 
* RETURN: number of pending transmit frames in queue (0..16)
*/
unsigned char		omethTransmitPending
(
 OMETH_H				hEth		/* handle of ethernet driver, see omethCreate() */
);

/*****************************************************************************
* 
* omethStart - start ethernet driver
* 
* When the driver is started the functions omethHookCreate() and
* omethFilterCreate() are not allowed.
*
* The function omethResponseInit() for a specific filter is only allowed if
* it was already called at least once before starting the driver.
* 
* RETURN: -
*
*/
void			omethStart
(
 OMETH_H		hEth		/* handle of ethernet driver, see omethCreate() */
);

/*****************************************************************************
* 
* omethStop - stop ethernet driver
* 
* RETURN: -
*
*/
void			omethStop
(
 OMETH_H		hEth		/* handle of ethernet driver, see omethCreate() */
);

/*****************************************************************************
* 
* omethGetTimestamp - get timestamp of a received packet
* 
* RETURN: timestamp value (50 MHz clock)
*/
unsigned long	omethGetTimestamp
(
 ometh_packet_typ	*pPacket	/* address of rx packet*/
);

/*****************************************************************************
* 
* omethPacketFree - pass rx packet back to ethernet driver
* 
* After a client has processed the data of a receive buffer the client has
* to pass te buffer back to the ethernet driver.
*
* RETURN: -
*/
void		omethPacketFree
(
 ometh_packet_typ	*pPacket	/* address of rx packet	*/
);

/*****************************************************************************
* 
* omethStatistics - get ptr to statistics of ethernet adapter
* 
* RETURN:	pointer to internal statistics structure of the adapter
*			0 .. hEth invalid
*/
ometh_stat_typ	*omethStatistics
(
 OMETH_H		hEth		/* handle of ethernet driver, see omethCreate() */
);


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
	);
	void		omethSoftwareIrqStart
	(
	OMETH_H			hEth		/* handle of ethernet driver, see omethCreate() */
	);
#endif

#if (OMETH_ENABLE_RX_HANDSHAKE==1)
	/*****************************************************************************
	* 
	* omethSetRxHandshake - set rx-handshake
	* 
	* mode : OMETH_RX_HANDSHAKE_....
	*/
	#define OMETH_RX_HANDSHAKE_RECEIVE_ALL	1
	#define OMETH_RX_HANDSHAKE_BLOCK_LOW	2
	#define OMETH_RX_HANDSHAKE_BLOCK_ALL	3
	#define OMETH_RX_HANDSHAKE_BLOCK_AUTO	4	// low priority will be blocked if more than 8 RxIRQs are pending

	void	omethSetRxHandshake
	(
	OMETH_H		hEth,		/* handle of ethernet driver, see omethCreate() */
	unsigned short	mode
	);
#endif

/*****************************************************************************
* 
* omethRxIrqHandler - to be called from the MAC RX interrupt
* 
* Interrupt handler for the ethernet driver. The parameter hEth is the handle
* of a ethernet driver created with omethCreate().
* The function will call the hook functions given with omethHookCreate()
*
*	!!! 
*	Required Rx IRQ stack size on PX32 with TCP/IP Stack: min. 64 Byte
*	The required stack size heavily depents of the installed hook functions.
*
* RETURN: -
*
*/
void			omethRxIrqHandler
(
 OMETH_H		hEth		/* handle of ethernet driver, see omethCreate() */
);

/*****************************************************************************
* 
* omethTxIrqHandler - to be called from the MAC TX interrupt
* 
* Interrupt handler for the ethernet driver. The parameter hEth is the handle
* of a ethernet driver created with omethCreate().
*
*	!!! 
*	Required Tx IRQ stack size on PX32 with TCP/IP Stack: min. 32 Byte
*	The required stack size heavily depents of the buffer release functions
*	passed with omethTransmit()
*
* RETURN: -
*
*/
void			omethTxIrqHandler
(
 OMETH_H		hEth		/* handle of ethernet driver, see omethCreate() */
);

/*****************************************************************************
* 
* omethRxIrqHandlerMux - to be called from the MAC RX interrupt (multiplexer for all installed ethernet interfaces)
* 
* RETURN: -
*
*/
void			omethRxIrqHandlerMux
(
 void
);

/*****************************************************************************
* 
* omethTxIrqHandlerMux - to be called from the MAC TX interrupt (multiplexer for all installed ethernet interfaces)
* 
* RETURN: -
*
*/
void			omethTxIrqHandlerMux
(
 void
);

/*****************************************************************************
* 
* omethDestroy - stop ethernet driver instance and free all allocated resources
* 
* RETURN: 0..no error , -1 error
*
*	The following reasons can cause an error:
*	- hEth invalid
*
*/
int				omethDestroy
(
 OMETH_H		hEth		/* handle of ethernet driver, see omethCreate() */
);

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
);

/*****************************************************************************
* 
* omethMnTransmitTime - transmit a buffer to the network at defined time
*	(same like omethMnTransmit, just the optional argument is additional)
* 
*/
int						omethMnTransmitTime
(
 OMETH_H				hEth,		/* handle of ethernet driver, see omethCreate() */
 ometh_packet_typ*		pPacket,	/* packet to be sent							*/
 unsigned long			time,		/* timestamp									*/
 int					bTxBegIRQ	/* TRUE: Set Tx-BegInt							*/
);
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
);

/*****************************************************************************
* 
* omethMnResponseEnable - enable auto response frame -> fast
* 
*/
void				omethMnResponseEnable
(
 OMETH_FILTER_H		hFilter		/* filter handle				*/
);

/*****************************************************************************
* 
* omethMnResponseDisable - disable auto response frame -> fast
* 
*/
void				omethMnResponseDisable
(
 OMETH_FILTER_H		hFilter		/* filter handle				*/
);

/*****************************************************************************
* 
* omethMnSetNextRxBuffer - set the next rx-descriptor
* 
*/
void				omethMnSetNextRxBuffer
(
 OMETH_H			hEth,		/* handle of ethernet driver, see omethCreate()		*/
 ometh_packet_typ	*pPacket	/* new packet for the next rx-descriptor			*/
);

/*****************************************************************************
* 
* omethMnSetCurrentRxBuffer - set the current rx-descriptor
* 
*/
void				omethMnSetCurrentRxBuffer
(
 OMETH_H			hEth,		/* handle of ethernet driver, see omethCreate()		*/
 ometh_packet_typ	*pPacket	/* new packet for the current rx-descriptor			*/
);

/*****************************************************************************
* 
* omethMnRxIrqHandler - Rx-Fast IRQHandler
* 
*/
void			omethMnRxIrqHandler
(
 OMETH_H		hEth			/* handle of ethernet driver, see omethCreate() */
);

/*****************************************************************************
* 
* omethNoFilterMatchIrqHandler - no filter match irq handler
* 
*/
void			omethNoFilterMatchIrqHandler
(
 OMETH_H		hEth		/* handle of ethernet driver, see omethCreate() */
);

/*****************************************************************************
* 
* omethMnTxBegIrqHandler - tx beginn irq handler
* 
*/
void			omethMnTxBegIrqHandler
(
 OMETH_H		hEth		/* handle of ethernet driver, see omethCreate() */
);


#ifdef OMETH_TRACE
	//********* rx-trace callback function ****************
	typedef void		OMETH_TRACE_RX_FCT
	(
		unsigned long	adapter,	/* ethernet adapter number	*/
		ometh_buf_typ	*pRxBuf,	/* ptr to received buffer	*/
		unsigned long	flags		/* descriptor flags			*/
	);
	
	/*****************************************************************************
	* 
	* omethSetTraceFct - set trace fct before omethStart !!!
	* 
	* RETURN: -
	*/
	void omethSetTraceFct
	(
	 OMETH_TRACE_RX_FCT	*pTraceFct	/* ptr to trace function */
	);

	// optimized irq-multiplexer to handle 2 interfaces (2nd if only for trace)
	void			omethRxIrqHandlerTrace2
	(
	 OMETH_H		*phEth		/* pointer to 2 handles of ethernet driver, see omethCreate() */
	);
#endif

#endif
