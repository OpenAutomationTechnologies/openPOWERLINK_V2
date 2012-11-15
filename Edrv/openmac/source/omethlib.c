/* omethlib.h - Ethernet Library for FPGA MAC Controller */
/*
------------------------------------------------------------------------------
Copyright (c) 2009, B&R
All rights reserved.

Redistribution and use in source and binary forms,
with or without modification,
are permitted provided that the following conditions are met:

- Redistributions of source code must retain the above copyright notice,
this list of conditions and the following disclaimer.

- Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer
in the documentation and/or other materials provided with the distribution.

- Neither the name of the B&R nor the names of
its contributors may be used to endorse or promote products derived
from this software without specific prior written permission.


THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

------------------------------------------------------------------------------
 Module:    omethlib
 File:      omethlib.h
 Author:    Thomas Enzinger(enzingert)
 Created:   12.03.2004
 Revised:   25.01.2010
 State:     tested on Altera Nios II and Xilinx Microblaze
------------------------------------------------------------------------------

 Functions:
     omethMiiControl            - control reset pin of MII

     omethInit                  - initialize the ethernet driver (to be called once at startup)
     omethCreate                - create driver instance
    omethGetHandle              - get handle of an instance based on its adapter number
    omethGetLinkSpeed           - get link-speed of this interface (10/100 MBit)
    omethGetTxBufBase           - get MAC-internal tx buffer base address
    omethGetRxBufBase           - get MAC-internal rx buffer base address

    omethPeriodic               - must be called cyclic to update phy register table
    omethPhyHardwareAdr         - get hardware address of phy
    omethPhyInfo                - get address to phy register structure
    omethPhyLinkState           - get link state of phy
    omethPhyWrite               - write phy register (blocking)
    omethPhyRead                - read phy register (blocking)
    omethPhyReadNonBlocking     - read phy register (nonblocking, can not be combined with omethPeriodic())
    omethPhySetHalfDuplex       - set phy's halfduplex (the phy's will be set in omethPerodic())

    omethHookCreate             - create hook to get callbacks for received frames
    omethHookSetFunction        - change the callback function of a created hook

    omethFilterCreate           - create a filter to specify which frames should call a hook
    omethFilterSetPattern       - change all filter value/mask bytes of a filter
    omethFilterSetByteMask      - change 1 filter mask byte of a filter
    omethFilterSetByteValue     - change 1 filter value byte of a filter
    omethFilterSetArgument      - change the callback argument which a specific filter passes to the callback
    omethFilterSetNoMatchIRQ    - enable the NoMatch-IRQ
    omethFilterSetHubPort       - set HUB port to which the filter should be limited (MAC Version >= 1.67 required)

    omethSetSCNM                - define a filter for the slot communication management
    omethResponseInit           - prepare a filter for auto response
    omethResponseInitBuf        - prepare a filter for auto response and provide change buffers
    omethResponseSet            - align a packet which should be responded on a filter event
    omethResponseLink           - link existing response buffer to another filter
    omethResponseTime           - set ticks which are added to the IPG
    omethResponseDisable        - disable response frame
    omethResponseEnable         - enable response frame
    omethResponseCount          - get number of transmitted auto responses
    omethResponseCountReset     - reset number of transmitted auto responses

    omethTransmit               - transmit frame (immediately or after filter event set with omethSetSCNM)
    omethTransmitArg            - transmit frame with argument for tx-callback
    omethTransmitTime           - transmit frame at defined timestamp
    omethTransmitPending        - get number of pending frames in tx-queue

    omethStart                  - start ethernet driver
    omethStop                   - stop ethernet driver (disable all rx and tx events)

    omethGetTimestamp           - get timestamp of a received packet
    omethPacketFree             - function to release a received packet

    omethStatistics             - get pointer to ethernet statistics

    omethRxIrqHandler           - irq handler for packet receive
    omethTxIrqHandler           - irq handler for packet transmit

    omethRxIrqHandlerMux        - irq handler for packet receive of all installed ethernet interfaces
    omethTxIrqHandlerMux        - irq handler for packet transmit of all installed ethernet interfaces

    omethNoFilterMatchIrqHandler    - irq handler for No-Filter match IRQ

    omethDestroy                - destroy driver instance (stop MAC and free all allocated resources)

------------------------------------------------------------------------------
 History:
    12.03.2004  enzinger    created
    21.08.2009  zelenkaj    changes:    - line 1120 (omethPeriodic)
                                          changed "WORD" to "unsigned short"
                                        - omethFilterSetPattern() function
                                          changed pattern set for Microblaze
                                          NIOS II as before
    25.01.2010  zelenkaj    changes:    - RX buffer allocation
                                          omethCreateInt() line ~640
                                        - ometh_config_typ added buffer base
                                        - added functions:
                                          omethGetTxBufBase and
                                          omethGetRxBufBase
    25.01.2011  zelenkaj    changes:    - RX buffer allocation depends on
                                          location (pktLoc).
                                        - ometh_config_typ added pktLoc
    21.02.2011  zelenkaj    added:      - missing Microblaze support (minor)
    09.08.2011  zelenkaj    changed:    - RX buffer allocation for
                                          hook pending RX packets
    12.09.2011  zelenkaj    bugfix      - phyId on little endian incorrect
    03.11.2011  zelenkaj                - reverted previous Microblaze changes, since hw
                                          supports endian correctly
    11.04.2012  zelenkaj    bugfix      - search for phys starting with addr 1
                                        - added MICREL KSZ8051RNL support
    21.05.2012  muelhausens bugfix      - phyId on little endian==0 fix
    04.06.2012  zelenkaj    feature     - added omethPhyCfgUser
                                        - moved KSZ8051RNL for BeMicro support to
                                          omethlib_phycfg module

----------------------------------------------------------------------------*/

#include <omethlib.h>
#include <omethlibint.h>
#include <omethlib_phycfg.h>
#include <string.h>                // used functions: memcpy, memset
#include <stdlib.h>                // used functions: calloc
#include <omethlib_target.h>       // target specific defines (BIG/LITTLE endian)


// check if target-specific defines are ok
#ifndef OMETH_HW_MODE
    #error OMETH_HW_MODE not defined (see omethlib_target.h)
#endif

#if OMETH_HW_MODE > 1
    #error OMETH_HW_MODE invalid (see omethlib_target.h)
#endif


// only for special functions : mask and bits to adjust rx-mode in status register
#define OMETH_REG_RXMODE_MASK        0x0003

#define OMETH_REG_RXMODE_DEFAULT     0x0000
#define OMETH_REG_RXMODE_HIGH_PRIO   0x0001
#define OMETH_REG_RXMODE_BLOCK       0x0002
#define OMETH_REG_RXMODE_AUTO        0x0003


#define OMETH_MAX_RES_IPG            0x3F        // max response inter package gap

#define OMETH_MIN_TX_FRAME           60    // minimal transmit frame length (without checksum)

#define OMETH_INIT_LINK_SPEED        100

//--------- Response IPG reduction for systems with 1 phy ---------------
// (the real value will be higher because the lowest possible mac response ipg is 120ns)

#define MICREL_KS8721_PHY_ID        0x22161
#define    MICREL_KS8721_IPG        40            // due to phy runtime of 460ns the ipg can be reduced to 40 (40+2*460 = 960 ... minimum ipg)

#define MICREL_KSZ8051RNL_PHY_ID    0x22155
#define    MICREL_KSZ8051RNL_IPG    ~0    //TODO: measure IPG and compensate
//IPG compensation valid for EBV DBC3C40 with two National phys...
#define NATIONAL_DP83640_PHY_ID     0x20005CE
#define NATIONAL_DP83640_IPG        ~0    //TODO: measure IPG and compensate

#define MARVELL_88E1111_PHY_ID      0x001410CC
#define MARVELL_88E1111_IPG         ~0    //TODO: measure IPG and compensate


#define PHY_MICREL_REG1F_NOAUTOMDIX  0x2000

#define SMSC_LAN8700                 0x7C0C

#ifdef OMETH_SWAP_WIRE_PAIRS
    #define PHY_SMSC_REG1B_NOAUTOMDIX    0xA000
#else
    #define PHY_SMSC_REG1B_NOAUTOMDIX    0x8000
#endif

#define PHY_REG1F_OP_10HALF        1
#define PHY_REG1F_OP_10FULL        5
#define PHY_REG1F_OP_100HALF       2
#define PHY_REG1F_OP_100FULL       6

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

#define MII_REG_ENABLE              0x0080        // bit mask for reset bit in phy control

// constants for phy control
//MII Interface
#define PHY_REG_READ                0x6000    // MII read flags
#define PHY_REG_WRITE               0x5002    // MII write flags

#define SMI_REG_READ                0x4800    // SMI read flags  (micrel switch)
#define SMI_REG_WRITE               0x4002    // SMI write flags (micrel switch)

typedef struct ometh_internal_typ
{
    struct OMETH_TYP *pFirstEth;       // ptr to first instance handle
    struct OMETH_TYP *pPeriodicEth;    // ptr to currently used instance for periodic call
}ometh_internal_typ;

static ometh_internal_typ    omethInternal;    // driver internal data

#ifndef OMETH_MAX_RETRY
    #define OMETH_MAX_RETRY    0
#endif

#define OMETH_NO_RETRY     1

//*************************************************************************************
//    Begin of OMETH_TRANSMIT / Macro for omethTransmitXX()
//*************************************************************************************
#define    OMETH_TRANSMIT( ARG, TIME, addFlags, TX_QUEUE_INDEX )                                    \
    ometh_tx_info_typ    *pInfo    = hEth->pTxNext[TX_QUEUE_INDEX];    /* access to next tx info structure */   \
    ometh_desc_typ        *pDesc    = pInfo->pDesc;            /* access to tx descriptor */        \
    unsigned short        len;                                                                      \
                                                                                                    \
    /* check if descriptor is free */                                                               \
    if(pPacket == 0)                return 0;    /* invalid packet passed */                        \
    if(hEth->txQueueEnable == 0)    return 0;                                                       \
    if(pDesc->pData != 0)            return 0;    /* descriptor is not free (queue full !) */       \
                                                                                                    \
    len = pPacket->length;    /* padding, ethernet frames must be at least 64 byte long */          \
    if(len < OMETH_MIN_TX_FRAME) len=OMETH_MIN_TX_FRAME;                                            \
                                                                                                    \
    pDesc->pData    = (unsigned long)&pPacket->data;    /* write buffer ptr to descriptor */        \
    pDesc->len        = len;                                                                        \
    pInfo->fctFreeArg    = ARG;    /* store user argument for tx-callback */                        \
    pDesc->txStart        = TIME;    /* scheduled start time of this frame  */                      \
    /* (if reaching this descriptor the tx-queue will wait until the time is reached */             \
    pInfo->pFctFree    = (OMETH_BUF_FREE_FCT_ARG*)pFct;    /* store callback for free function */   \
                                                                                                    \
    hEth->cntTxQueueIn++;                                                                           \
    hEth->pTxNext[TX_QUEUE_INDEX] = pInfo->pNext;    /* switch to next info structure    */         \
                                                                                                    \
    if(addFlags == FLAGS1_START_TIME) /* in case of Time-Triggered TX no retry */                   \
    {                                                                                               \
        pDesc->flags.byte.low    = OMETH_NO_RETRY;                                                  \
    }                                                                                               \
    else                                                                                            \
    {                                                                                               \
        pDesc->flags.byte.low    = OMETH_MAX_RETRY;                                                 \
    }                                                                                               \
    pDesc->flags.byte.high    = pInfo->flags1 | addFlags;    /* set flag to start transmitter */    \
                                                                                                    \
    return len                                                                                      \

//*************************************************************************************
//    End of OMETH_TRANSMIT
//*************************************************************************************


//*************************************************************************************
//
// If the user has included assert.h in the file omethlib_target.h the functions will
// use assert-calls
//
//*************************************************************************************
#ifndef assert    // if assert.h is not defined ...
//    #define assert(x)    do{(x);}while(0)
    #define assert(x)
#endif


/* -- exchange table to allocate the next change index number for auto transmit descriptors --------------    */
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

// list of phy registers
static const unsigned char phyRegMapping[] = {0,1,2,3,4,5,6,7,0x1F};

/*****************************************************************************
*
* omethPacketFree - pass rx packet back to ethernet driver
*
*/
void        omethPacketFree
(
 ometh_packet_typ    *pPacket    /* address of rx packet    */
 )
{
    ometh_pending_typ    *pQueue;    // ptr to buffer queue
    OMETH_HOOK_H        hHook;        // hook handle leading to the data area of the buffer
    ometh_buf_typ        *pBuf;        // ptr to buffer

    // access to header of packet
    pBuf  = GET_TYPE_BASE( ometh_buf_typ, packet, pPacket);
    hHook = pBuf->hHook;

    if(pPacket==0 || hHook==0) return;    // invalid

    #ifdef OMETH_FREE_LOCK
        OMETH_FREE_LOCK
    #endif

    pQueue = hHook->pFreeWrite;

    pQueue->pBuf = pBuf;                  // pass buffer back to the hooks buffer list (buffer queue)

    hHook->pFreeWrite = pQueue->pNext;    // switch to next queue element

    #ifdef OMETH_FREE_UNLOCK
        OMETH_FREE_UNLOCK
    #endif
}

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
        ometh_packet_typ    *pPacket    /* packet which should be released    */
    )
    {
    }
#endif

/*****************************************************************************
*
* omethMiiControl - enable/disable phys
*
* RETURN:
*    -1   ... error
*     0-x ... depending on command
*        MII_CTRL_GET_STATE .. 0=reset 1=active
*
*    The following reasons can cause an error:
*    - pPhyBase = 0
*    - MII_CTRL_RESET and MII_CTRL_ACTIVE set at the same time
*
*/
int        omethMiiControl
(
 void            *pPhyBase,    /* ptr to phy register */
 unsigned short    command        /* combination of MII_CTRL_... values */
)
{
    // enable phys
    ometh_mii_typ* pPhy = OMETH_MAKE_NONCACHABLE(pPhyBase);

    if(pPhyBase==0) return -1;

    // reset and active at the same time ... not allowed
    if((command & MII_CTRL_RESET) && (command & MII_CTRL_ACTIVE)) return -1;

    if(command & MII_CTRL_RESET)
    {
        while(pPhy->control & MII_REG_ENABLE) pPhy->control &= (unsigned short)~MII_REG_ENABLE;
    }

    if(command & MII_CTRL_ACTIVE)
    {
        while((pPhy->control & MII_REG_ENABLE)==0) pPhy->control |= MII_REG_ENABLE;
    }

    // return 1 if state is requested and phy enable is set
    if((command & MII_CTRL_GET_STATE) && (pPhy->control & MII_REG_ENABLE) ) return 1;

    return 0;
}

/*****************************************************************************
*
* omethInit - initialize ethernet driver
*
*/
void    omethInit
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
static OMETH_H        omethCreateInt
(
 ometh_config_typ    *pEthConfig        /* ptr to ethernet config struct            */
)
{
    OMETH_H            hEth;
    unsigned long    len,i,phyId;
    unsigned char    *pByte;
    unsigned short    data,readData;
    unsigned char    nbTxDesc[2]={0,0};

    ometh_desc_typ            *pDesc;
    ometh_tx_info_typ        *pTxInfo;
    ometh_rx_info_typ        *pRxInfo;
    ometh_packet_typ        *pPacket;
    ometh_filter_typ        *pFilter;
    ometh_reg_typ            *pRegBase;
    struct OMETH_FILTER        *pFilterList;

    // check mempart handle and return if invalid
    if(pEthConfig->rxBuffers == 0) return 0;

    // check mode, if no duplex flag is set .. use half duplex
    if((pEthConfig->mode & (OMETH_MODE_FULLDUPLEX|OMETH_MODE_HALFDUPLEX)) == 0) pEthConfig->mode |= OMETH_MODE_HALFDUPLEX;

    // check speed, use 100MBit if no flag is set
    if((pEthConfig->mode & (OMETH_MODE_10MBIT|OMETH_MODE_100MBIT)) == 0) pEthConfig->mode |= OMETH_MODE_100MBIT;

    // validate config type (to avoid resource conflicts)
    for( hEth = omethInternal.pFirstEth ; hEth ; hEth = hEth->pNext )
    {
        // adapter number already used
        if(pEthConfig->adapter  == hEth->config.adapter)  return 0;

        if(pEthConfig->pRamBase == hEth->config.pRamBase) return 0;
        if(pEthConfig->pRegBase == hEth->config.pRegBase) return 0;
        if(pEthConfig->pRegBase == hEth->config.pBufBase) return 0;
    }

    pRegBase = OMETH_MAKE_NONCACHABLE(pEthConfig->pRegBase);

    // initialize control registers
    pRegBase->rxStatus.clrBit        = 0xFFFF;            // clear all bits in rx status
    pRegBase->rxStatus.setDescriptor = 0;

    // clear all bits in tx status except the hub-enable
    pRegBase->txStatus.clrBit        = ~(unsigned short)OMETH_REG_HALF;
    pRegBase->txStatus.setDescriptor = 0x0008;    // for MAC_TYP_03 .. reset queue index for 2nd tx queue

    // only halfduplex is allowed ... no fullduplex -> set half-bit in mac register (which also enables the HUB)
    if (((pEthConfig->mode & OMETH_MODE_HALFDUPLEX) != 0) && ((pEthConfig->mode & OMETH_MODE_FULLDUPLEX) == 0))
    {
        pRegBase->txStatus.setBit = OMETH_REG_HALF;
    }

    // clear all pending rx irqs
    while(pRegBase->rxStatus.value & OMETH_REG_PENDING) pRegBase->rxStatus.clrBit = OMETH_REG_IQUIT;

    // clear all pending rx-fail irq's
    while(pRegBase->rxStatus.value & OMETH_REG_RX_NOMATCH) pRegBase->rxStatus.clrBit = OMETH_REG_RX_NOMATCH;

    // clear all pending rx irqs
    while(pRegBase->txStatus.value & OMETH_REG_PENDING) pRegBase->txStatus.clrBit = OMETH_REG_IQUIT;

    // clear all pending tx-beg irqs
    while(pRegBase->txStatus.value & OMETH_REG_TX_BEG) pRegBase->txStatus.clrBit = OMETH_REG_TX_BEG;

    hEth = calloc(sizeof(struct OMETH_TYP),1);    // allocate memory for instance handle
    assert(hEth);
    if(hEth == 0) return 0;                        // return if alloc failed - system is out of memory

    // at the first start all pending IRQs shall be acknowledged
    hEth->clearPendingIrqAtStart = 1;

    // copy config structure to handle
    memcpy(&hEth->config, pEthConfig, sizeof(hEth->config));

    //convert pointers to hardware to non-cachable pointers
    hEth->config.pPhyBase = OMETH_MAKE_NONCACHABLE(hEth->config.pPhyBase);
    hEth->config.pRamBase = OMETH_MAKE_NONCACHABLE(hEth->config.pRamBase);
    hEth->config.pRegBase = OMETH_MAKE_NONCACHABLE(hEth->config.pRegBase);
    if(hEth->config.pktLoc == OMETH_PKT_LOC_MACINT)
    {
        hEth->config.pBufBase = OMETH_MAKE_NONCACHABLE(hEth->config.pBufBase);
    }


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
        omethPhyRead(hEth, hEth->phyCount-1, 1, &data);    // get register 1 to count linked ports at startup

        if(data==0xFFFF)
        {
            hEth->phyCount--;
            continue;                    // phy not connected, look for next
        }

        if(data & PHY_REG1_LINK) hEth->phyLinkCount++;    // count links at startup
    }

    if( (hEth->config.mode & OMETH_MODE_PHY_LIST) && (hEth->config.phyCount==0) )
    {
        // if phy-list-flag is set, and phyCount in configuration is 0 ... user knows that there are no phys to be configured
        phyId = 0;
    }
    else
    {
        if(hEth->phyCount==0) return hEth;    // error ... no phys found

        // get phy IDs (Reg 2 and 3) to 32Bit var phyId
        // note: big endian reads reg 2 first, otherwise reg 3 is read first
#if OMETH_HW_MODE == 0 //little
        omethPhyRead(hEth, 0, 3, &data);
#else //big
        omethPhyRead(hEth, 0, 2, &data);
#endif
        phyId = data;
#if OMETH_HW_MODE == 0 //little
        omethPhyRead(hEth, 0, 2, &data);
#else //big
        omethPhyRead(hEth, 0, 3, &data);
#endif
        phyId |= ((unsigned long)data << 16);
        phyId = phyId >> 4;    // remove revision number
    }

    // check if there is a special response IPG to be set
    i = ~0;

    // IPG controlled by user
    if(pEthConfig->mode & OMETH_MODE_SET_RES_IPG)
    {
        i = pEthConfig->responseIpg;
    }
    // reduce IPG automatically depending on phy type (if there is just 1 phy existing)
    else if (hEth->phyCount==1)
    {
        if(phyId==MICREL_KS8721_PHY_ID)    i = MICREL_KS8721_IPG;
    }
    // reduce IPG automatically depending on phy type (if there are 2 phy existing)
    else if (hEth->phyCount==2)
    {
        //e.g. EBV DB3C40 BOARD (Cyclone 3)
        if(phyId==NATIONAL_DP83640_PHY_ID) i = NATIONAL_DP83640_IPG;
        //e.g. TERASIC DE2-115 (Cyclone 4)
        else if(phyId==MARVELL_88E1111_PHY_ID) i = MARVELL_88E1111_IPG;
        //e.g. ARROW BeMicro
        else if(phyId==MICREL_KSZ8051RNL_PHY_ID) i = MICREL_KSZ8051RNL_IPG;
        //...add more if needed
    }

    // response IPG is defined, calculate value and write to descriptor
    if(i != ~0)
    {
        // calculate value for ipg-register (80 ns offset, 20ns resolution)
        if(i > 80)    i = (i-80)/20;
        else        i = 1;            // value too small, take minimum

        if (i > OMETH_MAX_RES_IPG) i = OMETH_MAX_RES_IPG;    // limit to maximum

        i = OMETH_REG_SET_RES_IPG | (i << 8);                // move to proper bit position
    }
    else
    {
        i = 0;    // no IPG to be set, clear all bits
    }

    // write IPG value to tx status register
    pRegBase->txStatus.setDescriptor = i;

    hEth->pRegBase    = hEth->config.pRegBase;
    hEth->pHardware    = hEth->config.pRamBase;
    pFilter            = hEth->config.pRamBase;

    // overtake parameters to instance handle
    hEth->rxLen = sizeof(pPacket->data) - sizeof(pPacket->data.minData) + hEth->config.rxMtu;

    // get size of DPR
    switch(hEth->config.macType)
    {
        case OMETH_MAC_TYPE_01:    // Typ 01
            hEth->nbFilter    = MAC_HW_TYP_01_NB_FILTER;

            len = sizeof(ometh_filter_typ) * hEth->nbFilter;

            // too many rx buffers configured (hardware dependent)
            if(hEth->config.rxBuffers > MAC_HW_TYP_01_NB_RXDESC) return hEth;

            // allocate structure with rx info
            hEth->pRxInfo    = calloc(hEth->config.rxBuffers, sizeof(ometh_rx_info_typ));

            if(hEth->pRxInfo == 0) return hEth;

            // write ptr to first rx descriptor to first info structure
            hEth->pRxInfo->pDesc = (ometh_desc_typ*)((size_t)hEth->config.pRamBase + len);

            len    = len + sizeof(ometh_desc_typ) * (MAC_HW_TYP_01_NB_RXDESC);

            // allocate structure with tx info
            nbTxDesc[0] = MAC_HW_TYP_01_NB_TXDESC;

            hEth->pTxInfo[0] = calloc(nbTxDesc[0] , sizeof(ometh_tx_info_typ));
            if(hEth->pTxInfo[0] == 0) return hEth;

            // write ptr to first tx descriptor to first info structure
            hEth->pTxInfo[0]->pDesc = (ometh_desc_typ*)((size_t)hEth->config.pRamBase + len);

            len = len + sizeof(ometh_desc_typ) * nbTxDesc[0];
            break;

        default:
            return hEth;            // not allowed
    }

    // start with first tx buffer
    memset(hEth->config.pRamBase, 0, len);            // reset DPR

    //----------------- set all tx descriptor pointers in info structure ----------
    for(i=1;i != -1;i--)    // process first [1] and then [0]
    {
        pTxInfo = hEth->pTxFree[i] = hEth->pTxNext[i] = hEth->pTxInfo[i];
        if(pTxInfo == 0) continue;

        pDesc = pTxInfo->pDesc;

        for(len=0 ; len < nbTxDesc[i] ; len++)
        {
            pTxInfo->flags1    = FLAGS1_OWNER;
            pTxInfo->pDesc    = pDesc;            // tx descriptor ptr
            pTxInfo->pNext    = pTxInfo+1;        // ptr to next info
            pTxInfo->index    = len;

            pTxInfo++;
            pDesc++;
        }

        pTxInfo--;    // switch back to last info structure
        pTxInfo->flags1    = FLAGS1_OWNER | FLAGS1_LAST;
        pTxInfo->pNext    = hEth->pTxInfo[i];                // ptr to first info

        // auto transmit descriptors start from the end of the last tx queue
        if(hEth->pTxAuto==0) hEth->pTxAuto = pTxInfo;
    }


    //----------------- allocate buffers for rx descriptors -----------------------

    // calc length of 1 rx buffer ( header + ethernet-header(14) + ip(mtu) + checksum(4)  )
    len = sizeof(ometh_buf_typ) - sizeof(pPacket->data.minData) + hEth->config.rxMtu;

    len = (len+3)&(~3);        // round up to next multiple of 4

    // allocate buffers for rx data
    if(hEth->config.pktLoc == OMETH_PKT_LOC_MACINT)
    {
        //use mac internal packet buffer
        pByte = hEth->config.pBufBase;

        //store tx buffer address for appi
        hEth->pTxBufBase = pByte + hEth->config.rxBuffers * len;
    }
    else if(hEth->config.pktLoc == OMETH_PKT_LOC_HEAP)
    {
        //use heap
        pByte = (unsigned char*)
                OMETH_UNCACHED_MALLOC(hEth->config.rxBuffers * len);

        //store tx buffer address equ. rx buffer -> tx is handled by user!
        hEth->pTxBufBase = pByte;
    }
    else
    {
        //error
        return hEth;
    }

    if(pByte == 0) return hEth;

    // store rx buffer address for appi and destroy function
    hEth->pRxBufBase = pByte;

    //----------------- set all rx descriptor pointers in info structure ----------
    pRxInfo    = hEth->pRxNext = hEth->pRxInfo;

    pDesc    = pRxInfo->pDesc;

    for(i=0 ; i<hEth->config.rxBuffers ; i++)
    {
        pDesc->flags.byte.high = pRxInfo->flags1 = FLAGS1_OWNER;

        pRxInfo->pDesc    = pDesc;        // rx descriptor ptr
        pRxInfo->pNext    = pRxInfo+1;    // ptr to next info

        pDesc->len        = hEth->rxLen;
        pDesc->pData     = (unsigned long)&((ometh_buf_typ*)pByte)->packet.data;

        pByte = pByte + len;            // switch to next allocated buffer

        pRxInfo++;
        pDesc++;
    }

    pRxInfo--;    // switch back to last info structure
    pDesc--;    // switch to last descriptor

    pDesc->flags.byte.high = pRxInfo->flags1    = FLAGS1_OWNER | FLAGS1_LAST;

    pRxInfo->pNext = hEth->pRxInfo;                // ptr to first info

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

        if(i < (hEth->nbFilter - hEth->nbFilterX))    // normal filter
        {
            pFilterList->pFilterData->len        = OMETH_FILTER_LEN;
            pFilterList->pFilterData->pCommand    = &pFilter->command;
        }
        else    // X-Filter
        {
            pFilterList->pFilterData->len        = OMETH_XFILTER_LEN;
            pFilterList->pFilterData->pCommand    = &((ometh_xfilter_typ*)pFilter)->command;
        }

        pFilterList++;    // next filter list entry
        pFilter++;        // next filter in RAM

        pByte = pByte + sizeof(ometh_filter_data_typ);    // next filter data
    }

    // set possible auto-negotiate baud rates for phy depending on mode
    // initialize all phy's and switch on/off hub
    data = PHY_REG4_SELECTOR;        // generate value for register 4 (advertisement register)

    if(hEth->config.mode & OMETH_MODE_FULLDUPLEX)
    {
        if(hEth->config.mode & OMETH_MODE_100MBIT) data |= PHY_REG4_100TX_FULL;
        if(hEth->config.mode & OMETH_MODE_10MBIT) data |= PHY_REG4_10T_FULL;
    }

    // half duplex
    if(hEth->config.mode & OMETH_MODE_HALFDUPLEX)
    {
        if(hEth->config.mode & OMETH_MODE_100MBIT) data |= PHY_REG4_100TX_HALF;
        if(hEth->config.mode & OMETH_MODE_10MBIT) data |= PHY_REG4_10T_HALF;
    }

    hEth->phyLinkCount = 0;

    // remember advertisement register for omethPeriodic()
    hEth->r4Init = data;

    // search for connected phys
    for(i=0 ; i < hEth->phyCount ; i++)
    {
        len = 0;

        // autoneg disabled for all ports
        if(hEth->config.mode & OMETH_MODE_DIS_AUTO_NEG)    len = 1;

        // check if autoneg is specially disabled for this port
        if( (i<8) && ((hEth->config.mode>>i) & OMETH_MODE_DIS_AUTO_NEG_P0) ) len = 1;

        if(len)    // disable autoneg for this port
        {
            data = 0;

            if(hEth->config.mode & OMETH_MODE_FULLDUPLEX) data |= PHY_REG0_FULL;
            if(hEth->config.mode & OMETH_MODE_100MBIT)    data |= PHY_REG0_100;

            omethPhyRead(hEth,i,0,&readData);

            // write reg0 to turn autoneg off
            if(readData != data) omethPhyWrite(hEth, i, 0, data);

            // also turn auto MDI/MDI-X off if autoneg is off
            if(phyId==MICREL_KS8721_PHY_ID)
            {
                omethPhyWrite(hEth, i, 0x1F, PHY_MICREL_REG1F_NOAUTOMDIX);
            }
            if(phyId==SMSC_LAN8700)
            {
                omethPhyWrite(hEth, i, 0x1B, PHY_SMSC_REG1B_NOAUTOMDIX);
            }
        }
        else
        {
            len = 0;    // flag to detect if reset is required

            omethPhyRead(hEth,i,4,&readData);
            if(readData != hEth->r4Init) len=1;    // set flag to restart autoneg if register 4 is not the desired value

            omethPhyWrite(hEth, i, 4, hEth->r4Init);    // set allowed modes (reg 4 , advertisement register)

            // readback register 4 from phy to make sure the phy is existing, otherwise error
            omethPhyRead(hEth,i,4,&readData);
            if(readData != hEth->r4Init) return hEth;

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
    hEth->pPhyReg = calloc(hEth->phyCount, sizeof(*hEth->pPhyReg));
    if(hEth->pPhyReg == 0) return hEth;

    //--------------------------------- call user's phy cfg -----------------------------------------
    if(omethPhyCfgUser(hEth) != 0)
    {
        return hEth;
    }

    #if (OMETH_ENABLE_SOFT_IRQ==1)
        hEth->pFctSoftIrq = &soft_irq_dummy;    // set dummy function for Soft-IRQ
    #endif

    hEth->linkSpeed        = OMETH_INIT_LINK_SPEED;    // start with speed 100, will be reduced if 10MBit link is detected
    hEth->txQueueEnable    = 1;                        // by default the tx queue is enabled

    return hEth;    // return instance handle
}

/*****************************************************************************
*
* omethCreate - create and initialize new instance of ethernet driver
*
*/
OMETH_H            omethCreate
(
 ometh_config_typ    *pEthConfig        /* ptr to ethernet config struct            */
)
{
    OMETH_H hEth;

    hEth = omethCreateInt(pEthConfig);    // call internal create function
    assert(hEth);
    if(hEth==0) return 0;

    // instance was allocated but not successful initialized
    if(hEth->txQueueEnable == 0)
    {
        omethDestroy(hEth);
        assert(0);
        return 0;
    }

    // add driver instance to list
    hEth->pNext                = omethInternal.pFirstEth;
    omethInternal.pFirstEth    = hEth;

    return hEth;
}

/*****************************************************************************
*
* omethGetHandle - get adapter handle
*
*/
OMETH_H            omethGetHandle
(
 int    adapter        /* number of network adapter (0..255) */
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
int                omethPhyHardwareAdr
(
 OMETH_H        hEth,        /* handle of ethernet driver, see omethCreate()        */
 unsigned short    port        /* phy number / port number of integrated hub (0-n)    */
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
phy_reg_typ*    omethPhyInfo
(
 OMETH_H        hEth,        /* handle of ethernet driver, see omethCreate()        */
 unsigned short    port        /* phy number / port number of integrated hub (0-n)    */
)
{
    if(hEth==0) return 0;

    if(port >= hEth->phyCount) return 0;

    return hEth->pPhyReg + port;    // return address
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
phy_stat_enum        omethPhyLinkState
(
 OMETH_H            hEth,        /* handle of ethernet driver, see omethCreate()        */
 unsigned short        port        /* phy number / port number of integrated hub (0-n)    */
)
{
    if(hEth==0)                    return OMETH_PHY_STATE_INVALID;
    if(port >= hEth->phyCount)    return OMETH_PHY_STATE_INVALID;

    port = (hEth->pPhyReg[port].r[8] >> 2) & 7;

    if(port==PHY_REG1F_OP_100HALF  || port==PHY_REG1F_OP_10HALF)  return OMETH_PHY_STATE_HALF;
    if(port==PHY_REG1F_OP_100FULL  || port==PHY_REG1F_OP_10FULL)  return OMETH_PHY_STATE_FULL;

    return OMETH_PHY_STATE_NOLINK;
}

/*****************************************************************************
*
* omethGetLinkSpeed - Get link speed of a adapter
*
* RETURN:
*     0     ... handle invalid or speed not known
*     10  ... 10 MBit
*     100 ... 100 MBit
*
*/
unsigned short        omethGetLinkSpeed
(
 OMETH_H            hEth        /* handle of ethernet driver, see omethCreate()        */
)
{
    if(hEth==0) return 0;

    return hEth->linkSpeed;
}

/*****************************************************************************
*
* omethGetTxBufBase - Get MAC-internal tx buffer base address
*
* RETURN:
*    tx buffer base pointer
*
*/
unsigned char *     omethGetTxBufBase
(
 OMETH_H            hEth        /* handle of ethernet driver, see omethCreate()     */
)
{
    return hEth->pTxBufBase;
}

/*****************************************************************************
*
* omethGetRxBufBase - Get MAC-internal rx buffer base address
*
* RETURN:
*    tx buffer base pointer
*
*/
unsigned char *     omethGetRxBufBase
(
 OMETH_H            hEth        /* handle of ethernet driver, see omethCreate()     */
)
{
    return hEth->pRxBufBase;
}

/*****************************************************************************
*
* omethGetConfigMode - Get ethernet config mode
*
* RETURN:
*   handle invalid or speed not known:  0
*   config mode:                        != 0
*
*
*/
unsigned short        omethGetConfigMode
(
 OMETH_H            hEth        /* handle of ethernet driver, see omethCreate()        */
)
{
    if(hEth==0) return 0;

    return hEth->config.mode;
}



/*****************************************************************************
*
* omethPhyRead - Read Phy register
*
* !!! If the firmware uses the function omethPeriodic():
* The function can only be called in the same context as omethPeriodic()
*
* port  0 .. 15          : port index of this ethernet instance
* port  0x8000 .. 0x801F : the low byte is the phy-address
* port  0x4000           : SMI interface for micrel switch
*
* RETURN:
*     0    ... no error
*    -1    ... error
*        hEth invalid / port too high / reg too high
*
*/
int                    omethPhyRead
(
 OMETH_H            hEth,        /* handle of ethernet driver, see omethCreate()        */
 unsigned short        port,        /* phy number / port number of integrated hub (0-n)    */
                                /* interpret port as phy-address if bit 15 is set */

 unsigned short        reg,        /* read register number */
 unsigned short        *pValue        /* ptr to read value */
)
{
    ometh_mii_typ    *pMII = hEth->config.pPhyBase;
    unsigned short    dataBackup;

    if(hEth==0) return -1;

    while(pMII->cmd.ack & 1);    // wait until busy = 0
    dataBackup = pMII->data;    // backup data in case omethPeriodic() was waiting for a register

    if(port & 0x8000)        port = ((port&0x1F)<<7) | PHY_REG_READ;    // generate phy-read command from phy-address
    else if(port & 0x4000)    port = SMI_REG_READ | ((reg&0xE0)<<2);    // patch the higher 3 bits of the register number to phy address
    else                     port = hEth->phyCmdRead[port];            // take phy read command from ethernet-instance

    pMII->cmd.req = port | ( (reg&0x1F)<<2 );

    while(pMII->cmd.ack & 1);                        // wait until busy = 0

    if(pValue) *pValue = pMII->data;    // get response
    pMII->data = dataBackup;            // restore backup for next omethPeriodic() call

    return 0;
}

/*****************************************************************************
*
* omethPhyWrite - Write Phy register
*
* !!! If the firmware uses the function omethPeriodic():
* The function can only be called in the same context as omethPeriodic()
*
* port  0 .. 15          : port index of this ethernet instance
* port  0x8000 .. 0x801F : the low byte is the phy-address
* port  0x4000           : SMI interface for micrel switch
*
* RETURN:
*     0    ... no error
*    -1    ... error
*        hEth invalid / port too high / reg too high
*
*/
int                    omethPhyWrite
(
 OMETH_H            hEth,        /* handle of ethernet driver, see omethCreate()        */
 unsigned short        port,        /* phy number / port number of integrated hub (0-n)    */
 unsigned short        reg,        /* read register number */
 unsigned short        value        /* value */
)
{
    ometh_mii_typ *pMII = hEth->config.pPhyBase;
    unsigned short    dataBackup;

    if(hEth==0) return -1;

    while(pMII->cmd.ack & 1);    // wait until busy = 0
    dataBackup = pMII->data;    // backup data in case omethPeriodic() was waiting for a register

    if(port & 0x8000)        port = ((port&0x1F)<<7) | PHY_REG_WRITE;    // generate phy-read command from phy-address
    else if(port & 0x4000)    port = SMI_REG_WRITE | ((reg&0xE0)<<2);        // patch the higher 3 bits of the register number to phy address
    else                    port = hEth->phyCmdWrite[port];                // take phy read command from ethernet-instance

    pMII->data = value;
    pMII->cmd.req = port | ( (reg&0x1F) <<2 );
    while(pMII->cmd.ack & 1);    // wait until busy = 0

    pMII->data = dataBackup;    // restore backup for next omethPeriodic() call
    return 0;
}

/*****************************************************************************
*
* omethPhyReadNonBlocking - Read Phy register (non blocking)
*
* !!! use either omethPeriodic() or omethPhyReadNonBlocking(), not both !!!
*
*        - omethPeriodic() is required for single port interfaces (no hub) which support
*          full+half duplex. In this case  omethPhyReadNonBlocking() can not used.
*
*        - omethPhyReadNonBlocking() can only be used for systems forced to either full
*          or half-duplex
*
* port  0 .. 15          : port index of this ethernet instance
* port  0x8000 .. 0x801F : the low byte is the phy-address
* port  0x4000           : SMI interface for micrel switch
*
* RETURN:
*     0    ... no error : *pValue contains the value of the register passed at the previous call !
*    -1    ... blocking : *pValue was not modified
*
*/
int                    omethPhyReadNonBlocking
(
 OMETH_H            hEth,        /* handle of ethernet driver, see omethCreate()        */
 unsigned short        port,        /* phy number / port number of integrated hub (0-n)    */
 unsigned short        reg,        /* read register number                                */
 unsigned short        *pValue        /* ptr to read value                                */
)
{
    ometh_mii_typ*    pMII = hEth->config.pPhyBase;

    if (pMII->cmd.ack & 1) return -1;            // busy -> blocking

    if(port & 0x8000)        port = ((port&0x1F)<<7) | PHY_REG_READ;    // generate phy-read command from phy-address
    else if(port & 0x4000)    port = SMI_REG_READ | ((reg&0xE0)<<2);    // patch the higher 3 bits of the register number to phy address
    else                    port = hEth->phyCmdRead[port];            // take phy read command from ethernet-instance

    *pValue            = pMII->data;                // get response
    pMII->cmd.req = port | ( (reg&0x1F) <<2 );    // set new read commando

    return 0;
}

/*****************************************************************************
*
* omethSetHalfDuplex - set phy's halfduplex (the phy's will be set in omethPerodic())
*
* RETURN:
*     0    ... no error
*    -1    ... error (you must call omethPerodic first)
*/
int            omethPhySetHalfDuplex
(
 void
)
{
    OMETH_H hEth = omethInternal.pPeriodicEth;

    if (hEth == 0)    return -1;    // you must call omethPeriodic first

    hEth->config.mode |= OMETH_MODE_FULLDUPLEX;
    return 0;
}

/*****************************************************************************
*
* omethPeriodic - periodic call of ethernet driver
*
*/
void            omethPeriodic
(
 void
)
{
    OMETH_H            hEth = omethInternal.pPeriodicEth;
    phy_reg_typ        *pPhyReg;    // ptr to registers of processed phy
    ometh_mii_typ    *pMII;        // ptr to MII
    unsigned short    x,r4,speed; //WORD            x,r4,speed;

    if(hEth==0)    // initialize periodic if not yet done or if reset by a destroy-call
    {
        // reset phy-state machines of all instances
        for(hEth = omethInternal.pFirstEth ; hEth->pNext ; hEth = hEth->pNext)
        {
            if(hEth==0) return;    // no instance available

            hEth->txVal            = 0;
            hEth->phyReg        = 0;
            hEth->phyPort        = 0;    // start with port 0 if all are done
            hEth->phyLinkCount    = 0;    // reset link counter for next round
            hEth->phyHalfCount    = 0;    // counts all links with half duplex connection
        }

        omethInternal.pPeriodicEth = hEth;        // start periodic phy control with last found instance

        pMII = hEth->config.pPhyBase;
        pMII->cmd.req = hEth->phyCmdRead[0];    // initiate read command of phy 0 / register 0

        return;
    }

    pMII = hEth->config.pPhyBase;    // ptr to MII
    if (pMII->cmd.ack & 1)
    {
        return;    // mii busy
    }

    // switch to next instance if no phy is configured on this interface
    if(hEth->phyCount == 0)
    {
        // switch to next adapter
        hEth = hEth->pNext;
        if(hEth==0) hEth = omethInternal.pFirstEth;    // take first instance if this was the last
        omethInternal.pPeriodicEth = hEth;    // save for next periodic call

        // start read command
        pMII->cmd.req = hEth->phyCmdRead[0];    // initiate read command of phy 0 / register 0
        return;
    }

    // check if tx-job for port0/reg4 is pending
    if(hEth->txVal)
    {
        pMII->data    = hEth->txVal;
        pMII->cmd.req = hEth->phyCmdWrite[hEth->txPort] | (hEth->txReg << 2);

        hEth->txVal = 0;

        if(hEth->txReg==4)    // reset phy after writing register 4 (write autoneg-restart to register 0)
        {
            hEth->txVal = PHY_REG0_AUTONEG_ENABLE | PHY_REG0_AUTONEG_RESTART;
            hEth->txReg    = 0;
        }

        return;    // return to wait until ack bit goes to 0
    }

    pPhyReg = hEth->pPhyReg + hEth->phyPort;    //  access to phy register structure

    // get data from MII
    ((unsigned short*)(pPhyReg))[hEth->phyReg] = pMII->data;

    if(pMII->data == 0xFFFF) hEth->phyOffline = 1;

    hEth->phyReg++;
    if(hEth->phyReg >= sizeof(pPhyReg->r)/sizeof(pPhyReg->r[0]))
    {
        r4 = 0;

        // evaluate speed and duplex setting

        x = (pPhyReg->r[8] >> 2) & 7;    // get bit 4-2 which contains link status
        speed = 0;
        if(x==PHY_REG1F_OP_100HALF || x==PHY_REG1F_OP_100FULL) speed = 100;
        if(x==PHY_REG1F_OP_10HALF  || x==PHY_REG1F_OP_10FULL)  speed = 10;

        // count linked ports and switch to force duplex if required
        if(speed && (hEth->phyOffline==0) && (pPhyReg->r[1]&PHY_REG1_LINK) )
        {
            hEth->phyLinkCount++;

            if(speed != hEth->linkSpeed)    // link speed different to current detected link speed, something to do
            {
                if(speed > hEth->linkSpeed)    // reduce speed of this link if too fast
                {
                    r4 = pPhyReg->r[4] & ~(unsigned short)(PHY_REG4_100TX_FULL | PHY_REG4_100TX_HALF);
                }
                else    // otherwise change speed to 10 MBit
                {
                    hEth->pRegBase->rxStatus.setBit = OMETH_REG_10MBIT; // manipulate 10/100 flag
                    hEth->linkSpeed = 10;
                }
            }

            // link is made with full duplex, change phy register if mac is forced to half duplex
            if(x==PHY_REG1F_OP_10FULL || x==PHY_REG1F_OP_100FULL)
            {
                // (do not switch phys to half duplex if mac_type 2 is selected)
                if(((hEth->phyLinkActive > 1) || (hEth->phyHalfMax > 0) || ((hEth->config.mode & OMETH_MODE_FULLDUPLEX) == 0)))
                {
                    if(r4==0) r4 = pPhyReg->r[4];    // get r4 if not yet generated by last step

                    // remove full-duplex flags, add half duplex flags
                    r4 = r4 & ~(unsigned short)(PHY_REG4_100TX_FULL | PHY_REG4_10T_FULL);
                }
            }
            else
            {
                // counter for half-duplex link if local or remote does not support full duplex
                hEth->phyHalfCount++;
            }
        }
        // phy not linked, set advertisement register
        // (to avoid the phy links with too high speed, or with full duplex if not allowed)
        else
        {
            r4 = hEth->r4Init;

            // remove 100MBit if there is at least one 10MBit link established
            if(hEth->linkSpeed < 100)    r4 = r4 & ~(unsigned short)(PHY_REG4_100TX_FULL | PHY_REG4_100TX_HALF);

            // remove Full duplex capability if another port is connected
            if(hEth->phyHalfMax || hEth->phyLinkActive) r4 = r4 & ~(unsigned short)(PHY_REG4_100TX_FULL | PHY_REG4_10T_FULL);
        }

        // generate register-write if requested (and if new value is different to current setting)
        if(r4>0 && r4!=pPhyReg->r[4])
        {
            hEth->txVal  = r4;
            hEth->txReg  = 4;
            hEth->txPort = hEth->phyPort;
        }

        hEth->phyPort++;        // next port ...
        hEth->phyReg = 0;        // ... start at register 0
        hEth->phyOffline = 0;    // will be set if any register of the phy shows 0xFFFF

        // start with first port if all ports were processed
        if(hEth->phyPort >= hEth->phyCount)
        {
            hEth->phyLinkActive = hEth->phyLinkCount;  // store total number of linked ports
            if(hEth->phyHalfCount > hEth->phyHalfMax) hEth->phyHalfMax = hEth->phyHalfCount;

            // change back to 100 MBit (and setup MAC) (not for MACs with more than 1 port)
            if(hEth->phyLinkActive==0 && hEth->phyCount==1)
            {
                if(hEth->linkSpeed == 10) hEth->pRegBase->rxStatus.clrBit = OMETH_REG_10MBIT;
                hEth->linkSpeed        = OMETH_INIT_LINK_SPEED;
                hEth->phyHalfMax    = 0;
            }

            // set mac to half mode if ever more than 2 ports were active at the same time or the only active
            // port is linked with half duplex
            // (also set half-bit if MAC Type 2 is selected ... this mode always requires a half duplex mac
            if((hEth->phyLinkActive > 1) || ((hEth->config.mode & OMETH_MODE_FULLDUPLEX) == 0) ||
                (hEth->phyHalfCount > 0))
            {
                hEth->pRegBase->txStatus.setBit = OMETH_REG_HALF;
            }
            else
            {
                hEth->pRegBase->txStatus.clrBit = OMETH_REG_HALF;
            }

            hEth->phyPort        = 0;    // start with port 0 if all are done
            hEth->phyLinkCount    = 0;    // reset link counter for next round
            hEth->phyHalfCount    = 0;    // counts all links with half duplex connection

            // also switch to next adapter
            hEth = hEth->pNext;
            if(hEth==0) hEth = omethInternal.pFirstEth;    // take first instance if this was the last

            omethInternal.pPeriodicEth = hEth;    // save for next periodic call
        }
    }

    // start read command
    pMII->cmd.req = hEth->phyCmdRead[hEth->phyPort] | (phyRegMapping[hEth->phyReg]<<2);
}

/*****************************************************************************
*
* omethHookCreate - create a new hook for a client
*
*/
OMETH_HOOK_H    omethHookCreate
(
 OMETH_H        hEth,        /* handle of ethernet driver, see omethCreate() */
 OMETH_HOOK_FCT    *pFct,        /* callback function                            */
 unsigned short    maxPending    /* maximum number of pending buffers            */
)
{
    OMETH_HOOK_H        hHook;
    unsigned long        len,i;
    ometh_pending_typ    *pQueue;
    ometh_buf_typ        *pBuf;

    assert(hEth);
    assert(pFct);

    if(hEth==0 || pFct==0) return 0;

    if(pFct == OMETH_HOOK_DISABLED) pFct=0;

    // allocate structure array for hook list (one hook for each filter)
    hHook = calloc(sizeof(struct OMETH_HOOK) + maxPending * sizeof(ometh_pending_typ), 1);
    assert(hHook);
    if(hHook==0) return 0;

    // add to the hook list of the instance to be able to destroy complete instance
    hHook->pNext    = hEth->pHookList;
    hEth->pHookList    = hHook;

    hHook->cntOverflow    = 0;
    hHook->pFct            = pFct;
    hHook->hEth            = hEth;

    pQueue                = hHook->free;    // access to first queue entry
    hHook->maxPending    = maxPending;    // overtake max-pending value to hook (just for debug)

    if(maxPending)
    {
        // allocate buffers for this client and add to the buffer list
        len = sizeof(ometh_buf_typ)-sizeof(pBuf->packet.data) + hEth->rxLen;

        len = (len+3)&(~3);        // round up to next multiple of 4

        hHook->pFreeRead = hHook->free;

        //pBuf = calloc(len * maxPending, 1);
        //------------------------------------------------------------------------------
        if(hEth->config.pktLoc == OMETH_PKT_LOC_MACINT)
        {
            //pool has to be allocated in MAC internal packet buffer
            pBuf = (ometh_buf_typ*)hEth->pTxBufBase;

            //increment TX buffer base
            /* FIXME: There is no check done if we are out of memory, host has to provide
             * sufficiently large internal memory!
             */
            hEth->pTxBufBase = (unsigned char*)pBuf + len * maxPending;
        }
        else if(hEth->config.pktLoc == OMETH_PKT_LOC_HEAP)
        {
            //pool has to be allocated in heap, since RX packets are stored there
            pBuf = (ometh_buf_typ*)OMETH_UNCACHED_MALLOC(len * maxPending);
        }
        else
        {
            //error
            return 0;
        }
        //------------------------------------------------------------------------------

        assert(pBuf);
        if(pBuf==0) return 0;

        hHook->pRxBufBase = pBuf;    // store for destroy function

        // fill list with buffer pointers
        for(i=0; i<maxPending ; i++)
        {
            pQueue->pBuf    = pBuf;
            pQueue->pNext    = pQueue+1;

            pQueue++;
            pBuf = (ometh_buf_typ*)((size_t)pBuf + len);    // next buffer
        }
    }

    pQueue->pNext        = hHook->free;    // last element points to first
    hHook->pFreeWrite    = pQueue;        // also set freeWrite-pointer in case someone installs a hook with maxpending=0 but still calls omethPacketFree

    return hHook;
}

/*****************************************************************************
*
* omethHookSetFunction - change hook function of existing hook
*
*/
int                omethHookSetFunction
(
 OMETH_HOOK_H    hHook,        /* handle of existing hook                */
 OMETH_HOOK_FCT    *pFct        /* new callback function                */
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
OMETH_FILTER_H    omethFilterCreate
(
 OMETH_HOOK_H    hHook,            /* handle of a ethernet client                */
 void            *arg,            /* argument for hook function                */
 void            *pMask,            /* ptr to array with 17 mask bytes            */
 void            *pValue            /* ptr to array with 17 compare values        */
)
{
    OMETH_FILTER_H    hFilter;
    OMETH_H            hEth;
    int                i;

    assert(hHook);
    if(hHook==0) return 0;

    hEth = hHook->hEth;    // access to driver handle

    // find available filter entry
    // search for free hook structure
    hFilter = hEth->pFilterList;

    for(i=0;i<hEth->nbFilter - hEth->nbFilterX;i++)
    {
        if(hFilter->hHook == 0)    // free filter found
        {
            hFilter->arg                = arg;        // hook argument
            hFilter->hHook                = hHook;    // reference to hook
            hFilter->pFilterData->hEth    = hEth;        // reference to ethernet interface

            omethFilterSetPattern(hFilter, pMask, pValue);    // copy filter mask and values to the filter

            FILTER_SET_FLAG(hFilter->pFilterData, CMD_FILTER_ON);    // enable filter

            hFilter->pFilterData->cmdHigh = 0;
            *(hFilter->pFilterData->pCommand-1) = 0;

            hEth->cntFilterUsed++;

            return hFilter;
        }

        hFilter++;
    }

    assert(0);
    return 0;
}

/*****************************************************************************
*
* omethFilterCreateX - create a x-filter entry for a hook
*
*/
OMETH_FILTER_H    omethFilterCreateX
(
 OMETH_HOOK_H    hHook,            /* handle of a ethernet client                */
 void            *arg,            /* argument for hook function                */
 void            *pMask,            /* ptr to array with 17 mask bytes            */
 void            *pValue            /* ptr to array with 17 compare values        */
)
{
    OMETH_FILTER_H    hFilter;
    OMETH_H            hEth;
    int                i;

    assert(hHook);
    if(hHook==0) return 0;

    hEth = hHook->hEth;    // access to driver handle

    // find available filter entry
    // search for free hook structure
    hFilter = hEth->pFilterList + (hEth->nbFilter - hEth->nbFilterX);
    i       = hEth->nbFilterX;

    while(i--)
    {
        if(hFilter->hHook == 0)    // free filter found
        {
            hFilter->arg                = arg;        // hook argument
            hFilter->hHook                = hHook;    // reference to hook
            hFilter->pFilterData->hEth    = hEth;        // reference to ethernet interface

            omethFilterSetPattern(hFilter, pMask, pValue);    // copy filter mask and values to the filter

            FILTER_SET_FLAG(hFilter->pFilterData, CMD_FILTER_ON);    // enable filter

            hEth->cntFilterXUsed++;

            return hFilter;
        }

        hFilter++;
    }

    assert(hHook);
    return 0;
}

/*****************************************************************************
*
* omethFilterSetPattern - sets a new filter pattern (mask/value) to an existing
*                            filter
*
*/
int                omethFilterSetPattern
(
 OMETH_FILTER_H    hFilter,
 void            *pMask,            /* ptr to array with 17 mask bytes            */
 void            *pValue            /* ptr to array with 17 compare values        */
)
{
    ometh_filter_data_typ    *pFilterData;
    ometh_filter_entry_typ    *pFilterEntry;

    int            i;

    if(hFilter==0 || pMask==0 || pValue==0) return -1;

    pFilterData = hFilter->pFilterData;

    // disable filter
    *pFilterData->pCommand = pFilterData->cmd & ~CMD_FILTER_ON;

    pFilterEntry = pFilterData->pFilterWriteOnly;    // access to filter data

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
* omethFilterSetByteMask - sets one byte (mask, not value) to an filter
*
*    ! the function does not disable the filter while changing the mask
*        ->    if more than 1 byte should be changed consistent omethFilterSetPattern()
*            shall be used instead
*
*/
void            omethFilterSetByteMask
(
 OMETH_FILTER_H    hFilter,    /* filter handle                                    */
 unsigned short    offset,        /* offset in the filterarray                        */
 unsigned char    mask        /* mask to set                                        */
)
{
    hFilter->pFilterData->pFilterWriteOnly[offset].mask = mask;
}

/*****************************************************************************
*
* omethFilterSetArgument - sets a new argument which will be passed to the callback
*
*/
int                omethFilterSetArgument
(
 OMETH_FILTER_H    hFilter,
 void            *arg
)
{
    if(hFilter==0) return -1;    // verify filter
    hFilter->arg = arg;            // overtake new argument
    return 0;
}

/*****************************************************************************
*
* omethFilterSetHook - sets a new hook for the filter
*
*/
int                omethFilterSetHook
(
 OMETH_FILTER_H    hFilter,
 OMETH_HOOK_H    hHook            /* handle from omethHookCreate()        */
)
{
    if(hFilter==0) return -1;    // verify filter
    hFilter->hHook = hHook;        // overtake new hook
    return 0;
}

/*****************************************************************************
*
* omethFilterSetNoMatchIRQ - set/clear irq enable for no_match event on this filter
*
*    If a received frame does not match any of the installed filters, the NoMatch-IRQ
*    will be generated if enabled with this function (can be applied to any of the installed
*    filters)
*
*/
int                omethFilterSetNoMatchIRQ
(
 OMETH_FILTER_H    hFilter,        /* filter handle                            */
 int            irqEnable        /* TRUE: IRQ will be triggerd if frame does not match this filter */
)
{
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
    return 0;
}

/*****************************************************************************
*
* omethFilterSetHubPort - set HUB port to which this filter should react
*
*    After creating a filter the port will be set to -1 (filter reacts to frames from all ports)
*
*/
void                omethFilterSetHubPort
(
 OMETH_FILTER_H    hFilter,        /* filter handle                                    */
 int            port            /* -1 .. react to all ports, 0-x reac only to port x */
)
{
    ometh_filter_data_typ    *pFilterData = hFilter->pFilterData;

    // switch hub selection off
    pFilterData->cmdHigh = pFilterData->cmdHigh & ~(CMD_PORTSELECT_ENABLE + CMD_PORTSELECT_INDEX);    // clear bits for port control

    if(port != -1) pFilterData->cmdHigh = pFilterData->cmdHigh | CMD_PORTSELECT_ENABLE | port;

    *(pFilterData->pCommand-1) = pFilterData->cmdHigh;
}

/*****************************************************************************
*
* omethFilterEnable - enable filter
*
*/
void                omethFilterEnable
(
 OMETH_FILTER_H    hFilter
)
{
    // turn filter entry on
    FILTER_SET_FLAG(hFilter->pFilterData, CMD_FILTER_ON);
}

/*****************************************************************************
*
* omethSetSCNM - set SCNM filter
*
*/
int                omethSetSCNM
(
 OMETH_H        hEth,        /* handle of ethernet driver, see omethCreate() */
 OMETH_FILTER_H    hFilter        /* filter handle                                */
)
{
    if(hEth==0) return -1;

    if(hFilter)
    {
        // verify if filter belongs to this driver instance (and check for invalid handle)
        // if the handle is not valid the queue transmitter is disabled
        if(hFilter == OMETH_INVALID_FILTER || hEth != hFilter->pFilterData->hEth )
        {
            hEth->pFilterSCNM = 0;
            hEth->txQueueEnable = 0;    // queue sending is disabled
            return -2;
        }

        // Auto-TX filter can not be declared as SCNM filter
        if(hFilter->pTxInfo) return -1;

        // return directly if the given filter is already used as SCNM filter
        if(hEth->pFilterSCNM == hFilter->pFilterData) return 0;

        // clear sync flag in old filter if still set
        if(hEth->pFilterSCNM) FILTER_CLEAR_FLAG(hEth->pFilterSCNM, CMD_FILTER_SYNC);

        hEth->pFilterSCNM = hFilter->pFilterData;                // access to new filter

        hEth->pRegBase->txStatus.setBit = OMETH_REG_SYNC;        // set sync flag in control register

        FILTER_SET_FLAG(hEth->pFilterSCNM, CMD_FILTER_SYNC);    // set sync flag in this filter
    }
    else
    {
        // clear sync flag in old filter if still set
        if(hEth->pFilterSCNM) FILTER_CLEAR_FLAG(hEth->pFilterSCNM, CMD_FILTER_SYNC);

        // clear sync flag in control register
        hEth->pRegBase->txStatus.clrBit = OMETH_REG_SYNC;

        hEth->pFilterSCNM = 0;
    }

    hEth->txQueueEnable = 1;    // queue sending is allowed if valid filter is set or if we run in basic ethernet mode

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
    ometh_tx_info_typ    *pInfo,*pNew;

    // start with last buffer
    if(hEth->pTxInfo[1])    // take 2nd tx queue if available
    {
        pInfo = hEth->pTxInfo[1];
    }
    else                    // otherwise take 1st tx queue
    {
        pInfo = hEth->pTxInfo[0];
    }

    // error if no buffers available
    if(pInfo == pInfo->pNext)
    {
        assert(0);
        return 0;
    }

    // find info structure one before the last
    while(1)
    {
        pNew = pInfo->pNext;
        if(pNew->flags1 & FLAGS1_LAST) break;    // last found
        pInfo = pNew;
    }

    pInfo->flags1    = pNew->flags1;    // overtake last-flags
    pInfo->pNext    = pNew->pNext;    // overtake link to next info structure

    pNew->delayTime    = 0;
    pNew->flags1    = 0;            // auto answer buffers have no flags
    pNew->pNext        = pInfo;        // backwards-link of auto answer structures

    return pNew;
}

/*****************************************************************************
*
* omethResponseInitBuf - initialize a installed filter for auto response frames
*                        and provide change buffers
*                        (this avoids that the function returns 0-pointers at the first
*                        1 or 2 calls)
*/
int                omethResponseInitBuf
(
 OMETH_FILTER_H        hFilter,    /* filter handle                            */
 ometh_packet_typ    *pPacket1,    /* spare packet for change buffer            */
 ometh_packet_typ    *pPacket2    /* spare packet for change buffer            */
)
{
    OMETH_H                hEth;
    ometh_tx_info_typ    *pTxInfo;

    if(    hFilter==0 ||            // hFilter invalid
        hFilter->hHook==0 ||    // no hook to this filter, filter not valid
        hFilter->pTxInfo        // already initialized
      )
    {
        assert(0);
        return -1;
    }

    hEth = hFilter->pFilterData->hEth;                        // get driver instance from hook

    if(hEth->pRegBase->txStatus.value & OMETH_REG_RUN)
    {
        assert(0);
        return -1;    // new setup in run mode not allowed
    }

    if(hFilter->pFilterData == hEth->pFilterSCNM)
    {
        assert(0);
        return -1;        // SCNM filter can not be auto-response
    }

    pTxInfo = allocTxDescriptor(hEth);        // allocate tx descriptor

    assert(pTxInfo);
    if(pTxInfo==0) return -1;                // no descriptor available

    hFilter->pTxInfo = pTxInfo;                // this is the next autoresponse descriptor

    // preset spare buffers for exchange
    pTxInfo->pProduced[0] = pPacket1;
    pTxInfo->pProduced[2] = pPacket2;

    // prepare filter command for auto response
    FILTER_SET_FLAG(hFilter->pFilterData, pTxInfo->index | CMD_FILTER_ON);

    hFilter->pFilterData->txEnableRequest=1;    // auto tx flag should be set at next omethResponseSet
    return 0;
}

/*****************************************************************************
*
* omethResponseInit - initialize a installed filter for auto response frames
*
*/
int                omethResponseInit
(
 OMETH_FILTER_H    hFilter        /* filter handle                                */
)
{
    return omethResponseInitBuf(hFilter,0,0);
}

/*****************************************************************************
*
* omethResponseSet - set new packet for response frame
*
*/
ometh_packet_typ    *omethResponseSet
(
 OMETH_FILTER_H        hFilter,    /* filter handle                            */
 ometh_packet_typ    *pPacket    /* packet which shall be responded            */
)
{
    ometh_tx_info_typ    *pInfo;
    ometh_desc_typ        *pDesc;
    unsigned short        len;
    unsigned long        newChgIndex,freeChgIndex;

    if(pPacket==0)            return OMETH_INVALID_PACKET;    // invalid packet
    if(hFilter==0)            return OMETH_INVALID_PACKET;    // hFilter invalid

    pInfo = hFilter->pTxInfo;        // access to assigned tx descriptor

    if(pInfo==0)            return OMETH_INVALID_PACKET;    // response not initialized

    // generate a new change index, different to the old index values of read and write
    freeChgIndex = pInfo->chgIndexWrite;
    newChgIndex = chgIndexTab[freeChgIndex][pInfo->chgIndexRead];

    pDesc = pInfo->pDesc;            // access to hardware

    if(hFilter->pFilterData->len == OMETH_FILTER_LEN)    // normal filter
    {
        // set new packet for auto response
        len = pPacket->length;
        if(len < OMETH_MIN_TX_FRAME) len = OMETH_MIN_TX_FRAME;

        // write length before ptr only if new packet is bigger
        // (if new frame is smaller it could happen that the length is overtaken first and the mac sends
        //  the old ptr with too less bytes)
        if(len > pDesc->len) pDesc->len = len;

        // overtake buffer to descriptor
        pDesc->pData    = (unsigned long)&pPacket->data; // | chgIndexHighBit[newChgIndex];
    }
    else    // x-filter
    {
        // set new packet for auto response
        len = pPacket->length - OMETH_X_OFFSET;
        if(len < OMETH_MIN_TX_FRAME-OMETH_X_OFFSET) len = OMETH_MIN_TX_FRAME-OMETH_X_OFFSET;

        // write length before ptr only if new packet is bigger
        // (if new frame is smaller it could happen that the length is overtaken first and the mac sends
        //  the old ptr with too less bytes)
        if(len > pDesc->len) pDesc->len = len;

        // overtake buffer to descriptor
        pDesc->pData    = (((unsigned long)&pPacket->data)+OMETH_X_OFFSET) | chgIndexHighBit[newChgIndex];
    }

    pDesc->len        = len;

    // decide which buffer can be released
    pInfo->chgIndexRead = pDesc->txStart & 3;    // get current chg index from descriptor

    // if buffer was not overtaken just during the last lines we have to take another buffer to pass back to the user
    if (pInfo->chgIndexRead != newChgIndex) freeChgIndex = chgIndexTab[newChgIndex][pInfo->chgIndexRead];

    pInfo->pProduced[newChgIndex] = pPacket;    // remember the sent packet to free it as soon as possible
    pInfo->chgIndexWrite = newChgIndex;            // the next cycle has to know the change index which was written the last time to generate a different one

    // writing to flags1 is only allowed if owner is not yet set, otherwise collision with tx-irq-access to this field
    if ((pDesc->flags.byte.high & FLAGS1_OWNER) == 0)
    {
        pDesc->txStart            = pInfo->delayTime;
        pDesc->flags.byte.high    = FLAGS1_OWNER | FLAGS1_TX_DELAY;
    }

    // check tx-enable of filter and turn on if not already done
    if(hFilter->pFilterData->txEnableRequest)
    {
        hFilter->pFilterData->txEnableRequest = 0;
        FILTER_SET_FLAG(hFilter->pFilterData, CMD_TX_ENABLE);
    }

    return pInfo->pProduced[freeChgIndex];    // release the buffer which was produced at the last cycle
}

/*****************************************************************************
*
* omethResponseLink - link filter with response buffer of another filter
*
*/
int        omethResponseLink
(
 OMETH_FILTER_H        hFilterDst,        /* handle new filter which should get a response buffer    */
 OMETH_FILTER_H        hFilterSrc        /* response buffer from this filter is used                */
)
{
    if(hFilterSrc==0)                return -1;    // hFilter invalid
    if(hFilterDst==0)                return -1;    // hFilter invalid

    if(hFilterDst->pTxInfo != 0)    return -1;    // destination filter does already have response buffer
    if(hFilterSrc->pTxInfo == 0)    return -1;    // source filter does not have a response buffer

    hFilterDst->pTxInfo = hFilterSrc->pTxInfo;

    // prepare filter command for auto response
    FILTER_SET_FLAG(hFilterDst->pFilterData, hFilterDst->pTxInfo->index);

    return 0;
}


/*****************************************************************************
*
* omethResponseTime - set ticks which are added to the IPG
*
*/
int        omethResponseTime
(
 OMETH_FILTER_H        hFilter,        /* set time value for auto response        */
 unsigned long        ticks            /* delay ticks added to IPG                */
)
{
    ometh_tx_info_typ *pTxInfo;

    if(hFilter==0)        return -1;    // hFilter invalid

    pTxInfo = hFilter->pTxInfo;

    if(pTxInfo == 0)    return -1;    // no tx-info installed, not an auto-response filter ?

    pTxInfo->delayTime        = ticks;
    pTxInfo->pDesc->txStart    = ticks;

    return 0;
}

/*****************************************************************************
*
* omethResponseDisable - disable auto response frame
*
*/
int                    omethResponseDisable
(
 OMETH_FILTER_H        hFilter        /* filter handle                */
)
{
    if(hFilter==0) return -1;    // hFilter invalid

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
int                    omethResponseEnable
(
 OMETH_FILTER_H        hFilter        /* filter handle                */
)
{
    ometh_desc_typ        *pDesc;

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
* omethResponseCount - returns the number of autoresonse-frames which were
*                        sent for this filter
*/
unsigned long        omethResponseCount
(
 OMETH_FILTER_H        hFilter        /* filter handle                            */
)
{
    if(hFilter==0 || hFilter->pTxInfo==0) return 0;

    return hFilter->pTxInfo->autoTxCount;
}


/*****************************************************************************
*
* omethResponseCountReset - reset number of autoresonse-frames which were
*                            sent for this filter
*/
void                omethResponseCountReset
(
 OMETH_FILTER_H    hFilter        /* filter handle */
)
{
    if(hFilter==0 || hFilter->pTxInfo==0) return;

    hFilter->pTxInfo->autoTxCount = 0;
}


/*****************************************************************************
*
* omethTransmit - transmit a buffer to the network
*
*/
unsigned long        omethTransmit
(
 OMETH_H            hEth,        /* handle of ethernet driver, see omethCreate() */
 ometh_packet_typ    *pPacket,    /* packet to be sent                            */
 OMETH_BUF_FREE_FCT    *pFct        /* function ptr to sent-ack-function            */
)
{
    OMETH_TRANSMIT( 0 , 0 , 0 , 0);        // add frame to send queue
}

/*****************************************************************************
*
* omethTransmitArg - transmit a buffer to the network with argument for tx-callback
*
*/
unsigned long        omethTransmitArg
(
 OMETH_H                hEth,        /* handle of ethernet driver, see omethCreate() */
 ometh_packet_typ        *pPacket,    /* packet to be sent                            */
 OMETH_BUF_FREE_FCT_ARG    *pFct,        /* function ptr to sent-ack-function            */
 void                    *arg        /* argument which will be passed to free function */
)
{
    OMETH_TRANSMIT( arg, 0, 0 , 0);        // add frame to send queue
}

/*****************************************************************************
*
* omethTransmitArg2 - Transmit with 2nd transmit queue (everything else same as omethTransmit)
*
*/
unsigned long        omethTransmitArg2
(
 OMETH_H                hEth,        /* handle of ethernet driver, see omethCreate() */
 ometh_packet_typ        *pPacket,    /* packet to be sent                            */
 OMETH_BUF_FREE_FCT_ARG    *pFct,        /* function ptr to sent-ack-function            */
 void                    *arg        /* argument which will be passed to free function */
)
{
    OMETH_TRANSMIT( arg, 0, 0 , 1);        // add frame to send queue
}

/*****************************************************************************
*
* omethTransmitTime - transmit a buffer to the network at defined time
*    (same like omethTransmitArg, just the optional argument is additional)
*
*/
unsigned long        omethTransmitTime
(
 OMETH_H                hEth,        /* handle of ethernet driver, see omethCreate() */
 ometh_packet_typ        *pPacket,    /* packet to be sent                            */
 OMETH_BUF_FREE_FCT_ARG    *pFct,        /* function ptr to sent-ack-function            */
 void                    *arg,        /* argument which will be passed to free function */
 unsigned long            time        /* timestamp                                    */
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
unsigned char        omethTransmitPending
(
 OMETH_H                hEth        /* handle of ethernet driver, see omethCreate() */
)
{
    return hEth->cntTxQueueIn - hEth->cntTxQueueOut;
}

/*****************************************************************************
*
* omethStart - start ethernet driver
*
*/
void            omethStart
(
 OMETH_H        hEth,                /* handle of ethernet driver, see omethCreate() */
 int            bClearPendingIrqs    /* TRUE: clear pending irq's */
)
{
    unsigned short setBit;

    if(hEth==0) return;

    if (hEth->clearPendingIrqAtStart || bClearPendingIrqs)
    {
        hEth->clearPendingIrqAtStart=0;    // important for debugging .. clear pending irqs from last activation

        // quit pending irqs and clear lost-bit
        while(hEth->pRegBase->rxStatus.value & OMETH_REG_PENDING) hEth->pRegBase->rxStatus.clrBit = OMETH_REG_IQUIT;
        while(hEth->pRegBase->txStatus.value & OMETH_REG_PENDING) hEth->pRegBase->txStatus.clrBit = OMETH_REG_IQUIT;

        hEth->pRxNext = &hEth->pRxInfo[hEth->pRegBase->rxStatus.value & 0x0f]; // set pRxNext descriptor info
    }

    hEth->pRegBase->rxStatus.clrBit = OMETH_REG_LOST;

    setBit = OMETH_REG_RUN | OMETH_REG_IE;

    hEth->pRegBase->txStatus.setBit = setBit;

    if(hEth->config.mode & OMETH_MODE_CRC_DETECT) setBit = setBit | OMETH_REG_DIAG;

    hEth->pRegBase->rxStatus.setBit = setBit;
}

/*****************************************************************************
*
* omethStop - stop ethernet driver
*
*/
void            omethStop
(
 OMETH_H        hEth        /* handle of ethernet driver, see omethCreate() */
)
{
    if(hEth==0) return;

    // clear run bits for rx and tx
    hEth->pRegBase->rxStatus.clrBit = OMETH_REG_RUN | OMETH_REG_IE;
    hEth->pRegBase->txStatus.clrBit = OMETH_REG_RUN | OMETH_REG_IE;
}


/*****************************************************************************
*
* omethStatistics - get ptr to statistics of ethernet adapter
*
*/
ometh_stat_typ    *omethStatistics
(
 OMETH_H        hEth        /* handle of ethernet driver, see omethCreate() */
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
    *    !! this function does not work on all MAC designs !!
    *
    */
    void        omethSoftwareIrqSet
    (
    OMETH_H            hEth,        /* handle of ethernet driver, see omethCreate() */
    OMETH_BUF_FREE_FCT    *pFct        /* function ptr to soft-irq function            */
    )
    {
        if(hEth) hEth->pFctSoftIrq = pFct;
    }

    void        omethSoftwareIrqStart
    (
    OMETH_H            hEth        /* handle of ethernet driver, see omethCreate() */
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
    void    omethSetRxHandshake
    (
    OMETH_H        hEth,        /* handle of ethernet driver, see omethCreate() */
    unsigned short    mode
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
* omethNoFilterMatchIrqHandler - no filter match irq handler
*
*    Calls the function OMETH_NOFILTERMATCHIRQ_HOOK_FCT() which must be defined
*    in ometh_target.h if a received frame does not match any filter and the
*    NoMatch-IRQ is enabled.
*
*/
void            omethNoFilterMatchIrqHandler
(
 OMETH_H        hEth        /* handle of ethernet driver, see omethCreate() */
)
{
    if ((hEth->pRegBase->rxStatus.value & OMETH_REG_RX_NOMATCH) != OMETH_REG_RX_NOMATCH) return;    // no irq

    #ifdef OMETH_NOFILTERMATCHIRQ_HOOK_FCT
        if (OMETH_NOFILTERMATCHIRQ_HOOK_P != 0)    OMETH_NOFILTERMATCHIRQ_HOOK_FCT((void*)hEth->pRxNext->pDesc->pData);
    #endif

    hEth->pRegBase->rxStatus.clrBit = OMETH_REG_RX_NOMATCH;    // clear pending irq
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
int                omethDestroy
(
 OMETH_H        hEth        /* handle of ethernet driver, see omethCreate() */
)
{
    OMETH_H            hFind = omethInternal.pFirstEth;
    OMETH_HOOK_H    hHook,hFree;

    if(hEth==0) return -1;

    // find driver instance in list and remove
    if(hEth==hFind)    // first instance should be removed
    {
        omethInternal.pFirstEth = hEth->pNext;
    }
    else if (hFind)
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

    omethStop(hEth);    // stop ethernet driver to disable IRQs

    // reset ptr to current periodic instance to reset periodic phy control
    // (periodic phy control could be in a undefined state when removing the currently active instance)
    // (at the next call it will restart automatically)
    omethInternal.pPeriodicEth = 0;

    // release all resources of this instance

    // free all installed hooks
    hHook = hEth->pHookList;
    while(hHook)
    {

        //freePtr(hHook->pRxBufBase);    // free frame buffers allocated from the hooks
        //------------------------------------------------------------------------------
        if(hEth->config.pktLoc == OMETH_PKT_LOC_MACINT)
        {
            //nothing to do here...
        }
        else if(hEth->config.pktLoc == OMETH_PKT_LOC_HEAP)
        {
            //frame buffer pool is in heap, so free it...
            OMETH_UNCACHED_FREE(hHook->pRxBufBase);    // free frame buffers allocated from the hooks
        }
        else
        {
            //error
            return -1;
        }
        //------------------------------------------------------------------------------

        hFree = hHook;            // store handle to free it after overtakeing the next-pointer
        hHook = hHook->pNext;    // get next hook handle

        freePtr(hFree);
    }

    // free filter list and attached data structures
    if(hEth->pFilterList)    freePtr(hEth->pFilterList->pFilterData);

    freePtr(hEth->pFilterList);
    freePtr(hEth->pPhyReg);        // free phy register image
    if(hEth->config.pktLoc == OMETH_PKT_LOC_HEAP)
    {
        OMETH_UNCACHED_FREE(hEth->pRxBufBase);    // free allocated rx-buffers
    }
    freePtr(hEth->pRxInfo);        // free rx/tx info list
    freePtr(hEth->pTxInfo[0]);
    freePtr(hEth->pTxInfo[1]);
    freePtr(hEth);                // free instance

    return 0;
}

/*****************************************************************************
*
* omethRxIrqHandlerMux - to be called from the MAC RX interrupt (multiplexer for all ethernet interfaces)
*
*/
void            omethRxIrqHandlerMux
(
 void
 )
{
    OMETH_H        hEth = omethInternal.pFirstEth;

    // call irq handler for all ethernet adapters
    while(hEth)
    {
        omethRxIrqHandler(hEth);
        hEth=hEth->pNext;
    }
}

/*****************************************************************************
*
* omethTxIrqHandlerMux - to be called from the MAC TX interrupt (multiplexer for all ethernet interfaces)
*
*/
void            omethTxIrqHandlerMux
(
 void
 )
{
    OMETH_H        hEth = omethInternal.pFirstEth;

    // call irq handler for all ethernet adapters
    while(hEth)
    {
        omethTxIrqHandler(hEth);
        hEth=hEth->pNext;
    }
}

/*****************************************************************************
*
* omethRxTxIrqHandlerMux - to be called from the IRQ if all MACs on the system
*                          use the same IRQ for Rx and Tx
*
* RETURN: -
*
*/
void            omethRxTxIrqHandlerMux
(
 void
)
{
    OMETH_H            hEth = omethInternal.pFirstEth;
    OMETH_H            hEthProcess=0;
    ometh_reg_typ    *pRegBase;

    unsigned short    pending=0 , maxPending=0;

    // search for the irq source with the highest pending counter
    while(hEth)
    {
        pRegBase = hEth->pRegBase;

        pending = pRegBase->rxStatus.value & OMETH_REG_PENDING;
        if(pending > maxPending)
        {
            hEthProcess = hEth;
            maxPending = pending;
        }

        pending = pRegBase->txStatus.value & OMETH_REG_PENDING;
        if(pending > maxPending)
        {
            hEthProcess = (OMETH_H)((size_t)hEth | 1);    // use bit 0 to mark that this is a tx-IRQ
            maxPending = pending;
        }

        hEth=hEth->pNext;
    }

    if(hEthProcess==0)    // no pending irqs found ... suspicious
    {
        assert(0);
        return;
    }

    // call the respective IRQ handler
    if((size_t)hEthProcess & 1)    omethTxIrqHandler((OMETH_H)((size_t)hEthProcess ^ 1));
    else                        omethRxIrqHandler(hEthProcess);
}
