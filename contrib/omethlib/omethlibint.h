/* omethlibint.h - Ethernet Library for FPGA MAC Controller */
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
 Module:    omethlibint
 File:      omethlibint.h
 Author:    Thomas Enzinger(enzingert)
 Created:   05.10.2006
 Revised:   25.01.2010
 State:     tested on Altera Nios II and Xilinx Microblaze
------------------------------------------------------------------------------

 Functions:
    GET_TYPE_BASE
    FILTER_SET_FLAG
    FILTER_CLEAR_FLAG

------------------------------------------------------------------------------
 History:
    see omethlibint.c

----------------------------------------------------------------------------*/


#ifndef __OMETHLIBINT_H__
#define __OMETHLIBINT_H__

#include <omethlib_target.h>    // target specific defines (BIG/LITTLE endian)

//-------------------- filter entry type (for 1 byte) --------------------
typedef struct
{
    unsigned char value;
    unsigned char mask;
}ometh_filter_entry_typ;

//-------------------- filter type --------------------
#define OMETH_FILTER_LEN    31
#define OMETH_XFILTER_LEN    17    // x-filters have only 17 entries !
#define OMETH_X_OFFSET        24    // frame offset for x-response

typedef struct
{
    ometh_filter_entry_typ    b[OMETH_FILTER_LEN];    // 31 filter entries
    unsigned char            command;    // command register
    unsigned char            commandHigh;
}ometh_filter_typ;

typedef struct
{
    ometh_filter_entry_typ    b[OMETH_XFILTER_LEN];    // 17 filter entries
    unsigned char            command;    // command register
    unsigned char            reserve;
}ometh_xfilter_typ;

#define CMD_FILTER_SYNC    0x10    // this is a sync filter
#define CMD_RX_NOMATCH     0x20    // if no filter match -> set nomatch irq
#define CMD_FILTER_ON      0x40    // filter-on flag in command register
#define CMD_FILTER_nON     0xBF    // filter-on flag in command register (~ convert)
#define CMD_TX_ENABLE      0x80    // enables auto transmitting of a tx descriptor

#define CMD_PORTSELECT_ENABLE    0x08    // flag to enable port select (in high byte of filter command)
#define CMD_PORTSELECT_INDEX     0x07    // mask where hub port is stored (in high byte of filter command)

//-------------------- rx/tx descriptor --------------------
typedef struct
{
    unsigned char    low;
    unsigned char    high;
}ometh_desc_flags_bytes;

typedef union    // descriptor status
{
    ometh_desc_flags_bytes    byte;
    unsigned short            word;
}ometh_desc_flags_union;

typedef struct
{
    ometh_desc_flags_union    flags;
    unsigned short            len;        // number of bytes
    unsigned long    pData;        // ptr to data
    unsigned long    txStart;    // tx start time
    unsigned long    time;        // time stamp
}ometh_desc_typ;

#define FLAGS1_OWNER           0x01        // 1 if mac is the owner of the descriptor
#define FLAGS1_LAST            0x02        // last-bit
#define FLAGS1_SENT            0x04        // bit will be set if mac has sent a frame
#define FLAGS1_TX_DELAY        0x10        // auto-transmit will be delayed by the number of ticks in txStart
#define FLAGS1_TX_BEG          0x20        // irq will be sent at given start time in descriptor
#define FLAGS1_START_TIME      0x40        // frame will be sent at given start time in descriptor

#define FLAGS2_ALIGNMENT       0x10        // frame has alignment error

//-------------------- hardware type 01 --------------------
// Typ 01 has 16 filters, 16 rx descriptors and 16 tx descriptors
#define MAC_HW_TYP_01_NB_FILTER        16
#define MAC_HW_TYP_01_NB_RXDESC        16
#define MAC_HW_TYP_01_NB_TXDESC        16

//-------------------- register flags ------------------------
#define OMETH_REG_IE              0x8000        // irq enable
#define OMETH_REG_SOFTIRQ         0x4000        // software IRQ
#define OMETH_REG_HALF            0x2000        // hub enable (tx-control)

#define OMETH_REG_10MBIT         0x2000        // 10MBit (same bit as OMETH_REG_XNODE_1 because these 2 features can not exist at the same time)
#define OMETH_REG_XNODE_1        0x2000        // x-node enable (rx-control) filter 14,15
#define OMETH_REG_XNODE_2        0x4000        // x-node enable (rx-control) filter 12,13
#define OMETH_REG_XNODE          (OMETH_REG_XNODE_1 | OMETH_REG_XNODE_2)
#define OMETH_REG_SYNC           0x1000        // set sync mode
#define OMETH_REG_DIAG           0x1000        // enable diag (frames < 31 byte also generate IRQ)
#define OMETH_REG_PENDING        0x0F00        // mask for pending field
#define OMETH_REG_IQUIT          0x0100        // quit interrupt (only valid on clear port)
#define OMETH_REG_RUN            0x0080        // run bit
#define OMETH_REG_TX_BEG         0x0040        // quit tx beg irq
#define OMETH_REG_RX_NOMATCH     0x0040        // quit interrupt -> nomatch
#define OMETH_REG_LOST           0x0010        // lost flag (rx-control)
#define OMETH_REG_RXIDLE         0x0020        // rx-idle flag
#define OMETH_REG_SET_RES_IPG    0x4000        // flag to set response inter package gap

typedef struct
{
    ometh_filter_typ      filter[MAC_HW_TYP_01_NB_FILTER];
    ometh_desc_typ        rxDesc[MAC_HW_TYP_01_NB_RXDESC];
    ometh_desc_typ        txDesc[MAC_HW_TYP_01_NB_TXDESC];
}ometh_hw_typ1;

// additional info about pending frames
typedef struct ometh_pending_typ
{
    ometh_buf_typ                *pBuf;     // ptr to buffer
    struct ometh_pending_typ     *pNext;    // ptr to next pending
}ometh_pending_typ;

//-------------------- client hook --------------------
struct OMETH_HOOK
{
    OMETH_HOOK_FCT     *pFct;            // function ptr to hook
    unsigned char      maxPending;        // number of max pending buffers
    unsigned short     cntOverflow;    // statistics: buffer overflow

    OMETH_H              hEth;        // handle to ethernet instance
    ometh_buf_typ        *pRxBufBase;    // (only used for destroy function)
    struct OMETH_HOOK    *pNext;            // ptr to next hook (only used for destroy function)
    ometh_pending_typ    *pFreeRead;    // ptr to get next free ptr in the list
    ometh_pending_typ    *pFreeWrite;// ptr to release next buffer
    ometh_pending_typ    free[1];    // list with free pointers
};

// additional info about tx descriptors
typedef struct ometh_tx_info_typ
{
    unsigned char                flags1;            // flags for the tx descriptor
    unsigned char                index;            // index (required for filter configuration)
    unsigned char                chgIndexWrite;    // last write index of change-buffer-system (auto response buffers)
    unsigned char                chgIndexRead;    // last read index of change-buffer-system (auto response buffers)
    unsigned long                autoTxCount;    // counts auto-tx events on this descriptor
    unsigned long                delayTime;        // delay time for auto tx
    ometh_desc_typ               *pDesc;            // ptr to tx descriptor
    OMETH_BUF_FREE_FCT_ARG       *pFctFree;        // ptr to free function for this buffer (ptr to filter cmd for auto response descriptors)
    void                         *fctFreeArg;    // argument passed to free function after packet pointer
    ometh_packet_typ             *pProduced[3];    // remember the produced frames for auto-response buffers here
    struct ometh_tx_info_typ     *pNext;            // ptr to next tx info
}ometh_tx_info_typ;

// additional info about rx descriptors
typedef struct ometh_rx_info_typ
{
    unsigned char                flags1;        // flags for the rx descriptor
    ometh_desc_typ               *pDesc;        // ptr to rx descriptor
    struct ometh_rx_info_typ     *pNext;        // ptr to next rx descriptor
}ometh_rx_info_typ;

// reference to filter hardware ports
typedef struct ometh_filter_data_typ
{
    unsigned char            cmd;                // current filter command
    unsigned char            cmdHigh;            // high byte of filter command
    unsigned char            txEnableRequest;    // auto response should be enabled at next omethResponseSet
    unsigned char            len;                // filter length

    unsigned char             *pCommand;            // ptr to filter command (can be different for different filters)
    ometh_filter_entry_typ    *pFilterWriteOnly;    // ptr to filter data (write only area)
    OMETH_H                   hEth;                // handle to ethernet instance
}ometh_filter_data_typ;

//-------------------- filter handle (must be 32 byte for efficient access in IRQ) --------------
struct OMETH_FILTER
{
    OMETH_HOOK_H             hHook;            // ptr to used hook structure
    void                     *arg;            // argument for hook
    ometh_tx_info_typ        *pTxInfo;        // ptr to tx descriptor which holds the auto-answer for this filter
    ometh_filter_data_typ    *pFilterData;    // ptr to filter hardware
};

typedef struct
{
    unsigned short    value;            // read/write port
    unsigned short    setBit;            // set single bits in status register
    unsigned short    clrBit;            // clear single bits in status register
    unsigned short    setDescriptor;    // set descriptor index
}ometh_status_typ;                    // mac rx/tx status registers

typedef struct
{
    ometh_status_typ txStatus;    // tx control register
    ometh_status_typ rxStatus;    // rx control register
} volatile ometh_reg_typ;                    // mac control registers

//-------------------- instance of ethernet driver --------------------
struct OMETH_TYP
{
    unsigned short       rxLen;            // data length of rx buffers
    unsigned char        nbFilter;        // number of filters
    unsigned char        nbFilterX;        // number of filters for x-node functionality
    unsigned char        txQueueEnable;    // will be set to 0 if upper layer switches queue sending off

    unsigned char        cntTxQueueIn,cntTxQueueOut;        // counter to evaluate the number of pending tx descriptors
    unsigned char        cntFilterUsed,cntFilterXUsed;

    ometh_reg_typ        *pRegBase;        // control register base adr

    struct OMETH_FILTER  *pFilterList;    // ptr to filter table (list of handles)

    ometh_rx_info_typ    *pRxNext;    // ptr to next rx info

    ometh_stat_typ        stat;

    ometh_filter_data_typ    *pFilterSCNM;    // ptr to filter for SCNM Mode

    ometh_tx_info_typ    *pTxInfo[2];    // ptr to first tx info
    ometh_tx_info_typ    *pTxNext[2];    // ptr to next tx info
    ometh_tx_info_typ    *pTxFree[2];    // ptr to next release buffer

    ometh_tx_info_typ    *pTxAuto;        // first auto tx descriptor

    ometh_config_typ     config;            // copy of config structure from omethCreate

    unsigned char        phyCount;        // number of phy's on this mac (max 8)
    unsigned char        phyLinkCount;    // number of linked phys
    unsigned char        phyLinkActive;
    unsigned char        phyHalfCount;    // number of phys linked with half duplex
    unsigned char        phyHalfMax;
    unsigned char        phyOffline;

    unsigned char        phyAdr[OMETH_MAX_PHY_CNT];        // array with phy addresses on MII interface
    unsigned short       phyCmdRead[OMETH_MAX_PHY_CNT];    // phy write commands
    unsigned short       phyCmdWrite[OMETH_MAX_PHY_CNT];    // phy read commands

    phy_reg_typ          *pPhyReg;    // ptr to all phy register sets
    unsigned short       phyPort;    // current port
    unsigned short       phyReg;        // current register

    unsigned short       linkSpeed;    // 0/10/100 (initialized with 100, will be reduced to 10 when the first 10MBit device is detected)
    unsigned short       r4Init;        // initial value of r4

    unsigned short       txVal;        // tx value for phy write
    unsigned short       txPort;        // tx port for phy write
    unsigned char        txReg;        // tx register for phy write

    unsigned char        clearPendingIrqAtStart;    // will be set to 1 only for the first start

    // hardware access for debug
    union
    {
        ometh_hw_typ1    typ1;        // typ 1 hardware
    }*pHardware;

//------------- MAC-internal RX/TX buffer base addresses -------------
    unsigned char       *pRxBufBase;
    unsigned char       *pTxBufBase;
//--------------------------------------------------------------------
    struct OMETH_HOOK    *pHookList;        // (only used for destroy function)

    ometh_rx_info_typ    *pRxInfo;        // ptr to first rx info

#if (OMETH_ENABLE_SOFT_IRQ==1)
    OMETH_BUF_FREE_FCT   *pFctSoftIrq;    // function will be called if soft-IRQ was triggered (Tx-IRQ Level)
#endif

    struct OMETH_TYP     *pNext;            // ptr to next driver instance
};

//*************************************************************************************
//
// Get the base address of a structure where only the address of a sub-element is known
//
//    typ        : type name of the regarding struct (must be done with typedef)
//    element    : name of the structure element from which we know the address
//    ptr        : address of the structure element
//
//    result     : structure pointer to the base of the type
//
//  example:
//        ometh_buf_typ    *pBuf;        // ptr to buffer
//        // 'pPacket' is the address to the element 'packet' in the base type 'ometh_buf_typ'
//        // the macro will return the base ptr
//        pBuf = GET_TYPE_BASE( ometh_buf_typ, packet, pPacket);
//
//*************************************************************************************
#define GET_TYPE_BASE(typ, element, ptr)    \
    ((typ*)( ((size_t)ptr) - (size_t)&((typ*)0)->element ))

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

#endif
