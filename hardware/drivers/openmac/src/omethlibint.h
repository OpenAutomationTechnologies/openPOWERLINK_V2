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
    uint8_t value;
    uint8_t mask;
}ometh_filter_entry_typ;

//-------------------- filter type --------------------
#define OMETH_FILTER_LEN    31
#define OMETH_XFILTER_LEN    17    // x-filters have only 17 entries !
#define OMETH_X_OFFSET        24    // frame offset for x-response

typedef struct
{
    ometh_filter_entry_typ    b[OMETH_FILTER_LEN];    // 31 filter entries
    uint8_t                   command;    // command register
    uint8_t                   commandHigh;
}ometh_filter_typ;

typedef struct
{
    ometh_filter_entry_typ    b[OMETH_XFILTER_LEN];    // 17 filter entries
    uint8_t                   command;    // command register
    uint8_t                   reserve;
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
    uint8_t    low;
    uint8_t    high;
}ometh_desc_flags_bytes;

typedef union    // descriptor status
{
    ometh_desc_flags_bytes    byte;
    uint16_t                  word;
}ometh_desc_flags_union;

typedef struct
{
    ometh_desc_flags_union    flags;
    uint16_t                  len;        // number of bytes
    uint32_t                  pData;      // ptr to data
    uint32_t                  txStart;    // tx start time
    uint32_t                  time;       // time stamp
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

//-------------------- hardware type 02 --------------------
// Typ 02 has 16 filters, 32 rx descriptors and 32 tx descriptors
#define MAC_HW_TYP_02_NB_FILTER        16
#define MAC_HW_TYP_02_NB_RXDESC        32
#define MAC_HW_TYP_02_NB_TXDESC        32

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
#define OMETH_REG_LOST           0x0040        // lost flag (rx-control)
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
    uint8_t             maxPending;      // number of max pending buffers
    uint16_t            cntOverflow;     // statistics: buffer overflow

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
    uint8_t                       flags1;        // flags for the tx descriptor
    uint8_t                       index;         // index (required for filter configuration)
    uint8_t                       chgIndexWrite; // last write index of change-buffer-system (auto response buffers)
    uint8_t                       chgIndexRead;  // last read index of change-buffer-system (auto response buffers)
    uint32_t                      autoTxCount;   // counts auto-tx events on this descriptor
    uint32_t                      delayTime;     // delay time for auto tx
    ometh_desc_typ               *pDesc;            // ptr to tx descriptor
    OMETH_BUF_FREE_FCT_ARG       *pFctFree;        // ptr to free function for this buffer (ptr to filter cmd for auto response descriptors)
    void                         *fctFreeArg;    // argument passed to free function after packet pointer
    ometh_packet_typ             *pProduced[3];    // remember the produced frames for auto-response buffers here
    struct ometh_tx_info_typ     *pNext;            // ptr to next tx info
}ometh_tx_info_typ;

// additional info about rx descriptors
typedef struct ometh_rx_info_typ
{
    uint8_t                       flags1;       // flags for the rx descriptor
    ometh_desc_typ               *pDesc;        // ptr to rx descriptor
    struct ometh_rx_info_typ     *pNext;        // ptr to next rx descriptor
}ometh_rx_info_typ;

// reference to filter hardware ports
typedef struct ometh_filter_data_typ
{
    uint8_t                   cmd;                 // current filter command
    uint8_t                   cmdHigh;             // high byte of filter command
    uint8_t                   txEnableRequest;     // auto response should be enabled at next omethResponseSet
    uint8_t                   len;                 // filter length

    uint8_t                  *pCommand;            // ptr to filter command (can be different for different filters)
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
    uint16_t    value;            // read/write port
    uint16_t    setBit;           // set single bits in status register
    uint16_t    clrBit;           // clear single bits in status register
    uint16_t    setDescriptor;    // set descriptor index
}ometh_status_typ;                    // mac rx/tx status registers

typedef struct
{
    ometh_status_typ txStatus;    // tx control register
    ometh_status_typ rxStatus;    // rx control register
} volatile ometh_reg_typ;                    // mac control registers

//-------------------- instance of ethernet driver --------------------
struct OMETH_TYP
{
    uint16_t              rxLen;           // data length of rx buffers
    uint8_t               nbFilter;        // number of filters
    uint8_t               nbFilterX;       // number of filters for x-node functionality
    uint8_t               nbTxDesc;
    uint8_t               nbRxDesc;
    uint8_t               txQueueEnable;   // will be set to 0 if upper layer switches queue sending off

    uint8_t               cntTxQueueIn,cntTxQueueOut;       // counter to evaluate the number of pending tx descriptors
    uint8_t               cntFilterUsed,cntFilterXUsed;

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

    uint8_t              phyCount;        // number of phy's on this mac (max 8)
    uint8_t              phyLinkCount;    // number of linked phys
    uint8_t              phyLinkActive;
    uint8_t              phyHalfCount;    // number of phys linked with half duplex
    uint8_t              phyHalfMax;
    uint8_t              phyOffline;

    uint8_t              phyAdr[OMETH_MAX_PHY_CNT];        // array with phy addresses on MII interface
    uint16_t             phyCmdRead[OMETH_MAX_PHY_CNT];    // phy write commands
    uint16_t             phyCmdWrite[OMETH_MAX_PHY_CNT];    // phy read commands

    phy_reg_typ          *pPhyReg;    // ptr to all phy register sets
    uint16_t             phyPort;      // current port
    uint16_t             phyReg;       // current register

    uint16_t             linkSpeed;    // 0/10/100 (initialized with 100, will be reduced to 10 when the first 10MBit device is detected)
    uint16_t             r4Init;       // initial value of r4

    uint16_t             txVal;        // tx value for phy write
    uint16_t             txPort;       // tx port for phy write
    uint8_t              txReg;        // tx register for phy write

    uint8_t              clearPendingIrqAtStart;    // will be set to 1 only for the first start

    // hardware access for debug
    union
    {
        ometh_hw_typ1    typ1;        // typ 1 hardware
    }*pHardware;

//------------- MAC-internal RX/TX buffer base addresses -------------
    uint8_t              *pRxBufBase;
    uint8_t              *pTxBufBase;
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
#define FILTER_SET_FLAG(pFilter, flag) ometh_wr_8(pFilter->pCommand, (pFilter->cmd |= flag))

/*****************************************************************************
*
* filter_clear_flag - clear command flag of packet filter
*
* RETURN: -
*
*/
#define FILTER_CLEAR_FLAG(pFilter, flag) ometh_wr_8(pFilter->pCommand, (pFilter->cmd &= ~flag))

#endif
