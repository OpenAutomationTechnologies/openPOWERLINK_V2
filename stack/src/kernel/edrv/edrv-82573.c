/**
********************************************************************************
\file   edrv-82573.c

\brief  Implementation of Ethernet driver for Intel 82573

This file contains the implementation of the Ethernet driver for
Intel 82573 Gigabit Ethernet Controller and compatible devices.

Buffer handling:
All buffers are created statically (i.e. at compile time resp. at
initialisation via kmalloc() ) and not dynamically on request (i.e. via
edrv_allocTxBuffer().
edrv_allocTxBuffer() searches for an unused buffer which is large enough.
edrv_init() may allocate some buffers with sizes less than maximum frame
size (i.e. 1514 bytes), e.g. for SoC, SoA, StatusResponse, IdentResponse,
NMT requests / commands. The less the size of the buffer the less the
number of the buffer.

\ingroup module_edrv
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2013, SYSTEC electronic GmbH
Copyright (c) 2017, B&R Industrial Automation GmbH
Copyright (c) 2018, Kalycito Infotech Private Limited
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the copyright holders nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
------------------------------------------------------------------------------*/

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <common/oplkinc.h>
#include <common/ami.h>
#include <common/bufalloc.h>
#include <kernel/edrv.h>

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/pci.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/major.h>
#include <linux/version.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/atomic.h>
#include <asm/irq.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/gfp.h>

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 26))
#include <linux/semaphore.h>
#endif

//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 19))
#error "Linux Kernel versions older 2.6.19 are not supported by this driver!"
#endif

//------------------------------------------------------------------------------
// module global vars
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// global function prototypes
//------------------------------------------------------------------------------

//============================================================================//
//            P R I V A T E   D E F I N I T I O N S                           //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#ifndef EDRV_MAX_TX_BUFFERS
#define EDRV_MAX_TX_BUFFERS     256
#endif

#ifndef EDRV_MAX_TX_DESCS
#define EDRV_MAX_TX_DESCS       16
#define EDRV_TX_DESC_MASK       (EDRV_MAX_TX_DESCS - 1)
#endif

#ifndef EDRV_MAX_RX_BUFFERS
#define EDRV_MAX_RX_BUFFERS     256
#endif

#ifndef EDRV_MAX_RX_DESCS
#define EDRV_MAX_RX_DESCS       16
#define EDRV_RX_DESC_MASK       (EDRV_MAX_RX_DESCS - 1)
#endif

#define EDRV_MAX_FRAME_SIZE     0x600

#define EDRV_TX_BUFFER_SIZE     (EDRV_MAX_TX_BUFFERS * EDRV_MAX_FRAME_SIZE) // n * (MTU + 14 + 4)
#define EDRV_TX_DESCS_SIZE      (EDRV_MAX_TX_DESCS * sizeof(tEdrvTxDesc))

#define EDRV_RX_BUFFER_SIZE_SHIFT   11  // 2048 Byte
#define EDRV_RX_BUFFER_SIZE         (1 << EDRV_RX_BUFFER_SIZE_SHIFT)
#define EDRV_RX_DESCS_SIZE          (EDRV_MAX_RX_DESCS * sizeof(tEdrvRxDesc))

#define EDRV_AUTO_READ_DONE_TIMEOUT 10  // ms
#define EDRV_MASTER_DISABLE_TIMEOUT 90  // ms
#define EDRV_LINK_UP_TIMEOUT        3000 // ms


#define DRV_NAME                "plk"

// Tx descriptor definitions
#define EDRV_TX_DESC_CMD_RS     0x08000000  // Report Status
#define EDRV_TX_DESC_CMD_IFCS   0x02000000  // Insert Frame Check Sum
#define EDRV_TX_DESC_CMD_EOP    0x01000000  // End of Packet
#define EDRV_TX_DESC_CMD_DEF    (EDRV_TX_DESC_CMD_EOP | \
                                 EDRV_TX_DESC_CMD_IFCS | \
                                 EDRV_TX_DESC_CMD_RS)
#define EDRV_TX_DESC_STATUS_DD  0x01        // Descriptor Done
#define EDRV_TX_DESC_STATUS_EC  0x02        // Excess Collisions
#define EDRV_TX_DESC_STATUS_LC  0x04        // Late Collision


// Ethernet Controller Register Definitions
#define EDRV_REGDW_CTRL         0x00000     // Device Control
#define EDRV_REGDW_CTRL_FD      0x00000001  // Full-Duplex
#define EDRV_REGDW_CTRL_MST_DIS 0x00000004  // GIO Master Disable
#define EDRV_REGDW_CTRL_LRST    0x00000008  // Link Reset
#define EDRV_REGDW_CTRL_SLU     0x00000040  // Set Link Up
#define EDRV_REGDW_CTRL_RST     0x04000000  // Reset
#define EDRV_REGDW_CTRL_PHY_RST 0x80000000  // PHY Reset

#define EDRV_REGDW_CTRL_DEF     (EDRV_REGDW_CTRL_LRST | \
                                 EDRV_REGDW_CTRL_SLU)

#define EDRV_REGDW_STATUS       0x00008     // Device Status
#define EDRV_REGDW_STATUS_LU    0x00000002  // Link Up Indication
#define EDRV_REGDW_STATUS_MST_EN 0x00080000 // GIO Master Enable Status

#define EDRV_REGDW_EEC          0x00010     // EEPROM Control Register
#define EDRV_REGDW_EEC_AUTO_RD  0x00000200  // Auto Read Done

#define EDRV_REGDW_ICR          0x000C0     // Interrupt Cause Read
#define EDRV_REGDW_ITR          0x000C4     // Interrupt Throttling Rate
#define EDRV_REGDW_IMS          0x000D0     // Interrupt Mask Set/Read
#define EDRV_REGDW_IMC          0x000D8     // Interrupt Mask Clear
#define EDRV_REGDW_INT_MASK_ALL 0xFFFFFFFF  // mask all interrupts
#define EDRV_REGDW_INT_TXDW     0x00000001  // Transmit Descriptor Written Back
#define EDRV_REGDW_INT_TXQE     0x00000002  // Transmit Descriptor Queue Empty
#define EDRV_REGDW_INT_LSC      0x00000004  // Link Status Change
#define EDRV_REGDW_INT_RXSEQ    0x00000008  // Receive Sequence Error
#define EDRV_REGDW_INT_RXT0     0x00000080  // Receiver Timer Interrupt
#define EDRV_REGDW_INT_RXDMT0   0x00000010  // Receive Descriptor Minimum Threshold Reached
#define EDRV_REGDW_INT_RXO      0x00000040  // Receiver Overrun
#define EDRV_REGDW_INT_TXD_LOW  0x00008000  // Transmit Descriptor Low Threshold hit
#define EDRV_REGDW_INT_SRPD     0x00010000  // Small Receive Packet Detected
#define EDRV_REGDW_INT_INT_ASSERTED 0x80000000  // PCIe Int. has been asserted

#define EDRV_REGDW_INT_MASK_DEF (EDRV_REGDW_INT_TXDW | \
                                 EDRV_REGDW_INT_RXT0 | \
                                 EDRV_REGDW_INT_RXDMT0 | \
                                 EDRV_REGDW_INT_RXO | \
                                 EDRV_REGDW_INT_RXSEQ)

#define EDRV_REGDW_TIPG         0x00410     // Transmit Inter Packet Gap
#define EDRV_REGDW_TIPG_DEF     0x00702008  // default according to Intel PCIe GbE Controllers Open Source Software Developer's Manual

#define EDRV_REGDW_RDTR         0x02820     // Receive Interrupt Packet Delay Timer
#define EDRV_REGDW_RADV         0x0282C     // Receive Interrupt Absolute Delay Timer

#define EDRV_REGDW_TXDCTL       0x03828     // Transmit Descriptor Control
#define EDRV_REGDW_TXDCTL_GRAN  0x01000000  // Granularity (1=Descriptor, 0=Cache line)
//#define EDRV_REGDW_TXDCTL_WTHRESH 0x01000000  // Write Back Threshold
#define EDRV_REGDW_TXDCTL_DEF   (EDRV_REGDW_TXDCTL_GRAN)

#define EDRV_REGDW_RXDCTL       0x02828     // Receive Descriptor Control
#define EDRV_REGDW_RXDCTL_GRAN  0x01000000  // Granularity (1=Descriptor, 0=Cache line)
#define EDRV_REGDW_RXDCTL_DEF   (EDRV_REGDW_RXDCTL_GRAN)

#define EDRV_REGDW_TCTL         0x00400     // Transmit Control
#define EDRV_REGDW_TCTL_EN      0x00000002  // Transmit Enable
#define EDRV_REGDW_TCTL_PSP     0x00000008  // Pad Short Packets
#define EDRV_REGDW_TCTL_CT      0x000000F0  // Collision Threshold
#define EDRV_REGDW_TCTL_COLD    0x0003F000  // Collision Distance
#define EDRV_REGDW_TCTL_DEF     (EDRV_REGDW_TCTL_EN | \
                                 EDRV_REGDW_TCTL_PSP | \
                                 EDRV_REGDW_TCTL_CT | \
                                 EDRV_REGDW_TCTL_COLD)

#define EDRV_REGDW_RCTL         0x00100     // Receive Control
#define EDRV_REGDW_RCTL_EN      0x00000002  // Receive Enable
#define EDRV_REGDW_RCTL_SBP     0x00000004  // Store Bad Packets (to recognize collisions)
#define EDRV_REGDW_RCTL_BAM     0x00008000  // Broadcast Accept Mode
#define EDRV_REGDW_RCTL_SECRC   0x04000000  // Strip Ethernet CRC (do not store in host memory)

#define EDRV_REGDW_RCTL_BSIZE_2048  0x00000000  // buffer size is 2048 byte

#define EDRV_REGDW_RCTL_DEF     (EDRV_REGDW_RCTL_EN | \
                                 EDRV_REGDW_RCTL_SBP | \
                                 EDRV_REGDW_RCTL_BAM | \
                                 EDRV_REGDW_RCTL_SECRC | \
                                 EDRV_REGDW_RCTL_BSIZE_2048)

#define EDRV_REGDW_TDBAL        0x03800     // Transmit Descriptor Base Address Low
#define EDRV_REGDW_TDBAH        0x03804     // Transmit Descriptor Base Address High
#define EDRV_REGDW_TDLEN        0x03808     // Transmit Descriptor Length
#define EDRV_REGDW_TDH          0x03810     // Transmit Descriptor Head
#define EDRV_REGDW_TDT          0x03818     // Transmit Descriptor Tail

#define EDRV_REGDW_RDBAL0       0x02800     // Receive Descriptor Base Address Low
#define EDRV_REGDW_RDBAH0       0x02804     // Receive Descriptor Base Address High
#define EDRV_REGDW_RDLEN0       0x02808     // Receive Descriptor Length
#define EDRV_REGDW_RDH0         0x02810     // Receive Descriptor Head
#define EDRV_REGDW_RDT0         0x02818     // Receive Descriptor Tail

#define EDRV_REGDW_MTA(n)       (0x05200 + 4*n)  // Multicast Table Array

#define EDRV_REGDW_RAL(n)       (0x05400 + 8*n)  // Receive Address Low
#define EDRV_REGDW_RAH(n)       (0x05404 + 8*n)  // Receive Address HIGH
#define EDRV_REGDW_RAH_AV       0x80000000  // Receive Address Valid

#define EDRV_REGDW_CRCERRS      0x04000     // CRC Error Count
#define EDRV_REGDW_LATECOL      0x04020     // Late Collision Count
#define EDRV_REGDW_COLC         0x04028     // Collision Count
#define EDRV_REGDW_SEC          0x04038     // Sequence Error Count
#define EDRV_REGDW_RLEC         0x04040     // Receive Length Error Count

#define EDRV_REGDW_SWSM         0x05B50     // Software Semaphore
#define EDRV_REGDW_SWSM_SWESMBI 0x00000002  // Software EEPROM Semaphore Bit

// defines for the status byte in the receive descriptor
#define EDRV_RXSTAT_DD          0x01        // Descriptor Done (Processed by Hardware)
#define EDRV_RXSTAT_EOP         0x02        // End of Packet
#define EDRV_RXERR_CE           0x01        // CRC Error
#define EDRV_RXERR_SEQ          0x04        // Sequence Error
#define EDRV_RXERR_OTHER        0xE2        // Other Error


#define EDRV_REGDW_WRITE(reg, val)      writel(val, (UINT8*)edrvInstance_l.pIoAddr + reg)
#define EDRV_REGDW_READ(reg)            readl((UINT8*)edrvInstance_l.pIoAddr + reg)


#define EDRV_SAMPLE_NUM         10000


// TracePoint support for realtime-debugging
#ifdef _DBG_TRACE_POINTS_
void target_signalTracePoint(UINT8 tracePointNumber_p);
#define TGT_DBG_SIGNAL_TRACE_POINT(p)   target_signalTracePoint(p)
#else
#define TGT_DBG_SIGNAL_TRACE_POINT(p)
#endif

#define EDRV_COUNT_SEND                 TGT_DBG_SIGNAL_TRACE_POINT(2)
#define EDRV_COUNT_TIMEOUT              TGT_DBG_SIGNAL_TRACE_POINT(3)
#define EDRV_COUNT_PCI_ERR              TGT_DBG_SIGNAL_TRACE_POINT(4)
#define EDRV_COUNT_TX                   TGT_DBG_SIGNAL_TRACE_POINT(5)
#define EDRV_COUNT_RX                   TGT_DBG_SIGNAL_TRACE_POINT(6)
#define EDRV_COUNT_LATECOLLISION        TGT_DBG_SIGNAL_TRACE_POINT(10)
#define EDRV_COUNT_TX_COL_RL            TGT_DBG_SIGNAL_TRACE_POINT(11)
#define EDRV_COUNT_TX_FUN               TGT_DBG_SIGNAL_TRACE_POINT(12)
#define EDRV_COUNT_TX_TEST              TGT_DBG_SIGNAL_TRACE_POINT(13)
#define EDRV_COUNT_RX_ERR_CRC           TGT_DBG_SIGNAL_TRACE_POINT(14)
#define EDRV_COUNT_RX_ERR_MULT          TGT_DBG_SIGNAL_TRACE_POINT(15)
#define EDRV_COUNT_RX_ERR_SEQ           TGT_DBG_SIGNAL_TRACE_POINT(16)
#define EDRV_COUNT_RX_ERR_OTHER         TGT_DBG_SIGNAL_TRACE_POINT(17)
#define EDRV_COUNT_RX_ORUN              TGT_DBG_SIGNAL_TRACE_POINT(18)

#define EDRV_DIAG_HISTORY_COUNT     14

//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------
/**
\brief Structure describing a transmit descriptor

This structure describes a transmit descriptor of the Intel 82573 Ethernet
chipset.
*/
typedef struct
{
    UINT64          bufferAddr_le;                          ///< Buffer address (little endian)
    UINT32          lengthCmd_le;                           ///< Buffer length (little endian)
    UINT32          status_le;                              ///< Status (little endian)
} tEdrvTxDesc;

/**
\brief Structure describing a receive descriptor

This structure describes a receive descriptor of the Intel 82573 Ethernet
chipset.
*/
typedef struct
{
    UINT64          bufferAddr_le;                          ///< Buffer address (little endian)
    UINT16          length_le;                              ///< Buffer length (little endian)
    UINT16          checksum_le;                            ///< Checksum (little endian)
    UINT8           status;                                 ///< Status
    UINT8           error;                                  ///< Error
    UINT16          reserved_le;                            ///< Reserved
} tEdrvRxDesc;

/**
\brief Structure describing an instance of the Edrv

This structure describes an instance of the Ethernet driver.
*/
typedef struct
{
    tEdrvInitParam  initParam;                              ///< Init parameters
    struct pci_dev* pPciDev;                                ///< Pointer to the PCI device structure
    void*           pIoAddr;                                ///< Pointer to the register space of the Ethernet controller

    tEdrvRxDesc*    pRxDesc;                                ///< Pointer to the RX descriptors
    dma_addr_t      pRxDescDma;                             ///< Pointer to the RX descriptor DMA
    void*           apRxBufInDesc[EDRV_MAX_RX_DESCS];       ///< Stack of free RX buffers +1 additional place if ReleaseRxBuffer is called before return of RxHandler (multi processor)
    void*           apRxBufFree[EDRV_MAX_RX_BUFFERS - EDRV_MAX_RX_DESCS + 1];
                                                            ///< Array of free RX buffers
    int             rxBufFreeTop;                           ///< Index of the top of the free RX buffer array
    spinlock_t      spinLockRxBufRelease;                   ///< Spinlock for protecting the RX buffer release
    int             pageAllocations;                        ///< Counter of allocated pages

    void*           pTxBuf;                                 ///< Pointer to the TX buffer
    dma_addr_t      pTxBufDma;                              ///< Pointer to the DMA of the TX buffer
    tEdrvTxDesc*    pTxDesc;                                ///< Pointer to the TX descriptors
    dma_addr_t      pTxDescDma;                             ///< Pointer to the DMA of the TX descriptors
    tEdrvTxBuffer*  apTxBuffer[EDRV_MAX_TX_DESCS];          ///< Array of TX buffers
    BOOL            afTxBufUsed[EDRV_MAX_TX_BUFFERS];       ///< Array indicating the use of a specific TX buffer

    UINT            headTxDesc;                             ///< Index of the head of the TX descriptor buffer
    UINT            tailTxDesc;                             ///< Index of the tail of the TX descriptor buffer
    UINT            headRxDesc;                             ///< Index of the head of the RX descriptor buffer
    UINT            tailRxDesc;                             ///< Index of the tail of the RX descriptor buffer

#if (CONFIG_EDRV_USE_DIAGNOSTICS != FALSE)
    ULONGLONG       interruptCount;                         ///< Interrupt counter
    int             rxBufFreeMin;                           ///< Minimum number of free RX buffers
    UINT            rxCount[EDRV_SAMPLE_NUM];               ///< Array of RX counter samples
    UINT            txCount[EDRV_SAMPLE_NUM];               ///< Array of TX counter samples
    UINT            pos;                                    ///< Current sample position
#endif
    BOOL            fIrqHandlerEnabled;                     ///< Flag indicating whether the IRQ handler is enabled
} tEdrvInstance;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static irqreturn_t edrvIrqHandler (int irqNum_p, void* pDevInstData_p);
static int initOnePciDev(struct pci_dev* pPciDev_p, const struct pci_device_id* pId_p);
static void removeOnePciDev(struct pci_dev* pPciDev_p);

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
// buffers and buffer descriptors and pointers

static struct pci_device_id aEdrvPciTbl_l[] =
{
    {0x8086, 0x109a, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0},  // 82573L
    {0x8086, 0x1501, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0},  // 82567V
    {0x8086, 0x150c, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0},  // 82583V
    {0x8086, 0x10e5, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0},  // 82567LM ICH 9
    {0x8086, 0x10f5, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0},  // 82567LM ICH 9M
    {0x8086, 0x10cc, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0},  // 82567LM ICH 10
    {0x8086, 0x10de, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0},  // 82567LM ICH 10D
    {0x8086, 0x10d3, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0},  // 82574L
    {0x8086, 0x100E, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0},  // 82540EM
    {0, }
};
MODULE_DEVICE_TABLE(pci, aEdrvPciTbl_l);

static tEdrvInstance edrvInstance_l;
static tBufAlloc* pBufAlloc_l = NULL;
static struct pci_driver edrvDriver_l =
{
    .name         = DRV_NAME,
    .id_table     = aEdrvPciTbl_l,
    .probe        = initOnePciDev,
    .remove       = removeOnePciDev,
};

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Ethernet driver initialization

This function initializes the Ethernet driver.

\param[in]      pEdrvInitParam_p    Edrv initialization parameters

\return The function returns a tOplkError error code.

\ingroup module_edrv
*/
//------------------------------------------------------------------------------
tOplkError edrv_init(const tEdrvInitParam* pEdrvInitParam_p)
{
    tOplkError  ret = kErrorOk;
    int         result;
    int         i;
    tBufData    bufData;

    // Check parameter validity
    ASSERT(pEdrvInitParam_p != NULL);

    // clear instance structure
    OPLK_MEMSET(&edrvInstance_l, 0, sizeof(edrvInstance_l));

    // save the init data
    edrvInstance_l.initParam = *pEdrvInitParam_p;

    // clear driver structure
    OPLK_MEMSET(&edrvDriver_l, 0, sizeof(edrvDriver_l));
    edrvDriver_l.name         = DRV_NAME,
    edrvDriver_l.id_table     = aEdrvPciTbl_l,
    edrvDriver_l.probe        = initOnePciDev,
    edrvDriver_l.remove       = removeOnePciDev,

    // register PCI driver
    result = pci_register_driver(&edrvDriver_l);
    if (result != 0)
    {
        printk("%s pci_register_driver failed with %d\n", __func__, result);
        ret = kErrorNoResource;
        goto Exit;
    }

    if (edrvInstance_l.pPciDev == NULL)
    {
        printk("%s pPciDev=NULL\n", __func__);
        edrv_exit();
        ret = kErrorNoResource;
        goto Exit;
    }

    // init and fill buffer allocation instance
    if ((pBufAlloc_l = bufalloc_init(EDRV_MAX_TX_BUFFERS)) == NULL)
    {
        ret = kErrorNoResource;
        goto Exit;
    }

    for (i = 0; i < EDRV_MAX_TX_BUFFERS; i++)
    {
        bufData.bufferNumber = i;
        bufData.pBuffer = (UINT8*)edrvInstance_l.pTxBuf + (i * EDRV_MAX_FRAME_SIZE);

        bufalloc_addBuffer(pBufAlloc_l, &bufData);
    }

    // local MAC address might have been changed in initOnePciDev
    printk("%s local MAC = ", __func__);
    for (i = 0; i < 6; i++)
    {
        printk("%02X ", (UINT)edrvInstance_l.initParam.aMacAddr[i]);
    }
    printk("\n");

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Shut down Ethernet driver

This function shuts down the Ethernet driver.

\return The function returns a tOplkError error code.

\ingroup module_edrv
*/
//------------------------------------------------------------------------------
tOplkError edrv_exit(void)
{
    if (edrvDriver_l.name != NULL)
    {
        printk("%s calling pci_unregister_driver()\n", __func__);
        pci_unregister_driver(&edrvDriver_l);
        // clear buffer allocation
        bufalloc_exit(pBufAlloc_l);
        pBufAlloc_l = NULL;
        // clear driver structure
        OPLK_MEMSET(&edrvDriver_l, 0, sizeof(edrvDriver_l));
    }
    else
    {
        printk("%s PCI driver for openPOWERLINK already unregistered\n", __func__);
    }

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Get MAC address

This function returns the MAC address of the Ethernet controller

\return The function returns a pointer to the MAC address.

\ingroup module_edrv
*/
//------------------------------------------------------------------------------
const UINT8* edrv_getMacAddr(void)
{
    return edrvInstance_l.initParam.aMacAddr;
}

//------------------------------------------------------------------------------
/**
\brief  Set multicast address entry

This function sets a multicast entry into the Ethernet controller.

\param[in]      pMacAddr_p          Multicast address.

\return The function returns a tOplkError error code.

\ingroup module_edrv
*/
//------------------------------------------------------------------------------
tOplkError edrv_setRxMulticastMacAddr(const UINT8* pMacAddr_p)
{
    tOplkError  ret = kErrorOk;
    UINT32      data;
    int         i;

    // Check parameter validity
    ASSERT(pMacAddr_p != NULL);

    // entry 0 is used for local MAC address
    for (i = 1; i < 16; i++)
    {
        data = EDRV_REGDW_READ(EDRV_REGDW_RAH(i));
        if ((data & EDRV_REGDW_RAH_AV) == 0)
        {   // free MAC address entry
            break;
        }
    }

    if (i == 16)
    {   // no free entry found
        printk("%s Implementation of Multicast Table Array support required\n", __func__);
        ret = kErrorEdrvInit;
        goto Exit;
    }
    else
    {
        // write MAC address to free entry
        data = 0;
        data |= pMacAddr_p[0] <<  0;
        data |= pMacAddr_p[1] <<  8;
        data |= pMacAddr_p[2] << 16;
        data |= pMacAddr_p[3] << 24;
        EDRV_REGDW_WRITE(EDRV_REGDW_RAL(i), data);
        data = 0;
        data |= pMacAddr_p[4] <<  0;
        data |= pMacAddr_p[5] <<  8;
        data |= EDRV_REGDW_RAH_AV;
        EDRV_REGDW_WRITE(EDRV_REGDW_RAH(i), data);
    }

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Clear multicast address entry

This function removes the multicast entry from the Ethernet controller.

\param[in]      pMacAddr_p          Multicast address

\return The function returns a tOplkError error code.

\ingroup module_edrv
*/
//------------------------------------------------------------------------------
tOplkError edrv_clearRxMulticastMacAddr(const UINT8* pMacAddr_p)
{
    tOplkError  ret = kErrorOk;
    UINT32      data;
    int         i;
    UINT32      addrLow;
    UINT32      addrHigh;

    // Check parameter validity
    ASSERT(pMacAddr_p != NULL);

    addrLow   = 0;
    addrLow  |= pMacAddr_p[0] <<  0;
    addrLow  |= pMacAddr_p[1] <<  8;
    addrLow  |= pMacAddr_p[2] << 16;
    addrLow  |= pMacAddr_p[3] << 24;
    addrHigh  = 0;
    addrHigh |= pMacAddr_p[4] <<  0;
    addrHigh |= pMacAddr_p[5] <<  8;
    addrHigh |= EDRV_REGDW_RAH_AV;

    for (i = 1; i < 16; i++)
    {
        data = EDRV_REGDW_READ(EDRV_REGDW_RAH(i));
        if ((data & (EDRV_REGDW_RAH_AV | 0xFFFF)) == addrHigh)
        {
            data = EDRV_REGDW_READ(EDRV_REGDW_RAL(i));
            if (data == addrLow)
            {   // set address valid bit to invalid
                EDRV_REGDW_WRITE(EDRV_REGDW_RAH(i), 0);
                break;
            }
        }
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Change Rx filter setup

This function changes the Rx filter setup. The parameter entryChanged_p
selects the Rx filter entry that shall be changed and \p changeFlags_p determines
the property.
If \p entryChanged_p is equal or larger count_p all Rx filters shall be changed.

\note Rx filters are not supported by this driver!

\param[in,out]  pFilter_p           Base pointer of Rx filter array
\param[in]      count_p             Number of Rx filter array entries
\param[in]      entryChanged_p      Index of Rx filter entry that shall be changed
\param[in]      changeFlags_p       Bit mask that selects the changing Rx filter property

\return The function returns a tOplkError error code.

\ingroup module_edrv
*/
//------------------------------------------------------------------------------
tOplkError edrv_changeRxFilter(tEdrvFilter* pFilter_p,
                               UINT count_p,
                               UINT entryChanged_p,
                               UINT changeFlags_p)
{
    UNUSED_PARAMETER(pFilter_p);
    UNUSED_PARAMETER(count_p);
    UNUSED_PARAMETER(entryChanged_p);
    UNUSED_PARAMETER(changeFlags_p);

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Allocate Tx buffer

This function allocates a Tx buffer.

\param[in,out]  pBuffer_p           Tx buffer descriptor

\return The function returns a tOplkError error code.

\ingroup module_edrv
*/
//------------------------------------------------------------------------------
tOplkError edrv_allocTxBuffer(tEdrvTxBuffer* pBuffer_p)
{
    tOplkError  ret = kErrorOk;
    tBufData    bufData;

    // Check parameter validity
    ASSERT(pBuffer_p != NULL);

    if (pBuffer_p->maxBufferSize > EDRV_MAX_FRAME_SIZE)
    {
        ret = kErrorEdrvNoFreeBufEntry;
        goto Exit;
    }

    if (edrvInstance_l.pTxBuf == NULL)
    {
        printk("%s Tx buffers currently not allocated\n", __func__);
        ret = kErrorEdrvNoFreeBufEntry;
        goto Exit;
    }

    // get a free Tx buffer from the allocation instance
    ret = bufalloc_getBuffer(pBufAlloc_l, &bufData);
    if (ret != kErrorOk)
    {
        ret = kErrorEdrvNoFreeBufEntry;
        goto Exit;
    }

    pBuffer_p->pBuffer = bufData.pBuffer;
    pBuffer_p->txBufferNumber.value = bufData.bufferNumber;
    pBuffer_p->maxBufferSize = EDRV_MAX_FRAME_SIZE;
    edrvInstance_l.afTxBufUsed[bufData.bufferNumber] = TRUE;

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Free Tx buffer

This function releases the Tx buffer.

\param[in,out]  pBuffer_p           Tx buffer descriptor

\return The function returns a tOplkError error code.

\ingroup module_edrv
*/
//------------------------------------------------------------------------------
tOplkError edrv_freeTxBuffer(tEdrvTxBuffer* pBuffer_p)
{
    tOplkError  ret;
    tBufData    bufData;

    // Check parameter validity
    ASSERT(pBuffer_p != NULL);

    bufData.pBuffer = pBuffer_p->pBuffer;
    bufData.bufferNumber = pBuffer_p->txBufferNumber.value;

    edrvInstance_l.afTxBufUsed[pBuffer_p->txBufferNumber.value] = FALSE;
    ret = bufalloc_releaseBuffer(pBufAlloc_l, &bufData);

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Send Tx buffer

This function sends the Tx buffer.

\param[in,out]  pBuffer_p           Tx buffer descriptor

\return The function returns a tOplkError error code.

\ingroup module_edrv
*/
//------------------------------------------------------------------------------
tOplkError edrv_sendTxBuffer(tEdrvTxBuffer* pBuffer_p)
{
    tOplkError      ret = kErrorOk;
    UINT            bufferNumber;
    tEdrvTxDesc*    pTxDesc;

    // Check parameter validity
    ASSERT(pBuffer_p != NULL);

    bufferNumber = pBuffer_p->txBufferNumber.value;

    if ((bufferNumber >= EDRV_MAX_TX_BUFFERS) ||
        (edrvInstance_l.afTxBufUsed[bufferNumber] == FALSE))
    {
        ret = kErrorEdrvBufNotExisting;
        goto Exit;
    }

    // one descriptor has to be left empty for distinction between full and empty
    if (((edrvInstance_l.tailTxDesc + 1) & EDRV_TX_DESC_MASK) == edrvInstance_l.headTxDesc)
    {
        ret = kErrorEdrvNoFreeTxDesc;
        goto Exit;
    }

    EDRV_COUNT_SEND;

    // save pointer to buffer structure for TxHandler
    edrvInstance_l.apTxBuffer[edrvInstance_l.tailTxDesc] = pBuffer_p;

    pTxDesc = &edrvInstance_l.pTxDesc[edrvInstance_l.tailTxDesc];
    pTxDesc->bufferAddr_le = edrvInstance_l.pTxBufDma + (bufferNumber * EDRV_MAX_FRAME_SIZE);
    pTxDesc->status_le = 0;
    pTxDesc->lengthCmd_le = ((UINT32)pBuffer_p->txFrameSize) | EDRV_TX_DESC_CMD_DEF;

    // increment Tx descriptor queue tail pointer
    edrvInstance_l.tailTxDesc = (edrvInstance_l.tailTxDesc + 1) & EDRV_TX_DESC_MASK;

    // start transmission
    EDRV_REGDW_WRITE(EDRV_REGDW_TDT, edrvInstance_l.tailTxDesc);

Exit:
    return ret;
}

#if (CONFIG_EDRV_USE_DIAGNOSTICS != FALSE)
//------------------------------------------------------------------------------
/**
\brief  Get Edrv module diagnostics

This function returns the Edrv diagnostics to a provided buffer.

\param[out]     pBuffer_p           Pointer to buffer filled with diagnostics.
\param[in]      size_p              Size of buffer

\return The function returns the size of the diagnostics information.

\ingroup module_edrv
*/
//------------------------------------------------------------------------------
int edrv_getDiagnostics(char* pBuffer_p, size_t size_p)
{
    tEdrvTxDesc*    pTxDesc;
    UINT32          txStatus;
    size_t          usedSize = 0;

    // Check parameter validity
    ASSERT(pBuffer_p != NULL);

    usedSize += snprintf(pBuffer_p + usedSize, size_p - usedSize,
                         "\nEdrv Diagnostic Information\n");

    usedSize += snprintf(pBuffer_p + usedSize, size_p - usedSize,
                         "Head: %u (%lu)",
                         edrvInstance_l.headTxDesc,
                         (ULONG)EDRV_REGDW_READ(EDRV_REGDW_TDH));

    pTxDesc = &edrvInstance_l.pTxDesc[edrvInstance_l.headTxDesc];
    txStatus = pTxDesc->status_le;

    usedSize += snprintf(pBuffer_p + usedSize, size_p - usedSize,
                         "      Headstatus: %lX\n", (ULONG)txStatus);

    usedSize += snprintf(pBuffer_p + usedSize, size_p - usedSize,
                         "Tail: %u (%lu)\n",
                         edrvInstance_l.tailTxDesc,
                         (ULONG)EDRV_REGDW_READ(EDRV_REGDW_TDT));

    usedSize += snprintf(pBuffer_p + usedSize, size_p - usedSize,
                         "Free RxBuffers: %d (Min: %d)\n",
                         edrvInstance_l.rxBufFreeTop + 1,
                         edrvInstance_l.rxBufFreeMin);

#if 0 //FIXME: Use some fancy way to enable this block.
    for (i = 0; i < 16; i++)
    {
        usedSize += snprintf(pBuffer_p + usedSize, size_p - usedSize,
                             "RAH[%2u] RAL[%2u]: 0x%08lX 0x%08lX\n",
                             i, i,
                             (ULONG)EDRV_REGDW_READ(EDRV_REGDW_RAH(i)),
                             (ULONG)EDRV_REGDW_READ(EDRV_REGDW_RAL(i)));
    }

    usedSize += snprintf(pBuffer_p + usedSize, size_p - usedSize,
                         "Status Register:                     0x%08lX\n",
                         (ULONG)EDRV_REGDW_READ(EDRV_REGDW_STATUS));

    usedSize += snprintf(pBuffer_p + usedSize, size_p - usedSize,
                         "Interrupt Mask Set/Read Register:    0x%08lX\n",
                         (ULONG)EDRV_REGDW_READ(EDRV_REGDW_IMS));

    usedSize += snprintf(pBuffer_p + usedSize, size_p - usedSize,
                         "Receive Control Register:            0x%08lX\n",
                         (ULONG)EDRV_REGDW_READ(EDRV_REGDW_RCTL));

    usedSize += snprintf(pBuffer_p + usedSize, size_p - usedSize,
                         "Receive Descriptor Control Register: 0x%08lX\n",
                         (ULONG)EDRV_REGDW_READ(EDRV_REGDW_RXDCTL));

    usedSize += snprintf(pBuffer_p + usedSize, size_p - usedSize,
                         "Receive Descriptor Length Register:  0x%08lX\n",
                         (ULONG)EDRV_REGDW_READ(EDRV_REGDW_RDLEN0));

    usedSize += snprintf(pBuffer_p + usedSize, size_p - usedSize,
                         "Receive Descriptor Head Register:    0x%08lX (%u)\n",
                         (ULONG)EDRV_REGDW_READ(EDRV_REGDW_RDH0),
                         edrvInstance_l.headRxDesc);

    usedSize += snprintf(pBuffer_p + usedSize, size_p - usedSize,
                         "Receive Descriptor Tail Register:    0x%08lX (%u)\n",
                         (ULONG)EDRV_REGDW_READ(EDRV_REGDW_RDT0),
                         edrvInstance_l.tailRxDesc);
#endif

    usedSize += snprintf(pBuffer_p + usedSize, size_p - usedSize,
                         "Edrv Interrupts:                     %llu\n",
                         edrvInstance_l.interruptCount);

    usedSize += snprintf(pBuffer_p + usedSize, size_p - usedSize,
                         "CRC Error Count:                     %lu\n",
                         (ULONG)EDRV_REGDW_READ(EDRV_REGDW_CRCERRS));

    usedSize += snprintf(pBuffer_p + usedSize, size_p - usedSize,
                         "Late Collision Count:                %lu\n",
                         (ULONG)EDRV_REGDW_READ(EDRV_REGDW_LATECOL));

    usedSize += snprintf(pBuffer_p + usedSize, size_p - usedSize,
                         "Collision Count:                     %lu\n",
                         (ULONG)EDRV_REGDW_READ(EDRV_REGDW_COLC));

    usedSize += snprintf(pBuffer_p + usedSize, size_p - usedSize,
                         "Sequence Error Count:                %lu\n",
                         (ULONG)EDRV_REGDW_READ(EDRV_REGDW_SEC));

    usedSize += snprintf(pBuffer_p + usedSize, size_p - usedSize,
                         "Receive Length Error Count:          %lu\n",
                         (ULONG)EDRV_REGDW_READ(EDRV_REGDW_RLEC));

    {
        UINT    rxCountMean = 0;
        UINT    txCountMean = 0;
        UINT    aHistoryRx[EDRV_DIAG_HISTORY_COUNT];
        UINT    aHistoryTx[EDRV_DIAG_HISTORY_COUNT];
        UINT    historyRxMax;
        UINT    historyTxMax;
        int     i;

        for (i = 0; i < EDRV_SAMPLE_NUM; i++)
        {
            rxCountMean += edrvInstance_l.rxCount[i] * 100;
            txCountMean += edrvInstance_l.txCount[i] * 100;
        }

        rxCountMean /= EDRV_SAMPLE_NUM;
        txCountMean /= EDRV_SAMPLE_NUM;

        usedSize += snprintf(pBuffer_p + usedSize, size_p - usedSize,
                             "Rx/Int %u.%02u / Tx/Int %u.%02u\n",
                             rxCountMean / 100, rxCountMean % 100,
                             txCountMean / 100, txCountMean % 100);

        historyRxMax = 0;
        historyTxMax = 0;
        for (i = 0; i < EDRV_DIAG_HISTORY_COUNT; i++)
        {
            aHistoryRx[i] = 0;
            aHistoryTx[i] = 0;
        }

        for (i = 0; i < EDRV_SAMPLE_NUM; i++)
        {
            if (edrvInstance_l.rxCount[i] < EDRV_DIAG_HISTORY_COUNT)
            {
                aHistoryRx[edrvInstance_l.rxCount[i]]++;
            }
            else if (edrvInstance_l.rxCount[i] > historyRxMax)
            {
                historyRxMax = edrvInstance_l.rxCount[i];
            }

            if (edrvInstance_l.txCount[i] < EDRV_DIAG_HISTORY_COUNT)
            {
                aHistoryTx[edrvInstance_l.txCount[i]]++;
            }
            else if (edrvInstance_l.txCount[i] > historyTxMax)
            {
                historyTxMax = edrvInstance_l.txCount[i];
            }
        }

        if (historyRxMax > 0)
        {
            usedSize += snprintf(pBuffer_p + usedSize, size_p - usedSize,
                                 "MaxRx %3u\n", historyRxMax);
        }

        if (historyTxMax > 0)
        {
            usedSize += snprintf(pBuffer_p + usedSize, size_p - usedSize,
                                 "MaxTx %3u\n", historyTxMax);
        }

        for (i = EDRV_DIAG_HISTORY_COUNT - 1; i >= 0; i--)
        {
            if ((aHistoryRx[i] != 0) || (aHistoryTx[i] != 0))
            {
                usedSize += snprintf(pBuffer_p + usedSize, size_p - usedSize,
                                     "Hist  %3u  %5u  %5u\n",
                                     i, aHistoryRx[i], aHistoryTx[i]);
            }
        }
    }

    usedSize += snprintf(pBuffer_p + usedSize, size_p - usedSize,
                         "\n");

    return usedSize;
}
#endif

#if ((CONFIG_DLL_DEFERRED_RXFRAME_RELEASE_SYNC != FALSE) || (CONFIG_DLL_DEFERRED_RXFRAME_RELEASE_ASYNC != FALSE))
//------------------------------------------------------------------------------
/**
\brief  Release Rx buffer

This function releases a late release Rx buffer.

\param[in,out]  pRxBuffer_p         Rx buffer to be released

\return The function returns a tOplkError error code.

\ingroup module_edrv
*/
//------------------------------------------------------------------------------
tOplkError edrv_releaseRxBuffer(tEdrvRxBuffer* pRxBuffer_p)
{
    tOplkError ret = kErrorEdrvInvalidRxBuf;

    // Check parameter validity
    ASSERT(pRxBuffer_p != NULL);

    if (edrvInstance_l.rxBufFreeTop < (EDRV_MAX_RX_BUFFERS-1))
    {
        ULONG flags;

        spin_lock_irqsave(&edrvInstance_l.spinLockRxBufRelease, flags);
        edrvInstance_l.apRxBufFree[++edrvInstance_l.rxBufFreeTop] = pRxBuffer_p->pBuffer;
        spin_unlock_irqrestore(&edrvInstance_l.spinLockRxBufRelease, flags);

        ret = kErrorOk;
    }

    return ret;
}
#endif

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief  Ethernet driver interrupt handler

This function is the interrupt service routine for the Ethernet driver.

\param[in]      irqNum_p            IRQ number
\param[in,out]  pDevInstData_p      Pointer to private data provided by request_irq

\return The function returns an IRQ handled code.
*/
//------------------------------------------------------------------------------
static irqreturn_t edrvIrqHandler(int irqNum_p, void* pDevInstData_p)
{
    UINT32      status;
    irqreturn_t handled = IRQ_HANDLED;

    UNUSED_PARAMETER(irqNum_p);
    UNUSED_PARAMETER(pDevInstData_p);

    // Read the interrupt status
    status = EDRV_REGDW_READ(EDRV_REGDW_ICR);

    if ((status & EDRV_REGDW_INT_MASK_DEF) == 0)
    {
        handled = IRQ_NONE;
        EDRV_COUNT_PCI_ERR;
        goto Exit;
    }

    if ((status & EDRV_REGDW_INT_INT_ASSERTED) == 0)
    {   // Manual acknowledge required
        EDRV_REGDW_WRITE(EDRV_REGDW_ICR, status);
    }

#if (CONFIG_EDRV_USE_DIAGNOSTICS != FALSE)
    edrvInstance_l.interruptCount++;
    edrvInstance_l.rxCount[edrvInstance_l.pos] = 0;
    edrvInstance_l.txCount[edrvInstance_l.pos] = 0;
#endif

    // Receive or/and transmit interrupt
    // Handling of Rx has priority
    if ((status & (EDRV_REGDW_INT_RXT0 | EDRV_REGDW_INT_SRPD | EDRV_REGDW_INT_RXDMT0 | // Receive interrupt
                   EDRV_REGDW_INT_TXDW)) != 0)                                         // Transmit interrupt
    {
        UINT headRxDescOrg;

        if (edrvInstance_l.pTxBuf == NULL)
        {
            printk("%s Tx buffers currently not allocated\n", __func__);
            goto Exit;
        }

        headRxDescOrg = edrvInstance_l.headRxDesc;

        do
        {
            tEdrvRxDesc*    pRxDesc;
            tEdrvTxDesc*    pTxDesc;

            // Process receive descriptors
            pRxDesc = &edrvInstance_l.pRxDesc[edrvInstance_l.headRxDesc];

            while (pRxDesc->status != 0)
            {   // Rx frame available
                tEdrvRxBuffer           rxBuffer;
                tEdrvReleaseRxBuffer    retReleaseRxBuffer;
                UINT8                   rxStatus;
                UINT8                   rxError;

#if (CONFIG_EDRV_USE_DIAGNOSTICS != FALSE)
                edrvInstance_l.rxCount[edrvInstance_l.pos]++;
#endif

                rxStatus = pRxDesc->status;
                rxError  = pRxDesc->error;

                if ((rxStatus & EDRV_RXSTAT_DD) != 0)
                {   // Descriptor is valid
                    if ((rxStatus & EDRV_RXSTAT_EOP) == 0)
                    {   // Multiple descriptors used for one packet
                        EDRV_COUNT_RX_ERR_MULT;
                    }
                    else if ((rxError & EDRV_RXERR_CE) != 0)
                    {   // CRC error
                        EDRV_COUNT_RX_ERR_CRC;
                    }
                    else if ((rxError & EDRV_RXERR_SEQ) != 0)
                    {   // Packet sequence error
                        EDRV_COUNT_RX_ERR_SEQ;
                    }
                    else if ((rxError & EDRV_RXERR_OTHER) != 0)
                    {   // Other error
                        EDRV_COUNT_RX_ERR_OTHER;
                    }
                    else
                    {   // Packet is OK
                        void**  ppRxBufInDesc;

                        rxBuffer.bufferInFrame = kEdrvBufferLastInFrame;

                        // Get length of received packet
                        // length_le does not contain CRC as EDRV_REGDW_RCTL_SECRC is set
                        rxBuffer.rxFrameSize = ami_getUint16Le(&pRxDesc->length_le);

                        ppRxBufInDesc = &edrvInstance_l.apRxBufInDesc[edrvInstance_l.headRxDesc];
                        rxBuffer.pBuffer = *ppRxBufInDesc;

                        EDRV_COUNT_RX;

                        pci_dma_sync_single_for_cpu(edrvInstance_l.pPciDev,
                                                    (dma_addr_t)ami_getUint64Le(&pRxDesc->bufferAddr_le),
                                                    EDRV_RX_BUFFER_SIZE, PCI_DMA_FROMDEVICE);

                        // Call Rx handler of Data link layer
                        retReleaseRxBuffer = edrvInstance_l.initParam.pfnRxHandler(&rxBuffer);
                        if (retReleaseRxBuffer == kEdrvReleaseRxBufferLater)
                        {
                            if (edrvInstance_l.rxBufFreeTop >= 0)
                            {
                                dma_addr_t  dmaAddr;
                                void*       pRxBufInDescPrev;
                                ULONG       flags;

#if (CONFIG_EDRV_USE_DIAGNOSTICS != FALSE)
                                if (edrvInstance_l.rxBufFreeTop < edrvInstance_l.rxBufFreeMin)
                                {
                                    edrvInstance_l.rxBufFreeMin = edrvInstance_l.rxBufFreeTop;
                                }
#endif
                                pRxBufInDescPrev = *ppRxBufInDesc;

                                spin_lock_irqsave(&edrvInstance_l.spinLockRxBufRelease, flags);
                                *ppRxBufInDesc = edrvInstance_l.apRxBufFree[edrvInstance_l.rxBufFreeTop--];
                                spin_unlock_irqrestore(&edrvInstance_l.spinLockRxBufRelease, flags);

                                if (*ppRxBufInDesc != pRxBufInDescPrev)
                                {
                                    pci_unmap_single(edrvInstance_l.pPciDev,
                                                     (dma_addr_t)ami_getUint64Le(&pRxDesc->bufferAddr_le),
                                                     EDRV_RX_BUFFER_SIZE, PCI_DMA_FROMDEVICE);

                                    dmaAddr = pci_map_single(edrvInstance_l.pPciDev, *ppRxBufInDesc,
                                                             EDRV_RX_BUFFER_SIZE, PCI_DMA_FROMDEVICE);
                                    if (pci_dma_mapping_error(edrvInstance_l.pPciDev, dmaAddr))
                                    {
                                        printk("%s DMA mapping error\n", __func__);
                                        // $$$ Signal dma mapping error
                                    }

                                    ami_setUint64Le(&pRxDesc->bufferAddr_le, (UINT64)dmaAddr);
                                }
                            }
                            else
                            {
                                // $$$ How to signal no free RxBuffers left?
#if (CONFIG_EDRV_USE_DIAGNOSTICS != FALSE)
                                edrvInstance_l.rxBufFreeMin = -1;
#endif
                            }
                        }
                    }
                }
                else
                {   // Status written by hardware but desc not done
                    EDRV_COUNT_RX_ERR_OTHER;
                }

                pRxDesc->status = 0;

                edrvInstance_l.headRxDesc = (edrvInstance_l.headRxDesc + 1) & EDRV_RX_DESC_MASK;
                pRxDesc = &edrvInstance_l.pRxDesc[edrvInstance_l.headRxDesc];
            }

            // Process one transmit descriptor
            pTxDesc = &edrvInstance_l.pTxDesc[edrvInstance_l.headTxDesc];

            if ((pTxDesc->status_le & EDRV_TX_DESC_STATUS_DD) != 0)
            {   // Transmit finished
                tEdrvTxBuffer*  pTxBuffer;
                UINT32          txStatus;

#if (CONFIG_EDRV_USE_DIAGNOSTICS != FALSE)
                edrvInstance_l.txCount[edrvInstance_l.pos]++;
#endif

                txStatus = pTxDesc->status_le;

                // Delete DD flag
                pTxDesc->status_le = 0;

                pTxBuffer = edrvInstance_l.apTxBuffer[edrvInstance_l.headTxDesc];
                edrvInstance_l.apTxBuffer[edrvInstance_l.headTxDesc] = NULL;

                // Increment Tx descriptor queue head pointer
                edrvInstance_l.headTxDesc = (edrvInstance_l.headTxDesc + 1) & EDRV_TX_DESC_MASK;

                if ((txStatus & EDRV_TX_DESC_STATUS_EC) != 0)
                {
                    EDRV_COUNT_TX_COL_RL;
                }
                else if ((txStatus & EDRV_TX_DESC_STATUS_LC) != 0)
                {
                    EDRV_COUNT_LATECOLLISION;
                }
                else
                {
                    EDRV_COUNT_TX;
                }

                if (pTxBuffer != NULL)
                {
                    // Call Tx handler of Data link layer
                    if (pTxBuffer->pfnTxHandler != NULL)
                    {
                        pTxBuffer->pfnTxHandler(pTxBuffer);
                    }
                }
                else
                {
                    EDRV_COUNT_TX_FUN;
                }
            }
            else
            {
                break;
            }

            if (pRxDesc->status != 0)
            {
                EDRV_COUNT_TX_TEST;
            }
        }
        while (edrvInstance_l.headTxDesc != edrvInstance_l.tailTxDesc);

        if (edrvInstance_l.headRxDesc != headRxDescOrg)
        {
            // Release receive descriptors
            if (edrvInstance_l.headRxDesc == 0)
            {
                edrvInstance_l.tailRxDesc = EDRV_MAX_RX_DESCS - 1;
            }
            else
            {
                edrvInstance_l.tailRxDesc = edrvInstance_l.headRxDesc - 1;
            }

            EDRV_REGDW_WRITE(EDRV_REGDW_RDT0, edrvInstance_l.tailRxDesc);
        }
    }

    if ((status & (EDRV_REGDW_INT_RXSEQ | EDRV_REGDW_INT_RXO)) != 0)
    {   // Receive error interrupt

        if ((status & (EDRV_REGDW_INT_RXSEQ)) != 0)
        {   // Ethernet frame sequencing error
            EDRV_COUNT_RX_ERR_SEQ;
        }

        if ((status & (EDRV_REGDW_INT_RXO)) != 0)
        {   // Receive queue overrun
            EDRV_COUNT_RX_ORUN;
        }
    }

#if (CONFIG_EDRV_USE_DIAGNOSTICS != FALSE)
    edrvInstance_l.pos++;
    if (edrvInstance_l.pos == EDRV_SAMPLE_NUM)
    {
        edrvInstance_l.pos = 0;
    }
#endif

Exit:
    return handled;
}

//------------------------------------------------------------------------------
/**
\brief  Initialize one PCI device

This function initializes one PCI device.

\param[in,out]  pPciDev_p           Pointer to corresponding PCI device structure
\param[in]      pId_p               PCI device ID

\return The function returns an integer error code.
\retval 0           Successful
\retval Otherwise   Error
*/
//------------------------------------------------------------------------------
static int initOnePciDev(struct pci_dev* pPciDev_p, const struct pci_device_id* pId_p)
{
    int     result = 0;
    UINT32  temp;
    UINT64  descAddress;
    int     i;
    UINT    order;
    UINT    rxBuffersInAllocation;
    UINT    rxBuffer;

    if (edrvInstance_l.pPciDev != NULL)
    {   // Edrv is already connected to a PCI device
        printk("%s device %s discarded\n", __func__, pci_name(pPciDev_p));
        result = -ENODEV;
        goto Exit;
    }

    // enable device
    printk("%s enable device\n", __func__);
    result = pci_enable_device(pPciDev_p);
    if (result != 0)
    {
        goto Exit;
    }

    edrvInstance_l.pPciDev = pPciDev_p;

    if (edrvInstance_l.pPciDev == NULL)
    {
        printk("%s pPciDev_p==NULL\n", __func__);
    }

    result = pci_request_regions(pPciDev_p, DRV_NAME);
    if (result != 0)
    {
        goto ExitFail;
    }

    edrvInstance_l.pIoAddr = ioremap(pci_resource_start(pPciDev_p, 0), pci_resource_len(pPciDev_p, 0));
    if (edrvInstance_l.pIoAddr == NULL)
    {   // remap of controller's register space failed
        result = -EIO;
        goto ExitFail;
    }

    spin_lock_init(&edrvInstance_l.spinLockRxBufRelease);

    // enable PCI busmaster
    pci_set_master(pPciDev_p);

    // disable GIO Master accesses
    temp = EDRV_REGDW_READ(EDRV_REGDW_CTRL);
    temp |= EDRV_REGDW_CTRL_MST_DIS;
    EDRV_REGDW_WRITE(EDRV_REGDW_CTRL, temp);

    // wait until master is disabled
    for (i = EDRV_MASTER_DISABLE_TIMEOUT; i > 0; i--)
    {
        if ((EDRV_REGDW_READ(EDRV_REGDW_STATUS) & EDRV_REGDW_STATUS_MST_EN) == 0)
        {
            break;
        }

        msleep(1);
    }

    // From Intel documentation for 82540EM: this controller can't ack the 64-bit write when issuing the reset
    if ((i == 0) && !((pId_p->vendor == 0x8086) && (pId_p->device == 0x100E)))
    {
        result = -EIO;
        goto ExitFail;
    }

    // disable interrupts
    EDRV_REGDW_WRITE(EDRV_REGDW_IMC, EDRV_REGDW_INT_MASK_ALL);

    // reset controller
    temp |= EDRV_REGDW_CTRL_RST;
    EDRV_REGDW_WRITE(EDRV_REGDW_CTRL, temp);

    // wait until reset has finished and configuration from EEPROM was read
    for (i = EDRV_AUTO_READ_DONE_TIMEOUT; i > 0; i--)
    {
        if ((EDRV_REGDW_READ(EDRV_REGDW_EEC) & EDRV_REGDW_EEC_AUTO_RD) != 0)
        {
            break;
        }

        msleep(1);
    }
    if (i == 0)
    {
        result = -EIO;
        goto ExitFail;
    }

    // disable interrupts
    EDRV_REGDW_WRITE(EDRV_REGDW_IMC, EDRV_REGDW_INT_MASK_ALL);
    temp = EDRV_REGDW_READ(EDRV_REGDW_ICR);

    // set global configuration
    EDRV_REGDW_WRITE(EDRV_REGDW_CTRL, EDRV_REGDW_CTRL_DEF);

    // PHY reset by software
    // 1. Obtain the Software/Firmware semaphore (SWSM.SWESMBI). Set it to 1b.
    temp = EDRV_REGDW_READ(EDRV_REGDW_SWSM);
    temp |= EDRV_REGDW_SWSM_SWESMBI;
    EDRV_REGDW_WRITE(EDRV_REGDW_SWSM, temp);
    // 2. Drive PHY reset (CTRL.PHY_RST, write 1b, wait 100 us, and then write 0b).
    temp = EDRV_REGDW_READ(EDRV_REGDW_CTRL);
    temp |= EDRV_REGDW_CTRL_PHY_RST;
    EDRV_REGDW_WRITE(EDRV_REGDW_CTRL, temp);
    msleep(1);
    temp &= ~EDRV_REGDW_CTRL_PHY_RST;
    EDRV_REGDW_WRITE(EDRV_REGDW_CTRL, temp);
    // 3. Delay 10 ms
    msleep(10);
    // 4. Start configuring the PHY.

    // 5. Release the Software/Firmware semaphore
    temp = EDRV_REGDW_READ(EDRV_REGDW_SWSM);
    temp &= ~EDRV_REGDW_SWSM_SWESMBI;
    EDRV_REGDW_WRITE(EDRV_REGDW_SWSM, temp);

    // Clear statistical registers
    temp = EDRV_REGDW_READ(EDRV_REGDW_CRCERRS);
    temp = EDRV_REGDW_READ(EDRV_REGDW_LATECOL);
    temp = EDRV_REGDW_READ(EDRV_REGDW_COLC);
    temp = EDRV_REGDW_READ(EDRV_REGDW_SEC);
    temp = EDRV_REGDW_READ(EDRV_REGDW_RLEC);

    // set TIPG
    EDRV_REGDW_WRITE(EDRV_REGDW_TIPG, EDRV_REGDW_TIPG_DEF);

    // set interrupt throttling
    // setup 1: 12,8us inter-interrupt interval
    //EDRV_REGDW_WRITE(EDRV_REGDW_ITR, 50);
    // setup 2: 25,6us inter-interrupt interval
    //EDRV_REGDW_WRITE(EDRV_REGDW_ITR, 100);
    // setup 3: 51,2us inter-interrupt interval (sporadic errors)
    //EDRV_REGDW_WRITE(EDRV_REGDW_ITR, 200);
    // setup 4: 64us inter-interrupt interval (abort with error)
    //EDRV_REGDW_WRITE(EDRV_REGDW_ITR, 250);
    // setup 5: 10,2us interrupt delay after packet reception
    //          102,4us max. delay
    //          no throttling for packet transmission
    //EDRV_REGDW_WRITE(EDRV_REGDW_RDTR, 10);
    //EDRV_REGDW_WRITE(EDRV_REGDW_RADV, 100);

    // Enable Message Signaled Interrupt
    result = pci_enable_msi(pPciDev_p);
    if (result != 0)
    {
        printk("%s Could not enable MSI\n", __func__);
    }

    // install interrupt handler
    result = request_irq(pPciDev_p->irq, edrvIrqHandler, IRQF_SHARED, DRV_NAME, pPciDev_p);
    if (result != 0)
    {
        goto ExitFail;
    }
    edrvInstance_l.fIrqHandlerEnabled = TRUE;

    // allocate buffers
    result = pci_set_dma_mask(pPciDev_p, DMA_BIT_MASK(32));
    if (result != 0)
    {
        printk(KERN_WARNING "Edrv82573: No suitable DMA available.\n");
        goto ExitFail;
    }

    // allocate tx-buffers
    edrvInstance_l.pTxBuf = pci_alloc_consistent(pPciDev_p, EDRV_TX_BUFFER_SIZE,
                                                 &edrvInstance_l.pTxBufDma);
    if (edrvInstance_l.pTxBuf == NULL)
    {
        result = -ENOMEM;
        goto ExitFail;
    }

    // allocate tx-descriptors
    edrvInstance_l.pTxDesc = pci_alloc_consistent(pPciDev_p, EDRV_TX_DESCS_SIZE,
                                                  &edrvInstance_l.pTxDescDma);
    if (edrvInstance_l.pTxDesc == NULL)
    {
        result = -ENOMEM;
        goto ExitFail;
    }

    // allocate rx-descriptors
    edrvInstance_l.pRxDesc = pci_alloc_consistent(pPciDev_p, EDRV_RX_DESCS_SIZE,
                                                  &edrvInstance_l.pRxDescDma);
    if (edrvInstance_l.pRxDesc == NULL)
    {
        result = -ENOMEM;
        goto ExitFail;
    }


    // allocate rx-buffers
    if ((EDRV_RX_BUFFER_SIZE_SHIFT - PAGE_SHIFT) >= 0)
    {   // rx-buffer is larger than or equal to page size
        order = EDRV_RX_BUFFER_SIZE_SHIFT - PAGE_SHIFT;
        rxBuffersInAllocation = 1;
    }
    else
    {   // Multiple rx-buffers fit into one page
        order = 0;
        rxBuffersInAllocation = 1 << (PAGE_SHIFT - EDRV_RX_BUFFER_SIZE_SHIFT);
    }

    for (rxBuffer = 0; rxBuffer < EDRV_MAX_RX_BUFFERS;)
    {
        ULONG   bufferPointer;
        UINT    inAlloc;

        bufferPointer = __get_free_pages(GFP_KERNEL, order);
        if (bufferPointer == 0)
        {
            result = -ENOMEM;
            goto ExitFail;
        }
        edrvInstance_l.pageAllocations++;

        for (inAlloc = 0; inAlloc < rxBuffersInAllocation; inAlloc++)
        {
            if (rxBuffer < EDRV_MAX_RX_DESCS)
            {   // Insert rx-buffer in rx-descriptor
                edrvInstance_l.apRxBufInDesc[rxBuffer] = (void*)bufferPointer;
            }
            else
            {   // Insert rx-buffer in free-rx-buffer stack
                edrvInstance_l.apRxBufFree[rxBuffer - EDRV_MAX_RX_DESCS] = (void*)bufferPointer;
            }

            rxBuffer++;
            bufferPointer += EDRV_RX_BUFFER_SIZE;
        }
    }

    edrvInstance_l.rxBufFreeTop = EDRV_MAX_RX_BUFFERS - EDRV_MAX_RX_DESCS - 1;
#if (CONFIG_EDRV_USE_DIAGNOSTICS != FALSE)
    edrvInstance_l.rxBufFreeMin = EDRV_MAX_RX_BUFFERS - EDRV_MAX_RX_DESCS;
#endif


    // check if user specified a MAC address
    if ((edrvInstance_l.initParam.aMacAddr[0] != 0) ||
        (edrvInstance_l.initParam.aMacAddr[1] != 0) ||
        (edrvInstance_l.initParam.aMacAddr[2] != 0) ||
        (edrvInstance_l.initParam.aMacAddr[3] != 0) ||
        (edrvInstance_l.initParam.aMacAddr[4] != 0) ||
        (edrvInstance_l.initParam.aMacAddr[5] != 0))
    {   // write specified MAC address to controller
        temp = 0;
        EDRV_REGDW_WRITE(EDRV_REGDW_RAH(0), temp); // disable Entry
        temp |= edrvInstance_l.initParam.aMacAddr[0] <<  0;
        temp |= edrvInstance_l.initParam.aMacAddr[1] <<  8;
        temp |= edrvInstance_l.initParam.aMacAddr[2] << 16;
        temp |= edrvInstance_l.initParam.aMacAddr[3] << 24;
        EDRV_REGDW_WRITE(EDRV_REGDW_RAL(0), temp);
        temp = 0;
        temp |= edrvInstance_l.initParam.aMacAddr[4] <<  0;
        temp |= edrvInstance_l.initParam.aMacAddr[5] <<  8;
        temp |= EDRV_REGDW_RAH_AV;
        EDRV_REGDW_WRITE(EDRV_REGDW_RAH(0), temp);
    }
    else
    {   // read MAC address from controller
        temp = EDRV_REGDW_READ(EDRV_REGDW_RAL(0));
        edrvInstance_l.initParam.aMacAddr[0] = (temp >>  0) & 0xFF;
        edrvInstance_l.initParam.aMacAddr[1] = (temp >>  8) & 0xFF;
        edrvInstance_l.initParam.aMacAddr[2] = (temp >> 16) & 0xFF;
        edrvInstance_l.initParam.aMacAddr[3] = (temp >> 24) & 0xFF;
        temp = EDRV_REGDW_READ(EDRV_REGDW_RAH(0));
        edrvInstance_l.initParam.aMacAddr[4] = (temp >>  0) & 0xFF;
        edrvInstance_l.initParam.aMacAddr[5] = (temp >>  8) & 0xFF;
    }

    // initialize Multicast Table Array to 0
    for (i = 0; i < 128; i++)
    {
        EDRV_REGDW_WRITE(EDRV_REGDW_MTA(i), 0);
    }

    // initialize Rx descriptors
    for (i = 0; i < EDRV_MAX_RX_DESCS; i++)
    {
        dma_addr_t  dmaAddr;

        // get dma streaming
        dmaAddr = pci_map_single(edrvInstance_l.pPciDev, edrvInstance_l.apRxBufInDesc[i],
                                 EDRV_RX_BUFFER_SIZE, PCI_DMA_FROMDEVICE);
        if (pci_dma_mapping_error(edrvInstance_l.pPciDev, dmaAddr))
        {
            result = -ENOMEM;
            goto ExitFail;
        }

        ami_setUint64Le(&edrvInstance_l.pRxDesc[i].bufferAddr_le, (UINT64)dmaAddr);
        edrvInstance_l.pRxDesc[i].status = 0;
    }

    EDRV_REGDW_WRITE(EDRV_REGDW_RXDCTL, EDRV_REGDW_RXDCTL_DEF);
    // Rx buffer size is set to 2048 by default
    // Rx descriptor typ is set to legacy by default
    descAddress = edrvInstance_l.pRxDescDma;
    EDRV_REGDW_WRITE(EDRV_REGDW_RDBAL0, (descAddress & 0xFFFFFFFF));
    EDRV_REGDW_WRITE(EDRV_REGDW_RDBAH0, (descAddress >> 32));
    EDRV_REGDW_WRITE(EDRV_REGDW_RDLEN0, EDRV_RX_DESCS_SIZE);
    edrvInstance_l.headRxDesc = 0;
    EDRV_REGDW_WRITE(EDRV_REGDW_RDH0, 0);
    edrvInstance_l.tailRxDesc = EDRV_MAX_RX_DESCS - 1;
    EDRV_REGDW_WRITE(EDRV_REGDW_RDT0, EDRV_MAX_RX_DESCS - 1);

    // enable receiver
    EDRV_REGDW_WRITE(EDRV_REGDW_RCTL, EDRV_REGDW_RCTL_DEF);

    // initialize Tx descriptors
    OPLK_MEMSET(edrvInstance_l.apTxBuffer, 0, sizeof(edrvInstance_l.apTxBuffer));
    EDRV_REGDW_WRITE(EDRV_REGDW_TXDCTL, EDRV_REGDW_TXDCTL_DEF);
    descAddress = edrvInstance_l.pTxDescDma;
    EDRV_REGDW_WRITE(EDRV_REGDW_TDBAL, (descAddress & 0xFFFFFFFF));
    EDRV_REGDW_WRITE(EDRV_REGDW_TDBAH, (descAddress >> 32));
    EDRV_REGDW_WRITE(EDRV_REGDW_TDLEN, EDRV_TX_DESCS_SIZE);
    EDRV_REGDW_WRITE(EDRV_REGDW_TDH, 0);
    EDRV_REGDW_WRITE(EDRV_REGDW_TDT, 0);

    // enable transmitter
    EDRV_REGDW_WRITE(EDRV_REGDW_TCTL, EDRV_REGDW_TCTL_DEF);

    // enable interrupts
    EDRV_REGDW_WRITE(EDRV_REGDW_IMS, EDRV_REGDW_INT_MASK_DEF);

    // wait until link is up
    printk("%s waiting for link up...\n", __func__);
    for (i = EDRV_LINK_UP_TIMEOUT; i > 0; i -= 100)
    {
        if ((EDRV_REGDW_READ(EDRV_REGDW_STATUS) & EDRV_REGDW_STATUS_LU) != 0)
        {
            break;
        }

        msleep(100);
    }
    if (i == 0)
    {
        result = -EIO;
        goto ExitFail;
    }

    goto Exit;

ExitFail:
    removeOnePciDev(pPciDev_p);

Exit:
    printk("%s finished with %d\n", __func__, result);
    return result;
}

//------------------------------------------------------------------------------
/**
\brief  Remove one PCI device

This function removes one PCI device.

\param[in,out]  pPciDev_p           Pointer to corresponding PCI device structure
*/
//------------------------------------------------------------------------------
static void removeOnePciDev(struct pci_dev* pPciDev_p)
{
    UINT32  temp;
    UINT    order;
    ULONG   bufferPointer;
    UINT    rxBuffer;

    if (edrvInstance_l.pPciDev != pPciDev_p)
    {   // trying to remove unknown device
        BUG_ON(edrvInstance_l.pPciDev != pPciDev_p);
        goto Exit;
    }

    if (edrvInstance_l.pIoAddr != NULL)
    {
        // disable interrupts
        EDRV_REGDW_WRITE(EDRV_REGDW_IMC, EDRV_REGDW_INT_MASK_ALL);
        temp = EDRV_REGDW_READ(EDRV_REGDW_ICR);

        // disable transmitter and receiver
        EDRV_REGDW_WRITE(EDRV_REGDW_TCTL, 0);
    }

    // remove interrupt handler
    if (edrvInstance_l.fIrqHandlerEnabled)
    {
        free_irq(pPciDev_p->irq, pPciDev_p);
        edrvInstance_l.fIrqHandlerEnabled = FALSE;
    }

    // Disable Message Signaled Interrupt
    pci_disable_msi(pPciDev_p);

    // free buffers
    if (edrvInstance_l.pTxBuf != NULL)
    {
        pci_free_consistent(pPciDev_p, EDRV_TX_BUFFER_SIZE,
                            edrvInstance_l.pTxBuf, edrvInstance_l.pTxBufDma);
        edrvInstance_l.pTxBuf = NULL;
    }

    if (edrvInstance_l.pTxDesc != NULL)
    {
        pci_free_consistent(pPciDev_p, EDRV_TX_DESCS_SIZE,
                            edrvInstance_l.pTxDesc, edrvInstance_l.pTxDescDma);
        edrvInstance_l.pTxDesc = NULL;
    }

    if ((EDRV_RX_BUFFER_SIZE_SHIFT - PAGE_SHIFT) >= 0)
    {   // rx-buffer is larger than or equal to page size
        order = EDRV_RX_BUFFER_SIZE_SHIFT - PAGE_SHIFT;
    }
    else
    {   // Multiple rx-buffers fit into one page
        order = 0;
    }

    for (rxBuffer = 0; rxBuffer < EDRV_MAX_RX_DESCS; rxBuffer++)
    {
        pci_unmap_single(edrvInstance_l.pPciDev,
                         (dma_addr_t)ami_getUint64Le(&edrvInstance_l.pRxDesc[rxBuffer].bufferAddr_le),
                         EDRV_RX_BUFFER_SIZE, PCI_DMA_FROMDEVICE);

        bufferPointer = (ULONG)edrvInstance_l.apRxBufInDesc[rxBuffer];

        if ((order == 0) && ((bufferPointer & ((1UL << PAGE_SHIFT)-1)) == 0))
        {
            free_pages(bufferPointer, order);
            edrvInstance_l.pageAllocations--;
        }
    }

    if (edrvInstance_l.rxBufFreeTop < EDRV_MAX_RX_BUFFERS-EDRV_MAX_RX_DESCS - 1)
    {
        printk("%s %d rx-buffers were lost\n", __func__, edrvInstance_l.rxBufFreeTop);
    }

    for (; edrvInstance_l.rxBufFreeTop >= 0; edrvInstance_l.rxBufFreeTop--)
    {
        bufferPointer = (ULONG)edrvInstance_l.apRxBufFree[edrvInstance_l.rxBufFreeTop];

        if ((order == 0) && ((bufferPointer & ((1UL << PAGE_SHIFT)-1)) == 0))
        {
            free_pages(bufferPointer, order);
            edrvInstance_l.pageAllocations--;
        }
    }

    if (edrvInstance_l.pageAllocations > 0)
    {
        printk("%s Less pages freed than allocated (%d)\n", __func__, edrvInstance_l.pageAllocations);
    }
    else if (edrvInstance_l.pageAllocations < 0)
    {
        printk("%s Attempted to free more pages than allocated (%d)\n", __func__, (edrvInstance_l.pageAllocations * -1));
    }

    if (edrvInstance_l.pRxDesc != NULL)
    {
        pci_free_consistent(pPciDev_p, EDRV_RX_DESCS_SIZE,
                            edrvInstance_l.pRxDesc, edrvInstance_l.pRxDescDma);
        edrvInstance_l.pRxDesc = NULL;
    }

    // unmap controller's register space
    if (edrvInstance_l.pIoAddr != NULL)
    {
        iounmap(edrvInstance_l.pIoAddr);
        edrvInstance_l.pIoAddr = NULL;
    }

    // disable the PCI device
    pci_disable_device(pPciDev_p);

    // release memory regions
    pci_release_regions(pPciDev_p);

    edrvInstance_l.pPciDev = NULL;

Exit:
    return;
}

/// \}
