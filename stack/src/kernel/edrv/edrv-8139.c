/**
********************************************************************************
\file   edrv-8139.c

\brief  Implementation of Ethernet driver for Realtek RTL8139

This file contains the implementation of the Ethernet driver for
Realtek RTL8139 chips (revision C, C+, D).

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

#define EDRV_MAX_TX_DESCS       4
#define EDRV_TX_DESC_MASK       (EDRV_MAX_TX_DESCS-1)

#define EDRV_MAX_FRAME_SIZE     0x0600

#define EDRV_RX_BUFFER_SIZE     0x8610  // 32 kB + 16 Byte + 1,5 kB (WRAP is enabled)
#define EDRV_RX_BUFFER_LENGTH   (EDRV_RX_BUFFER_SIZE & 0xF800)  // buffer size cut down to 2 kB alignment

#define EDRV_TX_BUFFER_SIZE     (EDRV_MAX_TX_BUFFERS * EDRV_MAX_FRAME_SIZE) // n * (MTU + 14 + 4)

#define DRV_NAME                "plk"

#define EDRV_HASH_BITS          6       // used bits in hash
#define EDRV_CRC32_POLY         0x04C11DB6

#define EDRV_REGW_INT_MASK      0x3C    // interrupt mask register
#define EDRV_REGW_INT_STATUS    0x3E    // interrupt status register
#define EDRV_REGW_INT_ROK       0x0001  // Receive OK interrupt
#define EDRV_REGW_INT_RER       0x0002  // Receive error interrupt
#define EDRV_REGW_INT_TOK       0x0004  // Transmit OK interrupt
#define EDRV_REGW_INT_TER       0x0008  // Transmit error interrupt
#define EDRV_REGW_INT_RXOVW     0x0010  // Rx buffer overflow interrupt
#define EDRV_REGW_INT_PUN       0x0020  // Packet underrun/ link change interrupt
#define EDRV_REGW_INT_FOVW      0x0040  // Rx FIFO overflow interrupt
#define EDRV_REGW_INT_LENCHG    0x2000  // Cable length change interrupt
#define EDRV_REGW_INT_TIMEOUT   0x4000  // Time out interrupt
#define EDRV_REGW_INT_SERR      0x8000  // System error interrupt
#define EDRV_REGW_INT_MASK_DEF  (EDRV_REGW_INT_ROK |     \
                                 EDRV_REGW_INT_RER |     \
                                 EDRV_REGW_INT_TOK |     \
                                 EDRV_REGW_INT_TER |     \
                                 EDRV_REGW_INT_RXOVW |   \
                                 EDRV_REGW_INT_FOVW |    \
                                 EDRV_REGW_INT_PUN |     \
                                 EDRV_REGW_INT_TIMEOUT | \
                                 EDRV_REGW_INT_SERR)   // default interrupt mask

#define EDRV_REGB_COMMAND       0x37    // command register
#define EDRV_REGB_COMMAND_RST   0x10
#define EDRV_REGB_COMMAND_RE    0x08
#define EDRV_REGB_COMMAND_TE    0x04
#define EDRV_REGB_COMMAND_BUFE  0x01

#define EDRV_REGB_CMD9346       0x50        // 93C46 command register
#define EDRV_REGB_CMD9346_LOCK  0x00        // lock configuration registers
#define EDRV_REGB_CMD9346_UNLOCK 0xC0       // unlock configuration registers

#define EDRV_REGDW_RCR          0x44        // Rx configuration register
#define EDRV_REGDW_RCR_NO_FTH   0x0000E000  // no receive FIFO threshold
#define EDRV_REGDW_RCR_RBLEN32K 0x00001000  // 32 kB receive buffer
#define EDRV_REGDW_RCR_MXDMAUNL 0x00000700  // unlimited maximum DMA burst size
#define EDRV_REGDW_RCR_NOWRAP   0x00000080  // do not wrap frame at end of buffer
#define EDRV_REGDW_RCR_AER      0x00000020  // accept error frames (CRC, alignment, collided)
#define EDRV_REGDW_RCR_AR       0x00000010  // accept runt
#define EDRV_REGDW_RCR_AB       0x00000008  // accept broadcast frames
#define EDRV_REGDW_RCR_AM       0x00000004  // accept multicast frames
#define EDRV_REGDW_RCR_APM      0x00000002  // accept physical match frames
#define EDRV_REGDW_RCR_AAP      0x00000001  // accept all frames
#define EDRV_REGDW_RCR_DEF      (EDRV_REGDW_RCR_NO_FTH |   \
                                 EDRV_REGDW_RCR_RBLEN32K | \
                                 EDRV_REGDW_RCR_MXDMAUNL | \
                                 EDRV_REGDW_RCR_NOWRAP |   \
                                 EDRV_REGDW_RCR_AB |       \
                                 EDRV_REGDW_RCR_AM |       \
                                 EDRV_REGDW_RCR_AAP |           /* promiscuous mode */ \
                                 EDRV_REGDW_RCR_APM)            // default value

#define EDRV_REGDW_TCR          0x40        // Tx configuration register
#define EDRV_REGDW_TCR_VER_MASK 0x7CC00000  // mask for hardware version
#define EDRV_REGDW_TCR_VER_C    0x74000000  // RTL8139C
#define EDRV_REGDW_TCR_VER_CP   0x74800000  // RTL8139C+
#define EDRV_REGDW_TCR_VER_D    0x74400000  // RTL8139D
#define EDRV_REGDW_TCR_VER_B    0x78000000  // RTL8139B
#define EDRV_REGDW_TCR_IFG96    0x03000000  // default interframe gap (960 ns)
#define EDRV_REGDW_TCR_CRC      0x00010000  // disable appending of CRC by the controller
#define EDRV_REGDW_TCR_MXDMAUNL 0x00000700  // maximum DMA burst size of 2048 b
#define EDRV_REGDW_TCR_TXRETRY  0x00000000  // 16 retries
#define EDRV_REGDW_TCR_DEF      (EDRV_REGDW_TCR_IFG96 |    \
                                 EDRV_REGDW_TCR_MXDMAUNL | \
                                 EDRV_REGDW_TCR_TXRETRY)

#define EDRV_REGW_MULINT        0x5C        // multiple interrupt select register

#define EDRV_REGDW_MPC          0x4C        // missed packet counter register

#define EDRV_REGDW_TSAD0        0x20        // Transmit start address of descriptor 0
#define EDRV_REGDW_TSAD1        0x24        // Transmit start address of descriptor 1
#define EDRV_REGDW_TSAD2        0x28        // Transmit start address of descriptor 2
#define EDRV_REGDW_TSAD3        0x2C        // Transmit start address of descriptor 3
#define EDRV_REGDW_TSAD(n)      (EDRV_REGDW_TSAD0 + 4*n)   // Transmit start address of descriptor n

#define EDRV_REGDW_TSD0         0x10        // Transmit status of descriptor 0
#define EDRV_REGDW_TSD1         0x14        // Transmit status of descriptor 1
#define EDRV_REGDW_TSD2         0x18        // Transmit status of descriptor 2
#define EDRV_REGDW_TSD3         0x1C        // Transmit status of descriptor 3
#define EDRV_REGDW_TSD(n)       (EDRV_REGDW_TSD0 + 4*n)    // Transmit status of descriptor n

#define EDRV_REGDW_TSD_CRS      0x80000000  // Carrier sense lost
#define EDRV_REGDW_TSD_TABT     0x40000000  // Transmit Abort
#define EDRV_REGDW_TSD_OWC      0x20000000  // Out of window collision
#define EDRV_REGDW_TSD_TXTH_DEF 0x00020000  // Transmit FIFO threshold of 64 bytes
#define EDRV_REGDW_TSD_TOK      0x00008000  // Transmit OK
#define EDRV_REGDW_TSD_TUN      0x00004000  // Transmit FIFO underrun
#define EDRV_REGDW_TSD_OWN      0x00002000  // Owner

#define EDRV_REGDW_RBSTART      0x30        // Receive buffer start address

#define EDRV_REGW_CAPR          0x38        // Current address of packet read

#define EDRV_REGDW_IDR0         0x00        // ID register 0
#define EDRV_REGDW_IDR4         0x04        // ID register 4

#define EDRV_REGDW_MAR0         0x08        // Multicast address register 0
#define EDRV_REGDW_MAR4         0x0C        // Multicast address register 4


// defines for the status word in the receive buffer
#define EDRV_RXSTAT_MAR         0x8000      // Multicast address received
#define EDRV_RXSTAT_PAM         0x4000      // Physical address matched
#define EDRV_RXSTAT_BAR         0x2000      // Broadcast address received
#define EDRV_RXSTAT_ISE         0x0020      // Invalid symbol error
#define EDRV_RXSTAT_RUNT        0x0010      // Runt packet received
#define EDRV_RXSTAT_LONG        0x0008      // Long packet
#define EDRV_RXSTAT_CRC         0x0004      // CRC error
#define EDRV_RXSTAT_FAE         0x0002      // Frame alignment error
#define EDRV_RXSTAT_ROK         0x0001      // Receive OK


#define EDRV_REGDW_WRITE(reg, val)      writel((val), (UINT8*)edrvInstance_l.pIoAddr + (reg))
#define EDRV_REGW_WRITE(reg, val)       writew((val), (UINT8*)edrvInstance_l.pIoAddr + (reg))
#define EDRV_REGB_WRITE(reg, val)       writeb((val), (UINT8*)edrvInstance_l.pIoAddr + (reg))
#define EDRV_REGDW_READ(reg)            readl((UINT8*)edrvInstance_l.pIoAddr + (reg))
#define EDRV_REGW_READ(reg)             readw((UINT8*)edrvInstance_l.pIoAddr + (reg))
#define EDRV_REGB_READ(reg)             readb((UINT8*)edrvInstance_l.pIoAddr + (reg))


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
#define EDRV_COUNT_TX_ERR               TGT_DBG_SIGNAL_TRACE_POINT(13)
#define EDRV_COUNT_RX_CRC               TGT_DBG_SIGNAL_TRACE_POINT(14)
#define EDRV_COUNT_RX_ERR               TGT_DBG_SIGNAL_TRACE_POINT(15)
#define EDRV_COUNT_RX_FOVW              TGT_DBG_SIGNAL_TRACE_POINT(16)
#define EDRV_COUNT_RX_PUN               TGT_DBG_SIGNAL_TRACE_POINT(17)
#define EDRV_COUNT_RX_FAE               TGT_DBG_SIGNAL_TRACE_POINT(18)
#define EDRV_COUNT_RX_OVW               TGT_DBG_SIGNAL_TRACE_POINT(19)


//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------
/**
\brief Structure describing an instance of the Edrv

This structure describes an instance of the Ethernet driver.
*/
typedef struct
{
    tEdrvInitParam      initParam;                          ///< Init parameters
    struct pci_dev*     pPciDev;                            ///< Pointer to the PCI device structure
    void*               pIoAddr;                            ///< Pointer to the register space of the Ethernet controller
    void*               pRxBuf;                             ///< Pointer to the RX buffer
    dma_addr_t          pRxBufDma;                          ///< Pointer to the DMA of the RX buffer
    void*               pTxBuf;                             ///< Pointer to the TX buffer
    dma_addr_t          pTxBufDma;                          ///< Pointer to the DMA of the TX buffer
    BOOL                afTxBufUsed[EDRV_MAX_TX_BUFFERS];   ///< Array describing whether a TX buffer is used
    tEdrvTxBuffer*      apTxBuffer[EDRV_MAX_TX_DESCS];      ///< Array of TX buffers
    spinlock_t          txSpinlock;                         ///< Spinlock to protect critical sections
    UINT                headTxDesc;                         ///< Index of the head of the TX descriptor buffer
    UINT                tailTxDesc;                         ///< Index of the tail of the TX descriptor buffer
} tEdrvInstance;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static irqreturn_t edrvIrqHandler(int irqNum_p, void* pDevInstData_p);
static void        reinitRx(void);
static int         initOnePciDev(struct pci_dev* pPciDev_p, const struct pci_device_id* pId_p);
static void        removeOnePciDev(struct pci_dev* pPciDev_p);
static UINT8       calcHash(const UINT8* pMacAddr_p);

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
// buffers and buffer descriptors and pointers
static struct pci_device_id aEdrvPciTbl_l[] =
{
    {0x10ec, 0x8139, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0},
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
    // 2008-11-24 d.k. because pci_unregister_driver() doesn't do it correctly;
    //      one example: kobject_set_name() frees edrvDriver_l.driver.kobj.name,
    //      but does not set this pointer to NULL.
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

    // init and fill buffer allocation instance
    if ((pBufAlloc_l = bufalloc_init(EDRV_MAX_TX_BUFFERS)) == NULL)
    {
        ret = kErrorNoResource;
        goto Exit;
    }

    for (i = 0; i < EDRV_MAX_TX_BUFFERS; i++)
    {
        bufData.bufferNumber = i;
        bufData.pBuffer = edrvInstance_l.pTxBuf + (i * EDRV_MAX_FRAME_SIZE);

        bufalloc_addBuffer(pBufAlloc_l, &bufData);
    }

    if (edrvInstance_l.pPciDev == NULL)
    {
        printk("%s pPciDev=NULL\n", __func__);
        edrv_exit();
        ret = kErrorNoResource;
        goto Exit;
    }

    // read MAC address from controller
    printk("%s local MAC = ", __func__);
    for (i = 0; i < 6; i++)
    {
        edrvInstance_l.initParam.aMacAddr[i] = EDRV_REGB_READ(EDRV_REGDW_IDR0 + i);
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
    UINT8       hash;

    // Check parameter validity
    ASSERT(pMacAddr_p != NULL);

    hash = calcHash(pMacAddr_p);

    if (hash > 31)
    {
        data = EDRV_REGDW_READ(EDRV_REGDW_MAR4);
        data |= 1 << (hash - 32);
        EDRV_REGDW_WRITE(EDRV_REGDW_MAR4, data);
    }
    else
    {
        data = EDRV_REGDW_READ(EDRV_REGDW_MAR0);
        data |= 1 << hash;
        EDRV_REGDW_WRITE(EDRV_REGDW_MAR0, data);
    }

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
    UINT8       hash;

    // Check parameter validity
    ASSERT(pMacAddr_p != NULL);

    hash = calcHash(pMacAddr_p);

    if (hash > 31)
    {
        data = EDRV_REGDW_READ(EDRV_REGDW_MAR4);
        data &= ~(1 << (hash - 32));
        EDRV_REGDW_WRITE(EDRV_REGDW_MAR4, data);
    }
    else
    {
        data = EDRV_REGDW_READ(EDRV_REGDW_MAR0);
        data &= ~(1 << hash);
        EDRV_REGDW_WRITE(EDRV_REGDW_MAR0, data);
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
    tOplkError  ret = kErrorOk;
    UINT        bufferNumber;
    UINT32      temp;
    ULONG       flags;

    // Check parameter validity
    ASSERT(pBuffer_p != NULL);

    bufferNumber = pBuffer_p->txBufferNumber.value;

    if (pBuffer_p->pBuffer == NULL)
    {
        ret = kErrorEdrvBufNotExisting;
        goto Exit;
    }

    if ((bufferNumber >= EDRV_MAX_TX_BUFFERS) ||
        (edrvInstance_l.afTxBufUsed[bufferNumber] == FALSE))
    {
        ret = kErrorEdrvBufNotExisting;
        goto Exit;
    }

    // array of pointers to tx buffers in queue is checked
    // because all four tx descriptors should be used
    if (edrvInstance_l.apTxBuffer[edrvInstance_l.tailTxDesc] != NULL)
    {
        ret = kErrorEdrvNoFreeTxDesc;
        goto Exit;
    }

    EDRV_COUNT_SEND;

    // pad with zeros if necessary, because controller does not do it
    if (pBuffer_p->txFrameSize < EDRV_MIN_ETH_SIZE)
    {
        OPLK_MEMSET((UINT8*)pBuffer_p->pBuffer + pBuffer_p->txFrameSize, 0, EDRV_MIN_ETH_SIZE - pBuffer_p->txFrameSize);
        pBuffer_p->txFrameSize = EDRV_MIN_ETH_SIZE;
    }

    spin_lock_irqsave(&edrvInstance_l.txSpinlock, flags);

    // save pointer to buffer structure for TxHandler
    edrvInstance_l.apTxBuffer[edrvInstance_l.tailTxDesc] = pBuffer_p;

    // set DMA address of buffer
    EDRV_REGDW_WRITE(EDRV_REGDW_TSAD(edrvInstance_l.tailTxDesc), edrvInstance_l.pTxBufDma + (bufferNumber * EDRV_MAX_FRAME_SIZE));
    temp = EDRV_REGDW_READ(EDRV_REGDW_TSAD(edrvInstance_l.tailTxDesc));

    // start transmission
    EDRV_REGDW_WRITE(EDRV_REGDW_TSD(edrvInstance_l.tailTxDesc), EDRV_REGDW_TSD_TXTH_DEF | pBuffer_p->txFrameSize);
    temp = EDRV_REGDW_READ(EDRV_REGDW_TSD(edrvInstance_l.tailTxDesc));

    // increment tx queue tail
    edrvInstance_l.tailTxDesc = (edrvInstance_l.tailTxDesc + 1) & EDRV_TX_DESC_MASK;

    spin_unlock_irqrestore(&edrvInstance_l.txSpinlock, flags);

Exit:
    return ret;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief  Interrupt service routine

This function is the interrupt service routine for the Ethernet driver.

\param[in]      irqNum_p            IRQ number
\param[in,out]  pDevInstData_p      Pointer to private data provided by request_irq

\return The function returns an IRQ handled code.
*/
//------------------------------------------------------------------------------
static irqreturn_t edrvIrqHandler(int irqNum_p, void* pDevInstData_p)
{
    tEdrvRxBuffer   rxBuffer;
    UINT16          status;
    UINT32          txStatus;
    UINT32          rxStatus;
    UINT16          curRx;
    void*           pRxBuffer;
    UINT            length;
    irqreturn_t     handled = IRQ_HANDLED;

    UNUSED_PARAMETER(irqNum_p);
    UNUSED_PARAMETER(pDevInstData_p);

    // read the interrupt status
    status = EDRV_REGW_READ(EDRV_REGW_INT_STATUS);

    // acknowledge the interrupts
    EDRV_REGW_WRITE(EDRV_REGW_INT_STATUS, status);

    if (status == 0)
    {
        handled = IRQ_NONE;
        goto Exit;
    }

    // process tasks
    if ((status & (EDRV_REGW_INT_TER | EDRV_REGW_INT_TOK)) != 0)
    {   // transmit interrupt
        tEdrvTxBuffer* pTxBuffer;

        if (edrvInstance_l.pTxBuf == NULL)
        {
            printk("%s Tx buffers currently not allocated\n", __func__);
            goto Exit;
        }

        spin_lock(&edrvInstance_l.txSpinlock);
        pTxBuffer = edrvInstance_l.apTxBuffer[edrvInstance_l.headTxDesc];
        spin_unlock(&edrvInstance_l.txSpinlock);

        // pointer to tx buffer in queue is checked
        // because all four tx descriptors should be used
        while (pTxBuffer != NULL)
        {
            // read transmit status
            txStatus = EDRV_REGDW_READ(EDRV_REGDW_TSD(edrvInstance_l.headTxDesc));
            if ((txStatus & (EDRV_REGDW_TSD_TOK | EDRV_REGDW_TSD_TABT | EDRV_REGDW_TSD_TUN)) != 0)
            {   // transmit finished
                edrvInstance_l.apTxBuffer[edrvInstance_l.headTxDesc] = NULL;

                // increment tx queue head
                edrvInstance_l.headTxDesc = (edrvInstance_l.headTxDesc + 1) & EDRV_TX_DESC_MASK;

                if ((txStatus & EDRV_REGDW_TSD_TOK) != 0)
                {
                    EDRV_COUNT_TX;
                }
                else if ((txStatus & EDRV_REGDW_TSD_TUN) != 0)
                {
                    EDRV_COUNT_TX_FUN;
                }
                else
                {   // assume EDRV_REGDW_TSD_TABT
                    EDRV_COUNT_TX_COL_RL;
                }

                // call Tx handler of Data link layer
                if (pTxBuffer->pfnTxHandler != NULL)
                {
                    pTxBuffer->pfnTxHandler(pTxBuffer);
                }

                spin_lock(&edrvInstance_l.txSpinlock);
                pTxBuffer = edrvInstance_l.apTxBuffer[edrvInstance_l.headTxDesc];
                spin_unlock(&edrvInstance_l.txSpinlock);
            }
            else
            {
                EDRV_COUNT_TX_ERR;
                break;
            }
        }
    }

    if ((status & (EDRV_REGW_INT_RER | EDRV_REGW_INT_FOVW | EDRV_REGW_INT_RXOVW | EDRV_REGW_INT_PUN)) != 0)
    {   // receive error interrupt

        if ((status & EDRV_REGW_INT_FOVW) != 0)
        {
            EDRV_COUNT_RX_FOVW;
        }
        else if ((status & EDRV_REGW_INT_RXOVW) != 0)
        {
            EDRV_COUNT_RX_OVW;
        }
        else if ((status & EDRV_REGW_INT_PUN) != 0)
        {   // Packet underrun
            EDRV_COUNT_RX_PUN;
        }
        else /*if ((status & EDRV_REGW_INT_RER) != 0)*/
        {
            EDRV_COUNT_RX_ERR;
        }

        // reinitialize Rx process
        reinitRx();
    }

    if ((status & EDRV_REGW_INT_ROK) != 0)
    {   // receive interrupt

        if (edrvInstance_l.pRxBuf == NULL)
        {
            printk("%s Rx buffers currently not allocated\n", __func__);
            goto Exit;
        }

        // read current offset in receive buffer
        curRx = (EDRV_REGW_READ(EDRV_REGW_CAPR) + 0x10) % EDRV_RX_BUFFER_LENGTH;

        while ((EDRV_REGB_READ(EDRV_REGB_COMMAND) & EDRV_REGB_COMMAND_BUFE) == 0)
        {   // frame available

            // calculate pointer to current frame in receive buffer
            pRxBuffer = (UINT8*)edrvInstance_l.pRxBuf + curRx;

            // read receive status UINT32
            rxStatus = le32_to_cpu(*((UINT32*)pRxBuffer));

            // calculate length of received frame
            length = rxStatus >> 16;

            if (length == 0xFFF0)
            {   // frame is unfinished (maybe early Rx interrupt is active)
                break;
            }

            if ((rxStatus & EDRV_RXSTAT_ROK) == 0)
            {   // error occurred while receiving this frame
                // ignore it
                if ((rxStatus & EDRV_RXSTAT_FAE) != 0)
                {
                    EDRV_COUNT_RX_FAE;
                }
                else if ((rxStatus & EDRV_RXSTAT_CRC) != 0)
                {
                    EDRV_COUNT_RX_CRC;
                }
                else
                {
                    EDRV_COUNT_RX_ERR;
                }

                // reinitialize Rx process
                reinitRx();
                break;
            }
            else
            {   // frame is OK
                rxBuffer.bufferInFrame = kEdrvBufferLastInFrame;
                rxBuffer.rxFrameSize = length - EDRV_ETH_CRC_SIZE;
                rxBuffer.pBuffer = (UINT8*)pRxBuffer + sizeof(rxStatus);

                EDRV_COUNT_RX;

                // call Rx handler of Data link layer
                edrvInstance_l.initParam.pfnRxHandler(&rxBuffer);
            }

            // calculate new offset (UINT32 aligned)
            curRx = (UINT16)((curRx + length + sizeof(rxStatus) + 3) & ~0x3);
            EDRV_REGW_WRITE(EDRV_REGW_CAPR, curRx - 0x10);

            // re-read current offset in receive buffer
            curRx = (EDRV_REGW_READ(EDRV_REGW_CAPR) + 0x10) % EDRV_RX_BUFFER_LENGTH;
        }
    }

    if ((status & EDRV_REGW_INT_SERR) != 0)
    {   // PCI error
        EDRV_COUNT_PCI_ERR;
    }

    if ((status & EDRV_REGW_INT_TIMEOUT) != 0)
    {   // Timeout
        EDRV_COUNT_TIMEOUT;
    }

Exit:
    return handled;
}

//------------------------------------------------------------------------------
/**
\brief  Reinitialize Rx process

This function reinitializes the Rx process because of an error.
*/
//------------------------------------------------------------------------------
static void reinitRx(void)
{
    UINT8 cmd;

    // simply switch off and on the receiver
    // this will reset the CAPR register
    cmd = EDRV_REGB_READ(EDRV_REGB_COMMAND);
    EDRV_REGB_WRITE(EDRV_REGB_COMMAND, cmd & ~EDRV_REGB_COMMAND_RE);
    EDRV_REGB_WRITE(EDRV_REGB_COMMAND, cmd);

    // set receive configuration register
    EDRV_REGDW_WRITE(EDRV_REGDW_RCR, EDRV_REGDW_RCR_DEF);
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
static int initOnePciDev(struct pci_dev* pPciDev_p,
                         const struct pci_device_id* pId_p)
{
    UINT    index;
    UINT32  temp;
    int     result = 0;

    if (edrvInstance_l.pPciDev != NULL)
    {   // Edrv is already connected to a PCI device
        printk("%s device %s discarded\n", __func__, pci_name(pPciDev_p));
        result = -ENODEV;
        goto Exit;
    }

    edrvInstance_l.pPciDev = pPciDev_p;

    // enable device
    printk("%s enable device\n", __func__);
    result = pci_enable_device(pPciDev_p);
    if (result != 0)
    {
        goto Exit;
    }

    if ((pci_resource_flags(pPciDev_p, 1) & IORESOURCE_MEM) == 0)
    {
        result = -ENODEV;
        goto Exit;
    }

    printk("%s request regions\n", __func__);
    result = pci_request_regions(pPciDev_p, DRV_NAME);
    if (result != 0)
    {
        goto Exit;
    }

    printk("%s ioremap\n", __func__);
    edrvInstance_l.pIoAddr = ioremap(pci_resource_start(pPciDev_p, 1), pci_resource_len(pPciDev_p, 1));
    if (edrvInstance_l.pIoAddr == NULL)
    {   // remap of controller's register space failed
        result = -EIO;
        goto Exit;
    }

    // enable PCI busmaster
    printk("%s enable busmaster\n", __func__);
    pci_set_master(pPciDev_p);

    // reset controller
    printk("%s reset controller\n", __func__);
    EDRV_REGB_WRITE(EDRV_REGB_COMMAND, EDRV_REGB_COMMAND_RST);

    // wait until reset has finished
    for (result = 500; result > 0; result--)
    {
        if ((EDRV_REGB_READ(EDRV_REGB_COMMAND) & EDRV_REGB_COMMAND_RST) == 0)
        {
            break;
        }

        schedule_timeout(10);
    }

    // check hardware version, i.e. chip ID
    temp = EDRV_REGDW_READ(EDRV_REGDW_TCR);
    if (((temp & EDRV_REGDW_TCR_VER_MASK) != EDRV_REGDW_TCR_VER_C) &&
        ((temp & EDRV_REGDW_TCR_VER_MASK) != EDRV_REGDW_TCR_VER_D) &&
        ((temp & EDRV_REGDW_TCR_VER_MASK) != EDRV_REGDW_TCR_VER_B) &&
        ((temp & EDRV_REGDW_TCR_VER_MASK) != EDRV_REGDW_TCR_VER_CP))
    {   // unsupported chip
        printk("%s Unsupported chip! TCR = 0x%08lX\n", __func__, (ULONG)temp);
        result = -ENODEV;
        goto Exit;
    }

    // initialize spinlock for tx data structures
    // It is required because the interrupt handler does not use the tx queue
    // head pointer when checking for transmitted frames. Instead, it uses the
    // array which stores the pointers to the tx buffers in the queue.
    // By this means it is possible to make use of all four tx descriptors.
    spin_lock_init(&edrvInstance_l.txSpinlock);

    // disable interrupts
    printk("%s disable interrupts\n", __func__);
    EDRV_REGW_WRITE(EDRV_REGW_INT_MASK, 0);
    // acknowledge all pending interrupts
    EDRV_REGW_WRITE(EDRV_REGW_INT_STATUS, EDRV_REGW_READ(EDRV_REGW_INT_STATUS));

    // install interrupt handler
    printk("%s install interrupt handler\n", __func__);
    result = request_irq(pPciDev_p->irq, edrvIrqHandler, IRQF_SHARED, DRV_NAME /*pPciDev_p->dev.name*/, pPciDev_p);
    if (result != 0)
    {
        goto Exit;
    }

    // allocate buffers
    printk("%s allocate buffers\n", __func__);
    edrvInstance_l.pTxBuf = pci_alloc_consistent(pPciDev_p, EDRV_TX_BUFFER_SIZE,
                                                 &edrvInstance_l.pTxBufDma);
    if (edrvInstance_l.pTxBuf == NULL)
    {
        result = -ENOMEM;
        goto Exit;
    }

    edrvInstance_l.pRxBuf = pci_alloc_consistent(pPciDev_p, EDRV_RX_BUFFER_SIZE,
                                                 &edrvInstance_l.pRxBufDma);
    if (edrvInstance_l.pRxBuf == NULL)
    {
        result = -ENOMEM;
        goto Exit;
    }

    // reset pointers for Tx buffers
    printk("%s reset pointers fo Tx buffers\n", __func__);
    for (index = 0; index < EDRV_MAX_TX_DESCS; index++)
    {
        EDRV_REGDW_WRITE(EDRV_REGDW_TSAD(index), 0);
        temp = EDRV_REGDW_READ(EDRV_REGDW_TSAD(index));
        edrvInstance_l.apTxBuffer[index] = NULL;
    }

    edrvInstance_l.headTxDesc = 0;
    edrvInstance_l.tailTxDesc = 0;

    printk("    Command = 0x%02X\n", (UINT16)EDRV_REGB_READ(EDRV_REGB_COMMAND));

    // set pointer for receive buffer in controller
    printk("%s set pointer to Rx buffer\n", __func__);
    EDRV_REGDW_WRITE(EDRV_REGDW_RBSTART, edrvInstance_l.pRxBufDma);

    // enable transmitter and receiver
    printk("%s enable Tx and Rx", __func__);
    EDRV_REGB_WRITE(EDRV_REGB_COMMAND, EDRV_REGB_COMMAND_RE | EDRV_REGB_COMMAND_TE);
    printk("  Command = 0x%02X\n", (UINT16)EDRV_REGB_READ(EDRV_REGB_COMMAND));

    // clear missed packet counter to enable Rx/Tx process
    EDRV_REGDW_WRITE(EDRV_REGDW_MPC, 0);

    // set transmit configuration register
    printk("%s set Tx conf register", __func__);
    EDRV_REGDW_WRITE(EDRV_REGDW_TCR, EDRV_REGDW_TCR_DEF);
    printk(" = 0x%08X\n", EDRV_REGDW_READ(EDRV_REGDW_TCR));

    // set receive configuration register
    printk("%s set Rx conf register", __func__);
    EDRV_REGDW_WRITE(EDRV_REGDW_RCR, EDRV_REGDW_RCR_DEF);
    printk(" = 0x%08X\n", EDRV_REGDW_READ(EDRV_REGDW_RCR));

    // reset multicast MAC address filter
    EDRV_REGDW_WRITE(EDRV_REGDW_MAR0, 0);
    temp = EDRV_REGDW_READ(EDRV_REGDW_MAR0);
    EDRV_REGDW_WRITE(EDRV_REGDW_MAR4, 0);
    temp = EDRV_REGDW_READ(EDRV_REGDW_MAR4);

    // disable early interrupts
    EDRV_REGW_WRITE(EDRV_REGW_MULINT, 0);

    // enable interrupts
    printk("%s enable interrupts\n", __func__);
    EDRV_REGW_WRITE(EDRV_REGW_INT_MASK, EDRV_REGW_INT_MASK_DEF);

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
    if (edrvInstance_l.pPciDev != pPciDev_p)
    {   // trying to remove unknown device
        BUG_ON(edrvInstance_l.pPciDev != pPciDev_p);
        return;
    }

    // disable transmitter and receiver
    EDRV_REGB_WRITE(EDRV_REGB_COMMAND, 0);

    // disable interrupts
    EDRV_REGW_WRITE(EDRV_REGW_INT_MASK, 0);

    // remove interrupt handler
    free_irq(pPciDev_p->irq, pPciDev_p);


    // free buffers
    if (edrvInstance_l.pTxBuf != NULL)
    {
        pci_free_consistent(pPciDev_p, EDRV_TX_BUFFER_SIZE,
                            edrvInstance_l.pTxBuf, edrvInstance_l.pTxBufDma);
        edrvInstance_l.pTxBuf = NULL;
    }

    if (edrvInstance_l.pRxBuf != NULL)
    {
        pci_free_consistent(pPciDev_p, EDRV_RX_BUFFER_SIZE,
                            edrvInstance_l.pRxBuf, edrvInstance_l.pRxBufDma);
        edrvInstance_l.pRxBuf = NULL;
    }

    // unmap controller's register space
    if (edrvInstance_l.pIoAddr != NULL)
    {
        iounmap(edrvInstance_l.pIoAddr);
    }

    // disable the PCI device
    pci_disable_device(pPciDev_p);

    // release memory regions
    pci_release_regions(pPciDev_p);

    edrvInstance_l.pPciDev = NULL;
}

//------------------------------------------------------------------------------
/**
\brief  Calculate MAC address hash

This function calculates the entry for the hash-table from MAC address.

\param[in]      pMacAddr_p          Pointer to MAC address

\return The function returns the calculated hash table.
*/
//------------------------------------------------------------------------------
static UINT8 calcHash(const UINT8* pMacAddr_p)
{
    UINT32       byteCounter;
    UINT32       bitCounter;
    UINT32       data;
    UINT32       crc;
    UINT32       carry;
    const UINT8* pData;
    UINT8        hash;

    pData = pMacAddr_p;

    // calculate crc32 value of mac address
    crc = 0xFFFFFFFF;

    for (byteCounter = 0; byteCounter < 6; byteCounter++)
    {
        data = *pData;
        pData++;
        for (bitCounter = 0; bitCounter < 8; bitCounter++, data >>= 1)
        {
            carry = (((crc >> 31) ^ data) & 1);
            crc = crc << 1;
            if (carry != 0)
            {
                crc = (crc ^ EDRV_CRC32_POLY) | carry;
            }
        }
    }

    // only upper 6 bits (EDRV_HASH_BITS) are used
    // which point to specific bit in the hash registers
    hash = (UINT8)((crc >> (32 - EDRV_HASH_BITS)) & 0x3f);

    return hash;
}

/// \}
