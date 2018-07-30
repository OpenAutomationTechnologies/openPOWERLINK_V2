/**
********************************************************************************
\file   edrv-emacps.c

\brief  Ethernet driver for Gigabit Ethernet Controller (GEM) on Xilinx Zynq

This file contains the implementation of the Ethernet driver for
the EMACPS Gigabit Ethernet Controller (GEM) on the Xilinx Zynq SoC.

\ingroup module_edrv
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2018, Kalycito Infotech Private Limited
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
#include <common/ami.h>
#include <kernel/edrv.h>

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/mm.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/of_address.h>
#include <linux/of_mdio.h>
#include <linux/phy.h>
#include <linux/mii.h>
#include <linux/clk.h>

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

// PHY specific defines
#define MARVELL_PHY_ADDR                    0x07                                        // Phy Id
#define PHY_CONTROL_REG_OFFSET              0x00                                        // Phy control register offset
#define PHY_STATUS_REG_OFFSET               0x01                                        // Phy status register offset
#define PHY_LINK_SPEED_100                  0x2000                                      // Link speed 100 MB/s
#define PHY_RESET                           0x8000                                      // Phy reset bit

// register Offsets
#define EDRV_NET_CNTRL_REG                  0x00                                        // network control register
#define EDRV_NET_CONFIG_REG                 0x04                                        // network configuration register
#define EDRV_NET_STATUS_REG                 0x08                                        // network status register
#define EDRV_TX_STATUS_REG                  0x14                                        // transmit status register
#define EDRV_RXQBASE_REG                    0x18                                        // receive queue base register
#define EDRV_RX_STATUS_REG                  0x20                                        // receive status register
#define EDRV_TXQBASE_REG                    0x1c                                        // transmit queue base register
#define EDRV_INTR_DIS_REG                   0x2C                                        // interrupt disable register
#define EDRV_INTR_EN_REG                    0x28                                        // interrupt enable register
#define EDRV_INTR_STATUS_REG                0x24                                        // interrupt status register
#define EDRV_INTR_MASK_REG                  0x30                                        // interrupt mask register
#define EDRV_HASHL_REG                      0x80                                        // MAC filtering register lower
#define EDRV_HASHH_REG                      0x84                                        // MAC filtering register higher
#define EDRV_LADDR1L_REG                    0x88                                        // MAC address register lower
#define EDRV_LADDR1H_REG                    0x8C                                        // MAC address register higher
#define EDRV_DMACFG_REG                     0x10                                        // DMA configuration register

// Network control register masks
#define EDRV_NWCTRL_TXEN_MASK               0x00000008                                  // transmit enable
#define EDRV_NWCTLR_RXEN_MASK               0x00000004                                  // receive enable
#define EDRV_NWCTLR_LOOPEN_MASK             0x00000002                                  // loopback enable
#define EDRV_NWCTLR_MDEN_MASK               0x00000010                                  // MDIO enable
#define EDRV_NWCTLR_STARTTX_MASK            0x00000200                                  // start transmit

// Network configuration register masks
#define EDRV_NWCFG_MDCCLK_SHIFT             18                                          // MDC clock shift
#define EDRV_NWCFG_100_MASK                 0x00000001                                  // config speed 100MB/s
#define EDRV_NWCFG_COPYALLEN_MASK           0x00000010                                  // config promiscuous receive mode
#define EDRV_NWCFG_FCSREMOVE_MASK           0x00020000                                  // remove frame check sequence from receive frame
#define EDRV_NWCFG_MULTICASTEN_MASK         0x00000040                                  // config multicast receive mode
#define EDRV_NWCFG_UNICASTEN_MASK           0x00000080                                  // config unicast receive mode

// Network Status register
#define EDRV_NWSR_MDIOIDLE_MASK             0x00000004                                  // MDIO idle mask to mark the end of a PHY transfer

// DMA configuration register
#define EDRV_DMA_BURST_AHBINCR16_MASK       0x00000010                                  // AHB INCR16
#define EDRV_DMA_BURST_MASK                 0x00000001                                  // SINGLE AHB burst
#define EDRV_DMA_BURST_AHBINCR4_MASK        0x00000004                                  // AHB INCR4 burst
#define EDRV_DMA_SWAP_MGMT_MASK             0x00000040                                  // Desc access
#define EDRV_DMA_SWAP_PKTEN_MASK            0x00000080                                  // Pkt buffer
#define EDRV_DMA_RXPKT_BUFSIZE_MASK         0x00000300                                  // pkt buffer-8kb
#define EDRV_DMA_TXPKT_BUFSIZE4_MASK        0x00000400                                  // pkt buffer-4kb
#define EDRV_DMA_TXPKT_BUFSIZE2_MASK        0x00000000                                  // pkt buffer-2kb
#define EDRV_DMA_RXBUFF_SIZE_MASK           0x00180000                                  // set Rx buff size as 1536 bytes

//Interrupt define
#define EDRV_TX_USED_BIT_READ               0x00000008                                  // transmit descriptor used bit read
#define EDRV_TX_COMPLETE_READ               0x00000080                                  // transmit complete
#define EDRV_RX_COMPLETE_READ               0x00000002                                  // receive complete
#define EDRV_TX_AHB_CORR_READ               0x00000040                                  // AHB buffer corruption occurred
#define EDRV_TX_BUFF_UNDRUN_READ            0x00000010                                  // transmit buffer under run
#define EDRV_TX_LATE_COL_READ               0x00000020                                  // late collision of transmit frams
#define EDRV_RX_OVERUN_READ                 0x00000400                                  // receive overrun
#define EDRV_RX_USED_BIT_READ               0x00000004                                  // receive descriptor used bit read

// Descriptor defines
#define EDRV_DESC_LAST_BUFF_MASK            (1 << 15)                                   // last buffer in the queue
#define EDRV_TX_DESC_WRAP_MASK              0x40000000                                  // wrap mask
#define EDRV_DESC_USED_BIT_MASK             0x80000000                                  // used bit mask
#define EDRV_DESC_AHB_ERROR_MASK            0x08000000                                  // AHB error mask
#define EDRV_DESC_LATE_COLL_MASK            0x04000000                                  // late collision of the frame
#define EDRV_RX_FRAME_LENGTH_MASK           0x00001FFF                                  // receive frame length mask
#define EDRV_TX_FRAME_LENGTH_MASK           0x00003FFF                                  // transmit frame length mask
#define EDRV_RX_FRAME_START_MASK            0x00004000                                  // receive start mask
#define EDRV_TX_FRAME_END_MASK              0x00008000                                  // transmit frame end mask
#define EDRV_RXBUF_CLEAR_USED_MASK          0xFFFFFFFE                                  // clear used bit
#define EDRV_RXBUF_WRAP_MASK                0x00000002                                  // receive wrap mask
#define EDRV_MAX_TX_DESCRIPTOR              128                                         // Max no of transmit desc in mem
#define EDRV_MAX_RX_DESCRIPTOR              128                                         // Max no of receive desc in mem

#define EDRV_MAX_TX_DESC_LEN                (EDRV_MAX_TX_DESCRIPTOR - 1)                //one slot to diff full
#define EDRV_MAX_RX_DESC_LEN                (EDRV_MAX_RX_DESCRIPTOR - 1)

#ifndef EDRV_MAX_TX_BUFFERS
#define EDRV_MAX_TX_BUFFERS                 128                                         // up-to 128 buffers are supported per frame
#endif

#ifndef EDRV_MAX_RX_BUFFERS
#define EDRV_MAX_RX_BUFFERS                 128
#endif

#define EDRV_MAX_FRAME_SIZE                 0x600                                       // 1536 bytes
#define EDRV_TX_BUFFER_SIZE                 (EDRV_MAX_TX_BUFFERS * EDRV_MAX_FRAME_SIZE) // n * (MTU + 14 + 4)
#define EDRV_RX_BUFFER_SIZE                 (EDRV_MAX_RX_BUFFERS * EDRV_MAX_FRAME_SIZE)

#define EDRV_TX_DESCS_SIZE                  (EDRV_MAX_TX_DESCRIPTOR * sizeof(tEdrvTxDesc))
#define EDRV_RX_DESCS_SIZE                  (EDRV_MAX_RX_DESCRIPTOR * sizeof(tEdrvRxDesc))

#define EDRV_FRAME_MIN_SIZE                 60

// multicast filtering through hash table for MAC addr
#define _MULTICASTEN_MODE_
//#define _PROMISCUOUS_MODE_

// PHY Maintenance defines
#define EDRV_PHYMNTNC_OFFSET                0x00000034  // PHY Maintenance register offset
#define EDRV_PHYMNTNC_OP_MASK               0x40020000  // clause_22 and must_10 bits mask
#define EDRV_PHYMNTNC_OP_READ               0x20000000  // Set Read bit
#define EDRV_PHYMNTNC_OP_WRITE              0x10000000  // Set Write Bit
#define EDRV_PHYMNTNC_PHYAD_SHIFT           23          // shift bits to form PHY address
#define EDRV_PHYMNTNC_PHYREG_SHIFT          18          // shift bits to form PHY register
#define EDRV_PHYMNTNC_DATA_MASK             0x0000FFFF  // data bits

#define EDRV_IXR_FRAMERX_MASK               0x00000002
#define EDRV_DESC_ADDR_OFFSET               0x00000000
#define EDRV_DESC_CNTRL_OFFSET              0x00000004

#define DRV_NAME                            "plk_edrv"

#define EDRV_READ_REG(offset)                       __raw_readl((UINT8*)edrvInstance_l.pIoAddr + offset)
#define EDRV_WRITE_REG(offset, val)                 __raw_writel(val, (UINT8*)edrvInstance_l.pIoAddr + offset)

//Reading and writing to descriptor memory
#define EDRV_DESC_WRITE(descBase, offset, val)      __raw_writel(val, ((u8*)descBase + offset))
#define EDRV_DESC_READ(descBase, offset)            __raw_readl((u8*)descBase + offset)

//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------

// MDC clock division - set according to pclk speed.
enum
{
    MDC_DIV_8 = 0,
    MDC_DIV_16,
    MDC_DIV_32,
    MDC_DIV_48,
    MDC_DIV_64,
    MDC_DIV_96,
    MDC_DIV_128,
    MDC_DIV_224
};

/**
\brief  Transmit descriptor

The structure provides information for Transmit descriptor used by
GEM device.
*/
typedef struct
{
    UINT32  bufferAddr;                     ///< Address of the transmit buffer
    UINT32  controlStatus;                  ///< Length of frame, status, CRC information
} tEdrvTxDesc;

/**
\brief  Receive descriptor

The structure provides information for Receive descriptor used by
GEM device.
*/
typedef struct
{
    UINT32  bufferAddr;                     ///< Address of the receive buffer
    UINT32  status;                         ///< Additional information fields
} tEdrvRxDesc;

/**
\brief  Edrv driver instance

Provides all the necessary information used by the edrv module
to interact with the device and interface with the stack above it
*/
typedef struct
{
    struct platform_device* pPlatformDev;   ///< Pointer to platform device structure for driver
    void*                   pIoAddr;        ///< Pointer to register space of Ethernet controller
    tEdrvInitParam          initParam;      ///< Init param passed to edrv
    resource_size_t         resMemAddr;     ///< Address of the memory allocated for device by OS
    resource_size_t         resMemSize;     ///< Size of the memory allocated for device
    dma_addr_t              rxDescDma;      ///< Physical address of receive descriptor
    dma_addr_t              txDescDma;      ///< Physical address of transmit descriptor
    UINT8                   aMacAddr[6];    ///< MAC address for the device
    BOOL                    afTxBufUsed[EDRV_MAX_TX_BUFFERS];       ///< Array to hold status of used transmit buffers
    tEdrvTxBuffer*          apTxBuffer[EDRV_MAX_TX_DESCRIPTOR];     ///< Array of pointers to the transmit buffers
    void*                   apRxBufInDesc[EDRV_MAX_RX_DESCRIPTOR];  ///< Array to hold the receive buffer pointers
    UINT32                  txDescHead;     ///< Transmit Descriptor head marker
    UINT32                  txDescTail;     ///< Transmit Descriptor tail marker
    void*                   pTxDescVirt;    ///< Virtual address of transmit descriptor
    void*                   pRxDescVirt;    ///< Virtual address of receive descriptor
    void*                   pTxBuffer;      ///< Pointer to transmit buffer area
    void*                   pRxBuffer;      ///< Pointer to receive buffer area
    tEdrvTxDesc*            pTxDescAddr;    ///< Pointer to transmit descriptors
    tEdrvRxDesc*            pRxDescAddr;    ///< Pointer to receive descriptors
    UINT32                  rxDescHead;     ///< Receive Descriptor head marker
    UINT32                  rxDescTail;     ///< Receive Descriptor tail marker
    UINT32                  resIrq;         ///< Interrupt Id
    struct clk*             devClk;         ///< Pointer to MAC reference clock
    struct clk*             ambaPerClk;     ///< Pointer to AMBA peripheral clock for MAC
} tEdrvInstance;

tEdrvInstance    edrvInstance_l;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static irqreturn_t  edrvIrqHandler(int irqNum_p, void* pDevInstData_p);
static int          initOnePlatformDev(struct platform_device* pDev_p);
static int          removeOnePlatformDev(struct platform_device* pDev_p);
static UINT32       getBitFromMac(const UINT8* pMac_p, UINT32 bitPos_p);
static UINT32       calculateHashAddr(const UINT8* pMac_p);
static void         mdioPhyWrite(int phyId_p, int regAddr_p, UINT16 value_p);
static UINT16       mdioPhyRead(int phyId_p, int regAddr_p);
//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------

// Platform device identification
#ifdef CONFIG_OF
static struct of_device_id          xemacps_of_match[] =
{
    {   .compatible = "cdns,zynq-gem", },               // __devinitdata creates warning!
    { /* end of table */}                               // keep devinit in separate data section,
                                                        // linker is not able to link
};
MODULE_DEVICE_TABLE(of, xemacps_of_match);
#else
#define xemacps_of_match    NULL
#endif /* CONFIG_OF */

static struct platform_driver       edrvDriver_l =
{
    .probe      = initOnePlatformDev,
    .remove     = removeOnePlatformDev,
    .suspend    = NULL,                                     // Not handling power management functions
    .resume     = NULL,                                     // Not handling power management functions
    .driver     = { .name = DRV_NAME,
                    .owner = THIS_MODULE,
                    .of_match_table = xemacps_of_match,     // This function is to check the device
                  },                                        // from the device tree
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
    tOplkError      ret = kErrorOk;
    int             result;
    int             loop;

    // Check parameter validity
    ASSERT(pEdrvInitParam_p != NULL);

    // clear instance structure
    OPLK_MEMSET(&edrvInstance_l, 0x00, sizeof(edrvInstance_l));

    // save the init data
    edrvInstance_l.initParam = *pEdrvInitParam_p;

    printk("(%s) Registering the driver to the kernel...", __func__);

    /*TODO:This function can be replaced with platform_driver_probe
     to reduce memory footprints */
    result = platform_driver_register(&edrvDriver_l);
    if (result != 0)
    {
        return kErrorNoResource;
    }
    printk("Done \n");
    // local MAC address might have been changed in initOnePlatformDev
    printk("Local MAC = ");
    for (loop = 0; loop < 6; loop++)
    {
        printk("0x%02x ", edrvInstance_l.initParam.aMacAddr[loop]);
    }
    printk("\n");

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
    printk("%s calling platform_driver_unregister()\n", __func__);
    platform_driver_unregister(&edrvDriver_l);

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
\brief  Set multicast address entry

This function sets a multicast entry into the Ethernet controller.

\param[in]      pMacAddr_p          Multicast address.

\return The function returns a tOplkError error code.

\ingroup module_edrv
*/
//------------------------------------------------------------------------------
tOplkError edrv_setRxMulticastMacAddr(const UINT8* pMacAddr_p)
{
    UINT32      hashValue;
    UINT32      hashRegLVal = 0;
    UINT32      hashRegHVal = 0;

    // Check parameter validity
    ASSERT(pMacAddr_p != NULL);

    // calculate the hash value to written in register
    hashValue = calculateHashAddr(pMacAddr_p);

    hashRegLVal = EDRV_READ_REG(EDRV_HASHL_REG);
    hashRegHVal = EDRV_READ_REG(EDRV_HASHH_REG);

    if (hashValue < 32)
    {
        hashRegLVal |= (1 << hashValue);
    }
    else
    {
        hashRegHVal |= (1 << (hashValue - 32));
    }

    EDRV_WRITE_REG(EDRV_HASHL_REG, hashRegLVal);
    EDRV_WRITE_REG(EDRV_HASHH_REG, hashRegHVal);

    return kErrorOk;
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
    UINT32      hashValue;
    UINT32      hashRegLVal = 0;
    UINT32      hashRegHVal = 0;

    // Check parameter validity
    ASSERT(pMacAddr_p != NULL);

    // calculate the hash value to be cleared
    hashValue = calculateHashAddr(pMacAddr_p);

    hashRegLVal = EDRV_READ_REG(EDRV_HASHL_REG);
    hashRegHVal = EDRV_READ_REG(EDRV_HASHH_REG);

    if (hashValue < 32)
    {
        hashRegLVal &= ~(1 << hashValue);
    }
    else
    {
        hashRegHVal &= ~(1 << (hashValue - 32));
    }

    EDRV_WRITE_REG(EDRV_HASHL_REG, hashRegLVal);
    EDRV_WRITE_REG(EDRV_HASHH_REG, hashRegHVal);

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
    int    channel;

    // Check parameter validity
    ASSERT(pBuffer_p != NULL);

    if (pBuffer_p->maxBufferSize > EDRV_MAX_FRAME_SIZE)
    {
        return kErrorEdrvNoFreeBufEntry;
    }

    if (edrvInstance_l.pTxBuffer == NULL)
    {
        printk("%s Tx buffers currently not allocated\n", __func__);
        return kErrorEdrvNoFreeBufEntry;
    }

    // search a free Tx buffer with appropriate size
    for (channel = 0; channel < EDRV_MAX_TX_BUFFERS; channel++)
    {
        if (edrvInstance_l.afTxBufUsed[channel] == FALSE)
        {
            // free channel found
            edrvInstance_l.afTxBufUsed[channel] = TRUE;
            pBuffer_p->txBufferNumber.value = channel;
            pBuffer_p->pBuffer = (UINT8*)edrvInstance_l.pTxBuffer +
                                 (channel * EDRV_MAX_FRAME_SIZE);
            pBuffer_p->maxBufferSize = EDRV_MAX_FRAME_SIZE;
            break;
        }
    }

    if (channel >= EDRV_MAX_TX_BUFFERS)
    {
        return kErrorEdrvNoFreeBufEntry;
    }

    return kErrorOk;
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
    UINT32    bufferNumber;

    // Check parameter validity
    ASSERT(pBuffer_p != NULL);

    bufferNumber = pBuffer_p->txBufferNumber.value;

    if (bufferNumber < EDRV_MAX_TX_BUFFERS)
    {
        edrvInstance_l.afTxBufUsed[bufferNumber] = FALSE;
    }

    return kErrorOk;
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
    UINT32          bufferNumber;
    tEdrvTxDesc*    pTxDesc;
    dma_addr_t      txDmaAddr;
    UINT32          reg;

    // Check parameter validity
    ASSERT(pBuffer_p != NULL);

    bufferNumber = pBuffer_p->txBufferNumber.value;

    if ((bufferNumber >= EDRV_MAX_TX_BUFFERS) || (edrvInstance_l.afTxBufUsed[bufferNumber] == FALSE))
    {
        return kErrorEdrvBufNotExisting;
    }

    // one descriptor has to be left empty for distinction between full and empty
    if (((edrvInstance_l.txDescTail + 1) & (EDRV_MAX_TX_DESC_LEN)) == edrvInstance_l.txDescHead)
    {
        return kErrorEdrvNoFreeTxDesc;
    }

    // saved for call back
    edrvInstance_l.apTxBuffer[edrvInstance_l.txDescTail] = pBuffer_p;

    // minimum size for a frame should be 60
    if (pBuffer_p->txFrameSize < EDRV_FRAME_MIN_SIZE)
    {
        pBuffer_p->txFrameSize = EDRV_FRAME_MIN_SIZE;
    }

    pTxDesc = &edrvInstance_l.pTxDescAddr[edrvInstance_l.txDescTail];
    txDmaAddr = dma_map_single(&edrvInstance_l.pPlatformDev->dev,
                               pBuffer_p->pBuffer,
                               pBuffer_p->txFrameSize,
                               DMA_TO_DEVICE);
    EDRV_DESC_WRITE(pTxDesc, EDRV_DESC_ADDR_OFFSET, txDmaAddr);
    // no re-ordering!
    wmb();
    reg = 0;
    reg = EDRV_DESC_READ(pTxDesc, EDRV_DESC_CNTRL_OFFSET);
    //clear the used bit
    reg &= ~EDRV_DESC_USED_BIT_MASK;
    reg &= ~EDRV_TX_FRAME_LENGTH_MASK;
    //last buffer in the frame
    reg |= (pBuffer_p->txFrameSize | EDRV_DESC_LAST_BUFF_MASK);
    EDRV_DESC_WRITE(pTxDesc, EDRV_DESC_CNTRL_OFFSET, reg);

    //no re-ordering!
    wmb();

    // hand the descriptor to device
    reg = 0;
    reg = EDRV_READ_REG(EDRV_NET_CNTRL_REG);
    reg |= EDRV_NWCTLR_STARTTX_MASK;
    EDRV_WRITE_REG(EDRV_NET_CNTRL_REG, reg);
    //scale len to size
    edrvInstance_l.txDescTail = (edrvInstance_l.txDescTail + 1) & (EDRV_MAX_TX_DESC_LEN);

    return kErrorOk;
}

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
    UINT32              isrStatus;
    UINT32              stat = 0;
    irqreturn_t         result = IRQ_HANDLED;
    tEdrvTxDesc*        pTxDesc;
    tEdrvTxBuffer*      pTxBuffer;
    tEdrvRxDesc*        pRxDesc;

    UNUSED_PARAMETER(irqNum_p);
    UNUSED_PARAMETER(pDevInstData_p);

    isrStatus = EDRV_READ_REG(EDRV_INTR_STATUS_REG);
    // not a shared handler, yet!
    if (isrStatus == 0)
    {
        result = IRQ_NONE;
        goto Exit;
    }
    else
    {
        // acknowledge interrupt
        EDRV_WRITE_REG(EDRV_INTR_STATUS_REG, isrStatus);
    }

    // process Rx with higher priority
    if (isrStatus & (EDRV_RX_COMPLETE_READ))
    {
        if (isrStatus & (EDRV_RX_OVERUN_READ | EDRV_RX_USED_BIT_READ))
        {
            //ERROR: Shd we abort?
            goto Exit;
        }

        // extract the descriptor on head
        pRxDesc = &edrvInstance_l.pRxDescAddr[edrvInstance_l.rxDescHead];

        while (pRxDesc->bufferAddr & ~EDRV_RXBUF_CLEAR_USED_MASK)
        {
            tEdrvRxBuffer       rxBuffer;
            UINT16              frameLen_le;
            UINT32              reg;

            reg = 0;
            reg = EDRV_DESC_READ(pRxDesc, EDRV_DESC_CNTRL_OFFSET);
            frameLen_le = reg & EDRV_RX_FRAME_LENGTH_MASK;
            // prepare the receive frame
            rxBuffer.rxFrameSize = ami_getUint16Le(&frameLen_le);
            rxBuffer.pBuffer = edrvInstance_l.apRxBufInDesc[edrvInstance_l.rxDescHead];
            dma_sync_single_for_cpu(&edrvInstance_l.pPlatformDev->dev,
                                    pRxDesc->bufferAddr,
                                    frameLen_le,
                                    DMA_FROM_DEVICE);

            edrvInstance_l.initParam.pfnRxHandler(&rxBuffer);

            // clear the status
            reg = 0;
            EDRV_DESC_WRITE(pRxDesc, EDRV_DESC_CNTRL_OFFSET, reg);

            // clear used bit
            reg = EDRV_DESC_READ(pRxDesc, EDRV_DESC_ADDR_OFFSET);
            reg &= EDRV_RXBUF_CLEAR_USED_MASK;

            if ((EDRV_MAX_RX_DESCRIPTOR - 1) == edrvInstance_l.rxDescHead)
            {
                reg |= EDRV_RXBUF_WRAP_MASK;
            }

            EDRV_DESC_WRITE(pRxDesc, EDRV_DESC_ADDR_OFFSET, reg);
            edrvInstance_l.rxDescHead = (edrvInstance_l.rxDescHead + 1) & EDRV_MAX_RX_DESC_LEN;
            pRxDesc = &edrvInstance_l.pRxDescAddr[edrvInstance_l.rxDescHead];
            if (edrvInstance_l.rxDescHead == 0)
            {
                edrvInstance_l.rxDescTail = EDRV_MAX_RX_DESC_LEN;
            }
            else
            {
                edrvInstance_l.rxDescTail = edrvInstance_l.rxDescHead - 1;
            }
        }
    }

    // process Tx
    if (isrStatus & (EDRV_TX_COMPLETE_READ))
    {
        do
        {
            pTxDesc = &edrvInstance_l.pTxDescAddr[edrvInstance_l.txDescHead];

            if (pTxDesc == NULL)
            {
                //TODO: is this necessary??
                break;
            }

            stat = EDRV_DESC_READ(pTxDesc, EDRV_DESC_CNTRL_OFFSET);
            if (!(stat & EDRV_DESC_USED_BIT_MASK))
            {
                break;
            }
            if (stat & EDRV_DESC_AHB_ERROR_MASK)
            {
                //ERROR: Shd we abort?
            }
            else if (stat & EDRV_DESC_LATE_COLL_MASK)
            {
                //ERROR: Late coll, prone in gigabit mode
            }
            else
            {
                pTxBuffer = edrvInstance_l.apTxBuffer[edrvInstance_l.txDescHead];
                if (pTxBuffer == NULL)
                {
                    break;
                }

                edrvInstance_l.apTxBuffer[edrvInstance_l.txDescHead] = NULL;
                edrvInstance_l.txDescHead = ((edrvInstance_l.txDescHead + 1) & EDRV_MAX_TX_DESC_LEN);
                dma_unmap_single(&edrvInstance_l.pPlatformDev->dev,
                                 pTxDesc->bufferAddr,
                                 pTxBuffer->txFrameSize,
                                 DMA_TO_DEVICE);

                // Call Tx handler of Data link layer
                if (pTxBuffer->pfnTxHandler != NULL)
                {
                    pTxBuffer->pfnTxHandler(pTxBuffer);
                }
            }
        } while (edrvInstance_l.txDescHead != edrvInstance_l.txDescTail);
    }

Exit:
    return result;
}

//------------------------------------------------------------------------------
/**
\brief  Initialize one Platform device

This function initializes one platform device.

\param[in,out]  pDev_p              Pointer to corresponding platform device structure

\return The function returns an integer error code.
\retval 0           Successful
\retval Otherwise   Error
*/
//------------------------------------------------------------------------------
static int initOnePlatformDev(struct platform_device* pDev_p)
{
    struct resource*    pResMem;
    struct resource*    pResIrq;
    int                 result = 0;
    int                 loop = 0;
    UINT32              reg = 0;
    LONG                rate;
    UINT32              macLAddr;
    UINT16              macHAddr;
    dma_addr_t          rxDmaAddr;
    UINT16              controlReg;

    if (pDev_p == NULL)
    {
        printk("%s device discarded\n", __func__);
        result = -ENODEV;
        goto Exit;
    }

    if (edrvInstance_l.pPlatformDev != NULL)
    {
        printk("%s device (%s) already registered\n", __func__, pDev_p->name);
        result = -ENODEV;
        goto Exit;
    }

    //create local instance
    edrvInstance_l.pPlatformDev = pDev_p;

    printk("(%s) IOMEM Resource initialization...", __func__);
    pResMem = platform_get_resource(pDev_p, IORESOURCE_MEM, 0);
    if (pResMem == NULL)
    {
        printk("Failed \n");
        result = -ENODEV;
        goto Exit;
    }
    printk("Done \n");

    printk("(%s) IRQ Resource initialization...", __func__);
    pResIrq = platform_get_resource(pDev_p, IORESOURCE_IRQ, 0);
    if (pResIrq == NULL)
    {
        printk("Failed \n");
        result = -ENODEV;
        goto Exit;
    }
    printk("Done \n");

    /* Local instance copy to clear mem */
    edrvInstance_l.resMemAddr = pResMem->start;
    edrvInstance_l.resMemSize = (pResMem->end - pResMem->start + 1);

    /* Obtain the region exclusively for Edrv*/
    if (!request_mem_region(edrvInstance_l.resMemAddr,
                            edrvInstance_l.resMemSize, DRV_NAME))
    {
        printk("Req mem region failed \n");
        result = -ENOMEM;
        goto Exit;
    }
    printk("MEM_RESOURCE: Start 0x(%X), End 0x(%X) \n", pResMem->start,
           pResMem->end);

    /* Physical memory mapped to virtual memory*/
    edrvInstance_l.pIoAddr = ioremap(pResMem->start,
                                     (pResMem->end - pResMem->start + 1));
    if (edrvInstance_l.pIoAddr == NULL)
    {
        printk("Ioremap failed \n");
        result = -EIO;
        goto Exit;
    }

    // Request IRQ
    printk("Requesting IRQ resource ...");

    if (request_irq(pResIrq->start, edrvIrqHandler, IRQF_SHARED, pDev_p->name, pDev_p))
    {
        printk("Failed \n");
        result = -EIO;
        goto Exit;
    }
    edrvInstance_l.resIrq = pResIrq->start;
    printk("Done \n");

    // begin by clearing the registers ----------------->
    // disable Tx and Rx circuit
    EDRV_WRITE_REG(EDRV_NET_CNTRL_REG, 0x0);
    // disble interrupts
    EDRV_WRITE_REG(EDRV_INTR_DIS_REG, ~0x0);
    // clear Tx and Rx status registers
    EDRV_WRITE_REG(EDRV_TX_STATUS_REG, ~0x0);
    EDRV_WRITE_REG(EDRV_RX_STATUS_REG, ~0x0);
    // register clean done ----------------------------->

    // Initialize MAC clock
    edrvInstance_l.ambaPerClk = devm_clk_get(&pDev_p->dev, "pclk");
    if (edrvInstance_l.ambaPerClk == NULL)
    {
        printk("pclk clock not found.\n");
    }

    edrvInstance_l.devClk = devm_clk_get(&pDev_p->dev, "tx_clk");
    if (edrvInstance_l.devClk == NULL)
    {
        printk("tx_clk clock not found.\n");
    }

    result = clk_prepare_enable(edrvInstance_l.ambaPerClk);
    if (result)
    {
        printk("Unable to enable APER clock.\n");
    }

    result = clk_prepare_enable(edrvInstance_l.devClk);
    if (result)
    {
        printk("Unable to enable device clock.\n");
    }

    // Set the frequency of clock for 100MB/s
    rate = clk_round_rate(edrvInstance_l.devClk, 25000000);

    if ((result = clk_set_rate(edrvInstance_l.devClk, rate)))
    {
        printk("Setting new clock rate failed.\n");
    }

    // set MDC clock Division to be used
    reg = EDRV_READ_REG(EDRV_NET_CONFIG_REG);
    reg = 0;
    reg |= (MDC_DIV_224 << EDRV_NWCFG_MDCCLK_SHIFT);
    reg |= EDRV_NWCFG_100_MASK;    // set speed to 100Mbps

#if defined(_PROMISCUOUS_MODE_)
    reg |= EDRV_NWCFG_COPYALLEN_MASK;
#elif defined(_MULTICASTEN_MODE_)
    reg |= EDRV_NWCFG_MULTICASTEN_MASK;
#elif defined(__UNICASTEN_MODE_)
    reg |= EDRV_NWCFG_UNICASTEN_MASK;
#endif
    reg |= EDRV_NWCFG_FCSREMOVE_MASK;
    EDRV_WRITE_REG(EDRV_NET_CONFIG_REG, reg);

    // set the MAC address
    if ((edrvInstance_l.initParam.aMacAddr[0] != 0) |
        (edrvInstance_l.initParam.aMacAddr[1] != 0) |
        (edrvInstance_l.initParam.aMacAddr[2] != 0) |
        (edrvInstance_l.initParam.aMacAddr[3] != 0) |
        (edrvInstance_l.initParam.aMacAddr[4] != 0) |
        (edrvInstance_l.initParam.aMacAddr[5] != 0))
    {
        macLAddr = 0;
        macLAddr |= edrvInstance_l.initParam.aMacAddr[5];
        macLAddr |= edrvInstance_l.initParam.aMacAddr[4] << 8;
        macLAddr |= edrvInstance_l.initParam.aMacAddr[3] << 16;
        macLAddr |= edrvInstance_l.initParam.aMacAddr[2] << 24;
        EDRV_WRITE_REG(EDRV_LADDR1L_REG, macLAddr);
        macHAddr = 0;
        macHAddr |= edrvInstance_l.initParam.aMacAddr[1];
        macHAddr |= edrvInstance_l.initParam.aMacAddr[0] << 8;
        EDRV_WRITE_REG(EDRV_LADDR1H_REG, macHAddr);
    }
    else
    {
        macLAddr = 0;
        macLAddr = EDRV_READ_REG(EDRV_LADDR1L_REG);
        edrvInstance_l.initParam.aMacAddr[0] = (macLAddr & 0xFF);
        edrvInstance_l.initParam.aMacAddr[1] = ((macLAddr >> 8) & 0xFF);
        edrvInstance_l.initParam.aMacAddr[2] = ((macLAddr >> 16) & 0xFF);
        edrvInstance_l.initParam.aMacAddr[3] = ((macLAddr >> 24) & 0xFF);
        macHAddr = 0;
        macHAddr = EDRV_READ_REG(EDRV_LADDR1H_REG);
        edrvInstance_l.initParam.aMacAddr[4] = (macHAddr & 0xFF);
        edrvInstance_l.initParam.aMacAddr[5] = ((macHAddr >> 8) & 0xFF);
    }

    // allocate the Tx and Rx buffer
    /*For Buffer allocation should the memory be dma'ble
     Currently doing a kzalloc here*/
    // Zeros mem before returning pointer
    printk("Allocation of Tx Buffers ...");
    edrvInstance_l.pTxBuffer = kzalloc(EDRV_TX_BUFFER_SIZE, GFP_KERNEL);
    if (edrvInstance_l.pTxBuffer == NULL)
    {
        printk("Failed \n");
        result = -ENOMEM;
        goto Exit;
    }
    printk("Done \n");

    // allocate tx and rx buffer descriptors
    printk("Allocation of Tx Desc ...");
    edrvInstance_l.pTxDescVirt = dma_alloc_coherent(&pDev_p->dev,
                                                    EDRV_TX_DESCS_SIZE,
                                                    &edrvInstance_l.txDescDma,
                                                    GFP_KERNEL);
    if (edrvInstance_l.pTxDescVirt == NULL)
    {
        printk("Failed \n");
        result = -ENOMEM;
        goto Exit;
    }
    printk("Done \n");

    printk("Allocation of Rx Desc ...");
    edrvInstance_l.pRxDescVirt = dma_alloc_coherent(&pDev_p->dev,
                                                    EDRV_RX_DESCS_SIZE,
                                                    &edrvInstance_l.rxDescDma,
                                                    GFP_KERNEL);
    if (edrvInstance_l.pRxDescVirt == NULL)
    {
        printk("Failed \n");
        result = -ENOMEM;
        goto Exit;
    }
    printk("Done \n");
    edrvInstance_l.pRxDescAddr = (tEdrvRxDesc*)edrvInstance_l.pRxDescVirt;

    printk("Allocation of Rx Buffers ...");
    edrvInstance_l.pRxBuffer = kzalloc(EDRV_RX_BUFFER_SIZE, GFP_KERNEL);
    if (edrvInstance_l.pRxBuffer == NULL)
    {
        printk("Failed \n");
        result = -ENOMEM;
        goto Exit;
    }
    printk("Done \n");

    for (loop = 0; loop < EDRV_MAX_RX_DESCRIPTOR; loop++)
    {
        rxDmaAddr = dma_map_single(&pDev_p->dev,
                                   (UINT8*)edrvInstance_l.pRxBuffer + (loop * EDRV_MAX_FRAME_SIZE),
                                   EDRV_MAX_FRAME_SIZE,
                                   DMA_FROM_DEVICE);
        if (!rxDmaAddr)
        {
            printk("RxDmaAddr Allocation failed\n");
            result = -EIO;
            goto Exit;
        }
        edrvInstance_l.apRxBufInDesc[loop] = (UINT8*)edrvInstance_l.pRxBuffer +
                                              (loop * EDRV_MAX_FRAME_SIZE);
        edrvInstance_l.pRxDescAddr[loop].bufferAddr = rxDmaAddr;
        edrvInstance_l.pRxDescAddr[loop].bufferAddr &= EDRV_RXBUF_CLEAR_USED_MASK;

        if (loop == (EDRV_MAX_RX_DESCRIPTOR - 1))
        {
            edrvInstance_l.pRxDescAddr[loop].bufferAddr |= EDRV_RXBUF_WRAP_MASK;
        }
    }

    // set Dma parameters
    reg = 0;
    reg |= EDRV_DMA_BURST_MASK;
    reg |= EDRV_DMA_RXPKT_BUFSIZE_MASK;
    reg |= EDRV_DMA_TXPKT_BUFSIZE2_MASK;
    reg |= EDRV_DMA_RXBUFF_SIZE_MASK;

    EDRV_WRITE_REG(EDRV_DMACFG_REG, reg);

    reg = 0;
    reg = EDRV_READ_REG(EDRV_NET_CNTRL_REG);
    reg |= EDRV_NWCTRL_TXEN_MASK; // Enable TX
    EDRV_WRITE_REG(EDRV_NET_CNTRL_REG, reg);

    // set the TxQptr
    EDRV_WRITE_REG(EDRV_TXQBASE_REG, edrvInstance_l.txDescDma);
    // set the RxQptr
    EDRV_WRITE_REG(EDRV_RXQBASE_REG, edrvInstance_l.rxDescDma);

    edrvInstance_l.pTxDescAddr = (tEdrvTxDesc*)edrvInstance_l.pTxDescVirt;

    for (loop = 0; loop < EDRV_MAX_TX_DESCRIPTOR; loop++)
    {
        // set the used bit
        EDRV_DESC_WRITE(&edrvInstance_l.pTxDescAddr[loop],
                        EDRV_DESC_CNTRL_OFFSET,
                        EDRV_DESC_USED_BIT_MASK);
        if (loop == (EDRV_MAX_TX_DESCRIPTOR - 1))
        {
            // Wrap set for last desc
            EDRV_DESC_WRITE(&edrvInstance_l.pTxDescAddr[loop],
                            EDRV_DESC_CNTRL_OFFSET,
                            EDRV_TX_DESC_WRAP_MASK);
        }
    }
    // enable MDIO port
    reg = 0;
    reg = EDRV_READ_REG(EDRV_NET_CNTRL_REG);
    reg |= EDRV_NWCTLR_MDEN_MASK;
    EDRV_WRITE_REG(EDRV_NET_CNTRL_REG, reg);

    // Initialize the Phy
    mdioPhyWrite(MARVELL_PHY_ADDR,
                 PHY_CONTROL_REG_OFFSET,
                 PHY_LINK_SPEED_100);
    controlReg = mdioPhyRead(MARVELL_PHY_ADDR, PHY_CONTROL_REG_OFFSET);
    controlReg |= PHY_RESET;
    mdioPhyWrite(MARVELL_PHY_ADDR, PHY_CONTROL_REG_OFFSET, controlReg);

    msleep(10);

    // Enable Rx Circuit
    reg = 0;
    reg = EDRV_READ_REG(EDRV_NET_CNTRL_REG);
    reg |= EDRV_NWCTLR_RXEN_MASK;
    EDRV_WRITE_REG(EDRV_NET_CNTRL_REG, reg);

    //enable Tx_used_bit read INTR
    EDRV_WRITE_REG(EDRV_INTR_EN_REG,
                   (
                     EDRV_TX_AHB_CORR_READ |
                     EDRV_RX_COMPLETE_READ |
                     EDRV_TX_COMPLETE_READ |
                     EDRV_TX_BUFF_UNDRUN_READ |
                     EDRV_TX_LATE_COL_READ |
                     EDRV_RX_OVERUN_READ
                   ));

Exit:
    printk("%s finished with %d\n", __func__, result);
    return result;
}

//------------------------------------------------------------------------------
/**
\brief  Remove one platform device

This function removes one platform device.

\param[in,out]  pDev_p              Pointer to corresponding platform device structure

\return The function returns an integer error code.
\retval 0           Successful
*/
//------------------------------------------------------------------------------
static int removeOnePlatformDev(struct platform_device* pDev_p)
{
    int    loop;

    // disble interrupts
    EDRV_WRITE_REG(EDRV_INTR_DIS_REG, ~0x0);
    // disable Tx and Rx circuit
    EDRV_WRITE_REG(EDRV_NET_CNTRL_REG, 0x0);

    if (pDev_p != edrvInstance_l.pPlatformDev)
    {
        BUG_ON(edrvInstance_l.pPlatformDev != pDev_p);
        return 0;
    }

    if (edrvInstance_l.pTxBuffer != NULL)
    {
        kfree(edrvInstance_l.pTxBuffer);
        edrvInstance_l.pTxBuffer = NULL;
    }
    if (edrvInstance_l.pRxBuffer != NULL)
    {
        kfree(edrvInstance_l.pRxBuffer);
        edrvInstance_l.pRxBuffer = NULL;
    }
    if (edrvInstance_l.pTxDescVirt != NULL)
    {
        dma_free_coherent(&pDev_p->dev, EDRV_TX_DESCS_SIZE,
                          edrvInstance_l.pTxDescVirt,
                          edrvInstance_l.txDescDma);
        edrvInstance_l.pTxDescVirt = NULL;
    }

    for (loop = 0; loop < EDRV_MAX_RX_DESCRIPTOR; loop++)
    {
        dma_unmap_single(&pDev_p->dev,
                         edrvInstance_l.pRxDescAddr[loop].bufferAddr,
                         EDRV_MAX_FRAME_SIZE,
                         DMA_FROM_DEVICE);
    }

    if (edrvInstance_l.pRxDescVirt != NULL)
    {
        dma_free_coherent(&pDev_p->dev, EDRV_TX_DESCS_SIZE,
                          edrvInstance_l.pRxDescVirt,
                          edrvInstance_l.rxDescDma);

        edrvInstance_l.pRxDescVirt = NULL;
    }

    clk_disable_unprepare(edrvInstance_l.devClk);
    clk_disable_unprepare(edrvInstance_l.ambaPerClk);

    free_irq(edrvInstance_l.resIrq, pDev_p);
    release_mem_region(edrvInstance_l.resMemAddr, edrvInstance_l.resMemSize);

    if (edrvInstance_l.pIoAddr != NULL)
    {
        iounmap(edrvInstance_l.pIoAddr);
        edrvInstance_l.pIoAddr = NULL;
    }

    return 0;
}

//------------------------------------------------------------------------------
/**
\brief  Extract a bit from the given MAC address

This is a helper routine to calculate hash index for a given MAC
address. It extracts the value for a specific bit specified by position.

\param[in]      pMac_p              Pointer to MAC address
\param[in]      bitPos_p            Bit position to extract

\return Returns the value at the specified bit position
*/
//------------------------------------------------------------------------------
static UINT32 getBitFromMac(const UINT8* pMac_p, UINT32 bitPos_p)
{
    UINT8       macAdd;
    UINT32      bitVal;

    macAdd = (*(pMac_p + (bitPos_p / 8)));
    bitVal = ((macAdd >> (bitPos_p & 0x07)) & 0x01);

    return bitVal;
}

//------------------------------------------------------------------------------
/**
\brief  Calculate hash value

This routine calculates the hash value for the specified address
used to create the index table.

\param[in]      pMac_p              Pointer to MAC address

\return Returns the hash value for the MAC address
*/
//------------------------------------------------------------------------------
static UINT32 calculateHashAddr(const UINT8* pMac_p)
{
    UINT32      hashIndex = 0;
    int         loop;

    for (loop = 0; loop <= 5; loop++)
    {
        hashIndex |=  ((getBitFromMac(pMac_p, (0 + loop)) ^
                        getBitFromMac(pMac_p, (6 + loop)) ^
                        getBitFromMac(pMac_p, (12 + loop)) ^
                        getBitFromMac(pMac_p, (18 + loop)) ^
                        getBitFromMac(pMac_p, (24 + loop)) ^
                        getBitFromMac(pMac_p, (30 + loop)) ^
                        getBitFromMac(pMac_p, (36 + loop)) ^
                        getBitFromMac(pMac_p, (42 + loop))) << loop);
    }
    return hashIndex;
}

//------------------------------------------------------------------------------
/**
\brief  Write to a PHY register

This routine writes the specified word into the specified PHY register
using the MDIO interface.

\param[in]      phyId_p             ID/Address of the PHY to write
\param[in]      regAddr_p           Register to write
\param[in]      value_p             Value to write
*/
//------------------------------------------------------------------------------
static void mdioPhyWrite(int phyId_p, int regAddr_p, UINT16 value_p)
{
    UINT32              regVal = 0;
    volatile UINT32     phyIdleState;

    regVal |= EDRV_PHYMNTNC_OP_MASK;
    regVal |= EDRV_PHYMNTNC_OP_WRITE;
    regVal |= (phyId_p << EDRV_PHYMNTNC_PHYAD_SHIFT);
    regVal |= (regAddr_p << EDRV_PHYMNTNC_PHYREG_SHIFT);
    regVal |= value_p;

    EDRV_WRITE_REG(EDRV_PHYMNTNC_OFFSET, regVal);

    // wait for completion of transfer
    do
    {
        cpu_relax();
        phyIdleState = EDRV_READ_REG(EDRV_NET_STATUS_REG);
    } while ((phyIdleState & EDRV_NWSR_MDIOIDLE_MASK) == 0); //wait PHYIDLE is set 1
}

//------------------------------------------------------------------------------
/**
\brief  Read from a PHY register

This routine reads a word from the specified PHY register
using the MDIO interface.

\param[in]      phyId_p             ID/Address of the PHY to read
\param[in]      regAddr_p           Register to read

\return Returns the read value.
*/
//------------------------------------------------------------------------------
static UINT16 mdioPhyRead(int phyId_p, int regAddr_p)
{
    UINT32              regVal = 0;
    UINT16              value = 0;
    volatile UINT32     phyIdleState;

    regVal |= EDRV_PHYMNTNC_OP_MASK;
    regVal |= EDRV_PHYMNTNC_OP_READ;
    regVal |= (phyId_p << EDRV_PHYMNTNC_PHYAD_SHIFT);
    regVal |= (regAddr_p << EDRV_PHYMNTNC_PHYREG_SHIFT);

    EDRV_WRITE_REG(EDRV_PHYMNTNC_OFFSET, regVal);

    // wait till the value is being read

    do
    {
        cpu_relax();
        phyIdleState = EDRV_READ_REG(EDRV_NET_STATUS_REG);
    } while ((phyIdleState & EDRV_NWSR_MDIOIDLE_MASK) == 0); //wait PHYIDLE is set 1

    value = (EDRV_READ_REG(EDRV_PHYMNTNC_OFFSET) & EDRV_PHYMNTNC_DATA_MASK);

    return value;
}

/// \}
