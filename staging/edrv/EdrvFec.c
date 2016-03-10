/*
 * Fast Ethernet Controller (FEC) for Freescale i.MX51
 * Copyright (c) 2011 SYS TEC electronic GmbH
 * www.systec-electronic.com
 *
 * Derived from Linux kernel 2.6.35.3-433-g0fae922 /drivers/net/fec.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 *
 *
 * Original Header:
 * Fast Ethernet Controller (FEC) driver for Motorola MPC8xx.
 * Copyright (c) 1997 Dan Malek (dmalek@jlc.net)
 *
 * Right now, I am very wasteful with the buffers.  I allocate memory
 * pages and then divide them into 2K frame buffers.  This way I know I
 * have buffers large enough to hold one frame within one buffer descriptor.
 * Once I get this working, I will use 64 or 128 byte CPM buffers, which
 * will be much more memory efficient and will easily handle lots of
 * small packets.
 *
 * Much better multiple PHY support by Magnus Damm.
 * Copyright (c) 2000 Ericsson Radio Systems AB.
 *
 * Support for FEC controller of ColdFire processors.
 * Copyright (c) 2001-2005 Greg Ungerer (gerg@snapgear.com)
 *
 * Bug fixes and cleanup by Philippe De Muyter (phdm@macqel.be)
 * Copyright (c) 2004-2006 Macq Electronique SA.
 *
 * Support for FEC IEEE 1588.
 * Copyright (C) 2010 Freescale Semiconductor, Inc.
 *
 */


#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/ptrace.h>
#include <linux/errno.h>
#include <linux/ioport.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/pci.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include <linux/bitops.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/swab.h>
#include <linux/phy.h>
#include <linux/fec.h>

#include <asm/cacheflush.h>

#ifndef CONFIG_ARCH_MXC
#ifndef CONFIG_ARCH_MXS
#include <asm/coldfire.h>
#include <asm/mcfsim.h>
#endif
#endif


#include "kernel/edrv.h"


/* ---------------------------------------------------------------------------
 * from	fec.h  --  Fast Ethernet Controller for Motorola ColdFire SoC
 *		   processors.
 *
 *	(C) Copyright 2000-2005, Greg Ungerer (gerg@snapgear.com)
 *	(C) Copyright 2000-2001, Lineo (www.lineo.com)
 */

#if defined(CONFIG_M523x) || defined(CONFIG_M527x) || defined(CONFIG_M528x) || \
    defined(CONFIG_M520x) || defined(CONFIG_M532x) || \
    defined(CONFIG_ARCH_MXC) || defined(CONFIG_ARCH_MXS)
/*
 *	Just figures, Motorola would have to change the offsets for
 *	registers in the same peripheral device on different models
 *	of the ColdFire!
 */
#define FEC_IEVENT		0x004 /* Interrupt event reg */
#define FEC_IMASK		0x008 /* Interrupt mask reg */
#define FEC_R_DES_ACTIVE	0x010 /* Receive descriptor reg */
#define FEC_X_DES_ACTIVE	0x014 /* Transmit descriptor reg */
#define FEC_ECNTRL		0x024 /* Ethernet control reg */
#define FEC_MII_DATA		0x040 /* MII manage frame reg */
#define FEC_MII_SPEED		0x044 /* MII speed control reg */
#define FEC_MIB_CTRLSTAT	0x064 /* MIB control/status reg */
#define FEC_R_CNTRL		0x084 /* Receive control reg */
#define FEC_X_CNTRL		0x0c4 /* Transmit Control reg */
#define FEC_ADDR_LOW		0x0e4 /* Low 32bits MAC address */
#define FEC_ADDR_HIGH		0x0e8 /* High 16bits MAC address */
#define FEC_OPD			0x0ec /* Opcode + Pause duration */
#define FEC_HASH_TABLE_HIGH	0x118 /* High 32bits hash table */
#define FEC_HASH_TABLE_LOW	0x11c /* Low 32bits hash table */
#define FEC_GRP_HASH_TABLE_HIGH	0x120 /* High 32bits hash table */
#define FEC_GRP_HASH_TABLE_LOW	0x124 /* Low 32bits hash table */
#define FEC_X_WMRK		0x144 /* FIFO transmit water mark */
#define FEC_R_BOUND		0x14c /* FIFO receive bound reg */
#define FEC_R_FSTART		0x150 /* FIFO receive start reg */
#define FEC_R_DES_START		0x180 /* Receive descriptor ring */
#define FEC_X_DES_START		0x184 /* Transmit descriptor ring */
#define FEC_R_BUFF_SIZE		0x188 /* Maximum receive buff size */
#define FEC_MIIGSK_CFGR		0x300 /* MIIGSK Configuration reg */
#define FEC_MIIGSK_ENR		0x308 /* MIIGSK Enable reg */

/* Define the FEC 1588 registers offset */
#define FEC_ATIME_CTRL		0x400
#define FEC_ATIME		0x404
#define FEC_ATIME_EVT_OFFSET	0x408
#define FEC_ATIME_EVT_PERIOD	0x40c
#define FEC_ATIME_CORR		0x410
#define FEC_ATIME_INC		0x414
#define FEC_TS_TIMESTAMP	0x418

#else

#define FEC_ECNTRL		0x000 /* Ethernet control reg */
#define FEC_IEVENT		0x004 /* Interrupt even reg */
#define FEC_IMASK		0x008 /* Interrupt mask reg */
#define FEC_IVEC		0x00c /* Interrupt vec status reg */
#define FEC_R_DES_ACTIVE	0x010 /* Receive descriptor reg */
#define FEC_X_DES_ACTIVE	0x014 /* Transmit descriptor reg */
#define FEC_MII_DATA		0x040 /* MII manage frame reg */
#define FEC_MII_SPEED		0x044 /* MII speed control reg */
#define FEC_R_BOUND		0x08c /* FIFO receive bound reg */
#define FEC_R_FSTART		0x090 /* FIFO receive start reg */
#define FEC_X_WMRK		0x0a4 /* FIFO transmit water mark */
#define FEC_X_FSTART		0x0ac /* FIFO transmit start reg */
#define FEC_R_CNTRL		0x104 /* Receive control reg */
#define FEC_MAX_FRM_LEN		0x108 /* Maximum frame length reg */
#define FEC_X_CNTRL		0x144 /* Transmit Control reg */
#define FEC_ADDR_LOW		0x3c0 /* Low 32bits MAC address */
#define FEC_ADDR_HIGH		0x3c4 /* High 16bits MAC address */
#define FEC_GRP_HASH_TABLE_HIGH	0x3c8 /* High 32bits hash table */
#define FEC_GRP_HASH_TABLE_LOW	0x3cc /* Low 32bits hash table */
#define FEC_R_DES_START		0x3d0 /* Receive descriptor ring */
#define FEC_X_DES_START		0x3d4 /* Transmit descriptor ring */
#define FEC_R_BUFF_SIZE		0x3d8 /* Maximum receive buff size */
#define FEC_FIFO_RAM		0x400 /* FIFO RAM buffer */

#endif /* CONFIG_M5272 */

#if defined(CONFIG_ARCH_MX28) && defined(CONFIG_FEC_1588)
#define CONFIG_ENHANCED_BD
#endif

/*
 *	Define the buffer descriptor structure.
 */
#if defined(CONFIG_ARCH_MXC) || defined(CONFIG_ARCH_MXS)
struct bufdesc {
	unsigned short cbd_datlen;	/* Data length */
	unsigned short cbd_sc;	/* Control and status info */
	unsigned long cbd_bufaddr;	/* Buffer address */
#ifdef CONFIG_ENHANCED_BD
	unsigned long cbd_esc;
	unsigned long cbd_prot;
	unsigned long cbd_bdu;
	unsigned long ts;
	unsigned short res0[4];
#endif
};
#else
struct bufdesc {
	unsigned short	cbd_sc;			/* Control and status info */
	unsigned short	cbd_datlen;		/* Data length */
	unsigned long	cbd_bufaddr;		/* Buffer address */
};
#endif

/*
 *	The following definitions courtesy of commproc.h, which where
 *	Copyright (c) 1997 Dan Malek (dmalek@jlc.net).
 */
#define BD_SC_EMPTY     ((ushort)0x8000)        /* Recieve is empty */
#define BD_SC_READY     ((ushort)0x8000)        /* Transmit is ready */
#define BD_SC_WRAP      ((ushort)0x2000)        /* Last buffer descriptor */
#define BD_SC_INTRPT    ((ushort)0x1000)        /* Interrupt on change */
#define BD_SC_CM        ((ushort)0x0200)        /* Continous mode */
#define BD_SC_ID        ((ushort)0x0100)        /* Rec'd too many idles */
#define BD_SC_P         ((ushort)0x0100)        /* xmt preamble */
#define BD_SC_BR        ((ushort)0x0020)        /* Break received */
#define BD_SC_FR        ((ushort)0x0010)        /* Framing error */
#define BD_SC_PR        ((ushort)0x0008)        /* Parity error */
#define BD_SC_OV        ((ushort)0x0002)        /* Overrun */
#define BD_SC_CD        ((ushort)0x0001)        /* ?? */

/* Buffer descriptor control/status used by Ethernet receive.
*/
#define BD_ENET_RX_EMPTY        ((ushort)0x8000)
#define BD_ENET_RX_WRAP         ((ushort)0x2000)
#define BD_ENET_RX_INTR         ((ushort)0x1000)
#define BD_ENET_RX_LAST         ((ushort)0x0800)
#define BD_ENET_RX_FIRST        ((ushort)0x0400)
#define BD_ENET_RX_MISS         ((ushort)0x0100)
#define BD_ENET_RX_LG           ((ushort)0x0020)
#define BD_ENET_RX_NO           ((ushort)0x0010)
#define BD_ENET_RX_SH           ((ushort)0x0008)
#define BD_ENET_RX_CR           ((ushort)0x0004)
#define BD_ENET_RX_OV           ((ushort)0x0002)
#define BD_ENET_RX_CL           ((ushort)0x0001)
#define BD_ENET_RX_STATS        ((ushort)0x013f)        /* All status bits */

#define BD_ENET_RX_INT		0x00800000
#define BD_ENET_RX_PTP		((ushort)0x0400)

/* Buffer descriptor control/status used by Ethernet transmit.
*/
#define BD_ENET_TX_READY        ((ushort)0x8000)
#define BD_ENET_TX_PAD          ((ushort)0x4000)
#define BD_ENET_TX_WRAP         ((ushort)0x2000)
#define BD_ENET_TX_INTR         ((ushort)0x1000)
#define BD_ENET_TX_LAST         ((ushort)0x0800)
#define BD_ENET_TX_TC           ((ushort)0x0400)
#define BD_ENET_TX_DEF          ((ushort)0x0200)
#define BD_ENET_TX_HB           ((ushort)0x0100)
#define BD_ENET_TX_LC           ((ushort)0x0080)
#define BD_ENET_TX_RL           ((ushort)0x0040)
#define BD_ENET_TX_RCMASK       ((ushort)0x003c)
#define BD_ENET_TX_UN           ((ushort)0x0002)
#define BD_ENET_TX_CSL          ((ushort)0x0001)
#define BD_ENET_TX_STATS        ((ushort)0x03ff)        /* All status bits */

#define BD_ENET_TX_INT		0x40000000
#define BD_ENET_TX_PTP		((ushort)0x0100)

/* ------------------------------------------------------------------------ */


#if defined(CONFIG_ARCH_MXC) || defined(CONFIG_ARCH_MXS)
#define FEC_ALIGNMENT	0xf
#else
#define FEC_ALIGNMENT	0x3
#endif

#if defined(CONFIG_M5272)
/*
 * Some hardware gets it MAC address out of local flash memory.
 * if this is non-zero then assume it is the address to get MAC from.
 */
#if defined(CONFIG_NETtel)
#define	FEC_FLASHMAC	0xf0006006
#elif defined(CONFIG_GILBARCONAP) || defined(CONFIG_SCALES)
#define	FEC_FLASHMAC	0xf0006000
#elif defined(CONFIG_CANCam)
#define	FEC_FLASHMAC	0xf0020000
#elif defined (CONFIG_M5272C3)
#define	FEC_FLASHMAC	(0xffe04000 + 4)
#elif defined(CONFIG_MOD5272)
#define FEC_FLASHMAC 	0xffc0406b
#else
#define	FEC_FLASHMAC	0
#endif
#endif /* CONFIG_M5272 */

/* The number of Tx and Rx buffers.  These are allocated from the page
 * pool.  The code may assume these are power of two, so it it best
 * to keep them that size.
 * We don't need to allocate pages for the transmitter.  We just use
 * the skbuffer directly.
 */
#define FEC_ENET_RX_PAGES	8
#define FEC_ENET_RX_FRSIZE	2048
#define FEC_ENET_RX_FRPPG	(PAGE_SIZE / FEC_ENET_RX_FRSIZE)
#define RX_RING_SIZE		(FEC_ENET_RX_FRPPG * FEC_ENET_RX_PAGES)
//#define FEC_ENET_TX_FRSIZE	2048
//#define FEC_ENET_TX_FRPPG	(PAGE_SIZE / FEC_ENET_TX_FRSIZE)
#define TX_RING_SIZE		16	/* Must be power of two */
#define TX_RING_MOD_MASK	15	/*   for this to work */

#if (((RX_RING_SIZE + TX_RING_SIZE) * 8) > PAGE_SIZE)
#error "FEC: descriptor ring size constants too large"
#endif

/* Interrupt events/masks. */
#define FEC_ENET_HBERR	((uint)0x80000000)	/* Heartbeat error */
#define FEC_ENET_BABR	((uint)0x40000000)	/* Babbling receiver */
#define FEC_ENET_BABT	((uint)0x20000000)	/* Babbling transmitter */
#define FEC_ENET_GRA	((uint)0x10000000)	/* Graceful stop complete */
#define FEC_ENET_TXF	((uint)0x08000000)	/* Full frame transmitted */
#define FEC_ENET_TXB	((uint)0x04000000)	/* A buffer was transmitted */
#define FEC_ENET_RXF	((uint)0x02000000)	/* Full frame received */
#define FEC_ENET_RXB	((uint)0x01000000)	/* A buffer was received */
#define FEC_ENET_MII	((uint)0x00800000)	/* MII interrupt */
#define FEC_ENET_EBERR	((uint)0x00400000)	/* SDMA bus error */
#define FEC_ENET_TS_AVAIL	((uint)0x00010000)
#define FEC_ENET_TS_TIMER	((uint)0x00008000)

#if defined(CONFIG_FEC_1588) && defined(CONFIG_ARCH_MX28)
#define FEC_DEFAULT_IMASK (FEC_ENET_TXF | FEC_ENET_RXF | FEC_ENET_MII | \
				FEC_ENET_TS_AVAIL | FEC_ENET_TS_TIMER)
#else
#define FEC_DEFAULT_IMASK (FEC_ENET_TXF | FEC_ENET_RXF | FEC_ENET_MII)
#endif

/* The FEC stores dest/src/type, data, and checksum for receive packets.
 */
#define PKT_MAXBUF_SIZE		1518
#define PKT_MINBUF_SIZE		64
#define PKT_MAXBLR_SIZE		1520


/*
 * The 5270/5271/5280/5282/532x RX control register also contains maximum frame
 * size bits. Other FEC hardware does not, so we need to take that into
 * account when setting it.
 */
#if defined(CONFIG_M523x) || defined(CONFIG_M527x) || defined(CONFIG_M528x) || \
    defined(CONFIG_M520x) || defined(CONFIG_M532x) || \
    defined(CONFIG_ARCH_MXC) || defined(CONFIG_ARCH_MXS)
#define	OPT_FRAME_SIZE	(PKT_MAXBUF_SIZE << 16)
#else
#define	OPT_FRAME_SIZE	0
#endif


/* FEC MII MMFR bits definition */
#define FEC_MMFR_ST		(1 << 30)
#define FEC_MMFR_OP_READ	(2 << 28)
#define FEC_MMFR_OP_WRITE	(1 << 28)
#define FEC_MMFR_PA(v)		((v & 0x1f) << 23)
#define FEC_MMFR_RA(v)		((v & 0x1f) << 18)
#define FEC_MMFR_TA		(2 << 16)
#define FEC_MMFR_DATA(v)	(v & 0xffff)

#define FEC_MII_TIMEOUT		1000



#define EDRV_DEV_NAME           "fec_imx51"

#ifndef EDRV_MAX_TX_BUFFERS
#define EDRV_MAX_TX_BUFFERS     42
#endif

#define EDRV_MAX_FRAME_SIZE     0x600     /* MTU + 14 + 4 */

#ifndef EDRV_MAX_RX_BUFFERS
#define EDRV_MAX_RX_BUFFERS     256
#endif

#define EDRV_RX_BUFFER_SIZE_SHIFT   11  // 2048 Byte
#define EDRV_RX_BUFFER_SIZE         (1 << EDRV_RX_BUFFER_SIZE_SHIFT)



/* The FEC buffer descriptors track the ring buffers.  The rx_bd_base and
 * tx_bd_base always point to the base of the buffer descriptors.  The
 * cur_rx and cur_tx point to the currently available buffer.
 * The dirty_tx tracks the current buffer that is being sent by the
 * controller.  The cur_tx and dirty_tx are equal under both completely
 * empty and completely full conditions.  The empty/ready indicator in
 * the buffer descriptor determines the actual condition.
 */
struct fec_enet_private_t {
	/* Hardware registers of the FEC device */
	void __iomem *hwp;

	struct clk *clk;

	BYTE*   m_pbTxBufferBase;
	BOOL    m_afTxBufferUsed[EDRV_MAX_TX_BUFFERS];

	/* The saved address of a sent-in-place packet/buffer, for skfree(). */
	tEdrvTxBuffer*   m_apTxBuffer[TX_RING_SIZE];
	ushort	txb_cur;
	ushort	txb_dirty;

	/* Stack of free rx buffers
	 * +1 additional place if ReleaseRxBuffer is called
	 * before return of RxHandler (multi processor)
	 */
	BYTE*           m_apbFreeRxBuffer[EDRV_MAX_RX_BUFFERS - RX_RING_SIZE + 1];
	int             m_iFreeRxBufferTop;
	spinlock_t      m_SpinLockRxBufferRelease;
	int             m_iPageAllocations;

	/* CPM dual port RAM relative addresses */
	dma_addr_t	bd_dma;
	/* Address of Rx and Tx buffers */
	struct bufdesc	*rx_bd_base;
	struct bufdesc	*tx_bd_base;
	/* The next free ring entry */
	struct bufdesc	*cur_rx, *cur_tx;
	/* The ring entries to be free()ed */
	struct bufdesc	*dirty_tx;

	uint	tx_full;
	/* hold while accessing the HW like ringbuffer for tx/rx but not MAC */
	spinlock_t hw_lock;

	struct  platform_device *pdev;

	/* Phylib and MDIO interface */
	struct  mii_bus *mii_bus;
	struct  phy_device *phy_dev;
	int     mii_timeout;
	uint    phy_speed;
	phy_interface_t	phy_interface;
	int	link;
	int	full_duplex;
	struct  completion mdio_done;

	tEdrvInitParam m_InitParam;

#if EDRV_USE_DIAGNOSTICS != FALSE
	int           m_iFreeRxBufferMin;
	unsigned int  m_uiInterruptCount;
	unsigned int  m_uiEventCountRX;
	unsigned int  m_uiEventCountTX;
	unsigned int  m_uiEventCountMII;
	unsigned int  tx_errors;
	unsigned int  tx_heartbeat_errors;
	unsigned int  tx_window_errors;
	unsigned int  tx_aborted_errors;
	unsigned int  tx_fifo_errors;
	unsigned int  tx_carrier_errors;
	unsigned int  tx_packets;
	unsigned int  tx_packetssend;
	unsigned int  rx_errors;
	unsigned int  rx_length_errors;
	unsigned int  rx_frame_errors;
	unsigned int  rx_crc_errors;
	unsigned int  rx_fifo_errors;
	unsigned int  rx_packets;
	unsigned int  collisions;
#endif
};


static struct fec_enet_private_t fec_enet_private;

/*
 * Define the fixed address of the FEC hardware.
 */
static unsigned char	fec_mac_default[ETH_ALEN];


static irqreturn_t fec_enet_interrupt(int irq, void * dev_id);
static void fec_enet_tx(void);
static void fec_enet_rx(void);
static void fec_restart(int duplex);
static void fec_stop(void);

static int fec_probe(struct platform_device *pdev);
static int fec_remove(struct platform_device *pdev);
static int fec_suspend(struct platform_device *dev, pm_message_t state);
static int fec_resume(struct platform_device *dev);


static struct platform_driver fec_driver = {
	.driver	= {
		.name    = "fec",
		.owner	 = THIS_MODULE,
	},
	.probe   = fec_probe,
	.remove  = fec_remove,
	.suspend = fec_suspend,
	.resume  = fec_resume,
};


tOplkError  edrv_init(tEdrvInitParam* pEdrvInitParam_p)
{
tOplkError  Ret;
int         iRes;
int         iIndex;

    Ret = kErrorOk;

    // clear instance structure
    OPLK_MEMSET(&fec_enet_private, 0, sizeof (fec_enet_private));

    // save the init data
    fec_enet_private.m_InitParam = *pEdrvInitParam_p;

    printk("(%s) Registering the driver to the kernel...", __func__);

    // register platform driver
    iRes = platform_driver_register(&fec_driver);
    if (iRes)
    {
        printk("%s: platform_driver_register() failed (returned %d)\n", EDRV_DEV_NAME, iRes);
        Ret = kErrorNoResource;
        goto Exit;
    }

    // local MAC address might have been changed in fec_probe()
//    OPLK_MEMCPY(pEdrvInitParam_p->aMacAddr, fec_enet_private.m_InitParam.aMacAddr, ETH_ALEN);

    printk(KERN_INFO "%s: FEC ethernet driver initialized\n", EDRV_DEV_NAME);
    printk(KERN_INFO "%s: MAC = ", EDRV_DEV_NAME);

    for (iIndex = 0; iIndex < ETH_ALEN; iIndex++)
    {
        printk("%02x", (unsigned int) pEdrvInitParam_p->aMacAddr[iIndex]);
        if (iIndex < ETH_ALEN - 1)
        {
            printk(":");
        }
    }
    printk("\n");

Exit:
    return Ret;
}


tOplkError edrv_exit(void)
{
    printk("%s entered\n", __FUNCTION__);

    // unregister PCI driver
    platform_driver_unregister(&fec_driver);
    printk("%s: platform_driver_unregister() done\n", __FUNCTION__);

    return kErrorOk;
}


static void __inline__ fec_get_mac(unsigned char *dev_addr);

UINT8* edrv_getMacAddr(void)
{
	fec_get_mac(fec_enet_private.m_InitParam.aMacAddr);
	return fec_enet_private.m_InitParam.aMacAddr;

}



#define HASH_BITS	6		/* #bits in hash */
#define CRC32_POLY	0xEDB88320

tOplkError  edrv_setRxMulticastMacAddr(BYTE* pbMacAddr_p)
{
	struct fec_enet_private_t *fep = &fec_enet_private;
	tOplkError Ret;
#if 0
	struct netdev_hw_addr *ha;
	unsigned int i, bit, data, crc, tmp;
	unsigned char hash;
#else
	unsigned int tmp;
#endif

	Ret = kErrorOk;

	/* The following code origins from former function set_multicast_list() */

	/* Set or clear the multicast filter for this adaptor.
	 * Skeleton taken from sunlance driver.
	 * The CPM Ethernet implementation allows Multicast as well as individual
	 * MAC address filtering.  Some of the drivers check to make sure it is
	 * a group multicast address, and discard those that are not.  I guess I
	 * will do the same for now, but just remove the test if you want
	 * individual filtering as well (do the upper net layers want or support
	 * this kind of feature?).
	 */

	/* Disable promisc mode */
//	tmp = readl(fep->hwp + FEC_R_CNTRL);
//	tmp &= ~0x8;
//	writel(tmp, fep->hwp + FEC_R_CNTRL);

	/* Enable promisc mode */
	tmp = readl(fep->hwp + FEC_R_CNTRL);
	tmp |= 0x8;
	writel(tmp, fep->hwp + FEC_R_CNTRL);


//#warning "For testing"
	/* Catch all multicast addresses, so set the
	 * filter to all 1's
	 */
//	writel(0xffffffff, fep->hwp + FEC_GRP_HASH_TABLE_HIGH);
//	writel(0xffffffff, fep->hwp + FEC_GRP_HASH_TABLE_LOW);

#if 0
	/* Clear filter and add the addresses in hash register
	 */
	writel(0, fep->hwp + FEC_GRP_HASH_TABLE_HIGH);
	writel(0, fep->hwp + FEC_GRP_HASH_TABLE_LOW);

	netdev_for_each_mc_addr(ha, dev) {
		/* Only support group multicast for now */
		if (!(ha->addr[0] & 1))
			continue;

		/* calculate crc32 value of mac address */
		crc = 0xffffffff;

		for (i = 0; i < dev->addr_len; i++) {
			data = ha->addr[i];
			for (bit = 0; bit < 8; bit++, data >>= 1) {
				crc = (crc >> 1) ^
				(((crc ^ data) & 1) ? CRC32_POLY : 0);
			}
		}

		/* only upper 6 bits (HASH_BITS) are used
		 * which point to specific bit in he hash registers
		 */
		hash = (crc >> (32 - HASH_BITS)) & 0x3f;

		if (hash > 31) {
			tmp = readl(fep->hwp + FEC_GRP_HASH_TABLE_HIGH);
			tmp |= 1 << (hash - 32);
			writel(tmp, fep->hwp + FEC_GRP_HASH_TABLE_HIGH);
		} else {
			tmp = readl(fep->hwp + FEC_GRP_HASH_TABLE_LOW);
			tmp |= 1 << hash;
			writel(tmp, fep->hwp + FEC_GRP_HASH_TABLE_LOW);
		}
	}
#endif

    return Ret;
}


tOplkError  edrv_clearRxMulticastMacAddr(BYTE* pbMacAddr_p)
{
tOplkError  Ret = kErrorOk;

#warning "To be done"
    return Ret;
}


tOplkError  edrv_changeRxFilter(tEdrvFilter*    pFilter_p,
                             unsigned int    uiCount_p,
                             unsigned int    uiEntryChanged_p,
                             unsigned int    uiChangeFlags_p)
{
    return kErrorOk;
}


tOplkError  edrv_allocTxBuffer (tEdrvTxBuffer* pTxBuffer_p)
{
tOplkError   Ret;
unsigned int i;

    Ret = kErrorOk;

    if (pTxBuffer_p->maxBufferSize > EDRV_MAX_FRAME_SIZE)
    {
        Ret = kErrorEdrvNoFreeBufEntry;
        goto Exit;
    }

    if (fec_enet_private.m_pbTxBufferBase == NULL)
    {
        printk("%s: Tx buffers currently not allocated\n", EDRV_DEV_NAME);
        Ret = kErrorEdrvNoFreeBufEntry;
        goto Exit;
    }

    // search a free Tx buffer with appropriate size
    for (i = 0; i < EDRV_MAX_TX_BUFFERS; i++)
    {
        if (fec_enet_private.m_afTxBufferUsed[i] == FALSE)
        {
            // free channel found
            fec_enet_private.m_afTxBufferUsed[i] = TRUE;
            pTxBuffer_p->txBufferNumber.value = i;
            pTxBuffer_p->pBuffer = fec_enet_private.m_pbTxBufferBase + (i * EDRV_MAX_FRAME_SIZE);
            pTxBuffer_p->maxBufferSize = EDRV_MAX_FRAME_SIZE;
            break;
        }
    }

    if (i >= EDRV_MAX_TX_BUFFERS)
    {
        Ret = kErrorEdrvNoFreeBufEntry;
        goto Exit;
    }

Exit:
    return Ret;

}


tOplkError  edrv_freeTxBuffer (tEdrvTxBuffer * pTxBuffer_p)
{
unsigned int uiBufferNumber;

    uiBufferNumber = pTxBuffer_p->txBufferNumber.value;

    if (uiBufferNumber < EDRV_MAX_TX_BUFFERS)
    {
        fec_enet_private.m_afTxBufferUsed[uiBufferNumber] = FALSE;
    }

    return kErrorOk;

}


#ifdef CONFIG_ARCH_MXS
static void *swap_buffer(void *bufaddr, int len)
{
	int i;
	unsigned int *buf = bufaddr;

	for (i = 0; i < (len + 3) / 4; i++, buf++)
		*buf = __swab32(*buf);

	return bufaddr;
}
#endif


tOplkError  edrv_sendTxBuffer (tEdrvTxBuffer* pTxBuffer_p)
{
	struct fec_enet_private_t *fep = &fec_enet_private;
	struct bufdesc *bdp;
	void *bufaddr;
	unsigned short	status;
	unsigned long   flags;
	tOplkError      Ret;
	unsigned int    uiBufferNumber;

	Ret = kErrorOk;

	uiBufferNumber = pTxBuffer_p->txBufferNumber.value;

	if ((uiBufferNumber >= EDRV_MAX_TX_BUFFERS) ||
	    (fec_enet_private.m_afTxBufferUsed[uiBufferNumber] == FALSE))
	{
		Ret = kErrorEdrvBufNotExisting;
		goto Exit;
	}

#if EDRV_USE_DIAGNOSTICS != FALSE
	fep->tx_packetssend++;
#endif

#if 0
	if (!fep->link) {
		/* Link is down or autonegotiation is in progress. */
		Ret = kErrorEdrvNoLink;
		goto Exit;
	}
#endif

	spin_lock_irqsave(&fep->hw_lock, flags);
	/* Fill in a Tx ring entry */
	bdp = fep->cur_tx;

	status = bdp->cbd_sc;

	if (status & BD_ENET_TX_READY) {
		spin_unlock_irqrestore(&fep->hw_lock, flags);
                Ret = kErrorEdrvNoFreeTxDesc;
		goto Exit;
	}

	/* Clear all of the status flags */
	status &= ~BD_ENET_TX_STATS;

	/* Set buffer length and buffer pointer */
	bufaddr = (void*) pTxBuffer_p->pBuffer;
	bdp->cbd_datlen = pTxBuffer_p->txFrameSize;

#ifdef CONFIG_ARCH_MXS
	swap_buffer(bufaddr, pTxBuffer_p->txFrameSize);
#endif
	/* Save pointer to buffer structure for TxHandler */
	fep->m_apTxBuffer[fep->txb_cur] = pTxBuffer_p;
	fep->txb_cur = (fep->txb_cur+1) & TX_RING_MOD_MASK;

	/* Push the data cache so the CPM does not get stale memory
	 * data.
	 */
	bdp->cbd_bufaddr = dma_map_single(&fep->pdev->dev, bufaddr,
			EDRV_MAX_FRAME_SIZE, DMA_TO_DEVICE);

	/* Send it on its way.  Tell FEC it's ready, interrupt when done,
	 * it's the last BD of the frame, and to put the CRC on the end.
	 */
	status |= (BD_ENET_TX_READY | BD_ENET_TX_INTR
			| BD_ENET_TX_LAST | BD_ENET_TX_TC);
	bdp->cbd_sc = status;

	/* Trigger transmission start */
	writel(0, fep->hwp + FEC_X_DES_ACTIVE);

	/* If this was the last BD in the ring, start at the beginning again. */
	if (status & BD_ENET_TX_WRAP)
		bdp = fep->tx_bd_base;
	else
		bdp++;

	if (bdp == fep->dirty_tx) {
		fep->tx_full = 1;
	}

	fep->cur_tx = bdp;

	spin_unlock_irqrestore(&fep->hw_lock, flags);

Exit:
	return Ret;
}


#if EDRV_USE_DIAGNOSTICS != FALSE
int  EdrvGetDiagnostics (char* pszBuffer_p, int iSize_p)
{
int iUsedSize = 0;

    iUsedSize += snprintf (pszBuffer_p + iUsedSize, iSize_p - iUsedSize,
                       "\nEdrv Diagnostic Information\n");

    iUsedSize += snprintf (pszBuffer_p + iUsedSize, iSize_p - iUsedSize,
                       "Free RxBuffers: %d (Min: %d)\n",
                       fec_enet_private.m_iFreeRxBufferTop + 1,
                       fec_enet_private.m_iFreeRxBufferMin);

    iUsedSize += snprintf (pszBuffer_p + iUsedSize, iSize_p - iUsedSize,
                       "Total Edrv Interrupts:               %u\n",
                       fec_enet_private.m_uiInterruptCount);
    iUsedSize += snprintf (pszBuffer_p + iUsedSize, iSize_p - iUsedSize,
                       "Tx-Interrupts:                       %u\n",
                       fec_enet_private.m_uiEventCountTX);
    iUsedSize += snprintf (pszBuffer_p + iUsedSize, iSize_p - iUsedSize,
                       "Rx-Interrupts:                       %u\n",
                       fec_enet_private.m_uiEventCountRX);
    iUsedSize += snprintf (pszBuffer_p + iUsedSize, iSize_p - iUsedSize,
                       "MII-Interrupts:                      %u\n",
                       fec_enet_private.m_uiEventCountMII);

    iUsedSize += snprintf (pszBuffer_p + iUsedSize, iSize_p - iUsedSize,
                       "Tx packets:                          %u\n",
                       fec_enet_private.tx_packets);
    iUsedSize += snprintf (pszBuffer_p + iUsedSize, iSize_p - iUsedSize,
                       "Tx packets SendMsg:                  %u\n",
                       fec_enet_private.tx_packetssend);
    iUsedSize += snprintf (pszBuffer_p + iUsedSize, iSize_p - iUsedSize,
                       "Tx total errors:                     %u\n",
                       fec_enet_private.tx_errors);
    iUsedSize += snprintf (pszBuffer_p + iUsedSize, iSize_p - iUsedSize,
                       "Tx heartbeat errors:                 %u\n",
                       fec_enet_private.tx_heartbeat_errors);
    iUsedSize += snprintf (pszBuffer_p + iUsedSize, iSize_p - iUsedSize,
                       "Tx window errors:                    %u\n",
                       fec_enet_private.tx_window_errors);
    iUsedSize += snprintf (pszBuffer_p + iUsedSize, iSize_p - iUsedSize,
                       "Tx aborted errors:                   %u\n",
                       fec_enet_private.tx_aborted_errors);
    iUsedSize += snprintf (pszBuffer_p + iUsedSize, iSize_p - iUsedSize,
                       "Tx FIFO errors:                      %u\n",
                       fec_enet_private.tx_fifo_errors);
    iUsedSize += snprintf (pszBuffer_p + iUsedSize, iSize_p - iUsedSize,
                       "Tx carrier errors:                   %u\n",
                       fec_enet_private.tx_carrier_errors);

    iUsedSize += snprintf (pszBuffer_p + iUsedSize, iSize_p - iUsedSize,
                       "Rx packets:                          %u\n",
                       fec_enet_private.rx_packets);
    iUsedSize += snprintf (pszBuffer_p + iUsedSize, iSize_p - iUsedSize,
                       "Rx total errors:                     %u\n",
                       fec_enet_private.rx_errors);
    iUsedSize += snprintf (pszBuffer_p + iUsedSize, iSize_p - iUsedSize,
                       "Rx length errors:                    %u\n",
                       fec_enet_private.rx_length_errors);
    iUsedSize += snprintf (pszBuffer_p + iUsedSize, iSize_p - iUsedSize,
                       "Rx frame errors:                     %u\n",
                       fec_enet_private.rx_frame_errors);
    iUsedSize += snprintf (pszBuffer_p + iUsedSize, iSize_p - iUsedSize,
                       "Rx CRC errors:                       %u\n",
                       fec_enet_private.rx_crc_errors);
    iUsedSize += snprintf (pszBuffer_p + iUsedSize, iSize_p - iUsedSize,
                       "Rx FIFO errors:                      %u\n",
                       fec_enet_private.rx_fifo_errors);

    iUsedSize += snprintf (pszBuffer_p + iUsedSize, iSize_p - iUsedSize,
                       "Collision Count:                     %u\n",
                       fec_enet_private.collisions);

    iUsedSize += snprintf (pszBuffer_p + iUsedSize, iSize_p - iUsedSize,
                       "\n");

    return iUsedSize;
}
#endif


tOplkError  edrv_releaseRxBuffer (tEdrvRxBuffer* pRxBuffer_p)
{
tOplkError Ret;

    Ret = kErrorEdrvInvalidRxBuf;

    if (fec_enet_private.m_iFreeRxBufferTop < (EDRV_MAX_RX_BUFFERS-1))
    {
    unsigned long   ulFlags;

        spin_lock_irqsave(&fec_enet_private.m_SpinLockRxBufferRelease, ulFlags);
        fec_enet_private.m_apbFreeRxBuffer[++fec_enet_private.m_iFreeRxBufferTop] = pRxBuffer_p->pBuffer;
        spin_unlock_irqrestore(&fec_enet_private.m_SpinLockRxBufferRelease, ulFlags);

        Ret = kErrorOk;
    }

    return Ret; 
}


static irqreturn_t
fec_enet_interrupt(int irq, void * dev_id)
{
	struct fec_enet_private_t *fep = &fec_enet_private;
	uint	int_events;
	irqreturn_t ret = IRQ_NONE;

#if EDRV_USE_DIAGNOSTICS != FALSE
	fep->m_uiInterruptCount++;
#endif

	do {
		int_events = readl(fep->hwp + FEC_IEVENT);
		writel(int_events, fep->hwp + FEC_IEVENT);

#warning "Rx should have priority"
		if (int_events & FEC_ENET_RXF) {
#if EDRV_USE_DIAGNOSTICS != FALSE
			fep->m_uiEventCountRX++;
#endif
			ret = IRQ_HANDLED;
			fec_enet_rx();
		}

		/* Transmit OK, or non-fatal error. Update the buffer
		 * descriptors. FEC handles all errors, we just discover
		 * them as part of the transmit process.
		 */
		if (int_events & FEC_ENET_TXF) {
#if EDRV_USE_DIAGNOSTICS != FALSE
			fep->m_uiEventCountTX++;
#endif
			ret = IRQ_HANDLED;
			fec_enet_tx();
		}

		if (int_events & FEC_ENET_TS_AVAIL) {
			ret = IRQ_HANDLED;
		}

		if (int_events & FEC_ENET_TS_TIMER) {
			ret = IRQ_HANDLED;
		}

		if (int_events & FEC_ENET_MII) {
#if EDRV_USE_DIAGNOSTICS != FALSE
			fep->m_uiEventCountMII++;
#endif
			ret = IRQ_HANDLED;
			complete(&fep->mdio_done);
		}
	} while (int_events);

	return ret;
}


static void  fec_enet_tx (void)
{
	struct fec_enet_private_t *fep = &fec_enet_private;
	struct bufdesc *bdp;
	unsigned short status;
	tEdrvTxBuffer* pTxBuffer;

	spin_lock(&fep->hw_lock);
	bdp = fep->dirty_tx;

	while (((status = bdp->cbd_sc) & BD_ENET_TX_READY) == 0) {
		if (bdp == fep->cur_tx && fep->tx_full == 0)
			break;

		dma_unmap_single(&fep->pdev->dev, bdp->cbd_bufaddr, EDRV_MAX_FRAME_SIZE, DMA_TO_DEVICE);

		bdp->cbd_bufaddr = 0;

#if EDRV_USE_DIAGNOSTICS != FALSE
		/* Check for errors. */
		if (status & (BD_ENET_TX_HB | BD_ENET_TX_LC |
				   BD_ENET_TX_RL | BD_ENET_TX_UN |
				   BD_ENET_TX_CSL)) {
			fep->tx_errors++;
			if (status & BD_ENET_TX_HB)  /* No heartbeat */
				fep->tx_heartbeat_errors++;
			if (status & BD_ENET_TX_LC)  /* Late collision */
				fep->tx_window_errors++;
			if (status & BD_ENET_TX_RL)  /* Retrans limit */
				fep->tx_aborted_errors++;
			if (status & BD_ENET_TX_UN)  /* Underrun */
				fep->tx_fifo_errors++;
			if (status & BD_ENET_TX_CSL) /* Carrier lost */
				fep->tx_carrier_errors++;
		} else {
			fep->tx_packets++;
		}

		/* Deferred means some collisions occurred during transmit,
		 * but we eventually sent the packet OK.
		 */
		if (status & BD_ENET_TX_DEF)
			fep->collisions++;
#endif

		if (status & BD_ENET_TX_READY)
			printk("HEY! Enet xmit interrupt and TX_READY.\n");

		pTxBuffer = fep->m_apTxBuffer[fep->txb_dirty];
		fep->m_apTxBuffer[fep->txb_dirty] = NULL;

                if ((pTxBuffer != NULL) && (pTxBuffer->pfnTxHandler != NULL))
			// Call Tx handler of Data link layer
                        pTxBuffer->pfnTxHandler(pTxBuffer);

		fep->txb_dirty = (fep->txb_dirty + 1) & TX_RING_MOD_MASK;

		/* Update pointer to next buffer descriptor to be transmitted */
		if (status & BD_ENET_TX_WRAP)
			bdp = fep->tx_bd_base;
		else
			bdp++;

		/* Since we have freed up a buffer, the ring can not be full */
		fep->tx_full = 0;
	}
	fep->dirty_tx = bdp;
	spin_unlock(&fep->hw_lock);
}


/* During a receive, the cur_rx points to the current incoming buffer.
 * When we update through the ring, if the next incoming buffer has
 * not been given to the system, we just set the empty indicator,
 * effectively tossing the packet.
 */
static void  fec_enet_rx (void)
{
	struct	fec_enet_private_t *fep = &fec_enet_private;
	struct bufdesc *bdp;
	unsigned short status;
	ushort	pkt_len;
	__u8 *data;
	tEdrvRxBuffer           RxBuffer;
	tEdrvReleaseRxBuffer    RetReleaseRxBuffer;
	BYTE*                   pbFreeBuffer;

#ifdef CONFIG_M532x
	flush_cache_all();
#endif

	spin_lock(&fep->hw_lock);

	/* First, grab all of the stats for the incoming packet.
	 * These get messed up if we get called due to a busy condition.
	 */
	bdp = fep->cur_rx;

	while (!((status = bdp->cbd_sc) & BD_ENET_RX_EMPTY)) {

		/* Since we have allocated space to hold a complete frame,
		 * the last indicator should be set.
		 */
		if ((status & BD_ENET_RX_LAST) == 0)
			printk("%s: rcv is not +last\n", EDRV_DEV_NAME);

#if EDRV_USE_DIAGNOSTICS != FALSE
		/* Check for errors. */
		if (status & (BD_ENET_RX_LG | BD_ENET_RX_SH | BD_ENET_RX_NO |
			   BD_ENET_RX_CR | BD_ENET_RX_OV)) {
			fep->rx_errors++;
			if (status & (BD_ENET_RX_LG | BD_ENET_RX_SH)) {
				/* Frame too long or too short. */
				fep->rx_length_errors++;
			}
			if (status & BD_ENET_RX_NO)	/* Frame alignment */
				fep->rx_frame_errors++;
			if (status & BD_ENET_RX_CR)	/* CRC Error */
				fep->rx_crc_errors++;
			if (status & BD_ENET_RX_OV)	/* FIFO overrun */
				fep->rx_fifo_errors++;
		}
#endif

		/* Report late collisions as a frame error.
		 * On this error, the BD is closed, but we don't know what we
		 * have in the buffer.  So, just drop this frame on the floor.
		 */
		if (status & BD_ENET_RX_CL) {
#if EDRV_USE_DIAGNOSTICS != FALSE
			fep->rx_errors++;
			fep->rx_frame_errors++;
#endif
			goto rx_processing_done;
		}

		/* Process the incoming frame. */
#if EDRV_USE_DIAGNOSTICS != FALSE
		fep->rx_packets++;
#endif

		pkt_len = bdp->cbd_datlen;
		data = (__u8*)__va(bdp->cbd_bufaddr);

		/* The packet length includes FCS */
		RxBuffer.rxFrameSize    = pkt_len - 4;
		RxBuffer.pBuffer      = data;
		RxBuffer.bufferInFrame = kEdrvBufferLastInFrame;

		dma_sync_single_for_cpu(&fep->pdev->dev, bdp->cbd_bufaddr,
				EDRV_RX_BUFFER_SIZE, DMA_FROM_DEVICE);

#ifdef CONFIG_ARCH_MXS
		swap_buffer(data, pkt_len);
#endif

		/* Call Rx handler of Data link layer */
		RetReleaseRxBuffer = fep->m_InitParam.pfnRxHandler(&RxBuffer);
		if (RetReleaseRxBuffer == kEdrvReleaseRxBufferLater)
		{
			if (fep->m_iFreeRxBufferTop >= 0)
			{
				unsigned long   ulFlags;

#if EDRV_USE_DIAGNOSTICS != FALSE
				if (fep->m_iFreeRxBufferTop < fep->m_iFreeRxBufferMin)
				{
					fep->m_iFreeRxBufferMin = fep->m_iFreeRxBufferTop;
				}
#endif

				spin_lock_irqsave(&fep->m_SpinLockRxBufferRelease, ulFlags);
				pbFreeBuffer = fep->m_apbFreeRxBuffer[fep->m_iFreeRxBufferTop--];
				spin_unlock_irqrestore(&fep->m_SpinLockRxBufferRelease, ulFlags);

				dma_unmap_single(&fep->pdev->dev, bdp->cbd_bufaddr,
						EDRV_RX_BUFFER_SIZE, DMA_FROM_DEVICE);

				bdp->cbd_bufaddr = dma_map_single(&fep->pdev->dev, (void*) pbFreeBuffer,
						EDRV_RX_BUFFER_SIZE, DMA_FROM_DEVICE);
			}
			else
			{
				// $$$ How to signal no free RxBuffers left?
#if EDRV_USE_DIAGNOSTICS != FALSE
				fep->m_iFreeRxBufferMin = -1;
#endif
			}
		}

rx_processing_done:
		/* Clear the status flags for this buffer */
		status &= ~BD_ENET_RX_STATS;

		/* Mark the buffer empty */
		status |= BD_ENET_RX_EMPTY;
		bdp->cbd_sc = status;
#ifdef CONFIG_ENHANCED_BD
		bdp->cbd_esc = BD_ENET_RX_INT;
		bdp->cbd_prot = 0;
		bdp->cbd_bdu = 0;
#endif

		/* Update BD pointer to next entry */
		if (status & BD_ENET_RX_WRAP)
			bdp = fep->rx_bd_base;
		else
			bdp++;
		/* Doing this here will keep the FEC running while we process
		 * incoming frames.  On a heavily loaded network, we should be
		 * able to keep up at the expense of system resources.
		 */
		writel(0, fep->hwp + FEC_R_DES_ACTIVE);
	}
	fep->cur_rx = bdp;

	spin_unlock(&fep->hw_lock);
}

/* ------------------------------------------------------------------------- */
static void __inline__ fec_get_mac(unsigned char *dev_addr)
{
	struct fec_enet_private_t *fep = &fec_enet_private;
	unsigned char *iap, tmpaddr[ETH_ALEN];

#ifdef CONFIG_M5272
	if (FEC_FLASHMAC) {
		/*
		 * Get MAC address from FLASH.
		 * If it is all 1's or 0's, use the default.
		 */
		iap = (unsigned char *)FEC_FLASHMAC;
		if ((iap[0] == 0) && (iap[1] == 0) && (iap[2] == 0) &&
		    (iap[3] == 0) && (iap[4] == 0) && (iap[5] == 0))
			iap = fec_mac_default;
		if ((iap[0] == 0xff) && (iap[1] == 0xff) && (iap[2] == 0xff) &&
		    (iap[3] == 0xff) && (iap[4] == 0xff) && (iap[5] == 0xff))
			iap = fec_mac_default;
	}
#else
	if (is_valid_ether_addr(fec_mac_default)) {
		iap = fec_mac_default;
	}
#endif
	else {
		*((unsigned long *) &tmpaddr[0]) = be32_to_cpu(readl(fep->hwp + FEC_ADDR_LOW));
		*((unsigned short *) &tmpaddr[4]) = be16_to_cpu(readl(fep->hwp + FEC_ADDR_HIGH) >> 16);
		iap = &tmpaddr[0];
	}

	memcpy(dev_addr, iap, ETH_ALEN);
}

/* ------------------------------------------------------------------------- */

/*
 * Phy section
 */
static void fec_enet_adjust_link(void)
{
	struct fec_enet_private_t *fep = &fec_enet_private;
	struct phy_device *phy_dev = fep->phy_dev;
	unsigned long flags;

	int status_change = 0;

	spin_lock_irqsave(&fep->hw_lock, flags);

	/* Prevent a state halted on mii error */
	if (fep->mii_timeout && phy_dev->state == PHY_HALTED) {
		phy_dev->state = PHY_RESUMING;
		goto spin_unlock;
	}

	/* Duplex link change */
	if (phy_dev->link) {
		if (fep->full_duplex != phy_dev->duplex) {
			fec_restart(phy_dev->duplex);
			status_change = 1;
		}
	}

	/* Link on or off change */
	if (phy_dev->link != fep->link) {
		fep->link = phy_dev->link;
		if (phy_dev->link)
			fec_restart(phy_dev->duplex);
#if 0
		else
			fec_stop();
#endif
		status_change = 1;
	}

spin_unlock:
	spin_unlock_irqrestore(&fep->hw_lock, flags);

	if (status_change)
		phy_print_status(phy_dev);
}

static int fec_enet_mdio_read(struct mii_bus *bus, int mii_id, int regnum)
{
	struct fec_enet_private_t *fep = bus->priv;
	unsigned long time_left;

	fep->mii_timeout = 0;
	init_completion(&fep->mdio_done);

	/* start a read op */
	writel(FEC_MMFR_ST | FEC_MMFR_OP_READ |
		FEC_MMFR_PA(mii_id) | FEC_MMFR_RA(regnum) |
		FEC_MMFR_TA, fep->hwp + FEC_MII_DATA);

	/* wait for end of transfer */
	time_left = wait_for_completion_timeout(&fep->mdio_done,
		usecs_to_jiffies(FEC_MII_TIMEOUT));
	if (time_left == 0) {
		fep->mii_timeout = 1;
		printk(KERN_ERR "FEC: MDIO read timeout\n");
		return -ETIMEDOUT;
	}
	/* return value */
	return FEC_MMFR_DATA(readl(fep->hwp + FEC_MII_DATA));
}

static int fec_enet_mdio_write(struct mii_bus *bus, int mii_id, int regnum,
			   u16 value)
{
	struct fec_enet_private_t *fep = bus->priv;
	unsigned long time_left;

	fep->mii_timeout = 0;
	init_completion(&fep->mdio_done);

	/* start a write op */
	writel(FEC_MMFR_ST | FEC_MMFR_OP_WRITE |
		FEC_MMFR_PA(mii_id) | FEC_MMFR_RA(regnum) |
		FEC_MMFR_TA | FEC_MMFR_DATA(value),
		fep->hwp + FEC_MII_DATA);

	/* wait for end of transfer */
	time_left = wait_for_completion_timeout(&fep->mdio_done,
		usecs_to_jiffies(FEC_MII_TIMEOUT));
	if (time_left == 0) {
		fep->mii_timeout = 1;
		printk(KERN_ERR "FEC: MDIO write timeout\n");
		return -ETIMEDOUT;
	}

	return 0;
}

static int fec_enet_mdio_reset(struct mii_bus *bus)
{
	return 0;
}

static int fec_enet_mii_probe(void)
{
	struct fec_enet_private_t *fep = &fec_enet_private;
	struct phy_device *phy_dev = NULL;
	int phy_addr;
	int fec_index = fep->pdev->id > 0 ? fep->pdev->id : 0;

	fep->phy_dev = NULL;

	/* find the phy, assuming fec index corresponds to addr */
	for (phy_addr = 0; phy_addr < PHY_MAX_ADDR; phy_addr++) {
		if (fep->mii_bus->phy_map[phy_addr]) {
			if (fec_index--)
				continue;
			phy_dev = fep->mii_bus->phy_map[phy_addr];
			break;
		}
	}

	if (!phy_dev) {
		printk(KERN_ERR "%s: no PHY found\n", EDRV_DEV_NAME);
		return -ENODEV;
	}

	/* attach the mac to the phy */
// pnf@mar2016
	//phy_dev = phy_connect(fep->pdev,
	phy_dev = phy_connect(phy_dev->attached_dev,
			dev_name(&phy_dev->dev),
			(void (*)(struct net_device*)) fec_enet_adjust_link,
		//	0,
			fep->phy_interface);

	if (IS_ERR(phy_dev)) {
		printk(KERN_ERR "%s: Could not attach to PHY\n", EDRV_DEV_NAME);
		return PTR_ERR(phy_dev);
	}

	/* mask with MAC supported features */
	phy_dev->supported &= PHY_BASIC_FEATURES;
	phy_dev->advertising = phy_dev->supported;

	fep->phy_dev = phy_dev;
	fep->link = 0;
	fep->full_duplex = 0;

	printk(KERN_INFO "%s: Freescale FEC PHY driver [%s] "
		"(mii_bus:phy_addr=%s, irq=%d)\n", EDRV_DEV_NAME,
		fep->phy_dev->drv->name, dev_name(&fep->phy_dev->dev),
		fep->phy_dev->irq);

	return 0;
}

static struct mii_bus *fec_enet_mii_init(struct platform_device *pdev)
{
	struct fec_enet_private_t *fep = &fec_enet_private;
	int err = -ENXIO, i;

	fep->mii_timeout = 0;

	/*
	 * Set MII speed to 2.5 MHz (= clk_get_rate() / 2 * phy_speed)
	 */
	fep->phy_speed = DIV_ROUND_UP(clk_get_rate(fep->clk), 5000000) << 1;
#ifdef CONFIG_ARCH_MXS
	/* Can't get phy(8720) ID when set to 2.5M on MX28, lower it*/
	fep->phy_speed <<= 2;
#endif
	writel(fep->phy_speed, fep->hwp + FEC_MII_SPEED);

	fep->mii_bus = mdiobus_alloc();
	if (fep->mii_bus == NULL) {
		err = -ENOMEM;
		goto err_out;
	}

	fep->mii_bus->name = "fec_enet_mii_bus";
	fep->mii_bus->read = fec_enet_mdio_read;
	fep->mii_bus->write = fec_enet_mdio_write;
	fep->mii_bus->reset = fec_enet_mdio_reset;
	snprintf(fep->mii_bus->id, MII_BUS_ID_SIZE, "%x", pdev->id);
	fep->mii_bus->priv = fep;
	fep->mii_bus->parent = &pdev->dev;

	fep->mii_bus->irq = kmalloc(sizeof(int) * PHY_MAX_ADDR, GFP_KERNEL);
	if (!fep->mii_bus->irq) {
		err = -ENOMEM;
		goto err_out_free_mdiobus;
	}

	for (i = 0; i < PHY_MAX_ADDR; i++)
		fep->mii_bus->irq[i] = PHY_POLL;

	if (mdiobus_register(fep->mii_bus))
		goto err_out_free_mdio_irq;

	return fep->mii_bus;

err_out_free_mdio_irq:
	kfree(fep->mii_bus->irq);
err_out_free_mdiobus:
	mdiobus_free(fep->mii_bus);
err_out:
	return ERR_PTR(err);
}

static void fec_enet_mii_remove(void)
{
	struct fec_enet_private_t *fep = &fec_enet_private;

	if (fep->phy_dev)
		phy_disconnect(fep->phy_dev);
	mdiobus_unregister(fep->mii_bus);
	kfree(fep->mii_bus->irq);
	mdiobus_free(fep->mii_bus);
}

static void fec_free_buffers(void)
{
	struct fec_enet_private_t *fep = &fec_enet_private;
	struct bufdesc	*bdp;
	unsigned int    uiOrder;
	unsigned long   ulBufferPointer;
	unsigned int    nRxBuffer;

	if ((EDRV_RX_BUFFER_SIZE_SHIFT - PAGE_SHIFT) >= 0)
	{	/* rx-buffer is larger than or equal to page size */
		uiOrder = EDRV_RX_BUFFER_SIZE_SHIFT - PAGE_SHIFT;
	}
	else
	{	/* Multiple rx-buffers fit into one page */
		uiOrder = 0;
	}

	bdp = fep->rx_bd_base;
	for (nRxBuffer = 0; nRxBuffer < RX_RING_SIZE; nRxBuffer++)
	{
		if (bdp->cbd_bufaddr) {
			dma_unmap_single(&fep->pdev->dev, bdp->cbd_bufaddr,
					EDRV_RX_BUFFER_SIZE, DMA_FROM_DEVICE);
			ulBufferPointer = (unsigned long)(__u8*)__va(bdp->cbd_bufaddr);

#warning "What about higher order?"
			if ((uiOrder == 0) && ((ulBufferPointer & ((1UL << PAGE_SHIFT)-1)) == 0))
			{
				free_pages(ulBufferPointer, uiOrder);
				fep->m_iPageAllocations--;
			}
		}
		bdp++;
	}

	if (fep->m_iFreeRxBufferTop < EDRV_MAX_RX_BUFFERS-RX_RING_SIZE-1)
	{
#warning "There should be a - operation"
		printk("%s: %d rx-buffers were lost\n", EDRV_DEV_NAME, fep->m_iFreeRxBufferTop);
	}

	for (; fep->m_iFreeRxBufferTop >= 0; fep->m_iFreeRxBufferTop--)
	{
		ulBufferPointer = (unsigned long) fep->m_apbFreeRxBuffer[fep->m_iFreeRxBufferTop];

#warning "What about higher order?"
		if ((uiOrder == 0) && ((ulBufferPointer & ((1UL << PAGE_SHIFT)-1)) == 0))
		{
			free_pages(ulBufferPointer, uiOrder);
			fep->m_iPageAllocations--;
		}
	}

	if (fep->m_iPageAllocations > 0)
	{
		printk("%s: Less pages freed than allocated (%d)\n", EDRV_DEV_NAME, fep->m_iPageAllocations);
	}
	else if (fep->m_iPageAllocations < 0)
	{
		printk("%s: Attempted to free more pages than allocated (%d)\n", EDRV_DEV_NAME, (fep->m_iPageAllocations * -1));
	}

	/* Deallocate tx-buffers */
	bdp = fep->tx_bd_base;
#warning dma_unmap?
	kfree(fep->m_pbTxBufferBase);
}

static int fec_alloc_buffers(void)
{
	struct fec_enet_private_t *fep = &fec_enet_private;
	int i;
	struct bufdesc	*bdp;
	unsigned int    uiOrder;
	unsigned int    uiRxBuffersInAllocation;
	unsigned int    nRxBuffer;
	BYTE*           apbDescRxBuffer[RX_RING_SIZE];

	/* Allocate rx-buffers */
	if ((EDRV_RX_BUFFER_SIZE_SHIFT - PAGE_SHIFT) >= 0)
	{	/* rx-buffer is larger than or equal to page size */
		uiOrder = EDRV_RX_BUFFER_SIZE_SHIFT - PAGE_SHIFT;
		uiRxBuffersInAllocation = 1;
	}
	else
	{	/* Multiple rx-buffers fit into one page */
		uiOrder = 0;
		uiRxBuffersInAllocation = 1 << (PAGE_SHIFT - EDRV_RX_BUFFER_SIZE_SHIFT);
	}

	for (nRxBuffer = 0; nRxBuffer < EDRV_MAX_RX_BUFFERS; )
	{
		unsigned long   ulBufferPointer;
		unsigned int    nInAlloc;

		ulBufferPointer = __get_free_pages(GFP_KERNEL, uiOrder);
		if (ulBufferPointer == 0)
		{
			fec_free_buffers();
			return -ENOMEM;
		}
		fep->m_iPageAllocations++;

		for (nInAlloc = 0; nInAlloc < uiRxBuffersInAllocation; nInAlloc++)
		{
			if (nRxBuffer < RX_RING_SIZE)
			{	/* Store rx-buffer for rx-descriptor */
				apbDescRxBuffer[nRxBuffer] = (BYTE*) ulBufferPointer;
			}
			else
			{	/* Insert rx-buffer in free-rx-buffer stack */
				fep->m_apbFreeRxBuffer[nRxBuffer - RX_RING_SIZE] = (BYTE*) ulBufferPointer;
			}

			nRxBuffer++;
			ulBufferPointer += EDRV_RX_BUFFER_SIZE;
		}
	}

	fep->m_iFreeRxBufferTop = EDRV_MAX_RX_BUFFERS - RX_RING_SIZE - 1;
#if EDRV_USE_DIAGNOSTICS != FALSE
	fep->m_iFreeRxBufferMin = EDRV_MAX_RX_BUFFERS - RX_RING_SIZE;
#endif


	bdp = fep->rx_bd_base;
	for (i = 0; i < RX_RING_SIZE; i++) {
		bdp->cbd_bufaddr = dma_map_single(&fep->pdev->dev, (void*) apbDescRxBuffer[i],
				EDRV_RX_BUFFER_SIZE, DMA_FROM_DEVICE);
		bdp->cbd_sc = BD_ENET_RX_EMPTY;
#ifdef CONFIG_ENHANCED_BD
		bdp->cbd_esc = BD_ENET_RX_INT;
#endif
		bdp++;
	}

	/* Set the last buffer to wrap. */
	bdp--;
	bdp->cbd_sc |= BD_SC_WRAP;


	bdp = fep->tx_bd_base;
	for (i = 0; i < TX_RING_SIZE; i++) {
		bdp->cbd_sc = 0;
		bdp->cbd_bufaddr = 0;
#ifdef CONFIG_ENHANCED_BD
		bdp->cbd_esc = BD_ENET_TX_INT;
#endif
		bdp++;
	}

	/* Set the last buffer to wrap. */
	bdp--;
	bdp->cbd_sc |= BD_SC_WRAP;

	/* Allocate tx-buffers */
	fep->m_pbTxBufferBase = kmalloc(EDRV_MAX_TX_BUFFERS * EDRV_MAX_FRAME_SIZE, GFP_KERNEL);
	if (fep->m_pbTxBufferBase == NULL)
	{
		printk("%s: allocation of memory for tx-buffers failed\n", EDRV_DEV_NAME);
		fec_free_buffers();
		return -ENOMEM;
	}

	/* Mark all tx-buffers as unused */
	for (i = 0; i < EDRV_MAX_TX_BUFFERS; i++)
		fep->m_afTxBufferUsed[i] = FALSE;

	return 0;
}

static int fec_mac_addr_setup(char *mac_addr)
{
	char *ptr, *p = mac_addr;
	unsigned long tmp;
	int i = 0, ret = 0;

	while (p && (*p) && i < 6) {
		ptr = strchr(p, ':');
		if (ptr)
			*ptr++ = '\0';

		if (strlen(p)) {
			ret = kstrtoul(p, 16, &tmp);
			if (ret < 0 || tmp > 0xff)
				break;
			fec_mac_default[i++] = tmp;
		}
		p = ptr;
	}

	return 0;
}

__setup("fec_mac=", fec_mac_addr_setup);


/* This function is called to start or restart the FEC during a link
 * change.  This only happens when switching between half and full
 * duplex.
 */
static void  fec_restart (int duplex)
{
	struct fec_enet_private_t *fep = &fec_enet_private;
	int i;
	u32 temp_mac[2];
	unsigned long reg;
	int val;

	/* Whack a reset.  We should wait for this. */
	writel(1, fep->hwp + FEC_ECNTRL);
	udelay(10);

	/* Reset fec will reset MAC to zero, reconfig it again */
	memcpy(&temp_mac, fec_enet_private.m_InitParam.aMacAddr, ETH_ALEN);
	writel(cpu_to_be32(temp_mac[0]), fep->hwp + FEC_ADDR_LOW);
	writel(cpu_to_be32(temp_mac[1]), fep->hwp + FEC_ADDR_HIGH);

	/* Clear any outstanding interrupt. */
	writel(0xffc00000, fep->hwp + FEC_IEVENT);

	/* Reset all multicast.	*/
	writel(0, fep->hwp + FEC_GRP_HASH_TABLE_HIGH);
	writel(0, fep->hwp + FEC_GRP_HASH_TABLE_LOW);
#ifndef CONFIG_M5272
	writel(0, fep->hwp + FEC_HASH_TABLE_HIGH);
	writel(0, fep->hwp + FEC_HASH_TABLE_LOW);
#endif

	/* Set maximum receive buffer size. */
	writel(PKT_MAXBLR_SIZE, fep->hwp + FEC_R_BUFF_SIZE);

	/* Set receive and transmit descriptor base. */
	writel(fep->bd_dma, fep->hwp + FEC_R_DES_START);
	writel((unsigned long)fep->bd_dma + sizeof(struct bufdesc) * RX_RING_SIZE,
			fep->hwp + FEC_X_DES_START);

	fep->dirty_tx = fep->cur_tx = fep->tx_bd_base;
	fep->cur_rx = fep->rx_bd_base;

	/* Reset SKB transmit buffers. */
	fep->txb_cur = fep->txb_dirty = 0;
	for (i = 0; i <= TX_RING_MOD_MASK; i++)
		fep->m_apTxBuffer[i] = NULL;

	/* Enable MII mode */
	if (duplex) {
		/* MII enable / FD enable */
		writel(OPT_FRAME_SIZE | 0x04, fep->hwp + FEC_R_CNTRL);
		writel(0x04, fep->hwp + FEC_X_CNTRL);
	} else {
		/* MII enable / No Rcv on Xmit */
		writel(OPT_FRAME_SIZE | 0x06, fep->hwp + FEC_R_CNTRL);
		writel(0x0, fep->hwp + FEC_X_CNTRL);
	}
	fep->full_duplex = duplex;

#ifdef CONFIG_ARCH_MXS

	reg = readl(fep->hwp + FEC_R_CNTRL);

	/* Enable flow control and length check */
	reg |= (0x40000000 | 0x00000020);

	/* Check MII or RMII */
	if (fep->phy_interface == PHY_INTERFACE_MODE_RMII)
		reg |= 0x00000100;
	else
		reg &= ~0x00000100;

	/* Check 10M or 100M */
	if (fep->phy_dev && fep->phy_dev->speed == SPEED_100)
		reg &= ~0x00000200; /* 100M */
	else
		reg |= 0x00000200;  /* 10M */

	writel(reg, fep->hwp + FEC_R_CNTRL);

#endif
	/* Set MII speed */
	writel(fep->phy_speed, fep->hwp + FEC_MII_SPEED);

	reg = 0x0;

#ifdef FEC_MIIGSK_ENR
	if (fep->phy_interface == PHY_INTERFACE_MODE_RMII) {
		/* disable the gasket and wait */
		writel(0, fep->hwp + FEC_MIIGSK_ENR);
		while (readl(fep->hwp + FEC_MIIGSK_ENR) & 4)
			udelay(1);

		/* configure the gasket: RMII, 50 MHz, no loopback, no echo */
		val = 1;
		if (fep->phy_dev && fep->phy_dev->speed == SPEED_10)
			val |= 1 << 6;
		writel(val, fep->hwp + FEC_MIIGSK_CFGR);

		/* re-enable the gasket */
		writel(2, fep->hwp + FEC_MIIGSK_ENR);
	}
#endif

	/* And last, enable the transmit and receive processing */
	reg |= 0x00000002;
	writel(reg, fep->hwp + FEC_ECNTRL);
	writel(0, fep->hwp + FEC_R_DES_ACTIVE);

	/* Enable promisc mode */
	reg = readl(fep->hwp + FEC_R_CNTRL);
	reg |= 0x8;
	writel(reg, fep->hwp + FEC_R_CNTRL);

	/* Enable interrupts we wish to service */
	writel(FEC_DEFAULT_IMASK, fep->hwp + FEC_IMASK);
}

static void  fec_stop (void)
{
	struct fec_enet_private_t *fep = &fec_enet_private;

	/* We cannot expect a graceful transmit stop without link !!! */
	if (fep->link) {
		writel(1, fep->hwp + FEC_X_CNTRL); /* Graceful transmit stop */
		udelay(10);
		if (!(readl(fep->hwp + FEC_IEVENT) & FEC_ENET_GRA))
			printk("fec_stop : Graceful transmit stop did not complete !\n");
	}

	/* Whack a reset.  We should wait for this. */
	writel(1, fep->hwp + FEC_ECNTRL);
	udelay(10);

#ifdef CONFIG_ARCH_MXS
	/* Check MII or RMII */
	if (fep->phy_interface == PHY_INTERFACE_MODE_RMII)
		writel(readl(fep->hwp + FEC_R_CNTRL) | 0x100,
					fep->hwp + FEC_R_CNTRL);
	else
		writel(readl(fep->hwp + FEC_R_CNTRL) & ~0x100,
					fep->hwp + FEC_R_CNTRL);
#endif
	/* Clear outstanding MII command interrupts. */
	writel(FEC_ENET_MII, fep->hwp + FEC_IEVENT);
	writel(fep->phy_speed, fep->hwp + FEC_MII_SPEED);
	writel(FEC_DEFAULT_IMASK, fep->hwp + FEC_IMASK);
}

// pnf@mar2016
#define IRQF_DISABLED           0x00000020

static int
fec_probe(struct platform_device *pdev)
{
	struct fec_enet_private_t *fep = &fec_enet_private;
	struct fec_platform_data *pdata;
	struct mii_bus *fec_mii_bus;
	int i, irq, ret = 0;
	struct resource *r;
	struct bufdesc *cbd_base;
	struct bufdesc *bdp;

	printk("%s entered\n", __FUNCTION__);

	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!r) {
		printk("%s: platform_get_resource() failed\n", EDRV_DEV_NAME);
		return -ENXIO;
	}

	r = request_mem_region(r->start, resource_size(r), pdev->name);
	if (!r) {
		printk("%s: request_mem_region() failed\n", EDRV_DEV_NAME);
		return -EBUSY;
	}

	/* setup board info structure */
	fep->hwp  = ioremap(r->start, resource_size(r));
	fep->pdev = pdev;

	if (fep->hwp == NULL) {
		ret = -ENOMEM;
		goto failed_ioremap;
	}

	pdata = pdev->dev.platform_data;
	if (pdata)
		fep->phy_interface = pdata->phy;

	/* This device has up to three irqs on some platforms */
	for (i = 0; i < 3; i++) {
		irq = platform_get_irq(pdev, i);
		if (i && irq < 0)
			break;
		ret = request_irq(irq, fec_enet_interrupt, IRQF_DISABLED, pdev->name, pdev);
		if (ret) {
			while (i >= 0) {
				irq = platform_get_irq(pdev, i);
				free_irq(irq, pdev);
				i--;
			}
			goto failed_irq;
		}
	}

	fep->clk = clk_get(&pdev->dev, "fec_clk");
	if (IS_ERR(fep->clk)) {
		ret = PTR_ERR(fep->clk);
		goto failed_clk;
	}

	clk_enable(fep->clk);

	/* PHY reset should be done during clock on */
// pnf@mar2016
//	if (pdata && pdata->init) {
//		ret = pdata->init();
//		if (ret) {
//			printk("%s: platform_data init() failed (returned %u)\n", EDRV_DEV_NAME, ret);
//			goto failed_platform_init;
//		}
//	}

	/*
	 * The priority for getting MAC address is:
	 * (1) kernel command line fec_mac = xx:xx:xx...
	 * (2) platform data mac field got from fuse etc
	 * (3) bootloader set the FEC mac register
	 */

	if (pdata && !is_valid_ether_addr(fec_mac_default) &&
		pdata->mac && is_valid_ether_addr(pdata->mac))
		memcpy(fec_mac_default, pdata->mac, sizeof(fec_mac_default));

	/* Allocate memory for buffer descriptors. */
	cbd_base = dma_alloc_coherent(NULL, PAGE_SIZE, &fep->bd_dma,
			GFP_KERNEL);
	if (!cbd_base) {
		printk("%s: allocate descriptor memory failed?\n", EDRV_DEV_NAME);
		ret = -ENOMEM;
		goto failed_alloc_desc;
	}

	printk("%s: dma_alloc_coherent done\n", __FUNCTION__);

	spin_lock_init(&fep->hw_lock);

	/* Set the Ethernet address */
	fec_get_mac(fec_enet_private.m_InitParam.aMacAddr);

	/* Set receive and transmit descriptor base. */
	fep->rx_bd_base = cbd_base;
	fep->tx_bd_base = cbd_base + RX_RING_SIZE;

	/* Initialize the receive buffer descriptors. */
	bdp = fep->rx_bd_base;
	for (i = 0; i < RX_RING_SIZE; i++) {

		/* Initialize the BD for every fragment in the page. */
		bdp->cbd_sc = 0;
		bdp++;
	}

	/* Set the last buffer to wrap */
	bdp--;
	bdp->cbd_sc |= BD_SC_WRAP;

	/* ...and the same for transmit */
	bdp = fep->tx_bd_base;
	for (i = 0; i < TX_RING_SIZE; i++) {

		/* Initialize the BD for every fragment in the page. */
		bdp->cbd_sc = 0;
		bdp->cbd_bufaddr = 0;
		bdp++;
	}

	/* Set the last buffer to wrap */
	bdp--;
	bdp->cbd_sc |= BD_SC_WRAP;

	ret = fec_alloc_buffers();
	if (ret)
		goto failed_alloc_buffers;

	printk("%s: fec_alloc_buffers done\n", __FUNCTION__);

	fec_restart(0);

	fec_mii_bus = fec_enet_mii_init(pdev);
	if (IS_ERR(fec_mii_bus)) {
		ret = -ENOMEM;
		goto failed_mii_init;
	}

	printk("%s: fec_enet_mii_init done\n", __FUNCTION__);

	/* Probe and connect to PHY */
	ret = fec_enet_mii_probe();
	if (ret) {
		goto failed_mii_probe;
	}

	printk("%s: fec_enet_mii_probe done\n", __FUNCTION__);

	phy_start(fep->phy_dev);
	fec_restart(fep->phy_dev->duplex);

	return 0;

failed_mii_probe:
	fec_enet_mii_remove();
failed_mii_init:
	fec_free_buffers();
failed_alloc_buffers:
#warning "call counterpart to dma_alloc_coherent()"
failed_alloc_desc:
	clk_disable(fep->clk);
	clk_put(fep->clk);
failed_clk:
	for (i = 0; i < 3; i++) {
		irq = platform_get_irq(pdev, i);
		if (irq > 0)
			free_irq(irq, pdev);
	}
failed_irq:
	iounmap(fep->hwp);
failed_ioremap:
failed_platform_init:

	return ret;
}

static int
fec_remove(struct platform_device *pdev)
{
	struct fec_enet_private_t *fep = &fec_enet_private;
	struct fec_platform_data *pdata = pdev->dev.platform_data;
	struct resource *r;
	int i, irq;

	printk("%s entered\n", __FUNCTION__);

	fec_stop();
	printk("%s: fec_stop() done\n", __FUNCTION__);

	if (fep->phy_dev) {
		phy_stop(fep->phy_dev);
	}
	printk("%s: phy_stop() done\n", __FUNCTION__);

        fec_free_buffers();
	printk("%s: fec_free_buffers() done\n", __FUNCTION__);

	clk_disable(fep->clk);
	fec_enet_mii_remove();
	printk("%s: fec_enet_mii_remove() done\n", __FUNCTION__);
// pnf@mar2016
//	if (pdata && pdata->uninit)
//		pdata->uninit();
	clk_disable(fep->clk);
	clk_put(fep->clk);

	for (i = 0; i < 3; i++) {
		irq = platform_get_irq(pdev, i);
		if (irq > 0)
			free_irq(irq, pdev);
	}

	iounmap(fep->hwp);

	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!r) {
		printk("%s: platform_get_resource() failed\n", EDRV_DEV_NAME);
		return -ENXIO;
	}

	release_mem_region(r->start, resource_size(r));

	printk("%s: fec_remove() done\n", __FUNCTION__);

	return 0;
}

static int
fec_suspend(struct platform_device *dev, pm_message_t state)
{
	struct fec_enet_private_t *fep = &fec_enet_private;

	fec_stop();
	clk_disable(fep->clk);

	return 0;
}

static int
fec_resume(struct platform_device *dev)
{
	struct fec_enet_private_t *fep = &fec_enet_private;

	clk_enable(fep->clk);
	fec_restart(fep->full_duplex);

	return 0;
}

MODULE_LICENSE("GPL");

