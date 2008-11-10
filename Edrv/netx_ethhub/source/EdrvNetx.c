/****************************************************************************

  (c) SYSTEC electronic GmbH, D-07973 Greiz, August-Bebel-Str. 29
      www.systec-electronic.com

  Project:      openPOWERLINK

  Description:  Ethernet driver for Hilscher netX-500 using its embedded
                xC units as a 2-port Ethernet hub (xC 0 and xC 1).

  License:

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License version 2
    as published by the Free Software Foundation.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

  -------------------------------------------------------------------------

                $RCSfile$

                $Author$

                $Revision$  $Date$

                $State$

                Build Environment:
                GNU GCC for ARM under Linux

  -------------------------------------------------------------------------

  Revision History:

  2008/11/04 d.k.:   start of implementation

****************************************************************************/

#include "global.h"
#include "EplInc.h"
#include "edrv.h"

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
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

#include <asm/hardware.h>
#include <asm/arch/hardware.h>
#include <asm/arch/netx-regs.h>
#include <asm/arch/pfifo.h>
#include <asm/arch/xc.h>
#include <asm/arch/eth.h>

// include the register definition file of the 2port Ethernet Hub
#include "eth_2port_hub_xpec_regdef.h"


/***************************************************************************/
/*                                                                         */
/*                                                                         */
/*          G L O B A L   D E F I N I T I O N S                            */
/*                                                                         */
/*                                                                         */
/***************************************************************************/

// Buffer handling:
// All buffers are created statically (i.e. at compile time resp. at
// initialisation via kmalloc() ) and not dynamically on request (i.e. via
// EdrvAllocTxMsgBuffer().
// EdrvAllocTxMsgBuffer() searches for an unused buffer which is large enough.
// EdrvInit() may allocate some buffers with sizes less than maximum frame
// size (i.e. 1514 bytes), e.g. for SoC, SoA, StatusResponse, IdentResponse,
// NMT requests / commands. The less the size of the buffer the less the
// number of the buffer.


//---------------------------------------------------------------------------
// const defines
//---------------------------------------------------------------------------

#ifndef EDRV_MAX_TX_BUFFERS
#define EDRV_MAX_TX_BUFFERS     20
#endif

#define EDRV_MAX_FRAME_SIZE     0x600

//#define EDRV_RX_BUFFER_SIZE     0x8610  // 32 kB + 16 Byte + 1,5 kB (WRAP is enabled)
//#define EDRV_RX_BUFFER_LENGTH   (EDRV_RX_BUFFER_SIZE & 0xF800)  // buffer size cut down to 2 kB alignment

#define EDRV_TX_BUFFER_SIZE     (EDRV_MAX_TX_BUFFERS * EDRV_MAX_FRAME_SIZE) // n * (MTU + 14 + 4)

#define DRV_NAME                "netx-eth"

#define EDRV_MIN_FIFO_EMPTY_ENTRIES     3   // the minimum number of entries for
                                            // frame reception in empty FIFO

#define EDRV_FIFO_OVERFLOW_MASK         ((1 << ETHERNET_FIFO_IND_LO) \
                                        | (1 << ETHERNET_FIFO_IND_HI))


// defines from hal_eth_2port_hub.c

#define ETHERNET_MINIMUM_FRAMELENGTH                    60
#define ETHERNET_MAXIMUM_FRAMELENGTH                    1518

#define NUM_FIFO_CHANNELS_PER_UNIT                      8      /* Number of FIFO units per XC channel */
#define FIFO_ENTRIES                                    64     /* FIFO depth for each of the 8 FIFOs  */
#define ETH_FRAME_BUF_SIZE                              1560   /* size of a frame buffer     */
#define INTRAM_SEGMENT_SIZE                             0x8000 /* size of the internal ram segments */

#define ETHERNET_FIFO_EMPTY                             0 /* Empty pointer FIFO               */
#define ETHERNET_FIFO_IND_HI                            1 /* High priority indication FIFO    */
#define ETHERNET_FIFO_IND_LO                            2 /* Low priority indication FIFO     */
#define ETHERNET_FIFO_REQ_HI                            3 /* High priority request FIFO       */
#define ETHERNET_FIFO_REQ_LO                            4 /* Low priority request FIFO        */
#define ETHERNET_FIFO_CON_HI                            5 /* High priority confirmation FIFO  */
#define ETHERNET_FIFO_CON_LO                            6 /* Low priority confirmation FIFO   */

/* confirmation error codes */
#define CONF_ERRCODE_TX_SUCCESSFUL_WITHOUT_RETRIES      0x0 /* Confirmation: Success on first try   */
#define CONF_ERRCODE_TX_SUCCESSFUL_WITH_RETRIES         0x1 /* Confirmation: Success after some retries */
#define CONF_ERRCODE_TX_FAILED_LATE_COLLISION           0x8 /* Confirmation: Error (late collision) */
#define CONF_ERRCODE_TX_FAILED_LINK_DOWN_DURING_TX      0x9 /* Confirmation: Error (link down)      */
#define CONF_ERRCODE_TX_FAILED_EXCESSIVE_COLLISION      0xa /* Confirmation: Error (collision)      */
#define CONF_ERRCODE_TX_FAILED_UTX_UFL_DURING_TX        0xb /* Confirmation: Error (FIFO overflow)  */
#define CONF_ERRCODE_TX_FAILED_FATAL_ERROR              0xc /* Confirmation: Error (Fatal)  */

#define ETHHUB_FIFO_START                               0
#define ETHHUB_FIFO_END                                16


// defines from main.c of HAL demo
#define ACTIVITY_LED_FLASH_PERIOD 5000000 /* 50 ms */


// defines from netx-eth.c of Linux
#define PFIFO_MASK(xcno)        (0x7f << (xcno*8))


// defines for register accesses
#define EDRV_XPECRAM_WRITE(dwReg, dwVal) \
    writel(dwVal, EdrvInstance_l.m_pXpecBase + NETX_XPEC_RAM_START_OFS + (dwReg))
#define EDRV_XPECRAM_READ(dwReg) \
    readl(EdrvInstance_l.m_pXpecBase + NETX_XPEC_RAM_START_OFS + (dwReg))

//#define EDRV_REGW_WRITE(dwReg, wVal)    writew(wVal, EdrvInstance_l.m_pIoAddr + dwReg)
//#define EDRV_REGB_WRITE(dwReg, bVal)    writeb(bVal, EdrvInstance_l.m_pIoAddr + dwReg)
//#define EDRV_REGW_READ(dwReg)           readw(EdrvInstance_l.m_pIoAddr + dwReg)
//#define EDRV_REGB_READ(dwReg)           readb(EdrvInstance_l.m_pIoAddr + dwReg)


// TracePoint support for realtime-debugging
#ifdef _DBG_TRACE_POINTS_
    void  PUBLIC  TgtDbgSignalTracePoint (BYTE bTracePointNumber_p);
    void  PUBLIC  TgtDbgPostTraceValue (DWORD dwTraceValue_p);
    #define TGT_DBG_SIGNAL_TRACE_POINT(p)   TgtDbgSignalTracePoint(p)
    #define TGT_DBG_POST_TRACE_VALUE(v)     TgtDbgPostTraceValue(v)
#else
    #define TGT_DBG_SIGNAL_TRACE_POINT(p)
    #define TGT_DBG_POST_TRACE_VALUE(v)
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

#define EDRV_TRACE_CAPR(x)              TGT_DBG_POST_TRACE_VALUE(((x) & 0xFFFF) | 0x06000000)
#define EDRV_TRACE_RX_CRC(x)            TGT_DBG_POST_TRACE_VALUE(((x) & 0xFFFF) | 0x0E000000)
#define EDRV_TRACE_RX_ERR(x)            TGT_DBG_POST_TRACE_VALUE(((x) & 0xFFFF) | 0x0F000000)
#define EDRV_TRACE_RX_PUN(x)            TGT_DBG_POST_TRACE_VALUE(((x) & 0xFFFF) | 0x11000000)
#define EDRV_TRACE(x)                   TGT_DBG_POST_TRACE_VALUE(((x) & 0xFFFF0000) | 0x0000FEC0)


//---------------------------------------------------------------------------
// local types
//---------------------------------------------------------------------------
/*
typedef struct
{
    BOOL            m_fUsed;
    unsigned int    m_uiSize;
    MCD_bufDescFec *m_pBufDescr;

} tEdrvTxBufferIntern;
*/

// Private structure
typedef struct
{
    struct platform_device* m_apDev[2]; // pointer array to platform device structure
    struct xc*              m_apXc[2];  // pointer array to xC unit structure

    void __iomem*           m_pXpecBase;// pointer to register space of xPEC unit
    void __iomem*           m_apSramBase[2];// pointer to SRAM of xPEC unit


    BYTE*               m_pbRxBuf;      // pointer to Rx buffer
    dma_addr_t          m_pRxBufDma;
    BYTE*               m_pbTxBuf;      // pointer to Tx buffer
    dma_addr_t          m_pTxBufDma;

    BOOL                m_afTxBufUsed[EDRV_MAX_TX_BUFFERS];
    unsigned int        m_uiCurTxDesc;

    tEdrvInitParam      m_InitParam;
    tEdrvTxBuffer*      m_pLastTransmittedTxBuffer;

} tEdrvInstance;



//---------------------------------------------------------------------------
// local function prototypes
//---------------------------------------------------------------------------

static int EdrvInitOne(struct platform_device *pPlatformDev_p);

static int EdrvRemoveOne(struct platform_device *pPlatformDev_p);


//---------------------------------------------------------------------------
// modul globale vars
//---------------------------------------------------------------------------
// buffers and buffer descriptors and pointers


static tEdrvInstance EdrvInstance_l;


static struct platform_driver EdrvDriver = {
    .probe          = EdrvInitOne,
    .remove         = EdrvRemoveOne,
//  .suspend = netx_eth_drv_suspend,
//  .resume = netx_eth_drv_resume,
    .driver         = {
        .name       = DRV_NAME,
        .owner      = THIS_MODULE,
    },
};





/***************************************************************************/
/*                                                                         */
/*                                                                         */
/*          C L A S S  <edrv>                                              */
/*                                                                         */
/*                                                                         */
/***************************************************************************/
//
// Description:
//
//
/***************************************************************************/


//=========================================================================//
//                                                                         //
//          P R I V A T E   D E F I N I T I O N S                          //
//                                                                         //
//=========================================================================//

//---------------------------------------------------------------------------
// const defines
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
// local types
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
// local vars
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
// local function prototypes
//---------------------------------------------------------------------------

//static BYTE EdrvCalcHash (BYTE * pbMAC_p);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,19)
static int TgtEthIsr (int nIrqNum_p, void* ppDevInstData_p);
#else
static int TgtEthIsr (int nIrqNum_p, void* ppDevInstData_p, struct pt_regs* ptRegs_p);
#endif



//---------------------------------------------------------------------------
//
// Function:    EdrvInit
//
// Description: function for init of the Ethernet controller
//
// Parameters:  pEdrvInitParam_p    = pointer to struct including the init-parameters
//
// Returns:     Errorcode           = kEplSuccessful
//                                  = kEplNoResource
//
// State:
//
//---------------------------------------------------------------------------
tEplKernel EdrvInit(tEdrvInitParam * pEdrvInitParam_p)
{
tEplKernel  Ret;
int         iResult;
unsigned int uiCount;

    Ret = kEplSuccessful;

    // clear instance structure
    EPL_MEMSET(&EdrvInstance_l, 0, sizeof (EdrvInstance_l));

    // save the init data
    EdrvInstance_l.m_InitParam = *pEdrvInitParam_p;


    // register platform driver
    iResult = platform_driver_register(&EdrvDriver);
    if (iResult != 0)
    {
        Ret = kEplNoResource;
        goto Exit;
    }

    if ((EdrvInstance_l.m_apDev[0] == NULL)
        || (EdrvInstance_l.m_apDev[1] == NULL))
    {
        printk("%s: the necessary two platform devices were not found\n",
                __FUNCTION__);
        Ret = kEplNoResource;
        goto Exit;
    }

    // both xCs have been initialized
    // now, configure global things

    // copy xPEC base address of unit 0 to instance structure
    EdrvInstance_l.m_pXpecBase = EdrvInstance_l.m_apXc[0]->xpec_base;

    // enable monitoring mode
    EDRV_XPECRAM_WRITE(REL_Adr_ETHHUB_MONITORING_MODE, MSK_ETHHUB_MONITORING_MODE_EN);

    // set default traffic class arrangement
    EDRV_XPECRAM_WRITE(REL_Adr_ETHHUB_TRAFFIC_CLASS_ARRANGEMENT, 4);

    // configure flash period
    EDRV_XPECRAM_WRITE(REL_Adr_ETHHUB_PHY_LEDS_FLASH_PERIOD, ACTIVITY_LED_FLASH_PERIOD);

    // copy the current systime border to the copy inside the hardware block
    EDRV_XPECRAM_WRITE(REL_Adr_ETHHUB_SYSTIME_BORDER_COPY,
            readl(NETX_VA_SYSTIME + 0x8));


    // confirm all IRQs
    writel(0xffff, NETX_PFIFO_XPEC_ISR(0));
    writel(0xffff, NETX_PFIFO_XPEC_ISR(1));

    // install IRQ handler
    iResult = request_irq(EdrvInstance_l.m_apXc[0]->irq,
                          &TgtEthIsr,
                          IRQF_SHARED,
                          DRV_NAME,
                          &EdrvInstance_l);
    if (iResult != 0)
    {
        printk("%s: request_irq(%d) failed with %d\n",
                __FUNCTION__,
                EdrvInstance_l.m_apXc[0]->irq,
                iResult);
        Ret = kEplNoResource;
        goto Exit;
    }


    // start the Ethernet Hub

    // wait for PHYs to be initialized
    writel(0x00030000, NETX_PFIFO_XPEC_ISR(1));
    uiCount = 100;

    while (readl(NETX_PFIFO_XPEC_ISR(1)) == 0x00030000)
    {   // do nothing
        msleep(300);
        uiCount--;
        if (uiCount == 0)
        {
            break;
        }
    }


    printk("%s: XPEC_ISR1 = 0x%X\n",
            __FUNCTION__,
            readl(NETX_PFIFO_XPEC_ISR(1)));

    // check success
    if (readl(NETX_PFIFO_XPEC_ISR(1)) != 0)
    {
        printk("%s: PHY init failed\n",
                __FUNCTION__);
//        Ret = kEplNoResource;
//        goto Exit;
    }


    // set my mac addresses
    EDRV_XPECRAM_WRITE(REL_Adr_ETHHUB_LOCAL_MAC_ADDRESS_HI,
            (pEdrvInitParam_p->m_abMyMacAddr[5] << 8)  |
            pEdrvInitParam_p->m_abMyMacAddr[4]);

    EDRV_XPECRAM_WRITE(REL_Adr_ETHHUB_LOCAL_MAC_ADDRESS_LO,
            (pEdrvInitParam_p->m_abMyMacAddr[3] << 24) |
            (pEdrvInitParam_p->m_abMyMacAddr[2] << 16) |
            (pEdrvInitParam_p->m_abMyMacAddr[1] << 8)  |
            pEdrvInitParam_p->m_abMyMacAddr[0]);

    // enable irqs on ethernet ports
    EDRV_XPECRAM_WRITE(REL_Adr_ETHHUB_INTERRUPTS_ENABLE_IND_HI,
            MSK_ETHHUB_INTERRUPTS_ENABLE_IND_HI_VAL);
    EDRV_XPECRAM_WRITE(REL_Adr_ETHHUB_INTERRUPTS_ENABLE_IND_LO,
            MSK_ETHHUB_INTERRUPTS_ENABLE_IND_LO_VAL);
    EDRV_XPECRAM_WRITE(REL_Adr_ETHHUB_INTERRUPTS_ENABLE_CON_HI,
            MSK_ETHHUB_INTERRUPTS_ENABLE_CON_HI_VAL);
    EDRV_XPECRAM_WRITE(REL_Adr_ETHHUB_INTERRUPTS_ENABLE_CON_LO,
            MSK_ETHHUB_INTERRUPTS_ENABLE_CON_LO_VAL);
    EDRV_XPECRAM_WRITE(REL_Adr_ETHHUB_INTERRUPTS_ENABLE_LINK_CHANGED,
            MSK_ETHHUB_INTERRUPTS_ENABLE_LINK_CHANGED_VAL);
    EDRV_XPECRAM_WRITE(REL_Adr_ETHHUB_INTERRUPTS_ENABLE_RX_ERR,
            MSK_ETHHUB_XPEC2ARM_INTERRUPTS_RX_ERR);


Exit:
    return Ret;

}


//---------------------------------------------------------------------------
//
// Function:    EdrvShutdown
//
// Description: Shutdown the Ethernet controller
//
// Parameters:  void
//
// Returns:     Errorcode   = kEplSuccessful
//
// State:
//
//---------------------------------------------------------------------------
tEplKernel EdrvShutdown(void)
{

    // free global resources
    if (EdrvInstance_l.m_apXc[0] != NULL)
    {
        // disable irqs on ethernet ports
        EDRV_XPECRAM_WRITE(REL_Adr_ETHHUB_INTERRUPTS_ENABLE_IND_HI, 0);
        EDRV_XPECRAM_WRITE(REL_Adr_ETHHUB_INTERRUPTS_ENABLE_IND_LO, 0);
        EDRV_XPECRAM_WRITE(REL_Adr_ETHHUB_INTERRUPTS_ENABLE_CON_HI, 0);
        EDRV_XPECRAM_WRITE(REL_Adr_ETHHUB_INTERRUPTS_ENABLE_CON_LO, 0);
        EDRV_XPECRAM_WRITE(REL_Adr_ETHHUB_INTERRUPTS_ENABLE_LINK_CHANGED, 0);
        EDRV_XPECRAM_WRITE(REL_Adr_ETHHUB_INTERRUPTS_ENABLE_RX_ERR, 0);

        // free IRQ
        free_irq(EdrvInstance_l.m_apXc[0]->irq, &EdrvInstance_l);
    }

    // unregister platform driver
    platform_driver_unregister(&EdrvDriver);

    return kEplSuccessful;
}


//---------------------------------------------------------------------------
//
// Function:    EdrvDefineRxMacAddrEntry
//
// Description: Set a multicast entry into the Ethernet controller
//
// Parameters:  pbMacAddr_p     = pointer to multicast entry to set
//
// Returns:     Errorcode       = kEplSuccessful
//
// State:
//
//---------------------------------------------------------------------------
tEplKernel EdrvDefineRxMacAddrEntry (BYTE * pbMacAddr_p)
{
tEplKernel  Ret = kEplSuccessful;
/*
DWORD       dwData;
BYTE        bHash;

    bHash = EdrvCalcHash (pbMacAddr_p);

    if (bHash > 31)
    {
        dwData = EDRV_REGDW_READ(EDRV_REGDW_MAR4);
        dwData |= 1 << (bHash - 32);
        EDRV_REGDW_WRITE(EDRV_REGDW_MAR4, dwData);
    }
    else
    {
        dwData = EDRV_REGDW_READ(EDRV_REGDW_MAR0);
        dwData |= 1 << bHash;
        EDRV_REGDW_WRITE(EDRV_REGDW_MAR0, dwData);
    }
*/
    return Ret;
}


//---------------------------------------------------------------------------
//
// Function:    EdrvUndefineRxMacAddrEntry
//
// Description: Reset a multicast entry in the Ethernet controller
//
// Parameters:  pbMacAddr_p     = pointer to multicast entry to reset
//
// Returns:     Errorcode       = kEplSuccessful
//
// State:
//
//---------------------------------------------------------------------------
tEplKernel EdrvUndefineRxMacAddrEntry (BYTE * pbMacAddr_p)
{
tEplKernel  Ret = kEplSuccessful;
/*
DWORD       dwData;
BYTE        bHash;

    bHash = EdrvCalcHash (pbMacAddr_p);

    if (bHash > 31)
    {
        dwData = EDRV_REGDW_READ(EDRV_REGDW_MAR4);
        dwData &= ~(1 << (bHash - 32));
        EDRV_REGDW_WRITE(EDRV_REGDW_MAR4, dwData);
    }
    else
    {
        dwData = EDRV_REGDW_READ(EDRV_REGDW_MAR0);
        dwData &= ~(1 << bHash);
        EDRV_REGDW_WRITE(EDRV_REGDW_MAR0, dwData);
    }
*/
    return Ret;
}

//---------------------------------------------------------------------------
//
// Function:    EdrvAllocTxMsgBuffer
//
// Description: Register a Tx-Buffer
//
// Parameters:  pBuffer_p   = pointer to Buffer structure
//
// Returns:     Errorcode   = kEplSuccessful
//                          = kEplEdrvNoFreeBufEntry
//
// State:
//
//---------------------------------------------------------------------------
tEplKernel EdrvAllocTxMsgBuffer       (tEdrvTxBuffer * pBuffer_p)
{
tEplKernel Ret = kEplSuccessful;
ETHHUB_FIFO_ELEMENT_T tFifoPtr;
unsigned int uiRamSegment;
unsigned int uiFrameNr;
//DWORD i;

    if (pBuffer_p->m_uiMaxBufferLen > EDRV_MAX_FRAME_SIZE)
    {
        Ret = kEplEdrvNoFreeBufEntry;
        goto Exit;
    }

    // check if enough free FIFO elements are available
    if (pfifo_fill_level(ETHERNET_FIFO_EMPTY)
        <= EDRV_MIN_FIFO_EMPTY_ENTRIES)
    {   // not enough free elements
        Ret = kEplEdrvNoFreeBufEntry;
        goto Exit;
    }

    // retrieve the fifo element from the empty pointer FIFO
    tFifoPtr.val = pfifo_pop(ETHERNET_FIFO_EMPTY);

    // extract ram bank and frame number
    uiRamSegment = tFifoPtr.bf.INT_RAM_SEGMENT_NUM;
    uiFrameNr = tFifoPtr.bf.FRAME_BUF_NUM;

    // set result
    pBuffer_p->m_pbBuffer = EdrvInstance_l.m_apSramBase[uiRamSegment]
                            + (ETH_FRAME_BUF_SIZE * uiFrameNr);
    pBuffer_p->m_uiMaxBufferLen = EDRV_MAX_FRAME_SIZE;
    pBuffer_p->m_uiBufferNumber = tFifoPtr.val;

/*
    // search a free Tx buffer with appropriate size
    for (i = 0; i < EDRV_MAX_TX_BUFFERS; i++)
    {
        if (EdrvInstance_l.m_afTxBufUsed[i] == FALSE)
        {
            // free channel found
            EdrvInstance_l.m_afTxBufUsed[i] = TRUE;
            pBuffer_p->m_uiBufferNumber = i;
            pBuffer_p->m_pbBuffer = EdrvInstance_l.m_pbTxBuf + (i * EDRV_MAX_FRAME_SIZE);
            pBuffer_p->m_uiMaxBufferLen = EDRV_MAX_FRAME_SIZE;
            break;
        }
    }
    if (i >= EDRV_MAX_TX_BUFFERS)
    {
        Ret = kEplEdrvNoFreeBufEntry;
        goto Exit;
    }
*/

Exit:
    return Ret;

}


//---------------------------------------------------------------------------
//
// Function:    EdrvReleaseTxMsgBuffer
//
// Description: Register a Tx-Buffer
//
// Parameters:  pBuffer_p   = pointer to Buffer structure
//
// Returns:     Errorcode   = kEplSuccessful
//
// State:
//
//---------------------------------------------------------------------------
tEplKernel EdrvReleaseTxMsgBuffer     (tEdrvTxBuffer * pBuffer_p)
{
unsigned int uiBufferNumber;

    uiBufferNumber = pBuffer_p->m_uiBufferNumber;

    if (uiBufferNumber != 0)
    {
        // put the FIFO element back to the empty pointer FIFO
        pfifo_push(ETHERNET_FIFO_EMPTY, uiBufferNumber);
    }

    return kEplSuccessful;

}


//---------------------------------------------------------------------------
//
// Function:    EdrvSendTxMsg
//
// Description: immediately starts the transmission of the buffer
//
// Parameters:  pBuffer_p   = buffer descriptor to transmit
//
// Returns:     Errorcode   = kEplSuccessful
//
// State:
//
//---------------------------------------------------------------------------
tEplKernel EdrvSendTxMsg              (tEdrvTxBuffer * pBuffer_p)
{
tEplKernel Ret = kEplSuccessful;
unsigned int uiBufferNumber;
//DWORD       dwTemp;
ETHHUB_FIFO_ELEMENT_T tFifoPtr;

    uiBufferNumber = pBuffer_p->m_uiBufferNumber;

    if (uiBufferNumber == 0)
    {
        Ret = kEplEdrvBufNotExisting;
        goto Exit;
    }

    if (EdrvInstance_l.m_pLastTransmittedTxBuffer != NULL)
    {   // transmission is already active
        Ret = kEplInvalidOperation;
        goto Exit;
    }

    // save pointer to buffer structure for TxHandler
    EdrvInstance_l.m_pLastTransmittedTxBuffer = pBuffer_p;

    EDRV_COUNT_SEND;

    // pad with zeros if necessary, because controller does not do it
    if (pBuffer_p->m_uiTxMsgLen < MIN_ETH_SIZE)
    {
        EPL_MEMSET(pBuffer_p->m_pbBuffer + pBuffer_p->m_uiTxMsgLen, 0, MIN_ETH_SIZE - pBuffer_p->m_uiTxMsgLen);
        pBuffer_p->m_uiTxMsgLen = MIN_ETH_SIZE;
    }

    // create fifo element
    tFifoPtr.val = (uiBufferNumber) & ( MSK_ETHHUB_FIFO_ELEMENT_FRAME_BUF_NUM |
                    MSK_ETHHUB_FIFO_ELEMENT_INT_RAM_SEGMENT_NUM );
    tFifoPtr.bf.SUPPRESS_CON = 0;
    tFifoPtr.bf.FRAME_LEN = pBuffer_p->m_uiTxMsgLen;

    // request transmission by writing into according request fifo
    pfifo_push(ETHERNET_FIFO_REQ_LO, tFifoPtr.val);

Exit:
    return Ret;
}

#if 0
//---------------------------------------------------------------------------
//
// Function:    EdrvTxMsgReady
//
// Description: starts copying the buffer to the ethernet controller's FIFO
//
// Parameters:  pbBuffer_p - bufferdescriptor to transmit
//
// Returns:     Errorcode - kEplSuccessful
//
// State:
//
//---------------------------------------------------------------------------
tEplKernel EdrvTxMsgReady              (tEdrvTxBuffer * pBuffer_p)
{
tEplKernel Ret = kEplSuccessful;
unsigned int uiBufferNumber;


Exit:
    return Ret;
}


//---------------------------------------------------------------------------
//
// Function:    EdrvTxMsgStart
//
// Description: starts transmission of the ethernet controller's FIFO
//
// Parameters:  pbBuffer_p - bufferdescriptor to transmit
//
// Returns:     Errorcode - kEplSuccessful
//
// State:
//
//---------------------------------------------------------------------------
tEplKernel EdrvTxMsgStart              (tEdrvTxBuffer * pBuffer_p)
{
tEplKernel Ret = kEplSuccessful;



    return Ret;
}
#endif



//---------------------------------------------------------------------------
//
// Function:    EdrvReinitRx
//
// Description: reinitialize the Rx process, because of error
//
// Parameters:  void
//
// Returns:     void
//
// State:
//
//---------------------------------------------------------------------------
/*
static void EdrvReinitRx(void)
{
BYTE    bCmd;

    // simply switch off and on the receiver
    // this will reset the CAPR register
    bCmd = EDRV_REGB_READ(EDRV_REGB_COMMAND);
    EDRV_REGB_WRITE(EDRV_REGB_COMMAND, (bCmd & ~EDRV_REGB_COMMAND_RE));
    EDRV_REGB_WRITE(EDRV_REGB_COMMAND, bCmd);

    // set receive configuration register
    EDRV_REGDW_WRITE(EDRV_REGDW_RCR, EDRV_REGDW_RCR_DEF);
}
*/

//---------------------------------------------------------------------------
//
// Function:     EdrvInterruptHandler
//
// Description:  interrupt handler
//
// Parameters:   void
//
// Returns:      void
//
// State:
//
//---------------------------------------------------------------------------
#if 0
void EdrvInterruptHandler (void)
{
}
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,19)
static int TgtEthIsr (int nIrqNum_p, void* ppDevInstData_p)
#else
static int TgtEthIsr (int nIrqNum_p, void* ppDevInstData_p, struct pt_regs* ptRegs_p)
#endif
{
//    EdrvInterruptHandler();
tEdrvRxBuffer   RxBuffer;
tEdrvTxBuffer*  pTxBuffer;
DWORD           dwStatus;
int             iHandled = IRQ_HANDLED;
ETHHUB_FIFO_ELEMENT_T tFifoPtr;
int             iFillLevel;
ETHHUB_STATUS_AREA_BASE_T* ptCounters;
DWORD           dwFifoOverflow;
//DWORD           dwFifoReset;

//    printk("¤");

    // read the interrupt status
    dwStatus = readl(NETX_PFIFO_XPEC_ISR(0));

    // acknowledge the interrupts
    writel(dwStatus, NETX_PFIFO_XPEC_ISR(0));

//    printk("%s status = 0x%lX\n", __FUNCTION__, dwStatus);

    if (dwStatus == 0)
    {
        iHandled = IRQ_NONE;
        goto Exit;
    }

    // process tasks
    if ((dwStatus & (MSK_ETHHUB_INTERRUPTS_ENABLE_CON_LO_VAL)) != 0)
    {   // transmit interrupt (confirmation low FIFO)

        pTxBuffer = EdrvInstance_l.m_pLastTransmittedTxBuffer;
        EdrvInstance_l.m_pLastTransmittedTxBuffer = NULL;

        // retrieve the fifo element from the conf low FIFO
        tFifoPtr.val = pfifo_pop(ETHERNET_FIFO_CON_LO);

        // decode the error code
        switch(tFifoPtr.bf.ERROR_CODE)
        {
            case CONF_ERRCODE_TX_SUCCESSFUL_WITHOUT_RETRIES:
            case CONF_ERRCODE_TX_SUCCESSFUL_WITH_RETRIES:
            {
                EDRV_COUNT_TX;
                break;
            }

            case CONF_ERRCODE_TX_FAILED_LATE_COLLISION:
            case CONF_ERRCODE_TX_FAILED_EXCESSIVE_COLLISION:
            {
                EDRV_COUNT_TX_COL_RL;
                break;
            }

            case CONF_ERRCODE_TX_FAILED_UTX_UFL_DURING_TX:
            {
                EDRV_COUNT_TX_FUN;
                break;
            }

            case CONF_ERRCODE_TX_FAILED_LINK_DOWN_DURING_TX:
            case CONF_ERRCODE_TX_FAILED_FATAL_ERROR:
            default:
            {
                EDRV_COUNT_TX_ERR;
                break;
            }

        }

        if (pTxBuffer != NULL)
        {
            // call Tx handler of Data link layer
            EdrvInstance_l.m_InitParam.m_pfnTxHandler(pTxBuffer);
        }

    }

    if ((dwStatus & (MSK_ETHHUB_INTERRUPTS_ENABLE_RX_ERR_VAL)) != 0)
    {   // receive error interrupt

        printk("%s PFIFO underrun = 0x%X\n", __FUNCTION__, readl(NETX_PFIFO_UNDERRUN));

        dwFifoOverflow = readl(NETX_PFIFO_OVEFLOW);
        printk("%s PFIFO overflow = 0x%lX\n", __FUNCTION__, dwFifoOverflow);
/*
        if ((dwFifoOverflow & EDRV_FIFO_OVERFLOW_MASK) != 0)
        {
            dwFifoReset = readl(NETX_PFIFO_RESET);
            writel(dwFifoReset | EDRV_FIFO_OVERFLOW_MASK, NETX_PFIFO_RESET);
            writel(dwFifoReset, NETX_PFIFO_RESET);
        }
*/
        ptCounters = (ETHHUB_STATUS_AREA_BASE_T*) (EdrvInstance_l.m_pXpecBase + NETX_XPEC_RAM_START_OFS + REL_Adr_AREA_ETHHUB_STATUS_AREA_BASE);
        printk("%s frames received OK = %lu\n", __FUNCTION__, ptCounters->ulETHHUB_FRAMES_RECEIVED_OK);
        printk("%s CRC errors = %lu\n", __FUNCTION__, ptCounters->ulETHHUB_FRAME_CHECK_SEQUENCE_ERRORS);
        printk("%s align errors = %lu\n", __FUNCTION__, ptCounters->ulETHHUB_ALIGNMENT_ERRORS);
        printk("%s frame too long = %lu\n", __FUNCTION__, ptCounters->ulETHHUB_FRAME_TOO_LONG_ERRORS);
        printk("%s collision frags = %lu\n", __FUNCTION__, ptCounters->ulETHHUB_COLLISION_FRAGMENTS_RECEIVED);
        printk("%s frames dropped = %lu\n", __FUNCTION__, ptCounters->ulETHHUB_FRAMES_DROPPED_DUE_LOW_RESOURCE);
        printk("%s NIB len P0 = %lu\n", __FUNCTION__, ptCounters->ulETHHUB_FRAME_PREAMBLE_NIB_LEN_NOT_16_P0);
        printk("%s NIB len P1 = %lu\n", __FUNCTION__, ptCounters->ulETHHUB_FRAME_PREAMBLE_NIB_LEN_NOT_16_P1);
        printk("%s FIN set P0 = %lu\n", __FUNCTION__, ptCounters->ulETHHUB_RX_FRAME_FIN_SET_OUTSIDE_RX_FLOW_P0);
        printk("%s FIN set P1 = %lu\n", __FUNCTION__, ptCounters->ulETHHUB_RX_FRAME_FIN_SET_OUTSIDE_RX_FLOW_P1);
        printk("%s Rx fatal = %lu\n", __FUNCTION__, ptCounters->ulETHHUB_RX_FATAL_ERROR);

/*
        if ((wStatus & EDRV_REGW_INT_FOVW) != 0)
        {
            EDRV_COUNT_RX_FOVW;
        }
        else if ((wStatus & EDRV_REGW_INT_RXOVW) != 0)
        {
            EDRV_COUNT_RX_OVW;
        }
        else if ((wStatus & EDRV_REGW_INT_PUN) != 0)
        {   // Packet underrun
            EDRV_TRACE_RX_PUN(wStatus);
            EDRV_COUNT_RX_PUN;
        }
        else
        {
            EDRV_TRACE_RX_ERR(wStatus);
            EDRV_COUNT_RX_ERR;
        }

        // reinitialize Rx process
        EdrvReinitRx();
*/
    }

    if ((dwStatus & (MSK_ETHHUB_INTERRUPTS_ENABLE_IND_LO_VAL)) != 0)
    {   // receive interrupt

        iFillLevel = pfifo_fill_level(ETHERNET_FIFO_IND_LO);

        for ( ; iFillLevel > 0; iFillLevel--)
        {
            // retrieve the fifo element from the conf low FIFO
            tFifoPtr.val = pfifo_pop(ETHERNET_FIFO_IND_LO);

            RxBuffer.m_BufferInFrame = kEdrvBufferLastInFrame;
            RxBuffer.m_uiRxMsgLen = tFifoPtr.bf.FRAME_LEN;
            RxBuffer.m_pbBuffer = EdrvInstance_l.m_apSramBase[tFifoPtr.bf.INT_RAM_SEGMENT_NUM]
                            + (ETH_FRAME_BUF_SIZE * tFifoPtr.bf.FRAME_BUF_NUM);

//                printk("R");
            EDRV_COUNT_RX;

            // call Rx handler of Data link layer
            EdrvInstance_l.m_InitParam.m_pfnRxHandler(&RxBuffer);

            // release FIFO element to empty pointer FIFO
            pfifo_push(ETHERNET_FIFO_EMPTY, tFifoPtr.val);
        }

    }

Exit:
    return iHandled;
}


//---------------------------------------------------------------------------
//
// Function:    EdrvInitOne
//
// Description: initializes one platform device
//
// Parameters:  pPlatformDev_p      = pointer to corresponding platform device
//                                    structure
//
// Returns:     (int)               = error code
//
// State:
//
//---------------------------------------------------------------------------

static int EdrvInitOne(struct platform_device *pPlatformDev_p)
{
struct netxeth_platform_data* pPlatformData;
int                     iResult = 0;
int                     iXcNo;
ETHHUB_FIFO_ELEMENT_T   tFifoPtr;
unsigned int            uiEmptyPtrCnt;
unsigned int            uiFrame;

    pPlatformData =
        (struct netxeth_platform_data *)pPlatformDev_p->dev.platform_data;
    iXcNo = pPlatformData->xcno;

    if (iXcNo > 2)
    {   // the Ethernet hub uses only 2 xC units
        printk("%s device %s/%d discarded, because xc unit number is greater than 1\n",
                __FUNCTION__,
                pPlatformDev_p->name, pPlatformDev_p->id);
        iResult = -ENODEV;
        goto Exit;
    }

    if (EdrvInstance_l.m_apDev[iXcNo] != NULL)
    {   // Edrv is already connected to a platform device
        printk("%s device %s/%d discarded, because device with same xc unit number is already active\n",
                __FUNCTION__,
                pPlatformDev_p->name, pPlatformDev_p->id);
        iResult = -ENODEV;
        goto Exit;
    }

    // request the pointer FIFOs
    iResult = pfifo_request(PFIFO_MASK(iXcNo));
    if (iResult != 0) {
        printk("unable to request PFIFO\n");
        iResult = -ENODEV;
        goto Exit;
    }

    // $$$ d.k. configure pointer FIFO sizes (i.e. border_base)

    // fill the empty pointer FIFO
    tFifoPtr.val = 0;
    tFifoPtr.bf.INT_RAM_SEGMENT_NUM = iXcNo;

    // first DWORD in segment 0 is hardwired + IRQ vectors, so it cannot be used
    uiEmptyPtrCnt = (INTRAM_SEGMENT_SIZE / ETH_FRAME_BUF_SIZE) - 1;

    for (uiFrame = 1; uiFrame <= uiEmptyPtrCnt; uiFrame++)
    {
        tFifoPtr.bf.FRAME_BUF_NUM = uiFrame;
        pfifo_push(ETHERNET_FIFO_EMPTY, tFifoPtr.val);
    }

    // request the corresponding xC unit
    EdrvInstance_l.m_apXc[iXcNo] = request_xc(iXcNo, &pPlatformDev_p->dev);
    if (EdrvInstance_l.m_apXc[iXcNo] == NULL) {
        dev_err(&pPlatformDev_p->dev, "unable to request xc unit\n");
        iResult = -ENODEV;
        goto ExitFreePfifo;
    }

    dev_info(&pPlatformDev_p->dev, "loading firmware for xc unit\n");

    // stop xC unit before loading the new firmware
    xc_reset(EdrvInstance_l.m_apXc[iXcNo]);
    xc_stop(EdrvInstance_l.m_apXc[iXcNo]);

    // load the Ethernet Hub firmware to the xC unit
    if (xc_request_firmware(EdrvInstance_l.m_apXc[iXcNo])) {
        dev_err(&pPlatformDev_p->dev, "unable to load firmware for xc unit\n");
        iResult = -ENODEV;
        goto ExitFreeXc;
    }
    xc_reset(EdrvInstance_l.m_apXc[iXcNo]);
    xc_start(EdrvInstance_l.m_apXc[iXcNo]);

    // check if xC unit is running now
    iResult = xc_running(EdrvInstance_l.m_apXc[iXcNo]);
    if (iResult)
    {
        dev_info(&pPlatformDev_p->dev, "xc unit is running now\n");
    }
    else
    {
        dev_info(&pPlatformDev_p->dev, "xc unit is not running\n");
    }
    iResult = 0;

    // copy pointer to SRAM to instance structure
    EdrvInstance_l.m_apSramBase[iXcNo] = EdrvInstance_l.m_apXc[iXcNo]->sram_base;

/*
    // allocate buffers
    printk("%s allocate buffers\n", __FUNCTION__);
    EdrvInstance_l.m_pbTxBuf = pci_alloc_consistent(pPciDev, EDRV_TX_BUFFER_SIZE,
                     &EdrvInstance_l.m_pTxBufDma);
    if (EdrvInstance_l.m_pbTxBuf == NULL)
    {
        iResult = -ENOMEM;
        goto Exit;
    }

    EdrvInstance_l.m_pbRxBuf = pci_alloc_consistent(pPciDev, EDRV_RX_BUFFER_SIZE,
                     &EdrvInstance_l.m_pRxBufDma);
    if (EdrvInstance_l.m_pbRxBuf == NULL)
    {
        iResult = -ENOMEM;
        goto Exit;
    }
*/

    // initialization finished successfully

    // save pointer to device structure in corresponding element in instance structure
    EdrvInstance_l.m_apDev[iXcNo] = pPlatformDev_p;

    goto Exit;

ExitFreeXc:
    // release xC unit
    xc_stop(EdrvInstance_l.m_apXc[iXcNo]);
    free_xc(EdrvInstance_l.m_apXc[iXcNo]);

ExitFreePfifo:
    pfifo_free(PFIFO_MASK(iXcNo));

Exit:
    printk("%s(%d) finished with %d\n", __FUNCTION__, iXcNo, iResult);
    return iResult;
}


//---------------------------------------------------------------------------
//
// Function:    EdrvRemoveOne
//
// Description: shuts down one PCI device
//
// Parameters:  pPciDev             = pointer to corresponding PCI device structure
//
// Returns:     (void)
//
// State:
//
//---------------------------------------------------------------------------

static int EdrvRemoveOne(struct platform_device *pPlatformDev_p)
{
struct netxeth_platform_data* pPlatformData;
int     iXcNo;

    pPlatformData =
        (struct netxeth_platform_data *)pPlatformDev_p->dev.platform_data;
    iXcNo = pPlatformData->xcno;

    if (iXcNo > 2)
    {   // the Ethernet hub uses only 2 xC units
        printk("%s device %s/%d cannot be removed, because xc unit number is greater than 1\n",
                __FUNCTION__,
                pPlatformDev_p->name, pPlatformDev_p->id);
        goto Exit;
    }


    if (EdrvInstance_l.m_apDev[iXcNo] != pPlatformDev_p)
    {   // trying to remove unknown device
        goto Exit;
    }

    // release xC unit
    xc_stop(EdrvInstance_l.m_apXc[iXcNo]);
    free_xc(EdrvInstance_l.m_apXc[iXcNo]);

    pfifo_free(PFIFO_MASK(iXcNo));


/*
    // free buffers
    if (EdrvInstance_l.m_pbTxBuf != NULL)
    {
        pci_free_consistent(pPciDev, EDRV_TX_BUFFER_SIZE,
                     EdrvInstance_l.m_pbTxBuf, EdrvInstance_l.m_pTxBufDma);
        EdrvInstance_l.m_pbTxBuf = NULL;
    }

    if (EdrvInstance_l.m_pbRxBuf != NULL)
    {
        pci_free_consistent(pPciDev, EDRV_RX_BUFFER_SIZE,
                     EdrvInstance_l.m_pbRxBuf, EdrvInstance_l.m_pRxBufDma);
        EdrvInstance_l.m_pbRxBuf = NULL;
    }
*/

    EdrvInstance_l.m_apDev[iXcNo] = NULL;

Exit:
    return 0;
}


//---------------------------------------------------------------------------
//
// Function:    EdrvCalcHash
//
// Description: function calculates the entry for the hash-table from MAC
//              address
//
// Parameters:  pbMAC_p - pointer to MAC address
//
// Returns:     hash value
//
// State:
//
//---------------------------------------------------------------------------
#define HASH_BITS              6  // used bits in hash
#define CRC32_POLY    0x04C11DB6  //
//#define CRC32_POLY    0xEDB88320  //
// G(x) = x32 + x26 + x23 + x22 + x16 + x12 + x11 + x10 + x8 + x7 + x5 + x4 + x2 + x + 1
/*
static BYTE EdrvCalcHash (BYTE * pbMAC_p)
{
DWORD dwByteCounter;
DWORD dwBitCounter;
DWORD dwData;
DWORD dwCrc;
DWORD dwCarry;
BYTE * pbData;
BYTE bHash;

    pbData = pbMAC_p;

    // calculate crc32 value of mac address
    dwCrc = 0xFFFFFFFF;

    for (dwByteCounter = 0; dwByteCounter < 6; dwByteCounter++)
    {
        dwData = *pbData;
        pbData++;
        for (dwBitCounter = 0; dwBitCounter < 8; dwBitCounter++, dwData >>= 1)
        {
            dwCarry = (((dwCrc >> 31) ^ dwData) & 1);
            dwCrc = dwCrc << 1;
            if (dwCarry != 0)
            {
                dwCrc = (dwCrc ^ CRC32_POLY) | dwCarry;
            }
        }
    }

//    printk("MyCRC = 0x%08lX\n", dwCrc);
    // only upper 6 bits (HASH_BITS) are used
    // which point to specific bit in the hash registers
    bHash = (BYTE)((dwCrc >> (32 - HASH_BITS)) & 0x3f);

    return bHash;
}
*/
