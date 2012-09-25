/****************************************************************************

  (c) SYSTEC electronic GmbH, D-07973 Greiz, August-Bebel-Str. 29
      www.systec-electronic.com

  Project:      openPOWERLINK

  Description:  Ethernet driver for Davicom DM9003 Ethernet Switch Controller
                on Atmel AT91RM9200.

  License:

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    1. Redistributions of source code must retain the above copyright
       notice, this list of conditions and the following disclaimer.

    2. Redistributions in binary form must reproduce the above copyright
       notice, this list of conditions and the following disclaimer in the
       documentation and/or other materials provided with the distribution.

    3. Neither the name of SYSTEC electronic GmbH nor the names of its
       contributors may be used to endorse or promote products derived
       from this software without prior written permission. For written
       permission, please contact info@systec-electronic.com.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    POSSIBILITY OF SUCH DAMAGE.

    Severability Clause:

        If a provision of this License is or becomes illegal, invalid or
        unenforceable in any jurisdiction, that shall not affect:
        1. the validity or enforceability in that jurisdiction of any other
           provision of this License; or
        2. the validity or enforceability in other jurisdictions of that or
           any other provision of this License.

  -------------------------------------------------------------------------

                $RCSfile$

                $Author$

                $Revision$  $Date$

                $State$

                Build Environment:
                Dev C++ and GNU-Compiler for m68k

  -------------------------------------------------------------------------

  Revision History:

  2009/07/01 d.k.:   start of implementation

****************************************************************************/

#include "global.h"
#include "EplInc.h"
#include "edrv.h"
#include "Benchmark.h"

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/spinlock.h>
#include <linux/major.h>
#include <linux/version.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/atomic.h>
#include <asm/irq.h>
#include <linux/sched.h>
#include <linux/delay.h>
//#include <linux/crc32.h>


/***************************************************************************/
/*                                                                         */
/*                                                                         */
/*          G L O B A L   D E F I N I T I O N S                            */
/*                                                                         */
/*                                                                         */
/***************************************************************************/

// Chipselect CS2
// Address 0x30000000


//---------------------------------------------------------------------------
// const defines
//---------------------------------------------------------------------------

#define EDRV_MAX_FRAME_SIZE     0x600
#define EDRV_PHY_PORT_COUNT     2
#define DRV_NAME                "dm9sw_st"  // name of platform device


// register addresses
#define EDRV_REGB_NCR           0x00    // network control register
#define EDRV_REGB_NCR_RST           0x01    // software reset

#define EDRV_REGB_NSR           0x01    // network status register
#define EDRV_REGB_TCR           0x02    // TX control register
#define EDRV_REGB_TCR_TXREQ         0x01    // TX Request

#define EDRV_REGB_RCR           0x05    // RX control register
#define EDRV_REGB_RCR_RXEN          0x01    // RX enable
#define EDRV_REGB_RCR_PRMSC         0x02    // promiscuous mode
#define EDRV_REGB_RCR_ALL_MC        0x08    // pass all multicast packets
#define EDRV_REGB_RCR_HASHALL       0x80    // filter all address in hash table
#define EDRV_REGB_RCR_DEF           (EDRV_REGB_RCR_RXEN \
                                    | EDRV_REGB_RCR_ALL_MC)
//                                    | EDRV_REGB_RCR_PRMSC)

#define EDRV_REGB_RSR           0x06    // RX status register
#define EDRV_REGB_RSR_CE            0x02    // CRC error

#define EDRV_REGB_ROCR          0x07    // Receive overflow counter register
#define EDRV_REGB_FCR           0x0A    // Flow control register
#define EDRV_REGB_FCR_FLOW_EN       0x20    // Rx flow control enable

#define EDRV_REGB_EPCR          0x0B    // EEPROM & PHY control register
#define EDRV_REGB_EPAR          0x0C    // EEPROM & PHY address register
#define EDRV_REGB_EPDRL         0x0D    // EEPROM & PHY low byte data register
#define EDRV_REGB_EPDRH         0x0E    // EEPROM & PHY high byte data register
#define EDRV_REGB_WUCR          0x0F    // wake up control register
#define EDRV_REGB_PAR           0x10    // processor port physical address register
#define EDRV_REGB_MAR           0x16    // processor port multicast address register
#define EDRV_REGB_RXPLLR        0x20    // RX packet length low register
#define EDRV_REGB_RXPLHR        0x21    // RX packet length high register
#define EDRV_REGB_RASR          0x26    // RX additional status register
#define EDRV_REGB_RACR          0x27    // RX additional control register
#define EDRV_REGB_VIDL          0x28    // Vendor ID low register
#define EDRV_REGB_VIDH          0x29    // Vendor ID high register
#define EDRV_REGB_PIDL          0x2A    // Product ID low register
#define EDRV_REGB_PIDH          0x2B    // Product ID high register
#define EDRV_REGB_ID_DM9003         0x90030A46  // Vendor and Product ID of Davicom DM9003

#define EDRV_REGB_CHIPR         0x2C    // Chip revision register
#define EDRV_REGB_TCSCR         0x31    // transmit check sum control register
#define EDRV_REGB_RCSCSR        0x32    // receive check sum control status register
#define EDRV_REGB_DRIVER        0x38    // data bus driving capability register
#define EDRV_REGB_IRQCR         0x39    // IRQ pin control register
#define EDRV_REGB_SWITCHCR      0x52    // Switch control register
#define EDRV_REGB_SWITCHCR_RST_SW   0x40    // reset switch core
#define EDRV_REGB_SWITCHCR_SNF_PORT0        0x00    // sniffer port number 0
#define EDRV_REGB_SWITCHCR_SNF_PORT1        0x08    // sniffer port number 1
#define EDRV_REGB_SWITCHCR_SNF_PORT3        0x18    // sniffer port number 3 (processor port)
#define EDRV_REGB_SWITCHCR_SNF_PORT_MASK    0x18    // mask for sniffer port number
#define EDRV_REGB_SWITCHCR_AGE_NO           0x00    // no address aging
#define EDRV_REGB_SWITCHCR_AGE_64SEC        0x01    // 64 +-32 sec
#define EDRV_REGB_SWITCHCR_AGE_128SEC       0x02    // 128 +-64 sec
#define EDRV_REGB_SWITCHCR_AGE_256SEC       0x03    // 256 +-128 sec
#define EDRV_REGB_PINDEX        0x60    // per port control/status index register
#define EDRV_REGB_PPFCR         0x65    // per port forward control register
#define EDRV_REGB_PPFCR_MONI_TX     0x40    // Tx packet monitored
#define EDRV_REGB_PPFCR_MONI_RX     0x20    // Rx packet monitored
#define EDRV_REGB_PPFCR_DEF     (EDRV_REGB_PPFCR_MONI_TX \
                                 | EDRV_REGB_PPFCR_MONI_RX) // default
#define EDRV_REGB_MRCMDX        0xF0    // Memory data pre-fetch read command without address increment register
#define EDRV_REGB_MRCMD         0xF2    // Memory data read command with address increment register
#define EDRV_REGB_MWCMD         0xF8    // Memory data write command with address increment register
#define EDRV_REGB_TXPLLR        0xFC    // TX packet length low register
#define EDRV_REGB_TXPLHR        0xFD    // TX packet length high register
#define EDRV_REGB_ISR           0xFE    // Interrupt status register
#define EDRV_REGB_IMR           0xFF    // Interrupt mask register
#define EDRV_REGB_NCR           0x00    // network control register



// interrupt status and mask register values
#define EDRV_REGB_INT_PR        0x01    // Packet Received
#define EDRV_REGB_INT_PT        0x02    // Packet Transmitted
#define EDRV_REGB_INT_ROS       0x04    // Receive Overflow
#define EDRV_REGB_INT_ROO       0x08    // Receive Overflow Counter Overflow
#define EDRV_REGB_INT_CNT_ERR   0x10    // Memory Management error
#define EDRV_REGB_INT_LNKCHG    0x20    // Link Status Change of port 0 or 1
#define EDRV_REGB_INT_IO_8BIT   0x80    // 8 bit data bus width (only ISR)
#define EDRV_REGB_INT_TXRX_EN   0x80    // Enable the SRAM read/write pointer used as transmit/receive address (only IMR)
#define EDRV_REGB_INT_IOMODE    0xC0    // I/O mode (only ISR)
#define EDRV_REGB_INT_MASK_DEF  (EDRV_REGB_INT_PR \
                                 | EDRV_REGB_INT_PT \
                                 | EDRV_REGB_INT_ROS \
                                 | EDRV_REGB_INT_CNT_ERR \
                                 | EDRV_REGB_INT_TXRX_EN)   // default interrupt mask
#define EDRV_REGB_INT_MASK_DIS  (EDRV_REGB_INT_TXRX_EN)     // disabled interrupt mask


// defines for register access
#define EDRV_REGB_WRITE(bReg_p, bVal_p) do { \
                                            writeb(bReg_p, EdrvInstance_l.m_pIoAddr); \
                                            writeb(bVal_p, EdrvInstance_l.m_pIoAddr + 4); \
                                        } while (0)

#define EDRV_REGB_READ(bReg)            EdrvRegbRead(bReg)

#define EDRV_REG_GET_ADDRESS()          readb(EdrvInstance_l.m_pIoAddr)

#define EDRV_REG_SET_ADDRESS(bReg_p)    writeb(bReg_p, EdrvInstance_l.m_pIoAddr)



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


/* Port 0 can be enabled as sniffer port.
   This means if the multiple devices with this Ethernet controller are
   wired in line topology, you can watch all traffic on the first port.

   [PC]-----\      /----------\      /----------\      /------------
            |      |          |      |          |      |
         +------------+    +------------+    +------------+
         |Port0  Port1|    |Port0  Port1|    |Port0  Port1|
         |  Device A  |    |  Device B  |    |  Device C  |
         +------------+    +------------+    +------------+
*/
#ifndef EDRV_ENABLE_PORT0_SNIFFER
#define EDRV_ENABLE_PORT0_SNIFFER       FALSE
#endif


//---------------------------------------------------------------------------
// local types
//---------------------------------------------------------------------------

// Private structure
typedef struct
{
    struct platform_device* m_pDev;         // pointer to platform device structure
    void*                   m_pIoAddr;      // pointer to register space of Ethernet controller
    BYTE*                   m_pbRxBuf;      // pointer to Rx buffer

    tEdrvInitParam          m_InitParam;
    tEdrvTxBuffer*          m_pTransmittedTxBufferLastEntry;
    tEdrvTxBuffer*          m_pTransmittedTxBufferFirstEntry;
    spinlock_t              m_TxSpinlock;
    unsigned int            m_uiTxCount;

} tEdrvInstance;



//---------------------------------------------------------------------------
// local function prototypes
//---------------------------------------------------------------------------

static int EdrvInitOne(struct platform_device *pPlatformDev_p);

static int EdrvRemoveOne(struct platform_device *pPlatformDev_p);




//---------------------------------------------------------------------------
// module global vars
//---------------------------------------------------------------------------


static tEdrvInstance EdrvInstance_l;


static struct platform_driver EdrvDriver = {
	.probe		= EdrvInitOne,
	.remove		= EdrvRemoveOne,
	.driver		= {
		.name	= DRV_NAME,
		.owner	= THIS_MODULE,
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

static BYTE EdrvCalcHash (BYTE * pbMAC_p);


static inline BYTE EdrvRegbRead(BYTE bReg_p)
{
BYTE bVal;

    writeb(bReg_p, EdrvInstance_l.m_pIoAddr);
    bVal = readb(EdrvInstance_l.m_pIoAddr + 4);
    return bVal;
}


static inline void EdrvRegCopyTo(void* pDst_p, unsigned int uiCount_p)
{
unsigned int    uiIndex;
WORD*           pwDst;

    pwDst = (WORD*) pDst_p;

    for (uiIndex = (uiCount_p + 1) >> 1; uiIndex > 0; uiIndex--, pwDst++)
    {
        *pwDst = readw(EdrvInstance_l.m_pIoAddr + 4);
    }
}


static inline void EdrvRegCopyFrom(void* pSrc_p, unsigned int uiCount_p)
{
unsigned int    uiIndex;
WORD*           pwSrc;

    pwSrc = (WORD*) pSrc_p;

    for (uiIndex = (uiCount_p + 1) >> 1; uiIndex > 0; uiIndex--, pwSrc++)
    {
        writew(*pwSrc, EdrvInstance_l.m_pIoAddr + 4);
    }
}


static inline WORD EdrvPhyRead(BYTE bPhyPort_p, BYTE bPhyReg_p)
{
WORD wRegVal;

    EDRV_REGB_WRITE(EDRV_REGB_EPAR, ((bPhyPort_p << 6) | bPhyReg_p));
    EDRV_REGB_WRITE(EDRV_REGB_EPCR, 0x08); // clear command
    EDRV_REGB_WRITE(EDRV_REGB_EPCR, 0x0C); // issue read command
    EDRV_REGB_WRITE(EDRV_REGB_EPCR, 0x08); // clear command
    do
    {
        if ((EDRV_REGB_READ(EDRV_REGB_EPCR) & 0x01) == 0x00)
        {
            break;
        }
    }
    while (TRUE);

    wRegVal = (EDRV_REGB_READ(EDRV_REGB_EPDRH) << 8)
              | EDRV_REGB_READ(EDRV_REGB_EPDRL);
    return wRegVal;
}


static inline void EdrvPhyWrite(BYTE bPhyPort_p, BYTE bPhyReg_p, WORD wVal_p)
{

    EDRV_REGB_WRITE(EDRV_REGB_EPAR, ((bPhyPort_p << 6) | bPhyReg_p));

    EDRV_REGB_WRITE(EDRV_REGB_EPDRH, (wVal_p >> 8));
    EDRV_REGB_WRITE(EDRV_REGB_EPDRL, (wVal_p & 0xFF));

    EDRV_REGB_WRITE(EDRV_REGB_EPCR, 0x08); // clear command
    EDRV_REGB_WRITE(EDRV_REGB_EPCR, 0x0A); // issue write command
    EDRV_REGB_WRITE(EDRV_REGB_EPCR, 0x08); // clear command
    do
    {
        if ((EDRV_REGB_READ(EDRV_REGB_EPCR) & 0x01) == 0x00)
        {
            break;
        }
    }
    while (TRUE);

}


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

    Ret = kEplSuccessful;

    // clear instance structure
    EPL_MEMSET(&EdrvInstance_l, 0, sizeof (EdrvInstance_l));

    // save the init data
    EdrvInstance_l.m_InitParam = *pEdrvInitParam_p;

    // register platform driver
    iResult = platform_driver_register(&EdrvDriver);
    if (iResult != 0)
    {
        PRINTF("%s pci_register_driver failed with %d\n", __func__, iResult);
        Ret = kEplNoResource;
        goto Exit;
    }

    if (EdrvInstance_l.m_pDev == NULL)
    {
        PRINTF("%s m_pDev=NULL\n", __func__);
        Ret = kEplNoResource;
        goto Exit;
    }

/*
    // read MAC address from controller
    PRINTF("%s local MAC = ", __func__);
    for (iResult = 0; iResult < 6; iResult++)
    {
        pEdrvInitParam_p->m_abMyMacAddr[iResult] = EDRV_REGB_READ((EDRV_REGDW_IDR0 + iResult));
        PRINTF("%02X ", (unsigned int)pEdrvInitParam_p->m_abMyMacAddr[iResult]);
    }
    PRINTF("\n");
*/

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

    // unregister platform driver
    PRINTF("%s calling platform_driver_unregister()\n", __func__);
    platform_driver_unregister (&EdrvDriver);

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
BYTE        bHash;
BYTE        bData;
BYTE        bOffset;

    bHash = EdrvCalcHash (pbMacAddr_p);
/*
    {
        DWORD dwData;
    dwData = ether_crc(6, pbMacAddr_p);

    PRINTF("EdrvDefineRxMacAddrEntry('%02X:%02X:%02X:%02X:%02X:%02X') hash = %u / %u  ether_crc = 0x%08lX\n",
        (WORD) pbMacAddr_p[0], (WORD) pbMacAddr_p[1], (WORD) pbMacAddr_p[2],
        (WORD) pbMacAddr_p[3], (WORD) pbMacAddr_p[4], (WORD) pbMacAddr_p[5],
        (WORD) bHash, (WORD) (dwData & 0x3F), dwData);
    }
*/

    bOffset = (bHash >> 3);
    bData = EDRV_REGB_READ(EDRV_REGB_MAR + bOffset);

    PRINTF("%s MAR[%u] %02X | %02X\n", __func__, (WORD) bOffset, (WORD) bData,1 << (bHash & 0x07));

    bData |= 1 << (bHash & 0x07);
    EDRV_REGB_WRITE(EDRV_REGB_MAR + bOffset, bData);

/*
    PRINTF("%s", __func__);
    for (bOffset = 0; bOffset < 8; bOffset++)
    {
        PRINTF(" %02X", EDRV_REGB_READ(EDRV_REGB_MAR + bOffset));
    }
    PRINTF("\n");
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
BYTE        bHash;
BYTE        bData;
BYTE        bOffset;

    bHash = EdrvCalcHash (pbMacAddr_p);

    bOffset = (bHash >> 3);
    bData = EDRV_REGB_READ(EDRV_REGB_MAR + bOffset);
    bData &= ~(1 << (bHash & 0x07));
    EDRV_REGB_WRITE(EDRV_REGB_MAR + bOffset, bData);

    return Ret;
}


tEplKernel EdrvChangeFilter(tEdrvFilter*    pFilter_p,
                            unsigned int    uiCount_p,
                            unsigned int    uiEntryChanged_p,
                            unsigned int    uiChangeFlags_p)
{
tEplKernel      Ret = kEplSuccessful;

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

    if (pBuffer_p->m_uiMaxBufferLen > EDRV_MAX_FRAME_SIZE)
    {
        Ret = kEplEdrvNoFreeBufEntry;
        goto Exit;
    }

    // allocate buffer with malloc (in this case it is kmalloc)
    pBuffer_p->m_pbBuffer = EPL_MALLOC(pBuffer_p->m_uiMaxBufferLen);
    if (pBuffer_p->m_pbBuffer == NULL)
    {
        Ret = kEplEdrvNoFreeBufEntry;
        goto Exit;
    }

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
BYTE*   pbBuffer = pBuffer_p->m_pbBuffer;

    // mark buffer as free, before actually freeing it
    pBuffer_p->m_pbBuffer = NULL;

    EPL_FREE(pbBuffer);

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
tEplKernel      Ret = kEplSuccessful;
unsigned long   ulFlags;
unsigned int    uiCurTxCount;

    if (pBuffer_p->m_BufferNumber.m_pVal != NULL)
    {
        Ret = kEplInvalidOperation;
        goto Exit;
    }

    spin_lock_irqsave(&EdrvInstance_l.m_TxSpinlock, ulFlags);
    if (EdrvInstance_l.m_pTransmittedTxBufferLastEntry == NULL)
    {
        EdrvInstance_l.m_pTransmittedTxBufferLastEntry =
            EdrvInstance_l.m_pTransmittedTxBufferFirstEntry = pBuffer_p;
    }
    else
    {
        EdrvInstance_l.m_pTransmittedTxBufferLastEntry->m_BufferNumber.m_pVal = pBuffer_p;
        EdrvInstance_l.m_pTransmittedTxBufferLastEntry = pBuffer_p;
    }
    uiCurTxCount = EdrvInstance_l.m_uiTxCount;
    spin_unlock_irqrestore(&EdrvInstance_l.m_TxSpinlock, ulFlags);

    EDRV_COUNT_SEND;

    if (uiCurTxCount < 2)
    {
        // start transmit buffer write with automatic pointer incrementation
        EDRV_REG_SET_ADDRESS(EDRV_REGB_MWCMD);

        // copy frame to Ethernet controller
        EdrvRegCopyFrom(pBuffer_p->m_pbBuffer, pBuffer_p->m_uiTxMsgLen);

        spin_lock_irqsave(&EdrvInstance_l.m_TxSpinlock, ulFlags);
        uiCurTxCount = ++EdrvInstance_l.m_uiTxCount;
        if (uiCurTxCount == 1)
        {
            // write length
            EDRV_REGB_WRITE(EDRV_REGB_TXPLLR, (pBuffer_p->m_uiTxMsgLen & 0xFF));
            EDRV_REGB_WRITE(EDRV_REGB_TXPLHR, ((pBuffer_p->m_uiTxMsgLen >> 8) & 0xFF));

            // start transmission
            EDRV_REGB_WRITE(EDRV_REGB_TCR, EDRV_REGB_TCR_TXREQ);
        }
        spin_unlock_irqrestore(&EdrvInstance_l.m_TxSpinlock, ulFlags);
    }

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
static void EdrvReinitRx(void)
{
/*
BYTE    bCmd;

    // simply switch off and on the receiver
    // this will reset the CAPR register
    bCmd = EDRV_REGB_READ(EDRV_REGB_COMMAND);
    EDRV_REGB_WRITE(EDRV_REGB_COMMAND, (bCmd & ~EDRV_REGB_COMMAND_RE));
    EDRV_REGB_WRITE(EDRV_REGB_COMMAND, bCmd);

    // set receive configuration register
    EDRV_REGDW_WRITE(EDRV_REGDW_RCR, EDRV_REGDW_RCR_DEF);
*/
}


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
static irqreturn_t TgtEthIsr (int nIrqNum_p, void* ppDevInstData_p)
#else
static irqreturn_t TgtEthIsr (int nIrqNum_p, void* ppDevInstData_p, struct pt_regs* ptRegs_p)
#endif
{
//    EdrvInterruptHandler();
tEdrvRxBuffer   RxBuffer;
tEdrvTxBuffer*  pTxBuffer;
BYTE            bSavedRegAddress;
BYTE            bStatus;
BYTE            bRxStatus;
BYTE*           pbRxBuf;
unsigned int    uiLength;
int             iHandled = IRQ_HANDLED;

    // save current register address
    bSavedRegAddress = EDRV_REG_GET_ADDRESS();

    // disable the interrupts
    EDRV_REGB_WRITE(EDRV_REGB_IMR, EDRV_REGB_INT_MASK_DIS);

    // read the interrupt status
    bStatus = EDRV_REGB_READ(EDRV_REGB_ISR) & ~EDRV_REGB_INT_IOMODE;

    // acknowledge the interrupts
    EDRV_REGB_WRITE(EDRV_REGB_ISR, bStatus);

    if (bStatus == 0)
    {
        iHandled = IRQ_NONE;
        goto Exit;
    }

    // process tasks
    if ((bStatus & EDRV_REGB_INT_PT) != 0)
    {   // transmit interrupt
    unsigned long   ulFlags;
    unsigned int    uiCurTxCount;
    tEdrvTxBuffer*  pNextTxBuffer = NULL;

        EDRV_COUNT_TX;

        spin_lock_irqsave(&EdrvInstance_l.m_TxSpinlock, ulFlags);
        pTxBuffer = EdrvInstance_l.m_pTransmittedTxBufferFirstEntry;
        EdrvInstance_l.m_pTransmittedTxBufferFirstEntry = EdrvInstance_l.m_pTransmittedTxBufferFirstEntry->m_BufferNumber.m_pVal;
        if (EdrvInstance_l.m_pTransmittedTxBufferFirstEntry == NULL)
        {
            EdrvInstance_l.m_pTransmittedTxBufferLastEntry = NULL;
        }
        uiCurTxCount = --EdrvInstance_l.m_uiTxCount;
        if (uiCurTxCount >= 1)
        {
            pNextTxBuffer = pTxBuffer->m_BufferNumber.m_pVal;

            if (pNextTxBuffer != NULL)
            {
                // write length
                EDRV_REGB_WRITE(EDRV_REGB_TXPLLR, (pNextTxBuffer->m_uiTxMsgLen & 0xFF));
                EDRV_REGB_WRITE(EDRV_REGB_TXPLHR, ((pNextTxBuffer->m_uiTxMsgLen >> 8) & 0xFF));

                // start transmission
                EDRV_REGB_WRITE(EDRV_REGB_TCR, EDRV_REGB_TCR_TXREQ);
            }
        }
        spin_unlock_irqrestore(&EdrvInstance_l.m_TxSpinlock, ulFlags);

        pTxBuffer->m_BufferNumber.m_pVal = NULL;

        if (pTxBuffer->m_pfnTxHandler != NULL)
        {
            BENCHMARK_MOD_01_SET(1);
            pTxBuffer->m_pfnTxHandler(pTxBuffer);
            BENCHMARK_MOD_01_RESET(1);
        }

        if ((uiCurTxCount >= 2) && (pNextTxBuffer != NULL))
        {
            pNextTxBuffer = pNextTxBuffer->m_BufferNumber.m_pVal;

            if (pNextTxBuffer != NULL)
            {
                // start transmit buffer write with automatic pointer incrementation
                EDRV_REG_SET_ADDRESS(EDRV_REGB_MWCMD);

                // copy frame to Ethernet controller
                EdrvRegCopyFrom(pNextTxBuffer->m_pbBuffer, pNextTxBuffer->m_uiTxMsgLen);
            }
        }

    }

    if ((bStatus & EDRV_REGB_INT_ROS) != 0)
    {   // receive error interrupt

        EDRV_COUNT_RX_OVW;

        // reinitialize Rx process
        EdrvReinitRx();
    }

    if ((bStatus & EDRV_REGB_INT_PR) != 0)
    {   // receive interrupt

        if (EdrvInstance_l.m_pbRxBuf == NULL)
        {
            PRINTF("%s Rx buffers currently not allocated\n", __func__);
            goto Exit;
        }

        BENCHMARK_MOD_01_SET(6);
        for (;;)
        {
            pbRxBuf = EdrvInstance_l.m_pbRxBuf;

            // dummy read
            bRxStatus = EDRV_REGB_READ(EDRV_REGB_MRCMDX);

            // read first byte
            EdrvRegCopyTo(pbRxBuf, 1);

            if (*pbRxBuf == 0)
            {   // no frame available
                break;
            }
            else if (*pbRxBuf != 1)
            {   // receive buffer error
                EDRV_TRACE_RX_ERR(*pbRxBuf);
                EDRV_COUNT_RX_ERR;

                // reinitialize Rx process
                EdrvReinitRx();
                break;
            }

            // restart receive buffer read with automatic pointer incrementation
            EDRV_REG_SET_ADDRESS(EDRV_REGB_MRCMD);

            // get status DWORD from receive buffer
            EdrvRegCopyTo(pbRxBuf, 4);

            // fetch length from status DWORD
            uiLength = AmiGetWordFromLe(pbRxBuf + 2);

            // fetch Rx status from status DWORD
            bRxStatus = *(pbRxBuf + 1);

            // forward buffer pointer
            pbRxBuf += 4;

            // copy frame to buffer
            EdrvRegCopyTo(pbRxBuf, uiLength);

            if ((bRxStatus & EDRV_REGB_RSR_CE) != 0)
            {   // CRC error (ignore frame)
                EDRV_TRACE_RX_CRC(bRxStatus);
                EDRV_COUNT_RX_CRC;
            }
            else
            {   // frame is OK
                RxBuffer.m_BufferInFrame = kEdrvBufferLastInFrame;
                RxBuffer.m_uiRxMsgLen = uiLength - ETH_CRC_SIZE;
                RxBuffer.m_pbBuffer = pbRxBuf;

//                PRINTF("R");
                EDRV_COUNT_RX;

                // call Rx handler of Data link layer
                EdrvInstance_l.m_InitParam.m_pfnRxHandler(&RxBuffer);
            }

        }
        BENCHMARK_MOD_01_RESET(6);
        
    }

    if ((bStatus & EDRV_REGB_INT_CNT_ERR) != 0)
    {   // Memory management error
        EDRV_COUNT_PCI_ERR;
    }

Exit:
    // enable the interrupts
    EDRV_REGB_WRITE(EDRV_REGB_IMR, EDRV_REGB_INT_MASK_DEF);

    EDRV_REG_SET_ADDRESS(bSavedRegAddress);

    return iHandled;
}


//---------------------------------------------------------------------------
//
// Function:    EdrvInitOne
//
// Description: initializes one PCI device
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
int     iResult = 0;
int     iIndex = 0;
DWORD   dwTemp;
struct resource* pResource;

    if (EdrvInstance_l.m_pDev != NULL)
    {   // Edrv is already connected to a PCI device
        PRINTF("%s device %s discarded\n", __func__, pPlatformDev_p->name);
        iResult = -ENODEV;
        goto Exit;
    }

    EdrvInstance_l.m_pDev = pPlatformDev_p;

    PRINTF("%s get resource IOMEM... ", __func__);
    pResource = platform_get_resource(pPlatformDev_p, IORESOURCE_MEM, 0);
    if (pResource == NULL)
    {
        PRINTF("FAILED\n");
        iResult = -ENODEV;
        goto Exit;
    }
    else
    {
        PRINTF("OK\n");
    }

    PRINTF("%s ioremap... ", __func__);
    EdrvInstance_l.m_pIoAddr = ioremap (pResource->start, 0x08);
    if (EdrvInstance_l.m_pIoAddr == NULL)
    {   // remap of controller's register space failed
        PRINTF("FAILED\n");
        iResult = -EIO;
        goto Exit;
    }
    else
    {
        PRINTF("OK\n");
    }

    // check Vendor and Product ID
    dwTemp = EDRV_REGB_READ(EDRV_REGB_VIDL);
    dwTemp |= EDRV_REGB_READ(EDRV_REGB_VIDH) << 8;
    dwTemp |= EDRV_REGB_READ(EDRV_REGB_PIDL) << 16;
    dwTemp |= EDRV_REGB_READ(EDRV_REGB_PIDH) << 24;

    PRINTF("%s check device ID (%X)... ", __func__, EDRV_REGB_ID_DM9003);
    if (dwTemp != EDRV_REGB_ID_DM9003)
    {   // device is not supported by this driver
        PRINTF("FAILED\n");
        PRINTF("%s device ID %lX not supported\n", __func__, dwTemp);
        iResult = -ENODEV;
        goto Exit;
    }
    else
    {
        PRINTF("OK\n");
    }

    // reset switch
    PRINTF("%s reset switch... ", __func__);
    EDRV_REGB_WRITE(EDRV_REGB_SWITCHCR, EDRV_REGB_SWITCHCR_RST_SW);

    // wait until reset has finished
    for (iResult = 50; iResult > 0; iResult--)
    {
        if ((EDRV_REGB_READ(EDRV_REGB_SWITCHCR) & EDRV_REGB_SWITCHCR_RST_SW) == 0)
        {
            break;
        }

        schedule_timeout(10);
    }
    PRINTF("Done\n");


    // reset controller
    PRINTF("%s reset controller... ", __func__);
    EDRV_REGB_WRITE(EDRV_REGB_NCR, EDRV_REGB_NCR_RST);

    // wait until reset has finished
    for (iResult = 50; iResult > 0; iResult--)
    {
        if ((EDRV_REGB_READ(EDRV_REGB_NCR) & EDRV_REGB_NCR_RST) == 0)
        {
            break;
        }

        schedule_timeout(10);
    }
    PRINTF("Done\n");

    // disable interrupts
    PRINTF("%s disable interrupts\n", __func__);
    EDRV_REGB_WRITE(EDRV_REGB_IMR, 0);
    // acknowledge all pending interrupts
    EDRV_REGB_WRITE(EDRV_REGB_ISR, (EDRV_REGB_READ(EDRV_REGB_ISR) & ~EDRV_REGB_INT_IO_8BIT));

    // install interrupt handler
    PRINTF("%s install interrupt handler... ", __func__);
    iResult = request_irq(platform_get_irq(pPlatformDev_p, 0),
                          TgtEthIsr,
                          IRQF_SHARED, //IRQF_NODELAY,
                          DRV_NAME,
                          pPlatformDev_p);
    if (iResult != 0)
    {
        PRINTF("FAILED\n");
        goto Exit;
    }
    else
    {
        PRINTF("OK\n");
    }

    // allocate Rx buffer
    PRINTF("%s allocate Rx buffer... ", __func__);
    EdrvInstance_l.m_pbRxBuf = EPL_MALLOC(EDRV_MAX_FRAME_SIZE);
    if (EdrvInstance_l.m_pbRxBuf == NULL)
    {
        PRINTF("FAILED\n");
        iResult = -ENOMEM;
        goto Exit;
    }
    else
    {
        PRINTF("OK\n");
    }

    // reset PHYs
    PRINTF("%s reset PHYs... ", __func__);
    for (iIndex = 0; iIndex < EDRV_PHY_PORT_COUNT; iIndex++)
    {
        EdrvPhyWrite(iIndex, 0x00, 0x8000);
    }

    for (iIndex = 0; iIndex < EDRV_PHY_PORT_COUNT; iIndex++)
    {
        for (iResult = 50; iResult > 0; iResult--)
        {
            if ((EdrvPhyRead(iIndex, 0x00) & 0x8000) == 0)
            {
                break;
            }

            schedule_timeout(10);
        }
    }
    PRINTF("Done\n");

    // enable Rx flow control
    PRINTF("%s enable Rx flow control\n", __func__);
    EDRV_REGB_WRITE(EDRV_REGB_FCR, EDRV_REGB_FCR_FLOW_EN);

    // set MAC address
    for (iResult = 0; iResult < 6; iResult++)
    {
        EDRV_REGB_WRITE((EDRV_REGB_PAR + iResult), EdrvInstance_l.m_InitParam.m_abMyMacAddr[iResult]);
    }

    // set multicast MAC filter (enable reception of broadcast frames)
    for (iResult = 0; iResult < 7; iResult++)
    {
        EDRV_REGB_WRITE((EDRV_REGB_MAR + iResult), 0x00);
    }
    EDRV_REGB_WRITE((EDRV_REGB_MAR + iResult), 0x80);
    iResult = 0;

/*
    {
        BYTE abBroadcast[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
        PRINTF("%s broadcast hash: %02X\n", __func__, (WORD) EdrvCalcHash(abBroadcast));
    }
*/

    #if EDRV_ENABLE_PORT0_SNIFFER != FALSE
    PRINTF("%s set port 0 as sniffer port... ", __func__);
    EDRV_REGB_WRITE(EDRV_REGB_SWITCHCR,
        ((EDRV_REGB_READ(EDRV_REGB_SWITCHCR) & ~EDRV_REGB_SWITCHCR_SNF_PORT_MASK) | EDRV_REGB_SWITCHCR_AGE_64SEC));
    /* forward RX_MONI and TX_MONI for port 0 to sniffer port */
    EDRV_REGB_WRITE(EDRV_REGB_PINDEX, (EDRV_REGB_READ(EDRV_REGB_PINDEX) & 0xfc));
    EDRV_REGB_WRITE(EDRV_REGB_PPFCR, EDRV_REGB_PPFCR_DEF);
    /* forward RX_MONI and TX_MONI for port 3 (processor port) to sniffer port */
    EDRV_REGB_WRITE(EDRV_REGB_PINDEX, (EDRV_REGB_READ(EDRV_REGB_PINDEX) | 0x03));
    EDRV_REGB_WRITE(EDRV_REGB_PPFCR, EDRV_REGB_PPFCR_DEF);
    #endif

    // enable receiver
    PRINTF("%s enable Rx\n", __func__);
    EDRV_REGB_WRITE(EDRV_REGB_RCR, EDRV_REGB_RCR_DEF);

    // enable interrupts
    PRINTF("%s enable interrupts\n", __func__);
    EDRV_REGB_WRITE(EDRV_REGB_IMR, EDRV_REGB_INT_MASK_DEF);

    spin_lock_init(&EdrvInstance_l.m_TxSpinlock);

Exit:
    PRINTF("%s finished with %d\n", __func__, iResult);
    return iResult;
}


//---------------------------------------------------------------------------
//
// Function:    EdrvRemoveOne
//
// Description: shuts down one PCI device
//
// Parameters:  pPlatformDev_p      = pointer to corresponding platform device
//                                    structure
//
// Returns:     (void)
//
// State:
//
//---------------------------------------------------------------------------

static int EdrvRemoveOne(struct platform_device *pPlatformDev_p)
{

    if (EdrvInstance_l.m_pDev != pPlatformDev_p)
    {   // trying to remove unknown device
        BUG_ON(EdrvInstance_l.m_pDev != pPlatformDev_p);
        goto Exit;
    }

    // disable receiver
    EDRV_REGB_WRITE(EDRV_REGB_RCR, 0);

    // disable interrupts
    EDRV_REGB_WRITE(EDRV_REGB_IMR, 0);

    // remove interrupt handler
    free_irq(platform_get_irq(pPlatformDev_p, 0), pPlatformDev_p);


    // free buffers
    if (EdrvInstance_l.m_pbRxBuf != NULL)
    {
        EPL_FREE(EdrvInstance_l.m_pbRxBuf);
        EdrvInstance_l.m_pbRxBuf = NULL;
    }

    // unmap controller's register space
    if (EdrvInstance_l.m_pIoAddr != NULL)
    {
        iounmap(EdrvInstance_l.m_pIoAddr);
    }

    EdrvInstance_l.m_pDev = NULL;

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
#define CRC32_POLY    0x04C11DB7  // 6
//#define CRC32_POLY    0xEDB88320  //
// G(x) = x32 + x26 + x23 + x22 + x16 + x12 + x11 + x10 + x8 + x7 + x5 + x4 + x2 + x + 1

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

//    PRINTF("MyCRC = 0x%08lX\n", dwCrc);
    bHash = (BYTE)(dwCrc & 0x3f);

    return bHash;
}

