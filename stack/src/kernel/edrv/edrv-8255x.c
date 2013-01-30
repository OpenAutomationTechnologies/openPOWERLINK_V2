/****************************************************************************

Copyright (c) 2011 Kalycito Infotech Private Limited

Project: openPOWERLINK

Description: Ethernet driver for Intel 8255x 10/100 Fast Ethernet Controller.

License:

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions
are met:

1. Redistributions of source code must retain the above copyright
notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright
notice, this list of conditions and the following disclaimer in the
documentation and/or other materials provided with the distribution.

3. Neither the name of Kalycito Infotech nor the names of its
contributors may be used to endorse or promote products derived
from this software without prior written permission. For written
permission, please contact info@kalycito.com.

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

Based on
1. ife_gem.c: Intel 8255x 10/100 Ethernet Controller driver for Solaris
by Masayuki Murayama.
2. Intel 8255x 10/100 Mbps Ethernet Controller Family Open Source Software
Developer Manual.

****************************************************************************/

/*
* ife_gem.c: Intel 8255x 10/100 ethernet controler driver for Solaris
*
* Copyright (c) 2003-2008 Masayuki Murayama. All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice,
* this list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following disclaimer in the documentation
* and/or other materials provided with the distribution.
*
* 3. Neither the name of the author nor the names of its contributors may be
* used to endorse or promote products derived from this software without
* specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
* OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
* AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
* OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
* DAMAGE.
*/

#include "global.h"
#include "EplInc.h"
#include "edrv.h"

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
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 26)
#include <linux/semaphore.h>
#endif

/***************************************************************************/
/* */
/* */
/* G L O B A L D E F I N I T I O N S */
/* */
/* */
/***************************************************************************/
//---------------------------------------------------------------------------
// const defines
//---------------------------------------------------------------------------

#ifndef EDRV_MAX_TX_BUFFERS
#define EDRV_MAX_TX_BUFFERS 42
#endif

#ifndef EDRV_MAX_TX_DESCS
#define EDRV_MAX_TX_DESCS 16
#define EDRV_TX_DESC_MASK (EDRV_MAX_TX_DESCS - 1)
#endif

#ifndef EDRV_MAX_RX_BUFFERS
#define EDRV_MAX_RX_BUFFERS 256
#endif

#ifndef EDRV_MAX_RX_DESCS
#define EDRV_MAX_RX_DESCS 16
#define EDRV_RX_DESC_MASK (EDRV_MAX_RX_DESCS - 1)
#endif

#define EDRV_MAX_FRAME_SIZE 0x600

#define EDRV_TX_BUFFER_SIZE (EDRV_MAX_TX_BUFFERS * EDRV_MAX_FRAME_SIZE) // n * (MTU + 14 + 4)

#define EDRV_RX_BUFFER_SIZE_SHIFT 11 // 2048 Byte
#define EDRV_RX_BUFFER_SIZE (1 << EDRV_RX_BUFFER_SIZE_SHIFT)

#define DRV_NAME "epl"

// 8255x hash defines - start
/*
* Register offsets
*/
#define SCBSTAT 0x00 /* W SCB status word */
#define SCBCMD 0x02 /* W SCB command word */
#define GENPTR 0x04 /* D SCB general pointer */
#define PORT 0x08 /* D Port interface */
#define EECTRL 0x0e /* B EEPROM control register */
#define EARLYRXINT 0x18 /* B earlt rx interrupt register */
#define FCTRL 0x1a /* B Flow control on/off register */

/* SCB Status Word */
#define SS_CX 0x8000 /* cmd done w/ intr */
#define SS_FR 0x4000 /* Rx done */
#define SS_CNA 0x2000 /* CU not active / */
#define SS_RNR 0x1000 /* RU not ready */

/* SCB Command Word */
#define SC_M 0x0100 /* interrupt mask 1:mask 0: nomask */
#define SC_CUC 0x00f0 /* CU command */
#define SC_CUC_SHIFT 4
#define SC_CUC_START (0x1 << SC_CUC_SHIFT)
#define SC_CUC_LOADSDMP (0x4 << SC_CUC_SHIFT)
#define SC_CUC_LOADBASE (0x6 << SC_CUC_SHIFT)
#define SC_RUC 0x0007 /* RU command */
#define SC_RUC_SHIFT 0
#define SC_RUC_START (1 << SC_RUC_SHIFT)
#define SC_RUC_ABORT (4 << SC_RUC_SHIFT)
#define SC_RUC_LOADHDS (5 << SC_RUC_SHIFT)
#define SC_RUC_LOADBASE (6 << SC_RUC_SHIFT)

/* Port register */
#define PORT_SOFTRESET 0x0 /* entire reset */

/* EEPROM control register */
#define EC_EEDO_SHIFT 3
#define EC_EEDO (1 << EC_EEDO_SHIFT) /* serial data-out from eeprom */
#define EC_EEDI_SHIFT 2
#define EC_EECS 0x02 /* chip select 0:disable, 1:enable */
#define EC_EESK 0x01 /* serial clock */

/* Shared memory operations */

/* OPCODES */
#define OP_NOP 0
#define OP_ADDRSETUP 1 /* individual address setup */
#define OP_CONFIGURE 2 /* load the device with operating parameters */
#define OP_MULTICAST 3 /* setup one or more multicast address */
#define OP_TX 4 /* Transmit a single frame */

#define CS_EL 0x80000000
#define CS_S 0x40000000 /* suspend */
#define CS_I 0x20000000 /* generate interrupt */
#define CS_SF_ 0x00080000 /* 0:simplify mode, 1:flexible mode */
#define CS_OP_SHIFT 16
#define CS_C 0x00008000 /* completed */
#define CS_OK 0x00002000 /* no error */

#define TCB_TBDNUM_SHIFT 24
#define TCB_TXTHR_SHIFT 16
#define TCB_EOF 0x00008000 /* for backword compatibility */

/* Transmit buffer descriptor */

#define RFD_SIZE 0x3fff0000 /* prepared buffer size */
#define RFD_SIZE_SHIFT 16

#define RFD_EOF 0x00008000 /* end of frame */
#define RFD_F 0x00004000
#define RFD_COUNT 0x00003fff /* actual received byte count */

// 8255x hash defines - end

// custom hash defines - start
/*
* EEPROM I/O routines
*/
#define EEPROM_READ_CMD 6

// number of multicast addresses
#define MULTICAST_ADDR_NUM 4

// the number of bytes in a MAC address
#define MAC_ADDRESS_LEN 6

// number of bytes required for one Command Block
// to be incremented by 6 for every additional
// address that needs to be added to the
// multicast table of the controller
// to be kept 32 bit aligned
#define CB_REQUIRED_SIZE 56

// maximum number of Command Blocks
#define MAX_CBS 16

// maximum size of array for storing TX buffer pointers
// for use during callbacks in ISR
#define MAX_HANDLER_ADDR 4

// number of bytes required for one RX descriptor block + RX buffer section for that descriptor block
#define RFD_REQUIRED_SIZE 1534
// number of bytes required for one RX buffer
#define RX_BUFFER_REQUIRED_SIZE 1518
// maximum number of RX descriptors
#define MAX_RFDS 16
// command mode to EdrvMulticastCmd()
#define MULTICAST_ADDR_ADD 0 /* to add multicast address */
#define MULTICAST_ADDR_REM 1 /* to remove multicast address */
#define DELAY_CLEAR_INT 5
#define DELAY_SYS_TX_CLK 10
// custom hash defines - end

//Global Structure Definition
struct cb
{
    volatile UINT m_uiCmdStat;
    volatile UINT m_uiLink;
};

/* Transmit command block */
struct tcb
{
    volatile UINT m_uiTcbTbdPtr; /* tbd array address */
    volatile UINT m_uiTcbCtrl;
};

struct tbd
{
    volatile UINT m_uiTbdAddr;
#define TBD_ADDR_NULL 0xffffffff
    volatile UINT m_uiTbdSize;
#define TBD_EL 0x00010000
};

struct rfd
{
    volatile UINT m_uiCmdStat;
    volatile UINT m_uiLink;
    volatile UINT :32; //Reserved
    volatile UINT m_uiSize;
};

/* command block */
typedef struct
{
    volatile UINT m_uiStatCommand;
    volatile UINT m_uiLinkAddr;
    volatile UINT m_uiValue1;
    volatile UINT m_uiValue2;
    volatile UINT m_uiValue3;
    volatile UINT m_uiValue4;
    volatile UINT m_uiValue5;
    volatile UINT m_uiValue6;
}cbstruct;

// custom vars - end

// TracePoint support for realtime-debugging
#ifdef _DBG_TRACE_POINTS_
    void PUBLIC TgtDbgSignalTracePoint (BYTE bTracePointNumber_p);
    void PUBLIC TgtDbgPostTraceValue (DWORD dwTraceValue_p);
#define TGT_DBG_SIGNAL_TRACE_POINT(p) TgtDbgSignalTracePoint(p)
#define TGT_DBG_POST_TRACE_VALUE(v) TgtDbgPostTraceValue(v)
#else
#define TGT_DBG_SIGNAL_TRACE_POINT(p)
#define TGT_DBG_POST_TRACE_VALUE(v)
#endif

#define EDRV_COUNT_SEND TGT_DBG_SIGNAL_TRACE_POINT(2)
#define EDRV_COUNT_TIMEOUT TGT_DBG_SIGNAL_TRACE_POINT(3)
#define EDRV_COUNT_PCI_ERR TGT_DBG_SIGNAL_TRACE_POINT(4)
#define EDRV_COUNT_TX TGT_DBG_SIGNAL_TRACE_POINT(5)
#define EDRV_COUNT_RX TGT_DBG_SIGNAL_TRACE_POINT(6)
#define EDRV_COUNT_LATECOLLISION TGT_DBG_SIGNAL_TRACE_POINT(10)
#define EDRV_COUNT_TX_COL_RL TGT_DBG_SIGNAL_TRACE_POINT(11)
#define EDRV_COUNT_TX_FUN TGT_DBG_SIGNAL_TRACE_POINT(12)
#define EDRV_COUNT_TX_TEST TGT_DBG_SIGNAL_TRACE_POINT(13)
#define EDRV_COUNT_RX_ERR_CRC TGT_DBG_SIGNAL_TRACE_POINT(14)
#define EDRV_COUNT_RX_ERR_MULT TGT_DBG_SIGNAL_TRACE_POINT(15)
#define EDRV_COUNT_RX_ERR_SEQ TGT_DBG_SIGNAL_TRACE_POINT(16)
#define EDRV_COUNT_RX_ERR_OTHER TGT_DBG_SIGNAL_TRACE_POINT(17)
#define EDRV_COUNT_RX_ORUN TGT_DBG_SIGNAL_TRACE_POINT(18)

#define EDRV_TRACE_CAPR(x) TGT_DBG_POST_TRACE_VALUE(((x) & 0xFFFF) | 0x06000000)
#define EDRV_TRACE_RX_CRC(x) TGT_DBG_POST_TRACE_VALUE(((x) & 0xFFFF) | 0x0E000000)
#define EDRV_TRACE_RX_ERR(x) TGT_DBG_POST_TRACE_VALUE(((x) & 0xFFFF) | 0x0F000000)
#define EDRV_TRACE_RX_PUN(x) TGT_DBG_POST_TRACE_VALUE(((x) & 0xFFFF) | 0x11000000)
#define EDRV_TRACE(x) TGT_DBG_POST_TRACE_VALUE(((x) & 0xFFFF0000) | 0x0000FEC0)

//---------------------------------------------------------------------------
// local types
//---------------------------------------------------------------------------

// Private structure
typedef struct
{
    struct pci_dev* m_pPciDev; // pointer to PCI device structure
    void* m_pIoAddr; // pointer to register space of Ethernet controller

    BYTE* m_apbRxBufInDesc[EDRV_MAX_RX_DESCS];
                                        // Stack of free rx buffers
                                        // +1 additional place if ReleaseRxBuffer is called
                                        // before return of RxHandler (multi processor)
    BYTE* m_apbRxBufFree[EDRV_MAX_RX_BUFFERS - EDRV_MAX_RX_DESCS + 1];
    int m_iRxBufFreeTop;
    spinlock_t m_SpinLockTx;
    int m_iPageAllocations;

    BYTE* m_pbTxBuf; // pointer to Tx buffer
    dma_addr_t m_pTxBufDma;
    tEdrvTxBuffer* m_apTxBuffer[EDRV_MAX_TX_DESCS];
    BOOL m_afTxBufUsed[EDRV_MAX_TX_BUFFERS];

    UINT m_uiHeadTxDesc;
    UINT m_uiTailTxDesc;
    UINT m_uiHeadRxDesc;
    UINT m_uiTailRxDesc;

    tEdrvInitParam m_InitParam;

    // variable used to store EEPROM address bits
    UINT m_uiEepromAddrBits;

    BYTE *m_pCbVirtAdd;
    BYTE *m_pRfdVirtAdd;

    dma_addr_t m_CbDmaHandle;
    dma_addr_t m_RfdDmaAdd;

    // array to store virtual address of pTxBuffer
    volatile ULONG m_ulaCbVirtAddrBuf[MAX_CBS];
    // array to store dma mapped address of pTxBuffer->m_pBuffer
    volatile ULONG m_ulaCbDmaAddrBuf[MAX_CBS];

    WORD m_wMulticastAddrByteCnt;


} tEdrvInstance;

//---------------------------------------------------------------------------
// local function prototypes
//---------------------------------------------------------------------------

// custom function prototypes
tEplKernel EdrvIndividualAddressCmd(UINT uiOpcode_p, UINT uiCount_p);
tEplKernel EdrvConfigureCmd(UINT uiOpcode_p, UINT uiCount_p);
tEplKernel EdrvMulticastCmd(UINT uiOpcode_p, UINT uiCount_p, BYTE *pbMacAddr_p,
                                 UINT uiMode_p);
tEplKernel EdrvTransmitCmd(UINT uiOpcode_p, UINT uiCount_p);

void EdrvIfeEepromDelay(void);
static void EdrvIfeCheckEepromSize(void);
static WORD EdrvIfeReadEeprom(UINT uiAddr_p);
tEplKernel EdrvIfeCmdDescWrite(UINT uiOpcode_p, UINT uiCount_p);
tEplKernel EdrvIfeRxDescWrite(INT iCount_p);

static INT EdrvInitOne(struct pci_dev *pPciDev_p,
                       const struct pci_device_id *pId_p);

static void EdrvRemoveOne(struct pci_dev *pPciDev_p);

//---------------------------------------------------------------------------
// module global vars
//---------------------------------------------------------------------------
// buffers and buffer descriptors and pointers

static struct pci_device_id aEdrvPciTbl[] = {
    {0x8086, 0x1091, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0},
    {0x8086, 0x1092, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0},
    {0x8086, 0x1093, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0},
    {0x8086, 0x1094, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0},
    {0x8086, 0x1095, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0},
    {0x8086, 0x1209, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0}, // Intel Corporation 8255xER/82551IT (APC-620)
    {0x8086, 0x1229, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0}, // Intel Corporation 82557/8/9/0/1 Ethernet Pro 100
    {0,}
};
MODULE_DEVICE_TABLE (pci, aEdrvPciTbl);

static tEdrvInstance EdrvInstance_l;

static struct pci_driver EdrvDriver = {
    .name = DRV_NAME,
    .id_table = aEdrvPciTbl,
    .probe = EdrvInitOne,
    .remove = EdrvRemoveOne,
};

//---------------------------------------------------------------------------
//
// Function: EdrvInit
//
// Description: function for init of the Ethernet controller
//
// Parameters: pEdrvInitParam_p = pointer to struct including the init-parameters
//
// Returns: Errorcode = kEplSuccessful
// = kEplNoResource
//
// State:
//
//---------------------------------------------------------------------------
tEplKernel EdrvInit(tEdrvInitParam * pEdrvInitParam_p)
{
tEplKernel Ret;
INT iResult;
INT iIndex;

    Ret = kEplSuccessful;

    // clear instance structure
    EPL_MEMSET(&EdrvInstance_l, 0, sizeof (EdrvInstance_l));

    // save the init data
    EdrvInstance_l.m_InitParam = *pEdrvInitParam_p;

    // clear driver structure
    EPL_MEMSET(&EdrvDriver, 0, sizeof (EdrvDriver));
    EdrvDriver.name = DRV_NAME,
    EdrvDriver.id_table = aEdrvPciTbl,
    EdrvDriver.probe = EdrvInitOne,
    EdrvDriver.remove = EdrvRemoveOne,

    // register PCI driver
    iResult = pci_register_driver (&EdrvDriver);
    if (iResult != 0)
    {
        printk("%s pci_register_driver failed with %d\n", __FUNCTION__, iResult);
        Ret = kEplNoResource;
        goto Exit;
    }

    if (EdrvInstance_l.m_pPciDev == NULL)
    {
        printk("%s m_pPciDev=NULL\n", __FUNCTION__);
        Ret = EdrvShutdown();
        Ret = kEplNoResource;
        goto Exit;
    }

    // local MAC address might have been changed in EdrvInitOne
    EPL_MEMCPY(pEdrvInitParam_p->m_abMyMacAddr, EdrvInstance_l.m_InitParam.m_abMyMacAddr, 6);

    printk("%s local MAC = ", __FUNCTION__);
    for (iIndex = 0; iIndex < 6; iIndex++)
    {
        printk("%02X ", (UINT)pEdrvInitParam_p->m_abMyMacAddr[iIndex]);
    }
    printk("\n");

Exit:
    return Ret;
}

//---------------------------------------------------------------------------
//
// Function: EdrvShutdown
//
// Description: Shutdown the Ethernet controller
//
// Parameters: void
//
// Returns: Errorcode = kEplSuccessful
//
// State:
//
//---------------------------------------------------------------------------
tEplKernel EdrvShutdown(void)
{
    // unregister PCI driver
    printk("%s calling pci_unregister_driver()\n", __FUNCTION__);
    pci_unregister_driver (&EdrvDriver);

    return kEplSuccessful;
}

//---------------------------------------------------------------------------
//
// Function: EdrvDefineRxMacAddrEntry
//
// Description: Set a multicast entry into the Ethernet controller
//
// Parameters: pbMacAddr_p = pointer to multicast entry to set
//
// Returns: Errorcode = kEplSuccessful
//
// State:
//
//---------------------------------------------------------------------------
tEplKernel EdrvDefineRxMacAddrEntry (BYTE * pbMacAddr_p)
{
tEplKernel Ret = kEplSuccessful;
struct cb *pCb;
BYTE *pbCnt;
int iCbCnt = 0;
static BOOL fIsFirstEntry = FALSE;

    if(unlikely(FALSE == fIsFirstEntry))
    {
        fIsFirstEntry = TRUE;
        //pointer to the Command Block specified by the count value
        pCb = (struct cb *)(EdrvInstance_l.m_pCbVirtAdd + (CB_REQUIRED_SIZE * iCbCnt));

        //set the byte count (number of mac addresses * 6 bytes per mac address)
        //in the corresponding section of the descriptor to zero
        EdrvInstance_l.m_wMulticastAddrByteCnt = 0;
        pbCnt = (BYTE *)&pCb[1];
        *(WORD *)pbCnt = EdrvInstance_l.m_wMulticastAddrByteCnt;
    }

    Ret = EdrvMulticastCmd(OP_MULTICAST, iCbCnt, pbMacAddr_p, MULTICAST_ADDR_ADD);

    return Ret;
}

//---------------------------------------------------------------------------
//
// Function: EdrvUndefineRxMacAddrEntry
//
// Description: Reset a multicast entry in the Ethernet controller
//
// Parameters: pbMacAddr_p = pointer to multicast entry to reset
//
// Returns: Errorcode = kEplSuccessful
//
// State:
//
//---------------------------------------------------------------------------
tEplKernel EdrvUndefineRxMacAddrEntry (BYTE * pbMacAddr_p)
{
tEplKernel Ret = kEplSuccessful;

    Ret = EdrvMulticastCmd(OP_MULTICAST, 0, pbMacAddr_p, MULTICAST_ADDR_REM);

    return Ret;
}


tEplKernel EdrvChangeFilter(tEdrvFilter* pFilter_p,
                            UINT uiCount_p,
                            UINT uiEntryChanged_p,
                            UINT uiChangeFlags_p)
{
tEplKernel Ret = kEplSuccessful;

    return Ret;
}

//---------------------------------------------------------------------------
//
// Function: EdrvAllocTxMsgBuffer
//
// Description: Register a Tx-Buffer
//
// Parameters: pBuffer_p = pointer to Buffer structure
//
// Returns: Errorcode = kEplSuccessful
// = kEplEdrvNoFreeBufEntry
//
// State:
//
//---------------------------------------------------------------------------
tEplKernel EdrvAllocTxMsgBuffer (tEdrvTxBuffer * pBuffer_p)
{
tEplKernel Ret = kEplSuccessful;
DWORD dwIndex;

    if (pBuffer_p->m_uiMaxBufferLen > EDRV_MAX_FRAME_SIZE)
    {
        Ret = kEplEdrvNoFreeBufEntry;
        goto Exit;
    }

    if (EdrvInstance_l.m_pbTxBuf == NULL)
    {
        printk("%s Tx buffers currently not allocated\n", __FUNCTION__);
        Ret = kEplEdrvNoFreeBufEntry;
        goto Exit;
    }

    // search a free Tx buffer with appropriate size
    for (dwIndex = 0; dwIndex < EDRV_MAX_TX_BUFFERS; dwIndex++)
    {
        if (EdrvInstance_l.m_afTxBufUsed[dwIndex] == FALSE)
        {
            // free channel found
            EdrvInstance_l.m_afTxBufUsed[dwIndex] = TRUE;
            pBuffer_p->m_BufferNumber.m_dwVal = dwIndex;
            pBuffer_p->m_pbBuffer = EdrvInstance_l.m_pbTxBuf + (dwIndex * EDRV_MAX_FRAME_SIZE);
            pBuffer_p->m_uiMaxBufferLen = EDRV_MAX_FRAME_SIZE;
            break;
        }
    }
    if (dwIndex >= EDRV_MAX_TX_BUFFERS)
    {
        Ret = kEplEdrvNoFreeBufEntry;
        goto Exit;
    }

Exit:
    return Ret;

}

//---------------------------------------------------------------------------
//
// Function: EdrvReleaseTxMsgBuffer
//
// Description: Register a Tx-Buffer
//
// Parameters: pBuffer_p = pointer to Buffer structure
//
// Returns: Errorcode = kEplSuccessful
//
// State:
//
//---------------------------------------------------------------------------
tEplKernel EdrvReleaseTxMsgBuffer (tEdrvTxBuffer * pBuffer_p)
{
UINT uiBufferNumber;

    uiBufferNumber = pBuffer_p->m_BufferNumber.m_dwVal;

    if (uiBufferNumber < EDRV_MAX_TX_BUFFERS)
    {
        EdrvInstance_l.m_afTxBufUsed[uiBufferNumber] = FALSE;
    }

    return kEplSuccessful;

}

//---------------------------------------------------------------------------
//
// Function: EdrvIfeIssueScbcmd
//
// Description: issues the scb command for the CU or RU as required
//
// Parameters: wCmd_p - command to be issued to the CU or RU
// uiArg_p - address to be written to the genptr register
// uiOpcode_p - the opcode used while issuing this command
//
// Returns: (int) = error code
//
// State:
//
//---------------------------------------------------------------------------
static bool EdrvIfeIssueScbcmd(WORD wCmd_p, UINT uiArg_p, UINT uiOpcode_p)
{
WORD wStat = 0;
static INT iCuStatusFirstTime = 0;
static WORD wPrevCmd = 0;

    // this code section used for waiting till the previous cmd has been accepted
    while (ioread8(EdrvInstance_l.m_pIoAddr + SCBCMD) & (SC_CUC | SC_RUC));

    if(OP_TX == uiOpcode_p)
    {
        wStat = ioread16(EdrvInstance_l.m_pIoAddr + SCBSTAT);
        while((wStat & SS_CNA) == 0 && (iCuStatusFirstTime == 1))
        {
            wStat = ioread16(EdrvInstance_l.m_pIoAddr + SCBSTAT);
        }
        if(iCuStatusFirstTime < 1 )
        {
            iCuStatusFirstTime++;
        }
    }

    switch (wCmd_p)
    {
        case SC_CUC_START:
        case SC_CUC_LOADSDMP:
        case SC_CUC_LOADBASE:
        case SC_RUC_START:
        case SC_RUC_LOADHDS:
        case SC_RUC_LOADBASE:
            iowrite32(uiArg_p, EdrvInstance_l.m_pIoAddr + GENPTR);
            break;
    }
    iowrite16(SS_CNA, EdrvInstance_l.m_pIoAddr + SCBSTAT);
    iowrite8(wCmd_p, EdrvInstance_l.m_pIoAddr + SCBCMD);

    wPrevCmd = wCmd_p;

    return TRUE;
}

//---------------------------------------------------------------------------
//
// Function: EdrvSendTxMsg
//
// Description: immediately starts the transmission of the buffer
//
// Parameters: pBuffer_p = buffer descriptor to transmit
//
// Returns: Errorcode = kEplSuccessful
//
// State:
//
//---------------------------------------------------------------------------
tEplKernel EdrvSendTxMsg (tEdrvTxBuffer * pTxBuffer_p)
{
tEplKernel Ret = kEplSuccessful;
struct cb *cbp;

    if (((EdrvInstance_l.m_uiTailTxDesc + 1) % MAX_CBS) == EdrvInstance_l.m_uiHeadTxDesc)
    {
        Ret = kEplEdrvNoFreeTxDesc;
            goto Exit;
    }

    cbp = (struct cb *)((EdrvInstance_l.m_pCbVirtAdd) + (CB_REQUIRED_SIZE * EdrvInstance_l.m_uiTailTxDesc));

    // array to store virtual address of pTxBuffer
    EdrvInstance_l.m_ulaCbVirtAddrBuf[EdrvInstance_l.m_uiTailTxDesc] = (ULONG)pTxBuffer_p;

    // array to store dma mapped address of data pointed by pTxBuffer_p->m_pbBuffer
    EdrvInstance_l.m_ulaCbDmaAddrBuf[EdrvInstance_l.m_uiTailTxDesc] = pci_map_single(EdrvInstance_l.m_pPciDev, pTxBuffer_p->m_pbBuffer,
                                                pTxBuffer_p->m_uiTxMsgLen, PCI_DMA_TODEVICE);



    // fill the TXCB descriptor with the relevant data

    // increment Tx descriptor queue tail pointer
    EdrvInstance_l.m_uiTailTxDesc = ((EdrvInstance_l.m_uiTailTxDesc + 1) % MAX_CBS);

    if(0 == EdrvInstance_l.m_uiTailTxDesc)
    {
        EdrvIfeCmdDescWrite(OP_TX, (MAX_CBS-1));
        EdrvIfeIssueScbcmd(SC_CUC_START, (EdrvInstance_l.m_CbDmaHandle) + (CB_REQUIRED_SIZE * (MAX_CBS-1)),OP_TX);
    }
    else
    {
    EdrvIfeCmdDescWrite(OP_TX, (EdrvInstance_l.m_uiTailTxDesc - 1));
    EdrvIfeIssueScbcmd(SC_CUC_START, EdrvInstance_l.m_CbDmaHandle + (CB_REQUIRED_SIZE * (EdrvInstance_l.m_uiTailTxDesc - 1)),OP_TX);
    }

    Exit:
        return Ret;
}

//---------------------------------------------------------------------------
//
// Function: EdrvInterruptHandler
//
// Description: interrupt handler
//
// Parameters: void
//
// Returns: void
//
// State:
//
//---------------------------------------------------------------------------
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,19)
static irqreturn_t TgtEthIsr (INT nIrqNum_p, void* ppDevInstData_p)
#else
static INT TgtEthIsr (INT nIrqNum_p, void* ppDevInstData_p, struct pt_regs* ptRegs_p)
#endif
{
INT iHandled;
WORD wStat;
struct cb *cbp;
UINT uiCmdStat;
struct rfd *rfdp;
tEdrvTxBuffer* pTxBuffer = NULL;
tEdrvTxBuffer* pDmaBuffer = NULL;

    iHandled = IRQ_HANDLED;
    wStat = ioread16(EdrvInstance_l.m_pIoAddr + SCBSTAT);

    if(((wStat & 0xFF00) == 0x0000) || ((wStat & 0xFF00) == 0xFF00))
    {
        iHandled = IRQ_NONE;
        goto Exit;
    }

    /* clear interrupt */
    iowrite16((wStat & 0xDF00), EdrvInstance_l.m_pIoAddr + SCBSTAT);

    // open if(TX/RX interrupt)
    if ((wStat & SS_CX) | (wStat & (SS_FR | SS_RNR)))
    {
        if (EdrvInstance_l.m_pbTxBuf == NULL)
        {
            printk("%s Tx buffers currently not allocated\n", __FUNCTION__);
            goto Exit;
        }

        if (wStat & SS_RNR)
        {
            printk("RU not ready\n");
        }

        do
        {
            // Process receive descriptors
            rfdp = (struct rfd *)((EdrvInstance_l.m_pRfdVirtAdd) + (RFD_REQUIRED_SIZE * EdrvInstance_l.m_uiHeadRxDesc));

            while (rfdp->m_uiCmdStat & CS_C)
            { // Rx frame available
                tEdrvRxBuffer RxBuffer;
                tEdrvReleaseRxBuffer RetReleaseRxBuffer;

                if (((rfdp->m_uiSize & RFD_EOF) != 0) && ((rfdp->m_uiSize & RFD_F) != 0))
                { // Descriptor is valid
                    // Packet is OK

                    RxBuffer.m_BufferInFrame = kEdrvBufferLastInFrame;

                    // Get length of received packet
                    // In the default configuration for 82559 CRC is not transferred into host memory
                    // so we can use the RX byte count from the RFD as it is
                    RxBuffer.m_uiRxMsgLen = (rfdp->m_uiSize) & RFD_COUNT;

                    RxBuffer.m_pbBuffer = ((EdrvInstance_l.m_pRfdVirtAdd + (RFD_REQUIRED_SIZE * EdrvInstance_l.m_uiHeadRxDesc))+(sizeof(struct rfd)));

                    // Call Rx handler of Data link layer

                    RetReleaseRxBuffer = EdrvInstance_l.m_InitParam.m_pfnRxHandler(&RxBuffer);
                }// closing Descriptor is valid

                // clean the status bits of the currently handled descriptor
                // so that it is available for use the next time
                EdrvIfeRxDescWrite(EdrvInstance_l.m_uiHeadRxDesc);
                EdrvInstance_l.m_uiHeadRxDesc = ((EdrvInstance_l.m_uiHeadRxDesc + 1) % MAX_RFDS);
                rfdp = (struct rfd *)((EdrvInstance_l.m_pRfdVirtAdd) + (RFD_REQUIRED_SIZE * EdrvInstance_l.m_uiTailRxDesc));
                rfdp->m_uiCmdStat = 0x00000000;
                if (0 == EdrvInstance_l.m_uiHeadRxDesc)
                {
                    EdrvInstance_l.m_uiTailRxDesc = MAX_RFDS - 1;
                }
                else
                {
                    EdrvInstance_l.m_uiTailRxDesc = EdrvInstance_l.m_uiHeadRxDesc - 1;
                }
                rfdp = (struct rfd *)((EdrvInstance_l.m_pRfdVirtAdd) + (RFD_REQUIRED_SIZE * EdrvInstance_l.m_uiTailRxDesc));
                rfdp->m_uiCmdStat = CS_S;
                rfdp = (struct rfd *)(EdrvInstance_l.m_pRfdVirtAdd + (RFD_REQUIRED_SIZE * EdrvInstance_l.m_uiHeadRxDesc));
            }// closing RX while loop

            cbp = (struct cb *)((EdrvInstance_l.m_pCbVirtAdd) + (CB_REQUIRED_SIZE * EdrvInstance_l.m_uiHeadTxDesc));

            if(cbp->m_uiCmdStat & CS_C)
            {
                uiCmdStat = cbp->m_uiCmdStat;
                // clear the status bits of the current CB
                cbp->m_uiCmdStat &= 0XFFFF0000;

                // retrieve the address of the pTxBuffer pointer
                // that was received in EdrvSendTxMsg
                pTxBuffer = (tEdrvTxBuffer *)EdrvInstance_l.m_ulaCbVirtAddrBuf[EdrvInstance_l.m_uiHeadTxDesc];

                pDmaBuffer = (tEdrvTxBuffer *)EdrvInstance_l.m_ulaCbDmaAddrBuf[EdrvInstance_l.m_uiHeadTxDesc];

                EdrvInstance_l.m_ulaCbVirtAddrBuf[EdrvInstance_l.m_uiHeadTxDesc] = 0;

                // Increment Tx descriptor queue head pointer
                EdrvInstance_l.m_uiHeadTxDesc = ((EdrvInstance_l.m_uiHeadTxDesc + 1) % MAX_CBS);

                if(NULL != pDmaBuffer)
                {
                    // retrieve the address of the DMA handle to
                    // pTxBuffer_p->m_pbBuffer pointer
                    // that was received in EdrvSendTxMsg
                    // so that the mapping can be correspondingly unmapped
                    pci_unmap_single(EdrvInstance_l.m_pPciDev,
                                     (dma_addr_t) EdrvInstance_l.m_ulaCbDmaAddrBuf[EdrvInstance_l.m_uiHeadTxDesc],
                                     pTxBuffer->m_uiTxMsgLen, PCI_DMA_TODEVICE);
                }

                if (pTxBuffer != NULL)
                {
                    // Call Tx handler of Data link layer
                    if (pTxBuffer->m_pfnTxHandler != NULL)
                    {
                        pTxBuffer->m_pfnTxHandler(pTxBuffer);
                        pTxBuffer=NULL;
                    }
                }
                else
                {
                        //Currently not handled
                }
            }//Closing if C bit set in the CB
            else
            {
                    break;
            }
        }while (EdrvInstance_l.m_uiHeadTxDesc != EdrvInstance_l.m_uiTailTxDesc);

    }// close if(TX/RX interrupt)

Exit:
    return iHandled;
}

//---------------------------------------------------------------------------
//
// Function: EdrvIfeEepromDelay
//
// Description: To give a delay before successive operations
//
// Parameters: void
//
// Returns: void
//
//
//---------------------------------------------------------------------------

//jba: Is this always reliable on different processors?

/* 2 reads required for 66MHz operation */
void EdrvIfeEepromDelay(void)
{
    ioread8(EdrvInstance_l.m_pIoAddr + EECTRL);
    ioread8(EdrvInstance_l.m_pIoAddr + EECTRL);
}

//---------------------------------------------------------------------------
//
// Function: EdrvIfeCheckEepromSize
//
// Description: To check the size
//
// Parameters: void
//
// Returns: void
//
//---------------------------------------------------------------------------
static void EdrvIfeCheckEepromSize(void)
{
INT iLoopCount;
BYTE bChipSelect;
BYTE bDi;

    /* enable eeprom interface register */
    bChipSelect = EC_EECS;

    iowrite8(bChipSelect, EdrvInstance_l.m_pIoAddr + EECTRL);

    /* output eeprom command */
    for (iLoopCount = 4; iLoopCount >= 0; iLoopCount--)
    {
        bDi = ((EEPROM_READ_CMD >> iLoopCount) & 1) << EC_EEDI_SHIFT;

        iowrite8(bChipSelect | bDi,EdrvInstance_l.m_pIoAddr + EECTRL);

        iowrite8(bChipSelect | bDi | EC_EESK, EdrvInstance_l.m_pIoAddr + EECTRL);
        EdrvIfeEepromDelay();


        iowrite8(bChipSelect | bDi, EdrvInstance_l.m_pIoAddr + EECTRL);
        EdrvIfeEepromDelay();
    }

    /* How many address bits required until eeprom responds with 0 */
    iLoopCount = 0;
    do {
        iowrite8(bChipSelect, EdrvInstance_l.m_pIoAddr + EECTRL);

        iowrite8(bChipSelect | EC_EESK, EdrvInstance_l.m_pIoAddr + EECTRL);
        EdrvIfeEepromDelay();

        iowrite8(bChipSelect, EdrvInstance_l.m_pIoAddr + EECTRL);
        EdrvIfeEepromDelay();
        iLoopCount++;
    } while ((ioread8(EdrvInstance_l.m_pIoAddr + EECTRL) & EC_EEDO) && iLoopCount < 8 );
    /* save the result */

    EdrvInstance_l.m_uiEepromAddrBits = iLoopCount;

    /* read 16bits of data to terminate the sequence */
    for (iLoopCount = 16; iLoopCount > 0; iLoopCount--)
    {
        iowrite8(bChipSelect | EC_EESK, EdrvInstance_l.m_pIoAddr + EECTRL);
        EdrvIfeEepromDelay();

        iowrite8(bChipSelect, EdrvInstance_l.m_pIoAddr + EECTRL);
        EdrvIfeEepromDelay();
    }

    /* De-activate the EEPROM */

    iowrite8(0, EdrvInstance_l.m_pIoAddr + EECTRL);
}

//---------------------------------------------------------------------------
//
// Function: EdrvIfeReadEeprom
//
// Description: To read the EEPROM for the MAC address
//
// Parameters: uiAddr_p = Address from which MAC has to be read
//
// Returns: MAC address
//
//---------------------------------------------------------------------------
static WORD EdrvIfeReadEeprom(UINT uiAddr_p)
{
INT iLoopCount;
UINT uiCmd;
BYTE bChipSelect;
BYTE bDi;
WORD wRet;

    if (uiAddr_p >= (1 << EdrvInstance_l.m_uiEepromAddrBits))
    {
        return (0);
    }

    /* make command bits */
    uiCmd = (EEPROM_READ_CMD << EdrvInstance_l.m_uiEepromAddrBits) | uiAddr_p;

    /* enable eeprom interface register */
    bChipSelect = EC_EECS;

    iowrite8(bChipSelect , EdrvInstance_l.m_pIoAddr + EECTRL);

    EdrvIfeEepromDelay();

    /* output eeprom command */
    for (iLoopCount = 4 + EdrvInstance_l.m_uiEepromAddrBits; iLoopCount >= 0; iLoopCount--)
    {
        bDi = ((uiCmd >> iLoopCount) & 1) << EC_EEDI_SHIFT;

        iowrite8(bChipSelect | bDi, EdrvInstance_l.m_pIoAddr + EECTRL);
        iowrite8(bChipSelect | bDi | EC_EESK, EdrvInstance_l.m_pIoAddr + EECTRL);
        EdrvIfeEepromDelay();

        iowrite8(bChipSelect | bDi, EdrvInstance_l.m_pIoAddr + EECTRL);
        EdrvIfeEepromDelay();
    }

    /* get returned value */
    wRet = 0;
    for (iLoopCount = 16; iLoopCount > 0; iLoopCount--)
    {
        /* get 1 bit */
        iowrite8(bChipSelect | EC_EESK, EdrvInstance_l.m_pIoAddr + EECTRL);
        EdrvIfeEepromDelay();

        wRet = (wRet << 1)
              | ((ioread8(EdrvInstance_l.m_pIoAddr + EECTRL) >> EC_EEDO_SHIFT) & 1);

        iowrite8(bChipSelect, EdrvInstance_l.m_pIoAddr + EECTRL);
        EdrvIfeEepromDelay();
    }

    /* Terminate the EEPROM access. */

    iowrite8(0, EdrvInstance_l.m_pIoAddr + EECTRL);
    EdrvIfeEepromDelay();

    return (wRet);
}

//---------------------------------------------------------------------------
//
// Function: EdrvIndividualAddressCmd
//
// Description: To insert the MAC address
//
// Parameters: uiOpcode_p = opcode to be filled in the Command Block
// uiCount_p = count value which indicates which Command Block is to
// be filled
//
// Returns: Ret = kEplSuccessful
//
//---------------------------------------------------------------------------
tEplKernel EdrvIndividualAddressCmd(UINT uiOpcode_p, UINT uiCount_p)
{
cbstruct *cbstructp;
tEplKernel Ret = kEplSuccessful;

    //pointer to the Command Block specified by the count value
    cbstructp = (cbstruct *)(EdrvInstance_l.m_pCbVirtAdd + (CB_REQUIRED_SIZE * uiCount_p));

    //fill the individual address command to be executed
    cbstructp->m_uiStatCommand = CS_EL | (uiOpcode_p<<CS_OP_SHIFT); //0x80010000

    //The MAC address bytes used below have been read from the EEPROM

    //MAC ADDRESS (BYTE4<<24) | (BYTE3<<16) | (BYTE2<<8) | (BYTE1<<0)
    cbstructp->m_uiValue1 = EdrvInstance_l.m_InitParam.m_abMyMacAddr[0] |
                            EdrvInstance_l.m_InitParam.m_abMyMacAddr[1] << 8 |
                            EdrvInstance_l.m_InitParam.m_abMyMacAddr[2] << 16 |
                            EdrvInstance_l.m_InitParam.m_abMyMacAddr[3] << 24;

    //MAC ADDRESS (BYTE6<<8) | (BYTE5<<0)
    cbstructp->m_uiValue2 = EdrvInstance_l.m_InitParam.m_abMyMacAddr[4] |
                                            EdrvInstance_l.m_InitParam.m_abMyMacAddr[5] << 8;

    //start command issued to execute the individual address setup
    EdrvIfeIssueScbcmd(SC_CUC_START, EdrvInstance_l.m_CbDmaHandle, OP_ADDRSETUP);

    //wait for command to complete successfully
    while((cbstructp->m_uiStatCommand & CS_C)==0);
    //printk("CmdStat Value : %x\n",cbstructp->m_uiStatCommand);
    if(cbstructp->m_uiStatCommand & CS_OK)
    {
        //printk("Individual Address Setup command successful\n");
        Ret = kEplSuccessful;
    }
    else
    {
        //printk("Individual Address Setup command unsuccessful\n");
        Ret = kEplEdrvInitError;
    }

    return Ret;
}

//---------------------------------------------------------------------------
//
// Function: EdrvConfigureCmd
//
// Description: To configure the Ethernet Controller
//
// Parameters: opcode = opcode to be filled in the Command Block
// count = count value which indicates which Command Block is to
// be filled
//
// Returns: Ret = kEplSuccessful
//
//---------------------------------------------------------------------------
tEplKernel EdrvConfigureCmd(UINT uiOpcode_p, UINT uiCount_p)
{
cbstruct *cbstructp;
tEplKernel Ret = kEplSuccessful;

    //pointer to the Command Block specified by the uiCount_p value
    cbstructp = (cbstruct *)(EdrvInstance_l.m_pCbVirtAdd + (CB_REQUIRED_SIZE * uiCount_p));

    //fill the configure command to be executed
    cbstructp->m_uiStatCommand = CS_EL | (uiOpcode_p << CS_OP_SHIFT); //0x80020000

    //(Byte3<<24) | (Byte2<<16) | (Byte1<<8) | (Byte0<<0)
    //Byte0 - Byte Count of 16 bytes used in this configuration
    //Byte1 - Default Value
    //Byte2 - Default Value
    //Byte3 - Default Value
    cbstructp->m_uiValue1 = (0X00<<24) | (0X00<<16) | (0X08<<8) | (0X10<<0); //0x00000810

    //Byte4 - Default Value
    //Byte5 - Default Value
    //Byte6 - Default Value for 82559
    //Byte7 - Default Value for 82559
    //(Byte7<<24) | (Byte6<<16) | (Byte5<<8) | (Byte4<<0)
    cbstructp->m_uiValue2 = (0X00<<24) | (0X30<<16) | (0X00<<8) | (0X00<<0); //0x00300000

    //Byte8 - Bit0 set for 82559, Bit0 cleared to enable link operation, Rest of the
    // bits are default values for 82559
    //Byte9 - Default value for 82559
    //Byte10 - Loopback disabled, default preamble length, source address insertion
    // disabled, other bits default for 82559
    //Byte11 - Default value for 82559
    //(Byte11<<24) | (Byte10<<16) | (Byte9<<8) | (Byte8<<0)
    cbstructp->m_uiValue3 = (0X00<<24) | (0X2E<<16) | (0X00<<8) | (0X01<<0); //0x002E0001

    //Byte12 - Interframe spacing - default value, other bits default value for 82559
    //Byte13 - Default Value for 82559
    //Byte14 - Default Value for 82559
    //Byte15 - Bit0 cleared to disable promiscuous mode, other bits default value for 82559
    //(Byte15<<24) | (Byte14<<16) | (Byte13<<8) | (Byte12<<0)
    cbstructp->m_uiValue4 = (0XC8<<24) | (0XF2<<16) | (0X00<<8) | (0X61<<0); //0xC8F20061

    //NOTE: the following configuration bytes not yet configured
    //Byte16 - Default Value
    //Byte17 - Default Value
    //Byte18 - Default Value
    //Byte19 - Default Value
    //(Byte19<<24) | (Byte18<<16) | (Byte17<<8) | (Byte16<<0)
    //cbstructp->m_uiValue5 = (0X80<<24) | (0XF2<<16) | (0X40<<8) | (0X00<<0); //

    //Byte20 - Defaut Value
    //Byte21 - Default Value
    //Byte22 - Zero Padding
    //Byte23 - Zero Padding
    //(Byte23<<24) | (Byte22<<16) | (Byte21<<8) | (Byte20<<0)
    //cbstructp->m_uiValue6 = (0X00<<24) | (0X00<<16) | (0X05<<8) | (0X3F<<0); //

    //start command issued to configure the device as per the configuration byte values
    EdrvIfeIssueScbcmd(SC_CUC_START, EdrvInstance_l.m_CbDmaHandle, OP_CONFIGURE);

    //wait for command to complete successfully
    while((cbstructp->m_uiStatCommand & CS_C) == 0);

    if(cbstructp->m_uiStatCommand & CS_OK)
    {
        Ret = kEplSuccessful;
    }
    else
    {
        Ret = kEplEdrvInitError;
    }

    return Ret;
}

//---------------------------------------------------------------------------
//
// Function: EdrvMulticastCmd
//
// Description: To fill the Multicast Command Block
//
// Parameters: uiOpcode_p = opcode to be filled in the Command Block
// uiCount_p = count value which indicates which Command
// Block is to be filled
// pbMacAddr_p = pointer to the multicast address to be filled
// uiMode_p = index variable used for the position of the
// multicast address in the command block
//
// Returns: Ret = kEplSuccessful
//
//---------------------------------------------------------------------------
tEplKernel EdrvMulticastCmd(UINT uiOpcode_p, UINT uiCount_p, BYTE *pbMacAddr_p,
                            UINT uiMode_p)
{
struct cb *cbp;
BYTE *bp;
WORD *pwByteCount;
WORD wMulticastAddrCnt;
WORD wMulticastAddrLoop;
UINT uiMemCmpref = 0;
tEplKernel Ret = kEplSuccessful;

    //pointer to the Command Block specified by the uiCount_p value
    cbp = (struct cb *)(EdrvInstance_l.m_pCbVirtAdd + (CB_REQUIRED_SIZE * uiCount_p));

    //fill the multicast command to be executed
    cbp->m_uiCmdStat = CS_EL | (uiOpcode_p << CS_OP_SHIFT); //0x80030000

    bp = (BYTE *)&cbp[1];
    //point the address where the byte uiCount_p for multicast entries to be stored
    pwByteCount = (WORD *)bp;
    wMulticastAddrCnt = EdrvInstance_l.m_wMulticastAddrByteCnt;
    //move the pointr to first multicast entry
    bp += (sizeof (WORD));

    if(MULTICAST_ADDR_ADD == uiMode_p)
    {
        //fill the byte count (number of mac addresses * 6 bytes per mac address)
        //in the corresponding section of the descriptor
        //according to the u32Index value
        EdrvInstance_l.m_wMulticastAddrByteCnt += MAC_ADDRESS_LEN;
        *pwByteCount = EdrvInstance_l.m_wMulticastAddrByteCnt;

        //fill the multicast entry in the corresponding section of
        //the descriptor according to the u32Index value
        bp += wMulticastAddrCnt;
        EPL_MEMCPY(bp, pbMacAddr_p, MAC_ADDRESS_LEN);
    }
    else if(MULTICAST_ADDR_REM == uiMode_p)
    {
        //find the byte count (number of mac addresses * 6 bytes per mac address)
        //in the corresponding section of the descriptor

        // search for the mac address to be removed from multicast address
        for(wMulticastAddrLoop = 0; wMulticastAddrLoop < wMulticastAddrCnt ; wMulticastAddrLoop += MAC_ADDRESS_LEN)
        {
            if(0 == EPL_MEMCMP(bp, pbMacAddr_p, MAC_ADDRESS_LEN))
            {
                // entry found reduce the count and remove that entry from the descriptor
                EdrvInstance_l.m_wMulticastAddrByteCnt -= MAC_ADDRESS_LEN;
                *pwByteCount = EdrvInstance_l.m_wMulticastAddrByteCnt;

                if( 0 != ( wMulticastAddrCnt - (wMulticastAddrLoop + MAC_ADDRESS_LEN) ))
                    EPL_MEMCPY(bp, bp+MAC_ADDRESS_LEN, ( wMulticastAddrCnt - (wMulticastAddrLoop + MAC_ADDRESS_LEN) ) );

                    uiMemCmpref = 1;

                break;
            }
            else
            {
                uiMemCmpref = 0;
            }
            bp += MAC_ADDRESS_LEN;
        }

        if(0 == uiMemCmpref)
        {
                Ret = kEplSuccessful;
                goto Exit;
        }
    }
    else
    {
        printk("%s(): Unknown mode:%d \n", __FUNCTION__, uiMode_p);
        goto Exit;
    }

    //start command issued to set the multicast address
    EdrvIfeIssueScbcmd(SC_CUC_START, EdrvInstance_l.m_CbDmaHandle, uiOpcode_p);

    //wait for command to complete successfully
    while((cbp->m_uiCmdStat & CS_C)==0);

    if(cbp->m_uiCmdStat & CS_OK)
    {
        Ret = kEplSuccessful;
    }
    else
    {
        Ret = kEplEdrvInitError;
    }

Exit:
    return Ret;
}

//---------------------------------------------------------------------------
//
// Function: EdrvTransmitCmd
//
// Description: To fill the Transmit Command Block
//
// Parameters: opcode to be filled in the Command Block
// count value which indicates which Command Block is to be filled
//
// Returns: Ret = kEplSuccessful
//
//---------------------------------------------------------------------------
tEplKernel EdrvTransmitCmd(UINT uiOpcode_p, UINT uiCount_p)
{
UINT tmp0;
tEplKernel Ret = kEplSuccessful;
struct cb *cbp;
struct tcb *tcbp;
struct tbd *tbdp;
UINT cbp_dma;
BYTE *bp;
tEdrvTxBuffer *pTxBuffer;

    bp = NULL;
    cbp = NULL;
    cbp_dma = 0;
    pTxBuffer = NULL;

    pTxBuffer = (tEdrvTxBuffer *)EdrvInstance_l.m_ulaCbVirtAddrBuf[uiCount_p];
    //pointer to the Command Block specified by the uiCount_p value
    cbp = (struct cb *)(EdrvInstance_l.m_pCbVirtAdd + (CB_REQUIRED_SIZE * uiCount_p));
    //physical address corresponding to the virtual address of the Command Block specified above
    cbp_dma = (EdrvInstance_l.m_CbDmaHandle + (CB_REQUIRED_SIZE * uiCount_p));

    bp = (BYTE *)&cbp[1];


    tcbp = (struct tcb *)bp;
    tbdp = (struct tbd *)&tcbp[1];
    //fill the length of the buffer to be transmitted and indicate that it is the end of the frame
    tbdp->m_uiTbdSize = TBD_EL | (pTxBuffer->m_uiTxMsgLen);
    //fill the physical address of the buffer to be transmitted
    tbdp->m_uiTbdAddr = EdrvInstance_l.m_ulaCbDmaAddrBuf[uiCount_p];
    //fill the status bits and the end of frame indication bit in the TCB Control section
    tcbp->m_uiTcbCtrl = (0X01<<TCB_TBDNUM_SHIFT)
                        | (0X01<<TCB_TXTHR_SHIFT)
                        | TCB_EOF; //0x01018000

    //fill in the physical address of the buffer descriptor in the TX descriptor's TCB address section
    tmp0 = cbp_dma +
            sizeof (struct cb) + sizeof (struct tcb);
    tcbp->m_uiTcbTbdPtr = tmp0;

    /* make command block */
    tmp0 = uiOpcode_p << CS_OP_SHIFT;

    //set the interrupt bit in the command
    tmp0 |= CS_I;
    //configure for flexible mode
    tmp0 |= CS_SF_;

    // set the suspend bit in the command
    tmp0 |= CS_S;

    cbp->m_uiCmdStat = tmp0;

    return Ret;
}

//---------------------------------------------------------------------------
//
// Function: EdrvIfeCmdDescWrite
//
// Description: To fill the TX descriptors
//
// Parameters: uiOpcode_p = Opcode value that indicates the command to be
// executed.
// uiCount_p = Count value that indicates the descriptor number
//
// Returns: Ret = kEplSuccessful
//
//---------------------------------------------------------------------------
tEplKernel EdrvIfeCmdDescWrite(UINT uiOpcode_p, UINT uiCount_p)
{
tEplKernel Ret = kEplSuccessful;

    switch (uiOpcode_p)
    {
        case OP_NOP: /* 0 */
            break;

        case OP_ADDRSETUP: /* 1 */
            Ret = EdrvIndividualAddressCmd(uiOpcode_p, uiCount_p);
            break;

        case OP_CONFIGURE: /* 2 */
            Ret = EdrvConfigureCmd(uiOpcode_p, uiCount_p);
            break;

        case OP_MULTICAST: /* 3 */
            break;

        case OP_TX: /* 4 */
            Ret = EdrvTransmitCmd(uiOpcode_p, uiCount_p);
            break;

        default:
            break;
    }
    return Ret;
}

//---------------------------------------------------------------------------
//
// Function: ife_rx_desc_write
//
// Description: Used to clear status bits of the used RX descriptor
//
// Parameters: iCount_p = count which indicates which RX descriptor's status
// bits are to be cleared
//
// Returns: Ret = kEplSuccessful
//
//---------------------------------------------------------------------------
tEplKernel EdrvIfeRxDescWrite(INT iCount_p)
{
struct rfd *rfd;
tEplKernel Ret = kEplSuccessful;

    //select the virtual address of the RX descriptor whose status bits are to be cleared
    rfd = (struct rfd *)((EdrvInstance_l.m_pRfdVirtAdd) + (RFD_REQUIRED_SIZE * iCount_p));

    //clear the status bits of the selected RX descriptor
    rfd->m_uiCmdStat = 0x00000000;

    //clear the status bits in the Size field of the RX descriptor
    rfd->m_uiSize = (((RFD_REQUIRED_SIZE - sizeof (struct rfd)))
                   << RFD_SIZE_SHIFT) & RFD_SIZE;

    return Ret;
}

//---------------------------------------------------------------------------
//
// Function: EdrvInitOne
//
// Description: initializes one PCI device
//
// Parameters: pPciDev_p = pointer to corresponding PCI device structure
// pId_p = PCI device ID
//
// Returns: (int) = error code
//
// State:
//
//-----------------------------EdrvInstance_l.m_pIoAddr----------------------------------------------

static INT EdrvInitOne(struct pci_dev *pPciDev_p, const struct pci_device_id *pId_p)
{
INT iResult = 0;
DWORD dwTemp;
INT iLoop;
struct cb *cbp;
struct rfd *rfdp;
WORD val;
BYTE bTemp;
WORD wTemp;
UINT uiTemp;
tEplKernel Ret = kEplSuccessful;

    if (EdrvInstance_l.m_pPciDev != NULL)
    { // Edrv is already connected to a PCI device
        printk("%s device %s discarded\n", __FUNCTION__, pci_name(pPciDev_p));
        iResult = -ENODEV;
        goto Exit;
    }

    // enable device
    printk("%s enable device\n", __FUNCTION__);
    iResult = pci_enable_device(pPciDev_p);
    if (iResult != 0)
    {
        goto Exit;
    }

    wTemp = 0;
    pci_read_config_word(pPciDev_p, PCI_VENDOR_ID, &wTemp);
    printk("Vendor ID : %d\n",wTemp);

    wTemp = 0;
    pci_read_config_word(pPciDev_p, PCI_DEVICE_ID, &wTemp);
    printk("Device ID : %d\n",wTemp);

    bTemp = 0;
    pci_read_config_byte(pPciDev_p, PCI_REVISION_ID, &bTemp);
    printk("Revision ID : %d\n",bTemp);

    EdrvInstance_l.m_pPciDev = pPciDev_p;

    if (EdrvInstance_l.m_pPciDev == NULL)
    {
        printk("%s pPciDev==NULL\n", __FUNCTION__);
        goto ExitFail;
    }

    //printk("%s request regions\n", __FUNCTION__);
    iResult = pci_request_regions(pPciDev_p, DRV_NAME);
    if (iResult != 0)
    {
        goto ExitFail;
    }

    //printk("%s ioremap\n", __FUNCTION__);
    EdrvInstance_l.m_pIoAddr = ioremap(pci_resource_start(pPciDev_p, 0), pci_resource_len(pPciDev_p, 0));
    if (EdrvInstance_l.m_pIoAddr == NULL)
    { // remap of controller's register space failed
        iResult = -EIO;
        goto ExitFail;
    }

    // enable PCI busmaster
    //printk("%s enable busmaster\n", __FUNCTION__);
    pci_set_master (pPciDev_p);

    // reset chip code - start
    /* clear pended interrupts */
    uiTemp = ioread16(EdrvInstance_l.m_pIoAddr + SCBSTAT);
    udelay(DELAY_CLEAR_INT);
    iowrite16(uiTemp, EdrvInstance_l.m_pIoAddr + SCBSTAT);

    /* write a reset command into PORT register */
    iowrite32(PORT_SOFTRESET, EdrvInstance_l.m_pIoAddr + PORT);

    /* wait 10 system clocks and 5 transmit clocks (10uS) */
    udelay(DELAY_SYS_TX_CLK);

    /* update software copy of device state */
    // lp->cuc_active = B_FALSE;

    /* mask interrupt */
    iowrite8(SC_M >> 8, EdrvInstance_l.m_pIoAddr + SCBCMD + 1);
    // reset chip code - end

    // init chip code start
    /* disable early Rx Int */
    iowrite8(0, EdrvInstance_l.m_pIoAddr + EARLYRXINT);

    /* disable flow control */
    iowrite8(0, EdrvInstance_l.m_pIoAddr + FCTRL);

    // issue commands to scb to load the cu and ru base addresses - start
     EdrvIfeIssueScbcmd(SC_CUC_LOADBASE, 0, 0);
     EdrvIfeIssueScbcmd(SC_RUC_LOADBASE, 0, 0);

     iResult = pci_set_dma_mask(pPciDev_p, DMA_BIT_MASK(32));
     if (iResult != 0)
     {
         //printk(KERN_WARNING "Edrv8255x: No suitable DMA available.\n");
         iResult = -ENOMEM;
         goto ExitFail;
     }

     EdrvInstance_l.m_pCbVirtAdd = pci_alloc_consistent(pPciDev_p, CB_REQUIRED_SIZE * MAX_CBS, &(EdrvInstance_l.m_CbDmaHandle));
    if (EdrvInstance_l.m_pCbVirtAdd == NULL)
    {
        iResult = -ENOMEM;
        goto ExitFail;
    }

    // fill CBs for which memory has been allocated - start
    for(iLoop = 0; iLoop < MAX_CBS; iLoop++)
    {
        cbp = (struct cb *)((EdrvInstance_l.m_pCbVirtAdd) + (CB_REQUIRED_SIZE * iLoop));
        if(iLoop == (MAX_CBS-1))
        {
            cbp->m_uiCmdStat = 0x00000000;
            cbp->m_uiLink = EdrvInstance_l.m_CbDmaHandle;
        }
        else
        {
            cbp->m_uiCmdStat = 0x00000000;
            cbp->m_uiLink = EdrvInstance_l.m_CbDmaHandle + (CB_REQUIRED_SIZE * (iLoop+1));
        }
    }
    // fill CBs for which memory has been allocated - end

    // allocate memory for RFDs - start
    EdrvInstance_l.m_pRfdVirtAdd = pci_alloc_consistent(pPciDev_p, RFD_REQUIRED_SIZE * MAX_RFDS, &(EdrvInstance_l.m_RfdDmaAdd));
    if (EdrvInstance_l.m_pRfdVirtAdd == NULL)
    {
        iResult = -ENOMEM;
        goto ExitFail;
    }
    // allocate memory for RFDs - end

    // fill RFDs for which memory has been allocated - start
    for(iLoop = 0; iLoop < MAX_RFDS; iLoop++)
    {
        rfdp = (struct rfd *)((EdrvInstance_l.m_pRfdVirtAdd) + (RFD_REQUIRED_SIZE * iLoop));
        if(iLoop == (MAX_RFDS-1))
        {
            rfdp->m_uiCmdStat = 0x00000000;
            rfdp->m_uiSize = (((RFD_REQUIRED_SIZE - sizeof (struct rfd)))
                                                                            << RFD_SIZE_SHIFT) & RFD_SIZE;
            rfdp->m_uiLink = EdrvInstance_l.m_RfdDmaAdd;
        }
        else
        {
            rfdp->m_uiCmdStat = 0x00000000;
            rfdp->m_uiSize = (((RFD_REQUIRED_SIZE - sizeof (struct rfd)))
                            << RFD_SIZE_SHIFT) & RFD_SIZE;
            rfdp->m_uiLink = EdrvInstance_l.m_RfdDmaAdd + (RFD_REQUIRED_SIZE * (iLoop+1));
        }
    }

    // fill RFDs for which memory has been allocated - end

    Ret = EdrvIfeCmdDescWrite(OP_ADDRSETUP, 0);
    if(Ret != kEplSuccessful)
    {
        iResult = -EIO;
        goto Exit;
    }
    Ret = EdrvIfeCmdDescWrite(OP_CONFIGURE, 0);
    if(Ret != kEplSuccessful)
    {
        iResult = -EIO;
        goto Exit;
    }

    //acknowledge pended interrupts and clear interrupt mask
    uiTemp = ioread16(EdrvInstance_l.m_pIoAddr + SCBSTAT);
    udelay(5);
    iowrite16(uiTemp, EdrvInstance_l.m_pIoAddr + SCBSTAT);
    iowrite8(0x2C, EdrvInstance_l.m_pIoAddr + SCBCMD + 1);

    // install interrupt handler
    iResult = request_irq(pPciDev_p->irq, TgtEthIsr, IRQF_SHARED, DRV_NAME, pPciDev_p);
    if (iResult != 0)
    {
        goto ExitFail;
    }

    // allocate buffers
    printk("%s allocate buffers\n", __FUNCTION__);
    // allocate tx-buffers
    EdrvInstance_l.m_pbTxBuf = pci_alloc_consistent(pPciDev_p, EDRV_TX_BUFFER_SIZE,
                    &EdrvInstance_l.m_pTxBufDma);
    if (EdrvInstance_l.m_pbTxBuf == NULL)
    {
       iResult = -ENOMEM;
        goto ExitFail;
    }


     // check if user specified a MAC address
    //printk("%s check specified MAC address\n", __FUNCTION__);
    if ((EdrvInstance_l.m_InitParam.m_abMyMacAddr[0] != 0) |
        (EdrvInstance_l.m_InitParam.m_abMyMacAddr[1] != 0) |
        (EdrvInstance_l.m_InitParam.m_abMyMacAddr[2] != 0) |
        (EdrvInstance_l.m_InitParam.m_abMyMacAddr[3] != 0) |
        (EdrvInstance_l.m_InitParam.m_abMyMacAddr[4] != 0) |
        (EdrvInstance_l.m_InitParam.m_abMyMacAddr[5] != 0) )
    { // write specified MAC address to controller
        dwTemp = 0;

        dwTemp |= EdrvInstance_l.m_InitParam.m_abMyMacAddr[0] << 0;
        dwTemp |= EdrvInstance_l.m_InitParam.m_abMyMacAddr[1] << 8;
        dwTemp |= EdrvInstance_l.m_InitParam.m_abMyMacAddr[2] << 16;
        dwTemp |= EdrvInstance_l.m_InitParam.m_abMyMacAddr[3] << 24;
        dwTemp = 0;
        dwTemp |= EdrvInstance_l.m_InitParam.m_abMyMacAddr[4] << 0;
        dwTemp |= EdrvInstance_l.m_InitParam.m_abMyMacAddr[5] << 8;
     // dwTemp |= EDRV_REGDW_RAH_AV;

    }
    else
    //Read from EEPROM
    {
        EdrvIfeCheckEepromSize();

        for (iLoop = 0; iLoop < 6; iLoop += 2)
        {
            val = EdrvIfeReadEeprom(iLoop/2);
            EdrvInstance_l.m_InitParam.m_abMyMacAddr[iLoop] = (BYTE)val;
            EdrvInstance_l.m_InitParam.m_abMyMacAddr[iLoop+1] = (BYTE)(val >> 8);
            //printk("mac address : %x %x\n",(BYTE)val,(BYTE)(val >> 8));
        }
    }

    EdrvInstance_l.m_uiHeadRxDesc = 0;
    EdrvInstance_l.m_uiTailRxDesc = MAX_RFDS-1;
    rfdp = (struct rfd *)((EdrvInstance_l.m_pRfdVirtAdd) + (RFD_REQUIRED_SIZE * EdrvInstance_l.m_uiTailRxDesc));
    rfdp->m_uiCmdStat = CS_S;

    //Enable the receiver
    EdrvIfeIssueScbcmd(SC_RUC_START, EdrvInstance_l.m_RfdDmaAdd, 0);

    goto Exit;

ExitFail:
    EdrvRemoveOne(pPciDev_p);

Exit:
    printk("%s finished with %d\n", __FUNCTION__, iResult);
    return iResult;
}

//---------------------------------------------------------------------------
//
// Function: EdrvRemoveOne
//
// Description: shuts down one PCI device
//
// Parameters: pPciDev_p = pointer to corresponding PCI device structure
//
// Returns: (void)
//
// State:
//
//---------------------------------------------------------------------------
static void EdrvRemoveOne(struct pci_dev *pPciDev_p)
{
    if (EdrvInstance_l.m_pPciDev != pPciDev_p)
    { // trying to remove unknown device
        BUG_ON(EdrvInstance_l.m_pPciDev != pPciDev_p);
        goto Exit;
    }

    if (EdrvInstance_l.m_pIoAddr != NULL)
    {
        // disable interrupts
        iowrite8(SC_M >> 8, EdrvInstance_l.m_pIoAddr + SCBCMD + 1);

        // disable the receiver
        EdrvIfeIssueScbcmd(SC_RUC_ABORT, 0, 0);
    }

     // remove interrupt handler
    free_irq(pPciDev_p->irq, pPciDev_p);

    // free buffers

    if(EdrvInstance_l.m_pCbVirtAdd != NULL)
    {
        pci_free_consistent(pPciDev_p, CB_REQUIRED_SIZE * MAX_CBS, EdrvInstance_l.m_pCbVirtAdd, EdrvInstance_l.m_CbDmaHandle);
        EdrvInstance_l.m_pCbVirtAdd = NULL;
    }

    // rfd memory free in edrvremoveone - start
    if(EdrvInstance_l.m_pRfdVirtAdd != NULL)
    {
        pci_free_consistent(pPciDev_p, RFD_REQUIRED_SIZE * MAX_RFDS, EdrvInstance_l.m_pRfdVirtAdd, EdrvInstance_l.m_RfdDmaAdd);
        EdrvInstance_l.m_pRfdVirtAdd = NULL;
    }
    // rfd memory free in edrvremoveone - end

    if (EdrvInstance_l.m_pbTxBuf != NULL)
    {
        pci_free_consistent(pPciDev_p, EDRV_TX_BUFFER_SIZE,
                                 EdrvInstance_l.m_pbTxBuf, EdrvInstance_l.m_pTxBufDma);
        EdrvInstance_l.m_pbTxBuf = NULL;
    }

    // unmap controller's register space
    if (EdrvInstance_l.m_pIoAddr != NULL)
    {
        iounmap(EdrvInstance_l.m_pIoAddr);
        EdrvInstance_l.m_pIoAddr = NULL;
    }

    // disable the PCI device
    pci_disable_device(pPciDev_p);

    // release memory regions
    pci_release_regions(pPciDev_p);

    EdrvInstance_l.m_pPciDev = NULL;

Exit:
    return;
}
