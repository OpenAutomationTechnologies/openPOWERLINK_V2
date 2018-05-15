/**
********************************************************************************
\file   edrv-8255x.c

\brief  Implementation of Ethernet driver for Intel 8255x

This file contains the implementation of the Ethernet driver for
Intel 8255x 10/100 Fast Ethernet Controller

Based on
1. ife_gem.c: Intel 8255x 10/100 Ethernet Controller driver for Solaris
by Masayuki Murayama.
2. Intel 8255x 10/100 Mbps Ethernet Controller Family Open Source Software
Developer Manual.

\ingroup module_edrv
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2013, Kalycito Infotech Private Limited
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

/*------------------------------------------------------------------------------
ife_gem.c: Intel 8255x 10/100 Ethernet controller driver for Solaris

Copyright (c) 2003-2008 Masayuki Murayama. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation
and/or other materials provided with the distribution.

3. Neither the name of the author nor the names of its contributors may be
used to endorse or promote products derived from this software without
specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
DAMAGE.
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
// TracePoint support for realtime-debugging
#ifdef _DBG_TRACE_POINTS_
void target_signalTracePoint(UINT8 tracePointNumber_p);
#define TGT_DBG_SIGNAL_TRACE_POINT(p)   target_signalTracePoint(p)
#else
#define TGT_DBG_SIGNAL_TRACE_POINT(p)
#endif

//============================================================================//
//            P R I V A T E   D E F I N I T I O N S                           //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#ifndef EDRV_MAX_TX_BUFFERS
#define EDRV_MAX_TX_BUFFERS         256
#endif

#ifndef EDRV_MAX_TX_DESCS
#define EDRV_MAX_TX_DESCS           16
#define EDRV_TX_DESC_MASK           (EDRV_MAX_TX_DESCS - 1)
#endif

#ifndef EDRV_MAX_RX_BUFFERS
#define EDRV_MAX_RX_BUFFERS         256
#endif

#ifndef EDRV_MAX_RX_DESCS
#define EDRV_MAX_RX_DESCS           16
#define EDRV_RX_DESC_MASK           (EDRV_MAX_RX_DESCS - 1)
#endif

#define EDRV_MAX_FRAME_SIZE         0x0600

#define EDRV_TX_BUFFER_SIZE         (EDRV_MAX_TX_BUFFERS * EDRV_MAX_FRAME_SIZE) // n * (MTU + 14 + 4)

#define EDRV_RX_BUFFER_SIZE_SHIFT   11 // 2048 Byte
#define EDRV_RX_BUFFER_SIZE         (1 << EDRV_RX_BUFFER_SIZE_SHIFT)

#define DRV_NAME                    "plk"

// 8255x hash defines - start
/*
* Register offsets
*/
#define SCBSTAT                     0x00 /* W SCB status word */
#define SCBCMD                      0x02 /* W SCB command word */
#define GENPTR                      0x04 /* D SCB general pointer */
#define PORT                        0x08 /* D Port interface */
#define EECTRL                      0x0e /* B EEPROM control register */
#define EARLYRXINT                  0x18 /* B earlt rx interrupt register */
#define FCTRL                       0x1a /* B Flow control on/off register */

/* SCB Status Word */
#define SS_CX                       0x8000 /* cmd done w/ intr */
#define SS_FR                       0x4000 /* Rx done */
#define SS_CNA                      0x2000 /* CU not active / */
#define SS_RNR                      0x1000 /* RU not ready */

/* SCB Command Word */
#define SC_M                        0x0100 /* interrupt mask 1:mask 0: nomask */
#define SC_CUC                      0x00f0 /* CU command */
#define SC_CUC_SHIFT                4
#define SC_CUC_START                (0x1 << SC_CUC_SHIFT)
#define SC_CUC_LOADSDMP             (0x4 << SC_CUC_SHIFT)
#define SC_CUC_LOADBASE             (0x6 << SC_CUC_SHIFT)
#define SC_RUC                      0x0007 /* RU command */
#define SC_RUC_SHIFT                0
#define SC_RUC_START                (1 << SC_RUC_SHIFT)
#define SC_RUC_ABORT                (4 << SC_RUC_SHIFT)
#define SC_RUC_LOADHDS              (5 << SC_RUC_SHIFT)
#define SC_RUC_LOADBASE             (6 << SC_RUC_SHIFT)

/* Port register */
#define PORT_SOFTRESET              0x0 /* entire reset */

/* EEPROM control register */
#define EC_EEDO_SHIFT               3
#define EC_EEDO                     (1 << EC_EEDO_SHIFT) /* serial data-out from eeprom */
#define EC_EEDI_SHIFT               2
#define EC_EECS                     0x02 /* chip select 0:disable, 1:enable */
#define EC_EESK                     0x01 /* serial clock */

/* Shared memory operations */

/* OPCODES */
#define OP_NOP                      0
#define OP_ADDRSETUP                1 /* individual address setup */
#define OP_CONFIGURE                2 /* load the device with operating parameters */
#define OP_MULTICAST                3 /* setup one or more multicast address */
#define OP_TX                       4 /* Transmit a single frame */

#define CS_EL                       0x80000000
#define CS_S                        0x40000000 /* suspend */
#define CS_I                        0x20000000 /* generate interrupt */
#define CS_SF_                      0x00080000 /* 0:simplify mode, 1:flexible mode */
#define CS_OP_SHIFT                 16
#define CS_C                        0x00008000 /* completed */
#define CS_OK                       0x00002000 /* no error */

#define TCB_TBDNUM_SHIFT            24
#define TCB_TXTHR_SHIFT             16
#define TCB_EOF                     0x00008000 /* for backword compatibility */

/* Transmit buffer descriptor */
#define RFD_SIZE                    0x3fff0000 /* prepared buffer size */
#define RFD_SIZE_SHIFT              16

#define RFD_EOF                     0x00008000 /* end of frame */
#define RFD_F                       0x00004000
#define RFD_COUNT                   0x00003fff /* actual received byte count */

// 8255x hash defines - end

// custom hash defines - start
/*
* EEPROM I/O routines
*/
#define EEPROM_READ_CMD             6

// number of multicast addresses
#define MULTICAST_ADDR_NUM          4

// the number of bytes in a MAC address
#define MAC_ADDRESS_LEN             6

// number of bytes required for one Command Block
// to be incremented by 6 for every additional
// address that needs to be added to the
// multicast table of the controller
// to be kept 32 bit aligned
#define CB_REQUIRED_SIZE            56

// maximum number of Command Blocks
#define MAX_CBS                     16

// maximum size of array for storing TX buffer pointers
// for use during callbacks in ISR
#define MAX_HANDLER_ADDR            4

// number of bytes required for one RX descriptor block + RX buffer section for that descriptor block
#define RFD_REQUIRED_SIZE           1534
// number of bytes required for one RX buffer
#define RX_BUFFER_REQUIRED_SIZE     1518
// maximum number of RX descriptors
#define MAX_RFDS                    16
// command mode to multicastCmd()
#define MULTICAST_ADDR_ADD          0 /* to add multicast address */
#define MULTICAST_ADDR_REM          1 /* to remove multicast address */
#define DELAY_CLEAR_INT             5
#define DELAY_SYS_TX_CLK            10
// custom hash defines - end

#define TBD_ADDR_NULL               0xffffffff
#define TBD_EL                      0x00010000


#define EDRV_COUNT_SEND             TGT_DBG_SIGNAL_TRACE_POINT(2)
#define EDRV_COUNT_TIMEOUT          TGT_DBG_SIGNAL_TRACE_POINT(3)
#define EDRV_COUNT_PCI_ERR          TGT_DBG_SIGNAL_TRACE_POINT(4)
#define EDRV_COUNT_TX               TGT_DBG_SIGNAL_TRACE_POINT(5)
#define EDRV_COUNT_RX               TGT_DBG_SIGNAL_TRACE_POINT(6)
#define EDRV_COUNT_LATECOLLISION    TGT_DBG_SIGNAL_TRACE_POINT(10)
#define EDRV_COUNT_TX_COL_RL        TGT_DBG_SIGNAL_TRACE_POINT(11)
#define EDRV_COUNT_TX_FUN           TGT_DBG_SIGNAL_TRACE_POINT(12)
#define EDRV_COUNT_TX_TEST          TGT_DBG_SIGNAL_TRACE_POINT(13)
#define EDRV_COUNT_RX_ERR_CRC       TGT_DBG_SIGNAL_TRACE_POINT(14)
#define EDRV_COUNT_RX_ERR_MULT      TGT_DBG_SIGNAL_TRACE_POINT(15)
#define EDRV_COUNT_RX_ERR_SEQ       TGT_DBG_SIGNAL_TRACE_POINT(16)
#define EDRV_COUNT_RX_ERR_OTHER     TGT_DBG_SIGNAL_TRACE_POINT(17)
#define EDRV_COUNT_RX_ORUN          TGT_DBG_SIGNAL_TRACE_POINT(18)

//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------
/**
\brief Structure describing a transmit buffer descriptor

This structure describes a transmit buffer descriptor used on the
Intel 8255x Ethernet controller.
*/
typedef struct
{
    volatile UINT tbdAddr;                                  ///< Transmit buffer address
    volatile UINT tbdSize;                                  ///< Transmit buffer size
} tTxDescCmdBlock;

/**
\brief Structure describing a command block

This structure describes a command block used on the Intel 8255x
Ethernet controller.
*/
typedef struct
{
    volatile UINT statusCommand;                            ///< Status / command register
    volatile UINT linkAddr;                                 ///< Link address (offset to the next command block)
} tCommandBlock;

/**
\brief Structure describing a receive descriptor command block

This structure describes a receive descriptor command block used on the
Intel 8255x Ethernet controller.
*/
typedef struct
{
    volatile UINT statusCommand;                            ///< Status / command register
    volatile UINT linkAddr;                                 ///< Link address (offset to the next command block)
    volatile UINT reserved : 32;                            ///< Reserved
    volatile UINT size;                                     ///< Data buffer size
} tRxDescCmdBlock;

/**
\brief Structure describing a transmit command block

This structure describes a transmit command block used on the Intel 8255x
Ethernet controller.
*/
typedef struct
{
    volatile UINT   statusCommand;                          ///< Status / command register
    volatile UINT   linkAddr;                               ///< Link address (offset to the next command block)
    volatile UINT   tcbTbdPtr;                              ///< Transmit buffer descriptor array address
    volatile UINT   tcbCtrl;                                ///< Transmit command buffer control register (number, threshold, EOF, block byte count)
} tTxCmdBlock;

/**
\brief Structure describing a generic command block

This structure describes a generic command block used on the Intel 8255x
Ethernet controller.
*/
typedef struct
{
    volatile UINT   statusCommand;                          ///< Status / command register
    volatile UINT   linkAddr;                               ///< Link address (offset to the next command block)
    volatile UINT   value[6];                               ///< Values
} tCommandBlockGen;

/**
\brief Structure describing an instance of the Edrv

This structure describes an instance of the Ethernet driver.
*/
typedef struct
{
    tEdrvInitParam       initParam;                              ///< Init parameters
    struct pci_dev*      pPciDev;                                ///< Pointer to the PCI device structure
    void*                pIoAddr;                                ///< Pointer to the register space of the Ethernet controller

    UINT                 eepromAddrBits;                         ///< Used to store EEPROM address bits
    UINT16               multicastAddrByteCnt;                   ///< Number of Bytes used for multicast addresses

    void*                pTxBuf;                                 ///< Pointer to the TX buffer
    dma_addr_t           pTxBufDma;                              ///< Pointer to the DMA of the TX buffer
    BOOL                 afTxBufUsed[EDRV_MAX_TX_BUFFERS];       ///< Array describing whether a TX buffer is used

    UINT                 headTxDesc;                             ///< Index of the head of the TX descriptor buffer
    UINT                 tailTxDesc;                             ///< Index of the tail of the TX descriptor buffer
    UINT                 headRxDesc;                             ///< Index of the head of the RX descriptor buffer
    UINT                 tailRxDesc;                             ///< Index of the tail of the RX descriptor buffer

    void*                pCbVirtAdd;                             ///< Virtual address of the command block
    void*                pRfdVirtAdd;                            ///< Virtual address of the receive descriptors

    dma_addr_t           cbDmaHandle;                            ///< Command block DMA handle
    dma_addr_t           rfdDmaAdd;                              ///< Receive descriptor DMA handle

    volatile ULONG       aCbVirtAddrBuf[MAX_CBS];                ///< Array to store virtual address of a TX buffer
    volatile dma_addr_t  aCbDmaAddrBuf[MAX_CBS];                 ///< Array to store DMA mapped address of a TX buffer
} tEdrvInstance;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static irqreturn_t edrvIrqHandler(int irqNum_p, void* pDevInstData_p);
static tOplkError individualAddressCmd(UINT opcode_p, UINT count_p);
static tOplkError configureCmd(UINT opcode_p, UINT count_p);
static tOplkError multicastCmd(UINT opcode_p, UINT count_p, const UINT8* pMacAddr_p, UINT mode_p);
static tOplkError transmitCmd(UINT opcode_p, UINT count_p);

static void eepromDelay(void);
static void checkEepromSize(void);
static UINT16 readEeprom(UINT addr_p);
static tOplkError cmdDescWrite(UINT opcode_p, UINT count_p);
static tOplkError rxDescWrite(int count_p);
static BOOL issueScbcmd(UINT16 cmd_p, UINT arg_p, UINT opcode_p);

static int initOnePciDev(struct pci_dev* pPciDev_p, const struct pci_device_id* pId_p);
static void removeOnePciDev(struct pci_dev* pPciDev_p);

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
// buffers and buffer descriptors and pointers
static struct pci_device_id aEdrvPciTbl_l[] =
{
    {0x8086, 0x1091, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0},
    {0x8086, 0x1092, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0},
    {0x8086, 0x1093, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0},
    {0x8086, 0x1094, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0},
    {0x8086, 0x1095, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0},
    {0x8086, 0x1209, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0}, // Intel Corporation 8255xER/82551IT (APC-620)
    {0x8086, 0x1229, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0}, // Intel Corporation 82557/8/9/0/1 Ethernet Pro 100
    {0, }
};
MODULE_DEVICE_TABLE (pci, aEdrvPciTbl_l);

static tEdrvInstance edrvInstance_l;
static tBufAlloc* pBufAlloc_l = NULL;

static struct pci_driver edrvDriver_l =
{
    .name = DRV_NAME,
    .id_table = aEdrvPciTbl_l,
    .probe = initOnePciDev,
    .remove = removeOnePciDev,
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
    edrvDriver_l.name = DRV_NAME,
    edrvDriver_l.id_table = aEdrvPciTbl_l,
    edrvDriver_l.probe = initOnePciDev,
    edrvDriver_l.remove = removeOnePciDev,

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
    tOplkError          ret = kErrorOk;
    tCommandBlock*      pCb;
    UINT8*              pCnt;
    int                 cbCnt = 0;
    static BOOL         fIsFirstEntry = FALSE;

    // Check parameter validity
    ASSERT(pMacAddr_p != NULL);

    if (unlikely(fIsFirstEntry == FALSE))
    {
        fIsFirstEntry = TRUE;
        //pointer to the Command Block specified by the count value
        pCb = (tCommandBlock*)(edrvInstance_l.pCbVirtAdd + (CB_REQUIRED_SIZE * cbCnt));

        //set the byte count (number of mac addresses * 6 bytes per mac address)
        //in the corresponding section of the descriptor to zero
        edrvInstance_l.multicastAddrByteCnt = 0;
        pCnt = (UINT8*)&pCb[1];
        *(UINT16*)pCnt = edrvInstance_l.multicastAddrByteCnt;
    }

    ret = multicastCmd(OP_MULTICAST, cbCnt, pMacAddr_p, MULTICAST_ADDR_ADD);

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
    tOplkError ret = kErrorOk;

    // Check parameter validity
    ASSERT(pMacAddr_p != NULL);

    ret = multicastCmd(OP_MULTICAST, 0, pMacAddr_p, MULTICAST_ADDR_REM);

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
    tOplkError          ret = kErrorOk;

    // Check parameter validity
    ASSERT(pBuffer_p != NULL);

    if (((edrvInstance_l.tailTxDesc + 1) % MAX_CBS) == edrvInstance_l.headTxDesc)
    {
        ret = kErrorEdrvNoFreeTxDesc;
        goto Exit;
    }

    // array to store virtual address of pTxBuffer
    edrvInstance_l.aCbVirtAddrBuf[edrvInstance_l.tailTxDesc] = (ULONG)pBuffer_p;

    // array to store dma mapped address of data pointed by pBuffer_p->pBuffer
    edrvInstance_l.aCbDmaAddrBuf[edrvInstance_l.tailTxDesc] = pci_map_single(edrvInstance_l.pPciDev, pBuffer_p->pBuffer,
                                                pBuffer_p->txFrameSize, PCI_DMA_TODEVICE);

    // fill the TXCB descriptor with the relevant data

    // increment Tx descriptor queue tail pointer
    edrvInstance_l.tailTxDesc = ((edrvInstance_l.tailTxDesc + 1) % MAX_CBS);

    if (edrvInstance_l.tailTxDesc == 0)
    {
        cmdDescWrite(OP_TX, (MAX_CBS - 1));
        issueScbcmd(SC_CUC_START, edrvInstance_l.cbDmaHandle + (CB_REQUIRED_SIZE * (MAX_CBS - 1)), OP_TX);
    }
    else
    {
        cmdDescWrite(OP_TX, (edrvInstance_l.tailTxDesc - 1));
        issueScbcmd(SC_CUC_START, edrvInstance_l.cbDmaHandle + (CB_REQUIRED_SIZE * (edrvInstance_l.tailTxDesc - 1)), OP_TX);
    }

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
\brief  Ethernet driver interrupt handler

This function is the interrupt service routine for the Ethernet driver.

\param[in]      irqNum_p            IRQ number
\param[in,out]  pDevInstData_p      Pointer to private data provided by request_irq

\return The function returns an IRQ handled code.
*/
//------------------------------------------------------------------------------
static irqreturn_t edrvIrqHandler(int irqNum_p, void* pDevInstData_p)
{
    irqreturn_t             handled;
    UINT16                  state;
    tCommandBlock*          pCmdBlock;
    UINT                    statusCommand;
    tRxDescCmdBlock*        pRxDescCmdBlock;
    tEdrvTxBuffer*          pTxBuffer = NULL;
    dma_addr_t              txDmaAddr;

    UNUSED_PARAMETER(irqNum_p);
    UNUSED_PARAMETER(pDevInstData_p);

    handled = IRQ_HANDLED;
    state = ioread16(edrvInstance_l.pIoAddr + SCBSTAT);

    if (((state & 0xFF00) == 0x0000) || ((state & 0xFF00) == 0xFF00))
    {
        handled = IRQ_NONE;
        goto Exit;
    }

    /* clear interrupt */
    iowrite16((state & 0xDF00), edrvInstance_l.pIoAddr + SCBSTAT);

    // open if(TX/RX interrupt)
    if ((state & SS_CX) | (state & (SS_FR | SS_RNR)))
    {
        if (edrvInstance_l.pTxBuf == NULL)
        {
            printk("%s Tx buffers currently not allocated\n", __func__);
            goto Exit;
        }

        if (state & SS_RNR)
        {
            printk("RU not ready\n");
        }

        do
        {
            // Process receive descriptors
            pRxDescCmdBlock = (tRxDescCmdBlock*)((UINT8*)edrvInstance_l.pRfdVirtAdd + (RFD_REQUIRED_SIZE * edrvInstance_l.headRxDesc));

            while (pRxDescCmdBlock->statusCommand & CS_C)
            { // Rx frame available
                tEdrvRxBuffer        rxBuffer;
                tEdrvReleaseRxBuffer retReleaseRxBuffer;

                if (((pRxDescCmdBlock->size & RFD_EOF) != 0) && ((pRxDescCmdBlock->size & RFD_F) != 0))
                { // Descriptor is valid
                    // Packet is OK

                    rxBuffer.bufferInFrame = kEdrvBufferLastInFrame;

                    // Get length of received packet
                    // In the default configuration for 82559 CRC is not transferred into host memory
                    // so we can use the RX byte count from the RFD as it is
                    rxBuffer.rxFrameSize = pRxDescCmdBlock->size & RFD_COUNT;

                    rxBuffer.pBuffer = (((UINT8*)edrvInstance_l.pRfdVirtAdd +
                                        RFD_REQUIRED_SIZE * edrvInstance_l.headRxDesc) +
                                        sizeof(tRxDescCmdBlock));

                    // Call Rx handler of Data link layer
                    retReleaseRxBuffer = edrvInstance_l.initParam.pfnRxHandler(&rxBuffer);
                }// closing Descriptor is valid

                // clean the status bits of the currently handled descriptor
                // so that it is available for use the next time
                rxDescWrite(edrvInstance_l.headRxDesc);
                edrvInstance_l.headRxDesc = (edrvInstance_l.headRxDesc + 1) % MAX_RFDS;
                pRxDescCmdBlock = (tRxDescCmdBlock*)((UINT8*)edrvInstance_l.pRfdVirtAdd + (RFD_REQUIRED_SIZE * edrvInstance_l.tailRxDesc));
                pRxDescCmdBlock->statusCommand = 0x00000000;

                if (edrvInstance_l.headRxDesc == 0)
                {
                    edrvInstance_l.tailRxDesc = MAX_RFDS - 1;
                }
                else
                {
                    edrvInstance_l.tailRxDesc = edrvInstance_l.headRxDesc - 1;
                }
                pRxDescCmdBlock = (tRxDescCmdBlock*)((UINT8*)edrvInstance_l.pRfdVirtAdd + (RFD_REQUIRED_SIZE * edrvInstance_l.tailRxDesc));
                pRxDescCmdBlock->statusCommand = CS_S;
                pRxDescCmdBlock = (tRxDescCmdBlock*)((UINT8*)edrvInstance_l.pRfdVirtAdd + (RFD_REQUIRED_SIZE * edrvInstance_l.headRxDesc));
            } // closing RX while loop

            pCmdBlock = (tCommandBlock*)((UINT8*)edrvInstance_l.pCbVirtAdd + (CB_REQUIRED_SIZE * edrvInstance_l.headTxDesc));


            statusCommand = pCmdBlock->statusCommand;
            if ((pCmdBlock->statusCommand & CS_C) &&
                (pCmdBlock->statusCommand & CS_SF_) &&
                (CS_SF_) &&
                (pCmdBlock->statusCommand & (OP_TX << CS_OP_SHIFT)))
            {
                // clear the status bits of the current CB
                pCmdBlock->statusCommand &= 0XFFFF0000;

                // retrieve the address of the pTxBuffer pointer
                // that was received in edrv_sendTxBuffer
                pTxBuffer = (tEdrvTxBuffer*)edrvInstance_l.aCbVirtAddrBuf[edrvInstance_l.headTxDesc];

                txDmaAddr = edrvInstance_l.aCbDmaAddrBuf[edrvInstance_l.headTxDesc];

                edrvInstance_l.aCbVirtAddrBuf[edrvInstance_l.headTxDesc] = 0;
                edrvInstance_l.aCbDmaAddrBuf[edrvInstance_l.headTxDesc] = 0;

                // Increment Tx descriptor queue head pointer
                edrvInstance_l.headTxDesc = ((edrvInstance_l.headTxDesc + 1) % MAX_CBS);

                if ((txDmaAddr != 0) && (pTxBuffer != NULL))
                {
                    // retrieve the address of the DMA handle to
                    // pTxBuffer_p->pBuffer pointer
                    // that was received in edrv_sendTxBuffer
                    // so that the mapping can be correspondingly unmapped
                    pci_unmap_single(edrvInstance_l.pPciDev,
                                     txDmaAddr,
                                     pTxBuffer->txFrameSize, PCI_DMA_TODEVICE);
                }

                if (pTxBuffer != NULL)
                {
                    // Call Tx handler of Data link layer
                    if (pTxBuffer->pfnTxHandler != NULL)
                    {
                        pTxBuffer->pfnTxHandler(pTxBuffer);
                        pTxBuffer=NULL;
                    }
                }
                else
                {
                        //Currently not handled
                }
            } //Closing if C bit set in the CB
            else
            {
                break;
            }
        } while (edrvInstance_l.headTxDesc != edrvInstance_l.tailTxDesc);

    } // close if(TX/RX interrupt)

Exit:
    return handled;
}

//------------------------------------------------------------------------------
/**
\brief  Issue individual address command

This function issues an individual address command to insert the MAC address.

\param[in]      opcode_p            Opcode to be filled in the command block
\param[in]      count_p             Count value indicating which command block is to be filled

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError individualAddressCmd(UINT opcode_p, UINT count_p)
{
    tOplkError          ret = kErrorOk;
    tCommandBlockGen*   pCmdBlock;

    //pointer to the Command Block specified by the count value
    pCmdBlock = (tCommandBlockGen*)((UINT8*)edrvInstance_l.pCbVirtAdd + (CB_REQUIRED_SIZE * count_p));

    //fill the individual address command to be executed
    pCmdBlock->statusCommand = CS_EL | (opcode_p << CS_OP_SHIFT); //0x80010000

    //The MAC address bytes used below have been read from the EEPROM

    //MAC ADDRESS (BYTE4<<24) | (BYTE3<<16) | (BYTE2<<8) | (BYTE1<<0)
    pCmdBlock->value[0] = edrvInstance_l.initParam.aMacAddr[0] |
                          edrvInstance_l.initParam.aMacAddr[1] << 8 |
                          edrvInstance_l.initParam.aMacAddr[2] << 16 |
                          edrvInstance_l.initParam.aMacAddr[3] << 24;

    //MAC ADDRESS (BYTE6<<8) | (BYTE5<<0)
    pCmdBlock->value[1] = edrvInstance_l.initParam.aMacAddr[4] |
                          edrvInstance_l.initParam.aMacAddr[5] << 8;

    //start command issued to execute the individual address setup
    issueScbcmd(SC_CUC_START, edrvInstance_l.cbDmaHandle, OP_ADDRSETUP);

    //wait for command to complete successfully
    while ((pCmdBlock->statusCommand & CS_C) == 0);

    if (pCmdBlock->statusCommand & CS_OK)
    {
        ret = kErrorOk;
    }
    else
    {
        ret = kErrorEdrvInit;
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Issue configure command

This function issues a configure command to the Ethernet controller.

\param[in]      opcode_p            Opcode to be filled in the command block
\param[in]      count_p             Count value indicating which command block is to be filled

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError configureCmd(UINT opcode_p, UINT count_p)
{
    tOplkError          ret = kErrorOk;
    tCommandBlockGen*   pCmdBlock;

    //pointer to the Command Block specified by the count_p value
    pCmdBlock = (tCommandBlockGen*)((UINT8*)edrvInstance_l.pCbVirtAdd + (CB_REQUIRED_SIZE * count_p));

    //fill the configure command to be executed
    pCmdBlock->statusCommand = CS_EL | (opcode_p << CS_OP_SHIFT); //0x80020000

    //(Byte3<<24) | (Byte2<<16) | (Byte1<<8) | (Byte0<<0)
    //Byte0 - Byte Count of 16 bytes used in this configuration
    //Byte1 - Default Value
    //Byte2 - Default Value
    //Byte3 - Default Value
    pCmdBlock->value[0] = (0X00 << 24) | (0X00 << 16) | (0X08 << 8) | (0X10 << 0); //0x00000810

    //Byte4 - Default Value
    //Byte5 - Default Value
    //Byte6 - Default Value for 82559
    //Byte7 - Default Value for 82559
    //(Byte7<<24) | (Byte6<<16) | (Byte5<<8) | (Byte4<<0)
    pCmdBlock->value[1] = (0X00 << 24) | (0X30 << 16) | (0X00 << 8) | (0X00 << 0); //0x00300000

    //Byte8 - Bit0 set for 82559, Bit0 cleared to enable link operation, Rest of the
    // bits are default values for 82559
    //Byte9 - Default value for 82559
    //Byte10 - Loopback disabled, default preamble length, source address insertion
    // disabled, other bits default for 82559
    //Byte11 - Default value for 82559
    //(Byte11<<24) | (Byte10<<16) | (Byte9<<8) | (Byte8<<0)
    pCmdBlock->value[2] = (0X00 << 24) | (0X2E << 16) | (0X00 << 8) | (0X01 << 0); //0x002E0001

    //Byte12 - Interframe spacing - default value, other bits default value for 82559
    //Byte13 - Default Value for 82559
    //Byte14 - Default Value for 82559
    //Byte15 - Bit0 cleared to disable promiscuous mode, other bits default value for 82559
    //(Byte15<<24) | (Byte14<<16) | (Byte13<<8) | (Byte12<<0)
    pCmdBlock->value[3] = (0XC8 << 24) | (0XF2 << 16) | (0X00 << 8) | (0X61 << 0); //0xC8F20061

    //NOTE: the following configuration bytes not yet configured
    //Byte16 - Default Value
    //Byte17 - Default Value
    //Byte18 - Default Value
    //Byte19 - Default Value
    //(Byte19<<24) | (Byte18<<16) | (Byte17<<8) | (Byte16<<0)
    //pCmdBlock->value[4] = (0X80<<24) | (0XF2<<16) | (0X40<<8) | (0X00<<0); //

    //Byte20 - Default Value
    //Byte21 - Default Value
    //Byte22 - Zero Padding
    //Byte23 - Zero Padding
    //(Byte23<<24) | (Byte22<<16) | (Byte21<<8) | (Byte20<<0)
    //pCmdBlock->value[5] = (0X00<<24) | (0X00<<16) | (0X05<<8) | (0X3F<<0); //

    //start command issued to configure the device as per the configuration byte values
    issueScbcmd(SC_CUC_START, edrvInstance_l.cbDmaHandle, OP_CONFIGURE);

    //wait for command to complete successfully
    while ((pCmdBlock->statusCommand & CS_C) == 0);

    if (pCmdBlock->statusCommand & CS_OK)
    {
        ret = kErrorOk;
    }
    else
    {
        ret = kErrorEdrvInit;
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Issue multicast command

This function issues a multicast command to the Ethernet controller.

\param[in]      opcode_p            Opcode to be filled in the command block
\param[in]      count_p             Count value indicating which command block is to be filled
\param[in]      pMacAddr_p          Pointer to the multicast MAC address to be filled
\param[in]      mode_p              Index variable used for the position of the multicast MAC address
                                    in the command block

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError multicastCmd(UINT opcode_p, UINT count_p, const UINT8* pMacAddr_p, UINT mode_p)
{
    tOplkError          ret = kErrorOk;
    tCommandBlock*      pCmdBlock;
    UINT8*              pByte;
    UINT16*             pByteCount;
    UINT16              multicastAddrCnt;
    UINT16              multicastAddrLoop;
    UINT                memCmpref = 0;

    //pointer to the Command Block specified by the count_p value
    pCmdBlock = (tCommandBlock*)((UINT8*)edrvInstance_l.pCbVirtAdd + (CB_REQUIRED_SIZE * count_p));

    //fill the multicast command to be executed
    pCmdBlock->statusCommand = CS_EL | (opcode_p << CS_OP_SHIFT); //0x80030000

    pByte = (UINT8*)&pCmdBlock[1];
    //point the address where the byte count_p for multicast entries to be stored
    pByteCount = (UINT16*)pByte;
    multicastAddrCnt = edrvInstance_l.multicastAddrByteCnt;
    //move the pointer to first multicast entry
    pByte += (sizeof(UINT16));

    if (mode_p == MULTICAST_ADDR_ADD)
    {
        //fill the byte count (number of mac addresses * 6 bytes per mac address)
        //in the corresponding section of the descriptor
        //according to the u32Index value
        edrvInstance_l.multicastAddrByteCnt += MAC_ADDRESS_LEN;
        *pByteCount = edrvInstance_l.multicastAddrByteCnt;

        //fill the multicast entry in the corresponding section of
        //the descriptor according to the u32Index value
        pByte += multicastAddrCnt;
        OPLK_MEMCPY(pByte, pMacAddr_p, MAC_ADDRESS_LEN);
    }
    else if (mode_p == MULTICAST_ADDR_REM)
    {
        //find the byte count (number of mac addresses * 6 bytes per mac address)
        //in the corresponding section of the descriptor

        // search for the mac address to be removed from multicast address
        for (multicastAddrLoop = 0; multicastAddrLoop < multicastAddrCnt ; multicastAddrLoop += MAC_ADDRESS_LEN)
        {
            if (OPLK_MEMCMP(pByte, pMacAddr_p, MAC_ADDRESS_LEN) == 0)
            {
                // entry found reduce the count and remove that entry from the descriptor
                edrvInstance_l.multicastAddrByteCnt -= MAC_ADDRESS_LEN;
                *pByteCount = edrvInstance_l.multicastAddrByteCnt;

                if ((multicastAddrCnt - (multicastAddrLoop + MAC_ADDRESS_LEN)) != 0)
                    OPLK_MEMCPY(pByte, pByte + MAC_ADDRESS_LEN, (multicastAddrCnt - (multicastAddrLoop + MAC_ADDRESS_LEN)));

                memCmpref = 1;
                break;
            }
            else
            {
                memCmpref = 0;
            }
            pByte += MAC_ADDRESS_LEN;
        }

        if (memCmpref == 0)
        {
            ret = kErrorOk;
            goto Exit;
        }
    }
    else
    {
        printk("%s(): Unknown mode:%d \n", __func__, mode_p);
        goto Exit;
    }

    //start command issued to set the multicast address
    issueScbcmd(SC_CUC_START, edrvInstance_l.cbDmaHandle, opcode_p);

    //wait for command to complete successfully
    while ((pCmdBlock->statusCommand & CS_C) == 0)
        ;

    if (pCmdBlock->statusCommand & CS_OK)
    {
        ret = kErrorOk;
    }
    else
    {
        ret = kErrorEdrvInit;
    }

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Issue transmit command

This function issues a transmit command to the Ethernet controller

\param[in]      opcode_p            Opcode to be filled in the command block
\param[in]      count_p             Count value indicating which command block is to be filled

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError transmitCmd(UINT opcode_p, UINT count_p)
{
    tOplkError              ret = kErrorOk;
    tTxCmdBlock*            pTxCmdBlock = NULL;
    tTxDescCmdBlock*        pTxDescCmdBlock = NULL;
    UINT                    cbpDma = 0; //FIXME: Give me a meaningful name, now!
    tEdrvTxBuffer*          pTxBuffer = NULL;
    UINT                    temp;

    pTxBuffer = (tEdrvTxBuffer*)edrvInstance_l.aCbVirtAddrBuf[count_p];
    //pointer to the Command Block specified by the count_p value
    pTxCmdBlock = (tTxCmdBlock*)((UINT8*)edrvInstance_l.pCbVirtAdd + (CB_REQUIRED_SIZE * count_p));
    pTxDescCmdBlock = (tTxDescCmdBlock*)&pTxCmdBlock[1];

    //physical address corresponding to the virtual address of the Command Block specified above
    cbpDma = (edrvInstance_l.cbDmaHandle + (CB_REQUIRED_SIZE * count_p));


    //fill the length of the buffer to be transmitted and indicate that it is the end of the frame
    pTxDescCmdBlock->tbdSize = TBD_EL | (pTxBuffer->txFrameSize);
    //fill the physical address of the buffer to be transmitted
    pTxDescCmdBlock->tbdAddr = edrvInstance_l.aCbDmaAddrBuf[count_p];
    //fill the status bits and the end of frame indication bit in the TCB Control section
    pTxCmdBlock->tcbCtrl = (0X01 << TCB_TBDNUM_SHIFT) |
                           (0X01 << TCB_TXTHR_SHIFT) |
                           TCB_EOF; //0x01018000

    //fill in the physical address of the buffer descriptor in the TX descriptor's TCB address section
    temp = cbpDma + sizeof(tTxCmdBlock);
    pTxCmdBlock->tcbTbdPtr = temp;

    /* make command block */
    temp = opcode_p << CS_OP_SHIFT;

    //set the interrupt bit in the command
    temp |= CS_I;
    //configure for flexible mode
    temp |= CS_SF_;

    // set the suspend bit in the command
    temp |= CS_S;

    pTxCmdBlock->statusCommand = temp;

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  EEPROM delay

This function inserts a delay before successive operations to the EEPROM.
*/
//------------------------------------------------------------------------------
static void eepromDelay(void)
{
    //jba: Is this always reliable on different processors?
    /* 2 reads required for 66MHz operation */
    ioread8(edrvInstance_l.pIoAddr + EECTRL);
    ioread8(edrvInstance_l.pIoAddr + EECTRL);
}

//------------------------------------------------------------------------------
/**
\brief  Check EEPROM size

This function verifies the EEPROM's size.
*/
//------------------------------------------------------------------------------
static void checkEepromSize(void)
{
    int     loopCount;
    UINT8   chipselect;
    UINT8   di; //FIXME: Give me a meaningful name, now!

    /* enable eeprom interface register */
    chipselect = EC_EECS;
    iowrite8(chipselect, edrvInstance_l.pIoAddr + EECTRL);

    /* output eeprom command */
    for (loopCount = 4; loopCount >= 0; loopCount--)
    {
        di = ((EEPROM_READ_CMD >> loopCount) & 1) << EC_EEDI_SHIFT;

        iowrite8(chipselect | di, edrvInstance_l.pIoAddr + EECTRL);
        iowrite8(chipselect | di | EC_EESK, edrvInstance_l.pIoAddr + EECTRL);
        eepromDelay();

        iowrite8(chipselect | di, edrvInstance_l.pIoAddr + EECTRL);
        eepromDelay();
    }

    /* How many address bits required until eeprom responds with 0 */
    loopCount = 0;
    do
    {
        iowrite8(chipselect, edrvInstance_l.pIoAddr + EECTRL);
        iowrite8(chipselect | EC_EESK, edrvInstance_l.pIoAddr + EECTRL);
        eepromDelay();

        iowrite8(chipselect, edrvInstance_l.pIoAddr + EECTRL);
        eepromDelay();

        loopCount++;
    } while ((ioread8(edrvInstance_l.pIoAddr + EECTRL) & EC_EEDO) && loopCount < 8);
    /* save the result */

    edrvInstance_l.eepromAddrBits = loopCount;

    /* read 16bits of data to terminate the sequence */
    for (loopCount = 16; loopCount > 0; loopCount--)
    {
        iowrite8(chipselect | EC_EESK, edrvInstance_l.pIoAddr + EECTRL);
        eepromDelay();

        iowrite8(chipselect, edrvInstance_l.pIoAddr + EECTRL);
        eepromDelay();
    }

    /* De-activate the EEPROM */
    iowrite8(0, edrvInstance_l.pIoAddr + EECTRL);
}

//------------------------------------------------------------------------------
/**
\brief  Get MAC address from EEPROM

This function obtains the MAC address from the EEPROM.

\param[in]      addr_p              Address from which the MAC address has to be read

\return The function returns the MAC address.
*/
//------------------------------------------------------------------------------
static UINT16 readEeprom(UINT addr_p)
{
    UINT16  ret;
    int     loopCount;
    UINT    cmd;
    UINT8   chipselect;
    UINT8   di; //FIXME: Give me a meaningful name, now!

    if (addr_p >= (1 << edrvInstance_l.eepromAddrBits))
    {
        return 0;
    }

    /* make command bits */
    cmd = (EEPROM_READ_CMD << edrvInstance_l.eepromAddrBits) | addr_p;

    /* enable eeprom interface register */
    chipselect = EC_EECS;

    iowrite8(chipselect, edrvInstance_l.pIoAddr + EECTRL);
    eepromDelay();

    /* output eeprom command */
    for (loopCount = 4 + edrvInstance_l.eepromAddrBits; loopCount >= 0; loopCount--)
    {
        di = ((cmd >> loopCount) & 1) << EC_EEDI_SHIFT;

        iowrite8(chipselect | di, edrvInstance_l.pIoAddr + EECTRL);
        iowrite8(chipselect | di | EC_EESK, edrvInstance_l.pIoAddr + EECTRL);
        eepromDelay();

        iowrite8(chipselect | di, edrvInstance_l.pIoAddr + EECTRL);
        eepromDelay();
    }

    /* get returned value */
    ret = 0;
    for (loopCount = 16; loopCount > 0; loopCount--)
    {
        /* get 1 bit */
        iowrite8(chipselect | EC_EESK, edrvInstance_l.pIoAddr + EECTRL);
        eepromDelay();

        ret = (ret << 1) | ((ioread8(edrvInstance_l.pIoAddr + EECTRL) >> EC_EEDO_SHIFT) & 1);

        iowrite8(chipselect, edrvInstance_l.pIoAddr + EECTRL);
        eepromDelay();
    }

    /* Terminate the EEPROM access. */
    iowrite8(0, edrvInstance_l.pIoAddr + EECTRL);
    eepromDelay();

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Write Tx descriptors

This function issues a command to write to the Tx descriptors.

\param[in]      opcode_p            Opcode value indicating the command to be executed
\param[in]      count_p             Count value indicating which the descriptor number

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError cmdDescWrite(UINT opcode_p, UINT count_p)
{
    tOplkError ret = kErrorOk;

    switch (opcode_p)
    {
        case OP_NOP: /* 0 */
            break;

        case OP_ADDRSETUP: /* 1 */
            ret = individualAddressCmd(opcode_p, count_p);
            break;

        case OP_CONFIGURE: /* 2 */
            ret = configureCmd(opcode_p, count_p);
            break;

        case OP_MULTICAST: /* 3 */
            break;

        case OP_TX: /* 4 */
            ret = transmitCmd(opcode_p, count_p);
            break;

        default:
            break;
    }
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Write Rx descriptors

This function issues a command writing to the Rx descriptors.

\param[in]      count_p             Count value indicating which Rx descriptor's
                                    status bits are cleared

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError rxDescWrite(int count_p)
{
    tRxDescCmdBlock*        pRxDescCmdBlock;
    tOplkError              ret = kErrorOk;

    //select the virtual address of the RX descriptor whose status bits are to be cleared
    pRxDescCmdBlock = (tRxDescCmdBlock*)((UINT8*)edrvInstance_l.pRfdVirtAdd + (RFD_REQUIRED_SIZE * count_p));

    //clear the status bits of the selected RX descriptor
    pRxDescCmdBlock->statusCommand = 0x00000000;

    //clear the status bits in the Size field of the RX descriptor
    pRxDescCmdBlock->size = (((RFD_REQUIRED_SIZE - sizeof(tRxDescCmdBlock)))
                             << RFD_SIZE_SHIFT) & RFD_SIZE;

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Issues a SCB command

This function issues a SCB command for the CU or RU as required.

\param[in]      cmd_p               Command to be issued to the CU or RU
\param[in]      arg_p               Address to be written to the GENPTR register
\param[in]      opcode_p            Opcode used while issuing the command

\return The function returns TRUE.
*/
//------------------------------------------------------------------------------
static BOOL issueScbcmd(UINT16 cmd_p, UINT arg_p, UINT opcode_p)
{
    UINT16          state = 0;
    static int      curStatusFirstTime = 0;
    static UINT16   prevCmd = 0;

    // this code section used for waiting till the previous cmd has been accepted
    while (ioread8((UINT8*)edrvInstance_l.pIoAddr + SCBCMD) & (SC_CUC | SC_RUC))
        ;

    if (opcode_p == OP_TX)
    {
        state = ioread16((UINT8*)edrvInstance_l.pIoAddr + SCBSTAT);
        while ((state & SS_CNA) == 0 && (curStatusFirstTime == 1))
        {
            state = ioread16((UINT8*)edrvInstance_l.pIoAddr + SCBSTAT);
        }

        if (curStatusFirstTime < 1)
        {
            curStatusFirstTime++;
        }
    }

    switch (cmd_p)
    {
        case SC_CUC_START:
        case SC_CUC_LOADSDMP:
        case SC_CUC_LOADBASE:
        case SC_RUC_START:
        case SC_RUC_LOADHDS:
        case SC_RUC_LOADBASE:
            iowrite32(arg_p, (UINT8*)edrvInstance_l.pIoAddr + GENPTR);
            break;
    }
    iowrite16(SS_CNA, (UINT8*)edrvInstance_l.pIoAddr + SCBSTAT);
    iowrite8(cmd_p, (UINT8*)edrvInstance_l.pIoAddr + SCBCMD);

    prevCmd = cmd_p;

    return TRUE;
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
    tOplkError              ret = kErrorOk;
    int                     result = 0;
    int                     loopCount;
    tCommandBlock*          pCmdBlock;
    tRxDescCmdBlock*        pRxDescCmdBlock;
    UINT16                  wordValue;
    UINT32                  dwordValue;
    UINT8                   byteValue;

    if (edrvInstance_l.pPciDev != NULL)
    { // Edrv is already connected to a PCI device
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

    wordValue = 0;
    pci_read_config_word(pPciDev_p, PCI_VENDOR_ID, &wordValue);
    printk("Vendor ID : %d\n", wordValue);

    wordValue = 0;
    pci_read_config_word(pPciDev_p, PCI_DEVICE_ID, &wordValue);
    printk("Device ID : %d\n", wordValue);

    byteValue = 0;
    pci_read_config_byte(pPciDev_p, PCI_REVISION_ID, &byteValue);
    printk("Revision ID : %d\n", byteValue);

    edrvInstance_l.pPciDev = pPciDev_p;

    if (edrvInstance_l.pPciDev == NULL)
    {
        printk("%s pPciDev==NULL\n", __func__);
        goto ExitFail;
    }

    result = pci_request_regions(pPciDev_p, DRV_NAME);
    if (result != 0)
    {
        goto ExitFail;
    }

    edrvInstance_l.pIoAddr = ioremap(pci_resource_start(pPciDev_p, 0), pci_resource_len(pPciDev_p, 0));
    if (edrvInstance_l.pIoAddr == NULL)
    { // remap of controller's register space failed
        result = -EIO;
        goto ExitFail;
    }

    // enable PCI busmaster
    pci_set_master(pPciDev_p);

    // reset chip code - start
    /* clear pending interrupts */
    wordValue = ioread16((UINT8*)edrvInstance_l.pIoAddr + SCBSTAT);
    udelay(DELAY_CLEAR_INT);
    iowrite16(wordValue, (UINT8*)edrvInstance_l.pIoAddr + SCBSTAT);

    /* write a reset command into PORT register */
    iowrite32(PORT_SOFTRESET, (UINT8*)edrvInstance_l.pIoAddr + PORT);

    /* wait 10 system clocks and 5 transmit clocks (10uS) */
    udelay(DELAY_SYS_TX_CLK);

    /* update software copy of device state */
    // lp->cuc_active = B_FALSE;

    /* mask interrupt */
    iowrite8(SC_M >> 8, (UINT8*)edrvInstance_l.pIoAddr + SCBCMD + 1);
    // reset chip code - end

    // init chip code start
    /* disable early Rx Int */
    iowrite8(0, (UINT8*)edrvInstance_l.pIoAddr + EARLYRXINT);

    /* disable flow control */
    iowrite8(0, (UINT8*)edrvInstance_l.pIoAddr + FCTRL);

    // issue commands to scb to load the cu and ru base addresses - start
    issueScbcmd(SC_CUC_LOADBASE, 0, 0);
    issueScbcmd(SC_RUC_LOADBASE, 0, 0);

    result = pci_set_dma_mask(pPciDev_p, DMA_BIT_MASK(32));
    if (result != 0)
    {
        result = -ENOMEM;
        goto ExitFail;
    }

    edrvInstance_l.pCbVirtAdd = pci_alloc_consistent(pPciDev_p, CB_REQUIRED_SIZE * MAX_CBS, &(edrvInstance_l.cbDmaHandle));
    if (edrvInstance_l.pCbVirtAdd == NULL)
    {
        result = -ENOMEM;
        goto ExitFail;
    }

    // fill CBs for which memory has been allocated - start
    for (loopCount = 0; loopCount < MAX_CBS; loopCount++)
    {
        pCmdBlock = (tCommandBlock*)((UINT8*)edrvInstance_l.pCbVirtAdd + (CB_REQUIRED_SIZE * loopCount));
        if (loopCount == (MAX_CBS - 1))
        {
            pCmdBlock->statusCommand = 0x00000000;
            pCmdBlock->linkAddr = edrvInstance_l.cbDmaHandle;
        }
        else
        {
            pCmdBlock->statusCommand = 0x00000000;
            pCmdBlock->linkAddr = edrvInstance_l.cbDmaHandle + (CB_REQUIRED_SIZE * (loopCount + 1));
        }
    }
    // fill CBs for which memory has been allocated - end

    // allocate memory for RFDs - start
    edrvInstance_l.pRfdVirtAdd = pci_alloc_consistent(pPciDev_p, RFD_REQUIRED_SIZE * MAX_RFDS, &(edrvInstance_l.rfdDmaAdd));
    if (edrvInstance_l.pRfdVirtAdd == NULL)
    {
        result = -ENOMEM;
        goto ExitFail;
    }
    // allocate memory for RFDs - end

    // fill RFDs for which memory has been allocated - start
    for (loopCount = 0; loopCount < MAX_RFDS; loopCount++)
    {
        pRxDescCmdBlock = (tRxDescCmdBlock*)((UINT8*)edrvInstance_l.pRfdVirtAdd + (RFD_REQUIRED_SIZE * loopCount));
        if (loopCount == (MAX_RFDS - 1))
        {
            pRxDescCmdBlock->statusCommand = 0x00000000;
            pRxDescCmdBlock->size = (((RFD_REQUIRED_SIZE - sizeof(tRxDescCmdBlock)))
                                     << RFD_SIZE_SHIFT) & RFD_SIZE;
            pRxDescCmdBlock->linkAddr = edrvInstance_l.rfdDmaAdd;
        }
        else
        {
            pRxDescCmdBlock->statusCommand = 0x00000000;
            pRxDescCmdBlock->size = (((RFD_REQUIRED_SIZE - sizeof(tRxDescCmdBlock)))
                                     << RFD_SIZE_SHIFT) & RFD_SIZE;
            pRxDescCmdBlock->linkAddr = edrvInstance_l.rfdDmaAdd + (RFD_REQUIRED_SIZE * (loopCount + 1));
        }
    }

    //acknowledge pended interrupts and clear interrupt mask
    wordValue = ioread16((UINT8*)edrvInstance_l.pIoAddr + SCBSTAT);
    udelay(5);
    iowrite16(wordValue, (UINT8*)edrvInstance_l.pIoAddr + SCBSTAT);
    iowrite8(0x2C, (UINT8*)edrvInstance_l.pIoAddr + SCBCMD + 1);

    // install interrupt handler
    result = request_irq(pPciDev_p->irq, edrvIrqHandler, IRQF_SHARED, DRV_NAME, pPciDev_p);
    if (result != 0)
    {
        goto ExitFail;
    }

    // allocate buffers
    printk("%s allocate buffers\n", __func__);
    // allocate tx-buffers
    edrvInstance_l.pTxBuf = pci_alloc_consistent(pPciDev_p, EDRV_TX_BUFFER_SIZE,
                                                 &edrvInstance_l.pTxBufDma);
    if (edrvInstance_l.pTxBuf == NULL)
    {
        result = -ENOMEM;
        goto ExitFail;
    }


     // check if user specified a MAC address
    if ((edrvInstance_l.initParam.aMacAddr[0] != 0) ||
        (edrvInstance_l.initParam.aMacAddr[1] != 0) ||
        (edrvInstance_l.initParam.aMacAddr[2] != 0) ||
        (edrvInstance_l.initParam.aMacAddr[3] != 0) ||
        (edrvInstance_l.initParam.aMacAddr[4] != 0) ||
        (edrvInstance_l.initParam.aMacAddr[5] != 0))
    { // write specified MAC address to controller
        dwordValue = 0;

        dwordValue |= edrvInstance_l.initParam.aMacAddr[0] << 0;
        dwordValue |= edrvInstance_l.initParam.aMacAddr[1] << 8;
        dwordValue |= edrvInstance_l.initParam.aMacAddr[2] << 16;
        dwordValue |= edrvInstance_l.initParam.aMacAddr[3] << 24;
        dwordValue = 0;
        dwordValue |= edrvInstance_l.initParam.aMacAddr[4] << 0;
        dwordValue |= edrvInstance_l.initParam.aMacAddr[5] << 8;
     // dwordValue |= EDRV_REGDW_RAH_AV;
    }
    else
    //Read from EEPROM
    {
        checkEepromSize();

        for (loopCount = 0; loopCount < 6; loopCount += 2)
        {
            wordValue = readEeprom(loopCount / 2);
            edrvInstance_l.initParam.aMacAddr[loopCount] = (UINT8)wordValue;
            edrvInstance_l.initParam.aMacAddr[loopCount+1] = (UINT8)(wordValue >> 8);
        }
    }
    // fill RFDs for which memory has been allocated - end

    ret = cmdDescWrite(OP_ADDRSETUP, 0);
    if (ret != kErrorOk)
    {
        result = -EIO;
        goto Exit;
    }
    ret = cmdDescWrite(OP_CONFIGURE, 0);
    if (ret != kErrorOk)
    {
        result = -EIO;
        goto Exit;
    }

    edrvInstance_l.tailTxDesc = 0;
    edrvInstance_l.headTxDesc = 0;
    edrvInstance_l.headRxDesc = 0;
    edrvInstance_l.tailRxDesc = MAX_RFDS - 1;
    pRxDescCmdBlock = (tRxDescCmdBlock*)((UINT8*)edrvInstance_l.pRfdVirtAdd + (RFD_REQUIRED_SIZE * edrvInstance_l.tailRxDesc));
    pRxDescCmdBlock->statusCommand = CS_S;

    //Enable the receiver
    issueScbcmd(SC_RUC_START, edrvInstance_l.rfdDmaAdd, 0);
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
    if (edrvInstance_l.pPciDev != pPciDev_p)
    { // trying to remove unknown device
        BUG_ON(edrvInstance_l.pPciDev != pPciDev_p);
        goto Exit;
    }

    if (edrvInstance_l.pIoAddr != NULL)
    {
        // disable interrupts
        iowrite8(SC_M >> 8, (UINT8*)edrvInstance_l.pIoAddr + SCBCMD + 1);

        // disable the receiver
        issueScbcmd(SC_RUC_ABORT, 0, 0);
    }

     // remove interrupt handler
    free_irq(pPciDev_p->irq, pPciDev_p);

    // free buffers
    if (edrvInstance_l.pCbVirtAdd != NULL)
    {
        pci_free_consistent(pPciDev_p, CB_REQUIRED_SIZE * MAX_CBS, edrvInstance_l.pCbVirtAdd, edrvInstance_l.cbDmaHandle);
        edrvInstance_l.pCbVirtAdd = NULL;
    }

    // tRxDescCmdBlock memory free in edrvremoveone - start
    if (edrvInstance_l.pRfdVirtAdd != NULL)
    {
        pci_free_consistent(pPciDev_p, RFD_REQUIRED_SIZE * MAX_RFDS, edrvInstance_l.pRfdVirtAdd, edrvInstance_l.rfdDmaAdd);
        edrvInstance_l.pRfdVirtAdd = NULL;
    }
    // tRxDescCmdBlock memory free in edrvremoveone - end

    if (edrvInstance_l.pTxBuf != NULL)
    {
        pci_free_consistent(pPciDev_p, EDRV_TX_BUFFER_SIZE,
                            edrvInstance_l.pTxBuf, edrvInstance_l.pTxBufDma);
        edrvInstance_l.pTxBuf = NULL;
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
