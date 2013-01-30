/****************************************************************************

  (c) SYSTEC electronic GmbH, D-07973 Greiz, August-Bebel-Str. 29
      www.systec-electronic.com

  Project:      openPOWERLINK

  Description:  ethernetdriver
                "fast ethernet controller" (FEC)
                freescale coldfire MCF548x and compatible FEC

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

  2006/07/04 m.b.:   start of implementation

****************************************************************************/

#include "global.h"
#include "EplInc.h"
#include "edrv.h"

//#include <stdlib.h>
#if TARGET_SYSTEM == _NO_OS_
    #include "common.h"
    #include "dma_utils.h"
    #include "MCD_dma.h"
    #include "general.h"

#elif TARGET_SYSTEM == _LINUX_

    #include <linux/config.h>
    #include <linux/module.h>
    #include <linux/kernel.h>
    #include <linux/init.h>
    #include <linux/errno.h>
    #include <linux/major.h>
    #include <linux/version.h>
    #include <asm/io.h>
    #include <asm/uaccess.h>
    #include <asm/atomic.h>
    #include <linux/sched.h>
    #include <linux/delay.h>

    #ifdef CONFIG_COLDFIRE
        #include <asm/coldfire.h>
        #include <asm/sys_sram.h>
        #include <asm/virtconvert.h>
        #include <asm/dma.h>
        #include <asm/MCD_dma.h>
        #include <asm/m5485gpio.h>
    #endif

#else
    #error "TARGET_SYSTEM currently not supported by FEC driver"
#endif

#include "fec.h"
#include "ks8721.h"
#include "ksz8893.h"

/***************************************************************************/
/*                                                                         */
/*                                                                         */
/*          G L O B A L   D E F I N I T I O N S                            */
/*                                                                         */
/*                                                                         */
/***************************************************************************/
//
// half duplex mode is neccessary, do not use full duplex!

// Buffer handling:
// All buffers are created statically (i.e. at compile time resp. at
// initialisation via kmalloc() ) and not dynamically on request (i.e. via
// EdrvAllocTxMsgBuffer(). The MCD_bufDescFec (Tx and Rx) structures
// are in SYS_SRAM (i.e. in SYS_SRAM_FEC_START). These must be 32 bit aligned.
// The frame buffers must be 16 byte aligned and are located in normal RAM.
// EdrvAllocTxMsgBuffer() searches for an unused buffer which is large enough.
// EdrvInit() may allocate some buffers with sizes less than maximum frame
// size (i.e. 1520 bytes), e.g. for SoC, SoA, StatusResponse, IdentResponse,
// NMT requests / commands. The less the size of the buffer the less the
// number of the buffer.


//---------------------------------------------------------------------------
// const defines
//---------------------------------------------------------------------------

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

#define FEC_COUNT_LATECOLLISION         TGT_DBG_SIGNAL_TRACE_POINT(10)
#define FEC_COUNT_TX_COL_RL             TGT_DBG_SIGNAL_TRACE_POINT(11)
#define FEC_COUNT_TX_FUN                TGT_DBG_SIGNAL_TRACE_POINT(12)
#define FEC_COUNT_TX_WAIT               TGT_DBG_SIGNAL_TRACE_POINT(13)
#define FEC_COUNT_TX_FAE                TGT_DBG_SIGNAL_TRACE_POINT(14)
#define FEC_COUNT_TX_FUF                TGT_DBG_SIGNAL_TRACE_POINT(15)
#define FEC_COUNT_TX_FOF                TGT_DBG_SIGNAL_TRACE_POINT(16)
#define FEC_COUNT_RX_WAIT               TGT_DBG_SIGNAL_TRACE_POINT(17)
#define FEC_COUNT_RX_FAE                TGT_DBG_SIGNAL_TRACE_POINT(18)
#define FEC_COUNT_RX_FUF                TGT_DBG_SIGNAL_TRACE_POINT(19)

#define FEC_TRACE(x)                    TGT_DBG_POST_TRACE_VALUE(((x) & 0xFFFF0000) | 0x0000FEC0 | (FEC_ECR(base_addr) & 0x0000000F))

//---------------------------------------------------------------------------
// local types
//---------------------------------------------------------------------------

typedef unsigned char mac_addr[6];  // MAC address type.

// Private structure
struct fec_priv
{
    unsigned int fecpriv_current_rx;            // Index of the reception array
    MCD_bufDescFec *fecpriv_rxdesc;             // Pointer to the reception array of descriptors
	unsigned char *fecpriv_rxbuf;		        // Address of reception buffers
    void * base_addr;                           // Base address
    volatile unsigned int fecpriv_flags;        // Flags
    unsigned int fecpriv_initiator_rx;          // Reception DMA initiator
    unsigned int fecpriv_initiator_tx;          // Transmission DMA initiator
    int fecpriv_fec_rx_channel;                 // Reception DMA channel
    int fecpriv_fec_tx_channel;                 // Transmission DMA channel
    int fecpriv_rx_requestor;                   // Reception DMA requestor
    int fecpriv_tx_requestor;                   // Transmission DMA requestor
    void * fecpriv_interrupt_fec_rx_handler;    // Reception handler
    void * fecpriv_interrupt_fec_tx_handler;    // Transmission handler

    unsigned char *fecpriv_mac_addr;            // MAC address
};



struct eth_if
{
    void *          if_ptr;          // points to specific control data structure of current interface.
};




//---------------------------------------------------------------------------
// module global vars
//---------------------------------------------------------------------------
// buffers and buffer descriptors and pointers

#if TARGET_SYSTEM == _LINUX_
    unsigned char *fec_rxbuf_g;       // pointer to Rx array of buffers
    unsigned char *fec_txbuf_g;       // pointer to Tx array of buffers
#elif TARGET_SYSTEM == _NO_OS_
    extern unsigned char  fec_rxbuf_g[(FEC_RX_BUF_NUMBER * FEC_MAXBUF_SIZE) + 15];       // Reception array of buffers
    extern unsigned char  fec_txbuf_g[(FEC_TX_BUF_NUMBER * FEC_MAXBUF_SIZE) + 15];       // Transmission array of buffers
#endif


#if (EDRV_USED_ETH_CTRL == 0)
    mac_addr fec_mac_addr_fec0 = {0x00,0x11,0x22,0x33,0x44,0x50};   //Default IP address of FEC0

    struct fec_priv fec_priv_fec0;

    struct eth_if fec_dev_fec0=
    {
       &fec_priv_fec0,
    };

#else
    mac_addr fec_mac_addr_fec1 = {0x00,0x11,0x22,0x33,0x44,0x51};   //Default IP address of FEC1

    struct fec_priv fec_priv_fec1;

    struct eth_if fec_dev_fec1=
    {
       &fec_priv_fec1,
    };

#endif

//---------------------------------------------------------------------------
// local function prototypes
//---------------------------------------------------------------------------
// FEC functions
void fec_initialize (struct eth_if *nd);
int fec_reset_controller (struct eth_if *nd);
void fec_stop_controller (struct eth_if *nd);
void fec_interrupt_fec_tx_handler(struct eth_if *nd);
void fec_interrupt_fec_rx_handler(struct eth_if *nd);
void fec_interrupt_fec_tx_handler_fec0(void);
void fec_interrupt_fec_rx_handler_fec0(void);
void fec_interrupt_fec_tx_handler_fec1(void);
void fec_interrupt_fec_rx_handler_fec1(void);



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

typedef struct
{
    BOOL                m_afTxBufUsed[FEC_TX_BUF_NUMBER];
    unsigned long       m_ulTxBufPointer;             // pointer to Tx buffer
    tEdrvTxBuffer*      m_pLastTransmittedTxBuffer;
    MCD_bufDescFec*     m_pTxDesc;
    unsigned int        m_uiHeadTxDesc;
    unsigned int        m_uiTailTxDesc;
    tEdrvTxBuffer*      m_apTxBuffer[FEC_TX_DESC_NUMBER]; // pointers to previously sent tx buffers

} tEdrvInstance;


//---------------------------------------------------------------------------
// local vars
//---------------------------------------------------------------------------
tEdrvInitParam  EdrvInitParam_l;
tEdrvInstance   EdrvInstance_l;

//---------------------------------------------------------------------------
// local function prototypes
//---------------------------------------------------------------------------

BYTE EdrvCalcHash (BYTE * pbMAC_p);



//---------------------------------------------------------------------------
//
// Function:     EdrvInit
//
// Description:  function for init of the FEC
//
// Parameters:   EdrvInit_p - pointer to struct including the init-parameters
//
// Returns:      Errorcode - kEplSuccessful
//
// State:
//
//---------------------------------------------------------------------------
tEplKernel EdrvInit                   (tEdrvInitParam * pEdrvInitParam_p)
{
DWORD               i;
tEplKernel          Ret;
int                 iResult;
unsigned long       ulOffset;
struct fec_priv *   fp;

    Ret = kEplSuccessful;

    // clear instance structure
    EPL_MEMSET(&EdrvInstance_l, 0, sizeof (EdrvInstance_l));

    // save the init data
    EdrvInitParam_l = *pEdrvInitParam_p;

#if TARGET_SYSTEM == _LINUX_
    // -------- Memory allocation for Tx buffer --------

    fec_txbuf_g = kmalloc(FEC_TX_BUF_NUMBER * FEC_MAXBUF_SIZE + 15, GFP_DMA);

    if (!fec_txbuf_g)
    {
        return -ENOMEM;
    }

    ulOffset = ((virt_to_phys(fec_txbuf_g) + 15) & 0xFFFFFFF0) - virt_to_phys(fec_txbuf_g);

    EdrvInstance_l.m_ulTxBufPointer = ((unsigned long)fec_txbuf_g + ulOffset);
#else
    // -------- Tx buffer pointer adjustment to static buffer -------

    EdrvInstance_l.m_ulTxBufPointer = ((unsigned long)(fec_txbuf_g + 15) & 0xFFFFFFF0);
#endif

    // adjust pointer to Tx buffer descriptors
#if (EDRV_USED_ETH_CTRL == 0)
    EdrvInstance_l.m_pTxDesc = (MCD_bufDescFec*)FEC_TX_DESC_FEC0;
#else
    EdrvInstance_l.m_pTxDesc = (MCD_bufDescFec*)FEC_TX_DESC_FEC1;
#endif

    // initialize Tx buffer descriptors
    for (i = 0; i < FEC_TX_DESC_NUMBER; i++)
    {
        EdrvInstance_l.m_pTxDesc[i].statCtrl = MCD_FEC_INTERRUPT | MCD_FEC_END_FRAME;
        EdrvInstance_l.m_apTxBuffer[i] = NULL;
    }
    EdrvInstance_l.m_pTxDesc[FEC_TX_DESC_NUMBER-1].statCtrl |= MCD_FEC_WRAP;

    EdrvInstance_l.m_uiHeadTxDesc = 0;
    EdrvInstance_l.m_uiTailTxDesc = 0;

    // set mac address
#if (EDRV_USED_ETH_CTRL == 0)
    fec_mac_addr_fec0[0] = pEdrvInitParam_p->m_abMyMacAddr[0];
    fec_mac_addr_fec0[1] = pEdrvInitParam_p->m_abMyMacAddr[1];
    fec_mac_addr_fec0[2] = pEdrvInitParam_p->m_abMyMacAddr[2];
    fec_mac_addr_fec0[3] = pEdrvInitParam_p->m_abMyMacAddr[3];
    fec_mac_addr_fec0[4] = pEdrvInitParam_p->m_abMyMacAddr[4];
    fec_mac_addr_fec0[5] = pEdrvInitParam_p->m_abMyMacAddr[5];
#else
    fec_mac_addr_fec1[0] = EdrvInitParam_l.m_abMyMacAddr[0];
    fec_mac_addr_fec1[1] = EdrvInitParam_l.m_abMyMacAddr[1];
    fec_mac_addr_fec1[2] = EdrvInitParam_l.m_abMyMacAddr[2];
    fec_mac_addr_fec1[3] = EdrvInitParam_l.m_abMyMacAddr[3];
    fec_mac_addr_fec1[4] = EdrvInitParam_l.m_abMyMacAddr[4];
    fec_mac_addr_fec1[5] = EdrvInitParam_l.m_abMyMacAddr[5];
#endif

    printk( "Initializing FEC for EPL...\n" );

#if TARGET_SYSTEM == _NO_OS_
    dma_irq_disable();
    iResult = MCD_initDma (  (dmaRegs*)(MCF_MBAR+0x8000),(void *)SYS_SRAM_DMA_START, MCD_RELOC_TASKS );

    if(iResult != MCD_OK)
    {
        printk( "ERROR...MCD_initDma()\n" );
    }

//m.b. das muss variabel sein
//    dma_irq_enable(6, 3);
    dma_irq_enable(3, 3);
#endif

    // disable FEC and DMA interrupts
    // d.k.: this is not really necessary, because TgtInitEthIsr does this
/*#if (EDRV_USED_ETH_CTRL == 0)
    TgtEnableEthInterrupt0(FALSE, EPL_TGT_INTMASK_ETH | EPL_TGT_INTMASK_DMA);
#else
    TgtEnableEthInterrupt1(FALSE, EPL_TGT_INTMASK_ETH | EPL_TGT_INTMASK_DMA);
#endif
*/


#if EDRV_BENCHMARK != FALSE
    MCF_GPIO_PODR_PCIBR |= 0x03;        // set LED data bits to off
    MCF_GPIO_PDDR_PCIBR |= 0x03;        // set LED direction bits to out
    MCF_GPIO_PAR_PCIBR  &= 0xFFF0;      // set PCIBR0-PCIBR1 to GPIO pins
#endif

    // setup interrupt controller
    Ret = TgtInitEthIsr();
    if (Ret != kEplSuccessful)
    {
        goto Exit;
    }

    iResult = fec_init_module();
    if(iResult != OK)
    {
        printk( "ERROR: fec_init_module()\n" );
        Ret = kEplNoResource;
        TgtFreeEthIsr();
        goto Exit;
    }

#if (EDRV_USED_ETH_CTRL == 0)
    iResult = fec_reset_controller (&fec_dev_fec0);
#else
    iResult = fec_reset_controller (&fec_dev_fec1);
#endif
    if(iResult != OK)
    {
        printk( "ERROR: fec_reset_controller()\n" );
        Ret = kEplNoResource;
        TgtFreeEthIsr();
        goto Exit;
    }


    // enable FEC and DMA interrupts
#if (EDRV_USED_ETH_CTRL == 0)
    TgtEnableEthInterrupt0(TRUE, EPL_TGT_INTMASK_ETH | EPL_TGT_INTMASK_DMA);
#else
    TgtEnableEthInterrupt1(TRUE, EPL_TGT_INTMASK_ETH | EPL_TGT_INTMASK_DMA);
#endif

    // Receive the pointer to the private structure
#if (EDRV_USED_ETH_CTRL == 0)
    fp = (struct fec_priv *)fec_dev_fec0.if_ptr;
#else
    fp = (struct fec_priv *)fec_dev_fec1.if_ptr;
#endif

    // start tx dma task
    MCD_startDma(fp->fecpriv_fec_tx_channel, (s8*) &EdrvInstance_l.m_pTxDesc[0], 0,
             (s8*) &(FEC_FECTFDR(fp->base_addr)), 0,
             FEC_MAX_FRM_SIZE, 0, fp->fecpriv_initiator_tx,
             FEC_TX_DMA_PRI,
             MCD_FECTX_DMA | MCD_INTERRUPT | MCD_TT_FLAGS_DEF | MCD_TT_FLAGS_SP,
             MCD_NO_CSUM | MCD_NO_BYTE_SWAP);

//    printf("A: LR=0x%03lX R=0x%03lX LW=0x%03lX W=0x%03lX\n", FEC_FECTLRFP(fp->base_addr), FEC_FECTFRP(fp->base_addr), FEC_FECTLWFP(fp->base_addr), FEC_FECTFWP(fp->base_addr));

Exit:
    return Ret;

}

//---------------------------------------------------------------------------
//
// Function:     EdrvShutdown
//
// Description:  Shutdown the FEC
//
// Parameters:   ---
//
// Returns:      Errorcode - kEplSuccessful
//
// State:
//
//---------------------------------------------------------------------------
tEplKernel EdrvShutdown               (void)
{
#if (EDRV_USED_ETH_CTRL == 0)
    fec_stop_controller (&fec_dev_fec0);
#else
    fec_stop_controller (&fec_dev_fec1);
#endif

    // disable FEC but not DMA interrupts
#if (EDRV_USED_ETH_CTRL == 0)
    TgtEnableEthInterrupt0(FALSE, EPL_TGT_INTMASK_ETH);
#else
    TgtEnableEthInterrupt1(FALSE, EPL_TGT_INTMASK_ETH);
#endif

    fec_cleanup_module();

#if TARGET_SYSTEM == _LINUX_
    // memory deallocation of Tx buffers
	kfree(fec_txbuf_g);
#endif

    TgtFreeEthIsr();

    return kEplSuccessful;
}

//---------------------------------------------------------------------------
//
// Function:     EdrvDefineRxMacAddrEntry
//
// Description:  Set a MultiCast-Entry into the Ethernet-Controller
//
// Parameters:   pbMacAddr - pointer to muticast entry to set
//
// Returns:      Errorcode - kEplSuccessful
//
// State:
//
//---------------------------------------------------------------------------
tEplKernel EdrvDefineRxMacAddrEntry (BYTE * pbMacAddr_p)
{
tEplKernel Ret = kEplSuccessful;
DWORD dwData;
BYTE bHash;

    //Receive the pointer to the private structure
#if (EDRV_USED_ETH_CTRL == 0)
    struct fec_priv *fp = (struct fec_priv *)fec_dev_fec0.if_ptr;
#else
    struct fec_priv *fp = (struct fec_priv *)fec_dev_fec1.if_ptr;
#endif

    // Receive the base address
    unsigned long base_addr = (unsigned long)fp->base_addr;

    bHash = EdrvCalcHash (pbMacAddr_p);

    // check for MultiCast or UniCast address
    if ((*pbMacAddr_p & 1))
    {
        // MultiCast
        if (bHash > 31)
        {
            dwData = FEC_GAUR(base_addr);
            dwData |= 1 << (bHash - 32);
            FEC_GAUR(base_addr) = dwData;
        }
        else
        {
            dwData = FEC_GALR(base_addr);
            dwData |= 1 << bHash;
            FEC_GALR(base_addr) = dwData;
        }
    }
    else
    {
        // UniCast
        if (bHash > 31)
        {
            dwData = FEC_IAUR(base_addr);
            dwData |= 1 << (bHash - 32);
            FEC_IAUR(base_addr) = dwData;
        }
        else
        {
            dwData = FEC_IALR(base_addr);
            dwData |= 1 << bHash;
            FEC_IALR(base_addr) = dwData;
        }
    }

    return Ret;
}


//---------------------------------------------------------------------------
//
// Function:     EdrvUndefineRxMacAddrEntry
//
// Description:  Reset a MultiCast-Entry into the Ethernet-Controller
//
// Parameters:   pbMCEntry_p - pointer to muticast entry to reset
//
// Returns:      Errorcode - kEplSuccessful
//
// State:
//
//---------------------------------------------------------------------------
tEplKernel EdrvUndefineRxMacAddrEntry (BYTE * pbMacAddr_p)
{
tEplKernel Ret = kEplSuccessful;
DWORD dwData;
BYTE bHash;

    //Receive the pointer to the private structure
#if (EDRV_USED_ETH_CTRL == 0)
    struct fec_priv *fp = (struct fec_priv *)fec_dev_fec0.if_ptr;
#else
    struct fec_priv *fp = (struct fec_priv *)fec_dev_fec1.if_ptr;
#endif

    // Receive the base address
    unsigned long base_addr = (unsigned long)fp->base_addr;

    bHash = EdrvCalcHash (pbMacAddr_p);

    // check for MultiCast or UniCast address
    if ((*pbMacAddr_p & 1))
    {
        // MultiCast
        if (bHash > 31)
        {
            dwData = FEC_GAUR(base_addr);
            dwData &= ~(1 << (bHash - 32));
            FEC_GAUR(base_addr) = dwData;
        }
        else
        {
            dwData = FEC_GALR(base_addr);
            dwData &= ~(1 << bHash);
            FEC_GALR(base_addr) = dwData;
        }
    }
    else
    {
        // UniCast
        if (bHash > 31)
        {
            dwData = FEC_IAUR(base_addr);
            dwData &= ~(1 << (bHash - 32));
            FEC_IAUR(base_addr) = dwData;
        }
        else
        {
            dwData = FEC_IALR(base_addr);
            dwData &= ~(1 << bHash);
            FEC_IALR(base_addr) = dwData;
        }
    }

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
// Function:     EdrvAllocTxMsgBuffer
//
// Description:  Register a Tx-Buffer
//
// Parameters:   pBuffer_p - pointer to Buffer structure
//
// Returns:      Errorcode - kEplSuccessful
//                         - kEplEdrvNoFreeBufEntry
//
// State:
//
//---------------------------------------------------------------------------
tEplKernel EdrvAllocTxMsgBuffer       (tEdrvTxBuffer * pBuffer_p)
{
tEplKernel Ret = kEplSuccessful;
DWORD i;

    if (pBuffer_p->m_uiMaxBufferLen > FEC_MAXBUF_SIZE)
    {
        Ret = kEplEdrvNoFreeBufEntry;
        goto Exit;
    }

    // search a free Tx buffer with appropriate size
    for (i = 0; i < FEC_TX_BUF_NUMBER; i++)
    {
        if (EdrvInstance_l.m_afTxBufUsed[i] == FALSE)
        {
            // free channel found
            EdrvInstance_l.m_afTxBufUsed[i] = TRUE;
            pBuffer_p->m_BufferNumber.m_dwVal = i;
#if TARGET_SYSTEM == _LINUX_
            pBuffer_p->m_pbBuffer = (BYTE *) phys_to_virt(EdrvInstance_l.m_ulTxBufPointer + FEC_MAXBUF_SIZE * i);
#else
            pBuffer_p->m_pbBuffer = (BYTE *) (EdrvInstance_l.m_ulTxBufPointer + FEC_MAXBUF_SIZE * i);
#endif
            pBuffer_p->m_uiMaxBufferLen = FEC_MAXBUF_SIZE;

            break;
        }
    }
    if (i >= FEC_TX_BUF_NUMBER)
    {
        Ret = kEplEdrvNoFreeBufEntry;
        goto Exit;
    }

Exit:
    return Ret;

}


//---------------------------------------------------------------------------
//
// Function:     EdrvReleaseTxMsgBuffer
//
// Description:  Register a Tx-Buffer
//
// Parameters:   pBuffer_p - pointer to Buffer structure
//
// Returns:      Errorcode - kEplSuccessful
//
// State:
//
//---------------------------------------------------------------------------
tEplKernel EdrvReleaseTxMsgBuffer     (tEdrvTxBuffer * pBuffer_p)
{
unsigned int uiBufferNumber;

uiBufferNumber = pBuffer_p->m_BufferNumber.m_dwVal;

    if (uiBufferNumber < FEC_TX_BUF_NUMBER)
    {
        EdrvInstance_l.m_afTxBufUsed[uiBufferNumber] = FALSE;
    }

    return kEplSuccessful;

}

//---------------------------------------------------------------------------
//
// Function:     EdrvSendTxMsg
//
// Description:  starts a transmission of the buffer
//
// Parameters:   pbBuffer_p - bufferdescriptor to transmit
//
// Returns:      Errorcode - kEplSuccessful
//
// State:
//
//---------------------------------------------------------------------------
tEplKernel EdrvSendTxMsg              (tEdrvTxBuffer * pTxBuffer_p)
{
tEplKernel          Ret;
MCD_bufDescFec *    pTxDesc;
unsigned int        uiBufferNumber;
unsigned long       base_addr;
struct fec_priv *   fp;

    Ret = kEplSuccessful;

    uiBufferNumber = pTxBuffer_p->m_BufferNumber.m_dwVal;

    if ((uiBufferNumber >= FEC_TX_BUF_NUMBER)
        || (EdrvInstance_l.m_afTxBufUsed[uiBufferNumber] == FALSE))
    {
        Ret = kEplEdrvBufNotExisting;
        goto Exit;
    }

    // one descriptor has to be left empty for distinction between full and empty
    if (((EdrvInstance_l.m_uiTailTxDesc + 1) & FEC_TX_DESC_MASK) == EdrvInstance_l.m_uiHeadTxDesc)
    {
        Ret = kEplEdrvNoFreeTxDesc;
        goto Exit;
    }


    // Receive the pointer to the private structure
#if (EDRV_USED_ETH_CTRL == 0)
    fp = (struct fec_priv *)fec_dev_fec0.if_ptr;
#else
    fp = (struct fec_priv *)fec_dev_fec1.if_ptr;
#endif

    // Receive the base address
    base_addr = (unsigned long)fp->base_addr;

    if ((FEC_TCR(base_addr) & FEC_TCR_GTS) != 0)
    {   // transmission was already stopped
        // Disable FEC (just resetting the (transmit) FIFOs does not work)
        FEC_ECR(base_addr) &= ~FEC_ECR_ETHEREN;
        // Reset FIFOs
        FEC_FECFRST(base_addr) |= FEC_SW_RST;
        FEC_FECFRST(base_addr) &= ~FEC_SW_RST;
        // Enable FEC
        FEC_ECR(base_addr) |= FEC_ECR_ETHEREN;

        // restart transmission
        FEC_TCR(base_addr) &= ~FEC_TCR_GTS;
    }


    // save pointer to buffer structure for TxHandler
    EdrvInstance_l.m_apTxBuffer[EdrvInstance_l.m_uiTailTxDesc] = pTxBuffer_p;

    pTxDesc = &EdrvInstance_l.m_pTxDesc[EdrvInstance_l.m_uiTailTxDesc];
    pTxDesc->length = pTxBuffer_p->m_uiTxMsgLen;

#if TARGET_SYSTEM == _LINUX_
    pTxDesc->dataPointer = (unsigned int) virt_to_phys((void*)(EdrvInstance_l.m_ulTxBufPointer
                                                               + FEC_MAXBUF_SIZE * uiBufferNumber));
#else
    pTxDesc->dataPointer = EdrvInstance_l.m_ulTxBufPointer + FEC_MAXBUF_SIZE * uiBufferNumber;
#endif

    // flush data cache before initializing the descriptor and starting DMA
//    DcacheFlushInvalidateCacheBlock((void*)pBufferDesc->dataPointer, pBufferDesc->length);

    pTxDesc->statCtrl   |= MCD_FEC_BUF_READY;

    // increment tx queue tail
    EdrvInstance_l.m_uiTailTxDesc = (EdrvInstance_l.m_uiTailTxDesc + 1) & FEC_TX_DESC_MASK;

//    printk("S: LR=0x%03X R=0x%03X LW=0x%03X W=0x%03X\n", FEC_FECTLRFP(base_addr), FEC_FECTFRP(base_addr), FEC_FECTLWFP(base_addr), FEC_FECTFWP(base_addr));

#if EDRV_BENCHMARK != FALSE
    // set LED
    MCF_GPIO_PODR_PCIBR |= 0x01;  // Level
//    MCF_GPIO_PODR_PCIBG |= 0x02;  // Level

    // reset LED
//    MCF_GPIO_PODR_PCIBG &= ~0x02;  // Level
#endif

    TGT_DBG_SIGNAL_TRACE_POINT(2);

    // Tell the DMA to continue the reception
    MCD_continDma(fp->fecpriv_fec_tx_channel);

//    printk("A: LR=0x%03X R=0x%03X LW=0x%03X W=0x%03X\n", FEC_FECTLRFP(base_addr), FEC_FECTFRP(base_addr), FEC_FECTLWFP(base_addr), FEC_FECTFWP(base_addr));

#if EDRV_BENCHMARK != FALSE
    // reset LED
//    MCF_GPIO_PODR_PCIBG &= ~0x04;  // Level
#endif

Exit:
    return Ret;
}

//---------------------------------------------------------------------------
//
// Function:     EdrvTxMsgReady
//
// Description:  starts copying the buffer to the ethernet controller's FIFO
//
// Parameters:   pbBuffer_p - bufferdescriptor to transmit
//
// Returns:      Errorcode - kEplSuccessful
//
// State:
//
//---------------------------------------------------------------------------
tEplKernel EdrvTxMsgReady              (tEdrvTxBuffer * pTxBuffer_p)
{
tEplKernel          Ret;
MCD_bufDescFec *    pTxDesc;
unsigned int        uiBufferNumber;
struct fec_priv *   fp;
unsigned long       base_addr;

    Ret = kEplSuccessful;

    uiBufferNumber = pTxBuffer_p->m_BufferNumber.m_dwVal;

    if ((uiBufferNumber >= FEC_TX_BUF_NUMBER)
        || (EdrvInstance_l.m_afTxBufUsed[uiBufferNumber] == FALSE))
    {
        Ret = kEplEdrvBufNotExisting;
        goto Exit;
    }

    // one descriptor has to be left empty for distinction between full and empty
    if (((EdrvInstance_l.m_uiTailTxDesc + 1) & FEC_TX_DESC_MASK) == EdrvInstance_l.m_uiHeadTxDesc)
    {
        Ret = kEplEdrvNoFreeTxDesc;
        goto Exit;
    }


    //Receive the pointer to the private structure
#if (EDRV_USED_ETH_CTRL == 0)
    fp = (struct fec_priv *)fec_dev_fec0.if_ptr;
#else
    fp = (struct fec_priv *)fec_dev_fec1.if_ptr;
#endif

    // Receive the base address
    base_addr = (unsigned long)fp->base_addr;

    if ((FEC_TCR(base_addr) & FEC_TCR_GTS) != 0)
    {   // transmission was already stopped
        // Disable FEC (just resetting the (transmit) FIFOs does not work)
        // $$$ d.k. This is dangerous. We might lose Rx frames.
        FEC_ECR(base_addr) &= ~FEC_ECR_ETHEREN;
        // Reset FIFOs
        FEC_FECFRST(base_addr) |= FEC_SW_RST;
        FEC_FECFRST(base_addr) &= ~FEC_SW_RST;
        // Enable FEC
        FEC_ECR(base_addr) |= FEC_ECR_ETHEREN;

        // empty transmit FIFO
        // set write pointer to below half full
//        FEC_FECTFWP(base_addr) = FEC_FECTFRP(base_addr) + 4;
        // set write pointer to read pointer
//        FEC_FECTFWP(base_addr) = FEC_FECTFRP(base_addr);
//        FEC_FECTFRP(base_addr) = 0;
//        FEC_FECTFWP(base_addr) = 0;

    }

    // stop transmission gracefully
    FEC_TCR(base_addr) |= FEC_TCR_GTS;


    // save pointer to buffer structure for TxHandler
    EdrvInstance_l.m_apTxBuffer[EdrvInstance_l.m_uiTailTxDesc] = pTxBuffer_p;

    pTxDesc = &EdrvInstance_l.m_pTxDesc[EdrvInstance_l.m_uiTailTxDesc];
    pTxDesc->length = pTxBuffer_p->m_uiTxMsgLen;

#if TARGET_SYSTEM == _LINUX_
    pTxDesc->dataPointer = (unsigned int) virt_to_phys((void*)(EdrvInstance_l.m_ulTxBufPointer
                                                               + FEC_MAXBUF_SIZE * uiBufferNumber));
#else
    pTxDesc->dataPointer = EdrvInstance_l.m_ulTxBufPointer + FEC_MAXBUF_SIZE * uiBufferNumber;
#endif

    // flush data cache before initializing the descriptor and starting DMA
    DcacheFlushInvalidateCacheBlock((void*)pTxDesc->dataPointer, pTxDesc->length);

    pTxDesc->statCtrl   |= MCD_FEC_BUF_READY;

    // increment tx queue tail
    EdrvInstance_l.m_uiTailTxDesc = (EdrvInstance_l.m_uiTailTxDesc + 1) & FEC_TX_DESC_MASK;

//    printf("B: LR=0x%03lX R=0x%03lX LW=0x%03lX W=0x%03lX\n", FEC_FECTLRFP(base_addr), FEC_FECTFRP(base_addr), FEC_FECTLWFP(base_addr), FEC_FECTFWP(base_addr));
//    FEC_FECTFRP(base_addr) = (FEC_FECTFRP(base_addr) - 0x40) & FEC_FIFOPOINTERMASK;
//    FEC_FECTLRFP(base_addr) = (FEC_FECTLRFP(base_addr) - 0x4) & FEC_FIFOPOINTERMASK;

    // wait
//    uiBufferNumber = 1000000;
//    while (uiBufferNumber--);
//    printf("C: LR=0x%03lX R=0x%03lX LW=0x%03lX W=0x%03lX\n", FEC_FECTLRFP(base_addr), FEC_FECTFRP(base_addr), FEC_FECTLWFP(base_addr), FEC_FECTFWP(base_addr));

    // set LED
//    MCF_GPIO_PODR_PCIBG |= 0x02;  // Level
    TGT_DBG_SIGNAL_TRACE_POINT(3);

    // Tell the DMA to continue the reception
    MCD_continDma(fp->fecpriv_fec_tx_channel);

//    printf("D: LR=0x%03lX R=0x%03lX LW=0x%03lX W=0x%03lX\n", FEC_FECTLRFP(base_addr), FEC_FECTFRP(base_addr), FEC_FECTLWFP(base_addr), FEC_FECTFWP(base_addr));

    // reset LED
//    MCF_GPIO_PODR_PCIBG &= ~0x04;  // Level

Exit:
    return Ret;
}


//---------------------------------------------------------------------------
//
// Function:     EdrvTxMsgStart
//
// Description:  starts transmission of the ethernet controller's FIFO
//
// Parameters:   pbBuffer_p - bufferdescriptor to transmit
//
// Returns:      Errorcode - kEplSuccessful
//
// State:
//
//---------------------------------------------------------------------------
tEplKernel EdrvTxMsgStart              (tEdrvTxBuffer * pBuffer_p)
{
tEplKernel          Ret;
struct fec_priv *   fp;
unsigned long       base_addr;

    Ret = kEplSuccessful;


    //Receive the pointer to the private structure
#if (EDRV_USED_ETH_CTRL == 0)
    fp = (struct fec_priv *)fec_dev_fec0.if_ptr;
#else
    fp = (struct fec_priv *)fec_dev_fec1.if_ptr;
#endif

    // Receive the base address
    base_addr = (unsigned long)fp->base_addr;

//    printf("1: LR=0x%03lX R=0x%03lX LW=0x%03lX W=0x%03lX\n", FEC_FECTLRFP(base_addr), FEC_FECTFRP(base_addr), FEC_FECTLWFP(base_addr), FEC_FECTFWP(base_addr));
    // restart transmission
    FEC_TCR(base_addr) &= ~FEC_TCR_GTS;

    TGT_DBG_SIGNAL_TRACE_POINT(4);

    // set LED
//    MCF_GPIO_PODR_PCIBG |= 0x02;  // Level

//    FEC_FECTFRP(base_addr) = FEC_FECTLRFP(base_addr);
//    FEC_FECTFRP(base_addr) = FEC_FECTFRP(base_addr);
/*    FEC_FECTFRP(base_addr) = (FEC_FECTLRFP(base_addr) + 1) & FEC_FIFOPOINTERMASK;

    FEC_FECTLRFP(base_addr) = (FEC_FECTFRP(base_addr) + 1) & FEC_FIFOPOINTERMASK;
    FEC_FECTLRFP(base_addr) = (FEC_FECTFRP(base_addr) - 1) & FEC_FIFOPOINTERMASK;
*/
//    printf("2: LR=0x%03lX R=0x%03lX LW=0x%03lX W=0x%03lX\n", FEC_FECTLRFP(base_addr), FEC_FECTFRP(base_addr), FEC_FECTLWFP(base_addr), FEC_FECTFWP(base_addr));

    // reset LED
//    MCF_GPIO_PODR_PCIBG &= ~0x04;  // Level

    return Ret;
}



/************************************************************************
* NAME: fec_init_module
*
* DESCRIPTION: This function performs the general initialization of
*                the fast ethernet controllers (FECs)
*
* RETURNS: This function returns OK.
*************************************************************************/
int fec_init_module(void)
{

#if TARGET_SYSTEM == _LINUX_
	unsigned long offset;
#endif

#if (EDRV_USED_ETH_CTRL == 0)
    // ========= FEC0 ==========
    memset(&fec_priv_fec0, 0, sizeof (struct fec_priv));

#if TARGET_SYSTEM == _LINUX_
    // -------- Memory allocation for Rx buffer --------

    fec_rxbuf_g = kmalloc(FEC_RX_BUF_NUMBER * FEC_MAXBUF_SIZE + 15, GFP_DMA);

    if (!fec_rxbuf_g)
        return -ENOMEM;

    offset = (((unsigned int)virt_to_phys(fec_rxbuf_g) + 15) & 0xFFFFFFF0) - (unsigned int)virt_to_phys(fec_rxbuf_g);

    fec_priv_fec0.fecpriv_rxbuf = (void*)((unsigned long)fec_rxbuf_g + offset);
#else
    // -------- Rx buffer pointer adjustment -------

    fec_priv_fec0.fecpriv_rxbuf = (void*)(((unsigned long)fec_rxbuf_g + 15) & 0xFFFFFFF0);

#endif
    // -------- Initialize FEC0 --------

    // Disable FEC0
    FEC_ECR(FEC_BASE_ADDR_FEC0) = FEC_ECR_DISABLE;

    // Set the base address of FEC0
    fec_priv_fec0.base_addr = (void*)FEC_BASE_ADDR_FEC0;

    // Set the requestor numbers
    fec_priv_fec0.fecpriv_rx_requestor = DMA_FEC0_RX;
    fec_priv_fec0.fecpriv_tx_requestor = DMA_FEC0_TX;


    // Set the pointers of the handlers of FEC0
    fec_priv_fec0.fecpriv_interrupt_fec_rx_handler = fec_interrupt_fec_rx_handler_fec0;
    fec_priv_fec0.fecpriv_interrupt_fec_tx_handler = fec_interrupt_fec_tx_handler_fec0;

    // Initialize the pointer to the transmission array of FEC0
    //mb  fec_priv_fec0.fecpriv_txbuf = fec_txbuf_fec0;
    //mb Align Buffer Descriptors to 4-byte boundary
    //fec_priv_fec0.fecpriv_txbuf = (((int)fec_txbuf_fec0 + 3) & 0xFFFFFFFC);

    // Initialize the pointer to the reception array of descriptors
    fec_priv_fec0.fecpriv_rxdesc = (void*)FEC_RX_DESC_FEC0;
    //mb  fec_priv_fec0.fecpriv_rxdesc = fec_rxdesc_fec0;
    //mb Align Buffer Descriptors to 4-byte boundary
//    fec_priv_fec0.fecpriv_rxdesc = (MCD_bufDescFec*)(((int)fec_rxdesc_fec0 + 3) & 0xFFFFFFFC);

    // Set the  MAC address to the private structure of FEC0
    fec_priv_fec0.fecpriv_mac_addr = fec_mac_addr_fec0;

    // Continue the initialization of FEC0
    fec_initialize(&fec_dev_fec0);

#else
    // ========= FEC1 ==========
    memset(&fec_priv_fec1, 0, sizeof (struct fec_priv));

#if TARGET_SYSTEM == _LINUX_
    // -------- Memory allocation for Rx buffer --------

    fec_rxbuf_g = kmalloc(FEC_RX_BUF_NUMBER * FEC_MAXBUF_SIZE + 15, GFP_DMA);

    if (!fec_rxbuf_g)
        return -ENOMEM;

    offset = (((unsigned int)virt_to_phys(fec_rxbuf_g) + 15) & 0xFFFFFFF0) - (unsigned int)virt_to_phys(fec_rxbuf_g);

    fec_priv_fec1.fecpriv_rxbuf = (void*)((unsigned long)fec_rxbuf_g + offset);
#else
    // -------- Rx buffer pointer adjustment -------

    fec_priv_fec1.fecpriv_rxbuf = (void*)(((unsigned long)fec_rxbuf_g + 15) & 0xFFFFFFF0);

#endif

    // -------- Initialize FEC1 --------

    // Disable FEC1
    FEC_ECR(FEC_BASE_ADDR_FEC1) = FEC_ECR_DISABLE;


    // Set the base address of FEC1
    fec_priv_fec1.base_addr = (void*)FEC_BASE_ADDR_FEC1;

    // Set the requestor numbers
    fec_priv_fec1.fecpriv_rx_requestor = DMA_FEC1_RX;
    fec_priv_fec1.fecpriv_tx_requestor = DMA_FEC1_TX;

    // Set the pointers of the handlers of FEC1
    fec_priv_fec1.fecpriv_interrupt_fec_rx_handler = fec_interrupt_fec_rx_handler_fec1;
    fec_priv_fec1.fecpriv_interrupt_fec_tx_handler = fec_interrupt_fec_tx_handler_fec1;


    // Initialize the pointer to the transmission array of FEC1
    //mb fec_priv_fec1.fecpriv_txbuf = fec_txbuf_fec1;

    // Initialize the pointer to the reception array of descriptors
    fec_priv_fec1.fecpriv_rxdesc = (void*)FEC_RX_DESC_FEC1;
    //mb Align Buffer Descriptors to 4-byte boundary
//    fec_priv_fec1.fecpriv_rxdesc = (MCD_bufDescFec*)(((int)fec_rxdesc_fec1 + 3) & 0xFFFFFFFC);

    // Set the  MAC address to the private structure of FEC1
    fec_priv_fec1.fecpriv_mac_addr = fec_mac_addr_fec1;

    // Continue the initialization of FEC1
    fec_initialize(&fec_dev_fec1);

#endif

    // Initialize FEC/I2C/IRQ Pin Assignment Register
    GPIO_PAR_FECI2CIRQ |= GPIO_PAR_FECI2CIRQ_FEC;

    return OK;
}

/************************************************************************
* NAME: init_module
*
* DESCRIPTION: This function releases the resourses
*
* RETURNS: This function returns zero.
*************************************************************************/
void fec_cleanup_module(void)
{
#if (EDRV_USED_ETH_CTRL == 0)

    // Remove the initiators of FEC0
    dma_remove_initiator(fec_priv_fec0.fecpriv_initiator_tx);
    dma_remove_initiator(fec_priv_fec0.fecpriv_initiator_rx);
#else

    // Remove the initiators of FEC1
    dma_remove_initiator(fec_priv_fec1.fecpriv_initiator_tx);
    dma_remove_initiator(fec_priv_fec1.fecpriv_initiator_rx);
#endif

#if TARGET_SYSTEM == _LINUX_
    // memory deallocation of Rx buffers
	kfree(fec_rxbuf_g);
#endif
}

/************************************************************************
* NAME: fec_initialize
*
* DESCRIPTION: This function performs the initialization of
*                private structure, and reserves the DMA initiators
*
*************************************************************************/
void fec_initialize (struct eth_if *nd)
{

    //Receive the pointer to the private structure
    struct fec_priv *fp = (struct fec_priv *)nd->if_ptr;

    // Temporary variable
    int i;
    // Grab the FEC initiators
    dma_set_initiator(fp->fecpriv_tx_requestor);
    fp->fecpriv_initiator_tx = dma_get_initiator(fp->fecpriv_tx_requestor);
    dma_set_initiator(fp->fecpriv_rx_requestor);
    fp->fecpriv_initiator_rx = dma_get_initiator(fp->fecpriv_rx_requestor);

    // Reset the DMA channels
    fp->fecpriv_fec_rx_channel = -1;
    fp->fecpriv_fec_tx_channel = -1;

    fp->fecpriv_flags = 0;

    // initialize receive buffers with 0xFF for BABR (early receive) interrupt
    for (i = 0; i < (FEC_RX_BUF_NUMBER * FEC_MAXBUF_SIZE); i++)
    {
        fp->fecpriv_rxbuf[i] = 0xFF;
    }

    // Initialize the pointers to the receive buffers
    for (i = 0; i < FEC_RX_BUF_NUMBER; i++)
    {
        // fecpriv_rxbuf was already 16 byte aligned in fec_init_module()
#if TARGET_SYSTEM == _LINUX_
        fp->fecpriv_rxdesc[i].dataPointer = (unsigned int) virt_to_phys(&fp->fecpriv_rxbuf[i * FEC_MAXBUF_SIZE]);
#else
        fp->fecpriv_rxdesc[i].dataPointer = (unsigned int)(&fp->fecpriv_rxbuf[i * FEC_MAXBUF_SIZE]);
#endif
        //mb      fp->fecpriv_rxdesc[i].dataPointer = (unsigned int)&fp->fecpriv_rxbuf[i * FEC_MAXBUF_SIZE];
        // the datapointers must be aligned to 32 (16?) bit addresses
    }

}

/************************************************************************
* NAME: fec_reset_controller
*
* DESCRIPTION: This function performs the initialization of
*                of the fast ethernet controller (FEC)
*                 and corresponding KS8721 transiver
*
* RETURNS: If no error occurs, fec_reset_controller returns OK. Otherwise
*          this function returns ERR
*************************************************************************/

int fec_reset_controller (struct eth_if *nd)
{

    //Receive the pointer to the private structure
    struct fec_priv *fp = (struct fec_priv *)nd->if_ptr;

    // Receive the base address
    unsigned long base_addr = (unsigned long)fp->base_addr;

    // Flag of the duplex mode
    int fduplex = 0;

    // Index
    int i;

    // Channel number
    int channel;

    // Request the DMA channels
    // request the channel for Tx before Rx,
    // so the handler will be called before Rx
    channel = dma_set_channel(fp->fecpriv_tx_requestor);

    if(channel == -1)
    {
        goto ERRORS;
    }

    fp->fecpriv_fec_tx_channel = channel;

    dma_connect(channel, (int) fp->fecpriv_interrupt_fec_tx_handler);

    // Request DMA channel for Rx
    channel = dma_set_channel(fp->fecpriv_rx_requestor);

    if(channel == -1)
    {
        goto ERRORS;
    }

    fp->fecpriv_fec_rx_channel = channel;

    dma_connect(channel, (int) fp->fecpriv_interrupt_fec_rx_handler);

    // Reset FIFOs
    FEC_FECFRST(base_addr) |= FEC_SW_RST;
    FEC_FECFRST(base_addr) &= ~FEC_SW_RST;

    // Reset and disable FEC
    FEC_ECR(base_addr) = FEC_ECR_RESET;

    // Wait
#if TARGET_SYSTEM == _LINUX_
    udelay(10);
#else
    for(i = 0; i < 10; ++i)
    {
        asm("nop");
    }
#endif

    // Clear all events
    FEC_EIR(base_addr) = FEC_EIR_CLEAR;

    // Reset FIFO status
    FEC_FECTFSR(base_addr) = FEC_FECTFSR_MSK;
    FEC_FECRFSR(base_addr) = FEC_FECRFSR_MSK;

    // Set the default mac address
    FEC_PALR(base_addr) = (fp->fecpriv_mac_addr[0] << 24) | (fp->fecpriv_mac_addr[1] << 16) | (fp->fecpriv_mac_addr[2] << 8) | fp->fecpriv_mac_addr[3];
    FEC_PAUR(base_addr) = (fp->fecpriv_mac_addr[4] << 24) | (fp->fecpriv_mac_addr[5] << 16) | 0x8808;

    // Reset the group address descriptor
    FEC_GALR(base_addr) = 0x00000000;
    FEC_GAUR(base_addr) = 0x00000000;

    // Reset the individual address descriptor
    FEC_IALR(base_addr) = 0x00000000;
    FEC_IAUR(base_addr) = 0x00000000;

    // Set the receive control register
    //mb mit dem PROM-bit wird alles reingelassen
    //    FEC_RCR(base_addr) = FEC_RCR_MAX_FRM_SIZE | FEC_RCR_MII | FEC_RCR_PROM;
//    FEC_RCR(base_addr) = FEC_RCR_MAX_FRM_SIZE | FEC_RCR_MII;
    // generate BABR interrupt after 16th byte of frame
    FEC_RCR(base_addr) = (0 << 16) | FEC_RCR_MII;

    // Set the receive FIFO control register
    FEC_FECRFCR(base_addr) = FEC_FECRFCR_FRM | FEC_FECRFCR_GR
                             | (FEC_FECRFCR_MSK     // disable all but ...
                                & ~FEC_FECRFCR_FAE  // enable frame accept error
                                & ~FEC_FECRFCR_RXW  // enable receive wait condition
                                & ~FEC_FECRFCR_UF); // enable FIFO underflow

    //Set the receive FIFO alarm register
    FEC_FECRFAR(base_addr) = FEC_FECRFAR_ALARM;

    // Set the transmit FIFO control register
    FEC_FECTFCR(base_addr) = FEC_FECTFCR_FRM | FEC_FECTFCR_GR
                             | (FEC_FECTFCR_MSK     // disable all but ...
                                & ~FEC_FECTFCR_FAE  // enable frame accept error
//                                & ~FEC_FECTFCR_TXW  // enable transmit wait condition
                                & ~FEC_FECTFCR_UF   // enable FIFO underflow
                                & ~FEC_FECTFCR_OF); // enable FIFO overflow

    //Set the transmit FIFO alarm register
    FEC_FECTFAR(base_addr) = FEC_FECTFAR_ALARM;

    // Set the Tx FIFO watermark
    FEC_FECTFWR(base_addr) = FEC_FECTFWR_XWMRK;

    // Enable the transmitter to append the CRC
    FEC_CTCWR(base_addr) = FEC_CTCWR_TFCW_CRC;

#if (EDRV_USED_ETH_CTRL == 0)
    // Initialize the transceiver (KS8721) for FEC0
    if(ks8721_init_transceiver(base_addr, &fduplex))
    {
       goto ERRORS;
    }
#else
    // Initialize the transceiver (KSZ8893) for FEC1
    if(ksz8893_init_transceiver(base_addr, &fduplex))
    {
       goto ERRORS;
    }
#endif

    if (fduplex)
    {
        // Enable the full duplex mode
        FEC_TCR(base_addr)= FEC_TCR_FDEN |  FEC_TCR_HBC;
    }
    else
    {
        // Disable reception of frames while transmitting
        FEC_RCR(base_addr) |= FEC_RCR_DRT;
    }

    // Enable the ethernet interrupts
    FEC_EIMR(base_addr) = FEC_EIMR_DISABLE
                            | FEC_EIR_LC
                            | FEC_EIR_RL
                            | FEC_EIR_XFUN
                            | FEC_EIR_XFERR
                            | FEC_EIR_RFERR
#if EDRV_EARLY_RX_INT != FALSE
                            | FEC_EIR_BABR
#endif
#if EDRV_DMA_TX_HANDLER == FALSE
                            | FEC_EIR_TXF
#endif
                            ;

    // Enable FEC
    FEC_ECR(base_addr) |= FEC_ECR_ETHEREN;

    // Initialize the reception buffers
    for (i = 0; i < FEC_RX_BUF_NUMBER; i++)
    {
        fp->fecpriv_rxdesc[i].statCtrl = MCD_FEC_BUF_READY | MCD_FEC_INTERRUPT;
        fp->fecpriv_rxdesc[i].length = FEC_MAXBUF_SIZE;
//       fp->fecpriv_rxdesc[i].length = 16;
    }

    fp->fecpriv_rxdesc[i - 1].statCtrl |= MCD_FEC_WRAP;
    fp->fecpriv_current_rx = 0;

    // flush entire data cache before restarting the DMA
    DcacheFlushInvalidate();

    MCD_startDma(fp->fecpriv_fec_rx_channel, (s8*) fp->fecpriv_rxdesc, 0,
             (s8*) &(FEC_FECRFDR(base_addr)), 0,
//             16, 0, fp->fecpriv_initiator_rx,
             FEC_MAX_FRM_SIZE, 0, fp->fecpriv_initiator_rx,
             FEC_RX_DMA_PRI,
             MCD_FECRX_DMA | MCD_INTERRUPT | MCD_TT_FLAGS_DEF,
             MCD_NO_CSUM | MCD_NO_BYTE_SWAP);

    return OK;


ERRORS:

   // Remove the channels and return with the error
   if (fp->fecpriv_fec_rx_channel != -1)
   {
       dma_disconnect(fp->fecpriv_fec_rx_channel);
       dma_remove_channel_by_number(fp->fecpriv_fec_rx_channel);
       fp->fecpriv_fec_rx_channel = -1;
   }

   if (fp->fecpriv_fec_tx_channel != -1)
   {
       dma_disconnect(fp->fecpriv_fec_tx_channel);
       dma_remove_channel_by_number(fp->fecpriv_fec_tx_channel);
       fp->fecpriv_fec_tx_channel = -1;
   }

   return ERR;

}
/************************************************************************
* NAME: fec_stop_controller
*
* DESCRIPTION: This function performs the graceful stop of the
*                transmission and disables FEC
*
*************************************************************************/
void fec_stop_controller (struct eth_if *nd)
{

    //Receive the pointer to the private structure
    struct fec_priv *fp = (struct fec_priv *)nd->if_ptr;

    // Receive the base address
    unsigned long base_addr = (unsigned long)fp->base_addr;

#if TARGET_SYSTEM == _LINUX_
    unsigned long time;
#endif

    // Perform the graceful stop
    FEC_TCR(base_addr) |= FEC_TCR_GTS;

    // Wait for the graceful stop
#if TARGET_SYSTEM == _LINUX_
    time = jiffies;

    // Wait for the graceful stop
    while (!(FEC_EIR(base_addr) & FEC_EIR_GRA) && jiffies - time < FEC_GRA_TIMEOUT * HZ)
        schedule();
#else
    while (!(FEC_EIR(base_addr) & FEC_EIR_GRA));// && timer_get_interval(time, current_time) < FEC_GRA_TIMEOUT);
#endif

    // Disable FEC
    FEC_ECR(base_addr) = FEC_ECR_DISABLE;

    // disable DMA interrupts
#if (EDRV_USED_ETH_CTRL == 0)
    TgtEnableEthInterrupt0(FALSE, EPL_TGT_INTMASK_DMA);
#else
    TgtEnableEthInterrupt1(FALSE, EPL_TGT_INTMASK_DMA);
#endif

    // Reset the DMA channels
    MCD_killDma (fp->fecpriv_fec_rx_channel);

    // enable DMA interrupts
#if (EDRV_USED_ETH_CTRL == 0)
    TgtEnableEthInterrupt0(TRUE, EPL_TGT_INTMASK_DMA);
#else
    TgtEnableEthInterrupt1(TRUE, EPL_TGT_INTMASK_DMA);
#endif

    dma_remove_channel_by_number(fp->fecpriv_fec_rx_channel);
    dma_disconnect(fp->fecpriv_fec_rx_channel);
    fp->fecpriv_fec_rx_channel = -1;

    MCD_killDma (fp->fecpriv_fec_tx_channel);
    dma_remove_channel_by_number(fp->fecpriv_fec_tx_channel);
    dma_disconnect(fp->fecpriv_fec_tx_channel);
    fp->fecpriv_fec_tx_channel = -1;


    return;
}


/************************************************************************
* NAME: fec_read_mii
*
* DESCRIPTION: This function reads the value from the MII register
*
* RETURNS: If no error occurs, this function returns OK. Otherwise
*            it returns ERR
*************************************************************************/
int fec_read_mii(unsigned int base_addr, unsigned int pa, unsigned int ra, unsigned int *data)
{
#if TARGET_SYSTEM == _LINUX_
    unsigned long time;
#endif

    // Clear the MII interrupt bit
    FEC_EIR(base_addr) = FEC_EIR_MII;

    // Write to the MII management frame register
    FEC_MMFR(base_addr) = FEC_MMFR_READ | (pa << 23) | (ra << 18);

    // Wait for the data reading
#if TARGET_SYSTEM == _LINUX_
    time = jiffies;

    while (!(FEC_EIR(base_addr) & FEC_EIR_MII))
    {
        if (jiffies - time > FEC_MII_TIMEOUT * HZ)
            return -ETIME;
        schedule();
    }
#else
    while (!(FEC_EIR(base_addr) & FEC_EIR_MII));
#endif

    // Clear the MII interrupt bit
    FEC_EIR(base_addr) = FEC_EIR_MII;

    *data = FEC_MMFR(base_addr) & 0x0000FFFF;

    return OK;
}

/************************************************************************
* NAME: fec_write_mii
*
* DESCRIPTION: This function writes the value to the MII register
*
* RETURNS: If no error occurs, this function returns OK. Otherwise
*            it returns ERR
*************************************************************************/
int fec_write_mii(unsigned int base_addr, unsigned int pa, unsigned int ra, unsigned int data)
{
#if TARGET_SYSTEM == _LINUX_
    unsigned long time;
#endif

    // Clear the MII interrupt bit
    FEC_EIR(base_addr) = FEC_EIR_MII;

    // Write to the MII management frame register
    FEC_MMFR(base_addr) = FEC_MMFR_WRITE | (pa << 23) | (ra << 18) | data;

    // Wait for the writing
#if TARGET_SYSTEM == _LINUX_
    time = jiffies;

    while (!(FEC_EIR(base_addr) & FEC_EIR_MII))
    {
        if (jiffies - time > FEC_MII_TIMEOUT * HZ)
            return -ETIME;

        schedule();
    }
#else
    while (!(FEC_EIR(base_addr) & FEC_EIR_MII));
#endif

    // Clear the MII interrupt bit
    FEC_EIR(base_addr) = FEC_EIR_MII;

    return OK;
}

/************************************************************************
* NAME: fec_access_mii
*
* DESCRIPTION: This function accesses the MII management interface.
*              It can be used to access the registers of the Micrel KSZ8893
*              via its proprietary Serial Management Interface.
*
* RETURNS: If no error occurs, this function returns OK. Otherwise
*            it returns ERR
*************************************************************************/
int fec_access_mii(unsigned int base_addr, unsigned int *data)
{
#if TARGET_SYSTEM == _LINUX_
    unsigned long time;
#endif

    // Clear the MII interrupt bit
    FEC_EIR(base_addr) = FEC_EIR_MII;

    // Write to the MII management frame register
    FEC_MMFR(base_addr) = *data;

    // Wait for the data reading
#if TARGET_SYSTEM == _LINUX_
    time = jiffies;

    while (!(FEC_EIR(base_addr) & FEC_EIR_MII))
    {
        if (jiffies - time > FEC_MII_TIMEOUT * HZ)
            return -ETIME;
        schedule();
    }
#else
    while (!(FEC_EIR(base_addr) & FEC_EIR_MII));
#endif

    // Clear the MII interrupt bit
    FEC_EIR(base_addr) = FEC_EIR_MII;

    *data = FEC_MMFR(base_addr);

    return OK;
}


/************************************************************************
* NAME: EdrvInterruptHandler
*
* DESCRIPTION: This function is called when a FEC event occurs.
*
*************************************************************************/
void EdrvInterruptHandler (void)
{
struct fec_priv *   fp;
unsigned long       base_addr;
#if EDRV_EARLY_RX_INT != FALSE
tEdrvRxBuffer       RxBuffer;
#endif


    //Receive the pointer to the private structure
#if (EDRV_USED_ETH_CTRL == 0)
    fp = (struct fec_priv *)fec_dev_fec0.if_ptr;
#else
    fp = (struct fec_priv *)fec_dev_fec1.if_ptr;
#endif

    // Receive the base address
    base_addr = (unsigned long)fp->base_addr;

//    isr_lock();
#if EDRV_EARLY_RX_INT != FALSE
    if ((FEC_EIR(base_addr) & FEC_EIR_BABR) != 0)
    {
        // reset BABR event
        FEC_EIR(base_addr) = FEC_EIR_BABR;

#if EDRV_BENCHMARK != FALSE
        // set LED
        MCF_GPIO_PODR_PCIBR |= 0x02;  // Level
        //MCF_GPIO_PODR_PCIBG |= 0x10;  // Level
#endif

        // wait until first bytes of Rx buffer are filled by DMA
        // $$$ d.k.: use phys_to_virt() under Linux for access to dataPointer
        while (((BYTE *)fp->fecpriv_rxdesc[fp->fecpriv_current_rx].dataPointer)[14] == 0xFF)
        {
        }

        RxBuffer.m_BufferInFrame = kEdrvBufferFirstInFrame;
        RxBuffer.m_uiRxMsgLen = fp->fecpriv_rxdesc[fp->fecpriv_current_rx].length;
        RxBuffer.m_pbBuffer = (BYTE *)fp->fecpriv_rxdesc[fp->fecpriv_current_rx].dataPointer;

#if EDRV_BENCHMARK != FALSE
        // reset LED
        MCF_GPIO_PODR_PCIBR &= ~0x02;  // Level
        //MCF_GPIO_PODR_PCIBG &= ~0x10;  // Level
#endif

        fp->fecpriv_flags |= FEC_FLAGS_RCV_PROC;
        EdrvInitParam_l.m_pfnRxHandler (&RxBuffer);
        fp->fecpriv_flags &= ~FEC_FLAGS_RCV_PROC;
    }
#endif

#if EDRV_DMA_TX_HANDLER == FALSE
    if ((FEC_EIR(base_addr) & FEC_EIR_TXF) != 0)
    {
        // reset TXF event
        FEC_EIR(base_addr) = FEC_EIR_TXF;

        TGT_DBG_SIGNAL_TRACE_POINT(5);

        while (EdrvInstance_l.m_uiHeadTxDesc != EdrvInstance_l.m_uiTailTxDesc)
        {   // transmission is active
        tEdrvTxBuffer*  pTxBuffer;

#if EDRV_BENCHMARK != FALSE
            // reset LED
            MCF_GPIO_PODR_PCIBR &= ~0x01;  // Level
#endif

            pTxBuffer = EdrvInstance_l.m_apTxBuffer[EdrvInstance_l.m_uiHeadTxDesc];
            EdrvInstance_l.m_apTxBuffer[EdrvInstance_l.m_uiHeadTxDesc] = NULL;

            EdrvInstance_l.m_uiHeadTxDesc = (EdrvInstance_l.m_uiHeadTxDesc + 1) & FEC_TX_DESC_MASK;

            if (pTxBuffer != NULL)
            {
                if (pTxBuffer->m_pfnTxHandler != NULL)
                {
                    pTxBuffer->m_pfnTxHandler(pTxBuffer);
                }
            }
        }
    }
#endif

    // receive FIFO error
    if ((FEC_EIR(base_addr) & FEC_EIR_RFERR) != 0)
    {
        FEC_TRACE(FEC_FECRFSR(base_addr));

        // kill DMA receive channel
        MCD_killDma (fp->fecpriv_fec_rx_channel);

        // Reset FIFOs
        FEC_FECFRST(base_addr) |= FEC_SW_RST;
        FEC_FECFRST(base_addr) &= ~FEC_SW_RST;

        // check receive FIFO status register
        if ((FEC_FECRFSR(base_addr) & FEC_FECRFSR_FAE) != 0)
        {
            // reset frame accept error
            FEC_FECRFSR(base_addr) = FEC_FECRFSR_FAE;
            FEC_COUNT_RX_FAE;
        }
        if ((FEC_FECRFSR(base_addr) & FEC_FECRFSR_RXW) != 0)
        {
            // reset receive wait condition
            FEC_FECRFSR(base_addr) = FEC_FECRFSR_RXW;
            FEC_COUNT_RX_WAIT;
        }
        if ((FEC_FECRFSR(base_addr) & FEC_FECRFSR_UF) != 0)
        {
            // reset receive FIFO underflow
            FEC_FECRFSR(base_addr) = FEC_FECRFSR_UF;
            FEC_COUNT_RX_FUF;
        }

        // reset RFERR event
        FEC_EIR(base_addr) = FEC_EIR_RFERR;

        // mark current buffer as empty
        fp->fecpriv_rxdesc[fp->fecpriv_current_rx].statCtrl = MCD_FEC_BUF_READY | MCD_FEC_INTERRUPT;

        // mark last buffer as wrap around
        fp->fecpriv_rxdesc[(FEC_RX_BUF_NUMBER - 1)].statCtrl |= MCD_FEC_WRAP;

        fp->fecpriv_rxdesc[fp->fecpriv_current_rx].length = FEC_MAXBUF_SIZE;

        // restart DMA from beginning
        fp->fecpriv_current_rx = 0;

        // flush entire data cache before restarting the DMA
        DcacheFlushInvalidate();

        MCD_startDma(fp->fecpriv_fec_rx_channel, (s8*) fp->fecpriv_rxdesc, 0,
                 (s8*) &(FEC_FECRFDR(base_addr)), 0,
                 FEC_MAX_FRM_SIZE, 0, fp->fecpriv_initiator_rx,
                 FEC_RX_DMA_PRI,
                 MCD_FECRX_DMA | MCD_INTERRUPT | MCD_TT_FLAGS_DEF,
                 MCD_NO_CSUM | MCD_NO_BYTE_SWAP);

        // Enable FEC
        FEC_ECR(base_addr) |= FEC_ECR_ETHEREN;

    }

    // transmit FIFO error
    if ((FEC_EIR(base_addr) & FEC_EIR_XFERR) != 0)
    {
        FEC_TRACE(FEC_FECTFSR(base_addr));

        // kill running transmission by DMA
        MCD_killDma (fp->fecpriv_fec_tx_channel);

        // Reset FIFOs
        FEC_FECFRST(base_addr) |= FEC_SW_RST;
        FEC_FECFRST(base_addr) &= ~FEC_SW_RST;

        // check transmit FIFO status register
        if ((FEC_FECTFSR(base_addr) & FEC_FECTFSR_FAE) != 0)
        {
            // reset frame accept error
            FEC_FECTFSR(base_addr) = FEC_FECTFSR_FAE;
            FEC_COUNT_TX_FAE;
        }
        if ((FEC_FECTFSR(base_addr) & FEC_FECTFSR_TXW) != 0)
        {
            // reset transmit wait condition
            FEC_FECRFSR(base_addr) = FEC_FECTFSR_TXW;
            FEC_COUNT_TX_WAIT;
        }
        if ((FEC_FECTFSR(base_addr) & FEC_FECTFSR_UF) != 0)
        {
            // reset transmit FIFO underflow
            FEC_FECTFSR(base_addr) = FEC_FECTFSR_UF;
            FEC_COUNT_TX_FUF;
        }
        if ((FEC_FECTFSR(base_addr) & FEC_FECTFSR_OF) != 0)
        {
            // reset transmit FIFO overflow
            FEC_FECTFSR(base_addr) = FEC_FECTFSR_OF;
            FEC_COUNT_TX_FOF;
        }

        // $$$ d.k.: inform DLL about dropped frame

        // reset XFERR event
        FEC_EIR(base_addr) = FEC_EIR_XFERR;

        // Enable FEC
        FEC_ECR(base_addr) |= FEC_ECR_ETHEREN;
    }

    // transmit FIFO underrun
    if ((FEC_EIR(base_addr) & FEC_EIR_XFUN) != 0)
    {
        // reset XFUN event
        FEC_EIR(base_addr) = FEC_EIR_XFUN;
        FEC_COUNT_TX_FUN;
    }

    // late collision
    if ((FEC_EIR(base_addr) & FEC_EIR_LC) != 0)
    {
        // reset LC event
        FEC_EIR(base_addr) = FEC_EIR_LC;
        FEC_COUNT_LATECOLLISION;
    }

    // collision retry limit
    if ((FEC_EIR(base_addr) & FEC_EIR_RL) != 0)
    {
        // reset RL event
        FEC_EIR(base_addr) = FEC_EIR_RL;
        FEC_COUNT_TX_COL_RL;
    }

//    isr_unlock();
}


/************************************************************************
* NAME: fec_interrupt_tx_handler
*
* DESCRIPTION: This function is called when the data
*              transmission from the buffer to the FEC is completed.
*
*************************************************************************/
void fec_interrupt_fec_tx_handler(struct eth_if *nd)
{

    TGT_DBG_SIGNAL_TRACE_POINT(9);

#if EDRV_DMA_TX_HANDLER != FALSE
    while (EdrvInstance_l.m_uiHeadTxDesc != EdrvInstance_l.m_uiTailTxDesc)
    {   // transmission is active
    tEdrvTxBuffer*  pTxBuffer;

#if EDRV_BENCHMARK != FALSE
        // reset LED
        MCF_GPIO_PODR_PCIBR &= ~0x01;  // Level
#endif

        pTxBuffer = EdrvInstance_l.m_apTxBuffer[EdrvInstance_l.m_uiHeadTxDesc];
        EdrvInstance_l.m_apTxBuffer[EdrvInstance_l.m_uiHeadTxDesc] = NULL;

        EdrvInstance_l.m_uiHeadTxDesc = (EdrvInstance_l.m_uiHeadTxDesc + 1) & FEC_TX_DESC_MASK;

        if (pTxBuffer != NULL)
        {
            if (pTxBuffer->m_pfnTxHandler != NULL)
            {
                pTxBuffer->m_pfnTxHandler(pTxBuffer);
            }
        }
    }
#endif
}

/************************************************************************
* NAME: fec_interrupt_rx_handler
*
* DESCRIPTION: This function is called when the data
*              reception from the FEC to the reception buffer is completed.
*
*************************************************************************/
void fec_interrupt_fec_rx_handler(struct eth_if *nd)
{
tEdrvRxBuffer   RxBuffer;

    //Receive the pointer to the private structure
    struct fec_priv *fp = (struct fec_priv *)nd->if_ptr;

#if EDRV_EARLY_RX_INT != FALSE
    int i;
#endif

#if EDRV_BENCHMARK != FALSE
    // set LED
    MCF_GPIO_PODR_PCIBR |= 0x02;  // Level
    //MCF_GPIO_PODR_PCIBG |= 0x04;  // Level
#endif
    TGT_DBG_SIGNAL_TRACE_POINT(6);

#if EDRV_EARLY_RX_INT != FALSE
    // Set the FEC_FLAGS_RCV_PROC flag. This flag allows to avoid the new call of this handler while the frame processing is not done
//    isr_lock();
    if(fp->fecpriv_flags & FEC_FLAGS_RCV_PROC)
    {
//        isr_unlock();
        return;
    }

    fp->fecpriv_flags |= FEC_FLAGS_RCV_PROC;
//    isr_unlock();
#endif

    // Process all received frames
    for(; fp->fecpriv_rxdesc[fp->fecpriv_current_rx].statCtrl & MCD_FEC_END_FRAME; fp->fecpriv_current_rx = (fp->fecpriv_current_rx + 1) & FEC_RX_INDEX_MASK)
    {

//        isr_lock();

        // before forwarding the Rx event, check if Tx finished,
        // so Tx and Rx frames are processed in the right order
        while (EdrvInstance_l.m_uiHeadTxDesc != EdrvInstance_l.m_uiTailTxDesc)
        {   // transmission is active
            if ((EdrvInstance_l.m_pTxDesc[EdrvInstance_l.m_uiHeadTxDesc].statCtrl & MCD_FEC_BUF_READY) == 0)
            {   // transmission has finished
            tEdrvTxBuffer*  pTxBuffer;

#if EDRV_BENCHMARK != FALSE
                // reset LED
                MCF_GPIO_PODR_PCIBR &= ~0x01;  // Level
#endif

                pTxBuffer = EdrvInstance_l.m_apTxBuffer[EdrvInstance_l.m_uiHeadTxDesc];
                EdrvInstance_l.m_apTxBuffer[EdrvInstance_l.m_uiHeadTxDesc] = NULL;

                EdrvInstance_l.m_uiHeadTxDesc = (EdrvInstance_l.m_uiHeadTxDesc + 1) & FEC_TX_DESC_MASK;

                if (pTxBuffer->m_pfnTxHandler != NULL)
                {
                    pTxBuffer->m_pfnTxHandler(pTxBuffer);
                }
            }
            else
            {
                break;
            }
        }


        RxBuffer.m_BufferInFrame = kEdrvBufferLastInFrame;
        RxBuffer.m_uiRxMsgLen = fp->fecpriv_rxdesc[fp->fecpriv_current_rx].length - 4;
#if TARGET_SYSTEM == _LINUX_
        RxBuffer.m_pbBuffer = (BYTE *) phys_to_virt(fp->fecpriv_rxdesc[fp->fecpriv_current_rx].dataPointer);
#else
        RxBuffer.m_pbBuffer = (BYTE *)fp->fecpriv_rxdesc[fp->fecpriv_current_rx].dataPointer;
#endif
        EdrvInitParam_l.m_pfnRxHandler (&RxBuffer);

        // flush data cache before initializing the descriptor and starting DMA
        DcacheFlushInvalidateCacheBlock((void*)fp->fecpriv_rxdesc[fp->fecpriv_current_rx].dataPointer, FEC_MAXBUF_SIZE);

#if EDRV_EARLY_RX_INT != FALSE
        // reset receive buffer for BABR interrupt
        for (i = 0; i < 16; i++)
        {
            RxBuffer.m_pbBuffer[i] = 0xFF;
        }
#endif

//        isr_unlock();

        // Free the reception buffer
        fp->fecpriv_rxdesc[fp->fecpriv_current_rx].length = FEC_MAXBUF_SIZE;
        fp->fecpriv_rxdesc[fp->fecpriv_current_rx].statCtrl &= ~MCD_FEC_END_FRAME;
        fp->fecpriv_rxdesc[fp->fecpriv_current_rx].statCtrl |= MCD_FEC_BUF_READY;
    }

#if EDRV_EARLY_RX_INT != FALSE
    fp->fecpriv_flags &= ~FEC_FLAGS_RCV_PROC;

//    isr_unlock();
#endif

    // Tell the DMA to continue the reception
    MCD_continDma(fp->fecpriv_fec_rx_channel);

#if EDRV_BENCHMARK != FALSE
    // reset LED
    MCF_GPIO_PODR_PCIBR &= ~0x02;  // Level
    //MCF_GPIO_PODR_PCIBG &= ~0x04;  // Level
#endif
}

#if (EDRV_USED_ETH_CTRL == 0)
/************************************************************************
* NAME: fec_interrupt_tx_handler_fec0
*
* DESCRIPTION: This is the DMA interrupt handler using  for FEC0
*              transmission.
*
*************************************************************************/
void fec_interrupt_fec_tx_handler_fec0(void)
{
    fec_interrupt_fec_tx_handler(&fec_dev_fec0);
}

/************************************************************************
* NAME: fec_interrupt_rx_handler_fec0
*
* DESCRIPTION: This is the DMA interrupt handler using for the FEC0
*              reception.
*
*************************************************************************/
void fec_interrupt_fec_rx_handler_fec0(void)
{
    fec_interrupt_fec_rx_handler(&fec_dev_fec0);
}

#else
/************************************************************************
* NAME: fec_interrupt_tx_handler_fec1
*
* DESCRIPTION: This is the DMA interrupt handler using for the FEC1
*              transmission.
*
*************************************************************************/
void fec_interrupt_fec_tx_handler_fec1(void)
{
    fec_interrupt_fec_tx_handler(&fec_dev_fec1);
}

/************************************************************************
* NAME: fec_interrupt_rx_handler_fec1
*
* DESCRIPTION: This is the DMA interrupt handler using for the FEC1
*              reception.
*
*************************************************************************/
void fec_interrupt_fec_rx_handler_fec1(void)
{
    fec_interrupt_fec_rx_handler(&fec_dev_fec1);
}
#endif


//---------------------------------------------------------------------------
//
// Function:     EdrvCalcHash
//
// Description:  function calculates the entry for the hash-table from MAC
//               address
//
// Parameters:   pbMAC_p - pointer to MAC address
//
// Returns:      hash value
//
// State:
//
//---------------------------------------------------------------------------
#define HASH_BITS              6  // used bits in hash
#define CRC32_POLY    0xEDB88320  //
// G(x) = x32 + x26 + x23 + x22 + x16 + x12 + x11 + x10 + x8 + x7 + x5 + x4 + x2 + x + 1

BYTE EdrvCalcHash (BYTE * pbMAC_p)
{
DWORD dwByteCounter;
DWORD dwBitCounter;
DWORD dwData;
DWORD dwCrc;
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
            dwCrc = (dwCrc >> 1) ^ (((dwCrc ^ dwData) & 1) ? CRC32_POLY : 0);
        }
    }

    // only upper 6 bits (HASH_BITS) are used
    // which point to specific bit in he hash registers
    bHash = (BYTE)((dwCrc >> (32 - HASH_BITS)) & 0x3f);

    return bHash;
}

