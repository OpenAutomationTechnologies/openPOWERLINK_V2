/*----------------------------------------------------------------------------
Copyright (c) 2009, B&R
Copyright (c) 2009, SYSTEC electronic GmbH
All rights reserved.

Redistribution and use in source and binary forms,
with or without modification,
are permitted provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright notice,
      this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright notice,
      this list of conditions and the following disclaimer
      in the documentation and/or other materials provided with the distribution.
    * Neither the name of the B&R nor the names of its contributors
      may be used to endorse or promote products derived from this software
      without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
------------------------------------------------------------------------------

  Project:          openPOWERLINK with openMAC

  Description:      Ethernet Driver for openMAC

  $Revision: 1.10 $  $Date: 2009/09/25 11:28:20 $

  Author:           Joerg Zelenka (zelenkaj)
                    D. Krueger, completely reworked

  State:            tested on openPOWERLINK V1.5.9

------------------------------------------------------------------------------
  History

 2009/03/24     zelenkaj    V1.0 implementation finished
 2009/03/26     zelenkaj    revised
 2009/08/14     d.k.        adapted to new Ethernet driver interface
 2009/11/23		zelenkaj	adapted for Xilinx Microblaze
 2010/05/03     zelenkaj    adapted to new MAC-internal packet buffer
 2010/07/12     hoggerm     adapted for auto response delay

----------------------------------------------------------------------------*/


#include "global.h"
#include "EplInc.h"
#include "edrv.h"
#include "Benchmark.h"

#include "omethlib.h"   // openMAC header

#include "EplTgtTimeStamp_openMac.h"
#ifdef __NIOS2__
#include <sys/alt_cache.h>
#include <sys/alt_irq.h>
#include <alt_types.h>
#include "system.h"     // FPGA system definitions
#endif

#ifdef __MICROBLAZE__
#include "xparameters.h"
#include "stdio.h"
#include "xutil.h"
#include "IRQ_Microblaze.h"
#define     u8      Xuint8
#define     u16     Xuint16
#define     u32     Xuint32
#endif


//---------------------------------------------------------------------------
// defines
//---------------------------------------------------------------------------

//------------------------------------------------------
//--- set the system's base addresses ---
#ifdef __NIOS2__
//default settings for Nios 2
#define EDRV_MAC_BASE                   (void *)POWERLINK_0_MAC_REG_BASE
#define EDRV_MAC_SPAN                           POWERLINK_0_MAC_REG_SPAN
#define EDRV_MAC_IRQ                            POWERLINK_0_MAC_REG_IRQ
#define EDRV_RAM_BASE                   (void *)(EDRV_MAC_BASE + 0x0800)
#define EDRV_MII_BASE                   (void *)(EDRV_MAC_BASE + 0x1000)
#define EDRV_IRQ_BASE                   (void *)(EDRV_MAC_BASE + 0x1010)
#define EDRV_PKT_BASE                   (void *)POWERLINK_0_MAC_BUF_BASE
#define EDRV_PKT_SPAN                           POWERLINK_0_MAC_BUF_SPAN
#define EDRV_CMP_BASE                   (void *)POWERLINK_0_MAC_CMP_BASE
#define EDRV_CMP_SPAN                           POWERLINK_0_MAC_CMP_SPAN
#endif

#ifdef __MICROBLAZE__
//default settings for Microblaze
#define EDRV_MAC_BASE                   (void *)XPAR_XIL_OPENMAC_0_MEM0_BASEADDR
#define EDRV_MII_BASE                   (void *)(XPAR_XIL_OPENMAC_0_MEM0_BASEADDR+0x2000)
#define EDRV_RAM_BASE 					(void *)(EDRV_MAC_BASE + 0x1000)
#define EDRV_PKT_BASE                   (void *)XPAR_PACKETS_CNTLR_0_BASEADDR
#define EDRV_PKT_SPAN                           (XPAR_PACKETS_CNTLR_0_HIGHADDR - XPAR_PACKETS_CNTLR_0_BASEADDR)
#endif


//--- set driver's MTU ---
#define EDRV_MAX_BUFFER_SIZE        1518

//--- set driver's RX buffers ---
#define EDRV_MAX_RX_BUFFERS         6 //16

//--- set driver's filters ---
#define EDRV_MAX_FILTERS            16

//--- set driver's auto-response frames ---
#define EDRV_MAX_AUTO_RESPONSES     14


#if (EDRV_MAX_RX_BUFFERS > 16)
	#error This MAC version can handle 16 rx buffers, not more!
#endif

//MAC-internal buffer size estimation
//#if ( ((EDRV_MAX_BUFFER_SIZE+16)*EDRV_MAX_RX_BUFFERS) > EDRV_PKT_LENGTH )
//    #error The MAC-internal buffer size is to small!
//#endif //TODO: MAC Buffer big enough?

#if (EDRV_AUTO_RESPONSE == FALSE)
    #error Please enable EDRV_AUTO_RESPONSE in EplCfg.h!
#endif


// borrowed from omethlibint.h
#define GET_TYPE_BASE(typ, element, ptr)    \
    ((typ*)( ((size_t)ptr) - (size_t)&((typ*)0)->element ))


//---------------------------------------------------------------------------
// local types
//---------------------------------------------------------------------------

typedef struct _tEdrvInstance
{
    //EPL spec
    tEdrvInitParam          m_InitParam;

    //openMAC HAL Ethernet Driver
    ometh_config_typ        m_EthConf;
    OMETH_H                 m_hOpenMac;
    OMETH_HOOK_H            m_hHook;
    OMETH_FILTER_H          m_ahFilter[EDRV_MAX_FILTERS];

    phy_reg_typ*            m_pPhy;

    // auto-response Tx buffers
    tEdrvTxBuffer*          m_apTxBuffer[EDRV_MAX_FILTERS];

    //tx msg counter
    DWORD                     msgfree;
    DWORD                     msgsent;
    //needed for buffer management
    void                    *pRxBufBase;
    void                    *pTxBufBase;
    BYTE                    txBufferCnt;

} tEdrvInstance;

//---------------------------------------------------------------------------
// prototypes
//---------------------------------------------------------------------------


// RX Hook function
static int EdrvRxHook(void *arg, ometh_packet_typ  *pPacket, OMETH_BUF_FREE_FCT  *pFct);

static void EdrvCbSendAck(ometh_packet_typ *pPacket, void *arg, unsigned long time);

#ifdef __NIOS2__
static void EdrvIrqHandler (void* pArg_p, alt_u32 dwInt_p);
#endif

#ifdef __MICROBLAZE__
static void EdrvRxInterruptHandler (void* pArg_p);
static void EdrvTxInterruptHandler (void* pArg_p);
#endif

#if defined(__NIOS2__)
#define OMETH_MII_CFG_DELAY for(i=0;i<1000;i++) asm("NOP;")
#elif defined(__MICROBLAZE__)
#define OMETH_MII_CFG_DELAY for(i=0;i<100000;i++) asm("NOP;")
#else
#define OMETH_MII_CFG_DELAY 		
#endif
//---------------------------------------------------------------------------
// module globale vars
//---------------------------------------------------------------------------


static tEdrvInstance EdrvInstance_l;



//=========================================================================//
//                                                                         //
//          P U B L I C   F U N C T I O N S                                //
//                                                                         //
//=========================================================================//

//---------------------------------------------------------------------------
//
// Function:    EdrvInit
//
// Description:
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
tEplKernel      Ret = kEplSuccessful;
int             iRet;
int             i;
BYTE            abFilterMask[31],
                abFilterValue[31];


    PRINTF0("initalize Ethernet Driver for openMAC\n");
    memset(&EdrvInstance_l, 0, sizeof(EdrvInstance_l)); //reset driver struct
    memset(EDRV_PKT_BASE, 0, EDRV_PKT_SPAN); //reset MAC-internal buffers

    EdrvInstance_l.m_InitParam = *pEdrvInitParam_p;

    printf("MAC adr: %.2X-%.2X-%.2X-%.2X-%.2X-%.2X\n",
        EdrvInstance_l.m_InitParam.m_abMyMacAddr[0],
        EdrvInstance_l.m_InitParam.m_abMyMacAddr[1],
        EdrvInstance_l.m_InitParam.m_abMyMacAddr[2],
        EdrvInstance_l.m_InitParam.m_abMyMacAddr[3],
        EdrvInstance_l.m_InitParam.m_abMyMacAddr[4],
        EdrvInstance_l.m_InitParam.m_abMyMacAddr[5]);

    PRINTF0("init openMAC...");

    ////////////////////
    // initialize phy //
    ////////////////////
    EdrvInstance_l.m_pPhy = omethPhyInfo(EdrvInstance_l.m_hOpenMac, 0);
    PRINTF0("Phy activation ... ");
    omethMiiControl(EDRV_MII_BASE, MII_CTRL_RESET);
    OMETH_MII_CFG_DELAY;
    omethMiiControl(EDRV_MII_BASE, MII_CTRL_ACTIVE);
    OMETH_MII_CFG_DELAY;
    omethMiiControl(EDRV_MII_BASE, MII_CTRL_RESET);
    OMETH_MII_CFG_DELAY;
    omethMiiControl(EDRV_MII_BASE, MII_CTRL_ACTIVE);
    OMETH_MII_CFG_DELAY;
    PRINTF0("done\n");

    ////////////////////////////////
    // initialize ethernet driver //
    ////////////////////////////////
    omethInit();

    EdrvInstance_l.m_EthConf.adapter = 0; //adapter number
    EdrvInstance_l.m_EthConf.macType = OMETH_MAC_TYPE_01;    // more info in omethlib.h
    EdrvInstance_l.m_EthConf.mode = /*OMETH_MODE_FULLDUPLEX +*/
                       OMETH_MODE_HALFDUPLEX;   // supported modes
#ifdef __NIOS2__
    alt_remap_uncached((void*)EDRV_MAC_BASE, EDRV_MAC_SPAN);
    alt_remap_uncached((void*)EDRV_CMP_BASE, EDRV_CMP_SPAN);

    EdrvInstance_l.m_EthConf.pPhyBase = EDRV_MII_BASE;
    EdrvInstance_l.m_EthConf.pRamBase = EDRV_RAM_BASE;
    EdrvInstance_l.m_EthConf.pRegBase = EDRV_MAC_BASE;
	EdrvInstance_l.m_EthConf.pBufBase = (void*) alt_remap_uncached(EDRV_PKT_BASE, EDRV_PKT_SPAN);
#endif

#ifdef __MICROBLAZE__
	//Mac's address range should be set as uncached in EDK!
    EdrvInstance_l.m_EthConf.pPhyBase = (void*) EDRV_MII_BASE;
    EdrvInstance_l.m_EthConf.pRamBase = (void*) EDRV_RAM_BASE;
    EdrvInstance_l.m_EthConf.pRegBase = (void*) EDRV_MAC_BASE;
	EdrvInstance_l.m_EthConf.pBufBase = (void*) EDRV_PKT_BASE;
#endif

    EdrvInstance_l.m_EthConf.rxBuffers = EDRV_MAX_RX_BUFFERS;
    EdrvInstance_l.m_EthConf.rxMtu = EDRV_MAX_BUFFER_SIZE;

    EdrvInstance_l.m_hOpenMac = omethCreate(&EdrvInstance_l.m_EthConf);

    if (EdrvInstance_l.m_hOpenMac == 0)
    {
        Ret = kEplNoResource;
        PRINTF0(" error!\n");
        goto Exit;
    }

    //get rx/tx buffer base
    EdrvInstance_l.pRxBufBase = omethGetRxBufBase(EdrvInstance_l.m_hOpenMac);
    EdrvInstance_l.pTxBufBase = omethGetTxBufBase(EdrvInstance_l.m_hOpenMac);

    //init driver struct
    EdrvInstance_l.msgfree = 0;
    EdrvInstance_l.msgsent = 0;

    // initialize the filters, so that they won't match any normal Ethernet frame
    EPL_MEMSET(abFilterMask, 0, sizeof(abFilterMask));
    EPL_MEMSET(abFilterMask, 0xFF, 6);
    EPL_MEMSET(abFilterValue, 0, sizeof(abFilterValue));

    // initialize RX hook
    EdrvInstance_l.m_hHook = omethHookCreate(EdrvInstance_l.m_hOpenMac, EdrvRxHook, 0); //last argument max. pending
    if (EdrvInstance_l.m_hHook == 0)
    {
        Ret = kEplNoResource;
        goto Exit;
    }

    for (i = 0; i < EDRV_MAX_FILTERS; i++)
    {
        EdrvInstance_l.m_ahFilter[i] = omethFilterCreate(EdrvInstance_l.m_hHook, (void*) i, abFilterMask, abFilterValue);
        if (EdrvInstance_l.m_ahFilter[i] == 0)
        {
            Ret = kEplNoResource;
            goto Exit;
        }

        omethFilterDisable(EdrvInstance_l.m_ahFilter[i]);

        if (i < EDRV_MAX_AUTO_RESPONSES)
        {
            // initialize the auto response for each filter ...
            iRet = omethResponseInit(EdrvInstance_l.m_ahFilter[i]);
            if (iRet != 0)
            {
                Ret = kEplNoResource;
                goto Exit;
            }

            // ... but disable it
            omethResponseDisable(EdrvInstance_l.m_ahFilter[i]);
        }
    }

    // $$$ d.k. additional Filters for own MAC address and broadcast MAC
    //          will be necessary, if Virtual Ethernet driver is available

    ///////////////////////////
    // start Ethernet Driver //
    ///////////////////////////
    omethStart(EdrvInstance_l.m_hOpenMac, TRUE);
    PRINTF0("Ethernet Driver started\n");

#ifdef __NIOS2__
    ////////////////////
    // link NIOS' irq //
    ////////////////////
// register openMAC Rx and Tx IRQ
    if (alt_irq_register(EDRV_MAC_IRQ, EdrvInstance_l.m_hOpenMac, EdrvIrqHandler))
    {
        Ret = kEplNoResource;
    }
#endif

#ifdef __MICROBLAZE__
	{
    	XIntc *Intc = getIntc(); //get Interrupt Controller Instance
    	XStatus Status = 0;
		
		PRINTF0("Connect MAC's IRQ to Interrupt Controller:\n");
		Status = XIntc_Connect(	Intc, 
								XPAR_XPS_INTC_0_XIL_OPENMAC_0_RX_IR_N_INTR, 
								(XInterruptHandler)EdrvRxInterruptHandler, 
								EdrvInstance_l.m_hOpenMac);
		if(Status != XST_SUCCESS)
		{
			Ret = kEplNoResource;
			goto Exit;
		}
		
		Status = XIntc_Connect(	Intc, 
								XPAR_XPS_INTC_0_XIL_OPENMAC_0_TX_IR_N_INTR, 
								(XInterruptHandler)EdrvTxInterruptHandler, 
								EdrvInstance_l.m_hOpenMac);
		
		if(Status != XST_SUCCESS)
		{
			Ret = kEplNoResource;
			goto Exit;
		}
		
		PRINTF0("done!\n");
	}
#endif

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
    PRINTF0("Shutdown Ethernet Driver... ");

    omethStop(EdrvInstance_l.m_hOpenMac);
#ifdef __NIOS2__
// deregister openMAC Rx and Tx IRQ
	alt_irq_register(EDRV_MAC_IRQ, NULL, NULL);
#endif

#ifdef __MICROBLAZE__
	{
    	XIntc *Intc = getIntc(); //get Interrupt Controller Instance
		
		XIntc_Disconnect(Intc, XPAR_XPS_INTC_0_XIL_OPENMAC_0_RX_IR_N_INTR);
		XIntc_Disconnect(Intc, XPAR_XPS_INTC_0_XIL_OPENMAC_0_TX_IR_N_INTR);

	}
#endif

    if (omethDestroy(EdrvInstance_l.m_hOpenMac) != 0) {
        PRINTF0("error\n");
        return kEplNoResource;
    }
    PRINTF0("done\n");

    return kEplSuccessful;
}


//---------------------------------------------------------------------------
//
// Function:    EdrvAllocTxMsgBuffer
//
// Description:
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
tEplKernel          Ret = kEplSuccessful;
ometh_packet_typ*   pPacket = NULL;

    if (pBuffer_p->m_uiMaxBufferLen > EDRV_MAX_BUFFER_SIZE)
    {
        Ret = kEplEdrvNoFreeBufEntry;
        goto Exit;
    }

    // malloc aligns each allocated buffer so every type fits into this buffer.
    // this means 8 Byte alignment.
    //pPacket = (ometh_packet_typ*) alt_uncached_malloc(pBuffer_p->m_uiMaxBufferLen + sizeof (pPacket->length));
    {
        static void *pNextBuffer = NULL;
        if ( EdrvInstance_l.txBufferCnt == 0 ) //Edrv instance stores tx buffer numbers
            pPacket = (ometh_packet_typ*) EdrvInstance_l.pTxBufBase; //no tx buffers set
        else
            pPacket = (ometh_packet_typ*)pNextBuffer; //one/more buffers set

        //set new buffer to zeros (on-chip mem cont. is unknown!)
        memset(pPacket, 0, (pBuffer_p->m_uiMaxBufferLen + sizeof (pPacket->length))); //set zeros

        //calc next buffer for next allocation
        pNextBuffer = (((void*)pPacket) + (pBuffer_p->m_uiMaxBufferLen + sizeof (pPacket->length)));
        //align buffer
        {   DWORD tmp = (DWORD)pNextBuffer;
            tmp += 3; tmp &= ~3;
            pNextBuffer = (void*)tmp;
        }
        if ( (void*)pNextBuffer > EDRV_PKT_SPAN + EdrvInstance_l.pRxBufBase )
        {   //mac-internal buffer overflow
            printf("MAC-internal buffer overflow!\n");
            Ret = kEplEdrvNoFreeBufEntry;
            goto Exit;
        }
        EdrvInstance_l.txBufferCnt++; //add new buffer to cntr
    }
    if (pPacket == NULL)
    {
        Ret = kEplEdrvNoFreeBufEntry;
        goto Exit;
    }

    pPacket->length = pBuffer_p->m_uiMaxBufferLen;

    pBuffer_p->m_BufferNumber.m_dwVal = EDRV_MAX_FILTERS;

    pBuffer_p->m_pbBuffer = (BYTE*) &pPacket->data;

Exit:
    return Ret;
}


//---------------------------------------------------------------------------
//
// Function:    EdrvReleaseTxMsgBuffer
//
// Description:
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
tEplKernel          Ret = kEplSuccessful;
ometh_packet_typ*   pPacket = NULL;

    if (pBuffer_p->m_BufferNumber.m_dwVal < EDRV_MAX_FILTERS)
    {
        // disable auto-response
        omethResponseDisable(EdrvInstance_l.m_ahFilter[pBuffer_p->m_BufferNumber.m_dwVal]);
    }

    if (pBuffer_p->m_pbBuffer == NULL)
    {
        Ret = kEplEdrvInvalidParam;
        goto Exit;
    }

    pPacket = GET_TYPE_BASE(ometh_packet_typ, data, pBuffer_p->m_pbBuffer);

    // mark buffer as free, before actually freeing it
    pBuffer_p->m_pbBuffer = NULL;

    //free tx buffer
    // buffers are not in heap, decr Edrvinstance buffer cnt
    EdrvInstance_l.txBufferCnt--;//alt_uncached_free(pPacket);

Exit:
    return Ret;
}


//---------------------------------------------------------------------------
//
// Function:    EdrvUpdateTxMsgBuffer
//
// Description: Update tx-message buffer for use with auto-response filter
//
// Parameters:  pBuffer_p   = pointer to Buffer structure
//
// Returns:     Errorcode   = kEplSuccessful
//
// State:
//
//---------------------------------------------------------------------------

tEplKernel EdrvUpdateTxMsgBuffer     (tEdrvTxBuffer * pBuffer_p)
{
tEplKernel          Ret = kEplSuccessful;
ometh_packet_typ*   pPacket = NULL;

    if (pBuffer_p->m_BufferNumber.m_dwVal >= EDRV_MAX_FILTERS)
    {
        Ret = kEplEdrvInvalidParam;
        goto Exit;
    }

    pPacket = GET_TYPE_BASE(ometh_packet_typ, data, pBuffer_p->m_pbBuffer);

    pPacket->length = pBuffer_p->m_uiTxMsgLen;

	// Update autoresponse buffer
    EdrvInstance_l.m_apTxBuffer[pBuffer_p->m_BufferNumber.m_dwVal] = pBuffer_p;

    pPacket = omethResponseSet(EdrvInstance_l.m_ahFilter[pBuffer_p->m_BufferNumber.m_dwVal], pPacket);
    if (pPacket == OMETH_INVALID_PACKET)
    {
        Ret = kEplNoResource;
        goto Exit;
    }

Exit:
    return Ret;
}


//---------------------------------------------------------------------------
//
// Function:    EdrvSendTxMsg
//
// Description:
//
// Parameters:  pBuffer_p   = buffer descriptor to transmit
//
// Returns:     Errorcode   = kEplSuccessful
//                          = kEplEdrvNoFreeBufEntry
//
// State:
//
//---------------------------------------------------------------------------

tEplKernel EdrvSendTxMsg              (tEdrvTxBuffer * pBuffer_p)
{
tEplKernel          Ret = kEplSuccessful;
ometh_packet_typ*   pPacket = NULL;
unsigned long       ulTxLength;

    if (pBuffer_p->m_BufferNumber.m_dwVal < EDRV_MAX_FILTERS)
    {
        Ret = kEplEdrvInvalidParam;
        goto Exit;
    }

    pPacket = GET_TYPE_BASE(ometh_packet_typ, data, pBuffer_p->m_pbBuffer);

    pPacket->length = pBuffer_p->m_uiTxMsgLen;

    ulTxLength = omethTransmitArg(EdrvInstance_l.m_hOpenMac, pPacket,
                        EdrvCbSendAck, pBuffer_p);
    if (ulTxLength > 0)
    {
        EdrvInstance_l.msgsent++;
        Ret = kEplSuccessful;
    }
    else
    {
        Ret = kEplEdrvNoFreeBufEntry;
    }

Exit:
    return Ret;
}


//---------------------------------------------------------------------------
//
// Function:    EdrvChangeFilter
//
// Description: Change all rx-filters or one specific rx-filter
//              of the openMAC
//
// Parameters:  pFilter_p           = pointer to array of filter entries
//              uiCount_p           = number of filters in array
//              uiEntryChanged_p    = selects one specific filter which is
//                                    to be changed. If value is equal to
//                                    or larger than uiCount_p, all entries
//                                    are selected.
//              uiChangeFlags_p     = If one specific entry is selected,
//                                    these flag bits show which filter
//                                    properties have been changed.
//                                    available flags:
//                                      EDRV_FILTER_CHANGE_MASK
//                                      EDRV_FILTER_CHANGE_VALUE
//                                      EDRV_FILTER_CHANGE_STATE
//                                      EDRV_FILTER_CHANGE_AUTO_RESPONSE
//                                    if auto-response delay is supported:
//                                      EDRV_FILTER_CHANGE_AUTO_RESPONSE_DELAY
//
// Returns:     Errorcode           = kEplSuccessful
//                                  = kEplEdrvInvalidParam
//
// State:
//
//---------------------------------------------------------------------------

tEplKernel EdrvChangeFilter(tEdrvFilter*    pFilter_p,
                            unsigned int    uiCount_p,
                            unsigned int    uiEntryChanged_p,
                            unsigned int    uiChangeFlags_p)
{
tEplKernel      Ret = kEplSuccessful;
unsigned int    uiIndex;
unsigned int    uiEntry;

    if (((uiCount_p != 0) && (pFilter_p == NULL))
        || (uiCount_p >= EDRV_MAX_AUTO_RESPONSES))
    {
        Ret = kEplEdrvInvalidParam;
        goto Exit;
    }

    if (uiEntryChanged_p >= uiCount_p)
    {   // no specific entry changed
        // -> all entries changed

        // at first, disable all filters in openMAC
        for (uiEntry = 0; uiEntry < EDRV_MAX_FILTERS; uiEntry++)
        {
            omethFilterDisable(EdrvInstance_l.m_ahFilter[uiEntry]);
            omethResponseDisable(EdrvInstance_l.m_ahFilter[uiEntry]);
        }

        for (uiEntry = 0; uiEntry < uiCount_p; uiEntry++)
        {
            // set filter value and mask
            for (uiIndex = 0; uiIndex < sizeof (pFilter_p->m_abFilterValue); uiIndex++)
            {
                omethFilterSetByteValue(EdrvInstance_l.m_ahFilter[uiEntry],
                                        uiIndex,
                                        pFilter_p[uiEntry].m_abFilterValue[uiIndex]);

                omethFilterSetByteMask(EdrvInstance_l.m_ahFilter[uiEntry],
                                       uiIndex,
                                       pFilter_p[uiEntry].m_abFilterMask[uiIndex]);
            }

            // set auto response
            if (pFilter_p[uiEntry].m_pTxBuffer != NULL)
            {
                EdrvInstance_l.m_apTxBuffer[uiEntry] = pFilter_p[uiEntry].m_pTxBuffer;

                // set buffer number of TxBuffer to filter entry
                pFilter_p[uiEntry].m_pTxBuffer[0].m_BufferNumber.m_dwVal = uiEntry;
                pFilter_p[uiEntry].m_pTxBuffer[1].m_BufferNumber.m_dwVal = uiEntry;
                EdrvUpdateTxMsgBuffer(pFilter_p[uiEntry].m_pTxBuffer);
                omethResponseEnable(EdrvInstance_l.m_ahFilter[uiEntry]);

#if EDRV_AUTO_RESPONSE_DELAY != FALSE
                {
                DWORD dwDelayNs;

                    // set auto-response delay
                    dwDelayNs = pFilter_p[uiEntry].m_pTxBuffer->m_dwTimeOffsetNs;
                    if (dwDelayNs == 0)
                    {   // no auto-response delay is set
                        // send frame immediately after IFG
                        omethResponseTime(EdrvInstance_l.m_ahFilter[uiEntry], 0);
                    }
                    else
                    {   // auto-response delay is set
                    DWORD dwDelayAfterIfgNs;

                        if (dwDelayNs < EPL_C_DLL_T_IFG)
                        {   // set delay to a minimum of IFG
                            dwDelayNs = EPL_C_DLL_T_IFG;
                        }
                        dwDelayAfterIfgNs = dwDelayNs - EPL_C_DLL_T_IFG;
                        omethResponseTime(EdrvInstance_l.m_ahFilter[uiEntry],
                                          OMETH_NS_2_TICKS(dwDelayAfterIfgNs));
                    }
                }
#endif
            }

            if (pFilter_p[uiEntry].m_fEnable != FALSE)
            {   // enable the filter
                omethFilterEnable(EdrvInstance_l.m_ahFilter[uiEntry]);
            }
        }
    }
    else
    {   // specific entry should be changed

        if (((uiChangeFlags_p & (EDRV_FILTER_CHANGE_VALUE
                                 | EDRV_FILTER_CHANGE_MASK
#if EDRV_AUTO_RESPONSE_DELAY != FALSE
                                 | EDRV_FILTER_CHANGE_AUTO_RESPONSE_DELAY
#endif
                                 | EDRV_FILTER_CHANGE_AUTO_RESPONSE)) != 0)
            || (pFilter_p[uiEntryChanged_p].m_fEnable == FALSE))
        {
            // disable this filter entry
            omethFilterDisable(EdrvInstance_l.m_ahFilter[uiEntryChanged_p]);

            if ((uiChangeFlags_p & EDRV_FILTER_CHANGE_VALUE) != 0)
            {   // filter value has changed
                for (uiIndex = 0; uiIndex < sizeof (pFilter_p->m_abFilterValue); uiIndex++)
                {
                    omethFilterSetByteValue(EdrvInstance_l.m_ahFilter[uiEntryChanged_p],
                                            uiIndex,
                                            pFilter_p[uiEntryChanged_p].m_abFilterValue[uiIndex]);
                }
            }

            if ((uiChangeFlags_p & EDRV_FILTER_CHANGE_MASK) != 0)
            {   // filter mask has changed
                for (uiIndex = 0; uiIndex < sizeof (pFilter_p->m_abFilterMask); uiIndex++)
                {
                    omethFilterSetByteMask(EdrvInstance_l.m_ahFilter[uiEntryChanged_p],
                                           uiIndex,
                                           pFilter_p[uiEntryChanged_p].m_abFilterMask[uiIndex]);
                }
            }

            if ((uiChangeFlags_p & EDRV_FILTER_CHANGE_AUTO_RESPONSE) != 0)
            {   // filter auto-response state or frame has changed
                if (pFilter_p[uiEntryChanged_p].m_pTxBuffer != NULL)
                {   // auto-response enable
                    // set buffer number of TxBuffer to filter entry
                    pFilter_p[uiEntryChanged_p].m_pTxBuffer[0].m_BufferNumber.m_dwVal = uiEntryChanged_p;
                    pFilter_p[uiEntryChanged_p].m_pTxBuffer[1].m_BufferNumber.m_dwVal = uiEntryChanged_p;
                    EdrvUpdateTxMsgBuffer(pFilter_p[uiEntryChanged_p].m_pTxBuffer);
                    omethResponseEnable(EdrvInstance_l.m_ahFilter[uiEntryChanged_p]);
                }
                else
                {   // auto-response disable
                    omethResponseDisable(EdrvInstance_l.m_ahFilter[uiEntryChanged_p]);
                }
            }

#if EDRV_AUTO_RESPONSE_DELAY != FALSE
            if ((uiChangeFlags_p & EDRV_FILTER_CHANGE_AUTO_RESPONSE_DELAY) != 0)
            {   // filter auto-response delay has changed
            DWORD dwDelayNs;

                if (pFilter_p[uiEntryChanged_p].m_pTxBuffer == NULL)
                {
                    Ret = kEplEdrvInvalidParam;
                    goto Exit;
                }
                dwDelayNs = pFilter_p[uiEntryChanged_p].m_pTxBuffer->m_dwTimeOffsetNs;

                if (dwDelayNs == 0)
                {   // no auto-response delay is set
                    // send frame immediately after IFG
                    omethResponseTime(EdrvInstance_l.m_ahFilter[uiEntryChanged_p], 0);
                }
                else
                {   // auto-response delay is set
                DWORD dwDelayAfterIfgNs;

                    if (dwDelayNs < EPL_C_DLL_T_IFG)
                    {   // set delay to a minimum of IFG
                        dwDelayNs = EPL_C_DLL_T_IFG;
                    }
                    dwDelayAfterIfgNs = dwDelayNs - EPL_C_DLL_T_IFG;
                    omethResponseTime(EdrvInstance_l.m_ahFilter[uiEntryChanged_p],
                                      OMETH_NS_2_TICKS(dwDelayAfterIfgNs));
                }
            }
#endif
        }

        if (pFilter_p[uiEntryChanged_p].m_fEnable != FALSE)
        {   // enable the filter
            omethFilterEnable(EdrvInstance_l.m_ahFilter[uiEntryChanged_p]);
        }
    }

Exit:
    return Ret;
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

    return Ret;
}


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


//=========================================================================//
//                                                                         //
//          P R I V A T E   F U N C T I O N S                              //
//                                                                         //
//=========================================================================//

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

#if defined __NIOS2__
static void EdrvIrqHandler (void* pArg_p, alt_u32 dwInt_p)
{
    if(*((unsigned short *)EDRV_IRQ_BASE) & 0x1)
    {
        omethTxIrqHandler(pArg_p);
    }
    if(*((unsigned short *)EDRV_IRQ_BASE) & 0x2)
    {
        omethRxIrqHandler(pArg_p);
    }	

}
#endif

#if defined __MICROBLAZE__
static void EdrvRxInterruptHandler (void* pArg_p)
{
    omethRxIrqHandler(pArg_p);
}

static void EdrvTxInterruptHandler (void* pArg_p)
{
    omethTxIrqHandler(pArg_p);
}

#endif



//---------------------------------------------------------------------------
//
// Function:    EdrvCbSendAck
//
// Description: Tx callback function from openMAC
//
// Parameters:  *pPacket        = packet which should be released
//
// Returns:     void
//
// State:
//
//---------------------------------------------------------------------------
static void EdrvCbSendAck(ometh_packet_typ *pPacket, void *arg, unsigned long time)
{
    EdrvInstance_l.msgfree++;
    BENCHMARK_MOD_01_SET(1);
    if (arg != NULL)
    {
    tEdrvTxBuffer*  pTxBuffer = arg;

        if (pTxBuffer->m_pfnTxHandler != NULL)
        {
        	pTxBuffer->m_pfnTxHandler(pTxBuffer);
        }
    }
    BENCHMARK_MOD_01_RESET(1);
}


//---------------------------------------------------------------------------
//
// Function:    EdrvRxHook
//
// Description: This function will be called out of the Interrupt, when the
//              received packet fits to the filter
//
// Parameters:  arg         = user specific argument pointer
//              pPacket     = pointer to received packet
//              pFct        = pointer to free function (don't care)
//
// Returns:     0 if frame was used
//              -1 if frame was not used
//
// State:
//
//---------------------------------------------------------------------------

static int EdrvRxHook(void *arg, ometh_packet_typ  *pPacket, OMETH_BUF_FREE_FCT  *pFct)
{
tEdrvRxBuffer       rxBuffer;
unsigned int        uiIndex;
tEplTgtTimeStamp    TimeStamp;

    BENCHMARK_MOD_01_SET(6);
    rxBuffer.m_BufferInFrame = kEdrvBufferLastInFrame;
    rxBuffer.m_pbBuffer = (BYTE *) &pPacket->data;
    rxBuffer.m_uiRxMsgLen = pPacket->length;
    TimeStamp.m_dwTimeStamp = omethGetTimestamp(pPacket);
    rxBuffer.m_pTgtTimeStamp = &TimeStamp;
#if XPAR_MICROBLAZE_0_USE_DCACHE
	/*
	 * before handing over the received packet to the stack
	 * flush the packet's memory range
	 */
	microblaze_flush_dcache_range((void*)pPacket, pPacket->length);
	//microblaze_invalidate_dcache_range((void*)pPacket, pPacket->length);
#endif
	EdrvInstance_l.m_InitParam.m_pfnRxHandler(&rxBuffer); //pass frame to Powerlink Stack

    uiIndex = (unsigned int) arg;

    if (EdrvInstance_l.m_apTxBuffer[uiIndex] != NULL)
    {   // filter with auto-response frame triggered
        BENCHMARK_MOD_01_SET(5);
        // call Tx handler function from DLL
        if (EdrvInstance_l.m_apTxBuffer[uiIndex]->m_pfnTxHandler != NULL)
        {
            EdrvInstance_l.m_apTxBuffer[uiIndex]->m_pfnTxHandler(EdrvInstance_l.m_apTxBuffer[uiIndex]);
        }
        BENCHMARK_MOD_01_RESET(5);
    }

    BENCHMARK_MOD_01_RESET(6);

    return 0;
}

