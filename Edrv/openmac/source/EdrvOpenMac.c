/*----------------------------------------------------------------------------
Copyright (c) 2009, B&R
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

  Project:          openPowerlink on NIOS II + openMAC

  Description:      Ethernet Driver for openMAC

  Version:          1.0 (2009/03/24)
  
  Author:           Joerg Zelenka (zelenkaj)
  
  State:            tested on openPowerlink V1.3.0
                    wait for next release (better compatibility to Stack's DLL)
                    Auto Response ability experimental state!

------------------------------------------------------------------------------
  History

 2009/03/24     zelenkaj    V1.0 implementation finished
 2009/03/26     zelenkaj    revised
  
----------------------------------------------------------------------------*/


#include "global.h"
#include "EplInc.h"
#include "edrv.h"
#include "EplAmi.h"

#include "system.h" //FPGA system definitions
#include "omethlib.h" //openMAC header

#include <sys/alt_cache.h>
#include <sys/alt_irq.h>
#include <alt_types.h>
#define     u8      alt_u8
#define     u16     alt_u16
#define     u32     alt_u32


//---------------------------------------------------------------------------
// defines
//---------------------------------------------------------------------------

//------------------------------------------------------
//--- set the system's base adr ---
// MAC / MII IP CORE
#define EDRV_MAC_BASE                   (void *)EPL_OPENMAC_0_REG_BASE
#define EDRV_MII_BASE                   (void *)EPL_OPENMAC_0_MII_BASE
#define EPL_MAC_TX_IRQ                  EPL_OPENMAC_0_REG_IRQ
#define EPL_MAC_RX_IRQ                  EPL_OPENMAC_0_RX_IRQ
//--- set the system's base adr ---

//--- set driver feature(s) ---
#define EDRV_AUTO_RESP //experimental!!!
//--- set driver feature(s) ---

//--- set driver's MTU ---
#define EDRV_MAX_BUFFER_SIZE        1518
//--- set driver's MTU ---

//--- set driver's hooks pendings ---
//RX hook buffer pendings
// set to 0 if no buffer-pending is allowed
// set 1..n the max pending RX buffers
#define EDRV_MAX_PEN_SOA            0 //1 buffer (no pending)
#define EDRV_MAX_PEN_SOC            0 //1 buffer (no pending)
#define EDRV_MAX_PEN_PREQ           0 //1 buffer (no pending)
#define EDRV_MAX_PEN_ASND           12 //12 buffers (with pending)
#define EDRV_MAX_PEN_TOME           0 //1 buffer (no pending)
//--- set driver's hooks pendings ---
// 1 + 1 + 1 + 12 + 1 = 16
// so you need 16 buffers set next definition to the sum

//--- set driver's RX buffers ---
#define EDRV_MAX_RX_BUFFERS         16
//--- set driver's RX buffers ---

//RX FIFO (for executing ASnd RX frames)
//--- set FIFO size ---
#define EDRV_MAX_RX_FIFO_SIZE       16 //should be 2^n not bigger than 32 and
                                       //bigger than EDRV_MAX_PEN_ASND
//--- set FIFO size ---
//------------------------------------------------------

#define EDRV_RAM_BASE (void *)(EDRV_MAC_BASE + 0x0800)

#if EPL_MAC_RX_IRQ > EPL_MAC_TX_IRQ
    #warning RX IRQ should have a higher IRQ priority than TX!!!
#endif

#if EPL_MAC_RX_IRQ > 0
    #warning RX IRQ should have the highest IRQ priority!!!
#endif

#if (EDRV_MAX_RX_BUFFERS > 16)
	#error This MAC version can handle 16 rx buffers, not more!
#endif

#if (EDRV_MAX_RX_FIFO_SIZE < EDRV_MAX_PEN_ASND)
	#error The FIFO size was set to small. FIFO size > ASnd pendings
#endif
#if (EDRV_MAX_RX_FIFO_SIZE > 32)
    #undef EDRV_MAX_RX_FIFO_SIZE
    #define EDRV_MAX_RX_FIFO_SIZE 32 //FIFO max. 32 entries!
#endif
#define EDRV_MASK_RX_FIFO (EDRV_MAX_RX_FIFO_SIZE - 1) //for masking the indices

#ifndef EDRV_MAX_TX_BUFFERS
    #error Set EDRV_MAX_TX_BUFFERS define in EplCfg.h!
#endif

//---------------------------------------------------------------------------
// variables
//---------------------------------------------------------------------------

struct EthDriver {
    //EPL spec
    tEdrvInitParam          driverParam;
    u8                      this_node_id;
    
    //openMAC HAL Ethernet Driver
    ometh_config_typ        ethConf;
    OMETH_H                 openMAC;
	//Hooks
    struct {
        OMETH_HOOK_H            SoA;
        OMETH_HOOK_H            SoC;
        OMETH_HOOK_H            PReq;
        OMETH_HOOK_H            ASnd;
		OMETH_HOOK_H            tome;
        //add more hooks here...
    } EthHooks;
    //its max. pendings
    struct {
        u8                      SoA;
        u8                      SoC;
        u8                      PReq;
        u8                      ASnd;
		u8                      tome;
        //add more pendings here...
    } EthHooksMaxPendings;
    //its filters
    struct {
        OMETH_FILTER_H          SoA;
        OMETH_FILTER_H          SoC;
        OMETH_FILTER_H          PReq;
        OMETH_FILTER_H          ASnd;
		OMETH_FILTER_H          tome;
        //add more filters here...
    } EthFilters;
    
    phy_reg_typ            *pPhy;
    
    //tx msg counter
    u32                     msgfree;
    u32                     msgsent;
    u32                     fulltxqueue;
    
    //tx space
    void*                   txSpace[EDRV_MAX_TX_BUFFERS];
    u8                      used_tx_buffers;
    
    //RX ASnd FIFO
    struct {
        tEdrvRxBuffer           fifo[EDRV_MAX_RX_FIFO_SIZE];
        u32                     i_wr;
        u32                     i_rd;
        u32                     mask;
        u32                     full;
        u8                      size;
    } RXFIFO;
#ifdef EDRV_AUTO_RESP
    //autoresp PRes
//    ometh_packet_typ       *autoresp_PRes;
#endif
} Eth;

//---------------------------------------------------------------------------
// prototypes
//---------------------------------------------------------------------------
//openMAC
tEplKernel init_openMAC(void);
//RX Hook functions
int SoA_Hook(void *arg, ometh_packet_typ  *pPacket, OMETH_BUF_FREE_FCT  *pFct);
int SoC_Hook(void *arg, ometh_packet_typ  *pPacket, OMETH_BUF_FREE_FCT  *pFct);
int PReq_Hook(void *arg, ometh_packet_typ  *pPacket, OMETH_BUF_FREE_FCT  *pFct);
int ASnd_Hook(void *arg, ometh_packet_typ  *pPacket, OMETH_BUF_FREE_FCT  *pFct);
int tome_Hook(void *arg, ometh_packet_typ  *pPacket, OMETH_BUF_FREE_FCT  *pFct);
tEplKernel initHook(OMETH_HOOK_H *hook,
                    OMETH_HOOK_FCT *hookfct,
                    OMETH_FILTER_H *filter,
                    u8 *filterMask, u8 *filterValue, u8 maxPending);
void sendAck(ometh_packet_typ *pPacket, void *arg, unsigned long time);
void set_NodeID(u8 nodeid);
void asyncCall(void);

//---------------------------------------------------------------------------
//
// Function:    init_openMAC
//
// Description: sets the phy, inits the MAC core and creats an instance
//
// Parameters:  void
//
// Returns:     Errorcode           = kEplSuccessful
//                                  = kEplNoResource
//
// State:
//
//---------------------------------------------------------------------------
tEplKernel init_openMAC(void) {
    int i;
    
    ////////////////////
    // initialize phy //
    ////////////////////
    Eth.pPhy = omethPhyInfo(Eth.openMAC, 0);
    //printf("Phy activation ... ");
    omethMiiControl(EDRV_MII_BASE, MII_CTRL_RESET);
    for(i=0;i<1000;i++);
    omethMiiControl(EDRV_MII_BASE, MII_CTRL_ACTIVE);
    for(i=0;i<1000;i++);
    omethMiiControl(EDRV_MII_BASE, MII_CTRL_RESET);
    for(i=0;i<1000;i++);
    omethMiiControl(EDRV_MII_BASE, MII_CTRL_ACTIVE);
    for(i=0;i<1000;i++);  
    //printf("done\n");
    
    ////////////////////////////////
    // initialize ethernet driver //
    ////////////////////////////////
    omethInit();
    
    Eth.ethConf.adapter = 0; //adapter number
    Eth.ethConf.macType = OMETH_MAC_TYPE_01; //more info in omethlib.h
    Eth.ethConf.mode = OMETH_MODE_FULLDUPLEX + OMETH_MODE_HALFDUPLEX;
    
    Eth.ethConf.pPhyBase = EDRV_MII_BASE;
    Eth.ethConf.pRamBase = EDRV_RAM_BASE;
    Eth.ethConf.pRegBase = EDRV_MAC_BASE;
    
    Eth.ethConf.rxBuffers = EDRV_MAX_RX_BUFFERS;
    Eth.ethConf.rxMtu = EDRV_MAX_BUFFER_SIZE;
    
    Eth.openMAC = omethCreate(&Eth.ethConf);
    
    if(Eth.openMAC == 0)
        return kEplNoResource;
    
    return kEplSuccessful;
}

//---------------------------------------------------------------------------
//
// Function:    initHook
//
// Description: 
//
// Parameters:  *hook			points to the created hook after calling initHook
//				*hookfct		should point to the hook function befor calling initHook
//				*filter			points to the created filter after calling initHook
//				*filterMask		should point to the first entry in an array (size 31)
//				*filterValue	should point to the first entry in an array (size 31)
//				 maxPending		should be the max. pending value for the hook
//								0 ... the hook buffer will be reused after hook return
//								1-n ... the buffer(s) will be used by the driver, 
//                                       until the buffer is passed back with omethPacketFree()
//
// Returns:     Errorcode           = kEplSuccessful
//                                  = kEplNoResource
//
// State:
//
//---------------------------------------------------------------------------
tEplKernel initHook(OMETH_HOOK_H *hook,
             OMETH_HOOK_FCT *hookfct,
             OMETH_FILTER_H *filter,
             u8 *filterMask, u8 *filterValue, u8 maxPending) {
    
    *hook = omethHookCreate(Eth.openMAC, hookfct, maxPending); //last argument max. pending
    if(*hook == 0)
        return kEplNoResource;
    
    *filter = omethFilterCreate(*hook, 0, filterMask, filterValue);
    if(*filter == 0)
        return kEplNoResource;
    
    return kEplSuccessful;
}

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
tEplKernel EdrvInit(tEdrvInitParam * pEdrvInitParam_p) {
    tEplKernel      Ret = kEplSuccessful;
    int             i;
    u8              filterMask[31],
                    filterValue[31];
    
    
    printf("initalize Ethernet Driver for openMAC\n");
    memset(&Eth, 0, sizeof(Eth)); //reset driver struct
    
    Eth.driverParam = *pEdrvInitParam_p;
    
    printf("MAC adr: %.2X-%.2X-%.2X-%.2X-%.2X-%.2X\n",
        Eth.driverParam.m_abMyMacAddr[0],
        Eth.driverParam.m_abMyMacAddr[1],
        Eth.driverParam.m_abMyMacAddr[2],
        Eth.driverParam.m_abMyMacAddr[3],
        Eth.driverParam.m_abMyMacAddr[4],
        Eth.driverParam.m_abMyMacAddr[5]);
    
    printf("init openMAC...");
    Ret = init_openMAC();
    if(Ret == kEplSuccessful)
        printf(" done\n");
    else
        printf(" error!\n");
    
    //init driver struct
    Eth.fulltxqueue = 0;
    Eth.msgfree = 0;
    Eth.msgsent = 0;
    Eth.this_node_id = 0x01;
    Eth.used_tx_buffers = 0;
    
    //init RX FIFO
    Eth.RXFIFO.full = 0;
    Eth.RXFIFO.i_rd = 0;
    Eth.RXFIFO.i_wr = 0;
    Eth.RXFIFO.mask = EDRV_MASK_RX_FIFO;
    Eth.RXFIFO.size = EDRV_MAX_RX_FIFO_SIZE;
    for(i=0; i<Eth.RXFIFO.size; i++) {
        Eth.RXFIFO.fifo[i].m_BufferInFrame = kEdrvBufferLastInFrame;
        Eth.RXFIFO.fifo[i].m_NetTime.m_dwNanoSec = 0;
        Eth.RXFIFO.fifo[i].m_NetTime.m_dwSec = 0;
        Eth.RXFIFO.fifo[i].m_uiRxMsgLen = 0;
        Eth.RXFIFO.fifo[i].m_pbBuffer = NULL;        
    }
    
    /////////////////////////
    // initialize RX hooks //
    /////////////////////////
    //RX max pending buffers
	// set the definitions above!
    Eth.EthHooksMaxPendings.ASnd = EDRV_MAX_PEN_ASND;
    Eth.EthHooksMaxPendings.PReq = EDRV_MAX_PEN_PREQ;
    Eth.EthHooksMaxPendings.SoA = EDRV_MAX_PEN_SOA;
    Eth.EthHooksMaxPendings.SoC = EDRV_MAX_PEN_SOC;
	Eth.EthHooksMaxPendings.tome = EDRV_MAX_PEN_TOME;
    //add more hook pendings here...
        
    //Hook for PReq
    printf("Hook for PReq...");
    memset(filterMask, 0, sizeof(filterMask));
    filterMask[12] = 0xFF; filterMask[13] = 0xFF; filterMask[14] = 0xFF;
    filterMask[15] = 0xFF;
    filterValue[12] = 0x88; filterValue[13] = 0xab; filterValue[14] = 0x03;
    filterValue[15] = Eth.this_node_id;
    
    Ret = initHook(&Eth.EthHooks.PReq, PReq_Hook, &Eth.EthFilters.PReq,
                    filterMask, filterValue, Eth.EthHooksMaxPendings.PReq);
    if(Ret == kEplSuccessful)
        printf(" done\n");
    else
        printf(" error!\n");
    
#ifdef EDRV_AUTO_RESP
    printf("PRes auto resp init...");
    if( !omethResponseInit(Eth.EthFilters.PReq) )
        printf(" done\n");
    else
        printf(" error!\n");
#endif

    //Hook for SoA to me
    printf("Hook for SoA...");
    memset(filterMask, 0, sizeof(filterMask));
    filterMask[12] = 0xFF; filterMask[13] = 0xFF; filterMask[14] = 0xFF;
    filterMask[21] = 0xFF;
    filterValue[12] = 0x88; filterValue[13] = 0xab; filterValue[14] = 0x05;
    filterValue[21] = Eth.this_node_id;

    Ret = initHook(&Eth.EthHooks.SoA, SoA_Hook, &Eth.EthFilters.SoA,
                    filterMask, filterValue, Eth.EthHooksMaxPendings.SoA);
    if(Ret == kEplSuccessful)
        printf(" done\n");
    else
        printf(" error!\n");
    
    //Hook for SoC
    printf("Hook for SoC...");
    memset(filterMask, 0, sizeof(filterMask));
    filterMask[12] = 0xFF; filterMask[13] = 0xFF; filterMask[14] = 0xFF;
    filterValue[12] = 0x88; filterValue[13] = 0xab; filterValue[14] = 0x01;
    
    Ret = initHook(&Eth.EthHooks.SoC, SoC_Hook, &Eth.EthFilters.SoC,
                    filterMask, filterValue, Eth.EthHooksMaxPendings.SoC);
    if(Ret == kEplSuccessful)
        printf(" done\n");
    else
        printf(" error!\n");
    
    //Hook for ASnd
    printf("Hook for ASnd...");
    memset(filterMask, 0, sizeof(filterMask));
    filterMask[12] = 0xFF; filterMask[13] = 0xFF; filterMask[14] = 0xFF;
    filterMask[15] = 0xFF;
    filterValue[12] = 0x88; filterValue[13] = 0xab; filterValue[14] = 0x06;
    filterValue[15] = Eth.this_node_id;

    Ret = initHook(&Eth.EthHooks.ASnd, ASnd_Hook, &Eth.EthFilters.ASnd,
                    filterMask, filterValue, Eth.EthHooksMaxPendings.ASnd);
    if(Ret == kEplSuccessful)
        printf(" done\n");
    else
        printf(" error!\n");

    //Hook for all Packets addressed to this mac and not hooked by the others
    /* to configer a hook to get all packets that had not been hooked by the others
     *  set all filter mask bytes to zero and don't care the filter values
     */
    printf("Hook for this MAC...");
    memset(filterMask, 0, sizeof(filterMask));
    filterMask[0] = 0xFF; filterMask[1] = 0xFF; filterMask[2] = 0xFF;
	filterMask[3] = 0xFF; filterMask[4] = 0xFF; filterMask[5] = 0xFF; 
    filterValue[0] = Eth.driverParam.m_abMyMacAddr[0];
	filterValue[1] = Eth.driverParam.m_abMyMacAddr[1];
	filterValue[2] = Eth.driverParam.m_abMyMacAddr[2];
	filterValue[3] = Eth.driverParam.m_abMyMacAddr[3];
	filterValue[4] = Eth.driverParam.m_abMyMacAddr[4];
	filterValue[5] = Eth.driverParam.m_abMyMacAddr[5];

    Ret = initHook(&Eth.EthHooks.tome, tome_Hook, &Eth.EthFilters.tome,
                    filterMask, filterValue, Eth.EthHooksMaxPendings.tome);
    if(Ret == kEplSuccessful)
        printf(" done\n");
    else
        printf(" error!\n");
    
    ///////////////////////////
    // start Ethernet Driver //
    ///////////////////////////
    omethStart(Eth.openMAC);
    printf("Start Ethernet Driver\n");
    
    ////////////////////
    // link NIOS' irq //
    ////////////////////
    alt_irq_register(EPL_MAC_RX_IRQ, (void *)Eth.openMAC, omethRxIrqHandler);
    alt_irq_register(EPL_MAC_TX_IRQ, (void *)Eth.openMAC, omethTxIrqHandler);
    
    return Ret;
}

//---------------------------------------------------------------------------
//
// Function:    set_NodeID
//
// Description: set the device's Node ID in the ethernet driver
//              this is important for filtering EPL frames
//              it will be used to set packet-hardware filter
//
// Parameters:  nodeid - u8 value containing  Node ID
//
// Returns:     void
//
// State:
//
//---------------------------------------------------------------------------
void set_NodeID(u8 nodeid) {
    Eth.this_node_id = nodeid;
    printf("set nodeID %i to Ethernet Driver\n", Eth.this_node_id);
}

//---------------------------------------------------------------------------
//
// Function:    receive Hook functions
//
// Description: these functions will be called out of the Interrupt, when the
//				received packet fits to the filter
//
// Parameters:  *arg (don't care)
//              *pPacket pointer to received packet
//              *pointer to free function (don't care)
//
// Returns:     0 if frame was used
//              -1 if frame was not used
//
// State:
//
//---------------------------------------------------------------------------

//SoA addressed to this node
int SoA_Hook(void *arg, ometh_packet_typ  *pPacket, OMETH_BUF_FREE_FCT  *pFct) {
    tEdrvRxBuffer       rxBuffer;
    
    rxBuffer.m_BufferInFrame = kEdrvBufferLastInFrame;
    rxBuffer.m_pbBuffer = (u8 *) &pPacket->data;
    rxBuffer.m_uiRxMsgLen = pPacket->length;
    rxBuffer.m_NetTime.m_dwNanoSec = 0;
    rxBuffer.m_NetTime.m_dwSec = 0;
        
    Eth.driverParam.m_pfnRxHandler(&rxBuffer); //pass frame to Powerlink Stack
    
    return 0;
}

//every SoC
int SoC_Hook(void *arg, ometh_packet_typ  *pPacket, OMETH_BUF_FREE_FCT  *pFct) {
    tEdrvRxBuffer       rxBuffer;
    
    rxBuffer.m_BufferInFrame = kEdrvBufferLastInFrame;
    rxBuffer.m_pbBuffer = (u8 *) &pPacket->data;
    rxBuffer.m_uiRxMsgLen = pPacket->length;
    rxBuffer.m_NetTime.m_dwNanoSec = 0;
    rxBuffer.m_NetTime.m_dwSec = 0;
        
    Eth.driverParam.m_pfnRxHandler(&rxBuffer); //pass frame to Powerlink Stack
    
    return 0;
}

//PReq addressed to this node
int PReq_Hook(void *arg, ometh_packet_typ  *pPacket, OMETH_BUF_FREE_FCT  *pFct) {
    tEdrvRxBuffer       rxBuffer;
    
    rxBuffer.m_BufferInFrame = kEdrvBufferLastInFrame;
    rxBuffer.m_pbBuffer = (u8 *) &pPacket->data;
    rxBuffer.m_uiRxMsgLen = pPacket->length;
    rxBuffer.m_NetTime.m_dwNanoSec = 0;
    rxBuffer.m_NetTime.m_dwSec = 0;
        
    Eth.driverParam.m_pfnRxHandler(&rxBuffer); //pass frame to Powerlink Stack
    
    return 0;
}

//ASnd addressed to this node
int ASnd_Hook(void *arg, ometh_packet_typ  *pPacket, OMETH_BUF_FREE_FCT  *pFct) {
    if( (Eth.RXFIFO.i_wr - Eth.RXFIFO.i_rd) >= Eth.EthHooksMaxPendings.ASnd) {
        Eth.RXFIFO.full++;
        return -1; //ack this frame
    }
    //take the frame into the FIFO
    Eth.RXFIFO.fifo[Eth.RXFIFO.i_rd & Eth.RXFIFO.mask].m_pbBuffer = (u8 *)
        &pPacket->data;
    Eth.RXFIFO.fifo[Eth.RXFIFO.i_rd & Eth.RXFIFO.mask].m_uiRxMsgLen =
        pPacket->length;
    
    Eth.RXFIFO.i_wr++;
    return 0; //the frame is used, so it 'll be acked later
}

//every Ethernet Frame which was not hooked by the others and is addressed to this MAC
int tome_Hook(void *arg, ometh_packet_typ  *pPacket, OMETH_BUF_FREE_FCT  *pFct) {
	//TODO implement it

	return 0; //ack frame
}

//---------------------------------------------------------------------------
//
// Function:    asyncCall
//
// Description: This function should be called ot of a while loop
//				in the main-function. It executes the packets in the FIFO and
//				lets them free (buffer will be returned to the MAC).
//
// Parameters:  void
//
// Returns:     void
//
// State:
//
//---------------------------------------------------------------------------
void asyncCall(void) {
    while(Eth.RXFIFO.i_wr - Eth.RXFIFO.i_rd) {
        //pass frame to EPL Stack
        Eth.driverParam.m_pfnRxHandler(&Eth.RXFIFO.fifo[Eth.RXFIFO.i_rd & Eth.RXFIFO.mask]);
        //ack frame in MAC
        omethPacketFree( (ometh_packet_typ*)(Eth.RXFIFO.fifo[Eth.RXFIFO.i_rd & Eth.RXFIFO.mask].m_pbBuffer) - 4 );
        
        Eth.RXFIFO.i_rd++;
    }
    
    omethPeriodic();
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
tEplKernel EdrvAllocTxMsgBuffer       (tEdrvTxBuffer * pBuffer_p) {
    u8             *p=NULL;
        
    if(pBuffer_p->m_uiMaxBufferLen > EDRV_MAX_BUFFER_SIZE)
        return kEplEdrvNoFreeBufEntry;

    if(Eth.used_tx_buffers >= EDRV_MAX_TX_BUFFERS)
        return kEplEdrvNoFreeBufEntry;

    //p = malloc(buffer_size + 32);
    p = (u8 *)alt_uncached_malloc(pBuffer_p->m_uiMaxBufferLen + 0xF);
    if(p == NULL)
    {
        return kEplEdrvNoFreeBufEntry;
    }
    
    pBuffer_p->m_uiBufferNumber = Eth.used_tx_buffers;
    
    Eth.txSpace[Eth.used_tx_buffers++] = (u32 *)p;
    
    // align frame pointer to 16 Byte boundary
    p = (u8*) (((unsigned long) (p + 0xF)) & 0xFFFFFFF0);
    
    *((u32 *)p) = pBuffer_p->m_uiMaxBufferLen;
    
    pBuffer_p->m_pbBuffer = (p + 4);
/*    
#ifdef EDRV_AUTO_RESP
    if(pBuffer_p->m_EplMsgType == kEplMsgTypePres) {
        Eth.autoresp_PRes = (ometh_packet_typ *)p;
        if(omethResponseSet(Eth.EthFilters.PReq, Eth.autoresp_PRes) == OMETH_INVALID_PACKET)
            printf("Auto Resp Error!\n");
    }
    //add other auto-response frames here...
#endif
*/
    return kEplSuccessful;
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
tEplKernel EdrvReleaseTxMsgBuffer     (tEdrvTxBuffer * pBuffer_p) {   
    int i = pBuffer_p->m_uiBufferNumber;
    
    if(Eth.txSpace[i] != NULL) {
        //free(Eth.txSpace[i]); // free space
        alt_uncached_free(Eth.txSpace[i]); // free space
        pBuffer_p->m_pbBuffer = NULL;
        Eth.txSpace[i] = NULL;
        Eth.used_tx_buffers--;
        return kEplSuccessful;
    }
    else
        return kEplEdrvBufNotExisting;
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
tEplKernel EdrvSendTxMsg              (tEdrvTxBuffer * pBuffer_p) {
    u16                 txLength = 0;
    ometh_packet_typ*   pPacket = (ometh_packet_typ *)( pBuffer_p->m_pbBuffer - 4 );
    
#ifdef EDRV_AUTO_RESP
    tEplFrame      *pFrame;
    pFrame = (tEplFrame *) pBuffer_p->m_pbBuffer;
    
    if((tEplMsgType)AmiGetByteFromLe(&pFrame->m_le_bMessageType) == kEplMsgTypePres) {
        if(omethResponseSet(Eth.EthFilters.PReq, pPacket) == OMETH_INVALID_PACKET)
        {
            printf("Auto Resp Error!\n");
        }
        
        Eth.driverParam.m_pfnTxHandler(pBuffer_p);
        return kEplSuccessful; //don't send PRes (auto send mode...)
    }
#endif
    
    pPacket->length = pBuffer_p->m_uiTxMsgLen;
    
    txLength = omethTransmitArg(Eth.openMAC, pPacket, 
                        sendAck, pBuffer_p);
    if(txLength > 0)
    {
        Eth.msgsent++;
        return kEplSuccessful;
    }
    else
    {
        return kEplEdrvNoFreeBufEntry;
    }
}


//---------------------------------------------------------------------------
//
// Function:    fctFree
//
// Description: function is needed by openMAC
//
// Parameters:  *pPacket ... packet which should be released
//
// Returns:     void
//
// State:
//
//---------------------------------------------------------------------------
void sendAck(ometh_packet_typ *pPacket, void *arg, unsigned long time) {
    Eth.msgfree++;
    Eth.driverParam.m_pfnTxHandler(arg);
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
tEplKernel EdrvShutdown(void) {    
    printf("Shutdown Ethernet Driver... ");
    if(omethDestroy(Eth.openMAC) != 0) {
        printf("error\n");
        return kEplNoResource;
    }
    printf("done\n");
    
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
tEplKernel EdrvDefineRxMacAddrEntry (BYTE * pbMacAddr_p) {
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
tEplKernel EdrvUndefineRxMacAddrEntry (BYTE * pbMacAddr_p) {
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

void EdrvInterruptHandler (void) {
    
}
