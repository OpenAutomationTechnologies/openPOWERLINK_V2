/**
********************************************************************************
\file   edrv-openmac.c

\brief  Implementation of openMAC Ethernet driver

This file contains the implementation of the openMAC Ethernet driver.

\ingroup module_edrv
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2013, SYSTEC electronic GmbH
Copyright (c) 2017, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
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
#include <kernel/edrv.h>
#include <kernel/dllkfilter.h>
#include <common/target.h>
#include <oplk/benchmark.h>

#include <target/openmac.h>
#include <omethlib.h>

//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

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

#ifndef CONFIG_EDRV_RX_BUFFERS
#define CONFIG_EDRV_RX_BUFFERS      16
#endif

#if (CONFIG_EDRV_RX_BUFFERS > EDRV_MAX_RX_BUFFERS)
#error "The number of Rx buffers exceeds the limit!"
#endif

#ifndef CONFIG_EDRV_TIME_TRIG_TX
#define CONFIG_EDRV_TIME_TRIG_TX FALSE
#endif

#ifndef CONFIG_EDRV_MAX_TX2_BUFFERS
#define CONFIG_EDRV_MAX_TX2_BUFFERS 16
#endif

//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------
#if (CONFIG_EDRV_TIME_TRIG_TX != FALSE)
/**
\brief Structure describing the second TX queue.

This structure describes the second TX queue of the openMAC Ethernet driver.
*/
typedef struct
{
    tEdrvTxBuffer*  pBuffer;                                    ///< Pointer to the TX buffer
    UINT32          timeOffsetAbs;                              ///< Absolute time offset for packet sending
} tEdrv2ndTxQueue;
#endif

/**
\brief Structure describing an instance of the Edrv

This structure describes an instance of the openMAC Ethernet driver.
*/
typedef struct
{
    tEdrvInitParam      initParam;                              ///< Init parameters
    ometh_config_typ    macConf;                                ///< MAC configuration parameters
    OMETH_H             pMacInst;                               ///< Handle of the openMAC low-level driver
    OMETH_HOOK_H        pRxHookInst;                            ///< Handle of the MAC RX hook configuration
    OMETH_FILTER_H      apRxFilterInst[EDRV_MAX_FILTERS];       ///< Array of RX filter configurations
    phy_reg_typ*        apPhyInst[OPENMAC_PHYCNT];              ///< Array of PHY register configurations
    UINT8               phyInstCount;                           ///< Number of PYHs instantiated
    UINT32              txPacketFreed;                          ///< Counter of freed packet buffers
    UINT32              txPacketSent;                           ///< Counter of of sent packets
#if (EDRV_MAX_AUTO_RESPONSES > 0)
    // auto-response Tx buffers
    tEdrvTxBuffer*      apTxBuffer[EDRV_MAX_AUTO_RESPONSES];    ///< Array of auto-response TX buffers
#endif
#if (OPENMAC_DMAOBSERV != 0)
    BOOL                fDmaError;                              ///< Flag indicating a DMA error
#endif
#if (OPENMAC_PKTLOCTX == OPENMAC_PKTBUF_LOCAL)
    void*               pTxBufferBase;                          ///< Pointer to the TX buffer base address
    void*               pNextBufferBase;                        ///< Pointer to the next buffer base address
    UINT8               txBufferCount;                          ///< TX buffer counter
    UINT                usedMemorySpace;                        ///< Used memory for the driver
#endif
#if (OPENMAC_PKTLOCRX == OPENMAC_PKTBUF_LOCAL)
    void*               pRxBufferBase;                          ///< Pointer to the RX buffer base address
#endif
#if (CONFIG_EDRV_TIME_TRIG_TX != FALSE)
    //additional tx queue
    tEdrv2ndTxQueue     txQueue[CONFIG_EDRV_MAX_TX2_BUFFERS];   ///< Array of buffers for the second TX queue
    int                 txQueueWriteIndex;                      ///< Current index in the queue for writes
    int                 txQueueReadIndex;                       ///< Current index in the queue for reads
#endif
#if (CONFIG_DLL_DEFERRED_RXFRAME_RELEASE_ASYNC != FALSE)
    OMETH_HOOK_H        pRxAsndHookInst;                        ///< Pointer to the ASnd receive hook
#endif
#if defined(CONFIG_INCLUDE_VETH)
    OMETH_HOOK_H        pRxVethHookInst;                        ///< Pointer to virtual Ethernet receive hook
#endif
#if (CONFIG_EDRV_USE_DIAGNOSTICS != FALSE)
    UINT                asyncFrameLostCount;                    ///< Number of ASync frame not released to openMAC
    UINT                vethFrameLostCount;                     ///< Number of VEth frame not released to openMAC
    UINT                asyncBufFreedCount;                     ///< Number of ASync buffers released to openMAC
    UINT                vethBufFreedCount;                      ///< Number of VEth buffers released to openMAC
    UINT                unknownBufFreedCount;                   ///< Number of unidentified buffers released to openMAC
    UINT                asyncBufAcquiredCount;                  ///< Number of ASync buffers acquired from openMAC for processing
    UINT                vethBufAcquiredCount;                   ///< Number of VEth buffers acquired from openMAC for processing
    void*               apASyncBufAddr[CONFIG_EDRV_ASND_DEFERRED_RX_BUFFERS];   ///< Pointer array to hold the addresses of ASync buffers acquired from openMAC
    void*               apVEthBufAddr[CONFIG_EDRV_VETH_DEFERRED_RX_BUFFERS];    ///< Pointer array to hold the addresses of VEth buffers acquired from openMAC
    int                 asyncBufIdx;                            ///< Index pointing to the next location to store ASync buffer address
    int                 vethBufIdx;                             ///< Index pointing to the next location to store VEth buffer address
#endif
} tEdrvInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tEdrvInstance edrvInstance_l;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static ometh_config_typ getMacConfig(UINT adapter_p);
static tOplkError initRxFilters(void);
#if (OPENMAC_PKTLOCTX == OPENMAC_PKTBUF_LOCAL)
static ometh_packet_typ* allocTxMsgBufferIntern(const tEdrvTxBuffer* pBuffer_p);
static void freeTxMsgBufferIntern(const tEdrvTxBuffer* pBuffer_p);
#endif

// Diagnostic functions
#if (CONFIG_EDRV_USE_DIAGNOSTICS != FALSE)
static void diagnoseAcquiredAsndFrame(int openMacFilterId_p, void* pAsndFrame_p);
static void diagnoseReleasedAsndFrame(void* pAsndFrame_p);
#endif

// RX Hook function
static int rxHook(void* pArg_p, ometh_packet_typ* pPacket_p, OMETH_BUF_FREE_FCT* pfnFree_p) SECTION_EDRVOPENMAC_RX_HOOK;
static void txAckCb(ometh_packet_typ* pPacket_p, void* pArg_p, ULONG time_p);
static void irqHandler(void* pArg_p) SECTION_EDRVOPENMAC_IRQ_HDL;

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
    int         i;

    // Check parameter validity
    ASSERT(pEdrvInitParam_p != NULL);

    DEBUG_LVL_EDRV_TRACE("*** %s ***\n", __func__);
    DEBUG_LVL_EDRV_TRACE(" PHY_NUM = %d\n", OPENMAC_PHYCNT);
    DEBUG_LVL_EDRV_TRACE(" RX_BUFFERS = %d\n", CONFIG_EDRV_RX_BUFFERS);
    DEBUG_LVL_EDRV_TRACE(" MAX_RX_BUFFERS = %d\n", EDRV_MAX_RX_BUFFERS);
    DEBUG_LVL_EDRV_TRACE(" PKTLOCTX = %d\n", OPENMAC_PKTLOCTX);
    DEBUG_LVL_EDRV_TRACE(" PKTLOCRX = %d\n", OPENMAC_PKTLOCRX);
    DEBUG_LVL_EDRV_TRACE(" PKTBUFSIZE = %d byte\n", OPENMAC_PKTBUFSIZE);

    OPLK_MEMSET(&edrvInstance_l, 0, sizeof(edrvInstance_l));

    edrvInstance_l.initParam = *pEdrvInitParam_p;

    // The phys are reset by the initial hw state
    //  If sw comes here again, no phy reset is done!
    target_msleep(EDRV_PHY_RST_PULSE_MS);

    // activate phys and wait until ready
    omethMiiControl((void*)OPENMAC_PHY_BASE, MII_CTRL_ACTIVE);
    target_msleep(EDRV_PHY_RST_READY_MS);

    omethInit();

    edrvInstance_l.macConf = getMacConfig(0);

    edrvInstance_l.pMacInst = omethCreate(&edrvInstance_l.macConf);

    if (edrvInstance_l.pMacInst == NULL)
    {
        ret = kErrorNoResource;
        DEBUG_LVL_ERROR_TRACE("%s() omethCreate failed\n", __func__);
        goto Exit;
    }

    //verify phy management
    for (i = 0; i < OPENMAC_PHYCNT; i++)
    {
        edrvInstance_l.apPhyInst[i] = omethPhyInfo(edrvInstance_l.pMacInst, i);
        if (edrvInstance_l.apPhyInst[i] != 0)
        {
            edrvInstance_l.phyInstCount++;
        }
    }

    if (edrvInstance_l.phyInstCount != OPENMAC_PHYCNT)
    {
        DEBUG_LVL_ERROR_TRACE("%s() Not all phy are found as configured (%d)!\n", __func__, OPENMAC_PHYCNT);
        ret = kErrorNoResource;
        goto Exit;
    }

    // initialize RX hook
    edrvInstance_l.pRxHookInst = omethHookCreate(edrvInstance_l.pMacInst, rxHook, 0); //last argument max. pending
    if (edrvInstance_l.pRxHookInst == NULL)
    {
        ret = kErrorNoResource;
        goto Exit;
    }

#if (CONFIG_DLL_DEFERRED_RXFRAME_RELEASE_ASYNC != FALSE)
    // initialize Rx hook for Asnd frames with pending allowed
    edrvInstance_l.pRxAsndHookInst = omethHookCreate(edrvInstance_l.pMacInst, rxHook, CONFIG_EDRV_ASND_DEFERRED_RX_BUFFERS);
    if (edrvInstance_l.pRxAsndHookInst == NULL)
    {
        DEBUG_LVL_ERROR_TRACE("%s() Rx hook creation for Asnd frames failed!\n", __func__);
        ret = kErrorNoResource;
        goto Exit;
    }
#endif

#if defined(CONFIG_INCLUDE_VETH)
    // initialize Rx hook for Veth frames with pending allowed
    edrvInstance_l.pRxVethHookInst = omethHookCreate(edrvInstance_l.pMacInst, rxHook, CONFIG_EDRV_VETH_DEFERRED_RX_BUFFERS);
    if (edrvInstance_l.pRxVethHookInst == NULL)
    {
        DEBUG_LVL_ERROR_TRACE("%s() Rx hook creation for Veth frames failed!\n", __func__);
        ret = kErrorNoResource;
        goto Exit;
    }
#endif

    ret = initRxFilters();
    if (ret != kErrorOk)
        goto Exit;

    //moved following lines here, since omethHookCreate may change tx buffer base!
#if ((OPENMAC_PKTLOCRX == OPENMAC_PKTBUF_LOCAL) && (OPENMAC_PKTLOCTX == OPENMAC_PKTBUF_LOCAL))
    //get rx/tx buffer base
    edrvInstance_l.pRxBufferBase = omethGetRxBufBase(edrvInstance_l.pMacInst);
    edrvInstance_l.pTxBufferBase = omethGetTxBufBase(edrvInstance_l.pMacInst);
#elif (OPENMAC_PKTLOCTX == OPENMAC_PKTBUF_LOCAL)
    //get tx buffer base
    edrvInstance_l.pTxBufferBase = OPENMAC_MEMUNCACHED((void*)OPENMAC_PKT_BASE, OPENMAC_PKT_SPAN);
#endif

    omethStart(edrvInstance_l.pMacInst, TRUE);
    DEBUG_LVL_EDRV_TRACE(" OPENMAC started\n");

    ret = openmac_isrReg(kOpenmacIrqTxRx, irqHandler, (void*)edrvInstance_l.pMacInst);

    if (ret != kErrorOk)
        goto Exit;

    //wait some time (phy may not be ready...)
    target_msleep(1000);

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
    omethStop(edrvInstance_l.pMacInst);

    openmac_isrReg(kOpenmacIrqTxRx, NULL, NULL);

    //okay, before we destroy openMAC, observe its statistics!
    DEBUG_LVL_ERROR_TRACE("%s() \n", __func__);
    {
        ometh_stat_typ* pMacStat = NULL;

        pMacStat = omethStatistics(edrvInstance_l.pMacInst);

        if (pMacStat == NULL)
        {
            DEBUG_LVL_ERROR_TRACE(" Serious error occurred!? Can't find the statistics!\n");
        }
        else
        {
            DEBUG_LVL_EDRV_TRACE(" --- omethStatistics ---\n");
            DEBUG_LVL_EDRV_TRACE(" ----  RX           ----\n");
            DEBUG_LVL_EDRV_TRACE("  CRC ERROR = %i\n", (int)pMacStat->rxCrcError);
            DEBUG_LVL_EDRV_TRACE("  HOOK DISABLED = %i\n", (int)pMacStat->rxHookDisabled);
            DEBUG_LVL_EDRV_TRACE("  HOOK OVERFLOW = %i\n", (int)pMacStat->rxHookOverflow);
            DEBUG_LVL_EDRV_TRACE("  LOST = %i\n", (int)pMacStat->rxLost);
            DEBUG_LVL_EDRV_TRACE("  OK = %i\n", (int)pMacStat->rxOk);
            DEBUG_LVL_EDRV_TRACE("  OVERSIZE = %i\n", (int)pMacStat->rxOversize);
            DEBUG_LVL_EDRV_TRACE(" ----  TX           ----\n");
            DEBUG_LVL_EDRV_TRACE("  COLLISION = %i\n", (int)pMacStat->txCollision);
            DEBUG_LVL_EDRV_TRACE("  DONE = %i\n", (int)pMacStat->txDone[0]);
            DEBUG_LVL_EDRV_TRACE("  SPURIOUS IRQ = %i\n", (int)pMacStat->txSpuriousInt);
        }
        DEBUG_LVL_EDRV_TRACE("\n");
    }

#if (CONFIG_EDRV_USE_DIAGNOSTICS != FALSE)
    DEBUG_LVL_EDRV_TRACE(" ---    edrvDiagnostics    ---\n");
    DEBUG_LVL_EDRV_TRACE(" ----  ASnd Late Release  ----\n");
    DEBUG_LVL_EDRV_TRACE("  Acquired ASync buffers = %u\n", edrvInstance_l.asyncBufAcquiredCount);
    DEBUG_LVL_EDRV_TRACE("  Released ASync buffers = %u\n", edrvInstance_l.asyncBufFreedCount);
    DEBUG_LVL_EDRV_TRACE("  Acquired VEth  buffers = %u\n", edrvInstance_l.vethBufAcquiredCount);
    DEBUG_LVL_EDRV_TRACE("  Released VEth  buffers = %u\n", edrvInstance_l.vethBufFreedCount);
    DEBUG_LVL_EDRV_TRACE("  Predicted lost ASync  frame count = %u\n", edrvInstance_l.asyncFrameLostCount);
    DEBUG_LVL_EDRV_TRACE("  Predicted lost VEth   frame count = %u\n", edrvInstance_l.vethFrameLostCount);
    DEBUG_LVL_EDRV_TRACE("  Released unidentified frame count = %u\n", edrvInstance_l.unknownBufFreedCount++;);
    DEBUG_LVL_EDRV_TRACE("\n");
#endif

#if (OPENMAC_DMAOBSERV != 0)
    if (edrvInstance_l.fDmaError != FALSE)
    {
        //if you see this openMAC DMA is connected to slow memory!
        // -> use embedded memory or 10 nsec SRAM!!!
        DEBUG_LVL_ERROR_TRACE("%s() OPENMAC DMA TRANSFER ERROR\n", __func__);
    }
#endif

    if (omethDestroy(edrvInstance_l.pMacInst) != 0)
    {
        DEBUG_LVL_ERROR_TRACE("%s() Edrv Shutdown failed\n", __func__);
        return kErrorNoResource;
    }
    DEBUG_LVL_EDRV_TRACE("Edrv Shutdown done\n");

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
\brief  Allocate Tx buffer

This function allocates a Tx buffer.

\param[in,out]  pBuffer_p           Tx buffer descriptor

\return The function returns a tOplkError error code.

\ingroup module_edrv
*/
//------------------------------------------------------------------------------
tOplkError edrv_allocTxBuffer(tEdrvTxBuffer* pBuffer_p)
{
    tOplkError          ret = kErrorOk;
    ometh_packet_typ*   pPacket = NULL;

    // Check parameter validity
    ASSERT(pBuffer_p != NULL);

    if (pBuffer_p->maxBufferSize > EDRV_MAX_BUFFER_SIZE)
    {
        ret = kErrorEdrvNoFreeBufEntry;
        goto Exit;
    }

    //openMAC does no padding, use memory for padding
    if (pBuffer_p->maxBufferSize < EDRV_MIN_ETH_SIZE)
    {
        pBuffer_p->maxBufferSize = EDRV_MIN_ETH_SIZE;
    }

#if (OPENMAC_PKTLOCTX == OPENMAC_PKTBUF_LOCAL)
    pPacket = allocTxMsgBufferIntern(pBuffer_p);
#else
    DEBUG_LVL_EDRV_TRACE("%s() allocate %i bytes\n", __func__, (int)(pBuffer_p->maxBufferSize + sizeof(pPacket->length)));
    pPacket = (ometh_packet_typ*)openmac_uncachedMalloc(pBuffer_p->maxBufferSize + sizeof(pPacket->length));
#endif

    if (pPacket == NULL)
    {
        DEBUG_LVL_ERROR_TRACE("%s() Memory allocation error\n", __func__);
        ret = kErrorEdrvNoFreeBufEntry;
        goto Exit;
    }

    pPacket->length = (unsigned long)pBuffer_p->maxBufferSize;

    pBuffer_p->txBufferNumber.value = EDRV_MAX_FILTERS;

    pBuffer_p->pBuffer = &pPacket->data;

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
    tOplkError          ret = kErrorOk;
#if (OPENMAC_PKTLOCTX != OPENMAC_PKTBUF_LOCAL)
    ometh_packet_typ*   pPacket = NULL;
#endif

    // Check parameter validity
    ASSERT(pBuffer_p != NULL);

#if (EDRV_MAX_AUTO_RESPONSES > 0)
    if (pBuffer_p->txBufferNumber.value < EDRV_MAX_AUTO_RESPONSES)
    {
        // disable auto-response
        omethResponseDisable(edrvInstance_l.apRxFilterInst[pBuffer_p->txBufferNumber.value]);
    }
#endif

    if (pBuffer_p->pBuffer == NULL)
    {
        ret = kErrorEdrvInvalidParam;
        goto Exit;
    }

#if (OPENMAC_PKTLOCTX == OPENMAC_PKTBUF_LOCAL)
    freeTxMsgBufferIntern(pBuffer_p);
#else
    pPacket = GET_TYPE_BASE(ometh_packet_typ, data, pBuffer_p->pBuffer);
    openmac_uncachedFree(pPacket);
#endif

    // mark buffer as free
    pBuffer_p->pBuffer = NULL;

Exit:
    return ret;
}

#if (CONFIG_EDRV_AUTO_RESPONSE != FALSE)
//------------------------------------------------------------------------------
/**
\brief  Update Tx buffer

This function updates the Tx buffer for use with auto-response filter.

\param[in,out]  pBuffer_p           Tx buffer descriptor

\return The function returns a tOplkError error code.

\ingroup module_edrv
*/
//------------------------------------------------------------------------------
tOplkError edrv_updateTxBuffer(tEdrvTxBuffer* pBuffer_p)
{
    tOplkError          ret = kErrorOk;
#if (EDRV_MAX_AUTO_RESPONSES > 0)
    ometh_packet_typ*   pPacket = NULL;

    // Check parameter validity
    ASSERT(pBuffer_p != NULL);

    if (pBuffer_p->txBufferNumber.value >= EDRV_MAX_AUTO_RESPONSES)
    {
        ret = kErrorEdrvInvalidParam;
        goto Exit;
    }

    pPacket = GET_TYPE_BASE(ometh_packet_typ, data, pBuffer_p->pBuffer);

    pPacket->length = (unsigned long)pBuffer_p->txFrameSize;

    // Flush data cache before handing over the packet buffer to openMAC.
    OPENMAC_FLUSHDATACACHE(pBuffer_p->pBuffer, pBuffer_p->txFrameSize);

    // Update auto-response buffer
    edrvInstance_l.apTxBuffer[pBuffer_p->txBufferNumber.value] = pBuffer_p;

    pPacket = omethResponseSet(edrvInstance_l.apRxFilterInst[pBuffer_p->txBufferNumber.value], pPacket);
    if (pPacket == OMETH_INVALID_PACKET)
    {
        ret = kErrorNoResource;
        goto Exit;
    }

Exit:
#else
    UNUSED_PARAMETER(pBuffer_p);
    //invalid call, since auto-response is deactivated for MN support
    ret = kErrorEdrvInvalidParam;
#endif
    return ret;
}
#endif /* (CONFIG_EDRV_AUTO_RESPONSE != FALSE) */

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
    ometh_packet_typ*   pPacket = NULL;
    ULONG               txLength;

    // Check parameter validity
    ASSERT(pBuffer_p != NULL);

    // Flush data cache before sending the frame to the network
    OPENMAC_FLUSHDATACACHE(pBuffer_p->pBuffer, pBuffer_p->txFrameSize);

    pPacket = GET_TYPE_BASE(ometh_packet_typ, data, pBuffer_p->pBuffer);

    pPacket->length = (unsigned long)pBuffer_p->txFrameSize;
#if (CONFIG_EDRV_TIME_TRIG_TX != FALSE)
    if (pBuffer_p->fLaunchTimeValid)
    {
        //free tx descriptors available
        txLength = omethTransmitTime(edrvInstance_l.pMacInst, pPacket,
                                     txAckCb, pBuffer_p, pBuffer_p->launchTime.ticks);

        if (txLength == 0)
        {
#if (CONFIG_EDRV_TIME_TRIG_TX != FALSE)
            //time triggered sent failed => move to 2nd tx queue
            if ((edrvInstance_l.txQueueWriteIndex - edrvInstance_l.txQueueReadIndex) >= CONFIG_EDRV_MAX_TX2_BUFFERS)
            {
                DEBUG_LVL_ERROR_TRACE("%s() Edrv 2nd TX queue is full\n", __func__);
                ret = kErrorEdrvNoFreeBufEntry;
                goto Exit;
            }
            else
            {
                tEdrv2ndTxQueue* pTxqueue = &edrvInstance_l.txQueue[edrvInstance_l.txQueueWriteIndex & (CONFIG_EDRV_MAX_TX2_BUFFERS-1)];
                pTxqueue->pBuffer = pBuffer_p;
                pTxqueue->timeOffsetAbs = pBuffer_p->launchTime.ticks;

                edrvInstance_l.txQueueWriteIndex++;
                ret = kErrorOk;
                goto Exit; //packet will be sent!
            }
#else
            DEBUG_LVL_ERROR_TRACE("%s() No TX descriptor available\n", __func__);
            ret = kErrorEdrvNoFreeBufEntry;
            goto Exit;
#endif
        }
    }
    else
    {
#endif
        txLength = omethTransmitArg(edrvInstance_l.pMacInst, pPacket,
                                    txAckCb, pBuffer_p);
#if (CONFIG_EDRV_TIME_TRIG_TX != FALSE)
    }
#endif

    if (txLength > 0)
    {
        edrvInstance_l.txPacketSent++;
        ret = kErrorOk;
    }
    else
    {
        ret = kErrorEdrvNoFreeBufEntry;
    }

#if (CONFIG_EDRV_TIME_TRIG_TX != FALSE)
Exit:
#endif
    if (ret != kErrorOk)
    {
        BENCHMARK_MOD_01_TOGGLE(7);
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Change Rx filter setup

This function changes the Rx filter setup. The parameter entryChanged_p
selects the Rx filter entry that shall be changed and \p changeFlags_p determines
the property.
If entryChanged_p is equal or larger \p count_p all Rx filters shall be changed.

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
    tOplkError  ret = kErrorOk;
    UINT        index;
    UINT        entry;

    if (((count_p != 0) && (pFilter_p == NULL)) || (count_p >= EDRV_MAX_FILTERS))
    {
        ret = kErrorEdrvInvalidParam;
        goto Exit;
    }

    if (entryChanged_p >= count_p)
    {   // no specific entry changed
        // -> all entries changed

        // at first, disable all filters in openMAC
        for (entry = 0; entry < EDRV_MAX_FILTERS; entry++)
        {
            omethFilterDisable(edrvInstance_l.apRxFilterInst[entry]);
            omethResponseDisable(edrvInstance_l.apRxFilterInst[entry]);
        }

        for (entry = 0; entry < count_p; entry++)
        {
            // set filter value and mask
            for (index = 0; index < sizeof(pFilter_p->aFilterValue); index++)
            {
                omethFilterSetByteValue(edrvInstance_l.apRxFilterInst[entry],
                                        index,
                                        pFilter_p[entry].aFilterValue[index]);

                omethFilterSetByteMask(edrvInstance_l.apRxFilterInst[entry],
                                       index,
                                       pFilter_p[entry].aFilterMask[index]);
            }
#if (EDRV_MAX_AUTO_RESPONSES > 0)
            // set auto response
            if (pFilter_p[entry].pTxBuffer != NULL)
            {
                edrvInstance_l.apTxBuffer[entry] = pFilter_p[entry].pTxBuffer;

                // set buffer number of TxBuffer to filter entry
                pFilter_p[entry].pTxBuffer[0].txBufferNumber.value = entry;
                pFilter_p[entry].pTxBuffer[1].txBufferNumber.value = entry;
                edrv_updateTxBuffer(pFilter_p[entry].pTxBuffer);
                omethResponseEnable(edrvInstance_l.apRxFilterInst[entry]);

#if (CONFIG_EDRV_AUTO_RESPONSE_DELAY != FALSE)
                {
                    UINT32 delayNs;

                    // set auto-response delay
                    delayNs = pFilter_p[entry].pTxBuffer->timeOffsetNs;
                    if (delayNs == 0)
                    {   // no auto-response delay is set
                        // send frame immediately after IFG
                        omethResponseTime(edrvInstance_l.apRxFilterInst[entry], 0);
                    }
                    else
                    {   // auto-response delay is set
                        UINT32 delayAfterIfgNs;

                        if (delayNs < C_DLL_T_IFG)
                        {   // set delay to a minimum of IFG
                            delayNs = C_DLL_T_IFG;
                        }
                        delayAfterIfgNs = delayNs - C_DLL_T_IFG;
                        omethResponseTime(edrvInstance_l.apRxFilterInst[entry],
                                          OMETH_NS_2_TICKS(delayAfterIfgNs));
                    }
                }
#endif
            }
#endif

            if (pFilter_p[entry].fEnable != FALSE)
            {   // enable the filter
                omethFilterEnable(edrvInstance_l.apRxFilterInst[entry]);
            }
        }
    }
    else
    {   // specific entry should be changed

        if (((changeFlags_p & (EDRV_FILTER_CHANGE_VALUE |
                               EDRV_FILTER_CHANGE_MASK |
#if (CONFIG_EDRV_AUTO_RESPONSE_DELAY != FALSE)
                               EDRV_FILTER_CHANGE_AUTO_RESPONSE_DELAY |
#endif
                               EDRV_FILTER_CHANGE_AUTO_RESPONSE)) != 0) ||
              (pFilter_p[entryChanged_p].fEnable == FALSE))
        {
            // disable this filter entry
            omethFilterDisable(edrvInstance_l.apRxFilterInst[entryChanged_p]);

            if ((changeFlags_p & EDRV_FILTER_CHANGE_VALUE) != 0)
            {   // filter value has changed
                for (index = 0; index < sizeof(pFilter_p->aFilterValue); index++)
                {
                    omethFilterSetByteValue(edrvInstance_l.apRxFilterInst[entryChanged_p],
                                            index,
                                            pFilter_p[entryChanged_p].aFilterValue[index]);
                }
            }

            if ((changeFlags_p & EDRV_FILTER_CHANGE_MASK) != 0)
            {   // filter mask has changed
                for (index = 0; index < sizeof(pFilter_p->aFilterMask); index++)
                {
                    omethFilterSetByteMask(edrvInstance_l.apRxFilterInst[entryChanged_p],
                                           index,
                                           pFilter_p[entryChanged_p].aFilterMask[index]);
                }
            }

            if ((changeFlags_p & EDRV_FILTER_CHANGE_AUTO_RESPONSE) != 0)
            {   // filter auto-response state or frame has changed
                if (pFilter_p[entryChanged_p].pTxBuffer != NULL)
                {   // auto-response enable
                    // set buffer number of TxBuffer to filter entry
                    pFilter_p[entryChanged_p].pTxBuffer[0].txBufferNumber.value = entryChanged_p;
                    pFilter_p[entryChanged_p].pTxBuffer[1].txBufferNumber.value = entryChanged_p;
                    edrv_updateTxBuffer(pFilter_p[entryChanged_p].pTxBuffer);
                    omethResponseEnable(edrvInstance_l.apRxFilterInst[entryChanged_p]);
                }
                else
                {   // auto-response disable
                    omethResponseDisable(edrvInstance_l.apRxFilterInst[entryChanged_p]);
                }
            }

#if (CONFIG_EDRV_AUTO_RESPONSE_DELAY != FALSE)
            if ((changeFlags_p & EDRV_FILTER_CHANGE_AUTO_RESPONSE_DELAY) != 0)
            {   // filter auto-response delay has changed
                UINT32 delayNs;

                if (pFilter_p[entryChanged_p].pTxBuffer == NULL)
                {
                    ret = kErrorEdrvInvalidParam;
                    goto Exit;
                }
                delayNs = pFilter_p[entryChanged_p].pTxBuffer->timeOffsetNs;

                if (delayNs == 0)
                {   // no auto-response delay is set
                    // send frame immediately after IFG
                    omethResponseTime(edrvInstance_l.apRxFilterInst[entryChanged_p], 0);
                }
                else
                {   // auto-response delay is set
                    UINT32 delayAfterIfgNs;

                    if (delayNs < C_DLL_T_IFG)
                    {   // set delay to a minimum of IFG
                        delayNs = C_DLL_T_IFG;
                    }
                    delayAfterIfgNs = delayNs - C_DLL_T_IFG;
                    omethResponseTime(edrvInstance_l.apRxFilterInst[entryChanged_p],
                                      OMETH_NS_2_TICKS(delayAfterIfgNs));
                }
            }
#endif
        }

        if (pFilter_p[entryChanged_p].fEnable != FALSE)
        {   // enable the filter
            omethFilterEnable(edrvInstance_l.apRxFilterInst[entryChanged_p]);
        }
    }

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Set multicast address entry

This function sets a multicast entry into the Ethernet controller.

\note The multicast filters are not supported by this driver.

\param[in]      pMacAddr_p          Multicast address.

\return The function returns a tOplkError error code.

\ingroup module_edrv
*/
//------------------------------------------------------------------------------
tOplkError edrv_setRxMulticastMacAddr(const UINT8* pMacAddr_p)
{
    UNUSED_PARAMETER(pMacAddr_p);

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Clear multicast address entry

This function removes the multicast entry from the Ethernet controller.

\note The multicast filters are not supported by this driver.

\param[in]      pMacAddr_p          Multicast address

\return The function returns a tOplkError error code.

\ingroup module_edrv
*/
//------------------------------------------------------------------------------
tOplkError edrv_clearRxMulticastMacAddr(const UINT8* pMacAddr_p)
{
    UNUSED_PARAMETER(pMacAddr_p);

    return kErrorOk;
}

#if ((CONFIG_DLL_DEFERRED_RXFRAME_RELEASE_SYNC != FALSE) || (CONFIG_DLL_DEFERRED_RXFRAME_RELEASE_ASYNC != FALSE))
//------------------------------------------------------------------------------
/**
\brief  Release Rx buffer

This function releases a late release Rx buffer.

\param[in,out]  pRxBuffer_p         Rx buffer to be released

\return The function returns a tOplkError error code.

\ingroup module_edrv
*/
//------------------------------------------------------------------------------
tOplkError edrv_releaseRxBuffer(tEdrvRxBuffer* pRxBuffer_p)
{
    tOplkError          ret = kErrorOk;
    ometh_packet_typ*   pPacket = NULL;

    // Check parameter validity
    ASSERT(pRxBuffer_p != NULL);

    pPacket = GET_TYPE_BASE(ometh_packet_typ, data, pRxBuffer_p->pBuffer);
    pPacket->length = (unsigned long)pRxBuffer_p->rxFrameSize;

#if (CONFIG_EDRV_USE_DIAGNOSTICS != FALSE)
    diagnoseReleasedAsndFrame(pPacket);
#endif

    if (pPacket->length != 0)
    {
        target_enableGlobalInterrupt(FALSE);

        // Freeing the Rx buffer is done in a critical section
        omethPacketFree(pPacket);

        target_enableGlobalInterrupt(TRUE);
    }
    else
        ret = kErrorEdrvInvalidRxBuf;

    return ret;
}
#endif

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief  Get openMAC configuration

This function returns the openMAC configuration depending on the adapter.

\param[in]      adapter_p           Adapter number

\return The function returns the adapters configuration.
*/
//------------------------------------------------------------------------------
static ometh_config_typ getMacConfig(UINT adapter_p)
{
    ometh_config_typ config;

    OPLK_MEMSET(&config, 0, sizeof(config));

    config.adapter = adapter_p;
    config.macType = OMETH_MAC_TYPE_02;

    config.mode = 0 |
                  OMETH_MODE_HALFDUPLEX |       // Half-duplex
                  OMETH_MODE_100MBIT |          // 100 Mbps
                  OMETH_MODE_DIS_AUTO_NEG       // Disable Phy auto-negotiation
            ;

    config.rxBuffers = CONFIG_EDRV_RX_BUFFERS;
    config.rxMtu = EDRV_MAX_BUFFER_SIZE;

    config.pPhyBase = (void*)OPENMAC_PHY_BASE;
    config.pRamBase = (void*)OPENMAC_RAM_BASE;
    config.pRegBase = (void*)OPENMAC_REG_BASE;

#if (OPENMAC_PKTLOCRX == OPENMAC_PKTBUF_LOCAL)
    config.pBufBase = (void*)OPENMAC_PKT_BASE;
    config.pktLoc = OMETH_PKT_LOC_MACINT;
#else
    config.pBufBase = NULL;
    config.pktLoc = OMETH_PKT_LOC_HEAP;
#endif

    return config;
}

//------------------------------------------------------------------------------
/**
\brief  Initialize Rx filters

This function initializes all Rx filters and disables them.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError initRxFilters(void)
{
    tOplkError      ret = kErrorOk;
    UINT            i;
    UINT8           aMask[31];
    UINT8           aValue[31];
    OMETH_HOOK_H    pHook;

    // initialize the filters, so that they won't match any normal Ethernet frame
    OPLK_MEMSET(aMask, 0, sizeof(aMask));
    OPLK_MEMSET(aMask, 0xFF, 6);
    OPLK_MEMSET(aValue, 0, sizeof(aValue));

    for (i = 0; i < EDRV_MAX_FILTERS; i++)
    {
        // Assign filters to corresponding hooks
        switch (i)
        {
#if (CONFIG_DLL_DEFERRED_RXFRAME_RELEASE_ASYNC != FALSE)
            case DLLK_FILTER_ASND:
                pHook = edrvInstance_l.pRxAsndHookInst;
                break;
#endif
#if defined(CONFIG_INCLUDE_VETH)
            case DLLK_FILTER_VETH_UNICAST:
            case DLLK_FILTER_VETH_BROADCAST:
                pHook = edrvInstance_l.pRxVethHookInst;
                break;
#endif
            default:
                pHook = edrvInstance_l.pRxHookInst;
                break;
        }

        edrvInstance_l.apRxFilterInst[i] = omethFilterCreate(pHook, (void*)i, aMask, aValue);
        if (edrvInstance_l.apRxFilterInst[i] == 0)
        {
            DEBUG_LVL_ERROR_TRACE("%s() Creating filter %d failed\n", __func__, i);
            ret = kErrorNoResource;
            goto Exit;
        }

        omethFilterDisable(edrvInstance_l.apRxFilterInst[i]);

#if (EDRV_MAX_AUTO_RESPONSES > 0)
        if (i < EDRV_MAX_AUTO_RESPONSES)
        {
            // initialize the auto response for each filter ...
            if (omethResponseInit(edrvInstance_l.apRxFilterInst[i]) != 0)
            {
                ret = kErrorNoResource;
                goto Exit;
            }

            // ... but disable it
            omethResponseDisable(edrvInstance_l.apRxFilterInst[i]);
        }
#endif
    }

Exit:
    return ret;
}

#if (OPENMAC_PKTLOCTX == OPENMAC_PKTBUF_LOCAL)
//------------------------------------------------------------------------------
/**
\brief  Allocate Tx buffer locally

This function allocates local memory for the Tx buffer descriptor pBuffer_p.

\param[in]      pBuffer_p           Tx buffer descriptor

\return The function returns the allocated packet buffer's descriptor.
*/
//------------------------------------------------------------------------------
static ometh_packet_typ* allocTxMsgBufferIntern(const tEdrvTxBuffer* pBuffer_p)
{
    ometh_packet_typ*   pPacket;
    size_t              bufferSize;
    void*               pBufferBase = OPENMAC_MEMUNCACHED((void*)OPENMAC_PKT_BASE, OPENMAC_PKT_SPAN);

    // Initialize if no buffer is allocated
    if (edrvInstance_l.txBufferCount == 0)
    {
        edrvInstance_l.pNextBufferBase = edrvInstance_l.pTxBufferBase;
        edrvInstance_l.usedMemorySpace = 0;
    }

    // Get buffer size from descriptor and add packet length
    bufferSize = pBuffer_p->maxBufferSize + sizeof(((ometh_packet_typ*)0)->length);

    // Align the buffer size to 4 byte alignment
    bufferSize += 0x3U;
    bufferSize &= 0xFFFFFFFCU;

    // Check for enough memory space
    if (bufferSize > OPENMAC_PKTBUFSIZE - edrvInstance_l.usedMemorySpace)
    {
        DEBUG_LVL_ERROR_TRACE("%s() Out of local memory\n", __func__);
        return NULL;
    }

    pPacket = (ometh_packet_typ*)edrvInstance_l.pNextBufferBase;

    // Return if the requested buffer is not within the memory range
    if (!(edrvInstance_l.pTxBufferBase <= (void*)pPacket && (void*)pPacket < (void*)((UINT32)pBufferBase + OPENMAC_PKTBUFSIZE)))
    {
        DEBUG_LVL_ERROR_TRACE("%s() Out of local memory\n", __func__);
        return NULL;
    }

    // Set allocated buffer to zeros
    OPLK_MEMSET(pPacket, 0, bufferSize);

    // Calculate next buffer address for next allocation
    edrvInstance_l.pNextBufferBase = (UINT8*)pPacket + bufferSize;

    // New buffer added
    edrvInstance_l.txBufferCount++;
    edrvInstance_l.usedMemorySpace += bufferSize;

    DEBUG_LVL_EDRV_TRACE("%s() Add buffer @ 0x%08X with size %4d byte ", __func__, pPacket, bufferSize);
    DEBUG_LVL_EDRV_TRACE("(Used memory %4d byte %2d buffers)\n", edrvInstance_l.usedMemorySpace, edrvInstance_l.txBufferCount);

    return pPacket;
}

//------------------------------------------------------------------------------
/**
\brief  Free Tx buffer locally

This function frees local memory for the Tx buffer descriptor pBuffer_p.

\param[in]      pBuffer_p           Packet buffer descriptor

\return The function returns the allocated packet buffer's descriptor.
*/
//------------------------------------------------------------------------------
static void freeTxMsgBufferIntern(const tEdrvTxBuffer* pBuffer_p)
{
    size_t  bufferSize = pBuffer_p->maxBufferSize + sizeof(((ometh_packet_typ*)0)->length);

    // Align the buffer size to 4 byte alignment
    bufferSize += 0x3U;
    bufferSize &= 0xFFFFFFFCU;

    // Free the buffer from local memory
    edrvInstance_l.txBufferCount--;
    edrvInstance_l.usedMemorySpace -= bufferSize;

    DEBUG_LVL_EDRV_TRACE("%s() Remove buffer with size %4d byte ", __func__, bufferSize);
    DEBUG_LVL_EDRV_TRACE("(Used memory %4d byte %2d buffers)\n", edrvInstance_l.usedMemorySpace, edrvInstance_l.txBufferCount);
}
#endif /* OPENMAC_PKTLOCTX == OPENMAC_PKTBUF_LOCAL */

#if (CONFIG_EDRV_USE_DIAGNOSTICS != FALSE)
//------------------------------------------------------------------------------
/**
\brief  Diagnose the acquired ASnd frame from openMAC

This function stores addresses of acquired ASnd frames from openMAC, in an array
and updates the corresponding diagnostic information.

\param[in]      openMacFilterId_p   openMAC filter id as used by DLL.
\param[in,out]  pAsndFrame_p        Pointer to the acquired ASnd frame.
*/
//------------------------------------------------------------------------------
static void diagnoseAcquiredAsndFrame(int openMacFilterId_p, void* pAsndFrame_p)
{
    // Store packet address in an array
    if (DLLK_FILTER_ASND == openMacFilterId_p)
    {
        if (edrvInstance_l.apASyncBufAddr[edrvInstance_l.asyncBufIdx] != NULL)
        {
            edrvInstance_l.asyncFrameLostCount++;
            DEBUG_LVL_EDRV_TRACE("The previous ASync buffer(0x%08X) @index %d is not freed!!\n",
                                 (UINT32)edrvInstance_l.apASyncBufAddr[edrvInstance_l.asyncBufIdx],
                                 edrvInstance_l.asyncBufIdx);
        }

        edrvInstance_l.apASyncBufAddr[edrvInstance_l.asyncBufIdx++] = pAsndFrame_p;
        edrvInstance_l.asyncBufAcquiredCount++;
        if (edrvInstance_l.asyncBufIdx >= CONFIG_EDRV_ASND_DEFERRED_RX_BUFFERS)
        {
            edrvInstance_l.asyncBufIdx = 0;
        }
    }
    else
    {
        if ((DLLK_FILTER_VETH_BROADCAST == openMacFilterId_p) || (DLLK_FILTER_VETH_UNICAST == openMacFilterId_p))
        {
            if (edrvInstance_l.apVEthBufAddr[edrvInstance_l.vethBufIdx] != NULL)
            {
                edrvInstance_l.vethFrameLostCount++;
                DEBUG_LVL_EDRV_TRACE("The previous VEth buffer(0x%08X) @index %d is not freed!!\n",
                                     (UINT32)edrvInstance_l.apVEthBufAddr[edrvInstance_l.vethBufIdx],
                                     edrvInstance_l.vethBufIdx);
            }

            edrvInstance_l.apVEthBufAddr[edrvInstance_l.vethBufIdx++] = pAsndFrame_p;
            edrvInstance_l.vethBufAcquiredCount++;
            if (edrvInstance_l.vethBufIdx >= CONFIG_EDRV_VETH_DEFERRED_RX_BUFFERS)
            {
                edrvInstance_l.vethBufIdx = 0;
            }
        }
        else
        {
            DEBUG_LVL_EDRV_TRACE("Unrecognized ASnd frame received!!\n");
        }
    }
}

//------------------------------------------------------------------------------
/**
\brief  Diagnose the released ASnd frame to openMAC

This function searches the addresses of the ASnd frames to be released to openMAC,
in the array which stores the buffer addresses during acquisition and updates
corresponding diagnostic information.

\param[in,out]  pAsndFrame_p        Pointer to the acquired ASnd frame.
*/
//------------------------------------------------------------------------------
static void diagnoseReleasedAsndFrame(void* pAsndFrame_p)
{
    int         i = 0;

    for (i = 0; i < CONFIG_EDRV_ASND_DEFERRED_RX_BUFFERS; i++)
    {
        if (edrvInstance_l.apASyncBufAddr[i] == pAsndFrame_p)
        {
            edrvInstance_l.asyncBufFreedCount++;
            edrvInstance_l.apASyncBufAddr[i] = NULL;
            break;
        }
    }

    if (i >= CONFIG_EDRV_ASND_DEFERRED_RX_BUFFERS)
    {
        for (i = 0; i < CONFIG_EDRV_VETH_DEFERRED_RX_BUFFERS; i++)
        {
            if (edrvInstance_l.apVEthBufAddr[i] == pAsndFrame_p)
            {
                edrvInstance_l.vethBufFreedCount++;
                edrvInstance_l.apVEthBufAddr[i] = NULL;
                break;
            }
        }

        if (i >= CONFIG_EDRV_VETH_DEFERRED_RX_BUFFERS)
        {
            edrvInstance_l.unknownBufFreedCount++;
        }
    }
}
#endif

//------------------------------------------------------------------------------
/**
\brief  Ethernet driver interrupt handler

This function is invoked by the Ethernet controller interrupt.

\param[in,out]  pArg_p              Interrupt service routine argument
*/
//------------------------------------------------------------------------------
static void irqHandler(void* pArg_p)
{
#if (OPENMAC_DMAOBSERV == 0)
    UNUSED_PARAMETER(pArg_p);
#endif

    BENCHMARK_MOD_01_SET(1);

    target_setInterruptContextFlag(TRUE);

#if (OPENMAC_DMAOBSERV != 0)
    UINT16 dmaObservVal = OPENMAC_GETDMAOBSERVER();

    //read DMA observer feature
    if (dmaObservVal != 0)
    {
        edrvInstance_l.fDmaError = TRUE;
        BENCHMARK_MOD_01_TOGGLE(7);
        DEBUG_LVL_ERROR_TRACE("%s() DMA observer recognized overflow! (%X)\n", __func__, dmaObservVal);

        omethStop(pArg_p); //since openMAC was naughty, stop it!
    }
#endif

    omethRxTxIrqHandlerMux();

#if (CONFIG_EDRV_TIME_TRIG_TX != FALSE)
    //observe additional TX queue and send packet if necessary
    while ((edrvInstance_l.txQueueWriteIndex - edrvInstance_l.txQueueReadIndex) &&
           (omethTransmitPending(edrvInstance_l.pMacInst) < 16U))
    {
        ometh_packet_typ*   pPacket;
        ULONG               txLength = 0U;
        tEdrv2ndTxQueue*    pTxqueue = &edrvInstance_l.txQueue[edrvInstance_l.txQueueReadIndex & (CONFIG_EDRV_MAX_TX2_BUFFERS-1)];
        tEdrvTxBuffer*      pBuffer_p = pTxqueue->pBuffer;

        pPacket = GET_TYPE_BASE(ometh_packet_typ, data, pBuffer_p->pBuffer);

        pPacket->length = (unsigned long)pBuffer_p->txFrameSize;

        //offset is the openMAC time tick (no conversion needed)
        txLength = omethTransmitTime(edrvInstance_l.pMacInst, pPacket,
                                     txAckCb, pBuffer_p, pTxqueue->timeOffsetAbs);

        if (txLength > 0)
        {
            edrvInstance_l.txQueueReadIndex++;
            edrvInstance_l.txPacketSent++;
        }
        else
        {
            //no tx descriptor is free
        }
    }
#endif

    target_setInterruptContextFlag(FALSE);

    BENCHMARK_MOD_01_RESET(1);
}

//------------------------------------------------------------------------------
/**
\brief  Tx buffer sent call back

This function is called by omethlib in Tx interrupt context.

\param[in,out]  pPacket_p           Sent packet
\param[in,out]  pArg_p              User specific argument holding the Tx buffer descriptor
\param[in]      time_p              Tx time stamp
*/
//------------------------------------------------------------------------------
static void txAckCb(ometh_packet_typ* pPacket_p, void* pArg_p, ULONG time_p)
{
    tEdrvTxBuffer* pTxBuffer = (tEdrvTxBuffer*)pArg_p;

    UNUSED_PARAMETER(pPacket_p);
    UNUSED_PARAMETER(time_p);

    edrvInstance_l.txPacketFreed++;

    if ((pArg_p != NULL) && (pTxBuffer->pfnTxHandler != NULL))
        pTxBuffer->pfnTxHandler(pTxBuffer);
}

//------------------------------------------------------------------------------
/**
\brief  Rx buffer hook call back

This function is called by omethlib in Rx interrupt context.

\param[in,out]  pArg_p              User specific argument holding the Tx response index
\param[in,out]  pPacket_p           Received packet
\param[in]      pfnFree_p           Function pointer to free function

\return The function returns an Rx buffer release command.
\retval 0           Packet buffer \p pPacket_p is deferred
\retval -1          Packet buffer \p pPacket_p can be freed immediately
*/
//------------------------------------------------------------------------------
static int rxHook(void* pArg_p, ometh_packet_typ* pPacket_p, OMETH_BUF_FREE_FCT* pfnFree_p)
{
    int                     ret;
    tEdrvRxBuffer           rxBuffer;
    tEdrvReleaseRxBuffer    releaseRxBuffer;
    tTimestamp              timeStamp;
#if (EDRV_MAX_AUTO_RESPONSES > 0)
    UINT                    txRespIndex = (UINT)pArg_p;
#else
    UNUSED_PARAMETER(pArg_p);
#endif
    UNUSED_PARAMETER(pfnFree_p);

    rxBuffer.bufferInFrame = kEdrvBufferLastInFrame;
    rxBuffer.pBuffer = &pPacket_p->data;
    rxBuffer.rxFrameSize = pPacket_p->length;
    timeStamp.timeStamp = omethGetTimestamp(pPacket_p);
    rxBuffer.pRxTimeStamp = &timeStamp;

    // Before handing over the Rx packet to the stack invalidate the packet's
    // memory range.
    OPENMAC_INVALIDATEDATACACHE(rxBuffer.pBuffer, rxBuffer.rxFrameSize);

    releaseRxBuffer = edrvInstance_l.initParam.pfnRxHandler(&rxBuffer); //pass frame to Powerlink Stack

    if (releaseRxBuffer == kEdrvReleaseRxBufferLater)
    {
#if (CONFIG_EDRV_USE_DIAGNOSTICS != FALSE)
        diagnoseAcquiredAsndFrame((int)pArg_p, pPacket_p);
#endif
        ret = 0; // Packet is deferred, openMAC may not use this buffer!
    }
    else
        ret = -1; // Packet processing is done, returns to openMAC again

#if (EDRV_MAX_AUTO_RESPONSES > 0)
    if (edrvInstance_l.apTxBuffer[txRespIndex] != NULL)
    {   // filter with auto-response frame triggered
        BENCHMARK_MOD_01_SET(5);
        // call Tx handler function from DLL
        if (edrvInstance_l.apTxBuffer[txRespIndex]->pfnTxHandler != NULL)
        {
            edrvInstance_l.apTxBuffer[txRespIndex]->pfnTxHandler(edrvInstance_l.apTxBuffer[txRespIndex]);
        }
        BENCHMARK_MOD_01_RESET(5);
    }
#endif

    return ret;
}

/// \}
