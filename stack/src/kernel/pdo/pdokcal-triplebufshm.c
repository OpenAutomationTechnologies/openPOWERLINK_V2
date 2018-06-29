/**
********************************************************************************
\file   pdokcal-triplebufshm.c

\brief  Shared memory triple buffer implementation for kernel PDO CAL module

This file contains an implementation for the kernel PDO CAL module which uses
a shared memory region between user and kernel layer. PDOs are transfered
through triple buffering between the layers. Therefore, reads and writes to
the PDOs can occur completely asynchronously.

This file contains no specific shared memory implementation. This is encapsulated
in the pdokcalmem-XX.c modules.

A critical part is when switching the buffers. To implement safe operation
without locking, the buffer switching has to be performed in an atomic operation.

\ingroup module_pdokcal
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2017, B&R Industrial Automation GmbH
Copyright (c) 2018, Kalycito Infotech Private Limited
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
#include <common/target.h>
#include <kernel/pdokcal.h>

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

//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tPdoMemRegion*       pPdoMem_l;
static size_t               pdoMemRegionSize_l;
static void*                pTripleBuf_l[3];

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static void setupPdoMemInfo(const tPdoChannelSetup* pPdoChannels_p,
                            tPdoMemRegion* pPdoMemRegion_p);

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Get address of PDO memory region

The function returns the address of the PDO memory region.

\param[out]     ppPdoMemBase        Double pointer to the PDO memory.
\param[out]     pPdoMemSize_p       Pointer to the size of PDO memory.

\note pPdoMemSize_p is optional, caller can specify NULL if the size is not required.

\return Returns the address of the PDO memory region.

\ingroup module_pdokcal
*/
//------------------------------------------------------------------------------
tOplkError pdokcal_getPdoMemRegion(void** ppPdoMemBase, size_t* pPdoMemSize_p)
{
    if (ppPdoMemBase == NULL)
        return kErrorInvalidOperation;

    if (pPdoMemSize_p != NULL)
        *pPdoMemSize_p = pdoMemRegionSize_l;

    *ppPdoMemBase = pPdoMem_l;

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Initialize PDO memory

The function initializes the memory needed to transfer PDOs.

\param[in]      pPdoChannels        Pointer to PDO channel configuration.
\param[in]      rxPdoMemSize_p      Size of RX PDO buffers.
\param[in]      txPdoMemSize_p      Size of TX PDO buffers.

\return The function returns a tOplkError error code.

\ingroup module_pdokcal
*/
//------------------------------------------------------------------------------
tOplkError pdokcal_initPdoMem(const tPdoChannelSetup* pPdoChannels,
                              size_t rxPdoMemSize_p,
                              size_t txPdoMemSize_p)
{
    void*   pMem;
    size_t  pdoMemSize;

    // Check parameter validity
    ASSERT(pPdoChannels != NULL);

    pdoMemSize = txPdoMemSize_p + rxPdoMemSize_p;

    if (pPdoMem_l != NULL)
        pdokcal_freeMem(pPdoMem_l, pdoMemRegionSize_l);

    pdoMemRegionSize_l = (pdoMemSize * 3) + sizeof(tPdoMemRegion);
    if (pdokcal_allocateMem(pdoMemRegionSize_l, &pMem) != kErrorOk)
        return kErrorNoResource;

    pPdoMem_l = (tPdoMemRegion*)pMem;

    pTripleBuf_l[0] = (UINT8*)pPdoMem_l + sizeof(tPdoMemRegion);
    pTripleBuf_l[1] = (UINT8*)pTripleBuf_l[0] + pdoMemSize;
    pTripleBuf_l[2] = (UINT8*)pTripleBuf_l[1] + pdoMemSize;

    DEBUG_LVL_PDO_TRACE("%s() PdoMem:%p size:%d Triple buffers at: %p/%p/%p\n",
                        __func__,
                        pPdoMem_l,
                        pdoMemRegionSize_l,
                        pTripleBuf_l[0],
                        pTripleBuf_l[1],
                        pTripleBuf_l[2]);

    OPLK_MEMSET(pPdoMem_l, 0, pdoMemRegionSize_l);
    setupPdoMemInfo(pPdoChannels, pPdoMem_l);

    OPLK_ATOMIC_INIT(pPdoMem_l);

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Clean up PDO memory

The function cleans the memory allocated for PDO buffers.

\ingroup module_pdokcal
*/
//------------------------------------------------------------------------------
void pdokcal_cleanupPdoMem(void)
{
    DEBUG_LVL_PDO_TRACE("%s()\n", __func__);

    if (pPdoMem_l != NULL)
        pdokcal_freeMem(pPdoMem_l, pdoMemRegionSize_l);

    pPdoMem_l = NULL;
    pdoMemRegionSize_l = 0;
    pTripleBuf_l[0] = NULL;
    pTripleBuf_l[1] = NULL;
    pTripleBuf_l[2] = NULL;
}

//------------------------------------------------------------------------------
/**
\brief  Write RXPDO to PDO memory

The function writes a received RXPDO into the PDO memory range.

\param[in]      channelId_p         Channel ID of PDO to write.
\param[in]      pPayload_p          Pointer to received PDO payload.
\param[in]      pdoSize_p           Size of received PDO.

\return Returns an error code

\ingroup module_pdokcal
*/
//------------------------------------------------------------------------------
tOplkError pdokcal_writeRxPdo(UINT8 channelId_p, const void* pPayload_p, UINT16 pdoSize_p)
{
    void*           pPdo;
    OPLK_ATOMIC_T   temp;

    // Check parameter validity
    ASSERT(pPayload_p != NULL);

    // Invalidate data cache for addressed rxChannelInfo
    OPLK_DCACHE_INVALIDATE(&(pPdoMem_l->rxChannelInfo[channelId_p]), sizeof(tPdoBufferInfo));

    pPdo = (UINT8*)pTripleBuf_l[pPdoMem_l->rxChannelInfo[channelId_p].writeBuf] +
           pPdoMem_l->rxChannelInfo[channelId_p].channelOffset;
    //TRACE("%s() chan:%d wi:%d\n", __func__, channelId_p, pPdoMem_l->rxChannelInfo[channelId_p].writeBuf);

    OPLK_MEMCPY(pPdo, pPayload_p, pdoSize_p);

    OPLK_DCACHE_FLUSH(pPdo, pdoSize_p);

    // Invalidate the cache again, as the previous cache invalidation is
    // not under the lock context, and also the value of .cleanBuf may
    // be changed on the physical memory.
    OPLK_DCACHE_INVALIDATE(&(pPdoMem_l->rxChannelInfo[channelId_p]),
                           sizeof(tPdoBufferInfo));
    temp = pPdoMem_l->rxChannelInfo[channelId_p].writeBuf;
    OPLK_ATOMIC_EXCHANGE(&pPdoMem_l->rxChannelInfo[channelId_p].cleanBuf,
                         temp,
                         pPdoMem_l->rxChannelInfo[channelId_p].writeBuf);

    pPdoMem_l->rxChannelInfo[channelId_p].newData = 1;

    // Flush data cache for variables changed in this function
    OPLK_DCACHE_FLUSH(&(pPdoMem_l->rxChannelInfo[channelId_p]),
                      sizeof(tPdoBufferInfo));

    //TRACE("%s() chan:%d new wi:%d\n", __func__, channelId_p, pPdoMem_l->rxChannelInfo[channelId_p].writeBuf);
    //TRACE("%s() *pPayload_p:%02x\n", __func__, *pPayload_p);
    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Read TXPDO from PDO memory

The function reads a TXPDO to be sent from the PDO memory range.

\param[in]      channelId_p         Channel ID of PDO to read.
\param[out]     pPayload_p          Pointer to PDO payload which will be transmitted.
\param[in]      pdoSize_p           Size of PDO to be transmitted.

\return Returns an error code

\ingroup module_pdokcal
*/
//------------------------------------------------------------------------------
tOplkError pdokcal_readTxPdo(UINT8 channelId_p, void* pPayload_p, UINT16 pdoSize_p)
{
    void*           pPdo;
    OPLK_ATOMIC_T   readBuf;

    // Check parameter validity
    ASSERT(pPayload_p != NULL);

    // Invalidate data cache for addressed txChannelInfo
    OPLK_DCACHE_INVALIDATE(&(pPdoMem_l->txChannelInfo[channelId_p]), sizeof(tPdoBufferInfo));

    if (pPdoMem_l->txChannelInfo[channelId_p].newData)
    {
        readBuf = pPdoMem_l->txChannelInfo[channelId_p].readBuf;
        OPLK_ATOMIC_EXCHANGE(&pPdoMem_l->txChannelInfo[channelId_p].cleanBuf,
                             readBuf,
                             pPdoMem_l->txChannelInfo[channelId_p].readBuf);
        pPdoMem_l->txChannelInfo[channelId_p].newData = 0;

        // Flush data cache for variables changed in this function
        OPLK_DCACHE_FLUSH(&(pPdoMem_l->txChannelInfo[channelId_p]),
                          sizeof(tPdoBufferInfo));
    }

    pPdo = (UINT8*)pTripleBuf_l[pPdoMem_l->txChannelInfo[channelId_p].readBuf] +
               pPdoMem_l->txChannelInfo[channelId_p].channelOffset;

    DEBUG_LVL_PDO_TRACE("%s() chan:%d ri:%d\n",
                        __func__,
                        channelId_p,
                        pPdoMem_l->txChannelInfo[channelId_p].readBuf);

    OPLK_DCACHE_INVALIDATE(pPdo, pdoSize_p);

    OPLK_MEMCPY(pPayload_p, pPdo, pdoSize_p);

    return kErrorOk;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief  Setup PDO memory info

The function sets up the PDO memory info. For each channel the offset in the
shared buffer and the size are stored.

\param[in]      pPdoChannels_p      Pointer to PDO channel setup.
\param[in,out]  pPdoMemRegion_p     Pointer to shared PDO memory region.
*/
//------------------------------------------------------------------------------
static void setupPdoMemInfo(const tPdoChannelSetup* pPdoChannels_p,
                            tPdoMemRegion* pPdoMemRegion_p)
{
    UINT8               channelId;
    UINT                offset;
    const tPdoChannel*  pPdoChannel;

    offset = 0;
    for (channelId = 0, pPdoChannel = pPdoChannels_p->pRxPdoChannel;
         channelId < pPdoChannels_p->allocation.rxPdoChannelCount;
         channelId++, pPdoChannel++)
    {
        //TRACE("RPDO %d at offset:%d\n", channelId, offset);
        pPdoMemRegion_p->rxChannelInfo[channelId].channelOffset = offset;
        pPdoMemRegion_p->rxChannelInfo[channelId].readBuf = 0;
        pPdoMemRegion_p->rxChannelInfo[channelId].writeBuf = 1;
        pPdoMemRegion_p->rxChannelInfo[channelId].cleanBuf = 2;
        pPdoMemRegion_p->rxChannelInfo[channelId].newData = 0;
        offset += pPdoChannel->nextChannelOffset - pPdoChannel->offset;
    }

    for (channelId = 0, pPdoChannel = pPdoChannels_p->pTxPdoChannel;
         channelId < pPdoChannels_p->allocation.txPdoChannelCount;
         channelId++, pPdoChannel++)
    {
        //TRACE("TPDO %d at offset:%d\n", channelId, offset);
        pPdoMemRegion_p->txChannelInfo[channelId].channelOffset = offset;
        pPdoMemRegion_p->txChannelInfo[channelId].readBuf = 0;
        pPdoMemRegion_p->txChannelInfo[channelId].writeBuf = 1;
        pPdoMemRegion_p->txChannelInfo[channelId].cleanBuf = 2;
        pPdoMemRegion_p->txChannelInfo[channelId].newData = 0;
        offset += pPdoChannel->nextChannelOffset - pPdoChannel->offset;
    }
    pPdoMemRegion_p->pdoMemSize = offset;

    OPLK_DCACHE_FLUSH(pPdoMemRegion_p, sizeof(tPdoMemRegion));
}

/// \}
