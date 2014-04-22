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
Copyright (c) 2014, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
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
#include <oplk/oplkinc.h>
#include <common/pdo.h>
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
static BYTE*                pTripleBuf_l[3];

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static void setupPdoMemInfo(tPdoChannelSetup* pPdoChannels_p, tPdoMemRegion* pPdoMemRegion_p);

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Get address of PDO memory reagion

The function returns the address of the PDO memory region.

\return Returns the address of the PDO memory region.

\ingroup module_pdokcal
*/
//------------------------------------------------------------------------------
BYTE* pdokcal_getPdoMemRegion(void)
{
    return (BYTE*)pPdoMem_l;
}

//------------------------------------------------------------------------------
/**
\brief  Initialize PDO memory

The function initializes the memory needed to transfer PDOs.

\param  pPdoChannels        Pointer to PDO channel configuration.
\param  rxPdoMemSize_p      Size of RX PDO buffers.
\param  txPdoMemSize_p      Size of TX PDO buffers.

\return The function returns a tOplkError error code.

\ingroup module_pdokcal
*/
//------------------------------------------------------------------------------
tOplkError pdokcal_initPdoMem(tPdoChannelSetup* pPdoChannels, size_t rxPdoMemSize_p,
                              size_t txPdoMemSize_p)
{
    size_t  pdoMemSize;

    pdoMemSize = txPdoMemSize_p + rxPdoMemSize_p;

    if (pPdoMem_l != NULL)
        pdokcal_freeMem((BYTE*)pPdoMem_l, pdoMemRegionSize_l);

    pdoMemRegionSize_l = (pdoMemSize * 3) + sizeof(tPdoMemRegion);
    if (pdokcal_allocateMem(pdoMemRegionSize_l, (BYTE**)&pPdoMem_l) != kErrorOk)
    {
        return kErrorNoResource;
    }

    pTripleBuf_l[0] = (BYTE*)pPdoMem_l + sizeof(tPdoMemRegion);
    pTripleBuf_l[1] = pTripleBuf_l[0] + pdoMemSize;
    pTripleBuf_l[2] = pTripleBuf_l[1] + pdoMemSize;

    TRACE ("%s() PdoMem:%p size:%d Triple buffers at: %p/%p/%p\n", __func__,
           pPdoMem_l, pdoMemRegionSize_l,
           pTripleBuf_l[0], pTripleBuf_l[1], pTripleBuf_l[2]);

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
    TRACE("%s()\n", __func__);

    if (pPdoMem_l != NULL)
        pdokcal_freeMem((BYTE*)pPdoMem_l, pdoMemRegionSize_l);

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

\param  channelId_p             Channel ID of PDO to write.
\param  pPayload_p              Pointer to received PDO payload.
\param  pdoSize_p               Size of received PDO.

\return Returns an error code

\ingroup module_pdokcal
*/
//------------------------------------------------------------------------------
tOplkError pdokcal_writeRxPdo(UINT channelId_p, BYTE* pPayload_p, UINT16 pdoSize_p)
{
    BYTE*           pPdo;
    OPLK_ATOMIC_T   temp;

    TARGET_INVALIDATE_DCACHE((u32)&pPdoMem_l->rxChannelInfo[channelId_p],sizeof(tPdoBufferInfo ));
    pPdo = pTripleBuf_l[pPdoMem_l->rxChannelInfo[channelId_p].writeBuf] +
           pPdoMem_l->rxChannelInfo[channelId_p].channelOffset;
    //TRACE ("%s() chan:%d wi:%d\n", __func__, channelId_p, pPdoMem_l->rxChannelInfo[channelId_p].writeBuf);

    OPLK_MEMCPY(pPdo, pPayload_p, pdoSize_p);
    TARGET_FLUSH_DCACHE((u32)pPdo,pdoSize_p);
    temp = pPdoMem_l->rxChannelInfo[channelId_p].writeBuf;
    OPLK_ATOMIC_EXCHANGE(&pPdoMem_l->rxChannelInfo[channelId_p].cleanBuf,
                         temp,
                         pPdoMem_l->rxChannelInfo[channelId_p].writeBuf);

    pPdoMem_l->rxChannelInfo[channelId_p].newData = 1;
    TARGET_FLUSH_DCACHE((u32)&pPdoMem_l->rxChannelInfo[channelId_p],sizeof(tPdoBufferInfo ));
    //TRACE ("%s() chan:%d new wi:%d\n", __func__, channelId_p, pPdoMem_l->rxChannelInfo[channelId_p].writeBuf);
    //TRACE ("%s() *pPayload_p:%02x\n", __func__, *pPayload_p);
    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Read TXPDO from PDO memory

The function reads a TXPDO to be sent from the PDO memory range.

\param  channelId_p             Channel ID of PDO to read.
\param  pPayload_p              Pointer to PDO payload which will be transmitted.
\param  pdoSize_p               Size of PDO to be transmitted.

\return Returns an error code

\ingroup module_pdokcal
*/
//------------------------------------------------------------------------------
tOplkError pdokcal_readTxPdo(UINT channelId_p, BYTE* pPayload_p, UINT16 pdoSize_p)
{
    BYTE*           pPdo;
    OPLK_ATOMIC_T   readBuf;

    TARGET_INVALIDATE_DCACHE((u32)&pPdoMem_l->txChannelInfo[channelId_p],sizeof(tPdoBufferInfo ));

    if (pPdoMem_l->txChannelInfo[channelId_p].newData)
    {
        readBuf = pPdoMem_l->txChannelInfo[channelId_p].readBuf;
        OPLK_ATOMIC_EXCHANGE(&pPdoMem_l->txChannelInfo[channelId_p].cleanBuf,
                             readBuf,
                             pPdoMem_l->txChannelInfo[channelId_p].readBuf);
        pPdoMem_l->txChannelInfo[channelId_p].newData = 0;
    }

    /*TRACE ("%s() pPdo_p:%p pPayload:%p size:%d value:%d\n", __func__,
            pPdo_p, pPayload_p, pdoSize_p, *pPdo_p);*/
    //TRACE ("%s() chan:%d ri:%d\n", __func__, channelId_p, pPdoMem_l->txChannelInfo[channelId_p].readBuf);
    pPdo =  pTripleBuf_l[pPdoMem_l->txChannelInfo[channelId_p].readBuf] +
            pPdoMem_l->txChannelInfo[channelId_p].channelOffset;

    TARGET_FLUSH_DCACHE((u32)&pPdoMem_l->txChannelInfo[channelId_p],sizeof(tPdoBufferInfo ));

    TARGET_INVALIDATE_DCACHE((u32)pPdo,pdoSize_p);

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

\param  pPdoChannels_p      Pointer to PDO channel setup.
\param  pPdoMemRegion_p     Pointer to shared PDO memory region.

\return The function returns the size of the used PDO memory
*/
//------------------------------------------------------------------------------
static void setupPdoMemInfo(tPdoChannelSetup* pPdoChannels_p, tPdoMemRegion* pPdoMemRegion_p)
{
    UINT                channelId;
    UINT                offset;
    tPdoChannel*        pPdoChannel;

    offset = 0;
    for (channelId = 0, pPdoChannel = pPdoChannels_p->pRxPdoChannel;
         channelId < pPdoChannels_p->allocation.rxPdoChannelCount;
         channelId++, pPdoChannel++)
    {
        //TRACE ("RPDO %d at offset:%d\n", channelId, offset);
        pPdoMemRegion_p->rxChannelInfo[channelId].channelOffset = offset;
        pPdoMemRegion_p->rxChannelInfo[channelId].readBuf = 0;
        pPdoMemRegion_p->rxChannelInfo[channelId].writeBuf = 1;
        pPdoMemRegion_p->rxChannelInfo[channelId].cleanBuf = 2;
        pPdoMemRegion_p->rxChannelInfo[channelId].newData = 0;
        offset += pPdoChannel->pdoSize;
    }

    for (channelId = 0, pPdoChannel = pPdoChannels_p->pTxPdoChannel;
         channelId < pPdoChannels_p->allocation.txPdoChannelCount;
         channelId++, pPdoChannel++)
    {
        //TRACE ("TPDO %d at offset:%d\n", channelId, offset);
        pPdoMemRegion_p->txChannelInfo[channelId].channelOffset = offset;
        pPdoMemRegion_p->txChannelInfo[channelId].readBuf = 0;
        pPdoMemRegion_p->txChannelInfo[channelId].writeBuf = 1;
        pPdoMemRegion_p->txChannelInfo[channelId].cleanBuf = 2;
        pPdoMemRegion_p->txChannelInfo[channelId].newData = 0;
        offset += pPdoChannel->pdoSize;
    }
    pPdoMemRegion_p->pdoMemSize = offset;

    TARGET_FLUSH_DCACHE((u32)pPdoMemRegion_p,sizeof(tPdoMemRegion));
}
///\}

