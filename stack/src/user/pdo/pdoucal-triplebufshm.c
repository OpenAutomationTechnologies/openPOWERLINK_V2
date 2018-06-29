/**
********************************************************************************
\file   pdoucal-triplebufshm.c

\brief  Shared memory triple buffer implementation for user PDO CAL module

This file contains an implementation for the user PDO CAL module which uses
a shared memory region between user and kernel layer. PDOs are transfered
through triple buffering between the layers. Therefore, reads and writes to
the PDOs can occur completely asynchronously.

This file contains no specific shared memory implementation. This is encapsulated
in the pdoucalmem-XX.c modules.

A critical part is when switching the buffers. To implement safe operation
without locking, the buffer switching has to be performed in an atomic operation.

\ingroup module_pdoucal
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
#include <user/pdoucal.h>

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
static tPdoMemRegion*   pPdoMem_l;
static size_t           memSize_l;
static void*            pTripleBuf_l[3];

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Initialize PDO memory

The function initializes the memory needed to transfer PDOs.

\param[in]      pPdoChannels_p      Pointer to PDO channel configuration.
\param[in]      rxPdoMemSize_p      Size of RX PDO buffers.
\param[in]      txPdoMemSize_p      Size of TX PDO buffers.

\return The function returns a tOplkError error code.

\ingroup module_pdoucal
*/
//------------------------------------------------------------------------------
tOplkError pdoucal_initPdoMem(const tPdoChannelSetup* pPdoChannels_p,
                              size_t rxPdoMemSize_p,
                              size_t txPdoMemSize_p)
{
    void*   pMem = NULL;
    size_t  pdoMemSize;

    UNUSED_PARAMETER(pPdoChannels_p);

    pdoMemSize = rxPdoMemSize_p + txPdoMemSize_p;

    if (pPdoMem_l != NULL)
        pdoucal_cleanupPdoMem();

    memSize_l = (pdoMemSize * 3) + sizeof(tPdoMemRegion);
    if (memSize_l != 0)
    {
        if (pdoucal_allocateMem(memSize_l, &pMem) != kErrorOk)
        {
            DEBUG_LVL_ERROR_TRACE("%s() Allocating PDO memory failed!\n", __func__);
            return kErrorNoResource;
        }
    }

    pPdoMem_l = (tPdoMemRegion*)pMem;

    pTripleBuf_l[0] = (UINT8*)pPdoMem_l + sizeof(tPdoMemRegion);
    pTripleBuf_l[1] = (UINT8*)pTripleBuf_l[0] + pdoMemSize;
    pTripleBuf_l[2] = (UINT8*)pTripleBuf_l[1] + pdoMemSize;

    DEBUG_LVL_PDO_TRACE("%s() Mapped shared memory for PDO mem region at %p size %d\n",
                        __func__,
                        pPdoMem_l,
                        memSize_l);
    DEBUG_LVL_PDO_TRACE("%s() Triple buffers at: %p/%p/%p\n",
                        __func__,
                        pTripleBuf_l[0],
                        pTripleBuf_l[1],
                        pTripleBuf_l[2]);

    OPLK_ATOMIC_INIT(pPdoMem_l);

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Clean up PDO memory

The function cleans the memory allocated for PDO buffers.

\ingroup module_pdoucal
*/
//------------------------------------------------------------------------------
void pdoucal_cleanupPdoMem(void)
{
    if (pPdoMem_l != NULL)
    {
        if (pdoucal_freeMem(pPdoMem_l, memSize_l) != kErrorOk)
        {
            DEBUG_LVL_ERROR_TRACE("%s() Unmapping shared PDO mem failed\n", __func__);
        }
    }
}

//------------------------------------------------------------------------------
/**
\brief  Get address of TX PDO buffer

The function returns the address of the TXPDO buffer specified.

\param[in]      channelId_p         The PDO channel ID of the PDO to get the address.

\return Returns the address of the specified PDO buffer.

\ingroup module_pdoucal
*/
//------------------------------------------------------------------------------
void* pdoucal_getTxPdoAdrs(UINT8 channelId_p)
{
    OPLK_ATOMIC_T   wi;
    void*           pPdo;

    // Invalidate data cache for addressed txChannelInfo
    OPLK_DCACHE_INVALIDATE(&(pPdoMem_l->txChannelInfo[channelId_p]), sizeof(tPdoBufferInfo));

    wi = pPdoMem_l->txChannelInfo[channelId_p].writeBuf;

    DEBUG_LVL_PDO_TRACE("%s() channelId:%d wi:%d\n",
                        __func__,
                        channelId_p,
                        wi);

    pPdo = (UINT8*)pTripleBuf_l[wi] + pPdoMem_l->txChannelInfo[channelId_p].channelOffset;

    return pPdo;
}

//------------------------------------------------------------------------------
/**
\brief  Write TXPDO to PDO memory

The function writes a TXPDO to the PDO memory range.

\param[in]      channelId_p         Channel ID of PDO to write.
\param[in]      pPdo_p              Pointer to PDO data.
\param[in]      pdoSize_p           Size of PDO to write.

\return The function returns a tOplkError error code.

\ingroup module_pdoucal
*/
//------------------------------------------------------------------------------
tOplkError pdoucal_setTxPdo(UINT8 channelId_p,
                            void* pPdo_p,
                            size_t pdoSize_p)
{
    OPLK_ATOMIC_T    temp;

    UNUSED_PARAMETER(pPdo_p);       // Used to avoid compiler warning if OPLK_DCACHE_FLUSH is not set
    UNUSED_PARAMETER(pdoSize_p);    // Used to avoid compiler warning if OPLK_DCACHE_FLUSH is not set

    OPLK_DCACHE_FLUSH(pPdo_p, pdoSize_p);

    DEBUG_LVL_PDO_TRACE("%s() chan:%d wi:%d\n",
                        __func__,
                        channelId_p,
                        pPdoMem_l->txChannelInfo[channelId_p].writeBuf);

    // Invalidate data cache already done in pdoucal_getTxPdoAdrs()
    OPLK_DCACHE_INVALIDATE(&pPdoMem_l->txChannelInfo[channelId_p],
                           sizeof(tPdoBufferInfo));
    temp = pPdoMem_l->txChannelInfo[channelId_p].writeBuf;
    OPLK_ATOMIC_EXCHANGE(&pPdoMem_l->txChannelInfo[channelId_p].cleanBuf,
                         temp,
                         pPdoMem_l->txChannelInfo[channelId_p].writeBuf);
    pPdoMem_l->txChannelInfo[channelId_p].newData = 1;

    // Flush data cache for variables changed in this function
    OPLK_DCACHE_FLUSH(&pPdoMem_l->txChannelInfo[channelId_p],
                      sizeof(tPdoBufferInfo));

    DEBUG_LVL_PDO_TRACE("%s() chan:%d new wi:%d\n",
                        __func__,
                        channelId_p,
                        pPdoMem_l->txChannelInfo[channelId_p].writeBuf);

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Read RXPDO from PDO memory

The function reads an RXPDO from the PDO buffer.

\param[out]     ppPdo_p             Pointer to store the RXPDO data address.
\param[in]      channelId_p         Channel ID of PDO to read.
\param[in]      pdoSize_p           Size of PDO.

\return The function returns a tOplkError error code.

\ingroup module_pdoucal
*/
//------------------------------------------------------------------------------
tOplkError pdoucal_getRxPdo(void** ppPdo_p,
                            UINT8 channelId_p,
                            size_t pdoSize_p)
{
    OPLK_ATOMIC_T    readBuf;

    UNUSED_PARAMETER(pdoSize_p);    // Used to avoid compiler warning if OPLK_DCACHE_INVALIDATE is not set

    // Check parameter validity
    ASSERT(ppPdo_p != NULL);

    // Invalidate data cache for addressed txChannelInfo
    OPLK_DCACHE_INVALIDATE(&(pPdoMem_l->rxChannelInfo[channelId_p]),
                           sizeof(tPdoBufferInfo));

    if (pPdoMem_l->rxChannelInfo[channelId_p].newData)
    {
        readBuf = pPdoMem_l->rxChannelInfo[channelId_p].readBuf;
        OPLK_ATOMIC_EXCHANGE(&pPdoMem_l->rxChannelInfo[channelId_p].cleanBuf,
                             readBuf,
                             pPdoMem_l->rxChannelInfo[channelId_p].readBuf);
        pPdoMem_l->rxChannelInfo[channelId_p].newData = 0;

        // Flush data cache for variables changed in this function
        OPLK_DCACHE_FLUSH(&(pPdoMem_l->rxChannelInfo[channelId_p]),
                          sizeof(tPdoBufferInfo));
    }

    *ppPdo_p = (UINT8*)pTripleBuf_l[pPdoMem_l->rxChannelInfo[channelId_p].readBuf] +
                pPdoMem_l->rxChannelInfo[channelId_p].channelOffset;

    OPLK_DCACHE_INVALIDATE(*ppPdo_p, pdoSize_p);

    return kErrorOk;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

/// \}
