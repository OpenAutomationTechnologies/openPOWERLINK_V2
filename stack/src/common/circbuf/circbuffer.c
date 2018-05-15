/**
********************************************************************************
\file   circbuf/circbuffer.c

\brief  Circular buffer library

This file contains the implementation of a circular buffer library. The
interface of the circular buffer library is defined in circbuffer.h.

The circular buffer library needs low level functions for locking and memory
allocation. Therefore, for each supported architecture a low-level module must
be implemented. The interface of the low-level module is defined in
circbuf-arch.h.

\ingroup module_lib_circbuf
*******************************************************************************/

/*------------------------------------------------------------------------------
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

/**
********************************************************************************

\defgroup   module_lib_circbuf    Circular Buffer Library
\ingroup    libraries

The circular buffer library is designed for concurrent access of different
instances (threads, processes, etc.) to the circular buffer. Therefore, a
shared memory which could be accessed by all instances is needed. Additionally,
a lock mechanism is needed to lockout the critical sections.

The circular buffer library provides "an asynchronous interface". This means,
there is a primary instance using a different function set to all "secondary"
instances accessing the buffer. The primary instance creates the buffer by
calling circbuf_alloc(). All other instances need to connect to the buffer by
calling circbuf_connect(). The same applies for deinitialization.
After all connected instances are disconnected by calling circbuf_disconnect(),
the main instance can clean up and free the buffer by calling circbuf_free().

*******************************************************************************/

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include "circbuf-arch.h"

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

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Allocate a circular buffer

The function allocates a circular buffer.

\param[in]      id_p                The ID of the buffer to allocate,
\param[in]      size_p              The size of the buffer to allocate.
\param[out]     ppInstance_p        A pointer to store the pointer to the instance
                                    of the allocated circular buffer.

\return The function returns a tCircBufError error code.

\ingroup module_lib_circbuf
*/
//------------------------------------------------------------------------------
tCircBufError circbuf_alloc(UINT8 id_p, size_t size_p, tCircBufInstance** ppInstance_p)
{
    size_t              alignedSize;
    tCircBufInstance*   pInstance;
    tCircBufError       ret;

    // Check parameter validity
    ASSERT(ppInstance_p != NULL);

    if ((size_p == 0) || (id_p >= NR_OF_CIRC_BUFFERS))
    {
        DEBUG_LVL_ERROR_TRACE("%s() invalid arg!\n", __func__);
        return kCircBufInvalidArg;
    }

    if ((pInstance  = circbuf_createInstance(id_p, TRUE)) == NULL)
        return kCircBufNoResource;

    alignedSize = (size_p + (CIRCBUF_BLOCK_ALIGNMENT - 1)) & ~(CIRCBUF_BLOCK_ALIGNMENT - 1);

    if ((ret = circbuf_allocBuffer(pInstance, &alignedSize)) != kCircBufOk)
    {
        circbuf_freeInstance(pInstance);
        return ret;
    }

    pInstance->pCircBufHeader->bufferSize = (UINT32)alignedSize;
    pInstance->pCircBufHeader->freeSize = (UINT32)alignedSize;
    pInstance->pCircBufHeader->readOffset = 0;
    pInstance->pCircBufHeader->writeOffset = 0;
    pInstance->pCircBufHeader->dataCount = 0;
#ifdef DEBUG_CIRCBUF_SIZE_CHECK
    pInstance->pCircBufHeader->maxSize = 0;
#endif
    pInstance->pfnSigCb = NULL;

    OPLK_DCACHE_FLUSH(pInstance->pCircBufHeader, sizeof(tCircBufHeader));
    *ppInstance_p = pInstance;

    return kCircBufOk;
}

//------------------------------------------------------------------------------
/**
\brief  Free a circular buffer

The function releases/frees a circular buffer.

\param[in]      pInstance_p         The instance of the buffer to free.

\return The function returns a tCircBufError error code.

\ingroup module_lib_circbuf
*/
//------------------------------------------------------------------------------
tCircBufError circbuf_free(tCircBufInstance* pInstance_p)
{
    // Check parameter validity
    ASSERT(pInstance_p != NULL);

    circbuf_freeBuffer(pInstance_p);
    circbuf_freeInstance(pInstance_p);

    return kCircBufOk;
}

//------------------------------------------------------------------------------
/**
\brief  Connect to a circular buffer

The function connects to an existing circular buffer.

\param[in]      id_p                The ID of the buffer to connect to.
\param[out]     ppInstance_p        A pointer to store the pointer to the instance
                                    of the connected circular buffer.

\return The function returns a tCircBufError error code.

\ingroup module_lib_circbuf
*/
//------------------------------------------------------------------------------
tCircBufError circbuf_connect(UINT8 id_p, tCircBufInstance** ppInstance_p)
{
    tCircBufError       ret;
    tCircBufInstance*   pInstance;

    // Check parameter validity
    ASSERT(ppInstance_p != NULL);

    if (id_p >= NR_OF_CIRC_BUFFERS)
        return kCircBufInvalidArg;

    if ((pInstance  = circbuf_createInstance(id_p, FALSE)) == NULL)
        return kCircBufNoResource;

    if ((ret = circbuf_connectBuffer(pInstance)) != kCircBufOk)
    {
        circbuf_freeInstance(pInstance);
        return kCircBufNoResource;
    }

    *ppInstance_p = pInstance;

    return kCircBufOk;
}

//------------------------------------------------------------------------------
/**
\brief  Disconnect from a circular buffer

The function disconnects from a circular buffer.

\param[in]      pInstance_p         The instance of the buffer to disconnect.

\return The function returns a tCircBufError error code.

\ingroup module_lib_circbuf
*/
//------------------------------------------------------------------------------
tCircBufError circbuf_disconnect(tCircBufInstance* pInstance_p)
{
    // Check parameter validity
    ASSERT(pInstance_p != NULL);

    circbuf_disconnectBuffer(pInstance_p);
    circbuf_freeInstance(pInstance_p);

    return kCircBufOk;
}
//------------------------------------------------------------------------------
/**
\brief  Reset a circular buffer

The function resets a circular buffer. The read and write pointer are set
to the start address of the buffer.

\param[in]      pInstance_p         Pointer to circular buffer instance to be reset.

\ingroup module_lib_circbuf
*/
//------------------------------------------------------------------------------
void circbuf_reset(tCircBufInstance* pInstance_p)
{
    tCircBufHeader*     pHeader;

    // Check parameter validity
    ASSERT(pInstance_p != NULL);

    circbuf_lock(pInstance_p);
    OPLK_DCACHE_INVALIDATE(pInstance_p->pCircBufHeader, sizeof(tCircBufHeader));

    pHeader = pInstance_p->pCircBufHeader;
    pHeader->readOffset = 0;
    pHeader->writeOffset = 0;
    pHeader->freeSize = pHeader->bufferSize;
    pHeader->dataCount = 0;

    OPLK_DCACHE_FLUSH(pInstance_p->pCircBufHeader, sizeof(tCircBufHeader));
    circbuf_unlock(pInstance_p);
}

//------------------------------------------------------------------------------
/**
\brief  Write data to a circular buffer

The function writes a data block to a circular buffer.

\param[in]      pInstance_p         Pointer to circular buffer instance.
\param[in]      pData_p             Pointer to the data which should be written.
\param[in]      size_p              The size of the data to write.

\return The function returns a tCircBufError error code.

\ingroup module_lib_circbuf
*/
//------------------------------------------------------------------------------
tCircBufError circbuf_writeData(tCircBufInstance* pInstance_p,
                                const void* pData_p,
                                size_t size_p)
{
    UINT32              blockSize;
    UINT32              fullBlockSize;
    UINT32              chunkSize;
    tCircBufHeader*     pHeader;
    UINT8*              pCircBuf;

    // Check parameter validity
    ASSERT(pInstance_p != NULL);

    if ((pData_p == NULL) || (size_p == 0))
        return kCircBufOk;

    blockSize     = ((UINT32)size_p + (CIRCBUF_BLOCK_ALIGNMENT - 1)) & ~(CIRCBUF_BLOCK_ALIGNMENT - 1);
    fullBlockSize = blockSize + (UINT32)sizeof(UINT32);

    circbuf_lock(pInstance_p);

    pHeader = pInstance_p->pCircBufHeader;
    pCircBuf = (UINT8*)pInstance_p->pCircBuf;

    OPLK_DCACHE_INVALIDATE(pHeader, sizeof(tCircBufHeader));
    if (fullBlockSize > pHeader->freeSize)
    {
        circbuf_unlock(pInstance_p);
        return kCircBufBufferFull;
    }

    if (pHeader->writeOffset + fullBlockSize <= pHeader->bufferSize)
    {
        *(UINT32*)(pCircBuf + pHeader->writeOffset) = (UINT32)size_p;

        OPLK_MEMCPY(pCircBuf + pHeader->writeOffset + sizeof(UINT32),
                    pData_p, size_p);

        OPLK_DCACHE_FLUSH((pCircBuf + pHeader->writeOffset), fullBlockSize);

        if (pHeader->writeOffset + fullBlockSize == pHeader->bufferSize)
            pHeader->writeOffset = 0;
        else
            pHeader->writeOffset += fullBlockSize;
    }
    else
    {
        *(UINT32*)(pCircBuf + pHeader->writeOffset) = (UINT32)size_p;
        chunkSize = pHeader->bufferSize - pHeader->writeOffset - (UINT32)sizeof(UINT32);

        OPLK_MEMCPY(pCircBuf + pHeader->writeOffset + sizeof(UINT32),
                    pData_p, chunkSize);
        OPLK_DCACHE_FLUSH((pCircBuf + pHeader->writeOffset), chunkSize + sizeof(UINT32));
        OPLK_MEMCPY(pCircBuf, (const UINT8*)pData_p + chunkSize, size_p - chunkSize);
        OPLK_DCACHE_FLUSH((pCircBuf), (size_p - chunkSize));

        pHeader->writeOffset = blockSize - chunkSize;
    }
    pHeader->freeSize -= fullBlockSize;
    pHeader->dataCount++;

#ifdef DEBUG_CIRCBUF_SIZE_CHECK
    if (pHeader->bufferSize - pHeader->freeSize > pHeader->maxSize)
        pHeader->maxSize = pHeader->bufferSize - pHeader->freeSize;
#endif
    OPLK_DCACHE_FLUSH(pHeader, sizeof(tCircBufHeader));

    circbuf_unlock(pInstance_p);

    if (pInstance_p->pfnSigCb != NULL)
    {
        pInstance_p->pfnSigCb();
    }

    return kCircBufOk;
}

//------------------------------------------------------------------------------
/**
\brief  Write multiple data to a circular buffer

The function writes two different source data block to a circular buffer.

\param[in]      pInstance_p         Pointer to circular buffer instance.
\param[in]      pData_p             Pointer to the first data block to be written.
\param[in]      size_p              The size of the first data block to be written.
\param[in]      pData2_p            Pointer to the second data block to be written.
\param[in]      size2_p             The size of the second data block to be written.

\return The function returns a tCircBufError error code.

\ingroup module_lib_circbuf
*/
//------------------------------------------------------------------------------
tCircBufError circbuf_writeMultipleData(tCircBufInstance* pInstance_p,
                                        const void* pData_p, size_t size_p,
                                        const void* pData2_p, size_t size2_p)
{
    UINT32              blockSize;
    UINT32              fullBlockSize;
    UINT32              chunkSize;
    UINT32              partSize;
    tCircBufHeader*     pHeader;
    UINT8*              pCircBuf;

    // Check parameter validity
    ASSERT(pInstance_p != NULL);

    if ((pData_p == NULL) || (size_p == 0) || (pData2_p == NULL) || (size2_p == 0))
    {
        DEBUG_LVL_ERROR_TRACE("%s() Invalid pointer or size!\n");
        return kCircBufOk;
    }

    pHeader = pInstance_p->pCircBufHeader;
    pCircBuf = (UINT8*)pInstance_p->pCircBuf;
    blockSize = ((UINT32)size_p + (UINT32)size2_p + (CIRCBUF_BLOCK_ALIGNMENT - 1)) & ~(CIRCBUF_BLOCK_ALIGNMENT - 1);
    fullBlockSize = blockSize + (UINT32)sizeof(UINT32);

    //TRACE("%s() size:%d wroff:%d\n", __func__, pHeader->bufferSize, pHeader->writeOffset);
    //TRACE("%s() ptr1:%p size1:%d ptr2:%p size2:%d\n", __func__, pData_p, size_p, pData2_p, size2_p);
    circbuf_lock(pInstance_p);
    OPLK_DCACHE_INVALIDATE(pHeader, sizeof(tCircBufHeader));
    if (fullBlockSize > pHeader->freeSize)
    {
        circbuf_unlock(pInstance_p);
        return kCircBufBufferFull;
    }

    if (pHeader->writeOffset + fullBlockSize <= pHeader->bufferSize)
    {
        *(UINT32*)(pCircBuf + pHeader->writeOffset) = (UINT32)(size_p + size2_p);

        OPLK_MEMCPY(pCircBuf + pHeader->writeOffset + sizeof(UINT32),
                    pData_p, size_p);
        OPLK_MEMCPY(pCircBuf + pHeader->writeOffset + sizeof(UINT32) + size_p,
                    pData2_p, size2_p);

        OPLK_DCACHE_FLUSH((pCircBuf + pHeader->writeOffset), fullBlockSize);

        if (pHeader->writeOffset + fullBlockSize == pHeader->bufferSize)
            pHeader->writeOffset = 0;
        else
            pHeader->writeOffset += fullBlockSize;
    }
    else
    {
        // we assume that there is at least size to store the size header
        *(UINT32*)(pCircBuf + pHeader->writeOffset) = (UINT32)(size_p + size2_p);
        chunkSize = pHeader->bufferSize - pHeader->writeOffset - (UINT32)sizeof(UINT32);
        if (size_p <= chunkSize)
        {
            OPLK_MEMCPY(pCircBuf + pHeader->writeOffset + sizeof(UINT32),
                        pData_p, size_p);
            partSize = chunkSize - (UINT32)size_p;
            OPLK_MEMCPY(pCircBuf + pHeader->writeOffset + size_p + sizeof(UINT32),
                        pData2_p, partSize);

            OPLK_DCACHE_FLUSH((pCircBuf + pHeader->writeOffset), chunkSize + sizeof(UINT32));
            OPLK_MEMCPY(pCircBuf, (const UINT8*)pData2_p + partSize, size2_p - partSize);
            OPLK_DCACHE_FLUSH((pCircBuf), size2_p - partSize);
        }
        else
        {
            partSize = (UINT32)size_p - chunkSize;
            OPLK_MEMCPY(pCircBuf + pHeader->writeOffset + sizeof(UINT32),
                        pData_p, chunkSize);
            OPLK_DCACHE_FLUSH((pCircBuf + pHeader->writeOffset), chunkSize + sizeof(UINT32));
            OPLK_MEMCPY(pCircBuf, (const UINT8*)pData_p + chunkSize, partSize);
            OPLK_MEMCPY(pCircBuf + partSize, pData2_p, size2_p);

            OPLK_DCACHE_FLUSH((pCircBuf), partSize + size2_p);
        }
        pHeader->writeOffset = blockSize - chunkSize;

    }
    pHeader->freeSize -= fullBlockSize;
    pHeader->dataCount++;

#ifdef DEBUG_CIRCBUF_SIZE_CHECK
    if (pHeader->bufferSize - pHeader->freeSize > pHeader->maxSize)
        pHeader->maxSize = pHeader->bufferSize - pHeader->freeSize;
#endif
    OPLK_DCACHE_FLUSH(pHeader, sizeof(tCircBufHeader));

    circbuf_unlock(pInstance_p);
    if (pInstance_p->pfnSigCb != NULL)
    {
        pInstance_p->pfnSigCb();
    }
    return kCircBufOk;
}

//------------------------------------------------------------------------------
/**
\brief  Read data from a circular buffer

The function reads a data block from a circular buffer.

\param[in]      pInstance_p         Pointer to circular buffer instance.
\param[out]     pData_p             Pointer to store the read data.
\param[in]      size_p              The size of the destination buffer to store the data.
\param[out]     pDataBlockSize_p    Pointer to store the size of the read data.

\return The function returns a tCircBufError error code.

\ingroup module_lib_circbuf
*/
//------------------------------------------------------------------------------
tCircBufError circbuf_readData(tCircBufInstance* pInstance_p, void* pData_p,
                               size_t size_p, size_t* pDataBlockSize_p)
{
    UINT32              dataSize;
    UINT32              blockSize;
    UINT32              fullBlockSize;
    UINT32              chunkSize;
    tCircBufHeader*     pHeader;
    UINT8*              pCircBuf;

    // Check parameter validity
    ASSERT(pInstance_p != NULL);
    ASSERT(pDataBlockSize_p != NULL);

    if ((pData_p == NULL) || (size_p == 0))
        return kCircBufOk;

    pHeader = pInstance_p->pCircBufHeader;
    pCircBuf = (UINT8*)pInstance_p->pCircBuf;

    circbuf_lock(pInstance_p);

    OPLK_DCACHE_INVALIDATE(pHeader, sizeof(tCircBufHeader));
    if (pHeader->freeSize == pHeader->bufferSize)
    {
        circbuf_unlock(pInstance_p);
        return kCircBufNoReadableData;
    }

    OPLK_DCACHE_INVALIDATE((pCircBuf + pHeader->readOffset), sizeof(UINT32));

    dataSize = *(const UINT32*)(pCircBuf + pHeader->readOffset);
    blockSize = (dataSize + (CIRCBUF_BLOCK_ALIGNMENT - 1)) & ~(CIRCBUF_BLOCK_ALIGNMENT - 1);
    fullBlockSize = blockSize + (UINT32)sizeof(UINT32);

    if (dataSize > size_p)
    {
        circbuf_unlock(pInstance_p);
        return kCircBufReadsizeTooSmall;
    }

    if (pHeader->readOffset + fullBlockSize <= pHeader->bufferSize)
    {
        OPLK_DCACHE_INVALIDATE((pCircBuf + pHeader->readOffset + sizeof(UINT32)),
                                 blockSize);
        OPLK_MEMCPY(pData_p, pCircBuf + pHeader->readOffset + sizeof(UINT32),
                    dataSize);
        if (pHeader->readOffset + fullBlockSize == pHeader->bufferSize)
            pHeader->readOffset = 0;
        else
            pHeader->readOffset += fullBlockSize;
    }
    else
    {
        chunkSize = pHeader->bufferSize - pHeader->readOffset - (UINT32)sizeof(UINT32);
        OPLK_DCACHE_INVALIDATE((pCircBuf + pHeader->readOffset + sizeof(UINT32)),
                                 chunkSize);
        OPLK_MEMCPY(pData_p, pCircBuf + pHeader->readOffset + sizeof(UINT32),
                    chunkSize);

        OPLK_DCACHE_INVALIDATE(pCircBuf, dataSize - chunkSize);

        OPLK_MEMCPY((UINT8*)pData_p + chunkSize, pCircBuf, dataSize - chunkSize);
        pHeader->readOffset = blockSize - chunkSize;
    }
    pHeader->freeSize += fullBlockSize;
    pHeader->dataCount--;

    OPLK_DCACHE_FLUSH(pHeader, sizeof(tCircBufHeader));

    circbuf_unlock(pInstance_p);

    *pDataBlockSize_p = dataSize;
    return kCircBufOk;
}

//------------------------------------------------------------------------------
/**
\brief  Get the available data count

The function returns the available data count

\param[in]      pInstance_p         Pointer to circular buffer instance.

\return The function returns the available data count

\ingroup module_lib_circbuf
*/
//------------------------------------------------------------------------------
UINT32 circbuf_getDataCount(const tCircBufInstance* pInstance_p)
{
    const tCircBufHeader*   pHeader;

    // Check parameter validity
    ASSERT(pInstance_p != NULL);

    pHeader = pInstance_p->pCircBufHeader;
    OPLK_DCACHE_INVALIDATE(&pHeader->dataCount, sizeof(UINT32));

    return pHeader->dataCount;
}

#ifdef DEBUG_CIRCBUF_SIZE_CHECK
//------------------------------------------------------------------------------
/**
\brief  Get maximum used size of circular buffer

The function returns the maximum used size of the circular buffer.

\param[in]      pInstance_p         Pointer to circular buffer instance.

\return The function returns the maximum used data size of the buffer.

\ingroup module_lib_circbuf
*/
//------------------------------------------------------------------------------
UINT32 circbuf_getMaxSize(const tCircBufInstance* pInstance_p)
{
    // Check parameter validity
    ASSERT(pInstance_p != NULL);

    return pInstance_p->pCircBufHeader->maxSize;
}
#endif

//------------------------------------------------------------------------------
/**
\brief  Set signaling for a buffer

The function sets up signaling for a specified buffer.

\param[in]      pInstance_p         Pointer to circular buffer instance.
\param[in]      pfnSigCb_p          Pointer to signaling callback function.

\return The function returns a tCircBufError error code.

\ingroup module_lib_circbuf
*/
//------------------------------------------------------------------------------
tCircBufError circBuf_setSignaling(tCircBufInstance* pInstance_p, VOIDFUNCPTR pfnSigCb_p)
{
    // Check parameter validity
    ASSERT(pInstance_p != NULL);

    pInstance_p->pfnSigCb = pfnSigCb_p;
    return kCircBufOk;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

/// \}
