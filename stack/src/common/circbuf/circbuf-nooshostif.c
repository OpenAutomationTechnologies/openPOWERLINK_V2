/**
********************************************************************************
\file   circbuf/circbuf-nooshostif.c

\brief  Circular buffer implementation for non-os systems with host interface

This file contains the architecture specific circular buffer functions
for systems without operating system (e.g. microcontrollers, niosII, microblaze)
and host interface IP-Core.

This implementation stores the circular buffer instances in a global variable
because there is no multitasking and this memory can be accessed from normal
execution context as well as from interrupt context. Locking is simply performed
by switching off interrupts. For queues between kernel and user layer a spinlock
implementation is applied.

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

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <common/oplkinc.h>
#include <common/target.h>

#include "circbuf-arch.h"
#include <hostiflib.h>

//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

#ifndef CONFIG_HOSTIF_PCP
#error "CONFIG_HOSTIF_PCP is needed for this implementation!"
#endif

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
const tHostifInstanceId hostifInstance[NR_OF_CIRC_BUFFERS] =
{
        kHostifInstIdU2KQueue,      ///< User-to-kernel event queue
        kHostifInstIdK2UQueue,      ///< Kernel-to-user event queue
        kHostifInstIdInvalid,       ///< Kernel internal event queue
        kHostifInstIdInvalid,       ///< User internal event queue
        kHostifInstIdTxGenQueue,    ///< Queue for sending generic requests in the DLLCAL
        kHostifInstIdTxNmtQueue,    ///< Queue for sending NMT requests in the DLLCAL
        kHostifInstIdTxSyncQueue,   ///< Queue for sending sync requests in the DLLCAL
        kHostifInstIdInvalid,       ///< NMT request queue for MN asynchronous scheduler
        kHostifInstIdInvalid,       ///< Generic request queue for MN asynchronous scheduler
        kHostifInstIdInvalid,       ///< Ident request queue for MN asynchronous scheduler
        kHostifInstIdInvalid,       ///< Status request queue for MN asynchronous scheduler
        kHostifInstIdTxVethQueue,   ///< Queue for sending virtual Ethernet frames in the DLLCAL
};

#if (CONFIG_HOSTIF_PCP != FALSE)
#define CIRCBUF_HOSTIF_LOCK_LOCAL       0
#define CIRCBUF_HOSTIF_LOCK_OTHER       1
#else
#define CIRCBUF_HOSTIF_LOCK_LOCAL       1
#define CIRCBUF_HOSTIF_LOCK_OTHER       0
#endif


//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------
/**
* \brief Host interface queue buffer lock
*
* The structure defines the shared buffer lock for a host interface queue.
*/
typedef struct
{
    UINT8           turn;           ///< Flag to determine the processor holding or requesting the lock
    UINT8           afFlag[2];      ///< Flag to request lock for the processor
    UINT8           reserved;       ///< Reserved
} tCircBufHostifLock;

/**
* \brief Host interface queue buffer
*
* The structure defines the shared memory buffer for a host interface queue.
*/
typedef struct
{
    tCircBufHostifLock  lock;           ///< Circbuf lock
    tCircBufHeader      circBufHeader;  ///< Circbuf Header
} tCircBufHostiBuffer;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
tCircBufInstance        instance_l[NR_OF_CIRC_BUFFERS];

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
#define GET_QUEUE_BUF_BASE(pHeader) \
    ((tCircBufHostiBuffer*)(((size_t)pHeader) - (size_t)&((tCircBufHostiBuffer*)0)->circBufHeader))

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Create circular buffer instance

The function allocates the memory needed for the circular buffer instance.

\param[in]      id_p                ID of the circular buffer.
\param[in]      fNew_p              The parameter determines if a new circular buffer
                                    instance should be created (TRUE) or if it should
                                    connect to an existing instance (FALSE).

\return The function returns the pointer to the buffer instance or NULL on error.

\ingroup module_lib_circbuf
*/
//------------------------------------------------------------------------------
tCircBufInstance* circbuf_createInstance(UINT8 id_p, BOOL fNew_p)
{
    tCircBufInstance*           pInstance;

    UNUSED_PARAMETER(fNew_p);

    pInstance = &instance_l[id_p];

    if (hostifInstance[id_p] == kHostifInstIdInvalid)
    {
        // Queue must use local resources
        pInstance->pCircBufArchInstance = NULL;
    }
    else
    {   // Queue must use host interface
        tHostifInstance pHostif;

        // Get host interface instance
        pHostif = hostif_getInstance(0);
        if (pHostif == NULL)
        {
            DEBUG_LVL_ERROR_TRACE("%s getting hostif instance failed!\n", __func__);
            return NULL;
        }

        pInstance->pCircBufArchInstance = (void*)pHostif;
    }

    pInstance->bufferId = id_p;

    return pInstance;
}

//------------------------------------------------------------------------------
/**
\brief  Free circular buffer instance

The function frees the allocated memory used by the circular buffer instance.

\param[in]      pInstance_p         Pointer to circular buffer instance.

\ingroup module_lib_circbuf
*/
//------------------------------------------------------------------------------
void circbuf_freeInstance(tCircBufInstance* pInstance_p)
{
    UNUSED_PARAMETER(pInstance_p);
}

//------------------------------------------------------------------------------
/**
\brief  Allocate memory for circular buffer

The function allocates the memory needed for the circular buffer.

Circular buffer instances for host interface use memory buffers provided by
hostif drivers. All other instances are allocated locally.

\param[in]      pInstance_p         Pointer to the circular buffer instance.
\param[in,out]  pSize_p             Size of memory to allocate.
                                    Returns the actually allocated buffer size.

\return The function returns a tCircBufError error code.

\ingroup module_lib_circbuf
*/
//------------------------------------------------------------------------------
tCircBufError circbuf_allocBuffer(tCircBufInstance* pInstance_p, size_t* pSize_p)
{
    size_t size;

    // Check parameter validity
    ASSERT(pInstance_p != NULL);
    ASSERT(pSize_p != NULL);

    size = *pSize_p;

    if (pInstance_p->pCircBufArchInstance == NULL)
    {
        // Allocate requested size + header
        size += sizeof(tCircBufHeader);

        pInstance_p->pCircBufHeader = (tCircBufHeader*)OPLK_MALLOC(size);

        if (pInstance_p->pCircBufHeader == NULL)
        {
            DEBUG_LVL_ERROR_TRACE("%s() malloc failed!\n", __func__);
            return kCircBufNoResource;
        }

        pInstance_p->pCircBuf = (UINT8*)pInstance_p->pCircBufHeader + sizeof(tCircBufHeader);

        // Return buffer size: pSize_p already holds the right value!
    }
    else
    {   // Queue must use host interface
        tHostifReturn           ret;
        tHostifInstance         pHostif = (tHostifInstance*)pInstance_p->pCircBufArchInstance;
        void*                   pBufBase;
        size_t                  bufSize;
        tCircBufHostiBuffer*    pHostifBuffer;

        // Get buffer for queue
        ret = hostif_getBuf(pHostif, hostifInstance[pInstance_p->bufferId],
                            &pBufBase, &bufSize);
        if (ret != kHostifSuccessful)
        {
            DEBUG_LVL_ERROR_TRACE("%s getting hostif buffer instance failed with 0x%X!\n",
                                  __func__, ret);
            return kCircBufNoResource;
        }

        // Check if there is enough memory available
        if (size > bufSize)
        {
            DEBUG_LVL_ERROR_TRACE("%s Hostif buffer (id=%d) only provides %lu Bytes instead of %lu Bytes!\n",
                                  __func__, pInstance_p->bufferId, (ULONG)bufSize, (ULONG)size);
            return kCircBufNoResource;
        }

        pHostifBuffer = (tCircBufHostiBuffer*)pBufBase;

        pInstance_p->pCircBufHeader = &(pHostifBuffer->circBufHeader);
        pInstance_p->pCircBuf = (UINT8*)pHostifBuffer + sizeof(tCircBufHostiBuffer);
        size -= sizeof(tCircBufHostiBuffer);

        // Return buffer size
        *pSize_p = size;

        OPLK_MEMSET(&pHostifBuffer->lock, 0, sizeof(pHostifBuffer->lock));
    }

    return kCircBufOk;
}

//------------------------------------------------------------------------------
/**
\brief  Free memory used by circular buffer

The function frees the allocated memory used by the circular buffer.

\param[in]      pInstance_p         Pointer to circular buffer instance.

\ingroup module_lib_circbuf
*/
//------------------------------------------------------------------------------
void circbuf_freeBuffer(tCircBufInstance* pInstance_p)
{
    // Check parameter validity
    ASSERT(pInstance_p != NULL);

    if (pInstance_p->pCircBufArchInstance == NULL)
        OPLK_FREE(pInstance_p->pCircBufHeader);
}

//------------------------------------------------------------------------------
/**
\brief  Connect to circular buffer

The function connects the calling thread to the circular buffer.

\param[in]      pInstance_p         Pointer to circular buffer instance.

\return The function returns a tCircBufError error code.

\ingroup module_lib_circbuf
*/
//------------------------------------------------------------------------------
tCircBufError circbuf_connectBuffer(tCircBufInstance* pInstance_p)
{
    // Check parameter validity
    ASSERT(pInstance_p != NULL);

    if (pInstance_p->pCircBufArchInstance != NULL)
    {   // Queue must use host interface
        tHostifReturn           ret;
        tHostifInstance         pHostif = (tHostifInstance*)pInstance_p->pCircBufArchInstance;
        void*                   pBufBase;
        size_t                  bufSize;
        tCircBufHostiBuffer*    pHostifBuffer;

        // Get buffer for queue
        ret = hostif_getBuf(pHostif, hostifInstance[pInstance_p->bufferId],
                            &pBufBase, &bufSize);
        if (ret != kHostifSuccessful)
        {
            DEBUG_LVL_ERROR_TRACE("%s getting hostif buffer instance failed with 0x%X!\n",
                                  __func__, ret);
            return kCircBufNoResource;
        }

        pHostifBuffer = (tCircBufHostiBuffer*)pBufBase;

        pInstance_p->pCircBufHeader = &(pHostifBuffer->circBufHeader);
        pInstance_p->pCircBuf = (UINT8*)pHostifBuffer + sizeof(tCircBufHostiBuffer);
    }

    return kCircBufOk;
}

//------------------------------------------------------------------------------
/**
\brief  Disconnect from circular buffer

The function disconnects the calling thread from the circular buffer.

\param[in]      pInstance_p         Pointer to circular buffer instance.

\ingroup module_lib_circbuf
*/
//------------------------------------------------------------------------------
void circbuf_disconnectBuffer(tCircBufInstance* pInstance_p)
{
    UNUSED_PARAMETER(pInstance_p);
}

//------------------------------------------------------------------------------
/**
\brief  Lock circular buffer

The function enters a locked section of the circular buffer.

The locking between two processors is achieved using Peterson's algorithm
(https://en.wikipedia.org/wiki/Peterson's_algorithm).

\param[in]      pInstance_p         Pointer to circular buffer instance.

\ingroup module_lib_circbuf
*/
//------------------------------------------------------------------------------
void circbuf_lock(tCircBufInstance* pInstance_p)
{
    // Check parameter validity
    ASSERT(pInstance_p != NULL);

    if (pInstance_p->pCircBufArchInstance != NULL)
    {
        tCircBufHostiBuffer*    pHostifBuf = GET_QUEUE_BUF_BASE(pInstance_p->pCircBufHeader);
        tCircBufHostifLock*     pLock = &pHostifBuf->lock;
        UINT8                   local = CIRCBUF_HOSTIF_LOCK_LOCAL;
        UINT8                   other = CIRCBUF_HOSTIF_LOCK_OTHER;

        OPLK_DCACHE_INVALIDATE(pLock, sizeof(*pLock));

        OPLK_IO_WR8(&pLock->afFlag[local], 1);
        OPLK_DCACHE_FLUSH(&pLock->afFlag[local], sizeof(pLock->afFlag[local]));

        OPLK_IO_WR8(&pLock->turn, other);
        OPLK_DCACHE_FLUSH(&pLock->turn, sizeof(pLock->turn));

        OPLK_MEMBAR();

        do
        {
            OPLK_DCACHE_INVALIDATE(pLock, sizeof(*pLock));
        } while (OPLK_IO_RD8(&pLock->afFlag[other]) && OPLK_IO_RD8(&pLock->turn) == other);
    }
    else
    {
        // Global interrupt disable only for local queues
        target_enableGlobalInterrupt(FALSE);
    }
}

//------------------------------------------------------------------------------
/**
\brief  Unlock circular buffer

The function leaves a locked section of the circular buffer.

\param[in]      pInstance_p         Pointer to circular buffer instance.

\ingroup module_lib_circbuf
*/
//------------------------------------------------------------------------------
void circbuf_unlock(tCircBufInstance* pInstance_p)
{
    // Check parameter validity
    ASSERT(pInstance_p != NULL);

    if (pInstance_p->pCircBufArchInstance != NULL)
    {
        tCircBufHostiBuffer* pHostifBuf = GET_QUEUE_BUF_BASE(pInstance_p->pCircBufHeader);
        tCircBufHostifLock*  pLock = &pHostifBuf->lock;
        UINT8                local = CIRCBUF_HOSTIF_LOCK_LOCAL;

        OPLK_IO_WR8(&pLock->afFlag[local], 0);
        OPLK_DCACHE_FLUSH(&pLock->afFlag[local], sizeof(pLock->afFlag[local]));
    }
    else
    {
        target_enableGlobalInterrupt(TRUE);
    }
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

/// \}
