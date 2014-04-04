/**
********************************************************************************
\file   lfqueue.c

\brief  This is the implementation of a lock-free queue.

The lock-free queue implementation enables an independent producer- and
consumer-process to access the shared resource without critical sections.
This lock-free queue does not support multiple producer- and consumer-processes!

\ingroup module_hostiflib
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
#include "lfqueue.h"

#include <stdlib.h>
#include <string.h>

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
#define ENTRY_MIN_SIZE      4U      ///< UINT32-alignment

#define ALIGN32(ptr)        (((UINT32)(ptr) + 3U) & 0xFFFFFFFCU)
                                    ///< aligns the pointer to UINT32 (4 byte)
#define UNALIGNED32(ptr)    ((UINT32)(ptr) & 3U)
                                    ///< checks if the pointer is UINT32-aligned

//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------

/**
\brief Queue entry header

The queue entry header enables to identify a valid queue entry and its size.
For future enhancements a reserved UINT32 is available.
Note: This struct has to be UINT32-aligned!
*/
typedef struct sEntryHeader
{
    UINT16          magic;              ///< Magic word to identify an entry
    UINT16          payloadSize;        ///< Provides the entry's payload size
    UINT8           aReserved[4];       ///< reserved
} tEntryHeader;

/**
\brief Queue entry

The queue entry is built out of a header and its payload.
*/
typedef struct sQueueEntry
{
    tEntryHeader    header;                         ///< Queue entry header
    UINT8           aPayload[QUEUE_MAX_PAYLOAD];    ///< Entry payload
} tQueueEntry;

/**
\brief Queue index data type

Determines the data type of an index (write and read).
*/
typedef UINT16 tIndex;

/**
\brief Both queue indices data type

Determines the data type of both indices (write and read).
*/
typedef UINT32 tBothIndices;

/**
\brief Queue indices

These are the queue indices
*/
typedef union Indices
{
    struct
    {
        tIndex      write;              ///< write index
        tIndex      read;               ///< read index
    } ind;
    tBothIndices    bothIndices;        ///< combined indices
} tIndices;

/**
\brief Queue indices hw access

These are the queue indices
*/
typedef union uIndicesHw
{
    volatile tIndices       set;        ///< use this for setting individual
    volatile tBothIndices   get;        ///< use this to get indices
    volatile tBothIndices   reset;      ///< use this to reset indices
} tIndicesHw;

/**
\brief Queue state

This type defines valid queue states.
*/
typedef enum eQueueState
{
    kQueueStateInvalid      = 0,        ///< queue invalid
    kQueueStateReset        = 1,        ///< queue is in reset
    kQueueStateOperational  = 2,        ///< queue is operational
} tQueueState;

/**
\brief Queue buffer header

This is the header of a queue buffer, with a size of 16 byte.
*/
typedef struct sQueueBufferHdr
{
    volatile UINT8      state;          ///< queue state
    volatile UINT8      aReserved[3];
    tIndicesHw          spaceIndices;   ///< gives the offset within the queue
    tIndicesHw          entryIndices;   ///< gives the number of entries
    volatile UINT32     reserved;
} tQueueBufferHdr;

/**
\brief Queue buffer header

This structure is shared by the queue consumer and producer.
Note that the data section starting with data must have a span of power 2
(e.g. 1 kB, 2 kB or 4 kB).
*/
typedef struct sQueueBuffer
{
    tQueueBufferHdr     header;         ///< queue buffer header
    volatile UINT8      data;           ///< start of data section
} tQueueBuffer;

/**
\brief Queue instance type

The queue instance type holds the queue configuration, buffer information
and a local copy of the indices.
Note that the members IndicesLocal_m, freeSpace and usedSpace will be
updated to the most current state of the shared queue with the function
getQueueState.
*/
typedef struct sQueue
{
    tQueueConfig    config;             ///< Copy of the queue configuration
    UINT8*          pBase;              ///< Queue base address
    UINT16          span;               ///< Queue span
    struct
    {
        tIndices    spaceIndices;       ///< Local copy of memory space indices
        UINT16      freeSpace;          ///< Local copy of free memory space
        UINT16      usedSpace;          ///< Local copy of used memory space
        tIndices    entryIndices;       ///< Local copy of entry indices
        UINT16      usedEntries;        ///< Local copy of used entries
    } local;
    UINT16          maxEntries;         ///< Maximum addressable entries
    UINT16          queueBufferSpan;    ///< queue buffer span
    tQueueBuffer*   pQueueBuffer;       ///< pointer to queue buffer
} tQueue;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static void freePtr(void* p);
static void criticalSection(tQueue* pQueue_p, BOOL fEnable_p);

HOSTIF_INLINE static void getHwQueueBufferHeader(tQueue* pQueue_p);
HOSTIF_INLINE static tQueueState getHwQueueState(tQueue* pQueue_p);
HOSTIF_INLINE static void setHwQueueState(tQueue* pQueue_p, tQueueState State_p);
HOSTIF_INLINE static void setHwQueueWrite(tQueue* pQueue_p);
HOSTIF_INLINE static void setHwQueueRead(tQueue* pQueue_p);
HOSTIF_INLINE static void resetHwQueue(tQueue* pQueue_p);

HOSTIF_INLINE static UINT16 getOffsetInCirBuffer(tQueue* pQueue_p, UINT16 index_p);

HOSTIF_INLINE static BOOL checkMagicValid(tEntryHeader* pHeader_p);
HOSTIF_INLINE static BOOL checkPayloadFitable(tQueue* pQueue_p, UINT16 payloadSize_p);
HOSTIF_INLINE static BOOL checkQueueEmpty(tQueue* pQueue_p);

HOSTIF_INLINE static void writeHeader(tQueue* pQueue_p, tEntryHeader* pHeader_p);
HOSTIF_INLINE static void writeData(tQueue* pQueue_p, UINT8* pData_p, UINT16 size_p);
HOSTIF_INLINE static void writeCirMemory(tQueue* pQueue_p, UINT16 offset_p,
                                         UINT8* pSrc_p, UINT16 srcSpan_p);

HOSTIF_INLINE static void readHeader(tQueue* pQueue_p, tEntryHeader* pHeader_p);
HOSTIF_INLINE static void readData(tQueue* pQueue_p, UINT8* pData_p, UINT16 size_p);
HOSTIF_INLINE static void readCirMemory(tQueue* pQueue_p, UINT16 offset_p,
                                        UINT8* pDst_p, UINT16 dstSpan_p);

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief    Create a lock-free queue instance

This function creates a lock-free queue instance, and completely initializes
the queue depending on the pQueueConfig_p parameter. After this function call
the queue is usable.

\param  pQueueConfig_p          The caller provides configuration parameters
                                with this parameter.
\param  ppInstance_p            The function returns with this double-pointer,
                                the created instance pointer. (return)

\return tQueueReturn
\retval kQueueSuccessful        The queue is created successfully with the
                                provided configuration parameters
\retval kQueueInvalidParamter   The parameter pointers are NULL
\retval kQueueAlignment         The allocated buffer queue does not satisfy
                                UINT32-alignment
\retval kQueueNoResource        Either the heap allocation fails or the queue
                                size exceeds the limit QUEUE_MAX_ENTRIES
\retval kQueueHwError           The queue producer is not able to reset the
                                queue indices (e.g. hardware error)

\ingroup module_hostiflib
*/
//------------------------------------------------------------------------------
tQueueReturn lfq_create(tQueueConfig* pQueueConfig_p,
                        tQueueInstance* ppInstance_p)
{
    tQueue* pQueue = NULL;

    if (pQueueConfig_p == NULL || ppInstance_p == NULL)
        return kQueueInvalidParameter;

    if (UNALIGNED32(pQueueConfig_p->span - sizeof(tQueueBufferHdr)))
        return kQueueAlignment;

    pQueue = (tQueue*)malloc(sizeof(tQueue));

    if (pQueue == NULL)
        return kQueueNoResource;

    memset(pQueue, 0, sizeof(tQueue));

    /// store configuration
    pQueue->config = *pQueueConfig_p;

    /// allocate memory for queue
    pQueue->span = pQueue->config.span;

    if (pQueue->config.fAllocHeap != FALSE)
        pQueue->pBase = (UINT8*)HOSTIF_UNCACHED_MALLOC(pQueue->span);
    else
        pQueue->pBase = pQueue->config.pBase;

    /// kill instance if allocation is faulty
    if (pQueue->pBase == NULL || UNALIGNED32(pQueue->pBase))
    {
        lfq_delete(pQueue);
        return kQueueNoResource;
    }

    /// set queue buffer
    pQueue->pQueueBuffer = (tQueueBuffer*)pQueue->pBase;
    pQueue->queueBufferSpan = pQueue->span - sizeof(tQueueBufferHdr);

    /// initialize max entries (= mask)
    pQueue->maxEntries = pQueue->queueBufferSpan / ENTRY_MIN_SIZE;

    if (pQueue->maxEntries > QUEUE_MAX_ENTRIES)
    {
        lfq_delete(pQueue);
        return kQueueNoResource;
    }

    // Set queue to reset state, first enqueue resets the queue
    setHwQueueState(pQueue, kQueueStateReset);

    /// return initialized queue instance
    *ppInstance_p = pQueue;

    return kQueueSuccessful;
}

//------------------------------------------------------------------------------
/**
\brief    Delete a lock-free queue instance

This function deletes a lock-free queue instance. After this function call
the queue must not be used!

\param  pInstance_p             The queue instance that should be deleted

\return tQueueReturn
\retval kQueueSuccessful        The queue instance is deleted successfully
\retval kQueueInvalidParamter   The queue instance is invalid

\ingroup module_hostiflib
*/
//------------------------------------------------------------------------------
tQueueReturn lfq_delete(tQueueInstance pInstance_p)
{
    tQueue* pQueue = (tQueue*)pInstance_p;

    if (pQueue == NULL)
        return kQueueInvalidParameter;

    switch (pQueue->config.queueRole)
    {
        case kQueueProducer:
            break;

        case kQueueConsumer:
        case kQueueBoth:
            setHwQueueState(pQueue, kQueueStateInvalid);
    }

    /// free memory for queue
    if (pQueue->config.fAllocHeap != FALSE)
        HOSTIF_UNCACHED_FREE(pQueue->pBase);

    /// free memory for queue instance
    freePtr(pQueue);

    return kQueueSuccessful;
}

//------------------------------------------------------------------------------
/**
\brief    Resets the queue instance

This function resets the queue of the given instance

\param  pInstance_p             The queue instance that should be reset

\return tQueueReturn
\retval kQueueSuccessful        The returned base address is valid
\retval kQueueInvalidParamter   The parameter pointers are NULL
\retval kQueueWrongCaller       Producer is not allowed to reset queue

\ingroup module_hostiflib
*/
//------------------------------------------------------------------------------
tQueueReturn lfq_reset(tQueueInstance pInstance_p)
{
    tQueueReturn Ret = kQueueSuccessful;
    tQueue* pQueue = (tQueue*)pInstance_p;

    if (pQueue == NULL)
    {
        Ret = kQueueInvalidParameter;
        goto Exit;
    }

    if (pQueue->config.queueRole == kQueueProducer)
    {
        Ret = kQueueWrongCaller;
        goto Exit;
    }

    /// signalize reset of the queue
    setHwQueueState(pQueue, kQueueStateReset);

Exit:
    return Ret;
}

//------------------------------------------------------------------------------
/**
\brief    Checks if the queue instance is empty

This function returns the empty state of the queue instance (TRUE = is empty).

\param  pInstance_p             The queue instance of interest
\param  pfIsEmpty_p             Provides the queue empty state

\return tQueueReturn
\retval kQueueSuccessful        The returned base address is valid
\retval kQueueInvalidParamter   The parameter pointers are NULL

\ingroup module_hostiflib
*/
//------------------------------------------------------------------------------
tQueueReturn lfq_checkEmpty(tQueueInstance pInstance_p,
                            BOOL* pfIsEmpty_p)
{
    tQueueReturn Ret = kQueueSuccessful;
    tQueue* pQueue = (tQueue*)pInstance_p;

    if (pQueue == NULL || pfIsEmpty_p == NULL)
        return kQueueInvalidParameter;

    // Enter critical section
    criticalSection(pQueue, TRUE);

    switch (getHwQueueState(pQueue))
    {
        case kQueueStateOperational:
            break;

        default:
            // Consumer must see an empty and producer a full queue!
            if (pQueue->config.queueRole == kQueueConsumer)
                *pfIsEmpty_p = TRUE;
            else
                *pfIsEmpty_p = FALSE;

            goto Exit;
    }

    getHwQueueBufferHeader(pQueue);

    *pfIsEmpty_p = checkQueueEmpty(pQueue);

Exit:
    // Exit critical section
    criticalSection(pQueue, FALSE);

    return Ret;
}

//------------------------------------------------------------------------------
/**
\brief    Obtains the number of entries available in the given queue

This function returns the number of entries in the queue instance.

\param  pInstance_p             The queue instance of interest
\param  pEntryCount_p           Provides the number of added entries

\return tQueueReturn
\retval kQueueSuccessful        The returned base address is valid
\retval kQueueInvalidParamter   The parameter pointers are NULL

\ingroup module_hostiflib
*/
//------------------------------------------------------------------------------
tQueueReturn lfq_getEntryCount(tQueueInstance pInstance_p,
                               UINT16* pEntryCount_p)
{
    tQueueReturn Ret = kQueueSuccessful;
    tQueue* pQueue = (tQueue*)pInstance_p;

    if (pQueue == NULL || pEntryCount_p == NULL)
        return kQueueInvalidParameter;

    // Enter critical section
    criticalSection(pQueue, TRUE);

    switch (getHwQueueState(pQueue))
    {
        case kQueueStateOperational:
            break;
        default:
            // Consumer must see an empty and producer a full queue!
            if (pQueue->config.queueRole == kQueueConsumer)
                *pEntryCount_p = 0;
            else
                *pEntryCount_p = pQueue->maxEntries;

            goto Exit;
    }

    getHwQueueBufferHeader(pQueue);

    *pEntryCount_p = pQueue->local.usedEntries;

Exit:
    // Exit critical section
    criticalSection(pQueue, FALSE);

    return Ret;
}

//------------------------------------------------------------------------------
/**
\brief    Enqueue an entry into the queue instance

This function enqueues an entry (pEntry_p) into the given queue instance
(pInstance_p).
Note that the entry which pEntry_p points to has to be initialized with the
payload and its size (magic and reserved are set by this function).

\param  pInstance_p             The queue instance of interest
\param  pData_p                 Data to be inserted
\param  size_p                  Size of data to be inserted

\return tQueueReturn
\retval kQueueSuccessful        Entry is enqueued successfully
\retval kQueueInvalidParamter   The parameter pointers are NULL
\retval kQueueAlignment         The entry is not UINT32 aligned
\retval kQueueFull              The queue instance is full
\retval kQueueHwError           Queue is invalid

\ingroup module_hostiflib
*/
//------------------------------------------------------------------------------
tQueueReturn lfq_entryEnqueue(tQueueInstance pInstance_p,
                              UINT8* pData_p, UINT16 size_p)
{
    tQueueReturn    Ret = kQueueSuccessful;
    tQueue*         pQueue = (tQueue*)pInstance_p;
    UINT16          entryPayloadSize;
    tEntryHeader    entryHeader;

    if (pQueue == NULL || pData_p == NULL || size_p > QUEUE_MAX_PAYLOAD)
        return kQueueInvalidParameter;

    // Enter critical section
    criticalSection(pQueue, TRUE);

    switch (getHwQueueState(pQueue))
    {
        case kQueueStateOperational:
            break;

        case kQueueStateReset:
            // Consumer requests reset, so do it
            resetHwQueue(pQueue);

            // Mark queue to be operational again
            setHwQueueState(pQueue, kQueueStateOperational);
            break;

        default:
        case kQueueStateInvalid:
            Ret = kQueueHwError;
            goto Exit;
    }

    if (UNALIGNED32(pData_p))
    {
        Ret = kQueueAlignment;
        goto Exit;
    }

    getHwQueueBufferHeader(pQueue);

    entryPayloadSize = ALIGN32(size_p);

    if (!checkPayloadFitable(pQueue, entryPayloadSize))
    {
        Ret = kQueueFull;
        goto Exit;
    }

    /// prepare header
    entryHeader.magic = QUEUE_MAGIC;
    entryHeader.payloadSize = entryPayloadSize;
    memset(entryHeader.aReserved, 0, sizeof(entryHeader.aReserved));

    writeHeader(pQueue, &entryHeader);

    writeData(pQueue, pData_p, entryPayloadSize);

    /// new element is written
    pQueue->local.entryIndices.ind.write += 1;

    switch (getHwQueueState(pQueue))
    {
        case kQueueStateOperational:
            // Queue is operational, write new indices to hw
            setHwQueueWrite(pQueue);
            break;
        case kQueueStateReset:
            // Consumer requests reset, dump the entry and return with full
            Ret = kQueueFull;
            goto Exit;
        default:
        case kQueueStateInvalid:
            Ret = kQueueHwError;
            goto Exit;
    }

Exit:
    // Exit critical section
    criticalSection(pQueue, FALSE);

    return Ret;
}

//------------------------------------------------------------------------------
/**
\brief    Dequeue an entry from the queue instance

This function dequeues an entry from the given queue instance to the provided
entry buffer (pEntry_p). The caller has to provide the buffer size, to which the
pointer pEntry_p points to.

\param  pInstance_p             The queue instance of interest
\param  pData_p                 Buffer to be used for extracting next entry
\param  pSize_p                 Size of the buffer, returns the actual size of
                                the entry

\return tQueueReturn
\retval kQueueSuccessful        The returned base address is valid
\retval kQueueInvalidParamter   The parameter pointers are NULL
\retval kQueueAlignment         The entry buffer is not UINT32 aligned
\retval kQueueEmpty             The queue instance is empty
\retval kQueueInvalidEntry      The read entry of the queue instance is
                                invalid (magic queue word is wrong!)
\retval kQueueNoResource        The provided entry buffer is too small

\ingroup module_hostiflib
*/
//------------------------------------------------------------------------------
tQueueReturn lfq_entryDequeue(tQueueInstance pInstance_p,
                              UINT8* pData_p, UINT16* pSize_p)
{
    tQueueReturn    Ret = kQueueSuccessful;
    tQueue*         pQueue = (tQueue*)pInstance_p;
    tEntryHeader    EntryHeader;
    UINT16          size;

    if (pQueue == NULL || pData_p == NULL)
        return kQueueInvalidParameter;

    if (UNALIGNED32(pData_p))
        return kQueueAlignment;

    // Enter critical section
    criticalSection(pQueue, TRUE);

    getHwQueueBufferHeader(pQueue);

    switch (getHwQueueState(pQueue))
    {
        case kQueueStateOperational:
            // Queue is operational, move on
            break;
        case kQueueStateReset:
            // Queue is in reset state, return with empty
            Ret = kQueueEmpty;
            goto Exit;
        default:
        case kQueueStateInvalid:
            // Queue is invalid
            Ret = kQueueHwError;
            goto Exit;
    }

    if(checkQueueEmpty(pQueue))
    {
        Ret = kQueueEmpty;
        goto Exit;
    }

    readHeader(pQueue, &EntryHeader);

    if (!checkMagicValid(&EntryHeader))
    {
        Ret = kQueueInvalidEntry;
        goto Exit;
    }

    size = ALIGN32(EntryHeader.payloadSize);

    if (size > *pSize_p)
    {
        Ret = kQueueNoResource;
        goto Exit;
    }

    readData(pQueue, pData_p, size);

    /// element is read
    pQueue->local.entryIndices.ind.read += 1;

    switch (getHwQueueState(pQueue))
    {
        case kQueueStateOperational:
            // Queue is operational, write new indices to hw
            setHwQueueRead(pQueue);
            break;
        case kQueueStateReset:
            // Queue is in reset state, return with empty
            Ret = kQueueEmpty;
            goto Exit;
        default:
        case kQueueStateInvalid:
            // Queue is invalid
            Ret = kQueueHwError;
            goto Exit;
    }

    /// return entry size
    *pSize_p = size;

Exit:
    // Exit critical section
    criticalSection(pQueue, FALSE);

    return Ret;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Free pointers which are not NULL

\param  p                       Pointer to be freed
*/
//------------------------------------------------------------------------------
static void freePtr(void* p)
{
    if (p != NULL)
        free(p);
}

//------------------------------------------------------------------------------
/**
\brief  Enable/Disable critical section

\param  pQueue_p    The queue instance of interest
\param  fEnable_p   Enable/Disable critical section
*/
//------------------------------------------------------------------------------
static void criticalSection(tQueue* pQueue_p, BOOL fEnable_p)
{
    if (pQueue_p->config.pfnCriticalSection != NULL)
    {
        pQueue_p->config.pfnCriticalSection(fEnable_p);
    }
}

//------------------------------------------------------------------------------
/**
\brief    Get queue buffer header from shared memory

This function reads the queue buffer instance header from the shared memory
and writes it to the queue instance.
This ensures reading queue indices consistently.

\param  pQueue_p                The queue instance of interest
*/
//------------------------------------------------------------------------------
static void getHwQueueBufferHeader(tQueue* pQueue_p)
{
    pQueue_p->local.spaceIndices.bothIndices =
            HOSTIF_RD32(pQueue_p->pQueueBuffer,
                        offsetof(tQueueBuffer, header.spaceIndices));

    pQueue_p->local.entryIndices.bothIndices =
            HOSTIF_RD32(pQueue_p->pQueueBuffer,
                        offsetof(tQueueBuffer, header.entryIndices));

    pQueue_p->local.usedSpace = pQueue_p->local.spaceIndices.ind.write -
            pQueue_p->local.spaceIndices.ind.read;

    pQueue_p->local.freeSpace =
            pQueue_p->maxEntries - pQueue_p->local.usedSpace;

    pQueue_p->local.usedEntries = pQueue_p->local.entryIndices.ind.write -
            pQueue_p->local.entryIndices.ind.read;
}

//------------------------------------------------------------------------------
/**
\brief    Get queue buffer state from shared memory

\param  pQueue_p                The queue instance of interest

\return The function returns the state of the queue instance.
*/
//------------------------------------------------------------------------------
static tQueueState getHwQueueState(tQueue* pQueue_p)
{
    return (tQueueState)HOSTIF_RD8(pQueue_p->pQueueBuffer,
                                   offsetof(tQueueBuffer, header.state));
}

//------------------------------------------------------------------------------
/**
\brief    Set queue buffer state to shared memory

\param  pQueue_p                The queue instance of interest
\param  State_p                 Queue state to be written
*/
//------------------------------------------------------------------------------
static void setHwQueueState(tQueue* pQueue_p, tQueueState State_p)
{
    HOSTIF_WR8(pQueue_p->pQueueBuffer,
               offsetof(tQueueBuffer, header.state), (UINT8)State_p);
}

//------------------------------------------------------------------------------
/**
\brief    Set queue buffer write index from local instance to shared memory

This function writes the local write indices to the shared memory.

\param  pQueue_p                The queue instance of interest
*/
//------------------------------------------------------------------------------
static void setHwQueueWrite(tQueue* pQueue_p)
{
    HOSTIF_WR16(pQueue_p->pQueueBuffer,
                offsetof(tQueueBuffer, header.spaceIndices.set.ind.write),
                pQueue_p->local.spaceIndices.ind.write);

    HOSTIF_WR16(pQueue_p->pQueueBuffer,
                offsetof(tQueueBuffer, header.entryIndices.set.ind.write),
                pQueue_p->local.entryIndices.ind.write);
}

//------------------------------------------------------------------------------
/**
\brief    Set queue buffer read index from local instance to shared memory

This function writes the local read indices to the shared memory.

\param  pQueue_p                The queue instance of interest
*/
//------------------------------------------------------------------------------
static void setHwQueueRead(tQueue* pQueue_p)
{
    HOSTIF_WR16(pQueue_p->pQueueBuffer,
                offsetof(tQueueBuffer, header.spaceIndices.set.ind.read),
                pQueue_p->local.spaceIndices.ind.read);

    HOSTIF_WR16(pQueue_p->pQueueBuffer,
                offsetof(tQueueBuffer, header.entryIndices.set.ind.read),
                pQueue_p->local.entryIndices.ind.read);
}

//------------------------------------------------------------------------------
/**
\brief    Reset the queue instance

This function resets the queue buffer instance by writing directly to the shared
memory region.

\param  pQueue_p                The queue instance of interest
*/
//------------------------------------------------------------------------------
static void resetHwQueue(tQueue* pQueue_p)
{
    HOSTIF_WR32(pQueue_p->pQueueBuffer,
                offsetof(tQueueBuffer, header.spaceIndices.reset), 0);

    HOSTIF_WR32(pQueue_p->pQueueBuffer,
                offsetof(tQueueBuffer, header.entryIndices.reset), 0);
}

//------------------------------------------------------------------------------
/**
\brief    Get offset in circular buffer

This function returns the offset of an index of a queue instance.

\param  pQueue_p                The queue instance of interest
\param  index_p                 The index of interest

\return The function returns the offset of the specified index.
*/
//------------------------------------------------------------------------------
static UINT16 getOffsetInCirBuffer(tQueue* pQueue_p, UINT16 index_p)
{
    return (index_p & (pQueue_p->maxEntries - 1)) * ENTRY_MIN_SIZE;
}

//------------------------------------------------------------------------------
/**
\brief    Check if a queue entry has a valid magic word

This function checks the magic word of a queue entry header.

\param  pHeader_p               Reference to the queue entry header.

\return The function returns TRUE if the queue entry header magic is valid.
*/
//------------------------------------------------------------------------------
static BOOL checkMagicValid(tEntryHeader* pHeader_p)
{
    if (pHeader_p->magic != QUEUE_MAGIC)
    {
        return FALSE;
    }
    else
    {
        return TRUE;
    }
}

//------------------------------------------------------------------------------
/**
\brief    Check if a entry will fit into the queue instance

This function checks if the specified payloadSize_p will fit into the queue
instance.

\param  pQueue_p                Queue instance of interest
\param  payloadSize_p           Payload size of the queue entry

\return The function returns TRUE if the entry will fit.
*/
//------------------------------------------------------------------------------
static BOOL checkPayloadFitable(tQueue* pQueue_p, UINT16 payloadSize_p)
{
    UINT16 space = (sizeof(tEntryHeader) + payloadSize_p) / ENTRY_MIN_SIZE;

    if (space > pQueue_p->local.freeSpace)
    {
        return FALSE;
    }
    else
    {
        return TRUE;
    }
}

//------------------------------------------------------------------------------
/**
\brief    Check if the queue instance is empty

Note that the queue empty state is checked with the local copy! In order to get
the most current empty state, the function getHwQueueBufferHeader() must be
called.

\param  pQueue_p                Queue instance of interest

\return The function returns TRUE if the queue instance is empty.
*/
//------------------------------------------------------------------------------
static BOOL checkQueueEmpty(tQueue* pQueue_p)
{
    return (pQueue_p->local.usedSpace == 0);
}

//------------------------------------------------------------------------------
/**
\brief    Write queue entry header to queue instance

\param  pQueue_p                Queue instance of interest
\param  pHeader_p               Reference to header to be written
*/
//------------------------------------------------------------------------------
static void writeHeader(tQueue* pQueue_p, tEntryHeader* pHeader_p)
{
    UINT16 offset = getOffsetInCirBuffer(pQueue_p,
                                         pQueue_p->local.spaceIndices.ind.write);

    writeCirMemory(pQueue_p, offset, (UINT8*)pHeader_p, sizeof(tEntryHeader));

    pQueue_p->local.spaceIndices.ind.write += sizeof(tEntryHeader) /
                                              ENTRY_MIN_SIZE;
}

//------------------------------------------------------------------------------
/**
\brief    Write queue entry payload to queue instance

\param  pQueue_p                Queue instance of interest
\param  pData_p                 Payload data to be written
\param  size_p                  Size of payload data to be written
*/
//------------------------------------------------------------------------------
static void writeData(tQueue* pQueue_p, UINT8* pData_p, UINT16 size_p)
{
    UINT16 offset = getOffsetInCirBuffer(pQueue_p,
                                         pQueue_p->local.spaceIndices.ind.write);

    writeCirMemory(pQueue_p, offset, pData_p, size_p);

    pQueue_p->local.spaceIndices.ind.write += size_p / ENTRY_MIN_SIZE;
}

//------------------------------------------------------------------------------
/**
\brief    Write data to circular memory

This function writes data from a source to a circular memory.

\param  pQueue_p                Queue instance of interest
\param  offset_p                Offset where to start writing to
\param  pSrc_p                  Source data base address
\param  srcSpan_p               Source data size
*/
//------------------------------------------------------------------------------
static void writeCirMemory(tQueue* pQueue_p, UINT16 offset_p,
                           UINT8* pSrc_p, UINT16 srcSpan_p)
{
    UINT8 *pDst = (UINT8*)(&pQueue_p->pQueueBuffer->data);
    UINT16 part;

    if (offset_p + srcSpan_p <= pQueue_p->queueBufferSpan)
    {
        memcpy(pDst + offset_p, pSrc_p, srcSpan_p);
    }
    else
    {
        /// mind the circular nature of this buffer!
        part = pQueue_p->queueBufferSpan - offset_p;

        /// copy to the buffer's end
        memcpy(pDst + offset_p, pSrc_p, part);

        /// copy the rest starting at the buffer's head
        memcpy(pDst, pSrc_p + part, srcSpan_p - part);
    }
}

//------------------------------------------------------------------------------
/**
\brief    Read queue entry header from queue instance

\param  pQueue_p                Queue instance of interest
\param  pHeader_p               Reference to data buffer that will be filled
                                with the read header
*/
//------------------------------------------------------------------------------
static void readHeader(tQueue* pQueue_p, tEntryHeader* pHeader_p)
{
    UINT16 offset = getOffsetInCirBuffer(pQueue_p,
                                         pQueue_p->local.spaceIndices.ind.read);

    readCirMemory(pQueue_p, offset, (UINT8*)pHeader_p, sizeof(tEntryHeader));

    pQueue_p->local.spaceIndices.ind.read += sizeof(tEntryHeader) /
                                             ENTRY_MIN_SIZE;
}

//------------------------------------------------------------------------------
/**
\brief    Read queue entry payload from queue instance

\param  pQueue_p                Queue instance of interest
\param  pData_p                 Reference to data buffer that will be filled
                                with the read payload
\param  size_p                  Size of data buffer
*/
//------------------------------------------------------------------------------
static void readData(tQueue* pQueue_p, UINT8* pData_p, UINT16 size_p)
{
    UINT16 offset = getOffsetInCirBuffer(pQueue_p,
                                         pQueue_p->local.spaceIndices.ind.read);

    readCirMemory(pQueue_p, offset, pData_p, size_p);

    pQueue_p->local.spaceIndices.ind.read += size_p / ENTRY_MIN_SIZE;
}

//------------------------------------------------------------------------------
/**
\brief    Read data from circular memory

This function reads data from a circular memory.

\param  pQueue_p                Queue instance of interest
\param  offset_p                Offset where to start reading from
\param  pDst_p                  Destination data base address
\param  dstSpan_p               Destination data size
*/
//------------------------------------------------------------------------------
static void readCirMemory(tQueue* pQueue_p, UINT16 offset_p,
                          UINT8* pDst_p, UINT16 dstSpan_p)
{
    UINT8* pSrc = (UINT8*)(&pQueue_p->pQueueBuffer->data);
    UINT16 part;

    if (offset_p + dstSpan_p <= pQueue_p->queueBufferSpan)
    {
        memcpy(pDst_p, pSrc + offset_p, dstSpan_p);
    }
    else
    {
        /// mind the circular nature of this buffer!
        part = pQueue_p->queueBufferSpan - offset_p;

        /// copy until the buffer's end
        memcpy(pDst_p, pSrc + offset_p, part);

        /// copy the rest starting at the buffer's head
        memcpy(pDst_p + part, pSrc, dstSpan_p - part);
    }
}

