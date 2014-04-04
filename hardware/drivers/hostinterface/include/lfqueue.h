/**
********************************************************************************
\file   lfqueue.h

\brief  This is the interface description of a lock-free queue.

The lock-free queue implementation enables an independent producer- and
consumer-process to access the shared resource without critical sections.
This lock-free queue does not support multiple producer- and consumer-processes!

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

#ifndef _INC_lfqueue_H_
#define _INC_lfqueue_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include "hostiflib_target.h"

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#define QUEUE_MAX_ENTRIES   32768U
#define QUEUE_MAX_PAYLOAD   (2048U + 24U)   ///< sizeof(tEvent) + ARG

#define QUEUE_MAGIC         0x1234

//------------------------------------------------------------------------------
// typedef
//------------------------------------------------------------------------------

/**
\brief Queue instance return type

The interface provides a specific error code.
*/
typedef enum eQueueReturn
{
    kQueueSuccessful        = 0x0,      ///< call was successful
    kQueueInvalidParameter  = 0x1,      ///< the call parameters are invalid
    kQueueNoResource        = 0x2,      ///< no resource available
    kQueueWrongCaller       = 0x3,      ///< the caller instance is invalid
    kQueueHwError           = 0x4,      ///< hardware error
    kQueueIsInReset         = 0x6,      ///< queue instance is in reset state
    kQueueEmpty             = 0x11,     ///< the queue instance is empty
    kQueueFull              = 0x12,     ///< the queue instance is full
    kQueueInvalidEntry      = 0x13,     ///< the read queue entry is invalid
    kQueueAlignment         = 0x20,     ///< wrong alignment

} tQueueReturn;

/**
\brief Queue role

When creating a queue the role has to be determined of the queue instance.
*/
typedef enum eQueueRole
{
    kQueueProducer,                     ///< caller instance is producer
    kQueueConsumer,                     ///< caller instance is consumer
    kQueueBoth                          ///< caller is producer and consumer
} tQueueRole;

/**
\brief Function type definition for critical section callback

This function callback is called if a critical section is entered. The function
should enable or disable global interrupts.

\param fEnable_p    Enter/Exit critical section
*/
typedef void (*tQueueCriticalSection) (BOOL fEnable_p);

/**
\brief Queue instance configuration

The queue creation is directed by the parameters in this structure.
*/
typedef struct sQueueConfig
{
    tQueueRole      queueRole;        ///< Queue instance role
    BOOL            fAllocHeap;       ///< Allocate queue in heap memory
    UINT8*          pBase;
        ///< if fAllocHeap is FALSE, this base address is used
    UINT16          span;
        ///< size of queue buffer, must be power of 2 + sizeof(tQueueBufferHdr)
    tQueueCriticalSection   pfnCriticalSection; ///< Critical section callback
} tQueueConfig;

typedef void* tQueueInstance;

//------------------------------------------------------------------------------
// function prototypes
//------------------------------------------------------------------------------

#ifdef __cplusplus
extern "C" {
#endif

tQueueReturn lfq_create(tQueueConfig* pQueueConfig_p,
                        tQueueInstance* ppInstance_p);
tQueueReturn lfq_delete(tQueueInstance pInstance_p);

tQueueReturn lfq_reset(tQueueInstance pInstance_p);

tQueueReturn lfq_checkEmpty(tQueueInstance pInstance_p,
                            BOOL* pfIsEmpty_p);
tQueueReturn lfq_getEntryCount(tQueueInstance pInstance_p,
                               UINT16* pEntryCount_p);

tQueueReturn lfq_entryEnqueue(tQueueInstance pInstance_p,
                              UINT8* pData_p, UINT16 size_p);
tQueueReturn lfq_entryDequeue(tQueueInstance pInstance_p,
                              UINT8* pData_p, UINT16* pSize_p);

#ifdef __cplusplus
}
#endif

#endif /* _INC_lfqueue_H_ */

