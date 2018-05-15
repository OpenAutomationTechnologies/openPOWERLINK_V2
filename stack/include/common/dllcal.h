/**
********************************************************************************
\file   common/dllcal.h

\brief  Definitions for DLL CAL module

The file contains definitions for the DLL CAL module

Copyright (c) 2012, SYSTEC electronik GmbH
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
*******************************************************************************/
#ifndef _INC_common_dllcal_H_
#define _INC_common_dllcal_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <common/oplkinc.h>
#include <oplk/dll.h>

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

#ifndef CONFIG_DLLCAL_BUFFER_SIZE_TX_NMT
#define CONFIG_DLLCAL_BUFFER_SIZE_TX_NMT    32767
#endif

#ifndef CONFIG_DLLCAL_BUFFER_SIZE_TX_GEN
#define CONFIG_DLLCAL_BUFFER_SIZE_TX_GEN    32767
#endif

#ifndef CONFIG_DLLCAL_BUFFER_SIZE_TX_SYNC
#define CONFIG_DLLCAL_BUFFER_SIZE_TX_SYNC   8192
#endif

#ifndef CONFIG_DLLCAL_BUFFER_SIZE_TX_VETH
#define CONFIG_DLLCAL_BUFFER_SIZE_TX_VETH   8192
#endif

/* setup interface getting function for DLLCAL queue */
#if (CONFIG_DLLCAL_QUEUE == DIRECT_QUEUE)
#define GET_DLLKCAL_INTERFACE dllcaldirect_getInterface
#define GET_DLLUCAL_INTERFACE dllcaldirect_getInterface
#elif (CONFIG_DLLCAL_QUEUE == IOCTL_QUEUE)
#define GET_DLLKCAL_INTERFACE dllcalioctl_getInterface
#define GET_DLLUCAL_INTERFACE dllcalioctl_getInterface
#elif (CONFIG_DLLCAL_QUEUE == CIRCBUF_QUEUE)
#define GET_DLLKCAL_INTERFACE dllkcalcircbuf_getInterface
#define GET_DLLUCAL_INTERFACE dllucalcircbuf_getInterface
#else
#error "Unsupported DLLCAL_QUEUE"
#endif

//------------------------------------------------------------------------------
// typedef
//------------------------------------------------------------------------------
typedef struct
{
    tDllAsndServiceId       serviceId;
    tDllAsndFilter          filter;
} tDllCalAsndServiceIdFilter;

typedef struct
{
    tDllReqServiceId        service;
    UINT                    nodeId;
    UINT8                   soaFlag1;
} tDllCalIssueRequest;

/**
\brief enumerator for queue

This enumerator identifies DLLCal queue instance in order to differ between
the queues.
*/
typedef enum
{
    kDllCalQueueTxNmt        = 0x01, ///< TX NMT queue
    kDllCalQueueTxGen        = 0x02, ///< TX Generic queue
    kDllCalQueueTxSync       = 0x03, ///< Tx Sync queue
    kDllCalQueueTxVeth       = 0x04, ///< Virtual Ethernet Tx queue
} eDllCalQueue;

/**
\brief DLL CAL queue data type

Data type for the enumerator \ref eDllCalQueue.
*/
typedef UINT32 tDllCalQueue;

/**
\brief type for DLL CAL queue instance

The DllCalQueueInstance is used to identify the abstracted queue instance in
dll*cal. The queue itself is defined by its implementation (e.g. DIRECT or
SHB)
*/
typedef void* tDllCalQueueInstance;

typedef struct
{
    tOplkError (*pfnAddInstance)(tDllCalQueueInstance* ppDllCalQueue_p, tDllCalQueue DllCalQueue_p);
    tOplkError (*pfnDelInstance)(tDllCalQueueInstance pDllCalQueue_p);
    tOplkError (*pfnInsertDataBlock)(tDllCalQueueInstance pDllCalQueue_p, const void* pData_p, size_t dataSize_p);
    tOplkError (*pfnGetDataBlock)(tDllCalQueueInstance pDllCalQueue_p, void* pData_p, size_t* pDataSize_p);
    tOplkError (*pfnGetDataBlockCount)(tDllCalQueueInstance pDllCalQueue_p, UINT* pDataBlockCount_p);
    tOplkError (*pfnResetDataBlockQueue)(tDllCalQueueInstance pDllCalQueue_p);
} tDllCalFuncIntf;

//------------------------------------------------------------------------------
// function prototypes
//------------------------------------------------------------------------------
#ifdef __cplusplus
extern "C"
{
#endif

tDllCalFuncIntf* dllcaldirect_getInterface(void);
tDllCalFuncIntf* dllcalioctl_getInterface(void);
tDllCalFuncIntf* dllucalcircbuf_getInterface(void);
tDllCalFuncIntf* dllkcalcircbuf_getInterface(void);

#ifdef __cplusplus
}
#endif

#endif /* _INC_common_dllcal_H_ */
