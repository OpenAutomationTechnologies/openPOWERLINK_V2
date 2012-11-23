/**
********************************************************************************
\file   dllcal.h

\brief  Definitions for DLL CAL module

The file contains definitions for the DLL CAL module

Copyright (c) 2012, SYSTEC electronik GmbH
Copyright (c) 2012, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
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

#ifndef _INC_dllcal_H_
#define _INC_dllcal_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <EplDll.h>

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
/*
#ifndef EPL_DLLCAL_BUFFER_ID_RX
#define EPL_DLLCAL_BUFFER_ID_RX    "EplSblDllCalRx"
#endif

#ifndef EPL_DLLCAL_BUFFER_SIZE_RX
#define EPL_DLLCAL_BUFFER_SIZE_RX  32767
#endif
*/

#ifndef DLLCAL_BUFFER_ID_TX_NMT
#define DLLCAL_BUFFER_ID_TX_NMT     "EplSblDllCalTxNmt"
#endif

#ifndef DLLCAL_BUFFER_SIZE_TX_NMT
#define DLLCAL_BUFFER_SIZE_TX_NMT   32767
#endif

#ifndef DLLCAL_BUFFER_ID_TX_GEN
#define DLLCAL_BUFFER_ID_TX_GEN     "EplSblDllCalTxGen"
#endif

#ifndef DLLCAL_BUFFER_SIZE_TX_GEN
#define DLLCAL_BUFFER_SIZE_TX_GEN   32767
#endif

#ifndef DLLCAL_BUFFER_ID_TX_SYNC
#define DLLCAL_BUFFER_ID_TX_SYNC    "EplSblDllCalTxSync"
#endif

#ifndef DLLCAL_BUFFER_SIZE_TX_SYNC
#define DLLCAL_BUFFER_SIZE_TX_SYNC  8192
#endif

/* setup interface getting function for TX NMT queue */
#if (EPL_DLLCAL_TX_NMT_QUEUE == EPL_QUEUE_DIRECT)
#define GET_TX_NMT_INTERFACE dllcaldirect_getInterface
#elif (EPL_DLLCAL_TX_NMT_QUEUE == EPL_QUEUE_SHB)
#define GET_TX_NMT_INTERFACE dllcalshb_getInterface
#elif (EPL_DLLCAL_TX_NMT_QUEUE == EPL_QUEUE_HOSTINTERFACE)
#define GET_TX_NMT_INTERFACE dllcalhostif_getInterface
#else
#error "Unsupported TX_NMT_QUEUE"
#endif

/* setup interface getting function for TX GEN queue */
#if (EPL_DLLCAL_TX_NMT_QUEUE == EPL_QUEUE_DIRECT)
#define GET_TX_GEN_INTERFACE dllcaldirect_getInterface
#elif (EPL_DLLCAL_TX_NMT_QUEUE == EPL_QUEUE_SHB)
#define GET_TX_GEN_INTERFACE dllcalshb_getInterface
#elif (EPL_DLLCAL_TX_NMT_QUEUE == EPL_QUEUE_HOSTINTERFACE)
#define GET_TX_GEN_INTERFACE dllcalhostif_getInterface
#else
#error "Unsupported TX_GEN_QUEUE"
#endif

/* setup interface getting function for TX SYNC queue */
#if (EPL_DLLCAL_TX_NMT_QUEUE == EPL_QUEUE_DIRECT)
#define GET_TX_SYNC_INTERFACE dllcaldirect_getInterface
#elif (EPL_DLLCAL_TX_NMT_QUEUE == EPL_QUEUE_SHB)
#define GET_TX_SYNC_INTERFACE dllcalshb_getInterface
#elif (EPL_DLLCAL_TX_NMT_QUEUE == EPL_QUEUE_HOSTINTERFACE)
#define GET_TX_SYNC_INTERFACE dllcalhostif_getInterface
#else
#error "Unsupported TX_SYNC_QUEUE"
#endif


//------------------------------------------------------------------------------
// typedef
//------------------------------------------------------------------------------
typedef struct
{
    tEplDllAsndServiceId    serviceId;
    tEplDllAsndFilter       filter;
} tDllCalAsndServiceIdFilter;

typedef struct
{
    tEplDllReqServiceId     service;
    UINT                    nodeId;
    BYTE                    soaFlag1;
} tDllCalIssueRequest;

/**
\brief enumerator for Queue

This enumerator identifies DLLCal queue instance in order to differ between
the queues.
*/
typedef enum
{
    kDllCalQueueTxNmt        = 0x01, ///< TX NMT queue
    kDllCalQueueTxGen        = 0x02, ///< TX Generic queue
    kDllCalQueueTxSync       = 0x03, ///< Tx Sync queue
} tDllCalQueue;

/**
\brief type for DLL CAL queue instance

The DllCalQueueInstance is used to identify the abstracted queue instance in
EplDll*Cal. The queue itself is defined by its implementation (e.g. DIRECT or
SHB)
*/
typedef void* tDllCalQueueInstance;


typedef struct
{
    tEplKernel (* pfnAddInstance)(tDllCalQueueInstance* ppDllCalQueue_p, tDllCalQueue DllCalQueue_p);
    tEplKernel (* pfnDelInstance)(tDllCalQueueInstance pDllCalQueue_p);
    tEplKernel (* pfnInsertDataBlock)(tDllCalQueueInstance pDllCalQueue_p, BYTE *pData_p, UINT* pDataSize_p);
    tEplKernel (* pfnGetDataBlock)(tDllCalQueueInstance pDllCalQueue_p, BYTE *pData_p, UINT* pDataSize_p);
    tEplKernel (* pfnGetDataBlockCount)(tDllCalQueueInstance pDllCalQueue_p, ULONG* pDataBlockCount_p);
    tEplKernel (* pfnResetDataBlockQueue)(tDllCalQueueInstance pDllCalQueue_p, ULONG timeOutMs_p);
} tDllCalFuncIntf;

//------------------------------------------------------------------------------
// function prototypes
//------------------------------------------------------------------------------

#ifdef __cplusplus
extern "C" {
#endif

tDllCalFuncIntf* dllcaldirect_getInterface(void);
tDllCalFuncIntf* dllcalshb_getInterface(void);

#ifdef __cplusplus
}
#endif

#endif /* _INC_dllcal_H_ */


