/**
********************************************************************************
\file   hostiflib.h

\brief  Host Interface Library - High Level Driver Header

The Host Interface Library High Level Driver provides a software library for the
host interface IP-Core.
The hostiflib provides several features like queues and linear memory modules.

*******************************************************************************/

/*------------------------------------------------------------------------------
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
------------------------------------------------------------------------------*/

#ifndef _INC_HOSTIFLIB_H_
#define _INC_HOSTIFLIB_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include "hostiflib_target.h"

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#if HOSTIF_SIZE_DYNBUF0 < HOSTIF_SIZE_DYNBUF1
#define HOSTIF_DYNBUF_MAXSIZE   HOSTIF_SIZE_DYNBUF0
#else
#define HOSTIF_DYNBUF_MAXSIZE   HOSTIF_SIZE_DYNBUF1
#endif

//------------------------------------------------------------------------------
// typedef
//------------------------------------------------------------------------------
/**
\brief Return codes
*/
typedef enum eHostifReturn
{
    kHostifSuccessful     = 0x0000,       ///< no error / successful run
    kHostifNoResource     = 0x0001,       ///< resource could not be created
    kHostifInvalidParameter = 0x0002,     ///< function parameter invalid
    kHostifWrongProcInst  = 0x0003,       ///< Processor instance wrong
    kHostifWrongMagic     = 0x0010,       ///< magic word invalid
    kHostifWrongVersion   = 0x0011,       ///< hw/sw version mismatch
    kHostifHwWriteError   = 0x0012,       ///< write to hw failed
    kHostifBridgeDisabled = 0x0013,       ///< bridge is disabled
    kHostifBufferOverflow = 0x0020,       ///< buffer size overflow
    kHostifBufferEmpty    = 0x0021,       ///< buffer is empty
    kHostifBufferError    = 0x0023,       ///< buffer is faulty

    kHostifUnspecError    = 0xFFFF        ///< unspecified error
} tHostifReturn;

/**
\brief Command code

The command code has to be defined by higher layers
*/
typedef UINT16 tHostifCommand;

/**
\brief State code

The state code has to be defined by higher layers
*/
typedef UINT16 tHostifState;

/**
\brief Error/Return code

The error/return code has to be defined by higher layers
*/
typedef UINT16 tHostifError;

/**
\brief Processor instance

The processor instance determines if the caller is the Pcp or the Host.
*/
typedef enum eHostifProcInstance
{
    kHostifProcPcp        = 0,            ///< instance on PCP
    kHostifProcHost       = 1,            ///< instance on Host

} tHostifProcInstance;

/**
\brief Interrupt source

The enum determines the address interrupt source
*/
typedef enum eHostifIrqSrc
{
    kHostifIrqSrcSync       = 0,        ///< sync irq
    kHostifIrqSrcEvent      = 1,        ///< event irq
    kHostifIrqSrcAsyncTx    = 2,        ///< async tx irq
    kHostifIrqSrcAsyncRx    = 3,        ///< async rx irq
    kHostifIrqSrcLast                   ///< last entry in enum
} tHostifIrqSrc;

/**
\brief Function type definition for interrupt callback

This function callback is called for a given interrupt source, registerd by the
host.
*/
typedef void (*tHostifIrqCb) (void *pArg_p);

/**
\brief Instance Id

The instance ids for the resources for the Host.
*/
typedef enum eHostifDynInstanceId
{
    kHostifDynInstIdDynBufRxVeth = 0,     ///< dynamic buffer for RX Veth Queue
    kHostifDynInstIdDynBufK2U,            ///< dynamic buffer for K2U Queue
    kHostifDynInstIdLast                  ///< last instance id
} tHostifDynInstanceId;

/**
\brief Instance Id

The instance ids for the resources.
*/
typedef enum eHostifInstanceId
{
    kHostifInstIdErrCount = 0,            ///< error counters
    kHostifInstIdTxNmtQueue,              ///< NMT TX queue
    kHostifInstIdTxGenQueue,              ///< generic TX queue
    kHostifInstIdTxSyncQueue,             ///< sync TX queue
    kHostifInstIdTxVethQueue,             ///< VEth TX queue
    kHostifInstIdRxVethQueue,             ///< VEth RX queue
    kHostifInstIdK2UQueue,                ///< K2U Queue
    kHostifInstIdU2KQueue,                ///< U2K Queue
    kHostifInstIdTpdo,                    ///< Tpdo
    kHostifInstIdRpdo,                    ///< Rpdo
    kHostifInstIdLast                     ///< last instance id
} tHostifInstanceId;

/**
\brief Driver instance configuration

Configures the driver instance.
*/
typedef struct sHostifConfig
{
    tHostifProcInstance   ProcInstance; ///< Processor instance (Pcp/Host)

} tHostifConfig;

/**
\brief Function type definition for queue callback

This function callback is invoked by the hostif_process if the corresponding queue
is not empty.
*/
typedef void (*tQueueCb) (void *pArg_p);

/**
\brief Hostiflib queue instance
*/
typedef void* tHostifQueueInstance;

/**
\brief Hostiflib linmem instance
*/
typedef void* tHostifLimInstance;

/**
\brief Hostiflib dynamic buffer instance (for host only)
*/
typedef void* tHostifDynBufsas;

/**
\brief Driver Instance
*/
typedef void* tHostifInstance;

//------------------------------------------------------------------------------
// function prototypes
//------------------------------------------------------------------------------

#ifdef __cplusplus
extern "C" {
#endif

tHostifReturn hostif_create (tHostifConfig *pConfig_p, tHostifInstance *ppInstance_p);
tHostifReturn hostif_delete (tHostifInstance pInstance_p);
tHostifInstance hostif_getInstance (tHostifProcInstance Instance_p);

tHostifReturn hostif_process (tHostifInstance pInstance_p);

tHostifReturn hostif_irqRegHdl (tHostifInstance pInstance_p,
        tHostifIrqSrc irqSrc_p, tHostifIrqCb pfnCb_p);
tHostifReturn hostif_irqSourceEnable (tHostifInstance pInstance_p,
        tHostifIrqSrc irqSrc_p, BOOL fEnable_p);
tHostifReturn hostif_irqMasterEnable (tHostifInstance pInstance_p,
        BOOL fEnable_p);

tHostifReturn hostif_setBootBase (tHostifInstance pInstance_p, UINT32 base_p);
tHostifReturn hostif_getBootBase (tHostifInstance pInstance_p, UINT32 *pBase_p);
tHostifReturn hostif_setInitBase (tHostifInstance pInstance_p, UINT32 base_p);
tHostifReturn hostif_getInitBase (tHostifInstance pInstance_p, UINT32 *pBase_p);
tHostifReturn hostif_setCommand (tHostifInstance pInstance_p, tHostifCommand cmd_p);
tHostifReturn hostif_getCommand (tHostifInstance pInstance_p, tHostifCommand *pCmd_p);
tHostifReturn hostif_setState (tHostifInstance pInstance_p, tHostifState sta_p);
tHostifReturn hostif_getState (tHostifInstance pInstance_p, tHostifState *pSta_p);
tHostifReturn hostif_setError (tHostifInstance pInstance_p, tHostifError err_p);
tHostifReturn hostif_getError (tHostifInstance pInstance_p, tHostifError *pErr_p);
tHostifReturn hostif_setHeartbeat (tHostifInstance pInstance_p, UINT16 heartbeat_p);
tHostifReturn hostif_getHeartbeat (tHostifInstance pInstance_p, UINT16 *pHeartbeat_p);

tHostifReturn hostif_dynBufAcquire (tHostifInstance pInstance_p, UINT32 pcpBaseAddr_p,
        UINT8 **ppDynBufBase_p);
tHostifReturn hostif_dynBufFree (tHostifInstance pInstance_p, UINT32 pcpBaseAddr_p);

tHostifReturn hostif_queueCreate (tHostifInstance pInstance_p,
        tHostifInstanceId InstanceId_p, tHostifQueueInstance *ppQueueInstance_p);
tHostifReturn hostif_queueDelete (tHostifQueueInstance pQueueInstance_p);
tHostifReturn hostif_queueCallback (tHostifQueueInstance pQueueInstance_p,
        tQueueCb pfnQueueCb_p, void *pArg_p);
tHostifReturn hostif_queueReset (tHostifQueueInstance pQueueInstance_p);
tHostifReturn hostif_queueGetEntryCount (tHostifQueueInstance pQueueInstance_p,
        UINT16 *pEntryCount_p);
tHostifReturn hostif_queueInsert (tHostifQueueInstance pQueueInstance_p,
        UINT8 *pData_p, UINT16 size_p);
tHostifReturn hostif_queueExtract (tHostifQueueInstance pQueueInstance_p,
        UINT8 *pData_p, UINT16 *pSize_p);

tHostifReturn hostif_limCreate (tHostifInstance pInstance_p,
        tHostifInstanceId InstanceId_p, tHostifLimInstance *ppLimInstance_p);
tHostifReturn hostif_limDelete (tHostifLimInstance pLimInstance_p);
tHostifReturn hostif_limWrite (tHostifLimInstance pLimInstance_p,
        UINT16 offset_p, UINT8 *pSrc_p, UINT16 size_p);
tHostifReturn hostif_limRead (tHostifLimInstance pLimInstance_p,
        UINT8 *pDst_p, UINT16 offset_p, UINT16 size_p);
tHostifReturn hostif_limGetBuffer (tHostifLimInstance pLimInstance_p,
        UINT8 **ppBase_p, UINT16 *pSpan_p);

#ifdef __cplusplus
}
#endif

#endif /* _INC_HOSTIFLIB_H_ */
