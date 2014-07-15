/**
********************************************************************************
\file   hostiflib.h

\brief  Host Interface Library - High Level Driver Header

The Host Interface Library High Level Driver provides a software library for the
host interface IP-Core.
The hostiflib provides several features like queues and linear memory modules.

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

#ifndef _INC_hostiflib_H_
#define _INC_hostiflib_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include "hostiflib_target.h"
#include "hostiflib_l.h"

#ifndef CONFIG_HOSTIF_PCP
#error "Define CONFIG_HOSTIF_PCP to TRUE if this is PCP, otherwise FALSE!"
#endif

#if CONFIG_HOSTIF_PCP != FALSE
#include <hostiflib-mem.h> // Only Pcp has access to the ipcore settings
#else
/* This is the host interface version for host. */
#define HOSTIF_VERSION_MAJOR            0xFF
#define HOSTIF_VERSION_MINOR            0xFF
#define HOSTIF_VERSION_REVISION         0xFF
#endif

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#define HOSTIF_USER_INIT_PAR_SIZE       1024    ///< User specific initialization size

//------------------------------------------------------------------------------
// typedef
//------------------------------------------------------------------------------
/**
\brief Return codes
*/
typedef enum eHostifReturn
{
    kHostifSuccessful       = 0x0000,           ///< no error / successful run
    kHostifNoResource       = 0x0001,           ///< resource could not be created
    kHostifInvalidParameter = 0x0002,           ///< function parameter invalid
    kHostifWrongMagic       = 0x0010,           ///< magic word invalid
    kHostifWrongVersion     = 0x0011,           ///< hw/sw version mismatch
    kHostifHwWriteError     = 0x0012,           ///< write to hw failed
    kHostifBridgeDisabled   = 0x0013,           ///< bridge is disabled

    kHostifUnspecError      = 0xFFFF            ///< unspecified error
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
typedef void (*tHostifIrqCb)(void* pArg_p);

/**
\brief Instance Id

The instance ids for the resources.
*/
typedef enum eHostifInstanceId
{
#if CONFIG_HOSTIF_PCP != FALSE
    kHostifInstIdErrCount = 0,                      ///< error counters
#else
    /* Dynamic buffer instances have id 0 to HOSTIF_DYNBUF_COUNT */
    kHostifInstIdErrCount = HOSTIF_DYNBUF_COUNT,    ///< error counters
#endif
    kHostifInstIdTxNmtQueue,                        ///< NMT TX queue
    kHostifInstIdTxGenQueue,                        ///< generic TX queue
    kHostifInstIdTxSyncQueue,                       ///< sync TX queue
    kHostifInstIdTxVethQueue,                       ///< VEth TX queue
    kHostifInstIdRxVethQueue,                       ///< VEth RX queue
    kHostifInstIdK2UQueue,                          ///< K2U Queue
    kHostifInstIdU2KQueue,                          ///< U2K Queue
    kHostifInstIdPdo,                               ///< Pdo
    kHostifInstIdLast,                              ///< last instance id
    kHostifInstIdInvalid = -1                       ///< Invalid instance id
} tHostifInstanceId;

/**
\brief Host interface version

Used to obtain hardware/software mismatch
*/
typedef struct sHostifVersion
{
    UINT8       revision;                           ///< Revision field
    UINT8       minor;                              ///< Minor field
    UINT8       major;                              ///< Major field
} tHostifVersion;

/**
\brief Driver instance configuration

Configures the driver instance.
*/
typedef struct sHostifConfig
{
    UINT8*              pBase;                      ///< Base address to host interface hardware
    UINT                instanceNum;                ///< Number that identifies the hostif instance
    tHostifVersion      version;                    ///< Host interface version
} tHostifConfig;

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

tHostifReturn   hostif_create(tHostifConfig* pConfig_p,
                              tHostifInstance* ppInstance_p);
tHostifReturn   hostif_delete(tHostifInstance pInstance_p);
tHostifInstance hostif_getInstance(UINT Instance_p);

tHostifReturn   hostif_irqRegHdl(tHostifInstance pInstance_p,
                                 tHostifIrqSrc irqSrc_p, tHostifIrqCb pfnCb_p);
tHostifReturn   hostif_irqSourceEnable(tHostifInstance pInstance_p,
                                       tHostifIrqSrc irqSrc_p, BOOL fEnable_p);
tHostifReturn   hostif_irqMasterEnable(tHostifInstance pInstance_p,
                                       BOOL fEnable_p);

tHostifReturn   hostif_setCommand(tHostifInstance pInstance_p, tHostifCommand cmd_p);
tHostifReturn   hostif_getCommand(tHostifInstance pInstance_p, tHostifCommand* pCmd_p);
tHostifReturn   hostif_setState(tHostifInstance pInstance_p, tHostifState sta_p);
tHostifReturn   hostif_getState(tHostifInstance pInstance_p, tHostifState* pSta_p);
tHostifReturn   hostif_setError(tHostifInstance pInstance_p, tHostifError err_p);
tHostifReturn   hostif_getError(tHostifInstance pInstance_p, tHostifError* pErr_p);
tHostifReturn   hostif_setHeartbeat(tHostifInstance pInstance_p, UINT16 heartbeat_p);
tHostifReturn   hostif_getHeartbeat(tHostifInstance pInstance_p, UINT16* pHeartbeat_p);

tHostifReturn   hostif_dynBufAcquire(tHostifInstance pInstance_p, UINT32 pcpBaseAddr_p,
                                     UINT8** ppBufBase_p);
tHostifReturn   hostif_dynBufFree(tHostifInstance pInstance_p, UINT8* pBufBase_p);

tHostifReturn   hostif_getBuf(tHostifInstance pInstance_p, tHostifInstanceId instId_p,
                              UINT8** ppBufBase_p, UINT* pBufSize_p);

tHostifReturn   hostif_getInitParam(tHostifInstance pInstance_p, UINT8** ppBase_p);

#ifdef __cplusplus
}
#endif

#endif /* _INC_hostiflib_H_ */
