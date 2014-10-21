/**
********************************************************************************
\file   dualprocshm.h

\brief  Dual Processor Library - Header

Dual processor library provides routines for initialization of memory
and interrupt resources for a shared memory interface between dual
processor.

*******************************************************************************/
/*------------------------------------------------------------------------------
Copyright (c) 2014 Kalycito Infotech Private Limited
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
#ifndef _INC_dualprocshm_H_
#define _INC_dualprocshm_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include "dualprocshm-target.h"

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

#define  DUALPROC_DYNBUF_COUNT    2         ///< Number of supported dynamic buffers
//------------------------------------------------------------------------------
// typedef
//------------------------------------------------------------------------------
/**
\brief Dual processor library return codes
*/
typedef enum eDualprocReturn
{
    kDualprocSuccessful     = 0x0000,       ///< No error / successful run
    kDualprocNoResource     = 0x0001,       ///< Resource could not be created
    kDualprocInvalidParameter = 0x0002,     ///< Function parameter invalid
    kDualprocWrongProcInst  = 0x0003,       ///< Processor instance wrong
    kDualprocHwWriteError   = 0x0004,       ///< Write to hw failed
    kDualprocBufferOverflow = 0x0005,       ///< Buffer size overflow
    kDualprocBufferEmpty    = 0x0006,       ///< Buffer is empty
    kDualprocBufferError    = 0x0007,       ///< Buffer is faulty

    kDualprocUnspecError    = 0xFFFF        ///< Unspecified error
} tDualprocReturn;

#if (OPLK_OPTIMIZE != FALSE)
/**
\brief Dual processor buffer Id

This is required to handle specially handle DLL circular buffers queue
to optimize stack on non-OS platforms.

*/
typedef enum eDualprocBuffId
{
    kDualprocUsertoKernelQ      = 0x0000,   ///< User-to-kernel event queue
    kDualprocKerneltoUserQ      = 0x0001,   ///< Kernel-to-user event queue
    kDualprocKernelIntQ         = 0x0002,   ///< Kernel internal event queue
    kDualprocUserIntQ           = 0x0003,   ///< User internal event queue
    kDualprocDllCalTxGenQ       = 0x0004,   ///< Queue for sending generic requests in the DLLCAL
    kDualprocDllCalTxNmtQ       = 0x0005,   ///< Queue for sending NMT requests in the DLLCAL
    kDualprocDllCalTxSyncQ      = 0x0006,   ///< Queue for sending sync requests in the DLLCAL
    kDualprocDllCalCnReqNmtQ    = 0x0007,   ///< NMT request queue for MN asynchronous scheduler
    kDualprocDllCalCnReqGenQ    = 0x0008,   ///< Generic request queue for MN asynchronous scheduler
    kDualprocDllCalCnReqIdentQ  = 0x0009,   ///< Ident request queue for MN asynchronous scheduler
    kDualprocDllCalCnReqStatusQ = 0x000A,   ///< Status request queue for MN asynchronous scheduler
    kDualprocErrorHandlerBuff   = 0x000B,   ///< Error handler shared buffer
    kDualprocPdoBuff            = 0x000C,   ///< PDO exchange buffer

    kDualprocUnspecQ            = 0xFFFF    ///< Unspecified queue
} tDualprocBuffId;

#endif

/**
\brief Processor instance

The processor instance determines if the caller is the Pcp or the Host.
*/
typedef enum eDualProcInstance
{
    kDualProcFirst        = 0,              ///< Instance on first processor
    kDualProcSecond       = 1,              ///< Instance on second processor
} tDualProcInstance;

/**
\brief Driver instance configuration

Structure to hold the configuration driver instance .
*/
typedef struct sDualprocConfig
{
    tDualProcInstance       procInstance;   ///< Processor instance
    UINT16                  commMemSize;    ///< Minimum size of common memory
    UINT8                   procId;         ///< Processor Id
} tDualprocConfig;

/**
\brief Memory Instance

Holds information of a dynamic memory
*/
typedef struct sDualprocMemInst
{
    UINT16      span;                       ///< Span of the dynamic buffer
    UINT8       lock;                       ///< Lock for memory
    UINT8       resv;                       ///< Reserved byte;
}tDualprocMemInst;

/**
\brief Dual procesor driver instance
*/
typedef void* tDualprocDrvInstance;

/**
\brief Function type definition for target interrupt callback

This function callback is called for a given interrupt source, registered by
a specific user layer module.
*/
typedef void (*tTargetIrqCb)(void);

/**
\brief Function type to set the address of a Buffer

This function type enables to set the corresponding dynamic shared memory address
register for a dynamic buffer.
*/
typedef void (*tSetDynRes)(tDualprocDrvInstance pDrvInst_p, UINT16 index_p, UINT32 addr_p);

/**
\brief Function type to get the address of a Buffer

This function type enables to get the address set in the dynamic shared buffer
address register.
*/
typedef UINT32 (*tGetDynRes)(tDualprocDrvInstance pDrvInst_p, UINT16 index_p);

/**
\brief Structure for dual processor dynamic resources(buffers)

This structure defines for each dynamic resources instance the set and get
functions. Additionally the base and span is provided.
*/
typedef struct sDualprocDynRes
{
    tSetDynRes          pfnSetDynAddr;              ///< This function sets the dynamic buffer base to hardware
    tGetDynRes          pfnGetDynAddr;              ///< This function gets the dynamic buffer base from hardware
    UINT8*              pBase;                      ///< Base of the dynamic buffer
    tDualprocMemInst*   memInst;                    ///< Pointer to memory instance
} tDualprocDynResConfig;

/**
\brief Dual Processor Instance

Holds the configuration passed to the instance at creation.
*/
typedef struct sDualProcDrv
{
    tDualprocConfig             config;             ///< Copy of configuration
    UINT8*                      pCommMemBase;       ///< Base address of the common memory
    UINT8*                      pAddrTableBase; ///< Base address of the location to place dynamic
                                                    ///< memory address table
    int                         iMaxDynBuffEntries; ///< Number of dynamic buffers (Pcp/Host)
    tDualprocDynResConfig*      pDynResTbl;         ///< Dynamic buffer table (Pcp/Host)
} tDualProcDrv;

//------------------------------------------------------------------------------
// function prototypes
//------------------------------------------------------------------------------

#ifdef __cplusplus
extern "C" {
#endif

tDualprocReturn         dualprocshm_create(tDualprocConfig*pConfig_p, tDualprocDrvInstance*ppInstance_p);
tDualprocReturn         dualprocshm_delete(tDualprocDrvInstance pInstance_p);
tDualprocDrvInstance    dualprocshm_getDrvInst(tDualProcInstance instance_p);
tDualprocReturn         dualprocshm_getMemory(tDualprocDrvInstance pInstance_p, UINT8 id_p,
                                              UINT8**ppAddr_p, size_t*pSize_p, BOOL fAlloc_p);
tDualprocReturn         dualprocshm_freeMemory(tDualprocDrvInstance pInstance_p, UINT8 id_p,
                                               BOOL fFree_p);
tDualprocReturn         dualprocshm_writeData(tDualprocDrvInstance pInstance_p, UINT8 id_p,
                                              UINT32 offset_p, size_t Size_p, UINT8* pData_p);
tDualprocReturn         dualprocshm_readData(tDualprocDrvInstance pInstance_p, UINT8 id_p,
                                             UINT32 offset_p, size_t Size_p, UINT8* pData_p);
tDualprocReturn         dualprocshm_readDataCommon(tDualprocDrvInstance pInstance_p, UINT32 offset_p,
                                                   size_t Size_p, UINT8* pData_p);
tDualprocReturn         dualprocshm_writeDataCommon(tDualprocDrvInstance pInstance_p, UINT32 offset_p,
                                                    size_t Size_p, UINT8* pData_p);

tDualprocReturn         dualprocshm_acquireBuffLock(tDualprocDrvInstance pInstance_p, UINT8 id_p) SECTION_DUALPROCSHM_RE_BUFF_LOCK;
tDualprocReturn         dualprocshm_releaseBuffLock(tDualprocDrvInstance pInstance_p, UINT8 id_p) SECTION_DUALPROCSHM_RE_BUFF_LOCK;

tDualprocReturn         dualprocshm_initInterrupts(tDualprocDrvInstance pInstance_p);
tDualprocReturn         dualprocshm_freeInterrupts(tDualprocDrvInstance pInstance_p);
tDualprocReturn         dualprocshm_registerHandler(tDualprocDrvInstance pInstance_p,
                                                    UINT8 irqId_p, tTargetIrqCb pfnIrqHandler_p);
tDualprocReturn         dualprocshm_enableIrq(tDualprocDrvInstance pInstance_p,
                                              UINT8 irqId_p, BOOL fEnable_p) SECTION_DUALPROCSHM_IRQ_ENABLE;
tDualprocReturn         dualprocshm_setIrq(tDualprocDrvInstance pInstance_p, UINT8 irqId_p, BOOL fSet_p) SECTION_DUALPROCSHM_IRQ_SET;

#ifdef __cplusplus
}
#endif
#endif  // _INC_dualprocshm_H_
