/**
********************************************************************************
\file   dualprocshm.h

\brief  Dual Processor Library - Header

The dual processor library provides routines for initialization of memory
and interrupt resources for a shared memory interface of two processors.

*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2015, Kalycito Infotech Private Limited
Copyright (c) 2016, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
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
#define  DUALPROC_DYNBUF_COUNT      2       ///< Number of supported dynamic buffers

//------------------------------------------------------------------------------
// typedef
//------------------------------------------------------------------------------
/**
\brief Dual processor library return codes

This enumeration specifies library return codes.
*/
typedef enum
{
    kDualprocSuccessful         = 0x0000,   ///< No error / successful run
    kDualprocNoResource         = 0x0001,   ///< Resource could not be created
    kDualprocInvalidParameter   = 0x0002,   ///< Function parameter invalid
    kDualprocWrongProcInst      = 0x0003,   ///< Processor instance wrong
    kDualprocHwWriteError       = 0x0004,   ///< Write to hardware failed
    kDualprocBufferOverflow     = 0x0005,   ///< Buffer size overflow
    kDualprocBufferEmpty        = 0x0006,   ///< Buffer is empty
    kDualprocBufferError        = 0x0007,   ///< Buffer is faulty
    kDualprocHwReadError        = 0x0008,   ///< Read from hardware failed
    kDualprocInvalidCommHeader  = 0x0009,   ///< Common memory header data is invalid
    kDualprocInvalidInstance    = 0x000A,   ///< DualprocShm instance is not configured
    kDualprocshmIntfEnabled     = 0x000B,   ///< DualprocShm interface is enabled/ active
    kDualprocshmIntfDisabled    = 0x000C,   ///< DualprocShm interface is disabled/ not active

    kDualprocUnspecError        = 0xFFFF    ///< Unspecified error
} eDualprocReturn;

/**
\brief Return code data type

Data type for the enumerator \ref eDualprocReturn.
*/
typedef UINT16 tDualprocReturn;

/**
\brief Driver instance configuration

Structure to hold the configuration driver instance.
*/
typedef struct sDualprocConfig
{
    tDualProcInstance   procInstance;       ///< Processor instance
    UINT16              commonMemSize;      ///< Minimum size of common memory
    UINT8               resv;               ///< Reserved
} tDualprocConfig;

/**
\brief Memory Instance

Holds information of a dynamic memory.
*/
typedef struct sDualprocMemInst
{
    UINT16              span;               ///< Span of the dynamic buffer
    UINT16              padding1;           ///< Padding variable 1
    tDualprocLock       lock;               ///< Lock for memory
} tDualprocMemInst;

/**
\brief Shared memory instance

Structure to hold shared memory information.

*/
typedef struct sDualprocSharedMemInst
{
    UINT64              baseAddr;         ///< Base address of the shared memory.
    UINT32              span;             ///< Size of the shared memory.
} tDualprocSharedMemInst;

/**
\brief Dual processor driver instance

This type defines the driver instance.
*/
typedef void* tDualprocDrvInstance;

/**
\brief Function type definition for target interrupt callback

This function callback is called for a given interrupt source, registered by
a specific user layer module.
*/
typedef void (*tTargetIrqCb)(void);

/**
\brief Function type to set the address of a buffer

This function type enables to set the corresponding dynamic shared memory address
register for a dynamic buffer.

\param[in]      pDrvInst_p          The dual processor driver instance
\param[in]      index_p             Index to select the dynamic buffer
\param[in]      addr_p              Address to the memory space referenced by the dynamic buffer

\return The function returns 0 if the address has been set successfully, otherwise -1.
*/
typedef INT (*tSetDynRes)(tDualprocDrvInstance pDrvInst_p,
                          UINT16 index_p,
                          UINT64 addr_p);

/**
\brief Function type to get the address of a Buffer

This function type enables to get the address which is set in the dynamic shared
buffer address register.

\param[in]      pDrvInst_p          The dual processor driver instance
\param[in]      index_p             Index to select the dynamic buffer

\return The function returns the address to the memory space referenced by the
        dynamic buffer.
*/
typedef void* (*tGetDynRes)(tDualprocDrvInstance pDrvInst_p,
                            UINT16 index_p);

/**
\brief Structure for dual processor dynamic resources (buffers)

This structure defines for each dynamic resources instance the set and get
functions. Additionally the base and span is provided.
*/
typedef struct sDualprocDynRes
{
    tSetDynRes          pfnSetDynAddr;    ///< This function sets the dynamic buffer base to hardware
    tGetDynRes          pfnGetDynAddr;    ///< This function gets the dynamic buffer base from hardware
    void*               pBase;            ///< Base of the dynamic buffer
    tDualprocMemInst*   pMemInst;         ///< Pointer to memory instance
} tDualprocDynResConfig;

/**
\brief Header structure for dual processor library

Currently holds the address of shared memory on the first processor.

*/
typedef struct sDualprocHeader
{
    UINT16              shmMagic;                       ///< Dualprocshm interface magic field. This is used to indicate valid header data
    UINT16              shmIntfState;                   ///< Dualprocshm interface state indicator field
    UINT32              span;                           ///< Size of the shared memory.
    UINT64              sharedMemBase[kDualProcLast];   ///< Shared memory addresses of both processors
} tDualprocHeader;

/**
\brief Dual Processor common memory instance

Holds the individual segment configuration details, inside common memory
*/
typedef struct sCommonMemInst
{
    tDualprocHeader*    pCommonMemHeader;               ///< Pointer to the common memory header segment
    void*               pCommonMemBase;                 ///< Pointer to the common memory data segment.
} tDualprocCommonMemInst;

/**
\brief Dual Processor instance

Holds the configuration passed to the instance at creation.
*/
typedef struct sDualProcDrv
{
    tDualprocConfig             config;             ///< Copy of configuration
    tDualprocCommonMemInst      commonMemInst;      ///< Common memory instance
    void*                       pAddrTableBase;     ///< Pointer to dynamic memory address table
    INT                         maxDynBuffEntries;  ///< Number of dynamic buffers (First & Second processor)
    tDualprocDynResConfig*      pDynResTbl;         ///< Dynamic buffer table (First & Second processor)
} tDualProcDrv;

//------------------------------------------------------------------------------
// function prototypes
//------------------------------------------------------------------------------
#ifdef __cplusplus
extern "C"
{
#endif

tDualprocReturn         dualprocshm_create(const tDualprocConfig* pConfig_p,
                                           tDualprocDrvInstance* ppInstance_p);
tDualprocReturn         dualprocshm_delete(tDualprocDrvInstance pInstance_p);
tDualprocDrvInstance    dualprocshm_getLocalProcDrvInst(void);
tDualProcInstance       dualprocshm_getLocalProcInst(void);
tDualProcInstance       dualprocshm_getRemoteProcInst(void);
tDualprocReturn         dualprocshm_getMemory(tDualprocDrvInstance pInstance_p,
                                              UINT8 id_p,
                                              void** ppAddr_p,
                                              size_t* pSize_p,
                                              BOOL fAlloc_p);
tDualprocReturn         dualprocshm_freeMemory(tDualprocDrvInstance pInstance_p,
                                               UINT8 id_p,
                                               BOOL fFree_p);
tDualprocReturn         dualprocshm_writeData(tDualprocDrvInstance pInstance_p,
                                              UINT8 id_p,
                                              UINT32 offset_p,
                                              size_t size_p,
                                              const void* pData_p);
tDualprocReturn         dualprocshm_readData(tDualprocDrvInstance pInstance_p,
                                             UINT8 id_p,
                                             UINT32 offset_p,
                                             size_t size_p,
                                             void* pData_p);
tDualprocReturn         dualprocshm_readDataCommon(tDualprocDrvInstance pInstance_p,
                                                   UINT32 offset_p,
                                                   size_t size_p,
                                                   void* pData_p);
tDualprocReturn         dualprocshm_writeDataCommon(tDualprocDrvInstance pInstance_p,
                                                    UINT32 offset_p,
                                                    size_t size_p,
                                                    const void* pData_p);
tDualprocReturn         dualprocshm_enableShmIntf(tDualprocDrvInstance pInstance_p);
tDualprocReturn         dualprocshm_checkShmIntfState(tDualprocDrvInstance pInstance_p);
tDualprocReturn         dualprocshm_getSharedMemInfo(tDualprocDrvInstance pInstance_p,
                                                     tDualProcInstance procInstance_p,
                                                     tDualprocSharedMemInst* pShmMemInst_p);
tDualprocReturn         dualprocshm_acquireBuffLock(tDualprocDrvInstance pInstance_p,
                                                    UINT8 id_p)
                                                    SECTION_DUALPROCSHM_RE_BUFF_LOCK;
tDualprocReturn         dualprocshm_releaseBuffLock(tDualprocDrvInstance pInstance_p,
                                                    UINT8 id_p)
                                                    SECTION_DUALPROCSHM_RE_BUFF_LOCK;
tDualprocReturn         dualprocshm_initInterrupts(tDualprocDrvInstance pInstance_p);
tDualprocReturn         dualprocshm_freeInterrupts(tDualprocDrvInstance pInstance_p);
tDualprocReturn         dualprocshm_registerHandler(tDualprocDrvInstance pInstance_p,
                                                    UINT8 irqId_p,
                                                    tTargetIrqCb pfnIrqHandler_p);
tDualprocReturn         dualprocshm_enableIrq(tDualprocDrvInstance pInstance_p,
                                              UINT8 irqId_p,
                                              BOOL fEnable_p)
                                              SECTION_DUALPROCSHM_IRQ_ENABLE;
tDualprocReturn         dualprocshm_setIrq(tDualprocDrvInstance pInstance_p,
                                           UINT8 irqId_p,
                                           BOOL fSet_p)
                                           SECTION_DUALPROCSHM_IRQ_SET;

#ifdef __cplusplus
}
#endif

#endif  // _INC_dualprocshm_H_
