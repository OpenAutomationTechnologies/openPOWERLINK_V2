/**
********************************************************************************
\file   dualprocshm-target.h

\brief  Dual processor library - Target header file

This header file defines platform specific macros (e.g. data types) and selects
the platform specific header files (e.g. dualprocshm-zynq.h) which contains the
information of processors part of the platform.

*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2015, Kalycito Infotech Private Limited
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

#ifndef _INC_dualprocshm_target_H_
#define _INC_dualprocshm_target_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------

#if defined(__ZYNQ__)

#include "dualprocshm-zynq.h"

#elif defined(__C5SOC__)

#include "dualprocshm-c5soc.h"

#elif defined(_KERNEL_MODE)

#include "dualprocshm-pcie.h"

#elif defined (__DE2i_150__)

#include "dualprocshm-pcie.h"

#elif defined (__BR_OPLK_PCIe_IF__)

#include "dualprocshm-pcie.h"

#elif defined (__PCIE__)

#include "dualprocshm-pcie.h"

#else

#error "Platform is not supported! Please point the target platform file in dualprocshm-target.h "

#endif

#ifndef SECTION_DUALPROCSHM_ACQUIRE_LOCK
#define SECTION_DUALPROCSHM_ACQUIRE_LOCK
#endif
#ifndef SECTION_DUALPROCSHM_RELEASE_LOCK
#define SECTION_DUALPROCSHM_RELEASE_LOCK
#endif
#ifndef SECTION_DUALPROCSHM_AQ_BUFF_LOCK
#define SECTION_DUALPROCSHM_AQ_BUFF_LOCK
#endif
#ifndef SECTION_DUALPROCSHM_RE_BUFF_LOCK
#define SECTION_DUALPROCSHM_RE_BUFF_LOCK
#endif
#ifndef SECTION_DUALPROCSHM_IRQ_ENABLE
#define SECTION_DUALPROCSHM_IRQ_ENABLE
#endif
#ifndef SECTION_DUALPROCSHM_IRQ_SET
#define SECTION_DUALPROCSHM_IRQ_SET
#endif
#ifndef SECTION_DUALPROCSHM_IRQ_HDL
#define SECTION_DUALPROCSHM_IRQ_HDL
#endif

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
/**
\name Data types

If the following data types are not defined in the environment, then they are
set to those provided by stdint.h.
*/
/**@{*/
#ifndef _NTDEF_        // defined in ntdef.H, included by dualprocshm-winkernel.h
#ifndef INT
#define INT         int
#endif

#ifndef UINT
#define UINT        unsigned int
#endif

#ifndef ULONG
#define ULONG       unsigned long
#endif

#ifndef UINT8
#define UINT8     uint8_t
#endif

#ifndef UINT16
#define UINT16    uint16_t
#endif

#ifndef UINT32
#define UINT32    uint32_t
#endif

#ifndef UINT64
#define UINT64      uint64_t
#endif
/**@}*/

#ifndef FALSE
#define FALSE     0x00
#endif

#ifndef TRUE
#define TRUE      0xFF
#endif

#endif // _NTDEF_

#ifndef BOOL
#if defined(_WIN32) || defined(_WIN64)
#define BOOL      unsigned char
#else
#define BOOL      uint8_t
#endif // _WIN32
#endif // BOOL

#ifndef UNUSED_PARAMETER
#define UNUSED_PARAMETER(par)    (void)par
#endif

#ifndef TRACE
#define TRACE(...)
#endif

#ifndef PTR_T
#define PTR_T          unsigned long
#endif

/**
\name Memory operations

If the following memory operations are not defined by the target, then they are
set to the following by default.
*/
/**@{*/
#ifndef DUALPROCSHM_MALLOC
#define DUALPROCSHM_MALLOC(size_p)          malloc(size_p)
#endif

#ifndef DUALPROCSHM_FREE
#define DUALPROCSHM_FREE(ptr_p)             free(ptr_p)
#endif

#ifndef DUALPROCSHM_MEMSET
#define DUALPROCSHM_MEMSET(dst, val, siz)   memset((dst), (val), (siz))
#endif

#ifndef DUALPROCSHM_MEMCPY
#define DUALPROCSHM_MEMCPY(dst, src, siz)   memcpy((dst), (src), (siz))
#endif

#ifndef DPSHM_MAKE_NONCACHEABLE
#define DPSHM_MAKE_NONCACHEABLE(pHdl_p)     pHdl_p
#endif

/**@}*/

#ifndef MAX_COMMON_MEM_SIZE
#define MAX_COMMON_MEM_SIZE        2048                         ///< Max common memory size
#endif

#ifndef MAX_DYNAMIC_BUFF_COUNT
#define MAX_DYNAMIC_BUFF_COUNT     20                           ///< Number of maximum dynamic buffers
#endif

#ifndef MAX_DYNAMIC_BUFF_COUNT
#define MAX_DYNAMIC_BUFF_SIZE      MAX_DYNAMIC_BUFF_COUNT * 4   ///< Max dynamic buffer size
#endif

#ifndef SHARED_MEM_BASE
#error "Shared memory base not defined!!!"
#endif

#ifndef COMMON_MEM_BASE
#error "Common memory base not defined!!!"
#endif

#ifndef MEM_ADDR_TABLE_BASE
#error "Dynamic memory address table base not defined!!!"
#endif

#ifndef MEM_INTR_BASE
#error "Interrupt memory address not defined!!!"
#endif

//------------------------------------------------------------------------------
// typedef
//------------------------------------------------------------------------------
/**
\brief Function type definition for target synchronization handle

This function callback is called by the synchronization event.
*/
typedef void (*targetSyncHdl)(void*);

/**
\brief Processor instance

The processor instance determines if the caller is the Pcp or the Host.
*/
typedef enum
{
    kDualProcFirst = 0,              ///< Instance on first processor
    kDualProcSecond = 1,             ///< Instance on second processor
    kDualProcLast = 2,               ///< End of list flag
} eDualProcInstance;

/**
\brief Processor instance data type

Data type for the enumerator \ref eDualProcInstance.
*/
typedef UINT8 tDualProcInstance;

//------------------------------------------------------------------------------
// function prototypes
//------------------------------------------------------------------------------

#ifdef __cplusplus
extern "C"
{
#endif
UINT8*  dualprocshm_getCommonMemAddr(UINT16* pSize_p);
UINT8*  dualprocshm_getSharedMemInst(UINT32* pSize_p);
UINT8*  dualprocshm_getDynMapTableAddr(void);
UINT8*  dualprocshm_getIntrMemAddr(void);
void    dualprocshm_releaseIntrMemAddr(void);
void    dualprocshm_targetReadData(UINT8* pBase_p, UINT16 size_p, UINT8* pData_p);
void    dualprocshm_targetWriteData(UINT8* pBase_p, UINT16 size_p, UINT8* pData_p);
void    dualprocshm_releaseCommonMemAddr(UINT16 pSize_p);
void    dualprocshm_releaseDynMapTableAddr(void);
void    dualprocshm_targetAcquireLock(tDualprocLock* pBase_p, tDualProcInstance procInstance_p) SECTION_DUALPROCSHM_ACQUIRE_LOCK;
void    dualprocshm_targetReleaseLock(tDualprocLock* pBase_p, tDualProcInstance procInstance_p) SECTION_DUALPROCSHM_RELEASE_LOCK;
void    dualprocshm_regSyncIrqHdl(targetSyncHdl callback_p, void* pArg_p);
void    dualprocshm_enableSyncIrq(BOOL fEnable_p);
#ifdef __cplusplus
}
#endif

#endif /* _INC_dualprocshm_target_H_ */
