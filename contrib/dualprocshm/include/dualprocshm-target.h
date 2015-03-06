/**
********************************************************************************
\file   dualprocshm-target.h

\brief  Dual Processor Library - Target header file

This header file defines platform specific macros (e.g. data types) and selects
the platform specific header files (e.g. dualprocshm-zynq.h) which contains the
information of processors part of the platform.

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

#ifndef _INC_dualprocshm_target_H_
#define _INC_dualprocshm_target_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <stdint.h>

#if defined(__ZYNQ__)

#include "dualprocshm-zynq.h"

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
#ifndef INT
#define INT         int32_t
#endif

#ifndef UINT8
#define UINT8       uint8_t
#endif

#ifndef UINT16
#define UINT16      uint16_t
#endif

#ifndef UINT32
#define UINT32      uint32_t
#endif

#ifndef BOOL
#define BOOL        uint8_t
#endif
/**@}*/

#ifndef FALSE
#define FALSE       0x00
#endif

#ifndef TRUE
#define TRUE        0xFF
#endif

#ifndef UNUSED_PARAMETER
#define UNUSED_PARAMETER(par)    (void)par
#endif

#ifndef TRACE
#define TRACE(...)
#endif

/**
\name Memory operations

If the following memory operations are not defined by the target, then they are
set to the following by default.
*/
/**@{*/
#ifndef DUALPROCSHM_MALLOC
#define DUALPROCSHM_MALLOC(size_p)      malloc(size_p)
#endif

#ifndef DUALPROCSHM_FREE
#define DUALPROCSHM_FREE(ptr_p)         free(ptr_p)
#endif

#ifndef DUALPROCSHM_MEMSET
#define DUALPROCSHM_MEMSET(...)         memset(__VA_ARGS__)
#endif
/**@}*/

//------------------------------------------------------------------------------
// typedef
//------------------------------------------------------------------------------
/**
\brief Function type definition for target synchronization handle

This function callback is called by the synchronization event.
*/
typedef void (*targetSyncHdl)(void*);

//------------------------------------------------------------------------------
// function prototypes
//------------------------------------------------------------------------------

#ifdef __cplusplus
extern "C" {
#endif
UINT8*  dualprocshm_getCommonMemAddr(UINT16* pSize_p);
UINT8*  dualprocshm_getSharedMemBaseAddr(void);
UINT8*  dualprocshm_getDynMapTableAddr(void);
UINT8*  dualprocshm_getIntrMemAddr(void);
void    dualprocshm_releaseIntrMemAddr();
void    dualprocshm_targetReadData(UINT8* pBase_p, UINT16 size_p, UINT8* pData_p);
void    dualprocshm_targetWriteData(UINT8* pBase_p, UINT16 size_p, UINT8* pData_p);
void    dualprocshm_releaseCommonMemAddr(UINT16 pSize_p);
void    dualprocshm_releaseDynMapTableAddr(void);
void    dualprocshm_targetAcquireLock(UINT8* pBase_p, UINT8 lockToken_p) SECTION_DUALPROCSHM_ACQUIRE_LOCK;
void    dualprocshm_targetReleaseLock(UINT8* pBase_p) SECTION_DUALPROCSHM_RELEASE_LOCK;
void    dualprocshm_regSyncIrqHdl(targetSyncHdl callback_p, void* pArg_p);
void    dualprocshm_enableSyncIrq(BOOL fEnable_p);
#ifdef __cplusplus
}
#endif

#endif /* _INC_dualprocshm_target_H_ */
