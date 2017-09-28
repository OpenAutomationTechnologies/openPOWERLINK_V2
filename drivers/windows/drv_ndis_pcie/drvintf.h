/**
********************************************************************************
\file   drv_ndis_pcie/drvintf.h

\brief  Driver interface header file

Driver interface for the kernel daemon - Header file

*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2017, Kalycito Infotech Private Limited
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
#ifndef _INC_drvintf_H_
#define _INC_drvintf_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <common/driver.h>
#include <common/ctrl.h>
#include <common/ctrlcal-mem.h>

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// typedef
//------------------------------------------------------------------------------

/*
/brief File context for user application

This structure contains lock to rundown threads that are dispatching I/Os on
driver file handle while the cleanup is in progress.
*/
typedef struct
{
    IO_REMOVE_LOCK      driverAccessLock;       ///< Driver lock for IO access.
} tFileContext;

//------------------------------------------------------------------------------
// function prototypes
//------------------------------------------------------------------------------
#ifdef __cplusplus
extern "C"
{
#endif

tOplkError drv_init(void);
void       drv_exit(void);
tOplkError drv_executeCmd(tCtrlCmd* ctrlCmd_p);
tOplkError drv_readInitParam(tCtrlInitParam* pInitParam_p);
tOplkError drv_storeInitParam(const tCtrlInitParam* pInitParam_p);
tOplkError drv_getStatus(UINT16* pStatus_p);
tOplkError drv_getHeartbeat(UINT16* pHeartbeat_p);
tOplkError drv_sendAsyncFrame(const tIoctlDllCalAsync* pAsyncFrameInfo_p);
tOplkError drv_writeErrorObject(const tErrHndIoctl* pWriteObject_p);
tOplkError drv_readErrorObject(tErrHndIoctl* pReadObject_p);
tOplkError drv_postEvent(const void* pEvent_p);
tOplkError drv_getEvent(void* pEvent_p,
                        size_t* pSize_p);
tOplkError drv_getPdoMem(UINT32* pPdoMemOffs_p,
                         size_t memSize_p);
tOplkError drv_getBenchmarkMem(void** ppBenchmarkMem_p);
void       drv_freeBenchmarkMem(void* pBenchmarkMem_p);
tOplkError drv_mapKernelMem(void** ppKernelMem_p,
                            void** ppUserMem_p,
                            UINT32* pSize_p);
void       drv_unmapKernelMem(void* pUserMem_p);
tOplkError drv_writeFileBuffer(const tIoctlFileChunk* pIoctlFileChunk_p);
size_t     drv_getFileBufferSize(void);
#if defined(CONFIG_INCLUDE_SOC_TIME_FORWARD)
tOplkError drv_getTimesyncMem(ptrdiff_t* pSocMemOffs_p,
                              size_t socMemSize_p);
#endif

#ifdef __cplusplus
}
#endif

#endif /* _INC_drvintf_H_ */
