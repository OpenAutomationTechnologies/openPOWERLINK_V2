/**
********************************************************************************
\file   drv_kernelmod_pcie/drvintf.h

\brief  openPOWERLINK PCIe driver Driver interface header file

openPOWERLINK PCIe driver interface to PCP - Header file

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
#include <oplk/oplk.h>
#include <common/ctrl.h>
#include <common/ctrlcal-mem.h>
#include <common/dllcal.h>

#if defined(CONFIG_INCLUDE_SOC_TIME_FORWARD)
#include <common/timesync.h>
#endif

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// typedef
//------------------------------------------------------------------------------
#if defined(CONFIG_INCLUDE_VETH)
/**
\brief Type for VEth frame receive callback function pointer

This type defines a function pointer to the VEth frame received
callback function.

\param[in]      pFrameInfo_p        Frame info of the received frame.

\return The function returns a tOplkError error code.
*/
typedef tOplkError (*tDrvIntfCbVeth)(const tFrameInfo* pFrameInfo_p);
#endif

//------------------------------------------------------------------------------
// function prototypes
//------------------------------------------------------------------------------
#ifdef __cplusplus
extern "C"
{
#endif
tOplkError             drvintf_init(void);
void                   drvintf_exit(void);
tOplkError             drvintf_executeCmd(tCtrlCmd* ctrlCmd_p);
tOplkError             drvintf_waitSyncEvent(void);
tOplkError             drvintf_readInitParam(tCtrlInitParam* pInitParam_p);
tOplkError             drvintf_storeInitParam(const tCtrlInitParam* pInitParam_p);
tOplkError             drvintf_getStatus(UINT16* pStatus_p);
tOplkError             drvintf_getHeartbeat(UINT16* pHeartbeat_p);
#if defined(CONFIG_INCLUDE_VETH)
tOplkError             drvintf_regVethHandler(tDrvIntfCbVeth pfnDrvIntfCbVeth_p);
tOplkError             drvintf_sendVethFrame(const tFrameInfo* pFrameInfo_p);
#endif
tOplkError             drvintf_sendAsyncFrame(tDllCalQueue queue_p,
                                              size_t size_p,
                                              const void* pData_p);
tOplkError             drvintf_writeErrorObject(UINT32 offset_p,
                                                UINT32 errVal_p);
tOplkError             drvintf_readErrorObject(UINT32 offset_p,
                                               UINT32* pErrVal_p);
tOplkError             drvintf_postEvent(const tEvent* pEvent_p);
tOplkError             drvintf_getEvent(tEvent* pK2UEvent_p,
                                        size_t* pSize_p);
tOplkError             drvintf_getPdoMem(void** ppPdoMem_p,
                                         size_t* pMemSize_p);
tOplkError             drvintf_freePdoMem(void** ppPdoMem_p,
                                          size_t memSize_p);
tOplkError             drvintf_getBenchmarkMem(void** ppBenchmarkMem_p);
tOplkError             drvintf_freeBenchmarkMem(void** ppBenchmarkMem_p);
tOplkError             drvintf_mapKernelMem(const void* pKernelMem_p,
                                            void** ppUserMem_p,
                                            size_t size_p);
void                   drvintf_unmapKernelMem(void** ppUserMem_p);
tOplkError             drvintf_writeFileBuffer(const tOplkApiFileChunkDesc* pDesc_p,
                                               const void* pBuf_p);
ULONG                  drvintf_getFileBufferSize(void);
#if defined(CONFIG_INCLUDE_SOC_TIME_FORWARD)
tOplkError             drvintf_initTimesyncShm(void);
tOplkError             drvintf_exitTimesyncShm(void);
tTimesyncSharedMemory* drvintf_getTimesyncShm(void);
#endif

#ifdef __cplusplus
}
#endif

#endif /* _INC_drvintf_H_ */
