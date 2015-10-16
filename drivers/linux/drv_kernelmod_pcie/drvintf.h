/**
********************************************************************************
\file   drv_kernelmod_pcie/drvintf.h

\brief  openPOWERLINK PCIe driver Driver interface header file

openPOWERLINK PCIe driver interface to PCP - Header file

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

#ifndef _INC_drvintf_H_
#define _INC_drvintf_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <common/driver.h>
#include <common/ctrl.h>
#include <common/target.h>
#include <kernel/timesynckcal.h>
#include <common/ctrlcal-mem.h>

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#define FIELD_OFFSET(...)                       offsetof(__VA_ARGS__)

//------------------------------------------------------------------------------
// typedef
//------------------------------------------------------------------------------
#if defined(CONFIG_INCLUDE_VETH)
/**
\brief Type for VEth frame receive callback function pointer

This type defines a function pointer to the VEth frame received
callback function.

\param  pFrameInfo_p        Frame info of the received frame.

\return The function returns a tOplkError error code.
*/
typedef tOplkError (*tDrvIntfCbVeth)(tFrameInfo* pFrameInfo_p);
#endif

//------------------------------------------------------------------------------
// function prototypes
//------------------------------------------------------------------------------

#ifdef __cplusplus
extern "C"
{
#endif
tOplkError drvintf_init(void);
void       drvintf_exit(void);
tOplkError drvintf_executeCmd(tCtrlCmd* ctrlCmd_p);
tOplkError drvintf_readInitParam(tCtrlInitParam* pInitParam_p);
tOplkError drvintf_storeInitParam(tCtrlInitParam* pInitParam_p);
tOplkError drvintf_getStatus(UINT16* pStatus_p);
tOplkError drvintf_getHeartbeat(UINT16* pHeartbeat_p);
tOplkError drvintf_postEvent(tEvent* pEvent_p);
tOplkError drvintf_getEvent(tEvent* pK2UEvent_p, size_t* pSize_p);
tOplkError drvintf_sendAsyncFrame(tDllCalQueue queue_p, size_t size_p, void* pData_p);
tOplkError drvintf_writeErrorObject(UINT32 offset_p, UINT32 errVal_p);
tOplkError drvintf_readErrorObject(UINT32 offset_p, UINT32* pErrVal_p);
tOplkError drvintf_getPdoMem(UINT8** ppPdoMem_p, size_t* pMemSize_p);
tOplkError drvintf_freePdoMem(UINT8** ppPdoMem_p, size_t memSize_p);
tOplkError drvintf_getBenchmarkMem(UINT8** ppBenchmarkMem_p);
tOplkError drvintf_freeBenchmarkMem(UINT8** ppBenchmarkMem_p);
tOplkError drvintf_mapKernelMem(UINT8* pKernelMem_p, UINT8** ppUserMem_p, size_t size_p);
void       drvintf_unmapKernelMem(UINT8** ppUserMem_p);
tOplkError drvintf_waitSyncEvent(void);
#if defined(CONFIG_INCLUDE_VETH)
tOplkError drvintf_regVethHandler(tDrvIntfCbVeth pfnDrvIntfCbVeth_p);
tOplkError drvintf_sendVethFrame(tFrameInfo* pFrameInfo_p);
#endif

#ifdef __cplusplus
}
#endif

#endif /* _INC_drvintf_H_ */
