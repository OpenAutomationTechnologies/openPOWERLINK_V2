/**
********************************************************************************
\file   drvintf.c

\brief  Interface module for application interface to kernel daemon in Windows
        kernel

This module handles all the application request forwarded to the daemon
in Windows kernel. It uses dualprocshm and circbuf libraries to manage PDO
memory, error objects shared memory, event and DLL queues.

The module also implements mapping of kernel memory into user space to provide
direct access for specific shared memory regions to the user application.

\ingroup module_driver_ndispcie
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

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include "drvintf.h"

#include <common/target.h>
#include <common/circbuffer.h>
#include <common/ami.h>
#include <common/errhnd.h>

#include <dualprocshm.h>
#include <ndis-intf.h>

//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// module global vars
//------------------------------------------------------------------------------
#define DUALPROCSHM_BUFF_ID_ERRHDLR  12
#define DUALPROCSHM_BUFF_ID_PDO      13
#define BENCHMARK_OFFSET             0x00001000  //TODO: Get this value from PCIe header files
#define DPSHM_ENABLE_TIMEOUT_SEC     10          // wait for dpshm interface enable time out
#if defined(CONFIG_INCLUDE_SOC_TIME_FORWARD)
#define DUALPROCSHM_BUFF_ID_TIMESYNC 14
#endif

//------------------------------------------------------------------------------
// global function prototypes
//------------------------------------------------------------------------------

//============================================================================//
//            P R I V A T E   D E F I N I T I O N S                           //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#define CMD_TIMEOUT_CNT             500         // loop counter for command timeout

//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------
/**
\brief Mapped memory information

This structure stores the information for a memory shared between user and
kernel space.
*/
typedef struct
{
    PMDL                    pMdl;                                   ///< Pointer to memory descriptor list describing the memory.
    size_t                  memSize;                                ///< Size of the memory.
    void*                   pKernelVa;                              ///< Pointer to memory in kernel space.
    void*                   pUserVa;                                ///< Pointer to memory mapped in user space.
} tMemInfo;

/**
\brief Driver instance

The driver instance stores the local parameters used by the
driver interface module during runtime.
*/
typedef struct
{
    tDualprocDrvInstance    pDualProcDrvInst;                       ///< Dual processor driver instance.
    tCircBufInstance*       apEventQueueInst[kEventQueueNum];       ///< Event queue instances.
    tCircBufInstance*       apDllQueueInst[kDllCalQueueTxVeth + 1]; ///< DLL queue instances.
    tErrHndObjects*         pErrorObjects;                          ///< Pointer to error objects.
    tMemInfo                pdoMem;                                 ///< PDO memory information mapped to user space.
    tMemInfo                benchmarkMem;                           ///< Benchmark memory information mapped to user space.
    tMemInfo                kernel2UserMem;                         ///< Kernel to user mapped memory.
    UINT64                  remoteProcBase;                         ///< Base address for the shared memory of the second processor.
    BOOL                    fDriverActive;                          ///< Flag to identify status of driver interface.
} tDriverInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tDriverInstance      drvInstance_l;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static tOplkError initDualProcShm(void);
static tOplkError initEvent(void);
static tOplkError initDllQueues(void);
static tOplkError initErrHndl(void);
static void       exitDualProcShm(void);
static void       exitEvent(void);
static void       exitDllQueues(void);
static void       exitErrHndl(void);
static tOplkError insertDataBlock(tCircBufInstance* pDllCircBuffInst_p,
                                  const void* pData_p,
                                  size_t dataSize_p);
static tOplkError mapMemory(tMemInfo* pMemInfo_p);
static void       unmapMemory(tMemInfo* pMemInfo_p);
#if defined(CONFIG_INCLUDE_VETH)
static tOplkError getMemory(const void* pPCIeBuf_p,
                            void** ppKernelMem_p,
                            UINT32 size_p);
static tOplkError nonPlkFrameSendCb(void* pBuffer_p,
                                    size_t size_p);
static tOplkError receiveNonPlkFrame(void* pEvent_p);
#endif

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Initialize driver interface

This function initializes necessary resources required for driver interface.

\return Returns a tOplkError error code.

\ingroup module_driver_ndispcie
*/
//------------------------------------------------------------------------------
tOplkError drv_init(void)
{
    tOplkError  ret;

    DEBUG_LVL_ALWAYS_TRACE("Initialize driver interface...");

    OPLK_MEMSET(&drvInstance_l, 0, sizeof(tDriverInstance));

    // Initialize the dualprocshm library
    ret = initDualProcShm();
    if (ret != kErrorOk)
    {
        DEBUG_LVL_ERROR_TRACE("Dual processor shared memory interface Initialization failed (0x%X)\n",
                              ret);
        return ret;
    }

#if defined(CONFIG_INCLUDE_VETH)
    ndis_registerVethHandler(nonPlkFrameSendCb);
#endif
    drvInstance_l.fDriverActive = TRUE;

    DEBUG_LVL_ALWAYS_TRACE(" OK\n");

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Close the driver interface

This function frees all the resources used by the driver interface and shuts down
the interface.

\ingroup module_driver_ndispcie
*/
//------------------------------------------------------------------------------
void drv_exit(void)
{
    DEBUG_LVL_ALWAYS_TRACE("Exit driver interface...\n");

    if (drvInstance_l.fDriverActive)
    {
        drvInstance_l.fDriverActive = FALSE;
#if defined(CONFIG_INCLUDE_VETH)
        // Clear VEth Handler
        ndis_registerVethHandler(NULL);
#endif
        // Close dualprocshm library interface
        exitDualProcShm();
    }
}

//------------------------------------------------------------------------------
/**
\brief  Execute a control command from user application

This function extracts the control command passed from the openPOWERLINK user
layer and passes it to the kernel control module for processing. The return
value is again passed to user layer by copying it into the common
control structure.

\param[in,out]  pCtrlCmd_p          Pointer to control command structure.

\return Returns a tOplkError error code.

\ingroup module_driver_ndispcie
*/
//------------------------------------------------------------------------------
tOplkError drv_executeCmd(tCtrlCmd* pCtrlCmd_p)
{
    tOplkError  ret = kErrorOk;
    UINT16      status;
    UINT16      cmd;
    INT         timeout;

    if ((pCtrlCmd_p == NULL) ||
        !drvInstance_l.fDriverActive)
        return kErrorNoResource;

    cmd = pCtrlCmd_p->cmd;

    // Clean up stack
    if ((cmd == kCtrlCleanupStack) ||
        (cmd == kCtrlShutdown))
    {
        exitDllQueues();
        exitErrHndl();
        exitEvent();
    }

    if (dualprocshm_writeDataCommon(drvInstance_l.pDualProcDrvInst,
                                    FIELD_OFFSET(tCtrlBuf, ctrlCmd),
                                    sizeof(tCtrlCmd),
                                    pCtrlCmd_p) != kDualprocSuccessful)
        return kErrorNoResource;

    // wait for response
    for (timeout = 0; timeout < CMD_TIMEOUT_CNT; timeout++)
    {
        target_msleep(10);

        if (dualprocshm_readDataCommon(drvInstance_l.pDualProcDrvInst,
                                       FIELD_OFFSET(tCtrlBuf, ctrlCmd),
                                       sizeof(tCtrlCmd),
                                       pCtrlCmd_p) != kDualprocSuccessful)
            return kErrorNoResource;

        if (pCtrlCmd_p->cmd == 0)
            break;
    }

    if (timeout == CMD_TIMEOUT_CNT)
        return kErrorGeneralError;

    if ((cmd == kCtrlInitStack) &&
        (pCtrlCmd_p->retVal == kErrorOk))
    {
        ret = initEvent();
        if (ret != kErrorOk)
        {
            DEBUG_LVL_ERROR_TRACE("Event initialization failed (0x%X)\n", ret);
            return ret;
        }

        ret = initErrHndl();
        if (ret != kErrorOk)
        {
            DEBUG_LVL_ERROR_TRACE("Error module initialization failed (0x%X)\n", ret);
            return ret;
        }

        ret = initDllQueues();
        if (ret != kErrorOk)
        {
            DEBUG_LVL_ERROR_TRACE("Dll queues initialization failed (0x%X)\n", ret);
            return ret;
        }

        // Enable the upper binding of the virtual Ethernet interface
        ndis_setAdapterState(kNdisBindingRunning);
    }

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Read initialization parameters

Read the initialization parameters from the kernel stack.

\param[out]     pInitParam_p        Pointer to initialization parameters structure.

\return Returns a tOplkError error code.

\ingroup module_driver_ndispcie
*/
//------------------------------------------------------------------------------
tOplkError drv_readInitParam(tCtrlInitParam* pInitParam_p)
{
    tDualprocReturn dualRet;

    if (!drvInstance_l.fDriverActive)
        return kErrorNoResource;

    dualRet = dualprocshm_readDataCommon(drvInstance_l.pDualProcDrvInst,
                                         FIELD_OFFSET(tCtrlBuf, initParam),
                                         sizeof(tCtrlInitParam),
                                         pInitParam_p);
    if (dualRet != kDualprocSuccessful)
    {
        DEBUG_LVL_ERROR_TRACE("Cannot read initparam (0x%X)\n", dualRet);
        return kErrorNoResource;
    }

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Write initialization parameters

Write the initialization parameters from the user layer into kernel memory.

\param[in]      pInitParam_p        Pointer to initialization parameters structure.

\return Returns a tOplkError error code.

\ingroup module_driver_ndispcie
*/
//------------------------------------------------------------------------------
tOplkError drv_storeInitParam(const tCtrlInitParam* pInitParam_p)
{
    tDualprocReturn dualRet;

    if (!drvInstance_l.fDriverActive)
        return kErrorNoResource;

    dualRet = dualprocshm_writeDataCommon(drvInstance_l.pDualProcDrvInst,
                                          FIELD_OFFSET(tCtrlBuf, initParam),
                                          sizeof(tCtrlInitParam),
                                          pInitParam_p);
    if (dualRet != kDualprocSuccessful)
    {
        DEBUG_LVL_ERROR_TRACE("Cannot store initparam (0x%X)\n", dualRet);
        return kErrorNoResource;
    }

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Get status from openPOWELINK kernel stack

Returns the current status of kernel stack.

\param[out]     pStatus_p           Pointer to status variable to return.

\return Returns a tOplkError error code.

\ingroup module_driver_ndispcie
*/
//------------------------------------------------------------------------------
tOplkError drv_getStatus(UINT16* pStatus_p)
{
    if (!drvInstance_l.fDriverActive)
        return kErrorNoResource;

    if (dualprocshm_readDataCommon(drvInstance_l.pDualProcDrvInst,
                                   FIELD_OFFSET(tCtrlBuf, status),
                                   sizeof(UINT16),
                                   pStatus_p) != kDualprocSuccessful)
    {
        DEBUG_LVL_ERROR_TRACE("Error Reading Status\n");
        return kErrorNoResource;
    }

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Get heartbeat

Return the current heartbeat value in kernel.

\param[out]     pHeartbeat_p        Pointer to heartbeat variable to return.

\return Returns a tOplkError error code.

\ingroup module_driver_ndispcie
*/
//------------------------------------------------------------------------------
tOplkError drv_getHeartbeat(UINT16* pHeartbeat_p)
{
    if (!drvInstance_l.fDriverActive)
        return kErrorNoResource;

    if (dualprocshm_readDataCommon(drvInstance_l.pDualProcDrvInst,
                                   FIELD_OFFSET(tCtrlBuf, heartbeat),
                                   sizeof(UINT16),
                                   pHeartbeat_p) != kDualprocSuccessful)
    {
        DEBUG_LVL_ERROR_TRACE("Error Reading HeartBeat\n");
        return kErrorNoResource;
    }

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Write asynchronous frame

This routines extracts the asynchronous frame from the ioctl buffer and writes
it into the specified DLL queue for processing by openPOWERLINK kernel layer.

\param[out]     pArg_p              Pointer to ioctl buffer.

\return Returns a tOplkError error code.

\ingroup module_driver_ndispcie
*/
//------------------------------------------------------------------------------
tOplkError drv_sendAsyncFrame(const tIoctlDllCalAsync* pAsyncFrameInfo_p)
{
    tFrameInfo  frameInfo;
    tOplkError  ret =  kErrorOk;

    if ((pAsyncFrameInfo_p == NULL) ||
        !drvInstance_l.fDriverActive)
        return kErrorNoResource;

    frameInfo.frameSize = pAsyncFrameInfo_p->size;
    if (pAsyncFrameInfo_p->pData == NULL)
        frameInfo.frame.pBuffer = (tPlkFrame*)((UINT8*)pAsyncFrameInfo_p + sizeof(tIoctlDllCalAsync));
    else
        frameInfo.frame.pBuffer = (tPlkFrame*)pAsyncFrameInfo_p->pData;

    ret = insertDataBlock(drvInstance_l.apDllQueueInst[pAsyncFrameInfo_p->queue],
                          frameInfo.frame.pBuffer,
                          (size_t)frameInfo.frameSize);

    if (ret != kErrorOk)
    {
        DEBUG_LVL_ERROR_TRACE("Error sending async frame to queue %d (0x%X)\n",
                              pAsyncFrameInfo_p->queue,
                              ret);
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Write error object

This routines updates the error objects in shared memory with the value passed
from user layer.

\param[in]      pWriteObject_p      Pointer to write object to update.

\return Returns a tOplkError error code.

\ingroup module_driver_ndispcie
*/
//------------------------------------------------------------------------------
tOplkError drv_writeErrorObject(const tErrHndIoctl* pWriteObject_p)
{
    tErrHndObjects* errorObjects = drvInstance_l.pErrorObjects;

    if ((pWriteObject_p == NULL) ||
        !drvInstance_l.fDriverActive)
        return kErrorNoResource;

    *((UINT32*)((UINT8*)errorObjects + pWriteObject_p->offset)) = pWriteObject_p->errVal;

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Read error object

This routines fetches the error objects in shared memory to be passed to user
layer.

\param[out]     pReadObject_p       Pointer to read object to fetch.

\return Returns a tOplkError error code.

\ingroup module_driver_ndispcie
*/
//------------------------------------------------------------------------------
tOplkError drv_readErrorObject(tErrHndIoctl* pReadObject_p)
{
    tErrHndObjects* errorObjects = drvInstance_l.pErrorObjects;

    if ((pReadObject_p == NULL) ||
        !drvInstance_l.fDriverActive)
        return kErrorNoResource;

    pReadObject_p->errVal = *((UINT32*)((UINT8*)errorObjects + pReadObject_p->offset));

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Post a user event

Copies the event from user layer into user to kernel event queue.

\param[in]      pEvent_p            Pointer to event.

\return Returns a tOplkError error code.

\ingroup module_driver_ndispcie
*/
//------------------------------------------------------------------------------
tOplkError drv_postEvent(const tEvent* pEvent_p)
{
    tOplkError          ret = kErrorOk;
    tCircBufError       circError;
    tCircBufInstance*   pCircBufInstance = drvInstance_l.apEventQueueInst[kEventQueueU2K];

    if ((pEvent_p == NULL) ||
        !drvInstance_l.fDriverActive)
        return kErrorNoResource;

    if (pEvent_p->eventArgSize == 0)
    {
        circError = circbuf_writeData(pCircBufInstance,
                                      pEvent_p,
                                      sizeof(tEvent));
    }
    else
    {
        circError = circbuf_writeMultipleData(pCircBufInstance,
                                              pEvent_p,
                                              sizeof(tEvent),
                                              pEvent_p->eventArg.pEventArg,
                                              pEvent_p->eventArgSize);
    }

    if (circError != kCircBufOk)
    {
        DEBUG_LVL_ERROR_TRACE("Error in posting event (0x%X)\n", circError);
        ret = kErrorEventPostError;
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Get a user event

Retrieves an event from kernel to user event queue for the user layer.

\param[out]     pEvent_p            Pointer to event memory.
\param[out]     pSize_p             Size of the event buffer.

\return Returns a tOplkError error code and the size of the read data.

\ingroup module_driver_ndispcie
*/
//------------------------------------------------------------------------------
tOplkError drv_getEvent(void* pEvent_p,
                        size_t* pSize_p)
{
    tCircBufInstance*   pCircBufInstance = drvInstance_l.apEventQueueInst[kEventQueueK2U];
    tOplkError          ret = kErrorOk;

    if ((pEvent_p == NULL) ||
        (pSize_p == NULL) ||
        !drvInstance_l.fDriverActive)
        return kErrorNoResource;

    if (circbuf_getDataCount(pCircBufInstance) > 0)
    {
        circbuf_readData(pCircBufInstance,
                         pEvent_p,
                         sizeof(tEvent) + MAX_EVENT_ARG_SIZE,
                         pSize_p);
        //TODO: Handle circbuf return value!
    }
    else
        *pSize_p = 0;

#if defined(CONFIG_INCLUDE_VETH)

    if (*pSize_p != 0)
    {
        ret = receiveNonPlkFrame(pEvent_p);
        if (ret == kErrorOk)
        {
            *pSize_p = 0;
        }
        else
        {
            if (ret == kErrorReject)
            {
                ret = kErrorOk;
            }
            else
            {
                DEBUG_LVL_ERROR_TRACE("Error handling non-PLK frame (0x%X)\n", ret);
            }
        }
    }
#endif
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Get PDO memory offset

Retrieves the PDO memory offset from the dualprocshm library and shares it
with user application.

\param[out]     pPdoMemOffs_p       Pointer to PDO memory offset value.
\param[in]      memSize_p           Size of the PDO memory.

\return Returns a tOplkError error code.

\ingroup module_driver_ndispcie
*/
//------------------------------------------------------------------------------
tOplkError drv_getPdoMem(UINT32* pPdoMemOffs_p,
                         size_t memSize_p)
{
    tDualprocReturn dualRet;
    ptrdiff_t       offset;
    void*           pMem = NULL;
    UINT8*          pBar0 = (UINT8*)ndis_getBarAddr(OPLK_PCIEBAR_SHM);

    if (!drvInstance_l.fDriverActive)
        return kErrorNoResource;

    dualRet = dualprocshm_getMemory(drvInstance_l.pDualProcDrvInst,
                                    DUALPROCSHM_BUFF_ID_PDO,
                                    &pMem,
                                    &memSize_p,
                                    FALSE);
    if (dualRet != kDualprocSuccessful)
    {
        DEBUG_LVL_ERROR_TRACE("%s() couldn't allocate PDO buffer (0x%X)\n",
                              __func__,
                              dualRet);
        return kErrorNoResource;
    }

    offset = (UINT8*)pMem - pBar0;

    *pPdoMemOffs_p = (UINT32)offset;

    DEBUG_LVL_ALWAYS_TRACE("%s() PDO memory offset is %x\n", __func__, offset);
    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Get benchmark base

Retrieves the benchmark memory from NDIS driver and maps it into user virtual
address space for providing access to user layer.

\param[out]     ppBenchmarkMem_p    Pointer to benchmark memory.

\return Returns a tOplkError error code and the benchmark base address.

\ingroup module_driver_ndispcie
*/
//------------------------------------------------------------------------------
tOplkError drv_getBenchmarkMem(void** ppBenchmarkMem_p)
{
    UINT8*      pMem;
    tMemInfo*   pBenchmarkMemInfo = &drvInstance_l.benchmarkMem;

    if (!drvInstance_l.fDriverActive)
        return kErrorNoResource;

    // Check if memory is already allocated and mapped
    if (pBenchmarkMemInfo->pUserVa != NULL)
        goto Exit;

    pMem = (UINT8*)ndis_getBarAddr(OPLK_PCIEBAR_COMM_MEM);

    if (pMem == NULL)
        return kErrorNoResource;

    pBenchmarkMemInfo->pKernelVa = pMem + BENCHMARK_OFFSET;
    pBenchmarkMemInfo->memSize = 4;

    if (mapMemory(pBenchmarkMemInfo) != kErrorOk)
    {
        DEBUG_LVL_ERROR_TRACE("%s() error mapping memory\n", __func__);
        return kErrorNoResource;
    }

Exit:
    *ppBenchmarkMem_p = pBenchmarkMemInfo->pUserVa;

    DEBUG_LVL_ALWAYS_TRACE("%s() Benchmark memory address in user space %p\n",
                           __func__,
                           pBenchmarkMemInfo->pUserVa);
    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Free Benchmark memory

Frees the benchmark memory mapped in user space.

\param[in,out]  pBenchmarkMem_p     Pointer to benchmark memory.

\ingroup module_driver_ndispcie
*/
//------------------------------------------------------------------------------
void drv_freeBenchmarkMem(void* pBenchmarkMem_p)
{
    tMemInfo*   pBenchmarkMemInfo = &drvInstance_l.benchmarkMem;

    UNUSED_PARAMETER(pBenchmarkMem_p);

    unmapMemory(pBenchmarkMemInfo);
}

//------------------------------------------------------------------------------
/**
\brief  Map complete shared memory into user layer

Maps the shared memory between openPOWERLINK kernel and user stack
into the user space of the OS. The routine also shares the base address of
the shared memory in openPOWERLINK kernel layer (i.e. on board memory) for
offset calculation.

This mapped memory can then be used by the user application without accessing
the driver.

\param[out]     ppKernelMem_p       Double pointer to kernel memory.
\param[out]     ppUserMem_p         Double pointer to kernel memory mapped in user layer.
\param[out]     pSize_p             Size of the shared memory.

\return Returns tOplkError error code and the pointer to the memory mapped in
        user space.

\ingroup module_driver_ndispcie
*/
//------------------------------------------------------------------------------
tOplkError drv_mapKernelMem(void** ppKernelMem_p,
                            void** ppUserMem_p,
                            UINT32* pSize_p)
{
    tDualprocReturn         dualRet;
    tMemInfo*               pKernel2UserMemInfo = &drvInstance_l.kernel2UserMem;
    tDualprocSharedMemInst  sharedMemInst;

    if ((ppKernelMem_p == NULL) ||
        (ppUserMem_p == NULL))
        return kErrorNoResource;

    dualRet = dualprocshm_getSharedMemInfo(drvInstance_l.pDualProcDrvInst,
                                           dualprocshm_getLocalProcInst(),
                                           &sharedMemInst);

    if (dualRet != kDualprocSuccessful)
    {
        DEBUG_LVL_ERROR_TRACE("%s() Unable to map kernel memory error %x\n",
                              __func__,
                              dualRet);
        return kErrorNoResource;
    }

    pKernel2UserMemInfo->pKernelVa = (void*)sharedMemInst.baseAddr;
    // TODO: The span for shared memory is doubled to include the
    //       shared memory mapped through atomic modify IP.
    //       Use PCIe bar header to identify the actual size
    pKernel2UserMemInfo->memSize = sharedMemInst.span * 2;

    if (pKernel2UserMemInfo->pUserVa != NULL)
    {
        // The memory is already mapped. Return existing information
        goto Exit;
    }

    if (mapMemory(pKernel2UserMemInfo) != kErrorOk)
    {
        DEBUG_LVL_ERROR_TRACE("%s() error mapping memory\n", __func__);
        return kErrorNoResource;
    }

Exit:
    dualRet = dualprocshm_getSharedMemInfo(drvInstance_l.pDualProcDrvInst,
                                           dualprocshm_getRemoteProcInst(),
                                           &sharedMemInst);

    if (dualRet != kDualprocSuccessful)
    {
        DEBUG_LVL_ERROR_TRACE("%s() Unable to map kernel memory error %x\n",
                              __func__,
                              dualRet);
        return kErrorNoResource;
    }

    drvInstance_l.remoteProcBase = sharedMemInst.baseAddr;
    *ppUserMem_p = pKernel2UserMemInfo->pUserVa;
    *ppKernelMem_p = (void*)sharedMemInst.baseAddr;
    *pSize_p = pKernel2UserMemInfo->memSize;

    DEBUG_LVL_ALWAYS_TRACE("Mapped memory info U:%p K:%p size %x",
                           pKernel2UserMemInfo->pUserVa,
                           sharedMemInst.baseAddr,
                           pKernel2UserMemInfo->memSize);
    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Unmap mapped memory

Unmap and free the kernel to user memory mapped before.

\param[in,out]  pUserMem_p          Pointer to mapped user memory.

\ingroup module_driver_ndispcie
*/
//------------------------------------------------------------------------------
void drv_unmapKernelMem(void* pUserMem_p)
{
    tMemInfo*   pKernel2UserMemInfo = &drvInstance_l.kernel2UserMem;

    UNUSED_PARAMETER(pUserMem_p);

    if (pKernel2UserMemInfo != NULL)
        unmapMemory(pKernel2UserMemInfo);
}

//------------------------------------------------------------------------------
/**
\brief  Write file chunk

This function writes the given file chunk to the file transfer buffer

\param[in]      pIoctlFileChunk_p   Ioctl file chunk buffer

\return The function returns a tOplkError error code.

\ingroup module_driver_ndispcie
*/
//------------------------------------------------------------------------------
tOplkError drv_writeFileBuffer(const tIoctlFileChunk* pIoctlFileChunk_p)
{
    tDualprocReturn dualRet;

    if (pIoctlFileChunk_p == NULL)
        return kErrorNoResource;

    if (!drvInstance_l.fDriverActive)
        return kErrorNoResource;

    if (pIoctlFileChunk_p->desc.length > CONFIG_CTRL_FILE_CHUNK_SIZE)
    {
        DEBUG_LVL_ERROR_TRACE("File chunk size exceeds limit!\n");
        return kErrorNoResource;
    }

    // Write descriptor
    dualRet = dualprocshm_writeDataCommon(drvInstance_l.pDualProcDrvInst,
                                          FIELD_OFFSET(tCtrlBuf, fileChunkDesc),
                                          sizeof(tOplkApiFileChunkDesc),
                                          &pIoctlFileChunk_p->desc);
    if (dualRet != kDualprocSuccessful)
    {
        DEBUG_LVL_ERROR_TRACE("Cannot store file chunk descriptor (0x%X)\n", dualRet);
        return kErrorGeneralError;
    }

    // Write file chunk data into buffer
    dualRet = dualprocshm_writeDataCommon(drvInstance_l.pDualProcDrvInst,
                                          FIELD_OFFSET(tCtrlBuf, aFileChunkBuffer),
                                          pIoctlFileChunk_p->desc.length,
                                          &pIoctlFileChunk_p->pData);
    if (dualRet != kDualprocSuccessful)
    {
        DEBUG_LVL_ERROR_TRACE("Cannot store file chunk data (0x%X)\n", dualRet);
        return kErrorGeneralError;
    }

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Get maximum supported file chunk size

This function returns the maximum file chunk size which is supported by the
CAL implementation.

\return The function returns the supported file chunk size.

\ingroup module_driver_ndispcie
*/
//------------------------------------------------------------------------------
size_t drv_getFileBufferSize(void)
{
    return CONFIG_CTRL_FILE_CHUNK_SIZE;
}

#if defined(CONFIG_INCLUDE_SOC_TIME_FORWARD)
//------------------------------------------------------------------------------
/**
\brief  Get timesync memory offset

Retrieves the SoC memory offset from the dualprocshm library and shares it
with user application.

\param[out]     pSocMemOffs_p       Pointer to SoC memory offset value.
\param[in]      socMemSize_p        Size of the SoC memory.

\return The function returns a tOplkError error code.

\ingroup module_driver_ndispcie
*/
//------------------------------------------------------------------------------
tOplkError drv_getTimesyncMem(ptrdiff_t* pSocMemOffs_p,
                              size_t socMemSize_p)
{
    void*           pMem;
    void*           pBar0;
    tDualprocReturn dualRet = kDualprocSuccessful;

    pBar0 = ndis_getBarAddr(OPLK_PCIEBAR_SHM);

    if (!drvInstance_l.fDriverActive || (pBar0 == NULL) || (pSocMemOffs_p == NULL))
        return kErrorNoResource;

    dualRet = dualprocshm_getMemory(drvInstance_l.pDualProcDrvInst,
                                    DUALPROCSHM_BUFF_ID_TIMESYNC,
                                    &pMem,
                                    &socMemSize_p,
                                    FALSE);

    if (dualRet != kDualprocSuccessful)
    {
        DEBUG_LVL_ERROR_TRACE("%s() Couldn't allocate SoC buffer (0x%X)\n",
                              __func__,
                              dualRet);
        return kErrorNoResource;
    }

    *pSocMemOffs_p = (UINT8*)pMem - (UINT8*)pBar0;
    if (*pSocMemOffs_p == 0)
    {
        DEBUG_LVL_ERROR_TRACE("%s() Timesync shared memory offset is invalid\n", __func__);
        return kErrorNoResource;
    }

    DEBUG_LVL_ALWAYS_TRACE("%s() Timesync shared memory offset is %x\n", __func__, *pSocMemOffs_p);

    return kErrorOk;
}
#endif

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief  Initialize dual processor shared memory driver instance

This routine initializes the driver instance of dualprocshm for HOST processor.

\return Returns a tOplkError error code.

\ingroup module_driver_ndispcie
*/
//------------------------------------------------------------------------------
static tOplkError initDualProcShm(void)
{
    tDualprocReturn dualRet;
    tDualprocConfig dualProcConfig;
    INT             loopCount = 0;

    OPLK_MEMSET(&dualProcConfig, 0, sizeof(tDualprocConfig));

    dualProcConfig.procInstance = kDualProcSecond;

    dualRet = dualprocshm_create(&dualProcConfig, &drvInstance_l.pDualProcDrvInst);
    if (dualRet != kDualprocSuccessful)
    {
        DEBUG_LVL_ERROR_TRACE("%s(): Could not create dual processor driver instance (0x%X)\n",
                              __func__,
                              dualRet);
        dualprocshm_delete(drvInstance_l.pDualProcDrvInst);
        return kErrorNoResource;
    }

    for (loopCount = 0; loopCount <= DPSHM_ENABLE_TIMEOUT_SEC; loopCount++)
    {
        target_msleep(1000U);
        dualRet = dualprocshm_checkShmIntfState(drvInstance_l.pDualProcDrvInst);
        if (dualRet != kDualprocshmIntfDisabled)
            break;
    }

    if (dualRet != kDualprocshmIntfEnabled)
    {
        DEBUG_LVL_ERROR_TRACE("%s(): Dual processor interface is not enabled (0x%X)\n",
                              __func__,
                              dualRet);
        return kErrorNoResource;
    }

    dualRet = dualprocshm_initInterrupts(drvInstance_l.pDualProcDrvInst);
    if (dualRet != kDualprocSuccessful)
    {
        DEBUG_LVL_ERROR_TRACE("%s(): Error Initializing interrupts %x\n ",
                              __func__,
                              dualRet);
        return kErrorNoResource;
    }

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Delete dual processor shared memory driver instance

This routine deletes the driver instance of dualprocshm created during
initialization.

\ingroup module_driver_ndispcie
*/
//------------------------------------------------------------------------------
static void exitDualProcShm(void)
{
    tDualprocReturn dualRet;

    // disable system irq
    dualprocshm_freeInterrupts(drvInstance_l.pDualProcDrvInst);

    dualRet = dualprocshm_delete(drvInstance_l.pDualProcDrvInst);
    if (dualRet != kDualprocSuccessful)
    {
        DEBUG_LVL_ERROR_TRACE("Could not delete dual processor driver inst (0x%X)\n", dualRet);
    }
}

//------------------------------------------------------------------------------
/**
\brief  Initialize event queues

Initializes shared event queues between openPOWERLINK user and kernel stacks.
The memory for the queues are allocated in PCIe memory and is retrieved
using dualprocshm library. The circular buffer library is used to manage the
queues.

\return Returns a tOplkError error code.

*/
//------------------------------------------------------------------------------
static tOplkError initEvent(void)
{
    tCircBufError   circError = kCircBufOk;

    if (!drvInstance_l.fDriverActive)
        return kErrorNoResource;

    circError = circbuf_connect(CIRCBUF_USER_TO_KERNEL_QUEUE, &drvInstance_l.apEventQueueInst[kEventQueueU2K]);
    if (circError != kCircBufOk)
    {
        DEBUG_LVL_ALWAYS_TRACE("PLK : Could not allocate CIRCBUF_USER_TO_KERNEL_QUEUE circbuffer\n");
        return kErrorNoResource;
    }

    circError = circbuf_connect(CIRCBUF_KERNEL_TO_USER_QUEUE, &drvInstance_l.apEventQueueInst[kEventQueueK2U]);
    if (circError != kCircBufOk)
    {
        DEBUG_LVL_ALWAYS_TRACE("PLK : Could not allocate CIRCBUF_KERNEL_TO_USER_QUEUE circbuffer\n");
        return kErrorNoResource;
    }

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Close event queues

Close event queues initialized earlier.

*/
//------------------------------------------------------------------------------
static void exitEvent(void)
{
    if (drvInstance_l.apEventQueueInst[kEventQueueK2U] != NULL)
        circbuf_disconnect(drvInstance_l.apEventQueueInst[kEventQueueK2U]);

    if (drvInstance_l.apEventQueueInst[kEventQueueU2K] != NULL)
        circbuf_disconnect(drvInstance_l.apEventQueueInst[kEventQueueU2K]);
}

//------------------------------------------------------------------------------
/**
\brief  Initialize error handler memory

Retrieves the shared memory for the error handler module. This memory is only
accessible to user space through ioctl calls.

\return Returns a tOplkError error code.

*/
//------------------------------------------------------------------------------
static tOplkError initErrHndl(void)
{
    tDualprocReturn dualRet;
    void*           pBase;
    size_t          span;

    if (!drvInstance_l.fDriverActive)
        return kErrorNoResource;

    if (drvInstance_l.pErrorObjects != NULL)
        return kErrorInvalidOperation;

    dualRet = dualprocshm_getMemory(drvInstance_l.pDualProcDrvInst,
                                    DUALPROCSHM_BUFF_ID_ERRHDLR,
                                    &pBase,
                                    &span,
                                    FALSE);
    if (dualRet != kDualprocSuccessful)
    {
        DEBUG_LVL_ERROR_TRACE("%s(): couldn't get Error counter buffer(%d)\n",
                              __func__,
                              dualRet);
        return kErrorNoResource;
    }

    if (span < sizeof(tErrHndObjects))
    {
        DEBUG_LVL_ERROR_TRACE("%s(): Error Handler Object Buffer too small\n",
                              __func__);
        return kErrorNoResource;
    }

    drvInstance_l.pErrorObjects = (tErrHndObjects*)pBase;

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Free error handler memory

*/
//------------------------------------------------------------------------------
static void exitErrHndl(void)
{
    if (drvInstance_l.pErrorObjects != NULL)
    {
        dualprocshm_freeMemory(drvInstance_l.pDualProcDrvInst,
                               DUALPROCSHM_BUFF_ID_ERRHDLR,
                               FALSE);
        drvInstance_l.pErrorObjects = NULL;
    }
}

//------------------------------------------------------------------------------
/**
\brief  Initialize DLL queues for user layer

This routine initializes the DLL queues shared between user and kernel stack.
The memories for the queue are located in PCIe memory and is accessed using
circular buffer and dualprocshm library.

\return Returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError initDllQueues(void)
{
    tCircBufError   circError = kCircBufOk;

    if (!drvInstance_l.fDriverActive)
        return kErrorNoResource;

    circError = circbuf_connect(CIRCBUF_DLLCAL_TXGEN, &drvInstance_l.apDllQueueInst[kDllCalQueueTxGen]);
    if (circError != kCircBufOk)
    {
        DEBUG_LVL_ERROR_TRACE("PLK : Could not allocate CIRCBUF_DLLCAL_TXGEN circbuffer\n");
        return kErrorNoResource;
    }

    circError = circbuf_connect(CIRCBUF_DLLCAL_TXNMT, &drvInstance_l.apDllQueueInst[kDllCalQueueTxNmt]);
    if (circError != kCircBufOk)
    {
        DEBUG_LVL_ERROR_TRACE("PLK : Could not allocate CIRCBUF_DLLCAL_TXNMT circbuffer\n");
        return kErrorNoResource;
    }

    circError = circbuf_connect(CIRCBUF_DLLCAL_TXSYNC, &drvInstance_l.apDllQueueInst[kDllCalQueueTxSync]);
    if (circError != kCircBufOk)
    {
        DEBUG_LVL_ERROR_TRACE("PLK : Could not allocate CIRCBUF_DLLCAL_TXSYNC circbuffer\n");
        return kErrorNoResource;
    }

#if defined(CONFIG_INCLUDE_VETH)
    circError = circbuf_connect(CIRCBUF_DLLCAL_TXVETH, &drvInstance_l.apDllQueueInst[kDllCalQueueTxVeth]);
    if (circError != kCircBufOk)
    {
        DEBUG_LVL_ERROR_TRACE("PLK : Could not allocate CIRCBUF_DLLCAL_TXVETH circbuffer\n");
        return kErrorNoResource;
    }
#endif

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Free DLL queues for user layer

*/
//------------------------------------------------------------------------------
static void exitDllQueues(void)
{
    if (drvInstance_l.apDllQueueInst[kDllCalQueueTxGen] != NULL)
        circbuf_disconnect(drvInstance_l.apDllQueueInst[kDllCalQueueTxGen]);

    if (drvInstance_l.apDllQueueInst[kDllCalQueueTxNmt] != NULL)
        circbuf_disconnect(drvInstance_l.apDllQueueInst[kDllCalQueueTxNmt]);

    if (drvInstance_l.apDllQueueInst[kDllCalQueueTxSync] != NULL)
        circbuf_disconnect(drvInstance_l.apDllQueueInst[kDllCalQueueTxSync]);

#if defined(CONFIG_INCLUDE_VETH)
    if (drvInstance_l.apDllQueueInst[kDllCalQueueTxVeth] != NULL)
        circbuf_disconnect(drvInstance_l.apDllQueueInst[kDllCalQueueTxVeth]);

#endif
}

//------------------------------------------------------------------------------
/**
\brief  Write data into DLL queue

Writes the data into specified DLL queue shared between user and kernel.

\param[in,out]  pDllCircBuffInst_p  Pointer to the DLL queue instance.
\param[in]      pData_p             Pointer to the data to be inserted.
\param[in]      dataSize_p          Size of data.

\return Returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError insertDataBlock(tCircBufInstance* pDllCircBuffInst_p,
                                  const void* pData_p,
                                  size_t dataSize_p)
{
    tOplkError       ret = kErrorOk;
    tCircBufError    error;

    if (!drvInstance_l.fDriverActive)
        return kErrorNoResource;

    if (pDllCircBuffInst_p == NULL)
    {
        ret = kErrorInvalidInstanceParam;
        goto Exit;
    }

    error = circbuf_writeData(pDllCircBuffInst_p, pData_p, dataSize_p);
    switch (error)
    {
        case kCircBufOk:
            break;

        case kCircBufBufferFull:
            ret = kErrorDllAsyncTxBufferFull;
            break;

        case kCircBufInvalidArg:
        default:
            ret = kErrorNoResource;
            break;
    }

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Map memory to user space

Maps the specified memory into user space.

\param[in,out]  pMemInfo_p          Pointer to memory map information structure
                                    for the memory to be mapped.

\return Returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError mapMemory(tMemInfo* pMemInfo_p)
{
    if (pMemInfo_p->pKernelVa == NULL)
        return kErrorNoResource;

    // Allocate new MDL pointing to PDO memory
    pMemInfo_p->pMdl = IoAllocateMdl(pMemInfo_p->pKernelVa,
                                     pMemInfo_p->memSize,
                                     FALSE,
                                     FALSE,
                                     NULL);

    if (pMemInfo_p->pMdl == NULL)
    {
        DEBUG_LVL_ERROR_TRACE("%s() Error allocating MDL !\n", __func__);
        return kErrorNoResource;
    }

    // Update the MDL with physical addresses
    MmBuildMdlForNonPagedPool(pMemInfo_p->pMdl);
    // Map the memory in user space and get the address
    pMemInfo_p->pUserVa = MmMapLockedPagesSpecifyCache(pMemInfo_p->pMdl,       // MDL
                                                       UserMode,               // Mode
                                                       MmNonCached,            // Caching
                                                       NULL,                   // Address
                                                       FALSE,                  // Bugcheck?
                                                       NormalPagePriority);    // Priority

    if (pMemInfo_p->pUserVa == NULL)
    {
        MmUnmapLockedPages(pMemInfo_p->pUserVa, pMemInfo_p->pMdl);
        IoFreeMdl(pMemInfo_p->pMdl);
        DEBUG_LVL_ERROR_TRACE("%s() Error mapping MDL !\n", __func__);
        return kErrorNoResource;
    }

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Unmap memory from user space

Unmap the specified memory mapped into user space.

\param[in,out]  pMemInfo_p          Pointer to memory map information structure for
                                    the memory to unmap.

\return Returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static void unmapMemory(tMemInfo* pMemInfo_p)
{
    if (pMemInfo_p->pMdl == NULL)
    {
        DEBUG_LVL_ERROR_TRACE("%s() MDL already deleted !\n", __func__);
        return;
    }

    if (pMemInfo_p->pUserVa != NULL)
    {
        MmUnmapLockedPages(pMemInfo_p->pUserVa, pMemInfo_p->pMdl);
        IoFreeMdl(pMemInfo_p->pMdl);
    }
}

#if defined(CONFIG_INCLUDE_VETH)
//------------------------------------------------------------------------------
/**
\brief  Get kernel memory

The routine calculates the offset for the specified PCIe buffer and returns
the corresponding base address for the buffer in kernel virtual memory.

\param[in]      pPCIeBuf_p          Pointer to PCIe memory.
\param[out]     ppKernelMem_p       Double pointer to the kernel virtual memory
                                    for PCIe buffer.
\param[in]      size_p              Size of the memory

\return Returns a tOplkError error code and base address for the PCIe buffer
        in kernel space.
*/
//------------------------------------------------------------------------------
static tOplkError getMemory(const void* pPCIeBuf_p,
                            void** ppKernelMem_p,
                            UINT32 size_p)
{
    tMemInfo*   pKernel2UserMemInfo = &drvInstance_l.kernel2UserMem;
    ptrdiff_t   buffOffset;

    if ((pKernel2UserMemInfo->pKernelVa == NULL) ||
        (ppKernelMem_p == NULL))
        return kErrorNoResource;

    if ((pPCIeBuf_p < (const void*)drvInstance_l.remoteProcBase) ||
        (((const UINT8*)pPCIeBuf_p + size_p) >
         ((const UINT8*)pKernel2UserMemInfo->pKernelVa + pKernel2UserMemInfo->memSize)))
        return kErrorNoResource;

    buffOffset = (const UINT8*)pPCIeBuf_p - (const UINT8*)drvInstance_l.remoteProcBase;

    *ppKernelMem_p = (UINT8*)pKernel2UserMemInfo->pKernelVa + buffOffset;

    return kErrorOk;
}

///-----------------------------------------------------------------------------
/**
\brief  Send callback for virtual Ethernet frame

The routine forwards the virtual frame to kernel stack.

\param[in]      pBuffer_p           Pointer to frame buffer.
\param[in]      size_p              Size of the frame.

\return Returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError nonPlkFrameSendCb(void* pBuffer_p,
                                    size_t size_p)
{
    tOplkError              ret = kErrorOk;
    tIoctlDllCalAsync       asyncFrameInfo;
    tDllAsyncReqPriority    priority = kDllAsyncReqPrioGeneric;
    tCircBufInstance*       pCircBufInstance = drvInstance_l.apEventQueueInst[kEventQueueU2K];
    tCircBufError           circError;
    tEvent                  event;

    if (!drvInstance_l.fDriverActive ||
        (pBuffer_p == NULL) ||
        (size_p == 0))
        return kErrorNoResource;

    asyncFrameInfo.size = size_p;
    asyncFrameInfo.pData = pBuffer_p;
    asyncFrameInfo.queue = kDllCalQueueTxVeth;

    ret = drv_sendAsyncFrame(&asyncFrameInfo);
    if (ret != kErrorOk)
    {
        DEBUG_LVL_ERROR_TRACE("Error sending VEth frame queue %d\n",
                              asyncFrameInfo.queue);
        return ret;
    }

    // post event to DLL
    event.eventSink = kEventSinkDllk;
    event.eventType = kEventTypeDllkFillTx;
    event.eventArg.pEventArg = &priority;
    event.eventArgSize = sizeof(tDllAsyncReqPriority);

    ret = drv_postEvent(&event);

    return ret;
}

///------------------------------------------------------------------------------
/**
\brief  Handle non-PLK frame

This routine parses the received event to identify the receipt of non-PLK frames
and forwards it to NDIS miniport section for processing.

\param[in,out]  pEvent_p            Pointer to event.

\return The function returns a tOplkError error code.
\retval kErrorOk, If the event contains a non-PLK frame and it was successfully
                  handled.
\retval kErrorReject, If the event does not contain a non-PLK frame.
*/
//------------------------------------------------------------------------------
static tOplkError receiveNonPlkFrame(void* pEvent_p)
{
    tEvent*              pEvent;
    UINT16               etherType;
    tFrameInfo*          pFrameInfo = NULL;
    tPlkFrame*           pBuffer;
    UINT32               frameSize;

    if (pEvent_p == NULL)
        return kErrorNoResource;

    pEvent = (tEvent*)pEvent_p;
    if (pEvent->eventArgSize != 0)
        pEvent->eventArg.pEventArg = (void*)((UINT8*)pEvent_p + sizeof(tEvent));
    else
        return kErrorReject;

    switch (pEvent->eventType)
    {
        case kEventTypeAsndRx:
            pBuffer = (tPlkFrame*)pEvent->eventArg.pEventArg;
            frameSize = pEvent->eventArgSize;
            break;

        case kEventTypeAsndRxInfo:
            pFrameInfo = (tFrameInfo*)pEvent->eventArg.pEventArg;

            if (getMemory(pFrameInfo->frame.pBuffer, &pBuffer, pFrameInfo->frameSize) != kErrorOk)
            {
                DEBUG_LVL_ERROR_TRACE("Failed to get kernel memory\n");
                return kErrorNoResource;
            }

            frameSize = pFrameInfo->frameSize;
            break;

        default:
            return kErrorReject;
    }

    if (pBuffer == NULL)
        return kErrorNoResource;

    etherType = ami_getUint16Be(&pBuffer->etherType);
    if (etherType != C_DLL_ETHERTYPE_EPL)
    {
        ndis_vethReceive((void*)pBuffer, frameSize);

        if (pFrameInfo != NULL)
        {
            // Post event to release ASync frame
            tEvent      event;

            // Post the event to free the ASync frame buffer
            event.eventSink = kEventSinkDllkCal;
            event.eventType = kEventTypeReleaseRxFrame;
            event.eventArgSize = sizeof(tFrameInfo);
            event.eventArg.pEventArg = (void*)pFrameInfo;
            drv_postEvent(&event);
        }
        else
            return kErrorNoResource;
    }
    else
        return kErrorReject;

    return kErrorOk;
}
#endif

/// \}
