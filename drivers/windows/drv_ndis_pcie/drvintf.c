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

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <oplk/oplk.h>

#include <kernel/eventk.h>
#include <kernel/eventkcal.h>
#include <errhndkcal.h>
#include <dualprocshm.h>
#include <common/circbuffer.h>

#include <drvintf.h>
#include <ndis.h>

//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#define PROC_ID                        0xFA

//------------------------------------------------------------------------------
// module global vars
//------------------------------------------------------------------------------
#define DUALPROCSHM_BUFF_ID_ERRHDLR    12
#define DUALPROCSHM_BUFF_ID_PDO        13
#define BENCHMARK_OFFSET               0x00001000   //TODO: Get this value from PCIe header files
#define DPSHM_ENABLE_TIMEOUT_SEC       10           // wait for dpshm interface enable time out

//------------------------------------------------------------------------------
// global function prototypes
//------------------------------------------------------------------------------

//============================================================================//
//            P R I V A T E   D E F I N I T I O N S                           //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#define CMD_TIMEOUT_CNT    500               // loop counter for command timeout

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
    PMDL      pMdl;                 ///< Pointer to memory descriptor list describing the memory.
    size_t    memSize;              ///< Size of the memory.
    void*     pKernelVa;            ///< Pointer to memory in kernel space.
    void*     pUserVa;              ///< Pointer to memory mapped in user space.
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
    BOOL                    fDriverActive;                          ///< Flag to identify status of driver interface.
} tDriverInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tDriverInstance    drvInstance_l;

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
                                  UINT8* pData_p, UINT* pDataSize_p);
static tOplkError mapMemory(tMemInfo* pMemInfo_p);
static void       unmapMemory(tMemInfo* pMemInfo_p);

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Initialize driver interface

This function initializes necessary resources required for driver interface.

\return Returns tOplkError error code.

\ingroup module_driver_ndispcie
*/
//------------------------------------------------------------------------------
tOplkError drv_init(void)
{
    tOplkError ret = kErrorOk;

    TRACE("Initialize driver interface...");

    OPLK_MEMSET(&drvInstance_l, 0, sizeof(tDriverInstance));

    // Initialize the dualprocshm library
    ret = initDualProcShm();
    if (ret != kErrorOk)
    {
        DEBUG_LVL_ERROR_TRACE("Dual processor shared memory interface Initialization failed (0x%X)\n",
                              ret);
        return ret;
    }

    drvInstance_l.fDriverActive = TRUE;

    TRACE(" OK\n");

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
    TRACE("Exit driver interface...\n");

    if (drvInstance_l.fDriverActive)
    {
        drvInstance_l.fDriverActive = FALSE;

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

\param  pCtrlCmd_p       Pointer to control command structure.

\return Returns tOplkError error code.

\ingroup module_driver_ndispcie
*/
//------------------------------------------------------------------------------
tOplkError drv_executeCmd(tCtrlCmd* pCtrlCmd_p)
{
    tOplkError    ret = kErrorOk;
    UINT16        status;
    UINT16        cmd;
    INT           timeout;

    if ((pCtrlCmd_p == NULL) || !drvInstance_l.fDriverActive)
        return kErrorNoResource;

    cmd = pCtrlCmd_p->cmd;

    // Clean up stack
    if (cmd == kCtrlCleanupStack || cmd == kCtrlShutdown)
    {
        exitDllQueues();
        exitErrHndl();
        exitEvent();
    }

    if (dualprocshm_writeDataCommon(drvInstance_l.pDualProcDrvInst, FIELD_OFFSET(tCtrlBuf, ctrlCmd),
        sizeof(tCtrlCmd), (UINT8*)pCtrlCmd_p) != kDualprocSuccessful)
        return kErrorNoResource;

    // wait for response
    for (timeout = 0; timeout < CMD_TIMEOUT_CNT; timeout++)
    {
        target_msleep(10);

        if (dualprocshm_readDataCommon(drvInstance_l.pDualProcDrvInst, FIELD_OFFSET(tCtrlBuf, ctrlCmd),
            sizeof(tCtrlCmd), (UINT8*)pCtrlCmd_p) != kDualprocSuccessful)
            return kErrorNoResource;

        if (pCtrlCmd_p->cmd == 0)
            break;
    }

    if (timeout == CMD_TIMEOUT_CNT)
        return kErrorGeneralError;

    if ((cmd == kCtrlInitStack) && (pCtrlCmd_p->retVal == kErrorOk))
    {
        ret = initEvent();
        if (ret != kErrorOk)
        {
            DEBUG_LVL_ERROR_TRACE("Event Initialization Failed (0x%X)\n", ret);
            return ret;
        }

        ret = initErrHndl();
        if (ret != kErrorOk)
        {
            DEBUG_LVL_ERROR_TRACE("Error Module Initialization Failed (0x%X)\n", ret);
            return ret;
        }

        ret = initDllQueues();
        if (ret != kErrorOk)
        {
            DEBUG_LVL_ERROR_TRACE("Dll Queues Initialization Failed (0x%X)\n", ret);
            return ret;
        }
    }

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Read initialization parameters

Read the initialization parameters from the kernel stack.

\param  pInitParam_p       Pointer to initialization parameters structure.

\return Returns tOplkError error code.

\ingroup module_driver_ndispcie
*/
//------------------------------------------------------------------------------
tOplkError drv_readInitParam(tCtrlInitParam* pInitParam_p)
{
    tDualprocReturn    dualRet;

    if (!drvInstance_l.fDriverActive)
        return kErrorNoResource;

    dualRet = dualprocshm_readDataCommon(drvInstance_l.pDualProcDrvInst, FIELD_OFFSET(tCtrlBuf, initParam),
                                         sizeof(tCtrlInitParam), (UINT8*)pInitParam_p);
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

\param  pInitParam_p       Pointer to initialization parameters structure.

\return Returns tOplkError error code.

\ingroup module_driver_ndispcie
*/
//------------------------------------------------------------------------------
tOplkError drv_storeInitParam(tCtrlInitParam* pInitParam_p)
{
    tDualprocReturn    dualRet;

    if (!drvInstance_l.fDriverActive)
        return kErrorNoResource;

    dualRet = dualprocshm_writeDataCommon(drvInstance_l.pDualProcDrvInst, FIELD_OFFSET(tCtrlBuf, initParam),
                                sizeof(tCtrlInitParam), (UINT8*)pInitParam_p);
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

\param  pStatus_p       Pointer to status variable to return.

\return Returns tOplkError error code.

\ingroup module_driver_ndispcie
*/
//------------------------------------------------------------------------------
tOplkError drv_getStatus(UINT16* pStatus_p)
{
    if (!drvInstance_l.fDriverActive)
        return kErrorNoResource;

    if (dualprocshm_readDataCommon(drvInstance_l.pDualProcDrvInst, FIELD_OFFSET(tCtrlBuf, status),
        sizeof(UINT16), (UINT8*)pStatus_p) != kDualprocSuccessful)
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

\param  pHeartbeat_p       Pointer to heartbeat variable to return.

\return Returns tOplkError error code.

\ingroup module_driver_ndispcie
*/
//------------------------------------------------------------------------------
tOplkError drv_getHeartbeat(UINT16* pHeartbeat_p)
{
    if (!drvInstance_l.fDriverActive)
        return kErrorNoResource;

    if (dualprocshm_readDataCommon(drvInstance_l.pDualProcDrvInst, FIELD_OFFSET(tCtrlBuf, heartbeat),
        sizeof(UINT16), (UINT8*)pHeartbeat_p) != kDualprocSuccessful)
    {
        DEBUG_LVL_ERROR_TRACE("Error Reading HeartBeat\n");
        return kErrorNoResource;
    }

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Write asynchronous frame

This routines extracts the asynchronous frame from the IOCTL buffer and writes
it into the specified DLL queue for processing by openPOWERLINK kernel layer.

\param  pArg_p       Pointer to IOCTL buffer.

\return Returns tOplkError error code.

\ingroup module_driver_ndispcie
*/
//------------------------------------------------------------------------------
tOplkError drv_sendAsyncFrame(tIoctlDllCalAsync* pAsyncFrameInfo_p)
{
    tFrameInfo            frameInfo;
    tOplkError            ret =  kErrorOk;

    if ((pAsyncFrameInfo_p == NULL) || !drvInstance_l.fDriverActive)
        return kErrorNoResource;

    frameInfo.frameSize = pAsyncFrameInfo_p->size;
    frameInfo.frame.pBuffer = (tPlkFrame*)((UINT8*)pAsyncFrameInfo_p + sizeof(tIoctlDllCalAsync));

    ret = insertDataBlock(drvInstance_l.apDllQueueInst[pAsyncFrameInfo_p->queue],
                          (UINT8*)frameInfo.frame.pBuffer,
                          &(frameInfo.frameSize));

    if (ret != kErrorOk)
    {
        DEBUG_LVL_ERROR_TRACE("Error sending async frame to queue %d (0x%X)\n",
                              pAsyncFrameInfo_p->queue, ret);
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Write error object

This routines updates the error objects in shared memory with the value passed
from user layer.

\param  pWriteObject_p       Pointer to writeobject to update.

\return Returns tOplkError error code.

\ingroup module_driver_ndispcie
*/
//------------------------------------------------------------------------------
tOplkError drv_writeErrorObject(tErrHndIoctl* pWriteObject_p)
{
    tErrHndObjects*   errorObjects = drvInstance_l.pErrorObjects;

    if (pWriteObject_p == NULL || !drvInstance_l.fDriverActive)
        return kErrorNoResource;

    *((UINT32*)((UINT8*)errorObjects + pWriteObject_p->offset)) = pWriteObject_p->errVal;

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Read error object

This routines fetches the error objects in shared memory to be passed to user
layer.

\param  pWriteObject_p       Pointer to pReadObject_p to fetch.

\return Returns tOplkError error code.

\ingroup module_driver_ndispcie
*/
//------------------------------------------------------------------------------
tOplkError drv_readErrorObject(tErrHndIoctl* pReadObject_p)
{
    tErrHndObjects*   errorObjects = drvInstance_l.pErrorObjects;

    if ((pReadObject_p == NULL) || !drvInstance_l.fDriverActive)
        return kErrorNoResource;

    pReadObject_p->errVal = *((UINT32*)((UINT8*)errorObjects + pReadObject_p->offset));

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Post an user event

Copies the event from user layer into user to kernel event queue.

\param  pEvent_p    Pointer to event.

\return Returns tOplkError error code.

\ingroup module_driver_ndispcie
*/
//------------------------------------------------------------------------------
tOplkError drv_postEvent(void* pEvent_p)
{
    tOplkError          ret = kErrorOk;
    tCircBufError       circError;
    tEvent              event;
    tCircBufInstance*   pCircBufInstance = drvInstance_l.apEventQueueInst[kEventQueueU2K];

    if ((pEvent_p == NULL) || !drvInstance_l.fDriverActive)
        return kErrorNoResource;

    OPLK_MEMCPY(&event, pEvent_p, sizeof(tEvent));

    if (event.eventArgSize != 0)
        event.eventArg.pEventArg = (void*)((UINT8*)pEvent_p + sizeof(tEvent));

    if (event.eventArgSize == 0)
    {
        circError = circbuf_writeData(pCircBufInstance, &event, sizeof(tEvent));
    }
    else
    {
        circError = circbuf_writeMultipleData(pCircBufInstance, pEvent_p, sizeof(tEvent),
                                              event.eventArg.pEventArg, event.eventArgSize);
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
\brief  Get an user event

Retrieves an event from kernel to user event queue for the user layer.

\param  pEvent_p    Pointer to event memory.
\param  pSize_p     Size of the event buffer.

\return Returns tOplkError error code and the size of the read data.

\ingroup module_driver_ndispcie
*/
//------------------------------------------------------------------------------
tOplkError drv_getEvent(void* pEvent_p, size_t* pSize_p)
{
    tCircBufInstance*   pCircBufInstance = drvInstance_l.apEventQueueInst[kEventQueueK2U];

    if ((pEvent_p == NULL) || (pSize_p == NULL) || !drvInstance_l.fDriverActive)
        return kErrorNoResource;

    if (circbuf_getDataCount(pCircBufInstance) > 0)
    {
        circbuf_readData(pCircBufInstance, pEvent_p,
                         sizeof(tEvent) + MAX_EVENT_ARG_SIZE, pSize_p);
    }
    else
    {
        *pSize_p = 0;
    }

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Get PDO memory offset

Retrieves the PDO memory offset from the dualprocshm library and shares it
with user application.

\param  pPdoMemOffs_p    Pointer to PDO memory offset value.
\param  memSize_p        Size of the PDO memory.

\return Returns tOplkError error code.

\ingroup module_driver_ndispcie
*/
//------------------------------------------------------------------------------
tOplkError drv_getPdoMem(UINT32* pPdoMemOffs_p, size_t memSize_p)
{
    tDualprocReturn    dualRet;
    UINT32             offset;
    UINT8*             pMem = NULL;
    UINT8*             bar0Addr = (UINT8*)ndis_getBarAddr(OPLK_PCIEBAR_SHM);

    if (!drvInstance_l.fDriverActive)
        return kErrorNoResource;

    dualRet = dualprocshm_getMemory(drvInstance_l.pDualProcDrvInst,
                                    DUALPROCSHM_BUFF_ID_PDO, &pMem, &memSize_p, FALSE);
    if (dualRet != kDualprocSuccessful)
    {
        DEBUG_LVL_ERROR_TRACE("%s() couldn't allocate Pdo buffer (0x%X)\n",
                              __func__, dualRet);
        return kErrorNoResource;
    }

    offset = (UINT32)(pMem - bar0Addr);

    *pPdoMemOffs_p = offset;

    TRACE("%s() PDO memory offset is %x\n", __func__, offset);
    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Get benchmark base

Retrieves the benchmark memory from NDIS driver and maps it into user virtual
address space for providing access to user layer.

\param  ppBenchmarkMem_p    Pointer to benchmark memory.

\return Returns tOplkError error code.

\ingroup module_driver_ndispcie
*/
//------------------------------------------------------------------------------
tOplkError drv_getBenchmarkMem(UINT8** ppBenchmarkMem_p)
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

    TRACE("%s() Benchmark memory address in user space %p\n", __func__, pBenchmarkMemInfo->pUserVa);
    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Free Benchmark memory

Frees the benchmark memory mapped in user space.

\param  pBenchmarkMem_p    Pointer to benchmark memory.

\ingroup module_driver_ndispcie
*/
//------------------------------------------------------------------------------
void drv_freeBenchmarkMem(UINT8* pBenchmarkMem_p)
{
    tMemInfo*   pBenchmarkMemInfo = &drvInstance_l.benchmarkMem;

    unmapMemory(pBenchmarkMemInfo);

    pBenchmarkMem_p = NULL;
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

\param  ppKernelMem_p    Double pointer to kernel memory.
\param  ppUserMem_p      Double pointer to kernel memory mapped in user layer.
\param  pSize_p          Size of the shared memory.

\return Returns tOplkError error code.

\ingroup module_driver_ndispcie
*/
//------------------------------------------------------------------------------
tOplkError drv_mapKernelMem(UINT8** ppKernelMem_p, UINT8** ppUserMem_p, UINT32* pSize_p)
{
    tDualprocReturn         dualRet;
    tMemInfo*               pKernel2UserMemInfo = &drvInstance_l.kernel2UserMem;
    tDualprocSharedMemInst  sharedMemInst;

    if (*ppKernelMem_p == NULL || *ppUserMem_p == NULL)
        return kErrorNoResource;

    dualRet = dualprocshm_getSharedMemInfo(drvInstance_l.pDualProcDrvInst,
                                           dualprocshm_getLocalProcInst(),
                                           &sharedMemInst);

    if (dualRet != kDualprocSuccessful)
    {
        DEBUG_LVL_ERROR_TRACE("%s() Unable to map kernel memory error %x\n",
                              __func__, dualRet);
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
                              __func__, dualRet);
        return kErrorNoResource;
    }

    *ppUserMem_p = pKernel2UserMemInfo->pUserVa;
    *ppKernelMem_p = (UINT8*)sharedMemInst.baseAddr;
    *pSize_p = pKernel2UserMemInfo->memSize;

    TRACE("Mapped memory info U:%p K:%p size %x", pKernel2UserMemInfo->pUserVa,
                                                  (UINT8*)sharedMemInst.baseAddr,
                                                  pKernel2UserMemInfo->memSize);
    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Unmap mapped memory

Unmap and free the kernel to user memory mapped before.

\param  pUserMem_p    Pointer to mapped user memory.

\ingroup module_driver_ndispcie
*/
//------------------------------------------------------------------------------
void drv_unmapKernelMem(UINT8* pUserMem_p)
{
    tMemInfo*    pKernel2UserMemInfo = &drvInstance_l.kernel2UserMem;

    if (pKernel2UserMemInfo != NULL)
        unmapMemory(pKernel2UserMemInfo);

    pUserMem_p = NULL;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief  Initialize dual processor shared memory driver instance

This routine initializes the driver instance of dualprocshm for HOST processor.

\return Returns tOplkError error code.

\ingroup module_driver_ndispcie
*/
//------------------------------------------------------------------------------
static tOplkError initDualProcShm(void)
{
    tDualprocReturn     dualRet;
    tDualprocConfig     dualProcConfig;
    INT                 loopCount = 0;

    OPLK_MEMSET(&dualProcConfig, 0, sizeof(tDualprocConfig));

    dualProcConfig.procInstance = kDualProcSecond;
    dualProcConfig.procId = PROC_ID;

    dualRet = dualprocshm_create(&dualProcConfig, &drvInstance_l.pDualProcDrvInst);
    if (dualRet != kDualprocSuccessful)
    {
        DEBUG_LVL_ERROR_TRACE(" {%s} Could not create dual processor driver instance (0x%X)\n",
                              __func__, dualRet);
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
        DEBUG_LVL_ERROR_TRACE("%s Dual processor interface is not enabled (0x%X)\n",
                              __func__, dualRet);
        return kErrorNoResource;
    }

    dualRet = dualprocshm_initInterrupts(drvInstance_l.pDualProcDrvInst);
    if (dualRet != kDualprocSuccessful)
    {
        DEBUG_LVL_ERROR_TRACE("{%s} Error Initializing interrupts %x\n ", __func__, dualRet);
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
    tDualprocReturn    dualRet;

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

\return Returns tOplkError error code.

*/
//------------------------------------------------------------------------------
static tOplkError initEvent(void)
{
    tCircBufError    circError = kCircBufOk;

    if (!drvInstance_l.fDriverActive)
        return kErrorNoResource;

    circError = circbuf_connect(CIRCBUF_USER_TO_KERNEL_QUEUE, &drvInstance_l.apEventQueueInst[kEventQueueU2K]);
    if (circError != kCircBufOk)
    {
        TRACE("PLK : Could not allocate CIRCBUF_USER_TO_KERNEL_QUEUE circbuffer\n");
        return kErrorNoResource;
    }

    circError = circbuf_connect(CIRCBUF_KERNEL_TO_USER_QUEUE, &drvInstance_l.apEventQueueInst[kEventQueueK2U]);
    if (circError != kCircBufOk)
    {
        TRACE("PLK : Could not allocate CIRCBUF_KERNEL_TO_USER_QUEUE circbuffer\n");
        return kErrorNoResource;
    }

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Close event queues

Close event queues initialized earlier.

\return Returns tOplkError error code.

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
accessible to user space through IOCTL calls.

\return Returns tOplkError error code.

*/
//------------------------------------------------------------------------------
static tOplkError initErrHndl(void)
{
    tDualprocReturn    dualRet;
    UINT8*             pBase;
    size_t             span;

    if (!drvInstance_l.fDriverActive)
        return kErrorNoResource;

    if (drvInstance_l.pErrorObjects != NULL)
        return kErrorInvalidOperation;

    dualRet = dualprocshm_getMemory(drvInstance_l.pDualProcDrvInst, DUALPROCSHM_BUFF_ID_ERRHDLR,
                                    &pBase, &span, FALSE);
    if (dualRet != kDualprocSuccessful)
    {
        DEBUG_LVL_ERROR_TRACE("%s() couldn't get Error counter buffer(%d)\n",
                              __func__, dualRet);
        return kErrorNoResource;
    }

    if (span < sizeof(tErrHndObjects))
    {
        DEBUG_LVL_ERROR_TRACE("%s: Error Handler Object Buffer too small\n",
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
        dualprocshm_freeMemory(drvInstance_l.pDualProcDrvInst, DUALPROCSHM_BUFF_ID_ERRHDLR, FALSE);
        drvInstance_l.pErrorObjects = NULL;
    }
}

//------------------------------------------------------------------------------
/**
\brief  Initialize DLL queues for user layer

This routine initializes the DLL queues shared between user and kernel stack.
The memories for the queue are located in PCIe memory and is accessed using
circular buffer and dualprocshm library.

\return Returns tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError initDllQueues(void)
{
    tCircBufError    circError = kCircBufOk;

    if (!drvInstance_l.fDriverActive)
        return kErrorNoResource;

    circError = circbuf_connect(CIRCBUF_DLLCAL_TXGEN, &drvInstance_l.apDllQueueInst[kDllCalQueueTxGen]);
    if (circError != kCircBufOk)
    {
        TRACE("PLK : Could not allocate CIRCBUF_DLLCAL_TXGEN circbuffer\n");
        return kErrorNoResource;
    }

    circError = circbuf_connect(CIRCBUF_DLLCAL_TXNMT, &drvInstance_l.apDllQueueInst[kDllCalQueueTxNmt]);
    if (circError != kCircBufOk)
    {
        TRACE("PLK : Could not allocate CIRCBUF_DLLCAL_TXNMT circbuffer\n");
        return kErrorNoResource;
    }

    circError = circbuf_connect(CIRCBUF_DLLCAL_TXSYNC, &drvInstance_l.apDllQueueInst[kDllCalQueueTxSync]);
    if (circError != kCircBufOk)
    {
        TRACE("PLK : Could not allocate CIRCBUF_DLLCAL_TXSYNC circbuffer\n");
        return kErrorNoResource;
    }

    //TODO: VETH to be integrated later
    /*    circError = circbuf_connect(CIRCBUF_DLLCAL_TXVETH, &drvInstance_l.dllQueueInst[kDllCalQueueTxVeth]);

        if (circError != kCircBufOk)
        {
        TRACE("PLK : Could not allocate CIRCBUF_DLLCAL_TXVETH circbuffer\n");
        return kErrorNoResource;
        }
        */

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
    /*
    //TODO: VETH to be integrated later
    if (drvInstance_l.dllQueueInst[kDllCalQueueTxVeth] != NULL)
        circbuf_disconnect(drvInstance_l.dllQueueInst[kDllCalQueueTxVeth]);
    */
}

//------------------------------------------------------------------------------
/**
\brief  Write data into DLL queue

Writes the data into specified DLL queue shared between user and kernel.

\param  pDllCircBuffInst_p  Pointer to the DLL queue instance.
\param  pData_p             Pointer to the data to be inserted.
\param  pDataSize_p         Pointer to size of data.

\return Returns tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError insertDataBlock(tCircBufInstance* pDllCircBuffInst_p,
                                  UINT8* pData_p, UINT* pDataSize_p)
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

    error = circbuf_writeData(pDllCircBuffInst_p, pData_p, *pDataSize_p);
    switch (error)
    {
        case kCircBufOk:
            break;

        case kCircBufExceedDataSizeLimit:
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

\param  pMemInfo_p          Pointer to memory map information structure for the
                            memory to be mapped.

\return Returns tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError mapMemory(tMemInfo* pMemInfo_p)
{
    if (pMemInfo_p->pKernelVa == NULL)
        return kErrorNoResource;

    // Allocate new MDL pointing to PDO memory
    pMemInfo_p->pMdl = IoAllocateMdl(pMemInfo_p->pKernelVa, pMemInfo_p->memSize,
                                     FALSE, FALSE, NULL);

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
                                                       MmCached,               // Caching
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

\param  pMemInfo_p          Pointer to memory map information structure for the
                            memory to unmap.

\return Returns tOplkError error code.
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

/// \}
