/**
********************************************************************************
\file   drvintf-dualprocshm.c

\brief  Dualprocshm interface module to kernel stack for openPOWERLINK
        interface driver

This module handles all the application request forwarded to the openPOWERLINK
interface driver in Linux kernel. It uses dualprocshm and circbuf libraries to
manage PDO memory, error objects shared memory, event and DLL queues.

The module also implements mapping of kernel stack memory into user stack to
provide direct access to user application for specific shared memory regions.

\ingroup module_driver_linux_kernel_pcie
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
#include <errhndkcal.h>
#include <dualprocshm.h>
#include <common/circbuffer.h>
#include <drvintf.h>
#include <kernel/pdokcal.h>
#include <common/timer.h>
#if defined(CONFIG_INCLUDE_VETH)
#include <kernel/veth.h>
#include <common/ami.h>
#endif

//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#define PROC_INSTANCE_ID                0xBA

//------------------------------------------------------------------------------
// module global vars
//------------------------------------------------------------------------------
// $$ Get the following value from configuration header files
#define DUALPROCSHM_BUFF_ID_ERRHDLR     12
#define DUALPROCSHM_BUFF_ID_PDO         13
#define BENCHMARK_OFFSET                0x00001000

//------------------------------------------------------------------------------
// global function prototypes
//------------------------------------------------------------------------------

//============================================================================//
//            P R I V A T E   D E F I N I T I O N S                           //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#define CMD_TIMEOUT_CNT                 500 // Loop count for command timeout in units of 10ms
#define DPSHM_ENABLE_TIMEOUT_SEC        10  // Wait for dpshm interface enable time out
#define DRV_DLLCALTXQUEUE_INSTCNT       (kDllCalQueueTxVeth + 1)    // Number of queue instances in dllcal. The indices from 1 to
                                                                    // kDllCalQueueTxVeth are used for storing the correspoding queue instance.
                                                                    // Zeroeth index is unused.

//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------

/**
\brief Interface module instance - User Layer

This instance stores the local parameters used by the
interface module during runtime.
*/
typedef struct
{
    tDualprocDrvInstance    dualProcDrvInst;                    ///< Dual processor driver instance.
    tCircBufInstance*       apEventQueueInst[kEventQueueNum];   ///< Event queue instances.
    tCircBufInstance*       apDllQueueInst[DRV_DLLCALTXQUEUE_INSTCNT]; ///< DLL queue instances.
    tErrHndObjects*         pErrorObjects;                      ///< Pointer to error objects.
    BOOL                    fDriverActive;                      ///< Flag to identify status of driver interface.
#if defined(CONFIG_INCLUDE_VETH)
    BOOL                    fVEthActive;                        ///< Flag to indicate whether the VEth interface is intialized.
    tDrvIntfCbVeth          pfnCbVeth;                          ///< Callback function to the VEth interface.
#endif
    ULONG                   shmMemLocal;                        ///< Shared memory base address for local processor (OS).
    ULONG                   shmMemRemote;                       ///< Shared memory base address for remote processor (PCP).
    size_t                  shmSize;                            ///< Shared memory span.
}tDrvIntfInstance;
//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tDrvIntfInstance    drvIntfInstance_l;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static tOplkError   initDualProcShmDriver(void);
static tOplkError   initStackInterface(void);
static tOplkError   initEventInterface(void);
static tOplkError   initDllQueueInterface(void);
static tOplkError   initErrHandleInterface(void);
static void         exitDualProcShmDriver(void);
static void         exitStackInterface(void);
static void         exitEventInterface(void);
static void         exitDllQueueInterface(void);
static void         exitErrHandleInterface(void);
static tOplkError   insertAsyncDataBlock(tCircBufInstance* pDllCircBuffInst_p,
                                         UINT8* pData_p, UINT* pDataSize_p);

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Initialize driver interface

This function initializes necessary resources required for driver interface.

\return Returns tOplkError error code.

\ingroup module_driver_linux_kernel_pcie
*/
//------------------------------------------------------------------------------
tOplkError drvintf_init(void)
{
    tOplkError ret = kErrorOk;

    DEBUG_LVL_DRVINTF_TRACE("Initialize driver interface...");

    OPLK_MEMSET(&drvIntfInstance_l, 0, sizeof(tDrvIntfInstance));

    // Initialize the dualprocshm library
    ret = initDualProcShmDriver();
    if (ret != kErrorOk)
    {
        DEBUG_LVL_ERROR_TRACE("Dual processor shared memory interface Initialization failed (0x%X)\n",
                              ret);
        return ret;
    }

    drvIntfInstance_l.fDriverActive = TRUE;

    DEBUG_LVL_DRVINTF_TRACE(" OK\n");

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Close the driver interface

This function frees all the resources used by the driver interface and shuts down
the interface.

\ingroup module_driver_linux_kernel_pcie
*/
//------------------------------------------------------------------------------
void drvintf_exit(void)
{
    DEBUG_LVL_DRVINTF_TRACE("Exit driver interface...\n");

    if (drvIntfInstance_l.fDriverActive)
    {
        drvIntfInstance_l.fDriverActive = FALSE;

        // Close dualprocshm library interface
        exitDualProcShmDriver();
    }
}

//------------------------------------------------------------------------------
/**
\brief  Execute a control command from user application

This function parses the control command from user and passes it to PCP
control module for processing. The return value is again passed to user by
copying it into the common control structure.

\param  pCtrlCmd_p       Pointer to control command structure.

\return Returns tOplkError error code.

\ingroup module_driver_linux_kernel_pcie
*/
//------------------------------------------------------------------------------
tOplkError drvintf_executeCmd(tCtrlCmd* pCtrlCmd_p)
{
    tOplkError      ret = kErrorOk;
    UINT16          cmd = pCtrlCmd_p->cmd;
    INT             timeout;

    // Clean up stack
    if ((cmd == kCtrlCleanupStack) || (cmd == kCtrlShutdown))
    {
        exitStackInterface();
    }

    if (dualprocshm_writeDataCommon(drvIntfInstance_l.dualProcDrvInst,
                                    FIELD_OFFSET(tCtrlBuf, ctrlCmd),
                                    sizeof(tCtrlCmd),
                                    (UINT8*)pCtrlCmd_p) != kDualprocSuccessful)
        return kErrorNoResource;

    // Wait for response
    for (timeout = 0; timeout < CMD_TIMEOUT_CNT; timeout++)
    {
        msleep(10);

        if (dualprocshm_readDataCommon(drvIntfInstance_l.dualProcDrvInst,
                                       FIELD_OFFSET(tCtrlBuf, ctrlCmd),
                                       sizeof(tCtrlCmd),
                                       (UINT8*)pCtrlCmd_p) != kDualprocSuccessful)
            return kErrorNoResource;

        if (pCtrlCmd_p->cmd == 0)
            break;
    }

    if (timeout == CMD_TIMEOUT_CNT)
        return kErrorGeneralError;

    if ((cmd == kCtrlInitStack) && (pCtrlCmd_p->retVal == kErrorOk))
    {
        ret = initStackInterface();
        if (ret != kErrorOk)
        {
            DEBUG_LVL_ERROR_TRACE("Stack interface initialization to PCP Failed (0x%X)\n", ret);
            pCtrlCmd_p->retVal = ret;
            return ret;
        }
    }

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Wait for a sync event

The function waits for a sync event.

\return The function returns a tOplkError error code.

\ingroup module_driver_linux_kernel_pcie
*/
//------------------------------------------------------------------------------
tOplkError drvintf_waitSyncEvent(void)
{
    return timesynckcal_waitSyncEvent();
}

//------------------------------------------------------------------------------
/**
\brief  Read initialization parameters

Read the initialization parameters from the kernel stack.

\param  pInitParam_p       Pointer to initialization parameters structure.

\return Returns tOplkError error code.

\ingroup module_driver_linux_kernel_pcie
*/
//------------------------------------------------------------------------------
tOplkError drvintf_readInitParam(tCtrlInitParam* pInitParam_p)
{
    tDualprocReturn    dualRet;
    tOplkError         ret = kErrorOk;

    if (!drvIntfInstance_l.fDriverActive)
        return kErrorNoResource;

    dualRet = dualprocshm_readDataCommon(drvIntfInstance_l.dualProcDrvInst,
                                         FIELD_OFFSET(tCtrlBuf, initParam),
                                         sizeof(tCtrlInitParam),
                                         (UINT8*)pInitParam_p);

    if (dualRet != kDualprocSuccessful)
    {
        DEBUG_LVL_ERROR_TRACE("Cannot read initparam (0x%X)\n", dualRet);
        return kErrorNoResource;
    }

#if defined(CONFIG_INCLUDE_VETH)
    // Initialize virtual Ethernet interface
    if (drvIntfInstance_l.fVEthActive == FALSE)
    {
        ret = veth_init((UINT8*)(tCtrlInitParam*)pInitParam_p->aMacAddress);
        if (ret != kErrorOk)
        {
            DEBUG_LVL_ERROR_TRACE("VEth Initialization Failed %x\n", ret);
            return ret;
        }

        drvIntfInstance_l.fVEthActive = TRUE;
    }
#endif

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Write initialization parameters

Write the initialization parameters from the user layer into kernel layer
memory of stack.

\param  pInitParam_p       Pointer to initialization parameters structure.

\return Returns tOplkError error code.

\ingroup module_driver_linux_kernel_pcie
*/
//------------------------------------------------------------------------------
tOplkError drvintf_storeInitParam(tCtrlInitParam* pInitParam_p)
{
    tDualprocReturn    dualRet;

    if (!drvIntfInstance_l.fDriverActive)
        return kErrorNoResource;

    dualRet = dualprocshm_writeDataCommon(drvIntfInstance_l.dualProcDrvInst,
                                          FIELD_OFFSET(tCtrlBuf, initParam),
                                          sizeof(tCtrlInitParam),
                                          (UINT8*)pInitParam_p);
    if (dualRet != kDualprocSuccessful)
    {
        DEBUG_LVL_ERROR_TRACE("Cannot store initparam (0x%X)\n", dualRet);
        return kErrorNoResource;
    }

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Get kernel stack status

Return the current status of kernel stack.

\param  pStatus_p       Pointer to status variable to return.

\return Returns tOplkError error code.

\ingroup module_driver_linux_kernel_pcie
*/
//------------------------------------------------------------------------------
tOplkError drvintf_getStatus(UINT16* pStatus_p)
{
    if (!drvIntfInstance_l.fDriverActive)
        return kErrorNoResource;

    if (dualprocshm_readDataCommon(drvIntfInstance_l.dualProcDrvInst,
                                   FIELD_OFFSET(tCtrlBuf, status),
                                   sizeof(UINT16),
                                   (UINT8*)pStatus_p) != kDualprocSuccessful)
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

\param  pHeartbeat       Pointer to heartbeat variable to return.

\return Returns tOplkError error code.

\ingroup module_driver_linux_kernel_pcie
*/
//------------------------------------------------------------------------------
tOplkError drvintf_getHeartbeat(UINT16* pHeartbeat_p)
{
    if (!drvIntfInstance_l.fDriverActive)
        return kErrorNoResource;

    if (dualprocshm_readDataCommon(drvIntfInstance_l.dualProcDrvInst,
                                   FIELD_OFFSET(tCtrlBuf, heartbeat),
                                   sizeof(UINT16),
                                   (UINT8*)pHeartbeat_p) != kDualprocSuccessful)
    {
        DEBUG_LVL_ERROR_TRACE("Error Reading HeartBeat\n");
        return kErrorNoResource;
    }

    return kErrorOk;
}

#if defined(CONFIG_INCLUDE_VETH)
///------------------------------------------------------------------------------
/**
\brief  Write virtual Ethernet frame

This routine extracts the non POWERLINK Ethernet frame data from the passed
frame structure and uses the async data write function to post it into the DLL
VEth queue for processing by kernel stack.

\param  pFrameInfo_p    Pointer to the VEth frame buffer.

\ingroup module_driver_linux_kernel_pcie
*/
//------------------------------------------------------------------------------
tOplkError drvintf_sendVethFrame(tFrameInfo* pFrameInfo_p)
{
    tOplkError              ret = kErrorOk;
    tEvent                  event;
    tDllAsyncReqPriority    priority = kDllAsyncReqPrioGeneric;

    if (!drvIntfInstance_l.fDriverActive)
        return kErrorNoResource;

    ret = drvintf_sendAsyncFrame(kDllCalQueueTxVeth, pFrameInfo_p->frameSize,
                                 pFrameInfo_p->frame.pBuffer);
    if (ret != kErrorOk)
    {
        DEBUG_LVL_ERROR_TRACE("Error sending VEth frame queue %d\n",
                              kDllCalQueueTxVeth);
        return ret;
    }

    // post event to DLL
    event.eventSink = kEventSinkDllk;
    event.eventType = kEventTypeDllkFillTx;
    OPLK_MEMSET(&event.netTime, 0x00, sizeof(event.netTime));
    event.eventArg.pEventArg = &priority;
    event.eventArgSize = sizeof(priority);
    ret = drvintf_postEvent(&event);

    return ret;
}

///------------------------------------------------------------------------------
/**
\brief  Register VEth Rx handler

This routines saves the callback function to be called when a non POWERLINK
frame us received by the driver. If NULL is passed to this function, no callback
is not called.

\param  pfnDrvIntfCbVeth_p  Pointer to callback function.

\ingroup module_driver_linux_kernel_pcie
*/
//------------------------------------------------------------------------------
tOplkError drvintf_regVethHandler(tDrvIntfCbVeth pfnDrvIntfCbVeth_p)
{
    drvIntfInstance_l.pfnCbVeth = pfnDrvIntfCbVeth_p;
    return kErrorOk;
}
#endif

///------------------------------------------------------------------------------
/**
\brief  Write asynchronous frame

This routines writes asynchronous frame buffer into the specified DLL queue
 for processing by PCP.

\param  queue_p         Dll queue id, to which the frame would be copied.
\param  size_p          Size of the async data.
\param  pData_p         Pointer to the async buffer.

\return Returns tOplkError error code.

\ingroup module_driver_linux_kernel_pcie
*/
//------------------------------------------------------------------------------
tOplkError drvintf_sendAsyncFrame(tDllCalQueue queue_p,
                                  size_t size_p,
                                  void* pData_p)
{
    tOplkError              ret = kErrorOk;

    if (!drvIntfInstance_l.fDriverActive)
        return kErrorNoResource;

    ret = insertAsyncDataBlock(drvIntfInstance_l.apDllQueueInst[queue_p],
                               (UINT8*)pData_p,
                               (UINT*)(&size_p));

    if (ret != kErrorOk)
    {
        DEBUG_LVL_ERROR_TRACE("Error sending async frame queue %d\n", queue_p);
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Write error object

This routines updates the error objects in shared memory with the value passed
from user layer.

\param  offset_p        Offset of the error object.
\param  errVal_p        Error value to be written.

\return Returns tOplkError error code.

\ingroup module_driver_linux_kernel_pcie
*/
//------------------------------------------------------------------------------
tOplkError drvintf_writeErrorObject(UINT32 offset_p, UINT32 errVal_p)
{
    tErrHndObjects*   errorObjects = drvIntfInstance_l.pErrorObjects;

    if (!drvIntfInstance_l.fDriverActive)
        return kErrorNoResource;

    *((UINT32*)((UINT8*)errorObjects + offset_p)) = errVal_p;

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Read error object

This routines fetches the error objects in shared memory to be passed to user
layer.

\param  offset_p        Offset of the error object.
\param  pErrVal_p       Pointer to copy the read error value.

\return Returns tOplkError error code.

\ingroup module_driver_linux_kernel_pcie
*/
//------------------------------------------------------------------------------
tOplkError drvintf_readErrorObject(UINT32 offset_p, UINT32* pErrVal_p)
{
    tErrHndObjects*   errorObjects = drvIntfInstance_l.pErrorObjects;

    if (!drvIntfInstance_l.fDriverActive)
        return kErrorNoResource;

    *pErrVal_p = *((UINT32*)((UINT8*)errorObjects + offset_p));
    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Post an user event

Copies the event from user layer into user to kernel(U2K) event queue.

\param  pEvent_p    Pointer to user event memory.

\return Returns tOplkError error code.

\ingroup module_driver_linux_kernel_pcie
*/
//------------------------------------------------------------------------------
tOplkError drvintf_postEvent(tEvent* pEvent_p)
{
    tOplkError          ret = kErrorOk;
    tCircBufError       circBufErr = kCircBufOk;
    tCircBufInstance*   pCircBufInstance = drvIntfInstance_l.apEventQueueInst[kEventQueueU2K];

    if ((pEvent_p == NULL) || (!drvIntfInstance_l.fDriverActive))
        return kErrorNoResource;

    if (pEvent_p->eventArgSize == 0)
    {
        circBufErr = circbuf_writeData(pCircBufInstance, pEvent_p, sizeof(tEvent));
    }
    else
    {
        circBufErr = circbuf_writeMultipleData(pCircBufInstance, (void*)pEvent_p,
                                               sizeof(tEvent),
                                               pEvent_p->eventArg.pEventArg,
                                               pEvent_p->eventArgSize);
    }

    if (circBufErr != kCircBufOk)
    {
        DEBUG_LVL_ERROR_TRACE("Error in Post event %x\n", circBufErr);
        ret = kErrorEventPostError;
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Get an user event

Retrieves an event from kernel to user event(K2U) queue for the user layer.

\param  pK2UEvent_p Pointer to user event memory.
\param  pSize_p     Pointer to store the size of the event buffer.

\return Returns tOplkError error code and the size of the read data.

\ingroup module_driver_linux_kernel_pcie
*/
//------------------------------------------------------------------------------
tOplkError drvintf_getEvent(tEvent* pK2UEvent_p, size_t* pSize_p)
{
    tCircBufError           circBufErr = kCircBufOk;
    tCircBufInstance*       pCircBufInstance = drvIntfInstance_l.apEventQueueInst[kEventQueueK2U];
    tOplkError              ret = kErrorOk;
#if defined(CONFIG_INCLUDE_VETH)
    tFrameInfo*             pFrameInfo;
    tFrameInfo              frameInfo;
    UINT16                  etherType;
    UINT8*                  pKernelMemBuffer = NULL;
    UINT8*                  pUserMemBuffer = NULL;
    tEvent                  u2kEvent;
#endif

    if ((pK2UEvent_p == NULL) || (pSize_p == NULL) ||
        (!drvIntfInstance_l.fDriverActive))
        return kErrorNoResource;

    if (circbuf_getDataCount(pCircBufInstance) > 0)
    {
        circBufErr = circbuf_readData(pCircBufInstance, (void*)pK2UEvent_p,
                                      sizeof(tEvent) + MAX_EVENT_ARG_SIZE,
                                      pSize_p);
        if ((circBufErr != kCircBufOk) && (circBufErr != kCircBufNoReadableData))
        {
            *pSize_p = 0;
            DEBUG_LVL_ERROR_TRACE("Error in reading circular buffer event data!!\n");
            return kErrorInvalidInstanceParam;
        }

#if defined(CONFIG_INCLUDE_VETH)
        if (drvIntfInstance_l.pfnCbVeth != NULL)
        {
            // Check if this is a VEth event
            switch (pK2UEvent_p->eventType)
            {
                case kEventTypeAsndRxInfo:
                    // Get the event argument from the copied data buffer
                    pK2UEvent_p->eventArg.pEventArg = (char*)pK2UEvent_p + sizeof(tEvent);
                    pFrameInfo = (tFrameInfo*)pK2UEvent_p->eventArg.pEventArg;
                    pKernelMemBuffer = (UINT8*)pFrameInfo->frame.pBuffer;

                    // Get the bus address for the data buffer
                    ret = drvintf_mapKernelMem((UINT8*)pKernelMemBuffer,
                                               (UINT8**)&pUserMemBuffer,
                                               (size_t)pFrameInfo->frameSize);
                    if (ret != kErrorOk)
                    {
                        return ret;
                    }

                    pFrameInfo->frame.pBuffer = (tPlkFrame*)pUserMemBuffer;

                    // Check if the frame is of non POWERLINK type
                    etherType = ami_getUint16Be(&pFrameInfo->frame.pBuffer->etherType);
                    if (etherType != C_DLL_ETHERTYPE_EPL)
                    {
                        ret = drvIntfInstance_l.pfnCbVeth(pFrameInfo);

                        // Indicate that this event is not to be posted to the user layer
                        *pSize_p = 0;

                        // Restore frame info for releasing Rx frame
                        pFrameInfo->frame.pBuffer = (tPlkFrame*)pKernelMemBuffer;

                        // Post the event to free the veth frame buffer
                        u2kEvent.eventSink = kEventSinkDllkCal;
                        u2kEvent.eventType = kEventTypeReleaseRxFrame;
                        u2kEvent.eventArgSize = sizeof(tFrameInfo);
                        u2kEvent.eventArg.pEventArg = pFrameInfo;

                        drvintf_postEvent(&u2kEvent);
                    }

                    // Unmap the memory mapped previosly
                    drvintf_unmapKernelMem(&pUserMemBuffer);

                    break;

                case kEventTypeAsndRx:
                    // Get the event argument from the copied data buffer
                    pK2UEvent_p->eventArg.pEventArg = (char*)pK2UEvent_p + sizeof(tEvent);
                    // Argument pointer is frame
                    frameInfo.frame.pBuffer = (tPlkFrame*)pK2UEvent_p->eventArg.pEventArg;
                    frameInfo.frameSize = pK2UEvent_p->eventArgSize;
                    pFrameInfo = &frameInfo;

                    // Check if the frame is of non POWERLINK type
                    etherType = ami_getUint16Be(&pFrameInfo->frame.pBuffer->etherType);
                    if (etherType != C_DLL_ETHERTYPE_EPL)
                    {
                        ret = drvIntfInstance_l.pfnCbVeth(pFrameInfo);

                        // Indicate that this event is not to be posted to the user layer
                        *pSize_p = 0;
                    }

                    break;

                default:
                    // Nothing to be done
                    break;
            }
        }
#endif
    }
    else
    {
        *pSize_p = 0;
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Get PDO memory

Retrieves the PDO memory address from the dualprocshm library and maps it into
user space before sharing it to user layer(host).

\param  ppPdoMem_p    Pointer to PDO memory.
\param  pMemSize_p    Pointer to the size of the PDO memory.

\return Returns tOplkError error code.

\ingroup module_driver_linux_kernel_pcie
*/
//------------------------------------------------------------------------------
tOplkError drvintf_getPdoMem(UINT8** ppPdoMem_p, size_t* pMemSize_p)
{
    tDualprocReturn     dualRet;
    UINT8*              pPdoMem = NULL;

    if (!drvIntfInstance_l.fDriverActive)
        return kErrorNoResource;

    if ((ppPdoMem_p == NULL) || (pMemSize_p == NULL))
        return kErrorInvalidInstanceParam;

    dualRet = dualprocshm_getMemory(drvIntfInstance_l.dualProcDrvInst,
                                    DUALPROCSHM_BUFF_ID_PDO, &pPdoMem,
                                    pMemSize_p, FALSE);

    if (dualRet != kDualprocSuccessful)
    {
        DEBUG_LVL_ERROR_TRACE("%s() couldn't allocate Pdo buffer (%d)\n",
                              __func__, dualRet);
        return kErrorNoResource;
    }

    *ppPdoMem_p = pPdoMem;
    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Free PDO memory

Frees the PDO memory previously allocated.

\param  ppPdoMem_p    Double pointer to PDO memory.
\param  memSize_p     Size of the PDO memory.

\return Returns tOplkError error code.

\ingroup module_driver_linux_kernel_pcie
*/
//------------------------------------------------------------------------------
tOplkError drvintf_freePdoMem(UINT8** ppPdoMem_p, size_t memSize_p)
{
    tDualprocReturn     dualRet;

    UNUSED_PARAMETER(memSize_p);

    if (!drvIntfInstance_l.fDriverActive)
        return kErrorNoResource;

    if (ppPdoMem_p == NULL)
        return kErrorInvalidInstanceParam;

    dualRet = dualprocshm_freeMemory(drvIntfInstance_l.dualProcDrvInst,
                                     DUALPROCSHM_BUFF_ID_PDO, FALSE);
    if (dualRet != kDualprocSuccessful)
    {
        DEBUG_LVL_ERROR_TRACE("%s() couldn't free PDO buffer (%d)\n",
                              __func__, dualRet);
        return kErrorNoResource;
    }

    *ppPdoMem_p = NULL;
    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Get Benchmark Base

Retrieves the benchmark memory from NDIS driver and maps it into user virtual
address space for accessing for user layer.
\note This function is not implemented for Linux PCIe design.

\param  ppBenchmarkMem_p    Double pointer to benchmark memory.

\return Returns tOplkError error code.

\ingroup module_driver_linux_kernel_pcie
*/
//------------------------------------------------------------------------------
tOplkError drvintf_getBenchmarkMem(UINT8** ppBenchmarkMem_p)
{
    UNUSED_PARAMETER(ppBenchmarkMem_p);
    return kErrorInvalidOperation;
}

//------------------------------------------------------------------------------
/**
\brief  Free Benchmark memory

Frees the benchmark memory previously allocated.
\note This function is not implemented for Linux PCIe design.

\param  ppBenchmarkMem_p    Double pointer to benchmark memory.

\return Returns tOplkError error code.

\ingroup module_driver_linux_kernel_pcie
*/
//------------------------------------------------------------------------------
tOplkError drvintf_freeBenchmarkMem(UINT8** ppBenchmarkMem_p)
{
    UNUSED_PARAMETER(ppBenchmarkMem_p);
    return kErrorInvalidOperation;
}

//------------------------------------------------------------------------------
/**
\brief  Map memory in openPOWERLINK kernel into user layer

Maps the kernel layer memory specified by the caller into user layer.

\param  pKernelMem_p     Pointer to kernel memory.
\param  ppUserMem_p      Double pointer to mapped kernel memory in user layer.
\param  size_p           Size of the memory to be mapped.

\return Returns tOplkError error code.

\ingroup module_driver_linux_kernel_pcie
*/
//------------------------------------------------------------------------------
tOplkError drvintf_mapKernelMem(UINT8* pKernelMem_p,
                                UINT8** ppUserMem_p, size_t size_p)
{
    tDualprocReturn             dualRet;
    tDualprocSharedMemInst      localProcSharedMemInst;
    tDualprocSharedMemInst      remoteProcSharedMemInst;

    if ((pKernelMem_p == NULL) || (ppUserMem_p == NULL) ||
        (!drvIntfInstance_l.fDriverActive))
        return kErrorNoResource;

    UNUSED_PARAMETER(size_p);

    if ((drvIntfInstance_l.shmMemLocal == 0) ||
        (drvIntfInstance_l.shmMemRemote == 0) ||
        (drvIntfInstance_l.shmSize == 0))
    {
        // Get the local processor's shared memory instance
        dualRet = dualprocshm_getSharedMemInfo(drvIntfInstance_l.dualProcDrvInst,
                                               dualprocshm_getLocalProcInst(),
                                               &localProcSharedMemInst);

        if ((dualRet != kDualprocSuccessful) ||
            (localProcSharedMemInst.baseAddr == (UINT64)0))
        {
            DEBUG_LVL_ERROR_TRACE("%s() Unable to map kernel memory error %x\n",
                                  __func__, dualRet);
            return kErrorNoResource;
        }

        // Get the remote processor's shared memory instance
        dualRet = dualprocshm_getSharedMemInfo(drvIntfInstance_l.dualProcDrvInst,
                                               dualprocshm_getRemoteProcInst(),
                                               &remoteProcSharedMemInst);

        if ((dualRet != kDualprocSuccessful) ||
            (remoteProcSharedMemInst.baseAddr == (UINT64)0))
        {
            DEBUG_LVL_ERROR_TRACE("%s() Unable to map kernel memory error %x\n",
                                  __func__, dualRet);
            return kErrorNoResource;
        }

        drvIntfInstance_l.shmMemLocal = (ULONG)localProcSharedMemInst.baseAddr;
        drvIntfInstance_l.shmMemRemote = (ULONG)remoteProcSharedMemInst.baseAddr;
        drvIntfInstance_l.shmSize = remoteProcSharedMemInst.span;
    }

    if ((ULONG)pKernelMem_p <= drvIntfInstance_l.shmMemRemote)
    {
        DEBUG_LVL_ERROR_TRACE("%s() Error: PCP buffer pointer lies outside the shared memory region\n");
        return kErrorInvalidInstanceParam;
    }
    else
    {
        *ppUserMem_p = (UINT8*)(((ULONG)pKernelMem_p - (ULONG)drvIntfInstance_l.shmMemRemote) +
                                (ULONG)drvIntfInstance_l.shmMemLocal);
        DEBUG_LVL_DRVINTF_TRACE("LA: 0x%lX, RA: 0x%lX, KA: 0x%lX, UA: 0x%lX\n",
                                drvIntfInstance_l.shmMemLocal,
                                drvIntfInstance_l.shmMemRemote,
                                (ULONG)pKernelMem_p, (ULONG)(*ppUserMem_p));
    }

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Unmap mapped memory

Unmap and free the kernel to user memory mapped before.

\param  ppUserMem_p    Double pointer to mapped user memory.

\ingroup module_driver_linux_kernel_pcie
*/
//------------------------------------------------------------------------------
void drvintf_unmapKernelMem(UINT8** ppUserMem_p)
{
    if (ppUserMem_p == NULL)
        return;

    *ppUserMem_p = NULL;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief  Initialize dual processor shared memory driver instance

This routine initializes the driver instance of dualprocshm for host processor.

\return Returns tOplkError error code.

*/
//------------------------------------------------------------------------------
static tOplkError initDualProcShmDriver(void)
{
    tDualprocReturn     dualRet;
    tDualprocConfig     dualProcConfig;
    INT                 loopCount = 0;

    OPLK_MEMSET(&dualProcConfig, 0, sizeof(tDualprocConfig));

    dualProcConfig.procInstance = kDualProcSecond;

    dualRet = dualprocshm_create(&dualProcConfig,
                                 &drvIntfInstance_l.dualProcDrvInst);
    if (dualRet != kDualprocSuccessful)
    {
        DEBUG_LVL_ERROR_TRACE(" %s(): Could not create dual processor driver instance (0x%X)\n",
                              __func__, dualRet);
        dualprocshm_delete(drvIntfInstance_l.dualProcDrvInst);
        return kErrorNoResource;
    }

    for (loopCount = 0; loopCount <= (DPSHM_ENABLE_TIMEOUT_SEC * 100); loopCount++)
    {
        msleep(10U);
        dualRet = dualprocshm_checkShmIntfState(drvIntfInstance_l.dualProcDrvInst);
        if (dualRet != kDualprocshmIntfDisabled)
            break;
    }

    if (dualRet != kDualprocshmIntfEnabled)
    {
        DEBUG_LVL_ERROR_TRACE("%s(): Dual processor interface is not enabled (0x%X)\n",
                              __func__, dualRet);
        return kErrorNoResource;
    }

    dualRet = dualprocshm_initInterrupts(drvIntfInstance_l.dualProcDrvInst);
    if (dualRet != kDualprocSuccessful)
    {
        DEBUG_LVL_ERROR_TRACE("{%s} Error Initializing interrupts %x\n ",
                              __func__, dualRet);
        return kErrorNoResource;
    }

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Delete dual processor shared memory driver instance

This routine deletes the driver instance of dualprocshm created during
initialization.

*/
//------------------------------------------------------------------------------
static void exitDualProcShmDriver(void)
{
    tDualprocReturn    dualRet;

    // disable system irq
    dualRet = dualprocshm_freeInterrupts(drvIntfInstance_l.dualProcDrvInst);
    if (dualRet != kDualprocSuccessful)
    {
        DEBUG_LVL_ERROR_TRACE("Could not free dual processor interrupts (0x%X)\n",
                              dualRet);
    }

    dualRet = dualprocshm_delete(drvIntfInstance_l.dualProcDrvInst);
    if (dualRet != kDualprocSuccessful)
    {
        DEBUG_LVL_ERROR_TRACE("Could not delete dual processor driver inst (0x%X)\n",
                              dualRet);
    }
}

//------------------------------------------------------------------------------
/**
\brief  Initialize the stack dualprocshm interface

Initializes the stack modules with the dualprocshm interface which would be
used to handle user calls via ioctl and forwarded to the PCP.

\return Returns tOplkError error code.

*/
//------------------------------------------------------------------------------
static tOplkError initStackInterface(void)
{
    tOplkError      ret = kErrorOk;

    ret = initEventInterface();
    if (ret != kErrorOk)
    {
        DEBUG_LVL_ERROR_TRACE("Event Initialization Failed (0x%X)\n", ret);
        return ret;
    }

    ret = initErrHandleInterface();
    if (ret != kErrorOk)
    {
        DEBUG_LVL_ERROR_TRACE("Error Module Initialization Failed (0x%X)\n", ret);
        return ret;
    }

    ret = initDllQueueInterface();
    if (ret != kErrorOk)
    {
        DEBUG_LVL_ERROR_TRACE("Dll Queues Initialization Failed (0x%X)\n", ret);
        return ret;
    }

    ret = timesynckcal_init();
    if (ret != kErrorOk)
    {
        DEBUG_LVL_ERROR_TRACE("Sync Initialization Failed %x\n", ret);
        return ret;
    }

#if defined(CONFIG_INCLUDE_VETH)
    // Mark the VEth interface initialization state as pending
    // This will be completed when the MAC address is read from the PCP
    drvIntfInstance_l.fVEthActive = FALSE;
#endif

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Initialize event queues

Initializes shared event queues between user and kernel layer of stack. The
memory for the queues allocated in PCIe memory by PCP, are retrieved using
dualprocshm library. The circular buffer library is used to manage the queues.

\return Returns tOplkError error code.

*/
//------------------------------------------------------------------------------
static tOplkError initEventInterface(void)
{
    tCircBufError    circError = kCircBufOk;

    if (!drvIntfInstance_l.fDriverActive)
        return kErrorNoResource;

    circError = circbuf_connect(CIRCBUF_USER_TO_KERNEL_QUEUE,
                                &drvIntfInstance_l.apEventQueueInst[kEventQueueU2K]);
    if (circError != kCircBufOk)
    {
        DEBUG_LVL_DRVINTF_TRACE("PLK : Could not allocate CIRCBUF_USER_TO_KERNEL_QUEUE circbuffer\n");
        return kErrorNoResource;
    }

    circError = circbuf_connect(CIRCBUF_KERNEL_TO_USER_QUEUE,
                                &drvIntfInstance_l.apEventQueueInst[kEventQueueK2U]);
    if (circError != kCircBufOk)
    {
        DEBUG_LVL_DRVINTF_TRACE("PLK : Could not allocate CIRCBUF_KERNEL_TO_USER_QUEUE circbuffer\n");
        return kErrorNoResource;
    }

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Close stack dualprocshm interface

This function closes the stack dualprocshm interface to PCP, initialized earlier.

*/
//------------------------------------------------------------------------------
static void exitStackInterface(void)
{
#if defined(CONFIG_INCLUDE_VETH)
    // Mark the VEth interface as inactive
    veth_exit();
    drvIntfInstance_l.fVEthActive = FALSE;
#endif
    exitDllQueueInterface();
    exitErrHandleInterface();
    exitEventInterface();
}

//------------------------------------------------------------------------------
/**
\brief  Close user layer event queues

Close event queues initialized earlier.

*/
//------------------------------------------------------------------------------
static void exitEventInterface(void)
{
    if (drvIntfInstance_l.apEventQueueInst[kEventQueueK2U] != NULL)
        circbuf_disconnect(drvIntfInstance_l.apEventQueueInst[kEventQueueK2U]);

    if (drvIntfInstance_l.apEventQueueInst[kEventQueueU2K] != NULL)
        circbuf_disconnect(drvIntfInstance_l.apEventQueueInst[kEventQueueU2K]);
}

//------------------------------------------------------------------------------
/**
\brief  Initialize user layer error handler memory

Retrieves the shared memory for the error handler module. This memory is only
accessible to user space through ioctl calls.

\return Returns tOplkError error code.

*/
//------------------------------------------------------------------------------
static tOplkError initErrHandleInterface(void)
{
    tDualprocReturn     dualRet;
    UINT8*              pBase;
    size_t              span;

    if (!drvIntfInstance_l.fDriverActive)
        return kErrorNoResource;

    if (drvIntfInstance_l.pErrorObjects != NULL)
        return kErrorInvalidOperation;

    dualRet = dualprocshm_getMemory(drvIntfInstance_l.dualProcDrvInst,
                                    DUALPROCSHM_BUFF_ID_ERRHDLR,
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

    drvIntfInstance_l.pErrorObjects = (tErrHndObjects*)pBase;

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Free error handler memory

*/
//------------------------------------------------------------------------------
static void exitErrHandleInterface(void)
{
    if (drvIntfInstance_l.pErrorObjects != NULL)
    {
        dualprocshm_freeMemory(drvIntfInstance_l.dualProcDrvInst,
                               DUALPROCSHM_BUFF_ID_ERRHDLR, FALSE);
        drvIntfInstance_l.pErrorObjects = NULL;
    }
}

//------------------------------------------------------------------------------
/**
\brief  Initialize DLL queues for user layer

This routine retrieves the DLL queues shared between user and kernel stack, for
user layer. The queues are located in PCIe memory and are accessed using
circular buffer and dualprocshm library.

\return Returns tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError initDllQueueInterface(void)
{
    tCircBufError    circError = kCircBufOk;

    if (!drvIntfInstance_l.fDriverActive)
        return kErrorNoResource;

    circError = circbuf_connect(CIRCBUF_DLLCAL_TXGEN,
                                &drvIntfInstance_l.apDllQueueInst[kDllCalQueueTxGen]);
    if (circError != kCircBufOk)
    {
        DEBUG_LVL_DRVINTF_TRACE("PLK : Could not allocate CIRCBUF_DLLCAL_TXGEN circbuffer\n");
        return kErrorNoResource;
    }

    circError = circbuf_connect(CIRCBUF_DLLCAL_TXNMT,
                                &drvIntfInstance_l.apDllQueueInst[kDllCalQueueTxNmt]);
    if (circError != kCircBufOk)
    {
        DEBUG_LVL_DRVINTF_TRACE("PLK : Could not allocate CIRCBUF_DLLCAL_TXNMT circbuffer\n");
        return kErrorNoResource;
    }

    circError = circbuf_connect(CIRCBUF_DLLCAL_TXSYNC,
                                &drvIntfInstance_l.apDllQueueInst[kDllCalQueueTxSync]);
    if (circError != kCircBufOk)
    {
        DEBUG_LVL_DRVINTF_TRACE("PLK : Could not allocate CIRCBUF_DLLCAL_TXSYNC circbuffer\n");
        return kErrorNoResource;
    }

#if defined(CONFIG_INCLUDE_VETH)
    circError = circbuf_connect(CIRCBUF_DLLCAL_TXVETH,
                                &drvIntfInstance_l.apDllQueueInst[kDllCalQueueTxVeth]);
    if (circError != kCircBufOk)
    {
        DEBUG_LVL_DRVINTF_TRACE("PLK : Could not allocate CIRCBUF_DLLCAL_TXVETH circbuffer\n");
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
static void exitDllQueueInterface(void)
{
    if (drvIntfInstance_l.apDllQueueInst[kDllCalQueueTxGen] != NULL)
        circbuf_disconnect(drvIntfInstance_l.apDllQueueInst[kDllCalQueueTxGen]);

    if (drvIntfInstance_l.apDllQueueInst[kDllCalQueueTxNmt] != NULL)
        circbuf_disconnect(drvIntfInstance_l.apDllQueueInst[kDllCalQueueTxNmt]);

    if (drvIntfInstance_l.apDllQueueInst[kDllCalQueueTxSync] != NULL)
        circbuf_disconnect(drvIntfInstance_l.apDllQueueInst[kDllCalQueueTxSync]);

#if defined(CONFIG_INCLUDE_VETH)
    if (drvIntfInstance_l.apDllQueueInst[kDllCalQueueTxVeth] != NULL)
        circbuf_disconnect(drvIntfInstance_l.apDllQueueInst[kDllCalQueueTxVeth]);
#endif
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
static tOplkError insertAsyncDataBlock(tCircBufInstance* pDllCircBuffInst_p,
                                       UINT8* pData_p, UINT* pDataSize_p)
{
    tOplkError          ret = kErrorOk;
    tCircBufError       error;

    if (!drvIntfInstance_l.fDriverActive)
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

/// \}
