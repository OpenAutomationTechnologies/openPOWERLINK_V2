/**
********************************************************************************
\file   drvintf.c

\brief  Interface module for application interface to kernel daemon in Windows

This module handles all the application request forwarded to the daemon
in Windows kernel. It parses the data from the application and copies into
suitable structure before forwarding to a specific kernel stack module.

\ingroup module_driver_ndisim
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

#include <common/driver.h>
#include <common/ctrl.h>
#include <common/ctrlcal-mem.h>
#include <kernel/ctrlk.h>
#include <kernel/ctrlkcal.h>
#include <kernel/dllkcal.h>
#include <kernel/pdokcal.h>

#include <kernel/eventk.h>
#include <kernel/eventkcal.h>
#include <errhndkcal.h>

#include <drvintf.h>

//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// module global vars
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// global function prototypes
//------------------------------------------------------------------------------

//============================================================================//
//            P R I V A T E   D E F I N I T I O N S                           //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

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

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tMemInfo    pdoMemInfo_l;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Initialize driver interface

This function initializes necessary resources required for driver interface.

\return Returns tOplkError error code.

\ingroup module_driver_ndisim
*/
//------------------------------------------------------------------------------
tOplkError drv_init(void)
{
    tOplkError ret = kErrorOk;

    TRACE("Initialize driver interface...");

    TRACE(" OK\n");

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Close the driver interface

This function frees all the resources used by the driver interface and shuts down
the interface.

\ingroup module_driver_ndisim
*/
//------------------------------------------------------------------------------
void drv_exit(void)
{
    TRACE("Exit driver interface...\n");
}

//------------------------------------------------------------------------------
/**
\brief  Execute a control command from user application

This function parse the control command from user and passes it to the kernel
control module for processing. The return value is again passed to user by copying it
into the common control structure.

\param  pCtrlCmd_p       Pointer to control command structure.

\return Returns tOplkError error code.

\ingroup module_driver_ndispcie
*/
//------------------------------------------------------------------------------
tOplkError drv_executeCmd(tCtrlCmd* pCtrlCmd_p)
{
    tOplkError    oplkRet = kErrorOk;
    UINT16        retVal;
    UINT16        status;

    if (pCtrlCmd_p == NULL)
        return kErrorInvalidOperation;

    oplkRet = ctrlk_executeCmd(pCtrlCmd_p->cmd, &retVal, &status, NULL);
    if (oplkRet != kErrorOk)
    {
        pCtrlCmd_p->cmd = 0;
        pCtrlCmd_p->retVal = (UINT16)oplkRet;
        return oplkRet;
    }

    pCtrlCmd_p->cmd = 0;
    pCtrlCmd_p->retVal = retVal;

    ctrlkcal_setStatus(status);

    return oplkRet;
}

//------------------------------------------------------------------------------
/**
\brief  Read initialization parameters

Read the initialization parameters from the kernel stack.

\param  pInitParam_p       Pointer to initialization parameters structure.

\ingroup module_driver_ndisim
*/
//------------------------------------------------------------------------------
tOplkError drv_readInitParam(tCtrlInitParam* pInitParam_p)
{
    if (pInitParam_p == NULL)
        return kErrorInvalidOperation;

    return ctrlkcal_readInitParam(pInitParam_p);
}

//------------------------------------------------------------------------------
/**
\brief  Write initialization parameters

Write the initialization parameters from the user layer into kernel memory.

\param  pInitParam_p       Pointer to initialization parameters structure.

\ingroup module_driver_ndisim
*/
//------------------------------------------------------------------------------
tOplkError drv_storeInitParam(tCtrlInitParam* pInitParam_p)
{
    if (pInitParam_p == NULL)
        return kErrorInvalidOperation;

    ctrlkcal_storeInitParam(pInitParam_p);

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Get kernel status

Return the current status of kernel stack.

\param  pStatus_p       Pointer to status variable to return.

\ingroup module_driver_ndisim
*/
//------------------------------------------------------------------------------
tOplkError drv_getStatus(UINT16* pStatus_p)
{
    if (pStatus_p == NULL)
        return kErrorInvalidOperation;

    *pStatus_p = ctrlkcal_getStatus();

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Get heartbeat

Return the current heartbeat value in kernel.

\param  pHeartbeat_p       Pointer to heartbeat variable to return.

\ingroup module_driver_ndisim
*/
//------------------------------------------------------------------------------
tOplkError drv_getHeartbeat(UINT16* pHeartbeat_p)
{
    if (pHeartbeat_p == NULL)
        return kErrorInvalidOperation;

    *pHeartbeat_p = ctrlk_getHeartbeat();

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Write asynchronous frame

This routines extracts the asynchronous frame from the IOCTL buffer and passes it
to DLL module for processing.

\param  pArg_p       Pointer to IOCTL buffer.

\ingroup module_driver_ndisim
*/
//------------------------------------------------------------------------------
tOplkError drv_sendAsyncFrame(UINT8* pArg_p)
{
    tIoctlDllCalAsync*    asyncFrameInfo = NULL;
    tFrameInfo            frameInfo;

    if (pArg_p == NULL)
        return kErrorInvalidOperation;

    asyncFrameInfo = (tIoctlDllCalAsync*)pArg_p;
    frameInfo.frameSize = asyncFrameInfo->size;
    frameInfo.frame.pBuffer = (tPlkFrame*)(pArg_p + sizeof(tIoctlDllCalAsync));

    return dllkcal_writeAsyncFrame(&frameInfo, asyncFrameInfo->queue);
}

//------------------------------------------------------------------------------
/**
\brief  Write error object

This routines updates the error objects in kernel with the value passed from
user layer.

\param  pWriteObject_p       Pointer to write-object to update.

\ingroup module_driver_ndisim
*/
//------------------------------------------------------------------------------
tOplkError drv_writeErrorObject(tErrHndIoctl* pWriteObject_p)
{
    tErrHndObjects*   errorObjects = NULL;

    if (pWriteObject_p == NULL)
        return kErrorInvalidOperation;

    errorObjects = errhndkcal_getMemPtr();
    *((UINT32*)((UINT8*)errorObjects + pWriteObject_p->offset)) = pWriteObject_p->errVal;

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Read error object

This routines fetches the error objects in kernel to be passed to user layer.

\param  pWriteObject_p       Pointer to pReadObject_p to fetch.

\ingroup module_driver_ndisim
*/
//------------------------------------------------------------------------------
tOplkError drv_readErrorObject(tErrHndIoctl* pReadObject_p)
{
    tErrHndObjects*   errorObjects = NULL;

    if (pReadObject_p == NULL)
        return kErrorInvalidOperation;

    errorObjects = errhndkcal_getMemPtr();
    pReadObject_p->errVal = *((UINT8*)errorObjects + pReadObject_p->offset);

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Map PDO shared memory

This routine maps the PDO memory allocated in the kernel layer of the openPOWERLINK
stack. This allows user stack to access the PDO memory directly.

\param  ppKernelMem_p           Double pointer to the shared memory segment in kernel space.
\param  ppUserMem_p             Double pointer to the shared memory segment in user space.
\param  memSize_p               Pointer to size of PDO memory.

\return The function returns a tOplkError error code.

\ingroup module_driver_ndisim
*/
//------------------------------------------------------------------------------
tOplkError drv_mapPdoMem(UINT8** ppKernelMem_p, UINT8** ppUserMem_p,
                         UINT32* pMemSize_p)
{
    tOplkError      ret;

    // Get PDO memory
    ret = pdokcal_getPdoMemRegion((UINT8**)&pdoMemInfo_l.pKernelVa,
                                  &pdoMemInfo_l.memSize);

    if (ret != kErrorOk || pdoMemInfo_l.pKernelVa == NULL)
        return kErrorNoResource;

    if (*pMemSize_p > pdoMemInfo_l.memSize)
    {
        DEBUG_LVL_ERROR_TRACE("%s() Higher Memory requested (Kernel-%d User-%d) !\n",
                              __func__, pdoMemInfo_l.memSize, *pMemSize_p);
        *pMemSize_p = 0;
        return kErrorNoResource;
    }

    // Allocate new MDL pointing to PDO memory
    pdoMemInfo_l.pMdl = IoAllocateMdl(pdoMemInfo_l.pKernelVa, pdoMemInfo_l.memSize, FALSE, FALSE,
                                      NULL);

    if (pdoMemInfo_l.pMdl == NULL)
    {
        DEBUG_LVL_ERROR_TRACE("%s() Error allocating MDL !\n", __func__);
        return kErrorNoResource;
    }

    // Update the MDL with physical addresses
    MmBuildMdlForNonPagedPool(pdoMemInfo_l.pMdl);

    // Map the memory in user space and get the address
    pdoMemInfo_l.pUserVa = MmMapLockedPagesSpecifyCache(pdoMemInfo_l.pMdl,    // MDL
                                                        UserMode,             // Mode
                                                        MmCached,             // Caching
                                                        NULL,                 // Address
                                                        FALSE,                // Bug-check?
                                                        NormalPagePriority);  // Priority

    if (pdoMemInfo_l.pUserVa == NULL)
    {
        MmUnmapLockedPages(pdoMemInfo_l.pUserVa, pdoMemInfo_l.pMdl);
        IoFreeMdl(pdoMemInfo_l.pMdl);
        DEBUG_LVL_ERROR_TRACE("%s() Error mapping MDL !\n", __func__);
        return kErrorNoResource;
    }

    *ppKernelMem_p = pdoMemInfo_l.pKernelVa;
    *ppUserMem_p = pdoMemInfo_l.pUserVa;
    *pMemSize_p = pdoMemInfo_l.memSize;

    TRACE("Mapped memory info U:%p K:%p size %x", pdoMemInfo_l.pUserVa,
                                                 (UINT8*)pdoMemInfo_l.pKernelVa,
                                                 pdoMemInfo_l.memSize);
    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Unmap PDO shared memory

Unmap the PDO memory shared with the user layer. The memory will be freed in
pdokcal_freeMem().

\param  pMem_p                  Pointer to the shared memory segment.
\param  memSize_p               Size of PDO memory.

\ingroup module_pdokcal
*/
//------------------------------------------------------------------------------
void drv_unMapPdoMem(UINT8* pMem_p, size_t memSize_p)
{
    UNUSED_PARAMETER(memSize_p);
    UNUSED_PARAMETER(pMem_p);

    if (pdoMemInfo_l.pMdl == NULL)
    {
        DEBUG_LVL_ERROR_TRACE("%s() MDL already deleted !\n", __func__);
        return;
    }

    if (pdoMemInfo_l.pUserVa != NULL)
    {
        MmUnmapLockedPages(pdoMemInfo_l.pUserVa, pdoMemInfo_l.pMdl);
        IoFreeMdl(pdoMemInfo_l.pMdl);
    }

    pdoMemInfo_l.pUserVa = NULL;
}


//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

/// \}
