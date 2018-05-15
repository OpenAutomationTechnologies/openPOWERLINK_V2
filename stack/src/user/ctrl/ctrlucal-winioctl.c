/**
********************************************************************************
\file   ctrlucal-winioctl.c

\brief  User control CAL module using IOCTL calls to Windows kernel

This file contains the implementation of user control CAL module which uses
IOCTLs to communicate with openPOWERLINK kernel layer.

The control module forwards all commands as IOCTL calls to driver in Windows
kernel. The driver either handles them in the kernel layer or forwards it
to the openPOWERLINK kernel layer running on an external PCIe card.

\ingroup module_ctrlucal
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2017, Kalycito Infotech Private Limited
Copyright (c) 2016, B&R Industrial Automation GmbH
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
#include <stddef.h>

#include <oplk/oplk.h>
#include <common/ctrl.h>
#include <common/ctrlcal.h>
#include <common/ctrlcal-mem.h>
#include <user/ctrlucal.h>
#include <common/target.h>

#include <common/driver.h>

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

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static OPLK_FILE_HANDLE hFileHandle_l;
static tMemStruc        sharedMemStruc_l;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Initialize user control CAL module

The function initializes the user control CAL module.

\return The function returns a tOplkError error code.

\ingroup module_ctrlucal
*/
//------------------------------------------------------------------------------
tOplkError ctrlucal_init(void)
{
    UINT32  errCode;

    hFileHandle_l = CreateFile(PLK_DEV_FILE,                         // Name of the NT "device" to open
                               GENERIC_READ | GENERIC_WRITE,         // Access rights requested
                               FILE_SHARE_READ | FILE_SHARE_WRITE,   // Share access - NONE
                               NULL,                                 // Security attributes - not used!
                               OPEN_EXISTING,                        // Device must exist to open it.
                               FILE_ATTRIBUTE_NORMAL,                // Open for overlapped I/O
                               NULL);                                // Extended attributes - not used!

    if (hFileHandle_l == INVALID_HANDLE_VALUE)
    {
        errCode = GetLastError();
        DEBUG_LVL_ERROR_TRACE("%s() CreateFile failed with error 0x%x\n",
                              __func__,
                              errCode);
        return kErrorNoResource;
    }

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Clean up user control CAL module

The function cleans up the user control CAL module.

\ingroup module_ctrlucal
*/
//------------------------------------------------------------------------------
void ctrlucal_exit(void)
{
    tMemStruc*  pMemStruc = &sharedMemStruc_l;
    ULONG       bytesReturned;

    if (!DeviceIoControl(hFileHandle_l,
                         PLK_CMD_UNMAP_MEM,
                         pMemStruc,
                         sizeof(tMemStruc),
                         NULL,
                         0,
                         &bytesReturned,
                         NULL))
    {
        DEBUG_LVL_ERROR_TRACE("%s() Unable to free mem %d\n",
                              __func__,
                              GetLastError());
        return;
    }

    sharedMemStruc_l.pKernelAddr = NULL;
    sharedMemStruc_l.pUserAddr = NULL;
    sharedMemStruc_l.size = 0;

    CloseHandle(hFileHandle_l);
}

//------------------------------------------------------------------------------
/**
\brief  Process user control CAL module

This function provides processing time for the CAL module.

\return The function returns a tOplkError error code.

\ingroup module_ctrlucal
*/
//------------------------------------------------------------------------------
tOplkError ctrlucal_process(void)
{
    // nothing to be processed
    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief    Execute a ctrl command

The function forwards a control command to the kernel stack.

\param[in]      cmd_p               Command to execute.
\param[out]     pRetVal_p           Return value from the control command.

\return The function returns a tOplkError error code.

\ingroup module_ctrlucal
*/
//------------------------------------------------------------------------------
tOplkError ctrlucal_executeCmd(tCtrlCmdType cmd_p,
                               UINT16* pRetVal_p)
{
    tCtrlCmd    ctrlCmd;
    tCtrlCmd    ctrlCmdRes;
    ULONG       bytesReturned;
    BOOL        fIoctlRet;

    // Check parameter validity
    ASSERT(pRetVal_p != NULL);

    ctrlCmd.cmd = cmd_p;
    ctrlCmd.retVal = 0;

    fIoctlRet = DeviceIoControl(hFileHandle_l,
                                PLK_CMD_CTRL_EXECUTE_CMD,
                                &ctrlCmd,
                                sizeof(tCtrlCmd),
                                &ctrlCmdRes,
                                sizeof(tCtrlCmd),
                                &bytesReturned,
                                NULL);
    if (!fIoctlRet || (bytesReturned == 0))
    {
        DEBUG_LVL_ERROR_TRACE("%s() Error in DeviceIoControl : %d\n",
                              __func__,
                              GetLastError());
        return kErrorGeneralError;
    }

    *pRetVal_p = ctrlCmdRes.retVal;
    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief Check state of kernel stack

The function checks the state of the kernel stack. If it is already running
it tries to shutdown.

\return The function returns a tOplkError error code.
\retval kErrorOk                    Kernel stack is initialized
\retval kErrorNoResource            Kernel stack is not running or in wrong state

\ingroup module_ctrlucal
*/
//------------------------------------------------------------------------------
tOplkError ctrlucal_checkKernelStack(void)
{
    UINT16      kernelStatus;
    tOplkError  ret;
    UINT16      retVal;

    DEBUG_LVL_CTRL_TRACE("Checking for kernel stack...\n");

    kernelStatus = ctrlucal_getStatus();
    switch (kernelStatus)
    {
        case kCtrlStatusReady:
            DEBUG_LVL_CTRL_TRACE("-> Kernel stack is ready\n");
            ret = kErrorOk;
            break;

        case kCtrlStatusRunning:
            /* try to shutdown kernel stack */
            DEBUG_LVL_CTRL_TRACE("-> Try to shutdown kernel stack\n");
            ret = ctrlucal_executeCmd(kCtrlCleanupStack, &retVal);
            if ((ret != kErrorOk) || ((tOplkError)retVal != kErrorOk))
            {
                ret = kErrorNoResource;
                break;
            }

            target_msleep(1000);

            kernelStatus = ctrlucal_getStatus();
            if (kernelStatus != kCtrlStatusReady)
                ret = kErrorNoResource;
            else
                ret = kErrorOk;
            break;

        default:
            ret = kErrorNoResource;
            break;
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief    Get status of kernel stack

The function gets the status of the kernel stack

\return The function returns the kernel status.

\ingroup module_ctrlucal
*/
//------------------------------------------------------------------------------
UINT16 ctrlucal_getStatus(void)
{
    UINT16  status;
    ULONG   bytesReturned;
    BOOL    fIoctlRet;

    fIoctlRet = DeviceIoControl(hFileHandle_l,
                                PLK_CMD_CTRL_GET_STATUS,
                                NULL,
                                0,
                                &status,
                                sizeof(UINT16),
                                &bytesReturned,
                                NULL);
    if (!fIoctlRet || (bytesReturned == 0))
    {
        DEBUG_LVL_ERROR_TRACE("%s() Error in DeviceIoControl : %d\n",
                              __func__,
                              GetLastError());
        return kCtrlStatusUnavailable;
    }

    return status;
}

//------------------------------------------------------------------------------
/**
\brief Get the heartbeat of the kernel stack

The function reads the heartbeat generated by the kernel stack.

\return The function returns the heartbeat counter.

\ingroup module_ctrlucal
*/
//------------------------------------------------------------------------------
UINT16 ctrlucal_getHeartbeat(void)
{
    UINT16  heartbeat;
    ULONG   bytesReturned;
    BOOL    fIoctlRet;

    fIoctlRet = DeviceIoControl(hFileHandle_l,
                                PLK_CMD_CTRL_GET_HEARTBEAT,
                                NULL,
                                0,
                                &heartbeat,
                                sizeof(UINT16),
                                &bytesReturned,
                                NULL);
    if (!fIoctlRet || (bytesReturned == 0))
    {
        DEBUG_LVL_ERROR_TRACE("%s() Error in DeviceIoControl : %d\n",
                              __func__,
                              GetLastError());
        return 0;
    }

    return heartbeat;
}

//------------------------------------------------------------------------------
/**
\brief  Store the init parameters for kernel use

The function stores the openPOWERLINK initialization parameter so that they
can be accessed by the kernel stack.

\param[in]      pInitParam_p        Specifies where to read the init parameters.

\ingroup module_ctrlucal
*/
//------------------------------------------------------------------------------
void ctrlucal_storeInitParam(const tCtrlInitParam* pInitParam_p)
{
    ULONG   bytesReturned;
    BOOL    fIoctlRet;

    // Check parameter validity
    ASSERT(pInitParam_p != NULL);

    fIoctlRet = DeviceIoControl(hFileHandle_l,
                                PLK_CMD_CTRL_STORE_INITPARAM,
                                (LPVOID)pInitParam_p,
                                sizeof(tCtrlInitParam),
                                NULL,
                                0,
                                &bytesReturned,
                                NULL);
    if (!fIoctlRet || (bytesReturned == 0))
    {
        DEBUG_LVL_ERROR_TRACE("%s() Error in DeviceIoControl : %d\n",
                              __func__,
                              GetLastError());
    }
}

//------------------------------------------------------------------------------
/**
\brief  Read the init parameters from kernel

The function reads the initialization parameter from the kernel stack.

\param[out]     pInitParam_p        Specifies where to store the read init parameters.

\return The function returns a tOplkError error code.

\ingroup module_ctrlucal
*/
//------------------------------------------------------------------------------
tOplkError ctrlucal_readInitParam(tCtrlInitParam* pInitParam_p)
{
    ULONG   bytesReturned;
    BOOL    fIoctlRet;

    // Check parameter validity
    ASSERT(pInitParam_p != NULL);

    fIoctlRet = DeviceIoControl(hFileHandle_l,
                                PLK_CMD_CTRL_READ_INITPARAM,
                                NULL,
                                0,
                                pInitParam_p,
                                sizeof(tCtrlInitParam),
                                &bytesReturned,
                                NULL);
    if (!fIoctlRet || (bytesReturned == 0))
    {
        DEBUG_LVL_ERROR_TRACE("%s() Error in DeviceIoControl : %d\n",
                              __func__,
                              GetLastError());
        return kErrorGeneralError;
    }

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Write file chunk

This function writes the given file chunk to the file transfer buffer

\param[in]      pDesc_p             Descriptor for the file chunk.
\param[in]      pBuffer_p           Buffer holding the file chunk.

\return The function returns a tOplkError error code.

\ingroup module_ctrlucal
*/
//------------------------------------------------------------------------------
tOplkError ctrlucal_writeFileBuffer(const tOplkApiFileChunkDesc* pDesc_p,
                                    const void* pBuffer_p)
{
    ULONG               bytesReturned;
    BOOL                fIoctlRet;
    tIoctlFileChunk*    pIoctlFileChunk = NULL;
    UINT32              ioctlBufferSize;

    // Check parameter validity
    ASSERT(pDesc_p != NULL);
    ASSERT(pBuffer_p != NULL);

    ioctlBufferSize = (UINT32)(sizeof(tIoctlFileChunk) + pDesc_p->length);
    pIoctlFileChunk = (tIoctlFileChunk*)OPLK_MALLOC(ioctlBufferSize);
    if (pIoctlFileChunk == NULL)
        return kErrorNoResource;

    OPLK_MEMCPY(&pIoctlFileChunk->desc, pDesc_p, sizeof(tOplkApiFileChunkDesc));
    OPLK_MEMCPY(&pIoctlFileChunk->pData, pBuffer_p, pDesc_p->length);

    fIoctlRet = DeviceIoControl(hFileHandle_l,
                                PLK_CMD_CTRL_WRITE_FILE_BUFFER,
                                pIoctlFileChunk,
                                ioctlBufferSize,
                                NULL,
                                0,
                                &bytesReturned,
                                NULL);
    if (!fIoctlRet || (bytesReturned == 0))
    {
        DEBUG_LVL_ERROR_TRACE("%s() Error in DeviceIoControl : %d\n",
                              __func__,
                              GetLastError());
        OPLK_FREE(pIoctlFileChunk);
        return kErrorGeneralError;
    }

    OPLK_FREE(pIoctlFileChunk);

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Get maximum supported file chunk size

This function returns the maximum file chunk size which is supported by the
CAL implementation.

\return The function returns the supported file chunk size.

\ingroup module_ctrlucal
*/
//------------------------------------------------------------------------------
size_t ctrlucal_getFileBufferSize(void)
{
    BOOL    fIoctlRet;
    ULONG   bytesReturned;
    size_t  fileBufferSize;

    fIoctlRet = DeviceIoControl(hFileHandle_l,
                                PLK_CMD_CTRL_GET_FILE_BUFFER_SIZE,
                                NULL,
                                0,
                                &fileBufferSize,
                                sizeof(size_t),
                                &bytesReturned,
                                NULL);

    if (!fIoctlRet || (bytesReturned == 0))
    {
        DEBUG_LVL_ERROR_TRACE("%s() Error in DeviceIoControl : %d\n",
                              __func__,
                              GetLastError());
        return 0;
    }

    return fileBufferSize;
}

//------------------------------------------------------------------------------
/**
\brief  Return the file descriptor of the kernel module

The function returns the file descriptor of the kernel module.

\return The function returns the file descriptor.

\ingroup module_ctrlucal
*/
//------------------------------------------------------------------------------
OPLK_FILE_HANDLE ctrlucal_getFd(void)
{
    return hFileHandle_l;
}

//------------------------------------------------------------------------------
/**
\brief  Get user memory

The routine calculates the base address of the memory in user space using
the provided offset and returns the address back.

\param[in]      kernelOffs_p        Offset of the memory in kernel.
\param[in]      size_p              Size of the memory.
\param[out]     ppUserMem_p         Pointer to the user memory.

\return The function returns a tOplkError error code.
\retval kErrorOk                    The memory was successfully returned.
\retval kErrorNoResource            No memory available.
\retval kErrorInvalidOperation      The provided offset is incorrect.

\ingroup module_ctrlucal
*/
//------------------------------------------------------------------------------
tOplkError ctrlucal_getMappedMem(size_t kernelOffs_p,
                                 size_t size_p,
                                 void** ppUserMem_p)
{
    tMemStruc   inMemStruc;
    tMemStruc*  pOutMemStruc = &sharedMemStruc_l;
    ULONG       bytesReturned;
    UINT8*      pUserMemHighAddr = NULL;
    UINT8*      pMem = NULL;

    // Check parameter validity
    ASSERT(ppUserMem_p != NULL);

    inMemStruc.size = size_p;

    if (!DeviceIoControl(hFileHandle_l,
                         PLK_CMD_MAP_MEM,
                         &inMemStruc,
                         sizeof(tMemStruc),
                         pOutMemStruc,
                         sizeof(tMemStruc),
                         &bytesReturned,
                         NULL))
    {
        DEBUG_LVL_ERROR_TRACE("%s() Failed to get mapped memory. Error 0x%x\n",
                              __func__,
                              GetLastError());
        return kErrorNoResource;
    }

    pUserMemHighAddr = (UINT8*)sharedMemStruc_l.pUserAddr +
                               sharedMemStruc_l.size;

    if (sharedMemStruc_l.pUserAddr == NULL)
        return kErrorNoResource;

    pMem = (UINT8*)sharedMemStruc_l.pUserAddr + kernelOffs_p;

    if ((pMem + size_p) > pUserMemHighAddr)
        return kErrorInvalidOperation;

    *ppUserMem_p = pMem;

    return kErrorOk;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

/// \}
