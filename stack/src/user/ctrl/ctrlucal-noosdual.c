/**
********************************************************************************
\file   ctrlucal-noosdual.c

\brief  User control CAL module using a dual processor shared memory library

This file contains an implementation of the user control CAL module which uses
a shared memory block for communication with the kernel layer.

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
#include <unistd.h>

#include <oplk/oplk.h>
#include <common/ctrl.h>
#include <common/ctrlcal.h>
#include <user/ctrlucal.h>
#include <common/target.h>
#include <common/ctrlcal-mem.h>

#include <dualprocshm.h>

//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#define CMD_TIMEOUT_SEC                 20          // command timeout in seconds
#define CMD_TIMEOUT_LOOP_MSEC           10          // wait for dpshm interface enable time out
#define CMD_TIMEOUT_LOOP_COUNT          (CMD_TIMEOUT_SEC * 1000U / CMD_TIMEOUT_LOOP_MSEC)     // loop count value

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
\brief Control module instance - User Layer

The control module instance stores the local parameters used by the
control CAL module during runtime
*/
typedef struct
{
    tDualprocDrvInstance    dualProcDrvInst;    ///< Dual processor driver instance
}tCtrluCalInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tCtrluCalInstance    instance_l;

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
    tDualprocReturn dualRet;
    tDualprocConfig dualProcConfig;
    INT             loopCount = 0;

    OPLK_MEMSET(&instance_l, 0, sizeof(tCtrluCalInstance));
    OPLK_MEMSET(&dualProcConfig, 0, sizeof(tDualprocConfig));

    dualProcConfig.procInstance = kDualProcSecond;

    dualRet = dualprocshm_create(&dualProcConfig, &instance_l.dualProcDrvInst);
    if (dualRet != kDualprocSuccessful)
    {
        DEBUG_LVL_ERROR_TRACE("%s Could not create dual processor driver instance (0x%X)\n",
                              __func__,
                              dualRet);
        dualprocshm_delete(instance_l.dualProcDrvInst);
        return kErrorNoResource;
    }

    for (loopCount = 0; loopCount <= CMD_TIMEOUT_LOOP_MSEC; loopCount++)
    {
        target_msleep(1000U);
        dualRet = dualprocshm_checkShmIntfState(instance_l.dualProcDrvInst);
        if (dualRet != kDualprocshmIntfDisabled)
            break;
    }

    if (dualRet != kDualprocshmIntfEnabled)
    {
        DEBUG_LVL_ERROR_TRACE("%s dualprocshm  interface is not enabled (0x%X)\n",
                              __func__,
                              dualRet);
        return kErrorNoResource;
    }

    dualRet = dualprocshm_initInterrupts(instance_l.dualProcDrvInst);
    if (dualRet != kDualprocSuccessful)
    {
        DEBUG_LVL_ERROR_TRACE("%s Error Initializing interrupts %x\n ",
                              __func__,
                              dualRet);
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
    tDualprocReturn dualRet;

    // disable system IRQ
    dualprocshm_freeInterrupts(instance_l.dualProcDrvInst);

    dualRet = dualprocshm_delete(instance_l.dualProcDrvInst);
    if (dualRet != kDualprocSuccessful)
    {
        DEBUG_LVL_ERROR_TRACE("Could not delete dual proc driver instance (0x%X)\n",
                              dualRet);
    }
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
    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief    Execute a ctrl command

The function executes a control command in the kernel stack.

\param[in]      cmd_p               Command to execute.
\param[out]     pRetVal_p           Return value from the control command.

\return The function returns a tOplkError error code.

\ingroup module_ctrlucal
*/
//------------------------------------------------------------------------------
tOplkError ctrlucal_executeCmd(tCtrlCmdType cmd_p,
                               UINT16* pRetVal_p)
{
    tDualprocReturn dualRet;
    tCtrlCmd        ctrlCmd;
    UINT32          timeout;

    // Check parameter validity
    ASSERT(pRetVal_p != NULL);

    // write command into shared buffer
    ctrlCmd.cmd = cmd_p;
    ctrlCmd.retVal = 0;

    dualRet = dualprocshm_writeDataCommon(instance_l.dualProcDrvInst,
                                          offsetof(tCtrlBuf, ctrlCmd),
                                          sizeof(tCtrlCmd),
                                          &ctrlCmd);
    if (dualRet != kDualprocSuccessful)
        return kErrorGeneralError;

    // wait for response
    for (timeout = 0; timeout < CMD_TIMEOUT_LOOP_COUNT; timeout++)
    {
        target_msleep(CMD_TIMEOUT_LOOP_MSEC);

        dualRet = dualprocshm_readDataCommon(instance_l.dualProcDrvInst,
                                             offsetof(tCtrlBuf, ctrlCmd),
                                             sizeof(tCtrlCmd),
                                             &ctrlCmd);
        if (dualRet != kDualprocSuccessful)
            return kErrorGeneralError;

        if (ctrlCmd.cmd == 0)
        {
            *pRetVal_p = ctrlCmd.retVal;
            return kErrorOk;
        }
    }

    DEBUG_LVL_ERROR_TRACE("%s() Timeout waiting for return!\n", __func__);
    return kErrorGeneralError;
}

//------------------------------------------------------------------------------
/**
\brief Check state of kernel stack

The function checks the state of the kernel stack. If it is already running
it tries to shutdown.

\return The function returns a tOplkError error code.
\retval kErrorOk                    Kernel stack is initialized
\retval kErrorNoResource            Kernel stack is not running or in wrong state
\retval kErrorGeneralError          Can't read data from memory

\ingroup module_ctrlucal
*/
//------------------------------------------------------------------------------
tOplkError ctrlucal_checkKernelStack(void)
{
    tDualprocReturn     dualRet;
    tCtrlKernelStatus   kernelStatus;
    tOplkError          ret;
    UINT16              magic;
    UINT16              retVal;
    UINT32              timeout = 0;
    BOOL                fExit = FALSE;

    DEBUG_LVL_CTRL_TRACE("Checking for kernel stack...\n");

    dualRet = dualprocshm_readDataCommon(instance_l.dualProcDrvInst,
                                         offsetof(tCtrlBuf, magic),
                                         sizeof(magic),
                                         &magic);
    if (dualRet != kDualprocSuccessful)
        return kErrorGeneralError;

    if (magic != CTRL_MAGIC)
    {
        DEBUG_LVL_CTRL_TRACE("Kernel daemon not running! Exiting...\n");
        return kErrorNoResource;
    }

    while (!fExit)
    {
        switch (kernelStatus = ctrlucal_getStatus())
        {
            case kCtrlStatusReady:
                DEBUG_LVL_CTRL_TRACE("-> Kernel stack is ready\n");
                fExit = TRUE;
                ret = kErrorOk;
                break;

            case kCtrlStatusRunning:
                /* try to shutdown kernel stack */
                DEBUG_LVL_CTRL_TRACE("-> Try to shutdown kernel stack\n");
                ret = ctrlucal_executeCmd(kCtrlCleanupStack, &retVal);
                if ((ret != kErrorOk) || ((tOplkError)retVal != kErrorOk))
                {
                    fExit = TRUE;
                    ret = kErrorNoResource;
                    break;
                }
                break;

            default:
                if (timeout == 0)
                {
                    DEBUG_LVL_CTRL_TRACE("-> Wait for kernel stack\n");
                }

                target_msleep(1000U);

                if (timeout++ >= CMD_TIMEOUT_SEC)
                {
                    fExit = TRUE;
                    ret = kErrorNoResource;
                }
                break;
        }
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
tCtrlKernelStatus ctrlucal_getStatus(void)
{
    tDualprocReturn     dualRet;
    tCtrlKernelStatus   status;

    dualRet = dualprocshm_readDataCommon(instance_l.dualProcDrvInst,
                                         offsetof(tCtrlBuf, status),
                                         sizeof(status),
                                         &status);
    if (dualRet == kDualprocSuccessful)
        return status;
    else
        return kCtrlStatusUnavailable;
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
    tDualprocReturn dualRet;
    UINT16          heartbeat;

    dualRet = dualprocshm_readDataCommon(instance_l.dualProcDrvInst,
                                         offsetof(tCtrlBuf, heartbeat),
                                         sizeof(heartbeat),
                                         &heartbeat);
    if (dualRet == kDualprocSuccessful)
        return heartbeat;
    else
        return 0;
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
    // Check parameter validity
    ASSERT(pInitParam_p != NULL);

    dualprocshm_writeDataCommon(instance_l.dualProcDrvInst,
                                offsetof(tCtrlBuf, initParam),
                                sizeof(tCtrlInitParam),
                                pInitParam_p);
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
    tDualprocReturn dualRet;

    // Check parameter validity
    ASSERT(pInitParam_p != NULL);

    dualRet = dualprocshm_readDataCommon(instance_l.dualProcDrvInst,
                                         offsetof(tCtrlBuf, initParam),
                                         sizeof(tCtrlInitParam),
                                         pInitParam_p);

    if (dualRet != kDualprocSuccessful)
    {
        DEBUG_LVL_ERROR_TRACE("Cannot read initparam (0x%X)\n", dualRet);
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
    UNUSED_PARAMETER(pDesc_p);
    UNUSED_PARAMETER(pBuffer_p);

    // This CAL is not supporting that feature -> return no resource available.
    return kErrorNoResource;
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
    // This CAL is not supporting that feature -> return zero size.
    return 0;
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
    return 0;
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
    UNUSED_PARAMETER(kernelOffs_p);
    UNUSED_PARAMETER(size_p);
    UNUSED_PARAMETER(ppUserMem_p);

    return kErrorNoResource;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

/// \}
