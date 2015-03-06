/**
********************************************************************************
\file   ctrlkcal-noosdual.c

\brief  Kernel control CAL module using the dual processor shared memory library

This file contains an implementation of the kernel CAL control module for non-OS
systems running on two processors.
The implementation uses the dual processor shared memory for communication with
the user layer. It also provides support for interrupt generation
for different causes such as events, data exchange, errors etc.

\ingroup module_ctrlkcal
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2014, Kalycito Infotech Private Limited
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
#include <common/ctrl.h>
#include <common/ctrlcal.h>
#include <common/ctrlcal-mem.h>
#include <kernel/ctrlkcal.h>

#include <dualprocshm.h>

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
#define CTRL_PROC_ID    0xFB                    ///< Processor Id for kernel layer

//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------
/**
\brief Control module instance - Kernel Layer

The control module instance stores the local parameters used by the
control CAL module during runtime
*/
typedef struct
{
    tDualprocDrvInstance    dualProcDrvInst;    ///< Dual processor driver instance
}tCtrlkCalInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tCtrlkCalInstance    instance_l;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Initialize kernel control CAL module

The function initializes the kernel control CAL module. It initializes the
control memory block and the underlying CAL module used for implementing
the memory block access functions.

\return The function returns a tOplkError error code.

\ingroup module_ctrlkcal
*/
//------------------------------------------------------------------------------
tOplkError ctrlkcal_init(void)
{
    tOplkError          ret = kErrorOk;
    tDualprocReturn     dualRet;
    tDualprocConfig     dualProcConfig;
    UINT16              magic;

    OPLK_MEMSET(&instance_l, 0, sizeof(tCtrlkCalInstance));

    OPLK_MEMSET(&dualProcConfig, 0, sizeof(tDualprocConfig));

    dualProcConfig.procInstance = kDualProcFirst;
    dualProcConfig.procId = CTRL_PROC_ID;

    dualRet = dualprocshm_create(&dualProcConfig, &instance_l.dualProcDrvInst);
    if (dualRet != kDualprocSuccessful)
    {
        DEBUG_LVL_ERROR_TRACE(" {%s} Could not create dual processor driver instance (0x%X)\n",
                              __func__, dualRet);
        ret = kErrorNoResource;

        // Since dualprocshm_delete() is already called in dualprocshm_create()
        // invalidate driver instance in local instance avoiding recalling
        // dualprocshm_delete().
        instance_l.dualProcDrvInst = NULL;
        goto Exit;
    }

    dualRet = dualprocshm_enableBridge(instance_l.dualProcDrvInst);
    if (dualRet != kDualprocSuccessful)
    {
        DEBUG_LVL_ERROR_TRACE("{%s} Error Enabling Bridge %x\n ", __func__, dualRet);
        ret = kErrorNoResource;
        goto Exit;
    }

    dualRet = dualprocshm_initInterrupts(instance_l.dualProcDrvInst);
    if (dualRet != kDualprocSuccessful)
    {
        DEBUG_LVL_ERROR_TRACE("{%s} Error Initializing interrupts %x\n ", __func__, dualRet);
        ret = kErrorNoResource;
        goto Exit;
    }

    magic = CTRL_MAGIC;
    dualRet = dualprocshm_writeDataCommon(instance_l.dualProcDrvInst, offsetof(tCtrlBuf, magic),
                                          sizeof(magic), (UINT8*)&magic);
    if (dualRet != kDualprocSuccessful)
    {
        DEBUG_LVL_ERROR_TRACE(" {%s} Could not create write magic (0x%X)\n",
                              __func__, dualRet);
        ret = kErrorNoResource;
        goto Exit;
    }

    ctrlkcal_setStatus(kCtrlStatusReady);

Exit:
    if ((ret != kErrorOk) && (instance_l.dualProcDrvInst != NULL))
        dualprocshm_delete(instance_l.dualProcDrvInst);

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Clean up kernel control CAL module

The function cleans up the kernel control CAL module. It resets the control
memory block and cleans up the underlying CAL module used for implementing
the memory block access functions.

\ingroup module_ctrlkcal
*/
//------------------------------------------------------------------------------
void ctrlkcal_exit(void)
{
    tDualprocReturn    dualRet;

    dualRet = dualprocshm_delete(instance_l.dualProcDrvInst);
    if (dualRet != kDualprocSuccessful)
    {
        DEBUG_LVL_ERROR_TRACE("Could not delete dual processor driver (0x%X)\n", dualRet);
    }
}

//------------------------------------------------------------------------------
/**
\brief  Process kernel control CAL module

This function provides processing time for the CAL module.

\return The function returns a tOplkError error code. The function always returns
        kErrorOk

\ingroup module_ctrlkcal
*/
//------------------------------------------------------------------------------
tOplkError ctrlkcal_process(void)
{
    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Get control command

The function reads a control command stored by the user in the control memory
block to execute a kernel control function.

\param  pCmd_p            The command to be executed.

\return The function returns a tOplkError error code.

\ingroup module_ctrlkcal
*/
//------------------------------------------------------------------------------
tOplkError ctrlkcal_getCmd(tCtrlCmdType* pCmd_p)
{
    UINT16    cmd;

    if (dualprocshm_readDataCommon(instance_l.dualProcDrvInst, offsetof(tCtrlBuf, ctrlCmd.cmd),
                                   sizeof(cmd), (UINT8*)&cmd) != kDualprocSuccessful)
        return kErrorGeneralError;

    *pCmd_p = (tCtrlCmdType)cmd;

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Send a return value

The function sends the return value of an executed command to the user stack
by storing it in the control memory block.

\param  retval_p            Return value to send.

\ingroup module_ctrlkcal
*/
//------------------------------------------------------------------------------
void ctrlkcal_sendReturn(UINT16 retval_p)
{
    tCtrlCmd    ctrlCmd;

    ctrlCmd.cmd = 0;
    ctrlCmd.retVal = retval_p;

    dualprocshm_writeDataCommon(instance_l.dualProcDrvInst, offsetof(tCtrlBuf, ctrlCmd),
                                sizeof(tCtrlCmd), (UINT8*)&ctrlCmd);
}

//------------------------------------------------------------------------------
/**
\brief  Set the kernel stack status

The function stores the status of the kernel stack in the control memory block.

\param  status_p                Status to set.

\ingroup module_ctrlkcal
*/
//------------------------------------------------------------------------------
void ctrlkcal_setStatus(UINT16 status_p)
{
    dualprocshm_writeDataCommon(instance_l.dualProcDrvInst, offsetof(tCtrlBuf, status),
                                sizeof(status_p), (UINT8*)&status_p);
}

//------------------------------------------------------------------------------
/**
\brief  Update the heartbeat counter

The function updates its heartbeat counter in the control memory block which
can be used by the user stack to detect if the kernel stack is still running.

\param  heartbeat_p         Heartbeat counter to store in the control memory
                            block.
\ingroup module_ctrlkcal
*/
//------------------------------------------------------------------------------
void ctrlkcal_updateHeartbeat(UINT16 heartbeat_p)
{
    dualprocshm_writeDataCommon(instance_l.dualProcDrvInst, offsetof(tCtrlBuf, heartbeat),
                                sizeof(heartbeat_p), (UINT8*)&heartbeat_p);
}

//------------------------------------------------------------------------------
/**
\brief  Store the init parameters for user stack

The function stores the openPOWERLINK initialization parameter so that they
can be accessed by the user stack. It is used to notify the user stack about
parameters modified in the kernel stack.

\param  pInitParam_p        Specifies where to read the init parameters.

\ingroup module_ctrlkcal

*/
//------------------------------------------------------------------------------
void ctrlkcal_storeInitParam(tCtrlInitParam* pInitParam_p)
{
    dualprocshm_writeDataCommon(instance_l.dualProcDrvInst, offsetof(tCtrlBuf, initParam),
                                sizeof(tCtrlInitParam), (UINT8*)pInitParam_p);
}

//------------------------------------------------------------------------------
/**
\brief  Read the init parameters from user stack

The function reads the initialization parameter from the user stack.

\param  pInitParam_p        Specifies where to store the read init parameters.

\return The function returns a tOplkError error code.

\ingroup module_ctrlkcal
*/
//------------------------------------------------------------------------------
tOplkError ctrlkcal_readInitParam(tCtrlInitParam* pInitParam_p)
{
    tDualprocReturn    dualRet;

    dualRet = dualprocshm_readDataCommon(instance_l.dualProcDrvInst, offsetof(tCtrlBuf, initParam),
                                         sizeof(tCtrlInitParam), (UINT8*)pInitParam_p);

    if (dualRet != kDualprocSuccessful)
    {
        DEBUG_LVL_ERROR_TRACE("Cannot read initparam (0x%X)\n", dualRet);
        return kErrorGeneralError;
    }

    return kErrorOk;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

/// \}
