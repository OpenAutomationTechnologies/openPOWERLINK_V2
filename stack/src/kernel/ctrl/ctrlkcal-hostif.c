/**
********************************************************************************
\file   ctrlkcal-hostif.c

\brief  Kernel control CAL module using the host interface ipcore

This file contains an implementation of the kernel control CAL module which uses
the host interface ipcore.

\ingroup module_ctrlkcal
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2012, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
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

#include <common/ctrl.h>
#include <common/ctrlcal.h>
#include <kernel/ctrlkcal.h>

#include <hostiflib.h>

//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#ifndef HOSTIF_BASE
#error "Host interface base address not set!"
#endif

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
#define CTRL_HOSTIF_INITPARAM_SIZE  HOSTIF_USER_INIT_PAR_SIZE

//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------
typedef struct
{
    tHostifInstance     hifInstance;
    UINT8*              pInitParamBase;
} tCtrlkCalInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tCtrlkCalInstance instance_l;

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

\return The function returns a tEplKernel error code.

\ingroup module_ctrlkcal
*/
//------------------------------------------------------------------------------
tEplKernel ctrlkcal_init (void)
{
    tHostifReturn hifRet;
    tHostifConfig hifConfig;

    EPL_MEMSET(&instance_l, 0, sizeof(instance_l));

    EPL_MEMSET(&hifConfig, 0, sizeof(hifConfig));

    hifConfig.instanceNum = 0;
    hifConfig.pBase = (UINT8*)HOSTIF_BASE;
    hifConfig.version.revision = HOSTIF_VERSION_REVISION;
    hifConfig.version.minor = HOSTIF_VERSION_MINOR;
    hifConfig.version.major = HOSTIF_VERSION_MAJOR;

    hifRet = hostif_create(&hifConfig, &instance_l.hifInstance);
    if(hifRet != kHostifSuccessful)
        goto Cleanup;

    hifRet = hostif_getInitParam(instance_l.hifInstance, &instance_l.pInitParamBase);
    if(hifRet != kHostifSuccessful)
        goto Cleanup;

    ctrlkcal_setStatus(kCtrlStatusReady);

    return kEplSuccessful;

Cleanup:
    EPL_DBGLVL_ERROR_TRACE ("Could not initialize Host Interface (0x%X)\n", hifRet);
    ctrlkcal_exit();
    return kEplNoResource;
}

//------------------------------------------------------------------------------
/**
\brief  Cleanup kernel control CAL module

The function cleans up the kernel control CAL module. It resets the control
memory block and cleans up the underlying CAL module used for implementing
the memory block access functions.

\ingroup module_ctrlkcal
*/
//------------------------------------------------------------------------------
void ctrlkcal_exit (void)
{
    tHostifReturn hifRet;

    instance_l.pInitParamBase = NULL;

    hifRet = hostif_delete(instance_l.hifInstance);
    if(hifRet != kHostifSuccessful)
        EPL_DBGLVL_ERROR_TRACE("Could not delete Host Inetrface (0x%X)\n", hifRet);
}

//------------------------------------------------------------------------------
/**
\brief  Process kernel control CAL module

This function provides processing time for the CAL module.

\return The function returns a tEplKernel error code.

\ingroup module_ctrlkcal
*/
//------------------------------------------------------------------------------
tEplKernel ctrlkcal_process (void)
{
    return kEplSuccessful;
}

//------------------------------------------------------------------------------
/**
\brief  Get control command

The function reads a control command stored by the user in the control memory
block to execute a kernel control function.

\param  pCmd_p            The command to be executed.

\return The function returns a tEplKernel error code.

\ingroup module_ctrlkcal
*/
//------------------------------------------------------------------------------
tEplKernel ctrlkcal_getCmd (tCtrlCmdType *pCmd_p)
{
    tHostifReturn hifret;
    tHostifCommand hifcmd;

    if(pCmd_p == NULL)
        return kEplGeneralError;

    hifret = hostif_getCommand(instance_l.hifInstance, &hifcmd);
    if(hifret != kHostifSuccessful)
        return kEplGeneralError;

    *pCmd_p = (tCtrlCmdType)hifcmd;

    return kEplSuccessful;
}

//------------------------------------------------------------------------------
/**
\brief  Reset control command

The function reads a control command stored by the user in the control memory
block to execute a kernel control function.
The function resets the control command, which informs the user layer that the
command was complete.

\ingroup module_ctrlkcal
*/
//------------------------------------------------------------------------------
void ctrlkcal_rstCmd (void)
{
    hostif_setCommand(instance_l.hifInstance, 0);
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
    tHostifCommand hifcmd = 0;
    tHostifError hiferr = (tHostifError)retval_p;

    hostif_setError(instance_l.hifInstance, hiferr);
    hostif_setCommand(instance_l.hifInstance, hifcmd);
}

//------------------------------------------------------------------------------
/**
\brief  Set the kernel stack status

The function stores the status of the kernel stack in the control memory block.

\param  status_p                Status to set.

\ingroup module_ctrlkcal
*/
//------------------------------------------------------------------------------
void ctrlkcal_setStatus (UINT16 status_p)
{
    tHostifState hifsta = (tHostifState)status_p;

    hostif_setState(instance_l.hifInstance, hifsta);
}

//------------------------------------------------------------------------------
/**
\brief  Update the heartbeat counter

The function updates it's heartbeat counter in the control memory block which
can be used by the user stack to detect if the kernel stack is still running.

\param  heartbeat_p         Heartbeat counter to store in the control memory
                            block.
\ingroup module_ctrlkcal
*/
//------------------------------------------------------------------------------
void ctrlkcal_updateHeartbeat (UINT16 heartbeat_p)
{
    hostif_setHeartbeat(instance_l.hifInstance, heartbeat_p);
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
    if(instance_l.pInitParamBase != NULL)
        EPL_MEMCPY(instance_l.pInitParamBase, pInitParam_p, sizeof(tCtrlInitParam));
}

//------------------------------------------------------------------------------
/**
\brief  Read the init parameters from user stack

The function reads the initialization parameter from the user stack.

\param  pInitParam_p        Specifies where to store the read init parameters.

\return The function returns a tEplKernel error code.

\ingroup module_ctrlkcal
*/
//------------------------------------------------------------------------------
tEplKernel ctrlkcal_readInitParam(tCtrlInitParam* pInitParam_p)
{
    if(instance_l.pInitParamBase == NULL)
        return kEplNoResource;

    EPL_MEMCPY(pInitParam_p, instance_l.pInitParamBase, sizeof(tCtrlInitParam));

    return kEplSuccessful;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
