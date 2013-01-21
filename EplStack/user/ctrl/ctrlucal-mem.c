/**
********************************************************************************
\file   ctrlucal-mem.c

\brief  User control CAL module using a shared memory block

This file contains an implementation of the user control CAL module which uses
a shared memory block for communication with the kernel layer.

\ingroup module_ctrlucal
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
#include <stddef.h>

#include <EplTgtConio.h>
#include <ctrl.h>
#include <ctrlcal.h>
#include <ctrlcal-mem.h>
#include <user/ctrlucal.h>

//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#define CMD_TIMEOUT_CNT     100     // loop counter for command timeout

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

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
UINT16 getMagic (void);

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Initialize user control CAL module

The function initializes the user control CAL module.

\return The function returns a tEplKernel error code.

\ingroup module_ctrlucal
*/
//------------------------------------------------------------------------------
tEplKernel ctrlucal_init(void)
{
    return ctrlcal_init(sizeof(tCtrlBuf));
}

//------------------------------------------------------------------------------
/**
\brief  Cleanup user control CAL module

The function cleans-up the user control CAL module.

\ingroup module_ctrlucal
*/
//------------------------------------------------------------------------------
void ctrlucal_exit (void)
{
    ctrlcal_exit();
}

//------------------------------------------------------------------------------
/**
\brief    Execute a ctrl command

The function executes a control command in the kernel stack.

\param  cmd_p            Command to execute

\return The function returns a tEplKernel error code.

\ingroup module_ctrlucal
*/
//------------------------------------------------------------------------------
tEplKernel ctrlucal_executeCmd(tCtrlCmdType cmd_p)
{
    tEplKernel          ret;
    tCtrlCmd            ctrlCmd;
    int                 timeout;

    /* write command into shared buffer */
    ctrlCmd.cmd = cmd_p;
    ctrlCmd.retVal = 0;

    ctrlcal_writeData(offsetof(tCtrlBuf, ctrlCmd), &ctrlCmd, sizeof(tCtrlCmd));

    /* wait for response */
    for (timeout = 0; timeout < CMD_TIMEOUT_CNT; timeout++)
    {
        EplTgtMilliSleep(10);
        ctrlcal_readData(&ctrlCmd, offsetof(tCtrlBuf, ctrlCmd), sizeof(tCtrlCmd));
        if (ctrlCmd.cmd == 0)
        {
            ret = ctrlCmd.retVal;
            return ret;
        }
    }

    TRACE("%s() Timeout waiting for return!\n", __func__);
    return kEplGeneralError;
}


//------------------------------------------------------------------------------
/**
\brief Check state of kernel stack

The function checks the state of the kernel stack. If it is already running
it tries to shutdown.

\return The function returns a tEplKernel error code.
\retval kEplSuccessful  If kernel stack is initialized
\retval kEplNoResource  If kernel stack is not running or in wrong state

\ingroup module_ctrlucal
*/
//------------------------------------------------------------------------------
tEplKernel ctrlucal_checkKernelStack(void)
{
    UINT16              kernelStatus;
    tEplKernel          ret;

    TRACE ("Checking for kernel stack...\n");
    if (getMagic() != CTRL_MAGIC)
    {
        TRACE ("Kernel daemon not running! Exiting...\n");
        return kEplNoResource;
    }

    kernelStatus = ctrlucal_getStatus();

    switch(kernelStatus)
    {
        case kCtrlStatusReady:
            ret = kEplSuccessful;
            break;

        case kCtrlStatusRunning:
            /* try to shutdown kernel stack */
            ret = ctrlucal_executeCmd(kCtrlCleanupStack);
            if (ret != kEplSuccessful)
            {
                ret = kEplNoResource;
                break;
            }

            EplTgtMilliSleep(1000);

            kernelStatus = ctrlucal_getStatus();
            if (kernelStatus != kCtrlStatusReady)
            {
                ret = kEplNoResource;
            }
            break;

        default:
            ret = kEplNoResource;
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
    UINT16          status;

    if ((ctrlcal_readData(&status, offsetof(tCtrlBuf, status),
                          sizeof(UINT16))) == kEplSuccessful)
        return status;
    else
        return kCtrlStatusUnavailable;
}

//------------------------------------------------------------------------------
/**
\brief Get the heartbeat of the kernel stack

The function reads the heartbeat genereated by the kernel stack.

\return The function returns the heartbeat counter.

\ingroup module_ctrlucal
*/
//------------------------------------------------------------------------------
UINT16 ctrlucal_getHeartbeat(void)
{
    UINT16      heartbeat;

    if ((ctrlcal_readData(&heartbeat, offsetof(tCtrlBuf, heartbeat),
                                sizeof(UINT16))) == kEplSuccessful)
        return heartbeat;
    else
        return 0;
}

//------------------------------------------------------------------------------
/**
\brief  Store the init parameters for kernel use

The function stores the openPOWERLINK initialization parameter so that they
can be accessed by the kernel stack.

\param  pInitParam_p        Specifies where to read the init parameters.

\ingroup module_ctrlucal
*/
//------------------------------------------------------------------------------
void ctrlucal_storeInitParam(tCtrlInitParam* pInitParam_p)
{
    ctrlcal_writeData(offsetof(tCtrlBuf, initParam), pInitParam_p,
                      sizeof(tCtrlInitParam));
}

//------------------------------------------------------------------------------
/**
\brief  Read the init parameters from kernel

The function reads the initialization parameter from the kernel stack.

\param  pInitParam_p        Specifies where to store the read init parameters.

\return The function returns a tEplKernel error code. It returns always
        kEplSuccessful!

\ingroup module_ctrlucal
*/
//------------------------------------------------------------------------------
tEplKernel ctrlucal_readInitParam(tCtrlInitParam* pInitParam_p)
{
    return ctrlcal_readData(pInitParam_p, offsetof(tCtrlBuf, initParam),
                            sizeof(tCtrlInitParam));
}


//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Get magic number

The function reads the magic number stored in the control memory block to
detect if it's a valid memory block.

\return Return the magic number

\ingroup module_ctrl
*/
//------------------------------------------------------------------------------
UINT16 getMagic (void)
{
    UINT16          magic;

    if ((ctrlcal_readData(&magic, offsetof(tCtrlBuf, magic),
                          sizeof(UINT16))) == kEplSuccessful)
    {
        return magic;
    }

    return 0;
}
