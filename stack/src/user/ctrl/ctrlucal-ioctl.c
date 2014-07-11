/**
********************************************************************************
\file   ctrlucal-ioctl.c

\brief  User control CAL module using a ioctl calls to Linux kernel

This file contains an implementation of the user control CAL module which uses
ioctl calls for communication with the kernel layer.

\ingroup module_ctrlucal
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2014, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
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
#include <common/oplkinc.h>
#include <user/ctrlucal.h>
#include <common/ctrlcal-mem.h>
#include <common/target.h>
#include <oplk/powerlink-module.h>

#include <unistd.h>
#include <stddef.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/mman.h>

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
static int fd_l;           // file descriptor for powerlink device

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
    if ((fd_l = open(PLK_DEV_FILE, O_RDWR)) < 0)
    {
        DEBUG_LVL_ERROR_TRACE("%s() open return error %d (%s)\n", __func__, fd_l, strerror(fd_l));
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
    close(fd_l);
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

\param  cmd_p            Command to execute

\return The function returns a tOplkError error code.

\ingroup module_ctrlucal
*/
//------------------------------------------------------------------------------
tOplkError ctrlucal_executeCmd(tCtrlCmdType cmd_p)
{
    tCtrlCmd            ctrlCmd;
    int                 ret;

    ctrlCmd.cmd = cmd_p;
    ctrlCmd.retVal = 0;

    if ((ret = ioctl(fd_l, PLK_CMD_CTRL_EXECUTE_CMD, &ctrlCmd)) != 0)
    {
        DEBUG_LVL_ERROR_TRACE("%s() ioctl error %d\n", __func__, ret);
        return kErrorGeneralError;
    }

    return ctrlCmd.retVal;
}


//------------------------------------------------------------------------------
/**
\brief Check state of kernel stack

The function checks the state of the kernel stack. If it is already running
it tries to shutdown.

\return The function returns a tOplkError error code.
\retval kErrorOk             Kernel stack is initialized
\retval kErrorNoResource     Kernel stack is not running or in wrong state

\ingroup module_ctrlucal
*/
//------------------------------------------------------------------------------
tOplkError ctrlucal_checkKernelStack(void)
{
    UINT16              kernelStatus;
    tOplkError          ret;

    DEBUG_LVL_CTRL_TRACE("Checking for kernel stack...\n");
    kernelStatus = ctrlucal_getStatus();

    switch (kernelStatus)
    {
        case kCtrlStatusReady:
            ret = kErrorOk;
            break;

        case kCtrlStatusRunning:
            /* try to shutdown kernel stack */
            ret = ctrlucal_executeCmd(kCtrlCleanupStack);
            if (ret != kErrorOk)
            {
                ret = kErrorNoResource;
                break;
            }

            target_msleep(1000);

            kernelStatus = ctrlucal_getStatus();
            if (kernelStatus != kCtrlStatusReady)
            {
                ret = kErrorNoResource;
            }
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
    int         ret;
    UINT16      status;

    if ((ret = ioctl(fd_l, PLK_CMD_CTRL_GET_STATUS, &status)) != 0)
    {
        DEBUG_LVL_ERROR_TRACE("%s() ioctl error %d\n", __func__, ret);
        return kCtrlStatusUnavailable;
    }
    return status;
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
    int         ret;
    UINT16      heartbeat;

    if ((ret = ioctl(fd_l, PLK_CMD_CTRL_GET_HEARTBEAT, &heartbeat)) != 0)
    {
        DEBUG_LVL_ERROR_TRACE("%s() error %d\n", __func__, ret);
        return 0;
    }
    return heartbeat;
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
    int                 ret;

    if ((ret = ioctl(fd_l, PLK_CMD_CTRL_STORE_INITPARAM, pInitParam_p)) != 0)
    {
        DEBUG_LVL_ERROR_TRACE("%s() ioctl error %d\n", __func__, ret);
    }

}

//------------------------------------------------------------------------------
/**
\brief  Read the init parameters from kernel

The function reads the initialization parameter from the kernel stack.

\param  pInitParam_p        Specifies where to store the read init parameters.

\return The function returns a tOplkError error code. It returns always
        kErrorOk!

\ingroup module_ctrlucal
*/
//------------------------------------------------------------------------------
tOplkError ctrlucal_readInitParam(tCtrlInitParam* pInitParam_p)
{
    int                 ret;

    if ((ret = ioctl(fd_l, PLK_CMD_CTRL_READ_INITPARAM, pInitParam_p)) != 0)
    {
        DEBUG_LVL_ERROR_TRACE("%s() ioctl error %d\n", __func__, ret);
        return kErrorGeneralError;
    }
    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Return the file descriptor of the kernel module

The function returns the file descriptor of the kernel module.

\return The function returns the file descriptor.

\ingroup module_ctrlucal
*/
//------------------------------------------------------------------------------
int ctrlucal_getFd(void)
{
    return fd_l;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

/// \}
