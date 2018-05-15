/**
********************************************************************************
\file   ctrlkcal-hostif.c

\brief  Kernel control CAL module using the host interface IP-core

This file contains an implementation of the kernel control CAL module which uses
the host interface IP-core.

\ingroup module_ctrlkcal
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2017, B&R Industrial Automation GmbH
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
/**
\brief  Instance of the kernel control CAL host interface module

The structure defines the instance variables of the kernel control CAL module
for the host interface.
*/
typedef struct
{
    tHostifInstance     hifInstance;                        ///< Host interface instance
    void*               pInitParamBase;                     ///< Init parameters
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

\return The function returns a tOplkError error code.

\ingroup module_ctrlkcal
*/
//------------------------------------------------------------------------------
tOplkError ctrlkcal_init(void)
{
    tHostifReturn hifRet;
    tHostifConfig hifConfig;

    // Reset the instance
    OPLK_MEMSET(&instance_l, 0, sizeof(instance_l));

    OPLK_MEMSET(&hifConfig, 0, sizeof(hifConfig));

    hifConfig.instanceNum = 0;
    hifConfig.pBase = (UINT8*)HOSTIF_BASE;
    hifConfig.version.revision = HOSTIF_VERSION_REVISION;
    hifConfig.version.minor = HOSTIF_VERSION_MINOR;
    hifConfig.version.major = HOSTIF_VERSION_MAJOR;

    hifRet = hostif_create(&hifConfig, &instance_l.hifInstance);
    if (hifRet != kHostifSuccessful)
        goto Cleanup;

    hifRet = hostif_getInitParam(instance_l.hifInstance, &instance_l.pInitParamBase);
    if (hifRet != kHostifSuccessful)
        goto Cleanup;

    ctrlkcal_setStatus(kCtrlStatusReady);

    return kErrorOk;

Cleanup:
    DEBUG_LVL_ERROR_TRACE("Could not initialize Host Interface (0x%X)\n", hifRet);
    ctrlkcal_exit();
    return kErrorNoResource;
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
    hostif_delete(instance_l.hifInstance);
    // ignore ret, because we shutdown anyway

    // Reset the instance
    OPLK_MEMSET(&instance_l, 0, sizeof(instance_l));
}

//------------------------------------------------------------------------------
/**
\brief  Process kernel control CAL module

This function provides processing time for the CAL module.

\return The function returns a tOplkError error code.

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

\param[out]     pCmd_p              The command to be executed.

\return The function returns a tOplkError error code.

\ingroup module_ctrlkcal
*/
//------------------------------------------------------------------------------
tOplkError ctrlkcal_getCmd(tCtrlCmdType* pCmd_p)
{
    tHostifReturn hifret;
    tHostifCommand hifcmd;

    if (pCmd_p == NULL)
        return kErrorGeneralError;

    hifret = hostif_getCommand(instance_l.hifInstance, &hifcmd);
    if (hifret != kHostifSuccessful)
        return kErrorGeneralError;

    *pCmd_p = (tCtrlCmdType)hifcmd;

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Send a return value

The function sends the return value of an executed command to the user stack
by storing it in the control memory block.

\param[in]      retval_p            Return value to send.

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

\param[in]      status_p            Status to set.

\ingroup module_ctrlkcal
*/
//------------------------------------------------------------------------------
void ctrlkcal_setStatus(tCtrlKernelStatus status_p)
{
    tHostifState hifsta = (tHostifState)status_p;

    hostif_setState(instance_l.hifInstance, hifsta);
}

//------------------------------------------------------------------------------
/**
\brief  Get the kernel stack status

The function gets the status of the kernel stack.

\return The function returns the status of the kernel stack.

\ingroup module_ctrlkcal
*/
//------------------------------------------------------------------------------
tCtrlKernelStatus ctrlkcal_getStatus(void)
{
    return kCtrlStatusUnavailable;
}

//------------------------------------------------------------------------------
/**
\brief  Update the heartbeat counter

The function updates its heartbeat counter in the control memory block which
can be used by the user stack to detect if the kernel stack is still running.

\param[in]      heartbeat_p         Heartbeat counter to store in the control memory
                                    block.
\ingroup module_ctrlkcal
*/
//------------------------------------------------------------------------------
void ctrlkcal_updateHeartbeat(UINT16 heartbeat_p)
{
    hostif_setHeartbeat(instance_l.hifInstance, heartbeat_p);
}

//------------------------------------------------------------------------------
/**
\brief  Store the init parameters for user stack

The function stores the openPOWERLINK initialization parameter so that they
can be accessed by the user stack. It is used to notify the user stack about
parameters modified in the kernel stack.

\param[in]      pInitParam_p        Specifies where to read the init parameters.

\ingroup module_ctrlkcal

*/
//------------------------------------------------------------------------------
void ctrlkcal_storeInitParam(const tCtrlInitParam* pInitParam_p)
{
    // Check parameter validity
    ASSERT(pInitParam_p != NULL);

    if (instance_l.pInitParamBase != NULL)
        OPLK_MEMCPY(instance_l.pInitParamBase, pInitParam_p, sizeof(tCtrlInitParam));
}

//------------------------------------------------------------------------------
/**
\brief  Read the init parameters from user stack

The function reads the initialization parameter from the user stack.

\param[out]     pInitParam_p        Specifies where to store the read init parameters.

\return The function returns a tOplkError error code.

\ingroup module_ctrlkcal
*/
//------------------------------------------------------------------------------
tOplkError ctrlkcal_readInitParam(tCtrlInitParam* pInitParam_p)
{
    // Check parameter validity
    ASSERT(pInitParam_p != NULL);

    if (instance_l.pInitParamBase == NULL)
        return kErrorNoResource;

    OPLK_MEMCPY(pInitParam_p, instance_l.pInitParamBase, sizeof(tCtrlInitParam));

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Read file chunk

The function reads the file chunk descriptor and data from the file transfer
buffer.

\param[out]     pDesc_p             Pointer to buffer for storing the chunk descriptor
\param[in]      bufferSize_p        Size of buffer for storing the chunk data
\param[out]     pBuffer_p           Pointer to buffer for storing the chunk data

\return The function returns a tOplkError code.

\ingroup module_ctrlk
*/
//------------------------------------------------------------------------------
tOplkError ctrlkcal_readFileChunk(tOplkApiFileChunkDesc* pDesc_p,
                                  size_t bufferSize_p,
                                  void* pBuffer_p)
{
    UNUSED_PARAMETER(pDesc_p);
    UNUSED_PARAMETER(bufferSize_p);
    UNUSED_PARAMETER(pBuffer_p);

    // This CAL is not supporting that feature -> return no resource available.
    return kErrorNoResource;
}

//------------------------------------------------------------------------------
/**
\brief  Get maximum file chunk size

The function returns the maximum file chunk size supported by the ctrl cal
implementation.

\return The function returns the maximum file chunk size.

\ingroup module_ctrlk
*/
//------------------------------------------------------------------------------
size_t ctrlkcal_getMaxFileChunkSize(void)
{
    // This CAL is not supporting that feature -> return zero size.
    return 0;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

/// \}
