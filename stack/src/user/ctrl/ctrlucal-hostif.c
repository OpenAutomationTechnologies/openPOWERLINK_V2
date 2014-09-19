/**
********************************************************************************
\file   ctrlucal-hostif.c

\brief  User control CAL module using a shared memory block

This file contains an implementation of the user control CAL module which uses
a shared memory block for communication with the kernel layer.

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
#include <common/target.h>
#include <user/ctrlucal.h>

#include <unistd.h>
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
#define CMD_TIMEOUT_SEC     20 // command timeout in seconds

//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------
/**
\brief  Instance of the user control CAL host interface module

The structure defines the instance variables of the user control CAL module
for the host interface.
*/
typedef struct
{
    tHostifInstance     hifInstance;                        ///< Host interface instance
    BOOL                fIrqMasterEnable;                   ///< IRQ master enable
    tHostifInstanceId   dynBufInst;                         ///< Dynamic buffer instance
} tCtrluCalInstance;
//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tCtrluCalInstance instance_l;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static UINT8* getDynBuff(UINT32 pcpBase_p);
static void freeDynBuff(UINT8* pDynBufBase_p);

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
    tHostifReturn hifRet;
    tHostifConfig hifConfig;

    OPLK_MEMSET(&instance_l, 0, sizeof(instance_l));

    OPLK_MEMSET(&hifConfig, 0, sizeof(hifConfig));

    hifConfig.instanceNum = 0;
    hifConfig.pBase = (UINT8*)HOSTIF_BASE; //FIXME: Get it from somewhere else?
    hifConfig.version.revision = HOSTIF_VERSION_REVISION;
    hifConfig.version.minor = HOSTIF_VERSION_MINOR;
    hifConfig.version.major = HOSTIF_VERSION_MAJOR;

    hifRet = hostif_create(&hifConfig, &instance_l.hifInstance);
    if (hifRet != kHostifSuccessful)
    {
        DEBUG_LVL_ERROR_TRACE("Could not initialize Host Interface (0x%X)\n", hifRet);
        return kErrorNoResource;
    }

    //disable master irq
    instance_l.fIrqMasterEnable = FALSE;

    hifRet = hostif_irqMasterEnable(instance_l.hifInstance, instance_l.fIrqMasterEnable);
    if (hifRet != kHostifSuccessful)
    {
        DEBUG_LVL_ERROR_TRACE("Could not disable Master Irq (0x%X)\n", hifRet);
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
    tHostifReturn hifRet;

    //disable master irq
    instance_l.fIrqMasterEnable = FALSE;

    hifRet = hostif_irqMasterEnable(instance_l.hifInstance, instance_l.fIrqMasterEnable);
    if (hifRet != kHostifSuccessful)
    {
        DEBUG_LVL_ERROR_TRACE("Could not disable Master Irq (0x%X)\n", hifRet);
    }

    hifRet = hostif_delete(instance_l.hifInstance);
    if (hifRet != kHostifSuccessful)
    {
        DEBUG_LVL_ERROR_TRACE("Could not delete Host Inetrface (0x%X)\n", hifRet);
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
    tHostifReturn hifRet;

    if (instance_l.fIrqMasterEnable == FALSE)
    {
        //enable master irq
        instance_l.fIrqMasterEnable = TRUE;

        hifRet = hostif_irqMasterEnable(instance_l.hifInstance, instance_l.fIrqMasterEnable);
        if (hifRet != kHostifSuccessful)
        {
            DEBUG_LVL_ERROR_TRACE("Could not enable Master Irq (0x%X)\n", hifRet);
            return kErrorNoResource;
        }
    }

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
    tHostifReturn hifret;
    tHostifCommand hifcmd = (tHostifCommand)cmd_p;
    tHostifError hiferr = 0;
    int timeout;

    hostif_setError(instance_l.hifInstance, hiferr);
    hostif_setCommand(instance_l.hifInstance, hifcmd);

    /* wait for response */
    for (timeout = 0; timeout < CMD_TIMEOUT_SEC; timeout++)
    {
        target_msleep(1000U);

        hifret = hostif_getCommand(instance_l.hifInstance, &hifcmd);
        if (hifret != kHostifSuccessful)
            return kErrorGeneralError;

        hifret = hostif_getError(instance_l.hifInstance, &hiferr);
        if (hifret != kHostifSuccessful)
            return kErrorGeneralError;

        if (hifcmd == 0)
        {
            return hiferr;
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
\retval kErrorOk             Kernel stack is initialized
\retval kErrorNoResource     Kernel stack is not running or in wrong state

\ingroup module_ctrlucal
*/
//------------------------------------------------------------------------------
tOplkError ctrlucal_checkKernelStack(void)
{
    tOplkError ret;
    UINT16 kernelStatus;
    BOOL fExit = FALSE;
    int timeout = 0;

    DEBUG_LVL_CTRL_TRACE("Check Kernel Stack...\n");

    while (!fExit)
    {
        switch (kernelStatus = ctrlucal_getStatus())
        {
            case kCtrlStatusReady:
                DEBUG_LVL_CTRL_TRACE("-> Kernel Stack is ready\n");
                fExit = TRUE;
                ret = kErrorOk;
                break;

            case kCtrlStatusRunning:
                /* try to shutdown kernel stack */
                DEBUG_LVL_CTRL_TRACE("-> Try to shutdown Kernel Stack\n");

                ret = ctrlucal_executeCmd(kCtrlCleanupStack);
                if (ret != kErrorOk)
                {
                    fExit = TRUE;
                    ret = kErrorNoResource;
                }
                break;

            default:
                if (timeout == 0)
                {
                    DEBUG_LVL_CTRL_TRACE("-> Wait for Kernel Stack\n");
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
UINT16 ctrlucal_getStatus(void)
{
    tHostifReturn hifret;
    UINT16 status;

    hifret = hostif_getState(instance_l.hifInstance, (tHostifState*)&status);
    if (hifret != kHostifSuccessful)
        status = kCtrlStatusUnavailable;

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
    tHostifReturn hifret;
    UINT16 heartbeat;

    hifret = hostif_getHeartbeat(instance_l.hifInstance, &heartbeat);
    if (hifret != kHostifSuccessful)
        heartbeat = 0; // return constant heartbeat, so the user recognizes issue

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
    tHostifReturn   hifret;
    UINT8*          pInitBase;
    UINT8*          pDst;

    hifret = hostif_getInitParam(instance_l.hifInstance, &pInitBase);
    if (hifret != kHostifSuccessful)
    {
        DEBUG_LVL_ERROR_TRACE("%s() Getting init base failed (0x%X)!\n", __func__, hifret);
        return;
    }

    pDst = getDynBuff((UINT32)pInitBase);

    if (pDst != NULL)
        OPLK_MEMCPY(pDst, pInitParam_p, sizeof(tCtrlInitParam));

    freeDynBuff(pDst);
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
    tOplkError      ret = kErrorOk;
    tHostifReturn   hifret;
    UINT8*          pInitBase;
    UINT8*          pSrc;

    hifret = hostif_getInitParam(instance_l.hifInstance, &pInitBase);
    if (hifret != kHostifSuccessful)
    {
        DEBUG_LVL_ERROR_TRACE("%s() Getting init base failed (0x%X)!\n", __func__, hifret);
        ret = kErrorNoResource;
        goto Exit;
    }

    pSrc = getDynBuff((UINT32)pInitBase);

    if (pSrc == NULL)
        return kErrorNoResource;

    OPLK_MEMCPY(pInitParam_p, pSrc, sizeof(tCtrlInitParam));

    freeDynBuff(pSrc);

Exit:
    return ret;
}


//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief  Get a dynamic buffer

This function sets a dynamic buffer to the provided Pcp memory buffer.

\param  pcpBase_p       Address to buffer in Pcp's memory environment

\return The function returns the acquired dynamic buffer.
\retval NULL    The dynamic buffer allocation failed.
*/
//------------------------------------------------------------------------------
static UINT8* getDynBuff(UINT32 pcpBase_p)
{
    tHostifReturn hifret;
    UINT8* pDynBufBase;

    hifret = hostif_dynBufAcquire(instance_l.hifInstance, pcpBase_p, &pDynBufBase);
    if (hifret != kHostifSuccessful)
    {
        DEBUG_LVL_ERROR_TRACE("%s() Acquiring dynamic buffer failed (0x%X)!\n", __func__, hifret);
        pDynBufBase = NULL;
    }

    return (UINT8*)pDynBufBase;
}

//------------------------------------------------------------------------------
/**
\brief  Free a dynamic buffer

This function frees a dynamic buffer previously acquired.

\param  pDynBufBase_p   Address to buffer in Pcp's memory environment
*/
//------------------------------------------------------------------------------
static void freeDynBuff(UINT8* pDynBufBase_p)
{
    tHostifReturn hifret;

    hifret = hostif_dynBufFree(instance_l.hifInstance, pDynBufBase_p);

    if (hifret != kHostifSuccessful)
    {
        DEBUG_LVL_ERROR_TRACE("%s() Freeing dynamic buffer failed (0x%X)", __func__, hifret);
    }
}

///\}
