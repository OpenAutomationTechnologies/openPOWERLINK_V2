/**
********************************************************************************
\file   ctrlu.c

\brief  User stack control module

This file contains the implementation of the user stack control module.

\ingroup module_ctrlu
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
#include <Epl.h>
#include <kernel/EplDllk.h>
#include <kernel/eventk.h>
#include <kernel/EplNmtk.h>
#include <kernel/EplObdk.h>
#include <kernel/dllkcal.h>
#include <kernel/pdokcal.h>
#include <user/pdoucal.h>
#include <user/pdou.h>
#include <user/dllucal.h>
#include <user/errhndu.h>
#include <user/EplLedu.h>
#include <user/EplNmtCnu.h>
#include <user/EplNmtMnu.h>
#include <user/EplSdoComu.h>
#include <user/EplIdentu.h>
#include <user/EplStatusu.h>
#include <user/EplTimeru.h>
#include <user/EplCfmu.h>
#include <EplTgtConio.h>

#include <ctrl.h>
#include <user/ctrlucal.h>

#if defined(CONFIG_INCLUDE_VETH)
#include <kernel/VirtualEthernet.h>
#endif

#if defined(CONFIG_INCLUDE_PDOK)
#include <kernel/pdok.h>
#endif

#if EPL_USE_SHAREDBUFF != FALSE
#include <SharedBuff.h>
#endif

#if (EPL_OBD_USE_LOAD_CONCISEDCF != FALSE)
#include <EplObdCdc.h>
#endif

#if EPL_NMTMNU_PRES_CHAINING_MN != FALSE
#include <user/EplSyncu.h>
#endif

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
typedef struct
{
    UINT16          lastHeartbeat;          ///< last detected heartbeat
} tCtrluInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tCtrluInstance      instance_l;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static tEplKernel initNmtu(tEplApiInstance *pApiInstance_p, tEplApiCbFuncs* pCbFuncs_p);
static tEplKernel initObd(tEplApiInstance *pApiInstance_p);

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Initialize user control module

The function initializes the user control module.

\return The function returns a tEplKernel error code.

\ingroup module_ctrlu
*/
//------------------------------------------------------------------------------
tEplKernel ctrlu_init(void)
{
    tShbError           shbError;
    tEplKernel          ret;

    TRACE ("Initialize ctrl module ...\n");

    instance_l.lastHeartbeat = 0;

#if EPL_USE_SHAREDBUFF != FALSE
    shbError = ShbInit();
    if (shbError != kShbOk)
    {
        EPL_DBGLVL_ERROR_TRACE ("Could not initialize Shared Buffer\n");
        return kEplNoResource;
    }
#endif

    if ((ret = ctrlucal_init()) != kEplSuccessful)
    {
        EPL_DBGLVL_ERROR_TRACE ("Could not initialize ctrlucal\n");
        goto Exit;
    }

    if ((ret = ctrlucal_checkKernelStack()) != kEplSuccessful)
    {
        ctrlucal_exit();
        goto Exit;
    }
    return kEplSuccessful;

Exit:
#if EPL_USE_SHAREDBUFF != FALSE
    ShbExit();
#endif
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Cleanup user control module

The function cleans up the user control module.

\return The function returns a tEplKernel error code.

\ingroup module_ctrlu
*/
//------------------------------------------------------------------------------
void ctrlu_exit(void)
{
    ctrlucal_exit();

#if EPL_USE_SHAREDBUFF != FALSE
    ShbExit();
#endif
}

//------------------------------------------------------------------------------
/**
\brief  Initialize openPOWERLINK stack

The function initializes the openPOWERLINK stack. It initializes all
user modules and communication with the kernel control module to initialize
the kernel modules.

After return from this function the application must start the NMT state
machine via EplApiExecNmtCommand(kEplNmtEventSwReset) and thereby the whole
openPOWERLINK stack.

\param  pInitParam_p            Pointer to the initialization parameters
                                provided by the application.
\param  pApiInstance_p          Pointer to the instance variable of the stack.
\param  pCbFuncs_p              Pointer to API callback function structure.

\return The function returns a tEplKernel error code.

\ingroup module_ctrlu
*/
//------------------------------------------------------------------------------
tEplKernel ctrlu_initStack(tEplApiInitParam * pInitParam_p,
                           tEplApiInstance* pApiInstance_p,
                           tEplApiCbFuncs* pCbFuncs_p)
{
    tEplKernel              ret = kEplSuccessful;
    tCtrlInitParam          ctrlParam;

    // reset instance structure
    EPL_MEMSET(pApiInstance_p, 0, sizeof (tEplApiInstance));
    EPL_MEMCPY(&pApiInstance_p->m_InitParam, pInitParam_p,
               min(sizeof (tEplApiInitParam),
               (size_t) pInitParam_p->m_uiSizeOfStruct));

    // check event callback function pointer
    if (pApiInstance_p->m_InitParam.m_pfnCbEvent == NULL)
    {   // application must always have an event callback function
        ret = kEplApiInvalidParam;
        goto Exit;
    }

    if ((ret = initObd(pApiInstance_p)) != kEplSuccessful)
        goto Exit;

    TRACE ("Initialize Eventu module...\n");
    if ((ret = eventu_init(pCbFuncs_p->pfnCbProcessEvent)) != kEplSuccessful)
        goto Exit;

    TRACE ("Initialize Timeru module...\n");
    if ((ret = EplTimeruInit()) != kEplSuccessful)
        goto Exit;

    TRACE ("Initializing kernel modules ...\n");
    EPL_MEMCPY (ctrlParam.aMacAddress, pApiInstance_p->m_InitParam.m_abMacAddress, 6);
    strncpy(ctrlParam.szEthDevName, pApiInstance_p->m_InitParam.m_HwParam.m_pszDevName, 127);
    ctrlParam.ethDevNumber = pApiInstance_p->m_InitParam.m_HwParam.m_uiDevNumber;
    ctrlucal_storeInitParam(&ctrlParam);

    if ((ret = ctrlucal_executeCmd(kCtrlInitStack)) != kEplSuccessful)
        goto Exit;

    /* Read back init param because current MAC address was copied by DLLK */
    ret = ctrlucal_readInitParam(&ctrlParam);
    if (ret != kEplSuccessful)
    {
        goto Exit;
    }

    EPL_MEMCPY (pApiInstance_p->m_InitParam.m_abMacAddress, ctrlParam.aMacAddress, 6);

    TRACE ("initialize error handler user module...\n");
    ret = errhndu_init();
    if (ret != kEplSuccessful)
    {
        goto Exit;
    }

#if defined(CONFIG_INCLUDE_DLLU)
    TRACE ("Initialize DlluCal module...\n");
    ret = dllucal_init();
    if (ret != kEplSuccessful)
    {
        goto Exit;
    }
#endif

#if defined(CONFIG_INCLUDE_PDOU)
    TRACE ("Initialize Pdou module...\n");
    ret = pdou_init(pApiInstance_p->m_InitParam.m_pfnCbSync);
    if (ret != kEplSuccessful)
    {
        goto Exit;
    }
#endif

#if defined(CONFIG_INCLUDE_NMTU)
    if ((ret = initNmtu(pApiInstance_p, pCbFuncs_p)) != kEplSuccessful)
        goto Exit;
#endif

#if defined(CONFIG_INCLUDE_LEDU)
    ret = EplLeduInit(pCbFuncs_p->pfnCbLedStateChange);
    if (ret != kEplSuccessful)
    {
        goto Exit;
    }
#endif

#if defined(CONFIG_INCLUDE_SDOS) || defined(CONFIG_INCLUDE_SDOC)
    // init sdo command layer
    TRACE ("Initialize SdoCom module...\n");
    ret = EplSdoComInit();
    if (ret != kEplSuccessful)
    {
        goto Exit;
    }
#endif

#if defined (CONFIG_INCLUDE_CFM)
    TRACE ("Initialize Cfm module...\n");
    ret = EplCfmuAddInstance(pCbFuncs_p->pfnCbCfmProgress, pCbFuncs_p->pfnCbCfmResult);
    if (ret != kEplSuccessful)
    {
        goto Exit;
    }
#endif

    // the application must start NMT state machine
    // via EplApiExecNmtCommand(kEplNmtEventSwReset)
    // and thereby the whole EPL stack

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Shutdown openPOWERLINK stack

The function shuts down the openPOWERLINK stack. I cleans up all user modules
and the kernel modules by using the kernel control module.

\return The function returns a tEplKernel error code.

\ingroup module_ctrlu
*/
//------------------------------------------------------------------------------
tEplKernel ctrlu_shutdownStack(void)
{
    tEplKernel      ret = kEplSuccessful;

#if defined(CONFIG_INCLUDE_CFM)
    ret = EplCfmuDelInstance();
    TRACE("EplCfmuDelInstance():    0x%X\n", ret);
#endif

#if defined(CONFIG_INCLUDE_SDOS) || defined(CONFIG_INCLUDE_SDOC)
    ret = EplSdoComDelInstance();
    TRACE("EplSdoComDelInstance():  0x%X\n", ret);
#endif

#if defined(CONFIG_INCLUDE_LEDU)
    ret = EplLeduDelInstance();
    TRACE("EplLeduDelInstance():    0x%X\n", ret);
#endif

#if defined(CONFIG_INCLUDE_NMT_MN)
    ret = EplNmtMnuDelInstance();
    TRACE("EplNmtMnuDelInstance():  0x%X\n", ret);

    ret = EplIdentuDelInstance();
    TRACE("EplIdentuDelInstance():  0x%X\n", ret);

    ret = EplStatusuDelInstance();
    TRACE("EplStatusuDelInstance():  0x%X\n", ret);

#if EPL_NMTMNU_PRES_CHAINING_MN != FALSE
    ret = EplSyncuDelInstance();
#endif

#endif

#if defined(CONFIG_INCLUDE_NMT_CN)
    ret = EplNmtCnuDelInstance();
    TRACE("EplNmtCnuDelInstance():  0x%X\n", ret);
#endif

#if defined(CONFIG_INCLUDE_NMTU)
    ret = EplNmtuDelInstance();
    TRACE("EplNmtuDelInstance():    0x%X\n", ret);
#endif

#if defined(CONFIG_INCLUDE_PDOU)
    ret = pdou_exit();
    TRACE("pdou_exit():    0x%X\n", ret);
#endif

#if defined(CONFIG_INCLUDE_VETH)
    ret = VEthDelInstance();
#endif

#if defined(CONFIG_INCLUDE_DLLU)
    ret = dllucal_exit();
    TRACE("dllucal_exit(): 0x%X\n", ret);
#endif

    ret = errhndu_exit();
    TRACE("errhndu_exit():  0x%X\n", ret);

    ret = EplTimeruDelInstance();
    TRACE("EplTimeruDelInstance():  0x%X\n", ret);

    ret = ctrlucal_executeCmd(kCtrlCleanupStack);
    TRACE("shoutdown kernel modules():  0x%X\n", ret);

    ret = eventu_exit();
    TRACE("eventu_exit():  0x%X\n", ret);

    ret = EplObdDeleteInstance();

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Check if kernel stack is running

The function checks if the kernel stack is still running.

\return Returns TRUE if the kernel stack is running or FALSE if is not running.

\ingroup module_ctrlu
*/
//------------------------------------------------------------------------------
BOOL ctrlu_checkKernelStack(void)
{
    UINT16 heartbeat;

    heartbeat = ctrlucal_getHeartbeat();
    if (heartbeat == instance_l.lastHeartbeat)
    {
        return FALSE;
    }
    else
    {
        instance_l.lastHeartbeat = heartbeat;
        return TRUE;
    }
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief  initialize NMTU modules

The function initializes the NMTU modules.

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
static tEplKernel initNmtu(tEplApiInstance* pApiInstance_p, tEplApiCbFuncs* pCbFuncs_p)
{
    tEplKernel      Ret = kEplSuccessful;

    // initialize EplNmtCnu module
#if defined(CONFIG_INCLUDE_NMT_CN)
    TRACE ("Initialize NMT_CN module...\n");
    Ret = EplNmtCnuAddInstance(pApiInstance_p->m_InitParam.m_uiNodeId);
    if (Ret != kEplSuccessful)
        goto Exit;

    Ret = EplNmtCnuRegisterCheckEventCb(pCbFuncs_p->pfnCbCnCheckEvent);
    if (Ret != kEplSuccessful)
        goto Exit;
#else
    UNUSED_PARAMETER(pApiInstance_p);
#endif

    // initialize EplNmtu module
#if defined(CONFIG_INCLUDE_NMTU)
    TRACE ("Initialize NMTu module...\n");
    Ret = EplNmtuInit();
    if (Ret != kEplSuccessful)
        goto Exit;

    // register NMT event callback function
    Ret = EplNmtuRegisterStateChangeCb(pCbFuncs_p->pfnCbNmtStateChange);
    if (Ret != kEplSuccessful)
        goto Exit;
#endif

#if defined(CONFIG_INCLUDE_NMT_MN)
    // initialize EplNmtMnu module
    TRACE ("Initialize NMT_MN module...\n");
    Ret = EplNmtMnuInit(pCbFuncs_p->pfnCbNodeEvent, pCbFuncs_p->pfnCbBootEvent);
    if (Ret != kEplSuccessful)
        goto Exit;

    // initialize EplIdentu module
    TRACE ("Initialize Identu module...\n");
    Ret = EplIdentuInit();
    if (Ret != kEplSuccessful)
        goto Exit;

    // initialize EplStatusu module
    TRACE ("Initialize Statusu module...\n");
    Ret = EplStatusuInit();
    if (Ret != kEplSuccessful)
        goto Exit;

#if EPL_NMTMNU_PRES_CHAINING_MN != FALSE
    // initialize EplSyncu module
    TRACE ("Initialize Syncu module...\n");
    Ret = EplSyncuInit();
    if (Ret != kEplSuccessful)
        goto Exit;
#endif

#endif

Exit:
    return Ret;
}

//------------------------------------------------------------------------------
/**
\brief Initilize object dictionary

The function initializes the object dictionary

\param  pApiInstance_p          Pointer to API instance variable

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
static tEplKernel initObd(tEplApiInstance *pApiInstance_p)
{
    tEplKernel          ret = kEplSuccessful;
    tEplObdInitParam    ObdInitParam;

// TODO jba Is this really a obdk??
#if defined(CONFIG_INCLUDE_OBDK)
    TRACE ("Initialize OBD module...\n");
    if (pApiInstance_p->m_InitParam.m_pfnObdInitRam == NULL)
        return kEplApiNoObdInitRam;

    ret = pApiInstance_p->m_InitParam.m_pfnObdInitRam(&ObdInitParam);
    if (ret != kEplSuccessful)
        return ret;

    ret = EplObdInit(&ObdInitParam);
    if (ret != kEplSuccessful)
        return ret;
#endif

    return ret;
}

//------------------------------------------------------------------------------
/// \}
//------------------------------------------------------------------------------

