/**
********************************************************************************
\file   pdokcalsync-noosdual.c

\brief  Dual Processor PDO CAL kernel sync module

This file contains an implementation for the kernel PDO CAL sync module which
uses the dualprocshm interrupt feature for synchronisation.

The sync module is responsible to notify the user layer that new PDO data
can be transferred.

\ingroup module_pdokcal
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
#include <oplk/oplkinc.h>
#include <common/pdo.h>
#include <kernel/pdokcal.h>

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
#define TARGET_SYNC_INTERRUPT_ID    1

//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Initialize kernel PDO CAL sync module

The function initializes the kernel PDO CAL sync module.

\return The function returns a tOplkError error code.

\ingroup module_pdokcal
*/
//------------------------------------------------------------------------------
tOplkError pdokcal_initSync(void)
{
    tDualprocDrvInstance    pInstance = dualprocshm_getLocalProcDrvInst();
    tDualprocReturn         dualRet;

    if (pInstance == NULL)
    {
        DEBUG_LVL_ERROR_TRACE("%s() couldn't get PCP dual proc driver instance\n",
                              __func__);
        return kErrorNoResource;
    }

    dualRet = dualprocshm_enableIrq(pInstance, TARGET_SYNC_INTERRUPT_ID, FALSE);

    if (dualRet != kDualprocSuccessful)
    {
        return kErrorNoResource;
    }

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Clean up PDO CAL sync module

The function cleans up the PDO CAL sync module

\ingroup module_pdokcal
*/
//------------------------------------------------------------------------------
void pdokcal_exitSync(void)
{
    tDualprocDrvInstance    pInstance = dualprocshm_getLocalProcDrvInst();

    if (pInstance == NULL)
    {
        DEBUG_LVL_ERROR_TRACE("%s() couldn't get PCP dual proc driver instance\n",
                              __func__);
    }

    dualprocshm_enableIrq(pInstance, TARGET_SYNC_INTERRUPT_ID, FALSE);
}

//------------------------------------------------------------------------------
/**
\brief  Send a sync event

The function sends a sync event.

\return The function returns a tOplkError error code.

\ingroup module_pdokcal
*/
//------------------------------------------------------------------------------
tOplkError pdokcal_sendSyncEvent(void)
{
    tDualprocDrvInstance    pInstance = dualprocshm_getLocalProcDrvInst();
    tDualprocReturn         dualRet;

    if (pInstance == NULL)
    {
        DEBUG_LVL_ERROR_TRACE("%s() couldn't get PCP dual proc driver instance\n",
                              __func__);
        return kErrorNoResource;
    }

    dualRet = dualprocshm_setIrq(pInstance, TARGET_SYNC_INTERRUPT_ID, TRUE);

    if (dualRet != kDualprocSuccessful)
    {
        return kErrorNoResource;
    }

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Enable sync events

The function enables sync events.

\param  fEnable_p               enable/disable sync event

\return The function returns a tOplkError error code.

\ingroup module_pdokcal
*/
//------------------------------------------------------------------------------
tOplkError pdokcal_controlSync(BOOL fEnable_p)
{
    tDualprocDrvInstance    pInstance = dualprocshm_getLocalProcDrvInst();
    tDualprocReturn         dualRet;

    if (pInstance == NULL)
    {
        DEBUG_LVL_ERROR_TRACE("%s() couldn't get PCP dual proc driver instance\n",
                              __func__);
        return kErrorNoResource;
    }

    dualRet = dualprocshm_enableIrq(pInstance, TARGET_SYNC_INTERRUPT_ID, fEnable_p);

    if (dualRet != kDualprocSuccessful)
    {
        return kErrorNoResource;
    }

    return kErrorOk;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

///\}
