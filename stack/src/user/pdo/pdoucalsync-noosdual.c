/**
********************************************************************************
\file   pdoucalsync-noosdual.c

\brief  Dual Processor PDO CAL user sync module

This file contains the dualprocshm sync implementation for the PDO user CAL
module.

\ingroup module_pdoucal
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2014 Kalycito Infotech Private Limited
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
#include <user/pdoucal.h>

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
static tSyncCb    pfnSyncCb_l = NULL;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static void irqSyncCb(void);

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Initialize PDO user CAL sync module

The function initializes the PDO user CAL sync module

\param  pfnSyncCb_p             Function that is called in case of sync event

\return The function returns a tOplkError error code.

\ingroup module_pdoucal
*/
//------------------------------------------------------------------------------
tOplkError pdoucal_initSync(tSyncCb pfnSyncCb_p)
{
    tDualprocReturn         dualRet;
    tDualprocDrvInstance    pInstance = dualprocshm_getLocalProcDrvInst();

    if (pInstance == NULL)
    {
        DEBUG_LVL_ERROR_TRACE("%s() couldn't get Host dual proc driver instance\n",
                              __func__);
        return kErrorNoResource;
    }

    dualRet = dualprocshm_registerHandler(pInstance, TARGET_SYNC_INTERRUPT_ID, irqSyncCb);
    if (dualRet != kDualprocSuccessful)
    {
        DEBUG_LVL_ERROR_TRACE("%s: Enable irq not possible!\n", __func__);
        return kErrorNoResource;
    }

    pfnSyncCb_l = pfnSyncCb_p;

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Cleanup PDO user CAL sync module

The function cleans up the PDO user CAL sync module

\ingroup module_pdoucal
*/
//------------------------------------------------------------------------------
void pdoucal_exitSync(void)
{
    tDualprocDrvInstance    pInstance = dualprocshm_getLocalProcDrvInst();

    if (pInstance == NULL)
    {
        DEBUG_LVL_ERROR_TRACE("%s() couldn't get Host dual proc driver instance\n",
                              __func__);
        return;
    }

    dualprocshm_registerHandler(pInstance, TARGET_SYNC_INTERRUPT_ID, NULL);
}

//------------------------------------------------------------------------------
/**
\brief  Wait for a sync event

The function waits for a sync event.

\param  timeout_p       Specifies a timeout in microseconds. If 0 it waits
                        forever.

\return The function returns a tOplkError error code.
\retval kErrorOk              Successfully received sync event
\retval kErrorGeneralError    Error while waiting on sync event

\ingroup module_pdoucal
*/
//------------------------------------------------------------------------------
tOplkError pdoucal_waitSyncEvent(ULONG timeout_p)
{
    UNUSED_PARAMETER(timeout_p);
    return kErrorOk;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{
//------------------------------------------------------------------------------
/**
\brief  Synchronization callback

This function is called for synchronization by the dualprocshm instance.

*/
//------------------------------------------------------------------------------
static void irqSyncCb(void)
{
    pfnSyncCb_l();
}

///\}
