/**
********************************************************************************
\file   pdokcalsync-hostif.c

\brief  Host interface PDO CAL kernel sync module

The sync module is responsible to notify the user layer that new PDO data
could be transfered.

\ingroup module_pdokcal
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
#include <oplk/EplInc.h>
#include <common/pdo.h>
#include <kernel/pdokcal.h>

#include <hostiflib.h>

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

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tHostifInstance pHifInstance_l;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static tEplKernel enableSyncIrq(BOOL fEnable_p);

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Initialize kernel PDO CAL sync module

The function initializes the kernel PDO CAL sync module.

\return The function returns a tEplKernel error code.

\ingroup module_pdokcal
*/
//------------------------------------------------------------------------------
tEplKernel pdokcal_initSync(void)
{
    pHifInstance_l = hostif_getInstance(0);

    if(pHifInstance_l == NULL)
    {
        EPL_DBGLVL_ERROR_TRACE("%s: Could not find hostif instance!\n", __func__);
        return kEplNoResource;
    }

    return enableSyncIrq(FALSE);
}

//------------------------------------------------------------------------------
/**
\brief  Cleanup PDO CAL sync module

The function cleans up the PDO CAL sync module

\ingroup module_pdokcal
*/
//------------------------------------------------------------------------------
void pdokcal_exitSync(void)
{
    if(pHifInstance_l == NULL)
    {
        EPL_DBGLVL_ERROR_TRACE("%s: Could not find hostif instance!\n", __func__);
        return;
    }

    enableSyncIrq(FALSE);
}

//------------------------------------------------------------------------------
/**
\brief  Send a sync event

The function sends a sync event

\return The function returns a tEplKernel error code.

\ingroup module_pdokcal
*/
//------------------------------------------------------------------------------
tEplKernel pdokcal_sendSyncEvent(void)
{
    return kEplSuccessful;
}

//------------------------------------------------------------------------------
/**
\brief  Enable sync events

The function enables sync events

\param  fEnable_p               enable/disable sync event

\return The function returns a tEplKernel error code.

\ingroup module_pdokcal
*/
//------------------------------------------------------------------------------
tEplKernel pdokcal_controlSync(BOOL fEnable_p)
{
    if(pHifInstance_l == NULL)
    {
        EPL_DBGLVL_ERROR_TRACE("%s: Could not find hostif instance!\n", __func__);
        return kEplNoResource;
    }

    return enableSyncIrq(fEnable_p);
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Enable sync interrupt source in host interface ipcore

\param  fEnable_p               enable/disable sync interrupt source

\return The function returns a tEplKernel error code.
*/
//------------------------------------------------------------------------------
static tEplKernel enableSyncIrq(BOOL fEnable_p)
{
    tHostifReturn hifRet;

    hifRet = hostif_irqSourceEnable(pHifInstance_l, kHostifIrqSrcSync, fEnable_p);

    if(hifRet != kHostifSuccessful)
    {
        EPL_DBGLVL_ERROR_TRACE("%s irq not possible (%d)!\n",
                   fEnable_p ? "enable" : "disable", hifRet);
        return kEplNoResource;
    }

    return kEplSuccessful;
}
