/**
********************************************************************************
\file   timesynckcal-winkernel.c

\brief  Windows kernel driver timesync module

This file contains the implementation for kernel CAL timesync module
for Windows kernel. The timesync module is responsible to synchronize
data exchange with user layer.

The module uses NDIS events to get notification of new data and pass it to the
user layer by completing pending IOCTLs.

\ingroup module_timesynckcal
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2015, Kalycito Infotech Private Limited
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
#include <kernel/timesynckcal.h>

#include <ndisintermediate/ndis-im.h>
#include <ndis.h>

//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#define TIMERSYNCK_TAG    'knsT'

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
/**
/brief  Timesync module instance structure

Local variables and flags used by timesync module.

*/
typedef struct
{
    NDIS_EVENT        syncWaitEvent;    ///< NDIS event for synchronization events.
    BOOL              fInitialized;     ///< Flag to identify initialization status of module.
} tTimesynckCalInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tTimesynckCalInstance*   pInstance_l;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Initialize kernel CAL timesync module

The function initializes the kernel CAL timesync module.

\return The function returns a tOplkError error code.

\ingroup module_timesynckcal
*/
//------------------------------------------------------------------------------
tOplkError timesynckcal_init(void)
{
    NDIS_HANDLE    adapterHandle = ndis_getAdapterHandle();

    pInstance_l = NdisAllocateMemoryWithTagPriority(adapterHandle, sizeof(tTimesynckCalInstance),
                                                   TIMERSYNCK_TAG, NormalPoolPriority);

    if (pInstance_l == NULL)
        return kErrorNoResource;

    NdisInitializeEvent(&pInstance_l->syncWaitEvent);

    pInstance_l->fInitialized = TRUE;

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Clean up CAL timesync module

The function cleans up the CAL timesync module.

\ingroup module_timesynckcal
*/
//------------------------------------------------------------------------------
void timesynckcal_exit(void)
{
    if (pInstance_l != NULL)
    {
        NdisFreeMemory(pInstance_l, 0, 0);
        pInstance_l->fInitialized = FALSE;
    }
}

//------------------------------------------------------------------------------
/**
\brief  Send a sync event

The function sends a sync event.

\return The function returns a tOplkError error code.

\ingroup module_timesynckcal
*/
//------------------------------------------------------------------------------
tOplkError timesynckcal_sendSyncEvent(void)
{
    // For kernel implementation polling with wait time is used instead of
    // signal/event for synchronization and data exchange. So no sync event
    // exchange is required. \refer timesynckcal_waitSyncEvent

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Wait for a sync event

The function waits for a sync event

\return The function returns a tOplkError error code.

\ingroup module_timesynckcal
*/
//------------------------------------------------------------------------------
tOplkError timesynckcal_waitSyncEvent(void)
{
    INT        timeout = 1000;
    BOOLEAN    fRet;

    if (!pInstance_l->fInitialized)
        return kErrorNoResource;

    fRet = NdisWaitEvent(&pInstance_l->syncWaitEvent, timeout);

    if (fRet)
    {
        NdisResetEvent(&pInstance_l->syncWaitEvent);
    }
    else
    {
        return kErrorRetry;
    }

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Enable sync events

The function enables sync events.

\param  fEnable_p               Enable/disable sync event.

\return The function returns a tOplkError error code.

\ingroup module_timesynckcal
*/
//------------------------------------------------------------------------------
tOplkError timesynckcal_controlSync(BOOL fEnable_p)
{
    UNUSED_PARAMETER(fEnable_p);
    return kErrorOk;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

/// \}
