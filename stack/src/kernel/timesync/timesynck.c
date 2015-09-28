/**
********************************************************************************
\file   timesynck.c

\brief  Kernel timesync module

This file contains the main implementation of the kernel timesync module.

\ingroup module_timesynck
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2015, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
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
#include <kernel/timesynck.h>
#include <kernel/timesynckcal.h>

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
/**
\brief Timesync instance

The following structure defines the instance variable of the kernel timesync module.
*/
typedef struct
{
    UINT32      syncEventCycle;     ///< Synchronization event cycle
} tTimesynckInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tTimesynckInstance   timesynckInstance_l;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Initialize kernel timesync module

The function initializes the kernel timesync module.

\return The function returns a tOplkError error code.

\ingroup module_timesynck
*/
//------------------------------------------------------------------------------
tOplkError timesynck_init(void)
{
    OPLK_MEMSET(&timesynckInstance_l, 0, sizeof(timesynckInstance_l));

    timesynckInstance_l.syncEventCycle = 1; // Default every cycle

    return timesynckcal_init();
}

//------------------------------------------------------------------------------
/**
\brief  Cleanup timesync module

The function cleans up the timesync module.

\ingroup module_timesynck
*/
//------------------------------------------------------------------------------
void timesynck_exit(void)
{
    timesynckcal_exit();
}

//------------------------------------------------------------------------------
/**
\brief  Set cycle time to timesync module

The function sets the POWERLINK cycle time to the timesync module

\param  cycleLen_p      POWERLINK Cycle time [us]
\param  minSyncTime_p   Minimum period for sending sync event [us]

\return The function returns a tOplkError error code.

\ingroup module_timesynck
*/
//------------------------------------------------------------------------------
tOplkError timesynck_setCycleTime(UINT32 cycleLen_p, UINT32 minSyncTime_p)
{
    tOplkError  ret = kErrorOk;

    if ((cycleLen_p == 0) || (minSyncTime_p == 0))
    {
        // - Handle a cycle time of 0 (avoids div by 0)
        // - Handle not configured minimum sync period
        timesynckInstance_l.syncEventCycle = 1;
    }
    else
    {
        // Calculate synchronization event cycle
        timesynckInstance_l.syncEventCycle = ((minSyncTime_p + cycleLen_p - 1) / cycleLen_p);
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Send sync event

The function sends a synchronization event to the user layer.

\return The function returns a tOplkError error code.

\ingroup module_timesynck
*/
//------------------------------------------------------------------------------
tOplkError timesynck_sendSyncEvent(void)
{
    tOplkError      ret = kErrorOk;
    static UINT32   cycleCnt = 0;

    if ((++cycleCnt == timesynckInstance_l.syncEventCycle))
    {
        ret = timesynckcal_sendSyncEvent();

        cycleCnt = 0;
    }
    else if (cycleCnt > timesynckInstance_l.syncEventCycle)
    {
        cycleCnt = 0;
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Process events for timesync

The function processes events intended for the kernel timesync module.

\param  pEvent_p        Pointer to event

\return The function returns a tOplkError error code.

\ingroup module_timesynck
*/
//------------------------------------------------------------------------------
tOplkError timesynck_process(tEvent* pEvent_p)
{
    tOplkError ret = kErrorOk;

    switch (pEvent_p->eventType)
    {
        case kEventTypeTimesynckControl:
            ret = timesynckcal_controlSync(*((BOOL*)pEvent_p->eventArg.pEventArg));
            break;

        default:
            ret = kErrorInvalidEvent;
            break;
    }

    return ret;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

/// \}
