/**
********************************************************************************
\file   eventucal-nooscircbuf.c

\brief  User event CAL module for non-OS platform using circbuffer

This file implements the user event handler CAL module for a non-OS
platform. It uses the circular buffer interface for the kernel-to-user
event queue and direct calls for the user-internal queue.

\see eventucalintf-circbuf.c

\ingroup module_eventucal
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2016, B&R Industrial Automation GmbH
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
#include <user/eventucal.h>
#include <user/eventucalintf.h>
#include <user/eventu.h>
#include <kernel/eventk.h>
#include <common/target.h>

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
\brief User event CAL instance type

The structure contains all necessary information needed by the user event
CAL module.
*/
typedef struct
{
    BOOL                    fInitialized;
} tEventuCalInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tEventuCalInstance   instance_l;             ///< Instance variable of user event CAL module

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief    Initialize architecture specific stuff of user event CAL module

The function initializes the architecture specific stuff of the user event
CAL module.

\return The function returns a tOplkError error code.
\retval kErrorOk                    Function executes correctly
\retval other error codes           An error occurred

\ingroup module_eventucal
*/
//------------------------------------------------------------------------------
tOplkError eventucal_init(void)
{
    OPLK_MEMSET(&instance_l, 0, sizeof(tEventuCalInstance));

    if (eventucal_initQueueCircbuf(kEventQueueK2U) != kErrorOk)
        goto Exit;

    instance_l.fInitialized = TRUE;
    return kErrorOk;

Exit:
    eventucal_exitQueueCircbuf(kEventQueueK2U);
    return kErrorNoResource;
}

//------------------------------------------------------------------------------
/**
\brief    Clean up kernel event CAL module

The function cleans up the kernel event CAL module. For cleanup it calls the exit
functions of the queue implementations for each used queue.

\return The function returns a tOplkError error code.
\retval kErrorOk                    Function executes correctly
\retval other error codes           An error occurred

\ingroup module_eventucal
*/
//------------------------------------------------------------------------------
tOplkError eventucal_exit(void)
{
    if (instance_l.fInitialized != FALSE)
        eventucal_exitQueueCircbuf(kEventQueueK2U);

    instance_l.fInitialized = FALSE;

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief    Post kernel event

This function posts an event to a queue. It is called from the generic kernel
event post function in the event handler. Depending on the sink the appropriate
queue post function is called.

\param[in]      pEvent_p            Event to be posted.

\return The function returns a tOplkError error code.
\retval kErrorOk                    Function executes correctly
\retval other error codes           An error occurred

\ingroup module_eventucal
*/
//------------------------------------------------------------------------------
tOplkError eventucal_postKernelEvent(const tEvent* pEvent_p)
{
    tOplkError  ret;

    // Check parameter validity
    ASSERT(pEvent_p != NULL);

    target_enableGlobalInterrupt(FALSE);

    ret = eventk_process(pEvent_p);

    target_enableGlobalInterrupt(TRUE);

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief    Post user event

This function posts an event to a queue. It is called from the generic kernel
event post function in the event handler. Depending on the sink the appropriate
queue post function is called.

\param[in]      pEvent_p            Event to be posted.

\return The function returns a tOplkError error code.
\retval kErrorOk                    Function executes correctly
\retval other error codes           An error occurred

\ingroup module_eventucal
*/
//------------------------------------------------------------------------------
tOplkError eventucal_postUserEvent(const tEvent* pEvent_p)
{
    tOplkError  ret;

    // Check parameter validity
    ASSERT(pEvent_p != NULL);

    ret = eventu_process(pEvent_p);

    return ret;
}


//------------------------------------------------------------------------------
/**
\brief  Process function of user CAL module

This function will be called by the systems process function.

\ingroup module_eventucal
*/
//------------------------------------------------------------------------------
void eventucal_process(void)
{
    if (eventucal_getEventCountCircbuf(kEventQueueK2U) > 0)
        eventucal_processEventCircbuf(kEventQueueK2U);
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

/// \}
