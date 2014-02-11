/**
********************************************************************************
\file   pdokcalsync-linuxkernel.c

\brief  PDO CAL kernel sync module using the openPOWERLINK Linux kernel driver

This file contains an implementation for the kernel PDO CAL sync module which
uses the openPOWERLINK Linux kernel driver interface..

The sync module is responsible to notify the user layer that new PDO data
could be transfered.

\ingroup module_pdokcal
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2013, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
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

#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/wait.h>

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
    wait_queue_head_t       syncWaitQueue;
    BOOL                    fSync;
    BOOL                    fInitialized;
} tPdokCalSyncInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tPdokCalSyncInstance     instance_l;             ///< Instance variable of kernel PDOKCAL sync module

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
    OPLK_MEMSET(&instance_l, 0, sizeof(tPdokCalSyncInstance));

    init_waitqueue_head(&instance_l.syncWaitQueue);
    instance_l.fInitialized = TRUE;

    return kErrorOk;
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
    instance_l.fInitialized = FALSE;
}

//------------------------------------------------------------------------------
/**
\brief  Send a sync event

The function sends a sync event

\return The function returns a tOplkError error code.

\ingroup module_pdokcal
*/
//------------------------------------------------------------------------------
tOplkError pdokcal_sendSyncEvent(void)
{
    if (instance_l.fInitialized)
    {
        instance_l.fSync = TRUE;
        wake_up_interruptible(&instance_l.syncWaitQueue);
    }
    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Wait for a sync event

The function waits for a sync event

\return The function returns a tOplkError error code.

\ingroup module_pdokcal
*/
//------------------------------------------------------------------------------
tOplkError pdokcal_waitSyncEvent(void)
{
    int                 ret;
    int                 timeout = 1000 * HZ / 1000;

    if (!instance_l.fInitialized)
        return kErrorNoResource;

    ret = wait_event_interruptible_timeout(instance_l.syncWaitQueue,
                                           instance_l.fSync == TRUE, timeout);
    if (ret == 0)
        return kErrorRetry;

    instance_l.fSync = FALSE;
    return kErrorOk;
}


//------------------------------------------------------------------------------
/**
\brief  Enable sync events

The function enables sync events

\param  fEnable_p               enable/disable sync event

\return The function returns a tOplkError error code.

\ingroup module_pdokcal
*/
//------------------------------------------------------------------------------
tOplkError pdokcal_controlSync(BOOL fEnable_p)
{
    UNUSED_PARAMETER(fEnable_p);
    return kErrorOk;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
