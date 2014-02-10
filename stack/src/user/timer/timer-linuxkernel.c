/**
********************************************************************************
\file   timer-linuxkernel.c

\brief  Implementation of user timer module for Linux kernelspace

This file contains the implementation of the user timer module for Linux
kernelspace.

\ingroup module_timeru
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2013, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
Copyright (c) 2013, SYSTEC electronic GmbH
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
#include <user/timeru.h>
#include <linux/timer.h>


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
    struct timer_list   timer;
    tTimerArg           timerArgument;
} tTimeruData;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static void cbTimer(ULONG parameter_p);

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Initialize user timers

The function initializes the user timer module.

\return The function returns a tOplkError error code.

\ingroup module_timeru
*/
//------------------------------------------------------------------------------
tOplkError timeru_init(void)
{
    return timeru_addInstance();
}

//------------------------------------------------------------------------------
/**
\brief  Add user timer instance

The function adds a user timer instance.

\return The function returns a tOplkError error code.

\ingroup module_timeru
*/
//------------------------------------------------------------------------------
tOplkError timeru_addInstance(void)
{
    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Delete user timer instance

The function deletes a user timer instance.

\return The function returns a tOplkError error code.

\ingroup module_timeru
*/
//------------------------------------------------------------------------------
tOplkError timeru_delInstance(void)
{
    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  User timer process function

This function must be called repeatedly from within the application. It checks
whether a timer has expired.

\note The function is not used in the Linux kernelspace implementation!

\return The function returns a tOplkError error code.

\ingroup module_timeru
*/
//------------------------------------------------------------------------------
tOplkError timeru_process(void)
{
    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Create and set a timer

This function creates a timer, sets up the timeout and saves the
corresponding timer handle.

\param  pTimerHdl_p     Pointer to store the timer handle.
\param  timeInMs_p      Timeout in milliseconds.
\param  argument_p      User definable argument for timer.

\return The function returns a tOplkError error code.

\ingroup module_timeru
*/
//------------------------------------------------------------------------------
tOplkError timeru_setTimer(tTimerHdl* pTimerHdl_p, ULONG timeInMs_p, tTimerArg argument_p)
{
    tOplkError          ret = kErrorOk;
    tTimeruData*        pData;

    // check pointer to handle
    if(pTimerHdl_p == NULL)
        return kErrorTimerInvalidHandle;

    pData = (tTimeruData*) EPL_MALLOC(sizeof (tTimeruData));
    if (pData == NULL)
        return kErrorNoResource;

    init_timer(&pData->timer);
    pData->timer.function = cbTimer;
    pData->timer.data = (unsigned long) pData;
    pData->timer.expires = jiffies + 1 + ((timeInMs_p * HZ) + 999) / 1000;

    EPL_MEMCPY(&pData->timerArgument, &argument_p, sizeof(tTimerArg));

    add_timer(&pData->timer);
    *pTimerHdl_p = (tTimerHdl) pData;
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Modifies an existing timer

This function modifies an existing timer. If the timer was not yet created
it creates the timer and stores the new timer handle at \p pTimerHdl_p.

\param  pTimerHdl_p     Pointer to store the timer handle.
\param  timeInMs_p      Timeout in milliseconds.
\param  argument_p      User definable argument for timer.

\return The function returns a tOplkError error code.

\ingroup module_timeru
*/
//------------------------------------------------------------------------------
tOplkError timeru_modifyTimer(tTimerHdl* pTimerHdl_p, ULONG timeInMs_p, tTimerArg argument_p)
{
    tOplkError          ret = kErrorOk;
    tTimeruData*        pData;

    // check pointer to handle
    if(pTimerHdl_p == NULL)
        return kErrorTimerInvalidHandle;

    // check handle itself, i.e. was the handle initialized before
    if (*pTimerHdl_p == 0)
    {
        return timeru_setTimer(pTimerHdl_p, timeInMs_p, argument_p);
    }
    pData = (tTimeruData*) *pTimerHdl_p;
    if ((tTimeruData*)pData->timer.data != pData)
        return kErrorTimerInvalidHandle;

    mod_timer(&pData->timer, (jiffies + 1 + ((timeInMs_p * HZ) + 999) / 1000));

    // copy the TimerArg after the timer is restarted,
    // so that a timer occurred immediately before mod_timer
    // won't use the new TimerArg and
    // therefore the old timer cannot be distinguished from the new one.
    // But if the new timer is too fast, it may get lost.
    EPL_MEMCPY(&pData->timerArgument, &argument_p, sizeof(tTimerArg));

    // check if timer is really running
    if (timer_pending(&pData->timer) == 0)
    {   // timer is not running
        // retry starting it
        add_timer(&pData->timer);
    }
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Delete a timer

This function deletes an existing timer.

\param  pTimerHdl_p     Pointer to timer handle of timer to delete.

\return The function returns a tOplkError error code.
\retval kErrorTimerInvalidHandle  If an invalid timer handle was specified.
\retval kErrorOk          If the timer is deleted.

\ingroup module_timeru
*/
//------------------------------------------------------------------------------
tOplkError timeru_deleteTimer(tTimerHdl* pTimerHdl_p)
{
    tOplkError          ret = kErrorOk;
    tTimeruData*        pData;

    // check pointer to handle
    if(pTimerHdl_p == NULL)
        return kErrorTimerInvalidHandle;

    // check handle itself, i.e. was the handle initialized before
    if (*pTimerHdl_p == 0)
        return kErrorOk;

    pData = (tTimeruData*) *pTimerHdl_p;
    if ((tTimeruData*)pData->timer.data != pData)
        return kErrorTimerInvalidHandle;

    del_timer(&pData->timer);         // try to delete the timer
    kfree(pData);                       // free memory in any case

    *pTimerHdl_p = 0;                   // uninitialize handle
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Check for an active timer

This function checks if a timer is active (is running).

\param  timerHdl_p     Handle of timer to check.

\return The function returns TRUE if the timer is active, otherwise FALSE.

\ingroup module_timeru
*/
//------------------------------------------------------------------------------
BOOL timeru_isActive(tTimerHdl timerHdl_p)
{
    BOOL                fActive = FALSE;
    tTimeruData*        pData;

    // check handle itself, i.e. was the handle initialized before
    if (timerHdl_p == 0)
    {   // timer was not created yet, so it is not active
        return fActive;
    }

    pData = (tTimeruData*) timerHdl_p;
    if ((tTimeruData*)pData->timer.data != pData)
    {   // invalid timer
        return fActive;
    }

    // check if timer is running
    if (timer_pending(&pData->timer) != 0)
    {   // timer is not running
        fActive = TRUE;
    }
    return fActive;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief  Timer callback function

This function is registered if a timer is started and therefore will be called
by the timer when it expires.

\param  parameter_p     The user defined parameter supplied when starting the
                        timer.
*/
//------------------------------------------------------------------------------
static void cbTimer(ULONG parameter_p)
{
    tOplkError          ret = kErrorOk;
    tTimeruData*        pData;
    tEvent              event;
    tTimerEventArg      timerEventArg;

    pData = (tTimeruData*) parameter_p;

    // call event function
    timerEventArg.timerHdl = (tTimerHdl)pData;
    EPL_MEMCPY(&timerEventArg.argument, &pData->timerArgument.argument, sizeof (timerEventArg.argument));

    event.eventSink = pData->timerArgument.eventSink;
    event.eventType = kEventTypeTimer;
    EPL_MEMSET(&event.netTime, 0x00, sizeof(tEplNetTime));
    event.pEventArg = &timerEventArg;
    event.eventArgSize = sizeof(timerEventArg);

    ret = eventu_postEvent(&event);
    // d.k. do not free memory, user has to call timeru_deleteTimer()
}

///\}
