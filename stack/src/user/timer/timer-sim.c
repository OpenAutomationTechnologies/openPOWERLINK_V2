/**
********************************************************************************
\file   timer-sim.c

\brief  Implementation of user timer module using the simulation interface

This file contains an implementation of the user timer module which uses the
interface to a simulation environment implementing the functionality.

\ingroup module_timeru
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

#include <sim-timer.h>

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

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------

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
    return sim_initTimer();
}

//------------------------------------------------------------------------------
/**
\brief  Shutdown user timer

The function shuts down the user timer instance.

\return The function returns a tOplkError error code.

\ingroup module_timeru
*/
//------------------------------------------------------------------------------
tOplkError timeru_exit(void)
{

    return sim_exitTimer();
}

//------------------------------------------------------------------------------
/**
\brief  User timer process function

This function must be called repeatedly from within the application. It checks
whether a timer has expired.

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

\param[out]     pTimerHdl_p         Pointer to store the timer handle.
\param[in]      timeInMs_p          Timeout in milliseconds.
\param[in]      pArgument_p         Pointer to user definable argument for timer.

\return The function returns a tOplkError error code.

\ingroup module_timeru
*/
//------------------------------------------------------------------------------
tOplkError timeru_setTimer(tTimerHdl* pTimerHdl_p,
                           ULONG timeInMs_p,
                           const tTimerArg* pArgument_p)
{
    return sim_setTimer(pTimerHdl_p, timeInMs_p, *pArgument_p);
}

//------------------------------------------------------------------------------
/**
\brief  Modifies an existing timer

This function modifies an existing timer. If the timer was not yet created
it creates the timer and stores the new timer handle at \p pTimerHdl_p.

\param[in,out]  pTimerHdl_p         Pointer to store the timer handle.
\param[in]      timeInMs_p          Timeout in milliseconds.
\param[in]      pArgument_p         Pointer to user definable argument for timer.

\return The function returns a tOplkError error code.

\ingroup module_timeru
*/
//------------------------------------------------------------------------------
tOplkError timeru_modifyTimer(tTimerHdl* pTimerHdl_p,
                              ULONG timeInMs_p,
                              const tTimerArg* pArgument_p)
{

    return sim_modifyTimer(pTimerHdl_p, timeInMs_p, *pArgument_p);
}

//------------------------------------------------------------------------------
/**
\brief  Delete a timer

This function deletes an existing timer.

\param[in,out]  pTimerHdl_p         Pointer to timer handle of timer to delete.

\return The function returns a tOplkError error code.
\retval kErrorTimerInvalidHandle    An invalid timer handle was specified.
\retval kErrorOk                    The timer is deleted.

\ingroup module_timeru
*/
//------------------------------------------------------------------------------
tOplkError timeru_deleteTimer(tTimerHdl* pTimerHdl_p)
{
    return sim_deleteTimer(pTimerHdl_p);
}

//------------------------------------------------------------------------------
/**
\brief  Check for an active timer

This function checks if a timer is active (is running).

\param[in]      timerHdl_p          Handle of timer to check.

\return The function returns TRUE if the timer is active, otherwise FALSE.

\ingroup module_timeru
*/
//------------------------------------------------------------------------------
BOOL timeru_isActive(tTimerHdl timerHdl_p)
{
    return sim_isTimerActive(timerHdl_p);
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

/// \}
