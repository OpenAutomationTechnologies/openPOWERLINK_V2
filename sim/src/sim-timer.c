/**
********************************************************************************
\file   sim-timer.c

\brief  Implementation of the simulation interface for user timer functions

\ingroup module_sim
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2016, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
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
#include <sim-timer.h>
#include <user/eventu.h>

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
\brief Instance struct for sim-timer module

This struct contains information about the current instance.
 */
typedef struct
{
    tTimerFunctions         timerFunctions; ///< Struct with all simulation interface functions
    tSimulationInstanceHdl  simHdl;         ///< Handle to running simulation for multiple simulated instances
    BOOL                    fInitialized;   ///< Initialization flag signaling if the stores functions are valid
} tSimTimerInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------

/* The function pointers are only accessed after successful initialization,
 * therefore the initialization can be skipped */
static tSimTimerInstance    instance_l =
{
    .simHdl = 0,
    .fInitialized = FALSE
};

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief Set the function pointer for the user timer simulation interface

This function sets the function pointer connecting the simulation to the
simulation environment's user timer functionalities.

\param[in]      simHdl_p            The instance handle defining the current stack instance
\param[in]      timerFunctions_p    Structure containing all connecting function pointers

\return BOOL value showing the success of the initialization of the simulation
    interface
 */
//------------------------------------------------------------------------------
BOOL sim_setTimerFunctions(tSimulationInstanceHdl simHdl_p,
                           tTimerFunctions timerFunctions_p)
{
    if (!instance_l.fInitialized)
    {
        // check function pointer
        if ((timerFunctions_p.pfnInitTimer == NULL) ||
            (timerFunctions_p.pfnExitTimer == NULL) ||
            (timerFunctions_p.pfnSetTimer == NULL) ||
            (timerFunctions_p.pfnModifyTimer == NULL) ||
            (timerFunctions_p.pfnDeleteTimer == NULL) ||
            (timerFunctions_p.pfnIsTimerActive == NULL))
            return FALSE;

        instance_l.timerFunctions = timerFunctions_p;
        instance_l.simHdl = simHdl_p;
        instance_l.fInitialized = TRUE;

        return TRUE;
    }

    return FALSE;
}

//------------------------------------------------------------------------------
/**
\brief Unsets the function pointer for the user timer simulation interface

This function unsets the function pointer connecting the simulation to the
simulation environment's user timer functionalities.
 */
//------------------------------------------------------------------------------
void sim_unsetTimerFunctions(void)
{
    instance_l.fInitialized = FALSE;
}

//------------------------------------------------------------------------------
/**
\brief Callback function for user timer

This callback function provides the connection for the expiration of a simulated
user timer within the connected simulation environment.

\param[in]      timerHdl_p          Handle of timer which has expired
\param[in]      argument_p          Timer argument passed to timer
 */
//------------------------------------------------------------------------------
void sim_userTimerCallback(tTimerHdl timerHdl_p,
                           tTimerArg argument_p)
{
    tEvent          event;
    tTimerEventArg  timerEventArg;

    // call event function
    timerEventArg.timerHdl.handle = timerHdl_p;
    OPLK_MEMCPY(&timerEventArg.argument,
                &argument_p.argument,
                sizeof(timerEventArg.argument));

    event.eventSink = argument_p.eventSink;
    event.eventType = kEventTypeTimer;
    OPLK_MEMSET(&event.netTime, 0x00, sizeof(tNetTime));
    event.eventArg.pEventArg = &timerEventArg;
    event.eventArgSize = sizeof(timerEventArg);

    eventu_postEvent(&event);
}

//------------------------------------------------------------------------------
/**
\brief Initializes the simulated user timer module

This function forwards the initialization of the user timer module to the
connected simulation environment.

\return This functions returns the resulting tOplkError return code.
 */
//------------------------------------------------------------------------------
tOplkError sim_initTimer(void)
{
    // check if module was initialized
    if (instance_l.fInitialized)
    {
        // call function
        return instance_l.timerFunctions.pfnInitTimer(instance_l.simHdl);
    }

    return kErrorApiNotInitialized;
}

//------------------------------------------------------------------------------
/**
\brief Denitializes the simulated user timer module

This function forwards the deinitialization of the user timer module to the
connected simulation environment.

\return This functions returns the resulting tOplkError return code.
 */
//------------------------------------------------------------------------------
tOplkError sim_exitTimer(void)
{
    // check if module was initialized
    if (instance_l.fInitialized)
    {
        // call function
        return instance_l.timerFunctions.pfnExitTimer(instance_l.simHdl);
    }

    return kErrorApiNotInitialized;
}

//------------------------------------------------------------------------------
/**
\brief Set a simulated user timer

This function forwards the set user timer command to the connected simulation
environment.

\param[out]     pTimerHdl_p         Pointer to store the timer handle
\param[in]      timeInMs_p          Timeout in milliseconds
\param[in]      argument_p          User definable argument for timer

\return This functions returns the resulting tOplkError return code.
 */
//------------------------------------------------------------------------------
tOplkError sim_setTimer(tTimerHdl* pTimerHdl_p,
                        ULONG timeInMs_p,
                        tTimerArg argument_p)
{
    // check if module was initialized
    if (instance_l.fInitialized)
    {
        // call function
        return instance_l.timerFunctions.pfnSetTimer(instance_l.simHdl,
                                                     pTimerHdl_p,
                                                     timeInMs_p,
                                                     argument_p);
    }

    return kErrorApiNotInitialized;
}

//------------------------------------------------------------------------------
/**
\brief Modify an simulated user timer

This function forwards the modify user timer command to the connected simulation
environment.

\param[out]     pTimerHdl_p         Pointer to store the timer handle
\param[in]      timeInMs_p          Timeout in milliseconds
\param[in]      argument_p          User definable argument for timer

\return This functions returns the resulting tOplkError return code.
 */
//------------------------------------------------------------------------------
tOplkError sim_modifyTimer(tTimerHdl* pTimerHdl_p,
                           ULONG timeInMs_p,
                           tTimerArg argument_p)
{
    // check if module was initialized
    if (instance_l.fInitialized)
    {
        // call function
        return instance_l.timerFunctions.pfnModifyTimer(instance_l.simHdl,
                                                        pTimerHdl_p,
                                                        timeInMs_p,
                                                        argument_p);
    }

    return kErrorApiNotInitialized;
}

//------------------------------------------------------------------------------
/**
\brief Delete an simulated user timer

This function forwards the delete user timer command to the connected simulation
environment.

\param[in,out]  pTimerHdl_p         Pointer to timer handle of timer to delete

\return This functions returns the resulting tOplkError return code.
 */
//------------------------------------------------------------------------------
tOplkError sim_deleteTimer(tTimerHdl* pTimerHdl_p)
{
    // check if module was initialized
    if (instance_l.fInitialized)
    {
        // call function
        return instance_l.timerFunctions.pfnDeleteTimer(instance_l.simHdl,
                                                        pTimerHdl_p);
    }

    return kErrorApiNotInitialized;
}

//------------------------------------------------------------------------------
/**
\brief Check for an active simulated user timer

This function forwards the is user timer active command to the connected
simulation environment.

\param[in]      timerHdl_p          Handle of timer to check

\return This functions returns the resulting tOplkError return code.
 */
//------------------------------------------------------------------------------
BOOL sim_isTimerActive(tTimerHdl timerHdl_p)
{
    // check if module was initialized
    if (instance_l.fInitialized)
    {
        // call function
        return instance_l.timerFunctions.pfnIsTimerActive(instance_l.simHdl,
                                                          timerHdl_p);
    }

    return FALSE;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

/// \}
