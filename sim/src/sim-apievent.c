/**
********************************************************************************
\file   sim-apievent.c

\brief  Implementation of the simulation interface for API event functions

\ingroup module_sim
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2016, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
Copyright (c) 2016, Franz Profelt (franz.profelt@gmail.com)
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
#include <sim-apievent.h>

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
\brief  Instance struct for sim-apievent module

This struct contains information about the current instance.
*/
typedef struct
{
    tApiEventFunctions      eventFunctions;     ///< Struct with all simulation interface functions
    tSimulationInstanceHdl  simHdl;             ///< Handle to running simulation for multiple simulated instances
    BOOL                    fInitialized;       ///< Initialization flag signaling if the stored functions are valid
} tSimApiInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------

/* The function pointers are only accessed after successful initialization,
 * therefore the initialization can be skipped */
static tSimApiInstance  instance_l =
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
\brief  Setter for API event simulation interface functions

This function sets the function pointer connecting the simulation to the
simulation environment's API event functionalities.

\param[in]      simHdl_p            The handle of the currently simulated stack instance
\param[in]      eventFunctions_p    Structure with all simulation interface functions

\return The function returns a BOOL value.
\retval TRUE                        The function pointers are set successfully to
                                    the instance.
\retval FALSE                       The function pointers are not set because of
                                    an error.
*/
//------------------------------------------------------------------------------
BOOL sim_setApiEventFunctions(tSimulationInstanceHdl simHdl_p,
                              tApiEventFunctions eventFunctions_p)
{
    if (!instance_l.fInitialized)
    {
        // check function pointers
        if (eventFunctions_p.pfnCbEvent == NULL)
            return FALSE;

        instance_l.eventFunctions = eventFunctions_p;
        instance_l.simHdl = simHdl_p;
        instance_l.fInitialized = TRUE;

        return TRUE;
    }

    return FALSE;
}

//------------------------------------------------------------------------------
/**
\brief Unset API event simulation interface function pointers

This function unsets the function pointer connecting the simulation to the
simulation environment's API event functionalities.
 */
//------------------------------------------------------------------------------
void sim_unsetApiEventFunctions(void)
{
    instance_l.fInitialized = FALSE;
}

//------------------------------------------------------------------------------
/**
\brief API event callback connecting to simulated API event module

This callback forwards the API event callback to the connected simulation
environment.

\param[in]      eventType_p         The type of the event
\param[in]      pEventArg_p         Pointer to the event argument
\param[in]      pUserArg_p          Pointer to the user defined argument

\return This functions returns the resulting tOplkError return code.
 */
//------------------------------------------------------------------------------
tOplkError sim_eventCb(tOplkApiEventType eventType_p,
                       const tOplkApiEventArg* pEventArg_p,
                       void* pUserArg_p)
{
    // check module initialization
    if (instance_l.fInitialized)
    {
        // call function pointer
        return instance_l.eventFunctions.pfnCbEvent(instance_l.simHdl,
                                                    eventType_p,
                                                    pEventArg_p,
                                                    pUserArg_p);
    }

    return kErrorApiNotInitialized;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{


/// \}
