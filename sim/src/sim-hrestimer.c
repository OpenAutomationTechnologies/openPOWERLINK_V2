/**
********************************************************************************
\file   sim-hrestimer.c

\brief  Implementation of the simulation interface for hrestimer functions

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
#include <sim-hrestimer.h>
#include <sim.h>

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
\brief Instance struct for sim-hrestimer module

This struct contains information about the current instance.
 */
typedef struct
{
    tHresTimerFunctions     hresTimerFunctions; ///< Struct with all simulation interface functions
    tSimulationInstanceHdl  simHdl;             ///< Handle to running simulation for multiple simulated instances
    BOOL                    fInitialized;       ///< Initialization flag signaling if the stores functions are valid
} tSimHresTimerInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------

/* The function pointers are only accessed after successful initialization,
 * therefore the initialization can be skipped */
static tSimHresTimerInstance    instance_l =
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
\brief Set the function pointer for the hres timer simulation interface

This function sets the function pointer connecting the simulation to the
simulation environment's hres timer functionalities.

\param[in]      simHdl_p                The instance handle defining the current stack instance
\param[in]      hresTimerFunctions_p    Structure containing all connecting function pointers

\return The function returns a BOOL value.
\retval TRUE                            The function pointers are set successfully
                                        to the instance.
\retval FALSE                           The function pointers are not set because
                                        of an error.
 */
//------------------------------------------------------------------------------
BOOL sim_setHresTimerFunctions(tSimulationInstanceHdl simHdl_p,
                               tHresTimerFunctions hresTimerFunctions_p)
{
    if (!instance_l.fInitialized)
    {
        // check function pointer
        if ((hresTimerFunctions_p.pfnInitHresTimer == NULL) ||
            (hresTimerFunctions_p.pfnExitHresTimer == NULL) ||
            (hresTimerFunctions_p.pfnModifyHresTimer == NULL) ||
            (hresTimerFunctions_p.pfnDeleteHresTimer == NULL))
            return FALSE;

        instance_l.hresTimerFunctions = hresTimerFunctions_p;
        instance_l.simHdl = simHdl_p;
        instance_l.fInitialized = TRUE;

        return TRUE;
    }

    return FALSE;
}

//------------------------------------------------------------------------------
/**
\brief Unsets the function pointer for the hres timer simulation interface

This function unsets the function pointer connecting the simulation to the
simulation environment's hres timer functionalities.
 */
//------------------------------------------------------------------------------
void sim_unsetHresTimerFunctions(void)
{
    instance_l.fInitialized = FALSE;
}

//------------------------------------------------------------------------------
/**
\brief Initializes the simulated hres timer module

This function forwards the initialization of the hres timer module to the
connected simulation environment.

\return This functions returns the resulting tOplkError return code.
 */
//------------------------------------------------------------------------------
tOplkError sim_initHresTimer(void)
{
    // check if module was initialized
    if (instance_l.fInitialized)
    {
        // call function
        return instance_l.hresTimerFunctions.pfnInitHresTimer(instance_l.simHdl);
    }

    return kErrorApiNotInitialized;
}

//------------------------------------------------------------------------------
/**
\brief De-initializes the simulated hres timer module

This function forwards the de-initialization of the hres timer module to the
connected simulation environment.

\return This functions returns the resulting tOplkError return code.
 */
//------------------------------------------------------------------------------
tOplkError sim_exitHresTimer(void)
{
    // check if module was initialized
    if (instance_l.fInitialized)
    {
        // call function
        return instance_l.hresTimerFunctions.pfnExitHresTimer(instance_l.simHdl);
    }

    return kErrorApiNotInitialized;
}

//------------------------------------------------------------------------------
/**
\brief Modify a simulated hres timer

This function forwards the modify hres timer command to the connected simulation
environment.

\param[in,out]  pTimerHdl_p         Pointer to timer handle.
\param[in]      time_p              Relative timeout in [ns].
\param[in]      pfnCallback_p       Callback function, which is called when timer expires.
                                    (The function is called mutually exclusive with the Edrv
                                    callback functions (Rx and Tx)).
\param[in]      argument_p          User-specific argument
\param[in]      fContinue_p         If TRUE, the callback function will be called continuously.
                                    Otherwise, it is a one-shot timer.

\return This function returns the resulting tOplkError error code.
 */
//------------------------------------------------------------------------------
tOplkError sim_modifyHresTimer(tTimerHdl* pTimerHdl_p,
                               ULONGLONG time_p,
                               tTimerkCallback pfnCallback_p,
                               ULONG argument_p,
                               BOOL fContinue_p)
{
    // check if module was initialized
    if (instance_l.fInitialized)
    {
        // call function
        return instance_l.hresTimerFunctions.pfnModifyHresTimer(instance_l.simHdl,
                                                                pTimerHdl_p,
                                                                time_p,
                                                                pfnCallback_p,
                                                                argument_p,
                                                                fContinue_p);
    }

    return kErrorApiNotInitialized;
}

//------------------------------------------------------------------------------
/**
\brief Delete a simulated hres timer

This function forwards the delete hres timer command to the connected simulation
environment.

\param[in,out]  pTimerHdl_p         Pointer to timer handle.

\return This function returns the resulting tOplkError error code.
 */
//------------------------------------------------------------------------------
tOplkError sim_deleteHresTimer(tTimerHdl* pTimerHdl_p)
{
    // check if module was initialized
    if (instance_l.fInitialized)
    {
        // call function
        return instance_l.hresTimerFunctions.pfnDeleteHresTimer(instance_l.simHdl,
                                                                pTimerHdl_p);
    }

    return kErrorApiNotInitialized;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

/// \}
