/**
********************************************************************************
\file   sim-trace.c

\brief  Implementation of the simulation interface for trace functions

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
#include <sim-trace.h>

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
\brief Instance struct for sim-trace module

This struct contains information about the current instance.
 */
typedef struct
{
    tTraceFunctions         traceFunctions;     ///< Struct with all simulation interface functions
    tSimulationInstanceHdl  simHdl;             ///< Handle to running simulation for multiple simulated instances
    BOOL                    fInitialized;       ///< Initialization flag signaling if the stores functions are valid
} tSimTraceInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------

/* The function pointers are only accessed after successful initialization,
 * therefore the initialization can be skipped */
static tSimTraceInstance    instance_l =
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
\brief Set the function pointer for the trace simulation interface

This function sets the function pointer connecting the simulation to the
simulation environment's trace functionalities.

\param[in]      simHdl_p            The instance handle defining the current stack instance
\param[in]      traceFunctions_p    Structure containing all connecting function pointers

\return BOOL value showing the success of the initialization of the simulation interface
 */
//------------------------------------------------------------------------------
BOOL sim_setTraceFunctions(tSimulationInstanceHdl simHdl_p,
                           tTraceFunctions traceFunctions_p)
{
    if (!instance_l.fInitialized)
    {
        // check function pointer
        if (traceFunctions_p.pfnTrace == NULL)
            return FALSE;

        instance_l.traceFunctions = traceFunctions_p;
        instance_l.simHdl = simHdl_p;
        instance_l.fInitialized = TRUE;

        return TRUE;
    }

    return FALSE;
}

//------------------------------------------------------------------------------
/**
\brief Unsets the function pointer for the trace simulation interface

This function unsets the function pointer connecting the simulation to the
simulation environment's trace functionalities.
 */
//------------------------------------------------------------------------------
void sim_unsetTraceFunctions(void)
{
    instance_l.fInitialized = FALSE;
}

//------------------------------------------------------------------------------
/**
\brief Trace the passed message in the simulation environment.

This function forwards the trace command to the connected simulation
environment.

\param[in]      pMsg_p              Trace message

*/
//------------------------------------------------------------------------------
void sim_trace(const char* pMsg_p)
{
    // check if module was initialized
    if (instance_l.fInitialized)
    {
        // call function
        instance_l.traceFunctions.pfnTrace(instance_l.simHdl, pMsg_p);
    }
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

/// \}
