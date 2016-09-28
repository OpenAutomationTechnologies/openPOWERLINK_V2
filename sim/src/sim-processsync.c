/**
********************************************************************************
\file   sim-processsync.c

\brief  Implementation of the simulation interface for process sync functions

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

#include <sim-processsync.h>

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
\brief  Instance struct for sim-processsync module

This struct contains informations about the current instance.
*/
typedef struct
{
    tProcessSyncFunctions   processSyncFunctions;   ///< Struct with all simulation interface functions
    tSimulationInstanceHdl  simHdl;                 ///< Handle to running simulation for multiple simulated instances
    BOOL                    fInitialized;           ///< Initialization flag signalling if the stores functions are valid
} tSimApiInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------

/* The function pointers are only accessed after successful initialization,
 * therefore the initialization can be skipped */
static tSimApiInstance instance_l = { .simHdl = 0, .fInitialized = FALSE };

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Setter for simulation interface functions for process sync

This function sets the function pointer connecting the simulation to the
simulation environment's process sync functionalities.

\param simHdl_p                 The handle of the current simulated stack instance
\param processSyncFunctions_p   Structure with all simulation interface functions

\return The function returns a BOOL value.
\retval TRUE    The function pointers are set successfully to the instance.
\retval FALSE   The function pointers are not set because of an error.
*/
//------------------------------------------------------------------------------
BOOL sim_setProcessSyncFunctions(tSimulationInstanceHdl simHdl_p,
                                 tProcessSyncFunctions processSyncFunctions_p)
{
    if (!instance_l.fInitialized)
    {
        // check function pointers
        if (processSyncFunctions_p.pfnCbProcessSync == NULL)
            return FALSE;

        instance_l.processSyncFunctions = processSyncFunctions_p;
        instance_l.simHdl = simHdl_p;
        instance_l.fInitialized = TRUE;

        return TRUE;
    }

    return FALSE;
}

//------------------------------------------------------------------------------
/**
\brief Unsets the function pointer for the process sync simulation interface

This function unsets the function pointer connecting the simulation to the
simulation environment's process sync functionalities.
 */
//------------------------------------------------------------------------------
void sim_unsetProcessSyncFunctions(void)
{
    instance_l.fInitialized = FALSE;
}

//------------------------------------------------------------------------------
/**
\brief Process sync callback connecting to simulated process sync module

This callback forwards the process sync call to the connected simulation
environment.

\return This functions returns the resulting tOplkError return code.
 */
//------------------------------------------------------------------------------
tOplkError sim_processSyncCb(void)
{
    // check module initialization
    if (instance_l.fInitialized)
    {
        // call function pointer
        return instance_l.processSyncFunctions.pfnCbProcessSync(instance_l.simHdl);
    }

    return kErrorApiNotInitialized;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

/// \}
