/**
********************************************************************************
\file   sim-target.c

\brief  Stub implementation of target specific functions for simulation

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

#include <sim-target.h>

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
\brief Instance struct for sim-target module

This struct contains information about the current instance.
 */
typedef struct
{
    tTargetFunctions        targetFunctions;    ///< Struct with all simulation interface functions
    tSimulationInstanceHdl  simHdl;             ///< Handle to running simulation for multiple simulated instances
    BOOL                    fInitialized;       ///< Initialization flag signalling if the stores functions are valid
} tSimTargetInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------

/* The function pointers are only accessed after successful initialization,
 * therefore the initialization can be skipped */
static tSimTargetInstance instance_l = { .simHdl = 0, .fInitialized = FALSE };

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief Set the function pointer for the target simulation interface

This function sets the function pointer connecting the simulation to the
simulation environment's target functionalities.

\param  simHdl_p            The instance handle defining the current stack instance
\param  targetFunctions_p   Structure containing all connecting function pointers

\return BOOL value showing the success of the initialization of the simulation interface
 */
//------------------------------------------------------------------------------
BOOL sim_setTargetFunctions(tSimulationInstanceHdl simHdl_p,
                            tTargetFunctions targetFunctions_p)
{
    if (!instance_l.fInitialized)
    {
        // check given functions
        if ((targetFunctions_p.pfnInit == NULL) ||
            (targetFunctions_p.pfnExit == NULL) ||
            (targetFunctions_p.pfnMsleep == NULL) ||
            (targetFunctions_p.pfnSetIp == NULL) ||
            (targetFunctions_p.pfnSetDefaultGateway == NULL) ||
            (targetFunctions_p.pfnGetTick == NULL) ||
            (targetFunctions_p.pfnSetLed == NULL))
            return FALSE;

        // set functions
        instance_l.targetFunctions = targetFunctions_p;
        instance_l.simHdl = simHdl_p;
        instance_l.fInitialized = TRUE;

        return TRUE;
    }

    return FALSE;
}

//------------------------------------------------------------------------------
/**
\brief Unsets the function pointer for the target simulation interface

This function unsets the function pointer connecting the simulation to the
simulation environment's target functionalities.
 */
//------------------------------------------------------------------------------
void sim_unsetTargetFunctions(void)
{
    instance_l.fInitialized = FALSE;
}

//------------------------------------------------------------------------------
/**
\brief Initializes the simulated target module

This function forwards the initialization of the target module to the connected
simulation environment.

\return This functions returns the resulting tOplkError return code.
 */
//------------------------------------------------------------------------------
tOplkError sim_initTarget(void)
{
    // check if module was initialized
    if (instance_l.fInitialized)
    {
        // call function
        return instance_l.targetFunctions.pfnInit(instance_l.simHdl);
    }

    return kErrorApiNotInitialized;
}

//------------------------------------------------------------------------------
/**
\brief Denitializes the simulated target module

This function forwards the deinitialization of the target module to the
connected simulation environment.

\return This functions returns the resulting tOplkError return code
 */
//------------------------------------------------------------------------------
tOplkError sim_exitTarget(void)
{
    // check if module was initialized
    if (instance_l.fInitialized)
    {
        // call function
        return instance_l.targetFunctions.pfnExit(instance_l.simHdl);
    }

    return kErrorApiNotInitialized;
}

//------------------------------------------------------------------------------
/**
\brief Sets the simulation to sleep for the given amount of time

This function forwards the sleep command to the connected simulation
environment.

\param  milliSeconds_p  Time to sleep in milliseconds
 */
//------------------------------------------------------------------------------
void sim_msleep(UINT32 milliSeconds_p)
{
    // check if module was initialized
    if (instance_l.fInitialized)
    {
        // call function
        instance_l.targetFunctions.pfnMsleep(instance_l.simHdl, milliSeconds_p);
    }
}

//------------------------------------------------------------------------------
/**
\brief Sets the simulated IP address

This function forwards the set IP address command to the connected simulation
environment.

\param  ifName_p                Name of ethernet interface
\param  ipAddress_p             IP address to set for interface
\param  subnetMask_p            Subnet mask to set for interface
\param  mtu_p                   MTU to set for interface

\return This functions returns the resulting tOplkError return code
 */
//------------------------------------------------------------------------------
tOplkError sim_setIpAdrs(char* ifName_p, UINT32 ipAddress_p,
                         UINT32 subnetMask_p, UINT16 mtu_p)
{
    // check if module was initialized
    if (instance_l.fInitialized)
    {
        // call function
        return instance_l.targetFunctions.pfnSetIp(instance_l.simHdl,
                                                   ifName_p,
                                                   ipAddress_p,
                                                   subnetMask_p,
                                                   mtu_p);
    }

    return kErrorApiNotInitialized;
}

//------------------------------------------------------------------------------
/**
\brief Sets the simulated default gateway

This function forwards the set default gateway command to the connected
simulation environment.

\param  defaultGateway_p            Default gateway to set

\return This functions returns the resulting tOplkError return code
 */
//------------------------------------------------------------------------------
tOplkError sim_setDefaultGateway(UINT32 defaultGateway_p)
{
    // check if module was initialized
    if (instance_l.fInitialized)
    {
        // call function
        return instance_l.targetFunctions.pfnSetDefaultGateway(instance_l.simHdl,
                                                               defaultGateway_p);
    }

    return kErrorApiNotInitialized;
}

//------------------------------------------------------------------------------
/**
\brief Gets the current simulation tick

This function forwards the get tick count command to the connected simulation
environment.

\return Returns the simulated system tick in milliseconds
 */
//------------------------------------------------------------------------------
UINT32 sim_getTickCount(void)
{
    // check if module was initialized
    if (instance_l.fInitialized)
    {
        // call function
        return instance_l.targetFunctions.pfnGetTick(instance_l.simHdl);
    }

    return 0;
}

//------------------------------------------------------------------------------
/**
\brief Sets a simulated LED

This function forwards the set LED command to the connected simulation
environment.

\param  ledType_p       Determines which LED shall be set/reset
\param  fLedOn_p        Set the addressed LED on (TRUE) or off (FALSE)

\return This functions returns the resulting tOplkError return code
 */
//------------------------------------------------------------------------------
tOplkError sim_setLed(tLedType ledType_p, BOOL fLedOn_p)
{
    // check if module was initialized
    if (instance_l.fInitialized)
    {
        // call function
        return instance_l.targetFunctions.pfnSetLed(instance_l.simHdl,
                                                    ledType_p,
                                                    fLedOn_p);
    }

    return kErrorApiNotInitialized;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

/// \}
