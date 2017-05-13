/**
********************************************************************************
\file   sim-edrv.c

\brief  Implementation of the simulation interface for edrv functions

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
#include <sim-edrv.h>

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
\brief  Instance struct for sim-edrv module

This struct contains informations about the current instance.
*/
typedef struct
{
    tEdrvFunctions          edrvFunctions;  ///< Struct with all simulation interface functions
    tSimulationInstanceHdl  simHdl;         ///< Handle to running simulation for multiple simulated instances
    BOOL                    fInitialized;   ///< Initialization flag signaling if the stores functions are valid
} tSimEdrvInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------

/* The function pointers are only accessed after successful initialization,
 * therefore the initialization can be skipped */
static tSimEdrvInstance instance_l =
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
\brief  Setter for edrv simulation interface functions

This function sets the function pointer connecting the simulation to the
simulation environment's edrv functionalities.

\param[in]      simHdl_p            The handle of the current simulated stack instance
\param[in]      edrvFunctions_p     Structure with all simulation interface functions

\return The function returns a BOOL value.
\retval TRUE                        The function pointers are set successfully to
                                    the instance.
\retval FALSE                       The function pointers are not set because of
                                    an error.
*/
//------------------------------------------------------------------------------
BOOL sim_setEdrvFunctions(tSimulationInstanceHdl simHdl_p,
                          tEdrvFunctions edrvFunctions_p)
{
    if (!instance_l.fInitialized)
    {
        // check function pointer
        if ((edrvFunctions_p.pfnInit == NULL) ||
            (edrvFunctions_p.pfnExit == NULL) ||
            (edrvFunctions_p.pfnGetMacAddr == NULL) ||
            (edrvFunctions_p.pfnSendTxBuffer == NULL) ||
            (edrvFunctions_p.pfnAllocTxBuffer == NULL) ||
            (edrvFunctions_p.pfnFreeTxBuffer == NULL) ||
            (edrvFunctions_p.pfnChangeRxFilter == NULL) ||
            (edrvFunctions_p.pfnSetMulticastMacAddr == NULL) ||
            (edrvFunctions_p.pfnClearMulticastMacAddr == NULL))
            return FALSE;

        instance_l.edrvFunctions = edrvFunctions_p;
        instance_l.simHdl = simHdl_p;
        instance_l.fInitialized = TRUE;

        return TRUE;
    }

    return FALSE;
}

//------------------------------------------------------------------------------
/**
\brief Unset edrv simulation interface function pointers

This function unsets the function pointer connecting the simulation to the
simulation environment's edrv functionalities.
 */
//------------------------------------------------------------------------------
void sim_unsetEdrvFunctions(void)
{
    instance_l.fInitialized = FALSE;
}

//------------------------------------------------------------------------------
/**
\brief Initializes the simulated edrv module

This function forwards the initialization of the edrv module to the connected
simulation environment.

\param[in]      pEdrvInitParam_p    Edrv initialization parameters

\return This functions returns the resulting tOplkError return code.
 */
//------------------------------------------------------------------------------
tOplkError sim_initEdrv(const tEdrvInitParam* pEdrvInitParam_p)
{
    // check if functions are initialized
    if (instance_l.fInitialized)
    {
        return instance_l.edrvFunctions.pfnInit(instance_l.simHdl,
                                                pEdrvInitParam_p);
    }

    return kErrorApiNotInitialized;
}

//------------------------------------------------------------------------------
/**
\brief De-initializes the simulated edrv module

This function forwards the de-initialization of the edrv module to the connected
simulation environment.

\return This functions returns the resulting tOplkError return code.
 */
//------------------------------------------------------------------------------
tOplkError sim_exitEdrv(void)
{
    // check if functions are initialized
    if (instance_l.fInitialized)
        return instance_l.edrvFunctions.pfnExit(instance_l.simHdl);

    return kErrorApiNotInitialized;
}

//------------------------------------------------------------------------------
/**
\brief  Get the MAC address of the simulated edrv module

This function forwards the acquisition of the MAC address to the connected
simulation environment.

\return The function returns a pointer to the MAC address of the connected
        simulated edrv module.
*/
//------------------------------------------------------------------------------
const UINT8* sim_getMacAddr(void)
{
    // check if functions are initialized
    if (instance_l.fInitialized)
        return instance_l.edrvFunctions.pfnGetMacAddr(instance_l.simHdl);

    return NULL;
}

//------------------------------------------------------------------------------
/**
\brief  Send Tx buffer

This function forwards the sending call to the connected simulation environment.

\param[in,out]  pBuffer_p           Tx buffer descriptor

\return This functions returns the resulting tOplkError return code.
*/
//------------------------------------------------------------------------------
tOplkError sim_sendTxBuffer(tEdrvTxBuffer* pBuffer_p)
{
    // check if functions are initialized
    if (instance_l.fInitialized)
    {
        return instance_l.edrvFunctions.pfnSendTxBuffer(instance_l.simHdl,
                                                        pBuffer_p);
    }

    return kErrorApiNotInitialized;
}

//------------------------------------------------------------------------------
/**
\brief  Allocate a Tx buffer within in the simulated edrv module

This function forwards the allocate Tx buffer command to the connected
simulation environment.

\param[in,out]  pBuffer_p           Tx buffer descriptor

\return This functions returns the resulting tOplkError return code.
*/
//------------------------------------------------------------------------------
tOplkError sim_allocTxBuffer(tEdrvTxBuffer* pBuffer_p)
{
    // check if functions are initialized
    if (instance_l.fInitialized)
    {
        return instance_l.edrvFunctions.pfnAllocTxBuffer(instance_l.simHdl,
                                                         pBuffer_p);
    }

    return kErrorApiNotInitialized;
}

//------------------------------------------------------------------------------
/**
\brief  Free the Tx buffer within the simulated edrv module

This function forwards the free tx buffer command to the connected
simulation environment.

\param[in,out]  pBuffer_p           Tx buffer descriptor

\return This functions returns the resulting tOplkError return code.
*/
//------------------------------------------------------------------------------
tOplkError sim_freeTxBuffer(tEdrvTxBuffer* pBuffer_p)
{
    // check if functions are initialized
    if (instance_l.fInitialized)
    {
        return instance_l.edrvFunctions.pfnFreeTxBuffer(instance_l.simHdl,
                                                        pBuffer_p);
    }

    return kErrorApiNotInitialized;
}

//------------------------------------------------------------------------------
/**
\brief Change the Rx filter of the simulated edrv module

This function forwards the change Rx filter command to the connected
simulation environment.

\param[in,out]  pFilter_p           Base pointer of Rx filter array
\param[in]      count_p             Number of Rx filter array entries
\param[in]      entryChanged_p      Index of Rx filter entry that shall be changed
\param[in]      changeFlags_p       Bit mask that selects the changing Rx filter property

\return This functions returns the resulting tOplkError return code.
*/
//------------------------------------------------------------------------------
tOplkError sim_changeRxFilter(tEdrvFilter* pFilter_p,
                              UINT count_p,
                              UINT entryChanged_p,
                              UINT changeFlags_p)
{
    // check if functions are initialized
    if (instance_l.fInitialized)
    {
        return instance_l.edrvFunctions.pfnChangeRxFilter(instance_l.simHdl,
                                                          pFilter_p,
                                                          count_p,
                                                          entryChanged_p,
                                                          changeFlags_p);
    }

    return kErrorApiNotInitialized;
}

//------------------------------------------------------------------------------
/**
\brief Clear the Rx multicast MAC address of the simulated edrv module

This function forwards the change Rx multicast MAC address command to the
connected simulation environment.

\param[in]      pMacAddr_p          Multicast address

\return This functions returns the resulting tOplkError return code.
*/
//------------------------------------------------------------------------------
tOplkError sim_clearRxMulticastMacAddr(const UINT8* pMacAddr_p)
{
    // check if functions are initialized
    if (instance_l.fInitialized)
    {
        return instance_l.edrvFunctions.pfnClearMulticastMacAddr(instance_l.simHdl,
                                                                 pMacAddr_p);
    }

    return kErrorApiNotInitialized;
}

//------------------------------------------------------------------------------
/**
\brief Set the Rx multicast MAC address of the simulated edrv module

This function forwards the set Rx multicast MAC address command to the
connected simulation environment.

\param[in]      pMacAddr_p          Multicast address

\return This functions returns the resulting tOplkError return code.
*/
//------------------------------------------------------------------------------
tOplkError sim_setRxMulticastMacAddr(const UINT8* pMacAddr_p)
{
    // check if functions are initialized
    if (instance_l.fInitialized)
    {
        return instance_l.edrvFunctions.pfnSetMulticastMacAddr(instance_l.simHdl,
                                                               pMacAddr_p);
    }

    return kErrorApiNotInitialized;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

/// \}
