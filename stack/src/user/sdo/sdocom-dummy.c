/**
********************************************************************************
\file   sdocom-dummy.c

\brief  SDO Dummy functions

This file contains the implementation of the SDO Dummy Command Layer.
It is used to avoid SDO access by other modules (i.e. cfm).

\ingroup module_sdocom_dummy
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2016, B&R Industrial Automation GmbH
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
#include <user/sdocom.h>

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
// local function prototypes
//------------------------------------------------------------------------------
static tOplkError sdoInit(tComdLayerObdCb pfnObdWrite_p,
                          tComdLayerObdCb pfnObdRead_p);
static tOplkError sdoExit(void);

#if defined(CONFIG_INCLUDE_SDOC)
static tOplkError sdoDefineConnection(tSdoComConHdl* pSdoComConHdl_p,
                                      UINT targetNodeId_p,
                                      tSdoType protType_p);
static tOplkError sdoInitTransferByIndex(const tSdoComTransParamByIndex* pSdoComTransParam_p);
static tOplkError sdoUndefineConnection(tSdoComConHdl sdoComConHdl_p);
static tOplkError sdoGetState(tSdoComConHdl sdoComConHdl_p,
                              tSdoComFinished* pSdoComFinished_p);
static UINT       sdoGetNodeId(tSdoComConHdl sdoComConHdl_p);
static tOplkError sdoAbortTransfer(tSdoComConHdl sdoComConHdl_p,
                                   UINT32 abortCode_p);
#endif //defined(CONFIG_INCLUDE_SDOC)

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------

/**
\brief Structure for the SDO command layer dummy function implementation

This structure provides the SDO command layer function interface for
the dummy implementation.
*/
static tSdoComFunctions dummySdoFunctions =
{
    sdoInit,
    sdoExit,
#if defined(CONFIG_INCLUDE_SDOC)
    sdoDefineConnection,
    sdoInitTransferByIndex,
    sdoUndefineConnection,
    sdoGetState,
    sdoGetNodeId,
    sdoAbortTransfer,
#endif // defined(CONFIG_INCLUDE_SDOC)
};

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Get the function interface pointer

This function returns a pointer to the function interface structure.

\return The function returns a pointer to the local tSdoComFunctions structure

\ingroup module_sdocom_dummy
*/
//------------------------------------------------------------------------------
tSdoComFunctions* sdocomdummy_getInterface(void)
{
    return &dummySdoFunctions;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief  Initialize SDO command layer module

This function does nothing, except returning kErrorOk.

\param[in]      pfnObdWrite_p       Callback function for OD write access
\param[in]      pfnObdRead_p        Callback function for OD read access

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError sdoInit(tComdLayerObdCb pfnObdWrite_p,
                          tComdLayerObdCb pfnObdRead_p)
{
    UNUSED_PARAMETER(pfnObdWrite_p);
    UNUSED_PARAMETER(pfnObdRead_p);

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Shut down the SDO command layer module

This function does nothing, except returning kErrorOk.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError sdoExit(void)
{
    return kErrorOk;
}

#if defined(CONFIG_INCLUDE_SDOC)
//------------------------------------------------------------------------------
/**
\brief  Define a command layer connection

This function does nothing, except returning kErrorOk.

\param[out]     pSdoComConHdl_p     Pointer to store the connection handle.
\param[in]      targetNodeId_p      The node ID to connect to.
\param[in]      protType_p          The protocol type to use for the connection
                                    (UDP and ASnd is supported)

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError sdoDefineConnection(tSdoComConHdl* pSdoComConHdl_p,
                                      UINT targetNodeId_p,
                                      tSdoType protType_p)
{
    // Ignore unused parameters
    UNUSED_PARAMETER(pSdoComConHdl_p);
    UNUSED_PARAMETER(targetNodeId_p);
    UNUSED_PARAMETER(protType_p);

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Initialize a transfer by index command

This function does nothing, except returning kErrorOk.

\param[in]      pSdoComTransParam_p Pointer to transfer command parameters

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError sdoInitTransferByIndex(const tSdoComTransParamByIndex* pSdoComTransParam_p)
{
    // Ignore unused parameters
    UNUSED_PARAMETER(pSdoComTransParam_p);

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Delete a command layer connection

This function does nothing, except returning kErrorOk.

\param[in]      sdoComConHdl_p      Handle of the connection to delete.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError sdoUndefineConnection(tSdoComConHdl sdoComConHdl_p)
{
    // Ignore unused parameters
    UNUSED_PARAMETER(sdoComConHdl_p);

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Get command layer connection state

This function does nothing, except returning kErrorOk.

\param[in]      sdoComConHdl_p      Handle of the command layer connection.
\param[out]     pSdoComFinished_p   Pointer to store connection information.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError sdoGetState(tSdoComConHdl sdoComConHdl_p,
                              tSdoComFinished* pSdoComFinished_p)
{
    // Ignore unused parameters
    UNUSED_PARAMETER(sdoComConHdl_p);
    UNUSED_PARAMETER(pSdoComFinished_p);

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Get remote node ID of connection

This function does nothing, except returning C_ADR_INVALID

\param[in]      sdoComConHdl_p      Handle of connection.

\return The function returns a C_ADR_INVALID error code.
*/
//------------------------------------------------------------------------------
static UINT sdoGetNodeId(tSdoComConHdl sdoComConHdl_p)
{
    // Ignore unused parameters
    UNUSED_PARAMETER(sdoComConHdl_p);

    return C_ADR_INVALID;
}

//------------------------------------------------------------------------------
/**
\brief  Abort a SDO transfer

This function does nothing, except returning kErrorOk.

\param[in]      sdoComConHdl_p      Handle of the connection to abort.
\param[in]      abortCode_p         The abort code to use.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError sdoAbortTransfer(tSdoComConHdl sdoComConHdl_p,
                                   UINT32 abortCode_p)
{
    // Ignore unused parameters
    UNUSED_PARAMETER(sdoComConHdl_p);
    UNUSED_PARAMETER(abortCode_p);

    return kErrorOk;
}
#endif // defined(CONFIG_INCLUDE_SDOC)

/// \}
