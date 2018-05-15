/**
********************************************************************************
\file   sdocom.c

\brief  SDO command layer wrapper

This file manages the available SDO stacks. The function calls are forwarded
to the SDO stack defined in the API init parameters.

\ingroup module_sdocom
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
tSdoComFunctions* sdocomdummy_getInterface(void);
tSdoComFunctions* sdocomstandard_getInterface(void);

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
static tSdoComFunctions* pSdoComInstance = NULL;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Initialize SDO stack

The function initializes the SDO stack.

\param[in]      stackType_p         Variable that defines which SDO stack to use.
\param[in]      pfnObdWrite_p       Callback function for OD write access
\param[in]      pfnObdRead_p        Callback function for OD read access

\return The function returns a tOplkError error code.

\ingroup module_sdocom
*/
//------------------------------------------------------------------------------
tOplkError sdocom_init(UINT stackType_p,
                       tComdLayerObdCb pfnObdWrite_p,
                       tComdLayerObdCb pfnObdRead_p)
{
    tOplkError  ret;

    switch (stackType_p)
    {
        case tOplkApiTestSdoCom:
        case tOplkApiTestSdoSeq:
            pSdoComInstance = sdocomdummy_getInterface();
            break;

        default:
        case tOplkApiStdSdoStack:
            pSdoComInstance = sdocomstandard_getInterface();
            break;
    }

    ret = pSdoComInstance->pfnInit(pfnObdWrite_p, pfnObdRead_p);

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Shut down SDO command layer instance

The function shuts down the SDO command layer.

\return The function returns a tOplkError error code.

\ingroup module_sdocom
*/
//------------------------------------------------------------------------------
tOplkError sdocom_exit(void)
{
    tOplkError ret = kErrorOk;

    if (pSdoComInstance != NULL)
    {
        ret = pSdoComInstance->pfnExit();
        pSdoComInstance = NULL;
    }

    return ret;
}

#if defined(CONFIG_INCLUDE_SDOC)
//------------------------------------------------------------------------------
/**
\brief  Initialize a SDO layer connection

The function initializes a SDO layer connection. It tries to reuse an
existing connection to the specified node.

\param[out]     pSdoComConHdl_p     Pointer to store the layer connection
                                    handle.
\param[in]      targetNodeId_p      Node ID of the target to connect to.
\param[in]      sdoType_p           Type of the SDO connection.

\return The function returns a tOplkError error code.

\ingroup module_sdocom
*/
//------------------------------------------------------------------------------
tOplkError sdocom_defineConnection(tSdoComConHdl* pSdoComConHdl_p,
                                   UINT targetNodeId_p,
                                   tSdoType sdoType_p)
{
    tOplkError ret = kErrorOk;

    // Check parameter validity
    ASSERT(pSdoComConHdl_p != NULL);

    if (pSdoComInstance != NULL)
        ret = pSdoComInstance->pfnDefineCon(pSdoComConHdl_p, targetNodeId_p, sdoType_p);

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Initialize a transfer by index command

The function initializes a "transfer by index" operation for a connection.

\param[in]      pSdoComTransParam_p Pointer to transfer command parameters

\return The function returns a tOplkError error code.

\ingroup module_sdocom
*/
//------------------------------------------------------------------------------
tOplkError sdocom_initTransferByIndex(const tSdoComTransParamByIndex* pSdoComTransParam_p)
{
    tOplkError ret = kErrorOk;

    // Check parameter validity
    ASSERT(pSdoComTransParam_p != NULL);

    if (pSdoComInstance != NULL)
        ret = pSdoComInstance->pfnTransByIdx(pSdoComTransParam_p);

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Get remote node ID of connection

The function returns the node ID of the remote node of a connection.

\param[in]      sdoComConHdl_p      Handle of connection.

\return The function returns the node ID of the remote node or C_ADR_INVALID
        on error.

\ingroup module_sdocom
*/
//------------------------------------------------------------------------------
UINT sdocom_getNodeId(tSdoComConHdl sdoComConHdl_p)
{
    UINT    node;

    if (pSdoComInstance != NULL)
        node = pSdoComInstance->pfnGetNodeId(sdoComConHdl_p);
    else
        node = C_ADR_INVALID;

    return node;
}

//------------------------------------------------------------------------------
/**
\brief  Get command layer connection state

The function returns the state of a command layer connection.

\param[in]      sdoComConHdl_p      Handle of the command layer connection.
\param[out]     pSdoComFinished_p   Pointer to store connection information.

\return The function returns a tOplkError error code.

\ingroup module_sdocom
*/
//------------------------------------------------------------------------------
tOplkError sdocom_getState(tSdoComConHdl sdoComConHdl_p,
                           tSdoComFinished* pSdoComFinished_p)
{
    tOplkError ret = kErrorOk;

    // Check parameter validity
    ASSERT(pSdoComFinished_p != NULL);

    if (pSdoComInstance != NULL)
        ret = pSdoComInstance->pfnGetState(sdoComConHdl_p, pSdoComFinished_p);

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Abort a SDO transfer

The function aborts an SDO transfer.

\param[in]      sdoComConHdl_p      Handle of the connection to abort.
\param[in]      abortCode_p         The abort code to use.

\return The function returns a tOplkError error code.

\ingroup module_sdocom
*/
//------------------------------------------------------------------------------
tOplkError sdocom_abortTransfer(tSdoComConHdl sdoComConHdl_p,
                                UINT32 abortCode_p)
{
    tOplkError ret = kErrorOk;

    if (pSdoComInstance != NULL)
        ret = pSdoComInstance->pfnSdoAbort(sdoComConHdl_p, abortCode_p);

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Delete a layer connection

The function closes and deletes an existing layer connection.

\param[in]      sdoComConHdl_p      Connection handle of command layer connection
                                    to delete.

\return The function returns a tOplkError error code.

\ingroup module_sdocom
*/
//------------------------------------------------------------------------------
tOplkError sdocom_undefineConnection(tSdoComConHdl sdoComConHdl_p)
{
    tOplkError ret = kErrorOk;

    if (pSdoComInstance != NULL)
        ret = pSdoComInstance->pfnDeleteCon(sdoComConHdl_p);

    return ret;
}
#endif // defined(CONFIG_INCLUDE_SDOC)
