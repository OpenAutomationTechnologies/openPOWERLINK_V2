/**
********************************************************************************
\file   identu.c

\brief  Implementation of ident module

This file contains the implementation of the ident module.

\ingroup module_identu
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2012, SYSTEC electronic GmbH
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
#include <user/identu.h>
#include <user/dllucal.h>
#include <common/ami.h>

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
typedef struct
{
    tIdentResponse*     apIdentResponse[254];    // the IdentResponse are managed dynamically
    tIdentuCbResponse   apfnCbResponse[254];
} tIdentuInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tIdentuInstance  instance_g;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static tOplkError       cbIdentResponse(const tFrameInfo* pFrameInfo_p);

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Init ident module

The function initializes an instance of the ident module

\return The function returns a tOplkError error code.

\ingroup module_identu
*/
//------------------------------------------------------------------------------
tOplkError identu_init(void)
{
    tOplkError  ret;

    OPLK_MEMSET(&instance_g, 0, sizeof(instance_g));

    // register IdentResponse callback function
    ret = dllucal_regAsndService(kDllAsndIdentResponse,
                                 cbIdentResponse,
                                 kDllAsndFilterAny);

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Shut down ident module instance

The function shuts down the ident module instance

\return The function returns a tOplkError error code.

\ingroup module_identu
*/
//------------------------------------------------------------------------------
tOplkError identu_exit(void)
{
    tOplkError  ret;

    // deregister IdentResponse callback function
    dllucal_regAsndService(kDllAsndIdentResponse, NULL, kDllAsndFilterNone);

    ret = identu_reset();

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Reset ident module instance

The function resets an ident module instance

\return The function returns a tOplkError error code.

\ingroup module_identu
*/
//------------------------------------------------------------------------------
tOplkError identu_reset(void)
{
    size_t  index;

    for (index = 0; index < tabentries(instance_g.apIdentResponse); index++)
    {
        if (instance_g.apIdentResponse[index] != NULL)
            OPLK_FREE(instance_g.apIdentResponse[index]);
    }

    OPLK_MEMSET(&instance_g, 0, sizeof(tIdentuInstance));

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Get ident response

The function gets the IdentResponse for a specified node.

\param[in]      nodeId_p            The Node ID to get the IdentResponse for.
\param[out]     ppIdentResponse_p   Pointer to store IdentResponse. NULL, if no IdentResponse
                                    is available

\return The function returns a tOplkError error code.

\ingroup module_identu
*/
//------------------------------------------------------------------------------
tOplkError identu_getIdentResponse(UINT nodeId_p,
                                   const tIdentResponse** ppIdentResponse_p)
{
    tOplkError      ret = kErrorOk;
    tIdentResponse* pIdentResponse;

    // Check parameter validity
    ASSERT(ppIdentResponse_p != NULL);

    // decrement node ID, because array is zero based
    nodeId_p--;
    if (nodeId_p < tabentries(instance_g.apIdentResponse))
    {
        pIdentResponse = instance_g.apIdentResponse[nodeId_p];
        *ppIdentResponse_p = pIdentResponse;

        // Check if ident response is valid, adjust return value otherwise
        if (pIdentResponse == NULL)
            ret = kErrorInvalidOperation;
    }
    else
    {   // invalid node ID specified
        *ppIdentResponse_p = NULL;
        ret = kErrorInvalidNodeId;
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Request ident response

The function requests the IdentResponse for a specified node.

\param[in]      nodeId_p            The Node ID to request the IdentResponse for.
\param[in]      pfnCbResponse_p     Function pointer to callback function which will
                                    be called if IdentResponse is received

\return The function returns a tOplkError error code.

\ingroup module_identu
*/
//------------------------------------------------------------------------------
tOplkError identu_requestIdentResponse(UINT nodeId_p,
                                       tIdentuCbResponse pfnCbResponse_p)
{
    tOplkError  ret = kErrorOk;

#if !defined(CONFIG_INCLUDE_NMT_MN)
    UNUSED_PARAMETER(pfnCbResponse_p);
#endif

#if defined(CONFIG_INCLUDE_NMT_MN)
    if (nodeId_p == 0)
    {   // issue request for local node
        ret = dllucal_issueRequest(kDllReqServiceIdent, 0x00, 0xFF);
        return ret;
    }
#endif

    // decrement node ID, because array is zero based
    nodeId_p--;
    if (nodeId_p < tabentries(instance_g.apfnCbResponse))
    {
#if defined(CONFIG_INCLUDE_NMT_MN)
        if (instance_g.apfnCbResponse[nodeId_p] != NULL)
        {   // request already issued (maybe by someone else)
            ret = kErrorInvalidOperation;
        }
        else
        {
            instance_g.apfnCbResponse[nodeId_p] = pfnCbResponse_p;
            ret = dllucal_issueRequest(kDllReqServiceIdent, (nodeId_p + 1), 0xFF);
        }
#else
        ret = kErrorInvalidOperation;
#endif
    }
    else
        ret = kErrorInvalidNodeId;

    return ret;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief  Callback function for IdentResponse

The function implements the callback function which will be called when a
IdentResponse is received.

\param[in]      pFrameInfo_p        Pointer to frame information structure describing
                                    the received IdentResponse frame.

\return The function returns a tOplkError error code.

\ingroup module_identu
*/
//------------------------------------------------------------------------------
static tOplkError cbIdentResponse(const tFrameInfo* pFrameInfo_p)
{
    tOplkError          ret = kErrorOk;
    UINT                nodeId;
    UINT                index;
    tIdentuCbResponse   pfnCbResponse;

    nodeId = ami_getUint8Le(&pFrameInfo_p->frame.pBuffer->srcNodeId);
    index = nodeId - 1;

    if (index < tabentries(instance_g.apfnCbResponse))
    {
        // save pointer to callback function
        pfnCbResponse = instance_g.apfnCbResponse[index];
        // reset callback function pointer so that caller may issue next request immediately
        instance_g.apfnCbResponse[index] = NULL;

        if (pfnCbResponse == NULL)
            goto Exit;

        if (pFrameInfo_p->frameSize < C_DLL_MINSIZE_IDENTRES)
        {   // IdentResponse not received or it has invalid size
            ret = pfnCbResponse(nodeId, NULL);
        }
        else
        {   // IdentResponse received
            if (instance_g.apIdentResponse[index] == NULL)
            {   // memory for IdentResponse must be allocated
                instance_g.apIdentResponse[index] = (tIdentResponse*)OPLK_MALLOC(sizeof(tIdentResponse));
                if (instance_g.apIdentResponse[index] == NULL)
                {   // malloc failed
                    ret = pfnCbResponse(nodeId,
                                        &pFrameInfo_p->frame.pBuffer->data.asnd.payload.identResponse);
                    goto Exit;
                }
            }

            // copy IdentResponse to instance structure
            OPLK_MEMCPY(instance_g.apIdentResponse[index],
                        &pFrameInfo_p->frame.pBuffer->data.asnd.payload.identResponse,
                        sizeof(tIdentResponse));
            ret = pfnCbResponse(nodeId, instance_g.apIdentResponse[index]);
        }
    }

Exit:
    return ret;
}

/// \}
