/**
********************************************************************************
\file   identu.c

\brief  Implementation of ident module

This file contains the implementation of the ident module.

\ingroup module_identu
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2012, SYSTEC electronic GmbH
Copyright (c) 2013, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
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
#include <oplk/ami.h>
#include <user/identu.h>
#include <user/dllucal.h>


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
    tEplIdentResponse*  apIdentResponse[254];    // the IdentResponse are managed dynamically
    tIdentuCbResponse   apfnCbResponse[254];
} tIdentuInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tIdentuInstance   instance_g;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static tEplKernel   identu_cbIdentResponse(tFrameInfo * pFrameInfo_p);

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Init ident module

The function initializes an instance of the ident module

\return The function returns a tEplKernel error code.

\ingroup module_identu
*/
//------------------------------------------------------------------------------
tEplKernel identu_init(void)
{
    return identu_addInstance();
}

//------------------------------------------------------------------------------
/**
\brief  Add ident module instance

The function adds an ident module instance

\return The function returns a tEplKernel error code.

\ingroup module_identu
*/
//------------------------------------------------------------------------------
tEplKernel identu_addInstance(void)
{
    tEplKernel ret = kEplSuccessful;

    EPL_MEMSET(&instance_g, 0, sizeof(instance_g));

    // register IdentResponse callback function
    ret = dllucal_regAsndService(kDllAsndIdentResponse, identu_cbIdentResponse,
                                 kDllAsndFilterAny);
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Delete ident module instance

The function deletes an ident module instance

\return The function returns a tEplKernel error code.

\ingroup module_identu
*/
//------------------------------------------------------------------------------
tEplKernel identu_delInstance(void)
    {
    tEplKernel  ret = kEplSuccessful;

    // deregister IdentResponse callback function
    dllucal_regAsndService(kDllAsndIdentResponse, NULL, kDllAsndFilterNone);

    ret = identu_reset();
    return ret;

}

//------------------------------------------------------------------------------
/**
\brief  Reset ident module instance

The function resets an ident module instance

\return The function returns a tEplKernel error code.

\ingroup module_identu
*/
//------------------------------------------------------------------------------
tEplKernel identu_reset()
{
    tEplKernel  ret;
    UINT        index;

    ret = kEplSuccessful;
    for (index = 0; index < tabentries(instance_g.apIdentResponse); index++)
    {
        if (instance_g.apIdentResponse[index] != NULL)
        {
            EPL_FREE(instance_g.apIdentResponse[index]);
        }
    }
    EPL_MEMSET(&instance_g, 0, sizeof (tIdentuInstance));

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Get ident response

The function gets the IdentResponse for a specified node.

\param  nodeId_p            The Node ID to get the IdentResponse for.
\param  ppIdentResponse_p   Pointer to store IdentResponse. NULL, if no IdentResponse
                            is available

\return The function returns a tEplKernel error code.

\ingroup module_identu
*/
//------------------------------------------------------------------------------
tEplKernel identu_getIdentResponse(UINT nodeId_p, tEplIdentResponse** ppIdentResponse_p)
{
    tEplKernel          ret = kEplSuccessful;
    tEplIdentResponse*  pIdentResponse;

    // decrement node ID, because array is zero based
    nodeId_p--;
    if (nodeId_p < tabentries(instance_g.apIdentResponse))
    {
        pIdentResponse      = instance_g.apIdentResponse[nodeId_p];
        *ppIdentResponse_p  = pIdentResponse;

        // Check if ident response is valid, adjust return value otherwise
        if( NULL == pIdentResponse )
            ret = kEplInvalidOperation;
    }
    else
    {   // invalid node ID specified
        *ppIdentResponse_p = NULL;
        ret = kEplInvalidNodeId;
    }
    return ret;

}

//------------------------------------------------------------------------------
/**
\brief  Request ident response

The function requests the IdentResponse for a specified node.

\param  nodeId_p            The Node ID to reqzest the IdentResponse for.
\param  pfnCbResponse_p     Function pointer to callback function which will
                            be called if IdentResponse is received

\return The function returns a tEplKernel error code.

\ingroup module_identu
*/
//------------------------------------------------------------------------------
tEplKernel identu_requestIdentResponse(UINT nodeId_p, tIdentuCbResponse pfnCbResponse_p)
{
    tEplKernel  ret = kEplSuccessful;

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
            ret = kEplInvalidOperation;
        }
        else
        {
            instance_g.apfnCbResponse[nodeId_p] = pfnCbResponse_p;
            ret = dllucal_issueRequest(kDllReqServiceIdent, (nodeId_p + 1), 0xFF);
        }
#else
        ret = kEplInvalidOperation;
#endif
    }
    else
    {
        ret = kEplInvalidNodeId;
    }
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Get running requests

The function returns a bitfield with the running requests for debugging
purpose.

\return The function returns a bitfield which includes the running requests for
        node 1-32.

\todo   Function is no longer exported! API must be enhanced to provide access
        to this function! Should also be enhanced to support all CNs!

\ingroup module_identu
*/
//------------------------------------------------------------------------------
UINT32 identu_getRunningRequests(void)
{
    UINT32      reqs = 0;
    UINT        index;

    for (index = 0; index < 32; index++)
    {
        if (instance_g.apfnCbResponse[index] != NULL)
            reqs |= (1 << index);
    }
    return reqs;
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

\param  pFrameInfo_p            Pointer to frame information structure describing
                                the received IdentResponse frame.

\return The function returns a tEplKernel error code.

\ingroup module_identu
*/
//------------------------------------------------------------------------------
static tEplKernel identu_cbIdentResponse(tFrameInfo* pFrameInfo_p)
{
    tEplKernel              ret = kEplSuccessful;
    UINT                    nodeId;
    UINT                    index;
    tIdentuCbResponse       pfnCbResponse;

    nodeId = ami_getUint8Le(&pFrameInfo_p->pFrame->m_le_bSrcNodeId);
    index = nodeId - 1;

    if (index < tabentries(instance_g.apfnCbResponse))
    {
        // memorize pointer to callback function
        pfnCbResponse = instance_g.apfnCbResponse[index];
        // reset callback function pointer so that caller may issue next request immediately
        instance_g.apfnCbResponse[index] = NULL;

        if (pfnCbResponse == NULL)
            goto Exit;

        if (pFrameInfo_p->frameSize < EPL_C_DLL_MINSIZE_IDENTRES)
        {   // IdentResponse not received or it has invalid size
            ret = pfnCbResponse(nodeId, NULL);
        }
        else
        {   // IdentResponse received
            if (instance_g.apIdentResponse[index] == NULL)
            {   // memory for IdentResponse must be allocated
                instance_g.apIdentResponse[index] = EPL_MALLOC(sizeof(tEplIdentResponse));
                if (instance_g.apIdentResponse[index] == NULL)
                {   // malloc failed
                    ret = pfnCbResponse(nodeId,
                                        &pFrameInfo_p->pFrame->m_Data.m_Asnd.m_Payload.m_IdentResponse);
                    goto Exit;
                }
            }

            // copy IdentResponse to instance structure
            EPL_MEMCPY(instance_g.apIdentResponse[index],
                       &pFrameInfo_p->pFrame->m_Data.m_Asnd.m_Payload.m_IdentResponse,
                       sizeof(tEplIdentResponse));
            ret = pfnCbResponse(nodeId, instance_g.apIdentResponse[index]);
        }
    }

Exit:
    return ret;
}

///\}

