/**
********************************************************************************
\file   statusu.c

\brief  Implementation of status module

This file contains the implementation of the status module.

\ingroup module_statusu
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2013, SYSTEC electronic GmbH
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
#include <user/statusu.h>
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
    tStatusuCbResponse  apfnCbResponse[254];
} tStatusuInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tStatusuInstance instance_g;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static tOplkError cbStatusResponse(const tFrameInfo* pFrameInfo_p);

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Init status module

The function initializes an instance of the status module

\return The function returns a tOplkError error code.

\ingroup module_statusu
*/
//------------------------------------------------------------------------------
tOplkError statusu_init(void)
{
    tOplkError  ret;

    OPLK_MEMSET(&instance_g, 0, sizeof(instance_g));

    // register StatusResponse callback function
    ret = dllucal_regAsndService(kDllAsndStatusResponse,
                                 cbStatusResponse,
                                 kDllAsndFilterAny);

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Shut down status module instance

The function shuts down the status module instance

\return The function returns a tOplkError error code.

\ingroup module_statusu
*/
//------------------------------------------------------------------------------
tOplkError statusu_exit(void)
{
    tOplkError  ret;

    // de-register StatusResponse callback function
    ret = dllucal_regAsndService(kDllAsndStatusResponse, NULL, kDllAsndFilterNone);

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Reset status module instance

The function resets a status module instance

\return The function returns a tOplkError error code.

\ingroup module_statusu
*/
//------------------------------------------------------------------------------
tOplkError statusu_reset(void)
{
    // reset instance structure
    OPLK_MEMSET(&instance_g, 0, sizeof(instance_g));

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Request StatusResponse

The function requests the StatusResponse for a specified node.

\param[in]      nodeId_p            The Node ID to request the StatusResponse for.
\param[in]      pfnCbResponse_p     Function pointer to callback function which will
                                    be called if StatusResponse is received

\return The function returns a tOplkError error code.

\ingroup module_statusu
*/
//------------------------------------------------------------------------------
tOplkError statusu_requestStatusResponse(UINT nodeId_p,
                                         tStatusuCbResponse pfnCbResponse_p)
{
    tOplkError  ret = kErrorOk;

#if !defined(CONFIG_INCLUDE_NMT_MN)
    UNUSED_PARAMETER(pfnCbResponse_p);
#endif

#if defined(CONFIG_INCLUDE_NMT_MN)
    if (nodeId_p == 0)
    {   // issue request for local node
        ret = dllucal_issueRequest(kDllReqServiceStatus, 0x00, 0xFF);
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
            ret = dllucal_issueRequest(kDllReqServiceStatus, (nodeId_p + 1), 0xFF);
        }
#else
        ret = kErrorInvalidOperation;
#endif
    }
    else
    {   // invalid node ID specified
        ret = kErrorInvalidNodeId;
    }

    return ret;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief  Callback function for StatusResponse

The function implements the callback function which will be called when a
StatusResponse is received.

\param[in]      pFrameInfo_p        Pointer to frame information structure describing
                                    the received StatusResponse frame.

\return The function returns a tOplkError error code.

\ingroup module_statusu
*/
//------------------------------------------------------------------------------
static tOplkError cbStatusResponse(const tFrameInfo* pFrameInfo_p)
{
    tOplkError          ret = kErrorOk;
    UINT                nodeId;
    UINT                index;
    tStatusuCbResponse  pfnCbResponse;

    nodeId = ami_getUint8Le(&pFrameInfo_p->frame.pBuffer->srcNodeId);
    index = nodeId - 1;

    if (index < tabentries(instance_g.apfnCbResponse))
    {
        // memorize pointer to callback function
        pfnCbResponse = instance_g.apfnCbResponse[index];
        if (pfnCbResponse == NULL)
        {   // response was not requested
            goto Exit;
        }
        // reset callback function pointer so that a caller may issue next request
        instance_g.apfnCbResponse[index] = NULL;

        if (pFrameInfo_p->frameSize < C_DLL_MINSIZE_STATUSRES)
        {   // StatusResponse not received or it has invalid size
            ret = pfnCbResponse(nodeId, NULL);
        }
        else
        {   // StatusResponse received
            ret = pfnCbResponse(nodeId, &pFrameInfo_p->frame.pBuffer->data.asnd.payload.statusResponse);
        }
    }

Exit:
    return ret;
}

/// \}
