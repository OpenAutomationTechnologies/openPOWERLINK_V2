/**
********************************************************************************
\file   syncu.c

\brief  Implementation of user sync module

This file contains the implementation of the sync module which is responsible
for handliny SyncReq/SyncResp frames used with poll response chaining.

\ingroup module_syncu
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2013, SYSTEC electronic GmbH
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
#include <user/syncu.h>
#include <user/dllucal.h>

#if EPL_DLL_PRES_CHAINING_MN != FALSE


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
    tSyncuCbResponse        apfnCbResponse[254];
} tSyncuInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tSyncuInstance   syncuInstance_g;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static tOplkError syncu_cbSyncResponse(tFrameInfo * pFrameInfo_p);

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Init sync module

The function initializes an instance of the sync module

\return The function returns a tOplkError error code.

\ingroup module_syncu
*/
//------------------------------------------------------------------------------
tOplkError syncu_init(void)
{
    return syncu_addInstance();
}

//------------------------------------------------------------------------------
/**
\brief  Add sync module instance

The function adds a sync module instance

\return The function returns a tOplkError error code.

\ingroup module_syncu
*/
//------------------------------------------------------------------------------
tOplkError syncu_addInstance(void)
{
    tOplkError ret = kErrorOk;

    OPLK_MEMSET(&syncuInstance_g, 0, sizeof (syncuInstance_g));
    ret = dllucal_regAsndService(kDllAsndSyncResponse, syncu_cbSyncResponse,
                                 kDllAsndFilterAny);

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Delete sync module instance

The function deletes a sync module instance

\return The function returns a tOplkError error code.

\ingroup module_syncu
*/
//------------------------------------------------------------------------------
tOplkError syncu_delInstance(void)
{
    tOplkError  ret;

    ret = dllucal_regAsndService(kDllAsndSyncResponse, NULL, kDllAsndFilterNone);
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Reset sync module instance

The function resets a sync module instance

\return The function returns a tOplkError error code.

\ingroup module_syncu
*/
//------------------------------------------------------------------------------
tOplkError syncu_reset(void)
{
    OPLK_MEMSET(&syncuInstance_g, 0, sizeof (syncuInstance_g));
    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Request sync response

The function requests the SyncResponse for a specified node.

\param  pfnCbResponse_p     Function pointer to callback function which will
                            be called if SyncResponse is received.
\param  pSyncRequestData_p  Pointer to SyncRequest data structure
\param  size_p              Size of the SyncRequest structure.

\return The function returns a tOplkError error code.

\ingroup module_syncu
*/
//------------------------------------------------------------------------------
tOplkError  syncu_requestSyncResponse(tSyncuCbResponse pfnCbResponse_p,
                                      tDllSyncRequest* pSyncRequestData_p,
                                      UINT size_p)
{
    tOplkError      ret;
    UINT            nodeId;

    ret = kErrorOk;
    nodeId = pSyncRequestData_p->nodeId;

    if (nodeId == 0)
    {
        return kErrorInvalidNodeId;
    }

    // decrement node ID, because array is zero based
    nodeId--;
    if (nodeId < tabentries (syncuInstance_g.apfnCbResponse))
    {
        if (syncuInstance_g.apfnCbResponse[nodeId] != NULL)
        {   // request already issued (maybe by someone else)
            ret = kErrorNmtSyncReqRejected;
        }
        else
        {
            syncuInstance_g.apfnCbResponse[nodeId] = pfnCbResponse_p;
            ret = dllucal_issueSyncRequest(pSyncRequestData_p, size_p);
        }
    }
    else
    {
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
\brief  Callback function for SyncResponse

The function implements the callback function which will be called when a
SyncResponse is received.

\param  pFrameInfo_p            Pointer to frame information structure describing
                                the received SyncResponse frame.

\return The function returns a tOplkError error code.

\ingroup module_identu
*/
//------------------------------------------------------------------------------
static tOplkError syncu_cbSyncResponse(tFrameInfo * pFrameInfo_p)
{
    tOplkError          ret;
    UINT                nodeId;
    UINT                index;
    tSyncuCbResponse    pfnCbResponse;

    ret = kErrorOk;

    nodeId = ami_getUint8Le(&pFrameInfo_p->pFrame->srcNodeId);
    index  = nodeId - 1;

    if (index < tabentries (syncuInstance_g.apfnCbResponse))
    {
        // memorize pointer to callback function
        pfnCbResponse = syncuInstance_g.apfnCbResponse[index];
        if (pfnCbResponse == NULL)
        {   // response was not requested
            return ret;
        }
        // reset callback function pointer so that caller may issue next request
        syncuInstance_g.apfnCbResponse[index] = NULL;

        if (pFrameInfo_p->frameSize < EPL_C_DLL_MINSIZE_SYNCRES)
        {   // SyncResponse not received or it has invalid size
            ret = pfnCbResponse(nodeId, NULL);
        }
        else
        {   // SyncResponse received
            ret = pfnCbResponse(nodeId, &pFrameInfo_p->pFrame->data.asnd.payload.syncResponse);
        }
    }
    return ret;
}

///\}

#endif // EPL_DLL_PRES_CHAINING_MN != FALSE



