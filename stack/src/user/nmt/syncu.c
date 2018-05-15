/**
********************************************************************************
\file   syncu.c

\brief  Implementation of user sync module

This file contains the implementation of the sync module which is responsible
for handling SyncReq/SyncResp frames used with PollResponse Chaining.

\ingroup module_syncu
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
#include <user/syncu.h>
#include <user/dllucal.h>
#include <common/ami.h>

#if defined(CONFIG_INCLUDE_NMT_MN)
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
#define SYNCU_RESPQUEUE_LENGTH      8

//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------

/**
\brief Sync Response Queue

This struct defines the queue storing the callback functions to be called when
a sync response frame is received.
*/
typedef struct
{
    tSyncuCbResponse    apfnCallback[SYNCU_RESPQUEUE_LENGTH];   ///< Array storing the callbacks
    UINT8               writeIndex;                             ///< Queue write index
    UINT8               readIndex;                              ///< Queue read index
    UINT8               level;                                  ///< Queue fill level
} tSyncuResponseQueue;

/**
\brief User sync module instance

The following structure defines the instance variable of the user sync module.
*/
typedef struct
{
    tSyncuResponseQueue aSyncRespQueue[NMT_MAX_NODE_ID];    ///< Sync response queue
} tSyncuInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tSyncuInstance   syncuInstance_g;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static tOplkError       syncResponseCb(const tFrameInfo* pFrameInfo_p);
static tSyncuCbResponse readResponseQueue(UINT nodeId_p);
static tOplkError       writeResponseQueue(UINT nodeId_p,
                                           tSyncuCbResponse pfnCbResp_p);

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
    tOplkError  ret;

    OPLK_MEMSET(&syncuInstance_g, 0, sizeof(syncuInstance_g));

    ret = dllucal_regAsndService(kDllAsndSyncResponse,
                                 syncResponseCb,
                                 kDllAsndFilterAny);

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Shut down sync module instance

The function shuts down the sync module instance

\return The function returns a tOplkError error code.

\ingroup module_syncu
*/
//------------------------------------------------------------------------------
tOplkError syncu_exit(void)
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
    OPLK_MEMSET(&syncuInstance_g, 0, sizeof(syncuInstance_g));

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Request sync response

The function requests the SyncResponse for a specified node.

\param[in]      pfnCbResponse_p     Function pointer to callback function which will
                                    be called if SyncResponse is received.
\param[in]      pSyncRequestData_p  Pointer to SyncRequest data structure
\param[in]      size_p              Size of the SyncRequest structure.

\return The function returns a tOplkError error code.

\ingroup module_syncu
*/
//------------------------------------------------------------------------------
tOplkError syncu_requestSyncResponse(tSyncuCbResponse pfnCbResponse_p,
                                     const tDllSyncRequest* pSyncRequestData_p,
                                     size_t size_p)
{
    tOplkError  ret = kErrorOk;
    UINT        nodeId;
    UINT        index;

    // Check parameter validity
    ASSERT(pSyncRequestData_p != NULL);

    nodeId = pSyncRequestData_p->nodeId;

    if (nodeId == 0)
        return kErrorInvalidNodeId;

    index = nodeId - 1;
    if (index < tabentries(syncuInstance_g.aSyncRespQueue))
    {
        ret = writeResponseQueue(nodeId, pfnCbResponse_p);
        if (ret == kErrorOk)
            ret = dllucal_issueSyncRequest(pSyncRequestData_p, size_p);
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
\brief  Callback function for SyncResponse

The function implements the callback function which will be called when a
SyncResponse is received.

\param[in]      pFrameInfo_p        Pointer to frame information structure describing
                                    the received SyncResponse frame.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError syncResponseCb(const tFrameInfo* pFrameInfo_p)
{
    tOplkError          ret = kErrorOk;
    UINT                nodeId;
    UINT                index;
    tSyncuCbResponse    pfnCbResponse;

    nodeId = ami_getUint8Le(&pFrameInfo_p->frame.pBuffer->srcNodeId);
    index  = nodeId - 1;

    if (index < tabentries(syncuInstance_g.aSyncRespQueue))
    {
        // memorize pointer to callback function
        pfnCbResponse = readResponseQueue(nodeId);
        if (pfnCbResponse == NULL)
        {   // response was not requested
            return ret;
        }

        if (pFrameInfo_p->frameSize < C_DLL_MINSIZE_SYNCRES)
        {   // SyncResponse not received or it has invalid size
            ret = pfnCbResponse(nodeId, NULL);
        }
        else
        {   // SyncResponse received
            ret = pfnCbResponse(nodeId, &pFrameInfo_p->frame.pBuffer->data.asnd.payload.syncResponse);
        }
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Read from sync response queue

The function reads from the sync response queue to get a callback for the given
node.

\param[in]      nodeId_p            Node ID of the sync response

\return The function returns the callback function.
\retval NULL                        No callback function was read from the queue.
*/
//------------------------------------------------------------------------------
static tSyncuCbResponse readResponseQueue(UINT nodeId_p)
{
    tSyncuCbResponse        pfnRet;
    tSyncuResponseQueue*    pSyncRespQueue;

    pSyncRespQueue = &syncuInstance_g.aSyncRespQueue[nodeId_p - 1];

    if (pSyncRespQueue->level == 0)
        return NULL; // No queue entry available

    pfnRet = pSyncRespQueue->apfnCallback[pSyncRespQueue->readIndex];

    pSyncRespQueue->readIndex++;
    if (pSyncRespQueue->readIndex == SYNCU_RESPQUEUE_LENGTH)
        pSyncRespQueue->readIndex = 0;

    pSyncRespQueue->level--;

    return pfnRet;
}

//------------------------------------------------------------------------------
/**
\brief  Write to sync response queue

The function writes to the sync response queue to store a callback for the given
node.

\param[in]      nodeId_p            Node ID of the sync response
\param[in]      pfnCbResp_p         Pointer to the SyncResponse callback function
                                    to be called.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError writeResponseQueue(UINT nodeId_p, tSyncuCbResponse pfnCbResp_p)
{
    tSyncuResponseQueue*    pSyncRespQueue;

    pSyncRespQueue = &syncuInstance_g.aSyncRespQueue[nodeId_p - 1];

    if (pSyncRespQueue->level == SYNCU_RESPQUEUE_LENGTH)
        return kErrorNmtSyncReqRejected; // Queue is full

    pSyncRespQueue->apfnCallback[pSyncRespQueue->writeIndex] = pfnCbResp_p;

    pSyncRespQueue->writeIndex++;
    if (pSyncRespQueue->writeIndex == SYNCU_RESPQUEUE_LENGTH)
        pSyncRespQueue->writeIndex = 0;

    pSyncRespQueue->level++;

    return kErrorOk;
}

/// \}

#endif
