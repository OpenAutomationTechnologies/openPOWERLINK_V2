/**
********************************************************************************
\file   sdotest-com.c

\brief  SDO command layer test functions

This file contains the implementation of the SDO Test Command Layer.

\ingroup module_sdotest_com
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2017, B&R Industrial Automation GmbH
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
#include <user/sdotest.h>

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
\brief  SDO command layer states

This enumeration defines all valid states for the SDO sequence layer state
machine.
*/
typedef enum
{
    kOplkTestSdoComStateIdle      = 0x00,       // Idle state
    kOplkTestSdoComStateWaitInit  = 0x10,       // Wait for init connection
    kOplkTestSdoComStateConnected = 0x11,       // Connection established
} eSdoTestComState;

/**
\brief SDO command layer state data type

Data type for the enumerator \ref eSdoTestComState.
*/
typedef UINT32 tSdoTestComState;

/**
\brief  SDO command layer connection control structure

This structure is used by the SDO command layer to save connection information.
*/
typedef struct
{
    UINT                     nodeId;            ///< NodeID used by this connection
    tSdoType                 tSdoType;          ///< SDO type used by this connection
    tSdoTestComState         tState;            ///< Current state of sequence layer
    tSdoSeqConHdl            tSeqHdl;           ///< Sequence layer handle
    tCircBufInstance*        pCbBufInst;        ///< Circular buffer used to avoid race conditions
} tSdoTestComCon;

/**
\brief  SDO command layer instance structure

This structure defines a SDO command layer instance.
*/
typedef struct
{
    tSdoTestComCon           tCmdCon;           ///< SDO command layer test connection
    sdoApiCbComTest          tApiCb;            ///< API callback
} tSdoTestCom;


//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tSdoTestCom sdoTestComInst_l;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
/// Callback function for receiving and forwarding SDO test command layer frames
static tOplkError receiveCb(tSdoComConHdl sdoSeqConHdl_p,
                            const tAsySdoCom* pAsySdoCom_p,
                            size_t dataSize_p);
/// Callback function for handling SDO test events
static tOplkError conCb(tSdoSeqConHdl sdoSeqConHdl_p,
                        tAsySdoConState sdoConState_p);

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Initialize the SDO command layer

This function initializes the SDO command layer test.

\param[in]      sdoComCbApi_p       pointer to the Command layer callback function

\return The function returns a tOplkError error code.

\ingroup module_sdotest_com
*/
//------------------------------------------------------------------------------
tOplkError sdotestcom_init(sdoApiCbComTest sdoComCbApi_p)
{
    tOplkError    ret;
    tCircBufError cbret;

    OPLK_MEMSET(&sdoTestComInst_l, 0, sizeof(sdoTestComInst_l));

    sdoTestComInst_l.tCmdCon.tState = kOplkTestSdoComStateIdle;
    sdoTestComInst_l.tApiCb = sdoComCbApi_p;

    // Init sequence layer
    ret = sdoseq_init(receiveCb, conCb);
    if (ret != kErrorOk)
        return kErrorInvalidOperation;

    // Init circular buffer for module internal communication
    cbret = circbuf_alloc(CIRCBUF_USER_INTERNAL_QUEUE,
                          CONFIG_EVENT_SIZE_CIRCBUF_USER_INTERNAL,
                          &sdoTestComInst_l.tCmdCon.pCbBufInst);

    if (cbret != kCircBufOk)
    {
        sdoseq_exit();
        return kErrorInvalidOperation;
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Shut down the SDO command layer

This function shuts down the SDO command layer.

\return The function returns a tOplkError error code.

\ingroup module_sdotest_com
*/
//------------------------------------------------------------------------------
tOplkError sdotestcom_exit(void)
{
    tOplkError      ret = kErrorOk;
    tOplkError      sequret;
    tCircBufError   cbret;

    // Free shared buffer
    cbret = circbuf_free(sdoTestComInst_l.tCmdCon.pCbBufInst);

    // Release sequence layer resources
    sequret = sdoseq_exit();

    if ((cbret != kCircBufOk) ||
        (sequret != kErrorOk))
        ret = kErrorInvalidOperation;
    else
        ret = kErrorOk;

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  SDO command layer send frame

This function sends an SDO command layer frame.

\param[in]      nodeId_p            Node ID of target node
\param[in]      sdoType_p           Type of SDO lower layer (ASnd/UDP)
\param[in]      pSdoCom_p           SDO command layer frame
\param[in]      sdoSize_p           Size of SDO command layer frame

\return The function returns a tOplkError error code.

\ingroup module_sdotest_com
*/
//------------------------------------------------------------------------------
tOplkError sdotestcom_sendFrame(UINT nodeId_p,
                                tSdoType sdoType_p,
                                const tAsySdoCom* pSdoCom_p,
                                size_t sdoSize_p)
{
    tOplkError      ret = kErrorOk;
    tSdoTestComCon* pCmdCon = &sdoTestComInst_l.tCmdCon;
    tEvent          event;
    tCircBufError   cbError;
    tPlkFrame*      pFrame;
    tAsySdoCom*     pSdoComDst;
    size_t          frameSize;

    frameSize = PLK_FRAME_OFFSET_SDO_COMU + sdoSize_p;

    // Check if parameters are valid
    if (frameSize > C_DLL_MAX_ASYNC_MTU)
        return kErrorInvalidOperation;

    if (kOplkTestSdoComStateIdle != pCmdCon->tState)
    {
        // If the connection is already in use, node ID and SDO type have to match
        if ((pCmdCon->nodeId != nodeId_p) ||
            (pCmdCon->tSdoType != sdoType_p))
            return kErrorInvalidOperation;
    }

    // Get frame buffer
    pFrame = (tPlkFrame*)OPLK_MALLOC(frameSize);
    if (pFrame == NULL)
        return kErrorNoResource;

    // Generate frame
    pSdoComDst = &pFrame->data.asnd.payload.sdoSequenceFrame.sdoSeqPayload;

    OPLK_MEMSET(pFrame, 0, frameSize);
    OPLK_MEMCPY(pSdoComDst, pSdoCom_p, sdoSize_p);

    // Save frame in shared buffer
    cbError = circbuf_writeData(pCmdCon->pCbBufInst, pFrame, frameSize);
    OPLK_FREE(pFrame);

    if (cbError != kCircBufOk)
        ret = kErrorInvalidOperation;
    else
    {
        // Sequence layer handling
        // Either create a new connection, or reuse existing one
        switch (pCmdCon->tState)
        {
            case kOplkTestSdoComStateIdle:
                // Get new sequence layer connection
                // When the connection is ready, the callback will trigger sending
                ret = sdoseq_initCon(&pCmdCon->tSeqHdl, nodeId_p, sdoType_p);

                pCmdCon->tState = kOplkTestSdoComStateWaitInit;
                pCmdCon->tSdoType = sdoType_p;
                pCmdCon->nodeId = nodeId_p;
                break;

            case kOplkTestSdoComStateWaitInit:
                // Connection setup is already in progress
                // Nothing left to do
                break;

            case kOplkTestSdoComStateConnected:
                // Connection is already up and running,
                // just trigger frame send event
                OPLK_MEMSET(&event.netTime, 0x00, sizeof(event.netTime));
                event.eventType = kEventTypeSdoAsySend;
                event.eventArg.pEventArg = pCmdCon;
                event.eventArgSize = sizeof(*pCmdCon);
                event.eventSink = kEventSinkSdoTest;
                ret = eventu_postEvent(&event);
                break;

            default:
                // Reject unknown states
                ret = kErrorInvalidOperation;
                break;
        }
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  SDO command layer close connection

This function closes a SDO command layer connection

\return The function returns a tOplkError error code.

\ingroup module_sdotest_com
*/
//------------------------------------------------------------------------------
tOplkError sdotestcom_closeCon(void)
{
    tOplkError      ret = kErrorOk;
    tSdoTestComCon* pCmdCon = &sdoTestComInst_l.tCmdCon;

    if (kOplkTestSdoComStateIdle != pCmdCon->tState)
    {
        // Release SDO sequence layer connection
        ret = sdoseq_deleteCon(pCmdCon->tSeqHdl);

        pCmdCon->tState = kOplkTestSdoComStateIdle;
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  SDO command layer module event handler

Handles events that come from the event module

\param[in]      pEvent_p            Event to be handled

\return The function returns a tOplkError error code.

\ingroup module_sdotest_com
*/
//------------------------------------------------------------------------------
tOplkError sdotestcom_cbEvent(const tEvent* pEvent_p)
{
    tOplkError      ret = kErrorOk;
    tOplkError      sequret;
    tSdoTestComCon* pCmdCon;
    tPlkFrame*      pFrame = NULL;
    tCircBufError   cbret;
    size_t          dataSize = 0;
    size_t          size = 0;
    size_t          frameSize = C_DLL_MAX_ASYNC_MTU;

    switch (pEvent_p->eventType)
    {
        case kEventTypeSdoAsySend:
            // Check parameter
            if (pEvent_p->eventArgSize == sizeof(*pCmdCon))
                pCmdCon = (tSdoTestComCon*)pEvent_p->eventArg.pEventArg;
            else
            {
                ret = kErrorSdoComInvalidParam;
                goto Exit;
            }

            pFrame = (tPlkFrame*)OPLK_MALLOC(frameSize);
            if (pFrame == NULL)
            {
                ret = kErrorNoResource;
                goto Exit;
            }

            // Send all frames in that are currently in the shared buffer
            do
            {
                OPLK_MEMSET(pFrame, 0, frameSize);
                cbret = circbuf_readData(pCmdCon->pCbBufInst, pFrame, size, &frameSize);
                if ((cbret == kCircBufOk) &&
                    (dataSize > PLK_FRAME_OFFSET_SDO_COMU))
                {
                    sequret = sdoseq_sendData(pCmdCon->tSeqHdl,
                                              dataSize - PLK_FRAME_OFFSET_SDO_COMU,
                                              pFrame);

                    // Send all frames, but return error code if any frame fails
                    if (sequret != kErrorOk)
                        ret = sequret;
                }
            } while (cbret == kCircBufOk);

            // kCircBufNoReadableData would be the only error free condition
            // for the preceding loop to end
            if (cbret != kCircBufNoReadableData)
                ret = kErrorInvalidOperation;
            break;

        default:
            // Reject unknown events
            ret = kErrorInvalidOperation;
            break;
    }

Exit:
    if (pFrame != NULL)
        OPLK_FREE(pFrame);

    return ret;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief  SDO command layer close connection

Receives and forwards SDO command layer frames

\param[in]      sdoSeqConHdl_p      Connection handle
\param[in]      pAsySdoCom_p        Command layer frame
\param[in]      dataSize_p          Size of command layer frame

\return The function returns a SDO frame.
*/
//------------------------------------------------------------------------------
static tOplkError receiveCb(tSdoSeqConHdl sdoSeqConHdl_p,
                            const tAsySdoCom* pAsySdoCom_p,
                            size_t dataSize_p)
{
    // Ignore unused parameter
    UNUSED_PARAMETER(sdoSeqConHdl_p);

    // Forward SDO frame to API
    return sdoTestComInst_l.tApiCb(pAsySdoCom_p, dataSize_p);
}

//------------------------------------------------------------------------------
/**
\brief  SDO command layer connection event handler

Handle SDO connection events

\param[in]      sdoSeqConHdl_p      Connection handle
\param[in]      sdoConState_p       State of sequence layer connection

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError conCb(tSdoSeqConHdl sdoSeqConHdl_p,
                        tAsySdoConState sdoConState_p)
{
    tOplkError      ret = kErrorOk;
    tSdoTestComCon* pCmdCon = &sdoTestComInst_l.tCmdCon;
    tEvent          event;

    // Ignore unused parameter
    UNUSED_PARAMETER(sdoSeqConHdl_p);

    switch (sdoConState_p)
    {
        case kAsySdoConStateConnected:
            pCmdCon->tState = kOplkTestSdoComStateConnected;

            // Trigger frame send event
            OPLK_MEMSET(&event.netTime, 0x00, sizeof(event.netTime));
            event.eventType = kEventTypeSdoAsySend;
            event.eventArg.pEventArg = pCmdCon;
            event.eventArgSize = sizeof(*pCmdCon);
            event.eventSink = kEventSinkSdoTest;
            ret = eventu_postEvent(&event);
            break;

        case kAsySdoConStateAckReceived:
        case kAsySdoConStateFrameSent:
            // These events are not forwarded to the app, and don't require any
            // handling here, so just do nothing in this case
            break;

        default:
        case kAsySdoConStateInitError:
        case kAsySdoConStateConClosed:
        case kAsySdoConStateTimeout:
        case kAsySdoConStateTransferAbort:
            // Close connection
            pCmdCon->tState = kOplkTestSdoComStateIdle;
            break;
    }

    return ret;
}

/// \}
