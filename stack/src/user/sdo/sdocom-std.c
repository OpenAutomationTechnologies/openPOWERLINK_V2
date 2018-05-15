/**
********************************************************************************
\file   sdocom-std.c

\brief  Implementation of SDO Command Layer

This file contains the standard command layer implementation.

\ingroup module_sdocom_std
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
#include <user/sdocomint.h>
#include <user/sdocomclt.h>
#include <user/sdocomsrv.h>
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
tSdoComInstance sdoComInstance_g;

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
static tOplkError processStateIdle(tSdoComConHdl sdoComConHdl_p,
                                   tSdoComConEvent sdoComConEvent_p,
                                   const tAsySdoCom* pRecvdCmdLayer_p);
//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
/**
\brief Structure for the SDO command layer standard function implementation

This structure provides the SDO command layer function interface for
the standard implementation.
*/
static tSdoComFunctions standardSdoFunctions =
{
   sdoInit,
   sdoExit,
#if defined(CONFIG_INCLUDE_SDOC)
   sdocomclt_defineConnection,
   sdocomclt_initTransferByIndex,
   sdocomclt_undefineConnection,
   sdocomclt_getState,
   sdocomclt_getNodeId,
   sdocomclt_abortTransfer,
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

\ingroup module_sdocom_std
*/
//------------------------------------------------------------------------------
tSdoComFunctions* sdocomstandard_getInterface(void)
{
    return &standardSdoFunctions;
}

//------------------------------------------------------------------------------
/**
\brief  Initialize SDO command layer of an empty POWERLINK frame

The function zeros the POWERLINK frame and builds a generic SDO command layer.

\param[out]     pPlkFrame_p         Pointer to empty POWERLINK frame
\param[in]      plkFrameSize_p      Size of POWERLINK frame
\param[in]      pSdoComCon_p        Pointer to SDO command layer connection structure
\param[out]     pCommandFrame_p     Returns pointer to command layer start
*/
//------------------------------------------------------------------------------
void sdocomint_initCmdFrameGeneric(tPlkFrame* pPlkFrame_p,
                                   size_t plkFrameSize_p,
                                   const tSdoComCon* pSdoComCon_p,
                                   tAsySdoCom** pCommandFrame_p)
{
    tAsySdoCom* pCmdFrame;

    OPLK_MEMSET(pPlkFrame_p, 0x00, plkFrameSize_p);

    // build generic part of command frame
    pCmdFrame = (tAsySdoCom*)&pPlkFrame_p->data.asnd.payload.sdoSequenceFrame.sdoSeqPayload;
    ami_setUint8Le(&pCmdFrame->commandId, pSdoComCon_p->sdoServiceType);
    ami_setUint8Le(&pCmdFrame->transactionId, pSdoComCon_p->transactionId);

    // return command frame pointer
    *pCommandFrame_p = pCmdFrame;
}

//------------------------------------------------------------------------------
/**
\brief  Set the flags field in a command layer frame

The function sets the flags field in a command layer frame without overwriting
existing flags (logical or).

\param[in,out]  pCommandFrame_p     Pointer to command layer start
\param[in]      flag_p              Flag(s) to be set
*/
//-----------------------------------------------------------------------------
void sdocomint_setCmdFrameHdrFlag(tAsySdoCom* pCommandFrame_p, UINT8 flag_p)
{
    UINT8   flag;

    flag = ami_getUint8Le(&pCommandFrame_p->flags);
    flag |= flag_p;
    ami_setUint8Le(&pCommandFrame_p->flags, flag);
}

//------------------------------------------------------------------------------
/**
\brief  Overwrite the flags field in a command layer frame

The function sets the flags field in a command layer frame. Existing flags
are overwritten.

\param[in,out]  pCommandFrame_p     Pointer to command layer start
\param[in]      flag_p              Flag(s) to be set
*/
//------------------------------------------------------------------------------
void sdocomint_overwriteCmdFrameHdrFlags(tAsySdoCom* pCommandFrame_p, UINT8 flag_p)
{
    ami_setUint8Le(&pCommandFrame_p->flags, flag_p);
}

//------------------------------------------------------------------------------
/**
\brief  Set the segment size of a command layer frame

The function sets the segment size header field of a command layer frame.

\param[in,out]  pCommandFrame_p     Pointer to command layer start
\param[in]      size_p              Command layer segment size
*/
//------------------------------------------------------------------------------
void sdocomint_setCmdFrameHdrSegmSize(tAsySdoCom* pCommandFrame_p, size_t size_p)
{
    ami_setUint16Le(&pCommandFrame_p->segmentSizeLe, (UINT16)size_p);
}

//------------------------------------------------------------------------------
/**
\brief  Copy data to a non-init segmented or expedited SDO command layer frame

The function copies data to the payload section of a segmented SDO command
layer for non-initial (second to last) segmented transfer frames.
It can also be used for an expedited ReadByIndex response.

\param[in,out]  pCommandFrame_p     Pointer to command layer start
\param[in]      pSrcData_p          Pointer to data source start
\param[in]      size_p              Size of data to copy
*/
//------------------------------------------------------------------------------
void sdocomint_fillCmdFrameDataSegm(tAsySdoCom* pCommandFrame_p,
                                    const void* pSrcData_p,
                                    size_t size_p)
{
    OPLK_MEMCPY(&pCommandFrame_p->aCommandData[0], pSrcData_p, size_p);
}

//------------------------------------------------------------------------------
/**
\brief  Update the SDO command layer connection structure size fields

The function updates the SDO command layer connection structure size related
members.

\param[in,out]  pSdoComCon_p        Pointer to SDO command layer connection structure
\param[in]      tranferredBytes_p   Size of transferred command layer data in bytes
\param[in]      fTransferComplete   Flag indicating a completed transfer
                                    (TRUE: completed, FALSE: incomplete)
*/
//------------------------------------------------------------------------------
void sdocomint_updateHdlTransfSize(tSdoComCon* pSdoComCon_p,
                                   size_t tranferredBytes_p,
                                   BOOL fTransferComplete)
{
    pSdoComCon_p->transferredBytes += tranferredBytes_p;

    if (fTransferComplete)
    {
        pSdoComCon_p->transferSize = 0;
        pSdoComCon_p->reqSegmSize = 0;
        pSdoComCon_p->respSegmSize = 0;
        pSdoComCon_p->pendingTxBytes = 0;
        pSdoComCon_p->pDataStart = NULL;
    }
    else
    {
        // prepare next transfer
        pSdoComCon_p->transferSize -= tranferredBytes_p;

#if defined(CONFIG_INCLUDE_SDOC)
        if ((pSdoComCon_p->sdoTransferType == kSdoTransSegmented) &&
            ((pSdoComCon_p->sdoComState == kSdoComStateClientConnected) ||
             (pSdoComCon_p->sdoComState == kSdoComStateClientSegmTrans)))
        {
            ASSERT(pSdoComCon_p->pDataStart != NULL);
            pSdoComCon_p->pData = (UINT8*)pSdoComCon_p->pDataStart + pSdoComCon_p->transferredBytes;
        }
#endif // defined(CONFIG_INCLUDE_SDOC)
    }

}

//------------------------------------------------------------------------------
/**
\brief  Receive callback function

The function implements the receive callback function that is called by the
SOD sequence layer when new data is received.

\param[in]      sdoSeqConHdl_p      Handle of the SDO sequence layer connection.
\param[in]      pSdoCom_p           Pointer to received command layer data.
\param[in]      dataSize_p          Size of the received data.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
tOplkError sdocomint_receiveCb(tSdoSeqConHdl sdoSeqConHdl_p,
                               const tAsySdoCom* pSdoCom_p,
                               size_t dataSize_p)
{
    tOplkError  ret;

    UNUSED_PARAMETER(dataSize_p);

    ret = sdocomint_processCmdLayerConnection(sdoSeqConHdl_p, kSdoComConEventRec, pSdoCom_p);
    if (ret == kErrorReject)
    {   // error code modified here, since sequence layer doesn't know about OD
        ret = kErrorSdoComHandleBusy;
    }

    DEBUG_LVL_SDO_TRACE("%s(): 0x%X, First Byte of pSdoCom_p: 0x%02X, dataSize_p: 0x%04X\n",
                        __func__,
                        (UINT16)sdoSeqConHdl_p,
                        (UINT16)pSdoCom_p->aCommandData[0],
                        (UINT32)dataSize_p);

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Connection state change callback function

The function implements the connection state change callback function. It is
called by the SDO sequence layer to inform the command layer about state changes
of the connection.

\param[in]      sdoSeqConHdl_p          Handle of the SDO sequence layer connection.
\param[in]      sdoConnectionState_p    SDO connection state.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
tOplkError sdocomint_conStateChangeCb(tSdoSeqConHdl sdoSeqConHdl_p,
                                      tAsySdoConState sdoConnectionState_p)
{
    tOplkError      ret;
    tSdoComConEvent sdoComConEvent = kSdoComConEventSendFirst;

    switch (sdoConnectionState_p)
    {
        case kAsySdoConStateConnected:
            DEBUG_LVL_SDO_TRACE("Connection established\n");
            sdoComConEvent = kSdoComConEventConEstablished;
            // start transmission if needed
            break;

        case kAsySdoConStateInitError:
            DEBUG_LVL_SDO_TRACE("Error during initialization\n");
            sdoComConEvent = kSdoComConEventInitError;
            // inform app about error and close sequence layer handle
            break;

        case kAsySdoConStateConClosed:
            DEBUG_LVL_SDO_TRACE("Connection closed\n");
            sdoComConEvent = kSdoComConEventConClosed;
            // close sequence layer handle
            break;

        case kAsySdoConStateAckReceived:
            DEBUG_LVL_SDO_TRACE("Acknowledge received\n");
            sdoComConEvent = kSdoComConEventAckReceived;
            // continue transmission
            break;

        case kAsySdoConStateFrameSent:
            DEBUG_LVL_SDO_TRACE("One frame sent\n");
            sdoComConEvent = kSdoComConEventFrameSent;
            // to continue transmission
            break;

        case kAsySdoConStateFrameReceived:
            DEBUG_LVL_SDO_TRACE("One frame received\n");
            sdoComConEvent = kSdoComConEventFrameReceived;
            // to continue transmission
            break;

        case kAsySdoConStateTimeout:
            DEBUG_LVL_SDO_TRACE("Timeout\n");
            sdoComConEvent = kSdoComConEventTimeout;
            // close sequence layer handle
            break;

        case kAsySdoConStateTransferAbort:
            DEBUG_LVL_SDO_TRACE("Transfer aborted\n");
            sdoComConEvent = kSdoComConEventTransferAbort;
            // inform higher layer if necessary,
            // but do not close sequence layer handle
            break;
    }

    ret = sdocomint_processCmdLayerConnection(sdoSeqConHdl_p, sdoComConEvent, NULL);
    if (ret == kErrorReject)
    {   // error code modified here, since sequence layer doesn't know about OD
        ret = kErrorSdoComHandleBusy;
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Search for a command layer control structure and process it

The function searches for a command layer control structure by an SDO sequence
layer handle and processes the command layer. If no existing connection is
found, a new one is created.

\param[in]      sdoSeqConHdl_p      Handle of the SDO sequence layer connection.
\param[in]      sdoComConEvent_p    Event to process for found connection.
\param[in]      pSdoCom_p           Pointer to received command layer data.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
tOplkError sdocomint_processCmdLayerConnection(tSdoSeqConHdl sdoSeqConHdl_p,
                                               tSdoComConEvent sdoComConEvent_p,
                                               const tAsySdoCom* pSdoCom_p)
{
    tOplkError      ret = kErrorSdoComNotResponsible;
    tSdoComCon*     pSdoComCon;
    tSdoComConHdl   hdlCount;
    tSdoComConHdl   hdlFree;

    // get pointer to first element of the array
    pSdoComCon = &sdoComInstance_g.sdoComCon[0];
    hdlCount = 0;
    hdlFree = 0xFFFF;
    while (hdlCount < CONFIG_SDO_MAX_CONNECTION_COM)
    {
        if (pSdoComCon->sdoSeqConHdl == sdoSeqConHdl_p)
        {   // matching command layer handle found
            ret = sdocomint_processState(hdlCount, sdoComConEvent_p, pSdoCom_p);
        }
        else if ((pSdoComCon->sdoSeqConHdl == 0) && (hdlFree == 0xFFFF))
            hdlFree = hdlCount;

        pSdoComCon++;
        hdlCount++;
    }

    if (ret == kErrorSdoComNotResponsible)
    {   // no responsible command layer handle found
        if (hdlFree == 0xFFFF)
        {   // no free handle delete connection immediately
            // 2008/04/14 m.u./d.k. This connection actually does not exist.
            //                      pSdoComCon is invalid.
            // ret = sdoseq_deleteCon(pSdoComCon->sdoSeqConHdl);
            ret = kErrorSdoComNoFreeHandle;
        }
        else
        {   // create new handle
            hdlCount = hdlFree;
            pSdoComCon = &sdoComInstance_g.sdoComCon[hdlCount];
            pSdoComCon->sdoSeqConHdl = sdoSeqConHdl_p;
            ret = sdocomint_processState(hdlCount, sdoComConEvent_p, pSdoCom_p);
        }
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Process SDO command layer state machine

The function processes the SDO command handler state machine. Depending
on the state the command layer event is processed.

\param[in]      sdoComConHdl_p      Handle to command layer connection.
\param[in]      sdoComConEvent_p    Event to process.
\param[in]      pRecvdCmdLayer_p    SDO command layer part of received frame.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
tOplkError sdocomint_processState(tSdoComConHdl sdoComConHdl_p,
                                  tSdoComConEvent sdoComConEvent_p,
                                  const tAsySdoCom* pRecvdCmdLayer_p)
{
    tOplkError          ret = kErrorOk;
    const tSdoComCon*   pSdoComCon;

#if (defined(WIN32) || defined(_WIN32))
    EnterCriticalSection(sdoComInstance_g.pCriticalSection);
    DEBUG_LVL_SDO_TRACE("\n\tEnterCriticalSection: %s()\n\n", __func__);
#endif

    // get pointer to control structure
    pSdoComCon = &sdoComInstance_g.sdoComCon[sdoComConHdl_p];

    // process state machine
    switch (pSdoComCon->sdoComState)
    {
        // idle state
        case kSdoComStateIdle:
            ret = processStateIdle(sdoComConHdl_p, sdoComConEvent_p, pRecvdCmdLayer_p);
            break;

#if defined(CONFIG_INCLUDE_SDOS)
        //----------------------------------------------------------------------
        // SDO Server part
        // segmented transfer
        case kSdoComStateServerSegmTrans:
            ret = sdocomsrv_processStateServerSegmTrans(sdoComConHdl_p, sdoComConEvent_p, pRecvdCmdLayer_p);
            break;
#endif // defined(CONFIG_INCLUDE_SDOS)

#if defined(CONFIG_INCLUDE_SDOC)
        //----------------------------------------------------------------------
        // SDO Client part
        // wait for finish of establishing connection
        case kSdoComStateClientWaitInit:
            ret = sdocomclt_processStateWaitInit(sdoComConHdl_p, sdoComConEvent_p, pRecvdCmdLayer_p);
            break;

        case kSdoComStateClientConnected:
            ret = sdocomclt_processStateConnected(sdoComConHdl_p, sdoComConEvent_p, pRecvdCmdLayer_p);
            break;

        // process segmented transfer
        case kSdoComStateClientSegmTrans:
            ret = sdocomclt_processStateSegmTransfer(sdoComConHdl_p, sdoComConEvent_p, pRecvdCmdLayer_p);
            break;
#endif // #if defined(CONFIG_INCLUDE_SDOC)
    }

#if (defined(WIN32) || defined(_WIN32))
    DEBUG_LVL_SDO_TRACE("\n\tLeaveCriticalSection: %s()\n\n", __func__);
    LeaveCriticalSection(sdoComInstance_g.pCriticalSection);
#endif

    return ret;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief  Initialize SDO command layer module

The function initializes the command layer module.

\param[in]      pfnObdWrite_p       Callback function for OD write access
\param[in]      pfnObdRead_p        Callback function for OD read access

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError sdoInit(tComdLayerObdCb pfnObdWrite_p,
                          tComdLayerObdCb pfnObdRead_p)
{
    tOplkError ret = kErrorOk;

    OPLK_MEMSET(&sdoComInstance_g, 0x00, sizeof(sdoComInstance_g));

#if defined(CONFIG_INCLUDE_SDOS)
    if ((pfnObdWrite_p != NULL) && (pfnObdRead_p != NULL))
    {
        sdoComInstance_g.pfnProcessObdWrite = pfnObdWrite_p;
        sdoComInstance_g.pfnProcessObdRead = pfnObdRead_p;
    }
    else
        return kErrorSdoComInvalidParam;
#else
    UNUSED_PARAMETER(pfnObdWrite_p);
    UNUSED_PARAMETER(pfnObdRead_p);
#endif // defined(CONFIG_INCLUDE_SDOS)

    ret = sdoseq_init(sdocomint_receiveCb, sdocomint_conStateChangeCb);
    if (ret != kErrorOk)
        return ret;

#if (defined(WIN32) || defined(_WIN32))
    sdoComInstance_g.pCriticalSection = &sdoComInstance_g.criticalSection;
    InitializeCriticalSection(sdoComInstance_g.pCriticalSection);
#endif

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Shut down the SDO command layer

The function shuts down the SDO command layer module.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError sdoExit(void)
{
    tOplkError  ret;

#if (defined(WIN32) || defined(_WIN32))
    DeleteCriticalSection(sdoComInstance_g.pCriticalSection);
#endif

    ret = sdoseq_exit();

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Process state kSdoComStateIdle

The function processes the SDO command handler state: kSdoComStateIdle

\param[in]      sdoComConHdl_p      Handle to command layer connection.
\param[in]      sdoComConEvent_p    Event to process.
\param[in]      pRecvdCmdLayer_p    SDO command layer part of received frame.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError processStateIdle(tSdoComConHdl sdoComConHdl_p,
                                   tSdoComConEvent sdoComConEvent_p,
                                   const tAsySdoCom* pRecvdCmdLayer_p)
{
#if !defined(CONFIG_INCLUDE_SDOS)
    // Ignore unused parameters
    UNUSED_PARAMETER(pRecvdCmdLayer_p);
#endif

    tOplkError  ret = kErrorOk;
    tSdoComCon* pSdoComCon;

    pSdoComCon = &sdoComInstance_g.sdoComCon[sdoComConHdl_p];

    switch (sdoComConEvent_p)
    {
#if defined(CONFIG_INCLUDE_SDOC)
        case kSdoComConEventInitCon: // init con for client
            // call of the init function already processed in sdocom_defineConnection()
            // only change state to kSdoComStateClientWaitInit
            pSdoComCon->sdoComState = kSdoComStateClientWaitInit;
            break;
#endif // defined(CONFIG_INCLUDE_SDOC)

        case kSdoComConEventRec:
#if defined(CONFIG_INCLUDE_SDOS)
            ret = sdocomsrv_initCon(pSdoComCon, pRecvdCmdLayer_p);
#endif
            break;

        // connection closed
        case kSdoComConEventInitError:
        case kSdoComConEventTimeout:
        case kSdoComConEventConClosed:
            ret = sdoseq_deleteCon(pSdoComCon->sdoSeqConHdl);
            OPLK_MEMSET(pSdoComCon, 0x00, sizeof(tSdoComCon));
            break;

        default:
            break;
    }

    return ret;
}

/// \}
