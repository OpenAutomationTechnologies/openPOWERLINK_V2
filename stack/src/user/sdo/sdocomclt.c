/**
********************************************************************************
\file   sdocomclt.c

\brief  Implementation of SDO Command Layer client

This file contains the standard command layer implementation of the SDO client.

\ingroup module_sdocom_std
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2017, B&R Industrial Automation GmbH
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
#include <common/ami.h>

#if defined(CONFIG_INCLUDE_SDOC)
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

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static tOplkError initSingleTransfer(tSdoComCon* pSdoComCon_p,
                                     const tSdoComTransParamByIndex* pSdoComTransParam_p);
static tOplkError prepareMultiWrite(tSdoComCon* pSdoComCon_p,
                                    const tSdoComTransParamByIndex* pSdoComTransParam_p,
                                    void* pFrameBuf_p,
                                    size_t bufSize_p);
static tOplkError prepareMultiRead(tSdoComCon* pSdoComCon_p,
                                   const tSdoComTransParamByIndex* pSdoComTransParam_p,
                                   void* frameBuf_p,
                                   size_t bufSize_p);
static tOplkError processFrame(tSdoComConHdl sdoComConHdl_p,
                               const tAsySdoCom* pSdoCom_p);
static tOplkError processMultiReadResp(tSdoComConHdl sdoComConHdl_p,
                                       tSdoComCon* pSdoComCon_p,
                                       const tAsySdoCom* pSdoCom_p);
static tOplkError copyToMultiBuffer(const tSdoComCon* pSdoComCon_p,
                                    const void* pSrcData_p);
static tOplkError sendSdo(tSdoComCon* pSdoComCon_p);
static tOplkError sendSdoAbort(tSdoComCon* pSdoComCon_p,
                               UINT32 abortCode_p);
static void       updateTransferAfterTx(tSdoComCon* pSdoComCon_p);
static tOplkError transferFinished(tSdoComConHdl sdoComConHdl_p,
                                   tSdoComCon* pSdoComCon_p,
                                   tSdoComConState sdoComConState_p);

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Define a command layer connection

The function defines an SDO command layer connection to another node. It
initializes the lower layer and returns a handle for the connection. Multiple
client connections to the same node via the same protocol are not allowed.
If this function detects such a situation, it will return
kErrorSdoComHandleExists and the handle of the existing connection in
pSdoComConHdl_p.

\param[out]     pSdoComConHdl_p     Pointer to store the connection handle.
\param[in]      targetNodeId_p      The node ID to connect to.
\param[in]      protType_p          The protocol type to use for the connection
                                    (UDP and ASnd are supported)

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
tOplkError sdocomclt_defineConnection(tSdoComConHdl* pSdoComConHdl_p,
                                      UINT targetNodeId_p,
                                      tSdoType protType_p)
{
    tOplkError      ret;
    UINT            count;
    tSdoComConHdl   freeHdl;
    tSdoComCon*     pSdoComCon;

    if ((targetNodeId_p == C_ADR_INVALID) || (targetNodeId_p >= C_ADR_BROADCAST))
        return kErrorInvalidNodeId;

    // search free control structure
    pSdoComCon = &sdoComInstance_g.sdoComCon[0];
    count = 0;
    freeHdl = (tSdoComConHdl)CONFIG_SDO_MAX_CONNECTION_COM;
    while (count < CONFIG_SDO_MAX_CONNECTION_COM)
    {
        if (pSdoComCon->sdoSeqConHdl == 0)
        {
            // free entry
            freeHdl = (tSdoComConHdl)count;
        }
        else if ((pSdoComCon->nodeId == targetNodeId_p) && (pSdoComCon->sdoProtocolType == protType_p))
        {
            // existing client connection with same node ID and same protocol type
            *pSdoComConHdl_p = (tSdoComConHdl)count;
            return kErrorSdoComHandleExists;
        }

        count++;
        pSdoComCon++;
    }

    if (freeHdl == (tSdoComConHdl)CONFIG_SDO_MAX_CONNECTION_COM)
        return kErrorSdoComNoFreeHandle;

    *pSdoComConHdl_p = freeHdl;                 // save handle for application

    pSdoComCon = &sdoComInstance_g.sdoComCon[freeHdl];
    pSdoComCon->sdoProtocolType = protType_p;
    pSdoComCon->nodeId = targetNodeId_p;
    pSdoComCon->transactionId = 0;

    switch (protType_p)
    {
        case kSdoTypeUdp:
            ret = sdoseq_initCon(&pSdoComCon->sdoSeqConHdl, pSdoComCon->nodeId, kSdoTypeUdp);
            if (ret != kErrorOk)
                return ret;
            break;

        case kSdoTypeAsnd:
            ret = sdoseq_initCon(&pSdoComCon->sdoSeqConHdl, pSdoComCon->nodeId, kSdoTypeAsnd);
            if (ret != kErrorOk)
                return ret;
            break;

        case kSdoTypePdo:       // SDO over PDO -> not supported
        default:
            return kErrorSdoComUnsupportedProt;
    }

    ret = sdocomint_processState(freeHdl, kSdoComConEventInitCon, NULL);
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Initialize a transfer by index command

The function initializes a "transfer by index" operation for a connection.

\param[in]      pSdoComTransParam_p Pointer to transfer command parameters

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
tOplkError sdocomclt_initTransferByIndex(const tSdoComTransParamByIndex* pSdoComTransParam_p)
{
    tOplkError  ret;
    tSdoComCon* pSdoComCon;

    if ((pSdoComTransParam_p->index == 0) ||
        (pSdoComTransParam_p->pData == NULL) ||
        (pSdoComTransParam_p->dataSize == 0))
        return kErrorSdoComInvalidParam;

    if (pSdoComTransParam_p->sdoComConHdl >= CONFIG_SDO_MAX_CONNECTION_COM)
        return kErrorSdoComInvalidHandle;

    // get pointer to control structure of connection
    pSdoComCon = &sdoComInstance_g.sdoComCon[pSdoComTransParam_p->sdoComConHdl];

    if (pSdoComCon->sdoSeqConHdl == 0)
        return kErrorSdoComInvalidHandle;

    switch (pSdoComTransParam_p->sdoAccessType)
    {
        case kSdoAccessTypeRead:
            ret = initSingleTransfer(pSdoComCon, pSdoComTransParam_p);
            if (ret != kErrorOk)
                return ret;

            pSdoComCon->sdoServiceType = kSdoServiceReadByIndex;
            break;

        case kSdoAccessTypeWrite:
            ret = initSingleTransfer(pSdoComCon, pSdoComTransParam_p);
            if (ret != kErrorOk)
                return ret;

            pSdoComCon->sdoServiceType = kSdoServiceWriteByIndex;
            break;

        case kSdoAccessTypeMultiWrite:
            ret = prepareMultiWrite(pSdoComCon,
                                    pSdoComTransParam_p,
                                    pSdoComTransParam_p->pMultiBuffer,
                                    pSdoComTransParam_p->multiBufSize);
            if ((ret != kErrorOk) && (ret != kErrorSdoComInvalidServiceType))
            {
                return ret;
            }
            else if (ret == kErrorSdoComInvalidServiceType)
            {   // standard single write
                initSingleTransfer(pSdoComCon, pSdoComTransParam_p);
                pSdoComCon->sdoServiceType = kSdoServiceWriteByIndex;
            }
            else
            {   // multi write
                pSdoComCon->pData = pSdoComTransParam_p->pMultiBuffer; // save pointer to data
                pSdoComCon->targetIndex = 0;
                pSdoComCon->targetSubIndex = 0;
                pSdoComCon->sdoServiceType = kSdoServiceWriteMultiByIndex;
            }
            break;

        case kSdoAccessTypeMultiRead:
            ret = prepareMultiRead(pSdoComCon,
                                   pSdoComTransParam_p,
                                   pSdoComTransParam_p->pMultiBuffer,
                                   pSdoComTransParam_p->multiBufSize);
            if ((ret != kErrorOk) && (ret != kErrorSdoComInvalidServiceType))
            {
                return ret;
            }
            else if (ret == kErrorSdoComInvalidServiceType)
            {   // standard single write
                initSingleTransfer(pSdoComCon, pSdoComTransParam_p);
                pSdoComCon->sdoServiceType = kSdoServiceReadByIndex;
            }
            else
            {   // multi read
                pSdoComCon->pData = pSdoComTransParam_p->pMultiBuffer; // save pointer to data
                pSdoComCon->targetIndex = 0;
                pSdoComCon->targetSubIndex = 0;
                pSdoComCon->sdoServiceType = kSdoServiceReadMultiByIndex;
            }
            break;

        default:
            return kErrorSdoComInvalidParam;
    }

    pSdoComCon->transferredBytes = 0;
    pSdoComCon->lastAbortCode = 0;
    pSdoComCon->sdoTransferType = kSdoTransAuto;
    // callback function for end of transfer
    pSdoComCon->pfnTransferFinished = pSdoComTransParam_p->pfnSdoFinishedCb;
    pSdoComCon->pUserArg = pSdoComTransParam_p->pUserArg;

    ret = sdocomint_processState(pSdoComTransParam_p->sdoComConHdl, kSdoComConEventSendFirst, NULL);

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Delete a command layer connection

The function deletes an SDO command layer connection to another node.

\param[in]      sdoComConHdl_p      Handle of the connection to delete.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
tOplkError sdocomclt_undefineConnection(tSdoComConHdl sdoComConHdl_p)
{
    tOplkError  ret = kErrorOk;
    tSdoComCon* pSdoComCon;

    if (sdoComConHdl_p >= CONFIG_SDO_MAX_CONNECTION_COM)
        return kErrorSdoComInvalidHandle;

    // get pointer to control structure
    pSdoComCon = &sdoComInstance_g.sdoComCon[sdoComConHdl_p];

    // $$$ d.k. abort a running transfer before closing the sequence layer
    if (((pSdoComCon->sdoSeqConHdl & ~SDO_SEQ_HANDLE_MASK) != SDO_SEQ_INVALID_HDL) &&
        (pSdoComCon->sdoSeqConHdl != 0))
    {
        // close connection in lower layer
        switch (pSdoComCon->sdoProtocolType)
        {
            case kSdoTypeAsnd:
            case kSdoTypeUdp:
                ret = sdoseq_deleteCon(pSdoComCon->sdoSeqConHdl);
                break;

            case kSdoTypePdo:
            case kSdoTypeAuto:
            default:
                return kErrorSdoComUnsupportedProt;
        }
    }

    OPLK_MEMSET(pSdoComCon, 0x00, sizeof(tSdoComCon));

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Get command layer connection state

The function returns the state of a command layer connection.

\param[in]      sdoComConHdl_p      Handle of the command layer connection.
\param[out]     pSdoComFinished_p   Pointer to store connection information.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
tOplkError sdocomclt_getState(tSdoComConHdl sdoComConHdl_p,
                              tSdoComFinished* pSdoComFinished_p)
{
    tOplkError  ret = kErrorOk;
    tSdoComCon* pSdoComCon;

    if (sdoComConHdl_p >= CONFIG_SDO_MAX_CONNECTION_COM)
        return kErrorSdoComInvalidHandle;

    // get pointer to control structure
    pSdoComCon = &sdoComInstance_g.sdoComCon[sdoComConHdl_p];

    // check if handle ok
    if (pSdoComCon->sdoSeqConHdl == 0)
        return kErrorSdoComInvalidHandle;

    pSdoComFinished_p->pUserArg = pSdoComCon->pUserArg;
    pSdoComFinished_p->nodeId = pSdoComCon->nodeId;
    pSdoComFinished_p->targetIndex = pSdoComCon->targetIndex;
    pSdoComFinished_p->targetSubIndex = pSdoComCon->targetSubIndex;
    pSdoComFinished_p->transferredBytes = (UINT)pSdoComCon->transferredBytes;
    pSdoComFinished_p->abortCode = pSdoComCon->lastAbortCode;
    pSdoComFinished_p->sdoComConHdl = sdoComConHdl_p;

    if (pSdoComCon->sdoServiceType == kSdoServiceWriteByIndex)
        pSdoComFinished_p->sdoAccessType = kSdoAccessTypeWrite;
    else
        pSdoComFinished_p->sdoAccessType = kSdoAccessTypeRead;

    if (pSdoComCon->lastAbortCode != 0)
    {
        // sdo abort
        pSdoComFinished_p->sdoComConState = kSdoComTransferRxAborted;
        // delete abort code
        pSdoComCon->lastAbortCode = 0;
    }
    else if ((pSdoComCon->sdoSeqConHdl & ~SDO_SEQ_HANDLE_MASK) == SDO_SEQ_INVALID_HDL)
    {
        // check state
        pSdoComFinished_p->sdoComConState = kSdoComTransferLowerLayerAbort;
    }
    else if (pSdoComCon->sdoComState == kSdoComStateClientWaitInit)
    {
        // finished
        pSdoComFinished_p->sdoComConState = kSdoComTransferNotActive;
    }
    else if (pSdoComCon->transferSize == 0)
    {
        // finished
        pSdoComFinished_p->sdoComConState = kSdoComTransferFinished;
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Get remote node ID of connection

The function returns the node ID of the remote node of a connection.

\param[in]      sdoComConHdl_p      Handle of connection.

\return The function returns the node ID of the remote node or C_ADR_INVALID
        on error.
*/
//------------------------------------------------------------------------------
UINT sdocomclt_getNodeId(tSdoComConHdl sdoComConHdl_p)
{
    UINT        nodeId = C_ADR_INVALID;
    tSdoComCon* pSdoComCon;

    if (sdoComConHdl_p >= CONFIG_SDO_MAX_CONNECTION_COM)
        return nodeId;

    // get pointer to control structure
    pSdoComCon = &sdoComInstance_g.sdoComCon[sdoComConHdl_p];

    if (pSdoComCon->sdoSeqConHdl == 0)
        return nodeId;

    nodeId = pSdoComCon->nodeId;
    return nodeId;
}

//------------------------------------------------------------------------------
/**
\brief  Abort an SDO transfer

The function aborts an SDO transfer.

\param[in]      sdoComConHdl_p      Handle of the connection to abort.
\param[in]      abortCode_p         The abort code to use.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
tOplkError sdocomclt_abortTransfer(tSdoComConHdl sdoComConHdl_p, UINT32 abortCode_p)
{
    tOplkError  ret;
    tSdoComCon* pSdoComCon;

    if (sdoComConHdl_p >= CONFIG_SDO_MAX_CONNECTION_COM)
        return kErrorSdoComInvalidHandle;

    // get pointer to control structure of connection
    pSdoComCon = &sdoComInstance_g.sdoComCon[sdoComConHdl_p];

    if (pSdoComCon->sdoSeqConHdl == 0)
        return kErrorSdoComInvalidHandle;

    pSdoComCon->pData = &abortCode_p;
    ret = sdocomint_processState(sdoComConHdl_p, kSdoComConEventAbort, (tAsySdoCom*)NULL);

    // reference is only valid locally in this function, therefore it is
    // invalidated here, since the storage is a global variable
    pSdoComCon->pData = NULL;

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Process state clientProcessStateWaitInit

The function processes the SDO command handler state: clientProcessStateWaitInit

\param[in]      sdoComConHdl_p      Handle to command layer connection.
\param[in]      sdoComConEvent_p    Event to process.
\param[in]      pRecvdCmdLayer_p    SDO command layer part of received frame.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
tOplkError sdocomclt_processStateWaitInit(tSdoComConHdl sdoComConHdl_p,
                                          tSdoComConEvent sdoComConEvent_p,
                                          const tAsySdoCom* pRecvdCmdLayer_p)
{
    tOplkError  ret = kErrorOk;
    tSdoComCon* pSdoComCon;

    UNUSED_PARAMETER(pRecvdCmdLayer_p);

    pSdoComCon = &sdoComInstance_g.sdoComCon[sdoComConHdl_p];

    // if connection handle is invalid reinit connection
    // d.k.: this will be done only on new events (i.e. InitTransfer)
    if ((pSdoComCon->sdoSeqConHdl & ~SDO_SEQ_HANDLE_MASK) == SDO_SEQ_INVALID_HDL)
    {
        // check kind of connection to reinit
        switch (pSdoComCon->sdoProtocolType)
        {
            case kSdoTypeUdp:
                ret = sdoseq_initCon(&pSdoComCon->sdoSeqConHdl, pSdoComCon->nodeId, kSdoTypeUdp);
                if (ret != kErrorOk)
                    return ret;
                break;

            case kSdoTypeAsnd:
                ret = sdoseq_initCon(&pSdoComCon->sdoSeqConHdl, pSdoComCon->nodeId, kSdoTypeAsnd);
                if (ret != kErrorOk)
                    return ret;
                break;

            case kSdoTypePdo:   // Pdo -> not supported
            default:
                ret = kErrorSdoComUnsupportedProt;
                return ret;
        }
        // d.k.: reset transaction ID, because new sequence layer connection was initialized
        // $$$ d.k. is this really necessary?
        //pSdoComCon->transactionId = 0;
    }

    switch (sdoComConEvent_p)
    {
        case kSdoComConEventConEstablished:
            // send first frame if needed
            if (((pSdoComCon->transferSize > 0) && (pSdoComCon->targetIndex != 0)) ||
                ((pSdoComCon->transferSize > 0) &&
                 ((pSdoComCon->sdoServiceType == kSdoServiceWriteMultiByIndex) ||
                  (pSdoComCon->sdoServiceType == kSdoServiceReadMultiByIndex))))
            {
                // start SDO transfer
                // check if segmented transfer
                if (pSdoComCon->sdoTransferType == kSdoTransSegmented)
                    pSdoComCon->sdoComState = kSdoComStateClientSegmTrans;
                else
                    pSdoComCon->sdoComState = kSdoComStateClientConnected;

                ret = sendSdo(pSdoComCon);
                if (ret != kErrorOk)
                    return ret;
            }
            else
                pSdoComCon->sdoComState = kSdoComStateClientConnected;
            break;

        case kSdoComConEventSendFirst:
            // infos for transfer already saved by function sdocom_initTransferByIndex
            break;

        // abort to send from higher layer
        case kSdoComConEventAbort:
            pSdoComCon->lastAbortCode = *((const UINT32*)pSdoComCon->pData);
            ret = transferFinished(sdoComConHdl_p, pSdoComCon, kSdoComTransferTxAborted);
            break;

        case kSdoComConEventConClosed:
        case kSdoComConEventInitError:
        case kSdoComConEventTimeout:
        case kSdoComConEventTransferAbort:
            sdoseq_deleteCon(pSdoComCon->sdoSeqConHdl);         // close sequence layer handle
            pSdoComCon->sdoSeqConHdl |= SDO_SEQ_INVALID_HDL;
            if (sdoComConEvent_p == kSdoComConEventTimeout)
                pSdoComCon->lastAbortCode = SDO_AC_TIME_OUT;
            else
                pSdoComCon->lastAbortCode = 0;

            ret = transferFinished(sdoComConHdl_p, pSdoComCon, kSdoComTransferLowerLayerAbort);
            // d.k.: do not clean control structure
            break;

        default:
            break;
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Process state kSdoComStateClientConnected

The function processes the SDO command handler state: kSdoComStateClientConnected

\param[in]      sdoComConHdl_p      Handle to command layer connection.
\param[in]      sdoComConEvent_p    Event to process.
\param[in]      pRecvdCmdLayer_p    SDO command layer part of received frame.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
tOplkError sdocomclt_processStateConnected(tSdoComConHdl sdoComConHdl_p,
                                           tSdoComConEvent sdoComConEvent_p,
                                           const tAsySdoCom* pRecvdCmdLayer_p)
{

    tOplkError  ret = kErrorOk;
    UINT8       flag;
    tSdoComCon* pSdoComCon;

    pSdoComCon = &sdoComInstance_g.sdoComCon[sdoComConHdl_p];

    switch (sdoComConEvent_p)
    {
        // send a frame
        case kSdoComConEventFrameSent:
             updateTransferAfterTx(pSdoComCon);
             // no break - fall through is intended
        case kSdoComConEventSendFirst:
        case kSdoComConEventAckReceived:
        case kSdoComConEventFrameReceived:
            ret = sendSdo(pSdoComCon);
            if (ret != kErrorOk)
                return ret;

            // check if read transfer finished
            if ((pSdoComCon->transferSize == 0) && (pSdoComCon->transferredBytes != 0) &&
                (pSdoComCon->sdoServiceType == kSdoServiceReadByIndex))
            {
                pSdoComCon->transactionId++;
                pSdoComCon->lastAbortCode = 0;
                ret = transferFinished(sdoComConHdl_p, pSdoComCon, kSdoComTransferFinished);
                return ret;
            }

            if (pSdoComCon->sdoTransferType == kSdoTransSegmented)
            {
                pSdoComCon->sdoComState = kSdoComStateClientSegmTrans;
                return ret;
            }
            break;

        case kSdoComConEventRec:
            // check if the frame is a SDO response and has the right transaction ID
            flag = ami_getUint8Le(&pRecvdCmdLayer_p->flags);
            if (((flag & SDO_CMDL_FLAG_RESPONSE) != 0) &&
                 (ami_getUint8Le(&pRecvdCmdLayer_p->transactionId) == pSdoComCon->transactionId))
            {
                if ((flag & SDO_CMDL_FLAG_ABORT) != 0)
                {
                    // send acknowledge without any Command layer data
                    sdoseq_sendData(pSdoComCon->sdoSeqConHdl, 0, (tPlkFrame*)NULL);
                    pSdoComCon->transactionId++;
                    pSdoComCon->lastAbortCode = ami_getUint32Le(&pRecvdCmdLayer_p->aCommandData[0]);
                    ret = transferFinished(sdoComConHdl_p, pSdoComCon, kSdoComTransferRxAborted);
                    return ret;
                }
                else
                {   // normal frame received
                    ret = processFrame(sdoComConHdl_p, pRecvdCmdLayer_p);
                    // check if transfer ready
                    if (pSdoComCon->transferSize == 0)
                    {
                        // send acknowledge without any Command layer data
                        sdoseq_sendData(pSdoComCon->sdoSeqConHdl, 0, (tPlkFrame*)NULL);
                        pSdoComCon->transactionId++;
                        pSdoComCon->lastAbortCode = 0;
                        ret = transferFinished(sdoComConHdl_p, pSdoComCon, kSdoComTransferFinished);
                        return ret;
                    }
                }
            }
            else
            {   // this command layer handle is not responsible
                // (wrong direction or wrong transaction ID)
                ret = kErrorSdoComNotResponsible;
                return ret;
            }
            break;

        // connection closed event go back to kSdoComStateClientWaitInit
        case kSdoComConEventConClosed:
            // connection closed by communication partner
            sdoseq_deleteCon(pSdoComCon->sdoSeqConHdl);         // close sequence layer handle
            pSdoComCon->sdoSeqConHdl |= SDO_SEQ_INVALID_HDL;
            pSdoComCon->sdoComState = kSdoComStateClientWaitInit;
            pSdoComCon->lastAbortCode = 0;
            ret = transferFinished(sdoComConHdl_p, pSdoComCon, kSdoComTransferLowerLayerAbort);
            break;

        case kSdoComConEventAbort:
            sendSdoAbort(pSdoComCon, *((const UINT32*)pSdoComCon->pData));
            pSdoComCon->transactionId++;
            pSdoComCon->lastAbortCode = *((const UINT32*)pSdoComCon->pData);
            ret = transferFinished(sdoComConHdl_p, pSdoComCon, kSdoComTransferTxAborted);
            break;

        case kSdoComConEventInitError:
        case kSdoComConEventTimeout:
            sdoseq_deleteCon(pSdoComCon->sdoSeqConHdl);         // close sequence layer handle
            pSdoComCon->sdoSeqConHdl |= SDO_SEQ_INVALID_HDL;
            pSdoComCon->sdoComState = kSdoComStateClientWaitInit;
            pSdoComCon->lastAbortCode = SDO_AC_TIME_OUT;
            ret = transferFinished(sdoComConHdl_p, pSdoComCon, kSdoComTransferLowerLayerAbort);
            break;

        case kSdoComConEventTransferAbort:
            pSdoComCon->sdoComState = kSdoComStateClientWaitInit;
            pSdoComCon->lastAbortCode = SDO_AC_TIME_OUT;
            ret = transferFinished(sdoComConHdl_p, pSdoComCon, kSdoComTransferLowerLayerAbort);
            break;

        default:
            break;
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Process state kSdoComStateClientSegmTrans

The function processes the SDO command handler state: kSdoComStateClientSegmTrans

\param[in]      sdoComConHdl_p      Handle to command layer connection.
\param[in]      sdoComConEvent_p    Event to process.
\param[in]      pRecvdCmdLayer_p    SDO command layer part of received frame.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
tOplkError sdocomclt_processStateSegmTransfer(tSdoComConHdl sdoComConHdl_p,
                                              tSdoComConEvent sdoComConEvent_p,
                                              const tAsySdoCom* pRecvdCmdLayer_p)
{
    tOplkError  ret = kErrorOk;
    UINT8       flag;
    tSdoComCon* pSdoComCon;

    pSdoComCon = &sdoComInstance_g.sdoComCon[sdoComConHdl_p];

    switch (sdoComConEvent_p)
    {
        // send a frame
        case kSdoComConEventFrameSent:
             updateTransferAfterTx(pSdoComCon);
             // no break - fall through is intended
        case kSdoComConEventSendFirst:
        case kSdoComConEventAckReceived:
        case kSdoComConEventFrameReceived:
            ret = sendSdo(pSdoComCon);
            if (ret != kErrorOk)
                return ret;

            // check if read transfer finished
            if ((pSdoComCon->transferSize == 0) && (pSdoComCon->sdoServiceType == kSdoServiceReadByIndex))
            {
                pSdoComCon->transactionId++;
                pSdoComCon->sdoComState = kSdoComStateClientConnected;
                pSdoComCon->lastAbortCode = 0;
                ret = transferFinished(sdoComConHdl_p, pSdoComCon, kSdoComTransferFinished);
                return ret;
            }
            break;

        case kSdoComConEventRec:
            // check if the frame is a response
            flag = ami_getUint8Le(&pRecvdCmdLayer_p->flags);
            if (((flag & SDO_CMDL_FLAG_RESPONSE) != 0) &&
                (ami_getUint8Le(&pRecvdCmdLayer_p->transactionId) == pSdoComCon->transactionId))
            {
                if ((flag & SDO_CMDL_FLAG_ABORT) != 0)
                {
                    // send acknowledge without any Command layer data
                    sdoseq_sendData(pSdoComCon->sdoSeqConHdl, 0, (tPlkFrame*)NULL);
                    pSdoComCon->transactionId++;
                    pSdoComCon->sdoComState = kSdoComStateClientConnected;
                    pSdoComCon->lastAbortCode = ami_getUint32Le(&pRecvdCmdLayer_p->aCommandData[0]);
                    ret = transferFinished(sdoComConHdl_p, pSdoComCon, kSdoComTransferRxAborted);
                    return ret;
                }
                else
                {   // normal frame received
                    ret = processFrame(sdoComConHdl_p, pRecvdCmdLayer_p);
                    // check if transfer ready
                    if (pSdoComCon->transferSize == 0)
                    {
                        // send acknowledge without any Command layer data
                        sdoseq_sendData(pSdoComCon->sdoSeqConHdl, 0, NULL);
                        pSdoComCon->transactionId++;
                        pSdoComCon->sdoComState = kSdoComStateClientConnected;
                        pSdoComCon->lastAbortCode = 0;
                        ret = transferFinished(sdoComConHdl_p, pSdoComCon, kSdoComTransferFinished);
                    }
                }
            }
            break;

        // connection closed event go back to kSdoComStateClientWaitInit
        case kSdoComConEventConClosed:
            // connection closed by communication partner
            sdoseq_deleteCon(pSdoComCon->sdoSeqConHdl);         // close sequence layer handle
            pSdoComCon->sdoSeqConHdl |= SDO_SEQ_INVALID_HDL;
            pSdoComCon->sdoComState = kSdoComStateClientWaitInit;
            pSdoComCon->transactionId++;
            pSdoComCon->lastAbortCode = 0;
            ret = transferFinished(sdoComConHdl_p, pSdoComCon, kSdoComTransferFinished);
            break;

        // abort to send from higher layer
        case kSdoComConEventAbort:
            sendSdoAbort(pSdoComCon, *((const UINT32*)pSdoComCon->pData));
            pSdoComCon->transactionId++;
            pSdoComCon->sdoComState = kSdoComStateClientConnected;
            pSdoComCon->lastAbortCode = *((const UINT32*)pSdoComCon->pData);
            ret = transferFinished(sdoComConHdl_p, pSdoComCon, kSdoComTransferTxAborted);
            break;

        case kSdoComConEventInitError:
        case kSdoComConEventTimeout:
            sdoseq_deleteCon(pSdoComCon->sdoSeqConHdl);         // close sequence layer handle
            pSdoComCon->sdoSeqConHdl |= SDO_SEQ_INVALID_HDL;
            pSdoComCon->sdoComState = kSdoComStateClientWaitInit;
            pSdoComCon->lastAbortCode = SDO_AC_TIME_OUT;
            ret = transferFinished(sdoComConHdl_p, pSdoComCon, kSdoComTransferLowerLayerAbort);
            break;

        case kSdoComConEventTransferAbort:
            pSdoComCon->sdoComState = kSdoComStateClientWaitInit;
            pSdoComCon->lastAbortCode = SDO_AC_TIME_OUT;
            ret = transferFinished(sdoComConHdl_p, pSdoComCon, kSdoComTransferLowerLayerAbort);
            break;

        default:
            break;
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
\brief  Initialize communication connection for single transfers

The function initialize a communication connection structure for single transfers
e.g. Write- and ReadByIndex.

\param[in,out]  pSdoComCon_p        Pointer to SDO command layer connection structure
\param[in]      pSdoComTransParam_p Pointer to transfer command parameters

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError initSingleTransfer(tSdoComCon* pSdoComCon_p,
                                     const tSdoComTransParamByIndex* pSdoComTransParam_p)
{
    // check if command layer is idle
    if ((pSdoComCon_p->transferredBytes + pSdoComCon_p->transferSize) > 0)
    {
        return kErrorSdoComHandleBusy;
    }

    pSdoComCon_p->pData = pSdoComTransParam_p->pData;             // save pointer to data
    pSdoComCon_p->pDataStart = pSdoComCon_p->pData;
    pSdoComCon_p->transferSize = pSdoComTransParam_p->dataSize;   // maximal bytes to transfer
    pSdoComCon_p->targetIndex = pSdoComTransParam_p->index;
    pSdoComCon_p->targetSubIndex = pSdoComTransParam_p->subindex;

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Set up a Write Multiple Parameters by Index frame

The function creates a WriteMultParam command frame. This function is called
once to set up the whole frame, and adds all entries into a caller-provided buffer.

\param[in,out]  pSdoComCon_p        Pointer to SDO command layer connection structure
\param[in]      pSdoComTransParam_p Pointer to transfer command parameters
\param[in]      pFrameBuf_p         Pointer to command layer payload buffer
\param[in]      bufSize_p           Size of command layer payload buffer

\return The function returns a tOplkError error code.
\retval kErrorNoResource:               Provided buffer is invalid or too small
\retval kErrorSdoComInvalidServiceType: Only one sub-request -> use standard transfer!
*/
//------------------------------------------------------------------------------
static tOplkError prepareMultiWrite(tSdoComCon* pSdoComCon_p,
                                    const tSdoComTransParamByIndex* pSdoComTransParam_p,
                                    void* pFrameBuf_p,
                                    size_t bufSize_p)
{
    tOplkError                      ret = kErrorOk;
    size_t                          cmdSegmSize = 0;
    size_t                          lastCmdSegmSize = 0;
    UINT8                           padBytes = 0;
    tAsySdoComMultWriteReqReadResp* pCurSubHdr = NULL;
    tAsySdoComMultWriteReqReadResp* pLastSubHdr= NULL;
    tSdoMultiAccEntry*              pCurSubAcc = NULL;
    UINT                            loopCnt = 0;

    // check if command layer is idle
    if (pSdoComCon_p->transferredBytes > 0)
        return kErrorSdoComHandleBusy;

    // check provided buffer
    if ((pFrameBuf_p == NULL) || (bufSize_p < SDO_CMD_SEGM_TX_MAX_SIZE))
    {
        // bad buffer provided by API
        return kErrorNoResource;
    }

    if (pSdoComTransParam_p->multiAccCnt == 1)
    {
        // only one object access requested -> use normal write by index transfer
        return kErrorSdoComInvalidServiceType;
    }

    pCurSubHdr = (tAsySdoComMultWriteReqReadResp*)pFrameBuf_p;
    pCurSubAcc = (tSdoMultiAccEntry*)pSdoComTransParam_p->paMultiAcc;
    pSdoComCon_p->multiAccCnt = 0;

    // loop over all sub accesses to build the frame
    for (; loopCnt < pSdoComTransParam_p->multiAccCnt; loopCnt++)
    {
        if ((pCurSubAcc->index == 0) || (pCurSubAcc->pData_le == NULL) ||
            (pCurSubAcc->dataSize == 0))
            return kErrorApiInvalidParam;

        // NOTE: padding for last sub-element could be skipped, but wasting 3 bytes is ok
        padBytes = (~pCurSubAcc->dataSize + 1U) & 0x03;
        cmdSegmSize = lastCmdSegmSize + 8 + pCurSubAcc->dataSize + padBytes;

        // check if transfer fits in buffer
        if (SDO_CMD_SEGM_TX_MAX_SIZE < cmdSegmSize)
        {   // too many or too big write accesses
            if (loopCnt == 0)
            {   // not even the first access fits into the buffer
                return kErrorNoResource;
            }
            else
            {   // do not add this and following objects to the transfer,
                // rather use current reduced count of objects
                break;
            }
        }

        ami_setUint32Le(&pCurSubHdr->byteOffsetNext, 0);
        ami_setUint16Le(&pCurSubHdr->index, pCurSubAcc->index);
        ami_setUint8Le(&pCurSubHdr->subIndex, pCurSubAcc->subIndex);
        pCurSubHdr->info = padBytes;
        OPLK_MEMCPY(&pCurSubHdr->aCommandData[0],
                    pCurSubAcc->pData_le,
                    pCurSubAcc->dataSize);

        if (loopCnt > 0)
        {   // it is the second or a following sub-element -> update last header
            ami_setUint32Le(&pLastSubHdr->byteOffsetNext, (UINT32)(lastCmdSegmSize + SDO_CMDL_HDR_FIXED_SIZE));
        }

        // prepare next iteration
        pLastSubHdr = pCurSubHdr;
        pCurSubHdr = (tAsySdoComMultWriteReqReadResp*)((UINT8*)pFrameBuf_p + cmdSegmSize);
        pSdoComCon_p->multiAccCnt++;
        pCurSubAcc++;

        lastCmdSegmSize = cmdSegmSize;
    }

    // save Tx transfer size
    pSdoComCon_p->transferSize = lastCmdSegmSize;
    pSdoComCon_p->transferredBytes = 0;

    //NOTE: No use to save paMultiAcc, but same effort for NULL assignment
    pSdoComCon_p->paMultiAcc = (tSdoMultiAccEntry*)pSdoComTransParam_p->paMultiAcc;

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Set up a Read Multiple Parameters by Index frame

The function creates a ReadMultParam command frame. This function is called
once to set up the whole frame, and adds all entries into a caller-provided buffer.

\param[in,out]  pSdoComCon_p        Pointer to SDO command layer connection structure
\param[in]      pSdoComTransParam_p Pointer to transfer command parameters
\param[in]      pFrameBuf_p         Pointer to command layer payload buffer
\param[in]      bufSize_p           Size of command layer payload buffer

\return The function returns a tOplkError error code.
\retval kErrorNoResource:               Provided buffer is invalid or too small
\retval kErrorSdoComInvalidServiceType: Only one sub-request -> use standard transfer!
*/
//------------------------------------------------------------------------------
static tOplkError prepareMultiRead(tSdoComCon* pSdoComCon_p,
                                   const tSdoComTransParamByIndex* pSdoComTransParam_p,
                                   void* pFrameBuf_p,
                                   size_t bufSize_p)
{
    tOplkError              ret = kErrorOk;
    tAsySdoComReadMultReq*  pCurSubHdr = NULL;
    tSdoMultiAccEntry*      pCurSubAcc = NULL;
    size_t                  loopCnt = 0;
    UINT                    subAccCnt = pSdoComTransParam_p->multiAccCnt;

    // check if command layer is idle
    if (pSdoComCon_p->transferredBytes > 0)
        return kErrorSdoComHandleBusy;

    // check provided buffer
    if ((pFrameBuf_p == NULL) ||
        (bufSize_p < SDO_CMD_SEGM_TX_MAX_SIZE) ||
        (SDO_CMD_SEGM_TX_MAX_SIZE < (sizeof(tAsySdoComReadMultReq) * subAccCnt)))
    {
        // bad buffer provided by API
        return kErrorNoResource;
    }

    if (subAccCnt == 1)
    {
        // only one object access requested -> use normal write by index transfer
        return kErrorSdoComInvalidServiceType;
    }

    pCurSubHdr = (tAsySdoComReadMultReq*)pFrameBuf_p;
    pCurSubAcc = (tSdoMultiAccEntry*)pSdoComTransParam_p->paMultiAcc;

    // loop over all sub accesses to build the frame
    for (; loopCnt < subAccCnt; loopCnt++)
    {
        if ((pCurSubAcc->index == 0) ||
            (pCurSubAcc->pData_le == NULL) ||
            (pCurSubAcc->dataSize == 0))
            return kErrorApiInvalidParam;

        ami_setUint16Le(&pCurSubHdr->index, pCurSubAcc->index);
        ami_setUint8Le(&pCurSubHdr->subIndex, pCurSubAcc->subIndex);
        ami_setUint8Le(&pCurSubHdr->reserved, 0);

        pCurSubHdr++;
        pCurSubAcc++;
    }

    pSdoComCon_p->paMultiAcc = (tSdoMultiAccEntry*)pSdoComTransParam_p->paMultiAcc;
    pSdoComCon_p->multiAccCnt = subAccCnt;

    // save Tx transfer size
    pSdoComCon_p->transferSize = sizeof(tAsySdoComReadMultReq) * subAccCnt;
    pSdoComCon_p->transferredBytes = 0;

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Process an SDO command layer frame on an SDO client

The function processes a received command layer frame on an SDO client.

\param[in]      sdoComConHdl_p      Handle of sequence layer connection.
\param[in]      pSdoCom_p           Pointer to received frame.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError processFrame(tSdoComConHdl sdoComConHdl_p,
                               const tAsySdoCom* pSdoCom_p)
{
    tOplkError                  ret = kErrorOk;
    UINT8                       flags;
    UINT8                       transactionId;
    UINT8                       command;
    size_t                      segmentSize;
    size_t                      dataSize;
    size_t                      transferSize;
    tSdoComCon*                 pSdoComCon;
    tAsySdoComWriteMultResp*    pMultWriteResp;
    UINT                        multWriteRespCnt = 0;

    // get pointer to control structure
    pSdoComCon = &sdoComInstance_g.sdoComCon[sdoComConHdl_p];

    transactionId = ami_getUint8Le(&pSdoCom_p->transactionId);
    if (pSdoComCon->transactionId != transactionId)
    {
        // if running transfer
        if ((pSdoComCon->transferredBytes != 0) && (pSdoComCon->transferSize != 0))
        {
            pSdoComCon->lastAbortCode = SDO_AC_GENERAL_ERROR;
            sendSdoAbort(pSdoComCon, pSdoComCon->lastAbortCode);
            ret = transferFinished(sdoComConHdl_p, pSdoComCon, kSdoComTransferTxAborted);
        }
    }
    else
    {
        // check if correct command
        command = ami_getUint8Le(&pSdoCom_p->commandId);
        if (pSdoComCon->sdoServiceType != command)
        {
            // incorrect command
            // if running transfer
            if ((pSdoComCon->transferredBytes != 0) && (pSdoComCon->transferSize != 0))
            {
                pSdoComCon->lastAbortCode = SDO_AC_GENERAL_ERROR;
                sendSdoAbort(pSdoComCon, pSdoComCon->lastAbortCode);
                ret = transferFinished(sdoComConHdl_p, pSdoComCon, kSdoComTransferTxAborted);
            }
        }
        else
        {
            // switch on command
            switch (pSdoComCon->sdoServiceType)
            {
                case kSdoServiceWriteByIndex:
                    // check if confirmation from server
                    // nothing more to do
                    break;

                case kSdoServiceWriteMultiByIndex:
                    // process multi-abort response
                    flags = ami_getUint8Le(&pSdoCom_p->flags);
                    flags &= SDO_CMDL_FLAG_SEGM_MASK;
                    switch (flags)
                    {
                        case SDO_CMDL_FLAG_EXPEDITED:
                            if (pSdoCom_p->segmentSizeLe >= 4)
                            {
                                // there are multi-aborts
                                pMultWriteResp = (tAsySdoComWriteMultResp*)&pSdoCom_p->aCommandData[0];
                                multWriteRespCnt = pSdoCom_p->segmentSizeLe / sizeof(tAsySdoComWriteMultResp);

                                for (; multWriteRespCnt > 0; multWriteRespCnt--)
                                {
                                    if (pMultWriteResp->abortFlag & 0x80)
                                    {
                                        pSdoComCon->targetIndex = ami_getUint16Le(&pMultWriteResp->index);
                                        pSdoComCon->targetSubIndex = ami_getUint8Le(&pMultWriteResp->subIndex);
                                        pSdoComCon->lastAbortCode = ami_getUint32Le(&pMultWriteResp->subAbortCode);
                                        transferFinished(sdoComConHdl_p, pSdoComCon, kSdoComTransferRxSubAborted);
                                    }
                                    pMultWriteResp++;
                                }

                                pSdoComCon->targetIndex = 0;
                                pSdoComCon->targetSubIndex = 0;
                                pSdoComCon->lastAbortCode = 0;
                                ret = transferFinished(sdoComConHdl_p, pSdoComCon, kSdoComTransferFinished);
                            }
                            break;

                        default:
                            // segmented access is not allowed
                            pSdoComCon->lastAbortCode = SDO_AC_UNSUPPORTED_ACCESS;
                            sendSdoAbort(pSdoComCon, pSdoComCon->lastAbortCode);
                            ret = transferFinished(sdoComConHdl_p, pSdoComCon, kSdoComTransferFinished);
                            break;
                    }
                    break;

                case kSdoServiceReadByIndex:
                    flags = ami_getUint8Le(&pSdoCom_p->flags);
                    flags &= SDO_CMDL_FLAG_SEGM_MASK;
                    switch (flags)
                    {
                        case SDO_CMDL_FLAG_EXPEDITED:
                            // re-init transfer sizes utilized by preceding request
                            pSdoComCon->transferredBytes = 0;
                            // check size of buffer
                            segmentSize = ami_getUint16Le(&pSdoCom_p->segmentSizeLe);
                            if (segmentSize > pSdoComCon->transferSize)
                            {
                                // buffer provided by the application is too small -> copy only a part
                                dataSize = pSdoComCon->transferSize;
                            }
                            else
                            {
                                // buffer fits
                                dataSize = segmentSize;
                            }

                            pSdoComCon->sdoTransferType = kSdoTransExpedited;

                            OPLK_MEMCPY(pSdoComCon->pData, &pSdoCom_p->aCommandData[0], dataSize);
                            sdocomint_updateHdlTransfSize(pSdoComCon, dataSize, TRUE);
                            break;

                        case SDO_CMDL_FLAG_SEGMINIT:
                            // get total size of transfer including the header
                            transferSize = (size_t)ami_getUint32Le(&pSdoCom_p->aCommandData[0]);
                            transferSize -= SDO_CMDL_HDR_VAR_SIZE;
                            if (transferSize <= pSdoComCon->transferSize)
                            {
                                // buffer fits
                                // re-init transfer sizes utilized by preceding request
                                pSdoComCon->transferSize = transferSize;
                                pSdoComCon->transferredBytes = 0;
                            }
                            else
                            {
                                // buffer too small -> send abort
                                pSdoComCon->lastAbortCode = SDO_AC_DATA_TYPE_LENGTH_TOO_HIGH;
                                sendSdoAbort(pSdoComCon, pSdoComCon->lastAbortCode);
                                ret = transferFinished(sdoComConHdl_p, pSdoComCon, kSdoComTransferTxAborted);
                                return ret;
                            }

                            pSdoComCon->sdoTransferType = kSdoTransSegmented;

                            // get segment size
                            // check size of buffer
                            segmentSize = (size_t)ami_getUint16Le(&pSdoCom_p->segmentSizeLe);
                            segmentSize -= SDO_CMDL_HDR_VAR_SIZE;
                            OPLK_MEMCPY(pSdoComCon->pData, &pSdoCom_p->aCommandData[SDO_CMDL_HDR_VAR_SIZE], segmentSize);
                            sdocomint_updateHdlTransfSize(pSdoComCon, segmentSize, FALSE);
                            break;

                        case SDO_CMDL_FLAG_SEGMENTED:
                            // get segment size
                            // check size of buffer
                            segmentSize = ami_getUint16Le(&pSdoCom_p->segmentSizeLe);
                            // check if data to copy fit to buffer
                            if (segmentSize > pSdoComCon->transferSize)
                            {
                                // segment too large -> send abort
                                pSdoComCon->lastAbortCode = SDO_AC_INVALID_BLOCK_SIZE;
                                sendSdoAbort(pSdoComCon, pSdoComCon->lastAbortCode);
                                ret = transferFinished(sdoComConHdl_p, pSdoComCon, kSdoComTransferTxAborted);
                                return ret;
                            }
                            OPLK_MEMCPY(pSdoComCon->pData, &pSdoCom_p->aCommandData[0], segmentSize);
                            sdocomint_updateHdlTransfSize(pSdoComCon, segmentSize, FALSE);
                            break;

                        case SDO_CMDL_FLAG_SEGMCOMPL:
                            // get segment size
                            // check size of buffer
                            segmentSize = ami_getUint16Le(&pSdoCom_p->segmentSizeLe);
                            // check if data to copy fit to buffer
                            if (segmentSize > pSdoComCon->transferSize)
                            {
                                // segment too large -> send abort
                                pSdoComCon->lastAbortCode = SDO_AC_INVALID_BLOCK_SIZE;
                                sendSdoAbort(pSdoComCon, pSdoComCon->lastAbortCode);
                                ret = transferFinished(sdoComConHdl_p, pSdoComCon, kSdoComTransferTxAborted);
                                return ret;
                            }
                            OPLK_MEMCPY(pSdoComCon->pData, &pSdoCom_p->aCommandData[0], segmentSize);
                            sdocomint_updateHdlTransfSize(pSdoComCon, segmentSize, TRUE);
                            break;
                    }
                    break;

                case kSdoServiceReadMultiByIndex:
                    // process multi-read response
                    flags = ami_getUint8Le(&pSdoCom_p->flags);
                    flags &= SDO_CMDL_FLAG_SEGM_MASK;
                    switch (flags)
                    {
                        case SDO_CMDL_FLAG_EXPEDITED:
                            // re-init transfer sizes utilized by preceding request
                            pSdoComCon->transferredBytes = 0;

                            ret = processMultiReadResp(sdoComConHdl_p, pSdoComCon, pSdoCom_p);
                            if (ret != kErrorOk)
                                return ret;

                            pSdoComCon->targetIndex = 0;
                            pSdoComCon->targetSubIndex = 0;
                            pSdoComCon->lastAbortCode = 0;
                            ret = transferFinished(sdoComConHdl_p, pSdoComCon, kSdoComTransferFinished);
                            break;

                        default:
                            // segmented access is not allowed
                            pSdoComCon->lastAbortCode = SDO_AC_UNSUPPORTED_ACCESS;
                            sendSdoAbort(pSdoComCon, pSdoComCon->lastAbortCode);
                            ret = transferFinished(sdoComConHdl_p, pSdoComCon, kSdoComTransferFinished);
                            break;
                    }
                    break;

                case kSdoServiceNIL:
                default:
                    // invalid service requested
                    // $$$ d.k. What should we do?
                    break;
            }
        }
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Process an SDO Multi Read Parameters by Index command response

The function processes a received SDO Multi Read Parameters by Index command
layer frame.

\param[in]      sdoComConHdl_p      Handle of sequence layer connection
\param[in,out]  pSdoComCon_p        Pointer to sequence layer connection
\param[in]      pSdoCom_p           Pointer to received frame

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError processMultiReadResp(tSdoComConHdl sdoComConHdl_p,
                                       tSdoComCon* pSdoComCon_p,
                                       const tAsySdoCom* pSdoCom_p)
{
    tOplkError                              ret = kErrorOk;
    const tAsySdoComMultWriteReqReadResp*   pCurSubHdr;
    const tAsySdoComMultWriteReqReadResp*   pNextSubHdr;
    BOOL                                    fLastDataSetReached = FALSE;

    pSdoComCon_p->respSegmSize = ami_getUint16Le(&pSdoCom_p->segmentSizeLe);
    pCurSubHdr = (const tAsySdoComMultWriteReqReadResp*)&pSdoCom_p->aCommandData[0];

    while (!fLastDataSetReached)
    {
        pNextSubHdr = (const tAsySdoComMultWriteReqReadResp*)((const UINT8*)pSdoCom_p + pCurSubHdr->byteOffsetNext);

        // save service
        pSdoComCon_p->targetIndex = ami_getUint16Le(&pCurSubHdr->index);
        pSdoComCon_p->targetSubIndex = ami_getUint8Le(&pCurSubHdr->subIndex);

        if (0 != pCurSubHdr->byteOffsetNext)
        {
            pSdoComCon_p->pendingTransferSize = (const UINT8*)pNextSubHdr - (const UINT8*)pCurSubHdr - \
                                                 SDO_CMDL_HDR_WRITEMULTBYINDEX_SIZE - \
                                                 (pCurSubHdr->info & SDO_CMDL_FLAG_PADSIZE_MASK);
        }
        else
        {   // it is the last data set thus another calculation method is used
            pSdoComCon_p->pendingTransferSize = (const UINT8*)&pSdoCom_p->aCommandData[0] + pSdoComCon_p->respSegmSize - \
                                                ((const UINT8*)pCurSubHdr + SDO_CMDL_HDR_WRITEMULTBYINDEX_SIZE) - \
                                                (pCurSubHdr->info & SDO_CMDL_FLAG_PADSIZE_MASK);
        }

        if (pCurSubHdr->info & 0x80)
        {   // send sub-abort to application
            pSdoComCon_p->lastAbortCode = ami_getUint32Le(&pCurSubHdr->aCommandData[0]);
            ret = transferFinished(sdoComConHdl_p, pSdoComCon_p, kSdoComTransferRxSubAborted);
            if (ret != kErrorOk)
                return ret;
        }
        else
        {   // send data to application
            ret = copyToMultiBuffer(pSdoComCon_p, &pCurSubHdr->aCommandData[0]);
            if (ret != kErrorOk)
            {
                pSdoComCon_p->lastAbortCode = SDO_AC_DATA_NOT_TRANSF_OR_STORED;
                ret = transferFinished(sdoComConHdl_p, pSdoComCon_p, kSdoComTransferRxSubAborted);
                if (ret != kErrorOk)
                    return ret;
            }
        }

        if (0 != pCurSubHdr->byteOffsetNext)
        {
            // increment to next object
            pCurSubHdr = pNextSubHdr;
            if ((const UINT8*)pNextSubHdr > ((const UINT8*)&pSdoCom_p->aCommandData[0] + pSdoComCon_p->respSegmSize))
            {   // frame overrun, bad response frame!
                sendSdoAbort(pSdoComCon_p, SDO_AC_INVALID_BLOCK_SIZE);
                ret = transferFinished(sdoComConHdl_p, pSdoComCon_p, kSdoComTransferFinished);
                if (ret != kErrorOk)
                    return ret;
            }
        }
        else
        {   // this was the last object -> exit loop
           fLastDataSetReached = TRUE;
        }
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Copy data to user provided multi read buffer

The function searches the user provided multi read access array for a
corresponding object entry and copies the data to this provided buffer in little
endian format.

\param[in]      pSdoComCon_p    Pointer to SDO command layer connection structure.
\param[in]      pSrcData_p      Pointer to the source data

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError copyToMultiBuffer(const tSdoComCon* pSdoComCon_p,
                                    const void* pSrcData_p)
{
    tSdoMultiAccEntry*  pCurSubAcc = NULL;
    size_t              loopCnt = 0;
    UINT                subAccCnt = 0;

    pCurSubAcc = pSdoComCon_p->paMultiAcc;
    subAccCnt = pSdoComCon_p->multiAccCnt;

    // check size of buffer
    for (; loopCnt < subAccCnt; loopCnt++)
    {
        if ((pSdoComCon_p->targetIndex == pCurSubAcc->index) &&
            (pSdoComCon_p->targetSubIndex == pCurSubAcc->subIndex))
        {
            // check buffer size
            if (pCurSubAcc->dataSize < pSdoComCon_p->pendingTransferSize)
            {
                // buffer too small
                return kErrorNoResource;
            }

            OPLK_MEMCPY(pCurSubAcc->pData_le, pSrcData_p, pSdoComCon_p->pendingTransferSize);
            // update real size
            pCurSubAcc->dataSize = (UINT)pSdoComCon_p->pendingTransferSize;
            return kErrorOk;
        }

        pCurSubAcc++;
    }

    // no corresponding entry found
    return kErrorNoResource;
}

//------------------------------------------------------------------------------
/**
\brief  Send an SDO command layer frame from an SDO client

The function starts an SDO transfer and sends all further frames.

\param[in,out]  pSdoComCon_p        Pointer to SDO command layer connection structure.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError sendSdo(tSdoComCon* pSdoComCon_p)
{
    tOplkError      ret = kErrorOk;
    UINT8           aFrame[SDO_MAX_TX_FRAME_SIZE];
    tPlkFrame*      pFrame;
    tAsySdoCom*     pCommandFrame;
    size_t          sizeOfCmdFrame;
    size_t          sizeOfCmdData;
    UINT8*          pPayload;

    pFrame = (tPlkFrame*)&aFrame[0];
    sdocomint_initCmdFrameGeneric(pFrame, sizeof(aFrame), pSdoComCon_p, &pCommandFrame);

    // check if first frame to send -> command header needed
    if (pSdoComCon_p->transferSize > 0)
    {
        if (pSdoComCon_p->transferredBytes == 0)
        {   // start SDO transfer
            // check if segmented or expedited transfer
            // only for write commands
            switch (pSdoComCon_p->sdoServiceType)
            {
                case kSdoServiceReadByIndex:
                    // first frame of read access always expedited
                    pSdoComCon_p->sdoTransferType = kSdoTransExpedited;
                    pPayload = &pCommandFrame->aCommandData[0];
                    ami_setUint16Le(&pCommandFrame->segmentSizeLe, SDO_CMDL_HDR_READBYINDEX_SIZE);
                    ami_setUint16Le(pPayload, pSdoComCon_p->targetIndex);
                    pPayload += 2;
                    ami_setUint8Le(pPayload, pSdoComCon_p->targetSubIndex);
                    sizeOfCmdFrame = SDO_CMDL_HDR_FIXED_SIZE + SDO_CMDL_HDR_READBYINDEX_SIZE;
                    // size is reinitialized to 0 for the response frame later, but we need to
                    // set a value != 0 here to indicate a started transfer
                    pSdoComCon_p->transferredBytes = SDO_CMDL_HDR_READBYINDEX_SIZE;
                    break;

                case kSdoServiceWriteByIndex:
                    if (pSdoComCon_p->transferSize > (SDO_CMD_SEGM_TX_MAX_SIZE - SDO_CMDL_HDR_WRITEBYINDEX_SIZE))
                    {
                        // segmented transfer -> variable part of header needed
                        pSdoComCon_p->sdoTransferType = kSdoTransSegmented;
                        ami_setUint16Le(&pCommandFrame->segmentSizeLe, SDO_CMD_SEGM_TX_MAX_SIZE);
                        sdocomint_overwriteCmdFrameHdrFlags(pCommandFrame, SDO_CMDL_FLAG_SEGMINIT);
                        ami_setUint32Le(&pCommandFrame->aCommandData[0], (UINT32)(pSdoComCon_p->transferSize + SDO_CMDL_HDR_FIXED_SIZE));
                        pPayload = &pCommandFrame->aCommandData[SDO_CMDL_HDR_VAR_SIZE];
                        ami_setUint16Le(pPayload, pSdoComCon_p->targetIndex);
                        pPayload += 2;
                        ami_setUint8Le(pPayload, pSdoComCon_p->targetSubIndex);
                        pPayload += 2;      // on byte for reserved
                        sizeOfCmdFrame = SDO_CMDL_HDR_FIXED_SIZE + SDO_CMD_SEGM_TX_MAX_SIZE;
                        sizeOfCmdData = SDO_CMD_SEGM_TX_MAX_SIZE - (SDO_CMDL_HDR_VAR_SIZE + SDO_CMDL_HDR_WRITEBYINDEX_SIZE);
                        OPLK_MEMCPY(pPayload, pSdoComCon_p->pData, sizeOfCmdData);
                        pSdoComCon_p->pendingTxBytes = sizeOfCmdData;
                    }
                    else
                    {
                        // expedited transfer
                        pSdoComCon_p->sdoTransferType = kSdoTransExpedited;
                        ami_setUint16Le(&pCommandFrame->segmentSizeLe, (UINT16)(pSdoComCon_p->transferSize + SDO_CMDL_HDR_WRITEBYINDEX_SIZE));
                        pPayload = &pCommandFrame->aCommandData[0];
                        ami_setUint16Le(pPayload, pSdoComCon_p->targetIndex);
                        pPayload += 2;
                        ami_setUint8Le(pPayload, pSdoComCon_p->targetSubIndex);
                        pPayload += 2;      // + 2 -> one byte for sub index and one byte reserved
                        sizeOfCmdFrame = SDO_CMDL_HDR_FIXED_SIZE + (pSdoComCon_p->transferSize + SDO_CMDL_HDR_WRITEBYINDEX_SIZE);
                        OPLK_MEMCPY(pPayload, pSdoComCon_p->pData, pSdoComCon_p->transferSize);
                    }
                    break;

                case kSdoServiceWriteMultiByIndex:
                case kSdoServiceReadMultiByIndex:
                    if (pSdoComCon_p->transferSize > SDO_CMD_SEGM_TX_MAX_SIZE)
                    {   // segmented transfer -> variable part of header needed
                        // -> not supported!
                        sdocomint_updateHdlTransfSize(pSdoComCon_p, 0, TRUE);
                        return kErrorSdoComInvalidParam;
                    }
                    else
                    {   // expedited transfer
                        pSdoComCon_p->sdoTransferType = kSdoTransExpedited;
                        ami_setUint16Le(&pCommandFrame->segmentSizeLe, (UINT16)(pSdoComCon_p->transferSize));
                        pPayload = &pCommandFrame->aCommandData[0];
                        sizeOfCmdFrame = SDO_CMDL_HDR_FIXED_SIZE + pSdoComCon_p->transferSize;
                        OPLK_MEMCPY(pPayload, pSdoComCon_p->pData, pSdoComCon_p->transferSize);
                    }
                    break;

                case kSdoServiceNIL:
                default:
                    // invalid service requested
                    return kErrorSdoComInvalidServiceType;
            }
        }
        else
        {
            // continue SDO transfer
            switch (pSdoComCon_p->sdoServiceType)
            {
                case kSdoServiceWriteByIndex:
                    // send next frame
                    if (pSdoComCon_p->sdoTransferType == kSdoTransSegmented)
                    {
                        if (pSdoComCon_p->transferSize > SDO_CMD_SEGM_TX_MAX_SIZE)
                        {
                            // next segment
                            sizeOfCmdData = SDO_CMD_SEGM_TX_MAX_SIZE;
                            sdocomint_fillCmdFrameDataSegm(pCommandFrame, pSdoComCon_p->pData, sizeOfCmdData);
                            sdocomint_overwriteCmdFrameHdrFlags(pCommandFrame, SDO_CMDL_FLAG_SEGMENTED);
                            sdocomint_setCmdFrameHdrSegmSize(pCommandFrame, sizeOfCmdData);
                            sizeOfCmdFrame = SDO_CMDL_HDR_FIXED_SIZE + sizeOfCmdData;
                            pSdoComCon_p->pendingTxBytes = sizeOfCmdData;
                        }
                        else
                        {
                            // end of transfer
                            sizeOfCmdData = pSdoComCon_p->transferSize;
                            sdocomint_fillCmdFrameDataSegm(pCommandFrame, pSdoComCon_p->pData, sizeOfCmdData);
                            sdocomint_overwriteCmdFrameHdrFlags(pCommandFrame, SDO_CMDL_FLAG_SEGMCOMPL);
                            sdocomint_setCmdFrameHdrSegmSize(pCommandFrame, sizeOfCmdData);
                            sizeOfCmdFrame = SDO_CMDL_HDR_FIXED_SIZE + sizeOfCmdData;
                            pSdoComCon_p->pendingTxBytes = sizeOfCmdData;
                        }
                    }
                    else
                        return ret;
                    break;

                // for expedited read is nothing to do -> server sends data
                default:
                    return ret;
            }
        }
    }
    else
        return ret;

    // call send function of lower layer
    switch (pSdoComCon_p->sdoProtocolType)
    {
        case kSdoTypeAsnd:
        case kSdoTypeUdp:
            ret = sdoseq_sendData(pSdoComCon_p->sdoSeqConHdl, sizeOfCmdFrame, pFrame);
            break;

        default:
            return kErrorSdoComUnsupportedProt;
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Update the SDO command layer connection structure size fields after Tx

The function updates the SDO command layer connection structure size related
members. It has to be called after a successful send (or storage in the history buffer)
of command layer data sent by the client. For certain transfer types
pSdoComCon_p->pendingTxBytes which was set before the actual transmission
happened, is used to update the connection structure.

\param[in,out]  pSdoComCon_p    Pointer to SDO command layer connection structure

*/
//------------------------------------------------------------------------------
static void updateTransferAfterTx(tSdoComCon* pSdoComCon_p)
{
    if (pSdoComCon_p->transferSize > 0)
    {
        if (pSdoComCon_p->transferredBytes == 0)
        {
            switch (pSdoComCon_p->sdoServiceType)
            {
                case kSdoServiceWriteByIndex:
                    if (pSdoComCon_p->transferSize > (SDO_CMD_SEGM_TX_MAX_SIZE - SDO_CMDL_HDR_WRITEBYINDEX_SIZE))
                    {
                        // segmented transfer
                        sdocomint_updateHdlTransfSize(pSdoComCon_p, pSdoComCon_p->pendingTxBytes, FALSE);
                        pSdoComCon_p->pendingTxBytes = 0;
                    }
                    else  // expedited transfer
                        sdocomint_updateHdlTransfSize(pSdoComCon_p, pSdoComCon_p->transferSize, TRUE);

                    break;

                case kSdoServiceWriteMultiByIndex:
                case kSdoServiceReadMultiByIndex:
                    if (pSdoComCon_p->transferSize <= SDO_CMD_SEGM_TX_MAX_SIZE) // expedited transfer
                        sdocomint_updateHdlTransfSize(pSdoComCon_p, pSdoComCon_p->transferSize, TRUE);

                    break;

                default:
                    break;
            }
        }
        else
        {
            // continue SDO transfer
            if (pSdoComCon_p->sdoServiceType == kSdoServiceWriteByIndex &&
                (pSdoComCon_p->sdoTransferType == kSdoTransSegmented))
            {
                if (pSdoComCon_p->transferSize > SDO_CMD_SEGM_TX_MAX_SIZE) // next segment
                    sdocomint_updateHdlTransfSize(pSdoComCon_p, pSdoComCon_p->pendingTxBytes, FALSE);
                else  // end of transfer
                    sdocomint_updateHdlTransfSize(pSdoComCon_p, pSdoComCon_p->pendingTxBytes, TRUE);
            }
        }
    }
}

//------------------------------------------------------------------------------
/**
\brief  Send an abort message

The function sends an abort message on an SDO client.

\param[in,out]  pSdoComCon_p        Pointer to SDO command layer connection structure.
\param[in]      abortCode_p         Abort code to send.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError sendSdoAbort(tSdoComCon* pSdoComCon_p, UINT32 abortCode_p)
{
    tOplkError      ret = kErrorOk;
    UINT8           aFrame[SDO_MAX_TX_FRAME_SIZE];
    tPlkFrame*      pFrame;
    tAsySdoCom*     pCommandFrame;
    size_t          sizeOfCmdFrame;
    size_t          sizeOfCmdData;

    pFrame = (tPlkFrame*)&aFrame[0];
    sdocomint_initCmdFrameGeneric(pFrame, sizeof(aFrame), pSdoComCon_p, &pCommandFrame);

    sizeOfCmdData = sizeof(UINT32);
    // copy abort code to frame
    ami_setUint32Le(&pCommandFrame->aCommandData[0], abortCode_p);
    sdocomint_setCmdFrameHdrFlag(pCommandFrame, SDO_CMDL_FLAG_ABORT);
    sdocomint_setCmdFrameHdrSegmSize(pCommandFrame, sizeOfCmdData);

    sdocomint_updateHdlTransfSize(pSdoComCon_p, sizeOfCmdData, TRUE);
    sizeOfCmdFrame = SDO_CMDL_HDR_FIXED_SIZE + sizeOfCmdData;
    pSdoComCon_p->lastAbortCode = abortCode_p;

    // call send function of lower layer
    switch (pSdoComCon_p->sdoProtocolType)
    {
        case kSdoTypeAsnd:
        case kSdoTypeUdp:
            ret = sdoseq_sendData(pSdoComCon_p->sdoSeqConHdl, sizeOfCmdFrame, pFrame);
            if (ret == kErrorSdoSeqConnectionBusy)
            {
                DEBUG_LVL_SDO_TRACE("%s tried to send abort 0x%lX while connection is already closed\n",
                                    __func__,
                                    (UINT32)abortCode_p);
                ret = kErrorOk;
            }
            break;

        default:
            ret = kErrorSdoComUnsupportedProt;
            break;
    }
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Finish SDO transfer

The function finishes an SDO transfer by calling the applications callback
function.

\param[in]      sdoComConHdl_p      Handle of sequence layer connection.
\param[in,out]  pSdoComCon_p        Pointer to SDO command layer connection structure.
\param[in]      sdoComConState_p    Connection state of the SDO transfer.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError transferFinished(tSdoComConHdl sdoComConHdl_p,
                                   tSdoComCon* pSdoComCon_p,
                                   tSdoComConState sdoComConState_p)
{
    tOplkError      ret = kErrorOk;
    tSdoFinishedCb  pfnTransferFinished;
    tSdoComFinished sdoComFinished;

    if (pSdoComCon_p->pfnTransferFinished != NULL)
    {
        sdoComFinished.pUserArg = pSdoComCon_p->pUserArg;
        sdoComFinished.nodeId = pSdoComCon_p->nodeId;
        sdoComFinished.targetIndex = pSdoComCon_p->targetIndex;
        sdoComFinished.targetSubIndex = pSdoComCon_p->targetSubIndex;
        sdoComFinished.transferredBytes = (UINT)pSdoComCon_p->transferredBytes;
        sdoComFinished.abortCode = pSdoComCon_p->lastAbortCode;
        sdoComFinished.sdoComConHdl = sdoComConHdl_p;
        sdoComFinished.sdoComConState = sdoComConState_p;
        sdoComFinished.multiSubAccCnt = 0;

        switch (pSdoComCon_p->sdoServiceType)
        {
            case kSdoServiceWriteByIndex:
                sdoComFinished.sdoAccessType = kSdoAccessTypeWrite;
                break;

            case kSdoServiceReadByIndex:
                sdoComFinished.sdoAccessType = kSdoAccessTypeRead;
                break;

            case kSdoServiceWriteMultiByIndex:
                sdoComFinished.sdoAccessType = kSdoAccessTypeMultiWrite;
                sdoComFinished.multiSubAccCnt = pSdoComCon_p->multiAccCnt;
                break;

            case kSdoServiceReadMultiByIndex:
                sdoComFinished.sdoAccessType = kSdoAccessTypeMultiRead;
                sdoComFinished.multiSubAccCnt = pSdoComCon_p->multiAccCnt;
                break;

            default:
                return kErrorSdoComInvalidParam;
        }

        pfnTransferFinished = pSdoComCon_p->pfnTransferFinished;

        // Each kSdoServiceWriteMultiByIndex and kSdoServiceReadMultiByIndex
        // sub-abort using kSdoComTransferRxSubAborted should not count as final abort.
        // The final abort is a kSdoComTransferFinished event in this case.
        if (!(((pSdoComCon_p->sdoServiceType == kSdoServiceWriteMultiByIndex) ||
              (pSdoComCon_p->sdoServiceType == kSdoServiceReadMultiByIndex)) &&
              (sdoComConState_p == kSdoComTransferRxSubAborted)))
        {
            // reset transfer state so this handle is not busy anymore
            pSdoComCon_p->transferredBytes = 0;
            pSdoComCon_p->transferSize = 0;
            // delete function pointer to inform application only once for each transfer
            pSdoComCon_p->pfnTransferFinished = NULL;
        }

        // call application's callback function
        pfnTransferFinished(&sdoComFinished);
        // reset abort code
        pSdoComCon_p->lastAbortCode = 0;
    }

    return ret;
}

/// \}

#endif // defined(CONFIG_INCLUDE_SDOC)
