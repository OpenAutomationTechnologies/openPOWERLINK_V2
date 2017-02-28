/**
********************************************************************************
\file   sdocomclt.c

\brief  Implementation of SDO Command Layer client

This file contains the standard command layer implementation of the SDO client.

\ingroup module_sdocom_std
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2017, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
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
static tOplkError clientSend(tSdoComCon* pSdoComCon_p);
static tOplkError clientProcessFrame(tSdoComConHdl sdoComConHdl_p,
                                     const tAsySdoCom* pSdoCom_p);
static tOplkError clientSendAbort(tSdoComCon* pSdoComCon_p,
                                  UINT32 abortCode_p);
static tOplkError clientTransferFinished(tSdoComConHdl sdoComConHdl_p,
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
If this function detects such a situation it will return
kErrorSdoComHandleExists and the handle of the existing connection in
pSdoComConHdl_p.

\param[out]     pSdoComConHdl_p     Pointer to store the connection handle.
\param[in]      targetNodeId_p      The node ID to connect to.
\param[in]      protType_p          The protocol type to use for the connection
                                    (UDP and ASnd is supported)

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
tOplkError clientSdoDefineConnection(tSdoComConHdl* pSdoComConHdl_p,
                                     UINT targetNodeId_p,
                                     tSdoType protType_p)
{
    tOplkError  ret;
    UINT        count;
    UINT        freeHdl;
    tSdoComCon* pSdoComCon;

    if ((targetNodeId_p == C_ADR_INVALID) || (targetNodeId_p >= C_ADR_BROADCAST))
        return kErrorInvalidNodeId;

    // search free control structure
    pSdoComCon = &sdoComInstance_l.sdoComCon[0];
    count = 0;
    freeHdl = CONFIG_SDO_MAX_CONNECTION_COM;
    while (count < CONFIG_SDO_MAX_CONNECTION_COM)
    {
        if (pSdoComCon->sdoSeqConHdl == 0)
        {   // free entry
            freeHdl = count;
        }
        else if ((pSdoComCon->nodeId == targetNodeId_p) && (pSdoComCon->sdoProtocolType == protType_p))
        {   // existing client connection with same node ID and same protocol type
            *pSdoComConHdl_p = count;
            return kErrorSdoComHandleExists;
        }

        count++;
        pSdoComCon++;
    }

    if (freeHdl == CONFIG_SDO_MAX_CONNECTION_COM)
        return kErrorSdoComNoFreeHandle;

    *pSdoComConHdl_p = freeHdl;                 // save handle for application

    pSdoComCon = &sdoComInstance_l.sdoComCon[freeHdl];
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

    ret = processState(freeHdl, kSdoComConEventInitCon, NULL);
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
tOplkError clientSdoInitTransferByIndex(const tSdoComTransParamByIndex* pSdoComTransParam_p)
{
    tOplkError      ret;
    tSdoComCon*     pSdoComCon;

    if ((pSdoComTransParam_p->subindex >= 0xFF) ||
        (pSdoComTransParam_p->index == 0) ||
        (pSdoComTransParam_p->index > 0xFFFF) ||
        (pSdoComTransParam_p->pData == NULL) ||
        (pSdoComTransParam_p->dataSize == 0))
        return kErrorSdoComInvalidParam;

    if (pSdoComTransParam_p->sdoComConHdl >= CONFIG_SDO_MAX_CONNECTION_COM)
        return kErrorSdoComInvalidHandle;

    // get pointer to control structure of connection
    pSdoComCon = &sdoComInstance_l.sdoComCon[pSdoComTransParam_p->sdoComConHdl];

    if (pSdoComCon->sdoSeqConHdl == 0)
        return kErrorSdoComInvalidHandle;

    // check if command layer is idle
    if ((pSdoComCon->transferredBytes + pSdoComCon->transferSize) > 0)
        return kErrorSdoComHandleBusy;

    // callback function for end of transfer
    pSdoComCon->pfnTransferFinished = pSdoComTransParam_p->pfnSdoFinishedCb;
    pSdoComCon->pUserArg = pSdoComTransParam_p->pUserArg;

    if (pSdoComTransParam_p->sdoAccessType == kSdoAccessTypeRead)
        pSdoComCon->sdoServiceType = kSdoServiceReadByIndex;
    else
        pSdoComCon->sdoServiceType = kSdoServiceWriteByIndex;

    pSdoComCon->pData = (UINT8*)pSdoComTransParam_p->pData;     // save pointer to data
    pSdoComCon->transferSize = pSdoComTransParam_p->dataSize;   // maximal bytes to transfer
    pSdoComCon->transferredBytes = 0;                           // bytes already transfered
    pSdoComCon->lastAbortCode = 0;
    pSdoComCon->sdoTransferType = kSdoTransAuto;
    pSdoComCon->targetIndex = pSdoComTransParam_p->index;
    pSdoComCon->targetSubIndex = pSdoComTransParam_p->subindex;

    ret = processState(pSdoComTransParam_p->sdoComConHdl, kSdoComConEventSendFirst, NULL);

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
tOplkError clientSdoUndefineConnection(tSdoComConHdl sdoComConHdl_p)
{
    tOplkError  ret = kErrorOk;
    tSdoComCon* pSdoComCon;

    if (sdoComConHdl_p >= CONFIG_SDO_MAX_CONNECTION_COM)
        return kErrorSdoComInvalidHandle;

    // get pointer to control structure
    pSdoComCon = &sdoComInstance_l.sdoComCon[sdoComConHdl_p];

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
tOplkError clientSdoGetState(tSdoComConHdl sdoComConHdl_p,
                             tSdoComFinished* pSdoComFinished_p)
{
    tOplkError  ret = kErrorOk;
    tSdoComCon* pSdoComCon;

    if (sdoComConHdl_p >= CONFIG_SDO_MAX_CONNECTION_COM)
        return kErrorSdoComInvalidHandle;

    // get pointer to control structure
    pSdoComCon = &sdoComInstance_l.sdoComCon[sdoComConHdl_p];

    // check if handle ok
    if (pSdoComCon->sdoSeqConHdl == 0)
        return kErrorSdoComInvalidHandle;

    pSdoComFinished_p->pUserArg = pSdoComCon->pUserArg;
    pSdoComFinished_p->nodeId = pSdoComCon->nodeId;
    pSdoComFinished_p->targetIndex = pSdoComCon->targetIndex;
    pSdoComFinished_p->targetSubIndex = pSdoComCon->targetSubIndex;
    pSdoComFinished_p->transferredBytes = pSdoComCon->transferredBytes;
    pSdoComFinished_p->abortCode = pSdoComCon->lastAbortCode;
    pSdoComFinished_p->sdoComConHdl = sdoComConHdl_p;
    if (pSdoComCon->sdoServiceType == kSdoServiceWriteByIndex)
        pSdoComFinished_p->sdoAccessType = kSdoAccessTypeWrite;
    else
        pSdoComFinished_p->sdoAccessType = kSdoAccessTypeRead;

    if (pSdoComCon->lastAbortCode != 0)
    {   // sdo abort
        pSdoComFinished_p->sdoComConState = kSdoComTransferRxAborted;
        // delete abort code
        pSdoComCon->lastAbortCode = 0;
    }
    else if ((pSdoComCon->sdoSeqConHdl & ~SDO_SEQ_HANDLE_MASK) == SDO_SEQ_INVALID_HDL)
    {   // check state
        pSdoComFinished_p->sdoComConState = kSdoComTransferLowerLayerAbort;
    }
    else if (pSdoComCon->sdoComState == kSdoComStateClientWaitInit)
    {   // finished
        pSdoComFinished_p->sdoComConState = kSdoComTransferNotActive;
    }
    else if (pSdoComCon->transferSize == 0)
    {   // finished
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
UINT clientSdoGetNodeId(tSdoComConHdl sdoComConHdl_p)
{
    UINT        nodeId = C_ADR_INVALID;
    tSdoComCon* pSdoComCon;

    if (sdoComConHdl_p >= CONFIG_SDO_MAX_CONNECTION_COM)
        return nodeId;

    // get pointer to control structure
    pSdoComCon = &sdoComInstance_l.sdoComCon[sdoComConHdl_p];

    if (pSdoComCon->sdoSeqConHdl == 0)
        return nodeId;

    nodeId = pSdoComCon->nodeId;
    return nodeId;
}

//------------------------------------------------------------------------------
/**
\brief  Abort a SDO transfer

The function aborts an SDO transfer.

\param[in]      sdoComConHdl_p      Handle of the connection to abort.
\param[in]      abortCode_p         The abort code to use.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
tOplkError clientSdoAbortTransfer(tSdoComConHdl sdoComConHdl_p, UINT32 abortCode_p)
{
    tOplkError  ret;
    tSdoComCon* pSdoComCon;

    if (sdoComConHdl_p >= CONFIG_SDO_MAX_CONNECTION_COM)
        return kErrorSdoComInvalidHandle;

    // get pointer to control structure of connection
    pSdoComCon = &sdoComInstance_l.sdoComCon[sdoComConHdl_p];

    if (pSdoComCon->sdoSeqConHdl == 0)
        return kErrorSdoComInvalidHandle;

    pSdoComCon->pData = (UINT8*)&abortCode_p;
    ret = processState(sdoComConHdl_p, kSdoComConEventAbort, (tAsySdoCom*)NULL);

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
tOplkError clientProcessStateWaitInit(tSdoComConHdl sdoComConHdl_p,
                                      tSdoComConEvent sdoComConEvent_p,
                                      const tAsySdoCom* pRecvdCmdLayer_p)
{
    tOplkError  ret = kErrorOk;
    tSdoComCon* pSdoComCon;

    UNUSED_PARAMETER(pRecvdCmdLayer_p);

    pSdoComCon = &sdoComInstance_l.sdoComCon[sdoComConHdl_p];

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
            if ((pSdoComCon->transferSize > 0) && (pSdoComCon->targetIndex != 0))
            {   // start SDO transfer
                // check if segmented transfer
                if (pSdoComCon->sdoTransferType == kSdoTransSegmented)
                    pSdoComCon->sdoComState = kSdoComStateClientSegmTrans;
                else
                    pSdoComCon->sdoComState = kSdoComStateClientConnected;

                ret = clientSend(pSdoComCon);
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
            ret = clientTransferFinished(sdoComConHdl_p, pSdoComCon, kSdoComTransferTxAborted);
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

            ret = clientTransferFinished(sdoComConHdl_p, pSdoComCon, kSdoComTransferLowerLayerAbort);
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
tOplkError clientProcessStateConnected(tSdoComConHdl sdoComConHdl_p,
                                       tSdoComConEvent sdoComConEvent_p,
                                       const tAsySdoCom* pRecvdCmdLayer_p)
{

    tOplkError  ret = kErrorOk;
    UINT8       flag;
    tSdoComCon* pSdoComCon;

    pSdoComCon = &sdoComInstance_l.sdoComCon[sdoComConHdl_p];

    switch (sdoComConEvent_p)
    {
        // send a frame
        case kSdoComConEventSendFirst:
        case kSdoComConEventAckReceived:
        case kSdoComConEventFrameReceived:
        case kSdoComConEventFrameSent:
            ret = clientSend(pSdoComCon);
            if (ret != kErrorOk)
                return ret;

            // check if read transfer finished
            if ((pSdoComCon->transferSize == 0) && (pSdoComCon->transferredBytes != 0) &&
                (pSdoComCon->sdoServiceType == kSdoServiceReadByIndex))
            {
                pSdoComCon->transactionId++;
                pSdoComCon->lastAbortCode = 0;
                ret = clientTransferFinished(sdoComConHdl_p, pSdoComCon, kSdoComTransferFinished);
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
                    ret = clientTransferFinished(sdoComConHdl_p, pSdoComCon, kSdoComTransferRxAborted);
                    return ret;
                }
                else
                {   // normal frame received
                    ret = clientProcessFrame(sdoComConHdl_p, pRecvdCmdLayer_p);
                    // check if transfer ready
                    if (pSdoComCon->transferSize == 0)
                    {
                        // send acknowledge without any Command layer data
                        sdoseq_sendData(pSdoComCon->sdoSeqConHdl, 0, (tPlkFrame*)NULL);
                        pSdoComCon->transactionId++;
                        pSdoComCon->lastAbortCode = 0;
                        ret = clientTransferFinished(sdoComConHdl_p, pSdoComCon, kSdoComTransferFinished);
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
            ret = clientTransferFinished(sdoComConHdl_p, pSdoComCon, kSdoComTransferLowerLayerAbort);
            break;

        case kSdoComConEventAbort:
            clientSendAbort(pSdoComCon, *((UINT32*)pSdoComCon->pData));
            pSdoComCon->transactionId++;
            pSdoComCon->lastAbortCode = *((UINT32*)pSdoComCon->pData);
            ret = clientTransferFinished(sdoComConHdl_p, pSdoComCon, kSdoComTransferTxAborted);
            break;

        case kSdoComConEventInitError:
        case kSdoComConEventTimeout:
            sdoseq_deleteCon(pSdoComCon->sdoSeqConHdl);         // close sequence layer handle
            pSdoComCon->sdoSeqConHdl |= SDO_SEQ_INVALID_HDL;
            pSdoComCon->sdoComState = kSdoComStateClientWaitInit;
            pSdoComCon->lastAbortCode = SDO_AC_TIME_OUT;
            ret = clientTransferFinished(sdoComConHdl_p, pSdoComCon, kSdoComTransferLowerLayerAbort);
            break;

        case kSdoComConEventTransferAbort:
            pSdoComCon->sdoComState = kSdoComStateClientWaitInit;
            pSdoComCon->lastAbortCode = SDO_AC_TIME_OUT;
            ret = clientTransferFinished(sdoComConHdl_p, pSdoComCon, kSdoComTransferLowerLayerAbort);
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
tOplkError clientProcessStateSegmTransfer(tSdoComConHdl sdoComConHdl_p,
                                          tSdoComConEvent sdoComConEvent_p,
                                          const tAsySdoCom* pRecvdCmdLayer_p)
{
    tOplkError  ret = kErrorOk;
    UINT8       flag;
    tSdoComCon* pSdoComCon;

    pSdoComCon = &sdoComInstance_l.sdoComCon[sdoComConHdl_p];

    switch (sdoComConEvent_p)
    {
        case kSdoComConEventSendFirst:
        case kSdoComConEventAckReceived:
        case kSdoComConEventFrameReceived:
        case kSdoComConEventFrameSent:
            ret = clientSend(pSdoComCon);
            if (ret != kErrorOk)
                return ret;

            // check if read transfer finished
            if ((pSdoComCon->transferSize == 0) && (pSdoComCon->sdoServiceType == kSdoServiceReadByIndex))
            {
                pSdoComCon->transactionId++;
                pSdoComCon->sdoComState = kSdoComStateClientConnected;
                pSdoComCon->lastAbortCode = 0;
                ret = clientTransferFinished(sdoComConHdl_p, pSdoComCon, kSdoComTransferFinished);
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
                    ret = clientTransferFinished(sdoComConHdl_p, pSdoComCon, kSdoComTransferRxAborted);
                    return ret;
                }
                else
                {   // normal frame received
                    ret = clientProcessFrame(sdoComConHdl_p, pRecvdCmdLayer_p);
                    // check if transfer ready
                    if (pSdoComCon->transferSize == 0)
                    {
                        // send acknowledge without any Command layer data
                        sdoseq_sendData(pSdoComCon->sdoSeqConHdl, 0, NULL);
                        pSdoComCon->transactionId++;
                        pSdoComCon->sdoComState = kSdoComStateClientConnected;
                        pSdoComCon->lastAbortCode = 0;
                        ret = clientTransferFinished(sdoComConHdl_p, pSdoComCon, kSdoComTransferFinished);
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
            ret = clientTransferFinished(sdoComConHdl_p, pSdoComCon, kSdoComTransferFinished);
            break;

        // abort to send from higher layer
        case kSdoComConEventAbort:
            clientSendAbort(pSdoComCon, *((const UINT32*)pSdoComCon->pData));
            pSdoComCon->transactionId++;
            pSdoComCon->sdoComState = kSdoComStateClientConnected;
            pSdoComCon->lastAbortCode = *((const UINT32*)pSdoComCon->pData);
            ret = clientTransferFinished(sdoComConHdl_p, pSdoComCon, kSdoComTransferTxAborted);
            break;

        case kSdoComConEventInitError:
        case kSdoComConEventTimeout:
            sdoseq_deleteCon(pSdoComCon->sdoSeqConHdl);         // close sequence layer handle
            pSdoComCon->sdoSeqConHdl |= SDO_SEQ_INVALID_HDL;
            pSdoComCon->sdoComState = kSdoComStateClientWaitInit;
            pSdoComCon->lastAbortCode = SDO_AC_TIME_OUT;
            ret = clientTransferFinished(sdoComConHdl_p, pSdoComCon, kSdoComTransferLowerLayerAbort);
            break;

        case kSdoComConEventTransferAbort:
            pSdoComCon->sdoComState = kSdoComStateClientWaitInit;
            pSdoComCon->lastAbortCode = SDO_AC_TIME_OUT;
            ret = clientTransferFinished(sdoComConHdl_p, pSdoComCon, kSdoComTransferLowerLayerAbort);
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
\brief  Send an SDO command layer frame from a SDO client

The function starts an SDO transfer and sends all further frames..

\param[in,out]  pSdoComCon_p        Pointer to SDO command layer connection structure.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError clientSend(tSdoComCon* pSdoComCon_p)
{
    tOplkError      ret = kErrorOk;
    UINT8           aFrame[SDO_MAX_TX_FRAME_SIZE];
    tPlkFrame*      pFrame;
    tAsySdoCom*     pCommandFrame;
    UINT            sizeOfCmdFrame;
    UINT            sizeOfCmdData;
    UINT8*          pPayload;
    UINT            payloadSize;

    pFrame = (tPlkFrame*)&aFrame[0];
    initCmdFrameGeneric(pFrame, sizeof(aFrame), pSdoComCon_p, &pCommandFrame);

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
                    ami_setUint16Le(pPayload, (WORD)pSdoComCon_p->targetIndex);
                    pPayload += 2;
                    ami_setUint8Le(pPayload, (UINT8)pSdoComCon_p->targetSubIndex);
                    sizeOfCmdFrame = SDO_CMDL_HDR_FIXED_SIZE + SDO_CMDL_HDR_READBYINDEX_SIZE;
                    // size is reinitialized to 0 for the response frame later, but we need to
                    // set a value != 0 here to indicate a started transfer
                    pSdoComCon_p->transferredBytes = SDO_CMDL_HDR_READBYINDEX_SIZE;
                    break;

                case kSdoServiceWriteByIndex:
                    if (pSdoComCon_p->transferSize > (SDO_CMD_SEGM_TX_MAX_SIZE - SDO_CMDL_HDR_WRITEBYINDEX_SIZE))
                    {   // segmented transfer -> variable part of header needed
                        pSdoComCon_p->sdoTransferType = kSdoTransSegmented;
                        ami_setUint16Le(&pCommandFrame->segmentSizeLe, SDO_CMD_SEGM_TX_MAX_SIZE);
                        overwriteCmdFrameHdrFlags(pCommandFrame, SDO_CMDL_FLAG_SEGMINIT);
                        ami_setUint32Le(&pCommandFrame->aCommandData[0], pSdoComCon_p->transferSize + SDO_CMDL_HDR_FIXED_SIZE);
                        pPayload = &pCommandFrame->aCommandData[SDO_CMDL_HDR_VAR_SIZE];
                        ami_setUint16Le(pPayload, (WORD)pSdoComCon_p->targetIndex);
                        pPayload += 2;
                        ami_setUint8Le(pPayload, (UINT8)pSdoComCon_p->targetSubIndex);
                        pPayload += 2;      // on byte for reserved
                        sizeOfCmdFrame = SDO_CMDL_HDR_FIXED_SIZE + SDO_CMD_SEGM_TX_MAX_SIZE;

                        payloadSize = SDO_CMD_SEGM_TX_MAX_SIZE - (SDO_CMDL_HDR_VAR_SIZE + SDO_CMDL_HDR_WRITEBYINDEX_SIZE);
                        OPLK_MEMCPY(pPayload, pSdoComCon_p->pData, payloadSize);
                        updateHdlTransfSize(pSdoComCon_p, payloadSize, FALSE);
                    }
                    else
                    {   // expedited transfer
                        pSdoComCon_p->sdoTransferType = kSdoTransExpedited;
                        ami_setUint16Le(&pCommandFrame->segmentSizeLe, (WORD)(pSdoComCon_p->transferSize + SDO_CMDL_HDR_WRITEBYINDEX_SIZE));
                        pPayload = &pCommandFrame->aCommandData[0];
                        ami_setUint16Le(pPayload, (WORD)pSdoComCon_p->targetIndex);
                        pPayload += 2;
                        ami_setUint8Le(pPayload, (UINT8)pSdoComCon_p->targetSubIndex);
                        pPayload += 2;      // + 2 -> one byte for sub index and one byte reserved
                        sizeOfCmdFrame = SDO_CMDL_HDR_FIXED_SIZE + (pSdoComCon_p->transferSize + SDO_CMDL_HDR_WRITEBYINDEX_SIZE);

                        OPLK_MEMCPY(pPayload, pSdoComCon_p->pData,  pSdoComCon_p->transferSize);
                        updateHdlTransfSize(pSdoComCon_p, pSdoComCon_p->transferSize, TRUE);
                    }
                    break;

                case kSdoServiceNIL:
                default:
                    // invalid service requested
                    return kErrorSdoComInvalidServiceType;
            }
        }
        else
        {   // continue SDO transfer
            switch (pSdoComCon_p->sdoServiceType)
            {
                case kSdoServiceWriteByIndex:
                    // send next frame
                    if (pSdoComCon_p->sdoTransferType == kSdoTransSegmented)
                    {
                        if (pSdoComCon_p->transferSize > SDO_CMD_SEGM_TX_MAX_SIZE)
                        {   // next segment
                            sizeOfCmdData = SDO_CMD_SEGM_TX_MAX_SIZE;
                            fillCmdFrameDataSegm(pCommandFrame, pSdoComCon_p->pData, sizeOfCmdData);
                            overwriteCmdFrameHdrFlags(pCommandFrame, SDO_CMDL_FLAG_SEGMENTED);
                            setCmdFrameHdrSegmSize(pCommandFrame, sizeOfCmdData);

                            updateHdlTransfSize(pSdoComCon_p, sizeOfCmdData, FALSE);
                            sizeOfCmdFrame = SDO_CMDL_HDR_FIXED_SIZE + sizeOfCmdData;
                        }
                        else
                        {   // end of transfer
                            sizeOfCmdData = pSdoComCon_p->transferSize;
                            fillCmdFrameDataSegm(pCommandFrame, pSdoComCon_p->pData, sizeOfCmdData);
                            overwriteCmdFrameHdrFlags(pCommandFrame, SDO_CMDL_FLAG_SEGMCOMPL);
                            setCmdFrameHdrSegmSize(pCommandFrame, sizeOfCmdData);

                            updateHdlTransfSize(pSdoComCon_p, sizeOfCmdData, TRUE);
                            sizeOfCmdFrame = SDO_CMDL_HDR_FIXED_SIZE + sizeOfCmdData;
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
\brief  Process an SDO command layer frame on a SDO client

The function processes a received command layer frame on an SDO client.

\param[in]      sdoComConHdl_p      Handle of sequence layer connection.
\param[in]      pSdoCom_p           Pointer to received frame.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError clientProcessFrame(tSdoComConHdl sdoComConHdl_p,
                                     const tAsySdoCom* pSdoCom_p)
{
    tOplkError  ret = kErrorOk;
    UINT8       flags;
    UINT8       transactionId;
    UINT8       command;
    UINT        segmentSize;
    UINT        dataSize;
    ULONG       transferSize;
    tSdoComCon* pSdoComCon;

    // get pointer to control structure
    pSdoComCon = &sdoComInstance_l.sdoComCon[sdoComConHdl_p];

    transactionId = ami_getUint8Le(&pSdoCom_p->transactionId);
    if (pSdoComCon->transactionId != transactionId)
    {
        // if running transfer
        if ((pSdoComCon->transferredBytes != 0) && (pSdoComCon->transferSize !=0))
        {
            pSdoComCon->lastAbortCode = SDO_AC_GENERAL_ERROR;
            clientSendAbort(pSdoComCon, pSdoComCon->lastAbortCode);
            ret = clientTransferFinished(sdoComConHdl_p, pSdoComCon, kSdoComTransferTxAborted);
        }
    }
    else
    {   // check if correct command
        command = ami_getUint8Le(&pSdoCom_p->commandId);
        if (pSdoComCon->sdoServiceType != command)
        {
            // incorrect command
            // if running transfer
            if ((pSdoComCon->transferredBytes != 0) && (pSdoComCon->transferSize !=0))
            {
                pSdoComCon->lastAbortCode = SDO_AC_GENERAL_ERROR;
                clientSendAbort(pSdoComCon, pSdoComCon->lastAbortCode);
                ret = clientTransferFinished(sdoComConHdl_p, pSdoComCon, kSdoComTransferTxAborted);
            }
        }
        else
        {   // switch on command
            switch (pSdoComCon->sdoServiceType)
            {
                case kSdoServiceWriteByIndex:
                    // check if confirmation from server
                    // nothing more to do
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
                            {   // buffer provided by the application is too small -> copy only a part
                                dataSize = pSdoComCon->transferSize;
                            }
                            else
                            {   // buffer fits
                                dataSize = segmentSize;
                            }

                            OPLK_MEMCPY(pSdoComCon->pData, &pSdoCom_p->aCommandData[0], dataSize);
                            updateHdlTransfSize(pSdoComCon, dataSize, TRUE);
                            break;

                        case SDO_CMDL_FLAG_SEGMINIT:
                            // get total size of transfer including the header
                            transferSize = ami_getUint32Le(&pSdoCom_p->aCommandData[0]);
                            transferSize -= SDO_CMDL_HDR_VAR_SIZE;
                            if (transferSize <= pSdoComCon->transferSize)
                            {   // buffer fits
                                // re-init transfer sizes utilized by preceding request
                                pSdoComCon->transferSize = (UINT)transferSize;
                                pSdoComCon->transferredBytes = 0;
                            }
                            else
                            {   // buffer too small -> send abort
                                pSdoComCon->lastAbortCode = SDO_AC_DATA_TYPE_LENGTH_TOO_HIGH;
                                clientSendAbort(pSdoComCon, pSdoComCon->lastAbortCode);
                                ret = clientTransferFinished(sdoComConHdl_p, pSdoComCon, kSdoComTransferTxAborted);
                                return ret;
                            }

                            // get segment size
                            // check size of buffer
                            segmentSize = ami_getUint16Le(&pSdoCom_p->segmentSizeLe);
                            segmentSize -= SDO_CMDL_HDR_VAR_SIZE;
                            OPLK_MEMCPY(pSdoComCon->pData, &pSdoCom_p->aCommandData[SDO_CMDL_HDR_VAR_SIZE], segmentSize);
                            updateHdlTransfSize(pSdoComCon, segmentSize, FALSE);
                            break;

                        case SDO_CMDL_FLAG_SEGMENTED:
                            // get segment size
                            // check size of buffer
                            segmentSize = ami_getUint16Le(&pSdoCom_p->segmentSizeLe);
                            // check if data to copy fit to buffer
                            if (segmentSize > pSdoComCon->transferSize)
                            {   // segment too large -> send abort
                                pSdoComCon->lastAbortCode = SDO_AC_INVALID_BLOCK_SIZE;
                                clientSendAbort(pSdoComCon, pSdoComCon->lastAbortCode);
                                ret = clientTransferFinished(sdoComConHdl_p, pSdoComCon, kSdoComTransferTxAborted);
                                return ret;
                            }
                            OPLK_MEMCPY(pSdoComCon->pData, &pSdoCom_p->aCommandData[0], segmentSize);
                            updateHdlTransfSize(pSdoComCon, segmentSize, FALSE);
                            break;

                        case SDO_CMDL_FLAG_SEGMCOMPL:
                            // get segment size
                            // check size of buffer
                            segmentSize = ami_getUint16Le(&pSdoCom_p->segmentSizeLe);
                            // check if data to copy fit to buffer
                            if (segmentSize > pSdoComCon->transferSize)
                            {   // segment too large -> send abort
                                pSdoComCon->lastAbortCode = SDO_AC_INVALID_BLOCK_SIZE;
                                clientSendAbort(pSdoComCon, pSdoComCon->lastAbortCode);
                                ret = clientTransferFinished(sdoComConHdl_p, pSdoComCon, kSdoComTransferTxAborted);
                                return ret;
                            }
                            OPLK_MEMCPY(pSdoComCon->pData, &pSdoCom_p->aCommandData[0], segmentSize);
                            updateHdlTransfSize(pSdoComCon, segmentSize, TRUE);
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
\brief  Send an abort message

The function sends an abort message on an SDO client.

\param[in,out]  pSdoComCon_p        Pointer to SDO command layer connection structure.
\param[in]      abortCode_p         Abort code to send.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError clientSendAbort(tSdoComCon* pSdoComCon_p, UINT32 abortCode_p)
{
    tOplkError      ret = kErrorOk;
    UINT8           aFrame[SDO_MAX_TX_FRAME_SIZE];
    tPlkFrame*      pFrame;
    tAsySdoCom*     pCommandFrame;
    UINT            sizeOfCmdFrame;
    UINT            sizeOfCmdData;

    pFrame = (tPlkFrame*)&aFrame[0];
    initCmdFrameGeneric(pFrame, sizeof(aFrame), pSdoComCon_p, &pCommandFrame);

    sizeOfCmdData = sizeof(UINT32);
    // copy abort code to frame
    ami_setUint32Le(&pCommandFrame->aCommandData[0], abortCode_p);
    setCmdFrameHdrFlag(pCommandFrame, SDO_CMDL_FLAG_ABORT);
    setCmdFrameHdrSegmSize(pCommandFrame, sizeOfCmdData);

    updateHdlTransfSize(pSdoComCon_p, sizeOfCmdData, TRUE);
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
                                    (ULONG)abortCode_p);
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
static tOplkError clientTransferFinished(tSdoComConHdl sdoComConHdl_p,
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
        sdoComFinished.transferredBytes = pSdoComCon_p->transferredBytes;
        sdoComFinished.abortCode = pSdoComCon_p->lastAbortCode;
        sdoComFinished.sdoComConHdl = sdoComConHdl_p;
        sdoComFinished.sdoComConState = sdoComConState_p;
        if (pSdoComCon_p->sdoServiceType == kSdoServiceWriteByIndex)
            sdoComFinished.sdoAccessType = kSdoAccessTypeWrite;
        else
            sdoComFinished.sdoAccessType = kSdoAccessTypeRead;

        // reset transfer state so this handle is not busy anymore
        pSdoComCon_p->transferredBytes = 0;
        pSdoComCon_p->transferSize = 0;

        pfnTransferFinished = pSdoComCon_p->pfnTransferFinished;
        // delete function pointer to inform application only once for each transfer
        pSdoComCon_p->pfnTransferFinished = NULL;

        // call application's callback function
        pfnTransferFinished(&sdoComFinished);
    }

    return ret;
}

#endif // defined(CONFIG_INCLUDE_SDOC)

/// \}
