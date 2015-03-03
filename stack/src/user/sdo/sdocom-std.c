/**
********************************************************************************
\file   sdocom-std.c

\brief  Implementation of SDO Command Layer

This file contains the standard command layer implementation.

\ingroup module_sdocom_std
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2014, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
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
#include <user/sdoseq.h>
#include <oplk/obd.h>
#include <common/ami.h>


#if !defined(CONFIG_INCLUDE_SDOS) && !defined(CONFIG_INCLUDE_SDOC)
#error 'ERROR: At least SDO Server or SDO Client should be activate!'
#endif

//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#ifndef SDO_MAX_SEGMENT_SIZE
#define SDO_MAX_SEGMENT_SIZE        256
#endif

#ifndef CONFIG_SDO_MAX_CONNECTION_COM
#define CONFIG_SDO_MAX_CONNECTION_COM         5
#endif

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
\brief Enumeration for SDO transfer types

This enumeration lists all valid SDO transfer types.
*/
typedef enum
{
    kSdoTransAuto                       = 0x00,     ///< Automatically select the transfer type
    kSdoTransExpedited                  = 0x01,     ///< SDO expedited transfer
    kSdoTransSegmented                  = 0x02      ///< SDO segmented transfer
} tSdoTransType;

/**
\brief Enumeration for SDO service types (command IDs)

This enumeration lists all valid SDO command IDs.
*/
typedef enum
{
    kSdoServiceNIL                      = 0x00,     ///< SDO NIL (do nothing)
    kSdoServiceWriteByIndex             = 0x01,     ///< SDO Write of a single object/sub-object
    kSdoServiceReadByIndex              = 0x02,     ///< SDO Read of a single object/sub-object

    // the following services are optional and are not supported now
    kSdoServiceWriteAllByIndex          = 0x03,     ///< SDO Write of all sub-objects of an object
    kSdoServiceReadAllByIndex           = 0x04,     ///< SDO Read of all sub-objects of an object
    kSdoServiceWriteByName              = 0x05,     ///< SDO Write of a single object/sub-object given by its name
    kSdoServiceReadByName               = 0x06,     ///< SDO Read of a single object/sub-object given by its name
    kSdoServiceFileWrite                = 0x20,     ///< SDO File Write service
    kSdoServiceFileRead                 = 0x21,     ///< SDO File Read service
    kSdoServiceWriteMultiByIndex        = 0x31,     ///< SDO Write of multiple objects/sub-objects
    kSdoServiceReadMultiByIndex         = 0x32,     ///< SDO Read of multiple objects/sub-objects
    kSdoServiceMaxSegSize               = 0x70      ///< SDO maximum segment size
    // 0x80 - 0xFF manufacturer specific
} tSdoServiceType;

/**
\brief  SDO command layer events

This enumeration defines all valid events which will be processed by the
SDO command layer.
*/
typedef enum
{
    kSdoComConEventSendFirst        = 0x00, ///< First frame to send
    kSdoComConEventRec              = 0x01, ///< Frame received
    kSdoComConEventConEstablished   = 0x02, ///< Connection established
    kSdoComConEventConClosed        = 0x03, ///< Connection closed
    kSdoComConEventAckReceived      = 0x04, ///< Acknowledge received by lower layer -> continue sending
    kSdoComConEventFrameSent        = 0x05, ///< Lower has sent a frame
    kSdoComConEventInitError        = 0x06, ///< Error during initialization of the connection
    kSdoComConEventTimeout          = 0x07, ///< Timeout in lower layer
    kSdoComConEventTransferAbort    = 0x08, ///< Transfer abort by lower layer
#if defined(CONFIG_INCLUDE_SDOC)
    kSdoComConEventInitCon          = 0x09, ///< Init connection (only client)
    kSdoComConEventAbort            = 0x0A, ///< Abort SDO transfer (only client)
#endif
} tSdoComConEvent;

/**
\brief  SDO command layer message types

This enumeration defines all valid message types handled by the SDO command layer.
*/
typedef enum
{
    kSdoComSendTypeReq              = 0x00,  ///< Send a request
    kSdoComSendTypeAckRes           = 0x01,  ///< Send a response without data
    kSdoComSendTypeRes              = 0x02,  ///< Send response with data
    kSdoComSendTypeAbort            = 0x03   ///< Send abort
} tSdoComSendType;

/**
\brief  SDO command layer states

This enumeration defines all valid states of the SDO command layer state
machine.
*/
typedef enum
{
    kSdoComStateIdle                = 0x00, ///< Idle state
#if defined(CONFIG_INCLUDE_SDOS)
    kSdoComStateServerSegmTrans     = 0x01, ///< Client: Send following frames
#endif
#if defined(CONFIG_INCLUDE_SDOC)
    kSdoComStateClientWaitInit      = 0x10, ///< Server: Wait for init connection on lower layer
    kSdoComStateClientConnected     = 0x11, ///< Server: Connection established
    kSdoComStateClientSegmTrans     = 0x12  ///< Server: Send following frames
#endif
} tSdoComState;

/**
\brief  SDO command layer connection control structure

This structure defines the connection control structure of the command layer.
*/
typedef struct
{
    tSdoSeqConHdl       sdoSeqConHdl;       ///< SDO sequence layer connection handle (only valid if not 0)
    tSdoComState        sdoComState;        ///< SDO command layer state
    UINT8               transactionId;      ///< Transaction ID
    UINT                nodeId;             ///< NodeId of the target -> needed to reinit connection after timeout
    tSdoTransType       sdoTransferType;    ///< Transfer Type: Auto, Expedited, Segmented
    tSdoServiceType     sdoServiceType;     ///< Service Type: WriteByIndex, ReadByIndex
    tSdoType            sdoProtocolType;    ///< Protocol Type: Auto, Udp, ASnd
    UINT8*              pData;              ///< Pointer to data
    UINT                transferSize;       ///< Number of bytes to transfer
    UINT                transferredBytes;   ///< Number of bytes already transferred
    tSdoFinishedCb      pfnTransferFinished;///< Callback function to be called in the end of the SDO transfer
    void*               pUserArg;           ///< User definable argument pointer
    UINT32              lastAbortCode;      ///< Last abort code
#if defined(CONFIG_INCLUDE_SDOC)
    UINT                targetIndex;        ///< Object Index to access
    UINT                targetSubIndex;     ///< Object subindex to access
#endif
} tSdoComCon;

/**
\brief  SDO command layer instance structure

This structure describes a SDO command layer instance
*/
typedef struct
{
    tSdoComCon          sdoComCon[CONFIG_SDO_MAX_CONNECTION_COM]; ///< Array to store command layer connections
#if defined(WIN32) || defined(_WIN32)
    LPCRITICAL_SECTION  pCriticalSection;
    CRITICAL_SECTION    criticalSection;
#endif
} tSdoComInstance;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static tOplkError sdoInit(void);
static tOplkError sdoAddInstance(void);
static tOplkError sdoDelInstance(void);
static tOplkError sdoDefineConnection(tSdoComConHdl* pSdoComConHdl_p, UINT targetNodeId_p,
                                          tSdoType protType_p);
static tOplkError sdoInitTransferByIndex(tSdoComTransParamByIndex* pSdoComTransParam_p);
static tOplkError sdoUndefineConnection(tSdoComConHdl sdoComConHdl_p);
static tOplkError sdoGetState(tSdoComConHdl sdoComConHdl_p, tSdoComFinished* pSdoComFinished_p);
static UINT       sdoGetNodeId(tSdoComConHdl sdoComConHdl_p);
static tOplkError sdoAbortTransfer(tSdoComConHdl sdoComConHdl_p, UINT32 abortCode_p);
static tOplkError receiveCb(tSdoSeqConHdl sdoSeqConHdl_p, tAsySdoCom* pSdoCom_p, UINT dataSize_p);
static tOplkError conStateChangeCb(tSdoSeqConHdl sdoSeqConHdl_p, tAsySdoConState sdoConnectionState_p);
static tOplkError searchConnection(tSdoSeqConHdl sdoSeqConHdl_p, tSdoComConEvent sdoComConEvent_p, tAsySdoCom* pSdoCom_p);
static tOplkError processState(tSdoComConHdl sdoComConHdl_p, tSdoComConEvent sdoComConEvent_p,
                               tAsySdoCom* pSdoCom_p);
static tOplkError processStateIdle(tSdoComConHdl sdoComConHdl_p, tSdoComConEvent sdoComConEvent_p,
                            tAsySdoCom* pRecvdCmdLayer_p);
static tOplkError processStateServerSegmTrans(tSdoComConHdl sdoComConHdl_p, tSdoComConEvent sdoComConEvent_p,
                                              tAsySdoCom* pRecvdCmdLayer_p);
static tOplkError processStateClientWaitInit(tSdoComConHdl sdoComConHdl_p, tSdoComConEvent sdoComConEvent_p,
                                             tAsySdoCom* pRecvdCmdLayer_p);
static tOplkError processStateClientConnected(tSdoComConHdl sdoComConHdl_p, tSdoComConEvent sdoComConEvent_p,
                                              tAsySdoCom* pRecvdCmdLayer_p);
static tOplkError processStateClientSegmTransfer(tSdoComConHdl sdoComConHdl_p, tSdoComConEvent sdoComConEvent_p,
                                                 tAsySdoCom* pRecvdCmdLayer_p);
static tOplkError transferFinished(tSdoComConHdl sdoComConHdl_p, tSdoComCon* pSdoComCon_p,
                                   tSdoComConState sdoComConState_p);

#if defined (CONFIG_INCLUDE_SDOS)
static tOplkError serverInitReadByIndex(tSdoComCon* pSdoComCon_p, tAsySdoCom* pSdoCom_p);
static tOplkError serverSendFrame(tSdoComCon* pSdoComCon_p, UINT index_p,
                                  UINT subIndex_p, tSdoComSendType sendType_p);
static tOplkError serverInitWriteByIndex(tSdoComCon* pSdoComCon_p, tAsySdoCom* pSdoCom_p);
#endif

#if defined(CONFIG_INCLUDE_SDOC)
static tOplkError clientSend(tSdoComCon* pSdoComCon_p);
static tOplkError clientProcessFrame(tSdoComConHdl sdoComConHdl_p, tAsySdoCom* pSdoCom_p);
static tOplkError clientSendAbort(tSdoComCon* pSdoComCon_p, UINT32 abortCode_p);
#endif

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tSdoComInstance sdoComInstance_l;

/**
\brief Structure for the SDO command layer standard function implementation

This structure provides the SDO command layer function interface for
the standard implementation.
*/
static tSdoComFunctions standardSdoFunctions =
{
   sdoInit,
   sdoAddInstance,
   sdoDelInstance,
#if defined(CONFIG_INCLUDE_SDOC)
   sdoDefineConnection,
   sdoInitTransferByIndex,
   sdoUndefineConnection,
   sdoGetState,
   sdoGetNodeId,
   sdoAbortTransfer,
#endif
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

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief  Initialize SDO command layer module

The function initializes the command layer module.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError sdoInit(void)
{
    return sdoAddInstance();
}

//------------------------------------------------------------------------------
/**
\brief  Add an instance of the SDO command layer module

The function adds an instance of the command layer module.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError sdoAddInstance(void)
{
    tOplkError ret = kErrorOk;

    OPLK_MEMSET(&sdoComInstance_l, 0x00, sizeof(sdoComInstance_l));

    ret = sdoseq_addInstance(receiveCb, conStateChangeCb);
    if (ret != kErrorOk)
        return ret;

#if defined(WIN32) || defined(_WIN32)
    sdoComInstance_l.pCriticalSection = &sdoComInstance_l.criticalSection;
    InitializeCriticalSection(sdoComInstance_l.pCriticalSection);
#endif
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Delete an instance of the SDO command layer module

The function deletes an instance of the command layer module.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError sdoDelInstance(void)
{
    tOplkError  ret = kErrorOk;

#if defined(WIN32) || defined(_WIN32)
    DeleteCriticalSection(sdoComInstance_l.pCriticalSection);
#endif
    ret = sdoseq_delInstance();
    return ret;
}

#if defined(CONFIG_INCLUDE_SDOC)
//------------------------------------------------------------------------------
/**
\brief  Define a command layer connection

The function defines an SDO command layer connection to another node. It
initializes the lower layer and returns a handle for the connection. Multiple
client connections to the same node via the same protocol are not allowed.
If this function detects such a situation it will return
kErrorSdoComHandleExists and the handle of the existing connection in
pSdoComConHdl_p.

\param  pSdoComConHdl_p         Pointer to store the connection handle.
\param  targetNodeId_p          The node ID to connect to.
\param  protType_p              The protocol type to use for the connection
                                (UDP and ASnd is supported)

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError sdoDefineConnection(tSdoComConHdl* pSdoComConHdl_p, UINT targetNodeId_p,
                                       tSdoType protType_p)
{
    tOplkError  ret;
    UINT        count;
    UINT        freeHdl;
    tSdoComCon* pSdoComCon;

    if ((targetNodeId_p == C_ADR_INVALID) || (targetNodeId_p >= C_ADR_BROADCAST))
        ret = kErrorInvalidNodeId;

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
    {
        return kErrorSdoComNoFreeHandle;
    }

    pSdoComCon = &sdoComInstance_l.sdoComCon[freeHdl];

    *pSdoComConHdl_p = freeHdl;                 // save handle for application

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
            break;
    }

    ret = processState(freeHdl, kSdoComConEventInitCon, NULL);
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Initialize a transfer by index command

The function initializes a "transfer by index" operation for a connection.

\param  pSdoComTransParam_p     Pointer to transfer command parameters

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError sdoInitTransferByIndex(tSdoComTransParamByIndex* pSdoComTransParam_p)
{
    tOplkError      ret;
    tSdoComCon*     pSdoComCon;

    if ((pSdoComTransParam_p->subindex >= 0xFF) || (pSdoComTransParam_p->index == 0) ||
        (pSdoComTransParam_p->index > 0xFFFF) || (pSdoComTransParam_p->pData == NULL) ||
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
    {
        pSdoComCon->sdoServiceType = kSdoServiceReadByIndex;
    }
    else
    {
        pSdoComCon->sdoServiceType = kSdoServiceWriteByIndex;
    }

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

\param  sdoComConHdl_p          Handle of the connection to delete.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError sdoUndefineConnection(tSdoComConHdl sdoComConHdl_p)
{
    tOplkError          ret = kErrorOk;
    tSdoComCon*         pSdoComCon;

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
                break;
        }
    }

    OPLK_MEMSET(pSdoComCon, 0x00, sizeof(tSdoComCon));
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Get command layer connection state

The function returns the state of a command layer connection.

\param  sdoComConHdl_p          Handle of the command layer connection.
\param  pSdoComFinished_p       Pointer to store connection information.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError sdoGetState(tSdoComConHdl sdoComConHdl_p, tSdoComFinished* pSdoComFinished_p)
{
    tOplkError          ret = kErrorOk;
    tSdoComCon*         pSdoComCon;

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
    {
        pSdoComFinished_p->sdoAccessType = kSdoAccessTypeWrite;
    }
    else
    {
        pSdoComFinished_p->sdoAccessType = kSdoAccessTypeRead;
    }

    if (pSdoComCon->lastAbortCode != 0)
    {   // sdo abort
        pSdoComFinished_p->sdoComConState = kSdoComTransferRxAborted;
        // delete abort code
        pSdoComCon->lastAbortCode = 0;
    }
    else if ((pSdoComCon->sdoSeqConHdl & ~SDO_SEQ_HANDLE_MASK)== SDO_SEQ_INVALID_HDL)
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

\param  sdoComConHdl_p          Handle of connection.

\return The function returns the node ID of the remote node or C_ADR_INVALID
        on error.
*/
//------------------------------------------------------------------------------
static UINT sdoGetNodeId(tSdoComConHdl sdoComConHdl_p)
{
    UINT            nodeId = C_ADR_INVALID;
    tSdoComCon*     pSdoComCon;

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

\param  sdoComConHdl_p          Handle of the connection to abort.
\param  abortCode_p             The abort code to use.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError sdoAbortTransfer(tSdoComConHdl sdoComConHdl_p, UINT32 abortCode_p)
{
    tOplkError      ret;
    tSdoComCon*     pSdoComCon;

    if (sdoComConHdl_p >= CONFIG_SDO_MAX_CONNECTION_COM)
        return kErrorSdoComInvalidHandle;

    // get pointer to control structure of connection
    pSdoComCon = &sdoComInstance_l.sdoComCon[sdoComConHdl_p];

    if (pSdoComCon->sdoSeqConHdl == 0)
        return kErrorSdoComInvalidHandle;

    pSdoComCon->pData = (UINT8*)&abortCode_p;
    ret = processState(sdoComConHdl_p, kSdoComConEventAbort, (tAsySdoCom*)NULL);
    return ret;
}

#endif

//------------------------------------------------------------------------------
/**
\brief  Receive callback function

The function implements the receive callback function that is called by the
SOD sequence layer when new data is received.

\param  sdoSeqConHdl_p          Handle of the SDO sequence layer connection.
\param  pSdoCom_p               Pointer to received command layer data.
\param  dataSize_p              Size of the received data.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError receiveCb(tSdoSeqConHdl sdoSeqConHdl_p, tAsySdoCom* pSdoCom_p, UINT dataSize_p)
{
    tOplkError       ret;

    UNUSED_PARAMETER(dataSize_p);

    ret = searchConnection(sdoSeqConHdl_p, kSdoComConEventRec, pSdoCom_p);
    DEBUG_LVL_SDO_TRACE("receiveCb SdoSeqConHdl: 0x%X, First Byte of pSdoCom_p: 0x%02X, dataSize_p: 0x%04X\n",
                         sdoSeqConHdl_p, (WORD)pSdoCom_p->aCommandData[0], dataSize_p);
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Connection state change callback function

The function implements the connection stat change callback function. It is
called by the SDO sequence layer to inform the command layer about state changes
of the connection.

\param  sdoSeqConHdl_p          Handle of the SDO sequence layer connection.
\param  sdoConnectionState_p    SDO connection state.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError conStateChangeCb(tSdoSeqConHdl sdoSeqConHdl_p,
                                   tAsySdoConState sdoConnectionState_p)
{
    tOplkError          ret = kErrorOk;
    tSdoComConEvent     sdoComConEvent = kSdoComConEventSendFirst;

    switch (sdoConnectionState_p)
    {
        case kAsySdoConStateConnected:
            DEBUG_LVL_SDO_TRACE("Connection established\n");
            sdoComConEvent = kSdoComConEventConEstablished;
            // start transmission if needed
            break;

        case kAsySdoConStateInitError:
            DEBUG_LVL_SDO_TRACE("Error during initialisation\n");
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
            DEBUG_LVL_SDO_TRACE("One Frame sent\n");
            sdoComConEvent = kSdoComConEventFrameSent;
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

    ret = searchConnection(sdoSeqConHdl_p, sdoComConEvent, (tAsySdoCom*)NULL);
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Search a connection

The function searches for an SDO sequence layer connection.

\param  sdoSeqConHdl_p          Handle of the SDO sequence layer connection.
\param  sdoComConEvent_p        Event to process for found connection.
\param  pSdoCom_p               Pointer to received command layer data.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError searchConnection(tSdoSeqConHdl sdoSeqConHdl_p, tSdoComConEvent sdoComConEvent_p,
                                   tAsySdoCom* pSdoCom_p)
{
    tOplkError          ret;
    tSdoComCon*         pSdoComCon;
    tSdoComConHdl       hdlCount;
    tSdoComConHdl       hdlFree;

    ret = kErrorSdoComNotResponsible;

    // get pointer to first element of the array
    pSdoComCon = &sdoComInstance_l.sdoComCon[0];
    hdlCount = 0;
    hdlFree = 0xFFFF;
    while (hdlCount < CONFIG_SDO_MAX_CONNECTION_COM)
    {
        if (pSdoComCon->sdoSeqConHdl == sdoSeqConHdl_p)
        {   // matching command layer handle found
            ret = processState(hdlCount, sdoComConEvent_p, pSdoCom_p);
        }
        else if ((pSdoComCon->sdoSeqConHdl == 0) && (hdlFree == 0xFFFF))
        {
            hdlFree = hdlCount;
        }
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
            pSdoComCon = &sdoComInstance_l.sdoComCon[hdlCount];
            pSdoComCon->sdoSeqConHdl = sdoSeqConHdl_p;
            ret = processState(hdlCount, sdoComConEvent_p, pSdoCom_p);
        }
    }
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Process state kSdoComStateIdle

The function processes the SDO command handler state: kSdoComStateIdle

\param  sdoComConHdl_p          Handle to command layer connection.
\param  sdoComConEvent_p        Event to process.
\param  pRecvdCmdLayer_p        SDO command layer part of received frame.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError processStateIdle(tSdoComConHdl sdoComConHdl_p, tSdoComConEvent sdoComConEvent_p,
                                   tAsySdoCom* pRecvdCmdLayer_p)
{
    tOplkError          ret = kErrorOk;
    tSdoComCon*         pSdoComCon;

#if defined(CONFIG_INCLUDE_SDOS)
    UINT32              abortCode;
#endif

    pSdoComCon = &sdoComInstance_l.sdoComCon[sdoComConHdl_p];

    switch (sdoComConEvent_p)
    {
#if defined(CONFIG_INCLUDE_SDOC)
        case kSdoComConEventInitCon: // init con for client
            // call of the init function already processed in sdocom_defineConnection()
            // only change state to kSdoComStateClientWaitInit
            pSdoComCon->sdoComState = kSdoComStateClientWaitInit;
            break;
#endif

        case kSdoComConEventRec: // int con for server
#if defined(CONFIG_INCLUDE_SDOS)
            // check if init of an transfer and no SDO abort
            if ((pRecvdCmdLayer_p->flags & SDO_CMDL_FLAG_RESPONSE) == 0)
            {   // SDO request
                if ((pRecvdCmdLayer_p->flags & SDO_CMDL_FLAG_ABORT) == 0)
                {   // no SDO abort, save transaction id
                    pSdoComCon->transactionId = ami_getUint8Le(&pRecvdCmdLayer_p->transactionId);

                    switch (pRecvdCmdLayer_p->commandId)
                    {
                        case kSdoServiceNIL:
                            // simply acknowledge NIL command on sequence layer
                            ret = sdoseq_sendData(pSdoComCon->sdoSeqConHdl, 0, (tPlkFrame*)NULL);
                            break;

                        case kSdoServiceReadByIndex:
                            // read by index, search entry an start transfer
                            serverInitReadByIndex(pSdoComCon, pRecvdCmdLayer_p);
                            // check next state
                            if (pSdoComCon->transferSize == 0)
                            {   // ready -> stay idle
                                pSdoComCon->sdoComState = kSdoComStateIdle;
                                pSdoComCon->lastAbortCode = 0;
                            }
                            else
                            {   // segmented transfer
                                pSdoComCon->sdoComState = kSdoComStateServerSegmTrans;
                            }
                            break;

                        case kSdoServiceWriteByIndex:
                            // search entry an start write
                            serverInitWriteByIndex(pSdoComCon, pRecvdCmdLayer_p);
                            // check next state
                            if (pSdoComCon->transferSize == 0)
                            {   // already -> stay idle
                                pSdoComCon->sdoComState = kSdoComStateIdle;
                                pSdoComCon->lastAbortCode = 0;
                            }
                            else
                            {   // segmented transfer
                                pSdoComCon->sdoComState = kSdoComStateServerSegmTrans;
                            }
                            break;

                        default:
                            //  unsupported command -> send abort
                            abortCode = SDO_AC_UNKNOWN_COMMAND_SPECIFIER;
                            pSdoComCon->pData = (UINT8*)&abortCode;
                            ret = serverSendFrame(pSdoComCon, 0, 0, kSdoComSendTypeAbort);
                            break;
                    }
                }
            }
            else
            {   // this command layer handle is not responsible
                // (wrong direction or wrong transaction ID)
                return kErrorSdoComNotResponsible;
            }
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

#if defined(CONFIG_INCLUDE_SDOS)

//------------------------------------------------------------------------------
/**
\brief  Process state kSdoComStateServerSegmTrans

The function processes the SDO command handler state: kSdoComStateServerSegmTrans

\param  sdoComConHdl_p          Handle to command layer connection.
\param  sdoComConEvent_p        Event to process.
\param  pRecvdCmdLayer_p        SDO command layer part of received frame.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError processStateServerSegmTrans(tSdoComConHdl sdoComConHdl_p, tSdoComConEvent sdoComConEvent_p,
                                              tAsySdoCom* pRecvdCmdLayer_p)
{
    tOplkError          ret = kErrorOk;
    UINT                size;
    UINT8               flag;
    tSdoComCon*         pSdoComCon;

    pSdoComCon = &sdoComInstance_l.sdoComCon[sdoComConHdl_p];

    switch (sdoComConEvent_p)
    {
        // send next frame
        case kSdoComConEventAckReceived:
        case kSdoComConEventFrameSent:
            // check if it is a read
            if (pSdoComCon->sdoServiceType == kSdoServiceReadByIndex)
            {
                // send next frame
                serverSendFrame(pSdoComCon, 0, 0, kSdoComSendTypeRes);
                // if all send -> back to idle
                if (pSdoComCon->transferSize == 0)
                {   // back to idle
                    pSdoComCon->sdoComState = kSdoComStateIdle;
                    pSdoComCon->lastAbortCode = 0;
                }
            }
            break;

        // process next frame
        case kSdoComConEventRec:
            // check if the frame is an SDO response and has the right transaction ID
            flag = ami_getUint8Le(&pRecvdCmdLayer_p->flags);

            if (((flag & SDO_CMDL_FLAG_RESPONSE) == 0) &&
                (ami_getUint8Le(&pRecvdCmdLayer_p->transactionId) == pSdoComCon->transactionId))
            {
                // check if it is a abort
                if ((flag & SDO_CMDL_FLAG_ABORT) != 0)
                {
                    pSdoComCon->transferSize = 0;
                    pSdoComCon->transferredBytes = 0;
                    pSdoComCon->sdoComState = kSdoComStateIdle;
                    pSdoComCon->lastAbortCode = 0;
                    // d.k.: do not execute anything further on this command
                    break;
                }

                // check if it is a write
                if (pSdoComCon->sdoServiceType == kSdoServiceWriteByIndex)
                {
                    size = ami_getUint16Le(&pRecvdCmdLayer_p->segmentSizeLe);
                    if (size > pSdoComCon->transferSize)
                    {
                        pSdoComCon->lastAbortCode = SDO_AC_DATA_TYPE_LENGTH_TOO_HIGH;
                        ret = serverSendFrame(pSdoComCon, 0, 0, kSdoComSendTypeAbort);
                        return ret;
                    }
                    if (pSdoComCon->lastAbortCode == 0)
                    {
                        OPLK_MEMCPY(pSdoComCon->pData, &pRecvdCmdLayer_p->aCommandData[0], size);
                        (pSdoComCon->pData) += size;
                    }
                    pSdoComCon->transferredBytes += size;
                    pSdoComCon->transferSize -= size;

                    // check end of transfer
                    if ((pRecvdCmdLayer_p->flags & SDO_CMDL_FLAG_SEGM_MASK) == SDO_CMDL_FLAG_SEGMCOMPL)
                    {   // transfer ready
                        pSdoComCon->transferSize = 0;

                        if (pSdoComCon->lastAbortCode == 0)
                        {
                            // send response
                            serverSendFrame(pSdoComCon, 0, 0, kSdoComSendTypeRes);
                            // if all send -> back to idle
                            if (pSdoComCon->transferSize == 0)
                            {   // back to idle
                                pSdoComCon->sdoComState = kSdoComStateIdle;
                                pSdoComCon->lastAbortCode = 0;
                            }
                        }
                        else
                        {   // send abort
                            pSdoComCon->pData = (UINT8*)&pSdoComCon->lastAbortCode;
                            ret = serverSendFrame(pSdoComCon, 0, 0, kSdoComSendTypeAbort);
                            pSdoComCon->lastAbortCode = 0;
                        }
                    }
                    else
                    {
                        // send acknowledge without any Command layer data
                        ret = sdoseq_sendData(pSdoComCon->sdoSeqConHdl, 0, (tPlkFrame*)NULL);
                    }
                }
            }
            else
            {   // this command layer handle is not responsible
                // (wrong direction or wrong transaction ID)
                ret = kErrorSdoComNotResponsible;
            }
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
#endif

#if defined(CONFIG_INCLUDE_SDOC)
//------------------------------------------------------------------------------
/**
\brief  Process state processStateClientWaitInit

The function processes the SDO command handler state: processStateClientWaitInit

\param  sdoComConHdl_p          Handle to command layer connection.
\param  sdoComConEvent_p        Event to process.
\param  pRecvdCmdLayer_p        SDO command layer part of received frame.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError processStateClientWaitInit(tSdoComConHdl sdoComConHdl_p, tSdoComConEvent sdoComConEvent_p,
                                             tAsySdoCom* pRecvdCmdLayer_p)
{
    tOplkError          ret = kErrorOk;
    tSdoComCon*         pSdoComCon;

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
                break;
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
                // check if segemted transfer
                if (pSdoComCon->sdoTransferType == kSdoTransSegmented)
                {
                    pSdoComCon->sdoComState = kSdoComStateClientSegmTrans;
                }
                else
                {
                    pSdoComCon->sdoComState = kSdoComStateClientConnected;
                }

                ret = clientSend(pSdoComCon);
                if (ret != kErrorOk)
                    return ret;
            }
            else
            {
                pSdoComCon->sdoComState = kSdoComStateClientConnected;
            }
            break;

        case kSdoComConEventSendFirst:
            // infos for transfer already saved by function sdocom_initTransferByIndex
            break;

        // abort to send from higher layer
        case kSdoComConEventAbort:
            pSdoComCon->lastAbortCode = *((UINT32*)pSdoComCon->pData);
            ret = transferFinished(sdoComConHdl_p, pSdoComCon, kSdoComTransferTxAborted);
            break;

        case kSdoComConEventConClosed:
        case kSdoComConEventInitError:
        case kSdoComConEventTimeout:
        case kSdoComConEventTransferAbort:
            ret = sdoseq_deleteCon(pSdoComCon->sdoSeqConHdl);         // close sequence layer handle
            pSdoComCon->sdoSeqConHdl |= SDO_SEQ_INVALID_HDL;
            if (sdoComConEvent_p == kSdoComConEventTimeout)
            {
                pSdoComCon->lastAbortCode = SDO_AC_TIME_OUT;
            }
            else
            {
                pSdoComCon->lastAbortCode = 0;
            }
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

\param  sdoComConHdl_p          Handle to command layer connection.
\param  sdoComConEvent_p        Event to process.
\param  pRecvdCmdLayer_p        SDO command layer part of received frame.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError processStateClientConnected(tSdoComConHdl sdoComConHdl_p, tSdoComConEvent sdoComConEvent_p,
                                              tAsySdoCom* pRecvdCmdLayer_p)
{

    tOplkError          ret = kErrorOk;
    UINT8               flag;
    tSdoComCon*         pSdoComCon;

    pSdoComCon = &sdoComInstance_l.sdoComCon[sdoComConHdl_p];

    switch (sdoComConEvent_p)
    {
        // send a frame
        case kSdoComConEventSendFirst:
        case kSdoComConEventAckReceived:
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
                    ret = sdoseq_sendData(pSdoComCon->sdoSeqConHdl, 0, (tPlkFrame*)NULL);
                    pSdoComCon->transactionId++;
                    pSdoComCon->lastAbortCode = ami_getUint32Le(&pRecvdCmdLayer_p->aCommandData[0]);
                    ret = transferFinished(sdoComConHdl_p, pSdoComCon, kSdoComTransferRxAborted);
                    return ret;
                }
                else
                {   // normal frame received
                    ret = clientProcessFrame(sdoComConHdl_p, pRecvdCmdLayer_p);
                    // check if transfer ready
                    if (pSdoComCon->transferSize == 0)
                    {
                        // send acknowledge without any Command layer data
                        ret = sdoseq_sendData(pSdoComCon->sdoSeqConHdl, 0, (tPlkFrame*)NULL);
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
            ret = sdoseq_deleteCon(pSdoComCon->sdoSeqConHdl);         // close sequence layer handle
            pSdoComCon->sdoSeqConHdl |= SDO_SEQ_INVALID_HDL;
            pSdoComCon->sdoComState = kSdoComStateClientWaitInit;
            pSdoComCon->lastAbortCode = 0;
            ret = transferFinished(sdoComConHdl_p, pSdoComCon, kSdoComTransferLowerLayerAbort);
            break;

        case kSdoComConEventAbort:
            clientSendAbort(pSdoComCon, *((UINT32*)pSdoComCon->pData));
            pSdoComCon->transactionId++;
            pSdoComCon->lastAbortCode = *((UINT32*)pSdoComCon->pData);
            ret = transferFinished(sdoComConHdl_p, pSdoComCon, kSdoComTransferTxAborted);
            break;

        case kSdoComConEventInitError:
        case kSdoComConEventTimeout:
            ret = sdoseq_deleteCon(pSdoComCon->sdoSeqConHdl);         // close sequence layer handle
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

\param  sdoComConHdl_p          Handle to command layer connection.
\param  sdoComConEvent_p        Event to process.
\param  pRecvdCmdLayer_p        SDO command layer part of received frame.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError processStateClientSegmTransfer(tSdoComConHdl sdoComConHdl_p, tSdoComConEvent sdoComConEvent_p,
                                                 tAsySdoCom* pRecvdCmdLayer_p)
{
    tOplkError          ret = kErrorOk;
    UINT8               flag;
    tSdoComCon*         pSdoComCon;

    pSdoComCon = &sdoComInstance_l.sdoComCon[sdoComConHdl_p];

    switch (sdoComConEvent_p)
    {
        case kSdoComConEventSendFirst:
        case kSdoComConEventAckReceived:
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
                    ret = sdoseq_sendData(pSdoComCon->sdoSeqConHdl, 0, (tPlkFrame*)NULL);
                    pSdoComCon->transactionId++;
                    pSdoComCon->sdoComState = kSdoComStateClientConnected;
                    pSdoComCon->lastAbortCode = ami_getUint32Le(&pRecvdCmdLayer_p->aCommandData[0]);
                    ret = transferFinished(sdoComConHdl_p, pSdoComCon, kSdoComTransferRxAborted);
                    return ret;
                }
                else
                {   // normal frame received
                    ret = clientProcessFrame(sdoComConHdl_p, pRecvdCmdLayer_p);
                    // check if transfer ready
                    if (pSdoComCon->transferSize == 0)
                    {
                        // send acknowledge without any Command layer data
                        ret = sdoseq_sendData(pSdoComCon->sdoSeqConHdl, 0, (tPlkFrame*)NULL);
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
            ret = sdoseq_deleteCon(pSdoComCon->sdoSeqConHdl);         // close sequence layer handle
            pSdoComCon->sdoSeqConHdl |= SDO_SEQ_INVALID_HDL;
            pSdoComCon->sdoComState = kSdoComStateClientWaitInit;
            pSdoComCon->transactionId++;
            pSdoComCon->lastAbortCode = 0;
            ret = transferFinished(sdoComConHdl_p, pSdoComCon, kSdoComTransferFinished);
            break;

        // abort to send from higher layer
        case kSdoComConEventAbort:
            clientSendAbort(pSdoComCon, *((UINT32*)pSdoComCon->pData));
            pSdoComCon->transactionId++;
            pSdoComCon->sdoComState = kSdoComStateClientConnected;
            pSdoComCon->lastAbortCode = *((UINT32*)pSdoComCon->pData);
            ret = transferFinished(sdoComConHdl_p, pSdoComCon, kSdoComTransferTxAborted);
            break;

        case kSdoComConEventInitError:
        case kSdoComConEventTimeout:
            ret = sdoseq_deleteCon(pSdoComCon->sdoSeqConHdl);         // close sequence layer handle
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

#endif

//------------------------------------------------------------------------------
/**
\brief  Process SDO command layer state machine

The function processes the SDO command handler state machine. Depending
on the state the command layer event is processed.

\param  sdoComConHdl_p          Handle to command layer connection.
\param  sdoComConEvent_p        Event to process.
\param  pRecvdCmdLayer_p        SDO command layer part of received frame.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError processState(tSdoComConHdl sdoComConHdl_p, tSdoComConEvent sdoComConEvent_p,
                               tAsySdoCom* pRecvdCmdLayer_p)
{
    tOplkError          ret = kErrorOk;
    tSdoComCon*         pSdoComCon;

#if defined(WIN32) || defined(_WIN32)
    EnterCriticalSection(sdoComInstance_l.pCriticalSection);
    DEBUG_LVL_SDO_TRACE("\n\tEnterCiticalSection processState\n\n");
#endif

    // get pointer to control structure
    pSdoComCon = &sdoComInstance_l.sdoComCon[sdoComConHdl_p];

    // process state maschine
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
            ret = processStateServerSegmTrans(sdoComConHdl_p, sdoComConEvent_p, pRecvdCmdLayer_p);
            break;
#endif

#if defined(CONFIG_INCLUDE_SDOC)
        //----------------------------------------------------------------------
        // SDO Client part
        // wait for finish of establishing connection
        case kSdoComStateClientWaitInit:
            ret = processStateClientWaitInit(sdoComConHdl_p, sdoComConEvent_p, pRecvdCmdLayer_p);
            break;

        case kSdoComStateClientConnected:
            ret = processStateClientConnected(sdoComConHdl_p, sdoComConEvent_p, pRecvdCmdLayer_p);
            break;

        // process segmented transfer
        case kSdoComStateClientSegmTrans:
            ret = processStateClientSegmTransfer(sdoComConHdl_p, sdoComConEvent_p, pRecvdCmdLayer_p);
            break;
#endif
    }

#if defined(WIN32) || defined(_WIN32)
    DEBUG_LVL_SDO_TRACE("\n\tLeaveCriticalSection processState\n\n");
    LeaveCriticalSection(sdoComInstance_l.pCriticalSection);
#endif
    return ret;
}

#if defined(CONFIG_INCLUDE_SDOS)

//------------------------------------------------------------------------------
/**
\brief  Initialize a ReadByIndex command

The function initializes a ReadByIndex SDO command.

\param  pSdoComCon_p            Pointer to SDO command layer connection structure.
\param  pSdoCom_p               Pointer to received command layer data.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError serverInitReadByIndex(tSdoComCon* pSdoComCon_p, tAsySdoCom* pSdoCom_p)
{
    tOplkError      ret;
    UINT            index;
    UINT            subindex;
    tObdSize        entrySize;
    tObdAccess      accessType;
    UINT32          abortCode;

    abortCode = 0;

    // An init of a read could not be a segmented transfer -> no variable part of header

    index = ami_getUint16Le(&pSdoCom_p->aCommandData[0]);
    subindex = ami_getUint8Le(&pSdoCom_p->aCommandData[2]);

    ret = obd_getAccessType(index, subindex, &accessType);
    if (ret == kErrorObdSubindexNotExist)
    {   // subentry doesn't exist
        abortCode = SDO_AC_SUB_INDEX_NOT_EXIST;
        pSdoComCon_p->pData = (UINT8*)&abortCode;
        ret = serverSendFrame(pSdoComCon_p, index, subindex, kSdoComSendTypeAbort);
        return ret;
    }
    else if (ret != kErrorOk)
    {   // entry doesn't exist
        abortCode = SDO_AC_OBJECT_NOT_EXIST;
        pSdoComCon_p->pData = (UINT8*)&abortCode;
        ret = serverSendFrame(pSdoComCon_p, index, subindex, kSdoComSendTypeAbort);
        return ret;
    }

    // access type must be read or const
    if (((accessType & kObdAccRead) == 0) && ((accessType & kObdAccConst) == 0))
    {
        if ((accessType & kObdAccWrite) != 0)
        {
            abortCode = SDO_AC_READ_TO_WRITE_ONLY_OBJ;
        }
        else
        {
            abortCode = SDO_AC_UNSUPPORTED_ACCESS;
        }
        pSdoComCon_p->pData = (UINT8*)&abortCode;
        ret = serverSendFrame(pSdoComCon_p, index, subindex, kSdoComSendTypeAbort);
        return ret;
    }

    pSdoComCon_p->sdoServiceType = kSdoServiceReadByIndex;

    // get size of object to see if segmented or expedited transfer
    entrySize = obd_getDataSize(index, subindex);
    if (entrySize > SDO_MAX_SEGMENT_SIZE)
    {
        pSdoComCon_p->sdoTransferType = kSdoTransSegmented;
        pSdoComCon_p->pData = (UINT8*)obd_getObjectDataPtr(index, subindex);
    }
    else
    {
        pSdoComCon_p->sdoTransferType = kSdoTransExpedited;
    }

    pSdoComCon_p->transferSize = entrySize;
    pSdoComCon_p->transferredBytes = 0;

    ret = serverSendFrame(pSdoComCon_p, index, subindex, kSdoComSendTypeRes);
    if (ret != kErrorOk)
    {
        abortCode = SDO_AC_GENERAL_ERROR;
        pSdoComCon_p->pData = (UINT8*)&abortCode;
        ret = serverSendFrame(pSdoComCon_p, index, subindex, kSdoComSendTypeAbort);
        return ret;
    }
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Send a SDO command layer frame from a SDO server

The function creates and sends a command layer frame from a SDO server.

\param  pSdoComCon_p            Pointer to SDO command layer connection structure.
\param  index_p                 Index of object to send if it is an expedited
                                transfer. (Ignored otherwise)
\param  subIndex_p              Sub index of object to send if it is an expedited
                                transfer. (Ignored otherwise)
\param  sendType_p              Type of data to send.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError serverSendFrame(tSdoComCon* pSdoComCon_p, UINT index_p,
                                  UINT subIndex_p, tSdoComSendType sendType_p)
{
    tOplkError      ret = kErrorOk;
    UINT8           aFrame[SDO_MAX_FRAME_SIZE];
    tPlkFrame *     pFrame;
    tAsySdoCom*     pCommandFrame;
    UINT            sizeOfFrame;
    UINT8           flag;

    pFrame = (tPlkFrame*)&aFrame[0];
    OPLK_MEMSET(&aFrame[0], 0x00, sizeof(aFrame));

    // build generic part of frame - get pointer to command layer part of frame
    pCommandFrame = &pFrame->data.asnd.payload.sdoSequenceFrame.sdoSeqPayload;
    ami_setUint8Le(&pCommandFrame->commandId, pSdoComCon_p->sdoServiceType);
    ami_setUint8Le(&pCommandFrame->transactionId, pSdoComCon_p->transactionId);

    sizeOfFrame = SDO_CMDL_HDR_FIXED_SIZE;

    switch (sendType_p)
    {

        case kSdoComSendTypeReq:    // request frame to send
            // nothing to do for server -> error
            return kErrorSdoComInvalidSendType;
            break;

        case kSdoComSendTypeAckRes: // response without data to send
            ami_setUint8Le(&pCommandFrame->flags,  SDO_CMDL_FLAG_RESPONSE);
            ret = sdoseq_sendData(pSdoComCon_p->sdoSeqConHdl, sizeOfFrame, pFrame);
            break;

        case kSdoComSendTypeRes:    // response frame to send
            flag = ami_getUint8Le(&pCommandFrame->flags);
            flag |= SDO_CMDL_FLAG_RESPONSE;
            ami_setUint8Le(&pCommandFrame->flags,  flag);

            if (pSdoComCon_p->sdoTransferType == kSdoTransExpedited)
            {   // Expedited transfer
                // copy data in frame
                ret = obd_readEntryToLe(index_p, subIndex_p,
                                        &pCommandFrame->aCommandData[0],
                                        (tObdSize*)&pSdoComCon_p->transferSize);
                if (ret != kErrorOk)
                    return ret;

                ami_setUint16Le(&pCommandFrame->segmentSizeLe, (WORD)pSdoComCon_p->transferSize);
                sizeOfFrame += pSdoComCon_p->transferSize;
                pSdoComCon_p->transferredBytes += pSdoComCon_p->transferSize;
                pSdoComCon_p->transferSize = 0;

                sizeOfFrame += pSdoComCon_p->transferSize;
                ret = sdoseq_sendData(pSdoComCon_p->sdoSeqConHdl, sizeOfFrame, pFrame);
            }
            else if (pSdoComCon_p->sdoTransferType == kSdoTransSegmented)
            {   // segmented transfer
                // distinguish between init, segment and complete
                if (pSdoComCon_p->transferredBytes == 0)
                {   // init
                    flag = ami_getUint8Le(&pCommandFrame->flags);
                    flag |= SDO_CMDL_FLAG_SEGMINIT;
                    ami_setUint8Le(&pCommandFrame->flags,  flag);
                    // init data size in variable header, which includes itself
                    ami_setUint32Le(&pCommandFrame->aCommandData[0], pSdoComCon_p->transferSize + SDO_CMDL_HDR_VAR_SIZE);
                    OPLK_MEMCPY(&pCommandFrame->aCommandData[SDO_CMDL_HDR_VAR_SIZE], pSdoComCon_p->pData, (SDO_MAX_SEGMENT_SIZE - SDO_CMDL_HDR_VAR_SIZE));

                    pSdoComCon_p->transferSize -= (SDO_MAX_SEGMENT_SIZE - SDO_CMDL_HDR_VAR_SIZE);
                    pSdoComCon_p->transferredBytes += (SDO_MAX_SEGMENT_SIZE - SDO_CMDL_HDR_VAR_SIZE);
                    pSdoComCon_p->pData +=(SDO_MAX_SEGMENT_SIZE - SDO_CMDL_HDR_VAR_SIZE);

                    ami_setUint16Le(&pCommandFrame->segmentSizeLe, SDO_MAX_SEGMENT_SIZE);

                    sizeOfFrame += SDO_MAX_SEGMENT_SIZE;
                    ret = sdoseq_sendData(pSdoComCon_p->sdoSeqConHdl, sizeOfFrame, pFrame);
                }
                else if ((pSdoComCon_p->transferredBytes > 0) &&(pSdoComCon_p->transferSize > SDO_MAX_SEGMENT_SIZE))
                {   // segment
                    flag = ami_getUint8Le(&pCommandFrame->flags);
                    flag |= SDO_CMDL_FLAG_SEGMENTED;
                    ami_setUint8Le(&pCommandFrame->flags, flag);

                    OPLK_MEMCPY(&pCommandFrame->aCommandData[0], pSdoComCon_p->pData, SDO_MAX_SEGMENT_SIZE);
                    pSdoComCon_p->transferSize -= SDO_MAX_SEGMENT_SIZE;
                    pSdoComCon_p->transferredBytes += SDO_MAX_SEGMENT_SIZE;
                    pSdoComCon_p->pData +=SDO_MAX_SEGMENT_SIZE;
                    ami_setUint16Le(&pCommandFrame->segmentSizeLe, SDO_MAX_SEGMENT_SIZE);

                    sizeOfFrame += SDO_MAX_SEGMENT_SIZE;
                    ret = sdoseq_sendData(pSdoComCon_p->sdoSeqConHdl, sizeOfFrame, pFrame);
                }
                else
                {
                    if ((pSdoComCon_p->transferSize == 0) && (pSdoComCon_p->sdoServiceType != kSdoServiceWriteByIndex))
                        return ret;

                    // complete
                    flag = ami_getUint8Le(&pCommandFrame->flags);
                    flag |= SDO_CMDL_FLAG_SEGMCOMPL;
                    ami_setUint8Le(&pCommandFrame->flags,  flag);
                    OPLK_MEMCPY(&pCommandFrame->aCommandData[0], pSdoComCon_p->pData, pSdoComCon_p->transferSize);
                    pSdoComCon_p->transferredBytes += pSdoComCon_p->transferSize;
                    pSdoComCon_p->pData +=pSdoComCon_p->transferSize;
                    ami_setUint16Le(&pCommandFrame->segmentSizeLe, (WORD) pSdoComCon_p->transferSize);

                    sizeOfFrame += pSdoComCon_p->transferSize;
                    pSdoComCon_p->transferSize = 0;
                    ret = sdoseq_sendData(pSdoComCon_p->sdoSeqConHdl, sizeOfFrame, pFrame);
                }
            }
            break;

        case kSdoComSendTypeAbort:
            flag = ami_getUint8Le(&pCommandFrame->flags);
            flag |= (SDO_CMDL_FLAG_RESPONSE | SDO_CMDL_FLAG_ABORT);
            ami_setUint8Le(&pCommandFrame->flags,  flag);

            // copy abort code to frame
            ami_setUint32Le(&pCommandFrame->aCommandData[0], *((UINT32*)pSdoComCon_p->pData));
            ami_setUint16Le(&pCommandFrame->segmentSizeLe, sizeof(UINT32));
            pSdoComCon_p->transferredBytes = sizeof(UINT32);
            pSdoComCon_p->transferSize = 0;

            sizeOfFrame += sizeof(UINT32);
            ret = sdoseq_sendData(pSdoComCon_p->sdoSeqConHdl, sizeOfFrame, pFrame);
            DEBUG_LVL_SDO_TRACE("ERROR: SDO Aborted!\n");
            break;
    }
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Initialize a WriteByIndex command

The function initializes a WriteByIndex SDO command.

\param  pSdoComCon_p            Pointer to SDO command layer connection structure.
\param  pSdoCom_p               Pointer to received command layer data.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError serverInitWriteByIndex(tSdoComCon* pSdoComCon_p, tAsySdoCom* pSdoCom_p)
{
    tOplkError      ret = kErrorOk;
    UINT            index;
    UINT            subindex;
    UINT            bytesToTransfer;
    tObdSize        entrySize;
    tObdAccess      accessType;
    UINT8*          pSrcData;

    // An init of a write -> variable part of header possible

    // check if expedited or segmented transfer
    if ((pSdoCom_p->flags & SDO_CMDL_FLAG_SEGM_MASK) == SDO_CMDL_FLAG_SEGMINIT)
    {
        pSdoComCon_p->sdoTransferType = kSdoTransSegmented;
        index = ami_getUint16Le(&pSdoCom_p->aCommandData[SDO_CMDL_HDR_VAR_SIZE]);
        subindex = ami_getUint8Le(&pSdoCom_p->aCommandData[SDO_CMDL_HDR_VAR_SIZE + 2]);
        pSrcData = &pSdoCom_p->aCommandData[SDO_CMDL_HDR_VAR_SIZE + SDO_CMDL_HDR_WRITEBYINDEX_SIZE];
        pSdoComCon_p->transferSize = ami_getUint32Le(&pSdoCom_p->aCommandData[0]);
        pSdoComCon_p->transferSize -= (SDO_CMDL_HDR_VAR_SIZE + SDO_CMDL_HDR_WRITEBYINDEX_SIZE);
    }
    else if ((pSdoCom_p->flags & SDO_CMDL_FLAG_SEGM_MASK) == SDO_CMDL_FLAG_EXPEDITED)
    {
        pSdoComCon_p->sdoTransferType = kSdoTransExpedited;
        index = ami_getUint16Le(&pSdoCom_p->aCommandData[0]);
        subindex = ami_getUint8Le(&pSdoCom_p->aCommandData[2]);
        pSrcData = &pSdoCom_p->aCommandData[SDO_CMDL_HDR_WRITEBYINDEX_SIZE];
        pSdoComCon_p->transferSize = ami_getUint16Le(&pSdoCom_p->segmentSizeLe);
        pSdoComCon_p->transferSize -= SDO_CMDL_HDR_WRITEBYINDEX_SIZE;
    }
    else
    {
        return ret;                 // just ignore any other transfer type
    }

    ret = obd_getAccessType(index, subindex, &accessType);
    if (ret == kErrorObdSubindexNotExist)
    {
        pSdoComCon_p->lastAbortCode = SDO_AC_SUB_INDEX_NOT_EXIST;
        // send abort
        // d.k. This is wrong: k.t. not needed send abort on end of write
        /*pSdoComCon_p->pData = (UINT8*)pSdoComCon_p->lastAbortCode;
        ret = serverSendFrame(pSdoComCon_p, index, subindex, kSdoComSendTypeAbort);*/
        goto Abort;
    }
    else if (ret != kErrorOk)
    {   // entry doesn't exist
        pSdoComCon_p->lastAbortCode = SDO_AC_OBJECT_NOT_EXIST;
        // send abort
        // d.k. This is wrong: k.t. not needed send abort on end of write
        /*
        pSdoComCon_p->pData = (UINT8*)&abortCode;
        ret = serverSendFrame(pSdoComCon_p, index, subindex, kSdoComSendTypeAbort);*/
        goto Abort;
    }

    // compare access type,  must be read
    if ((accessType & kObdAccWrite) == 0)
    {
        if ((accessType & kObdAccRead) != 0)
        {
            pSdoComCon_p->lastAbortCode = SDO_AC_WRITE_TO_READ_ONLY_OBJ;
        }
        else
        {
            pSdoComCon_p->lastAbortCode = SDO_AC_UNSUPPORTED_ACCESS;
        }

        // send abort
        // d.k. This is wrong: k.t. not needed send abort on end of write
        /*pSdoComCon_p->pData = (UINT8*)&abortCode;
        ret = serverSendFrame(pSdoComCon_p, index, subindex, kSdoComSendTypeAbort);*/
        goto Abort;
    }

    // save service
    pSdoComCon_p->sdoServiceType = kSdoServiceWriteByIndex;
    pSdoComCon_p->transferredBytes = 0;

    // write data to OD
    if (pSdoComCon_p->sdoTransferType == kSdoTransExpedited)
    {   // expedited transfer, size checking is done by obd_writeEntryFromLe()

        ret = obd_writeEntryFromLe(index, subindex, pSrcData, pSdoComCon_p->transferSize);
        switch (ret)
        {
            case kErrorOk:
                break;

            case kErrorObdAccessViolation:
                pSdoComCon_p->lastAbortCode = SDO_AC_UNSUPPORTED_ACCESS;
                // send abort
                goto Abort;
                break;

            case kErrorObdValueLengthError:
                pSdoComCon_p->lastAbortCode = SDO_AC_DATA_TYPE_LENGTH_NOT_MATCH;
                // send abort
                goto Abort;
                break;

            case kErrorObdValueTooHigh:
                pSdoComCon_p->lastAbortCode = SDO_AC_VALUE_RANGE_TOO_HIGH;
                // send abort
                goto Abort;
                break;

            case kErrorObdValueTooLow:
                pSdoComCon_p->lastAbortCode = SDO_AC_VALUE_RANGE_TOO_LOW;
                // send abort
                goto Abort;
                break;

            default:
                pSdoComCon_p->lastAbortCode = SDO_AC_GENERAL_ERROR;
                // send abort
                goto Abort;
                break;
        }

        // send command acknowledge
        ret = serverSendFrame(pSdoComCon_p, 0, 0, kSdoComSendTypeAckRes);

        pSdoComCon_p->transferSize = 0;
        return ret;
    }
    else
    {
        // get size of the object to check if it fits
        // because we directly write to the destination memory
        // d.k. no one calls the user OD callback function

        entrySize = obd_getDataSize(index, subindex);
        if (entrySize < pSdoComCon_p->transferSize)
        {   // parameter too big
            pSdoComCon_p->lastAbortCode = SDO_AC_DATA_TYPE_LENGTH_TOO_HIGH;
            // send abort
            // d.k. This is wrong: k.t. not needed send abort on end of write
            /*pSdoComCon_p->pData = (UINT8*)&abortCode;
            ret = serverSendFrame(pSdoComCon_p, index, subindex, kSdoComSendTypeAbort);*/
            goto Abort;
        }

        bytesToTransfer = ami_getUint16Le(&pSdoCom_p->segmentSizeLe);
        bytesToTransfer -= (SDO_CMDL_HDR_VAR_SIZE + SDO_CMDL_HDR_WRITEBYINDEX_SIZE);
        pSdoComCon_p->pData = (UINT8*)obd_getObjectDataPtr(index, subindex);    // get pointer to object entry
        if (pSdoComCon_p->pData == NULL)
        {
            pSdoComCon_p->lastAbortCode = SDO_AC_GENERAL_ERROR;
            // send abort
            // d.k. This is wrong: k.t. not needed send abort on end of write
/*            pSdoComCon_p->pData = (UINT8*)&pSdoComCon_p->lastAbortCode;
            ret = serverSendFrame(pSdoComCon_p, index, subindex, kSdoComSendTypeAbort);*/
            goto Abort;
        }

        OPLK_MEMCPY(pSdoComCon_p->pData, pSrcData, bytesToTransfer);
        pSdoComCon_p->transferredBytes = bytesToTransfer;
        pSdoComCon_p->transferSize -= bytesToTransfer;
        (/*(UINT8*)*/pSdoComCon_p->pData) += bytesToTransfer;

        // send acknowledge without any Command layer data
        ret = sdoseq_sendData(pSdoComCon_p->sdoSeqConHdl, 0, (tPlkFrame*)NULL);
        return ret;
    }

Abort:
    if (pSdoComCon_p->lastAbortCode != 0)
    {
        // send abort
        pSdoComCon_p->pData = (UINT8*)&pSdoComCon_p->lastAbortCode;
        ret = serverSendFrame(pSdoComCon_p, index, subindex, kSdoComSendTypeAbort);
        // reset abort code
        pSdoComCon_p->lastAbortCode = 0;
        pSdoComCon_p->transferSize = 0;
    }

    return ret;
}
#endif

#if defined(CONFIG_INCLUDE_SDOC)

//------------------------------------------------------------------------------
/**
\brief  Send an SDO command layer frame from a SDO client

The function starts an SDO transfer and sends all further frames..

\param  pSdoComCon_p            Pointer to SDO command layer connection structure.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError clientSend(tSdoComCon* pSdoComCon_p)
{
    tOplkError      ret = kErrorOk;
    UINT8           aFrame[SDO_MAX_FRAME_SIZE];
    tPlkFrame*      pFrame;
    tAsySdoCom*     pCommandFrame;
    UINT            sizeOfFrame;
    UINT8           flags;
    UINT8*          pPayload;
    UINT            payloadSize;

    pFrame = (tPlkFrame*)&aFrame[0];

    OPLK_MEMSET(&aFrame[0], 0x00, sizeof(aFrame));

    // build generic part of frame
    pCommandFrame = &pFrame->data.asnd.payload.sdoSequenceFrame.sdoSeqPayload;
    ami_setUint8Le(&pCommandFrame->commandId, pSdoComCon_p->sdoServiceType);
    ami_setUint8Le(&pCommandFrame->transactionId, pSdoComCon_p->transactionId);
    sizeOfFrame = SDO_CMDL_HDR_FIXED_SIZE;

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
                    sizeOfFrame += SDO_CMDL_HDR_READBYINDEX_SIZE;
                    pSdoComCon_p->transferredBytes = 1;
                    break;

                case kSdoServiceWriteByIndex:
                    if (pSdoComCon_p->transferSize > (SDO_MAX_SEGMENT_SIZE - SDO_CMDL_HDR_WRITEBYINDEX_SIZE))
                    {   // segmented transfer -> variable part of header needed
                        pSdoComCon_p->sdoTransferType = kSdoTransSegmented;
                        ami_setUint32Le(&pCommandFrame->aCommandData[0], pSdoComCon_p->transferSize + SDO_CMDL_HDR_FIXED_SIZE);
                        pPayload = &pCommandFrame->aCommandData[SDO_CMDL_HDR_VAR_SIZE];
                        ami_setUint16Le(&pCommandFrame->segmentSizeLe, SDO_MAX_SEGMENT_SIZE);
                        flags = SDO_CMDL_FLAG_SEGMINIT;
                        ami_setUint8Le(&pCommandFrame->flags, flags);
                        ami_setUint16Le(pPayload, (WORD)pSdoComCon_p->targetIndex);
                        pPayload += 2;
                        ami_setUint8Le(pPayload, (UINT8)pSdoComCon_p->targetSubIndex);
                        pPayload += 2;      // on byte for reserved
                        sizeOfFrame += SDO_MAX_SEGMENT_SIZE;
                        payloadSize = SDO_MAX_SEGMENT_SIZE - (SDO_CMDL_HDR_VAR_SIZE + SDO_CMDL_HDR_WRITEBYINDEX_SIZE);
                        OPLK_MEMCPY(pPayload, pSdoComCon_p->pData, payloadSize);
                        pSdoComCon_p->pData += payloadSize;
                        pSdoComCon_p->transferSize -= payloadSize;
                        pSdoComCon_p->transferredBytes = payloadSize;
                    }
                    else
                    {   // expedited transfer
                        pSdoComCon_p->sdoTransferType = kSdoTransExpedited;
                        pPayload = &pCommandFrame->aCommandData[0];
                        ami_setUint16Le(pPayload, (WORD)pSdoComCon_p->targetIndex);
                        pPayload += 2;
                        ami_setUint8Le(pPayload, (UINT8)pSdoComCon_p->targetSubIndex);
                        pPayload += 2;      // + 2 -> one byte for sub index and one byte reserved
                        OPLK_MEMCPY(pPayload, pSdoComCon_p->pData,  pSdoComCon_p->transferSize);
                        sizeOfFrame += (pSdoComCon_p->transferSize + SDO_CMDL_HDR_WRITEBYINDEX_SIZE);
                        ami_setUint16Le(&pCommandFrame->segmentSizeLe, (WORD)(pSdoComCon_p->transferSize + SDO_CMDL_HDR_WRITEBYINDEX_SIZE));
                        pSdoComCon_p->transferredBytes = pSdoComCon_p->transferSize;
                        pSdoComCon_p->transferSize = 0;
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
                // for expedited read is nothing to do -> server sends data
                case kSdoServiceWriteByIndex:
                    // send next frame
                    if (pSdoComCon_p->sdoTransferType == kSdoTransSegmented)
                    {
                        if (pSdoComCon_p->transferSize > SDO_MAX_SEGMENT_SIZE)
                        {   // next segment
                            pPayload = &pCommandFrame->aCommandData[0];
                            ami_setUint16Le(&pCommandFrame->segmentSizeLe, SDO_MAX_SEGMENT_SIZE);
                            flags = SDO_CMDL_FLAG_SEGMENTED;
                            ami_setUint8Le(&pCommandFrame->flags, flags);
                            OPLK_MEMCPY(pPayload, pSdoComCon_p->pData, SDO_MAX_SEGMENT_SIZE);
                            pSdoComCon_p->pData += SDO_MAX_SEGMENT_SIZE;
                            pSdoComCon_p->transferSize -= SDO_MAX_SEGMENT_SIZE;
                            pSdoComCon_p->transferredBytes += SDO_MAX_SEGMENT_SIZE;
                            sizeOfFrame += SDO_MAX_SEGMENT_SIZE;
                        }
                        else
                        {   // end of transfer
                            pPayload = &pCommandFrame->aCommandData[0];
                            ami_setUint16Le(&pCommandFrame->segmentSizeLe, (WORD)pSdoComCon_p->transferSize);
                            flags = SDO_CMDL_FLAG_SEGMCOMPL;
                            ami_setUint8Le(&pCommandFrame->flags, flags);
                            OPLK_MEMCPY(pPayload, pSdoComCon_p->pData, pSdoComCon_p->transferSize);
                            pSdoComCon_p->pData += pSdoComCon_p->transferSize;
                            sizeOfFrame += pSdoComCon_p->transferSize;
                            pSdoComCon_p->transferredBytes += pSdoComCon_p->transferSize;
                            pSdoComCon_p->transferSize = 0;
                        }
                    }
                    else
                    {
                        return ret;
                    }
                    break;

                default:
                    return ret;
                    break;
            }
        }
    }
    else
    {
        return ret;
    }

    // call send function of lower layer
    switch (pSdoComCon_p->sdoProtocolType)
    {
        case kSdoTypeAsnd:
        case kSdoTypeUdp:
            ret = sdoseq_sendData(pSdoComCon_p->sdoSeqConHdl, sizeOfFrame, pFrame);
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

\param  sdoComConHdl_p          Handle of sequence layer connection.
\param  pSdoCom_p               Pointer to received frame.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError clientProcessFrame(tSdoComConHdl sdoComConHdl_p, tAsySdoCom* pSdoCom_p)
{
    tOplkError          ret = kErrorOk;
    UINT8               flags;
    UINT8               transactionId;
    UINT8               command;
    UINT                segmentSize;
    UINT                dataSize;
    ULONG               transferSize;
    tSdoComCon*         pSdoComCon;

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
            ret = transferFinished(sdoComConHdl_p, pSdoComCon, kSdoComTransferTxAborted);
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
                ret = transferFinished(sdoComConHdl_p, pSdoComCon, kSdoComTransferTxAborted);
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
                            pSdoComCon->transferSize = 0;
                            pSdoComCon->transferredBytes = dataSize;
                            break;

                        case SDO_CMDL_FLAG_SEGMINIT:
                            // get total size of transfer including the header
                            transferSize = ami_getUint32Le(&pSdoCom_p->aCommandData[0]);
                            transferSize -= SDO_CMDL_HDR_VAR_SIZE;
                            if (transferSize <= pSdoComCon->transferSize)
                            {   // buffer fits
                                pSdoComCon->transferSize = (UINT)transferSize;
                            }
                            else
                            {   // buffer too small -> send abort
                                pSdoComCon->lastAbortCode = SDO_AC_DATA_TYPE_LENGTH_TOO_HIGH;
                                clientSendAbort(pSdoComCon, pSdoComCon->lastAbortCode);
                                ret = transferFinished(sdoComConHdl_p, pSdoComCon, kSdoComTransferTxAborted);
                                return ret;
                            }

                            // get segment size
                            // check size of buffer
                            segmentSize = ami_getUint16Le(&pSdoCom_p->segmentSizeLe);
                            segmentSize -= SDO_CMDL_HDR_VAR_SIZE;
                            OPLK_MEMCPY(pSdoComCon->pData, &pSdoCom_p->aCommandData[SDO_CMDL_HDR_VAR_SIZE], segmentSize);

                            // correct counter an pointer
                            pSdoComCon->pData += segmentSize;
                            pSdoComCon->transferredBytes = segmentSize;
                            pSdoComCon->transferSize -= segmentSize;
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
                                ret = transferFinished(sdoComConHdl_p, pSdoComCon, kSdoComTransferTxAborted);
                                return ret;
                            }
                            OPLK_MEMCPY(pSdoComCon->pData, &pSdoCom_p->aCommandData[0], segmentSize);
                            pSdoComCon->pData += segmentSize;
                            pSdoComCon->transferredBytes += segmentSize;
                            pSdoComCon->transferSize -= segmentSize;
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
                                ret = transferFinished(sdoComConHdl_p, pSdoComCon, kSdoComTransferTxAborted);
                                return ret;
                            }
                            OPLK_MEMCPY(pSdoComCon->pData, &pSdoCom_p->aCommandData[0], segmentSize);
                            pSdoComCon->pData += segmentSize;
                            pSdoComCon->transferredBytes += segmentSize;
                            pSdoComCon->transferSize  = 0;
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

\param  pSdoComCon_p            Pointer to SDO command layer connection structure.
\param  abortCode_p             Abort code to send.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError clientSendAbort(tSdoComCon* pSdoComCon_p, UINT32 abortCode_p)
{
    tOplkError      ret = kErrorOk;
    UINT8           aFrame[SDO_MAX_FRAME_SIZE];
    tPlkFrame*      pFrame;
    tAsySdoCom*     pCommandFrame;
    UINT            sizeOfFrame;

    pFrame = (tPlkFrame*)&aFrame[0];

    OPLK_MEMSET(&aFrame[0], 0x00, sizeof(aFrame));

    // build generic part of frame
    pCommandFrame = &pFrame->data.asnd.payload.sdoSequenceFrame.sdoSeqPayload;
    ami_setUint8Le(&pCommandFrame->commandId, pSdoComCon_p->sdoServiceType);
    ami_setUint8Le(&pCommandFrame->transactionId, pSdoComCon_p->transactionId);

    sizeOfFrame = SDO_CMDL_HDR_FIXED_SIZE;
    pCommandFrame->flags |= SDO_CMDL_FLAG_ABORT;

    ami_setUint32Le(&pCommandFrame->aCommandData[0], abortCode_p);
    ami_setUint16Le(&pCommandFrame->segmentSizeLe, sizeof(UINT32));

    pSdoComCon_p->transferredBytes = sizeof(UINT32);
    pSdoComCon_p->transferSize = 0;

    sizeOfFrame += sizeof(UINT32);

    pSdoComCon_p->lastAbortCode = abortCode_p;

    // call send function of lower layer
    switch (pSdoComCon_p->sdoProtocolType)
    {
        case kSdoTypeAsnd:
        case kSdoTypeUdp:
            ret = sdoseq_sendData(pSdoComCon_p->sdoSeqConHdl, sizeOfFrame, pFrame);
            if (ret == kErrorSdoSeqConnectionBusy)
            {
                DEBUG_LVL_SDO_TRACE("%s tried to send abort 0x%lX while connection is already closed\n",
                    __func__, (ULONG)abortCode_p);
                ret = kErrorOk;
            }
            break;

        default:
            ret = kErrorSdoComUnsupportedProt;
            break;
    }
    return ret;
}
#endif

//------------------------------------------------------------------------------
/**
\brief  Finish SDO transfer

The function finishes an SDO transfer by calling the applications callback
function.

\param  sdoComConHdl_p          Handle of sequence layer connection.
\param  pSdoComCon_p            Pointer to SDO command layer connection structure.
\param  sdoComConState_p        Connection state of the SDO transfer.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError transferFinished(tSdoComConHdl sdoComConHdl_p, tSdoComCon* pSdoComCon_p,
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
        {
            sdoComFinished.sdoAccessType = kSdoAccessTypeWrite;
        }
        else
        {
            sdoComFinished.sdoAccessType = kSdoAccessTypeRead;
        }

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

///\}
