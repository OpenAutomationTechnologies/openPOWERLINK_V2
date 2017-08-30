/**
********************************************************************************
\file   sdocom-std.c

\brief  Implementation of SDO Command Layer

This file contains the standard command layer implementation.

\ingroup module_sdocom_std
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2015, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
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
#include <user/obdu.h>
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

/** SDO command layer transmit segment size (without the fixed command
 *  layer header).
 *  It is calculated by subtracting the size of all headers from
 *  a maximum SDO TX frame (given by SDO_MAX_TX_FRAME_SIZE). The total size of
 *  headers is calculated for SDO over UDP (which is larger than SDO over ASnd):
 *  14 (Ethernet) + 20 (IPv4) + 8 (UDP) + 4 (PLK) + 4 (SDO Seq.) + 8 (SDO Cmd.)
 *  = 58 Bytes
 */
#define SDO_CMD_SEGM_TX_MAX_SIZE            (SDO_MAX_TX_FRAME_SIZE - 58)
#define SDO_CMD_INIT_READ_SEMG_TX_MAX_SIZE  (SDO_CMD_SEGM_TX_MAX_SIZE - SDO_CMDL_HDR_VAR_SIZE)

#if SDO_CMD_SEGM_TX_MAX_SIZE < 256
#error "SDO command layer segment size to low (limit 256 bytes)!"
#elif SDO_CMD_SEGM_TX_MAX_SIZE > 1456
#error "SDO command layer segment size to high (limit 1456 bytes)!"
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
} eSdoTransType;

/**
\brief SDO transfer data type

Data type for the enumerator \ref eSdoTransType.
*/
typedef UINT32 tSdoTransType;

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
} eSdoServiceType;

/**
\brief SDO service type data type

Data type for the enumerator \ref eSdoServiceType.
*/
typedef UINT32 tSdoServiceType;

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
    kSdoComConEventFrameReceived    = 0x05, ///< Lower layer has received a command layer frame
    kSdoComConEventFrameSent        = 0x06, ///< Lower layer has sent a command layer frame
    kSdoComConEventInitError        = 0x07, ///< Error during initialization of the connection
    kSdoComConEventTimeout          = 0x08, ///< Timeout in lower layer
    kSdoComConEventTransferAbort    = 0x09, ///< Transfer abort by lower layer
#if defined(CONFIG_INCLUDE_SDOC)
    kSdoComConEventInitCon          = 0x0A, ///< Init connection (only client)
    kSdoComConEventAbort            = 0x0B, ///< Abort SDO transfer (only client)
#endif
} eSdoComConEvent;

/**
\brief SDO command layer event data type

Data type for the enumerator \ref eSdoComConEvent.
*/
typedef UINT32 tSdoComConEvent;

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
} eSdoComState;

/**
\brief SDO command layer state data type

Data type for the enumerator \ref eSdoComState.
*/
typedef UINT32 tSdoComState;

#if defined(CONFIG_INCLUDE_SDOS)
/**
\brief Enumeration for SDO server object dictionary access types

This enumeration lists all valid SDO server access types to the object
dictionary.
*/
typedef enum
{
    kSdoComConObdInitReadByIndex    = 0x01, ///< initial read by index forwarded to OD
    kSdoComConObdReadByIndex        = 0x02, ///< non-initial read by index forwarded to OD
    kSdoComConObdInitWriteByIndex   = 0x03, ///< initial write by index forwarded to OD
    kSdoComConObdWriteByIndex       = 0x04  ///< non-initial write by index forwarded to OD
} eSdoObdAccType;

/**
\brief SDO transfer data type

Data type for the enumerator \ref eSdoObdAccType.
*/
typedef UINT8 tSdoObdAccType;
#endif // defined(CONFIG_INCLUDE_SDOS)

/**
\brief  SDO command layer connection control structure

This structure defines the connection control structure of the command layer.
*/
typedef struct
{
    tSdoSeqConHdl       sdoSeqConHdl;        ///< SDO sequence layer connection handle (only valid if not 0)
    tSdoComState        sdoComState;         ///< SDO command layer state
    UINT8               transactionId;       ///< Transaction ID
    UINT                nodeId;              ///< NodeId of the target -> needed to reinit connection after timeout
    tSdoTransType       sdoTransferType;     ///< Transfer Type: Auto, Expedited, Segmented
    tSdoServiceType     sdoServiceType;      ///< Service Type: WriteByIndex, ReadByIndex
    tSdoType            sdoProtocolType;     ///< Protocol Type: Auto, Udp, ASnd
    UINT8*              pData;               ///< Pointer to data
    UINT                transferSize;        ///< Number of bytes to transfer
    UINT                transferredBytes;    ///< Number of bytes already transferred
#if defined(CONFIG_INCLUDE_SDOS)
    tSdoComConHdl       sdoObdConHdl;        ///< OD connection handle (only valid if not 0)
    UINT                pendingTransferSize; ///< Due bytes waiting for confirmation from pending OD access
                                             /**< WriteByIndex: Due bytes waiting for confirmation from pending
                                                                OD write access
                                                  ReadByIndex: Max. Tx buffer size for initial OD read access */
    tSdoObdAccType      sdoObdAccType;       ///< Used for processing decision after the OD access has finished
#endif
    tSdoFinishedCb      pfnTransferFinished; ///< Callback function to be called in the end of the SDO transfer
    void*               pUserArg;            ///< User definable argument pointer
    UINT32              lastAbortCode;       ///< Last abort code
    UINT                targetIndex;         ///< Object index to access
    UINT                targetSubIndex;      ///< Object subindex to access
} tSdoComCon;

/**
\brief  SDO command layer instance structure

This structure describes a SDO command layer instance
*/
typedef struct
{
    tSdoComCon      sdoComCon[CONFIG_SDO_MAX_CONNECTION_COM];   ///< Array to store command layer connections
#if defined(CONFIG_INCLUDE_SDOS)
    tSdoComConHdl   sdoObdConCounter;                           ///< OD connection handle counter for object accesses
    tComdLayerObdCb pfnProcessObdWrite;                         ///< OD callback function for WriteByIndex processing
    tComdLayerObdCb pfnProcessObdRead;                          ///< OD callback function for ReadByIndex processing
#endif
#if defined(WIN32) || defined(_WIN32)
    LPCRITICAL_SECTION  pCriticalSection;
    CRITICAL_SECTION    criticalSection;
#endif
} tSdoComInstance;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static tOplkError sdoInit(tComdLayerObdCb pfnObdWrite_p,
                          tComdLayerObdCb pfnObdRead_p);
static tOplkError sdoExit(void);
static tOplkError receiveCb(tSdoSeqConHdl sdoSeqConHdl_p, tAsySdoCom* pSdoCom_p, UINT dataSize_p);
static tOplkError conStateChangeCb(tSdoSeqConHdl sdoSeqConHdl_p, tAsySdoConState sdoConnectionState_p);
static tOplkError processCmdLayerConnection(tSdoSeqConHdl sdoSeqConHdl_p,
                                            tSdoComConEvent sdoComConEvent_p,
                                            tAsySdoCom* pSdoCom_p);
static tOplkError processStateIdle(tSdoComConHdl sdoComConHdl_p, tSdoComConEvent sdoComConEvent_p,
                            tAsySdoCom* pRecvdCmdLayer_p);
static tOplkError processState(tSdoComConHdl sdoComConHdl_p, tSdoComConEvent sdoComConEvent_p,
                               tAsySdoCom* pSdoCom_p);
static void       initCmdFrameGeneric(tPlkFrame* pPlkFrame_p, UINT plkFrameSize_p,
                                tSdoComCon* pSdoComCon_p, tAsySdoCom** pCommandFrame_p);
static void       setCmdFrameHdrFlag(tAsySdoCom* pCommandFrame_p, UINT8 flag_p);
static void       overwriteCmdFrameHdrFlags(tAsySdoCom* pCommandFrame_p, UINT8 flag_p);
static void       setCmdFrameHdrSegmSize(tAsySdoCom* pCommandFrame_p, UINT size_p);
static void       fillCmdFrameDataSegm(tAsySdoCom* pCommandFrame_p, UINT8* pSrcData_p, UINT size_p);
static void       updateHdlTransfSize(tSdoComCon* pSdoComCon_p, UINT tranferredBytes_p, BOOL fTransferComplete);
static UINT32     getSdoErrorCode(tOplkError errorCode_p);
static void       assignSdoErrorCode(tOplkError errorCode_p, UINT32* pSetSdoError_p);

#if defined(CONFIG_INCLUDE_SDOS)
static tOplkError serverInitCon(tSdoComCon* pSdoComCon_p, tAsySdoCom* pRecvdCmdLayer_p);
static tOplkError serverInitReadByIndex(tSdoComCon* pSdoComCon_p, tAsySdoCom* pSdoCom_p);
static tOplkError serverFinishInitReadByIndex(tSdoComCon* pSdoComCon_p,
                                              tPlkFrame* pPlkFrame_p,
                                              UINT sdoCmdDataSize_p);
static tOplkError serverObdSearchConnection(tSdoComConHdl sdoObdConHdl_p, tSdoComCon** pSdoComCon_p);
static tOplkError serverSaveObdConnectionHdl(tSdoComCon* pSdoComCon_p,
                                             tSdoObdAccType sdoAccessType_p);
static tOplkError serverProcessResponseReadByIndex(tSdoComCon* pSdoComCon_p);
static tOplkError serverFinishReadByIndex(tSdoComCon* pSdoComCon_p,
                                          tPlkFrame* pPlkFrame_p,
                                          UINT sdoCmdDataSize_p);
static void       serverFillCmdFrameDataSegmInit(tAsySdoCom* pCommandFrame_p, UINT8* pSrcData_p, UINT size_p);
static void       serverSetCmdFrameHdrSegmTtlSize(tAsySdoCom* pCommandFrame_p, UINT sizeTotal_p);
static tOplkError serverSendAckResponseFrame(tSdoComCon* pSdoComCon_p);
static tOplkError serverSendResponseFrame(tSdoComCon* pSdoComCon_p,
                                          tPlkFrame* pPlkFrame_p,
                                          UINT sdoCmdDataSize_p);
static tOplkError serverAbortTransfer(tSdoComCon* pSdoComCon_p, UINT32 abortCode_p);
static tOplkError serverInitWriteByIndex(tSdoComCon* pSdoComCon_p, tAsySdoCom* pSdoCom_p);
static tOplkError serverFinishInitWriteByIndex(tSdoComCon* pSdoComCon_p);
static tOplkError serverFinishWriteByIndex(tSdoComCon* pSdoComCon_p);
static tOplkError serverProcessResponseWriteByIndex(tSdoComCon* pSdoComCon_p,
                                                    tAsySdoCom* pRecvdCmdLayer_p);
static tOplkError serverProcessStateServerSegmTrans(tSdoComConHdl sdoComConHdl_p,
                                                    tSdoComConEvent sdoComConEvent_p,
                                                    tAsySdoCom* pRecvdCmdLayer_p);
static tOplkError serverObdFinishCb(tSdoObdConHdl* pObdHdl_p);
#endif // defined(CONFIG_INCLUDE_SDOS)

#if defined(CONFIG_INCLUDE_SDOC)
static tOplkError clientSend(tSdoComCon* pSdoComCon_p);
static tOplkError clientProcessFrame(tSdoComConHdl sdoComConHdl_p, tAsySdoCom* pSdoCom_p);
static tOplkError clientProcessStateWaitInit(tSdoComConHdl sdoComConHdl_p, tSdoComConEvent sdoComConEvent_p,
                                             tAsySdoCom* pRecvdCmdLayer_p);
static tOplkError clientProcessStateConnected(tSdoComConHdl sdoComConHdl_p, tSdoComConEvent sdoComConEvent_p,
                                              tAsySdoCom* pRecvdCmdLayer_p);
static tOplkError clientProcessStateSegmTransfer(tSdoComConHdl sdoComConHdl_p, tSdoComConEvent sdoComConEvent_p,
                                                 tAsySdoCom* pRecvdCmdLayer_p);
static tOplkError clientSendAbort(tSdoComCon* pSdoComCon_p, UINT32 abortCode_p);
static tOplkError clientSdoDefineConnection(tSdoComConHdl* pSdoComConHdl_p,
                                            UINT targetNodeId_p,
                                            tSdoType protType_p);
static tOplkError clientSdoInitTransferByIndex(tSdoComTransParamByIndex* pSdoComTransParam_p);
static tOplkError clientSdoUndefineConnection(tSdoComConHdl sdoComConHdl_p);
static tOplkError clientSdoGetState(tSdoComConHdl sdoComConHdl_p, tSdoComFinished* pSdoComFinished_p);
static UINT       clientSdoGetNodeId(tSdoComConHdl sdoComConHdl_p);
static tOplkError clientSdoAbortTransfer(tSdoComConHdl sdoComConHdl_p, UINT32 abortCode_p);
static tOplkError clientTransferFinished(tSdoComConHdl sdoComConHdl_p,
                                         tSdoComCon* pSdoComCon_p,
                                         tSdoComConState sdoComConState_p);
#endif // defined(CONFIG_INCLUDE_SDOC)

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
   sdoExit,
#if defined(CONFIG_INCLUDE_SDOC)
   clientSdoDefineConnection,
   clientSdoInitTransferByIndex,
   clientSdoUndefineConnection,
   clientSdoGetState,
   clientSdoGetNodeId,
   clientSdoAbortTransfer,
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

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief  Initialize SDO command layer module

The function initializes the command layer module.

\param  pfnObdWrite_p   Callback function for OD write access
\param  pfnObdRead_p    Callback function for OD read access

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError sdoInit(tComdLayerObdCb pfnObdWrite_p,
                          tComdLayerObdCb pfnObdRead_p)
{
    tOplkError ret = kErrorOk;

    OPLK_MEMSET(&sdoComInstance_l, 0x00, sizeof(sdoComInstance_l));

#if defined(CONFIG_INCLUDE_SDOS)
    if ((pfnObdWrite_p != NULL) && (pfnObdRead_p != NULL))
    {
        sdoComInstance_l.pfnProcessObdWrite = pfnObdWrite_p;
        sdoComInstance_l.pfnProcessObdRead = pfnObdRead_p;
    }
    else
    {
        return kErrorSdoComInvalidParam;
    }
#else
    UNUSED_PARAMETER(pfnObdWrite_p);
    UNUSED_PARAMETER(pfnObdRead_p);
#endif // defined(CONFIG_INCLUDE_SDOS)

    ret = sdoseq_init(receiveCb, conStateChangeCb);
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
\brief  Shut down the SDO command layer

The function shuts down the SDO command layer module.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError sdoExit(void)
{
    tOplkError  ret = kErrorOk;

#if defined(WIN32) || defined(_WIN32)
    DeleteCriticalSection(sdoComInstance_l.pCriticalSection);
#endif
    ret = sdoseq_exit();
    return ret;
}

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

    ret = processCmdLayerConnection(sdoSeqConHdl_p, kSdoComConEventRec, pSdoCom_p);
    if (ret == kErrorReject)
    {   // error code modified here, since sequence layer doesn't know about OD
        ret = kErrorSdoComHandleBusy;
    }

    DEBUG_LVL_SDO_TRACE("receiveCb SdoSeqConHdl: 0x%X, First Byte of pSdoCom_p: 0x%02X, dataSize_p: 0x%04X\n",
                         sdoSeqConHdl_p, (WORD)pSdoCom_p->aCommandData[0], dataSize_p);
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Connection state change callback function

The function implements the connection state change callback function. It is
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

        case kAsySdoConStateFrameReceived:
            DEBUG_LVL_SDO_TRACE("One Frame received\n");
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

    ret = processCmdLayerConnection(sdoSeqConHdl_p, sdoComConEvent, (tAsySdoCom*)NULL);
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

\param  sdoSeqConHdl_p          Handle of the SDO sequence layer connection.
\param  sdoComConEvent_p        Event to process for found connection.
\param  pSdoCom_p               Pointer to received command layer data.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError processCmdLayerConnection(tSdoSeqConHdl sdoSeqConHdl_p,
                                            tSdoComConEvent sdoComConEvent_p,
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
#if !defined(CONFIG_INCLUDE_SDOS)
    // Ignore unused parameters
    UNUSED_PARAMETER(pRecvdCmdLayer_p);
#endif

    tOplkError          ret = kErrorOk;
    tSdoComCon*         pSdoComCon;

    pSdoComCon = &sdoComInstance_l.sdoComCon[sdoComConHdl_p];

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
            ret = serverInitCon(pSdoComCon, pRecvdCmdLayer_p);
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
            ret = serverProcessStateServerSegmTrans(sdoComConHdl_p, sdoComConEvent_p, pRecvdCmdLayer_p);
            break;
#endif // defined(CONFIG_INCLUDE_SDOS)

#if defined(CONFIG_INCLUDE_SDOC)
        //----------------------------------------------------------------------
        // SDO Client part
        // wait for finish of establishing connection
        case kSdoComStateClientWaitInit:
            ret = clientProcessStateWaitInit(sdoComConHdl_p, sdoComConEvent_p, pRecvdCmdLayer_p);
            break;

        case kSdoComStateClientConnected:
            ret = clientProcessStateConnected(sdoComConHdl_p, sdoComConEvent_p, pRecvdCmdLayer_p);
            break;

        // process segmented transfer
        case kSdoComStateClientSegmTrans:
            ret = clientProcessStateSegmTransfer(sdoComConHdl_p, sdoComConEvent_p, pRecvdCmdLayer_p);
            break;
#endif // #if defined(CONFIG_INCLUDE_SDOC)
    }

#if defined(WIN32) || defined(_WIN32)
    DEBUG_LVL_SDO_TRACE("\n\tLeaveCriticalSection processState\n\n");
    LeaveCriticalSection(sdoComInstance_l.pCriticalSection);
#endif
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Initialize SDO command layer of an empty POWERLINK frame

The function zeros the POWERLINK frame and build a generic SDO command layer.

\param  pPlkFrame_p         Pointer to empty POWERLINK frame
\param  plkFrameSize_p      Size of POWERLINK frame
\param  pSdoComCon_p        Pointer to SDO command layer connection structure
\param  pCommandFrame_p     Returns pointer to command layer start
*/
//------------------------------------------------------------------------------
static void initCmdFrameGeneric(tPlkFrame* pPlkFrame_p, UINT plkFrameSize_p,
                                tSdoComCon* pSdoComCon_p, tAsySdoCom** pCommandFrame_p)
{
    tAsySdoCom* pCmdFrame;

    OPLK_MEMSET(pPlkFrame_p, 0x00, plkFrameSize_p);

    // build generic part of command frame
    pCmdFrame = &pPlkFrame_p->data.asnd.payload.sdoSequenceFrame.sdoSeqPayload;
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

\param  pCommandFrame_p     Pointer to command layer start
\param  flag_p              Flag(s) to be set
*/
//------------------------------------------------------------------------------
static void setCmdFrameHdrFlag(tAsySdoCom* pCommandFrame_p, UINT8 flag_p)
{
    UINT8           flag;

    flag = ami_getUint8Le(&pCommandFrame_p->flags);
    flag |= flag_p;
    ami_setUint8Le(&pCommandFrame_p->flags,  flag);
}

//------------------------------------------------------------------------------
/**
\brief  Overwrite the flags field in a command layer frame

The function sets the flags field in a command layer frame. Existing flags
are overwritten.

\param  pCommandFrame_p     Pointer to command layer start
\param  flag_p              Flag(s) to be set
*/
//------------------------------------------------------------------------------
static void overwriteCmdFrameHdrFlags(tAsySdoCom* pCommandFrame_p, UINT8 flag_p)
{
    ami_setUint8Le(&pCommandFrame_p->flags,  flag_p);
}

//------------------------------------------------------------------------------
/**
\brief  Set the segment size of a command layer frame

The function sets the segment size header field of a command layer frame.

\param  pCommandFrame_p     Pointer to command layer start
\param  size_p              Command layer segment size
*/
//------------------------------------------------------------------------------
static void setCmdFrameHdrSegmSize(tAsySdoCom* pCommandFrame_p, UINT size_p)
{
    ami_setUint16Le(&pCommandFrame_p->segmentSizeLe, size_p);
}

//------------------------------------------------------------------------------
/**
\brief  Copy data to a non-init segmented or expedited SDO command layer frame

The function copies data to the payload section of a segmented SDO command
layer for non-initial (second to last) segmented transfer frames.
It can also be used for an expedited ReadByIndex response.

\param  pCommandFrame_p     Pointer to command layer start
\param  pSrcData_p          Pointer to data source start
\param  size_p              Size of data to copy
*/
//------------------------------------------------------------------------------
static void fillCmdFrameDataSegm(tAsySdoCom* pCommandFrame_p, UINT8* pSrcData_p, UINT size_p)
{
    OPLK_MEMCPY(&pCommandFrame_p->aCommandData[0], pSrcData_p, size_p);
}

//------------------------------------------------------------------------------
/**
\brief  Update the SDO command layer connection structure size fields

The function updates the SDO command layer connection structure size related
members.

\param  pSdoComCon_p        Pointer to SDO command layer connection structure
\param  tranferredBytes_p   Size of transferred command layer data in bytes
\param  fTransferComplete   Flag indicating a completed transfer
                            (TRUE: completed, FALSE: incomplete)
*/
//------------------------------------------------------------------------------
static void updateHdlTransfSize(tSdoComCon* pSdoComCon_p, UINT tranferredBytes_p, BOOL fTransferComplete)
{
    if (fTransferComplete)
    {
        pSdoComCon_p->transferSize = 0;
    }
    else
    {
        // prepare next transfer
        pSdoComCon_p->transferSize -= tranferredBytes_p;

#if defined(CONFIG_INCLUDE_SDOC)
        pSdoComCon_p->pData += tranferredBytes_p;
#endif // defined(CONFIG_INCLUDE_SDOC)
    }

    pSdoComCon_p->transferredBytes += tranferredBytes_p;
}

//------------------------------------------------------------------------------
/**
\brief  Get SDO error code from openPOWERLINK stack error

The function translates an eOplkError code to an SDO error code.

\param  errorCode_p         Error code of type eOplkError
\return The function returns an SDO error code.
*/
//------------------------------------------------------------------------------
static UINT32 getSdoErrorCode(tOplkError errorCode_p)
{
    UINT32 sdoErrorCode = 0;

    switch (errorCode_p)
    {
        case kErrorOk:
        case kErrorReject:
            sdoErrorCode = 0;
            break;

        case kErrorObdIndexNotExist:
        case kErrorObdVarEntryNotExist:
            sdoErrorCode = SDO_AC_OBJECT_NOT_EXIST;
            break;

        case kErrorObdSubindexNotExist:
            sdoErrorCode = SDO_AC_SUB_INDEX_NOT_EXIST;
            break;

        case kErrorObdReadViolation:
            sdoErrorCode = SDO_AC_READ_TO_WRITE_ONLY_OBJ;
            break;

        case kErrorObdWriteViolation:
            sdoErrorCode = SDO_AC_WRITE_TO_READ_ONLY_OBJ;
            break;

        case kErrorObdAccessViolation:
            sdoErrorCode = SDO_AC_UNSUPPORTED_ACCESS;
            break;

        case kErrorObdValueTooLow:
            sdoErrorCode = SDO_AC_VALUE_RANGE_TOO_LOW;
            break;

        case kErrorObdValueTooHigh:
            sdoErrorCode = SDO_AC_VALUE_RANGE_TOO_HIGH;
            break;

        case kErrorObdValueLengthError:
            sdoErrorCode = SDO_AC_DATA_TYPE_LENGTH_NOT_MATCH;
            break;

        case kErrorObdOutOfMemory:
            sdoErrorCode = SDO_AC_OUT_OF_MEMORY;
            break;

        case kErrorObdNoConfigData:
            sdoErrorCode = SDO_AC_CONFIG_DATA_EMPTY;
            break;

        default:
            sdoErrorCode = SDO_AC_GENERAL_ERROR;
            break;
    }

    return sdoErrorCode;
}

//------------------------------------------------------------------------------
/**
\brief  Sets an SDO error code to the provided address

The function sets an SDO error code to the provided address in case of an error.
If there is no error, no value is written. The SDO error code is translated
from an eOplkError (\ref tOplkError) error code.

\param[in]   errorCode_p        Error code of type eOplkError
\param[out]  pSetSdoError_p     SDO error code is written to this address
*/
//------------------------------------------------------------------------------
static void assignSdoErrorCode(tOplkError errorCode_p, UINT32* pSetSdoError_p)
{
    if ((errorCode_p != kErrorOk) && (errorCode_p != kErrorReject))
    {
        if (pSetSdoError_p != NULL)
        {
            *pSetSdoError_p = getSdoErrorCode(errorCode_p);
        }
    }
}

#if defined(CONFIG_INCLUDE_SDOS)
//------------------------------------------------------------------------------
/**
\brief  Initialize SDO server connection

The function initializes the SDO server connection

\param  pSdoComCon_p        Command layer connection control handle
\param  pRecvdCmdLayer_p    SDO command layer part of received frame

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError serverInitCon(tSdoComCon* pSdoComCon_p, tAsySdoCom* pRecvdCmdLayer_p)
{
    tOplkError      ret = kErrorOk;

    // check if init of a transfer and no SDO abort
    if ((pRecvdCmdLayer_p->flags & SDO_CMDL_FLAG_RESPONSE) == 0)
    {   // SDO request
        if ((pRecvdCmdLayer_p->flags & SDO_CMDL_FLAG_ABORT) == 0)
        {   // no SDO abort, save transaction id
            pSdoComCon_p->transactionId = ami_getUint8Le(&pRecvdCmdLayer_p->transactionId);

            switch (pRecvdCmdLayer_p->commandId)
            {
                case kSdoServiceNIL:
                    // simply acknowledge NIL command on sequence layer
                    ret = sdoseq_sendData(pSdoComCon_p->sdoSeqConHdl, 0, (tPlkFrame*)NULL);
                    break;

                case kSdoServiceReadByIndex:
                    // read by index, search entry and start transfer
                    ret = serverInitReadByIndex(pSdoComCon_p, pRecvdCmdLayer_p);
                    break;

                case kSdoServiceWriteByIndex:
                    // search entry an start write
                    ret = serverInitWriteByIndex(pSdoComCon_p, pRecvdCmdLayer_p);
                    break;

                default:
                    //  unsupported command -> send abort
                    ret = serverAbortTransfer(pSdoComCon_p, SDO_AC_UNKNOWN_COMMAND_SPECIFIER);
                    break;
            }
        }
    }
    else
    {   // this command layer handle is not responsible
        // (wrong direction or wrong transaction ID)
        return kErrorSdoComNotResponsible;
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Initialize a ReadByIndex command

The function initializes a ReadByIndex SDO command.

\param  pSdoComCon_p    Pointer to SDO command layer connection structure.
\param  pSdoCom_p       Pointer to received command layer data.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError serverInitReadByIndex(tSdoComCon* pSdoComCon_p, tAsySdoCom* pSdoCom_p)
{
    tOplkError      ret = kErrorOk;
    UINT            index;
    UINT            subindex;
    tSdoObdConHdl   obdHdl;
    UINT8           aFrame[SDO_MAX_TX_FRAME_SIZE];
    tPlkFrame*      pFrame = (tPlkFrame*)&aFrame[0];
    UINT            maxReadBuffSize = SDO_CMD_INIT_READ_SEMG_TX_MAX_SIZE;

    OPLK_MEMSET(pFrame, 0x00, sizeof(aFrame));

    // get requested object of received read command
    index = ami_getUint16Le(&pSdoCom_p->aCommandData[0]);
    subindex = ami_getUint8Le(&pSdoCom_p->aCommandData[2]);

    // save service
    pSdoComCon_p->targetIndex = index;
    pSdoComCon_p->targetSubIndex = subindex;
    pSdoComCon_p->sdoServiceType = kSdoServiceReadByIndex;
    pSdoComCon_p->transferredBytes = 0;
    pSdoComCon_p->pendingTransferSize = maxReadBuffSize;

    // request command data from OD
    OPLK_MEMSET(&obdHdl, 0x00, sizeof(obdHdl));
    obdHdl.index = index;
    obdHdl.subIndex = subindex;
    // shift command payload buffer artificially to the right, and relocate header later if necessary
    obdHdl.pDstData = &pFrame->data.asnd.payload.sdoSequenceFrame.sdoSeqPayload.aCommandData[SDO_CMDL_HDR_VAR_SIZE];
    obdHdl.dataSize = maxReadBuffSize;
    ret = serverSaveObdConnectionHdl(pSdoComCon_p, kSdoComConObdInitReadByIndex);
    if (ret != kErrorOk)
    {
        return ret;
    }
    obdHdl.sdoHdl = pSdoComCon_p->sdoObdConHdl;
    ret = sdoComInstance_l.pfnProcessObdRead(&obdHdl, serverObdFinishCb);
    assignSdoErrorCode(ret, &pSdoComCon_p->lastAbortCode);
    if (ret == kErrorReject)
    {   // exit immediately, further processing happens with callback
        return ret;
    }

    // save service - init read by index
    pSdoComCon_p->transferSize = obdHdl.totalPendSize; // from function call

    if (pSdoComCon_p->transferSize <= maxReadBuffSize)
    {   // expedited transfer -> relocate frame header

        // SDO command payload in buffer set to variable header location,
        // therefore relocate frame header to align it to the payload
        pFrame = (void*)((BYTE*)pFrame + SDO_CMDL_HDR_VAR_SIZE);
    }

    if (pSdoComCon_p->lastAbortCode != 0)
    {
        goto Abort;
    }

    ret = serverFinishInitReadByIndex(pSdoComCon_p,
                                      pFrame,
                                      obdHdl.dataSize);
    if (ret != kErrorOk)
    {
        goto Abort;
    }

Abort:
    if (pSdoComCon_p->lastAbortCode != 0)
    {
        ret = serverAbortTransfer(pSdoComCon_p, pSdoComCon_p->lastAbortCode);
        // reset abort code
        pSdoComCon_p->lastAbortCode = 0;
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Finish an initial ReadByIndex command

The function completes an initial ReadByIndex SDO command.

\param  pSdoComCon_p        Pointer to SDO command layer connection structure.
\param  pPlkFrame_p         Pointer to PLK frame with SDO command layer data and
                            size sdoCmdDataSize_p (optional).
                            If not used, little endian command layer data
                            will be copied from pSdoComCon_p->pData.
\param  sdoCmdDataSize_p    Size of command layer data

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError serverFinishInitReadByIndex(tSdoComCon* pSdoComCon_p,
                                              tPlkFrame* pPlkFrame_p,
                                              UINT sdoCmdDataSize_p)
{
    tOplkError      ret = kErrorOk;

    pSdoComCon_p->sdoObdConHdl = 0;

    // decide if segmented or expedited transfer

    // NOTE: maximum Tx buffer size was stored in pSdoComCon_p->pendingTransferSize
    // It is known by this module, that the send function also provides
    // the same maximum size for initial segmented read transfers, therefore
    // this decision can be taken here.
    if (pSdoComCon_p->transferSize > pSdoComCon_p->pendingTransferSize)
    {
        pSdoComCon_p->sdoTransferType = kSdoTransSegmented;
        pSdoComCon_p->sdoComState = kSdoComStateServerSegmTrans;
    }
    else
    {
        pSdoComCon_p->sdoTransferType = kSdoTransExpedited;
    }

    ret = serverSendResponseFrame(pSdoComCon_p,
                                  pPlkFrame_p,
                                  sdoCmdDataSize_p);
    if (ret != kErrorOk)
    {
        ret = serverAbortTransfer(pSdoComCon_p, SDO_AC_GENERAL_ERROR);
        goto Exit;
    }

    // check next state
    if (pSdoComCon_p->transferSize == 0)
    {   // already finished -> stay idle
        pSdoComCon_p->sdoComState = kSdoComStateIdle;
        pSdoComCon_p->lastAbortCode = 0;
    }

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Processes the ReadByIndex response to a received frame

The function processes a ReadByIndex command layer response to a received frame.

\param  pSdoComCon_p        Command layer connection control handle

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError serverProcessResponseReadByIndex(tSdoComCon* pSdoComCon_p)
{
    tOplkError      ret = kErrorOk;
    UINT8           aFrame[SDO_MAX_TX_FRAME_SIZE];
    tPlkFrame*      pFrame = (tPlkFrame*)&aFrame[0];
    UINT            maxReadBuffSize = SDO_CMD_SEGM_TX_MAX_SIZE;
    tSdoObdConHdl   obdHdl;

    OPLK_MEMSET(pFrame, 0x00, sizeof(aFrame));

    // send next frame
    // request command data from OD
    OPLK_MEMSET(&obdHdl, 0x00, sizeof(obdHdl));
    obdHdl.index = pSdoComCon_p->targetIndex;
    obdHdl.subIndex = pSdoComCon_p->targetSubIndex;
    obdHdl.pDstData = &pFrame->data.asnd.payload.sdoSequenceFrame.sdoSeqPayload.aCommandData[0];
    obdHdl.dataSize = maxReadBuffSize;
    obdHdl.dataOffset = pSdoComCon_p->transferredBytes;
    obdHdl.totalPendSize = pSdoComCon_p->transferSize;
    ret = serverSaveObdConnectionHdl(pSdoComCon_p, kSdoComConObdReadByIndex);
    if (ret != kErrorOk)
    {
        return ret;
    }
    obdHdl.sdoHdl = pSdoComCon_p->sdoObdConHdl;
    ret = sdoComInstance_l.pfnProcessObdRead(&obdHdl, serverObdFinishCb);
    assignSdoErrorCode(ret, &pSdoComCon_p->lastAbortCode);
    if (ret == kErrorReject)
    {   // exit immediately, further processing happens with callback
        return ret;
    }

    if (pSdoComCon_p->lastAbortCode != 0)
    {
        goto Abort;
    }

    ret = serverFinishReadByIndex(pSdoComCon_p,
                                  pFrame,
                                  obdHdl.dataSize);
    if (ret != kErrorOk)
    {
        goto Abort;
    }

Abort:
    if (pSdoComCon_p->lastAbortCode != 0)
    {
        ret = serverAbortTransfer(pSdoComCon_p, pSdoComCon_p->lastAbortCode);
        // reset abort code
        pSdoComCon_p->lastAbortCode = 0;
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Finish a segmented ReadByIndex command

The function completes a non-initial segmented ReadByIndex SDO command.

\param  pSdoComCon_p        Pointer to SDO command layer connection structure.
\param  pPlkFrame_p         Pointer to PLK frame with SDO command layer data and
                            size sdoCmdDataSize_p (optional).
                            If not used, little endian command layer data
                            will be copied from pSdoComCon_p->pData.
\param  sdoCmdDataSize_p    Size of command layer data

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError serverFinishReadByIndex(tSdoComCon* pSdoComCon_p,
                                          tPlkFrame* pPlkFrame_p,
                                          UINT sdoCmdDataSize_p)
{
    tOplkError      ret = kErrorOk;

    pSdoComCon_p->sdoObdConHdl = 0;

    if (sdoCmdDataSize_p > pSdoComCon_p->transferSize)
    {
        ret = serverAbortTransfer(pSdoComCon_p, SDO_AC_DATA_TYPE_LENGTH_TOO_HIGH);
        goto Exit;
    }

    ret = serverSendResponseFrame(pSdoComCon_p,
                                  pPlkFrame_p,
                                  sdoCmdDataSize_p);
    if (ret != kErrorOk)
    {
        ret = serverAbortTransfer(pSdoComCon_p, SDO_AC_GENERAL_ERROR);
        goto Exit;
    }

    // if all send -> back to idle
    if (pSdoComCon_p->transferSize == 0)
    {   // back to idle
        pSdoComCon_p->sdoComState = kSdoComStateIdle;
        pSdoComCon_p->lastAbortCode = 0;
    }

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Search SDO connection handle by OD handle number

The function searches for SDO connection control structures by using a handle
number previously provided to an external (object dictionary) module.

\param[in]   sdoObdConHdl_p  Previously provided connection handle number
\param[out]  ppSdoComCon_p   Pointer to found SDO control structure,
                             NULL if not found (call by reference).

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError serverObdSearchConnection(tSdoComConHdl sdoObdConHdl_p, tSdoComCon** ppSdoComCon_p)
{
    tSdoComCon*         pSdoComCon;
    tSdoComConHdl       hdlCount;

    // get pointer to first element of the array
    pSdoComCon = &sdoComInstance_l.sdoComCon[0];
    hdlCount = 0;

    // get pointer to control structure of connection
    while (hdlCount < CONFIG_SDO_MAX_CONNECTION_COM)
    {
        if (pSdoComCon->sdoObdConHdl == sdoObdConHdl_p)
        {   // matching command layer handle found
            if (pSdoComCon->sdoSeqConHdl == 0)
                return kErrorSdoComInvalidHandle;

            *ppSdoComCon_p = pSdoComCon;
            return kErrorOk;
        }
        pSdoComCon++;
        hdlCount++;
    }

    *ppSdoComCon_p = NULL;
    return kErrorSdoComInvalidHandle;
}

//------------------------------------------------------------------------------
/**
\brief  Generate and save a new connection handle for OD access

The function generates a new connection number and stores it to the command
layer control structure. If this was successful, pSdoComCon_p->sdoObdConHdl
can be forwarded as handle to the OD. The OD has to provide this handle
together with a delayed answer.

\param  pSdoComCon_p     pointer SDO control structure
\param  sdoAccessType_p  SDO server object dictionary access type

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError serverSaveObdConnectionHdl(tSdoComCon* pSdoComCon_p,
                                             tSdoObdAccType sdoAccessType_p)
{
    if (pSdoComCon_p->sdoObdConHdl == 0)
    {   // set new handle
        if (++sdoComInstance_l.sdoObdConCounter == 0)
        {
            ++sdoComInstance_l.sdoObdConCounter;
        }
        pSdoComCon_p->sdoObdConHdl = sdoComInstance_l.sdoObdConCounter;
        pSdoComCon_p->sdoObdAccType = sdoAccessType_p;
        return kErrorOk;
    }
    else
    {   // OD connection is currently in use for this command layer connection
        return kErrorSdoComHandleBusy;
    }
}

//------------------------------------------------------------------------------
/**
\brief  Copy data to an initial segmented SDO command layer frame

The function copies data to the payload section of an SDO command layer frame
for initial segmented transfer frames.

\param  pCommandFrame_p     Pointer to command layer start
\param  pSrcData_p          Pointer to data source start
\param  size_p              Size of data to copy
*/
//------------------------------------------------------------------------------
static void serverFillCmdFrameDataSegmInit(tAsySdoCom* pCommandFrame_p, UINT8* pSrcData_p, UINT size_p)
{
    OPLK_MEMCPY(&pCommandFrame_p->aCommandData[SDO_CMDL_HDR_VAR_SIZE], pSrcData_p, size_p);
}

//------------------------------------------------------------------------------
/**
\brief  Set the total transfer size for an initial segmented transfer

The function sets the total transfer size for an initial segmented transfer in
the variable header of a command layer frame.

\param  pCommandFrame_p     Pointer to command layer start
\param  sizeTotal_p         Total transfer size
*/
//------------------------------------------------------------------------------
static void serverSetCmdFrameHdrSegmTtlSize(tAsySdoCom* pCommandFrame_p, UINT sizeTotal_p)
{
    // init data size in variable header, which includes itself
    ami_setUint32Le(&pCommandFrame_p->aCommandData[0], sizeTotal_p);
}

//------------------------------------------------------------------------------
/**
\brief  Send an empty SDO command layer ack frame from SDO server

The function creates and sends a command layer acknowledge frame from an
 SDO server. This frame is without command layer data.

\param  pSdoComCon_p    Pointer to SDO command layer connection structure.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError serverSendAckResponseFrame(tSdoComCon* pSdoComCon_p)
{
    tOplkError      ret = kErrorOk;
    UINT8           aFrame[SDO_MAX_TX_FRAME_SIZE];
    tPlkFrame*      pFrame;
    tAsySdoCom*     pCommandFrame;
    UINT            sizeOfCmdFrame;

    pFrame = (tPlkFrame*)&aFrame[0];
    initCmdFrameGeneric(pFrame, sizeof(aFrame), pSdoComCon_p, &pCommandFrame);

    overwriteCmdFrameHdrFlags(pCommandFrame, SDO_CMDL_FLAG_RESPONSE);
    sizeOfCmdFrame = SDO_CMDL_HDR_FIXED_SIZE;
    ret = sdoseq_sendData(pSdoComCon_p->sdoSeqConHdl, sizeOfCmdFrame, pFrame);

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Send an SDO command layer response frame from an SDO server

The function creates and sends a command layer frame from an SDO server.
It can either copy data directly from pSdoComCon_p->data into an empty frame,
or if provided, only set the command layer header around existing command
layer payload in pPlkFrame_p.

\param  pSdoComCon_p        Pointer to SDO command layer connection structure.
\param  pPlkFrame_p         Pointer to PLK frame with SDO command layer data.
                            If not used (NULL), little endian command layer data
                            will be copied from pSdoComCon_p->pData.
\param  sdoCmdDataSize_p    Size of command layer data

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError serverSendResponseFrame(tSdoComCon* pSdoComCon_p,
                                          tPlkFrame* pPlkFrame_p,
                                          UINT sdoCmdDataSize_p)
{
    tOplkError      ret = kErrorOk;
    UINT8           aFrame[SDO_MAX_TX_FRAME_SIZE];
    tPlkFrame*      pFrame;
    tAsySdoCom*     pCommandFrame;
    UINT            sizeOfCmdFrame;
    UINT            sizeOfCmdData = 0;  ///> command layer payload size

    if (pPlkFrame_p == NULL)
    {   // only pointer to frame with little endian command layer data is provided
        // by caller -> this function will do the copy operation, and provide the buffer
        pFrame = (tPlkFrame*)&aFrame[0];
        initCmdFrameGeneric(pFrame, sizeof(aFrame), pSdoComCon_p, &pCommandFrame);
    }
    else
    {   // frame with command layer data is provided by caller
        pFrame = pPlkFrame_p;
        // build generic part of command frame
        pCommandFrame = &pFrame->data.asnd.payload.sdoSequenceFrame.sdoSeqPayload;
        ami_setUint8Le(&pCommandFrame->commandId, pSdoComCon_p->sdoServiceType);
        ami_setUint8Le(&pCommandFrame->transactionId, pSdoComCon_p->transactionId);
    }

    setCmdFrameHdrFlag(pCommandFrame, SDO_CMDL_FLAG_RESPONSE);

    // setup command layer
    if (pSdoComCon_p->sdoTransferType == kSdoTransExpedited)
    {   // Expedited transfer
        if (sdoCmdDataSize_p == pSdoComCon_p->transferSize)
        {
            sizeOfCmdData = sdoCmdDataSize_p;
        }
        else
        {
            ret = kErrorSdoSeqFrameSizeError;
            goto Exit;
        }

        // copy data into frame, if frame not provided by caller
        if (pPlkFrame_p == NULL)
        {
            fillCmdFrameDataSegm(pCommandFrame, pSdoComCon_p->pData, sizeOfCmdData);
        }

        setCmdFrameHdrSegmSize(pCommandFrame, sizeOfCmdData);

        updateHdlTransfSize(pSdoComCon_p, sizeOfCmdData, TRUE);
        sizeOfCmdFrame = SDO_CMDL_HDR_FIXED_SIZE + sizeOfCmdData;
        ret = sdoseq_sendData(pSdoComCon_p->sdoSeqConHdl, sizeOfCmdFrame, pFrame);
    }
    else if (pSdoComCon_p->sdoTransferType == kSdoTransSegmented)
    {   // segmented transfer
        // distinguish between init, segment and complete
        if (pSdoComCon_p->transferredBytes == 0)
        {   // init
            if (sdoCmdDataSize_p <= SDO_CMD_INIT_READ_SEMG_TX_MAX_SIZE)
            {
                sizeOfCmdData = sdoCmdDataSize_p;
            }
            else
            {
                ret = kErrorSdoSeqFrameSizeError;
                goto Exit;
            }

            // copy data into frame, if frame not provided by caller
            if (pPlkFrame_p == NULL)
            {
                serverFillCmdFrameDataSegmInit(pCommandFrame, pSdoComCon_p->pData, sizeOfCmdData);
            }

            setCmdFrameHdrFlag(pCommandFrame, SDO_CMDL_FLAG_SEGMINIT);
            setCmdFrameHdrSegmSize(pCommandFrame, sizeOfCmdData + SDO_CMDL_HDR_VAR_SIZE);
            serverSetCmdFrameHdrSegmTtlSize(pCommandFrame, pSdoComCon_p->transferSize + SDO_CMDL_HDR_VAR_SIZE);

            updateHdlTransfSize(pSdoComCon_p, sizeOfCmdData, FALSE);
            sizeOfCmdFrame = SDO_CMDL_HDR_FIXED_SIZE + SDO_CMDL_HDR_VAR_SIZE + sizeOfCmdData;
            ret = sdoseq_sendData(pSdoComCon_p->sdoSeqConHdl, sizeOfCmdFrame, pFrame);
        }
        else if ((pSdoComCon_p->transferredBytes > 0) && (pSdoComCon_p->transferSize > SDO_CMD_SEGM_TX_MAX_SIZE))
        {   // segment
            if (sdoCmdDataSize_p <= SDO_CMD_SEGM_TX_MAX_SIZE)
            {
                sizeOfCmdData = sdoCmdDataSize_p;
            }
            else
            {
                ret = kErrorSdoSeqFrameSizeError;
                goto Exit;
            }

            // copy data into frame, if frame not provided by caller
            if (pPlkFrame_p == NULL)
            {
                fillCmdFrameDataSegm(pCommandFrame, pSdoComCon_p->pData, sizeOfCmdData);
            }

            setCmdFrameHdrFlag(pCommandFrame, SDO_CMDL_FLAG_SEGMENTED);
            setCmdFrameHdrSegmSize(pCommandFrame, sizeOfCmdData);

            updateHdlTransfSize(pSdoComCon_p, sizeOfCmdData, FALSE);
            sizeOfCmdFrame = SDO_CMDL_HDR_FIXED_SIZE + sizeOfCmdData;
            ret = sdoseq_sendData(pSdoComCon_p->sdoSeqConHdl, sizeOfCmdFrame, pFrame);
        }
        else
        {   // complete

            // block sending empty frames from other transfer types than kSdoServiceWriteByIndex
            if ((pSdoComCon_p->transferSize == 0) && (pSdoComCon_p->sdoServiceType != kSdoServiceWriteByIndex))
                return ret;

            if (sdoCmdDataSize_p == pSdoComCon_p->transferSize)
            {
                sizeOfCmdData = sdoCmdDataSize_p;
            }
            else
            {
                ret = kErrorSdoSeqFrameSizeError;
                goto Exit;;
            }

            // copy data into frame, if frame not provided by caller
            if (pPlkFrame_p == NULL)
            {
                fillCmdFrameDataSegm(pCommandFrame, pSdoComCon_p->pData, sizeOfCmdData);
            }

            if (pSdoComCon_p->sdoServiceType == kSdoServiceReadByIndex)
            {
                setCmdFrameHdrFlag(pCommandFrame, SDO_CMDL_FLAG_SEGMCOMPL);
            }
            setCmdFrameHdrSegmSize(pCommandFrame, sizeOfCmdData);

            updateHdlTransfSize(pSdoComCon_p, sizeOfCmdData, TRUE);
            sizeOfCmdFrame = SDO_CMDL_HDR_FIXED_SIZE + sizeOfCmdData;
            ret = sdoseq_sendData(pSdoComCon_p->sdoSeqConHdl, sizeOfCmdFrame, pFrame);
        }
    }

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Abort an SDO transfer

The function sends an abort message, and causes a state change back to the
initial SDO server state.

\param  pSdoComCon_p    Pointer to SDO command layer connection structure.
\param  abortCode_p     Abort code to send.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError serverAbortTransfer(tSdoComCon* pSdoComCon_p, UINT32 abortCode_p)
{
    tOplkError      ret = kErrorOk;
    UINT8           aFrame[SDO_MAX_TX_FRAME_SIZE];
    tPlkFrame*      pFrame;
    tAsySdoCom*     pCommandFrame;
    UINT            sizeOfCmdFrame;
    UINT            sizeOfCmdData;

    pFrame = (tPlkFrame*)&aFrame[0];
    initCmdFrameGeneric(pFrame, sizeof(aFrame), pSdoComCon_p, &pCommandFrame);

    sizeOfCmdData = sizeof(abortCode_p);
    // copy abort code to frame
    ami_setUint32Le(&pCommandFrame->aCommandData[0], abortCode_p);
    setCmdFrameHdrFlag(pCommandFrame, SDO_CMDL_FLAG_RESPONSE | SDO_CMDL_FLAG_ABORT);
    setCmdFrameHdrSegmSize(pCommandFrame, sizeOfCmdData);

    updateHdlTransfSize(pSdoComCon_p, sizeOfCmdData, TRUE);
    sizeOfCmdFrame = SDO_CMDL_HDR_FIXED_SIZE + sizeOfCmdData;
    ret = sdoseq_sendData(pSdoComCon_p->sdoSeqConHdl, sizeOfCmdFrame, pFrame);
    DEBUG_LVL_SDO_TRACE("ERROR: SDO Aborted!\n");

    pSdoComCon_p->sdoComState = kSdoComStateIdle;
    // invalidate connection to OD, so a delayed response from OD recognizes
    // that there is no connection present
    pSdoComCon_p->sdoObdConHdl = 0;

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
    UINT            segmPayloadSize = 0;
    UINT8*          pSrcData;
    tSdoObdConHdl   obdHdl;

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
        segmPayloadSize = ami_getUint16Le(&pSdoCom_p->segmentSizeLe);
        segmPayloadSize -= (SDO_CMDL_HDR_VAR_SIZE + SDO_CMDL_HDR_WRITEBYINDEX_SIZE);
        // prepare next state here, due to the possibility of a pending transfer
        // and immediately following segments
        pSdoComCon_p->sdoComState = kSdoComStateServerSegmTrans;
    }
    else if ((pSdoCom_p->flags & SDO_CMDL_FLAG_SEGM_MASK) == SDO_CMDL_FLAG_EXPEDITED)
    {
        pSdoComCon_p->sdoTransferType = kSdoTransExpedited;
        index = ami_getUint16Le(&pSdoCom_p->aCommandData[0]);
        subindex = ami_getUint8Le(&pSdoCom_p->aCommandData[2]);
        pSrcData = &pSdoCom_p->aCommandData[SDO_CMDL_HDR_WRITEBYINDEX_SIZE];
        pSdoComCon_p->transferSize = ami_getUint16Le(&pSdoCom_p->segmentSizeLe);
        pSdoComCon_p->transferSize -= SDO_CMDL_HDR_WRITEBYINDEX_SIZE;
        // next state assigned later, since no following segments are expected
    }
    else
    {
        return ret; // just ignore any other transfer type
    }

    // save service
    pSdoComCon_p->targetIndex = index;
    pSdoComCon_p->targetSubIndex = subindex;
    pSdoComCon_p->sdoServiceType = kSdoServiceWriteByIndex;
    pSdoComCon_p->transferredBytes = 0;
    pSdoComCon_p->pendingTransferSize = segmPayloadSize;

    // forward command data to OD
    obdHdl.index = index;
    obdHdl.subIndex = subindex;
    obdHdl.pSrcData = pSrcData;
    obdHdl.totalPendSize = pSdoComCon_p->transferSize;
    obdHdl.dataSize = segmPayloadSize;
    obdHdl.dataOffset = 0;              // first segment
    ret = serverSaveObdConnectionHdl(pSdoComCon_p, kSdoComConObdInitWriteByIndex);
    if (ret != kErrorOk)
    {
        return ret;
    }
    obdHdl.sdoHdl = pSdoComCon_p->sdoObdConHdl;
    ret = sdoComInstance_l.pfnProcessObdWrite(&obdHdl, serverObdFinishCb);
    assignSdoErrorCode(ret, &pSdoComCon_p->lastAbortCode);
    if (ret == kErrorReject)
    {   // exit immediately, further processing happens with callback
        return ret;
    }

    if (pSdoComCon_p->lastAbortCode != 0)
    {
        goto Abort;
    }

    ret = serverFinishInitWriteByIndex(pSdoComCon_p);
    if (ret != kErrorOk)
    {
        goto Abort;
    }

Abort:
    if (pSdoComCon_p->lastAbortCode != 0)
    {
        ret = serverAbortTransfer(pSdoComCon_p, pSdoComCon_p->lastAbortCode);
        // reset abort code
        pSdoComCon_p->lastAbortCode = 0;
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Finish an initial WriteByIndex command

The function completes an initial WriteByIndex SDO command.

\param  pSdoComCon_p        Pointer to SDO command layer connection structure.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError serverFinishInitWriteByIndex(tSdoComCon* pSdoComCon_p)
{
    tOplkError      ret = kErrorOk;

    pSdoComCon_p->sdoObdConHdl = 0;

    if (pSdoComCon_p->sdoTransferType == kSdoTransExpedited)
    {   // expedited transfer finished, send command acknowledge
        ret = serverSendAckResponseFrame(pSdoComCon_p);
        pSdoComCon_p->transferSize = 0;

        // prepare next state
        pSdoComCon_p->sdoComState = kSdoComStateIdle;
        pSdoComCon_p->lastAbortCode = 0;
    }
    else
    {   // segmented transfer started
        // Note: State change to kSdoComStateServerSegmTrans already done earlier
        updateHdlTransfSize(pSdoComCon_p, pSdoComCon_p->pendingTransferSize, FALSE);

        // send acknowledge without any command layer data  (sequence layer ack)
        ret = sdoseq_sendData(pSdoComCon_p->sdoSeqConHdl, 0, (tPlkFrame*)NULL);
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Processes the WriteByIndex response to a received frame

The function processes a WriteByIndex command layer response to a received
frame.

\param  pSdoComCon_p        Command layer connection control handle
\param  pRecvdCmdLayer_p    SDO command layer part of received frame

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError serverProcessResponseWriteByIndex(tSdoComCon* pSdoComCon_p, tAsySdoCom* pRecvdCmdLayer_p)
{
    tOplkError      ret = kErrorOk;
    UINT            size;
    tSdoObdConHdl   obdHdl;

    size = ami_getUint16Le(&pRecvdCmdLayer_p->segmentSizeLe);
    if (size > pSdoComCon_p->transferSize)
    {
        pSdoComCon_p->lastAbortCode = SDO_AC_DATA_TYPE_LENGTH_TOO_HIGH;
        ret = serverAbortTransfer(pSdoComCon_p, pSdoComCon_p->lastAbortCode);
        return ret;
    }

    // save size for handle update
    pSdoComCon_p->pendingTransferSize = size;

    // forward command data to OD
    obdHdl.index = pSdoComCon_p->targetIndex;
    obdHdl.subIndex = pSdoComCon_p->targetSubIndex;
    obdHdl.pSrcData = &(pRecvdCmdLayer_p->aCommandData[0]);
    obdHdl.totalPendSize = pSdoComCon_p->transferSize;
    obdHdl.dataSize = size;
    obdHdl.dataOffset = pSdoComCon_p->transferredBytes;
    // check end of transfer before forwarding to object dictionary
    if ((pRecvdCmdLayer_p->flags & SDO_CMDL_FLAG_SEGM_MASK) == SDO_CMDL_FLAG_SEGMCOMPL)
    {   // transfer finished
        pSdoComCon_p->transferSize = 0;
    }
    ret = serverSaveObdConnectionHdl(pSdoComCon_p, kSdoComConObdWriteByIndex);
    if (ret != kErrorOk)
    {
        return ret;
    }
    obdHdl.sdoHdl = pSdoComCon_p->sdoObdConHdl;
    ret = sdoComInstance_l.pfnProcessObdWrite(&obdHdl, serverObdFinishCb);
    assignSdoErrorCode(ret, &pSdoComCon_p->lastAbortCode);
    if (ret == kErrorReject)
    {   // exit immediately, further processing happens with callback
        return ret;
    }

    if (pSdoComCon_p->lastAbortCode != 0)
    {
        goto Abort;
    }

    ret = serverFinishWriteByIndex(pSdoComCon_p);
    if (ret != kErrorOk)
    {
        goto Abort;
    }

Abort:
    if (pSdoComCon_p->lastAbortCode != 0)
    {
        ret = serverAbortTransfer(pSdoComCon_p, pSdoComCon_p->lastAbortCode);
        // reset abort code
        pSdoComCon_p->lastAbortCode = 0;
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Finish a segmented WriteByIndex command

The function completes a non-initial segmented WriteByIndex SDO command.

\param  pSdoComCon_p        Pointer to SDO command layer connection structure.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError serverFinishWriteByIndex(tSdoComCon* pSdoComCon_p)
{
    tOplkError      ret = kErrorOk;

    pSdoComCon_p->sdoObdConHdl = 0;

    if (pSdoComCon_p->transferSize != 0)
    {   // not yet the last segment
        updateHdlTransfSize(pSdoComCon_p, pSdoComCon_p->pendingTransferSize, FALSE);

        // send acknowledge without any Command layer data (sequence layer ack)
        ret = sdoseq_sendData(pSdoComCon_p->sdoSeqConHdl, 0, (tPlkFrame*)NULL);
    }
    else
    {   // transfer finished
        updateHdlTransfSize(pSdoComCon_p, pSdoComCon_p->pendingTransferSize, TRUE);

        if (pSdoComCon_p->lastAbortCode == 0)
        {
            // send empty response indicating completed transfer
            serverSendResponseFrame(pSdoComCon_p, NULL, 0);
            // if all send -> back to idle
            if (pSdoComCon_p->transferSize == 0)
            {   // back to idle
                pSdoComCon_p->sdoComState = kSdoComStateIdle;
                pSdoComCon_p->lastAbortCode = 0;
            }
            else
            {
                ret = serverAbortTransfer(pSdoComCon_p, pSdoComCon_p->lastAbortCode);
                pSdoComCon_p->lastAbortCode = 0;
            }
        }
    }

    return ret;
}

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
static tOplkError serverProcessStateServerSegmTrans(tSdoComConHdl sdoComConHdl_p,
                                                    tSdoComConEvent sdoComConEvent_p,
                                                    tAsySdoCom* pRecvdCmdLayer_p)
{
    tOplkError      ret = kErrorOk;
    tSdoComCon*     pSdoComCon;
    UINT8           flag;

    pSdoComCon = &sdoComInstance_l.sdoComCon[sdoComConHdl_p];

    switch (sdoComConEvent_p)
    {
        // send next frame
        // these events are used for requesting new data from the OD
        // and transferring it to the sequence layer.
        // Practically, kSdoComConEventAckReceived happens only if the sequence
        // layer Tx history buffer is full, and an ack was received.
        case kSdoComConEventAckReceived:
        case kSdoComConEventFrameSent:
            // check if it is a read
            if ((pSdoComCon->sdoServiceType == kSdoServiceReadByIndex) &&
                (pSdoComCon->transferSize != 0))
            {
                // ignore info about last sent segment
                ret = serverProcessResponseReadByIndex(pSdoComCon);
            }
            break;

        // process next command layer frame
        // Note: content is only valid for WriteByIndex due to sequence layer
        //       forwarding mechanism
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
                    // higher layer will not be informed
                    return ret;
                }

                if (pSdoComCon->sdoServiceType == kSdoServiceWriteByIndex)
                {
                    ret = serverProcessResponseWriteByIndex(pSdoComCon, pRecvdCmdLayer_p);
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

//------------------------------------------------------------------------------
/**
\brief  Object dictionary finishes an SDO server access

The function completes an SDO server access and will be called by the object
dictionary.

\parblock
\param  pObdHdl_p    Connection handle to SDO command layer. Used members:
        \li \ref tSdoObdConHdl::sdoHdl
        \li \ref tSdoObdConHdl::plkError

        Only used for read access (status saved in local instance):
        \li tSdoObdConHdl::pSrcData
        \li tSdoObdConHdl::dataSize      Data size to be copied

        Only used for initial read or write access
        (status saved in local instance):
        \li tSdoObdConHdl::totalPendSize object size
\endparblock

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError serverObdFinishCb(tSdoObdConHdl* pObdHdl_p)
{
    tOplkError      ret = kErrorOk;
    tSdoComCon*     pSdoComCon = NULL;

    ret = serverObdSearchConnection(pObdHdl_p->sdoHdl, &pSdoComCon);
    if (ret != kErrorOk)
    {
        // overwrite potentially existing error
        pObdHdl_p->plkError = ret;
        if (pSdoComCon == NULL)
        {   // no handle found, exit immediately
            goto Exit;
        }
    }

    if (pObdHdl_p->plkError != kErrorOk)
    {
        assignSdoErrorCode(pObdHdl_p->plkError, &pSdoComCon->lastAbortCode);
        ret = serverAbortTransfer(pSdoComCon, pSdoComCon->lastAbortCode);
        ret = sdoseq_deleteCon(pSdoComCon->sdoSeqConHdl);
        OPLK_MEMSET(pSdoComCon, 0x00, sizeof(tSdoComCon));
        goto Exit;
    }

    switch (pSdoComCon->sdoObdAccType)
    {
        case kSdoComConObdInitReadByIndex:
            if (pObdHdl_p->pSrcData == NULL)
            {
                ret = serverAbortTransfer(pSdoComCon, SDO_AC_DATA_NOT_TRANSF_OR_STORED);
                goto Exit;
            }
            // start of command layer data (in little endian format)
            // to be copied by SDO server
            pSdoComCon->pData = pObdHdl_p->pSrcData;
            // save service - init read by index
            pSdoComCon->transferSize = pObdHdl_p->totalPendSize;
            ret = serverFinishInitReadByIndex(pSdoComCon, NULL, pObdHdl_p->dataSize);
            break;

        case kSdoComConObdReadByIndex:
            if (pObdHdl_p->pSrcData == NULL)
            {
                ret = serverAbortTransfer(pSdoComCon, SDO_AC_DATA_NOT_TRANSF_OR_STORED);
                goto Exit;
            }
            // start of command layer data to be copied by SDO server
            pSdoComCon->pData = pObdHdl_p->pSrcData;
            ret = serverFinishReadByIndex(pSdoComCon, NULL, pObdHdl_p->dataSize);
            break;

        case kSdoComConObdInitWriteByIndex:
            // save service - init write by index
            pSdoComCon->transferSize = pObdHdl_p->totalPendSize;
            ret = serverFinishInitWriteByIndex(pSdoComCon);
            break;

        case kSdoComConObdWriteByIndex:
            ret = serverFinishWriteByIndex(pSdoComCon);
            break;

        default:
            ret = kErrorSdoComInvalidHandle;
            break;
    }

Exit:
    return ret;
}
#endif // defined(CONFIG_INCLUDE_SDOS)

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
static tOplkError clientSdoDefineConnection(tSdoComConHdl* pSdoComConHdl_p,
                                            UINT targetNodeId_p,
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
static tOplkError clientSdoInitTransferByIndex(tSdoComTransParamByIndex* pSdoComTransParam_p)
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
static tOplkError clientSdoUndefineConnection(tSdoComConHdl sdoComConHdl_p)
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
static tOplkError clientSdoGetState(tSdoComConHdl sdoComConHdl_p, tSdoComFinished* pSdoComFinished_p)
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
static UINT clientSdoGetNodeId(tSdoComConHdl sdoComConHdl_p)
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
static tOplkError clientSdoAbortTransfer(tSdoComConHdl sdoComConHdl_p, UINT32 abortCode_p)
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

//------------------------------------------------------------------------------
/**
\brief  Process state clientProcessStateWaitInit

The function processes the SDO command handler state: clientProcessStateWaitInit

\param  sdoComConHdl_p          Handle to command layer connection.
\param  sdoComConEvent_p        Event to process.
\param  pRecvdCmdLayer_p        SDO command layer part of received frame.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError clientProcessStateWaitInit(tSdoComConHdl sdoComConHdl_p, tSdoComConEvent sdoComConEvent_p,
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
            ret = clientTransferFinished(sdoComConHdl_p, pSdoComCon, kSdoComTransferTxAborted);
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

\param  sdoComConHdl_p          Handle to command layer connection.
\param  sdoComConEvent_p        Event to process.
\param  pRecvdCmdLayer_p        SDO command layer part of received frame.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError clientProcessStateConnected(tSdoComConHdl sdoComConHdl_p, tSdoComConEvent sdoComConEvent_p,
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
                    ret = sdoseq_sendData(pSdoComCon->sdoSeqConHdl, 0, (tPlkFrame*)NULL);
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
                        ret = sdoseq_sendData(pSdoComCon->sdoSeqConHdl, 0, (tPlkFrame*)NULL);
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
            ret = sdoseq_deleteCon(pSdoComCon->sdoSeqConHdl);         // close sequence layer handle
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
            ret = sdoseq_deleteCon(pSdoComCon->sdoSeqConHdl);         // close sequence layer handle
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

\param  sdoComConHdl_p          Handle to command layer connection.
\param  sdoComConEvent_p        Event to process.
\param  pRecvdCmdLayer_p        SDO command layer part of received frame.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError clientProcessStateSegmTransfer(tSdoComConHdl sdoComConHdl_p, tSdoComConEvent sdoComConEvent_p,
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
                    ret = sdoseq_sendData(pSdoComCon->sdoSeqConHdl, 0, (tPlkFrame*)NULL);
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
                        ret = sdoseq_sendData(pSdoComCon->sdoSeqConHdl, 0, (tPlkFrame*)NULL);
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
            ret = sdoseq_deleteCon(pSdoComCon->sdoSeqConHdl);         // close sequence layer handle
            pSdoComCon->sdoSeqConHdl |= SDO_SEQ_INVALID_HDL;
            pSdoComCon->sdoComState = kSdoComStateClientWaitInit;
            pSdoComCon->transactionId++;
            pSdoComCon->lastAbortCode = 0;
            ret = clientTransferFinished(sdoComConHdl_p, pSdoComCon, kSdoComTransferFinished);
            break;

        // abort to send from higher layer
        case kSdoComConEventAbort:
            clientSendAbort(pSdoComCon, *((UINT32*)pSdoComCon->pData));
            pSdoComCon->transactionId++;
            pSdoComCon->sdoComState = kSdoComStateClientConnected;
            pSdoComCon->lastAbortCode = *((UINT32*)pSdoComCon->pData);
            ret = clientTransferFinished(sdoComConHdl_p, pSdoComCon, kSdoComTransferTxAborted);
            break;

        case kSdoComConEventInitError:
        case kSdoComConEventTimeout:
            ret = sdoseq_deleteCon(pSdoComCon->sdoSeqConHdl);         // close sequence layer handle
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
\brief  Send an SDO command layer frame from a SDO client

The function starts an SDO transfer and sends all further frames..

\param  pSdoComCon_p            Pointer to SDO command layer connection structure.

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
                    pSdoComCon_p->transferredBytes = 1;
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
                    {
                        return ret;
                    }
                    break;

                // for expedited read is nothing to do -> server sends data
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
                                pSdoComCon->transferSize = (UINT)transferSize;
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

\param  pSdoComCon_p            Pointer to SDO command layer connection structure.
\param  abortCode_p             Abort code to send.

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
#endif // ifdef CONFIG_INCLUDE_SDOC

/// \}
