/**
********************************************************************************
\file   user/sdocomint.h

\brief  Definitions for internal SDO Command Layer functions

The file contains internal definitions for the SDO Command Layer module.
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

#ifndef _INC_user_sdocomint_H_
#define _INC_user_sdocomint_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <user/sdocom.h>
#include <user/sdoseq.h>

#if (!defined(CONFIG_INCLUDE_SDOS) && !defined(CONFIG_INCLUDE_SDOC))
#error "ERROR: At least SDO Server or SDO Client should be activated!"
#endif

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
#define SDO_CMD_SEGM_TX_MAX_SIZE                (SDO_MAX_TX_FRAME_SIZE - 58)
#define SDO_CMD_TX_MAX_SIZE                     (SDO_CMD_SEGM_TX_MAX_SIZE + 8)
#define SDO_CMD_INIT_READ_SEMG_TX_MAX_SIZE      (SDO_CMD_SEGM_TX_MAX_SIZE - SDO_CMDL_HDR_VAR_SIZE)

#if (SDO_CMD_SEGM_TX_MAX_SIZE < 256)
#error "SDO command layer segment size to low (limit 256 bytes)!"
#elif (SDO_CMD_SEGM_TX_MAX_SIZE > 1456)
#error "SDO command layer segment size to high (limit 1456 bytes)!"
#endif

#ifndef CONFIG_SDO_MAX_CONNECTION_COM
#define CONFIG_SDO_MAX_CONNECTION_COM           5
#endif

//------------------------------------------------------------------------------
// typedef
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
    tSdoServiceType     sdoServiceType;      ///< Service Type: WriteByIndex, ReadByIndex, WriteMultParam, ReadMultParam
    tSdoType            sdoProtocolType;     ///< Protocol Type: Auto, Udp, ASnd
    void*               pData;               ///< Pointer to data
    void*               pDataStart;          ///< Pointer start of data for segmented transfer
    size_t              transferSize;        ///< Number of bytes to transfer
    size_t              transferredBytes;    ///< Number of bytes already transferred
    size_t              pendingTxBytes;      ///< Due bytes waiting for transmission
    size_t              reqSegmSize;         ///< Segment size of WriteMultParam or ReadMultParam request for server or max. buffer size for client
    size_t              respSegmSize;        ///< Segment size of WriteMultParam or ReadMultParam response
    tSdoMultiAccEntry*  paMultiAcc;          ///< Pointer to multi access array provided by user
    UINT                multiAccCnt;         ///< Count of processed multi access array elements
#if defined(CONFIG_INCLUDE_SDOS)
    tSdoComConHdl       sdoObdConHdl;        ///< OD connection handle (only valid if not 0)
    size_t              pendingTransferSize; ///< Due bytes waiting for confirmation from pending OD access
                                             /**< WriteByIndex: Due bytes waiting for confirmation from pending
                                                                OD write access
                                                  ReadByIndex: Max. Tx buffer size for initial OD read access */
    tSdoObdAccType      sdoObdAccType;       ///< Used for processing decision after the OD access has finished
#endif
    tSdoFinishedCb      pfnTransferFinished; ///< Callback function to be called in the end of the SDO transfer
    void*               pUserArg;            ///< User definable argument pointer
    UINT32              lastAbortCode;       ///< Last abort code
    UINT16              targetIndex;         ///< Object index to access
    UINT8               targetSubIndex;      ///< Object subindex to access
} tSdoComCon;

/**
\brief  SDO command layer instance structure

This structure describes a SDO command layer instance
*/
typedef struct
{
    tSdoComCon          sdoComCon[CONFIG_SDO_MAX_CONNECTION_COM];   ///< Array to store command layer connections
#if defined(CONFIG_INCLUDE_SDOS)
    tSdoComConHdl       sdoObdConCounter;                           ///< OD connection handle counter for object accesses
    tComdLayerObdCb     pfnProcessObdWrite;                         ///< OD callback function for WriteByIndex processing
    tComdLayerObdCb     pfnProcessObdRead;                          ///< OD callback function for ReadByIndex processing
#endif
#if (defined(WIN32) || defined(_WIN32))
    LPCRITICAL_SECTION  pCriticalSection;
    CRITICAL_SECTION    criticalSection;
#endif
} tSdoComInstance;

//------------------------------------------------------------------------------
// global variable declarations
//------------------------------------------------------------------------------
extern tSdoComInstance sdoComInstance_g;

//------------------------------------------------------------------------------
// function prototypes
//------------------------------------------------------------------------------
tOplkError sdocomint_receiveCb(tSdoSeqConHdl sdoSeqConHdl_p,
                               const tAsySdoCom* pSdoCom_p,
                               size_t dataSize_p);
tOplkError sdocomint_conStateChangeCb(tSdoSeqConHdl sdoSeqConHdl_p,
                                      tAsySdoConState sdoConnectionState_p);
tOplkError sdocomint_processCmdLayerConnection(tSdoSeqConHdl sdoSeqConHdl_p,
                                               tSdoComConEvent sdoComConEvent_p,
                                               const tAsySdoCom* pSdoCom_p);
tOplkError sdocomint_processState(tSdoComConHdl sdoComConHdl_p,
                                  tSdoComConEvent sdoComConEvent_p,
                                  const tAsySdoCom* pSdoCom_p);
void       sdocomint_initCmdFrameGeneric(tPlkFrame* pPlkFrame_p,
                                         size_t plkFrameSize_p,
                                         const tSdoComCon* pSdoComCon_p,
                                         tAsySdoCom** pCommandFrame_p);
void       sdocomint_setCmdFrameHdrFlag(tAsySdoCom* pCommandFrame_p,
                                        UINT8 flag_p);
void       sdocomint_overwriteCmdFrameHdrFlags(tAsySdoCom* pCommandFrame_p,
                                               UINT8 flag_p);
void       sdocomint_setCmdFrameHdrSegmSize(tAsySdoCom* pCommandFrame_p,
                                            size_t size_p);
void       sdocomint_fillCmdFrameDataSegm(tAsySdoCom* pCommandFrame_p,
                                          const void* pSrcData_p,
                                          size_t size_p);
void       sdocomint_updateHdlTransfSize(tSdoComCon* pSdoComCon_p,
                                         size_t tranferredBytes_p,
                                         BOOL fTransferComplete);
#endif /* _INC_user_sdocomint_H_ */
