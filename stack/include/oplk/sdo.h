/**
********************************************************************************
\file   sdo.h

\brief  Definitions for SDO module

This file contains definitions for the SDO module.
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2013, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
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

#ifndef _INC_sdo_H_
#define _INC_sdo_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <oplk/oplkinc.h>
#include <oplk/frame.h>
#include <oplk/sdoabortcodes.h>

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#ifndef SDO_MAX_SEGMENT_SIZE
#define SDO_MAX_SEGMENT_SIZE        256
#endif

// handle between Protocol Abstraction Layer and asynchronous SDO Sequence Layer
#define SDO_UDP_HANDLE              0x8000
#define SDO_ASND_HANDLE             0x4000
#define SDO_ASY_HANDLE_MASK         0xC000
#define SDO_ASY_INVALID_HDL         0x3FFF

// handle between SDO Sequence Layer and SDO command layer
#define SDO_ASY_HANDLE              0x8000
#define SDO_PDO_HANDLE              0x4000
#define SDO_SEQ_HANDLE_MASK         0xC000
#define SDO_SEQ_INVALID_HDL         0x3FFF

#define ASND_HEADER_SIZE            4

#define SEQ_NUM_MASK                0xFC

// size for send buffer and history
#define SDO_MAX_FRAME_SIZE          EPL_C_IP_MIN_MTU
// size for receive frame
// -> needed because SND-Kit sends up to 1518 Byte
//    without Sdo-Command: Maximum Segment Size
#define SDO_MAX_REC_FRAME_SIZE      EPL_C_IP_MAX_MTU

//------------------------------------------------------------------------------
// Type definitions
//------------------------------------------------------------------------------

/// Data type for handle between Protocol Abstraction Layer and asynchronous SDO Sequence Layer
typedef UINT tSdoConHdl;

/// Callback function pointer for Protocol Abstraction Layer to call asynchronous SDO Sequence Layer
typedef tOplkError (*tSequLayerReceiveCb)(tSdoConHdl ConHdl_p, tAsySdoSeq* pSdoSeqData_p, UINT uiDataSize_p);

/// Data type for handle between asynchronous SDO Sequence Layer and SDO Command layer
typedef UINT tSdoSeqConHdl;

/// Callback function pointer for asynchronous SDO Sequence Layer to call SDO Command layer for received data
typedef tOplkError (*tSdoComReceiveCb)(tSdoSeqConHdl SdoSeqConHdl_p, tAsySdoCom* pAsySdoCom_p, UINT uiDataSize_p);

/**
\brief Enumeration lists valid SDO connection states

This enumeration lists all valid SDO connection states.
*/
typedef enum
{
    kAsySdoConStateConnected            = 0x00,
    kAsySdoConStateInitError            = 0x01,
    kAsySdoConStateConClosed            = 0x02,
    kAsySdoConStateAckReceived          = 0x03,
    kAsySdoConStateFrameSended          = 0x04,
    kAsySdoConStateTimeout              = 0x05,
    kAsySdoConStateTransferAbort        = 0x06,
}tAsySdoConState;

/// callback function pointer for asynchronous SDO sequence layer to call SDO command layer for connection status
typedef tOplkError (*tSdoComConCb)(tSdoSeqConHdl SdoSeqConHdl_p, tAsySdoConState AsySdoConState_p);

/// Data type for handle between SDO command layer and application
typedef UINT tSdoComConHdl;

/**
\brief Enumeration lists valid SDO command layer connection states

This enumeration lists all valid SDO command layer connection states.
*/
typedef enum
{
    kEplSdoComTransferNotActive         = 0x00,
    kEplSdoComTransferRunning           = 0x01,
    kEplSdoComTransferTxAborted         = 0x02,
    kEplSdoComTransferRxAborted         = 0x03,
    kEplSdoComTransferFinished          = 0x04,
    kEplSdoComTransferLowerLayerAbort   = 0x05
} tSdoComConState;

/**
\brief Enumeration for SDO service types (command IDs)

This enumeration lists all valid SDO command IDs.
*/
typedef enum
{
    kSdoServiceNIL                      = 0x00,
    kSdoServiceWriteByIndex             = 0x01,
    kSdoServiceReadByIndex              = 0x02,

    // the following services are optional and are not supported now
    kSdoServiceWriteAllByIndex          = 0x03,
    kSdoServiceReadAllByIndex           = 0x04,
    kSdoServiceWriteByName              = 0x05,
    kSdoServiceReadByName               = 0x06,
    kSdoServiceFileWrite                = 0x20,
    kSdoServiceFileRead                 = 0x21,
    kSdoServiceWriteMultiByIndex        = 0x31,
    kSdoServiceReadMultiByIndex         = 0x32,
    kSdoServiceMaxSegSize               = 0x70
    // 0x80 - 0xFF manufacturer specific
} tSdoServiceType;


/**
\brief Enumeration for SDO access types

This enumeration lists all valid SDO access types.
*/
typedef enum
{
    kSdoAccessTypeRead                  = 0x00,
    kSdoAccessTypeWrite                 = 0x01
} tSdoAccessType;

/**
\brief Enumeration for SDO types

This enumeration lists all valid SDO types.
*/
typedef enum
{
    kSdoTypeAuto                        = 0x00,
    kSdoTypeUdp                         = 0x01,
    kSdoTypeAsnd                        = 0x02,
    kSdoTypePdo                         = 0x03
}tSdoType;

/**
\brief Enumeration for SDO transfer types

This enumeration lists all valid SDO transfer types.
*/
typedef enum
{
    kSdoTransAuto                       = 0x00,
    kSdoTransExpedited                  = 0x01,
    kSdoTransSegmented                  = 0x02
} tSdoTransType;

/**
\brief Structure for finished SDO transfer

This structure is used to inform the application about a finished SDO transfer.
*/
typedef struct
{
    tSdoComConHdl       sdoComConHdl;           ///< Handle to SDO command layer connection
    tSdoComConState     sdoComConState;         ///< Status of SDO command layer connection
    UINT32              abortCode;              ///< SDO abort code
    tSdoAccessType      sdoAccessType;          ///< SDO access type
    UINT                nodeId;                 ///< The node ID of the target
    UINT                targetIndex;            ///< Index which was accessed
    UINT                targetSubIndex;         ///< Sub-index which was accessed
    UINT                transferredBytes;       ///< The number of bytes transferred
    void*               pUserArg;               ///< The user defined argument pointer
} tSdoComFinished;


/// callback function pointer to inform application about connection
typedef tOplkError (*tSdoFinishedCb)(tSdoComFinished* pSdoComFinished_p);

/**
\brief Structure for initializing Read/Write by Index SDO transfer

This structure is used to initialize a SDO transfer of a Read or Write
by Index command.
*/
typedef struct
{
    tSdoComConHdl       sdoComConHdl;           ///< Handle to SDO command layer connection
    UINT                index;                  ///< Index to read/write
    UINT                subindex;               ///< Sub-index to read/write
    void*               pData;                  ///< Pointer to data which should be transfered
    UINT                dataSize;               ///< Size of data to be transfered
    UINT                timeout;                ///< Timeout: not supported in this version of openPOWERLINK
    tSdoAccessType      sdoAccessType;          ///< The SDO access type (Read or Write) for this transfer
    tSdoFinishedCb      pfnSdoFinishedCb;       ///< Pointer to callback function which will be called when transfer is finished.
    void*               pUserArg;               ///< User definable argument pointer
} tSdoComTransParamByIndex;

//------------------------------------------------------------------------------
// function prototypes
//------------------------------------------------------------------------------

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
}
#endif

#endif /* _INC_sdo_H_ */

