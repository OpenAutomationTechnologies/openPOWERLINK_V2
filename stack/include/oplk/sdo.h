/**
********************************************************************************
\file   oplk/sdo.h

\brief  Definitions for SDO module

This file contains definitions for the SDO module.
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

#ifndef _INC_oplk_sdo_H_
#define _INC_oplk_sdo_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <oplk/oplkinc.h>
#include <oplk/sdoabortcodes.h>

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

// size for send buffer and history
#define SDO_MAX_FRAME_SIZE          C_IP_MIN_MTU
// size for receive frame
// -> needed because SND-Kit sends up to 1518 Byte
//    without Sdo-Command: Maximum Segment Size
#define SDO_MAX_REC_FRAME_SIZE      C_IP_MAX_MTU

//------------------------------------------------------------------------------
// Type definitions
//------------------------------------------------------------------------------
/// Data type for handle between SDO Command Layer and application
typedef UINT tSdoComConHdl;

/**
\brief Enumeration for SDO types

This enumeration lists all valid SDO types.
*/
typedef enum
{
    kSdoTypeAuto                        = 0x00,     ///< SDO connection type is automatically selected
    kSdoTypeUdp                         = 0x01,     ///< Use SDO via UDP
    kSdoTypeAsnd                        = 0x02,     ///< Use SDO via ASnd
    kSdoTypePdo                         = 0x03      ///< Use SDO via PDO
} eSdoType;

/**
\brief SDO type data type

Data type for the enumerator \ref eSdoType.
*/
typedef UINT32 tSdoType;

/**
\brief Enumeration lists valid SDO Command Layer connection states

This enumeration lists all valid SDO Command Layer connection states.
*/
typedef enum
{
    kSdoComTransferNotActive            = 0x00,     ///< SDO transfer is not active
    kSdoComTransferRunning              = 0x01,     ///< SDO transfer is currently running
    kSdoComTransferTxAborted            = 0x02,     ///< SDO transfer is aborted (abort code is going to be sent)
    kSdoComTransferRxAborted            = 0x03,     ///< SDO transfer has been aborted by the remote side
    kSdoComTransferFinished             = 0x04,     ///< SDO transfer is finished
    kSdoComTransferLowerLayerAbort      = 0x05      ///< SDO transfer has been aborted by the SDO sequence layer
} eSdoComConState;

/**
\brief SDO command layer connection state data type

Data type for the enumerator \ref eSdoComConState.
*/
typedef UINT32 tSdoComConState;

/**
\brief Enumeration for SDO access types

This enumeration lists all valid SDO access types.
*/
typedef enum
{
    kSdoAccessTypeRead                  = 0x00,     ///< SDO read access
    kSdoAccessTypeWrite                 = 0x01      ///< SDO write access
} eSdoAccessType;

/**
\brief SDO access data type

Data type for the enumerator \ref eSdoAccessType.
*/
typedef UINT32 tSdoAccessType;

/**
\brief Structure for finished SDO transfer

This structure is used to inform the application about a finished SDO transfer.
*/
typedef struct
{
    tSdoComConHdl       sdoComConHdl;               ///< Handle to SDO Command Layer connection
    tSdoComConState     sdoComConState;             ///< Status of SDO Command Layer connection
    UINT32              abortCode;                  ///< SDO abort code
    tSdoAccessType      sdoAccessType;              ///< SDO access type
    UINT                nodeId;                     ///< The node ID of the target
    UINT                targetIndex;                ///< Index which was accessed
    UINT                targetSubIndex;             ///< Sub-index which was accessed
    UINT                transferredBytes;           ///< The number of bytes transferred
    void*               pUserArg;                   ///< The user defined argument pointer
} tSdoComFinished;

#endif /* _INC_oplk_sdo_H_ */
