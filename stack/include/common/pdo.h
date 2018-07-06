/**
********************************************************************************
\file   common/pdo.h

\brief  Include file for PDO module

*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2012, SYSTEC electronic GmbH
Copyright (c) 2017, B&R Industrial Automation GmbH
Copyright (c) 2018, Kalycito Infotech Private Limited
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
#ifndef _INC_common_pdo_H_
#define _INC_common_pdo_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <common/oplkinc.h>

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#define PDO_SHMEM_NAME                  "/pdoShm"

#define PDO_INVALID_NODE_ID             0xFF    // invalid PDO-NodeId
#define PDO_PREQ_NODE_ID                0x00    // NodeId for PReq RPDO
#define PDO_PRES_NODE_ID                0x00    // NodeId for PRes TPDO

//------------------------------------------------------------------------------
// typedef
//------------------------------------------------------------------------------

/**
\brief PDO allocation parameter structure

This structure specifies a PDO allocation parameter. It saves information about
the number of used PDO channels.
*/
typedef struct
{
    UINT                rxPdoChannelCount;      ///< max. number of RPDO channels
    UINT                txPdoChannelCount;      ///< max. number of TPDO channels
} tPdoAllocationParam;

/**
\brief PDO channel

This structure specifies a PDO channel. The PDO channel contains all
information needed to transfer the PDO on the network.
*/
typedef struct
{
    /** The node ID for this PDO
    0xFF = invalid; RPDO: 0x00=PReq, localNodeId=PRes, remoteNodeId=PRes;
    TPDO: 0x00=PRes, MN: CnNodeId=PReq
    */
    UINT                nodeId;
    UINT16              offset;                 ///< Offset of PDO channel (first mapped object) in bytes
    UINT16              nextChannelOffset;      ///< Offset of the following PDO channel
    UINT8               mappingVersion;         ///< The mapping version of this PDO
    UINT32              mappObjectCount;        ///< The actual number of used mapped objects
} tPdoChannel;

/**
\brief PDO channel configuration

This structure specifies a PDO channel configuration. It is used to exchange
PDO channel information between the user and the kernel layer.
*/
typedef struct
{
    UINT8               channelId;              ///< ID of the PDO channel
    UINT8               fTx;                    ///< Flag determines the direction. TRUE = TPDO, FALSE = RPDO
    UINT8               aPadding[2];            ///< Padding for 32 bit alignment
    tPdoChannel         pdoChannel;             ///< The PDO channel itself
} tPdoChannelConf;

/**
\brief PDO channel setup

This structure specifies a PDO channel setup. It is the basic structure used
to manage the complete setup of the PDO channels.
*/
typedef struct
{
    tPdoAllocationParam allocation;             ///< Number of used PDO channels
    tPdoChannel*        pRxPdoChannel;          ///< Pointer to RXPDO channel table
    tPdoChannel*        pTxPdoChannel;          ///< Pointer to TXPDO channel table
} tPdoChannelSetup;

/**
\brief PDO buffer information

This structure specifies a PDO channel buffer. Each PDO channel has got an
offset in the buffers, and specifies the currently used buffer for consuming
data, producing data and a clean buffer.
*/
typedef struct
{
    UINT32              channelOffset;          ///< Offset of the channel in the buffers
    OPLK_ATOMIC_T       readBuf;                ///< Current buffer to consume data from
    OPLK_ATOMIC_T       writeBuf;               ///< Current buffer to produce data to
    OPLK_ATOMIC_T       cleanBuf;               ///< Current clean (i.e. unused) buffer
    UINT8               newData;                ///< Flag indicating whether new data has been produced
} tPdoBufferInfo;

/**
\brief PDO memory region

This structure specifies a PDO memory region. It consists of arrays
of receive and transmit PDO buffers.
*/
typedef struct
{
    UINT16              valid;                                      ///< Defines whether the memory region is valid
    UINT32              pdoMemSize;                                 ///< Size of the overall PDO memory
    tPdoBufferInfo      rxChannelInfo[D_PDO_RPDOChannels_U16];      ///< Array of RPDO channels
    tPdoBufferInfo      txChannelInfo[D_PDO_TPDOChannels_U16];      ///< Array of TPDO channels
    OPLK_LOCK_T         lock;                                       ///< Locking variable
} tPdoMemRegion;

/**
\brief PDO memory size

This structure specifies the sizes of the RPDO and TPDO memory.
*/
typedef struct
{
    UINT32      rxPdoMemSize;                   ///< Size of the RPDO memory
    UINT32      txPdoMemSize;                   ///< Size of the TPDO memory
} tPdoMemSize;

#endif /* _INC_common_pdo_H_ */
