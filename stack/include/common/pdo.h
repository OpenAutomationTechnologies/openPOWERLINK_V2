/**
********************************************************************************
\file   pdo.h

\brief  Include file for PDO module

*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2012, SYSTEC electronic GmbH
Copyright (c) 2012, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
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

#ifndef _INC_pdo_H_
#define _INC_pdo_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <oplk/oplkinc.h>

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#define PDO_SHB_BUF_ID                  "PdoMem"
#define PDO_SYNC_BSDSEM                 "/semPdoSync"
#define PDO_SHMEM_NAME                  "/podShm"

#define PDO_MAX_ALLOC_SIZE      239 * 2 * 1500      //jba replace with a clean solution

// PDO mapping related OD defines
#define PDOU_OBD_IDX_RX_COMM_PARAM      0x1400
#define PDOU_OBD_IDX_RX_MAPP_PARAM      0x1600
#define PDOU_OBD_IDX_TX_COMM_PARAM      0x1800
#define PDOU_OBD_IDX_TX_MAPP_PARAM      0x1A00
#define PDOU_OBD_IDX_MAPP_PARAM         0x0200
#define PDOU_OBD_IDX_MASK               0xFF00
#define PDOU_PDO_ID_MASK                0x00FF

#define PDOU_MAX_PDO_OBJECTS            256
#define PDO_MAX_PDO_CHANNELS            256


// invalid PDO-NodeId
#define PDO_INVALID_NODE_ID             0xFF
// NodeId for PReq RPDO
#define PDO_PREQ_NODE_ID                0x00
// NodeId for PRes TPDO
#define PDO_PRES_NODE_ID                0x00

#define PDO_COMMUNICATION_PROFILE_START 0x1000

#define PDO_MAPPOBJECT_IS_NUMERIC(pPdoMappObject_p) \
            (pPdoMappObject_p->byteSizeOrType < PDO_COMMUNICATION_PROFILE_START)

#define PDO_MAPPOBJECT_GET_VAR(pPdoMappObject_p) \
            pPdoMappObject_p->pVar

#define PDO_MAPPOBJECT_SET_VAR(pPdoMappObject_p, pVar_p) \
            (pPdoMappObject_p->pVar = pVar_p)

#define PDO_MAPPOBJECT_GET_BITOFFSET(pPdoMappObject_p) \
            pPdoMappObject_p->bitOffset

#define PDO_MAPPOBJECT_SET_BITOFFSET(pPdoMappObject_p, wBitOffset_p) \
            (pPdoMappObject_p->bitOffset = wBitOffset_p)

#define PDO_MAPPOBJECT_GET_BYTESIZE(pPdoMappObject_p) \
            (pPdoMappObject_p->byteSizeOrType - PDO_COMMUNICATION_PROFILE_START)

#define PDO_MAPPOBJECT_GET_TYPE(pPdoMappObject_p) \
            ((tObdType) pPdoMappObject_p->byteSizeOrType)

#define PDO_MAPPOBJECT_SET_BYTESIZE_OR_TYPE(pPdoMappObject_p, wByteSize_p, ObdType_p) \
            if ((ObdType_p == kObdTypeVString) || (ObdType_p == kObdTypeOString) || (ObdType_p == kObdTypeDomain)) \
            { \
                pPdoMappObject_p->byteSizeOrType = wByteSize_p + PDO_COMMUNICATION_PROFILE_START; \
            } \
            else \
            { \
                pPdoMappObject_p->byteSizeOrType = ObdType_p; \
            }

//------------------------------------------------------------------------------
// typedef
//------------------------------------------------------------------------------

/**
\brief PDO allocation param structure

This structure specifies a PDO allocation parameter. It saves information about
the number of used PDO channels.
*/
typedef struct
{
    UINT                rxPdoChannelCount;      ///< max. number of RPDO channels
    UINT                txPdoChannelCount;      ///< max. number of TPDO channels
} tPdoAllocationParam;

/**
\brief PDO mapping object

This structure structure specifies a PDO mapping object.
*/
typedef struct
{
    void*               pVar;                   ///< Pointer to PDO data
    UINT16              bitOffset;              ///< Frame offset in bits
    UINT16              byteSizeOrType;         ///< The size of the data in bytes
} tPdoMappObject;

/**
\brief PDO channel

This structure structure specifies a PDO channel. The PDO channel contains all
information needed to transfer the PDO on the network.
*/
typedef struct
{
    /** The node ID for this PDO
    0xFF = invalid; RPDO: 0x00=PReq, localNodeId=PRes, remoteNodeId=PRes;
    TPDO: 0x00=PRes, MN: CnNodeId=PReq
    */
    UINT                nodeId;
    void *              pVar;                   ///< Pointer to frame data
    WORD                pdoSize;                ///< Size of this PDO
    BYTE                mappingVersion;         ///< The mapping version of this PDO
    unsigned int        mappObjectCount;        ///< The actual number of used mapped objects
} tPdoChannel;

/**
\brief PDO channel configuration

This structure structure specifies a PDO channel configuration. It is used to
exchange PDO channel information between the user and the kernel layer.
*/
typedef struct
{
    UINT                channelId;              ///< ID of the PDO channel
    BOOL                fTx;                    ///< Flag determines the direction. TRUE = TPDO, FALSE = RPDO
    tPdoChannel         pdoChannel;             ///< The PDO channel itself
} tPdoChannelConf;

/**
\brief PDO channel setup

This structure structure specifies a PDO channel setup. It is basic structure
used to manage the complete setup of PDO channels.
*/
typedef struct
{
    tPdoAllocationParam allocation;             ///< Number of used PDO channels
    tPdoChannel*        pRxPdoChannel;          ///< Pointer to RXPDO channel table
    tPdoChannel*        pTxPdoChannel;          ///< Pointer to TXPDO channel table
} tPdoChannelSetup;


typedef struct
{
    ULONG           channelOffset;
    OPLK_ATOMIC_T   readBuf;
    OPLK_ATOMIC_T   writeBuf;
    OPLK_ATOMIC_T   cleanBuf;
    UINT8           newData;
} tPdoBufferInfo;

typedef struct
{
    UINT16              valid;
    size_t              pdoMemSize;
    tPdoBufferInfo      rxChannelInfo[EPL_D_PDO_RPDOChannels_U16];
    tPdoBufferInfo      txChannelInfo[EPL_D_PDO_TPDOChannels_U16];
#ifdef OPLK_LOCK_T
    OPLK_LOCK_T         lock;
#endif
} tPdoMemRegion;


typedef struct
{
    size_t      rxPdoMemSize;
    size_t      txPdoMemSize;
} tPdoMemSize;

//------------------------------------------------------------------------------
// function prototypes
//------------------------------------------------------------------------------

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
}
#endif

#endif /* _INC_pdo_H_ */


