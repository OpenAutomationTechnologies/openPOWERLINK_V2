/**
********************************************************************************
\file   oplk/sdo.h

\brief  Definitions for SDO module

This file contains definitions for the SDO module.
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

// size for complete SDO send frame (without CRC)
// valid range: [C_DLL_MIN_ASYNC_MTU .. C_DLL_MAX_ASYNC_MTU] + Eth. header
#define SDO_MAX_TX_FRAME_SIZE           314


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
    kSdoComTransferLowerLayerAbort      = 0x05,     ///< SDO transfer has been aborted by the SDO sequence layer
    kSdoComTransferRxSubAborted         = 0x06      ///< SDO transfer has been aborted by the remote side with multiple sub-abort codes
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
    kSdoAccessTypeWrite                 = 0x01,     ///< SDO write access
    kSdoAccessTypeMultiWrite            = 0x02,     ///< SDO multi-write access
    kSdoAccessTypeMultiRead             = 0x03      ///< SDO multi-read access
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
    size_t              transferredBytes;           ///< The number of bytes transferred
    UINT                multiSubAccCnt;             ///< Number of multi-write objects access attempts which fit in this transfer
    void*               pUserArg;                   ///< The user defined argument pointer
} tSdoComFinished;


/**
\brief Structure for SDO command layer connection to object dictionary

This structure is used to transfer data from the SDO command layer to the
object dictionary and vice versa (SDO Write-/ReadbyIndex). It is only applied
for the SDO server, which requires a connection to an object dictionary.
*/
typedef struct
{
    UINT                index;                      ///< Index to read/write
    UINT                subIndex;                   ///< Sub-index to read/write
    void*               pSrcData;                   ///< Pointer to data which should be transferred
    void*               pDstData;                   ///< Pointer to storage destination
    size_t              totalPendSize;              ///< Total pending SDO command layer transfer size
    size_t              dataSize;                   ///< SDO command layer segment size (payload)
    size_t              dataOffset;                 ///< Payload offset of SDO command layer data
    tSdoComConHdl       sdoHdl;                     ///< Handle to SDO command layer connection, generated by SDO server
    tOplkError          plkError;                   ///< Error signaling between SDO and object dictionary.
} tSdoObdConHdl;

/**
\brief Structure for multiple parameter access

This structure is used as sub-entry of a multiple parameter access the local or
a remote node (via SDO MultiWrite-/MultiReadbyIndex). It is only used for the
SDO client.
*/
typedef struct
{
    UINT                index;                      ///< Index to read/write
    UINT                subIndex;                   ///< Sub-index to read/write
    void*               pData_le;                   ///< Pointer to little endian data.
                                                    /**> Multi-write: pointer to source data
                                                         Multi-read:  pointer to storage destination */
    size_t              dataSize;                   ///< Size of buffer for read- and size of data for write access
} tSdoMultiAccEntry;

/**
\brief Callback for object dictionary to finish an SDO read or write access

This callback function is used for the object dictionary to finish a read or
write access from an SDO command layer server.

\param pObdHdl_p                SDO command layer connection to the object dictionary

\return The function returns a tOplkError error code.
*/
typedef tOplkError (*tCmdLayerObdFinishedCb)(tSdoObdConHdl* pObdHdl_p);

/**
\brief Callback for SDO read or write access to the object dictionary

This callback function is used for the SDO command layer server to process
an object dictionary access.

\param pSdoHdl_p                SDO command layer connection to the
                                object dictionary
\param pfnFinishSdoCb_p         Callback for object dictionary to finish
                                SDO read or write access

\return The function returns a tOplkError error code.
*/
typedef tOplkError (*tComdLayerObdCb)(tSdoObdConHdl* pSdoHdl_p,
                                      tCmdLayerObdFinishedCb pfnFinishSdoCb_p);

#endif /* _INC_oplk_sdo_H_ */
