/**
********************************************************************************
\file   obdu.h

\brief  Definitions for user OBD module

This file contains definitions for the user OBD module
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2015, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
Copyright (c) 2013, SYSTEC electronic GmbH
Copyright (c) 2013, Kalycito Infotech Private Ltd.All rights reserved.
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

#ifndef _INC_obdu_H_
#define _INC_obdu_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <common/oplkinc.h>

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#define OBD_NODE_ID_INDEX                           0x1F93      // default OD index for Node id
#define OBD_NODE_ID_SUBINDEX                        0x01        // default subindex for NodeId in OD
#define OBD_NODE_ID_HWBOOL_SUBINDEX                 0x02        // default subindex for NodeIDByHW_BOOL

//------------------------------------------------------------------------------
// typedef
//------------------------------------------------------------------------------

/**
* \brief Directions for access to object dictionary
*
* This enumeration defines valid "directions" for accesses to the object
* dictionary.
*/
typedef enum
{
    kObdDirInit             = 0x00,    ///< Initialising after power on
    kObdDirStore            = 0x01,    ///< Store all object values to non volatile memory
    kObdDirLoad             = 0x02,    ///< Load all object values from non volatile memory
    kObdDirRestore          = 0x03,    ///< Deletes non volatile memory (restore)
    kObdDirOBKCheck         = 0xFF     ///< Reserved
} eObdDir;

/**
\brief Directions for access to object dictionary data type

Data type for the enumerator \ref eObdDir.
*/
typedef UINT32 tObdDir;

/**
* \brief Valid OD store commands
*
* This enumeration defines valid store commands for the OD
*/
typedef enum
{
    kObdCmdOpenWrite        = 0x01,
    kObdCmdWriteObj         = 0x02,
    kObdCmdCloseWrite       = 0x03,
    kObdCmdOpenRead         = 0x04,
    kObdCmdReadObj          = 0x05,
    kObdCmdCloseRead        = 0x06,
    kObdCmdClear            = 0x07,
} eObdCommand;

/**
\brief Valid OD store command data type

Data type for the enumerator \ref eObdCommand.
*/
typedef UINT32 tObdCommand;

/**
\brief Structure for parameters of the store/restore commands

This structure specifies the parameters for the store/restore commands.
*/
typedef struct
{
    tObdCommand         command;
    tObdPart            currentOdPart;
    void MEM*           pData;
    tObdSize            objSize;
} tObdCbStoreParam;

typedef tOplkError (ROM *tInitTabEntryCallback)(void MEM* pTabEntry_p, UINT uiObjIndex_p);
typedef tOplkError (ROM *tObdStoreLoadCallback)(tObdCbStoreParam MEM* pCbStoreParam_p);

/**
\brief Enumeration for Node ID setting types

This structure defines constants for the types of setting the node ID.
They are used in the function obd_setNodeId()
*/
typedef enum
{
    kObdNodeIdUnknown       = 0x00,         ///< unknown how the node id was set
    kObdNodeIdSoftware      = 0x01,         ///< node id set by software
    kObdNodeIdHardware      = 0x02          ///< node id set by hardware
} eObdNodeIdType;

/**
\brief Node ID setting data type

Data type for the enumerator \ref eObdNodeIdType.
*/
typedef UINT32 tObdNodeIdType;


//------------------------------------------------------------------------------
// function prototypes
//------------------------------------------------------------------------------
#ifdef __cplusplus
extern "C"
{
#endif

tOplkError obd_init(tObdInitParam MEM* pInitParam_p);
tOplkError obd_exit(void);
tOplkError obd_writeEntry(UINT index_p, UINT subIndex_p, void* pSrcData_p, tObdSize size_p);
tOplkError obd_readEntry(UINT index_p, UINT subIndex_p, void* pDstData_p, tObdSize* pSize_p);
tOplkError obd_accessOdPart(tObdPart obdPart_p, tObdDir direction_p);
tOplkError obd_defineVar(tVarParam MEM* pVarParam_p);
void*      obd_getObjectDataPtr(UINT index_p, UINT subIndex_p);
tOplkError obd_registerUserOd(tObdEntryPtr pUserOd_p);
void       obd_initVarEntry(tObdVarEntry MEM* pVarEntry_p, tObdType type_p, tObdSize obdSize_p);
tObdSize   obd_getDataSize(UINT index_p, UINT subIndex_p);
UINT       obd_getNodeId(void);
tOplkError obd_setNodeId(UINT nodeId_p, tObdNodeIdType nodeIdType_p);
tOplkError obd_isNumerical(UINT index_p, UINT subIndex_p, BOOL* pfEntryNumerical_p);
tOplkError obd_getType(UINT index_p, UINT subIndex_p, tObdType* pType_p);
tOplkError obd_writeEntryFromLe(UINT index_p, UINT subIndex_p, void* pSrcData_p, tObdSize size_p);
tOplkError obd_readEntryToLe(UINT index_p, UINT subIndex_p, void* pDstData_p, tObdSize* pSize_p);
tOplkError obd_getAccessType(UINT index_p, UINT subIndex_p, tObdAccess* pAccessType_p);
tOplkError obd_searchVarEntry(UINT index_p, UINT subindex_p, tObdVarEntry MEM** ppVarEntry_p);

#if defined(CONFIG_OBD_USE_STORE_RESTORE) && (CONFIG_OBD_USE_STORE_RESTORE != FALSE)
tOplkError obd_storeLoadObjCallback(tObdStoreLoadCallback pfnCallback_p);
#endif

#if defined(CONFIG_OBD_CALC_OD_SIGNATURE) && (CONFIG_OBD_CALC_OD_SIGNATURE != FALSE)
UINT32     obd_getOdSignature(tObdPart odPart_p);
#endif

tOplkError obd_processWrite(tSdoObdConHdl* pSdoObdConHdl_p);
tOplkError obd_processRead(tSdoObdConHdl* pSdoObdConHdl_p);

#ifdef __cplusplus
}
#endif

#endif /* _INC_obdu_H_ */
