/**
********************************************************************************
\file   obd.h

\brief  Definitions for OBD module

This file contains definitions for the OBD module
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2013, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
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

#ifndef _INC_obd_H_
#define _INC_obd_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <EplInc.h>

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#define EPL_OBD_TABLE_INDEX_END     0xFFFF

// for the usage of BOOLEAN in OD
#define OBD_TRUE                                    0x01
#define OBD_FALSE                                   0x00

#define OBD_NODE_ID_INDEX                           0x1F93      // default OD index for Node id
#define OBD_NODE_ID_SUBINDEX                        0x01        // default subindex for NodeId in OD
#define OBD_NODE_ID_HWBOOL_SUBINDEX                 0x02        // default subindex for NodeIDByHW_BOOL

// object IDs of error handling objects
#define OID_DLL_MN_CRCERROR_REC                     0x1C00
#define OID_DLL_MN_CYCTIME_EXCEED_REC               0x1C02
#define OID_DLL_CN_LOSSSOC_REC                      0x1C0B
#define OID_DLL_CN_LOSSPREQ_REC                     0x1C0D
#define OID_DLL_CN_CRCERROR_REC                     0x1C0F

#define SUBIDX_DLL_ERROR_CUM_CNT                    1
#define SUBIDX_DLL_ERROR_THR_CNT                    2
#define SUBIDX_DLL_ERROR_THRESHOLD                  3

#define NUM_DLL_MNCN_LOSSPRES_OBJS                  254
#define OID_DLL_MNCN_LOSSPRES_CUMCNT_AU32           0x1C07
#define OID_DLL_MNCN_LOSSPRES_THRCNT_AU32           0x1C08
#define OID_DLL_MNCN_LOSSPRES_THRESHOLD_AU32        0x1C09

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
}tObdDir;

/**
* \brief Valid OD store commands
*
* This enumeration defines valid store commands for the OD
*/
typedef enum
{
    kObdCmdNothing          = 0x00,
    kObdCmdOpenWrite        = 0x01,
    kObdCmdWriteObj         = 0x02,
    kObdCmdCloseWrite       = 0x03,
    kObdCmdOpenRead         = 0x04,
    kObdCmdReadObj          = 0x05,
    kObdCmdCloseRead        = 0x06,
    kObdCmdClear            = 0x07,
    kObdCmdUnknown          = 0xFF
}tObdCommand;

/**
* \brief Events of object callback function
*
* This enumeration defines events that can be handled by the object
* callback function.
*/
typedef enum
{
    kObdEvCheckExist        = 0x06,     ///< Checking if object does exist (reading and writing).  pArg points to: NULL
    kObdEvPreRead           = 0x00,     ///< Called before reading an object. pArg points to: source data buffer in OD
    kObdEvPostRead          = 0x01,     ///< Called after reading an object. pArg points to: destination data buffer from caller
    kObdEvWrStringDomain    = 0x07,     ///< Event for changing string/domain data pointer or size. pArg points to: struct tObdVStringDomain in RAM
    kObdEvInitWrite         = 0x04,     ///< Initializes writing an object (checking object size). pArg points to: size of object in OD (tObdSize)
    kObdEvPreWrite          = 0x02,     ///< Called before writing an object. pArg points to: source data buffer from caller
    kObdEvPostWrite         = 0x03,     ///< Called after writing an object. pArg points to: destination data buffer in OD
    kObdEvPostDefault       = 0x08,     ///< Called after setting default values. pArg points to: data buffer in OD
} tObdEvent;

typedef unsigned int tObdPart;          ///< Data type for OD part definitions

// Definitions for parts of the OD (bit oriented)
#define kObdPartNo          0x00        ///< Nothing
#define kObdPartGen         0x01        ///< Communication part (0x1000 - 0x1FFF)
#define kObdPartMan         0x02        ///< Manufacturer part (0x2000 - 0x5FFF)
#define kObdPartDev         0x04        ///< Device part (0x6000 - 0x9FFF)
#define kObdPartUsr         0x08        ///< Dynamic part e.g. for ICE61131-3

// combinations
#define kObdPartApp         (              kObdPartMan | kObdPartDev | kObdPartUsr)   ///< Manufacturer, device part and user OD
#define kObdPartAll         (kObdPartGen | kObdPartMan | kObdPartDev | kObdPartUsr)   ///< The whole OD

typedef unsigned int tObdAccess;        ///< Data type for OD access types

//-----------------------------------------------------------------------------------------------------------
// access types for objects
#define kObdAccRead         0x01    // object can be read
#define kObdAccWrite        0x02    // object can be written
#define kObdAccConst        0x04    // object contains a constant value
#define kObdAccPdo          0x08    // object can be mapped in a PDO
#define kObdAccArray        0x10    // object contains an array of numerical values
#define kObdAccRange        0x20    // object contains lower and upper limit
#define kObdAccVar          0x40    // object data is placed in application
#define kObdAccStore        0x80    // object data can be stored to non volatile memory

// combinations (not all combinations are required)
#define kObdAccR            (0            | 0          | 0            | 0          | 0            | 0            | kObdAccRead)
#define kObdAccW            (0            | 0          | 0            | 0          | 0            | kObdAccWrite | 0          )
#define kObdAccRW           (0            | 0          | 0            | 0          | 0            | kObdAccWrite | kObdAccRead)
#define kObdAccCR           (0            | 0          | 0            | 0          | kObdAccConst | 0            | kObdAccRead)
#define kObdAccGR           (0            | 0          | kObdAccRange | 0          | 0            | 0            | kObdAccRead)
#define kObdAccGW           (0            | 0          | kObdAccRange | 0          | 0            | kObdAccWrite | 0          )
#define kObdAccGRW          (0            | 0          | kObdAccRange | 0          | 0            | kObdAccWrite | kObdAccRead)
#define kObdAccVR           (0            | kObdAccVar | 0            | 0          | 0            | 0            | kObdAccRead)
#define kObdAccVW           (0            | kObdAccVar | 0            | 0          | 0            | kObdAccWrite | 0          )
#define kObdAccVRW          (0            | kObdAccVar | 0            | 0          | 0            | kObdAccWrite | kObdAccRead)
#define kObdAccVPR          (0            | kObdAccVar | 0            | kObdAccPdo | 0            | 0            | kObdAccRead)
#define kObdAccVPW          (0            | kObdAccVar | 0            | kObdAccPdo | 0            | kObdAccWrite | 0          )
#define kObdAccVPRW         (0            | kObdAccVar | 0            | kObdAccPdo | 0            | kObdAccWrite | kObdAccRead)
#define kObdAccVGR          (0            | kObdAccVar | kObdAccRange | 0          | 0            | 0            | kObdAccRead)
#define kObdAccVGW          (0            | kObdAccVar | kObdAccRange | 0          | 0            | kObdAccWrite | 0          )
#define kObdAccVGRW         (0            | kObdAccVar | kObdAccRange | 0          | 0            | kObdAccWrite | kObdAccRead)
#define kObdAccVGPR         (0            | kObdAccVar | kObdAccRange | kObdAccPdo | 0            | 0            | kObdAccRead)
#define kObdAccVGPW         (0            | kObdAccVar | kObdAccRange | kObdAccPdo | 0            | kObdAccWrite | 0          )
#define kObdAccVGPRW        (0            | kObdAccVar | kObdAccRange | kObdAccPdo | 0            | kObdAccWrite | kObdAccRead)
#define kObdAccSR           (kObdAccStore | 0          | 0            | 0          | 0            | 0            | kObdAccRead)
#define kObdAccSW           (kObdAccStore | 0          | 0            | 0          | 0            | kObdAccWrite | 0          )
#define kObdAccSRW          (kObdAccStore | 0          | 0            | 0          | 0            | kObdAccWrite | kObdAccRead)
#define kObdAccSCR          (kObdAccStore | 0          | 0            | 0          | kObdAccConst | 0            | kObdAccRead)
#define kObdAccSGR          (kObdAccStore | 0          | kObdAccRange | 0          | 0            | 0            | kObdAccRead)
#define kObdAccSGW          (kObdAccStore | 0          | kObdAccRange | 0          | 0            | kObdAccWrite | 0          )
#define kObdAccSGRW         (kObdAccStore | 0          | kObdAccRange | 0          | 0            | kObdAccWrite | kObdAccRead)
#define kObdAccSVR          (kObdAccStore | kObdAccVar | 0            | 0          | 0            | 0            | kObdAccRead)
#define kObdAccSVW          (kObdAccStore | kObdAccVar | 0            | 0          | 0            | kObdAccWrite | 0          )
#define kObdAccSVRW         (kObdAccStore | kObdAccVar | 0            | 0          | 0            | kObdAccWrite | kObdAccRead)
#define kObdAccSVPR         (kObdAccStore | kObdAccVar | 0            | kObdAccPdo | 0            | 0            | kObdAccRead)
#define kObdAccSVPW         (kObdAccStore | kObdAccVar | 0            | kObdAccPdo | 0            | kObdAccWrite | 0          )
#define kObdAccSVPRW        (kObdAccStore | kObdAccVar | 0            | kObdAccPdo | 0            | kObdAccWrite | kObdAccRead)
#define kObdAccSVGR         (kObdAccStore | kObdAccVar | kObdAccRange | 0          | 0            | 0            | kObdAccRead)
#define kObdAccSVGW         (kObdAccStore | kObdAccVar | kObdAccRange | 0          | 0            | kObdAccWrite | 0          )
#define kObdAccSVGRW        (kObdAccStore | kObdAccVar | kObdAccRange | 0          | 0            | kObdAccWrite | kObdAccRead)
#define kObdAccSVGPR        (kObdAccStore | kObdAccVar | kObdAccRange | kObdAccPdo | 0            | 0            | kObdAccRead)
#define kObdAccSVGPW        (kObdAccStore | kObdAccVar | kObdAccRange | kObdAccPdo | 0            | kObdAccWrite | 0          )
#define kObdAccSVGPRW       (kObdAccStore | kObdAccVar | kObdAccRange | kObdAccPdo | 0            | kObdAccWrite | kObdAccRead)

typedef unsigned int tObdSize;      // For all objects as objects size are used an unsigned int.

/**
* \brief Enumeration for object data types (DS301)
*
* This enumeration defines the data types of objects in object dictionary.
* DS-301 defines these types as UINT16
* openPOWERLINK support only the listed data types. Other types are not supported
* in this version.
*/
typedef enum
{
    kObdTypeBool            = 0x0001,

    kObdTypeInt8            = 0x0002,
    kObdTypeInt16           = 0x0003,
    kObdTypeInt32           = 0x0004,
    kObdTypeUInt8           = 0x0005,
    kObdTypeUInt16          = 0x0006,
    kObdTypeUInt32          = 0x0007,
    kObdTypeReal32          = 0x0008,
    kObdTypeVString         = 0x0009,
    kObdTypeOString         = 0x000A,
    kObdTypeDomain          = 0x000F,

    kObdTypeInt24           = 0x0010,
    kObdTypeUInt24          = 0x0016,

    kObdTypeReal64          = 0x0011,
    kObdTypeInt40           = 0x0012,
    kObdTypeInt48           = 0x0013,
    kObdTypeInt56           = 0x0014,
    kObdTypeInt64           = 0x0015,
    kObdTypeUInt40          = 0x0018,
    kObdTypeUInt48          = 0x0019,
    kObdTypeUInt56          = 0x001A,
    kObdTypeUInt64          = 0x001B,
    kObdTypeTimeOfDay       = 0x000C,
    kObdTypeTimeDiff        = 0x000D
} tObdType;

// type definitions for data types defined in DS301
typedef unsigned char               tObdBoolean;            // 0001
typedef signed char                 tObdInteger8;           // 0002
typedef signed short int            tObdInteger16;          // 0003
typedef signed int                  tObdInteger32;          // 0004
typedef unsigned char               tObdUnsigned8;          // 0005
typedef unsigned short int          tObdUnsigned16;         // 0006
typedef unsigned int                tObdUnsigned32;         // 0007
typedef float                       tObdReal32;             // 0008
typedef unsigned char               tObdDomain;             // 000F
typedef signed   int                tObdInteger24;          // 0010
typedef unsigned int                tObdUnsigned24;         // 0016

typedef signed long long int        tObdInteger40;          // 0012
typedef signed long long int        tObdInteger48;          // 0013
typedef signed long long int        tObdInteger56;          // 0014
typedef signed long long int        tObdInteger64;          // 0015

typedef unsigned long long int      tObdUnsigned40;         // 0018
typedef unsigned long long int      tObdUnsigned48;         // 0019
typedef unsigned long long int      tObdUnsigned56;         // 001A
typedef unsigned long long int      tObdUnsigned64;         // 001B

typedef double                      tObdReal64;             // 0011

typedef tTimeOfDay                  tObdTimeOfDay;          // 000C
typedef tTimeOfDay                  tObdTimeDifference;     // 000D

typedef enum
{
    kVarValidSize           = 0x01,
    kVarValidData           = 0x02,
    kVarValidAll            = 0x03          // currently only size and data are implemented and used
}tVarParamValid;

typedef struct
{
    tVarParamValid      validFlag;
    UINT                index;
    UINT                subindex;
    tObdSize            size;
    void MEM*           pData;
} tVarParam;

typedef struct
{
    void MEM*           pData;
    tObdSize            size;
} tObdVarEntry;

typedef struct
{
   tObdSize             size;
   BYTE *               pString;
} tObdOString;                              // 0009

typedef struct
{
   tObdSize             size;
   UINT8*               pDefString;         // must be same offset as pString in tObdVString
   UINT8*               pString;
} tObdOStringDef;

typedef struct
{
   tObdSize             size;
   char*                pString;
} tObdVString;                              // 000A

typedef struct
{
    tObdSize            size;
    CONST char*         pDefString;         // must be same offset as pString in tObdVString
    char*               pString;
} tObdVStringDef;

//r.d. parameter struct for changing object size and/or pointer to data of Strings or Domains
typedef struct
{
   tObdSize             downloadSize;       // download size from SDO or APP
   tObdSize             objSize;            // current object size from OD - should be changed from callback function
   void*                pData;              // current object ptr  from OD - should be changed from callback function
} tObdVStringDomain;                        // 000D

/**
* \brief Parameters for callback function
*
* This structure defines the parameters for the OD callback function.
*/
typedef struct
{
    tObdEvent           obdEvent;       ///< Event that caused calling the function.
    UINT                index;          ///< Index of the accessed object.
    UINT                subIndex;       ///< Subindex of the accessed object.
    void*               pArg;           ///< Additional argument.
    UINT32              abortCode;      ///< Abort Code.
} tObdCbParam;

// define type for callback function: pParam_p points to tObdCbParam
typedef tEplKernel (ROM* tObdCallback) (tObdCbParam MEM* pParam_p);

/**
* \brief Structure for subindices
*
* This structure defines a subindex in the OD.
*/
typedef struct
{
    UINT                subIndex;           ///< Subindex of the object
    tObdType            type;               ///< Data type of the object
    tObdAccess          access;             ///< Access type of the object
    CONST void ROM*     pDefault;           ///< Pointer to default data
    void  MEM*          pCurrent;           ///< Pointer to data (points always to RAM)
} tObdSubEntry;

typedef tObdSubEntry * tObdSubEntryPtr;


/**
* \brief Structure for indices
*
* This structure defines an index in the OD.
*/
typedef struct
{
    UINT                index;              ///< Index of the object
    tObdSubEntryPtr     pSubIndex;          ///< Points to subindex structures of this object
    UINT                count;              ///< number of subindices.
    tObdCallback        pfnCallback;        ///< function is called back if object access
} tObdEntry;

typedef tObdEntry * tObdEntryPtr;

/**
* \brief Structure for OBD init parameters
*
* This structure defines the init parameters of the OBD module.
*/
struct _tObdInitParam
{
    tObdEntryPtr        pGenericPart;           /// Pointer to generic part of OD
    tObdEntryPtr        pManufacturerPart;      ///< Pointer to manufacturer part of OD
    tObdEntryPtr        pDevicePart;            ///< Pointer to device part of OD
#if (defined (EPL_OBD_USER_OD) && (EPL_OBD_USER_OD != FALSE))
    tObdEntryPtr        pUserPart;              ///< Pointer to user part of OD
#endif
};

typedef struct _tObdInitParam tObdInitParam;

/**
* \brief Structure for parameters of the store/restore commands
*
* This structure specifies the parameters for the store/restore commands.
*/
typedef struct
{
    tObdCommand         command;
    tObdPart            currentOdPart;
    void MEM*           pData;
    tObdSize            objSize;
} tObdCbStoreParam;


typedef tEplKernel (ROM* tObdInitRam) (tObdInitParam MEM* pInitParam_p);
typedef tEplKernel (ROM* tObdDeinitRam) (tObdInitParam MEM* pInitParam_p);
typedef tEplKernel (ROM* tInitTabEntryCallback) (void MEM* pTabEntry_p, UINT uiObjIndex_p);
typedef tEplKernel (ROM* tEplObdStoreLoadObjCallback) (tObdCbStoreParam MEM* pCbStoreParam_p);

/**
* \brief Enumeration for Node ID setting types
*
* This structure defines constants for the types of setting the node ID.
* They are used in the function obd_setNodeId()
*/
typedef enum
{
    kObdNodeIdUnknown       = 0x00,         ///< unknown how the node id was set
    kObdNodeIdSoftware      = 0x01,         ///< node id set by software
    kObdNodeIdHardware      = 0x02          ///< node id set by hardware
}tObdNodeIdType;

//------------------------------------------------------------------------------
// function prototypes
//------------------------------------------------------------------------------
#ifdef __cplusplus
extern "C" {
#endif

tEplKernel  obd_init(tObdInitParam MEM* pInitParam_p);
tEplKernel  obd_addInstance(tObdInitParam MEM* pInitParam_p);
tEplKernel  obd_deleteInstance(void);
tEplKernel  obd_writeEntry(UINT index_p, UINT subIndex_p, void* pSrcData_p, tObdSize size_p);
tEplKernel  obd_readEntry(UINT index_p, UINT subIndex_p, void* pDstData_p, tObdSize *pSize_p);
tEplKernel  obd_storeLoadObjCallback(tEplObdStoreLoadObjCallback fpCallback_p);
tEplKernel  obd_accessOdPart(tObdPart obdPart_p, tObdDir direction_p);
tEplKernel  obd_defineVar(tVarParam MEM* pVarParam_p);
void*       obd_getObjectDataPtr(UINT index_p, UINT subIndex_p);
tEplKernel  obd_registerUserOd(tObdEntryPtr pUserOd_p);
void        obd_initVarEntry(tObdVarEntry MEM* pVarEntry_p, tObdType type_p, tObdSize obdSize_p);
tObdSize    obd_getDataSize(UINT index_p, UINT subIndex_p);
UINT        obd_getNodeId(void);
tEplKernel  obd_setNodeId(UINT nodeId_p, tObdNodeIdType nodeIdType_p);
tEplKernel  obd_isNumerical(UINT index_p, UINT subIndex_p, BOOL* pfEntryNumerical_p);
tEplKernel  obd_getType(UINT index_p, UINT subIndex_p, tObdType* pType_p);
tEplKernel  obd_writeEntryFromLe(UINT index_p, UINT subIndex_p, void* pSrcData_p, tObdSize size_p);
tEplKernel  obd_readEntryToLe(UINT index_p, UINT subIndex_p, void* pDstData_p, tObdSize* pSize_p);
tEplKernel  obd_getAccessType(UINT index_p, UINT subIndex_p, tObdAccess* pAccessTyp_p);
tEplKernel  obd_searchVarEntry(UINT index_p, UINT subindex_p, tObdVarEntry MEM** ppVarEntry_p);

#ifdef __cplusplus
}
#endif

#endif /* _INC_obd_H_ */

