/****************************************************************************

  (c) SYSTEC electronic GmbH, D-07973 Greiz, August-Bebel-Str. 29
      www.systec-electronic.com

  Project:      openPOWERLINK

  Description:  include file for api function of EplOBD-Module

  License:

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    1. Redistributions of source code must retain the above copyright
       notice, this list of conditions and the following disclaimer.

    2. Redistributions in binary form must reproduce the above copyright
       notice, this list of conditions and the following disclaimer in the
       documentation and/or other materials provided with the distribution.

    3. Neither the name of SYSTEC electronic GmbH nor the names of its
       contributors may be used to endorse or promote products derived
       from this software without prior written permission. For written
       permission, please contact info@systec-electronic.com.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    POSSIBILITY OF SUCH DAMAGE.

    Severability Clause:

        If a provision of this License is or becomes illegal, invalid or
        unenforceable in any jurisdiction, that shall not affect:
        1. the validity or enforceability in that jurisdiction of any other
           provision of this License; or
        2. the validity or enforceability in other jurisdictions of that or
           any other provision of this License.

  -------------------------------------------------------------------------

                $RCSfile$

                $Author$

                $Revision$  $Date$

                $State$

                Build Environment:
                Microsoft VC7

  -------------------------------------------------------------------------

  Revision History:

  2006/06/02 k.t.:   start of the implementation, version 1.00


****************************************************************************/

#ifndef _EPLOBD_H_
#define _EPLOBD_H_

#include "EplInc.h"


// ============================================================================
// defines
// ============================================================================

#define EPL_OBD_TABLE_INDEX_END     0xFFFF

// for the usage of BOOLEAN in OD
#define OBD_TRUE                                    0x01
#define OBD_FALSE                                   0x00

#define OBD_NODE_ID_INDEX                           0x1F93      // default OD index for Node id
#define OBD_NODE_ID_SUBINDEX                        0x01        // default subindex for NodeId in OD
#define OBD_NODE_ID_HWBOOL_SUBINDEX                 0x02        // default subindex for NodeIDByHW_BOOL

//------------------------------------------------------------------------------
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

// ============================================================================
// enums
// ============================================================================

// directions for access to object dictionary
typedef enum
{
    kObdDirInit             = 0x00,    // initialising after power on
    kObdDirStore            = 0x01,    // store all object values to non volatile memory
    kObdDirLoad             = 0x02,    // load all object values from non volatile memory
    kObdDirRestore          = 0x03,    // deletes non volatile memory (restore)
    kObdDirOBKCheck         = 0xFF     // reserved
}tObdDir;

// commands for store
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

// events of object callback function
typedef enum
{
//                                                                                                      m_pArg points to
//                                                                                                    ---------------------
    kObdEvCheckExist        = 0x06,    // checking if object does exist (reading and writing)    NULL
    kObdEvPreRead           = 0x00,    // before reading an object                               source data buffer in OD
    kObdEvPostRead          = 0x01,    // after reading an object                                destination data buffer from caller
    kObdEvWrStringDomain    = 0x07,    // event for changing string/domain data pointer or size  struct tObdVStringDomain in RAM
    kObdEvInitWrite         = 0x04,    // initializes writing an object (checking object size)   size of object in OD (tObdSize)
    kObdEvPreWrite          = 0x02,    // before writing an object                               source data buffer from caller
    kObdEvPostWrite         = 0x03,    // after writing an object                                destination data buffer in OD
//    kObdEvAbortSdo          = 0x05     // after an abort of an SDO transfer
    kObdEvPostDefault       = 0x08,    // after setting default values                           data buffer in OD
} tObdEvent;

// part of OD (bit oriented)
typedef unsigned int tObdPart;

#define kObdPartNo          0x00    // nothing
#define kObdPartGen         0x01    //  part      (0x1000 - 0x1FFF)
#define kObdPartMan         0x02    // manufacturer part (0x2000 - 0x5FFF)
#define kObdPartDev         0x04    // device part       (0x6000 - 0x9FFF)
#define kObdPartUsr         0x08    // dynamic part e.g. for ICE61131-3

// combinations
#define kObdPartApp         (              kObdPartMan | kObdPartDev | kObdPartUsr)   // manufacturer and device part (0x2000 - 0x9FFF) and user OD
#define kObdPartAll         (kObdPartGen | kObdPartMan | kObdPartDev | kObdPartUsr)   // whole OD

//-----------------------------------------------------------------------------------------------------------
// access types for objects
// must be a difine because bit-flags
typedef unsigned int tObdAccess;

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



typedef unsigned int tObdSize; // For all objects as objects size are used an unsigned int.


// -------------------------------------------------------------------------
// types for data types defined in DS301
// -------------------------------------------------------------------------

// types of objects in object dictionary
// DS-301 defines these types as WORD
typedef enum
{
// types which are always supported
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
// other types are not supported in this version


// -------------------------------------------------------------------------
// types for data types defined in DS301
// -------------------------------------------------------------------------

typedef unsigned char               tObdBoolean;      // 0001
typedef signed char                 tObdInteger8;     // 0002
typedef signed short int            tObdInteger16;    // 0003
typedef signed int                  tObdInteger32;    // 0004
typedef unsigned char               tObdUnsigned8;    // 0005
typedef unsigned short int          tObdUnsigned16;   // 0006
typedef unsigned int                tObdUnsigned32;   // 0007
typedef float                       tObdReal32;       // 0008
typedef unsigned char               tObdDomain;       // 000F
typedef signed   int                tObdInteger24;    // 0010
typedef unsigned int                tObdUnsigned24;   // 0016

typedef signed long long int        tObdInteger40;    // 0012
typedef signed long long int        tObdInteger48;    // 0013
typedef signed long long int        tObdInteger56;    // 0014
typedef signed long long int        tObdInteger64;    // 0015

typedef unsigned long long int      tObdUnsigned40;   // 0018
typedef unsigned long long int      tObdUnsigned48;   // 0019
typedef unsigned long long int      tObdUnsigned56;   // 001A
typedef unsigned long long int      tObdUnsigned64;   // 001B

typedef double                      tObdReal64;       // 0011

typedef tTimeOfDay                  tObdTimeOfDay;         // 000C
typedef tTimeOfDay                  tObdTimeDifference;    // 000D


// -------------------------------------------------------------------------
// structur for defining a variable
// -------------------------------------------------------------------------
// -------------------------------------------------------------------------
typedef enum
{
    kVarValidSize           = 0x01,
    kVarValidData           = 0x02,
//    kVarValidCallback       = 0x04,
//    kVarValidArg            = 0x08,
    kVarValidAll            = 0x03  // currently only size and data are implemented and used
}tVarParamValid;


//typedef tEplKernel (PUBLIC ROM* tEplVarCallback) (CCM_DECL_INSTANCE_HDL_
//    void * pParam_p);

typedef struct
{
    tVarParamValid      m_ValidFlag;
    UINT                m_uiIndex;
    UINT                m_uiSubindex;
    tObdSize            m_Size;
    void MEM*           m_pData;
//    tEplVarCallback     m_fpCallback;
//    void *       m_pArg;
} tVarParam;

typedef struct
{
    void MEM*           m_pData;
    tObdSize            m_Size;
/*
    #if (EPL_PDO_USE_STATIC_MAPPING == FALSE)
        tEplVarCallback    m_fpCallback;
        void *   m_pArg;
    #endif
*/
} tObdVarEntry;

typedef struct
{
   tObdSize         m_Size;
   BYTE *           m_pString;
} tObdOString;                          // 000C

typedef struct
{
   tObdSize         m_Size;
   char*            m_pString;
} tObdVString;                          // 000D


typedef struct
{
    tObdSize        m_Size;
    CONST char*     m_pDefString;         // must be same offset as m_pString in tObdVString
    char*           m_pString;
} tObdVStringDef;

typedef struct
{
   tObdSize         m_Size;
   UINT8*           m_pDefString;   // must be same offset as m_pString in tObdVString
   UINT8*           m_pString;
} tObdOStringDef;

//r.d. parameter struct for changing object size and/or pointer to data of Strings or Domains
typedef struct
{
   tObdSize      m_DownloadSize;     // download size from SDO or APP
   tObdSize      m_ObjSize;          // current object size from OD - should be changed from callback function
   void*         m_pData;            // current object ptr  from OD - should be changed from callback function
} tObdVStringDomain;                          // 000D


// ============================================================================
// types
// ============================================================================
// -------------------------------------------------------------------------
// subindexstruct
// -------------------------------------------------------------------------

// Change not the order for this struct!!!
typedef struct
{
    UINT            m_uiSubIndex;
    tObdType        m_Type;
    tObdAccess      m_Access;
    CONST void ROM* m_pDefault;
    void  MEM*      m_pCurrent;     // points always to RAM
} tObdSubEntry;

// r.d.: has always to be, because of new OBD-Macros for arrays
typedef tObdSubEntry * tObdSubEntryPtr;

// -------------------------------------------------------------------------
// callback function for object dictionary module
// -------------------------------------------------------------------------

// parameters for callback function
typedef struct
{
    tObdEvent       m_ObdEvent;
    UINT            m_uiIndex;
    UINT            m_uiSubIndex;
    void *          m_pArg;
    UINT32          m_dwAbortCode;
} tObdCbParam;

// define type for callback function: pParam_p points to tObdCbParam
typedef tEplKernel (PUBLIC ROM* tObdCallback) (tObdCbParam MEM* pParam_p);

// do not change the order for this struct!!!

typedef struct
{
    UINT                m_uiIndex;
    tObdSubEntryPtr     m_pSubIndex;
    UINT                m_uiCount;
    tObdCallback        m_fpCallback;   // function is called back if object access
} tObdEntry;

// allways  pointer
typedef tObdEntry * tObdEntryPtr;


// -------------------------------------------------------------------------
// structur to initialize OBD module
// -------------------------------------------------------------------------

struct _tObdInitParam
{
    tObdEntryPtr            m_pGenericPart;
    tObdEntryPtr            m_pManufacturerPart;
    tObdEntryPtr            m_pDevicePart;

    #if (defined (EPL_OBD_USER_OD) && (EPL_OBD_USER_OD != FALSE))

          tObdEntryPtr      m_pUserPart;

    #endif

};

typedef struct _tObdInitParam tObdInitParam;


// -------------------------------------------------------------------------
// structur for parameters of STORE RESTORE command
// -------------------------------------------------------------------------

typedef struct
{
    tObdCommand     m_bCommand;
    tObdPart        m_bCurrentOdPart;
    void MEM*       m_pData;
    tObdSize        m_ObjSize;

} tObdCbStoreParam;


typedef tEplKernel (PUBLIC ROM* tObdInitRam) (tObdInitParam MEM* pInitParam_p);

typedef tEplKernel (PUBLIC ROM* tObdDeinitRam) (tObdInitParam MEM* pInitParam_p);


typedef tEplKernel (PUBLIC ROM* tInitTabEntryCallback) (
    void MEM* pTabEntry_p,
    UINT uiObjIndex_p);


typedef tEplKernel (PUBLIC ROM* tEplObdStoreLoadObjCallback) (tObdCbStoreParam MEM* pCbStoreParam_p);

// -------------------------------------------------------------------------
// this stucture is used for parameters for function ObdInitModuleTab()
// -------------------------------------------------------------------------
typedef struct
{
    UINT                        m_uiLowerObjIndex;  // lower limit of ObjIndex
    UINT                        m_uiUpperObjIndex;  // upper limit of ObjIndex
    tInitTabEntryCallback       m_fpInitTabEntry;   // will be called if ObjIndex was found
    void MEM*                   m_pTabBase;         // base address of table
    UINT                        m_uiEntrySize;      // size of table entry      // 25-feb-2005 r.d.: expansion from BYTE to WORD necessary for PDO bit mapping
    UINT                        m_uiMaxEntries;     // max. tabel entries

} tObdModulTabParam;

//-------------------------------------------------------------------
//  enum for function obd_setNodeId
//-------------------------------------------------------------------
typedef enum
{
    kObdNodeIdUnknown       = 0x00,   // unknown how the node id was set
    kObdNodeIdSoftware      = 0x01,   // node id set by software
    kObdNodeIdHardware      = 0x02    // node id set by hardware
}tObdNodeIdType;

// ============================================================================
// global variables
// ============================================================================



// ============================================================================
// public functions
// ============================================================================
// ---------------------------------------------------------------------
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

#endif  // #ifndef _EPLOBD_H_


