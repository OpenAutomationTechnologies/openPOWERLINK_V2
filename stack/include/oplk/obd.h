/**
********************************************************************************
\file   oplk/obd.h

\brief  Definitions for OBD module

This file contains definitions for the OBD module
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2017, B&R Industrial Automation GmbH
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
#ifndef _INC_oplk_obd_H_
#define _INC_oplk_obd_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <oplk/oplkinc.h>
#include <oplk/sdo.h>

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#define OBD_TABLE_INDEX_END     0xFFFF

// for the usage of BOOLEAN in OD
#define OBD_TRUE                                    0x01
#define OBD_FALSE                                   0x00

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
} eObdEvent;

/**
\brief Object callback function event data type

Data type for the enumerator \ref eObdEvent.
*/
typedef UINT32 tObdEvent;

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

typedef UINT16 tObdAccess;        ///< Data type for OD access types


///\{
/**
* \anchor sect_obdAccessRights
* \name   Access rights for objects
*
* The following macros define the access rights for objects.
*/
#define kObdAccRead         0x01    ///< Object can be read
#define kObdAccWrite        0x02    ///< Object can be written
#define kObdAccConst        0x04    ///< Object contains a constant value
#define kObdAccPdo          0x08    ///< Object can be mapped to a PDO (always in conjunction with kObdAccVar)
#define kObdAccArray        0x10    ///< Object contains an array of numerical values
#define kObdAccRange        0x20    ///< Object contains lower and upper limit
#define kObdAccVar          0x40    ///< Object data is placed in application (contains a variable information structure)
#define kObdAccStore        0x80    ///< Object data can be stored to non-volatile memory

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
///\}

typedef size_t tObdSize;                // Use size_t for object dictionary sizes

/**
\brief Enumeration for object data types (DS301)

This enumeration defines the data types of objects in object dictionary.
DS-301 defines these types as UINT16.

openPOWERLINK supports only the listed data types. Other types are not supported
in this version.
*/
typedef enum
{
    kObdTypeBool            = 0x0001,      ///< 0001 - BOOLEAN
    kObdTypeInt8            = 0x0002,      ///< 0002 - INTEGER8
    kObdTypeInt16           = 0x0003,      ///< 0003 - INTEGER16
    kObdTypeInt32           = 0x0004,      ///< 0004 - INTEGER32
    kObdTypeUInt8           = 0x0005,      ///< 0005 - UNSIGNED8
    kObdTypeUInt16          = 0x0006,      ///< 0006 - UNSIGNED16
    kObdTypeUInt32          = 0x0007,      ///< 0007 - UNSIGNED32
    kObdTypeReal32          = 0x0008,      ///< 0008 - REAL32
    kObdTypeVString         = 0x0009,      ///< 0009 - VISIBLE_STRING
    kObdTypeOString         = 0x000A,      ///< 000A - OCTET_STRING

    kObdTypeTimeOfDay       = 0x000C,      ///< 000C - TIME_OF_DAY
    kObdTypeTimeDiff        = 0x000D,      ///< 000D - TIME_DIFFERENCE

    kObdTypeDomain          = 0x000F,      ///< 000F - DOMAIN
    kObdTypeInt24           = 0x0010,      ///< 0010 - INTEGER24
    kObdTypeReal64          = 0x0011,      ///< 0011 - REAL64
    kObdTypeInt40           = 0x0012,      ///< 0012 - INTEGER40
    kObdTypeInt48           = 0x0013,      ///< 0013 - INTEGER48
    kObdTypeInt56           = 0x0014,      ///< 0014 - INTEGER56
    kObdTypeInt64           = 0x0015,      ///< 0015 - INTEGER64
    kObdTypeUInt24          = 0x0016,      ///< 0016 - UNSIGNED24

    kObdTypeUInt40          = 0x0018,      ///< 0018 - UNSIGNED40
    kObdTypeUInt48          = 0x0019,      ///< 0019 - UNSIGNED48
    kObdTypeUInt56          = 0x001A,      ///< 001A - UNSIGNED56
    kObdTypeUInt64          = 0x001B,      ///< 001B - UNSIGNED64
    kObdTypeMax             = 0x001C
} eObdType;

/**
\brief Object data type data type

Data type for the enumerator \ref eObdType.
*/
typedef UINT16 tObdType;

///\{
/**
\name C type definitions for data types defined in POWERLINK DS301

The following C data types are defined according to the POWERLINK DS301
specification.
*/
typedef unsigned char               tObdBoolean;            ///< for DS301 data type \ref kObdTypeBool
typedef signed char                 tObdInteger8;           ///< for DS301 data type \ref kObdTypeInt8
typedef signed short int            tObdInteger16;          ///< for DS301 data type \ref kObdTypeInt16
typedef signed int                  tObdInteger32;          ///< for DS301 data type \ref kObdTypeInt32
typedef unsigned char               tObdUnsigned8;          ///< for DS301 data type \ref kObdTypeUInt8
typedef unsigned short int          tObdUnsigned16;         ///< for DS301 data type \ref kObdTypeUInt16
typedef unsigned int                tObdUnsigned32;         ///< for DS301 data type \ref kObdTypeUInt32
typedef float                       tObdReal32;             ///< for DS301 data type \ref kObdTypeReal32

typedef tTimeOfDay                  tObdTimeOfDay;          ///< for DS301 data type \ref kObdTypeTimeOfDay
typedef tTimeOfDay                  tObdTimeDifference;     ///< for DS301 data type \ref kObdTypeTimeDiff

typedef unsigned char               tObdDomain;             ///< for DS301 data type \ref kObdTypeDomain
typedef signed   int                tObdInteger24;          ///< for DS301 data type \ref kObdTypeInt24
typedef double                      tObdReal64;             ///< for DS301 data type \ref kObdTypeReal64
typedef signed long long int        tObdInteger40;          ///< for DS301 data type \ref kObdTypeInt40
typedef signed long long int        tObdInteger48;          ///< for DS301 data type \ref kObdTypeInt48
typedef signed long long int        tObdInteger56;          ///< for DS301 data type \ref kObdTypeInt56
typedef signed long long int        tObdInteger64;          ///< for DS301 data type \ref kObdTypeInt64
typedef unsigned int                tObdUnsigned24;         ///< for DS301 data type \ref kObdTypeUInt24

typedef unsigned long long int      tObdUnsigned40;         ///< for DS301 data type \ref kObdTypeUInt40
typedef unsigned long long int      tObdUnsigned48;         ///< for DS301 data type \ref kObdTypeUInt48
typedef unsigned long long int      tObdUnsigned56;         ///< for DS301 data type \ref kObdTypeUInt56
typedef unsigned long long int      tObdUnsigned64;         ///< for DS301 data type \ref kObdTypeUInt64
///\}

typedef enum
{
    kVarValidSize           = 0x01,
    kVarValidData           = 0x02,
    kVarValidAll            = 0x03          // currently only size and data are implemented and used
} eVarParamValid;

typedef UINT32 tVarParamValid;

typedef struct
{
    tVarParamValid      validFlag;
    UINT                index;
    UINT                subindex;
    tObdSize            size;
    void*               pData;
} tVarParam;

typedef struct
{
    void*               pData;
    tObdSize            size;
} tObdVarEntry;

/// C type definition for DS301 data type \ref kObdTypeOString
typedef struct
{
   tObdSize             size;
   BYTE*                pString;
} tObdOString;                              // 0009

typedef struct
{
   tObdSize             size;
   UINT8*               pDefString;         // must be same offset as pString in tObdVString
   UINT8*               pString;
} tObdOStringDef;


/// C type definition for DS301 data type \ref kObdTypeVString
typedef struct
{
   tObdSize             size;
   char*                pString;
} tObdVString;                              // 000A

typedef struct
{
    tObdSize            size;
    const char*         pDefString;         // must be same offset as pString in tObdVString
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
\brief Parameters for callback function

This structure defines the parameters for the OD callback function.
*/
typedef struct
{
    tObdEvent           obdEvent;       ///< Event that caused calling the function.
    UINT                index;          ///< Index of the accessed object.
    UINT                subIndex;       ///< Subindex of the accessed object.
    const void*         pArg;           ///< Additional argument.
    UINT32              abortCode;      ///< Abort Code.
} tObdCbParam;

// define type for callback function: pParam_p points to tObdCbParam
typedef tOplkError (*tObdCallback)(tObdCbParam* pParam_p);

/**
\brief Structure for subindices

This structure defines a subindex in the OD.
*/
typedef struct
{
    UINT                subIndex;           ///< Subindex of the object
    tObdType            type;               ///< Data type of the object
    tObdAccess          access;             ///< Access type of the object
    const void*         pDefault;           ///< Pointer to default data
    void*               pCurrent;           ///< Pointer to data (points always to RAM)
} tObdSubEntry;

/**
\brief Structure for indices

This structure defines an index in the OD.
*/
typedef struct
{
    UINT                index;              ///< Index of the object
    tObdSubEntry*       pSubIndex;          ///< Points to subindex structures of this object
    UINT                count;              ///< Number of subindices.
    BOOL                fUserEvent;         ///< Flag enabling the generation of a user event
} tObdEntry;

/**
\brief Structure for OBD init parameters

This structure defines the init parameters of the OBD module.
*/
struct _tObdInitParam
{
    tObdEntry*          pGenericPart;           ///< Pointer to generic part of OD
    UINT32              numGeneric;             ///< Number of entries in generic partition
    tObdEntry*          pManufacturerPart;      ///< Pointer to manufacturer part of OD
    UINT32              numManufacturer;        ///< Number of entries in manufacturer partition
    tObdEntry*          pDevicePart;            ///< Pointer to device part of OD
    UINT32              numDevice;              ///< Number of entries in device partition
#if (defined(OBD_USER_OD) && (OBD_USER_OD != FALSE))
    tObdEntry*          pUserPart;              ///< Pointer to user part of OD
    UINT32              numUser;                ///< Number of entries in user partition
#endif
};

typedef struct _tObdInitParam tObdInitParam;

//------------------------------------------------------------------------------
// function prototypes
//------------------------------------------------------------------------------
#ifdef __cplusplus
extern "C"
{
#endif

#ifdef __cplusplus
}
#endif

#endif /* _INC_oplk_obd_H_ */
