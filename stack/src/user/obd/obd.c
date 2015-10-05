/**
********************************************************************************
\file   obd.c

\brief  Implementation of object dictionary (OD) module

This file contains the implementation of the object dictionary (OD) module.

\ingroup module_obd
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2015, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
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

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <common/oplkinc.h>
#include <oplk/obd.h>
#include <common/ami.h>

#if (CONFIG_OBD_CALC_OD_SIGNATURE != FALSE)
#include <common/target.h>
#endif

//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// module global vars
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// global function prototypes
//------------------------------------------------------------------------------

//============================================================================//
//            P R I V A T E   D E F I N I T I O N S                           //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------
// decomposition of float
typedef union
{
    tObdReal32      flRealPart;
    int             nIntegerPart;
} tObdRealParts;

typedef struct
{
    size_t          size;
    tObdSize        (*pfnGetObjSize)(tObdSubEntryPtr pSubIndexEntry_p);
} tObdDataTypeSize;

typedef struct
{
    tObdInitParam                   initParam;
    tObdStoreLoadCallback           pfnStoreLoadObjectCb;
#if (CONFIG_OBD_CALC_OD_SIGNATURE != FALSE)
    UINT32                          aOdSignature[3];
#endif
    UINT8                           obdTrashObject[8];
} tObdInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tObdInstance                 obdInstance_l;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static tOplkError   writeEntryPre(UINT uiIndex_p, UINT subIndex_p, void* pSrcData_p, void** ppDstData_p,
                                  tObdSize size_p, tObdEntryPtr* ppObdEntry_p, tObdSubEntryPtr* ppSubEntry_p,
                                  tObdCbParam MEM* pCbParam_p, tObdSize* pObdSize_p);
static tOplkError   writeEntryPost(tObdEntryPtr pObdEntry_p, tObdSubEntryPtr pSubEntry_p,
                                   tObdCbParam MEM* pCbParam_p, void* pSrcData_p,
                                   void* pDstData_p, tObdSize obdSize_p);
static tObdSize     getDataSize(tObdSubEntryPtr pSubIndexEntry_p);
static tObdSize     getObdStringLen(void* pObjData_p, tObdSize objLen_p, tObdType objType_p);
static tObdSize     getDomainSize(tObdSubEntryPtr pSubIndexEntry_p);
static tObdSize     getVstringSize(tObdSubEntryPtr pSubIndexEntry_p);
static tObdSize     getOstringSize(tObdSubEntryPtr pSubIndexEntry_p);
static tObdSize     getObjectSize(tObdSubEntryPtr pSubIndexEntry_p);
static tOplkError   getVarEntry(tObdSubEntryPtr pSubIndexEntry_p, tObdVarEntry MEM** ppVarEntry_p);
static tOplkError   getEntry(UINT index_p, UINT subIndex_p, tObdEntryPtr* ppObdEntry_p,
                             tObdSubEntryPtr* ppObdSubEntry_p);
static CONST void*  getObjectDefaultPtr(tObdSubEntryPtr pSubIndexEntry_p);
static void MEM*    getObjectCurrentPtr(tObdSubEntryPtr pSubIndexEntry_p);
static void*        getObjectDataPtr(tObdSubEntryPtr pSubIndexEntry_p);
static tObdEntryPtr searchIndex(tObdEntryPtr pObdEntry_p, UINT32 numEntries_p, UINT index_p);
static tOplkError   getIndex(tObdInitParam MEM* pInitParam_p, UINT index_p, tObdEntryPtr* ppObdEntry_p);
static tOplkError   getSubindex(tObdEntryPtr pObdEntry_p, UINT subIndex_p, tObdSubEntryPtr* ppObdSubEntry_p);
static tOplkError   accessOdPartition(tObdPart currentOdPart_p, tObdEntryPtr pObdEnty_p, tObdDir direction_p);
static void         copyObjectData(void MEM* pDstData_p, CONST void* pSrcData_p, tObdSize objSize_p,
                                   tObdType objType_p);
static tOplkError   callObjectCallback(tObdCallback pfnCallback_p, tObdCbParam MEM* pCbParam_p);
static tOplkError   callPostDefault(void* pData_p, tObdEntryPtr pObdEntry_p, tObdSubEntryPtr pObdSubEntry_p);
static tOplkError   isNumerical(tObdSubEntryPtr pObdSubEntry_p, BOOL* pfEntryNumerical_p);
static UINT32       calcPartitionIndexNum(tObdEntryPtr pObdEntry_p);
static void         calcOdIndexNum(tObdInitParam* pInitParam_p);

#if (CONFIG_OBD_CHECK_OBJECT_RANGE != FALSE)
static tOplkError   checkObjectRange(tObdSubEntryPtr pSubIndexEntry_p, void* pData_p);
#endif

#if (CONFIG_OBD_USE_STORE_RESTORE != FALSE)
static tOplkError   prepareStoreRestore(tObdDir direction_p, tObdCbStoreParam MEM* pCbStore_p);
static tOplkError   cleanupStoreRestore(tObdDir direction_p, tObdCbStoreParam MEM* pCbStore_p);
static tOplkError   doStoreRestore(tObdAccess access_p, tObdCbStoreParam MEM* pCbStore_p,
                                   void MEM* pObjData_p, tObdSize objSize_p);
static tOplkError   callStoreCallback(tObdCbStoreParam MEM* pCbStoreParam_p);
#endif // (CONFIG_OBD_USE_STORE_RESTORE != FALSE)

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------

static tObdDataTypeSize dataTypeSize_l[] =
{
    {0,                             NULL },
    { sizeof(tObdBoolean),          NULL },             // 0001 BOOLEAN
    { sizeof(tObdInteger8),         NULL },             // 0002 INTEGER8
    { sizeof(tObdInteger16),        NULL },             // 0003 INTEGER16
    { sizeof(tObdInteger32),        NULL },             // 0004 INTEGER32
    { sizeof(tObdUnsigned8),        NULL },             // 0005 UNSIGNED8
    { sizeof(tObdUnsigned16),       NULL },             // 0006 UNSIGNED16
    { sizeof(tObdUnsigned32),       NULL },             // 0007 UNSIGNED32
    { sizeof(tObdReal32),           NULL },             // 0008 REAL32
    { 0,                            getVstringSize },   // 0009 VISIBLE_STRING
    { 0,                            getOstringSize },   // 000A OCTET_STRING
    { 0,                            NULL },             // 000B UNICODE_STRING - not supported
    { 6,                            NULL },             // 000C TIME_OF_DAY
    { 6,                            NULL },             // 000D TIME_DIFFERENCE
    { 0,                            NULL },             // 000E not specified
    { 0,                            getDomainSize },    // 000F DOMAIN
    { 3,                            NULL },             // 0010 INTEGER24
    { sizeof(tObdReal64),           NULL },             // 0011 REAL64
    { 5,                            NULL },             // 0012 INTEGER40
    { 6,                            NULL },             // 0013 INTEGER48
    { 7,                            NULL },             // 0014 INTEGER56
    { sizeof(tObdInteger64),        NULL },             // 0015 INTEGER64
    { 3,                            NULL },             // 0016 UNSIGNED24
    { 0,                            NULL },             // 0017 not specified
    { 5,                            NULL },             // 0018 UNSIGNED40
    { 6,                            NULL },             // 0019 UNSIGNED48
    { 7,                            NULL },             // 001A UNSIGNED56
    { sizeof(tObdUnsigned64),       NULL },             // 001B UNSIGNED64
};

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Initialize OD module

The function initializes the OD module.

\param  pInitParam_p            Pointer to OD initialization parameters.

\return The function returns a tOplkError error code.

\ingroup module_obd
*/
//------------------------------------------------------------------------------
tOplkError obd_init(tObdInitParam MEM* pInitParam_p)
{
    tOplkError      ret;

    if (pInitParam_p == NULL)
        return kErrorOk;

    OPLK_MEMCPY(&obdInstance_l.initParam, pInitParam_p, sizeof(tObdInitParam));

    // clear callback function for command LOAD and STORE
    obdInstance_l.pfnStoreLoadObjectCb = NULL;

#if (CONFIG_OBD_CALC_OD_SIGNATURE != FALSE)
    OPLK_MEMSET(obdInstance_l.aOdSignature, -1, sizeof(obdInstance_l.aOdSignature));
#endif

    calcOdIndexNum(&obdInstance_l.initParam);

    // initialize object dictionary
    // so all all VarEntries will be initialized to trash object and default values will be set to current data
    ret = obd_accessOdPart(kObdPartAll, kObdDirInit);

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Shutdown OD module

The function shuts down the OD module.

\return The function returns a tOplkError error code.

\ingroup module_obd
*/
//------------------------------------------------------------------------------
tOplkError obd_exit(void)
{
    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Write OD entry

The function writes data to an OD entry. Strings are stored with added '\0'
character.

\param      index_p         Index to write.
\param      subIndex_p      Sub-index to write.
\param      pSrcData_p      Pointer to data which should be written.
\param      size_p          Size of data to write.

\return The function returns a tOplkError error code.

\ingroup module_obd
*/
//------------------------------------------------------------------------------
tOplkError obd_writeEntry(UINT index_p, UINT subIndex_p, void* pSrcData_p, tObdSize size_p)
{
    tOplkError              ret;
    tObdEntryPtr            pObdEntry;
    tObdSubEntryPtr         pSubEntry;
    tObdCbParam MEM         cbParam;
    void MEM*               pDstData;
    tObdSize                obdSize;

    ret = writeEntryPre(index_p, subIndex_p, pSrcData_p, &pDstData, size_p,
                        &pObdEntry, &pSubEntry, &cbParam, &obdSize);
    if (ret != kErrorOk)
        return ret;

    ret = writeEntryPost(pObdEntry, pSubEntry, &cbParam, pSrcData_p, pDstData, obdSize);
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Read OD entry

The function reads an OD entry. The object can always be read, even if the
attribute kObdAccRead is not set. The attribute is only checked for SDO
transfers.

\param      index_p         Index to read.
\param      subIndex_p      Sub-index to read.
\param      pDstData_p      Pointer to store the read data.
\param      pSize_p         Pointer to size of buffer. The real data size will
                            be written to this location.

\return The function returns a tOplkError error code.

\ingroup module_obd
*/
//------------------------------------------------------------------------------
tOplkError obd_readEntry(UINT index_p, UINT subIndex_p, void* pDstData_p, tObdSize* pSize_p)
{
    tOplkError                      ret;
    tObdEntryPtr                    pObdEntry;
    tObdSubEntryPtr                 pSubEntry;
    tObdCbParam  MEM                cbParam;
    void*                           pSrcData;
    tObdSize                        obdSize;

    if ((pDstData_p == NULL) || (pSize_p == NULL))
        return kErrorInvalidInstanceParam;

    ret = getEntry(index_p, subIndex_p, &pObdEntry, &pSubEntry);
    if (ret != kErrorOk)
        return ret;

     pSrcData = getObjectDataPtr(pSubEntry);

    // check source pointer
    if (pSrcData == NULL)
        return kErrorObdReadViolation;

    // address of source data to structure of callback parameters
    // so callback function can change this data before reading
    cbParam.index = index_p;
    cbParam.subIndex = subIndex_p;
    cbParam.pArg = pSrcData;
    cbParam.obdEvent = kObdEvPreRead;
    ret = callObjectCallback(pObdEntry->pfnCallback, &cbParam);
    if (ret != kErrorOk)
        return ret;

    // get size of data and check if application has reserved enough memory
    obdSize = getDataSize(pSubEntry);
    if (*pSize_p < obdSize)
        return kErrorObdValueLengthError;

    // read value from object
    OPLK_MEMCPY(pDstData_p, pSrcData, obdSize);
    if (pSubEntry->type == kObdTypeVString)
    {
        if (*pSize_p > obdSize)
        {   // space left to set the terminating null-character
            ((char MEM*)pDstData_p)[obdSize] = '\0';
            obdSize++;
        }
    }
    *pSize_p = obdSize;

    // write address of destination data to structure of callback parameters
    // so callback function can change this data after reading
    cbParam.pArg     = pDstData_p;
    cbParam.obdEvent = kObdEvPostRead;
    ret = callObjectCallback(pObdEntry->pfnCallback, &cbParam);
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Access part of OD

restores default values of one part of OD


\param      obdPart_p
\param      direction_p

\return The function returns a tOplkError error code.

\ingroup module_obd
*/
//------------------------------------------------------------------------------
tOplkError obd_accessOdPart(tObdPart obdPart_p, tObdDir direction_p)
{
    tOplkError      ret = kErrorOk;
    BOOL            fPartFount;
    tObdEntryPtr    pObdEntry;

    fPartFount = FALSE;

    pObdEntry = obdInstance_l.initParam.pGenericPart;
    if (((obdPart_p & kObdPartGen) != 0) && (pObdEntry != NULL))
    {
        fPartFount = TRUE;
        ret = accessOdPartition(kObdPartGen, pObdEntry, direction_p);
        if (ret != kErrorOk)
            return ret;
    }

    pObdEntry = obdInstance_l.initParam.pManufacturerPart;
    if (((obdPart_p & kObdPartMan) != 0) && (pObdEntry != NULL))
    {
        fPartFount = TRUE;
        ret = accessOdPartition(kObdPartMan, pObdEntry, direction_p);
        if (ret != kErrorOk)
            return ret;
    }

    pObdEntry = obdInstance_l.initParam.pDevicePart;
    if (((obdPart_p & kObdPartDev) != 0) && (pObdEntry != NULL))
    {
        fPartFount = TRUE;
        ret = accessOdPartition(kObdPartDev, pObdEntry, direction_p);
        if (ret != kErrorOk)
            return ret;
    }

#if (defined (OBD_USER_OD) && (OBD_USER_OD != FALSE))
    pObdEntry = obdInstance_l.initParam.m_pUserPart;
    if (((obdPart_p & kObdPartUsr) != 0) && (pObdEntry != NULL))
    {
        fPartFount = TRUE;
        ret = accessOdPartition (kObdPartUsr, pObdEntry, direction_p);
        if (ret != kErrorOk)
            return ret;
    }
#endif

    // no access to an OD part was done? illegal OD part was specified!
    if (fPartFount == FALSE)
        ret = kErrorObdIllegalPart;
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Define an OD variable

The function defines an OD variable.

\param  pVarParam_p             Pointer to the object variable structure.

\return The function returns a tOplkError error code.

\ingroup module_obd
*/
//------------------------------------------------------------------------------
tOplkError obd_defineVar(tVarParam MEM* pVarParam_p)
{
    tOplkError              ret;
    tObdVarEntry MEM*       pVarEntry;
    tVarParamValid          varValid;
    tObdSubEntryPtr         pSubIndexEntry;

    // get address of sub-index entry
    ret = getEntry(pVarParam_p->index, pVarParam_p->subindex, NULL, &pSubIndexEntry);
    if (ret != kErrorOk)
        return ret;

    // get var entry
    ret = getVarEntry(pSubIndexEntry, &pVarEntry);
    if (ret != kErrorOk)
        return ret;

    varValid =  pVarParam_p->validFlag;

    // copy only values for which the valid flag is set
    if ((varValid & kVarValidSize) != 0)
    {
        if (pSubIndexEntry->type != kObdTypeDomain)
        {
            tObdSize dataSize;

            // check passed size parameter
            dataSize = getObjectSize(pSubIndexEntry);
            if (dataSize != pVarParam_p->size)
            {   // size of variable does not match
                return kErrorObdValueLengthError;
            }
        }
        else
        {   // size can be set only for objects of type DOMAIN
            pVarEntry->size = pVarParam_p->size;
        }
    }

    if ((varValid & kVarValidData) != 0)
    {
       pVarEntry->pData = pVarParam_p->pData;
    }
    // ret is already set to kErrorOk from getVarEntry()
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Get current data pointer of object entry

The function returns the current data pointer. If object is a
constant object it returns the default pointer.

\param  index_p             Index of the entry.
\param  subIndex_p          Sub-index of the entry.

\return The function returns the pointer to the object data

\ingroup module_obd
*/
//------------------------------------------------------------------------------
void* obd_getObjectDataPtr(UINT index_p, UINT subIndex_p)
{
    tOplkError          ret;
    void*               pData;
    tObdEntryPtr        pObdEntry;
    tObdSubEntryPtr     pObdSubEntry;

    // get pointer to index structure
    ret = getIndex(&obdInstance_l.initParam, index_p, &pObdEntry);
    if (ret != kErrorOk)
    {
        pData = NULL;
        return pData;
    }

    ret = getSubindex(pObdEntry, subIndex_p, &pObdSubEntry);
    if (ret != kErrorOk)
    {
        pData = NULL;
        return pData;
    }
    pData = getObjectDataPtr(pObdSubEntry);
    return pData;
}

#if (defined (OBD_USER_OD) && (OBD_USER_OD != FALSE))
//------------------------------------------------------------------------------
/**
\brief  Register a user OD

The function registers a user object dictionary.

\param  pUserOd_p            Pointer to user OD.

\return The function returns a tOplkError error code.

\ingroup module_obd
*/
//------------------------------------------------------------------------------
tOplkError obd_registerUserOd(tObdEntryPtr pUserOd_p)
{
    obdInitParam_l.m_pUserPart = pUserOd_p;
    return kErrorOk;
}
#endif

//------------------------------------------------------------------------------
/**
\brief  Initialize VarEntry

The function initializes the VarEntry dependant on the object type.
The function will not be used for strings.

\param  pVarEntry_p             Pointer to VarEntry structure.
\param  type_p                  Object type.
\param  obdSize_p               Size of object data.

\ingroup module_obd
*/
//------------------------------------------------------------------------------
void obd_initVarEntry(tObdVarEntry MEM* pVarEntry_p, tObdType type_p, tObdSize obdSize_p)
{
    if ((type_p == kObdTypeDomain))
    {
        // variables which are defined as DOMAIN or VSTRING should not point to
        // trash object, because this trash object contains only 8 bytes. DOMAINS or
        // STRINGS can be longer.
        pVarEntry_p->pData = NULL;
        pVarEntry_p->size  = 0;
    }
    else
    {
        // set address to variable data to trash object
        // This prevents an access violation if user forgets to call obd_defineVar()
        // for this variable but mappes it in a PDO.
        pVarEntry_p->pData = &obdInstance_l.obdTrashObject[0];
        pVarEntry_p->size  = obdSize_p;
    }
}

//------------------------------------------------------------------------------
/**
\brief  Get data size

The function gets the data size of an object. For string objects it returns the
string length without terminating null-character.

\param  index_p                 Index of object.
\param  subIndex_p              Sub-index of object.

\return The function returns the data size.

\ingroup module_obd
*/
//------------------------------------------------------------------------------
tObdSize obd_getDataSize(UINT index_p, UINT subIndex_p)
{
    tOplkError          ret;
    tObdSize            obdSize;
    tObdEntryPtr        pObdEntry;
    tObdSubEntryPtr     pObdSubEntry;

    ret = getIndex(&obdInstance_l.initParam, index_p, &pObdEntry);
    if (ret != kErrorOk)
    {
        obdSize = 0;
        return obdSize;
    }

    ret = getSubindex(pObdEntry, subIndex_p, &pObdSubEntry);
    if (ret != kErrorOk)
    {
        obdSize = 0;
        return obdSize;
    }

    obdSize = getDataSize(pObdSubEntry);
    return obdSize;
}

//------------------------------------------------------------------------------
/**
\brief  Get node ID

The function gets the node ID which is stored in object 0x1F93.

\return The function returns the node ID.

\ingroup module_obd
*/
//------------------------------------------------------------------------------
UINT obd_getNodeId(void)
{
    tOplkError      ret;
    tObdSize        obdSize;
    UINT8           nodeId;

    nodeId = 0;
    obdSize = sizeof(nodeId);
    ret = obd_readEntry(OBD_NODE_ID_INDEX, OBD_NODE_ID_SUBINDEX, &nodeId, &obdSize);
    if (ret != kErrorOk)
    {
        nodeId = C_ADR_INVALID;
    }
    return (UINT) nodeId;
}

//------------------------------------------------------------------------------
/**
\brief  Set node ID

The function sets the node ID in object 0x1F93.

\param  nodeId_p            Node ID to set.
\param  nodeIdType_p        Node ID setting type.

\return The function returns a tOplkError error code.

\ingroup module_obd
*/
//------------------------------------------------------------------------------
tOplkError obd_setNodeId(UINT nodeId_p, tObdNodeIdType nodeIdType_p)
{
    tOplkError  ret;
    tObdSize    obdSize;
    UINT8       fHwBool;
    UINT8       nodeId;

    if (nodeId_p == C_ADR_INVALID)
        return kErrorInvalidNodeId;

    nodeId = (UINT8)nodeId_p;
    obdSize = sizeof(UINT8);
    ret = obd_writeEntry(OBD_NODE_ID_INDEX, OBD_NODE_ID_SUBINDEX, &nodeId, obdSize);
    if (ret != kErrorOk)
        return ret;

    // set HWBOOL-Flag in Sub-index OBD_NODE_ID_HWBOOL_SUBINDEX
    switch (nodeIdType_p)
    {
        // type unknown
        case kObdNodeIdUnknown:
            fHwBool = OBD_FALSE;
            break;

        case kObdNodeIdSoftware:
            fHwBool = OBD_FALSE;
            break;

        case kObdNodeIdHardware:
            fHwBool = OBD_TRUE;
            break;

        default:
            fHwBool = OBD_FALSE;
            break;
    }

    obdSize = sizeof(fHwBool);
    ret = obd_writeEntry(OBD_NODE_ID_INDEX, OBD_NODE_ID_HWBOOL_SUBINDEX, &fHwBool, obdSize);
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Check for numeric entries

The function checks if a entry is numerical or not.

\param  index_p                 Index of object to check.
\param  subIndex_p              Sub-index of object to check.
\param  pfEntryNumerical_p      Pointer to store result. TRUE if entry is numerical,
                                FALSE if entry is not numerical.

\return The function returns a tOplkError error code.

\ingroup module_obd
*/
//------------------------------------------------------------------------------
tOplkError obd_isNumerical(UINT index_p, UINT subIndex_p, BOOL* pfEntryNumerical_p)
{
    tOplkError          ret;
    tObdEntryPtr        pObdEntry;
    tObdSubEntryPtr     pObdSubEntry;

    ret = getIndex(&obdInstance_l.initParam, index_p, &pObdEntry);
    if (ret != kErrorOk)
        return ret;

    // get pointer to sub-index structure
    ret = getSubindex(pObdEntry, subIndex_p, &pObdSubEntry);
    if (ret != kErrorOk)
        return ret;

    ret = isNumerical(pObdSubEntry, pfEntryNumerical_p);
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Get type of object

The function returns the data type of the specified entry.

\param  index_p                 Index of object to check.
\param  subIndex_p              Sub-index of object to check.
\param  pType_p                 Pointer to store the type of the entry.

\return The function returns a tOplkError error code.

\ingroup module_obd
*/
//------------------------------------------------------------------------------
tOplkError obd_getType(UINT index_p, UINT subIndex_p, tObdType* pType_p)
{
    tOplkError          ret;
    tObdEntryPtr        pObdEntry;
    tObdSubEntryPtr     pObdSubEntry;

    ret = getIndex(&obdInstance_l.initParam, index_p, &pObdEntry);
    if (ret != kErrorOk)
        return ret;

    ret = getSubindex(pObdEntry, subIndex_p, &pObdSubEntry);
    if (ret != kErrorOk)
        return ret;

    *pType_p = pObdSubEntry->type;
    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Read entry and convert it to little endian

The function reads an object entry and converts numerical types into the little
endian byte order for numerical types. For other types a normal read will be
performed. This is useful for the PDO and SDO module.

The application can always read the data even if the attribute kObdAccRead is
not set. The attribute is only checked on SDO transfers.

\param  index_p                 Index of object to read.
\param  subIndex_p              Sub-index of object to read.
\param  pDstData_p              Pointer to location where to store the read data.
\param  pSize_p                 Pointer to the size of the buffer. The function
                                stores the number of read bytes at this
                                location.

\return The function returns a tOplkError error code.

\ingroup module_obd
*/
//------------------------------------------------------------------------------
tOplkError obd_readEntryToLe(UINT index_p, UINT subIndex_p, void* pDstData_p,
                             tObdSize* pSize_p)
{
    tOplkError                      ret;
    tObdEntryPtr                    pObdEntry;
    tObdSubEntryPtr                 pSubEntry;
    tObdCbParam  MEM                cbParam;
    void*                           pSrcData;
    tObdSize                        obdSize;

    ret = getEntry(index_p, subIndex_p, &pObdEntry, &pSubEntry);
    if (ret != kErrorOk)
        return ret;

    pSrcData = getObjectDataPtr(pSubEntry);
    if (pSrcData == NULL)
        return kErrorObdReadViolation;

    // address of source data to structure of callback parameters
    // so callback function can change this data before reading
    cbParam.index =     index_p;
    cbParam.subIndex =  subIndex_p;
    cbParam.pArg =      pSrcData;
    cbParam.obdEvent =  kObdEvPreRead;
    ret = callObjectCallback(pObdEntry->pfnCallback, &cbParam);
    if (ret != kErrorOk)
        return ret;

    // get size of data and check if application has reserved enough memory
    obdSize = getDataSize(pSubEntry);
    if (*pSize_p < obdSize)
        return kErrorObdValueLengthError;

    // check if numerical type
    switch (pSubEntry->type)
    {
        case kObdTypeVString:
        case kObdTypeOString:
        case kObdTypeDomain:
        default:
            OPLK_MEMCPY(pDstData_p, pSrcData, obdSize);
            if (pSubEntry->type == kObdTypeVString)
            {
                if (*pSize_p > obdSize)
                {   // space left to set the terminating null-character
                    ((char MEM*)pDstData_p)[obdSize] = '\0';
                    obdSize++;
                }
            }
            break;

        case kObdTypeBool:
        case kObdTypeInt8:
        case kObdTypeUInt8:
            ami_setUint8Le(pDstData_p, *((UINT8*)pSrcData));
            break;

        case kObdTypeInt16:
        case kObdTypeUInt16:
            ami_setUint16Le(pDstData_p, *((UINT16*)pSrcData));
            break;

        case kObdTypeInt24:
        case kObdTypeUInt24:
            ami_setUint24Le(pDstData_p, *((UINT32*)pSrcData));
            break;

        case kObdTypeInt32:
        case kObdTypeUInt32:
        case kObdTypeReal32:
            ami_setUint32Le(pDstData_p, *((UINT32*)pSrcData));
            break;

        case kObdTypeInt40:
        case kObdTypeUInt40:
            ami_setUint40Le(pDstData_p, *((UINT64*)pSrcData));
            break;

        case kObdTypeInt48:
        case kObdTypeUInt48:
            ami_setUint48Le(pDstData_p, *((UINT64*)pSrcData));
            break;

        case kObdTypeInt56:
        case kObdTypeUInt56:
            ami_setUint56Le(pDstData_p, *((UINT64*)pSrcData));
            break;

        case kObdTypeInt64:
        case kObdTypeUInt64:
        case kObdTypeReal64:
            ami_setUint64Le(pDstData_p, *((UINT64*)pSrcData));
            break;

        // time of day
        case kObdTypeTimeOfDay:
        case kObdTypeTimeDiff:
            ami_setTimeOfDay(pDstData_p, ((tTimeOfDay*)pSrcData));
            break;
    }

    *pSize_p = obdSize;

    // write address of destination data to structure of callback parameters
    // so callback function can change this data after reading
    cbParam.pArg     = pDstData_p;
    cbParam.obdEvent = kObdEvPostRead;
    ret = callObjectCallback(pObdEntry->pfnCallback, &cbParam);
    return ret;

}

//------------------------------------------------------------------------------
/**
\brief  Write entry and convert it from little endian

The function writes an object entry and converts numerical types from the little
endian byte order into the system byte order. For other types a normal write will
be performed. Strings are stored with added '\0' character.

\param  index_p                 Index of object to write.
\param  subIndex_p              Sub-index of object to write.
\param  pSrcData_p              Pointer to the data which should be written.
\param  size_p                  Size of the data to be written.

\return The function returns a tOplkError error code.

\ingroup module_obd
*/
//------------------------------------------------------------------------------
tOplkError obd_writeEntryFromLe(UINT index_p, UINT subIndex_p, void* pSrcData_p,
                                tObdSize size_p)
{
    tOplkError              ret;
    tObdEntryPtr            pObdEntry;
    tObdSubEntryPtr         pSubEntry;
    tObdCbParam MEM         cbParam;
    void MEM*               pDstData;
    tObdSize                obdSize;
    UINT64                  buffer;
    void*                   pBuffer = &buffer;

    ret = writeEntryPre(index_p, subIndex_p, pSrcData_p, &pDstData, size_p,
                        &pObdEntry, &pSubEntry, &cbParam, &obdSize);
    if (ret != kErrorOk)
        return ret;

    switch (pSubEntry->type)
    {
        case kObdTypeBool:
        case kObdTypeInt8:
        case kObdTypeUInt8:
            *((UINT8*)pBuffer) = ami_getUint8Le(pSrcData_p);
            break;

        case kObdTypeInt16:
        case kObdTypeUInt16:
            *((UINT16*)pBuffer) = ami_getUint16Le(pSrcData_p);
            break;

        case kObdTypeInt24:
        case kObdTypeUInt24:
            *((UINT32*)pBuffer) = ami_getUint24Le(pSrcData_p);
            break;

        case kObdTypeInt32:
        case kObdTypeUInt32:
        case kObdTypeReal32:
            *((UINT32*)pBuffer) = ami_getUint32Le(pSrcData_p);
            break;

        case kObdTypeInt40:
        case kObdTypeUInt40:
            *((UINT64*)pBuffer) = ami_getUint40Le(pSrcData_p);
            break;

        case kObdTypeInt48:
        case kObdTypeUInt48:
            *((UINT64*)pBuffer) = ami_getUint48Le(pSrcData_p);
            break;

        case kObdTypeInt56:
        case kObdTypeUInt56:
            *((UINT64*)pBuffer) = ami_getUint56Le(pSrcData_p);
            break;

        case kObdTypeInt64:
        case kObdTypeUInt64:
        case kObdTypeReal64:
            *((UINT64*)pBuffer) = ami_getUint64Le(pSrcData_p);
            break;

        case kObdTypeTimeOfDay:
        case kObdTypeTimeDiff:
            ami_getTimeOfDay(pBuffer, ((tTimeOfDay*)pSrcData_p));
            break;

        default:
            // do nothing, i.e. use the given source pointer
            pBuffer = pSrcData_p;
            break;
    }

    ret = writeEntryPost(pObdEntry, pSubEntry, &cbParam, pBuffer, pDstData, obdSize);
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Get access type of an entry

The function gets the access type of the entry.

\param  index_p                 Index of object.
\param  subIndex_p              Sub-index of object.
\param  pAccessType_p           Pointer to store the access type.

\return The function returns a tOplkError error code.

\ingroup module_obd
*/
//------------------------------------------------------------------------------
tOplkError obd_getAccessType(UINT index_p, UINT subIndex_p, tObdAccess* pAccessType_p)
{
    tOplkError          ret;
    tObdEntryPtr        pObdEntry;
    tObdSubEntryPtr     pObdSubEntry;

    ret = getIndex(&obdInstance_l.initParam, index_p, &pObdEntry);
    if (ret != kErrorOk)
        return ret;

    ret = getSubindex(pObdEntry, subIndex_p, &pObdSubEntry);
    if (ret != kErrorOk)
        return ret;

    *pAccessType_p = pObdSubEntry->access;
    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Get VarEntry structure of object

The function gets the VarEntry structure of an object.

\param  index_p                 Index of object.
\param  subIndex_p              Sub-index of object.
\param  ppVarEntry_p            Pointer to store pointer to the VarEntry structure.

\return The function returns a tOplkError error code.

\ingroup module_obd
*/
//------------------------------------------------------------------------------
tOplkError obd_searchVarEntry(UINT index_p, UINT subIndex_p, tObdVarEntry MEM** ppVarEntry_p)
{
    tOplkError           ret;
    tObdSubEntryPtr      pSubIndexEntry;

    ret = getEntry(index_p, subIndex_p, NULL, &pSubIndexEntry);
    if (ret == kErrorOk)
    {
        ret = getVarEntry(pSubIndexEntry, ppVarEntry_p);
    }
    return ret;
}

#if (CONFIG_OBD_CALC_OD_SIGNATURE != FALSE)
//------------------------------------------------------------------------------
/**
\brief  Get OD part archive signature

The function reads the OD signature for checking valid OD part in Store/Restore
archive file.

\param  odPart_p        The OD part specifier.

\return The function returns the OD part archive signature.
\retVal UINT32_MAX      The OD part parameter is invalid

\ingroup module_obd
*/
//------------------------------------------------------------------------------
UINT32 obd_getOdSignature(tObdPart odPart_p)
{
    UINT32 odCrc = (UINT32)~0U;

    switch (odPart_p)
    {
        case kObdPartGen:
            odCrc = obdInstance_l.aOdSignature[0];
            break;
        case kObdPartMan:
            odCrc = obdInstance_l.aOdSignature[1];
            break;
        case kObdPartDev:
            odCrc = obdInstance_l.aOdSignature[2];
            break;
        default:
            break;
    }

    return odCrc;
}
#endif

#if (CONFIG_OBD_USE_STORE_RESTORE != FALSE)
//------------------------------------------------------------------------------
/**
\brief  Set callback function for load/store command

The function sets the callback function for the load/store command.

\param  pfnCallback_p           Pointer to the callback function.

\return The function returns a tOplkError error code.

\ingroup module_obd
*/
//------------------------------------------------------------------------------
tOplkError obd_storeLoadObjCallback(tObdStoreLoadCallback pfnCallback_p)
{
    // set new address of callback function
    obdInstance_l.pfnStoreLoadObjectCb = pfnCallback_p;
    return kErrorOk;
}
#endif // (CONFIG_OBD_USE_STORE_RESTORE != FALSE)

//------------------------------------------------------------------------------
/**
\brief  Initialize write to OD

The function initializes write of data to an OBD entry.
It is used by SDO command layer to store segmented data.

\param  index_p                 Index of object.
\param  subIndex_p              Sub-index of object.
\param  ppDstData_p             Pointer to store object data pointer.
\param  size_p                  Size of the data to be written.

\return The function returns a tOplkError error code.

\ingroup module_obd
*/
//------------------------------------------------------------------------------
tOplkError obd_initWrite(UINT index_p, UINT subIndex_p, void** ppDstData_p,
                         tObdSize size_p)
{
    tOplkError              ret;
    tObdEntryPtr            pObdEntry;
    tObdSubEntryPtr         pSubEntry;
    tObdAccess              access;
    void MEM*               pDstData;
    tObdSize                obdSize;
    tObdCbParam             cbParam;

#if (CONFIG_OBD_USE_STRING_DOMAIN_IN_RAM != FALSE)
    tObdVStringDomain MEM   memVStringDomain;
    void MEM*               pCurrData;
#endif

    ret = getEntry(index_p, subIndex_p, &pObdEntry, &pSubEntry);
    if (ret != kErrorOk)
        return ret;

    access = (tObdAccess)pSubEntry->access;
    // check access for write
    if ((access & kObdAccConst) != 0)
        return kErrorObdAccessViolation;

    // To use the same callback function for ObdWriteEntry as well as for
    // an SDO download call at first (kObdEvPre...) the callback function
    // with the argument pointer to object size.
    cbParam.index    = index_p;
    cbParam.subIndex = subIndex_p;

    // Because object size and object pointer are adapted by user callback
    // function, re-read this values.
    obdSize = getObjectSize(pSubEntry);
    pDstData = (void MEM*)getObjectDataPtr(pSubEntry);

    // Function obd_writeEntry() calls event kObdEvWrStringDomain for String or
    // Domain which lets called module directly change the data pointer or size.
    // This prevents a recursive call to the callback function if it calls getEntry().
#if (CONFIG_OBD_USE_STRING_DOMAIN_IN_RAM != FALSE)
    if ((pSubEntry->type == kObdTypeVString) || (pSubEntry->type == kObdTypeDomain) ||
        (pSubEntry->type == kObdTypeOString))
    {
        if (pSubEntry->type == kObdTypeVString)
        {
            // reserve one byte for 0-termination
            size_p += 1;
        }

        memVStringDomain.downloadSize = size_p;
        memVStringDomain.objSize      = obdSize;
        memVStringDomain.pData        = pDstData;
        cbParam.obdEvent = kObdEvWrStringDomain;
        cbParam.pArg     = &memVStringDomain;
        ret = callObjectCallback(pObdEntry->pfnCallback, &cbParam);
        if (ret != kErrorOk)
            return ret;

        // write back new settings
        pCurrData = pSubEntry->pCurrent;
        if ((pSubEntry->type == kObdTypeVString) || (pSubEntry->type == kObdTypeOString))
        {
            ((tObdVString MEM*)pCurrData)->size    = memVStringDomain.objSize;
            ((tObdVString MEM*)pCurrData)->pString = memVStringDomain.pData;
        }
        else
        {
            tObdVarEntry MEM*    pVarEntry = NULL;

            ret = getVarEntry(pSubEntry, &pVarEntry);
            if (ret != kErrorOk)
                return ret;

            if (pVarEntry == NULL)
            {
                return kErrorObdAccessViolation;
            }
            pVarEntry->size  = memVStringDomain.objSize;
            pVarEntry->pData = (void MEM*)memVStringDomain.pData;
        }

        // Because object size and object pointer are adapted by user callback
        // function, re-read this values.
        obdSize  = memVStringDomain.objSize;
        pDstData = (void MEM*)memVStringDomain.pData;
    }
#endif

    // access violation if adress to current value is NULL
    if (pDstData == NULL)
       return kErrorObdAccessViolation;

    cbParam.pArg     = &obdSize;
    cbParam.obdEvent = kObdEvInitWrite;
    ret = callObjectCallback(pObdEntry->pfnCallback, &cbParam);
    if (ret != kErrorOk)
        return ret;

    if (size_p > obdSize)
        return kErrorObdValueLengthError;

    if (ppDstData_p)
        *ppDstData_p = pDstData;

    return kErrorOk;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief  Prepare writes to OD

The function prepares write of data to an OBD entry. Strings are stored with
added '\0' character.

\param  index_p                 Index of object.
\param  subIndex_p              Sub-index of object.
\param  pSrcData_p              Points to the data which should be written.
\param  ppDstData_p             Pointer to store object data pointer.
\param  size_p                  Size of the data to be written.
\param  ppObdEntry_p            Pointer to store pointer to object entry.
\param  ppSubEntry_p            Pointer to store pointer to sub-index entry.
\param  pCbParam_p              Points to the callback parameter structure.
\param  pObdSize_p              Pointer to store size of the object.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError writeEntryPre(UINT index_p, UINT subIndex_p, void* pSrcData_p,
                                void** ppDstData_p, tObdSize size_p, tObdEntryPtr* ppObdEntry_p,
                                tObdSubEntryPtr* ppSubEntry_p, tObdCbParam MEM* pCbParam_p,
                                tObdSize*  pObdSize_p)
{
    tOplkError              ret;
    tObdEntryPtr            pObdEntry;
    tObdSubEntryPtr         pSubEntry;
    tObdAccess              access;
    void MEM*               pDstData;
    tObdSize                obdSize;
    BOOL                    fEntryNumerical;

#if (CONFIG_OBD_USE_STRING_DOMAIN_IN_RAM != FALSE)
    tObdVStringDomain MEM   memVStringDomain;
    void MEM*               pCurrData;
#endif

    ret = getEntry(index_p, subIndex_p, &pObdEntry, &pSubEntry);
    if (ret != kErrorOk)
        return ret;

    access = (tObdAccess)pSubEntry->access;
    // check access for write
    if ((access & kObdAccConst) != 0)
        return kErrorObdAccessViolation;

    // To use the same callback function for ObdWriteEntry as well as for
    // an SDO download call at first (kObdEvPre...) the callback function
    // with the argument pointer to object size.
    pCbParam_p->index    = index_p;
    pCbParam_p->subIndex = subIndex_p;

    // Because object size and object pointer are adapted by user callback
    // function, re-read this values.
    obdSize = getObjectSize(pSubEntry);
    pDstData = (void MEM*)getObjectDataPtr(pSubEntry);

    // Function obd_writeEntry() calls event kObdEvWrStringDomain for String or
    // Domain which lets called module directly change the data pointer or size.
    // This prevents a recursive call to the callback function if it calls getEntry().
#if (CONFIG_OBD_USE_STRING_DOMAIN_IN_RAM != FALSE)
    if ((pSubEntry->type == kObdTypeVString) || (pSubEntry->type == kObdTypeDomain) ||
        (pSubEntry->type == kObdTypeOString))
    {
        if (pSubEntry->type == kObdTypeVString)
        {
            // reserve one byte for 0-termination
            size_p += 1;
        }

        memVStringDomain.downloadSize = size_p;
        memVStringDomain.objSize      = obdSize;
        memVStringDomain.pData        = pDstData;
        pCbParam_p->obdEvent = kObdEvWrStringDomain;
        pCbParam_p->pArg     = &memVStringDomain;
        ret = callObjectCallback(pObdEntry->pfnCallback, pCbParam_p);
        if (ret != kErrorOk)
            return ret;

        // write back new settings
        pCurrData = pSubEntry->pCurrent;
        if ((pSubEntry->type == kObdTypeVString) || (pSubEntry->type == kObdTypeOString))
        {
            ((tObdVString MEM*)pCurrData)->size    = memVStringDomain.objSize;
            ((tObdVString MEM*)pCurrData)->pString = memVStringDomain.pData;
        }
        else
        {
            tObdVarEntry MEM*    pVarEntry = NULL;

            ret = getVarEntry(pSubEntry, &pVarEntry);
            if (ret != kErrorOk)
                return ret;

            if (pVarEntry == NULL)
            {
                return kErrorObdAccessViolation;
            }
            pVarEntry->size  = memVStringDomain.objSize;
            pVarEntry->pData = (void MEM*)memVStringDomain.pData;
        }

        // Because object size and object pointer are adapted by user callback
        // function, re-read this values.
        obdSize  = memVStringDomain.objSize;
        pDstData = (void MEM*)memVStringDomain.pData;
    }
#endif

    // access violation if adress to current value is NULL
    if (pDstData == NULL)
       return kErrorObdAccessViolation;

    pCbParam_p->pArg     = &obdSize;
    pCbParam_p->obdEvent = kObdEvInitWrite;
    ret = callObjectCallback(pObdEntry->pfnCallback, pCbParam_p);
    if (ret != kErrorOk)
        return ret;

    if (size_p > obdSize)
        return kErrorObdValueLengthError;

    if (pSubEntry->type == kObdTypeVString)
    {
        if (((char MEM*)pSrcData_p)[size_p - 1] == '\0')
        {   // last byte of source string contains null character
            // reserve one byte in destination for 0-termination
            size_p -= 1;
        }
        else if (size_p >= obdSize)
        {   // source string is not 0-terminated and destination buffer is too short
            return kErrorObdValueLengthError;
        }
    }

    ret = isNumerical(pSubEntry, &fEntryNumerical);
    if (ret != kErrorOk)
        return ret;

    if ((fEntryNumerical != FALSE) && (size_p != obdSize))
    {
        return kErrorObdValueLengthError;     // type is numerical, therefore size has to fit, but it does not.
    }

    obdSize = size_p;   // use given size, because non-numerical objects can be written with shorter values

    // set output parameters
    *pObdSize_p = obdSize;
    *ppObdEntry_p = pObdEntry;
    *ppSubEntry_p = pSubEntry;
    *ppDstData_p = pDstData;

    // all checks are done
    // the caller may now convert the numerical source value to platform byte order in a temporary buffer
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Finish writes to OD

The function finishes write of data to an OBD entry. Strings are stored with
added '\0' character.

\param  pObdEntry_p            Pointer to object entry.
\param  pSubEntry_p            Pointer to sub-index entry.
\param  pCbParam_p             Points to the callback parameter structure.
\param  pSrcData_p             Points to the data which should be written.
\param  pDstData_p             Pointer where to store the data.
\param  obdSize_p              The Size of the object.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError writeEntryPost(tObdEntryPtr pObdEntry_p, tObdSubEntryPtr pSubEntry_p,
                                 tObdCbParam MEM* pCbParam_p, void* pSrcData_p,
                                 void* pDstData_p, tObdSize obdSize_p)
{
    tOplkError              ret;

    // caller converted the source value to platform byte order
    // now the range of the value may be checked
#if (CONFIG_OBD_CHECK_OBJECT_RANGE != FALSE)
    {
        ret = checkObjectRange(pSubEntry_p, pSrcData_p);
        if (ret != kErrorOk)
            return ret;
    }
#endif

    // now call user callback function to check value write address of source data
    // to structure of callback parameters so callback function can check this data.
    pCbParam_p->pArg     = pSrcData_p;
    pCbParam_p->obdEvent = kObdEvPreWrite;
    ret = callObjectCallback(pObdEntry_p->pfnCallback, pCbParam_p);
    if (ret != kErrorOk)
        return ret;

    // copy object data to OBD
    OPLK_MEMCPY(pDstData_p, pSrcData_p, obdSize_p);

    if (pSubEntry_p->type == kObdTypeVString)
    {
        ((char MEM*)pDstData_p)[obdSize_p] = '\0';
    }

    // write address of destination to structure of callback parameters
    // so callback function can change data subsequently
    pCbParam_p->pArg     = pDstData_p;
    pCbParam_p->obdEvent = kObdEvPostWrite;
    ret = callObjectCallback(pObdEntry_p->pfnCallback, pCbParam_p);
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Get data size of an object

The function gets the data size of an object. For string objects it returns
the string length without the terminating null-character.

\param  pSubIndexEntry_p        Pointer to the sub-index entry of the object.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tObdSize getDataSize(tObdSubEntryPtr pSubIndexEntry_p)
{
    tObdSize    dataSize;
    void MEM*   pData;

    // If OD entry is defined by macro OBD_SUBINDEX_ROM_VSTRING
    // then the current pointer is always NULL. The function
    // returns the length of default string.
    dataSize = getObjectSize(pSubIndexEntry_p);

    if (pSubIndexEntry_p->type == kObdTypeVString)
    {
        // The pointer to current value can be received from getObjectCurrentPtr()
        pData = ((void MEM*)getObjectCurrentPtr(pSubIndexEntry_p));
        if (pData != NULL)
        {
            dataSize = getObdStringLen((void *)pData, dataSize, pSubIndexEntry_p->type);
        }
    }
    return dataSize;
}

//------------------------------------------------------------------------------
/**
\brief  Get length of string

The function calculates the length of string. The '\0' character is NOT
included.

\param  pObjData_p          The pointer to the object.
\param  objLen_p            The maximum length of the object.
\param  objType_p           The type of the object (VSTRING, ...)

\return The function returns the size of the string.
*/
//------------------------------------------------------------------------------
static tObdSize getObdStringLen(void* pObjData_p, tObdSize objLen_p, tObdType objType_p)
{
    tObdSize        strLen = 0;
    UINT8*          pString;

    if (pObjData_p == NULL)
        return 0;

    // Visible String: data format byte
    if (objType_p == kObdTypeVString)
    {
        pString = pObjData_p;
        for (strLen = 0; strLen < objLen_p; strLen++)
        {
            if (*pString == '\0')
                break;
            pString++;
        }
    }
    else
    {
        // all other types
        strLen = 0;
    }
    return strLen;
}

//------------------------------------------------------------------------------
/**
\brief  Get size of domain object

The function returns the size of a domain object.

\param  pSubIndexEntry_p        Pointer to sub-index entry.

\return The function returns the size of the object.
*/
//------------------------------------------------------------------------------
static tObdSize getDomainSize(tObdSubEntryPtr pSubIndexEntry_p)
{
    tObdSize                dataSize = 0;
    tObdVarEntry MEM*       pVarEntry = NULL;
    tOplkError              ret;

    ret = getVarEntry(pSubIndexEntry_p, &pVarEntry);
    if ((ret == kErrorOk) && (pVarEntry != NULL))
    {
        dataSize = pVarEntry->size;
    }
    return dataSize;
}

//------------------------------------------------------------------------------
/**
\brief  Get size of VSTRING object

The function returns the size of an VSTRING object.

\param  pSubIndexEntry_p        Pointer to sub-index entry.

\return The function returns the size of the object.
*/
//------------------------------------------------------------------------------
static tObdSize getVstringSize(tObdSubEntryPtr pSubIndexEntry_p)
{
    tObdSize                dataSize = 0;
    void*                   pData;

    // If OD entry is defined by macro OBD_SUBINDEX_ROM_VSTRING
    // then the current pointer is always NULL. The function
    // returns the length of default string.
    pData = (void*)pSubIndexEntry_p->pCurrent;
    if ((void MEM*)pData != (void MEM*)NULL)
    {
        // The max. size of strings defined by STRING-Macro is stored in
        // tObdVString of current value.
        // (types tObdVString, tObdOString and tObdUString has the same members)
        dataSize = ((tObdVString MEM*)pData)->size;
    }
    else
    {
        // The current position is not declared. The string
        // is located in ROM, therefore use default pointer.
        pData = (void*)pSubIndexEntry_p->pDefault;
        if ((CONST void ROM*)pData != (CONST void ROM*)NULL)
        {
           // The max. size of strings defined by STRING-Macro is stored in
           // tObdVString of default value.
           dataSize = ((CONST tObdVString ROM*)pData)->size;
        }
    }
    return dataSize;
}

//------------------------------------------------------------------------------
/**
\brief  Get size of OSTRING object

The function returns the size of an OSTRING object.

\param  pSubIndexEntry_p        Pointer to sub-index entry.

\return The function returns the size of the object.
*/
//------------------------------------------------------------------------------
static tObdSize getOstringSize(tObdSubEntryPtr pSubIndexEntry_p)
{
    tObdSize                dataSize = 0;
    void*                   pData;

    pData = (void*)pSubIndexEntry_p->pCurrent;
    if ((void MEM*)pData != (void MEM*)NULL)
    {
        // The max. size of strings defined by STRING-Macro is stored in
        // tObdVString of current value.
        // (types tObdVString, tObdOString and tObdUString has the same members)
        dataSize = ((tObdOString MEM*)pData)->size;
    }
    else
    {
        // The current position is not declared. The string
        // is located in ROM, therefore use default pointer.
        pData = (void*)pSubIndexEntry_p->pDefault;
        if ((CONST void ROM*)pData != (CONST void ROM*)NULL)
        {
           // The max. size of strings defined by STRING-Macro is stored in
           // tObdVString of default value.
           dataSize = ((CONST tObdOString ROM*)pData)->size;
        }
    }
    return dataSize;
}

//------------------------------------------------------------------------------
/**
\brief  Get size of object

The function returns the size of an object. For strings the function returns
the whole object size not the length of string.

\param  pSubIndexEntry_p        Pointer to sub-index entry.

\return The function returns the size of the object.
*/
//------------------------------------------------------------------------------
static tObdSize getObjectSize(tObdSubEntryPtr pSubIndexEntry_p)
{
    if (pSubIndexEntry_p->type >= kObdTypeMax)
        return 0;

    if (dataTypeSize_l[pSubIndexEntry_p->type].pfnGetObjSize == NULL)
    {
        return dataTypeSize_l[pSubIndexEntry_p->type].size;
    }
    else
    {
        return dataTypeSize_l[pSubIndexEntry_p->type].pfnGetObjSize(pSubIndexEntry_p);
    }
}

//------------------------------------------------------------------------------
/**
\brief  Get a variable entry of an object

The function returns the variable entry of an object.

\param  pSubIndexEntry_p        Pointer to sub-index entry.
\param  ppVarEntry_p            Pointer to store VarEntry pointer of object.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError getVarEntry(tObdSubEntryPtr pSubIndexEntry_p, tObdVarEntry MEM** ppVarEntry_p)
{
    tOplkError ret = kErrorObdVarEntryNotExist;

    // check VAR-Flag - only this object points to variables
    if ((pSubIndexEntry_p->access & kObdAccVar) != 0)
    {
        // check if object is an array
        if ((pSubIndexEntry_p->access & kObdAccArray) != 0)
        {
            *ppVarEntry_p = &((tObdVarEntry MEM*)pSubIndexEntry_p->pCurrent)[pSubIndexEntry_p->subIndex - 1];
        }
        else
        {
            *ppVarEntry_p = (tObdVarEntry MEM*)pSubIndexEntry_p->pCurrent;
        }
        ret = kErrorOk;
    }
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Get entry of an object

The function gets the entries of an object in the OD.

\param  index_p                 Index of object for which to get entries.
\param  subIndex_p              Sub-index of object for which to get entries.
\param  ppObdEntry_p            Pointer to store object entry pointer.
\param  ppObdSubEntry_p         Pointer to store sub-index entry pointer.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError getEntry(UINT index_p, UINT subIndex_p, tObdEntryPtr* ppObdEntry_p,
                           tObdSubEntryPtr* ppObdSubEntry_p)
{
    tObdEntryPtr            pObdEntry;
    tObdCbParam MEM         cbParam;
    tOplkError              ret;

    ret = getIndex(&obdInstance_l.initParam, index_p, &pObdEntry);
    if (ret != kErrorOk)
        return ret;

    ret = getSubindex(pObdEntry, subIndex_p, ppObdSubEntry_p);
    if (ret != kErrorOk)
        return ret;

    // call callback function to inform user/stack that an object will be searched
    // if the called module returns an error then we abort the searching with kErrorObdIndexNotExist
    cbParam.index = index_p;
    cbParam.subIndex = subIndex_p;
    cbParam.pArg = NULL;
    cbParam.obdEvent = kObdEvCheckExist;
    ret = callObjectCallback(pObdEntry->pfnCallback, &cbParam);
    if (ret != kErrorOk)
        return kErrorObdIndexNotExist;

    // it is allowed to set ppObdEntry_p to NULL
    // if so, no address will be written to calling function
    if (ppObdEntry_p != NULL)
    {
        *ppObdEntry_p = pObdEntry;
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Get default pointer of object

The function returns the default pointer of the object entry.

\param  pSubIndexEntry_p        Pointer to sub-index entry.

\return The function returns the pointer to the default data.
*/
//------------------------------------------------------------------------------
static CONST void* getObjectDefaultPtr(tObdSubEntryPtr pSubIndexEntry_p)
{
    CONST void*     pDefault;
    tObdType        type;

    ASSERTMSG(pSubIndexEntry_p != NULL, "getObjectDefaultPtr(): pointer to SubEntry not valid!\n");

    // get address to default data from default pointer
    pDefault = pSubIndexEntry_p->pDefault;
    if (pDefault != NULL)
    {
        // there are some special types, whose default pointer always is NULL or
        // has to get from other structure get type from sub-index structure
        type = pSubIndexEntry_p->type;

        // check if object type is a string value
        if (type == kObdTypeVString)
        {
            pDefault = ((tObdVStringDef*)pDefault)->pDefString;
        }
        else if (type == kObdTypeOString)
        {
             pDefault = ((tObdOStringDef*)pDefault)->pDefString;
        }
    }
    return pDefault;
}

//------------------------------------------------------------------------------
/**
\brief  Get current data pointer of object

The function returns the pointer to the current data of an object.

\param  pSubIndexEntry_p        Pointer to sub-index entry of object.

\return The function returns the pointer to the current data.
*/
//------------------------------------------------------------------------------
static void MEM* getObjectCurrentPtr(tObdSubEntryPtr pSubIndexEntry_p)
{
    void MEM*       pData;
    UINT            arrayIndex;
    tObdSize        size;

    pData = pSubIndexEntry_p->pCurrent;

    // check if constant object
    if (pData != NULL)
    {
        if ((pSubIndexEntry_p->access & kObdAccArray) != 0)
        {
            // calculate correct data pointer
            arrayIndex = pSubIndexEntry_p->subIndex - 1;
            if ((pSubIndexEntry_p->access & kObdAccVar) != 0)
            {
                size = sizeof(tObdVarEntry);
            }
            else
            {
                size = getObjectSize(pSubIndexEntry_p);
            }
            pData = ((BYTE MEM*)pData) + (size * arrayIndex);
        }

        if ((pSubIndexEntry_p->access & kObdAccVar) != 0)
        {
            pData = ((tObdVarEntry MEM*)pData)->pData;
        }
        else if (pSubIndexEntry_p->type == kObdTypeVString)
        {
            pData = (void MEM*)((tObdVString MEM*)pData)->pString;
        }
        else if (pSubIndexEntry_p->type == kObdTypeOString)
        {
            pData = (void MEM*)((tObdOString MEM*)pData)->pString;
        }
    }
    return pData;
}

//------------------------------------------------------------------------------
/**
\brief  Get object data pointer

The function gets the data pointer of an object. It returns the current data
pointer. If the object is a constant object, it returns the default pointer.

\param  pSubIndexEntry_p        Pointer to sub-index entry of object.

\return The function returns the data pointer of the object.
*/
//------------------------------------------------------------------------------
static void* getObjectDataPtr(tObdSubEntryPtr pSubIndexEntry_p)
{
    void*       pData;
    tObdAccess  access;

    ASSERTMSG(pSubIndexEntry_p != NULL, "getObjectDataPtr(): pointer to SubEntry not valid!\n");

    // there are are some objects whose data pointer has to get from other structure
    // get access type for this object
    access = pSubIndexEntry_p->access;

    // If object has access type = const,
    // only the default value exists.
    if ((access & kObdAccConst) != 0)
    {
        // The pointer to default value can be received from ObdGetObjectDefaultPtr()
        pData = ((void*)getObjectDefaultPtr(pSubIndexEntry_p));
    }
    else
    {
        // The pointer to current value can be received from ObdGetObjectCurrentPtr()
        pData = getObjectCurrentPtr(pSubIndexEntry_p);
    }
    return pData;
}

//------------------------------------------------------------------------------
/**
\brief  Search for index in OBD

The function searches for an index in an OD part. It uses a binary search
algorithm for searching.

\param  pObdEntry_p         OD entry to start searching.
\param  numEntries_p        Number of OD entries.
\param  index_p             Index to search.

\return The function returns the pointer to the OD entry of the searched index.
        If the index isn't found it returns NULL.
*/
//------------------------------------------------------------------------------
static tObdEntryPtr searchIndex(tObdEntryPtr pObdEntry_p, UINT32 numEntries_p, UINT index_p)
{
    UINT32          first;
    UINT32          last;
    UINT32          middle;

    first = 0;
    last = numEntries_p - 1;

    while (first <= last)
    {
        middle = (first + last) >> 1;
        if (pObdEntry_p[middle].index == index_p)
        {
            return &pObdEntry_p[middle];
        }
        else if (pObdEntry_p[middle].index < index_p)
            first = middle + 1;
        else
            last = middle - 1;
    }
    return NULL;
}

//------------------------------------------------------------------------------
/**
\brief  Calculate number of OD entries in partition

The function calculates the number of OD index entries in a partition.

\param  pObdEntry_p         Pointer to the first Index entry.

\return The function returns the number of OD index entries in the partition.
*/
//------------------------------------------------------------------------------
static UINT32 calcPartitionIndexNum(tObdEntryPtr pObdEntry_p)
{
    UINT            index;
    UINT            numEntries = 0;

    index = pObdEntry_p->index;
    // search index in OD part
    while (index != OBD_TABLE_INDEX_END)
    {
        numEntries++;
        pObdEntry_p++;
        index = pObdEntry_p->index;
    }
    return numEntries;
}

//------------------------------------------------------------------------------
/**
\brief  Calculate number of OD entries

The function calculates the number of OD index entries in the OD.

\param  pInitParam_p        Pointer to the OD initialization parameters.
*/
//------------------------------------------------------------------------------
static void calcOdIndexNum(tObdInitParam* pInitParam_p)
{
    tObdEntryPtr    pObdEntry;

    pObdEntry = pInitParam_p->pGenericPart;
    pInitParam_p->numGeneric = calcPartitionIndexNum(pObdEntry);

    pObdEntry = pInitParam_p->pManufacturerPart;
    pInitParam_p->numManufacturer = calcPartitionIndexNum(pObdEntry);

    pObdEntry = pInitParam_p->pDevicePart;
    pInitParam_p->numDevice = calcPartitionIndexNum(pObdEntry);

#if (defined (OBD_USER_OD) && (OBD_USER_OD != FALSE))
    pObdEntry = pInitParam_p->pUserPart;
    pInitParam_p->numUser = calcPartitionIndexNum(pObdEntry);
#endif
}

//------------------------------------------------------------------------------
/**
\brief  Get an index entry from the OD

The function searches for an index entry in the OD.

\param  pInitParam_p        Pointer to the OD initialization parameters.
\param  index_p             Index to search.
\param  ppObdEntry_p        Pointer to store OD entry.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError getIndex(tObdInitParam MEM* pInitParam_p, UINT index_p,
                           tObdEntryPtr* ppObdEntry_p)
{
    tObdEntryPtr    pObdEntry;
    UINT32          numEntries;

#if (defined (OBD_USER_OD) && (OBD_USER_OD != FALSE))
    UINT            nLoop;

    // if user OD is used then objekts also has to be searched in user OD
    // there is less code need if we do this in a loop
    nLoop = 2;
#endif

    // get start address of OD part
    // start address depends on object index because
    // object dictionary is divided in 3 parts
    if ((index_p >= 0x1000) && (index_p < 0x2000))
    {
        pObdEntry = pInitParam_p->pGenericPart;
        numEntries = pInitParam_p->numGeneric;
    }
    else if ((index_p >= 0x2000) && (index_p < 0x6000))
    {
        pObdEntry = pInitParam_p->pManufacturerPart;
        numEntries = pInitParam_p->numManufacturer;
    }

    // index range 0xA000 to 0xFFFF is reserved for DSP-405
    // DS-301 defines that range 0x6000 to 0x9FFF (!!!) is stored if "store" was written to 0x1010/3.
    // Therefore default configuration is CONFIG_OBD_INCLUDE_A000_TO_DEVICE_PART = FALSE.
    // But a CANopen Application which does not implement dynamic OD or user-OD
    // but wants to use static objets 0xA000... should set
    // CONFIG_OBD_INCLUDE_A000_TO_DEVICE_PART to TRUE.
#if (CONFIG_OBD_INCLUDE_A000_TO_DEVICE_PART == FALSE)
    else if ((index_p >= 0x6000) && (index_p < 0x9FFF))
#else
    else if ((index_p >= 0x6000) && (index_p < 0xFFFF))
#endif
    {
        pObdEntry = pInitParam_p->pDevicePart;
        numEntries = pInitParam_p->numDevice;
    }

#if (defined (OBD_USER_OD) && (OBD_USER_OD != FALSE))
    // if index does not match in static OD then index only has to be searched in user OD
    else
    {
        pObdEntry = pInitParam_p->pUserPart;            // begin from first entry of user OD part

        // no user OD is available
        if (pObdEntry == NULL)
            return kErrorObdIndexNotExist;

        numEntries = pInitParam_p->numUser;
        nLoop = 1;                                      // loop must only run once
    }
#else
    // no user OD is available, so other object can be found in OD
    else
    {
        return kErrorObdIllegalPart;
    }
#endif

#if (defined (OBD_USER_OD) && (OBD_USER_OD != FALSE))
    do
    {
        if ((*ppObdEntry_p = searchIndex(pObdEntry, numEntries, index_p)) != NULL)
            return kErrorOk;

        // begin from first entry of user OD part
        pObdEntry = pInitParam_p->pUserPart;
        numEntries = pInitParam_p->numUser;

        // no user OD is available
        if (pObdEntry == NULL)
            return kErrorObdIndexNotExist;

        // switch next loop for user OD
        nLoop--;
    } while (nLoop > 0);
#else
    // No user OD we only need to search once
    if ((*ppObdEntry_p = searchIndex(pObdEntry, numEntries, index_p)) != NULL)
        return kErrorOk;
#endif

    return kErrorObdIndexNotExist;
}

//------------------------------------------------------------------------------
/**
\brief  Get an sub-index entry from the OD

The function searches for an sub-index entry in the OD.

\param  pObdEntry_p         Pointer to the index entry of object.
\param  subIndex_p          Sub-index to search.
\param  ppObdSubEntry_p     Pointer to store sub-index entry.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError getSubindex(tObdEntryPtr pObdEntry_p, UINT subIndex_p,
                              tObdSubEntryPtr* ppObdSubEntry_p)
{
    tObdSubEntryPtr     pSubEntry;
    UINT                nSubIndexCount;

    // get start address of sub-index table and count of sub-indices
    pSubEntry =      pObdEntry_p->pSubIndex;
    nSubIndexCount = pObdEntry_p->count;

    // search sub-index in sub-index table
    while (nSubIndexCount > 0)
    {
        if ((pSubEntry->access & kObdAccArray) != 0)
        {
            if (subIndex_p < pObdEntry_p->count)  // check if sub-index is in range
            {
                // update sub-index number (sub-index entry of an array is always in RAM !!!)
                pSubEntry->subIndex = subIndex_p;
                *ppObdSubEntry_p = pSubEntry;
                return kErrorOk;
            }
        }
        else if (subIndex_p == pSubEntry->subIndex)
        {
            *ppObdSubEntry_p = pSubEntry;       // we found it
            return kErrorOk;
        }

        // Objects are sorted in OD. If the current sub-index in OD is greater
        // than the sub-index which is to be searched, we can stop.
        // In this case user OD has to be searched too.
        if (subIndex_p < pSubEntry->subIndex)
            break;

        pSubEntry++;
        nSubIndexCount--;
    }
    return kErrorObdSubindexNotExist;
}

//------------------------------------------------------------------------------
/**
\brief  Execute a job in an OD partition

The functions runs a job in an OD partition.

\param  currentOdPart_p         OD partition on which to perform the job.
\param  pObdEntry_p             Pointer to OD entry.
\param  direction_p             Determines which job should be done.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError accessOdPartition(tObdPart currentOdPart_p, tObdEntryPtr pObdEntry_p,
                                    tObdDir direction_p)
{
    tObdSubEntryPtr             pSubIndex;
    UINT                        nSubIndexCount;
    tObdAccess                  access;
    void MEM*                   pDstData;
    CONST void*                 pDefault;
    tObdSize                    objSize;
    tOplkError                  ret = kErrorOk;
    tObdVarEntry MEM*           pVarEntry = NULL;

#if (CONFIG_OBD_USE_STORE_RESTORE != FALSE)
    tObdCbStoreParam MEM        cbStore;
    tOplkError                  archiveState = kErrorOk;
#else
    UNUSED_PARAMETER(currentOdPart_p);
#endif

#if (CONFIG_OBD_CALC_OD_SIGNATURE != FALSE)
    UINT32                      odCrc = 0;
#endif

#if (CONFIG_OBD_USE_STORE_RESTORE != FALSE)

    // prepare structure for STORE RESTORE callback function
    cbStore.currentOdPart   = (UINT8)currentOdPart_p;
    cbStore.pData           = NULL;
    cbStore.objSize         = 0;

    // command of first action depends on direction to access
    archiveState = prepareStoreRestore(direction_p, &cbStore);
    if ((archiveState != kErrorOk) && (archiveState != kErrorObdStoreDataObsolete))
        return archiveState;
#endif

    // we should not restore the OD values here
    // the next NMT command "Reset Node" or "Reset Communication" resets the OD data
    if (direction_p != kObdDirRestore)
    {
        while (pObdEntry_p->index != OBD_TABLE_INDEX_END)    // walk through OD part till end is found
        {
            pSubIndex = pObdEntry_p->pSubIndex;
            nSubIndexCount = pObdEntry_p->count;

#if (CONFIG_OBD_CALC_OD_SIGNATURE != FALSE)
            if (direction_p == kObdDirInit)
            {
                odCrc = OPLK_CALCULATE_CRC16(odCrc, (UINT8*)&pObdEntry_p->index, sizeof(pObdEntry_p->index));
                odCrc = OPLK_CALCULATE_CRC16(odCrc, (UINT8*)&pObdEntry_p->count, sizeof(pObdEntry_p->count));
            }
#endif

            while (nSubIndexCount != 0)                         // walk through sub-index table till all sub-indices were restored
            {
                access = (tObdAccess)pSubIndex->access;
                pDefault = getObjectDefaultPtr(pSubIndex);
                pDstData = getObjectCurrentPtr(pSubIndex);
                objSize  = getObjectSize(pSubIndex);

#if (CONFIG_OBD_CALC_OD_SIGNATURE != FALSE)
                if (direction_p == kObdDirInit)
                {
                    odCrc = OPLK_CALCULATE_CRC16(odCrc, (UINT8*)&pSubIndex->subIndex, sizeof(pSubIndex->subIndex));
                    odCrc = OPLK_CALCULATE_CRC16(odCrc, (UINT8*)&pSubIndex->type, sizeof(pSubIndex->type));
                    odCrc = OPLK_CALCULATE_CRC16(odCrc, (UINT8*)&pSubIndex->access, sizeof(pSubIndex->access));
                }
#endif

                switch (direction_p)
                {
                    // VarEntry structures has to be initialized
                    case kObdDirInit:
                        // If VAR-Flag is set, pCurrent means not address of data but address of tObdVarEntry.
                        // Address of data has to be get from this structure.
                        if ((access & kObdAccVar) != 0)
                        {
                            getVarEntry(pSubIndex, &pVarEntry);
                            obd_initVarEntry(pVarEntry, pSubIndex->type, objSize);
                            // at this time no application variable is defined therefore data can not be copied!
                            break;
                        }
                        else if (pSubIndex->type == kObdTypeVString)
                        {
                            // If pCurrent is not NULL then the string was defined with OBD_SUBINDEX_RAM_VSTRING.
                            // The current pointer points to struct tObdVString located in MEM. The element size includes
                            // the max. number of bytes. pString includes the pointer to string in MEM. The memory
                            // location of default string must be copied to memory location of current string.
                            if (pSubIndex->pCurrent != NULL)
                            {
                                // For copying data we have to set the destination pointer to the real RAM string. This
                                // pointer to RAM string is located in default string info structure.
                                pDstData = (void MEM*)((tObdVStringDef ROM*)pSubIndex->pDefault)->pString;
                                objSize  = ((tObdVStringDef ROM*)pSubIndex->pDefault)->size;

                                ((tObdVString MEM*)pSubIndex->pCurrent)->pString = pDstData;
                                ((tObdVString MEM*)pSubIndex->pCurrent)->size    = objSize;
                            }
                        }
                        else if (pSubIndex->type == kObdTypeOString)
                        {
                            if (pSubIndex->pCurrent != NULL)
                            {
                                // For copying data we have to set the destination pointer to the real RAM string. This
                                // pointer to RAM string is located in default string info structure.
                                pDstData = (void MEM*)((tObdOStringDef ROM*)pSubIndex->pDefault)->pString;
                                objSize  = ((tObdOStringDef ROM*)pSubIndex->pDefault)->size;

                                ((tObdOString MEM*)pSubIndex->pCurrent)->pString = pDstData;
                                ((tObdOString MEM*)pSubIndex->pCurrent)->size    = objSize;
                            }
                        }

                        copyObjectData(pDstData, pDefault, objSize, pSubIndex->type);
                        callPostDefault(pDstData, pObdEntry_p, pSubIndex);
                        break;

                    // objects with attribute kObdAccStore has to be load from EEPROM or from a file
                    case kObdDirLoad:
                        copyObjectData(pDstData, pDefault, objSize, pSubIndex->type);
                        callPostDefault(pDstData, pObdEntry_p, pSubIndex);
#if (CONFIG_OBD_USE_STORE_RESTORE != FALSE)
                        if (archiveState == kErrorOk)
                        {
                            ret = doStoreRestore(access, &cbStore, pDstData, objSize);
                            if (ret != kErrorOk)
                                goto Exit;

                        }
#endif
                        break;

                    // objects with attribute kObdAccStore has to be stored in EEPROM or in a file
                    case kObdDirStore:
#if (CONFIG_OBD_USE_STORE_RESTORE != FALSE)
                        ret = doStoreRestore(access, &cbStore, pDstData, objSize);
                        if (ret != kErrorOk)
                            goto Exit;
#endif
                        break;

                    // if OD Builder key has to be checked no access to sub-index and data should be made
                    case kObdDirOBKCheck:
                        nSubIndexCount = 1;         // break the second loop earlier
                        break;

                    // unknown Direction
                    default:
                        nSubIndexCount = 1;         // break the second loop earlier
                        break;
                }

                nSubIndexCount--;

                // next sub-index entry
                if ((access & kObdAccArray) == 0)
                {
                    pSubIndex++;
                    if ((nSubIndexCount > 0) && ((pSubIndex->access & kObdAccArray) != 0))
                    {
                        pSubIndex->subIndex = 1;    // next sub-index points to an array - reset sub-index number
                    }
                }
                else
                {
                    if (nSubIndexCount > 0)
                    {
                        pSubIndex->subIndex++;      // next sub-index points to an array - increment sub-index number
                    }
                }
            }
            pObdEntry_p++;                          // next index entry
        }
    }

#if (CONFIG_OBD_CALC_OD_SIGNATURE != FALSE)
    // Save the calculated CRC for writing when the archive is being closed
    if (direction_p == kObdDirInit)
    {
        switch (currentOdPart_p)
        {
            case kObdPartGen:
                obdInstance_l.aOdSignature[0] = odCrc;
                break;
            case kObdPartMan:
                obdInstance_l.aOdSignature[1] = odCrc;
                break;
            case kObdPartDev:
                obdInstance_l.aOdSignature[2] = odCrc;
                break;
            default:
                break;
        }
    }
#endif

#if (CONFIG_OBD_USE_STORE_RESTORE != FALSE)
Exit:
    // command of last action depends on direction to access
    archiveState = cleanupStoreRestore(direction_p, &cbStore);
    if (ret == kErrorOk)
        ret = archiveState;
#endif
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Copy object data

The functions copies object data.

\param  pDstData_p              Destination for copy operation.
\param  pSrcData_p              Source for copy operation.
\param  objSize_p               Size of data to copy.
\param  objType_p               Type of object.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static void copyObjectData(void MEM* pDstData_p, CONST void* pSrcData_p,
                           tObdSize objSize_p, tObdType objType_p)
{
    tObdSize StrSize = 0;

    // it is allowed to set default and current address to NULL (nothing to copy)
    if (pDstData_p != NULL)
    {
        if (objType_p == kObdTypeVString)
        {
            // The function calculates the really number of characters of string. The
            // object entry size can be bigger as string size of default string.
            // The '\0'-termination is NOT included. A string with no characters has a
            // size of 0.
            StrSize = getObdStringLen((void*)pSrcData_p, objSize_p, kObdTypeVString);

            // If the string length is greater than or equal to the entry size in OD then only copy
            // entry size - 1 and always set the '\0'-termination.
            if (StrSize >= objSize_p)
            {
                StrSize = objSize_p - 1;
            }
        }

        if (pSrcData_p != NULL)
        {
            OPLK_MEMCPY(pDstData_p, pSrcData_p, objSize_p);
            if (objType_p == kObdTypeVString)
            {
                ((char MEM*)pDstData_p)[StrSize] = '\0';
            }
        }
    }
}

//------------------------------------------------------------------------------
/**
\brief  Call object callback function

The function calls the callback function of an object.

\param  pfnCallback_p           Pointer to the callback function.
\param  pCbParam_p              Pointer to callback function parameter structure.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError callObjectCallback(tObdCallback pfnCallback_p, tObdCbParam MEM* pCbParam_p)
{
    tOplkError           ret = kErrorOk;
    tObdCallback MEM     pfnCallback;

    if (pfnCallback_p != NULL)
    {
        pfnCallback = pfnCallback_p;    // Necessary to fix bug for KEIL C51 V6.01!
        ret = pfnCallback(pCbParam_p);
    }
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Call callback function of post callback event

The functions calls the callback function of the post callback event.

\param  pData_p                 Pointer to object data.
\param  pObdEntry_p             Pointer to index entry of object.
\param  pObdSubEntry_p          Pointer to sub-index entry of object.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError callPostDefault(void* pData_p, tObdEntryPtr pObdEntry_p,
                                  tObdSubEntryPtr pObdSubEntry_p)
{
    tOplkError          ret;
    tObdCbParam         cbParam;

    cbParam.index    = pObdEntry_p->index;
    cbParam.subIndex = pObdSubEntry_p->subIndex;
    cbParam.pArg     = pData_p;
    cbParam.obdEvent = kObdEvPostDefault;
    ret = callObjectCallback(pObdEntry_p->pfnCallback, &cbParam);
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Check if object is numerical

The functions checks if an object is a numerical object.

\param  pObdSubEntry_p          Pointer to sub-index entry of object.
\param  pfEntryNumerical_p      Pointer to store flag. TRUE if it is numerical,
                                FALSE otherwise.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError isNumerical(tObdSubEntryPtr pObdSubEntry_p,
                              BOOL* pfEntryNumerical_p)
{
    tOplkError          ret = kErrorOk;

    // get Type
    if ((pObdSubEntry_p->type == kObdTypeVString) ||
        (pObdSubEntry_p->type == kObdTypeOString) ||
        (pObdSubEntry_p->type == kObdTypeDomain))
    {   // not numerical types
        *pfEntryNumerical_p = FALSE;
    }
    else
    {   // numerical types
        *pfEntryNumerical_p = TRUE;
    }
    return ret;
}

#if (CONFIG_OBD_CHECK_OBJECT_RANGE != FALSE)
//------------------------------------------------------------------------------
/**
\brief  Check value range of object

The function checks the value range of an object.

\note The pointer of data (pData_p) must point out to an even address, if
ObjType is unequal to kObdTypeInt8 or kObdTypeUInt8!

\param  pSubIndexEntry_p    Pointer to the sub-index entry structure of the object.
\param  pData_p             Pointer to the data to be checked.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError checkObjectRange(tObdSubEntryPtr pSubIndexEntry_p, void* pData_p)
{
    tOplkError          ret = kErrorOk;
    const void*         pRangeData;

    // check if data range has to be checked
    if ((pSubIndexEntry_p->access & kObdAccRange) == 0)
        return ret;

    pRangeData = pSubIndexEntry_p->pDefault;    // get address of default data

    // jump to called object type
    switch ((tObdType)pSubIndexEntry_p->type)
    {
        // ObdType kObdTypeBool will not be checked because there are only
        // two possible values 0 or 1.

        // ObdTypes which has to be checked up because numerical values
        case kObdTypeInt8:
            pRangeData = ((tObdInteger8*)pRangeData) + 1;            // switch to lower limit
            if (*((tObdInteger8*)pData_p) < *((tObdInteger8*)pRangeData))
            {
                ret = kErrorObdValueTooLow;
                break;
            }
            pRangeData = ((tObdInteger8*)pRangeData) + 1;            // switch to higher limit
            if (*((tObdInteger8*)pData_p) > *((tObdInteger8*)pRangeData))
            {
                ret = kErrorObdValueTooHigh;
            }
            break;

        case kObdTypeUInt8:
            pRangeData = ((tObdUnsigned8*)pRangeData) + 1;           // switch to lower limit
            if (*((tObdUnsigned8*)pData_p) < *((tObdUnsigned8*)pRangeData))
            {
                ret = kErrorObdValueTooLow;
                break;
            }
            pRangeData = ((tObdUnsigned8*)pRangeData) + 1;           // switch to higher limit
            if (*((tObdUnsigned8*)pData_p) > *((tObdUnsigned8*)pRangeData))
            {
                ret = kErrorObdValueTooHigh;
            }
            break;

        case kObdTypeInt16:
            pRangeData = ((tObdInteger16*)pRangeData) + 1;           // switch to lower limit
            if (*((tObdInteger16*)pData_p) < *((tObdInteger16*)pRangeData))
            {
                ret = kErrorObdValueTooLow;
                break;
            }
            pRangeData = ((tObdInteger16*)pRangeData) + 1;           // switch to higher limit
            if (*((tObdInteger16*)pData_p) > *((tObdInteger16*)pRangeData))
            {
                ret = kErrorObdValueTooHigh;
            }
            break;

        case kObdTypeUInt16:
            pRangeData = ((tObdUnsigned16*)pRangeData) + 1;          // switch to lower limit
            if (*((tObdUnsigned16*)pData_p) < *((tObdUnsigned16*)pRangeData))
            {
                ret = kErrorObdValueTooLow;
                break;
            }
            pRangeData = ((tObdUnsigned16*)pRangeData) + 1;          // switch to higher limit
            if (*((tObdUnsigned16*)pData_p) > *((tObdUnsigned16*)pRangeData))
            {
                ret = kErrorObdValueTooHigh;
            }
            break;

        case kObdTypeInt32:
            pRangeData = ((tObdInteger32*)pRangeData) + 1;           // switch to lower limit
            if (*((tObdInteger32*)pData_p) < *((tObdInteger32*)pRangeData))
            {
                ret = kErrorObdValueTooLow;
                break;
            }
            pRangeData = ((tObdInteger32*)pRangeData) + 1;           // switch to higher limit
            if (*((tObdInteger32*)pData_p) > *((tObdInteger32*)pRangeData))
            {
                ret = kErrorObdValueTooHigh;
            }

            break;

        case kObdTypeUInt32:
            pRangeData = ((tObdUnsigned32*)pRangeData) + 1;          // switch to lower limit
            if (*((tObdUnsigned32*)pData_p) < *((tObdUnsigned32*)pRangeData))
            {
                ret = kErrorObdValueTooLow;
                break;
            }
            pRangeData = ((tObdUnsigned32*)pRangeData) + 1;          // switch to higher limit
            if (*((tObdUnsigned32*)pData_p) > *((tObdUnsigned32*)pRangeData))
            {
                ret = kErrorObdValueTooHigh;
            }
            break;

        case kObdTypeReal32:
            pRangeData = ((tObdReal32*)pRangeData) + 1;              // switch to lower limit
            if (*((tObdReal32*)pData_p) < *((tObdReal32*)pRangeData))
            {
                ret = kErrorObdValueTooLow;
                break;
            }
            pRangeData = ((tObdReal32*)pRangeData) + 1;              // switch to higher limit
            if (*((tObdReal32*)pData_p) > *((tObdReal32*)pRangeData))
            {
                ret = kErrorObdValueTooHigh;
            }
            break;

        case kObdTypeInt40:
        case kObdTypeInt48:
        case kObdTypeInt56:
        case kObdTypeInt64:
            pRangeData = ((tObdInteger64*)pRangeData) + 1;           // switch to lower limit
            if (*((tObdInteger64*)pData_p) < *((tObdInteger64*)pRangeData))
            {
                ret = kErrorObdValueTooLow;
                break;
            }
            pRangeData = ((tObdInteger64*)pRangeData) + 1;           // switch to higher limit
            if (*((tObdInteger64*)pData_p) > *((tObdInteger64*)pRangeData))
            {
                ret = kErrorObdValueTooHigh;
            }
            break;

        case kObdTypeUInt40:
        case kObdTypeUInt48:
        case kObdTypeUInt56:
        case kObdTypeUInt64:
            pRangeData = ((tObdUnsigned64*)pRangeData) + 1;          // switch to lower limit
            if (*((tObdUnsigned64*)pData_p) < *((tObdUnsigned64*)pRangeData))
            {
                ret = kErrorObdValueTooLow;
                break;
            }
            pRangeData = ((tObdUnsigned64*)pRangeData) + 1;          // switch to higher limit
            if (*((tObdUnsigned64*)pData_p) > *((tObdUnsigned64*)pRangeData))
            {
                ret = kErrorObdValueTooHigh;
            }
            break;

        case kObdTypeReal64:
            pRangeData = ((tObdReal64*)pRangeData) + 1;              // switch to lower limit
            if (*((tObdReal64*)pData_p) < *((tObdReal64*)pRangeData))
            {
                ret = kErrorObdValueTooLow;
                break;
            }
            pRangeData = ((tObdReal64*)pRangeData) + 1;              // switch to higher limit
            if (*((tObdReal64*)pData_p) > *((tObdReal64*)pRangeData))
            {
                ret = kErrorObdValueTooHigh;
            }
            break;

        case kObdTypeTimeOfDay:
        case kObdTypeTimeDiff:
            break;

        // ObdTypes kObdTypeXString and kObdTypeDomain can not be checked because
        // they have no numerical value.
        default:
            ret = kErrorObdUnknownObjectType;
            break;
    }
    return ret;
}
#endif // (CONFIG_OBD_CHECK_OBJECT_RANGE != FALSE)

#if (CONFIG_OBD_USE_STORE_RESTORE != FALSE)
//------------------------------------------------------------------------------
/**
\brief  Prepare Store/Restore of objects

The functions prepares a store/restore command.

\param  direction_p             OD command direction.
\param  pCbStore_p              Pointer to store callback parameters.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError prepareStoreRestore(tObdDir direction_p, tObdCbStoreParam MEM* pCbStore_p)
{
    tOplkError          ret;

    if (direction_p == kObdDirLoad)
    {
        pCbStore_p->command = (UINT8)kObdCmdOpenRead;
        // call callback function for previous command
        ret = callStoreCallback(pCbStore_p);
        if (ret != kErrorOk)
            return ret;

        // set command for index and sub-index loop
        pCbStore_p->command = (UINT8)kObdCmdReadObj;
    }
    else
    {
        if (direction_p == kObdDirStore)
        {
            pCbStore_p->command = (UINT8)kObdCmdOpenWrite;
            // call callback function for previous command
            ret = callStoreCallback (pCbStore_p);
            if (ret != kErrorOk)
                return ret;

            // set command for index and sub-index loop
            pCbStore_p->command = (UINT8)kObdCmdWriteObj;
        }
    }

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Cleanup Store/Restore of objects

The functions cleans up a store/restore command.

\param  direction_p             OD command direction.
\param  pCbStore_p              Pointer to store callback parameters.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError cleanupStoreRestore(tObdDir direction_p, tObdCbStoreParam MEM* pCbStore_p)
{
    tOplkError          ret = kErrorOk;

    if (direction_p == kObdDirOBKCheck)
    {
        return ret;
    }
    else
    {
        if (direction_p == kObdDirLoad)
        {
            pCbStore_p->command = (UINT8)kObdCmdCloseRead;
        }
        else if (direction_p == kObdDirStore)
        {
            pCbStore_p->command = (UINT8)kObdCmdCloseWrite;
        }
        else if (direction_p == kObdDirRestore)
        {
            pCbStore_p->command = (UINT8)kObdCmdClear;
        }
        else
        {
            return ret;
        }

        // Call callback function for last command
        ret = callStoreCallback(pCbStore_p);
        return ret;
    }
}

//------------------------------------------------------------------------------
/**
\brief  Do Store/Restore of objects

The functions executes a store/restore command.

\param  access_p                OD access command.
\param  pCbStore_p              Pointer to store callback parameters.
\param  pObjData_p              Pointer to object data.
\param  objSize_p               Size of object.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError doStoreRestore(tObdAccess access_p, tObdCbStoreParam MEM* pCbStore_p,
                                 void MEM* pObjData_p, tObdSize objSize_p)
{
    tOplkError          ret = kErrorOk;

    // when attribute kObdAccStore is set, then call callback function
    if ((access_p & kObdAccStore) != 0)
    {
        // fill out data pointer and size of data
        pCbStore_p->pData   = pObjData_p;
        pCbStore_p->objSize = objSize_p;

        // call callback function for read or write object
        ret = callStoreCallback(pCbStore_p);
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Call store callback function

The functions calls the store callback function.

\param  pCbStoreParam_p         Pointer to callback function parameters.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError callStoreCallback(tObdCbStoreParam MEM* pCbStoreParam_p)
{
    tOplkError ret = kErrorOk;

    if (obdInstance_l.pfnStoreLoadObjectCb != NULL)
    {
        ret = obdInstance_l.pfnStoreLoadObjectCb(pCbStoreParam_p);
    }

    return ret;
}
#endif // (CONFIG_OBD_USE_STORE_RESTORE != FALSE)

/// \}
