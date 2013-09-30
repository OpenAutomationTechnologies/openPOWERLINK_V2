/****************************************************************************

  (c) SYSTEC electronic GmbH, D-07973 Greiz, August-Bebel-Str. 29
      www.systec-electronic.com

  Project:      openPOWERLINK

  Description:  source file for api function of EplOBD-Module

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
             ->based on CANopen OBD-Modul

****************************************************************************/

#include "EplInc.h"
#include "obd.h"


/***************************************************************************/
/*                                                                         */
/*                                                                         */
/*          G L O B A L   D E F I N I T I O N S                            */
/*                                                                         */
/*                                                                         */
/***************************************************************************/

//---------------------------------------------------------------------------
// const defines
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
// local types
//---------------------------------------------------------------------------

// struct for instance table
static tObdInitParam                m_ObdInitParam;
static tEplObdStoreLoadObjCallback  m_fpStoreLoadObjCallback;

// decomposition of float
typedef union
{
    tObdReal32      m_flRealPart;
    int             m_nIntegerPart;

} tEplObdRealParts;


//---------------------------------------------------------------------------
// module global vars
//---------------------------------------------------------------------------

// This macro replace the unspecific pointer to an instance through
// the modul specific type for the local instance table. This macro
// must defined in each modul.
//#define tEplPtrInstance             tEplInstanceInfo MEM*

BYTE MEM            abEplObdTrashObject_g[8];


//---------------------------------------------------------------------------
// local function prototypes
//---------------------------------------------------------------------------

static tEplKernel   callObjectCallback(tObdCallback pfnCallback_p, tObdCbParam MEM* pCbParam_p);
static tObdSize     getDataSizeIntern(tObdSubEntryPtr pSubIndexEntry_p);
static tObdSize     getObdStringLen(void* pObjData_p, tObdSize objLen_p, tObdType objType_p);
#if (EPL_OBD_CHECK_OBJECT_RANGE != FALSE)
static tEplKernel   checkObjectRange(tObdSubEntryPtr pSubindexEntry_p, void * pData_p);
#endif
static tEplKernel   getVarEntry(tObdSubEntryPtr pSubindexEntry_p, tObdVarEntry MEM** ppVarEntry_p);
static tEplKernel   getEntry(UINT index_p, UINT subindex_p, tObdEntryPtr* ppObdEntry_p,
                             tObdSubEntryPtr* ppObdSubEntry_p);
static tObdSize     getObjectSize(tObdSubEntryPtr pSubIndexEntry_p);
static tEplKernel   getIndexIntern(tObdInitParam MEM* pInitParam_p, UINT index_p, tObdEntryPtr* ppObdEntry_p);
static tEplKernel   getSubindexIntern(tObdEntryPtr pObdEntry_p, UINT subIndex_p, tObdSubEntryPtr* ppObdSubEntry_p);
static tEplKernel   accessOdPartIntern(tObdPart currentOdPart_p, tObdEntryPtr pObdEnty_p, tObdDir direction_p);
static CONST void*  getObjectDefaultPtr (tObdSubEntryPtr pSubIndexEntry_p);
static void MEM*    getObjectCurrentPtr (tObdSubEntryPtr pSubIndexEntry_p);
#if (EPL_OBD_USE_STORE_RESTORE != FALSE)
static tEplKernel   callStoreCallback(tObdCbStoreParam MEM* pCbStoreParam_p);
#endif // (EPL_OBD_USE_STORE_RESTORE != FALSE)
static void         copyObjectData(void MEM* pDstData_p, CONST void* pSrcData_p, tObdSize objSize_p,
                                   tObdType objType_p);
static tEplKernel   callPostDefault(void *pData_p, tObdEntryPtr pObdEntry_p, tObdSubEntryPtr pSubIndex_p);
static void*        getObjectDataPtrIntern(tObdSubEntryPtr pSubindexEntry_p);
static tEplKernel   isNumericalIntern(tObdSubEntryPtr pObdSubEntry_p, BOOL* pfEntryNumerical_p);
static tEplKernel   writeEntryPre(UINT uiIndex_p, UINT subIndex_p, void* pSrcData_p, void** ppDstData_p,
                                  tObdSize Size_p, tObdEntryPtr* ppObdEntry_p, tObdSubEntryPtr* ppSubEntry_p,
                                  tObdCbParam MEM* pCbParam_p, tObdSize* pObdSize_p);
static tEplKernel   writeEntryPost(tObdEntryPtr pObdEntry_p, tObdSubEntryPtr pSubEntry_p,
                                   tObdCbParam MEM* pCbParam_p, void* pSrcData_p,
                                   void* pDstData_p, tObdSize obdSize_p);

//=========================================================================//
//                                                                         //
//          P U B L I C   F U N C T I O N S                                //
//                                                                         //
//=========================================================================//

//---------------------------------------------------------------------------
//
// Function:    obd_init()
//
// Description: initializes the first instance
//
// Parameters:  pInitParam_p    = init parameter
//
// Return:      tEplKernel      =   errorcode
//
// State:
//
//---------------------------------------------------------------------------

tEplKernel obd_init(tObdInitParam MEM* pInitParam_p)
{

tEplKernel Ret;

    if (pInitParam_p == NULL)
    {
        Ret = kEplSuccessful;
        goto Exit;
    }

    Ret = obd_addInstance (
        pInitParam_p);

Exit:
    return Ret;

}


//---------------------------------------------------------------------------
//
// Function:    obd_addInstance()
//
// Description: adds a new instance
//
// Parameters:  pInitParam_p
//
// Return:      tEplKernel
//
// State:
//
//---------------------------------------------------------------------------

tEplKernel obd_addInstance(tObdInitParam MEM* pInitParam_p)
{

tEplKernel Ret;


    // save init parameters
    EPL_MEMCPY (&m_ObdInitParam, pInitParam_p, sizeof (tObdInitParam));

    // clear callback function for command LOAD and STORE
    m_fpStoreLoadObjCallback = NULL;

    // initialize object dictionary
    // so all all VarEntries will be initialized to trash object and default values will be set to current data
    Ret = obd_accessOdPart (kObdPartAll, kObdDirInit);

    return Ret;

}


//---------------------------------------------------------------------------
//
// Function:    obd_deleteInstance()
//
// Description: delete instance
//
// Parameters:
//
// Return:      tEplKernel
//
// State:
//
//---------------------------------------------------------------------------
#if (EPL_USE_DELETEINST_FUNC != FALSE)
tEplKernel obd_deleteInstance(void)
{
    return kEplSuccessful;

}
#endif // (EPL_USE_DELETEINST_FUNC != FALSE)


//---------------------------------------------------------------------------
//
// Function:    obd_writeEntry()
//
// Description: Function writes data to an OBD entry. Strings
//              are stored with added '\0' character.
//
// Parameters:
//              uiIndex_p       =   Index of the OD entry
//              subIndex_p    =   Subindex of the OD Entry
//              pSrcData_p      =   Pointer to the data to write
//              Size_p          =   Size of the data in Byte
//
// Return:      tEplKernel      =   Errorcode
//
//
// State:
//
//---------------------------------------------------------------------------

tEplKernel obd_writeEntry(UINT index_p, UINT subIndex_p, void* pSrcData_p, tObdSize size_p)
{

tEplKernel              Ret;
tObdEntryPtr            pObdEntry;
tObdSubEntryPtr         pSubEntry;
tObdCbParam MEM         CbParam;
void MEM*               pDstData;
tObdSize                ObdSize;


    Ret = writeEntryPre (
                               index_p,
                               subIndex_p,
                               pSrcData_p,
                               &pDstData,
                               size_p,
                               &pObdEntry,
                               &pSubEntry,
                               &CbParam,
                               &ObdSize);
    if (Ret != kEplSuccessful)
    {
        goto Exit;
    }

    Ret = writeEntryPost (
                                pObdEntry,
                                pSubEntry,
                                &CbParam,
                                pSrcData_p,
                                pDstData,
                                ObdSize);
    if (Ret != kEplSuccessful)
    {
        goto Exit;
    }

Exit:

    return Ret;

}


//---------------------------------------------------------------------------
//
// Function:    obd_readEntry()
//
// Description: The function reads an object entry. The application
//              can always read the data even if attrib kObdAccRead
//              is not set. The attrib is only checked up for SDO transfer.
//
// Parameters:
//              index_p       = Index oof the OD entry to read
//              subIndex_p    = Subindex to read
//              pDstData_p      = pointer to the buffer for data
//              Offset_p        = offset in data for read access
//              pSize_p         = IN: Size of the buffer
//                                OUT: number of readed Bytes
//
// Return:      tEplKernel
//
// State:
//
//---------------------------------------------------------------------------

tEplKernel obd_readEntry(UINT index_p, UINT subIndex_p, void* pDstData_p, tObdSize* pSize_p)
{

tEplKernel                      Ret;
tObdEntryPtr                    pObdEntry;
tObdSubEntryPtr                 pSubEntry;
tObdCbParam  MEM                CbParam;
void *                          pSrcData;
tObdSize                        ObdSize;

    ASSERT (pDstData_p != NULL);
    ASSERT (pSize_p != NULL);

    // get address of index and subindex entry
    Ret = getEntry (
        index_p, subIndex_p, &pObdEntry, &pSubEntry);
    if (Ret != kEplSuccessful)
    {
        goto Exit;
    }

    // get pointer to object data
     pSrcData = getObjectDataPtrIntern (pSubEntry);

    // check source pointer
    if (pSrcData == NULL)
    {
        Ret = kEplObdReadViolation;
        goto Exit;
    }

    //------------------------------------------------------------------------
    // address of source data to structure of callback parameters
    // so callback function can change this data before reading
    CbParam.index   = index_p;
    CbParam.subIndex = subIndex_p;
    CbParam.pArg = pSrcData;
    CbParam.obdEvent = kObdEvPreRead;
    Ret = callObjectCallback (
        pObdEntry->pfnCallback, &CbParam);
    if (Ret != kEplSuccessful)
    {
        goto Exit;
    }

    // get size of data and check if application has reserved enough memory
    ObdSize = getDataSizeIntern (pSubEntry);

    // check if offset given and calc correct number of bytes to read
    if (*pSize_p < ObdSize)
    {
        Ret = kEplObdValueLengthError;
        goto Exit;
    }

    // read value from object
    EPL_MEMCPY (pDstData_p, pSrcData, ObdSize);

    if (pSubEntry->type == kObdTypeVString)
    {
        if (*pSize_p > ObdSize)
        {   // space left to set the terminating null-character
            ((char MEM*) pDstData_p)[ObdSize] = '\0';
            ObdSize++;
        }
    }
    *pSize_p = ObdSize;

    // write address of destination data to structure of callback parameters
    // so callback function can change this data after reading
    CbParam.pArg     = pDstData_p;
    CbParam.obdEvent = kObdEvPostRead;
    Ret = callObjectCallback (
        pObdEntry->pfnCallback, &CbParam);

Exit:

    return Ret;

}

//---------------------------------------------------------------------------
//
// Function:    obd_accessOdPart()
//
// Description: restores default values of one part of OD
//
// Parameters:  ObdPart_p
//              direction_p
//
// Return:      tEplKernel
//
// State:
//
//---------------------------------------------------------------------------

tEplKernel obd_accessOdPart (tObdPart obdPart_p, tObdDir  direction_p)
{

tEplKernel      Ret = kEplSuccessful;
BOOL            fPartFount;
tObdEntryPtr    pObdEntry;

    //  part always has to be unequal to NULL
    pObdEntry = m_ObdInitParam.pGenericPart;
    ASSERTMSG (pObdEntry != NULL, "obd_accessOdPart(): no  OD part is defined!\n");

    // if obdPart_p is not valid fPartFound keeps FALSE and function returns kEplObdIllegalPart
    fPartFount = FALSE;

    // access to  part
    if ((obdPart_p & kObdPartGen) != 0)
    {
        fPartFount = TRUE;

        Ret = accessOdPartIntern (
            kObdPartGen, pObdEntry, direction_p);
        if (Ret != kEplSuccessful)
        {
            goto Exit;
        }
    }

    // access to manufacturer part
    pObdEntry = m_ObdInitParam.pManufacturerPart;

    if ( ((obdPart_p & kObdPartMan) != 0) &&
         (pObdEntry != NULL) )
    {
        fPartFount = TRUE;

        Ret = accessOdPartIntern (
            kObdPartMan, pObdEntry, direction_p);
        if (Ret != kEplSuccessful)
        {
            goto Exit;
        }
    }

    // access to device part
    pObdEntry = m_ObdInitParam.pDevicePart;

    if ( ((obdPart_p & kObdPartDev) != 0) &&
         (pObdEntry != NULL) )
    {
        fPartFount = TRUE;

        Ret = accessOdPartIntern (
            kObdPartDev, pObdEntry, direction_p);
        if (Ret != kEplSuccessful)
        {
            goto Exit;
        }
    }

    #if (defined (EPL_OBD_USER_OD) && (EPL_OBD_USER_OD != FALSE))
    {
        // access to user part
        pObdEntry = m_ObdInitParam.m_pUserPart;

        if ( ((obdPart_p & kObdPartUsr) != 0) &&
             (pObdEntry != NULL) )
        {
            fPartFount = TRUE;

            Ret = accessOdPartIntern (kObdPartUsr, pObdEntry, direction_p);
            if (Ret != kEplSuccessful)
            {
                goto Exit;
            }
        }
    }
    #endif

    // no access to an OD part was done? illegal OD part was specified!
    if (fPartFount == FALSE)
    {
        Ret = kEplObdIllegalPart;
    }

Exit:

    return Ret;

}


//---------------------------------------------------------------------------
//
// Function:    obd_defineVar()
//
// Description: defines a variable in OD
//
// Parameters:  pEplVarParam_p
//
// Return:      tEplKernel
//
// State:
//
//---------------------------------------------------------------------------

tEplKernel obd_defineVar (tVarParam MEM* pVarParam_p)
{

tEplKernel              Ret;
tObdVarEntry MEM*       pVarEntry;
tVarParamValid          VarValid;
tObdSubEntryPtr         pSubindexEntry;

    ASSERT (pVarParam_p != NULL);   // is not allowed to be NULL

    // get address of subindex entry
    Ret = getEntry (
        pVarParam_p->index,
        pVarParam_p->subindex,
        NULL, &pSubindexEntry);
    if (Ret != kEplSuccessful)
    {
        goto Exit;
    }

    // get var entry
    Ret = getVarEntry (pSubindexEntry, &pVarEntry);
    if (Ret != kEplSuccessful)
    {
        goto Exit;
    }

    VarValid =  pVarParam_p->validFlag;

    // copy only this values, which valid flag is set
    if ((VarValid & kVarValidSize) != 0)
    {
        if (pSubindexEntry->type != kObdTypeDomain)
        {
        tObdSize DataSize;

            // check passed size parameter
            DataSize = getObjectSize(pSubindexEntry);
            if (DataSize != pVarParam_p->size)
            {   // size of variable does not match
                Ret = kEplObdValueLengthError;
                goto Exit;
            }
        }
        else
        {   // size can be set only for objects of type DOMAIN
            pVarEntry->size = pVarParam_p->size;
        }
    }

    if ((VarValid & kVarValidData) != 0)
    {
       pVarEntry->pData = pVarParam_p->pData;
    }
/*
    #if (EPL_PDO_USE_STATIC_MAPPING == FALSE)
    {
        if ((VarValid & kVarValidCallback) != 0)
        {
           pVarEntry->m_fpCallback = pVarParam_p->m_fpCallback;
        }

        if ((VarValid & kVarValidArg) != 0)
        {
           pVarEntry->m_pArg = pVarParam_p->m_pArg;
        }
    }
    #endif
*/
    // Ret is already set to kEplSuccessful from ObdGetVarIntern()

Exit:

    return Ret;

}


//---------------------------------------------------------------------------
//
// Function:    obd_getObjectDataPtr()
//
// Description: It returnes the current data pointer. But if object is an
//              constant object it returnes the default pointer.
//
// Parameters:  index_p    =   Index of the entry
//              uiSubindex_p =   Subindex of the entry
//
// Return:      void *    = pointer to object data
//
// State:
//
//---------------------------------------------------------------------------

void* obd_getObjectDataPtr (UINT index_p, UINT subIndex_p)
 {
tEplKernel          Ret;
void *              pData;
tObdEntryPtr        pObdEntry;
tObdSubEntryPtr     pObdSubEntry;


    // get pointer to index structure
    Ret = getIndexIntern (&m_ObdInitParam,
                                index_p,
                                &pObdEntry);
    if(Ret != kEplSuccessful)
    {
        pData = NULL;
        goto Exit;
    }

    // get pointer to subindex structure
    Ret = getSubindexIntern (pObdEntry,
                                subIndex_p,
                                &pObdSubEntry);
    if(Ret != kEplSuccessful)
    {
        pData = NULL;
        goto Exit;
    }
    // get Datapointer
    pData = getObjectDataPtrIntern(pObdSubEntry);

Exit:
    return pData;

}


#if (defined (EPL_OBD_USER_OD) && (EPL_OBD_USER_OD != FALSE))
//jba dead code!
//---------------------------------------------------------------------------
//
// Function:    obd_registerUserOd()
//
// Description: function registers the user OD
//
// Parameters:  pUserOd_p   =pointer to user ODd
//
// Return:     tEplKernel = errorcode
//
// State:
//
//---------------------------------------------------------------------------
tEplKernel obd_registerUserOd (tObdEntryPtr pUserOd_p)
{

    m_ObdInitParam.m_pUserPart = pUserOd_p;

    return kEplSuccessful;

}

#endif


//---------------------------------------------------------------------------
//
// Function:    obd_initVarEntry()
//
// Description: function to initialize VarEntry dependened on object type
//
// Parameters:  pVarEntry_p = pointer to var entry structure
//              Type_p      = object type
//              obdSize_p   = size of object data
//
// Returns:     none
//
// State:
//
//---------------------------------------------------------------------------

void obd_initVarEntry (tObdVarEntry MEM* pVarEntry_p, tObdType type_p, tObdSize obdSize_p)
{
/*
    #if (EPL_PDO_USE_STATIC_MAPPING == FALSE)
    {
        // reset pointer to VAR callback and argument
        pVarEntry_p->m_fpCallback  = NULL;
        pVarEntry_p->m_pArg = NULL;
    }
    #endif
*/

// 10-dec-2004 r.d.: this function will not be used for strings
    if ((type_p == kObdTypeDomain))
//         (bType_p == kObdTypeVString) /* ||
//         (bType_p == kObdTypeOString) ||
//         (bType_p == kObdTypeUString)    */ )
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
        pVarEntry_p->pData = &abEplObdTrashObject_g[0];
        pVarEntry_p->size  = obdSize_p;
    }

}


//---------------------------------------------------------------------------
//
// Function:    obd_getDataSize()
//
// Description: gets the data size of an object,
//              for string objects it returnes the string length
//              without terminating null-character
//
// Parameters:
//              index_p   =   Index
//              subIndex_p=   Subindex
//
// Return:      tObdSize
//
// State:
//
//---------------------------------------------------------------------------
tObdSize obd_getDataSize(UINT index_p, UINT subIndex_p)
{
tEplKernel          Ret;
tObdSize            ObdSize;
tObdEntryPtr        pObdEntry;
tObdSubEntryPtr     pObdSubEntry;


    // get pointer to index structure
    Ret = getIndexIntern (&m_ObdInitParam,
                                index_p,
                                &pObdEntry);
    if(Ret != kEplSuccessful)
    {
        ObdSize = 0;
        goto Exit;
    }

    // get pointer to subindex structure
    Ret = getSubindexIntern (pObdEntry,
                                subIndex_p,
                                &pObdSubEntry);
    if(Ret != kEplSuccessful)
    {
        ObdSize = 0;
        goto Exit;
    }

    // get size
    ObdSize = getDataSizeIntern (pObdSubEntry);
Exit:
    return ObdSize;
}
//---------------------------------------------------------------------------
//
// Function:    obd_getNodeId()
//
// Description: function returns nodeid from entry 0x1F93
//
//
// Parameters:
//
// Return:      UINT = Node Id
//
// State:
//
//---------------------------------------------------------------------------
UINT obd_getNodeId(void)
{
tEplKernel      Ret;
tObdSize        ObdSize;
BYTE            bNodeId;

    bNodeId = 0;
    ObdSize = sizeof(bNodeId);
    Ret = obd_readEntry(
                            OBD_NODE_ID_INDEX,
                            OBD_NODE_ID_SUBINDEX,
                            &bNodeId,
                            &ObdSize);
    if(Ret != kEplSuccessful)
    {
        bNodeId = EPL_C_ADR_INVALID;
        goto Exit;
    }

Exit:
    return (UINT) bNodeId;

}


//---------------------------------------------------------------------------
//
// Function:    obd_setNodeId()
//
// Description: function sets nodeid in entry 0x1F93
//
//
// Parameters:
//              uiNodeId_p  =   Node Id to set
//              NodeIdType_p=   Type on which way the Node Id was set
//
// Return:      tEplKernel = Errorcode
//
// State:
//
//---------------------------------------------------------------------------
tEplKernel obd_setNodeId(UINT nodeId_p, tObdNodeIdType nodeIdType_p)
{
tEplKernel  Ret;
tObdSize    ObdSize;
BYTE        fHwBool;
BYTE        bNodeId;

    // check Node Id
    if(nodeId_p == EPL_C_ADR_INVALID)
    {
        Ret = kEplInvalidNodeId;
        goto Exit;
    }
    bNodeId = (BYTE)nodeId_p;
    ObdSize = sizeof(BYTE);
    // write NodeId to OD entry
    Ret = obd_writeEntry(
                            OBD_NODE_ID_INDEX,
                            OBD_NODE_ID_SUBINDEX,
                            &bNodeId,
                            ObdSize);
    if(Ret != kEplSuccessful)
    {
        goto Exit;
    }

    // set HWBOOL-Flag in Subindex OBD_NODE_ID_HWBOOL_SUBINDEX
    switch (nodeIdType_p)
    {
        // type unknown
        case kObdNodeIdUnknown:
        {
            fHwBool = OBD_FALSE;
            break;
        }

        case kObdNodeIdSoftware:
        {
            fHwBool = OBD_FALSE;
            break;
        }

        case kObdNodeIdHardware:
        {
            fHwBool = OBD_TRUE;
            break;
        }

        default:
        {
            fHwBool = OBD_FALSE;
        }

    }   // end of switch (NodeIdType_p)

    // write flag
    ObdSize = sizeof(fHwBool);
    Ret = obd_writeEntry(
                            OBD_NODE_ID_INDEX,
                            OBD_NODE_ID_HWBOOL_SUBINDEX,
                            &fHwBool,
                            ObdSize);
    if(Ret != kEplSuccessful)
    {
        goto Exit;
    }

Exit:
    return Ret;
}

//---------------------------------------------------------------------------
//
// Function:    obd_isNumerical()
//
// Description: function checks if a entry is numerical or not
//
//
// Parameters:
//              index_p           = Index
//              subIndex_p        = Subindex
//              pfEntryNumerical_p  = pointer to BOOL for returnvalue
//                                  -> TRUE if entry a numerical value
//                                  -> FALSE if entry not a numerical value
//
// Return:      tEplKernel = Errorcode
//
// State:
//
//---------------------------------------------------------------------------
tEplKernel obd_isNumerical(UINT index_p, UINT subIndex_p, BOOL* pfEntryNumerical_p)
{
tEplKernel          Ret;
tObdEntryPtr        pObdEntry;
tObdSubEntryPtr     pObdSubEntry;


    // get pointer to index structure
    Ret = getIndexIntern (&m_ObdInitParam,
                                index_p,
                                &pObdEntry);
    if(Ret != kEplSuccessful)
    {
        goto Exit;
    }

    // get pointer to subindex structure
    Ret = getSubindexIntern (pObdEntry,
                                subIndex_p,
                                &pObdSubEntry);
    if(Ret != kEplSuccessful)
    {
        goto Exit;
    }

    Ret = isNumericalIntern(pObdSubEntry, pfEntryNumerical_p);


Exit:
    return Ret;

}


//---------------------------------------------------------------------------
//
// Function:    obd_getType()
//
// Description: function returns the data type of the specified entry
//
//
// Parameters:
//              index_p           = Index
//              subIndex_p        = Subindex
//              pType_p             = pointer to tObdType for returnvalue
//
// Return:      tEplKernel = Errorcode
//
// State:
//
//---------------------------------------------------------------------------
tEplKernel obd_getType(UINT index_p, UINT subIndex_p, tObdType* pType_p)
{
tEplKernel          Ret;
tObdEntryPtr        pObdEntry;
tObdSubEntryPtr     pObdSubEntry;


    // get pointer to index structure
    Ret = getIndexIntern (&m_ObdInitParam,
                                index_p,
                                &pObdEntry);
    if(Ret != kEplSuccessful)
    {
        goto Exit;
    }

    // get pointer to subindex structure
    Ret = getSubindexIntern (pObdEntry,
                                subIndex_p,
                                &pObdSubEntry);
    if(Ret != kEplSuccessful)
    {
        goto Exit;
    }

    *pType_p = pObdSubEntry->type;

Exit:
    return Ret;

}


//---------------------------------------------------------------------------
//
// Function:    obd_readEntryToLe()
//
// Description: The function reads an object entry from the byteoder
//              of the system to the little endian byteorder for numerical values.
//              For other types a normal read will be processed. This is usefull for
//              the PDO and SDO module. The application
//              can always read the data even if attrib kObdAccRead
//              is not set. The attrib is only checked up for SDO transfer.
//
// Parameters:
//              index_p       = Index of the OD entry to read
//              subIndex_p    = Subindex to read
//              pDstData_p      = pointer to the buffer for data
//              Offset_p        = offset in data for read access
//              pSize_p         = IN: Size of the buffer
//                                OUT: number of readed Bytes
//
// Return:      tEplKernel
//
// State:
//
//---------------------------------------------------------------------------
tEplKernel obd_readEntryToLe (UINT index_p, UINT subIndex_p, void* pDstData_p, tObdSize* pSize_p)
{
tEplKernel                      Ret;
tObdEntryPtr                    pObdEntry;
tObdSubEntryPtr                 pSubEntry;
tObdCbParam  MEM                CbParam;
void *                          pSrcData;
tObdSize                        ObdSize;

    ASSERT (pDstData_p != NULL);
    ASSERT (pSize_p != NULL);

    // get address of index and subindex entry
    Ret = getEntry (
        index_p, subIndex_p, &pObdEntry, &pSubEntry);
    if (Ret != kEplSuccessful)
    {
        goto Exit;
    }

    // get pointer to object data
     pSrcData = getObjectDataPtrIntern (pSubEntry);

    // check source pointer
    if (pSrcData == NULL)
    {
        Ret = kEplObdReadViolation;
        goto Exit;
    }

    //------------------------------------------------------------------------
    // address of source data to structure of callback parameters
    // so callback function can change this data before reading
    CbParam.index = index_p;
    CbParam.subIndex = subIndex_p;
    CbParam.pArg = pSrcData;
    CbParam.obdEvent = kObdEvPreRead;
    Ret = callObjectCallback (
        pObdEntry->pfnCallback, &CbParam);
    if (Ret != kEplSuccessful)
    {
        goto Exit;
    }

    // get size of data and check if application has reserved enough memory
    ObdSize = getDataSizeIntern (pSubEntry);

    // check if offset given and calc correct number of bytes to read
    if (*pSize_p < ObdSize)
    {
        Ret = kEplObdValueLengthError;
        goto Exit;
    }

    // check if numerical type
    switch(pSubEntry->type)
    {
        //-----------------------------------------------
        // types without ami
        case kObdTypeVString:
        case kObdTypeOString:
        case kObdTypeDomain:
        default:
        {
            // read value from object
            EPL_MEMCPY (pDstData_p, pSrcData, ObdSize);

            if (pSubEntry->type == kObdTypeVString)
            {
                if (*pSize_p > ObdSize)
                {   // space left to set the terminating null-character
                    ((char MEM*) pDstData_p)[ObdSize] = '\0';
                    ObdSize++;
                }
            }
            break;
        }

        //-----------------------------------------------
        // numerical type which needs ami-write
        // 8 bit or smaller values
        case kObdTypeBool:
        case kObdTypeInt8:
        case kObdTypeUInt8:
        {
            AmiSetByteToLe(pDstData_p, *((BYTE*)pSrcData));
            break;
        }

        // 16 bit values
        case kObdTypeInt16:
        case kObdTypeUInt16:
        {
            AmiSetWordToLe(pDstData_p, *((WORD*)pSrcData));
            break;
        }

        // 24 bit values
        case kObdTypeInt24:
        case kObdTypeUInt24:
        {
            AmiSetDword24ToLe(pDstData_p, *((DWORD*)pSrcData));
            break;
        }

        // 32 bit values
        case kObdTypeInt32:
        case kObdTypeUInt32:
        case kObdTypeReal32:
        {
            AmiSetDwordToLe(pDstData_p, *((DWORD*)pSrcData));
            break;
        }

        // 40 bit values
        case kObdTypeInt40:
        case kObdTypeUInt40:
        {
            AmiSetQword40ToLe(pDstData_p, *((QWORD*)pSrcData));
            break;
        }

        // 48 bit values
        case kObdTypeInt48:
        case kObdTypeUInt48:
        {
            AmiSetQword48ToLe(pDstData_p, *((QWORD*)pSrcData));
            break;
        }

        // 56 bit values
        case kObdTypeInt56:
        case kObdTypeUInt56:
        {
            AmiSetQword56ToLe(pDstData_p, *((QWORD*)pSrcData));
            break;
        }

        // 64 bit values
        case kObdTypeInt64:
        case kObdTypeUInt64:
        case kObdTypeReal64:
        {
            AmiSetQword64ToLe(pDstData_p, *((QWORD*)pSrcData));
            break;
        }

        // time of day
        case kObdTypeTimeOfDay:
        case kObdTypeTimeDiff:
        {
            AmiSetTimeOfDay(pDstData_p, ((tTimeOfDay*)pSrcData));
            break;
        }

    }// end of switch(pSubEntry->m_Type)

    *pSize_p = ObdSize;


    // write address of destination data to structure of callback parameters
    // so callback function can change this data after reading
    CbParam.pArg     = pDstData_p;
    CbParam.obdEvent = kObdEvPostRead;
    Ret = callObjectCallback (
        pObdEntry->pfnCallback, &CbParam);

Exit:

    return Ret;

}

//---------------------------------------------------------------------------
//
// Function:    obd_writeEntryFromLe()
//
// Description: Function writes data to an OBD entry from a source with
//              little endian byteorder to the od with system specuific
//              byteorder. Not numerical values will only by copied. Strings
//              are stored with added '\0' character.
//
// Parameters:
//              index_p       =   Index of the OD entry
//              subIndex_p    =   Subindex of the OD Entry
//              pSrcData_p      =   Pointer to the data to write
//              Size_p          =   Size of the data in Byte
//
// Return:      tEplKernel      =   Errorcode
//
//
// State:
//
//---------------------------------------------------------------------------
tEplKernel obd_writeEntryFromLe (UINT index_p, UINT subIndex_p, void* pSrcData_p, tObdSize size_p)
{
tEplKernel              Ret;
tObdEntryPtr            pObdEntry;
tObdSubEntryPtr         pSubEntry;
tObdCbParam MEM         CbParam;
void MEM*               pDstData;
tObdSize                ObdSize;
QWORD                   qwBuffer;
void*                   pBuffer = &qwBuffer;


    Ret = writeEntryPre (
                               index_p,
                               subIndex_p,
                               pSrcData_p,
                               &pDstData,
                               size_p,
                               &pObdEntry,
                               &pSubEntry,
                               &CbParam,
                               &ObdSize);
    if (Ret != kEplSuccessful)
    {
        goto Exit;
    }


    // check if numerical type
    switch(pSubEntry->type)
    {
        //-----------------------------------------------
        // types without ami
        default:
        {   // do nothing, i.e. use the given source pointer
            pBuffer = pSrcData_p;
            break;
        }

        //-----------------------------------------------
        // numerical type which needs ami-write
        // 8 bit or smaller values
        case kObdTypeBool:
        case kObdTypeInt8:
        case kObdTypeUInt8:
        {
            *((BYTE*)pBuffer) = AmiGetByteFromLe(pSrcData_p);
            break;
        }

        // 16 bit values
        case kObdTypeInt16:
        case kObdTypeUInt16:
        {
            *((WORD*)pBuffer) = AmiGetWordFromLe(pSrcData_p);
            break;
        }

        // 24 bit values
        case kObdTypeInt24:
        case kObdTypeUInt24:
        {
            *((DWORD*)pBuffer) = AmiGetDword24FromLe(pSrcData_p);
            break;
        }

        // 32 bit values
        case kObdTypeInt32:
        case kObdTypeUInt32:
        case kObdTypeReal32:
        {
            *((DWORD*)pBuffer) = AmiGetDwordFromLe(pSrcData_p);
            break;
        }

        // 40 bit values
        case kObdTypeInt40:
        case kObdTypeUInt40:
        {
            *((QWORD*)pBuffer) = AmiGetQword40FromLe(pSrcData_p);
            break;
        }

        // 48 bit values
        case kObdTypeInt48:
        case kObdTypeUInt48:
        {
            *((QWORD*)pBuffer) = AmiGetQword48FromLe(pSrcData_p);
            break;
        }

        // 56 bit values
        case kObdTypeInt56:
        case kObdTypeUInt56:
        {
            *((QWORD*)pBuffer) = AmiGetQword56FromLe(pSrcData_p);
            break;
        }

        // 64 bit values
        case kObdTypeInt64:
        case kObdTypeUInt64:
        case kObdTypeReal64:
        {
            *((QWORD*)pBuffer) = AmiGetQword64FromLe(pSrcData_p);
            break;
        }

        // time of day
        case kObdTypeTimeOfDay:
        case kObdTypeTimeDiff:
        {
            AmiGetTimeOfDay(pBuffer, ((tTimeOfDay*)pSrcData_p));
            break;
        }

    }// end of switch(pSubEntry->m_Type)


    Ret = writeEntryPost (
                                pObdEntry,
                                pSubEntry,
                                &CbParam,
                                pBuffer,
                                pDstData,
                                ObdSize);
    if (Ret != kEplSuccessful)
    {
        goto Exit;
    }

Exit:

    return Ret;

}

//---------------------------------------------------------------------------
//
// Function:    obd_getAccessType()
//
// Description: Function returns accesstype of the entry
//
// Parameters:
//              index_p       =   Index of the OD entry
//              subIndex_p    =   Subindex of the OD Entry
//              pAccessTyp_p    =   pointer to buffer to store accesstype
//
// Return:      tEplKernel     =   errorcode
//
//
// State:
//
//---------------------------------------------------------------------------
tEplKernel obd_getAccessType(UINT index_p, UINT subIndex_p, tObdAccess* pAccessTyp_p)

{
tEplKernel          Ret;
tObdEntryPtr        pObdEntry;
tObdSubEntryPtr     pObdSubEntry;


    // get pointer to index structure
    Ret = getIndexIntern (&m_ObdInitParam,
                                index_p,
                                &pObdEntry);
    if(Ret != kEplSuccessful)
    {
        goto Exit;
    }

    // get pointer to subindex structure
    Ret = getSubindexIntern (pObdEntry,
                                subIndex_p,
                                &pObdSubEntry);
    if(Ret != kEplSuccessful)
    {
        goto Exit;
    }

    // get accessType
    *pAccessTyp_p = pObdSubEntry->access;


Exit:
    return Ret;
}

//---------------------------------------------------------------------------
//
// Function:    obd_searchVarEntry()
//
// Description: gets variable from OD
//
// Parameters:  index_p       =   index of the var entry to search
//              uiSubindex_p    =   subindex of var entry to search
//              ppVarEntry_p    =   pointer to the pointer to the varentry
//
// Return:      tEplKernel
//
// State:
//
//---------------------------------------------------------------------------

tEplKernel obd_searchVarEntry (UINT index_p, UINT subindex_p, tObdVarEntry MEM** ppVarEntry_p)
{

tEplKernel           Ret;
tObdSubEntryPtr      pSubindexEntry;

    // get address of subindex entry
    Ret = getEntry (
        index_p, subindex_p, NULL, &pSubindexEntry);
    if (Ret == kEplSuccessful)
    {
        // get var entry
        Ret = getVarEntry (pSubindexEntry, ppVarEntry_p);
    }

    return Ret;

}
//=========================================================================//
//                                                                         //
//          P R I V A T E   D E F I N I T I O N S                          //
//                                                                         //
//=========================================================================//

//---------------------------------------------------------------------------
//
// Function:    callObjectCallback()
//
// Description: calls callback function of an object or of a variable
//
// Parameters:  pfnCallback_p
//              pCbParam_p
//
// Return:      tEplKernel
//
// State:
//
//---------------------------------------------------------------------------

static tEplKernel callObjectCallback(tObdCallback pfnCallback_p, tObdCbParam MEM* pCbParam_p)
{

tEplKernel           Ret;
tObdCallback MEM     pfnCallback;

    ASSERT (pCbParam_p != NULL);

    Ret = kEplSuccessful;

    // check address of callback function before calling it
    if (pfnCallback_p != NULL)
    {
        // KEIL C51 V6.01 has a bug.
        // Therefore the parameter fpCallback_p has to be copied in local variable fpCallback.
        pfnCallback = pfnCallback_p;

        // call callback function for this object
        Ret = pfnCallback (pCbParam_p);
    }

    return Ret;
}

//---------------------------------------------------------------------------
//
// Function:    getDataSizeIntern()
//
// Description: gets the data size of an object.
//              for string objects it returnes the string length
//              without terminating null-character
//
// Parameters:  pSubIndexEntry_p
//
// Return:      tObdSize
//
// State:
//
//---------------------------------------------------------------------------

static tObdSize getDataSizeIntern(tObdSubEntryPtr pSubIndexEntry_p)
{

tObdSize    DataSize;
void MEM*   pData;

    // If OD entry is defined by macro EPL_OBD_SUBINDEX_ROM_VSTRING
    // then the current pointer is always NULL. The function
    // returns the length of default string.
    DataSize = getObjectSize (pSubIndexEntry_p);

    if (pSubIndexEntry_p->type == kObdTypeVString)
    {
        // The pointer to current value can be received from getObjectCurrentPtr()
        pData = ((void MEM*) getObjectCurrentPtr (pSubIndexEntry_p));
        if (pData != NULL)
        {
            DataSize = getObdStringLen ((void *) pData, DataSize, pSubIndexEntry_p->type);
        }

    }

    return DataSize;

}


//---------------------------------------------------------------------------
//
// Function:    getObdStringLen()
//
// Description: The function calculates the length of string. The '\0'
//              character is NOT included!!
//
// Parameters:  pObjData_p          = pointer to string
//              objLen_p            = max. length of object entry
//              bobjType_p          = object type (VSTRING, ...)
//
// Returns:     string length
//
// State:
//
//---------------------------------------------------------------------------

static tObdSize getObdStringLen(void* pObjData_p, tObdSize objLen_p, tObdType objType_p)
{

tObdSize    StrLen = 0;
BYTE *  pbString;

    if (pObjData_p == NULL)
    {
        goto Exit;
    }

    //----------------------------------------
    // Visible String: data format byte
    if (objType_p == kObdTypeVString)
    {
        pbString = pObjData_p;

        for (StrLen = 0; StrLen < objLen_p; StrLen++)
        {
            if (*pbString == '\0')
            {
//                StrLen++;
                break;
            }

            pbString++;
        }
    }

    //----------------------------------------
    // other string types ...

Exit:
    return (StrLen);

}



#if (EPL_OBD_CHECK_OBJECT_RANGE != FALSE)

//---------------------------------------------------------------------------
//
// Function:    checkObjectRange()
//
// Description: function to check value range of object data
//
// NOTICE: The pointer of data (pData_p) must point out to an even address,
//         if ObjType is unequal to kObdTypeInt8 or kObdTypeUInt8! But it is
//         always realiced because pointer m_pDefault points always to an
//         array of the SPECIFIED type.
//
// Parameters:  pSubindexEntry_p
//              pData_p
//
// Return:      tEplKernel
//
// State:
//
//---------------------------------------------------------------------------

static tEplKernel checkObjectRange (tObdSubEntryPtr pSubindexEntry_p, void * pData_p)
{

tEplKernel      Ret;
const void *   pRangeData;

    ASSERTMSG (pSubindexEntry_p != NULL,
        "checkObjectRange(): no address to subindex struct!\n");

    Ret  = kEplSuccessful;

    // check if data range has to be checked
    if ((pSubindexEntry_p->m_Access & kObdAccRange) == 0)
    {
        goto Exit;
    }

    // get address of default data
    pRangeData = pSubindexEntry_p->m_pDefault;

    // jump to called object type
    switch ((tObdType) pSubindexEntry_p->m_Type)
    {
        // -----------------------------------------------------------------
        // ObdType kObdTypeBool will not be checked because there are only
        // two possible values 0 or 1.

        // -----------------------------------------------------------------
        // ObdTypes which has to be check up because numerical values
        case kObdTypeInt8:

            // switch to lower limit
            pRangeData = ((tObdInteger8 *) pRangeData) + 1;

            // check if value is to low
            if (*((tObdInteger8 *) pData_p) < *((tObdInteger8 *) pRangeData))
            {
                Ret = kEplObdValueTooLow;
                break;
            }

            // switch to higher limit
            pRangeData = ((tObdInteger8 *) pRangeData) + 1;

            // check if value is to high
            if (*((tObdInteger8 *) pData_p) > *((tObdInteger8 *) pRangeData))
            {
                Ret = kEplObdValueTooHigh;
            }

            break;

        case kObdTypeUInt8:

            // switch to lower limit
            pRangeData = ((tObdUnsigned8 *) pRangeData) + 1;

            // check if value is to low
            if (*((tObdUnsigned8 *) pData_p) < *((tObdUnsigned8 *) pRangeData))
            {
                Ret = kEplObdValueTooLow;
                break;
            }

            // switch to higher limit
            pRangeData = ((tObdUnsigned8*) pRangeData) + 1;

            // check if value is to high
            if (*((tObdUnsigned8 *) pData_p) > *((tObdUnsigned8 *) pRangeData))
            {
                Ret = kEplObdValueTooHigh;
            }

            break;

        case kObdTypeInt16:

            // switch to lower limit
            pRangeData = ((tObdInteger16 *) pRangeData) + 1;

            // check if value is to low
            if (*((tObdInteger16 *) pData_p) < *((tObdInteger16 *) pRangeData))
            {
                Ret = kEplObdValueTooLow;
                break;
            }

            // switch to higher limit
            pRangeData = ((tObdInteger16 *) pRangeData) + 1;

            // check if value is to high
            if (*((tObdInteger16 *) pData_p) > *((tObdInteger16 *) pRangeData))
            {
                Ret = kEplObdValueTooHigh;
            }

            break;

        case kObdTypeUInt16:

            // switch to lower limit
            pRangeData = ((tObdUnsigned16 *) pRangeData) + 1;

            // check if value is to low
            if (*((tObdUnsigned16 *) pData_p) < *((tObdUnsigned16 *) pRangeData))
            {
                Ret = kEplObdValueTooLow;
                break;
            }

            // switch to higher limit
            pRangeData = ((tObdUnsigned16 *) pRangeData) + 1;

            // check if value is to high
            if (*((tObdUnsigned16 *) pData_p) > *((tObdUnsigned16 *) pRangeData))
            {
                Ret = kEplObdValueTooHigh;
            }

            break;

        case kObdTypeInt32:

            // switch to lower limit
            pRangeData = ((tObdInteger32 *) pRangeData) + 1;

            // check if value is to low
            if (*((tObdInteger32 *) pData_p) < *((tObdInteger32 *) pRangeData))
            {
                Ret = kEplObdValueTooLow;
                break;
            }

            // switch to higher limit
            pRangeData = ((tObdInteger32 *) pRangeData) + 1;

            // check if value is to high
            if (*((tObdInteger32 *) pData_p) > *((tObdInteger32 *) pRangeData))
            {
                Ret = kEplObdValueTooHigh;
            }

            break;

        case kObdTypeUInt32:

            // switch to lower limit
            pRangeData = ((tObdUnsigned32 *) pRangeData) + 1;

            // check if value is to low
            if (*((tObdUnsigned32 *) pData_p) < *((tObdUnsigned32 *) pRangeData))
            {
                Ret = kEplObdValueTooLow;
                break;
            }

            // switch to higher limit
            pRangeData = ((tObdUnsigned32 *) pRangeData) + 1;

            // check if value is to high
            if (*((tObdUnsigned32 *) pData_p) > *((tObdUnsigned32 *) pRangeData))
            {
                Ret = kEplObdValueTooHigh;
            }

            break;

        case kObdTypeReal32:

            // switch to lower limit
            pRangeData = ((tObdReal32 *) pRangeData) + 1;

            // check if value is to low
            if (*((tObdReal32 *) pData_p) < *((tObdReal32 *) pRangeData))
            {
                Ret = kEplObdValueTooLow;
                break;
            }

            // switch to higher limit
            pRangeData = ((tObdReal32 *) pRangeData) + 1;

            // check if value is to high
            if (*((tObdReal32 *) pData_p) > *((tObdReal32 *) pRangeData))
            {
                Ret = kEplObdValueTooHigh;
            }

            break;

        // -----------------------------------------------------------------
        case kObdTypeInt40:
        case kObdTypeInt48:
        case kObdTypeInt56:
        case kObdTypeInt64:

            // switch to lower limit
            pRangeData = ((signed QWORD *) pRangeData) + 1;

            // check if value is to low
            if (*((signed QWORD *) pData_p) < *((signed QWORD *) pRangeData))
            {
                Ret = kEplObdValueTooLow;
                break;
            }

            // switch to higher limit
            pRangeData = ((signed QWORD *) pRangeData) + 1;

            // check if value is to high
            if (*((signed QWORD *) pData_p) > *((signed QWORD *) pRangeData))
            {
                Ret = kEplObdValueTooHigh;
            }

            break;

        // -----------------------------------------------------------------
        case kObdTypeUInt40:
        case kObdTypeUInt48:
        case kObdTypeUInt56:
        case kObdTypeUInt64:

            // switch to lower limit
            pRangeData = ((unsigned QWORD *) pRangeData) + 1;

            // check if value is to low
            if (*((unsigned QWORD *) pData_p) < *((unsigned QWORD *) pRangeData))
            {
                Ret = kEplObdValueTooLow;
                break;
            }

            // switch to higher limit
            pRangeData = ((unsigned QWORD *) pRangeData) + 1;

            // check if value is to high
            if (*((unsigned QWORD *) pData_p) > *((unsigned QWORD *) pRangeData))
            {
                Ret = kEplObdValueTooHigh;
            }

            break;

        // -----------------------------------------------------------------
        case kObdTypeReal64:

            // switch to lower limit
            pRangeData = ((tObdReal64 *) pRangeData) + 1;

            // check if value is to low
            if (*((tObdReal64 *) pData_p) < *((tObdReal64 *) pRangeData))
            {
                Ret = kEplObdValueTooLow;
                break;
            }

            // switch to higher limit
            pRangeData = ((tObdReal64 *) pRangeData) + 1;

            // check if value is to high
            if (*((tObdReal64 *) pData_p) > *((tObdReal64 *) pRangeData))
            {
                Ret = kEplObdValueTooHigh;
            }

            break;

        // -----------------------------------------------------------------
        case kObdTypeTimeOfDay:
        case kObdTypeTimeDiff:
            break;

        // -----------------------------------------------------------------
        // ObdTypes kObdTypeXString and kObdTypeDomain can not be checkt because
        // they have no numerical value.
        default:

            Ret = kEplObdUnknownObjectType;
            break;
    }

Exit:

    return Ret;

}
#endif // (EPL_OBD_CHECK_OBJECT_RANGE != FALSE)

//---------------------------------------------------------------------------
//
// Function:    writeEntryPre()
//
// Description: Function prepares write of data to an OBD entry. Strings
//              are stored with added '\0' character.
//
// Parameters:
//              index_p       =   Index of the OD entry
//              subIndex_p    =   Subindex of the OD Entry
//              pSrcData_p      =   Pointer to the data to write
//              Size_p          =   Size of the data in Byte
//
// Return:      tEplKernel      =   Errorcode
//
//
// State:
//
//---------------------------------------------------------------------------

static tEplKernel writeEntryPre (UINT index_p, UINT subIndex_p, void* pSrcData_p,
                                 void** ppDstData_p, tObdSize Size_p, tObdEntryPtr* ppObdEntry_p,
                                 tObdSubEntryPtr* ppSubEntry_p, tObdCbParam MEM* pCbParam_p,
                                 tObdSize*  pObdSize_p)
{

tEplKernel              Ret;
tObdEntryPtr            pObdEntry;
tObdSubEntryPtr         pSubEntry;
tObdAccess              Access;
void MEM*               pDstData;
tObdSize                ObdSize;
BOOL                    fEntryNumerical;

#if (EPL_OBD_USE_STRING_DOMAIN_IN_RAM != FALSE)
    tObdVStringDomain MEM       MemVStringDomain;
    void MEM*                   pCurrData;
#endif

    ASSERT (pSrcData_p != NULL);    // should never be NULL

    //------------------------------------------------------------------------
    // get address of index and subindex entry
    Ret = getEntry (
        index_p, subIndex_p, &pObdEntry, &pSubEntry);
    if (Ret != kEplSuccessful)
    {
        goto Exit;
    }

    Access = (tObdAccess) pSubEntry->access;

    // check access for write
    if ((Access & kObdAccConst) != 0)
    {
        Ret = kEplObdAccessViolation;
        goto Exit;
    }

    //------------------------------------------------------------------------
    // To use the same callback function for ObdWriteEntry as well as for
    // an SDO download call at first (kObdEvPre...) the callback function
    // with the argument pointer to object size.
    pCbParam_p->index    = index_p;
    pCbParam_p->subIndex = subIndex_p;

    // Because object size and object pointer are
    // adapted by user callback function, re-read
    // this values.
    ObdSize = getObjectSize (pSubEntry);
    // get pointer to object data
    pDstData = (void MEM*) getObjectDataPtrIntern (pSubEntry);

    // 09-dec-2004 r.d.:
    //      Function obd_writeEntry() calls new event kObdEvWrStringDomain
    //      for String or Domain which lets called module directly change
    //      the data pointer or size. This prevents a recursive call to
    //      the callback function if it calls getEntry().
    #if (EPL_OBD_USE_STRING_DOMAIN_IN_RAM != FALSE)
    if ( (pSubEntry->type == kObdTypeVString) ||
         (pSubEntry->type == kObdTypeDomain)  ||
         (pSubEntry->type == kObdTypeOString))
    {
        if (pSubEntry->type == kObdTypeVString)
        {
            // reserve one byte for 0-termination
            // -as ObdSize -= 1;
            Size_p += 1;
        }

        // fill out new arg-struct
        MemVStringDomain.downloadSize = Size_p;
        MemVStringDomain.objSize      = ObdSize;
        MemVStringDomain.pData        = pDstData;

        pCbParam_p->obdEvent = kObdEvWrStringDomain;
        pCbParam_p->pArg     = &MemVStringDomain;
        //  call user callback
        Ret = callObjectCallback (
                pObdEntry->pfnCallback, pCbParam_p);
        if (Ret != kEplSuccessful)
        {
            goto Exit;
        }

        // write back new settings
        pCurrData = pSubEntry->pCurrent;
        if ((pSubEntry->type == kObdTypeVString)
            ||(pSubEntry->type ==  kObdTypeOString))
        {
            ((tObdVString MEM*) pCurrData)->size    = MemVStringDomain.objSize;
            ((tObdVString MEM*) pCurrData)->pString = MemVStringDomain.pData;
        }
        else // if (pSdosTableEntry_p->m_bObjType == kObdTypeDomain)
        {
        tObdVarEntry MEM*    pVarEntry = NULL;

            Ret = getVarEntry(pSubEntry, &pVarEntry);
            if (Ret != kEplSuccessful)
            {
                goto Exit;
            }
            if (pVarEntry == NULL)
            {
                Ret = kEplObdAccessViolation;
                goto Exit;
            }

            pVarEntry->size  = MemVStringDomain.objSize;
            pVarEntry->pData = (void MEM*) MemVStringDomain.pData;
        }

        // Because object size and object pointer are
        // adapted by user callback function, re-read
        // this values.
        ObdSize  = MemVStringDomain.objSize;
        pDstData = (void MEM*) MemVStringDomain.pData;
    }
    #endif //#if (OBD_USE_STRING_DOMAIN_IN_RAM != FALSE)

    // access violation if adress to current value is NULL
    if (pDstData == NULL)
    {
        Ret = kEplObdAccessViolation;
        goto Exit;
    }

    // 07-dec-2004 r.d.: size from application is needed because callback function can change the object size
    // -as 16.11.04 CbParam.m_pArg     = &ObdSize;
    // 09-dec-2004 r.d.: CbParam.m_pArg     = &Size_p;
    pCbParam_p->pArg     = &ObdSize;
    pCbParam_p->obdEvent = kObdEvInitWrite;
    Ret = callObjectCallback (
        pObdEntry->pfnCallback, pCbParam_p);
    if (Ret != kEplSuccessful)
    {
        goto Exit;
    }

    if (Size_p > ObdSize)
    {
        Ret = kEplObdValueLengthError;
        goto Exit;
    }

    if (pSubEntry->type == kObdTypeVString)
    {
        if (((char MEM*) pSrcData_p)[Size_p - 1] == '\0')
        {   // last byte of source string contains null character

            // reserve one byte in destination for 0-termination
            Size_p  -= 1;
        }
        else if (Size_p >= ObdSize)
        {   // source string is not 0-terminated
            // and destination buffer is too short
            Ret = kEplObdValueLengthError;
            goto Exit;
        }
    }

    Ret = isNumericalIntern(pSubEntry, &fEntryNumerical);
    if (Ret != kEplSuccessful)
    {
        goto Exit;
    }

    if ((fEntryNumerical != FALSE)
        && (Size_p != ObdSize))
    {
        // type is numerical, therefore size has to fit, but it does not.
        Ret = kEplObdValueLengthError;
        goto Exit;
    }

    // use given size, because non-numerical objects can be written with shorter values
    ObdSize = Size_p;

    // set output parameters
    *pObdSize_p = ObdSize;
    *ppObdEntry_p = pObdEntry;
    *ppSubEntry_p = pSubEntry;
    *ppDstData_p = pDstData;

    // all checks are done
    // the caller may now convert the numerical source value to platform byte order in a temporary buffer

Exit:

    return Ret;

}


//---------------------------------------------------------------------------
//
// Function:    writeEntryPost()
//
// Description: Function finishes write of data to an OBD entry. Strings
//              are stored with added '\0' character.
//
// Parameters:
//              index_p       =   Index of the OD entry
//              subIndex_p    =   Subindex of the OD Entry
//              pSrcData_p      =   Pointer to the data to write
//              Size_p          =   Size of the data in Byte
//
// Return:      tEplKernel      =   Errorcode
//
//
// State:
//
//---------------------------------------------------------------------------

static tEplKernel writeEntryPost (tObdEntryPtr pObdEntry_p, tObdSubEntryPtr pSubEntry_p,
                                  tObdCbParam MEM* pCbParam_p, void* pSrcData_p,
                                  void* pDstData_p, tObdSize obdSize_p)
{

tEplKernel              Ret;


    // caller converted the source value to platform byte order
    // now the range of the value may be checked

    #if (EPL_OBD_CHECK_OBJECT_RANGE != FALSE)
    {
        // check data range
        Ret = checkObjectRange (pSubEntry_p, pSrcData_p);
        if (Ret != kEplSuccessful)
        {
            goto Exit;
        }
    }
    #endif

    // now call user callback function to check value
    // write address of source data to structure of callback parameters
    // so callback function can check this data
    pCbParam_p->pArg     = pSrcData_p;
    pCbParam_p->obdEvent = kObdEvPreWrite;
    Ret = callObjectCallback (
        pObdEntry_p->pfnCallback, pCbParam_p);
    if (Ret != kEplSuccessful)
    {
        goto Exit;
    }

    // copy object data to OBD
    EPL_MEMCPY (pDstData_p, pSrcData_p, obdSize_p);

    // terminate string with 0
    if (pSubEntry_p->type == kObdTypeVString)
    {
        ((char MEM*) pDstData_p)[obdSize_p] = '\0';
    }

    // write address of destination to structure of callback parameters
    // so callback function can change data subsequently
    pCbParam_p->pArg     = pDstData_p;
    pCbParam_p->obdEvent = kObdEvPostWrite;
    Ret = callObjectCallback (
        pObdEntry_p->pfnCallback, pCbParam_p);

Exit:

    return Ret;

}


//---------------------------------------------------------------------------
//
// Function:    getObjectSize()
//
// Description: function to get size of object
//              The function determines if an object type an fixed data type (BYTE, WORD, ...)
//              or non fixed object (string, domain). This information is used to decide
//              if download data are stored temporary or not. For objects with fixed data length
//              and types a value range checking can process.
//              For strings the function returns the whole object size not the
//              length of string.
//
// Parameters:  pSubIndexEntry_p
//
// Return:      tObdSize
//
// State:
//
//---------------------------------------------------------------------------

static tObdSize getObjectSize(tObdSubEntryPtr pSubIndexEntry_p)
{

tObdSize DataSize = 0;
void * pData;

    switch (pSubIndexEntry_p->type)
    {
        // -----------------------------------------------------------------
        case kObdTypeBool:

            DataSize = 1;
            break;

        // -----------------------------------------------------------------
        // ObdTypes which has to be check because numerical values
        case kObdTypeInt8:
            DataSize = sizeof (tObdInteger8);
            break;

        // -----------------------------------------------------------------
        case kObdTypeUInt8:
            DataSize = sizeof (tObdUnsigned8);
            break;

        // -----------------------------------------------------------------
        case kObdTypeInt16:
            DataSize = sizeof (tObdInteger16);
            break;

        // -----------------------------------------------------------------
        case kObdTypeUInt16:
            DataSize = sizeof (tObdUnsigned16);
            break;

        // -----------------------------------------------------------------
        case kObdTypeInt32:
            DataSize = sizeof (tObdInteger32);
            break;

        // -----------------------------------------------------------------
        case kObdTypeUInt32:
            DataSize = sizeof (tObdUnsigned32);
            break;

        // -----------------------------------------------------------------
        case kObdTypeReal32:
            DataSize = sizeof (tObdReal32);
            break;

        // -----------------------------------------------------------------
        // ObdTypes which has to be not checked because not NUM values
        case kObdTypeDomain:
        {
        tObdVarEntry MEM*       pVarEntry = NULL;
        tEplKernel              Ret;

            Ret = getVarEntry(pSubIndexEntry_p, &pVarEntry);
            if ((Ret == kEplSuccessful)
                && (pVarEntry != NULL))
            {
                DataSize = pVarEntry->size;
            }
            break;
        }

        // -----------------------------------------------------------------
        case kObdTypeVString:
        //case kObdTypeUString:

            // If OD entry is defined by macro EPL_OBD_SUBINDEX_ROM_VSTRING
            // then the current pointer is always NULL. The function
            // returns the length of default string.
            pData = (void *) pSubIndexEntry_p->pCurrent;
            if ((void MEM*) pData != (void MEM*) NULL)
            {
                // The max. size of strings defined by STRING-Macro is stored in
                // tObdVString of current value.
                // (types tObdVString, tObdOString and tEplObdUString has the same members)
                DataSize = ((tObdVString MEM*) pData)->size;
            }
            else
            {
                // The current position is not declared. The string
                // is located in ROM, therefore use default pointer.
                pData = (void *) pSubIndexEntry_p->pDefault;
                if ((CONST void ROM*) pData != (CONST void ROM*) NULL)
                {
                   // The max. size of strings defined by STRING-Macro is stored in
                   // tObdVString of default value.
                   DataSize = ((CONST tObdVString ROM*) pData)->size;
                }
            }

            break;

        // -----------------------------------------------------------------
        case kObdTypeOString:

            pData = (void *) pSubIndexEntry_p->pCurrent;
            if ((void MEM*) pData != (void MEM*) NULL)
            {
                // The max. size of strings defined by STRING-Macro is stored in
                // tObdVString of current value.
                // (types tObdVString, tObdOString and tEplObdUString has the same members)
                DataSize = ((tObdOString MEM*) pData)->size;
            }
            else
            {
                // The current position is not declared. The string
                // is located in ROM, therefore use default pointer.
                pData = (void *) pSubIndexEntry_p->pDefault;
                if ((CONST void ROM*) pData != (CONST void ROM*) NULL)
                {
                   // The max. size of strings defined by STRING-Macro is stored in
                   // tObdVString of default value.
                   DataSize = ((CONST tObdOString ROM*) pData)->size;
                }
            }
            break;

        // -----------------------------------------------------------------
        case kObdTypeInt24:
        case kObdTypeUInt24:

            DataSize = 3;
            break;


        // -----------------------------------------------------------------
        case kObdTypeInt40:
        case kObdTypeUInt40:

            DataSize = 5;
            break;

        // -----------------------------------------------------------------
        case kObdTypeInt48:
        case kObdTypeUInt48:

            DataSize = 6;
            break;

        // -----------------------------------------------------------------
        case kObdTypeInt56:
        case kObdTypeUInt56:

            DataSize = 7;
            break;

        // -----------------------------------------------------------------
        case kObdTypeInt64:
        case kObdTypeUInt64:
        case kObdTypeReal64:

            DataSize = 8;
            break;

        // -----------------------------------------------------------------
        case kObdTypeTimeOfDay:
        case kObdTypeTimeDiff:

            DataSize = 6;
            break;

        // -----------------------------------------------------------------
        default:
            break;
    }

    return DataSize;
}

//---------------------------------------------------------------------------
//
// Function:    getObjectDefaultPtr()
//
// Description: function to get the default pointer (type specific)
//
// Parameters:  pSubIndexEntry_p    = pointer to subindex structure
//
// Returns:     (CONST void *)      = pointer to default value
//
// State:
//
//---------------------------------------------------------------------------

static CONST void* getObjectDefaultPtr(tObdSubEntryPtr pSubIndexEntry_p)
{

CONST void*     pDefault;
tObdType        Type;

    ASSERTMSG (pSubIndexEntry_p != NULL, "getObjectDefaultPtr(): pointer to SubEntry not valid!\n");

    // get address to default data from default pointer
    pDefault = pSubIndexEntry_p->pDefault;
    if (pDefault != NULL)
    {
        // there are some special types, whose default pointer always is NULL or has to get from other structure
        // get type from subindex structure
        Type = pSubIndexEntry_p->type;

        // check if object type is a string value
        if ((Type == kObdTypeVString) /* ||
            (Type == kObdTypeUString) */ )
        {

            // EPL_OBD_SUBINDEX_RAM_VSTRING
            //    tObdSize         m_Size;       --> size of default string
            //    char *    m_pDefString; --> pointer to  default string
            //    char *    m_pString;    --> pointer to string in RAM
            //
            pDefault = ((tObdVStringDef *) pDefault)->pDefString;
        }
        else if(Type == kObdTypeOString)
        {
             pDefault = ((tObdOStringDef *) pDefault)->pDefString;
        }
    }

    return pDefault;

}


//---------------------------------------------------------------------------
//
// Function:    getVarEntry()
//
// Description: gets a variable entry of an object
//
// Parameters:  pSubindexEntry_p
//              ppVarEntry_p
//
// Return:      tCopKernel
//
// State:
//
//---------------------------------------------------------------------------

static tEplKernel getVarEntry (tObdSubEntryPtr pSubindexEntry_p, tObdVarEntry MEM** ppVarEntry_p)
{

tEplKernel Ret = kEplObdVarEntryNotExist;

    ASSERT (ppVarEntry_p != NULL);   // is not allowed to be NULL
    ASSERT (pSubindexEntry_p != NULL);

    // check VAR-Flag - only this object points to variables
    if ((pSubindexEntry_p->access & kObdAccVar) != 0)
    {
        // check if object is an array
        if ((pSubindexEntry_p->access & kObdAccArray) != 0)
        {
            *ppVarEntry_p = &((tObdVarEntry MEM*) pSubindexEntry_p->pCurrent)[pSubindexEntry_p->subIndex - 1];
        }
        else
        {
            *ppVarEntry_p = (tObdVarEntry MEM*) pSubindexEntry_p->pCurrent;
        }

        Ret = kEplSuccessful;
    }

    return Ret;

}

//---------------------------------------------------------------------------
//
// Function:    getEntry()
//
// Description: gets a index entry from OD
//
// Parameters:  index_p       =   Index number
//              uiSubindex_p    =   Subindex number
//              ppObdEntry_p    =   pointer to the pointer to the entry
//              ppObdSubEntry_p =   pointer to the pointer to the subentry
//
// Return:      tEplKernel

//
// State:
//
//---------------------------------------------------------------------------

static tEplKernel getEntry(UINT index_p, UINT uiSubindex_p, tObdEntryPtr* ppObdEntry_p,
                           tObdSubEntryPtr* ppObdSubEntry_p)
{

tObdEntryPtr            pObdEntry;
tObdCbParam MEM         CbParam;
tEplKernel              Ret;

    //------------------------------------------------------------------------
    // get address of entry of index
    Ret = getIndexIntern (&m_ObdInitParam, index_p, &pObdEntry);
    if (Ret != kEplSuccessful)
    {
        goto Exit;
    }

    //------------------------------------------------------------------------
    // get address of entry of subindex
    Ret = getSubindexIntern (pObdEntry, uiSubindex_p, ppObdSubEntry_p);
    if (Ret != kEplSuccessful)
    {
        goto Exit;
    }

    //------------------------------------------------------------------------
    // call callback function to inform user/stack that an object will be searched
    // if the called module returnes an error then we abort the searching with kEplObdIndexNotExist
    CbParam.index    = index_p;
    CbParam.subIndex = uiSubindex_p;
    CbParam.pArg       = NULL;
    CbParam.obdEvent   = kObdEvCheckExist;
    Ret = callObjectCallback (
        pObdEntry->pfnCallback, &CbParam);
    if (Ret != kEplSuccessful)
    {
        Ret = kEplObdIndexNotExist;
        goto Exit;
    }

    //------------------------------------------------------------------------
    // it is allowed to set ppObdEntry_p to NULL
    // if so, no address will be written to calling function
    if (ppObdEntry_p != NULL)
    {
        *ppObdEntry_p = pObdEntry;
    }

Exit:

    return Ret;

}
//---------------------------------------------------------------------------
//
// Function:    getObjectCurrentPtr()
//
// Description: function to get Current pointer (type specific)
//
// Parameters:  pSubIndexEntry_p
//
// Return:      void MEM*
//
// State:
//
//---------------------------------------------------------------------------

static void MEM* getObjectCurrentPtr(tObdSubEntryPtr pSubIndexEntry_p)
{

void MEM*       pData;
unsigned int    uiArrayIndex;
tObdSize        Size;

    pData = pSubIndexEntry_p->pCurrent;

    // check if constant object
    if (pData != NULL)
    {
        // check if object is an array
        if ((pSubIndexEntry_p->access & kObdAccArray) != 0)
        {
            // calculate correct data pointer
            uiArrayIndex = pSubIndexEntry_p->subIndex - 1;
            if ((pSubIndexEntry_p->access & kObdAccVar) != 0)
            {
                Size = sizeof (tObdVarEntry);
            }
            else
            {
                Size = getObjectSize (pSubIndexEntry_p);
            }
            pData = ((BYTE MEM*) pData) + (Size * uiArrayIndex);
        }

        // check if VarEntry
        if ((pSubIndexEntry_p->access & kObdAccVar) != 0)
        {
            // The data pointer is stored in VarEntry->pData
            pData = ((tObdVarEntry MEM*) pData)->pData;
        }

        // the default pointer is stored for strings in tObdVString
        else if ((pSubIndexEntry_p->type == kObdTypeVString) /* ||
            (pSubIndexEntry_p->m_Type == kObdTypeUString)    */ )
        {
            pData = (void MEM*) ((tObdVString MEM*) pData)->pString;
        }
        else if (pSubIndexEntry_p->type == kObdTypeOString)
        {
            pData = (void MEM*) ((tObdOString MEM*) pData)->pString;
        }
    }

    return pData;

}


//---------------------------------------------------------------------------
//
// Function:    getIndexIntern()
//
// Description: gets a index entry from OD
//
// Parameters:  pInitParam_p
//              index_p
//              ppObdEntry_p
//
// Return:      tEplKernel
//
// State:
//
//---------------------------------------------------------------------------

static tEplKernel getIndexIntern (tObdInitParam MEM* pInitParam_p, UINT index_p, tObdEntryPtr* ppObdEntry_p)
{

tObdEntryPtr    pObdEntry;
tEplKernel      Ret;
unsigned int    uiIndex;

#if (defined (EPL_OBD_USER_OD) && (EPL_OBD_USER_OD != FALSE))

unsigned int  nLoop;

    // if user OD is used then objekts also has to be searched in user OD
    // there is less code need if we do this in a loop
    nLoop = 2;

#endif

    ASSERTMSG (ppObdEntry_p != NULL, "getIndexIntern(): pointer to index entry is NULL!\n");

    Ret = kEplObdIndexNotExist;

    // get start address of OD part
    // start address depends on object index because
    // object dictionary is divided in 3 parts
    if ((index_p >= 0x1000) && (index_p < 0x2000))
    {
        pObdEntry = pInitParam_p->pGenericPart;
    }
    else if ((index_p >= 0x2000) && (index_p < 0x6000))
    {
        pObdEntry = pInitParam_p->pManufacturerPart;
    }

    // index range 0xA000 to 0xFFFF is reserved for DSP-405
    // DS-301 defines that range 0x6000 to 0x9FFF (!!!) is stored if "store" was written to 0x1010/3.
    // Therefore default configuration is OBD_INCLUDE_A000_TO_DEVICE_PART = FALSE.
    // But a CANopen Application which does not implement dynamic OD or user-OD but wants to use static objets 0xA000...
    // should set OBD_INCLUDE_A000_TO_DEVICE_PART to TRUE.

#if (EPL_OBD_INCLUDE_A000_TO_DEVICE_PART == FALSE)
    else if ((index_p >= 0x6000) && (index_p < 0x9FFF))
#else
    else if ((index_p >= 0x6000) && (index_p < 0xFFFF))
#endif
    {
        pObdEntry = pInitParam_p->pDevicePart;
    }


#if (defined (EPL_OBD_USER_OD) && (EPL_OBD_USER_OD != FALSE))

    // if index does not match in static OD then index only has to be searched in user OD
    else
    {
        // begin from first entry of user OD part
        pObdEntry = pInitParam_p->pUserPart;

        // no user OD is available
        if (pObdEntry == NULL)
        {
            goto Exit;
        }

        // loop must only run once
        nLoop = 1;
    }

    do
    {

#else

        // no user OD is available
        // so other object can be found in OD
        else
        {
            Ret = kEplObdIllegalPart;
            goto Exit;
        }

#endif

        // note:
        // The end of Index table is marked with m_uiIndex = 0xFFFF.
        // If this function will be called with wIndex_p = 0xFFFF, entry
        // should not be found. Therefor it is important to use
        // while{} instead of do{}while !!!

        // get first index of index table
        uiIndex = pObdEntry->index;

        // search Index in OD part
        while (uiIndex != EPL_OBD_TABLE_INDEX_END)
        {
            // go to the end of this function if index is found
            if (index_p == uiIndex)
            {
                // write address of OD entry to calling function
                *ppObdEntry_p = pObdEntry;
                Ret = kEplSuccessful;
                goto Exit;
            }

            // objects are sorted in OD
            // if the current index in OD is greater than the index which is to search then break loop
            // in this case user OD has to be search too
            if (index_p < uiIndex)
            {
                break;
            }

            // next entry in index table
            pObdEntry++;

            // get next index of index table
            uiIndex = pObdEntry->index;
        }

#if (defined (EPL_OBD_USER_OD) && (EPL_OBD_USER_OD != FALSE))

        // begin from first entry of user OD part
        pObdEntry = pInitParam_p->pUserPart;

        // no user OD is available
        if (pObdEntry == NULL)
        {
            goto Exit;
        }

        // switch next loop for user OD
        nLoop--;

    } while (nLoop > 0);

#endif

    // in this line Index was not found

Exit:

    return Ret;

}


//---------------------------------------------------------------------------
//
// Function:    getSubindexIntern()
//
// Description: gets a subindex entry from a index entry
//
// Parameters:  pObdEntry_p
//              bSubIndex_p
//              ppObdSubEntry_p
//
// Return:      tEplKernel
//
// State:
//
//---------------------------------------------------------------------------

static tEplKernel getSubindexIntern (tObdEntryPtr pObdEntry_p, UINT subIndex_p, tObdSubEntryPtr* ppObdSubEntry_p)
{

tObdSubEntryPtr    pSubEntry;
unsigned int       nSubIndexCount;
tEplKernel         Ret;

    ASSERTMSG (pObdEntry_p != NULL, "getSubindexIntern(): pointer to index is NULL!\n");
    ASSERTMSG (ppObdSubEntry_p != NULL, "getSubindexIntern(): pointer to subindex is NULL!\n");

    Ret = kEplObdSubindexNotExist;

    // get start address of subindex table and count of subindices
    pSubEntry     = pObdEntry_p->pSubIndex;
    nSubIndexCount =  pObdEntry_p->count;
    ASSERTMSG ((pSubEntry != NULL) && (nSubIndexCount > 0),
        "ObdGetSubindexIntern(): invalid subindex table within index table!\n");   // should never be NULL

    // search subindex in subindex table
    while (nSubIndexCount > 0)
    {
        // check if array is found
        if ((pSubEntry->access & kObdAccArray) != 0)
        {
            // check if subindex is in range
            if (subIndex_p < pObdEntry_p->count)
            {
                // update subindex number (subindex entry of an array is always in RAM !!!)
                pSubEntry->subIndex = subIndex_p;
                *ppObdSubEntry_p = pSubEntry;
                Ret = kEplSuccessful;
                goto Exit;
            }
        }

        // go to the end of this function if subindex is found
        else if (subIndex_p == pSubEntry->subIndex)
        {
            *ppObdSubEntry_p = pSubEntry;
            Ret = kEplSuccessful;
            goto Exit;
        }

        // objects are sorted in OD
        // if the current subindex in OD is greater than the subindex which is to search then break loop
        // in this case user OD has to be search too
        if (subIndex_p < pSubEntry->subIndex)
        {
            break;
        }

        pSubEntry++;
        nSubIndexCount--;
    }

    // in this line SubIndex was not fount

Exit:

    return Ret;

}


//---------------------------------------------------------------------------
//
// Function:    obd_storeLoadObjCallback()
//
// Description: function set address to callbackfunction for command Store and Load
//
// Parameters:  fpCallback_p
//
// Return:      tEplKernel
//
// State:
//
//---------------------------------------------------------------------------
#if (EPL_OBD_USE_STORE_RESTORE != FALSE)
tEplKernel obd_storeLoadObjCallback (tEplObdStoreLoadObjCallback fpCallback_p)
{

    // set new address of callback function
    m_fpStoreLoadObjCallback = fpCallback_p;

    return kEplSuccessful;

}
#endif // (EPL_OBD_USE_STORE_RESTORE != FALSE)


//---------------------------------------------------------------------------
//
// Function:    accessOdPartIntern()
//
// Description: runs through OD and executes a job
//
// Parameters:  currentOdPart_p
//              pObdEnty_p
//              direction_p     = what is to do (load values from flash or EEPROM, store, ...)
//
// Return:      tEplKernel
//
// State:
//
//---------------------------------------------------------------------------

static tEplKernel accessOdPartIntern (tObdPart currentOdPart_p, tObdEntryPtr pObdEnty_p,
                                      tObdDir direction_p)
{

tObdSubEntryPtr             pSubIndex;
unsigned int                nSubIndexCount;
tObdAccess                  Access;
void MEM*                   pDstData;
CONST void*                 pDefault;
tObdSize                    ObjSize;
tEplKernel                  Ret;
tObdVarEntry MEM*           pVarEntry = NULL;

#if (EPL_OBD_USE_STORE_RESTORE != FALSE)
tObdCbStoreParam MEM        CbStore;
#else
UNUSED_PARAMETER(currentOdPart_p);
#endif

    ASSERT (pObdEnty_p != NULL);

    Ret = kEplSuccessful;
#if (EPL_OBD_USE_STORE_RESTORE != FALSE)
    // prepare structure for STORE RESTORE callback function
    CbStore.currentOdPart = (BYTE) currentOdPart_p;
    CbStore.pData          = NULL;
    CbStore.objSize        = 0;
#endif

    // command of first action depends on direction to access
    #if (EPL_OBD_USE_STORE_RESTORE != FALSE)
    if (direction_p == kObdDirLoad)
    {
        CbStore.command = (BYTE) kObdCmdOpenRead;

        // call callback function for previous command
        Ret = callStoreCallback (
            &CbStore);
        if (Ret != kEplSuccessful)
        {
            goto Exit;
        }

        // set command for index and subindex loop
        CbStore.command = (BYTE) kObdCmdReadObj;
    }
    else if (direction_p == kObdDirStore)
    {
        CbStore.command = (BYTE) kObdCmdOpenWrite;

        // call callback function for previous command
        Ret = callStoreCallback (&CbStore);
        if (Ret != kEplSuccessful)
        {
            goto Exit;
        }

        // set command for index and subindex loop
        CbStore.command = (BYTE) kObdCmdWriteObj;
    }
    #endif // (EPL_OBD_USE_STORE_RESTORE != FALSE)

    // we should not restore the OD values here
    // the next NMT command "Reset Node" or "Reset Communication" resets the OD data
    if (direction_p != kObdDirRestore)
    {
        // walk through OD part till end is found
        while (pObdEnty_p->index != EPL_OBD_TABLE_INDEX_END)
        {
            // get address to subindex table and count of subindices
            pSubIndex     = pObdEnty_p->pSubIndex;
            nSubIndexCount = pObdEnty_p->count;
            ASSERT ((pSubIndex != NULL) && (nSubIndexCount > 0));    // should never be NULL

            // walk through subindex table till all subinices were restored
            while (nSubIndexCount != 0)
            {
                Access = (tObdAccess) pSubIndex->access;

                // get pointer to current and default data
                pDefault = getObjectDefaultPtr (pSubIndex);
                pDstData = getObjectCurrentPtr (pSubIndex);

                // NOTE (for kObdTypeVString):
                //      The function returnes the max. number of bytes for a
                //      current string.
                //      r.d.: For stings the default-size will be read in other lines following (kObdDirInit).
                ObjSize  = getObjectSize (pSubIndex);

                // switch direction of OD access
                switch (direction_p)
                {
                    // --------------------------------------------------------------------------
                    // VarEntry structures has to be initialized
                    case kObdDirInit:

                        // If VAR-Flag is set, m_pCurrent means not address of data
                        // but address of tObdVarEntry. Address of data has to be get from
                        // this structure.
                        if ((Access & kObdAccVar) != 0)
                        {
                            getVarEntry (pSubIndex, &pVarEntry);
                            obd_initVarEntry (pVarEntry, pSubIndex->type, ObjSize);
/*
                            if ((Access & kObdAccArray) == 0)
                            {
                                obd_initVarEntry (pSubIndex->m_pCurrent, pSubIndex->m_Type, ObjSize);
                            }
                            else
                            {
                                obd_initVarEntry ((tObdVarEntry MEM*) (((BYTE MEM*) pSubIndex->m_pCurrent) + (sizeof (tObdVarEntry) * pSubIndex->m_uiSubIndex)),
                                    pSubIndex->m_Type, ObjSize);
                            }
*/
                            // at this time no application variable is defined !!!
                            // therefore data can not be copied.
                            break;
                        }
                        else if (pSubIndex->type == kObdTypeVString)
                        {
                            // If pointer m_pCurrent is not equal to NULL then the
                            // string was defined with EPL_OBD_SUBINDEX_RAM_VSTRING. The current
                            // pointer points to struct tObdVString located in MEM.
                            // The element size includes the max. number of
                            // bytes. The element m_pString includes the pointer
                            // to string in MEM. The memory location of default string
                            // must be copied to memory location of current string.

                            pDstData = pSubIndex->pCurrent;
                            if (pDstData != NULL)
                            {
                                // 08-dec-2004: code optimization !!!
                                //              entries ((tObdVStringDef ROM*) pSubIndex->m_pDefault)->m_pString
                                //              and ((tObdVStringDef ROM*) pSubIndex->m_pDefault)->m_Size were read
                                //              twice. thats not necessary!

                                // For copying data we have to set the destination pointer to the real RAM string. This
                                // pointer to RAM string is located in default string info structure. (translated r.d.)
                                pDstData = (void MEM*) ((tObdVStringDef ROM*) pSubIndex->pDefault)->pString;
                                ObjSize  = ((tObdVStringDef ROM*) pSubIndex->pDefault)->size;


                                ((tObdVString MEM*) pSubIndex->pCurrent)->pString = pDstData;
                                ((tObdVString MEM*) pSubIndex->pCurrent)->size    = ObjSize;
                            }

                        }
                        else if(pSubIndex->type == kObdTypeOString)
                        {
                            pDstData = pSubIndex->pCurrent;
                            if (pDstData != NULL)
                            {
                                // 08-dec-2004: code optimization !!!
                                //              entries ((tObdOStringDef ROM*) pSubIndex->m_pDefault)->m_pString
                                //              and ((tObdOStringDef ROM*) pSubIndex->m_pDefault)->m_Size were read
                                //              twice. thats not necessary!

                                // For copying data we have to set the destination pointer to the real RAM string. This
                                // pointer to RAM string is located in default string info structure. (translated r.d.)
                                pDstData = (void MEM*) ((tObdOStringDef ROM*) pSubIndex->pDefault)->pString;
                                ObjSize  = ((tObdOStringDef ROM*) pSubIndex->pDefault)->size;


                                ((tObdOString MEM*) pSubIndex->pCurrent)->pString = pDstData;
                                ((tObdOString MEM*) pSubIndex->pCurrent)->size    = ObjSize;
                            }

                        }


                        // no break !! because copy of data has to done too.

                    // --------------------------------------------------------------------------
                    // all objects has to be restored with default values
                    case kObdDirRestore:

                        // 09-dec-2004 r.d.: optimization! the same code for kObdDirRestore and kObdDirLoad
                        //                   is replaced to function ObdCopyObjectData() with a new parameter.
                        // restore object data for init phase

                        copyObjectData (pDstData, pDefault, ObjSize, pSubIndex->type);

                        // execute post default event
                        callPostDefault (pDstData, pObdEnty_p, pSubIndex);

                        break;

                    // --------------------------------------------------------------------------
                    // objects with attribute kObdAccStore has to be load from EEPROM or from a file
                    case kObdDirLoad:

                        // restore object data for init phase
                        copyObjectData (pDstData, pDefault, ObjSize, pSubIndex->type);

                        // execute post default event
                        callPostDefault (pDstData, pObdEnty_p, pSubIndex);

                        // no break !! because callback function has to be called too.

                    // --------------------------------------------------------------------------
                    // objects with attribute kObdAccStore has to be stored in EEPROM or in a file
                    case kObdDirStore:

                        // when attribute kObdAccStore is set, then call callback function
                        #if (EPL_OBD_USE_STORE_RESTORE != FALSE)
                        if ((Access & kObdAccStore) != 0)
                        {
                            // fill out data pointer and size of data
                            CbStore.pData    = pDstData;
                            CbStore.objSize  = ObjSize;

                            // call callback function for read or write object
                            Ret = ObdCallStoreCallback (&CbStore);
                            if (Ret != kEplSuccessful)
                            {
                                goto Exit;
                            }
                        }
                        #endif // (EPL_OBD_USE_STORE_RESTORE != FALSE)
                        break;


                    // --------------------------------------------------------------------------
                    // if OD Builder key has to be checked no access to subindex and data should be made
                    case kObdDirOBKCheck:

                        // no break !! because we want to break the second loop too.


                    // --------------------------------------------------------------------------
                    // unknown Direction
                    default:

                        // so we can break the second loop earler
                        nSubIndexCount = 1;
                        break;
                }

                nSubIndexCount--;

                // next subindex entry
                if ((Access & kObdAccArray) == 0)
                {
                    pSubIndex++;
                    if ((nSubIndexCount > 0)
                        && ((pSubIndex->access & kObdAccArray) != 0))
                    {
                        // next subindex points to an array
                        // reset subindex number
                        pSubIndex->subIndex = 1;
                    }
                }
                else
                {
                    if (nSubIndexCount > 0)
                    {
                        // next subindex points to an array
                        // increment subindex number
                        pSubIndex->subIndex++;
                    }
                }
            }

            // next index entry
            pObdEnty_p++;
        }
    }

    // -----------------------------------------------------------------------------------------
    // command of last action depends on direction to access
    if (direction_p == kObdDirOBKCheck)
    {

        goto Exit;
    }
    #if (EPL_OBD_USE_STORE_RESTORE != FALSE)
    else
    {
        if (direction_p == kObdDirLoad)
        {
            CbStore.command = (BYTE) kObdCmdCloseRead;
        }
        else if (direction_p == kObdDirStore)
        {
            CbStore.command = (BYTE) kObdCmdCloseWrite;
        }
        else if (direction_p == kObdDirRestore)
        {
            CbStore.command = (BYTE) kObdCmdClear;
        }
        else
        {
            goto Exit;
        }

        // call callback function for last command
        Ret = callStoreCallback (&CbStore);
    }
    #endif // (EPL_OBD_USE_STORE_RESTORE != FALSE)

//    goto Exit;

Exit:

    return Ret;

}


// ----------------------------------------------------------------------------
// Function:    copyObjectData()
//
// Description: checks pointers to object data and copy them from source to destination
//
// Parameters:  pDstData_p              = destination pointer
//              pSrcData_p              = source pointer
//              objSize_p               = size of object
//              objType_p               =
//
// Returns:     tEplKernel              = error code
// ----------------------------------------------------------------------------

static void copyObjectData (void MEM* pDstData_p, CONST void* pSrcData_p,
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
            StrSize = getObdStringLen ((void *) pSrcData_p, objSize_p, kObdTypeVString);

            // If the string length is greater than or equal to the entry size in OD then only copy
            // entry size - 1 and always set the '\0'-termination.
            if (StrSize >= objSize_p)
            {
                StrSize = objSize_p - 1;
            }
        }

        if (pSrcData_p != NULL)
        {
            // copy data
            EPL_MEMCPY (pDstData_p, pSrcData_p, objSize_p);

            if (objType_p == kObdTypeVString)
            {
                ((char MEM*) pDstData_p)[StrSize] = '\0';
            }
        }
    }

}

// ----------------------------------------------------------------------------
// Function:    callPostDefault()
//
// Description: calls the callback function with post callback event
//
// Parameters:  pDstData_p              = data pointer
//              pObdEntry_p             = pointer to obd entry
//              pSubIndex_p             = pointer to obd subentry
//
// Returns:     tEplKernel              = error code
// ----------------------------------------------------------------------------
static tEplKernel callPostDefault (void *pData_p, tObdEntryPtr pObdEntry_p,
                                   tObdSubEntryPtr pSubIndex_p)
{
    tEplKernel          ret;
    tObdCbParam         cbParam;

    cbParam.index    = pObdEntry_p->index;
    cbParam.subIndex = pSubIndex_p->subIndex;
    cbParam.pArg     = pData_p;
    cbParam.obdEvent = kObdEvPostDefault;
    ret = callObjectCallback ( pObdEntry_p->pfnCallback,
                                    &cbParam);

    return ret;
}

//---------------------------------------------------------------------------
//
// Function:    isNumericalIntern()
//
// Description: function checks if a entry is numerical or not
//
//
// Parameters:
//              index_p           = Index
//              subIndex_p        = Subindex
//              pfEntryNumerical_p  = pointer to BOOL for returnvalue
//                                  -> TRUE if entry a numerical value
//                                  -> FALSE if entry not a numerical value
//
// Return:      tEplKernel = Errorcode
//
// State:
//
//---------------------------------------------------------------------------
static tEplKernel isNumericalIntern(tObdSubEntryPtr pObdSubEntry_p,
                                    BOOL* pfEntryNumerical_p)
{
tEplKernel          Ret = kEplSuccessful;


    // get Type
    if((pObdSubEntry_p->type == kObdTypeVString)
        || (pObdSubEntry_p->type == kObdTypeOString)
        || (pObdSubEntry_p->type == kObdTypeDomain))
    {   // not numerical types
        *pfEntryNumerical_p = FALSE;
    }
    else
    {   // numerical types
        *pfEntryNumerical_p = TRUE;
    }

    return Ret;

}


// -------------------------------------------------------------------------
// function to classify object type (fixed/non fixed)
// -------------------------------------------------------------------------

// ----------------------------------------------------------------------------
// Function:    callStoreCallback()
//
// Description: checks address to callback function and calles it when unequal
//              to NULL
//
// Parameters:
//              pCbStoreParam_p        = address to callback parameters
//
// Returns:     tEplKernel             = error code
// ----------------------------------------------------------------------------
#if (EPL_OBD_USE_STORE_RESTORE != FALSE)
static tEplKernel callStoreCallback(tObdCbStoreParam MEM* pCbStoreParam_p)
{

tEplKernel Ret = kEplSuccessful;

    ASSERT (pCbStoreParam_p != NULL);

    // check if function pointer is NULL - if so, no callback should be called
    if (m_fpStoreLoadObjCallback != NULL)
    {
        Ret = m_fpStoreLoadObjCallback(pCbStoreParam_p);
    }

    return Ret;

}
#endif // (EPL_OBD_USE_STORE_RESTORE != FALSE)
//---------------------------------------------------------------------------
//
// Function:    getObjectDataPtrIntern()
//
// Description: Function gets the data pointer of an object.
//              It returnes the current data pointer. But if object is an
//              constant object it returnes the default pointer.
//
// Parameters:  pSubindexEntry_p = pointer to subindex entry
//
// Return:      void *    = pointer to object data
//
// State:
//
//---------------------------------------------------------------------------

static void* getObjectDataPtrIntern(tObdSubEntryPtr pSubindexEntry_p)
{

void * pData;
tObdAccess Access;

    ASSERTMSG (pSubindexEntry_p != NULL, "getObjectDataPtrIntern(): pointer to SubEntry not valid!\n");

    // there are are some objects whose data pointer has to get from other structure
    // get access type for this object
    Access = pSubindexEntry_p->access;

    // If object has access type = const,
    // only the default value exists.
    if ((Access & kObdAccConst) != 0)
    {
        // The pointer to default value can be received from ObdGetObjectDefaultPtr()
        pData = ((void *) getObjectDefaultPtr (pSubindexEntry_p));
    }
    else
    {
        // The pointer to current value can be received from ObdGetObjectCurrentPtr()
        pData = getObjectCurrentPtr (pSubindexEntry_p);
    }

    return pData;

}

// EOF

