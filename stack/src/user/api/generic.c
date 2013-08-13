/****************************************************************************

  (c) SYSTEC electronic GmbH, D-07973 Greiz, August-Bebel-Str. 29
      www.systec-electronic.com

  Project:      openPOWERLINK

  Description:  source file for generic EPL API module

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
                    GCC V3.4

  -------------------------------------------------------------------------

  Revision History:

  2006/09/05 d.k.:   start of the implementation, version 1.00

****************************************************************************/

#include "Epl.h"
#include "kernel/dllk.h"
#include "kernel/eventk.h"
#include "kernel/nmtk.h"
#include "kernel/EplObdk.h"
#include "kernel/dllkcal.h"
#include "kernel/pdokcal.h"
#include "user/pdoucal.h"
#include "user/pdou.h"
#include "user/dllucal.h"
#include "user/ledu.h"
#include "user/nmtcnu.h"
#include "user/nmtmnu.h"
#include "user/EplSdoComu.h"
#include "user/identu.h"
#include "user/statusu.h"
#include "user/cfmu.h"

#include <user/ctrlu.h>
#include <errhnd.h>
#include <kernel/errhndk.h>
#include <user/errhndu.h>

#include <stddef.h>
#include <limits.h>

#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_VETH)) != 0)
#include <kernel/veth.h>
#endif

#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_PDOK)) != 0)
#include "kernel/pdok.h"
#endif

#if(((EPL_MODULE_INTEGRATION) & (EPL_MODULE_OBDK)) == 0)
#error "EPL API layer needs EPL module OBDK!"
#endif

#if (EPL_OBD_USE_LOAD_CONCISEDCF != FALSE)
#include "EplObdCdc.h"
#endif

#if EPL_NMTMNU_PRES_CHAINING_MN != FALSE
#include "user/syncu.h"
#endif

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

//---------------------------------------------------------------------------
// module global vars
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
// local function prototypes
//---------------------------------------------------------------------------


/***************************************************************************/
/*                                                                         */
/*                                                                         */
/*          C L A S S  EplApi                                              */
/*                                                                         */
/*                                                                         */
/***************************************************************************/
//
// Description:
//
//
/***************************************************************************/


//=========================================================================//
//                                                                         //
//          P R I V A T E   D E F I N I T I O N S                          //
//                                                                         //
//=========================================================================//

//---------------------------------------------------------------------------
// const defines
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
// local types
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
// local vars
//---------------------------------------------------------------------------

static tEplApiInstance  EplApiInstance_g;


//---------------------------------------------------------------------------
// local function prototypes
//---------------------------------------------------------------------------

// EplNmtCnu check event callback function
static tEplKernel PUBLIC EplApiCbCnCheckEvent(tNmtEvent NmtEvent_p);

// NMT state change event callback function
static tEplKernel PUBLIC EplApiCbNmtStateChange(tEventNmtStateChange NmtStateChange_p);

// update DLL configuration from OD
static tEplKernel PUBLIC EplApiUpdateDllConfig(BOOL fUpdateIdentity_p);

// update SDO configuration from OD
static tEplKernel PUBLIC EplApiUpdateSdoConfig();

// update OD from init param
static tEplKernel PUBLIC EplApiUpdateObd(void);

// process events from user event queue
static tEplKernel PUBLIC EplApiProcessEvent(tEplEvent* pEplEvent_p);

#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_SDOC)) != 0)
// callback function of SDO module
static tEplKernel PUBLIC  EplApiCbSdoCon(tEplSdoComFinished* pSdoComFinished_p);
#endif

#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)
// callback functions of NmtMnu module
static tEplKernel PUBLIC  EplApiCbNodeEvent(unsigned int uiNodeId_p,
                                            tNmtNodeEvent NodeEvent_p,
                                            tNmtState NmtState_p,
                                            WORD wErrorCode_p,
                                            BOOL fMandatory_p);
#endif // (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)

static tEplKernel PUBLIC  EplApiCbBootEvent(tNmtBootEvent BootEvent_p,
                                            tNmtState NmtState_p,
                                            WORD wErrorCode_p);

#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_LEDU)) != 0)
// callback function of Ledu module
static tEplKernel PUBLIC  EplApiCbLedStateChange(tLedType LedType_p,
                                                 BOOL fOn_p);
#endif

#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_CFM)) != 0)
static tEplKernel PUBLIC  EplApiCbCfmEventCnProgress(tCfmEventCnProgress* pEventCnProgress_p);
static tEplKernel PUBLIC  EplApiCbCfmEventCnResult(unsigned int uiNodeId_p, tNmtNodeCommand NodeCommand_p);
#endif

static tEplKernel PUBLIC EplApiCbReceivedAsnd(tFrameInfo *pFrameInfo_p);

// OD initialization function (implemented in Objdict.c)
//tEplKernel PUBLIC  EplObdInitRam (tEplObdInitParam MEM* pInitParam_p);


//=========================================================================//
//                                                                         //
//          P U B L I C   F U N C T I O N S                                //
//                                                                         //
//=========================================================================//

//---------------------------------------------------------------------------
//
// Function:    EplApiInitialize()
//
// Description: add and initialize new instance of EPL stack.
//              After return from this function the application must start
//              the NMT state machine via
//              EplApiExecNmtCommand(kNmtEventSwReset)
//              and thereby the whole EPL stack :-)
//
// Parameters:  pInitParam_p            = initialisation parameters
//
// Returns:     tEplKernel              = error code
//
//
// State:
//
//---------------------------------------------------------------------------
tEplKernel PUBLIC EplApiInitialize(tEplApiInitParam * pInitParam_p)
{
    tEplKernel          ret;
    tEplApiCbFuncs      cbFuncs;

    cbFuncs.pfnCbCnCheckEvent = EplApiCbCnCheckEvent;
    cbFuncs.pfnCbNmtStateChange = EplApiCbNmtStateChange;
    cbFuncs.pfnCbBootEvent = EplApiCbBootEvent;
    cbFuncs.pfnCbProcessEvent = EplApiProcessEvent;
#if defined(CONFIG_INCLUDE_CFM)
    cbFuncs.pfnCbCfmProgress = EplApiCbCfmEventCnProgress;
    cbFuncs.pfnCbCfmResult = EplApiCbCfmEventCnResult;
#endif
#if defined(CONFIG_INCLUDE_NMT_MN)
    cbFuncs.pfnCbNodeEvent = EplApiCbNodeEvent;
#endif
#if defined(CONFIG_INCLUDE_LEDU)
    cbFuncs.pfnCbLedStateChange = EplApiCbLedStateChange;
#endif
    if ((ret = ctrlu_init()) != kEplSuccessful)
        return ret;

    return ctrlu_initStack(pInitParam_p, &EplApiInstance_g, &cbFuncs);
}

//---------------------------------------------------------------------------
//
// Function:    EplApiShutdown()
//
// Description: deletes an instance of EPL stack
//
// Parameters:  (none)
//
// Returns:     tEplKernel              = error code
//
//
// State:
//
//---------------------------------------------------------------------------
tEplKernel PUBLIC EplApiShutdown(void)
{
    tEplKernel          ret;

    ret = ctrlu_shutdownStack();
    ctrlu_exit();

    return ret;
}

//----------------------------------------------------------------------------
// Function:    EplApiExecNmtCommand()
//
// Description: executes a NMT command, i.e. post the NMT command/event to the
//              NMTk module. NMT commands which are not appropriate in the current
//              NMT state are silently ignored. Please keep in mind that the
//              NMT state may change until the NMT command is actually executed.
//
// Parameters:  NmtEvent_p              = NMT command/event
//
// Returns:     tEplKernel              = error code
//
// State:
//----------------------------------------------------------------------------

tEplKernel PUBLIC EplApiExecNmtCommand(tNmtEvent NmtEvent_p)
{
tEplKernel      Ret = kEplSuccessful;

#if(((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMTU)) != 0)
    Ret = nmtu_postNmtEvent(NmtEvent_p);
#endif

    return Ret;
}


//----------------------------------------------------------------------------
// Function:    EplApiLinkObject()
//
// Description: Function maps array of application variables onto specified object in OD
//
// Parameters:  uiObjIndex_p            = Function maps variables for this object index
//              pVar_p                  = Pointer to data memory area for the specified object
//              puiVarEntries_p         = IN: pointer to number of entries to map
//                                        OUT: pointer to number of actually used entries
//              pEntrySize_p            = IN: pointer to size of one entry;
//                                            if size is zero, the actual size will be read from OD
//                                        OUT: pointer to entire size of all entries mapped
//              uiFirstSubindex_p       = This is the first subindex to be mapped.
//
// Returns:     tEplKernel              = error code
//
// State:
//----------------------------------------------------------------------------

tEplKernel PUBLIC EplApiLinkObject( unsigned int    uiObjIndex_p,
                                    void*           pVar_p,
                                    unsigned int*   puiVarEntries_p,
                                    tEplObdSize*    pEntrySize_p,
                                    unsigned int    uiFirstSubindex_p)
{
BYTE            bVarEntries;
BYTE            bIndexEntries;
BYTE MEM*       pbData;
unsigned int    uiSubindex;
tEplVarParam    VarParam;
tEplObdSize     EntrySize;
tEplObdSize     UsedSize;

tEplKernel      RetCode = kEplSuccessful;

    if ((pVar_p == NULL)
        || (puiVarEntries_p == NULL)
        || (*puiVarEntries_p == 0)
        || (pEntrySize_p == NULL))
    {
        RetCode = kEplApiInvalidParam;
        goto Exit;
    }

    pbData      = (BYTE MEM*) pVar_p;
    bVarEntries = (BYTE) *puiVarEntries_p;
    UsedSize    = 0;

    // init VarParam structure with default values
    VarParam.m_uiIndex    = uiObjIndex_p;
    VarParam.m_ValidFlag  = kVarValidAll;

    if (uiFirstSubindex_p != 0)
    {   // check if object exists by reading subindex 0x00,
        // because user wants to link a variable to a subindex unequal 0x00
        // read number of entries
        EntrySize = (tEplObdSize)  sizeof(bIndexEntries);
        RetCode = EplObdReadEntry (
                                uiObjIndex_p,
                                0x00,
                                (void GENERIC*) &bIndexEntries,
                                &EntrySize );

        if ((RetCode != kEplSuccessful) || (bIndexEntries == 0x00) )
        {
            // Object doesn't exist or invalid entry number
            TRACE("%s() Object %04x not existing\n", __func__, uiObjIndex_p);
            RetCode = kEplObdIndexNotExist;
            goto Exit;
        }
    }
    else
    {   // user wants to link a variable to subindex 0x00
        // that's OK
        bIndexEntries = 0;
    }

    // Correct number of entries if number read from OD is greater
    // than the specified number.
    // This is done, so that we do not set more entries than subindexes the
    // object actually has.
    if ((bIndexEntries > (bVarEntries + uiFirstSubindex_p - 1)) &&
        (bVarEntries   != 0x00) )
    {
        bIndexEntries = (BYTE) (bVarEntries + uiFirstSubindex_p - 1);
    }

    // map entries
    for (uiSubindex = uiFirstSubindex_p; uiSubindex <= bIndexEntries; uiSubindex++)
    {
        // if passed entry size is 0, then get size from OD
        if (*pEntrySize_p == 0x00)
        {
            // read entry size
            EntrySize = EplObdGetDataSize(uiObjIndex_p, uiSubindex);

            if (EntrySize == 0x00)
            {
                // invalid entry size (maybe object doesn't exist or entry of type DOMAIN is empty)
                RetCode = kEplObdSubindexNotExist;
                break;
            }
        }
        else
        {   // use passed entry size
            EntrySize = *pEntrySize_p;
        }

        VarParam.m_uiSubindex = uiSubindex;

        // set pointer to user var
        VarParam.m_Size  = EntrySize;
        VarParam.m_pData = pbData;

        UsedSize += EntrySize;
        pbData   += EntrySize;

        RetCode = EplObdDefineVar(&VarParam);
        if (RetCode != kEplSuccessful)
        {
            break;
        }
    }

    // set number of mapped entries and entry size
    *puiVarEntries_p = ((bIndexEntries - uiFirstSubindex_p) + 1);
    *pEntrySize_p = UsedSize;


Exit:

    return (RetCode);

}


// ----------------------------------------------------------------------------
//
// Function:    EplApiReadObject()
//
// Description: reads the specified entry from the OD of the specified node.
//              If this node is a remote node, it performs a SDO transfer, which
//              means this function returns kEplApiTaskDeferred and the application
//              is informed via the event callback function when the task is completed.
//
// Parameters:  pSdoComConHdl_p         = INOUT: pointer to SDO connection handle (may be NULL in case of local OD access)
//              uiNodeId_p              = IN: node ID (0 = itself)
//              uiIndex_p               = IN: index of object in OD
//              uiSubindex_p            = IN: sub-index of object in OD
//              pDstData_le_p           = OUT: pointer to data in little endian
//              puiSize_p               = INOUT: pointer to size of data
//              SdoType_p               = IN: type of SDO transfer
//              pUserArg_p              = IN: user-definable argument pointer,
//                                            which will be passed to the event callback function
//
// Return:      tEplKernel              = error code
//
// ----------------------------------------------------------------------------

tEplKernel PUBLIC EplApiReadObject(
            tEplSdoComConHdl* pSdoComConHdl_p,
            unsigned int      uiNodeId_p,
            unsigned int      uiIndex_p,
            unsigned int      uiSubindex_p,
            void*             pDstData_le_p,
            unsigned int*     puiSize_p,
            tEplSdoType       SdoType_p,
            void*             pUserArg_p)
{
tEplKernel      Ret = kEplSuccessful;

    if ((uiIndex_p == 0) || (pDstData_le_p == NULL) || (puiSize_p == NULL) || (*puiSize_p == 0))
    {
        Ret = kEplApiInvalidParam;
        goto Exit;
    }

    if (uiNodeId_p == 0
        || uiNodeId_p == EplObdGetNodeId())
    {   // local OD access can be performed
    tEplObdSize     ObdSize;

        ObdSize = (tEplObdSize) *puiSize_p;
        Ret = EplObdReadEntryToLe(uiIndex_p, uiSubindex_p, pDstData_le_p, &ObdSize);
        *puiSize_p = (unsigned int) ObdSize;
    }
    else
    {   // perform SDO transfer
#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_SDOC)) != 0)
    tEplSdoComTransParamByIndex TransParamByIndex;

        // check if application provides space for handle
        if (pSdoComConHdl_p == NULL)
        {
            Ret = kEplApiInvalidParam;
            goto Exit;
        }

#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_CFM)) != 0)
        if (cfmu_isSdoRunning(uiNodeId_p))
        {
            Ret = kEplApiSdoBusyIntern;
            goto Exit;
        }
#endif

        // init command layer connection
        Ret = EplSdoComDefineCon(pSdoComConHdl_p,
                                    uiNodeId_p,  // target node id
                                    SdoType_p);    // SDO type
        if ((Ret != kEplSuccessful) && (Ret != kEplSdoComHandleExists))
        {
            goto Exit;
        }
        TransParamByIndex.m_pData = pDstData_le_p;
        TransParamByIndex.m_SdoAccessType = kEplSdoAccessTypeRead;
        TransParamByIndex.m_SdoComConHdl = *pSdoComConHdl_p;
        TransParamByIndex.m_uiDataSize = *puiSize_p;
        TransParamByIndex.m_uiIndex = uiIndex_p;
        TransParamByIndex.m_uiSubindex = uiSubindex_p;
        TransParamByIndex.m_pfnSdoFinishedCb = EplApiCbSdoCon;
        TransParamByIndex.m_pUserArg = pUserArg_p;

        Ret = EplSdoComInitTransferByIndex(&TransParamByIndex);
        if (Ret != kEplSuccessful)
        {
            goto Exit;
        }
        Ret = kEplApiTaskDeferred;

#else
        Ret = kEplApiInvalidParam;
#endif
    }

Exit:
    return Ret;
}


// ----------------------------------------------------------------------------
//
// Function:    EplApiWriteObject()
//
// Description: writes the specified entry to the OD of the specified node.
//              If this node is a remote node, it performs a SDO transfer, which
//              means this function returns kEplApiTaskDeferred and the application
//              is informed via the event callback function when the task is completed.
//
// Parameters:  pSdoComConHdl_p         = INOUT: pointer to SDO connection handle (may be NULL in case of local OD access)
//              uiNodeId_p              = IN: node ID (0 = itself)
//              uiIndex_p               = IN: index of object in OD
//              uiSubindex_p            = IN: sub-index of object in OD
//              pSrcData_le_p           = IN: pointer to data in little endian
//              uiSize_p                = IN: size of data in bytes
//              SdoType_p               = IN: type of SDO transfer
//              pUserArg_p              = IN: user-definable argument pointer,
//                                            which will be passed to the event callback function
//
// Return:      tEplKernel              = error code
//
// ----------------------------------------------------------------------------

tEplKernel PUBLIC EplApiWriteObject(
            tEplSdoComConHdl* pSdoComConHdl_p,
            unsigned int      uiNodeId_p,
            unsigned int      uiIndex_p,
            unsigned int      uiSubindex_p,
            void*             pSrcData_le_p,
            unsigned int      uiSize_p,
            tEplSdoType       SdoType_p,
            void*             pUserArg_p)
{
tEplKernel      Ret = kEplSuccessful;

    if ((uiIndex_p == 0) || (pSrcData_le_p == NULL) || (uiSize_p == 0))
    {
        Ret = kEplApiInvalidParam;
        goto Exit;
    }

    if (uiNodeId_p == 0
        || uiNodeId_p == EplObdGetNodeId())
    {   // local OD access can be performed

        Ret = EplObdWriteEntryFromLe(uiIndex_p, uiSubindex_p, pSrcData_le_p, uiSize_p);
    }
    else
    {   // perform SDO transfer
#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_SDOC)) != 0)
    tEplSdoComTransParamByIndex TransParamByIndex;

        // check if application provides space for handle
        if (pSdoComConHdl_p == NULL)
        {
            Ret = kEplApiInvalidParam;
            goto Exit;
        }

#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_CFM)) != 0)
        if (cfmu_isSdoRunning(uiNodeId_p))
        {
            Ret = kEplApiSdoBusyIntern;
            goto Exit;
        }
#endif

        // d.k.: How to recycle command layer connection?
        //       Try to redefine it, which will return kEplSdoComHandleExists
        //       and the existing command layer handle.
        //       If the returned handle is busy, EplSdoComInitTransferByIndex()
        //       will return with error.

        // init command layer connection
        Ret = EplSdoComDefineCon(pSdoComConHdl_p,
                                    uiNodeId_p,  // target node id
                                    SdoType_p);    // SDO type
        if ((Ret != kEplSuccessful) && (Ret != kEplSdoComHandleExists))
        {
            goto Exit;
        }
        TransParamByIndex.m_pData = pSrcData_le_p;
        TransParamByIndex.m_SdoAccessType = kEplSdoAccessTypeWrite;
        TransParamByIndex.m_SdoComConHdl = *pSdoComConHdl_p;
        TransParamByIndex.m_uiDataSize = uiSize_p;
        TransParamByIndex.m_uiIndex = uiIndex_p;
        TransParamByIndex.m_uiSubindex = uiSubindex_p;
        TransParamByIndex.m_pfnSdoFinishedCb = EplApiCbSdoCon;
        TransParamByIndex.m_pUserArg = pUserArg_p;

        Ret = EplSdoComInitTransferByIndex(&TransParamByIndex);
        if (Ret != kEplSuccessful)
        {
            goto Exit;
        }
        Ret = kEplApiTaskDeferred;

#else
        Ret = kEplApiInvalidParam;
#endif
    }

Exit:
    return Ret;
}


// ----------------------------------------------------------------------------
//
// Function:    EplApiFreeSdoChannel()
//
// Description: frees the specified SDO channel.
//              This function must be called when the SDO channel to a remote node
//              is not needed anymore. This may be done in the event callback function
//              when the last SDO transfer to a remote node has completed.
//
// Parameters:  SdoComConHdl_p          = IN: SDO connection handle which is not valid
//                                        anymore after this call.
//
// Return:      tEplKernel              = error code
//
// ----------------------------------------------------------------------------

tEplKernel PUBLIC EplApiFreeSdoChannel(
            tEplSdoComConHdl SdoComConHdl_p)
{
tEplKernel      Ret = kEplSuccessful;

#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_SDOC)) != 0)

#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_CFM)) != 0)
    if (cfmu_isSdoRunning(EplSdoComGetNodeId(SdoComConHdl_p)))
    {
        Ret = kEplApiSdoBusyIntern;
    }
    else
#endif
    {
        // delete command layer connection
        Ret = EplSdoComUndefineCon(SdoComConHdl_p);
    }
#else
    Ret = kEplApiInvalidParam;
#endif

    return Ret;
}


// ----------------------------------------------------------------------------
//
// Function:    EplApiAbortSdo()
//
// Description: aborts the running SDO transfer on the specified SDO channel.
//
// Parameters:  SdoComConHdl_p          = IN: SDO connection handle
//              dwAbortCode_p           = IN: SDO abort code which shall be send
//                                        to the remote node.
//
// Return:      tEplKernel              = error code
//
// ----------------------------------------------------------------------------

tEplKernel PUBLIC EplApiAbortSdo(
            tEplSdoComConHdl SdoComConHdl_p,
            DWORD            dwAbortCode_p)
{
tEplKernel      Ret = kEplSuccessful;

#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_SDOC)) != 0)

#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_CFM)) != 0)
    if (cfmu_isSdoRunning(EplSdoComGetNodeId(SdoComConHdl_p)))
    {
        Ret = kEplApiSdoBusyIntern;
    }
    else
#endif
    {
        Ret = EplSdoComSdoAbort(SdoComConHdl_p, dwAbortCode_p);
    }
#else
    Ret = kEplApiInvalidParam;
#endif

    return Ret;
}


// ----------------------------------------------------------------------------
//
// Function:    EplApiReadLocalObject()
//
// Description: reads the specified entry from the local OD.
//
// Parameters:  uiIndex_p               = IN: index of object in OD
//              uiSubindex_p            = IN: sub-index of object in OD
//              pDstData_p              = OUT: pointer to data in platform byte order
//              puiSize_p               = INOUT: pointer to size of data
//
// Return:      tEplKernel              = error code
//
// ----------------------------------------------------------------------------

tEplKernel PUBLIC EplApiReadLocalObject(
            unsigned int      uiIndex_p,
            unsigned int      uiSubindex_p,
            void*             pDstData_p,
            unsigned int*     puiSize_p)
{
tEplKernel      Ret = kEplSuccessful;
tEplObdSize     ObdSize;

    ObdSize = (tEplObdSize) *puiSize_p;
    Ret = EplObdReadEntry(uiIndex_p, uiSubindex_p, pDstData_p, &ObdSize);
    *puiSize_p = (unsigned int) ObdSize;

    return Ret;
}


// ----------------------------------------------------------------------------
//
// Function:    EplApiWriteLocalObject()
//
// Description: writes the specified entry to the local OD.
//
// Parameters:  uiIndex_p               = IN: index of object in OD
//              uiSubindex_p            = IN: sub-index of object in OD
//              pSrcData_p              = IN: pointer to data in platform byte order
//              uiSize_p                = IN: size of data in bytes
//
// Return:      tEplKernel              = error code
//
// ----------------------------------------------------------------------------

tEplKernel PUBLIC EplApiWriteLocalObject(
            unsigned int      uiIndex_p,
            unsigned int      uiSubindex_p,
            void*             pSrcData_p,
            unsigned int      uiSize_p)
{
tEplKernel      Ret = kEplSuccessful;

    Ret = EplObdWriteEntry(uiIndex_p, uiSubindex_p, pSrcData_p, (tEplObdSize) uiSize_p);

    return Ret;
}

// ----------------------------------------------------------------------------
//
// Function:    EplApiSendAsndFrame()
//
// Description: Send a generic Asnd frame
//
// Parameters:  bDstNodeId_p            = Node ID of destination node
//              pAsndFrame_p            = Pointer to Asnd frame that should be sent
//              uiAsndSize_p            = Size of Asnd frame (service ID + payload)
//
// Return:      tEplKernel              = error code
//
// ----------------------------------------------------------------------------
tEplKernel PUBLIC EplApiSendAsndFrame
(
    BYTE            bDstNodeId_p,
    tEplAsndFrame   *pAsndFrame_p,
    size_t          uiAsndSize_p
)
{
    tEplKernel      Ret;
    tFrameInfo   FrameInfo;
    BYTE            Buffer[EPL_C_DLL_MAX_ASYNC_MTU];

    // Calculate size of frame (Asnd data + header)
    FrameInfo.frameSize = uiAsndSize_p + offsetof(tEplFrame, m_Data);

    // Check for correct input
    if
    (
        ( pAsndFrame_p              == NULL             )  ||
        ( FrameInfo.frameSize   >= sizeof(Buffer)   )
    )
    {
        return  kEplReject;
    }

    // Calculate size of frame (Asnd data + header)
    FrameInfo.frameSize = uiAsndSize_p + offsetof(tEplFrame, m_Data);
    FrameInfo.pFrame      = (tEplFrame *) Buffer;

    // Copy Asnd data
    EPL_MEMSET( FrameInfo.pFrame, 0x00, FrameInfo.frameSize );
    EPL_MEMCPY( &FrameInfo.pFrame->m_Data.m_Asnd, pAsndFrame_p, uiAsndSize_p );

    // Fill in additional data (SrcNodeId is filled by DLL if it is set to 0)
    AmiSetByteToLe( &FrameInfo.pFrame->m_le_bMessageType, (BYTE) kEplMsgTypeAsnd  );
    AmiSetByteToLe( &FrameInfo.pFrame->m_le_bDstNodeId,   (BYTE) bDstNodeId_p     );
    AmiSetByteToLe( &FrameInfo.pFrame->m_le_bSrcNodeId,   (BYTE) 0                );

    // Request frame transmission
    Ret = dllucal_sendAsyncFrame( &FrameInfo, kDllAsyncReqPrioGeneric);

    return Ret;
}

// ----------------------------------------------------------------------------
//
// Function:    EplApiSetAsndForward()
//
// Description: Enable or disable the forwarding of received Asnd frames
//              Asnd frames from the DLL to the application.
//
// Parameters:  bServiceId_p            = The Asnd service ID which should be configured
//              FilterType_p            = Specifies which types of Asnd frames should
//                                        be received (none, unicast, all)
//
// Return:      tEplKernel              = error code
//
// ----------------------------------------------------------------------------
tEplKernel PUBLIC EplApiSetAsndForward
(
    BYTE                bServiceId_p,
    tEplApiAsndFilter   FilterType_p
)
{
    tEplKernel          Ret;
    tDllAsndFilter      DllFilter;

    // Map API filter types to stack internal filter types
    switch( FilterType_p )
    {
        case kEplApiAsndFilterLocal:    DllFilter   = kDllAsndFilterLocal;
                                        break;

        case kEplApiAsndFilterAny:      DllFilter   = kDllAsndFilterAny;
                                        break;

        default:
        case kEplApiAsndFilterNone:     DllFilter   = kDllAsndFilterNone;
                                        break;
    }

    Ret = dllucal_regAsndService( bServiceId_p, EplApiCbReceivedAsnd, DllFilter);

    return Ret;
}

// ----------------------------------------------------------------------------
//
// Function:    EplApiPostUserEvent()
//
// Description: post user-defined event to event processing thread,
//              i.e. calls user event callback function with event kEplApiEventUserDef.
//              This function is thread safe and is meant for synchronization.
//
// Parameters:  pUserArg_p              = IN: user-defined pointer
//
// Return:      tEplKernel              = error code
//
// ----------------------------------------------------------------------------

tEplKernel PUBLIC EplApiPostUserEvent(void* pUserArg_p)
{
tEplKernel  Ret;
tEplEvent   Event;

    Event.m_EventSink = kEplEventSinkApi;
    Event.m_NetTime.m_dwNanoSec = 0;
    Event.m_NetTime.m_dwSec = 0;
    Event.m_EventType = kEplEventTypeApiUserDef;
    Event.m_pArg = &pUserArg_p;
    Event.m_uiSize = sizeof (pUserArg_p);

    Ret = eventu_postEvent(&Event);

    return Ret;
}

//---------------------------------------------------------------------------
//
// Function:    EplApiCbCnCheckEvent
//
// Description: posts boot event directly to API layer (without using
//              shared buffers)
//
// Parameters:  NmtEvent_p     = Nmt event
//
// Returns:     tEpKernel  = errorcode
//
// State:
//
//---------------------------------------------------------------------------
static tEplKernel PUBLIC EplApiCbCnCheckEvent(tNmtEvent NmtEvent_p)
{
tEplKernel              Ret = kEplSuccessful;
tNmtState               NmtState;

   switch (NmtEvent_p)
   {
        case kNmtEventEnableReadyToOperate:
        {
            NmtState = nmtu_getNmtState();

            // inform application
            Ret = EplApiCbBootEvent(kNmtBootEventEnableReadyToOp,
                                    NmtState,
                                    EPL_E_NO_ERROR);
            if (Ret != kEplSuccessful)
            {
                goto exit;
            }

            break;
        }

        default:
        break;
   }

exit:
   return Ret;
}


#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)
// ----------------------------------------------------------------------------
//
// Function:    EplApiMnTriggerStateChange()
//
// Description: triggers the specified node command for the specified node.
//
// Parameters:  uiNodeId_p              = node ID for which the node command will be executed
//              NodeCommand_p           = node command
//
// Return:      tEplKernel              = error code
//
// ----------------------------------------------------------------------------

tEplKernel PUBLIC EplApiMnTriggerStateChange(unsigned int uiNodeId_p,
                                             tNmtNodeCommand  NodeCommand_p)
{
tEplKernel      Ret = kEplSuccessful;

    Ret = nmtmnu_triggerStateChange(uiNodeId_p, NodeCommand_p);

    return Ret;
}

#endif // (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)


#if (EPL_OBD_USE_LOAD_CONCISEDCF != FALSE)
//---------------------------------------------------------------------------
//
// Function:    EplApiSetCdcBuffer
//
// Description: sets the buffer containing the ConciseDCF (CDC file)
//
// Parameters:  pbCdc_p                 = pointer to byte array containing the CDC
//              uiCdcSize_p             = size of the buffer
//
// Returns:     tEplKernel              = error code
//
//
// State:
//
//---------------------------------------------------------------------------

EPLDLLEXPORT tEplKernel PUBLIC EplApiSetCdcBuffer(BYTE* pbCdc_p, unsigned int uiCdcSize_p)
{
tEplKernel      Ret = kEplSuccessful;

    EplApiInstance_g.m_pbCdc = pbCdc_p;
    EplApiInstance_g.m_uiCdcSize = uiCdcSize_p;

    return Ret;
}


//---------------------------------------------------------------------------
//
// Function:    EplApiSetCdcFilename
//
// Description: sets the file name of the ConciseDCF (CDC file)
//
// Parameters:  pszCdcFilename_p        = pointer to string with the CDC file name
//
// Returns:     tEplKernel              = error code
//
//
// State:
//
//---------------------------------------------------------------------------

EPLDLLEXPORT tEplKernel PUBLIC EplApiSetCdcFilename(char* pszCdcFilename_p)
{
tEplKernel      Ret = kEplSuccessful;

    EplApiInstance_g.m_pszCdcFilename = pszCdcFilename_p;

    return Ret;
}
#endif // (EPL_OBD_USE_LOAD_CONCISEDCF != FALSE)


//---------------------------------------------------------------------------
//
// Function:    EplApiCbObdAccess
//
// Description: callback function for OD accesses
//
// Parameters:  pParam_p                = OBD parameter
//
// Returns:     tEplKernel              = error code
//
//
// State:
//
//---------------------------------------------------------------------------

tEplKernel PUBLIC EplApiCbObdAccess(tEplObdCbParam MEM* pParam_p)
{
tEplKernel          Ret = kEplSuccessful;

#if (EPL_API_OBD_FORWARD_EVENT != FALSE)
tEplApiEventArg     EventArg;

    // call user callback
    // must be disabled for EplApiLinuxKernel.c, because of reentrancy problem
    // for local OD access. This is not so bad as user callback function in
    // application does not use OD callbacks at the moment.
    EventArg.m_ObdCbParam = *pParam_p;
    Ret = EplApiInstance_g.m_InitParam.m_pfnCbEvent(kEplApiEventObdAccess,
                                                    &EventArg,
                                                    EplApiInstance_g.m_InitParam.m_pEventUserArg);

    if (Ret != kEplSuccessful)
    {   // do not do any further processing on this object
        if (Ret == kEplReject)
        {
            Ret = kEplSuccessful;
        }

        goto Exit;
    }
#endif

    switch (pParam_p->m_uiIndex)
    {
        //case 0x1006:    // NMT_CycleLen_U32 (valid on reset)
        case 0x1C14:    // DLL_LossOfFrameTolerance_U32
        //case 0x1F98:    // NMT_CycleTiming_REC (valid on reset)
        {
            if (pParam_p->m_ObdEvent == kEplObdEvPostWrite)
            {
                // update DLL configuration
                Ret = EplApiUpdateDllConfig(FALSE);
            }
            break;
        }

        case 0x1020:    // CFM_VerifyConfiguration_REC.ConfId_U32 != 0
        {
            if ((pParam_p->m_ObdEvent == kEplObdEvPostWrite)
                && (pParam_p->m_uiSubIndex == 3)
                && (*((DWORD*)pParam_p->m_pArg) != 0))
            {
            DWORD   dwVerifyConfInvalid = 0;
                // set CFM_VerifyConfiguration_REC.VerifyConfInvalid_U32 to 0
                Ret = EplObdWriteEntry(0x1020, 4, &dwVerifyConfInvalid, 4);
                // ignore any error because this objekt is optional
                Ret = kEplSuccessful;
            }
            break;
        }

        case 0x1F9E:    // NMT_ResetCmd_U8
        {
            if (pParam_p->m_ObdEvent == kEplObdEvPreWrite)
            {
            BYTE    bNmtCommand;

                bNmtCommand = *((BYTE *) pParam_p->m_pArg);
                // check value range
                switch ((tNmtCommand)bNmtCommand)
                {
                    case kNmtCmdResetNode:
                    case kNmtCmdResetCommunication:
                    case kNmtCmdResetConfiguration:
                    case kNmtCmdSwReset:
                    case kNmtCmdInvalidService:
                        // valid command identifier specified
                        break;

                    default:
                        pParam_p->m_dwAbortCode = EPL_SDOAC_VALUE_RANGE_EXCEEDED;
                        Ret = kEplObdAccessViolation;
                        break;
                }
            }
            else if (pParam_p->m_ObdEvent == kEplObdEvPostWrite)
            {
            BYTE    bNmtCommand;

                bNmtCommand = *((BYTE *) pParam_p->m_pArg);
                // check value range
                switch ((tNmtCommand)bNmtCommand)
                {
#if(((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMTU)) != 0)
                    case kNmtCmdResetNode:
                        Ret = nmtu_postNmtEvent(kNmtEventResetNode);
                        break;

                    case kNmtCmdResetCommunication:
                        Ret = nmtu_postNmtEvent(kNmtEventResetCom);
                        break;

                    case kNmtCmdResetConfiguration:
                        Ret = nmtu_postNmtEvent(kNmtEventResetConfig);
                        break;

                    case kNmtCmdSwReset:
                        Ret = nmtu_postNmtEvent(kNmtEventSwReset);
                        break;
#endif

                    case kNmtCmdInvalidService:
                        break;

                    default:
                        pParam_p->m_dwAbortCode = EPL_SDOAC_VALUE_RANGE_EXCEEDED;
                        Ret = kEplObdAccessViolation;
                        break;
                }
            }
            break;
        }

#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)
        case 0x1F9F:    // NMT_RequestCmd_REC
        {
            if ((pParam_p->m_ObdEvent == kEplObdEvPostWrite)
                && (pParam_p->m_uiSubIndex == 1)
                && (*((BYTE*)pParam_p->m_pArg) != 0))
            {
            BYTE        bCmdId;
            BYTE        bCmdTarget;
            tEplObdSize ObdSize;
            tNmtState    NmtState;

                ObdSize = sizeof (bCmdId);
                Ret = EplObdReadEntry(0x1F9F, 2, &bCmdId, &ObdSize);
                if (Ret != kEplSuccessful)
                {
                    pParam_p->m_dwAbortCode = EPL_SDOAC_GENERAL_ERROR;
                    goto Exit;
                }

                ObdSize = sizeof (bCmdTarget);
                Ret = EplObdReadEntry(0x1F9F, 3, &bCmdTarget, &ObdSize);
                if (Ret != kEplSuccessful)
                {
                    pParam_p->m_dwAbortCode = EPL_SDOAC_GENERAL_ERROR;
                    goto Exit;
                }

                NmtState = nmtu_getNmtState();

                if (NmtState < kNmtMsNotActive)
                {   // local node is CN
                    // forward the command to the MN
                    // d.k. this is a manufacturer specific feature
                    Ret = nmtcnu_sendNmtRequest(bCmdTarget,
                                                  (tNmtCommand) bCmdId);
                }
                else
                {   // local node is MN
                    // directly execute the requested NMT command
                    Ret = nmtmnu_requestNmtCommand(bCmdTarget,
                                                     (tNmtCommand) bCmdId);
                }
                if (Ret != kEplSuccessful)
                {
                    pParam_p->m_dwAbortCode = EPL_SDOAC_GENERAL_ERROR;
                }

                // reset request flag
                *((BYTE*)pParam_p->m_pArg) = 0;
            }
            break;
        }
#endif // (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)

        default:
            break;
    }

Exit:
    return Ret;
}

//---------------------------------------------------------------------------
//
// Function:    EplApiProcess
//
// Description: Process function for use in single threaded environment
//              e.g. without any OS. It gives processing time to several
//              tasks in the EPL stack.
//
// Parameters:  none
//
// Returns:     tEplKernel              = error code
//
//
// State:
//
//---------------------------------------------------------------------------

tEplKernel PUBLIC EplApiProcess(void)
{
    return ctrlu_processStack();
}

//------------------------------------------------------------------------------
/**
\brief Check if kernel stack is alive

The function checks if the kernel part of the stack is alive.

\return Returns the status of the kernel stack.
\retval TRUE        If the kernel stack is alive.
\retval FALSE       IF the kernel stack is dead.

\ingroup module_api
*/
//------------------------------------------------------------------------------
BOOL PUBLIC api_checkKernelStack(void)
{
    return ctrlu_checkKernelStack();
}

//------------------------------------------------------------------------------
/**
\brief Wait for sync event

The function waits for a sync event.

\param  timeout_p       Time to wait for event

\return The function returns a tEplKernel error code.

\ingroup module_api
*/
//------------------------------------------------------------------------------
tEplKernel PUBLIC api_waitSyncEvent(ULONG timeout_p)
{
    return pdoucal_waitSyncEvent(timeout_p);
}

//=========================================================================//
//                                                                         //
//          P R I V A T E   F U N C T I O N S                              //
//                                                                         //
//=========================================================================//

//---------------------------------------------------------------------------
//
// Function:    EplApiProcessEvent
//
// Description: processes events from event queue and forwards these to
//              the application's event callback function
//
// Parameters:  pEplEvent_p =   pointer to event
//
// Returns:     tEplKernel  = errorcode
//
// State:
//
//---------------------------------------------------------------------------

static tEplKernel PUBLIC EplApiProcessEvent(tEplEvent* pEplEvent_p)
{
tEplKernel          Ret;
tEplEventError*     pEventError;
tEplApiEventType    EventType;

    Ret = kEplSuccessful;

    // process event
    switch(pEplEvent_p->m_EventType)
    {
        // error event
        case kEplEventTypeError:
        {
            pEventError = (tEplEventError*) pEplEvent_p->m_pArg;
            switch (pEventError->m_EventSource)
            {
                // treat the errors from the following sources as critical
                case kEplEventSourceEventk:
                case kEplEventSourceEventu:
                case kEplEventSourceDllk:
                {
                    EventType = kEplApiEventCriticalError;
                    // halt the stack by entering NMT state Off
                    Ret = nmtu_postNmtEvent(kNmtEventCriticalError);
                    break;
                }

                // the other errors are just warnings
                default:
                {
                    EventType = kEplApiEventWarning;
                    break;
                }
            }

            // call user callback
            Ret = EplApiInstance_g.m_InitParam.m_pfnCbEvent(EventType, (tEplApiEventArg*) pEventError, EplApiInstance_g.m_InitParam.m_pEventUserArg);
            // discard error from callback function, because this could generate an endless loop
            Ret = kEplSuccessful;
            break;
        }

        // Error history entry event
        case kEplEventTypeHistoryEntry:
        {
            if (pEplEvent_p->m_uiSize != sizeof (tEplErrHistoryEntry))
            {
                Ret = kEplEventWrongSize;
                goto Exit;
            }
            EventType = kEplApiEventHistoryEntry;

            // call user callback
            Ret = EplApiInstance_g.m_InitParam.m_pfnCbEvent(EventType, (tEplApiEventArg*) pEplEvent_p->m_pArg, EplApiInstance_g.m_InitParam.m_pEventUserArg);
            break;
        }

        // user-defined event
        case kEplEventTypeApiUserDef:
        {
        tEplApiEventArg ApiEventArg;

            EventType = kEplApiEventUserDef;
            ApiEventArg.m_pUserArg = *(void**)pEplEvent_p->m_pArg;

            // call user callback
            Ret = EplApiInstance_g.m_InitParam.m_pfnCbEvent(EventType, &ApiEventArg, EplApiInstance_g.m_InitParam.m_pEventUserArg);
            break;
        }

        // at present, there are no other events for this module
        default:
        {
            Ret = kEplInvalidEvent;
            break;
        }
    }

Exit:
    return Ret;
}

//---------------------------------------------------------------------------
//
// Function:    EplApiCbNmtStateChange
//
// Description: callback function for NMT state changes
//
// Parameters:  NmtStateChange_p        = NMT state change event
//
// Returns:     tEplKernel              = error code
//
//
// State:
//
//---------------------------------------------------------------------------

static tEplKernel PUBLIC EplApiCbNmtStateChange(tEventNmtStateChange NmtStateChange_p)
{
tEplKernel          Ret = kEplSuccessful;
BYTE                bNmtState;
tEplApiEventArg     EventArg;

    // save NMT state in OD
    bNmtState = (BYTE) NmtStateChange_p.newNmtState;
    Ret = EplObdWriteEntry(0x1F8C, 0, &bNmtState, 1);
    if(Ret != kEplSuccessful)
    {
        goto Exit;
    }

    // do work which must be done in that state
    switch (NmtStateChange_p.newNmtState)
    {
        // EPL stack is not running
        case kNmtGsOff:
            break;

        // first init of the hardware
        case kNmtGsInitialising:
#if 0
#if(((EPL_MODULE_INTEGRATION) & (EPL_MODULE_SDO_UDP)) != 0)
            // configure SDO via UDP (i.e. bind it to the EPL ethernet interface)
            Ret = EplSdoUdpuConfig(EplApiInstance_g.m_InitParam.m_dwIpAddress, EPL_C_SDO_EPL_PORT);
            if (Ret != kEplSuccessful)
            {
                goto Exit;
            }
#endif
#endif

            break;

        // init of the manufacturer-specific profile area and the
        // standardised device profile area
        case kNmtGsResetApplication:
        {
            // reset application part of OD
            Ret = EplObdAccessOdPart(
                kEplObdPartApp,
                kEplObdDirLoad);
            if (Ret != kEplSuccessful)
            {
                goto Exit;
            }

            break;
        }

        // init of the communication profile area
        case kNmtGsResetCommunication:
        {
            // reset communication part of OD
            Ret = EplObdAccessOdPart(
                kEplObdPartGen,
                kEplObdDirLoad);

            if (Ret != kEplSuccessful)
            {
                goto Exit;
            }

            // $$$ d.k.: update OD only if OD was not loaded from non-volatile memory
            Ret = EplApiUpdateObd();
            if (Ret != kEplSuccessful)
            {
                goto Exit;
            }

#if (EPL_OBD_USE_LOAD_CONCISEDCF != FALSE)
            if (EplApiInstance_g.m_pbCdc != NULL)
            {
                Ret = EplObdCdcLoadBuffer(EplApiInstance_g.m_pbCdc, EplApiInstance_g.m_uiCdcSize);
            }
            else if (EplApiInstance_g.m_pszCdcFilename != NULL)
            {
                Ret = EplObdCdcLoadFile(EplApiInstance_g.m_pszCdcFilename);
            }
            else
            {
                Ret = EplObdCdcLoadFile(EPL_OBD_DEF_CONCISEDCF_FILENAME);
            }
            if (Ret != kEplSuccessful)
            {
                if (Ret == kEplReject)
                {
                    Ret = kEplSuccessful;
                }
                else
                {
                    goto Exit;
                }
            }

#endif
            break;
        }

        // build the configuration with infos from OD
        case kNmtGsResetConfiguration:
        {

            Ret = EplApiUpdateDllConfig(TRUE);
            if (Ret != kEplSuccessful)
            {
                goto Exit;
            }

            Ret = EplApiUpdateSdoConfig();
            if (Ret != kEplSuccessful)
            {
                goto Exit;
            }

            break;
        }

        //-----------------------------------------------------------
        // CN part of the state machine

        // node list for EPL-Frames and check timeout
        case kNmtCsNotActive:
        {
            // indicate completion of reset in NMT_ResetCmd_U8
            bNmtState = (BYTE) kNmtCmdInvalidService;
            Ret = EplObdWriteEntry(0x1F9E, 0, &bNmtState, 1);
            if (Ret != kEplSuccessful)
            {
                goto Exit;
            }

            break;
        }

        // node process only async frames
        case kNmtCsPreOperational1:
        {
            break;
        }

        // node process isochronous and asynchronous frames
        case kNmtCsPreOperational2:
        {
            break;
        }

        // node should be configured and application is ready
        case kNmtCsReadyToOperate:
        {
            break;
        }

        // normal work state
        case kNmtCsOperational:
        {
            break;
        }

        // node stopped by MN
        // -> only process asynchronous frames
        case kNmtCsStopped:
        {
            break;
        }

        // no EPL cycle
        // -> normal ethernet communication
        case kNmtCsBasicEthernet:
        {
            break;
        }

        //-----------------------------------------------------------
        // MN part of the state machine

        // node listens for EPL-Frames and check timeout
        case kNmtMsNotActive:
        {
            break;
        }

        // node processes only async frames
        case kNmtMsPreOperational1:
        {
            break;
        }

        // node processes isochronous and asynchronous frames
        case kNmtMsPreOperational2:
        {
            break;
        }

        // node should be configured and application is ready
        case kNmtMsReadyToOperate:
        {
            break;
        }

        // normal work state
        case kNmtMsOperational:
        {
            break;
        }

        // no EPL cycle
        // -> normal ethernet communication
        case kNmtMsBasicEthernet:
        {
            break;
        }

        default:
        {
            TRACE("EplApiCbNmtStateChange(): unhandled NMT state\n");
            break;
        }
    }

#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_LEDU)) != 0)
    // forward event to Led module
    Ret = ledu_cbNmtStateChange(NmtStateChange_p);
    if (Ret != kEplSuccessful)
    {
        goto Exit;
    }
#endif

#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_PDOU)) != 0)
    // forward event to Pdou module
    Ret = pdou_cbNmtStateChange(NmtStateChange_p);
    if (Ret != kEplSuccessful)
    {
        goto Exit;
    }
#endif

#if(((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)
    // forward event to NmtMn module
    Ret = nmtmnu_cbNmtStateChange(NmtStateChange_p);
    if (Ret != kEplSuccessful)
    {
        goto Exit;
    }
#endif

    // call user callback
    EventArg.m_NmtStateChange = NmtStateChange_p;
    Ret = EplApiInstance_g.m_InitParam.m_pfnCbEvent(kEplApiEventNmtStateChange,
                                                    &EventArg,
                                                    EplApiInstance_g.m_InitParam.m_pEventUserArg);

Exit:
    return Ret;
}

//---------------------------------------------------------------------------
//
// Function:    EplApiUpdateDllConfig
//
// Description: update configuration of DLL
//
// Parameters:  fUpdateIdentity_p       = TRUE, if identity must be updated
//
// Returns:     tEplKernel              = error code
//
//
// State:
//
//---------------------------------------------------------------------------

static tEplKernel PUBLIC EplApiUpdateDllConfig(BOOL fUpdateIdentity_p)
{
tEplKernel          Ret = kEplSuccessful;
tDllConfigParam     DllConfigParam;
tDllIdentParam      DllIdentParam;
tEplObdSize         ObdSize;
WORD                wTemp;
BYTE                bTemp;

    // configure Dll
    EPL_MEMSET(&DllConfigParam, 0, sizeof (DllConfigParam));
    DllConfigParam.nodeId = EplObdGetNodeId();

    // Cycle Length (0x1006: NMT_CycleLen_U32) in [us]
    ObdSize = 4;
    Ret = EplObdReadEntry(0x1006, 0, &DllConfigParam.cycleLen, &ObdSize);
    if(Ret != kEplSuccessful)
    {
        goto Exit;
    }

    // 0x1F82: NMT_FeatureFlags_U32
    ObdSize = 4;
    Ret = EplObdReadEntry(0x1F82, 0, &DllConfigParam.featureFlags, &ObdSize);
    if(Ret != kEplSuccessful)
    {
        goto Exit;
    }

    // d.k. There is no dependence between FeatureFlags and async-only CN
    DllConfigParam.fAsyncOnly = EplApiInstance_g.m_InitParam.m_fAsyncOnly;

    // 0x1C14: DLL_LossOfFrameTolerance_U32 in [ns]
    ObdSize = 4;
    Ret = EplObdReadEntry(0x1C14, 0, &DllConfigParam.lossOfFrameTolerance, &ObdSize);
    if(Ret != kEplSuccessful)
    {
        goto Exit;
    }

    // 0x1F98: NMT_CycleTiming_REC
    // 0x1F98.1: IsochrTxMaxPayload_U16
    ObdSize = 2;
    Ret = EplObdReadEntry(0x1F98, 1, &wTemp, &ObdSize);
    if(Ret != kEplSuccessful)
    {
        goto Exit;
    }
    DllConfigParam.isochrTxMaxPayload = wTemp;

    // 0x1F98.2: IsochrRxMaxPayload_U16
    ObdSize = 2;
    Ret = EplObdReadEntry(0x1F98, 2, &wTemp, &ObdSize);
    if(Ret != kEplSuccessful)
    {
        goto Exit;
    }
    DllConfigParam.isochrRxMaxPayload = wTemp;

    // 0x1F98.3: PResMaxLatency_U32
    ObdSize = 4;
    Ret = EplObdReadEntry(0x1F98, 3, &DllConfigParam.presMaxLatency, &ObdSize);
    if(Ret != kEplSuccessful)
    {
        goto Exit;
    }

    // 0x1F98.4: PReqActPayloadLimit_U16
    ObdSize = 2;
    Ret = EplObdReadEntry(0x1F98, 4, &wTemp, &ObdSize);
    if(Ret != kEplSuccessful)
    {
        goto Exit;
    }
    DllConfigParam.preqActPayloadLimit = wTemp;

    // 0x1F98.5: PResActPayloadLimit_U16
    ObdSize = 2;
    Ret = EplObdReadEntry(0x1F98, 5, &wTemp, &ObdSize);
    if(Ret != kEplSuccessful)
    {
        goto Exit;
    }
    DllConfigParam.presActPayloadLimit = wTemp;

    // 0x1F98.6: ASndMaxLatency_U32
    ObdSize = 4;
    Ret = EplObdReadEntry(0x1F98, 6, &DllConfigParam.asndMaxLatency, &ObdSize);
    if(Ret != kEplSuccessful)
    {
        goto Exit;
    }

    // 0x1F98.7: MultiplCycleCnt_U8
    ObdSize = 1;
    Ret = EplObdReadEntry(0x1F98, 7, &bTemp, &ObdSize);
    if(Ret != kEplSuccessful)
    {
        goto Exit;
    }
    DllConfigParam.multipleCycleCnt = bTemp;

    // 0x1F98.8: AsyncMTU_U16
    ObdSize = 2;
    Ret = EplObdReadEntry(0x1F98, 8, &wTemp, &ObdSize);
    if(Ret != kEplSuccessful)
    {
        goto Exit;
    }
    DllConfigParam.asyncMtu = wTemp;

    // $$$ Prescaler

#if(((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)
    // 0x1F8A.1: WaitSoCPReq_U32 in [ns]
    ObdSize = 4;
    Ret = EplObdReadEntry(0x1F8A, 1, &DllConfigParam.waitSocPreq, &ObdSize);
    if(Ret != kEplSuccessful)
    {
        goto Exit;
    }

    // 0x1F8A.2: AsyncSlotTimeout_U32 in [ns] (optional)
    ObdSize = 4;
    Ret = EplObdReadEntry(0x1F8A, 2, &DllConfigParam.asyncSlotTimeout, &ObdSize);
/*    if(Ret != kEplSuccessful)
    {
        goto Exit;
    }*/
#endif

#if EPL_DLL_PRES_CHAINING_CN != FALSE
    DllConfigParam.syncResLatency = EplApiInstance_g.m_InitParam.m_dwSyncResLatency;
#endif

    DllConfigParam.fSyncOnPrcNode = EplApiInstance_g.m_InitParam.m_fSyncOnPrcNode;
    DllConfigParam.syncNodeId = EplApiInstance_g.m_InitParam.m_uiSyncNodeId;

    DllConfigParam.sizeOfStruct = sizeof (DllConfigParam);
    Ret = dllucal_config(&DllConfigParam);
    if(Ret != kEplSuccessful)
    {
        goto Exit;
    }

    if (fUpdateIdentity_p != FALSE)
    {
        // configure Identity
        EPL_MEMSET(&DllIdentParam, 0, sizeof (DllIdentParam));

        ObdSize = 4;
        Ret = EplObdReadEntry(0x1000, 0, &DllIdentParam.deviceType, &ObdSize);
        if(Ret != kEplSuccessful)
        {
            goto Exit;
        }

        ObdSize = 4;
        Ret = EplObdReadEntry(0x1018, 1, &DllIdentParam.vendorId, &ObdSize);
        if(Ret != kEplSuccessful)
        {
            goto Exit;
        }
        ObdSize = 4;
        Ret = EplObdReadEntry(0x1018, 2, &DllIdentParam.productCode, &ObdSize);
        if(Ret != kEplSuccessful)
        {
            goto Exit;
        }
        ObdSize = 4;
        Ret = EplObdReadEntry(0x1018, 3, &DllIdentParam.revisionNumber, &ObdSize);
        if(Ret != kEplSuccessful)
        {
            goto Exit;
        }
        ObdSize = 4;
        Ret = EplObdReadEntry(0x1018, 4, &DllIdentParam.serialNumber, &ObdSize);
        if(Ret != kEplSuccessful)
        {
            goto Exit;
        }

        DllIdentParam.ipAddress = EplApiInstance_g.m_InitParam.m_dwIpAddress;
        DllIdentParam.subnetMask = EplApiInstance_g.m_InitParam.m_dwSubnetMask;

        ObdSize = sizeof (DllIdentParam.defaultGateway);
        Ret = EplObdReadEntry(0x1E40, 5, &DllIdentParam.defaultGateway, &ObdSize);
        if (Ret != kEplSuccessful)
        {   // NWL_IpAddrTable_Xh_REC.DefaultGateway_IPAD seams to not exist,
            // so use the one supplied in the init parameter
            DllIdentParam.defaultGateway = EplApiInstance_g.m_InitParam.m_dwDefaultGateway;
        }

#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_VETH)) != 0)
        // configure Virtual Ethernet Driver
        Ret = veth_setIpAdrs(DllIdentParam.ipAddress, DllIdentParam.m_dwSubnetMask, (WORD) DllConfigParam.m_uiAsyncMtu);
        if(Ret != kEplSuccessful)
        {
            goto Exit;
        }

        Ret = veth_setDefaultGateway(DllIdentParam.m_dwDefaultGateway);
        if(Ret != kEplSuccessful)
        {
            goto Exit;
        }
#endif

        ObdSize = sizeof (DllIdentParam.sHostname);
        Ret = EplObdReadEntry(0x1F9A, 0, &DllIdentParam.sHostname[0], &ObdSize);
        if (Ret != kEplSuccessful)
        {   // NMT_HostName_VSTR seams to not exist,
            // so use the one supplied in the init parameter
            EPL_MEMCPY(DllIdentParam.sHostname, EplApiInstance_g.m_InitParam.m_sHostname, sizeof (DllIdentParam.sHostname));
        }

        ObdSize = 4;
        Ret = EplObdReadEntry(0x1020, 1, &DllIdentParam.verifyConfigurationDate, &ObdSize);
        // ignore any error, because this object is optional

        ObdSize = 4;
        Ret = EplObdReadEntry(0x1020, 2, &DllIdentParam.verifyConfigurationTime, &ObdSize);
        // ignore any error, because this object is optional

        DllIdentParam.applicationSwDate = EplApiInstance_g.m_InitParam.m_dwApplicationSwDate;
        DllIdentParam.applicationSwTime = EplApiInstance_g.m_InitParam.m_dwApplicationSwTime;

        DllIdentParam.vendorSpecificExt1 = EplApiInstance_g.m_InitParam.m_qwVendorSpecificExt1;

        EPL_MEMCPY(&DllIdentParam.aVendorSpecificExt2[0], &EplApiInstance_g.m_InitParam.m_abVendorSpecificExt2[0], sizeof (DllIdentParam.aVendorSpecificExt2));

        DllIdentParam.sizeOfStruct = sizeof (DllIdentParam);
        Ret = dllucal_setIdentity(&DllIdentParam);
        if(Ret != kEplSuccessful)
        {
            goto Exit;
        }
    }

Exit:
    return Ret;
}

//---------------------------------------------------------------------------
//
// Function:    EplApiUpdateSdoConfig
//
// Description: update configuration of SDO modules
//
// Parameters:  -
//
// Returns:     tEplKernel              = error code
//
//
//---------------------------------------------------------------------------
static tEplKernel PUBLIC EplApiUpdateSdoConfig()
{
    tEplKernel          Ret = kEplSuccessful;
    tEplObdSize         ObdSize;
    DWORD               SdoSequTimeout;

    ObdSize = sizeof(SdoSequTimeout);
    Ret = EplObdReadEntry(0x1300, 0, &SdoSequTimeout, &ObdSize);
    if(Ret != kEplSuccessful)
    {
        goto Exit;
    }

    Ret = EplSdoAsySeqSetTimeout( SdoSequTimeout );
    if(Ret != kEplSuccessful)
    {
        goto Exit;
    }

Exit:
    return Ret;
}
//---------------------------------------------------------------------------
//
// Function:    EplApiUpdateObd
//
// Description: update OD from init param
//
// Parameters:  (none)
//
// Returns:     tEplKernel              = error code
//
//
// State:
//
//---------------------------------------------------------------------------

static tEplKernel PUBLIC EplApiUpdateObd(void)
{
tEplKernel          Ret = kEplSuccessful;
WORD                wTemp;
BYTE                bTemp;

    // set node id in OD
    Ret = EplObdSetNodeId(EplApiInstance_g.m_InitParam.m_uiNodeId,    // node id
                            kEplObdNodeIdHardware); // set by hardware
    if(Ret != kEplSuccessful)
    {
        goto Exit;
    }

    if (EplApiInstance_g.m_InitParam.m_dwCycleLen != UINT_MAX)
    {
        Ret = EplObdWriteEntry(0x1006, 0,
                        &EplApiInstance_g.m_InitParam.m_dwCycleLen,
                        4);
    }

    if (EplApiInstance_g.m_InitParam.m_dwLossOfFrameTolerance != UINT_MAX)
    {
        Ret = EplObdWriteEntry(0x1C14, 0,
                        &EplApiInstance_g.m_InitParam.m_dwLossOfFrameTolerance,
                        4);
    }

    // d.k. There is no dependance between FeatureFlags and async-only CN.
    if (EplApiInstance_g.m_InitParam.m_dwFeatureFlags != UINT_MAX)
    {
        Ret = EplObdWriteEntry(0x1F82, 0,
                               &EplApiInstance_g.m_InitParam.m_dwFeatureFlags,
                               4);
    }

    wTemp = (WORD) EplApiInstance_g.m_InitParam.m_uiIsochrTxMaxPayload;
    Ret = EplObdWriteEntry(0x1F98, 1, &wTemp, 2);

    wTemp = (WORD) EplApiInstance_g.m_InitParam.m_uiIsochrRxMaxPayload;
    Ret = EplObdWriteEntry(0x1F98, 2, &wTemp, 2);

    Ret = EplObdWriteEntry(0x1F98, 3,
                           &EplApiInstance_g.m_InitParam.m_dwPresMaxLatency,
                           4);

    if (EplApiInstance_g.m_InitParam.m_uiPreqActPayloadLimit <= EPL_C_DLL_ISOCHR_MAX_PAYL)
    {
        wTemp = (WORD) EplApiInstance_g.m_InitParam.m_uiPreqActPayloadLimit;
        Ret = EplObdWriteEntry(0x1F98, 4, &wTemp, 2);
    }

    if (EplApiInstance_g.m_InitParam.m_uiPresActPayloadLimit <= EPL_C_DLL_ISOCHR_MAX_PAYL)
    {
        wTemp = (WORD) EplApiInstance_g.m_InitParam.m_uiPresActPayloadLimit;
        Ret = EplObdWriteEntry(0x1F98, 5, &wTemp, 2);
    }

    Ret = EplObdWriteEntry(0x1F98, 6,
                           &EplApiInstance_g.m_InitParam.m_dwAsndMaxLatency,
                           4);

    if (EplApiInstance_g.m_InitParam.m_uiMultiplCycleCnt <= 0xFF)
    {
        bTemp = (BYTE) EplApiInstance_g.m_InitParam.m_uiMultiplCycleCnt;
        Ret = EplObdWriteEntry(0x1F98, 7, &bTemp, 1);
    }

    if (EplApiInstance_g.m_InitParam.m_uiAsyncMtu <= EPL_C_DLL_MAX_ASYNC_MTU)
    {
        wTemp = (WORD) EplApiInstance_g.m_InitParam.m_uiAsyncMtu;
        Ret = EplObdWriteEntry(0x1F98, 8, &wTemp, 2);
    }

    if (EplApiInstance_g.m_InitParam.m_uiPrescaler <= 1000)
    {
        wTemp = (WORD) EplApiInstance_g.m_InitParam.m_uiPrescaler;
        Ret = EplObdWriteEntry(0x1F98, 9, &wTemp, 2);
    }

#if(((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)
    if (EplApiInstance_g.m_InitParam.m_dwWaitSocPreq != UINT_MAX)
    {
        Ret = EplObdWriteEntry(0x1F8A, 1,
                               &EplApiInstance_g.m_InitParam.m_dwWaitSocPreq,
                               4);
    }

    if ((EplApiInstance_g.m_InitParam.m_dwAsyncSlotTimeout != 0) && (EplApiInstance_g.m_InitParam.m_dwAsyncSlotTimeout != UINT_MAX))
    {
        Ret = EplObdWriteEntry(0x1F8A, 2,
                            &EplApiInstance_g.m_InitParam.m_dwAsyncSlotTimeout,
                            4);
    }
#endif

    // configure Identity
    if (EplApiInstance_g.m_InitParam.m_dwDeviceType != UINT_MAX)
    {
        Ret = EplObdWriteEntry(0x1000, 0,
                               &EplApiInstance_g.m_InitParam.m_dwDeviceType,
                               4);
    }

    if (EplApiInstance_g.m_InitParam.m_dwVendorId != UINT_MAX)
    {
        Ret = EplObdWriteEntry(0x1018, 1,
                               &EplApiInstance_g.m_InitParam.m_dwVendorId,
                               4);
    }

    if (EplApiInstance_g.m_InitParam.m_dwProductCode != UINT_MAX)
    {
        Ret = EplObdWriteEntry(0x1018, 2,
                               &EplApiInstance_g.m_InitParam.m_dwProductCode,
                               4);
    }

    if (EplApiInstance_g.m_InitParam.m_dwRevisionNumber != UINT_MAX)
    {
        Ret = EplObdWriteEntry(0x1018, 3,
                               &EplApiInstance_g.m_InitParam.m_dwRevisionNumber,
                               4);
    }

    if (EplApiInstance_g.m_InitParam.m_dwSerialNumber != UINT_MAX)
    {
        Ret = EplObdWriteEntry(0x1018, 4,
                               &EplApiInstance_g.m_InitParam.m_dwSerialNumber,
                               4);
    }

    if (EplApiInstance_g.m_InitParam.m_pszDevName != NULL)
    {
        // write Device Name (0x1008)
        Ret = EplObdWriteEntry (
            0x1008, 0,
            (void GENERIC*) EplApiInstance_g.m_InitParam.m_pszDevName,
            (tEplObdSize) strlen(EplApiInstance_g.m_InitParam.m_pszDevName));
    }

    if (EplApiInstance_g.m_InitParam.m_pszHwVersion != NULL)
    {
        // write Hardware version (0x1009)
        Ret = EplObdWriteEntry (
            0x1009, 0,
            (void GENERIC*) EplApiInstance_g.m_InitParam.m_pszHwVersion,
            (tEplObdSize) strlen(EplApiInstance_g.m_InitParam.m_pszHwVersion));
    }

    if (EplApiInstance_g.m_InitParam.m_pszSwVersion != NULL)
    {
        // write Software version (0x100A)
        Ret = EplObdWriteEntry (
            0x100A, 0,
            (void GENERIC*) EplApiInstance_g.m_InitParam.m_pszSwVersion,
            (tEplObdSize) strlen(EplApiInstance_g.m_InitParam.m_pszSwVersion));
    }

#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_VETH)) != 0)
    // write NMT_HostName_VSTR (0x1F9A)
    Ret = EplObdWriteEntry (
        0x1F9A, 0,
        (void GENERIC*) &EplApiInstance_g.m_InitParam.m_sHostname[0],
        sizeof (EplApiInstance_g.m_InitParam.m_sHostname));

//    PRINTF("%s: write NMT_HostName_VSTR %d\n", __func__, Ret);

    // write NWL_IpAddrTable_Xh_REC.Addr_IPAD (0x1E40/2)
    Ret = EplObdWriteEntry (
        0x1E40, 2,
        (void GENERIC*) &EplApiInstance_g.m_InitParam.m_dwIpAddress,
        sizeof (EplApiInstance_g.m_InitParam.m_dwIpAddress));

    // write NWL_IpAddrTable_Xh_REC.NetMask_IPAD (0x1E40/3)
    Ret = EplObdWriteEntry (
        0x1E40, 3,
        (void GENERIC*) &EplApiInstance_g.m_InitParam.m_dwSubnetMask,
        sizeof (EplApiInstance_g.m_InitParam.m_dwSubnetMask));

    // write NWL_IpAddrTable_Xh_REC.DefaultGateway_IPAD (0x1E40/5)
    Ret = EplObdWriteEntry (
        0x1E40, 5,
        (void GENERIC*) &EplApiInstance_g.m_InitParam.m_dwDefaultGateway,
        sizeof (EplApiInstance_g.m_InitParam.m_dwDefaultGateway));

#endif // (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_VETH)) != 0)

    // ignore return code
    Ret = kEplSuccessful;

Exit:
    return Ret;
}


#if(((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)
//---------------------------------------------------------------------------
//
// Function:    EplApiGetIdentResponse
//
// Description: returns the stored IdentResponse frame of the specified node.
//
// Parameters:  uiNodeId_p              = node-ID for which the IdentResponse shall be returned
//              ppIdentResponse_p       = pointer to pointer to IdentResponse
//
// Returns:     tEplKernel              = error code
//
//
// State:
//
//---------------------------------------------------------------------------

tEplKernel PUBLIC EplApiGetIdentResponse(
                                    unsigned int        uiNodeId_p,
                                    tEplIdentResponse** ppIdentResponse_p)
{
    return identu_getIdentResponse(uiNodeId_p, ppIdentResponse_p);
}
#endif


//---------------------------------------------------------------------------
//
// Function:    EplApiCbSdoCon
//
// Description: callback function for SDO transfers
//
// Parameters:  pSdoComFinished_p       = SDO parameter
//
// Returns:     tEplKernel              = error code
//
//
// State:
//
//---------------------------------------------------------------------------

#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_SDOC)) != 0)
static tEplKernel PUBLIC  EplApiCbSdoCon(tEplSdoComFinished* pSdoComFinished_p)
{
tEplKernel Ret;
tEplApiEventArg EventArg;

    Ret = kEplSuccessful;

    // call user callback
    EventArg.m_Sdo = *pSdoComFinished_p;
    Ret = EplApiInstance_g.m_InitParam.m_pfnCbEvent(kEplApiEventSdo,
                                                    &EventArg,
                                                    EplApiInstance_g.m_InitParam.m_pEventUserArg);

    return Ret;

}
#endif


#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)

//---------------------------------------------------------------------------
//
// Function:    EplApiCbNodeEvent
//
// Description: callback function for node events
//
// Parameters:  uiNodeId_p              = node ID of the CN
//              NodeEvent_p             = event from the specified CN
//              NmtState_p              = current NMT state of the CN
//              wErrorCode_p            = EPL error code if NodeEvent_p==kNmtNodeEventError
//              fMandatory_p            = flag if CN is mandatory
//
// Returns:     tEplKernel              = error code
//
//
// State:
//
//---------------------------------------------------------------------------

static tEplKernel PUBLIC  EplApiCbNodeEvent(unsigned int uiNodeId_p,
                                            tNmtNodeEvent NodeEvent_p,
                                            tNmtState NmtState_p,
                                            WORD wErrorCode_p,
                                            BOOL fMandatory_p)
{
tEplKernel Ret;
tEplApiEventArg EventArg;

    Ret = kEplSuccessful;

    // call user callback
    EventArg.m_Node.m_uiNodeId = uiNodeId_p;
    EventArg.m_Node.m_NodeEvent = NodeEvent_p;
    EventArg.m_Node.m_NmtState = NmtState_p;
    EventArg.m_Node.m_wErrorCode = wErrorCode_p;
    EventArg.m_Node.m_fMandatory = fMandatory_p;

    Ret = EplApiInstance_g.m_InitParam.m_pfnCbEvent(kEplApiEventNode,
                                                    &EventArg,
                                                    EplApiInstance_g.m_InitParam.m_pEventUserArg);
    if (Ret != kEplSuccessful)
    {
        goto Exit;
    }

#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_CFM)) != 0)
    Ret = cfmu_processNodeEvent(uiNodeId_p, NodeEvent_p);
#endif

Exit:
    return Ret;

}

#endif // (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)

//---------------------------------------------------------------------------
//
// Function:    EplApiCbBootEvent
//
// Description: callback function for boot events
//
// Parameters:  BootEvent_p             = event from the boot-up process
//              NmtState_p              = current local NMT state
//              wErrorCode_p            = EPL error code if BootEvent_p==kNmtBootEventError
//
// Returns:     tEplKernel              = error code
//
//
// State:
//
//---------------------------------------------------------------------------

static tEplKernel PUBLIC  EplApiCbBootEvent(tNmtBootEvent BootEvent_p,
                                            tNmtState NmtState_p,
                                            WORD wErrorCode_p)
{
tEplKernel Ret;
tEplApiEventArg EventArg;

    Ret = kEplSuccessful;

    // call user callback
    EventArg.m_Boot.m_BootEvent = BootEvent_p;
    EventArg.m_Boot.m_NmtState = NmtState_p;
    EventArg.m_Boot.m_wErrorCode = wErrorCode_p;

    Ret = EplApiInstance_g.m_InitParam.m_pfnCbEvent(kEplApiEventBoot,
                                                    &EventArg,
                                                    EplApiInstance_g.m_InitParam.m_pEventUserArg);

    return Ret;

}



#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_LEDU)) != 0)

//---------------------------------------------------------------------------
//
// Function:    EplApiCbLedStateChange
//
// Description: callback function for LED change events.
//
// Parameters:  LedType_p       = type of LED
//              fOn_p           = state of LED
//
// Returns:     tEplKernel      = errorcode
//
// State:
//
//---------------------------------------------------------------------------

static tEplKernel PUBLIC  EplApiCbLedStateChange(tLedType LedType_p,
                                                 BOOL fOn_p)
{
tEplKernel Ret;
tEplApiEventArg EventArg;

    Ret = kEplSuccessful;

    // call user callback
    EventArg.m_Led.m_LedType = LedType_p;
    EventArg.m_Led.m_fOn = fOn_p;

    Ret = EplApiInstance_g.m_InitParam.m_pfnCbEvent(kEplApiEventLed,
                                                    &EventArg,
                                                    EplApiInstance_g.m_InitParam.m_pEventUserArg);

    return Ret;

}

#endif


#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_CFM)) != 0)

//---------------------------------------------------------------------------
//
// Function:    EplApiCbCfmEventCnProgress
//
// Description: callback function for CFM progress events.
//
// Parameters:  pEventCnProgress_p  = pointer to structure with additional information
//
// Returns:     tEplKernel      = errorcode
//
// State:
//
//---------------------------------------------------------------------------

static tEplKernel PUBLIC  EplApiCbCfmEventCnProgress(tCfmEventCnProgress* pEventCnProgress_p)
{
tEplKernel Ret;
tEplApiEventArg EventArg;

    Ret = kEplSuccessful;

    // call user callback
    EventArg.m_CfmProgress = *pEventCnProgress_p;

    Ret = EplApiInstance_g.m_InitParam.m_pfnCbEvent(kEplApiEventCfmProgress,
                                                    &EventArg,
                                                    EplApiInstance_g.m_InitParam.m_pEventUserArg);

    return Ret;
}


//---------------------------------------------------------------------------
//
// Function:    EplApiCbCfmEventCnResult
//
// Description: callback function for CFM CN result events.
//
// Parameters:  uiNodeId_p      = node-ID of CN
//              NodeCommand_p   = NMT command which shall be executed
//
// Returns:     tEplKernel      = errorcode
//
// State:
//
//---------------------------------------------------------------------------

static tEplKernel PUBLIC  EplApiCbCfmEventCnResult(unsigned int uiNodeId_p, tNmtNodeCommand NodeCommand_p)
{
tEplKernel Ret;
tEplApiEventArg EventArg;

    EventArg.m_CfmResult.m_uiNodeId = uiNodeId_p;
    EventArg.m_CfmResult.m_NodeCommand = NodeCommand_p;

    Ret = EplApiInstance_g.m_InitParam.m_pfnCbEvent(kEplApiEventCfmResult,
                                                    &EventArg,
                                                    EplApiInstance_g.m_InitParam.m_pEventUserArg);
    if (Ret != kEplSuccessful)
    {
        if (Ret == kEplReject)
        {
            Ret = kEplSuccessful;
        }
        goto Exit;
    }

    Ret = nmtmnu_triggerStateChange(uiNodeId_p, NodeCommand_p);

Exit:
    return Ret;
}

#endif

//---------------------------------------------------------------------------
//
// Function:    EplApiCbReceivedAsnd
//
// Description: Callback to handle received Asnd frames.
//              Frames will be forwarded to the application layer.
//
// Parameters:  pFrameInfo_p    Pointer to structure with information
//                              about the received frame
//
// Returns:     tEplKernel
//---------------------------------------------------------------------------
static tEplKernel PUBLIC EplApiCbReceivedAsnd
(
    tFrameInfo *pFrameInfo_p
)
{
    tEplKernel          Ret = kEplSuccessful;
    unsigned int        uiAsndOffset;
    tEplApiEventArg     ApiEventArg;
    tEplApiEventType    EventType;

    // Check for correct input
    uiAsndOffset    = offsetof(tEplFrame, m_Data.m_Asnd);

    if
    (
        ( pFrameInfo_p->frameSize    <= uiAsndOffset + 1        ) ||
        ( pFrameInfo_p->frameSize    >  EPL_C_DLL_MAX_ASYNC_MTU )
    )
    {
        return kEplReject;
    }

    // Forward received ASnd frame
    ApiEventArg.m_RcvAsnd.m_pFrame      = pFrameInfo_p->pFrame;
    ApiEventArg.m_RcvAsnd.m_FrameSize   = pFrameInfo_p->frameSize;

    EventType = kEplApiEventReceivedAsnd;

    // call user callback
    Ret = EplApiInstance_g.m_InitParam.m_pfnCbEvent(EventType, &ApiEventArg, EplApiInstance_g.m_InitParam.m_pEventUserArg);

    return Ret;
}


// EOF

