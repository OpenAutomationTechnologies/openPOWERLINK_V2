/****************************************************************************

  (c) SYSTEC electronic GmbH, D-07973 Greiz, August-Bebel-Str. 29
      www.systec-electronic.com

  Project:      openPOWERLINK

  Description:  source file for EPL API module in Linux userspace
                as counterpart to the EPL Linux kernel module

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
                KEIL uVision 2

  -------------------------------------------------------------------------

  Revision History:

  2006/10/11 d.k.:   start of the implementation, version 1.00

****************************************************************************/

#include "Epl.h"
#include "EplApiLinux.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>

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

typedef struct
{
    tEplApiInitParam    m_InitParam;
    tEplApiEventArg     m_EventArg;
    tEplApiEventType    m_EventType;
    tEplLinEvent        m_Event;
    int                 m_hDrvInst;           // driver file descriptior

} tEplApiInstance;

//---------------------------------------------------------------------------
// local vars
//---------------------------------------------------------------------------

static tEplApiInstance  EplApiInstance_g;

//---------------------------------------------------------------------------
// local function prototypes
//---------------------------------------------------------------------------



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
//              EplApiExecNmtCommand(kEplNmtEventSwReset)
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
tEplKernel   Ret = kEplSuccessful;
const char*  pszDrvName;                // driver name
int          iRet;

    // reset instance structure
    EPL_MEMSET(&EplApiInstance_g, 0, sizeof (EplApiInstance_g));

    EPL_MEMCPY(&EplApiInstance_g.m_InitParam, pInitParam_p,
        (sizeof (tEplApiInitParam) < pInitParam_p->m_uiSizeOfStruct) ? sizeof (tEplApiInitParam) : pInitParam_p->m_uiSizeOfStruct);

    // check event callback function pointer
    if (EplApiInstance_g.m_InitParam.m_pfnCbEvent == NULL)
    {   // application must always have an event callback function
        Ret = kEplApiInvalidParam;
        goto Exit;
    }

    pszDrvName = "/dev/"EPLLIN_DEV_NAME;
    EplApiInstance_g.m_hDrvInst = -1;

    // open driver
    TRACE1("EPL: Try to open driver '%s'\n", pszDrvName);
    EplApiInstance_g.m_hDrvInst = open (pszDrvName, O_RDWR);
    if (EplApiInstance_g.m_hDrvInst > -1)
    {
        TRACE2("EPL: open '%s' successful -> hDrvInst=%d\n", pszDrvName, EplApiInstance_g.m_hDrvInst);
    }
    else
    {
        TRACE1("EPL: ERROR: Can't open '%s'\n", pszDrvName);
        Ret = kEplNoResource;
        goto Exit;
    }

    // initialize hardware
    iRet = ioctl (EplApiInstance_g.m_hDrvInst, EPLLIN_CMD_INITIALIZE, (unsigned long)&EplApiInstance_g.m_InitParam);
    if (iRet < 0)
    {
        Ret = kEplNoResource;
        goto Exit;
    }
    else
    {
        Ret = (tEplKernel)iRet;
    }

    // the application must start NMT state machine
    // via EplApiExecNmtCommand(kEplNmtEventSwReset)
    // and thereby the whole EPL stack :-)
/*#if(((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMTU)) != 0)
    Ret = EplNmtuNmtEvent(kEplNmtEventSwReset);
#endif*/

Exit:
    return Ret;
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
tEplKernel      Ret = kEplSuccessful;
int             iRet = 0;

    // delete instance for all modules

    // close driver
    if (EplApiInstance_g.m_hDrvInst >= 0)
    {
        // shutdown the threads
        iRet = ioctl (EplApiInstance_g.m_hDrvInst, EPLLIN_CMD_SHUTDOWN, 0);
        if (iRet < 0)
        {
            Ret = kEplNoResource;
        }

        printf("EplApiShutdown(): calling close(%d) ...\n", EplApiInstance_g.m_hDrvInst);
        iRet = close (EplApiInstance_g.m_hDrvInst);
        EplApiInstance_g.m_hDrvInst = -1;
        if (iRet != 0)
        {
            iRet = errno;
            printf("EplApiShutdown(): close() -> %d\n", iRet);
            Ret = kEplNoResource;
        }
    }
    else
    {
        Ret = kEplIllegalInstance;
    }

    return Ret;
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

tEplKernel PUBLIC EplApiExecNmtCommand(tEplNmtEvent NmtEvent_p)
{
tEplKernel      Ret = kEplSuccessful;
int             iRet;

    // forward NMT command to Linux kernel module
    iRet = ioctl (EplApiInstance_g.m_hDrvInst, EPLLIN_CMD_NMT_COMMAND, (unsigned long)NmtEvent_p);
    if (iRet < 0)
    {
        Ret = kEplNoResource;
    }
    else
    {
        Ret = (tEplKernel)iRet;
    }

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
tEplLinLocalObject  LocalObject;
int             iRet;

    LocalObject.m_uiIndex = uiIndex_p;
    LocalObject.m_uiSubindex = uiSubindex_p;
    LocalObject.m_uiSize = *puiSize_p;
    LocalObject.m_pData = pDstData_p;

    // forward command to Linux kernel module
    iRet = ioctl (EplApiInstance_g.m_hDrvInst, EPLLIN_CMD_READ_LOCAL_OBJECT, (unsigned long)&LocalObject);
    if (iRet < 0)
    {
        Ret = kEplNoResource;
    }
    else
    {
        Ret = (tEplKernel)iRet;
        *puiSize_p = LocalObject.m_uiSize;
    }

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
tEplLinLocalObject  LocalObject;
int             iRet;

    LocalObject.m_uiIndex = uiIndex_p;
    LocalObject.m_uiSubindex = uiSubindex_p;
    LocalObject.m_uiSize = uiSize_p;
    LocalObject.m_pData = pSrcData_p;

    // forward command to Linux kernel module
    iRet = ioctl (EplApiInstance_g.m_hDrvInst, EPLLIN_CMD_WRITE_LOCAL_OBJECT, (unsigned long)&LocalObject);
    if (iRet < 0)
    {
        Ret = kEplNoResource;
    }
    else
    {
        Ret = (tEplKernel)iRet;
    }

    return Ret;
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
// Parameters:  pSdoComConHdl_p         = INOUT: pointer to SDO connection handle (may be NULL)
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
tEplKernel          Ret = kEplSuccessful;
tEplLinSdoObject    SdoObject;
int                 iRet;

    SdoObject.m_uiIndex = uiIndex_p;
    SdoObject.m_uiSubindex = uiSubindex_p;
    if (puiSize_p == NULL)
    {
        Ret = kEplApiInvalidParam;
        goto Exit;
    }
    SdoObject.m_uiSize = *puiSize_p;
    SdoObject.m_le_pData = pDstData_le_p;
    SdoObject.m_uiNodeId = uiNodeId_p;
    SdoObject.m_pUserArg = pUserArg_p;
    SdoObject.m_SdoType = SdoType_p;
    if (pSdoComConHdl_p == NULL)
    {
        SdoObject.m_fValidSdoComConHdl = FALSE;
    }
    else
    {
        SdoObject.m_fValidSdoComConHdl = TRUE;
        SdoObject.m_SdoComConHdl = *pSdoComConHdl_p;
    }

    // forward command to Linux kernel module
    iRet = ioctl (EplApiInstance_g.m_hDrvInst, EPLLIN_CMD_READ_OBJECT, (unsigned long)&SdoObject);
    if (iRet < 0)
    {
        Ret = kEplNoResource;
    }
    else
    {
        Ret = (tEplKernel)iRet;
        *puiSize_p = SdoObject.m_uiSize;
        if (pSdoComConHdl_p != NULL)
        {
            *pSdoComConHdl_p = SdoObject.m_SdoComConHdl;
        }
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
// Parameters:  pSdoComConHdl_p         = INOUT: pointer to SDO connection handle (may be NULL)
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
tEplKernel          Ret = kEplSuccessful;
tEplLinSdoObject    SdoObject;
int                 iRet;

    SdoObject.m_uiIndex = uiIndex_p;
    SdoObject.m_uiSubindex = uiSubindex_p;
    SdoObject.m_uiSize = uiSize_p;
    SdoObject.m_le_pData = pSrcData_le_p;
    SdoObject.m_uiNodeId = uiNodeId_p;
    SdoObject.m_pUserArg = pUserArg_p;
    SdoObject.m_SdoType = SdoType_p;
    if (pSdoComConHdl_p == NULL)
    {
        SdoObject.m_fValidSdoComConHdl = FALSE;
    }
    else
    {
        SdoObject.m_fValidSdoComConHdl = TRUE;
        SdoObject.m_SdoComConHdl = *pSdoComConHdl_p;
    }

    // forward command to Linux kernel module
    iRet = ioctl (EplApiInstance_g.m_hDrvInst, EPLLIN_CMD_WRITE_OBJECT, (unsigned long)&SdoObject);
    if (iRet < 0)
    {
        Ret = kEplNoResource;
    }
    else
    {
        Ret = (tEplKernel)iRet;
        if (pSdoComConHdl_p != NULL)
        {
            *pSdoComConHdl_p = SdoObject.m_SdoComConHdl;
        }
    }

    return Ret;
}


// ----------------------------------------------------------------------------
//
// Function:    EplApiFreeSdoChannel()
//
// Description: frees the specified SDO channel.
//              This function must be called after each call to EplApiReadObject()/EplApiWriteObject()
//              which returns kEplApiTaskDeferred and the application
//              is informed via the event callback function when the task is completed.
//
// Parameters:  SdoComConHdl_p          = IN: SDO connection handle
//
// Return:      tEplKernel              = error code
//
// ----------------------------------------------------------------------------

tEplKernel PUBLIC EplApiFreeSdoChannel(
            tEplSdoComConHdl SdoComConHdl_p)
{
tEplKernel      Ret = kEplSuccessful;
int             iRet;

    // forward SDO handle to Linux kernel module
    iRet = ioctl (EplApiInstance_g.m_hDrvInst, EPLLIN_CMD_FREE_SDO_CHANNEL, (unsigned long)SdoComConHdl_p);
    if (iRet < 0)
    {
        Ret = kEplNoResource;
    }
    else
    {
        Ret = (tEplKernel)iRet;
    }

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
tEplKernel      Ret = kEplSuccessful;
int             iRet;

    // forward user argument to Linux kernel module
    iRet = ioctl (EplApiInstance_g.m_hDrvInst, EPLLIN_CMD_POST_USER_EVENT, (unsigned long)pUserArg_p);
    if (iRet < 0)
    {
        Ret = kEplNoResource;
    }
    else
    {
        Ret = (tEplKernel)iRet;
    }

    return Ret;
}


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
                                             tEplNmtNodeCommand NodeCommand_p)
{
tEplKernel            Ret = kEplSuccessful;
tEplLinNodeCmdObject  NodeCmdObject;
int                   iRet;

    NodeCmdObject.m_uiNodeId    = uiNodeId_p;
    NodeCmdObject.m_NodeCommand = NodeCommand_p;

    // forward node command to Linux kernel module
    iRet = ioctl (EplApiInstance_g.m_hDrvInst,
                  EPLLIN_CMD_MN_TRIGGER_STATE_CHANGE, (unsigned long)&NodeCmdObject);
    if (iRet < 0)
    {
        Ret = kEplNoResource;
    }
    else
    {
        Ret = (tEplKernel)iRet;
    }

    return Ret;
}


//----------------------------------------------------------------------------
// Function:    EplApiProcess()
//
// Description: waits for events from the EPL stack and calls the event callback
//              function. It should be executed in a separate thread.
//
// Parameters:  (none)
//
// Returns:     tEplKernel              = error code
//
// State:
//----------------------------------------------------------------------------

tEplKernel PUBLIC EplApiProcess(void)
{
tEplKernel      Ret = kEplSuccessful;
int             iRet;

#if 0
int             hFdTracingEnabled;

    // open file from debugfs to disable tracing on interresting event
    hFdTracingEnabled = open("/sys/kernel/debug/tracing/tracing_enabled", O_WRONLY, 0666);
#endif

    EplApiInstance_g.m_Event.m_pEventArg = &EplApiInstance_g.m_EventArg;
    EplApiInstance_g.m_Event.m_pEventType = &EplApiInstance_g.m_EventType;
    EplApiInstance_g.m_Event.m_uiEventArgSize = sizeof (EplApiInstance_g.m_EventArg);
    EplApiInstance_g.m_Event.m_RetCbEvent = kEplSuccessful;

    while (1)
    {
        iRet = ioctl (EplApiInstance_g.m_hDrvInst, EPLLIN_CMD_GET_EVENT, (unsigned long)&EplApiInstance_g.m_Event);
        if (iRet != 0)
        {
            Ret = kEplShutdown;
            goto Exit;
        }

#if 0
        if ((EplApiInstance_g.m_Event.m_pEventArg->m_ErrHistoryEntry.m_wErrorCode == EPL_E_DLL_CYCLE_EXCEED)
            && (hFdTracingEnabled != -1))
        {
            lseek(hFdTracingEnabled, 0, SEEK_SET);
            write(hFdTracingEnabled, "0", 1);
        }
#endif

        // call user event callback function
        EplApiInstance_g.m_Event.m_RetCbEvent = EplApiInstance_g.m_InitParam.m_pfnCbEvent(EplApiInstance_g.m_EventType, &EplApiInstance_g.m_EventArg, EplApiInstance_g.m_InitParam.m_pEventUserArg);
    }

#if 0
    close(hFdTracingEnabled);
#endif

Exit:
    return Ret;
}


//---------------------------------------------------------------------------
//
// Function:    EplApiProcessImageAlloc()
//
// Description: allocates a dynamic process image
//
// Parameters:  uiSizeProcessImageIn_p  = size of input process image
//              uiSizeProcessImageOut_p = size of output process image
//
// Returns:     tEplKernel              = error code
//
// State:
//
//---------------------------------------------------------------------------

tEplKernel PUBLIC EplApiProcessImageAlloc(
    unsigned int uiSizeProcessImageIn_p,
    unsigned int uiSizeProcessImageOut_p,
    unsigned int uiQueueEntriesLo_p,
    unsigned int uiQueueEntriesHi_p)
{
tEplKernel      Ret = kEplSuccessful;
int             iRet;
tEplLinProcessImageAlloc    PIAlloc;

    PIAlloc.m_uiSizeProcessImageIn  = uiSizeProcessImageIn_p;
    PIAlloc.m_uiSizeProcessImageOut = uiSizeProcessImageOut_p;
    PIAlloc.m_uiQueueEntriesLo      = uiQueueEntriesLo_p;
    PIAlloc.m_uiQueueEntriesHi      = uiQueueEntriesHi_p;

    iRet = ioctl (EplApiInstance_g.m_hDrvInst, EPLLIN_CMD_PI_ALLOC, &PIAlloc);
    Ret = (tEplKernel)iRet;

    return Ret;
}


//---------------------------------------------------------------------------
//
// Function:    EplApiProcessImageFree()
//
// Description: frees the dynamic process image
//
// Parameters:  (none)
//
// Returns:     tEplKernel              = error code
//
// State:
//
//---------------------------------------------------------------------------

tEplKernel PUBLIC EplApiProcessImageFree(void)
{
tEplKernel      Ret = kEplSuccessful;
int             iRet;

    iRet = ioctl (EplApiInstance_g.m_hDrvInst, EPLLIN_CMD_PI_FREE, 0);
    Ret = (tEplKernel)iRet;

    return Ret;
}


//---------------------------------------------------------------------------
//
// Function:    EplApiProcessImageLinkObject()
//
// Description: link process variable from process image to object in OD
//
// Parameters:  uiObjIndex_p            = object index
//              uiFirstSubindex_p       = sub-index of object where first variable shall be linked to
//              uiOffsetPI_p            = offset of first process variable in process image
//              fOutputPI_p             = FALSE, input process image
//                                        TRUE, output process image
//              EntrySize_p             = size of one process variable
//              puiVarEntries_p         = [IN] number of process variables, which shall be linked to OD
//                                        [OUT] actual number of process variable, which were linked to OD
//
// Returns:     tEplKernel              = error code
//
// State:
//
//---------------------------------------------------------------------------

tEplKernel PUBLIC EplApiProcessImageLinkObject(
    unsigned int    uiObjIndex_p,
    unsigned int    uiFirstSubindex_p,
    unsigned int    uiOffsetPI_p,
    BOOL            fOutputPI_p,
    tEplObdSize     EntrySize_p,
    unsigned int*   puiVarEntries_p)
{
tEplKernel      Ret = kEplSuccessful;
int             iRet;
tEplLinProcessImageLinkObject   PILinkObject;

    PILinkObject.m_uiObjIndex       = uiObjIndex_p;
    PILinkObject.m_uiFirstSubindex  = uiFirstSubindex_p;
    PILinkObject.m_uiOffsetPI       = uiOffsetPI_p;
    PILinkObject.m_fOutputPI        = fOutputPI_p;
    PILinkObject.m_EntrySize        = EntrySize_p;
    PILinkObject.m_puiVarEntries    = puiVarEntries_p;

    iRet = ioctl (EplApiInstance_g.m_hDrvInst, EPLLIN_CMD_PI_LINKOBJECT, &PILinkObject);
    Ret = (tEplKernel)iRet;

    return Ret;
}


//---------------------------------------------------------------------------
//
// Function:    EplApiProcessImageExchange()
//
// Description: Perform a copy job.
//
// Parameters:  pCopyJob_p              = pointer to copy job structure
//
// Returns:     tEplKernel              = error code
//
// State:
//
//---------------------------------------------------------------------------

tEplKernel PUBLIC EplApiProcessImageExchange(
    tEplApiProcessImageCopyJob* pCopyJob_p)
{
tEplKernel      Ret = kEplSuccessful;
int             iRet;

    iRet = ioctl (EplApiInstance_g.m_hDrvInst, EPLLIN_CMD_PI_EXCHANGE, pCopyJob_p);
    Ret = (tEplKernel)iRet;

    return Ret;
}


#if (EPL_OBD_USE_LOAD_CONCISEDCF != FALSE)
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
tEplKernel          Ret = kEplSuccessful;
int                 iRet;
tEplLinCdcFilename  CdcFilename;

    CdcFilename.m_pszCdcFilename = pszCdcFilename_p;
    CdcFilename.m_uiFilenameSize = strlen(pszCdcFilename_p);

    iRet = ioctl (EplApiInstance_g.m_hDrvInst, EPLLIN_CMD_SET_CDC_FILENAME, &CdcFilename);
    Ret = (tEplKernel)iRet;

    return Ret;
}
#endif // (EPL_OBD_USE_LOAD_CONCISEDCF != FALSE)


#if 0
//---------------------------------------------------------------------------
//
// Function:    EplApiProcessImageSetup()
//
// Description: sets up a static process image
//              i.e. maps the process image to static OD entries
//              at 0x2000, 0x2001, 0x2010, 0x2011, 0x2020, 0x2021,
//                 0x2030, 0x2031, 0x2040, 0x2041, 0x2050, 0x2051.
//
// Parameters:  (none)
//
// Returns:     tEplKernel              = error code
//
// State:
//
//---------------------------------------------------------------------------

tEplKernel PUBLIC EplApiProcessImageSetup(void)
{
tEplKernel      Ret = kEplSuccessful;
int             iRet;

    iRet = ioctl (EplApiInstance_g.m_hDrvInst, EPLLIN_CMD_PI_SETUP, 0);
    Ret = (tEplKernel)iRet;

    return Ret;
}

//----------------------------------------------------------------------------
// Function:    EplApiProcessImageExchangeIn()
//
// Description: replaces passed input process image with the one of EPL stack
//
// Parameters:  pPI_p                   = input process image
//
// Returns:     tEplKernel              = error code
//
// State:
//----------------------------------------------------------------------------

tEplKernel PUBLIC EplApiProcessImageExchangeIn(tEplApiProcessImage* pPI_p)
{
tEplKernel      Ret = kEplSuccessful;
int             iRet;

    // fetch input process image from Linux kernel module
    iRet = ioctl (EplApiInstance_g.m_hDrvInst, EPLLIN_CMD_PI_IN, (unsigned long)pPI_p);
    if (iRet < 0)
    {
        Ret = kEplNoResource;
    }
    else
    {
        Ret = (tEplKernel)iRet;
    }

    return Ret;
}


//----------------------------------------------------------------------------
// Function:    EplApiProcessImageExchangeOut()
//
// Description: copies passed output process image to EPL stack and marks
//              TPDOs as valid.
//
// Parameters:  pPI_p                   = output process image
//
// Returns:     tEplKernel              = error code
//
// State:
//----------------------------------------------------------------------------

tEplKernel PUBLIC EplApiProcessImageExchangeOut(tEplApiProcessImage* pPI_p)
{
tEplKernel      Ret = kEplSuccessful;
int             iRet;

    // forward output process image to Linux kernel module
    iRet = ioctl (EplApiInstance_g.m_hDrvInst, EPLLIN_CMD_PI_OUT, (unsigned long)pPI_p);
    if (iRet < 0)
    {
        Ret = kEplNoResource;
    }
    else
    {
        Ret = (tEplKernel)iRet;
    }

    return Ret;
}
#endif

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
    // This function is currently not supported
    // in the Linux kernel space implementation
    *ppIdentResponse_p  = NULL;

    return  kEplInvalidOperation;
}

//=========================================================================//
//                                                                         //
//          P R I V A T E   F U N C T I O N S                              //
//                                                                         //
//=========================================================================//




// EOF

