/**
********************************************************************************
\file   main.c

\brief  Main file for Windows kernel PCIe interface module

This file contains the module initialization of the Windows kernel driver
implementation for PCIe device.

\ingroup module_driver_ndispcie
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2017, Kalycito Infotech Private Limited
Copyright (c) 2016, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
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
#include "drvintf.h"

#include <ndis.h>
#include <ntddk.h>

#include <ndis-intf.h>


//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#define PLK_MEM_TAG         'klpO'

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
/**
\brief  Instance for openPOWERLINK driver

The structure specifies the instance variable for the application interface device
*/
typedef struct
{
    NDIS_SPIN_LOCK      syncQueueLock;          ///< Synchronization queue lock
    LIST_ENTRY          syncQueueHead;          ///< Pointer to sync queue head entry
    BOOL                fSyncClean;             ///< Clean pending synchronization IOCTLs
} tPlkDeviceInstance;

/**
\brief  Instance for openPOWERLINK driver

The structure specifies the instance variable of the Windows kernel driver
*/
typedef struct
{
    PDEVICE_OBJECT      pDrvDeviceObject;       ///< IOCTL interface device object
    NDIS_HANDLE         pDrvDeviceHandle;       ///< IOCTL interface device handle
    NDIS_HANDLE         driverHandle;           ///< Miniport driver handle
    BOOL                fInitialized;           ///< Initialization status
    tPlkDeviceInstance* pDeviceInst;            ///< Pointer to IOCTL device instance
    UINT                instanceCount;          ///< Number of open instances
} tPlkDriverInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tPlkDriverInstance   plkDriverInstance_l;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
DRIVER_DISPATCH powerlinkCreate;
DRIVER_DISPATCH powerlinkCleanup;
DRIVER_DISPATCH powerlinkClose;
DRIVER_DISPATCH powerlinkIoctl;

static void registerDrvIntf(NDIS_HANDLE driverHandle_p);
static void deregisterDrvIntf(void);
static void increaseHeartbeatCb(void* unusedParameter1_p,
                                void* functionContext_p,
                                void* unusedParameter2_p,
                                void* unusedParameter3_p);
static void syncCleanUp(void);
static void syncInterruptHandler(void);

//------------------------------------------------------------------------------
//  Kernel module specific data structures
//------------------------------------------------------------------------------

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
//  Initialize driver
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
/**
\brief  Driver initialization routine

The function implements openPOWERLINK Windows kernel driver initialization callback.
OS calls this routine on driver registration.

\param[in]      driverObject_p      Pointer to the system's driver object structure
                                    for this driver.
\param[in]      registryPath_p      System's registry path for this driver.

\return This routine returns an NTSTATUS error code.
\retval STATUS_SUCCESS If no error occurs.

\ingroup module_driver_ndispcie
*/
//------------------------------------------------------------------------------
NTSTATUS DriverEntry(PDRIVER_OBJECT driverObject_p,
                     PUNICODE_STRING registryPath_p)
{
    NDIS_STATUS ndisStatus;

    DEBUG_LVL_ALWAYS_TRACE("PLK: + %s()\n", __func__);
    ndisStatus = ndis_initDriver(driverObject_p, registryPath_p);

    if (ndisStatus != NDIS_STATUS_SUCCESS)
    {
        DEBUG_LVL_ERROR_TRACE("%s() Failed to initialize driver 0x%X\n",
                              __func__,
                              ndisStatus);
        return ndisStatus;
    }

    // register application interface handlers
    ndis_registerDrvIntf(registerDrvIntf, deregisterDrvIntf);
    plkDriverInstance_l.fInitialized = FALSE;
    plkDriverInstance_l.instanceCount = 0;

    DEBUG_LVL_ALWAYS_TRACE("PLK: + %s() - OK\n", __func__);
    return ndisStatus;
}

//------------------------------------------------------------------------------
/**
\brief  openPOWERLINK driver create function

The function implements openPOWERLINK kernel module create callback. OS calls
this routine when an application tries to open a file interface to this driver
using CreateFile().

\param[in]      pDeviceObject_p     Pointer to device object allocated for the IOCTL device.
\param[in,out]  pIrp_p              Pointer to I/O request packet for this call.

\return This routine returns an NTSTATUS error code.
\retval STATUS_SUCCESS If no error occurs

\ingroup module_driver_ndispcie
*/
//------------------------------------------------------------------------------
NTSTATUS powerlinkCreate(PDEVICE_OBJECT pDeviceObject_p,
                         PIRP pIrp_p)
{
    NDIS_TIMER_CHARACTERISTICS  timerChars;
    tFileContext*               pFileContext;
    PIO_STACK_LOCATION          irpStack;
    NDIS_STATUS                 status;

    UNUSED_PARAMETER(pDeviceObject_p);

    DEBUG_LVL_ALWAYS_TRACE("PLK: + %s() ...\n", __func__);

    irpStack = IoGetCurrentIrpStackLocation(pIrp_p);

    pFileContext = ExAllocatePoolWithQuotaTag(NonPagedPool,
                                              sizeof(tFileContext),
                                              PLK_MEM_TAG);

    if (pFileContext == NULL)
    {
        DEBUG_LVL_ERROR_TRACE("PLK: Failed to create file context\n");
    }

    IoInitializeRemoveLock(&pFileContext->driverAccessLock,
                           PLK_MEM_TAG,
                           0,
                           0);

    irpStack->FileObject->FsContext = (void*)pFileContext;

    if (!plkDriverInstance_l.fInitialized)
    {
        drv_init();
        plkDriverInstance_l.fInitialized = TRUE;
    }

    plkDriverInstance_l.pDeviceInst->fSyncClean = TRUE;

    // Increase the count for open instances
    plkDriverInstance_l.instanceCount++;

    pIrp_p->IoStatus.Information = 0;
    pIrp_p->IoStatus.Status = STATUS_SUCCESS;
    IoCompleteRequest(pIrp_p, IO_NO_INCREMENT);

    DEBUG_LVL_ALWAYS_TRACE("PLK: + %s() - OK\n", __func__);

    return STATUS_SUCCESS;
}

//------------------------------------------------------------------------------
/**
\brief  openPOWERLINK driver clean-up function

The function implements the clean-up callback. OS calls this when an application
closes the file interface to the IOCTL device.

\param[in]      pDeviceObject_p     Pointer to device object allocated for the IOCTL device.
\param[in,out]  pIrp_p              Pointer to I/O request packet for this call.

\return This routine returns an NTSTATUS error code.
\retval STATUS_SUCCESS If no error occurs.

\ingroup module_driver_ndispcie
*/
//------------------------------------------------------------------------------
NTSTATUS powerlinkCleanup(PDEVICE_OBJECT pDeviceObject_p,
                          PIRP pIrp_p)
{
    UNUSED_PARAMETER(pDeviceObject_p);
    UNUSED_PARAMETER(pIrp_p);

    syncCleanUp();

    return STATUS_SUCCESS;
}

//------------------------------------------------------------------------------
/**
\brief  openPOWERLINK driver close function

The function implements openPOWERLINK kernel driver close callback. OS calls
this function when the user application calls CloseHandle() for the device.

\param[in]      pDeviceObject_p     Pointer to device object allocated for the IOCTL device.
\param[in,out]  pIrp_p              Pointer to I/O request packet for this call.

\return This routine returns an NTSTATUS error code.
\retval Always return STATUS_SUCCESS.

\ingroup module_driver_ndispcie
*/
//------------------------------------------------------------------------------
NTSTATUS powerlinkClose(PDEVICE_OBJECT pDeviceObject_p,
                        PIRP pIrp_p)
{
    tFileContext*       pFileContext;
    PIO_STACK_LOCATION  irpStack;
    UINT16              status;
    tCtrlCmd            ctrlCmd;

    UNUSED_PARAMETER(pDeviceObject_p);

    DEBUG_LVL_ALWAYS_TRACE("PLK: + %s()...\n", __func__);

    irpStack = IoGetCurrentIrpStackLocation(pIrp_p);
    pFileContext = irpStack->FileObject->FsContext;
    ExFreePoolWithTag(pFileContext, PLK_MEM_TAG);

    plkDriverInstance_l.instanceCount--;

    // Close lower driver resources only if all open instances have closed.
    if (plkDriverInstance_l.fInitialized &&
        (plkDriverInstance_l.instanceCount == 0))
    {
        plkDriverInstance_l.fInitialized = FALSE;
        drv_exit();
    }

    pIrp_p->IoStatus.Information = 0;
    pIrp_p->IoStatus.Status = STATUS_SUCCESS;
    IoCompleteRequest(pIrp_p, IO_NO_INCREMENT);

    DEBUG_LVL_ALWAYS_TRACE("PLK: + %s() - OK\n", __func__);

    return STATUS_SUCCESS;
}

//------------------------------------------------------------------------------
/**
\brief  openPOWERLINK driver IOCTL handler

The function implements IOCTL callback. OS calls this routine when the user
application calls DeviceIoControl() for the device.

\param[in]      pDeviceObject_p     Pointer to device object allocated for the IOCTL device.
\param[in,out]  pIrp_p              Pointer to I/O request packet for this call.

\return This routine returns an NTSTATUS error code.

\ingroup module_driver_ndispcie
*/
//------------------------------------------------------------------------------
NTSTATUS powerlinkIoctl(PDEVICE_OBJECT pDeviceObject_p,
                        PIRP pIrp_p)
{
    PIO_STACK_LOCATION  irpStack;
    NTSTATUS            status = STATUS_SUCCESS;
    ULONG               inlen, outlen;
    void*               pInBuffer;
    void*               pOutBuffer;
    tFileContext*       pFileContext;
    tOplkError          oplkRet;

    UNUSED_PARAMETER(pDeviceObject_p);

    irpStack = IoGetCurrentIrpStackLocation(pIrp_p);

    pFileContext = irpStack->FileObject->FsContext;

    // Acquire the IRP remove lock
    status = IoAcquireRemoveLock(&pFileContext->driverAccessLock, pIrp_p);

    if (!NT_SUCCESS(status))
    {
        // Lock is in a removed state. That means driver already received
        // clean up request for this handle.
        pIrp_p->IoStatus.Status = status;
        IoCompleteRequest(pIrp_p, IO_NO_INCREMENT);
        return status;
    }

    inlen = irpStack->Parameters.DeviceIoControl.InputBufferLength;
    outlen = irpStack->Parameters.DeviceIoControl.OutputBufferLength;

    switch (irpStack->Parameters.DeviceIoControl.IoControlCode)
    {
        case PLK_CMD_CTRL_EXECUTE_CMD:
        {
            tCtrlCmd*   pCtrlCmd = (tCtrlCmd*)pIrp_p->AssociatedIrp.SystemBuffer;

            oplkRet = drv_executeCmd(pCtrlCmd);
            if (oplkRet != kErrorOk)
                pCtrlCmd->retVal = (UINT16)oplkRet;

            pIrp_p->IoStatus.Information = sizeof(tCtrlCmd);

            break;
        }

        case PLK_CMD_CTRL_STORE_INITPARAM:
        {
            tCtrlInitParam* pCtrlInitCmd = (tCtrlInitParam*)pIrp_p->AssociatedIrp.SystemBuffer;

            oplkRet = drv_storeInitParam(pCtrlInitCmd);
            if (oplkRet != kErrorOk)
                pIrp_p->IoStatus.Information = 0;
            else
                pIrp_p->IoStatus.Information = sizeof(tOplkError);

            break;
        }

        case PLK_CMD_CTRL_READ_INITPARAM:
        {
            tCtrlInitParam* pCtrlInitCmd = (tCtrlInitParam*)pIrp_p->AssociatedIrp.SystemBuffer;

            oplkRet = drv_readInitParam(pCtrlInitCmd);
            if (oplkRet != kErrorOk)
                pIrp_p->IoStatus.Information = 0;
            else
                pIrp_p->IoStatus.Information = sizeof(tCtrlInitParam);

            break;
        }

        case PLK_CMD_CTRL_GET_STATUS:
        {
            UINT16* pStatus = (UINT16*)pIrp_p->AssociatedIrp.SystemBuffer;

            oplkRet = drv_getStatus(pStatus);
            if (oplkRet != kErrorOk)
                pIrp_p->IoStatus.Information = 0;
            else
                pIrp_p->IoStatus.Information = sizeof(UINT16);

            break;
        }

        case PLK_CMD_CTRL_GET_HEARTBEAT:
        {
            UINT16* pHeartBeat = (UINT16*)pIrp_p->AssociatedIrp.SystemBuffer;

            oplkRet = drv_getHeartbeat(pHeartBeat);
            if (oplkRet != kErrorOk)
                pIrp_p->IoStatus.Information = 0;
            else
                pIrp_p->IoStatus.Information = sizeof(UINT16);

            break;
        }

        case PLK_CMD_POST_EVENT:
        {
            tEvent* pEvent = (tEvent*)pIrp_p->AssociatedIrp.SystemBuffer;

            if (pEvent->eventArgSize != 0)
                pEvent->eventArg.pEventArg = (void*)((UINT8*)pEvent + sizeof(tEvent));
            else
                pEvent->eventArg.pEventArg = NULL;

            oplkRet = drv_postEvent(pEvent);
            if (oplkRet != kErrorOk)
                pIrp_p->IoStatus.Information = 0;
            else
                pIrp_p->IoStatus.Information = sizeof(tOplkError);

            break;
        }

        case PLK_CMD_GET_EVENT:
        {
            size_t  eventSize = 0;

            pOutBuffer = pIrp_p->AssociatedIrp.SystemBuffer;

            oplkRet = drv_getEvent(pOutBuffer, &eventSize);
            if (pIrp_p->Cancel ||
                (oplkRet != kErrorOk))
                pIrp_p->IoStatus.Information = 0;
            else
                pIrp_p->IoStatus.Information = eventSize;

            break;
        }

        case PLK_CMD_DLLCAL_ASYNCSEND:
        {
            pInBuffer = pIrp_p->AssociatedIrp.SystemBuffer;

            oplkRet = drv_sendAsyncFrame(pInBuffer);
            if (oplkRet != kErrorOk)
                pIrp_p->IoStatus.Information = 0;
            else
                pIrp_p->IoStatus.Information = sizeof(tOplkError);

            break;
        }

        case PLK_CMD_ERRHND_WRITE:
        {
            tErrHndIoctl*   pWriteObject = (tErrHndIoctl*)pIrp_p->AssociatedIrp.SystemBuffer;

            oplkRet = drv_writeErrorObject(pWriteObject);
            if (oplkRet != kErrorOk)
                pIrp_p->IoStatus.Information = 0;
            else
                pIrp_p->IoStatus.Information = sizeof(tOplkError);

            break;
        }

        case PLK_CMD_ERRHND_READ:
        {
            tErrHndIoctl*   pReadObject = (tErrHndIoctl*)pIrp_p->AssociatedIrp.SystemBuffer;

            oplkRet = drv_readErrorObject(pReadObject);
            if (oplkRet != kErrorOk)
                pIrp_p->IoStatus.Information = 0;
            else
                pIrp_p->IoStatus.Information = sizeof(tErrHndIoctl);

            break;
        }

        case PLK_CMD_TIMESYNC_SYNC:
        {
            if (!pIrp_p->Cancel)
            {
                IoMarkIrpPending(pIrp_p);
                NdisInterlockedInsertTailList(&plkDriverInstance_l.pDeviceInst->syncQueueHead,
                                              &pIrp_p->Tail.Overlay.ListEntry,
                                              &plkDriverInstance_l.pDeviceInst->syncQueueLock);
            }
            else
            {
                pIrp_p->IoStatus.Information = 0;
                status = STATUS_CANCELLED;
                break;
            }

            status = STATUS_PENDING;
            break;
        }

        case PLK_CMD_PDO_GET_MEM:
        {
            tPdoMem*    pPdoMem = (tPdoMem*)pIrp_p->AssociatedIrp.SystemBuffer;

            oplkRet = drv_getPdoMem(&pPdoMem->pdoMemOffset, pPdoMem->memSize);
            if (oplkRet != kErrorOk)
            {
                // return size zero to indicate failure
                pIrp_p->IoStatus.Information = 0;
            }
            else
                pIrp_p->IoStatus.Information = sizeof(tPdoMem);

            status = STATUS_SUCCESS;
            break;
        }

        case PLK_CMD_CLEAN:
        {
            syncCleanUp();
            break;
        }

        case PLK_GET_BENCHMARK_BASE:
        {
            tBenchmarkMem*  pBenchmarkMem = (tBenchmarkMem*)pIrp_p->AssociatedIrp.SystemBuffer;

            oplkRet = drv_getBenchmarkMem(&pBenchmarkMem->pBaseAddr);
            if (oplkRet != kErrorOk)
                pIrp_p->IoStatus.Information = 0;
            else
                pIrp_p->IoStatus.Information = sizeof(tBenchmarkMem);

            status = STATUS_SUCCESS;
            break;
        }

        case PLK_FREE_BENCHMARK_BASE:
        {
            tBenchmarkMem*  pBenchmarkMem = (tBenchmarkMem*)pIrp_p->AssociatedIrp.SystemBuffer;

            drv_freeBenchmarkMem(pBenchmarkMem->pBaseAddr);
            status = STATUS_SUCCESS;
            pIrp_p->IoStatus.Information = 0;
            break;
        }

        case PLK_CMD_MAP_MEM:
        {
            tMemStruc*  pMemStruc = (tMemStruc*)pIrp_p->AssociatedIrp.SystemBuffer;

            oplkRet = drv_mapKernelMem(&pMemStruc->pKernelAddr,
                                       &pMemStruc->pUserAddr,
                                       &pMemStruc->size);

            if (oplkRet != kErrorOk)
            {
                // Error occurred, return size 0.
                pIrp_p->IoStatus.Information = 0;
            }
            else
            {
                pIrp_p->IoStatus.Information = sizeof(tMemStruc);
            }

            // complete the IOCTL. If error occurred the return parameter size will be zero.
            status = STATUS_SUCCESS;
            break;

        }

        case PLK_CMD_UNMAP_MEM:
        {
            tMemStruc*  pMemStruc = (tMemStruc*)pIrp_p->AssociatedIrp.SystemBuffer;

            drv_unmapKernelMem(pMemStruc->pUserAddr);
            status = STATUS_SUCCESS;
            pIrp_p->IoStatus.Information = 0;
            break;
        }

        case PLK_CMD_CTRL_WRITE_FILE_BUFFER:
        {
            tIoctlFileChunk*    pFileChunk = (tIoctlFileChunk*)pIrp_p->AssociatedIrp.SystemBuffer;

            oplkRet = drv_writeFileBuffer(pFileChunk);
            if (oplkRet != kErrorOk)
                pIrp_p->IoStatus.Information = 0;
            else
                pIrp_p->IoStatus.Information = sizeof(tIoctlFileChunk);

            status = STATUS_SUCCESS;
            break;
        }

        case PLK_CMD_CTRL_GET_FILE_BUFFER_SIZE:
        {
            size_t* pFileBufferSize = (size_t*)pIrp_p->AssociatedIrp.SystemBuffer;

            *pFileBufferSize = drv_getFileBufferSize();
            pIrp_p->IoStatus.Information = sizeof(size_t);
            status = STATUS_SUCCESS;
            break;
        }

#if defined(CONFIG_INCLUDE_SOC_TIME_FORWARD)
       case PLK_CMD_SOC_GET_MEM:
        {
            tSocMem* pSocMem = (tSocMem*)pIrp_p->AssociatedIrp.SystemBuffer;

            oplkRet = drv_getTimesyncMem((void*)&pSocMem->socMemOffset, pSocMem->socMemSize);
            if (oplkRet != kErrorOk)
                pIrp_p->IoStatus.Information = 0; // return size zero to indicate failure
            else
                pIrp_p->IoStatus.Information = sizeof(tSocMem);

            status = STATUS_SUCCESS;
            break;
        }
#endif

        default:
            DEBUG_LVL_ERROR_TRACE("PLK: - Invalid cmd (cmd=%d)\n",
                                  irpStack->Parameters.DeviceIoControl.IoControlCode);
            break;
    }

    if (status != STATUS_PENDING)
    {
        // complete the Irp if its not pending
        pIrp_p->IoStatus.Status = status;
        IoCompleteRequest(pIrp_p, IO_NO_INCREMENT);
    }

    // Release lock
    IoReleaseRemoveLock(&pFileContext->driverAccessLock, pIrp_p);

    return status;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief Register application interface device

This routine is called from the miniport initialize to register an IOCTL
interface for the driver. A user application can use this interfaces to
communicate with this driver.

A device object to be used for this purpose is created by NDIS when
NdisMRegisterDevice is called.

This routine is called whenever a new miniport instance is initialized.
However, only one global device object is created, when the first miniport
instance is initialized.

\param[in]      driverHandle_p      Miniport driver handle returned by OS on registration

*/
//------------------------------------------------------------------------------
static void registerDrvIntf(NDIS_HANDLE driverHandle_p)
{
    NDIS_STATUS                     status = NDIS_STATUS_SUCCESS;
    UNICODE_STRING                  deviceName;
    UNICODE_STRING                  deviceLinkUnicodeString;
    NDIS_DEVICE_OBJECT_ATTRIBUTES   deviceObjectAttributes;
    PDRIVER_DISPATCH                dispatchTable[IRP_MJ_MAXIMUM_FUNCTION + 1];

    DEBUG_LVL_ALWAYS_TRACE("PLK %s()...\n", __func__);
    plkDriverInstance_l.driverHandle = driverHandle_p;

    NdisZeroMemory(dispatchTable, (IRP_MJ_MAXIMUM_FUNCTION + 1) * sizeof(PDRIVER_DISPATCH));
    dispatchTable[IRP_MJ_CREATE] = powerlinkCreate;
    dispatchTable[IRP_MJ_CLEANUP] = powerlinkCleanup;
    dispatchTable[IRP_MJ_CLOSE] = powerlinkClose;
    dispatchTable[IRP_MJ_DEVICE_CONTROL] = powerlinkIoctl;

    NdisInitUnicodeString(&deviceName, PLK_DEV_STRING);
    NdisInitUnicodeString(&deviceLinkUnicodeString, PLK_LINK_NAME);

    NdisZeroMemory(&deviceObjectAttributes, sizeof(NDIS_DEVICE_OBJECT_ATTRIBUTES));
    // type implicit from the context
    deviceObjectAttributes.Header.Type = NDIS_OBJECT_TYPE_DEVICE_OBJECT_ATTRIBUTES;
    deviceObjectAttributes.Header.Revision = NDIS_DEVICE_OBJECT_ATTRIBUTES_REVISION_1;
    deviceObjectAttributes.Header.Size = sizeof(NDIS_DEVICE_OBJECT_ATTRIBUTES);
    deviceObjectAttributes.DeviceName = &deviceName;
    deviceObjectAttributes.SymbolicName = &deviceLinkUnicodeString;
    deviceObjectAttributes.MajorFunctions = &dispatchTable[0];
    deviceObjectAttributes.ExtensionSize = sizeof(tPlkDeviceInstance);

    status = NdisRegisterDeviceEx(driverHandle_p,
                                  &deviceObjectAttributes,
                                  &plkDriverInstance_l.pDrvDeviceObject,
                                  &plkDriverInstance_l.pDrvDeviceHandle);
    if (status != NDIS_STATUS_SUCCESS)
    {
        DEBUG_LVL_ERROR_TRACE("%s() Failed to register interface device (0x%X)\n", status);
        return;
    }

    plkDriverInstance_l.pDrvDeviceObject->Flags |= DO_BUFFERED_IO;
    plkDriverInstance_l.pDeviceInst = (tPlkDeviceInstance*)NdisGetDeviceReservedExtension(plkDriverInstance_l.pDrvDeviceObject);
    if (plkDriverInstance_l.pDeviceInst == NULL)
    {
        DEBUG_LVL_ERROR_TRACE("%s() Unable to retrieve device extension\n", __func__);
        return;
    }

    NdisInitializeListHead(&plkDriverInstance_l.pDeviceInst->syncQueueHead);
    NdisAllocateSpinLock(&plkDriverInstance_l.pDeviceInst->syncQueueLock);
    plkDriverInstance_l.pDeviceInst->fSyncClean = TRUE;

    ndis_registerIntrHandler(syncInterruptHandler);

    DEBUG_LVL_ALWAYS_TRACE("PLK %s() - OK\n", __func__);
}

//------------------------------------------------------------------------------
/**
\brief De-register application interface device

De-register the IOCTL interface registered during initialization,

*/
//------------------------------------------------------------------------------
static void deregisterDrvIntf(void)
{
    ndis_registerIntrHandler(NULL);

    // Complete all pending IOCTLs
    syncCleanUp();

    NdisFreeSpinLock(&plkDriverInstance_l.pDeviceInst->syncQueueLock);

    if (plkDriverInstance_l.pDrvDeviceHandle != NULL)
        NdisDeregisterDeviceEx(plkDriverInstance_l.pDrvDeviceHandle);
}

//------------------------------------------------------------------------------
/**
\brief Synchronization interrupt handler

This is the interrupt routine for synchronization interrupt from PCIe.

The user layer of the stack sends an IOCTL to the driver to check the presence
of new data. These IOCTLs are marked as pending by the driver if the data is not
ready and queued to be handled in this interrupt handler. Once the IOCTL is
marked pending, the user application waits for the IOCTL request to be completed.

On an interrupt a single IOCTL request is dequeued and completed to indicate
presence of new data from openPOWERLINK kernel layer. The user application
then wakes up and processes the new data.

*/
//------------------------------------------------------------------------------
static void syncInterruptHandler(void)
{
    PLIST_ENTRY         pSyncListEntry = NULL;
    PIRP                pIrp = NULL;
    NTSTATUS            status;
    PIO_STACK_LOCATION  pIrpStack;

    if (IsListEmpty(&plkDriverInstance_l.pDeviceInst->syncQueueHead) ||
        (&plkDriverInstance_l.pDeviceInst->syncQueueHead == NULL))
        return;

    pSyncListEntry = NdisInterlockedRemoveHeadList(&plkDriverInstance_l.pDeviceInst->syncQueueHead,
                                                   &plkDriverInstance_l.pDeviceInst->syncQueueLock);
    pIrp = CONTAINING_RECORD(pSyncListEntry, IRP, Tail.Overlay.ListEntry);

    if (pIrp->Cancel)
        status = STATUS_CANCELLED;
    else
        status = STATUS_SUCCESS;

    pIrp->IoStatus.Status = status;
    pIrp->IoStatus.Information = 0;
    IoCompleteRequest(pIrp, IO_NO_INCREMENT);
}

//------------------------------------------------------------------------------
/**
\brief Clean up pending IOCTL

This function cleans all the pending IOCTL from user application and completes
them with CANCELLED status.

*/
//------------------------------------------------------------------------------
static void syncCleanUp(void)
{
    PIRP        pIrp;
    PLIST_ENTRY pListEntry;

    if ((&plkDriverInstance_l.pDeviceInst->syncQueueHead != NULL) &&
        plkDriverInstance_l.pDeviceInst->fSyncClean)
    {
        while (!IsListEmpty(&plkDriverInstance_l.pDeviceInst->syncQueueHead))
        {
            pListEntry = NdisInterlockedRemoveHeadList(&plkDriverInstance_l.pDeviceInst->syncQueueHead,
                                                       &plkDriverInstance_l.pDeviceInst->syncQueueLock);
            pIrp = CONTAINING_RECORD(pListEntry, IRP, Tail.Overlay.ListEntry);

            if (pIrp != NULL)
            {
                pIrp->IoStatus.Status = STATUS_CANCELLED;
                pIrp->IoStatus.Information = 0;
                IoCompleteRequest(pIrp, IO_NO_INCREMENT);
            }
        }
    }
    plkDriverInstance_l.pDeviceInst->fSyncClean = FALSE;
}

/// \}
