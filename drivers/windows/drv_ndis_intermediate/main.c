/**
********************************************************************************
\file   main.c

\brief  Main file for Windows kernel module

This file contains the main part of the Windows kernel module implementation of
the openPOWERLINK kernel stack daemon.

\ingroup module_driver_ndisim
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2015, Kalycito Infotech Private Limited
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
#include <ndis.h>
#include <ntddk.h>

#include <oplk/oplk.h>
#include <kernel/eventk.h>
#include <kernel/eventkcal.h>
#include <kernel/timesynckcal.h>
#include <errhndkcal.h>

#include <ndisintermediate/ndis-im.h>
#include <drvintf.h>

//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#define PLK_MEM_TAG       'klpO'

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
\brief  Instance for POWERLINK driver

The structure specifies the instance variable of the Windows kernel driver
*/
typedef struct
{
    PDEVICE_OBJECT        pDrvDeviceObject;         ///< IOCTL interface device object
    NDIS_HANDLE           pDrvDeviceHandle;         ///< IOCTL interface device handle
    NDIS_HANDLE           driverHandle;             ///< Miniport driver handle
    BOOL                  fInitialized;             ///< Initialization status
    UINT                  instanceCount;            ///< Number of open instances
}tPlkDriverInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tPlkDriverInstance    plkDriverInstance_l;
static NDIS_HANDLE           heartbeatTimer_l;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
DRIVER_DISPATCH       powerlinkCreate;
DRIVER_DISPATCH       powerlinkCleanup;
DRIVER_DISPATCH       powerlinkClose;
DRIVER_DISPATCH       powerlinkIoctl;

static void registerDrvIntf(NDIS_HANDLE driverHandle_p);
static void deregisterDrvIntf(void);
static void increaseHeartbeatCb(void* pSystemParam1_p, void* pFunctionContext_p,
                                void* pSystemParam2_p, void* pSystemParam3_p);
static void startHeartbeatTimer(LONG timeInMs_p);
static void stopHeartbeatTimer(void);

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

\param  driverObject_p       Pointer to the system's driver object structure
                             for this driver.
\param  registryPath_p       System's registry path for this driver.

\return This routine returns an NTSTATUS error code.
\retval STATUS_SUCCESS If no error occurs.

\ingroup module_driver_ndisim
*/
//------------------------------------------------------------------------------
NTSTATUS DriverEntry(PDRIVER_OBJECT driverObject_p, PUNICODE_STRING registryPath_p)
{
    NDIS_STATUS    ndisStatus = NDIS_STATUS_SUCCESS;

    DEBUG_LVL_ALWAYS_TRACE("PLK: + Driver Entry\n");
    ndisStatus = ndis_initDriver(driverObject_p, registryPath_p);

    if (ndisStatus != NDIS_STATUS_SUCCESS)
    {
        DEBUG_LVL_ERROR_TRACE("%s() Failed to initialize driver 0x%X\n", __FUNCTION__,
                              ndisStatus);
        return ndisStatus;
    }

    // register driver interface handlers
    ndis_registerDrvIntf(registerDrvIntf, deregisterDrvIntf);
    plkDriverInstance_l.fInitialized = FALSE;
    plkDriverInstance_l.instanceCount = 0;

    DEBUG_LVL_ALWAYS_TRACE("PLK: + Driver Entry - OK\n");
    return ndisStatus;
}

//------------------------------------------------------------------------------
/**
\brief  openPOWERLINK driver create function

The function implements openPOWERLINK kernel module create callback. OS calls
this routine when an application tries to open a file interface to this driver
using CreateFile().

\param  pDeviceObject_p     Pointer to device object allocated for the IOCTL device.
\param  pIrp_p              Pointer to I/O request packet for this call.

\return This routine returns an NTSTATUS error code.
\retval STATUS_SUCCESS If no error occurs

\ingroup module_driver_ndispcie
*/
//------------------------------------------------------------------------------
NTSTATUS powerlinkCreate(PDEVICE_OBJECT pDeviceObject_p, PIRP pIrp_p)
{
    NDIS_TIMER_CHARACTERISTICS    timerChars;
    tFileContext*                 pFileContext;
    PIO_STACK_LOCATION            irpStack;
    NDIS_STATUS                   status;

    UNREFERENCED_PARAMETER(pDeviceObject_p);

    DEBUG_LVL_ALWAYS_TRACE("PLK: + powerlinkCreate ...\n");

    if (pIrp_p == NULL)
        return NDIS_STATUS_RESOURCES;

    irpStack = IoGetCurrentIrpStackLocation(pIrp_p);

    pFileContext = ExAllocatePoolWithQuotaTag(NonPagedPool, sizeof(tFileContext),
                                              PLK_MEM_TAG);

    if (pFileContext == NULL)
    {
        DEBUG_LVL_ERROR_TRACE("PLK: Failed to create file context\n");
    }

    IoInitializeRemoveLock(&pFileContext->driverAccessLock, PLK_MEM_TAG, 0, 0);

    irpStack->FileObject->FsContext = (void*)pFileContext;

    if (!plkDriverInstance_l.fInitialized)
    {
        NdisZeroMemory(&timerChars, sizeof(timerChars));

        C_ASSERT(NDIS_SIZEOF_TIMER_CHARACTERISTICS_REVISION_1 <= sizeof(timerChars));
        timerChars.Header.Type = NDIS_OBJECT_TYPE_TIMER_CHARACTERISTICS;
        timerChars.Header.Size = NDIS_SIZEOF_TIMER_CHARACTERISTICS_REVISION_1;
        timerChars.Header.Revision = NDIS_TIMER_CHARACTERISTICS_REVISION_1;

        timerChars.TimerFunction = increaseHeartbeatCb;
        timerChars.FunctionContext = NULL;
        timerChars.AllocationTag = PLK_MEM_TAG;

        status = NdisAllocateTimerObject(plkDriverInstance_l.driverHandle,
                                         &timerChars,
                                         &heartbeatTimer_l);
        if (status != NDIS_STATUS_SUCCESS)
        {
            DEBUG_LVL_ERROR_TRACE("%s() Timer Creation Failed %x\n", __func__, status);
            return STATUS_SUCCESS;
        }

        if (ctrlk_init(NULL) != kErrorOk)
        {
            return NDIS_STATUS_RESOURCES;
        }

        startHeartbeatTimer(20);
        plkDriverInstance_l.fInitialized = TRUE;
    }

    // Increase the count for open instances
    plkDriverInstance_l.instanceCount++;

    pIrp_p->IoStatus.Information = 0;
    pIrp_p->IoStatus.Status = STATUS_SUCCESS;
    IoCompleteRequest(pIrp_p, IO_NO_INCREMENT);

    DEBUG_LVL_ALWAYS_TRACE("PLK: + powerlinkCreate - OK\n");

    return STATUS_SUCCESS;
}

//------------------------------------------------------------------------------
/**
\brief  openPOWERLINK driver clean-up function

The function implements the clean-up callback. OS calls this when an application
closes the file interface to the IOCTL device.

\param  pDeviceObject_p     Pointer to device object allocated for the IOCTL device.
\param  pIrp_p              Pointer to I/O request packet for this call.

\return This routine returns an NTSTATUS error code.
\retval STATUS_SUCCESS If no error occurs.

\ingroup module_driver_ndispcie
*/
//------------------------------------------------------------------------------
NTSTATUS powerlinkCleanup(PDEVICE_OBJECT pDeviceObject_p, PIRP pIrp_p)
{
    UNUSED_PARAMETER(pDeviceObject_p);
    UNUSED_PARAMETER(pIrp_p);
    return STATUS_SUCCESS;
}

//------------------------------------------------------------------------------
/**
\brief  openPOWERLINK driver close function

The function implements openPOWERLINK kernel driver close callback. OS calls
this function when the user application calls CloseHandle() for the device.

\param  pDeviceObject_p     Pointer to device object allocated for the IOCTL device.
\param  pIrp_p              Pointer to I/O request packet for this call.

\return This routine returns an NTSTATUS error code.
\retval Always return STATUS_SUCCESS.

\ingroup module_driver_ndispcie
*/
//------------------------------------------------------------------------------
NTSTATUS powerlinkClose(PDEVICE_OBJECT pDeviceObject_p, PIRP pIrp_p)
{
    tFileContext*         pFileContext;
    PIO_STACK_LOCATION    irpStack;
    UINT16                status;
    tCtrlCmd              ctrlCmd;

    UNUSED_PARAMETER(pDeviceObject_p);

    DEBUG_LVL_ALWAYS_TRACE("PLK: + powerlinkClose...\n");

    if (pIrp_p == NULL)
        return NDIS_STATUS_RESOURCES;

    irpStack = IoGetCurrentIrpStackLocation(pIrp_p);

    pFileContext = irpStack->FileObject->FsContext;
    ExFreePoolWithTag(pFileContext, PLK_MEM_TAG);

    plkDriverInstance_l.instanceCount--;

    // Close lower driver resources only if all open instances have closed.
    if (plkDriverInstance_l.fInitialized && (plkDriverInstance_l.instanceCount == 0))
    {
        plkDriverInstance_l.fInitialized = FALSE;
        stopHeartbeatTimer();

        ctrlk_exit();

        drv_getStatus(&status);

        if (status == kCtrlStatusRunning)
        {
            ctrlCmd.cmd = kCtrlShutdown;
            drv_executeCmd(&ctrlCmd);
        }
    }

    pIrp_p->IoStatus.Information = 0;
    pIrp_p->IoStatus.Status = STATUS_SUCCESS;
    IoCompleteRequest(pIrp_p, IO_NO_INCREMENT);

    DEBUG_LVL_ALWAYS_TRACE("PLK: + powerlinkClose - OK\n");

    return STATUS_SUCCESS;
}

//------------------------------------------------------------------------------
/**
\brief  openPOWERLINK driver IOCTL handler

The function implements IOCTL callback. OS calls this routine when the user
application calls DeviceIoControl() for the device.

\param  pDeviceObject_p     Pointer to device object allocated for the IOCTL device.
\param  pIrp_p              Pointer to I/O request packet for this call.

\return This routine returns an NTSTATUS error code.

\ingroup module_driver_ndispcie
*/
//------------------------------------------------------------------------------
NTSTATUS powerlinkIoctl(PDEVICE_OBJECT pDeviceObject_p, PIRP pIrp_p)
{
    PIO_STACK_LOCATION    irpStack;
    NTSTATUS              status = STATUS_SUCCESS;
    ULONG                 inlen, outlen;
    void*                 pInBuffer;
    void*                 pOutBuffer;
    tFileContext*         pFileContext;
    tOplkError            oplkRet;

    UNREFERENCED_PARAMETER(pDeviceObject_p);

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
            tCtrlInitParam*   pCtrlInitCmd = (tCtrlInitParam*)pIrp_p->AssociatedIrp.SystemBuffer;

            oplkRet = drv_storeInitParam(pCtrlInitCmd);
            if (oplkRet != kErrorOk)
                pIrp_p->IoStatus.Information = 0;
            else
                pIrp_p->IoStatus.Information = sizeof(tOplkError);

            break;
        }

        case PLK_CMD_CTRL_READ_INITPARAM:
        {
            tCtrlInitParam*   pCtrlInitCmd = (tCtrlInitParam*)pIrp_p->AssociatedIrp.SystemBuffer;

            oplkRet = drv_readInitParam(pCtrlInitCmd);
            if (oplkRet != kErrorOk)
                pIrp_p->IoStatus.Information = 0;
            else
                pIrp_p->IoStatus.Information = sizeof(tCtrlInitParam);

            break;
        }

        case PLK_CMD_CTRL_GET_STATUS:
        {
            UINT16*   pStatus = (UINT16*)pIrp_p->AssociatedIrp.SystemBuffer;

            oplkRet = drv_getStatus(pStatus);
            if (oplkRet != kErrorOk)
                pIrp_p->IoStatus.Information = 0;
            else
                pIrp_p->IoStatus.Information = sizeof(UINT16);

            break;
        }

        case PLK_CMD_CTRL_GET_HEARTBEAT:
        {
            UINT16*   pHeartBeat = (UINT16*)pIrp_p->AssociatedIrp.SystemBuffer;

            oplkRet = drv_getHeartbeat(pHeartBeat);
            if (oplkRet != kErrorOk)
                pIrp_p->IoStatus.Information = 0;
            else
                pIrp_p->IoStatus.Information = sizeof(UINT16);

            break;
        }

        case PLK_CMD_POST_EVENT:
        {
            pInBuffer = pIrp_p->AssociatedIrp.SystemBuffer;
            eventkcal_postEventFromUser(pInBuffer);

            pIrp_p->IoStatus.Information = sizeof(oplkRet);

            status = STATUS_SUCCESS;
            break;
        }

        case PLK_CMD_GET_EVENT:
        {
            size_t    eventSize = 0;
            pOutBuffer = pIrp_p->AssociatedIrp.SystemBuffer;
            eventkcal_getEventForUser(pOutBuffer, &eventSize);

            if (!pIrp_p->Cancel)
                pIrp_p->IoStatus.Information = eventSize;
            else
                pIrp_p->IoStatus.Information = 0;

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
            oplkRet = timesynckcal_waitSyncEvent();
            if (oplkRet != kErrorRetry && oplkRet != kErrorOk)
                status = STATUS_INSUFFICIENT_RESOURCES;
            else
                status = STATUS_SUCCESS;
            break;
        }

        case PLK_CMD_PDO_GET_MEM:
        {
            tPdoMem*   pPdoMem = (tPdoMem*)pIrp_p->AssociatedIrp.SystemBuffer;
            pPdoMem->pdoMemOffset = 0;
            pIrp_p->IoStatus.Information = sizeof(tPdoMem);
            status = STATUS_SUCCESS;
            break;
        }

        case PLK_CMD_CLEAN:
        {
            status = STATUS_SUCCESS;
            break;
        }

        case PLK_CMD_MAP_MEM:
        {
            tMemStruc*   pMemStruc = (tMemStruc*)pIrp_p->AssociatedIrp.SystemBuffer;
            oplkRet = drv_mapPdoMem((UINT8**)&pMemStruc->pKernelAddr,
                                    (UINT8**)&pMemStruc->pUserAddr,
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
            tMemStruc*   pMemStruc = (tMemStruc*)pIrp_p->AssociatedIrp.SystemBuffer;
            drv_unMapPdoMem(pMemStruc->pUserAddr, pMemStruc->size);
            status = STATUS_SUCCESS;
            pIrp_p->IoStatus.Information = 0;
            break;
        }

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

\param  driverHandle_p      Miniport driver handle returned by OS on registration

*/
//------------------------------------------------------------------------------
static void registerDrvIntf(NDIS_HANDLE driverHandle_p)
{
    NDIS_STATUS                      status = NDIS_STATUS_SUCCESS;
    UNICODE_STRING                   deviceName;
    UNICODE_STRING                   deviceLinkUnicodeString;
    NDIS_DEVICE_OBJECT_ATTRIBUTES    deviceObjectAttributes;
    PDRIVER_DISPATCH                 dispatchTable[IRP_MJ_MAXIMUM_FUNCTION + 1];

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
    deviceObjectAttributes.Header.Type = NDIS_OBJECT_TYPE_DEFAULT;
    deviceObjectAttributes.Header.Revision = NDIS_DEVICE_OBJECT_ATTRIBUTES_REVISION_1;
    deviceObjectAttributes.Header.Size = sizeof(NDIS_DEVICE_OBJECT_ATTRIBUTES);
    deviceObjectAttributes.DeviceName = &deviceName;
    deviceObjectAttributes.SymbolicName = &deviceLinkUnicodeString;
    deviceObjectAttributes.MajorFunctions = &dispatchTable[0];
    deviceObjectAttributes.ExtensionSize = 0;
    deviceObjectAttributes.DefaultSDDLString = NULL;
    deviceObjectAttributes.DeviceClassGuid = 0;

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
    if (plkDriverInstance_l.pDrvDeviceHandle != NULL)
    {
        NdisDeregisterDeviceEx(plkDriverInstance_l.pDrvDeviceHandle);
        plkDriverInstance_l.pDrvDeviceHandle = NULL;
    }
}

//------------------------------------------------------------------------------
/**
\brief  Start heartbeat timer

The function starts the timer used for updating the heartbeat counter.

\param  timeInMs_p          Timeout value in milliseconds

\ingroup module_driver_linux_kernel
*/
//------------------------------------------------------------------------------
void startHeartbeatTimer(LONG timeInMs_p)
{
    LARGE_INTEGER    dueTime;

    if (timeInMs_p < 0)
    {
        // negative time not possible.
        DEBUG_LVL_ERROR_TRACE("Negative time for heartbeat timer was specified\n");
        return;
    }

    dueTime.QuadPart = -(timeInMs_p * 10000);
    NdisSetTimerObject(heartbeatTimer_l, dueTime, timeInMs_p, NULL);
}

//------------------------------------------------------------------------------
/**
\brief  Stop heartbeat timer

The function stops the timer used for updating the heartbeat counter.

\ingroup module_driver_linux_kernel
*/
//------------------------------------------------------------------------------
void stopHeartbeatTimer(void)
{
    NdisCancelTimerObject(heartbeatTimer_l);
}

//------------------------------------------------------------------------------
/**
\brief  Increase heartbeat timer callback

The function implements the timer callback function used to increase the
heartbeat counter.

\param  pSystemParam1_p         A pointer to a system-specific value that is
                                reserved for system use.
\param  pFunctionContext_p      A pointer to a driver-supplied context area that
                                the driver passed to the NdisSetTimerObject
                                function. Optional.
\param  pSystemParam2_p         A pointer to a system-specific value that is
                                reserved for system use.
\param  pSystemParam3_p         A pointer to a system-specific value that is
                                reserved for system use.

\ingroup module_driver_linux_kernel
*/
//------------------------------------------------------------------------------
static void increaseHeartbeatCb(void* pSystemParam1_p, void* pFunctionContext_p,
                                void* pSystemParam2_p, void* pSystemParam3_p)
{
    UNUSED_PARAMETER(pSystemParam1_p);
    UNUSED_PARAMETER(pFunctionContext_p);
    UNUSED_PARAMETER(pSystemParam2_p);
    UNUSED_PARAMETER(pSystemParam3_p);

    ctrlk_updateHeartbeat();
}

/// \}
