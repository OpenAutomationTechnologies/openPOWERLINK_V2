/**
********************************************************************************
\file   main.c

\brief  main file for Linux kernel PCIe interface module

This file contains the main part of the Linux kernel PCIe interface module
for the openPOWERLINK kernel stack.

\ingroup module_driver_linux_kernel_pcie
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2014, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
Copyright (c) 2015, Kalycito Private Limited
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
#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/version.h>
#include <linux/mm.h>
#include <asm/uaccess.h>
#include <asm/page.h>
#include <asm/atomic.h>
#include <linux/kthread.h>
#include <linux/delay.h>

#include <common/driver.h>
#include <common/memmap.h>
#include <drvintf.h>
#include <pciedrv.h>
//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
MODULE_LICENSE("Dual BSD/GPL");
MODULE_AUTHOR("chidrupaya.sr@kalycito.com");
MODULE_DESCRIPTION("openPOWERLINK PCIe driver");

// VM_RESERVED is removed in kernels > 3.7
#ifndef VM_RESERVED
#define VM_RESERVED    (VM_DONTEXPAND | VM_DONTDUMP)
#endif

//------------------------------------------------------------------------------
// module global vars
//------------------------------------------------------------------------------

INT                     plkMajor_g = 0;
INT                     plkMinor_g = 0;
INT                     plkNrDevs_g = 1;
dev_t                   plkDev_g;
struct class*           plkClass_g;
struct cdev             plkCdev_g;
atomic_t                openCount_g;

//------------------------------------------------------------------------------
// global function prototypes
//------------------------------------------------------------------------------

//============================================================================//
//            P R I V A T E   D E F I N I T I O N S                           //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#define QUEUE_WAIT_TIMEOUT          (10 * HZ / 1000)    // 10ms timeout
#define K2U_EVENT_WAIT_TIMEOUT      (500 * HZ / 1000)

//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------
/**
\brief Structure holding the main module instance

The structure holds the instance parameters for the PCIe interface driver main module.
*/
typedef struct
{
    wait_queue_head_t       userWaitQueue;  ///< Wait queue for receiving user events from PCP.
    UINT8                   aK2URxBuffer[sizeof(tEvent) + MAX_EVENT_ARG_SIZE];  ///< Temporary buffer for receiving user events.
    ULONG                   bufPageOffset;  ///< Temporary variable to hold the byte offset of the last memory mapped kernel address, from the page boundary.
    UINT8*                  pPdoMem;        ///< Pointer to the PDO memory.
    size_t                  pdoMemSize;     ///< Size of the PDO memory.
    ULONG                   pdoVmaStartAddr;///< Vma start address of the mmapped PDO memory. Used to free PDO memory.
    BOOL                    fSyncEnabled;   ///< Flag to indicate whether user sync event forwarding is enabled.
} tDrvInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tDrvInstance    instance_l;          // Instance of this driver

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------

static INT __init   plkIntfInit(void);
static void __exit  plkIntfExit(void);

static INT          plkIntfOpen(struct inode* pDeviceFile_p,
                                struct file* pInstance_p);
static INT          plkIntfRelease(struct inode* pDeviceFile_p,
                                   struct file* pInstance_p);
static ssize_t      plkIntfRead(struct file* pInstance_p, char* pDstBuff_p,
                                size_t buffSize_p, loff_t* pFileOffs_p);
static ssize_t      plkIntfWrite(struct file* pInstance_p, const char* pSrcBuff_p,
                                 size_t buffSize_p, loff_t* pFileOffs_p);
#ifdef HAVE_UNLOCKED_IOCTL
static long         plkIntfIoctl(struct file* pFile_p, unsigned INT cmd, ULONG arg_p);
#else
static INT          plkIntfIoctl(struct inode* pDev_p, struct file* pFile_p,
                                 unsigned INT cmd_p, ULONG arg_p);
#endif

static INT          plkIntfMmap(struct file* pFile_p, struct vm_area_struct* pVma_p);
static void         plkIntfVmaOpen(struct vm_area_struct* pVma_p);
static void         plkIntfVmaClose(struct vm_area_struct* pVma_p);

static INT          executeCmd(ULONG arg_p);
static INT          readInitParam(ULONG arg_p);
static INT          storeInitParam(ULONG arg_p);
static INT          getStatus(ULONG arg_p);
static INT          getHeartbeat(ULONG arg_p);
static INT          sendAsyncFrame(ULONG arg_p);
static INT          writeErrorObject(ULONG arg_p);
static INT          readErrorObject(ULONG arg_p);
static INT          getEventForUser(ULONG arg_p);
static INT          postEventFromUser(ULONG arg_p);

//------------------------------------------------------------------------------
//  Kernel module specific data structures
//------------------------------------------------------------------------------
module_init(plkIntfInit);
module_exit(plkIntfExit);

static struct file_operations powerlinkFileOps_g =
{
    .owner =     THIS_MODULE,
    .open =      plkIntfOpen,
    .release =   plkIntfRelease,
    .read =      plkIntfRead,
    .write =     plkIntfWrite,
#ifdef HAVE_UNLOCKED_IOCTL
    .unlocked_ioctl = plkIntfIoctl,
#else
    .ioctl =     plkIntfIoctl,
#endif
    .mmap =      plkIntfMmap,
};

static struct vm_operations_struct powerlinkVmOps =
{
    .open = plkIntfVmaOpen,
    .close = plkIntfVmaClose,
};

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  module initialization

The function implements openPOWERLINK kernel pcie interface module
initialization function.

\ingroup module_driver_linux_kernel_pcie
*/
//------------------------------------------------------------------------------
static INT __init plkIntfInit(void)
{
    INT    err;

    DEBUG_LVL_ALWAYS_TRACE("PLK: plkIntfInit()  Driver build: %s / %s\n", __DATE__, __TIME__);
    DEBUG_LVL_ALWAYS_TRACE("PLK: plkIntfInit()  Stack version: %s\n", PLK_DEFINED_STRING_VERSION);
    plkDev_g = 0;
    atomic_set(&openCount_g, 0);

    if ((err = alloc_chrdev_region(&plkDev_g, plkMinor_g, plkNrDevs_g, PLK_DRV_NAME)) < 0)
    {
        DEBUG_LVL_ERROR_TRACE("PLK: Failing allocating major number\n");
        return err;
    }

    plkMajor_g = MAJOR(plkDev_g);
    TRACE("Allocated major number: %d\n", plkMajor_g);

    if ((plkClass_g = class_create(THIS_MODULE, PLK_DRV_NAME)) == NULL)
    {
        DEBUG_LVL_ERROR_TRACE("class_create() failed!\n");
        unregister_chrdev_region(plkDev_g, plkNrDevs_g);
        return -1;
    }

    if (device_create(plkClass_g, NULL, plkDev_g, NULL, PLK_DRV_NAME) == NULL)
    {
        DEBUG_LVL_ERROR_TRACE("device_create() failed!\n");
        class_destroy(plkClass_g);
        unregister_chrdev_region(plkDev_g, plkNrDevs_g);
        return -1;
    }

    cdev_init(&plkCdev_g, &powerlinkFileOps_g);
    if ((err = cdev_add(&plkCdev_g, plkDev_g, 1)) == -1)
    {
        DEBUG_LVL_ERROR_TRACE("cdev_add() failed!\n");
        device_destroy(plkClass_g, plkDev_g);
        class_destroy(plkClass_g);
        unregister_chrdev_region(plkDev_g, plkNrDevs_g);
        return -1;
    }

    return 0;
}

//------------------------------------------------------------------------------
/**
\brief  module clean up and exit

The function implements openPOWERLINK kernel pcie interface module exit function.

\ingroup module_driver_linux_kernel_pcie
*/
//------------------------------------------------------------------------------
static void __exit plkIntfExit(void)
{
    DEBUG_LVL_ALWAYS_TRACE("PLK: plkIntfExit...\n");

    cdev_del(&plkCdev_g);
    device_destroy(plkClass_g, plkDev_g);
    class_destroy(plkClass_g);
    unregister_chrdev_region(plkDev_g, plkNrDevs_g);

    DEBUG_LVL_ALWAYS_TRACE("PLK: Driver '%s' removed.\n", PLK_DRV_NAME);
}

//------------------------------------------------------------------------------
/**
\brief  openPOWERLINK pcie driver open function

The function implements openPOWERLINK kernel pcie interface module open function.

\param  pDeviceFile_p   Pointer to device file data structure.
\param  pInstance_p     Pointer to the device file object.

\return The function returns an integer value.
\retval 0               Successful.
\retval -ENOTTY         One instance of the driver is already active.
\retval -EIO            Unable to initialize the driver.

\ingroup module_driver_linux_kernel_pcie
*/
//------------------------------------------------------------------------------
static INT plkIntfOpen(struct inode* pDeviceFile_p, struct file* pInstance_p)
{
    DEBUG_LVL_ALWAYS_TRACE("PLK: + plkIntfOpen...\n");

    if (atomic_inc_return(&openCount_g) > 1)
    {
        atomic_dec(&openCount_g);
        return -ENOTTY;
    }

    UNUSED_PARAMETER(pDeviceFile_p);
    UNUSED_PARAMETER(pInstance_p);

    instance_l.bufPageOffset = 0;
    instance_l.pPdoMem = NULL;
    instance_l.fSyncEnabled = FALSE;

    if (pciedrv_init() != kErrorOk)
    {
        atomic_dec(&openCount_g);
        return -EIO;
    }

    if (drvintf_init() != kErrorOk)
    {
        atomic_dec(&openCount_g);
        return -EIO;
    }

    init_waitqueue_head(&instance_l.userWaitQueue);

    DEBUG_LVL_ALWAYS_TRACE("PLK: + plkIntfOpen - OK\n");

    return 0;
}

//------------------------------------------------------------------------------
/**
\brief  openPOWERLINK pcie driver close function

The function implements openPOWERLINK kernel module close function.

\param  pDeviceFile_p   Pointer to device file data structure.
\param  pInstance_p     Pointer to the device file object.

\return The function returns an integer value.
\retval 0               Successful.

\ingroup module_driver_linux_kernel_pcie
*/
//------------------------------------------------------------------------------
static INT  plkIntfRelease(struct inode* pDeviceFile_p, struct file* pInstance_p)
{
    DEBUG_LVL_ALWAYS_TRACE("PLK: + plkIntfRelease...\n");

    UNUSED_PARAMETER(pDeviceFile_p);
    UNUSED_PARAMETER(pInstance_p);

    instance_l.bufPageOffset = 0;
    instance_l.pPdoMem = NULL;
    instance_l.fSyncEnabled = FALSE;

    drvintf_exit();
    pciedrv_shutdown();
    atomic_dec(&openCount_g);
    DEBUG_LVL_ALWAYS_TRACE("PLK: + plkIntfRelease - OK\n");
    return 0;
}

//------------------------------------------------------------------------------
/**
\brief  openPOWERLINK pcie driver read function

The function implements openPOWERLINK kernel pcie interface module read function.

\param  pInstance_p     Pointer to the device file object.
\param  pDstBuff_p      Pointer to the destination buffer, to copy the read data.
\param  buffSize_p      Size of the destination buffer.
\param  pFileOffs_p     Pointer to the long offset in the file.

\return The function returns the actual size of the read data.

\ingroup module_driver_linux_kernel_pcie
*/
//------------------------------------------------------------------------------
static ssize_t plkIntfRead(struct file* pInstance_p, char* pDstBuff_p,
                             size_t buffSize_p, loff_t* pFileOffs_p)
{
    INT    ret = 0;

    UNUSED_PARAMETER(pInstance_p);
    UNUSED_PARAMETER(pDstBuff_p);
    UNUSED_PARAMETER(buffSize_p);
    UNUSED_PARAMETER(pFileOffs_p);

    DEBUG_LVL_ALWAYS_TRACE("PLK: + plkIntfRead...\n");
    DEBUG_LVL_ALWAYS_TRACE("PLK:   Sorry, this operation isn't supported.\n");
    ret = -EINVAL;
    DEBUG_LVL_ALWAYS_TRACE("PLK: - plkIntfRead (iRet=%d)\n", ret);
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  openPOWERLINK pcie driver write function

The function implements openPOWERLINK kernel pcie interface module write function.

\param  pInstance_p     Pointer to the device file object.
\param  pSrcBuff_p      Pointer to the source buffer, to copy the data to
                        be written.
\param  buffSize_p      Size of the data to be written.
\param  pFileOffs_p     Pointer to the long offset in the file.

\return The function returns the actual size of the written data.

\ingroup module_driver_linux_kernel_pcie
*/
//------------------------------------------------------------------------------
static ssize_t plkIntfWrite(struct file* pInstance_p, const char* pSrcBuff_p,
                              size_t buffSize_p, loff_t* pFileOffs_p)
{
    INT    ret = 0;

    UNUSED_PARAMETER(pInstance_p);
    UNUSED_PARAMETER(pSrcBuff_p);
    UNUSED_PARAMETER(buffSize_p);
    UNUSED_PARAMETER(pFileOffs_p);

    DEBUG_LVL_ALWAYS_TRACE("PLK: + plkIntfWrite...\n");
    DEBUG_LVL_ALWAYS_TRACE("PLK:   Sorry, this operation isn't supported.\n");
    ret = -EINVAL;
    DEBUG_LVL_ALWAYS_TRACE("PLK: - plkIntfWrite (iRet=%d)\n", ret);
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  openPOWERLINK pcie driver ioctl function

The function implements openPOWERLINK kernel pcie interface module ioctl function.

\param  pDev_p      Pointer to device file data structure.
\param  pFile_p     Pointer to the device file object.
\param  cmd_p       Type of the ioctl operation.
\param  arg_p       Argument address for the ioctl operation.

\return The function returns an integer value.
\retval 0           The ioctl operation was successful.
\retval < 0         The ioctl operation generateed error.

\ingroup module_driver_linux_kernel_pcie
*/
//------------------------------------------------------------------------------
#ifdef HAVE_UNLOCKED_IOCTL
static long plkIntfIoctl(struct file* pFile_p, UINT cmd_p,
                         ULONG arg_p)
#else
static INT  plkIntfIoctl(struct inode* pDev_p, struct file* pFile_p,
                           UINT cmd_p, ULONG arg_p)
#endif
{
    INT             ret = -EINVAL;
    tOplkError      oplRet;

    UNUSED_PARAMETER(pFile_p);
#ifndef HAVE_UNLOCKED_IOCTL
    UNUSED_PARAMETER(pDev_p);
#endif

    switch (cmd_p)
    {
        case PLK_CMD_CTRL_EXECUTE_CMD:
            ret = executeCmd(arg_p);
            break;

        case PLK_CMD_CTRL_STORE_INITPARAM:
            ret = storeInitParam(arg_p);
            break;

        case PLK_CMD_CTRL_READ_INITPARAM:
            ret = readInitParam(arg_p);
            break;

        case PLK_CMD_CTRL_GET_STATUS:
            ret = getStatus(arg_p);
            break;

        case PLK_CMD_CTRL_GET_HEARTBEAT:
            ret = getHeartbeat(arg_p);
            break;

        case PLK_CMD_POST_EVENT:
            ret = postEventFromUser(arg_p);
            break;

        case PLK_CMD_GET_EVENT:
            ret = getEventForUser(arg_p);
            break;

        case PLK_CMD_DLLCAL_ASYNCSEND:
            ret = sendAsyncFrame(arg_p);
            break;

        case PLK_CMD_ERRHND_WRITE:
            ret = writeErrorObject(arg_p);
            break;

        case PLK_CMD_ERRHND_READ:
            ret = readErrorObject(arg_p);
            break;

        case PLK_CMD_TIMESYNC_SYNC:
            if (instance_l.fSyncEnabled == FALSE)
            {
                pciedrv_regSyncHandler(timesynckcal_sendSyncEvent);
                pciedrv_enableSync(TRUE);
                instance_l.fSyncEnabled = TRUE;
            }

            // $$ Handle other errors
            if ((oplRet = timesynckcal_waitSyncEvent()) == kErrorRetry)
                ret = -ERESTARTSYS;
            else
                ret = 0;
            break;

        case PLK_CMD_PDO_MAP_OFFSET:
            if (copy_to_user((void __user*)arg_p, &instance_l.bufPageOffset, sizeof(ULONG)))
            {
                DEBUG_LVL_ERROR_TRACE("PDO mmapped offset fetch Error!!\n");
                ret = -EFAULT;
            }
            else
                ret = 0;

            break;

        default:
            DEBUG_LVL_ERROR_TRACE("PLK: - Invalid cmd (cmd=%d type=%d)\n", _IOC_NR(cmd_p), _IOC_TYPE(cmd_p));
            ret = -ENOTTY;
            break;
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  openPOWERLINK pcie driver mmap function

The function implements openPOWERLINK kernel pcie interface module mmap function.

\param  pFile_p     Pointer to the device file object.
\param  pVma_p      Pointer to the virtual memory object of user.

\return The function returns an integer value.
\retval 0           The ioctl operation was successful.
\retval < 0         The ioctl operation generateed error.

\ingroup module_driver_linux_kernel_pcie
*/
//------------------------------------------------------------------------------
static INT plkIntfMmap(struct file* pFile_p, struct vm_area_struct* pVma_p)
{
    UINT8*          pPciMem = NULL;
    size_t          memSize = 0;
    tOplkError      ret = kErrorOk;
    ULONG           pageAddr = 0;
    BOOL            fPdoMem = FALSE;

    DEBUG_LVL_ALWAYS_TRACE("%s() vma: vm_start:%lX vm_end:%lX vm_pgoff:%lX\n",
                           __func__, pVma_p->vm_start,
                           pVma_p->vm_end, pVma_p->vm_pgoff);

    UNUSED_PARAMETER(pFile_p);

    pVma_p->vm_flags |= VM_RESERVED | VM_IO;
    pVma_p->vm_ops = &powerlinkVmOps;

    if (pVma_p->vm_pgoff == 0)
    {
        // Get kernel space address for the pdo memory
        ret = drvintf_getPdoMem(&pPciMem, &memSize);

        if ((pPciMem == NULL) || (ret != kErrorOk))
        {
            DEBUG_LVL_ERROR_TRACE("%s() no pdo memory allocated!\n", __func__);
            return -ENOMEM;
        }

        fPdoMem = TRUE;
        instance_l.pPdoMem = pPciMem;
        instance_l.pdoMemSize = memSize;
        instance_l.pdoVmaStartAddr = pVma_p->vm_start;
        // Get the bus address of the pdo memory
        pageAddr = pciedrv_getBarPhyAddr(0) + ((ULONG)pPciMem - pciedrv_getBarAddr(0));
    }
    else
    {
        pPciMem = (UINT8*)(pVma_p->vm_pgoff << PAGE_SHIFT);
        // Get kernel space address for the passed memory
        ret = drvintf_mapKernelMem((UINT8*)pPciMem,
                                   (UINT8**)&pageAddr,
                                   (size_t)memSize);
        memSize = pVma_p->vm_end - pVma_p->vm_start;
        if (ret != kErrorOk)
            return -ENOMEM;

        // Get the bus address of the passed memory
        pageAddr = pciedrv_getBarPhyAddr(0) + ((ULONG)pageAddr - pciedrv_getBarAddr(0));
    }

    pVma_p->vm_pgoff = pageAddr >> PAGE_SHIFT;
    // Save the offset of the PDO memory address from the start of page boundary
    instance_l.bufPageOffset = (ULONG)(pageAddr - (pVma_p->vm_pgoff << PAGE_SHIFT));

    if (io_remap_pfn_range(pVma_p, pVma_p->vm_start, pVma_p->vm_pgoff,
                           memSize + instance_l.bufPageOffset,
                           pVma_p->vm_page_prot))
    {
        DEBUG_LVL_ERROR_TRACE("%s() remap_pfn_range failed\n", __func__);
        return -EAGAIN;
    }

    if (fPdoMem == TRUE)
    {
        // Get the bus address of the atomic access memory
        pageAddr = pciedrv_getBarPhyAddr(0) + ((ULONG)pPciMem + ATOMIC_MEM_OFFSET - pciedrv_getBarAddr(0));
        if (io_remap_pfn_range(pVma_p, pVma_p->vm_start + ATOMIC_MEM_OFFSET, pageAddr >> PAGE_SHIFT,
                               memSize + instance_l.bufPageOffset,
                               pVma_p->vm_page_prot))
        {
            DEBUG_LVL_ERROR_TRACE("%s() remap_pfn_range failed\n", __func__);
            return -EAGAIN;
        }
    }

    plkIntfVmaOpen(pVma_p);
    return 0;
}

//------------------------------------------------------------------------------
/**
\brief  openPOWERLINK pcie driver VMA open function

The function implements openPOWERLINK kernel module VMA open function.

\param  pVma_p      Pointer to the virtual memory object of user.

\ingroup module_driver_linux_kernel_pcie
*/
//------------------------------------------------------------------------------
static void plkIntfVmaOpen(struct vm_area_struct* pVma_p)
{
    DEBUG_LVL_ALWAYS_TRACE("%s() vma: vm_start:%lX vm_end:%lX vm_pgoff:%lX\n",
                           __func__, pVma_p->vm_start,
                           pVma_p->vm_end, pVma_p->vm_pgoff);
    UNUSED_PARAMETER(pVma_p);
}

//------------------------------------------------------------------------------
/**
\brief  openPOWERLINK pcie driver VMA close function

The function implements openPOWERLINK kernel module VMA close function.

\param  pVma_p      Pointer to the virtual memory object of user.

\ingroup module_driver_linux_kernel_pcie
*/
//------------------------------------------------------------------------------
static void plkIntfVmaClose(struct vm_area_struct* pVma_p)
{
    tOplkError      ret = kErrorOk;

    DEBUG_LVL_ALWAYS_TRACE("%s() vma: vm_start:%lX vm_end:%lX vm_pgoff:%lX\n",
                           __func__, pVma_p->vm_start,
                           pVma_p->vm_end, pVma_p->vm_pgoff);

    // Check if it is the PDO memory being freed
    if (instance_l.pdoVmaStartAddr == pVma_p->vm_start)
    {
        ret = drvintf_freePdoMem(&instance_l.pPdoMem, instance_l.pdoMemSize);
        if (ret != kErrorOk)
        {
            DEBUG_LVL_ERROR_TRACE("Pdo memory could not be freed!\n", __func__);
            return;
        }

        instance_l.pPdoMem = NULL;
        instance_l.pdoMemSize = 0;
        instance_l.pdoVmaStartAddr = 0;
    }
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief    Get an event for the user layer

This function waits for events to the user.

\param  arg_p              Ioctl argument. Contains the received event.

\return The function returns Linux error code.
*/
//------------------------------------------------------------------------------
static INT getEventForUser(ULONG arg_p)
{
    INT             ret;
    size_t          readSize;
    INT             loopCount = (K2U_EVENT_WAIT_TIMEOUT / QUEUE_WAIT_TIMEOUT);
    INT             i = 0;

    for (i = 0; i < loopCount; i++)
    {
        if (drvintf_getEvent((tEvent*)instance_l.aK2URxBuffer, &readSize) != kErrorOk)
        {
            ret = -EFAULT;
            break;
        }

        if (readSize > 0)
        {
            DEBUG_LVL_DRVINTF_TRACE("%s() copy kernel event to user: %d Bytes\n",
                                    __func__, readSize);
            if (copy_to_user((void __user*)arg_p, instance_l.aK2URxBuffer, readSize))
            {
                DEBUG_LVL_ERROR_TRACE("Event fetch Error!!\n");
                ret = -EFAULT;
                break;
            }

            ret = 0;
            break;
        }

        ret = wait_event_interruptible_timeout(instance_l.userWaitQueue, 0,
                                               QUEUE_WAIT_TIMEOUT);

        // Ignore timeout (ret = 0) condition as we are using it to sleep

        if (ret == -ERESTARTSYS)
        {
            DEBUG_LVL_ERROR_TRACE("%s() interrupted\n", __func__);
            break;
        }
        // else ignore the rest of the cases as we are not using any condition or signal to wakeup
    }

    if (i == loopCount)
    {
        // No event received from the kernel
        ret = -ERESTARTSYS;
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief    Post event from user

This function posts an event from the user layer to the kernel queue.

\param  arg_p              Ioctl argument. Contains the event to post.

\return The function returns Linux error code.
*/
//------------------------------------------------------------------------------
static INT postEventFromUser(ULONG arg_p)
{
    tOplkError      ret = kErrorOk;
    tEvent          event;
    UINT8*          pArg = NULL;
    INT             order = 0;

    if (copy_from_user(&event, (const void __user*)arg_p, sizeof(tEvent)))
        return -EFAULT;

    if (event.eventArgSize != 0)
    {
        order = get_order(event.eventArgSize);
        pArg = (UINT8*)__get_free_pages(GFP_KERNEL, order);

        if (!pArg)
            return -EIO;

        if (copy_from_user(pArg, (const void __user*)event.eventArg.pEventArg, event.eventArgSize))
        {
            free_pages((ULONG)pArg, order);
            return -EFAULT;
        }

        event.eventArg.pEventArg = (void*)pArg;
    }

    switch (event.eventSink)
    {
        case kEventSinkNmtk:
        case kEventSinkSync:
        case kEventSinkDllk:
        case kEventSinkDllkCal:
        case kEventSinkPdok:
        case kEventSinkPdokCal:
        case kEventSinkErrk:
            /* TRACE("U2K  type:(%d) sink:(%d) size:%d!\n",
                     event.eventType,
                     event.eventSink,
                     event.eventArgSize); */
            if (drvintf_postEvent(&event) != kErrorOk)
                ret = -EIO;
            break;

        case kEventSinkNmtMnu:
        case kEventSinkNmtu:
        case kEventSinkSdoAsySeq:
        case kEventSinkApi:
        case kEventSinkDlluCal:
        case kEventSinkErru:
        default:
            ret = -EIO;
            break;
    }

    if (event.eventArgSize != 0)
        free_pages((ULONG)pArg, order);

    return 0;
}

//------------------------------------------------------------------------------
/**
\brief  Execute control command ioctl

The function implements the calling of the executeCmd function in the control
module using the ioctl interface and forwards it to the driver using via the
shared memory interface.

\param arg_p    Control command argument passed by the ioctl interface.

\return The function returns Linux error code.
*/
//------------------------------------------------------------------------------
static INT executeCmd(ULONG arg_p)
{
    tCtrlCmd    ctrlCmd;

    if (copy_from_user(&ctrlCmd, (const void __user*)arg_p, sizeof(tCtrlCmd)))
        return -EFAULT;

    if (drvintf_executeCmd(&ctrlCmd) != kErrorOk)
        return -EFAULT;

    if (copy_to_user((void __user*)arg_p, &ctrlCmd, sizeof(tCtrlCmd)))
        return -EFAULT;

    return 0;
}

//------------------------------------------------------------------------------
/**
\brief  Store init param ioctl

The function implements the calling of the storeInitParam function in the
control module using the ioctl interface and forwards it to the driver using
via the shared memory interface.

\param arg_p    Control module initialization parameters argument passed by
                the ioctl interface.

\return The function returns Linux error code.
*/
//------------------------------------------------------------------------------
static INT storeInitParam(ULONG arg_p)
{
    tCtrlInitParam    initParam;

    if (copy_from_user(&initParam, (const void __user*)arg_p, sizeof(tCtrlInitParam)))
        return -EFAULT;

    if (drvintf_storeInitParam(&initParam) != kErrorOk)
        return -EFAULT;

    return 0;
}

//------------------------------------------------------------------------------
/**
\brief  Read init param ioctl

The function implements the calling of the readInitParam function in the control
module using the ioctl interface and forwards it to the driver using via the
shared memory interface.

\param arg_p    Pointer to the control module initialization parameters
                argument passed by the ioctl interface.

\return The function returns Linux error code.
*/
//------------------------------------------------------------------------------
static INT readInitParam(ULONG arg_p)
{
    tCtrlInitParam    initParam;

    if (drvintf_readInitParam(&initParam) != kErrorOk)
        return -EFAULT;

    if (copy_to_user((void __user*)arg_p, &initParam, sizeof(tCtrlInitParam)))
        return -EFAULT;

    return 0;
}

//------------------------------------------------------------------------------
/**
\brief  Get status ioctl

The function implements the calling of the getStatus function in the control
module using the ioctl interface and forwards it to the driver using via the
shared memory interface.

\param arg_p    Pointer to the control status argument passed by the ioctl interface.

\return The function returns Linux error code.
*/
//------------------------------------------------------------------------------
static INT getStatus(ULONG arg_p)
{
    UINT16    status;

    if (drvintf_getStatus(&status) != kErrorOk)
        return -EFAULT;

    put_user(status, (unsigned short __user*)arg_p);
    return 0;
}

//------------------------------------------------------------------------------
/**
\brief  Get heartbeat ioctl

The function implements the calling of the getHeartbeat function in the control
module using the ioctl interface and forwards it to the driver using via the
shared memory interface.

\param arg_p    Pointer to the PCP heartbeat argument passed by the ioctl interface.

\return The function returns Linux error code.
*/
//------------------------------------------------------------------------------
static INT getHeartbeat(ULONG arg_p)
{
    UINT16    heartbeat;

    if (drvintf_getHeartbeat(&heartbeat) != kErrorOk)
        return -EFAULT;

    put_user(heartbeat, (unsigned short __user*)arg_p);
    return 0;
}

//------------------------------------------------------------------------------
/**
\brief  Sending async frame ioctl

The function implements the ioctl used for sending asynchronous frames.

\param arg_p    Pointer to the async send argument passed by the ioctl interface.

\return The function returns Linux error code.
*/
//------------------------------------------------------------------------------
static INT sendAsyncFrame(ULONG arg_p)
{
    UINT8*                  pBuf;
    tIoctlDllCalAsync       asyncFrameInfo;
    INT                     order;
    INT                     ret = 0;

    order = get_order(C_DLL_MAX_ASYNC_MTU);
    pBuf = (UINT8*)__get_free_pages(GFP_KERNEL, order);

    if (copy_from_user(&asyncFrameInfo, (const void __user*)arg_p, sizeof(tIoctlDllCalAsync)))
    {
        free_pages((ULONG)pBuf, order);
        return -EFAULT;
    }

    if (copy_from_user(pBuf, (const void __user*)asyncFrameInfo.pData, asyncFrameInfo.size))
    {
        free_pages((ULONG)pBuf, order);
        return -EFAULT;
    }

    asyncFrameInfo.pData = pBuf;
    if (drvintf_sendAsyncFrame(asyncFrameInfo.queue, asyncFrameInfo.size,
                               asyncFrameInfo.pData) != kErrorOk)
        ret = -EFAULT;

    free_pages((ULONG)pBuf, order);

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Write error object ioctl

The function implements the ioctl for writing an error object.

\param arg_p    Pointer to the error object argument passed by the ioctl interface.

\return The function returns Linux error code.
*/
//------------------------------------------------------------------------------
static INT writeErrorObject(ULONG arg_p)
{
    tErrHndIoctl        errorObject;

    if (copy_from_user(&errorObject, (const void __user*)arg_p, sizeof(tErrHndIoctl)))
        return -EFAULT;

    if (drvintf_writeErrorObject(errorObject.offset, errorObject.errVal) != kErrorOk)
        return -EFAULT;

    return 0;
}

//------------------------------------------------------------------------------
/**
\brief  Read error object ioctl

The function implements the ioctl for reading error objects.

\param arg_p    Pointer to the error object argument passed by the ioctl interface.

\return The function returns Linux error code.
*/
//------------------------------------------------------------------------------
static INT readErrorObject(ULONG arg_p)
{
    tErrHndIoctl        errorObject;

    if (copy_from_user(&errorObject, (const void __user*)arg_p, sizeof(tErrHndIoctl)))
        return -EFAULT;

    if (drvintf_readErrorObject(errorObject.offset, &errorObject.errVal) != kErrorOk)
        return -EFAULT;

    if (copy_to_user((void __user*)arg_p, &errorObject, sizeof(tErrHndIoctl)))
        return -EFAULT;

    return 0;
}

/// \}
