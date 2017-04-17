/**
********************************************************************************
\file   main.c

\brief  main file for Linux kernel Zynq interface module

This file contains the main part of the Linux kernel Zynq interface module
for the openPOWERLINK kernel stack.

\ingroup module_driver_linux_kernel_zynq
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2016, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
Copyright (c) 2017, Kalycito Private Limited
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
#include "zynqdrv.h"

#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/version.h>
#include <linux/mm.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <asm/uaccess.h>
#include <asm/page.h>
#include <asm/atomic.h>

#include <common/driver.h>
#include <common/memmap.h>
#include <kernel/timesynckcal.h>

//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
MODULE_LICENSE("Dual BSD/GPL");
MODULE_AUTHOR("swetha.r@kalycito.com");
MODULE_DESCRIPTION("openPOWERLINK Linux kernel driver for Zynq/FPGA");

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

The structure holds the instance parameters for the Zynq interface driver main module.
*/
typedef struct
{
    wait_queue_head_t   userWaitQueue;      ///< Wait queue for receiving user events from PCP.
    UINT8               aK2URxBuffer[sizeof(tEvent) + MAX_EVENT_ARG_SIZE];  ///< Temporary buffer for receiving user events.
    ULONG               bufPageOffset;      ///< Temporary variable to hold the byte offset of the last memory mapped kernel address, from the page boundary.
    void*               pPdoMem;            ///< Pointer to the PDO memory.
    size_t              pdoMemSize;         ///< Size of the PDO memory.
    ULONG               pdoVmaStartAddr;    ///< Vma start address of the mmapped PDO memory. Used to free PDO memory.
    BOOL                fSyncEnabled;       ///< Flag to indicate whether user sync event forwarding is enabled.
} tDrvInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tDrvInstance    instance_l;          // Instance of this driver

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static int __init   plkIntfInit(void);
static void __exit  plkIntfExit(void);

static int          plkIntfOpen(struct inode* pInode_p,
                                struct file* pFile_p);
static int          plkIntfRelease(struct inode* pInode_p,
                                   struct file* pFile_p);
static ssize_t      plkIntfRead(struct file* pFile_p,
                                char __user* pDstBuff_p,
                                size_t buffSize_p,
                                loff_t* pFileOffs_p);
static ssize_t      plkIntfWrite(struct file* pFile_p,
                                 const char __user* pSrcBuff_p,
                                 size_t buffSize_p,
                                 loff_t* pFileOffs_p);
#ifdef HAVE_UNLOCKED_IOCTL
static long         plkIntfIoctl(struct file* pFile_p,
                                 unsigned int cmd_p,
                                 unsigned long arg_p);
#else
static int          plkIntfIoctl(struct inode* pInode_p,
                                 struct file* pFile_p,
                                 unsigned int cmd_p,
                                 unsigned long arg_p);
#endif
static int          plkIntfMmap(struct file* pFile_p,
                                struct vm_area_struct* pVma_p);

static INT          plkIntfMmap(struct file* pFile_p, struct vm_area_struct* pVma_p);
static void         plkIntfVmaOpen(struct vm_area_struct* pVma_p);
static void         plkIntfVmaClose(struct vm_area_struct* pVma_p);

static int          executeCmd(unsigned long arg_p);
static int          readInitParam(unsigned long arg_p);
static int          storeInitParam(unsigned long arg_p);
static int          getStatus(unsigned long arg_p);
static int          getHeartbeat(unsigned long arg_p);
static int          sendAsyncFrame(unsigned long arg_p);
static int          writeErrorObject(unsigned long arg_p);
static int          readErrorObject(unsigned long arg_p);
static int          getEventForUser(unsigned long arg_p);
static int          postEventFromUser(unsigned long arg_p);
static int          writeFileBuffer(unsigned long arg_p);
static int          getFileBufferSize(unsigned long arg_p);
#if defined(CONFIG_INCLUDE_SOC_TIME_FORWARD)
static int          getSocTimestampAddress(unsigned long arg_p);
#endif

//------------------------------------------------------------------------------
//  Kernel module specific data structures
//------------------------------------------------------------------------------
module_init(plkIntfInit);               // Define kernel module init function
module_exit(plkIntfExit);               // Define kernel module exit function

static struct file_operations   powerlinkFileOps_l =
{
    .owner          = THIS_MODULE,
    .open           = plkIntfOpen,
    .release        = plkIntfRelease,
    .read           = plkIntfRead,
    .write          = plkIntfWrite,
#if defined(HAVE_UNLOCKED_IOCTL)
    .unlocked_ioctl = plkIntfIoctl,
#else
    .ioctl          = plkIntfIoctl,
#endif
    .mmap           = plkIntfMmap,
};

static struct vm_operations_struct  powerlinkVmOps_l =
{
    .open  = plkIntfVmaOpen,
    .close = plkIntfVmaClose,
};

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  module initialization

The function implements openPOWERLINK kernel zynq interface module
initialization function.

\ingroup module_driver_linux_kernel_zynq
*/
//------------------------------------------------------------------------------
static int __init plkIntfInit(void)
{
    int err;

    DEBUG_LVL_ALWAYS_TRACE("PLK: %s()  Driver build: %s / %s\n",
                           __func__,
                           __DATE__,
                           __TIME__);
    DEBUG_LVL_ALWAYS_TRACE("PLK: %s()  Stack version: %s\n",
                           __func__,
                           PLK_DEFINED_STRING_VERSION);

    plkDev_g = 0;
    atomic_set(&openCount_g, 0);

    err = alloc_chrdev_region(&plkDev_g, plkMinor_g, plkNrDevs_g, PLK_DRV_NAME);
    if (err < 0)
    {
        DEBUG_LVL_ERROR_TRACE("PLK: Allocating major number failed.\n");
        return err;
    }

    plkMajor_g = MAJOR(plkDev_g);
    DEBUG_LVL_ALWAYS_TRACE("Allocated major number: %d\n", plkMajor_g);

    plkClass_g = class_create(THIS_MODULE, PLK_DRV_NAME);
    if (plkClass_g == NULL)
    {
        DEBUG_LVL_ERROR_TRACE("class_create() failed!\n");
        unregister_chrdev_region(plkDev_g, plkNrDevs_g);
        return -EPERM;
    }

    if (device_create(plkClass_g, NULL, plkDev_g, NULL, PLK_DRV_NAME) == NULL)
    {
        DEBUG_LVL_ERROR_TRACE("device_create() failed!\n");
        class_destroy(plkClass_g);
        unregister_chrdev_region(plkDev_g, plkNrDevs_g);
        return -EPERM;
    }

    cdev_init(&plkCdev_g, &powerlinkFileOps_l);
    err = cdev_add(&plkCdev_g, plkDev_g, 1);
    if (err == -1)
    {
        DEBUG_LVL_ERROR_TRACE("cdev_add() failed!\n");
        device_destroy(plkClass_g, plkDev_g);
        class_destroy(plkClass_g);
        unregister_chrdev_region(plkDev_g, plkNrDevs_g);
        return -EPERM;
    }

    return 0;
}

//------------------------------------------------------------------------------
/**
\brief  module clean up and exit

The function implements openPOWERLINK kernel zynq interface module exit function.

\ingroup module_driver_linux_kernel_zynq
*/
//------------------------------------------------------------------------------
static void __exit plkIntfExit(void)
{
    DEBUG_LVL_ALWAYS_TRACE("PLK: %s()...\n", __func__);

    cdev_del(&plkCdev_g);
    device_destroy(plkClass_g, plkDev_g);
    class_destroy(plkClass_g);
    unregister_chrdev_region(plkDev_g, plkNrDevs_g);

    DEBUG_LVL_ALWAYS_TRACE("PLK: Driver '%s' removed.\n", PLK_DRV_NAME);
}

//------------------------------------------------------------------------------
/**
\brief  openPOWERLINK zynq driver open function

The function implements openPOWERLINK kernel zynq interface module open function.

\param[in,out]  pInode_p            Pointer to inode data structure.
\param[in,out]  pFile_p             Pointer to the device file object.

\return The function returns an integer value.
\retval 0                           Successful.
\retval -ENOTTY                     One instance of the driver is already active.
\retval -EIO                        Unable to initialize the driver.

\ingroup module_driver_linux_kernel_zynq
*/
//------------------------------------------------------------------------------
static int plkIntfOpen(struct inode* pInode_p,
                       struct file* pFile_p)
{
    UNUSED_PARAMETER(pInode_p);
    UNUSED_PARAMETER(pFile_p);

    DEBUG_LVL_ALWAYS_TRACE("PLK: + %s()...\n", __func__);

    if (atomic_inc_return(&openCount_g) > 1)
    {
        atomic_dec(&openCount_g);
        return -ENOTTY;
    }

    instance_l.bufPageOffset = 0;
    instance_l.pPdoMem = NULL;
    instance_l.fSyncEnabled = FALSE;

    if (zynqdrv_init() != kErrorOk)
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

    DEBUG_LVL_ALWAYS_TRACE("PLK: + %s() - OK\n", __func__);

    return 0;
}

//------------------------------------------------------------------------------
/**
\brief  openPOWERLINK zynq driver close function

The function implements openPOWERLINK kernel module close function.

\param[in,out]  pInode_p            Pointer to the inode data structure.
\param[in,out]  pFile_p             Pointer to the device file object.

\return The function returns an integer value.
\retval 0                           Successful.

\ingroup module_driver_linux_kernel_zynq
*/
//------------------------------------------------------------------------------
static int plkIntfRelease(struct inode* pInode_p,
                          struct file* pFile_p)
{
    UNUSED_PARAMETER(pInode_p);
    UNUSED_PARAMETER(pFile_p);

    DEBUG_LVL_ALWAYS_TRACE("PLK: + %s()...\n", __func__);

    instance_l.bufPageOffset = 0;
    instance_l.pPdoMem = NULL;
    instance_l.fSyncEnabled = FALSE;

    drvintf_exit();
    zynqdrv_exit();
    atomic_dec(&openCount_g);

    DEBUG_LVL_ALWAYS_TRACE("PLK: + %s() - OK\n", __func__);

    return 0;
}

//------------------------------------------------------------------------------
/**
\brief  openPOWERLINK zynq driver read function

The function implements openPOWERLINK kernel zynq interface module read function.

\param[in,out]  pFile_p             Pointer to the device file object.
\param[out]     pDstBuff_p          Pointer to the destination buffer, to copy the read data.
\param[in]      buffSize_p          Size of the destination buffer.
\param[in,out]  pFileOffs_p         Pointer to the long offset in the file.

\return Returns the amount of data that has been read.

\ingroup module_driver_linux_kernel_zynq
*/
//------------------------------------------------------------------------------
static ssize_t plkIntfRead(struct file* pFile_p,
                           char __user* pDstBuff_p,
                           size_t buffSize_p,
                           loff_t* pFileOffs_p)
{
    int ret;

    UNUSED_PARAMETER(pFile_p);
    UNUSED_PARAMETER(pDstBuff_p);
    UNUSED_PARAMETER(buffSize_p);
    UNUSED_PARAMETER(pFileOffs_p);

    DEBUG_LVL_ALWAYS_TRACE("PLK: + %s()...\n", __func__);
    DEBUG_LVL_ALWAYS_TRACE("PLK:   Sorry, this operation isn't supported.\n");

    ret = -EINVAL;

    DEBUG_LVL_ALWAYS_TRACE("PLK: - %s() (iRet=%d)\n", __func__, ret);

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  openPOWERLINK zynq driver write function

The function implements openPOWERLINK kernel zynq interface module write function.

\param[in,out]  pFile_p             Pointer to the device file object.
\param[in]      pSrcBuff_p          Pointer to the source buffer, to copy the data to
                                    be written.
\param[in]      buffSize_p          Size of the data to be written.
\param[in,out]  pFileOffs_p         Pointer to the long offset in the file.

\return Returns the amount of data that has been written.

\ingroup module_driver_linux_kernel_zynq
*/
//------------------------------------------------------------------------------
static ssize_t plkIntfWrite(struct file* pFile_p,
                            const char __user* pSrcBuff_p,
                            size_t buffSize_p,
                            loff_t* pFileOffs_p)
{
    int ret;

    UNUSED_PARAMETER(pFile_p);
    UNUSED_PARAMETER(pSrcBuff_p);
    UNUSED_PARAMETER(buffSize_p);
    UNUSED_PARAMETER(pFileOffs_p);

    DEBUG_LVL_ALWAYS_TRACE("PLK: + %s()...\n", __func__);
    DEBUG_LVL_ALWAYS_TRACE("PLK:   Sorry, this operation isn't supported.\n");

    ret = -EINVAL;

    DEBUG_LVL_ALWAYS_TRACE("PLK: - %s() (iRet=%d)\n", __func__, ret);

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  openPOWERLINK zynq driver ioctl function

The function implements openPOWERLINK kernel zynq interface module ioctl function.

\param[in,out]  pInode_p            Pointer to device file data structure.
\param[in,out]  pFile_p             Pointer to the device file object.
\param[in]      cmd_p               Type of the ioctl operation.
\param[in]      arg_p               Argument address for the ioctl operation.

\return The function returns an integer value.
\retval 0                           The ioctl operation was successful.
\retval < 0                         The ioctl operation generated error.

\ingroup module_driver_linux_kernel_zynq
*/
//------------------------------------------------------------------------------
#if defined(HAVE_UNLOCKED_IOCTL)
static long plkIntfIoctl(struct file* pFile_p,
                         unsigned int cmd_p,
                         unsigned long arg_p)
#else
static int plkIntfIoctl(struct inode* pInode_p,
                        struct file* pFile_p,
                        unsigned int cmd_p,
                        unsigned long arg_p)
#endif
{
    int         ret = -EINVAL;
    tOplkError  oplRet;

#if !defined(HAVE_UNLOCKED_IOCTL)
    UNUSED_PARAMETER(pInode_p);
#endif
    UNUSED_PARAMETER(pFile_p);

    DEBUG_LVL_ALWAYS_TRACE("PLK: + %s() (cmd=%d type=%d)...\n",
                           __func__,
                           _IOC_NR(cmd_p),
                           _IOC_TYPE(cmd_p));

    // Add some checks for valid commands here
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
                zynqdrv_regSyncHandler(timesynckcal_sendSyncEvent);
                zynqdrv_enableSync(TRUE);
                instance_l.fSyncEnabled = TRUE;
            }

            //TODO: Handle other errors
            oplRet = timesynckcal_waitSyncEvent();
            if (oplRet == kErrorRetry)
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

        case PLK_CMD_CTRL_WRITE_FILE_BUFFER:
            ret = writeFileBuffer(arg_p);
            break;

        case PLK_CMD_CTRL_GET_FILE_BUFFER_SIZE:
            ret = getFileBufferSize(arg_p);
            break;

#if defined(CONFIG_INCLUDE_SOC_TIME_FORWARD)
        case PLK_CMD_TIMESYNC_MAP_OFFSET:
            ret = getSocTimestampAddress(arg_p);
            break;
#endif

        default:
            DEBUG_LVL_ERROR_TRACE("PLK: - Invalid command (cmd=%d type=%d)\n",
                                  _IOC_NR(cmd_p),
                                  _IOC_TYPE(cmd_p));
            ret = -ENOTTY;
            break;
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  openPOWERLINK zynq driver mmap function

The function implements openPOWERLINK kernel zynq interface module mmap function.

\param[in,out]  pFile_p             Pointer to the device file object.
\param[in,out]  pVmArea_p           Pointer to the virtual memory object of user.

\return The function returns an integer value.

\ingroup module_driver_linux_kernel_zynq
*/
//------------------------------------------------------------------------------
static int plkIntfMmap(struct file* pFile_p,
                       struct vm_area_struct* pVmArea_p)
{
    void*       pPciMem = NULL;
    size_t      memSize = 0;
    tOplkError  ret = kErrorOk;
    void*       pageAddr = NULL;

    UNUSED_PARAMETER(pFile_p);

    DEBUG_LVL_ALWAYS_TRACE("%s() vma: vm_start:%lX vm_end:%lX vm_pgoff:%lX\n",
                           __func__,
                           pVmArea_p->vm_start,
                           pVmArea_p->vm_end,
                           pVmArea_p->vm_pgoff);

    pVmArea_p->vm_flags |= VM_RESERVED | VM_IO;
    pVmArea_p->vm_ops = &powerlinkVmOps_l;

    if (pVmArea_p->vm_pgoff == 0)
    {
        // Get kernel space address for the pdo memory
        ret = drvintf_getPdoMem(&pPciMem, &memSize);

        if ((pPciMem == NULL) ||
            (ret != kErrorOk))
        {
            DEBUG_LVL_ERROR_TRACE("%s() no pdo memory allocated!\n", __func__);
            return -ENOMEM;
        }

        instance_l.pPdoMem = pPciMem;
        instance_l.pdoMemSize = memSize;
        instance_l.pdoVmaStartAddr = pVmArea_p->vm_start;
        // Get the bus address of the pdo memory
        pageAddr = zynqdrv_getMemPhyAddr(kIoMemRegionShared) + (pPciMem - zynqdrv_getMemRegionAddr(kIoMemRegionShared));
    }
    else
    {
        pPciMem = (UINT8*)(pVmArea_p->vm_pgoff << PAGE_SHIFT);
        // Get kernel space address for the passed memory
        ret = drvintf_mapKernelMem(pPciMem,
                                   &pageAddr,
                                   memSize);
        memSize = pVmArea_p->vm_end - pVmArea_p->vm_start;
        if (ret != kErrorOk)
            return -ENOMEM;

        // Get the bus address of the passed memory
        pageAddr = zynqdrv_getMemPhyAddr(kIoMemRegionShared) + (pageAddr - zynqdrv_getMemRegionAddr(kIoMemRegionShared));
    }

    pVmArea_p->vm_pgoff = (ULONG)pageAddr >> PAGE_SHIFT;
    // Save the offset of the mapped memory address from the start of page boundary
    instance_l.bufPageOffset = ((ULONG)pageAddr - (pVmArea_p->vm_pgoff << PAGE_SHIFT));
    // Disable cache of the mapped memory address
    pVmArea_p->vm_page_prot = pgprot_noncached(pVmArea_p->vm_page_prot);

    if (io_remap_pfn_range(pVmArea_p,
                           pVmArea_p->vm_start,
                           pVmArea_p->vm_pgoff,
                           memSize + instance_l.bufPageOffset,
                           pVmArea_p->vm_page_prot))
    {
        DEBUG_LVL_ERROR_TRACE("%s() remap_pfn_range failed\n", __func__);
        return -EAGAIN;
    }

    plkIntfVmaOpen(pVmArea_p);

    return 0;
}

//------------------------------------------------------------------------------
/**
\brief  openPOWERLINK zynq driver VMA open function

The function implements openPOWERLINK kernel module VMA open function.

\param[in,out]  pVmArea_p           Pointer to the virtual memory object of user.

\ingroup module_driver_linux_kernel_zynq
*/
//------------------------------------------------------------------------------
static void plkIntfVmaOpen(struct vm_area_struct* pVmArea_p)
{
    UNUSED_PARAMETER(pVmArea_p);

    DEBUG_LVL_ALWAYS_TRACE("%s() vma: vm_start:%lX vm_end:%lX vm_pgoff:%lX\n",
                           __func__,
                           pVmArea_p->vm_start,
                           pVmArea_p->vm_end,
                           pVmArea_p->vm_pgoff);
}

//------------------------------------------------------------------------------
/**
\brief  openPOWERLINK zynq driver VMA close function

The function implements openPOWERLINK kernel module VMA close function.

\param[in,out]  pVmArea_p           Pointer to the virtual memory object of user.

\ingroup module_driver_linux_kernel_zynq
*/
//------------------------------------------------------------------------------
static void plkIntfVmaClose(struct vm_area_struct* pVmArea_p)
{
    tOplkError  ret = kErrorOk;

    DEBUG_LVL_ALWAYS_TRACE("%s() vma: vm_start:%lX vm_end:%lX vm_pgoff:%lX\n",
                           __func__,
                           pVmArea_p->vm_start,
                           pVmArea_p->vm_end,
                           pVmArea_p->vm_pgoff);

    // Check if it is the PDO memory being freed
    if (instance_l.pdoVmaStartAddr == pVmArea_p->vm_start)
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

\param[in]      arg_p               Ioctl argument. Contains the received event.

\return The function returns Linux error code.
*/
//------------------------------------------------------------------------------
static int getEventForUser(unsigned long arg_p)
{
    int     ret;
    size_t  readSize;
    int     loopCount = (K2U_EVENT_WAIT_TIMEOUT / QUEUE_WAIT_TIMEOUT);
    int     i = 0;

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
                                    __func__,
                                    readSize);
            if (copy_to_user((void __user*)arg_p, instance_l.aK2URxBuffer, readSize))
            {
                DEBUG_LVL_ERROR_TRACE("Event fetch error!!\n");
                ret = -EFAULT;
                break;
            }

            ret = 0;
            break;
        }

        ret = wait_event_interruptible_timeout(instance_l.userWaitQueue,
                                               0,
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

\param[in]      arg_p               Ioctl argument. Contains the event to post.

\return The function returns Linux error code.
*/
//------------------------------------------------------------------------------
static int postEventFromUser(unsigned long arg_p)
{
    tOplkError  ret = kErrorOk;
    tEvent      event;
    UINT8*      pArg = NULL;
    INT         order = 0;

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

\param[in]      arg_p               Control command argument passed by the ioctl interface.

\return The function returns Linux error code.
*/
//------------------------------------------------------------------------------
static int executeCmd(unsigned long arg_p)
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

\param[in]      arg_p               Control module initialization parameters argument
                                    passed by the ioctl interface.

\return The function returns Linux error code.
*/
//------------------------------------------------------------------------------
static int storeInitParam(unsigned long arg_p)
{
    tCtrlInitParam  initParam;

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

\param[in]      arg_p               Pointer to the control module initialization
                                    parameters argument passed by the ioctl
                                    interface.

\return The function returns Linux error code.
*/
//------------------------------------------------------------------------------
static int readInitParam(unsigned long arg_p)
{
    tCtrlInitParam  initParam;

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

\param[in]      arg_p               Pointer to the control status argument passed by the ioctl interface.

\return The function returns Linux error code.
*/
//------------------------------------------------------------------------------
static int getStatus(unsigned long arg_p)
{
    UINT16  status;

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

\param[in]      arg_p               Pointer to the PCP heartbeat argument passed
                                    by the ioctl interface.

\return The function returns Linux error code.
*/
//------------------------------------------------------------------------------
static int getHeartbeat(unsigned long arg_p)
{
    UINT16  heartbeat;

    if (drvintf_getHeartbeat(&heartbeat) != kErrorOk)
        return -EFAULT;

    put_user(heartbeat, (unsigned short __user*)arg_p);

    return 0;
}

//------------------------------------------------------------------------------
/**
\brief  Sending async frame ioctl

The function implements the ioctl used for sending asynchronous frames.

\param[in]      arg_p               Pointer to the async send argument passed by
                                    the ioctl interface.

\return The function returns Linux error code.
*/
//------------------------------------------------------------------------------
static int sendAsyncFrame(unsigned long arg_p)
{
    UINT8*              pBuf;
    tIoctlDllCalAsync   asyncFrameInfo;
    INT                 order;
    int                 ret = 0;

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
    if (drvintf_sendAsyncFrame(asyncFrameInfo.queue,
                               asyncFrameInfo.size,
                               asyncFrameInfo.pData) != kErrorOk)
        ret = -EFAULT;

    free_pages((ULONG)pBuf, order);

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Write error object ioctl

The function implements the ioctl for writing an error object.

\param[in]      arg_p               Pointer to the error object argument passed by
                                    the ioctl interface.

\return The function returns Linux error code.
*/
//------------------------------------------------------------------------------
static int writeErrorObject(unsigned long arg_p)
{
    tErrHndIoctl    errorObject;

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

\param[in]      arg_p               Pointer to the error object argument passed by
                                    the ioctl interface.

\return The function returns Linux error code.
*/
//------------------------------------------------------------------------------
static int readErrorObject(unsigned long arg_p)
{
    tErrHndIoctl    errorObject;

    if (copy_from_user(&errorObject, (const void __user*)arg_p, sizeof(tErrHndIoctl)))
        return -EFAULT;

    if (drvintf_readErrorObject(errorObject.offset, &errorObject.errVal) != kErrorOk)
        return -EFAULT;

    if (copy_to_user((void __user*)arg_p, &errorObject, sizeof(tErrHndIoctl)))
        return -EFAULT;

    return 0;
}

//------------------------------------------------------------------------------
/**
\brief Write file chunk ioctl

The function implements the ioctl for writing the given file chunk to the
file transfer buffer.

\note This operation is not supported on Zynq yet.

\param[in]      arg_p               Pointer to the file chunk argument passed by
                                    the ioctl interface.

\return The function returns Linux error code.
*/
//------------------------------------------------------------------------------
static int writeFileBuffer(unsigned long arg_p)
{
    int                 ret = 0;

    DEBUG_LVL_ALWAYS_TRACE("PLK: + %s()...\n", __func__);
    DEBUG_LVL_ALWAYS_TRACE("PLK:   Sorry, this operation isn't supported.\n");
    ret = -EINVAL;
    DEBUG_LVL_ALWAYS_TRACE("PLK: - %s() (ret=%d)\n", __func__, ret);

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief Get maximum supported file chunk size ioctl

The function implements the ioctl for returning the maximum file chunk size
which is supported by the CAL implementation.

\note This operation is not supported on Zynq yet.

\param[in]      arg_p               Pointer to the file chunk size argument passed
                                    by the ioctl interface.

\return The function returns Linux error code.
*/
//------------------------------------------------------------------------------
static int getFileBufferSize(unsigned long arg_p)
{
    int ret = 0;

    DEBUG_LVL_ALWAYS_TRACE("PLK: + %s()...\n", __func__);
    DEBUG_LVL_ALWAYS_TRACE("PLK:   Sorry, this operation isn't supported.\n");
    ret = -EINVAL;
    DEBUG_LVL_ALWAYS_TRACE("PLK: - %s() (ret=%d)\n", __func__, ret);

    return ret;
}

#if defined(CONFIG_INCLUDE_SOC_TIME_FORWARD)
//------------------------------------------------------------------------------
/**
\brief  Get timesync SoC timestamp kernel address

The function implements the ioctl for getting the timesync shared memory pointer.

\param[in]      arg_p               Pointer to the timesync shared memory argument
                                    passed by the ioctl interface.

\return The function returns Linux error code.
*/
//------------------------------------------------------------------------------
static int getSocTimestampAddress(unsigned long arg_p)
{
    tTimesyncSharedMemory* pSharedMemory;

    // Gets the kernel address of timesync shared memory from timesync kernel CAL
    pSharedMemory = timesynckcal_getSharedMemory();

    if (pSharedMemory == NULL)
        return -ENXIO;

    // Copy the received kernel address to timesync user CAL
    if (copy_to_user((void __user*)arg_p, &pSharedMemory, sizeof(ULONG)))
        return -EFAULT;

    return 0;
}
#endif

/// \}
