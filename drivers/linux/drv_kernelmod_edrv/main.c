/**
********************************************************************************
\file   main.c

\brief  main file for Linux kernel module

This file contains the main part of the Linux kernel module implementation of
the openPOWERLINK kernel stack.

\ingroup module_driver_linux_kernel
*******************************************************************************/
/*------------------------------------------------------------------------------
Copyright (c) 2016, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
Copyright (c) 2017, Kalycito Infotech Private Limited
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
#include <common/driver.h>
#include <common/ctrl.h>
#include <common/ctrlcal-mem.h>
#include <kernel/ctrlk.h>
#include <kernel/ctrlkcal.h>
#include <kernel/dllkcal.h>
#include <kernel/pdokcal.h>
#include <kernel/timesynckcal.h>
#include <kernel/eventkcal.h>
#include <kernel/errhndk.h>

#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/version.h>
#include <linux/uaccess.h>
#include <linux/mm.h>
#include <asm/uaccess.h>
#include <asm/page.h>
#include <asm/atomic.h>


//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
MODULE_LICENSE("Dual BSD/GPL");
MODULE_AUTHOR("josef.baumgartner@br-automation.com");
MODULE_DESCRIPTION("openPOWERLINK driver");

// VM_RESERVED is removed in kernels > 3.7
#if !defined(VM_RESERVED)
#define VM_RESERVED     (VM_DONTEXPAND | VM_DONTDUMP)
#endif

//------------------------------------------------------------------------------
// module global vars
//------------------------------------------------------------------------------
int                 plkMajor_g = 0;
int                 plkMinor_g = 0;
int                 plkNrDevs_g = 1;
dev_t               plkDev_g;
struct class*       plkClass_g;
struct cdev         plkCdev_g;
struct timer_list   heartbeatTimer_g;
atomic_t            openCount_g;

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

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static int __init   powerlinkInit(void);
static void __exit  powerlinkExit(void);

static int          powerlinkOpen(struct inode* pInode_p,
                                  struct file* pFile_p);
static int          powerlinkRelease(struct inode* pInode_p,
                                     struct file* pFile_p);
static ssize_t      powerlinkRead(struct file* pFile_p,
                                  char __user* pDstBuff_p,
                                  size_t buffSize_p,
                                  loff_t* pFileOffs_p);
static ssize_t      powerlinkWrite(struct file* pFile_p,
                                   const char __user* pSrcBuff_p,
                                   size_t buffSize_p,
                                   loff_t* pFileOffs_p);
#ifdef HAVE_UNLOCKED_IOCTL
static long         powerlinkIoctl(struct file* pFile_p,
                                   unsigned int cmd_p,
                                   unsigned long arg_p);
#else
static int          powerlinkIoctl(struct inode* pInode_p,
                                   struct file* pFile_p,
                                   unsigned int cmd_p,
                                   unsigned long arg_p);
#endif
static int          powerlinkMmap(struct file* pFile_p,
                                  struct vm_area_struct* pVmArea_p);

static void         powerlinkVmaOpen(struct vm_area_struct* pVmArea_p);
static void         powerlinkVmaClose(struct vm_area_struct* pVmArea_p);

static int          executeCmd(unsigned long arg_p);
static int          readInitParam(unsigned long arg_p);
static int          storeInitParam(unsigned long arg_p);
static int          getStatus(unsigned long arg_p);
static int          getHeartbeat(unsigned long arg_p);
static int          sendAsyncFrame(unsigned long arg_p);
static int          writeErrorObject(unsigned long arg_p);
static int          readErrorObject(unsigned long arg_p);

static void         increaseHeartbeatCb(ULONG data_p);
static void         startHeartbeatTimer(ULONG timeInMs_p);
static void         stopHeartbeatTimer(void);

//------------------------------------------------------------------------------
//  Kernel module specific data structures
//------------------------------------------------------------------------------
module_init(powerlinkInit);             // Define kernel module init function
module_exit(powerlinkExit);             // Define kernel module exit function

static struct file_operations   powerlinkFileOps_l =
{
    .owner          = THIS_MODULE,
    .open           = powerlinkOpen,
    .release        = powerlinkRelease,
    .read           = powerlinkRead,
    .write          = powerlinkWrite,
#if defined(HAVE_UNLOCKED_IOCTL)
    .unlocked_ioctl = powerlinkIoctl,
#else
    .ioctl          = powerlinkIoctl,
#endif
    .mmap           = powerlinkMmap,
};

static struct vm_operations_struct  powerlinkVmOps_l =
{
    .open  = powerlinkVmaOpen,
    .close = powerlinkVmaClose,
};

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  module initialization

The function implements openPOWERLINK kernel module initialization function.

\return Returns an exit code

\ingroup module_driver_linux_kernel
*/
//------------------------------------------------------------------------------
static int __init powerlinkInit(void)
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
        return -1;
    }

    if (device_create(plkClass_g, NULL, plkDev_g, NULL, PLK_DRV_NAME) == NULL)
    {
        DEBUG_LVL_ERROR_TRACE("device_create() failed!\n");
        class_destroy(plkClass_g);
        unregister_chrdev_region(plkDev_g, plkNrDevs_g);
        return -1;
    }

    cdev_init(&plkCdev_g, &powerlinkFileOps_l);
    err = cdev_add(&plkCdev_g, plkDev_g, 1);
    if (err == -1)
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

The function implements openPOWERLINK kernel module exit function.

\ingroup module_driver_linux_kernel
*/
//------------------------------------------------------------------------------
static void __exit powerlinkExit(void)
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
\brief  openPOWERLINK driver open function

The function implements openPOWERLINK kernel module open function.

\param[in,out]  pInode_p            Pointer to inode data structure.
\param[in,out]  pFile_p             Pointer to the device file object.

\return The function returns an integer value.
\retval 0                           Successful.
\retval -ENOTTY                     One instance of the driver is already active.
\retval -EIO                        Unable to initialize the driver.

\ingroup module_driver_linux_kernel
*/
//------------------------------------------------------------------------------
static int powerlinkOpen(struct inode* pInode_p,
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

    init_timer(&heartbeatTimer_g);

    if (ctrlk_init(NULL) != kErrorOk)
    {
        atomic_dec(&openCount_g);
        return -EIO;
    }

    startHeartbeatTimer(20);

    DEBUG_LVL_ALWAYS_TRACE("PLK: + %s() - OK\n", __func__);

    return 0;
}

//------------------------------------------------------------------------------
/**
\brief  openPOWERLINK driver close function

The function implements openPOWERLINK kernel module close function.

\param[in,out]  pInode_p            Pointer to the inode data structure.
\param[in,out]  pFile_p             Pointer to the device file object.

\return The function returns an integer value.
\retval 0                           Successful.

\ingroup module_driver_linux_kernel
*/
//------------------------------------------------------------------------------
static int powerlinkRelease(struct inode* pInode_p,
                            struct file* pFile_p)
{
    tCtrlKernelStatus   status;
    UINT16              retVal;

    UNUSED_PARAMETER(pInode_p);
    UNUSED_PARAMETER(pFile_p);

    DEBUG_LVL_ALWAYS_TRACE("PLK: + %s()...\n", __func__);

    stopHeartbeatTimer();

    // Close lower driver resources
    status = ctrlkcal_getStatus();
    if (status == kCtrlStatusRunning)
    {
        ctrlk_executeCmd(kCtrlShutdown, &retVal, &status, NULL);
    }

    ctrlk_exit();
    atomic_dec(&openCount_g);

    DEBUG_LVL_ALWAYS_TRACE("PLK: + %s() - OK\n", __func__);

    return 0;
}

//------------------------------------------------------------------------------
/**
\brief  openPOWERLINK driver read function

The function implements openPOWERLINK kernel module read function.

\param[in,out]  pFile_p             Pointer to the device file object.
\param[out]     pDstBuff_p          Pointer to the destination buffer, to copy the read data.
\param[in]      buffSize_p          Size of the destination buffer.
\param[in,out]  pFileOffs_p         Pointer to the long offset in the file.

\return Returns the amount of data that has been read.

\ingroup module_driver_linux_kernel
*/
//------------------------------------------------------------------------------
static ssize_t powerlinkRead(struct file* pFile_p,
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

    DEBUG_LVL_ALWAYS_TRACE("PLK: - %s() (ret=%d)\n", __func__, ret);

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  openPOWERLINK driver write function

The function implements openPOWERLINK kernel module write function.

\param[in,out]  pFile_p             Pointer to the device file object.
\param[in]      pSrcBuff_p          Pointer to the source buffer, to copy the data to
                                    be written.
\param[in]      buffSize_p          Size of the data to be written.
\param[in,out]  pFileOffs_p         Pointer to the long offset in the file.

\return Returns the amount of data that has been written.

\ingroup module_driver_linux_kernel
*/
//------------------------------------------------------------------------------
static ssize_t powerlinkWrite(struct file* pFile_p,
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
\brief  openPOWERLINK driver ioctl function

The function implements openPOWERLINK kernel module ioctl function.

\param[in,out]  pInode_p            Pointer to the inode data structure.
\param[in,out]  pFile_p             Pointer to the device file object.
\param[in]      cmd_p               Type of the ioctl operation.
\param[in]      arg_p               Argument address for the ioctl operation.

\return The function returns an integer value.
\retval 0                           The ioctl operation was successful.
\retval < 0                         The ioctl operation generated error.

\ingroup module_driver_linux_kernel
*/
//------------------------------------------------------------------------------
#if defined(HAVE_UNLOCKED_IOCTL)
static long powerlinkIoctl(struct file* pFile_p,
                           unsigned int cmd_p,
                           unsigned long arg_p)
#else
static int  powerlinkIoctl(struct inode* pInode_p,
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
            ret = eventkcal_postEventFromUser(arg_p);
            break;

        case PLK_CMD_GET_EVENT:
            ret = eventkcal_getEventForUser(arg_p);
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
            oplRet = timesynckcal_waitSyncEvent();
            if (oplRet == kErrorRetry)
                ret = -ERESTARTSYS;
            else
                ret = 0;
            break;

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
\brief  openPOWERLINK driver mmap function

The function implements openPOWERLINK kernel module mmap function.

\param[in,out]  pFile_p             Pointer to the device file object.
\param[in,out]  pVmArea_p           Pointer to the virtual memory object of user.

\return The function returns an integer value.

\ingroup module_driver_linux_kernel
*/
//------------------------------------------------------------------------------
static int powerlinkMmap(struct file* pFile_p,
                         struct vm_area_struct* pVmArea_p)
{
    BYTE*                        pPdoMem;
#if defined(CONFIG_INCLUDE_SOC_TIME_FORWARD)
    tTimesyncSharedMemory*       pSocTimeMem;
#endif
    tOplkError  ret = kErrorOk;

    UNUSED_PARAMETER(pFile_p);

    DEBUG_LVL_ALWAYS_TRACE("%s() vma: vm_start:%lX vm_end:%lX vm_pgoff:%lX\n",
                           __func__,
                           pVmArea_p->vm_start,
                           pVmArea_p->vm_end,
                           pVmArea_p->vm_pgoff);

    pVmArea_p->vm_flags |= VM_RESERVED;
    pVmArea_p->vm_ops = &powerlinkVmOps_l;
#if defined(CONFIG_INCLUDE_SOC_TIME_FORWARD)
    if (pVmArea_p->vm_pgoff == 0)
    {
#endif
        ret = pdokcal_getPdoMemRegion(&pPdoMem, NULL);

        if ((ret != kErrorOk) || (pPdoMem == NULL))
        {
            DEBUG_LVL_ERROR_TRACE("%s() no PDO memory allocated!\n", __func__);
            return -ENOMEM;
        }

        if (remap_pfn_range(pVmArea_p,
                            pVmArea_p->vm_start,
                            (__pa(pPdoMem) >> PAGE_SHIFT),
                            pVmArea_p->vm_end - pVmArea_p->vm_start,
                            pVmArea_p->vm_page_prot))
        {
            DEBUG_LVL_ERROR_TRACE("%s() remap_pfn_range failed for PDO memory\n",
                                  __func__);
            return -EAGAIN;
        }
#if defined(CONFIG_INCLUDE_SOC_TIME_FORWARD)
    }
    else
    {
        pSocTimeMem = timesynckcal_getSharedMemory();
        if (pSocTimeMem == NULL)
        {
            DEBUG_LVL_ERROR_TRACE("%s() no timesync memory allocated!\n", __func__);
            return -ENOMEM;
        }

        if (remap_pfn_range(pVmArea_p,
                            pVmArea_p->vm_start,
                            (__pa(pSocTimeMem) >> PAGE_SHIFT),
                            pVmArea_p->vm_end - pVmArea_p->vm_start,
                            pVmArea_p->vm_page_prot))
        {
            DEBUG_LVL_ERROR_TRACE("%s() remap_pfn_range failed for timesync memory\n",
                                  __func__);
            return -EAGAIN;
        }

    }
#endif

    powerlinkVmaOpen(pVmArea_p);

    return 0;
}

//------------------------------------------------------------------------------
/**
\brief  openPOWERLINK driver VMA open function

The function implements openPOWERLINK kernel module VMA open function.

\param[in,out]  pVmArea_p           Pointer to the virtual memory object of user.

\ingroup module_driver_linux_kernel
*/
//------------------------------------------------------------------------------
static void powerlinkVmaOpen(struct vm_area_struct* pVmArea_p)
{
    DEBUG_LVL_ALWAYS_TRACE("%s() vma: vm_start:%lX vm_end:%lX vm_pgoff:%lX\n",
                           __func__,
                           pVmArea_p->vm_start,
                           pVmArea_p->vm_end,
                           pVmArea_p->vm_pgoff);
}

//------------------------------------------------------------------------------
/**
\brief  openPOWERLINK driver VMA close function

The function implements openPOWERLINK kernel module VMA close function.

\param[in,out]  pVmArea_p           Pointer to the virtual memory object of user.

\ingroup module_driver_linux_kernel
*/
//------------------------------------------------------------------------------
static void powerlinkVmaClose(struct vm_area_struct* pVmArea_p)
{
    DEBUG_LVL_ALWAYS_TRACE("%s() vma: vm_start:%lX vm_end:%lX vm_pgoff:%lX\n",
                           __func__,
                           pVmArea_p->vm_start,
                           pVmArea_p->vm_end,
                           pVmArea_p->vm_pgoff);
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief  Execute control command ioctl

The function implements the calling of the executeCmd function in the control
module using the ioctl interface.

\param[in]      arg_p               Arguments given along with the ioctl command.

\return Returns an error code.

\ingroup module_driver_linux_kernel
*/
//------------------------------------------------------------------------------
static int executeCmd(unsigned long arg_p)
{
    tCtrlCmd            ctrlCmd;
    UINT16              ret;
    tCtrlKernelStatus   status;

    if (copy_from_user(&ctrlCmd, (const void __user*)arg_p, sizeof(tCtrlCmd)))
        return -EFAULT;

    ctrlk_executeCmd(ctrlCmd.cmd, &ret, &status, NULL);
    ctrlCmd.cmd = 0;
    ctrlCmd.retVal = ret;
    ctrlkcal_setStatus(status);

    if (copy_to_user((void __user*)arg_p, &ctrlCmd, sizeof(tCtrlCmd)))
        return -EFAULT;

    return 0;
}

//------------------------------------------------------------------------------
/**
\brief  Store init param ioctl

The function implements the calling of the storeInitParam function in the
control module using the ioctl interface..

\param[in]      arg_p               Arguments given along with the ioctl command.

\return Returns an error code.

\ingroup module_driver_linux_kernel
*/
//------------------------------------------------------------------------------
static int storeInitParam(unsigned long arg_p)
{
    tCtrlInitParam  initParam;

    if (copy_from_user(&initParam, (const void __user*)arg_p, sizeof(tCtrlInitParam)))
        return -EFAULT;

    ctrlkcal_storeInitParam(&initParam);

    return 0;
}

//------------------------------------------------------------------------------
/**
\brief  Read init param ioctl

The function implements the calling of the readInitParam function in the control
module using the ioctl interface.

\param[in]      arg_p               Arguments given along with the ioctl command.

\return Returns an error code.

\ingroup module_driver_linux_kernel
*/
//------------------------------------------------------------------------------
static int readInitParam(unsigned long arg_p)
{
    tCtrlInitParam  initParam;

    ctrlkcal_readInitParam(&initParam);
    if (copy_to_user((void __user*)arg_p, &initParam, sizeof(tCtrlInitParam)))
        return -EFAULT;

    return 0;
}

//------------------------------------------------------------------------------
/**
\brief  Get status ioctl

The function implements the calling of the getStatus function in the control
module using the ioctl interface.

\param[in]      arg_p               Arguments given along with the ioctl command.

\return Returns an error code.

\ingroup module_driver_linux_kernel
*/
//------------------------------------------------------------------------------
static int getStatus(unsigned long arg_p)
{
    UINT16  status;

    status = ctrlkcal_getStatus();
    put_user(status, (unsigned short __user*)arg_p);

    return 0;
}

//------------------------------------------------------------------------------
/**
\brief  Get heartbeat ioctl

The function implements the calling of the getHeartbeat function in the control
module using the ioctl interface.

\param[in]      arg_p               Arguments given along with the ioctl command.

\return Returns an error code.

\ingroup module_driver_linux_kernel
*/
//------------------------------------------------------------------------------
static int getHeartbeat(unsigned long arg_p)
{
    UINT16  heartbeat;

    heartbeat = ctrlk_getHeartbeat();
    put_user(heartbeat, (unsigned short __user*)arg_p);

    return 0;
}

//------------------------------------------------------------------------------
/**
\brief  Sending async frame ioctl

The function implements the ioctl used for sending asynchronous frames.

\param[in]      arg_p               Arguments given along with the ioctl command.

\return Returns an error code.

\ingroup module_driver_linux_kernel
*/
//------------------------------------------------------------------------------
static int sendAsyncFrame(unsigned long arg_p)
{
    BYTE*               pBuf;
    tIoctlDllCalAsync   asyncFrameInfo;
    tFrameInfo          frameInfo;
    int                 order;

    order = get_order(C_DLL_MAX_ASYNC_MTU);
    pBuf = (BYTE*)__get_free_pages(GFP_KERNEL, order);

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

    DEBUG_LVL_ALWAYS_TRACE("%s() Received frame size:%d\n", __func__, asyncFrameInfo.size);
    frameInfo.frame.pBuffer = (tPlkFrame*)pBuf;
    frameInfo.frameSize = asyncFrameInfo.size;

    dllkcal_writeAsyncFrame(&frameInfo, asyncFrameInfo.queue);

    free_pages((ULONG)pBuf, order);

    return 0;
}

//------------------------------------------------------------------------------
/**
\brief  Write error object ioctl

The function implements the ioctl for writing an error object.

\param[in]      arg_p               Arguments given along with the ioctl command.

\return Returns an error code.

\ingroup module_driver_linux_kernel
*/
//------------------------------------------------------------------------------
static int writeErrorObject(unsigned long arg_p)
{
    tErrHndIoctl    writeObject;
    tErrHndObjects* errorObjects;

    if (copy_from_user(&writeObject, (const void __user*)arg_p, sizeof(tErrHndIoctl)))
        return -EFAULT;

    errorObjects = errhndk_getMemPtr();
    *((char*)errorObjects + writeObject.offset) = writeObject.errVal;

    return 0;
}

//------------------------------------------------------------------------------
/**
\brief  Read error object ioctl

The function implements the ioctl for reading error objects

\param[in]      arg_p               Arguments given along with the ioctl command.

\return Returns an error code.

\ingroup module_driver_linux_kernel
*/
//------------------------------------------------------------------------------
static int readErrorObject(unsigned long arg_p)
{
    tErrHndIoctl    readObject;
    tErrHndObjects* errorObjects;

    if (copy_from_user(&readObject, (const void __user*)arg_p, sizeof(tErrHndIoctl)))
        return -EFAULT;

    errorObjects = errhndk_getMemPtr();
    readObject.errVal = *((char*)errorObjects + readObject.offset);

    if (copy_to_user((void __user*)arg_p, &readObject, sizeof(tErrHndIoctl)))
        return -EFAULT;

    return 0;
}

//------------------------------------------------------------------------------
/**
\brief  Start heartbeat timer

The function starts the timer used for updating the heartbeat counter.

\param[in]      timeInMs_p          Timeout value in milliseconds

\ingroup module_driver_linux_kernel
*/
//------------------------------------------------------------------------------
void startHeartbeatTimer(ULONG timeInMs_p)
{
    heartbeatTimer_g.function = increaseHeartbeatCb;
    heartbeatTimer_g.data = 0;
    heartbeatTimer_g.expires = jiffies + (timeInMs_p * HZ / 1000);
    add_timer(&heartbeatTimer_g);
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
    del_timer(&heartbeatTimer_g);
}

//------------------------------------------------------------------------------
/**
\brief  Increase heartbeat

The function implements the timer callback function used to increase the
heartbeat counter.

\param[in]      data_p              Not used, need for timer interface

\ingroup module_driver_linux_kernel
*/
//------------------------------------------------------------------------------
void increaseHeartbeatCb(ULONG data_p)
{
    UNUSED_PARAMETER(data_p);

    ctrlk_updateHeartbeat();
    startHeartbeatTimer(20);
}

/// \}
