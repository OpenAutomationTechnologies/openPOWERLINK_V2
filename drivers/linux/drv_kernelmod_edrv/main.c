/**
********************************************************************************
\file   main.c

\brief  main file for Linux kernel module

This file contains the main part of the Linux kernel module implementation of
the openPOWERLINK kernel stack.

\ingroup module_driver_linux_kernel
*******************************************************************************/
/*------------------------------------------------------------------------------
Copyright (c) 2014, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
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
#include <asm/page.h>
#include <asm/uaccess.h>
#include <asm/page.h>
#include <asm/atomic.h>

#include <oplk/oplk.h>
#include <common/oplkinc.h>
#include <common/driver.h>

#include <common/ctrl.h>
#include <common/ctrlcal-mem.h>
#include <kernel/ctrlk.h>
#include <kernel/ctrlkcal.h>
#include <kernel/dllkcal.h>
#include <kernel/pdokcal.h>
#include <kernel/timesynckcal.h>

#include <kernel/eventk.h>
#include <kernel/eventkcal.h>
#include <errhndkcal.h>

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
#ifndef VM_RESERVED
#define VM_RESERVED   (VM_DONTEXPAND | VM_DONTDUMP)
#endif

//------------------------------------------------------------------------------
// module global vars
//------------------------------------------------------------------------------

int                     plkMajor_g = 0;
int                     plkMinor_g = 0;
int                     plkNrDevs_g = 1;
dev_t                   plkDev_g;
struct class*           plkClass_g;
struct cdev             plkCdev_g;
struct timer_list       heartbeatTimer_g;
tEvent                  event_g;
BOOL                    fEvent_g;
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

//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------

static int  __init  powerlinkInit(void);
static void __exit  powerlinkExit(void);

static int      powerlinkOpen(struct inode* pDeviceFile_p, struct file* pInstance_p);
static int      powerlinkRelease(struct inode* pDeviceFile_p, struct file* pInstance_p);
static ssize_t  powerlinkRead(struct file* pInstance_p, char* pDstBuff_p, size_t BuffSize_p, loff_t* pFileOffs_p);
static ssize_t  powerlinkWrite(struct file* pInstance_p, const char* pSrcBuff_p, size_t BuffSize_p, loff_t* pFileOffs_p);
#ifdef HAVE_UNLOCKED_IOCTL
static  long    powerlinkIoctl(struct file* filp, unsigned int cmd, unsigned long arg);
#else
static int      powerlinkIoctl(struct inode* dev, struct file* filp, unsigned int cmd, unsigned long arg);
#endif

static int      powerlinkMmap(struct file* filp, struct vm_area_struct* vma);
static void     powerlinkVmaOpen(struct vm_area_struct* vma);
static void     powerlinkVmaClose(struct vm_area_struct* vma);

static int      executeCmd(unsigned long arg);
static int      readInitParam(unsigned long arg);
static int      storeInitParam(unsigned long arg);
static int      getStatus(unsigned long arg);
static int      getHeartbeat(unsigned long arg);
static int      sendAsyncFrame(unsigned long arg);
static int      writeErrorObject(unsigned long arg);
static int      readErrorObject(unsigned long arg);

static void     increaseHeartbeatCb(ULONG data_p);
static void     startHeartbeatTimer(ULONG timeInMs_p);
static void     stopHeartbeatTimer(void);

//------------------------------------------------------------------------------
//  Kernel module specific data structures
//------------------------------------------------------------------------------
module_init(powerlinkInit);
module_exit(powerlinkExit);

static struct file_operations  powerlinkFileOps_g =
{
    .owner =     THIS_MODULE,
    .open =      powerlinkOpen,
    .release =   powerlinkRelease,
    .read =      powerlinkRead,
    .write =     powerlinkWrite,
#ifdef HAVE_UNLOCKED_IOCTL
    .unlocked_ioctl = powerlinkIoctl,
#else
    .ioctl =     powerlinkIoctl,
#endif
    .mmap =      powerlinkMmap,
};

static struct vm_operations_struct powerlinkVmOps =
{
        .open = powerlinkVmaOpen,
        .close = powerlinkVmaClose,
};

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//---------------------------------------------------------------------------
//  Initailize driver
//---------------------------------------------------------------------------

//------------------------------------------------------------------------------
/**
\brief  module initialization

The function implements openPOWERLINK kernel module initialization function.

\ingroup module_driver_linux_kernel
*/
//------------------------------------------------------------------------------
static int __init powerlinkInit(void)
{
    int  err;

    DEBUG_LVL_ALWAYS_TRACE("PLK: powerlinkInit()  Driver build: %s / %s\n", __DATE__, __TIME__);
    DEBUG_LVL_ALWAYS_TRACE("PLK: powerlinkInit()  Stack version: %s\n", PLK_DEFINED_STRING_VERSION);
    plkDev_g = 0;
    atomic_set(&openCount_g, 0);

    if ((err = alloc_chrdev_region(&plkDev_g, plkMinor_g, plkNrDevs_g, PLK_DRV_NAME)) < 0)
    {
        DEBUG_LVL_ERROR_TRACE ("PLK: Failing allocating major number\n");
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

The function implements openPOWERLINK kernel module exit function.

\ingroup module_driver_linux_kernel
*/
//------------------------------------------------------------------------------
static void __exit powerlinkExit(void)
{
    DEBUG_LVL_ALWAYS_TRACE("PLK: powerlinkExit...\n");

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

\ingroup module_driver_linux_kernel
*/
//------------------------------------------------------------------------------
static int powerlinkOpen(struct inode* pDeviceFile_p, struct file* pInstance_p)
{
    DEBUG_LVL_ALWAYS_TRACE("PLK: + powerlinkOpen...\n");

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

    DEBUG_LVL_ALWAYS_TRACE("PLK: + powerlinkOpen - OK\n");

    return 0;
}

//------------------------------------------------------------------------------
/**
\brief  openPOWERLINK driver close function

The function implements openPOWERLINK kernel module close function.

\ingroup module_driver_linux_kernel
*/
//------------------------------------------------------------------------------
static int  powerlinkRelease (struct inode* pDeviceFile_p, struct file* pInstance_p)
{
    DEBUG_LVL_ALWAYS_TRACE("PLK: + powerlinkRelease...\n");

    stopHeartbeatTimer();
    ctrlk_exit();
    atomic_dec(&openCount_g);
    DEBUG_LVL_ALWAYS_TRACE("PLK: + powerlinkRelease - OK\n");
    return 0;
}

//------------------------------------------------------------------------------
/**
\brief  openPOWERLINK driver read function

The function implements openPOWERLINK kernel module read function.

\ingroup module_driver_linux_kernel
*/
//------------------------------------------------------------------------------
static ssize_t powerlinkRead(struct file* pInstance_p, char* pDstBuff_p,
                             size_t BuffSize_p, loff_t* pFileOffs_p)
{
    int  ret;

    DEBUG_LVL_ALWAYS_TRACE("PLK: + powerlinkRead...\n");
    DEBUG_LVL_ALWAYS_TRACE("PLK:   Sorry, this operation isn't supported.\n");
    ret = -EINVAL;
    DEBUG_LVL_ALWAYS_TRACE("PLK: - powerlinkRead (iRet=%d)\n", ret);
    return ret;

}


//------------------------------------------------------------------------------
/**
\brief  openPOWERLINK driver write function

The function implements openPOWERLINK kernel module write function.

\ingroup module_driver_linux_kernel
*/
//------------------------------------------------------------------------------
static ssize_t powerlinkWrite(struct file* pInstance_p, const char* pSrcBuff_p,
                              size_t BuffSize_p, loff_t* pFileOffs_p)
{
    int  ret;

    DEBUG_LVL_ALWAYS_TRACE("PLK: + powerlinkWrite...\n");
    DEBUG_LVL_ALWAYS_TRACE("PLK:   Sorry, this operation isn't supported.\n");
    ret = -EINVAL;
    DEBUG_LVL_ALWAYS_TRACE("PLK: - powerlinkWrite (iRet=%d)\n", ret);
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  openPOWERLINK driver ioctl function

The function implements openPOWERLINK kernel module ioctl function.

\ingroup module_driver_linux_kernel
*/
//------------------------------------------------------------------------------
#ifdef HAVE_UNLOCKED_IOCTL
static long powerlinkIoctl(struct file* filp, unsigned int cmd,
                           unsigned long arg)
#else
static int  powerlinkIoctl(struct inode* dev, struct file* filp,
                           unsigned int cmd, unsigned long arg)
#endif
{
    int             ret;
    tOplkError      oplRet;

    //DEBUG_LVL_ALWAYS_TRACE("PLK: + powerlinkIoctl (cmd=%d type=%d)...\n", _IOC_NR(cmd), _IOC_TYPE(cmd));
    ret = -EINVAL;

    // Add some checks for valid commands here

    switch (cmd)
    {
        case PLK_CMD_CTRL_EXECUTE_CMD:
            ret = executeCmd(arg);
            break;

        case PLK_CMD_CTRL_STORE_INITPARAM:
            ret = storeInitParam(arg);
            break;

        case PLK_CMD_CTRL_READ_INITPARAM:
            ret = readInitParam(arg);
            break;

        case PLK_CMD_CTRL_GET_STATUS:
            ret = getStatus(arg);
            break;

        case PLK_CMD_CTRL_GET_HEARTBEAT:
            ret = getHeartbeat(arg);
            break;

        case PLK_CMD_POST_EVENT:
            ret = eventkcal_postEventFromUser(arg);
            break;

        case PLK_CMD_GET_EVENT:
            ret = eventkcal_getEventForUser(arg);
            break;

        case PLK_CMD_DLLCAL_ASYNCSEND:
            ret = sendAsyncFrame(arg);
            break;

        case PLK_CMD_ERRHND_WRITE:
            ret = writeErrorObject(arg);
            break;

        case PLK_CMD_ERRHND_READ:
            ret = readErrorObject(arg);
            break;

        case PLK_CMD_TIMESYNC_SYNC:
            if ((oplRet = timesynckcal_waitSyncEvent()) == kErrorRetry)
                ret = -ERESTARTSYS;
            else
                ret = 0;
            break;

        default:
            DEBUG_LVL_ERROR_TRACE("PLK: - Invalid cmd (cmd=%d type=%d)\n", _IOC_NR(cmd), _IOC_TYPE(cmd));
            ret = -ENOTTY;
            break;
    }

    //TRACE("PLK: - powerlinkIoctl (cmd=%d type=%d)..(ret=%d)\n", _IOC_NR(cmd), _IOC_TYPE(cmd), ret);
    return ret;

}

//------------------------------------------------------------------------------
/**
\brief  openPOWERLINK driver mmap function

The function implements openPOWERLINK kernel module mmap function.

\ingroup module_driver_linux_kernel
*/
//------------------------------------------------------------------------------
static int powerlinkMmap(struct file* filp, struct vm_area_struct* vma)
{
    BYTE*       pPdoMem;
    tOplkError  ret = kErrorOk;

    DEBUG_LVL_ALWAYS_TRACE("%s() vma: vm_start:%lX vm_end:%lX vm_pgoff:%lX\n",
                           __func__, vma->vm_start, vma->vm_end, vma->vm_pgoff);

    vma->vm_flags |= VM_RESERVED;
    vma->vm_ops = &powerlinkVmOps;

    ret = pdokcal_getPdoMemRegion(&pPdoMem, NULL);

    if (ret != kErrorOk || pPdoMem == NULL)
    {
        DEBUG_LVL_ERROR_TRACE("%s() no pdo memory allocated!\n", __func__);
        return -ENOMEM;
    }

    if (remap_pfn_range(vma, vma->vm_start, (__pa(pPdoMem) >> PAGE_SHIFT),
                        vma->vm_end - vma->vm_start, vma->vm_page_prot))
    {
        DEBUG_LVL_ERROR_TRACE("%s() remap_pfn_range failed\n", __func__);
        return -EAGAIN;
    }

    powerlinkVmaOpen(vma);
    return 0;

}

//------------------------------------------------------------------------------
/**
\brief  openPOWERLINK driver VMA open functionnet

The function implements openPOWERLINK kernel module VMA open function.

\ingroup module_driver_linux_kernel
*/
//------------------------------------------------------------------------------
static void powerlinkVmaOpen(struct vm_area_struct* vma)
{
    DEBUG_LVL_ALWAYS_TRACE("%s() vma: vm_start:%lX vm_end:%lX vm_pgoff:%lX\n",
                           __func__, vma->vm_start, vma->vm_end, vma->vm_pgoff);
}

//------------------------------------------------------------------------------
/**TRACE
\brief  openPOWERLINK driver VMA close function

The function implements openPOWERLINK kernel module VMA close function.

\ingroup module_driver_linux_kernel
*/
//------------------------------------------------------------------------------
static void powerlinkVmaClose(struct vm_area_struct* vma)
{
    DEBUG_LVL_ALWAYS_TRACE("%s() vma: vm_start:%lX vm_end:%lX vm_pgoff:%lX\n",
                           __func__, vma->vm_start, vma->vm_end, vma->vm_pgoff);
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
module using the ioctl interface..

\ingroup module_driver_linux_kernel
*/
//------------------------------------------------------------------------------
static int executeCmd(unsigned long arg)
{
    tCtrlCmd            ctrlCmd;
    UINT16              ret;
    tCtrlKernelStatus   status;

    if (copy_from_user(&ctrlCmd, (const void __user *)arg, sizeof(tCtrlCmd)))
        return -EFAULT;

    ctrlk_executeCmd(ctrlCmd.cmd, &ret, &status, NULL);
    ctrlCmd.cmd = 0;
    ctrlCmd.retVal = ret;
    ctrlkcal_setStatus(status);

    if (copy_to_user((void __user *)arg, &ctrlCmd, sizeof(tCtrlCmd)))
        return -EFAULT;

    return 0;
}

//------------------------------------------------------------------------------
/**
\brief  Store init param ioctl

The function implements the calling of the storeInitParam function in the
control module using the ioctl interface..

\ingroup module_driver_linux_kernel
*/
//------------------------------------------------------------------------------
static int storeInitParam(unsigned long arg)
{
    tCtrlInitParam      initParam;

    if (copy_from_user(&initParam, (const void __user *)arg, sizeof(tCtrlInitParam)))
        return -EFAULT;

    ctrlkcal_storeInitParam(&initParam);

    return 0;
}

//------------------------------------------------------------------------------
/**
\brief  Read init param ioctl

The function implements the calling of the readInitParam function in the control
module using the ioctl interface..

\ingroup module_driver_linux_kernel
*/
//------------------------------------------------------------------------------
static int readInitParam(unsigned long arg)
{
    tCtrlInitParam      initParam;

    ctrlkcal_readInitParam(&initParam);
    if (copy_to_user((void __user *)arg, &initParam, sizeof(tCtrlInitParam)))
        return -EFAULT;

    return 0;
}

//------------------------------------------------------------------------------
/**
\brief  Get status ioctl

The function implements the calling of the getStatus function in the control
module using the ioctl interface..

\ingroup module_driver_linux_kernel
*/
//------------------------------------------------------------------------------
static int getStatus(unsigned long arg)
{
    UINT16      status;

    status = ctrlkcal_getStatus();
    put_user(status, (unsigned short __user *)arg);
    return 0;
}

//------------------------------------------------------------------------------
/**
\brief  Get heartbeat ioctl

The function implements the calling of the getHeartbeat function in the control
module using the ioctl interface.

\ingroup module_driver_linux_kernel
*/
//------------------------------------------------------------------------------
static int getHeartbeat(unsigned long arg)
{
    UINT16      heartbeat;

    heartbeat = ctrlk_getHeartbeat();
    put_user(heartbeat, (unsigned short __user *)arg);
    return 0;
}

//------------------------------------------------------------------------------
/**
\brief  Sending async frame ioctl

The function implements the ioctl used for sending asynchronous frames.

\ingroup module_driver_linux_kernel
*/
//------------------------------------------------------------------------------
static int sendAsyncFrame(unsigned long arg)
{
    BYTE*               pBuf;
    tIoctlDllCalAsync   asyncFrameInfo;
    tFrameInfo          frameInfo;
    int                 order;

    order = get_order(C_DLL_MAX_ASYNC_MTU);
    pBuf = (BYTE*)__get_free_pages(GFP_KERNEL, order);

    if (copy_from_user(&asyncFrameInfo, (const void __user *)arg, sizeof(tIoctlDllCalAsync)))
    {
        free_pages((ULONG)pBuf, order);
        return -EFAULT;
    }

    if (copy_from_user(pBuf, (const void __user *)asyncFrameInfo.pData, asyncFrameInfo.size))
    {
        free_pages((ULONG)pBuf, order);
        return -EFAULT;
    }

    //TRACE("%s() Received frame size:%d\n", __func__, asyncFrame.size);
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

\ingroup module_driver_linux_kernel
*/
//------------------------------------------------------------------------------
static int writeErrorObject(unsigned long arg)
{
    tErrHndIoctl    writeObject;
    tErrHndObjects* errorObjects;

    if (copy_from_user(&writeObject, (const void __user *)arg, sizeof(tErrHndIoctl)))
        return -EFAULT;

    errorObjects = errhndkcal_getMemPtr();
    *((char*)errorObjects + writeObject.offset) = writeObject.errVal;

    return 0;
}

//------------------------------------------------------------------------------
/**
\brief  Read error object ioctl

The function implements the ioctl for reading error objects

\ingroup module_driver_linux_kernel
*/
//------------------------------------------------------------------------------
static int readErrorObject(unsigned long arg)
{
    tErrHndIoctl    readObject;
    tErrHndObjects* errorObjects;

    if (copy_from_user(&readObject, (const void __user *)arg, sizeof(tErrHndIoctl)))
        return -EFAULT;

    errorObjects = errhndkcal_getMemPtr();
    readObject.errVal = *((char*)errorObjects + readObject.offset);

    if (copy_to_user((void __user *)arg, &readObject, sizeof(tErrHndIoctl)))
        return -EFAULT;

    return 0;
}

//------------------------------------------------------------------------------
/**
\brief  Start heartbeat timer

The function starts the timer used for updating the heartbeat counter.

\param  timeInMs_p          Timeout value in milliseconds

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

\param  data_p          Not used, need for timer interface

\ingroup module_driver_linux_kernel
*/
//------------------------------------------------------------------------------
void increaseHeartbeatCb(ULONG data_p)
{
    UNUSED_PARAMETER(data_p);
    ctrlk_updateHeartbeat();
    startHeartbeatTimer(20);
}

///\}
