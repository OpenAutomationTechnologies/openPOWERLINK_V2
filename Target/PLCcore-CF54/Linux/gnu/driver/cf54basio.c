/****************************************************************************

  (c) SYSTEC electronic GmbH, D-07973 Greiz, August-Bebel-Str. 29
      www.systec-electronic.com

  Project:      Board driver for SYSTEC ECUcore-5484

  Description:  Implementation of Board driver for SYSTEC ECUcore-5484
                as loadable module

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

    PCB 4160.0      Basic I/O Board (Prototype)


    PCB 4158.1      Extended I/O Board
                    CS1 (PLD):          31-30   Run/Stop/MRes-Switch
                                        29      LED    D0 (DO0)
                                        28      Button S0 (DI0)
                                        27      Reset EthernetSwitch
                                        26-0    not used

                    CS3 (Periphery):    31-28   not used
                                        27-24   LED D4-D1 (DO4-DO1) (JP302 = 1-2)
                                        23-20   Button S4-S1 (DI4-DI1)
                                        19-16   PCB Version
                                        15-8    DIP-Switch
                                         7-0    HEX-Switch

                    JP302:  1-2     ->  LED D1-D4 (DO1-DO4) via CS3 (Periphery)
                            2-3     ->  LED D1-D4 (DO1-DO4) via PCI_IO1 ... PCI_IO4

                    JP300:  open    ->  Button S1-S4 (DI1-DI4) only via CS3 (Periphery)
                            closed  ->  Button S1-S4 (DI1-DI4) additionaly to
                                        CS3 (Periphery) also via PCI_IO5, PCI_IO6,
                                        TIN1 and /IRQ7

                    NOTE:   Button S1-S4 (DI1-DI4) are always readable via
                            CS3 (Periphery), independend from JP300

  -------------------------------------------------------------------------

    ToDo:   - Run/Stop-Switch entprellen
            - PROC-FS: CF54GetHardwareInfo und aktuelle I/O-Werte ausgeben

  -------------------------------------------------------------------------

  2006/09/27 -rs:   Initial Version
  2006/10/01 -rs:   Support for I/O board PCB 4160.0
  2006/10/03 -rs:   Support for I/O board PCB 4158.1
  2007/02/19 -rs:   Support for I/O board PCB 4158.1 with PLD
  2008/09/10 -rs:   V3.00 Adaption to I/O board PCB 4158.5 and CPU PLD V3

****************************************************************************/


#include <linux/version.h>
#include <linux/config.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/major.h>
#include <linux/devfs_fs_kernel.h>
#include <linux/proc_fs.h>

#include <asm/io.h>
#include <asm/segment.h>
#include <asm/system.h>
#include <asm/irq.h>
#include <asm/uaccess.h>

#include <linux/mm.h>
#include <linux/signal.h>
#include <linux/timer.h>
#include <linux/miscdevice.h>
#include <linux/ioport.h>

#include <asm/coldfire.h>
#include <asm/m5485sim.h>
#include <asm/m5485gpio.h>
#include <asm/m5485gpt.h>

#include "../../../../../Include/global.h"
#include "cf54drv.h"
#include "cf54def.h"


#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0)
    // remove ("make invisible") obsolete symbols for kernel versions 2.6
    // and higher
    #define MOD_INC_USE_COUNT
    #define MOD_DEC_USE_COUNT
    #define EXPORT_NO_SYMBOLS
#else
    #error "This driver needs a 2.6.x kernel or higher"
#endif





/***************************************************************************/
/*                                                                         */
/*                                                                         */
/*          G L O B A L   D E F I N I T I O N S                            */
/*                                                                         */
/*                                                                         */
/***************************************************************************/

//---------------------------------------------------------------------------
//  Configuration
//---------------------------------------------------------------------------

//#define PCB_VER                     4160-0

#define PCB_VER                     4158-1
#define CPU_PCB_VER     4152                // PCB number CPU board
#define IO_PCB_VER      4158                // PCB number base board

    // JP300 = open   (DI1-DI4 via CS3/Periphery)
    // JP300 = closed (DI1-DI4 via PCI_IO5, PCI_IO6, TIN1 and /IRQ7)
    // #define PCB_4158_DI1_4_VIA_CS3         1  // if #defined -> JP300 can be opend

    // JP302 = 1-2    (DO1-DO4 via CS3/Periphery)
    // JP302 = 2-3    (DO1-DO4 via PCI_IO1 ... PCI_IO4)
    // #define PCB_4158_DO1_4_VIA_CS3         1  // if #defined -> JP302 must be set to 1-2


#define RSM_SWITCH_DEBOUNCE_TIME1   500 // debounce time for STOP -> RUN in [ms]
#define RSM_SWITCH_DEBOUNCE_TIME2  1000 // debounce time for MRES -> RUN in [ms]


#define _CFG_DYNMAJOR_                  // enable dyn. major number support
#define _CFG_DEVFS_                     // enable DEV-FS  support
#define _CFG_PROCFS_                    // enable PROC-FS support


#define SYSTEC_GENERIC_DRIVER_MAJOR 124

#define DRV_VER_MAIN 1                  // version 1.xx
#define DRV_VER_REL  0                  // version x.00
#define DRV_NAME     "SYSTEC_cf54drv"   // used for <register_chrdev>
#define DEV_NAME     "cf54drv"          // used for "/dev" and "/proc" entry

#ifndef _CFG_DYNMAJOR_
    #define DRV_MAJOR   SYSTEC_GENERIC_DRIVER_MAJOR
#endif


#define PLC_CORE_PLD_TYPE_ID    0x01        // PLD type ID for PLCcore functionality (B0-B3 of PLD Version Register)

#define CS1_IO_BASE_ADDR    0xE4000000  // base address for periphery at CS1
#define CS1_IO_MEM_SIZE     0x00000100  // memory size for periphery at CS1 (PLD)

#define CS3_IO_BASE_ADDR    0xE4010000  // base address for periphery at CS3
#define CS3_IO_MEM_SIZE     0x00000004  // memory size for periphery at CS3


// Metainformation
/* MODULE_LICENSE("GPL"); */
#ifdef MODULE_AUTHOR
    MODULE_AUTHOR("Ronald.Sieber@systec-electronic.com");
    MODULE_DESCRIPTION("I/O driver module for SYSTEC PLCcore-5484");
#endif



//---------------------------------------------------------------------------
//  Constant definitions
//---------------------------------------------------------------------------

// PLD register offsets of new PLD-firmware
#define PLD_REG_VERSION         0x00000000
#define PLD_REG_AUX             0x00000004
#define PLD_REG_INT_STATUS      0x00000008
#define PLD_REG_INT_ENABLE      0x00000060
#define PLD_REG_DO_FUNC         0x0000000C
#define PLD_REG_DO_STATE        0x00000010
#define PLD_REG_DO_ENABLE       0x00000014
#define PLD_REG_DI_FUNC         0x00000018
#define PLD_REG_DI_STATE        0x0000001C
#define PLD_REG_CNT_MODE        0x00000024
#define PLD_REG_CNT_OV          0x00000028
#define PLD_REG_CNT_PREL0       0x0000002C
#define PLD_REG_CNT_VAL0        0x0000003C
#define PLD_REG_PWM_CONFIG      0x0000004C
#define PLD_REG_PWM_PERIOD0     0x00000050
#define PLD_REG_PWM_LENGTH0     0x00000054
#define PLD_REG_PWM_DELTA0      0x00000058
#define PLD_REG_PWM_COUNT0      0x0000005C

#define CF54_DRV_DEFAULT_HEX_NUM    0x20



//---------------------------------------------------------------------------
//  Additional register definitions
//---------------------------------------------------------------------------

#define MCF_EPORT_EPDDR       MCF_REG08(0x000F04)
#define MCF_EPORT_EPPDR       MCF_REG08(0x000F09)



//---------------------------------------------------------------------------
//  Global variables
//---------------------------------------------------------------------------

// driver major number
static int  nDrvMajorNumber_g;

// PLD
static char              szCS1ResName_g[64];
static struct resource*  pCS1Ressource_g;
static BYTE*             pCS1BaseAddr_g;

// Memory mapped I/O
static char              szCS3ResName_g[64];
static struct resource*  pCS3Ressource_g;
static BYTE*             pCS3BaseAddr_g;



//---------------------------------------------------------------------------
//  Local types
//---------------------------------------------------------------------------



//---------------------------------------------------------------------------
//  Local variables
//---------------------------------------------------------------------------

static BYTE   bLastRSMSwitch_l      = 0xFF;
static DWORD  dwDebounceInterval_l  = 0;
static DWORD  dwDebounceStartTime_l = 0;
static unsigned int uiPldVer_l = 0;


//---------------------------------------------------------------------------
//  Prototypes of internal functions
//---------------------------------------------------------------------------

static  int  __init  PLCcoreCF54DrvInit (void);
static  void __exit  PLCcoreCF54DrvExit (void);

static  int      PLCcoreCF54DrvOpen    (struct inode* pDeviceFile_p, struct file* pInstance_p);
static  int      PLCcoreCF54DrvRelease (struct inode* pDeviceFile_p, struct file* pInstance_p);
static  ssize_t  PLCcoreCF54DrvRead    (struct file* pInstance_p, char* pDstBuff_p, size_t BuffSize_p, loff_t* pFileOffs_p);
static  ssize_t  PLCcoreCF54DrvWrite   (struct file* pInstance_p, const char* pSrcBuff_p, size_t BuffSize_p, loff_t* pFileOffs_p);
static  int      PLCcoreCF54DrvIoctl   (struct inode* pDeviceFile_p, struct file* pInstance_p, unsigned int uiIoctlCmd_p, unsigned long ulArg_p);


static  int      PLCcoreCF54DrvInitHardware       (void);
static  int      PLCcoreCF54DrvReleaseHardware    (void);
static  int      PLCcoreCF54DrvClearOutputs       (void);

int      PLCcoreCF54DrvCmdInitialize      (WORD* pwDrvVer_p);
int      PLCcoreCF54DrvCmdShutdown        (void);
int      PLCcoreCF54DrvCmdGetHardwareInfo (tCF54HwInfo* pCF54HwInfo_p);
int      PLCcoreCF54DrvCmdSetRunLED       (BYTE bState_p);
int      PLCcoreCF54DrvCmdSetErrLED       (BYTE bState_p);
int      PLCcoreCF54DrvCmdGetRSMSwitch    (BYTE* pbRSMSwitch_p);
int      PLCcoreCF54DrvCmdGetHexSwitch    (BYTE* pbHexSwitch_p);
int      PLCcoreCF54DrvCmdGetDipSwitch    (BYTE* pbDipSwitch_p);
int      PLCcoreCF54DrvCmdGetDigiIn       (tCF54DigiIn* pDiData_p);
int      PLCcoreCF54DrvCmdSetDigiOut      (tCF54DigiOut* pDoData_p);
static  DWORD    PLCcoreCF54DrvGetTickCount       (void);

EXPORT_SYMBOL(PLCcoreCF54DrvCmdInitialize);
EXPORT_SYMBOL(PLCcoreCF54DrvCmdShutdown);
EXPORT_SYMBOL(PLCcoreCF54DrvCmdGetHardwareInfo);
EXPORT_SYMBOL(PLCcoreCF54DrvCmdSetRunLED);
EXPORT_SYMBOL(PLCcoreCF54DrvCmdSetErrLED);
EXPORT_SYMBOL(PLCcoreCF54DrvCmdGetRSMSwitch);
EXPORT_SYMBOL(PLCcoreCF54DrvCmdGetHexSwitch);
EXPORT_SYMBOL(PLCcoreCF54DrvCmdGetDipSwitch);
EXPORT_SYMBOL(PLCcoreCF54DrvCmdGetDigiIn);
EXPORT_SYMBOL(PLCcoreCF54DrvCmdSetDigiOut);

#ifdef _CFG_PROCFS_
    static  int  PLCcoreCF54DrvProcRead (char* pcBuffer_p, char** ppcStart_p, off_t Offset_p, int nBufferSize_p, int* pEof_p, void* pData_p);
    static int PLCcoreCF54DrvProcWrite(struct file *file, const char __user *buffer, unsigned long count, void *data);
#endif



//---------------------------------------------------------------------------
//  Inline functions
//---------------------------------------------------------------------------

// Write data to PLD
static  inline  void  PLD_Write (DWORD dwRegAddr_p, DWORD dwIoData_p)
{
    *(volatile DWORD*)(pCS1BaseAddr_g + dwRegAddr_p) = dwIoData_p;
}

// Read data from PLD
static  inline  DWORD  PLD_Read (DWORD dwRegAddr_p)
{
    return ( *(volatile DWORD*)(pCS1BaseAddr_g + dwRegAddr_p) );
}



//---------------------------------------------------------------------------
//  Kernel Module specific Data Structures
//---------------------------------------------------------------------------

EXPORT_NO_SYMBOLS;


module_init(PLCcoreCF54DrvInit);
module_exit(PLCcoreCF54DrvExit);


/*
#include <fs.h>
struct file
{
    struct list_head        f_list;
    struct dentry           *f_dentry;
    struct vfsmount         *f_vfsmnt;
    struct file_operations  *f_op;
    atomic_t                f_count;
    unsigned int            f_flags;
    mode_t                  f_mode;
    int                     f_error;
    loff_t                  f_pos;
    struct fown_struct      f_owner;
    unsigned int            f_uid;
    unsigned int            f_gid;
    struct file_ra_state    f_ra;
    unsigned long           f_version;
    void                    *f_security;
    void                    *private_data;
    struct address_space    *f_mapping;
};
*/



/*
#include <fs.h>
struct inode
{
    struct hlist_node       i_hash;
    struct list_head        i_list;
    struct list_head        i_dentry;
    unsigned long           i_ino;
    atomic_t                i_count;
    umode_t                 i_mode;
    unsigned int            i_nlink;
    uid_t                   i_uid;
    gid_t                   i_gid;
    dev_t                   i_rdev;
    loff_t                  i_size;
    struct timespec         i_atime;
    struct timespec         i_mtime;
    struct timespec         i_ctime;
    unsigned int            i_blkbits;
    unsigned long           i_blksize;
    unsigned long           i_version;
    unsigned long           i_blocks;
    unsigned short          i_bytes;
    unsigned char           i_sock;
    spinlock_t              i_lock;         // i_blocks, i_bytes, maybe i_size
    struct semaphore        i_sem;
    struct rw_semaphore     i_alloc_sem;
    struct inode_operations *i_op;
    struct file_operations  *i_fop;         // former ->i_op->default_file_ops
    struct super_block      *i_sb;
    struct file_lock        *i_flock;
    struct address_space    *i_mapping;
    struct address_space    i_data;
#ifdef CONFIG_QUOTA
    struct dquot            *i_dquot[MAXQUOTAS];
#endif
    // These three should probably be a union
    struct list_head        i_devices;
    struct pipe_inode_info  *i_pipe;
    struct block_device     *i_bdev;
    struct cdev             *i_cdev;
    int                     i_cindex;

    __u32                   i_generation;

#ifdef CONFIG_DNOTIFY
    unsigned long           i_dnotify_mask; // Directory notify events
    struct dnotify_struct   *i_dnotify;     // for directory notifications
#endif

    unsigned long           i_state;
    unsigned long           dirtied_when;   // jiffies of first dirtying
    unsigned int            i_flags;
    atomic_t                i_writecount;
    void                    *i_security;

    union
    {
        void                *generic_ip;
    } u;
#ifdef __NEED_I_SIZE_ORDERED
    seqcount_t              i_size_seqcount;
#endif
};


static inline unsigned iminor(struct inode *inode)
{
    return MINOR(inode->i_rdev);
}

static inline unsigned imajor(struct inode *inode)
{
    return MAJOR(inode->i_rdev);
}
*/



/*
#include <fs.h>
struct file_operations
{
    struct module   *owner;
    loff_t          (*llseek)            (struct file *, loff_t, int);
    ssize_t         (*read)              (struct file *, char __user *, size_t, loff_t *);
    ssize_t         (*aio_read)          (struct kiocb *, char __user *, size_t, loff_t);
    ssize_t         (*write)             (struct file *, const char __user *, size_t, loff_t *);
    ssize_t         (*aio_write)         (struct kiocb *, const char __user *, size_t, loff_t);
    int             (*readdir)           (struct file *, void *, filldir_t);
    unsigned int    (*poll)              (struct file *, struct poll_table_struct *);
    int             (*ioctl)             (struct inode *, struct file *, unsigned int, unsigned long);
    int             (*mmap)              (struct file *, struct vm_area_struct *);
    int             (*open)              (struct inode *, struct file *);
    int             (*flush)             (struct file *);
    int             (*release)           (struct inode *, struct file *);
    int             (*fsync)             (struct file *, struct dentry *, int datasync);
    int             (*aio_fsync)         (struct kiocb *, int datasync);
    int             (*fasync)            (int, struct file *, int);
    int             (*lock)              (struct file *, int, struct file_lock *);
    ssize_t         (*readv)             (struct file *, const struct iovec *, unsigned long, loff_t *);
    ssize_t         (*writev)            (struct file *, const struct iovec *, unsigned long, loff_t *);
    ssize_t         (*sendfile)          (struct file *, loff_t *, size_t, read_actor_t, void __user *);
    ssize_t         (*sendpage)          (struct file *, struct page *, int, size_t, loff_t *, int);
    unsigned long   (*get_unmapped_area) (struct file *, unsigned long, unsigned long, unsigned long, unsigned long);
};
*/



/*
#include <ioport.h>
struct resource {
    const char *name;
    unsigned long start, end;
    unsigned long flags;
    struct resource *parent, *sibling, *child;
};
*/



static struct file_operations  PLCcoreCF54DrvFileOps_g =
{

    owner:      THIS_MODULE,
    open:       PLCcoreCF54DrvOpen,
    release:    PLCcoreCF54DrvRelease,
    read:       PLCcoreCF54DrvRead,
    write:      PLCcoreCF54DrvWrite,
    ioctl:      PLCcoreCF54DrvIoctl,

};



//=========================================================================//
//                                                                         //
//          P U B L I C   F U N C T I O N S                                //
//                                                                         //
//=========================================================================//

//---------------------------------------------------------------------------
//  Initailize Driver
//---------------------------------------------------------------------------
//  -> insmod driver
//---------------------------------------------------------------------------

static int  __init  PLCcoreCF54DrvInit (void)
{

int  iErr;
int  iRet;

#if defined(_CFG_DEVFS_) || defined(_CFG_PROCFS_)
    int  nMinorNumber;
#endif


    TRACE("IODRV: + cf54basio#PLCcoreCF54DrvInit...\n");
    TRACE("IODRV:   Driver build: %s / %s\n", __DATE__, __TIME__);

    iRet = 0;


    // register character device handler
    #ifdef _CFG_DYNMAJOR_
    {
        TRACE("IODRV:   Installing Driver '%s', Version %u.%02u...\n", DRV_NAME, DRV_VER_MAIN, DRV_VER_REL);
        TRACE("IODRV:   (using dynamic major number assignment)\n");
        nDrvMajorNumber_g = register_chrdev (0, DRV_NAME, &PLCcoreCF54DrvFileOps_g);
        if (nDrvMajorNumber_g != 0)
        {
            TRACE("IODRV:   Driver '%s' installed successful, assigned MajorNumber=%d\n", DRV_NAME, nDrvMajorNumber_g);
        }
        else
        {
            TRACE("IODRV:   ERROR: Driver '%s' is unable to get a free MajorNumber!\n", DRV_NAME);
            iRet = -EIO;
            goto Exit;
        }
    }
    #else
    {
        TRACE("IODRV:   Installing Driver '%s', Version %u.%02u, MajorNumber=%d...\n", DRV_NAME, DRV_VER_MAIN, DRV_VER_REL, DRV_MAJOR);
        TRACE("IODRV:   (using static major number assignment)\n");
        nDrvMajorNumber_g = DRV_MAJOR;
        iErr = register_chrdev (nDrvMajorNumber_g, DRV_NAME, &PLCcoreCF54DrvFileOps_g);
        if (iErr == 0)
        {
            TRACE("IODRV:   Driver '%s' installed successful.\n", DRV_NAME);
        }
        else
        {
            TRACE("IODRV:   ERROR: Driver '%s' is unable to register MajorNumber %d!\n", DRV_NAME, nDrvMajorNumber_g);
            iRet = -EIO;
            goto Exit;
        }
    }
    #endif


    // create device node in DEVFS
    #ifdef _CFG_DEVFS_
    {
        nMinorNumber = 0;
        TRACE("IODRV:   Creating device node '/dev/%s'...\n", DEV_NAME);
        iErr = devfs_mk_cdev (MKDEV(nDrvMajorNumber_g, nMinorNumber), S_IFCHR | S_IRUGO | S_IWUGO, DEV_NAME);
        if (iErr == 0)
        {
            TRACE("IODRV:   Device node '/dev/%s' created successful.\n", DEV_NAME);
        }
        else
        {
            TRACE("IODRV:   ERROR: unable to create device node '/dev/%s'\n", DEV_NAME);
            iRet = -EIO;
            goto Exit;
        }
    }
    #endif


    // create device node in PROCFS
    #ifdef _CFG_PROCFS_
    {
        struct proc_dir_entry*  pProcDirEntry;
        char  szDevName[32];

        nMinorNumber = 0;
        TRACE("IODRV:   Creating device node '/proc/%s'...\n", DEV_NAME);

        snprintf (szDevName, sizeof(szDevName), "%s", DEV_NAME);
        pProcDirEntry = create_proc_entry (szDevName, S_IRUGO, NULL);
        if (pProcDirEntry != NULL)
        {
            pProcDirEntry->read_proc  = PLCcoreCF54DrvProcRead;
            pProcDirEntry->write_proc = PLCcoreCF54DrvProcWrite;
            pProcDirEntry->data       = (void*) (DWORD)nMinorNumber;

            TRACE("IODRV:   Device node '/proc/%s' created successful.\n", DEV_NAME);
        }
        else
        {
            TRACE("IODRV:   ERROR: unable to create device node '/proc/%s\n", DEV_NAME);
            iRet = -EIO;
            goto Exit;
        }
    }
    #endif


    // initialize hardware
    iRet = PLCcoreCF54DrvInitHardware();
    if (iRet != CF54DRV_RES_OK)
    {
        // unregister device
        PLCcoreCF54DrvExit();
    }


Exit:

    TRACE("IODRV: - cf54basio#PLCcoreCF54DrvInit (iRet=%d)\n", iRet);
    return (iRet);

}



//---------------------------------------------------------------------------
//  Remove Driver
//---------------------------------------------------------------------------
//  -> rmmod driver
//---------------------------------------------------------------------------

static void  __exit  PLCcoreCF54DrvExit (void)
{

#if defined(_CFG_DEVFS_) || defined(_CFG_PROCFS_)
    int  nMinorNumber;
#endif


    TRACE("IODRV: + cf54basio#PLCcoreCF54DrvExit...\n");


    // clear all output lines
    PLCcoreCF54DrvClearOutputs();

    // set LEDs to off
    PLCcoreCF54DrvCmdSetRunLED (OFF);
    PLCcoreCF54DrvCmdSetErrLED (OFF);


    // release hardware ressources
    PLCcoreCF54DrvReleaseHardware();


    // remove device node from DEVFS
    #ifdef _CFG_DEVFS_
    {
        nMinorNumber = 0;
        devfs_remove (DEV_NAME);
        TRACE("IODRV:   Device node '/dev/%s' removed.\n", DEV_NAME);
    }
    #endif


    // delete device nodes for all supported driver instances in PROCFS
    #ifdef _CFG_PROCFS_
    {
        char  szDevName[32];

        nMinorNumber = 0;
        snprintf (szDevName, sizeof(szDevName), "%s", DEV_NAME);
        remove_proc_entry (szDevName, NULL);
        TRACE("IODRV:   Device node '/proc/%s' removed.\n", DEV_NAME);
    }
    #endif


    // unregister character device handler
    unregister_chrdev (nDrvMajorNumber_g, DRV_NAME);
    TRACE("IODRV:   Driver '%s' removed.\n", DRV_NAME);


    TRACE("IODRV: - cf54basio#PLCcoreCF54DrvExit\n");

}



//---------------------------------------------------------------------------
//  Open Driver
//---------------------------------------------------------------------------
//  -> open("/dev/driver", O_RDWR)...
//---------------------------------------------------------------------------

static int  PLCcoreCF54DrvOpen (
    struct inode* pDeviceFile_p,    // information about the device to open
    struct file* pInstance_p)       // information about driver instance
{

int  iRet;


    TRACE("IODRV: + cf54basio#PLCcoreCF54DrvOpen...\n");

    MOD_INC_USE_COUNT;


    iRet = 0;




    TRACE("IODRV: - cf54basio#PLCcoreCF54DrvOpen (iRet=%d)\n", iRet);
    return (iRet);

}



//---------------------------------------------------------------------------
//  Close Driver
//---------------------------------------------------------------------------
//  -> close(device)...
//---------------------------------------------------------------------------

static int  PLCcoreCF54DrvRelease (
    struct inode* pDeviceFile_p,    // information about the device to open
    struct file* pInstance_p)       // information about driver instance
{

int  iRet;


    TRACE("IODRV: + cf54basio#PLCcoreCF54DrvRelease...\n");

    iRet = 0;


    MOD_DEC_USE_COUNT;

    return 0;


    TRACE("IODRV: - cf54basio#PLCcoreCF54DrvRelease (iRet=%d)\n", iRet);
    return (iRet);

}



//---------------------------------------------------------------------------
//  Read Data from Driver
//---------------------------------------------------------------------------
//  -> read(...)
//---------------------------------------------------------------------------

static ssize_t  PLCcoreCF54DrvRead (
    struct file* pInstance_p,       // information about driver instance
    char* pDstBuff_p,               // address of buffer to fill with data
    size_t BuffSize_p,              // length of the buffer
    loff_t* pFileOffs_p)            // offset in the file
{

int  iRet;


    TRACE("IODRV: + cf54basio#PLCcoreCF54DrvRead...\n");


    TRACE("IODRV:   Sorry, this operation isn't supported.\n");
    iRet = -EINVAL;


    TRACE("IODRV: - cf54basio#PLCcoreCF54DrvRead (iRet=%d)\n", iRet);
    return (iRet);

}



//---------------------------------------------------------------------------
//  Write Data to Driver
//---------------------------------------------------------------------------
//  -> write(...)
//---------------------------------------------------------------------------

static ssize_t  PLCcoreCF54DrvWrite (
    struct file* pInstance_p,       // information about driver instance
    const char* pSrcBuff_p,         // address of buffer to get data from
    size_t BuffSize_p,              // length of the buffer
    loff_t* pFileOffs_p)            // offset in the file
{

int  iRet;


    TRACE("IODRV: + cf54basio#PLCcoreCF54DrvWrite...\n");


    TRACE("IODRV:   Sorry, this operation isn't supported.\n");
    iRet = -EINVAL;


    TRACE("IODRV: - cf54basio#PLCcoreCF54DrvWrite (iRet=%d)\n", iRet);
    return (iRet);

}



//---------------------------------------------------------------------------
//  Generic Access to Driver
//---------------------------------------------------------------------------
//  -> ioctl(...)
//---------------------------------------------------------------------------

static int  PLCcoreCF54DrvIoctl (
    struct inode* pDeviceFile_p,    // information about the device to open
    struct file* pInstance_p,       // information about driver instance
    unsigned int uiIoctlCmd_p,      // Ioctl command to execute
    unsigned long ulArg_p)          // Ioctl command specific argument/parameter
{

int  iErr;
int  iRet;


    TRACE("IODRV: + cf54basio#PLCcoreCF54DrvIoctl (uiIoctlCmd_p=%d)...\n", uiIoctlCmd_p);


    iRet = -EINVAL;

    switch (uiIoctlCmd_p)
    {
        // ----------------------------------------------------------
        case CF54DRV_CMD_INITIALIZE:
        {
            WORD  wDrvVer;

            iRet = PLCcoreCF54DrvCmdInitialize (&wDrvVer);
            iErr = put_user (wDrvVer, (WORD*)ulArg_p);
            if (iErr != 0)
            {
                iRet = -EIO;
                goto Exit;
            }
            break;
        }


        // ----------------------------------------------------------
        case CF54DRV_CMD_SHUTDOWN:
        {
            iRet = PLCcoreCF54DrvCmdShutdown();
            break;
        }

/*
        // ----------------------------------------------------------
        case CF54DRV_CMD_RESET_TARGET:
        {
            iRet = PLCcoreCF54DrvCmdResetTarget();
            break;
        }


        // ----------------------------------------------------------
        case CF54DRV_CMD_ENABLE_WATCHDOG:
        {
            iRet = PLCcoreCF54DrvCmdEnableWatchdog();
            break;
        }


        // ----------------------------------------------------------
        case CF54DRV_CMD_SERVICE_WATCHDOG:
        {
            iRet = PLCcoreCF54DrvCmdServiveWatchdog();
            break;
        }
*/

        // ----------------------------------------------------------
        case CF54DRV_CMD_GET_HARDWARE_INFO:
        {
            tCF54HwInfo  CF54HwInfo;

            iRet = PLCcoreCF54DrvCmdGetHardwareInfo (&CF54HwInfo);
            iErr = copy_to_user ((tCF54HwInfo*)ulArg_p, &CF54HwInfo, sizeof(CF54HwInfo));
            if (iErr != 0)
            {
                iRet = -EIO;
                goto Exit;
            }
            break;
        }


        // ----------------------------------------------------------
        case CF54DRV_CMD_SET_RUN_LED:
        {
            BYTE  bState;

            bState = (BYTE)ulArg_p;
            iRet = PLCcoreCF54DrvCmdSetRunLED (bState);
            break;
        }


        // ----------------------------------------------------------
        case CF54DRV_CMD_SET_ERR_LED:
        {
            BYTE  bState;

            bState = (BYTE)ulArg_p;
            iRet = PLCcoreCF54DrvCmdSetErrLED (bState);
            break;
        }


        // ----------------------------------------------------------
        case CF54DRV_CMD_GET_RSM_SWITCH:
        {
            BYTE  bRSMSwitch;

            iRet = PLCcoreCF54DrvCmdGetRSMSwitch (&bRSMSwitch);
            iErr = put_user (bRSMSwitch, (BYTE*)ulArg_p);
            if (iErr != 0)
            {
                iRet = -EIO;
                goto Exit;
            }
            break;
        }


        // ----------------------------------------------------------
        case CF54DRV_CMD_GET_HEX_SWITCH:
        {
            BYTE  bHexSwitch;

            iRet = PLCcoreCF54DrvCmdGetHexSwitch (&bHexSwitch);
            iErr = put_user (bHexSwitch, (BYTE*)ulArg_p);
            if (iErr != 0)
            {
                iRet = -EIO;
                goto Exit;
            }
            break;
        }


        // ----------------------------------------------------------
        case CF54DRV_CMD_GET_DIP_SWITCH:
        {
            BYTE  bDipSwitch;

            iRet = PLCcoreCF54DrvCmdGetDipSwitch (&bDipSwitch);
            iErr = put_user (bDipSwitch, (BYTE*)ulArg_p);
            if (iErr != 0)
            {
                iRet = -EIO;
                goto Exit;
            }
            break;
        }


        // ----------------------------------------------------------
        case CF54DRV_CMD_GET_DIGI_IN:
        {
            tCF54DigiIn  DiData;

            iRet = PLCcoreCF54DrvCmdGetDigiIn (&DiData);
            iErr = copy_to_user ((tCF54DigiIn*)ulArg_p, &DiData, sizeof(tCF54DigiIn));
            if (iErr != 0)
            {
                iRet = -EIO;
                goto Exit;
            }
            break;
        }


        // ----------------------------------------------------------
        case CF54DRV_CMD_SET_DIGI_OUT:
        {
            tCF54DigiOut  DoData;
            iErr = copy_from_user (&DoData, (tCF54DigiOut*)ulArg_p, sizeof(tCF54DigiOut));
            if (iErr != 0)
            {
                iRet = -EIO;
                goto Exit;
            }
            iRet = PLCcoreCF54DrvCmdSetDigiOut (&DoData);
            break;
        }


        // ----------------------------------------------------------
        default:
        {
            break;
        }
    }


Exit:

    TRACE("IODRV: - cf54basio#PLCcoreCF54DrvIoctl (iRet=%d)\n", iRet);
    return (iRet);

}





//=========================================================================//
//                                                                         //
//          P R I V A T E   F U N C T I O N S                              //
//                                                                         //
//=========================================================================//

//---------------------------------------------------------------------------
//  Private: Initialize hardware
//---------------------------------------------------------------------------

static  int  PLCcoreCF54DrvInitHardware (void)
{

volatile DWORD  dwIoData;
DWORD  dwIoBase;
DWORD  dwIoSize;
BYTE   bCpuPldType;
int    iRet;


    TRACE("IODRV: + cf54basio#PLCcoreCF54DrvInitHardware...\n");

    iRet = CF54DRV_RES_OK;


    // initialize variable for RSM-Switch debouncing
    bLastRSMSwitch_l = SWITCH_MRES;
    dwDebounceInterval_l  = 0;
    dwDebounceStartTime_l = 0;


    //-------------------------------------------------------------------
    // Step (1): Allocate resources for memory maped I/O periphery
    //-------------------------------------------------------------------

    // -------- PLD --------
    // configure CS1 for PLD access
    dwIoBase = CS1_IO_BASE_ADDR;
    dwIoSize = CS1_IO_MEM_SIZE;
    strcpy (szCS1ResName_g, DRV_NAME);
    strcat (szCS1ResName_g, "_CS1_PLD");

    MCF_CSARn(1) = dwIoBase;
    MCF_CSCRn(1) = 0x00100100;
    MCF_CSMRn(1) = 1;

    // allocate hardware ressource for CS1 (PLD)
    TRACE("IODRV:   CS1: request_mem_region(dwIoBase=0x%08lX, dwIoSize=%lu)...\n", dwIoBase, dwIoSize);
    pCS1Ressource_g = request_mem_region (dwIoBase, dwIoSize, szCS1ResName_g);
    TRACE("IODRV:   pCS1Ressource = 0x%08lX\n", (DWORD)pCS1Ressource_g);
    if (pCS1Ressource_g == NULL)
    {
        TRACE("IODRV:   ERROR: Can't request memory region\n");
        iRet = -EIO;
        goto Exit;
    }

    TRACE("IODRV:       Name  = '%s'\n",           pCS1Ressource_g->name);
    TRACE("IODRV:       Start = 0x%08lX\n", (DWORD)pCS1Ressource_g->start);
    TRACE("IODRV:       End   = 0x%08lX\n", (DWORD)pCS1Ressource_g->end);
    TRACE("IODRV:       Flags = 0x%08lX\n", (DWORD)pCS1Ressource_g->flags);

    // get virtual pointer for access to this hardware ressource
    TRACE("IODRV:   ioremap(dwIoBase=0x%08lX, dwIoSize=%lu)...\n", pCS1Ressource_g->start, pCS1Ressource_g->end - pCS1Ressource_g->start + 1);
    pCS1BaseAddr_g = ioremap_nocache (pCS1Ressource_g->start, pCS1Ressource_g->end - pCS1Ressource_g->start);
    TRACE("IODRV:   pCS1BaseAddr = 0x%08lX\n", (DWORD)pCS1BaseAddr_g);
    if (pCS1BaseAddr_g == NULL)
    {
        TRACE("IODRV:   ERROR: Can't request virtual pointer for hardware access\n");
        iRet = -EIO;
        goto Exit;
    }



    // check PLD Type ID
    dwIoData = PLD_Read (PLD_REG_VERSION);
    bCpuPldType = (BYTE) ((dwIoData & 0x0000000F) >>  0);
    if (bCpuPldType != PLC_CORE_PLD_TYPE_ID)
    {
        printk("\n\npc5484drv - ERROR: Wrong PLD Type ID (expected=0x%02X, found=0x%02X)\n", PLC_CORE_PLD_TYPE_ID, (WORD)bCpuPldType);
//        TRACE("IODRV:   ERROR: Wrong PLD Type ID (expected=0x%02X, found=0x%02X)\n", PLC_CORE_PLD_TYPE_ID, (WORD)bCpuPldType);

        printk("pc5484drv Reg0 = 0x%lX, Reg1 = 0x%lX, Reg2 = 0x%lX, Reg3 = 0x%lX\n",
                PLD_Read(0x0),
                PLD_Read(0x4),
                PLD_Read(0x8),
                PLD_Read(0xC));
        // old PLD firmware present
        uiPldVer_l = 0;
//        iRet = -EIO;
//        goto Exit;
    }
    else
    {   // new PLD firmware present
        uiPldVer_l = ((dwIoData & 0x00000F00) >>  8);
    }


    // -------- I/O mapped periphery --------
    // configure CS3 for memory mapped I/O periphery access
    dwIoBase = CS3_IO_BASE_ADDR;
    dwIoSize = CS3_IO_MEM_SIZE;
    strcpy (szCS3ResName_g, DRV_NAME);
    strcat (szCS3ResName_g, "_CS3_IO");

    MCF_CSARn(3) = dwIoBase;
    MCF_CSCRn(3) = 0x00100100;
    MCF_CSMRn(3) = 1;       // dwIoSize;

    // allocate hardware ressource for CS3 (memory mapped I/O periphery)
    TRACE("IODRV:   CS3: request_mem_region(dwIoBase=0x%08lX, dwIoSize=%lu)...\n", dwIoBase, dwIoSize);
    pCS3Ressource_g = request_mem_region (dwIoBase, dwIoSize, szCS3ResName_g);
    TRACE("IODRV:   pCS3Ressource = 0x%08lX\n", (DWORD)pCS3Ressource_g);
    if (pCS3Ressource_g == NULL)
    {
        TRACE("IODRV:   ERROR: Can't request memory region\n");
        iRet = -EIO;
        goto Exit;
    }

    TRACE("IODRV:       Name  = '%s'\n",           pCS3Ressource_g->name);
    TRACE("IODRV:       Start = 0x%08lX\n", (DWORD)pCS3Ressource_g->start);
    TRACE("IODRV:       End   = 0x%08lX\n", (DWORD)pCS3Ressource_g->end);
    TRACE("IODRV:       Flags = 0x%08lX\n", (DWORD)pCS3Ressource_g->flags);

    // get virtual pointer for access to this hardware ressource
    TRACE("IODRV:   ioremap(dwIoBase=0x%08lX, dwIoSize=%lu)...\n", pCS3Ressource_g->start, pCS3Ressource_g->end - pCS3Ressource_g->start + 1);
    pCS3BaseAddr_g = ioremap_nocache (pCS3Ressource_g->start, pCS3Ressource_g->end - pCS3Ressource_g->start);
    TRACE("IODRV:   pCS1BaseAddr = 0x%08lX\n", (DWORD)pCS3BaseAddr_g);
    if (pCS3BaseAddr_g == NULL)
    {
        TRACE("IODRV:   ERROR: Can't request virtual pointer for hardware access\n");
        iRet = -EIO;
        goto Exit;
    }


    //-------------------------------------------------------------------
    // Step (2): Configure I/O periphery
    //-------------------------------------------------------------------

    if (uiPldVer_l > 0)
    {
        // -------- Inputs --------
        TRACE("IODRV:   Configure inputs...\n");
        // (DI0 [Btn S0]):  PLD -> select DI function in PLC configuration register
        // (DI8):           PLD -> select DI function in PLC configuration register
        // (DI9):           PLD -> select DI function in PLC configuration register
        // (DI10):          PLD -> select DI function in PLC configuration register
        // ...
        // (DI19):          PLD -> select DI function in PLC configuration register
        // (DI20):          PLD -> select DI function in PLC configuration register
        // (DI21):          PLD -> select DI function in PLC configuration register
        // (DI23):          PLD -> select DI function in PLC configuration register
        dwIoData = 0x00000000;              // select DI function for all inputs
        PLD_Write (PLD_REG_DI_FUNC, dwIoData);
    }

    #ifndef PCB_4158_DI1_4_VIA_CS3      // JP300 must be closed
    {
        //  DI1 [Btn S1]:   PCI_BR0 = PCI_IO5 = I
        //  DI2 [Btn S2]:   PCI_BR1 = PCI_IO6 = I
        MCF_GPIO_PDDR_PCIBR &= 0xFC;    // set PCI_BR0/1 direction bits to input
        MCF_GPIO_PAR_PCIBR  &= 0xFFF0;  // set PCI_BR0/1 to general purpose I/O

        //  DI3 [Btn S3]:   TIN1 = GPIO = I -> nothing to do, is always GPIO

        if (uiPldVer_l <= 2)
        {
        //  DI4 [Btn S4]:   /IRQ7 = EPDD7 = I (until 4158.4)
            MCF_EPIER       &= 0x7F;        // disable interrupts from IRQ7
            MCF_EPPAR       &= 0x3FFF;      // set EPPA7 resp. IRQ7 to level-sensitive
            MCF_EPORT_EPDDR &= 0x7F;        // set EPDD7 resp. IRQ7 direction bits to input
        }
        else
        {

            //  DI4 [Btn S4]:   TIN3 = GPIO = I (since 4158.5)
            MCF_GPIO_PAR_TIMER |= 0x30;     // set Timer3 to simple GPIO
        }


    }
    #endif  // #ifndef PCB_4158_DI1_4_VIA_CS3

    //  DI5:            /IRQ6 = EPDD6 = I
    //  DI6:            /IRQ5 = EPDD5 = I (until 4158.4)
    //  DI7:            /IRQ7 = EPDD7 = I (since 4158.5)
    MCF_EPIER              &= 0x1F;     // disable interrupts from IRQ6/5
    MCF_GPIO_PAR_FECI2CIRQ |= 0x0003;   // configure IRQ6/5 pins for GPIO
    MCF_EPPAR              &= 0x03FF;   // set EPPA6/5 resp. IRQ6/5 to level-sensitive
    MCF_EPORT_EPDDR &= 0x1F;            // set EPDD7/6/5 resp. IRQ7/6/5 direction bits to input

    //  DI7:            TIN0 = GPIO = I -> nothing to do, I-values in Reg. GSR (until 4158.4)

    if (uiPldVer_l == 0)
    {
        //  DI6:            /PSC3_RTS = PSC_IO6 = I
        MCF_GPIO_PDDR_PSC3PSC2 &= (!MCF_GPIO_PDDR_PSC3PSC2_PDDRPSC3PSC26); // set PPSC1PSC26 direction bits to input
        MCF_GPIO_PAR_PSC3 &= 0xCF; // PSC3_RTS defined as GPIO
    }

    // (DI8):           PLD -> nothing to do here
    // (DI9):           PLD -> nothing to do here
    // (DI10):          PLD -> nothing to do here
    // ...
    // (DI19):          PLD -> nothing to do here
    // (DI20):          PLD -> nothing to do here
    // (DI21):          PLD -> nothing to do here

    // DI22:            PSC1RTS = PPSC1PSC06 = I
    MCF_GPIO_PDDR_PSC1PSC0 &= 0xBD;     // set PPSC1PSC06 direction bits to input
    MCF_GPIO_PAR_PSC1      &= 0xCF;     // set PSC1RTS to general purpose I/O (PPSC1PSC06)

    // (DI23):          PLD -> nothing to do here


    if (uiPldVer_l > 0)
    {
        // -------- Outputs --------
        TRACE("IODRV:   Configure outputs...\n");
        // (DO0 [Btn S0]):  PLD -> nothing to do here
        // (DO8):           PLD -> nothing to do here
        // (DO9):           PLD -> nothing to do here
        // (DO10):          PLD -> nothing to do here
        // ...
        // (DO19):          PLD -> nothing to do here
        // (DO20):          PLD -> nothing to do here
        // (DO21):          PLD -> nothing to do here
        dwIoData = 0x00000000;              // set data bits to off
        TRACE("IODRV:   [PLD_REG_DO_STATE]   = 0x%08lX\n", dwIoData);
        PLD_Write (PLD_REG_DO_STATE, dwIoData);

        PLD_Write (PLD_REG_DO_FUNC, 0x00000000);

        dwIoData = 0x003FFF01;              // enable outputs
        TRACE("IODRV:   [PLD_REG_DO_ENABLE]  = 0x%08lX\n", dwIoData);
        PLD_Write (PLD_REG_DO_ENABLE, dwIoData);
    }

    #ifndef PCB_4158_DO1_4_VIA_CS3      // Jumper 302 must be set to 2-3
    {
        //  DO1 [LED D1]:   PCI_BG1 = PCI_IO1 = O
        //  DO2 [LED D2]:   PCI_BG2 = PCI_IO2 = O
        //  DO3 [LED D3]:   PCI_BG3 = PCI_IO3 = O
        //  DO4 [LED D4]:   PCI_BG4 = PCI_IO4 = O
        MCF_GPIO_PODR_PCIBG &= 0xE1;    // set data bits to off
        MCF_GPIO_PDDR_PCIBG |= 0x1E;    // set direction bits to output
        MCF_GPIO_PAR_PCIBG  &= 0xFC03;  // set PCIBG1-PCIBG4 to general purpose I/O
    }
    #endif  // #ifndef PCB_4158_DO1_4_VIA_CS3

    //  DO5:            PCI_BG0 = PCI_IO0 = O
    MCF_GPIO_PODR_PCIBG &= 0xFE;        // set data bits to off
    MCF_GPIO_PDDR_PCIBG |= 0x01;        // set direction bits to output
    MCF_GPIO_PAR_PCIBG  &= 0xFFFC;      // set PCIBG0 to general purpose I/O

    // Alternative 1 (selected by jumper):
    // DO6:             PSC3RTS = PPSC3PSC26 = O
    // MCF_GPIO_PODR_PSC3PSC2 &= 0xFB;     // set PPSC3PSC26 data bit to off
    // MCF_GPIO_PDDR_PSC3PSC2 |= 0x40;     // set PPSC3PSC26 direction bits to output
    // MCF_GPIO_PAR_PSC3      &= 0xCF;     // set PSC3RTS to general purpose I/O (PPSC3PSC26)
    // Alternative 2 (selected by jumper):
    // DO6:             TOUT3 = O
    MCF_GPT_GMS3 = 0x00000024;          // set Timer2 to simple GPIO and TOUT1 = 0


    // DO7:             TOUT1 = O
    MCF_GPT_GMS0 = 0x00000024;          // set Timer0 to simple GPIO and TOUT0 = 0
                                        // pin state: 0x30=on / 0x20=off

    // (DO8):           PLD -> nothing to do here
    // (DO9):           PLD -> nothing to do here
    // (DO10):          PLD -> nothing to do here
    // ...
    // (DO19):          PLD -> nothing to do here
    // (DO20):          PLD -> nothing to do here
    // (DO21):          PLD -> nothing to do here


    // -------- Run-/Error-LED --------
    //  LED D5  (Run):  PCI_BR2 = PCI_IO7 = O
    //  LED D10 (Err):  PCI_BR3 = PCI_IO8 = O
    MCF_GPIO_PODR_PCIBR |= 0x0C;        // set PCI_BR2/3 data bits to off
    MCF_GPIO_PDDR_PCIBR |= 0x0C;        // set PCI_BR2/3 direction bits to output
    MCF_GPIO_PAR_PCIBR  &= 0xFF0F;      // set PCI_BR2/3 to general purpose I/O

/*
    //-------------------------------------------------------------------
    // Step (2): Configure memory maped based I/O periphery
    //-------------------------------------------------------------------
    // configure CS1 for IO periphery access
    dwIoBase   = CS1_IO_BASE_ADDR;
    dwIoSize   = CS1_IO_MEM_SIZE;
    strcpy (szCS1ResName_g, DRV_NAME);
    strcat (szCS1ResName_g, "_CS1");

    MCF_CSARn(1) = dwIoBase;
    MCF_CSCRn(1) = 0x00100100;
    MCF_CSMRn(1) = 1;       // dwIoSize;

    // allocate hardware ressource for CS1
    TRACE("IODRV:   CS1: request_mem_region(dwIoBase=0x%08lX, dwIoSize=%lu)...\n", dwIoBase, dwIoSize);
    pCS1Ressource_g = request_mem_region (dwIoBase, dwIoSize, szCS1ResName_g);
    TRACE("IODRV:   pCS1Ressource = 0x%08lX\n", (DWORD)pCS1Ressource_g);
    if (pCS1Ressource_g == NULL)
    {
        TRACE("IODRV:   ERROR: Can't request memory region\n");
        iRet = -EIO;
        goto Exit;
    }

    TRACE("IODRV:       Name  = '%s'\n",           pCS1Ressource_g->name);
    TRACE("IODRV:       Start = 0x%08lX\n", (DWORD)pCS1Ressource_g->start);
    TRACE("IODRV:       End   = 0x%08lX\n", (DWORD)pCS1Ressource_g->end);
    TRACE("IODRV:       Flags = 0x%08lX\n", (DWORD)pCS1Ressource_g->flags);


    // get virtual pointer for access to this hardware ressource
    TRACE("IODRV:   ioremap(dwIoBase=0x%08lX, dwIoSize=%lu)...\n", pCS1Ressource_g->start, pCS1Ressource_g->end - pCS1Ressource_g->start + 1);
    // $$$$$$ hier besser <ioremap_nocache> statt "einfachem" <ioremap> ???
    pCS1BaseAddr_g = ioremap (pCS1Ressource_g->start, pCS1Ressource_g->end - pCS1Ressource_g->start);
    TRACE("IODRV:   pCS1BaseAddr = 0x%08lX\n", (DWORD)pCS1BaseAddr_g);
    if (pCS1BaseAddr_g == NULL)
    {
        TRACE("IODRV:   ERROR: Can't request virtual pointer for hardware access\n");
        iRet = -EIO;
        goto Exit;
    }


    // configure CS3 for IO periphery access
    dwIoBase   = CS3_IO_BASE_ADDR;
    dwIoSize   = CS3_IO_MEM_SIZE;
    strcpy (szCS3ResName_g, DRV_NAME);
    strcat (szCS3ResName_g, "_CS3");

    MCF_CSARn(3) = dwIoBase;
    MCF_CSCRn(3) = 0x00100100;
    MCF_CSMRn(3) = 1;       // dwIoSize;

    // allocate hardware ressource for CS3
    TRACE("IODRV:   CS3: request_mem_region(dwIoBase=0x%08lX, dwIoSize=%lu)...\n", dwIoBase, dwIoSize);
    pCS3Ressource_g = request_mem_region (dwIoBase, dwIoSize, szCS3ResName_g);
    TRACE("IODRV:   pCS3Ressource = 0x%08lX\n", (DWORD)pCS3Ressource_g);
    if (pCS3Ressource_g == NULL)
    {
        TRACE("IODRV:   ERROR: Can't request memory region\n");
        iRet = -EIO;
        goto Exit;
    }

    TRACE("IODRV:       Name  = '%s'\n",           pCS3Ressource_g->name);
    TRACE("IODRV:       Start = 0x%08lX\n", (DWORD)pCS3Ressource_g->start);
    TRACE("IODRV:       End   = 0x%08lX\n", (DWORD)pCS3Ressource_g->end);
    TRACE("IODRV:       Flags = 0x%08lX\n", (DWORD)pCS3Ressource_g->flags);


    // get virtual pointer for access to this hardware ressource
    TRACE("IODRV:   ioremap(dwIoBase=0x%08lX, dwIoSize=%lu)...\n", pCS3Ressource_g->start, pCS3Ressource_g->end - pCS3Ressource_g->start + 1);
    // $$$$$$ hier besser <ioremap_nocache> statt "einfachem" <ioremap> ???
    pCS3BaseAddr_g = ioremap (pCS3Ressource_g->start, pCS3Ressource_g->end - pCS3Ressource_g->start);
    TRACE("IODRV:   pCS1BaseAddr = 0x%08lX\n", (DWORD)pCS3BaseAddr_g);
    if (pCS3BaseAddr_g == NULL)
    {
        TRACE("IODRV:   ERROR: Can't request virtual pointer for hardware access\n");
        iRet = -EIO;
        goto Exit;
    }
*/

Exit:

    TRACE("IODRV: - cf54basio#PLCcoreCF54DrvInitHardware (iRet=%d)\n", iRet);
    return (iRet);

}



//---------------------------------------------------------------------------
//  Private: Release hardware
//---------------------------------------------------------------------------

static  int  PLCcoreCF54DrvReleaseHardware (void)
{

int  iRet;


    TRACE("IODRV: + cf54basio#PLCcoreCF54DrvReleaseHardware...\n");

    iRet = CF54DRV_RES_OK;


    // release ressources for IO periphery at CS1
    if (pCS1Ressource_g != NULL)
    {
        TRACE("IODRV:   release ressource for:\n");
        TRACE("IODRV:       Name  = '%s'\n",           pCS1Ressource_g->name);
        TRACE("IODRV:       Start = 0x%08lX\n", (DWORD)pCS1Ressource_g->start);
        TRACE("IODRV:       End   = 0x%08lX\n", (DWORD)pCS1Ressource_g->end);
        TRACE("IODRV:       Flags = 0x%08lX\n", (DWORD)pCS1Ressource_g->flags);

        release_mem_region (pCS1Ressource_g->start, pCS1Ressource_g->end - pCS1Ressource_g->start + 1);
        pCS1Ressource_g = NULL;
    }


    // release ressources for IO periphery at CS3
    if (pCS3Ressource_g != NULL)
    {
        TRACE("IODRV:   release ressource for:\n");
        TRACE("IODRV:       Name  = '%s'\n",           pCS3Ressource_g->name);
        TRACE("IODRV:       Start = 0x%08lX\n", (DWORD)pCS3Ressource_g->start);
        TRACE("IODRV:       End   = 0x%08lX\n", (DWORD)pCS3Ressource_g->end);
        TRACE("IODRV:       Flags = 0x%08lX\n", (DWORD)pCS3Ressource_g->flags);

        release_mem_region (pCS3Ressource_g->start, pCS3Ressource_g->end - pCS3Ressource_g->start + 1);
        pCS3Ressource_g = NULL;
    }


    TRACE("IODRV: - cf54basio#PLCcoreCF54DrvReleaseHardware (iRet=%d)\n", iRet);
    return (iRet);

}



//---------------------------------------------------------------------------
//  Private: Clear all output lines
//---------------------------------------------------------------------------

static  int  PLCcoreCF54DrvClearOutputs (void)
{

tCF54DigiOut  DoData;
int           iRet;


    TRACE("IODRV: + cf54basio#PLCcoreCF54DrvClearOutputs...\n");

    iRet = CF54DRV_RES_OK;


    DoData.m_bDoByte0 = 0x00;
    DoData.m_bDoByte1 = 0x00;
    DoData.m_bDoByte2 = 0x00;
    DoData.m_bDoByte3 = 0x00;
    iRet = PLCcoreCF54DrvCmdSetDigiOut (&DoData);

    PLCcoreCF54DrvCmdSetRunLED (OFF);
    PLCcoreCF54DrvCmdSetErrLED (OFF);


    TRACE("IODRV: - cf54basio#PLCcoreCF54DrvClearOutputs (iRet=%d)\n", iRet);
    return (iRet);

}



//---------------------------------------------------------------------------
//  Handler for command "CF54DRV_CMD_INITIALIZE"
//---------------------------------------------------------------------------

int  PLCcoreCF54DrvCmdInitialize (
    WORD* pwDrvVer_p)
{

WORD  wDrvVer;
int   iRet;


    TRACE("IODRV: + cf54basio#PLCcoreCF54DrvCmdInitialize...\n");


    PLCcoreCF54DrvClearOutputs();

    wDrvVer = (DRV_VER_MAIN << 8) | DRV_VER_REL;

    *pwDrvVer_p = wDrvVer;
    iRet = CF54DRV_RES_OK;


    TRACE("IODRV: - cf54basio#PLCcoreCF54DrvCmdInitialize (iRet=%d, wDrvVer=0x%04X)\n", iRet, wDrvVer);
    return (iRet);

}




//---------------------------------------------------------------------------
//  Handler for command "CF54DRV_CMD_SHUTDOWN"
//---------------------------------------------------------------------------

int  PLCcoreCF54DrvCmdShutdown (void)
{

int  iRet;


    TRACE("IODRV: + cf54basio#PLCcoreCF54DrvCmdShutdown...\n");


    PLCcoreCF54DrvClearOutputs();
    iRet = CF54DRV_RES_OK;


    TRACE("IODRV: - cf54basio#PLCcoreCF54DrvCmdShutdown (iRet=%d)\n", iRet);
    return (iRet);

}



//---------------------------------------------------------------------------
//  Handler for command "CF54DRV_CMD_GET_HARDWARE_INFO"
//---------------------------------------------------------------------------

int  PLCcoreCF54DrvCmdGetHardwareInfo (
    tCF54HwInfo* pCF54HwInfo_p)
{

volatile DWORD  dwIoData;
WORD   wCpuPldVersion;
BYTE   bCpuPldRevision;
BYTE   bCpuPldType;
BYTE  bCpuPcbRevision;
BYTE  bCpuPcbHwId;
BYTE  bIoPcbHwId;
WORD   wCfgDriver;
int    iRet;


    TRACE("IODRV: + cf54basio#PLCcoreCF54DrvCmdGetHardwareInfo...\n");


    //---------------------------------------------------------------
    #if (PCB_VER == 4160-0)
    {
        TRACE("IODRV:   ERROR: real I/O access not implemented!\n");

        bIoHwId    = 0x00;
        wCfgDriver = 0;

        pCF54HwInfo_p->m_wCpuPcbVersion  = 4152;
        pCF54HwInfo_p->m_bCpuPcbRevision = 0;
        pCF54HwInfo_p->m_bCpuHwId        = 0x00;
        pCF54HwInfo_p->m_wCpuPldVersion  = wCpuPldVersion;
        pCF54HwInfo_p->m_bCpuPldRevision = bCpuPldRevision;
        pCF54HwInfo_p->m_bCpuPldType     = bCpuPldType;
        pCF54HwInfo_p->m_wIoPcbVersion   = 4160;
        pCF54HwInfo_p->m_bIoPcbRevision  = 0;
        pCF54HwInfo_p->m_bIoHwId         = 0x00;
        pCF54HwInfo_p->m_wCfgDriver      = wCfgDriver;

        iRet = CF54DRV_RES_NOT_IMPLEMENTED;
    }

    //---------------------------------------------------------------
    #elif (PCB_VER == 4158-1)
    {
        if (uiPldVer_l == 0)
        {
            wCfgDriver = 0;

            #ifdef PCB_4158_DI1_4_VIA_CS3
                wCfgDriver |= CFG_4158_DI1_4_VIA_CS3;   // JP300 can be opend
            #endif

            #ifdef PCB_4158_DO1_4_VIA_CS3
                wCfgDriver |= CFG_4158_DO1_4_VIA_CS3;   // JP302 must be set to 1-2
            #endif

            dwIoData        = *(DWORD*)(pCS1BaseAddr_g+0x04);
            wCpuPldVersion  =  (WORD) ((dwIoData & 0x000000F0) >> 4);
            bCpuPldRevision =  (BYTE) ((dwIoData & 0x0000000F) >> 0);
            bCpuPldType     =  (BYTE) ((dwIoData & 0x0000000F) >> 8);

            dwIoData = *(DWORD*)pCS3BaseAddr_g;
            bIoPcbHwId  =  (BYTE) ((dwIoData & 0x000F0000) >> 16);


            pCF54HwInfo_p->m_wCpuPcbVersion  = 4152;
            pCF54HwInfo_p->m_bCpuPcbRevision = 0;
            pCF54HwInfo_p->m_bCpuHwId        = 0x00;
            pCF54HwInfo_p->m_wCpuPldVersion  = wCpuPldVersion;
            pCF54HwInfo_p->m_bCpuPldRevision = bCpuPldRevision;
            pCF54HwInfo_p->m_bCpuPldType     = bCpuPldType;
            pCF54HwInfo_p->m_wIoPcbVersion   = 4158;
            pCF54HwInfo_p->m_bIoPcbRevision  = bIoPcbHwId;
            pCF54HwInfo_p->m_bIoHwId         = bIoPcbHwId;
            pCF54HwInfo_p->m_wCfgDriver      = wCfgDriver;

            iRet = CF54DRV_RES_OK;
        }
        else
        {
            // since 4158.5
            dwIoData        = PLD_Read (PLD_REG_VERSION);
            bCpuPldType     =  (BYTE) ((dwIoData & 0x0000000F) >>  0);
            bCpuPldRevision =  (BYTE) ((dwIoData & 0x000000F0) >>  4);
            wCpuPldVersion  =  (WORD) ((dwIoData & 0x00000F00) >>  8);
            bCpuPcbRevision =  (BYTE) ((dwIoData & 0x000F0000) >> 16);
            bCpuPcbHwId     =  (BYTE) ((dwIoData & 0x00F00000) >> 20);

            TRACE("IODRV:   [PLD_REG_VERSION] = 0x%08lX\n", dwIoData);


            dwIoData   = *(volatile DWORD*)pCS3BaseAddr_g;
            bIoPcbHwId =  (BYTE) ((dwIoData & 0x000F0000) >> 16);


            wCfgDriver = 0;

            #ifdef PCB_4158_DI1_4_VIA_CS3
                wCfgDriver |= CFG_4158_DI1_4_VIA_CS3;   // JP300 can be opend
            #endif

            #ifdef PCB_4158_DO1_4_VIA_CS3
                wCfgDriver |= CFG_4158_DO1_4_VIA_CS3;   // JP302 must be set to 1-2
            #endif


            pCF54HwInfo_p->m_wCpuPcbVersion  = CPU_PCB_VER;
            pCF54HwInfo_p->m_bCpuPcbRevision = bCpuPcbRevision;
            pCF54HwInfo_p->m_bCpuHwId        = bCpuPcbHwId;
            pCF54HwInfo_p->m_wCpuPldVersion  = wCpuPldVersion;
            pCF54HwInfo_p->m_bCpuPldRevision = bCpuPldRevision;
            pCF54HwInfo_p->m_bCpuPldType     = bCpuPldType;
            pCF54HwInfo_p->m_wIoPcbVersion   = IO_PCB_VER;
            pCF54HwInfo_p->m_bIoPcbRevision  = bIoPcbHwId;
            pCF54HwInfo_p->m_bIoHwId         = bIoPcbHwId;
            pCF54HwInfo_p->m_wCfgDriver      = wCfgDriver;

            iRet = CF54DRV_RES_OK;
        }
    }
    #endif
    //---------------------------------------------------------------



    TRACE("IODRV: - cf54basio#PLCcoreCF54DrvCmdGetHardwareInfo (iRet=%d)\n", iRet);
    return (iRet);

}





//-------------------------------------------------------------------------//
//                                                                         //
//          U N I T:    O P E R A T O R   C O N T R O L S                  //
//                                                                         //
//-------------------------------------------------------------------------//

//---------------------------------------------------------------------------
//  Handler for command "CF54DRV_CMD_SET_RUN_LED"
//---------------------------------------------------------------------------

int  PLCcoreCF54DrvCmdSetRunLED (
    BYTE bState_p)
{

int  iRet;


    TRACE("IODRV: + cf54basio#PLCcoreCF54DrvCmdSetRunLED (bState_p=%d)...\n", (int)(bState_p & 0x01));


    if (bState_p & 0x01)
    {
        MCF_GPIO_PODR_PCIBR &= ~0x04;
    }
    else
    {
        MCF_GPIO_PODR_PCIBR |= 0x04;
    }

    iRet = CF54DRV_RES_OK;


    TRACE("IODRV: - cf54basio#PLCcoreCF54DrvCmdSetRunLED (iRet=%d)\n", iRet);
    return (iRet);

}



//---------------------------------------------------------------------------
//  Handler for command "CF54DRV_CMD_SET_ERR_LED"
//---------------------------------------------------------------------------

int  PLCcoreCF54DrvCmdSetErrLED (
    BYTE bState_p)
{

int  iRet;


    TRACE("IODRV: + cf54basio#PLCcoreCF54DrvCmdSetErrLED (bState_p=%d)...\n", (int)(bState_p & 0x01));


    if (bState_p & 0x01)
    {
        MCF_GPIO_PODR_PCIBR &= ~0x08;
    }
    else
    {
        MCF_GPIO_PODR_PCIBR |= 0x08;
    }

    iRet = CF54DRV_RES_OK;


    TRACE("IODRV: - cf54basio#PLCcoreCF54DrvCmdErrRunLED (iRet=%d)\n", iRet);
    return (iRet);

}



//---------------------------------------------------------------------------
//  Handler for command "CF54DRV_CMD_GET_RSM_SWITCH"
//---------------------------------------------------------------------------

int  PLCcoreCF54DrvCmdGetRSMSwitch (
    BYTE* pbRSMSwitch_p)
{

volatile DWORD  dwIoData;
BYTE   bRSMSwitch;
DWORD  dwTmp;
int    iRet;


    TRACE("IODRV: + cf54basio#PLCcoreCF54DrvCmdGetRSMSwitch...\n");


    // read switch (until 4158.2/3)
    if (uiPldVer_l == 0)
    {
        dwTmp = *(volatile DWORD*)pCS1BaseAddr_g;
        dwTmp = (dwTmp & 0xC0000000) >> 30;
        switch (dwTmp)
        {
            case 1:     bRSMSwitch = SWITCH_STOP;   break;
            case 2:     bRSMSwitch = SWITCH_MRES;   break;
            default:    bRSMSwitch = SWITCH_RUN;    break;
        }

        // initialize <bLastRSMSwitch_l> in first cycle with real value
        if (bLastRSMSwitch_l == 0xFF)
        {
            bLastRSMSwitch_l = bRSMSwitch;
        }


        //-----------------------------------------------------------
        //  debouncing mode
        //-----------------------------------------------------------
        if (dwDebounceInterval_l > 0)
        {
            // debounce time elapsed?
            if ((PLCcoreCF54DrvGetTickCount() - dwDebounceStartTime_l) >= dwDebounceInterval_l)
            {
                // debounce time elapsed -> return new stable value
                bLastRSMSwitch_l = bRSMSwitch;

                // stop debounce state
                dwDebounceInterval_l  = 0;
                dwDebounceStartTime_l = 0;
            }
            else
            {
                // debounce interval still running -> return last stable state
                bRSMSwitch = bLastRSMSwitch_l;
            }

            goto Exit;
        }


        //-----------------------------------------------------------
        //  normal operation mode
        //-----------------------------------------------------------
        // is switch position always the same since last call?
        if (bRSMSwitch == bLastRSMSwitch_l)
        {
            // return current = last stable state
            goto Exit;
        }


        // evaluate last stable state to process current state in correct way
        switch (bLastRSMSwitch_l)
        {
            //-------------------------------------------------------
            // last state was RUN, current state can be STOP or MRES
            case SWITCH_RUN:
            {
                // set current state valid (must be STOP or MRES)
                bLastRSMSwitch_l = bRSMSwitch;
                break;
            }


            //-------------------------------------------------------
            // last state was STOP, current state can be RUN or MRES
            case SWITCH_STOP:
            {
                // state change STOP -> RUN ?
                if (bRSMSwitch == SWITCH_RUN)
                {
                    // start debounce state
                    dwDebounceInterval_l  = RSM_SWITCH_DEBOUNCE_TIME1;
                    dwDebounceStartTime_l = PLCcoreCF54DrvGetTickCount();

                    // return meanwhile last stable state
                    bRSMSwitch = bLastRSMSwitch_l;
                }
                else
                {
                    // set current state valid (must be MRES)
                    bLastRSMSwitch_l = bRSMSwitch;
                }
                break;
            }


            //-------------------------------------------------------
            // last state was MRES, current state can be STOP or RUN
            case SWITCH_MRES:
            {
                // state change STOP -> RUN ?
                if (bRSMSwitch == SWITCH_RUN)
                {
                    // normally the direct change from MRES to RUN
                    // is impossible for the used 3-state-switch type,
                    // however, detecting the STOP state in polling
                    // mode is also unsave
                    // -> start a long debounce time in the hope to
                    //    supress temorary sates
                    dwDebounceInterval_l  = RSM_SWITCH_DEBOUNCE_TIME2;
                    dwDebounceStartTime_l = PLCcoreCF54DrvGetTickCount();

                    // return meanwhile last stable state
                    bRSMSwitch = bLastRSMSwitch_l;
                }
                else
                {
                    // set current state valid (must be STOP)
                    bLastRSMSwitch_l = bRSMSwitch;
                }
                break;
            }
        }
    }
    else
    {
        // read switch (since 4158.5)
        dwIoData = PLD_Read (PLD_REG_AUX);
        TRACE("IODRV:   [PLD_REG_AUX] = 0x%08lX\n", dwIoData);


        // decode switch position
        if (dwIoData & 0x80000000)
        {
            bRSMSwitch = SWITCH_RUN;
        }
        else if (dwIoData & 0x40000000)
        {
            bRSMSwitch = SWITCH_STOP;
        }
        else if (dwIoData & 0x20000000)
        {
            bRSMSwitch = SWITCH_MRES;
        }
        else
        {
            bRSMSwitch = bLastRSMSwitch_l;
        }

        // save last valid switsch position
        bLastRSMSwitch_l = bRSMSwitch;

    }


Exit:

    *pbRSMSwitch_p = bRSMSwitch;
    iRet = CF54DRV_RES_OK;


    TRACE("IODRV: - cf54basio#PLCcoreCF54DrvCmdGetRSMSwitch (iRet=%d, bRSMSwitch=0x%02X)\n", iRet, (WORD)bLastRSMSwitch_l);
    return (iRet);

}



//---------------------------------------------------------------------------
//  Handler for command "CF54DRV_CMD_GET_HEX_SWITCH"
//---------------------------------------------------------------------------

int  PLCcoreCF54DrvCmdGetHexSwitch (
    BYTE* pbHexSwitch_p)
{

BYTE   bHexSwitch;
int    iRet;


    TRACE("IODRV: + cf54basio#PLCcoreCF54DrvCmdGetHexSwitch...\n");


    //---------------------------------------------------------------
    #if (PCB_VER == 4160-0)
    {
        // HexNumber
        TRACE("IODRV:   ERROR: real I/O access not implemented!\n");
        bHexSwitch = CF54_DRV_DEFAULT_HEX_NUM;
        iRet = CF54DRV_RES_NOT_IMPLEMENTED;
    }

    //---------------------------------------------------------------
    #elif (PCB_VER == 4158-1)
    {
        DWORD  dwTmp;

        // HexNumber (S307/S306)
        dwTmp = *(volatile DWORD*)pCS3BaseAddr_g;
        bHexSwitch = (dwTmp & 0x000000FF);
        iRet = CF54DRV_RES_OK;
    }
    #endif
    //---------------------------------------------------------------


    *pbHexSwitch_p = bHexSwitch;


    TRACE("IODRV: - cf54basio#PLCcoreCF54DrvCmdGetHexSwitch (iRet=%d, bHexSwitch=0x%02X)\n", iRet, (WORD)bHexSwitch);
    return (iRet);

}



//---------------------------------------------------------------------------
//  Handler for command "CF54DRV_CMD_GET_DIP_SWITCH"
//---------------------------------------------------------------------------

int  PLCcoreCF54DrvCmdGetDipSwitch (
    BYTE* pbDipSwitch_p)
{

BYTE   bDipSwitch;
DWORD  dwTmp;
int    iRet;


    TRACE("IODRV: + cf54basio#PLCcoreCF54DrvCmdGetDipSwitch...\n");


    //---------------------------------------------------------------
    #if (PCB_VER == 4160-0)
    {
        // DIP-Switch 1-8
        dwTmp = *(volatile DWORD*)pCS1BaseAddr_g;
        dwTmp = (dwTmp & 0x3fc00000) >> 22;
        bDipSwitch = (BYTE)dwTmp;
    }

    //---------------------------------------------------------------
    #elif (PCB_VER == 4158-1)
    {
        // DIP-Switch 1-8
        dwTmp = *(volatile DWORD*)pCS3BaseAddr_g;
        dwTmp = (dwTmp & 0x0000FF00) >> 8;
        bDipSwitch = (BYTE)dwTmp;
    }
    #endif
    //---------------------------------------------------------------


    *pbDipSwitch_p = bDipSwitch;
    iRet = CF54DRV_RES_OK;


    TRACE("IODRV: - cf54basio#PLCcoreCF54DrvCmdGetDipSwitch (iRet=%d, bDipSwitch=0x%02X)\n", iRet, (WORD)bDipSwitch);
    return (iRet);

}





//-------------------------------------------------------------------------//
//                                                                         //
//          U N I T:    D I G I T A L   I N / O U T                        //
//                                                                         //
//-------------------------------------------------------------------------//

//---------------------------------------------------------------------------
//  Handler for command "CF54DRV_CMD_GET_DIGI_IN"
//---------------------------------------------------------------------------

int  PLCcoreCF54DrvCmdGetDigiIn (
    tCF54DigiIn* pDiData_p)
{

int    iRet;


    TRACE("IODRV: + cf54basio#PLCcoreCF54DrvCmdGetDigiIn...\n");


    pDiData_p->m_bDiByte0 = 0;
    pDiData_p->m_bDiByte1 = 0;
    pDiData_p->m_bDiByte2 = 0;
    pDiData_p->m_bDiByte3 = 0;


    //---------------------------------------------------------------
    #if (PCB_VER == 4160-0)
    {
        BYTE   bDataByte;
        BYTE   bTmp;
        DWORD  dwTmp;

        // DI0/DI1 (Buttons S1/S2)
        bDataByte = MCF_GPIO_PPDSDR_PCIBR & 0x03;

        // DI2 (Button S3)
        dwTmp = MCF_GPT_GSR1;
        dwTmp = (dwTmp & 0x00000100) >> 6;
        bDataByte |= (BYTE)dwTmp;

        // DI3 (Button S4)
        bTmp = MCF_EPORT_EPPDR;
        bTmp = (~bTmp & 0x80) >> 4;
        bDataByte |= bTmp;

        pDiData_p->m_bDiByte0 = bDataByte;
    }

    //---------------------------------------------------------------
    #elif (PCB_VER == 4158-1)
    {
        DWORD  dwIoData;
        BYTE   bTmp;

        if (uiPldVer_l == 0)
        {
            // DI0 (Btn S0), DI8...DI21 (until 4158.2)
            dwIoData = *(volatile DWORD*)(pCS1BaseAddr_g+0x08);
        }
        else
        {
            // DI0 (Btn S0), DI8...DI21, DI23 (since 4158.4)
            dwIoData = PLD_Read (PLD_REG_DI_STATE);
        }

        #ifndef PCB_4158_DI1_4_VIA_CS3          // Jumper 300 must be closed
        {
            // DI1-DI2 (Btn S1-S2)
            dwIoData |= ((MCF_GPIO_PPDSDR_PCIBR & 0x03) << 1);

            // DI3 (Btn S3)
            if (MCF_GPT_GSR1 & 0x00000100)
            {
                dwIoData |= 0x00000008;
            }

            if (uiPldVer_l <= 2)
            {
                // DI4 (Btn S4) (until 4158.4 / PLD-Ver 2)
                bTmp = MCF_EPORT_EPPDR;
                if ( !(bTmp & 0x80) )               // DI4 = /IRQ7 = EPDD7
                {
                    dwIoData |= 0x00000010;
                }
            }
            else
            {
                // DI4 (Btn S4) (since 4158.5)
                if (MCF_GPT_GSR3 & MCF_GPT_GSR_PIN)     // DI4 = TIN3
                {
                    dwIoData |= 0x00000010;
                }
            }

        }
        #else                                   // Jumper 300 can be opend
        {
            DWORD  dwCS3Data;

            // DI1-DI4 (Btn S1-S4)
            dwCS3Data  = *(volatile DWORD*)pCS3BaseAddr_g;
            dwCS3Data  = (dwCS3Data & 0x00F00000) >> 19;
            dwIoData  |= dwCS3Data;
        }
        #endif  // #ifndef PCB_4158_DI1_4_VIA_CS3


        // DI5...DI6
        bTmp = MCF_EPORT_EPPDR;
        if ( !(bTmp & 0x40) )                   // DI5 = /IRQ6 = EPDD6
        {
            dwIoData |= 0x00000020;
        }
        if (uiPldVer_l == 0)
        {
            if ( !(bTmp & 0x20) )                   // DI6 = /IRQ5 = EPDD5 (until 4158.4)
            {
                dwIoData |= 0x00000040;
            }
        }
        else
        {
            // DI6 (since 4158.5)
            bTmp = MCF_GPIO_PPDSDR_PSC3PSC2;
            if (bTmp & MCF_GPIO_PPDSDR_PSC3PSC2_PPDSDRPSC3PSC26)    // DI6 = PSC_RTS3
            {
                dwIoData |= 0x00000040;
            }
        }

        // DI7
        if (MCF_GPT_GSR0 & 0x00000100)
        {
            dwIoData |= 0x00000080;
        }

        // DI22
        bTmp = MCF_GPIO_PPDSDR_PSC1PSC0;
        if (bTmp & 0x40)                        // DI22 = PSC1RTS = PPSC1PSC06
        {
            dwIoData |= 0x00400000;
        }


        pDiData_p->m_bDiByte0 = (BYTE) ((dwIoData >>  0) & 0xFF);
        pDiData_p->m_bDiByte1 = (BYTE) ((dwIoData >>  8) & 0xFF);
        pDiData_p->m_bDiByte2 = (BYTE) ((dwIoData >> 16) & 0xFF);
    }
    #endif
    //---------------------------------------------------------------


    iRet = CF54DRV_RES_OK;


    TRACE("IODRV: - cf54basio#PLCcoreCF54DrvCmdGetDigiIn (iRet=%d, bInValue=0x%02X)\n", iRet, (WORD)bInValue);
    return (iRet);

}



//---------------------------------------------------------------------------
//  Handler for command "CF54DRV_CMD_SET_DIGI_OUT"
//---------------------------------------------------------------------------

int  PLCcoreCF54DrvCmdSetDigiOut (
    tCF54DigiOut* pDoData_p)
{

int  iRet;


    TRACE("IODRV: + cf54basio#PLCcoreCF54DrvCmdSetDigiOut...\n");


    //---------------------------------------------------------------
    #if (PCB_VER == 4160-0)
    {
        BYTE  bDataByte;

        // DO0-DO3 (LED D1-D4)
        bDataByte = pDoData_p->m_bDoByte0 & 0x0F;
        bDataByte <<= 1;
        MCF_GPIO_PODR_PCIBG = ~bDataByte;
    }

    //---------------------------------------------------------------
    #elif (PCB_VER == 4158-1)
    {
        DWORD  dwIoData;

        dwIoData  = (DWORD) (pDoData_p->m_bDoByte0 <<  0);
        dwIoData |= (DWORD) (pDoData_p->m_bDoByte1 <<  8);
        dwIoData |= (DWORD) (pDoData_p->m_bDoByte2 << 16);
        dwIoData |= (DWORD) (pDoData_p->m_bDoByte3 << 24);


        #ifndef PCB_4158_DO1_4_VIA_CS3      // Jumper 302 must be set to 2-3
        {
            // DO1-DO4 (LED D1-D4)
            MCF_GPIO_PODR_PCIBG = (pDoData_p->m_bDoByte0 & 0x1E);
        }
        #else                               // Jumper 302 must be set to 1-2
        {
            DWORD  dwCS3Data;

            // DO1-DO4 (LED D1-D4)
            dwCS3Data  = *(volatile DWORD*)pCS3BaseAddr_g;
            dwCS3Data &= 0xF0FFFFFF;
            dwCS3Data |= ((DWORD)(pDoData_p->m_bDoByte0 & 0x1E)) << 23;
            *(volatile DWORD*)pCS3BaseAddr_g = dwCS3Data;
        }
        #endif  // #ifndef PCB_4158_DO1_4_VIA_CS3


        // DO5
        if (dwIoData & 0x00000020)
        {
            MCF_GPIO_PODR_PCIBG |= 0x01;
        }
        else
        {
            MCF_GPIO_PODR_PCIBG &= ~0x01;
        }

        // DO6
        if (dwIoData & 0x00000040)
        {
            MCF_GPT_GMS3 |= 0x00000010;
        }
        else
        {
            MCF_GPT_GMS3 &= 0xFFFFFFEF;
        }

        // DO7
        if (dwIoData & 0x00000080)
        {
            MCF_GPT_GMS0 |= 0x00000010;
        }
        else
        {
            MCF_GPT_GMS0 &= 0xFFFFFFEF;
        }

        dwIoData &= 0x003FFF01;
        if (uiPldVer_l == 0)
        {
            // DO0 (LED D0), DO8...DO23 (until 4158.2)
            *(DWORD*)(pCS1BaseAddr_g+0x0C) = dwIoData;
        }
        else
        {
            // DO0 (LED D0), DO8...DO21 (since 4158.4)
            PLD_Write (PLD_REG_DO_STATE, dwIoData);
        }


    }
    #endif
    //---------------------------------------------------------------


    iRet = CF54DRV_RES_OK;


    TRACE("IODRV: - cf54basio#PLCcoreCF54DrvCmdSetDigiOut (iRet=%d)\n", iRet);
    return (iRet);

}



//---------------------------------------------------------------------------
//  Private: Get current timer ticks in [ms]
//---------------------------------------------------------------------------

static  DWORD  PLCcoreCF54DrvGetTickCount (void)
{

struct timeval  tvCurrTime;
DWORD           dwTickCount;


    do_gettimeofday (&tvCurrTime);

    dwTickCount =  tvCurrTime.tv_sec  * 1000;
    dwTickCount += tvCurrTime.tv_usec / 1000;

    return ((DWORD)dwTickCount);

}



//---------------------------------------------------------------------------
//  Private: Read function for PROC-FS read access
//---------------------------------------------------------------------------

#ifdef _CFG_PROCFS_
static  int  PLCcoreCF54DrvProcRead (
    char* pcBuffer_p,
    char** ppcStart_p,
    off_t Offset_p,
    int nBufferSize_p,
    int* pEof_p,
    void* pData_p)
{

tCF54HwInfo CF54HwInfo;
int         nSize;
int         Eof;
int         iRet;
BYTE        bValue;
tCF54DigiIn CF54DigiIn;
DWORD       dwReg0;
DWORD       dwReg1;


    TRACE("IODRV: + cf54basio#PLCcoreCF54DrvProcRead (Offset_p=%d, nBufferSize_p=%d)...\n", (int)Offset_p, nBufferSize_p);
    TRACE("IODRV: Parameters: pcBuffer_p=0x%08lX, *ppcStart_p=0x%08lX, *pEof_p=%d\n", (DWORD)pcBuffer_p, (DWORD)*ppcStart_p, *pEof_p);


    nSize = 0;
    Eof   = 0;


    //---------------------------------------------------------------
    // generate static information
    //---------------------------------------------------------------

    // ---- Driver information ----
    nSize += snprintf (pcBuffer_p + nSize, nBufferSize_p - nSize,
                       "Driver version:         %u.%02u  Type: %u\n",
                       DRV_VER_MAIN, DRV_VER_REL, uiPldVer_l);

    nSize += snprintf (pcBuffer_p + nSize, nBufferSize_p - nSize,
                       "Assigned major number:  %d\n",
                       nDrvMajorNumber_g);


    PLCcoreCF54DrvCmdGetHardwareInfo (&CF54HwInfo);
    nSize += snprintf (pcBuffer_p + nSize, nBufferSize_p - nSize,
                       "Hardware:  CPU Board:   %u.%02u (#%02XH)\n",
                       (WORD)CF54HwInfo.m_wCpuPcbVersion,
                       (WORD)CF54HwInfo.m_bCpuPcbRevision,
                       (WORD)CF54HwInfo.m_bCpuHwId);

    dwReg0 = PLD_Read (PLD_REG_VERSION);
    dwReg1 = PLD_Read (0x04);
    nSize += snprintf (pcBuffer_p + nSize, nBufferSize_p - nSize,
                       "           CPU PLD:     %u.%02u    (#%02XH)"
                       "    (Reg0=%lXH, Reg1=%lXH)\n",
                       (WORD)CF54HwInfo.m_wCpuPldVersion,
                       (WORD)CF54HwInfo.m_bCpuPldRevision,
                       (WORD)CF54HwInfo.m_bCpuPldType,
                       dwReg0,
                       dwReg1);
    nSize += snprintf (pcBuffer_p + nSize, nBufferSize_p - nSize,
                       "           IO Board:    %u.%02u (#%02XH)"
                       "    (Reg2=%lXH, Reg3=%lXH)\n",
                       (WORD)CF54HwInfo.m_wIoPcbVersion,
                       (WORD)CF54HwInfo.m_bIoPcbRevision,
                       (WORD)CF54HwInfo.m_bIoHwId,
                       PLD_Read (0x08),
                       PLD_Read (0x0C));
    nSize += snprintf (pcBuffer_p + nSize, nBufferSize_p - nSize,
                       "Driver:    Config:      %04XH\n",
                       (WORD)CF54HwInfo.m_wCfgDriver);


    //---------------------------------------------------------------
    // generate process information
    //---------------------------------------------------------------

    iRet = PLCcoreCF54DrvCmdGetDipSwitch(&bValue);

    nSize += snprintf (pcBuffer_p + nSize, nBufferSize_p - nSize,
                       "DipSwitch:              %02XH\n",
                       (WORD)bValue);

    iRet = PLCcoreCF54DrvCmdGetHexSwitch(&bValue);

    nSize += snprintf (pcBuffer_p + nSize, nBufferSize_p - nSize,
                       "HexSwitch:              %02X\n",
                       (WORD)bValue);

    iRet = PLCcoreCF54DrvCmdGetRSMSwitch(&bValue);

    nSize += snprintf (pcBuffer_p + nSize, nBufferSize_p - nSize,
                       "RSMSwitch:              %XH\n",
                       (WORD)bValue);

    iRet = PLCcoreCF54DrvCmdGetDigiIn(&CF54DigiIn);

    nSize += snprintf (pcBuffer_p + nSize, nBufferSize_p - nSize,
                       "DigiIn:                 %02X%02X%02X%02XH\n",
                       (WORD) CF54DigiIn.m_bDiByte3,
                       (WORD) CF54DigiIn.m_bDiByte2,
                       (WORD) CF54DigiIn.m_bDiByte1,
                       (WORD) CF54DigiIn.m_bDiByte0);

    // ...
    Eof = 1;


    *pEof_p = Eof;

    TRACE("IODRV: - cf54basio#PLCcoreCF54DrvProcRead (nSize=%d, *pEof_p=%d)\n", nSize, *pEof_p);

    return (nSize);

}


//---------------------------------------------------------------------------
//  Write function for PROC-FS write access
//---------------------------------------------------------------------------

static int PLCcoreCF54DrvProcWrite(struct file *file, const char __user *buffer, unsigned long count, void *data)
{
char            abBuffer[count + 1];
int             iErr;
tCF54DigiOut    CF54DigiOut;
int             iVal = 0;   // digital outputs
int             iVal2 = 0;  // run/error LEDs
DWORD           dwVal;

    if (count > 0)
    {
        iErr = copy_from_user(abBuffer, buffer, count);
        if (iErr != 0)
        {
            return count;
        }
        abBuffer[count] = '\0';

        iErr = sscanf(abBuffer, "%i %i", &iVal, &iVal2);

        if (iErr > 0)
        {   // at least one number was parsed from the string

            dwVal = (DWORD) iVal;

            CF54DigiOut.m_bDoByte0 = dwVal & 0xFF;
            dwVal >>= 8;
            CF54DigiOut.m_bDoByte1 = dwVal & 0xFF;
            dwVal >>= 8;
            CF54DigiOut.m_bDoByte2 = dwVal & 0xFF;
            dwVal >>= 8;
            CF54DigiOut.m_bDoByte3 = dwVal & 0xFF;

            if (iErr > 1)
            {   // value for run/error LED was specified
                PLCcoreCF54DrvCmdSetRunLED(iVal2);
                iVal2 >>= 1;
                PLCcoreCF54DrvCmdSetErrLED(iVal2);
            }

            iErr = PLCcoreCF54DrvCmdSetDigiOut(&CF54DigiOut);
        }
        else
        {   // no valid number supplied
        }

    }

    return count;
}


#endif



// EOF

