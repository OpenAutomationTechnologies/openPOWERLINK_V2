/**
********************************************************************************
\file   zynqdrv.c

\brief  Linux kernel driver for Zynq/FPGA

This module handles initialization and hardware specific operations of the
Zynq/FPGA - Linux on ARM + Microblaze design.

It facilitates the communication between ARM and Microblaze. It also enables
and handles the openMAC timer pulse interrupt from the openPOWERLINK driver.

\ingroup module_driver_linux_kernel_zynq
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2016, Kalycito Infotech Private Limited
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

#include "zynqdrv.h"

#include <common/driver.h>

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/version.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/atomic.h>
#include <asm/irq.h>
#include <linux/sched.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/gfp.h>
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 4, 0))
#include <linux/semaphore.h>
#endif


//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 4, 0))
#error "Linux Kernel versions older 4.4.0 are not supported by this driver!"
#endif

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
#define MB_RESET_PIN             960         // GPIO Pin number - Microblaze reset
#define MICROBLAZE_RESET           1         // GPIO Pin - set value

//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------
/**

 \brief Platform device IO memory resource

 The structure holds the information of the IO memory resource
 and the size of the memory regions.

 */
typedef struct
{
    struct resource*             pIoMemRes;        ///< Pointer to the IO Memory resource allocated by Linux.
    void*                        pIoMemBase;       ///< Virtual address of IO memory regions.
    resource_size_t              IoMemSize;        ///< Size of the memory allocated for the device.
} tIoMemoryResource;

/**

 \brief Platform device Interrupt Resource

 The structure holds the information of the Irq resource and
 the IRQ number assigned to Zynq platform device.

 */
typedef struct
{
    struct resource*             pIrqRes;          ///< Interrupt resource for the platform device.
    UINT32                       IrqNum;           ///< Interrupt number assigned by Linux.
} tIrqResource;

/**
 \brief  Zynq kernel driver instance

 Provides all the necessary information used by the Zynq kernel driver module
 to interact with the device and interface with the stack above it.
 */

typedef struct
{
    struct platform_device*      pPlatformDev;                          ///< Pointer to platform device structure for driver.
    tIoMemoryResource            aIoMemRes[kIoMemRegionLast];           ///< Memory instances of the platform device.
    tIrqResource                 IrqResource;                           ///< Interrupt resource for the platform device.
    tIrqCallback                 pfnCbSync;                             ///< Sync irq callback function of the upper user layer.
    BOOL                         fSyncEnabled;                          ///< Flag to check if sync irq for user has been enabled.
} tPcpDrvInstance;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static int          initOnePlatformDev(struct platform_device* pDev_p);
static int          removeOnePlatformDev(struct platform_device* pDev_p);
static irqreturn_t  pcpIrqHandler(int irqNum_p,
                                  void* ppDevInstData_p);

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
// Platform device identification
#if defined(CONFIG_OF)
static struct of_device_id  drv_of_match[] =
{
    {   .compatible = "plk_driver,DDR", },               // __devinitdata creates warning!
    { /* end of table */}                                // keep devinit in separate data section,
};                                                       // linker is not able to link

MODULE_DEVICE_TABLE(of, drv_of_match);
#else
#define drv_of_match    NULL
#endif /* CONFIG_OF */

static struct platform_driver   pcpDriver_l =
{
    .probe      = initOnePlatformDev,
    .remove     = removeOnePlatformDev,
    .suspend    = NULL,                                     // Not handling power management functions
    .resume     = NULL,                                     // Not handling power management functions
    .driver     = { .name = "zynq-platform-device",
                    .owner = THIS_MODULE,
                    .of_match_table = drv_of_match,         // This function is to check the device
                  },                                        // from the device tree
};

static tPcpDrvInstance instance_l;

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  openPOWERLINK zynq kernel driver initialization

This function initializes the openPOWERLINK zynq driver.

\return The function returns a tOplkError error code.

\ingroup module_driver_linux_kernel_zynq
*/
//------------------------------------------------------------------------------
tOplkError zynqdrv_init(void)
{
    tOplkError  ret = kErrorOk;
    INT         result;

    // Clear instance structure
    OPLK_MEMSET(&instance_l, 0, sizeof(instance_l));

    DEBUG_LVL_ALWAYS_TRACE("%s(): Registering the driver to the kernel...", __func__);

    /*
     * TODO: This function can be replaced with platform_driver_probe
     *       to reduce memory footprints
     */
    result = platform_driver_register(&pcpDriver_l);
    if (result != 0)
        return kErrorNoResource;

    DEBUG_LVL_ALWAYS_TRACE("Done\n");
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  openPOWERLINK zynq kernel driver shutdown

This function shuts down the openPOWERLINK zynq driver.

\return The function returns a tOplkError error code.

\ingroup module_driver_linux_kernel_zynq
*/
//------------------------------------------------------------------------------
tOplkError zynqdrv_exit(void)
{
    DEBUG_LVL_DRVINTF_TRACE("%s(): Calling platform_driver_unregister()\n", __func__);
    platform_driver_unregister(&pcpDriver_l);

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Get common memory base virtual address

This routine fetches the common memory base address after
its been remapped into Linux kernel virtual address space..

 \param[in]      memId_p          ID of the requested memory region.

 \return Returns the base address of common memory.

\ingroup module_driver_linux_kernel_zynq
*/
//------------------------------------------------------------------------------
void* zynqdrv_getMemRegionAddr(tIoMemRegions memId_p)
{
    if (memId_p >= kIoMemRegionLast)
        return NULL;

    return instance_l.aIoMemRes[memId_p].pIoMemBase;
}

//------------------------------------------------------------------------------
/**
\brief  Get shared memory physical address

This routine fetches the physical address of shared Memory.

 \param[in]      memId_p          ID of the requested memory region.

 \return Returns the physical address of Shared Memory.

\ingroup module_driver_linux_kernel_zynq
*/
//------------------------------------------------------------------------------
void* zynqdrv_getMemPhyAddr(tIoMemRegions memId_p)
{
    if (memId_p >= kIoMemRegionLast)
        return NULL;

    return (void*) instance_l.aIoMemRes[memId_p].pIoMemRes->start;
}

//------------------------------------------------------------------------------
/**
\brief  Register user sync interrupt callback

This function stores the user sync event callback function to be called from
the sync ISR.

\param[in]      cbSync_p            Pointer to the user sync callback function.

\return The function returns a tOplkError error code.

\ingroup module_driver_linux_kernel_zynq
*/
//------------------------------------------------------------------------------
tOplkError zynqdrv_regSyncHandler(tIrqCallback cbSync_p)
{
    instance_l.pfnCbSync = cbSync_p;

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Enable/Disable user sync interrupt

Enables or disable the forwarding of sync interrupt to user.

\param[in]      fEnable_p           Boolean value indicating whether or not to
                                    forward sync IRQ to user.

\return The function returns a tOplkError error code.

\ingroup module_driver_linux_kernel_zynq
*/
//------------------------------------------------------------------------------
tOplkError zynqdrv_enableSync(BOOL fEnable_p)
{
    instance_l.fSyncEnabled = fEnable_p;

    return kErrorOk;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief  openPOWERLINK Zynq kernel driver interrupt handler

This function is the interrupt service routine for the openPOWERLINK zynq driver.

\param[in]      irqNum_p            IRQ number
\param[in]      ppDevInstData_p     Pointer to private data provided by request_irq

\return The function returns an IRQ handled code.
*/
//------------------------------------------------------------------------------
static irqreturn_t pcpIrqHandler(int irqNum_p,
                                 void* ppDevInstData_p)
{
    irqreturn_t ret = IRQ_HANDLED;

    UNUSED_PARAMETER(irqNum_p);
    UNUSED_PARAMETER(ppDevInstData_p);

    if ((instance_l.pfnCbSync != NULL) &&
        (instance_l.fSyncEnabled == TRUE))
    {
        // User wants the interrupt, forward it without any argument
        instance_l.pfnCbSync();
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Initialize one Zynq device

This function initializes one Zynq device.

\param[in,out]  pDev_p              Pointer to corresponding Zynq device structure

\return The function returns an integer error code.
\retval 0                           Successful
\retval Otherwise                   Error
*/
//------------------------------------------------------------------------------
static int initOnePlatformDev(struct platform_device* pDev_p)
{
    INT     result = 0;
    UINT8   memId = 0;

    /* checking if Microblaze Reset pin number is valid */
    if (!gpio_is_valid(MB_RESET_PIN))
    {
        DEBUG_LVL_DRVINTF_TRACE("Microblaze Reset pin not valid\n");
        return 0;
    }

    gpio_request(MB_RESET_PIN, "Reset Button");

    gpio_export(MB_RESET_PIN, false);

    gpio_direction_output(MB_RESET_PIN, MICROBLAZE_RESET);

    /* Reset Microblaze */
    gpio_set_value(MB_RESET_PIN, MICROBLAZE_RESET);

    /* Wait for Microblaze to come out of reset */
    msleep(500);

    if (pDev_p == NULL)
    {
        DEBUG_LVL_DRVINTF_TRACE("%s(): Device discarded\n", __func__);
        result = -ENODEV;
        goto Exit;
    }

    if (instance_l.pPlatformDev != NULL)
    {
        DEBUG_LVL_DRVINTF_TRACE("%s(): Device (%s) already registered\n",
                               __func__, pDev_p->name);
        result = -ENODEV;
        goto Exit;
    }

    // Save the handle for the platform device
    instance_l.pPlatformDev = pDev_p;

    for (memId = 0; memId < kIoMemRegionLast; memId++)
    {
        DEBUG_LVL_DRVINTF_TRACE("%s(): IOMEM resource initialization...%s",
                               __func__,
                               (memId == 0?"Common Memory":"Shared Memory"));

        instance_l.aIoMemRes[memId].pIoMemRes =
                  platform_get_resource(pDev_p, IORESOURCE_MEM, memId);

        if (instance_l.aIoMemRes[memId].pIoMemRes == NULL)
        {
            result = -ENODEV;
            goto ExitClean;
        }

        instance_l.aIoMemRes[memId].IoMemSize =
                  (instance_l.aIoMemRes[memId].pIoMemRes->end -
                  instance_l.aIoMemRes[memId].pIoMemRes->start + 1);

        /* Mark the region exclusively for Zynq device */
        if (!request_mem_region
                  (instance_l.aIoMemRes[memId].pIoMemRes->start,
                  instance_l.aIoMemRes[memId].IoMemSize, PLK_DRV_NAME))
        {
            DEBUG_LVL_DRVINTF_TRACE("Request memory region failed for %s \n",
                                   (memId == 0?"Common Memory":"Shared Memory"));
            result = -ENOMEM;
            goto ExitClean;
        }

        DEBUG_LVL_DRVINTF_TRACE("MEM_RESOURCE: Start 0x(%X), End 0x(%X) \n",
                               instance_l.aIoMemRes[memId].pIoMemRes ->start,
                               instance_l.aIoMemRes[memId].pIoMemRes ->end);

        /* Remap Io memory regions into Linux virtual address space */
        instance_l.aIoMemRes[memId].pIoMemBase =
                  ioremap(instance_l.aIoMemRes[memId].pIoMemRes->start,
                         instance_l.aIoMemRes[memId].IoMemSize);

        if (instance_l.aIoMemRes[memId].pIoMemBase == NULL)
        {
            DEBUG_LVL_DRVINTF_TRACE("Ioremap failed for %s\n",
                                   (memId == 0?"Common Memory":"Shared Memory"));
            result = -EIO;
            goto ExitClean;
        }
    }

    DEBUG_LVL_DRVINTF_TRACE("%s(): IRQ resource initialization...", __func__);
    /* Get Interrupt resource for device */
    instance_l.IrqResource.pIrqRes = platform_get_resource(pDev_p, IORESOURCE_IRQ, 0);
    if (instance_l.IrqResource.pIrqRes == NULL)
    {
        DEBUG_LVL_DRVINTF_TRACE("Failed\n");
        result = -ENODEV;
        goto ExitClean;
    }
    /* Request IRQ */
    DEBUG_LVL_DRVINTF_TRACE("Requesting IRQ resource...");

    if (request_irq(instance_l.IrqResource.pIrqRes->start, pcpIrqHandler, 0,
                   PLK_DRV_NAME, pDev_p))
    {
        DEBUG_LVL_DRVINTF_TRACE("Request IRQ failed \n");
        result = -EIO;
        goto ExitClean;
    }
    instance_l.IrqResource.IrqNum = instance_l.IrqResource.pIrqRes->start;
    goto Exit;

ExitClean:
    DEBUG_LVL_DRVINTF_TRACE("%s(): finished with %d\n", __func__, result);
    removeOnePlatformDev(pDev_p);

Exit:
    DEBUG_LVL_DRVINTF_TRACE("%s(): finished with %d\n", __func__, result);
    return result;
}

//------------------------------------------------------------------------------
/**
\brief  Remove one zynq device

This function removes one zynq device.

\param[in,out]  pPciDev_p           Pointer to corresponding Zynq device structure
*/
//------------------------------------------------------------------------------
static int removeOnePlatformDev(struct platform_device* pDev_p)
{
    UINT8 memId = 0;

    /* Remove interrupt handler */
    if (instance_l.IrqResource.IrqNum != 0)
    {
        free_irq(instance_l.IrqResource.IrqNum, pDev_p);
        instance_l.IrqResource.IrqNum = 0;
    }

    for (memId = 0; memId < kIoMemRegionLast; memId++)
    {
        if (instance_l.aIoMemRes[memId].pIoMemBase != NULL)
        {
            iounmap(instance_l.aIoMemRes[memId].pIoMemBase);
            instance_l.aIoMemRes[memId].pIoMemBase = NULL;
        }

        if (instance_l.aIoMemRes[memId].pIoMemRes != NULL)
        {
            release_mem_region(instance_l.aIoMemRes[memId].pIoMemRes->start,
                              instance_l.aIoMemRes[memId].IoMemSize);
            instance_l.aIoMemRes[memId].pIoMemRes = NULL;
        }
    }
    /* Bring Microblaze to reset */
    gpio_free(MB_RESET_PIN);
    instance_l.pPlatformDev = NULL;
    return 0;
}
/// \}
