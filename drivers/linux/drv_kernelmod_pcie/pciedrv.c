/**
********************************************************************************
\file   pciedrv.c

\brief  Linux kernel driver for openPOWERLINK PCIe interface card

This module handles initialization and hardware specific operations of the
openPOWERLINK PCIe interface card.

It initializes and cleans up the PCIe hardware and maps the bus memory into
kernel space. It also enables and handles the PCIe interrupts from the
openPOWERLINK driver.

\ingroup module_driver_linux_kernel_pcie
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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/pci.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/major.h>
#include <linux/version.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/atomic.h>
#include <asm/irq.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/gfp.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 26)
#include <linux/semaphore.h>
#endif

#include <common/driver.h>
#include <kernel/pdokcal.h>
#include "pciedrv.h"

//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 19)
#error "Linux Kernel versions older 2.6.19 are not supported by this driver!"
#endif

#define OPLK_MAX_BAR_COUNT      6   // Maximum BARs polled in the PCIe

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

\brief PCIe BAR information

The structure holds the information of the PCIe BAR mapped by this driver.

*/
typedef struct
{
    ULONG       busAddr;            ///< Physical address of the BAR.
    ULONG       virtualAddr;        ///< Virtual address of the BAR in kernel memory.
    ULONG       length;             ///< Length of the BAR.
} tBarInfo;

/**

\brief PCIe Driver instance information

The structure holds the information of this PCIe driver instance.

*/
typedef struct
{
    struct pci_dev*     pPciDev;                        ///< Pointer to PCI device structure.
    tBarInfo            barInfo[OPLK_MAX_BAR_COUNT];    ///< Bar instances of the PCIe interface.
    irqCallback         cbSync;                         ///< Sync irq callback function of the upper user layer.
    BOOL                fSyncEnabled;                   ///< Flag to check if sync irq for user has been enabled.
} tPcieDrvInstance;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static INT          initOnePciDev(struct pci_dev* pPciDev_p,
                                  const struct pci_device_id* pId_p);
static void         removeOnePciDev(struct pci_dev* pPciDev_p);
static irqreturn_t  pcieDrvIrqHandler(INT irqNum_p, void* ppDevInstData_p);

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------

static struct pci_device_id     aDriverPciTbl_l[] =
{
    {0x1677, 0xe53f, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0},  // APC2100
    {0x1677, 0xe809, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0},  // APC2100
    {0x1172, 0xe001, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0},  // DE2i150
    {0, }
};

MODULE_DEVICE_TABLE(pci, aDriverPciTbl_l);

static struct pci_driver        oplkPcieDriver_l;

static tPcieDrvInstance         pcieDrvInstance_l;

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  openPOWERLINK PCIe driver initialization

This function initializes the openPOWERLINK PCIe driver.

\return The function returns a tOplkError error code.

\ingroup module_driver_linux_kernel_pcie
*/
//------------------------------------------------------------------------------
tOplkError pciedrv_init(void)
{
    tOplkError      ret = kErrorOk;
    INT             result;

    // Clear instance structure
    OPLK_MEMSET(&pcieDrvInstance_l, 0, sizeof(pcieDrvInstance_l));

    // Clear driver structure
    OPLK_MEMSET(&oplkPcieDriver_l, 0, sizeof(oplkPcieDriver_l));
    oplkPcieDriver_l.name         = PLK_DRV_NAME;
    oplkPcieDriver_l.id_table     = aDriverPciTbl_l;
    oplkPcieDriver_l.probe        = initOnePciDev;
    oplkPcieDriver_l.remove       = removeOnePciDev;

    // Register PCI driver
    result = pci_register_driver(&oplkPcieDriver_l);
    if (result != 0)
    {
        DEBUG_LVL_ERROR_TRACE("%s pci_register_driver failed with %d\n",
                              __FUNCTION__, result);
        ret = kErrorNoResource;
        goto Exit;
    }

    if (pcieDrvInstance_l.pPciDev == NULL)
    {
        DEBUG_LVL_ERROR_TRACE("%s pPciDev=NULL\n", __FUNCTION__);
        ret = pciedrv_shutdown();
        ret = kErrorNoResource;
        goto Exit;
    }

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  openPOWERLINK PCIe driver shutdown

This function shuts down the openPOWERLINK PCIe driver.

\return The function returns a tOplkError error code.

\ingroup module_driver_linux_kernel_pcie
*/
//------------------------------------------------------------------------------
tOplkError pciedrv_shutdown(void)
{
    // Unregister PCI driver
    DEBUG_LVL_DRVINTF_TRACE("%s calling pci_unregister_driver()\n", __FUNCTION__);
    pci_unregister_driver(&oplkPcieDriver_l);

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Get PCIe BAR virtual address

This routine fetches the BAR address of the requested BAR.

\param  barCount_p     ID of the requested BAR.

\return Returns the address of requested PCIe BAR.

\ingroup module_driver_linux_kernel_pcie
*/
//------------------------------------------------------------------------------
ULONG pciedrv_getBarAddr(UINT8 barCount_p)
{
    if (barCount_p >= OPLK_MAX_BAR_COUNT)
    {
        return 0;
    }

    return pcieDrvInstance_l.barInfo[barCount_p].virtualAddr;
}

//------------------------------------------------------------------------------
/**
\brief  Get PCIe BAR physical address

This routine fetches the physical BAR address of the requested BAR.

\param  barCount_p     ID of the requested BAR.

\return Returns the physical address of requested PCIe BAR.

\ingroup module_driver_linux_kernel_pcie
*/
//------------------------------------------------------------------------------
ULONG pciedrv_getBarPhyAddr(UINT8 barCount_p)
{
    if (barCount_p >= OPLK_MAX_BAR_COUNT)
    {
        return 0;
    }

    return pcieDrvInstance_l.barInfo[barCount_p].busAddr;
}

//------------------------------------------------------------------------------
/**
\brief  Get BAR Length

\param  barCount_p     ID of the requested BAR.

\return Returns the length of requested PCIe BAR.

\ingroup module_driver_linux_kernel_pcie
*/
//------------------------------------------------------------------------------
ULONG pciedrv_getBarLength(ULONG barCount_p)
{
    if (barCount_p >= OPLK_MAX_BAR_COUNT)
    {
        return 0;
    }

    return pcieDrvInstance_l.barInfo[barCount_p].length;
}

//------------------------------------------------------------------------------
/**
\brief  Register user sync interrupt callback

This function stores the user sync event callback function tobe called from
the PCIe sync ISR.

\param cbSync_p     Pinter to the user sync callback function.

\return The function returns a tOplkError error code.

\ingroup module_driver_linux_kernel_pcie
*/
//------------------------------------------------------------------------------
tOplkError pciedrv_regSyncHandler(irqCallback cbSync_p)
{
    pcieDrvInstance_l.cbSync = cbSync_p;
    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Enable/Disable user sync interrupt

Enables or disable the forwarding of sync interrupt to user.

\param fEnable_p    Boolean value indicating whether or not to forward sync irq
                    to user.

\return The function returns a tOplkError error code.

\ingroup module_driver_linux_kernel_pcie
*/
//------------------------------------------------------------------------------
tOplkError pciedrv_enableSync(BOOL fEnable_p)
{
    pcieDrvInstance_l.fSyncEnabled = fEnable_p;
    return kErrorOk;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief  openPOWERLINK PCIe driver interrupt handler

This function is the interrupt service routine for the openPOWERLINK PCIe driver.

\param  irqNum_p            IRQ number
\param  ppDevInstData_p     Pointer to private data provided by request_irq

\return The function returns an IRQ handled code.
*/
//------------------------------------------------------------------------------
static irqreturn_t pcieDrvIrqHandler(INT irqNum_p, void* ppDevInstData_p)
{
    INT         ret = IRQ_HANDLED;

    UNUSED_PARAMETER(irqNum_p);
    UNUSED_PARAMETER(ppDevInstData_p);

    // Currently only sync interrupt is produced by the PCIe, check if the user
    // wants the irq to be forwarded
    if ((pcieDrvInstance_l.cbSync != NULL) &&
        (pcieDrvInstance_l.fSyncEnabled == TRUE))
    {
        // User wants the interrupt, forward it without any argument
        pcieDrvInstance_l.cbSync();
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Initialize one PCI device

This function initializes one PCI device.

\param  pPciDev_p   Pointer to corresponding PCI device structure
\param  pId_p       PCI device ID

\return The function returns an integer error code.
\retval 0           Successful
\retval Otherwise   Error
*/
//------------------------------------------------------------------------------
static INT initOnePciDev(struct pci_dev* pPciDev_p,
                         const struct pci_device_id* pId_p)
{
    INT         result = 0;
    UINT8       barCount = 0;
    tBarInfo*   pBarInfo = NULL;

    UNUSED_PARAMETER(pId_p);

    if (pcieDrvInstance_l.pPciDev != NULL)
    {
        // This driver is already connected to a PCIe device
        DEBUG_LVL_DRVINTF_TRACE("%s device %s discarded\n",
                                __FUNCTION__, pci_name(pPciDev_p));
        result = -ENODEV;
        goto Exit;
    }

    pcieDrvInstance_l.pPciDev = pPciDev_p;

    // Enable the PCIe device
    DEBUG_LVL_DRVINTF_TRACE("%s enable device\n", __FUNCTION__);
    result = pci_enable_device(pPciDev_p);
    if (result != 0)
    {
        goto Exit;
    }

    DEBUG_LVL_DRVINTF_TRACE("%s request PCIe regions\n", __FUNCTION__);
    result = pci_request_regions(pPciDev_p, PLK_DRV_NAME);
    if (result != 0)
    {
        goto ExitFail;
    }

    // Ignoring whether or not any BAR is accessible
    for (barCount = 0; barCount < OPLK_MAX_BAR_COUNT; barCount++)
    {
        pBarInfo = &pcieDrvInstance_l.barInfo[barCount];

        if (pBarInfo->virtualAddr != (ULONG)NULL)
        {
            // The instance is already present
            result = -EIO;
            goto ExitFail;
        }

        // Look for the MMIO BARs
        if ((pci_resource_flags(pPciDev_p, barCount) & IORESOURCE_MEM) == 0)
        {
            continue;
        }

        // get the size of this field
        pBarInfo->length = pci_resource_len(pPciDev_p, barCount);

        // $$: Add check for weird broken IO regions

        pBarInfo->virtualAddr = (ULONG)ioremap_nocache(pci_resource_start(pPciDev_p,
                                                       barCount),
                                                       pBarInfo->length);
        if (pBarInfo->virtualAddr == (ULONG)NULL)
        {
            // Remap of controller's register space failed
            result = -EIO;
            goto ExitFail;
        }

        pBarInfo->busAddr = (ULONG)pci_resource_start(pPciDev_p, barCount);

        DEBUG_LVL_DRVINTF_TRACE("%s() --> ioremap\n", __FUNCTION__);
        DEBUG_LVL_DRVINTF_TRACE("\tbar#\t%u\n", barCount);
        DEBUG_LVL_DRVINTF_TRACE("\tbarLen\t%lu\n", pBarInfo->length);
        DEBUG_LVL_DRVINTF_TRACE("\tbarMap\t0x%lX\n", pBarInfo->virtualAddr);
        DEBUG_LVL_DRVINTF_TRACE("\tbarPhy\t0x%lX\n", pBarInfo->busAddr);
    }

    // Enable PCI busmaster
    DEBUG_LVL_DRVINTF_TRACE("%s enable busmaster\n", __FUNCTION__);
    pci_set_master(pPciDev_p);

    // Enable msi
    DEBUG_LVL_DRVINTF_TRACE("Enable MSI\n");
    result = pci_enable_msi(pPciDev_p);
    if (result != 0)
    {
        DEBUG_LVL_DRVINTF_TRACE("%s Could not enable MSI\n", __FUNCTION__);
    }

    // Install interrupt handler
    DEBUG_LVL_DRVINTF_TRACE("%s install interrupt handler\n", __FUNCTION__);
    result = request_irq(pPciDev_p->irq,
                         pcieDrvIrqHandler,
                         IRQF_SHARED,
                         PLK_DRV_NAME, /* pPciDev_p->dev.name */
                         pPciDev_p);
    if (result != 0)
    {
        goto ExitFail;
    }

    goto Exit;

ExitFail:
    removeOnePciDev(pPciDev_p);

Exit:
    DEBUG_LVL_DRVINTF_TRACE("%s finished with %d\n", __FUNCTION__, result);
    return result;
}

//------------------------------------------------------------------------------
/**
\brief  Remove one PCI device

This function removes one PCI device.

\param  pPciDev_p     Pointer to corresponding PCI device structure
*/
//------------------------------------------------------------------------------
static void removeOnePciDev(struct pci_dev* pPciDev_p)
{
    UINT8       barCount = 0;
    tBarInfo*   pBarInfo = NULL;

    if (pcieDrvInstance_l.pPciDev != pPciDev_p)
    {
        // Trying to remove unknown device
        BUG_ON(pcieDrvInstance_l.pPciDev != pPciDev_p);
        goto Exit;
    }

    // Remove interrupt handler
    if (pPciDev_p->irq != (INT)NULL)
        free_irq(pPciDev_p->irq, pPciDev_p);

    // Disable Message Signaled Interrupt
    DEBUG_LVL_DRVINTF_TRACE("%s Disable MSI\n", __FUNCTION__);
    pci_disable_msi(pPciDev_p);

    // unmap controller's register space
    for (barCount = 0; barCount < OPLK_MAX_BAR_COUNT; barCount++)
    {
        pBarInfo = &pcieDrvInstance_l.barInfo[barCount];

        if (pBarInfo->virtualAddr != (ULONG)NULL)
        {
            iounmap((void __iomem*)pBarInfo->virtualAddr);
            pBarInfo->virtualAddr = (ULONG)NULL;
        }
    }

    // disable the PCI device
    pci_disable_device(pPciDev_p);

    // release memory regions
    pci_release_regions(pPciDev_p);

    pcieDrvInstance_l.pPciDev = NULL;

Exit:
    return;
}

/// \}
