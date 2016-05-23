Known Issues of openPOWERLINK Protocol Stack {#page_known_issues}
============================================

[TOC]

# DE2i-150 board flash support {#sect_known_issues_de2i150_flash}

## Description
Programming the firmware onto the serial flash device is not supported.

## Workaround
Use JTAG to download the FPGA bitstream and Nios II elf manually.

# Windows PCIe network device {#sect_known_issues_winpcie_netdev}

## Description
Windows network device actions such as disable, restart and enable network
adapter are not supported.

## Workaround
Don't use these actions.

# Linux PCIe driver signing {#sect_known_issues_linpcie_devsign}

## Description
During kernel module insertion driver signing warnings are raised on systems
with CONFIG_MODULE_SIG enabled in the Linux kernel configuration.

## Workaround
The warnings can be ignored.

# Linux PCIe CPU load {#sect_known_issues_linpcie_load}

## Description
High CPU load could be caused because of low POWERLINK cycle times and
high PDO sizes.

## Workaround
Use the application synchronization period configuration
(\ref tOplkApiInitParam::minSyncTime) to reduce the frequency of
synchronization processing.

# NDIS intermediate driver multiple network adapters {#sect_known_issues_ndis_multi}

## Description
In a system with multiple network adapters it is not possible to select
the adapter to be used. The driver is configured to automatically select
the first driver for network communication.

## Workaround
Select first adapter for openPOWERLINK.

# Linux kernel module with edrv-i210 unhandled IRQ {#sect_known_issues_linkern_i210}

## Description
For the configuration of openPOWERLINK MN which uses edrv-i210 kernel module,
after stopping the MN application, the Linux kernel reports a warning for 'unhandled IRQ'
for the interrupt pin assigned to Intel's i210 NIC. The warning message is

_irq XX: nobody cared (try booting with the "irqpoll" option'_

This occurs on PCs where Intel's i210 NIC card shares an interrupt pin with any
of the other devices such as USB hub controller.

## Workaround
The warning can be ignored as it does not affect the functionality and performance.
