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
