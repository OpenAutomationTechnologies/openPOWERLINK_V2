Known Issues of openPOWERLINK Protocol Stack {#page_known_issues}
============================================

[TOC]

# Zynq hybrid: Microblaze BSP build fails due to missing openmac_cfg.h {#sect_known_issues_zynqhybrid_bsp}

## Description
Building the Microblaze BSP for the Zynq hybrid design fails due to missing openmac_cfg.h file.

        > In file included from /somewhere/stack/src/kernel/edrv/edrv-openmac.c:49:0:
        > /somewhere/stack/include/target/openmac.h:45:25: fatal error: openmac_cfg.h: No such file or directory

## Workaround
Vivado selects the wrong openMAC IP-Core, thus, make sure that no openMAC IP-Core intended for ISE can be
found by Vivado. In openPOWERLINK repository, simply remove the pcores directory:

        > rm -rf <OPLK>/hardware/ipcore/xilinx/components/pcores

# Zynq emacps: Kernel panic {#sect_known_issues_emacps_panic}

## Description
The openPOWERLINK kernel module results in a kernel panic if the application is restarted.

# Zynq emacps: Driver load fails {#sect_known_issues_emacps_drvloadfail}

## Description
The openPOWERLINK kernel module fails to load while restarting the MN application.

# Zynq emacps: MN fails in case of high PDO load {#sect_known_issues_emacps_mnhighpdoload}

## Description
The Linux openPOWERLINK MN on Zynq with emacps fails to run configurations with high PDO load.

The following configuration was tested:

MN              | CN               | CDC
----------------| ---------------- | -------------------------
demo-mn-console | X20 Bus couplers | 30CN/252TPDO/104RPDO/10ms

Error:

        > EVENT HISTORY Source = EventSourceEventk (0x06) OplkError = No free entry in internal buffer table for Tx frames (0x014) OrgSource = EventSourceNmtk 0x02
        > EVENT STATE_CHANGE   NmtMsNotActive->NmtGsOff Originating event:NmtEventCriticalError

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

`irq XX: nobody cared (try booting with the "irqpoll" option)`

This occurs on PCs where Intel's i210 NIC card shares an interrupt pin with any
of the other devices such as USB hub controller.

## Workaround
The warning can be ignored as it does not affect the functionality and performance.

# MN demo application compilation on 32 bit Windows with NDIS drivers {#sect_know_issues_win32_compilation}

## Description
The MN demo application (demo_mn_console) fails to compile on a 32 bit Windows system while using the
`Kernel stack on PCIe card` or `Windows Kernel Module` [stack libraries] (\ref sect_windows_components_libs)
with following error

        > error LNK1120: 1 unresolved externals
        > error LNK2019: unresolved external symbol _CancelIoEx referenced in function _system_stopSyncThread
        > (apps\demo_mn_console\build\windows\system-windows.obj) demo_mn_console
        > warning C4013: CancelIoEx undefined; assuming extern returning int (apps\common\src\system\system-windows.c) demo_mn_console

The error is produced due to missing compiler options to specify the required 32 bit
version of the system library which will be resolved in future release.

# Windows PCIe queue corruption with vitual Ethernet {#sect_know_issues_winpcie_veth}

## Description
The openPOWERLINK solution using Windows PCIe design with vitual Ethernet interface enabled, may
lead to issues with the queue corruption. The queue corruption is noticed when there is excessive
non-POWERLINK communication through the virtual Ethernet interface.

## Workaround
On systems which do not require virtual Ethernet with the Windows PCIe design,
the issue can be avoided by disabling virtual Ethernet interface (`CONFIG_INCLUDE_VETH`).

# Avnet LX150T MN design initilization {#sect_known_issues_avent-lx150t_initilization}

## Description
The Avnet LX150T MN design fails to initialize randomly reporting error with
the host interface initialization.

Problem arises due to unknown bug in the Xilinx ISE 14.7 which results in inconsistent
placement and routing on modifications to any IP core parameters such as base address,
and internal register configurations.
