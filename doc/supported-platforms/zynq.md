openPOWERLINK on Xilinx Zynq SoC {#page_platform_zynq}
================================

[TOC]

# Introduction {#sect_zynq_intro}

This file contains documentation for the openPOWERLINK stack on a Xilinx
Zynq SoC. On Zynq, openPOWERLINK can be running under Linux. This page will
give an overview of the supported environments and explains the steps to
build and run openPOWERLINK on Zynq SoC.

Currently, openPOWERLINK can run under the following environments on a Zynq SoC:

- __Linux on Zynq ARM__
  openPOWERLINK runs on Linux which is running on the ARM processing system (PS)
  of the SoCprocessor under Linux.

# Requirements {#sect_zynq_requirements}

## Development Boards {#sect_zynq_requirements_boards}

- Xilinx ZC702 Development Kit.
- Any other board with Zynq 7000 series SoC.

## POWERLINK network {#sect_zynq_requirements_network}

- POWERLINK network with a controlled nodes (CN)
  * openPOWERLINK controlled nodes, e.g. Linux
  * B&R POWERLINK controlled nodes

- POWERLINK network with a managing node (MN)
  * openPOWERLINK managing node, e.g. Linux
  * B&R POWERLINK managing node

## Tools {#sect_zynq_requirements_tools}

### Xilinx ISE
The following tool is necessary to evaluate a Xilinx Zynq based openPOWERLINK
solution:

* `Xilinx ISE - Embedded Edition` which is called `ISE Design Suite - 14.7 Full
  Product Installation` or `ISE Design Suite - 14.7 Embedded Edition` and can be
  downloaded from: http://www.xilinx.com/support/download/index.html. The evaluation license for a period of 30 days
  can be acquired from the `Xilinx Licensing Site` and is free for an evaluation
  product.

### CMake

For building the openPOWERLINK stack and demo applications, the Open Source
cross-platform build tool CMake is used (<http://www.cmake.org>). CMake
version V2.8.7 or higher is required.

For a detailed description of CMake look at the [cmake section](\ref sect_build_cmake).

### libpcap Library

In order to use the user-space POWERLINK stack under Linux, the libpcap library is needed
to access the Ethernet interface.

__NOTE:__ The libpcap library has to be cross-compiled for the ARM target in order to use
it with the Zynq SoC demo.

### Terminal software

A terminal software is required to interact with the device via a serial interface.
It will provide console access (keyboard input, text output) to the system.

Minicom is an Open Source terminal software. It is available for Debian and Ubuntu users
via apt-get:

    > sudo apt-get install minicom

TeraTerm is a program for printing out text which is transmitted over a serial
interface. It is open source and can be downloaded from: http://ttssh2.sourceforge.jp/

### Linux Kernel

A Linux kernel for the Zynq platform is required in order to run openPOWERLINK
on Linux. The Linux port for the Zynq platform is available on github
and can be downloaded or cloned from: https://github.com/Xilinx/linux-xlnx.

Linux kernel version 3.10 tagged as xilinx-v14.7 on github was used for testing at
the time of writing this document.

The steps to cross-compile Linux for Zynq and information about availability of other
libraries for Zynq can be found here: http://www.wiki.xilinx.com/Zynq+Linux.

# Linux on Zynq ARM {#sect_zynq_linux}

On the Zynq SoC, openPOWERLINK runs on a Linux OS which is running on the
ARM Cortex A9 processing system (PS) of the SoC. The following section contains
additional information about the openPOWERLINK Linux implementation on the Zynq SoC.

For general information about running openPOWERLINK on Linux refer to \ref page_platform_linux.

__NOTE__: Third-party libraries such as libpcap, and libqt etc. must also be
          cross-compiled for Zynq ARM apart from openPOWERLINK.

## Contents {#sect_zynq_linux_contents}
The following modules have been added to the openPOWERLINK stack to support the Zynq SoC:

- Edrv module for the Gigabit Ethernet controller on Zynq 7000 series SoCs (emacps).
- High-resolution timer module for the triple timer counter on Zynq 7000 series SoCs.

## openPOWERLINK Stack Components {#sect_zynq_linux_components}

The following section contains a description of the [openPOWERLINK components](\ref page_components)
which are available.

### Stack Libraries {#sect_zynq_linux_components_libs}

Please refer to [Linux Stack Libraries](\ref sect_linux_components_libs) for
information about the available stack libraries on Linux.

### Demo Applications  {#sect_zynq_linux_components_apps}

The following demo application are supported on Linux:

* [demo_mn_console](\ref sect_components_demo_mn_console)
* [demo_cn_console](\ref sect_components_demo_cn_console)

### Drivers  {#sect_zynq_linux_components_drivers}

Please refer to [Linux Drivers] (\ref sect_linux_components_drivers) for information
about the available drivers.

## Building {#sect_zynq_linux_build}

For cross-compiling openPOWERLINK for Linux on Zynq ARM refer to the
[generic build instructions](\ref page_build) and execute all the required build
steps for this section. The following build steps are valid for Linux on Zynq ARM:

* [Build the openPOWERLINK stack libraries](\ref sect_build_stack_build_linux)
* [Build Linux PCAP User Space Daemon](\ref sect_build_drivers_build_linux_pcap)
* [BUild Linux Edrv Kernel Driver](\ref sect_build_drivers_build_linux_edrv)
* [Build your application (or a delivered demo application)](\ref sect_build_demos_build_linux)

## Running openPOWERLINK {#sect_zynq_linux_running}

The following section will explain the steps required to run the Linux
implementation of openPOWERLINK on the Zynq ARM.

### Linux MN/CN console demo

In order to run the openPOWERLINK Linux demo applications on theZynq ARM,
follow the steps listed below:

* Prepare the SD Card for boot-up with following:
  - uImage : Linux kernel image
  - uramdisk.image.gz : Initramfs file
  - devicetree.dtb : Device tree blob
  - boot.bin : Zynq boot image (generated using first stage bootloader(fsbl.elf),
               u-boot binary (u-boot-elf) and system.bit)
* Copy the stack binaries to the SD card.
* Connect the host PC to the Zynq board using the serial interface on board.
* Use a terminal program to connect to the Linux console on Zynq.
* For starting the application, refer to [running openPOWERLINK on Linux] (\ref sect_linux_running).
