openPOWERLINK on Xilinx Zynq SoC {#page_platform_zynq}
==================================

[TOC]

# Introduction {#sect_zynq_intro}

This file contains documentation for the openPOWERLINK stack on Xilinx
Zynq SoC. Currently, the stack only provides components for using a Linux-based demo 
on Zynq.

## Contents {#sect_zynq_intro_contents}

- Edrv module for Gigabit Ethernet controller on Zynq 7000 series SoCs. 
- HighRes module for Triple timer counter on Zynq 7000 series SoCs.

# Requirements {#sect_zynq_requirements}

## Development Boards {#sect_zynq_requirements_boards}

- Xilinx ZC702 Development Kit
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
The following tool is necessary to evaluate a Xilinx Zynq based openPOWERLINK solution:

* `Xilinx ISE - Embedded Edition` which is called `ISE Design Suite - 14.7 Full
  Product Installation` or `ISE Design Suite - 14.7 Embedded Edition` and can be
  downloaded from: http://www.xilinx.com/support/download/index.htm. The license
  can be acquired from the `Xilinx Licensing Site` and is free for an evaluation
  product (30 days).

### CMake

For building the openPOWERLINK stack and demo applications the Open Source
cross-platform build tool CMake is used (<http://www.cmake.org>). CMake
version V2.8 or higher is required.

For a detailed description of CMake look at the
[cmake section](\ref sect_build_cmake).

### libpcap Library

In order to use the user-space POWERLINK stack the libpcap library is needed
to access the Ethernet interface.

The libpcap library has to be cross-compiled in order to use it with the Zynq SoC demo.

### Terminal software

A terminal software is required to interact with the device via a serial interface. 
It will provide console access (keyboard input, text output) to the system.

Minicom is an Open Source terminal software. It is available for Debian and Ubuntu users via apt-get:

    > sudo apt-get install minicom

### Linux Kernel

A Linux kernel version for the Zynq platform is required in order to run the Linux-based demo
of openPOWERLINK. The Linux ports for Zynq platform are available on github and can be 
downloaded or cloned from: https://github.com/Xilinx/linux-xlnx.

Linux kernel version 3.10 tagged as xilinx-v14.7 on github was used for testing at 
the time of writing this document.

The steps to cross-compile Linux for Zynq and information about availability of other 
libraries for Zynq can be found here: http://www.wiki.xilinx.com/Zynq+Linux.

# openPOWERLINK Stack Components {#sect_zynq_components}

The following section contains a description of the
[openPOWERLINK components](\ref page_components) available on a Zynq.

## Stack Libraries {#sect_zynq_components_libs}

The openPOWERLINK stack is divided into a user and a kernel part. The following
libraries are available for Zynq SoC:

* [Linux Stack Libraries](\ref sect_linux_components_libs)

## Demo Applications  {#sect_zynq_components_apps}

The following demo application are supported on Zynq:

* [demo_mn_console](\ref sect_components_demo_mn_console)
* [demo_cn_console](\ref sect_components_demo_cn_console)

## Drivers

The following drivers are available for Zynq SoC:

* [Linux Drivers] (\ref sect_linux_components_drivers)

# Building {#sect_zynq_build}

For building openPOWERLINK for Zynq refer to the
[generic build instructions](\ref page_build) and execute all required build
steps from this section. On the platform Xilinx Zynq the following build
steps can be carried out:

* [Build the openPOWERLINK stack libraries](\ref page_build_stack)
* [Build your application (or a delivered demo application)](\ref sect_build_demos_build_zynq)

# Running openPOWERLINK {#sect_zynq_running}

In order to run the openPOWERLINK demo applications on Zynq follow the steps listed below::

* Prepare the SD CARD for boot up with following:
  - uImage : Linux kernel image
  - uramdisk.image.gz : Initramfs file
  - devicetree.dtb : Device tree blob
  - boot.bin : Zynq boot image (generated using first stage bootloader(fsbl.elf), u-boot binary (u-boot-elf) and system.bit)
* Copy the stack binaries to the SD card.
* Connect the host PC to the Zynq board using the serial interface on board.
* Use a terminal program to connect to the Linux console on Zynq.
* For starting the application, refer to [running openPOWERLINK on Linux] (\ref sect_linux_running).
