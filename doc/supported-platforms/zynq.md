openPOWERLINK on Xilinx Zynq SoC {#page_platform_zynq}
==================================

[TOC]

# Introduction {#sect_zynq_intro}

This file contains documentation for the openPOWERLINK stack on Xilinx
Zynq SoC. Presently following designs have been integrated with the
stack for Zynq SoC:

- __Linux user-kernel Managing Node/Controlled Node demos:__
  Uses kernel part (time critical) part of stack in Linux kernel demo
  with API library for application user space.

- __Dual - Processor Managing Node Demo using shared memory:__
  Kernel part of the stack running on Microblaze softcore processor
  in PL part of Zynq and user application on ARM cortex A9 core 0
  of PS. The two processor interact through each other using DDR3 shared
  memory along with interrupts.

## Contents {#sect_zynq_intro_contents}

- Edrv module for Gigabit Ethernet controller on Zynq 7000 series SoCs.
- HighRes module for Triple timer counter on Zynq 7000 series SoCs.
- FPGA design with Microblaze CPU and openMAC IP-Core.
- Dual processor library.
- FAT16 SD card access library.

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

Minicom is an Open Source terminal software. It is available for Debian and Ubuntu users
via apt-get:

    > sudo apt-get install minicom

TeraTerm is a program for printing out text which is transmitted over a serial
interface. It is open source and can be downloaded from: http://ttssh2.sourceforge.jp/

### Linux Kernel

A Linux kernel version for the Zynq platform is required in order to run the Linux-based
demo of openPOWERLINK. The Linux ports for Zynq platform are available on github
and can be downloaded or cloned from: https://github.com/Xilinx/linux-xlnx.

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
* __Dual processor using shared memory__

  The application running on ARM cortex is linked to a application library
  which uses the dual processor library to access shared memory between
  host and PCP. The kernel part runs on a Microblaze processor using a daemon
  application linked to a separate stack library containing stack.

  _Libraries:_

  - `stack/proj/generic/liboplkmnapp-dualprocshm` (liboplkmnapp-dualprocshm.a)
  - `stack/proj/generic/liboplkmndrv-dualprocshm` (liboplkmndrv-dualprocshm.a)

## Demo Applications  {#sect_zynq_components_apps}

The following demo application are supported on Zynq:

* [demo_mn_console](\ref sect_components_demo_mn_console)
* [demo_cn_console](\ref sect_components_demo_cn_console)
* [Embedded MN demo] (\ref sect_components_demo_mn_embedded)

## Drivers

The following drivers are available for Zynq SoC:

* [Linux Drivers] (\ref sect_linux_components_drivers)
* __PCP Daemon on Microblaze__

The openPOWERLINK kernel part is compiled as a separate library linked to a
daemon application running on separate communication processor(PCP) Microblaze.
Shared memory is used as the communication interface between the PCP and the
host processor. The PCP is responsible to carry out time critical processing
to achieve higher performance by reducing the jitter.

The driver is located in: `drivers/xilinx-microblaze/drv_daemon`

## Bootloader {#sect_zynq_components_bootloader}

Zynq non-OS demos requires an First Stage Bootloader (FSBL).
The FSBL configures the FPGA with HW bit stream (if it exists) and loads the
Operating System (OS) Image or Standalone (SA) Image or 2nd Stage Boot Loader
image from the  non-volatile memory (NAND/NOR/QSPI) to RAM (DDR) and starts
executing it. It supports multiple partitions, and each partition can be a code
image or a bit stream.

The FSBL for openPOWERLINK demo on Zynq is compiled by importing the project files
from Xilinx installation directory into the dootloader repo and setting up necessary
configuration to build the bootloader using CMake configution files. Follow the
steps below to compile Zynq fsbl for demos:

* Open a shell where the Xilinx ISE 14.7 Toolchain is configured.
  - On a Windows host platform open the `ISE Design Suite [64,32] Bit Command
    Prompt`.
  - On a Linux host platform execute the script `<ISE_ROOT_DIR>/settings[32,64].sh>`
    to configure your current shell.

* Creating the executable

      > cd <openPOWERLINK_dir>/contrib/bootloader/xilinx-arm/zynq-fsbl/build
      > cmake -G"Unix Makefiles" -DCMAKE_TOOLCHAIN_FILE=../../../../../cmake/toolchain-xilinx-arm-gnu.cmake ../ -DCMAKE_BUILD_TYPE=[Debug,Release]
      > make all
      > make install

# Building {#sect_zynq_build}

For building openPOWERLINK for Zynq refer to the
[generic build instructions](\ref page_build) and execute all required build
steps from this section. On the platform Xilinx Zynq the following build
steps can be carried out:

* [Build the hardware platform](\ref page_build_hardware)
* [Build the openPOWERLINK stack libraries](\ref page_build_stack)
* [Build the PCP daemon driver] (\ref page_build_drivers)
* [Build your application (or a delivered demo application)](\ref page_build_demos)
* [Build Zynq bootloader] (\ref sect_zynq_components_bootloader)

# Running openPOWERLINK {#sect_zynq_running}

## Linux MN/CN console demo

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

## Dual Processor MN embedded demo

In order to debug and run the Zynq MN demo with Xilinx SDK refer to
[xapp1093](http://www.xilinx.com/support/documentation/application_notes/xapp1093-amp-bare-metal-microblaze.pdf)
(*Debugging the Design, page 22*).

The Zynq embedded demo can be started on the Zynq boards using the SD card boot mode.
Follow the steps below to start MN demo on board:

* Open the `ISE Design Suite Command Prompt` and execute the following
  commands:\n

      > cd <openPOWERLINK_directory>\bin\generic\arm_xilinx\[BOARD_NAME]\[DEMO_NAME]
      > make all

* Copy the generated BOOT.BIN file to the SD card.
* Copy the configuration file (mnobd.cdc) in case the demo is configured to use CDC_ON_SD.
* Use a terminal program to see the debug output (Only possible if the
  application is compiled in debug mode)
    - Baud rate: `115200`
    - Data Bits: `8`
    - Stop Bits: `1`
    - Parity: `none`
    - Flow control: `none`