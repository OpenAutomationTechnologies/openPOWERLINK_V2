openPOWERLINK on Xilinx Zynq SoC {#page_platform_zynq}
================================

[TOC]

# Introduction {#sect_zynq_intro}

This file contains documentation for the openPOWERLINK stack on a Xilinx
Zynq SoC. On Zynq, openPOWERLINK can be running under Linux as well as in
an embedded non-OS environment. This page will give an overview of the supported
environments and explains the steps to build and run openPOWERLINK on Zynq SoC.

Currently, openPOWERLINK can run under the following environments on a Zynq SoC:

- __Linux on Zynq ARM__
  openPOWERLINK runs on Linux which is running on the ARM processing system (PS)
  of the SoCprocessor under Linux.

- __Dual - Processor No-OS solution (ARM/FPGA)__
  The time-critical kernel part of the stack is running on a Microblaze softcore processor
  in the programming logic (PL) of the Zynq SoC. The application part of the stack is running on
  the ARM Cortex A9 Core 0 processing system (PS) of the SoC.

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
  downloaded from: http://www.xilinx.com/support/download/index.htm. The evaluation license for a period of 30 days
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

For general information about running openPOWERLINK on Linux refer to
["openPOWERLINK on Linux"](\ref page_platform_linux).

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

# Zynq SoC non-OS FPGA systems {#sect_zynq_noos}

openPOWERLINK can also run on a Zynq SoC in an embedded environment without operating
system. In this case the time-critical kernel part of the stack is running on a
Microblaze softcore processor in the programming logic (PL) of the Zynq SoC. The
application part of the stack is running on the ARM Cortex A9 Core 0 processing
system (PS) of the SoC. The communication between the two parts is implemented
using DDR3 shared memory along with interrupts. The following section contains
additional information about the non-OS implementation of openPOWERLINK.

## Contents {#sect_zynq_noos_contents}

This section lists the components which are included to support FPGA based
non-OS systems on the Zynq SoC:

- FPGA design with Microblaze CPU and openMAC IP-Core
- Dual processor shared memory library
- FAT16 SD card access library
- Zynq first stage bootloader[FSBL] CMake project

## openPOWERLINK Stack Components {#sect_zynq_noos_components}

The following section contains a description of the [openPOWERLINK components](\ref page_components)
available on a Zynq SoC non-OS system.

### Stack Libraries {#sect_zynq_noos_components_libs}

The openPOWERLINK stack is divided into a user and a kernel part. The application,
running on Zynq ARM is linked to an application library which uses a dual processor
shared memory library to communicate with the kernel driver. This driver is
linked with the driver library which uses openMAC to interface to the network.
It uses the dual processor shared memory library to communicate with the user
application library via the DDR3 shared memory.

The following libraries are available for the FPGA based system:

- `stack/proj/generic/liboplkmnapp-dualprocshm` (liboplkmnapp-dualprocshm.a)
- `stack/proj/generic/liboplkmndrv-dualprocshm` (liboplkmndrv-dualprocshm.a)

### Demo Applications  {#sect_zynq_noos_components_apps}

The following demos are supported for the FPGA based non-OS system on the Zynq SoC:

* [demo_mn_embedded] (\ref sect_components_demo_mn_embedded)

### Drivers  {#sect_zynq_noos_components_drivers}

The following drivers are supported for FPGA based non-OS system on Zynq SoC:

* __PCP Daemon on Microblaze__

The openPOWERLINK kernel part is compiled as a library which is linked to a
daemon. This daemon is running on a Microblaze softcore processor working
as the POWERLINK Communication Processor (PCP).
Shared memory is used as the communication interface between the PCP and the
host processor. The PCP is responsible for carrying out time critical processing
to achieve higher performance by reducing the jitter.

The driver is located in: `drivers/xilinx-microblaze/drv_daemon`

### Bootloader {#sect_zynq_noos_components_bootloader}

FPGA based non-OS systems on a Zynq SoC require a First Stage Bootloader (FSBL).
The FSBL configures the FPGA with a HW bit stream (if it exists) and loads the
Operating System (OS) Image or Standalone (SA) Image or 2nd Stage Boot Loader
image from the non-volatile memory (NAND/NOR/QSPI) to RAM (DDR) and starts
executing it. It supports multiple partitions, and each partition can be a code
image or a bit stream.

The FSBL for the openPOWERLINK demo on Zynq is compiled by importing the project
files from the Xilinx installation directory into the bootloader project
directory and setting up the necessary configuration to build the bootloader
using CMake configution files.
Follow the steps below to compile Zynq FSBL for demos:

* Open a shell where the Xilinx ISE 14.7 Toolchain is configured.
  - On a Windows host platform open the `ISE Design Suite [64,32] Bit Command
    Prompt`.
  - On a Linux host platform execute the script `<ISE_ROOT_DIR>/settings[32,64].sh>`
    to configure your current shell.

* Creating the executable

      > cd <openPOWERLINK_dir>/contrib/bootloader/xilinx-arm/fsbl/build
      > cmake -G"Unix Makefiles" -DCMAKE_TOOLCHAIN_FILE=../../../../../cmake/toolchain-xilinx-zynqarm-eabi-gnu.cmake .. -DCMAKE_BUILD_TYPE=[Debug,Release]
      > make all
      > make install

## Building {#sect_zynq_noos_build}

For building openPOWERLINK for FPGA based non-OS systema on Zynq SoC, refer to the
[generic build instructions](\ref page_build) and execute all required build
steps from this section. The following build steps can be carried out:

* [Build the hardware platform](\ref page_build_hardware)
* [Build the openPOWERLINK stack libraries](\ref page_build_stack)
* [Build the driver](\ref sect_build_drivers_build_daemon_microblaze)
* [Build your application (or a delivered demo application)](\ref page_build_demos)
* [Build the Zynq bootloader](\ref sect_zynq_noos_components_bootloader)

## Running openPOWERLINK {#sect_zynq_noos_running}

This section will explain the steps required to run openPOWERLINK demos on a
FPGA based non-OS system on a Zynq SoC.

### MN embedded demo

In order to debug and run the Zynq MN demo with Xilinx SDK refer to
[xapp1093](http://www.xilinx.com/support/documentation/application_notes/xapp1093-amp-bare-metal-microblaze.pdf)
(*Debugging the Design, page 22*).

The MN embedded demo can be started on the Zynq board using the SD card boot mode.
Follow the steps below to start the MN demo on the board:

* Open the `ISE Design Suite Command Prompt` and execute the following
  commands:\n

      > cd <openPOWERLINK_directory>\bin\generic\zynqarm\[BOARD_NAME]\[DEMO_NAME]
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
