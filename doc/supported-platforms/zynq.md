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
  of the SoC processor.

- __Zynq Hybrid Design__
  The time-critical kernel part of the stack is running on a Microblaze softcore
  processor as a bare metal application in the programming logic (PL) of the Zynq SoC.
  The application part of the stack runs on Linux which is running on the ARM processing
  system (PS) of the SoC processor.

# Requirements {#sect_zynq_requirements}

## Development Boards {#sect_zynq_requirements_boards}

- Xilinx ZC702 Development Kit
- Any other board with Zynq 7000 series SoC (porting required for *Zynq Hybrid Design*)

## POWERLINK network {#sect_zynq_requirements_network}

- POWERLINK network with a controlled nodes (CN)
  * openPOWERLINK controlled nodes, e.g. Linux
  * B&R POWERLINK controlled nodes

- POWERLINK network with a managing node (MN)
  * openPOWERLINK managing node, e.g. Linux
  * B&R POWERLINK managing node

## Tools {#sect_zynq_requirements_tools}

### Xilinx Vivado
The following tool is necessary to evaluate a Xilinx Zynq based openPOWERLINK
solution:

* `Xilinx Vivado - HLx Edition` which is called `Vivado Design Suite - 2016.2 Full
  Product Installation` or `Vivado Design Suite - 2016.2 HLx Edition` and can be
  downloaded from: http://www.xilinx.com/support/download/index.html

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
on Linux. The Linux port for the Zynq platform is available at Github
and can be downloaded or cloned from: https://github.com/Xilinx/linux-xlnx

Linux kernel version 4.4.0 tagged as xilinx-v2016.2 at Github was used for testing at
the time of writing this document.

The steps to cross-compile Linux for Zynq and information about availability of other
libraries for Zynq can be found here: http://www.wiki.xilinx.com/Zynq+Linux.

### RT Preempt patch

Download Preempt RT version 4.4.rt2 for the Xilinx-linux from
https://www.kernel.org/pub/linux/kernel/projects/rt/4.4/older/patch-4.4-rt2.patch.gz

Apply the patch using the following command:

    > patch -p1 << (gunzip -c /path/to/patch-4.4-rt2.patch.gz)

# Linux on Zynq ARM {#sect_zynq_linux}

On the Zynq SoC, openPOWERLINK runs on a Linux OS which is running on the
ARM Cortex A9 processing system (PS) of the SoC. The following section contains
additional information about the openPOWERLINK Linux implementation on the Zynq SoC.

For general information about running openPOWERLINK on Linux refer to \ref page_platform_linux.

__NOTE__: Third-party libraries such as libpcap and libqt must also be
          cross-compiled for Zynq ARM apart from openPOWERLINK.

## Contents {#sect_zynq_linux_contents}
The following modules have been added to the openPOWERLINK stack to support the Zynq SoC:

- Edrv module for the Gigabit Ethernet controller on Zynq 7000 series SoCs (emacps).

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

Please refer to [Linux Drivers](\ref sect_linux_components_drivers) for information
about the available drivers.

## Building {#sect_zynq_linux_build}

For cross-compiling openPOWERLINK for Linux on Zynq ARM refer to the
[generic build instructions](\ref page_build) and execute all the required build
steps for this section. The following build steps are valid for Linux on Zynq ARM:

* [Build the openPOWERLINK stack libraries](\ref sect_build_stack_build_linux)
* [Build Linux PCAP User Space Daemon](\ref sect_build_drivers_build_linux_pcap)
* [BUild Linux Edrv Kernel Driver](\ref sect_build_drivers_build_linux_edrv)
* [Build your application (or a delivered demo application)](\ref sect_build_demos_build_linux)

# Zynq Hybrid Design {#sect_zynq_hybrid}

openPOWERLINK can also run on a Zynq SoC with Linux operating system. In this case
the time-critical kernel part of the stack is running on a Microblaze softcore
processor in the programming logic (PL) of the Zynq SoC. The application part of the
stack is running on Linux on both ARM cores of the processing system (PS) of the SoC. The
communication between the ARM and Microblaze is using a Linux kernel driver
implementation that employs DDR3 shared memory along with interrupts. The following
section contains additional information about the Zynq hybrid design implementation
of openPOWERLINK.

## Contents {#sect_zynq_hybrid_contents}

This section lists the components which are included to support Zynq hybrid systems on the Zynq SoC:

* FPGA design with Microblaze CPU and openMAC IP-Core
* Dual processor shared memory library
* Zynq first stage bootloader (FSBL)

## openPOWERLINK Stack Componenets {#sect_zynq_hybrid_components}

The following section contains the description of the [openPOWERLINK components](\ref page_components)
available on a Zynq SoC Hybrid Design.

### Stack libraries {#sect_zynq_hybrid_components_libs}

The openPOWERLINK stack is divided into a user and a kernel part. The application,
running on Zynq ARM, is linked to an application library which uses a dual processor shared
memory library to communicate with the kernel driver. This driver is linked with the driver
library which uses openMAC to interface to the network. It uses the dual processor shared memory
library to communicate with the user application library via the DDR3 shared memory.

The following libraries are available for the FPGA based Zynq hybrid Design:

- `stack/proj/generic/liboplkmnapp-kernelpcp` (liboplkmnapp-kernelpcp.a)
- `stack/proj/generic/liboplkmndrv-dualprocshm` (liboplkmndrv-dualprocshm.a)

### Demo Applications {#sect_zynq_hybrid_components_apps}

The following demos are supported for the FPGA based Hybrid system on the Zynq SoC:

* [demo_mn_console](\ref sect_components_demo_mn_console)

### Drivers {#sect_zynq_hybrid_components_drivers}

The following drivers are supported for FPGA based Hybrid system on Zynq SoC:

* **PCP Daemon on Microblaze**

The openPOWERLINK kernel part is compiled as a library which is linked to a
daemon. This daemon is running on a Microblaze softcore processor working as the
POWERLINK Communication Processor (PCP). Shared memory is used as the communication
interface between the PCP and the host processor. The PCP is responsible for
carrying out time critical processing to achieve higher performance by reducing
the jitter.

The driver is located in: `drivers/xilinx-microblaze/drv_daemon`

## Building {#sect_zynq_hybrid_build}

For building openPOWERLINK for FPGA based Hybrid Design on Zynq SoC, refer to the generic build instructions
and execute all required build steps from this section.

__NOTE__: For building the Zynq Hybrid demo the pre-built Linux image for the ZC702 is required.
          Download it from: http://www.wiki.xilinx.com/Zynq+2016.2+Release

The following build steps can be carried out:

* [Build the hardware platform](\ref page_build_hardware)
* [Build the openPOWERLINK stack libraries](\ref page_build_stack)
* [Build the driver](\ref page_build_drivers)
* [Build your application](\ref page_build_demos)
* [Build boot loader](\ref sect_zynq_hybrid_build_fsbl)
* [Generate device tree](\ref sect_generate_device_tree_blob)
* [Generate boot.bin](\ref sect_zynq_hybrid_generate_boot_bin)
* [Generate uImage](\ref sect_generate_uimage)

### Build FSBL {#sect_zynq_hybrid_build_fsbl}

To generate the fsbl from SDK for the Vivado hardware the following steps need to be done:

* Create new application project and set the name to fsbl
* Ask for hardware path and get file from `<OPLK>/hardware/lib/generic/microblaze/xilinx-z702/mn-dual-shmem-gpio/hw_platform/system.hdf`
* Generate fsbl application and copy the fsbl.elf into bin for generating BOOT.bin

__NOTE__: Alternatively, the fsbl.elf file supplied with the pre-built Linux ZC702 image from the prior step can be used.

### Generate BOOT.bin {#sect_zynq_hybrid_generate_boot_bin}

In order to generate the BOOT.bin file the following files are required:

- fsbl.elf (First stage bootloader)
- u-boot.elf (For booting Linux Kernel on ARM, obtain from pre-built Linux image for the ZC702)
- download.bit (Bitstream to configure PL)
- oplkdrv-daemon_o.elf (ELF for Microblaze softcore processor in PL)

Run the following steps to generate the BOOT.bin. Note that the driver daemon for Microblaze
should be compiled and installed into `<OPLK>/bin/generic/microblaze/xilinx-z702/mn-dual-shmem-gpio`!

    > cd <OPLK>/bin/generic/microblaze/xilinx-z702/mn-dual-shmem-gpio
    > cp <OPLK>/tools/xilinx-zynqvivado/bootimage.bif .
    > cp /path/to/fsbl.elf .
    > cp /path/to/u-boot.elf .
    > bootgen -image bootimage.big -o BOOT.bin

### Generate device tree blob {#sect_generate_device_tree_blob}

The device tree blob is generated by using DTC, which is part of the Linux kernel sources. The required
handoff files are provided in the hardware design path: `<OPLK>/hardware/boards/xilinx-z702/mn-dual-shmem-gpio/sdk/handoff`

Run the following steps to generate the device tree blob:

    > cd <OPLK>/hardware/boards/xilinx-z702/mn-dual-shmem-gpio/sdk/handoff
    > <XILINX_LINUX_DIR>/scripts/dtc/dtc -I dts -O dtb -o devicetree.dtb system.dts


### Generate an uncompressed kernel image {#sect_generate_uimage}

To create the uImage file needed for booting the Zynq board execute the following commands:

    > cd <XILINX_LINUX_DIR>
    > make ARCH=arm UIMAGE_LOADADDR=0x8000 uImage CROSS_COMPILE=<XILINX_VIVADO_DIR>/SDK/2016.2/gnu/aarch32/lin/gcc-arm-linux-gnueabi/bin/arm-linuxgnueabihf-

## Running openPOWERLINK  {#sect_zynq_hybrid_running}

### MN console demo

The MN console demo can be started on the Zynq board using the SD card boot mode.
Follow the steps below to start the MN demo on the board:

- Copy the following contents to the SD card to run the demo:

   - uramdisk.image.gz (from pre-built Linux image for the ZC702)
   - devicetree.dtb (generated in \ref sect_generate_device_tree_blob)
   - BOOT.bin (generated in \ref sect_zynq_hybrid_generate_boot_bin)
   - uImage (from `<XILINX_LINUX_DIR>/arch/arm/boot`)
   - openPOWERLINK driver and application binaries from
     - `<OPLK>/bin/linux/arm/oplkdrv_kernelmodule_zynq`
     - `<OPLK>/bin/linux/arm/demo_mn_console`

- Insert the SD card in the Zynq702 board
- Connect USB UART port of the ZC702 board with Linux PC
- Start minicom from the shell (sudo minicom -s)
- Go to the serial port setup
- Change the serial device to the USB device (/dev/ttyUSB0)
- Set the hardware flow control settings to *NO*
- Save setup as dfl and exit
- Once autoboot finishes enter the user-name as *root*
- Mount the SD card using the following command:

    > mount /dev/mmcblk0p1 /mnt

- Change directory to /mnt/oplkdrv_kernelmodule_zynq

    > insmod oplkmnzynqintf.ko

- Change directory to /mnt/demo_mn_console

    > ./demo_mn_console
