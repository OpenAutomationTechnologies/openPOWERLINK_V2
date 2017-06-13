openPOWERLINK on Xilinx Zynq SoC {#page_platform_zynq}
================================

[TOC]

# Introduction {#sect_zynq_intro}

This file contains documentation for the openPOWERLINK stack on a Xilinx
Zynq SoC. On Zynq, openPOWERLINK can be running under Linux. This page will
give an overview of the supported environments and explains the steps to
build and run openPOWERLINK on Zynq SoC.

Currently, openPOWERLINK can run under the following environments on a Zynq SoC:

- \ref sect_zynq_hybrid
  The time-critical kernel part of the stack is running on a Microblaze softcore
  processor as a bare metal application in the programming logic (PL) of the Zynq SoC.
  The application part of the stack runs on Linux which is running on the ARM processing
  system (PS) of the SoC processor.

- \ref sect_linux_zynq_ARM
  openPOWERLINK runs on Linux which is running on the ARM processing system (PS)
  of the SoC processor.


# openPOWERLINK MN on Zynq Hybrid Design {#sect_zynq_hybrid}

This document serves as a quick start guide to setup the environment for compiling and executing the openPOWERLINK Linux MN demo for the Zynq Hybrid design using Vivado 2016.2 toolchain.

# Requirements {#sect_zynq_requirements}

This section describes the hardware and software requirements to execute the openPOWERLINK Linux MN for the Zynq Hybrid design.

## Hardware Requirements

- Zynq ZC702 board (used as openPOWERLINK MN)
- AVNET expander board (AES-FMC-ISMNET-G)
- Linux PC
- Micro SD card reader
- Micro SD card
- Ethernet cables
- 1 Mini USB serial cable

## Software Requirements

The following list of software packages and their dependencies are required to run the openPOWERLINK Linux MN demo on Zynq ZC702.

- Ubuntu 14.04 or later version
- Vivado-2016.2
- CMake v2.8.7 or later version
  * Install CMake and CMake GUI using the following commands:

        > sudo apt-get install cmake
        > sudo apt-get install cmake-gui

- Xilinx Linux (https://github.com/Xilinx/linux-xlnx)
  (Note: After cloning, use the following command to checkout the branch required for
  Vivado 2016.2 toolchain)

      > git checkout -b zynq-build xilinx-2016.2

- Install the libncurses5 library using the following command (Note: Used with menuconfig)

      > sudo apt-get install libncurses5-dev

- Install the u-boot-tools package to create uImage

      > sudo apt-get install u-boot-tools

- Download RT Preempt 4.4-rt2 version for the Xilinx-linux from the following link:
  * https://www.kernel.org/pub/linux/kernel/projects/rt/4.4/older/patch-4.4-rt2.patch.gz
- Download the openPOWERLINK stack from the following link:
  * https://sourceforge.net/projects/openpowerlink/
  * Change directory to the downloaded stack.
  * Checkout the 2.5.0 branch or later using the following command:

        > git checkout <branch_name>

# Steps to apply RT Preempt patch to the Linux kernel source {#sect_zynq_preempt_patch}

This section describes the steps to be carried out on the Linux PC to apply the RT Preempt patch to
the Xilinx Linux kernel sources and compile the kernel.

- Open terminal and move to the Xilinx Linux directory

      > cd <Xilinx_Linux_directory>

- Apply the patch using the following command:

      > patch -p1 < <(gunzip -c <path_to_patch-4.4-rt2.patch.gz>)

![](\ref zynq/apply_preempt_RT_patch.png)

## Steps to compile the Linux kernel source for Zynq ZC702 {#sect_preempt_patch}

This section describes the steps to be carried out on the Linux PC to compile the Linux kernel source and
create the kernel image file for Zynq ZC702.

- Export the cross compilation environment variables using the following command:

      > export CROSS_COMPILE=arm-linux-gnueabihf

- Configure the Linux kernel parameters using the default Zynq configuration file:

      > make ARCH=arm xilinx_zynq_defconfig

![](\ref zynq/compile_Linux_kernel.png)

- Compile the kernel using the following command:

      > make ARCH=arm CROSS_COMPILE=<Xilinx_dir>/SDK/2016.2/gnu/aarch32/lin/gcc-armlinux-gnueabi/bin/arm-linux-gnueabihf-

- To create uImage file:

      > make ARCH=arm UIMAGE_LOADADDR=0x8000 uImage CROSS_COMPILE=<Xilinx_dir>/SDK/2016.2/gnu/aarch32/lin/gcc-armlinux-gnueabi/bin/arm-linux-gnueabihf-

- Compile and install the modules for the Linux kernel using following commands:
  * Compile module:

            > make ARCH=arm CROSS_COMPILE=<Xilinx_dir>/SDK/2016.2/gnu/aarch32/lin/gcc-armlinux-gnueabi/bin/arm-linux-gnueabihf- modules

  * Install modules:

            > make ARCH=arm CROSS_COMPILE=<Xilinx_dir>/SDK/2016.2/gnu/aarch32/lin/gcc-armlinux-gnueabi/bin/arm-linux-gnueabihf- modules_install

![](\ref zynq/compile_install.png)

# Steps to build the Hardware for the Zynq Hybrid design {#sect_zynq_build_hardware}

- Change the path to Xilinx Vivado directory:

      > cd <Xilinx_dir>/Vivado/2016.2/bin

- Open Vivado TCL console 2016.2:

      > ./vivado -mode tcl

![](\ref zynq/vivado_tcl_console.png)

- Execute the following commands:

      > xsct
      > vivado -mode tcl
      > vivado -mode batch

![](\ref zynq/SDK_environment_path.png)

- Change directory:

      > cd <openPOWERLINK_dir>/hardware/build/xilinx-microblaze

- Execute the command:

      > cmake -GUnix\ Makefiles -DCMAKE_TOOLCHAIN_FILE=../../../cmake/toolchain-xilinx-microblaze-gnu.cmake ../..

- Execute the command:

      > cmake ../.. -DCMAKE_BUILD_TYPE=Debug -DSKIP_BITSTREAM=OFF -DDEMO_Z702_MN_DUAL_SHMEM_GPIO=ON

![](\ref zynq/configure_cmake.png)

- Execute the command:

      > make install

![](\ref zynq/hardware_build.png)

- Repeat the steps above to build in **Release** mode by setting CMAKE_BUILD_TYPE as Release.

![](\ref zynq/hardware_build_release.png)

- Execute the command:

      > make install

![](\ref zynq/hardware_build_release_install.png)

# Steps to build the PCP {#sect_zynq_build_pcp}

This section describes the steps to be carried out on the Linux PC to compile and build the PCP for the Zynq Hybrid design.

## Steps to build the driver library for Microblaze {#sect_build_driver_lib_for_MB}
- Change directory:

      > cd <openPOWERLINK_dir>/stack/build/xilinx-microblaze

- Execute the command:

      > cmake -GUnix\ Makefiles -DCMAKE_TOOLCHAIN_FILE=../../../cmake/toolchain-xilinx-microblaze-gnu.cmake ../.. -DCMAKE_BUILD_TYPE=Debug -DCFG_COMPILE_LIB_MNDRV_DUALPROCSHM=ON

![](\ref zynq/configure_mb_debug.png)

- Execute the command:

      > make install

![](\ref zynq/configure_mb_debug_install.png)

- Repeat the steps above to build in **Release** mode by setting CMAKE_BUILD_TYPE as Release.

![](\ref zynq/configure_mb_release.png)

- Execute the command:

      > make install

![](\ref zynq/configure_mb_release_install.png)

## Steps to the build driver application for Microblaze {#sect_build_driver_app_for_MB}

- Change directory:

      > cd <openPOWERLINK_dir>/drivers/xilinx-microblaze/drv_daemon/build

- Execute the command:

      > cmake -GUnix\ Makefiles -DCMAKE_TOOLCHAIN_FILE=../../../cmake/toolchain-xilinx-microblaze-gnu.cmake ../.. -DCMAKE_BUILD_TYPE=Release -DCFG_BUILD_KERNEL_STACK=PCP\ Daemon\ Dual-Proc -DCFG_HW_LIB=xilinx-z702/mn-dual-shmem-gpio ..

![](\ref zynq/config_driver_app.png)

- Execute the command:

      > make install

![](\ref zynq/build_driver_app.png)

- Exit from Vivado TCL console.

# Generate FSBL {#sect_zynq_generate_FSBL}

- Change directory:

      > cd <Xilinx_dir>/SDK/2016.2/bin

- Execute the command:

      > sudo ./xsdk

![](\ref zynq/create_workspace_in_sdk.png)

- Select an existing workspace or create a new workspace.
- Click File->New->Application project.
- Create a new application project and enter a project name.
- Click on **New** under target Hardware.

![](\ref zynq/new_application_project.png)

- Browse the hardware file path <openPOWERLINK_dir>/hardware/lib/generic/microblaze/xilinx-z702/mn-dual-shmem-gpio/hw_platform/system.hdf.

- Click **Finish** to proceed.

![](\ref zynq/select_hardware_platform.png)

- Ensure that the OS platform is **standalone** and the processor is **ps7_cortexa9** in the application project window.
- Clik **Next** to proceed.

![](\ref zynq/select_processor_target_hw_platform.png)

- Select **Zynq FSBL** and click **Finish**.

![](\ref zynq/generate_Zynq_FSBL.png)

- **fsbl.elf** is generated in the debug folder of the Xilinx SDK workspace.
- Exit from SDK workspace.

# Generate BOOT.bin {#sect_zynq_generate_boot_bin}

- Open terminal and change directory:

      > cd <openPOWERLINK_dir>/tools/xilinx-zynqvivado

- Copy all the required binaries to "<openPOWERLINK_dir>/tools/xilinx-zynqvivado"
- Files required for creating boot.bin:
  * fsbl.elf  (from <Xilinx_SDK_workspace>/<project_name>/Debug/)
  * download.bit (from <openPOWERLINK_dir>/bin/generic/microblaze/xilinx-z702/mn-dual-shmem-gpio)
  * u-boot.elf (from Zynq ZC702 package http://www.wiki.xilinx.com/Zynq+2016.2+Release)
  * oplkdrv-daemon_o.elf (from <openPOWERLINK_dir>/bin/generic/microblaze/xilinx-z702/mn-dual-shmem-gpio)
- Execute the command:

      > <Xilinx_dir>/SDK/2016.2/bin/bootgen -image bootimage.bif -o i boot.bin

![](\ref zynq/generate_boot_dot_bin.png)

# Generate device tree blob {#sect_generate_device_tree_blob}

- In terminal, change directory to the device tree source path using the following command.

      > cd <openPOWERLINK_dir>/hardware/boards/xilinx-z702/mn-dual-shmem-gpio/sdk/handoff/

- DTC is part of the Linux source directory. linux-xlnx/scripts/dtc/ contains the source code for DTC and needs to be compiled in order to be used.
- Build the DTS using the following command

      > <Xilinx_Linux_dir>/scripts/dtc/dtc -I dts -O dtb -o devicetree.dtb system.dts

![](\ref zynq/generate_device_tree_blob.png)

# Steps for cross-compiling the openPOWERLINK Linux MN for the Zynq Hybrid design {#sect_zynq_cross_compile}

This section describes the set of steps to cross compile the openPOWERLINK Linux MN Zynq ZC702 for the Zynq Hybrid design.

- Set the Xilinx Vivado environment by executing the following command:

      > source <Xilinx_dir>/Vivado/2016.2/settings64.sh

- Open CMake GUI using the following command:

      > cmake-gui

![](\ref zynq/set_Xilinx_Vivado_environment.png)

## To compile the stack libraries:
- Point the **Where is the source code** to the stack source folder
  - <openPOWERLINK_dir>/stack
- Point the **Where to build the binaries** to the stack build folder
  - <openPOWERLINK_dir>/stack/build/linux
- Click the **Configure** button
- In **Specify the Generator for this project** dialog box, select **Unix Makefiles** generator and select **Specify toolchain file for cross-compiling** and click **Next**.

![](\ref zynq/specify_generator_for_the_project.png)

- Provide the path for **Specify the Toolchain file** as below, <openPOWERLINK_dir>/cmake/toolchain-xilinx-vivado-arm-linux-eabi-gnu.cmake.

![](\ref zynq/toolchain_file_for_cross_compilation.png)

- Select the CFG_COMPILE_LIB_MNAPP_ZYNQINTF to build MN library.

![](\ref zynq/specify_compiler_lib_Zynq_MN.png)

- Click **Configure** to apply the settings and click **Generate** to create makefile with the modified configuration.
- Change directory to the stack build path.

      > cd <openPOWERLINK_dir>/stack/build/linux

- Use following command to compile the stack

      > make install

![](\ref zynq/build_stack_lib_for_Zynq_ARM.png)

- Repeat the steps above to build in **Release** mode by setting CMAKE_BUILD_TYPE as Release.

![](\ref zynq/config_stack_lib_release_mode.png)

- Execute the command:

      > make install

![](\ref zynq/build_stack_lib_for_Zynq_ARM_release.png)

## To compile the driver libraries:
- Provide the **Where is the source code** to the driver source folder
  - <openPOWERLINK_dir>/drivers/linux/drv_kernelmod_zynq>
- Provide the **Where to build the binaries** to the driver build folder
  - <openPOWERLINK_dir>/drivers/linux/drv_kernelmod_zynq/build>
- In **Specify the Generator for this project** dialog box, select **Unix Makefiles** generator and **Specify toolchain file for cross-compiling** as shown in the above *compile stack library* section.
- Provide the path for **Specify the Toolchain file** as below
  - <openPOWERLINK_dir>/cmake/toolchain-xilinx-vivado-arm-linux-eabi-gnu.cmake

![](\ref zynq/toolchain_file_for_driver_build.png)

- Set CFG_KERNEL_DIR to the "<Xilinx_Linux_dir>".

![](\ref zynq/kernerl_directory_for_driver_build.png)

- Select **Configure** to apply the settings and click **Generate** to create makefile with the modified configuration.
- Change directory to application build path.

      > cd <openPOWERLINK_dir>/drivers/linux/derv_kernelmod_zynq/build

- Use following command to compile the stack

      > make install

![](\ref zynq/build_driver_zynq_arm.png)

## To compile the application libraries:
- Provide  the **Where to build the binaries** to the application build folder
  - <openPOWERLINK_dir>/apps/demo_mn_console/
- Provide the **Where to build the binaries** to the application build folder
  - <openPOWERLINK_dir>/apps/demo_mn_console/ build/linux
- In **Specify the Generator for this project** dialog box, select **Unix Makefiles** generator and **Specify toolchain file for cross-compiling** as shown in the *compile stack library* section.

![](\ref zynq/specify_toolchain_for_demo_application.png)

- Provide the path for **Specify the Toolchain file** as below
  - <openPOWERLINK_dir>/cmake/toolchain-xilinx-vivado-arm-linux-eabi-gnu.cmake
- Set CFG_BUILD_KERNEL_STACK to **Kernel stack on Zynq PCP**.

![](\ref zynq/config_kernel_stack_as_Zynq_PCP.png)

- Select **Configure** to apply the settings and click **Generate** to create makefile with changed configuration.
- Change directory to the application build path.

      > cd <openPOWERLINK_dir>/apps/demo_mn_console/build/linux

- Use following command to compile the application.

      > make install

![](\ref zynq/build_demo_app.png)

# Steps to execute the openPOWERLINK Linux MN demo application on Zynq ZC702 {#sect_zynq_execute_demo}

This section describes the steps to run the openPOWERLINK Linux MN demo on Zynq ZC702 development board.

- Refer the below link to convert the SD card as bootable medium for Zynq
  * http://www.wiki.xilinx.com/Prepare+Boot+Medium
- Refer the below link to download the Zynq ZC702 2016.2 pre built Linux binaries (Assuming cross compilation for the Linux is done using Xilinx Vivado 2016.2 toolchain).
  * http://www.wiki.xilinx.com/Zynq+2016.2+Release
- Extract and copy the following content from the downloaded folder to the boot partition of SD card.
  * uramdisk.image.gz
  * devicetree.dtb
  * BOOT.bin
  * openPOWERLINK driver and application binaries from
    - <openPOWERLINK_dir>/bin/oplkdrv_kernelmodule_zynq
    - <openPOWERLINK_dir>/bin/demo_mn_console
  * Replace the existing uImage with the cross compiled uImage from
    - <Xilinx_Linux_dir>/arch/arm/boot
- Hardware setup
  * Connect the Avnet expander board to J3fmc1 connector of the Zynq ZC702 board.
  * Now connect the Ethernet cable to any of the Ethernet ports J6/J2 of the Avnet extension board and to the slave of the network.
- To run the openPOWERLINK
  * Insert the SD card into the Zynq ZC702 board.
  * Connect USB UART port in the Zynq ZC702 board with the Linux PC.
  * From the terminal, run minicom

        > sudo minicom -s

![](\ref zynq/open_minicom_terminal.png)

  * Go to serial port setup

![](\ref zynq/select_serial_port.png)

  * Change the serial device as per the USB name (/dev/ttyUSB0)
  * Set the hardware flow control settings to **NO**

![](\ref zynq/config_serial_port.png)

  * Save setup as dfl and exit.

![](\ref zynq/save_serial_port_config.png)

 * Once the autoboot finishes, enter **root** as username.
 * Mount the SD card using the following command:

        > mount /dev/mmcblk0p1 /mnt/

 * Change directory

        > cd /mnt/oplkdrv_kernelmodule_zynq/

 * Insert the driver module using the following command:

        > insmod oplkmnzynqintf.ko

 * Change the directory

        > cd /mnt/demo_mn_console

 * Run the openOWERLINK MN demo using the following command:

        > ./demo_mn_console

# Linux on Zynq ARM {#sect_linux_zynq_ARM}

On the Zynq SoC, openPOWERLINK runs on the ARM Cortex A9 processing system (Linux OS).
The following section contains additional information about the openPOWERLINK Linux implementation on the Zynq SoC.

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
