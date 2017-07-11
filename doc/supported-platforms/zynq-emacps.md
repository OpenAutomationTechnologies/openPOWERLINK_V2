openPOWERLINK MN on Zynq Emacps design {#page_zynq_emacps}
==================

[TOC]

# Introduction {#sect_emacps_introduction}

This section serves as a quick start guide to setup the environment for compiling and executing the
openPOWERLINK Linux MN demo for the Zynq Emacps design using the Vivado 2016.2 toolchain.

# Requirements {#sect_zynq_emacps_requirements}

This section describes the hardware and software requirements for running the openPOWERLINK Linux MN
demo for the Zynq Emacps design.

## Hardware Requirements {#sect_emacps_hardware_requirements}

- Zynq ZC702 board (used as the openPOWERLINK MN)
- Linux PC
- Micro SD card reader
- Micro SD card
- Ethernet cables
- 1 Mini USB serial cable

## Software Requirements {#sect_emacps_software_requirements}

The following list of software packages and their dependencies are required to
run the openPOWERLINK Linux MN demo on the Zynq ZC702.

- Ubuntu 14.04 or later
- Vivado-2016.2
- CMake v2.8.7 or later
  * Install CMake and CMake GUI using the following commands:

        > sudo apt-get install cmake
        > sudo apt-get install cmake-gui

- Xilinx Linux (https://github.com/Xilinx/linux-xlnx)
  (Note: After cloning, use the following command to checkout the branch required for
  Vivado 2016.2 toolchain)

      > git checkout -b zynq-build xilinx-2016.2

- Install the libncurses5 library using the following command: (Note: This is needed for kernel menuconfig)

      > sudo apt-get install libncurses5-dev

- Install the u-boot-tools package to create uImage

      > sudo apt-get install u-boot-tools

- Download RT Preempt 4.4-rt2 version for Xilinx-linux from the following link:
  * https://www.kernel.org/pub/linux/kernel/projects/rt/4.4/older/patch-4.4-rt2.patch.gz
- Download the openPOWERLINK stack source (2.5.0 or higher) from the following link:
  * http://openpowerlink.sourceforge.net/web/openPOWERLINK/Download.html

# Steps to apply the RT Preempt patch to the Linux kernel source {#sect_zynq_emacps_preempt_patch}

This section describes the steps to be carried out on the Linux PC to apply the RT Preempt patch to
the Xilinx Linux kernel sources and compile the kernel.

- Open a terminal and move to the Xilinx Linux directory

      > cd <Xilinx_Linux_dir>

- Apply the patch using the following command:

      > patch -p1 < <(gunzip -c <path_to_patch-4.4-rt2.patch.gz>)

# Steps to compile the Linux kernel source for the Zynq ZC702 {#sect_emacps_kernel_compile}

This section describes the steps to be carried out on the Linux PC to compile the Linux kernel
source and create the kernel image file for Zynq ZC702.

- Export the cross compilation environment variables using the following command:

      > export CROSS_COMPILE=arm-linux-gnueabihf

- Configure the Linux kernel parameters using the default Zynq configuration.

      > make ARCH=arm xilinx_zynq_defconfig

- Reconfigure the default settings according to the requirements and use the following command
to open the kernel configuration window:
      > make ARCH=arm menuconfig

- Major settings which should be changed for executing the openPOWERLINK Linux MN demo with the Emacps design are as follows:
  * In **Device Drivers** -> **Network device Support** -> **Ethernet driver support**, enable Xilinx device driver support
 i.e. **Xilinx 10/100 Ethernet Lite support**, **Xilinx 10/100/1000 AXI Ethernet support** and **Xilinx Zynq tri-speed EMAC support**.
(Enable by pressing space key, where [M] denotes that enabled devices are included as modules)
  * Disable other devices except Xilinx devices.
  * Click Save -> ok -> Exit. Then click on Exit to come out of the Kernel configuration window.

- Compile the kernel using the following command:

      > make ARCH=arm CROSS_COMPILE=<Xilinx_dir>/SDK/2016.2/gnu/aarch32/lin/gcc-armlinux-gnueabi/bin/arm-linux-gnueabihf-

- To create uImage file:

      > make ARCH=arm UIMAGE_LOADADDR=0x8000 uImage  CROSS_COMPILE=<Xilinx_dir>/SDK/2016.2/gnu/aarch32/lin/gcc-arm-linux-gnueabi/bin/arm-linux-gnueabihf-

- Compile and install the modules for the Linux kernel using the following commands:
  * Compile modules:

            > make ARCH=arm CROSS_COMPILE=<Xilinx_dir>/SDK/2016.2/gnu/aarch32/lin/gcc-armlinux-gnueabi/bin/arm-linux-gnueabihf- modules

  * Install modules:

            > make ARCH=arm CROSS_COMPILE=<Xilinx_dir>/SDK/2016.2/gnu/aarch32/lin/gcc-armlinux-gnueabi/bin/arm-linux-gnueabihf- modules_install

# Steps for cross-compiling the openPOWERLINK Linux MN for the Zynq Emacps design {#sect_cross_compile_openpowerlink}

This section describes the steps to cross-compile the openPOWERLINK modules for the Zynq Emacps design.

They are:
1. Stack libraries
2. Driver libraries
3. Application libraries

- Set the Xilinx Vivado environment by executing the following command:

      > source <Xilinx_dir>/Vivado/2016.2/settings64.sh

- Open the CMake GUI using the following command:

      > cmake-gui

## Compile the stack libraries {#sect_emacps_stack_build}

- Point the **Where is the source code** to the stack source folder
  - \<openPOWERLINK_dir\>/stack
- Point the **Where to build the binaries** to the stack build folder
  - \<openPOWERLINK_dir\>/stack/build/linux
- Click the **Configure** button.
- In the **Specify the Generator for this project** dialog box, select **Unix Makefiles**
generator and select **Specify toolchain file for cross-compiling** and click **Next**.
- Provide the path for **Specify the Toolchain file** as below,
  - \<openPOWERLINK_dir\>/cmake/toolchain-xilinx-vivado-arm-linux-eabi-gnu.cmake
- Click **Finish** to proceed.
- Select the **CFG_COMPILE_LIB_MNAPP_KERNELINTF** to build the MN library and
set the CMAKE_BUILD_TYPE to **Debug**.
- Click **Configure** to apply the settings and click **Generate** to
create a Makefile with the modified configuration.
- Change the directory to the stack build path using the following command:

      > cd <openPOWERLINK_dir>/stack/build/linux

- Use the following command to compile the stack in **Debug** mode:

      > make install

- Set CMAKE_BUILD_TYPE to **Release**.
- Click **Configure** and **Generate**.
- Use the following command to compile the stack in **Release** mode:

      > make install

## Compile the driver libraries {#sect_emacps_driver_build}

- Point the **Where is the source code** to the driver source folder
  - \<openPOWERLINK_dir\>/drivers/linux/drv_kernelmod_edrv
- Point the **Where to build the binaries** to the driver build folder
  - \<openPOWERLINK_dir\>/drivers/linux/drv_kernelmod_edrv/build
- Click the **Configure** button.
- In **Specify the Generator for this project** dialog box, select **Unix Makefiles**
generator and select **Specify toolchain file for cross-compiling** and click **Next**.
- Provide the path for **Specify the Toolchain file** as below,
  - \<openPOWERLINK_dir\>/cmake/toolchain-xilinx-vivado-arm-linux-eabi-gnu.cmake
- Click **Finish** to proceed.
- Set CFG_KERNEL_DIR to the "<Xilinx_Linux_dir>" then select **CFG_POWERLINK_EDRV_EMACPS**
and **CFG_OPLK_MN**.
- Select **Configure** to apply the settings and click **Generate** to create the Makefile
with the modified configuration.

- Change the directory to the driver build path:

      > cd <openPOWERLINK_dir>/drivers/linux/drv_kernelmod_edrv/build

- Use the following command to compile the driver:

      > make install

## Compile the application libraries {#sect_emacps_application_build}

- Point the **Where is the source code** to the application source folder
  - \<openPOWERLINK_dir\>/apps/demo_mn_console
- Point the **Where to build the binaries** to the application build folder
  - \<openPOWERLINK_dir\>/apps/demo_mn_console/build/linux
- Click the **Configure** button.
- In the **Specify the Generator for this project** dialog box, select **Unix Makefiles**
generator and select **Specify toolchain file for cross-compiling** and click **Next**.
- Provide the path for **Specify the Toolchain file** as below,
  - \<openPOWERLINK_dir\>/cmake/toolchain-xilinx-vivado-arm-linux-eabi-gnu.cmake
- Click **Finish** to proceed.
- Set CFG_KERNEL_DIR to the "<Xilinx_Linux_dir>" then select **CFG_POWERLINK_EDRV_EMACPS** and **CFG_OPLK_MN**.
- Select **Configure** to apply the settings and click **Generate** to create the Makefile
with the modified configuration.

- Change the directory to the application build path:

      > cd <openPOWERLINK_dir>/apps/demo_mn_console/build/linux

- Use the following command to compile the application:

      > make install

# Steps to execute the openPOWERLINK Linux MN demo application with the Zynq Emacps design {#sect_run_emacps_mn}

This section describes the steps to run the openPOWERLINK Linux MN demo on
the Zynq ZC702 development board.

- Refer to the link given below in order to convert the SD card to a bootable medium for the Zynq.
  * http://www.wiki.xilinx.com/Prepare+Boot+Medium
- Refer to the link given below to download the Zynq ZC702 2016.2 pre built Linux
binaries and extract the downloaded package.
  * http://www.wiki.xilinx.com/Zynq+2016.2+Release

- Copy the below items to the boot partition of the SD card.
  - From the pre-built Linux binaries package
   * uramdisk.image.gz
   * devicetree.dtb
   * BOOT.bin
   * fsbl.elf
   * u-boot.elf

  - openPOWERLINK driver and application binaries from:
     - \<openPOWERLINK_dir\>/bin/linux/arm/oplkdrv_kernelmodule_zynq
     - \<openPOWERLINK_dir\>/bin/linux/arm/demo_mn_console

  - The uImage file from:
     - \<Xilinx_Linux_dir\>/arch/arm/boot

- To run openPOWERLINK
  * Connect the openPOWERLINK CNs to the Zynq MN through the Ethernet
port available on the Zynq ZC702 board.
  * Insert the SD card into the Zynq ZC702 board.
  * Connect the USB UART port of the Zynq ZC702 board with the Linux PC.
  * Execute the following command on the PC to open the minicom terminal.

        > sudo minicom -s

  * Go to the serial port setup.
  * Change the serial device to the USB serial port name. (Example: /dev/ttyUSB0)
  * Set the hardware flow control settings to **NO**.
  * Select **Save setup as dfl**.
  * After saving the configuration, select **Exit** to proceed.
  * Once the auto-boot finishes, enter **root** as username.
  * Mount the SD card using the following command:

        > mount /dev/mmcblk0p1 /mnt/

  * Change the directory

        > cd /mnt/oplkdrv_kernelmodule_edrv/

  * Insert the driver module using the following command:

        > insmod oplkemacpsmn.ko

  * Change the directory

        > cd /mnt/demo_mn_console

  * Run the openPOWERLINK MN demo using the following command:

        > ./demo_mn_console