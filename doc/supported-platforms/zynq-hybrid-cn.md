openPOWERLINK CN on Zynq Hybrid design {#page_cn_zynq_hybrid}
=================

[TOC]

# Introduction {#sect_hybrid_cn_intro}

This section serves as a quick-start guide to setup the environment for compiling and
executing the openPOWERLINK Linux CN demo for the Zynq Hybrid design using the Vivado
2016.2 toolchain.

# Requirements {#sect_zynq_hybrid_cn_requirements}

This section describes the hardware and software requirements for executing the
openPOWERLINK Linux CN Demo with the Zynq Hybrid design.

## Hardware Requirements {#sect_cn_hardware_requirements}

- Zynq ZC702 board (used as openPOWERLINK CN)
- AVNET expander board (AES-FMC-ISMNET-G)
- Linux PC
- Micro SD card reader
- Micro SD card
- Ethernet cables
- 1 Mini USB serial cable

## Software Requirements {#sect_cn_software_requirements}

The following list of software packages and their dependencies are required to
run the openPOWERLINK Linux CN demo on the Zynq ZC702.

- Ubuntu 14.04 or later version
- Vivado-2016.2
- CMake v2.8.7 or later version
  * Install CMake and CMake GUI using the following commands:

        > sudo apt-get install cmake
        > sudo apt-get install cmake-gui

- Xilinx Linux (https://github.com/Xilinx/linux-xlnx)
  (Note: After cloning, use the following command to checkout the branch required for
  the Vivado 2016.2 toolchain)

      > git checkout -b zynq-build xilinx-2016.2

- Install the libncurses5 library using the following command: (Note: This is needed for kernel menuconfig)

      > sudo apt-get install libncurses5-dev

- Install the u-boot-tools package to create uImage

      > sudo apt-get install u-boot-tools

- Download RT Preempt 4.4-rt2 version for the Xilinx-linux from the following link:
  * https://www.kernel.org/pub/linux/kernel/projects/rt/4.4/older/patch-4.4-rt2.patch.gz
- Download the openPOWERLINK stack (2.5.0 or higher) from the following link:
  * http://openpowerlink.sourceforge.net/web/openPOWERLINK/Download.html

# Steps to apply the RT Preempt patch to the Linux kernel source {#sect_zynq_cn_hybrid_preempt_patch}

This section describes the steps to be carried out on the Linux PC to apply the
RT Preempt patch to the Xilinx Linux kernel sources and compile the kernel.

- Open a terminal and move to the Xilinx Linux directory

      > cd <Xilinx_Linux_directory>

- Apply the patch using the following command:

      > patch -p1 < <(gunzip -c <path_to_patch-4.4-rt2.patch.gz>)

# Steps to compile the Linux kernel source for the Zynq ZC702 {#sect_hybrid_cn_kernel_compile}

This section describes the steps to be carried out on the Linux PC to compile
the Linux kernel source and create the kernel image file for the Zynq ZC702.

- Export the cross compilation environment variables using the following command:

      > export CROSS_COMPILE=arm-linux-gnueabihf

- Configure the Linux kernel parameters using the default Zynq configuration file:

      > make ARCH=arm xilinx_zynq_defconfig

- Compile the kernel using the following command:

      > make ARCH=arm CROSS_COMPILE=<Xilinx_dir>/SDK/2016.2/gnu/aarch32/lin/gcc-arm-linux-gnueabi/bin/arm-linux-gnueabihf-

- To create the uImage file:

      > make ARCH=arm UIMAGE_LOADADDR=0x8000 uImage CROSS_COMPILE=<Xilinx_dir>/SDK/2016.2/gnu/aarch32/lin/gcc-arm-linux-gnueabi/bin/arm-linux-gnueabihf-

- Compile and install the modules for the Linux kernel using the following commands:
  * Compile modules:

            > make ARCH=arm CROSS_COMPILE=<Xilinx_dir>/SDK/2016.2/gnu/aarch32/lin/gcc-arm-linux-gnueabi/bin/arm-linux-gnueabihf- modules

  * Install modules:

            > make ARCH=arm CROSS_COMPILE=<Xilinx_dir>/SDK/2016.2/gnu/aarch32/lin/gcc-arm-linux-gnueabi/bin/arm-linux-gnueabihf- modules_install

# Steps to build the hardware for the Zynq Hybrid design {#sect_zynq_build_cn_hardware}

- Change the path to the Xilinx Vivado directory:

      > cd <Xilinx_dir>/Vivado/2016.2/bin

- Open Vivado TCL console 2016.2:

      > ./vivado -mode tcl

- Execute the following commands:

      > xsct
      > vivado -mode tcl
      > vivado -mode batch

- Change directory to the Xilinx Microblaze hardware project:

      > cd <openPOWERLINK_dir>/hardware/build/xilinx-microblaze

- Execute the following commands to build the hardware in **Debug** mode:

      > cmake -GUnix\ Makefiles -DCMAKE_TOOLCHAIN_FILE=../../../cmake/toolchain-xilinx-microblaze-gnu.cmake ../..
      > cmake ../.. -DCMAKE_BUILD_TYPE=Debug -DSKIP_BITSTREAM=OFF -DDEMO_Z702_CN_DUAL_SHMEM_GPIO=ON
      > make install

- Execute the following commands to build the hardware in **Release** mode:

      > cmake ../.. -DCMAKE_BUILD_TYPE=Release -DSKIP_BITSTREAM=OFF -DDEMO_Z702_CN_DUAL_SHMEM_GPIO=ON
      > make install

# Steps to build the PCP {#sect_zynq_build_cn_pcp}

This section describes the steps to be carried out on the Linux PC to compile
and build the PCP for the Zynq Hybrid design.

## Steps to build the driver library for Microblaze {#sect_build_driver_cn_lib_for_MB}
- Change directory:

      > cd <openPOWERLINK_dir>/stack/build/xilinx-microblaze

- Execute the following commands to build the PCP in **Debug** mode:

      > cmake -GUnix\ Makefiles -DCMAKE_TOOLCHAIN_FILE=../../../cmake/toolchain-xilinx-microblaze-gnu.cmake ../.. -DCMAKE_BUILD_TYPE=Debug -DCFG_COMPILE_LIB_CNDRV_DUALPROCSHM=ON
      > make install

- Execute the following commands to build the PCP in **Release** mode:

      > cmake -GUnix\ Makefiles -DCMAKE_TOOLCHAIN_FILE=../../../cmake/toolchain-xilinx-microblaze-gnu.cmake ../.. -DCMAKE_BUILD_TYPE=Release -DCFG_COMPILE_LIB_CNDRV_DUALPROCSHM=ON
      > make install

## Steps to build the driver application for Microblaze {#sect_build_driver_cn_app_for_MB}

- Change directory:

      > cd <openPOWERLINK_dir>/drivers/xilinx-microblaze/drv_daemon/build

- Execute the following commands to build the driver application for Microblaze:

      > cmake -GUnix\ Makefiles -DCMAKE_TOOLCHAIN_FILE=../../../cmake/toolchain-xilinx-microblaze-gnu.cmake ../.. -DCMAKE_BUILD_TYPE=Release -DCFG_BUILD_KERNEL_STACK=PCP\ Daemon\ Dual-Proc -DCFG_HW_LIB=xilinx-z702/cn-dual-shmem-gpio -DCFG_OPLK_MN=OFF ..
      > make install

- Exit from the Vivado TCL console.

# Generate FSBL {#sect_zynq_generate_cn_FSBL}

- Change to the SDK binary directory:

      > cd <Xilinx_dir>/SDK/2016.2/bin

- Execute the command:

      > sudo ./xsdk

- Select an existing workspace or create a new workspace.
- Click File->New->Application project.
- Create a new application project and enter a project name.
- Click on **New** under target Hardware.
- Browse the hardware file path "<openPOWERLINK_dir>/hardware/lib/generic/microblaze/xilinx-z702/cn-dual-shmem-gpio/hw_platform/system.hdf"
- Click **Finish** to proceed.
- Ensure that the OS platform is **standalone** and the processor is **ps7_cortexa9**
in the application project window.
- Click **Next** to proceed.
- Select **Zynq FSBL** and click **Finish**.
- **fsbl.elf** is generated in the debug folder of the Xilinx SDK workspace.
- Exit from SDK workspace.

# Generate BOOT.bin {#sect_zynq_generate_cn_boot_bin}

- Open terminal and change directory:

      > cd <openPOWERLINK_dir>/tools/xilinx-zynqvivado

- Copy all the required binaries to "<openPOWERLINK_dir>/tools/xilinx-zynqvivado"
- Files required for creating boot.bin:
  * **fsbl.elf** (from \<Xilinx_SDK_workspace\>/\<project_name\>/Debug/)
  * **download.bit** (from \<openPOWERLINK_dir\>/bin/generic/microblaze/xilinx-z702/cn-dual-shmem-gpio)
  * **u-boot.elf** (from Zynq ZC702 package http://www.wiki.xilinx.com/Zynq+2016.2+Release)
  * **oplkdrv-daemon_o.elf** (from \<openPOWERLINK_dir\>/bin/generic/microblaze/xilinx-z702/cn-dual-shmem-gpio)
- Execute the command:

      > <Xilinx_dir>/SDK/2016.2/bin/bootgen -image bootimage.bif -o i boot.bin

# Generate device tree blob {#sect_generate_cn_device_tree_blob}

- In the terminal, change the directory to the device tree source path using the following command:

      > cd <openPOWERLINK_dir>/hardware/boards/xilinx-z702/cn-dual-shmem-gpio/sdk/handoff/

- DTC is part of the Linux source directory. \<Xilinx_Linux_dir\>/scripts/dtc/ contains
the source code for DTC and needs to be compiled in order to be used.
- Build the DTS using the following command:

      > <Xilinx_Linux_dir>/scripts/dtc/dtc -I dts -O dtb -o devicetree.dtb system.dts

# Steps for cross-compiling the openPOWERLINK Linux CN for the Zynq Hybrid design {#sect_zynq_cn_cross_compile}

This section describes the set of steps to cross compile the openPOWERLINK
Linux CN Zynq ZC702 for the Zynq Hybrid design.

- Set the Xilinx Vivado environment by executing the following command:

      > source <Xilinx_dir>/Vivado/2016.2/settings64.sh

- Open CMake GUI using the following command:

      > cmake-gui

## Compile the stack libraries {#sect_compile_cn_stack}
- Point the **Where is the source code** to the stack source folder
  - \<openPOWERLINK_dir\>/stack
- Point the **Where to build the binaries** to the stack build folder
  - \<openPOWERLINK_dir\>/stack/build/linux
- Click the **Configure** button.
- In the **Specify the Generator for this project** dialog box, select **Unix Makefiles**
generator and select **Specify toolchain file for cross-compiling** and click **Next**.
- Provide the path for **Specify the Toolchain file** as below,
\<openPOWERLINK_dir\>/cmake/toolchain-xilinx-vivado-arm-linux-eabi-gnu.cmake.
- Select the CFG_COMPILE_LIB_CNAPP_ZYNQINTF to build the CN library.
- Click **Configure** to apply the settings and click **Generate** to create
the Makefile with the modified configuration.
- Change the directory to the stack build path.

      > cd <openPOWERLINK_dir>/stack/build/linux

- Use the following command to compile the stack in **Debug** mode:

      > make install

- Repeat the steps above to build in **Release** mode by setting CMAKE_BUILD_TYPE as Release.
- Use the following command to compile the stack in **Release** mode:

      > make install

## Compile the driver libraries {#sect_compile_cn_driver}
- Provide the **Where is the source code** to the driver source folder
  - \<openPOWERLINK_dir\>/drivers/linux/drv_kernelmod_zynq>
- Provide the **Where to build the binaries** to the driver build folder
  - \<openPOWERLINK_dir\>/drivers/linux/drv_kernelmod_zynq/build>
- Click the **Configure** button.
- In the **Specify the Generator for this project** dialog box, select **Unix Makefiles**
generator and **Specify toolchain file for cross-compiling** and click **Next**.
- Provide the path for **Specify the Toolchain file** as below,
  - \<openPOWERLINK_dir\>/cmake/toolchain-xilinx-vivado-arm-linux-eabi-gnu.cmake
- Click **Finish** to proceed.
- Set CFG_KERNEL_DIR to "<Xilinx_Linux_dir>".
- Unselect CFG_OPLK_MN.
- Select **Configure** to apply the settings and click **Generate** to create
the Makefile with the modified configuration.
- Change the directory to the driver build path.

      > cd <openPOWERLINK_dir>/drivers/linux/drv_kernelmod_zynq/build

- Use the following command to compile the driver:

      > make install

## Compile the application libraries {#sect_compile_cn_application}
- Provide the **Where to build the binaries** to the application build folder
  - \<openPOWERLINK_dir\>/apps/demo_cn_console/
- Provide the **Where to build the binaries** to the application build folder
  - \<openPOWERLINK_dir\>/apps/demo_cn_console/build/linux
- Click the **Configure** button.
- In the **Specify the Generator for this project** dialog box, select **Unix Makefiles**
generator and **Specify toolchain file for cross-compiling** and click **Next**.
- Provide the path for **Specify the Toolchain file** as below
  - \<openPOWERLINK_dir\>/cmake/toolchain-xilinx-vivado-arm-linux-eabi-gnu.cmake
- Set CFG_BUILD_KERNEL_STACK to **Kernel stack on Zynq PCP**.
- Select **Configure** to apply the settings and click **Generate** to create the Makefile
with the modified configuration.
- Change the directory to the application build path.

      > cd <openPOWERLINK_dir>/apps/demo_cn_console/build/linux

- Use the following command to compile the application:

      > make install

# Steps to execute the openPOWERLINK Linux CN demo application for the Zynq Hybrid design {#sect_zynq_execute_cn_demo}

This section describes the steps to run the openPOWERLINK Linux CN demo on the Zynq ZC702 development board.

- Refer to the link given below to convert the SD card to a bootable medium for the Zynq:
  * http://www.wiki.xilinx.com/Prepare+Boot+Medium
- Refer to the link given below to download the Zynq ZC702 2016.2 pre-built Linux binaries:
  * http://www.wiki.xilinx.com/Zynq+2016.2+Release
- Extract the Zynq ZC702 2016.2 pre-built Linux binaries package.
- Copy the following content to the boot partition of the SD card:
  * uramdisk.image.gz from:
    - Zynq ZC702 2016.2 pre-built Linux binaries package
  * devicetree.dtb from:
    - \<openPOWERLINK_dir>/hardware/boards/xilinx-z702/cn-dual-shmem-gpio/sdk/handoff
  * BOOT.bin from:
    - \<openPOWERLINK_dir>/tools/xilinx-zynqvivado
  * openPOWERLINK driver and application folders from:
    - \<openPOWERLINK_dir\>/bin/linux/arm/oplkdrv_kernelmodule_zynq
    - \<openPOWERLINK_dir\>/bin/linux/arm/demo_cn_console
  * The uImage from:
    - \<Xilinx_Linux_dir\>/arch/arm/boot
- Hardware setup
  * Connect the Avnet expander board to the J3 FMC1 connector of the Zynq ZC702 board.
  * Now connect the Ethernet cable to any of the Ethernet ports J6/J2 of the Avnet
extension board and to a MN in the network.
- To run openPOWERLINK
  * Insert the SD card into the Zynq ZC702 board.
  * Connect the USB UART port in the Zynq ZC702 board with the Linux PC.
  * From the terminal, run minicom using the following command:

        > sudo minicom -s

  * Go to the serial port setup.
  * Change the serial device to the USB serial device name. (Example: /dev/ttyUSB0)
  * Set the hardware flow control settings to **NO**.
  * Select **Save setup as dfl**.
  * After saving the configuration, select **Exit** to proceed.
  * Once the auto-boot finishes, enter **root** as username.
  * Mount the SD card using the following command:

        > mount /dev/mmcblk0p1 /mnt/

  * Change the directory

        > cd /mnt/oplkdrv_kernelmodule_zynq/

  * Insert the driver module using the following command:

        > insmod oplkcnzynqintf.ko

  * Change the directory

        > cd /mnt/demo_cn_console

  * Run the openPOWERLINK CN demo using the following command:

        > ./demo_cn_console
