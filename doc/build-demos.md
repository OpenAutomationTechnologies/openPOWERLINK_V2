Building openPOWERLINK Demo Applications {#page_build_demos}
========================================

[TOC]

# Building a Demo Application {#sect_build_demos_build}

The following section describes how the delivered demo applications can be
built. The demo applications are located in the directory `apps`. The default
binary installation path is: `<openPOWERLINK_DIR>/bin/<platform>/<ARCH>`

__NOTE:__ In order to be able to build an application, the Debug or the
Release library versions must be available, depending on your CMAKE_BUILD_TYPE.
If the needed library is missing you get an OPLKLIB-NOTFOUND or
OPLKLIB_DEBUG-NOTFOUND error in the demos CMake configuration.
If you use the Visual Studio generator on Windows, both libraries must be
available as you can swith between Debug and Release inside the Visual Studio
solution.


## Building on Linux {#sect_build_demos_build_linux}

      > cd <openPOWERLINK_dir>/apps/<demo_dir>/build/linux
      > cmake ../..
      > make
      > make install

## Building on Windows {#sect_build_demos_build_windows}

Open a Visual Studio command line and enter the following commands:

      > cd <openPOWERLINK_dir>\apps\<demo_dir>\build\windows
      > cmake -G"NMake Makefiles" ..\..
      > nmake
      > nmake install

__NOTE:__ You can also generate a Visual Studio Solution and compile the
libraries in Visual Studio. Please refer to the CMAKE documentation for
generating Visual Studio solution files.

## Building for embedded systems (Non-OS targets) {#sect_build_demos_noos}

### Building for target Microblaze {#sect_build_demos_build_microblaze}

Follow the steps below to cross compile your demo application for Microblaze:
* Open a shell where the Xilinx ISE 14.7 Toolchain is configured.
  - On a Windows host platform open the `ISE Design Suite [64,32] Bit Command
    Prompt`.
  - On a Linux host platform execute the script `<ISE_ROOT_DIR>/settings[32,64].sh>`
    to configure your current shell.

* Create the executable

      > cd <openPOWERLINK_dir>/apps/<demo_dir>/build/xilinx-microblaze
      > cmake -G"Unix Makefiles" -DCMAKE_TOOLCHAIN_FILE=../../../../cmake/toolchain-xilinx-microblaze-gnu.cmake ../.. -DCMAKE_BUILD_TYPE=[Debug,Release]
      > make all
      > make install

### Building for target Xilinx Zynq ARM {#sect_build_demos_build_xilinx_zynqarm}

Follow the steps below to cross compile your demo application for Zynq ARM:
* Open a shell where the Xilinx ISE 14.7 Toolchain is configured.
  - On a Windows host platform open the `ISE Design Suite [64,32] Bit Command
    Prompt`.
  - On a Linux host platform execute the script `<ISE_ROOT_DIR>/settings[32,64].sh>`
    to configure your current shell.

* Create the executable

      > cd <openPOWERLINK_dir>/apps/<demo_dir>/build/xilinx-zynqarm
      > cmake -G"Unix Makefiles" -DCMAKE_TOOLCHAIN_FILE=../../../../cmake/toolchain-xilinx-zynqarm-eabi-gnu.cmake ../.. -DCMAKE_BUILD_TYPE=[Debug,Release]
      > make all
      > make install

### Building for target Altera ARM {#sect_build_demos_build_altera_arm}

Follow the steps below to cross compile your demo application for Altera Cyclone V SoC ARM:

* Open an "SoC embedded shell".
* Create the executable

      > cd <openPOWERLINK_dir>/apps/<demo_dir>/build/altera-c5socarm
      > cmake -G"Unix Makefiles" -DCMAKE_TOOLCHAIN_FILE=../../../../cmake/toolchain-altera-c5socarm-eabi-gnu.cmake ../.. -DCMAKE_BUILD_TYPE=[Debug,Release]
      > make all
      > make install

# Configuration Options {#sect_build_demos_options}

## Generic Options {#sect_build_demos_generic_options}

- **CFG_DEBUG_LVL**

  Debug level to be used for openPOWERLINK debugging functions.

- **CMAKE_INSTALL_PREFIX**

  Specifies the installation directory where your files will be installed.
  Default directory is: `<openPOWERLINK_dir>/bin/<platform>/<ARCH>`

- **CMAKE_BUILD_TYPE**

  Specifies your build type.
  Valid build types are: _Debug_, _Release_

  If the build type _Debug_ is specified, the code is compiled with debugging
  options.

- **CFG_BUILD_KERNEL_STACK**

  Determines how to build the kernel stack. Depending on your system and
  architecture different options may be available. Please refer to the
  platform specific options.

- **CMAKE_TOOLCHAIN_FILE**

  Specifies a cross toolchain file to be used. It is only needed if you
  cross-compile for another target platform. (\ref sect_build_cmake_crosscompile)

## Linux Specific Options  {#sect_build_demos_linux_options}

- **CFG_BUILD_KERNEL_STACK**

  Determines how to build the kernel stack. The following options are available:
  - __Link to Application__

    The openPOWERLINK kernel part will be directly linked to the user part and
    application. libpcap will be used as Ethernet driver.

  - __Linux User Space Daemon__

    The library `liboplkappXn-userintf.a` will be used. It contains the interface
    to a Linux user space daemon. The kernel part of the openPOWERLINK stack is
    located in the separate user space daemon driver.

  - __Linux Kernel Module__

    The library `liboplkappXn-kernelintf.a` will be used. It contains the interface
    to a Linux kernel module. The kernel part of the openPOWERLINK stack is
    located in the separate kernel module driver.

  - __Stack on PCIe__

    The library `liboplkappmn-kernelpcie.a` will be used. It contains the interface
    to a Linux kernel PCIe interface driver.
    The kernel part of the openPOWERLINK stack is located on an external PCIe
    device. The status/control and data exchange between the application and kernel
    stack is handled by the PCIe interface driver.

## Windows Specific Options  {#sect_build_demos_windows_options}

- **CFG_BUILD_KERNEL_STACK**

  Determines how to build the kernel stack. The following options are available:

  - __Link to Application__

    The openPOWERLINK kernel part will be directly linked to the user part and
    application. WinPCAP will be used as Ethernet driver.

  - __Stack on PCIe__

    The library `liboplkmnapp-pcieintf.lib` will be used. It contains the interface
    to an external PCIe device via an NDIS PCIe miniport driver.
    The kernel part of the openPOWERLINK stack is located on the external PCIe device.
    Shared memory is used for status/control and data exchange between the user
    and kernel layers of the openPOWERLINK stack.

## Options for embedded platforms (Non-OS) {#sect_build_demos_noos_options}

### Microblaze Specific Options  {#sect_build_demos_microblaze_options}

- **CFG_HW_LIB_DIR**

  Path to the hardware platform install directory your application should refer to.
  (e.g: `<openPOWERLINK_DIR>/hardware/lib/generic/microblaze/<BOARD_NAME>/<DEMO_NAME>`)

- **CFG_BUILD_KERNEL_STACK**

  Determines how to build the kernel stack. The following option is available and
  automatically (implicitly) pre-selected:

  - __Link to Application__

    The openPOWERLINK kernel part will be directly linked to the user part and
    application. (Single processor demo)

### Xilinx Zynq ARM Specific Options  {#sect_build_demos_xilinx-zynqarm_options}

- **CFG_HW_LIB_DIR**

  Path to the hardware platform install directory your application should refer to.
  (e.g: `<openPOWERLINK_DIR>/hardware/lib/generic/zynqarm/<BOARD_NAME>/<DEMO_NAME>`)

- **CFG_BUILD_KERNEL_STACK**

  Determines how to build the kernel stack. The following option is available and
  automatically (implicitly) pre-selected:

  - __PCP Daemon using shared memory__

    The library liboplk[mn,cn]app-dualprocshm.a will be used. It contains the interface
    to the kernel daemon running on a separate processor. It uses the dual processor
    shared memory library to communicate with the kernel part of the openPOWERLINK
    running on the second processor.

### Altera Cyclone V SoC ARM Specific Options  {#sect_build_demos_altera-arm_options}

- **CFG_HW_LIB_DIR**

  Path to the hardware platform install directory that the application should refer to.
  (e.g: `<openPOWERLINK_DIR>/hardware/lib/generic/alterac5arm/<BOARD_NAME>/<DEMO_NAME>`)

- **CFG_BUILD_KERNEL_STACK**

  Determines how to build the kernel stack. The following option is available and
  automatically (implicitly) pre-selected:

  - __PCP Daemon using shared memory__

    The library liboplk[mn,cn]app-dualprocshm.a will be used. It contains the interface
    to the kernel daemon running on a separate processor. It uses the dual processor
    shared memory library to communicate with the kernel part of the openPOWERLINK
    stack running on the second processor.

- **CFG_DRV_BLD_PATH**

  Path to the driver daemon build location that the application should refer to.
   (e.g: `<openPOWERLINK_DIR>/drivers/altera-nios2/drv_daemon/build`)

- **CFG_DRV_BIN**

  Driver daemon binary for the NIOSII.
  (default: `drv_daemon.bin`)

- **CFG_FPGA_RBF**
  FPGA configuration file in rbf format.
  (default: `fpga.rbf`)

## Application Specific Options {#sect_build_demos_app_options}

### MN Applications

- **CFG_CFM**

  Determines if the application uses the configuration manager (CFM).

  This option must be enabled if the used openPOWERLINK stack is built with
  configuration manager functionality!
