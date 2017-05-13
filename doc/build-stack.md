Building openPOWERLINK Stack Libraries {#page_build_stack}
======================================

[TOC]

# Building Stack Libraries{#sect_build_stack_build}

The openPOWERLINK stack itself is implemented via stack libraries. A stack
library can either contain the whole stack (_complete library_), the user part
(_application library_) or the kernel part (_driver library_).

__NOTE:__ In order to be able to build an application with Visual Studio,
both, the Debug and the Release library versions must be available.
If one of the libraries is missing you get an OPLKLIB-NOTFOUND or
OPLKLIB_DEBUG-NOTFOUND error in the demos CMake configuration.
Both libraries are required as you can switch between Debug
and Release inside the Visual Studio solution.

## Linux {#sect_build_stack_build_linux}

Follow the steps below to build the stack on a Linux system. On Linux, CMake
generates Makefiles by default.

* Create debug libraries

      > cd <openPOWERLINK_dir>/stack/build/linux
      > cmake -DCMAKE_BUILD_TYPE=Debug ../..
      > make
      > make install

* Create release libraries

      > cd <openPOWERLINK_dir>/stack/build/linux
      > cmake -DCMAKE_BUILD_TYPE=Release ../..
      > make
      > make install

The default library installation path is: `<openPOWERLINK_dir>/lib/linux/<ARCH>`


## Windows {#sect_build_stack_build_windows}

Follow the steps below to build the stack on a Windows system using NMake.
Open a Visual Studio command line and enter the following commands:

* Create debug libraries

      > cd <openPOWERLINK_dir>\stack\build\windows
      > cmake -G"NMake Makefiles" -DCMAKE_BUILD_TYPE=Debug ..\..
      > nmake
      > nmake install

* Create release libraries

      > cd <openPOWERLINK_dir>\stack\build\windows
      > cmake -G"NMake Makefiles" -DCMAKE_BUILD_TYPE=Release ..\..
      > nmake
      > nmake install

The default library installation path is: `<openPOWERLINK_dir>\lib\windows\<ARCH>`

__NOTE:__ You can also generate a Visual Studio Solution and compile the
libraries in Visual Studio. Please refer to the CMAKE documentation for
generating Visual Studio solution files.

## Embedded Systems (No-OS) {#sect_build_stack_build_noos}

### Xilinx Microblaze (ISE) {#sect_build_stack_build_microblazeise}

Follow the steps below to build the stack library on your host platform:
* Open a shell where the Xilinx ISE 14.7 Toolchain is configured.
  - On a Windows host platform open the `ISE Design Suite [64,32] Bit Command
    Prompt`.
  - On a Linux host platform execute the script `<ISE_ROOT_DIR>/settings[32,64].sh>`
    to configure your current shell.

* Create debug libraries

      > cd <openPOWERLINK_dir>\stack\build\xilinx-microblaze
      > cmake -G"Unix Makefiles" -DCMAKE_TOOLCHAIN_FILE=../../../cmake/toolchain-xilinx-ise-microblaze-gnu.cmake ../.. -DCMAKE_BUILD_TYPE=Debug -DCFG_COMPILE_LIB_[LIB_NAME]=ON -DCFG_COMPILE_LIB_[LIB_NAME]_LIB_DIR=[PATH_TO_HW_LIB]
      > make all
      > make install

  This will create the `[LIB_NAME]` stack library (debug) for the hardware library in `[PATH_TO_HW_LIB]`.
  Multiple stack libraries can be built together by passing the define pairs (`CFG_COMPILE_LIB_[LIB_NAME]`
  and `CFG_COMPILE_LIB_[LIB_NAME]_LIB_DIR`) for each stack library to CMake.
  Refer to \ref sect_build_stack_options_noos_microblaze for details!

* Create release libraries

      > cd <openPOWERLINK_dir>\stack\build\xilinx-microblaze
      > cmake -G"Unix Makefiles" -DCMAKE_TOOLCHAIN_FILE=../../../cmake/toolchain-xilinx-ise-microblaze-gnu.cmake ../.. -DCMAKE_BUILD_TYPE=Release -DCFG_COMPILE_LIB_[LIB_NAME]=ON -DCFG_COMPILE_LIB_[LIB_NAME]_LIB_DIR=[PATH_TO_HW_LIB]
      > make all
      > make install

  This will create the `[LIB_NAME]` stack library for the hardware library in `[PATH_TO_HW_LIB]`.
  Multiple stack libraries can be built together by passing the define pairs (`CFG_COMPILE_LIB_[LIB_NAME]`
  and `CFG_COMPILE_LIB_[LIB_NAME]_LIB_DIR`) for each stack library to CMake.
  Refer to \ref sect_build_stack_options_noos_microblaze for details!

The default library installation path is:
`<openPOWERLINK_dir>/stack/lib/generic/microblaze/<BOARD_NAME>/<DEMO_NAME>`

### Xilinx Microblaze (Vivado) {#sect_build_stack_build_microblaze}

Follow the steps below to build the stack library on your host platform:
* Open a shell where the Xilinx Vivado 2016.2 Toolchain is configured:
  - On a Windows host platform open the `Vivado TCL console 2016.2` and run the following commands
    to load the environment:

        > <Vivado_ROOT_DIR>/2016.2/bin/vivado -mode tcl
        > xsct (To set the SDK environment path)
        > vivado -mode tcl

  - On a Linux host platform execute the settings script provided by the toolchain:

        > /opt/Xilinx/Vivado/2016.2/settings64.sh
        > /opt/Xilinx/SDK/2016.2/settings64.sh

* Create debug libraries

  * On a Windows host platform

        > cd <openPOWERLINK_dir>\stack\build\xilinx-microblaze
        > cmake -GUnix\ Makefiles -DCMAKE_TOOLCHAIN_FILE=../../../cmake/toolchain-xilinx-microblaze-gnu.cmake ../.. -DCMAKE_BUILD_TYPE=Debug -DCFG_COMPILE_LIB_[LIB_NAME]=ON -DCFG_COMPILE_LIB_[LIB_NAME]_LIB_DIR=[PATH_TO_HW_LIB]
        > make all
        > make install

  * On a Linux host platform

        > cd <openPOWERLINK_dir>\stack\build\xilinx-microblaze
        > cmake -DCMAKE_TOOLCHAIN_FILE=../../../cmake/toolchain-xilinx-microblaze-gnu.cmake ../.. -DCMAKE_BUILD_TYPE=Debug -DCFG_COMPILE_LIB_[LIB_NAME]=ON -DCFG_COMPILE_LIB_[LIB_NAME]_LIB_DIR=[PATH_TO_HW_LIB]
        > make all
        > make install

  This will create the `[LIB_NAME]` stack library (debug) for the hardware library in `[PATH_TO_HW_LIB]`.
  Multiple stack libraries can be built together by passing the define pairs (`CFG_COMPILE_LIB_[LIB_NAME]`
  and `CFG_COMPILE_LIB_[LIB_NAME]_LIB_DIR`) for each stack library to CMake.
  Refer to \ref sect_build_stack_options_noos_microblaze for details!

* Create release libraries

  * On a Windows host platform

        > cd <openPOWERLINK_dir>\stack\build\xilinx-microblaze
        > cmake -GUnix\ Makefiles -DCMAKE_TOOLCHAIN_FILE=../../../cmake/toolchain-xilinx-microblaze-gnu.cmake ../.. -DCMAKE_BUILD_TYPE=Release -DCFG_COMPILE_LIB_[LIB_NAME]=ON -DCFG_COMPILE_LIB_[LIB_NAME]_LIB_DIR=[PATH_TO_HW_LIB]
        > make all
        > make install

  * On a Linux host platform

        > cd <openPOWERLINK_dir>\stack\build\xilinx-microblaze
        > cmake -DCMAKE_TOOLCHAIN_FILE=../../../cmake/toolchain-xilinx-microblaze-gnu.cmake ../.. -DCMAKE_BUILD_TYPE=Release -DCFG_COMPILE_LIB_[LIB_NAME]=ON -DCFG_COMPILE_LIB_[LIB_NAME]_LIB_DIR=[PATH_TO_HW_LIB]
        > make all
        > make install

  This will create the `[LIB_NAME]` stack library for the hardware library in `[PATH_TO_HW_LIB]`.
  Multiple stack libraries can be built together by passing the define pairs (`CFG_COMPILE_LIB_[LIB_NAME]`
  and `CFG_COMPILE_LIB_[LIB_NAME]_LIB_DIR`) for each stack library to CMake.
  Refer to \ref sect_build_stack_options_noos_microblaze for details!

The default library installation path is:
`<openPOWERLINK_dir>/stack/lib/generic/microblaze/<BOARD_NAME>/<DEMO_NAME>`

### Altera Cyclone V SoC ARM {#sect_build_stack_build_c5socarm-altera}

Follow the steps below to build the stack library on your host platform:

* Open SoC embedded command shell.

* Create debug libraries

      > cd <openPOWERLINK_dir>\stack\build\altera-c5socarm
      > cmake -G"Unix Makefiles" -DCMAKE_TOOLCHAIN_FILE=../../../cmake/toolchain-altera-c5socarm-eabi-gnu.cmake ../.. -DCMAKE_BUILD_TYPE=Debug -DCFG_COMPILE_LIB_[LIB_NAME]=ON -DCFG_COMPILE_LIB_[LIB_NAME]_LIB_DIR=[PATH_TO_HW_LIB]
      > make all
      > make install

  This will create the `[LIB_NAME]` stack library (debug) for the hardware library in `[PATH_TO_HW_LIB]`.
  Multiple stack libraries can be built together by passing the define pairs (`CFG_COMPILE_LIB_[LIB_NAME]`
  and `CFG_COMPILE_LIB_[LIB_NAME]_LIB_DIR`) for each stack library to CMake.
  Refer to \ref sect_build_stack_options_noos_c5socarm for details!

* Create release libraries

      > cd <openPOWERLINK_dir>\stack\build\altera-c5socarm
      > cmake -G"Unix Makefiles" -DCMAKE_TOOLCHAIN_FILE=../../toolchain-altera-c5socarm-eabi-gnu.cmake ../.. -DCMAKE_BUILD_TYPE=Release -DCFG_COMPILE_LIB_[LIB_NAME]=ON -DCFG_COMPILE_LIB_[LIB_NAME]_LIB_DIR=[PATH_TO_HW_LIB]
      > make all
      > make install

  This will create the `[LIB_NAME]` stack library for the hardware library in `[PATH_TO_HW_LIB]`.
  Multiple stack libraries can be built together by passing the define pairs (`CFG_COMPILE_LIB_[LIB_NAME]`
  and `CFG_COMPILE_LIB_[LIB_NAME]_LIB_DIR`) for each stack library to CMake.
  Refer to \ref sect_build_stack_options_noos_c5socarm for details!

The default library installation path is:
`<openPOWERLINK_dir>/stack/lib/generic/alterac5arm/<BOARD_NAME>/<DEMO_NAME>`

# Configuration Options {#sect_build_stack_options}

If you would like to change the configuration options you have to provide
the configuration options on the command line (-DCFG_XXX=XXX) or
[call CMake in an interactive mode](\ref sect_build_cmake_interactive).

## Generic Configuration Options

- **CFG_DEBUG_LVL**

  Debug level to be used for openPOWERLINK debugging functions.

- **CMAKE_INSTALL_PREFIX**

  Specifies the installation directory where your files will be installed.

  Default directory is: `<openPOWERLINK_dir>/lib/${SYSTEM_DIR_NAME}/${CMAKE_SYSTEM_PROCESSOR}`

- **CMAKE_BUILD_TYPE**

  Specifies your build type.
  Valid build types are: _Debug_, _Release_

  If the build type _Debug_ is specified, the code is compiled with debugging
  options.

- **CMAKE_TOOLCHAIN_FILE**

  Specifies a cross toolchain file to be used. It is only needed if you
  cross-compile for another target platform. (\ref sect_build_cmake_crosscompile)

## Linux Configuration Options

- **CFG_COMPILE_LIB_MN**

  Compile complete openPOWERLINK MN library. The library contains an Ethernet
  driver which is using the PCAP library for accessing the network.

- **CFG_COMPILE_LIB_MNAPP_USERINTF**

  Compile openPOWERLINK MN application library which contains the interface
  to a Linux user space driver. It is used for implementing a multi-process
  solution where the openPOWERLINK kernel layer is running as a separate
  Linux user space daemon (e.g. a PCAP based user space daemon)

- **CFG_COMPILE_LIB_MNAPP_KERNELINTF**

  Compile openPOWERLINK MN application library which contains the interface to
  a Linux kernel space driver. It is used together with a Linux kernel module
  openPOWERLINK driver.

- **CFG_COMPILE_LIB_MNDRV_PCAP**

  Compile openPOWERLINK MN driver library for Linux user space. This library
  contains the openPOWERLINK kernel layer and uses the PCAP library for accessing
  the network. It is used by the Linux user space daemon driver.

- **CFG_COMPILE_LIB_CN**

  Compile a complete openPOWERLINK CN library. The library contains an Ethernet
  driver which is using the PCAP library for accessing the network. It is
  configured to contain only CN functionality.

- **CFG_COMPILE_LIB_CNAPP_USERINTF**

  Compile openPOWERLINK CN application library which contains the interface
  to a Linux user space driver. It is used for implementing a multi-process
  solution where the openPOWERLINK kernel layer is running as a separate
  Linux user space daemon (e.g. a PCAP based user space daemon). It is
  configured to contain only CN functionality.

- **CFG_COMPILE_LIB_CNAPP_KERNELINTF**

  Compile openPOWERLINK MN application library which contains the interface to
  a Linux kernel space driver. It is used together with a Linux kernel module
  openPOWERLINK driver. It is configured to contain only CN functionality.

- **CFG_COMPILE_LIB_CNDRV_PCAP**

  Compile openPOWERLINK CN driver library for Linux user space. This library
  contains the openPOWERLINK kernel layer and uses the PCAP library for accessing
  the network. It is used by the Linux user space daemon driver. It is configured
  to contain only CN functionality.

- **CFG_COMPILE_LIB_MNAPP_PCIEINTF**

  Compile openPOWERLINK MN application library which contains the interface to
  a Linux kernel PCIe interface driver. It is used along with a Linux kernel
  PCIe interface driver, for status/control and data exchange with the kernel
  stack which runs on an external PCIe device.

- **CFG_COMPILE_LIB_MNAPP_ZYNQINTF**

  Compile openPOWERLINK MN application library for Zynq platform, that contains the
  interface to a Linux kernel platform interface driver. It is used along with the Linux
  kernel platform interface driver for status/control and data exchange with the kernel
  stack which runs on Zynq Microblaze as a baremetal application.

- **CFG_COMPILE_LIB_CN_SIM**

  Compile a complete openPOWERLINK CN library including an open simulation
  interface. The library contains the connection to a simulated Ethernet driver,
  High Resolution Timer, User Timer, Target System, modules for Process Sync and
  API Event. The connection to a simulation environment can be established by
  exported functions and function pointers. It is configured to contain only CN
  functionality.

- **CFG_COMPILE_LIB_MN_SIM**

  Compile a complete openPOWERLINK MN library including an open simulation
  interface. The library contains the connection to a simulated Ethernet driver,
  High Resolution Timer, User Timer, Target System, modules for Process Sync and
  API Event. The connection to a simulation environment can be established by
  exported functions and function pointers.

## Windows Configuration Options

- **CFG_WINDOWS_DLL**

  If this option is set to ON, the libraries will be compiled as dynamic link
  libraries (DLL) instead of static libraries.

- **CFG_COMPILE_LIB_MN**

  Compile complete openPOWERLINK MN library. The library contains an Ethernet
  driver which is using the WinPCAP library for accessing the network.

- **CFG_COMPILE_LIB_CN**

  Compile complete openPOWERLINK CN library. The library contains an Ethernet
  driver which is using the WinPCAP library for accessing the network. It is
  configured to contain only CN functionality.

- **CFG_COMPILE_LIB_MNAPP_PCIEINTF**

  Compile openPOWERLINK MN application library which contains the interface to
  the openPOWERLINK driver running on an external PCIe device. It is used together
  with an NDIS PCIe miniport driver in the Windows kernel space. The NDIS PCIe
  miniport driver is used as the communication interface between the application
  library and the PCIe device running the openPOWERLINK kernel layer. The NDIS
  PCIe driver uses shared memory for status/control information and data
  exchange between application library and the PCIe device.

- **CFG_COMPILE_LIB_MNAPP_KERNELINTF**

  Compile openPOWERLINK MN application library which contains interface to the
  NDIS intermediate driver running in Windows kernel space. The kernel layer of
  the stack is located in the NDIS intermediate driver. The NDIS driver
  communicates with the native NIC miniport driver to access the network
  interface.

- **CFG_COMPILE_LIB_CN_SIM**

  Compile a complete openPOWERLINK CN library including an open simulation
  interface. The library contains the connection to a simulated Ethernet driver,
  High Resolution Timer, User Timer, Target System, modules for Process Sync and
  API Event. The connection to a simulation environment can be established by
  exported functions and function pointers. It is configured to contain only CN
  functionality.

- **CFG_COMPILE_LIB_MN_SIM**

  Compile a complete openPOWERLINK MN library including an open simulation
  interface. The library contains the connection to a simulated Ethernet driver,
  High Resolution Timer, User Timer, Target System, modules for Process Sync and
  API Event. The connection to a simulation environment can be established by
  exported functions and function pointers.

## Options for embedded systems (No-OS) {#sect_build_stack_options_noos}

### Microblaze Configuration Options (ISE) {#sect_build_stack_options_noos_microblaze}

- **CFG_COMPILE_LIB_CN**

  Compile complete openPOWERLINK CN library. The library consists of the user and
  kernel parts of the stack. It is configured to contain only CN functionality.
  - __CFG_COMPILE_LIB_CN_HW_LIB_DIR__

    Specify the path to the hardware platform the CN library should refer to.
    The path to the hardware platform should point to the export folder of the hardware
    project. (e.g: `<openPOWERLINK_dir>/hardware/lib/generic/microblaze/<BOARD_NAME>/<DEMO_NAME>`)

- **CFG_COMPILE_LIB_MNDRV_HOSTIF**

  Compile openPOWERLINK MN driver library for Xilinx Microblaze. This library
  contains the openPOWERLINK kernel layer and uses the openMAC driver for
  accessing the network. It communicates with the openPOWERLINK user part by
  using the host interface IP-Core. It can be used to implement
  an openPOWERLINK driver (PCP) on dual processor designs where the openPOWERLINK
  user layer is running on a separate processor connected via the host interface.

  - __CFG_COMPILE_LIB_MN_HW_LIB_DIR__

  Specify the path to the hardware platform the driver library should refer to.
  The path to the hardware platform should point to the export folder of the hardware
  project. (e.g: `<openPOWERLINK_dir>/hardware/lib/generic/microblaze/<BOARD_NAME>/<DEMO_NAME>`)

### Altera Cyclone V SoC ARM Configuration Options {#sect_build_stack_options_noos_c5socarm}

- **CFG_COMPILE_LIB_MNAPP_DUALPROCSHM**

  Compile the openPOWERLINK MN application library for Altera Cyclone V SoC ARM.
  The library contains the openPOWERLINK user layer with CAL modules for
  accessing the dual processor shared memory library. The dual processor
  shared memory library is used to communicate with the kernel part of
  openPOWERLINK. The library can be used to implement an application
  designed to run on non-OS C5SoC ARM processor communicating with the
  openPOWERLINK kernel layer running on a separate processor connected
  via a shared memory.

    - __CFG_COMPILE_LIB_MN_HW_LIB_DIR__

  Specifies the path to the hardware platform the application library should refer to.
  The path to the hardware platform should point to the export folder of the hardware
  project. (e.g: `<openPOWERLINK_dir>/hardware/lib/generic/alterac5arm/<BOARD_NAME>/<DEMO_NAME>`)
