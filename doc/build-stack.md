Building openPOWERLINK Stack Libraries {#page_build_stack}
======================================

[TOC]

# Building Stack Libraries{#sect_build_stack_build}

The openPOWERLINK stack itself is implemented via stack libraries. A stack
library can either contain the whole stack (_complete library_), the user part
(_application library_) or the kernel part (_driver library_).

__NOTE:__ In order to be able to build an application, both, the Debug and the Release
library versions must be created.

## Linux {#sect_build_stack_build_linux}

Follow the steps below to build the stack on a Linux system. On Linux, CMake
generates Makefiles by default.

* Creating debug libraries

      > cd openPOWERLINK/stack/build
      > cmake -DCMAKE_BUILD_TYPE=Debug ..
      > make
      > make install

* Creating release libraries

      > cd openPOWERLINK/stack/build
      > cmake -DCMAKE_BUILD_TYPE=Release ..
      > make
      > make install

The default library installation path is: `<openPOWERLINK_DIR>/lib/linux/<ARCH>`


## Windows {#sect_build_stack_build_windows}

Follow the steps below to build the stack on a Windows system using NMake.
Open a Visual Studio command line and enter the following commands:

* Creating debug libraries

      > cd <openPOWERLINK_directory>\stack\build
      > cmake -G"NMake Makefiles" -DCMAKE_BUILD_TYPE=Debug ..
      > nmake
      > nmake install

* Creating release libraries

      > cd <openPOWERLINK_directory>\stack\build
      > cmake -G"NMake Makefiles" -DCMAKE_BUILD_TYPE=Release ..
      > nmake
      > nmake install

The default library installation path is: `<openPOWERLINK_DIR>/lib/windows/<ARCH>`

__NOTE:__ You can also generate a Visual Studio Solution and compile the
libraries in Visual Studio. Please refer to the CMAKE documentation for
generating Visual Studio solution files.

# Configuration Options {#sect_build_stack_options}

If you would like to change the configuration options you have to provide
the configuration options on the command line (-DCFG_XXX=XXX) or
[call CMake in an interactive mode](\ref sect_build_cmake_interactive).

## Generic Configuration Options

- **CFG_DEBUG_LVL**

  Debug level to be used for openPOWERLINK debugging functions.

- **CMAKE_INSTALL_PREFIX**

  Specifies the installation directory where your files will be installed.

  Default directory is: `<openPOWERLINK_DIR>/lib/${SYSTEM_DIR_NAME}/${CMAKE_SYSTEM_PROCESSOR}`

- **CMAKE_BUILD_TYPE**

  Specifies your build type.
  Valid build types are: _Debug_, _Release_

  If the build type _Debug_ is specified, the code is compiled with debugging
  options.


## Linux Configuration Options

- **CFG_COMPILE_LIB_MN**

  Compile complete openPOWERLINK MN library. The library contains an Ethernet
  driver which is using the PCAP library for accessing the network.

- **CFG_COMPILE_LIB_MNAPP_USERINTF**

  Compile openPOWERLINK MN application library which contains the interface
  to a Linux userspace driver. It is used for implementing a multi-process
  solution where the openPOWERLINK kernel layer is running as a separate
  Linux userspace daemon (e.g. a PCAP based userspace daemon)

- **CFG_COMPILE_LIB_MNAPP_KERNELINTF**

  Compile openPOWERLINK MN application library which contains the interface to
  a Linux kernelspace driver. It is used together with a Linux kernel module
  openPOWERLINK driver.

- **CFG_COMPILE_LIB_MNDRV_PCAP**

  Compile openPOWERLINK MN driver library for Linux userspace. This library
  contains the openPOWERLINK kernel layer and uses the PCAP library for accessing
  the network. It is used by the Linux userspace daemon driver.

- **CFG_COMPILE_LIB_CN**

  Compile a complete openPOWERLINK CN library. The library contains an Ethernet
  driver which is using the PCAP library for accessing the network. It is
  configured to contain only CN functionality.

- **CFG_COMPILE_LIB_CNAPP_USERINTF**

  Compile openPOWERLINK CN application library which contains the interface
  to a Linux userspace driver. It is used for implementing a multi-process
  solution where the openPOWERLINK kernel layer is running as a separate
  Linux userspace daemon (e.g. a PCAP based userspace daemon). It is
  configured to contain only CN functionality.

- **CFG_COMPILE_LIB_CNAPP_KERNELINTF**

  Compile openPOWERLINK MN application library which contains the interface to
  a Linux kernelspace driver. It is used together with a Linux kernel module
  openPOWERLINK driver. It is configured to contain only CN functionality.

- **CFG_COMPILE_LIB_CNDRV_PCAP**

  Compile openPOWERLINK CN driver library for Linux userspace. This library
  contains the openPOWERLINK kernel layer and uses the PCAP library for accessing
  the network. It is used by the Linux userspace daemon driver. It is configured
  to contain only CN functionality.


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
