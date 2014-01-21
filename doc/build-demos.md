Building openPOWERLINK Demo Applications {#page_build_demos}
========================================

[TOC]

# Building a Demo Application {#sect_build_demos_build}

The following section describes how the delivered demo applications can be
built. The demo applications are located in the directory `apps`. The default
binary installation path is: `<openPOWERLINK_DIR>/bin/<platform>/<ARCH>`

__NOTE:__ In order to be able to build an application, both, the Debug and the Release
library versions must be available.


## Building on Linux {#sect_build_demos_build_linux}

      > cd <openPOWERLINK_dir>/apps/<demo_dir>/build/linux
      > cmake ../..
      > make
      > make install

## Building on Windows {#sect_build_demos_build_windows}

Open a Visual Studio command line and enter the following commands:

      > cd <openPOWERLINK_dir>/apps/<demo_dir>/build/windows
      > cmake -G"NMake Makefiles" ../..
      > nmake
      > nmake install

__NOTE:__ You can also generate a Visual Studio Solution and compile the
libraries in Visual Studio. Please refer to the CMAKE documentation for
generating Visual Studio solution files.


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

## Linux Specific Options  {#sect_build_demos_linux_options}

- **CFG_BUILD_KERNEL_STACK**

  Determines how to build the kernel stack. The following options are available:
  - __Link to Application__

    The openPOWERLINK kernel part will be directly linked to the user part and
    application. libpcap will be used as Ethernet driver.

  - __Linux Userspace Daemon__

    The library `liboplkappXn-userintf.a` will be used. It contains the interface
    to a Linux userspace daemon. The kernel part of the openPOWERLINK stack is
    located in the separate userspace daemon driver.

  - __Linux Kernel Module__

    The library `liboplkappXn-kernelintf.a` will be used. It contains the interface
    to a Linux kernel module. The kernel part of the openPOWERLINK stack is
    located in the separate kernel module driver.


## Windows Specific Options  {#sect_build_demos_windows_options}

- **CFG_BUILD_KERNEL_STACK**

  Determines how to build the kernel stack. The following option is available and
  automatically (implicitely) pre-selected:

  - __Link to Application__

    The openPOWERLINK kernel part will be directly linked to the user part and
    application. WinPCAP will be used as Ethernet driver.
