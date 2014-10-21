Building openPOWERLINK Demo Applications {#page_build_demos}
========================================

[TOC]

# Building a Demo Application {#sect_build_demos_build}

The following section describes how the delivered demo applications can be
built. The demo applications are located in the directory `apps`. The default
binary installation path is: `<openPOWERLINK_DIR>/bin/<platform>/<ARCH>`

__NOTE:__ In order to be able to build an application, both, the Debug and the
Release library versions must be available. If one of the libraries is missing
you get an OPLKLIB-NOTFOUND or OPLKLIB_DEBUG-NOTFOUND error in the demos
CMake configuration.


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

## Building for target Microblaze {#sect_build_demos_build_microblaze}

Follow the steps below to cross compile your demo application for Microblaze:
* Open a shell where the Xilinx ISE 14.7 Toolchain is configured.
  - On a Windows host platform open the `ISE Design Suite [64,32] Bit Command
    Prompt`.
  - On a Linux host platform execute the script `<ISE_ROOT_DIR>/settings[32,64].sh>`
    to configure your current shell.

* Creating the executable

      > cd <openPOWERLINK_dir>/apps/<demo_dir>/build/xilinx-microblaze
      > cmake -G"Unix Makefiles" -DCMAKE_TOOLCHAIN_FILE=../../../../cmake/toolchain-xilinx-microblaze-gnu.cmake ../.. -DCMAKE_BUILD_TYPE=[Debug,Release]
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


## Windows Specific Options  {#sect_build_demos_windows_options}

- **CFG_BUILD_KERNEL_STACK**

  Determines how to build the kernel stack. The following option is available and
  automatically (implicitly) pre-selected:

  - __Link to Application__

    The openPOWERLINK kernel part will be directly linked to the user part and
    application. WinPCAP will be used as Ethernet driver.

## Microblaze Specific Options  {#sect_build_demos_microblaze_options}

- **CFG_HW_LIB_DIR**

  Path to the hardware platform install directory your application should refer to.
  (e.g: `<openPOWERLINK_DIR>/hardware/lib/generic/microblaze/<BOARD_NAME>/<DEMO_NAME>`)

- **CFG_BUILD_KERNEL_STACK**

  Determines how to build the kernel stack. The following option is available and
  automatically (implicitly) pre-selected:

  - __Link to Application__

    The openPOWERLINK kernel part will be directly linked to the user part and
    application. (Single processor demo)

## Application Specific Options {#sect_build_demos_app_options}

### MN Applications

- **CFG_DEMO_PROJECT**

  Selects the openCONFIGURATOR project to be used with this demo. Available
  projects are provided by a drop-down list in the cmake QT GUI:

  - __Demo_3CN__: Demo project with 3 CNs using node ID 1, 32, 110. It uses the standard
                  CiA401_CN XDD and shall be used with openPOWERLINK CNs.
  - __Demo_3CN_BC0083__: Demo project with 3CNs using node ID 1, 32, 110. It uses
                       a modified XDD to work with a B&R BC0083 bus controller
                       together with a digital input and digital output module.

- **CFG_CFM**

  Determines if the application uses the configuration manager (CFM).

  This option must be enabled if the used openPOWERLINK stack is built with
  configuration manager functionality!
