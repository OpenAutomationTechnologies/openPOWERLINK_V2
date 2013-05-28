Building with CMake {#cmake}
===================

For building openPOWERLINK on Linux and Windows the build utility CMake
([http://www.cmake.org](http://www.cmake.org)) is used. CMake controls the
software compilation process using simple platform and compiler independent
configuration files. Depending on the architecture different build
files can be created. On Linux CMake may create:

* Unix Makefiles (tested)
* Eclipse CDT4 - Unix Makefiles
* Kdevelop3 - Unix Makefiles
* CodeBlocks - Unix Makefiles

## Executing CMake

The configuration of different build options of the openPOWERLINK stack will
be done through the cmake interface.

There are three possible ways for configuring your build options

* `cmake-gui`

  This is the most comfortable way for configuring the build options. cmake-gui
  is a graphical user interface for CMake.

* `ccmake`

  ccmake is a curses based user interface for cmake.

* `cmake -i`

  If cmake is called with option -i you can interactively select your build
  options.

## Configuration Options

The following build configuration options are available:

- **CFG_DEBUG_LVL**

  Debug level to be used for openPOWERLINK debugging functions.

- **CFG_POWERLINK_MN**

  If enabled, the POWERLINK stack will be compiled with MN functionality otherwise
  it will be compiled only with CN functionality.

- **CFG_DEMO_MN_CONSOLE**

  If enabled the MN console application will be compiled.

  Requires: *CFG_POWERLINK_MN*

- **CFG_DEMO_CN_CONSOLE**

  If enabled the CN console application will be compiled.

  Requires: *CFG_POWERLINK_MN = OFF*

- **CFG_DEMO_MN_CONSOLE_USE_SYNCTHREAD**

  If enabled the MN console application starts a separate thread for
  the synchronous (PDO) data handling.

  Requires: *CFG_DEMO_MN_CONSOLE*

- **CFG_DEMO_MN_QT**

  If enabled the MN QT application will be compiled. It can be used either with
  the kernel stack or with the userspace stack.

  Requires: *CFG_POWERLINK_MN*

- **CMAKE_INSTALL_PREFIX**

  Specifies the installation directory where your files will be installed.
  Default directory is: `${CMAKE_BUILD_DIR}/bin`

- **CMAKE_BUILD_TYPE**

  Specifies your build type.
  Valid build types are: Debug,Release
  If the build type debug is specified, the code is compiled with debugging
  options.

- **CFG_BUILD_UNITTESTS**

  If enabled, the unit tests will be compiled.

### Linux Configuration Options

- **CFG_BUILD_KERNEL_STACK**

  Determines how to build the kernel stack. The following options are available:
  - Link to Application

    The openPOWERLINK kernel part will be directly linked to the user part and
    application. libpcap will be used as ethernet driver.

  - Linux Userspace Daemon

    The openPOWERLINK kernel part will be compiled as a separate userspace process.
    libpcap will be used as ethernet driver.

  - Linux Kernel Module

    The openPOWERLINK kernel part will be compiled as a Linux kernel module.
    The configured Linux ethernet driver will be used.

- **CFG_KERNEL_DIR**

  The directory where the kernel sources used for building the kernel modules
  are located. If it is not set the sources of the running kernel will be used:
  `/lib/modules/$(shell uname -r)/build`

  Requires: *CFG_BUILD_KERNEL_STACK = Linux Kernel Module*

- **CFG_POWERLINK_EDRV**

  Selects the Ethernet driver used for the kernel based stack and demos.
  Valid options are:

  - **8139**:  Realtek 8139 based network interface cards
  - **82573**: Intel 82573 based network interface cards
  - **8255x**: Intel 8255x 100MBit based network interface cards

  Requires: *CFG_BUILD_KERNEL_STACK = Linux Kernel Module*

### Windows Configuration Options

- **CFG_BUILD_KERNEL_STACK**

  Determines how to build the kernel stack. The following options are available:
  - Link to Application

  The openPOWERLINK kernel part will be directly linked to the user part and
  application. winpcap will be used as ethernet driver.


## Out-of-Source Builds

CMake support out-of-source builds. Therefore, all generated files are located
in a seperate build directory which keeps your sources clean. It is recommended
to use a separate build directory for building the openPOWERLINK stack and the
demo applications.

