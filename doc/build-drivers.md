Building openPOWERLINK Drivers {#page_build_drivers}
==============================

[TOC]

# Building Drivers {#sect_build_drivers_build}

If openPOWERLINK is compiled with separate application and driver libraries,
the driver library needs to be linked to an executable to create a kernel mode
driver or a user mode daemon/service. In either way, openPOWERLINK calls the
result a _driver_.

The drivers are located in the directory `drivers`. To build a driver, the
necessary openPOWERLINK libraries must be compiled and installed.

__NOTE__: You don't need to compile a driver if you are using a single process
solution. (e.g. Linux PCAP single process, Windows PCAP)

## Building a Linux PCAP User Space Daemon {#sect_build_drivers_build_linux_pcap}

The user space daemon needs the PCAP driver libraries (`liboplkmndrv-pcap`, `liboplkcndrv-pcap`).
They must be compiled and installed to be able to compile the user space daemon.

To build the user space daemon (e.g. for an MN):

      > cd <openPOWERLINK_dir>/drivers/linux/drv_daemon_pcap/build
      > cmake -DCFG_OPLK_MN=TRUE ..
      > make
      > make install


## Building a Linux Edrv Kernel Driver {#sect_build_drivers_build_linux_edrv}

To build the kernel space driver the appropriate kernel sources must be installed
on your system. The path to the kernel sources can be configured by
__CFG_KERNEL_DIR__.

To build the kernel driver (e.g. for a MN using the Intel 82573 network interface):

      > cd <openPOWERLINK_dir>/drivers/linux/drv_kernelmod_edrv/build
      > cmake -DCFG_OPLK_MN=TRUE -DCFG_POWERLINK_EDRV=82573 ..
      > make
      > make install

## Building a Linux Kernel PCIe Interface Driver {#sect_build_drivers_build_linux_pcie}

To build the kernel space driver, the appropriate kernel sources must be installed
on your system. The path to the kernel sources can be configured by
__CFG_KERNEL_DIR__.

To build the kernel PCIe interface driver:

      > cd <openPOWERLINK_dir>/drivers/linux/drv_kernelmod_pcie/build
      > cmake -DCFG_OPLK_MN=TRUE
      > make
      > make install

The default driver installation path is: `<openPOWERLINK_DIR>/bin/linux/<ARCH>/oplkdrv_kernelmodule_pcie`

## Building a Windows NDIS PCIe miniport driver {#sect_build_drivers_build_windows_ndis_pcie}

To build the Windows NDIS PCIe miniport driver, an appropriate Windows Driver Kit (WDK)
version which supports the Windows version of the host system has to be installed
on the development system.

__NOTE__: The Windows PCIe driver currently provided with openPOWERLINK, can be used
with Windows 7 (64 bit) and requires Windows Driver Kit (WDK) 8.1 for compilation.
(<http://www.microsoft.com/en-us/download/details.aspx?id=42273>)


Follow the steps below to build the NDIS PCIe driver on a Windows system using MSbuild.
Open a Visual Studio command line and enter the following commands:

* Build driver for Windows 7 (64 bit) in debug mode

      > cd <openPOWERLINK_directory>\drivers\windows\drv_ndis_pcie\build
      > cmake -G"NMake Makefiles" -DCMAKE_BUILD_TYPE=Debug ..
      > msbuild /t:build /p:Platform=x64 /p:Configuration="Win7 Debug"

* Build driver for Windows 7 (64 bit) in release mode

      > cd <openPOWERLINK_directory>\drivers\windows\drv_ndis_pcie\build
      > cmake -G"NMake Makefiles" -DCMAKE_BUILD_TYPE=Release ..
      > msbuild /t:build /p:Platform=x64 /p:Configuration="Win7 Release"

`Platform` and `Configuration` parameters can be modified to compile the driver for
a different platform and Windows version.

The default driver installation path is: `<openPOWERLINK_DIR>\bin\windows\<ARCH>\drv_ndis_pcie_package`

## Building a PCP daemon for Microblaze {#sect_build_drivers_build_daemon_microblaze}

This section will explain the steps to build the PCP daemon for a Microblaze
softcore processor in a dual processor design.
The PCP daemon uses the driver library for the dual processor shared memory interface
(`liboplkmndrv-dualprocshm`).

To build the PCP daemon (e.g. for Microblaze in Zynq SoC's programming logic (PL) using shared memory interface):

      > cd <openPOWERLINK_dir>/drivers/xilinx-microblaze/drv_daemon/build
      > cmake -G"Unix Makefiles" -DCMAKE_TOOLCHAIN_FILE=../../../cmake/toolchain-xilinx-microblaze-gnu.cmake ../.. -DCMAKE_BUILD_TYPE=Release ..
      > make
      > make install

# Configuration Options {#sect_build_drivers_options}

## Generic Options {#sect_build_drivers_options_generic}

- **CFG_DEBUG_LVL**

  Debug level to be used for openPOWERLINK debugging functions.

- **CMAKE_INSTALL_PREFIX**

  Specifies the installation directory where your files will be installed.
  Default directory is: `<openPOWERLINK_dir>/bin/<system>/<ARCH>`

- **CMAKE_BUILD_TYPE**

  Specifies your build type.
  Valid build types are: _Debug_, _Release_

  If the build type _Debug_ is specified, the code is compiled with debugging
  options.

- **CMAKE_TOOLCHAIN_FILE**

  Specifies a cross toolchain file to be used. It is only needed if you
  cross-compile for another target platform. (\ref sect_build_cmake_crosscompile)

## Linux PCAP User Space Daemon {#sect_build_drivers_options_linux_pcap}

- **CFG_OPLK_MN**

  If enabled, the POWERLINK stack will be compiled with MN functionality, otherwise
  it will be compiled only with CN functionality.


## Linux Edrv Kernel Driver {#sect_build_drivers_options_linux_edrv}

- **CFG_KERNEL_DIR**

  The directory where the kernel sources used for building the kernel modules
  are located. If it is not set, the sources of the currently running kernel
  will be used:
  `/lib/modules/$(shell uname -r)/build`

- **CFG_OPLK_MN**

  If enabled, the openPOWERLINK stack will be compiled with MN functionality,
  otherwise it will be compiled only with CN functionality.

- **CFG_POWERLINK_EDRV**

  Selects the Ethernet driver used for the kernel-based stack and demos.
  Valid options are:

  - **8139**:  Realtek 8139-based network interface cards (100 MBit/s)
  - **8111**:  Realtek 8111/8168 network interface cards (1 GBit/s)
  - **8255x**: Intel 8255x-based network interface cards (100 MBit/s)
  - **82573**: Intel 82573-based network interface cards (1 GBit/s)
  - **i210**:  Intel I210-based network interface cards (1 GBit/s)
  - **emacps**:Zynq Emac network interface controller (1 GBit/s)

## PCP daemon on Microblaze {#sect_build_drivers_options_pcp_daemon}

Following options are available for a PCP daemon of a dual processor design:

- **CFG_HW_LIB_DIR**
  Path to the hardware platform installation directory your daemon should refer to.
  (e.g: `<openPOWERLINK_DIR>/hardware/lib/generic/microblaze/<BOARD_NAME>/<DEMO_NAME>`)

- **CFG_BUILD_KERNEL_STACK**

  Determines which driver library of the stack should be linked with the daemon.
  The following options are available and automatically (implicitly) pre-selected:

  - __PCP Daemon Dual-Proc__

    The openPOWERLINK user part will be running on a separate processor
    communicating with the daemon through shared memory (dual processor).

  - __host-interface__

    The openPOWERLINK user part will be running on a separate processor
    communicating with the daemon through the host interface IP (dual processor).
