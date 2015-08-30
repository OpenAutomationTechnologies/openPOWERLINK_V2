openPOWERLINK on Windows {#page_platform_windows}
========================

[TOC]

# Introduction {#sect_windows_intro}

This file contains documentation for the openPOWERLINK stack on Windows.

__NOTE:__ Because Windows does not provide real-time behavior, openPOWERLINK
solutions completly running on Windows, can only be run with cycle times above 10ms.
The achievable minimum cycle time depends very much on the used system and
cannot be guaranteed!

# Requirements {#sect_windows_require}

## POWERLINK Network {#sect_windows_plknet}

- POWERLINK network with controlled nodes (CN)
  * openPOWERLINK controlled nodes, e.g. Altera-based FPGA evaluation boards
  * B&R POWERLINK controlled nodes
  * other POWERLINK controlled nodes

## Network Controller {#sect_windows_controller}

On a Windows system openPOWERLINK uses the WinPCAP library to access the
network interface. Therefore, no specific network controller is required.
openPOWERLINK contains the WinPCAP library. It is located in: `contrib\pcap`

- WinPcap:  <http://www.winpcap.org>

## Windows Versions {#sect_windows_winversion}

- Supported Versions: Windows 2000, XP, Vista, 7

## Libraries and Tools {#sect_windows_libs}

### Compiler and Build Environment

openPOWERLINK support the following build environments:

- Microsoft Visual Studio 2005
- Microsoft Visual Studio 2008
- Microsoft Visual Studio 2010
- Microsoft Visual Studio 2013

### CMake

For building the openPOWERLINK stack and demo applications the Open Source
cross-platform build tool CMake is used (<http://www.cmake.org>). CMake
version V2.8.4 or higher is required.

For a detailed description of CMake look at the
[cmake section](\ref sect_build_cmake).

### QT4 Development Tools

If you want to build the QT demo application the QT4 development tools must
be installed on the system (<http://qt.digia.com/>).

### openCONFIGURATOR

The tool [openCONFIGURATOR](\ref page_openconfig) is needed to generate the
network configuration for your application.

### NSIS compiler

NSIS (Nullsoft Scriptable Install System) (<http://nsis.sourceforge.net/Download>)
is a professional open source system to create Windows installers.
openPOWERLINK uses NSIS scripts to create PCIe driver installer for
unattended driver installation on Windows.

# openPOWERLINK Stack Components {#sect_windows_components}

The following section contains a description of the
[openPOWERLINK components](\ref page_components) available on a Windows system.

## Stack Libraries {#sect_windows_components_libs}

The openPOWERLINK stack is divided into a user and a kernel part. On a Windows
system the following configurations are possible:

- __Direct Link to Application__

  The kernel part is directly linked to the user part and application (complete
  library) into a single executable. The stack uses the WinPCAP library for
  accessing the Ethernet device.

  _Libraries:_
  - `stack/proj/windows/liboplkmn` (liboplkmn.lib)
  - `stack/proj/windows/liboplkcn` (liboplkcn.lib)

- __Stack on PCIe__

  The application is linked to an application library which contains the interface
  to an external PCIe device running the kernel part of the openPOWERLINK stack.
  An NDIS PCIe miniport driver in Windows kernel space is used to communicate with
  the PCIe device. Shared memory is used for status/control and data exchange
  between the user and kernel layers of the openPOWERLINK stack.

  _Libraries:_
  - `stack/proj/windows/liboplkmnapp-pcieintf` (liboplkmnapp-pcieintf.lib)

## Drivers {#sect_windows_components_drivers}

### NDIS PCIe miniport driver

The openPOWERLINK kernel layer may be executed on an external PCIe
device which handles the time critical sections of the openPOWERLINK
stack. This solution allows a Windows user space application to achieve
lower cycle lengths(<250us) which is generally not possible due to
non-realtime behaviour of Windows.

An NDIS PCIe miniport driver is used to form the communication interface
between the openPOWERLINK application library and the openPOWERLINK kernel
stack runing on the PCIe device using shared memory for data exchange.

The driver is located in: `drivers/windows/drv_ndis_pcie`

## Demo Applications {#sect_windows_components_demos}

The following demo application are provided on Windows:

* [demo_mn_console](\ref sect_components_demo_mn_console)
* [demo_cn_console](\ref sect_components_demo_cn_console)
* [demo_mn_qt](\ref sect_components_demo_mn_qt)

## Tools {#sect_windows_components_tools}

The NSIS installer script, driver installer and uninstaller application
projects are included as part of the stack to create an installer
application for Windows. This script and applications helps users to
install and configure Windows NDIS PCIe driver avoiding manual steps.

The script and applications are available in directory: `tools/windows`

### Building Windows PCIe driver installer

Follow the steps below to build the Windows PCIe driver installer.

* Open a Visual Studio command line and enter the following commands:

  - Build installer application

        > cd <openPOWERLINK_directory>\tools\windows\installer-pcie\build\installer-pcie
        > msbuild /t:build /p:Platform=x64 /p:Configuration="Release"

  - Build un-installer application

        > cd <openPOWERLINK_directory>\tools\windows\uninstaller-pcie\build\uninstaller-pcie
        > msbuild /t:build /p:Platform=x64 /p:Configuration="Release"

* Copy the required Visual C++ Redistributable executable to `tools/windows/installer`
directory.

  Visual C++ Redistributable Package is required to run the installer and unistaller applications
  on the target machine.

  The Visual C++ Redistributable Packages for Visual Studio 2013 can be downloaded from:
  (<https://www.microsoft.com/en-in/download/details.aspx?id=40784>)

* Compile the NSIS script to generate the openPOWERLINK driver installer executable.

  The script can be compiled either from the context menu option `Compile NSIS Script` or
  using the command line utility `makensis`.

      > makensis <openPOWERLINK_directory>\tools\windows\installer\openPOWERLINK_demo.nsi

The default installation location for the openPOWERLINK driver installer executable is:
`tools/windows/installer`

# Running openPOWERLINK {#sect_windows_running}

The demo applications can be directly started by double-clicking the according
.exe file in Windows explorer.
