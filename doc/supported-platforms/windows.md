openPOWERLINK on Windows {#page_platform_windows}
========================

[TOC]

# Introduction {#sect_windows_intro}

This file contains documentation for the openPOWERLINK stack on Windows.

__NOTE:__ Because Windows does not provide real-time behavior, openPOWERLINK can
only be run with cycle times above 10ms. The achievable minimum cycle time
depends very much on the used system and cannot be guaranteed!

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

# openPOWERLINK Stack Components {#sect_windows_components}

The following section contains a description of the
[openPOWERLINK components](\ref page_components) available on a Windows system.

## Stack Libraries

The openPOWERLINK stack is divided into a user and a kernel part. On a Windows
system only a single process solution is available, in which the whole openPOWERLINK
stack is linked to the application. The WinPCAP library is used to access the
Ethernet interface.

_Libraries:_
- `stack/proj/windows/liboplkmn` (liboplkmn.lib)
- `stack/proj/windows/liboplkcn` (liboplkcn.lib)

## Demo Applications

The following demo application are provided on Windows:

* [demo_mn_console](\ref sect_components_demo_mn_console)
* [demo_cn_console](\ref sect_components_demo_cn_console)
* [demo_mn_qt](\ref sect_components_demo_mn_qt)

# Running openPOWERLINK {#sect_windows_running}

The demo applications can be directly started by double-clicking the according
.exe file in Windows explorer.
