openPOWERLINK on Windows {#page_platform_windows}
========================

[TOC]

# Introduction {#sect_windows_intro}

This file contains documentation for the openPOWERLINK stack on Windows.

__NOTE:__ Because Windows does not provide real-time behavior, purely
software-based openPOWERLINK solutions running on Windows can only be
run with cycle times above 10ms. The achievable minimum cycle time
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
- Microsoft Visual Studio 2013

__NOTE__: In order to build the Windows NDIS PCIe miniport and NDIS
          intermediate driver, only Microsoft Visual Studio 2013 can be used!
          For details refer to \ref sect_build_drivers_build_windows_ndis

### CMake

For building the openPOWERLINK stack and demo applications the Open Source
cross-platform build tool CMake is used (<http://www.cmake.org>). CMake
version V2.8.4 or higher is required.

For a detailed description of CMake look at the
[cmake section](\ref sect_build_cmake).

### QT5 Development Tools

If you want to build the QT demo application the QT5 development tools must
be installed on the system (<http://www.qt.io/>).

__NOTE:__ In order to automatically deploy the QT dependencies in Windows,
          the CMake install target contains a hard-coded list of dll files.
          This list is only verified with QT 5.5.1 and might need adaptation
          for other QT 5 versions.

### openCONFIGURATOR

The tool [openCONFIGURATOR](\ref page_openconfig) is needed to generate the
network configuration for your application.

### NSIS compiler

NSIS (Nullsoft Scriptable Install System) (<http://nsis.sourceforge.net/Download>)
is a professional Open Source system to create Windows installers.
openPOWERLINK uses NSIS scripts to create the PCIe driver installer for
unattended driver installation on Windows.

# openPOWERLINK Stack Components {#sect_windows_components}

The following section contains a description of the
[openPOWERLINK components](\ref page_components) available on a Windows system.

## Stack Libraries {#sect_windows_components_libs}

The openPOWERLINK stack is divided into a user and a kernel part. On a Windows
system the following configurations are possible:

- __Direct Link to Application__

  The kernel part is directly linked to the user part and the application (complete
  library). The stack uses the WinPCAP library for
  accessing the Ethernet device.

  _Libraries:_
  - `stack/proj/windows/liboplkmn` (liboplkmn.lib)
  - `stack/proj/windows/liboplkcn` (liboplkcn.lib)

- __Kernel stack on PCIe card__

  The application is linked to an application library which contains the interface
  to an external PCIe device running the kernel part of the openPOWERLINK stack.
  An NDIS PCIe miniport driver in Windows kernel space is used to communicate with
  the PCIe device. Shared memory is used for status/control and data exchange
  between the user and kernel layers of the openPOWERLINK stack.

  _Libraries:_
  - `stack/proj/windows/liboplkmnapp-pcieintf` (liboplkmnapp-pcieintf.lib)

- __Windows Kernel Module__

  The application is linked to an application library which contains the interface
  to the NDIS intermediate driver. The kernel part of the openPOWERLINK stack is
  compiled as an NDIS intermediate driver which executes in Windows kernel space.
  The NDIS intermediate driver communicates with the native NIC miniport drivers
  for packet exchange.

  _Libraries:_
  - `stack/proj/windows/liboplkmnapp-kernelintf` (liboplkmnapp-kernelintf.lib)

## Drivers {#sect_windows_components_drivers}

### NDIS PCIe miniport driver

The openPOWERLINK kernel layer may be executed on an external PCIe
device which handles the time critical sections of the openPOWERLINK
stack. This solution allows a Windows user space application to run a POWERLINK
network with cycle times as low as 250us which is otherwise not possible due to
the non-realtime behavior of Windows.

An NDIS PCIe miniport driver is used as a communication interface
between the openPOWERLINK application library and the openPOWERLINK kernel
stack running on the PCIe device using shared memory for data exchange.

The driver is located in: `drivers/windows/drv_ndis_pcie`

### NDIS intermediate driver

The openPOWERLINK kernel layer is compiled as an NDIS intermediate driver.
This solution uses the native NIC miniport drivers for accessing the network
interface and is totally independent of the network card used for communication.

This solution uses the NDIS timer object framework for high resolution timer
support. As the minimum resolution of these timers is 1ms, cycle times lower
than 5ms are not possible with this solution.

The driver is located in: `drivers/windows/drv_ndis_intermediate`

## Demo Applications {#sect_windows_components_demos}

The following demo application are provided on Windows:

* [demo_mn_console](\ref sect_components_demo_mn_console)
* [demo_cn_console](\ref sect_components_demo_cn_console)
* [demo_mn_qt](\ref sect_components_demo_mn_qt)

## Tools {#sect_windows_components_tools}

The NSIS installer script, driver installer and uninstaller application
projects are included as part of the stack to create an installer
application for Windows. This script and the applications help users
to avoid manual steps during installation and configuration of
the Windows NDIS driver.

The script and applications are available in the directory: `tools/windows`

### Building Windows PCIe driver installer

Follow the steps below to build the Windows PCIe driver installer.

* Open a Visual Studio command line and enter the following commands:

  - Build installer application

        > cd <openPOWERLINK_directory>\tools\windows\installer-pcie\build\installer-pcie
        > msbuild /t:build /p:Platform=x64 /p:Configuration="Release"

  - Build uninstaller application

        > cd <openPOWERLINK_directory>\tools\windows\uninstaller-pcie\build\uninstaller-pcie
        > msbuild /t:build /p:Platform=x64 /p:Configuration="Release"

* Copy the required Visual C++ Redistributable executable to `tools/windows/installer`
directory.

  The Visual C++ Redistributable Package is required to run the installer and
  uninstaller applications on the target machine.

  The Visual C++ Redistributable Package for Visual Studio 2013 can be downloaded from:
  (<https://www.microsoft.com/en-in/download/details.aspx?id=40784>)

* Compile the NSIS script to generate the openPOWERLINK driver installer executable.

  The script can be compiled either from the context menu option `Compile NSIS Script` or
  using the command line utility `makensis`.

      > makensis <openPOWERLINK_directory>\tools\windows\installer\openPOWERLINK_demo.nsi

The default installation location for the openPOWERLINK driver installer executable is:
`tools/windows/installer`

### Building NDIS intermediate driver installer

Follow the steps below to build the NDIS intermediate driver installer.

* Open a Visual Studio command line and enter the following commands:

  - Build installer application

        > cd <openPOWERLINK_directory>\tools\windows\installer-ndisim\build

        - Build in 64-bit Environment

             > msbuild /t:build /p:Platform=x64 /p:Configuration="Release"

        - Build in 32-bit Environment

             > msbuild /t:build /p:Platform=Win32 /p:Configuration="Release"

* Copy the required Visual C++ Redistributable executable to `tools/windows/installer`
directory.

  The Visual C++ Redistributable Package is required to run the installer and
  uninstaller applications on the target machine.

  The Visual C++ Redistributable Package for Visual Studio 2013 can be downloaded from:
  (<https://www.microsoft.com/en-in/download/details.aspx?id=40784>)

* Compile the NSIS script to generate the openPOWERLINK driver installer executable.

  Use oplk-ndisim-x64.nsi script for 64-bit Windows and oplk-ndisim-x86.nsi for 32-bit
  Windows driver installation

  The script can be compiled either from the context menu option `Compile NSIS Script` or
  using the command line utility `makensis`.
  Command line utility for 32-bit Windows is given below.

      > makensis <openPOWERLINK_directory>\tools\windows\installer\oplk-ndisim-x86.nsi

  Command line utility for 64-bit Windows is given below.

      > makensis <openPOWERLINK_directory>\tools\windows\installer\oplk-ndisim-x64.nsi

The default installation location for the openPOWERLINK driver installer executable is:
`bin\windows\amd64` for 64-bit and `bin\windows\x86` for 32-bit

__NOTE:__ Currently the script supports creation of executable only for 64-bit Windows.

# Windows driver signing for NDIS drivers

Starting with 64-bit versions of Windows Vista and later versions of Windows, driver
code signing policy requires that all driver code have a digital signature.

This section will give a brief overview of the driver signing requirements on Windows
and explain the steps to use a test-signed driver installation package on test system.

## Driver signing

The Windows device installation system verifies the integrity of device drivers and
the authenticity of the publishers. This requires the publishers to associate a digital
signature with the driver package through driver signing.

Some of the key components required for driver signing are:

- Software Publisher Certificate (SPC) issued by a commercial certificate authority (CA).
- Catalog file with digital signature.

For detailed steps to acquire a certificate, create a catalog and sign the driver, users can
refer the driver signing steps at
<https://msdn.microsoft.com/en-us/library/windows/hardware/ff544865%28v=vs.85%29.aspx>

## Steps for test-signing device drivers during development

Test-signing refers to using a test certificate to sign a pre-release version of a driver
package for use on test computers. In particular, this allows developers to sign kernel-mode
binaries by using self-signed certificates. The tool 'MakeCert' is one of the options to
generate self-signed certificates.

openPOWERLINK Windows NDIS drivers are already configured to produce test-signed drivers
for builds in Debug mode using 'MakeCert' generated certificate.

Please follow the steps below to enable use of test signed driver on a test system:

  - Open a command prompt as an admin and type following commands

        > bcdedit -set loadoptions DISABLE_INTEGRITY_CHECKS
        > bcdedit -set TESTSIGNING ON

  - Reboot the system.
  - Install the openPOWERLINK drivers using the installer.

During the installation of the driver, following message will be displayed to confirm
security exception to allow use of software from an unknown publisher.

![Driver signature warning message.](\ref driver_signature_warn.jpg)

Please select *Install this driver software anyway* to complete the driver installation
successfully.

# Running openPOWERLINK {#sect_windows_running}

The demo applications can be directly started by double-clicking the according
.exe file in Windows explorer.

# Troubleshooting {#sect_windows_trouble}

## Windows NDIS driver

- Visual Studio fails to load

  Make sure that all Visual Studio specific files (e.g. *.sln and *.vcxproj)
  have Windows line endings.

- Driver build fails

  Make sure that the driver INF file has Windows line endings.
