openPOWERLINK on Xilinx Microblaze {#page_platform_xilinx}
==================================

[TOC]

# Introduction {#sect_xilinx_intro}

This file contains documentation for the openPOWERLINK stack on Xilinx
Microblaze. It uses the openMAC IP-Core which is an optimized MAC
for the POWERLINK protocol. Additionally the IP-Core consists of a hub for
daisy chaining several controlled nodes.

## Contents {#sect_xilinx_intro_contents}

- FPGA design with Microblaze CPU and openMAC IP-Core.
- Digital I/0 Ports: 4 x 8Bit.\n
  Direction (input or output) can be defined by changing the application
  software.

## Performance Data {#sect_xilinx_intro_performance}

- Minimum cycle length: 400 us
- PReq-PRes Latency: 1 us
- Process data: 4 bytes input and 4 bytes output.
- There is 1 RPDO and 1 TPDO available.

# Requirements {#sect_xilinx_requirements}

## Development Boards {#sect_xilinx_requirements_boards}
An embedded development board with a Xilinx Spartan6 FPGA and a set of the
correct peripherals for openPOWERLINK is needed. The following Xilinx boards are
suited best to run the openPOWERLINK stack.

### Avnet Spartan6 POWERLINK Evaluation Board
This board from Avnet is designed to run the openPOWERLINK stack and can be
ordered from: http://www.em.avnet.com/en-us/design/drc/Pages/Xilinx-Spartan6-FPGA-Ethernet-Powerlink-Kit.aspx

Carry out the following steps to adjust the boards hardware setup:
* Connect the following cables:
  - Connect the power supply to connector CON600 or CON601.
  - Connect a USB cable from the 'Digilent USB Jtag' (CON101) to your host PC.
  - Connect a USB cable from the 'Jtag Uart' (CON100) to your host PC.
  - Connect the Ethernet jack on J400 or J401 to the Ethernet port of your PC.
* Set the following jumpers
  - Install jumper on J500 pins 2-3.
  - Install jumper on J501 pins 2-3.
  - Install jumper on J502 pins 1-2, 4-5 and 3-6.
* Set the node switches (SW500 and SW501) to a valid value.


## POWERLINK network {#sect_xilinx_requirements_network}

- POWERLINK network with a managing node (MN)
  * openPOWERLINK managing node, e.g. Linux
  * B&R POWERLINK managing node
  * other POWERLINK managing node

## Tools {#sect_xilinx_requirements_tools}

### Xilinx ISE
The following tool is necessary to evaluate a Xilinx FPGA-based openPOWERLINK
slave:
* `Xilinx ISE - Embedded Edition` which is called `ISE Design Suite - 14.7 Full
  Product Installation` or `ISE Design Suite - 14.7 Embedded Edition` and can be
  downloaded from: http://www.xilinx.com/support/download/index.htm. The license
  can be acquired from the `Xilinx Licensing Site` and is free for an evaluation
  product (30 days).

### CMake
For building the openPOWERLINK stack and demo applications the Open Source
cross-platform build tool CMake is used (<http://www.cmake.org>). CMake
version V2.8 or higher is required.

For a detailed description of CMake look at the
[cmake section](\ref sect_build_cmake).

### TeraTerm
TeraTerm is a program for printing out text which is transmitted over a serial
interface. It is open source and can be downloaded from: http://ttssh2.sourceforge.jp/

# openPOWERLINK Stack Components {#sect_xilinx_components}
The following section contains a description of the
[openPOWERLINK components](\ref page_components) available on a Microblaze
system.

## Stack Libraries {#sect_xilinx_components_libs}

The openPOWERLINK stack is divided into a user and a kernel part. On the target
Microblaze the following configurations are possible:

- __Direct Link to Application__

  The kernel part is directly linked to the user part and application (complete
  library) into a single executable.

  _Libraries:_
  - `stack/proj/generic/liboplkcn` (liboplkcn.a)

## Demo Applications  {#sect_xilinx_components_apps}

The following demo application are provided for the target Microblaze:

* [demo_cn_embedded](\ref sect_components_demo_cn_embedded)

# Building {#sect_xilinx_build}

For building openPOWERLINK for target Microblaze refer to the
[generic build instructions](\ref page_build) and execute all required build
steps from this section. On the platform Xilinx Microblaze the following build
steps can be carried out:
* [Build the hardware platform](\ref page_build_hardware)
* [Build the openPOWERLINK stack libraries](\ref page_build_stack)
* [Build your application (or a delivered demo application)](\ref page_build_demos)

# Running openPOWERLINK {#sect_xilinx_running}
In order to download the FPGA configuration and the application executable to
the target the following steps need to be executed:

* Open the `ISE Design Suite Command Prompt` and execute the following
  commands:\n

      > cd <openPOWERLINK_directory>\bin\generic\microblaze\[BOARD_NAME]\[DEMO_NAME]
      > make download-bits
      > make download-elf

* Use a terminal program to see the debug output (Only possible if the
  application is compiled in debug mode)
    - Baud rate: `9600`
    - Data Bits: `8`
    - Stop Bits: `1`
    - Parity: `none`
    - Flow control: `none`

# Debugging {#sect_xilinx_debug}
It is possible to debug the user application with the `GNU debugger` by using
the Xilinx Tool `Xilinx Software Development Kit (SDK)`.
For this, it is important that the application project is compiled with the
CMake variable `CMAKE_BUILD_TYPE` set to `Debug`!

CMake has already generated `Eclipse (SDK)` project files for all stack sub
projects which can be imported into the current workspace with:\n
`Import` -> `General` -> `Existing Project into Workspace`

Select the stack root for the folder to import and select all projects needed
for your demo. The following projects should be available:\n
- The hardware platform with the bistream
- The board support package
- A project for your application. e.g: __demo-cn-embedded__.
- The stack library project e.g: __oplkcn__. (Import of this project is
  optional!)
- The driver library for the omethlib. (Import of this project is optional!)

After the project import right lick on your demo project `demo-cn-embedded`
and use:\n
`Run` -> `Debug As` -> `Launch on Hardware` to start the debugger and step
through the code.
