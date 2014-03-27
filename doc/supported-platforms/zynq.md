openPOWERLINK on Xilinx Zynq SoC {#page_platform_zynq}
==================================

[TOC]

# Introduction {#sect_zynq_intro}

This file contains documentation for the openPOWERLINK stack on Xilinx
Zynq SoC. Currently stack only provides components for using Linux based demo
on Zynq.

## Contents {#sect_zynq_intro_contents}

- Edrv module for Gigabit Ethernet controller on Xilinx ZC702 development kit. 
- HighRes module for Triple timer counter Zynq 7000 series SoCs.

# Requirements {#sect_zynq_requirements}

## Development Boards {#sect_zynq_requirements_boards}

- Xilinx ZC702 Development Kit

## POWERLINK network {#sect_zynq_requirements_network}

- POWERLINK network with a control node (CN)
  * openPOWERLINK control node, e.g. Linux
  * B&R POWERLINK control node

- POWERLINK network with a managing node (MN)
  * openPOWERLINK managing node, e.g. Linux
  * B&R POWERLINK managing node

## Tools {#sect_zynq_requirements_tools}

### Xilinx ISE
The following tool is necessary to evaluate a Xilinx Zynq based openPOWERLINK solution:

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

### libpcap Library

In order to use the user-space POWERLINK stack the libpcap library is needed
to access the Ethernet interface.

The libpcap library has to cross-compiled to use it with Zynq SoC demo.

### Serial Communication software

A serial communication software is a program for printing out text which is 
transmitted over a serial interface and push input through serial interface.

e.g. : Minicom: It is available for Debian and Ubuntu users through apt-get 
interface through following command:

    > sudo apt-get install minicom

### Linux Kernel

Linux kernel port for Zynq platform is needed to run the Linux based demo of openPOWERLINK.
The Linux ports for Zynq platform are available on github and can be downloaded or 
cloned from: https://github.com/Xilinx/linux-xlnx.

Linux kernel version 3.10 tagged as xilinx-v14.7 on github was used for testing at 
the time of writing this document.

The steps to cross-compile Linux for Zynq and information about availability of other 
libraries for Zynq is available here : http://www.wiki.xilinx.com/Zynq+Linux.

# openPOWERLINK Stack Components {#sect_zynq_components}

The following section contains a description of the
[openPOWERLINK components](\ref page_components) available on a Zynq.

## Stack Libraries {#sect_zynq_components_libs}

The openPOWERLINK stack is divided into a user and a kernel part.The following
libraries are available for Zynq SoC:

* [Linux Stack Libraries](\ref sect_linux_components_libs)

## Demo Applications  {#sect_zynq_components_apps}

The following demo application are supported on Zynq:

* [demo_mn_console](\ref sect_components_demo_mn_console)
* [demo_cn_console](\ref sect_components_demo_cn_console)

## Drivers

The following drivers are available for Zynq SoC:

* [Linux Drivers] (\ref sect_linux_components_drivers)

# Building {#sect_zynq_build}

For building openPOWERLINK for Zynq refer to the
[generic build instructions](\ref page_build) and execute all required build
steps from this section. On the platform Xilinx Zynq the following build
steps can be carried out:

* [Build the openPOWERLINK stack libraries](\ref page_build_stack)
* [Build your application (or a delivered demo application)](\ref sect_build_demos_build_zynq)

# Running openPOWERLINK {#sect_zynq_running}

In order to run the openPOWERLINK demo applications on Zynq follow the following steps:

* Prepare the SDCARD for boot up with Linux kernel image and other components copy the 
  stack binaries to SDCARD.
* Connect host PC to Zynq board using the serial interface on board.
* Use a terminal program to connect to the Linux console on Zynq.
* Refer [running openPOWERLINK on Linux] (\ref sect_linux_running) to run application.
