openPOWERLINK on Linux {#page_platform_linux}
======================

[TOC]

# Introduction {#sect_linux_intro}

This file contains documentation for the openPOWERLINK stack on Linux.

# Requirements {#sect_linux_require}

## POWERLINK Network {#sect_linux_plknet}

- POWERLINK network with controlled nodes (CN)
  * openPOWERLINK controlled nodes, e.g. Altera-based FPGA evaluation boards
  * B&R POWERLINK controlled nodes
  * other POWERLINK controlled nodes

## Network Controller {#sect_linux_controller}

One of the following network controllers is required to run openPOWERLINK on
Linux.

- Network controller card with Intel 82573L (or compatible) 1GBit Ethernet chip
- Network controller card with Realtek RTL8139 Rev C or D chip
- Network controller card with Intel 8255x 100MBit Ethernet chip
- Standard Linux network controller through libpcap library (for user-space stack version)

## Linux Kernel {#sect_linux_kernel}
- Linux kernel version 2.6.23 or later with CONFIG_HIGH_RES_TIMERS enabled
     * this needs ACPI support, maybe you need to append "`highres=on`" or
       "`acpi=force`" to kernel command line for older BIOSes)
     * check `/proc/timer_list` if .`hres_active` is 1

           $ cat /proc/timer_list | grep 'hres_active'

### Real-Time Kernel

For best performance and minimal jitter on the POWERLINK cycle, a real-time
Linux kernel is recommended. The RT-Preempt patch maintained by Ingo Molnar
provides the necessary real-time extensions (https://rt.wiki.kernel.org).

Additional information on the Linux real-time kernel can be found on the
OSADL home page (http://www.osadl.org).

**Tested Kernel Versions**

Whereas openPOWERLINK should run on any kernel specified in: \ref sect_linux_kernel,
it is recommended to use one of the following kernel versions which were
extensively tested with this openPOWERLINK version.

- 2.6.33.7.2-rt30
- 3.12.24-rt38

**Thread Priorities**
If using a real-time kernel, the real-time priorities of the necessary
threads must be adjusted for deterministic POWERLINK behavior.
If you are using the kernel-based stack, the plkload script coming along
with the stack automatically carries out the required priority changes.
For the user space stack, a script setting the priorities is provided in
`tools/linux/set_prio`. It increases the priorities of the high resolution
timer softirq thread (only on 2.6 kernels) and of the Ethernet IRQ thread.

**Kernel 3.x**
The behavior of a 3.X real-time kernel differs to previous kernel version.
Split softirq threads are no longer available, although there is a patch which
implements split softirq locks. If you are using a 3.X real-time kernel you
should ensure that this patch is included.

Additionally, the following steps could be made to improve the real-time
behavior on a _multicore_ processor:

* Ensure that the following configuration options are set for your
  real-time kernel:

      CONFIG_RT_GROUP_SCHED is not set
      CONFIG_RCU_BOOST=y
      CONFIG_RCU_BOOST_PRIO=99

* Isolate the second core of the multi-core processor to be used
  exclusively for openPOWERLINK by specifying the kernel command line parameter
  `isolcpus=1`

* Set the default interrupt affinity to all other cores, e.g. for a
  dual-core system to core 0 by setting the kernel command line parameter
  `irqaffinity=0`

* Disable IRQ balancing by disable irqbalance. This depends on your
  Linux distribution. For example in Ubuntu edit `/etc/default/irqbalance`


## Libraries and Tools {#sect_linux_libs}

### CMake
For building the openPOWERLINK stack and demo applications the Open Source
cross-platform build tool CMake is used (<http://www.cmake.org>). CMake
version V2.8.7 or higher is required.

For a detailed description of CMake look at the
[cmake section](\ref sect_build_cmake).

### libpcap Library

In order to use the user space POWERLINK stack the libpcap library is needed
to access the Ethernet interface.

### QT4 Development Tools

If you want to build the QT demo application the QT4 development tools must
be installed on the system (<http://qt.digia.com/>).

### openCONFIGURATOR

The tool [openCONFIGURATOR](\ref page_openconfig) is needed to generate the
network configuration for your application.


# openPOWERLINK Stack Components {#sect_linux_components}

The following section contains a description of the
[openPOWERLINK components](\ref page_components) available on a Linux system.

## Stack Libraries {#sect_linux_components_libs}

The openPOWERLINK stack is divided into a user and a kernel part. On a Linux
system the following configurations are possible:

- __Direct Link to Application__

  The kernel part is directly linked to the user part and application (complete
  library) into a single executable. The stack uses the libpcap library for
  accessing the Ethernet device.

  _Libraries:_
  - `stack/proj/linux/liboplkmn` (liboplkmn.a)
  - `stack/proj/linux/liboplkcn` (liboplkcn.a)

- __Linux User Space Daemon__

  The application is linked to an application library which contains the
  interface to a Linux user space openPOWERLINK driver. The kernel part is
  compiled as a separate process (daemon) which runs in Linux user space.
  The stack uses the libpcap library for accessing the Ethernet device.

  _Libraries:_
  - `stack/proj/linux/liboplkmnapp-userintf` (libmnapp-userintf.a)
  - `stack/proj/linux/liboplkcnapp-userintf` (libcnapp-userintf.a)

- __Linux Kernel Module__

  The application is linked to an application library which contains
  the interface to a Linux kernel space openPOWERLINK driver.
  The kernel part is compiled as a Linux kernel module. The kernel module
  can be configured to use one of the available openPOWERLINK Ethernet drivers.

  _Libraries:_
  - `stack/proj/linux/liboplkmnapp-kernelintf` (liboplkmnapp-kernelintf.a)
  - `stack/proj/linux/liboplkcnapp-kernelintf` (liboplkcnapp-kernelintf.a)

- __Stack on PCIe__

  The application is linked to an application library which contains
  the interface to a Linux kernel space PCIe interface driver.
  The kernel part of openPOWERLINK runs on an external PCIe device. The PCIe
  interface driver handles the status/control and data exchange between the
  user and kernel layers of the stack.

  _Libraries:_
  - `stack/proj/linux/liboplkmnapp-kernelpcie` (liboplkmnapp-kernelpcie.a)

## Drivers {#sect_linux_components_drivers}

### Linux User Space Daemon using PCAP

The kernel part of the stack is compiled as a separate user space process. It
uses the libpcap library for accessing the network interface and is therefore
totally independent of the used network card and driver.

__NOTE:__ Due to the use of libpcap for accessing the Ethernet device, the
solution cannot reach the performance of the kernel space variant.

The driver is located in: `drivers/linux/drv_daemon_pcap`

### Linux Kernel Module

The openPOWERLINK kernel part may be implemented as Linux kernel module. This
solution provides the best performance, but is limited to the available
openPOWERLINK network card drivers.

The driver is located in: `drivers/linux/drv_kernelmod_edrv`

### Linux Kernel PCIe Interface

The openPOWERLINK kernel part may be executed on an external PCIe
device which handles the time critical sections of the openPOWERLINK
stack. This solution provides the best performance with lowest possible system
resource utilization.

A Linux kernel PCIe interface driver is used as a communication interface
between the openPOWERLINK application library and the openPOWERLINK kernel
stack, running on the PCIe device. The PCIe interface driver handles the
status/control and data exchange between the user and kernel layers of
the stack.

The driver is located in: `drivers/linux/drv_kernelmod_pcie`

## Demo Applications

The following demo application are provided on Linux:

* [demo_mn_console](\ref sect_components_demo_mn_console)
* [demo_cn_console](\ref sect_components_demo_cn_console)
* [demo_mn_qt](\ref sect_components_demo_mn_qt)

## Tools

There are some shell scripts used for loading POWERLINK modules, setting
thread priorities etc.

These tools are located in: `tools/linux`

# Building {#sect_linux_build}

For building openPOWERLINK on Linux refer to the
[generic build instructions](\ref page_build).

# Running openPOWERLINK {#sect_linux_running}

## Starting the Kernel Module

To start the POWERLINK kernel modules, the scripts `plkload` and `plkunload`
are used. The scripts will be installed in the installation directory.
Additionally to inserting the kernel module, the plkload script adjusts
priorities and unbinds the network device from the standard driver. This allows
the exclusive use of the Ethernet card by openPOWERLINK.

It is recommended to use this script to start openPOWERLINK!

For example:
Start the kernel stack using the Intel 82573 network controller:

    > cd <kernel_module_installation_dir>
    > sudo ./plkload oplk82573mn.ko

To unload the kernel module:

    > cd <kernel_module_installation_dir>
    > sudo ./plkunload oplk82573mn.ko

## Starting the User Space Daemon

If the stack is configured to use the Linux user space daemon, you must start it
before starting your application. The user space daemon is started by the
following command:

    > cd <userspace_daemon_installation_dir>
    > sudo ./oplkmnd_pcap

## Starting the Demo Application

If the demo application is configured to use a separately compiled kernel stack
you have to ensure that the kernel stack daemon (kernel driver module) is
running before you start your application. If the demo application is linked
with the complete openPOWERLINK stack, you can directly start it:

    > cd <demo_installation_dir>
    > sudo ./demo_mn_qt

## Adjusting Priority of Ethernet Thread

If you are using a PCAP Ethernet driver on a real-time kernel you should adjust
thread priorities using the delivered script `set_prio` before starting the
application. To be able to increase the priority of the right Ethernet interrupt
thread, you have to specify the used Ethernet interface. For example:

    > cd bin
    > sudo ./set_prio eth1

# Troubleshooting {#sect_linux_trouble}

## Linux User Space Stack

- The user space based application does not find a network interface

  Be sure that the pcap library is installed and you are running the demo
  as root.

## Linux Kernel Space Stack

- Linux kernel space: Check the kernel log

      $ dmesg

