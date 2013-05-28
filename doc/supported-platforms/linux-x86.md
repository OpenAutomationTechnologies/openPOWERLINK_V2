openPOWERLINK on Linux X86 {#linux-x86}
==========================

## Introduction

This file contains documentation for the openPOWERLINK stack on Linux x86.

## Requirements

### POWERLINK network

- POWERLINK network with controlled nodes (CN)
  * openPOWERLINK controlled nodes, e.g. Altera-based FPGA evaluation boards
  * B&R POWERLINK controlled nodes
  * other POWERLINK controlled nodes

### Network Controller
One of the following network controllers is required to run openPOWERLINK on
Linux x86.

- Network controller card with Intel 82573L (or compatible) 1GBit Ethernet chip  
  For example:
  - B&R APC 810 Industrial PC on-board network chip (tested)

- Network controller card with Realtek RTL8139 Rev C or D chip  
    For example:
    - PCI network cards:
      * Zyxel FN312 (tested)
      * Netgear FA311 v2 Rev-D1 (tested)
      * D-Link DFE-528TX
      * LevelOne FNC-0109TX
      * Typhoon Speednet Card 10/100 PCI (P/N 70035)
      * Longshine LCS-8038TX-R7
    - Cardbus network cards (PCMCIA):
      * Longshine LCS-8539TXR
      * Micronet SP160T V3
      * Micronet SP160TA V3

- Network controller card with Intel 8255x 100MBit Ethernet chip  
    For example:
    - B&R APC 620 Industrial PC on-board network chip (tested)
    - Intel Pro 100 82557/8/9/1 Ethernet Interface card (tested)

- Standard Linux network controller through libpcap library  
    (for user-space stack version)

### Linux Kernel
- Linux kernel version 2.6.23 or later with CONFIG_HIGH_RES_TIMERS enabled
     * this needs ACPI support, maybe you need to append "`highres=on`" or
       "`acpi=force`" to kernel command line for older BIOSes)
     * check `/proc/timer_list` if .`hres_active` is 1

           $ cat /proc/timer_list | grep 'hres_active'

#### Real-Time Kernel

For best performance and minimal jitter on the POWERLINK cycle a real-time
Linux kernel is recommended. The RT-Preempt patch maintaind by Ingo Molnar
provides the necessary real-time extensions (https://rt.wiki.kernel.org).

Additional information on the Linux realtime kernel can be found on the
OSADL home page (http://www.osadl.org).

**Thread Priorities**  
If using a real-time kernel, the real-time priorities of the necessary
threads must be adjusted for deterministic POWERLINK behaviour.
If you are using the kernel based stack, the plkload script coming along
with the stack automatically carries out the required priority changes.
For the userspace stack, a script setting the prioritiess provided in
tools/Linux/set_prio. It increases the priorities of the high resolution
timer softirq thread (only on 2.6 kernels) and of the Ethernet IRQ thread.

**Kernel 3.x**  
The behaviour of a 3.X real-time kernel changed. Split softirq threads
are no longer available but there is a patch which implements split softirq
locks. If you are using a 3.X real-time kernel you should ensure that this
patch is included.

For example: A current Linux version at writing of this document which
includes the patch is v3.6.11.4-rt36.

Additionally, the following steps could be made to improve the real-time
behaviour on a _multicore_ processor:

* Ensure that the following configuration options are set for your
  real-time kernel:

      CONFIG_RT_GROUP_SCHED is not set  
      CONFIG_RCU_BOOST=y  
      CONFIG_RCU_BOOST_PRIO=99

* Isolate the second core of the multi-core processor to be used
  exclusively for openPOWERLINK by specifying the kernel-commandline parameter
  `isolcpus=1`

* Set the default interrupt affinity to all other cores, e.g. for a
  dual-core system to core 0 by setting the kernel-commandline parameter
  `irqaffinity=0`

* Disable IRQ balancing by disable irqbalance. This depends on your
  Linux distribution. For example in Ubuntu edit `/etc/default/irqbalance`

### Libraries and Tools

#### CMake
For building the openPOWERLINK stack and demo applications the Open Source
cross-platform build tool CMake is used ([http://www.cmake.org]). CMake
version V2.8 or higher is required.

For a detailed description of the cmake options look at the [cmake documentation](\ref cmake).

#### libpcap library
In order to use the userspace POWERLINK stack the libpcap library is needed
to access the Ethernet interface.

#### QT4 development tools
If you want to build the QT demo application the QT4 development tools must
be installed on the system.

#### openCONFIGURATOR
For configuration of your POWERLINK network the Open Source configuration
tool openCONFIGURATOR should be used. The tool is available as SourceForge
project. [http://sourceforge.net/projects/openconf/](http://sourceforge.net/projects/openconf/)

The openCONFIGURATOR projects used by the demo examples are found in the
directory: *examples/openCONFIGURATOR_projects*.

openCONFIGURATOR creates two file which are used by the openPOWERLINK stack
and application:

* `xap.h`  
  The header file contains the structure definition for your
  process image. It depends on the available data field of the
  CNs used in your configuration.

* `mnobd.cdc`  
  This file is used to configure the MN stack. It includes all
  configuration data of the CNs and the network mapping
  information. CN configuration is handled by the configuration
  manager (CFM) module of the MN.


## openPOWERLINK Stack

The openPOWERLINK stack is divided in a user- and a kernel part. Whereas in
previous versions the whole stack runs in the same domain, the current stack
could run in different domains. On a Linux x86 system the following configurations
are possible:

- Direct Link to Application  
  The kernel part is directly linked to the user part and application into a
  single executable. The stack uses the libpcap library for accessing the
  ethernet device.

- Linux Userspace Daemon  
  The kernel part is compiled as a separate process (daemon) which runs in
  userspace. The stack uses the libpcap library for accessing the ethernet
  device.

- Linux Kernel Module  
  The kernel part is compiled as a Linux kernel module. The kernel module
  could be configured to use one of the available openPOWERLINK ethernet
  drivers.

### openPOWERLINK kernel stack

#### Direct Link to Application

If the openPOWERLINK stack is configured to be directly linked to the application
there is no need of a separate stack daemon. The whole stack is compiled into
the library libpowerlink.a. This library has to be linked by your application.

#### Linux Userspace Daemon

The kernel part of the stack is compiled as a separate userspace process. It
uses the libpcap library for accessing the network interface and is therefore
totally independant of the used network card and driver. Due to the usage of
libpcap for accessing the ethernet device it cannot reach the performance
of the kernel space stack!

The Linux userspace daemon is located in: `stack/make/driver/linux/powerlink_userspace_daemon`

#### Linux Kernel Module

The openPOWERLINK stack may be implemented as Linux kernel module. This
solution provides the best performance, but is limited to the available
openPOWERLINK network drivers.

The linux kernel module is located in: `stack/make/driver/linux/powerlink_kernel_module`

### openPOWERLINK stack library

#### openPOWERLINK stack library - complete stack

The openPOWERLINK stack library contains the whole openPOWERLINK stack. If you
want to create an openPOWERLINK application which contains the stack in a
single executable you only need to link your application to the openPOWERLINK
stack library libpowerlink.a.

In this case you are using the libpcap library to access the ethernet device.

The openPOWERLINK stack library is located in: `stack/make/lib/libpowerlink`


#### openPOWERLINK user part library - user part of stack

If you are using a separated kernel stack (either user space daemon or kernel
module) you need to link your application to the user part stack library
(libpowerlink_user.a). Depending on the used kernel stack daemon it uses libpcap
or a special openPOWERLINK ethernet driver to access the ethernet interface.

The openPOWERLINK user part stack library is located in: `stack/make/lib/libpowerlink_user`

## Tools

There are some shellscripts used for loading POWERLINK modules, setting
thread priorities etc.

These tools are located in: `tools/linux`


## Demo applications

There are several demo applications available. The POWERLINK demo applications
are able to visualize the digital inputs of POWERLINK controlled nodes and are
driving a running light on the CNs digital outputs.

### QT MN demo
  
This QT demo implements a POWERLINK managing node (MN) using the configuration
manager (CFM) to initialize the controlled nodes. It uses a network configuration
created with the openCONFIGURATOR tool.
  
It is found in: `examples/demo_mn_qt`

### Console MN demo

This demo also implements a POWERLINK MN with CFM. It is implemented as
console application and is intended to machines where no graphical user
interface is available.

It is located in: `examples/demo_mn_console`

### Console CN demo
  
This demo implements a POWERLINK CN digital I/O node according to CiA401
profile. It is implemented as console application.

It is located in: `examples/demo_cn_console`


## Building

### CMake
For building openPOWERLINK on Linux the build utility [CMake](\ref cmake)
is used. 

### Build Instructions

Follow the steps below to build the stack and demo applications:

* Creating the build directory
  To separate the build files from the source code, a separate directory shall
  be created, e.g. a directory called 'build' in the openPOWERLINK root:

      > cd openPOWERLINK
      > mkdir build

* Entering the build directory and executing cmake
  CMake has to be executed in the build directory with the path to the
  source directory as parameter.

      > cd build
      > cmake-gui .. (cmake -i .., ccmake ..)

  If using the cmake-gui, the build options are selected in the GUI. After
  setting the desired options press _Configure_. New options might appear
  according to the current selection. New options will be marked in _red_.
  Further configuration settings may be changed and accepted by pressing
  _Configure_ again. If there are no red-marked options, _Generate_ writes
  the build files (Unix Makefiles).
  
* Building
  No you can build all necessary software modules by calling
  
      > make

* Installation
  To install the compiled files to a single directory, type:

      > make install

  The target files will be installed in the configured installation
  directory (Default:bin). There, the stack and demos can be started.


## Running POWERLINK

### Starting the kernel module

To start the POWERLINK kernel modules, the scripts plkload and plkunload are
used. The scripts will be installed in the installation directory. Additionally
to inserting the kernel module the plkload script adjust priorities and unbinds
the network device from the standard driver. This allows the usage of the Ethernet
card by openPOWERLINK.

It is recommended to use this script to start openPOWERLINK!

For example:
Start the kernel stack using the Intel 82573 network controller:

    > cd bin
    > sudo ./plkload powerlink82573.ko

To unload the kernel module:
    > cd bin
    > sudo ./plkunload powerlink82573.ko

### Starting the userspace daemon

If the stack is configured to use the Linux userspace daemon, you must start it
before starting your application. The userspace daemon is started by the following
command:

    > cd bin
    > sudo ./powerlink_mn_daemon

### Starting the demo application

#### Demo uses separate kernel stack daemon

If the demo application is configured to use a seperately compiled kernel stack
you have to ensure that the kernel stack daemon is running before you start your
application. Then you could start it by:

    > cd bin
    > sudo ./demo_mn_qt

#### Demo is directly linked with the kernel stack

If the demo application is  linked with the complete openPOWERLINK stack, you
could directly start it:

    > cd bin
    > sudo ./demo_mn_qt

If you are using a real-time kernel you should adjust thread priorities using
the delivered script _set_prio_ before starting the application. To be able to
increase the priority of the right ethernet interrupt thread, you have to
specifcy the used ethernet interface. For example:

    > cd bin
    > sudo ./set_prio eth1


## Troubleshooting

### Linux userspace stack

- The userspace based application doesn't find a network interface

  Be sure that the pcap library is installed and you are running the demo
  as root.

### Linux kernelspace stack

- Linux kernel space: Check the kernel log
      $ dmesg


