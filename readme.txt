  (c) SYSTEC electronic GmbH, D-07973 Greiz, August-Bebel-Str. 29
        www.systec-electronic.com
	openPOWERLINK.sourceforge.net


	openPOWERLINK - Quick Start Guide
	==================================


1. Documentation
-----------------

* The documentation of the openPOWERLINK protocol stack can be found in the
  subdirectory "Documenation".

* Further documentation can be downloaded from

    http://www.systec-electronic.com/html/index.pl/en_download_OpenPOWERLINK

    It contains an introduction and a reference manual. A free registration
    is required for downloading.

* The openPOWERLINK LiveCD with a ready-to-run Managing Node reference application
  is available as pre-configured ISO image from

    http://www.systec-electronic.com/openpowerlink_livecd

* Update Guide for necessary changes to your application project, if you update
  from a previous version of openPOWERLINK, can be found in file update.txt.

* License: Please refer to the file "license.txt" for information about
  the license of the source code.


2. Generic Requirements for all demo applications
--------------------------------------------------

- POWERLINK network:
    * one or more POWERLINK I/O devices according device profile CiA-401
      (i.e. Controlled Nodes) with the following PDO mapping:

    RPDO (PollRequest from MN): length = 1 or 2 Bytes
	                            containing the values for the digital outputs
	                            PDO version = 0
	TPDO (PollResponse): length = min. 1 Byte
	                     containing the values from the digital inputs
	                     PDO version = 0

    * Node-IDs of the Controlled Nodes (CN): 1, 32 or 110

    * CAT5 cables to connect the POWERLINK devices with the demo application

- openPOWERLINK demo application with node-ID 240/0xF0.
  When the demo application runs as MN (node-ID 240/0xF0) it drives a running light
  on the CNs. Otherwise it just behaves as CiA-401 I/O device.


3. Available demo applications
-------------------------------

X86 PC with Linux or Windows operating system

  * MN demo application using a console based interface
    = Examples\X86\Generic\demo_process_image_console
  * MN demo application using a Qt based interface
    = Examples\X86\Generic\demo_process_image_qt

  These demo applications can be built for both Linux and Windows.
  CMake is used as a cross-platform build system.

  On Linux, these demo applications can be configured to use either
  a pcap based stack that is located in user space, or to use a 
  stack that is located in kernel space.
  Detailed documentation for Linux can be found in Documentation/linux-x86.txt

  Supported Versions: 2.6.23 or later
  Supported build environments: GCC
  Requirements:   libpcap  http://www.tcpdump.org
                  CMake    http://www.cmake.org
                  Qt       http://qt.nokia.com/

  On Windows, these demo applications only support a pcap based stack in
  use space.

  Supported Versions: Windows 2000, Xp, Vista, 7
  Supported build environments: Microsoft Visual Studio 2005, 2008, 2010
  Requirements:   WinPcap  http://www.winpcap.org
                  CMake    http://www.cmake.org
                  Qt       http://qt.nokia.com/
  
Freescale ColdFire MCF5484 (SYSTEC Development Board for ECUcore-5484)
with Linux operating system (see section 3.2 for requirements)
  * simple CN demo which controls the LEDs and reads the pushbuttons on the devboard:
    = Examples\PLCcore-CF54\Linux\gnu\demo_cn_kernel
  * simple CN and MN demo which controls the LEDs and reads the pushbuttons on the devboard
    and drives a running light on other CNs if running as MN (node-ID 240/0xF0):
    = Examples\PLCcore-CF54\Linux\gnu\demo_mn_kernel

Hilscher netX-500 (Evaluation board Hilscher NXEB 500-HMI)
  = GPL-Addon Package: Examples\netx500\Linux\gnu\demo_mn_kernel

Atmel AT91RM9200 with Davicom DM9003 under Linux
  = Examples\at91rm9200\Linux\gnu\demo_mn_dm9003_kernel

Altera Cyclone III on EBV DBC3C40 Development Board or SYS TEC ECUcore-EP3C Development Board
with Nios II Soft-CPU and openMAC
  * CN demo which controls the LEDs and reads the pushbuttons on the devboard:
    = Examples\altera_nios2\no_os\gnu\demo_cn_3r1tpdo

3.3. Requirements for X86 Linux demos
-------------------------------------

Detailed documentation is located in Documentation/linux-x86.txt


3.3. Requirements for ColdFire MCF5484 demo
--------------------------------------------

- Linux-BSP and toolchain for ColdFire MCF5484
- SYSTEC Developmentboard for ECUcore-5484
- Host PC with Linux


3.4. Steps to build and execute the demo application for the MCF5484
--------------------------------------------------------------------

1.  Setup build environment on the host computer
    (e.g. install Linux-BSP and toolchain for ColdFire MCF5484)

2.  Compile the sample application,
    e.g. for ColdFire MCF5484 with Linux execute the following commands
            $ cd Examples/PLCcore-CF54/Linux/gnu/demo_mn_kernel
            $ make

3.  Copy the built sample application (i.e. the Linux kernel object epl.ko) to
    the target (e.g. via FTP or NFS) and run it.
            $ insmod epl.ko
    With an additional parameter 'nodeid' the node-ID can be set manually.
    It overwrites any hardware settings.
            $ insmod epl.ko nodeid=240

4.  Now you may modify the sources to your needs and restart from 2.
    (e.g. change the cycle length and the network configuration in demo_main.c)



4. It does not work
--------------------

1.  Check the kernel log
            $ dmesg

2.  Make a trace with Wireshark on another PC that is connected to the
    POWERLINK network (www.wireshark.org)

3.  Study the output of
            $ cat /proc/epl

4.  Try to reset the NMT state machine with
            $ echo > /proc/epl

    (Hint: /proc/epl executes the NMT events defined in enum tEplNmtEvent in
           file Include/EplNmt.h like
            $ echo 0x13 > /proc/epl
           for NMT Reset Configuration)

5.  If TCP/IP communication over the POWERLINK network does not work
    check the configuration of the virtual network interface and the routing
            $ ifconfig epl
            $ netstat -r

