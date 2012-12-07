openPOWERLINK on other Platforms {#other-platforms}
================================

## openPOWERLINK on QNX

TBD

## openPOWERLINK on Windows

### Windows Requirements

- Supported Versions: Windows 2000, Xp, Vista, 7
- Supported build environments: Microsoft Visual Studio 2005, 2008, 2010
- WinPcap  <http://www.winpcap.org>
- CMake    <http://www.cmake.org>
- Qt       <http://qt.nokia.com/>

## openPOWERLINK on Freescale ColdFire MCF5484

### Requirements for ColdFire MCF5484 demo

- Linux-BSP and toolchain for ColdFire MCF5484
- SYSTEC Developmentboard for ECUcore-5484
- Host PC with Linux

#### Steps to build and execute the demo application for the MCF5484

1. Setup build environment on the host computer
   (e.g. install Linux-BSP and toolchain for ColdFire MCF5484)
2  Compile the sample application,
   e.g. for ColdFire MCF5484 with Linux execute the following commands\n
   `$ cd Examples/PLCcore-CF54/Linux/gnu/demo_mn_kernel`\n
   `$ make`
3. Copy the built sample application (i.e. the Linux kernel object epl.ko) to
   the target (e.g. via FTP or NFS) and run it.\n
   `$ insmod epl.ko`\n
   With an additional parameter 'nodeid' the node-ID can be set manually.
   It overwrites any hardware settings.\n
   `$ insmod epl.ko nodeid=240`\n
4. Now you may modify the sources to your needs and restart from 2.
   (e.g. change the cycle length and the network configuration in demo_main.c)
   
## openPOWERLINK on Hilscher netX-500

TBD

## openPOWERLINK on Atmel AT91RM9200 with Davicom DM9003 under Linux

TBD