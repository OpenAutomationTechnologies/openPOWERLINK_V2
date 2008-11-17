  (c) SYSTEC electronic GmbH, D-07973 Greiz, August-Bebel-Str. 29
        www.systec-electronic.com
	openPOWERLINK.sourceforge.net


	openPOWERLINK - Quick Start Guide
	==================================


  -------------------------------------------------------------------------

  License:

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    1. Redistributions of source code must retain the above copyright
       notice, this list of conditions and the following disclaimer.

    2. Redistributions in binary form must reproduce the above copyright
       notice, this list of conditions and the following disclaimer in the
       documentation and/or other materials provided with the distribution.

    3. Neither the name of SYSTEC electronic GmbH nor the names of its
       contributors may be used to endorse or promote products derived
       from this software without prior written permission. For written
       permission, please contact info@systec-electronic.com.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    POSSIBILITY OF SUCH DAMAGE.

    Serverability Clause:

        If a provision of this License is or becomes illegal, invalid or
        unenforceable in any jurisdiction, that shall not affect:
        1. the validity or enforceability in that jurisdiction of any other
           provision of this License; or
        2. the validity or enforceability in other jurisdictions of that or
           any other provision of this License.

  -------------------------------------------------------------------------


Documentation
-------------

The documentation of the openPOWERLINK protocol stack can be downloaded from

    http://www.systec-electronic.com/html/index.pl/en_download_OpenPOWERLINK

    It contains an introduction and a reference manual. A registration is
    required for downloading.


Generic Requirements for all demo applications
-----------------------------------------------

- POWERLINK network:
    * one or more POWERLINK I/O devices according CiA-401
      (i.e. Controlled Nodes) with the following PDO mapping:

    RPDO (PollRequest from MN): 1 Byte length
	                            containing the values for the digital outputs
	TPDO (PollResponse): 1 Byte length
	                     containing the values from the digital inputs

    * Node-IDs of the Controlled Nodes (CN): 1, 32 or 110

    * CAT5 cables to connect the POWERLINK devices with the demo application

- OpenPOWERLINK demo application with node-ID 240.
  When the demo application runs as MN (node-ID 240) it drives a run light
  on the CNs. Otherwise it just behaves as CiA-401 I/O device.


Requirements for X86 Demo
--------------------------

- Linux kernel version 2.6.23 or later
  with CONFIG_HIGH_RES_TIMERS enabled
    * this needs ACPI support, maybe you need to append "highres=on" or
      "acpi=force" to kernel command line for older BIOSes)
    * check /proc/timer_list if .hres_active is 1
      $ cat /proc/timer_list | grep 'hres_active'

- Network controller card with Realtek RTL8139 Rev C or D chip onboard
  These are for example
    * Zyxel FN312
    * Netgear FA311 v2 Rev-D1
    * D-Link DFE-528TX



Requirements for ColdFire MCF5484 demo
---------------------------------------

- Linux-BSP and toolchain for ColdFire MCF5484
- SYSTEC Developmentboard for ECUcore-5484
- Host PC with Linux


Steps to build and execute the demo application
------------------------------------------------

1.  Setup build environment on the host computer
    (e.g. install Linux-BSP and toolchain for ColdFire MCF5484
     or install Linux kernel sources for X86)

2.  Compile the sample application,
    e.g. for ColdFire MCF5484 with Linux execute the following commands
            $ cd Examples/PLCcore-CF54/Linux/gnu/demo_mn_kernel
            $ make

    for X86 with RTL8139 network controller try the following
	        $ cd Examples/X86/Linux/gnu/demo_mn_8139_kernel
	        $ make

3.  Unload an existing 8139 driver under Linux (if necessary):
	        $ rmmod 8139too.ko

    Unload the running application before:
            $ rmmod epl.ko

4.  Copy the built sample application (i.e. the Linux kernel object epl.ko) to
    the target (e.g. via FTP or NFS) and run it.
            $ insmod epl.ko
    With an additional parameter 'nodeid' the node-ID can be set manually.
    It overwrites any hardware settings.
            $ insmod epl.ko nodeid=240

5.  Now you may modify the sources to your needs and restart from 2.
    (e.g. change the cycle length and the network configuration in demo_main.c)



It does not work
-----------------

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

