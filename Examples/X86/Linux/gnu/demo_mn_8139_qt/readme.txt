  (c) SYSTEC electronic GmbH, D-07973 Greiz, August-Bebel-Str. 29
        www.systec-electronic.com
	openPOWERLINK.sourceforge.net


	openPOWERLINK - Qt Demo for RTL8139 network chips
	==================================================


Consists of:
------------

- Kernel module epl.ko
- User mode application EplUserApp


How to build kernel module:
---------------------------

run make in this directory

$ make


How to build user mode application:
-----------------------------------

create symlink for include directory

$ ln -s ../../../../../../../Include EplUserApp/include/epl

create Makefile with qmake

$ cd EplUserApp
$ qmake-qt4 -project -unix
$ qmake-qt4 -unix

finally, run make

$ make



How to run the demo:
--------------------

load the kernel driver with the Bash script EplLoad

$ sudo ./EplLoad

start the Qt application

$ ./EplUserApp/EplUserApp

optionally, configure the IP address of the virtual Ethernet driver
for TCP/IP communication in asynchronous phase.
This has to be done after starting the stack by pressing the button
"Start EPL Stack" in the Qt application. Because the virtual Ethernet
interface "epl" exists only, when the stack is running.

$ sudo ifconfig epl 192.168.100.240

