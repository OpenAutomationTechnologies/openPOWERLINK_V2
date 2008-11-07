  (c) SYSTEC electronic GmbH, D-07973 Greiz, August-Bebel-Str. 29
        www.systec-electronic.com
	openPOWERLINK.sourceforge.net


	openPOWERLINK - Demo for Hilscher netX-500
	===========================================


Consists of:
------------

- Kernel module epl.ko


Requirements:
-------------

- Linux kernel version 2.6.23.1 or above
  with the following configuration:
    * netx-eth driver disabled or compiled as module
    * firmware support
- udev with firmware support in Linux-BSP


How to build kernel module:
---------------------------

run make in this directory

$ make KDIR=<path to Linux kernel>



How to run the demo:
--------------------

copy the following files to the target (e.g. via CompactFlash card):
- epl.ko
- EplLoad
- xc0.bin, xc1.bin


load the kernel driver with the Bash script EplLoad

$ ./EplLoad

