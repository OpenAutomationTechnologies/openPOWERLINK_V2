  (c) SYSTEC electronic GmbH, D-07973 Greiz, August-Bebel-Str. 29
        www.systec-electronic.com
	openPOWERLINK.sourceforge.net


	openPOWERLINK - Demo for Hilscher netX-500
	===========================================


Contents
---------

- Demo for Managing Node as Linux kernel module epl.ko


Requirements
-------------

- Evaluation board Hilscher NXEB 500-HMI

- Linux kernel version 2.6.23.1 or above
  with the following configuration:
    * netx-eth driver disabled or compiled as module
    * firmware support
    * CONFIG_PREEMPT_RT disabled

- udev with firmware support in Linux-BSP
  therefor start 'ptxdist menuconfig' and activate the following items
        PTXdist Base Configuration         --->
            Shell & Console Tools           --->
                [*] udev                  --->
                    [*]   firmware helper

- POWERLINK network as described in main readme.txt


How to build the kernel module
-------------------------------

run make in this directory

$ make KDIR=<path to Linux kernel>



How to run the demo
--------------------

copy the following files to the target (e.g. via CompactFlash card):
- epl.ko
- EplStart
- xc0.bin, xc1.bin -> copied to /usr/local/lib/firmware
  This can be done via the copy_firmware.sh script in Edrv/netx_ethhub/firmware.


load the kernel module with the shell script EplStart

$ ./EplStart

