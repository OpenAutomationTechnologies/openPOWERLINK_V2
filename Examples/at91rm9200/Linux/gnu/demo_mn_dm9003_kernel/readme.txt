  (c) SYSTEC electronic GmbH, D-07973 Greiz, August-Bebel-Str. 29
        www.systec-electronic.com
	openPOWERLINK.sourceforge.net


	openPOWERLINK - Demo for Atmel AT91RM9200 with Davicom DM9003
	==============================================================


Contents
---------

- Demo for Managing Node as Linux kernel module 'epl.ko'


Requirements
-------------

- Atmel AT91RM9200 with Davicom DM9003 Ethernet controller

- SYSGO ELinOS with Linux kernel version 2.6.22 or above
  or any other Linux BSP with registered platform device for Davicom DM9003
  (with name "dm9sw_st")

- Bootloader U-Boot, that sets up the chip select configuration
  appropriately for the Davicom DM9003

- POWERLINK network as described in main readme.txt


How to build the kernel module
-------------------------------

1. Build appropriate ELinOS project with ELK and build the kernel image

2. Setup ELinOS environment within the ELinOS project directory

   $ . ELINOS.sh

3. Go to the directory of this demo project

   $ cd /projects/openPOWERLINK/Examples/at91rm9200/Linux/gnu/demo_mn_dm9003_kernel

4. Run make in this directory

   $ make



How to run the demo
--------------------

1. Setup the POWERLINK network as described in main readme.txt

2. Copy the following file to the target (e.g. via NFS):
        epl.ko

3. Load the kernel module

   $ insmod epl.ko nodeid=240 cyclelen=5000

