  (c) SYSTEC electronic GmbH, D-07973 Greiz, August-Bebel-Str. 29
        www.systec-electronic.com
	openPOWERLINK.sourceforge.net


	openPOWERLINK - Qt Demo for RTL8139 network chips
	==================================================


Consists of:
------------

- Kernel module epl.ko
- User mode application demo_userapp_qt located one directory upward.


How to build kernel module and user mode application:
-----------------------------------------------------

run make in this directory

$ make


How to run the demo:
--------------------

load the kernel driver with the Bash script EplLoad

$ sudo ./EplLoad

start the Qt application

$ ../demo_userapp_qt/demo_userapp_qt

Starting the stack by pressing the button "Start EPL Stack"
in the Qt application.
