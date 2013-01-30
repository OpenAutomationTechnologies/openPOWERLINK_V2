  (c) SYSTEC electronic GmbH, D-07973 Greiz, August-Bebel-Str. 29
        www.systec-electronic.com
	openPOWERLINK.sourceforge.net


openPOWERLINK - Kernel module for openPOWERLINK user space applications
=======================================================================


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

Start the stack by pressing the button "Start EPL Stack"
in the Qt application.


Howto generate a new network configuration with openCONFIGURATOR
-----------------------------------------------------------------

1. Download and install the latest openCONFIGURATOR from http://sourceforge.net/projects/openconf

2. Start openCONFIGURATOR

3. Select "Create New Project" and press "Ok"

4. Choose project name and path and press "Ok"

5. Select "Import XDC/XDD" under "MN Configuration"

6. Enter the path name of the following XDD (or browse to it) under the openPOWERLINK main directory
   /ObjDicts/Qt_MN/00000000_openPOWERLINK_demo_MN_PRC.xdd

7. Select "No" under "Auto Generate" (Important!)
    and press "Ok"

8. Select "Advanced View" under menu "View".

9. Add CNs via right mouse button click on item "openPOWERLINK_MN" in "Network Browser".
   Select "Import XDC/XDD" under "CN Configuration" and select the corresponding XDD of your CN.

10. Change the PDO mapping

11. Select "Build Project" under menu "Project"

12. Copy the generated file "mnobd.cdc" from the subdirectory "cdc_xap" to the subdirectory of this
    demo application.

13. Unload any running kernel driver with the Bash script EplUnload

    $ sudo ./EplUnload

14. Run the demo as stated above.
