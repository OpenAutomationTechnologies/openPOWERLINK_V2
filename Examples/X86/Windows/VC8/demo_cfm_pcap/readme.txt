  (c) SYSTEC electronic GmbH, D-07973 Greiz, August-Bebel-Str. 29
        www.systec-electronic.com
	openPOWERLINK.sourceforge.net


	openPOWERLINK - Demo with CFM for Windows with Pcap driver
	===========================================================


Contents
---------

- Demo for Managing Node with CFM under Windows with Pcap driver.


Requirements
-------------

- Windows XP or higher

- WinPcap driver installed

- Microsoft Visual C 2005 or newer

- Experiences with this development environment are required

- POWERLINK network with one of the following nodes:

    Node-ID 0x01:   B&R X20 POWERLINK EvalpackA

    Node-ID 0x02:   B&R X20 POWERLINK EvalpackB

    Node-ID 0x03:   B&R X20 POWERLINK EvalpackC


Howto generate a new network configuration with openCONFIGURATOR
-----------------------------------------------------------------

1. Download and install the latest openCONFIGURATOR from http://sourceforge.net/projects/openconf

2. Start openCONFIGURATOR

3. Select "Create New Project" and press "Ok"

4. Choose project name and path and press "Ok"

5. Select "Import XDC/XDD" under "MN Configuration"

6. Enter the path name of the following XDD (or browse to it) under the openPOWERLINK main directory
   /ObjDicts/CiA302-4_MN/00000000_openPOWERLINK_MN.xdd

7. Select "Yes" under "Auto Generate" (Important!)
    and press "Ok"
    
8. Select "Advanced View" under menu "View".

9. Add CNs via right mouse button click on item "openPOWERLINK_MN" in "Network Browser".
   Select "Import XDC/XDD" under "CN Configuration" and select the corresponding XDD of your CN.

10. Select "Build Project" under menu "Project"

11. Copy the generated files "mnobd.cdc" and "xap.h" from the subdirectory "cdc_xap"
    to the subdirectory of this demo application.

12. Adjust the function AppCbSync() in demo_main.c according to your network configuration.
