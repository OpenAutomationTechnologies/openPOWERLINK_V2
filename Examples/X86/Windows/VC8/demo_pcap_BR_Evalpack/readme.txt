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

- Microsoft Visual C 2005

- Experiences with this development environment are required

- POWERLINK network with one of the following nodes:

    Node-ID 0x20:   B&R X20 POWERLINK Controller with the following modules (in this order)
                    X20BC0083 X20 Bus Controller POWERLINK
                    X20PS9400 24 VDC power supply module for BC
                    X20DO2322 2 Outputs 24 VDC / 0.5 A
                    X20DI2371 2 Digital Inputs 24 VDC, Sink, IEC 61131-2, Type 1

    Node-ID 0x6E:   B&R X20 POWERLINK Controller with the following modules (in this order)
                    X20BC0083 X20 Bus Controller POWERLINK
                    X20PS9400 24 VDC power supply module for BC
                    X20DO2322 2 Outputs 24 VDC / 0.5 A
                    X20DI2371 2 Digital Inputs 24 VDC, Sink, IEC 61131-2, Type 1


Howto generate a new network configuration with openCONFIGURATOR
-----------------------------------------------------------------

1. Download and install the latest openCONFIGURATOR from http://sourceforge.net/projects/openconf

2. Start openCONFIGURATOR

3. Select "Create New Project" and press "Ok"

4. Choose project name and path and press "Ok"

5. Select "Import XDC/XDD" under "MN Configuration"

6. Enter the path name of the following XDD (or browse to it) under the openPOWERLINK main directory
   /ObjDicts/00000000_openPOWERLINK_demo_MN.xdd
   or choose the following, if you want to use the PollResponse Chaining feature
   /ObjDicts/00000000_openPOWERLINK_demo_MN_PRC.xdd

7. Select "No" under "Auto Generate" (Important!)
    and press "Ok"
    
8. Select "Advanced View" under menu "View".

9. Add CNs via right mouse button click on item "openPOWERLINK_MN" in "Network Browser".
   Select "Import XDC/XDD" under "CN Configuration" and select the corresponding XDD of your CN.

10. Change the PDO mapping

11. Select "Build Project" under menu "Project"

12. Copy the generated file "mnobd.cdc" from the subdirectory "cdc_xap" to the subdirectory of this
    demo application

