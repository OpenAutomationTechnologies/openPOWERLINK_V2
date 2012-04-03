  (c) SYSTEC electronic GmbH, D-07973 Greiz, August-Bebel-Str. 29
        www.systec-electronic.com
    openPOWERLINK.sourceforge.net


    openPOWERLINK - Demo for EBV DBC3C40 and TERASIC DE2-115
    ===========================================================


Contents
---------

- Demo for Controlled Node as standalone program on Nios II CPU.

Performance Data
-----------------

- Minimum cycle length: 400 µs
- PReq-PRes Latency: 1 µs
- Process data: 4 bytes input and 4 bytes output
- There is 1 RPDOs and 1 TPDO available. Cross-traffic is disabled.


Requirements
-------------

- Development Board EBV DBC3C40 (Mercury Board)
  or TERASIC DE2-115 (INK board)

- Altera Quartus II v10.1 SP1 or newer

- Altera Nios II Embedded Design Suite v10.1 or newer

- Experiences with this development environment are required

- POWERLINK network as described in main readme.txt
  or alternatively a POWERLINK network with Configuration Manager.
  The corresponding XDD for this node can be found in the subdirectory
  ObjDicts\CiA401_CN of the openPOWERLINK main directory.


How to build the binary
------------------------

1. Open "Nios II Command Shell"

2. Go to this subdirectory of openPOWERLINK:

    $ cd Examples\altera_nios2\no_os\gnu\demo_directIO

3. Run script create-this-app to create the Makefile for
   the demo application and the necessary BSP.
   (This step is only necessary once)

    $ ./create-this-app

4. Run make to build the ELF file after changing the sources.
   (The script create-this-app will do this automatically)

    $ make

5. Rebuild the Makefile for the demo application and the
   BSP, if the Nios II Design was changed inside the SOPC Builder.

    $ ./create-this-app --rebuild


How to run the demo
--------------------

1. Setup the POWERLINK network as described in main readme.txt located in
   openPOWERLINK main directory.

2. Program SOF file with Quartus II Programmer into FPGA.
   It is located in the following subdirectory of openPOWERLINK
   main directory:
   \Examples\altera_nios2\EBV_DBC3C40\design_nios2_openmac\nios_openMac.sof
   or
   \Examples\altera_nios2\TERASIC_DE2-115\design_nios2_openmac\nios_openMac.sof


3. Download the ELF file to the target:

    $ cd Examples\altera_nios2\no_os\gnu\demo_directIO
    $ make download-elf

4. Enjoy the running POWERLINK network.


How to import the project into Nios II IDE for debugging purposes
------------------------------------------------------------------

Requirement: Steps in the previous section "How to build the binary" are completed.

1. Start the "Nios II IDE"

2. Select menu "File" -> "Import..."

3. Select the import source "Altera Nios II" ->
   "Existing Nios II software build tools project or folder into workspace"

4. Browse to the main directory of openPOWERLINK
   (via the button "Browse...")

5. Press the button "Finish".

6. Select menu "File" -> "Import..."

7. Select the import source "Altera Nios II" ->
   "Existing Nios II software build tools project or folder into workspace"

8. Browse to the subdirectory
   Examples\altera_nios2\no_os\gnu\demo_directIO
   of openPOWERLINK (via the button "Browse...")

9. Press the button "Finish".

