openPOWERLINK CN on Altera Nios II {#page_platform_altera-cn}
===================================

[TOC]

# Introduction {#sect_altera-cn_introduction}

This file contains documentation for the openPOWERLINK stack on Altera NiosII.
 It uses the POWERLINK IP-Core which consists of a optimized MAC for POWERLINK
(openMAC) and a hub for daisy chaining several controled nodes.

# Contents {#sect_altera-cn_contents}

- FPGA design with Nios II CPU and POWERLINK IP-Core.
- Latched I/0 Ports: 4 x 8Bit - plus latch signal.
  Direction (input or output) can be defined by configuration-pin level.

# Performance Data

- Minimum cycle length: 400 us
- PReq-PRes Latency: 1 us
- Process data: 4 bytes input and 4 bytes output.
- There is 1 RPDOs and 1 TPDO available. Cross-traffic is disabled.

# Requirements {#sect_altera-cn_requirements}

- Development Board TERASIC_DE2-115 (INK Board) or SYSTEC ECUcore-EP3C.

- USB-Blaster for programming the SYSTEC ECUcore-EP3C.

- Altera Quartus II v10.1 SP1 or newer (Web Edition is also possible)
  and Altera Nios II Embedded Design Suite v10.1 SP1 or newer.
  * <https://www.altera.com/download/archives/arc-index.jsp>
  * <ftp://ftp.altera.com/outgoing/release/>

- Experiences with this development environment is required.

- POWERLINK network as described in main readme.txt
  or alternatively a POWERLINK network with Configuration Manager.
  The corresponding XDD for this node can be found in the subdirectory
  ObjDicts/CiA401_CN of the openPOWERLINK main directory.

# Hardware Setup {#sect_altera-cn_hardware}

- Setup for the TERASIC_DE2-115 (INK).
  * Download the user guide for the board from the
    [Terasic Website](http://www.terasic.com.tw/cgi-bin/page/archive.pl?Language=English&CategoryNo=139&No=502&PartNo=4)

  * Set the jumpers JP1, JP2, JP3, JP6 and JP7.

  * Set switch SW 19 to *Run*.

  * The jumpers for SPI should not be set, since you will run the directIO
    example, so leave them aside.

  * Connect the USB-Blaster to your PC via USB-cable.

  * Connect one Ethernet port of the INK-board to a POWERLINK MN
    with the delivered cables.

  * Set the POWERLINK Node ID to '1': Push SW 10 to upper position.

  * Switch on the board with the Power button (SW 18).


- Setup for the SYSTEC ECUcore-EP3C.
  * Details for the setup of this board is available on the
    [SYSTEC Website](http://www.systec-electronic.com/uploads/be/36/be36d3caf3ff425f0e6bdadfc6c01aaa/L-1266d_02_DevelopmentBoard-ECUcore-EP3C.pdf)

# How to build the binaries  {#sect_altera-cn_build}

## How to build the binaries for the SYSTEC ECUcore-EP3C 

Steps 1-6 are only necessary if you want to change the FPGA design.
Otherwise you can use the supplied SOF file and go directly to step 7.
1. Open the Quartus project file `nios_openMac.qpf` with Altera Quartus II
   according to the board you want to use.
2. Open the SOPC Builder via menu `Tools` -> `SOPC Builder`.
3. Press the button "Generate" in the SOPC Builder to regenerate the Nios II
   system.
4. Close the SOPC Builder when the generation has finished (shown as information
   output).
5. A message window will pop up. Choose "Update: All symbols or blocks in this
   file".
6. Start the compilation in the Quartus II window via menu
   `Processing` -> `Start Compilation`.
   Choose `Yes` for saving all changed files.
7. Use the design with the supplied demo projects in the openPOWERLINK
   subdirectory `Examples\arch\altera_nios2\no_os\gnu\demo_directIO`.
   Therefore open the `create-this-app.settings` file with a text editor and set the "Relative path to SOPC file"
   to `SOPC_DIR=../../../../../../fpga/boards/altera/SYSTEC_ECUcore-EP3C/design_nios2_directIO`\n
8. Open "Nios II Command Shell"
9. Run script `create-this-app` to create the Makefile for the demo application
   and the necessary BSP. (This step is only necessary once)\n
   `$ ./create-this-app`\n
10. Run make to build the ELF file after changing the sources.
   (The script create-this-app will do this automatically)\n
   `$ make`\n
11. Rebuild the Makefile for the demo application and the
   BSP, if the Nios II Design was changed inside the SOPC Builder.\n
   `$ ./create-this-app --rebuild`

## How to build the binaries for the TERASIC_DE2-115 (INK)

Steps 1-5 are only necessary if you want to change the FPGA design.\n
The steps 1-5 are combined in the script `create-this-fpga`. To run this script open the "Nios II Command Shell"
and change to the directory: `openPOWERLINK_ROOT/fpga/boards/altera/TERASIC_DE2-115/cn_directIO` and run
   `$ ./create-this-fpga`\n
Otherwise you can use the supplied SOF file and go directly to step 6.
1. Open the Quartus project file `cn_directIO.qpf` with Altera Quartus II
   according to the board you want to use.
2. Open the QSYS Tool via menu `Tools` -> `QSYS`.
3. Press the button `Generate` in the "Generation" tab of QSYS to regenerate the Nios II system.
4. Close QSYS when the generation has finished  (shown as information
   output).
5. Start the compilation to generate the "cn_directIO.sof" in the Quartus II window via menu
   `Processing` -> `Start Compilation`.
   Choose `Yes` for saving all changed files.
6. Use the design with the supplied demo projects in the openPOWERLINK
   subdirectory `Examples\arch\altera_nios2\no_os\gnu\demo_directIO`.
   Therefore open the `create-this-app.settings` file with a text editor and set the "Relative path to SOPC file"
   to `SOPC_DIR=../../../../../../fpga/boards/altera/TERASIC_DE2-115/cn_directIO/`\n
7. Open "Nios II Command Shell"
8. Change to the openPOWERLINK subdirectory:
  `openPOWERLINK_ROOT/Examples/arch/altera_nios2/no_os/gnu/demo_directIO`.
9. Run script `create-this-app` to create the Makefile for the demo application, the necessary BSP
   and the ELF file.(This step is only necessary once)\n
   `$ ./create-this-app`\n
10. Run make to build the ELF file after changing the sources.
   (The script create-this-app will do this automatically)\n
   `$ make`\n
11. Rebuild the Makefile for the demo application, the
    BSP and the ELF file, if the Nios II Design was changed inside the QSYS Builder.\n
    `$ ./create-this-app --rebuild`

# How to run the demo {#sect_altera-cn_run}

1. Setup the POWERLINK network as described in main readme.txt located in
   openPOWERLINK main directory.
2. Program the SOF file with Quartus II Programmer into FPGA.\n
   For the SYSTEC ECUcore-EP3C it is located in the following subdirectory of openPOWERLINK
   main directory:\n
   `openPOWERLINK_ROOT/fpga/boards/altera/SYSTEC_ECUcore-EP3C/design_nios2_directIO/nios_openMac.sof`\n
   And for the TERASIC_DE2-115 (INK) it is located in the following subdirectory:\n
   `openPOWERLINK_ROOT/fpga/boards/altera/TERASIC_DE2-115/cn_directIO/cn_directIO.sof`\n
3. To Download the ELF file to the target, change to the openPOWERLINK subdirectory:\n
   `openPOWERLINK_ROOT/Examples/arch/altera_nios2/no_os/gnu/demo_directIO`\n
   And call:\n
   `$ make download-elf`\n
4. Enjoy the running POWERLINK network.

# How to import the project into Nios II IDE for debugging purposes  {#sect_altera-cn_import}

Requirement: Steps in the previous section "How to build the binary" are
completed.
1. Start the "Nios II IDE"
2. Select menu `File` -> `Import...`
3. Select the import source `Altera Nios II` ->
   `Existing Nios II software build tools project or folder into workspace`
4. Browse to the subdirectory of openPOWERLINK
   (via the button `Browse...`)\n
   `openPOWERLINK_ROOT/Examples/altera_nios2/no_os/gnu/demo_directIO`
5. Press the button `Finish`.

# How to run or debug the project in the Nios II IDE  {#sect_altera-cn_debug}

Requirement: Steps in the previous section "How to import the project into Nios II IDE for debugging purposes" are completed.
1. Press button `Project` -> `Build All`
2. After a successful compilation it is possible to run or debug the project.\n
   For running the project press the button `Run` -> `Run`.\n
   and for debugging press the button `Run`-> `Debug`.\n
   If the HW-configuration is missing follow the steps 3-9.
3. To setup a new HW-configuration select the menu `Run`-> `Run Configurations`
4. Select "Nios II Hardware", then right click and select "New".
5. Choose a name for the "New_configuration".
6. In the "Project" tab  select for the "Project name" the `demo_directIO`, the "Project ELF file name" is selected automatically.
7. In the "Target Connection" tab the check boxes `Download ELF to selected target system` and `Start processor` should be set. The rest should be default.
8. In the "Debugger" tab select the `Altera CDI GDB Debugger`.
9. Press `Apply` and `Close`. Now it should be possible to do the step 2.

# How to write the program to local flash  {#sect_altera-cn_flash}

Requirement: Steps in the previous section _How to build the binary_ are
completed.

1. After successfully building the design use the makefile to program the
   flash:\n
   `$ make program-epcs`

# Troubleshooting {#sect_altera-cn_trouble}

1. It is adviced to clean all generated files after switching from one demo to
   the other.
