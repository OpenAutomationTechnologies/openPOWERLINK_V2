openPOWERLINK on Altera Nios {#altera}
============================

## Introduction

This file contains documentation for the openPOWERLINK stack on Altera NiosII.
 It uses the POWERLINK IP-Core which consists of a optimized MAC for POWERLINK
(openMAC) and a hub for daisy chaining several controled nodes.

## Contents

- FPGA design with Nios II CPU and POWERLINK IP-Core.
- Latched I/0 Ports: 4 x 8Bit - plus latch signal.
  Direction (input or output) can be defined by configuration-pin level.

## Performance Data

- Minimum cycle length: 400 us
- PReq-PRes Latency: 1 us
- Process data: 4 bytes input and 4 bytes output.
- There is 1 RPDOs and 1 TPDO available. Cross-traffic is disabled.

## Requirements

- Development Board TERASIC_DE2-115 (INK Board), SYSTEC ECUcore-EP3C or
  EBV DBC3C40.

- USB-Blaster for programming the SYSTEC ECUcore-EP3C and the EBV DBC3C40 board.

- Altera Quartus II v10.1 SP1 or newer (Web Edition is also possible)
  and Altera Nios II Embedded Design Suite v10.1 SP1 or newer.
  * <https://www.altera.com/download/archives/arc-index.jsp>
  * <ftp://ftp.altera.com/outgoing/release/>

- Experiences with this development environment is required.

- POWERLINK network as described in main readme.txt
  or alternatively a POWERLINK network with Configuration Manager.
  The corresponding XDD for this node can be found in the subdirectory
  ObjDicts/CiA401_CN of the openPOWERLINK main directory.

## Hardware Setup

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
    
- Setup for the EBV DBC3C40.
  * Connect the USB-Blaster to the JTAG port of the board
  
  * Connect one Ethernet port of the EBV board to a POWERLINK MN
    with the delivered cables.
    
  * To power-up the board use the 24V I/O power supply.
    Note: Take care that you don't use the 12V power adapter in combination
          with the 24V I/O power supply.

- Setup for the SYSTEC ECUcore-EP3C.
  * Details for the setup of this board is available on the
    [SYSTEC Website](http://www.systec-electronic.com/uploads/be/36/be36d3caf3ff425f0e6bdadfc6c01aaa/L-1266d_02_DevelopmentBoard-ECUcore-EP3C.pdf)

## How to build the binaries

Steps 1-7 are only necessary if you want to change the FPGA design.
Otherwise you can use the supplied SOF file and go directly to step 8.
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
7. Open "Nios II Command Shell"
8. Use the design with the supplied demo projects in the openPOWERLINK
   subdirectory `Examples\arch\altera_nios2\no_os\gnu\demo_directIO`.
   Therefore, execute the following command in the "Nios II Command Shell"
   before calling create-this-app to set the `SOPC_DIR`.
   `$ export SOPC_DIR=../../../../../../fpga/boards/altera/SYSTEC_ECUcore-EP3C/design_nios2_directIO`\n
   or\n
   `$ export SOPC_DIR=../../../../../../fpga/boards/altera/TERASIC_DE2-115/design_nios2_directIO`\n
   or\n
   `$ export SOPC_DIR=../../../../../../fpga/boards/altera/EBV_DBC3C40/design_nios2_directIO`\n
9. Run script `create-this-app` to create the Makefile for the demo application
   and the necessary BSP. (This step is only necessary once)\n
   `$ ./create-this-app`
10.Run make to build the ELF file after changing the sources.
   (The script create-this-app will do this automatically)\n
   `$ make`
11.Rebuild the Makefile for the demo application and the
   BSP, if the Nios II Design was changed inside the SOPC Builder.\n
   `$ ./create-this-app --rebuild`

## How to run the demo

1. Setup the POWERLINK network as described in main readme.txt located in
   openPOWERLINK main directory.
2. Program SOF file with Quartus II Programmer into FPGA.
   It is located in the following subdirectory of openPOWERLINK
   main directory:\n
   `/fpga/boards/altera/EBV_DBC3C40/design_nios2_directIO/nios_openMac.sof`\n
   or\n
   `/fpga/boards/altera/TERASIC_DE2-115/design_nios2_directIO/nios_openMac.sof`\n
   or\n
   `/fpga/boards/altera/SYSTEC_ECUcore-EP3C/design_nios2_directIO/nios_openMac.sof`\n
3. Download the ELF file to the target:\n
   `$ cd Examples/arch/altera_nios2/no_os/gnu/demo_directIO`\n
   `$ make download-elf`\n
4. Enjoy the running POWERLINK network.


## How to import the project into Nios II IDE for debugging purposes

Requirement: Steps in the previous section "How to build the binary" are
completed.
1. Start the "Nios II IDE"
2. Select menu `File` -> `Import...`
3. Select the import source `Altera Nios II` ->
   `Existing Nios II software build tools project or folder into workspace`
4. Browse to the main directory of openPOWERLINK
   (via the button `Browse...`)
5. Press the button `Finish`.
6. Select menu `File` -> `Import...`
7. Select the import source `Altera Nios II` ->
   `Existing Nios II software build tools project or folder into workspace`
8. Browse to the subdirectory
   `Examples\altera_nios2\no_os\gnu\demo_directIO`
   of openPOWERLINK (via the button `Browse...`)
9. Press the button `Finish`.

## How to write the program to local flash

Requirement: Steps in the previous section _How to build the binary_ are
completed.

1. After successfully building the design use the makefile to program the
   flash:\n
   `$ make program-epcs`

## Troubleshooting

1. It is adviced to clean all generated files after switching from one demo to
   the other.
