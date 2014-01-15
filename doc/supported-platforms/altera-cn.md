openPOWERLINK CN on Altera Nios II {#page_platform_altera-cn}
===================================

[TOC]

# Introduction {#sect_altera-cn_introduction}

This file contains documentation for the openPOWERLINK stack on Altera NiosII.
 It uses the POWERLINK IP-Core which consists of a optimized MAC for POWERLINK
(openMAC) and a hub for daisy chaining several controled nodes.

# Contents {#sect_altera-cn_contents}

- FPGA design with Nios II CPU and POWERLINK IP-Core.
- Use HEX, LEDs and push buttons as digital I/Os

# Performance Data

- Minimum cycle length: 400 us
- PReq-PRes Latency: 1 us
- Process data: 4 bytes input and 4 bytes output.
- There is 1 RPDOs and 1 TPDO available. Cross-traffic is disabled.

# Requirements {#sect_altera-cn_requirements}

- Development Board TERASIC_DE2-115 (INK Board)

- Altera Quartus II v13.0 SP1 (Web Edition is also possible)
  - <https://www.altera.com/download/archives/arc-index.jsp>
  - <ftp://ftp.altera.com/outgoing/release/>

- Experiences with this development environment is required.

- POWERLINK network with managing node (MN)
  * openPOWERLINK managing node, e.g. Linux
  * B&R POWERLINK managing node
  * other POWERLINK managing node

# Hardware Setup {#sect_altera-cn_hardware}

- Setup for the TERASIC_DE2-115 (INK).
  * Download the user guide for the board from the
    [Terasic Website](http://www.terasic.com.tw/cgi-bin/page/archive.pl?Language=English&CategoryNo=139&No=502&PartNo=4)

  * Set the jumpers JP1, JP2, JP3, JP6 and JP7.

  * Set switch SW 19 to *Run*.

  * Connect the USB-Blaster to your PC via USB-cable.

  * Connect one Ethernet port of the INK-board to a POWERLINK MN
    with the delivered cables.

  * Set the POWERLINK Node ID to '1': Push SW 10 to upper position.

  * Switch on the board with the Power button (SW 18).

# How to build the binaries  {#sect_altera-cn_build}

The Altera Quartus II projects are located in `fpga/boards/altera/TERASIC_DE2-115.`\n
The subdirectories starting with `cn_` refer to designs for POWERLINK CN demos.

The Altera Qsys subsystem `cn_pcp` is located in `fpga/ipcore/altera/qsys`.\n

Steps 1-5 can be carried out by calling `$ ./create-this-fpga` in a "Nios II Command Shell".

1. Open the Quartus project file `ink_de2-115.qpf` with Altera Quartus II according to the demo you want to use.
2. Open Qsys via menu *Tools* -> *Qsys*.
3. Open the Qsys file (`*.qsys`) in the demo's directory, change to *Generation* page and press the button *Generate* to generate the Nios II system.
4. Close Qsys when the generation has finished (shown as information output).
5. Start the compilation in the Quartus II window via menu *Processing* -> *Start Compilation*.
6. Open "Nios II Command Shell"
7. Use the software design available in the openPOWERLINK subdirectory \n
   `apps/arch/altera_nios2/no_os/gnu/demo_cn_embedded`. \n
   Change to the directory and open the create-this-app.settings file.
   Make sure that the SOPC_DIR variable is set to the board design of your choice. \n
   If you want to run the INK demo set `SOPC_DIR=../../../../../fpga/boards/altera/TERASIC_DE2-115/cn_directIO/` \n
8. Run script *create-this-app* in to create the Makefile for application and BSP. \n
   `$ ./create-this-app`
9. Run make to build the ELF file after changing the sources. (The script create-this-app will do this automatically) \n
   `$ make`
10. Rebuild the Makefile also if the Nios II Design was changed inside Qsys. \n
    `$ ./create-this-app --rebuild`

# How to run the demo {#sect_altera-cn_run}

Note: You can combine step 2-3 by typing the following into the
    "Nios II Command Shell":\n
    `$ make download-all`\n
    (This will download the bitstream to the FPGA and afterwards the elf to the
    Nios II CPU.)

1. Setup the POWERLINK network.
2. Program the SOF file with Quartus II Programmer into FPGA.\n
   For the TERASIC_DE2-115 (INK) it is located in the following subdirectory:\n
   `openPOWERLINK_ROOT/fpga/boards/altera/TERASIC_DE2-115/cn_directIO/ink_de2-115.sof`\n
3. To Download the ELF file to the target, change to the openPOWERLINK subdirectory:\n
   `openPOWERLINK_ROOT/apps/arch/altera_nios2/no_os/gnu/demo_cn_embedded`\n
   And call:\n
   `$ make download-elf`\n
4. Enjoy the running POWERLINK network.

# How to import the project into Nios II Software Build Tools for Eclipse for debugging purposes {#sect_altera-cn_import}

Requirement: Steps in the previous section "How to build the binary" are
completed.
1. Start the Nios II Software Build Tools for Eclipse
2. Select menu *File -> Import...*
3. Select the import source *Nios II Software Build Tools Project* -> *Import Nios II Software Build Tools Project*
4. Browse to `apps/arch/altera_nios2/no_os/gnu/demo_cn_embedded` (via the button *Browse...*)
5. Set the project name to `demo_cn_embedded`.
6. Press the button *Finish*.

# How to write the program to local flash  {#sect_altera-cn_flash}

Requirement: Steps in the previous section _How to build the binary_ are
completed.

1. After successfully building the design use the makefile to program the
   flash:\n
   `$ make program-epcs`

# Troubleshooting {#sect_altera-cn_trouble}

1. It is advised to clean all generated files after switching from one demo to
   the other.
