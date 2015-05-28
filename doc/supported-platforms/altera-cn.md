openPOWERLINK CN on Altera Nios II {#page_platform_altera-cn}
===================================

[TOC]

# Introduction {#sect_altera-cn_introduction}

The POWERLINK CN on Altera FPGA implementation utilizes IP-Cores available in
Altera Qsys and provided with the openPOWERLINK stack in the directory
`hardware/ipcore`.

The CN implementation applies the soft-core processor Altera Nios II to execute
the protocol stack. The Ethernet interface is built with the POWERLINK-optimized
controller openMAC, which enables e.g. low frame response delays.
Additionally, a hub IP-Core is provided enabling daisy chained networks.

# Contents {#sect_altera-cn_contents}

The following CN FPGA design is available for the TERASIC DE2-115 INK board:
- Single GPIO demo: \n
    `hardware/boards/terasic-de2-115/cn-single-gpio`

_Note that hardware designs can be ported easily to other platforms by reusing
the Qsys subsystem instances in `hardware/ipcore/altera/qsys/cn_pcp`!_

The software project for the PCP can be found in `apps`:
- PCP "demo_mn_embedded": \n
    `apps/demo_cn_embedded/build/altera-nios2`

# Performance Data

- Minimum cycle length: 400 us
- PReq-PRes Latency: 1 us
- Process data: 4 bytes input and 4 bytes output.
- There is 1 RPDO and 1 TPDO available. Cross-traffic is disabled.

# Requirements {#sect_altera-cn_requirements}

- Development Board TERASIC_DE2-115 (INK Board)

- Altera Quartus II v13.0 SP1 (Web Edition is also possible)
  - <https://www.altera.com/downloads/download-center.html#>

- Experience with this development environment is required.

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

The Altera Quartus II projects are located in
`hardware/boards/[EVALBOARD]/quartus.`\n
The subdirectories starting with `cn-` refer to designs for POWERLINK CN demos.

The Altera Qsys subsystem `cn_pcp` is located in
`hardware/ipcore/altera/qsys`.\n

Steps 1-5 can be carried out by executing `$ make all` in a
"Nios II Command Shell".

1. Open the Quartus project file `*.qpf` with Altera Quartus II according to the
   demo you want to use.
2. Open Qsys via menu *Tools* -> *Qsys*.
3. Open the Qsys file (`*.qsys`) in the demo's directory, change to
   *Generation* page
   and press the button *Generate* to generate the Nios II system.
4. Close Qsys when the generation has finished (shown as information output).
5. Start the compilation in the Quartus II window via menu *Processing* ->
   *Start Compilation*.
6. Open "Nios II Command Shell"
7. Use the software design available in the openPOWERLINK subdirectory \n
   `apps/demo_cn_embedded/build/altera-nios2`. \n
8. Run script *create-this-app* to create the Makefile for the demo. \n
   `$ ./create-this-app` \n
   Note: The script uses the TERASIC DE2-115 CN demo by default.
   If you want to change to another board, use the `--board` option: \n
   `$ ./create-this-app --board [PATH-TO-BOARD]` \n
9. Run make to build the ELF file after changing the sources.\n
   `$ make`
10. Rebuild the Makefile also if the Nios II Design was changed inside Qsys. \n
    `$ ./create-this-app --rebuild`
11. If you want to debug the demo, use the `--debug` option. \n
    `$ ./create-this-app --debug`
12. The script's help message can be printed with `--help`. \n
    `$ ./create-this-app --help`

# How to run the demo {#sect_altera-cn_run}

Note: You can combine step 2-3 by typing the following into the
    "Nios II Command Shell":\n
    `$ make download-all`\n
    (This will download the bitstream to the FPGA and afterwards the elf to the
    Nios II CPU.)

1. Setup the POWERLINK network.
2. Program the SOF file with Quartus II Programmer into FPGA.\n
   For the TERASIC_DE2-115 (INK) it is located in the following subdirectory:\n
   `openPOWERLINK_ROOT/hardware/boards/terasic-de2-115/cn-single-gpio/quartus/cnSingleGpio.sof`\n
3. To Download the ELF file to the target, change to the openPOWERLINK
   subdirectory:\n
   `openPOWERLINK_ROOT/apps/demo_cn_embedded/build/altera-nios2`\n
   And call:\n
   `$ make download-elf`\n
4. Enjoy the running POWERLINK network.

# How to import the project into Nios II Software Build Tools for Eclipse for debugging purposes {#sect_altera-cn_import}

Requirement: Steps in the previous section *How to build the binaries* are
completed.
1. Start the Nios II Software Build Tools for Eclipse\n
   Note: It is recommended to place the workspace location outside the openPOWERLINK directory
2. Select menu *File -> Import...*
3. Select the import source *Nios II Software Build Tools Project* -> *Import
   Nios II Software Build Tools Project*
4. Browse to `apps/demo_cn_embedded/build/altera-nios2` (via the button
   *Browse...*)
5. Set the project name to `demo_cn_embedded`.
6. Press the button *Finish*.

# How to write the program to local flash  {#sect_altera-cn_flash}

Requirement: Steps in the previous section *How to build the binaries* are
completed.

1. After successfully building the design use the makefile to program the
   flash:\n
   `$ make program-epcs`

# Troubleshooting {#sect_altera-cn_trouble}

1. It is advised to clean all generated files after switching from one demo to
   the other.
2. Eclipse project import fails\n
   Make sure the Eclipse workspace is neither located in the makefile's directory nor its subdirectory.
   Best practice is to place the workspace location outside the openPOWERLINK sources.
