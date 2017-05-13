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

The following CN Quartus projects are available for the TERASIC DE2-115 INK board:
- Single processor GPIO demo:
  - `hardware/boards/terasic-de2-115/cn-single-gpio`
- Dual processor GPIO demos:
  Both processors are interconnected by an parallel interface. The processor
  running the device's main application is called host. The host is
  implemented as a second internal Nios II processor or a external Nios II
  on a second TERASIC DE2-115 board.

  Internal host processor:
    - `hardware/boards/terasic-de2-115/cn-dual-hostif-gpio`

  External host processor:
    - `hardware/boards/terasic-de2-115/cn-single-hostif-drv`
    - `hardware/boards/terasic-de2-115/cn-single-hostif-gpio`

\note Hardware designs can be ported easily to other platforms by reusing
the Qsys subsystem instances in `hardware/ipcore/altera/qsys/cn_pcp`!

The software project for the host can be found in `apps`:
- Host "demo_cn_embedded":

  `apps/demo_cn_embedded/build/altera-nios2`

# Performance Data

- Minimum cycle length: 400 us
- PReq-PRes Latency: 1 us
- Process data: 4 bytes input and 4 bytes output.
- There is 1 RPDO and 1 TPDO available. Cross-traffic is disabled.

# Requirements and tools {#sect_altera-cn_requirements}

- Development Board TERASIC_DE2-115 (INK Board)

- Altera Quartus II v13.0 SP1 (Web Edition is also possible)
- Altera Nios II EDS v13.0 SP1
- Altera USB-Blaster driver

  <https://www.altera.com/downloads/download-center.html#>

  \note Contact your local Altera distributor in order to acquire a Nios
  II IP-Core license. This license is required for compiling the PCP FPGA
  configuration with Quartus II (generation of an *.sof file). Other than
  for your final product, no Nios II IP-Core license is needed for
  evaluating the supplied example designs.

- Experience with this development environment is required.

- POWERLINK network with managing node (MN)
  * openPOWERLINK managing node, e.g. Linux
  * B&R POWERLINK managing node
  * other POWERLINK managing node

# Build process overview {#sect_altera-cn_build_overview}

The following build steps need to be executed to run a CN demo:

1. \ref sect_altera-cn_hardware

    Preparation instructions for the demo board can be found in this section.

2. \ref sect_altera-cn_build-hardware

    If you have two TERASIC_DE2-115 boards available, you can consider to build the host processor example
    design `cn-single-hostif-gpio` for the second board in combination with the PCP design
    `cn-single-hostif-drv` running on the first board.\n
    Otherwise, you need to choose only one of the described FPGA projects.

3. \ref sect_altera-cn_makefile_build

    - Build the host processor software

        This only applies to the following FPGA projects:
        - cn-single-gpio
        - cn-dual-hostif-gpio
        - cn-single-hostif-gpio

    - Build the PCP driver software

        This only applies to the following FPGA projects:
        - cn-dual-hostif-gpio
        - cn-single-hostif-drv

4. \ref sect_altera-cn_run_demo

    - Program the FPGA bitstream to the FPGA.
    - Program the software to the FPGA board to run it on the FPGA soft-core(s).

# Hardware setup {#sect_altera-cn_hardware}

## Setup for the TERASIC_DE2-115 (INK) board

In order to get a POWERLINK slave up and running the TERASIC DE2-115 has to be
set up. This chapter explains how this board needs to be configured and connected
to the network.

For general information download the user guide for the board from the
[TERASIC Website](http://www.terasic.com.tw/cgi-bin/page/archive.pl?Language=English&CategoryNo=139&No=502&PartNo=4).

The following settings apply to all examples provided for this board:
* Set the jumpers JP1, JP2, JP3, JP6 and JP7.
* Set switch SW 19 to *Run*.
* Connect the USB-Blaster to your PC via the USB-cable – see
  "USB-Blaster Connection" in Settings for POWERLINK CN example
  designs on the INK Board.
* Connect one Ethernet port of the INK-board to a POWERLINK MN.
* Set the POWERLINK Node ID to '1': Push SW 10 to upper position.
* Switch on the board with the power button (SW 18).

\note In field applications CAT 5e cables are required.

\note The I/O standard for all involved pins is 3.3 V. Do not connect Vcc
signals between different boards, but connect the GND pins to ensure a
common ground level.

![Settings for POWERLINK CN example designs on the INK Board.](\ref a_cn_devboard-terasic-de2-115.png)

\note The application inputs are mapped to object number 0x6000 with subindex
0x01. The application outputs are mapped to object number 0x6200 with subindex 0x01.

### PCP with GPIO interface

GPIO FPGA pins are connected on the board to the application inputs and outputs.
This example can be found in `hardware/boards/terasic-de2-115/cn-single-gpio`.\n
No additional settings required.

### PCP with an FPGA internal interface

The PCP is connected internally in the FPGA to a host processor. The host processor is driving GPIO FPGA pins
on the board which are connected to the application inputs and outputs.
This example can be found in `hardware/boards/terasic-de2-115/cn-dual-hostif-gpio`.\n
No additional settings required.

### PCP with an external parallel interface

The PCP is connected to a host processor via external FPGA pins.
There are two hardware examples available in the `<OPLK_BASE_DIR>/hardware/boards/terasic-de2-115`

* cn-single-hostif-drv \n
Implements a PCP design with a 16 bit (data width) parallel interface.
The data exchange between host and PCP is done over a shared memory.

* cn-single-hostif-gpio \n
Implements an example host ready for connection to ink_pcp_16bitprll.
This is particularly useful if you have two TERASIC DE2-115 boards available.
The boards can be connected with the active JP5 header pins one by one.

When using a design with 16 bit parallel interface, the following pins
on header JP5 (16 bit parallel pins at INK board GPIO header JP5) are used for
externally connecting the application processor.

| PCP function |             FPGA PIN |   Header PIN  |  Header PIN  |  FPGA PIN  |  PCP function         |
| :----------: | :------------------: | :-----------: | :----------: | :--------: | :--------------:      |
| HOSTIF_AD[0] |             AB22     |   1           |  2           |  AC15      |  HOSTIF_AD[1]         |
| HOSTIF_AD[2] |             AB21     |   3           |  4           |  Y17       |  HOSTIF_AD[3]         |
| HOSTIF_AD[4] |             AC21     |   5           |  6           |  Y16       |  HOSTIF_AD[5]         |
| HOSTIF_AD[6] |             AD21     |   7           |  8           |  AE16      |  HOSTIF_AD[7]         |
| HOSTIF_AD[8] |             AD15     |   9           |  10          |  AE15      |  HOSTIF_AD[9]         |
| don’t connect 5V to host ! |        |   11          |  12          |            | connect GND to host!  |
| HOSTIF_AD[10] |            AC19     |   13          |  14          |  AF16      |  HOSTIF_AD[11]        |
| HOSTIF_AD[12] |            AD19     |   15          |  16          |  AF15      |  HOSTIF_AD[13]        |
| HOSTIF_AD[14] |            AF24     |   17          |  18          |  AE21      |  HOSTIF_AD[15]        |
| HOSTIF_AD[16] |            AF25     |   19          |  20          |  AC22      |                       |
|               |            AE22     |   21          |  22          |  AF21      |                       |
|               |            AF22     |   23          |  24          |  AD22      |                       |
|               |            AG25     |   25          |  26          |  AD25      |  HOSTIF_BE[0]         |
| HOSTIF_BE[1]  |            AH25     |   27          |  28          |  AE25      |                       |
| don’t connect 3.3V to host ! |      |   29          |  30          |            | connect GND to host!  |
|               |            AG22     |   31          |  32          |  AE24      |                       |
|               |            AH22     |   33          |  34          |  AF26      |                       |
| HOSTIF_ALE_n  |            AE20     |   35          |  36          |  AG23      |  HOSTIF_CS_n          |
| HOSTIF_WR_n   |            AF20     |   37          |  38          |  AH26      |  HOSTIF_RD_n          |
| HOSTIF_ACK_n  |            AH23     |   39          |  40          |  AG26      |  HOSTIF_IRQ_n         |


# Create the FPGA configuration  {#sect_altera-cn_build-hardware}

An FPGA configuration has to be built to implement the driver CPU (PCP) and/or the host CPU (choose one from the table below).
These FPGA configuration projects can be found in the folder `hardware/boards/terasic-de2-115`.
The compilation result of the FPGA project will be a binary file (*.sof) which can be loaded into the FPGA.\n
Afterwards, in section \ref sect_altera-cn_makefile_build a software has to be compiled for driver CPU (PCP) and/or the host CPU.

| design               | host | driver |
|:--------------------:|:----:|:------:|
|cn-single-gpio        |x| |
|cn-dual-hostif-gpio   |x|x|
|cn-single-hostif-drv  | |x|
|cn-single-hostif-gpio |x| |

There are two options to build the FPGA binary file (choose one):
1. Creating the FPGA configuration manually\n
   This involves using the graphical Quartus tools.
2. Creating the FPGA configuration by using make\n
   This is a pure command line approach.

## Option 1: Creating the FPGA configuration manually

This section describes the generation of the FPGA bitstream by using the
Altera toolchain.

The demo software for the POWERLINK evaluation boards is located in the
`hardware/boards/terasic-de2-115` directory of your POWERLINK Slave release.
This directory consists of the hardware configuration for the different
supported applications. In order to compile the FPGA configuration the following
steps need to be carried out:

  1. Open Quartus II 13.0 SP1 (32/64-bit)

  2. Open your desired project by using "File - Open Project" and choose the
  path to your design: (e.g: `<OPLK_BASE_DIR>/hardware/boards/terasic-de2-115/cn-dual-hostif-gpio/quartus/<design_name>.qpf`)

  3. Open Qsys (Tools - Qsys)

  4. In Qsys open the corresponding Qsys design (File - Open)

  5. In Qsys change to the "Generation" register and click the "Generate" button
  in order to generate the source files of the processor system.
  ![Qsys generate button](\ref a_cn_qsys_generate.png)

  6. After the system generation was successful Qsys can be closed and the
  FPGA configuration needs to be generated with Quartus II. Start the
  compilation by clicking "Processing - Start Compilation".

  7. After the compilation is completed Quartus displays a message box.


## Option 2: Create the FPGA configuration by using make

  1. Open the "Nios II Command Shell"

  2. Change the directory to the desired hardware project
  (e.g: `<OPLK_BASE_DIR>/hardware/boards/terasic-de2-115/<hardware_design_name>/quartus`)

  3. Execute "make all"

# Build the software  {#sect_altera-cn_makefile_build}

In order to compile and initialize the build system of the desired design a set
of scripts is provided. Each script builds a Makefile.

  1. Makefile for the host processor software.

  2. Makefile for the driver processor software (not needed for the "cn-single-gpio" design).

The Makefile is required for building the software with the "make" tools. After the Makefile
was generated, the software (*.elf file) can be built.

## Building the host processor software

  1. Open the "Nios II Command Shell"

  2. Open your openPOWERLINK software build subdirectory of the host.
  `<OPLK_BASE_DIR>/apps/demo_cn_embedded/build/altera-nios2`

  3. Run script *create-this-app* to create the Makefile for the demo.

   `$ ./create-this-app`

    - The script's help message can be printed with `--help`.

        `$ ./create-this-app --help`

    - Rebuild the Makefile, if the Nios II Design was changed inside Qsys.

        `$ ./create-this-app --rebuild`

    - For debugging purposes, use `--debug` option.

        `$ ./create-this-app --debug`

      \note If it is necessary to change the board, use the `--board` option:
      \note `$ ./create-this-app --board [PATH-TO-BOARD]`

  4. Run make to build the ELF file after changing the sources.

    `$ make all`

## Building the PCP driver software

\note This step is not needed for the "cn-single-gpio" design.

1. Open the "Nios II Command Shell"

2. Open openPOWERLINK software build subdirectory of the host.

  `<OPLK_BASE_DIR>/drivers/altera-nios2/drv_daemon/build`

3. Run the script *create-this-app* to create the Makefile for the demo.

  `$ ./create-this-app`

  - The script's help message can be printed with `--help`.

    `$ ./create-this-app --help`

  - Rebuild the Makefile, if the Nios II Design was changed inside Qsys.

    `$ ./create-this-app --rebuild`

  - If you want to debug the demo, use the `--debug` option.

    `$ ./create-this-app --debug`

    \note If you want to change to another board, use the `--board` option:
    \note `$ ./create-this-app --board [PATH-TO-BOARD]`

4. Run make to build the ELF file after changing the sources.

  `$ make all`


# How to run the demo {#sect_altera-cn_run_demo}

The following build steps are necessary to run the demo on the evaluation board.

1. Download the bitstream to the FPGA.

2. Program the software to the soft-core(s) on the FPGA.

3. Connect your POWERLINK slave to a POWERLINK network.


## Program the bitstream to the FPGA

There are two options to program the bitstream to the FPGA (choose one):

  1. Using the Quartus II Programmer

  2. Using the Makefile


### Option 1: Using the Quartus II Programmer

The SOF file for the TERASIC_DE2-115 (INK) is located in the following subdirectory:

  `<OPLK_BASE_DIR>/hardware/boards/terasic-de2-115/<hardware_design_name>/quartus/<hardware_design_name>.sof`

  - Open Quartus v13.0 SP1

  - Open Quartus II Programmer (Tools - Programmer)

  - Add *.sof file for downloading

    `<OPLK_BASE_DIR>/hardware/boards/terasic-de2-115/<hardware_design_name>/quartus/<hardware_design_name>.sof`

    ![Open Quartus II Programmer *.sof file](\ref a_cn_qua_prog_o_sof.png)

   - Press the start button

### Option 2: Using the Makefile

1. Open the "Nios II Command Shell"

2. Change the directory to the build directory of the driver or host.

  `<OPLK_BASE_DIR>/drivers/altera-nios2/drv_daemon/build`

  or

  `<OPLK_BASE_DIR>/apps/demo_cn_embedded/build/altera-nios2`

3. Run make download-bits to download the bitstream.

  `$ make download-bits`


## Program the software to the soft-core on the FPGA

\note If the FPGA project is `cn-dual-hostif-gpio`, you need to carry out
      the following steps for the driver \b and the host (in this order).

1. Open the "Nios II Command Shell"

2. Change the directory to the build directory of the driver or host.

  `<OPLK_BASE_DIR>/drivers/altera-nios2/drv_daemon/build`

  or

  `<OPLK_BASE_DIR>/apps/demo_cn_embedded/build/altera-nios2`

3. Run make download-elf to download the software.

  If the software was built in release mode use:

  - `$ make download-elf`

  or in debug mode use:

  - `$ make download-elf && nios2-terminal -i <instance USB-Blaster (1 or 0)>`

\note Workaround for testing without a Nios II license:\n
      If you did not specify a valid Nios II IP-Core license, the terminal windows
      must not be closed while executing the software in a second terminal window.
      The Quartus programmer is the preferred way for programming the time limited *.sof.
\par
\note A comfortable way to use "make" is described in \ref sect_altera-cn-make-EDS

## Connect your POWERLINK slave to a POWERLINK network

1. Setup the POWERLINK network.

2. Enjoy the running system.


# Additional information {#sect_altera-cn-addon}

## How to import the project into the Nios II EDS {#sect_altera-cn-imort-EDS}

In order to be able to use the "Nios II Software Build Tools for Eclipse" the
following steps are required for importing the SW reference project.

\note Before you can import the EDS project, the Makefile needs to be generated.
The Makefile will be build with create-this-app. This script generates the
Makefile and initializes the built system. It needs to be executed each time
the FPGA configuration has changed with the option "--rebuild". An error message
will be printed, if the Makefile is not up to date.

\note `Makefile not up to date.`

\note `<OPLK_BASE_DIR>/hardware/boards/terasic-de2-115/<hardware_design_name>/
quartus/<hardware_design_name>.sopcinfo has been modified since the BSP was generated.`

\note `Generate the BSP to update the Makefile, and then build again.`

\note `To generate from Eclipse:`\n
  `1. Right-click the BSP project.`\n
  `2. In the Nios II Menu, click Generate BSP.`

\note `To generate from the command line:`\n
  `nios2-bsp-generate-file --settings=<settings file> --bsp-dir=<target bsp files directory>`

#### Import Direct IO example

1. Open the Nios II EDS 13.0 SP1 Software Build Tools for Eclipse

2. Choose a new workspace for the POWERLINK Slave

3. In Nios II EDS click "File - Import"

4. Select "Nios II Software Build Tools Project - Import Nios II Software Build
Tools Project" and click "Next"
![Import an existing project into workspace](\ref a_cn_ec_import_proj.png)

5. In the following import dialog select the `<OPLK_BASE_DIR>/apps/demo_cn_embedded/build/altera-nios2`
directory.
![Import the pcp directIO project into the Nios II EDS](\ref a_cn_ec_import_app.png)

6. Choose an appropriate project name (e.g. "DirectIO") and set
"MinGW Nios II GCC4"(Windows default) or "Linux Nios II GCC4"(Linux default) as toolchain.

7. Click the "Finish" button.

In order to have all paths resolved correctly, you need to rebuild the software
from the Nios II EDS again, since it will parse those paths with the output of
"make" during the build. Refer to the next chapter to get more information
about the "make" build process.


#### Import dual processor example

The steps above have to be repeated in the same way when working with the dual
processor example. In this case, the following directories have to be imported:

- `<OPLK_BASE_DIR>/drivers/altera-nios2/drv_daemon/build`
![Import the Driver_PCP project into the Nios II EDS](\ref a_cn_ec_import_drv.png)

- `<OPLK_BASE_DIR>/apps/demo_cn_embedded/build/altera-nios2`
![Import the host project into the Nios II EDS](\ref a_cn_ec_import_app.png)


### Using make in Nios II EDS for software compilation {#sect_altera-cn-make-EDS}

It is handy to execute the Makefile from the "Nios II EDS". For these optional
feature the "Make Target" view has to be visible. The following steps show how
to do this.

1. In the EDS with your imported projects click on "Window - Show View - Other"
![Open the additional MakeTarget view in the EDS](\ref a_cn_ec_make_ta.png)

2. Select the "Make Target" under the category "Make"

3. A new window shows up in the bottom window of the EDS which is shown in
the figure below. In this view it is possible to add missing make targets from the
table below. However, not all projects will support all listed make targets.
![The MakeTarget view with all predefined make targets](\ref a_cn_ec_make_tag.png)

4. The table below gives a detailed description of all available make targets.

|Target        |Description |
|:------------:|:--:|
|all           |Recompile the changed parts of the software and link the .elf binary |
|clean         |Clean all generates files |
|download-bits |Download the bitstream to the target |
|download-elf  |Download the software to the Nios II processor |
|erase-epcs    |Erase the local EPCS flash device |
|program-epcs  |Program the generated flash image to the local EPCS flash device |

## How to write the program to the flash memory  {#sect_altera-cn_flash}

Requirement:\n
Steps in the previous sections \ref sect_altera-cn_build-hardware and \ref sect_altera-cn_makefile_build are completed.

- Use the Makefile to program the flash:\n
   `$ make program-epcs`

   This command will store the FPGA bitstream and the software in the board's
   flash memory. After the next power cycle the complete design will load
   automatically from flash.

## Troubleshooting {#sect_altera-cn_trouble}

1. It is advised to clean all generated files after switching from one demo to
   the other.
2. Eclipse project import fails\n
   Make sure the Eclipse workspace is neither located in the makefile's directory nor its subdirectory.
   Best practice is to place the workspace location outside the openPOWERLINK sources.
