openPOWERLINK on Xilinx Microblaze {#page_platform_xilinx}
==================================

[TOC]

# Introduction {#sect_xilinx_introduction}

This file contains documentation for the openPOWERLINK stack on Xilinx
Microblaze. It uses the POWERLINK IP-Core which consists of a optimized MAC
for POWERLINK (openMAC) and a hub for daisy chaining several controlled nodes.

 The demos and IP-Cores are available for the PLB and AXI bus system.


# Contents {#sect_xilinx_contents}

- FPGA design with Microblaze CPU and POWERLINK IP-Core.
- Latched I/0 Ports: 4 x 8Bit - plus latch signal.
  Direction (input or output) can be defined by configuration-pin level.

# Performance Data {#sect_xilinx_performance}

- Minimum cycle length: 400 us
- PReq-PRes Latency: 1 us
- Process data: 4 bytes input and 4 bytes output.
- There is 1 RPDOs and 1 TPDO available. Cross-traffic is disabled.

# Requirements {#sect_xilinx_requirements}

- Development Board Industrial Ethernet Kit (IEK, LX150T) with an
  ISMET FMC module or the Microbloard (micro, LX9).

- Xilinx Platform Cable USB II for JTAG programming (or similar).

- Xilinx ISE Design Suite 13.2 (Embedded Edition).

- Experiences with this development environment is required.

- POWERLINK network with managing node (MN)
  * openPOWERLINK managing node, e.g. Linux
  * B&R POWERLINK managing node
  * other POWERLINK managing node

# Hardware Setup {#sect_xilinx_hardware-setup}

- Setup for the Industrial Ethernet Kit (IEK, LX150T).
  * Download the user guide for the board from:
    (http://www.em.avnet.com/en-us/design/drc/Pages/Xilinx-Spartan-6-LX150T-Development-Kit.aspx)
  * Install jumper on JP4 pins 1-2.
  * Install jumper on JP8 pins 2-3.
  * Connect the Xilinx Platform Cable USB II to J9 and to the USB port of
    your PC.
  * Connect the USB cable to JR1 and to the USB port of your PC.
  * Connect the power supply to J16.
  * Connect the ISMNET module to FMC slot JX1.
  * Slide the SW11 Power switch on the LX150T board to the ON position.
  * On the ISMET FMC module do the following:
    + Install jumper on JP1 pins 2-3.
    + Install jumper on JP2 pins 1-2.
    + Install jumper on JP5 & JP10 pins 7-8.
    + Install jumper on JP4 & JP9.

- Setup for the Microboard (micro, LX9)
  * Download the user guide for the board from:
    (http://www.em.avnet.com/en-us/design/drc/Pages/Xilinx-Spartan-6-FPGA-LX9-MicroBoard.aspx)
  * Connect the Xilinx Platform Cable USB II to J6 and to the USB port of
    your PC.
  * Connect the USB cable to J3 and to the USB port of your PC.
  * Connect the Ethernet jack in J1 to the Ethernet port of your PC.

# How to build the binaries {#sect_xilinx_build}

1. Open the Xilinx Platform Studio (XPS) and set the 'Global Repository Search
   Path' to the POWERLINK IP-Core (ipcore) directory.\n
   `Edit` -> `Preferences` -> `Application` -> `Global Peripheral Repository Search`\n
   Path (e.g: $STACK_ROOT/fpga/ipcore/xilinx)
2. Open 'ISE Design Suite [64,32] Bit Command Prompt' depending on your
   platform.
3. Go to this subdirectory of openPOWERLINK:\n
   `$ cd Examples/arch/xilinx_microblaze/no_os/gnu/demo_cn_digitalio`
4. Edit the following parameters in file makefile.settings:\n
   `XPS_DIR=$STACK_ROOT/fpga/boards/xilinx/[avnet_lx150t,avnet_lx9]/design_microblaze_directIO-[plb,axi]`\n
   `BOARD_NAME=[lx150t,lx9]`\n
   `BUS_INTERFACE=[plb,axi]`
5. Run `make all` to generate the bitstream and build the ELF file (directIO.elf).\n
    `$ make all`
6. If you want to change to an other design please do a clean after changing
   the makefile.settings variables.\n
   `$ make clean`

# How to run the demo {#sect_xilinx_run}

1. Setup the POWERLINK network.
2. Run 'make download-bits' to download the bitstream to the target.\n
   `$ make download-bits`
3. Run 'make download-elf' to download the software to the target.\n
   `$ make download-elf`
4. Use a terminal program to see the debug output\n
    a. Baud rate: 9600\n
    b. Data Bits: 8\n
    c. Stop Bits: 1\n
    d. Parity: none\n
    e. Flow control: none
5. Have fun with openPOWERLINK on Xilinx!

# How to import the project into the SDK for debugging purposes {#sect_xilinx_debug}

1. Open the Xilinx Software Development Kit (SDK).
2. Import the 'demo_directIO' into the SDK by using\n
   `Import` -> `C/C++` -> `Existing Code As Makefile Project`\n
   Project Name: demo_directIO\n
   Code Location: `$STACK_ROOT/Examples/arch/xilinx_microblaze/no_os/gnu/demo_cn_digitalio`\n
   Language: C\n
   Compiler: Xilinx Microblaze GNU Toolchain\n
3. Create a new Hardware platform with:\n
   `File` -> `New` -> `Xilinx Hardware Platform Specification`\n
   Set the 'Target Hardware Specification' to\n
   `$STACK_ROOT/fpga/boards/xilinx/avnet_lx[150t,lx9]/design_microblaze_directIO-[plb,axi]/SDK/SDK_Export/hw`
4. Set the Repository Search Path with:\n
   `Xilinx Tools` -> `Repositories` -> `Local Repositories`\n
   (e.g: $STACK_ROOT/fpga/ipcore/xilinx)
5. Create a new Board Support Package with:\n
   `File` -> `New` -> `Xilinx Board Support Package`\n
   Select the previously created hardware platform as a reference.
6. Change the 'demo_directIO' software project to point to the board support
   package. (Right click on project -> `Change referenced BSP`).
7. `Run` -> `Debug As` -> `Lunch on Hardware` to start the debugger and step through
   the code.

# How to write the program to local flash {#sect_xilinx_flash}

In order to write the example to the non volatile memory a bootloader is needed
to read the program from the flash and write it to the external RAM. This
bootloader is available with the SDK and can be generated by simply creating a
new C project.

- Flash programming on the Industrial Ethernet Kit (IEK).
  * Generate the srec bootloader in SDK with:
    `File` -> `New` -> `Xilinx C Project (SREC bootloader)`
  * Set the correct flash offset to your blconfig.h. (e.g. 0x000000)
  * Generate a linker script and link all sections to the .boot memory.
  * Build the bootloader with optimization level to size. (-Os)
  * Merge the bootloader with your bitstream into a download.bit file with
    `Xilinx Tools` -> `Program FPGA` -> `Elf file to Initialize in Block RAM`
  * Open Xilinx iMPACT and click on `Create PROM file`.
    - Select Xilinx Flash/PROM and add xcf08p and xcf32p to your storage
      devices.
  * In the following window add the download.bit file to your flash image.
  * Generate the two flash images and write them to both flashes by using
    Xilinx iMPACT.
  * Inside the SDK use the `Xilinx Tools` -> `Flash Programmer` to write the
    software to the parallel flash.
  * Download the directIO.srec with the right offset to the parallel flash.

- Flash programming on the Microboard (LX9).
  * Generate the srec bootloader in SDK with:
    `File` -> `New` -> `Xilinx C Project (SREC bootloader)`
  * Download the tutorial 'Creating a Microblaze SPI Flash bootloader' from:
    http://www.em.avnet.com/en-us/design/drc/Pages/Xilinx-Spartan-6-FPGA-LX9-MicroBoard.aspx
  * Replace the bootloader.c with the one from the tutorial and select the
    library 'xilisf' with value 3 in your BSP.
  * Set the correct flash offset to your blconfig.h. (e.g. 0x060000)
  * Generate a linker script and link all sections to the .boot memory.
  * Build the bootloader with optimization level to size. (-Os)
  * Merge the bootloader with your bitstream into a download.bit file with
    `Xilinx Tools` -> `Program FPGA` -> `Elf file to Initialize in Block RAM`
  * Open Xilinx iMPACT and click on `Create PROM file`.
    - Select SPI Flash (Single FPGA) with a size of 128M.
    - Select `Add Non-Configuration Data File` in Step 3.
  * In the following window add the `download.bit` file and the `directIO.srec`
    file with the right offset to the flash image.
  * Generate the image and program it to the board by using iMPACT.

Finally your program should be written to non-volatile memory!

# Troubleshooting {#sect_xilinx_troubleshooting}

1. It is adviced to clean all generated files after switching from one demo to
   the other.\n
   `$ make clean_all`

