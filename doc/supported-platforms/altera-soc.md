openPOWERLINK on Altera Cyclone V SoC {#page_platform_altera-soc}
==================================

[TOC]

# Introduction {#sect_altera-soc_introduction}

openPOWERLINK on Altera Cyclone V SoC runs the time-critical kernel
part of the stack on a Nios II softcore processor in the programmable
logic (PL) section. The user part of the stack runs on the
ARM Cortex A9 Core 0 in the hard processor system (HPS) section.

The Ethernet interface is built with the POWERLINK-optimized controller
openMAC, which enables e.g. low SoC jitters. Additionally, a hub IP-Core
is provided enabling daisy chained networks.

The two processors communicate with each other using a shared memory interface.

# Contents {#sect_altera-soc_contents}

The following MN hardware design is available for the Altera Cyclone V SoC board:

- PCP in PL and host in ARM Cortex A9 core 0 of HPS (driver and application): \n
  `hardware/boards/altera-c5soc/mn-soc-shmem-gpio`

The software projects for the PCP and the host processor are split into
`drivers` and `apps`:
- PCP driver daemon: \n
    `drivers/altera-nios2/drv_daemon/build`
- Host "demo_mn_embedded": \n
    `apps/demo_mn_embedded/build/altera-c5socarm`

# Requirements {#sect_altera-soc_requirements}

- Altera Quartus II v14.0.200 (Web Edition is also possible)
  - <https://www.altera.com/downloads/download-center.html#>
- Experience with this development environment is required.
- Altera Cyclone V SoC Development Board.
- [CMake] (\ref sect_build_cmake)
- Terminal software to display debug prints received from the ARM Cortex A9
  through a serial interface.

# Hardware setup {#sect_altera-soc_hardware}

- Setup for Altera Cyclone V SoC Development Board.
 - To set the jumper and switch settings refer to
  [Altera Website](https://www.altera.com/literature/manual/rm_cv_soc_dev_board.pdf)
 (*Figure 3–1. Switch Locations and Default Settings, page 12*)
 - Connect any one port of the "Dual Ethernet port" on the
   Cyclone V SoC Development Board to the POWERLINK network.

# How to build the binaries {#sect_altera-soc_build}

## Generating the hardware

The Altera Quartus II project for Cyclone V SoC is located in
`hardware/boards/altera-c5soc/mn-soc-shmem-gpio/quartus.`\n

Steps 1-5 can be carried out by executing `$ make all` in a
"Nios II Command Shell".

1. Open the Quartus project file `mn-soc-shmem-gpio.qpf` with Altera Quartus II.
2. Open Qsys via the menu *Tools* -> *Qsys*.
3. Open the Qsys file (`mnSocShmemGpio.qsys`) in the demo directory and press
   the button *Generate* to generate the host and PCP system.
4. Close Qsys when the generation has finished (shown as information output).
5. Start the compilation in the Quartus II window via menu *Processing* ->
   *Start Compilation*.

## Building the driver daemon

To build the driver daemon for Altera Cyclone V SoC follow the steps below:

1. Open "Nios II Command Shell".
2. Use the software design available in the openPOWERLINK subdirectory \n
   `drivers/altera-nios2/drv_daemon/build`. \n
3. Run the script *create-this-app* to create the Makefile for the driver daemon. \n
   `$ ./create-this-app --board ../../../../hardware/boards/altera-c5soc/mn-soc-shmem-gpio` \n
4. Run make to build the ELF file after changing the sources. \n
   `$ make`
5. Run following command to generate the daemon and FPGA boot files. \n
   `$ make fpgaboot`

Some additional commands to rebuild, debug and get help information.
* After Nios II Design is changed inside Qsys, rebuild the Makefile using following command. \n
   `$ ./create-this-app --rebuild --board ../../../../hardware/boards/altera-c5soc/mn-soc-shmem-gpio`
* If you want to debug the demo by enabling prints in NIOSII, use the `--debug` option. \n
   `$ ./create-this-app --debug --board ../../../../hardware/boards/altera-c5soc/mn-soc-shmem-gpio`
* The script's help message can be printed with `--help`. \n
   `$ ./create-this-app --help`

## Building the embedded MN demo application

To build the embedded MN demo application for Altera Cyclone V SoC refer
to the [generic build instructions](\ref page_build) for using CMake and execute these steps:

* [Build hardware libraries and preloader] (\ref sect_build_hardware_build_altera_arm)
* [Build stack libraries] (\ref sect_build_stack_build_c5socarm-altera)
* [Build your application (or a delivered demo application)](\ref sect_build_demos_build_altera_arm)

# How to run the demo {#sect_altera-soc_run}

The MN embedded demo can be started on the Altera Cyclone V SoC board using the SD card boot mode.
Follow the steps below to start the MN demo on the board:

1. Open an "SoC command shell".
2. Prepare an SD card using the following commands after connecting it:\n
   - `cd <openPOWERLINK_directory>\bin\generic\alterac5arm\altera-c5soc\mn-soc-shmem-gpio`
   - `dd if=<Altera_installation_dir>/embedded/embeddedsw/socfpga/prebuilt_images/sd_card_linux_boot_image.img
      of=/dev/<sd_card_drive>`
   - `alt-boot-disk-util.exe -p preloader-mkpimage.bin -a write -d <sd_card_path> bs=1M`
3. Copy the `BOOT.bin` and `fpga.rbf` from `<openPOWERLINK_directory>\bin\generic\alterac5arm\altera-c5soc\mn-soc-shmem-gpio`
   to the SD card.
4. Boot the Altera Cylcone V SoC from the prepared SD card.
5. Enjoy running the POWERLINK network.

# How to import the project into Nios II Software Build Tools for Eclipse for debugging purposes {#sect_altera-socnios_import}

Requirement: Steps in the section *How to build the binary* are completed.

1. Start the Nios II Software Build Tools for Eclipse.\n
   Note: It is recommended to place the workspace location outside the openPOWERLINK directory.
2. Select menu *File -> Import...*
3. Select the import source *Nios II Software Build Tools Project* ->
   *Import Nios II Software Build Tools Project*
4. Browse to `drivers/altera-nios2/drv_daemon/build` (via the button
   *Browse...*)
5. Set the project name to `drv_daemon`.
6. Press the button *Finish*.

# How to import the project into Eclipse for DS5 for debugging purposes {#sect_altera-socarm_import}

Requirement: Steps in the section *How to build the binary* are completed.

1. Start an "SoC embedded shell".
2. Start the Eclipse for DS5 using following command: \n
   `$ eclipse`
3. Select menu *File -> Import...*
4. Select the import source *General* -> *Existing Project into Workspace*
5. Browse to `<openPOWERLINK_directory>` (via the button *Browse...*)
6. Select the `demo_mn_embedded` project.
7. Press the button *Finish*.

# Troubleshooting {#sect_altera-soc_trouble}

- Frame CRC error for the transmitted frames from PHY

  The PHYs of Altera Cyclone V SoC revision D development board are configured to
  operate in synchronous mode, by boot strapping resistors. For POWERLINK operation,
  the PHYs need to be configured in asynchronous mode.

  To enable asynchronous operation the boot strapping resistor `R522` needs to be removed.
  For more information refer to [Rocket boards website](http://rocketboards.org/foswiki/view/Documentation/CVSoCDevelopmentBoardRevisionInfo)

  In order to avoid the modifications on the development board, the hardware design
  is updated to choose PHY transmission modes based on the revision of the development board.
  The board revision can be specified using the __gBoardRev__ parameter in the toplevel.vhd file.
  By default the revision is set to __"D"__ and is tested on the same.
