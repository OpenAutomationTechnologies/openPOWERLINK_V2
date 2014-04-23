Building openPOWERLINK Hardware Platforms {#page_build_hardware}
==============================

[TOC]

# Building the Hardware Platform {#sect_build_hardware_build}

If the target platform is an embedded processor the openPOWERLINK stack needs
hardware near libraries like a board support package to run properly. In case
of an FPGA with a soft-core processor, it is also necessary to generate the
configuration for the FPGA.

This chapter describes the build steps needed to build the FPGA configuration
and all hardware near drivers.

**NOTE:** In order to be able to debug the final user application, both, the
Debug and the Release driver versions should be created.

## Target Xilinx Microblaze {#sect_build_stack_build_microblaze}

Execute the following steps below to generate the FPGA configuration and all
hardware near drivers for the target Xilinx Microblaze.
* Open the `Xilinx Platform Studio (XPS)` and set the `Global Repository Search
  Path` to the openPOWERLINK stack IP-Core repository.\n
  `Edit` -> `Preferences` -> `Application` -> `Global Peripheral Repository Search Path`
  (e.g: `$OPLK_ROOT/hardware/ipcore/xilinx`).\n
  (There is no need to open any project inside the **XPS**!)

* Open a shell where the Xilinx ISE 14.7 Toolchain is configured:
  - On a Windows host platform open the `ISE Design Suite [64,32] Bit Command
    Prompt`.
  - On a Linux host platform execute the script `<ISE_ROOT_DIR>/settings[32,64].sh>`
    to configure your current shell.

* Initialize the hardware platform build system

      > cd openPOWERLINK/hardware/build/xilinx-microblaze
      > cmake -G"Unix Makefiles" -DCMAKE_TOOLCHAIN_FILE=../../../cmake/toolchain-xilinx-microblaze-gnu.cmake ../..

   After this command CMake will search for available hardware platforms and
   report all found platforms by the following messages:

       Found hardware platform: DEMO_S6PLKEB_CN_SINGLE_GPIO set to OFF!
       Found hardware platform: DEMO_[BOARD_NAME]_[DEMO_NAME] set to OFF!

* Building all hardware platforms with all driver libraries set to debug

      > cmake ../.. -DCMAKE_BUILD_TYPE=Debug -DDEMO_S6PLKEB_CN_SINGLE_GPIO=ON -DDEMO_[BOARD_NAME]_[DEMO_NAME]=ON
      > make
      > make install

   This will build the hardware platform for the demos `DEMO_S6PLKEB_CN_SINGLE_GPIO`
   and `DEMO_[BOARD_NAME]_[DEMO_NAME]`.

* Building all hardware platforms with all driver libraries set to release

      > cmake ../.. -DCMAKE_BUILD_TYPE=Release -DDEMO_S6PLKEB_CN_SINGLE_GPIO=ON -DDEMO_[BOARD_NAME]_[DEMO_NAME]=ON
      > make
      > make install

The default hardware platform installation path is:
`<openPOWERLINK_DIR>/hardware/lib/generic/microblaze/<BOARD_NAME>/<DEMO_NAME>`
