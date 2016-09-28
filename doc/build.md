Building openPOWERLINK {#page_build}
======================

[TOC]

# CMake {#sect_build_cmake}

For building openPOWERLINK the build utility CMake
([http://www.cmake.org](http://www.cmake.org)) is used. CMake controls the
software compilation process using simple platform and compiler independent
configuration files.

A CMake version 2.8.7 or higher is required for building openPOWERLINK!

__NOTE:__ Currently, CMake is not used for all platforms. There are some platforms
where platform specific build files are provided (e.g. Makefiles or Eclipse
project files). Please refer to the [platform documentation](\ref page_platforms).

## Out-of-Source Builds {#sect_build_cmake_outofsource}

CMake supports out-of-source builds. Therefore, all generated files are located
in a separate build directory. This keeps your sources clean.

## Interactive Mode {#sect_build_cmake_interactive}

For the configuration of different build options of the openPOWERLINK stack,
CMake can be called in an interactive mode. There are three possible ways for
configuring your build options interactively:

* `cmake-gui`

  This is the most comfortable way for configuring the build options. cmake-gui
  is a graphical user interface for CMake.

  If using the cmake-gui, the build options are selected in the GUI. After
  setting the desired options press __Configure__. New options might appear
  according to the current selection. New options will be marked in _red_.
  Further configuration settings may be changed and accepted by pressing
  __Configure__ again. If there are no red-marked options, __Generate__ writes
  the build files.

* `ccmake`

  ccmake is a curses based user interface for cmake.

* `cmake -i`

  If cmake is called with option -i you can interactively select your build
  options on the console.

## Executing CMake {#sect_build_cmake_execute}

For configuration and generation of the build files you have to enter
the build directory and executing CMake with the path to the source directory
as parameter. For example:

        > cd build
        > cmake ..

## Cross Compilation {#sect_build_cmake_crosscompile}

CMake allows cross compiling for another platform. Therefore "cross platform
toolchain files" will be delivered in the `cmake` directory. To configure
the build to use a cross compiler and build environment you have to select
the appropriate cross toolchain file when calling cmake.

The following toolchain files are delivered in the main cmake directory:

| Platform                          | Toolchain file                            |
| --------------------------------- | ----------------------------------------- |
| Xilinx Microblaze                 | toolchain-xilinx-microblaze-gnu.cmake     |
| Xilinx ARM (Zynq) Linux           | toolchain-xilinx-arm-linux-eabi-gnu.cmake |
| Altera ARM (Cyclone V SoC) non-OS | toolchain-altera-c5socarm-eabi-gnu.cmake  |

The toolchain file must be specified with the CMake option __CMAKE_TOOLCHAIN_FILE__.

__For example:__

Configuring cross compilation for the Xilinx Microblaze platform:

    > cmake -DCMAKE_TOOLCHAIN_FILE=<OPENPOWERLINK_DIR>/cmake/toolchain-xilinx-microblaze-gnu.cmake <SOURCE_DIR>

# Build Steps {#sect_build_steps}

The following build steps are necessary to build an openPOWERLINK solution:

* [Build the hardware platform](\ref page_build_hardware)
  (This step is only necessary for embedded targets)
* [Build the openPOWERLINK stack libraries](\ref page_build_stack)
* [Build the necessary openPOWERLINK drivers](\ref page_build_drivers)
  (This step is only necessary for split stack versions)
* [Build your application (or a delivered demo application)](\ref page_build_demos)

# Platform Specific Build Information {#sect_build_platform}

Please have a look into the documentation for the related [platform](\ref page_platforms)
for further information on how to build openPOWERLINK for this platform.
