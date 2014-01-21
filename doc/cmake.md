Building with CMake {#page_cmake}
===================

[TOC]

For building openPOWERLINK the build utility CMake
([http://www.cmake.org](http://www.cmake.org)) is used. CMake controls the
software compilation process using simple platform and compiler independent
configuration files. Depending on the architecture different build
files can be created. On Linux CMake may create:

* Unix Makefiles (tested)
* Eclipse CDT4 - Unix Makefiles
* Kdevelop3 - Unix Makefiles
* CodeBlocks - Unix Makefiles

# Executing CMake {#sect_cmake_exec}

The configuration of different build options of the openPOWERLINK stack will
be done through the cmake interface.

There are three possible ways for configuring your build options

* `cmake-gui`

  This is the most comfortable way for configuring the build options. cmake-gui
  is a graphical user interface for CMake.

* `ccmake`

  ccmake is a curses based user interface for cmake.

* `cmake -i`

  If cmake is called with option -i you can interactively select your build
  options.




# Out-of-Source Builds {#sect_cmake_outofsource}

CMake support out-of-source builds. Therefore, all generated files are located
in a seperate build directory which keeps your sources clean. It is recommended
to use a separate build directory for building the openPOWERLINK stack and the
demo applications.

