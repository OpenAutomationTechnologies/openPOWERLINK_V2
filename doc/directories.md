Directory structure {#page_directories}
===================

[TOC]

The following page describes the directory structure of the openPOWERLINK stack.

# Main directories {#sect_directories_main}

Directory                     | Description
----------------------------- | -----------------------------------------------
cmake                         | Files for the CMake build tool
doc                           | Documentation of the openPOWERLINK stack
apps                          | Demo applications for evaluating the openPOWERLINK stack
fpga                          | FPGA specific stuff like IP cores and VHDL code
include                       | Include files needed for applications which link to the openPOWERLINK stack
contrib                       | Additional libraries used by the openPOWERLINK stack
stack                         | The actual openPOWERLINK stack
staging                       | Unstable, unclean and untested source code
tools                         | Miscelaneous tools and utilities
unittests                     | CUnit Unit test framework

## Examples {#sect_directories_examples}
A set of demo applications is included in the openPOWERLINK distribution. The
demos contain examples of how to implement an MN or CN using the openPOWERLINK
stack.

Directory                     | Description
----------------------------- | -----------------------------------------------
demo_cn_console               | Console application which implements a CN
demo_mn_console               | Console application which implements an MN
demo_mn_qt                    | QT based application which implements an MN
common                        | Contains common code which is used by all demos
arch                          | Contains architecture specific code
openCONFIGURATOR_projects     | Contains the [openCONFIGURATOR](http://sourceforge.net/p/openconf) projects used by the demos


## Libraries {#sect_directories_libraries}
The __libs__ directory contains libraries used by the openPOWERLINK stack.

Directory                     | Description
----------------------------- | -----------------------------------------------
circbuf                       | Circular buffer library
console                       | Utilities for console input and output
getopt                        | Command line parser
hostif                        | Host interface library
omethlib                      | openMAC controller library
pcap                          | libPcap library implementations
timer                         | Timer library
trace                         | Functions for handling trace output

## Object dictionaries {#sect_directories_objdict}

The directory contains the CANopen object dictionaries used by the stack.

Directory                     | Description
----------------------------- | -----------------------------------------------
CiA302-4_MN                   | Object dictionary according to the CiA profile 302-4 for MN implementations
CiA401_CN                     | Object dictionary according to the CiA 401 profile for I/O CN implementations

## Stack sources {#sect_directories_stack}

This directory contains the whole openPOWERLINK stack sources.

Directory                     | Description
----------------------------- | -----------------------------------------------
include                       | Stack internal include files
make                          | Stack library and driver build and configuration files
src                           | Stack sources
src/arch                      | Architecture specific helper functions
src/common                    | Common sources used by the user and the kernel layer
src/user                      | User layer stack sources
src/kernel                    | Kernel layer stack sources
