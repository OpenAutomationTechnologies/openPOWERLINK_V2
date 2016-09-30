openPOWERLINK Porting Guide {#page_porting-guide}
===========================

[TOC]

# General {#sect_porting_general}

This guide explains the platform dependencies of the openPOWERLINK stack and
analyzes the used strategies for supporting multiple platforms.
Modules which are necessary for a new porting can be identified using this guide.

# Platform dependencies {#sect_porting_dependencies}

The platform dependencies within the openPOWERLINK stack are realized via build
configurations (as described in \ref page_build and \ref page_platforms).
These configurations define specific implementations of general header files.
The different implementations for these function headers contain the target
specific functionalities.

## Target specific implementations {#sect_porting_target_specific}

According to the naming convention shown in [Coding style](\ref sect_coding_naming_files)
all target specific implementations provide their target system names hyphened
after the name of the implemented module.
These target specific implementations include and use any target dependent
functionality, the build configuration is providing the access to them.
The module's header file defines all target independent functions and types used
for the interface.

## Build configurations {#sect_porting_build}

As described in [CMake](\ref sect_build_cmake) the build utility CMake is used for
building different versions of the openPOWERLINK stack library.
The build configurations for different platforms consist of various sets of target
specific implementations.

These sets of target specific sources are defined within the common _CMake_ file
_src/cmake/stackfiles.cmake_.
This file includes groups defining source files for each module and each
implementation specialization.

The usage of such groups is done in project specific _CMake_ files composing
the stack variants for various platforms.
Different projects for various systems and platforms are located in the _stack/proj_
folder.
Each project is defined by it's own _CMake_ file and additional definition files.
The _CMake_ file _CMakeList.txt_ contains all used source groups and additional
definitions for building a specific project.
Additional defines e.g. for the preprocessor are located in a configuration file
named _oplkcfg.h_ also located in the project's folder.

# Platform dependencies in openPOWERLINK {#sect_porting_dependencies_oplk}

All platform dependencies are located within the following three groups of modules:

- Communication Abstraction Layer (\ref sect_porting_cal)
- Hardware Abstraction Layer (\ref sect_porting_hal)
- Abstract Memory Interface (\ref sect_porting_ami)

## CAL {#sect_porting_cal}

The [communication abstraction layer](\ref cal_layer) includes multiple modules
providing configurable implementations using different platform specific
functionalities.
The following listing shows the different modules with multiple implementations.
For further description of the implementations and the used functionalities the
specific documented files can be used.

- \ref cal_layer_ctrl
- \ref cal_layer_event
- \ref cal_layer_errhnd
- \ref cal_layer_dll
- \ref cal_layer_pdo

## HAL {#sect_porting_hal}

The hardware abstraction is done in a similar way as the _CAL_ by using specific
implementations of common header files.
The following listing includes all hardware dependent modules.
For further description of the implementations and the used functionalities the
documentation of the specific implementations can be used.

- target: The _target_ module is located in the _stack/src/arch_ folder and
provides target  functions controlling the defined IP address, default gateway,
global interrupt, status/error LEDs and a sleep function. (\ref target.h)

- circularbuffer: The _circularbuffer_ module is located in the _stack/src/common_
folder and provides an implementation for reading and writing variable sized data
segments from and to a circular buffer. (\ref module_lib_circbuf)

- memmap: The _memmap_ module is located in the _stack/src/common_ folder and
provides implementations for the handling of memory mapping.
This functionality is mostly used for mapping memory from the kernel layer to
the user layer which provides a memory handling. This reduces the necessity for
copying.
This module contains different implementations regarding the location of the
used memory and the handling of memory access. (\ref memmap.h)
These implementations use different functionalities providing a common interface
for memory mapping.
An empty implementation also provides the possibility of not using memory
mapping (\ref memmap-null.c).

- edrv: The _edrv_ module is located in the _stack/src/kernel_ folder and
provides implementations of various Ethernet drivers.
The driver provides functionalities for Ethernet communication and settings
regarding the underlying Ethernet controller. (\ref edrv.h)
This driver is implemented for various controllers and different platforms.

- hrestimer: The _hrestimer_ module is located in the _stack/src/kernel_ folder
and provides implementations for various high resolution timers.
(\ref module_hrestimer)

- veth: The _veth_ module is located in the _stack/src/kernel_ folder and
provides different implementations for a virtual Ethernet interface.
(\ref module_veth)

- sdo: The _sdo_ module is located in the _stack/src/user_ folder and provides
platform specific adaptations for the SDO protocol stack. Especially the
possibility for using existing platform functionalities can be used via the
_sdo-udp_ module.
(\ref user_layer_sdo)

- timer: The _timer_ module is located in the _stack/src/user_ folder and provides
different implementations for timer functionalities used by the user layer.
(\ref module_timeru)

- targetdefs: The _targetdefs_ folder is located within the _stack/include/oplk_
folder and contains different header files for specific targets.
These header files define preprocessor macros for common system specific
attributes and small functions regarding atomic operations and synchronization
functions.
The common header \ref targetsystem.h is located in the folder _stack/include/oplk_
and includes the according system and platform specific target file.

## AMI {#sect_porting_ami}

The [AMI](\ref module_ami) represents also a platform dependent module and
therefore provides implementations for little- and big-endian processors.
Additionally, an optimized implementation for x86 processors is provided.

None of these implementations use platform specific functionalities, but the
implementation varies for achieving the correct result when handling variables
consisting of multiple bytes.

# Porting of the openPOWERLINK Stack {#sect_porting_oplk}

Different levels of platform dependency can be achieved using the implementations
of various modules.
When porting the openPOWERLINK stack not every platform dependent module must be
adapted.
Available implementations might already provide support for the new platform or
are implemented in a generic way.
For porting the openPOWERLINK stack to a new platform using a minimal dependency
the following modules must be implemented:

 - edrv
 - hrestimer
 - target
 - targetdefs

This configuration targets an application which includes both user and kernel
layer and therefore does not require any porting of the \ref sect_porting_cal
modules.

All other modules listed in \ref sect_porting_dependencies_oplk provide
generic implementations which can be used.
The usage of generic implementations often provide less performance than an
optimized implementation using platform functionalities.

As an example of a port to a new platform the simulation interface can be
analyzed.
This interface implements the minimal configuration with a few additional
modules. (\ref module_sim)

## Porting of the User Layer {#sect_porting_user}

For porting the user layer to a new platform and using an existing kernel layer
configuration the required modules differ from the minimal configuration
mentioned in \ref sect_porting_oplk.
For such configurations the general system functionalities (target) and
definitions (targetdefs) must be provided.
Connecting the ported user layer to the existing kernel layer requires the
according implementation of the \ref sect_porting_cal modules.

## Porting examples {#sect_porting_examples}

The following table contains example scenarios which shows the different required
modules for different configurations.
The first two columns describe the scenarios with the layer which will be
ported and the existing functionalities which will be used without required
changes or implementations.

The following columns show the platform dependent modules and are marked according
to the necessity of a new implementation.

| Target Platform                                                                                                                      | target | targetdefs | edrv | hrestimer | user timer | sdo-udp | veth | memmap | circularbuffer | AMI | User Layer CAL | Kernel Layer CAL |
|--------------------------------------------------------------------------------------------------------------------------------------|:------:|:----------:|:----:|:---------:|:----------:|:-------:|:----:|:------:|:--------------:|:---:|:--------------:|:----------------:|
| Single application running on bare metal                                                                                             |    X   |      X     |   X  |     X     |      O     |    O    |   O  |    P   |        P       |  P  |        P       |         P        |
| User application running on bare metal connected\n via host interface (hostif) to dedicated Hardware\n running existing Kernel Layer |    X   |      X     |      |           |      O     |    O    |   O  |    P   |        P       |  P  |        X       |                  |
| Single application running on new OS providing\n memory mapping and network stack                                                    |    X   |      X     |   X  |     X     |      O     |    O    |   O  |    P   |        P       |  P  |        P       |         P        |
| User application running on new OS providing\n memory mapping and network stack                                                      |    X   |      X     |      |           |      O     |    O    |   O  |    P   |        P       |  P  |        X       |                  |
| Kernel module running on new OS providing\n memory mapping and network stack                                                         |    X   |      X     |   X  |     X     |            |         |      |    P   |        P       |  P  |                |         X        |
| Simulated application within simulation\n environment running on a linux machine                                                     |    X   |            |   X  |     X     |      O     |    O    |   O  |    O   |        O       |  O  |        O       |         O        |


X ... New implementation required \n
P ... Generic implementation exists, new implementation could lead to performance
optimizations \n
O ... Generic implementation exists, new implementation optional
