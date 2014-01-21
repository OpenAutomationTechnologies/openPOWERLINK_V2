openPOWERLINK Components {#page_components}
========================

[TOC]

A POWERLINK node using the openPOWERLINK stack is composed of the
following components:

1. openPOWERLINK Stack Library

2. openPOWERLINK Driver (*Required only if using a separate driver!*)

3. Application


# openPOWERLINK Stack Libraries {#sect_components_stacklib}

The openPOWERLINK stack is compiled into a static or dynamic library which
is linked to the application. Depending on the used platform different
openPOWERLINK libraries are available. There are different distinctions of
stack libraries:

- MN and CN libraries
- Application, driver and complete libraries
- Debug and Release libraries

## MN and CN Libraries

Any MN is always able to act as a CN as well. So any MN library can also be
used for implementing a CN. However, to reduce code size and increase
performance a CN library can be compiled containing only the features required
by a CN.

## Application and Driver Libraries

The openPOWERLINK stack is split into a user layer (application part) and a
kernel layer (driver part). The user layer contains the application-centric code such
as the object dictionary and network management functions while the kernel layer contains
the time critical parts (e.g. the data link layer). Depending on which code is compiled
into the library the following distinction is made:

- __Complete library__:
        A complete library contains both, the user and the kernel part and is
        linked to an application to implement a POWERLINK node. In this case,
        no driver is needed and the whole stack is linked to the application.

- __Application library__:
        An openPOWERLINK application library contains only the user layer
        (application part) of the stack. It is linked to an application and
        additionally requires a driver in order to implement a POWERLINK node.

- __Driver library__:
        An openPOWERLINK driver library contains only the kernel layer of the
        openPOWERLINK stack and is linked to driver source code to implement
        an openPOWERLINK driver. It requires an application in order to
        implement a POWERLINK node.

![](\ref openpowerlink_libraries.png)

## Debug and Release Libraries

The stack can be created as either debug or release library. A release library
is compiled with optimizations and does not contain any debug trace code.
A debug library is compiled without optimizations and contains debug trace code.
The debug level can be defined by a CMake configuration value. Debug libraries
contain the postfix "_d".

# openPOWERLINK Driver {#sect_components_driver}

The openPOWERLINK stack can use a separate driver for its kernel layer
modules. This may be required in order to get access to the hardware (e.g. a Linux
kernel driver) or to increase performance (e.g. by running the kernel part on a
separate processor).

Available drivers can be found in the __drivers__ directory.

# Application {#sect_components_application}

The application is the part of a POWERLINK device that implements the actual
functionality. It makes use of the openPOWERLINK stack API for transferring
data accross a POWERLINK network.

The openPOWERLINK stack distribution contains a set of demo applications which
demonstrate how a POWERLINK application can be implemented using the openPOWERLINK
stack.

## Available demo applications {#sect_components_demos}

### Console MN demo {#sect_components_demo_mn_console}

This demo implements a POWERLINK MN including a Configuration Manager (CFM).
It is realized as a console application and is intended for machines where
no graphical user interface is available.

The demo can be found in: `apps/demo_mn_console`

### Console CN demo {#sect_components_demo_cn_console}

This demo implements a POWERLINK CN with virtual digital I/Os according to
the CANopen device profile 401. It is implemented as console application, so
the I/Os are represented via keyboard input and screen output.

It is located in: `apps/demo_cn_console`

### QT MN demo  {#sect_components_demo_mn_qt}

The QT demo application implements a POWERLINK MN including a Configuration
Manager (CFM). It uses the QT framework to realize a platform-independent
graphical user interface (GUI).

The demo can be found in: `apps/demo_mn_qt`

### Embedded MN demo  {#sect_components_demo_mn_embedded}

This demo implements a POWERLINK MN with CFM for embedded devices (i.e. devices
without screen and keyboard).

It is located in: `apps/demo_mn_embedded`

### Embedded CN demo  {#sect_components_demo_cn_embedded}

This demo implements a POWERLINK CN with digital I/Os according to the CANopen
device profile 401. It is implemented as an embedded application and makes use
of GPIOs available on the according hardware.

It is located in: `apps/demo_cn_embedded`
