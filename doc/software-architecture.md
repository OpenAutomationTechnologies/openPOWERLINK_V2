Software Architecture {#page_software-architecture}
=====================

[TOC]

The following picture shows the software architecture of the openPOWERLINK
stack.

![](\ref openpowerlink_architecture.png)

The openPOWERLINK stack is separated into a user layer and a kernel layer. The user
layer contains the higher level parts wheras the kernel layer contains the lower
level and time critical parts. The communication between user and kernel layer
is performed by a communication abstraction layer (CAL). 

- \ref user_layer
- \ref kernel_layer
- \ref cal_layer
- \ref modules_common



