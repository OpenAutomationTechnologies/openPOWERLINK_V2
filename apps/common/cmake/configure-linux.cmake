################################################################################
#
# Linux configuration options for openPOWERLINK demo applications
#
# Copyright (c) 2014, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
# Copyright (c) 2016, kalycito Infotech Private Limited
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the copyright holders nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
################################################################################

SET(CFG_BUILD_KERNEL_STACK "Link to Application"
    CACHE STRING "Configure how to build the kernel stack")

SET(KernelStackBuildTypes
    "Link to Application;Linux Userspace Daemon;Linux Kernel Module;Kernel stack on PCIe card;Kernel stack on Zynq PCP;None"
    CACHE INTERNAL
    "List of possible kernel stack build types")

SET_PROPERTY(CACHE CFG_BUILD_KERNEL_STACK
             PROPERTY STRINGS ${KernelStackBuildTypes})

IF (CFG_BUILD_KERNEL_STACK STREQUAL "Link to Application")

    SET (CFG_KERNEL_STACK_DIRECTLINK ON CACHE INTERNAL
         "Link kernel stack directly into application (Single process solution)")
    UNSET (CFG_KERNEL_STACK_USERSPACE_DAEMON CACHE)
    UNSET (CFG_KERNEL_STACK_KERNEL_MODULE CACHE)
    UNSET (CFG_KERNEL_STACK_PCIE_INTF CACHE)
    UNSET (CFG_KERNEL_STACK_ZYNQ_INTF CACHE)

ELSEIF (CFG_BUILD_KERNEL_STACK STREQUAL "Linux Userspace Daemon")

    SET (CFG_KERNEL_STACK_USERSPACE_DAEMON ON CACHE INTERNAL
         "Build kernel stack as Linux userspace daemon")
    UNSET (CFG_KERNEL_STACK_DIRECTLINK CACHE)
    UNSET (CFG_KERNEL_STACK_KERNEL_MODULE CACHE)
    UNSET (CFG_KERNEL_STACK_PCIE_INTF CACHE)
    UNSET (CFG_KERNEL_STACK_ZYNQ_INTF CACHE)

ELSEIF (CFG_BUILD_KERNEL_STACK STREQUAL "Linux Kernel Module")

    SET (CFG_KERNEL_STACK_KERNEL_MODULE ON CACHE INTERNAL
         "Build kernel stack as Linux kernelspace module")
    UNSET (CFG_KERNEL_STACK_USERSPACE_DAEMON CACHE)
    UNSET (CFG_KERNEL_STACK_DIRECTLINK CACHE)
    UNSET (CFG_KERNEL_STACK_PCIE_INTF CACHE)
    UNSET (CFG_KERNEL_STACK_ZYNQ_INTF CACHE)

ELSEIF (CFG_BUILD_KERNEL_STACK STREQUAL "Kernel stack on PCIe card")

    SET (CFG_KERNEL_STACK_PCIE_INTF ON CACHE INTERNAL
         "Build kernel stack on an external PCIe device")
    UNSET (CFG_KERNEL_STACK_USERSPACE_DAEMON CACHE)
    UNSET (CFG_KERNEL_STACK_DIRECTLINK CACHE)
    UNSET (CFG_KERNEL_STACK_KERNEL_MODULE CACHE)
    UNSET (CFG_KERNEL_STACK_ZYNQ_INTF CACHE)

ELSEIF (CFG_BUILD_KERNEL_STACK STREQUAL "Kernel stack on Zynq PCP")

    SET (CFG_KERNEL_STACK_ZYNQ_INTF ON CACHE INTERNAL
         "Build kernel stack on an external zynq PCP device")
    UNSET (CFG_KERNEL_STACK_USERSPACE_DAEMON CACHE)
    UNSET (CFG_KERNEL_STACK_DIRECTLINK CACHE)
    UNSET (CFG_KERNEL_STACK_KERNEL_MODULE CACHE)
    UNSET (CFG_KERNEL_STACK_PCIE_INTF CACHE)

ELSEIF (CFG_BUILD_KERNEL_STACK STREQUAL "None")
    UNSET (CFG_KERNEL_STACK_USERSPACE_DAEMON CACHE)
    UNSET (CFG_KERNEL_STACK_KERNEL_MODULE CACHE)
    UNSET (CFG_KERNEL_STACK_DIRECTLINK CACHE)
    UNSET (CFG_KERNEL_STACK_PCIE_INTF CACHE)
    UNSET (CFG_KERNEL_STACK_ZYNQ_INTF CACHE)

ENDIF (CFG_BUILD_KERNEL_STACK STREQUAL "Link to Application")



