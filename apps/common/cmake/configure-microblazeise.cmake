################################################################################
#
# Microblaze ISE configuration options for openPOWERLINK stack
#
# Copyright (c) 2014, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
# Copyright (c) 2016, Kalycito Infotech Private Limited
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

################################################################################
# Handle includes
SET(CMAKE_MODULE_PATH "${OPLK_BASE_DIR}/cmake" ${CMAKE_MODULE_PATH})
INCLUDE(geneclipsefilelist)
INCLUDE(geneclipseincludelist)
INCLUDE(setmicroblazeiseboardconfig)
INCLUDE(listdir)

################################################################################
# Path to the hardware library folder of your board example
SET(CFG_HW_LIB_PATH ${OPLK_BASE_DIR}/hardware/lib/${SYSTEM_NAME_DIR}/${SYSTEM_PROCESSOR_DIR}
        CACHE PATH "Path to the hardware library folder")

# Get subdirectories (board/demo)
LIST_SUBDIRECTORIES(HW_BOARD_DEMOS ${CFG_HW_LIB_PATH} 2)

SET(CFG_HW_LIB avnet-s6plkeb/cn-single-gpio CACHE STRING
    "Subfolder of hardware board demo")
SET_PROPERTY(CACHE CFG_HW_LIB PROPERTY STRINGS ${HW_BOARD_DEMOS})

SET(CFG_HW_LIB_DIR ${CFG_HW_LIB_PATH}/${CFG_HW_LIB})

################################################################################
# Include board specific settings file
SET_BOARD_CONFIGURATION(${CFG_HW_LIB_DIR})

################################################################################
# Set variables
SET(ARCH_EXE_SUFFIX ".elf")
SET(ARCH_INSTALL_POSTFIX ${CFG_DEMO_BOARD_NAME}/${CFG_DEMO_NAME})
SET(XIL_TOOLS_DIR ${TOOLS_DIR}/xilinx-microblaze)

################################################################################
# Stack configuration
SET(CFG_BUILD_KERNEL_STACK "Link to Application"
    CACHE STRING "Configure how to build the kernel stack")

SET(KernelStackBuildTypes
    "Link to Application;PCP Daemon Host-Interface;None"
    CACHE INTERNAL
    "List of possible kernel stack build types")

SET_PROPERTY(CACHE CFG_BUILD_KERNEL_STACK
             PROPERTY STRINGS ${KernelStackBuildTypes})

IF (CFG_BUILD_KERNEL_STACK STREQUAL "Link to Application")

    SET(CFG_KERNEL_STACK_DIRECTLINK ON CACHE INTERNAL
         "Link kernel stack directly into application (Single process solution)")
    UNSET(CFG_KERNEL_STACK_PCP_HOSTIF_MODULE CACHE)

ELSEIF (CFG_BUILD_KERNEL_STACK STREQUAL "PCP Daemon Host-Interface")

    SET(CFG_KERNEL_STACK_PCP_HOSTIF_MODULE ON CACHE INTERNAL
         "Build kernel stack as PCP daemon (dual processor)")
    UNSET(CFG_KERNEL_STACK_DIRECTLINK CACHE)

ELSEIF (CFG_BUILD_KERNEL_STACK STREQUAL "None")
    UNSET(CFG_KERNEL_STACK_PCP_HOSTIF_MODULE CACHE)
    UNSET(CFG_KERNEL_STACK_DIRECTLINK CACHE)

ENDIF (CFG_BUILD_KERNEL_STACK STREQUAL "Link to Application")
