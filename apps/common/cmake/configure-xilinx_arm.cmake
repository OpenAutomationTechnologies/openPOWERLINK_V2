################################################################################
#
# Xilinx ARM configuration options for openPOWERLINK stack
#
# Copyright (c) 2014, Kalycito Infotech Private Limited
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
SET(CMAKE_MODULE_PATH "${OPLK_ROOT_DIR}/cmake" ${CMAKE_MODULE_PATH})
INCLUDE(geneclipsefilelist)
INCLUDE(geneclipseincludelist)
INCLUDE(setarma9boardconfig)

################################################################################
# Path to the hardware library folder of your board example
SET(CFG_HW_LIB_DIR ${OPLK_ROOT_DIR}/hardware/lib/${SYSTEM_NAME_DIR}/${SYSTEM_PROCESSOR_DIR}/xilinx-z702/mn-dual-shmem-gpio
        CACHE PATH "Path to the hardware library folder of your demo application")

# Include demo specific settings file
SET_BOARD_CONFIGURATION(${CFG_HW_LIB_DIR})

################################################################################
# Set variables
SET(ARCH_EXE_SUFFIX ".elf")
SET(ARCH_INSTALL_POSTFIX ${CFG_DEMO_BOARD_NAME}/${CFG_DEMO_NAME})
SET(XIL_TOOLS_DIR ${TOOLS_DIR}/xilinx-arm)

################################################################################
# Stack configuration
################################################################################
SET(CFG_KERNEL_DUALPROCSHM ON CACHE INTERNAL
    "Link to kernel running on separate processor interfaced through shared memory")
UNSET(CFG_KERNEL_STACK_USERSPACE_DAEMON CACHE)
UNSET(CFG_KERNEL_STACK_DIRECTLINK CACHE)
