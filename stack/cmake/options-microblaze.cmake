################################################################################
#
# CMake options for openPOWERLINK stack on Xilinx/Microblaze Vivado
#
# Copyright (c) 2014, B&R Industrial Automation GmbH
# Copyright (c) 2018, Kalycito Infotech Private Limited
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

MESSAGE(STATUS "Adding CMake configuration options for Microblaze")

################################################################################
# Handle includes
SET(CMAKE_MODULE_PATH "${OPLK_BASE_DIR}/cmake" ${CMAKE_MODULE_PATH})
INCLUDE(geneclipsefilelist)
INCLUDE(geneclipseincludelist)

################################################################################
# Set Paths
SET(XIL_HW_LIB_DIR ${OPLK_BASE_DIR}/hardware/lib/${SYSTEM_NAME_DIR}/${SYSTEM_PROCESSOR_DIR})
SET(XIL_TOOLS_DIR ${TOOLS_DIR}/xilinx-microblaze)

################################################################################
# Options for MN libraries
OPTION(CFG_COMPILE_LIB_MNDRV_DUALPROCSHM   "Compile openPOWERLINK MN driver library using dual procesor shared memory" OFF)

# Options for CN libraries
OPTION(CFG_COMPILE_LIB_CNDRV_DUALPROCSHM   "Compile openPOWERLINK CN driver library using dual procesor shared memory" OFF)

################################################################################
# Add library subdirectories and hardware library path

# MN libraries
IF (CFG_COMPILE_LIB_MNDRV_DUALPROCSHM)
    # Path to the MN hardware library folder of your board example
    SET(CFG_COMPILE_LIB_MN_HW_LIB_DIR ${XIL_HW_LIB_DIR}/xilinx-z702/mn-dual-shmem-gpio
            CACHE PATH "Path to the hardware library folder for the dual processor MN library")

    ADD_SUBDIRECTORY(proj/generic/liboplkmndrv-dualprocshm)
# CN libraries
ELSEIF (CFG_COMPILE_LIB_CNDRV_DUALPROCSHM)
    # Path to the CN hardware library folder of your board example
    SET(CFG_COMPILE_LIB_CN_HW_LIB_DIR ${XIL_HW_LIB_DIR}/xilinx-z702/cn-dual-shmem-gpio
            CACHE PATH "Path to the hardware library folder for the dual processor CN library")

    ADD_SUBDIRECTORY(proj/generic/liboplkcndrv-dualprocshm)
ELSE ()
    UNSET(CFG_COMPILE_LIB_MN_HW_PATH CACHE)
    UNSET(CFG_COMPILE_LIB_CN_HW_PATH CACHE)
ENDIF ()
