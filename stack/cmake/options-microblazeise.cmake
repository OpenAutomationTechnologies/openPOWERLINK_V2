################################################################################
#
# CMake options for openPOWERLINK stack on Xilinx/Microblaze ISE
#
# Copyright (c) 2014, B&R Industrial Automation GmbH
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

MESSAGE(STATUS "Adding CMake configuration options for Microblazeise")

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
# Options for CN libraries
OPTION(CFG_COMPILE_LIB_CN                           "Compile openPOWERLINK CN library" OFF)

################################################################################
# Options for MN libraries
OPTION(CFG_COMPILE_LIB_MNAPP_HOSTIF     "Compile openPOWERLINK MN host/application library" OFF)
OPTION(CFG_COMPILE_LIB_MNDRV_HOSTIF     "Compile openPOWERLINK MN driver library" OFF)
################################################################################
# Add library subdirectories and hardware library path

# CN libraries
IF(CFG_COMPILE_LIB_CN)
    # Path to the hardware library folder of your board example
    SET( CFG_COMPILE_LIB_CN_HW_LIB_DIR ${XIL_HW_LIB_DIR}/avnet-s6plkeb/cn-single-gpio
            CACHE PATH "Path to the hardware library folder for the single processor CN library")

    ADD_SUBDIRECTORY(proj/generic/liboplkcn)
ELSE()
    UNSET(CFG_COMPILE_LIB_CN_HW_PATH CACHE)
ENDIF ()
# MN libraries
IF (CFG_COMPILE_LIB_MNAPP_HOSTIF)
    # Path to the hardware library folder of your board example
    SET(CFG_COMPILE_LIB_MN_APP_HOSTIF_HW_LIB_DIR ${XIL_HW_LIB_DIR}/avnet-lx150t/mn-dual-hostif-gpio
            CACHE PATH "Path to the hardware host library folder for the dual processor MN library")
    ADD_SUBDIRECTORY(proj/generic/liboplkmnapp-hostif)
ELSE ()
    UNSET(CFG_COMPILE_LIB_MN_APP_HOSTIF_HW_LIB_DIR CACHE)
ENDIF ()
IF (CFG_COMPILE_LIB_MNDRV_HOSTIF)
    # Path to the hardware library folder of your board example
    SET(CFG_COMPILE_LIB_MN_DRV_HOSTIF_HW_LIB_DIR ${XIL_HW_LIB_DIR}/avnet-lx150t/mn-dual-hostif-gpio
            CACHE PATH "Path to the hardware pcp library folder for the dual processor MN library")
    ADD_SUBDIRECTORY(proj/generic/liboplkmndrv-hostif)
ELSE ()
    UNSET(CFG_COMPILE_LIB_MN_DRV_HOSTIF_HW_LIB_DIR CACHE)
ENDIF ()
