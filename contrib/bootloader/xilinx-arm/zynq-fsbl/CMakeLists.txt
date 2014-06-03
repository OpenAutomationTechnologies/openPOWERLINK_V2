################################################################################
#
# CMake file of the bootloader zynq-fsbl
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

CMAKE_MINIMUM_REQUIRED (VERSION 2.8.7)

################################################################################
# Setup project and generic options
PROJECT(zynq-fsbl C)

ENABLE_LANGUAGE(ASM)
SET(CMAKE_ASM_FLAGS_INIT "")

MESSAGE(STATUS "CMAKE_SYSTEM_NAME is ${CMAKE_SYSTEM_NAME}")
MESSAGE(STATUS "CMAKE_SYSTEM_PROCESSOR is ${CMAKE_SYSTEM_PROCESSOR}")

################################################################################
# Set variables
STRING(TOLOWER "${CMAKE_SYSTEM_NAME}" SYSTEM_NAME_DIR)
STRING(TOLOWER "${CMAKE_SYSTEM_PROCESSOR}" SYSTEM_PROCESSOR_DIR)

SET(OPLK_BASE_DIR ${PROJECT_SOURCE_DIR}/../../../../)
SET(TOOLS_DIR ${OPLK_BASE_DIR}/tools)
SET(XIL_ZYNQ_FSBL_SRC_DIR $ENV{XILINX_EDK}/sw/lib/sw_apps/fsbl)
SET(XIL_ZYNQ_FSBL_DIR ${PROJECT_SOURCE_DIR})
STRING(REPLACE "\\" "/" XIL_ZYNQ_FSBL_SRC_DIR ${XIL_ZYNQ_FSBL_SRC_DIR})

SET(CMAKE_CONFIGURATION_TYPES "None;Debug;Release;RelWithDebInfo;MinSizeRel"
    CACHE INTERNAL "Available Build Configurations" FORCE)

IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Release CACHE STRING
        "Choose the type of build"
        FORCE)
    SET_PROPERTY(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "None;Debug;Release;RelWithDebInfo;MinSizeRel")
ENDIF()

################################################################################
# Verify Xilinx install directory with Zynq FSBL exists
IF(NOT EXISTS "${XIL_ZYNQ_FSBL_SRC_DIR}" OR NOT IS_DIRECTORY "${XIL_ZYNQ_FSBL_SRC_DIR}")
    MESSAGE(FATAL_ERROR "unexpected: Can't find Zynq FSBL repository: ${XIL_ZYNQ_FSBL_SRC_DIR}. Please check Xilinx EDK path is set properly!")
ELSE()
    MESSAGE(STATUS "Found Zynq FSBL in folder: ${XIL_ZYNQ_FSBL_SRC_DIR}")
ENDIF()

###############################################################################
# Copy the source files for FSBL application from Xilinx Install directory
FILE(COPY ${XIL_ZYNQ_FSBL_SRC_DIR}/src DESTINATION ${XIL_ZYNQ_FSBL_DIR}/)

################################################################################
# Set sources of the bootloader
SET(FSBL_C_SRCS
    ${XIL_ZYNQ_FSBL_DIR}/src/ddr_init.c
    ${XIL_ZYNQ_FSBL_DIR}/src/ff.c
    ${XIL_ZYNQ_FSBL_DIR}/src/fsbl_hooks.c
    ${XIL_ZYNQ_FSBL_DIR}/src/image_mover.c
    ${XIL_ZYNQ_FSBL_DIR}/src/main.c
    ${XIL_ZYNQ_FSBL_DIR}/src/md5.c
    ${XIL_ZYNQ_FSBL_DIR}/src/mmc.c
    ${XIL_ZYNQ_FSBL_DIR}/src/nand.c
    ${XIL_ZYNQ_FSBL_DIR}/src/nor.c
    ${XIL_ZYNQ_FSBL_DIR}/src/ps7_init.c
    ${XIL_ZYNQ_FSBL_DIR}/src/pcap.c
    ${XIL_ZYNQ_FSBL_DIR}/src/qspi.c
    ${XIL_ZYNQ_FSBL_DIR}/src/rsa.c
    ${XIL_ZYNQ_FSBL_DIR}/src/sd.c
)

SET(FSBL_ASM_SRCS
    ${XIL_ZYNQ_FSBL_DIR}/src/fsbl_handoff.S
)

################################################################################
# Set includes of the bootloader
INCLUDE_DIRECTORIES(
                   ${XIL_ZYNQ_FSBL_DIR}/src
)

IF((CMAKE_SYSTEM_NAME STREQUAL "Generic") AND (CMAKE_SYSTEM_PROCESSOR STREQUAL "ARM_Xilinx"))
    INCLUDE(configure-arm_xilinx.cmake)
ELSE()
    MESSAGE(FATAL_ERROR "Zynq Fsbl only ported to target CMAKE_SYSTEM_PROCESSOR = ARM_Xilinx!")
ENDIF()

SET(EXECUTABLE_NAME ${ARCH_EXECUTABLE_NAME})

LINK_DIRECTORIES(${XIL_ZYNQ_FSBL_DIR}/src)
################################################################################
# Set the executable
ADD_EXECUTABLE(${EXECUTABLE_NAME} ${FSBL_C_SRCS} ${FSBL_ASM_SRCS})

# Set flags to source files
SET(CFLAGS "-Wall -Wextra -pedantic ")

SET_SOURCE_FILES_PROPERTIES(${FSBL_C_SRCS} PROPERTIES COMPILE_FLAGS "${CFLAGS} ${ARCH_CFLAGS}")
SET_SOURCE_FILES_PROPERTIES(${FSBL_ASM_SRCS} PROPERTIES COMPILE_FLAGS ${ARCH_ASMFLAGS})

SET_TARGET_PROPERTIES(${EXECUTABLE_NAME} PROPERTIES LINK_FLAGS ${ARCH_LINKERFLAGS})

TARGET_LINK_LIBRARIES(${EXECUTABLE_NAME} ${ARCH_LIBS})


################################################################################
# Add additional clean files
SET_DIRECTORY_PROPERTIES(PROPERTIES ADDITIONAL_MAKE_CLEAN_FILES "${ADD_CLEAN_FILES}")

################################################################################
# Installation rules
INSTALL(TARGETS ${EXECUTABLE_NAME} RUNTIME DESTINATION ${CFG_BIN_DIR})
