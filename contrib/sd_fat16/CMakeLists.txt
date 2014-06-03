################################################################################
#
# CMake file for FAT16 SD card library for Xilinx ZC702 board
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

SET(SDFAT16_NAME sdfat16-zc702)

SET(SDFAT16_PROJECT_NAME ${SDFAT16_NAME}-${DEMO_TARGET_NAME})
STRING(TOLOWER ${SDFAT16_PROJECT_NAME} SDFAT16_PROJECT_NAME)
SET(XIL_FAT_DIR $ENV{XILINX_EDK}/sw/lib/sw_apps/fsbl)
STRING(REPLACE "\\" "/" XIL_FAT_DIR ${XIL_FAT_DIR})

PROJECT(${SDFAT16_PROJECT_NAME} C)

CMAKE_MINIMUM_REQUIRED (VERSION 2.8.0)

################################################################################
# Set options

SET(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})

IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Release CACHE STRING
       "Choose the type of build, options are: None Debug Release"
       FORCE)
ENDIF()

# Set library binary name
IF(${CMAKE_BUILD_TYPE} STREQUAL "Debug")
    SET( SDFAT16_LIB_NAME "${SDFAT16_NAME}_d")
ELSE()
    SET( SDFAT16_LIB_NAME "${SDFAT16_NAME}")
ENDIF()

SET(BOARDS_COMMON_DIR ${CMAKE_SOURCE_DIR}/boards/${BOARD_NAME}/common)
SET(EXAMPLE_BINARY_DIR ${CMAKE_BINARY_DIR}/boards/${BOARD_NAME}/${EXAMPLE_NAME})

########################################################################
# Set all source files
########################################################################
SET(SDFAT16_LIB_SRCS
    ${XIL_FAT_DIR}/src/ff.c
    ${XIL_FAT_DIR}/src/mmc.c
    ${PROJECT_SOURCE_DIR}/src/fs_sdcard.c
)

########################################################################
# Set include paths
########################################################################
INCLUDE_DIRECTORIES(
                    ${PROJECT_SOURCE_DIR}/include
                    ${XIL_FAT_DIR}/src
                   )

########################################################################
# Configure compile definitions
########################################################################
ADD_DEFINITIONS("-Wall -Wextra -pedantic -std=c99")
ADD_DEFINITIONS("${SDFAT16_CFLAGS}")

########################################################################
# Setup the architecture specific definitions
########################################################################
IF(CMAKE_SYSTEM_NAME STREQUAL "Generic" AND CMAKE_SYSTEM_PROCESSOR STREQUAL "ARM_Xilinx")
    include(configure-arm_xilinx)
ELSE()
    MESSAGE(FATAL_ERROR "System ${CMAKE_SYSTEM_NAME} with processor ${CMAKE_SYSTEM_PROCESSOR} is not supported!")
ENDIF()

########################################################################
# Build library
########################################################################
ADD_LIBRARY(${SDFAT16_PROJECT_NAME} ${SDFAT16_LIB_SRCS} ${LIB_ARCH_SRCS})
SET_PROPERTY(TARGET ${SDFAT16_PROJECT_NAME} PROPERTY DEBUG_POSTFIX "_d")

########################################################################
# Installation rules
########################################################################
ADD_CUSTOM_COMMAND(TARGET ${SDFAT16_PROJECT_NAME}
                   POST_BUILD
                   COMMAND ${CMAKE_COMMAND} -E copy_if_different $<TARGET_FILE_NAME:${SDFAT16_PROJECT_NAME}> ${PROJECT_BINARY_DIR}/lib${SDFAT16_LIB_NAME}.a
                  )

SET_DIRECTORY_PROPERTIES(PROPERTIES
                         ADDITIONAL_MAKE_CLEAN_FILES "${PROJECT_BINARY_DIR}/lib${HOSTIF_LIB_NAME}.a"
                        )

INSTALL(PROGRAMS ${PROJECT_BINARY_DIR}/lib${SDFAT16_LIB_NAME}.a DESTINATION ${CFG_DEMO_BOARD_NAME}/${CFG_DEMO_NAME}/lib${SDFAT16_NAME})
INSTALL(DIRECTORY ${PROJECT_SOURCE_DIR}/include DESTINATION ${CFG_DEMO_BOARD_NAME}/${CFG_DEMO_NAME}/lib${SDFAT16_NAME}
        FILES_MATCHING PATTERN "*.h")
INSTALL(FILES ${XIL_FAT_DIR}/src/diskio.h ${XIL_FAT_DIR}/src/ff.h ${XIL_FAT_DIR}/src/ffconf.h
        ${XIL_FAT_DIR}/src/sd_hardware.h DESTINATION ${CFG_DEMO_BOARD_NAME}/${CFG_DEMO_NAME}/lib${SDFAT16_NAME}/include)