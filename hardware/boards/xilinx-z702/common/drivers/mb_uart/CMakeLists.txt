################################################################################
#
# CMake file for UART re-direction library for Microblaze on ZC702 board
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

SET(MB_UART_NAME mb-uart)

SET(MB_UART_PROJECT_NAME ${MB_UART_NAME}-${DEMO_TARGET_NAME})
STRING(TOLOWER ${MB_UART_PROJECT_NAME} MB_UART_PROJECT_NAME)

PROJECT(${MB_UART_PROJECT_NAME} C)

CMAKE_MINIMUM_REQUIRED (VERSION 2.8.7)

################################################################################
# Set options

SET(CMAKE_MODULE_PATH "${OPLK_BASE_DIR}/cmake" ${CMAKE_MODULE_PATH})

IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Release CACHE STRING
       "Choose the type of build, options are: None Debug Release"
       FORCE)
ENDIF()

# Set library binary name
IF(${CMAKE_BUILD_TYPE} STREQUAL "Debug")
    SET( MB_UART_NAME "${MB_UART_NAME}_d")
ELSE()
    SET( MB_UART_NAME "${MB_UART_NAME}")
ENDIF()

SET(BOARDS_COMMON_DIR ${CMAKE_SOURCE_DIR}/boards/${BOARD_NAME}/common)
SET(EXAMPLE_BINARY_DIR ${CMAKE_BINARY_DIR}/boards/${BOARD_NAME}/${EXAMPLE_NAME})

########################################################################
# Set all source files
########################################################################
SET(MB_UART_LIB_SRCS
    ${PROJECT_SOURCE_DIR}/src/mb_uart.c
)

########################################################################
# Set include paths
########################################################################
SET(MB_UART_LIB_INCS
    ${PROJECT_SOURCE_DIR}/include
    ${EXAMPLE_BINARY_DIR}/bsp${CFG_${PROC_INST_NAME}_NAME}/${CFG_${PROC_INST_NAME}_NAME}/include
   )

########################################################################
# Configure compile definitions
########################################################################
ADD_DEFINITIONS("-Wall -Wextra -pedantic -std=c99")
ADD_DEFINITIONS(${XIL_${PROC_INST_NAME}_CFLAGS} "-fmessage-length=0 -mcpu=${CFG_${PROC_INST_NAME}_CPU_VERSION} -ffunction-sections -fdata-sections")
########################################################################
# Build library
########################################################################
INCLUDE_DIRECTORIES(${MB_UART_LIB_INCS})
ADD_LIBRARY(${MB_UART_PROJECT_NAME} ${MB_UART_LIB_SRCS})
SET_PROPERTY(TARGET ${HOSTIF_PROJECT_NAME} PROPERTY DEBUG_POSTFIX "_d")

# Eclipse project files
SET(CFG_CPU_NAME ${CFG_${PROC_INST_NAME}_NAME})

GEN_ECLIPSE_FILE_LIST("${MB_UART_LIB_SRCS}" "" PART_ECLIPSE_FILE_LIST)
SET(ECLIPSE_FILE_LIST "${ECLIPSE_FILE_LIST} ${PART_ECLIPSE_FILE_LIST}")
GET_PROPERTY(LIBRARY_INCLUDES DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
GEN_ECLIPSE_INCLUDE_LIST("${LIBRARY_INCLUDES}" ECLIPSE_INCLUDE_LIST )
CONFIGURE_FILE(${ARCH_TOOLS_DIR}/eclipse/libproject.in ${PROJECT_BINARY_DIR}/.project @ONLY)
CONFIGURE_FILE(${ARCH_TOOLS_DIR}/eclipse/libcproject.in ${PROJECT_BINARY_DIR}/.cproject @ONLY)

########################################################################
# Installation rules
########################################################################
ADD_CUSTOM_COMMAND(TARGET ${MB_UART_PROJECT_NAME}
                   POST_BUILD
                   COMMAND ${CMAKE_COMMAND} -E copy_if_different $<TARGET_FILE_NAME:${MB_UART_PROJECT_NAME}> ${PROJECT_BINARY_DIR}/lib${MB_UART_NAME}.a
                  )

SET_DIRECTORY_PROPERTIES(PROPERTIES
                         ADDITIONAL_MAKE_CLEAN_FILES "${PROJECT_BINARY_DIR}/lib${MB_UART_NAME}.a"
                        )

INSTALL(PROGRAMS ${PROJECT_BINARY_DIR}/lib${MB_UART_NAME}.a DESTINATION ${CFG_DEMO_BOARD_NAME}/${CFG_DEMO_NAME}/lib${MB_UART_NAME})
INSTALL(DIRECTORY ${PROJECT_SOURCE_DIR}/include DESTINATION ${CFG_DEMO_BOARD_NAME}/${CFG_DEMO_NAME}/lib${MB_UART_NAME}
        FILES_MATCHING PATTERN "*.h")
