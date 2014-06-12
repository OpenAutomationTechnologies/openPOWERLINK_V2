################################################################################
#
# CMake file for omethlib libraries (Driver for the openMAC IP-Core)
#
# Copyright (c) 2014, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
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

SET(OMETH_NAME omethlib)

SET(OMETH_PROJECT_NAME ${OMETH_NAME}-${DEMO_TARGET_NAME})
STRING(TOLOWER ${OMETH_PROJECT_NAME} OMETH_PROJECT_NAME)

PROJECT(${OMETH_PROJECT_NAME} C)

CMAKE_MINIMUM_REQUIRED(VERSION 2.8.7)

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
    SET(OMETH_LIB_NAME "${OMETH_NAME}_d")
ELSE()
    SET(OMETH_LIB_NAME "${OMETH_NAME}")
ENDIF()

SET(BOARDS_COMMON_DIR ${CMAKE_SOURCE_DIR}/boards/${BOARD_NAME}/common)
SET(EXAMPLE_BINARY_DIR ${CMAKE_BINARY_DIR}/boards/${BOARD_NAME}/${EXAMPLE_NAME})

########################################################################
# Set all source files
########################################################################
SET(OMETH_LIB_SRCS
    ${PROJECT_SOURCE_DIR}/src/omethlib.c
    ${PROJECT_SOURCE_DIR}/src/omethlibint.c
   )

# Set include paths
SET(OMETH_LIB_INCS
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/src
    ${OPLK_BASE_DIR}/stack/include
   )

# Configure compile definitions
ADD_DEFINITIONS("-Wall -Wextra -pedantic -std=c99")

########################################################################
# Setup the architecture specific definitions
########################################################################
IF(CMAKE_SYSTEM_NAME STREQUAL "Generic" AND CMAKE_SYSTEM_PROCESSOR STREQUAL "Microblaze")
    include(configure-microblaze)
ELSE()
    MESSAGE(FATAL_ERROR "System ${CMAKE_SYSTEM_NAME} with processor ${CMAKE_SYSTEM_PROCESSOR} is not supported!")
ENDIF()

########################################################################
# Add library
########################################################################
INCLUDE_DIRECTORIES(${OMETH_LIB_INCS} ${LIB_ARCH_INCS})
ADD_LIBRARY(${OMETH_PROJECT_NAME} ${OMETH_LIB_SRCS} ${LIB_ARCH_SRCS})
SET_PROPERTY(TARGET ${OMETH_PROJECT_NAME} PROPERTY DEBUG_POSTFIX "_d")

########################################################################
# Installation rules
########################################################################
ADD_CUSTOM_COMMAND(TARGET ${OMETH_PROJECT_NAME}
                   POST_BUILD
                   COMMAND ${CMAKE_COMMAND} -E copy_if_different $<TARGET_FILE_NAME:${OMETH_PROJECT_NAME}> ${PROJECT_BINARY_DIR}/lib${OMETH_LIB_NAME}.a
                  )

SET_DIRECTORY_PROPERTIES(PROPERTIES
                         ADDITIONAL_MAKE_CLEAN_FILES "${PROJECT_BINARY_DIR}/lib${OMETH_LIB_NAME}.a"
                        )

INSTALL(PROGRAMS ${PROJECT_BINARY_DIR}/lib${OMETH_LIB_NAME}.a DESTINATION ${CFG_DEMO_BOARD_NAME}/${CFG_DEMO_NAME}/lib${OMETH_NAME})
INSTALL(DIRECTORY ${PROJECT_SOURCE_DIR}/include DESTINATION ${CFG_DEMO_BOARD_NAME}/${CFG_DEMO_NAME}/lib${OMETH_NAME}
        FILES_MATCHING PATTERN "*.h")
