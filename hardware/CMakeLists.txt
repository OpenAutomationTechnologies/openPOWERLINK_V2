################################################################################
#
# CMake root file for finding all board examples for the selected platform
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

PROJECT(Hardware NONE)

CMAKE_MINIMUM_REQUIRED(VERSION 2.8.0)

################################################################################
# Handle includes
SET(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})

INCLUDE(reduceboardname)

################################################################################
# Set paths
SET(OPLK_BASE_DIR ${CMAKE_SOURCE_DIR}/..)
SET(OMETHLIB_BASE_DIR ${PROJECT_SOURCE_DIR}/drivers/openmac)
SET(OMETHLIB_BUILD_DIR ${PROJECT_BINARY_DIR}/drivers/openmac)
SET(HOSTIF_BASE_DIR ${PROJECT_SOURCE_DIR}/drivers/hostinterface)
SET(HOSTIF_BUILD_DIR ${PROJECT_BINARY_DIR}/drivers/hostinterface)

STRING(TOLOWER "${CMAKE_SYSTEM_NAME}" SYSTEM_NAME_DIR)
STRING(TOLOWER "${CMAKE_SYSTEM_PROCESSOR}" SYSTEM_PROCESSOR_DIR)

IF(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
  SET(CMAKE_INSTALL_PREFIX
    ${CMAKE_SOURCE_DIR}/lib/${SYSTEM_NAME_DIR}/${SYSTEM_PROCESSOR_DIR} CACHE PATH "openPOWERLINK hardware install prefix" FORCE
    )
ENDIF()

################################################################################
# Handle target specific functions
IF(${CMAKE_SYSTEM_NAME} STREQUAL "Generic" AND ${CMAKE_SYSTEM_PROCESSOR} STREQUAL "Microblaze")
    INCLUDE(configure-microblaze)
ELSE()
    MESSAGE(FATAL_ERROR "Unknown platform selected!")
ENDIF()

################################################################################
# Set paths with target specific information
SET(IPCORE_REPOSITORY ${ARCH_IPCORE_REPO})

################################################################################
# Search for all demos
FOREACH(BOARD_DIR IN ITEMS ${BOARD_DIRS})

    GET_FILENAME_COMPONENT(BOARD_NAME ${BOARD_DIR} NAME)

    REDUCE_BOARD_NAME(${BOARD_NAME} "-" FRAG_BOARD_NAME)

    STRING(TOUPPER ${FRAG_BOARD_NAME} FRAG_BOARD_NAME)

    FILE(GLOB EXAMPLE_NAMES RELATIVE "${BOARD_DIR}/" "${BOARD_DIR}/[c|m]n-*")

    FOREACH(EXAMPLE_NAME IN ITEMS ${EXAMPLE_NAMES})
        STRING(TOUPPER ${EXAMPLE_NAME} EXAMPLE_NAME_VAR)
        STRING(REGEX REPLACE "-" "_" EXAMPLE_NAME_VAR ${EXAMPLE_NAME_VAR})
        SET(DEMO_TARGET_NAME "${FRAG_BOARD_NAME}_${EXAMPLE_NAME_VAR}")
        IF(NOT DEFINED DEMO_${DEMO_TARGET_NAME})
            OPTION(DEMO_${DEMO_TARGET_NAME} "Build example demo ${DEMO_TARGET_NAME}" OFF)
            MESSAGE(STATUS "Found hardware platform: DEMO_${DEMO_TARGET_NAME} set to ${DEMO_${DEMO_TARGET_NAME}}!")
        ELSE()
            MESSAGE(STATUS "Hardware platform: DEMO_${DEMO_TARGET_NAME} set to ${DEMO_${DEMO_TARGET_NAME}}!")
        ENDIF()

        IF(DEMO_${DEMO_TARGET_NAME})
            # Include board specific settings file
            SET_BOARD_CONFIGURATION(${BOARD_DIR}/${EXAMPLE_NAME}/cmake)

            CONFIGURE_FILE(cmake/BoardsCMakeLists.cmake.in ${BOARD_DIR}/${EXAMPLE_NAME}/CMakeLists.txt COPYONLY)
            ADD_SUBDIRECTORY(${BOARD_DIR}/${EXAMPLE_NAME})

            FOREACH(PROC_INST_NAME PCP HOST)
                # Create config variable prefix
                SET(CFG_PREFIX CFG_${PROC_INST_NAME})

                # Skip if configured processor does not exist or match current processor
                IF(DEFINED ${CFG_PREFIX}_PROCESSOR )
                    IF(${${CFG_PREFIX}_PROCESSOR} STREQUAL ${CMAKE_SYSTEM_PROCESSOR})
                        # Create targets for building the omethlib
                        IF(DEFINED ${CFG_PREFIX}_OMETHLIB_ENABLE  OR (NOT ${${CFG_PREFIX}_OMETHLIB_ENABLE} STREQUAL ""))
                            ADD_SUBDIRECTORY(${OMETHLIB_BASE_DIR} ${OMETHLIB_BUILD_DIR}/${BOARD_NAME}/${EXAMPLE_NAME})
                        ENDIF()

                        # Create targets for building the host interface
                        IF(DEFINED ${CFG_PREFIX}_HOSTIF_ENABLE  OR (NOT ${${CFG_PREFIX}HOSTIF_ENABLE} STREQUAL ""))
                            SET(SEL_CPU_NAME ${${CFG_PREFIX}_NAME})
                            ADD_SUBDIRECTORY(${HOSTIF_BASE_DIR} ${HOSTIF_BUILD_DIR}/${BOARD_NAME}/${EXAMPLE_NAME}/${${CFG_PREFIX}_NAME})
                        ENDIF()

                    ENDIF()
                ENDIF()
            ENDFOREACH(PROC_INST_NAME PCP HOST)
        ENDIF()
    ENDFOREACH(EXAMPLE_NAME IN ITEMS ${EXAMPLE_NAMES})

ENDFOREACH(BOARD_DIR IN ITEMS ${BOARD_DIRS})
