################################################################################
#
# CMake file for openPOWERLINK kernel stack (Linux kernel module)
#
# Copyright (c) 2014, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
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
#
################################################################################

################################################################################
# Setup project and generic options

PROJECT(oplkdrv_daemon C)

IF (CMAKE_SYSTEM_NAME STREQUAL "Windows")
    # ensure working support of VS.NET 2010
    CMAKE_MINIMUM_REQUIRED (VERSION 2.8.7)
ELSE ()
    CMAKE_MINIMUM_REQUIRED (VERSION 2.8.7)
ENDIF ()

INCLUDE(../../cmake/findoplkdrvlib.cmake)
MESSAGE(STATUS "CMAKE_SYSTEM_NAME is ${CMAKE_SYSTEM_NAME}")
MESSAGE(STATUS "CMAKE_SYSTEM_PROCESSOR is ${CMAKE_SYSTEM_PROCESSOR}")

STRING(TOLOWER "${CMAKE_SYSTEM_NAME}" SYSTEM_NAME_DIR)
STRING(TOLOWER "${CMAKE_SYSTEM_PROCESSOR}" SYSTEM_PROCESSOR_DIR)

###############################################################################
# Include CMake Modules
###############################################################################
SET(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/../../cmake ${CMAKE_MODULE_PATH})

IF (NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release CACHE STRING
      "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
      FORCE)
ENDIF (NOT CMAKE_BUILD_TYPE)

OPTION(CFG_OPLK_MN "Compile openPOWERLINK MN driver (Otherwise CN)" ON)

SET(CFG_DEBUG_LVL "0xEC000000L" CACHE STRING "Debug Level for debug output")

SET(CFG_BUILD_KERNEL_STACK "host-interface"
    CACHE STRING "Configure how to build the kernel stack")

SET(KernelStackBuildTypes
    "host-interface;None"
    CACHE INTERNAL
    "List of possible kernel stack build types")

SET_PROPERTY(CACHE CFG_BUILD_KERNEL_STACK
             PROPERTY STRINGS ${KernelStackBuildTypes})

IF (CFG_BUILD_KERNEL_STACK STREQUAL "host-interface")
    UNSET(CFG_KERNEL_DUALPROCSHM CACHE)
    SET(CFG_KERNEL_STACK_PCP_HOSTIF_MODULE ON CACHE INTERNAL
        "Kernel-User stack communication through host interface library(Dual processor solution)")

ELSEIF (CFG_BUILD_KERNEL_STACK STREQUAL "None")
    UNSET(CFG_KERNEL_DUALPROCSHM CACHE)
    UNSET(CFG_KERNEL_STACK_PCP_HOSTIF_MODULE CACHE)

ENDIF ()

################################################################################
# Set global directories
################################################################################
SET(OPLK_ROOT_DIR ${CMAKE_SOURCE_DIR}/../../..)
SET(CONTRIB_SOURCE_DIR ${OPLK_ROOT_DIR}/contrib)
SET(STACK_SOURCE_DIR ${OPLK_ROOT_DIR}/stack/src)
SET(COMMON_SOURCE_DIR ${OPLK_ROOT_DIR}/stack/src/common)
SET(KERNEL_SOURCE_DIR ${OPLK_ROOT_DIR}/stack/src/kernel)
SET(EDRV_SOURCE_DIR ${OPLK_ROOT_DIR}/stack/src/kernel/edrv)
SET(OPLK_INCLUDE_DIR ${OPLK_ROOT_DIR}/stack/include)
SET(TOOLS_DIR ${OPLK_ROOT_DIR}/tools)

IF (CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
  SET(CMAKE_INSTALL_PREFIX
    ${OPLK_ROOT_DIR}/bin/${SYSTEM_NAME_DIR}/${SYSTEM_PROCESSOR_DIR}/
    CACHE PATH "openPOWERLINK kernel driver install prefix" FORCE
    )
ENDIF ()

# Set generic sources and include directories for this demo
SET(DEMO_SOURCES
    ${PROJECT_SOURCE_DIR}/daemon.c
    )

INCLUDE_DIRECTORIES(
                    ${PROJECT_SOURCE_DIR}
                   )

IF (CMAKE_SYSTEM_NAME STREQUAL "Generic" AND CMAKE_SYSTEM_PROCESSOR STREQUAL "Microblaze")
    INCLUDE(microblaze.cmake)
ELSE ()
    MESSAGE(FATAL_ERROR "System ${CMAKE_SYSTEM_NAME} with processor ${CMAKE_SYSTEM_PROCESSOR} is not supported!")
ENDIF ()

SET(EXECUTABLE_NAME "${PROJECT_NAME}${ARCH_EXE_SUFFIX}")

IF (CFG_OPLK_MN)
FIND_OPLK_DRV_LIBRARY("mn")
ENDIF ()

################################################################################
# Set the executable
ADD_EXECUTABLE(${EXECUTABLE_NAME} ${DEMO_SOURCES} ${DEMO_ARCH_SOURCES})
SET_PROPERTY(TARGET ${EXECUTABLE_NAME}
             PROPERTY COMPILE_DEFINITIONS_DEBUG DEBUG;DEF_DEBUG_LVL=${CFG_DEBUG_LVL})

SET_PROPERTY(TARGET ${EXECUTABLE_NAME}
             PROPERTY LINK_FLAGS ${ARCH_LINKER_FLAGS})

################################################################################
# Libraries to link

TARGET_LINK_LIBRARIES(${EXECUTABLE_NAME} optimized ${OPLKLIB} debug ${OPLKLIB_DEBUG})
TARGET_LINK_LIBRARIES(${EXECUTABLE_NAME} ${ARCH_LIBRARIES})

################################################################################
# Post Build targets
IF (CMAKE_SYSTEM_NAME STREQUAL "Generic" AND CMAKE_SYSTEM_PROCESSOR STREQUAL "Microblaze")
    INCLUDE(microblazeapppostactions)
ENDIF ()

################################################################################
# Installation rules

INSTALL(TARGETS ${EXECUTABLE_NAME} RUNTIME DESTINATION ${CFG_DEMO_BOARD_NAME}/${CFG_DEMO_NAME})
