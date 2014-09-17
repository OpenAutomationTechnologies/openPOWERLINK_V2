################################################################################
#
# CMake file for openPOWERLINK userspace daemon driver
#
# Copyright (c) 2014, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
# Copyright (c) 2012, SYSTEC electronik GmbH
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

PROJECT(oplkd-pcap C)

MESSAGE(STATUS "CMAKE_SYSTEM_NAME is ${CMAKE_SYSTEM_NAME}")
MESSAGE(STATUS "CMAKE_SYSTEM_PROCESSOR is ${CMAKE_SYSTEM_PROCESSOR}")

IF(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    CMAKE_MINIMUM_REQUIRED (VERSION 2.8.7)
ELSE()
    MESSAGE(FATAL_ERROR "Unsupported system ${CMAKE_SYSTEM_NAME} for this project!")
ENDIF()

STRING(TOLOWER "${CMAKE_SYSTEM_NAME}" SYSTEM_NAME_DIR)
STRING(TOLOWER "${CMAKE_SYSTEM_PROCESSOR}" SYSTEM_PROCESSOR_DIR)

IF(CFG_OPLK_MN)
    SET(EXE_NAME oplkmnd-pcap)
ELSE()
    SET(EXE_NAME oplkcnd-pcap)
ENDIF()
MESSAGE(STATUS "Configuring ${EXE_NAME}")

###############################################################################
# Set global directories
###############################################################################
SET(OPLK_BASE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../..)
SET(DRV_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR})
SET(CONTRIB_SOURCE_DIR ${OPLK_BASE_DIR}/contrib)
SET(OPLK_INCLUDE_DIR ${OPLK_BASE_DIR}/stack/include)
SET(TOOLS_DIR ${OPLK_BASE_DIR}/tools)

###############################################################################
# Include CMake Modules
###############################################################################
SET(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../../cmake ${CMAKE_MODULE_PATH})

# include standard cmake modules
INCLUDE(CMakeDependentOption)

IF(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
  SET(CMAKE_INSTALL_PREFIX
    ${OPLK_BASE_DIR}/bin/${SYSTEM_NAME_DIR}/${SYSTEM_PROCESSOR_DIR} CACHE PATH "openPOWERLINK drivers install prefix" FORCE
    )
ENDIF(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release CACHE STRING
      "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
      FORCE)
ENDIF(NOT CMAKE_BUILD_TYPE)

OPTION (CFG_OPLK_MN "Compile openPOWERLINK MN driver (Otherwise CN)" ON)

SET(CFG_DEBUG_LVL "0xEC000000L" CACHE STRING "Debug Level for debug output")

STRING(TOUPPER "${CMAKE_BUILD_TYPE}" BUILD_TYPE_NAME)
IF("${BUILD_TYPE_NAME}" STREQUAL "DEBUG")
    SET(BUILD_TYPE_EXT "_d")
ELSE()
    SET(BUILD_TYPE_EXT "")
ENDIF()

# select libary and search for it
IF(CFG_OPLK_MN)
    SET(LIB_NAME oplkmndrv-pcap)
ELSE()
    SET(LIB_NAME oplkcndrv-pcap)
ENDIF()

SET(OPLKLIB_DIR ${OPLK_BASE_DIR}/stack/lib/${SYSTEM_NAME_DIR}/${SYSTEM_PROCESSOR_DIR})
SET(OPLKLIB_INCDIR ${OPLK_BASE_DIR}/stack/proj/${SYSTEM_NAME_DIR}/lib${LIB_NAME})

UNSET(OPLKLIB CACHE)
FIND_LIBRARY(OPLKLIB NAME ${LIB_NAME}${BUILD_TYPE_EXT}
                     HINTS ${OPLKLIB_DIR})
INCLUDE_DIRECTORIES(${OPLKLIB_INCDIR})

SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -pedantic -std=c99 -pthread -fno-strict-aliasing")
ADD_DEFINITIONS(-D_GNU_SOURCE -D_POSIX_C_SOURCE=200112L)
IF(CFG_OPLK_MN)
    ADD_DEFINITIONS(-DCONFIG_MN)
ENDIF()
#
# Source include directories
#
# set sources of POWERLINK application
SET(DRV_SOURCES
    daemon.c
    ${CONTRIB_SOURCE_DIR}/console/printlog.c
    ${CONTRIB_SOURCE_DIR}/console/console-linux.c
    )

INCLUDE_DIRECTORIES(
    ${OPLK_INCLUDE_DIR}
    ${CONTRIB_SOURCE_DIR}
    )

SET (ARCH_LIBRARIES pcap pthread rt)

ADD_EXECUTABLE(${EXE_NAME} ${DRV_SOURCES})
SET_PROPERTY(TARGET ${EXE_NAME} PROPERTY COMPILE_DEFINITIONS_DEBUG DEBUG;DEF_DEBUG_LVL=${CFG_DEBUG_LVL})
TARGET_LINK_LIBRARIES(${EXE_NAME} ${OPLKLIB} ${ARCH_LIBRARIES})

# add installation rules
INSTALL(TARGETS ${EXE_NAME} RUNTIME DESTINATION ${CMAKE_PROJECT_NAME})

