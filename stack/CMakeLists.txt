################################################################################
#
# Main CMake file for openPOWERLINK stack libraries
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

PROJECT (openPOWERLINK C)

CMAKE_MINIMUM_REQUIRED (VERSION 2.8.7)

MESSAGE(STATUS "CMAKE_SYSTEM_NAME is ${CMAKE_SYSTEM_NAME}")
MESSAGE(STATUS "CMAKE_SYSTEM_PROCESSOR is ${CMAKE_SYSTEM_PROCESSOR}")

STRING(TOLOWER "${CMAKE_SYSTEM_NAME}" SYSTEM_NAME_DIR)
STRING(TOLOWER "${CMAKE_SYSTEM_PROCESSOR}" SYSTEM_PROCESSOR_DIR)

IF(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
  SET(CMAKE_INSTALL_PREFIX
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/${SYSTEM_NAME_DIR}/${SYSTEM_PROCESSOR_DIR}
    CACHE PATH "openPOWERLINK library install prefix" FORCE
    )
ENDIF()

# include cmake modules
INCLUDE(CMakeDependentOption)

################################################################################
# Set options
################################################################################

SET(CMAKE_CONFIGURATION_TYPES "Debug;Release" CACHE INTERNAL "Available Build Configurations" FORCE)

IF(NOT CMAKE_GENERATOR MATCHES "Visual Studio")
    IF(NOT CMAKE_BUILD_TYPE)
      SET(CMAKE_BUILD_TYPE Release CACHE STRING
          "Choose the type of build, options are: None Debug Release"
          FORCE)
    ENDIF()
ENDIF()

SET(CFG_DEBUG_LVL "0xEC000000L" CACHE STRING "Debug Level for debug output")

STRING(TOUPPER "${CMAKE_BUILD_TYPE}" LIB_BUILD_TYPE)
IF("${LIB_BUILD_TYPE}" STREQUAL "DEBUG")
    SET(LIB_BUILD_TYPE_EXT "_d")
ELSE()
    SET(LIB_BUILD_TYPE_EXT "")
ENDIF()

################################################################################
# Set global definitions
################################################################################

SET(OPLK_BASE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/..)

INCLUDE(cmake/directories.cmake)
INCLUDE(cmake/stackfiles.cmake)

# set global include directories
INCLUDE_DIRECTORIES (
    ${STACK_INCLUDE_DIR}
    ${STACK_SOURCE_DIR}
    ${CONTRIB_SOURCE_DIR}
)

################################################################################
# Libraries
################################################################################
IF(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    INCLUDE(cmake/options-linux.cmake)
ELSEIF(CMAKE_SYSTEM_NAME STREQUAL "Windows")
    INCLUDE(cmake/options-windows.cmake)
ELSEIF((CMAKE_SYSTEM_NAME STREQUAL "Generic") AND (CMAKE_SYSTEM_PROCESSOR STREQUAL "Microblaze"))
    INCLUDE(cmake/options-microblaze.cmake)
ELSEIF((CMAKE_SYSTEM_NAME STREQUAL "Generic") AND (CMAKE_SYSTEM_PROCESSOR STREQUAL "zynqarm"))
    INCLUDE(cmake/options-zynqarm.cmake)
ELSEIF((CMAKE_SYSTEM_NAME STREQUAL "Generic") AND (CMAKE_SYSTEM_PROCESSOR STREQUAL "alterac5arm"))
    INCLUDE(cmake/options-c5socarm.cmake)
ELSE()
    MESSAGE(FATAL_ERROR "Unknown Platform and processor combination ${CMAKE_SYSTEM_NAME} and ${CMAKE_SYSTEM_PROCESSOR}!!")
ENDIF()

################################################################################
# Install oplk headers files
################################################################################
INSTALL(DIRECTORY ${STACK_INCLUDE_DIR}/oplk DESTINATION "include")
