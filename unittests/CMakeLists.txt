################################################################################
#
# CMake file of openPOWERLINK unit tests
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


################################################################################
# Setup project and generic options

PROJECT(unittests C)
MESSAGE(STATUS "Configuring openPOWERLINK unit tests")

CMAKE_MINIMUM_REQUIRED (VERSION 2.8.7)

STRING(TOLOWER "${CMAKE_SYSTEM_NAME}" SYSTEM_NAME_DIR)
STRING(TOLOWER "${CMAKE_SYSTEM_PROCESSOR}" SYSTEM_PROCESSOR_DIR)

################################################################################
# Macro for adding unit tests
MACRO(ADD_UNIT_TEST TestDirectory TestExeName TST_SOURCES)
    STRING (TOUPPER ${TestDirectory} TestName)

    ADD_DEFINITIONS (${TEST_XML_OUTPUT})
    ADD_EXECUTABLE (${TestExeName} ${TST_SOURCES})

    TARGET_LINK_LIBRARIES (${TestExeName} cunit)

    ADD_TEST (${TestName} ${PROJECT_BINARY_DIR}/${TestExeName})
ENDMACRO(ADD_UNIT_TEST)

################################################################################
# Set general directories
SET(OPLK_ROOT_DIR ${CMAKE_SOURCE_DIR}/..)
SET(OPLK_SOURCE_DIR ${OPLK_ROOT_DIR}/stack/src)
SET(OPLK_INCLUDE_DIR ${OPLK_ROOT_DIR}/stack/include)
SET(TEST_COMMON_SOURCE_DIR ${CMAKE_SOURCE_DIR}/common)

# We need a oplkcfg.h file for compiling the sources
# We are using the project for the complete MN library
SET(OPLK_LIB_NAME oplkmn)
SET(OPLK_PROJ_DIR ${OPLK_ROOT_DIR}/stack/proj/${SYSTEM_NAME_DIR}/lib${OPLK_LIB_NAME})

# general unit test includes
INCLUDE_DIRECTORIES (/usr/include)
INCLUDE_DIRECTORIES (${TEST_COMMON_SOURCE_DIR})
INCLUDE_DIRECTORIES (${OPLK_SOURCE_DIR})
INCLUDE_DIRECTORIES (${OPLK_INCLUDE_DIR})
INCLUDE_DIRECTORIES (${OPLK_PROJ_DIR})

################################################################################

IF(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
  SET(CMAKE_INSTALL_PREFIX
    ${OPLK_ROOT_DIR}/bin/${SYSTEM_NAME_DIR}/${SYSTEM_PROCESSOR_DIR}/unittests CACHE PATH "openPOWERLINK unittest install prefix" FORCE
    )
ENDIF()

################################################################################
# Add subdirectories with specific tests

# tests for event handler
ADD_SUBDIRECTORY (tests/event)
