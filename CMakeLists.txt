################################################################################
#
# Main CMake file of openPOWERLINK stack
#
# Copyright (c) 2013, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
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


IF (WIN32)
    # ensure working support of VS.NET 2010
    CMAKE_MINIMUM_REQUIRED (VERSION 2.8.4)
ELSE (WIN32)
    CMAKE_MINIMUM_REQUIRED (VERSION 2.8.0)
ENDIF (WIN32)

PROJECT (openPOWERLINK C CXX)

IF(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
  SET(CMAKE_INSTALL_PREFIX
    ${CMAKE_BINARY_DIR} CACHE PATH "openPOWERLINK install prefix" FORCE
    )
ENDIF(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)

# include cmake modules
INCLUDE(CMakeDependentOption)

# Search for installed packages
FIND_PACKAGE(Qt4 QUIET)

###############################################################################
# Set options
###############################################################################
#SET(CMAKE_INSTALL_PREFIX ${CMAKE_BINARY_DIR}/bin)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release CACHE STRING
      "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
      FORCE)
ENDIF(NOT CMAKE_BUILD_TYPE)

OPTION (CFG_POWERLINK_MN "Compile openPOWERLINK MN (Otherwise CN)" ON)
SET(CFG_DEBUG_LVL "0xEC000000L" CACHE STRING "Debug Level for debug output")

CMAKE_DEPENDENT_OPTION (CFG_DEMO_MN_CONSOLE "Build process image based console demo application" ON
                        "CFG_POWERLINK_MN" OFF)
CMAKE_DEPENDENT_OPTION (CFG_DEMO_MN_QT "Build process image based QT demo application" OFF
                        "CFG_POWERLINK_MN;QT4_FOUND" OFF)
CMAKE_DEPENDENT_OPTION (CFG_DEMO_CN_CONSOLE "Build CN console demo application" ON
                        "NOT CFG_POWERLINK_MN" OFF)

CMAKE_DEPENDENT_OPTION (CFG_BUILD_UNITTESTS "Build all unittests and include the necessary switches" OFF
                        "UNIX" OFF)

###############################################################################
# Set global definitions
###############################################################################
SET(POWERLINK_SOURCE_DIR ${CMAKE_SOURCE_DIR}/stack/src)
SET(USER_SOURCE_DIR ${CMAKE_SOURCE_DIR}/stack/src/user)
SET(KERNEL_SOURCE_DIR ${CMAKE_SOURCE_DIR}/stack/src/kernel)
SET(COMMON_SOURCE_DIR ${CMAKE_SOURCE_DIR}/stack/src/common)
SET(ARCH_SOURCE_DIR ${CMAKE_SOURCE_DIR}/stack/src/arch)
SET(EDRV_SOURCE_DIR ${CMAKE_SOURCE_DIR}/stack/src/kernel/edrv)
SET(LIB_SOURCE_DIR ${CMAKE_SOURCE_DIR}/libs)

SET(OBJDICT_DIR ${CMAKE_SOURCE_DIR}/objdicts)

SET (DEMO_DIR ${CMAKE_SOURCE_DIR}/examples)
SET (DEMO_COMMON_DIR ${CMAKE_SOURCE_DIR}/examples/common)

SET(POWERLINK_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/include)
SET(USER_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/include/user)
SET(KERNEL_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/include/kernel)
SET(STACK_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/stack/include)

SET(TOOLS_DIR ${CMAKE_SOURCE_DIR}/tools)

# set global include directories
INCLUDE_DIRECTORIES (
    ${POWERLINK_INCLUDE_DIR}
    ${POWERLINK_SOURCE_DIR}
    ${LIB_SOURCE_DIR}/sharedbuff
    ${LIB_SOURCE_DIR}/circbuf
    ${LIB_SOURCE_DIR}/ami
    ${LIB_SOURCE_DIR}
)

# setup architecture specific configuration
IF (UNIX)
    INCLUDE (cmake/configure-linux.cmake)
ENDIF (UNIX)

IF (WIN32)
    INCLUDE (cmake/configure-windows.cmake)
ENDIF (WIN32)


###############################################################################
# Examples
###############################################################################
# Add subdirectory of MN console demo application
IF (CFG_DEMO_MN_CONSOLE)
    ADD_SUBDIRECTORY(examples/demo_mn_console)
ENDIF (CFG_DEMO_MN_CONSOLE)

# Add subdirectory of MN QT demo application
IF (CFG_DEMO_MN_QT)
    ADD_SUBDIRECTORY(examples/demo_mn_qt)
ENDIF (CFG_DEMO_MN_QT)

# Add subdirectory of CN console demo application
IF (CFG_DEMO_CN_CONSOLE)
    ADD_SUBDIRECTORY(examples/demo_cn_console)
ENDIF (CFG_DEMO_CN_CONSOLE)

###############################################################################
# Unit tests
###############################################################################
IF(CFG_BUILD_UNITTESTS)
  ENABLE_TESTING()
  ADD_SUBDIRECTORY(unittests)
ENDIF(CFG_BUILD_UNITTESTS)
