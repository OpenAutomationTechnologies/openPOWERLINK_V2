################################################################################
#
# Generic CMake options openPOWERLINK demo applications
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

MESSAGE(STATUS "CMAKE_SYSTEM_NAME is ${CMAKE_SYSTEM_NAME}")
MESSAGE(STATUS "CMAKE_SYSTEM_PROCESSOR is ${CMAKE_SYSTEM_PROCESSOR}")

################################################################################
# Set global directories
################################################################################
SET(OPLK_ROOT_DIR ${CMAKE_SOURCE_DIR}/../..)
SET(DEMO_SOURCE_DIR ${CMAKE_SOURCE_DIR}/src)
SET(DEMO_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/include)
SET(COMMON_SOURCE_DIR ${CMAKE_SOURCE_DIR}/../common/src)
SET(OPENCONFIG_PROJ_DIR ${CMAKE_SOURCE_DIR}/../common/openCONFIGURATOR_projects)
SET(CONTRIB_SOURCE_DIR ${OPLK_ROOT_DIR}/contrib)
SET(OPLK_INCLUDE_DIR ${OPLK_ROOT_DIR}/stack/include)
SET(TOOLS_DIR ${OPLK_ROOT_DIR}/tools)
SET(BOARDS_DIR ${OPLK_ROOT_DIR}/hardware/boards)

################################################################################
# Include CMake Modules
################################################################################
SET(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/../common/cmake ${CMAKE_MODULE_PATH})

# include standard cmake modules
INCLUDE(CMakeDependentOption)

# include project specific modules
INCLUDE(findoplklib)

################################################################################
# Set options
################################################################################

STRING(TOLOWER "${CMAKE_SYSTEM_NAME}" SYSTEM_NAME_DIR)
STRING(TOLOWER "${CMAKE_SYSTEM_PROCESSOR}" SYSTEM_PROCESSOR_DIR)

IF(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
  SET(CMAKE_INSTALL_PREFIX
    ${OPLK_ROOT_DIR}/bin/${SYSTEM_NAME_DIR}/${SYSTEM_PROCESSOR_DIR} CACHE PATH "openPOWERLINK apps install prefix" FORCE
    )
ENDIF()

SET(CMAKE_CONFIGURATION_TYPES "Debug;Release" CACHE INTERNAL "Available Build Configurations" FORCE)

IF(NOT CMAKE_GENERATOR MATCHES "Visual Studio")
    IF(NOT CMAKE_BUILD_TYPE)
      SET(CMAKE_BUILD_TYPE Release CACHE STRING
          "Choose the type of build, options are: None Debug Release"
          FORCE)
    ENDIF()
ENDIF()

SET(CFG_DEBUG_LVL "0xEC000000L" CACHE STRING "Debug Level for debug output")

# set global include directories
INCLUDE_DIRECTORIES (
    ${OPLK_INCLUDE_DIR}
    ${CONTRIB_SOURCE_DIR}
    ${COMMON_SOURCE_DIR}
)

IF(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    INCLUDE(configure-linux)
ELSEIF(CMAKE_SYSTEM_NAME STREQUAL "Windows")
    INCLUDE(configure-windows)
ELSEIF(CMAKE_SYSTEM_NAME STREQUAL "Generic" AND CMAKE_SYSTEM_PROCESSOR STREQUAL "Microblaze")
    INCLUDE(configure-microblaze)
ELSEIF(CMAKE_SYSTEM_NAME STREQUAL "Generic" AND CMAKE_SYSTEM_PROCESSOR STREQUAL "ARM_Xilinx")
    INCLUDE(configure-xilinx_arm)
ENDIF()

