################################################################################
#
# CMake file of CiA 401 CN console demo application
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

PROJECT(demo_cn_console C)
MESSAGE(STATUS "Configuring demo_cn_console")

CMAKE_MINIMUM_REQUIRED (VERSION 2.8.7)

INCLUDE(${CMAKE_CURRENT_SOURCE_DIR}/../common/cmake/options.cmake)

################################################################################
# Setup project files and definitions

SET(OBJDICT CiA401_CN)              # Set object dictionary to use
FIND_OPLK_LIBRARY("cn")             # Find suitable openPOWERLINK library

# Set generic sources and include directories for this demo
SET(DEMO_SOURCES
    ${DEMO_SOURCE_DIR}/main.c
    ${DEMO_SOURCE_DIR}/app.c
    ${DEMO_SOURCE_DIR}/event.c
    ${CONTRIB_SOURCE_DIR}/console/printlog.c
    ${CONTRIB_SOURCE_DIR}/getopt/getopt.c
    )

INCLUDE_DIRECTORIES(
    ${DEMO_SOURCE_DIR}
    )

################################################################################
# Setup compile definitions depending on configuration

# PCAP is used for directlink of userspace daemon
IF (CFG_KERNEL_STACK_DIRECTLINK OR CFG_KERNEL_STACK_USERSPACE_DAEMON)
    ADD_DEFINITIONS(-DCONFIG_USE_PCAP)
    SET (DEMO_SOURCES ${DEMO_SOURCES} ${COMMON_SOURCE_DIR}/pcap/pcap-console.c)
ENDIF (CFG_KERNEL_STACK_DIRECTLINK OR CFG_KERNEL_STACK_USERSPACE_DAEMON)

# Setup compile definitions depending on configuration
IF (CFG_KERNEL_STACK_DIRECTLINK)
    UNSET (CFG_DEMO_CN_CONSOLE_USE_SYNCTHREAD CACHE)
    ADD_DEFINITIONS(-DCONFIG_KERNELSTACK_DIRECTLINK)
ELSE (CFG_KERNEL_STACK_DIRECTLINK)
    OPTION (CFG_DEMO_CN_CONSOLE_USE_SYNCTHREAD "Create separate thread for syncronous data exchange" ON)
    IF (CFG_DEMO_CN_CONSOLE_USE_SYNCTHREAD)
        ADD_DEFINITIONS(-DCONFIG_USE_SYNCTHREAD)
    ENDIF (CFG_DEMO_CN_CONSOLE_USE_SYNCTHREAD)
ENDIF (CFG_KERNEL_STACK_DIRECTLINK)


################################################################################
# Setup the architecture specific definitions

IF(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    include(linux.cmake)
ELSEIF(CMAKE_SYSTEM_NAME STREQUAL "Windows")
    include (windows.cmake)
ELSE()
    MESSAGE(FATAL_ERROR "System ${CMAKE_SYSTEM_NAME} is not supported!")
ENDIF()

################################################################################
# Group Source Files

SOURCE_GROUP("Demo Sources" FILES ${DEMO_SOURCES})
SOURCE_GROUP("Architecture Specific Sources" FILES ${DEMO_ARCH_SOURCES})
SOURCE_GROUP("openPOWERLINK Header Files" FILES ${OPLK_HEADERS})

################################################################################
# Set the executable

ADD_EXECUTABLE(demo_cn_console ${DEMO_SOURCES} ${DEMO_ARCH_SOURCES})
SET_PROPERTY(TARGET demo_cn_console
             PROPERTY COMPILE_DEFINITIONS_DEBUG DEBUG;DEF_DEBUG_LVL=${CFG_DEBUG_LVL})

################################################################################
# Libraries to link

OPLK_LINK_LIBRARIES(demo_cn_console)
TARGET_LINK_LIBRARIES(demo_cn_console ${ARCH_LIBRARIES})

################################################################################
# Installation rules

INSTALL(TARGETS demo_cn_console RUNTIME DESTINATION ${PROJECT_NAME})


