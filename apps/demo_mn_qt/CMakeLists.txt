################################################################################
#
# CMake file of CiA 302 MN QT demo application
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

PROJECT(demo_mn_qt C CXX)
MESSAGE(STATUS "Configuring demo_mn_qt")

CMAKE_MINIMUM_REQUIRED (VERSION 2.8.7)

INCLUDE(${CMAKE_CURRENT_SOURCE_DIR}/../common/cmake/options.cmake)

################################################################################
# Setup project files and definitions

SET(OBJDICT CiA302-4_MN)            # Set object dictionary to use
FIND_OPLK_LIBRARY("mn")             # Find suitable openPOWERLINK library
FIND_PACKAGE(Qt4 REQUIRED)          # Find Qt4

SET(CFG_DEMO_PROJECT "Demo_3CN" CACHE STRING
        "Valid projects are Demo_3CN, Demo_RMN_3CN")
SET_PROPERTY(CACHE CFG_DEMO_PROJECT PROPERTY STRINGS Demo_3CN Demo_RMN_3CN)

SET(CFG_CFM ON CACHE BOOL "Use Configuration Manager (CFM)")

# Set generic sources and include directories for this demo
SET (DEMO_SOURCES
     ${DEMO_SOURCE_DIR}/main.cpp
     ${DEMO_SOURCE_DIR}/MainWindow.cpp
     ${DEMO_SOURCE_DIR}/SdoDialog.cpp
     ${DEMO_SOURCE_DIR}/Api.cpp
     ${DEMO_SOURCE_DIR}/CnState.cpp
     ${DEMO_SOURCE_DIR}/DataInOutThread.cpp
     ${DEMO_SOURCE_DIR}/Input.cpp
     ${DEMO_SOURCE_DIR}/Output.cpp
     ${DEMO_SOURCE_DIR}/ProcessThread.cpp
     ${DEMO_SOURCE_DIR}/State.cpp
     ${DEMO_SOURCE_DIR}/Leds.cpp
     ${DEMO_SOURCE_DIR}/NodeState.cpp
     ${DEMO_SOURCE_DIR}/NmtCommandDialog.cpp
     ${CONTRIB_SOURCE_DIR}/console/printlog.c
     )

SET (DEMO_RESOURCES ${CMAKE_CURRENT_SOURCE_DIR}/resources/resources.qrc)

SET (DEMO_HEADERS
     ${DEMO_INCLUDE_DIR}/Api.h
     ${DEMO_INCLUDE_DIR}/CnState.h
     ${DEMO_INCLUDE_DIR}/DataInOutThread.h
     ${DEMO_INCLUDE_DIR}/Input.h
     ${DEMO_INCLUDE_DIR}/Output.h
     ${DEMO_INCLUDE_DIR}/ProcessThread.h
     ${DEMO_INCLUDE_DIR}/State.h
     ${DEMO_INCLUDE_DIR}/Leds.h
     ${DEMO_INCLUDE_DIR}/MainWindow.h
     ${DEMO_INCLUDE_DIR}/SdoDialog.h
     ${DEMO_INCLUDE_DIR}/NodeState.h
     ${DEMO_INCLUDE_DIR}/NmtCommandDialog.h
     )

INCLUDE_DIRECTORIES(
    ${DEMO_INCLUDE_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}
    )

################################################################################
# Setup compile definitions depending on configuration

IF(CFG_CFM)
    ADD_DEFINITIONS(-DCONFIG_INCLUDE_CFM)
ENDIF()

IF (CFG_KERNEL_STACK_DIRECTLINK OR CFG_KERNEL_STACK_USERSPACE_DAEMON)
    SET (DEMO_SOURCES ${DEMO_SOURCES} ${DEMO_SOURCE_DIR}/InterfaceSelectDialog.cpp)
    SET (DEMO_HEADERS ${DEMO_HEADERS} ${DEMO_INCLUDE_DIR}/InterfaceSelectDialog.h)
    ADD_DEFINITIONS (-DCONFIG_USE_PCAP)
ENDIF (CFG_KERNEL_STACK_DIRECTLINK OR CFG_KERNEL_STACK_USERSPACE_DAEMON)

# Setup compile definitions depending on configuration
IF (CFG_KERNEL_STACK_DIRECTLINK)
    UNSET (CFG_DEMO_MN_QT_USE_SYNCTHREAD CACHE)
    ADD_DEFINITIONS(-DCONFIG_KERNELSTACK_DIRECTLINK)
ELSE (CFG_KERNEL_STACK_DIRECTLINK)
    OPTION (CFG_DEMO_MN_QT_USE_SYNCTHREAD "Create separate thread for syncronous data exchange" ON)
    IF (CFG_DEMO_MN_QT_USE_SYNCTHREAD)
        ADD_DEFINITIONS(-DCONFIG_USE_SYNCTHREAD)
    ENDIF (CFG_DEMO_MN_QT_USE_SYNCTHREAD)
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

INCLUDE (${QT_USE_FILE})
ADD_DEFINITIONS (${QT_DEFINITIONS} -DCONFIG_MN)

################################################################################
#

# Setup QT4 Wrappers
QT4_WRAP_CPP (DEMO_HEADERS_MOC ${DEMO_HEADERS})
QT4_WRAP_UI (DEMO_FORMS_HEADERS ${DEMO_FORMS})
QT4_ADD_RESOURCES (DEMO_RESOURCES_RCC ${DEMO_RESOURCES})

################################################################################
# Group Source Files

SOURCE_GROUP("Source Files" FILES ${DEMO_SOURCES})
SOURCE_GROUP("QT-generated Files" FILES ${DEMO_HEADERS_MOC} ${DEMO_RESOURCES_RCC})
SOURCE_GROUP("Header Files" FILES ${DEMO_HEADERS})
SOURCE_GROUP("openPOWERLINK Header Files" FILES ${OPLK_HEADERS})

################################################################################
# Set the executable
ADD_EXECUTABLE (demo_mn_qt ${DEMO_SOURCES} ${DEMO_ARCH_SOURCES} ${DEMO_HEADERS} ${DEMO_HEADERS_MOC}
                           ${DEMO_FORMS_HEADERS} ${DEMO_RESOURCES_RCC} ${CMAKE_CURRENT_BINARY_DIR}/mnobd.cdc)
SET_PROPERTY(TARGET demo_mn_qt
             PROPERTY COMPILE_DEFINITIONS_DEBUG DEBUG;DEF_DEBUG_LVL=${CFG_DEBUG_LVL})

ADD_CUSTOM_COMMAND(OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/mnobd.cdc
                   COMMAND ${CMAKE_COMMAND} -E copy ${OPENCONFIG_PROJ_DIR}/${CFG_DEMO_PROJECT}/output/mnobd.cdc ${CMAKE_CURRENT_BINARY_DIR}/mnobd.cdc
                   DEPENDS ${OPENCONFIG_PROJ_DIR}/${CFG_DEMO_PROJECT}/output/mnobd.cdc
                   VERBATIM
                   )

################################################################################
# Libraries to link

OPLK_LINK_LIBRARIES(demo_mn_qt)
TARGET_LINK_LIBRARIES(demo_mn_qt ${ARCH_LIBRARIES} ${QT_LIBRARIES})

################################################################################
# Installation rules

INSTALL(TARGETS demo_mn_qt RUNTIME DESTINATION ${PROJECT_NAME})
INSTALL(FILES ${CMAKE_CURRENT_BINARY_DIR}/mnobd.cdc DESTINATION ${PROJECT_NAME})
