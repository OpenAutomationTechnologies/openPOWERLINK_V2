################################################################################
#
# CMake file of CiA302 MN embedded demo application
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
################################################################################

################################################################################
# Setup project and generic options
PROJECT(demo_mn_embedded C)
MESSAGE(STATUS "Configuring ${PROJECT_NAME}")

IF (CMAKE_SYSTEM_NAME STREQUAL "Windows")
    # ensure working support of VS.NET 2010
    CMAKE_MINIMUM_REQUIRED (VERSION 2.8.4)
ELSE ()
    CMAKE_MINIMUM_REQUIRED (VERSION 2.8.0)
ENDIF ()

INCLUDE(../common/cmake/options.cmake)

################################################################################
# Setup project files and definitions

SET(OBJDICT CiA302-4_MN)              # Set object dictionary to use
FIND_OPLK_LIBRARY("mn")

# Set generic sources and include directories for this demo
SET(DEMO_SOURCES
    ${DEMO_SOURCE_DIR}/main.c
    ${DEMO_SOURCE_DIR}/app.c
    ${DEMO_SOURCE_DIR}/event.c
    ${COMMON_SOURCE_DIR}/lcd/lcd.c
    )

INCLUDE_DIRECTORIES(
                    ${DEMO_SOURCE_DIR}
                    ${COMMON_SOURCE_DIR}/lcd
                   )

################################################################################
# Setup the architecture specific definitions

IF (CMAKE_SYSTEM_NAME STREQUAL "Generic" AND CMAKE_SYSTEM_PROCESSOR STREQUAL "Microblaze")
    include(microblaze.cmake)
ELSE ()
    MESSAGE(FATAL_ERROR "System ${CMAKE_SYSTEM_NAME} with processor ${CMAKE_SYSTEM_PROCESSOR} is not supported!")
ENDIF ()

SET(EXECUTABLE_NAME "${PROJECT_NAME}${ARCH_EXE_SUFFIX}")

################################################################################
# Setup compile definitions depending on configuration

IF (CFG_KERNEL_STACK_DIRECTLINK)
    ADD_DEFINITIONS(-DCONFIG_KERNELSTACK_DIRECTLINK)
ENDIF (CFG_KERNEL_STACK_DIRECTLINK)

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
# Add additional clean files
SET_DIRECTORY_PROPERTIES( PROPERTIES ADDITIONAL_MAKE_CLEAN_FILES "${ADD_CLEAN_FILES}" )

################################################################################
# Installation rules
INSTALL(TARGETS ${EXECUTABLE_NAME} RUNTIME DESTINATION ${CFG_DEMO_BOARD_NAME}/${CFG_DEMO_NAME})
