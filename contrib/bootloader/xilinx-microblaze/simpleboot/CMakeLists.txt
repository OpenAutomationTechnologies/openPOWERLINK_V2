################################################################################
#
# CMake file of the bootloader simpleboot
#
# Copyright (c) 2014, B&R Industrial Automation GmbH
# Copyright (c) 2016, Kalycito Infotech Private Limited
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

CMAKE_MINIMUM_REQUIRED (VERSION 2.8.7)

################################################################################
# Setup project and generic options
PROJECT(simpleboot C)

ENABLE_LANGUAGE(ASM-ATT)
SET(CMAKE_ASM-ATT_FLAGS_INIT "")

MESSAGE(STATUS "CMAKE_SYSTEM_NAME is ${CMAKE_SYSTEM_NAME}")
MESSAGE(STATUS "CMAKE_SYSTEM_PROCESSOR is ${CMAKE_SYSTEM_PROCESSOR}")

################################################################################
# Set variables
STRING(TOLOWER "${CMAKE_SYSTEM_NAME}" SYSTEM_NAME_DIR)
STRING(TOLOWER "${CMAKE_SYSTEM_PROCESSOR}" SYSTEM_PROCESSOR_DIR)

SET(OPLK_BASE_DIR ${PROJECT_SOURCE_DIR}/../../../../)
SET(TOOLS_DIR ${OPLK_BASE_DIR}/tools)

SET(CMAKE_CONFIGURATION_TYPES "None;Debug;Release;RelWithDebInfo;MinSizeRel"
    CACHE INTERNAL "Available Build Configurations" FORCE)

IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Release CACHE STRING
        "Choose the type of build"
        FORCE)
    SET_PROPERTY(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "None;Debug;Release;RelWithDebInfo;MinSizeRel")
ENDIF()

################################################################################
# Set sources of the bootloader
SET(BOOT_C_SRCS
    ${PROJECT_SOURCE_DIR}/src/main.c
    ${PROJECT_SOURCE_DIR}/src/spi.c
    ${PROJECT_SOURCE_DIR}/src/util.c
   )

SET(BOOT_ASM_SRCS
    ${PROJECT_SOURCE_DIR}/src/crt3.s
    ${PROJECT_SOURCE_DIR}/src/crtinit.s
    ${PROJECT_SOURCE_DIR}/src/init.s
   )

################################################################################
# Set includes of the bootloader
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/src
                   )

IF((CMAKE_SYSTEM_NAME STREQUAL "Generic") AND (CMAKE_SYSTEM_PROCESSOR STREQUAL "Microblazeise"))
    INCLUDE(configure-microblazeise.cmake)
ELSE()
    MESSAGE(FATAL_ERROR "unecpected: simpleboot only ported to target CMAKE_SYSTEM_PROCESSOR = Microblaze!")
ENDIF()

SET(EXECUTABLE_NAME ${ARCH_EXECUTABLE_NAME})

################################################################################
# Set the executable
ADD_EXECUTABLE(${EXECUTABLE_NAME} ${BOOT_C_SRCS} ${BOOT_ASM_SRCS})

# Set flags to source files
SET(CFLAGS "-Wall -Wextra -pedantic -std=c99")

SET_SOURCE_FILES_PROPERTIES(${BOOT_C_SRCS} PROPERTIES COMPILE_FLAGS "${CFLAGS} ${ARCH_CFLAGS}")
SET_SOURCE_FILES_PROPERTIES(${BOOT_ASM_SRCS} PROPERTIES COMPILE_FLAGS ${ARCH_ASMFLAGS})

SET_TARGET_PROPERTIES(${EXECUTABLE_NAME} PROPERTIES LINK_FLAGS ${ARCH_LINKERFLAGS})

TARGET_LINK_LIBRARIES(${EXECUTABLE_NAME} ${ARCH_LIBS})

################################################################################
# Post Build targets
IF(CMAKE_SYSTEM_NAME STREQUAL "Generic" AND CMAKE_SYSTEM_PROCESSOR STREQUAL "Microblazeise")
    INCLUDE(microblazeapppostactionsise)
ENDIF()

################################################################################
# Add additional clean files
SET_DIRECTORY_PROPERTIES(PROPERTIES ADDITIONAL_MAKE_CLEAN_FILES "${ADD_CLEAN_FILES}")

################################################################################
# Installation rules
INSTALL(TARGETS ${EXECUTABLE_NAME} RUNTIME DESTINATION ${ARCH_INSTALL_POSTFIX})
