################################################################################
#
# CMake file for openPOWERLINK xilinx-microblaze PCP driver
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

PROJECT(oplkdrv_daemon C)

MESSAGE(STATUS "CMAKE_SYSTEM_NAME is ${CMAKE_SYSTEM_NAME}")
MESSAGE(STATUS "CMAKE_SYSTEM_PROCESSOR is ${CMAKE_SYSTEM_PROCESSOR}")

IF(CMAKE_SYSTEM_NAME STREQUAL "Windows")
    # ensure working support of VS.NET 2010
    CMAKE_MINIMUM_REQUIRED (VERSION 2.8.4)
ELSE ()
    CMAKE_MINIMUM_REQUIRED (VERSION 2.8.0)
ENDIF()

STRING(TOLOWER "${CMAKE_SYSTEM_NAME}" SYSTEM_NAME_DIR)
STRING(TOLOWER "${CMAKE_SYSTEM_PROCESSOR}" SYSTEM_PROCESSOR_DIR)

MESSAGE(STATUS "Configuring ${EXE_NAME}")

###############################################################################
# Set global directories
###############################################################################
SET(OPLK_ROOT_DIR ${CMAKE_SOURCE_DIR}/../../..)
SET(DRV_SOURCE_DIR ${CMAKE_SOURCE_DIR})
SET(CONTRIB_SOURCE_DIR ${OPLK_ROOT_DIR}/contrib)
SET(OPLK_INCLUDE_DIR ${OPLK_ROOT_DIR}/stack/include)
SET(TOOLS_DIR ${OPLK_ROOT_DIR}/tools)
SET(XIL_TOOLS_DIR ${TOOLS_DIR}/xilinx-microblaze)

###############################################################################
# Include CMake Modules
###############################################################################
SET(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/../../cmake ${CMAKE_MODULE_PATH})

# include standard cmake modules
INCLUDE(CMakeDependentOption)

IF(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
  SET(CMAKE_INSTALL_PREFIX
    ${OPLK_ROOT_DIR}/bin/${SYSTEM_NAME_DIR}/${SYSTEM_PROCESSOR_DIR} CACHE PATH "openPOWERLINK drivers install prefix" FORCE
    )
ENDIF(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release CACHE STRING
      "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
      FORCE)
ENDIF(NOT CMAKE_BUILD_TYPE)

SET(HOST_PCP_LINK_TYPE "host-interface" CACHE STRING
        "Valid link between host and PCP is host-interface")
SET_PROPERTY(CACHE HOST_PCP_LINK_TYPE PROPERTY STRINGS "host-interface")

OPTION (CFG_OPLK_MN "Compile openPOWERLINK MN driver (Otherwise CN)" ON)

SET(CFG_DEBUG_LVL "0xEC000000L" CACHE STRING "Debug Level for debug output")

# select the stack libary and search for it
IF(CFG_OPLK_MN)
    ADD_DEFINITIONS(-DCONFIG_MN)

    IF("${HOST_PCP_LINK_TYPE}" STREQUAL "host-interface")
        SET(LIB_NAME oplkmndrv-hostif)
    ENDIF()

ELSE()
    IF("${HOST_PCP_LINK_TYPE}" STREQUAL "host-interface")
        MESSAGE(FATAL_ERROR "PCP driver for dual processor CN is not yet available with ${HOST_PCP_LINK_TYPE}")
    ENDIF()
ENDIF()


################################################################################
# Handle includes
SET(CMAKE_MODULE_PATH "${OPLK_ROOT_DIR}/cmake" ${CMAKE_MODULE_PATH})
INCLUDE(geneclipsefilelist)
INCLUDE(geneclipseincludelist)
INCLUDE(setmicroblazeboardconfig)
################################################################################
# Path to the hardware library folder of your board example

IF("${HOST_PCP_LINK_TYPE}" STREQUAL "host-interface")

    SET(CFG_HW_LIB_DIR ${OPLK_ROOT_DIR}/hardware/lib/${SYSTEM_NAME_DIR}/${SYSTEM_PROCESSOR_DIR}/avnet-lx150t/mn-dual-hostif-gpio
        CACHE PATH "Path to the hardware library folder of your demo application")

ELSE()
    UNSET(CFG_HW_LIB_DIR CACHE)
ENDIF ()

################################################################################
# Include board specific settings file
	SET_BOARD_CONFIGURATION(${CFG_HW_LIB_DIR})

################################################################################
# Set variables
SET(ARCH_EXE_SUFFIX ".elf")
SET(ARCH_INSTALL_POSTFIX ${CFG_DEMO_BOARD_NAME}/${CFG_DEMO_NAME})

################################################################################
# Set paths

SET(XIL_BSP_DIR ${CFG_HW_LIB_DIR}/bsp${CFG_PCP_NAME}/${CFG_PCP_NAME})
FILE(COPY ${CFG_HW_LIB_DIR}/bsp${CFG_PCP_NAME}/lscript.ld DESTINATION ${PROJECT_BINARY_DIR})
SET(LSSCRIPT ${PROJECT_BINARY_DIR}/lscript.ld)
SET(DEMO_CPU_NAME ${CFG_PCP_NAME})

################################################################################
# Find boards support package
UNSET(XIL_LIB_BSP CACHE)
MESSAGE(STATUS "Searching for the board support package in ${XIL_BSP_DIR}")
FIND_LIBRARY(XIL_LIB_BSP NAME xil
                     HINTS ${XIL_BSP_DIR}/lib
            )

################################################################################
# Find driver omethlib

IF(${CMAKE_BUILD_TYPE} STREQUAL "Debug")
    SET(LIB_OMETHLIB_NAME "omethlib_d")
ELSE()
    SET(LIB_OMETHLIB_NAME "omethlib")
ENDIF()

UNSET(XIL_LIB_OMETH CACHE)
MESSAGE(STATUS "Searching for LIBRARY ${LIB_OMETHLIB_NAME} in ${CFG_HW_LIB_DIR}/libomethlib")
    FIND_LIBRARY(XIL_LIB_OMETH NAMES ${LIB_OMETHLIB_NAME}
                         HINTS ${CFG_HW_LIB_DIR}/libomethlib
                )

################################################################################
# Find driver for host-pcp interface
                
IF("${HOST_PCP_LINK_TYPE}" STREQUAL "host-interface")
    
    IF(${CMAKE_BUILD_TYPE} STREQUAL "Debug")
        SET(LIB_HOSTIFLIB_NAME "hostiflib-pcp_d")
    ELSE()
        SET(LIB_HOSTIFLIB_NAME "hostiflib-pcp")
    ENDIF()

    UNSET(XIL_LIB_HOSTIF CACHE)
    MESSAGE(STATUS "Searching for LIBRARY ${LIB_HOSTIFLIB_NAME} in ${CFG_HW_LIB_DIR}/libhostiflib-pcp")
    FIND_LIBRARY(XIL_LIB_HOSTIF NAMES ${LIB_HOSTIFLIB_NAME}
                         HINTS ${CFG_HW_LIB_DIR}/libhostiflib-pcp
            )

ENDIF()

################################################################################
# Set architecture specific sources and include directories

# set sources of POWERLINK application
SET(DEMO_SOURCES
    ${DRV_SOURCE_DIR}/daemon.c
    )

INCLUDE_DIRECTORIES(
    ${OPLK_INCLUDE_DIR}
    ${CONTRIB_SOURCE_DIR}
    )

SET(DEMO_ARCH_SOURCES
    ${DEMO_ARCH_SOURCES}
    ${OPLK_ROOT_DIR}/stack/src/common/debugstr.c 
   )

INCLUDE_DIRECTORIES(
                    ${XIL_BSP_DIR}/include
                    ${OPLK_ROOT_DIR}/stack/src/arch/xilinx_microblaze
                   )
################################################################################
# Set architecture specific libraries

IF(NOT ${XIL_LIB_BSP} STREQUAL "XIL_LIB_BSP-NOTFOUND" )
    SET(ARCH_LIBRARIES  ${ARCH_LIBRARIES} ${XIL_LIB_BSP})

    LINK_DIRECTORIES(${XIL_BSP_DIR}/lib)
ELSE()
    MESSAGE(FATAL_ERROR "Board support package for board ${CFG_DEMO_BOARD_NAME} and demo ${CFG_DEMO_NAME} not found!")
ENDIF()            
            
IF(NOT ${XIL_LIB_OMETH} STREQUAL "XIL_LIB_OMETH-NOTFOUND")
    SET(ARCH_LIBRARIES  ${ARCH_LIBRARIES} ${XIL_LIB_OMETH})
ELSE()
    MESSAGE(FATAL_ERROR "${LIB_OMETHLIB_NAME} for board ${CFG_DEMO_BOARD_NAME} and demo ${CFG_DEMO_NAME} not found! Check the parameter CMAKE_BUILD_TYPE to confirm your 'Debug' or 'Release' settings")
ENDIF()

IF("${HOST_PCP_LINK_TYPE}" STREQUAL "host-interface")

    IF(NOT ${XIL_LIB_HOSTIF} STREQUAL "XIL_LIB_HOSTIF-NOTFOUND")
        SET(ARCH_LIBRARIES  ${ARCH_LIBRARIES} ${XIL_LIB_HOSTIF})
    ELSE()
        MESSAGE(FATAL_ERROR "${LIB_HOSTIFLIB_NAME} for board ${CFG_DEMO_BOARD_NAME} and demo ${CFG_DEMO_NAME} not found! Check the parameter CMAKE_BUILD_TYPE to confirm your 'Debug' or 'Release' settings")
    ENDIF()

ENDIF()

SET(OPLKLIB_DIR ${OPLK_ROOT_DIR}/stack/lib/${SYSTEM_NAME_DIR}/${SYSTEM_PROCESSOR_DIR}/${CFG_DEMO_BOARD_NAME}/${CFG_DEMO_NAME})
SET(OPLKLIB_INCDIR ${OPLK_ROOT_DIR}/stack/proj/${SYSTEM_NAME_DIR}/lib${LIB_NAME})

UNSET(OPLKLIB CACHE)
MESSAGE(STATUS "Searching for LIBRARY lib${LIB_NAME} in ${OPLKLIB_DIR}")
FIND_LIBRARY(OPLKLIB NAME ${LIB_NAME}
                     HINTS ${OPLKLIB_DIR})

MESSAGE(STATUS "Searching for LIBRARY lib${LIB_NAME}_d in ${OPLKLIB_DIR}")
FIND_LIBRARY(OPLKLIB_DEBUG NAME ${LIB_NAME}_d
                    HINTS ${OPLKLIB_DIR})
                    
INCLUDE_DIRECTORIES(${OPLKLIB_INCDIR})

################################################################################
# Set architecture specific definitions
ADD_DEFINITIONS(${XIL_CFLAGS} "-fmessage-length=0 -mcpu=${CFG_CPU_VERSION} -ffunction-sections -fdata-sections")

################################################################################
# Set architecture specific linker flags
SET(ARCH_LINKER_FLAGS "${XIL_PLAT_ENDIAN} -mcpu=${CFG_CPU_VERSION} -Wl,-T -Wl,${LSSCRIPT} -Wl,-Map,${PROJECT_NAME}.map" )

########################################################################
# Eclipse project files
GEN_ECLIPSE_FILE_LIST("${DEMO_SOURCES}" "" PART_ECLIPSE_FILE_LIST )
SET(ECLIPSE_FILE_LIST "${ECLIPSE_FILE_LIST} ${PART_ECLIPSE_FILE_LIST}")

GEN_ECLIPSE_FILE_LIST("${DEMO_ARCH_SOURCES}" "" PART_ECLIPSE_FILE_LIST)
SET(ECLIPSE_FILE_LIST "${ECLIPSE_FILE_LIST} ${PART_ECLIPSE_FILE_LIST}")

GET_PROPERTY(DEMO_INCLUDES DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
GEN_ECLIPSE_INCLUDE_LIST("${DEMO_INCLUDES}" ECLIPSE_INCLUDE_LIST)

CONFIGURE_FILE(${XIL_TOOLS_DIR}/eclipse/appproject.in ${PROJECT_BINARY_DIR}/.project @ONLY)
CONFIGURE_FILE(${XIL_TOOLS_DIR}/eclipse/appcproject.in ${PROJECT_BINARY_DIR}/.cproject @ONLY)


SET(DEMO_NAME "${PROJECT_NAME}${ARCH_EXE_SUFFIX}")

################################################################################
# Set the executable
ADD_EXECUTABLE(${DEMO_NAME} ${DEMO_SOURCES} ${DEMO_ARCH_SOURCES})
SET_PROPERTY(TARGET ${DEMO_NAME}
             PROPERTY COMPILE_DEFINITIONS_DEBUG DEBUG;DEF_DEBUG_LVL=${CFG_DEBUG_LVL})

SET_PROPERTY(TARGET ${DEMO_NAME}
             PROPERTY LINK_FLAGS ${ARCH_LINKER_FLAGS})

################################################################################
# Libraries to link

TARGET_LINK_LIBRARIES(${DEMO_NAME} optimized ${OPLKLIB} debug ${OPLKLIB_DEBUG})
TARGET_LINK_LIBRARIES(${DEMO_NAME} ${ARCH_LIBRARIES})

################################################################################
# Post Build targets
################################################################################
# U S E R   O P T I O N S

# Elf verify enable
OPTION(XIL_VERIFY_ELF "Verify the executable after download" OFF)
MARK_AS_ADVANCED(XIL_VERIFY_ELF)

##############################################################################
# Set paths
SET(XIL_HW_SPEC ${CFG_HW_LIB_DIR}/hw_platform)
SET(XIL_XPS_DEMO_DIR ${CFG_DEMO_DIR}/xps)

##############################################################################
# Demo post build action
ADD_CUSTOM_COMMAND(
    TARGET ${DEMO_NAME}
    POST_BUILD
    COMMAND mb-size ${DEMO_NAME} | tee "${DEMO_NAME}.size"
    COMMAND elfcheck ${DEMO_NAME} -hw ${XIL_HW_SPEC}/system.xml -pe ${DEMO_CPU_NAME} | tee "${DEMO_NAME}.elfcheck"
    COMMAND data2mem -bd ${DEMO_NAME} -d -o m ${DEMO_NAME}.mem
)

SET_DIRECTORY_PROPERTIES(PROPERTIES
                         ADDITIONAL_MAKE_CLEAN_FILES "${DEMO_NAME}.elf;${DEMO_NAME}.size;${DEMO_NAME}.elfcheck;${DEMO_NAME}.mem;${DEMO_NAME}.tmp;${DEMO_NAME}.map"
                        )

##############################################################################
# Add targets for the download of the bitstream
FIND_PROGRAM(XIL_MB_XMD NAMES xmd
             DOC "Xilinx Microblaze Debug"
            )

FIND_PROGRAM(XIL_IMPACT NAMES impact
             DOC "Xilinx iMPACT"
            )

IF(NOT ${XIL_IMPACT} STREQUAL "XIL_IMPACT-NOTFOUND")
    ADD_CUSTOM_TARGET(
            download-bits
            COMMAND ${XIL_IMPACT} -batch download.cmd
            WORKING_DIRECTORY ${XIL_HW_SPEC}
    )
ENDIF()

IF(NOT ${XIL_MB_XMD} STREQUAL "XIL_MB_XMD-NOTFOUND")
    ADD_CUSTOM_TARGET(
            download-elf
            COMMAND ${XIL_MB_XMD} -hw ${XIL_HW_SPEC}/system.xml -tcl ${XIL_TOOLS_DIR}/xmd-downloadelf.tcl ${DEMO_NAME} ${XIL_VERIFY_ELF}
    )
ENDIF()

################################################################################
# Set architecture specific installation files
INSTALL(FILES ${XIL_TOOLS_DIR}/xmd-downloadelf.tcl ${XIL_HW_SPEC}/download.bit ${XIL_HW_SPEC}/download.cmd ${XIL_HW_SPEC}/system.xml
        DESTINATION ${ARCH_INSTALL_POSTFIX}
       )
INSTALL(PROGRAMS ${XIL_TOOLS_DIR}/elfdownload.make
        DESTINATION ${ARCH_INSTALL_POSTFIX} RENAME Makefile
       )
################################################################################
# Installation rules

INSTALL(TARGETS ${DEMO_NAME} RUNTIME DESTINATION ${CFG_DEMO_BOARD_NAME}/${CFG_DEMO_NAME})
