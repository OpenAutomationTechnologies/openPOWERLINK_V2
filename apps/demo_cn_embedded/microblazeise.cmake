################################################################################
#
# CMake file of CiA 401 CN embedded demo application (Target is Microblaze)
#
# Copyright (c) 2014, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
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

################################################################################
# Set paths
IF(CFG_KERNEL_STACK_DIRECTLINK)
    SET(XIL_BSP_DIR ${CFG_HW_LIB_DIR}/bsp${CFG_PCP_NAME}/${CFG_PCP_NAME})
    EXECUTE_PROCESS(COMMAND ${CMAKE_COMMAND} -E copy "${CFG_HW_LIB_DIR}/bsp${CFG_PCP_NAME}/lscript.ld" "${PROJECT_BINARY_DIR}")
    SET(LSSCRIPT ${PROJECT_BINARY_DIR}/lscript.ld)
    SET(EXECUTABLE_CPU_NAME ${CFG_PCP_NAME})      # On direct link the CPU name is PCP
    SET(CPU_PREFIX PCP)
ELSE ()
    MESSAGE(FATAL_ERROR "Only CFG_KERNEL_STACK_DIRECTLINK is currently implemented on Microblaze!")
ENDIF()

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
# Set architecture specific sources and include directories

SET(DEMO_ARCH_SOURCES
    ${DEMO_ARCHSOURCES}
    ${COMMON_SOURCE_DIR}/gpio/gpio-microblaze.c
    ${COMMON_SOURCE_DIR}/lcd/lcdl-null.c
   )

INCLUDE_DIRECTORIES(
                    ${XIL_BSP_DIR}/include
                    ${OPLK_BASE_DIR}/stack/src/arch/xilinx-microblaze
                    ${COMMON_SOURCE_DIR}/gpio
                   )

################################################################################
# Set architecture specific definitions
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${XIL_${CPU_PREFIX}_CFLAGS} -fmessage-length=0 -mcpu=${CFG_${CPU_PREFIX}_CPU_VERSION} -ffunction-sections -fdata-sections")

################################################################################
# Set architecture specific linker flags
SET(ARCH_LINKER_FLAGS "${XIL_${CPU_PREFIX}_PLAT_ENDIAN} -mcpu=${CFG_${CPU_PREFIX}_CPU_VERSION} -Wl,-T -Wl,${LSSCRIPT} -Wl,-Map,${PROJECT_NAME}.map" )

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

########################################################################
# Eclipse project files
SET(CFG_CPU_NAME ${EXECUTABLE_CPU_NAME})

GEN_ECLIPSE_FILE_LIST("${DEMO_SOURCES}" "" PART_ECLIPSE_FILE_LIST )
SET(ECLIPSE_FILE_LIST "${ECLIPSE_FILE_LIST} ${PART_ECLIPSE_FILE_LIST}")

GEN_ECLIPSE_FILE_LIST("${DEMO_ARCH_SOURCES}" "" PART_ECLIPSE_FILE_LIST)
SET(ECLIPSE_FILE_LIST "${ECLIPSE_FILE_LIST} ${PART_ECLIPSE_FILE_LIST}")

GET_PROPERTY(DEMO_INCLUDES DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
GEN_ECLIPSE_INCLUDE_LIST("${DEMO_INCLUDES}" ECLIPSE_INCLUDE_LIST)

CONFIGURE_FILE(${XIL_TOOLS_DIR}/eclipse/appproject.in ${PROJECT_BINARY_DIR}/.project @ONLY)
CONFIGURE_FILE(${XIL_TOOLS_DIR}/eclipse/appcproject.in ${PROJECT_BINARY_DIR}/.cproject @ONLY)
