################################################################################
#
# CMake configuration for openPOWERLINK MN application/host
# library on Xilinx Microblaze
#
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
# Include demo specific settings file
INCLUDE(setmicroblazeiseboardconfig)

SET_BOARD_CONFIGURATION(${CFG_COMPILE_LIB_MN_APP_HOSTIF_HW_LIB_DIR})

################################################################################
# Set paths
SET(ARCH_INSTALL_POSTFIX ${CFG_DEMO_BOARD_NAME}/${CFG_DEMO_NAME})

################################################################################
# Find boards support package
SET(XIL_BSP_DIR ${CFG_COMPILE_LIB_MN_APP_HOSTIF_HW_LIB_DIR}/bsp${CFG_HOST_NAME}/${CFG_HOST_NAME})

MESSAGE(STATUS "Searching for the board support package in ${XIL_BSP_DIR}")
IF (EXISTS ${XIL_BSP_DIR})
    SET(XIL_LIB_BSP_INC ${XIL_BSP_DIR}/include)
ELSE ()
    MESSAGE(FATAL_ERROR "Board support package for board ${CFG_DEMO_BOARD_NAME} and demo ${CFG_DEMO_NAME} not found!")
ENDIF ()

################################################################################
# Set architecture specific sources
SET(LIB_ARCH_SOURCES
                     ${TARGET_MICROBLAZE_SOURCES}
                     ${TARGET_MICROBLAZE_DUAL_SOURCES}
                     ${SDO_SOCKETWRAPPER_SOURCES}
    )

################################################################################
# Set architecture specific includes
INCLUDE_DIRECTORIES(
                    ${XIL_LIB_BSP_INC}
                    ${ARCH_SOURCE_DIR}/xilinx-microblaze
                    ${CONTRIB_SOURCE_DIR}/socketwrapper
                    ${CFG_COMPILE_LIB_MN_APP_HOSTIF_HW_LIB_DIR}/libhostiflib-host/include
                    ${CFG_COMPILE_LIB_MN_APP_HOSTIF_HW_LIB_DIR}/include
                   )

################################################################################
# Set additional target specific compile flags
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${XIL_HOST_CFLAGS} -fmessage-length=0 -mcpu=${CFG_HOST_CPU_VERSION} -ffunction-sections -fdata-sections")

################################################################################
# Deactivate optimization for usleep
SET_SOURCE_FILES_PROPERTIES(${ARCH_SOURCE_DIR}/xilinx-microblaze/usleep.c
                            PROPERTIES COMPILE_FLAGS "-O0")

########################################################################
# Eclipse project files

GEN_ECLIPSE_FILE_LIST("${LIB_USER_SOURCES}" "user" PART_ECLIPSE_FILE_LIST)
SET(ECLIPSE_FILE_LIST "${ECLIPSE_FILE_LIST} ${PART_ECLIPSE_FILE_LIST}")

GEN_ECLIPSE_FILE_LIST("${LIB_COMMON_SOURCES}" "common" PART_ECLIPSE_FILE_LIST)
SET(ECLIPSE_FILE_LIST "${ECLIPSE_FILE_LIST} ${PART_ECLIPSE_FILE_LIST}")

GEN_ECLIPSE_FILE_LIST("${LIB_ARCH_SOURCES}" "arch" PART_ECLIPSE_FILE_LIST)
SET(ECLIPSE_FILE_LIST "${ECLIPSE_FILE_LIST} ${PART_ECLIPSE_FILE_LIST}")

GET_PROPERTY(LIBRARY_INCLUDES DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
GEN_ECLIPSE_INCLUDE_LIST("${LIBRARY_INCLUDES}" ECLIPSE_INCLUDE_LIST)

CONFIGURE_FILE(${XIL_TOOLS_DIR}/eclipse/libproject.in ${PROJECT_BINARY_DIR}/.project @ONLY)
CONFIGURE_FILE(${XIL_TOOLS_DIR}/eclipse/libcproject.in ${PROJECT_BINARY_DIR}/.cproject @ONLY)
