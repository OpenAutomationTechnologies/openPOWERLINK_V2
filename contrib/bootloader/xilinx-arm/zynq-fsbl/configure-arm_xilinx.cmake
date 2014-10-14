################################################################################
#
# CMake file of the bootloader zynq-fsbl on target Xilinx ARM
#
# Copyright (c) 2014, Kalycito Infotech Private Limited.
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

SET(CMAKE_MODULE_PATH "${OPLK_BASE_DIR}/cmake" ${CMAKE_MODULE_PATH})

################################################################################
# Set includes
INCLUDE(geneclipsefilelist)
INCLUDE(geneclipseincludelist)
INCLUDE(setxilinxarma9boardconfig)

################################################################################
# Set paths
SET(XIL_HW_LIB_DIR ${OPLK_BASE_DIR}/hardware/lib/${SYSTEM_NAME_DIR}/${SYSTEM_PROCESSOR_DIR})
SET(XIL_BIN_DIR ${OPLK_BASE_DIR}/bin/${SYSTEM_NAME_DIR}/${SYSTEM_PROCESSOR_DIR})
SET(XIL_LSCRIPT ${XIL_ZYNQ_FSBL_DIR}/src/lscript.ld)
################################################################################
# Set install prefix to bin export folder
SET(CMAKE_INSTALL_PREFIX
    ${XIL_BIN_DIR} CACHE PATH "bootloader install prefix" FORCE
    )

################################################################################
# U S E R   O P T I O N S

# Path to the hardware library folder of your board example
SET(CFG_HW_LIB_DIR ${XIL_HW_LIB_DIR}/xilinx-z702/mn-dual-shmem-gpio
    CACHE PATH "Path to the hardware library folder for the board example you want to use")

SET(CFG_BIN_DIR "xilinx-z702/mn-dual-shmem-gpio"
    CACHE PATH "Path to the target binary folder of your example design")

SET(ARCH_INSTALL_POSTFIX ${CFG_BIN_DIR})
# Include board specific settings file
SET_BOARD_CONFIGURATION(${CFG_HW_LIB_DIR})

# FSBL can work only on ARM on Zynq
SET(EXECUTABLE_CPU_NAME ${CFG_HOST_NAME})

###############################################################################
# Set paths to configuration
SET(XIL_BSP_DIR ${CFG_HW_LIB_DIR}/bsp${EXECUTABLE_CPU_NAME})
SET(XIL_HW_PLATFORM_DIR ${CFG_HW_LIB_DIR}/hw_platform)
SET(XIL_TOOLS_DIR ${TOOLS_DIR}/xilinx-arm)

################################################################################
# Verify Hardware Platform
IF(NOT EXISTS "${XIL_HW_PLATFORM_DIR}" OR NOT IS_DIRECTORY "${XIL_HW_PLATFORM_DIR}")
    MESSAGE(FATAL_ERROR "unexpected: There is no Hardware platform. Please generate the hardware project first!")
ELSE()
    MESSAGE(STATUS "Found Hardware platform in folder: ${XIL_HW_PLATFORM_DIR}")
ENDIF()

################################################################################
# Verify board support package
IF(NOT EXISTS "${XIL_BSP_DIR}" OR NOT IS_DIRECTORY "${XIL_BSP_DIR}")
    MESSAGE(FATAL_ERROR "unexpected: There is no board support package in the provided folder. Please generate the hardware project first!")
ELSE()
    MESSAGE(STATUS "Found board support package in folder: ${XIL_BSP_DIR}")
ENDIF()

################################################################################
# Set variables
SET(ARCH_EXE_SUFFIX ".elf")
SET(ARCH_EXECUTABLE_NAME ${PROJECT_NAME}${ARCH_EXE_SUFFIX})

#################################################################################
# Update generated PS initialization files
CONFIGURE_FILE(${XIL_HW_PLATFORM_DIR}/ps7_init.c ${XIL_ZYNQ_FSBL_DIR}/src/ps7_init.c)
CONFIGURE_FILE(${XIL_HW_PLATFORM_DIR}/ps7_init.h ${XIL_ZYNQ_FSBL_DIR}/src/ps7_init.h)

################################################################################
# Set target specific include files
INCLUDE_DIRECTORIES(${XIL_BSP_DIR}/${EXECUTABLE_CPU_NAME}/include
                   )

################################################################################
# Set path to bsp liraries
LINK_DIRECTORIES(${XIL_BSP_DIR}/${EXECUTABLE_CPU_NAME}/lib)

SET(ARCH_LIBS "-lrsa -Wl,--start-group,-lxil,-lgcc,-lc,--end-group")

################################################################################
# Set target specific compile flags
SET(ARCH_CFLAGS "${XIL_HOST_CFLAGS} -Wl, -fmessage-length=0 -ffunction-sections -fdata-sections")
SET(ARCH_ASMFLAGS "${XIL_HOST_PLAT_ENDIAN}")

SET(ARCH_LINKERFLAGS "${XIL_HOST_PLAT_ENDIAN} -Wl,-T -Wl,${XIL_LSCRIPT} -Wl,-Map,${PROJECT_NAME}.map")

########################################################################
# Eclipse project files
SET(CFG_CPU_NAME ${EXECUTABLE_CPU_NAME})

GEN_ECLIPSE_FILE_LIST("${FSBL_C_SRCS}" "" PART_ECLIPSE_FILE_LIST )
SET(ECLIPSE_FILE_LIST "${ECLIPSE_FILE_LIST} ${PART_ECLIPSE_FILE_LIST}")

GEN_ECLIPSE_FILE_LIST("${FSBL_ASM_SRCS}" "" PART_ECLIPSE_FILE_LIST)
SET(ECLIPSE_FILE_LIST "${ECLIPSE_FILE_LIST} ${PART_ECLIPSE_FILE_LIST}")

GET_PROPERTY(DEMO_INCLUDES DIRECTORY ${PROJECT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
GEN_ECLIPSE_INCLUDE_LIST("${DEMO_INCLUDES}" ECLIPSE_INCLUDE_LIST)

CONFIGURE_FILE(${XIL_TOOLS_DIR}/eclipse/appproject.in ${PROJECT_BINARY_DIR}/.project @ONLY)
CONFIGURE_FILE(${XIL_TOOLS_DIR}/eclipse/appcproject.in ${PROJECT_BINARY_DIR}/.cproject @ONLY)
