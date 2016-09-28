################################################################################
#
# CMake file of the bootloader simpleboot on target microblaze
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

SET(CMAKE_MODULE_PATH "${OPLK_BASE_DIR}/cmake" ${CMAKE_MODULE_PATH})

################################################################################
# Set includes
INCLUDE(geneclipsefilelist)
INCLUDE(geneclipseincludelist)
INCLUDE(setmicroblazeboardconfig)

################################################################################
# Set paths
SET(XIL_HW_LIB_DIR ${OPLK_BASE_DIR}/hardware/lib/${SYSTEM_NAME_DIR}/${SYSTEM_PROCESSOR_DIR})
SET(XIL_BIN_DIR ${OPLK_BASE_DIR}/bin/${SYSTEM_NAME_DIR}/${SYSTEM_PROCESSOR_DIR})

################################################################################
# Set install prefix to bin export folder
SET(CMAKE_INSTALL_PREFIX
    ${CFG_BIN_DIR} CACHE PATH "bootloader install prefix" FORCE
   )

SET(ARCH_INSTALL_POSTFIX ${PROJECT_NAME})

################################################################################
# U S E R   O P T I O N S

# Path to the hardware library folder of your board example
SET(CFG_HW_LIB_DIR ${XIL_HW_LIB_DIR}/avnet-s6plkeb/cn-single-gpio
    CACHE PATH "Path to the hardware library folder for the board example you want to use")

SET(CFG_BIN_DIR ${XIL_BIN_DIR}/avnet-s6plkeb/cn-single-gpio
    CACHE PATH "Path to the target binary folder of your example design")

# Include board specific settings file
SET_BOARD_CONFIGURATION(${CFG_HW_LIB_DIR})

UNSET(TARGET_CPU)

IF(DEFINED CFG_HOST_NAME AND DEFINED CFG_PCP_NAME)
    # There are both CPUs in one bitstream provide user option
    IF(NOT TARGET_CPU)
        SET(TARGET_CPU "pcp" CACHE STRING
            "Choose the target CPU you want to build the bootloader for"
            FORCE)
        SET_PROPERTY(CACHE TARGET_CPU PROPERTY STRINGS "pcp;app")
    ENDIF()
ELSEIF(DEFINED CFG_HOST_NAME)
    # Only host CPU is in the bitstream
    SET(TARGET_CPU "app")
ELSEIF(DEFINED CFG_PCP_NAME)
    # Only PCP CPU is in the bitstream
    SET(TARGET_CPU "pcp")
ENDIF()

IF(${TARGET_CPU} STREQUAL "app")
    SET(EXECUTABLE_CPU_NAME ${CFG_HOST_NAME})
    SET(CPU_PREFIX HOST)
ELSE()
    SET(EXECUTABLE_CPU_NAME ${CFG_PCP_NAME})
    SET(CPU_PREFIX PCP)
ENDIF()

################################################################################
# Find Xilinx tools
FIND_PROGRAM(XIL_PROMGEN NAMES promgen
             DOC "Xilinx Flash Image Generator"
            )

FIND_PROGRAM(XIL_PERL NAMES xilperl
             DOC "Xilinx Perl Distribution"
            )

FIND_PROGRAM(XIL_IMPACT NAMES impact
             DOC "Xilinx iMPACT"
            )


################################################################################
# Search for executable in bin folder
FILE(GLOB OPLK_ELF_FILE "${CFG_BIN_DIR}/*.mem")

LIST(LENGTH OPLK_ELF_FILE ENTRY_COUNT)

IF(${ENTRY_COUNT} EQUAL 0)
    MESSAGE(FATAL_ERROR "unexpected: Unable to find mem file of the POWERLINK example executeable!")
ELSEIF(${ENTRY_COUNT} GREATER 1)
    MESSAGE(FATAL_ERROR "unexpected: Found multiple mem files of the POWERLINK example executeable! Only one is allowed!")
ENDIF()

GET_FILENAME_COMPONENT(OPLK_ELF_FILE ${OPLK_ELF_FILE} REALPATH)

MESSAGE(STATUS "Found mem file of the POWERLINK example executeable: ${OPLK_ELF_FILE}")

################################################################################
# Set paths to configuration
SET(XIL_BSP_DIR ${CFG_HW_LIB_DIR}/bsp${EXECUTABLE_CPU_NAME})
SET(XIL_HW_SPEC ${CFG_HW_LIB_DIR}/hw_platform)

SET(XIL_LSCRIPT ${XIL_BSP_DIR}/lscript-bootloader.ld)

SET(XIL_TOOLS_DIR ${TOOLS_DIR}/xilinx-microblaze)

################################################################################
# Verify board support package
IF(NOT EXISTS "${XIL_BSP_DIR}" OR NOT IS_DIRECTORY "${XIL_BSP_DIR}")
    MESSAGE(FATAL_ERROR "unexpected: There is no board support package in the provided folder. Please generate the hardware project first!")
ELSE()
    MESSAGE(STATUS "Found board support package in folder: ${XIL_BSP_DIR}")
ENDIF()

################################################################################
# Set variables
SET(XIL_SCRIPT_DIR ${PROJECT_SOURCE_DIR}/scripts)
SET(ARCH_EXE_SUFFIX ".elf")
SET(ARCH_EXECUTABLE_NAME ${PROJECT_NAME}${ARCH_EXE_SUFFIX})

SET(FLASH_IMAGE_NAME "flash_image")

################################################################################
# Set target specific include files
INCLUDE_DIRECTORIES(${XIL_BSP_DIR}/${EXECUTABLE_CPU_NAME}/include
                   )

################################################################################
# Set path to bsp liraries
LINK_DIRECTORIES(${XIL_BSP_DIR}/${EXECUTABLE_CPU_NAME}/lib)

SET(ARCH_LIBS xil)

################################################################################
# Set target specific compile flags
SET(ARCH_CFLAGS "${XIL_${CPU_PREFIX}_CFLAGS} -Wl,--no-relax -ffunction-sections -fdata-sections")
SET(ARCH_ASMFLAGS "${XIL_${CPU_PREFIX}_PLAT_ENDIAN}")

SET(ARCH_LINKERFLAGS "${XIL_${CPU_PREFIX}_PLAT_ENDIAN} -Wl,-T -Wl,${XIL_LSCRIPT} -nostartfiles -Wl,-Map,${PROJECT_NAME}.map -Wl,--no-relax -Wl,--gc-sections")

##################################################################################
# Add custom target for flash image generation
ADD_CUSTOM_TARGET(
    gen-flash ALL
    DEPENDS ${FLASH_IMAGE_NAME}.mcs
)

ADD_CUSTOM_COMMAND(
    DEPENDS ${ARCH_EXECUTABLE_NAME}
    DEPENDS ${XIL_HW_SPEC}/system.bit
    OUTPUT  ${PROJECT_BINARY_DIR}/download.bit
    COMMAND elfcheck -hw ${XIL_HW_SPEC}/system.xml -mode bootload -mem BRAM -pe ${EXECUTABLE_CPU_NAME} ${PROJECT_BINARY_DIR}/${ARCH_EXECUTABLE_NAME}
    COMMAND data2mem -bm ${XIL_HW_SPEC}/system_bd.bmm -bt ${XIL_HW_SPEC}/system.bit -bd ${PROJECT_BINARY_DIR}/${ARCH_EXECUTABLE_NAME} tag ${EXECUTABLE_CPU_NAME} -o b ${PROJECT_BINARY_DIR}/download.bit
)

ADD_CUSTOM_COMMAND(
    DEPENDS ${PROJECT_BINARY_DIR}/download.bit
    OUTPUT  ${FLASH_IMAGE_NAME}.mcs
    COMMAND ${XIL_PROMGEN} ${CFG_PROMGEN_FLAGS} -o ${FLASH_IMAGE_NAME}.mcs ${CFG_PROMGEN_TYPE} ${PROJECT_BINARY_DIR}/download.bit ${CFG_PROMGEN_PREFIX}
    COMMAND ${XIL_PERL} ${XIL_SCRIPT_DIR}/pcublaze.pl ${CFG_PCUBLAZE_PARAMS} --memfile ${OPLK_ELF_FILE} --promfile ${FLASH_IMAGE_NAME}.mcs --outfile ${FLASH_IMAGE_NAME}.mcs
)

SET(ADD_CLEAN_FILES ${ADD_CLEAN_FILES}
                    ${FLASH_IMAGE_NAME}.cfi
                    ${FLASH_IMAGE_NAME}.prm
   )

##################################################################################
# Add custom target for flash programming

ADD_CUSTOM_TARGET (
    prog-flash
    DEPENDS gen-flash
    COMMAND ${XIL_IMPACT} -batch ${XIL_SCRIPT_DIR}/program-prom-${CFG_DEMO_BOARD_NAME}.cmd
)

########################################################################
# Eclipse project files
SET(CFG_CPU_NAME ${EXECUTABLE_CPU_NAME})

GEN_ECLIPSE_FILE_LIST("${BOOT_C_SRCS}" "" PART_ECLIPSE_FILE_LIST )
SET(ECLIPSE_FILE_LIST "${ECLIPSE_FILE_LIST} ${PART_ECLIPSE_FILE_LIST}")

GEN_ECLIPSE_FILE_LIST("${BOOT_ASM_SRCS}" "" PART_ECLIPSE_FILE_LIST)
SET(ECLIPSE_FILE_LIST "${ECLIPSE_FILE_LIST} ${PART_ECLIPSE_FILE_LIST}")

GET_PROPERTY(DEMO_INCLUDES DIRECTORY ${PROJECT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
GEN_ECLIPSE_INCLUDE_LIST("${DEMO_INCLUDES}" ECLIPSE_INCLUDE_LIST)

CONFIGURE_FILE(${XIL_TOOLS_DIR}/eclipse/appproject.in ${PROJECT_BINARY_DIR}/.project @ONLY)
CONFIGURE_FILE(${XIL_TOOLS_DIR}/eclipse/appcproject.in ${PROJECT_BINARY_DIR}/.cproject @ONLY)

################################################################################
# Set architecture specific installation files
INSTALL(FILES ${PROJECT_BINARY_DIR}/${FLASH_IMAGE_NAME}.mcs
        DESTINATION ${ARCH_INSTALL_POSTFIX}
       )
INSTALL(PROGRAMS ${XIL_SCRIPT_DIR}/program-prom-${CFG_DEMO_BOARD_NAME}.cmd
        DESTINATION ${ARCH_INSTALL_POSTFIX} RENAME program-prom.cmd
       )
