################################################################################
#
# CMake macro for generating the board support package for Altera designs
#
# Copyright (c) 2014, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
# Copyright (c) 2015, Kalycito Infotech Private Limited
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


MACRO(GENERATE_BSP EXAMPLE_NAME ALT_DEMO_DIR ALT_BSP_TARGET_DIR PROCESSOR_NAME TCIMEM_NAME ALT_PLK_IPCORE_REPO)

    PROJECT(bsp-${EXAMPLE_NAME} C)

    # Set hardware directory internal paths
    SET(BSP_QUARTUS_DIR ${ALT_DEMO_DIR}/quartus)
    SET(BSP_SDK_DIR ${BSP_QUARTUS_DIR}/sdk)
    SET(BSP_SOF_DIR ${BSP_QUARTUS_DIR}/output_files)
    SET(BSP_HAL_DIR ${ALT_BSP_TARGET_DIR}/${CFG_${PROC_INST_NAME}_NAME})
    SET(BSP_HAL_INC_DIR ${BSP_HAL_DIR}/include)
    SET(${PROC_INST_NAME}_HWLIB_PATH ${ARCH_HWLIB_PATH})
    SET(SPL_PATH ${ALT_BSP_TARGET_DIR}/../spl-${CFG_${PROC_INST_NAME}_NAME})

    FILE(GLOB QSYS_FILE_LIST RELATIVE "${BSP_QUARTUS_DIR}/" "${BSP_QUARTUS_DIR}/*.qsys")
    FOREACH (QSYS_FILE IN ITEMS ${QSYS_FILE_LIST})
        # should be one qsys file
        GET_FILENAME_COMPONENT(TOP_SYSTEM_NAME ${QSYS_FILE} NAME_WE)
    ENDFOREACH()

    SET(SPL_SRC_DIR ${BSP_QUARTUS_DIR}/${CFG_HOST_PROC_IP_NAME}_isw_handoff/${TOP_SYSTEM_NAME}_${CFG_${PROC_INST_NAME}_NAME})

    SET(BSP_SYSTEM_NAME system)
    SET(UBOOT_TARGET u-boot-spl)
    SET(HAL_TARGET hal)

    ADD_CUSTOM_TARGET(
        bsp-${EXAMPLE_NAME} ALL
        DEPENDS ${EXAMPLE_NAME}-${HAL_TARGET}
    )

    IF(DEFINED CFG_${PROC_INST_NAME}_BOOTLOADER_ENABLE AND CFG_${PROC_INST_NAME}_BOOTLOADER_ENABLE)

    ADD_DEPENDENCIES(bsp-${EXAMPLE_NAME} ${EXAMPLE_NAME}-${UBOOT_TARGET})
    FILE(MAKE_DIRECTORY ${SPL_PATH})
    FILE(RELATIVE_PATH REL_BSP_SPL_SRC_DIR ${CMAKE_CURRENT_BINARY_DIR} ${SPL_SRC_DIR})
    FILE(RELATIVE_PATH REL_SPL_PATH ${CMAKE_CURRENT_BINARY_DIR} ${SPL_PATH})
    SET(SPL_GEN_ARGS --preloader-settings-dir ${SPL_SRC_DIR} --settings ${SPL_PATH}/settings.bsp --type spl )
    SET(SPL_GEN_ARGS ${SPL_GEN_ARGS} --set spl.boot.WATCHDOG_ENABLE false --set spl.boot.SDRAM_SCRUBBING true )
    SET(SPL_GEN_ARGS ${SPL_GEN_ARGS} --set spl.debug.SKIP_SDRAM false --set spl.boot.SDRAM_SCRUB_REMAIN_REGION true )
    IF(DEFINED CFG_${PROC_INST_NAME}_SEMIHOSTING_ENABLE AND CFG_${PROC_INST_NAME}_SEMIHOSTING_ENABLE)
        SET(SPL_GEN_ARGS ${SPL_GEN_ARGS} --set spl.debug.SEMIHOSTING true --set spl.performance.SERIAL_SUPPORT false )
        SET(LINKER_SCRIPT ${BSP_SDK_DIR}/${CFG_${PROC_INST_NAME}_SEMIHOSTED_LINKER})
    ELSE()
        SET(SPL_GEN_ARGS ${SPL_GEN_ARGS} --set spl.debug.SEMIHOSTING false --set spl.performance.SERIAL_SUPPORT true )
        SET(LINKER_SCRIPT ${BSP_SDK_DIR}/${CFG_${PROC_INST_NAME}_UNHOSTED_LINKER})
    ENDIF()

    IF(DEFINED CFG_${PROC_INST_NAME}_BOOT_FROM_SDCARD AND CFG_${PROC_INST_NAME}_BOOT_FROM_SDCARD)
        SET (SPL_GEN_ARGS ${SPL_GEN_ARGS} --set spl.boot.BOOT_FROM_SDMMC true )
        SET (SPL_GEN_ARGS ${SPL_GEN_ARGS} --set spl.boot.FAT_SUPPORT true )
        SET (SPL_GEN_ARGS ${SPL_GEN_ARGS} --set spl.boot.FAT_LOAD_PAYLOAD_NAME BOOT.bin )
        UNSET(TARGET_INIT_SCRIPT)
        SET(TARGET_INIT_SCRIPT None)
    ELSE()
        SET (SPL_GEN_ARGS ${SPL_GEN_ARGS} --set spl.boot.BOOT_FROM_SDMMC false )
        SET (SPL_GEN_ARGS ${SPL_GEN_ARGS} --set spl.boot.FAT_SUPPORT false )
    ENDIF()

    ADD_CUSTOM_TARGET(
        ${EXAMPLE_NAME}-${UBOOT_TARGET}
        DEPENDS ${SPL_PATH}/settings.bsp
    )

    ADD_CUSTOM_COMMAND(
        DEPENDS ${SPL_SRC_DIR}/emif.xml
        DEPENDS ${SPL_SRC_DIR}/${CFG_HOST_PROC_IP_NAME}.xml
        OUTPUT ${SPL_PATH}/settings.bsp
        COMMAND chmod -R +rwx ${SPL_PATH}
        COMMAND bsp-create-settings.exe  ${SPL_GEN_ARGS}
        COMMAND bsp-generate-files.exe --settings ${SPL_PATH}/settings.bsp --bsp-dir ${SPL_PATH}
        WORKING_DIRECTORY ${SPL_PATH}
    )

    ADD_CUSTOM_COMMAND(
        TARGET ${EXAMPLE_NAME}-${UBOOT_TARGET}
        POST_BUILD
        COMMAND make
        WORKING_DIRECTORY ${SPL_PATH}
    )

    ENDIF()

    # A hardware library source has to be used. Address macros alone has to be generated from .sopc file

    FILE(MAKE_DIRECTORY ${BSP_HAL_DIR})
    FILE(MAKE_DIRECTORY ${BSP_HAL_INC_DIR})

    IF (DEFINED CFG_${PROC_INST_NAME}_HAL_TYPE AND (CFG_${PROC_INST_NAME}_HAL_TYPE STREQUAL "hwlib"))

        INCLUDE(${ALT_DEMO_DIR}/cmake/configure-hal.cmake)

        UNSET(LIB_ARCH_SRC_LIST)
        FOREACH(SRC IN ITEMS ${LIB_${PROC_INST_NAME}_ARCH_HAL_SRCS})
            SET_SOURCE_FILES_PROPERTIES(${BSP_HAL_DIR}/${SRC} PROPERTIES GENERATED 1)
            SET (LIB_ARCH_SRC_LIST ${LIB_ARCH_SRC_LIST} ${BSP_HAL_DIR}/${SRC})
        ENDFOREACH()

        UNSET(LIB_ARCH_HAL_INC_LIST)
        FOREACH(INC IN ITEMS ${LIB_${PROC_INST_NAME}_ARCH_HAL_INCS})
            FILE(GLOB INC_LIST "${INC}/*.h")
            SET(LIB_ARCH_HAL_INC_LIST ${LIB_ARCH_HAL_INC_LIST} ${INC_LIST})
        ENDFOREACH()

        SET(${PROC_INST_NAME}_HAL_SRCS ${LIB_ARCH_SRC_LIST})

        ADD_DEFINITIONS(${ALT_${PROC_INST_NAME}_CFLAGS} " " ${LIB_${PROC_INST_NAME}_ARCH_HAL_C_FLAGS})
        INCLUDE_DIRECTORIES(${LIB_${PROC_INST_NAME}_ARCH_HAL_INCS})
        ADD_LIBRARY(${EXAMPLE_NAME}-${HAL_TARGET} ${${PROC_INST_NAME}_HAL_SRCS})
        SET_TARGET_PROPERTIES(${EXAMPLE_NAME}-${HAL_TARGET} PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${ALT_BSP_TARGET_DIR})
        ADD_DEPENDENCIES(${EXAMPLE_NAME}-${HAL_TARGET} ${EXAMPLE_NAME}-HAL_SRC)

        ADD_CUSTOM_TARGET(
            ${EXAMPLE_NAME}-HAL_SRC
            DEPENDS ${BSP_HAL_INC_DIR}/system.h
            COMMAND tar -C ${${PROC_INST_NAME}_HWLIB_PATH} -cjf temp.tar ${LIB_${PROC_INST_NAME}_ARCH_HAL_SRCS}
            COMMAND tar xfjp temp.tar
            COMMAND ${CMAKE_COMMAND} -E remove temp.tar
            COMMAND chmod -R +rwx *
            COMMAND  ${CMAKE_COMMAND} -E copy_directory ${LIB_${PROC_INST_NAME}_ARCH_HAL_INCS} ${BSP_HAL_INC_DIR}
            WORKING_DIRECTORY ${BSP_HAL_DIR}
        )



        ADD_CUSTOM_COMMAND(
            DEPENDS ${BSP_QUARTUS_DIR}/${TOP_SYSTEM_NAME}.sopcinfo
            OUTPUT ${BSP_HAL_INC_DIR}/system.h
            COMMAND sopc-create-header-files ${BSP_QUARTUS_DIR}/${TOP_SYSTEM_NAME}.sopcinfo --module ${ARCH_${PROC_INST_NAME}_MODULE_NAME} --single system.h
            WORKING_DIRECTORY ${BSP_HAL_INC_DIR}
        )

    # BSP has to be generated from .sopc file
    ELSEIF (DEFINED CFG_${PROC_INST_NAME}_BSP_GEN AND (CFG_${PROC_INST_NAME}_HAL_TYPE STREQUAL "BSP"))
        #TODO For NIOS/ in-built bsp with Altera
        MESSAGE(FATAL_ERROR "Current design only supports a HAL from the hardware library!!")
    ENDIF()


SET_DIRECTORY_PROPERTIES(PROPERTIES
                         ADDITIONAL_MAKE_CLEAN_FILES "${PROJECT_BINARY_DIR}/lib${EXAMPLE_NAME}-${HAL_TARGET}.a"
                        )
ENDMACRO()
