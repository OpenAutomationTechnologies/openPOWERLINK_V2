################################################################################
#
# CMake macro for generating the board support package for Microblaze
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


MACRO(GENERATE_BSP EXAMPLE_NAME XIL_DEMO_DIR XIL_BSP_TARGET_DIR PROCESSOR_NAME TCIMEM_NAME XIL_PLK_IPCORE_REPO)

    # Set hardware directory internal paths
    SET(BSP_SDK_DIR ${XIL_DEMO_DIR}/sdk)
    SET(BSP_XPS_DIR ${XIL_DEMO_DIR}/xps)
    SET(BSP_SDKEXPORT_DIR ${BSP_XPS_DIR}/SDK/SDK_Export/hw)
    SET(BSP_SYSTEM_NAME system)

    FILE(MAKE_DIRECTORY ${XIL_BSP_TARGET_DIR})
    CONFIGURE_FILE(${BSP_SDK_DIR}/${PROCESSOR_NAME}${BSP_SYSTEM_NAME}.mss ${XIL_BSP_TARGET_DIR}/${BSP_SYSTEM_NAME}.mss COPY_ONLY)
    CONFIGURE_FILE(${BSP_SDK_DIR}/${PROCESSOR_NAME}lscript.ld ${XIL_BSP_TARGET_DIR}/lscript.ld COPY_ONLY)
    IF(DEFINED CFG_MB_BOOTLOADER_ENABLE AND CFG_MB_BOOTLOADER_ENABLE)
    CONFIGURE_FILE(${BSP_SDK_DIR}/${PROCESSOR_NAME}lscript-bootloader.ld ${XIL_BSP_TARGET_DIR}/lscript-bootloader.ld COPY_ONLY)
    ENDIF(DEFINED CFG_MB_BOOTLOADER_ENABLE AND CFG_MB_BOOTLOADER_ENABLE)

    IF(NOT XIL_LIBGEN STREQUAL "XIL_LIBGEN-NOTFOUND")
        ADD_CUSTOM_TARGET(
            bsp-${EXAMPLE_NAME} ALL
            DEPENDS ${XIL_BSP_TARGET_DIR}/${PROCESSOR_NAME}/lib/libxil.a
        )

        ADD_CUSTOM_COMMAND(
            DEPENDS ${BSP_SDKEXPORT_DIR}/${BSP_SYSTEM_NAME}.xml
            OUTPUT ${XIL_BSP_TARGET_DIR}/${PROCESSOR_NAME}/lib/libxil.a
            COMMAND ${XIL_LIBGEN} -hw ${BSP_SDKEXPORT_DIR}/${BSP_SYSTEM_NAME}.xml -lp ${XIL_PLK_IPCORE_REPO} -pe ${PROCESSOR_NAME} -od ${XIL_BSP_TARGET_DIR} -log libgen.log ${XIL_BSP_TARGET_DIR}/${BSP_SYSTEM_NAME}.mss
        )

        ADD_CUSTOM_TARGET(
            clean-bsp-${EXAMPLE_NAME}
            COMMAND ${CMAKE_COMMAND} -E remove_directory ${XIL_BSP_TARGET_DIR}
        )

        # Add all generated files to clean target
        SET(ADD_CLEAN_FILES ${ADD_CLEAN_FILES} ${XIL_BSP_TARGET_DIR})
    ELSE()
        MESSAGE(FATAL_ERROR "libgen was not found in system PATH or ISE installation")
    ENDIF()

ENDMACRO()
