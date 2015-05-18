################################################################################
#
# CMake macro for installing the bsp for Altera Designs
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

MACRO(INSTALL_BSP BSP_SOURCE_DIR BSP_TARGET_DIR BSP_CPU_NAME)
    GET_FILENAME_COMPONENT(BSP_TARGET_NAME ${BSP_SOURCE_DIR} NAME)
    SET(BSP_CPU_NAME ${BSP_CPU_NAME})
    SET(CFG_CPU_NAME ${BSP_CPU_NAME})


INSTALL(PROGRAMS ${BSP_SOURCE_DIR}/lib${BSP_DEMO_NAME_${PROC_INST_NAME}}-${HAL_TARGET}.a DESTINATION ${BSP_TARGET_DIR}/bsp${BSP_CPU_NAME}/${BSP_CPU_NAME} RENAME lib${HAL_TARGET}.a)
INSTALL(DIRECTORY ${BSP_HAL_INC_DIR} DESTINATION ${BSP_TARGET_DIR}/bsp${BSP_CPU_NAME}/${BSP_CPU_NAME}
        FILES_MATCHING PATTERN "*.h")

#set_target_properties(${LINKER_SCRIPT} PROPERTIES OUTPUT_NAME linker.ld)
INSTALL(FILES ${LINKER_SCRIPT} DESTINATION ${BSP_TARGET_DIR}/linker RENAME linker.ld)

IF(NOT TARGET_INIT_SCRIPT STREQUAL None)
INSTALL(FILES ${TARGET_INIT_SCRIPT} DESTINATION ${BSP_TARGET_DIR}/init RENAME init.ds)
ENDIF()

IF(DEFINED CFG_${PROC_INST_NAME}_BOOTLOADER_ENABLE AND CFG_${PROC_INST_NAME}_BOOTLOADER_ENABLE)
INSTALL(FILES ${SPL_PATH}/uboot.ds DESTINATION ${BSP_TARGET_DIR}/spl_bsp)
INSTALL(FILES ${SPL_PATH}/preloader-mkpimage.bin DESTINATION ${BSP_TARGET_DIR}/spl_bsp)
INSTALL(FILES ${SPL_PATH}/preloader.ds DESTINATION ${BSP_TARGET_DIR}/spl_bsp)
#INSTALL(DIRECTORY ${SPL_PATH}/uboot-socfpga DESTINATION ${BSP_TARGET_DIR}/spl_bsp)
INSTALL(FILES ${SPL_PATH}/uboot-socfpga/spl/u-boot-spl DESTINATION ${BSP_TARGET_DIR}/spl_bsp/uboot-socfpga/spl/)
ENDIF()

SET(ADD_CLEAN_FILES ${ADD_CLEAN_FILES} ${BSP_TARGET_DIR})
ENDMACRO()
