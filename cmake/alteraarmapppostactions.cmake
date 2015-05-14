################################################################################
#
# CMake file for Altera Cyclone V SoC ARM post build actions
#
# Copyright (c) 2015, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
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

################################################################################
# U S E R   O P T I O N S


##############################################################################
# Set paths

##############################################################################
# Demo pre build action

ADD_DEPENDENCIES(${EXECUTABLE_NAME} ${EXECUTABLE_NAME}-ARCH_PRE_BUILD_DEPS)
ADD_CUSTOM_TARGET(${EXECUTABLE_NAME}-ARCH_PRE_BUILD_DEPS
            COMMAND chmod +x ${ARCH_TOOLS_PATH}/fix-app-makefile.sh
            COMMAND ${ARCH_TOOLS_PATH}/fix-app-makefile.sh ${CMAKE_BINARY_DIR}/CMakeFiles/demo_mn_embedded.axf.dir/build.make
)

##############################################################################
# Demo post build action

IF(DEFINED CFG_${CPU_INST_NAME}_BOOT_FROM_SDCARD AND CFG_${CPU_INST_NAME}_BOOT_FROM_SDCARD)
    ADD_CUSTOM_COMMAND(
        TARGET ${EXECUTABLE_NAME}
        POST_BUILD
        COMMAND arm-altera-eabi-objcopy -O binary ${PROJECT_NAME}.axf ${PROJECT_NAME}.bin
        COMMAND mkimage -A arm -T standalone -C none -O u-boot -a 0x10000000 -e 0x10000040 -n "baremetal image" -d ${PROJECT_NAME}.bin ${PROJECT_NAME}-mkimage.bin
    )
ENDIF()

SET_DIRECTORY_PROPERTIES(PROPERTIES
                         ADDITIONAL_MAKE_CLEAN_FILES "${EXECUTABLE_NAME};${PROJECT_NAME}.bin;${PROJECT_NAME}-mkimage.bin"
                        )
################################################################################
# Set list of additional make clean files
SET(ADD_CLEAN_FILES ${ADD_CLEAN_FILES}
                    ${EXECUTABLE_NAME}
                    ${PROJECT_NAME}.bin
                    ${PROJECT_NAME}-mkimage.bin
   )

################################################################################
# Set architecture specific installation files

IF(DEFINED CFG_${CPU_INST_NAME}_SEMIHOSTING_ENABLE AND CFG_${CPU_INST_NAME}_SEMIHOSTING_ENABLE)
    CONFIGURE_FILE(${ALT_TOOLS_DIR}/debug-semihosted.ds.in ${ARCH_INSTALL_POSTFIX}/debug-semihosted.ds @ONLY)
ELSE()
    CONFIGURE_FILE(${ALT_TOOLS_DIR}/debug-unhosted.ds.in ${ARCH_INSTALL_POSTFIX}/debug-unhosted.ds @ONLY)
ENDIF()

IF(DEFINED CFG_${CPU_INST_NAME}_BOOT_FROM_SDCARD AND CFG_${CPU_INST_NAME}_BOOT_FROM_SDCARD)
    INSTALL(PROGRAMS ${CMAKE_BINARY_DIR}/${PROJECT_NAME}-mkimage.bin
        DESTINATION ${ARCH_INSTALL_POSTFIX}
        RENAME BOOT.bin
       )

    INSTALL(PROGRAMS ${CFG_HW_LIB_DIR}/spl_bsp/preloader-mkpimage.bin
            DESTINATION ${ARCH_INSTALL_POSTFIX}
           )

   INSTALL(PROGRAMS ${CFG_FPGA_RBF}
        DESTINATION ${ARCH_INSTALL_POSTFIX}
       )
ELSE()
    INSTALL(PROGRAMS ${CFG_HW_LIB_DIR}/spl_bsp/uboot-socfpga/spl/u-boot-spl
            DESTINATION ${ARCH_INSTALL_POSTFIX}
           )
ENDIF()


INSTALL(PROGRAMS ${ALT_TOOLS_DIR}/buildboot.make
        DESTINATION ${ARCH_INSTALL_POSTFIX} RENAME Makefile
       )
