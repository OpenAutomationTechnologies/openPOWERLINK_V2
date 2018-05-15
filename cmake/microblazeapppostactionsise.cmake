################################################################################
#
# CMake file for microblaze post build actions
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
    TARGET ${EXECUTABLE_NAME}
    POST_BUILD
    COMMAND mb-size ${EXECUTABLE_NAME} | tee "${PROJECT_NAME}.size"
    COMMAND elfcheck ${EXECUTABLE_NAME} -hw ${XIL_HW_SPEC}/system.xml -pe ${EXECUTABLE_CPU_NAME} | tee "${PROJECT_NAME}.elfcheck"
    COMMAND data2mem -bd ${EXECUTABLE_NAME} -d -o m ${PROJECT_NAME}.mem
)

SET_DIRECTORY_PROPERTIES(PROPERTIES
                         ADDITIONAL_MAKE_CLEAN_FILES "${EXECUTABLE_NAME};${PROJECT_NAME}.size;${PROJECT_NAME}.elfcheck;${PROJECT_NAME}.mem;${PROJECT_NAME}.map"
                        )
################################################################################
# Set list of additional make clean files
SET(ADD_CLEAN_FILES ${ADD_CLEAN_FILES}
                    ${EXECUTABLE_NAME}
                    ${PROJECT_NAME}.size
                    ${PROJECT_NAME}.elfcheck
                    ${PROJECT_NAME}.mem
                    ${PROJECT_NAME}.tmp
                    ${PROJECT_NAME}.map
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
            COMMAND ${XIL_MB_XMD} -hw ${XIL_HW_SPEC}/system.xml -tcl ${XIL_TOOLS_DIR}/xmd-downloadelf.tcl ${EXECUTABLE_NAME} ${XIL_VERIFY_ELF}
    )
ENDIF()

################################################################################
# Set architecture specific installation files
INSTALL(FILES ${XIL_TOOLS_DIR}/xmd-downloadelf.tcl
              ${XIL_HW_SPEC}/download.bit
              ${XIL_HW_SPEC}/download.cmd
              ${XIL_HW_SPEC}/system.xml
              ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.mem
        DESTINATION ${ARCH_INSTALL_POSTFIX}
       )
INSTALL(PROGRAMS ${XIL_TOOLS_DIR}/elfdownload.make
        DESTINATION ${ARCH_INSTALL_POSTFIX} RENAME Makefile
       )
