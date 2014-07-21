################################################################################
#
# CMake macro for installing the bitstream for Microblaze
#
# Copyright (c) 2014, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
# Copyright (c) 2014, Kalycito Infotech Private Limited
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

MACRO(INSTALL_BITSTREAM EXAMPLE_ROOT BITS_DESTINATION)
    SET(SDK_EXPORT ${EXAMPLE_ROOT}/xps/SDK/SDK_Export/hw)
    SET(DLCMD_SOURCE_DIR ${EXAMPLE_ROOT}/xps/etc)

##########################################################
# Additionally include the default XML path to avoid tool
# related issues
    IF(${CFG_DEMO_BOARD_NAME} STREQUAL "xilinx-z702")
    SET(ZYNQ_XML ${EXAMPLE_ROOT}/sdk)
    ENDIF(${CFG_DEMO_BOARD_NAME} STREQUAL "xilinx-z702")

    # Remove folder prefix from download.cmd script
    FILE(READ ${DLCMD_SOURCE_DIR}/download.cmd DLCMD_CONTENT)
    STRING(REGEX REPLACE "implementation/" ""
        MODIFIED_DLCMD_CONTENT "${DLCMD_CONTENT}")
    FILE(WRITE ${PROJECT_BINARY_DIR}/download.cmd "${MODIFIED_DLCMD_CONTENT}")

    # Copy hardware platform eclipse project file
    CONFIGURE_FILE(${ARCH_TOOLS_DIR}/eclipse/hwplatformproject.in ${PROJECT_BINARY_DIR} @ONLY)

    INSTALL(FILES ${SDK_EXPORT}/system.bit ${SDK_EXPORT}/download.bit ${SDK_EXPORT}/system.xml ${SDK_EXPORT}/system_bd.bmm
            DESTINATION ${BITS_DESTINATION}
           )
    # Additional initialization modules generated for ARM on Zynq
    IF(${CFG_DEMO_BOARD_NAME} STREQUAL "xilinx-z702")
        INSTALL(FILES ${SDK_EXPORT}/ps7_init.tcl ${SDK_EXPORT}/ps7_init.c ${SDK_EXPORT}/ps7_init.h ${ZYNQ_XML}/system.xml
            DESTINATION ${BITS_DESTINATION}
           )
    ENDIF(${CFG_DEMO_BOARD_NAME} STREQUAL "xilinx-z702")

    INSTALL(FILES ${PROJECT_BINARY_DIR}/hwplatformproject.in
            DESTINATION ${BITS_DESTINATION} RENAME .project
           )
    INSTALL(FILES ${PROJECT_BINARY_DIR}/download.cmd
            DESTINATION ${BITS_DESTINATION}
           )
ENDMACRO()
