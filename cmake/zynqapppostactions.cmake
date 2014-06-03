################################################################################
#
# CMake file for zynq SoC post build actions
#
# Copyright (c) 2014, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
# Copyright (c) 2014, Kalycito Infotech Pvt. Ltd.
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
SET(XIL_ZYNQ_BINARY_DIR ${CMAKE_INSTALL_PREFIX}/${CFG_DEMO_BOARD_NAME}/${CFG_DEMO_NAME})
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

ADD_CUSTOM_TARGET(
            build-sd
            COMMAND make all
            WORKING_DIRECTORY ${XIL_ZYNQ_BINARY_DIR}
    )

################################################################################
# Set architecture specific installation files
INSTALL(FILES ${XIL_TOOLS_DIR}/bootimage.bif
              ${XIL_HW_SPEC}/system.bit
              ${XIL_HW_SPEC}/system.xml
              ${XIL_HW_SPEC}/system_bd.bmm
        DESTINATION ${ARCH_INSTALL_POSTFIX}
       )
INSTALL(PROGRAMS ${XIL_TOOLS_DIR}/buildboot.make
        DESTINATION ${ARCH_INSTALL_POSTFIX} RENAME Makefile
       )
