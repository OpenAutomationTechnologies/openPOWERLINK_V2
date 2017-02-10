################################################################################
#
# CMake macro for generating the bitstream for Microblaze Vivado
#
# Copyright (c) 2017, Kalycito Infotech Private Limited
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

MACRO(GENERATE_BITS EXAMPLE_NAME HW_DEMO_DIR SKIP_BITSTREAM)

    SET(BITS_SYSTEM_NAME system_wrapper)
    SET(VIVADO_DEMO_DIR ${HW_DEMO_DIR}/vivado)
    SET(BITS_SDK_EXPORT ${VIVADO_DEMO_DIR}/system/system.sdk)

    IF(NOT ${XIL_VIVADO} STREQUAL "XIL_VIVADO-NOTFOUND")

        IF(SKIP_BITSTREAM)
            ADD_CUSTOM_COMMAND(
                OUTPUT ${VIVADO_DEMO_DIR}/system/system.runs/impl_1/system_wrapper_timing_summary_routed.rpt
                COMMAND ${XIL_VIVADO} -mode batch -source ${ARCH_TOOLS_DIR}/gennobitvivado.tcl -tclargs ${VIVADO_DEMO_DIR}
                WORKING_DIRECTORY ${VIVADO_DEMO_DIR}
            )

        ELSE()
            ADD_CUSTOM_COMMAND(
                OUTPUT ${VIVADO_DEMO_DIR}/system/system.runs/impl_1/system_wrapper_timing_summary_routed.rpt
                DEPENDS ${VIVADO_DEMO_DIR}/system.xdc
                DEPENDS ${VIVADO_DEMO_DIR}/system_bd.tcl
                COMMAND ${XIL_VIVADO} -mode batch -source ${ARCH_TOOLS_DIR}/genbitvivado.tcl -tclargs ${VIVADO_DEMO_DIR}
                WORKING_DIRECTORY ${VIVADO_DEMO_DIR}
            )

        ENDIF()

        ADD_CUSTOM_TARGET(
            bits-${EXAMPLE_NAME} ALL
            DEPENDS ${VIVADO_DEMO_DIR}/system/system.runs/impl_1/system_wrapper_timing_summary_routed.rpt
        )

        ADD_CUSTOM_TARGET(
            clean-bits-${EXAMPLE_NAME}
            COMMAND ${XIL_VIVADO} -mode tcl
            COMMAND reset_project
            WORKING_DIRECTORY ${VIVADO_DEMO_DIR}/system
        )

        # Add all generated files to clean target
        SET(ADD_CLEAN_FILES ${ADD_CLEAN_FILES}
                            ${VIVADO_DEMO_DIR}/vivado.jou
                            ${VIVADO_DEMO_DIR}/vivado.log
                            ${VIVADO_DEMO_DIR}/system
           )
    ELSE()
        MESSAGE(FATAL_ERROR "Vivado is not found by cmake!")
    ENDIF()
ENDMACRO()
