################################################################################
#
# CMake macro for generating the bitstream for Microblaze
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

MACRO(GENERATE_BITS EXAMPLE_NAME HW_DEMO_DIR)

    SET(BITS_SYSTEM_NAME system)
    SET(XPS_DEMO_DIR ${HW_DEMO_DIR}/xps)
    SET(BITS_SDK_EXPORT ${XPS_DEMO_DIR}/SDK/SDK_Export/hw)

    IF(NOT ${XIL_XPS} STREQUAL "XIL_XPS-NOTFOUND")

        ADD_CUSTOM_COMMAND(
            OUTPUT ${XPS_DEMO_DIR}/${BITS_SYSTEM_NAME}.make
            COMMAND ${XIL_XPS} -nw -scr ${ARCH_TOOLS_DIR}/xps-genmakefile.tcl ${BITS_SYSTEM_NAME}.xmp
            WORKING_DIRECTORY ${XPS_DEMO_DIR}
        )

        ADD_CUSTOM_COMMAND(
            DEPENDS ${XPS_DEMO_DIR}/${BITS_SYSTEM_NAME}.make
            OUTPUT ${XPS_DEMO_DIR}/implementation/${BITS_SYSTEM_NAME}.bit
            COMMAND ${CMAKE_MAKE_PROGRAM} -C ${XPS_DEMO_DIR} -f ${BITS_SYSTEM_NAME}.make bits
        )

        ADD_CUSTOM_COMMAND(
            DEPENDS ${XPS_DEMO_DIR}/implementation/${BITS_SYSTEM_NAME}.bit
            OUTPUT ${BITS_SDK_EXPORT}/${BITS_SYSTEM_NAME}.xml
            COMMAND ${CMAKE_MAKE_PROGRAM} -C ${XPS_DEMO_DIR} -f ${BITS_SYSTEM_NAME}.make exporttosdk
            COMMAND ${CMAKE_MAKE_PROGRAM} -C ${XPS_DEMO_DIR} -f ${BITS_SYSTEM_NAME}.make init_bram
            COMMAND ${CMAKE_COMMAND} -E copy_if_different ${XPS_DEMO_DIR}/implementation/download.bit ${BITS_SDK_EXPORT}
        )

        ADD_CUSTOM_TARGET(
            bits-${EXAMPLE_NAME} ALL
            DEPENDS ${BITS_SDK_EXPORT}/${BITS_SYSTEM_NAME}.xml
        )

        ADD_CUSTOM_TARGET(
            clean-bits-${EXAMPLE_NAME}
            DEPENDS ${XPS_DEMO_DIR}/${BITS_SYSTEM_NAME}.make
            COMMAND ${CMAKE_MAKE_PROGRAM} -C ${XPS_DEMO_DIR} -f ${BITS_SYSTEM_NAME}.make clean
        )

        # Add all generated files to clean target
        SET(ADD_CLEAN_FILES ${ADD_CLEAN_FILES}
                            ${XPS_DEMO_DIR}/__xps
                            ${XPS_DEMO_DIR}/bootloops
                            ${XPS_DEMO_DIR}/hdl
                            ${XPS_DEMO_DIR}/implementation
                            ${XPS_DEMO_DIR}/SDK
                            ${XPS_DEMO_DIR}/synthesis
                            ${XPS_DEMO_DIR}/_impactbatch.log
                            ${XPS_DEMO_DIR}/bitinit.log
                            ${XPS_DEMO_DIR}/clock_generator_0.log
                            ${XPS_DEMO_DIR}/platgen.log
                            ${XPS_DEMO_DIR}/platgen.opt
                            ${XPS_DEMO_DIR}/psf2Edward.log
                            ${XPS_DEMO_DIR}/system.log
                            ${XPS_DEMO_DIR}/system.make
                            ${XPS_DEMO_DIR}/system_incl.make
                            ${XPS_DEMO_DIR}/xdsgen.log
           )
    ELSE()
        MESSAGE(FATAL_ERROR "Xilinx Platform Studio is not found by cmake!")
    ENDIF()
ENDMACRO()
