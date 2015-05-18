################################################################################
#
# CMake macro for generating the bitstream for Altera Designs
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

MACRO(GENERATE_BITS EXAMPLE_NAME HW_DEMO_DIR SKIP_BITSTREAM)

    SET(BSP_QUARTUS_DIR ${HW_DEMO_DIR}/quartus)
    SET(BITSTREAM_TARGET sof)

    FILE(GLOB QSYS_FILE_LIST RELATIVE "${BSP_QUARTUS_DIR}/" "${BSP_QUARTUS_DIR}/*.qsys")
    FOREACH (QSYS_FILE IN ITEMS ${QSYS_FILE_LIST})
        # should be one qsys file
        GET_FILENAME_COMPONENT(TOP_SYSTEM_NAME ${QSYS_FILE} NAME_WE)
    ENDFOREACH()

    IF (NOT SKIP_BITSTREAM)
        ADD_CUSTOM_COMMAND(
            OUTPUT ${BSP_QUARTUS_DIR}/${TOP_SYSTEM_NAME}.sopcinfo
            COMMAND make
            WORKING_DIRECTORY ${BSP_QUARTUS_DIR}
        )
    ENDIF()

ENDMACRO()
