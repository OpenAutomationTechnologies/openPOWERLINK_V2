################################################################################
#
# CMake boards configuration file for Microblaze platform
#
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
# Handle includes
SET(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/xilinx" ${CMAKE_MODULE_PATH})
SET(CMAKE_MODULE_PATH "${OPLK_BASE_DIR}/cmake" ${CMAKE_MODULE_PATH})

INCLUDE(geneclipsefilelist)
INCLUDE(geneclipseincludelist)
INCLUDE(setmicroblazeboardconfig)

################################################################################
# U S E R    O P T I O N S

# Assemble path to all boards with Xilinx demos
SET(BOARD_DIRS ${PROJECT_SOURCE_DIR}/boards/xilinx-z702)

# Skip bitstream generation
OPTION(SKIP_BITSTREAM "Skip bitstream generation to save time." OFF)
MARK_AS_ADVANCED(SKIP_BITSTREAM)

################################################################################
# Find the Xilinx toolchain
# Find the Xilinx toolchain
UNSET(XIL_SDK CACHE)
FIND_PROGRAM(XIL_SDK NAMES xsct
    PATHS
    ${XILINX_VIVADO}SDK/2016.2/bin
    DOC "Xilinx board support package generation tool"
)

UNSET(XIL_VIVADO CACHE)
FIND_PROGRAM(XIL_VIVADO NAMES vivado
    PATHS
    ${XILINX_VIVADO}Vivado/2016.2/bin
    DOC "Vivado TCL command"
)

################################################################################
# Set path to system folders
SET(ARCH_IPCORE_REPO ${PROJECT_SOURCE_DIR}/ipcore/xilinx)
SET(ARCH_TOOLS_DIR ${OPLK_BASE_DIR}/tools/xilinx-microblaze)
