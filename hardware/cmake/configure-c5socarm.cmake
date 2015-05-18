################################################################################
#
# CMake boards configuration file for Altera Cyclone V ARM
#
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

################################################################################
# Handle includes

MESSAGE(STATUS "INFO: Altera Cyclone V SoC platform selected")
SET(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/altera" ${CMAKE_MODULE_PATH})
SET(CMAKE_MODULE_PATH "${OPLK_BASE_DIR}/cmake" ${CMAKE_MODULE_PATH})

INCLUDE(setalteraarmboardconfig)

################################################################################
# U S E R    O P T I O N S

# Assemble path to all boards with Xilinx demos
SET(BOARD_DIRS ${PROJECT_SOURCE_DIR}/boards/altera-c5soc)

# Skip bitstream generation
OPTION(SKIP_BITSTREAM "Skip bitstream generation to save time." ON)

################################################################################
# Find the Altera ARM toolchain
SET (SOC_EDS_ROOT_PATH $ENV{SOCEDS_DEST_ROOT})
IF("${SOC_EDS_ROOT_PATH}" STREQUAL "")
    MESSAGE(FATAL_ERROR "Run this program from the soc embedded shell!")
ENDIF()

UNSET(ALT_LIBGEN CACHE)
SET (ALT_LIBGEN bsp-editor)

################################################################################
# Set path to system folders
SET(ARCH_IPCORE_REPO None)
SET(ARCH_TOOLS_DIR ${OPLK_BASE_DIR}/tools/altera-arm)
SET(ARCH_HWLIB_PATH ${SOC_EDS_ROOT_PATH}/ip/altera/hps/altera_hps/hwlib)
SET(ARCH_SOC_TOOLS_PATH ${SOC_EDS_ROOT_PATH}/host_tools/altera/preloadergen)
MESSAGE("The Hardware library Path is set to ${ARCH_HWLIB_PATH}")