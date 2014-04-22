################################################################################
#
# CMake macro for setting the board managing the configuration of the current
# selected board.
#
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

MACRO(SET_BOARD_CONFIGURATION PATH_TO_CONFIGURATION)

###############################################################################
# Unset all old configurations
UNSET(CFG_DEMO_NAME)
UNSET(CFG_DEMO_BOARD_NAME)
UNSET(CFG_DEMO_BUS_SYSTEM)

UNSET(CFG_PCP_TCIMEM_NAME)

UNSET(CFG_PCP_NAME)
UNSET(CFG_HOST_NAME)

UNSET(CFG_CPU_VERSION)

UNSET(CFG_MICROBLAZE_HW_MULT)
UNSET(CFG_MICROBLAZE_HW_DIV)
UNSET(CFG_MICROBLAZE_PAT_COMP)
UNSET(CFG_MICROBLAZE_BARREL_SHIFT)
UNSET(CFG_MICROBLAZE_REORDER)

UNSET(CFG_HOSTIF_ENABLE)

UNSET(CFG_PROMGEN_FLAGS)
UNSET(CFG_PROMGEN_TYPE)
UNSET(CFG_PROMGEN_PREFIX)

UNSET(CFG_PCUBLAZE_PARAMS)

###############################################################################
# Include new configuration file
IF(EXISTS "${PATH_TO_CONFIGURATION}/settings.cmake")
    INCLUDE(${PATH_TO_CONFIGURATION}/settings.cmake)
ELSE()
    MESSAGE(FATAL_ERROR "Settings file for demo ${PATH_TO_CONFIGURATION} does not exist!")
ENDIF()

###############################################################################
# Set CFLAGS variable depending on current board
IF(CFG_DEMO_BUS_SYSTEM MATCHES axi)
    SET(XIL_CFLAGS "${XIL_CFLAGS} -mlittle-endian")
    SET(XIL_PLAT_ENDIAN -mlittle-endian)
ELSE()

ENDIF()

################################################################################
# ARM version to use
SET(XIL_CFLAGS "${XIL_CFLAGS} -march=armv7-a")

IF(CFG_ARM_FRAME_AAPCS)
    SET(XIL_CFLAGS "${XIL_CFLAGS} -mapcs-frame")
ELSE()
    SET(XIL_CFLAGS "${XIL_CFLAGS} -mno-apcs-frame")
ENDIF()

if(CFG_ARM_SHORT_ENUMS)
    SET(XIL_CFLAGS "${XIL_CFLAGS} -fno-short-enums")
ELSE()
    SET(XIL_CFLAGS "${XIL_CFLAGS} -fshort-enums")
ENDIF()

ENDMACRO(SET_BOARD_CONFIGURATION)