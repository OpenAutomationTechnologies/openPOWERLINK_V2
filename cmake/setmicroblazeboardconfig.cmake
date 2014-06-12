################################################################################
#
# CMake macro for setting the board managing the configuration of the current
# selected board.
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

MACRO(SET_BOARD_CONFIGURATION PATH_TO_CONFIGURATION)

###############################################################################
# Unset all old configurations
UNSET(CFG_DEMO_NAME)
UNSET(CFG_DEMO_BOARD_NAME)
UNSET(CFG_DEMO_BUS_SYSTEM)
UNSET(CFG_CDC_ONSDCARD)

FOREACH(PROC_INST_NAME PCP HOST)
    UNSET(CFG_${PROC_INST_NAME}_NAME)
    UNSET(CFG_${PROC_INST_NAME}_CPU_VERSION)
    UNSET(CFG_${PROC_INST_NAME}_TCIMEM_NAME)
    UNSET(CFG_${PROC_INST_NAME}_MICROBLAZE_HW_MULT)
    UNSET(CFG_${PROC_INST_NAME}_MICROBLAZE_HW_DIV)
    UNSET(CFG_${PROC_INST_NAME}_MICROBLAZE_PAT_COMP)
    UNSET(CFG_${PROC_INST_NAME}_MICROBLAZE_BARREL_SHIFT)
    UNSET(CFG_${PROC_INST_NAME}_MICROBLAZE_REORDER)
    UNSET(CFG_${PROC_INST_NAME}_ARM_FRAME_AAPCS)
    UNSET(CFG_${PROC_INST_NAME}_ARM_SHORT_ENUMS)
    UNSET(CFG_${PROC_INST_NAME}_DUALPROCSHM_ENABLE)
    UNSET(CFG_${PROC_INST_NAME}_OMETHLIB_ENABLE)
    UNSET(CFG_${PROC_INST_NAME}_HOSTIF_ENABLE)
ENDFOREACH()

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
FOREACH(PROC_INST_NAME PCP HOST)
    IF(CFG_DEMO_BUS_SYSTEM MATCHES plb)
        SET(XIL_${PROC_INST_NAME}_CFLAGS "${XIL_${PROC_INST_NAME}_CFLAGS} -mbig-endian")
        SET(XIL_${PROC_INST_NAME}_PLAT_ENDIAN -mbig-endian)
    ELSE()
        SET(XIL_${PROC_INST_NAME}_CFLAGS "${XIL_${PROC_INST_NAME}_CFLAGS} -mlittle-endian")
        SET(XIL_${PROC_INST_NAME}_PLAT_ENDIAN -mlittle-endian)
    ENDIF()

    IF(CFG_${PROC_INST_NAME}_MICROBLAZE_HW_MULT)
        SET(XIL_${PROC_INST_NAME}_CFLAGS "${XIL_${PROC_INST_NAME}_CFLAGS} -mno-xl-soft-mul")
    ELSE()
        SET(XIL_${PROC_INST_NAME}_CFLAGS "${XIL_${PROC_INST_NAME}_CFLAGS} -mxl-soft-mul")
    ENDIF()

    IF(CFG_${PROC_INST_NAME}_MICROBLAZE_HW_DIV)
        SET(XIL_${PROC_INST_NAME}_CFLAGS "${XIL_${PROC_INST_NAME}_CFLAGS} -mno-xl-soft-div")
    ELSE()
        SET(XIL_${PROC_INST_NAME}_CFLAGS "${XIL_${PROC_INST_NAME}_CFLAGS} -mxl-soft-div")
    ENDIF()

    IF(CFG_${PROC_INST_NAME}_MICROBLAZE_PAT_COMP)
        SET(XIL_${PROC_INST_NAME}_CFLAGS "${XIL_${PROC_INST_NAME}_CFLAGS} -mxl-pattern-compare")
    ELSE()
        SET(XIL_${PROC_INST_NAME}_CFLAGS "${XIL_${PROC_INST_NAME}_CFLAGS} -mno-xl-pattern-compare")
    ENDIF()

    IF(CFG_${PROC_INST_NAME}_MICROBLAZE_BARREL_SHIFT)
        SET(XIL_${PROC_INST_NAME}_CFLAGS "${XIL_${PROC_INST_NAME}_CFLAGS} -mxl-barrel-shift")
    ELSE ()
        SET(XIL_${PROC_INST_NAME}_CFLAGS "${XIL_${PROC_INST_NAME}_CFLAGS} -mno-xl-barrel-shift")
    ENDIF()

    IF (CFG_${PROC_INST_NAME}_MICROBLAZE_REORDER)
        SET(XIL_${PROC_INST_NAME}_CFLAGS "${XIL_${PROC_INST_NAME}_CFLAGS} -mxl-reorder")
    ELSE ()
        SET(XIL_${PROC_INST_NAME}_CFLAGS "${XIL_${PROC_INST_NAME}_CFLAGS} -mno-xl-reorder")
    ENDIF()
ENDFOREACH()

ENDMACRO(SET_BOARD_CONFIGURATION)