################################################################################
#
# CMake global settings file for target Xilinx Microblaze
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

###############################################################################
# Handle Microblaze settings
IF(CFG_DEMO_BUS_SYSTEM MATCHES plb)
    SET(XIL_CFLAGS "${XIL_CFLAGS} -mbig-endian")
    SET(XIL_PLAT_ENDIAN -mbig-endian)
ELSE()
    SET(XIL_CFLAGS "${XIL_CFLAGS} -mlittle-endian")
    SET(XIL_PLAT_ENDIAN -mlittle-endian)
ENDIF()

IF(CFG_MICROBLAZE_HW_MULT)
    SET(XIL_CFLAGS "${XIL_CFLAGS} -mno-xl-soft-mul")
ELSE()
    SET(XIL_CFLAGS "${XIL_CFLAGS} -mxl-soft-mul")
ENDIF()

IF(CFG_MICROBLAZE_HW_DIV)
    SET(XIL_CFLAGS "${XIL_CFLAGS} -mno-xl-soft-div")
ELSE(CFG_MICROBLAZE_HW_DIV)
    SET(XIL_CFLAGS "${XIL_CFLAGS} -mxl-soft-div")
ENDIF(CFG_MICROBLAZE_HW_DIV)

IF(CFG_MICROBLAZE_PAT_COMP)
    SET(XIL_CFLAGS "${XIL_CFLAGS} -mxl-pattern-compare")
ELSE()
    SET(XIL_CFLAGS "${XIL_CFLAGS} -mno-xl-pattern-compare")
ENDIF()

IF(CFG_MICROBLAZE_BARREL_SHIFT)
    SET(XIL_CFLAGS "${XIL_CFLAGS} -mxl-barrel-shift")
ELSE ()
    SET(XIL_CFLAGS "${XIL_CFLAGS} -mno-xl-barrel-shift")
ENDIF()

IF (CFG_MICROBLAZE_REORDER)
    SET(XIL_CFLAGS "${XIL_CFLAGS} -mxl-reorder")
ELSE ()
    SET(XIL_CFLAGS "${XIL_CFLAGS} -mno-xl-reorder")
ENDIF()
