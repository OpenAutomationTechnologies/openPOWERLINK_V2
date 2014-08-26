################################################################################
#
# CMake settings file for cn-single-gpio demo on avnet-s6plkeb
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

#################################################################################
# D E M O   I D E N T I F I C A T I O N

# Name of the demo
SET(CFG_DEMO_NAME "mn-dual-hostif-gpio")

# Board of the demo
SET(CFG_DEMO_BOARD_NAME "avnet-lx150t")

# Bus system used in the demo
SET(CFG_DEMO_BUS_SYSTEM "axi")

#################################################################################
# P R O C E S S O R   F E A T U R E S

# Name of the POWERLINK processor
SET(CFG_PCP_NAME pcp)

# Processor type that matches CMAKE_SYSTEM_PROCESSOR in toolchain file
SET(CFG_PCP_PROCESSOR Microblaze)
# Version of the Microblaze instance
SET(CFG_PCP_CPU_VERSION "v8.50.c")

# Pcp's tightly coupled instruction memory name
SET(CFG_PCP_TCIMEM_NAME pcp_ilmb_cntlr)
# Microblaze has enabled multiplier
OPTION(CFG_PCP_MICROBLAZE_HW_MULT "Microblaze has enabled hardware multiplier" ON)
MARK_AS_ADVANCED(CFG_PCP_MICROBLAZE_HW_MULT)

# Microblaze has enabled divider
OPTION(CFG_PCP_MICROBLAZE_HW_DIV "Microblaze has enabled hardware divider" ON)
MARK_AS_ADVANCED(CFG_PCP_MICROBLAZE_HW_DIV)

# Microblaze has enabled pattern compare
OPTION(CFG_PCP_MICROBLAZE_PAT_COMP "Microblaze has enabled pattern compare" ON)
MARK_AS_ADVANCED(CFG_PCP_MICROBLAZE_PAT_COMP)

# Microblaze has enabled the barrel shifter
OPTION(CFG_PCP_MICROBLAZE_BARREL_SHIFT "Microblaze has enabled the barrel shifter" ON)
MARK_AS_ADVANCED(CFG_PCP_MICROBLAZE_BARREL_SHIFT)

# Microblaze has enabled the byte swapping instruction
OPTION(CFG_PCP_MICROBLAZE_REORDER "Microblaze has enabled the byte swapping instruction" ON)
MARK_AS_ADVANCED(CFG_PCP_MICROBLAZE_REORDER)
# Interface between host and pcp
SET(CFG_PCP_HOSTIF_ENABLE TRUE)
# Enable openMAC driver (omethlib)
SET(CFG_PCP_OMETHLIB_ENABLE "TRUE")

################################################################################
# P R O C E S S O R   F E A T U R E S   ( H O S T )

# Name of the HOST processor
SET(CFG_HOST_NAME host)

# Processor type that matches CMAKE_SYSTEM_PROCESSOR in toolchain file
SET(CFG_HOST_PROCESSOR Microblaze)

# Version of the Microblaze instance
SET(CFG_HOST_CPU_VERSION "v8.50.c")

# Host's tightly coupled instruction memory name
SET(CFG_HOST_TCIMEM_NAME host_ilmb_cntlr)

# Microblaze has enabled multiplier
OPTION(CFG_HOST_MICROBLAZE_HW_MULT "Microblaze has enabled hardware multiplier" ON)
MARK_AS_ADVANCED(CFG_HOST_MICROBLAZE_HW_MULT)

# Microblaze has enabled divider
OPTION(CFG_HOST_MICROBLAZE_HW_DIV "Microblaze has enabled hardware divider" ON)
MARK_AS_ADVANCED(CFG_HOST_MICROBLAZE_HW_DIV)

# Microblaze has enabled pattern compare
OPTION(CFG_HOST_MICROBLAZE_PAT_COMP "Microblaze has enabled pattern compare" ON)
MARK_AS_ADVANCED(CFG_HOST_MICROBLAZE_PAT_COMP)

# Microblaze has enabled the barrel shifter
OPTION(CFG_HOST_MICROBLAZE_BARREL_SHIFT "Microblaze has enabled the barrel shifter" ON)
MARK_AS_ADVANCED(CFG_HOST_MICROBLAZE_BARREL_SHIFT)

# Microblaze has enabled the byte swapping instruction
OPTION(CFG_HOST_MICROBLAZE_REORDER "Microblaze has enabled the byte swapping instruction" ON)
MARK_AS_ADVANCED(CFG_HOST_MICROBLAZE_REORDER)

# Interface between host and pcp
SET(CFG_HOST_HOSTIF_ENABLE TRUE)
