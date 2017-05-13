################################################################################
#
# CMake settings file for mn-dual-shmem-gpio demo on xilinx-z702
#
# Copyright (c) 2016, Kalycito Infotech Pvt. Ltd.
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
# D E M O   I D E N T I F I C A T I O N

# Name of the demo
SET(CFG_DEMO_NAME "mn-dual-shmem-gpio")

# Board of the demo
SET(CFG_DEMO_BOARD_NAME "xilinx-z702")

# Identify board demo as Zynq
SET(CFG_DEMO_BOARD_ZYNQ "TRUE")

# Bus system used in the demo
SET(CFG_DEMO_BUS_SYSTEM "axi")

# Enable FAT FS support library build
# Useful when reading CDC file from SDCARD
SET(CFG_FAT_SUPPORT FALSE)

################################################################################
# P R O C E S S O R   F E A T U R E S   ( P C P )

# Name of the POWERLINK processor
SET(CFG_PCP_NAME pcp)

# Processor type that matches CMAKE_SYSTEM_PROCESSOR in toolchain file
SET(CFG_PCP_PROCESSOR Microblaze)

# Version of the Microblaze instance
SET(CFG_PCP_CPU_VERSION "v9.6")

# Pcp's tightly coupled instruction memory name
SET(CFG_PCP_TCIMEM_NAME pcp_bram_ilmb_bram_if_cntlr)

# Bootloader not available for Microblaze
SET(CFG_MB_BOOTLOADER_ENABLE "FALSE")

SET(CFG_MB_UART "TRUE")

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

################################################################################
# E N A B L E   P R O C E S S O R   S O F T W A R E   ( P C P )

# Enable Dual Processor Shared Memory in contrib directory
SET(CFG_PCP_DUALPROCSHM_ENABLE "TRUE")

# Enable openMAC driver (omethlib)
SET(CFG_PCP_OMETHLIB_ENABLE "TRUE")
