################################################################################
#
# CMake settings file for mn-dual-shmem-gpio demo on xilinx-z702
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

#################################################################################
# D E M O   I D E N T I F I C A T I O N

# Name of the demo
SET(CFG_DEMO_NAME "mn-dual-shmem-gpio")

# Board of the demo
SET(CFG_DEMO_BOARD_NAME "xilinx-z702")

# Bus system used in the demo
SET(CFG_DEMO_BUS_SYSTEM "axi")

# Host's tightly coupled instruction memory name
SET(CFG_HOST_TCIMEM_NAME dummy)

# Dual processor design
SET(CFG_DESIGN_DUAL "Yes")

#################################################################################
# P R O C E S S O R   F E A T U R E S

#Name of the POWERLINK processor
SET(CFG_HOST_NAME ps7_cortexa9_0)

# Version of the Microblaze instance
SET(CFG_CPU_VERSION "cortex-a9")

SET(CFG_DUALPROCSHM_ENABLE "TRUE")

SET(CFG_CDC_ONSDCARD "TRUE")

# ARM core has enabled frame compliant with AAPCS
OPTION(CFG_ARM_FRAME_AAPCS "ARM core has Stack frame compliant with AAPCS" ON)
MARK_AS_ADVANCED(CFG_ARM_FRAME_AAPCS)

# ARM core should use 4 byte enums
OPTION(CFG_ARM_SHORT_ENUMS "ARM core uses 4-bytes enums" ON)
MARK_AS_ADVANCED(CFG_ARM_SHORT_ENUMS)


