################################################################################
#
# CMake settings file for mn-dual-hostif-gpio demo on altera-c5soc
#
# Copyright (c) 2015, Kalycito Infotech Pvt. Ltd.
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
SET(CFG_DEMO_NAME "mn-soc-shmem-gpio")

# Board of the demo
SET(CFG_DEMO_BOARD_NAME "altera-c5soc")

# Bus system used in the demo
SET(CFG_DEMO_BUS_SYSTEM "axi")

# Boot ARM from SD card
OPTION(CFG_HOST_BOOT_FROM_SDCARD "Boot ARM from SD card" ON)

# Enable semihosting operation for debugging from DS5
#CMAKE_DEPENDENT_OPTION(CFG_HOST_SEMIHOSTING_ENABLE "Debug ARM core from DS5" ON
#                       "NOT CFG_HOST_BOOT_FROM_SDCARD" OFF)
IF (NOT CFG_HOST_BOOT_FROM_SDCARD)
    OPTION(CFG_HOST_SEMIHOSTING_ENABLE "Debug ARM core from DS5" ON)
ELSE()
    SET(CFG_HOST_SEMIHOSTING_ENABLE OFF)
ENDIF()

# Enable CDC file on SDCARD
#SET(CFG_CDC_ONSDCARD FALSE)

# Bootloader Available for ARM
OPTION(CFG_HOST_BOOTLOADER_ENABLE "Build ARM Preloader" ON)

# BSP generation not Available for ARM
SET(CFG_HOST_HAL_TYPE "hwlib")

# Linker scripts used for ARM
SET(CFG_HOST_SEMIHOSTED_LINKER cycloneV-dk-ram-semihosted.ld)
SET(CFG_HOST_UNHOSTED_LINKER cycloneV-dk-ram-unhosted.ld)
################################################################################
# P R O C E S S O R   F E A T U R E S   ( H O S T )

# Qsys subsystem name of processor that processes host/app
SET(CFG_HOST_SUB_NAME "host_0")
# Processor that processes host/app
SET(CFG_HOST_PROC_NAME "hps_0")
SET(CFG_HOST_PROC_IP_NAME "hps")

# Name of host subsystem
SET(CFG_HOST_NAME ${CFG_HOST_SUB_NAME}_${CFG_HOST_PROC_NAME})

# Processor type that matches CMAKE_SYSTEM_PROCESSOR in toolchain file
SET(CFG_HOST_PROCESSOR alterac5arm)

# Version of the Microblaze instance
SET(CFG_HOST_CPU_VERSION "cortex-a9")

# Host's tightly coupled instruction memory name
SET(CFG_HOST_TCIMEM_NAME dummy)

# ARM core has enabled frame compliant with AAPCS
OPTION(CFG_HOST_ARM_FRAME_AAPCS "ARM core has Stack frame compliant with AAPCS" ON)
MARK_AS_ADVANCED(CFG_HOST_ARM_FRAME_AAPCS)

# ARM core has enabled frame compliant with AAPCS
OPTION(CFG_HOST_ARM_USE_CACHE "ARM core uses cache" ON)
MARK_AS_ADVANCED(CFG_HOST_ARM_USE_CACHE)

# ARM core should use 4 byte enums
OPTION(CFG_HOST_ARM_SHORT_ENUMS "ARM core uses 4-bytes enums" ON)
MARK_AS_ADVANCED(CFG_HOST_ARM_SHORT_ENUMS)

# ARM has enabled divider
OPTION(CFG_HOST_ARM_HW_FLOAT "ARM has enabled onchip FPU" OFF)
MARK_AS_ADVANCED(CFG_HOST_ARM_HW_FLOAT)

################################################################################
# E N A B L E   P R O C E S S O R   S O F T W A R E   ( H O S T )

# Interface between host and pcp
SET(CFG_HOST_DUALPROCSHM_ENABLE TRUE)
