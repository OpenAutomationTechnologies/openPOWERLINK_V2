################################################################################
#
# CMake options for openPOWERLINK stack on Linux
#
# Copyright (c) 2016, B&R Industrial Automation GmbH
# Copyright (c) 2016, Franz Profelt (franz.profelt@gmail.com)
# Copyright (c) 2018, Kalycito Infotech Private Limited
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

MESSAGE(STATUS "Adding CMAKE configuration options for Linux")

################################################################################
# Options for MN libraries

OPTION (CFG_COMPILE_LIB_MN                      "Compile openPOWERLINK MN library" ON)
OPTION (CFG_COMPILE_LIB_MNAPP_USERINTF          "Compile openPOWERLINK MN application library for userspace" ON)
OPTION (CFG_COMPILE_LIB_MNAPP_KERNELINTF        "Compile openPOWERLINK MN application library for kernel interface" ON)
OPTION (CFG_COMPILE_LIB_MNAPP_PCIEINTF          "Compile openPOWERLINK MN application library for PCIe interface" ON)
OPTION (CFG_COMPILE_LIB_MNAPP_ZYNQINTF          "Compile openPOWERLINK MN application library for zynq/FPGA interface" ON)
OPTION (CFG_COMPILE_LIB_MNDRV_PCAP              "Compile openPOWERLINK MN driver library for linux userspace (pcap)" ON)
OPTION (CFG_COMPILE_LIB_MN_SIM                  "Compile openPOWERLINK MN library with simulation interface" ON)

################################################################################
# Options for CN libraries

OPTION (CFG_COMPILE_LIB_CN                      "Compile openPOWERLINK CN library" ON)
OPTION (CFG_COMPILE_LIB_CNAPP_USERINTF          "Compile openPOWERLINK CN application library for userspace" ON)
OPTION (CFG_COMPILE_LIB_CNAPP_KERNELINTF        "Compile openPOWERLINK CN application library for kernel interface" ON)
OPTION (CFG_COMPILE_LIB_CNAPP_ZYNQINTF          "Compile openPOWERLINK CN application library for zynq/FPGA interface" ON)
OPTION (CFG_COMPILE_LIB_CNDRV_PCAP              "Compile openPOWERLINK CN driver library for linux userspace (pcap)" ON)
OPTION (CFG_COMPILE_LIB_CN_SIM                  "Compile openPOWERLINK CN library with simulation interface" ON)

################################################################################
# Options for shared libraries

OPTION (CFG_COMPILE_SHARED_LIBRARY              "Build openPOWERLINK library as shared library" OFF)

################################################################################
# Options for library features

OPTION (CFG_USE_PCAP_EDRV                       "Compile openPOWERLINK library with pcap edrv" OFF)
OPTION (CFG_INCLUDE_MN_REDUNDANCY               "Compile MN redundancy functions into MN libraries" OFF)
CMAKE_DEPENDENT_OPTION (CFG_STORE_RESTORE       "Support storing of OD in non-volatile memory (file system)" ON
                                                "CFG_COMPILE_LIB_CN OR CFG_COMPILE_LIB_CNAPP_USERINTF OR CFG_COMPILE_LIB_CNAPP_KERNELINTF" OFF)

################################################################################
# Add library subdirectories

# Add MN libraries
IF(CFG_COMPILE_LIB_MN)
    ADD_SUBDIRECTORY(proj/linux/liboplkmn)
ENDIF()

IF(CFG_COMPILE_LIB_MNAPP_USERINTF)
    ADD_SUBDIRECTORY(proj/linux/liboplkmnapp-userintf)
ENDIF()

IF(CFG_COMPILE_LIB_MNAPP_KERNELINTF)
    ADD_SUBDIRECTORY(proj/linux/liboplkmnapp-kernelintf)
ENDIF()

IF((CFG_COMPILE_LIB_MNAPP_PCIEINTF) OR (CFG_COMPILE_LIB_MNAPP_ZYNQINTF))
    ADD_SUBDIRECTORY(proj/linux/liboplkmnapp-kernelpcp)
ENDIF()

IF(CFG_COMPILE_LIB_MNDRV_PCAP)
    ADD_SUBDIRECTORY(proj/linux/liboplkmndrv-pcap)
ENDIF()

IF(CFG_COMPILE_LIB_MN_SIM)
    ADD_SUBDIRECTORY(proj/linux/liboplkmn-sim)
ENDIF()

# Add CN libraries
IF(CFG_COMPILE_LIB_CN)
    ADD_SUBDIRECTORY(proj/linux/liboplkcn)
ENDIF()

IF(CFG_COMPILE_LIB_CNAPP_USERINTF)
    ADD_SUBDIRECTORY(proj/linux/liboplkcnapp-userintf)
ENDIF()

IF(CFG_COMPILE_LIB_CNAPP_KERNELINTF)
    ADD_SUBDIRECTORY(proj/linux/liboplkcnapp-kernelintf)
ENDIF()

IF(CFG_COMPILE_LIB_CNAPP_ZYNQINTF)
    ADD_SUBDIRECTORY(proj/linux/liboplkcnapp-kernelpcp)
ENDIF()

IF(CFG_COMPILE_LIB_CNDRV_PCAP)
    ADD_SUBDIRECTORY(proj/linux/liboplkcndrv-pcap)
ENDIF()

IF(CFG_COMPILE_LIB_CN_SIM)
    ADD_SUBDIRECTORY(proj/linux/liboplkcn-sim)
ENDIF()
