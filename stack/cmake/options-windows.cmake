################################################################################
#
# CMake options for openPOWERLINK stack on Windows
#
# Copyright (c) 2016, B&R Industrial Automation GmbH
# Copyright (c) 2016, Franz Profelt (franz.profelt@gmail.com)
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

MESSAGE(STATUS "Adding CMake configuration options for Windows")

OPTION(CFG_COMPILE_LIB_MN                       "Compile openPOWERLINK MN library" ON)
OPTION(CFG_COMPILE_LIB_CN                       "Compile openPOWERLINK CN library" ON)
OPTION(CFG_COMPILE_LIB_MNAPP_PCIEINTF           "Compile openPOWERLINK MN application library for PCIe interface" ON)
OPTION(CFG_COMPILE_LIB_MNAPP_KERNELINTF         "Compile openPOWERLINK MN application library for kernel interface" ON)
OPTION(CFG_COMPILE_LIB_MN_SIM                   "Compile openPOWERLINK MN library with simulation interface" ON)
OPTION(CFG_COMPILE_LIB_CN_SIM                   "Compile openPOWERLINK CN library with simulation interface" ON)

OPTION(CFG_WINDOWS_DLL              "Build openPOWERLINK library as DLL" OFF)

################################################################################
# Options for library features

OPTION (CFG_INCLUDE_MN_REDUNDANCY               "Compile MN redundancy functions into MN libraries" OFF)
CMAKE_DEPENDENT_OPTION (CFG_STORE_RESTORE       "Support storing of OD in non-volatile memory (file system)" ON
                                                "CFG_COMPILE_LIB_CN" OFF)

# MN libraries
IF(CFG_COMPILE_LIB_MN)
    ADD_SUBDIRECTORY(proj/windows/liboplkmn)
ENDIF()

IF(CFG_COMPILE_LIB_MNAPP_PCIEINTF)
    ADD_SUBDIRECTORY(proj/windows/liboplkmnapp-pcieintf)
ENDIF()

IF(CFG_COMPILE_LIB_MNAPP_KERNELINTF)
    ADD_SUBDIRECTORY(proj/windows/liboplkmnapp-kernelintf)
ENDIF()
IF(CFG_COMPILE_LIB_MN_SIM)
    ADD_SUBDIRECTORY(proj/windows/liboplkmn-sim)
ENDIF()

# CN libraries
IF(CFG_COMPILE_LIB_CN)
    ADD_SUBDIRECTORY(proj/windows/liboplkcn)
ENDIF()
IF(CFG_COMPILE_LIB_CN_SIM)
    ADD_SUBDIRECTORY(proj/windows/liboplkcn-sim)
ENDIF()



