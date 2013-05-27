################################################################################
#
# Windows CMake configuration for openPOWERLINK stack library
#
# Copyright (c) 2013, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
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

ADD_DEFINITIONS(-D_CONSOLE -DWPCAP -DHAVE_REMOTE -D_CRT_SECURE_NO_WARNINGS)

#
# Set type of library
#
IF(CFG_X86_WINDOWS_DLL)
    SET(LIB_TYPE "SHARED")
ELSE(CFG_X86_WINDOWS_DLL)
    SET(LIB_TYPE "STATIC")
ENDIF(CFG_X86_WINDOWS_DLL)

SET (LIB_ARCH_SOURCES
     ${LIB_ARCH_SOURCES}
     ${EDRV_SOURCE_DIR}/edrv-pcap.c
     ${COMMON_SOURCE_DIR}/timer/timer-generic.c
     ${LIB_SOURCE_DIR}/circbuf/circbuf-win32.c
     ${LIB_SOURCE_DIR}/trace/trace-windows.c
     ${ARCH_SOURCE_DIR}/windows/target-windows.c
     ${USER_SOURCE_DIR}/event/eventucal-win32.c
     ${USER_SOURCE_DIR}/event/eventucalintf-circbuf.c
     ${KERNEL_SOURCE_DIR}/event/eventkcal-win32.c
     ${KERNEL_SOURCE_DIR}/event/eventkcalintf-circbuf.c
     )

INCLUDE_DIRECTORIES(${INCLUDE_DIRECTORIES}
                    ${LIB_SOURCE_DIR}/pcap/windows/WpdPack/Include
                    )

#
# Library include directories
#
IF (CMAKE_CL_64)
    LINK_DIRECTORIES(${LIB_SOURCE_DIR}/pcap/windows/WpdPack/Lib/x64)
ELSE (CMAKE_CL_64)
    LINK_DIRECTORIES(${LIB_SOURCE_DIR}/pcap/windows/WpdPack/Lib)
ENDIF (CMAKE_CL_64)

SET (ARCH_LIBRARIES wpcap iphlpapi)


