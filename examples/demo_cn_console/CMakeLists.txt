################################################################################
#
# CMake file of CiA 401 CN console demo application
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

SET(OBJDICT CiA401_CN)

# set sources of demo application
SET (DEMO_SOURCES
     main.c
     app.c
     event.c
     ${DEMO_COMMON_DIR}/pcap-console.c
     ${LIB_SOURCE_DIR}/console/printlog.c
     ${LIB_SOURCE_DIR}/getopt/getopt.c
     )

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR})
INCLUDE_DIRECTORIES(${DEMO_COMMON_DIR})

# PCAP is used for directlink of userspace daemon
IF (CFG_KERNEL_STACK_DIRECTLINK OR CFG_KERNEL_STACK_USERSPACE_DAEMON)
    ADD_DEFINITIONS(-DCONFIG_USE_PCAP)
ENDIF (CFG_KERNEL_STACK_DIRECTLINK OR CFG_KERNEL_STACK_USERSPACE_DAEMON)

IF (CFG_KERNEL_STACK_DIRECTLINK)
    INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/stack/make/lib/libpowerlink)
    ADD_DEFINITIONS(-DCONFIG_KERNELSTACK_DIRECTLINK)
ELSE (CFG_KERNEL_STACK_DIRECTLINK)
    INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/stack/make/lib/libpowerlink_user)
ENDIF (CFG_KERNEL_STACK_DIRECTLINK)

IF (UNIX)
    include(linux.cmake)
ELSEIF (WIN32)
    include (windows.cmake)
ENDIF (UNIX)

SOURCE_GROUP("Demo Sources" FILES ${DEMO_SOURCES})
SOURCE_GROUP("Architecture Specific Sources" FILES ${DEMO_ARCH_SOURCES})

ADD_EXECUTABLE(demo_cn_console ${DEMO_SOURCES} ${DEMO_ARCH_SOURCES})

SET_PROPERTY(TARGET demo_cn_console
             PROPERTY COMPILE_DEFINITIONS_DEBUG DEBUG;DEF_DEBUG_LVL=${CFG_DEBUG_LVL})

IF (CFG_KERNEL_STACK_DIRECTLINK)
    TARGET_LINK_LIBRARIES(demo_cn_console powerlink)
ELSE (CFG_KERNEL_STACK_DIRECTLINK)
    TARGET_LINK_LIBRARIES(demo_cn_console powerlink_user)
ENDIF (CFG_KERNEL_STACK_DIRECTLINK)

TARGET_LINK_LIBRARIES(demo_cn_console ${ARCH_LIBRARIES})

# add installation rules
INSTALL(TARGETS demo_cn_console RUNTIME DESTINATION bin)


