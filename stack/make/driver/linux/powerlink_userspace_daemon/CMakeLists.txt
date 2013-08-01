################################################################################
#
# \file CMakeLists.txt
#
# \brief cmake file for openPOWERLINK kernel stack process
#
# Copyright (c) 2012, SYSTEC electronik GmbH
# Copyright (c) 2012, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
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
#
################################################################################

IF(CFG_POWERLINK_MN)
    SET(EXE_NAME powerlink_mn_daemon)
    ADD_DEFINITIONS(-DCONFIG_MN)
ELSE(CFG_POWERLINK_MN)
    SET(EXE_NAME powerlink_cn_daemon)
ENDIF(CFG_POWERLINK_MN)

#
# Source include directories
#
# set sources of POWERLINK application
SET (DAEMON_MAIN_SOURCES
     daemon.c
     )

SET (DAEMON_COMMON_SOURCES
     ${COMMON_SOURCE_DIR}/debug.c
     ${COMMON_SOURCE_DIR}/event/event.c
     ${LIB_SOURCE_DIR}/ami/amix86.c
     ${LIB_SOURCE_DIR}/circbuf/circbuffer.c
     ${EDRV_SOURCE_DIR}/edrvcyclic.c
     )

SET (DAEMON_KERNEL_SOURCES
     ${KERNEL_SOURCE_DIR}/dll/dllk.c
     ${KERNEL_SOURCE_DIR}/dll/dllkfilter.c
     ${KERNEL_SOURCE_DIR}/dll/dllkstatemachine.c
     ${KERNEL_SOURCE_DIR}/dll/dllkevent.c
     ${KERNEL_SOURCE_DIR}/dll/dllkframe.c
     ${KERNEL_SOURCE_DIR}/dll/dllkcal.c
     ${KERNEL_SOURCE_DIR}/dll/dllkcal-circbuf.c
     ${KERNEL_SOURCE_DIR}/event/eventk.c
     ${KERNEL_SOURCE_DIR}/nmt/nmtk.c
     ${KERNEL_SOURCE_DIR}/pdo/pdok.c
     ${KERNEL_SOURCE_DIR}/pdo/pdokcal.c
     ${KERNEL_SOURCE_DIR}/pdo/pdokcal-triplebufshm.c
     ${KERNEL_SOURCE_DIR}/errhnd/errhndk.c
     ${KERNEL_SOURCE_DIR}/errhnd/errhndkcal-posixshm.c
     ${KERNEL_SOURCE_DIR}/ctrl/ctrlk.c
     ${KERNEL_SOURCE_DIR}/ctrl/ctrlkcal-mem.c
     )

SET (DAEMON_HEADERS
     ./EplCfg.h
     ${POWERLINK_INCLUDE_DIR}/Benchmark.h
     ${POWERLINK_INCLUDE_DIR}/Debug.h
     ${POWERLINK_INCLUDE_DIR}/edrv.h
     ${POWERLINK_INCLUDE_DIR}/Epl.h
     ${POWERLINK_INCLUDE_DIR}/EplAmi.h
     ${POWERLINK_INCLUDE_DIR}/EplCfm.h
     ${POWERLINK_INCLUDE_DIR}/EplDef.h
     ${POWERLINK_INCLUDE_DIR}/EplDll.h
     ${POWERLINK_INCLUDE_DIR}/dllcal.h
     ${POWERLINK_INCLUDE_DIR}/EplErrDef.h
     ${POWERLINK_INCLUDE_DIR}/event.h
     ${POWERLINK_INCLUDE_DIR}/EplFrame.h
     ${POWERLINK_INCLUDE_DIR}/EplInc.h
     ${POWERLINK_INCLUDE_DIR}/EplInstDef.h
     ${POWERLINK_INCLUDE_DIR}/nmt.h
     ${POWERLINK_INCLUDE_DIR}/EplObd.h
     ${POWERLINK_INCLUDE_DIR}/EplObdMacro.h
     ${POWERLINK_INCLUDE_DIR}/EplObjDef.h
     ${POWERLINK_INCLUDE_DIR}/pdo.h
     ${POWERLINK_INCLUDE_DIR}/EplSdo.h
     ${POWERLINK_INCLUDE_DIR}/EplSdoAc.h
     ${POWERLINK_INCLUDE_DIR}/EplTarget.h
     ${POWERLINK_INCLUDE_DIR}/EplTimer.h
     ${POWERLINK_INCLUDE_DIR}/EplVersion.h
     ${POWERLINK_INCLUDE_DIR}/global.h
     ${POWERLINK_INCLUDE_DIR}/errhnd.h
     ${POWERLINK_INCLUDE_DIR}/ctrl.h
     ${POWERLINK_INCLUDE_DIR}/ctrlcal.h
     ${LIB_SOURCE_DIR}/sharedbuff/SharedBuff.h
     ${LIB_SOURCE_DIR}/sharedbuff/ShbIpc.h
     )

SET (DAEMON_KERNEL_HEADERS
     ${KERNEL_INCLUDE_DIR}/dllk.h
     ${KERNEL_INCLUDE_DIR}/dllkcal.h
     ${KERNEL_INCLUDE_DIR}/dllktgt.h
     ${KERNEL_INCLUDE_DIR}/errhndk.h
     ${KERNEL_INCLUDE_DIR}/ctrlk.h
     ${KERNEL_INCLUDE_DIR}/ctrlkcal.h
     ${KERNEL_INCLUDE_DIR}/eventk.h
     ${KERNEL_INCLUDE_DIR}/eventkcal.h
     ${KERNEL_INCLUDE_DIR}/nmtk.h
     ${KERNEL_INCLUDE_DIR}/EplObdk.h
     ${KERNEL_INCLUDE_DIR}/pdok.h
     ${KERNEL_INCLUDE_DIR}/pdokcal.h
     ${KERNEL_INCLUDE_DIR}/EplTimerHighResk.h
     ${KERNEL_INCLUDE_DIR}/EplTimerk.h
     )

#INCLUDE_DIRECTORIES(${USERLIB_INCLUDE_DIR})
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR})

IF (UNIX)
    include(linux.cmake)
ELSEIF (WIN32)
    include (windows.cmake)
ENDIF (UNIX)

#
# Group source files
#
SOURCE_GROUP("Source Files" FILES ${DEAMON_MAIN_SOURCES})
SOURCE_GROUP("Source Files\\Common Source Files" FILES ${DAEMON_COMMON_SOURCES})
SOURCE_GROUP("Source Files\\Kernel Source Files" FILES ${DAEMON_KERNEL_SOURCES})
SOURCE_GROUP("Source Files\\Architecture Source Files" FILES ${DAEMON_ARCH_SOURCES})
SOURCE_GROUP("Header Files" FILES ${DAEMON_HEADERS})
SOURCE_GROUP("Header Files\\Kernel Header Files" FILES ${DAEMON_KERNEL_HEADERS})

ADD_EXECUTABLE(${EXE_NAME}
               ${DAEMON_MAIN_SOURCES}
               ${DAEMON_ARCH_SOURCES}
               ${DAEMON_COMMON_SOURCES}
               ${DAEMON_KERNEL_SOURCES}
               )

SET_PROPERTY(TARGET ${EXE_NAME}
             PROPERTY COMPILE_DEFINITIONS_DEBUG DEBUG;DEF_DEBUG_LVL=${CFG_DEBUG_LVL})

TARGET_LINK_LIBRARIES(${EXE_NAME} ${ARCH_LIBRARIES})

# add installation rules
INSTALL(TARGETS ${EXE_NAME} RUNTIME DESTINATION bin)

