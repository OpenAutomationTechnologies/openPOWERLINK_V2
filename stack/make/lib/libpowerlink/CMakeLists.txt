################################################################################
#
# Project: openPOWERLINK
#
# (c) Bernecker + Rainer Industrie-Elektronik Ges.m.b.H.
#     B&R Strasse 1, A-5142 Eggelsberg
#     www.br-automation.com
#
# Description: cmake file of userspace openPOWERLINK library
#
# License:
#
#   Redistribution and use in source and binary forms, with or without
#   modification, are permitted provided that the following conditions
#   are met:
#
#   1. Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#   2. Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#   3. Neither the name of the copyright holders nor the names of its
#      contributors may be used to endorse or promote products derived
#      from this software without prior written permission. For written
#      permission, please contact office@br-automation.com.
#
#   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#   FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#   COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#   INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#   BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#   CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#   POSSIBILITY OF SUCH DAMAGE.
#
#   Severability Clause:
#
#       If a provision of this License is or becomes illegal, invalid or
#       unenforceable in any jurisdiction, that shall not affect:
#       1. the validity or enforceability in that jurisdiction of any other
#          provision of this License; or
#       2. the validity or enforceability in other jurisdictions of that or
#          any other provision of this License.
#
################################################################################

#
# additional compiler flags
#
IF (UNIX)
    ADD_DEFINITIONS(-Wall -Wextra -pedantic -std=c99 -pthread -D_GNU_SOURCE -D_POSIX_C_SOURCE=200112L)
ELSEIF (WIN32)
    ADD_DEFINITIONS(-D_CONSOLE -DWPCAP -DHAVE_REMOTE)
ENDIF (UNIX)

# Select object dictionary
IF(CFG_POWERLINK_MN)
    SET(OBJDICT CiA302-4_MN)
ELSE(CFG_POWERLINK_MN)
    SET(OBJDICT CiA401_CN)
ENDIF(CFG_POWERLINK_MN)

#
# Configure depending selected mode
#
IF(CFG_POWERLINK_MN)
    SET(LIB_SOURCES ${LIB_SOURCES}
                    ${USER_SOURCE_DIR}/obd/obdcdc.c
        )
    SET (LIB_SOURCES_USER ${LIB_SOURCES_USER}
                          ${USER_SOURCE_DIR}/cfmu.c
        )
    SET (LIB_HEADERS_USER ${LIB_HEADERS_USER}
                          ${USER_INCLUDE_DIR}/EplCfmu.h
        )

    ADD_DEFINITIONS(-DCONFIG_CFM -DCONFIG_OPENCONFIGURATOR_MAPPING -DCONFIG_MN)
ENDIF(CFG_POWERLINK_MN)

#
# Set type of library
#
IF (WIN32)
    IF(CFG_X86_WINDOWS_DLL)
        SET(LIB_TYPE "SHARED")
    ELSE(CFG_X86_WINDOWS_DLL)
        SET(LIB_TYPE "STATIC")
    ENDIF(CFG_X86_WINDOWS_DLL)
ENDIF ()

#
# setup ethernet driver to use
#
IF (UNIX)
    SET(LIB_SOURCES ${LIB_SOURCES}
                    ${EDRV_SOURCE_DIR}/edrv-pcap_linux.c
        )
ELSEIF (WIN32)
    SET(LIB_SOURCES ${LIB_SOURCES}
                    ${EDRV_SOURCE_DIR}/edrv-pcap.c
        )
ENDIF ()

#
# setup object dictionary to use
#
SET(LIB_OBJDICT ${LIB_OBJDICT}
                ${OBJDICT_DIR}/${OBJDICT}/Objdict.c
                ${OBJDICT_DIR}/${OBJDICT}/objdict.h
                ${OBJDICT_DIR}/${OBJDICT}/obdcfg.h
                ${OBJDICT_DIR}/generic/objdict_1000-13ff.h
                ${OBJDICT_DIR}/generic/objdict_1b00-1fff.h
    )

#
# set general sources of POWERLINK library
#
SET (LIB_SOURCES ${LIB_SOURCES}
                 ${USER_SOURCE_DIR}/api/generic.c
                 ${USER_SOURCE_DIR}/api/processimage.c
                 ${COMMON_SOURCE_DIR}/debug.c
                 ${USER_SOURCE_DIR}/obd/obd.c
                 ${COMMON_SOURCE_DIR}/dll/dllcal-shb.c
                 ${COMMON_SOURCE_DIR}/event/event.c
                 ${COMMON_SOURCE_DIR}/event/eventcal-shb.c
                 ${LIB_SOURCE_DIR}/ami/amix86.c
                 ${LIB_SOURCE_DIR}/sharedbuff/SharedBuff.c
                 ${EDRV_SOURCE_DIR}/edrvcyclic.c
    )
    

IF (UNIX)
    SET (LIB_SOURCES ${LIB_SOURCES}
                     ${USER_SOURCE_DIR}/sdo/sdo-udpu.c
                     ${COMMON_SOURCE_DIR}/timer/timer-linuxuser.c
                     ${KERNEL_SOURCE_DIR}/hrtimer/hrtimer-posix.c
                     ${LIB_SOURCE_DIR}/sharedbuff/ShbIpc-LinuxPthreads.c
                     ${ARCH_SOURCE_DIR}/linux/ftrace-debug.c
                     ${ARCH_SOURCE_DIR}/linux/targetinit-posix.c
        )
ELSEIF (WIN32)
    SET (LIB_SOURCES ${LIB_SOURCES}
                     ${COMMON_SOURCE_DIR}/timer/timer-generic.c
                     ${LIB_SOURCE_DIR}/sharedbuff/ShbIpc-Win32.c
                     ${LIB_SOURCE_DIR}/misc/trace.c
        )
ENDIF ()

#
# set kernel part sources of POWERLINK library
#
SET (LIB_SOURCES_KERNEL ${LIB_SOURCES_KERNEL}
                        ${KERNEL_SOURCE_DIR}/dll/dllk.c
                        ${KERNEL_SOURCE_DIR}/dll/dllkcal.c
                        ${KERNEL_SOURCE_DIR}/event/eventk.c
                        ${KERNEL_SOURCE_DIR}/event/eventkcal.c
                        ${KERNEL_SOURCE_DIR}/event/eventkcal-shb.c
                        ${KERNEL_SOURCE_DIR}/nmt/nmtk.c
                        ${KERNEL_SOURCE_DIR}/nmt/nmtkcal.c
                        ${KERNEL_SOURCE_DIR}/pdo/pdok.c
                        ${KERNEL_SOURCE_DIR}/pdo/pdokcal.c
                        ${KERNEL_SOURCE_DIR}/pdo/pdokcal-shb.c
                        ${KERNEL_SOURCE_DIR}/pdo/pdokcalsync-null.c
                        ${KERNEL_SOURCE_DIR}/errhnd/errhndk.c
                        ${KERNEL_SOURCE_DIR}/errhnd/errhndkcal-shb.c
                        ${KERNEL_SOURCE_DIR}/ctrl/ctrlk.c
                        ${KERNEL_SOURCE_DIR}/ctrl/ctrlkcal-direct.c
    )

#
# set user part sources of POWERLINK library
#
SET (LIB_SOURCES_USER ${LIB_SOURCES_USER}
                      ${USER_SOURCE_DIR}/dll/dllucal.c
                      ${USER_SOURCE_DIR}/event/eventu.c
                      ${USER_SOURCE_DIR}/event/eventucal.c
                      ${USER_SOURCE_DIR}/event/eventucal-shb.c
                      ${USER_SOURCE_DIR}/nmt/nmtu.c
                      ${USER_SOURCE_DIR}/nmt/nmtucal.c
                      ${USER_SOURCE_DIR}/nmt/nmtcnu.c
                      ${USER_SOURCE_DIR}/nmt/nmtmnu.c
                      ${USER_SOURCE_DIR}/nmt/identu.c
                      ${USER_SOURCE_DIR}/nmt/statusu.c
                      ${USER_SOURCE_DIR}/nmt/syncu.c
                      ${USER_SOURCE_DIR}/pdo/pdou.c
                      ${USER_SOURCE_DIR}/pdo/pdoucal.c
                      ${USER_SOURCE_DIR}/pdo/pdoucal-shb.c
                      ${USER_SOURCE_DIR}/pdo/pdoucalsync-null.c
                      ${USER_SOURCE_DIR}/sdo/sdo-comu.c
                      ${USER_SOURCE_DIR}/sdo/sdo-asysequ.c
                      ${USER_SOURCE_DIR}/sdo/sdo-asndu.c
                      ${USER_SOURCE_DIR}/errhnd/errhndu.c
                      ${USER_SOURCE_DIR}/errhnd/errhnducal-shb.c
                      ${USER_SOURCE_DIR}/ctrl/ctrlu.c
                      ${USER_SOURCE_DIR}/ctrl/ctrlucal-direct.c
    )

#
# set general header files of POWERLINK library
#
SET (LIB_HEADERS ${LIB_HEADERS}
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
                 ${POWERLINK_INCLUDE_DIR}/eventcal.h
                 ${POWERLINK_INCLUDE_DIR}/EplFrame.h
                 ${POWERLINK_INCLUDE_DIR}/EplInc.h
                 ${POWERLINK_INCLUDE_DIR}/EplInstDef.h
                 ${POWERLINK_INCLUDE_DIR}/EplNmt.h
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

#
# set kernel part header files of POWERLINK library
#
SET (LIB_HEADERS_KERNEL ${LIB_HEADERS_KERNEL}
                        ${KERNEL_INCLUDE_DIR}/EplDllk.h
                        ${KERNEL_INCLUDE_DIR}/dllkcal.h
                        ${KERNEL_INCLUDE_DIR}/EplDllkTgt.h
                        ${KERNEL_INCLUDE_DIR}/errhndk.h
                        ${KERNEL_INCLUDE_DIR}/ctrlk.h
                        ${KERNEL_INCLUDE_DIR}/ctrlkcal.h
                        ${KERNEL_INCLUDE_DIR}/eventk.h
                        ${KERNEL_INCLUDE_DIR}/eventkcal.h
                        ${KERNEL_INCLUDE_DIR}/EplNmtk.h
                        ${KERNEL_INCLUDE_DIR}/EplNmtkCal.h
                        ${KERNEL_INCLUDE_DIR}/EplObdk.h
                        ${KERNEL_INCLUDE_DIR}/pdok.h
                        ${KERNEL_INCLUDE_DIR}/pdokcal.h
                        ${KERNEL_INCLUDE_DIR}/EplTimerHighResk.h
                        ${KERNEL_INCLUDE_DIR}/EplTimerk.h
    )

#
# set user part header files of POWERLINK library
#
SET (LIB_HEADERS_USER ${LIB_HEADERS_USER}
                      ${USER_INCLUDE_DIR}/EplDllu.h
                      ${USER_INCLUDE_DIR}/dllucal.h
                      ${USER_INCLUDE_DIR}/eventu.h
                      ${USER_INCLUDE_DIR}/eventucal.h
                      ${USER_INCLUDE_DIR}/errhndu.h
                      ${USER_INCLUDE_DIR}/ctrlu.h
                      ${USER_INCLUDE_DIR}/ctrlucal.h
                      ${USER_INCLUDE_DIR}/EplIdentu.h
                      ${USER_INCLUDE_DIR}/EplLedu.h
                      ${USER_INCLUDE_DIR}/EplNmtCnu.h
                      ${USER_INCLUDE_DIR}/EplNmtMnu.h
                      ${USER_INCLUDE_DIR}/EplNmtu.h
                      ${USER_INCLUDE_DIR}/EplNmtuCal.h
                      ${USER_INCLUDE_DIR}/EplObdu.h
                      ${USER_INCLUDE_DIR}/EplObduCal.h
                      ${USER_INCLUDE_DIR}/pdou.h
                      ${USER_INCLUDE_DIR}/pdoucal.h
                      ${USER_INCLUDE_DIR}/EplSdoAsndu.h
                      ${USER_INCLUDE_DIR}/EplSdoAsySequ.h
                      ${USER_INCLUDE_DIR}/EplSdoComu.h
                      ${USER_INCLUDE_DIR}/EplStatusu.h
                      ${USER_INCLUDE_DIR}/EplTimeru.h
    )

IF (CMAKE_CL_64)
    LINK_DIRECTORIES(${LIB_SOURCE_DIR}/pcap/windows/WpdPack/Lib/x64)
ELSE (CMAKE_CL_64)
    LINK_DIRECTORIES(${LIB_SOURCE_DIR}/pcap/windows/WpdPack/Lib)
ENDIF (CMAKE_CL_64)

#
# Group source files
#
SOURCE_GROUP("Source Files" FILES ${LIB_SOURCES})
SOURCE_GROUP("Source Files\\Kernel Part" FILES ${LIB_SOURCES_KERNEL})
SOURCE_GROUP("Source Files\\User Part" FILES ${LIB_SOURCES_USER})
SOURCE_GROUP("Header Files" FILES ${LIB_HEADERS})
SOURCE_GROUP("Header Files\\Kernel Part" FILES ${LIB_HEADERS_KERNEL})
SOURCE_GROUP("Header Files\\User Part" FILES ${LIB_HEADERS_USER})
SOURCE_GROUP("Object Dictionary" FILES ${LIB_OBJDICT})

#
# Source include directories
#
INCLUDE_DIRECTORIES(
    .
    ${OBJDICT_DIR}/${OBJDICT}
    ${POWERLINK_SOURCE_DIR}
)

IF (WIN32)
    INCLUDE_DIRECTORIES(${INCLUDE_DIRECTORIES}
                        ${LIB_SOURCE_DIR}/pcap/windows/WpdPack/Include
    )
ENDIF (WIN32)


#
# Library include directories
#
IF (WIN32)
    IF (CMAKE_CL_64)
        LINK_DIRECTORIES(${LIB_SOURCE_DIR}/pcap/windows/WpdPack/Lib/x64)
    ELSE (CMAKE_CL_64)
        LINK_DIRECTORIES(${LIB_SOURCE_DIR}/pcap/windows/WpdPack/Lib)
    ENDIF (CMAKE_CL_64)
ENDIF ()

#
# Define library source code 
#
IF (UNIX)
    ADD_LIBRARY(powerlink ${LIB_SOURCES} ${LIB_SOURCES_KERNEL} ${LIB_SOURCES_USER} ${LIB_HEADERS} ${LIB_HEADERS_KERNEL} ${LIB_HEADERS_USER} ${LIB_OBJDICT})

    SET_PROPERTY(TARGET powerlink
                 PROPERTY COMPILE_DEFINITIONS_DEBUG DEBUG;DEF_DEBUG_LVL=${CFG_DEBUG_LVL})   
ELSEIF (WIN32)
    ADD_LIBRARY(openPOWERLINK ${LIB_TYPE} ${LIB_SOURCES} ${LIB_SOURCES_KERNEL} ${LIB_SOURCES_USER} ${LIB_HEADERS} ${LIB_HEADERS_KERNEL} ${LIB_HEADERS_USER} ${LIB_OBJDICT})

    TARGET_LINK_LIBRARIES(openPOWERLINK wpcap iphlpapi)

    SET_PROPERTY(TARGET openPOWERLINK
                 PROPERTY COMPILE_DEFINITIONS_DEBUG DEBUG;DEF_DEBUG_LVL=${CFG_DEBUG_LVL})
ENDIF ()

#
# Installation
#
IF (WIN32)
    IF (CFG_X86_WINDOWS_DLL)
        INSTALL(TARGETS openPOWERLINK RUNTIME DESTINATION bin)
    ENDIF (CFG_X86_WINDOWS_DLL)
ENDIF ()
