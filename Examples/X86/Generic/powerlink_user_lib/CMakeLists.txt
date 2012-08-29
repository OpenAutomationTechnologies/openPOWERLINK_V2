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
                    ${POWERLINK_SOURCE_DIR}/EplStack/EplObdCdc.c
        )
    SET (LIB_SOURCES_USER ${LIB_SOURCES_USER}
                          ${POWERLINK_SOURCE_DIR}/EplStack/EplCfmu.c
        )
    SET (LIB_HEADERS_USER ${LIB_HEADERS_USER}
                          ${POWERLINK_SOURCE_DIR}/Include/user/EplCfmu.h
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
                    ${POWERLINK_SOURCE_DIR}/Edrv/EdrvPcapLinux.c
        )
ELSEIF (WIN32)
    SET(LIB_SOURCES ${LIB_SOURCES}
                    ${POWERLINK_SOURCE_DIR}/Edrv/EdrvPcap.c
        )
ENDIF ()

#
# setup object dictionary to use
#
SET(LIB_OBJDICT ${LIB_OBJDICT}
                ${POWERLINK_SOURCE_DIR}/ObjDicts/${OBJDICT}/Objdict.c
                ${POWERLINK_SOURCE_DIR}/ObjDicts/${OBJDICT}/objdict.h
                ${POWERLINK_SOURCE_DIR}/ObjDicts/${OBJDICT}/obdcfg.h
                ${POWERLINK_SOURCE_DIR}/ObjDicts/Generic/objdict_1000-13ff.h
                ${POWERLINK_SOURCE_DIR}/ObjDicts/Generic/objdict_1b00-1fff.h
    )

#
# set general sources of POWERLINK library
#
SET (LIB_SOURCES ${LIB_SOURCES}
                 ${POWERLINK_SOURCE_DIR}/EplStack/EplApiGeneric.c
                 ${POWERLINK_SOURCE_DIR}/EplStack/EplDebug.c
                 ${POWERLINK_SOURCE_DIR}/EplStack/EplObd.c
                 ${POWERLINK_SOURCE_DIR}/EplStack/EplApiProcessImage.c
                 ${POWERLINK_SOURCE_DIR}/Edrv/EdrvCyclic.c
                 ${POWERLINK_SOURCE_DIR}/EplStack/amix86.c
                 ${POWERLINK_SOURCE_DIR}/SharedBuff/SharedBuff.c
                 ${POWERLINK_SOURCE_DIR}/Edrv/EdrvCyclic.c
    )

IF (UNIX)
    SET (LIB_SOURCES ${LIB_SOURCES}
                     ${POWERLINK_SOURCE_DIR}/EplStack/EplSdoUdpu.c
                     ${POWERLINK_SOURCE_DIR}/EplStack/EplTimeruLinuxUser.c
                     ${POWERLINK_SOURCE_DIR}/EplStack/EplTimerHighReskPosix.c
                     ${POWERLINK_SOURCE_DIR}/SharedBuff/ShbIpc-LinuxPthreads.c
                     ${POWERLINK_SOURCE_DIR}/Target/X86/Linux/gnu/generic/ftraceDebug.c
                     ${POWERLINK_SOURCE_DIR}/Target/X86/Linux/gnu/generic/EplTgtInitPosix.c
        )
ELSEIF (WIN32)
    SET (LIB_SOURCES ${LIB_SOURCES}
                     ${POWERLINK_SOURCE_DIR}/EplStack/EplTimeruGeneric.c
                     ${POWERLINK_SOURCE_DIR}/SharedBuff/ShbIpc-Win32.c
                     ${POWERLINK_SOURCE_DIR}/EplStack/trace.c
        )
ENDIF ()

#
# set kernel part sources of POWERLINK library
#
SET (LIB_SOURCES_KERNEL ${LIB_SOURCES_KERNEL}
                        ${POWERLINK_SOURCE_DIR}/EplStack/EplDllk.c
                        ${POWERLINK_SOURCE_DIR}/EplStack/EplDllkCal.c
                        ${POWERLINK_SOURCE_DIR}/EplStack/EplEventk.c
                        ${POWERLINK_SOURCE_DIR}/EplStack/EplNmtk.c
                        ${POWERLINK_SOURCE_DIR}/EplStack/EplNmtkCal.c
                        ${POWERLINK_SOURCE_DIR}/EplStack/EplPdok.c
                        ${POWERLINK_SOURCE_DIR}/EplStack/EplPdokCal.c
                        ${POWERLINK_SOURCE_DIR}/EplStack/EplErrorHandlerk.c
    )

#
# set user part sources of POWERLINK library
#
SET (LIB_SOURCES_USER ${LIB_SOURCES_USER}
                      ${POWERLINK_SOURCE_DIR}/EplStack/EplDlluCal.c
                      ${POWERLINK_SOURCE_DIR}/EplStack/EplEventu.c
                      ${POWERLINK_SOURCE_DIR}/EplStack/EplNmtu.c
                      ${POWERLINK_SOURCE_DIR}/EplStack/EplNmtuCal.c
                      ${POWERLINK_SOURCE_DIR}/EplStack/EplNmtCnu.c
                      ${POWERLINK_SOURCE_DIR}/EplStack/EplNmtMnu.c
                      ${POWERLINK_SOURCE_DIR}/EplStack/EplIdentu.c
                      ${POWERLINK_SOURCE_DIR}/EplStack/EplStatusu.c
                      ${POWERLINK_SOURCE_DIR}/EplStack/EplSyncu.c
                      ${POWERLINK_SOURCE_DIR}/EplStack/EplPdou.c
                      ${POWERLINK_SOURCE_DIR}/EplStack/EplPdouCal.c
                      ${POWERLINK_SOURCE_DIR}/EplStack/EplSdoComu.c
                      ${POWERLINK_SOURCE_DIR}/EplStack/EplSdoAsySequ.c
                      ${POWERLINK_SOURCE_DIR}/EplStack/EplSdoAsndu.c
    )

#
# set general header files of POWERLINK library
#
SET (LIB_HEADERS ${LIB_HEADERS}
                 ./EplCfg.h
                 ${POWERLINK_SOURCE_DIR}/Include/Benchmark.h
                 ${POWERLINK_SOURCE_DIR}/Include/Debug.h
                 ${POWERLINK_SOURCE_DIR}/Include/edrv.h
                 ${POWERLINK_SOURCE_DIR}/Include/Epl.h
                 ${POWERLINK_SOURCE_DIR}/Include/EplAmi.h
                 ${POWERLINK_SOURCE_DIR}/Include/EplCfm.h
                 ${POWERLINK_SOURCE_DIR}/Include/EplDef.h
                 ${POWERLINK_SOURCE_DIR}/Include/EplDll.h
                 ${POWERLINK_SOURCE_DIR}/Include/EplDllCal.h
                 ${POWERLINK_SOURCE_DIR}/Include/EplErrDef.h
                 ${POWERLINK_SOURCE_DIR}/Include/EplEvent.h
                 ${POWERLINK_SOURCE_DIR}/Include/EplFrame.h
                 ${POWERLINK_SOURCE_DIR}/Include/EplInc.h
                 ${POWERLINK_SOURCE_DIR}/Include/EplInstDef.h
                 ${POWERLINK_SOURCE_DIR}/Include/EplNmt.h
                 ${POWERLINK_SOURCE_DIR}/Include/EplObd.h
                 ${POWERLINK_SOURCE_DIR}/Include/EplObdMacro.h
                 ${POWERLINK_SOURCE_DIR}/Include/EplObjDef.h
                 ${POWERLINK_SOURCE_DIR}/Include/EplPdo.h
                 ${POWERLINK_SOURCE_DIR}/Include/EplSdo.h
                 ${POWERLINK_SOURCE_DIR}/Include/EplSdoAc.h
                 ${POWERLINK_SOURCE_DIR}/Include/EplTarget.h
                 ${POWERLINK_SOURCE_DIR}/Include/EplTimer.h
                 ${POWERLINK_SOURCE_DIR}/Include/EplVersion.h
                 ${POWERLINK_SOURCE_DIR}/Include/global.h
                 ${POWERLINK_SOURCE_DIR}/SharedBuff/SharedBuff.h
                 ${POWERLINK_SOURCE_DIR}/SharedBuff/ShbIpc.h
    )

#
# set kernel part header files of POWERLINK library
#
SET (LIB_HEADERS_KERNEL ${LIB_HEADERS_KERNEL}
                        ${POWERLINK_SOURCE_DIR}/Include/kernel/EplDllk.h
                        ${POWERLINK_SOURCE_DIR}/Include/kernel/EplDllkCal.h
                        ${POWERLINK_SOURCE_DIR}/Include/kernel/EplDllkTgt.h
                        ${POWERLINK_SOURCE_DIR}/Include/kernel/EplErrorHandlerk.h
                        ${POWERLINK_SOURCE_DIR}/Include/kernel/EplEventk.h
                        ${POWERLINK_SOURCE_DIR}/Include/kernel/EplNmtk.h
                        ${POWERLINK_SOURCE_DIR}/Include/kernel/EplNmtkCal.h
                        ${POWERLINK_SOURCE_DIR}/Include/kernel/EplObdk.h
                        ${POWERLINK_SOURCE_DIR}/Include/kernel/EplObdkCal.h
                        ${POWERLINK_SOURCE_DIR}/Include/kernel/EplPdok.h
                        ${POWERLINK_SOURCE_DIR}/Include/kernel/EplPdokCal.h
                        ${POWERLINK_SOURCE_DIR}/Include/kernel/EplTimerHighResk.h
                        ${POWERLINK_SOURCE_DIR}/Include/kernel/EplTimerk.h
    )

#
# set user part header files of POWERLINK library
#
SET (LIB_HEADERS_USER ${LIB_HEADERS_USER}
                      ${POWERLINK_SOURCE_DIR}/Include/user/EplDllu.h
                      ${POWERLINK_SOURCE_DIR}/Include/user/EplDlluCal.h
                      ${POWERLINK_SOURCE_DIR}/Include/user/EplEventu.h
                      ${POWERLINK_SOURCE_DIR}/Include/user/EplIdentu.h
                      ${POWERLINK_SOURCE_DIR}/Include/user/EplLedu.h
                      ${POWERLINK_SOURCE_DIR}/Include/user/EplNmtCnu.h
                      ${POWERLINK_SOURCE_DIR}/Include/user/EplNmtMnu.h
                      ${POWERLINK_SOURCE_DIR}/Include/user/EplNmtu.h
                      ${POWERLINK_SOURCE_DIR}/Include/user/EplNmtuCal.h
                      ${POWERLINK_SOURCE_DIR}/Include/user/EplObdu.h
                      ${POWERLINK_SOURCE_DIR}/Include/user/EplObduCal.h
                      ${POWERLINK_SOURCE_DIR}/Include/user/EplPdou.h
                      ${POWERLINK_SOURCE_DIR}/Include/user/EplPdouCal.h
                      ${POWERLINK_SOURCE_DIR}/Include/user/EplSdoAsndu.h
                      ${POWERLINK_SOURCE_DIR}/Include/user/EplSdoAsySequ.h
                      ${POWERLINK_SOURCE_DIR}/Include/user/EplSdoComu.h
                      ${POWERLINK_SOURCE_DIR}/Include/user/EplStatusu.h
                      ${POWERLINK_SOURCE_DIR}/Include/user/EplTimeru.h
    )

IF (CMAKE_CL_64)
    LINK_DIRECTORIES(${POWERLINK_SOURCE_DIR}/Target/X86/Windows/WpdPack/Lib/x64)
ELSE (CMAKE_CL_64)
    LINK_DIRECTORIES(${POWERLINK_SOURCE_DIR}/Target/X86/Windows/WpdPack/Lib)
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
    ${POWERLINK_SOURCE_DIR}/ObjDicts/${OBJDICT}
)

IF (WIN32)
    INCLUDE_DIRECTORIES(${INCLUDE_DIRECTORIES}
                        ${POWERLINK_SOURCE_DIR}/Target/X86/Windows/WpdPack/Include
    )
ENDIF (WIN32)


#
# Library include directories
#
IF (WIN32)
    IF (CMAKE_CL_64)
        LINK_DIRECTORIES(${POWERLINK_SOURCE_DIR}/Target/X86/Windows/WpdPack/Lib/x64)
    ELSE (CMAKE_CL_64)
        LINK_DIRECTORIES(${POWERLINK_SOURCE_DIR}/Target/X86/Windows/WpdPack/Lib)
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
