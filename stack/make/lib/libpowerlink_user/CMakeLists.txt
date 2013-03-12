################################################################################
#
# CMake file of openPOWERLINK user stack library
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
    SET(LIB_SOURCES
        ${LIB_SOURCES}
        ${USER_SOURCE_DIR}/obd/obdcdc.c
        )

    SET (LIB_USER_SOURCES
        ${LIB_USER_SOURCES}
        ${USER_SOURCE_DIR}/cfmu.c
        )

    SET (LIB_USER_HEADERS
        ${LIB_USER_HEADERS}
        ${USER_INCLUDE_DIR}/EplCfmu.h
        )

    ADD_DEFINITIONS(-DCONFIG_CFM -DCONFIG_OPENCONFIGURATOR_MAPPING -DCONFIG_MN)
ENDIF(CFG_POWERLINK_MN)

#
# setup object dictionary to use
#
SET (LIB_OBJDICT
     ${LIB_OBJDICT}
     ${OBJDICT_DIR}/${OBJDICT}/Objdict.c
     ${OBJDICT_DIR}/${OBJDICT}/objdict.h
     ${OBJDICT_DIR}/${OBJDICT}/obdcfg.h
     ${OBJDICT_DIR}/generic/objdict_1000-13ff.h
     ${OBJDICT_DIR}/generic/objdict_1b00-1fff.h
     )

#
# set general sources of POWERLINK library
#
SET (LIB_SOURCES
     ${LIB_SOURCES}
     ${USER_SOURCE_DIR}/api/generic.c
     ${USER_SOURCE_DIR}/api/processimage.c
     ${COMMON_SOURCE_DIR}/debug.c
     ${USER_SOURCE_DIR}/obd/obd.c
     ${COMMON_SOURCE_DIR}/dll/dllcal-shb.c
     ${COMMON_SOURCE_DIR}/ctrl/ctrlcal-shb.c
     ${COMMON_SOURCE_DIR}/event/event.c
     ${COMMON_SOURCE_DIR}/event/eventcal-shb.c
     ${LIB_SOURCE_DIR}/ami/amix86.c
     ${LIB_SOURCE_DIR}/sharedbuff/SharedBuff.c
     )

#
# set user part sources of POWERLINK library
#
SET (LIB_USER_SOURCES
     ${LIB_USER_SOURCES}
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
     ${USER_SOURCE_DIR}/sdo/sdo-comu.c
     ${USER_SOURCE_DIR}/sdo/sdo-asysequ.c
     ${USER_SOURCE_DIR}/sdo/sdo-asndu.c
     ${USER_SOURCE_DIR}/errhnd/errhndu.c
     ${USER_SOURCE_DIR}/errhnd/errhnducal-shb.c
     ${USER_SOURCE_DIR}/ctrl/ctrlu.c
     ${USER_SOURCE_DIR}/ctrl/ctrlucal-mem.c
     )

#
# set general header files of POWERLINK library
#
SET (LIB_HEADERS
     ${LIB_HEADERS}
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
# set user part header files of POWERLINK library
#
SET (LIB_USER_HEADERS
     ${LIB_USER_HEADERS}
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

IF (UNIX)
    include(linux.cmake)
ELSEIF (WIN32)
    include (windows.cmake)
ENDIF (UNIX)

#
# Group source files
#
SOURCE_GROUP("Source Files" FILES ${LIB_SOURCES})
SOURCE_GROUP("Source Files\\Architecture Source Files" FILES ${LIB_ARCH_SOURCES})
SOURCE_GROUP("Source Files\\User Source Files" FILES ${LIB_USER_SOURCES})
SOURCE_GROUP("Header Files" FILES ${LIB_HEADERS})
SOURCE_GROUP("Header Files\\User Header Files" FILES ${LIB_USER_HEADERS})
SOURCE_GROUP("Object Dictionary" FILES ${LIB_OBJDICT})

#
# Source include directories
#
INCLUDE_DIRECTORIES(
    .
    ${OBJDICT_DIR}/${OBJDICT}
    ${POWERLINK_SOURCE_DIR}
)

#
# Define library source code
#
ADD_LIBRARY(powerlink_user ${LIB_TYPE} ${LIB_SOURCES} ${LIB_ARCH_SOURCES}
                           ${LIB_USER_SOURCES} ${LIB_HEADERS} ${LIB_USER_HEADERS}
                           ${LIB_OBJDICT})

TARGET_LINK_LIBRARIES(powerlink_user ${ARCH_LIBRARIES})

SET_PROPERTY(TARGET powerlink_user
                 PROPERTY COMPILE_DEFINITIONS_DEBUG DEBUG;DEF_DEBUG_LVL=${CFG_DEBUG_LVL})

