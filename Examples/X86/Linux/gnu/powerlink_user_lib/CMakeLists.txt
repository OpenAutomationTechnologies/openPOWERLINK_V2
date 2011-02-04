################################################################################
#
# Project: openPOWERLINK
#
# (c) Bernecker + Rainer Industrie-Elektronik Ges.m.b.H.
#     B&R Strasse 1, A-5142 Eggelsberg
#     www.br-automation.com
#
# Description: cmake file of linux userspace openPOWERLINK library
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
ADD_DEFINITIONS(-Wall -Wextra -pedantic -std=c99 -pthread -D_GNU_SOURCE
                -D_POSIX_C_SOURCE=200112L)

#
# Configure depending selected mode
#
IF(CFG_POWERLINK_CFM)

    IF(CFG_POWERLINK_PROCESS_IMAGE)

        SET(OBJDICT CiA302-4_MN)
        ADD_DEFINITIONS(-DCONFIG_CFM -DCONFIG_OPENCONFIGURATOR_MAPPING)
        SET(LIB_SOURCES ${LIB_SOURCES}
                        ${POWERLINK_SOURCE_DIR}/EplStack/EplObdCdc.c
                        ${POWERLINK_SOURCE_DIR}/EplStack/EplCfmu.c)

    ELSE(CFG_POWERLINK_PROCESS_IMAGE)

        SET(OBJDICT Qt_MN)
        ADD_DEFINITIONS(-DCONFIG_CFM)
        SET(LIB_SOURCES ${LIB_SOURCES}
              ${POWERLINK_SOURCE_DIR}/EplStack/EplObdCdc.c
              ${POWERLINK_SOURCE_DIR}/EplStack/EplCfmu.c)

    ENDIF(CFG_POWERLINK_PROCESS_IMAGE)

ELSE(CFG_POWERLINK_CFM)

    SET(OBJDICT Qt_MN)

ENDIF(CFG_POWERLINK_CFM)

#
# setup ethernet driver to use
#
SET(LIB_SOURCES ${LIB_SOURCES}
    ${POWERLINK_SOURCE_DIR}/Edrv/EdrvPcapLinux.c
    )

#
# set sources of POWERLINK library
#
SET (LIB_SOURCES ${LIB_SOURCES}
	${POWERLINK_SOURCE_DIR}/EplStack/EplApiGeneric.c
	${POWERLINK_SOURCE_DIR}/EplStack/EplDebug.c
	${POWERLINK_SOURCE_DIR}/EplStack/EplDllk.c
	${POWERLINK_SOURCE_DIR}/EplStack/EplDllkCal.c
	${POWERLINK_SOURCE_DIR}/EplStack/EplDlluCal.c
	${POWERLINK_SOURCE_DIR}/EplStack/EplEventk.c
	${POWERLINK_SOURCE_DIR}/EplStack/EplEventu.c
	${POWERLINK_SOURCE_DIR}/EplStack/EplNmtk.c
	${POWERLINK_SOURCE_DIR}/EplStack/EplNmtkCal.c
	${POWERLINK_SOURCE_DIR}/EplStack/EplNmtu.c
	${POWERLINK_SOURCE_DIR}/EplStack/EplNmtuCal.c
	${POWERLINK_SOURCE_DIR}/EplStack/EplNmtCnu.c
	${POWERLINK_SOURCE_DIR}/EplStack/EplNmtMnu.c
	${POWERLINK_SOURCE_DIR}/EplStack/EplIdentu.c
	${POWERLINK_SOURCE_DIR}/EplStack/EplStatusu.c
	${POWERLINK_SOURCE_DIR}/EplStack/EplSyncu.c
	${POWERLINK_SOURCE_DIR}/EplStack/EplPdok.c
	${POWERLINK_SOURCE_DIR}/EplStack/EplPdokCal.c
	${POWERLINK_SOURCE_DIR}/EplStack/EplPdou.c
	${POWERLINK_SOURCE_DIR}/EplStack/EplPdouCal.c
	${POWERLINK_SOURCE_DIR}/EplStack/EplObd.c
	${POWERLINK_SOURCE_DIR}/EplStack/EplSdoUdpu.c
	${POWERLINK_SOURCE_DIR}/EplStack/EplSdoComu.c
	${POWERLINK_SOURCE_DIR}/EplStack/EplSdoAsySequ.c
	${POWERLINK_SOURCE_DIR}/EplStack/EplSdoAsndu.c
	${POWERLINK_SOURCE_DIR}/EplStack/EplTimeruLinuxUser.c
	${POWERLINK_SOURCE_DIR}/EplStack/EplErrorHandlerk.c
	${POWERLINK_SOURCE_DIR}/EplStack/EplApiProcessImage.c
    ${POWERLINK_SOURCE_DIR}/EplStack/EplTimerHighReskPosix.c
	${POWERLINK_SOURCE_DIR}/ObjDicts/${OBJDICT}/Objdict.c
	${POWERLINK_SOURCE_DIR}/EplStack/amix86.c
	${POWERLINK_SOURCE_DIR}/SharedBuff/SharedBuff.c
	${POWERLINK_SOURCE_DIR}/SharedBuff/ShbIpc-LinuxPthreads.c
	${POWERLINK_SOURCE_DIR}/Target/X86/Linux/gnu/generic/ftraceDebug.c
	${POWERLINK_SOURCE_DIR}/Target/X86/Linux/gnu/generic/EplTgtInitPosix.c
	${POWERLINK_SOURCE_DIR}/Edrv/EdrvCyclic.c
	)

INCLUDE_DIRECTORIES(
    .
    ${POWERLINK_SOURCE_DIR}/ObjDicts/${OBJDICT}
    )

ADD_LIBRARY(powerlink ${LIB_SOURCES})

SET_PROPERTY(TARGET powerlink
             PROPERTY COMPILE_DEFINITIONS_DEBUG DEBUG;DEF_DEBUG_LVL=${CFG_DEBUG_LVL})
