################################################################################
#
# Project: openPOWERLINK
#
# (c) Bernecker + Rainer Industrie-Elektronik Ges.m.b.H.
#     B&R Strasse 1, A-5142 Eggelsberg
#     www.br-automation.com
#
# Description: cmake file for linux kernel demo application
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
#   3. Neither the name of copyright holders nor the names of its
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
# Configure depending selected ethernet driver
#
IF(CFG_POWERLINK_EDRV STREQUAL "8139")

    SET(MODULE_NAME "demo_powerlink8139")
    SET(MODULE_DEFS "${MODULE_DEFS} -DCONFIG_EDRV=8139")
    SET(MODULE_SOURCE_FILES ${MODULE_SOURCE_FILES} ${POWERLINK_SOURCE_DIR}/Edrv/Edrv8139.c)

ELSEIF(CFG_POWERLINK_EDRV STREQUAL "82573")

    SET(MODULE_NAME "demo_powerlink82573")
    SET(MODULE_DEFS "${MODULE_DEFS} -DCONFIG_EDRV=82573")
    SET(MODULE_SOURCE_FILES ${MODULE_SOURCE_FILES} ${POWERLINK_SOURCE_DIR}/Edrv/Edrv82573.c)

ELSEIF(CFG_POWERLINK_EDRV STREQUAL "Fec")

    SET(MODULE_NAME "demo_powerlinkfec")
    SET(MODULE_DEFS "${MODULE_DEFS} -DCONFIG_EDRV=0")
    SET(MODULE_SOURCE_FILES ${MODULE_SOURCE_FILES} ${POWERLINK_SOURCE_DIR}/Edrv/EdrvFec.c)

ELSE(CFG_POWERLINK_EDRV STREQUAL "8139")

    message(FATAL_ERROR
            "No valid ethernet driver (8139, 82573, Fec) was specified in CFG_POWERLINK_EDRV")

ENDIF(CFG_POWERLINK_EDRV STREQUAL "8139")

#
# Configure depending selected mode
#
IF(CFG_POWERLINK_CFM)

    SET(MODULE_SOURCE_FILES ${MODULE_SOURCE_FILES}
                            ${POWERLINK_SOURCE_DIR}/EplStack/EplObdCdc.c
                            ${POWERLINK_SOURCE_DIR}/EplStack/EplCfmu.c)
    SET(MODULE_DEFS "${MODULE_DEFS} -DCONFIG_CFM")

    IF (CFG_POWERLINK_PROCESS_IMAGE)

        #MESSAGE(FATAL_ERROR "Process image demo not available as kernel demo!")
        SET(OBJDICT CiA302-4_MN)
        SET(MODULE_DEFS "${MODULE_DEFS} -DCONFIG_OPENCONFIGURATOR_MAPPING")
        SET(MODULE_SOURCE_FILES ${MODULE_SOURCE_FILES}
                                ${CMAKE_CURRENT_SOURCE_DIR}/demo_main.c
                                ${POWERLINK_SOURCE_DIR}/ObjDicts/CiA302-4_MN/EplApiProcessImageSetup.c
                                )

    ELSE(CFG_POWERLINK_PROCESS_IMAGE)

        SET(OBJDICT Qt_MN)
        SET(MODULE_SOURCE_FILES ${MODULE_SOURCE_FILES}
                                ${CMAKE_CURRENT_SOURCE_DIR}/demo_main.c
                                )
    ENDIF(CFG_POWERLINK_PROCESS_IMAGE)

ELSE(CFG_POWERLINK_CFM)

    SET(OBJDICT Qt_MN)
    SET(MODULE_SOURCE_FILES ${MODULE_SOURCE_FILES}
                            ${CMAKE_CURRENT_SOURCE_DIR}/demo_main.c)
ENDIF(CFG_POWERLINK_CFM)

#
# specifiy standard definitions
#
IF(CMAKE_BUILD_TYPE STREQUAL Debug OR CMAKE_BUILD_TYPE STREQUAL DEBUG)
    SET(MODULE_DEFS "${MODULE_DEFS} -D_DEBUG")
    SET(MODULE_DEFS "${MODULE_DEFS} -D_DBG_TRACE_POINTS_")
    SET(MODULE_DEFS "${MODULE_DEFS} -DDEF_DEBUG_LVL=${CFG_DEBUG_LVL}")
ENDIF(CMAKE_BUILD_TYPE STREQUAL Debug OR CMAKE_BUILD_TYPE STREQUAL DEBUG)

#
# specifiy standard include directories
#
SET(MODULE_INCLUDES "${MODULE_INCLUDES} -I${CMAKE_CURRENT_SOURCE_DIR}")
SET(MODULE_INCLUDES "${MODULE_INCLUDES} -I${POWERLINK_INCLUDE_DIR}")
SET(MODULE_INCLUDES "${MODULE_INCLUDES} -I${POWERLINK_SOURCE_DIR}/SharedBuff")
SET(MODULE_INCLUDES "${MODULE_INCLUDES} -I${POWERLINK_SOURCE_DIR}/ObjDicts/${OBJDICT}")
SET(MODULE_INCLUDES "${MODULE_INCLUDES} -I${POWERLINK_SOURCE_DIR}/Target/PLCcore-CF54/Linux/gnu/generic")

#
# specifiy source files
#
SET(MODULE_SOURCE_FILES
    ${MODULE_SOURCE_FILES}
    ${POWERLINK_SOURCE_DIR}/Edrv/EdrvCyclic.c
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
    ${POWERLINK_SOURCE_DIR}/EplStack/EplPdok.c
    ${POWERLINK_SOURCE_DIR}/EplStack/EplPdokCal.c
    ${POWERLINK_SOURCE_DIR}/EplStack/EplPdou.c
    ${POWERLINK_SOURCE_DIR}/EplStack/EplPdouCal.c
    ${POWERLINK_SOURCE_DIR}/EplStack/EplObd.c
    ${POWERLINK_SOURCE_DIR}/EplStack/EplSdoUdpu.c
    ${POWERLINK_SOURCE_DIR}/EplStack/EplSdoComu.c
    ${POWERLINK_SOURCE_DIR}/EplStack/EplSdoAsySequ.c
    ${POWERLINK_SOURCE_DIR}/EplStack/EplSdoAsndu.c
    ${POWERLINK_SOURCE_DIR}/EplStack/EplSyncu.c
    ${POWERLINK_SOURCE_DIR}/EplStack/EplTimeruLinuxKernel.c
    ${POWERLINK_SOURCE_DIR}/EplStack/EplErrorHandlerk.c
    ${POWERLINK_SOURCE_DIR}/EplStack/EplApiGeneric.c
    ${POWERLINK_SOURCE_DIR}/EplStack/EplApiProcessImage.c
    ${POWERLINK_SOURCE_DIR}/EplStack/VirtualEthernetLinux.c
    ${POWERLINK_SOURCE_DIR}/EplStack/SocketLinuxKernel.c
    ${POWERLINK_SOURCE_DIR}/EplStack/PosixFileLinuxKernel.c
    ${POWERLINK_SOURCE_DIR}/SharedBuff/SharedBuff.c
    ${POWERLINK_SOURCE_DIR}/SharedBuff/ShbIpc-LinuxKernel.c
    ${POWERLINK_SOURCE_DIR}/Target/X86/Linux/gnu/generic/TimerHighReskX86.c
    ${POWERLINK_SOURCE_DIR}/Target/PLCcore-CF54/Linux/gnu/generic/proc_fs.c
    ${POWERLINK_SOURCE_DIR}/ObjDicts/${OBJDICT}/Objdict.c
)

IF (CMAKE_SYSTEM_PROCESSOR STREQUAL x86)
    SET(MODULE_SOURCE_FILES ${MODULE_SOURCE_FILES} ${POWERLINK_SOURCE_DIR}/EplStack/amix86.c)

ELSEIF (   CMAKE_SYSTEM_PROCESSOR STREQUAL arm
        OR CMAKE_SYSTEM_PROCESSOR STREQUAL armv7l)
    SET(MODULE_SOURCE_FILES ${MODULE_SOURCE_FILES} ${POWERLINK_SOURCE_DIR}/EplStack/amiarm.c)

ELSEIF (   CMAKE_SYSTEM_PROCESSOR STREQUAL i686
	OR CMAKE_SYSTEM_PROCESSOR STREQUAL x86_64)
    SET(MODULE_SOURCE_FILES ${MODULE_SOURCE_FILES} ${POWERLINK_SOURCE_DIR}/EplStack/amix86.c)

ELSE (CMAKE_SYSTEM_PROCESSOR STREQUAL x86)
    MESSAGE(FATAL_ERROR
            "ami: CMAKE_SYSTEM_PROCESSOR is set to ${CMAKE_SYSTEM_PROCESSOR}. Valid targets are (x86, x86_64, arm, armv7l, i686).")
ENDIF (CMAKE_SYSTEM_PROCESSOR STREQUAL x86)

SET(MODULE_FILE
    ${MODULE_NAME}.ko
)

SET(MODULE_OUTPUT_DIR
    ${CMAKE_CURRENT_BINARY_DIR}
)

IF (CFG_KERNEL_DIR STREQUAL "")
    SET(KERNEL_DIR "/lib/modules/${CMAKE_SYSTEM_VERSION}/build")
ELSE(CFG_KERNEL_DIR STREQUAL "")
    SET(KERNEL_DIR ${CFG_KERNEL_DIR})
ENDIF (CFG_KERNEL_DIR STREQUAL "")

FOREACH(MODULE_SOURCE_FILE ${MODULE_SOURCE_FILES})
    GET_FILENAME_COMPONENT(MODULE_SOURCE_FILENAME ${MODULE_SOURCE_FILE} NAME)
    GET_FILENAME_COMPONENT(MODULE_SOURCE_BASENAME ${MODULE_SOURCE_FILE} NAME_WE)
    CONFIGURE_FILE(${MODULE_SOURCE_FILE} ${MODULE_OUTPUT_DIR}/src/${MODULE_SOURCE_FILENAME} COPYONLY)
    SET(MODULE_OBJS "${MODULE_OBJS} src/${MODULE_SOURCE_BASENAME}.o")
ENDFOREACH(MODULE_SOURCE_FILE)

CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/../tools/Kbuild.in ${MODULE_OUTPUT_DIR}/Kbuild)

IF(CMAKE_CROSSCOMPILING)
    IF(DEFINED MAKE_KERNEL_ARCH)
        SET(MAKE_KERNEL_CROSSCOMPILING_PARAMS ARCH=${MAKE_KERNEL_ARCH})
    ENDIF(DEFINED MAKE_KERNEL_ARCH)

    IF(DEFINED MAKE_KERNEL_CROSS_COMPILE)
        SET(MAKE_KERNEL_CROSSCOMPILING_PARAMS
                ${MAKE_KERNEL_CROSSCOMPILING_PARAMS} CROSS_COMPILE=${MAKE_KERNEL_CROSS_COMPILE})
    ENDIF(DEFINED MAKE_KERNEL_CROSS_COMPILE)
ENDIF(CMAKE_CROSSCOMPILING)

ADD_CUSTOM_COMMAND(
    OUTPUT ${MODULE_OUTPUT_DIR}/${MODULE_FILE}
    COMMAND ${CMAKE_MAKE_PROGRAM} ${MAKE_KERNEL_CROSSCOMPILING_PARAMS} -C ${KERNEL_DIR} M=${MODULE_OUTPUT_DIR} modules
    WORKING_DIRECTORY ${MODULE_OUTPUT_DIR}
    DEPENDS ${MODULE_SOURCE_FILES} ${CMAKE_CURRENT_SOURCE_DIR}/../tools/Kbuild.in
    VERBATIM
)

ADD_CUSTOM_TARGET(
    ${MODULE_NAME}
    ALL
    DEPENDS ${MODULE_OUTPUT_DIR}/${MODULE_FILE}
    )

ADD_CUSTOM_TARGET(
    demo_module_clean
    COMMAND ${CMAKE_MAKE_PROGRAM} -C ${KERNEL_DIR} M=${MODULE_OUTPUT_DIR} clean
    #COMMAND ${CMAKE_COMMAND} -E remove_directory ${MODULE_OUTPUT_DIR}/src
    )

# add installation rules
INSTALL(FILES ${MODULE_OUTPUT_DIR}/${MODULE_NAME}.ko DESTINATION bin)

IF(CFG_POWERLINK_CFM)
    IF (CFG_POWERLINK_PROCESS_IMAGE)
        INSTALL(FILES mnobd_pi.cdc DESTINATION bin RENAME mnobd.cdc)
    ELSE (CFG_POWERLINK_PROCESS_IMAGE)
        INSTALL(FILES mnobd.cdc DESTINATION bin)
    ENDIF (CFG_POWERLINK_PROCESS_IMAGE)
ENDIF(CFG_POWERLINK_CFM)
INSTALL(PROGRAMS ${CMAKE_CURRENT_SOURCE_DIR}/../tools/EplLoad DESTINATION bin)
INSTALL(PROGRAMS ${CMAKE_CURRENT_SOURCE_DIR}/../tools/EplUnload DESTINATION bin)
