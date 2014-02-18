################################################################################
#
# CMake file for openPOWERLINK kernel stack (Linux kernel module)
#
# Copyright (c) 2013, SYSTEC electronik GmbH
# Copyright (c) 2014, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
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

PROJECT (oplkdrv_kernelmodule_edrv C)

IF(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    CMAKE_MINIMUM_REQUIRED (VERSION 2.8.0)
ELSE()
    MESSAGE(FATAL_ERROR "Unsupported system ${CMAKE_SYSTEM_NAME} for this project!")
ENDIF()

MESSAGE(STATUS "CMAKE_SYSTEM_NAME is ${CMAKE_SYSTEM_NAME}")
MESSAGE(STATUS "CMAKE_SYSTEM_PROCESSOR is ${CMAKE_SYSTEM_PROCESSOR}")

STRING(TOLOWER "${CMAKE_SYSTEM_NAME}" SYSTEM_NAME_DIR)
STRING(TOLOWER "${CMAKE_SYSTEM_PROCESSOR}" SYSTEM_PROCESSOR_DIR)

################################################################################
# Configuration options

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release CACHE STRING
      "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
      FORCE)
ENDIF()

OPTION (CFG_OPLK_MN "Compile openPOWERLINK MN (Otherwise CN)" ON)

SET(CFG_DEBUG_LVL "0xEC000000L" CACHE STRING "Debug Level for debug output")

SET(CFG_KERNEL_DIR "" CACHE PATH
        "Select the kernel directory to be used, if not specified, system kernel dir will be used!")

SET(CFG_POWERLINK_EDRV "82573" CACHE STRING
        "Valid drivers are 8139, 82573, 8255x, Fec")
SET_PROPERTY(CACHE CFG_POWERLINK_EDRV PROPERTY STRINGS 8139 82573 8255x Fec)

################################################################################
# Set global directories
################################################################################
SET(OPLK_ROOT_DIR ${CMAKE_SOURCE_DIR}/../../..)
SET(CONTRIB_SOURCE_DIR ${OPLK_ROOT_DIR}/contrib)
SET(STACK_SOURCE_DIR ${OPLK_ROOT_DIR}/stack/src)
SET(COMMON_SOURCE_DIR ${OPLK_ROOT_DIR}/stack/src/common)
SET(KERNEL_SOURCE_DIR ${OPLK_ROOT_DIR}/stack/src/kernel)
SET(EDRV_SOURCE_DIR ${OPLK_ROOT_DIR}/stack/src/kernel/edrv)
SET(OPLK_INCLUDE_DIR ${OPLK_ROOT_DIR}/stack/include)
SET(TOOLS_DIR ${OPLK_ROOT_DIR}/tools)

IF(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
  SET(CMAKE_INSTALL_PREFIX
    ${OPLK_ROOT_DIR}/bin/${SYSTEM_NAME_DIR}/${SYSTEM_PROCESSOR_DIR}/
    CACHE PATH "openPOWERLINK kernel driver install prefix" FORCE
    )
ENDIF()

################################################################################
#
# Configure depending selected ethernet driver
#
IF(CFG_POWERLINK_EDRV STREQUAL "8139")

    SET(MODULE_NAME "oplk8139")
    SET(MODULE_DEFS "${MODULE_DEFS} -DCONFIG_EDRV=8139")
    SET(MODULE_SOURCE_FILES ${MODULE_SOURCE_FILES} ${EDRV_SOURCE_DIR}/edrv-8139.c)

ELSEIF(CFG_POWERLINK_EDRV STREQUAL "82573")

    SET(MODULE_NAME "oplk82573")
    SET(MODULE_DEFS "${MODULE_DEFS} -DCONFIG_EDRV=82573")
    SET(MODULE_SOURCE_FILES ${MODULE_SOURCE_FILES} ${EDRV_SOURCE_DIR}/edrv-82573.c)

ELSEIF(CFG_POWERLINK_EDRV STREQUAL "8255x")

    SET(MODULE_NAME "oplk8255x")
    SET(MODULE_DEFS "${MODULE_DEFS} -DCONFIG_EDRV=8255")
    SET(MODULE_SOURCE_FILES ${MODULE_SOURCE_FILES} ${EDRV_SOURCE_DIR}/edrv-8255x.c)

ELSE()

    message(FATAL_ERROR
            "No valid ethernet driver (8139, 82573, 8255x) was specified in CFG_POWERLINK_EDRV")

ENDIF()

###############################################################################
#
# Configure depending selected mode
#
IF(CFG_OPLK_MN)
    SET(MODULE_DEFS "${MODULE_DEFS} -DCONFIG_MN")
    SET(MODULE_NAME "${MODULE_NAME}mn")
ELSE()
    SET(MODULE_NAME "${MODULE_NAME}cn")
ENDIF()

###############################################################################
#
# specifiy standard definitions
#
IF(CMAKE_BUILD_TYPE STREQUAL Debug OR CMAKE_BUILD_TYPE STREQUAL DEBUG)
    SET(MODULE_DEFS "${MODULE_DEFS} -D_DEBUG")
    #SET(MODULE_DEFS "${MODULE_DEFS} -D_DBG_TRACE_POINTS_")
    SET(MODULE_DEFS "${MODULE_DEFS} -DDEF_DEBUG_LVL=${CFG_DEBUG_LVL}")
ENDIF()

###############################################################################
#
# specifiy standard include directories
#

SET(MODULE_INCLUDES "${MODULE_INCLUDES} -I${CMAKE_SOURCE_DIR}")
SET(MODULE_INCLUDES "${MODULE_INCLUDES} -I${OPLK_INCLUDE_DIR}")
SET(MODULE_INCLUDES "${MODULE_INCLUDES} -I${STACK_SOURCE_DIR}")
SET(MODULE_INCLUDES "${MODULE_INCLUDES} -I${COMMON_SOURCE_DIR}/circbuf")
SET(MODULE_INCLUDES "${MODULE_INCLUDES} -I${KERNEL_SOURCE_DIR}/errhnd")
SET(MODULE_INCLUDES "${MODULE_INCLUDES} -I${KERNEL_SOURCE_DIR}/dll")
SET(MODULE_INCLUDES "${MODULE_INCLUDES} -I${STACK_INCLUDE_DIR}/target")
SET(MODULE_INCLUDES "${MODULE_INCLUDES} -I${STACK_INCLUDE_DIR}/target/linux")

#
# specifiy source files
#
SET(MODULE_SOURCE_FILES
    ${MODULE_SOURCE_FILES}
    ${CMAKE_SOURCE_DIR}/main.c
    ${CONTRIB_SOURCE_DIR}/trace/trace-printk.c
    ${EDRV_SOURCE_DIR}/edrvcyclic.c
    ${KERNEL_SOURCE_DIR}/ctrl/ctrlk.c
    ${KERNEL_SOURCE_DIR}/ctrl/ctrlkcal-linuxkernel.c
    ${KERNEL_SOURCE_DIR}/dll/dllk.c
    ${KERNEL_SOURCE_DIR}/dll/dllkfilter.c
    ${KERNEL_SOURCE_DIR}/dll/dllkstatemachine.c
    ${KERNEL_SOURCE_DIR}/dll/dllkevent.c
    ${KERNEL_SOURCE_DIR}/dll/dllkframe.c
    ${KERNEL_SOURCE_DIR}/dll/dllkcal.c
    ${KERNEL_SOURCE_DIR}/dll/dllkcal-linuxkernel.c
    ${KERNEL_SOURCE_DIR}/event/eventk.c
    ${KERNEL_SOURCE_DIR}/event/eventkcal-linuxkernel.c
    ${KERNEL_SOURCE_DIR}/event/eventkcalintf-circbuf.c
    ${COMMON_SOURCE_DIR}/event/event.c
    ${COMMON_SOURCE_DIR}/event/event.h
    ${KERNEL_SOURCE_DIR}/nmt/nmtk.c
    ${KERNEL_SOURCE_DIR}/pdo/pdok.c
    ${KERNEL_SOURCE_DIR}/pdo/pdokcal.c
    ${KERNEL_SOURCE_DIR}/pdo/pdokcal-triplebufshm.c
    ${KERNEL_SOURCE_DIR}/pdo/pdokcalmem-linuxkernel.c
    ${KERNEL_SOURCE_DIR}/pdo/pdokcalsync-linuxkernel.c
    ${KERNEL_SOURCE_DIR}/errhnd/errhndk.c
    ${KERNEL_SOURCE_DIR}/errhnd/errhndkcal-local.c
    ${KERNEL_SOURCE_DIR}/veth/veth-linuxkernel.c
    ${KERNEL_SOURCE_DIR}/timer/hrestimer-linuxkernel.c
    ${COMMON_SOURCE_DIR}/circbuf/circbuffer.c
    ${COMMON_SOURCE_DIR}/circbuf/circbuf-linuxkernel.c
    ${COMMON_SOURCE_DIR}/debugstr.c
    )

IF((CMAKE_SYSTEM_PROCESSOR MATCHES x86*) OR (CMAKE_SYSTEM_PROCESSOR MATCHES i686))
    SET(MODULE_SOURCE_FILES ${MODULE_SOURCE_FILES} ${COMMON_SOURCE_DIR}/ami/amix86.c)
ELSE()
    MESSAGE(FATAL_ERROR
            "ami: CMAKE_SYSTEM_PROCESSOR is set to ${CMAKE_SYSTEM_PROCESSOR}. Valid targets are (x86, x86_64, arm, armv7l, i686).")
ENDIF()

SET(MODULE_FILE ${MODULE_NAME}.ko)
SET(MODULE_OUTPUT_DIR ${CMAKE_CURRENT_BINARY_DIR})

IF (CFG_KERNEL_DIR STREQUAL "")
    SET(KERNEL_DIR "/lib/modules/${CMAKE_SYSTEM_VERSION}/build")
ELSE()
    SET(KERNEL_DIR ${CFG_KERNEL_DIR})
ENDIF()

# Link source files to do an out-of-source build
FOREACH(MODULE_SOURCE_FILE ${MODULE_SOURCE_FILES})
    GET_FILENAME_COMPONENT(MODULE_SOURCE_FILENAME ${MODULE_SOURCE_FILE} NAME)
    GET_FILENAME_COMPONENT(MODULE_SOURCE_BASENAME ${MODULE_SOURCE_FILE} NAME_WE)
    ADD_CUSTOM_COMMAND(OUTPUT ${MODULE_OUTPUT_DIR}/src/${MODULE_SOURCE_FILENAME}
                       COMMAND ${CMAKE_COMMAND} -E make_directory ${MODULE_OUTPUT_DIR}/src
                       COMMAND ${CMAKE_COMMAND} -E create_symlink ${MODULE_SOURCE_FILE} ${MODULE_OUTPUT_DIR}/src/${MODULE_SOURCE_FILENAME}
                       )
    SET(MODULE_OBJS "${MODULE_OBJS} src/${MODULE_SOURCE_BASENAME}.o")
    SET(MODULE_LINKED_SOURCE_FILES ${MODULE_LINKED_SOURCE_FILES} ${MODULE_OUTPUT_DIR}/src/${MODULE_SOURCE_FILENAME})
ENDFOREACH()

CONFIGURE_FILE(${TOOLS_DIR}/linux/Kbuild.in ${MODULE_OUTPUT_DIR}/Kbuild)

IF(CMAKE_CROSSCOMPILING)
    IF(DEFINED MAKE_KERNEL_ARCH)
        SET(MAKE_KERNEL_CROSSCOMPILING_PARAMS ARCH=${MAKE_KERNEL_ARCH})
    ENDIF()

    IF(DEFINED MAKE_KERNEL_CROSS_COMPILE)
        SET(MAKE_KERNEL_CROSSCOMPILING_PARAMS
                ${MAKE_KERNEL_CROSSCOMPILING_PARAMS} CROSS_COMPILE=${MAKE_KERNEL_CROSS_COMPILE})
    ENDIF()
ENDIF()

ADD_CUSTOM_COMMAND(
    OUTPUT ${MODULE_OUTPUT_DIR}/${MODULE_FILE}
    COMMAND ${CMAKE_MAKE_PROGRAM} ${MAKE_KERNEL_CROSSCOMPILING_PARAMS} -C ${KERNEL_DIR} M=${MODULE_OUTPUT_DIR} modules
    WORKING_DIRECTORY ${MODULE_OUTPUT_DIR}
    DEPENDS ${MODULE_LINKED_SOURCE_FILES} ${TOOLS_DIR}/linux/Kbuild.in
    VERBATIM
)

ADD_CUSTOM_TARGET(
    ${MODULE_NAME}
    ALL
    DEPENDS ${MODULE_OUTPUT_DIR}/${MODULE_FILE}
    )

ADD_CUSTOM_TARGET(
    module_clean
    COMMAND ${CMAKE_MAKE_PROGRAM} -C ${KERNEL_DIR} M=${MODULE_OUTPUT_DIR} clean
    #COMMAND ${CMAKE_COMMAND} -E remove_directory ${MODULE_OUTPUT_DIR}/src
    )

################################################################################
# add installation rules

INSTALL(FILES ${MODULE_OUTPUT_DIR}/${MODULE_NAME}.ko DESTINATION ${CMAKE_PROJECT_NAME})
INSTALL(FILES ${TOOLS_DIR}/linux/50-openPOWERLINK.rules DESTINATION ${CMAKE_PROJECT_NAME})
INSTALL(PROGRAMS ${TOOLS_DIR}/linux/plkload DESTINATION ${CMAKE_PROJECT_NAME})
INSTALL(PROGRAMS ${TOOLS_DIR}/linux/plkunload DESTINATION ${CMAKE_PROJECT_NAME})
INSTALL(PROGRAMS ${TOOLS_DIR}/linux/devices.txt DESTINATION ${CMAKE_PROJECT_NAME})
