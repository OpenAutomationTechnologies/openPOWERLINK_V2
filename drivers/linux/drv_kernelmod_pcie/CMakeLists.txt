################################################################################
#
# CMake file for openPOWERLINK user PCIe interface (Linux kernel PCIe module)
#
# Copyright (c) 2013, SYSTEC electronik GmbH
# Copyright (c) 2014, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
# Copyright (c) 2015, Kalycito Infotech Private Limited
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

PROJECT (oplkdrv_kernelmodule_pcie C)

IF(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    CMAKE_MINIMUM_REQUIRED (VERSION 2.8.7)
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

################################################################################
# Set global directories
################################################################################
SET(OPLK_BASE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../..)
SET(CONTRIB_SOURCE_DIR ${OPLK_BASE_DIR}/contrib)
SET(STACK_SOURCE_DIR ${OPLK_BASE_DIR}/stack/src)
SET(COMMON_SOURCE_DIR ${OPLK_BASE_DIR}/stack/src/common)
SET(ARCH_SOURCE_DIR ${OPLK_BASE_DIR}/stack/src/arch/linuxkernel)
SET(KERNEL_SOURCE_DIR ${OPLK_BASE_DIR}/stack/src/kernel)
SET(EDRV_SOURCE_DIR ${OPLK_BASE_DIR}/stack/src/kernel/edrv)
SET(OPLK_INCLUDE_DIR ${OPLK_BASE_DIR}/stack/include)
SET(TOOLS_DIR ${OPLK_BASE_DIR}/tools)

IF(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
  SET(CMAKE_INSTALL_PREFIX
    ${OPLK_BASE_DIR}/bin/${SYSTEM_NAME_DIR}/${SYSTEM_PROCESSOR_DIR}/
    CACHE PATH "openPOWERLINK kernel driver install prefix" FORCE
    )
ENDIF()

###############################################################################
#
# Configure depending selected mode
#
IF(CFG_OPLK_MN)
# Export C defines needed to identify the Linux PCIe MN design
    ADD_DEFINITIONS(-D__PCIE__ -DCONFIG_MN)
    SET(MODULE_NAME "${MODULE_NAME}mn")
    INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/proj/mn)
ELSE()
    MESSAGE(SEND_ERROR "Currently the PCIe interface only supports MN!")
ENDIF()

SET(MODULE_NAME "oplk${MODULE_NAME}pcieintf")

###############################################################################
#
# Specifiy standard definitions
#
IF(CMAKE_BUILD_TYPE STREQUAL Debug OR CMAKE_BUILD_TYPE STREQUAL DEBUG)
    ADD_DEFINITIONS(-D_DEBUG -DDEF_DEBUG_LVL=${CFG_DEBUG_LVL})
    #ADD_DEFINITIONS(-D_DBG_TRACE_POINTS_)
ELSE()
    ADD_DEFINITIONS(-DNDEBUG)
ENDIF()

GET_PROPERTY(MODULE_DEFS_LIST DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY COMPILE_DEFINITIONS)
FOREACH(NEXT_MODULE_DEF ${MODULE_DEFS_LIST})
    SET(MODULE_DEFS "${MODULE_DEFS} -D${NEXT_MODULE_DEF}")
ENDFOREACH()

###############################################################################
#
# Specifiy standard include directories
#

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR})
INCLUDE_DIRECTORIES(${OPLK_INCLUDE_DIR})
INCLUDE_DIRECTORIES(${STACK_SOURCE_DIR})
INCLUDE_DIRECTORIES(${COMMON_SOURCE_DIR}/circbuf)
INCLUDE_DIRECTORIES(${KERNEL_SOURCE_DIR}/errhnd)
INCLUDE_DIRECTORIES(${KERNEL_SOURCE_DIR}/dll)
INCLUDE_DIRECTORIES(${STACK_INCLUDE_DIR}/target)
INCLUDE_DIRECTORIES(${STACK_INCLUDE_DIR}/target/linux)
INCLUDE_DIRECTORIES(${CONTRIB_SOURCE_DIR}/dualprocshm/include)

GET_PROPERTY(DRIVER_INCLUDE_DIRS DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
FOREACH(NEXT_INCLUDE_DIR ${DRIVER_INCLUDE_DIRS})
    SET(MODULE_INCLUDES "${MODULE_INCLUDES} -I${NEXT_INCLUDE_DIR}")
ENDFOREACH()

#
# Specifiy source files
#
SET(MODULE_SOURCE_FILES
    ${MODULE_SOURCE_FILES}
    ${CMAKE_CURRENT_SOURCE_DIR}/main.c
    ${CMAKE_CURRENT_SOURCE_DIR}/drvintf-dualprocshm.c
    ${CMAKE_CURRENT_SOURCE_DIR}/pciedrv.c
    ${CONTRIB_SOURCE_DIR}/dualprocshm/src/dualprocshm-linuxpcie.c
    ${CONTRIB_SOURCE_DIR}/dualprocshm/src/dualprocshm-intr.c
    ${CONTRIB_SOURCE_DIR}/dualprocshm/src/dualprocshm-noos.c
    ${COMMON_SOURCE_DIR}/circbuf/circbuf-noosdual.c
    ${COMMON_SOURCE_DIR}/circbuf/circbuffer.c
    ${COMMON_SOURCE_DIR}/debugstr.c
    ${CONTRIB_SOURCE_DIR}/trace/trace-printk.c
    ${KERNEL_SOURCE_DIR}/timesync/timesynck.c
    ${KERNEL_SOURCE_DIR}/timesync/timesynckcal-linuxkernel.c
    ${KERNEL_SOURCE_DIR}/veth/veth-linuxpcie.c
    ${ARCH_SOURCE_DIR}/target-linuxkernel.c
    )

IF(CMAKE_SYSTEM_PROCESSOR MATCHES "^(i.86|x86(_64)?)$")
    SET(MODULE_SOURCE_FILES ${MODULE_SOURCE_FILES} ${COMMON_SOURCE_DIR}/ami/amix86.c)
ELSEIF(CMAKE_SYSTEM_PROCESSOR MATCHES arm*)
    SET(MODULE_SOURCE_FILES ${MODULE_SOURCE_FILES} ${COMMON_SOURCE_DIR}/ami/amile.c)
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
    ELSE()
        MESSAGE(WARNING "ARCH is not set while cross-compiling !\n"
            "Set MAKE_KERNEL_ARCH with your target architecture.")
    ENDIF()

    IF(DEFINED MAKE_KERNEL_CROSS_COMPILE)
        SET(MAKE_KERNEL_CROSSCOMPILING_PARAMS
                ${MAKE_KERNEL_CROSSCOMPILING_PARAMS} CROSS_COMPILE=${MAKE_KERNEL_CROSS_COMPILE})
    ELSE()
        MESSAGE(WARNING "CROSS_COMPILE is not set while cross-compiling !\n"
            "Set MAKE_KERNEL_CROSS_COMPILE with your toolchain prefix")
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

INSTALL(FILES ${MODULE_OUTPUT_DIR}/${MODULE_NAME}.ko DESTINATION ${PROJECT_NAME})
