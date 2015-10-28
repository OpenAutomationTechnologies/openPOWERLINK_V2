################################################################################
#
# CMake file for openPOWERLINK kernel stack (Windows kernel driver)
#
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

PROJECT (drv_ndis_intermediate NONE)

# include cmake modules
INCLUDE(CMakeDependentOption)

CMAKE_MINIMUM_REQUIRED(VERSION 2.8.0)

################################################################################
# Set global directories
################################################################################
SET(OPLK_ROOT_DIR ${CMAKE_SOURCE_DIR}/../../..)
SET(CONTRIB_SOURCE_DIR ${OPLK_ROOT_DIR}/contrib)
SET(STACK_SOURCE_DIR ${OPLK_ROOT_DIR}/stack/src)
SET(COMMON_SOURCE_DIR ${OPLK_ROOT_DIR}/stack/src/common)
SET(ARCH_SOURCE_DIR ${OPLK_ROOT_DIR}/stack/src/arch/winkernel)
SET(KERNEL_SOURCE_DIR ${OPLK_ROOT_DIR}/stack/src/kernel)
SET(OPLK_INCLUDE_DIR ${OPLK_ROOT_DIR}/stack/include)
SET(TOOLS_DIR ${OPLK_ROOT_DIR}/tools)
SET(NDISLIB_SOURCE_DIR ${CONTRIB_SOURCE_DIR}/ndislib)

SET(CMAKE_MODULE_PATH "${OPLK_ROOT_DIR}/cmake" ${CMAKE_MODULE_PATH})
INCLUDE(genvsfilelist)
INCLUDE(genvsfilters)

SET(KERNEL_SOURCE_FILES
    ${KERNEL_SOURCE_DIR}/ctrl/ctrlk.c
    ${KERNEL_SOURCE_DIR}/ctrl/ctrlkcal-direct.c
    ${KERNEL_SOURCE_DIR}/dll/dllk.c
    ${KERNEL_SOURCE_DIR}/dll/dllkcal-circbuf.c
    ${KERNEL_SOURCE_DIR}/dll/dllkcal.c
    ${KERNEL_SOURCE_DIR}/dll/dllkevent.c
    ${KERNEL_SOURCE_DIR}/dll/dllkfilter.c
    ${KERNEL_SOURCE_DIR}/dll/dllkframe.c
    ${KERNEL_SOURCE_DIR}/dll/dllknode.c
    ${KERNEL_SOURCE_DIR}/dll/dllkstatemachine.c
    ${KERNEL_SOURCE_DIR}/edrv/edrv-ndisintermediate.c
    ${KERNEL_SOURCE_DIR}/edrv/edrvcyclic.c
    ${KERNEL_SOURCE_DIR}/errhnd/errhndk.c
    ${KERNEL_SOURCE_DIR}/errhnd/errhndkcal-local.c
    ${KERNEL_SOURCE_DIR}/event/eventk.c
    ${KERNEL_SOURCE_DIR}/event/eventkcal-winkernel.c
    ${KERNEL_SOURCE_DIR}/event/eventkcalintf-circbuf.c
    ${KERNEL_SOURCE_DIR}/nmt/nmtk.c
    ${KERNEL_SOURCE_DIR}/pdo/pdok.c
    ${KERNEL_SOURCE_DIR}/pdo/pdokcal-triplebufshm.c
    ${KERNEL_SOURCE_DIR}/pdo/pdokcal.c
    ${KERNEL_SOURCE_DIR}/pdo/pdokcalmem-winkernel.c
    ${KERNEL_SOURCE_DIR}/pdo/pdoklut.c
    ${KERNEL_SOURCE_DIR}/timesync/timesynck.c
    ${KERNEL_SOURCE_DIR}/timesync/timesynckcal-winkernel.c
    ${KERNEL_SOURCE_DIR}/timer/hrestimer-ndistimer.c
    ${KERNEL_SOURCE_DIR}/veth/veth-ndisintemediate.c
    )

SET(COMMON_SOURCE_FILES
    ${COMMON_SOURCE_DIR}/circbuf/circbuf-winkernel.c
    ${COMMON_SOURCE_DIR}/circbuf/circbuffer.c
    ${COMMON_SOURCE_DIR}/debugstr.c
   )

SET(ARCH_SOURCE_FILES
    ${COMMON_SOURCE_DIR}/ami/amix86.c
    ${ARCH_SOURCE_DIR}/target-winkernel.c
    )

SET(NDIS_SOURCE_FILES
    ${NDISLIB_SOURCE_DIR}/ndisintermediate/ndis-imMiniport.c
    ${NDISLIB_SOURCE_DIR}/ndisintermediate/ndis-imProtocol.c
    ${NDISLIB_SOURCE_DIR}/ndisintermediate/ndis-im.c
    )

SET(DRIVER_SOURCE_FILES
    ${CMAKE_SOURCE_DIR}/drvintf.c
    ${CMAKE_SOURCE_DIR}/main.c
    )

SET(DRIVER_SOURCE_LIST
    ${DRIVER_SOURCE_FILES}
    ${KERNEL_SOURCE_FILES}
    ${COMMON_SOURCE_FILES}
    ${ARCH_SOURCE_FILES}
    ${NDIS_SOURCE_FILES}
    )

GEN_VS_FILE_LIST("${DRIVER_SOURCE_LIST}" PART_VS_FILE_LIST)
SET(VS_FILE_LIST "${VS_FILE_LIST} ${PART_VS_FILE_LIST}")

GEN_VS_FILTER_LIST("${KERNEL_SOURCE_FILES}" "Kernel Source Files" PART_VS_FILTER_LIST)
SET(VS_FILTER_LIST "${VS_FILTER_LIST} ${PART_VS_FILTER_LIST}")

GEN_VS_FILTER_LIST("${COMMON_SOURCE_FILES}" "Common Source Files" PART_VS_FILTER_LIST)
SET(VS_FILTER_LIST "${VS_FILTER_LIST} ${PART_VS_FILTER_LIST}")

GEN_VS_FILTER_LIST("${ARCH_SOURCE_FILES}" "Architecture Source Files" PART_VS_FILTER_LIST)
SET(VS_FILTER_LIST "${VS_FILTER_LIST} ${PART_VS_FILTER_LIST}")

GEN_VS_FILTER_LIST("${NDIS_SOURCE_FILES}" "Ndis Intermediate Source" PART_VS_FILTER_LIST)
SET(VS_FILTER_LIST "${VS_FILTER_LIST} ${PART_VS_FILTER_LIST}")

GEN_VS_FILTER_LIST("${DRIVER_SOURCE_FILES}" "" PART_VS_FILTER_LIST)
SET(VS_FILTER_LIST "${VS_FILTER_LIST} ${PART_VS_FILTER_LIST}")

CONFIGURE_FILE(${TOOLS_DIR}/windows/vstargets/sourceslist.in ${PROJECT_BINARY_DIR}/drv_ndis_intermediate/sources.targets @ONLY)
CONFIGURE_FILE(${TOOLS_DIR}/windows/vstargets/sourcefilters.in ${PROJECT_BINARY_DIR}/drv_ndis_intermediate/filters.targets @ONLY)
