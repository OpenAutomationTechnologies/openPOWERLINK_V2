################################################################################
#
# CMake file for openPOWERLINK kernel stack (Windows kernel driver)
#
# Copyright (c) 2015, Kalycito Infotech Private Limited
# Copyright (c) 2016, B&R Industrial Automation GmbH
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

PROJECT (drv_ndis_pcie NONE)

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
SET(DPSHM_SOURCE_DIR ${CONTRIB_SOURCE_DIR}/dualprocshm)
SET(NDISLIB_SOURCE_DIR ${CONTRIB_SOURCE_DIR}/ndislib)

SET(CMAKE_MODULE_PATH "${OPLK_ROOT_DIR}/cmake" ${CMAKE_MODULE_PATH})
INCLUDE(genvsfilelist)
INCLUDE(genvsfilters)

SET(DRIVER_SOURCE_FILES
    ${CMAKE_SOURCE_DIR}/drvintf.c
    ${CMAKE_SOURCE_DIR}/main.c
    ${DPSHM_SOURCE_DIR}/src/dualprocshm-intr.c
    ${DPSHM_SOURCE_DIR}/src/dualprocshm-noos.c
    ${DPSHM_SOURCE_DIR}/src/dualprocshm-lock.c
    ${DPSHM_SOURCE_DIR}/src/dualprocshm-winpcie.c
    ${CONTRIB_SOURCE_DIR}/trace/trace-windows.c
    ${NDISLIB_SOURCE_DIR}/ndispcie/ndis-pcie.c
    ${NDISLIB_SOURCE_DIR}/ndispcie/ndis-pcieMiniport.c
    ${ARCH_SOURCE_DIR}/target-winkernel.c
    ${COMMON_SOURCE_DIR}/ami/amix86.c
    ${COMMON_SOURCE_DIR}/circbuf/circbuf-noosdual.c
    ${COMMON_SOURCE_DIR}/circbuf/circbuffer.c
    ${COMMON_SOURCE_DIR}/debugstr.c
   )

GEN_VS_FILE_LIST("${DRIVER_SOURCE_FILES}" PART_VS_FILE_LIST)
SET(VS_FILE_LIST "${VS_FILE_LIST} ${PART_VS_FILE_LIST}")

GEN_VS_FILTER_LIST("${DRIVER_SOURCE_FILES}" "" PART_VS_FILTER_LIST)
SET(VS_FILTER_LIST "${VS_FILTER_LIST} ${PART_VS_FILTER_LIST}")

CONFIGURE_FILE(${TOOLS_DIR}/windows/vstargets/sourceslist.in ${PROJECT_BINARY_DIR}/drv_ndis_pcie/sources.targets @ONLY)
CONFIGURE_FILE(${TOOLS_DIR}/windows/vstargets/sourcefilters.in ${PROJECT_BINARY_DIR}/drv_ndis_pcie/filters.targets @ONLY)
