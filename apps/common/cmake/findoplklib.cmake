################################################################################
#
# CMake script for finding the openPOWERLINK library
#
# Copyright (c) 2014, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
# Copyright (c) 2016, Kalycito Infotech Private Limited
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

MACRO(FIND_OPLK_LIBRARY OPLK_NODE_TYPE)

    UNSET(OPLKLIB CACHE)
    UNSET(OPLKLIB_DEBUG CACHE)

    IF(CMAKE_SYSTEM_NAME STREQUAL "Linux")

        IF(CFG_KERNEL_STACK_DIRECTLINK)
            SET(OPLKLIB_NAME oplk${OPLK_NODE_TYPE})
        ELSEIF (CFG_KERNEL_STACK_USERSPACE_DAEMON)
            SET(OPLKLIB_NAME oplk${OPLK_NODE_TYPE}app-userintf)
        ELSEIF (CFG_KERNEL_STACK_KERNEL_MODULE)
            SET(OPLKLIB_NAME oplk${OPLK_NODE_TYPE}app-kernelintf)
        ELSEIF ((CFG_KERNEL_STACK_PCIE_INTF) OR (CFG_KERNEL_STACK_ZYNQ_INTF))
            SET(OPLKLIB_NAME oplk${OPLK_NODE_TYPE}app-kernelpcp)
        ENDIF (CFG_KERNEL_STACK_DIRECTLINK)

    ELSEIF(CMAKE_SYSTEM_NAME STREQUAL "Windows")

        IF(CFG_KERNEL_STACK_DIRECTLINK)
            SET(OPLKLIB_NAME oplk${OPLK_NODE_TYPE})
        ELSEIF (CFG_KERNEL_STACK_PCIE)
            SET(OPLKLIB_NAME oplk${OPLK_NODE_TYPE}app-pcieintf)
        ELSEIF (CFG_KERNEL_STACK_KERNEL_MODULE)
            SET(OPLKLIB_NAME oplk${OPLK_NODE_TYPE}app-kernelintf)
        ENDIF (CFG_KERNEL_STACK_DIRECTLINK)

    ELSEIF(CMAKE_SYSTEM_NAME STREQUAL "Generic" AND CMAKE_SYSTEM_PROCESSOR STREQUAL "Microblazeise")
        IF (CFG_KERNEL_STACK_DIRECTLINK)
            SET(OPLKLIB_NAME oplk${OPLK_NODE_TYPE})
        ELSEIF (CFG_KERNEL_STACK_PCP_HOSTIF_MODULE)
            SET(OPLKLIB_NAME oplk${OPLK_NODE_TYPE}app-hostif)
        ENDIF (CFG_KERNEL_STACK_DIRECTLINK)

    ELSEIF (CMAKE_SYSTEM_NAME STREQUAL "Generic" AND CMAKE_SYSTEM_PROCESSOR STREQUAL "alterac5arm")
        IF (CFG_KERNEL_STACK_DIRECTLINK)
            SET(OPLKLIB_NAME oplk${OPLK_NODE_TYPE})
        ELSEIF (CFG_KERNEL_STACK_PCP_HOSTIF_MODULE)
            SET(OPLKLIB_NAME oplk${OPLK_NODE_TYPE}app-hostif)
        ELSEIF(CFG_KERNEL_DUALPROCSHM)
            SET(OPLKLIB_NAME oplk${OPLK_NODE_TYPE}app-dualprocshm)
        ELSE()
            MESSAGE("No configuration selected for ${CMAKE_SYSTEM_PROCESSOR}!!")
        ENDIF (CFG_KERNEL_STACK_DIRECTLINK)

    ELSE ()

        MESSAGE(FATAL_ERROR "Unsupported CMAKE_SYSTEM_NAME ${CMAKE_SYSTEM_NAME} or CMAKE_SYSTEM_PROCESSOR ${CMAKE_SYSTEM_PROCESSOR}")

    ENDIF()

    SET(OPLKLIB_DEBUG_NAME "${OPLKLIB_NAME}_d")

    # Set oplk library directory
    SET(OPLKLIB_DIR ${OPLK_BASE_DIR}/stack/lib/${SYSTEM_NAME_DIR}/${SYSTEM_PROCESSOR_DIR})

    IF((CMAKE_GENERATOR MATCHES "Visual Studio") OR (CMAKE_BUILD_TYPE STREQUAL "Release"))
        # Search for release library
        UNSET(OPLKLIB CACHE)
        MESSAGE(STATUS "Searching for LIBRARY ${OPLKLIB_NAME} in ${OPLKLIB_DIR}")
        FIND_LIBRARY(OPLKLIB NAME ${OPLKLIB_NAME}
                             HINTS ${OPLKLIB_DIR} ${OPLKLIB_DIR}/${CFG_DEMO_BOARD_NAME}/${CFG_DEMO_NAME})

        IF(CMAKE_SYSTEM_NAME STREQUAL "Windows")

            UNSET(OPLKDLL CACHE)
            FIND_PROGRAM(OPLKDLL NAME ${OPLKLIB_NAME}.dll
                                 HINTS ${OPLKLIB_DIR})

        ENDIF(CMAKE_SYSTEM_NAME STREQUAL "Windows")
    ENDIF()

    IF((CMAKE_GENERATOR MATCHES "Visual Studio") OR (CMAKE_BUILD_TYPE STREQUAL "Debug"))
        # Search for debug library
        UNSET(OPLKLIB_DEBUG CACHE)
        MESSAGE(STATUS "Searching for LIBRARY ${OPLKLIB_DEBUG_NAME} in ${OPLKLIB_DIR}")
        FIND_LIBRARY(OPLKLIB_DEBUG NAME ${OPLKLIB_DEBUG_NAME}
                                   HINTS ${OPLKLIB_DIR} ${OPLKLIB_DIR}/${CFG_DEMO_BOARD_NAME}/${CFG_DEMO_NAME})

        IF(CMAKE_SYSTEM_NAME STREQUAL "Windows")

            UNSET(OPLKDLL_DEBUG CACHE)
            FIND_PROGRAM(OPLKDLL_DEBUG NAME ${OPLKLIB_DEBUG_NAME}.dll
                                       HINTS ${OPLKLIB_DIR})

        ENDIF(CMAKE_SYSTEM_NAME STREQUAL "Windows")
    ENDIF()

ENDMACRO(FIND_OPLK_LIBRARY)
