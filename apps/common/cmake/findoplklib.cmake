################################################################################
#
# CMake script for finding the openPOWERLINK library
#
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
################################################################################

MACRO(FIND_OPLK_LIBRARY OPLK_NODE_TYPE)

    IF(CMAKE_SYSTEM_NAME STREQUAL "Linux")

        IF(CFG_KERNEL_STACK_DIRECTLINK)
            SET(OPLKLIB_NAME oplk${OPLK_NODE_TYPE})
        ELSEIF (CFG_KERNEL_STACK_USERSPACE_DAEMON)
            SET(OPLKLIB_NAME oplk${OPLK_NODE_TYPE}app-userintf)
        ELSEIF (CFG_KERNEL_STACK_KERNEL_MODULE)
            SET(OPLKLIB_NAME oplk${OPLK_NODE_TYPE}app-kernelintf)
        ENDIF (CFG_KERNEL_STACK_DIRECTLINK)

    ELSEIF(CMAKE_SYSTEM_NAME STREQUAL "Windows")

        SET(OPLKLIB_NAME oplk${OPLK_NODE_TYPE})

    ELSE(CMAKE_SYSTEM_NAME STREQUAL "Linux")

        MESSAGE(FATAL_ERROR "Unsupported CMAKE_SYSTEM_NAME ${CMAKE_SYSTEM_NAME}")

    ENDIF(CMAKE_SYSTEM_NAME STREQUAL "Linux")

    SET(OPLKLIB_DEBUG_NAME "${OPLKLIB_NAME}_d")

    # Set oplk library and include directory
    SET(OPLKLIB_DIR ${OPLK_ROOT_DIR}/stack/lib/${SYSTEM_NAME_DIR}/${SYSTEM_PROCESSOR_DIR})
    SET(OPLKLIB_INCDIR ${OPLK_ROOT_DIR}/stack/proj/${SYSTEM_NAME_DIR}/lib${OPLKLIB_NAME})

    # Search for release library
    UNSET(OPLKLIB CACHE)
    MESSAGE(STATUS "Searching for LIBRARY ${OPLKLIB_NAME} in ${OPLKLIB_DIR}")
    FIND_LIBRARY(OPLKLIB NAME ${OPLKLIB_NAME}
                         HINTS ${OPLKLIB_DIR})

    # Search for debug library
    UNSET(OPLKLIB_DEBUG CACHE)
    MESSAGE(STATUS "Searching for LIBRARY ${OPLKLIB_DEBUG_NAME} in ${OPLKLIB_DIR}")
    FIND_LIBRARY(OPLKLIB_DEBUG NAME ${OPLKLIB_DEBUG_NAME}
                               HINTS ${OPLKLIB_DIR})

    INCLUDE_DIRECTORIES(${OPLKLIB_INCDIR})

ENDMACRO(FIND_OPLK_LIBRARY)
