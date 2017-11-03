################################################################################
#
# CMake file for openPOWERLINK MN user library on
# no OS platforms with host interface
#
# Copyright (c) 2016, Kalycito Infotech Private Limited
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
################################################################################

PROJECT(oplkmnapp-hostif C)
MESSAGE(STATUS "Configuring ${PROJECT_NAME}")

################################################################################

# set sources of POWERLINK library
SET(LIB_KERNEL_SOURCES
     ${KERNEL_SOURCES}
     ${CTRL_KCAL_HOSTIF_SOURCES}
     ${DLL_KCAL_CIRCBUF_SOURCES}
     ${EVENT_KCAL_NOOSHOSTIF_SOURCES}
     ${ERRHND_KCAL_HOSTIF_SOURCES}
     ${PDO_KCAL_HOSTIF_SOURCES}
     ${HARDWARE_DRIVER_OPENMAC_SOURCES}
     ${HARDWARE_DRIVER_OPENMAC_MN_SOURCES}
    )

SET(LIB_USER_SOURCES
     ${USER_MN_SOURCES}
     ${USER_SOURCES}
     ${CTRL_UCAL_HOSTIF_SOURCES}
     ${DLL_UCAL_CIRCBUF_SOURCES}
     ${ERRHND_UCAL_HOSTIF_SOURCES}
     ${MEMMAP_NOOSHOSTIF_SOURCES}
     ${PDO_UCAL_HOSTIF_SOURCES}
     ${EVENT_UCAL_NOOSHOSTIF_SOURCES}
     ${USER_TIMER_GENERIC_SOURCES}
    )

SET(LIB_COMMON_SOURCES
     ${COMMON_SOURCES}
     ${CIRCBUF_NOOSHOSTIF_SOURCES}
     ${COMMON_NOOS_SOURCES}
     ${ARCH_LE_SOURCES}
    )

# Create full source list
SET(LIB_SOURCES
     ${LIB_USER_SOURCES}
     ${LIB_COMMON_SOURCES}
    )

# set includes of POWERLINK library
INCLUDE_DIRECTORIES(
                    ${PROJECT_SOURCE_DIR}
                   )

# set flags of the openPOWERLINK library
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -pedantic -std=c99")
ADD_DEFINITIONS(-DCONFIG_HOSTIF_PCP=FALSE -DCONFIG_PCP=FALSE)

################################################################################
# Handle target specific functions
IF (${CMAKE_SYSTEM_NAME} STREQUAL "Generic" AND ${CMAKE_SYSTEM_PROCESSOR} STREQUAL "Microblazeise")
    INCLUDE(configure-microblazeise.cmake)
ELSE ()
    MESSAGE(FATAL_ERROR "Unknown platform selected!")
ENDIF ()

################################################################################
# Define library and installation rules
ADD_LIBRARY(${PROJECT_NAME} ${LIB_TYPE} ${LIB_SOURCES} ${LIB_ARCH_SOURCES})
SET_PROPERTY(TARGET ${PROJECT_NAME} PROPERTY COMPILE_DEFINITIONS_DEBUG DEBUG;DEF_DEBUG_LVL=${CFG_DEBUG_LVL})
SET_PROPERTY(TARGET ${PROJECT_NAME} PROPERTY DEBUG_POSTFIX "_d")
INSTALL(TARGETS ${PROJECT_NAME} ARCHIVE DESTINATION ${ARCH_INSTALL_POSTFIX})
INSTALL(FILES ${PROJECT_SOURCE_DIR}/oplkcfg.h DESTINATION ${ARCH_INSTALL_POSTFIX}/config-${PROJECT_NAME})
