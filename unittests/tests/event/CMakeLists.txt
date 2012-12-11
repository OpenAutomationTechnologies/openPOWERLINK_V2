################################################################################
#
# Project: openPOWERLINK
#
# Description: CMake file for unit tests of event module
#
# Copyright (c) 2012, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
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

CMAKE_MINIMUM_REQUIRED (VERSION 2.8)

PROJECT (unittest-event)

# Drivers implement the tests and provide the testmethods
SET (TEST_DRIVER
    ${PROJECT_SOURCE_DIR}/test-event.c
    ${PROJECT_SOURCE_DIR}/tests.c
)

# Provide all stubs needed for running the tests
SET (TEST_STUBS
    ${PROJECT_SOURCE_DIR}/stubs.c
    ${PROJECT_SOURCE_DIR}/stub_eventkcal.c
)

# Provide all openPOWERLINK files needed to compile
SET (TEST_OPENPOWERLINK
)

INCLUDE_DIRECTORIES ("${PROJECT_SOURCE_DIR}")

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR})

#
# additional compiler flags
#
ADD_DEFINITIONS(-Wall -Wextra -pedantic -std=c99 -pthread -D_GNU_SOURCE -D_POSIX_C_SOURCE=200112L)
ADD_DEFINITIONS(-DCONFIG_CFM -DCONFIG_OPENCONFIGURATOR_MAPPING -DCONFIG_MN -DCONFIG_POWERLINK_USERSTACK)

# set sources of event test
SET (TEST_SOURCES ${CMAKE_SOURCE_DIR}/unittests/common/basictest.c
                  ${TEST_DRIVER}
                  ${TEST_STUBS}
                  ${TEST_OPENPOWERLINK}
                  ${POWERLINK_SOURCE_DIR}/EplStack/common/event/event.c
                  ${POWERLINK_SOURCE_DIR}/EplStack/kernel/event/eventk.c
)

ADD_UNIT_TEST ("Unit test for event handler" "test_event" "${TEST_SOURCES}" )

SET_PROPERTY(TARGET test_event
             PROPERTY COMPILE_DEFINITIONS_DEBUG DEBUG;DEF_DEBUG_LVL=${CFG_DEBUG_LVL})

TARGET_LINK_LIBRARIES(test_event pthread rt)


