################################################################################
#
# Project: openPOWERLINK
#
# Description: CMake file of openPOWERLINK unit tests
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

# Instructions for adding a unit test
MACRO(ADD_UNIT_TEST TestDirectory TestExeName TST_SOURCES)
    STRING (TOUPPER ${TestDirectory} TestName)

    ADD_DEFINITIONS (${TEST_XML_OUTPUT})
    ADD_EXECUTABLE (${TestExeName} ${TST_SOURCES})

    TARGET_LINK_LIBRARIES (${TestExeName} cunit)

    ADD_TEST (${TestName} ${PROJECT_BINARY_DIR}/${TestExeName})
ENDMACRO(ADD_UNIT_TEST)

# general unit test includes
INCLUDE_DIRECTORIES ("/usr/include")
INCLUDE_DIRECTORIES ("${CMAKE_SOURCE_DIR}/unittests/common")
INCLUDE_DIRECTORIES ("${CMAKE_SOURCE_DIR}/EplStack")
INCLUDE_DIRECTORIES ("${CMAKE_SOURCE_DIR}/Include")
INCLUDE_DIRECTORIES ("${CMAKE_SOURCE_DIR}/Examples/X86/Generic/powerlink_user_lib")

# tests for event handler
ADD_SUBDIRECTORY (tests/event)