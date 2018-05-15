################################################################################
#
# Windows definitions for demo_mn_qt application
#
# Copyright (c) 2017, B&R Industrial Automation GmbH
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

################################################################################
# Set architecture specific definitions
ADD_DEFINITIONS(-D_CONSOLE -DHAVE_REMOTE -D_CRT_SECURE_NO_WARNINGS)

################################################################################
# Set architecture specific sources and include directories

SET(DEMO_ARCH_SOURCES
    ${DEMO_ARCH_SOURCES}
    ${COMMON_SOURCE_DIR}/system/system-windows.c
    )


################################################################################
# Set architecture specific libraries

IF(CMAKE_CL_64)
    LINK_DIRECTORIES(${CONTRIB_SOURCE_DIR}/pcap/windows/WpdPack/Lib/x64)
ELSE()
    LINK_DIRECTORIES(${CONTRIB_SOURCE_DIR}/pcap/windows/WpdPack/Lib)
ENDIF()

SET(ARCH_LIBRARIES ${ARCH_LIBRARIES} wpcap iphlpapi)

################################################################################
# Set architecture specific installation files
IF(DEFINED OPLKDLL)
    IF(NOT(${OPLKDLL} STREQUAL "OPLKDLL-NOTFOUND"))
        INSTALL(FILES ${OPLKDLL}
                DESTINATION ${CMAKE_PROJECT_NAME}
                CONFIGURATIONS "Release"
                )
    ENDIF()
ENDIF()

IF(DEFINED OPLKDLL_DEBUG)
    IF(NOT(${OPLKDLL_DEBUG} STREQUAL "OPLKDLL_DEBUG-NOTFOUND"))
        INSTALL(FILES ${OPLKDLL_DEBUG}
                DESTINATION ${CMAKE_PROJECT_NAME}
                CONFIGURATIONS "Debug"
                )
    ENDIF()
ENDIF()

GET_TARGET_PROPERTY(QtCore_location Qt5::Core LOCATION)
GET_FILENAME_COMPONENT(_qtLocation "${QtCore_location}" DIRECTORY)

INSTALL(FILES ${_qtLocation}/Qt5Core.dll
              ${_qtLocation}/Qt5Gui.dll
              ${_qtLocation}/Qt5Widgets.dll
              ${_qtLocation}/icudt54.dll
              ${_qtLocation}/icuin54.dll
              ${_qtLocation}/icuuc54.dll
              ${_qtLocation}/libEGL.dll
              ${_qtLocation}/libGLESv2.dll
        DESTINATION ${CMAKE_PROJECT_NAME}
        CONFIGURATIONS "Release"
        )
INSTALL(FILES ${_qtLocation}/../plugins/platforms/qwindows.dll
        DESTINATION ${CMAKE_PROJECT_NAME}/platforms
        CONFIGURATIONS "Release"
        )

INSTALL(FILES ${_qtLocation}/Qt5Cored.dll
              ${_qtLocation}/Qt5Guid.dll
              ${_qtLocation}/Qt5Widgetsd.dll
              ${_qtLocation}/icudt54.dll
              ${_qtLocation}/icuin54.dll
              ${_qtLocation}/icuuc54.dll
              ${_qtLocation}/libEGLd.dll
              ${_qtLocation}/libGLESv2d.dll
        DESTINATION ${CMAKE_PROJECT_NAME}
        CONFIGURATIONS "Debug"
        )
INSTALL(FILES ${_qtLocation}/../plugins/platforms/qwindowsd.dll
        DESTINATION ${CMAKE_PROJECT_NAME}/platforms
        CONFIGURATIONS "Debug"
        )
