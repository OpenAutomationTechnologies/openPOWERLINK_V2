################################################################################
#
# CMake file of CiA 302 MN QT demo application
#
# Copyright (c) 2013, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
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
FIND_PACKAGE(Qt4 REQUIRED)

SET(OBJDICT CiA302-4_MN)

SET (DEMO_SOURCES
     src/main.cpp
     src/MainWindow.cpp
     src/Api.cpp
     src/CnState.cpp
     src/DataInOutThread.cpp
     src/Input.cpp
     src/Output.cpp
     src/ProcessThread.cpp
     src/State.cpp
     src/Leds.cpp
     src/NodeState.cpp
     ${LIB_SOURCE_DIR}/console/printlog.c
     )

SET (DEMO_RESOURCES resources/resources.qrc)

SET (DEMO_HEADERS
     include/Api.h
     include/CnState.h
     include/DataInOutThread.h
     include/Input.h
     include/Output.h
     include/ProcessThread.h
     include/State.h
     include/Leds.h
     include/MainWindow.h
     include/NodeState.h
     )

IF (CFG_KERNEL_STACK_DIRECTLINK OR CFG_KERNEL_STACK_USERSPACE_DAEMON)
    SET (DEMO_SOURCES ${DEMO_SOURCES} src/InterfaceSelectDialog.cpp)
    SET (DEMO_HEADERS ${DEMO_HEADERS} include/InterfaceSelectDialog.h)
    ADD_DEFINITIONS (-DCONFIG_USE_PCAP)
ENDIF (CFG_KERNEL_STACK_DIRECTLINK OR CFG_KERNEL_STACK_USERSPACE_DAEMON)


INCLUDE (${QT_USE_FILE})
ADD_DEFINITIONS (${QT_DEFINITIONS})

INCLUDE_DIRECTORIES (${CMAKE_CURRENT_SOURCE_DIR}/include
                    ${CMAKE_CURRENT_BINARY_DIR}
                    )

ADD_DEFINITIONS(-DCONFIG_CFM -DCONFIG_OPENCONFIGURATOR_MAPPING -DCONFIG_MN)

IF (CFG_KERNEL_STACK_DIRECTLINK)
    INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/stack/make/lib/libpowerlink)
    UNSET (CFG_DEMO_MN_CONSOLE_USE_SYNCTHREAD CACHE)
    ADD_DEFINITIONS(-DCONFIG_KERNELSTACK_DIRECTLINK)
ELSE (CFG_KERNEL_STACK_DIRECTLINK)
    INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/stack/make/lib/libpowerlink_user)
    OPTION (CFG_DEMO_MN_CONSOLE_USE_SYNCTHREAD "Create separate thread for syncronous data exchange" ON)
    IF (CFG_DEMO_MN_CONSOLE_USE_SYNCTHREAD)
        ADD_DEFINITIONS(-DCONFIG_USE_SYNCTHREAD)
    ENDIF (CFG_DEMO_MN_CONSOLE_USE_SYNCTHREAD)
ENDIF (CFG_KERNEL_STACK_DIRECTLINK)

# Setup QT4 Wrappers
QT4_WRAP_CPP (DEMO_HEADERS_MOC ${DEMO_HEADERS})
QT4_WRAP_UI (DEMO_FORMS_HEADERS ${DEMO_FORMS})
QT4_ADD_RESOURCES (DEMO_RESOURCES_RCC ${DEMO_RESOURCES})

IF (UNIX)
    include(linux.cmake)
ELSEIF (WIN32)
    include (windows.cmake)
ENDIF (UNIX)

SOURCE_GROUP("Source Files" FILES ${DEMO_SOURCES})
SOURCE_GROUP("QT-generated Files" FILES ${DEMO_HEADERS_MOC} ${DEMO_RESOURCES_RCC})
SOURCE_GROUP("Header Files" FILES ${DEMO_HEADERS})

ADD_EXECUTABLE (demo_mn_qt ${DEMO_SOURCES} ${DEMO_ARCH_SOURCES} ${DEMO_HEADERS} ${DEMO_HEADERS_MOC}
                           ${DEMO_FORMS_HEADERS} ${DEMO_RESOURCES_RCC})
SET_PROPERTY(TARGET demo_mn_qt
             PROPERTY COMPILE_DEFINITIONS_DEBUG DEBUG;DEF_DEBUG_LVL=${CFG_DEBUG_LVL})

IF (CFG_KERNEL_STACK_DIRECTLINK)
    TARGET_LINK_LIBRARIES(demo_mn_qt powerlink)
ELSE (CFG_KERNEL_STACK_DIRECTLINK)
    TARGET_LINK_LIBRARIES(demo_mn_qt powerlink_user)
ENDIF (CFG_KERNEL_STACK_DIRECTLINK)

TARGET_LINK_LIBRARIES(demo_mn_qt ${ARCH_LIBRARIES} ${QT_LIBRARIES})

# add installation rules
INSTALL(TARGETS demo_mn_qt RUNTIME DESTINATION bin)
INSTALL(FILES mnobd.cdc DESTINATION bin)

