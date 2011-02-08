################################################################################
#
# Project: openPOWERLINK
#
# (c) Bernecker + Rainer Industrie-Elektronik Ges.m.b.H.
#     B&R Strasse 1, A-5142 Eggelsberg
#     www.br-automation.com
#
# Description: cmake file of CiA 302 MN QT demo application
#
# License:
#
#   Redistribution and use in source and binary forms, with or without
#   modification, are permitted provided that the following conditions
#   are met:
#
#   1. Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#   2. Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#   3. Neither the name of the copyright holders nor the names of its
#      contributors may be used to endorse or promote products derived
#      from this software without prior written permission. For written
#      permission, please contact office@br-automation.com.
#
#   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#   FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#   COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#   INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#   BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#   CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#   POSSIBILITY OF SUCH DAMAGE.
#
#   Severability Clause:
#
#       If a provision of this License is or becomes illegal, invalid or
#       unenforceable in any jurisdiction, that shall not affect:
#       1. the validity or enforceability in that jurisdiction of any other
#          provision of this License; or
#       2. the validity or enforceability in other jurisdictions of that or
#          any other provision of this License.
#
################################################################################

FIND_PACKAGE(Qt4 REQUIRED)

SET(KERNELMOD_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../powerlink_kernel_module)
SET(USERLIB_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../powerlink_user_lib)

SET(DEMO_SOURCES src/EplApi.cpp
                 src/EplCnState.cpp
                 src/EplDataInOutThread.cpp
                 src/EplInput.cpp
                 src/EplOutput.cpp
                 src/EplProcessThread.cpp
                 src/EplState.cpp
                 src/Leds.cpp
                 src/main.cpp
                 src/MainWindow.cpp
                 src/NodeState.cpp
                 ${POWERLINK_SOURCE_DIR}/ObjDicts/CiA302-4_MN/EplApiProcessImageSetup.c)
                         
                 
SET(DEMO_HEADERS include/EplCnState.h
                 include/EplDataInOutThread.h
                 include/EplInput.h
                 include/EplOutput.h
                 include/EplProcessThread.h
                 include/EplState.h
                 include/Leds.h
                 include/MainWindow.h
                 include/NodeState.h)

SET(DEMO_RESOURCES resources/Resources.qrc)

IF (CFG_KERNEL_STACK)

    SET(DEMO_SOURCES ${DEMO_SOURCES}
        ${POWERLINK_SOURCE_DIR}/EplStack/EplApiLinuxUser.c)
         
ELSE (CFG_KERNEL_STACK)

    SET(DEMO_SOURCES ${DEMO_SOURCES}
        src/InterfaceSelectDialog.cpp)
        
    SET(DEMO_HEADERS ${DEMO_HEADERS} include/InterfaceSelectDialog.h)      
        
ENDIF (CFG_KERNEL_STACK)                 

INCLUDE(${QT_USE_FILE})
ADD_DEFINITIONS(${QT_DEFINITIONS})

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/include
                    ${POWERLINK_INCLUDE_DIR}
                    ${CMAKE_CURRENT_BINARY_DIR}
                    )
                    
ADD_DEFINITIONS(-DCONFIG_CFM -DCONFIG_OPENCONFIGURATOR_MAPPING)                                       

# Configure depending on using usermode stack or kernel stack
IF (CFG_KERNEL_STACK)
    INCLUDE_DIRECTORIES(${KERNELMOD_INCLUDE_DIR})
    ADD_DEFINITIONS(-DCONFIG_POWERLINK_KERNELSTACK)
ELSE (CFG_KERNEL_STACK)
    INCLUDE_DIRECTORIES(${USERLIB_INCLUDE_DIR})
    ADD_DEFINITIONS(-DCONFIG_POWERLINK_USERSTACK) 
ENDIF (CFG_KERNEL_STACK)                 

# Setup QT4 Wrappers
QT4_WRAP_CPP(DEMO_HEADERS_MOC ${DEMO_HEADERS})
QT4_WRAP_UI(DEMO_FORMS_HEADERS ${DEMO_FORMS})
QT4_ADD_RESOURCES(DEMO_RESOURCES_RCC ${DEMO_RESOURCES})

ADD_EXECUTABLE(demo_pi_qt ${DEMO_SOURCES}
                    ${DEMO_HEADERS_MOC}
                    ${DEMO_FORMS_HEADERS}
                    ${DEMO_RESOURCES_RCC})
                                                              
TARGET_LINK_LIBRARIES(demo_pi_qt ${QT_LIBRARIES})
                                       
IF (NOT CFG_KERNEL_STACK)
    TARGET_LINK_LIBRARIES(demo_pi_qt powerlink pcap pthread rt)  
ENDIF (NOT CFG_KERNEL_STACK)  

# add installation rules
INSTALL(TARGETS demo_pi_qt RUNTIME DESTINATION bin)
INSTALL(FILES mnobd.cdc DESTINATION bin)
IF (NOT CFG_KERNEL_STACK)
    INSTALL(PROGRAMS ${CMAKE_CURRENT_SOURCE_DIR}/../tools/set_prio DESTINATION bin)
ENDIF (NOT CFG_KERNEL_STACK)



                                                          
                