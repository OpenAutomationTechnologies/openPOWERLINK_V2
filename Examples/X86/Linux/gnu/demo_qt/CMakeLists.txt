################################################################################
#
# Project: openPOWERLINK
#
# (c) Bernecker + Rainer Industrie-Elektronik Ges.m.b.H.
#     B&R Strasse 1, A-5142 Eggelsberg
#     www.br-automation.com
#
# Description: cmake file of QT demo application
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

IF (NOT CFG_KERNEL_STACK)
        MESSAGE(WARNING "This demo does only work with the kernel base stack!")
ENDIF (NOT CFG_KERNEL_STACK)    
                       

SET(DEMO_SOURCES src/Circles.cpp
                 src/EplApi.cpp
                 src/EplDataInOutThread.cpp
                 src/EplProcessThread.cpp
                 src/EplState.cpp
                 src/Leds.cpp
                 src/main.cpp
                 src/MainWindow.cpp
                 src/Slides.cpp
        ${POWERLINK_SOURCE_DIR}/EplStack/EplApiLinuxUser.c) 
                 
SET(DEMO_HEADERS include/Circles.h
                 include/EplDataInOutThread.h
                 include/EplProcessThread.h
                 include/EplState.h
                 include/Leds.h
                 include/MainWindow.h
                 include/Slides.h)
                               
SET(DEMO_RESOURCES src/Resources.qrc)

INCLUDE(${QT_USE_FILE})
ADD_DEFINITIONS(${QT_DEFINITIONS})

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/include
                    ${POWERLINK_INCLUDE_DIR}
                    ${CMAKE_CURRENT_BINARY_DIR}
                    ${KERNELMOD_INCLUDE_DIR}
                    )
                    
ADD_DEFINITIONS(-DCONFIG_POWERLINK_KERNELSTACK)
                    
                    #
# Configure depending selected mode
#
IF(CFG_POWERLINK_CFM)

    ADD_DEFINITIONS(-DCONFIG_CFM)                            

    IF (CFG_POWERLINK_PROCESS_IMAGE)
    
        MESSAGE(WARNING "This demo does not using process image! "
                        "Please use demo_process_image_qt for process image demo application!")
    
    ENDIF(CFG_POWERLINK_PROCESS_IMAGE)
    
ENDIF(CFG_POWERLINK_CFM)
                    
QT4_WRAP_CPP(DEMO_HEADERS_MOC ${DEMO_HEADERS})
QT4_ADD_RESOURCES(DEMO_RESOURCES_RCC ${DEMO_RESOURCES})

ADD_EXECUTABLE(demo_qt ${DEMO_SOURCES}
                    ${DEMO_HEADERS_MOC}
                    ${DEMO_RESOURCES_RCC})
                                                              
TARGET_LINK_LIBRARIES(demo_qt ${QT_LIBRARIES})
                                       
# add installation rules
INSTALL(TARGETS demo_qt RUNTIME DESTINATION bin)
INSTALL(FILES mnobd.cdc DESTINATION bin)
                                                                 
                