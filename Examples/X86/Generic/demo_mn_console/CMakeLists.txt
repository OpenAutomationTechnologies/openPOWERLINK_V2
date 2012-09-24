################################################################################
#
# Project: openPOWERLINK
#
# (c) Bernecker + Rainer Industrie-Elektronik Ges.m.b.H.
#     B&R Strasse 1, A-5142 Eggelsberg
#     www.br-automation.com
#
# Description: cmake file of CiA 302 MN console demo application
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

#
# additional compiler flags
#
IF (UNIX)
    ADD_DEFINITIONS(-Wall -Wextra -pedantic -std=c99 -pthread -D_GNU_SOURCE -D_POSIX_C_SOURCE=200112L)
ELSEIF (WIN32)
    ADD_DEFINITIONS(-D_CONSOLE -DWPCAP -DHAVE_REMOTE)
ENDIF ()

SET(OBJDICT CiA302-4_MN)

# additional include directories
SET(USERLIB_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../powerlink_user_lib)
IF (UNIX)
    SET(KERNELMOD_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../../X86/Linux/gnu/powerlink_kernel_module)
ENDIF (UNIX)

#
# Source include directories
#
INCLUDE_DIRECTORIES(
    ${POWERLINK_SOURCE_DIR}/ObjDicts/${OBJDICT}
)

# set sources of POWERLINK application
SET (DEMO_SOURCES demo_main.c
                  ${POWERLINK_SOURCE_DIR}/ObjDicts/${OBJDICT}/EplApiProcessImageSetup.c
                  ${POWERLINK_SOURCE_DIR}/EplStack/EplTgtConio.c
)

# The TRACE macros need trace.c on the Windows platform
IF (WIN32)
    SET(DEMO_SOURCES ${DEMO_SOURCES}
                     ${POWERLINK_SOURCE_DIR}/EplStack/trace.c
       )
ENDIF (WIN32)

IF (UNIX)
    IF (CFG_KERNEL_STACK)
        SET(DEMO_SOURCES ${DEMO_SOURCES}
                         ${POWERLINK_SOURCE_DIR}/EplStack/EplDebug.c
                         ${POWERLINK_SOURCE_DIR}/EplStack/EplApiLinuxUser.c
            )
        INCLUDE_DIRECTORIES(${KERNELMOD_INCLUDE_DIR})
        ADD_DEFINITIONS(-DCONFIG_POWERLINK_KERNELSTACK)
    ELSE (CFG_KERNEL_STACK)
        INCLUDE_DIRECTORIES(${USERLIB_INCLUDE_DIR})
        ADD_DEFINITIONS(-DCONFIG_POWERLINK_USERSTACK)
    ENDIF (CFG_KERNEL_STACK) 	
ELSE (UNIX)
    ADD_DEFINITIONS(-DCONFIG_POWERLINK_USERSTACK)
    INCLUDE_DIRECTORIES(${USERLIB_INCLUDE_DIR})
ENDIF (UNIX)


INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR})

IF (WIN32)
    INCLUDE_DIRECTORIES(${POWERLINK_SOURCE_DIR}/Target/X86/Windows/WpdPack/Include)

    IF (CMAKE_CL_64)
        LINK_DIRECTORIES(${POWERLINK_SOURCE_DIR}/Target/X86/Windows/WpdPack/Lib/x64)
    ELSE (CMAKE_CL_64)
        LINK_DIRECTORIES(${POWERLINK_SOURCE_DIR}/Target/X86/Windows/WpdPack/Lib)
    ENDIF (CMAKE_CL_64)
ENDIF (WIN32)


ADD_EXECUTABLE(demo_mn_console ${DEMO_SOURCES})

SET_PROPERTY(TARGET demo_mn_console
             PROPERTY COMPILE_DEFINITIONS_DEBUG DEBUG;DEF_DEBUG_LVL=${CFG_DEBUG_LVL})

IF (UNIX)
    IF (NOT CFG_KERNEL_STACK)
        TARGET_LINK_LIBRARIES(demo_mn_console powerlink pcap pthread rt)
    ELSE (NOT CFG_KERNEL_STACK)
        TARGET_LINK_LIBRARIES(demo_mn_console pthread rt)
    ENDIF (NOT CFG_KERNEL_STACK)
ELSEIF (WIN32)
    TARGET_LINK_LIBRARIES(demo_mn_console openPOWERLINK wpcap iphlpapi)
ENDIF ()

# add installation rules
INSTALL(TARGETS demo_mn_console RUNTIME DESTINATION bin)
INSTALL(FILES mnobd.cdc DESTINATION bin)

IF (UNIX)
    IF (NOT CFG_KERNEL_STACK)
        INSTALL(PROGRAMS ${CMAKE_CURRENT_SOURCE_DIR}/../../Linux/gnu/tools/set_prio DESTINATION bin)
    ENDIF (NOT CFG_KERNEL_STACK)
ENDIF (UNIX)

SOURCE_GROUP("Demo Application" FILES ${DEMO_SOURCES})
