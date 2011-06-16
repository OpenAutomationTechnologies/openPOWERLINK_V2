################################################################################
#
# Project: openPOWERLINK
#
# (c) Bernecker + Rainer Industrie-Elektronik Ges.m.b.H.
#     B&R Strasse 1, A-5142 Eggelsberg
#     www.br-automation.com
#
# Description: Main cmake file of openPOWERLINK stack
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

IF (WIN32)
    # ensure working support of VS.NET 2010
    CMAKE_MINIMUM_REQUIRED (VERSION 2.8.4)
ELSE (WIN32)
    CMAKE_MINIMUM_REQUIRED (VERSION 2.6)
ENDIF (WIN32)

PROJECT (openPOWERLINK C CXX)

IF(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
  SET(CMAKE_INSTALL_PREFIX
    ${CMAKE_BINARY_DIR} CACHE PATH "openPOWERLINK install prefix" FORCE
    )
ENDIF(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)

# include cmake modules
INCLUDE(CMakeDependentOption)

# Search for installed packages
FIND_PACKAGE(Qt4 QUIET)

###############################################################################
# Set options
###############################################################################
#SET(CMAKE_INSTALL_PREFIX ${CMAKE_BINARY_DIR}/bin)

SET(CFG_DEBUG_LVL "0xEC000000L" CACHE STRING "Debug Level for debug output")
OPTION (CFG_POWERLINK_CFM "Use POWERLINK configuration manager (CFM)" ON)
OPTION (CFG_POWERLINK_PROCESS_IMAGE "Use process image to transfer synchronous data between stack and application" ON)

CMAKE_DEPENDENT_OPTION (CFG_X86_DEMO_PROCESS_IMAGE_CONSOLE "Build process image based console demo application" ON
                        "CFG_POWERLINK_PROCESS_IMAGE" OFF)

IF (QT4_FOUND)	
    CMAKE_DEPENDENT_OPTION (CFG_X86_DEMO_PROCESS_IMAGE_QT "Build process image based QT demo application" OFF
                            "CFG_POWERLINK_PROCESS_IMAGE" OFF)
ENDIF (QT4_FOUND)

# additional Linux settings
IF (UNIX)
    SET(CFG_KERNEL_DIR "" CACHE PATH "Select the kernel directory to be used, if not specified, system kernel dir will be used!")
    SET(CFG_POWERLINK_EDRV "82573" CACHE STRING "Valid drivers are 8139,82573")
    OPTION (CFG_KERNEL_STACK "Use POWERLINK kernel based stack (if OFF, userspace library will be used!)" OFF)
    OPTION (CFG_X86_LINUX_DEMO_KERNEL "Build Linux kernel based demo application" OFF)
    CMAKE_DEPENDENT_OPTION (CFG_X86_LINUX_DEMO_QT "Build Linux QT demo application" OFF
                            "NOT CFG_POWERLINK_PROCESS_IMAGE;CFG_KERNEL_STACK" OFF)
ENDIF (UNIX)

# additional Windows settings
IF (WIN32)
    OPTION (CFG_X86_WINDOWS_DLL "Build Windows POWERLINK library as DLL" ON)
ENDIF (WIN32)

###############################################################################
# Set global definitions
###############################################################################

# set global include directories
INCLUDE_DIRECTORIES (
    ${openPOWERLINK_SOURCE_DIR}/Include
    ${openPOWERLINK_SOURCE_DIR}/SharedBuff
)

SET(POWERLINK_SOURCE_DIR ${CMAKE_SOURCE_DIR})
SET(POWERLINK_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/Include)
SET(EDRV_SOURCE_DIR ${CMAKE_SOURCE_DIR}/Edrv)

###############################################################################
# Examples
###############################################################################
# Add subdirectory of pcap based MN CFM library
IF (NOT CFG_KERNEL_STACK)
    ADD_SUBDIRECTORY(Examples/X86/Generic/powerlink_user_lib)
ENDIF (NOT CFG_KERNEL_STACK)

# Add subdirectory of process image based console demo application        
IF (CFG_X86_DEMO_PROCESS_IMAGE_CONSOLE)
    ADD_SUBDIRECTORY(Examples/X86/Generic/demo_process_image_console)
ENDIF (CFG_X86_DEMO_PROCESS_IMAGE_CONSOLE)

# Add subdirectory of process image based QT demo application
IF (CFG_X86_DEMO_PROCESS_IMAGE_QT)
    ADD_SUBDIRECTORY(Examples/X86/Generic/demo_process_image_qt)
ENDIF (CFG_X86_DEMO_PROCESS_IMAGE_QT)

# Linux specific part
IF(UNIX)
    # Add subdirectory of kernel module
    IF (CFG_KERNEL_STACK)
        ADD_SUBDIRECTORY(Examples/X86/Linux/gnu/powerlink_kernel_module)
    ENDIF (CFG_KERNEL_STACK)

    # Add subdirectory of kernel demo application
    IF (CFG_X86_LINUX_DEMO_KERNEL)
        ADD_SUBDIRECTORY(Examples/X86/Linux/gnu/demo_kernel)
    ENDIF (CFG_X86_LINUX_DEMO_KERNEL)

    # Add subdirectory of QT demo application
    IF (CFG_X86_LINUX_DEMO_QT)
        ADD_SUBDIRECTORY(Examples/X86/Linux/gnu/demo_qt)
    ENDIF (CFG_X86_LINUX_DEMO_QT)
ENDIF(UNIX)
