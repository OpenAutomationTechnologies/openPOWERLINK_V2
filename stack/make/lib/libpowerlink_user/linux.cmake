################################################################################
#
# Linux CMake configuration for openPOWERLINK user stack library
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


ADD_DEFINITIONS(-Wall -Wextra -pedantic -std=c99 -pthread -D_GNU_SOURCE -D_POSIX_C_SOURCE=200112L
                -fno-strict-aliasing)

SET(LIB_ARCH_SOURCES 
     ${USER_SOURCE_DIR}/sdo/sdo-udpu.c
     ${COMMON_SOURCE_DIR}/timer/timer-linuxuser.c
     ${ARCH_SOURCE_DIR}/linux/ftrace-debug.c
     ${ARCH_SOURCE_DIR}/linux/target-linux.c
     ${LIB_SOURCE_DIR}/trace/trace-printf.c
     )

IF (CFG_KERNEL_STACK_KERNEL_MODULE)
ADD_DEFINITIONS(-DCONFIG_USE_KERNEL_MODULE)
SET (LIB_ARCH_SOURCES
     ${LIB_ARCH_SOURCES}
     ${USER_SOURCE_DIR}/pdo/pdoucalsync-ioctl.c
     ${USER_SOURCE_DIR}/pdo/pdoucalmem-linuxmmap.c
     ${USER_SOURCE_DIR}/dll/dllucal-ioctl.c
     ${USER_SOURCE_DIR}/ctrl/ctrlucal-ioctl.c
     ${USER_SOURCE_DIR}/event/eventucal-linuxioctl.c
     ${USER_SOURCE_DIR}/errhnd/errhnducal-ioctl.c
     )
ELSE (CFG_KERNEL_STACK_KERNEL_MODULE)
SET (LIB_ARCH_SOURCES
     ${LIB_ARCH_SOURCES}
     ${LIB_SOURCE_DIR}/circbuf/circbuf-posixshm.c
     ${USER_SOURCE_DIR}/pdo/pdoucalmem-posixshm.c
     ${USER_SOURCE_DIR}/pdo/pdoucalsync-bsdsem.c
     ${USER_SOURCE_DIR}/dll/dllucal-circbuf.c
     ${USER_SOURCE_DIR}/ctrl/ctrlucal-mem.c
     ${COMMON_SOURCE_DIR}/ctrl/ctrlcal-posixshm.c
     ${USER_SOURCE_DIR}/event/eventucal-linux.c
     ${USER_SOURCE_DIR}/event/eventucalintf-circbuf.c
     ${USER_SOURCE_DIR}/errhnd/errhnducal-posixshm.c
     )
ENDIF(CFG_KERNEL_STACK_KERNEL_MODULE)


