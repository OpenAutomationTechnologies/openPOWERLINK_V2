################################################################################
#
# Directory list for stack cmake build system
#
# Copyright (c) 2014, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
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

# Source directories
SET(STACK_SOURCE_DIR ${OPLK_BASE_DIR}/stack/src)
SET(USER_SOURCE_DIR ${OPLK_BASE_DIR}/stack/src/user)
SET(KERNEL_SOURCE_DIR ${OPLK_BASE_DIR}/stack/src/kernel)
SET(COMMON_SOURCE_DIR ${OPLK_BASE_DIR}/stack/src/common)
SET(ARCH_SOURCE_DIR ${OPLK_BASE_DIR}/stack/src/arch)
SET(EDRV_SOURCE_DIR ${OPLK_BASE_DIR}/stack/src/kernel/edrv)
SET(CONTRIB_SOURCE_DIR ${OPLK_BASE_DIR}/contrib)

# Include file directories
SET(OPLK_INCLUDE_DIR ${OPLK_BASE_DIR}/include)
SET(STACK_INCLUDE_DIR ${OPLK_BASE_DIR}/stack/include)
SET(USER_STACK_INCLUDE_DIR ${OPLK_BASE_DIR}/stack/include/user)
SET(KERNEL_STACK_INCLUDE_DIR ${OPLK_BASE_DIR}/stack/include/kernel)

# Other directories
SET(OBJDICT_DIR ${OPLK_BASE_DIR}/objdicts)
