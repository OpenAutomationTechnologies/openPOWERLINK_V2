################################################################################
#
# Directory list for stack cmake build system
#
# Copyright (c) 2016, B&R Industrial Automation GmbH
# Copyright (c) 2016, Franz Profelt (franz.profelt@gmail.com)
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
SET(STACK_SOURCE_DIR ${OPLK_STACK_DIR}/src)
SET(USER_SOURCE_DIR ${STACK_SOURCE_DIR}/user)
SET(KERNEL_SOURCE_DIR ${STACK_SOURCE_DIR}/kernel)
SET(COMMON_SOURCE_DIR ${STACK_SOURCE_DIR}/common)
SET(ARCH_SOURCE_DIR ${STACK_SOURCE_DIR}/arch)
SET(EDRV_SOURCE_DIR ${STACK_SOURCE_DIR}/kernel/edrv)
SET(CONTRIB_SOURCE_DIR ${OPLK_BASE_DIR}/contrib)
SET(SIM_SOURCE_DIR ${OPLK_BASE_DIR}/sim/src)
SET(SIM_INCLUDE_DIR ${OPLK_BASE_DIR}/sim/include)

# Include file directories
SET(STACK_INCLUDE_DIR ${OPLK_STACK_DIR}/include)

# Other directories
SET(TOOLS_DIR ${OPLK_BASE_DIR}/tools)
