################################################################################
#
# CMake macro for installing the bsp for Microblaze
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

MACRO(INSTALL_BSP BSP_SOURCE_DIR BSP_TARGET_DIR BSP_CPU_NAME)
    GET_FILENAME_COMPONENT(BSP_TARGET_NAME ${BSP_SOURCE_DIR} NAME )
    SET(BSP_CPU_NAME ${BSP_CPU_NAME})
    SET(CFG_CPU_NAME ${BSP_CPU_NAME})

    # Copy hardware platform eclipse project file
    CONFIGURE_FILE(${ARCH_TOOLS_DIR}/eclipse/bspcproject.in ${BSP_SOURCE_DIR} COPY_ONLY)
    CONFIGURE_FILE(${ARCH_TOOLS_DIR}/eclipse/bspproject.in ${BSP_SOURCE_DIR} COPY_ONLY)
    CONFIGURE_FILE(${ARCH_TOOLS_DIR}/eclipse/bspsdkproject.in ${BSP_SOURCE_DIR} @ONLY)
    CONFIGURE_FILE(${ARCH_TOOLS_DIR}/eclipse/bsplibgen.options.in ${BSP_SOURCE_DIR} @ONLY)

    INSTALL(DIRECTORY ${BSP_SOURCE_DIR}
            DESTINATION ${BSP_TARGET_DIR}
            PATTERN "*.in" EXCLUDE
           )

    INSTALL(FILES ${BSP_SOURCE_DIR}/bspcproject.in
            DESTINATION ${BSP_TARGET_DIR}/${BSP_TARGET_NAME} RENAME .cproject
           )
    INSTALL(FILES ${BSP_SOURCE_DIR}/bspproject.in
            DESTINATION ${BSP_TARGET_DIR}/${BSP_TARGET_NAME} RENAME .project
           )
    INSTALL(FILES ${BSP_SOURCE_DIR}/bspsdkproject.in
            DESTINATION ${BSP_TARGET_DIR}/${BSP_TARGET_NAME} RENAME .sdkproject
           )
    INSTALL(FILES ${BSP_SOURCE_DIR}/bsplibgen.options.in
            DESTINATION ${BSP_TARGET_DIR}/${BSP_TARGET_NAME} RENAME libgen.options
           )
    INSTALL(FILES ${ARCH_TOOLS_DIR}/eclipse/bspmakefile.in
            DESTINATION ${BSP_TARGET_DIR}/${BSP_TARGET_NAME} RENAME Makefile
           )
ENDMACRO()
