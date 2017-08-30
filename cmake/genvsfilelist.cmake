################################################################################
#
# CMake macro for generating a properties file suitable for Visual Studio
# MSbuild projects.
#
# Copyright (c) 2015, Kalycito Infotech Private Limited
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
MACRO(GEN_VS_FILE_LIST IN_SRC_LIST RES_SRC_LIST)

    SET(TMP_RESULT "  <ItemGroup>\n")

    FOREACH(SRC_FILE IN ITEMS ${IN_SRC_LIST})
        GET_FILENAME_COMPONENT(SRC_NAME ${SRC_FILE} NAME)
        SET(TMP_RESULT "${TMP_RESULT}    <ClCompile Include=\"${SRC_FILE}\" />\n")
    ENDFOREACH()

    SET(TMP_RESULT "${TMP_RESULT}  </ItemGroup>\n")
    # Add to result list
    SET(${RES_SRC_LIST} ${TMP_RESULT})
    MESSAGE(STATUS "Source list generated")
ENDMACRO()
