################################################################################
#
# CMake macro for generating a file list suitable for a eclipse .project file
# file
#
# Copyright (c) 2014, B&R Industrial Automation GmbH
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
MACRO(GEN_ECLIPSE_FILE_LIST IN_SRC_LIST FOLDER_PREFIX RES_SRC_LIST)

    SET(TMP_RESULT "")

    FOREACH(SRC_FILE IN ITEMS ${IN_SRC_LIST})
        GET_FILENAME_COMPONENT(SRC_NAME ${SRC_FILE} NAME)
        IF("${FOLDER_PREFIX}" STREQUAL "")
            SET(TMP_RESULT "${TMP_RESULT}\t\t<link>\r\t\t\t<name>${SRC_NAME}</name>\r\t\t\t<type>1</type>\r\t\t\t<location>${SRC_FILE}</location>\r\t\t</link>\r")
        ELSE()
            SET(TMP_RESULT "${TMP_RESULT}\t\t<link>\r\t\t\t<name>${FOLDER_PREFIX}/${SRC_NAME}</name>\r\t\t\t<type>1</type>\r\t\t\t<location>${SRC_FILE}</location>\r\t\t</link>\r")
        ENDIF("${FOLDER_PREFIX}" STREQUAL "")
    ENDFOREACH()

    # Add to result list
    SET(${RES_SRC_LIST} ${TMP_RESULT})
ENDMACRO()