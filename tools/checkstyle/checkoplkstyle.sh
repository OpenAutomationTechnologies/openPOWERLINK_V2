#!/bin/bash
################################################################################
#
# \file  checkoplkstyle.sh
#
# \brief Check oplk formating style of all files in a commit
#
# Copyright (c) 2017, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
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
#
################################################################################

files=`git diff-tree --no-commit-id --diff-filter=AMCR --name-only --relative -r $TRAVIS_COMMIT | grep -E "\.[h|c]$"`
parameters="-P max-file-length=6000 -P max-line-length=160"

# Filter files and directories which should not be checked
for i in $files; do
    if [[ ! $i =~ ^contrib.* && ! $i =~ ^staging.* && ! $i =~ ^hardware.* && ! $i =~ ^unittests.*  && ! $i =~ .*xap\.h.* ]]; then
        filtered="$filtered $i"
    fi
done

if [[ $filtered != "" ]]; then
    tools/checkstyle/.vera++/vera++ --profile oplk $parameters -e -s $filtered
else
    echo Nothing to be checked!
fi
