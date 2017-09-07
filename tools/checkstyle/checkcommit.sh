#!/bin/bash
################################################################################
#
# \file  checkcommit.sh
#
# \brief A shell script to verify a commit by executing some checks on it
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

set -e

#
# verify commit message line length
#
len=`git log -n 1 --format=%B $GIT_COMMIT | awk '{ if (length > L) {L=length} }END{ print L}'`
if [[ "$len" -gt "72" ]]; then
	echo "ERROR: Maximum line length of commit message ($len) is bigger than 72 characters!"
	exit 1
fi

#
# Verify if first line begins with [FIX], [TASK] or [FEATURE]
#
tag=`git log -n 1 --format=%B $GIT_COMMIT | awk -F" " 'NR == 1 { print $1 }'`

if [[ "$tag" != "[MERGE]"  && "$tag" != "[FIX]"  && "$tag" != "[TASK]" && "$tag" != "[FEATURE]" ]]; then
	echo "ERROR: Commit message does not begin with tag [MERGE] or [FIX] or [TASK] or [FEATURE]!"
	exit 1
fi

echo "git commit message successfully checked!"

exit 0