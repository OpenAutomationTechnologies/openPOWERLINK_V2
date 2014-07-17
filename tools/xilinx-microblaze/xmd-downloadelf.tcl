################################################################################
#
# Descripition:
#   Connects to mb, download the elf file and set Microblaze to run
#
# Copyright (c) 2014, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
# Copyright (c) 2014, Kalycito Infotech Pvt Ltd.
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

if { $argc != 2 } {
    #Exit on error
    puts "Warning: Too Many download script parameters!"

    #exit 1;
    if { $argc != 3 } {
     puts "Error: Invalid download script parameters!"
     exit 1;
    }
}

if { $argc == 3 } {

    set enVerify [lindex $argv 2]

    # Establish connection to debug module
    # 0- app 1- drv
    connect mb mdm -debugdevice cpunr 1

    # Read name of executeable from script arguments
    puts "[lindex $argv 1] is downloading....."
    set executeable "[lindex $argv 1]"
    dow $executeable

    if { $enVerify } {
        puts "INFO: Verify elf download. This can take a few minutes!"
        elf_verify
    }
    # Reset system before run
    debugconfig -reset_on_run processor enable
    run
    disconnect 0

    #############################################
    # Establish connection to debug module
    connect mb mdm -debugdevice cpunr 2

    # Read name of executeable from script arguments
    puts "[lindex $argv 0] is downloading....."
    set executeable "[lindex $argv 0]"
    dow $executeable

    if { $enVerify } {
        puts "INFO: Verify elf download. This can take a few minutes!"
        elf_verify
    }

    # Reset system before run
    debugconfig -reset_on_run processor enable
    run
    disconnect 1
    exit;
} else {

set enVerify [lindex $argv 1]

# Establish connection to debug module
connect mb mdm

# Read name of executeable from script arguments
set executeable "[lindex $argv 0]"
dow $executeable

if { $enVerify } {
    puts "INFO: Verify elf download. This can take a few minutes!"
    elf_verify
}

# Reset system before run
debugconfig -reset_on_run system enable

run

disconnect 0
exit;
}
