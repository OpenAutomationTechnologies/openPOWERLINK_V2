################################################################################
#
# Generate hardware configuration files without bitstream for Zynq Vivado design
#
# Copyright (c) 2017, Kalycito Infotech Private Limited
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
# Getting the argument to set the path
if { $argc != 1 } {
        puts "The script requires 1 input."
        puts "Please try again."
        exit -1
    } else {
        #puts [expr [lindex $argv 0] + [lindex $argv 1]]
    }

# Local parameters
set proj_name system
set bd_source "${proj_name}/${proj_name}.srcs/sources_1/bd/${proj_name}"
set hdl_source hdl
set sdk_path "${proj_name}/${proj_name}.sdk"
set constrs_path "${proj_name}/${proj_name}.srcs/constrs_1"
set imp_path "${proj_name}/${proj_name}.runs/impl_1"

# create block design
source [lindex $argv 0]/${proj_name}_bd.tcl

# Validate the block design:
validate_bd_design

# Save block design:
save_bd_design

# Generate Block design:
generate_target all [get_files  [lindex $argv 0]/${bd_source}/${proj_name}.bd]

# Create HDL file:
make_wrapper -files [get_files [lindex $argv 0]/${bd_source}/${proj_name}.bd] -top

# Add HDL wrapper file:
add_files -norecurse [lindex $argv 0]/${bd_source}/${hdl_source}/${proj_name}_wrapper.vhd

# Add constraint
file mkdir [lindex $argv 0]/${constrs_path}
add_files -fileset constrs_1 -norecurse [lindex $argv 0]/${proj_name}.xdc

# Export Hardware design without bitstream
file mkdir  [lindex $argv 0]/${sdk_path}
write_hwdef -force  -file [lindex $argv 0]/${sdk_path}/${proj_name}_wrapper.hdf
