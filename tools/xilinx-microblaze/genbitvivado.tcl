################################################################################
#
# Generate the bitstream for Zynq Vivado design
#
# Copyright (c) 2016, Kalycito Infotech Private Limited
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

# Create project
set proj_name system
create_project $proj_name [lindex $argv 0]/system -part xc7z020clg484-1

# Set project properties
set obj [get_projects $proj_name]
set_property board_part xilinx.com:zc702:part0:1.2 [current_project]
set_property target_language VHDL [current_project]
set_property target_language VHDL [current_project]

# set the directory
set proj_dir [get_property directory $obj]
set_property  ip_repo_paths  $proj_dir/../../../../../ipcore/xilinx/components/ip/axi_openmac_v1_02_a [current_project]
update_ip_catalog

#create block design
source $proj_dir/../system_bd.tcl

#Validate the block design:
validate_bd_design

#Save block design:
save_bd_design

#Generate Block design:
generate_target all [get_files  $proj_dir/system.srcs/sources_1/bd/system/system.bd]

#Create HDL file:
make_wrapper -files [get_files $proj_dir/system.srcs/sources_1/bd/system/system.bd] -top

#Add HDL wrapper file:
add_files -norecurse $proj_dir/system.srcs/sources_1/bd/system/hdl/system_wrapper.vhd

#Add constraint
file mkdir $proj_dir/system.srcs/constrs_1
add_files -fileset constrs_1 -norecurse $proj_dir/../system.xdc

#Launch Synthesis:
launch_runs synth_1
wait_on_run synth_1

#Generate Bitstream
launch_runs impl_1 -to_step write_bitstream
wait_on_run impl_1

#Export Hardware design
file mkdir  $proj_dir/system.sdk
file copy -force [lindex $argv 0]/system/system.runs/impl_1/system_wrapper.sysdef [lindex $argv 0]/system/system.sdk/system_wrapper.hdf
