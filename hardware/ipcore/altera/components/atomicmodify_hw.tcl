# -----------------------------------------------------------------------------
# atomicmodify_hw.tcl
# -----------------------------------------------------------------------------
#
#    (c) B&R Industrial Automation GmbH, 2015
#
#    Redistribution and use in source and binary forms, with or without
#    modification, are permitted provided that the following conditions
#    are met:
#
#    1. Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#
#    2. Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#
#    3. Neither the name of B&R nor the names of its
#       contributors may be used to endorse or promote products derived
#       from this software without prior written permission. For written
#       permission, please contact office@br-automation.com
#
#    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#    COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#    POSSIBILITY OF SUCH DAMAGE.
#
# -----------------------------------------------------------------------------

package require -exact sopc 10.1

# -----------------------------------------------------------------------------
# module
# -----------------------------------------------------------------------------
set_module_property NAME atomicmodify
set_module_property VERSION 1.1.0
set_module_property INTERNAL false
set_module_property AUTHOR "B&R"
set_module_property DISPLAY_NAME "Atomic Modify"
set_module_property TOP_LEVEL_HDL_FILE "../../altera/atomicmodify/src/alteraAtomicmodifyRtl.vhd"
set_module_property TOP_LEVEL_HDL_MODULE alteraAtomicmodify
set_module_property INSTANTIATE_IN_SYSTEM_MODULE true
set_module_property EDITABLE false
set_module_property ELABORATION_CALLBACK elaboration_callback
set_module_property ANALYZE_HDL false

# -----------------------------------------------------------------------------
# file sets
# -----------------------------------------------------------------------------
add_file "../../altera/atomicmodify/src/alteraAtomicmodifyRtl.vhd" {SYNTHESIS SIMULATION}
add_file "../../common/atomicmodify/src/atomicmodifyRtl.vhd" {SYNTHESIS SIMULATION}

# -----------------------------------------------------------------------------
# VHDL parameters
# -----------------------------------------------------------------------------
add_parameter           gAddrWidth  NATURAL
set_parameter_property  gAddrWidth  DEFAULT_VALUE       16
set_parameter_property  gAddrWidth  TYPE                NATURAL
set_parameter_property  gAddrWidth  DERIVED             true
set_parameter_property  gAddrWidth  HDL_PARAMETER       true
set_parameter_property  gAddrWidth  AFFECTS_GENERATION  true
set_parameter_property  gAddrWidth  VISIBLE             false

# -----------------------------------------------------------------------------
# System Info parameters
# -----------------------------------------------------------------------------
add_parameter           sys_m0Addrw NATURAL             10
set_parameter_property  sys_m0Addrw SYSTEM_INFO         {ADDRESS_WIDTH m0}
set_parameter_property  sys_m0Addrw DERIVED             true
set_parameter_property  sys_m0Addrw VISIBLE             false

# -----------------------------------------------------------------------------
# GUI parameters
# -----------------------------------------------------------------------------

# -----------------------------------------------------------------------------
# GUI configuration
# -----------------------------------------------------------------------------

# -----------------------------------------------------------------------------
# callbacks
# -----------------------------------------------------------------------------
proc elaboration_callback {} {
    # -------------------------------------------------------------------------
    # Forward VHDL Generics
    set_parameter_value gAddrWidth  [get_parameter_value sys_m0Addrw]
}

# -----------------------------------------------------------------------------
# connection points
# -----------------------------------------------------------------------------
# connection point r0
add_interface r0 reset end
set_interface_property r0 associatedClock c0
set_interface_property r0 synchronousEdges DEASSERT
set_interface_property r0 ENABLED true

add_interface_port r0 rsi_r0_reset reset Input 1

# connection point c0
add_interface c0 clock end
set_interface_property c0 clockRate 0
set_interface_property c0 ENABLED true

add_interface_port c0 csi_c0_clock clk Input 1

# connection point s0
add_interface s0 avalon end
set_interface_property s0 addressAlignment DYNAMIC
set_interface_property s0 addressUnits WORDS
set_interface_property s0 associatedClock c0
set_interface_property s0 associatedReset r0
set_interface_property s0 burstOnBurstBoundariesOnly false
set_interface_property s0 explicitAddressSpan 0
set_interface_property s0 holdTime 0
set_interface_property s0 isMemoryDevice false
set_interface_property s0 isNonVolatileStorage false
set_interface_property s0 linewrapBursts false
set_interface_property s0 maximumPendingReadTransactions 0
set_interface_property s0 printableDevice false
set_interface_property s0 readLatency 0
set_interface_property s0 readWaitTime 1
set_interface_property s0 setupTime 0
set_interface_property s0 timingUnits Cycles
set_interface_property s0 writeWaitTime 0
set_interface_property s0 ENABLED true

add_interface_port s0 avs_s0_address address Input (gAddrWidth-2)
add_interface_port s0 avs_s0_byteenable byteenable Input 4
add_interface_port s0 avs_s0_write write Input 1
add_interface_port s0 avs_s0_read read Input 1
add_interface_port s0 avs_s0_writedata writedata Input 32
add_interface_port s0 avs_s0_readdata readdata Output 32
add_interface_port s0 avs_s0_waitrequest waitrequest Output 1

# connection point m0
add_interface m0 avalon start
set_interface_property m0 associatedClock c0
set_interface_property m0 associatedReset r0
set_interface_property m0 burstOnBurstBoundariesOnly false
set_interface_property m0 linewrapBursts false
set_interface_property m0 ENABLED true

add_interface_port m0 avm_m0_address address Output gAddrWidth
add_interface_port m0 avm_m0_byteenable byteenable Output 4
add_interface_port m0 avm_m0_write write Output 1
add_interface_port m0 avm_m0_writedata writedata Output 32
add_interface_port m0 avm_m0_read read Output 1
add_interface_port m0 avm_m0_readdata readdata Input 32
add_interface_port m0 avm_m0_waitrequest waitrequest Input 1
add_interface_port m0 avm_m0_lock lock Output 1
