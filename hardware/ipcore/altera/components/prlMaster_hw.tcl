# -----------------------------------------------------------------------------
# prlMaster_hw.tcl
# -----------------------------------------------------------------------------
#
#    (c) B&R, 2014
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

# -----------------------------------------------------------------------------
# PACKAGES
# -----------------------------------------------------------------------------
# Insert local packages.
source "../../common/util/tcl/ipcoreUtil.tcl"
source "../../altera/components/tcl/qsysUtil.tcl"

# Use SOPC version 10.1
package require -exact sopc 10.1

# Use package ipcoreUtil for general functions...
package require ipcoreUtil 0.0.1

# Use package qsysUtil for Qsys helpers...
package require qsysUtil 0.0.1

# -----------------------------------------------------------------------------
# module
# -----------------------------------------------------------------------------
set_module_property NAME prlMaster
set_module_property VERSION 1.0.2
set_module_property INTERNAL false
set_module_property OPAQUE_ADDRESS_MAP true
set_module_property AUTHOR "B&R"
set_module_property DISPLAY_NAME "Parallel Address/Data-Bus Master"
set_module_property INSTANTIATE_IN_SYSTEM_MODULE true
set_module_property EDITABLE false
set_module_property VALIDATION_CALLBACK validation_callback
set_module_property ELABORATION_CALLBACK elaboration_callback
set_module_property ANALYZE_HDL false
set_module_property ICON_PATH "img/br.png"

# -----------------------------------------------------------------------------
# file sets
# -----------------------------------------------------------------------------
add_fileset             QUARTUS_SYNTH QUARTUS_SYNTH fileset_callback
set_fileset_property    QUARTUS_SYNTH TOP_LEVEL     prlMaster

# -----------------------------------------------------------------------------
# VHDL parameters
# -----------------------------------------------------------------------------
set hdlParamVisible FALSE

qsysUtil::addHdlParam  gEnableMux   NATURAL 0   $hdlParamVisible
qsysUtil::addHdlParam  gDataWidth   NATURAL 16  $hdlParamVisible
qsysUtil::addHdlParam  gAddrWidth   NATURAL 16  $hdlParamVisible
qsysUtil::addHdlParam  gAddrLow     NATURAL 1   $hdlParamVisible
qsysUtil::addHdlParam  gAdWidth     NATURAL 1   $hdlParamVisible

# -----------------------------------------------------------------------------
# System Info parameters
# -----------------------------------------------------------------------------
set sysParamVisible FALSE

qsysUtil::addSysParam sys_clk INTEGER 0 {CLOCK_RATE c0} $sysParamVisible

# -----------------------------------------------------------------------------
# GUI parameters
# -----------------------------------------------------------------------------
qsysUtil::addGuiParam  gui_enableMux BOOLEAN FALSE "Enable MUX Bus" "" ""
qsysUtil::addGuiParam  gui_dataWidth NATURAL 16 "Data width"    "Bits" "8 16 32"
qsysUtil::addGuiParam  gui_addrWidth NATURAL 16 "Address width" "Bits" "1:32"

# -----------------------------------------------------------------------------
# GUI configuration
# -----------------------------------------------------------------------------

# -----------------------------------------------------------------------------
# callbacks
# -----------------------------------------------------------------------------
proc validation_callback {} {
    # Get mux enable
    set muxEnable [get_parameter_value gui_enableMux]
    # Get configured width
    set dataBits  [get_parameter_value gui_dataWidth]
    set addrBits  [get_parameter_value gui_addrWidth]

    # Get data width in bytes
    set dataBytes [expr int( $dataBits / 8 )]

    # Get log2 of data bytes
    set dataBytesLog2 [expr int( log($dataBytes) / log(2) )]

    # Get max bits
    if { $dataBits > $addrBits } {
        set maxBits $dataBits
    } else {
        set maxBits $addrBits
    }

    if { $muxEnable } {
        # TRUE
        set enableMux 1
    } else {
        # FALSE
        set enableMux 0
    }

    # Assign HDL generics
    set_parameter_value gEnableMux  $enableMux
    set_parameter_value gDataWidth  $dataBits
    set_parameter_value gAddrWidth  $addrBits
    set_parameter_value gAddrLow    $dataBytesLog2
    set_parameter_value gAdWidth    $maxBits
}

proc elaboration_callback {} {
    # Get mux enable
    set muxEnable [get_parameter_value gui_enableMux]

    if { $muxEnable } {
        # TRUE
        set_port_property oPrlMst_ale       termination FALSE
        set_port_property oPrlMst_ad_o      termination FALSE
        set_port_property iPrlMst_ad_i      termination FALSE
        set_port_property oPrlMst_ad_oen    termination FALSE
        set_port_property oPrlMst_addr      termination TRUE
        set_port_property iPrlMst_data_i    termination TRUE
        set_port_property oPrlMst_data_o    termination TRUE
        set_port_property oPrlMst_data_oen  termination TRUE
    } else {
        # FALSE
        set_port_property oPrlMst_ale       termination TRUE
        set_port_property oPrlMst_ad_o      termination TRUE
        set_port_property iPrlMst_ad_i      termination TRUE
        set_port_property oPrlMst_ad_oen    termination TRUE
        set_port_property oPrlMst_addr      termination FALSE
        set_port_property iPrlMst_data_i    termination FALSE
        set_port_property oPrlMst_data_o    termination FALSE
        set_port_property oPrlMst_data_oen  termination FALSE
    }
}

proc fileset_callback { entityName } {
    send_message INFO "Generating entity $entityName"

    set clockFreqHz     [get_parameter_value sys_clk]
    set clockPeriodNs   [expr 1000000000 / $clockFreqHz]

    # Read sdc file
    set fp [open "sdc/prlMaster.sdc" r]
    set sdcFileIn [read $fp]

    # Process sdc file
    foreach line [split $sdcFileIn "\n"] {
        if { $line == "# SET INSTANCE NAME HERE #" } {
            append sdcFileOut "set INSTANCE_NAME   ${entityName}\n"
        } elseif { $line == "# SET CLOCK PERIOD HERE #" } {
            append sdcFileOut "set CLOCK_PERIOD    ${clockPeriodNs}\n"
        } else {
            append sdcFileOut "${line}\n"
        }
    }
    close $fp

    add_fileset_file "prlMaster-rtl-ea.vhd" VHDL PATH "../../common/parallelinterface/src/prlMaster-rtl-ea.vhd"
    add_fileset_file "${entityName}/prlMaster.sdc" SDC TEXT $sdcFileOut
}

# -----------------------------------------------------------------------------
# connection points
# -----------------------------------------------------------------------------
# connection point c0
add_interface c0 clock end
set_interface_property c0 clockRate 0
set_interface_property c0 ENABLED true

add_interface_port c0 iClk clk Input 1

# connection point r0
add_interface r0 reset end
set_interface_property r0 associatedClock c0
set_interface_property r0 synchronousEdges DEASSERT
set_interface_property r0 ENABLED true

add_interface_port r0 iRst reset Input 1

# connection point s0
add_interface s0 avalon end
set_interface_property s0 addressUnits WORDS
set_interface_property s0 associatedClock c0
set_interface_property s0 associatedReset r0
set_interface_property s0 bitsPerSymbol 8
set_interface_property s0 burstOnBurstBoundariesOnly false
set_interface_property s0 burstcountUnits WORDS
set_interface_property s0 explicitAddressSpan 0
set_interface_property s0 linewrapBursts false
set_interface_property s0 maximumPendingReadTransactions 0
set_interface_property s0 ENABLED true

add_interface_port s0 iSlv_address      address     Input   gAddrWidth-gAddrLow
add_interface_port s0 iSlv_read         read        Input   1
add_interface_port s0 oSlv_readdata     readdata    Output  gDataWidth
add_interface_port s0 iSlv_write        write       Input   1
add_interface_port s0 iSlv_writedata    writedata   Input   gDataWidth
add_interface_port s0 oSlv_waitrequest  waitrequest Output  1
add_interface_port s0 iSlv_byteenable   byteenable  Input   gDataWidth/8

# connection point prl0
add_interface prl0 conduit end
set_interface_property prl0 associatedClock c0
set_interface_property prl0 associatedReset r0
set_interface_property prl0 ENABLED true

add_interface_port prl0 oPrlMst_cs          export Output   1
add_interface_port prl0 iPrlMst_ad_i        export Input    gAdWidth
add_interface_port prl0 oPrlMst_ad_o        export Output   gAdWidth
add_interface_port prl0 oPrlMst_ad_oen      export Output   1
add_interface_port prl0 oPrlMst_addr        export Output   gAddrWidth
add_interface_port prl0 iPrlMst_data_i      export Input    gDataWidth
add_interface_port prl0 oPrlMst_data_o      export Output   gDataWidth
add_interface_port prl0 oPrlMst_data_oen    export Output   1
add_interface_port prl0 oPrlMst_be          export Output   gDataWidth/8
add_interface_port prl0 oPrlMst_ale         export Output   1
add_interface_port prl0 oPrlMst_wr          export Output   1
add_interface_port prl0 oPrlMst_rd          export Output   1
add_interface_port prl0 iPrlMst_ack         export Input    1
