# (C) 2001-2014 Altera Corporation. All rights reserved.
# Your use of Altera Corporation's design tools, logic functions and other 
# software and tools, and its AMPP partner logic functions, and any output 
# files any of the foregoing (including device programming or simulation 
# files), and any associated documentation or information are expressly subject 
# to the terms and conditions of the Altera Program License Subscription 
# Agreement, Altera MegaCore Function License Agreement, or other applicable 
# license agreement, including, without limitation, that your use is for the 
# sole purpose of programming logic devices manufactured by Altera and sold by 
# Altera or its authorized distributors.  Please refer to the applicable 
# agreement for further details.



package require -exact sopc 11.0

proc createRange { min max } {
    for { set i $min } { $i < [expr $max + 1] } {incr i} { 
	lappend toReturn $i
    }
    return ${toReturn}
}

# +-----------------------------------
# | module direct_window_bridge
# | 
set_module_property DESCRIPTION "Creates a windowed bridge to allow masters to access a larger address map."
set_module_property NAME altera_address_span_extender
set_module_property VERSION 14.0
set_module_property GROUP "Basic Functions/Bridges and Adaptors/Memory Mapped"
set_module_property AUTHOR "Altera Corporation"
set_module_property DISPLAY_NAME "Address Span Extender KN"
set_module_property TOP_LEVEL_HDL_FILE altera_address_span_extender.sv
set_module_property EDITABLE false
set_module_property ANALYZE_HDL false
set_module_property ELABORATION_CALLBACK elaborate
set_module_property SIMULATION_MODEL_IN_VHDL true
set_module_property HIDE_FROM_QUARTUS true
# | 
# +-----------------------------------

# +-----------------------------------
# |filesets
# |
add_fileset QUARTUS_SYNTH QUARTUS_SYNTH generate_synth
set_fileset_property QUARTUS_SYNTH TOP_LEVEL altera_address_span_extender

add_fileset SIM_VERILOG SIM_VERILOG generate_sim_ver
set_fileset_property SIM_VERILOG TOP_LEVEL altera_address_span_extender

add_fileset SIM_VHDL SIM_VHDL generate_sim_vhd
set_fileset_property SIM_VHDL TOP_LEVEL altera_address_span_extender
# | 
# +-----------------------------------

# +-----------------------------------
# | generate callbacks
# | 
proc generate_synth {output_name} {
        add_fileset_file altera_address_span_extender.sv SYSTEM_VERILOG PATH altera_address_span_extender.sv
}

proc generate_sim_ver {output_name} {
        add_fileset_file altera_address_span_extender.sv SYSTEM_VERILOG PATH altera_address_span_extender.sv 
}

proc generate_sim_vhd {output_name} {
        if { (1 == 1) && [file exist "./mentor/altera_address_span_extender.sv"] } {
            add_fileset_file ./mentor/altera_address_span_extender.sv SYSTEM_VERILOG_ENCRYPT PATH ./mentor/altera_address_span_extender.sv {MENTOR_SPECIFIC}
            add_fileset_file altera_address_span_extender.sv SYSTEM_VERILOG PATH altera_address_span_extender.sv {ALDEC_SPECIFIC CADENCE_SPECIFIC SYNOPYSYS_SPECIFIC} 
        } else {
            add_fileset_file altera_address_span_extender.sv SYSTEM_VERILOG PATH altera_address_span_extender.sv 
        }

}

# | 
# +-----------------------------------

# +-----------------------------------
# | parameters
# | 
add_parameter DATA_WIDTH INTEGER 32
set_parameter_property DATA_WIDTH DISPLAY_NAME "Datapath Width"
set_parameter_property DATA_WIDTH DESCRIPTION "Width of the slave and master data path."
set_parameter_property DATA_WIDTH HDL_PARAMETER true
set_parameter_property DATA_WIDTH ALLOWED_RANGES "8 16 32 64 128 256 512 1024"
set_parameter_property DATA_WIDTH DISPLAY_UNITS "bits"
set_parameter_property DATA_WIDTH GROUP "Data Path Properties"

add_parameter BYTEENABLE_WIDTH INTEGER 4
set_parameter_property BYTEENABLE_WIDTH DISPLAY_NAME "Byteenable Width"
set_parameter_property BYTEENABLE_WIDTH DESCRIPTION "Width of the slave and master byteenable port."
set_parameter_property BYTEENABLE_WIDTH HDL_PARAMETER true
set_parameter_property BYTEENABLE_WIDTH DERIVED true
set_parameter_property BYTEENABLE_WIDTH DISPLAY_UNITS "bits"
set_parameter_property BYTEENABLE_WIDTH GROUP "Data Path Properties"

add_parameter DATA_WORD_WIDTH INTEGER 4
set_parameter_property DATA_WORD_WIDTH DISPLAY_NAME "Data Word Width"
set_parameter_property DATA_WORD_WIDTH DESCRIPTION "Number of bytes in the data word."
set_parameter_property DATA_WORD_WIDTH DERIVED true
set_parameter_property DATA_WORD_WIDTH DISPLAY_UNITS "bytes"
set_parameter_property DATA_WORD_WIDTH GROUP "Data Path Properties"

add_parameter MASTER_ADDRESS_WIDTH INTEGER 32
set_parameter_property MASTER_ADDRESS_WIDTH DISPLAY_NAME "Expanded Master Byte Address Width"
set_parameter_property MASTER_ADDRESS_WIDTH DESCRIPTION "Width of the master byte address port."
set_parameter_property MASTER_ADDRESS_WIDTH HDL_PARAMETER true
set_parameter_property MASTER_ADDRESS_WIDTH ALLOWED_RANGES [ createRange 3 64 ]
set_parameter_property MASTER_ADDRESS_WIDTH DISPLAY_UNITS "bits"
set_parameter_property MASTER_ADDRESS_WIDTH GROUP "Address Properties"

add_parameter MASTER_ADDRESS_SPAN STRING ""
set_parameter_property MASTER_ADDRESS_SPAN DISPLAY_NAME "Expanded Master Address Span"
set_parameter_property MASTER_ADDRESS_SPAN DESCRIPTION "Byte address span of master interface."
set_parameter_property MASTER_ADDRESS_SPAN DERIVED true
set_parameter_property MASTER_ADDRESS_SPAN GROUP "Address Properties"

add_parameter SLAVE_ADDRESS_WIDTH INTEGER 16
set_parameter_property SLAVE_ADDRESS_WIDTH DISPLAY_NAME "Slave Word Address Width"
set_parameter_property SLAVE_ADDRESS_WIDTH DESCRIPTION "Width of the slave word address port."
set_parameter_property SLAVE_ADDRESS_WIDTH HDL_PARAMETER true
set_parameter_property SLAVE_ADDRESS_WIDTH ALLOWED_RANGES [ createRange 1 62 ]
set_parameter_property SLAVE_ADDRESS_WIDTH DISPLAY_UNITS "bits"
set_parameter_property SLAVE_ADDRESS_WIDTH GROUP "Address Properties"

add_parameter SLAVE_ADDRESS_SPAN STRING ""
set_parameter_property SLAVE_ADDRESS_SPAN DISPLAY_NAME "Slave Address Span"
set_parameter_property SLAVE_ADDRESS_SPAN DESCRIPTION "Byte address span of slave interface."
set_parameter_property SLAVE_ADDRESS_SPAN DERIVED true
set_parameter_property SLAVE_ADDRESS_SPAN GROUP "Address Properties"

add_parameter SLAVE_ADDRESS_SHIFT INTEGER 2
set_parameter_property SLAVE_ADDRESS_SHIFT DISPLAY_NAME "Slave Address Shift"
set_parameter_property SLAVE_ADDRESS_SHIFT DESCRIPTION "Number of address bits represented by the slave word width."
set_parameter_property SLAVE_ADDRESS_SHIFT HDL_PARAMETER true
set_parameter_property SLAVE_ADDRESS_SHIFT DERIVED true
set_parameter_property SLAVE_ADDRESS_SHIFT DISPLAY_UNITS "bits"
set_parameter_property SLAVE_ADDRESS_SHIFT GROUP "Address Properties"

add_parameter BURSTCOUNT_WIDTH INTEGER 1
set_parameter_property BURSTCOUNT_WIDTH DISPLAY_NAME "Burstcount Width"
set_parameter_property BURSTCOUNT_WIDTH DESCRIPTION "Width of the slave and master burstcount port."
set_parameter_property BURSTCOUNT_WIDTH HDL_PARAMETER true
set_parameter_property BURSTCOUNT_WIDTH ALLOWED_RANGES [ createRange 1 20 ]
set_parameter_property BURSTCOUNT_WIDTH DISPLAY_UNITS "bits"
set_parameter_property BURSTCOUNT_WIDTH GROUP "Burst Properties"

add_parameter MAX_BURST_WORDS INTEGER 32
set_parameter_property MAX_BURST_WORDS DISPLAY_NAME "Maximum burst"
set_parameter_property MAX_BURST_WORDS DESCRIPTION "Maximum words that a single burst can signal."
set_parameter_property MAX_BURST_WORDS DERIVED true
set_parameter_property MAX_BURST_WORDS DISPLAY_UNITS "words"
set_parameter_property MAX_BURST_WORDS GROUP "Burst Properties"

add_parameter MAX_BURST_BYTES INTEGER 32
set_parameter_property MAX_BURST_BYTES DISPLAY_NAME "Maximum burst"
set_parameter_property MAX_BURST_BYTES DESCRIPTION "Maximum bytes that a single burst can signal."
set_parameter_property MAX_BURST_BYTES DERIVED true
set_parameter_property MAX_BURST_BYTES DISPLAY_UNITS "bytes"
set_parameter_property MAX_BURST_BYTES GROUP "Burst Properties"

add_parameter CNTL_ADDRESS_WIDTH INTEGER 1
set_parameter_property CNTL_ADDRESS_WIDTH DISPLAY_NAME "Control slave address width"
set_parameter_property CNTL_ADDRESS_WIDTH DESCRIPTION "Width of the control slave address port."
set_parameter_property CNTL_ADDRESS_WIDTH HDL_PARAMETER true
set_parameter_property CNTL_ADDRESS_WIDTH DERIVED true
set_parameter_property CNTL_ADDRESS_WIDTH DISPLAY_UNITS "bits"
set_parameter_property CNTL_ADDRESS_WIDTH GROUP "Control Slave Properties"


add_parameter SUB_WINDOW_COUNT INTEGER 1
set_parameter_property SUB_WINDOW_COUNT DISPLAY_NAME "Number of sub-windows"
set_parameter_property SUB_WINDOW_COUNT DESCRIPTION "The slave port can represent 1 or more windows into the master address span."
set_parameter_property SUB_WINDOW_COUNT HDL_PARAMETER true
set_parameter_property SUB_WINDOW_COUNT ALLOWED_RANGES "1 2 4 8 16 32 64"
set_parameter_property SUB_WINDOW_COUNT DISPLAY_UNITS "sub-windows"
set_parameter_property SUB_WINDOW_COUNT GROUP "Control Slave Properties"

add_parameter MASTER_ADDRESS_DEF STD_LOGIC_VECTOR 0
set_parameter_property MASTER_ADDRESS_DEF WIDTH "64"
set_parameter_property MASTER_ADDRESS_DEF DISPLAY_NAME "Reset Default for Master Window"
set_parameter_property MASTER_ADDRESS_DEF DESCRIPTION "Reset default value for MASTER_WINDOW"
set_parameter_property MASTER_ADDRESS_DEF HDL_PARAMETER true
set_parameter_property MASTER_ADDRESS_DEF DISPLAY_HINT hexadecimal
set_parameter_property MASTER_ADDRESS_DEF DISPLAY_UNITS "address"
set_parameter_property MASTER_ADDRESS_DEF GROUP "Control Slave Properties"

add_parameter TERMINATE_SLAVE_PORT BOOLEAN false
set_parameter_property TERMINATE_SLAVE_PORT DISPLAY_NAME "Disable Slave Control Port"
set_parameter_property TERMINATE_SLAVE_PORT DESCRIPTION  "MASTER_WINDOW set by parameter"
set_parameter_property TERMINATE_SLAVE_PORT ALLOWED_RANGES "true,false"
set_parameter_property TERMINATE_SLAVE_PORT GROUP "Control Slave Properties"

add_parameter SUB_WINDOW_SPAN STRING ""
set_parameter_property SUB_WINDOW_SPAN DISPLAY_NAME "Sub window span"
set_parameter_property SUB_WINDOW_SPAN DESCRIPTION "Byte address span of a single sub window."
set_parameter_property SUB_WINDOW_SPAN DERIVED true
set_parameter_property SUB_WINDOW_SPAN GROUP "Control Slave Properties"

add_parameter MAX_PENDING_READS INTEGER 1
set_parameter_property MAX_PENDING_READS DISPLAY_NAME "Maximum Pending Reads"
set_parameter_property MAX_PENDING_READS DESCRIPTION "Sets the bridge slave's maximumPendingReadTransactions property, in certain system configurations this value should be increased to improve performance."
set_parameter_property MAX_PENDING_READS ALLOWED_RANGES "1 2 4 8 16 32 64"
set_parameter_property MAX_PENDING_READS DISPLAY_UNITS "read pending"
set_parameter_property MAX_PENDING_READS GROUP "Bridge Slave Properties"

# | 
# +-----------------------------------

# +-----------------------------------
# | display items
# | 
# | 
# +-----------------------------------

# +-----------------------------------
# | connection point clock
# | 
add_interface clock clock end
set_interface_property clock clockRate 0


add_interface_port clock clk clk Input 1
# | 
# +-----------------------------------

# +-----------------------------------
# | connection point reset
# | 
add_interface reset reset end
set_interface_property reset associatedClock clock
set_interface_property reset synchronousEdges DEASSERT


add_interface_port reset reset reset Input 1
# | 
# +-----------------------------------

# +-----------------------------------
# | connection point windowed_slave
# | 
add_interface windowed_slave avalon end
#declare NOT bridge by default.
set_interface_property windowed_slave bridgesToMaster ""
set_interface_property windowed_slave addressUnits WORDS
set_interface_property windowed_slave associatedClock clock
set_interface_property windowed_slave associatedReset reset
set_interface_property windowed_slave bitsPerSymbol 8
set_interface_property windowed_slave burstOnBurstBoundariesOnly false
set_interface_property windowed_slave burstcountUnits WORDS
set_interface_property windowed_slave explicitAddressSpan 0
set_interface_property windowed_slave linewrapBursts false
set_interface_property windowed_slave maximumPendingReadTransactions 1
set_interface_property windowed_slave readWaitTime 1
set_interface_property windowed_slave timingUnits Cycles


add_interface_port windowed_slave avs_s0_address address Input SLAVE_ADDRESS_WIDTH
add_interface_port windowed_slave avs_s0_read read Input 1
add_interface_port windowed_slave avs_s0_readdata readdata Output DATA_WIDTH
add_interface_port windowed_slave avs_s0_write write Input 1
add_interface_port windowed_slave avs_s0_writedata writedata Input DATA_WIDTH
add_interface_port windowed_slave avs_s0_readdatavalid readdatavalid Output 1
add_interface_port windowed_slave avs_s0_waitrequest waitrequest Output 1
add_interface_port windowed_slave avs_s0_byteenable byteenable Input BYTEENABLE_WIDTH
add_interface_port windowed_slave avs_s0_burstcount burstcount Input BURSTCOUNT_WIDTH
# | 
# +-----------------------------------

# +-----------------------------------
# | connection point expanded_master
# | 
add_interface expanded_master avalon start
set_interface_property expanded_master addressUnits SYMBOLS
set_interface_property expanded_master associatedClock clock
set_interface_property expanded_master associatedReset reset
set_interface_property expanded_master bitsPerSymbol 8
set_interface_property expanded_master burstOnBurstBoundariesOnly false
set_interface_property expanded_master burstcountUnits WORDS
set_interface_property expanded_master doStreamReads false
set_interface_property expanded_master doStreamWrites false
set_interface_property expanded_master holdTime 0
set_interface_property expanded_master linewrapBursts false
set_interface_property expanded_master maximumPendingReadTransactions 0
set_interface_property expanded_master readLatency 0
set_interface_property expanded_master readWaitTime 1
set_interface_property expanded_master setupTime 0
set_interface_property expanded_master timingUnits Cycles
set_interface_property expanded_master writeWaitTime 0


add_interface_port expanded_master avm_m0_address address Output MASTER_ADDRESS_WIDTH
add_interface_port expanded_master avm_m0_read read Output 1
add_interface_port expanded_master avm_m0_waitrequest waitrequest Input 1
add_interface_port expanded_master avm_m0_readdata readdata Input DATA_WIDTH
add_interface_port expanded_master avm_m0_write write Output 1
add_interface_port expanded_master avm_m0_writedata writedata Output DATA_WIDTH
add_interface_port expanded_master avm_m0_readdatavalid readdatavalid Input 1
add_interface_port expanded_master avm_m0_byteenable byteenable Output BYTEENABLE_WIDTH
add_interface_port expanded_master avm_m0_burstcount burstcount Output BURSTCOUNT_WIDTH
# | 
# +-----------------------------------

# +-----------------------------------
# | connection point cntl
# | 
add_interface cntl avalon end
set_interface_property cntl addressUnits WORDS
set_interface_property cntl associatedClock clock
set_interface_property cntl associatedReset reset
set_interface_property cntl bitsPerSymbol 8
set_interface_property cntl burstOnBurstBoundariesOnly false
set_interface_property cntl burstcountUnits WORDS
set_interface_property cntl explicitAddressSpan 0
set_interface_property cntl holdTime 0
set_interface_property cntl linewrapBursts false
set_interface_property cntl maximumPendingReadTransactions 0
set_interface_property cntl readLatency 1
set_interface_property cntl timingUnits Cycles


add_interface_port cntl avs_cntl_address address Input CNTL_ADDRESS_WIDTH
add_interface_port cntl avs_cntl_read read Input 1
add_interface_port cntl avs_cntl_readdata readdata Output 64
add_interface_port cntl avs_cntl_write write Input 1
add_interface_port cntl avs_cntl_writedata writedata Input 64
add_interface_port cntl avs_cntl_byteenable byteenable Input 8
# | 
# +-----------------------------------

proc elaborate {}  {

#
#   Configure the bridge slave properties
#
    set_interface_property windowed_slave maximumPendingReadTransactions [ get_parameter_value MAX_PENDING_READS ]

#
#   Calculate the data path widths
#
    set byteenable_width_var [ expr [ get_parameter_value DATA_WIDTH ] / 8 ]

    set_parameter_value BYTEENABLE_WIDTH $byteenable_width_var

    set_parameter_value DATA_WORD_WIDTH $byteenable_width_var

#
#   Validate the master and slave address widths and the burstcount width
#
    set slave_address_shift_var [ expr int( log( $byteenable_width_var ) / log( 2 ) ) ]
    set_parameter_value SLAVE_ADDRESS_SHIFT $slave_address_shift_var
    
    set master_address_width_var [ get_parameter_value MASTER_ADDRESS_WIDTH ]
    set slave_address_width_var [ get_parameter_value SLAVE_ADDRESS_WIDTH ]

    set slave_byte_address_width_var [ expr $slave_address_width_var + $slave_address_shift_var ]

    if { $slave_byte_address_width_var >= $master_address_width_var } {
        send_message Error "Slave address span cannot equal or exceed the master address span.  Please increase the master address span or reduce the slave address span."
    }
    
    if { [ expr int( log( [ get_parameter_value SUB_WINDOW_COUNT ] ) / log( 2 ) ) ] > $slave_address_width_var } {
        send_message Error "Sub-window count cannot exceed the span created by the slave address width.  Please increase the slave address span or reduce the sub-window count."
    }

    if { [ get_parameter_value BURSTCOUNT_WIDTH ] > [ expr $slave_address_width_var + 1 ] } {
        send_message Error "burstcount width cannot exceed the slave address width + 1.  Please increase the slave address span or reduce the burstcount width."
    }

#
#   Calculate the cntl address width and deal with the cntl_address port
#
    set sub_window_count_var [ get_parameter_value SUB_WINDOW_COUNT ]
    set cntl_address_width_var [ expr int( log( $sub_window_count_var ) / log( 2 ) ) ]

    set_port_property avs_cntl_address TERMINATION_VALUE 0
    set_port_property avs_cntl_address TERMINATION false

    if { $cntl_address_width_var == 0 } {
        set cntl_address_width_var 1
        set_port_property avs_cntl_address TERMINATION true
    }

    set_parameter_value CNTL_ADDRESS_WIDTH $cntl_address_width_var

#
#   If Master Add set only by parameter, then disable the slave control port
#
    if { [ get_parameter_value TERMINATE_SLAVE_PORT ] } {
       set_port_property avs_cntl_address    termination true
       set_port_property avs_cntl_address    termination_value 0
       set_port_property avs_cntl_read       termination true
       set_port_property avs_cntl_read       termination_value 0
       set_port_property avs_cntl_readdata   termination true
       #no termination value needed for output readdata
       set_port_property avs_cntl_write      termination true
       set_port_property avs_cntl_write      termination_value 0
       set_port_property avs_cntl_writedata  termination true
       set_port_property avs_cntl_writedata  termination_value 0
       set_port_property avs_cntl_byteenable termination true
       set_port_property avs_cntl_byteenable termination_value 0
          if { [get_parameter_value SUB_WINDOW_COUNT] > 1} {
             send_message Warning "If using more than one sub_window, slave control interface needed to set window fields."
          }
#treat as a bridge if window is fixed
       set_interface_property windowed_slave bridgesToMaster expanded_master
       set_interface_property windowed_slave bridgedAddressOffset [get_parameter_value MASTER_ADDRESS_DEF]

    } else {
#window is not fixed, don't treat as bridge
       set_interface_property windowed_slave bridgesToMaster ""
       set_interface_property windowed_slave bridgedAddressOffset ""
    }

#
#   Calculate the master address span
#
    set span_var 0
    set byteString ""
    
    if { $master_address_width_var >= 60 } {
        set span_var [ expr $master_address_width_var - 60 ]
    } elseif { $master_address_width_var >= 50 } {
        set span_var [ expr $master_address_width_var - 50 ]
    } elseif { $master_address_width_var >= 40 } {
        set span_var [ expr $master_address_width_var - 40 ]
    } elseif { $master_address_width_var >= 30 } {
        set span_var [ expr $master_address_width_var - 30 ]
    } elseif { $master_address_width_var >= 20 } {
        set span_var [ expr $master_address_width_var - 20 ]
    } elseif { $master_address_width_var >= 10 } {
        set span_var [ expr $master_address_width_var - 10 ]
    } else {
        set span_var [ expr $master_address_width_var - 00 ]
    }
    
    if { $master_address_width_var >= 60 } {
        set byteString "exabytes"
    } elseif { $master_address_width_var >= 50 } {
        set byteString "petabytes"
    } elseif { $master_address_width_var >= 40 } {
        set byteString "terabytes"
    } elseif { $master_address_width_var >= 30 } {
        set byteString "gigabytes"
    } elseif { $master_address_width_var >= 20 } {
        set byteString "megabytes"
    } elseif { $master_address_width_var >= 10 } {
        set byteString "kilobytes"
    } else {
        set byteString "bytes"
    }

    set_parameter_value MASTER_ADDRESS_SPAN [ format "%d %s" [ expr 1 << $span_var ] $byteString ]
    
#
#   Calculate the slave address span
#
    if { $slave_byte_address_width_var >= 60 } {
        set span_var [ expr $slave_byte_address_width_var - 60 ]
    } elseif { $slave_byte_address_width_var >= 50 } {
        set span_var [ expr $slave_byte_address_width_var - 50 ]
    } elseif { $slave_byte_address_width_var >= 40 } {
        set span_var [ expr $slave_byte_address_width_var - 40 ]
    } elseif { $slave_byte_address_width_var >= 30 } {
        set span_var [ expr $slave_byte_address_width_var - 30 ]
    } elseif { $slave_byte_address_width_var >= 20 } {
        set span_var [ expr $slave_byte_address_width_var - 20 ]
    } elseif { $slave_byte_address_width_var >= 10 } {
        set span_var [ expr $slave_byte_address_width_var - 10 ]
    } else {
        set span_var [ expr $slave_byte_address_width_var - 00 ]
    }
    
    if { $slave_byte_address_width_var >= 60 } {
        set byteString "exabytes"
    } elseif { $slave_byte_address_width_var >= 50 } {
        set byteString "petabytes"
    } elseif { $slave_byte_address_width_var >= 40 } {
        set byteString "terabytes"
    } elseif { $slave_byte_address_width_var >= 30 } {
        set byteString "gigabytes"
    } elseif { $slave_byte_address_width_var >= 20 } {
        set byteString "megabytes"
    } elseif { $slave_byte_address_width_var >= 10 } {
        set byteString "kilobytes"
    } else {
        set byteString "bytes"
    }

    set_parameter_value SLAVE_ADDRESS_SPAN  [ format "%d %s" [ expr 1 << $span_var ] $byteString ]
    
#
#   Calculate the sub window span
#
    set sub_window_span_var [ expr $slave_byte_address_width_var - int( log( $sub_window_count_var ) / log( 2 ) ) ]
    
    if { $sub_window_span_var >= 60 } {
        set span_var [ expr $sub_window_span_var - 60 ]
    } elseif { $sub_window_span_var >= 50 } {
        set span_var [ expr $sub_window_span_var - 50 ]
    } elseif { $sub_window_span_var >= 40 } {
        set span_var [ expr $sub_window_span_var - 40 ]
    } elseif { $sub_window_span_var >= 30 } {
        set span_var [ expr $sub_window_span_var - 30 ]
    } elseif { $sub_window_span_var >= 20 } {
        set span_var [ expr $sub_window_span_var - 20 ]
    } elseif { $sub_window_span_var >= 10 } {
        set span_var [ expr $sub_window_span_var - 10 ]
    } else {
        set span_var [ expr $sub_window_span_var - 00 ]
    }
    
    if { $sub_window_span_var >= 60 } {
        set byteString "exabytes"
    } elseif { $sub_window_span_var >= 50 } {
        set byteString "petabytes"
    } elseif { $sub_window_span_var >= 40 } {
        set byteString "terabytes"
    } elseif { $sub_window_span_var >= 30 } {
        set byteString "gigabytes"
    } elseif { $sub_window_span_var >= 20 } {
        set byteString "megabytes"
    } elseif { $sub_window_span_var >= 10 } {
        set byteString "kilobytes"
    } else {
        set byteString "bytes"
    }

    set_parameter_value SUB_WINDOW_SPAN  [ format "%d %s" [ expr 1 << $span_var ] $byteString ]

#
#   Calculate the maximum burst properties
#
    set_parameter_value MAX_BURST_WORDS [ expr pow( 2, [ get_parameter_value BURSTCOUNT_WIDTH ] - 1 ) ]
    set_parameter_value MAX_BURST_BYTES [ expr [ get_parameter_value MAX_BURST_WORDS ] * $byteenable_width_var ]

#
#   Configure C macros for software developers
#
    set_module_assignment embeddedsw.CMacro.DATA_WIDTH              [ get_parameter_value DATA_WIDTH ]
    set_module_assignment embeddedsw.CMacro.BYTEENABLE_WIDTH        [ get_parameter_value BYTEENABLE_WIDTH ]
    set_module_assignment embeddedsw.CMacro.MASTER_ADDRESS_WIDTH    [ get_parameter_value MASTER_ADDRESS_WIDTH ]
    set_module_assignment embeddedsw.CMacro.SLAVE_ADDRESS_WIDTH     [ get_parameter_value SLAVE_ADDRESS_WIDTH ]
    set_module_assignment embeddedsw.CMacro.SLAVE_ADDRESS_SHIFT     [ get_parameter_value SLAVE_ADDRESS_SHIFT ]
    set_module_assignment embeddedsw.CMacro.BURSTCOUNT_WIDTH        [ get_parameter_value BURSTCOUNT_WIDTH ]
    set_module_assignment embeddedsw.CMacro.MAX_BURST_WORDS         [ get_parameter_value MAX_BURST_WORDS ]
    set_module_assignment embeddedsw.CMacro.MAX_BURST_BYTES         [ get_parameter_value MAX_BURST_BYTES ]
    set_module_assignment embeddedsw.CMacro.CNTL_ADDRESS_WIDTH      [ get_parameter_value CNTL_ADDRESS_WIDTH ]
    set_module_assignment embeddedsw.CMacro.SUB_WINDOW_COUNT        [ get_parameter_value SUB_WINDOW_COUNT ]
    
    #
    # this assignment allows this component to appear as a memory target in the Nios II linker script configuration GUI
    # it makes the memory map of our slave look like a valid linker target memory region
    # <<to be revisited in 13.0>>
    set_interface_assignment windowed_slave embeddedsw.configuration.isMemoryDevice 1

    #
    # this assignment allows the macros of components attached to our master to appear in the Nios II system.h header
    # <<to be revisited in 13.0>>
    set_interface_assignment windowed_slave embeddedsw.configuration.affectsTransactionsOnMasters "expanded_master"

}
