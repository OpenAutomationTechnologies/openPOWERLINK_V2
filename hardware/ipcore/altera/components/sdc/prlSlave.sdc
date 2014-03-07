# ------------------------------------------------------------------------------
# Parallel Slave Interface Timing Constraints
# ------------------------------------------------------------------------------

# SET INSTANCE NAME HERE #
# SET ENABLE ALE HERE #
# SET CLOCK PERIOD HERE #

# FPGA Timing
set t_clk               ${CLOCK_PERIOD}
set t_sd                8.0
set t_hd                0
set t_comax             10.0
set t_comin             0

# Relax virtual to internal clock
set relaxFactor         3

set input_delay_max     [expr $t_clk * $relaxFactor - $t_sd]
set input_delay_min     $t_hd
set output_delay_max    [expr $t_clk * $relaxFactor - $t_comax]
set output_delay_min    $t_comin

# Interface registers
set reg_writedata   *${INSTANCE_NAME}*writeDataRegister*
set reg_address     *${INSTANCE_NAME}*addressRegister*
set latch_address   *${INSTANCE_NAME}*addrLatch*
set reg_byteenable  *${INSTANCE_NAME}*byteenableRegister*
set reg_oen         *${INSTANCE_NAME}*hostDataEnable_reg*
set reg_readdata    *${INSTANCE_NAME}*readDataRegister*
set reg_cs          *${INSTANCE_NAME}*syncChipselect|metaReg[0]
set reg_wr          *${INSTANCE_NAME}*syncWrite|metaReg[0]
set reg_rd          *${INSTANCE_NAME}*syncRead|metaReg[0]
set reg_ack         *${INSTANCE_NAME}*hostAck_reg

# Input register collection
set in_regs     "\
                $reg_writedata $reg_byteenable \
                $reg_cs $reg_wr $reg_rd \
                "

# Output register collection
set out_regs    "\
                $reg_readdata $reg_oen \
                $reg_ack \
                "

if { $ENABLE_ALE } {
    # Multiplexed Address/Data Bus

    # ALE timing
    set t_ale_on            50.0
    set t_ale_off           140.0
    set t_ale_per           [expr $t_ale_on + $t_ale_off]

    set t_ale_sd            10.0
    set t_ale_hd            0

    set ale_input_max       [expr $t_ale_per - $t_ale_sd]
    set ale_input_min       $t_ale_hd

    # Get ALE port
    set port_ale ""
    foreach_in_collection fans [get_fanins -clock *addrLatch*] { set port_ale [get_node_info -name $fans] }

    # Create clock for LATCH ENABLE
    create_clock -name PARALLEL_SLAVE_ALE -period $t_ale_per -waveform "0 $t_ale_on" [get_ports $port_ale]

    # Input delay PORTS --> LATCH DATAIN
    set_max_delay -from [get_ports *] -to [get_registers $latch_address] $ale_input_max
    set_min_delay -from [get_ports *] -to [get_registers $latch_address] $ale_input_min

    # LATCH ENABLE is exclusive clock
    set_false_path -to [get_clocks PARALLEL_SLAVE_ALE] -from [get_clocks *]
    set_false_path -to [get_clocks *] -from [get_clocks PARALLEL_SLAVE_ALE]
} else {
    # Demultiplexed Address/Data Bus

    # Add address registers to interface register collection
    set in_regs "$in_regs $reg_address"
}

# SETUP / HOLD
set_max_delay -from [get_ports *] -to [get_registers $in_regs] $input_delay_max
set_min_delay -from [get_ports *] -to [get_registers $in_regs] $input_delay_min

# CLOCK-TO-OUTPUT
set_max_delay -from [get_registers $out_regs] -to [get_ports *] $output_delay_max
set_min_delay -from [get_registers $out_regs] -to [get_ports *] $output_delay_min
