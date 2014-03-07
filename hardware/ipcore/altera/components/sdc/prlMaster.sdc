# ------------------------------------------------------------------------------
# Parallel Master Interface Timing Constraints
# ------------------------------------------------------------------------------

# SET INSTANCE NAME HERE #
# SET CLOCK PERIOD HERE #

# FPGA Timing
set t_clk               ${CLOCK_PERIOD}
set t_sd                8.0
set t_hd                0
set t_comax             10.0
set t_comin             0

# Relax virtual to internal clock
set relaxFactor         3
set multiSetupEnd       $relaxFactor
set multiHoldEnd        [expr $relaxFactor - 1]

set input_delay_max     [expr $t_clk * $relaxFactor - $t_sd]
set input_delay_min     $t_hd
set output_delay_max    [expr $t_clk * $relaxFactor - $t_comax]
set output_delay_min    $t_comin

# Interface registers
set reg_record  *${INSTANCE_NAME}*reg.*
set reg_syncAck *${INSTANCE_NAME}*syncAck|metaReg[0]

# Input register collection
set in_regs     "$reg_record $reg_syncAck"

# Output register collection
set out_regs    "$reg_record "

# SETUP / HOLD
set_max_delay -from [get_ports *] -to [get_registers $in_regs] $input_delay_max
set_min_delay -from [get_ports *] -to [get_registers $in_regs] $input_delay_min

# CLOCK-TO-OUTPUT
set_max_delay -from [get_registers $out_regs] -to [get_ports *] $output_delay_max
set_min_delay -from [get_registers $out_regs] -to [get_ports *] $output_delay_min
