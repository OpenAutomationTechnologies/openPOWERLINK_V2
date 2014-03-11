# ------------------------------------------------------------------------------
# SDC for TERASIC DE2-115 SDRAM
# -> SDRAM: IS42S16320B-7TL
# -> fCLK = 100 MHz | CAS latency = 2
# ------------------------------------------------------------------------------

proc timing_sdram { clkSDRAM clkMEMCTRL } {
    # FPGA to SDRAM delay [ns]
    set t_pcb   0.5

    # SDRAM timing from data sheet (CAS latency = 2)
    # tAC2  : Access Time From CLK [ns]
    set t_ac    6.0
    # tOH2  : Output Data Hold Time [ns]
    set t_oh    2.7
    # tDS   : Input Data Setup Time [ns]
    set t_ds    1.5
    # tDH   : Input Data Hold Time [ns]
    set t_dh    0.8

    # Create generated clock seen by the SDRAM:
    # - Use the clock that is driven to the outside.
    # - Add pcb delay
    create_generated_clock -name CLKSDRAM_pin -source [get_pins ${clkSDRAM}] [get_ports SDRAM_CLK]

    # Set maximum delay from PLL output to pin
    set_max_delay -from [get_pins ${clkSDRAM}] -to [get_ports SDRAM_CLK] 1.0
    set_min_delay -from [get_pins ${clkSDRAM}] -to [get_ports SDRAM_CLK] 0

    # Max pcb routing
    set t_pcb_min       [expr $t_pcb - 0.1 ]
    # Min pcb routing
    set t_pcb_max       [expr $t_pcb + 0.1 ]

    # Calculate input delay
    set in_delay_max    [expr $t_ac + $t_pcb_max ]
    set in_delay_min    [expr $t_oh + $t_pcb_min ]

    # Calculate output delay
    set out_delay_max   [expr $t_ds + $t_pcb_max ]
    set out_delay_min   [expr 0 - $t_dh + $t_pcb_min ]

    # TSU / TH
    set_input_delay -clock CLKSDRAM_pin -max $in_delay_max [get_ports SDRAM_DQ[*]]
    set_input_delay -clock CLKSDRAM_pin -min $in_delay_min [get_ports SDRAM_DQ[*]]

    # TCO / TH
    set_output_delay -clock CLKSDRAM_pin -max $out_delay_max [get_ports SDRAM_DQ[*]]
    set_output_delay -clock CLKSDRAM_pin -min $out_delay_min [get_ports SDRAM_DQ[*]]

    # TCO / TH
    set_output_delay -clock CLKSDRAM_pin -max $out_delay_max [get_ports SDRAM_ADDR[*]]
    set_output_delay -clock CLKSDRAM_pin -min $out_delay_min [get_ports SDRAM_ADDR[*]]

    # TCO / TH
    set_output_delay -clock CLKSDRAM_pin -max $out_delay_max [get_ports SDRAM_BA[*]]
    set_output_delay -clock CLKSDRAM_pin -min $out_delay_min [get_ports SDRAM_BA[*]]

    # TCO / TH
    set_output_delay -clock CLKSDRAM_pin -max $out_delay_max [get_ports SDRAM_DQM[*]]
    set_output_delay -clock CLKSDRAM_pin -min $out_delay_min [get_ports SDRAM_DQM[*]]

    # TCO / TH
    set_output_delay -clock CLKSDRAM_pin -max $out_delay_max [get_ports SDRAM_CAS_n]
    set_output_delay -clock CLKSDRAM_pin -min $out_delay_min [get_ports SDRAM_CAS_n]

    # TCO / TH
    set_output_delay -clock CLKSDRAM_pin -max $out_delay_max [get_ports SDRAM_RAS_n]
    set_output_delay -clock CLKSDRAM_pin -min $out_delay_min [get_ports SDRAM_RAS_n]

    # TCO / TH
    set_output_delay -clock CLKSDRAM_pin -max $out_delay_max [get_ports SDRAM_WE_n]
    set_output_delay -clock CLKSDRAM_pin -min $out_delay_min [get_ports SDRAM_WE_n]

    # TCO / TH
    set_output_delay -clock CLKSDRAM_pin -max $out_delay_max [get_ports SDRAM_CS_n]
    set_output_delay -clock CLKSDRAM_pin -min $out_delay_min [get_ports SDRAM_CS_n]

    # Skip timing for CKE
    set_false_path -from [get_registers *] -to [get_ports SDRAM_CKE]

    # Set multicycle exception
    set_multicycle_path -from [get_clocks CLKSDRAM_pin] -to [get_clocks ${clkMEMCTRL}] -setup -end 2

    return 0
}
