# ------------------------------------------------------------------------------
# SDC for TERASIC DE2I-150 development board SSRAM
# -> SSRAM: IS61LPS51236A-200TQLI
# ------------------------------------------------------------------------------

proc timing_ssram { clkSSRAM } {
    # Virtual clock for SSRAM generated with SSRAM_CLK
    create_generated_clock -name clkSSRAM_virt -source [get_pins ${clkSSRAM}] [get_ports SSRAM_CLK]

    # Set maximum delay from PLL output to pin
    set_max_delay -from [get_pins ${clkSSRAM}] -to [get_ports SSRAM_CLK] 7.0
    set_min_delay -from [get_pins ${clkSSRAM}] -to [get_ports SSRAM_CLK] 0

    #The following I/O timing is noted from the perspective of the memory (http://www.issi.com/WW/pdf/61VPS_LPS-51236A_102418A.pdf)
    set Max_clock_to_out  3.1
    set Min_clock_to_out 1.5
    set Min_setup 1.5
    set Min_hold 0.4

    set delay_in_max    [expr $Max_clock_to_out + 0.6]
    set delay_in_min    [expr $Min_clock_to_out + 0.4]
    set delay_out_max   [expr $Min_setup + 0.6]
    set delay_out_min   [expr -1.0 + 0.4]

    # -> TSU / TH
    set_input_delay -clock clkSSRAM_virt -max $delay_in_max [get_ports FS_DQ[*]]
    set_input_delay -clock clkSSRAM_virt -min $delay_in_min [get_ports FS_DQ[*]]
    # -> TCO
    set_output_delay -clock clkSSRAM_virt -max $delay_out_max [get_ports FS_DQ[*]]
    set_output_delay -clock clkSSRAM_virt -min $delay_out_min [get_ports FS_DQ[*]]
    # -> TCO
    set_output_delay -clock clkSSRAM_virt -max $delay_out_max [get_ports FS_ADDR[*]]
    set_output_delay -clock clkSSRAM_virt -min $delay_out_min [get_ports FS_ADDR[*]]
    # -> TCO
    set_output_delay -clock clkSSRAM_virt -max $delay_out_max [get_ports SSRAM_BE[*]]
    set_output_delay -clock clkSSRAM_virt -min $delay_out_min [get_ports SSRAM_BE[*]]
    # -> TCO
    set_output_delay -clock clkSSRAM_virt -max $delay_out_max [get_ports SSRAM_OE_N[*]]
    set_output_delay -clock clkSSRAM_virt -min $delay_out_min [get_ports SSRAM_OE_N[*]]
    # -> TCO
    set_output_delay -clock clkSSRAM_virt -max $delay_out_max [get_ports SSRAM_WE_N[*]]
    set_output_delay -clock clkSSRAM_virt -min $delay_out_min [get_ports SSRAM_WE_N[*]]

    # SSRAM
    # -> Cut path
    set_false_path -from [get_registers *] -to [get_ports SSRAM0_CE_N[0]]
    set_false_path -from [get_registers *] -to [get_ports SSRAM_ADSC_N[0]]

    return 0
}
