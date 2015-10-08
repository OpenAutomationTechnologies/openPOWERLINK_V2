# ------------------------------------------------------------------------------
# SDC for TERASIC DE2I-150 development board SRAM
# -> SRAM: IS61WV25616BLL-10BLI
# -> Tri-state bridge wr/rd: 10/20 ns
# ------------------------------------------------------------------------------

proc timing_sram { clkSRAM } {
    # Virtual clock for SRAM generated with clkSRAM
    create_generated_clock -source ${clkSRAM} -name CLKSRAM_virt

    # Tri-state bridge timing
    set sram_tperWr     20
    set sram_tperRd     20

    # SRAM Address Access Time (tAA)
    set sram_taa       10.0

    # PCB delay
    set sram_tpcb       0.1

    # FPGA's IO timing
    # -> tco depends on FPGA (refer to www.altera.com IO spreadsheet)
    set fpga_tco        7.5
    set fpga_tsu        [expr $sram_tperRd - $sram_taa - $fpga_tco - 2*$sram_tpcb]
    set fpga_th         0.0
    set fpga_tcomin     0.0

    set delay_in_max    [expr $sram_tperRd - $fpga_tsu]
    set delay_in_min    $fpga_th
    set delay_out_max   [expr $sram_tperWr - $fpga_tco]
    set delay_out_min   $fpga_tcomin

    # -> TSU / TH
    set_input_delay -clock CLKSRAM_virt -max $delay_in_max [get_ports FS_DQ[*]]
    set_input_delay -clock CLKSRAM_virt -min $delay_in_min [get_ports FS_DQ[*]]
    # -> TCO
    set_output_delay -clock CLKSRAM_virt -max $delay_out_max [get_ports FS_DQ[*]]
    set_output_delay -clock CLKSRAM_virt -min $delay_out_min [get_ports FS_DQ[*]]
    # -> TCO
    set_output_delay -clock CLKSRAM_virt -max $delay_out_max [get_ports FS_ADDR[*]]
    set_output_delay -clock CLKSRAM_virt -min $delay_out_min [get_ports FS_ADDR[*]]
    # -> TCO
    set_output_delay -clock CLKSRAM_virt -max $delay_out_max [get_ports SSRAM_BE[*]]
    set_output_delay -clock CLKSRAM_virt -min $delay_out_min [get_ports SSRAM_BE[*]]
    # -> TCO
    set_output_delay -clock CLKSRAM_virt -max $delay_out_max [get_ports SSRAM_OE_N]
    set_output_delay -clock CLKSRAM_virt -min $delay_out_min [get_ports SSRAM_OE_N]
    # -> TCO
    set_output_delay -clock CLKSRAM_virt -max $delay_out_max [get_ports SSRAM_WE_N]
    set_output_delay -clock CLKSRAM_virt -min $delay_out_min [get_ports SSRAM_WE_N]

    # Consider multicycle due to Tri-state bridge and SRAM virtual clock
    # -> Read path
    set_multicycle_path -from [get_clocks CLKSRAM_virt] -to [get_clocks ${clkSRAM}] -setup -end 2
    set_multicycle_path -from [get_clocks CLKSRAM_virt] -to [get_clocks ${clkSRAM}] -hold -end 1
    # -> Write path
    set_multicycle_path -from [get_clocks ${clkSRAM}] -to [get_clocks CLKSRAM_virt] -setup -end 2
    set_multicycle_path -from [get_clocks ${clkSRAM}] -to [get_clocks CLKSRAM_virt] -hold -end 1

    return 0
}
