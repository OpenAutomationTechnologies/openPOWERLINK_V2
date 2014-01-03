# SDC file for INK DE2-115 evaluation board
# - SRAM (10 ns - IS61WV102416BLL)
# - Nios II (PCP) with 100 MHz

# ----------------------------------------------------------------------------------
# clock definitions
## define the clocks in your design (depends on your PLL settings!)
##  (under "Compilation Report" - "TimeQuest Timing Analyzer" - "Clocks")
set ext_clk         EXT_CLK

set clk50           pllInst|altpll_component|auto_generated|pll1|clk[0]
set clk100          pllInst|altpll_component|auto_generated|pll1|clk[1]
set clk25           pllInst|altpll_component|auto_generated|pll1|clk[2]

## define which clock drives SRAM controller
set clkSRAM         $clk100
# ----------------------------------------------------------------------------------

# ----------------------------------------------------------------------------------
# note: changes below this line have to be done carefully

# ----------------------------------------------------------------------------------
# constrain JTAG
set_clock_groups -asynchronous -group {altera_reserved_tck}
set_input_delay -clock {altera_reserved_tck} 20 [get_ports altera_reserved_tdi]
set_input_delay -clock {altera_reserved_tck} 20 [get_ports altera_reserved_tms]
set_output_delay -clock {altera_reserved_tck} 20 [get_ports altera_reserved_tdo]
# ----------------------------------------------------------------------------------

# ----------------------------------------------------------------------------------
# derive pll clocks (generated + input)
derive_pll_clocks -create_base_clocks
derive_clock_uncertainty
# ----------------------------------------------------------------------------------

# ----------------------------------------------------------------------------------
# create virtual clocks
## used by SRAM
create_generated_clock -source $clkSRAM -name CLKSRAM_virt

# cut reset input
#set_false_path -from [get_ports RESET_n] -to [get_registers *]
# ----------------------------------------------------------------------------------

# ----------------------------------------------------------------------------------
# sram (IS61WV102416BLL-10TLI)
## SRAM is driven by 100 MHz fsm.
## Note: Qsys inserts 1 write and 2 read cycles, thus, the SRAM writes 
##       with 100 MHz and read with 50 MHz cycles.
set sram_clkRd      50.0
set sram_clkWr      100.0
set sram_tperRd     [expr 1000.0 / $sram_clkRd]
set sram_tperWr     [expr 1000.0 / $sram_clkWr]
## delay Address Access Time (tAA) = 10.0 ns
set sram_ddel       10.0
## pcb delay
set sram_tpcb       0.1
## fpga settings...
set sram_tco        7.5
set sram_tsu        [expr $sram_tperRd - $sram_ddel - $sram_tco - 2*$sram_tpcb]
set sram_th         0.0
set sram_tcom       0.0

set sram_in_max     [expr $sram_tperRd - $sram_tsu]
set sram_in_min     $sram_th
set sram_out_max    [expr $sram_tperWr - $sram_tco]
set sram_out_min    $sram_tcom

## TSU / TH
set_input_delay -clock CLKSRAM_virt -max $sram_in_max [get_ports SRAM_DQ[*]]
set_input_delay -clock CLKSRAM_virt -min $sram_in_min [get_ports SRAM_DQ[*]]
## TCO
set_output_delay -clock CLKSRAM_virt -max $sram_out_max [get_ports SRAM_DQ[*]]
set_output_delay -clock CLKSRAM_virt -min $sram_out_min [get_ports SRAM_DQ[*]]
## TCO
set_output_delay -clock CLKSRAM_virt -max $sram_out_max [get_ports SRAM_ADDR[*]]
set_output_delay -clock CLKSRAM_virt -min $sram_out_min [get_ports SRAM_ADDR[*]]
## TCO
set_output_delay -clock CLKSRAM_virt -max $sram_out_max [get_ports SRAM_BE_n[*]]
set_output_delay -clock CLKSRAM_virt -min $sram_out_min [get_ports SRAM_BE_n[*]]
## TCO
set_output_delay -clock CLKSRAM_virt -max $sram_out_max [get_ports SRAM_OE_n]
set_output_delay -clock CLKSRAM_virt -min $sram_out_min [get_ports SRAM_OE_n]
## TCO
set_output_delay -clock CLKSRAM_virt -max $sram_out_max [get_ports SRAM_WE_n]
set_output_delay -clock CLKSRAM_virt -min $sram_out_min [get_ports SRAM_WE_n]
## TCO
set_output_delay -clock CLKSRAM_virt -max $sram_out_max [get_ports SRAM_CE_n]
set_output_delay -clock CLKSRAM_virt -min $sram_out_min [get_ports SRAM_CE_n]

## relax timing...
## Note: Nios II is running with 100 MHz, but Tri-State-bridge reads with 50 MHz.
### from FPGA to SRAM
#set_multicycle_path -from [get_clocks $clkSRAM] -to [get_clocks CLKSRAM_virt] -setup -start 2
#set_multicycle_path -from [get_clocks $clkSRAM] -to [get_clocks CLKSRAM_virt] -hold -start 1
### from SRAM to FPGA
set_multicycle_path -from [get_clocks CLKSRAM_virt] -to [get_clocks $clkSRAM] -setup -end 2
set_multicycle_path -from [get_clocks CLKSRAM_virt] -to [get_clocks $clkSRAM] -hold -end 1
# ----------------------------------------------------------------------------------

# ----------------------------------------------------------------------------------
# Set clock groups (cut paths)
set_clock_groups -asynchronous  \
                                            -group $clk50 \
                                            -group $clk25 \
                                            -group [format "%s %s" $clk100 CLKSRAM_virt] \
                                            -group $ext_clk

# ----------------------------------------------------------------------------------

# ----------------------------------------------------------------------------------
# IOs
## cut paths
###EPCS
set_false_path -from [get_registers *] -to [get_ports EPCS_DCLK]
set_false_path -from [get_registers *] -to [get_ports EPCS_SCE]
set_false_path -from [get_registers *] -to [get_ports EPCS_SDO]
set_false_path -from [get_ports EPCS_DATA0] -to [get_registers *]
###IOs
set_false_path -from [get_ports NODE_SWITCH[*]] -to [get_registers *]
set_false_path -from [get_registers *] -to [get_ports BENCHMARK[*]]
#### example for output: set_false_path -from [get_registers *] -to [get_ports LED[*]]
#### example for input:  set_false_path -from [get_ports BUTTON[*]] -to [get_registers *]
#############################################################
# add here your slow IOs...
set_false_path -from [get_ports KEY[*]] -to [get_registers *]
set_false_path -from [get_registers *] -to [get_ports LEDG[*]]
set_false_path -from [get_registers *] -to [get_ports LEDR[*]]
set_false_path -from [get_registers *] -to [get_ports HEX?[*]]
set_false_path -from [get_registers *] -to [get_ports LCD_*]
set_false_path -from [get_registers *] -to [get_ports LCD_DQ[*]]
set_false_path -from [get_ports LCD_DQ[*]] -to [get_registers *]
#############################################################
# ----------------------------------------------------------------------------------